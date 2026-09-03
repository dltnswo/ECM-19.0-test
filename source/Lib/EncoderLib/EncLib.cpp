/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     EncLib.cpp
    \brief    encoder class
*/
#include "EncLib.h"

#include "EncModeCtrl.h"
#include "AQp.h"
#include "EncCu.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/ChromaFormat.h"
#if ENABLE_SPLIT_PARALLELISM
#include <omp.h>
#endif
#include "EncLibCommon.h"
#include "CommonLib/ProfileLevelTier.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncLib::EncLib( EncLibCommon* encLibCommon )
  : m_cListPic( encLibCommon->getPictureBuffer() )
#if JVET_AK0065_TALF
   , m_cEncALF( encLibCommon->getApsIdStart(), encLibCommon->getApsIdStart2() )
#else
  , m_cEncALF( encLibCommon->getApsIdStart() )
#endif
  , m_spsMap( encLibCommon->getSpsMap() )
  , m_ppsMap( encLibCommon->getPpsMap() )
  , m_apsMap( encLibCommon->getApsMap() )
  , m_AUWriterIf( nullptr )
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_lmcsAPS(nullptr)
  , m_scalinglistAPS( nullptr )
  , m_doPlt( true )
  , m_vps( encLibCommon->getVPS() )
{
  m_iPOCLast          = -1;
  m_iNumPicRcvd       =  0;
  m_uiNumAllPicCoded  =  0;

  m_iMaxRefPicNum     = 0;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  g_pelBufOP.initPelBufOpsX86();
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif

  memset(m_apss, 0, sizeof(m_apss));

#if JVET_AK0065_TALF
  memset(m_apss2, 0, sizeof(m_apss2));
#endif
  m_layerId = NOT_VALID;
  m_picIdInGOP = NOT_VALID;
}

EncLib::~EncLib()
{
}

void EncLib::create( const int layerId )
{
  m_layerId = layerId;
  m_iPOCLast = m_compositeRefEnabled ? -2 : -1;
  // create processing unit classes
  m_cGOPEncoder.        create( );
#if JVET_AJ0237_INTERNAL_12BIT
  m_cGOPEncoder.m_cBilateralFilter.setInternalBitDepth(m_bitDepth[COMPONENT_Y]);
#endif
#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_SPLIT_PARALLELISM
  m_numCuEncStacks  = m_numSplitThreads == 1 ? 1 : NUM_RESERVERD_SPLIT_JOBS;
#else
  m_numCuEncStacks  = 1;
#endif

  m_cCuEncoder      = new EncCu              [m_numCuEncStacks];
  m_cInterSearch    = new InterSearch        [m_numCuEncStacks];
  m_cIntraSearch    = new IntraSearch        [m_numCuEncStacks];
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter = new BilateralFilter    [m_numCuEncStacks];
#endif
  m_cTrQuant        = new TrQuant            [m_numCuEncStacks];
  m_CABACEncoder    = new CABACEncoder       [m_numCuEncStacks];
  m_cRdCost         = new RdCost             [m_numCuEncStacks];
  m_ctxCache        = new CtxCache           [m_numCuEncStacks];

  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].         create( this );
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
    m_bilateralFilter[jId].    create();
#endif

  }
#else
  m_cCuEncoder.         create( this );
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter.    create();
#if JVET_AJ0237_INTERNAL_12BIT
  m_bilateralFilter.setInternalBitDepth(m_bitDepth[COMPONENT_Y]);
#endif
#endif
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cInterSearch.cacheAssign( &m_cacheModel );
#endif

  m_cLoopFilter.create(floorLog2(m_maxCUWidth) - MIN_CU_LOG2);

  if (!m_bLoopFilterDisable && m_encDbOpt)
  {
    m_cLoopFilter.initEncPicYuvBuffer(m_chromaFormatIDC, Size(getSourceWidth(), getSourceHeight()), getMaxCUWidth());
  }

#if ENABLE_SPLIT_PARALLELISM
  m_cReshaper = new EncReshape[m_numCuEncStacks];
#endif
  if (m_lmcsEnabled)
  {
#if ENABLE_SPLIT_PARALLELISM
    for (int jId = 0; jId < m_numCuEncStacks; jId++)
    {
      m_cReshaper[jId].createEnc(getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
    }
#else
    m_cReshaper.createEnc( getSourceWidth(), getSourceHeight(), m_maxCUWidth, m_maxCUHeight, m_bitDepth[COMPONENT_Y]);
#endif
  }
  if ( m_RCEnableRateControl )
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    m_cRateCtrl.init(m_framesToBeEncoded, m_RCTargetBitrate, (int)((double)m_iFrameRate / m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_sourceWidth, m_sourceHeight,
      m_maxCUWidth, m_maxCUHeight, getBitDepth(CHANNEL_TYPE_LUMA), m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList);
#else
    m_cRateCtrl.init(m_framesToBeEncoded, m_RCTargetBitrate, (int)((double)m_iFrameRate / m_temporalSubsampleRatio + 0.5), m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
      m_maxCUWidth, m_maxCUHeight, getBitDepth(CHANNEL_TYPE_LUMA), m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList);
#endif
  }

  if (m_alf)
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    m_cEncALF.create(this, m_sourceWidth, m_sourceHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, m_bitDepth, m_inputBitDepth);
#else
    m_cEncALF.create(this, m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, m_bitDepth, m_inputBitDepth);
#endif
  }
#if JVET_V0094_BILATERAL_FILTER
#if JVET_W0066_CCSAO
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (m_bUseSAO || m_BIF || m_CCSAO || m_chromaBIF)
#else
  if (m_bUseSAO || m_BIF || m_CCSAO)
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (m_bUseSAO || m_BIF || m_chromaBIF)
#else
  if (m_bUseSAO || m_BIF)
#endif
#endif
#else
#if JVET_W0066_CCSAO
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (m_bUseSAO || m_CCSAO || m_chromaBIF)
#else
  if (m_bUseSAO || m_CCSAO)
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (m_bUseSAO || m_chromaBIF)
#else
  if (m_bUseSAO)
#endif
#endif
#endif
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    const uint32_t widthInCtus = (m_sourceWidth + m_maxCUWidth - 1) / m_maxCUWidth;
    const uint32_t heightInCtus = (m_sourceHeight + m_maxCUHeight - 1) / m_maxCUHeight;
    const uint32_t numCtuInFrame = widthInCtus * heightInCtus;
    m_cEncSAO.create(m_sourceWidth, m_sourceHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, (uint32_t)std::max(0, m_bitDepth[CHANNEL_TYPE_LUMA] - MAX_SAO_TRUNCATED_BITDEPTH), (uint32_t)std::max(0, m_bitDepth[CHANNEL_TYPE_CHROMA] - MAX_SAO_TRUNCATED_BITDEPTH));
#else
    const uint32_t widthInCtus = (m_iSourceWidth + m_maxCUWidth - 1) / m_maxCUWidth;
    const uint32_t heightInCtus = (m_iSourceHeight + m_maxCUHeight - 1) / m_maxCUHeight;
    const uint32_t numCtuInFrame = widthInCtus * heightInCtus;
    m_cEncSAO.create(m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, (uint32_t)std::max(0, m_bitDepth[CHANNEL_TYPE_LUMA] - MAX_SAO_TRUNCATED_BITDEPTH), (uint32_t)std::max(0, m_bitDepth[CHANNEL_TYPE_CHROMA] - MAX_SAO_TRUNCATED_BITDEPTH));
#endif
    m_cEncSAO.createEncData(m_saoCtuBoundary, numCtuInFrame);
#if JVET_AJ0237_INTERNAL_12BIT
    m_cEncSAO.m_bilateralFilter.setInternalBitDepth(m_bitDepth[COMPONENT_Y]);
#endif
  }
}

void EncLib::destroy ()
{
  // destroy processing unit classes
  m_cGOPEncoder.        destroy();
  m_cSliceEncoder.      destroy();
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cCuEncoder[jId].destroy();
  }
#else
  m_cCuEncoder.         destroy();
#endif
  if( m_alf )
  {
    m_cEncALF.destroy();
  }
  m_cEncSAO.            destroy();
  m_cLoopFilter.        destroy();
  m_cRateCtrl.          destroy();
#if ENABLE_SPLIT_PARALLELISM
  for (int jId = 0; jId < m_numCuEncStacks; jId++)
  {
    m_cReshaper[jId].   destroy();
  }
#else
  m_cReshaper.          destroy();
#endif
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cInterSearch[jId].   destroy();
    m_cIntraSearch[jId].   destroy();
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
    m_bilateralFilter[jId].destroy();
#endif
  }
#else
  m_cInterSearch.       destroy();
  m_cIntraSearch.       destroy();
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter.    destroy();
#endif
#endif

#if ENABLE_SPLIT_PARALLELISM
  delete[] m_cCuEncoder;
  delete[] m_cInterSearch;
  delete[] m_cIntraSearch;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  delete[] m_bilateralFilter;
#endif
  delete[] m_cTrQuant;
  delete[] m_CABACEncoder;
  delete[] m_cRdCost;
  delete[] m_ctxCache;
#endif

  return;
}

void EncLib::init( bool isFieldCoding, AUWriterIf* auWriterIf )
{
  m_AUWriterIf = auWriterIf;

  SPS &sps0 = *(m_spsMap.allocatePS( m_vps->getGeneralLayerIdx( m_layerId ) )); // NOTE: implementations that use more than 1 SPS need to be aware of activation issues.
  PPS &pps0 = *( m_ppsMap.allocatePS( m_vps->getGeneralLayerIdx( m_layerId ) ) );
  APS &aps0 = *( m_apsMap.allocatePS( SCALING_LIST_APS ) );
  aps0.setAPSId( 0 );
  aps0.setAPSType( SCALING_LIST_APS );

  if (getAvoidIntraInDepLayer() && getNumRefLayers(m_vps->getGeneralLayerIdx( getLayerId())) > 0)
  {
    setIDRRefParamListPresent(true);
  }
  // initialize SPS
  xInitSPS( sps0 );
  xInitVPS( sps0 );

  xInitDCI(m_dci, sps0);
#if ENABLE_SPLIT_PARALLELISM
  if( omp_get_dynamic() )
  {
    omp_set_dynamic( false );
  }
  omp_set_nested( true );
#endif

  if (getUseCompositeRef() || getDependentRAPIndicationSEIEnabled())
  {
    sps0.setLongTermRefsPresent(true);
  }

#if U0132_TARGET_BITS_SATURATION
  if (m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(sps0.getGeneralHrdParameters(), sps0.getOlsHrdParameters(), m_iFrameRate, m_RCInitialCpbFullness);
  }
#endif
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    m_cRdCost[jId].setCostMode ( m_costMode );
  }
#else
  m_cRdCost.setCostMode ( m_costMode );
#endif

  // initialize PPS
#if JVET_AA0146_WRAP_AROUND_FIX
  pps0.setPicWidthInLumaSamples( m_sourceWidth );
  pps0.setPicHeightInLumaSamples( m_sourceHeight );
#else
  pps0.setPicWidthInLumaSamples( m_iSourceWidth );
  pps0.setPicHeightInLumaSamples( m_iSourceHeight );
#endif
#if JVET_R0068_ASPECT6_ENC_RESTRICTION
  if (pps0.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps0.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
  {
    pps0.setConformanceWindow( sps0.getConformanceWindow() );
    pps0.setConformanceWindowFlag( false );
  }
  else
  {
    pps0.setConformanceWindow( m_conformanceWindow );
    pps0.setConformanceWindowFlag( m_conformanceWindow.getWindowEnabledFlag() );
  }
#else
  pps0.setConformanceWindow( m_conformanceWindow );
#endif
  xInitPPS(pps0, sps0);
  // initialize APS
  xInitRPL(sps0, isFieldCoding);
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  if (sps0.getUseAML())
  {
    sps0.setNumLambda(m_numQPOffset);
    int maxBits = 0;
    for (int idx = 0; idx < m_numQPOffset; idx++)
    {
      sps0.setQPOffsets(idx, m_qpOffsetList[idx]);
      const uint32_t lambda = (uint32_t)LAMBDA_DEC_SIDE[min(max(26 + pps0.getPicInitQPMinus26() + m_qpOffsetList[idx] - 4 * ((int)m_isRA), 0), MAX_QP)];
      sps0.setLambdaVal(idx, lambda);
      for (int shift = 0; shift < 16; shift++)
        if (lambda >> shift == 0)
        {
          if (shift > maxBits)
          {
            maxBits = shift;
          }
          break;
        }
    }
    sps0.setMaxbitsLambdaVal(maxBits);
  }
#endif

  if (m_resChangeInClvsEnabled)
  {
    PPS &pps = *( m_ppsMap.allocatePS( ENC_PPS_ID_RPR ) );
    Window& inputScalingWindow = pps0.getScalingWindow();
    int scaledWidth = int( ( pps0.getPicWidthInLumaSamples() - SPS::getWinUnitX( sps0.getChromaFormatIdc() ) * ( inputScalingWindow.getWindowLeftOffset() + inputScalingWindow.getWindowRightOffset() ) ) / m_scalingRatioHor );
    int minSizeUnit = std::max(8, 1 << sps0.getLog2MinCodingBlockSize());
    int temp = scaledWidth / minSizeUnit;
    int width = ( scaledWidth - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

    int scaledHeight = int( ( pps0.getPicHeightInLumaSamples() - SPS::getWinUnitY( sps0.getChromaFormatIdc() ) * ( inputScalingWindow.getWindowTopOffset() + inputScalingWindow.getWindowBottomOffset() ) ) / m_scalingRatioVer );
    temp = scaledHeight / minSizeUnit;
    int height = ( scaledHeight - ( temp * minSizeUnit) > 0 ? temp + 1 : temp ) * minSizeUnit;

    pps.setPicWidthInLumaSamples( width );
    pps.setPicHeightInLumaSamples( height );
#if JVET_AC0096
    pps.setSliceChromaQpFlag(true);
#endif
    Window conformanceWindow;
    conformanceWindow.setWindow( 0, ( width - scaledWidth ) / SPS::getWinUnitX( sps0.getChromaFormatIdc() ), 0, ( height - scaledHeight ) / SPS::getWinUnitY( sps0.getChromaFormatIdc() ) );
#if JVET_R0068_ASPECT6_ENC_RESTRICTION
    if (pps.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
    {
      pps.setConformanceWindow( sps0.getConformanceWindow() );
      pps.setConformanceWindowFlag( false );
    }
    else
    {
      pps.setConformanceWindow( conformanceWindow );
      pps.setConformanceWindowFlag( pps.getConformanceWindow().getWindowEnabledFlag() );
    }
#else
    pps.setConformanceWindow( conformanceWindow );
#endif

    Window scalingWindow;
    scalingWindow.setWindow( 0, ( width - scaledWidth ) / SPS::getWinUnitX( sps0.getChromaFormatIdc() ), 0, ( height - scaledHeight ) / SPS::getWinUnitY( sps0.getChromaFormatIdc() ) );
    pps.setScalingWindow( scalingWindow );

    //register the width/height of the current pic into reference SPS
    if (!sps0.getPPSValidFlag(pps.getPPSId()))
    {
      sps0.setPPSValidFlag(pps.getPPSId(), true);
      sps0.setScalingWindowSizeInPPS(pps.getPPSId(), scaledWidth, scaledHeight);
    }
    int curSeqMaxPicWidthY = sps0.getMaxPicWidthInLumaSamples();    // pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.getMaxPicHeightInLumaSamples();  // pic_height_max_in_luma_samples
    int curPicWidthY = width;                                       // pic_width_in_luma_samples
    int curPicHeightY = height;                                     // pic_height_in_luma_samples
    int max8MinCbSizeY = std::max((int)8, (1 << sps0.getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)
    //Warning message of potential scaling window size violation
    for (int i = 0; i < 64; i++)
    {
      if (sps0.getPPSValidFlag(i))
      {
        if ((scaledWidth * curSeqMaxPicWidthY) < sps0.getScalingWindowSizeInPPS(i).width * (curPicWidthY - max8MinCbSizeY))
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        if ((scaledHeight * curSeqMaxPicHeightY) < sps0.getScalingWindowSizeInPPS(i).height * (curPicHeightY - max8MinCbSizeY))
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original resolution)
    m_noPicPartitionFlag = true;

    xInitPPS( pps, sps0 ); // will allocate memory for and initialize pps.pcv inside
    
    if( pps.getWrapAroundEnabledFlag() )
    {
      int minCbSizeY = (1 << sps0.getLog2MinCodingBlockSize());
      pps.setPicWidthMinusWrapAroundOffset      ((pps.getPicWidthInLumaSamples()/minCbSizeY) - (m_wrapAroundOffset * pps.getPicWidthInLumaSamples() / pps0.getPicWidthInLumaSamples() / minCbSizeY) );
      pps.setWrapAroundOffset                   (minCbSizeY * (pps.getPicWidthInLumaSamples() / minCbSizeY - pps.getPicWidthMinusWrapAroundOffset()));

    }
    else 
    {
      pps.setPicWidthMinusWrapAroundOffset      (0);
      pps.setWrapAroundOffset                   ( 0 );       
    }
  }
#if JVET_AC0096
#if JVET_AG0116
  if (m_resChangeInClvsEnabled && (m_rprFunctionalityTestingEnabledFlag || m_gopBasedRPREnabledFlag))
#else
  if (m_resChangeInClvsEnabled && m_rprFunctionalityTestingEnabledFlag)
#endif
  {
    // allocate PPS that can be used
    double scalingRatioHor = m_scalingRatioHor2;
    double scalingRatioVer = m_scalingRatioVer2;
    PPS& pps = *(m_ppsMap.allocatePS(ENC_PPS_ID_RPR2));
    Window& inputScalingWindow = pps0.getScalingWindow();
    int scaledWidth = int((pps0.getPicWidthInLumaSamples() - SPS::getWinUnitX(sps0.getChromaFormatIdc()) * (inputScalingWindow.getWindowLeftOffset() + inputScalingWindow.getWindowRightOffset())) / scalingRatioHor);
    int minSizeUnit = std::max(8, 1 << sps0.getLog2MinCodingBlockSize());
    int temp = scaledWidth / minSizeUnit;
    int width = (scaledWidth - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    int scaledHeight = int((pps0.getPicHeightInLumaSamples() - SPS::getWinUnitY(sps0.getChromaFormatIdc()) * (inputScalingWindow.getWindowTopOffset() + inputScalingWindow.getWindowBottomOffset())) / scalingRatioVer);
    temp = scaledHeight / minSizeUnit;
    int height = (scaledHeight - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    pps.setPicWidthInLumaSamples(width);
    pps.setPicHeightInLumaSamples(height);
    pps.setSliceChromaQpFlag(true);
    Window conformanceWindow;
    conformanceWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.getChromaFormatIdc()), 0, (height - scaledHeight) / SPS::getWinUnitY(sps0.getChromaFormatIdc()));
#if JVET_R0068_ASPECT6_ENC_RESTRICTION
    if (pps.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
    {
      pps.setConformanceWindow(sps0.getConformanceWindow());
      pps.setConformanceWindowFlag(false);
    }
    else
    {
      pps.setConformanceWindow(conformanceWindow);
      pps.setConformanceWindowFlag(pps.getConformanceWindow().getWindowEnabledFlag());
    }
#else
    pps.setConformanceWindow(conformanceWindow);
#endif

    Window scalingWindow;
    scalingWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.getChromaFormatIdc()), 0, (height - scaledHeight) / SPS::getWinUnitY(sps0.getChromaFormatIdc()));
    pps.setScalingWindow(scalingWindow);

    //register the width/height of the current pic into reference SPS
    if (!sps0.getPPSValidFlag(pps.getPPSId()))
    {
      sps0.setPPSValidFlag(pps.getPPSId(), true);
      sps0.setScalingWindowSizeInPPS(pps.getPPSId(), scaledWidth, scaledHeight);
    }
    int curSeqMaxPicWidthY = sps0.getMaxPicWidthInLumaSamples();    // pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.getMaxPicHeightInLumaSamples();  // pic_height_max_in_luma_samples
    int curPicWidthY = width;                                       // pic_width_in_luma_samples
    int curPicHeightY = height;                                     // pic_height_in_luma_samples
    int max8MinCbSizeY = std::max((int)8, (1 << sps0.getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)
    //Warning message of potential scaling window size violation
    for (int i = 0; i < 64; i++)
    {
      if (sps0.getPPSValidFlag(i))
      {
        if ((scaledWidth * curSeqMaxPicWidthY) < sps0.getScalingWindowSizeInPPS(i).width * (curPicWidthY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        }
        if ((scaledHeight * curSeqMaxPicHeightY) < sps0.getScalingWindowSizeInPPS(i).height * (curPicHeightY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
        }
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original resolution)
    m_noPicPartitionFlag = true;

    xInitPPS(pps, sps0); // will allocate memory for and initialize pps.pcv inside

    if (pps.getWrapAroundEnabledFlag())
    {
      int minCbSizeY = (1 << sps0.getLog2MinCodingBlockSize());
      pps.setPicWidthMinusWrapAroundOffset((pps.getPicWidthInLumaSamples() / minCbSizeY) - (m_wrapAroundOffset * pps.getPicWidthInLumaSamples() / pps0.getPicWidthInLumaSamples() / minCbSizeY));
      pps.setWrapAroundOffset(minCbSizeY * (pps.getPicWidthInLumaSamples() / minCbSizeY - pps.getPicWidthMinusWrapAroundOffset()));

    }
    else
    {
      pps.setPicWidthMinusWrapAroundOffset(0);
      pps.setWrapAroundOffset(0);
    }
  }
#if JVET_AG0116
  if (m_resChangeInClvsEnabled && (m_rprFunctionalityTestingEnabledFlag || m_gopBasedRPREnabledFlag))
#else
  if (m_resChangeInClvsEnabled && m_rprFunctionalityTestingEnabledFlag)
#endif
  {
    // allocate PPS that can be used
    double scalingRatioHor = m_scalingRatioHor3;
    double scalingRatioVer = m_scalingRatioVer3;
    PPS& pps = *(m_ppsMap.allocatePS(ENC_PPS_ID_RPR3));
    Window& inputScalingWindow = pps0.getScalingWindow();
    int scaledWidth = int((pps0.getPicWidthInLumaSamples() - SPS::getWinUnitX(sps0.getChromaFormatIdc()) * (inputScalingWindow.getWindowLeftOffset() + inputScalingWindow.getWindowRightOffset())) / scalingRatioHor);
    int minSizeUnit = std::max(8, 1 << sps0.getLog2MinCodingBlockSize());
    int temp = scaledWidth / minSizeUnit;
    int width = (scaledWidth - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    int scaledHeight = int((pps0.getPicHeightInLumaSamples() - SPS::getWinUnitY(sps0.getChromaFormatIdc()) * (inputScalingWindow.getWindowTopOffset() + inputScalingWindow.getWindowBottomOffset())) / scalingRatioVer);
    temp = scaledHeight / minSizeUnit;
    int height = (scaledHeight - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    pps.setPicWidthInLumaSamples(width);
    pps.setPicHeightInLumaSamples(height);
    pps.setSliceChromaQpFlag(true);

    Window conformanceWindow;
    conformanceWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.getChromaFormatIdc()), 0, (height - scaledHeight) / SPS::getWinUnitY(sps0.getChromaFormatIdc()));
#if JVET_R0068_ASPECT6_ENC_RESTRICTION
    if (pps.getPicWidthInLumaSamples() == sps0.getMaxPicWidthInLumaSamples() && pps.getPicHeightInLumaSamples() == sps0.getMaxPicHeightInLumaSamples())
    {
      pps.setConformanceWindow(sps0.getConformanceWindow());
      pps.setConformanceWindowFlag(false);
    }
    else
    {
      pps.setConformanceWindow(conformanceWindow);
      pps.setConformanceWindowFlag(pps.getConformanceWindow().getWindowEnabledFlag());
    }
#else
    pps.setConformanceWindow(conformanceWindow);
#endif

    Window scalingWindow;
    scalingWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.getChromaFormatIdc()), 0, (height - scaledHeight) / SPS::getWinUnitY(sps0.getChromaFormatIdc()));
    pps.setScalingWindow(scalingWindow);

    //register the width/height of the current pic into reference SPS
    if (!sps0.getPPSValidFlag(pps.getPPSId()))
    {
      sps0.setPPSValidFlag(pps.getPPSId(), true);
      sps0.setScalingWindowSizeInPPS(pps.getPPSId(), scaledWidth, scaledHeight);
    }
    int curSeqMaxPicWidthY = sps0.getMaxPicWidthInLumaSamples();    // pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.getMaxPicHeightInLumaSamples();  // pic_height_max_in_luma_samples
    int curPicWidthY = width;                                       // pic_width_in_luma_samples
    int curPicHeightY = height;                                     // pic_height_in_luma_samples
    int max8MinCbSizeY = std::max((int)8, (1 << sps0.getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)
    //Warning message of potential scaling window size violation
    for (int i = 0; i < 64; i++)
    {
      if (sps0.getPPSValidFlag(i))
      {
        if ((scaledWidth * curSeqMaxPicWidthY) < sps0.getScalingWindowSizeInPPS(i).width * (curPicWidthY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        }
        if ((scaledHeight * curSeqMaxPicHeightY) < sps0.getScalingWindowSizeInPPS(i).height * (curPicHeightY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
        }
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original resolution)
    m_noPicPartitionFlag = true;

    xInitPPS(pps, sps0); // will allocate memory for and initialize pps.pcv inside

    if (pps.getWrapAroundEnabledFlag())
    {
      int minCbSizeY = (1 << sps0.getLog2MinCodingBlockSize());
      pps.setPicWidthMinusWrapAroundOffset((pps.getPicWidthInLumaSamples() / minCbSizeY) - (m_wrapAroundOffset * pps.getPicWidthInLumaSamples() / pps0.getPicWidthInLumaSamples() / minCbSizeY));
      pps.setWrapAroundOffset(minCbSizeY * (pps.getPicWidthInLumaSamples() / minCbSizeY - pps.getPicWidthMinusWrapAroundOffset()));

    }
    else
    {
      pps.setPicWidthMinusWrapAroundOffset(0);
      pps.setWrapAroundOffset(0);
    }
  }
#endif
#if ER_CHROMA_QP_WCG_PPS
  if (m_wcgChromaQpControl.isEnabled())
  {
    PPS &pps1=*(m_ppsMap.allocatePS(1));
    xInitPPS(pps1, sps0);
  }
#endif
  if (getUseCompositeRef())
  {
    PPS &pps2 = *(m_ppsMap.allocatePS(2));
    xInitPPS(pps2, sps0);
    xInitPPSforLT(pps2);
  }
  xInitPicHeader(m_picHeader, sps0, pps0);

  // initialize processing unit classes
  m_cGOPEncoder.  init( this );
  m_cSliceEncoder.init( this, sps0 );
#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 0; jId < m_numCuEncStacks; jId++ )
  {
    // precache a few objects
    for( int i = 0; i < 10; i++ )
    {
      auto x = m_ctxCache[jId].get();
      m_ctxCache[jId].cache( x );
    }

    m_cCuEncoder[jId].init( this, sps0, jId );

    // initialize transform & quantization class
    m_cTrQuant[jId].init( jId == 0 ? nullptr : m_cTrQuant[0].getQuant(),
                          1 << m_log2MaxTbSize,

                          m_useRDOQ,
                          m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                          m_useSelectiveRDOQ,
#endif
                          true
    );

    // initialize encoder search class
    CABACWriter* cabacEstimator = m_CABACEncoder[jId].getCABACEstimator( &sps0 );
    m_cIntraSearch[jId].init( this,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                             &m_bilateralFilter[jId],
#endif
                              &m_cTrQuant[jId],
                              &m_cRdCost[jId],
                              cabacEstimator,
                              getCtxCache( jId ), m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize
                            , &m_cReshaper[jId]
                            , sps0.getBitDepth(CHANNEL_TYPE_LUMA)
    );
    m_cInterSearch[jId].init( this,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                             &m_bilateralFilter[jId],
#endif
                              &m_cTrQuant[jId],
                              m_iSearchRange,
                              m_bipredSearchRange,
                              m_motionEstimationSearchMethod,
                              getUseCompositeRef(),
                              m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, &m_cRdCost[jId], cabacEstimator, getCtxCache( jId )
                           , &m_cReshaper[jId]
    );

    // link temporary buffets from intra search with inter search to avoid unnecessary memory overhead
    m_cInterSearch[jId].setTempBuffers( m_cIntraSearch[jId].getSplitCSBuf(), m_cIntraSearch[jId].getFullCSBuf(), m_cIntraSearch[jId].getSaveCSBuf() );
  }
#else  // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  m_cCuEncoder.   init( this, sps0 );

  // initialize transform & quantization class
  m_cTrQuant.init( nullptr,
                   1 << m_log2MaxTbSize,
                   m_useRDOQ,
                   m_useRDOQTS,
#if T0196_SELECTIVE_RDOQ
                   m_useSelectiveRDOQ,
#endif
                   true
  );

  // initialize encoder search class
  CABACWriter* cabacEstimator = m_CABACEncoder.getCABACEstimator(&sps0);
  m_cIntraSearch.init( this,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                       &m_bilateralFilter,
#endif
                       &m_cTrQuant,
                       &m_cRdCost,
                       cabacEstimator,
                       getCtxCache(), m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize
                     , &m_cReshaper
                     , sps0.getBitDepth(CHANNEL_TYPE_LUMA)
  );
  m_cInterSearch.init( this,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                       &m_bilateralFilter,
#endif
                       &m_cTrQuant,
                       m_iSearchRange,
                       m_bipredSearchRange,
                       m_motionEstimationSearchMethod,
                       getUseCompositeRef(),
    m_maxCUWidth, m_maxCUHeight, floorLog2(m_maxCUWidth) - m_log2MinCUSize, &m_cRdCost, cabacEstimator, getCtxCache()
                     , &m_cReshaper
#if JVET_Z0153_IBC_EXT_REF
                     , pps0.getPicWidthInLumaSamples()
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
                     , pps0.getPicHeightInLumaSamples()
#endif
#endif
  );

  // link temporary buffets from intra search with inter search to avoid unneccessary memory overhead
  m_cInterSearch.setTempBuffers( m_cIntraSearch.getSplitCSBuf(), m_cIntraSearch.getFullCSBuf(), m_cIntraSearch.getSaveCSBuf() );
#endif // ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

#if JVET_AE0159_FIBC || JVET_AE0059_INTER_CCCM || JVET_AE0078_IBC_LIC_EXTENSION || JVET_AF0073_INTER_CCP_MERGE
  m_cInterSearch.setIntraPrediction(&m_cIntraSearch);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  m_cIntraSearch.setInterPrediction(&m_cInterSearch);
#endif
  m_iMaxRefPicNum = 0;

#if ER_CHROMA_QP_WCG_PPS
  if( m_wcgChromaQpControl.isEnabled() )
  {
    xInitScalingLists( sps0, *m_apsMap.getPS( 1 ) );
    xInitScalingLists( sps0, aps0 );
  }
  else
#endif
  {
    xInitScalingLists( sps0, aps0 );
  }

  if (m_resChangeInClvsEnabled)
  {
    xInitScalingLists( sps0, *m_apsMap.getPS( ENC_PPS_ID_RPR ) );
  }

  if (getUseCompositeRef())
  {
    Picture *picBg = new Picture;

    picBg->create(
      sps0.getRprEnabledFlag(),
#if JVET_Z0118_GDR
      sps0.getGDREnabledFlag(),
#endif
#if NN_LF_UNIFIED
      sps0.getNnlfEnabledFlag(),
#endif
      sps0.getWrapAroundEnabledFlag(), sps0.getChromaFormatIdc(),
      Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(),
      sps0.getMaxCUWidth() + EXT_PICTURE_SIZE, false, m_layerId, getGopBasedTemporalFilterEnabled());

    picBg->getRecoBuf().fill(0);

#if JVET_AK0065_TALF
    picBg->finalInit( m_vps, sps0, pps0, &m_picHeader, m_apss, m_apss2, m_lmcsAPS, m_scalinglistAPS );
#else
    picBg->finalInit( m_vps, sps0, pps0, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
#endif


    picBg->allocateNewSlice();
    picBg->createSpliceIdx(pps0.pcv->sizeInCtus);
    m_cGOPEncoder.setPicBg(picBg);
    Picture *picOrig = new Picture;

    picOrig->create(
      sps0.getRprEnabledFlag(),
#if JVET_Z0118_GDR
      sps0.getGDREnabledFlag(),
#endif
#if NN_LF_UNIFIED
      sps0.getNnlfEnabledFlag(),
#endif
      sps0.getWrapAroundEnabledFlag(), sps0.getChromaFormatIdc(),
      Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples()), sps0.getMaxCUWidth(),
      sps0.getMaxCUWidth() + EXT_PICTURE_SIZE, false, m_layerId, getGopBasedTemporalFilterEnabled());

    picOrig->getOrigBuf().fill(0);
    m_cGOPEncoder.setPicOrig(picOrig);
  }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  if (sps0.getALFEnabledFlag())
  {
    int picWidth = pps0.getPicWidthInLumaSamples();
    int picHeight = pps0.getPicHeightInLumaSamples();
    int numCTUsInWidth = (picWidth / m_maxCUWidth) + ((picWidth % m_maxCUWidth) ? 1 : 0);
    int numCTUsInHeight = (picHeight / m_maxCUHeight) + ((picHeight % m_maxCUHeight) ? 1 : 0);
    int numCTUsInPic = numCTUsInWidth * numCTUsInHeight;
    for (int aps = 0; aps < ALF_CTB_MAX_NUM_APS; aps++)
    {
      g_pictureControlInfo[aps].resize(numCTUsInPic);
    }
  }
#endif
}

void EncLib::xInitScalingLists( SPS &sps, APS &aps )
{
  // Initialise scaling lists
  // The encoder will only use the SPS scaling lists. The PPS will never be marked present.
  const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE] =
  {
    sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_LUMA),
    sps.getMaxLog2TrDynamicRange(CHANNEL_TYPE_CHROMA)
  };

  Quant* quant = getTrQuant()->getQuant();

  if(getUseScalingListId() == SCALING_LIST_OFF)
  {
    quant->setFlatScalingList(maxLog2TrDynamicRange, sps.getBitDepths());
    quant->setUseScalingList(false);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setFlatScalingList( maxLog2TrDynamicRange, sps.getBitDepths() );
      getTrQuant( jId )->getQuant()->setUseScalingList( false );
    }
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_DEFAULT)
  {
    aps.getScalingList().setDefaultScalingList ();
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
    sps.setDisableScalingMatrixForLfnstBlks(getDisableScalingMatrixForLfnstBlks());
#endif
  }
  else if(getUseScalingListId() == SCALING_LIST_FILE_READ)
  {
    aps.getScalingList().setDefaultScalingList();
    CHECK( aps.getScalingList().xParseScalingList( getScalingListFileName() ), "Error Parsing Scaling List Input File" );
    aps.getScalingList().checkDcOfMatrix();
    if( aps.getScalingList().isNotDefaultScalingList() == false )
    {
      setUseScalingListId( SCALING_LIST_DEFAULT );
    }
    aps.getScalingList().setChromaScalingListPresentFlag((sps.getChromaFormatIdc()!=CHROMA_400));
    quant->setScalingList( &( aps.getScalingList() ), maxLog2TrDynamicRange, sps.getBitDepths() );
    quant->setUseScalingList(true);
#if ENABLE_SPLIT_PARALLELISM
    for( int jId = 1; jId < m_numCuEncStacks; jId++ )
    {
      getTrQuant( jId )->getQuant()->setUseScalingList( true );
    }
#endif

    sps.setDisableScalingMatrixForLfnstBlks(getDisableScalingMatrixForLfnstBlks());
  }
  else
  {
    THROW("error : ScalingList == " << getUseScalingListId() << " not supported\n");
  }

  if( getUseScalingListId() == SCALING_LIST_FILE_READ )
  {
    // Prepare delta's:
    for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
    {
      if (aps.getScalingList().getChromaScalingListPresentFlag()||aps.getScalingList().isLumaScalingList(scalingListId))
      {
        aps.getScalingList().checkPredMode(scalingListId);
      }
    }
  }
}

void EncLib::xInitPPSforLT(PPS& pps)
{
  pps.setOutputFlagPresentFlag(true);
  pps.setDeblockingFilterControlPresentFlag(true);
  pps.setPPSDeblockingFilterDisabledFlag(true);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncLib::deletePicBuffer()
{
  PicList::iterator iterPic = m_cListPic.begin();
  int iSize = int( m_cListPic.size() );

  for ( int i = 0; i < iSize; i++ )
  {
    Picture* pcPic = *(iterPic++);

    pcPic->destroy();

    // get rid of the qpadaption layer
    while( pcPic->aqlayer.size() )
    {
      delete pcPic->aqlayer.back(); pcPic->aqlayer.pop_back();
    }

    delete pcPic;
    pcPic = NULL;
  }

  m_cListPic.clear();
}

bool EncLib::encodePrep(bool flush, PelStorage* pcPicYuvOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded
#if JVET_AG0116
    , PelStorage** ppcPicYuvRPR
#endif
)
{
  if( m_compositeRefEnabled && m_cGOPEncoder.getPicBg()->getSpliceFull() && m_iPOCLast >= 10 && m_iNumPicRcvd == 0 && m_cGOPEncoder.getEncodedLTRef() == false )
  {
    Picture* picCurr = NULL;
    xGetNewPicBuffer( rcListPicYuvRecOut, picCurr, 2 );
    const PPS *pps = m_ppsMap.getPS( 2 );
    const SPS *sps = m_spsMap.getPS( pps->getSPSId() );

    picCurr->M_BUFS( 0, PIC_ORIGINAL ).copyFrom( m_cGOPEncoder.getPicBg()->getRecoBuf() );

#if JVET_AK0065_TALF
    picCurr->finalInit(m_vps, *sps, *pps, &m_picHeader, m_apss, m_apss2, m_lmcsAPS, m_scalinglistAPS);
#else
    picCurr->finalInit( m_vps, *sps, *pps, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
#endif

    picCurr->poc = m_iPOCLast - 1;
    m_iPOCLast -= 2;
    if( getUseAdaptiveQP() )
    {
      AQpPreanalyzer::preanalyze( picCurr );
    }
    if( m_RCEnableRateControl )
    {
      m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }

    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, false, false, snrCSC,
                              m_printFrameMSE,
#if MSSIM_UNIFORM_METRICS_LOG
                              m_printMSSSIM,
#endif
                              true, 0);

#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif
    m_cGOPEncoder.setEncodedLTRef( true );
    if( m_RCEnableRateControl )
    {
      m_cRateCtrl.destroyRCGOP();
    }

    iNumEncoded = 0;
    m_iNumPicRcvd = 0;
  }

  //PROF_ACCUM_AND_START_NEW_SET( getProfilerPic(), P_GOP_LEVEL );
  if( pcPicYuvOrg != NULL )
  {
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
    CHECK(m_horCollocatedChromaFlag < 0, "HorCollocatedChroma error");
#endif
    // get original YUV
    Picture* pcPicCurr = NULL;

    int ppsID = -1; // Use default PPS ID
#if ER_CHROMA_QP_WCG_PPS
    if( getWCGChromaQPControl().isEnabled() )
    {
      ppsID = getdQPs()[m_iPOCLast / ( m_compositeRefEnabled ? 2 : 1 ) + 1];
      ppsID += ( getSwitchPOC() != -1 && ( m_iPOCLast + 1 >= getSwitchPOC() ) ? 1 : 0 );
    }
#endif

#if !RPR_ENABLE
    if( m_resChangeInClvsEnabled && m_intraPeriod == -1 )
#endif
    {
      const int poc = m_iPOCLast + ( m_compositeRefEnabled ? 2 : 1 );

#if RPR_ENABLE
#if JVET_AC0096
      if (!(m_resChangeInClvsEnabled && m_rprFunctionalityTestingEnabledFlag))
      {
        ppsID = 0;
      }
#else
      ppsID = 0;
#endif
      bool  bApplyRpr = false;
#if JVET_AC0096
      if (m_resChangeInClvsEnabled && m_rprFunctionalityTestingEnabledFlag)
      {
        if (poc % m_rprSwitchingSegmentSize == 0)
        {
          int currPoc = poc + m_FrameSkip;
          int rprSegment = getRprSwitchingSegment(currPoc);
          ppsID =  getRprSwitchingPPSID(rprSegment);
          m_gopRprPpsId = ppsID;
        }
        else
        {
          ppsID = m_gopRprPpsId;
        }
      }
#if JVET_AG0116
      else if (m_resChangeInClvsEnabled && m_gopBasedRPREnabledFlag && (m_iQP >= getGOPBasedRPRQPThreshold()))
      {
        double upscaledPSNR = 0.0;
        if (poc % getGOPSize() == 0)
        {
          int xScale = 32768;
          int yScale = 32768;
          std::pair<int, int> downScalingRatio = std::pair<int, int>(xScale, yScale);
          xScale = 8192;
          yScale = 8192;
          std::pair<int, int> upScalingRatio = std::pair<int, int>(xScale, yScale);

          const PPS* orgPPS = m_ppsMap.getPS(0);
          const SPS* orgSPS = m_spsMap.getPS(orgPPS->getSPSId());
          const ChromaFormat chFormatIdc = orgSPS->getChromaFormatIdc();

          const PPS* pTempPPS = m_ppsMap.getPS(ENC_PPS_ID_RPR);
          Picture::rescalePicture(downScalingRatio, *pcPicYuvOrg, orgPPS->getScalingWindow(), *ppcPicYuvRPR[1], pTempPPS->getScalingWindow(), chFormatIdc, orgSPS->getBitDepths(), true, true,
            orgSPS->getHorCollocatedChromaFlag(), orgSPS->getVerCollocatedChromaFlag());
          Picture::rescalePicture(upScalingRatio, *ppcPicYuvRPR[1], orgPPS->getScalingWindow(), *ppcPicYuvRPR[0], pTempPPS->getScalingWindow(), chFormatIdc, orgSPS->getBitDepths(), true, false,
            orgSPS->getHorCollocatedChromaFlag(), orgSPS->getVerCollocatedChromaFlag());
          // Calculate PSNR
          const  Pel* pSrc0 = pcPicYuvOrg->get(COMPONENT_Y).bufAt(0, 0);
          const  Pel* pSrc1 = ppcPicYuvRPR[0]->get(COMPONENT_Y).bufAt(0, 0);

          uint64_t totalDiff = 0;
          for (int y = 0; y < pcPicYuvOrg->get(COMPONENT_Y).height; y++)
          {
            for (int x = 0; x < pcPicYuvOrg->get(COMPONENT_Y).width; x++)
            {
              int diff = pSrc0[x] - pSrc1[x];
              totalDiff += uint64_t(diff) * uint64_t(diff);
            }
            pSrc0 += pcPicYuvOrg->get(COMPONENT_Y).stride;
            pSrc1 += ppcPicYuvRPR[0]->get(COMPONENT_Y).stride;
          }

          const uint32_t maxval = 255 << (orgSPS->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
          upscaledPSNR = totalDiff ? 10.0 * log10((double)maxval * maxval * orgPPS->getPicWidthInLumaSamples() * orgPPS->getPicHeightInLumaSamples() / (double)totalDiff) : 999.99;
        }

        if (poc % getGOPSize() == 0)
        {
          const int qpBias = 37;
          if ((m_psnrThresholdRPR - (m_iQP - qpBias) * 0.5) < upscaledPSNR)
          {
            ppsID = ENC_PPS_ID_RPR;
          }
          else
          {
            if ((m_psnrThresholdRPR2 - (m_iQP - qpBias) * 0.5) < upscaledPSNR)
            {
              ppsID = ENC_PPS_ID_RPR2;
            }
            else
            {
              if ((m_psnrThresholdRPR3 - (m_iQP - qpBias) * 0.5) < upscaledPSNR)
              {
                ppsID = ENC_PPS_ID_RPR3;
              }
              else
              {
                ppsID = 0;
              }
            }
          }
          m_gopRprPpsId = ppsID;
        }
        else
        {
          ppsID = m_gopRprPpsId;
        }

      }
#endif
      else
      {
        bApplyRpr |= (m_switchPocPeriod < 0);                                   // RPR applied for all pictures
        bApplyRpr |= (m_switchPocPeriod > 0) && (poc / m_switchPocPeriod % 2);  // RPR applied for periods RA or LDB
      }
#else
      bApplyRpr |= (m_switchPocPeriod < 0);                                   // RPR applied for all pictures
      bApplyRpr |= (m_switchPocPeriod > 0) && (poc / m_switchPocPeriod % 2);  // RPR applied for periods RA or LDB
#endif
      if( bApplyRpr )
#else
      if( poc / m_switchPocPeriod % 2 )
#endif
      {
        ppsID = ENC_PPS_ID_RPR;
      }
      else
      {
#if JVET_AC0096
#if JVET_AG0116
        if (!(m_resChangeInClvsEnabled && (m_rprFunctionalityTestingEnabledFlag || (m_gopBasedRPREnabledFlag && (m_iQP >= getGOPBasedRPRQPThreshold())))))
#else
        if (!(m_resChangeInClvsEnabled && m_rprFunctionalityTestingEnabledFlag))
#endif
        {
          ppsID = 0;
        }
#else
        ppsID = 0;
#endif
      }
    }

    if( m_vps->getMaxLayers() > 1 )
    {
      ppsID = m_vps->getGeneralLayerIdx( m_layerId );
    }

    xGetNewPicBuffer( rcListPicYuvRecOut, pcPicCurr, ppsID );

    const PPS *pPPS = ( ppsID < 0 ) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS( ppsID );
    const SPS *pSPS = m_spsMap.getPS( pPPS->getSPSId() );

    if (m_resChangeInClvsEnabled)
    {
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Y ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Y ) );
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cb ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cb ) );
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL_INPUT ).getBuf( COMPONENT_Cr ).copyFrom( pcPicYuvOrg->getBuf( COMPONENT_Cr ) );

      const ChromaFormat chromaFormatIDC = pSPS->getChromaFormatIdc();

      const PPS *refPPS = m_ppsMap.getPS( 0 );
      const Window& curScalingWindow = pPPS->getScalingWindow();
      int curPicWidth = pPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( pSPS->getChromaFormatIdc() ) * ( curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset() );
      int curPicHeight = pPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( pSPS->getChromaFormatIdc() ) * ( curScalingWindow.getWindowTopOffset()  + curScalingWindow.getWindowBottomOffset() );

      const Window& refScalingWindow = refPPS->getScalingWindow();
      int refPicWidth = refPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( pSPS->getChromaFormatIdc() ) * ( refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset() );
      int refPicHeight = refPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( pSPS->getChromaFormatIdc() ) * ( refScalingWindow.getWindowTopOffset()  + refScalingWindow.getWindowBottomOffset() );

      int xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
      int yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;
      std::pair<int, int> scalingRatio = std::pair<int, int>( xScale, yScale );

      Picture::rescalePicture( scalingRatio, *pcPicYuvOrg, refPPS->getScalingWindow(), pcPicCurr->getOrigBuf(), pPPS->getScalingWindow(), chromaFormatIDC, pSPS->getBitDepths(), true, true,
        pSPS->getHorCollocatedChromaFlag(), pSPS->getVerCollocatedChromaFlag() );
    }
    else
    {
      pcPicCurr->M_BUFS( 0, PIC_ORIGINAL ).swap( *pcPicYuvOrg );
    }

#if JVET_AK0065_TALF
    pcPicCurr->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_apss2, m_lmcsAPS, m_scalinglistAPS );
#else
    pcPicCurr->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
#endif

    pcPicCurr->poc = m_iPOCLast;

    // compute image characteristics
    if( getUseAdaptiveQP() )
    {
      AQpPreanalyzer::preanalyze( pcPicCurr );
    }
  }

  if( ( m_iNumPicRcvd == 0 ) || ( !flush && ( m_iPOCLast != 0 ) && ( m_iNumPicRcvd != m_iGOPSize ) && ( m_iGOPSize != 0 ) ) )
  {
    iNumEncoded = 0;
    return true;
  }

  if( m_RCEnableRateControl )
  {
    m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
  }

  m_picIdInGOP = 0;

  return false;
}

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \param   pcPicYuvTrueOrg
 \param   snrCSC
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  accessUnitsOut      list of output access units
 \retval  iNumEncoded         number of encoded pictures
 */

bool EncLib::encode( const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded )
{
  // compress GOP
  m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, false, false, snrCSC,
                            m_printFrameMSE,
#if MSSIM_UNIFORM_METRICS_LOG
                            m_printMSSSIM,
#endif
                            false, m_picIdInGOP);

  m_picIdInGOP++;

  // go over all pictures in a GOP excluding the first IRAP
  if( m_picIdInGOP != m_iGOPSize && m_iPOCLast )
  {
    return true;
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

  if( m_RCEnableRateControl )
  {
    m_cRateCtrl.destroyRCGOP();
  }

  iNumEncoded = m_iNumPicRcvd;
  m_iNumPicRcvd = 0;
  m_uiNumAllPicCoded += iNumEncoded;

  return false;
}

#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
void EncLib::setQPOffsetList(const int QPOffset[MAX_GOP])
{
  std::memcpy(m_qpOffsetList, QPOffset,(MAX_GOP) * sizeof(int));
}
#endif

/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
void separateFields(Pel* org, Pel* dstField, uint32_t stride, uint32_t width, uint32_t height, bool isTop)
{
  if (!isTop)
  {
    org += stride;
  }
  for (int y = 0; y < height>>1; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstField[x] = org[x];
    }

    dstField += stride;
    org += stride*2;
  }

}

bool EncLib::encodePrep(bool flush, PelStorage* pcPicYuvOrg, const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut,
  int& iNumEncoded, bool isTff)
{
  iNumEncoded = 0;
  bool keepDoing = true;

  for( int fieldNum = 0; fieldNum < 2; fieldNum++ )
  {
    if( pcPicYuvOrg )
    {
      /* -- field initialization -- */
      const bool isTopField = isTff == ( fieldNum == 0 );

      Picture *pcField;
      xGetNewPicBuffer( rcListPicYuvRecOut, pcField, -1 );

      for( uint32_t comp = 0; comp < ::getNumberValidComponents( pcPicYuvOrg->chromaFormat ); comp++ )
      {
        const ComponentID compID = ComponentID( comp );
        {
          PelBuf compBuf = pcPicYuvOrg->get( compID );
          separateFields( compBuf.buf,
            pcField->getOrigBuf().get( compID ).buf,
            compBuf.stride,
            compBuf.width,
            compBuf.height,
            isTopField );
        }
      }

      int ppsID = -1; // Use default PPS ID
      const PPS *pPPS = ( ppsID < 0 ) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS( ppsID );
      const SPS *pSPS = m_spsMap.getPS( pPPS->getSPSId() );

#if JVET_AK0065_TALF
      pcField->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_apss2, m_lmcsAPS, m_scalinglistAPS );
#else
      pcField->finalInit( m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS );
#endif

      pcField->poc = m_iPOCLast;
      pcField->reconstructed = false;

      pcField->setBorderExtension( false );// where is this normally?
#if JVET_AK0085_TM_BOUNDARY_PADDING
      pcField->setUseTMBP(true);
#endif

      pcField->topField = isTopField;                  // interlaced requirement

      // compute image characteristics
      if( getUseAdaptiveQP() )
      {
        AQpPreanalyzer::preanalyze( pcField );
      }
    }

  }

  if( m_iNumPicRcvd && ( flush || m_iPOCLast == 1 || m_iNumPicRcvd == m_iGOPSize ) )
  {
    m_picIdInGOP = 0;
    keepDoing = false;
  }

  return keepDoing;
}

bool EncLib::encode( const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf*>& rcListPicYuvRecOut, int& iNumEncoded, bool isTff )
{
  iNumEncoded = 0;

  for( int fieldNum = 0; fieldNum < 2; fieldNum++ )
  {
    m_iPOCLast = m_iPOCLast < 2 ? fieldNum : m_iPOCLast;

    // compress GOP
    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iPOCLast < 2 ? m_iPOCLast + 1 : m_iNumPicRcvd, m_cListPic,
                              rcListPicYuvRecOut, true, isTff, snrCSC, m_printFrameMSE,
#if MSSIM_UNIFORM_METRICS_LOG
                              m_printMSSSIM,
#endif
                              false, m_picIdInGOP);
#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

    m_picIdInGOP++;
  }

  // go over all pictures in a GOP excluding first top field and first bottom field
  if( m_picIdInGOP != m_iGOPSize && m_iPOCLast > 1 )
  {
    return true;
  }

  iNumEncoded += m_iNumPicRcvd;
  m_uiNumAllPicCoded += m_iNumPicRcvd;
  m_iNumPicRcvd = 0;

  return false;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
void EncLib::xGetNewPicBuffer ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, int ppsId )
{
  // rotate the output buffer
  rcListPicYuvRecOut.push_back( rcListPicYuvRecOut.front() ); rcListPicYuvRecOut.pop_front();

  rpcPic=0;

  // At this point, the SPS and PPS can be considered activated - they are copied to the new Pic.
  const PPS *pPPS=(ppsId<0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsId);
  CHECK(!(pPPS!=0), "Unspecified error");
  const PPS &pps=*pPPS;

  const SPS *pSPS=m_spsMap.getPS(pps.getSPSId());
  CHECK(!(pSPS!=0), "Unspecified error");
  const SPS &sps=*pSPS;

  Slice::sortPicList(m_cListPic);

  // use an entry in the buffered list if the maximum number that need buffering has been reached:
  int maxDecPicBuffering = ( m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1 ) ? sps.getMaxDecPicBuffering( MAX_TLAYER - 1 ) : m_vps->getMaxDecPicBuffering( MAX_TLAYER - 1 );

  if( m_cListPic.size() >= (uint32_t)( m_iGOPSize + maxDecPicBuffering + 2 ) )
  {
    PicList::iterator iterPic = m_cListPic.begin();
    int iSize = int( m_cListPic.size() );
    for( int i = 0; i < iSize; i++ )
    {
      rpcPic = *iterPic;
      if( !rpcPic->referenced && rpcPic->layerId == m_layerId )
      {
        break;
      }
      else
      {
        rpcPic = nullptr;
      }
      iterPic++;
    }

    // If PPS ID is the same, we will assume that it has not changed since it was last used
    // and return the old object.
    if( rpcPic && pps.getPPSId() != rpcPic->cs->pps->getPPSId() )
    {
      // the IDs differ - free up an entry in the list, and then create a new one, as with the case where the max buffering state has not been reached.
      rpcPic->destroy();
      delete rpcPic;
      m_cListPic.erase(iterPic);
      rpcPic=0;
    }
  }

  if (rpcPic==0)
  {
    rpcPic = new Picture;

    rpcPic->create(
      isRprEnabled(),
#if JVET_Z0118_GDR
      getGdrEnabled(),
#endif
#if NN_LF_UNIFIED
      getUseNnlfUnified(),
#endif
      sps.getWrapAroundEnabledFlag(), sps.getChromaFormatIdc(),
      Size(pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples()), sps.getMaxCUWidth(),
      sps.getMaxCUWidth() + EXT_PICTURE_SIZE, false, m_layerId, getGopBasedTemporalFilterEnabled());


    if (m_resChangeInClvsEnabled)
    {
      const PPS &pps0 = *m_ppsMap.getPS(0);
      rpcPic->M_BUFS(0, PIC_ORIGINAL_INPUT).create(sps.getChromaFormatIdc(), Area(Position(), Size(pps0.getPicWidthInLumaSamples(), pps0.getPicHeightInLumaSamples())));
    }
    if ( getUseAdaptiveQP() )
    {
      const uint32_t iMaxDQPLayer = m_picHeader.getCuQpDeltaSubdivIntra()/2+1;
      rpcPic->aqlayer.resize( iMaxDQPLayer );
      for (uint32_t d = 0; d < iMaxDQPLayer; d++)
      {
        rpcPic->aqlayer[d] = new AQpLayer( pps.getPicWidthInLumaSamples(), pps.getPicHeightInLumaSamples(), sps.getMaxCUWidth() >> d, sps.getMaxCUHeight() >> d );
      }
    }

    m_cListPic.push_back( rpcPic );
  }

  rpcPic->setBorderExtension( false );
#if JVET_AK0085_TM_BOUNDARY_PADDING
  rpcPic->setUseTMBP(true);
#endif
  rpcPic->reconstructed = false;
  rpcPic->referenced = true;
  rpcPic->getHashMap()->clearAll();

  m_iPOCLast += (m_compositeRefEnabled ? 2 : 1);
  m_iNumPicRcvd++;
}

void EncLib::xInitVPS( const SPS& sps )
{
  // The SPS must have already been set up.
  // set the VPS profile information.

  m_vps->m_olsHrdParams.clear();
  m_vps->m_olsHrdParams.resize(m_vps->getNumOlsHrdParamsMinus1(), std::vector<OlsHrdParams>(m_vps->getMaxSubLayers()));
  ProfileLevelTierFeatures profileLevelTierFeatures;
  profileLevelTierFeatures.extractPTLInformation( sps );

  m_vps->deriveOutputLayerSets();
  m_vps->deriveTargetOutputLayerSet( m_vps->m_targetOlsIdx );

  // number of the DPB parameters is set equal to the number of OLS containing multi layers
  if( !m_vps->getEachLayerIsAnOlsFlag() )
  {
    m_vps->m_numDpbParams = m_vps->getNumMultiLayeredOlss();
  }

  if( m_vps->m_dpbParameters.size() != m_vps->m_numDpbParams )
  {
    m_vps->m_dpbParameters.resize( m_vps->m_numDpbParams );
  }

  if( m_vps->m_dpbMaxTemporalId.size() != m_vps->m_numDpbParams )
  {
    m_vps->m_dpbMaxTemporalId.resize( m_vps->m_numDpbParams );
  }

  for( int olsIdx = 0, dpbIdx = 0; olsIdx < m_vps->m_numOutputLayersInOls.size(); olsIdx++ )
  {
    if ( m_vps->getNumLayersInOls(olsIdx) > 1 )
    { 
      if( std::find( m_vps->m_layerIdInOls[olsIdx].begin(), m_vps->m_layerIdInOls[olsIdx].end(), m_layerId ) != m_vps->m_layerIdInOls[olsIdx].end() )
      {
        m_vps->setOlsDpbPicWidth( olsIdx, std::max<int>( sps.getMaxPicWidthInLumaSamples(), m_vps->getOlsDpbPicSize( olsIdx ).width ) );
        m_vps->setOlsDpbPicHeight( olsIdx, std::max<int>( sps.getMaxPicHeightInLumaSamples(), m_vps->getOlsDpbPicSize( olsIdx ).height ) );
        m_vps->setOlsDpbChromaFormatIdc( olsIdx, std::max<int>(sps.getChromaFormatIdc(), m_vps->getOlsDpbChromaFormatIdc( olsIdx )));
        m_vps->setOlsDpbBitDepthMinus8( olsIdx, std::max<int>(sps.getBitDepth(CHANNEL_TYPE_LUMA) - 8, m_vps->getOlsDpbBitDepthMinus8( olsIdx )));
      }

      m_vps->setOlsDpbParamsIdx( olsIdx, dpbIdx );
      dpbIdx++;
    }
  }

  //for( int i = 0; i < m_vps->m_numDpbParams; i++ )
  for( int i = 0; i < m_vps->m_numOutputLayersInOls.size(); i++ )
  {
    if ( m_vps->getNumLayersInOls(i) > 1 )
    { 
      int dpbIdx = m_vps->getOlsDpbParamsIdx( i );

      if( m_vps->getMaxSubLayers() == 1 )
      {
        // When vps_max_sublayers_minus1 is equal to 0, the value of dpb_max_temporal_id[ dpbIdx ] is inferred to be equal to 0.
        m_vps->m_dpbMaxTemporalId[dpbIdx] = 0;
      }
      else
      {
        if( m_vps->getAllLayersSameNumSublayersFlag() )
        {
          // When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1, the value of dpb_max_temporal_id[ dpbIdx ] is inferred to be equal to vps_max_sublayers_minus1.
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->getMaxSubLayers() - 1;
        }
        else
        {
#if JVET_S0100_ASPECT3
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->getMaxSubLayers() - 1;
#else
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_maxTempLayer;
#endif
        }
      }
    
      for( int j = ( m_vps->m_sublayerDpbParamsPresentFlag ? 0 : m_vps->m_dpbMaxTemporalId[dpbIdx] ); j <= m_vps->m_dpbMaxTemporalId[dpbIdx]; j++ )
      {
        m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] = profileLevelTierFeatures.getMaxDpbSize( m_vps->getOlsDpbPicSize( i ).width * m_vps->getOlsDpbPicSize( i ).height );
        m_vps->m_dpbParameters[dpbIdx].m_numReorderPics[j] = m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j];
        m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[j] = 0;
      }

      for( int j = ( m_vps->m_sublayerDpbParamsPresentFlag ? m_vps->m_dpbMaxTemporalId[dpbIdx] : 0 ); j < m_vps->m_dpbMaxTemporalId[dpbIdx]; j++ )
      {
        // When max_dec_pic_buffering_minus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[j] = m_vps->m_dpbParameters[dpbIdx].m_maxDecPicBuffering[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When max_num_reorder_pics[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_num_reorder_pics[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_numReorderPics[j] = m_vps->m_dpbParameters[dpbIdx].m_numReorderPics[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When max_latency_increase_plus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_latency_increase_plus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[j] = m_vps->m_dpbParameters[dpbIdx].m_maxLatencyIncreasePlus1[m_vps->m_dpbMaxTemporalId[dpbIdx]];
      }
    }
  }
#if JVET_S0100_ASPECT3
  for (int i = 0; i < m_vps->getNumOutputLayerSets(); i++)
  {
    m_vps->setHrdMaxTid(i, m_vps->getMaxSubLayers() - 1);
  }
#endif

  if (m_cfgVPSParameters.m_maxTidILRefPicsPlus1 >= 0)
  {
    for (int i = 0; i < m_vps->getMaxLayers(); i++)
    {
      m_vps->setMaxTidIlRefPicsPlus1(i, m_cfgVPSParameters.m_maxTidILRefPicsPlus1);
    }
  }
#if JVET_S0100_ASPECT3
  m_vps->checkVPS();
#endif
}

void EncLib::xInitDCI(DCI& dci, const SPS& sps)
{
  dci.setMaxSubLayersMinus1(sps.getMaxTLayers() - 1);
  std::vector<ProfileTierLevel> ptls;
  ptls.resize(1);
  ptls[0] = *sps.getProfileTierLevel();
  dci.setProfileTierLevel(ptls);
}

void EncLib::xInitSPS( SPS& sps )
{
  ProfileTierLevel* profileTierLevel = sps.getProfileTierLevel();
  ConstraintInfo* cinfo = profileTierLevel->getConstraintInfo();

#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
  cinfo->setGciPresentFlag(m_gciPresentFlag);
#endif
#if !JVET_S0266_VUI_length
  cinfo->setNonPackedConstraintFlag     (m_nonPackedConstraintFlag);
  cinfo->setNonProjectedConstraintFlag(m_nonProjectedConstraintFlag);
#endif
#if JVET_Q0114_ASPECT5_GCI_FLAG
  cinfo->setNoRprConstraintFlag(m_noRprConstraintFlag);
#endif
  cinfo->setNoResChangeInClvsConstraintFlag(m_noResChangeInClvsConstraintFlag);
  cinfo->setOneTilePerPicConstraintFlag(m_oneTilePerPicConstraintFlag);
  cinfo->setPicHeaderInSliceHeaderConstraintFlag(m_picHeaderInSliceHeaderConstraintFlag);
  cinfo->setOneSlicePerPicConstraintFlag(m_oneSlicePerPicConstraintFlag);
#if JVET_S0113_S0195_GCI
  cinfo->setNoIdrRplConstraintFlag(m_noIdrRplConstraintFlag);
  cinfo->setNoRectSliceConstraintFlag(m_noRectSliceConstraintFlag);
  cinfo->setOneSlicePerSubpicConstraintFlag(m_oneSlicePerSubpicConstraintFlag);
  cinfo->setNoSubpicInfoConstraintFlag(m_noSubpicInfoConstraintFlag);
#else
  cinfo->setOneSubpicPerPicConstraintFlag(m_oneSubpicPerPicConstraintFlag);
#endif
#if !JVET_S0138_GCI_PTL
  cinfo->setFrameOnlyConstraintFlag     (m_frameOnlyConstraintFlag);
#endif
  cinfo->setOnePictureOnlyConstraintFlag(m_onePictureOnlyConstraintFlag);
  cinfo->setIntraOnlyConstraintFlag         (m_intraOnlyConstraintFlag);
  cinfo->setMaxBitDepthConstraintIdc    (m_maxBitDepthConstraintIdc);
  cinfo->setMaxChromaFormatConstraintIdc((int)m_maxChromaFormatConstraintIdc);
#if !JVET_S0138_GCI_PTL
  cinfo->setSingleLayerConstraintFlag (m_singleLayerConstraintFlag);
#endif
  cinfo->setAllLayersIndependentConstraintFlag (m_allLayersIndependentConstraintFlag);
  cinfo->setNoMrlConstraintFlag (m_noMrlConstraintFlag);
  cinfo->setNoIspConstraintFlag (m_noIspConstraintFlag);
  cinfo->setNoMipConstraintFlag (m_noMipConstraintFlag);
  cinfo->setNoLfnstConstraintFlag (m_noLfnstConstraintFlag);
  cinfo->setNoMmvdConstraintFlag (m_noMmvdConstraintFlag);
  cinfo->setNoSmvdConstraintFlag (m_noSmvdConstraintFlag);
  cinfo->setNoProfConstraintFlag (m_noProfConstraintFlag);
  cinfo->setNoPaletteConstraintFlag (m_noPaletteConstraintFlag);
  cinfo->setNoActConstraintFlag (m_noActConstraintFlag);
  cinfo->setNoLmcsConstraintFlag (m_noLmcsConstraintFlag);
#if JVET_S0050_GCI
  cinfo->setNoExplicitScaleListConstraintFlag(m_noExplicitScaleListConstraintFlag);
  cinfo->setNoVirtualBoundaryConstraintFlag(m_noVirtualBoundaryConstraintFlag);
#endif
#if JVET_S0058_GCI
  cinfo->setNoMttConstraintFlag(m_noMttConstraintFlag);
#endif
#if JVET_R0341_GCI
  cinfo->setNoChromaQpOffsetConstraintFlag(m_noChromaQpOffsetConstraintFlag);
#endif
  cinfo->setNoQtbttDualTreeIntraConstraintFlag(m_noQtbttDualTreeIntraConstraintFlag);
  cinfo->setNoPartitionConstraintsOverrideConstraintFlag(m_noPartitionConstraintsOverrideConstraintFlag);
  cinfo->setNoSaoConstraintFlag(m_noSaoConstraintFlag);
#if JVET_W0066_CCSAO
  cinfo->setNoCCSaoConstraintFlag(m_noCCSaoConstraintFlag);
#endif
  cinfo->setNoAlfConstraintFlag(m_noAlfConstraintFlag);
  cinfo->setNoCCAlfConstraintFlag(m_noCCAlfConstraintFlag);
#if JVET_S0058_GCI
  cinfo->setNoWeightedPredictionConstraintFlag(m_noWeightedPredictionConstraintFlag);
#endif
  cinfo->setNoRefWraparoundConstraintFlag(m_noRefWraparoundConstraintFlag);
  cinfo->setNoTemporalMvpConstraintFlag(m_noTemporalMvpConstraintFlag);
  cinfo->setNoSbtmvpConstraintFlag(m_noSbtmvpConstraintFlag);
  cinfo->setNoAmvrConstraintFlag(m_noAmvrConstraintFlag);
  cinfo->setNoBdofConstraintFlag(m_noBdofConstraintFlag);
  cinfo->setNoDmvrConstraintFlag(m_noDmvrConstraintFlag);
  cinfo->setNoCclmConstraintFlag(m_noCclmConstraintFlag);
  cinfo->setNoMtsConstraintFlag(m_noMtsConstraintFlag);
  cinfo->setNoSbtConstraintFlag(m_noSbtConstraintFlag);
  cinfo->setNoAffineMotionConstraintFlag(m_noAffineMotionConstraintFlag);
  cinfo->setNoBcwConstraintFlag(m_noBcwConstraintFlag);
  cinfo->setNoIbcConstraintFlag(m_noIbcConstraintFlag);
#if ENABLE_DIMD
  cinfo->setNoDimdConstraintFlag(m_noDimdConstraintFlag);
#endif
#if JVET_W0123_TIMD_FUSION
  cinfo->setNoTimdConstraintFlag(m_noTimdConstraintFlag);
#endif
#if JVET_AB0155_SGPM
  cinfo->setNoSgpmConstraintFlag(m_noSgpmConstraintFlag);
#endif
#if JVET_AD0082_TMRL_CONFIG
  cinfo->setNoTmrlConstraintFlag(m_noTmrlConstraintFlag);
#endif
#if JVET_AG0058_EIP
  cinfo->setNoEipConstraintFlag(m_noEipConstraintFlag);
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  cinfo->setNoIntraPredBfConstraintFlag(m_noIntraPredBfConstraintFlag);
#endif
#if ENABLE_OBMC
  cinfo->setNoObmcConstraintFlag(m_noObmcConstraintFlag);
#endif
  cinfo->setNoCiipConstraintFlag(m_noCiipConstraintFlag);
  cinfo->setNoGeoConstraintFlag(m_noGeoConstraintFlag);
  cinfo->setNoLadfConstraintFlag(m_noLadfConstraintFlag);
  cinfo->setNoTransformSkipConstraintFlag(m_noTransformSkipConstraintFlag);
  cinfo->setNoBDPCMConstraintFlag(m_noBDPCMConstraintFlag);
  cinfo->setNoJointCbCrConstraintFlag(m_noJointCbCrConstraintFlag);
  cinfo->setNoQpDeltaConstraintFlag(m_noQpDeltaConstraintFlag);
  cinfo->setNoDepQuantConstraintFlag(m_noDepQuantConstraintFlag);
  cinfo->setNoSignDataHidingConstraintFlag(m_noSignDataHidingConstraintFlag);
  cinfo->setNoTrailConstraintFlag(m_noTrailConstraintFlag);
  cinfo->setNoStsaConstraintFlag(m_noStsaConstraintFlag);
  cinfo->setNoRaslConstraintFlag(m_noRaslConstraintFlag);
  cinfo->setNoRadlConstraintFlag(m_noRadlConstraintFlag);
  cinfo->setNoIdrConstraintFlag(m_noIdrConstraintFlag);
  cinfo->setNoCraConstraintFlag(m_noCraConstraintFlag);
  cinfo->setNoGdrConstraintFlag(m_noGdrConstraintFlag);
  cinfo->setNoApsConstraintFlag(m_noApsConstraintFlag);

  profileTierLevel->setLevelIdc                    (m_level);
  profileTierLevel->setTierFlag                    (m_levelTier);
  profileTierLevel->setProfileIdc                  (m_profile);
#if JVET_S0138_GCI_PTL
  profileTierLevel->setFrameOnlyConstraintFlag     (m_frameOnlyConstraintFlag);
  profileTierLevel->setMultiLayerEnabledFlag       (m_multiLayerEnabledFlag);
#endif
  profileTierLevel->setNumSubProfile(m_numSubProfile);
  for (int k = 0; k < m_numSubProfile; k++)
  {
    profileTierLevel->setSubProfileIdc(k, m_subProfile[k]);
  }
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */
  sps.setVPSId( m_vps->getVPSId() );
#if JVET_Z0118_GDR
  if (getGdrEnabled())
  {
    sps.setGDREnabledFlag(true);
  }
  else
  {
    sps.setGDREnabledFlag(false);
  }
#else
  sps.setGDREnabledFlag(false);
#endif

#if JVET_AA0146_WRAP_AROUND_FIX
  sps.setMaxPicWidthInLumaSamples( m_sourceWidth );
  sps.setMaxPicHeightInLumaSamples( m_sourceHeight );
#else
  sps.setMaxPicWidthInLumaSamples( m_iSourceWidth );
  sps.setMaxPicHeightInLumaSamples( m_iSourceHeight );
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  sps.setUseInterSliceSeparateTree ( m_interSliceSeparateTreeEnabled );
  if (getFrameRate() < 50 && (m_sourceWidth * m_sourceHeight) <= (832 * 480))
  {
    sps.setUseInterSliceSeparateTree ( false );
  }
  if(m_iQP < 27 || m_iQP>32)
  {
    sps.setUseInterSliceSeparateTree ( false );
  }
#endif

  if (m_resChangeInClvsEnabled)
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    int maxPicWidth = std::max(m_sourceWidth, (int)((double)m_sourceWidth / m_scalingRatioHor + 0.5));
    int maxPicHeight = std::max(m_sourceHeight, (int)((double)m_sourceHeight / m_scalingRatioVer + 0.5));
#else
    int maxPicWidth = std::max(m_iSourceWidth, (int)((double)m_iSourceWidth / m_scalingRatioHor + 0.5));
    int maxPicHeight = std::max(m_iSourceHeight, (int)((double)m_iSourceHeight / m_scalingRatioVer + 0.5));
#endif
#if JVET_AC0096
#if JVET_AG0116
    if (m_rprFunctionalityTestingEnabledFlag || m_gopBasedRPREnabledFlag)
#else
    if (m_rprFunctionalityTestingEnabledFlag)
#endif
    {
      maxPicWidth = std::max(maxPicWidth, (int)((double)m_sourceWidth / m_scalingRatioHor2 + 0.5));
      maxPicHeight = std::max(maxPicHeight, (int)((double)m_sourceHeight / m_scalingRatioVer2 + 0.5));
      maxPicWidth = std::max(maxPicWidth, (int)((double)m_sourceWidth / m_scalingRatioHor3 + 0.5));
      maxPicHeight = std::max(maxPicHeight, (int)((double)m_sourceHeight / m_scalingRatioVer3 + 0.5));
    }
#endif

    const int minCuSize = std::max(8, 1 << m_log2MinCUSize);
    if (maxPicWidth % minCuSize)
    {
      maxPicWidth += ((maxPicWidth / minCuSize) + 1) * minCuSize - maxPicWidth;
    }
    if (maxPicHeight % minCuSize)
    {
      maxPicHeight += ((maxPicHeight / minCuSize) + 1) * minCuSize - maxPicHeight;
    }
    sps.setMaxPicWidthInLumaSamples( maxPicWidth );
    sps.setMaxPicHeightInLumaSamples( maxPicHeight );
  }
  sps.setConformanceWindow( m_conformanceWindow );

  sps.setMaxCUWidth             ( m_maxCUWidth        );
  sps.setMaxCUHeight            ( m_maxCUHeight       );
  sps.setLog2MinCodingBlockSize ( m_log2MinCUSize );
  sps.setChromaFormatIdc        ( m_chromaFormatIDC   );

  sps.setCTUSize                             ( m_CTUSize );
  sps.setSplitConsOverrideEnabledFlag        ( m_useSplitConsOverride );
  // convert the Intra Chroma minQT setting from chroma unit to luma unit
  m_uiMinQT[2] <<= getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, m_chromaFormatIDC);
  sps.setMinQTSizes                          ( m_uiMinQT );
  sps.setMaxMTTHierarchyDepth                ( m_uiMaxMTTHierarchyDepth, m_uiMaxMTTHierarchyDepthI, m_uiMaxMTTHierarchyDepthIChroma );
  sps.setMaxBTSize( m_uiMaxBT[1], m_uiMaxBT[0], m_uiMaxBT[2] );
  sps.setMaxTTSize( m_uiMaxTT[1], m_uiMaxTT[0], m_uiMaxTT[2] );
  sps.setIDRRefParamListPresent              ( m_idrRefParamList );
  sps.setUseDualITree                        ( m_dualITree );
#if SIGN_PREDICTION
  sps.setNumPredSigns                        ( m_numPredSign );
#if JVET_Y0141_SIGN_PRED_IMPROVE
  sps.setLog2SignPredArea                    (m_log2SignPredArea);
#endif
#endif
#if AHG7_MTS_TOOLOFF_CFG
  sps.setUseMTSExt(m_MTSExt);
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  sps.setUseIntraLFNSTISlice                 ( m_intraLFNSTISlice );
  sps.setUseIntraLFNSTPBSlice                ( m_intraLFNSTPBSlice );
  sps.setUseInterLFNST                       ( m_interLFNST );
#else
  sps.setUseLFNST                            ( m_LFNST );
#endif
#if AHG7_LN_TOOLOFF_CFG
  sps.setUseNSPT                             ( m_NSPT );
  sps.setUseLFNSTExt                         ( m_LFNSTExt );
#endif
  sps.setSbTMVPEnabledFlag(m_sbTmvpEnableFlag);
  sps.setAMVREnabledFlag                ( m_ImvMode != IMV_OFF );
  sps.setBDOFEnabledFlag                    ( m_BIO );
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  sps.setMvLdbRefineSwitch                  ( m_BIO ? m_sourceWidth >= 1280 && m_sourceHeight >= 720 : false );
#endif
#if JVET_W0090_ARMC_TM
  sps.setUseAML                             ( m_AML );
#if JVET_AG0276_NLIC
  sps.setUseAltLM                           ( m_altLM );
  sps.setUseAffAltLM                        ( m_affAltLM );
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  sps.setUseMergeOppositeLic                ( m_mergeOppositeLic );
  sps.setUseTMMergeOppositeLic              ( m_mergeTMOppositeLic );
  sps.setUseAffMergeOppositeLic             ( m_mergeAffOppositeLic );
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  sps.setUseFastSubTmvp                     ((m_sourceWidth * m_sourceHeight) > (m_intraPeriod == -1 ? 0 : 832 * 480));
#endif
#if JVET_AI0183_MVP_EXTENSION
  sps.setConfigScaledMvExtTmvp( m_scaledMvExtTmvp );
  if (m_intraPeriod == -1)
  {
    sps.setConfigScaledMvExtTmvp( false );
    setMaxNumAffineMergeCand(getMaxNumAffineMergeCand() - 2);
  }
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  sps.setConfigSbTmvpMvExt(m_sbTmvpMvExt);
  if (m_intraPeriod == -1)
  {
    sps.setConfigSbTmvpMvExt(false);
    setMaxNumAffineMergeCand(getMaxNumAffineMergeCand() - 2);
  }
#endif
#if JVET_AL0157_MERGE_CANDIDATE_EXTENSION
  sps.setConfigBoundaryMvpExt(m_boundaryMvpExt);
  if (m_intraPeriod == -1) 
  {
    if ((m_sourceWidth * m_sourceHeight) < ( 1280 * 720) )
    {
      sps.setConfigBoundaryMvpExt(false);
    }
  }
  else
  {
    if ((m_sourceWidth * m_sourceHeight) >= (1280 * 720))
    {
      sps.setConfigBoundaryMvpExt(false);
    }
  }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  if (m_sourceWidth * m_sourceHeight > 1920 * 1080)
  {
    m_refineTmvpCfgIdx = 2;
  }
  else if (m_sourceWidth * m_sourceHeight > 832 * 480)
  {
    m_refineTmvpCfgIdx = 1;
  }
  else
  {
    m_refineTmvpCfgIdx = 0;
  }
  sps.setRefineTmvpCfgIdx(m_refineTmvpCfgIdx);
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  sps.setUseArmcRefinedMotion               ( m_armcRefinedMotion );
#endif
  sps.setMaxNumMergeCand(getMaxNumMergeCand());
#if JVET_AG0276_LIC_FLAG_SIGNALING
  sps.setMaxNumOppositeLicMergeCand( getMaxNumOppositeLicMergeCand() );
#endif
#if JVET_X0049_ADAPT_DMVR
  sps.setMaxNumBMMergeCand(getMaxNumBMMergeCand());
#endif
  sps.setMaxNumAffineMergeCand(getMaxNumAffineMergeCand());
#if JVET_AG0276_LIC_FLAG_SIGNALING
  sps.setMaxNumAffineOppositeLicMergeCand( getMaxNumAffineOppositeLicMergeCand() );
  if (getIntraPeriod() < 0 && getBaseQP() > 32 )
  {
    sps.setUseMergeOppositeLic(false);
    sps.setUseTMMergeOppositeLic(false);
    sps.setUseAffMergeOppositeLic(false);
  }
#endif
  sps.setMaxNumIBCMergeCand(getMaxNumIBCMergeCand());
  sps.setMaxNumGeoCand(getMaxNumGeoCand());
#if JVET_AG0164_AFFINE_GPM
#if JVET_AJ0274_GPM_AFFINE_TM
  sps.setMaxNumGpmAffCand      (getMaxNumGpmAffCand());
  sps.setMaxNumGpmAffTmCand    (m_intraPeriod == -1 ? (m_sourceWidth * m_sourceHeight >= 1920 * 1080 ? 0 : getMaxNumGpmAffTmCand() - 2) : getMaxNumGpmAffTmCand());
#else
  sps.setMaxNumGpmAffCand      (((m_sourceWidth * m_sourceHeight) > (m_intraPeriod == -1 ? 1280 * 720 : 0)) ? getMaxNumGpmAffCand() : 0);
#endif
#endif
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  sps.setMaxNumMHPCand(getMaxNumMHPCand());
#endif
  sps.setUseAffine             ( m_Affine );
  sps.setUseAffineType         ( m_AffineType );
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  sps.setUseAltCost            ( m_useAltCost );
  if ((getSourceWidth() * getSourceHeight()) > (832 * 480) && ((getSourceWidth() * getSourceHeight()) < (1920 * 1080)))
  {
    if (getBaseQP() > 27)
    {
      sps.setUseAltCost(false);
    }
  }
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  sps.setUseExtAmvp            ( m_useExtAmvp );
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  sps.setUseAffineTM           ( m_useAffineTM );
#if JVET_AG0276_NLIC
  sps.setUseAffAltLMTM         ( m_useAffAltLMTM );
  if (getIntraPeriod() > 0)
  {
    if ((getSourceWidth() * getSourceHeight()) > (832 * 480) && ((getSourceWidth() * getSourceHeight()) < (3840 * 2160)))
    {
      sps.setUseAffAltLMTM(false);
    }
    if (getBaseQP() > 32)
    {
      sps.setUseAltLM(false);
      sps.setUseAffAltLM(false);
      sps.setUseAffAltLMTM(false);
    }
    else if (getBaseQP() < 27)
    {
      sps.setUseAltLM(false);
      sps.setUseAffAltLM(true);
      sps.setUseAffAltLMTM(true);
    }
  }
  else
  {
    sps.setUseAffAltLM(false);
    sps.setUseAffAltLMTM(false);
    if (getBaseQP() < 27)
    {
      sps.setUseAltLM(false);
    }
  }
#endif
#if JVET_AH0119_SUBBLOCK_TM
  sps.setUseSbTmvpTM(m_useSbTmvpTM);
  if (getBaseQP() < 27)
  {
    sps.setUseSbTmvpTM(false);
    sps.setUseAffineTM(false);
  }
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
  sps.setUseCiipAffine         (((m_sourceWidth * m_sourceHeight) > (m_intraPeriod == -1 ? 832 * 480 : 0)) ? m_useCiipAffine : false);
#endif
#if AFFINE_MMVD
  sps.setUseAffineMmvdMode     ( m_AffineMmvdMode );
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  sps.setUseDMVDMode           ( m_DMVDMode );
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  sps.setTMToolsEnableFlag     ( m_tmToolsEnableFlag );
#if TM_AMVP
  sps.setUseTMAmvpMode         ( m_tmAmvpMode );
#endif
#if TM_MRG
  sps.setUseTMMrgMode          ( m_tmMrgMode );
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  sps.setUseGPMTMMode          ( m_tmGPMMode );
#endif
#if JVET_Z0061_TM_OBMC && ENABLE_OBMC
  sps.setUseOBMCTMMode         ( m_tmOBMCMode );
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  sps.setUseTmvpNmvpReordering ( m_useTmvpNmvpReorder );
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  sps.setUseTMMMVD             ( m_useTMMMVD );
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  sps.setUseAltGPMSplitModeCode( m_altGPMSplitModeCode );
#endif
#if JVET_AL0134_SGPM_INTER
  sps.setUseSgpmInter(m_useSgpmInter);
#endif
  sps.setUsePROF               ( m_PROF );
  sps.setUseLMChroma           ( m_LMChroma ? true : false );
  sps.setHorCollocatedChromaFlag( m_horCollocatedChromaFlag );
  sps.setVerCollocatedChromaFlag( m_verCollocatedChromaFlag );
  sps.setUseMTS                ( m_IntraMTS || m_InterMTS || m_ImplicitMTS );
  sps.setUseIntraMTS           ( m_IntraMTS );
  sps.setUseInterMTS           ( m_InterMTS );
  sps.setUseSBT                             ( m_SBT );
#if JVET_AI0050_INTER_MTSS
  sps.setUseInterMTSS          ( m_useInterMTSS );
#endif
#if JVET_AI0050_SBT_LFNST
  sps.setUseSbtLFNST           ( m_useSbtLFNST );
#endif
#if JVET_AL0181_ASBT
  if ( !m_PLTMode && (m_intraPeriod != 1) && (m_iQP > 22) )
  {
    sps.setUseASBT(true);
    if ( m_intraPeriod > 1 || ( (m_iQP >= 27) && (m_iQP < 32) ) )
    {
      sps.setUseASBTphSignaling(false);
    }
    else
    {
      sps.setUseASBTphSignaling(true);
    }
  }
  else
  {
    sps.setUseASBT(false);
  }
#endif
  sps.setUseSMVD                ( m_SMVD );
  sps.setUseBcw                ( m_bcw );
#if INTER_LIC
  sps.setLicEnabledFlag        ( m_lic );
#if JVET_AG0276_LIC_SLOPE_ADJUST
  sps.setLicSlopeAdjustEnabledFlag( m_licSlopeAdjust );
#endif
#endif
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  sps.setLadfEnabled           ( m_LadfEnabled );
  if ( m_LadfEnabled )
  {
    sps.setLadfNumIntervals    ( m_LadfNumIntervals );
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      sps.setLadfQpOffset( m_LadfQpOffset[k], k );
      sps.setLadfIntervalLowerBound( m_LadfIntervalLowerBound[k], k );
    }
    CHECK( m_LadfIntervalLowerBound[0] != 0, "abnormal value set to LadfIntervalLowerBound[0]" );
  }
#endif
#if JVET_AA0133_INTER_MTS_OPT
  sps.setInterMTSMaxSize(m_interMTSMaxSize);
#endif
#if AHG7_MTS_TOOLOFF_CFG
  sps.setIntraMTSMaxSize(m_intraMTSMaxSize);
#endif
#if ENABLE_DIMD
  sps.setUseDimd            ( m_dimd );
#endif
#if JVET_AL0108_BVG_DIMD
  sps.setUseBvgDimd         ( m_bvgDimd );
#endif
#if JVET_W0123_TIMD_FUSION
  sps.setUseTimd            ( m_timd );
#if JVET_AJ0061_TIMD_MERGE
  sps.setUseTimdMrg         ( m_timdMrg );
#endif
#endif
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  sps.setUseCiipTimd        ( m_ciipTimd );
#endif
#if JVET_AB0155_SGPM
  sps.setUseSgpm            ( m_sgpm );
#endif
#if JVET_AD0082_TMRL_CONFIG
  sps.setUseTmrl            ( m_tmrl );
#endif
#if JVET_AN0095_QUANTIZATION_CENTER_SHIFT
  sps.setUseQcs             ( m_qcs );
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  sps.setTMnoninterToolsEnableFlag            ( m_tmNoninterToolsEnableFlag );
#endif
#if JVET_AG0058_EIP
  sps.setUseEip             ( m_eip );
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  sps.setUseIntraPredBf     ( m_intraPredBf );
#endif
#if JVET_AD0085_MPM_SORTING
  sps.setUseMpmSorting      ( m_mpmSorting );
#endif
#if JVET_AK0059_MDIP
  sps.setUseMdip            ( m_mdip );
#endif
#if JVET_AH0136_CHROMA_REORDERING
  sps.setUseChromaReordering (m_chromaReordering);
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  sps.setUseCccm            ( m_cccm );
#endif
#if JVET_AD0188_CCP_MERGE
  sps.setUseCcpMerge        ( m_ccpMerge );
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  if (sps.getUseCcpMerge())
  {
    sps.setMaxNumCcpMergeAdjustCand ( m_intraPeriod == 1 && m_sourceWidth * m_sourceHeight >= 3840 * 2160 ? m_maxNumCcpMergeAdjustCand - 1 : (m_intraPeriod != 1 && m_sourceWidth * m_sourceHeight >= 3840 * 2160 ? 0 : m_maxNumCcpMergeAdjustCand) );
  }
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  sps.setUseDdCcpFusion     ( m_ddCcpFusion );
#endif
#if ENABLE_OBMC
  sps.setUseOBMC            ( m_OBMC );
#endif
  sps.setUseCiip            ( m_ciip );
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if(sps.getUseCiip())
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
   if(m_tmCIIPMode == 2)
   {
#endif
   if(getIntraPeriod() < 0)
   {
      sps.setUseCiipTmMrg (false);
   }
   else
   {
      sps.setUseCiipTmMrg (true);
   }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
   }
   else
   {
      sps.setUseCiipTmMrg  (m_tmCIIPMode == 1);
   }
#endif
  }
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  sps.setUseTemporalAffineOpt  ( m_sourceWidth * m_sourceHeight > 832 * 480 && getBaseQP() > 22 );
  sps.setUseSyntheticAffine    ( m_sourceWidth * m_sourceHeight < 3840 * 2160 && getBaseQP() > 22 );
#endif
  sps.setUseGeo                ( m_Geo );
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  #if JVET_AN0203_RGPM_NO_TM
  sps.setUseGeoBlend           ( m_Geo && m_geoBlendInter );
#if JVET_AK0101_REGRESSION_GPM_INTRA
  sps.setUseGeoBlendIntra      ( m_Geo && m_geoBlendInter && m_geoBlendIntra && getBaseQP() < 37 );
#endif
#else
  sps.setUseGeoBlend           ( m_Geo && m_tmToolsEnableFlag );
#if JVET_AK0101_REGRESSION_GPM_INTRA
  sps.setUseGeoBlendIntra      ( m_Geo && m_tmToolsEnableFlag && m_geoBlendIntra && getBaseQP() < 37);
#endif
#endif
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  sps.setUseGeoInterIbc        ( m_Geo ? m_geoInterIbc : false );
#endif
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  sps.setUseGeoShapeAdapt      ( m_geoShapeAdapt );
#endif
  sps.setUseMMVD               ( m_MMVD );
  sps.setFpelMmvdEnabledFlag   (( m_MMVD ) ? m_allowDisFracMMVD : false);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  sps.setUseMvdPred            (m_mvdPred);
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  sps.setUseBvdPred            (m_bvdPred);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  sps.setUseBvpCluster         (m_bvpCluster);
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  sps.setUseARL                (m_useARL);
#endif
  sps.setBdofControlPresentFlag(m_BIO);
  sps.setDmvrControlPresentFlag(m_DMVR);
  sps.setProfControlPresentFlag(m_PROF);
  sps.setAffineAmvrEnabledFlag              ( m_AffineAmvr );
  sps.setUseDMVR                            ( m_DMVR );
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  sps.setUseAffineParaRefinement            (m_affineParaRefinement);
#endif
  sps.setUseColorTrans(m_useColorTrans);
  sps.setPLTMode                            ( m_PLTMode);
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  sps.setIBCFlag                            ( m_IBCMode);
#else
  sps.setIBCFlag                            ( m_IBCMode & 0x01);
  sps.setIBCFlagInterSlice                  ( m_IBCMode & 0x02);
  sps.setUseRRIbc                           ( m_rribc );
  sps.setUseTMIbc                           ( m_tmibc );
  sps.setUseIbcMerge                        ( m_ibcMerge );
  sps.setIBCFracFlag                        ( m_IBCFracMode);
#endif
#if JVET_AA0061_IBC_MBVD
  sps.setUseIbcMbvd                         ( m_ibcMbvd );
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  sps.setUseIbcMbvdAdSearch                 ( m_ibcMbvdAdSearch );
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  sps.setUseIbcCiip                         ( m_ibcCiip );
#endif
#if JVET_AC0112_IBC_GPM
  sps.setUseIbcGpm                          ( m_ibcGpm );
#endif
#if JVET_AC0112_IBC_LIC
  sps.setUseIbcLic                          ( m_ibcLic );
#endif
#if JVET_AE0159_FIBC
  sps.setUseIbcFilter                       ( m_ibcFilter );
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  sps.setUseIbcNonAdjCand                   ( m_ibcNonAdjCand );
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  sps.setItmpLicExtension                   ( m_itmpLicExtension );
  sps.setItmpLicMode                        ( m_itmpLicMode );
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  sps.setDisableRefFilter                   ( false );
  sps.setDisablePdpc                        ( false );
  sps.setDisableIntraFusion                 ( false );
#endif
  sps.setWrapAroundEnabledFlag                      ( m_wrapAround );
#if JVET_AH0135_TEMPORAL_PARTITIONING
  sps.setEnableMaxMttIncrease               ( m_enableMaxMttIncrease );
#endif
#if MULTI_HYP_PRED
  sps.setMaxNumAddHyps(m_maxNumAddHyps);
  sps.setNumAddHypWeights(m_numAddHypWeights);
  sps.setMaxNumAddHypRefFrames(m_maxNumAddHypRefFrames);
#endif
#if JVET_V0130_INTRA_TMP
  sps.setUseIntraTMP(m_intraTMP);
  sps.setIntraTMPMaxSize(m_intraTmpMaxSize);
#endif
#if JVET_AE0100_BVGCCCM
  sps.setUseBvgCccm(m_bvgCccm);
#endif
#if JVET_AC0071_DBV
  sps.setUseIntraDBV(m_intraDBV);
#endif
#if JVET_AE0059_INTER_CCCM
  sps.setUseInterCccm(m_interCccm);
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  sps.setUseInterCcpMerge(m_interCcpMerge);
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  sps.setUseInterCcpMergeZeroLumaCbf(m_interCcpMergeZeroLumaCbf);
#endif
#endif
#if JVET_AH0209_PDP
  sps.setUsePDP( m_pdp );
#endif
#if JVET_AI0183_MVP_EXTENSION
  sps.setConfigScaledMvExtBiTmvp( m_scaledMvExtBiTmvp );
  if (getBaseQP() < 27 && ((getSourceWidth() * getSourceHeight()) < (3840 * 2160)))
  {
    sps.setConfigScaledMvExtBiTmvp( false );
  }
#endif
  // ADD_NEW_TOOL : (encoder lib) set tool enabling flags and associated parameters here
  sps.setUseISP                             ( m_ISP );
  sps.setUseLmcs                            ( m_lmcsEnabled );
  sps.setUseMRL                ( m_MRL );
  sps.setUseMIP                ( m_MIP );
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  sps.setNnipMode(m_nnip);
#endif
  CHECK(m_log2MinCUSize > std::min(6, floorLog2(sps.getMaxCUWidth())), "log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");
  CHECK(m_uiMaxMTTHierarchyDepth > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK(m_uiMaxMTTHierarchyDepthI > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK(m_uiMaxMTTHierarchyDepthIChroma > 2 * (floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize()), "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  sps.setTransformSkipEnabledFlag(m_useTransformSkip);
  sps.setLog2MaxTransformSkipBlockSize(m_log2MaxTransformSkipBlockSize);
  sps.setBDPCMEnabledFlag(m_useBDPCM);

  sps.setSPSTemporalMVPEnabledFlag((getTMVPModeId() == 2 || getTMVPModeId() == 1));
#if JVET_AL0160_SBSMVP
  sps.setSpatialMVPEnabledFlag( m_sourceHeight < 2160 );
#endif
  sps.setLog2MaxTbSize   ( m_log2MaxTbSize );

  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    sps.setBitDepth      (ChannelType(channelType), m_bitDepth[channelType] );
    sps.setQpBDOffset  (ChannelType(channelType), (6 * (m_bitDepth[channelType] - 8)));
    sps.setInternalMinusInputBitDepth(ChannelType(channelType), max(0, (m_bitDepth[channelType] - m_inputBitDepth[channelType])));
  }

  sps.setEntropyCodingSyncEnabledFlag( m_entropyCodingSyncEnabledFlag );
  sps.setEntryPointsPresentFlag( m_entryPointPresentFlag );

  sps.setUseWP( m_useWeightedPred );
  sps.setUseWPBiPred( m_useWeightedBiPred );

  sps.setSAOEnabledFlag( m_bUseSAO );
#if JVET_W0066_CCSAO
  sps.setCCSAOEnabledFlag( m_CCSAO );
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  sps.setAlfPrecisionFlag( m_alfPrecision );
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION
  sps.setCCALFPrecisionFlag( m_ccalfPrecision );
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  sps.setAlfLumaFixedFilterAdjust( m_intraPeriod < 0 ? false : true );
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  sps.setInloopOffsetRefineFlag( m_intraPeriod < 0 ? false : true );
  sps.setInloopOffsetRefineFunc( m_intraPeriod < 0 ? ( getBaseQP() < 30 ? 1 : 0 ) : 0 );
#endif
  sps.setJointCbCrEnabledFlag( m_JointCbCrMode );
  sps.setMaxTLayers( m_maxTempLayer );
  sps.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );

  for (int i = 0; i < std::min(sps.getMaxTLayers(), (uint32_t) MAX_TLAYER); i++ )
  {
    sps.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
    sps.setNumReorderPics(m_numReorderPics[i], i);
  }

  sps.setScalingListFlag ( (m_useScalingListId == SCALING_LIST_OFF) ? 0 : 1 );
  if (sps.getUseColorTrans() && sps.getScalingListFlag())
  {
    sps.setScalingMatrixForAlternativeColourSpaceDisabledFlag( m_disableScalingMatrixForAlternativeColourSpace );
  }
  else
  {
    sps.setScalingMatrixForAlternativeColourSpaceDisabledFlag( false );
  }
  if (sps.getScalingMatrixForAlternativeColourSpaceDisabledFlag())
  {
    sps.setScalingMatrixDesignatedColourSpaceFlag( m_scalingMatrixDesignatedColourSpace );
  }
  else
  {
    sps.setScalingMatrixDesignatedColourSpaceFlag( true );
  }
#if JVET_AL0153_ALF_CCCM
  sps.setLfCccmEnabledFlag( m_lfCccm );
#endif
  sps.setALFEnabledFlag( m_alf );
  sps.setCCALFEnabledFlag( m_ccalf );

#if NN_COMMON_SPS
  sps.setNnlfEnabledFlag(m_nnlf);
  sps.setNnlfId(m_nnlfId);
#endif
  
#if NN_LF_UNIFIED
  sps.setNnlfUnifiedEnabledFlag(m_nnlfUnified);
  if (sps.getNnlfUnifiedEnabledFlag())
  {
    uint32_t nnlfInferSize[] = { m_nnlfUnifiedBlockSize >> 1, m_nnlfUnifiedBlockSize, m_nnlfUnifiedBlockSize << 1 };
    sps.setNnlfUnifiedInferSize(nnlfInferSize);
    sps.setNnlfUnifiedInfSizeExt(m_nnlfUnifiedInfSizeExt);
    sps.setNnlfUnifiedMaxNumPrms(m_nnlfUnifiedMaxNumPrms); // only on/off for all-intra
  }
#endif

  sps.setFieldSeqFlag(false);
  sps.setVuiParametersPresentFlag(getVuiParametersPresentFlag());

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  sps.setTempCabacInitMode( m_tempCabacInitMode );
#endif

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  sps.setAlfScalePrevEnabled( true );
  if ( getIntraPeriod() == 1 )
  {
    sps.setAlfScaleMode( 0 );
  }
  else if ( getIntraPeriod() < 0 )
  {
    if ( m_sourceWidth * m_sourceHeight < 1920 * 1080 )
    {
      sps.setAlfScaleMode( 2 );
      sps.setAlfScalePrevEnabled( false );
    }
    else
    {
      sps.setAlfScaleMode( 3 );
    }
  }
  else
  {
    sps.setAlfScaleMode( 1 );
  }
#endif
#if JVET_AK0065_TALF
  sps.setUseTAlf(m_alf);
#endif
#if JVET_AN0168_REGRESSION_CCP_FUSION
  sps.setCcpFusionMode( getIntraPeriod() == 1 ? 0 : 1 );
#endif

  if (sps.getVuiParametersPresentFlag())
  {
    VUI* pcVUI = sps.getVuiParameters();
    pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
    pcVUI->setAspectRatioConstantFlag(!getSampleAspectRatioInfoSEIEnabled());
    pcVUI->setAspectRatioIdc(getAspectRatioIdc());
    pcVUI->setSarWidth(getSarWidth());
    pcVUI->setSarHeight(getSarHeight());
    pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
    pcVUI->setColourPrimaries(getColourPrimaries());
    pcVUI->setTransferCharacteristics(getTransferCharacteristics());
    pcVUI->setMatrixCoefficients(getMatrixCoefficients());
    pcVUI->setProgressiveSourceFlag       (getProgressiveSourceFlag());
    pcVUI->setInterlacedSourceFlag        (getInterlacedSourceFlag());
#if JVET_S0266_VUI_length
    pcVUI->setNonPackedFlag               (getNonPackedConstraintFlag());
    pcVUI->setNonProjectedFlag            (getNonProjectedConstraintFlag());
#endif
    pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
    pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
    pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
    pcVUI->setChromaSampleLocType(getChromaSampleLocType());
    pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
    pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
    pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
  }

  sps.setNumLongTermRefPicSPS(NUM_LONG_TERM_REF_PIC_SPS);
  CHECK(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.setLtRefPicPocLsbSps(k, 0);
    sps.setUsedByCurrPicLtSPSFlag(k, 0);
  }
  int numQpTables = m_chromaQpMappingTableParams.getSameCQPTableForAllChromaFlag() ? 1 : (sps.getJointCbCrEnabledFlag() ? 3 : 2);
  m_chromaQpMappingTableParams.setNumQpTables(numQpTables);
  sps.setChromaQpMappingTableFromParams(m_chromaQpMappingTableParams, sps.getQpBDOffset(CHANNEL_TYPE_CHROMA));
  sps.derivedChromaQPMappingTables();

#if U0132_TARGET_BITS_SATURATION
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() || getCpbSaturationEnabled() )
#else
  if( getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
#endif
  {
    xInitHrdParameters(sps);
  }
  if( getBufferingPeriodSEIEnabled() || getPictureTimingSEIEnabled() || getDecodingUnitInfoSEIEnabled() )
  {
    sps.setGeneralHrdParametersPresentFlag(true);
  }

  // Set up SPS range extension settings
  sps.getSpsRangeExtension().setTransformSkipRotationEnabledFlag(m_transformSkipRotationEnabledFlag);
  sps.getSpsRangeExtension().setTransformSkipContextEnabledFlag(m_transformSkipContextEnabledFlag);
  sps.getSpsRangeExtension().setExtendedPrecisionProcessingFlag(m_extendedPrecisionProcessingFlag);
  sps.getSpsRangeExtension().setIntraSmoothingDisabledFlag( m_intraSmoothingDisabledFlag );
  sps.getSpsRangeExtension().setHighPrecisionOffsetsEnabledFlag(m_highPrecisionOffsetsEnabledFlag);
  sps.getSpsRangeExtension().setPersistentRiceAdaptationEnabledFlag(m_persistentRiceAdaptationEnabledFlag);
  sps.getSpsRangeExtension().setCabacBypassAlignmentEnabledFlag(m_cabacBypassAlignmentEnabledFlag);

  sps.setSubPicInfoPresentFlag(m_subPicInfoPresentFlag);
  if (m_subPicInfoPresentFlag)
  {
    sps.setNumSubPics(m_numSubPics);
#if JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT
    sps.setSubPicSameSizeFlag(m_subPicSameSizeFlag);
    if (m_subPicSameSizeFlag)
    {
#if JVET_AA0146_WRAP_AROUND_FIX
      uint32_t numSubpicCols = (m_sourceWidth + m_CTUSize - 1) / m_CTUSize / m_subPicWidth[0];
#else
      uint32_t numSubpicCols = (m_iSourceWidth + m_CTUSize - 1) / m_CTUSize / m_subPicWidth[0];
#endif
      for (unsigned int i = 0; i < m_numSubPics; i++)
      {
        sps.setSubPicCtuTopLeftX(i, (i % numSubpicCols) * m_subPicWidth[0]);
        sps.setSubPicCtuTopLeftY(i, (i / numSubpicCols) * m_subPicHeight[0]);
        sps.setSubPicWidth(i, m_subPicWidth[0]);
        sps.setSubPicHeight(i, m_subPicHeight[0]);
      }
    }
    else
    {
      sps.setSubPicCtuTopLeftX(m_subPicCtuTopLeftX);
      sps.setSubPicCtuTopLeftY(m_subPicCtuTopLeftY);
      sps.setSubPicWidth(m_subPicWidth);
      sps.setSubPicHeight(m_subPicHeight);
    }
#else
    sps.setSubPicCtuTopLeftX(m_subPicCtuTopLeftX);
    sps.setSubPicCtuTopLeftY(m_subPicCtuTopLeftY);
    sps.setSubPicWidth(m_subPicWidth);
    sps.setSubPicHeight(m_subPicHeight);
#endif
    sps.setSubPicTreatedAsPicFlag(m_subPicTreatedAsPicFlag);
    sps.setLoopFilterAcrossSubpicEnabledFlag(m_loopFilterAcrossSubpicEnabledFlag);
    sps.setSubPicIdLen(m_subPicIdLen);
    sps.setSubPicIdMappingExplicitlySignalledFlag(m_subPicIdMappingExplicitlySignalledFlag);
    if (m_subPicIdMappingExplicitlySignalledFlag)
    {
      sps.setSubPicIdMappingInSpsFlag(m_subPicIdMappingInSpsFlag);
      if (m_subPicIdMappingInSpsFlag)
      {
        sps.setSubPicId(m_subPicId);
      }
    }
  }
  else   //In that case, there is only one subpicture that contains the whole picture
  {
    sps.setNumSubPics(1);
    sps.setSubPicCtuTopLeftX(0, 0);
    sps.setSubPicCtuTopLeftY(0, 0);
#if JVET_AA0146_WRAP_AROUND_FIX
    sps.setSubPicWidth(0, m_sourceWidth);
    sps.setSubPicHeight(0, m_sourceHeight);
#else
    sps.setSubPicWidth(0, m_iSourceWidth);
    sps.setSubPicHeight(0, m_iSourceHeight);
#endif
    sps.setSubPicTreatedAsPicFlag(0, 1);
    sps.setLoopFilterAcrossSubpicEnabledFlag(0, 0);
    sps.setSubPicIdLen(0);
    sps.setSubPicIdMappingExplicitlySignalledFlag(false);
  }

#if TCQ_8STATES
  sps.setDepQuantEnabledFlag( m_DepQuantEnabledIdc ? true : false );
#else
  sps.setDepQuantEnabledFlag( m_DepQuantEnabledFlag );
#endif

  if (!sps.getDepQuantEnabledFlag())
  {
    sps.setSignDataHidingEnabledFlag( m_SignDataHidingEnabledFlag );
  }
  else
  {
    sps.setSignDataHidingEnabledFlag(false);
  }
  sps.setVirtualBoundariesEnabledFlag( m_virtualBoundariesEnabledFlag );
  if( sps.getVirtualBoundariesEnabledFlag() )
  {
    sps.setVirtualBoundariesPresentFlag( m_virtualBoundariesPresentFlag );
    CHECK( sps.getSubPicInfoPresentFlag() && sps.getVirtualBoundariesPresentFlag() != 1, "When subpicture signalling if present, the signalling of virtual boundaries, is present, shall be in the SPS" );
    sps.setNumVerVirtualBoundaries            ( m_numVerVirtualBoundaries );
    sps.setNumHorVirtualBoundaries            ( m_numHorVirtualBoundaries );
    for( unsigned int i = 0; i < m_numVerVirtualBoundaries; i++ )
    {
      sps.setVirtualBoundariesPosX            ( m_virtualBoundariesPosX[i], i );
    }
    for( unsigned int i = 0; i < m_numHorVirtualBoundaries; i++ )
    {
      sps.setVirtualBoundariesPosY            ( m_virtualBoundariesPosY[i], i );
    }
  }
#if JVET_AK0085_TM_BOUNDARY_PADDING
  sps.setTMBP(m_templateMatchingBoundaryPrediction);
#endif

  sps.setInterLayerPresentFlag( m_layerId > 0 && m_vps->getMaxLayers() > 1 && !m_vps->getAllIndependentLayersFlag() && !m_vps->getIndependentLayerFlag( m_vps->getGeneralLayerIdx( m_layerId ) ) );
  CHECK( m_vps->getIndependentLayerFlag( m_vps->getGeneralLayerIdx( m_layerId ) ) && sps.getInterLayerPresentFlag(), " When vps_independent_layer_flag[GeneralLayerIdx[nuh_layer_id ]]  is equal to 1, the value of inter_layer_ref_pics_present_flag shall be equal to 0." );

  sps.setResChangeInClvsEnabledFlag(m_resChangeInClvsEnabled);
#if JVET_Q0114_ASPECT5_GCI_FLAG
  sps.setRprEnabledFlag(m_rprEnabledFlag);
#else
  sps.setRprEnabledFlag((m_resChangeInClvsEnabled) || sps.getInterLayerPresentFlag());
#endif

  sps.setLog2ParallelMergeLevelMinus2( m_log2ParallelMergeLevelMinus2 );

  CHECK(sps.getResChangeInClvsEnabledFlag() && sps.getVirtualBoundariesEnabledFlag(), "when the value of res_change_in_clvs_allowed_flag is equal to 1, the value of sps_virtual_boundaries_present_flag shall be equal to 0");
}

void EncLib::xInitHrdParameters(SPS &sps)
{
  m_encHRD.initHRDParameters((EncCfg*) this);

  GeneralHrdParams *generalHrdParams = sps.getGeneralHrdParameters();
  *generalHrdParams = m_encHRD.getGeneralHrdParameters();

  OlsHrdParams *spsOlsHrdParams = sps.getOlsHrdParameters();
  for(int i = 0; i < MAX_TLAYER; i++)
  {
    *spsOlsHrdParams = m_encHRD.getOlsHrdParameters(i);
    spsOlsHrdParams++;
  }
}

void EncLib::xInitPPS(PPS &pps, const SPS &sps)
{
  // pps ID already initialised.
  pps.setSPSId(sps.getSPSId());

  pps.setNumSubPics(sps.getNumSubPics());
  pps.setSubPicIdMappingInPpsFlag(false);
  pps.setSubPicIdLen(sps.getSubPicIdLen());
  for(int picIdx=0; picIdx<pps.getNumSubPics(); picIdx++)
  {
    pps.setSubPicId(picIdx, sps.getSubPicId(picIdx));
  }
  bool bUseDQP = (getCuQpDeltaSubdiv() > 0)? true : false;

  if((getMaxDeltaQP() != 0 )|| getUseAdaptiveQP())
  {
    bUseDQP = true;
  }

#if JVET_AB0171_ASYMMETRIC_DB_FOR_GDR
  pps.setAsymmetricILF(getAsymmetricILF());
#endif

#if SHARP_LUMA_DELTA_QP
  if ( getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    bUseDQP = true;
  }
#endif
#if ENABLE_QPA
  if (getUsePerceptQPA() && !bUseDQP)
  {
    CHECK( m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!" );
    bUseDQP = (getBaseQP() < 38) && (getSourceWidth() > 512 || getSourceHeight() > 320);
  }
#endif
#if JVET_Y0240_BIM
  if (m_bimEnabled)
  {
    bUseDQP = true;
  }
#endif

  if (m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING)
  {
    bUseDQP=false;
  }


  if ( m_RCEnableRateControl )
  {
    pps.setUseDQP(true);
  }
  else if(bUseDQP)
  {
    pps.setUseDQP(true);
  }
  else
  {
    pps.setUseDQP(false);
  }

  if ( m_cuChromaQpOffsetSubdiv >= 0 )
  {
    pps.clearChromaQpOffsetList();
    pps.setChromaQpOffsetListEntry(1, 6, 6, 6);
    /* todo, insert table entries from command line (NB, 0 should not be touched) */
  }
  else
  {
    pps.clearChromaQpOffsetList();
  }
  {
    int baseQp = 26;
    if( 16 == getGOPSize() )
    {
      baseQp = getBaseQP()-24;
    }
    else
    {
      baseQp = getBaseQP()-26;
    }
    const int maxDQP = 37;
    const int minDQP = -26 + sps.getQpBDOffset(CHANNEL_TYPE_LUMA);

    pps.setPicInitQPMinus26( std::min( maxDQP, std::max( minDQP, baseQp ) ));
  }

  if( sps.getJointCbCrEnabledFlag() == false || getChromaFormatIdc() == CHROMA_400 || m_chromaCbCrQpOffset == 0 )
  {
    pps.setJointCbCrQpOffsetPresentFlag(false);
  }
  else
  {
    pps.setJointCbCrQpOffsetPresentFlag(true);
  }

#if ER_CHROMA_QP_WCG_PPS
  if (getWCGChromaQPControl().isEnabled())
  {
    const int baseQp=m_iQP+pps.getPPSId();
    const double chromaQp = m_wcgChromaQpControl.chromaQpScale * baseQp + m_wcgChromaQpControl.chromaQpOffset;
    const double dcbQP = m_wcgChromaQpControl.chromaCbQpScale * chromaQp;
    const double dcrQP = m_wcgChromaQpControl.chromaCrQpScale * chromaQp;
    const int cbQP =(int)(dcbQP + ( dcbQP < 0 ? -0.5 : 0.5) );
    const int crQP =(int)(dcrQP + ( dcrQP < 0 ? -0.5 : 0.5) );
    pps.setQpOffset(COMPONENT_Cb, Clip3( -12, 12, min(0, cbQP) + m_chromaCbQpOffset ));
    pps.setQpOffset(COMPONENT_Cr, Clip3( -12, 12, min(0, crQP) + m_chromaCrQpOffset));
    if(pps.getJointCbCrQpOffsetPresentFlag())
      pps.setQpOffset(JOINT_CbCr, Clip3(-12, 12, (min(0, cbQP) + min(0, crQP)) / 2 + m_chromaCbCrQpOffset));
    else
      pps.setQpOffset(JOINT_CbCr, 0);
  }
  else
  {
#endif
  pps.setQpOffset(COMPONENT_Cb, m_chromaCbQpOffset );
  pps.setQpOffset(COMPONENT_Cr, m_chromaCrQpOffset );
  if (pps.getJointCbCrQpOffsetPresentFlag())
    pps.setQpOffset(JOINT_CbCr, m_chromaCbCrQpOffset);
  else
    pps.setQpOffset(JOINT_CbCr, 0);
#if ER_CHROMA_QP_WCG_PPS
  }
#endif
#if W0038_CQP_ADJ
  bool bChromaDeltaQPEnabled = false;
  {
    bChromaDeltaQPEnabled = ( m_sliceChromaQpOffsetIntraOrPeriodic[0] || m_sliceChromaQpOffsetIntraOrPeriodic[1] );
    if( !bChromaDeltaQPEnabled )
    {
      for( int i=0; i<m_iGOPSize; i++ )
      {
        if( m_GOPList[i].m_CbQPoffset || m_GOPList[i].m_CrQPoffset )
        {
          bChromaDeltaQPEnabled = true;
          break;
        }
      }
    }
  }
 #if ENABLE_QPA
  if ((getUsePerceptQPA() || getSliceChromaOffsetQpPeriodicity() > 0) && (getChromaFormatIdc() != CHROMA_400))
  {
    bChromaDeltaQPEnabled = true;
  }
 #endif
  pps.setSliceChromaQpFlag(bChromaDeltaQPEnabled);
#endif
  if (
    !pps.getSliceChromaQpFlag() && sps.getUseDualITree()
    && (getChromaFormatIdc() != CHROMA_400))
  {
    pps.setSliceChromaQpFlag(m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0);
  }
#if JVET_AC0096
#if JVET_AG0116
  if (m_rprFunctionalityTestingEnabledFlag || m_gopBasedRPREnabledFlag)
#else
  if (m_rprFunctionalityTestingEnabledFlag)
#endif
  {
    if (pps.getPPSId() == ENC_PPS_ID_RPR || pps.getPPSId() == ENC_PPS_ID_RPR2 || pps.getPPSId() == ENC_PPS_ID_RPR3)
    {
      pps.setSliceChromaQpFlag(true);
    }
  }
#endif

  int minCbSizeY = (1 << sps.getLog2MinCodingBlockSize());
  pps.setWrapAroundEnabledFlag                ( m_wrapAround );
  if( m_wrapAround )
  {
    pps.setPicWidthMinusWrapAroundOffset      ((pps.getPicWidthInLumaSamples()/minCbSizeY) - (m_wrapAroundOffset / minCbSizeY));
    pps.setWrapAroundOffset                   (minCbSizeY *(pps.getPicWidthInLumaSamples() / minCbSizeY- pps.getPicWidthMinusWrapAroundOffset()));
  }
  else 
  {
    pps.setPicWidthMinusWrapAroundOffset      ( 0 );
    pps.setWrapAroundOffset                   ( 0 );       
  }
  CHECK( !sps.getWrapAroundEnabledFlag() && pps.getWrapAroundEnabledFlag(), "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");
  CHECK( (((sps.getCTUSize() / minCbSizeY) + 1) > ((pps.getPicWidthInLumaSamples() / minCbSizeY) - 1)) && pps.getWrapAroundEnabledFlag(), "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pic_width_in_luma_samples / MinCbSizeY - 1, the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");

  pps.setNoPicPartitionFlag( m_noPicPartitionFlag );
  if( m_noPicPartitionFlag == false )
  {
    pps.setLog2CtuSize( ceilLog2( sps.getCTUSize()) );
    pps.setNumExpTileColumns( (uint32_t) m_tileColumnWidth.size() );
    pps.setNumExpTileRows( (uint32_t) m_tileRowHeight.size() );
    pps.setTileColumnWidths( m_tileColumnWidth );
    pps.setTileRowHeights( m_tileRowHeight );
    pps.initTiles();
    pps.setRectSliceFlag( m_rectSliceFlag );
    if( m_rectSliceFlag )
    {
      pps.setSingleSlicePerSubPicFlag(m_singleSlicePerSubPicFlag);
      pps.setNumSlicesInPic( m_numSlicesInPic );
      pps.setTileIdxDeltaPresentFlag( m_tileIdxDeltaPresentFlag );
      pps.setRectSlices( m_rectSlices );
      pps.initRectSliceMap(&sps);
    }
    else
    {
      pps.initRasterSliceMap( m_rasterSliceSize );
    }
    pps.initSubPic(sps);
    pps.setLoopFilterAcrossTilesEnabledFlag( m_bLFCrossTileBoundaryFlag );
    pps.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );
  }
  else
  {
    pps.setLog2CtuSize( ceilLog2( sps.getCTUSize()) );
    pps.setNumExpTileColumns(1);
    pps.setNumExpTileRows(1);
    pps.addTileColumnWidth( pps.getPicWidthInCtu( ) );
    pps.addTileRowHeight( pps.getPicHeightInCtu( ) );
    pps.initTiles();
    pps.setRectSliceFlag( 1 );
    pps.setNumSlicesInPic( 1 );
    pps.initRectSlices( );
    pps.setTileIdxDeltaPresentFlag( 0 );
    pps.setSliceTileIdx( 0, 0 );
    pps.initRectSliceMap( &sps );
    pps.initSubPic(sps);
    pps.setLoopFilterAcrossTilesEnabledFlag( true );
    pps.setLoopFilterAcrossSlicesEnabledFlag( true );
  }

  pps.setUseWP( m_useWeightedPred );
  pps.setWPBiPred( m_useWeightedBiPred );
  pps.setOutputFlagPresentFlag( false );

#if JVET_AC0189_SGPM_NO_BLENDING
  pps.setUseSgpmNoBlend                ( m_sgpmNoBlend );
#endif
#if JVET_V0094_BILATERAL_FILTER
  pps.setUseBIF                ( m_BIF );
  pps.setBIFStrength           ( m_BIFStrength );
  pps.setBIFQPOffset           ( m_BIFQPOffset );
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  pps.setUseChromaBIF          ( m_chromaBIF );
  pps.setChromaBIFStrength     ( m_chromaBIFStrength );
  pps.setChromaBIFQPOffset     ( m_chromaBIFQPOffset );
#endif

  if ( getDeblockingFilterMetric() )
  {
    pps.setDeblockingFilterOverrideEnabledFlag(true);
    pps.setPPSDeblockingFilterDisabledFlag(false);
  }
  else
  {
    pps.setDeblockingFilterOverrideEnabledFlag( !getLoopFilterOffsetInPPS() );
    pps.setPPSDeblockingFilterDisabledFlag( getLoopFilterDisable() );
  }

  if (! pps.getPPSDeblockingFilterDisabledFlag())
  {
    pps.setDeblockingFilterBetaOffsetDiv2( getLoopFilterBetaOffset() );
    pps.setDeblockingFilterTcOffsetDiv2( getLoopFilterTcOffset() );
    pps.setDeblockingFilterCbBetaOffsetDiv2( getLoopFilterCbBetaOffset() );
    pps.setDeblockingFilterCbTcOffsetDiv2( getLoopFilterCbTcOffset() );
    pps.setDeblockingFilterCrBetaOffsetDiv2( getLoopFilterCrBetaOffset() );
    pps.setDeblockingFilterCrTcOffsetDiv2( getLoopFilterCrTcOffset() );
  }
  else
  {
#if DB_PARAM_TID
    pps.setDeblockingFilterBetaOffsetDiv2(std::vector<int>(5,0));
    pps.setDeblockingFilterTcOffsetDiv2(std::vector<int>(5, 0));
#else
    pps.setDeblockingFilterBetaOffsetDiv2(0);
    pps.setDeblockingFilterTcOffsetDiv2(0);
#endif

    pps.setDeblockingFilterCbBetaOffsetDiv2(0);
    pps.setDeblockingFilterCbTcOffsetDiv2(0);
    pps.setDeblockingFilterCrBetaOffsetDiv2(0);
    pps.setDeblockingFilterCrTcOffsetDiv2(0);
  }

  // deblockingFilterControlPresentFlag is true if any of the settings differ from the inferred values:
#if DB_PARAM_TID
  bool TcAllZero = true, BetaAllZero = true;
  for (int i = 0; i < pps.getDeblockingFilterBetaOffsetDiv2().size(); i++)
  {
    if (pps.getDeblockingFilterBetaOffsetDiv2()[i] != 0)
    {
      BetaAllZero = false;
      break;
    }
  }
  for (int i = 0; i < pps.getDeblockingFilterTcOffsetDiv2().size(); i++)
  {
    if (pps.getDeblockingFilterTcOffsetDiv2()[i] != 0)
    {
      TcAllZero = false;
      break;
    }
  }
  const bool deblockingFilterControlPresentFlag = pps.getDeblockingFilterOverrideEnabledFlag()   ||
                                                  pps.getPPSDeblockingFilterDisabledFlag()       ||
                                                  BetaAllZero == false                           ||
                                                  TcAllZero   == false                           ||
                                                  pps.getDeblockingFilterCbBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCbTcOffsetDiv2() != 0   ||
                                                  pps.getDeblockingFilterCrBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCrTcOffsetDiv2() != 0;
#else
  const bool deblockingFilterControlPresentFlag = pps.getDeblockingFilterOverrideEnabledFlag()   ||
                                                  pps.getPPSDeblockingFilterDisabledFlag()       ||
                                                  pps.getDeblockingFilterBetaOffsetDiv2() != 0   ||
                                                  pps.getDeblockingFilterTcOffsetDiv2() != 0     ||
                                                  pps.getDeblockingFilterCbBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCbTcOffsetDiv2() != 0   ||
                                                  pps.getDeblockingFilterCrBetaOffsetDiv2() != 0 ||
                                                  pps.getDeblockingFilterCrTcOffsetDiv2() != 0;
#endif

  pps.setDeblockingFilterControlPresentFlag(deblockingFilterControlPresentFlag);

  pps.setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
  pps.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );

  bool chromaQPOffsetNotZero = false;
  if( pps.getQpOffset(COMPONENT_Cb) != 0 || pps.getQpOffset(COMPONENT_Cr) != 0 || pps.getJointCbCrQpOffsetPresentFlag() || pps.getSliceChromaQpFlag() || pps.getCuChromaQpOffsetListEnabledFlag() )
  {
    chromaQPOffsetNotZero = true;
  }

  bool chromaDbfOffsetNotSameAsLuma = true;
#if !DB_PARAM_TID 
  if( pps.getDeblockingFilterCbBetaOffsetDiv2() == pps.getDeblockingFilterBetaOffsetDiv2() && pps.getDeblockingFilterCrBetaOffsetDiv2() == pps.getDeblockingFilterBetaOffsetDiv2()
     && pps.getDeblockingFilterCbTcOffsetDiv2() == pps.getDeblockingFilterTcOffsetDiv2() && pps.getDeblockingFilterCrTcOffsetDiv2() == pps.getDeblockingFilterTcOffsetDiv2() )
  {
    chromaDbfOffsetNotSameAsLuma = false;
  }
#endif

#if !JVET_S0052_RM_SEPARATE_COLOUR_PLANE
  const uint32_t chromaArrayType = (int)sps.getSeparateColourPlaneFlag() ? 0 : sps.getChromaFormatIdc();
  if( ( chromaArrayType != CHROMA_400 ) && ( chromaQPOffsetNotZero || chromaDbfOffsetNotSameAsLuma ) )
#else
  if ((sps.getChromaFormatIdc() != CHROMA_400) && (chromaQPOffsetNotZero || chromaDbfOffsetNotSameAsLuma))
#endif
  {
    pps.setPPSChromaToolFlag(true);
  }
  else
  {
    pps.setPPSChromaToolFlag(false);
  }

  int histogram[MAX_NUM_REF + 1];
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    histogram[i]=0;
  }
  for( int i = 0; i < getGOPSize(); i++)
  {
    CHECK(!(getRPLEntry(0, i).m_numRefPicsActive >= 0 && getRPLEntry(0, i).m_numRefPicsActive <= MAX_NUM_REF), "Unspecified error");
    histogram[getRPLEntry(0, i).m_numRefPicsActive]++;
  }

  int maxHist=-1;
  int bestPos=0;
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    if(histogram[i]>maxHist)
    {
      maxHist=histogram[i];
      bestPos=i;
    }
  }
  CHECK(!(bestPos <= 15), "Unspecified error");
    pps.setNumRefIdxL0DefaultActive(bestPos);
  pps.setNumRefIdxL1DefaultActive(bestPos);
  pps.setPictureHeaderExtensionPresentFlag(false);

  pps.setRplInfoInPhFlag(getSliceLevelRpl() ? false : true);
  pps.setDbfInfoInPhFlag(getSliceLevelDblk() ? false : true);
  pps.setSaoInfoInPhFlag(getSliceLevelSao() ? false : true);
  pps.setAlfInfoInPhFlag(getSliceLevelAlf() ? false : true);
  pps.setWpInfoInPhFlag(getSliceLevelWp() ? false : true);
  pps.setQpDeltaInfoInPhFlag(getSliceLevelDeltaQp() ? false : true);

  pps.pcv = new PreCalcValues( sps, pps, true );
  pps.setRpl1IdxPresentFlag(sps.getRPL1IdxPresentFlag());
}

void EncLib::xInitPicHeader(PicHeader &picHeader, const SPS &sps, const PPS &pps)
{
  int i;
  picHeader.initPicHeader();

  // parameter sets
  picHeader.setSPSId( sps.getSPSId() );
  picHeader.setPPSId( pps.getPPSId() );

  // merge list sizes
  picHeader.setMaxNumAffineMergeCand(getMaxNumAffineMergeCand());
#if JVET_AG0276_LIC_FLAG_SIGNALING
  picHeader.setMaxNumAffineOppositeLicMergeCand( getMaxNumAffineOppositeLicMergeCand() );
#endif
  // copy partitioning constraints from SPS
  picHeader.setSplitConsOverrideFlag(false);
  picHeader.setMinQTSizes( sps.getMinQTSizes() );
  picHeader.setMaxMTTHierarchyDepths( sps.getMaxMTTHierarchyDepths() );
  picHeader.setMaxBTSizes( sps.getMaxBTSizes() );
  picHeader.setMaxTTSizes( sps.getMaxTTSizes() );

  bool bUseDQP = (getCuQpDeltaSubdiv() > 0)? true : false;

  if( (getMaxDeltaQP() != 0 )|| getUseAdaptiveQP() )
  {
    bUseDQP = true;
  }

#if SHARP_LUMA_DELTA_QP
  if( getLumaLevelToDeltaQPMapping().isEnabled() )
  {
    bUseDQP = true;
  }
#endif
#if ENABLE_QPA
  if( getUsePerceptQPA() && !bUseDQP )
  {
    CHECK( m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!" );
    bUseDQP = (getBaseQP() < 38) && (getSourceWidth() > 512 || getSourceHeight() > 320);
  }
#endif

  if( m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING )
  {
    bUseDQP=false;
  }

  if( m_RCEnableRateControl )
  {
    picHeader.setCuQpDeltaSubdivIntra( 0 );
    picHeader.setCuQpDeltaSubdivInter( 0 );
  }
  else if( bUseDQP )
  {
    picHeader.setCuQpDeltaSubdivIntra( m_cuQpDeltaSubdiv );
    picHeader.setCuQpDeltaSubdivInter( m_cuQpDeltaSubdiv );
  }
  else
  {
    picHeader.setCuQpDeltaSubdivIntra( 0 );
    picHeader.setCuQpDeltaSubdivInter( 0 );
  }

  if( m_cuChromaQpOffsetSubdiv >= 0 )
  {
    picHeader.setCuChromaQpOffsetSubdivIntra(m_cuChromaQpOffsetSubdiv);
    picHeader.setCuChromaQpOffsetSubdivInter(m_cuChromaQpOffsetSubdiv);
  }
  else
  {
    picHeader.setCuChromaQpOffsetSubdivIntra(0);
    picHeader.setCuChromaQpOffsetSubdivInter(0);
  }


  // virtual boundaries
  if( sps.getVirtualBoundariesEnabledFlag() )
  {
    picHeader.setVirtualBoundariesPresentFlag( sps.getVirtualBoundariesPresentFlag() );
    picHeader.setNumVerVirtualBoundaries(sps.getNumVerVirtualBoundaries());
    picHeader.setNumHorVirtualBoundaries(sps.getNumHorVirtualBoundaries());
    for(i=0; i<3; i++) {
      picHeader.setVirtualBoundariesPosX(sps.getVirtualBoundariesPosX(i), i);
      picHeader.setVirtualBoundariesPosY(sps.getVirtualBoundariesPosY(i), i);
    }
  }

#if JVET_Z0118_GDR
    picHeader.setGdrOrIrapPicFlag(false);    
#endif

  // gradual decoder refresh flag
  picHeader.setGdrPicFlag(false);

  // BDOF / DMVR / PROF
  picHeader.setDisBdofFlag(false);
  picHeader.setDisDmvrFlag(false);
  picHeader.setDisProfFlag(false);
#if JVET_W0097_GPM_MMVD_TM
  if (sps.getUseGeo())
  {
#if TOOLS
    if (getIntraPeriod() > 0)
    {
      if ((getSourceWidth() * getSourceHeight()) > (1920 * 1080))
      {
        picHeader.setGPMMMVDTableFlag(false);
      }
      else
      {
        picHeader.setGPMMMVDTableFlag(true);
      }
    }
    else
    {
      picHeader.setGPMMMVDTableFlag(true);
    }
#else
  picHeader.setGPMMMVDTableFlag(false);
#endif
  }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  picHeader.setDisFracMBVD(true);
  if (sps.getIBCFracFlag())
  {
    if ((getSourceWidth() * getSourceHeight()) <= (1920 * 1080))
    {
      picHeader.setDisFracMBVD(false);
    }
  }
#endif
}

void EncLib::xInitAPS(APS &aps)
{
  //Do nothing now
}

void EncLib::xInitRPL(SPS &sps, bool isFieldCoding)
{
  ReferencePictureList*      rpl;

  int numRPLCandidates = getRPLCandidateSize(0);
  // To allocate one additional memory for RPL of POC1 (first bottom field) which is not specified in cfg file
  sps.createRPLList0(numRPLCandidates + (isFieldCoding ? 1 : 0));
  sps.createRPLList1(numRPLCandidates + (isFieldCoding ? 1 : 0));
  RPLList* rplList = 0;

  for (int i = 0; i < 2; i++)
  {
    rplList = (i == 0) ? sps.getRPLList0() : sps.getRPLList1();
    for (int j = 0; j < numRPLCandidates; j++)
    {
      const RPLEntry &ge = getRPLEntry(i, j);
      rpl = rplList->getReferencePictureList(j);
      rpl->setNumberOfShorttermPictures(ge.m_numRefPics);
      rpl->setNumberOfLongtermPictures(0);   //Hardcoded as 0 for now. need to update this when implementing LTRP
      rpl->setNumberOfActivePictures(ge.m_numRefPicsActive);
      rpl->setLtrpInSliceHeaderFlag(ge.m_ltrp_in_slice_header_flag);
      rpl->setInterLayerPresentFlag( sps.getInterLayerPresentFlag() );
      // inter-layer reference picture is not signaled in SPS RPL, SPS is shared currently
      rpl->setNumberOfInterLayerPictures( 0 );

      for (int k = 0; k < ge.m_numRefPics; k++)
      {
#if JVET_S0045_SIGN
        rpl->setRefPicIdentifier(k, -ge.m_deltaRefPics[k], 0, false, 0);
#else
        rpl->setRefPicIdentifier( k, ge.m_deltaRefPics[k], 0, false, 0 );
#endif
      }
    }
  }

  if (isFieldCoding)
  {
    // To set RPL of POC1 (first bottom field) which is not specified in cfg file
    for (int i = 0; i < 2; i++)
    {
      rplList = (i == 0) ? sps.getRPLList0() : sps.getRPLList1();
      rpl = rplList->getReferencePictureList(numRPLCandidates);
      rpl->setNumberOfShorttermPictures(1);
      rpl->setNumberOfLongtermPictures(0);
      rpl->setNumberOfActivePictures(1);
      rpl->setLtrpInSliceHeaderFlag(0);
#if JVET_S0045_SIGN
      rpl->setRefPicIdentifier(0, -1, 0, false, 0);
#else
      rpl->setRefPicIdentifier(0, 1, 0, false, 0);
#endif
      rpl->setPOC(0, 0);
    }
  }

  bool isRpl1CopiedFromRpl0 = true;
  for( int i = 0; isRpl1CopiedFromRpl0 && i < numRPLCandidates; i++)
  {
    if( sps.getRPLList0()->getReferencePictureList(i)->getNumRefEntries() == sps.getRPLList1()->getReferencePictureList(i)->getNumRefEntries() )
    {
      for( int j = 0; isRpl1CopiedFromRpl0 && j < sps.getRPLList0()->getReferencePictureList(i)->getNumRefEntries(); j++ )
      {
        if( sps.getRPLList0()->getReferencePictureList(i)->getRefPicIdentifier(j) != sps.getRPLList1()->getReferencePictureList(i)->getRefPicIdentifier(j) )
        {
          isRpl1CopiedFromRpl0 = false;
        }
      }
    }
    else
    {
      isRpl1CopiedFromRpl0 = false;
    }
  }
  sps.setRPL1CopyFromRPL0Flag(isRpl1CopiedFromRpl0);

  //Check if all delta POC of STRP in each RPL has the same sign
  //Check RPLL0 first
  const RPLList* rplList0 = sps.getRPLList0();
  const RPLList* rplList1 = sps.getRPLList1();
  uint32_t numberOfRPL = sps.getNumRPL0();

  bool isAllEntriesinRPLHasSameSignFlag = true;
  bool isFirstEntry = true;
  bool lastSign = true;        //true = positive ; false = negative
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && ii < numberOfRPL; ii++)
  {
    const ReferencePictureList* rpl = rplList0->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  //Check RPLL1. Skip it if it is already found out that this flag is not true for RPL0 or if RPL1 is the same as RPL0
  numberOfRPL = sps.getNumRPL1();
  isFirstEntry = true;
  lastSign = true;
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && !sps.getRPL1CopyFromRPL0Flag() && ii < numberOfRPL; ii++)
  {
    isFirstEntry = true;
    const ReferencePictureList* rpl = rplList1->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->getNumberOfActivePictures(); jj++)
    {
      if (!rpl->isRefPicLongterm(jj) && isFirstEntry)
      {
        lastSign = (rpl->getRefPicIdentifier(jj) >= 0) ? true : false;
        isFirstEntry = false;
      }
      else if (!rpl->isRefPicLongterm(jj) && (((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) >= 0 && lastSign == false) || ((rpl->getRefPicIdentifier(jj) - rpl->getRefPicIdentifier(jj - 1)) < 0 && lastSign == true)))
      {
        isAllEntriesinRPLHasSameSignFlag = false;
      }
    }
  }
  sps.setAllActiveRplEntriesHasSameSignFlag(isAllEntriesinRPLHasSameSignFlag);
}

void EncLib::getActiveRefPicListNumForPOC(const SPS *sps, int POCCurr, int GOPid, uint32_t *activeL0, uint32_t *activeL1)
{
  if (m_intraPeriod < 0)  //Only for RA
  {
    *activeL0 = *activeL1 = 0;
    return;
  }
  uint32_t rpl0Idx = GOPid;
  uint32_t rpl1Idx = GOPid;

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;
  if (m_intraPeriod < 0)
  {
    if (POCCurr < (2 * m_iGOPSize + 2))
    {
      int candidateIdx = (POCCurr + m_iGOPSize - 1 >= fullListNum + partialListNum) ? GOPid : POCCurr + m_iGOPSize - 1;
      rpl0Idx = candidateIdx;
      rpl1Idx = candidateIdx;
    }
    else
    {
      rpl0Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
      rpl1Idx = (POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1;
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum<fullListNum + partialListNum; extraNum++)
  {
    if (m_intraPeriod > 0 && getDecodingRefreshType() > 0)
    {
      int POCIndex = POCCurr % m_intraPeriod;
      if (POCIndex == 0)
      {
        POCIndex = m_intraPeriod;
      }
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        rpl0Idx = extraNum;
        rpl1Idx = extraNum;
        extraNum++;
      }
    }
  }

  const ReferencePictureList *rpl0 = sps->getRPLList0()->getReferencePictureList(rpl0Idx);
  *activeL0 = rpl0->getNumberOfActivePictures();
  const ReferencePictureList *rpl1 = sps->getRPLList1()->getReferencePictureList(rpl1Idx);
  *activeL1 = rpl1->getNumberOfActivePictures();
}

void EncLib::selectReferencePictureList(Slice* slice, int POCCurr, int GOPid, int ltPoc)
{
  bool isEncodeLtRef = (POCCurr == ltPoc);
  if (m_compositeRefEnabled && isEncodeLtRef)
  {
    POCCurr++;
  }

  slice->setRPL0idx(GOPid);
  slice->setRPL1idx(GOPid);

  int fullListNum = m_iGOPSize;
  int partialListNum = getRPLCandidateSize(0) - m_iGOPSize;
  int extraNum = fullListNum;

  int rplPeriod = m_intraPeriod;
  if( rplPeriod < 0 )  //Need to check if it is low delay or RA but with no RAP
  {
    if( slice->getSPS()->getRPLList0()->getReferencePictureList(1)->getRefPicIdentifier(0) * slice->getSPS()->getRPLList1()->getReferencePictureList(1)->getRefPicIdentifier(0) < 0)
    {
      rplPeriod = m_iGOPSize * 2;
    }
  }

  if (rplPeriod < 0)
  {
    if (POCCurr < (2 * m_iGOPSize + 2))
    {
      int candidateIdx = (POCCurr + m_iGOPSize - 1 >= fullListNum + partialListNum) ? GOPid : POCCurr + m_iGOPSize - 1;
      slice->setRPL0idx(candidateIdx);
      slice->setRPL1idx(candidateIdx);
    }
    else
    {
      slice->setRPL0idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
      slice->setRPL1idx((POCCurr%m_iGOPSize == 0) ? m_iGOPSize - 1 : POCCurr%m_iGOPSize - 1);
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum < fullListNum + partialListNum; extraNum++)
  {
    if( rplPeriod > 0 )
    {
      int POCIndex = POCCurr % rplPeriod;
      if (POCIndex == 0)
      {
        POCIndex = rplPeriod;
      }
      if (POCIndex == m_RPLList0[extraNum].m_POC)
      {
        slice->setRPL0idx(extraNum);
        slice->setRPL1idx(extraNum);
        extraNum++;
      }
    }
  }

  if (slice->getPic()->fieldPic)
  {
    // To set RPL index of POC1 (first bottom field)
    if (POCCurr == 1)
    {
      slice->setRPL0idx(getRPLCandidateSize(0));
      slice->setRPL1idx(getRPLCandidateSize(0));
    }
    else if( rplPeriod < 0 )
    {
      // To set RPL indexes for LD
      int numRPLCandidates = getRPLCandidateSize(0);
      if (POCCurr < numRPLCandidates - m_iGOPSize + 2)
      {
        slice->setRPL0idx(POCCurr + m_iGOPSize - 2);
        slice->setRPL1idx(POCCurr + m_iGOPSize - 2);
      }
      else
      {
        if (POCCurr%m_iGOPSize == 0)
        {
          slice->setRPL0idx(m_iGOPSize - 2);
          slice->setRPL1idx(m_iGOPSize - 2);
        }
        else if (POCCurr%m_iGOPSize == 1)
        {
          slice->setRPL0idx(m_iGOPSize - 1);
          slice->setRPL1idx(m_iGOPSize - 1);
        }
        else
        {
          slice->setRPL0idx(POCCurr % m_iGOPSize - 2);
          slice->setRPL1idx(POCCurr % m_iGOPSize - 2);
        }
      }
    }
  }

  const ReferencePictureList *rpl0 = (slice->getSPS()->getRPLList0()->getReferencePictureList(slice->getRPL0idx()));
  const ReferencePictureList *rpl1 = (slice->getSPS()->getRPLList1()->getReferencePictureList(slice->getRPL1idx()));
  slice->setRPL0(rpl0);
  slice->setRPL1(rpl1);
}


void EncLib::setParamSetChanged(int spsId, int ppsId)
{
  m_ppsMap.setChangedFlag(ppsId);
  m_spsMap.setChangedFlag(spsId);
}
bool EncLib::APSNeedsWriting(int apsId)
{
  bool isChanged = m_apsMap.getChangedFlag(apsId);
  m_apsMap.clearChangedFlag(apsId);
  return isChanged;
}

bool EncLib::PPSNeedsWriting(int ppsId)
{
  bool bChanged=m_ppsMap.getChangedFlag(ppsId);
  m_ppsMap.clearChangedFlag(ppsId);
  return bChanged;
}

bool EncLib::SPSNeedsWriting(int spsId)
{
  bool bChanged=m_spsMap.getChangedFlag(spsId);
  m_spsMap.clearChangedFlag(spsId);
  return bChanged;
}

void EncLib::checkPltStats( Picture* pic )
{
  int totalArea = 0;
  int pltArea = 0;
  for (auto apu : pic->cs->pus)
  {
    for (int i = 0; i < MAX_NUM_TBLOCKS; ++i)
    {
      int puArea = apu->blocks[i].width * apu->blocks[i].height;
      if (apu->blocks[i].width > 0 && apu->blocks[i].height > 0)
      {
        totalArea += puArea;
        if (CU::isPLT(*apu->cu) || CU::isIBC(*apu->cu))
        {
          pltArea += puArea;
        }
        break;
      }

    }
  }
  if (pltArea * PLT_FAST_RATIO < totalArea)
  {
    m_doPlt = false;
  }
  else
  {
    m_doPlt = true;
  }
}

#if X0038_LAMBDA_FROM_QP_CAPABILITY
int EncCfg::getQPForPicture(const uint32_t gopIndex, const Slice *pSlice) const
{
  const int lumaQpBDOffset = pSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
  int qp;

  if (getCostMode()==COST_LOSSLESS_CODING)
  {
    qp = getBaseQP();
  }
  else
  {
    const SliceType sliceType=pSlice->getSliceType();

    qp = getBaseQP();

    // switch at specific qp and keep this qp offset
    static int appliedSwitchDQQ = 0; /* TODO: MT */
    if( pSlice->getPOC() == getSwitchPOC() )
    {
      appliedSwitchDQQ = getSwitchDQP();
    }
    qp += appliedSwitchDQQ;

#if QP_SWITCHING_FOR_PARALLEL
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[pSlice->getPOC() / (m_compositeRefEnabled ? 2 : 1)];
    }
#endif

    if(sliceType==I_SLICE)
    {
      qp += getIntraQPOffset();
    }
    else
    {
        const GOPEntry &gopEntry=getGOPEntry(gopIndex);
        // adjust QP according to the QP offset for the GOP entry.
        qp +=gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset=qp*gopEntry.m_QPOffsetModelScale+gopEntry.m_QPOffsetModelOffset+0.5;
        int qpOffset = (int)floor(Clip3<double>(0.0, 3.0, dqpOffset));
        qp += qpOffset ;
      }

#if JVET_AC0096
    if (m_rprFunctionalityTestingEnabledFlag)
    {
      int currPoc = pSlice->getPOC() + EncCfg::m_FrameSkip;
      int rprSegment = EncCfg::getRprSwitchingSegment(currPoc);
      qp += EncCfg::m_rprSwitchingQPOffsetOrderList[rprSegment];
  }
#endif
#if JVET_AG0116
    if (m_gopBasedRPREnabledFlag)
    {
      if (pSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR)
      {
        qp += EncCfg::m_qpOffsetRPR;
      }
      if (pSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR2)
      {
        qp += EncCfg::m_qpOffsetRPR2;
      }
      if (pSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR3)
      {
        qp += EncCfg::m_qpOffsetRPR3;
      }
  }
#endif
#if !QP_SWITCHING_FOR_PARALLEL
    // modify QP if a fractional QP was originally specified, cause dQPs to be 0 or 1.
    const int* pdQPs = getdQPs();
    if ( pdQPs )
    {
      qp += pdQPs[ pSlice->getPOC() ];
    }
#endif
  }
  qp = Clip3( -lumaQpBDOffset, MAX_QP, qp );
  return qp;
}
#endif

#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
void EncLib::deriveHorCollocatedChroma(PelStorage* pcPicYuvOrg)
{
  if (m_horCollocatedChromaFlag < 0)
  {
    CHECK(m_chromaFormatIDC != CHROMA_420, "HorCollocatedChroma error");
    m_horCollocatedChromaFlag = m_cIntraSearch.calcHorCollocatedChroma(pcPicYuvOrg, m_bitDepth[CH_C], &m_cRdCost);
    SPS &sps0 = *(m_spsMap.allocatePS( m_vps->getGeneralLayerIdx( m_layerId ) ));
    sps0.setHorCollocatedChromaFlag(m_horCollocatedChromaFlag);
  }
}
#endif

//! \}
