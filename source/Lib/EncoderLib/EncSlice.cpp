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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#include <math.h>

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
 : m_encCABACTableIdx(I_SLICE)
#if ENABLE_QPA
 , m_adaptedLumaQP(-1)
#endif
{
}

EncSlice::~EncSlice()
{
  destroy();
}

void EncSlice::create( int iWidth, int iHeight, ChromaFormat chromaFormat, uint32_t iMaxCUWidth, uint32_t iMaxCUHeight, uint8_t uhTotalDepth )
{
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  int numBinBuffers = iWidth / iMaxCUWidth + 1;
  
  for ( int i = 0; i < numBinBuffers; i++ )
  {
    m_binVectors.push_back( BinStoreVector() );
    m_binVectors[i].reserve( CABAC_SPATIAL_MAX_BINS );
  }
#endif
}

void EncSlice::destroy()
{
  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  m_binVectors.clear();
#endif
}

void EncSlice::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcCfg             = pcEncLib;
  m_pcLib             = pcEncLib;
  m_pcListPic         = pcEncLib->getListPic();

  m_pcGOPEncoder      = pcEncLib->getGOPEncoder();
  m_pcCuEncoder       = pcEncLib->getCuEncoder();
  m_pcInterSearch     = pcEncLib->getInterSearch();
  m_CABACWriter       = pcEncLib->getCABACEncoder()->getCABACWriter   (&sps);
  m_CABACEstimator    = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_pcTrQuant         = pcEncLib->getTrQuant();
  m_pcRdCost          = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_vdRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_viRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncLib->getRateCtrl();
}

void
EncSlice::setUpLambda( Slice* slice, const double dLambda, int iQP)
{
  m_pcRdCost->resetStore();
  m_pcTrQuant->resetStore();
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for( uint32_t compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    int chromaQPOffset       = slice->getPPS()->getQpOffset( compID ) + slice->getSliceChromaQpDelta( compID );
    int qpc = slice->getSPS()->getMappedChromaQpValue(compID, iQP) + chromaQPOffset;
    double tmpWeight         = pow( 2.0, ( iQP - qpc ) / 3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
#if TCQ_8STATES
    if( slice->getDepQuantEnabledIdc() )
#else
    if( slice->getDepQuantEnabledFlag() )
#endif
    {
      tmpWeight *= ( m_pcCfg->getGOPSize() >= 8 ? pow( 2.0, 0.1/3.0 ) : pow( 2.0, 0.2/3.0 ) );  // increase chroma weight for dependent quantization (in order to reduce bit rate shift from chroma to luma)
    }
    m_pcRdCost->setDistortionWeight( compID, tmpWeight );
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
  // for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

  // for SAO
  slice->setLambdas( dLambdas );
}

#if ENABLE_QPA

static inline int apprI3Log2 (const double d) // rounded 3*log2(d)
{
  return d < 1.5e-13 ? -128 : int (floor (3.0 * log (d) / log (2.0) + 0.5));
}

static inline int lumaDQPOffset (const uint32_t avgLumaValue, const int bitDepth)
{
  return (1 - int ((3 * uint64_t (avgLumaValue * avgLumaValue)) >> uint64_t (2 * bitDepth - 1)));
}

static void filterAndCalculateAverageEnergies (const Pel* pSrc, const int  iSrcStride,
                                               double &hpEner,  const int  iHeight,    const int iWidth,
                                               const uint32_t uBitDepth /* luma bit-depth (4-16) */)
{
  uint64_t saAct = 0;

  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  for (int y = 1; y < iHeight - 1; y++)
  {
    // skip column as there may be a black border frame

    for (int x = 1; x < iWidth - 1; x++) // and columns
    {
      const int f = 12 * (int)pSrc[x  ] - 2 * ((int)pSrc[x-1] + (int)pSrc[x+1] + (int)pSrc[x  -iSrcStride] + (int)pSrc[x  +iSrcStride])
                       - (int)pSrc[x-1-iSrcStride] - (int)pSrc[x+1-iSrcStride] - (int)pSrc[x-1+iSrcStride] - (int)pSrc[x+1+iSrcStride];
      saAct += abs (f);
    }
    // skip column as there may be a black border frame
    pSrc += iSrcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(saAct) / double((iWidth - 2) * (iHeight - 2));

  // lower limit, compensate for highpass amplification
  if (hpEner < double(1 << (uBitDepth - 4))) hpEner = double(1 << (uBitDepth - 4));
}

#ifndef GLOBAL_AVERAGING
  #define GLOBAL_AVERAGING 1 // "global" averaging of a_k across a set instead of one picture
#endif

#if GLOBAL_AVERAGING
static double getAveragePictureEnergy (const CPelBuf picOrig, const uint32_t uBitDepth)
{
  const double hpEnerPic = 16.0 * sqrt ((3840.0 * 2160.0) / double(picOrig.width * picOrig.height)) * double(1 << uBitDepth);

  return sqrt (hpEnerPic); // square-root of a_pic value
}
#endif

static int getGlaringColorQPOffset (Picture* const pcPic, const int ctuAddr, Slice* const pcSlice,
                                    const int bitDepth,   uint32_t &avgLumaValue)
{
  const PreCalcValues& pcv  = *pcPic->cs->pcv;
  const ChromaFormat chrFmt = pcPic->chromaFormat;
  const uint32_t chrWidth   = pcv.maxCUWidth  >> getChannelTypeScaleX (CH_C, chrFmt);
  const uint32_t chrHeight  = pcv.maxCUHeight >> getChannelTypeScaleY (CH_C, chrFmt);
  const int      midLevel   = 1 << (bitDepth - 1);
  int chrValue = MAX_INT;
  avgLumaValue = (pcSlice != nullptr) ? 0 : (uint32_t)pcPic->getOrigBuf().Y().computeAvg();

  if (ctuAddr >= 0) // luma
  {
    avgLumaValue = (uint32_t)pcPic->m_iOffsetCtu[ctuAddr];
  }
  else if (pcSlice != nullptr)
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );
      avgLumaValue += pcPic->m_iOffsetCtu[ctuRsAddr];
    }
    avgLumaValue = (avgLumaValue + (pcSlice->getNumCtuInSlice() >> 1)) / pcSlice->getNumCtuInSlice();
  }

  for (uint32_t comp = COMPONENT_Cb; comp < MAX_NUM_COMPONENT; comp++)
  {
    const ComponentID compID = (ComponentID)comp;
    int avgCompValue;

    if (ctuAddr >= 0) // chroma
    {
      const CompArea chrArea = clipArea (CompArea (compID, chrFmt, Area ((ctuAddr % pcv.widthInCtus) * chrWidth, (ctuAddr / pcv.widthInCtus) * chrHeight, chrWidth, chrHeight)), pcPic->block (compID));

      avgCompValue = pcPic->getOrigBuf (chrArea).computeAvg();
    }
    else avgCompValue = pcPic->getOrigBuf (pcPic->block (compID)).computeAvg();

    if (chrValue > avgCompValue) chrValue = avgCompValue; // minimum of the DC offsets
  }
  CHECK (chrValue < 0, "DC offset cannot be negative!");

  chrValue = (int)avgLumaValue - chrValue;

  if (chrValue > midLevel) return apprI3Log2 (double (chrValue * chrValue) / double (midLevel * midLevel));

  return 0;
}

static int applyQPAdaptationChroma (Picture* const pcPic, Slice* const pcSlice, EncCfg* const pcEncCfg, const int sliceQP)
{
  const int bitDepth               = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA); // overall image bit-depth
  double hpEner[MAX_NUM_COMPONENT] = {0.0, 0.0, 0.0};
  int    optSliceChromaQpOffset[2] = {0, 0};
  int    savedLumaQP               = -1;
  uint32_t meanLuma                = MAX_UINT;

  for (uint32_t comp = 0; comp < getNumberValidComponents (pcPic->chromaFormat); comp++)
  {
    const ComponentID compID = (ComponentID)comp;
    const CPelBuf    picOrig = pcPic->getOrigBuf (pcPic->block (compID));

    filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, hpEner[comp],
                                       picOrig.height, picOrig.width,  bitDepth - (isChroma (compID) ? 1 : 0));
    if (isChroma (compID))
    {
      const int  adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);

      if (savedLumaQP < 0)
      {
#if GLOBAL_AVERAGING
        int     averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), bitDepth)));
#else
        int     averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP); // mean slice QP
#endif

        averageAdaptedLumaQP += getGlaringColorQPOffset (pcPic, -1 /*ctuRsAddr*/, nullptr /*pcSlice*/, bitDepth, meanLuma);

        if (averageAdaptedLumaQP > MAX_QP
#if SHARP_LUMA_DELTA_QP
            && (pcEncCfg->getLumaLevelToDeltaQPMapping().mode != LUMALVL_TO_DQP_NUM_MODES)
#endif
            ) averageAdaptedLumaQP = MAX_QP;
#if SHARP_LUMA_DELTA_QP

        // change mean picture QP index based on picture's average luma value (Sharp)
        if (pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES)
        {
          if (meanLuma == MAX_UINT) meanLuma = pcPic->getOrigBuf().Y().computeAvg();

          averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
        }
#endif

        savedLumaQP = averageAdaptedLumaQP;
      } // savedLumaQP < 0

      const int lumaChromaMappingDQP = savedLumaQP - pcSlice->getSPS()->getMappedChromaQpValue(compID, savedLumaQP);

      optSliceChromaQpOffset[comp-1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
    }
  }

  pcEncCfg->setSliceChromaOffsetQpIntraOrPeriodic (pcEncCfg->getSliceChromaOffsetQpPeriodicity(), optSliceChromaQpOffset);

  return savedLumaQP;
}

#endif // ENABLE_QPA

/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */
void EncSlice::initEncSlice(Picture* pcPic, const int pocLast, const int pocCurr, const int iGOPid, Slice*& rpcSlice, const bool isField,
                            bool isEncodeLtRef, int layerId)
{
  double dQP;
  double dLambda;
  PicHeader *picHeader = pcPic->cs->picHeader;
  pcPic->cs->resetPrevPLT(pcPic->cs->prevPLT);

  rpcSlice = pcPic->slices[0];
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->setPicHeader( picHeader );
  rpcSlice->initSlice();
  rpcSlice->setNalUnitLayerId(layerId);

  int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;
  if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
  {
    picHeader->setPicOutputFlag(false);
  }
  else
  {
    picHeader->setPicOutputFlag(true);
  }
  rpcSlice->setPOC( pocCurr );

  if( m_pcCfg->getCostMode() != COST_LOSSLESS_CODING )
  {
#if TCQ_8STATES
    rpcSlice->setDepQuantEnabledIdc( m_pcCfg->getDepQuantEnabledIdc() );
#else    
    rpcSlice->setDepQuantEnabledFlag( m_pcCfg->getDepQuantEnabledFlag() );
#endif

    rpcSlice->setSignDataHidingEnabledFlag( m_pcCfg->getSignDataHidingEnabledFlag() );
    rpcSlice->setTSResidualCodingDisabledFlag( false );

#if TCQ_8STATES
    CHECK( (m_pcCfg->getDepQuantEnabledIdc() || m_pcCfg->getSignDataHidingEnabledFlag() ) 
           && rpcSlice->getTSResidualCodingDisabledFlag() , "TSRC cannot be bypassed if either DQ or SDH are enabled at slice level.");
#else
    CHECK( (m_pcCfg->getDepQuantEnabledFlag() || m_pcCfg->getSignDataHidingEnabledFlag() ) 
           && rpcSlice->getTSResidualCodingDisabledFlag() , "TSRC cannot be bypassed if either DQ or SDH are enabled at slice level.");
#endif           
  }
  else
  {
#if TCQ_8STATES
    rpcSlice->setDepQuantEnabledIdc(0); //should be disabled for lossless
#else     
    rpcSlice->setDepQuantEnabledFlag( false ); //should be disabled for lossless
#endif

    rpcSlice->setSignDataHidingEnabledFlag( false ); //should be disabled for lossless
    if( m_pcCfg->getTSRCdisableLL() )
    {
      rpcSlice->setTSResidualCodingDisabledFlag( true );
    }
  }

#if INTER_LIC
  rpcSlice->setUseLIC(false);
#endif

#if SHARP_LUMA_DELTA_QP
  pcPic->fieldPic = isField;
  m_gopID = iGOPid;
#endif

  // depth computation based on GOP size
  int depth;
  {
    int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % (m_pcCfg->getGOPSize() * multipleFactor);
    }

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      int step = m_pcCfg->getGOPSize() * multipleFactor;
      depth    = 0;
      for( int i=step>>1; i>=1; i>>=1 )
      {
        for (int j = i; j<(m_pcCfg->getGOPSize() * multipleFactor); j += step)
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  const bool useIlRef = m_pcCfg->getAvoidIntraInDepLayer() && rpcSlice->getPic()->cs->vps && m_pcCfg->getNumRefLayers(rpcSlice->getPic()->cs->vps->getGeneralLayerIdx(layerId));
  if (m_pcCfg->getIntraPeriod() > 0 )
  {
    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || pocCurr % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) && (!useIlRef) ? I_SLICE : eSliceType;
#if JVET_Z0118_GDR
        if (m_pcCfg->getGdrEnabled() && (pocCurr >= m_pcCfg->getGdrPocStart()))
        {
          eSliceType = B_SLICE;
        }
#endif
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) && (!useIlRef) ? I_SLICE : eSliceType;
#if JVET_Z0118_GDR
        if (m_pcCfg->getGdrEnabled() && (pocCurr >= m_pcCfg->getGdrPocStart()))
        {
          eSliceType = B_SLICE;
        }
        else if (m_pcCfg->getGdrEnabled() && (pocCurr != 0) && (pocCurr < m_pcCfg->getGdrPocStart()))
        {
          eSliceType = B_SLICE;
        }
#endif
      }
    }
  }
  else
  {
    eSliceType = (pocLast == 0 || pocCurr == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
  }

  rpcSlice->setDepth        ( depth );
  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  pcPic->referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice);
#else
  dQP = m_pcCfg->getBaseQP();
  if(eSliceType!=I_SLICE)
  {
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  const int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif
#endif
  int iQP;
  double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
    // compute lambda value
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda (rpcSlice, iGOPid, dQP, dQP, iQP);
#else
    dLambda = initializeLambda (rpcSlice, iGOPid, int (dQP + 0.5), dQP);
    iQP = Clip3 (-rpcSlice->getSPS()->getQpBDOffset (CHANNEL_TYPE_LUMA), MAX_QP, int (dQP + 0.5));
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp    [iDQpIdx] = dQP;
    m_viRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp    [0];
  iQP     = m_viRdPicQp    [0];

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if W0038_CQP_ADJ
 #if ENABLE_QPA
  m_adaptedLumaQP = -1;

  if ((m_pcCfg->getUsePerceptQPA() || m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0) && !m_pcCfg->getUseRateCtrl() && rpcSlice->getPPS()->getSliceChromaQpFlag() &&
      (rpcSlice->isIntra() || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0)))
  {
    m_adaptedLumaQP = applyQPAdaptationChroma (pcPic, rpcSlice, m_pcCfg, iQP);
  }
 #endif
  if(rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    const bool bUseIntraOrPeriodicOffset = (rpcSlice->isIntra() && !rpcSlice->getUseIBC()) || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
#else
    const bool bUseIntraOrPeriodicOffset = (rpcSlice->isIntra() && !rpcSlice->getSPS()->getIBCFlag()) || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
#endif
    int cbQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
    int crQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true)  : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

#if JVET_AC0096
    if (m_pcCfg->getRprFunctionalityTestingEnabledFlag())
    {
      auto mappedQpDelta = [&](ComponentID c, int qpOffset) -> int {
        const int mappedQpBefore = rpcSlice->getSPS()->getMappedChromaQpValue(c, iQP - qpOffset);
        const int mappedQpAfter = rpcSlice->getSPS()->getMappedChromaQpValue(c, iQP);
        return mappedQpBefore - mappedQpAfter + qpOffset;
      };
      int currPoc = rpcSlice->getPOC() + m_pcCfg->getFrameSkip();
      int rprSegment = m_pcCfg->getRprSwitchingSegment(currPoc);
      cbQP += mappedQpDelta(COMPONENT_Cb, m_pcCfg->getRprSwitchingQPOffsetOrderList(rprSegment));
      crQP += mappedQpDelta(COMPONENT_Cr, m_pcCfg->getRprSwitchingQPOffsetOrderList(rprSegment));
    }
#endif
#if JVET_AG0116
    // adjust chroma QP such that it corresponds to the luma QP change when encoding in reduced resolution
    if (m_pcCfg->getGOPBasedRPREnabledFlag())
    {
      auto mappedQpDelta = [&](ComponentID c, int qpOffset) -> int {
        const int mappedQpBefore = rpcSlice->getSPS()->getMappedChromaQpValue(c, iQP - qpOffset);
        const int mappedQpAfter = rpcSlice->getSPS()->getMappedChromaQpValue(c, iQP);
        return mappedQpBefore - mappedQpAfter + qpOffset;
      };
      if (rpcSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR) // ScalingRatioHor/ScalingRatioVer
      {
        cbQP += mappedQpDelta(COMPONENT_Cb, m_pcCfg->getQpOffsetChromaRPR());
        crQP += mappedQpDelta(COMPONENT_Cr, m_pcCfg->getQpOffsetChromaRPR());
      }
      else if (rpcSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR2) // ScalingRatioHor2/ScalingRatioVer2
      {
        cbQP += mappedQpDelta(COMPONENT_Cb, m_pcCfg->getQpOffsetChromaRPR2());
        crQP += mappedQpDelta(COMPONENT_Cr, m_pcCfg->getQpOffsetChromaRPR2());
      }
      else if (rpcSlice->getPPS()->getPPSId() == ENC_PPS_ID_RPR3) // ScalingRatioHor3/ScalingRatioVer3
      {
        cbQP += mappedQpDelta(COMPONENT_Cb, m_pcCfg->getQpOffsetChromaRPR3());
        crQP += mappedQpDelta(COMPONENT_Cr, m_pcCfg->getQpOffsetChromaRPR3());
      }
    }
#endif
    int cbCrQP = (cbQP + crQP) >> 1; // use floor of average chroma QP offset for joint-Cb/Cr coding

    cbQP = Clip3( -12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
    crQP = Clip3( -12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3( -12, 12, cbQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)>=-12), "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3( -12, 12, crQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)>=-12), "Unspecified error");
    if (rpcSlice->getSPS()->getJointCbCrEnabledFlag())
    {
      cbCrQP = Clip3(-12, 12, cbCrQP + rpcSlice->getPPS()->getQpOffset(JOINT_CbCr)) - rpcSlice->getPPS()->getQpOffset(JOINT_CbCr);
      rpcSlice->setSliceChromaQpDelta(JOINT_CbCr, Clip3( -12, 12, cbCrQP ));
    }
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setSliceChromaQpDelta( JOINT_CbCr, 0 );
  }
#endif

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  double lambdaModifier;
  if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }

  dLambda *= lambdaModifier;
#endif

#if RDOQ_CHROMA_LAMBDA
  m_pcRdCost->setDistortionWeight (COMPONENT_Y, 1.0); // no chroma weighting for luma
#endif
  setUpLambda(rpcSlice, dLambda, iQP);

#if WCG_EXT
  // cost = Distortion + Lambda*R,
  // when QP is adjusted by luma, distortion is changed, so we have to adjust lambda to match the distortion, then the cost function becomes
  // costA = Distortion + AdjustedLambda * R          -- currently, costA is still used when calculating intermediate cost of using SAD, HAD, resisual etc.
  // an alternative way is to weight the distortion to before the luma QP adjustment, then the cost function becomes
  // costB = weightedDistortion + Lambda * R          -- currently, costB is used to calculat final cost, and when DF_FUNC is DF_DEFAULT
  m_pcRdCost->saveUnadjustedLambda();
#endif

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())
  {
    // restore original slice type

    if (m_pcCfg->getIntraPeriod() > 0 )
    {
      if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
          eSliceType = (pocLast == 0 || pocCurr % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) && (!useIlRef) ? I_SLICE : eSliceType;
#if JVET_Z0118_GDR
          if (m_pcCfg->getGdrEnabled() && (pocCurr >= m_pcCfg->getGdrPocStart()))
          {
            eSliceType = B_SLICE;
          }
          else if (m_pcCfg->getGdrEnabled() && (pocCurr != 0) && (pocCurr < m_pcCfg->getGdrPocStart()))
          {
            eSliceType = B_SLICE;
          }
          else if (m_pcCfg->getGdrEnabled() && (pocCurr != 0) && (pocCurr < m_pcCfg->getGdrPocStart()))
          {
            eSliceType = B_SLICE;
          }
#endif
        }
        else
        {
          eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) && (!useIlRef) ? I_SLICE : eSliceType;
#if JVET_Z0118_GDR
          if (m_pcCfg->getGdrEnabled() && (pocCurr >= m_pcCfg->getGdrPocStart()))
          {
            eSliceType = B_SLICE;
          }
          else if (m_pcCfg->getGdrEnabled() && (pocCurr != 0) && (pocCurr < m_pcCfg->getGdrPocStart()))
          {
            eSliceType = B_SLICE;
          }
#endif
        }
      }
    }
    else
    {
      eSliceType = (pocLast == 0 || pocCurr == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }

    rpcSlice->setSliceType        ( eSliceType );
  }

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = Clip3( -rpcSlice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );
  }

  rpcSlice->setSliceQp           ( iQP );
  rpcSlice->setSliceQpDelta      ( 0 );
  pcPic->setLossyQPValue(iQP);
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  rpcSlice->setSliceChromaQpDelta( JOINT_CbCr,   0 );
#endif
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getCuChromaQpOffsetListEnabledFlag() );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0, m_pcCfg->getRPLEntry(0, iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1, m_pcCfg->getRPLEntry(1, iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCbTcOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCrBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCrTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPPSDeblockingFilterDisabledFlag() );
#if DB_PARAM_TID
    const PPS* pcPPS = rpcSlice->getPPS();
    int betaIdx = Clip3(0, (int)pcPPS->getDeblockingFilterBetaOffsetDiv2().size() - 1, (int)rpcSlice->getTLayer() + (rpcSlice->isIntra() ? 0 : 1));
    int tcIdx = Clip3(0, (int)pcPPS->getDeblockingFilterTcOffsetDiv2().size() - 1, (int)rpcSlice->getTLayer() + (rpcSlice->isIntra() ? 0 : 1));
#endif
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)
      {
#if DB_PARAM_TID
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()[betaIdx]  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset()[tcIdx] );

        if( rpcSlice->getPPS()->getPPSChromaToolFlag() )
        {
          rpcSlice->setDeblockingFilterCbBetaOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CbBetaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()[betaIdx]);
          rpcSlice->setDeblockingFilterCbTcOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CbTcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset()[tcIdx]);
          rpcSlice->setDeblockingFilterCrBetaOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CrBetaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()[betaIdx]);
          rpcSlice->setDeblockingFilterCrTcOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CrTcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset()[tcIdx]);
        }
        else
        {
          rpcSlice->setDeblockingFilterCbBetaOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CbBetaOffsetDiv2 );
          rpcSlice->setDeblockingFilterCbTcOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CbTcOffsetDiv2 );
          rpcSlice->setDeblockingFilterCrBetaOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CrBetaOffsetDiv2 );
          rpcSlice->setDeblockingFilterCrTcOffsetDiv2(m_pcCfg->getGOPEntry(iGOPid).m_CrTcOffsetDiv2 );
        }
#else
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
        if( rpcSlice->getPPS()->getPPSChromaToolFlag() )
        {
          rpcSlice->setDeblockingFilterCbBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_CbBetaOffsetDiv2 + m_pcCfg->getLoopFilterCbBetaOffset() );
          rpcSlice->setDeblockingFilterCbTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_CbTcOffsetDiv2 + m_pcCfg->getLoopFilterCbTcOffset() );
          rpcSlice->setDeblockingFilterCrBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_CrBetaOffsetDiv2 + m_pcCfg->getLoopFilterCrBetaOffset() );
          rpcSlice->setDeblockingFilterCrTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_CrTcOffsetDiv2 + m_pcCfg->getLoopFilterCrTcOffset() );
        }
        else
        {
          rpcSlice->setDeblockingFilterCbBetaOffsetDiv2( rpcSlice->getDeblockingFilterBetaOffsetDiv2() );
          rpcSlice->setDeblockingFilterCbTcOffsetDiv2( rpcSlice->getDeblockingFilterTcOffsetDiv2() );
          rpcSlice->setDeblockingFilterCrBetaOffsetDiv2( rpcSlice->getDeblockingFilterBetaOffsetDiv2() );
          rpcSlice->setDeblockingFilterCrTcOffsetDiv2( rpcSlice->getDeblockingFilterTcOffsetDiv2() );
        }
#endif
      }
      else
      {
#if DB_PARAM_TID
        rpcSlice->setDeblockingFilterBetaOffsetDiv2(m_pcCfg->getLoopFilterBetaOffset()[betaIdx]);
        rpcSlice->setDeblockingFilterTcOffsetDiv2(m_pcCfg->getLoopFilterTcOffset()[tcIdx]);
        rpcSlice->setDeblockingFilterCbBetaOffsetDiv2(m_pcCfg->getLoopFilterBetaOffset()[betaIdx]);
        rpcSlice->setDeblockingFilterCbTcOffsetDiv2(m_pcCfg->getLoopFilterTcOffset()[tcIdx]);
        rpcSlice->setDeblockingFilterCrBetaOffsetDiv2(m_pcCfg->getLoopFilterBetaOffset()[betaIdx]);
        rpcSlice->setDeblockingFilterCrTcOffsetDiv2(m_pcCfg->getLoopFilterTcOffset()[tcIdx]);
#else
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
        rpcSlice->setDeblockingFilterCbBetaOffsetDiv2( m_pcCfg->getLoopFilterCbBetaOffset() );
        rpcSlice->setDeblockingFilterCbTcOffsetDiv2( m_pcCfg->getLoopFilterCbTcOffset() );
        rpcSlice->setDeblockingFilterCrBetaOffsetDiv2( m_pcCfg->getLoopFilterCrBetaOffset() );
        rpcSlice->setDeblockingFilterCrTcOffsetDiv2( m_pcCfg->getLoopFilterCrTcOffset() );
#endif
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCbBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCbTcOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCrBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterCrTcOffsetDiv2( 0 );
  }

  pcPic->temporalId =  temporalId;
  if(eSliceType==I_SLICE)
  {
    pcPic->temporalId = 0;
  }
  rpcSlice->setTLayer( pcPic->temporalId );

  rpcSlice->setDisableSATDForRD(false);

  if( eSliceType == I_SLICE || ( m_pcCfg->getIBCHashSearch() && m_pcCfg->getIBCMode() ) || ( eSliceType != I_SLICE && m_pcCfg->getAllowDisFracMMVD() ) )
  {
    m_pcCuEncoder->getIbcHashMap().destroy();
    m_pcCuEncoder->getIbcHashMap().init( pcPic->cs->pps->getPicWidthInLumaSamples(), pcPic->cs->pps->getPicHeightInLumaSamples() );
  }
#if JVET_Z0118_GDR
  if (m_pcCfg->getGdrEnabled())
  {
    int gdrPocStart = m_pcCuEncoder->getEncCfg()->getGdrPocStart();
    int gdrPeriod = m_pcCuEncoder->getEncCfg()->getGdrPeriod();
    int gdrInterval = m_pcCuEncoder->getEncCfg()->getGdrInterval();

    int picWidth = rpcSlice->getPPS()->getPicWidthInLumaSamples();

    int curPoc = rpcSlice->getPOC();
    int gdrPoc = (curPoc - gdrPocStart) % gdrPeriod;

    int  offset = (curPoc < gdrPocStart) ? 0 : (((curPoc - gdrPocStart) / gdrPeriod) * gdrPeriod);
    int  actualGdrStart = gdrPocStart + offset;
    int  actualGdrInterval = min(gdrInterval, (int)(pcPic->getPicWidthInLumaSamples() / 8));
    int  recoveryPocCnt = actualGdrInterval - 1;
    int  recoveryPicPoc = actualGdrStart + recoveryPocCnt;

    bool isInGdrInterval = (curPoc >= actualGdrStart) && (curPoc < recoveryPicPoc);
    bool isRecoveryPocPic = (curPoc == recoveryPicPoc);
    bool isOutGdrInterval = !(isInGdrInterval || isRecoveryPocPic);
    bool isGdrPic = (actualGdrStart == curPoc);

    pcPic->cs->picHeader->setGdrPicFlag(false);
    pcPic->cs->picHeader->setRecoveryPocCnt(0);
    pcPic->cs->picHeader->setInGdrInterval(false);
    pcPic->cs->picHeader->setIsGdrRecoveryPocPic(false);
    pcPic->cs->picHeader->setVirtualBoundariesPresentFlag(false);

#if GDR_ENC_TRACE
    printf("\n");
    printf("-poc:%d gdrPocStart:%d actualGdrStart:%d actualGdrInterval:%d actualGdrEndPoc:%d isInGdrInterval:%d isRecoveryPocPic:%d\n", rpcSlice->getPOC(), gdrPocStart, actualGdrStart, actualGdrInterval, recoveryPicPoc - 1, isInGdrInterval, isRecoveryPocPic);
#endif
    
    // for none gdr period pictures
    if ((curPoc < gdrPocStart) || isOutGdrInterval)
    {
      pcPic->cs->picHeader->setInGdrInterval(false);
      pcPic->cs->picHeader->setVirtualBoundariesPresentFlag(false);

      pcPic->cs->picHeader->setNumHorVirtualBoundaries(0);
      pcPic->cs->picHeader->setNumVerVirtualBoundaries(0);

#if GDR_ENC_TRACE
      printf("-poc:%d no virtual boundary\n", rpcSlice->getPOC());
#endif
    }
    // for gdr inteval pictures
    else
    {
      int gdrBegX;
      int gdrEndX;
      int m1, m2, n1;

      double dd = (picWidth / (double)gdrInterval);
      int mm = (int)((picWidth / (double)gdrInterval) + 0.49999);
      m1 = ((mm + 7) >> 3) << 3;
      m2 = ((mm + 0) >> 3) << 3;

      if (dd > mm && m1 == m2)
      {
        m1 = m1 + 8;
      }

      n1 = (picWidth - m2 * gdrInterval) / 8;

      if (gdrPoc < n1)
      {
        gdrBegX = m1 * gdrPoc;
        gdrEndX = gdrBegX + m1;
      }
      else
      {
        gdrBegX = m1 * n1 + m2 * (gdrPoc - n1);
        gdrEndX = gdrBegX + m2;
        if (picWidth <= gdrBegX)
        {
          gdrBegX = picWidth;
          gdrEndX = picWidth;
        }
      }

      pcPic->cs->picHeader->setGdrBegX(gdrBegX);
      pcPic->cs->picHeader->setGdrEndX(gdrEndX);

      if (isGdrPic)
      {
        pcPic->cs->picHeader->setGdrOrIrapPicFlag(true);
        pcPic->cs->picHeader->setGdrPicFlag(true);

        pcPic->cs->picHeader->setInGdrInterval(true);
        pcPic->cs->picHeader->setIsGdrRecoveryPocPic(false);

        pcPic->cs->picHeader->setVirtualBoundariesPresentFlag(true);
        pcPic->cs->picHeader->setNumHorVirtualBoundaries(0);
        pcPic->cs->picHeader->setNumVerVirtualBoundaries(1);
        pcPic->cs->picHeader->setVirtualBoundariesPosX(gdrEndX, 0);

        pcPic->cs->picHeader->setRecoveryPocCnt(recoveryPocCnt);
        m_pcGOPEncoder->setLastGdrIntervalPoc(recoveryPicPoc - 1);
      }      
      else if (isInGdrInterval)
      {
        pcPic->cs->picHeader->setGdrOrIrapPicFlag(false);

        pcPic->cs->picHeader->setInGdrInterval(true);
        pcPic->cs->picHeader->setIsGdrRecoveryPocPic(false);

        pcPic->cs->picHeader->setVirtualBoundariesPresentFlag(true);
        pcPic->cs->picHeader->setNumHorVirtualBoundaries(0);
        pcPic->cs->picHeader->setNumVerVirtualBoundaries(1);
        pcPic->cs->picHeader->setVirtualBoundariesPosX(gdrEndX, 0);
      }
      else if (isRecoveryPocPic)
      {
        pcPic->cs->picHeader->setGdrOrIrapPicFlag(false);

        pcPic->cs->picHeader->setInGdrInterval(false);
        pcPic->cs->picHeader->setIsGdrRecoveryPocPic(true);

        pcPic->cs->picHeader->setVirtualBoundariesPresentFlag(false);
        pcPic->cs->picHeader->setNumHorVirtualBoundaries(0);
        pcPic->cs->picHeader->setNumVerVirtualBoundaries(0);
#if GDR_ENC_TRACE
        printf("-poc:%d no virtual boundary\n", rpcSlice->getPOC());
#endif
      }
      
#if GDR_ENC_TRACE
      if (isGdrPic || isInGdrInterval)
      {        
        printf("-poc:%d beg:%d end:%d\n", rpcSlice->getPOC(), gdrBegX, gdrEndX);
      }
#endif
    }

#if JVET_Z0118_GDR
    pcPic->initCleanCurPicture();    
#endif

  }
#endif
#if JVET_AL0153_ALF_CCCM
  rpcSlice->lfCccmClearControlInformation();
#endif
}

double EncSlice::initializeLambda(const Slice* slice, const int GOPid, const int refQP, const double dQP)
{
  const int   bitDepthLuma  = slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  const int   bitDepthShift = 6 * (bitDepthLuma - 8 - DISTORTION_PRECISION_ADJUSTMENT(bitDepthLuma)) - 12;
  const int   numberBFrames = m_pcCfg->getGOPSize() - 1;
  const SliceType sliceType = slice->getSliceType();
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int      temporalId = m_pcCfg->getGOPEntry(GOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers = m_pcCfg->getIntraLambdaModifier();
#endif
  // case #1: I or P slices (key-frame)
  double dQPFactor = m_pcCfg->getGOPEntry(GOPid).m_QPFactor;
  double dLambda, lambdaModifier;

  if (sliceType == I_SLICE)
  {
    if ((m_pcCfg->getIntraQpFactor() >= 0.0) && (m_pcCfg->getGOPEntry(GOPid).m_sliceType != I_SLICE))
    {
      dQPFactor = m_pcCfg->getIntraQpFactor();
    }
    else
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if (m_pcCfg->getLambdaFromQPEnable())
      {
        dQPFactor = 0.57;
      }
      else
#endif
      dQPFactor = 0.57 * (1.0 - Clip3(0.0, 0.5, 0.05 * double (slice->getPic()->fieldPic ? numberBFrames >> 1 : numberBFrames)));
    }
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  else if (m_pcCfg->getLambdaFromQPEnable())
  {
    dQPFactor = 0.57;
  }
#endif

  dLambda = dQPFactor * pow(2.0, (dQP + bitDepthShift) / 3.0);

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if (slice->getDepth() > 0 && !m_pcCfg->getLambdaFromQPEnable())
#else
  if (slice->getDepth() > 0)
#endif
  {
    dLambda *= Clip3(2.0, 4.0, ((refQP + bitDepthShift) / 6.0));
  }
  // if Hadamard is used in motion estimation process
  if (!m_pcCfg->getUseHADME() && (sliceType != I_SLICE))
  {
    dLambda *= 0.95;
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if ((sliceType != I_SLICE) || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier(temporalId);
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[temporalId < intraLambdaModifiers.size() ? temporalId : intraLambdaModifiers.size() - 1];
  }
  dLambda *= lambdaModifier;
#endif

  return dLambda;
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
double EncSlice::calculateLambda( const Slice*     slice,
                                  const int        GOPid, // entry in the GOP table
                                  const double     refQP, // initial slice-level QP
                                  const double     dQP,   // initial double-precision QP
                                        int       &iQP )  // returned integer QP.
{
  double dLambda = initializeLambda (slice, GOPid, int (refQP + 0.5), dQP);
  iQP = Clip3 (-slice->getSPS()->getQpBDOffset (CHANNEL_TYPE_LUMA), MAX_QP, int (dQP + 0.5));

#if TCQ_8STATES
	if (slice->getDepQuantEnabledIdc())
#else
  if( slice->getDepQuantEnabledFlag() )
#endif
  {
    dLambda *= pow( 2.0, 0.25/3.0 ); // slight lambda adjustment for dependent quantization (due to different slope of quantizer)
  }

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

void EncSlice::resetQP( Picture* pic, int sliceQP, double lambda )
{
  Slice* slice = pic->slices[0];

  // store lambda
  slice->setSliceQp( sliceQP );
#if RDOQ_CHROMA_LAMBDA
  m_pcRdCost->setDistortionWeight (COMPONENT_Y, 1.0); // no chroma weighting for luma
#endif
  setUpLambda(slice, lambda, sliceQP);
#if WCG_EXT
  if (!m_pcCfg->getLumaLevelToDeltaQPMapping().isEnabled())
  {
    m_pcRdCost->saveUnadjustedLambda();
  }
#endif
}

#if ENABLE_QPA
static bool applyQPAdaptation (Picture* const pcPic,       Slice* const pcSlice,        const PreCalcValues& pcv,
                               const bool useSharpLumaDQP,
                               const bool useFrameWiseQPA, const int previouslyAdaptedLumaQP = -1)
{
  const int  bitDepth    = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA);
  const int  iQPIndex    = pcSlice->getSliceQp(); // initial QP index for current slice, used in following loops
  bool   sliceQPModified = false;
  uint32_t   meanLuma    = MAX_UINT;
  double     hpEnerAvg   = 0.0;

#if GLOBAL_AVERAGING
  if (!useFrameWiseQPA || previouslyAdaptedLumaQP < 0)  // mean visual activity value and luma value in each CTU
#endif
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );
      const Position pos ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);
      const CompArea ctuArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
      const CompArea fltArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
      const CPelBuf  picOrig    = pcPic->getOrigBuf (fltArea);
      double hpEner = 0.0;

      filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, hpEner,
                                         picOrig.height, picOrig.width,  bitDepth);
      hpEnerAvg += hpEner;
      pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
      pcPic->m_iOffsetCtu[ctuRsAddr] = pcPic->getOrigBuf (ctuArea).computeAvg();
    }

    hpEnerAvg /= double (pcSlice->getNumCtuInSlice());
  }
#if GLOBAL_AVERAGING
  const double hpEnerPic = 1.0 / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), bitDepth);  // inverse, speed
#else
  const double hpEnerPic = 1.0 / hpEnerAvg; // speedup: multiply instead of divide in loop below; 1.0 for tuning
#endif

  if (useFrameWiseQPA || (iQPIndex >= MAX_QP))
  {
    int iQPFixed = (previouslyAdaptedLumaQP < 0) ? Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (hpEnerAvg * hpEnerPic)) : previouslyAdaptedLumaQP;

    if (isChromaEnabled (pcPic->chromaFormat) && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      iQPFixed += getGlaringColorQPOffset (pcPic, -1 /*ctuRsAddr*/, pcSlice, bitDepth, meanLuma);

      if (iQPFixed > MAX_QP
#if SHARP_LUMA_DELTA_QP
          && !useSharpLumaDQP
#endif
          ) iQPFixed = MAX_QP;
    }
#if SHARP_LUMA_DELTA_QP

    // change new fixed QP based on average CTU luma value (Sharp)
    if (useSharpLumaDQP && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      if (meanLuma == MAX_UINT) // collect picture mean luma value
      {
        meanLuma = 0;

        for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
        {
          uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );

          meanLuma += pcPic->m_iOffsetCtu[ctuRsAddr];  // CTU mean
        }
        meanLuma = (meanLuma + (pcSlice->getNumCtuInSlice() >> 1)) / pcSlice->getNumCtuInSlice();
      }
      iQPFixed = Clip3 (0, MAX_QP, iQPFixed + lumaDQPOffset (meanLuma, bitDepth));
    }
#endif

    if (iQPIndex >= MAX_QP) iQPFixed = MAX_QP;
    else
    if (iQPFixed != iQPIndex)
    {
      const double* oldLambdas = pcSlice->getLambdas();
      const double  corrFactor = pow (2.0, double(iQPFixed - iQPIndex) / 3.0);
      const double  newLambdas[MAX_NUM_COMPONENT] = {oldLambdas[0] * corrFactor, oldLambdas[1] * corrFactor, oldLambdas[2] * corrFactor};

      CHECK (iQPIndex != pcSlice->getSliceQpBase(), "Invalid slice QP!");
      pcSlice->setLambdas (newLambdas);
      pcSlice->setSliceQp (iQPFixed); // update the slice/base QPs
      pcSlice->setSliceQpBase (iQPFixed);

      sliceQPModified = true;
    }

    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPFixed; // fixed QPs
    }
  }
  else // CTU-wise QPA
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );

      int iQPAdapt = Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (pcPic->m_uEnerHpCtu[ctuRsAddr] * hpEnerPic));

      if (pcv.widthInCtus > 1) // try to enforce CTU SNR greater than zero dB
      {
        meanLuma = (uint32_t)pcPic->m_iOffsetCtu[ctuRsAddr];

        if (isChromaEnabled (pcPic->chromaFormat))
        {
          iQPAdapt += getGlaringColorQPOffset (pcPic, (int)ctuRsAddr, nullptr, bitDepth, meanLuma);

          if (iQPAdapt > MAX_QP
#if SHARP_LUMA_DELTA_QP
              && !useSharpLumaDQP
#endif
              ) iQPAdapt = MAX_QP;
          CHECK (meanLuma != (uint32_t)pcPic->m_iOffsetCtu[ctuRsAddr], "luma DC offsets don't match");
        }
#if SHARP_LUMA_DELTA_QP

        // change adaptive QP based on mean CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
 #if ENABLE_QPA_SUB_CTU
          pcPic->m_uEnerHpCtu[ctuRsAddr] = (double)meanLuma; // for sub-CTU QPA
 #endif
          iQPAdapt = Clip3 (0, MAX_QP, iQPAdapt + lumaDQPOffset (meanLuma, bitDepth));
        }

#endif
        const uint32_t uRefScale  = g_invQuantScales[0][iQPAdapt % 6] << ((iQPAdapt / 6) + bitDepth - 4);
        const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
        const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
        const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
        const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
        const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
        uint32_t uAbsDCless = 0;

        // compute sum of absolute DC-less (high-pass) luma values
        for (SizeType h = 0; h < iSrcHeight; h++)
        {
          for (SizeType w = 0; w < iSrcWidth; w++)
          {
            uAbsDCless += (uint32_t)abs (pSrc[w] - (Pel)meanLuma);
          }
          pSrc += iSrcStride;
        }

        if (iSrcHeight >= 64 || iSrcWidth >= 64)  // normalization
        {
          const uint64_t blockSize = uint64_t(iSrcWidth * iSrcHeight);

          uAbsDCless = uint32_t((uint64_t(uAbsDCless) * 64*64 + (blockSize >> 1)) / blockSize);
        }

        if (uAbsDCless < 64*64) uAbsDCless = 64*64;  // limit to 1

        // reduce QP index if CTU would be fully quantized to zero
        if (uAbsDCless < uRefScale)
        {
          const int limit  = std::min (0, ((iQPIndex + 4) >> 3) - 6);
          const int redVal = std::max (limit, apprI3Log2 ((double)uAbsDCless / (double)uRefScale));

          iQPAdapt = std::max (0, iQPAdapt + redVal);
        }
      }

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt; // adapted QPs

#if ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->getCuQpDeltaSubdiv() == 0)  // reduce local DQP rate peaks
#elif ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->getPPS()->getMaxCuDQPDepth() == 0)  // reduce local DQP rate peaks
#else
      if (pcv.widthInCtus > 1) // try to reduce local bitrate peaks via minimum smoothing of the adapted QPs
#endif
      {
        iQPAdapt = ctuRsAddr % pcv.widthInCtus; // horizontal offset
        if (iQPAdapt == 0)
        {
          iQPAdapt = (ctuRsAddr > 1) ? pcPic->m_iOffsetCtu[ctuRsAddr - 2] : 0;
        }
        else // iQPAdapt >= 1
        {
          iQPAdapt = (iQPAdapt > 1) ? std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 2], pcPic->m_iOffsetCtu[ctuRsAddr]) : pcPic->m_iOffsetCtu[ctuRsAddr];
        }
        if (ctuRsAddr > pcv.widthInCtus)
        {
          iQPAdapt = std::min (iQPAdapt, (int)pcPic->m_iOffsetCtu[ctuRsAddr - 1 - pcv.widthInCtus]);
        }
        if ((ctuRsAddr > 0) && (pcPic->m_iOffsetCtu[ctuRsAddr - 1] < (Pel)iQPAdapt))
        {
          pcPic->m_iOffsetCtu[ctuRsAddr - 1] = (Pel)iQPAdapt;
        }
        if ((ctuIdx == pcSlice->getNumCtuInSlice() - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
        {
          iQPAdapt = std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 1], pcPic->m_iOffsetCtu[ctuRsAddr - pcv.widthInCtus]);
          if (pcPic->m_iOffsetCtu[ctuRsAddr] < (Pel)iQPAdapt)
          {
            pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;
          }
        }
      }
    } // end iteration over all CTUs in current slice
  }

  return sliceQPModified;
}

#if ENABLE_QPA_SUB_CTU
static int applyQPAdaptationSubCtu (CodingStructure &cs, const UnitArea ctuArea, const uint32_t ctuAddr, const bool useSharpLumaDQP)
{
  const PreCalcValues &pcv = *cs.pcv;
  const Picture     *pcPic = cs.picture;
  const int       bitDepth = cs.slice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA); // overall image bit-depth
  const int   adaptedCtuQP = pcPic ? pcPic->m_iOffsetCtu[ctuAddr] : cs.slice->getSliceQpBase();

  if (!pcPic || cs.slice->getCuQpDeltaSubdiv() == 0) return adaptedCtuQP;

  for (unsigned addr = 0; addr < cs.picture->m_subCtuQP.size(); addr++)
  {
    cs.picture->m_subCtuQP[addr] = (int8_t)adaptedCtuQP;
  }
  if (cs.slice->getSliceQp() < MAX_QP && pcv.widthInCtus > 1)
  {
#if SHARP_LUMA_DELTA_QP
    const int   lumaCtuDQP = useSharpLumaDQP ? lumaDQPOffset ((uint32_t)pcPic->m_uEnerHpCtu[ctuAddr], bitDepth) : 0;
#endif
    const unsigned     mts = std::min (cs.sps->getMaxTbSize(), pcv.maxCUWidth);
    const unsigned mtsLog2 = (unsigned)floorLog2(mts);
    const unsigned  stride = pcv.maxCUWidth >> mtsLog2;
    unsigned numAct = 0;    // number of block activities
    double   sumAct = 0.0; // sum of all block activities
    double   subAct[16];   // individual block activities
#if SHARP_LUMA_DELTA_QP
    uint32_t subMLV[16];   // individual mean luma values
#endif

    CHECK (mts * 4 < pcv.maxCUWidth || mts * 4 < pcv.maxCUHeight, "max. transform size is too small for given CTU size");

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr    = w + h * stride;
        const PosType  x       = ctuArea.lx() + w * mts;
        const PosType  y       = ctuArea.ly() + h * mts;
        const CompArea fltArea = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (x > 0 ? x - 1 : 0, y > 0 ? y - 1 : 0, mts + (x > 0 ? 2 : 1), mts + (y > 0 ? 2 : 1))), pcPic->Y());
        const CPelBuf  picOrig = pcPic->getOrigBuf (fltArea);

        if (x >= pcPic->lwidth() || y >= pcPic->lheight())
        {
          continue;
        }
        filterAndCalculateAverageEnergies (picOrig.buf,    picOrig.stride, subAct[addr],
                                           picOrig.height, picOrig.width,  bitDepth);
        numAct++;
        sumAct += subAct[addr];
#if SHARP_LUMA_DELTA_QP

        if (useSharpLumaDQP)
        {
          const CompArea subArea = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (x, y, mts, mts)), pcPic->Y());

          subMLV[addr] = pcPic->getOrigBuf (subArea).computeAvg();
        }
#endif
      }
    }
    if (sumAct <= 0.0) return adaptedCtuQP;

    sumAct = double(numAct) / sumAct; // 1.0 / (average CTU activity)

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr = w + h * stride;

        if (ctuArea.lx() + w * mts >= pcPic->lwidth() || ctuArea.ly() + h * mts >= pcPic->lheight())
        {
          continue;
        }
        cs.picture->m_subCtuQP[addr] = (int8_t)Clip3 (0, MAX_QP, adaptedCtuQP + apprI3Log2 (subAct[addr] * sumAct));
#if SHARP_LUMA_DELTA_QP

        // change adapted QP based on mean sub-CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
          cs.picture->m_subCtuQP[addr] = (int8_t)Clip3 (0, MAX_QP, (int)cs.picture->m_subCtuQP[addr] - lumaCtuDQP + lumaDQPOffset (subMLV[addr], bitDepth));
        }
#endif
      }
    }
  }

  return adaptedCtuQP;
}
#endif // ENABLE_QPA_SUB_CTU
#endif // ENABLE_QPA

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
void EncSlice::setSearchRange( Slice* pcSlice )
{
  int iCurrPOC = pcSlice->getPOC();
  int iRefPOC;
  int iGOPSize = m_pcCfg->getGOPSize();
  int iOffset = (iGOPSize >> 1);
  int iMaxSR = m_pcCfg->getSearchRange();
  int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      int newSearchRange = Clip3(m_pcCfg->getMinSearchWindow(), iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(iDir, iRefIdx, newSearchRange);
    }
  }
}

void EncSlice::setLosslessSlice(Picture* pcPic, bool islossless) 
{
  Slice* slice = pcPic->slices[getSliceSegmentIdx()];
  slice->setLossless(islossless);

  if (m_pcCfg->getCostMode() == COST_LOSSLESS_CODING)
  {
    if (islossless)
    {
      int losslessQp = LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP - ((slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8) * 6);
      slice->setSliceQp(losslessQp); // update the slice/base QPs

     slice->setTSResidualCodingDisabledFlag(m_pcCfg->getTSRCdisableLL() ? true : false);

    }
    else
    {
        slice->setSliceQp(pcPic->getLossyQPValue());
        slice->setTSResidualCodingDisabledFlag(false);
    }
  }
}


/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
void EncSlice::precompressSlice( Picture* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled." );
  }

  Slice* pcSlice        = pcPic->slices[getSliceSegmentIdx()];



  double     dPicRdCostBest = MAX_DOUBLE;
  uint32_t       uiQpIdxBest = 0;

  double dFrameLambda;
  int SHIFT_QP = 12
                 + 6
                     * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                        - DISTORTION_PRECISION_ADJUSTMENT(pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for ( uint32_t uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdx] );
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp    [uiQpIdx]);

    // try compress
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());

    uint64_t uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
#if W0038_DB_OPT
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pcPic );
#endif
    // compute RD cost and choose the best
    double dPicRdCost = double( uiPicDist ) + dFrameLambda * double( m_uiPicTotalBits );

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdxBest] );
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp    [uiQpIdxBest]);
}

void EncSlice::calCostSliceI(Picture* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  double         iSumHadSlice      = 0;
  Slice * const  pcSlice           = pcPic->slices[getSliceSegmentIdx()];
  const PreCalcValues& pcv         = *pcPic->cs->pcv;
  const SPS     &sps               = *(pcSlice->getSPS());
  const int      shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const int      offset            = (shift>0)?(1<<(shift-1)):0;


  for( uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++ )
  {
    uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );
    Position pos( (ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const int height  = std::min( pcv.maxCUHeight, pcv.lumaHeight - pos.y );
    const int width   = std::min( pcv.maxCUWidth,  pcv.lumaWidth  - pos.x );
    const CompArea blk( COMPONENT_Y, pcv.chrFormat, pos, Size( width, height));
    int iSumHad = m_pcCuEncoder->updateCtuDataISlice( pcPic->getOrigBuf( blk ) );

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

void EncSlice::calCostPictureI(Picture* picture)
{
  double         sumHadPicture = 0;
  Slice * const  slice = picture->slices[getSliceSegmentIdx()];
  const PreCalcValues& pcv = *picture->cs->pcv;
  const SPS     &sps = *(slice->getSPS());
  const int      shift = sps.getBitDepth(CHANNEL_TYPE_LUMA) - 8;
  const int      offset = (shift>0) ? (1 << (shift - 1)) : 0;

  for (uint32_t ctuIdx = 0; ctuIdx < picture->m_ctuNums; ctuIdx++)
  {
    Position pos((ctuIdx % pcv.widthInCtus) * pcv.maxCUWidth, (ctuIdx / pcv.widthInCtus) * pcv.maxCUHeight);

    const int height = std::min(pcv.maxCUHeight, pcv.lumaHeight - pos.y);
    const int width = std::min(pcv.maxCUWidth, pcv.lumaWidth - pos.x);
    const CompArea blk(COMPONENT_Y, pcv.chrFormat, pos, Size(width, height));
    int sumHad = m_pcCuEncoder->updateCtuDataISlice(picture->getOrigBuf(blk));

    (m_pcRateCtrl->getRCPic()->getLCU(ctuIdx)).m_costIntra = (sumHad + offset) >> shift;
    sumHadPicture += (m_pcRateCtrl->getRCPic()->getLCU(ctuIdx)).m_costIntra;
  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(sumHadPicture);
}

/** \param pcPic   picture class
 */
void EncSlice::compressSlice( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  Slice* const pcSlice    = pcPic->slices[getSliceSegmentIdx()];

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_uiPicDist       = 0;

  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT 
  m_CABACEstimator->m_CABACDataStore->updateBufferState( pcSlice );
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  clearAmvpSbTmvpStatArea(pcSlice);
#endif

  m_CABACEstimator->initCtxModels( *pcSlice );

#if ENABLE_SPLIT_PARALLELISM
  for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
  {
    CABACWriter* cw = m_pcLib->getCABACEncoder( jId )->getCABACEstimator( pcSlice->getSPS() );
    cw->initCtxModels( *pcSlice );
  }

#endif
  m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);


  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  const bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if(pcSlice->getSPS()->getIBCFlag())
  {
    auto sliceType = pcSlice->getSliceType();

    if(sliceType == I_SLICE)
    {
      pcSlice->setUseIBC(pcSlice->getSPS()->getIBCFlag());
    }
    else
    {
      pcSlice->setUseIBC(pcSlice->getSPS()->getIBCFlagInterSlice());
    }
  }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pcSlice->getUseIBC())
#else
  if (pcSlice->getSPS()->getIBCFlag())
#endif
  {
    bool mode = m_pcCfg->getIbcBiPred();
    pcSlice->setBiPredictionIBCFlag(mode);
  }
  else
  {
    pcSlice->setBiPredictionIBCFlag(false);
  }
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pcSlice->getPicHeader()->getEnableTMVPFlag() && !pcSlice->isIntra())
  {
    int minPoc = abs(pcSlice->getRefPic(RefPicList(1 - pcSlice->getColFromL0Flag()), pcSlice->getColRefIdx())->getPOC() - pcSlice->getPOC());
    if (pcSlice->isInterB() && pcSlice->getCheckLDC())
    {
      int min2ndPoc = abs(pcSlice->getRefPic(RefPicList(1 - pcSlice->getColFromL0Flag2nd()), pcSlice->getColRefIdx2nd())->getPOC() - pcSlice->getPOC());
      minPoc = min(minPoc, min2ndPoc);
    }
    if (minPoc > 4)
    {
      pcSlice->setAmvpSbTmvpEnabledFlag(false);
      pcSlice->setAmvpSbTmvpAmvrEnabledFlag(false);
    }
    else
    {
      pcSlice->setAmvpSbTmvpEnabledFlag(true);

      g_picAmvpSbTmvpEnabledArea = 0;
      uint32_t prevEnabledArea;
      bool isExist = loadAmvpSbTmvpStatArea(pcSlice->getTLayer(), prevEnabledArea);
      if (isExist)
      {
        int ratio = int(prevEnabledArea * 100.0 / (pcSlice->getPic()->getPicWidthInLumaSamples() * pcSlice->getPic()->getPicHeightInLumaSamples()));
        if (ratio < 4)
        {
          pcSlice->setAmvpSbTmvpNumOffset(1);
        }
        else if (ratio < 7)
        {
          pcSlice->setAmvpSbTmvpNumOffset(2);
        }
        else
        {
          pcSlice->setAmvpSbTmvpNumOffset(3);
        }
      }
      else
      {
        pcSlice->setAmvpSbTmvpNumOffset(2);
      }
      if (pcSlice->isInterB() && pcSlice->getCheckLDC())
      {
        if (pcSlice->getRefPic(RefPicList(1 - pcSlice->getColFromL0Flag()), pcSlice->getColRefIdx())->getPOC() == pcSlice->getRefPic(RefPicList(1 - pcSlice->getColFromL0Flag2nd()), pcSlice->getColRefIdx2nd())->getPOC())
        {
          pcSlice->setAmvpSbTmvpNumColPic(1);
        }
        else
        {
          pcSlice->setAmvpSbTmvpNumColPic(2);
        }
      }
      else
      {
        pcSlice->setAmvpSbTmvpNumColPic(1);
      }
      pcSlice->setAmvpSbTmvpAmvrEnabledFlag(pcSlice->getPic()->getPicWidthInLumaSamples() * pcSlice->getPic()->getPicHeightInLumaSamples() < 3840*2160 ? false : true);
    }
  }
  else
  {
    pcSlice->setAmvpSbTmvpEnabledFlag(false);
  }
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  if (!pcSlice->isIntra())
  {
    int picH = pcSlice->getPic()->getPicHeightInLumaSamples();
    pcSlice->setExtAmvpLevel(picH >= 2160 ? 3 : (picH >= 1080 ? 2 : (picH >= 720 ? 1 : 0)));
  }
#endif

  if ( bWp_explicit )
  {

    xEstimateWPParamSlice( pcSlice, m_pcCfg->getWeightedPredictionMethod() );
    pcSlice->initWpScaling(pcSlice->getSPS());

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }

#if INTER_LIC
  pcSlice->setUseLICOnPicLevel(m_pcCfg->getFastPicLevelLIC());
#endif

    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();

  CHECK( pcPic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  CodingStructure&  cs          = *pcPic->cs;
  cs.slice    = pcSlice;
  cs.pcv      = pcSlice->getPPS()->pcv;
  cs.fracBits = 0;

  if( pcSlice->getFirstCtuRsAddrInSlice() == 0 && ( pcSlice->getPOC() != m_pcCfg->getSwitchPOC() || -1 == m_pcCfg->getDebugCTU() ) )
  {
    cs.initStructData (pcSlice->getSliceQp());
  }

#if ENABLE_QPA
  if (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl())
  {
    if (applyQPAdaptation (pcPic, pcSlice, *cs.pcv, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES,
                           (m_pcCfg->getBaseQP() >= 38) || (m_pcCfg->getSourceWidth() <= 512 && m_pcCfg->getSourceHeight() <= 320), m_adaptedLumaQP))
    {
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT 
      m_CABACEstimator->m_CABACDataStore->updateBufferState( pcSlice );
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
      clearAmvpSbTmvpStatArea(pcSlice);
#endif
      m_CABACEstimator->initCtxModels (*pcSlice);
#if ENABLE_SPLIT_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        CABACWriter* cw = m_pcLib->getCABACEncoder (jId)->getCABACEstimator (pcSlice->getSPS());
        cw->initCtxModels (*pcSlice);
      }
#endif
        pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
      if (pcSlice->getFirstCtuRsAddrInSlice() == 0)
      {
        cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
      }
    }
  }
#endif // ENABLE_QPA

  bool checkPLTRatio = m_pcCfg->getIntraPeriod() != 1 && pcSlice->isIRAP();
  if (checkPLTRatio)
  {
    m_pcCuEncoder->getModeCtrl()->setPltEnc(true);
  }
  else
  {
    bool doPlt = m_pcLib->getPltEnc();
    m_pcCuEncoder->getModeCtrl()->setPltEnc(doPlt);
  }

#if K0149_BLOCK_STATISTICS
  const SPS *sps = pcSlice->getSPS();
  CHECK(sps == 0, "No SPS present");
  writeBlockStatisticsHeader(sps);
#endif
  m_pcInterSearch->resetAffineMVList();
  m_pcInterSearch->resetUniMvList();
  ::memset(g_isReusedUniMVsFilled, 0, sizeof(g_isReusedUniMVsFilled));
#if INTER_LIC
  if (pcSlice->getUseLIC())
  {
    ::memset(g_isReusedUniMVsFilledLIC, 0, sizeof(g_isReusedUniMVsFilledLIC));
  }
#endif
  encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, m_pcLib );
  if (checkPLTRatio) m_pcLib->checkPltStats( pcPic );
}

void EncSlice::checkDisFracMmvd( Picture* pcPic, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr )
{
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t    widthInCtus   = pcv.widthInCtus;
  const uint32_t hashThreshold    = 20;
  uint32_t totalCtu               = 0;
  uint32_t hashRatio              = 0;

  if ( !pcSlice->getSPS()->getFpelMmvdEnabledFlag() )
  {
    return;
  }

  for ( uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++ )
  {
    const uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos ( ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight );
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );

    hashRatio += m_pcCuEncoder->getIbcHashMap().getHashHitRatio( ctuArea.Y() );
    totalCtu++;
  }

  if ( hashRatio > totalCtu * hashThreshold )
  {
    pcPic->cs->picHeader->setDisFracMMVD( true );
  }
  if (!pcPic->cs->picHeader->getDisFracMMVD()) {
    bool useIntegerMVD = (pcPic->lwidth()*pcPic->lheight() > 1920 * 1080);
    pcPic->cs->picHeader->setDisFracMMVD( useIntegerMVD );
  }
}


void EncSlice::setJointCbCrModes( CodingStructure& cs, const Position topLeftLuma, const Size sizeLuma )
{
  bool              sgnFlag = true;

  if( isChromaEnabled( cs.picture->chromaFormat) )
  {
    const CompArea  cbArea  = CompArea( COMPONENT_Cb, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CompArea  crArea  = CompArea( COMPONENT_Cr, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CPelBuf   orgCb   = cs.picture->getOrigBuf( cbArea );
    const CPelBuf   orgCr   = cs.picture->getOrigBuf( crArea );
    const int       x0      = ( cbArea.x > 0 ? 0 : 1 );
    const int       y0      = ( cbArea.y > 0 ? 0 : 1 );
    const int       x1      = ( cbArea.x + cbArea.width  < cs.picture->Cb().width  ? cbArea.width  : cbArea.width  - 1 );
    const int       y1      = ( cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1 );
    const int       cbs     = orgCb.stride;
    const int       crs     = orgCr.stride;
    const Pel*      pCb     = orgCb.buf + y0 * cbs;
    const Pel*      pCr     = orgCr.buf + y0 * crs;
    int64_t         sumCbCr = 0;

    // determine inter-chroma transform sign from correlation between high-pass filtered (i.e., zero-mean) Cb and Cr planes
    for( int y = y0; y < y1; y++, pCb += cbs, pCr += crs )
    {
      for( int x = x0; x < x1; x++ )
      {
        int cb = ( 12*(int)pCb[x] - 2*((int)pCb[x-1] + (int)pCb[x+1] + (int)pCb[x-cbs] + (int)pCb[x+cbs]) - ((int)pCb[x-1-cbs] + (int)pCb[x+1-cbs] + (int)pCb[x-1+cbs] + (int)pCb[x+1+cbs]) );
        int cr = ( 12*(int)pCr[x] - 2*((int)pCr[x-1] + (int)pCr[x+1] + (int)pCr[x-crs] + (int)pCr[x+crs]) - ((int)pCr[x-1-crs] + (int)pCr[x+1-crs] + (int)pCr[x-1+crs] + (int)pCr[x+1+crs]) );
        sumCbCr += cb*cr;
      }
    }

    sgnFlag = ( sumCbCr < 0 );
  }

  cs.picHeader->setJointCbCrSignFlag( sgnFlag );
}


void EncSlice::encodeCtus( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP, EncLib* pEncLib )
{
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t        widthInCtus   = pcv.widthInCtus;
#if ENABLE_QPA
  const int iQPIndex              = pcSlice->getSliceQpBase();
#endif
#if JVET_AG0145_ADAPTIVE_CLIPPING
  pcPic->calcLumaClpParams();
#endif

#if ENABLE_SPLIT_PARALLELISM
  const int       dataId          = 0;
#endif
  CABACWriter*    pCABACWriter    = pEncLib->getCABACEncoder( PARL_PARAM0( dataId ) )->getCABACEstimator( pcSlice->getSPS() );
  TrQuant*        pTrQuant        = pEncLib->getTrQuant( PARL_PARAM0( dataId ) );
  RdCost*         pRdCost         = pEncLib->getRdCost( PARL_PARAM0( dataId ) );
  EncCfg*         pCfg            = pEncLib;
  RateCtrl*       pRateCtrl       = pEncLib->getRateCtrl();
  pRdCost->setLosslessRDCost(pcSlice->isLossless());
#if RDOQ_CHROMA_LAMBDA
  pTrQuant    ->setLambdas( pcSlice->getLambdas() );
#else
  pTrQuant    ->setLambda ( pcSlice->getLambdas()[0] );
#endif
  pRdCost     ->setLambda ( pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths() );
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS && ENABLE_QPA
  if (!pCfg->getWCGChromaQPControl().isEnabled() && pCfg->getUsePerceptQPA() && !pCfg->getUseRateCtrl())
  {
    pRdCost->saveUnadjustedLambda();
  }
#endif

  int prevQP[2];
  int currQP[2];
  prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
  currQP[0] = currQP[1] = pcSlice->getSliceQp();

#if (JVET_AC0335_CONTENT_ADAPTIVE_OBMC_ENABLING && ENABLE_OBMC) || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS || JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0 || JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  int hashBlkHitPerc = -1;
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if( pcSlice->isIntra() || ( !pcSlice->isIntra() && pcSlice->getSPS()->getFpelMmvdEnabledFlag() ) || ( pcSlice->getUseIBC() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch() ) )
#else
  if( pcSlice->isIntra() || ( !pcSlice->isIntra() && pcSlice->getSPS()->getFpelMmvdEnabledFlag() ) || ( pcSlice->getSPS()->getIBCFlag() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch() ) )
#endif
  {
#if JVET_AA0070_RRIBC
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap(cs.picture->getTrueOrigBuf(), CS::isDualITree(cs) || cs.slice->isIntra());
#else
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap(cs.picture->getTrueOrigBuf(), CS::isDualITree(cs));
#endif
#else
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap(cs.picture->getTrueOrigBuf());
#endif
    if (m_pcCfg->getIntraPeriod() != -1)
    {
#if (JVET_AC0335_CONTENT_ADAPTIVE_OBMC_ENABLING && ENABLE_OBMC) || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      hashBlkHitPerc = m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y());
#else
      int hashBlkHitPerc = m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y());
#endif
      cs.slice->setDisableSATDForRD(hashBlkHitPerc > 59);
    }
  }

#if JVET_AC0335_CONTENT_ADAPTIVE_OBMC_ENABLING && ENABLE_OBMC
  if (m_pcCuEncoder->getEncCfg()->getUseOBMC())
  {
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
    {
      SPS* spsTmp = const_cast<SPS*>(cs.sps);
      hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      bool hashScc = hashBlkHitPerc < 57;
      spsTmp->setUseOBMC(hashScc);
#if JVET_Z0061_TM_OBMC
      if( m_pcLib->getUseOBMCTMMode() )
      {
        spsTmp->setUseOBMCTMMode( hashScc );
      }
#endif
    }
  }
#endif

#if JVET_AE0159_FIBC
  if (m_pcCuEncoder->getEncCfg()->getIbcFilter())
  {
    if( cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE ) // ensure sequential and parallel simulation generate same output
    {
      SPS* spsTmp = const_cast< SPS* >( cs.sps );
      hashBlkHitPerc = ( hashBlkHitPerc == -1 ) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc( cs.area.Y() ) : hashBlkHitPerc;
      bool isSCC = hashBlkHitPerc >= 20;
      spsTmp->setUseIbcFilter( isSCC );
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
      if( m_pcCuEncoder->getEncCfg()->getUseGeo() && m_pcCuEncoder->getEncCfg()->getTMToolsEnableFlag() )
      {
        spsTmp->setUseGeoBlend( !isSCC );
#if JVET_AK0101_REGRESSION_GPM_INTRA
        spsTmp->setUseGeoBlendIntra( !isSCC );
#endif
      }
#endif
#if JVET_AG0164_AFFINE_GPM
      if( m_pcCuEncoder->getEncCfg()->getMaxNumGpmAffCand() > 0 && isSCC )
      {
        spsTmp->setMaxNumGpmAffCand( m_pcCuEncoder->getEncCfg()->getMaxNumGpmAffCand() );
      }
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
      const int pictureArea = m_pcCuEncoder->getEncCfg()->getSourceWidth() * m_pcCuEncoder->getEncCfg()->getSourceHeight();

      if( pictureArea > 832 * 480 && m_pcCuEncoder->getEncCfg()->getBaseQP() > 22 )
      {
        spsTmp->setUseSyntheticAffine( !isSCC );
      }

      if( pictureArea < 3840 * 2160 && m_pcCuEncoder->getEncCfg()->getBaseQP() > 22 )
      {
        spsTmp->setUseTemporalAffineOpt( !m_pcCuEncoder->getEncCfg()->getPLTMode() );
      }
#endif
    }
  }
#endif

#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  if (m_pcCuEncoder->getEncCfg()->getUseInterCcpMerge())
  {
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
    {
      hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      SPS* spsTmp = const_cast<SPS*>(cs.sps);
      spsTmp->setUseInterCcpMergeZeroLumaCbf(hashBlkHitPerc > 40);
    }
  }
#endif

#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
  {
    hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
    SPS* spsTmp = const_cast<SPS*>(cs.sps);
    spsTmp->setUseLargeIBCLSR(hashBlkHitPerc > 56);
  }
#endif

#if JVET_AH0209_PDP
  if( m_pcCuEncoder->getEncCfg()->getUsePDP() )
  {
    if( cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE ) // ensure sequential and parallel simulation generate same output
    {
      SPS* spsTmp = const_cast<SPS*>( cs.sps );
      hashBlkHitPerc = ( hashBlkHitPerc == -1 ) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc( cs.area.Y() ) : hashBlkHitPerc;
      bool isSCC = hashBlkHitPerc >= 20;
      spsTmp->setUsePDP( !isSCC );
    }
  }
#endif

#if JVET_AJ0097_BDOF_LDB
  if (m_pcCuEncoder->getEncCfg()->getBIO())
  {
    hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
    bool isSCC = hashBlkHitPerc >= 10;

    int curPoc = cs.slice->getPOC();
    bool lowdelayRefExist = false;
    int n0 = pcSlice->getNumRefIdx(REF_PIC_LIST_0);
    int n1 = pcSlice->getNumRefIdx(REF_PIC_LIST_1);

    for (int idx0 = 0; idx0 < n0; idx0++)
    {
      int poc0 = pcSlice->getRefPOC(REF_PIC_LIST_0, idx0);

      for (int idx1 = 0; idx1 < n1; idx1++)
      {
        int poc1 = pcSlice->getRefPOC(REF_PIC_LIST_1, idx1);
        if ((curPoc - poc0) * (curPoc - poc1) > 0)
        {
          lowdelayRefExist = true;
          break;
        }
      }
    }
    cs.picHeader->setDisBdofFlag(isSCC && lowdelayRefExist);
  }
#endif

#if JVET_AI0082_GPM_WITH_INTER_IBC
  if (m_pcCuEncoder->getEncCfg()->getUseGeoInterIbc())
  {
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE)
    {
      SPS* spsTmp = const_cast<SPS*>(cs.sps);
      hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      bool isSCC = hashBlkHitPerc >= 20;
      spsTmp->setUseGeoInterIbc(isSCC);
    }
  }
#endif
#if JVET_AD0188_CCP_MERGE
  if ((pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU()))
  {
#if JVET_Z0118_GDR
    cs.ccpLut.lutCCP0.resize(0);
    cs.ccpLut.lutCCP1.resize(0);
#else
    cs.ccpLut.lutCCP.resize(0);
#endif
  }
#endif
#if JVET_AG0058_EIP
  if ((pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU()))
  {
#if JVET_Z0118_GDR
    cs.eipLut.lutEip0.resize(0);
    cs.eipLut.lutEip1.resize(0);
#else
    cs.eipLut.lutEip.resize(0);
#endif
  }
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  if( m_pcCuEncoder->getEncCfg()->getUseIntraPredBf() )
  {
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
    {
      SPS* spsTmp = const_cast<SPS*>(cs.sps);
      hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      bool isScreenContent = hashBlkHitPerc >= 20;
      spsTmp->setUseIntraPredBf( !isScreenContent );
    }
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pcSlice->getUseIBC() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch() && m_pcCuEncoder->getEncCfg()->getIBCFracMode())
  {
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
    {
      hashBlkHitPerc = hashBlkHitPerc == -1 ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      bool isSCC = hashBlkHitPerc >= 60;
      cs.picHeader->setDisFracMBVD(isSCC);
    }
  }
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  if (m_pcCuEncoder->getEncCfg()->getIntraToolControlMode() == 0)
  {
    SPS* spsTmp = const_cast<SPS*>(cs.sps);
    spsTmp->setDisableRefFilter(true);
    spsTmp->setDisablePdpc(true);
    spsTmp->setDisableIntraFusion(true);
  }
  else if (m_pcCuEncoder->getEncCfg()->getIntraToolControlMode() == 1)
  {
    SPS* spsTmp = const_cast<SPS*>(cs.sps);
    spsTmp->setDisableRefFilter(false);
    spsTmp->setDisablePdpc(false);
    spsTmp->setDisableIntraFusion(false);
  }
  else if (m_pcCuEncoder->getEncCfg()->getIntraToolControlMode() == 2)
  {
    SPS* spsTmp = const_cast<SPS*>(cs.sps);    
    if (cs.slice->getPOC() == 0 || cs.slice->getSliceType() == I_SLICE) // ensure sequential and parallel simulation generate same output
    {
      hashBlkHitPerc = (hashBlkHitPerc == -1) ? m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y()) : hashBlkHitPerc;
      bool isSCC = hashBlkHitPerc >= 20;
      spsTmp->setDisableRefFilter(isSCC);
      spsTmp->setDisablePdpc(isSCC);
      spsTmp->setDisableIntraFusion(isSCC);
    }
  }  
#endif

  // for every CTU in the slice
  for( uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++ )
  {
    const int32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );

#if JVET_AJ0226_MTT_SKIP 
    m_pcCuEncoder->getModeCtrl()->resetSplitSignalCostParams();
#endif
    // update CABAC state
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    if( pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU() )
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if ((cs.slice->getSliceType() != I_SLICE || cs.slice->getUseIBC()) && cs.pps->ctuIsTileColBd( ctuXPosInCtus ))
#else
    if ((cs.slice->getSliceType() != I_SLICE || cs.sps->getIBCFlag()) && cs.pps->ctuIsTileColBd( ctuXPosInCtus ))
#endif
    {
#if JVET_Z0118_GDR
      cs.motionLut.lut0.resize(0);      
      cs.motionLut.lutIbc0.resize(0);
      if (pCfg->getGdrEnabled())
      {
        cs.motionLut.lut1.resize(0);
        cs.motionLut.lutIbc1.resize(0);
      }
#else
      cs.motionLut.lut.resize(0);
      cs.motionLut.lutIbc.resize(0);
#endif

#if JVET_Z0139_HIST_AFF
      for (int i = 0; i < 2 * MAX_NUM_AFFHMVP_ENTRIES_ONELIST; i++)
      {
#if JVET_Z0118_GDR
        cs.motionLut.lutAff0[i].resize(0);
        if (pCfg->getGdrEnabled())
        {
          cs.motionLut.lutAff1[i].resize(0);
        }
#else
          cs.motionLut.lutAff[i].resize(0);
#endif
        }
#if JVET_Z0118_GDR
        cs.motionLut.lutAffInherit0.resize(0);
        if (pCfg->getGdrEnabled())
        {
          cs.motionLut.lutAffInherit1.resize(0);
        }
#else
        cs.motionLut.lutAffInherit.resize(0);
#endif
#endif
    }

#if JVET_AD0188_CCP_MERGE
    if ((pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU()) && cs.pps->ctuIsTileColBd(ctuXPosInCtus))
    {
#if JVET_Z0118_GDR
      cs.ccpLut.lutCCP0.resize(0);
      cs.ccpLut.lutCCP1.resize(0);
#else
      cs.ccpLut.lutCCP.resize(0);
#endif
    }
#endif
#if JVET_AG0058_EIP
    if ((pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU()) && cs.pps->ctuIsTileColBd(ctuXPosInCtus))
    {
#if JVET_Z0118_GDR
      cs.eipLut.lutEip0.resize(0);
      cs.eipLut.lutEip1.resize(0);
#else
      cs.eipLut.lutEip.resize(0);
#endif
    }
#endif
    const SubPic &curSubPic = pcSlice->getPPS()->getSubPicFromPos(pos);
    // padding/restore at slice level
    if (pcSlice->getPPS()->getNumSubPics() >= 2 && curSubPic.getTreatedAsPicFlag() && ctuIdx == 0)
    {
      int subPicX = (int)curSubPic.getSubPicLeft();
      int subPicY = (int)curSubPic.getSubPicTop();
      int subPicWidth = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();

      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
      {
        int n = pcSlice->getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = pcSlice->getRefPic((RefPicList)rlist, idx);

#if JVET_S0258_SUBPIC_CONSTRAINTS
          if( !refPic->getSubPicSaved() && refPic->subPictures.size() > 1 )
#else
          if (!refPic->getSubPicSaved() && refPic->numSubpics > 1)
#endif
          {
            refPic->saveSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->extendSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->setSubPicSaved(true);
          }
        }
      }
    }
    if (cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && cs.pps->ctuIsTileRowBd( ctuYPosInCtus ))
    {
      pCABACWriter->initCtxModels( *pcSlice );
      cs.resetPrevPLT(cs.prevPLT);
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
    else if (cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && pEncLib->getEntropyCodingSyncEnabledFlag())
    {
      // reset and then update contexts to the state at the end of the top CTU (if within current slice and tile).
      pCABACWriter->initCtxModels( *pcSlice );
      cs.resetPrevPLT(cs.prevPLT);
      if( cs.getCURestricted( pos.offset(0, -1), pos, pcSlice->getIndependentSliceIdx(), cs.pps->getTileIdx( pos ), CH_L ) )
      {
        // Top is available, we use it.
        pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
        cs.setPrevPLT(pEncLib->m_palettePredictorSyncState);
      }
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }


#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    double oldLambdaArray[MAX_NUM_COMPONENT] = {0.0};
#endif
    const double oldLambda = pRdCost->getLambda();
    if ( pCfg->getUseRateCtrl() )
    {
      int estQP        = pcSlice->getSliceQp();
      double estLambda = -1.0;
      double bpp       = -1.0;

      if( ( pcPic->slices[0]->isIRAP() && pCfg->getForceIntraQP() ) || !pCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->isIRAP());
        if ( pcPic->slices[0]->isIntra())
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = pRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());
#if WCG_EXT
        pRdCost->saveUnadjustedLambda();
#endif

#if RDOQ_CHROMA_LAMBDA
        const double lambdaArray[MAX_NUM_COMPONENT] = {estLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Y),
                                                       estLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cb),
                                                       estLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cr)};
        pTrQuant->setLambdas( lambdaArray );
#else
        pTrQuant->setLambda( estLambda );
#endif
      }

      pRateCtrl->setRCQP( estQP );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if ENABLE_QPA_SUB_CTU
      const int adaptedQP    = applyQPAdaptationSubCtu (cs, ctuArea, ctuRsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES);
#else
      const int adaptedQP    = pcPic->m_iOffsetCtu[ctuRsAddr];
#endif
      const double newLambda = pcSlice->getLambdas()[0] * pow (2.0, double (adaptedQP - iQPIndex) / 3.0);
      pcPic->m_uEnerHpCtu[ctuRsAddr] = newLambda; // for ALF and SAO
#if !ENABLE_QPA_SUB_CTU
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->getLambdas (oldLambdaArray); // save the old lambdas
      const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Y),
                                                     newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cb),
                                                     newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cr)};
      pTrQuant->setLambdas (lambdaArray);
#else
      pTrQuant->setLambda (newLambda);
#endif
      pRdCost->setLambda (newLambda, pcSlice->getSPS()->getBitDepths());
#endif
      currQP[0] = currQP[1] = adaptedQP;
    }
#endif

    bool updateBcwCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuIdx == 0;
    if( updateBcwCodingOrder )
    {
      resetBcwCodingOrder(false, cs);
      m_pcInterSearch->initWeightIdxBits();
    }
#if !JVET_V0094_BILATERAL_FILTER && !JVET_X0071_CHROMA_BILATERAL_FILTER
    if (pcSlice->getSPS()->getUseLmcs())
#endif
    {
      m_pcCuEncoder->setDecCuReshaperInEncCU(m_pcLib->getReshaper(), pcSlice->getSPS()->getChromaFormatIdc());

#if ENABLE_SPLIT_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        m_pcLib->getCuEncoder(jId)->setDecCuReshaperInEncCU(m_pcLib->getReshaper(jId), pcSlice->getSPS()->getChromaFormatIdc());
      }
#endif
    }
    if( !cs.slice->isIntra() && pCfg->getMCTSEncConstraint() )
    {
      pcPic->mctsInfo.init( &cs, ctuRsAddr );
    }

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Update the CABAC states based on CTU above
    if ( ctuYPosInCtus )
    {
      pCABACWriter->updateCtxs( getBinVector(ctuXPosInCtus) );
    }

    // No data collection during the compress pass
    pCABACWriter->setBinBuffer( nullptr );
#endif

    if (pCfg->getSwitchPOC() != pcPic->poc || ctuRsAddr >= pCfg->getDebugCTU())
    {
      m_pcCuEncoder->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
    }

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Clear the bin counters and prepare for collecting new data for this CTU
    pCABACWriter->setBinBuffer( getBinVector(ctuXPosInCtus) );
#endif

#if K0149_BLOCK_STATISTICS
    getAndStoreBlockStatistics(cs, ctuArea);
#endif

    pCABACWriter->resetBits();
    pCABACWriter->coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr, true, true
#if JVET_AL0153_ALF_CCCM
                                   , true
#endif
                                   );
    const int numberOfWrittenBits = int( pCABACWriter->getEstFracBits() >> SCALE_BITS );

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Done with the data collection for this CTU
    pCABACWriter->setBinBuffer( nullptr );
#endif

#if ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
    pcSlice->setSliceBits( ( uint32_t ) ( pcSlice->getSliceBits() + numberOfWrittenBits ) );
#if ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif

    // Store probabilities of first CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if( cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && pEncLib->getEntropyCodingSyncEnabledFlag() )
    {
      pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
      cs.storePrevPLT(pEncLib->m_palettePredictorSyncState);
    }

    int actualBits = int(cs.fracBits >> SCALE_BITS);
    actualBits    -= (int)m_uiPicTotalBits;
    if ( pCfg->getUseRateCtrl() )
    {
      int actualQP        = g_RCInvalidQPValue;
      double actualLambda = pRdCost->getLambda();
      int numberOfEffectivePixels    = 0;

      int numberOfSkipPixel = 0;
      for (auto &cu : cs.traverseCUs(ctuArea, CH_L))
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if ( cu.separateTree && cu.chType != CH_L )
        {
          continue;
        }
#endif
        numberOfSkipPixel += cu.skip*cu.lumaSize().area();
      }

      for( auto &cu : cs.traverseCUs( ctuArea, CH_L ) )
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if ( cu.separateTree && cu.chType != CH_L )
        {
          continue;
        }
#endif
        if( !cu.skip || cu.rootCbf )
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }
      double skipRatio = (double)numberOfSkipPixel / ctuArea.lumaSize().area();
      CodingUnit* cu = cs.getCU( ctuArea.lumaPos(), CH_L );

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
      pRateCtrl->getRCPic()->updateAfterCTU(pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda, skipRatio,
        pcSlice->isIRAP() ? 0 : pCfg->getLCULevelRC());
    }
#if ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->setLambdas (oldLambdaArray);
#else
      pTrQuant->setLambda (oldLambda);
#endif
      pRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
    }
#endif

    m_uiPicTotalBits += actualBits;
    m_uiPicDist       = cs.dist;
    // for last Ctu in the slice
    if (pcSlice->getPPS()->getNumSubPics() >= 2 && curSubPic.getTreatedAsPicFlag() && ctuIdx == (pcSlice->getNumCtuInSlice() - 1))
    {

      int subPicX = (int)curSubPic.getSubPicLeft();
      int subPicY = (int)curSubPic.getSubPicTop();
      int subPicWidth = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();

      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
      {
        int n = pcSlice->getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = pcSlice->getRefPic((RefPicList)rlist, idx);
          if (refPic->getSubPicSaved())
          {
            refPic->restoreSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->setSubPicSaved(false);
          }
        }
      }
    }
  }

#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  pcPic->setResiBufPLT();
#endif
#if JVET_AC0089_NNVC_USE_BPM_INFO
  pcPic->dumpPicBpmInfo();
#if JVET_AJ0124_QP_BLOCK
  pcPic->dumpQpBlock();
#endif
#endif

  // this is wpp exclusive section

//  m_uiPicTotalBits += actualBits;
//  m_uiPicDist       = cs.dist;

}
void EncSlice::encodeSlice   ( Picture* pcPic, OutputBitstream* pcSubstreams, uint32_t &numBinsCoded )
{

  Slice *const pcSlice                 = pcPic->slices[getSliceSegmentIdx()];
  const bool wavefrontsEnabled         = pcSlice->getSPS()->getEntropyCodingSyncEnabledFlag();
  const bool entryPointsPresentFlag    = pcSlice->getSPS()->getEntryPointsPresentFlag();
  uint32_t substreamSize               = 0;
  pcSlice->resetNumberOfSubstream();


  // setup coding structure
  CodingStructure& cs = *pcPic->cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels( *pcSlice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->getPOC() );

  pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();

  const PreCalcValues& pcv = *cs.pcv;
  const uint32_t widthInCtus   = pcv.widthInCtus;
  uint32_t uiSubStrm = 0;

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  static Ctx storedCtx;
#endif
  // for every CTU in the slice...
  for( uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++ )
  {
    const uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice( ctuIdx );
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    m_CABACWriter->initBitstream( &pcSubstreams[uiSubStrm] );

    // set up CABAC contexts' state for this CTU
    if ( cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && cs.pps->ctuIsTileRowBd( ctuYPosInCtus ) )
    {
      if (ctuIdx != 0) // if it is the first CTU, then the entropy coder has already been reset
      {
        numBinsCoded += m_CABACWriter->getNumBins();
        m_CABACWriter->initCtxModels( *pcSlice );
        cs.resetPrevPLT(cs.prevPLT);
      }
    }
    else if (cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper CTU if it's available and at the start of a line.
      if (ctuIdx != 0) // if it is the first CTU, then the entropy coder has already been reset
      {
        numBinsCoded += m_CABACWriter->getNumBins();
        m_CABACWriter->initCtxModels( *pcSlice );
        cs.resetPrevPLT(cs.prevPLT);
      }
      if( cs.getCURestricted( pos.offset( 0, -1 ), pos, pcSlice->getIndependentSliceIdx(), cs.pps->getTileIdx( pos ), CH_L ) )
      {
        // Top is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
        cs.setPrevPLT(m_palettePredictorSyncState);
      }
    }

    bool updateBcwCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuIdx == 0;
    if( updateBcwCodingOrder )
    {
      resetBcwCodingOrder(false, cs);
    }

#if JVET_V0094_BILATERAL_FILTER
    if (ctuRsAddr == 0)
    {
      if (cs.pps->getUseBIF())
      {
        m_CABACWriter->bif(COMPONENT_Y, *pcSlice, cs.picture->getBifParam(COMPONENT_Y));
      }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if (cs.pps->getUseChromaBIF())
      {
        m_CABACWriter->bif(COMPONENT_Cb, *pcSlice, cs.picture->getBifParam(COMPONENT_Cb));
        m_CABACWriter->bif(COMPONENT_Cr, *pcSlice, cs.picture->getBifParam(COMPONENT_Cr));
      }
#endif
    }
#endif

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Final writing of the bitstream - update the CABAC states based on CTU above
    if ( ctuYPosInCtus )
    {
      m_CABACWriter->updateCtxs( getBinVector(ctuXPosInCtus) );
    }

    // Clear the bin counters and prepare for collecting new data for this CTU
    m_CABACWriter->setBinBuffer( getBinVector(ctuXPosInCtus) );
#endif

    m_CABACWriter->coding_tree_unit( cs, ctuArea, pcPic->m_prevQP, ctuRsAddr );

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Done with data collection for this CTU
    m_CABACWriter->setBinBuffer( nullptr );
#endif

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
    // store CABAC context to be used in next frames
    if( storeContexts( pcSlice, ctuXPosInCtus, ctuYPosInCtus ) )
    {
      storedCtx = m_CABACWriter->getCtx();
    }
#endif

    // store probabilities of first CTU in line into buffer
    if( cs.pps->ctuIsTileColBd( ctuXPosInCtus ) && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
      cs.storePrevPLT(m_palettePredictorSyncState);
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    bool isLastCTUsinSlice = ctuIdx == pcSlice->getNumCtuInSlice()-1;
    bool isLastCTUinTile  = !isLastCTUsinSlice && cs.pps->getTileIdx( ctuRsAddr ) != cs.pps->getTileIdx( pcSlice->getCtuAddrInSlice( ctuIdx + 1 ) );
    bool isLastCTUinWPP    = !isLastCTUsinSlice && !isLastCTUinTile && wavefrontsEnabled && cs.pps->ctuIsTileColBd( pcSlice->getCtuAddrInSlice( ctuIdx + 1 ) % cs.pps->getPicWidthInCtu() );
    if (isLastCTUsinSlice || isLastCTUinTile || isLastCTUinWPP )         // this the the last CTU of the slice, tile, or WPP
    {
      m_CABACWriter->end_of_slice();  // end_of_slice_one_bit, end_of_tile_one_bit, or end_of_subset_one_bit

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      if (!isLastCTUsinSlice) //Byte alignment only when it is not the last substream in the slice
      {
        // write sub-stream size
        substreamSize += (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations();
        pcSlice->increaseNumberOfSubstream();
        if( entryPointsPresentFlag )
        {
          pcSlice->addSubstreamSize(substreamSize);
          substreamSize = 0;
        }
      }
      uiSubStrm++;
    }
  } // CTU-loop
  if(pcSlice->getPPS()->getCabacInitPresentFlag())
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId( *pcSlice );
  }
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  numBinsCoded += m_CABACWriter->getNumBins();

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  // store CABAC context to be used in next frames when the last CTU in a picture is processed
  if( pcSlice->getPPS()->pcv->sizeInCtus - 1 == pcSlice->getCtuAddrInSlice( pcSlice->getNumCtuInSlice() - 1 ) )
  {
    m_CABACWriter->m_CABACDataStore->storeCtxStates( pcSlice, storedCtx );
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (!pcSlice->isIntra())
  {
    storeAmvpSbTmvpStatArea(pcSlice->getTLayer(), g_picAmvpSbTmvpEnabledArea);
  }
#endif
}


double EncSlice::xGetQPValueAccordingToLambda ( double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
