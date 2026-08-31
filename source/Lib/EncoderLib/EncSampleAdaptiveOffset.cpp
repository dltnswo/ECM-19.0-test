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

/**
 \file     EncSampleAdaptiveOffset.cpp
 \brief       estimation part of sample adaptive offset class
 */
#include "EncSampleAdaptiveOffset.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/CodingStructure.h"
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup EncoderLib
//! \{


#define SAOCtx(c) SubCtx( Ctx::Sao, c )

#if JVET_W0066_CCSAO
#include <algorithm>

struct SetIdxCount
{
  uint8_t  setIdx;
  uint16_t count;
};

struct CtbCost
{
  int16_t pos;
  double  cost;
};

bool compareSetIdxCount(SetIdxCount a, SetIdxCount b) { return a.count > b.count; }

bool compareCtbCost(CtbCost a, CtbCost b) { return a.cost < b.cost; }
#endif

//! rounding with IBDI
inline double xRoundIbdi2(int bitDepth, double x)
{
#if FULL_NBIT
  return ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x) -0.5)));
#else
  if (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) == 0)
    return ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x) -0.5)));
  else
    return ((x) > 0) ? (int)(((int)(x) + (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) - 1)))
                             / (1 << DISTORTION_PRECISION_ADJUSTMENT(bitDepth)))
                     : ((int)(((int)(x) - (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepth) - 1)))
                              / (1 << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))));
#endif
}

inline double xRoundIbdi(int bitDepth, double x)
{
  return (bitDepth > 8 ? xRoundIbdi2(bitDepth, (x)) : ((x)>=0 ? ((int)((x)+0.5)) : ((int)((x)-0.5)))) ;
}


EncSampleAdaptiveOffset::EncSampleAdaptiveOffset()
{
  m_CABACEstimator = NULL;

  ::memset( m_saoDisabledRate, 0, sizeof( m_saoDisabledRate ) );
}

EncSampleAdaptiveOffset::~EncSampleAdaptiveOffset()
{
}

void EncSampleAdaptiveOffset::createEncData(bool isPreDBFSamplesUsed, uint32_t numCTUsPic)
{
  //statistics
  const uint32_t sizeInCtus = numCTUsPic;
  m_statData.resize( sizeInCtus );
  for(uint32_t i=0; i< sizeInCtus; i++)
  {
    m_statData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
    for(uint32_t compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      m_statData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
    }
  }
  if(isPreDBFSamplesUsed)
  {
    m_preDBFstatData.resize( sizeInCtus );
    for(uint32_t i=0; i< sizeInCtus; i++)
    {
      m_preDBFstatData[i] = new SAOStatData*[MAX_NUM_COMPONENT];
      for(uint32_t compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        m_preDBFstatData[i][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
      }
    }

  }


  for(int typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
    m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

    m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
    m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;

    if(isPreDBFSamplesUsed)
    {
      switch(typeIdc)
      {
      case SAO_TYPE_EO_0:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      case SAO_TYPE_EO_90:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_EO_135:
      case SAO_TYPE_EO_45:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 5;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 3;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 2;
        }
        break;
      case SAO_TYPE_BO:
        {
          m_skipLinesR[COMPONENT_Y ][typeIdc]= 4;
          m_skipLinesR[COMPONENT_Cb][typeIdc]= m_skipLinesR[COMPONENT_Cr][typeIdc]= 2;

          m_skipLinesB[COMPONENT_Y ][typeIdc]= 3;
          m_skipLinesB[COMPONENT_Cb][typeIdc]= m_skipLinesB[COMPONENT_Cr][typeIdc]= 1;
        }
        break;
      default:
        {
          THROW("Not a supported type");
        }
      }
    }
  }

#if JVET_W0066_CCSAO
  if (m_createdEnc)
  {
    return;
  }
  m_createdEnc = true;

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    m_ccSaoStatData    [i] = new CcSaoStatData[m_numCTUsInPic];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    m_ccSaoStatDataEdge[i] = new CcSaoStatData[m_numCTUsInPic];
#endif
  }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  int numStatsEdge = m_numCTUsInPic * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR * MAX_CCSAO_BAND_IDC * MAX_NUM_COMPONENT * MAX_CCSAO_EDGE_IDC;
  m_ccSaoStatDataEdgePre = new CcSaoStatData[numStatsEdge];
#else
  for (int comp = Y_C; comp < N_C; comp++)
  {
    m_ccSaoStatDataEdgeNew[comp] = new CcSaoStatData[m_numCTUsInPic * (CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C)
                                                     * CCSAO_QUAN_NUM * CCSAO_EDGE_TYPE];
  }
#endif
#endif
  m_bestCcSaoControl = new uint8_t[m_numCTUsInPic];
  m_tempCcSaoControl = new uint8_t[m_numCTUsInPic];
  m_initCcSaoControl = new uint8_t[m_numCTUsInPic];

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    m_trainingDistortion[i] = new int64_t[m_numCTUsInPic];
  }
#endif
}

void EncSampleAdaptiveOffset::destroyEncData()
{
  for(uint32_t i=0; i< m_statData.size(); i++)
  {
    for(uint32_t compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
    {
      delete[] m_statData[i][compIdx];
    }
    delete[] m_statData[i];
  }
  m_statData.clear();


  for(int i=0; i< m_preDBFstatData.size(); i++)
  {
    for(int compIdx=0; compIdx< MAX_NUM_COMPONENT; compIdx++)
    {
      delete[] m_preDBFstatData[i][compIdx];
    }
    delete[] m_preDBFstatData[i];
  }
  m_preDBFstatData.clear();

#if JVET_W0066_CCSAO
  if (!m_createdEnc)
  {
    return;
  }
  m_createdEnc = false;

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    if (m_ccSaoStatData    [i]) { delete[] m_ccSaoStatData    [i]; m_ccSaoStatData    [i] = nullptr; }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    if (m_ccSaoStatDataEdge[i]) { delete[] m_ccSaoStatDataEdge[i]; m_ccSaoStatDataEdge[i] = nullptr; }
#endif
  }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  if (m_ccSaoStatDataEdgePre) { delete[] m_ccSaoStatDataEdgePre; m_ccSaoStatDataEdgePre = nullptr; }
#else
  for (int comp = Y_C; comp < N_C; comp++)
  {
    if (m_ccSaoStatDataEdgeNew[comp])
    {
      delete[] m_ccSaoStatDataEdgeNew[comp];
      m_ccSaoStatDataEdgeNew[comp] = nullptr;
    }
  }
#endif
#endif

  if (m_bestCcSaoControl) { delete[] m_bestCcSaoControl; m_bestCcSaoControl = nullptr; }
  if (m_tempCcSaoControl) { delete[] m_tempCcSaoControl; m_tempCcSaoControl = nullptr; }
  if (m_initCcSaoControl) { delete[] m_initCcSaoControl; m_initCcSaoControl = nullptr; }

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    if (m_trainingDistortion[i]) { delete[] m_trainingDistortion[i]; m_trainingDistortion[i] = nullptr; }
  }
#endif
}

void EncSampleAdaptiveOffset::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_ctxCache       = ctxCache;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}


void EncSampleAdaptiveOffset::SAOProcess( CodingStructure& cs, bool* sliceEnabled, const double* lambdas,
#if ENABLE_QPA
                                          const double lambdaChromaWeight,
#endif
                                          const bool bTestSAODisableAtPictureLevel, const double saoEncodingRate, const double saoEncodingRateChroma, const bool isPreDBFSamplesUsed, bool isGreedyMergeEncoding
#if JVET_V0094_BILATERAL_FILTER
                                          ,BIFCabacEst* bifCABACEstimator
#endif
                                         )
{
#if ALF_SAO_TRUE_ORG && !JVET_V0094_BILATERAL_FILTER && !JVET_X0071_CHROMA_BILATERAL_FILTER
  PelUnitBuf org = cs.getTrueOrgBuf();
#else
  PelUnitBuf org = cs.getOrgBuf();
#endif
  PelUnitBuf res = cs.getRecoBuf();
  PelUnitBuf src = m_tempBuf;
#if !JVET_V0094_BILATERAL_FILTER && !JVET_X0071_CHROMA_BILATERAL_FILTER
  // Moved until after the bilateral filter has been initialized
  memcpy(m_lambda, lambdas, sizeof(m_lambda));
#endif

  src.copyFrom(res);

#if JVET_V0094_BILATERAL_FILTER
  const PreCalcValues& pcv = *cs.pcv;
  BifParams& bifParams = cs.picture->getBifParam(COMPONENT_Y);
  int width = cs.picture->lwidth();
  int height = cs.picture->lheight();
  int block_width = pcv.maxCUWidth;
  int block_height = pcv.maxCUHeight;

  int width_in_blocks = width / block_width + (width % block_width != 0);
  int height_in_blocks = height / block_height + (height % block_height != 0);

  bifParams.numBlocks = width_in_blocks * height_in_blocks;
  bifParams.ctuOn.resize(bifParams.numBlocks);

  std::fill(bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 0);

  // Currently no RDO to figure out if we should turn CTUs on or off
  bifParams.frmOn = 1;
  bifParams.allCtuOn = 1;

  if( bifParams.frmOn == 0 )
  {
    std::fill( bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 0 );
  }
  else if( bifParams.allCtuOn )
  {
    std::fill( bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 1 );
  }

  //double MseNoFltFrame = 0;
  //double MseFltDefFrame = 0;
  //double MseFltDefSwitchFrame = 0;
  //int CtuIdx = 0;
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(cs.pps->getUseChromaBIF())
  {
    const PreCalcValues& pcv = *cs.pcv;
    BifParams& bifParamsCb = cs.picture->getBifParam( COMPONENT_Cb );
    BifParams& bifParamsCr = cs.picture->getBifParam( COMPONENT_Cr );
    int width = cs.picture->lwidth();
    int height = cs.picture->lheight();
    int blockWidth = pcv.maxCUWidth;
    int blockHeight = pcv.maxCUHeight;

    int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
    int heightInBlocks = height / blockHeight + (height % blockHeight != 0);

    bifParamsCb.numBlocks = widthInBlocks * heightInBlocks;
    bifParamsCr.numBlocks = widthInBlocks * heightInBlocks;

    bifParamsCb.ctuOn.resize( bifParamsCb.numBlocks);
    bifParamsCr.ctuOn.resize( bifParamsCr.numBlocks);
    std::fill( bifParamsCb.ctuOn.begin(), bifParamsCb.ctuOn.end(), 0);
    std::fill( bifParamsCr.ctuOn.begin(), bifParamsCr.ctuOn.end(), 0);
    bifParamsCb.frmOn = 0;
    bifParamsCr.frmOn = 0;
    bifParamsCb.allCtuOn = 0;
    bifParamsCr.allCtuOn = 0;

    if ( bifParamsCb.frmOn == 0)
    {
      std::fill( bifParamsCb.ctuOn.begin(), bifParamsCb.ctuOn.end(), 0 );
    }
    else if ( bifParamsCb.allCtuOn)
    {
      std::fill( bifParamsCb.ctuOn.begin(), bifParamsCb.ctuOn.end(), 1 );
    }

    if ( bifParamsCr.frmOn == 0)
    {
      std::fill( bifParamsCr.ctuOn.begin(), bifParamsCr.ctuOn.end(), 0 );
    }
    else if ( bifParamsCr.allCtuOn)
    {
      std::fill( bifParamsCr.ctuOn.begin(), bifParamsCr.ctuOn.end(), 1 );
    }
  }
#endif
  
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    BilateralFilter bilateralFilter;
    if(!cs.sps->getSAOEnabledFlag() && (cs.pps->getUseBIF() || cs.pps->getUseChromaBIF()))
    {
      bilateralFilter.create();
#if JVET_AJ0237_INTERNAL_12BIT
      bilateralFilter.setInternalBitDepth(cs.sps->getBitDepth(CHANNEL_TYPE_LUMA));
#endif
      if( cs.pps->getUseBIF() )
      {
        bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Y, cs, src, bifCABACEstimator ); // Filters from src to res
      }
      if( cs.pps->getUseChromaBIF() )
      {
        //Cb
        bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cb, cs, src, bifCABACEstimator );
        //Cr
        bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cr, cs, src, bifCABACEstimator );
      }
      bilateralFilter.destroy();
      return;
    }
    memcpy(m_lambda, lambdas, sizeof(m_lambda));
#else
  BilateralFilter bilateralFilter;
  
  // Special case when SAO = 0 and BIF = 1.
  // Just filter reconstruction and return.
  // No need to estimate SAO parameters.
  if(!cs.sps->getSAOEnabledFlag() && cs.pps->getUseBIF())
  {
    bilateralFilter.create();
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Y, cs, src, bifCABACEstimator); // Filters from src to res
    bilateralFilter.destroy();
    return;
  }
  memcpy(m_lambda, lambdas, sizeof(m_lambda));
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  BilateralFilter bilateralFilter;
  if(!cs.sps->getSAOEnabledFlag() && cs.pps->getUseChromaBIF())
  {
    bilateralFilter.create();
    //Cb
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cb, cs, src, bifCABACEstimator );
    //Cr
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cr, cs, src, bifCABACEstimator );
    bilateralFilter.destroy();
    return;
  }
  memcpy(m_lambda, lambdas, sizeof(m_lambda));
#else
    //do nothing
#endif
#endif
  
  //collect statistics
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if( cs.pps->getUseBIF() || cs.pps->getUseChromaBIF() )
  {
    bilateralFilter.create();
#if JVET_AJ0237_INTERNAL_12BIT
    bilateralFilter.setInternalBitDepth(cs.sps->getBitDepth(CHANNEL_TYPE_LUMA));
#endif
    if( cs.pps->getUseBIF() )
    {
      bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Y, cs, src, bifCABACEstimator ); // Filters from src to res'
    }
    if( cs.pps->getUseChromaBIF() )
    {
      //Cb
      bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cb, cs, src, bifCABACEstimator );
      //Cr
      bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cr, cs, src, bifCABACEstimator );
    }
    getStatistics( m_statData, org, src, res, cs );
    bilateralFilter.destroy();
  }
  else
  {
    getStatistics(m_statData, org, src, src, cs);
  }
#else
  //apply BILAT to res
  if(cs.pps->getUseBIF())
  {
    bilateralFilter.create();
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Y, cs, src, bifCABACEstimator); // Filters from src to res
    getStatistics(m_statData, org, src, res, cs);
    bilateralFilter.destroy();
  }
  else
  {
    getStatistics(m_statData, org, src, src, cs);
  }
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(cs.pps->getUseChromaBIF())
  {
    bilateralFilter.create();
    //Cb
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cb, cs, src, bifCABACEstimator );
    //Cr
    bilateralFilter.bilateralFilterPicRDOperCTU( COMPONENT_Cr, cs, src, bifCABACEstimator );
    getStatistics(m_statData, org, src, res, cs);
    bilateralFilter.destroy();
  }
  else
  {
    getStatistics(m_statData, org, src, src, cs);
  }
#else
  getStatistics(m_statData, org, src, cs);
#endif
#endif

  if(isPreDBFSamplesUsed)
  {
    addPreDBFStatistics(m_statData);
  }
  
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(cs.pps->getUseBIF() || cs.pps->getUseChromaBIF())
  {
    res.copyFrom(src);
  }
#else
  //undo BILAT on res
  if(cs.pps->getUseBIF())
    res.copyFrom(src);
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(cs.pps->getUseChromaBIF())
  {
    res.copyFrom(src);
  }
#else
    //do nothing
#endif
#endif
  
  //slice on/off
  decidePicParams(*cs.slice, sliceEnabled, saoEncodingRate, saoEncodingRateChroma);

  //block on/off
  std::vector<SAOBlkParam> reconParams(cs.pcv->sizeInCtus);
  decideBlkParams( cs, sliceEnabled, m_statData, src, res, &reconParams[0], cs.picture->getSAO(), bTestSAODisableAtPictureLevel,
#if ENABLE_QPA
                   lambdaChromaWeight,
#endif
                   saoEncodingRate, saoEncodingRateChroma, isGreedyMergeEncoding );

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "SAO" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );

}


void EncSampleAdaptiveOffset::getPreDBFStatistics(CodingStructure& cs)
{
#if ALF_SAO_TRUE_ORG
  PelUnitBuf org = cs.getTrueOrgBuf();
#else
  PelUnitBuf org = cs.getOrgBuf();
#endif
  PelUnitBuf rec = cs.getRecoBuf();
  getStatistics(m_preDBFstatData, org,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                rec, rec,
#else
                rec,
#endif
                
                cs, true);
}

void EncSampleAdaptiveOffset::addPreDBFStatistics(std::vector<SAOStatData**>& blkStats)
{
  const uint32_t numCTUsPic = (uint32_t)blkStats.size();
  for(uint32_t n=0; n< numCTUsPic; n++)
  {
    for(uint32_t compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      for(uint32_t typeIdc=0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++)
      {
        blkStats[n][compIdx][typeIdc] += m_preDBFstatData[n][compIdx][typeIdc];
      }
    }
  }
}

void EncSampleAdaptiveOffset::getStatistics(std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                                            PelUnitBuf& bifYuv,
#endif
                                            CodingStructure& cs, bool isCalculatePreDeblockSamples)
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues& pcv = *cs.pcv;
  const int numberOfComponents = getNumberValidComponents(pcv.chrFormat);

  size_t lineBufferSize = pcv.maxCUWidth + 1;
  if (m_signLineBuf1.size() != lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int ctuRsAddr = 0;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail );

      //NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      //For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth  < pcv.lumaWidth );
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      int numHorVirBndry = 0, numVerVirBndry = 0;
      int horVirBndryPos[] = { -1,-1,-1 };
      int verVirBndryPos[] = { -1,-1,-1 };
      int horVirBndryPosComp[] = { -1,-1,-1 };
      int verVirBndryPosComp[] = { -1,-1,-1 };
      bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(xPos, yPos, width, height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader );

      for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        const ComponentID compID = ComponentID(compIdx);
        const CompArea& compArea = area.block( compID );

        int  srcStride  = srcYuv.get(compID).stride;
        Pel* srcBlk     = srcYuv.get(compID).bufAt( compArea );

        int  orgStride  = orgYuv.get(compID).stride;
        Pel* orgBlk     = orgYuv.get(compID).bufAt( compArea );

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
        int  bifStride  = bifYuv.get(compID).stride;
        Pel* bifBlk     = bifYuv.get(compID).bufAt( compArea );
#endif
        
        for (int i = 0; i < numHorVirBndry; i++)
        {
          horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
        }
        for (int i = 0; i < numVerVirBndry; i++)
        {
          verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
        }

        getBlkStats(compID, cs.sps->getBitDepth(toChannelType(compID)), blkStats[ctuRsAddr][compID]
                  , srcBlk, orgBlk,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                    bifBlk, bifStride,
#endif
                    srcStride, orgStride, compArea.width, compArea.height
                  , isLeftAvail,  isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
                  , isCalculatePreDeblockSamples
                  , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
                  );
      }
      ctuRsAddr++;
    }
  }
}

void EncSampleAdaptiveOffset::decidePicParams(const Slice& slice, bool* sliceEnabled, const double saoEncodingRate, const double saoEncodingRateChroma)
{
  if ( slice.getPendingRasInit() )
  { // reset
    for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      for (int tempLayer = 1; tempLayer < MAX_TLAYER; tempLayer++)
      {
        m_saoDisabledRate[compIdx][tempLayer] = 0.0;
      }
    }
  }

  const int picTempLayer = slice.getDepth();

  //decide sliceEnabled[compIdx]
  const int numberOfComponents = m_numberOfComponents;
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    sliceEnabled[compIdx] = false;
  }

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    // reset flags & counters
    sliceEnabled[compIdx] = true;

    if (saoEncodingRate>0.0)
    {
      if (saoEncodingRateChroma>0.0)
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[compIdx][picTempLayer-1] > ((compIdx==COMPONENT_Y) ? saoEncodingRate : saoEncodingRateChroma)) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
      else
      {
        // decide slice-level on/off based on previous results
        if( (picTempLayer > 0)
          && (m_saoDisabledRate[COMPONENT_Y][0] > saoEncodingRate) )
        {
          sliceEnabled[compIdx] = false;
        }
      }
    }
  }
}

int64_t EncSampleAdaptiveOffset::getDistortion(const int channelBitDepth, int typeIdc, int typeAuxInfo, int* invQuantOffset, SAOStatData& statData)
{
  int64_t dist        = 0;
  int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth);

  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        for (int offsetIdx=0; offsetIdx<NUM_SAO_EO_CLASSES; offsetIdx++)
        {
          dist += estSaoDist( statData.count[offsetIdx], invQuantOffset[offsetIdx], statData.diff[offsetIdx], shift);
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        for (int offsetIdx=typeAuxInfo; offsetIdx<typeAuxInfo+4; offsetIdx++)
        {
          int bandIdx = offsetIdx % NUM_SAO_BO_CLASSES ;
          dist += estSaoDist( statData.count[bandIdx], invQuantOffset[bandIdx], statData.diff[bandIdx], shift);
        }
      }
      break;
    default:
      {
        THROW("Not a supported type");
      }
  }

  return dist;
}

#if JVET_AJ0237_INTERNAL_12BIT
inline int64_t EncSampleAdaptiveOffset::estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift, int bdShift)
#else
inline int64_t EncSampleAdaptiveOffset::estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift)
#endif
{
#if JVET_AJ0237_INTERNAL_12BIT
  int64_t tmpOffset = offset << bdShift;
  return ((count * tmpOffset * tmpOffset - diffSum * tmpOffset * 2) >> shift);
#else
  return (( count*offset*offset-diffSum*offset*2 ) >> shift);
#endif
}


inline int EncSampleAdaptiveOffset::estIterOffset(int typeIdx, double lambda, int offsetInput, int64_t count, int64_t diffSum, int shift, int bitIncrease, int64_t& bestDist, double& bestCost, int offsetTh )
{
  int iterOffset, tempOffset;
  int64_t tempDist, tempRate;
  double tempCost, tempMinCost;
  int offsetOutput = 0;
  iterOffset = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can be used to measure the exact rate here.
  tempMinCost = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = (typeIdx == SAO_TYPE_BO) ? (abs((int)iterOffset)+2) : (abs((int)iterOffset)+1);
    if (abs((int)iterOffset)==offsetTh) //inclusive
    {
      tempRate --;
    }
    // Do the dequantization before distortion calculation
    tempOffset  = iterOffset << bitIncrease;
    tempDist    = estSaoDist( count, tempOffset, diffSum, shift);
    tempCost    = ((double)tempDist + lambda * (double) tempRate);
    if(tempCost < tempMinCost)
    {
      tempMinCost = tempCost;
      offsetOutput = iterOffset;
      bestDist = tempDist;
      bestCost = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset-1):(iterOffset+1);
  }
  return offsetOutput;
}

void EncSampleAdaptiveOffset::deriveOffsets(ComponentID compIdx, const int channelBitDepth, int typeIdc, SAOStatData& statData, int* quantOffsets, int& typeAuxInfo)
{
  int bitDepth = channelBitDepth;
  int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth);
  int offsetTh = SampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);  //inclusive

  ::memset(quantOffsets, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

  //derive initial offsets
  int numClasses = (typeIdc == SAO_TYPE_BO)?((int)NUM_SAO_BO_CLASSES):((int)NUM_SAO_EO_CLASSES);
  for(int classIdx=0; classIdx< numClasses; classIdx++)
  {
    if( (typeIdc != SAO_TYPE_BO) && (classIdx==SAO_CLASS_EO_PLAIN)  )
    {
      continue; //offset will be zero
    }

    if(statData.count[classIdx] == 0)
    {
      continue; //offset will be zero
    }

    quantOffsets[classIdx] =
      (int) xRoundIbdi(bitDepth, (double)(statData.diff[classIdx] << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))
                                   / (double)(statData.count[classIdx] << m_offsetStepLog2[compIdx]));
    quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
  }

  // adjust offsets
  switch(typeIdc)
  {
    case SAO_TYPE_EO_0:
    case SAO_TYPE_EO_90:
    case SAO_TYPE_EO_135:
    case SAO_TYPE_EO_45:
      {
        int64_t classDist;
        double classCost;
        for(int classIdx=0; classIdx<NUM_SAO_EO_CLASSES; classIdx++)
        {
          if(classIdx==SAO_CLASS_EO_FULL_VALLEY && quantOffsets[classIdx] < 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_HALF_VALLEY && quantOffsets[classIdx] < 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_HALF_PEAK   && quantOffsets[classIdx] > 0)
          {
            quantOffsets[classIdx] =0;
          }
          if(classIdx==SAO_CLASS_EO_FULL_PEAK   && quantOffsets[classIdx] > 0)
          {
            quantOffsets[classIdx] =0;
          }

          if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
          {
            quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], classDist , classCost , offsetTh );
          }
        }

        typeAuxInfo =0;
      }
      break;
    case SAO_TYPE_BO:
      {
        int64_t  distBOClasses[NUM_SAO_BO_CLASSES];
        double costBOClasses[NUM_SAO_BO_CLASSES];
        ::memset(distBOClasses, 0, sizeof(int64_t)*NUM_SAO_BO_CLASSES);
        for(int classIdx=0; classIdx< NUM_SAO_BO_CLASSES; classIdx++)
        {
          costBOClasses[classIdx]= m_lambda[compIdx];
          if( quantOffsets[classIdx] != 0 ) //iterative adjustment only when derived offset is not zero
          {
            quantOffsets[classIdx] = estIterOffset( typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], distBOClasses[classIdx], costBOClasses[classIdx], offsetTh );
          }
        }

        //decide the starting band index
        double minCost = MAX_DOUBLE, cost;
        for(int band=0; band< NUM_SAO_BO_CLASSES- 4+ 1; band++)
        {
          cost  = costBOClasses[band  ];
          cost += costBOClasses[band+1];
          cost += costBOClasses[band+2];
          cost += costBOClasses[band+3];

          if(cost < minCost)
          {
            minCost = cost;
            typeAuxInfo = band;
          }
        }
        //clear those unused classes
        int clearQuantOffset[NUM_SAO_BO_CLASSES];
        ::memset(clearQuantOffset, 0, sizeof(int)*NUM_SAO_BO_CLASSES);
        for(int i=0; i< 4; i++)
        {
          int band = (typeAuxInfo+i)%NUM_SAO_BO_CLASSES;
          clearQuantOffset[band] = quantOffsets[band];
        }
        ::memcpy(quantOffsets, clearQuantOffset, sizeof(int)*NUM_SAO_BO_CLASSES);
      }
      break;
    default:
      {
        THROW("Not a supported type");
      }

  }


}

void EncSampleAdaptiveOffset::deriveModeNewRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost )
{
  double minCost, cost;
  uint64_t previousFracBits;
  const int numberOfComponents = m_numberOfComponents;

  int64_t dist[MAX_NUM_COMPONENT], modeDist[MAX_NUM_COMPONENT];
  SAOOffset testOffset[MAX_NUM_COMPONENT];
  int invQuantOffset[MAX_NUM_SAO_CLASSES];
  for(int comp=0; comp < MAX_NUM_COMPONENT; comp++)
  {
    modeDist[comp] = 0;
  }

  //pre-encode merge flags
  modeParam[COMPONENT_Y].modeIdc = SAO_MODE_OFF;
  const TempCtx ctxStartBlk   ( m_ctxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), true );
  const TempCtx ctxStartLuma  ( m_ctxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBestLuma   ( m_ctxCache );

  //------ luma --------//
  const ComponentID compIdx = COMPONENT_Y;
  //"off" case as initial cost
  modeParam[compIdx].modeIdc = SAO_MODE_OFF;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->sao_offset_pars( modeParam[compIdx], compIdx, sliceEnabled[compIdx], bitDepths.recon[CHANNEL_TYPE_LUMA] );
  modeDist[compIdx] = 0;
  minCost = m_lambda[compIdx] * (FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits());
  ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
  if( sliceEnabled[compIdx] )
  {
    for( int typeIdc = 0; typeIdc < NUM_SAO_NEW_TYPES; typeIdc++ )
    {
      testOffset[compIdx].modeIdc = SAO_MODE_NEW;
      testOffset[compIdx].typeIdc = typeIdc;

      //derive coded offset
      deriveOffsets( compIdx, bitDepths.recon[CHANNEL_TYPE_LUMA], typeIdc, blkStats[ctuRsAddr][compIdx][typeIdc], testOffset[compIdx].offset, testOffset[compIdx].typeAuxInfo );

      //inversed quantized offsets
      invertQuantOffsets( compIdx, typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, testOffset[compIdx].offset );

      //get distortion
      dist[compIdx] = getDistortion( bitDepths.recon[CHANNEL_TYPE_LUMA], testOffset[compIdx].typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][compIdx][typeIdc] );

      //get rate
      m_CABACEstimator->getCtx() = SAOCtx( ctxStartLuma );
      m_CABACEstimator->resetBits();
      m_CABACEstimator->sao_offset_pars( testOffset[compIdx], compIdx, sliceEnabled[compIdx], bitDepths.recon[CHANNEL_TYPE_LUMA] );
      double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
      cost = ( double ) dist[compIdx] + m_lambda[compIdx] * rate;
      if( cost < minCost )
      {
        minCost = cost;
        modeDist[compIdx] = dist[compIdx];
        modeParam[compIdx] = testOffset[compIdx];
        ctxBestLuma = SAOCtx( m_CABACEstimator->getCtx() );
      }
    }
  }
  m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );

  //------ chroma --------//
  //"off" case as initial cost
  cost = 0;
  previousFracBits = 0;
  m_CABACEstimator->resetBits();
  for(uint32_t componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    modeParam[component].modeIdc = SAO_MODE_OFF;
    modeDist [component]         = 0;
    m_CABACEstimator->sao_offset_pars( modeParam[component], component, sliceEnabled[component], bitDepths.recon[CHANNEL_TYPE_CHROMA] );
    const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
    cost += m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits);
    previousFracBits = currentFracBits;
  }

  minCost = cost;

  //doesn't need to store cabac status here since the whole CTU parameters will be re-encoded at the end of this function

  for(int typeIdc=0; typeIdc< NUM_SAO_NEW_TYPES; typeIdc++)
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBestLuma );
    m_CABACEstimator->resetBits();
    previousFracBits = 0;
    cost = 0;

    for(uint32_t componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
    {
      const ComponentID component = ComponentID(componentIndex);
      if(!sliceEnabled[component])
      {
        testOffset[component].modeIdc = SAO_MODE_OFF;
        dist[component]= 0;
        continue;
      }
      testOffset[component].modeIdc = SAO_MODE_NEW;
      testOffset[component].typeIdc = typeIdc;

      //derive offset & get distortion
      deriveOffsets(component, bitDepths.recon[CHANNEL_TYPE_CHROMA], typeIdc, blkStats[ctuRsAddr][component][typeIdc], testOffset[component].offset, testOffset[component].typeAuxInfo);
      invertQuantOffsets(component, typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, testOffset[component].offset);
      dist[component] = getDistortion(bitDepths.recon[CHANNEL_TYPE_CHROMA], typeIdc, testOffset[component].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][component][typeIdc]);
      m_CABACEstimator->sao_offset_pars( testOffset[component], component, sliceEnabled[component], bitDepths.recon[CHANNEL_TYPE_CHROMA] );
      const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
      cost += dist[component] + (m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits));
      previousFracBits = currentFracBits;
    }

    if(cost < minCost)
    {
      minCost = cost;
      for(uint32_t componentIndex = COMPONENT_Cb; componentIndex < numberOfComponents; componentIndex++)
      {
        modeDist[componentIndex]  = dist[componentIndex];
        modeParam[componentIndex] = testOffset[componentIndex];
      }
    }

  } // SAO_TYPE loop

  //----- re-gen rate & normalized cost----//
  modeNormCost = 0;
  for(uint32_t componentIndex = COMPONENT_Y; componentIndex < numberOfComponents; componentIndex++)
  {
    modeNormCost += (double)modeDist[componentIndex] / m_lambda[componentIndex];
  }

  m_CABACEstimator->getCtx() = SAOCtx( ctxStartBlk );
  m_CABACEstimator->resetBits();
  m_CABACEstimator->sao_block_pars( modeParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
  modeNormCost += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
}

void EncSampleAdaptiveOffset::deriveModeMergeRDO(const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost )
{
  modeNormCost = MAX_DOUBLE;

  double cost;
  SAOBlkParam testBlkParam;
  const int numberOfComponents = m_numberOfComponents;

  const TempCtx ctxStart  ( m_ctxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
  TempCtx       ctxBest   ( m_ctxCache );

  for(int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    if(mergeList[mergeType] == NULL)
    {
      continue;
    }

    testBlkParam = *(mergeList[mergeType]);
    //normalized distortion
    double normDist=0;
    for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      testBlkParam[compIdx].modeIdc = SAO_MODE_MERGE;
      testBlkParam[compIdx].typeIdc = mergeType;

      SAOOffset& mergedOffsetParam = (*(mergeList[mergeType]))[compIdx];

      if( mergedOffsetParam.modeIdc != SAO_MODE_OFF)
      {
        //offsets have been reconstructed. Don't call inversed quantization function.
        normDist += (((double)getDistortion(bitDepths.recon[toChannelType(ComponentID(compIdx))], mergedOffsetParam.typeIdc, mergedOffsetParam.typeAuxInfo, mergedOffsetParam.offset, blkStats[ctuRsAddr][compIdx][mergedOffsetParam.typeIdc]))
                       /m_lambda[compIdx] );
      }
    }

    //rate
    m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_block_pars( testBlkParam, bitDepths, sliceEnabled, (mergeList[SAO_MERGE_LEFT]!= NULL), (mergeList[SAO_MERGE_ABOVE]!= NULL), false );
    double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
    cost = normDist+rate;

    if(cost < modeNormCost)
    {
      modeNormCost = cost;
      modeParam    = testBlkParam;
      ctxBest      = SAOCtx( m_CABACEstimator->getCtx() );
    }
  }
  if( modeNormCost < MAX_DOUBLE )
  {
    m_CABACEstimator->getCtx() = SAOCtx( ctxBest );
  }
}

void EncSampleAdaptiveOffset::decideBlkParams(CodingStructure& cs, bool* sliceEnabled, std::vector<SAOStatData**>& blkStats, PelUnitBuf& srcYuv, PelUnitBuf& resYuv,
                                               SAOBlkParam* reconParams, SAOBlkParam* codedParams, const bool bTestSAODisableAtPictureLevel,
#if ENABLE_QPA
                                               const double chromaWeight,
#endif
                                               const double saoEncodingRate, const double saoEncodingRateChroma, const bool isGreedymergeEncoding)

{
  const PreCalcValues& pcv = *cs.pcv;
  bool allBlksDisabled = true;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  if(cs.sps->getSAOEnabledFlag())
  {
    // If SAO is enabled, we should investigate the components.
    // If SAO is disabled, we should stick with allBlksDisabled=true;
#endif
  const uint32_t numberOfComponents = m_numberOfComponents;
  for(uint32_t compId = COMPONENT_Y; compId < numberOfComponents; compId++)
  {
    if (sliceEnabled[compId])
    {
      allBlksDisabled = false;
    }
  }
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  }
#endif

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  BilateralFilter bilateralFilter;
  bilateralFilter.create();
#if JVET_AJ0237_INTERNAL_12BIT
  bilateralFilter.setInternalBitDepth(cs.sps->getBitDepth(CHANNEL_TYPE_LUMA));
#endif
#endif
  
  const TempCtx ctxPicStart ( m_ctxCache, SAOCtx( m_CABACEstimator->getCtx() ) );

  SAOBlkParam modeParam;
  double minCost, modeCost;

  double minCost2 = 0;
  std::vector<SAOStatData**> groupBlkStat;
  if (isGreedymergeEncoding)
  {
    groupBlkStat.resize(cs.pcv->sizeInCtus);
    for (uint32_t k = 0; k < cs.pcv->sizeInCtus; k++)
    {
      groupBlkStat[k] = new SAOStatData*[MAX_NUM_COMPONENT];
      for (uint32_t compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        groupBlkStat[k][compIdx] = new SAOStatData[NUM_SAO_NEW_TYPES];
      }
    }
  }
  SAOBlkParam  testBlkParam;
  SAOBlkParam  groupParam;
  SAOBlkParam* tempMergeList[NUM_SAO_MERGE_TYPES] = { NULL };
  SAOBlkParam* startingMergeList[NUM_SAO_MERGE_TYPES] = { NULL };

  int     mergeCtuAddr = 1; //Ctu to be merged
  int     groupSize = 1;
  double  Cost[2] = { 0, 0 };
  TempCtx ctxBeforeMerge(m_ctxCache);
  TempCtx ctxAfterMerge(m_ctxCache);

  double totalCost = 0; // Used if bTestSAODisableAtPictureLevel==true

  int ctuRsAddr = 0;
#if ENABLE_QPA
  CHECK ((chromaWeight > 0.0) && (cs.slice->getFirstCtuRsAddrInSlice() != 0), "incompatible start CTU address, must be 0");
#endif

  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( pcv.chrFormat, Area( xPos , yPos, width, height) );

      if(allBlksDisabled)
      {
        codedParams[ctuRsAddr].reset();
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(!cs.pps->getUseBIF() && !cs.pps->getUseChromaBIF())
        {
            continue;
        }
#else
        // In the combined filter, we cannot continue here, even if SAO is
        // turned off for all blocks, since we need to perform the bilateral
        // filter later on (see next JVET_V0094_BILATERAL_FILTER).
        if(!cs.pps->getUseBIF())
        {
          // Unless we are not using BIF. Then it is safe to continue.
          continue;
        }
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(!cs.pps->getUseChromaBIF())
        {
            continue;
        }
#else
        continue;
#endif
#endif
      }

      const TempCtx  ctxStart ( m_ctxCache, SAOCtx( m_CABACEstimator->getCtx() ) );
      TempCtx        ctxBest  ( m_ctxCache );

      if (ctuRsAddr == (mergeCtuAddr - 1))
      {
        ctxBeforeMerge = SAOCtx(m_CABACEstimator->getCtx());
      }

      //get merge list
      SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
      getMergeList(cs, ctuRsAddr, reconParams, mergeList);

      minCost = MAX_DOUBLE;
#if ENABLE_QPA
      if (chromaWeight > 0.0) // temporarily adopt local (CTU-wise) lambdas from QPA
      {
        for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
        {
          m_lambda[compIdx] = isLuma ((ComponentID)compIdx) ? cs.picture->m_uEnerHpCtu[ctuRsAddr] : cs.picture->m_uEnerHpCtu[ctuRsAddr] / chromaWeight;
        }
      }
#endif
      for(int mode=1; mode < NUM_SAO_MODES; mode++)
      {
        if( mode > 1 )
        {
          m_CABACEstimator->getCtx() = SAOCtx( ctxStart );
        }
        switch(mode)
        {
        case SAO_MODE_NEW:
          {
            deriveModeNewRDO(cs.sps->getBitDepths(), ctuRsAddr, mergeList, sliceEnabled, blkStats, modeParam, modeCost );
          }
          break;
        case SAO_MODE_MERGE:
          {
            deriveModeMergeRDO(cs.sps->getBitDepths(), ctuRsAddr, mergeList, sliceEnabled, blkStats , modeParam, modeCost );
          }
          break;
        default:
          {
            THROW( "Not a supported SAO mode." );
          }
        }

        if(modeCost < minCost)
        {
          minCost                = modeCost;
          codedParams[ctuRsAddr] = modeParam;
          ctxBest                = SAOCtx( m_CABACEstimator->getCtx() );
        }
      } //mode

      if (!isGreedymergeEncoding)
      {
      totalCost += minCost;
      }


      m_CABACEstimator->getCtx() = SAOCtx( ctxBest );

      //apply reconstructed offsets
      reconParams[ctuRsAddr] = codedParams[ctuRsAddr];
      reconstructBlkSAOParam(reconParams[ctuRsAddr], mergeList);

      if (isGreedymergeEncoding)
      {
        if (ctuRsAddr == (mergeCtuAddr - 1))
        {
          Cost[0] = minCost;  //previous
          groupSize = 1;
          getMergeList(cs, ctuRsAddr, reconParams, startingMergeList);
        }
        else if (ctuRsAddr == mergeCtuAddr)
        {
          Cost[1] = minCost;
          minCost2 = MAX_DOUBLE;
          for (int tmp = groupSize; tmp >= 0; tmp--)
          {
            for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              for (int i = 0; i < NUM_SAO_NEW_TYPES; i++)
              {
                for (int j = 0; j < MAX_NUM_SAO_CLASSES; j++)
                {
                  if (tmp == groupSize)
                  {
                    groupBlkStat[ctuRsAddr][compIdx][i].count[j] = blkStats[ctuRsAddr - tmp][compIdx][i].count[j];
                    groupBlkStat[ctuRsAddr][compIdx][i].diff[j] = blkStats[ctuRsAddr - tmp][compIdx][i].diff[j];
                  }
                  else
                  {
                    groupBlkStat[ctuRsAddr][compIdx][i].count[j] += blkStats[ctuRsAddr - tmp][compIdx][i].count[j];
                    groupBlkStat[ctuRsAddr][compIdx][i].diff[j] += blkStats[ctuRsAddr - tmp][compIdx][i].diff[j];
                  }
                }
              }
            }
          }

          // Derive new offset for grouped CTUs
          m_CABACEstimator->getCtx() = SAOCtx(ctxBeforeMerge);
          deriveModeNewRDO(cs.sps->getBitDepths(), ctuRsAddr, startingMergeList, sliceEnabled, groupBlkStat, modeParam, modeCost);

          //rate for mergeLeft CTB
          testBlkParam[COMPONENT_Y].modeIdc = SAO_MODE_MERGE;
          testBlkParam[COMPONENT_Y].typeIdc = SAO_MERGE_LEFT;
          m_CABACEstimator->resetBits();
          m_CABACEstimator->sao_block_pars(testBlkParam, cs.sps->getBitDepths(), sliceEnabled, true, false, true);
          double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          modeCost += rate * groupSize;
          if (modeCost < minCost2)
          {
            groupParam = modeParam;
            minCost2 = modeCost;
            ctxAfterMerge = SAOCtx(m_CABACEstimator->getCtx());
          }

          // Test merge mode for grouped CTUs
          m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
          deriveModeMergeRDO(cs.sps->getBitDepths(), ctuRsAddr, startingMergeList, sliceEnabled, groupBlkStat, modeParam, modeCost);
          modeCost += rate * groupSize;
          if (modeCost < minCost2)
          {
            minCost2 = modeCost;
            groupParam = modeParam;
            ctxAfterMerge = SAOCtx(m_CABACEstimator->getCtx());
          }

          totalCost += Cost[0];
          totalCost += Cost[1];

          if ((Cost[0] + Cost[1]) > minCost2) //merge current CTU
          {
            //original merge all
            totalCost = totalCost - Cost[0] - Cost[1] + minCost2;
            codedParams[ctuRsAddr - groupSize] = groupParam;
            for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              codedParams[ctuRsAddr][compIdx].modeIdc = SAO_MODE_MERGE;
              codedParams[ctuRsAddr][compIdx].typeIdc = SAO_MERGE_LEFT;
            }
            for (int i = groupSize; i >= 0; i--) //change previous results
            {
              reconParams[ctuRsAddr - i] = codedParams[ctuRsAddr - i];
              getMergeList(cs, ctuRsAddr - i, reconParams, tempMergeList);
              reconstructBlkSAOParam(reconParams[ctuRsAddr - i], tempMergeList);
            }

            mergeCtuAddr += 1;
            if (mergeCtuAddr % pcv.widthInCtus == 0) //reaching the end of a row
            {
              mergeCtuAddr += 1;
            }
            else //next CTU can be merged with current group
            {
              Cost[0] = minCost2;
              groupSize += 1;
            }
            m_CABACEstimator->getCtx() = SAOCtx(ctxAfterMerge);
          }
          else // don't merge current CTU
          {
            mergeCtuAddr += 1;
            // Current block will be the starting block for successive operations
            Cost[0] = Cost[1];
            getMergeList(cs, ctuRsAddr, reconParams, startingMergeList);
            groupSize = 1;
            m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
            ctxBeforeMerge = SAOCtx(m_CABACEstimator->getCtx());
            m_CABACEstimator->getCtx() = SAOCtx(ctxBest);
            if (mergeCtuAddr% pcv.widthInCtus == 0) //reaching the end of a row
            {
              mergeCtuAddr += 1;
            }
          } //else, if(Cost[0] + Cost[1] > minCost2)
        }//else if (ctuRsAddr == mergeCtuAddr)
      }
      else
      {
#if JVET_W0066_CCSAO
      offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
#if JVET_V0094_BILATERAL_FILTER
      if (cs.pps->getUseBIF())
      {
        BifParams& bifParams = cs.picture->getBifParam( COMPONENT_Y );
        for (auto& currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
        {
          for (auto& currTU : CU::traverseTUs(currCU))
          {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
            bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
            bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
            bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
            if (applyBIF)
            {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
              int  numHorVirBndry = 0, numVerVirBndry = 0;
              int  horVirBndryPos[]               = { 0, 0, 0 };
              int  verVirBndryPos[]               = { 0, 0, 0 };
              bool isTUCrossedByVirtualBoundaries = bilateralFilter.isCrossedByVirtualBoundaries(
                cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop, clipBottom,
                clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif
              bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
              );
            }
          }
        }
      }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if(cs.pps->getUseChromaBIF())
      {
        bool isDualTree = CS::isDualITree(cs);
        ChannelType chType = isDualTree ? CH_C : CH_L;
        bool applyChromaBIF = false;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
        const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
#endif

        for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
        {
          bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
          if(!chromaValid)
          {
            continue;
          }
          for (auto &currTU : CU::traverseTUs(currCU))
          {
            for(int compIdx = COMPONENT_Cb ; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              ComponentID compID = ComponentID( compIdx );
              BifParams& chromaBifParams = cs.picture->getBifParam( compID );
              bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
              applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
              bool tuValid = false;
              bool tuCBF = false;
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              applyChromaBIF = false;
              if(!isDualTree)
              {
                tuValid = currTU.blocks[compIdx].valid();
                tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                if(tuValid)
                {
                  tuCBF = TU::getCbf(currTU, compID);
                }
                applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
              }
              else
              {
                tuCBF = TU::getCbf(currTU, compID);
                applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
              }
#endif
              if(applyChromaBIF)
              {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int       numHorVirBndry = 0, numVerVirBndry = 0;
                int       horVirBndryPos[]               = { 0, 0, 0 };
                int       verVirBndryPos[]               = { 0, 0, 0 };
                CompArea &myArea                         = currTU.block(compID);
                int       yPos                           = myArea.y << chromaScaleY;
                int       xPos                           = myArea.x << chromaScaleX;
                bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                  clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                );
              }
            }
          }
        }
      }
#endif
#else
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseBIF() || cs.pps->getUseChromaBIF())
#else
        if(cs.pps->getUseBIF())
#endif
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          // Avoid old slow code
          // offsetCTUonlyBIF(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          // and instead do the code included below.
          
          // We don't need to clip if SAO was not performed on luma.
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset& myCtbOffset     = mySAOblkParam[0];
          BifParams& bifParams = cs.picture->getBifParam(COMPONENT_Y);
          
          bool clipLumaIfNoBilat = false;
          if(myCtbOffset.modeIdc != SAO_MODE_OFF)
            clipLumaIfNoBilat = true;
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
          SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];

          bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

          if(myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cb] = true;
          }
          if(myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cr] = true;
          }
          if(cs.pps->getUseBIF())
          {
#endif
          
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto &currTU : CU::traverseTUs(currCU))
            {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
              
              if (applyBIF)
              {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int  numHorVirBndry = 0, numVerVirBndry = 0;
                int  horVirBndryPos[]               = { 0, 0, 0 };
                int  verVirBndryPos[]               = { 0, 0, 0 };
                bool isTUCrossedByVirtualBoundaries = bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                  clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif
                bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry
                  , clipTop, clipBottom, clipLeft, clipRight
#endif
                );
              }
              else
              {
                // We don't need to clip if SAO was not performed on luma.
                if( clipLumaIfNoBilat )
                {
                  bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng( COMPONENT_Y ), currTU );
                }
              }
            }
          }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          } // BIF LUMA is disabled
          else
          {
            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
            {
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                if(clipLumaIfNoBilat)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Y), currTU );
                }
              }
            }
          }
          if(cs.pps->getUseChromaBIF())
          {
            bool isDualTree = CS::isDualITree(cs);
            ChannelType chType = isDualTree ? CH_C : CH_L;
            bool applyChromaBIF = false;

            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if(!chromaValid)
              {
                continue;
              }
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
                {
                  ComponentID compID = ComponentID( compIdx );
                  BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                  bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                  applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                  bool tuValid = false;
                  bool tuCBF = false;
                  bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                  applyChromaBIF = false;
                  if(!isDualTree)
                  {
                    tuValid = currTU.blocks[compIdx].valid();
                    tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                    if(tuValid)
                    {
                      tuCBF = TU::getCbf(currTU, compID);
                    }
                    applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                  }
                  else
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                    applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                  }
#endif
                  if(applyChromaBIF)
                  {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                    int       numHorVirBndry = 0, numVerVirBndry = 0;
                    int       horVirBndryPos[] = { 0, 0, 0 };
                    int       verVirBndryPos[] = { 0, 0, 0 };
                    CompArea &myArea           = currTU.block(compID);
                    const int chromaScaleX     = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                    const int chromaScaleY     = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                    int       yPos             = myArea.y << chromaScaleY;
                    int       xPos             = myArea.x << chromaScaleX;
                    bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                      cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                      clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                    m_bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                    );
                  }
                  else
                  {
                    bool useClip = clipChromaIfNoBilat[compID];
                    if(useClip && currTU.blocks[compID].valid())
                    {
                      m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, srcYuv, resYuv, cs.slice->clpRng(compID), currTU );
                    }
                  }
                }
              }
            }
          }// BIF chroma is disabled
          else
          {
            bool isDualTree = CS::isDualITree(cs);
            ChannelType chType = isDualTree ? CH_C : CH_L;

            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if(!chromaValid)
              {
                continue;
              }
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                if( clipChromaIfNoBilat[COMPONENT_Cb] && currTU.blocks[COMPONENT_Cb].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cb, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Cb), currTU );
                }
                if( clipChromaIfNoBilat[COMPONENT_Cr] && currTU.blocks[COMPONENT_Cr].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cr, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Cr), currTU );
                }
              }
            }
          }
#endif
        }
        else
        {
          // We do not do BIF for this sequence, so we can use the regular SAO function
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseChromaBIF())
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset& myCtbOffset     = mySAOblkParam[0];

          bool clipLumaIfNoBilat = false;
          if(myCtbOffset.modeIdc != SAO_MODE_OFF)
          {
            clipLumaIfNoBilat = true;
          }

          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              if(clipLumaIfNoBilat)
              {
                bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Y), currTU );
              }
            }
          }

          SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
          SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];
          bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

          if(myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cb] = true;
          }
          if(myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cr] = true;
          }

          bool tuValid = false;
          bool tuCBF = false;
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          bool applyChromaBIF = false;

          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
              {
                ComponentID compID = ComponentID( compIdx );
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                applyChromaBIF = false;
                if(!isDualTree)
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if(tuValid)
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                  }
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                }
                else
                {
                  tuCBF = TU::getCbf(currTU, compID);
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                }
#endif
                if(applyChromaBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int       numHorVirBndry = 0, numVerVirBndry = 0;
                  int       horVirBndryPos[]               = { 0, 0, 0 };
                  int       verVirBndryPos[]               = { 0, 0, 0 };
                  CompArea &myArea                         = currTU.block(compID);
                  const int chromaScaleX                   = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                  const int chromaScaleY                   = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                  int       yPos                           = myArea.y << chromaScaleY;
                  int       xPos                           = myArea.x << chromaScaleX;
                  bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                    clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                  m_bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
                else
                {
                  bool useClip = clipChromaIfNoBilat[compID];
                  if(useClip && currTU.blocks[compID].valid())
                  {
                    m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, srcYuv, resYuv, cs.slice->clpRng(compID), currTU );
                  }
                }
              }
            }
          }
        }
        else
        {
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
#else
      offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
#endif
#endif
#endif

      }

      ctuRsAddr++;
    } //ctuRsAddr
  }

#if ENABLE_QPA
  // restore global lambdas (might be unnecessary)
  if (chromaWeight > 0.0) memcpy (m_lambda, cs.slice->getLambdas(), sizeof (m_lambda));

#endif
  //reconstruct
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (isGreedymergeEncoding || (cs.pps->getUseBIF() &&allBlksDisabled) || (cs.pps->getUseChromaBIF() &&allBlksDisabled))
#else
  if (isGreedymergeEncoding || (cs.pps->getUseBIF() &&allBlksDisabled) )
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (isGreedymergeEncoding || (cs.pps->getUseChromaBIF() &&allBlksDisabled) )
#else
  if (isGreedymergeEncoding)
#endif
#endif
  {
    ctuRsAddr = 0;
    for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
    {
      for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
      {
        const uint32_t width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
        const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

        const UnitArea area(pcv.chrFormat, Area(xPos, yPos, width, height));
        
#if JVET_V0094_BILATERAL_FILTER
        if(cs.pps->getUseBIF())
        {
          resYuv.subBuf( area ).bufs[COMPONENT_Y].copyFrom( srcYuv.subBuf( area ).bufs[COMPONENT_Y] );
        }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseChromaBIF())
        {
          resYuv.subBuf( area ).bufs[COMPONENT_Cb].copyFrom( srcYuv.subBuf( area ).bufs[COMPONENT_Cb] );
          resYuv.subBuf( area ).bufs[COMPONENT_Cr].copyFrom( srcYuv.subBuf( area ).bufs[COMPONENT_Cr] );
        }
#endif

#if JVET_W0066_CCSAO
        offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
#if JVET_V0094_BILATERAL_FILTER
        if (cs.pps->getUseBIF())
        {
          BifParams& bifParams = cs.picture->getBifParam( COMPONENT_Y );
          for (auto& currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto& currTU : CU::traverseTUs(currCU))
            {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
              if (applyBIF)
              {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int  numHorVirBndry = 0, numVerVirBndry = 0;
                int  horVirBndryPos[]               = { 0, 0, 0 };
                int  verVirBndryPos[]               = { 0, 0, 0 };
                bool isTUCrossedByVirtualBoundaries = bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                  clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif

                bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                );
              }
            }
          }
        }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseChromaBIF())
        {
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          bool applyChromaBIF = false;
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
          const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
#endif

          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }

            for (auto &currTU : CU::traverseTUs(currCU))
            {
              for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
              {
                ComponentID compID = ComponentID( compIdx );
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                bool tuValid = false;
                bool tuCBF = false;
                bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                applyChromaBIF = false;
                if(!isDualTree)
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if(tuValid)
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                  }
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                }
                else
                {
                  tuCBF = TU::getCbf(currTU, compID);
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                }
#endif
                if(applyChromaBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int       numHorVirBndry = 0, numVerVirBndry = 0;
                  int       horVirBndryPos[]               = { 0, 0, 0 };
                  int       verVirBndryPos[]               = { 0, 0, 0 };
                  CompArea &myArea                         = currTU.block(compID);
                  int       yPos                           = myArea.y << chromaScaleY;
                  int       xPos                           = myArea.x << chromaScaleX;
                  bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                    clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                  bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
              }
            }
          }
        }
#endif
#else
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseBIF() || cs.pps->getUseChromaBIF())
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset& myCtbOffset     = mySAOblkParam[0];
          BifParams& bifParams = cs.picture->getBifParam(COMPONENT_Y);

          bool clipLumaIfNoBilat = false;
          if(myCtbOffset.modeIdc != SAO_MODE_OFF)
          {
            clipLumaIfNoBilat = true;
          }

          SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
          SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];
          bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

          if(myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cb] = true;
          }
          if(myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cr] = true;
          }
          if(cs.pps->getUseBIF())
          {
            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
            {
              for (auto &currTU : CU::traverseTUs(currCU))
              {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
                bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
                if (applyBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int  numHorVirBndry = 0, numVerVirBndry = 0;
                  int  horVirBndryPos[]               = { 0, 0, 0 };
                  int  verVirBndryPos[]               = { 0, 0, 0 };
                  bool isTUCrossedByVirtualBoundaries = bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                    clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif
                  bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry
                  , clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
                else
                {
                  // We don't need to clip if SAO was not performed on luma.
                  if(clipLumaIfNoBilat)
                  {
                    bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Y), currTU );
                  }
                }
              }
            }
          }
          else
          {
            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
            {
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                if(clipLumaIfNoBilat)
                {
                  bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Y), currTU );
                }
              }
            }
          }

          if(cs.pps->getUseChromaBIF())
          {
            bool isDualTree = CS::isDualITree(cs);
            ChannelType chType = isDualTree ? CH_C : CH_L;
            bool applyChromaBIF = false;

            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if(!chromaValid)
              {
                continue;
              }
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
                {
                  ComponentID compID = ComponentID( compIdx );
                  BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                  bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
                  applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                  bool tuValid = false;
                  bool tuCBF = false;
                  bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                  applyChromaBIF = false;
                  if(!isDualTree)
                  {
                    tuValid = currTU.blocks[compIdx].valid();
                    tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                    if(tuValid)
                    {
                      tuCBF = TU::getCbf(currTU, compID);
                    }
                    applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                  }
                  else
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                    applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                  }
#endif
                  if(applyChromaBIF)
                  {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                    int       numHorVirBndry = 0, numVerVirBndry = 0;
                    int       horVirBndryPos[] = { 0, 0, 0 };
                    int       verVirBndryPos[] = { 0, 0, 0 };
                    CompArea &myArea           = currTU.block(compID);
                    const int chromaScaleX     = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                    const int chromaScaleY     = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                    int       yPos             = myArea.y << chromaScaleY;
                    int       xPos             = myArea.x << chromaScaleX;
                    bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                      cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                      clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                    bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                      ,isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                    );
                  }
                  else
                  {
                    bool useClip = clipChromaIfNoBilat[compID];
                    if(useClip && currTU.blocks[compIdx].valid())
                    {
                      bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, srcYuv, resYuv, cs.slice->clpRng(compID), currTU );
                    }
                  }
                }
              }
            }
          }
          else
          {
            bool isDualTree = CS::isDualITree(cs);
            ChannelType chType = isDualTree ? CH_C : CH_L;

            for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if(!chromaValid)
              {
                continue;
              }
              for (auto &currTU : CU::traverseTUs(currCU))
              {
                if( clipChromaIfNoBilat[COMPONENT_Cb] && currTU.blocks[COMPONENT_Cb].valid())
                {
                  bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cb, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Cb), currTU );
                }
                if( clipChromaIfNoBilat[COMPONENT_Cr] && currTU.blocks[COMPONENT_Cr].valid())
                {
                  bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cr, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Cr), currTU );
                }
              }
            }
          }
        }//BIF = 1 OR CBIF = 1
        else
        {
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
#else
        if(cs.pps->getUseBIF())
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          
          // Avoid old slow code
          // offsetCTUonlyBIF(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          // and instead do the code included below.
          
          // We don't need to clip if SAO was not performed on luma.
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset& myCtbOffset     = mySAOblkParam[0];
          BifParams& bifParams = cs.picture->getBifParam(COMPONENT_Y);
          
          bool clipLumaIfNoBilat = false;
          if( myCtbOffset.modeIdc != SAO_MODE_OFF )
          {
            clipLumaIfNoBilat = true;
          }
          
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto &currTU : CU::traverseTUs(currCU))
            {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
              if (applyBIF)
              {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int  numHorVirBndry = 0, numVerVirBndry = 0;
                int  horVirBndryPos[]               = { 0, 0, 0 };
                int  verVirBndryPos[]               = { 0, 0, 0 };
                bool isTUCrossedByVirtualBoundaries = bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                  clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif

                bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry
                  , clipTop, clipBottom, clipLeft, clipRight
#endif
                );
              }
              else
              {
                // We don't need to clip if SAO was not performed on luma.
                if( clipLumaIfNoBilat )
                {
                  bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng( COMPONENT_Y ), currTU );
                }
              }
            }
          }
        }
        else
        {
          // We do not use BIF so we can use the regular SAO function call
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(cs.pps->getUseChromaBIF())
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);

          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];

          SAOOffset& myCtbOffset     = mySAOblkParam[0];
          bool clipLumaIfNoBilat = false;
          if(myCtbOffset.modeIdc != SAO_MODE_OFF)
          {
            clipLumaIfNoBilat = true;
          }

          SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
          SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];
          bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

          if(myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cb] = true;
          }
          if(myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
          {
            clipChromaIfNoBilat[COMPONENT_Cr] = true;
          }

          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              if(clipLumaIfNoBilat)
              {
                bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, srcYuv, resYuv, cs.slice->clpRng(COMPONENT_Y), currTU );
              }
            }
          }

          bool tuValid = false;
          bool tuCBF = false;
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          bool applyChromaBIF = false;
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
              {
                ComponentID compID = ComponentID( compIdx );
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                applyChromaBIF = false;
                if(!isDualTree)
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if(tuValid)
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                  }
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                }
                else
                {
                  tuCBF = TU::getCbf(currTU, compID);
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                }
#endif
                if(applyChromaBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int       numHorVirBndry = 0, numVerVirBndry = 0;
                  int       horVirBndryPos[]               = { 0, 0, 0 };
                  int       verVirBndryPos[]               = { 0, 0, 0 };
                  CompArea &myArea                         = currTU.block(compID);
                  const int chromaScaleX                   = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                  const int chromaScaleY                   = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                  int       yPos                           = myArea.y << chromaScaleY;
                  int       xPos                           = myArea.x << chromaScaleX;
                  bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                    clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                  bilateralFilter.bilateralFilterDiamond5x5(compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
                else
                {
                  bool useClip = clipChromaIfNoBilat[compID];
                  if(useClip && currTU.blocks[compIdx].valid())
                  {
                    bilateralFilter.clipNotBilaterallyFilteredBlocksChroma( compID, srcYuv, resYuv, cs.slice->clpRng(compID), currTU );
                  }
                }
              }
            }
          }
        }
        else
        {
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
#else
        offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
#endif
#endif
#endif
        ctuRsAddr++;
      }
    }
    //delete memory
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    if(!(cs.pps->getUseBIF()) || !(cs.pps->getUseChromaBIF()) || !allBlksDisabled)
    {
#else
    if (!(cs.pps->getUseBIF()) ||  !allBlksDisabled)
    {
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    if (!(cs.pps->getUseChromaBIF()) ||  !allBlksDisabled)
    {
#else
//     DO NOTHING
#endif
#endif
    for (uint32_t i = 0; i< groupBlkStat.size(); i++)
    {
      for (uint32_t compIdx = 0; compIdx< MAX_NUM_COMPONENT; compIdx++)
      {
        delete[] groupBlkStat[i][compIdx];
      }
      delete[] groupBlkStat[i];
    }
    groupBlkStat.clear();
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
    }
#endif
  }
  if (!allBlksDisabled && (totalCost >= 0) && bTestSAODisableAtPictureLevel) //SAO has not beneficial in this case - disable it
  {
    for( ctuRsAddr = 0; ctuRsAddr < pcv.sizeInCtus; ctuRsAddr++)
    {
      codedParams[ctuRsAddr].reset();
    }

    for (uint32_t componentIndex = 0; componentIndex < MAX_NUM_COMPONENT; componentIndex++)
    {
      sliceEnabled[componentIndex] = false;
    }
    m_CABACEstimator->getCtx() = SAOCtx(ctxPicStart);
  }

  EncSampleAdaptiveOffset::disabledRate( cs, reconParams, saoEncodingRate, saoEncodingRateChroma );
  
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  bilateralFilter.destroy();
#endif
}

void EncSampleAdaptiveOffset::disabledRate( CodingStructure& cs, SAOBlkParam* reconParams, const double saoEncodingRate, const double saoEncodingRateChroma )
{
  if (saoEncodingRate > 0.0)
  {
    const PreCalcValues& pcv = *cs.pcv;
    const uint32_t numberOfComponents = getNumberValidComponents( cs.picture->chromaFormat );
    int picTempLayer = cs.slice->getDepth();
    int numCtusForSAOOff[MAX_NUM_COMPONENT];

    for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      numCtusForSAOOff[compIdx] = 0;
      for( int ctuRsAddr=0; ctuRsAddr< pcv.sizeInCtus; ctuRsAddr++)
      {
        if( reconParams[ctuRsAddr][compIdx].modeIdc == SAO_MODE_OFF)
        {
          numCtusForSAOOff[compIdx]++;
        }
      }
    }
    if (saoEncodingRateChroma > 0.0)
    {
      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        m_saoDisabledRate[compIdx][picTempLayer] = (double)numCtusForSAOOff[compIdx]/(double)pcv.sizeInCtus;
      }
    }
    else if (picTempLayer == 0)
    {
      m_saoDisabledRate[COMPONENT_Y][0] = (double)(numCtusForSAOOff[COMPONENT_Y]+numCtusForSAOOff[COMPONENT_Cb]+numCtusForSAOOff[COMPONENT_Cr])/(double)(pcv.sizeInCtus *3);
    }
  }
}

void EncSampleAdaptiveOffset::getBlkStats(const ComponentID compIdx, const int channelBitDepth, SAOStatData* statsDataTypes
                        , Pel* srcBlk, Pel* orgBlk,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                          Pel* bifBlk, int bifStride,
#endif
                                          int srcStride, int orgStride, int width, int height
                        , bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail
                        , bool isCalculatePreDeblockSamples
                        , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
                        )
{
  int x,y, startX, startY, endX, endY, edgeType, firstLineStartX, firstLineEndX;
  int8_t signLeft, signRight, signDown;
  int64_t *diff, *count;
  Pel *srcLine, *orgLine;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  Pel *bifLine;
#endif
  int* skipLinesR = m_skipLinesR[compIdx];
  int* skipLinesB = m_skipLinesB[compIdx];

  for(int typeIdx=0; typeIdx< NUM_SAO_NEW_TYPES; typeIdx++)
  {
    SAOStatData& statsData= statsDataTypes[typeIdx];
    statsData.reset();

    srcLine = srcBlk;
    orgLine = orgBlk;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
    bifLine = bifBlk;
#endif
    diff    = statsData.diff;
    count   = statsData.count;
    switch(typeIdx)
    {
    case SAO_TYPE_EO_0:
      {
        diff +=2;
        count+=2;
        endY   = (isBelowAvail) ? (height - skipLinesB[typeIdx]) : height;
        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        for (y=0; y<endY; y++)
        {
          signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
          for (x=startX; x<endX; x++)
          {
            signRight =  (int8_t)sgn(srcLine[x] - srcLine[x+1]);
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
            {
              signLeft = -signRight;
              continue;
            }
            edgeType  =  signRight + signLeft;
            signLeft  = -signRight;

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
            diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
            diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
            count[edgeType] ++;
          }
          srcLine  += srcStride;
          orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine  += bifStride;
#endif
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0 : 1;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
              for (x=startX; x<endX; x++)
              {
                signRight =  (int8_t)sgn(srcLine[x] - srcLine[x+1]);
                if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, endY + y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
                {
                  signLeft = -signRight;
                  continue;
                }
                edgeType  =  signRight + signLeft;
                signLeft  = -signRight;

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
                diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
              bifLine  += bifStride;
#endif
            }
          }
        }
      }
      break;
    case SAO_TYPE_EO_90:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine = &m_signLineBuf1[0];

        startX = (!isCalculatePreDeblockSamples) ? 0
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : width)
                                                 ;
        startY = isAboveAvail ? 0 : 1;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : width)
                                                 : width
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);
        if (!isAboveAvail)
        {
          srcLine += srcStride;
          orgLine += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine += bifStride;
#endif
        }

        Pel* srcLineAbove = srcLine - srcStride;
        for (x=startX; x<endX; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
        }

        Pel* srcLineBelow;
        for (y=startY; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              signUpLine[x] = -signDown;
              continue;
            }
            edgeType  = signDown + signUpLine[x];
            signUpLine[x]= -signDown;

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
            diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
            diff [edgeType] += (orgLine[x] - srcLine[x]);

#endif
            count[edgeType] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine += bifStride;
#endif
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x<endX; x++)
              {
                if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y + endY, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
                {
                  continue;
                }
                edgeType = sgn(srcLine[x] - srcLineBelow[x]) + sgn(srcLine[x] - srcLineAbove[x]);
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
                diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
              bifLine  += bifStride;
#endif
            }
          }
        }

      }
      break;
    case SAO_TYPE_EO_135:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine, *signDownLine, *signTmpLine;

        signUpLine  = &m_signLineBuf1[0];
        signDownLine= &m_signLineBuf2[0];

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;

        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]): (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        //prepare 2nd line's upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX; x<endX+1; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x-1]);
        }

        //1st line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = (!isCalculatePreDeblockSamples) ? (isAboveLeftAvail ? 0    : 1) : startX;
        firstLineEndX   = (!isCalculatePreDeblockSamples) ? (isAboveAvail     ? endX : 1) : endX;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          edgeType = sgn(srcLine[x] - srcLineAbove[x-1]) - signUpLine[x+1];
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
          diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
          count[edgeType] ++;
        }
        srcLine  += srcStride;
        orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
        bifLine  += bifStride;
#endif
        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x=startX; x<endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x+1]);
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              signDownLine[x + 1] = -signDown;
              continue;
            }
            edgeType = signDown + signUpLine[x];
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
            diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
            diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
            count[edgeType] ++;

            signDownLine[x+1] = -signDown;
          }
          signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);

          signTmpLine  = signUpLine;
          signUpLine   = signDownLine;
          signDownLine = signTmpLine;

          srcLine += srcStride;
          orgLine += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine += bifStride;
#endif
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0     : 1 ;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x< endX; x++)
              {
                if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y + endY, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
                {
                  continue;
                }
                edgeType = sgn(srcLine[x] - srcLineBelow[x+1]) + sgn(srcLine[x] - srcLineAbove[x-1]);
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
                diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
              bifLine  += bifStride;
#endif
            }
          }
        }
      }
      break;
    case SAO_TYPE_EO_45:
      {
        diff +=2;
        count+=2;
        int8_t *signUpLine = &m_signLineBuf1[1];

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail  ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 ;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 : (isRightAvail ? width : (width - 1))
                                                 ;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        //prepare 2nd line upper sign
        Pel* srcLineBelow = srcLine + srcStride;
        for (x=startX-1; x<endX; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
        }


        //first line
        Pel* srcLineAbove = srcLine - srcStride;
        firstLineStartX = (!isCalculatePreDeblockSamples) ? (isAboveAvail ? startX : endX)
                                                          : startX
                                                          ;
        firstLineEndX   = (!isCalculatePreDeblockSamples) ? ((!isRightAvail && isAboveRightAvail) ? width : endX)
                                                          : endX
                                                          ;
        for(x=firstLineStartX; x<firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) - signUpLine[x-1];
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
          diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
          count[edgeType] ++;
        }

        srcLine += srcStride;
        orgLine += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
        bifLine += bifStride;
#endif
        //middle lines
        for (y=1; y<endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for(x=startX; x<endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              signUpLine[x - 1] = -signDown;
              continue;
            }
            edgeType = signDown + signUpLine[x];
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
            diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
            diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
            count[edgeType] ++;

            signUpLine[x-1] = -signDown;
          }
          signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
          srcLine  += srcStride;
          orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine  += bifStride;
#endif
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = isLeftAvail  ? 0     : 1 ;
            endX   = isRightAvail ? width : (width -1);

            for(y=0; y<skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x=startX; x<endX; x++)
              {
                if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y + endY, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
                {
                  continue;
                }
                edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + sgn(srcLine[x] - srcLineAbove[x+1]);
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                diff [edgeType] += (orgLine[x] - bifLine[x]);
#else
                diff [edgeType] += (orgLine[x] - srcLine[x]);
#endif
                count[edgeType] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
              bifLine  += bifStride;
#endif
            }
          }
        }
      }
      break;
    case SAO_TYPE_BO:
      {
        startX = (!isCalculatePreDeblockSamples)?0
                                                :( isRightAvail?(width- skipLinesR[typeIdx]):width)
                                                ;
        endX   = (!isCalculatePreDeblockSamples)?(isRightAvail ? (width - skipLinesR[typeIdx]) : width )
                                                :width
                                                ;
        endY = isBelowAvail ? (height- skipLinesB[typeIdx]) : height;
        int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
        for (y=0; y< endY; y++)
        {
          for (x=startX; x< endX; x++)
          {

            int bandIdx= srcLine[x] >> shiftBits;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
            diff [bandIdx] += (orgLine[x] - bifLine[x]);
#else
            diff [bandIdx] += (orgLine[x] - srcLine[x]);
#endif
            count[bandIdx] ++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
          bifLine += bifStride;
#endif
        }
        if(isCalculatePreDeblockSamples)
        {
          if(isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for(y= 0; y< skipLinesB[typeIdx]; y++)
            {
              for (x=startX; x< endX; x++)
              {
                int bandIdx= srcLine[x] >> shiftBits;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                diff [bandIdx] += (orgLine[x] - bifLine[x]);
#else
                diff [bandIdx] += (orgLine[x] - srcLine[x]);
#endif
                count[bandIdx] ++;
              }
              srcLine  += srcStride;
              orgLine  += orgStride;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
              bifLine  += bifStride;
#endif
            }

          }
        }
      }
      break;
    default:
      {
        THROW("Not a supported SAO type");
      }
    }
  }
}

#if JVET_W0066_CCSAO
void EncSampleAdaptiveOffset::CCSAOProcess(CodingStructure& cs, const double* lambdas, const int intraPeriod)
{
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
#if JVET_Z0118_GDR
  if( cs.slice->isIDRorBLA() || cs.slice->getPendingRasInit() || cs.slice->isInterGDR() )
#else
  if( cs.slice->isIDRorBLA() || cs.slice->getPendingRasInit() )
#endif
  {
    g_ccSaoPrvParam[COMPONENT_Y ].clear();
    g_ccSaoPrvParam[COMPONENT_Cb].clear();
    g_ccSaoPrvParam[COMPONENT_Cr].clear();
  }
#endif

#if JVET_AJ0237_INTERNAL_12BIT
  if (!cs.slice->isIntra() && !cs.slice->getCheckLDC() && (cs.slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) > 10) && (cs.slice->getSliceQp() > 45) && (m_picWidth * m_picHeight <= 1920 * 1080))
  {
    for (int compIdx = COMPONENT_Y; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      ComponentID compID = (ComponentID)compIdx;
      m_ccSaoComParam.reset(compID);
      memset(m_ccSaoControl[compID], 0, sizeof(uint8_t) * m_numCTUsInPic);
    }
    return;
  }
#endif

  PelUnitBuf orgYuv = cs.getOrgBuf(); 
  PelUnitBuf dstYuv = cs.getRecoBuf();
  PelUnitBuf srcYuv = m_ccSaoBuf.getBuf( cs.area );
  srcYuv.extendBorderPel( MAX_CCSAO_FILTER_LENGTH >> 1 );
  m_intraPeriod = intraPeriod;

  setupCcSaoLambdas(cs, lambdas);
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  setupCcSaoSH(cs, orgYuv);
#endif

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  if (cs.slice->getSPS()->getCCSAOEnabledFlag())
  {
#endif
  const TempCtx ctxStartCcSao(m_ctxCache, SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx()));
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  resetblkStatsEdgePre(m_ccSaoStatDataEdgeNew);
  getCcSaoStatisticsEdgeNew(cs, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdgeNew, m_bestCcSaoParam);
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  for (int compIdx = COMPONENT_Y; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    ComponentID compID = (ComponentID)compIdx;
    resetCcSaoEdgeStats(m_ccSaoStatDataEdgePre);
    prepareCcSaoEdgeStats(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdgePre, m_bestCcSaoParam);
    m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc, ctxStartCcSao); 
    deriveCcSao(cs, compID, orgYuv, srcYuv, dstYuv);
  }
  setupCcSaoPrv(cs);
#else
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc, ctxStartCcSao); deriveCcSao(cs, COMPONENT_Y,  orgYuv, srcYuv, dstYuv);
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc, ctxStartCcSao); deriveCcSao(cs, COMPONENT_Cb, orgYuv, srcYuv, dstYuv);
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc, ctxStartCcSao); deriveCcSao(cs, COMPONENT_Cr, orgYuv, srcYuv, dstYuv);
#endif
  applyCcSao(cs, *cs.pcv, srcYuv, dstYuv);
#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  }
#endif
}

void EncSampleAdaptiveOffset::setupCcSaoLambdas(CodingStructure& cs, const double* lambdas)
{
  m_lambda[COMPONENT_Y ] = m_picWidth * m_picHeight <= 416 * 240 
                         ? lambdas[COMPONENT_Y ] * 4.0 
                         : lambdas[COMPONENT_Y ];
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb];
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr];
}

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::setupCcSaoSH(CodingStructure& cs, const CPelUnitBuf& orgYuv)
{
  Position topLeftLuma = Position(0, 0);
  Size     sizeLuma    = cs.area.lumaSize();

  if( isChromaEnabled( cs.picture->chromaFormat) )
  {
    const CompArea  yArea   = CompArea( COMPONENT_Y,  cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CompArea  cbArea  = CompArea( COMPONENT_Cb, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CompArea  crArea  = CompArea( COMPONENT_Cr, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CPelBuf   orgY    = cs.picture->getOrigBuf( yArea  );
    const CPelBuf   orgCb   = cs.picture->getOrigBuf( cbArea );
    const CPelBuf   orgCr   = cs.picture->getOrigBuf( crArea );

    const int       m0      = ( yArea.x > 0 ? 0 : 1 );
    const int       n0      = ( yArea.y > 0 ? 0 : 1 );
    const int       m1      = ( yArea.x + yArea.width  < cs.picture->Y().width  ? yArea.width  : yArea.width  - 1 );
    const int       n1      = ( yArea.y + yArea.height < cs.picture->Y().height ? yArea.height : yArea.height - 1 );
    const int       x0      = ( cbArea.x > 0 ? 0 : 1 );
    const int       y0      = ( cbArea.y > 0 ? 0 : 1 );
    const int       x1      = ( cbArea.x + cbArea.width  < cs.picture->Cb().width  ? cbArea.width  : cbArea.width  - 1 );
    const int       y1      = ( cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1 );
    const int       ys      = orgY .stride;
    const int       cbs     = orgCb.stride;
    const int       crs     = orgCr.stride;
    const Pel*      pY      = orgY .buf + n0 * ys;
    const Pel*      pCb     = orgCb.buf + y0 * cbs;
    const Pel*      pCr     = orgCr.buf + y0 * crs;

    int             absSumY  = 0;
    int             absSumCb = 0;
    int             absSumCr = 0;
    double          avgY     = 0;
    double          avgCb    = 0;
    double          avgCr    = 0;

    for( int n = n0; n < n1; n++, pY += ys )
    {
      for( int m = m0; m < m1; m++ )
      {
        int y  = ( 12*(int)pY [m] - 2*((int)pY [m-1] + (int)pY [m+1] + (int)pY [m-ys ] + (int)pY [m+ys ]) - ((int)pY [m-1-ys ] + (int)pY [m+1-ys ] + (int)pY [m-1+ys ] + (int)pY [m+1+ys ]) );
        absSumY += abs(y);
      }
    }

    for( int y = y0; y < y1; y++, pCb += cbs, pCr += crs )
    {
      for( int x = x0; x < x1; x++ )
      {
        int cb = ( 12*(int)pCb[x] - 2*((int)pCb[x-1] + (int)pCb[x+1] + (int)pCb[x-cbs] + (int)pCb[x+cbs]) - ((int)pCb[x-1-cbs] + (int)pCb[x+1-cbs] + (int)pCb[x-1+cbs] + (int)pCb[x+1+cbs]) );
        int cr = ( 12*(int)pCr[x] - 2*((int)pCr[x-1] + (int)pCr[x+1] + (int)pCr[x-crs] + (int)pCr[x+crs]) - ((int)pCr[x-1-crs] + (int)pCr[x+1-crs] + (int)pCr[x-1+crs] + (int)pCr[x+1+crs]) );
        absSumCb += abs(cb);
        absSumCr += abs(cr);
      }
    }

    avgY  = (double)absSumY  / (yArea .width * yArea .height);
    avgCb = (double)absSumCb / (cbArea.width * cbArea.height);
    avgCr = (double)absSumCr / (crArea.width * crArea.height);
    m_extChroma = 1.2 * avgCb > avgY || 1.2 * avgCr > avgY;
  }
}
#endif

void EncSampleAdaptiveOffset::deriveCcSao( CodingStructure& cs, const ComponentID compID
                                           , const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv )
{
  double bestCost = 0;
  double tempCost = 0;
  double bestCostS[MAX_CCSAO_SET_NUM + 1] = { 0 };

  double bestCostG[17] = { 0 };
  int    classNumG[17] = { 0 };
  int    stageNum = m_intraPeriod == 1 ? MAX_CCSAO_CLASS_NUM / 4 : MAX_CCSAO_CLASS_NUM / 16;
  for( int stage = 1; stage <= stageNum; stage++ )
  {
    classNumG[stage] = stage * (MAX_CCSAO_CLASS_NUM / stageNum);
  }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  const int edgeCmpNum = m_extChroma ? MAX_NUM_COMPONENT : MAX_NUM_LUMA_COMP;
  const int edgeIdcNum = m_intraPeriod != 1 || cs.sps->getPLTMode() || m_extChroma ? MAX_CCSAO_EDGE_IDC : 1;
#endif

  m_bestCcSaoParam.reset();
  memset( m_bestCcSaoControl, 0, sizeof( uint8_t ) * m_numCTUsInPic );

  for( int setNum = 1; setNum <= MAX_CCSAO_SET_NUM; setNum++ )
  {
    if( setNum > 1 )
    {
      getCcSaoStatistics( cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData, m_bestCcSaoParam );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      getCcSaoStatisticsEdge( cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre, m_bestCcSaoParam );
#else
      getCcSaoStatisticsEdge( cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgeNew, m_bestCcSaoParam );
#endif
#endif
    }
    setupInitCcSaoParam( cs, compID, setNum, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                         , m_ccSaoStatDataEdge, m_ccSaoStatFrameEdge
#endif
                         , m_initCcSaoParam, m_bestCcSaoParam, m_initCcSaoControl, m_bestCcSaoControl );


    for( int stage = 1; stage <= stageNum; stage++ )
    {
      for( int bandNumY = 1; bandNumY <= MAX_CCSAO_BAND_NUM_Y; bandNumY++ )
      {
        for( int bandNumU = 1; bandNumU <= MAX_CCSAO_BAND_NUM_U; bandNumU++ )
        {
          for( int bandNumV = 1; bandNumV <= MAX_CCSAO_BAND_NUM_V; bandNumV++ )
          {
            for( int candPosY = 0; candPosY < MAX_CCSAO_CAND_POS_Y && bandNumY > 1; candPosY++ )
            {
              if( bandNumY < bandNumU || bandNumY < bandNumV )
              {
                continue;
              }

              int classNum = bandNumY * bandNumU * bandNumV;
              if( classNum > MAX_CCSAO_CLASS_NUM )
              {
                continue;
              }

              if( classNum <= classNumG[stage - 1] || classNum > classNumG[stage] )
              {
                continue;
              }

#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
              setupTempCcSaoParam( cs, compID, setNum
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                                   , 0 /*dummy*/
#endif
                                   , candPosY, bandNumY, bandNumU, bandNumV
                                   , m_tempCcSaoParam, m_initCcSaoParam, m_tempCcSaoControl, m_initCcSaoControl /*, 0 (default Band type*/ );
#else
              setupTempCcSaoParam( cs, compID, setNum, candPosY, bandNumY, bandNumU, bandNumV, m_tempCcSaoParam,
                                   m_initCcSaoParam, m_tempCcSaoControl, m_initCcSaoControl );
#endif
              getCcSaoStatistics( cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData, m_tempCcSaoParam );
              deriveCcSaoRDO( cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                              , m_ccSaoStatDataEdge, m_ccSaoStatFrameEdge
#endif
                              , m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl, m_tempCcSaoControl, bestCost, tempCost );
            }
          }
        }
      }

      bestCostG[stage] = bestCost;
      if( bestCostG[stage] >= bestCostG[stage - 1] )
      {
        break;
      }
    }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    for (int edgeCmp = COMPONENT_Y; edgeCmp < edgeCmpNum; edgeCmp++)
    {
      for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
      {
        for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
        {
          for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
          {
            for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
            {
              const int edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
              const int bandNum = g_ccSaoBandTab[bandIdc][1];
              if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
              {
                continue;
              }
              
              setupTempCcSaoParam(cs, compID, setNum
                                , edgeCmp
                                , edgeDir, bandIdc, edgeThr
                                , edgeIdc
                                , m_tempCcSaoParam, m_initCcSaoParam, m_tempCcSaoControl, m_initCcSaoControl, CCSAO_SET_TYPE_EDGE);
#else
    tempCost = 0;
    for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
    {
      for (int mode = 0; mode < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; mode++)
      {
        for (int th = 0; th < CCSAO_QUAN_NUM; th++)
        {
          setupTempCcSaoParam(cs, compID, setNum, type, mode, th, th, m_tempCcSaoParam, m_initCcSaoParam,
                              m_tempCcSaoControl, m_initCcSaoControl, 1 /* Edge Class Type */);
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
              getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre, m_tempCcSaoParam);
#else
              getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgeNew, m_tempCcSaoParam);
#endif
              deriveCcSaoRDO(cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                           , m_ccSaoStatDataEdge, m_ccSaoStatFrameEdge
#endif
                       , m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl, m_tempCcSaoControl, bestCost, tempCost);
            }
          }
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        }
#endif
      }
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    }
#endif
#endif
    bestCostS[setNum] = bestCost;
    if (bestCostS[setNum] >= bestCostS[setNum - 1])
    {
      break;
    }
  }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  if (!cs.slice->isIntra())
  {
    for (int prvId = 0; prvId < g_ccSaoPrvParam[compID].size(); prvId++)
    {
      if (g_ccSaoPrvParam[compID][prvId].temporalId > cs.slice->getTLayer())
      {
        continue;
      }

#if JVET_AL0142_CCSAO_REUSE_CTU
      for (int reuseMode = CCSAO_REUSE_PARAM; reuseMode < NUM_CCSAO_REUSE_MODES; reuseMode++)
      {
        setupTempCcSaoParamFromPrv(cs, compID, prvId, reuseMode, m_tempCcSaoParam, g_ccSaoPrvParam[compID][prvId], m_tempCcSaoControl);
#else
        setupTempCcSaoParamFromPrv(cs, compID, prvId,            m_tempCcSaoParam, g_ccSaoPrvParam[compID][prvId], m_tempCcSaoControl);
#endif

        getCcSaoStatistics    (cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData,                             m_tempCcSaoParam);
        getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre, m_tempCcSaoParam);

        deriveCcSaoRDO(cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame
                     , m_ccSaoStatDataEdge,  m_ccSaoStatFrameEdge
                     , m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl, m_tempCcSaoControl, bestCost, tempCost);
#if JVET_AL0142_CCSAO_REUSE_CTU
      }
#endif
    }
  }
#endif

  bool oneBlockFiltered = false;
  for (int ctbIdx = 0; m_bestCcSaoParam.setNum > 0 && ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    if (m_bestCcSaoControl[ctbIdx])
    {
      oneBlockFiltered = true;
      break;
    }
  }
  
  m_ccSaoComParam.reset(compID);
  memset(m_ccSaoControl[compID], 0, sizeof(uint8_t) * m_numCTUsInPic);

  m_ccSaoComParam.enabled  [compID] = oneBlockFiltered;
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  m_ccSaoComParam.extChroma[compID] = m_extChroma;
#endif
  if (oneBlockFiltered)
  {
    CcSaoEncParam storedBestCcSaoParam = m_bestCcSaoParam;
    memcpy(m_tempCcSaoControl, m_bestCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);

    int setNum = 0;
    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      uint8_t setIdc = m_bestCcSaoParam.mapIdxToIdc[setIdx];
      if (m_bestCcSaoParam.setEnabled[setIdx])
      {
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          if (m_tempCcSaoControl[ctbIdx] == (setIdx + 1) )
          {
            m_bestCcSaoControl[ctbIdx] = setIdc;
          }
        }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
        m_bestCcSaoParam.setType[setIdc - 1]               = storedBestCcSaoParam.setType[setIdx];
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        m_bestCcSaoParam.candPos[setIdc - 1][COMPONENT_Cb] = storedBestCcSaoParam.candPos[setIdx][COMPONENT_Cb];
#endif
        m_bestCcSaoParam.candPos[setIdc - 1][COMPONENT_Y ] = storedBestCcSaoParam.candPos[setIdx][COMPONENT_Y ];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMPONENT_Y ] = storedBestCcSaoParam.bandNum[setIdx][COMPONENT_Y ];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMPONENT_Cb] = storedBestCcSaoParam.bandNum[setIdx][COMPONENT_Cb];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMPONENT_Cr] = storedBestCcSaoParam.bandNum[setIdx][COMPONENT_Cr];
        memcpy(m_bestCcSaoParam.offset[setIdc - 1], storedBestCcSaoParam.offset[setIdx], sizeof(storedBestCcSaoParam.offset[setIdx]));
        setNum++;
      }
      m_bestCcSaoParam.setEnabled[setIdx] = setIdx < m_bestCcSaoParam.setNum ? true : false;
    }
    CHECK(setNum != m_bestCcSaoParam.setNum, "Number of sets enabled != setNum");

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    m_ccSaoComParam.reusePrv  [compID] = m_bestCcSaoParam.reusePrv;
    m_ccSaoComParam.reusePrvId[compID] = m_bestCcSaoParam.reusePrvId;
#endif
    m_ccSaoComParam.setNum    [compID] = m_bestCcSaoParam.setNum;

    for ( int setIdx = 0; setIdx < m_bestCcSaoParam.setNum; setIdx++ )
    {
      m_ccSaoComParam.setEnabled[compID][setIdx]               = m_bestCcSaoParam.setEnabled[setIdx];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
      m_ccSaoComParam.setType[compID][setIdx] = m_bestCcSaoParam.setType[setIdx];
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      m_ccSaoComParam.candPos   [compID][setIdx][COMPONENT_Cb] = m_bestCcSaoParam.candPos   [setIdx][COMPONENT_Cb];
#endif
      m_ccSaoComParam.candPos   [compID][setIdx][COMPONENT_Y ] = m_bestCcSaoParam.candPos   [setIdx][COMPONENT_Y ];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      int offset = m_ccSaoComParam.setType[compID][setIdx] ? 1 : 0;
#endif
      m_ccSaoComParam.bandNum   [compID][setIdx][COMPONENT_Y ] = m_bestCcSaoParam.bandNum   [setIdx][COMPONENT_Y ]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      + offset;
#else
        ;
#endif
      m_ccSaoComParam.bandNum   [compID][setIdx][COMPONENT_Cb] = m_bestCcSaoParam.bandNum   [setIdx][COMPONENT_Cb]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      + offset;
#else
        ;
#endif
      m_ccSaoComParam.bandNum   [compID][setIdx][COMPONENT_Cr] = m_bestCcSaoParam.bandNum   [setIdx][COMPONENT_Cr]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      + offset;
#else
        ;
#endif
      memcpy(m_ccSaoComParam.offset[compID][setIdx], m_bestCcSaoParam.offset[setIdx], sizeof(m_bestCcSaoParam.offset[setIdx]));
    }
    memcpy(m_ccSaoControl[compID], m_bestCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);
  }
}

void EncSampleAdaptiveOffset::setupInitCcSaoParam(CodingStructure& cs, const ComponentID compID, const int setNum, int64_t* trainingDistortion[MAX_CCSAO_SET_NUM]
                                                , CcSaoStatData* blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                                                , CcSaoStatData* blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM]
#endif
                                                , CcSaoEncParam& initCcSaoParam, CcSaoEncParam& bestCcSaoParam
                                                , uint8_t* initCcSaoControl, uint8_t* bestCcSaoControl)
{
  initCcSaoParam.reset();
  memset(initCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);

#if JVET_AJ0237_INTERNAL_12BIT
  const int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->getBitDepth(CHANNEL_TYPE_LUMA));
#endif

  if (setNum == 1)
  {
    std::fill_n(initCcSaoControl, m_numCTUsInPic, 1);
    return;
  }

  for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
  {
    if (bestCcSaoParam.setEnabled[setIdx])
    {
      getCcSaoFrameStats(compID, setIdx, bestCcSaoControl, blkStats, frameStats
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                       , blkStatsEdge, frameStatsEdge, bestCcSaoParam.setType[setIdx]
#endif
      );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      getCcSaoDistortion(compID, setIdx
                       , bestCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND ? blkStats : blkStatsEdge
#if JVET_AJ0237_INTERNAL_12BIT
                       , bestCcSaoParam.offset, trainingDistortion, shift);
#else
                       , bestCcSaoParam.offset, trainingDistortion);
#endif
#else
      if (bestCcSaoParam.setType[setIdx] == 0) /* band */
      {
        getCcSaoDistortion(compID, setIdx, blkStats, bestCcSaoParam.offset, trainingDistortion);
      }
      else /* Edge */
      {
        getCcSaoDistortionEdge(compID, setIdx, blkStatsEdge, bestCcSaoParam.offset, trainingDistortion);
      }
#endif
#else
      getCcSaoDistortion(compID, setIdx, blkStats, bestCcSaoParam.offset, trainingDistortion);
#endif
    }
  }

  initCcSaoParam = bestCcSaoParam;

  int ctbCntOn = 0;
  CtbCost *ctbCost = new CtbCost[m_numCTUsInPic];

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    int64_t dist = 0;

    if (bestCcSaoControl[ctbIdx])
    {
      int setIdx = bestCcSaoControl[ctbIdx] - 1;
      dist = trainingDistortion[setIdx][ctbIdx];
      ctbCntOn++;
    }

    ctbCost[ctbIdx].pos = ctbIdx;
    ctbCost[ctbIdx].cost = (double)dist;
  }

  std::stable_sort(ctbCost, ctbCost + m_numCTUsInPic, compareCtbCost);

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    int ctbPos = ctbCost[ctbIdx].pos;
    if (ctbIdx < ctbCntOn)
    {
      if (ctbIdx * 2 > ctbCntOn)
      {
        initCcSaoControl[ctbPos] = setNum;
      }
      else
      {
        initCcSaoControl[ctbPos] = bestCcSaoControl[ctbPos];
      }
    }
    else
    {
      initCcSaoControl[ctbPos] = 0;
    }
  }
  
  delete[] ctbCost;
  ctbCost = nullptr;
}

void EncSampleAdaptiveOffset::setupTempCcSaoParam(CodingStructure& cs, const ComponentID compID, const int setNum
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                                                , const int edgeCmp
#endif
                                                , const int candPosY, const int bandNumY, const int bandNumU, const int bandNumV
                                                , CcSaoEncParam& tempCcSaoParam, CcSaoEncParam& initCcSaoParam
                                                , uint8_t* tempCcSaoControl, uint8_t* initCcSaoControl
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                                                , int setType
#endif
                                                 )
{
  tempCcSaoParam.reset();
  memset(tempCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);

  tempCcSaoParam = initCcSaoParam;
  memcpy(tempCcSaoControl, initCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);

  tempCcSaoParam.setNum = setNum;
  tempCcSaoParam.setEnabled[setNum - 1] = true;
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  tempCcSaoParam.setType   [setNum - 1] = setType;
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  tempCcSaoParam.candPos   [setNum - 1][COMPONENT_Cb] = edgeCmp;
#endif
  tempCcSaoParam.candPos   [setNum - 1][COMPONENT_Y ] = candPosY;
  tempCcSaoParam.bandNum   [setNum - 1][COMPONENT_Y ] = bandNumY;
  tempCcSaoParam.bandNum   [setNum - 1][COMPONENT_Cb] = bandNumU;
  tempCcSaoParam.bandNum   [setNum - 1][COMPONENT_Cr] = bandNumV;

  CHECK( setNum > MAX_CCSAO_SET_NUM, "setNum exceeds the buffer size" );

  for (int setIdx = 0; setIdx <= setNum; setIdx++)
  {
    tempCcSaoParam.mapIdxToIdc[setIdx] = setIdx < setNum ? setIdx + 1 : 0;
  }
}

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::setupTempCcSaoParamFromPrv(CodingStructure& cs, const ComponentID compID, const int prvId
#if JVET_AL0142_CCSAO_REUSE_CTU
                                                       , const int reuseMode
#endif
                                                       , CcSaoEncParam& tempCcSaoParam, CcSaoPrvParam& prvCcSaoParam
                                                       , uint8_t* tempCcSaoControl)
{
  tempCcSaoParam.reset();
  memset(tempCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  std::fill_n(tempCcSaoControl, m_numCTUsInPic, 1);

#if JVET_AL0142_CCSAO_REUSE_CTU
  tempCcSaoParam.reusePrv   = reuseMode;
#else
  tempCcSaoParam.reusePrv   = true;
#endif
  tempCcSaoParam.reusePrvId = prvId;
  tempCcSaoParam.setNum     = prvCcSaoParam.setNum;
  memcpy( tempCcSaoParam.setEnabled, prvCcSaoParam.setEnabled, sizeof( tempCcSaoParam.setEnabled ) );
  memcpy( tempCcSaoParam.setType   , prvCcSaoParam.setType   , sizeof( tempCcSaoParam.setType    ) );
  memcpy( tempCcSaoParam.candPos   , prvCcSaoParam.candPos   , sizeof( tempCcSaoParam.candPos    ) );
  memcpy( tempCcSaoParam.bandNum   , prvCcSaoParam.bandNum   , sizeof( tempCcSaoParam.bandNum    ) );
  memcpy( tempCcSaoParam.offset    , prvCcSaoParam.offset    , sizeof( tempCcSaoParam.offset     ) );
#if JVET_AL0142_CCSAO_REUSE_CTU
  if (reuseMode == CCSAO_REUSE_PARAM_CTU)
  {
    memcpy( tempCcSaoControl, prvCcSaoParam.ccSaoControl, sizeof(uint8_t) * m_numCTUsInPic );
  }
#endif

  CHECK( tempCcSaoParam.setNum > MAX_CCSAO_SET_NUM, "setNum exceeds the buffer size" );

  for (int setIdx = 0; setIdx <= tempCcSaoParam.setNum; setIdx++)
  {
    tempCcSaoParam.mapIdxToIdc[setIdx] = setIdx < tempCcSaoParam.setNum ? setIdx + 1 : 0;
  }
}
#endif

void EncSampleAdaptiveOffset::getCcSaoStatistics(CodingStructure& cs, const ComponentID compID
                                               , const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv
                                               , CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM], const CcSaoEncParam& ccSaoParam)
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues& pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail );

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int numHorVirBndry = 0, numVerVirBndry = 0;
      int horVirBndryPos[] = { -1,-1,-1 };
      int verVirBndryPos[] = { -1,-1,-1 };
      int horVirBndryPosComp[] = { -1,-1,-1 };
      int verVirBndryPosComp[] = { -1,-1,-1 };
      bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader);
#endif

      //NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      //For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth  < pcv.lumaWidth );
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
      {
        blkStats[setIdx][ctuRsAddr].reset();
        if (!ccSaoParam.setEnabled[setIdx])
          continue;
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        if (ccSaoParam.setType[setIdx] != CCSAO_SET_TYPE_BAND)
#else
        if (ccSaoParam.setType[setIdx] != 0)
#endif
        {
          continue;
        }
#endif
        const CompArea   &compArea   = area.block(compID);
        const int         srcStrideY = srcYuv.get(COMPONENT_Y ).stride;
        const int         srcStrideU = srcYuv.get(COMPONENT_Cb).stride;
        const int         srcStrideV = srcYuv.get(COMPONENT_Cr).stride;
        const Pel        *srcBlkY    = srcYuv.get(COMPONENT_Y ).bufAt(area.block(COMPONENT_Y ));
        const Pel        *srcBlkU    = srcYuv.get(COMPONENT_Cb).bufAt(area.block(COMPONENT_Cb));
        const Pel        *srcBlkV    = srcYuv.get(COMPONENT_Cr).bufAt(area.block(COMPONENT_Cr));
        const int         dstStride  = dstYuv.get(compID      ).stride;
        const int         orgStride  = orgYuv.get(compID      ).stride;
        const Pel        *dstBlk     = dstYuv.get(compID      ).bufAt(compArea);
        const Pel        *orgBlk     = orgYuv.get(compID      ).bufAt(compArea);

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        for (int i = 0; i < numHorVirBndry; i++)
        {
          horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
        }
        for (int i = 0; i < numVerVirBndry; i++)
        {
          verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
        }
#endif

        const uint16_t    candPosY   = ccSaoParam.candPos[setIdx][COMPONENT_Y ];
        const uint16_t    bandNumY   = ccSaoParam.bandNum[setIdx][COMPONENT_Y ];
        const uint16_t    bandNumU   = ccSaoParam.bandNum[setIdx][COMPONENT_Cb];
        const uint16_t    bandNumV   = ccSaoParam.bandNum[setIdx][COMPONENT_Cr];

        getCcSaoBlkStats(compID, cs.area.chromaFormat, cs.sps->getBitDepth(toChannelType(compID))
                       , setIdx, blkStats, ctuRsAddr
                       , candPosY, bandNumY, bandNumU, bandNumV
                       , srcBlkY, srcBlkU, srcBlkV, orgBlk, dstBlk
                       , srcStrideY, srcStrideU, srcStrideV, orgStride, dstStride, compArea.width, compArea.height
                       , isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                       , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
                        );
      }
      ctuRsAddr++;
    }
  }
}
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::resetCcSaoEdgeStats(CcSaoStatData *blkStatsEdge)
{
  int numStatsEdge = m_numCTUsInPic * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR
                   * MAX_NUM_COMPONENT * MAX_CCSAO_EDGE_IDC;

  for (int idx = 0; idx < numStatsEdge; idx++)
  {
    blkStatsEdge[idx].reset();
  }
}
#else
void EncSampleAdaptiveOffset::resetblkStatsEdgePre(CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM - 1])
{
  for (int comp = Y_C; comp < N_C; comp++)
  {
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      for (int mode = 0; mode < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; mode++)
      {
        for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
        {
          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            int index = calcEdgeStatIndex(ctbIdx, mode, type, th);
            blkStatsEdge[comp][index].reset();
          }
        }
      }
    }
  }
}
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::prepareCcSaoEdgeStats(CodingStructure& cs, const ComponentID compID
                                                  , const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv 
                                                  , CcSaoStatData* blkStatsEdge, const CcSaoEncParam& ccSaoParam)
#else
void EncSampleAdaptiveOffset::getCcSaoStatisticsEdgeNew(CodingStructure &cs, const CPelUnitBuf &orgYuv,
                                                        const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv,
                                                        CcSaoStatData *      blkStatsEdge[MAX_CCSAO_SET_NUM - 1],
                                                        const CcSaoEncParam &ccSaoParam)
#endif
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues &pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail);

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int numHorVirBndry = 0, numVerVirBndry = 0;
      int horVirBndryPos[] = { -1,-1,-1 };
      int verVirBndryPos[] = { -1,-1,-1 };
      int horVirBndryPosComp[] = { -1,-1,-1 };
      int verVirBndryPosComp[] = { -1,-1,-1 };
      bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader);
#endif

      // NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      // For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth < pcv.lumaWidth);
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      for (int comp = Y_C; comp < N_C; comp++)
      {
        const ComponentID compID     = ComponentID(comp);
#endif
        const CompArea   &compArea   = area.block(compID);
        const int         srcStrideY = srcYuv.get(COMPONENT_Y ).stride;
        const int         srcStrideU = srcYuv.get(COMPONENT_Cb).stride;
        const int         srcStrideV = srcYuv.get(COMPONENT_Cr).stride;
        const Pel        *srcBlkY    = srcYuv.get(COMPONENT_Y ).bufAt(area.block(COMPONENT_Y ));
        const Pel        *srcBlkU    = srcYuv.get(COMPONENT_Cb).bufAt(area.block(COMPONENT_Cb));
        const Pel        *srcBlkV    = srcYuv.get(COMPONENT_Cr).bufAt(area.block(COMPONENT_Cr));
        const int         dstStride  = dstYuv.get(compID      ).stride;
        const int         orgStride  = orgYuv.get(compID      ).stride;
        const Pel        *dstBlk     = dstYuv.get(compID      ).bufAt(compArea);
        const Pel        *orgBlk     = orgYuv.get(compID      ).bufAt(compArea);

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        for (int i = 0; i < numHorVirBndry; i++)
        {
          horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
        }
        for (int i = 0; i < numVerVirBndry; i++)
        {
          verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
        }
#endif

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const uint16_t candPosY = 0;
        const uint16_t bandNumC = 0;
        const int      setIdx   = 0;
        const uint16_t mode     = 0; /* temporary setting */
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        getCcSaoBlkStatsEdgePre(cs, compID, cs.area.chromaFormat, cs.sps->getBitDepth(toChannelType(compID))
                              , m_ccSaoStatDataEdgePre, ctuRsAddr
#else
        getCcSaoBlkStatsEdgeNew(compID, cs.area.chromaFormat, cs.sps->getBitDepth(toChannelType(compID))
                              , setIdx, m_ccSaoStatDataEdgeNew, ctuRsAddr
                              , candPosY, mode, bandNumC, bandNumC
#endif
                              , srcBlkY, srcBlkU, srcBlkV, orgBlk, dstBlk, srcStrideY, srcStrideU, srcStrideV, orgStride, dstStride
                              , compArea.width, compArea.height
                              , isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                              , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
                               );
#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      }
#endif
      ctuRsAddr++;
    }
  }
}

void EncSampleAdaptiveOffset::getCcSaoStatisticsEdge(CodingStructure& cs, const ComponentID compID
                                                   , const CPelUnitBuf& orgYuv, const CPelUnitBuf& srcYuv, const CPelUnitBuf& dstYuv
                                                   , CcSaoStatData* blkStatsEdge[MAX_CCSAO_SET_NUM]
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                                                   , CcSaoStatData* blkStatsEdgePre
#else
                                                   , CcSaoStatData* blkStatsEdgePre[MAX_CCSAO_SET_NUM - 1]
#endif
                                                   , const CcSaoEncParam& ccSaoParam)
{
  const PreCalcValues &pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
      {
        blkStatsEdge[setIdx][ctuRsAddr].reset();
        if (!ccSaoParam.setEnabled[setIdx])
        {
          continue;
        }
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        if (ccSaoParam.setType[setIdx] != CCSAO_SET_TYPE_EDGE)
#else
        if (ccSaoParam.setType[setIdx] != 1)
#endif
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const uint16_t edgeCmp = ccSaoParam.candPos[setIdx][COMPONENT_Cb];
        const uint16_t edgeIdc = ccSaoParam.bandNum[setIdx][COMPONENT_Cr];
        const uint16_t edgeDir = ccSaoParam.candPos[setIdx][COMPONENT_Y ];
        const uint16_t edgeThr = ccSaoParam.bandNum[setIdx][COMPONENT_Cb];
        const uint16_t bandIdc = ccSaoParam.bandNum[setIdx][COMPONENT_Y ];

        int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                    , edgeCmp, edgeIdc
                                    , edgeDir, edgeThr
                                     );
        blkStatsEdge[setIdx][ctuRsAddr] = blkStatsEdgePre[idx];
#else
        const uint16_t candPosY = ccSaoParam.candPos[setIdx][COMPONENT_Y];
        const uint16_t bandNumY = ccSaoParam.bandNum[setIdx][COMPONENT_Y];
        const uint16_t bandNumC = ccSaoParam.bandNum[setIdx][COMPONENT_Cb];   // treshold

        int index                       = calcEdgeStatIndex(ctuRsAddr, bandNumY, candPosY, bandNumC);
        blkStatsEdge[setIdx][ctuRsAddr] = blkStatsEdgePre[compID][index];
#endif
      }
      ctuRsAddr++;
    }
  }
}

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
int calcDiffRangeEnc(Pel a, Pel b, int th)
{
  int diff      = a - b;
  int value     = 0;
  int thred     = g_ccSaoQuanValue[th];
  int neg_thred = (-1) * thred;
  if (diff < 0)
  {
    if (diff < neg_thred)
    {
      value = 0;
    }
    else
    {
      value = 1;
    }
  }
  else
  {
    if (diff < thred)
    {
      value = 2;
    }
    else
    {
      value = 3;
    }
  }
  return value;
}
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
inline int EncSampleAdaptiveOffset::getCcSaoEdgeStatIdx(const int ctuRsAddr, const int bandIdc
                                                      , const int edgeCmp, const int edgeIdc
                                                      , const int edgeDir, const int edgeThr
                                                       )
{
  int idx = ctuRsAddr * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR
           + bandIdc                       * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR
           + edgeDir                                            * MAX_CCSAO_EDGE_THR 
           + edgeThr;
  idx = idx * MAX_NUM_COMPONENT + edgeCmp;
  idx = idx * MAX_CCSAO_EDGE_IDC + edgeIdc;

  return idx;
}
#else
int EncSampleAdaptiveOffset::calcEdgeStatIndex(const int ctuRsAddr, const int mode, const int type, const int th)
{
  int index = 0;
  index     = ctuRsAddr * (CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C) * CCSAO_EDGE_TYPE * CCSAO_QUAN_NUM
            + mode * CCSAO_EDGE_TYPE * CCSAO_QUAN_NUM + type * CCSAO_QUAN_NUM + th;
  return index;
}
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::getCcSaoBlkStatsEdgePre(CodingStructure& cs, const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                                                    , CcSaoStatData *blkStatsEdge, const int ctuRsAddr
#else
void EncSampleAdaptiveOffset::getCcSaoBlkStatsEdgeNew(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                                                    , const int setIdx, CcSaoStatData *blkStatsEdge[N_C], const int ctuRsAddr
                                                    , const uint16_t candPosY, const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
#endif
                                                    , const Pel *srcY, const Pel *srcU, const Pel *srcV
                                                    , const Pel *org, const Pel *dst
                                                    , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int orgStride, const int dstStride
                                                    , const int width, const int height
                                                    , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                                    , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                                                     )
{
#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  int signa, signb, band;
#endif

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleYM1 = 1 - chromaScaleY;

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  const int edgeCmpNum = m_extChroma ? MAX_NUM_COMPONENT : MAX_NUM_LUMA_COMP;
  const int edgeIdcNum = m_intraPeriod != 1 || cs.sps->getPLTMode() || m_extChroma ? MAX_CCSAO_EDGE_IDC : 1;
  const int srcStrideTab[MAX_NUM_COMPONENT] = { srcStrideY, srcStrideU, srcStrideU };
#endif

#if JVET_AJ0237_INTERNAL_12BIT
  const int bdShift = std::max(0, bitDepth - 10);
#endif

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int x, y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX;
  const Pel *srcYT = srcY;
  const Pel *srcUT = srcU;
  const Pel *srcVT = srcV;
  const Pel *orgT  = org;
  const Pel *dstT  = dst;

  switch (compID)
  {
  case COMPONENT_Y:
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    for (int edgeCmp = COMPONENT_Y; edgeCmp < edgeCmpNum; edgeCmp++)
    {
      const int srcStrideE = srcStrideTab[edgeCmp];
      for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
      {
        srcY = srcYT;
        srcU = srcUT;
        srcV = srcVT;
        org  = orgT;
        dst  = dstT;
        int edgePosXA = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
        int edgePosXB = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];

        switch (edgeDir)
        {
        case SAO_TYPE_EO_0:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);
          for (y = 0; y < height; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + x;
              const Pel *colU = srcU + (x >> chromaScaleX);
              const Pel *colV = srcV + (x >> chromaScaleX);
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                 );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_90:
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = isBelowAvail ? height : height - 1;
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            org += orgStride;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            for (x = 0; x < width; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + x;
              const Pel *colU = srcU + (x >> chromaScaleX);
              const Pel *colV = srcV + (x >> chromaScaleX);
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                 );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_135:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          for (x = firstLineStartX; x < firstLineEndX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }

            const Pel *colY = srcY + x;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);
            const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
            const Pel *colE = col[edgeCmp];
            const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
            const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

            for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
            {
              const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
              const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
              for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
              {
#if JVET_AJ0237_INTERNAL_12BIT
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                {
                  const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                  const int bandNum = g_ccSaoBandTab[bandIdc][1];
                  if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                  {
                    continue;
                  }

                  const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                  const int classIdx = bandIdx * edgeNum + edgeIdx;

                  int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                              , edgeCmp, edgeIdc
                                              , edgeDir, edgeThr
                                               );

                  blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                  blkStatsEdge[idx].count[classIdx]++;
                }
              }
            }
          }
          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + x;
              const Pel *colU = srcU + (x >> chromaScaleX);
              const Pel *colV = srcV + (x >> chromaScaleX);
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                 );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_45:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);
          
          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          for (x = firstLineStartX; x < firstLineEndX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }

            const Pel *colY = srcY + x;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);
            const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
            const Pel *colE = col[edgeCmp];
            const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
            const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

            for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
            {
              const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
              const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
              for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
              {
#if JVET_AJ0237_INTERNAL_12BIT
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                {
                  const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                  const int bandNum = g_ccSaoBandTab[bandIdc][1];
                  if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                  {
                    continue;
                  }

                  const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                  const int classIdx = bandIdx * edgeNum + edgeIdx;

                  int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                              , edgeCmp, edgeIdc
                                              , edgeDir, edgeThr
                                               );

                  blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                  blkStatsEdge[idx].count[classIdx]++;
                }
              }
            }
          }
          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + x;
              const Pel *colU = srcU + (x >> chromaScaleX);
              const Pel *colV = srcV + (x >> chromaScaleX);
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                 );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        }
      }
    }
#else
    for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
    {
      srcY           = srcYT;
      srcU           = srcUT;
      srcV           = srcVT;
      org            = orgT;
      dst            = dstT;
      int candPosYXA = g_ccSaoEdgeTypeX[type][0];
      int candPosYYA = g_ccSaoEdgeTypeY[type][0];
      int candPosYXB = g_ccSaoEdgeTypeX[type][1];
      int candPosYYB = g_ccSaoEdgeTypeY[type][1];
      switch (type)
      {
      case SAO_TYPE_EO_0:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        for (y = 0; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries
                && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x;
            const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);

              signa = signa * 4 + signb;

              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colU * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colV * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;

              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_0
      break;
      case SAO_TYPE_EO_90:
      {
        startY = isAboveAvail ? 0 : 1;
        endY   = isBelowAvail ? height : height - 1;
        if (!isAboveAvail)
        {
          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          org += orgStride;
          dst += dstStride;
        }
        for (y = startY; y < endY; y++)
        {
          for (x = 0; x < width; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x;
            const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);

              signa = signa * 4 + signb;

              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colU * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colV * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;

              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_90
      break;
      case SAO_TYPE_EO_135:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);

            signa = signa * 4 + signb;

            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colU * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colV * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;

            }   // mode
          }     // th
        }       // x
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        org += orgStride;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height - 1; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x;
            const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);

              signa = signa * 4 + signb;

              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colU * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colV * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;

              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          org += orgStride;
          dst += dstStride;
        }   // y

      }   // case SAO_TYPE_EO_135
      break;
      case SAO_TYPE_EO_45:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        // first line
        firstLineStartX = isAboveAvail ? startX : (width - 1);
        firstLineEndX   = isAboveRightAvail ? width : (width - 1);
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);

            signa = signa * 4 + signb;

            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colU * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colV * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;

            }   // mode
          }     // th
        }       // x
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        org += orgStride;
        dst += dstStride;
        // middle lines
        for (y = 1; y < height - 1; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x;
            const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);

              signa = signa * 4 + signb;

              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colU * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colV * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;

              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_45
      break;
      }   // switch (type)
    }     // for(type =0; type < CCSAO_EDGE_TYPE; type ++)
#endif
    break;
  }   // case COMPONENT_Y
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    for (int edgeCmp = COMPONENT_Y; edgeCmp < edgeCmpNum; edgeCmp++)
    {
      const int srcStrideE = srcStrideTab[edgeCmp];
      for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
      {
        srcY = srcYT;
        srcU = srcUT;
        srcV = srcVT;
        org  = orgT;
        dst  = dstT;
        int edgePosXA = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
        int edgePosXB = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];
        
        switch (edgeDir)
        {
        case SAO_TYPE_EO_0:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);
          for (y = 0; y < height; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }
              
              const Pel *colY = srcY + (x << chromaScaleX);
              const Pel *colU = srcU + x;
              const Pel *colV = srcV + x;
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_90:
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = isBelowAvail ? height : height - 1;
          if (!isAboveAvail)
          {
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            for (x = 0; x < width; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + (x << chromaScaleX);
              const Pel *colU = srcU + x;
              const Pel *colV = srcV + x;
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_135:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          for (x = firstLineStartX; x < firstLineEndX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }

            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colU = srcU + x;
            const Pel *colV = srcV + x;
            const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
            const Pel *colE = col[edgeCmp];
            const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
            const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

            for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
            {
              const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
              const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
              for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
              {
#if JVET_AJ0237_INTERNAL_12BIT
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                {
                  const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                  const int bandNum = g_ccSaoBandTab[bandIdc][1];
                  if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                  {
                    continue;
                  }

                  const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                  const int classIdx = bandIdx * edgeNum + edgeIdx;

                  int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                              , edgeCmp, edgeIdc
                                              , edgeDir, edgeThr
                                              );

                  blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                  blkStatsEdge[idx].count[classIdx]++;
                }
              }
            }
          }
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + (x << chromaScaleX);
              const Pel *colU = srcU + x;
              const Pel *colV = srcV + x;
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        case SAO_TYPE_EO_45:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          for (x = firstLineStartX; x < firstLineEndX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }

            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colU = srcU + x;
            const Pel *colV = srcV + x;
            const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
            const Pel *colE = col[edgeCmp];
            const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
            const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

            for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
            {
              const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
              const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
              for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
              {
#if JVET_AJ0237_INTERNAL_12BIT
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
                for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                {
                  const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                  const int bandNum = g_ccSaoBandTab[bandIdc][1];
                  if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                  {
                    continue;
                  }

                  const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                  const int classIdx = bandIdx * edgeNum + edgeIdx;

                  int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                              , edgeCmp, edgeIdc
                                              , edgeDir, edgeThr
                                              );

                  blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                  blkStatsEdge[idx].count[classIdx]++;
                }
              }
            }
          }
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            for (x = startX; x < endX; x++)
            {
              if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
              {
                continue;
              }

              const Pel *colY = srcY + (x << chromaScaleX);
              const Pel *colU = srcU + x;
              const Pel *colV = srcV + x;
              const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
              const Pel *colE = col[edgeCmp];
              const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
              const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

              for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
              {
                const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
                const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
                for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
                {
#if JVET_AJ0237_INTERNAL_12BIT
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
                  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
#endif
                  const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
                  const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
                  const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;

                  for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
                  {
                    const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                    const int bandNum = g_ccSaoBandTab[bandIdc][1];
                    if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                    {
                      continue;
                    }

                    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                    const int classIdx = bandIdx * edgeNum + edgeIdx;

                    int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                                , edgeCmp, edgeIdc
                                                , edgeDir, edgeThr
                                                );

                    blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                    blkStatsEdge[idx].count[classIdx]++;
                  }
                }
              }
            }
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
        }
        break;
        }
      }
    }
#else
    for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
    {
      srcY = srcYT;
      srcU = srcUT;
      srcV = srcVT;
      org  = orgT;
      dst  = dstT;

      int candPosYXA = g_ccSaoEdgeTypeX[type][0];
      int candPosYYA = g_ccSaoEdgeTypeY[type][0];
      int candPosYXB = g_ccSaoEdgeTypeX[type][1];
      int candPosYYB = g_ccSaoEdgeTypeY[type][1];
      switch (type)
      {
      case SAO_TYPE_EO_0:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        for (y = 0; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

            const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            const Pel *colCT = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remainig third component for bandIdx calc.*/

            signa = 0;
            signb = 0;

            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);
              signa = signa * 4 + signb;
              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colC * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colCT * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;
              }   // modeY
            }     // th
          }       // x
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_0
      break;
      case SAO_TYPE_EO_90:
      {
        startY = isAboveAvail ? 0 : 1;
        endY   = isBelowAvail ? height : height - 1;
        if (!isAboveAvail)
        {
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }
        for (y = startY; y < endY; y++)
        {
          for (x = 0; x < width; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

            const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            const Pel *colCT = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remainig third component for bandIdx calc.*/

            signa = 0;
            signb = 0;
            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);
              signa = signa * 4 + signb;
              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colC * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colCT * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;
              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_90
      break;
      case SAO_TYPE_EO_135:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

          const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          const Pel *colCT = (compID == COMPONENT_Cb)
                               ? srcV + x
                               : srcU + x; /* also use the remainig third component for bandIdx calc.*/

          signa = 0;
          signb = 0;
          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);
            signa = signa * 4 + signb;
            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colC * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colCT * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;
            }   // mode
          }     // th
        }       // x
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height - 1; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

            const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            const Pel *colCT = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remainig third component for bandIdx calc.*/

            signa = 0;
            signb = 0;
            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);
              signa = signa * 4 + signb;
              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colC * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colCT * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;
              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }   // y

      }   // case SAO_TYPE_EO_135
      break;
      case SAO_TYPE_EO_45:
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = isRightAvail ? width : (width - 1);
        // first line
        firstLineStartX = isAboveAvail ? startX : (width - 1);
        firstLineEndX   = isAboveRightAvail ? width : (width - 1);
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

          const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          const Pel *colCT = (compID == COMPONENT_Cb)
                               ? srcV + x
                               : srcU + x; /* also use the remainig third component for bandIdx calc.*/

          signa = 0;
          signb = 0;
          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);
            signa = signa * 4 + signb;
            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colC * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colCT * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;
            }   // mode
          }     // th
        }       // x
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
        // middle lines
        for (y = 1; y < height - 1; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX);
            const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
            const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

            const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            const Pel *colCT = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remainig third component for bandIdx calc.*/

            signa = 0;
            signb = 0;
            for (int th = 0; th < CCSAO_QUAN_NUM; th++)
            {
              signa = calcDiffRangeEnc(*colY, *colA, th);
              signb = calcDiffRangeEnc(*colY, *colB, th);
              signa = signa * 4 + signb;
              for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
              {
                int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
                if (modeY <= 3)
                {
                  band = (*colY * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else if (modeY > 3 && modeY <= 5)
                {
                  band = (*colC * bandNum) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }
                else
                {
                  band = (*colCT * (bandNum - 2)) >> bitDepth;
                  band = band * CCSAO_EDGE_NUM + signa;
                }

                CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

                int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

                blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
                blkStatsEdge[compID][index].count[band]++;
              }   // mode
            }     // th
          }       // x
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }   // y
      }     // case SAO_TYPE_EO_45
      break;

      }   // switch (type)
    }     // for(type =0; type < CCSAO_EDGE_TYPE; type ++)
#endif
    break;
  }   // case COMPONENT_Cb COMPONENT_Cr
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#else
  switch (compID)
  {
  case COMPONENT_Y:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        for (int edgeCmp = COMPONENT_Y; edgeCmp < edgeCmpNum; edgeCmp++)
        {
          const int srcStrideE = srcStrideTab[edgeCmp];
          for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
#else
        for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
#endif
        {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          int edgePosXA = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
          int edgePosXB = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];
     
          const Pel *colY = srcY + x;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;
#else
          int candPosYXA = g_ccSaoEdgeTypeX[type][0];
          int candPosYYA = g_ccSaoEdgeTypeY[type][0];
          int candPosYXB = g_ccSaoEdgeTypeX[type][1];
          int candPosYYB = g_ccSaoEdgeTypeY[type][1];

          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
          {
            const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
            const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
            for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
            {
              const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
              const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
              const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
              const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;

              for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
              {
                const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                const int bandNum = g_ccSaoBandTab[bandIdc][1];
                if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                {
                  continue;
                }

                const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                const int classIdx = bandIdx * edgeNum + edgeIdx;

                int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                            , edgeCmp, edgeIdc
                                            , edgeDir, edgeThr
                                             );

                blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                blkStatsEdge[idx].count[classIdx]++;
              }
            }
          }
#else
          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);

            signa = signa * 4 + signb;

            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colU * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colV * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");
              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;

            }   // mode
          }     // th
#endif
        }       // edge_type
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        }
#endif
      }         // x
      
      srcY += srcStrideY;
      srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
      srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
      org += orgStride;
      dst += dstStride;
    }   // y
  }     // case Y
  break;
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        for (int edgeCmp = COMPONENT_Y; edgeCmp < edgeCmpNum; edgeCmp++)
        {
          const int srcStrideE = srcStrideTab[edgeCmp];
          for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
#else
        for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
#endif
        {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          int edgePosXA = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
          int edgePosXB = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];

          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
          {
            const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
            const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
            for (int edgeThr = 0; edgeThr < MAX_CCSAO_EDGE_THR; edgeThr++)
            {
              const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
              const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
              const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
              const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;

              for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
              {
                const int bandCmp = g_ccSaoBandTab[bandIdc][0];
                const int bandNum = g_ccSaoBandTab[bandIdc][1];
                if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
                {
                  continue;
                }

                const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
                const int classIdx = bandIdx * edgeNum + edgeIdx;

                int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc
                                            , edgeCmp, edgeIdc
                                            , edgeDir, edgeThr
                                             );

                blkStatsEdge[idx].diff [classIdx] += org[x] - dst[x];
                blkStatsEdge[idx].count[classIdx]++;
              }
            }
          }
#else
          int candPosYXA = g_ccSaoEdgeTypeX[type][0];
          int candPosYYA = g_ccSaoEdgeTypeY[type][0];
          int candPosYXB = g_ccSaoEdgeTypeX[type][1];
          int candPosYYB = g_ccSaoEdgeTypeY[type][1];

          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;

          const Pel *colC  = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          const Pel *colCT = (compID == COMPONENT_Cb)
                               ? srcV + x
                               : srcU + x; /* also use the remainig third component for bandIdx calc.*/

          signa = 0;
          signb = 0;
          for (int th = 0; th < CCSAO_QUAN_NUM; th++)
          {
            signa = calcDiffRangeEnc(*colY, *colA, th);
            signb = calcDiffRangeEnc(*colY, *colB, th);
            signa = signa * 4 + signb;
            for (int modeY = 0; modeY < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; modeY++)
            {
              int bandNum = (modeY <= 3) ? modeY + 1 : modeY - 4 + 1;
              if (modeY <= 3)
              {
                band = (*colY * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else if (modeY > 3 && modeY <= 5)
              {
                band = (*colC * bandNum) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }
              else
              {
                band = (*colCT * (bandNum - 2)) >> bitDepth;
                band = band * CCSAO_EDGE_NUM + signa;
              }

              CHECK(band >= MAX_CCSAO_CLASS_NUM, "Band value cannot be greater than total CCSAO Edge class Num");

              int index = calcEdgeStatIndex(ctuRsAddr, modeY, type, th);

              blkStatsEdge[compID][index].diff[band] += org[x] - dst[x];
              blkStatsEdge[compID][index].count[band]++;
            }
          }
#endif
        }
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        }
#endif
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      org += orgStride;
      dst += dstStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#endif
}
#endif
void EncSampleAdaptiveOffset::getCcSaoBlkStats(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth
                                             , const int setIdx, CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM], const int ctuRsAddr
                                             , const uint16_t candPosY
                                             , const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
                                             , const Pel* srcY, const Pel* srcU, const Pel* srcV
                                             , const Pel* org, const Pel* dst
                                             , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int orgStride, const int dstStride
                                             , const int width, const int height
                                             , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                             , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                                              )
{
  const int candPosYX = g_ccSaoCandPosX[COMPONENT_Y][candPosY];
  const int candPosYY = g_ccSaoCandPosY[COMPONENT_Y][candPosY];

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleYM1 = 1 - chromaScaleY;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int x, y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX;

  switch (compID)
  {
  case COMPONENT_Y:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;

        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        org += orgStride;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
            blkStats[setIdx][ctuRsAddr].count[classIdx]++;
          }
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          org += orgStride;
          dst += dstStride;
        }
        break;
      }
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        org += orgStride;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
        blkStats[setIdx][ctuRsAddr].count[classIdx]++;
      }

      srcY += srcStrideY;
      srcU += srcStrideU * chromaScaleYM1;
      srcV += srcStrideV * chromaScaleYM1;
      org += orgStride;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    break;
    }
    break;
  }
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + x;
            const Pel *colV = srcV + x;

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
            blkStats[setIdx][ctuRsAddr].count[classIdx]++;
          }
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;
        }
        break;
      }
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        org += orgStride;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
        blkStats[setIdx][ctuRsAddr].count[classIdx]++;
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      org += orgStride;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          blkStats[setIdx][ctuRsAddr].diff[classIdx] += org[x] - dst[x];
          blkStats[setIdx][ctuRsAddr].count[classIdx]++;
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        org += orgStride;
        dst += dstStride;
      }
      break;
    }
    }
    break;
  }
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#else
  switch (compID)
  {
  case COMPONENT_Y:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        blkStats[setIdx][ctuRsAddr].diff [classIdx] += org[x] - dst[x];
        blkStats[setIdx][ctuRsAddr].count[classIdx]++;
      }

      srcY += srcStrideY;
      srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
      srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
      org += orgStride;
      dst += dstStride;
    }
  }
  break;
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        blkStats[setIdx][ctuRsAddr].diff [classIdx] += org[x] - dst[x];
        blkStats[setIdx][ctuRsAddr].count[classIdx]++;
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      org += orgStride;
      dst += dstStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#endif
}

void EncSampleAdaptiveOffset::getCcSaoFrameStats(const ComponentID compID, const int setIdx, const uint8_t* ccSaoControl
                                               , CcSaoStatData* blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                                               , CcSaoStatData* blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM]
                                               , const uint8_t setType
#endif
)
{
  frameStats    [setIdx].reset();
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  frameStatsEdge[setIdx].reset();
#endif
  int setIdc = setIdx + 1;

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    if (ccSaoControl[ctbIdx] == setIdc)
    {
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      if (setType == CCSAO_SET_TYPE_BAND) { frameStats    [setIdx] += blkStats    [setIdx][ctbIdx]; }
      else         /*CCSAO_SET_TYPE_EDGE*/{ frameStatsEdge[setIdx] += blkStatsEdge[setIdx][ctbIdx]; }
#else
      if (setType == 0) /* band offset */
      {
        frameStats[setIdx] += blkStats[setIdx][ctbIdx];
      }
      else
      {
        frameStatsEdge[setIdx] += blkStatsEdge[setIdx][ctbIdx];
      }
#endif
#else
      frameStats[setIdx] += blkStats[setIdx][ctbIdx];
#endif
    }
  }
}

inline int EncSampleAdaptiveOffset::estCcSaoIterOffset(const double lambda, const int offsetInput, const int64_t count, const int64_t diffSum, const int shift, const int bitIncrease, int64_t& bestDist, double& bestCost, const int offsetTh)
{
  int iterOffset, tempOffset;
  int64_t tempDist, tempRate;
  double tempCost, tempMinCost;
  int offsetOutput = 0;
  iterOffset = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can be used to measure the exact rate here.
  tempMinCost = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = lengthUvlc(abs(iterOffset)) + (iterOffset == 0 ? 0 : 1);

    // Do the dequantization before distortion calculation
    tempOffset = iterOffset << bitIncrease;
    tempDist = estSaoDist(count, tempOffset, diffSum, shift);
    tempCost = ((double)tempDist + lambda * (double)tempRate);
    if (tempCost < tempMinCost)
    {
      tempMinCost = tempCost;
      offsetOutput = iterOffset;
      bestDist = tempDist;
      bestCost = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset - 1) : (iterOffset + 1);
  }
  return offsetOutput;
}

void EncSampleAdaptiveOffset::deriveCcSaoOffsets(const ComponentID compID, const int bitDepth, const int setIdx
                                               , CcSaoStatData frameStats[MAX_CCSAO_SET_NUM]
                                               , short offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM])
{
  int quantOffsets[MAX_CCSAO_CLASS_NUM] = { 0 };

#if JVET_AJ0237_INTERNAL_12BIT
  int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth);
#endif

  for(int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    if(frameStats[setIdx].count[k] == 0)
      continue;

    quantOffsets[k] =
#if JVET_AJ0237_INTERNAL_12BIT
      (int) xRoundIbdi(bitDepth, (double)(frameStats[setIdx].diff [k] << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))
                               / (double)((int64_t)frameStats[setIdx].count[k] << m_offsetStepLog2[compID]));
#else
      (int) xRoundIbdi(bitDepth, (double)(frameStats[setIdx].diff [k] << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))
                               / (double)(frameStats[setIdx].count[k]));
#endif
    quantOffsets[k] = Clip3(-MAX_CCSAO_OFFSET_THR, MAX_CCSAO_OFFSET_THR, quantOffsets[k]);
  }

  int64_t dist[MAX_CCSAO_CLASS_NUM] = { 0 };
  double  cost[MAX_CCSAO_CLASS_NUM] = { 0 };
  for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    cost[k] = m_lambda[compID];
    if (quantOffsets[k] != 0)
    {
#if JVET_AJ0237_INTERNAL_12BIT
      quantOffsets[k] = estCcSaoIterOffset(m_lambda[compID], quantOffsets[k], frameStats[setIdx].count[k], frameStats[setIdx].diff[k], shift, m_offsetStepLog2[compID], dist[k], cost[k], MAX_CCSAO_OFFSET_THR);
#else
      quantOffsets[k] = estCcSaoIterOffset(m_lambda[compID], quantOffsets[k], frameStats[setIdx].count[k], frameStats[setIdx].diff[k], 0, 0, dist[k], cost[k], MAX_CCSAO_OFFSET_THR);
#endif
    }
  }

  for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    CHECK(quantOffsets[k] < -MAX_CCSAO_OFFSET_THR || quantOffsets[k] > MAX_CCSAO_OFFSET_THR, "Exceeded valid range for CCSAO offset");
    offset[setIdx][k] = quantOffsets[k];
  }
}

void EncSampleAdaptiveOffset::getCcSaoDistortion(const ComponentID compID, const int setIdx, CcSaoStatData* blkStats[MAX_CCSAO_SET_NUM]
                                               , short offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM]
#if JVET_AJ0237_INTERNAL_12BIT
                                               , int64_t* trainingDistortion[MAX_CCSAO_SET_NUM], const int shift)
#else
                                               , int64_t* trainingDistortion[MAX_CCSAO_SET_NUM])
#endif
{
  ::memset(trainingDistortion[setIdx], 0, sizeof(int64_t) * m_numCTUsInPic);

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
    {
      trainingDistortion[setIdx][ctbIdx]
#if JVET_AJ0237_INTERNAL_12BIT
        += estSaoDist(blkStats[setIdx][ctbIdx].count[k], offset[setIdx][k], blkStats[setIdx][ctbIdx].diff[k], shift, m_offsetStepLog2[toChannelType(compID)]);
#else
        += estSaoDist(blkStats[setIdx][ctbIdx].count[k], offset[setIdx][k], blkStats[setIdx][ctbIdx].diff[k], 0);
#endif
    }
  }
}
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::getCcSaoDistortionEdge(const ComponentID compID, const int setIdx,
                                                     CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM],
                                                     short          offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM],
                                                     int64_t *      trainingDistortion[MAX_CCSAO_SET_NUM])
{
  ::memset(trainingDistortion[setIdx], 0, sizeof(int64_t) * m_numCTUsInPic);

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
    {
      trainingDistortion[setIdx][ctbIdx] +=
        estSaoDist(blkStatsEdge[setIdx][ctbIdx].count[k], offset[setIdx][k], blkStatsEdge[setIdx][ctbIdx].diff[k], 0);
    }
  }
}
#endif
void EncSampleAdaptiveOffset::determineCcSaoControlIdc(CodingStructure& cs, const ComponentID compID 
                                                     , const int ctuWidthC, const int ctuHeightC, const int picWidthC, const int picHeightC
                                                     , CcSaoEncParam& ccSaoParam, uint8_t* ccSaoControl
                                                     , int64_t* trainingDistorsion[MAX_CCSAO_SET_NUM]
                                                     , int64_t& curTotalDist, double& curTotalRate)
{
  bool setEnabled[MAX_CCSAO_SET_NUM];
  std::fill_n(setEnabled, MAX_CCSAO_SET_NUM, false);

  SetIdxCount setIdxCount[MAX_CCSAO_SET_NUM];
  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    setIdxCount[i].setIdx = i;
    setIdxCount[i].count  = 0;
  }

  double prevRate = curTotalRate;

  TempCtx ctxInitial(m_ctxCache);
  TempCtx ctxBest(m_ctxCache);
  TempCtx ctxStart(m_ctxCache);
  ctxInitial = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());

  int ctbIdx = 0;
  for (int yCtb = 0; yCtb < picHeightC; yCtb += ctuHeightC)
  {
    for (int xCtb = 0; xCtb < picWidthC; xCtb += ctuWidthC)
    {
      int64_t  bestDist   = MAX_INT;
      double   bestRate   = MAX_DOUBLE;
      double   bestCost   = MAX_DOUBLE;
      uint8_t  bestSetIdc = 0;
      uint8_t  bestSetIdx = 0;

      m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc,ctxBest);
      ctxStart                   = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());

      for (int setIdx = 0; setIdx <= MAX_CCSAO_SET_NUM; setIdx++)
      {
        if (setIdx < MAX_CCSAO_SET_NUM && !ccSaoParam.setEnabled[setIdx])
          continue;

        uint8_t setIdc = ccSaoParam.mapIdxToIdc[setIdx];
        m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc,ctxStart);
        m_CABACEstimator->resetBits();
        const Position lumaPos = Position({ xCtb << getComponentScaleX(compID, cs.pcv->chrFormat),
                                            yCtb << getComponentScaleY(compID, cs.pcv->chrFormat) });
        m_CABACEstimator->codeCcSaoControlIdc(setIdc, cs, compID, ctbIdx, ccSaoControl, lumaPos, ccSaoParam.setNum);
        
        int64_t dist = setIdx == MAX_CCSAO_SET_NUM ? 0 : trainingDistorsion[setIdx][ctbIdx];
        double  rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        double  cost = rate * m_lambda[compID] + dist;

        if (cost < bestCost)
        {
          bestCost   = cost;
          bestRate   = rate;
          bestDist   = dist;
          bestSetIdc = setIdc;
          bestSetIdx = setIdx;
          ctxBest = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());
          ccSaoControl[ctbIdx] = setIdx == MAX_CCSAO_SET_NUM ? 0 : setIdx + 1;
        }
      }
      if (bestSetIdc != 0)
      {
        setEnabled [bestSetIdx] = true;
        setIdxCount[bestSetIdx].count++;
      }
      curTotalRate += bestRate;
      curTotalDist += bestDist;
      ctbIdx++;
    }
  }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  if (!ccSaoParam.reusePrv)
  {
#endif
  std::copy_n(setEnabled, MAX_CCSAO_SET_NUM, ccSaoParam.setEnabled);

  std::stable_sort(setIdxCount, setIdxCount + MAX_CCSAO_SET_NUM, compareSetIdxCount);

  int setIdc = 1;
  ccSaoParam.setNum = 0;
  for (SetIdxCount &s : setIdxCount)
  {
    int setIdx = s.setIdx;
    if (ccSaoParam.setEnabled[setIdx])
    {
      ccSaoParam.mapIdxToIdc[setIdx] = setIdc;
      ccSaoParam.setNum++;
      setIdc++;
    }
  }

  curTotalRate = prevRate;
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc,ctxInitial);
  m_CABACEstimator->resetBits();
  ctbIdx = 0;
  for (int yCtb = 0; yCtb < picHeightC; yCtb += ctuHeightC)
  {
    for (int xCtb = 0; xCtb < picWidthC; xCtb += ctuWidthC)
    {
      const int setIdxPlus1 = ccSaoControl[ctbIdx];
      const Position lumaPos = Position({ xCtb << getComponentScaleX(compID, cs.pcv->chrFormat), 
                                          yCtb << getComponentScaleY(compID, cs.pcv->chrFormat) });

      m_CABACEstimator->codeCcSaoControlIdc(setIdxPlus1 == 0 ? 0 : ccSaoParam.mapIdxToIdc[setIdxPlus1 - 1],
                                            cs, compID, ctbIdx, ccSaoControl, lumaPos, ccSaoParam.setNum);
      ctbIdx++;
    }
  }
  curTotalRate += FRAC_BITS_SCALE*m_CABACEstimator->getEstFracBits();
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  }
#endif

  // restore for next iteration
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc,ctxInitial);
}

int EncSampleAdaptiveOffset::lengthUvlc(int uiCode)
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}

int EncSampleAdaptiveOffset::getCcSaoParamRate(const ComponentID compID, const CcSaoEncParam& ccSaoParam)
{
  int bits = 0;

  if (ccSaoParam.setNum > 0)
  {
    bits += lengthUvlc(ccSaoParam.setNum - 1);

    int signaledSetNum = 0;
    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      if (ccSaoParam.setEnabled[setIdx])
      {
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        bits += 1;
        if (ccSaoParam.setType[setIdx] == CCSAO_SET_TYPE_EDGE)
        {
          bits += m_extChroma ? MAX_CCSAO_EDGE_CMP_BITS : 0;
          bits += MAX_CCSAO_EDGE_IDC_BITS;
          bits += MAX_CCSAO_EDGE_DIR_BITS;
          bits += MAX_CCSAO_EDGE_THR_BITS;
          bits += MAX_CCSAO_BAND_IDC_BITS;
        }
        else
        {
          bits += MAX_CCSAO_CAND_POS_Y_BITS;
          bits += MAX_CCSAO_BAND_NUM_Y_BITS;
          bits += MAX_CCSAO_BAND_NUM_U_BITS;
          bits += MAX_CCSAO_BAND_NUM_V_BITS;
        }
#else
        {
          bits += 1; /* Edge or Band classifier */
          if (ccSaoParam.setType[setIdx] == 1)
          {
            bits += MAX_CCSAO_CAND_POS_Y_BITS - 2;       // MAX_CCSAO_CAND_POS_Y_BITS
            bits += MAX_CCSAO_BAND_NUM_Y_BITS - 2 + 1;   // MAX_CCSAO_BAND_NUM_Y_BITS
            bits += MAX_CCSAO_BAND_NUM_U_BAND_BITS;      // THRESHOLD_BITS
          }
          else
          {
            bits += MAX_CCSAO_CAND_POS_Y_BITS;
            bits += MAX_CCSAO_BAND_NUM_Y_BITS;
            bits += MAX_CCSAO_BAND_NUM_U_BITS;
            bits += MAX_CCSAO_BAND_NUM_V_BITS;
          }
        }
#endif
#else
        bits += MAX_CCSAO_CAND_POS_Y_BITS;
        bits += MAX_CCSAO_BAND_NUM_Y_BITS;
        bits += MAX_CCSAO_BAND_NUM_U_BITS;
        bits += MAX_CCSAO_BAND_NUM_V_BITS;
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        int classNum = getCcSaoClassNumEnc(setIdx, ccSaoParam);
#else
        int classNum = ccSaoParam.bandNum[setIdx][COMPONENT_Y ]
                     * ccSaoParam.bandNum[setIdx][COMPONENT_Cb]
                     * ccSaoParam.bandNum[setIdx][COMPONENT_Cr];
#endif

#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER && !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        if (ccSaoParam.setType[setIdx] == 1)
        {
          if (ccSaoParam.bandNum[setIdx][COMPONENT_Y] < CCSAO_EDGE_COMPARE_VALUE + CCSAO_EDGE_COMPARE_VALUE)
          {
            classNum = (ccSaoParam.bandNum[setIdx][COMPONENT_Y] + 1) * CCSAO_EDGE_NUM;
          }
          else if (ccSaoParam.bandNum[setIdx][COMPONENT_Y]
                   < CCSAO_EDGE_COMPARE_VALUE + CCSAO_EDGE_COMPARE_VALUE + CCSAO_EDGE_COMPARE_VALUE)
          {
            classNum = (ccSaoParam.bandNum[setIdx][COMPONENT_Y] - 4 + 1) * CCSAO_EDGE_NUM;
          }
          else if (ccSaoParam.bandNum[setIdx][COMPONENT_Y] < CCSAO_EDGE_COMPARE_VALUE + CCSAO_EDGE_COMPARE_VALUE
                                                               + CCSAO_EDGE_COMPARE_VALUE + CCSAO_EDGE_COMPARE_VALUE)
          {
            classNum = (ccSaoParam.bandNum[setIdx][COMPONENT_Y] - 6 + 1) * CCSAO_EDGE_NUM;
          }
        }
#endif

        for (int i = 0; i < classNum; i++)
        {
          bits += lengthUvlc(abs(ccSaoParam.offset[setIdx][i])) + (ccSaoParam.offset[setIdx][i] == 0 ? 0 : 1);
        }
        signaledSetNum++;
      }
    }
    CHECK(signaledSetNum != ccSaoParam.setNum, "Number of sets signaled not the same as indicated");
  }
  return bits;
}

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
int EncSampleAdaptiveOffset::getCcSaoClassNumEnc(const int setIdx, const CcSaoEncParam& ccSaoParam)
{
  int classNum = 0;

  if (ccSaoParam.setType[setIdx] == CCSAO_SET_TYPE_EDGE)
  {
    int bandIdc = ccSaoParam.bandNum[setIdx][COMPONENT_Y ], bandNum = g_ccSaoBandTab[bandIdc][1];
    int edgeIdc = ccSaoParam.bandNum[setIdx][COMPONENT_Cr], edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
    classNum = bandNum * edgeNum;
  }
  else
  {
    classNum = ccSaoParam.bandNum[setIdx][COMPONENT_Y ]
             * ccSaoParam.bandNum[setIdx][COMPONENT_Cb]
             * ccSaoParam.bandNum[setIdx][COMPONENT_Cr];
  }
  
  return classNum;
}
#endif

void EncSampleAdaptiveOffset::deriveCcSaoRDO(CodingStructure& cs, const ComponentID compID, int64_t* trainingDistortion[MAX_CCSAO_SET_NUM]
                                           , CcSaoStatData* blkStats    [MAX_CCSAO_SET_NUM], CcSaoStatData frameStats    [MAX_CCSAO_SET_NUM]
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                                           , CcSaoStatData *blkStatsEdge[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM]
#endif
                                           , CcSaoEncParam& bestCcSaoParam, CcSaoEncParam& tempCcSaoParam
                                           , uint8_t* bestCcSaoControl, uint8_t* tempCcSaoControl
                                           , double& bestCost, double& tempCost)
{
  const int scaleX          = getComponentScaleX(compID, cs.pcv->chrFormat);
  const int scaleY          = getComponentScaleY(compID, cs.pcv->chrFormat);
  const int ctuWidthC       = cs.pcv->maxCUWidth  >> scaleX;
  const int ctuHeightC      = cs.pcv->maxCUHeight >> scaleY;
  const int picWidthC       = cs.pcv->lumaWidth   >> scaleX;
  const int picHeightC      = cs.pcv->lumaHeight  >> scaleY;
  const int maxTrainingIter = 15;

  const TempCtx ctxStartCcSaoControlFlag  ( m_ctxCache, SubCtx( Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx() ) );

#if JVET_AJ0237_INTERNAL_12BIT
  const int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->getBitDepth(toChannelType(compID)));
#endif

  int    trainingIter = 0;
  bool   keepTraining = true;
  bool   improved = false;
  double prevCost = MAX_DOUBLE;
  while (keepTraining)
  {
    improved = false;

    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      if (tempCcSaoParam.setEnabled[setIdx])
      {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        if (tempCcSaoParam.reusePrv)
        {
          getCcSaoDistortion(compID, setIdx
                           , tempCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND ? blkStats : blkStatsEdge
#if JVET_AJ0237_INTERNAL_12BIT
                           , tempCcSaoParam.offset, trainingDistortion, shift);
#else
                           , tempCcSaoParam.offset, trainingDistortion);
#endif
        }
        else
        {
#endif
        getCcSaoFrameStats(compID, setIdx, tempCcSaoControl, blkStats, frameStats
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
                         , blkStatsEdge, frameStatsEdge, tempCcSaoParam.setType[setIdx]
#endif
        );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        if (tempCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND)
#else
        if (tempCcSaoParam.setType[setIdx]== 0)
#endif
        {
          deriveCcSaoOffsets(compID, cs.sps->getBitDepth(toChannelType(compID)), setIdx, frameStats, tempCcSaoParam.offset);
#if JVET_AJ0237_INTERNAL_12BIT
          getCcSaoDistortion(compID, setIdx, blkStats, tempCcSaoParam.offset, trainingDistortion, shift);
#else
          getCcSaoDistortion(compID, setIdx, blkStats, tempCcSaoParam.offset, trainingDistortion);
#endif
        }
        else
        {
          deriveCcSaoOffsets(compID, cs.sps->getBitDepth(toChannelType(compID)), setIdx, frameStatsEdge, tempCcSaoParam.offset);
#if JVET_AJ0237_INTERNAL_12BIT
          getCcSaoDistortion(compID, setIdx, blkStatsEdge, tempCcSaoParam.offset, trainingDistortion, shift);
#else
          getCcSaoDistortion(compID, setIdx, blkStatsEdge, tempCcSaoParam.offset, trainingDistortion);
#endif
        }
#else
        deriveCcSaoOffsets(compID, cs.sps->getBitDepth(toChannelType(compID)), setIdx, frameStats,
                           tempCcSaoParam.offset);
        getCcSaoDistortion(compID, setIdx, blkStats, tempCcSaoParam.offset, trainingDistortion);
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        }
#endif
      }
    }

    m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc,ctxStartCcSaoControlFlag);

    int64_t curTotalDist = 0;
    double  curTotalRate = 0;
#if JVET_AL0142_CCSAO_REUSE_CTU
    if (tempCcSaoParam.reusePrv == CCSAO_REUSE_PARAM_CTU)
    {
      for (int ctbIdx = 0; ctbIdx < cs.pcv->sizeInCtus; ctbIdx++)
      {
        if (tempCcSaoControl[ctbIdx])
        {
          int setIdx = tempCcSaoControl[ctbIdx] - 1;
          curTotalDist += trainingDistortion[setIdx][ctbIdx];
        }
      }
    }
    else
    {
#endif
      determineCcSaoControlIdc(cs, compID, ctuWidthC, ctuHeightC, picWidthC, picHeightC,
                               tempCcSaoParam, tempCcSaoControl, trainingDistortion,
                               curTotalDist, curTotalRate);
#if JVET_AL0142_CCSAO_REUSE_CTU
    }
#endif

    if (tempCcSaoParam.setNum > 0)
    {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      curTotalRate += !cs.slice->isIntra() ? 1 : 0;  // +1 reusePrv flag 
#if JVET_AL0142_CCSAO_REUSE_CTU
      curTotalRate += tempCcSaoParam.reusePrv ? 1 : 0;  // +1 reuseMode flag 
#endif
      curTotalRate += tempCcSaoParam.reusePrv 
                    ? MAX_CCSAO_PRV_NUM_BITS 
                    : getCcSaoParamRate(compID, tempCcSaoParam);
#else
      curTotalRate += getCcSaoParamRate(compID, tempCcSaoParam);
#endif
      tempCost = curTotalRate * m_lambda[compID] + curTotalDist;

      if (tempCost < prevCost)
      {
        prevCost = tempCost;
        improved = true;
      }

      if (tempCost < bestCost)
      {
        bestCost = tempCost;
        bestCcSaoParam = tempCcSaoParam;
        memcpy(bestCcSaoControl, tempCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);
      }
    }

    trainingIter++;
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    if (!improved || trainingIter > maxTrainingIter || tempCcSaoParam.reusePrv)
#else
    if (!improved || trainingIter > maxTrainingIter)
#endif
    {
      keepTraining = false;
    }
  }
}

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void EncSampleAdaptiveOffset::setupCcSaoPrv(CodingStructure &cs)
{
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
#if JVET_AL0142_CCSAO_REUSE_CTU
    if (m_ccSaoComParam.enabled[compIdx] && m_ccSaoComParam.reusePrv[compIdx] != CCSAO_REUSE_PARAM_CTU)
#else
    if (m_ccSaoComParam.enabled[compIdx] && !m_ccSaoComParam.reusePrv[compIdx])
#endif
    {
      if (g_ccSaoPrvParam[compIdx].size() == MAX_CCSAO_PRV_NUM)
      {
        g_ccSaoPrvParam[compIdx].pop_back();
      }

      CcSaoPrvParam prvParam;
      prvParam.temporalId = cs.slice->getTLayer();
      prvParam.setNum     = m_ccSaoComParam.setNum[compIdx];
      std::memcpy( prvParam.setEnabled, m_ccSaoComParam.setEnabled[compIdx], sizeof( prvParam.setEnabled ) );
      std::memcpy( prvParam.setType,    m_ccSaoComParam.setType   [compIdx], sizeof( prvParam.setType    ) );
      std::memcpy( prvParam.candPos,    m_ccSaoComParam.candPos   [compIdx], sizeof( prvParam.candPos    ) );
      std::memcpy( prvParam.bandNum,    m_ccSaoComParam.bandNum   [compIdx], sizeof( prvParam.bandNum    ) );
      std::memcpy( prvParam.offset,     m_ccSaoComParam.offset    [compIdx], sizeof( prvParam.offset     ) );

      g_ccSaoPrvParam[compIdx].insert(g_ccSaoPrvParam[compIdx].begin(), prvParam);
    }
  }
}
#endif
#endif

void EncSampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos, bool& isLeftAvail, bool& isAboveAvail, bool& isAboveLeftAvail) const
{
  bool isLoopFiltAcrossSlicePPS = cs.pps->getLoopFilterAcrossSlicesEnabledFlag();
  bool isLoopFiltAcrossTilePPS = cs.pps->getLoopFilterAcrossTilesEnabledFlag();

  const int width = cs.pcv->maxCUWidth;
  const int height = cs.pcv->maxCUHeight;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-width, 0), CH_L);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -height), CH_L);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-width, -height), CH_L);

  if (!isLoopFiltAcrossSlicePPS)
  {
    isLeftAvail      = (cuLeft == NULL)      ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (cuAbove == NULL)     ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (cuAboveLeft == NULL) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }
  else
  {
    isLeftAvail      = (cuLeft != NULL);
    isAboveAvail     = (cuAbove != NULL);
    isAboveLeftAvail = (cuAboveLeft != NULL);
  }

  if (!isLoopFiltAcrossTilePPS)
  {
    isLeftAvail      = (!isLeftAvail)      ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail)     ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }

  const SubPic& curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
  if (!curSubPic.getloopFilterAcrossEnabledFlag())
  {
    isLeftAvail      = (!isLeftAvail)      ? false : CU::isSameSubPic(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail)     ? false : CU::isSameSubPic(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveLeft);
  }
}

//! \}
