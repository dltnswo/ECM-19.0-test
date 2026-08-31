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

/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __ENCCU__
#define __ENCCU__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/IbcHashMap.h"
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif
#include "CommonLib/LoopFilter.h"

#include "DecoderLib/DecCu.h"

#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "RateCtrl.h"
#include "EncModeCtrl.h"
//! \ingroup EncoderLib
//! \{

class EncLib;
class HLSWriter;
class EncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
struct GeoMergeCombo
{
  int splitDir;
  int mergeIdx0;
  int mergeIdx1;
  double cost;
  GeoMergeCombo() : splitDir(), mergeIdx0(-1), mergeIdx1(-1), cost(0.0) {};
  GeoMergeCombo(int _splitDir, int _mergeIdx0, int _mergeIdx1, double _cost) : splitDir(_splitDir), mergeIdx0(_mergeIdx0), mergeIdx1(_mergeIdx1), cost(_cost) {};
};
struct GeoMotionInfo
{
  uint8_t   m_candIdx0;
  uint8_t   m_candIdx1;

  GeoMotionInfo(uint8_t candIdx0, uint8_t candIdx1) : m_candIdx0(candIdx0), m_candIdx1(candIdx1) { }
  GeoMotionInfo() { m_candIdx0 = m_candIdx1 = 0; }
};
struct SmallerThanComboCost
{
  inline bool operator() (const GeoMergeCombo& first, const GeoMergeCombo& second)
  {
#if JVET_Z0118_GDR
    bool ret = true;
    
    ret = (first.cost < second.cost);
    
    if (first.cost == second.cost)
    {
      ret = first.splitDir < second.splitDir;
      if (first.splitDir == second.splitDir)
      {
        ret = first.mergeIdx0 < second.mergeIdx0;
        if (first.mergeIdx0 == second.mergeIdx0)
        {
          ret = first.mergeIdx1 < second.mergeIdx1;
        }
      }
    }

    return ret;
#else
      return (first.cost < second.cost);
#endif
  }
};
class GeoComboCostList
{
public:
  GeoComboCostList() {};
  ~GeoComboCostList() {};
  std::vector<GeoMergeCombo> list;
  void sortByCost() { std::stable_sort(list.begin(), list.end(), SmallerThanComboCost()); };
};
struct SingleGeoMergeEntry
{
  int mergeIdx;
  double cost;
  SingleGeoMergeEntry() : mergeIdx(0), cost(MAX_DOUBLE) {};
  SingleGeoMergeEntry(int _mergeIdx, double _cost) : mergeIdx(_mergeIdx), cost(_cost) {};
};
class FastGeoCostList
{
public:
  FastGeoCostList() { numGeoTemplatesInitialized = 0; };
  ~FastGeoCostList()
  {
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        delete[] singleDistList[partIdx][splitDir];
      }
      delete[] singleDistList[partIdx];
      singleDistList[partIdx] = nullptr;
    }
  };
  SingleGeoMergeEntry** singleDistList[2];
  void init(int numTemplates, int maxNumGeoCand)
  {
    if (numGeoTemplatesInitialized == 0 || numGeoTemplatesInitialized < numTemplates)
    {
      for (int partIdx = 0; partIdx < 2; partIdx++)
      {
        singleDistList[partIdx] = new SingleGeoMergeEntry*[numTemplates];
        for (int splitDir = 0; splitDir < numTemplates; splitDir++)
        {
          singleDistList[partIdx][splitDir] = new SingleGeoMergeEntry[maxNumGeoCand];
        }
      }
      numGeoTemplatesInitialized = numTemplates;
    }
  }
  void insert(int geoIdx, int partIdx, int mergeIdx, double cost)
  {
    CHECKD(geoIdx >= numGeoTemplatesInitialized, "Wrong geoIdx");
    singleDistList[partIdx][geoIdx][mergeIdx] = SingleGeoMergeEntry(mergeIdx, cost);
  }
  int numGeoTemplatesInitialized;
};
#if JVET_W0097_GPM_MMVD_TM
struct SingleGeoMMVDMergeEntry
{
  int mergeIdx;
  int mmvdIdx;   // 0 - mmvd OFF; 1 - mmvdIdx = 0; 2 - mmvdIdx = 1; 3 - mmvdIdx = 2; ...
  double cost;
  SingleGeoMMVDMergeEntry() : mergeIdx(0), mmvdIdx(0), cost(MAX_DOUBLE) {};
  SingleGeoMMVDMergeEntry(int _mergeIdx, int _mmvdIdx, double _cost) : mergeIdx(_mergeIdx), mmvdIdx(_mmvdIdx), cost(_cost) {};
};

class FastGeoMMVDCostList
{
public:
  FastGeoMMVDCostList()
  {
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      singleDistList[partIdx] = new SingleGeoMMVDMergeEntry**[GEO_NUM_PARTITION_MODE];
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
#if JVET_AG0164_AFFINE_GPM
#if JVET_AI0082_GPM_WITH_INTER_IBC
        singleDistList[partIdx][splitDir] = new SingleGeoMMVDMergeEntry*[GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS];
        for (int candIdx = 0; candIdx < GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS; candIdx++)
#else
        singleDistList[partIdx][splitDir] = new SingleGeoMMVDMergeEntry*[GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS];
        for (int candIdx = 0; candIdx < GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS; candIdx++)
#endif
        {
          singleDistList[partIdx][splitDir][candIdx] = new SingleGeoMMVDMergeEntry[GPM_EXT_MMVD_MAX_REFINE_NUM + 2];
        }
#else
#if JVET_Y0065_GPM_INTRA
#if JVET_AI0082_GPM_WITH_INTER_IBC
        singleDistList[partIdx][splitDir] = new SingleGeoMMVDMergeEntry*[GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS];
        for (int candIdx = 0; candIdx < GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS; candIdx++)
#else
        singleDistList[partIdx][splitDir] = new SingleGeoMMVDMergeEntry*[GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS];
        for (int candIdx = 0; candIdx < GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS; candIdx++)
#endif
#else
        singleDistList[partIdx][splitDir] = new SingleGeoMMVDMergeEntry*[MRG_MAX_NUM_CANDS];
        for (int candIdx = 0; candIdx < MRG_MAX_NUM_CANDS; candIdx++)
#endif
        {
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
          singleDistList[partIdx][splitDir][candIdx] = new SingleGeoMMVDMergeEntry[GPM_EXT_MMVD_MAX_REFINE_NUM + 2];
#else
          singleDistList[partIdx][splitDir][candIdx] = new SingleGeoMMVDMergeEntry[GPM_EXT_MMVD_MAX_REFINE_NUM + 1];
#endif
        }
#endif
      }
    }
  }
  ~FastGeoMMVDCostList()
  {
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
#if JVET_AG0164_AFFINE_GPM
#if JVET_AI0082_GPM_WITH_INTER_IBC
        for (int candIdx = 0; candIdx < GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS; candIdx++)
#else
        for (int candIdx = 0; candIdx < GEO_MAX_ALL_INTER_UNI_CANDS +GEO_MAX_NUM_INTRA_CANDS; candIdx++)
#endif
        {
          delete[] singleDistList[partIdx][splitDir][candIdx];
        }
#else
#if JVET_Y0065_GPM_INTRA
#if JVET_AI0082_GPM_WITH_INTER_IBC
        for (int candIdx = 0; candIdx < GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS+GEO_MAX_NUM_IBC_CANDS; candIdx++)
#else
        for (int candIdx = 0; candIdx < GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS; candIdx++)
#endif
#else
        for (int candIdx = 0; candIdx < MRG_MAX_NUM_CANDS; candIdx++)
#endif
        {
          delete[] singleDistList[partIdx][splitDir][candIdx];
        }
#endif
        delete[] singleDistList[partIdx][splitDir];
      }
      delete[] singleDistList[partIdx];
      singleDistList[partIdx] = nullptr;
    }
  }
  SingleGeoMMVDMergeEntry*** singleDistList[2];
  void insert(int geoIdx, int partIdx, int mergeIdx, int mmvdIdx, double cost)
  {
    singleDistList[partIdx][geoIdx][mergeIdx][mmvdIdx] = SingleGeoMMVDMergeEntry(mergeIdx, mmvdIdx, cost);
  }
};
#endif
class EncCu
  : DecCu
{
private:
  bool m_bestModeUpdated;
  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair>  m_ctxBuffer;
  CtxPair*              m_CurrCtx;
  CtxCache*             m_ctxCache;

#if ENABLE_SPLIT_PARALLELISM
  int                   m_dataId;
#endif

  //  Data : encoder control
  int                   m_cuChromaQpOffsetIdxPlus1; // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag will be 1.

  XUCache               m_unitCache;

  CodingStructure    ***m_pTempCS;
  CodingStructure    ***m_pBestCS;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CodingStructure    ***m_pTempSSTCS;
  CodingStructure    ***m_pBestSSTCS;
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CodingStructure    ***m_pTempCS2;
  CodingStructure    ***m_pBestCS2;
#endif
#if MULTI_HYP_PRED
  MEResultVec           m_baseResultsForMH;
#endif
#if ENABLE_OBMC
  CodingStructure    ***m_pTempCUWoOBMC; ///< Temporary CUs in each depth
  PelStorage          **m_pPredBufWoOBMC;
  PelStorage            m_tempWoOBMCBuffer;
#endif
  //  Access channel
  EncCfg*               m_pcEncCfg;
  IntraSearch*          m_pcIntraSearch;
  InterSearch*          m_pcInterSearch;
  TrQuant*              m_pcTrQuant;
  RdCost*               m_pcRdCost;
  EncSlice*             m_pcSliceEncoder;
  LoopFilter*           m_pcLoopFilter;

  CABACWriter*          m_CABACEstimator;
  RateCtrl*             m_pcRateCtrl;
  IbcHashMap            m_ibcHashMap;
  EncModeCtrl          *m_modeCtrl;

#if JVET_Y0065_GPM_INTRA
  PelStorage            m_acMergeBuffer[GEO_NUM_RDO_BUFFER];
#else
  PelStorage            m_acMergeBuffer[MMVD_MRG_MAX_RD_BUF_NUM];
#endif
#if INTER_LIC || MULTI_HYP_PRED
#if JVET_AD0213_LIC_IMP
  PelStorage            m_acRealMergeBuffer[MRG_MAX_NUM_CANDS * 3];
#else
  PelStorage            m_acRealMergeBuffer[MRG_MAX_NUM_CANDS * 2];
#endif
#else
  PelStorage            m_acRealMergeBuffer[MRG_MAX_NUM_CANDS];
#endif
#if JVET_AG0135_AFFINE_CIIP
  PelStorage            m_acMergeAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS];
#endif
  PelStorage            m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_LIC
                                           + 1
#endif
                                          ];
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  PelStorage            m_acTmMergeTmpBuffer[MRG_MAX_NUM_CANDS];
#endif
  PelStorage            m_ciipBuffer[2];

#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  PelStorage            m_acGeoWeightedBuffer[(GEO_MAX_TRY_WEIGHTED_SAD + (SGPM_INTER_NUM + SGPM_INTER_INTRA_NUM) + SGPM_AFFINE_NUM) * GEO_BLENDING_NUM + 1 + 1];
#elif JVET_AL0134_SGPM_INTER
  PelStorage            m_acGeoWeightedBuffer[(GEO_MAX_TRY_WEIGHTED_SAD + SGPM_INTER_NUM) * GEO_BLENDING_NUM + 1 + 1];   // to store weighted prediction pixels
#else
  PelStorage            m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD*GEO_BLENDING_NUM+1]; // to store weighted prediction pixels
#endif
#else
  PelStorage            m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD+1]; // to store weighted prediction pixles
#endif
#else
  PelStorage            m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD]; // to store weighted prediction pixels
#endif
#if !JVET_AG0164_AFFINE_GPM
  FastGeoCostList       m_GeoCostList;
#endif
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AG0164_AFFINE_GPM
  PelStorage            m_acGeoMMVDBuffer[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  PelStorage            m_acGeoMMVDTmpBuffer[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#else
  PelStorage            m_acGeoMMVDBuffer[MRG_MAX_NUM_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  PelStorage            m_acGeoMMVDTmpBuffer[MRG_MAX_NUM_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#endif
  FastGeoMMVDCostList   m_geoMMVDCostList;
  bool                  m_fastGpmMmvdSearch;
#if JVET_AJ0274_GPM_AFFINE_TM
  int                  m_fastGpmAffSearch;
#endif
  bool                  m_fastGpmMmvdRelatedCU;
  bool                  m_includeMoreMMVDCandFirstPass;
  int                   m_maxNumGPMDirFirstPass;
  int                   m_numCandPerPar;
#if TM_MRG
  PelStorage            m_acGeoMergeTmpBuffer[GEO_TM_MAX_NUM_CANDS];
  PelStorage            m_acGeoSADTmpBuffer[GEO_TM_MAX_NUM_CANDS];
#endif
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  PelStorage            m_acSgpmAffineBuffer[GEO_MAX_NUM_UNI_AFF_CANDS_ARMC];
  PelStorage            m_acSgpmIntraBuffer[SGPM_INTRA_NUM];
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
  PelStorage            m_acGeoBlendTMBuffer[GEO_TM_MAX_NUM_CANDS];
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  PelStorage            m_acRegGpmWeightedBuffer[GEO_MAX_NUM_UNI_CANDS];
#if JVET_AJ0274_REGRESSION_GPM_TM
  PelStorage            m_acRegGpmTmWeightedBuffer[GEO_MAX_NUM_UNI_CANDS];
#endif
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  PelStorage            m_acMergeIntraBuffer[GEO_BLEND_MAX_NUM_INTRA_CANDS];
  PelStorage            m_acMergeOBMCBuffer[GEO_MAX_NUM_UNI_CANDS];
#endif
  double                m_AFFBestSATDCost;
  double                m_mergeBestSATDCost;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if JVET_AG0098_AMVP_WITH_SBTMVP
  MotionInfo            m_subPuMiBuf[SUB_BUFFER_SIZE][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#else
  MotionInfo            m_subPuMiBuf[SUB_TMVP_NUM][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#else
  MotionInfo            m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#if JVET_AL0160_SBSMVP
  MotionInfo            m_subSpatialPuMiBuf[NUM_SUB_SMVP][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#if MULTI_PASS_DMVR
  Mv                    m_mvBufBDMVR[(MRG_MAX_NUM_CANDS << 1)][MAX_NUM_SUBCU_DMVR];
#if TM_MRG
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  Mv                    m_mvBufBDMVR4TM[(TM_MRG_MAX_NUM_INIT_CANDS << 1)][MAX_NUM_SUBCU_DMVR];
#else
  Mv                    m_mvBufBDMVR4TM[(TM_MRG_MAX_NUM_CANDS << 1)][MAX_NUM_SUBCU_DMVR];
#endif
#endif
#if (JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION) || JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  Mv                    m_mvBufBDMVR4AFFINE[(AFFINE_MRG_MAX_NUM_CANDS << 1)][MAX_NUM_SUBCU_DMVR];
#endif
  Mv                    m_mvBufEncBDOF[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
  Mv                    m_mvBufEncBDOF4TM[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
#if JVET_AJ0274_REGRESSION_GPM_TM
  Mv                    m_mvBufEncBDOF4TMBlend[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
#endif
#if JVET_X0049_ADAPT_DMVR
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  Mv                    m_mvBufBDMVR4BM[(BM_MRG_MAX_NUM_INIT_CANDS << 1)<<1][MAX_NUM_SUBCU_DMVR];
  Mv                    m_mvBufEncBDOF4BM[BM_MRG_MAX_NUM_INIT_CANDS<<1][BDOF_SUBPU_MAX_NUM];
#else
  Mv                    m_mvBufBDMVR4BM[(BM_MRG_MAX_NUM_CANDS << 1)<<1][MAX_NUM_SUBCU_DMVR];
  Mv                    m_mvBufEncBDOF4BM[BM_MRG_MAX_NUM_CANDS<<1][BDOF_SUBPU_MAX_NUM];
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  Mv                    m_mvBufBDMVR4OPPOSITELIC[MRG_MAX_NUM_CANDS << 1][MAX_NUM_SUBCU_DMVR];
  Mv                    m_mvBufEncBDOF4OPPOSITELIC[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
  Mv                    m_mvBufBDMVR4TMOPPOSITELIC[(TM_MRG_MAX_NUM_INIT_CANDS << 1)][MAX_NUM_SUBCU_DMVR];
  Mv                    m_mvBufEncBDOF4TMOPPOSITELIC[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
#endif
#if JVET_AE0046_BI_GPM
  Mv*                   m_mvBufBDMVR4GPM[2];
  Mv*                   m_mvBufEncBDOF4GPM[GEO_NUM_TM_MV_CAND][MRG_MAX_NUM_CANDS];
#if !JVET_AA0093_REFINED_MOTION_FOR_ARMC
  Mv                    m_mvExtraBufEncBDOF4GPM[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
#endif
#endif
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  Mv                    m_mvBufEncAffineBDOF[AFFINE_MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
  Mv                    m_mvBufEncAffineBmBDOF[AFFINE_ADAPTIVE_DMVR_INIT_SIZE << 1][BDOF_SUBPU_MAX_NUM];
  bool                  m_doEncAffineBDOF[AFFINE_MRG_MAX_NUM_CANDS];
  bool                  m_doEncAffineBmBDOF[AFFINE_ADAPTIVE_DMVR_INIT_SIZE << 1];
  Mv                    m_mvBufEncMhpAffineBDOF[BDOF_SUBPU_MAX_NUM];
#if JVET_AG0276_LIC_FLAG_SIGNALING
  Mv                    m_mvBufEncAffineBDOFOppositeLic[AFFINE_MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM];
  bool                  m_doEncAffineBDOFOppositeLic[AFFINE_MRG_MAX_NUM_CANDS];
  Mv                    m_mvBufEncMhpAffineBDOFOppositeLic[BDOF_SUBPU_MAX_NUM];
#endif
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE || JVET_AE0046_BI_GPM
  Mv                    m_mvBufEncAmBDMVR[2][MAX_NUM_SUBCU_DMVR];
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  MvField               m_mvFieldAmListEnc[MAX_NUM_AMVP_CANDS_MAX_REF << 1];
#if JVET_AD0213_LIC_IMP
  bool                  m_licAmListEnc[MAX_NUM_AMVP_CANDS_MAX_REF << 1];
#endif
#endif

  int                   m_ctuIbcSearchRangeX;
  int                   m_ctuIbcSearchRangeY;
#if ENABLE_SPLIT_PARALLELISM
  EncLib*               m_pcEncLib;
#endif
  int                   m_bestBcwIdx[2];
  double                m_bestBcwCost[2];
  GeoMotionInfo         m_GeoModeTest[GEO_MAX_NUM_CANDS];
#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
  void    updateLambda      ( Slice* slice, const int dQP,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                              const bool useWCGChromaControl,
 #endif
                              const bool updateRdCostLambda );
#endif
  double                m_sbtCostSave[2];
#if JVET_AA0133_INTER_MTS_OPT
  double                m_mtsCostSave;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  double                m_LNCostSave;
#endif
#if JVET_W0097_GPM_MMVD_TM
  MergeCtx              m_mergeCand;
  bool                  m_mergeCandAvail;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool                  m_skipIbcMerge;
#endif
public:
  /// copy parameters from encoder class
  void  init                ( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int jId = 0 ) );

  void setDecCuReshaperInEncCU(EncReshape* pcReshape, ChromaFormat chromaFormatIDC) { initDecCuReshaper((Reshape*) pcReshape, chromaFormatIDC); }
  /// create internal buffers
  void  create              ( EncCfg* encCfg );

  /// destroy internal buffers
  void  destroy             ();

  /// CTU analysis function
  void  compressCtu         ( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] );
  /// CTU encoding function
  int   updateCtuDataISlice ( const CPelBuf buf );
  /// function to check whether the current CU is luma and non-boundary CU
#if JVET_AI0087_BTCUS_RESTRICTION
  bool isLumaNonBoundaryCu(const Partitioner &partitioner, SizeType picWidth, SizeType picHeight);
  bool xStoreRDcostandPredMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode &encTestmode, double tempcost);
#endif
  EncModeCtrl* getModeCtrl  () { return m_modeCtrl; }

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  BilateralFilter *m_bilateralFilter;
#endif

  void   setMergeBestSATDCost(double cost) { m_mergeBestSATDCost = cost; }
  double getMergeBestSATDCost()            { return m_mergeBestSATDCost; }
  void   setAFFBestSATDCost(double cost)   { m_AFFBestSATDCost = cost; }
  double getAFFBestSATDCost()              { return m_AFFBestSATDCost; }
  IbcHashMap& getIbcHashMap()              { return m_ibcHashMap;        }
  EncCfg*     getEncCfg()            const { return m_pcEncCfg;          }

  EncCu();
  ~EncCu();

protected:

  void xCalDebCost            ( CodingStructure &cs, Partitioner &partitioner, bool calDist = false );
  Distortion getDistortionDb  ( CodingStructure &cs, CPelBuf org, CPelBuf reco, ComponentID compID, const CompArea& compArea, bool afterDb );
#if JVET_AJ0226_MTT_SKIP
  void xStoreMttSplitFlagCabacBits(CodingStructure*& tempCS, Partitioner& partitioner, int mttSplitFlagCabacBits);
#endif
  void xCompressCU            ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, double maxCostAllowed = MAX_DOUBLE 
#if JVET_AJ0226_MTT_SKIP 
    , int mttSplitFlagCabacBits = 0
#endif
  );
#if ENABLE_SPLIT_PARALLELISM
  void xCompressCUParallel    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm );
  void copyState              ( EncCu* other, Partitioner& pm, const UnitArea& currArea, const bool isDist );
#endif
  bool
    xCheckBestMode         ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestmode );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
#if JVET_Y0152_TT_ENC_SPEEDUP
  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass, double *splitRdCostBest 
#if JVET_AJ0226_MTT_SKIP
    , int mttSplitFlagCabacBits
#endif
  );
#else
  void xCheckModeSplit        ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass 
#if JVET_AJ0226_MTT_SKIP
    , int mttSplitFlagCabacBits
#endif
  );
#endif
#else
#if JVET_Y0152_TT_ENC_SPEEDUP
  void xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, double *splitRdCostBest
#if JVET_AJ0226_MTT_SKIP
    , int mttSplitFlagCabacBits
#endif
  );
#else
  void xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode 
#if JVET_AJ0226_MTT_SKIP
    , int mttSplitFlagCabacBits
#endif
  );
#endif
#endif
  bool xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, bool adaptiveColorTrans);
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void xCheckRDCostSeparateTreeIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif
  void xCheckDQP              ( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx = false);
  void xCheckChromaQPOffset   ( CodingStructure& cs, Partitioner& partitioner);
#if !REMOVE_PCM
  void xFillPCMBuffer         ( CodingUnit &cu);
#endif

#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  bool xCheckRDCostHashInter  ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#else
  void xCheckRDCostHashInter  ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif
#if MERGE_ENC_OPT
  void xCheckSATDCostRegularMerge 
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
#if !MULTI_PASS_DMVR
                                , Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS]
#endif
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
                                , bool* applyBDMVR
#endif
#if JVET_AF0057
                                , bool* dmvrImpreciseMv
#endif

                              );
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void xCheckSATDCostRegularMergeOppositeLic
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtxOppositeLic, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
#if !MULTI_PASS_DMVR
    , Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS]
#endif
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
    , bool* applyBDMVR
#endif
  );
#endif
#if JVET_X0049_ADAPT_DMVR
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  void xCheckSATDCostBMMerge
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx& mrgCtx, MergeCtx& mrgCtxDir2, bool armcRefinedMotion, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
                                , bool* applyBDMVR
#endif
                              );
#else
  void xCheckSATDCostBMMerge
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
                                , bool* applyBDMVR
#endif
                              );
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
  void xCheckSATDCostCiipAffineMerge
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS]
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#endif
#if JVET_AG0276_NLIC || JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void xCheckSATDCostCiipMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS],
                               unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart, MergeCtx mergeCtx1);
#else
  void xCheckSATDCostCiipMerge 
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  void xCheckSATDCostCiipTmMerge
                              (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acTmMergeTmpBuffer[MRG_MAX_NUM_CANDS]
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#endif
  void xCheckSATDCostMmvdMerge 
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                               , uint32_t * mmvdLUT = NULL
#endif
                               );
#if JVET_AG0135_AFFINE_CIIP
  void xCheckSATDCostAffineMerge
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    , AffineMergeCtx& affineMergeCtx
#else
    , AffineMergeCtx affineMergeCtx
#endif 
    , MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS]
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#else
  void xCheckSATDCostAffineMerge 
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void xCheckSATDCostAffineMergeOppositeLic
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    , AffineMergeCtx& affineMergeCtx
#else
    , AffineMergeCtx affineMergeCtx
#endif
    , MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
  );
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  void xCheckSATDCostBMAffineMerge
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    , AffineMergeCtx& affineMergeCtxL0
#else
    , AffineMergeCtx affineMergeCtxL0
#endif
    , RefPicList reflist, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
  );
#endif
#if AFFINE_MMVD
  void xCheckSATDCostAffineMmvdMerge
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                          , uint32_t * affMmvdLUT
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  , uint8_t numBaseAffine = AF_MMVD_BASE_NUM
#endif
#endif
                               );
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
  void xCheckSATDCostTMMergeOppositeLic
  (CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
    , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
    , bool* applyBDMVR
#endif
  );
#endif
#endif
#if TM_MRG
  void xCheckSATDCostTMMerge
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
                                , bool* applyBDMVR
#endif
#if JVET_AF0057
                                , bool dmvr4TMImpreciseMv
#endif
                              );
#endif
#if !JVET_W0097_GPM_MMVD_TM && !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void xCheckSATDCostGeoMerge 
                              ( CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx geoMergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
                                , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart);
#endif
#else
  void xCheckRDCostAffineMerge2Nx2N
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
  void xCheckRDCostAffineMmvd2Nx2N
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
#endif
#if TM_MRG && !MERGE_ENC_OPT
  void xCheckRDCostTMMerge2Nx2N
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
#endif
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  bool xCheckRDCostInter      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#else
  void xCheckRDCostInter      ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif
  bool xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, double &bestIntPelCost);
  void xEncodeDontSplit       ( CodingStructure &cs, Partitioner &partitioner);

  void xCheckRDCostMerge2Nx2N ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#if MULTI_HYP_PRED
  void xCheckRDCostInterMultiHyp2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode);
  void predInterSearchAdditionalHypothesisMulti(const MEResultVec& in, MEResultVec& out, PredictionUnit& pu, const MergeCtx &mrgCtx);
#endif
#if ENABLE_OBMC 
  void xCheckRDCostInterWoOBMC(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode);
#endif
#if JVET_W0097_GPM_MMVD_TM
  void xCheckRDCostMergeGeoComb2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, bool isSecondPass = false);
#else
  void xCheckRDCostMergeGeo2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode);
#endif
  void xEncodeInterResidual(   CodingStructure *&tempCS
                             , CodingStructure *&bestCS
                             , Partitioner &partitioner
                             , const EncTestMode& encTestMode
                             , int residualPass       = 0
                             , bool* bestHasNonResi   = NULL
                             , double* equBcwCost     = NULL
                           );
#if REUSE_CU_RESULTS
  void xReuseCachedResult     ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &Partitioner );
#endif
  bool xIsBcwSkip(const CodingUnit& cu)
  {
    if (cu.slice->getSliceType() != B_SLICE)
    {
      return true;
    }
    return((m_pcEncCfg->getBaseQP() > 32) && ((cu.slice->getTLayer() >= 4)
       || ((cu.refIdxBi[0] >= 0 && cu.refIdxBi[1] >= 0)
       && (abs(cu.slice->getPOC() - cu.slice->getRefPOC(REF_PIC_LIST_0, cu.refIdxBi[0])) == 1
       ||  abs(cu.slice->getPOC() - cu.slice->getRefPOC(REF_PIC_LIST_1, cu.refIdxBi[1])) == 1))));
  }
#if JVET_AA0070_RRIBC
  void xCheckRDCostIBCMode    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, bool isSecondPass = false );
#else
  void xCheckRDCostIBCMode    ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode );
#endif
  void xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,      \
                                    const EncTestMode &encTestMode);

  void xCheckPLT              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode );
};

//! \}

#endif // __ENCMB__
