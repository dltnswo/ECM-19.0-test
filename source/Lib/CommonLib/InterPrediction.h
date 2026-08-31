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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"
#if JVET_Y0065_GPM_INTRA || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#include "IntraPrediction.h"
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
class IntraPrediction;
#endif
#endif
// forward declaration
class Mv;
#if INTER_LIC || (TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM) || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_Z0061_TM_OBMC
class Reshape;
#endif

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
struct BMSubBlkInfo : Area
{
  Distortion m_bmCost;
  Mv       m_mv[2];
  Mv       m_mvRefine[2];
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  Mv       m_mvRefineL0[2];
  Mv       m_mvRefineL1[2];
#endif
  PosType  m_cXInPU; // x coordinate of center relative to PU's top left sample
  PosType  m_cYInPU; // y coordinate of center relative to PU's top left sample
};
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
class TplMatchingBuffers
{
#if JVET_AK0101_REGRESSION_GPM_INTRA
  static const int nbCandMax = GEO_MAX_NUM_UNI_CANDS + GEO_BLEND_MAX_NUM_INTRA_CANDS;
#else
  static const int nbCandMax = GEO_MAX_NUM_UNI_CANDS;
#endif
  std::vector<Pel> m_acYuvRefAMLTemplate[nbCandMax][2];   // [idxCand][0: top, 1: left]  predicted samples
  bool  m_availRec;
  bool  m_availPred[nbCandMax];
  int   m_idxCand[2];  // the current 2 idx candidates of the 2 partitions

public:
  TplMatchingBuffers()
  {
    reset();
  }

  void reset()
  {
    std::memset( m_availPred, false, sizeof(bool) * nbCandMax );
    m_availRec    = false;
    m_idxCand[0]  = -1;
    m_idxCand[1]  = -1;
  }

  void set( int idx0, int idx1, Size cuSize )
  {
    if ( idx0 >= nbCandMax || idx1 >= nbCandMax )
    {
      return;
    }

    m_idxCand[0] = idx0;
    m_idxCand[1] = idx1;
    for (int iPart = 0; iPart < 2; iPart++)
    {
      int idxCand = m_idxCand[iPart];
      if ( idxCand < 0 || idxCand >= nbCandMax )
      {
        continue;
      }
      if ( m_acYuvRefAMLTemplate[idxCand][0].size() < cuSize.width )
      {
        m_acYuvRefAMLTemplate[idxCand][0].resize(cuSize.width);
      }
      if ( m_acYuvRefAMLTemplate[idxCand][1].size() < cuSize.height )
      {
        m_acYuvRefAMLTemplate[idxCand][1].resize(cuSize.height);
      }
    }
  }

  bool getAvailRec() { return m_availRec; }
  bool getAvailPred( const int iPart )
  {
    int idxCand = m_idxCand[iPart];
    return m_availPred[idxCand];
  }
  void  setAvailRec() { m_availRec = true;}
  void  setAvailPred( const int iPart )
  {
    int idxCand = m_idxCand[iPart];
    m_availPred[idxCand] = true;
  }

  Pel* getPred( const int iPart, const int iTopLeft, bool checkAvail=true ) 
  { 
    int idxCand = m_idxCand[iPart];
    if ( (checkAvail && !m_availPred[idxCand]) || idxCand == (-1) )
    {
      return nullptr;
    }
    else
    {
      return m_acYuvRefAMLTemplate[idxCand][iTopLeft].data();
    }
  }

#if JVET_AK0101_REGRESSION_GPM_INTRA
  int getIdxCand(int idx)
  {
    return m_idxCand[idx];
  }
#endif
};
#endif

#if JVET_AD0213_LIC_IMP
class TplMatchingCtrl;
#endif
class InterPrediction : public WeightPrediction
{
#if INTER_LIC || (JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_LIC)
public:
  PelUnitBuf        m_predictionBeforeLIC;
#if JVET_AD0213_LIC_IMP
  PelStorage        m_acPredBeforeLICBuffer[2];
  bool              m_encMotionEstimation;
  bool              m_isAddHypMC;
#endif
  bool              m_storeBeforeLIC;
#if JVET_AG0276_NLIC
  bool              m_skipDoLic;
#endif
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
public:
  AffineAMVPInfo m_affineAmvpInfo[AFFINE_MODEL_NUM][NUM_IMV_MODES][NUM_REF_PIC_LIST_01][MAX_NUM_REF];
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  bool              m_geoOBMC;
  bool              m_obmcLoadMode;
  bool              m_nbIsSCC;
  bool              m_pixelRefine;
  bool              m_neighbSccChecked;
  bool              m_intraObmcReload;
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  Pel*              m_acSgpmAffineRefTemplate[GEO_MAX_NUM_UNI_AFF_CANDS_ARMC][2];
  bool              m_bSgpmAffineRefTemplate[GEO_MAX_NUM_UNI_AFF_CANDS_ARMC];
#endif
#if JVET_AD0140_MVD_PREDICTION
  struct MvdDerivedInfo
  {
    std::vector<std::array<Mv, 3>> mvdAffineVec;
    std::vector<bool>              isCandValid;
    int                            firstValidIdx;
  };
#endif
#if JVET_Z0136_OOB
  bool isMvOOB      (const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, bool *mcMaskChroma, bool lumaOnly = false);
  bool isMvOOBSubBlk(const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly = false);
#endif
#if INTER_LIC || (TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM) || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_Z0061_TM_OBMC // note: already refactor
  Reshape*          m_pcReshape;
#endif

private:
#if INTER_LIC
  static const int  m_licShift     = 5;
  static const int  m_licRegShift  = 7;
  static const int  m_licShiftDiff = 12;
  int               m_licMultApprox[64];
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
#if !JVET_AK0212_GPM_OBMC_MODIFICATION
  Pel* m_intraOBMCBuf[MAX_NUM_COMPONENT];
#endif
  Pel* m_beforeOBMCBuf[MAX_NUM_COMPONENT];
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  std::vector<uint8_t> nbEachLength[2];
#endif

#if JVET_AG0136_INTRA_TMP_LIC
  std::array<int, 7> m_arrayLicParams;
#endif
#if INTER_LIC || JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  // buffer size for left/above current templates and left/above reference templates
#if JVET_AD0213_LIC_IMP
  Pel* m_pcLICRefLeftTemplate[2][MAX_NUM_COMPONENT];
  Pel* m_pcLICRefAboveTemplate[2][MAX_NUM_COMPONENT];
  Pel* m_pcLICRecLeftTemplate[MAX_NUM_COMPONENT];
  Pel* m_pcLICRecAboveTemplate[MAX_NUM_COMPONENT];
#if JVET_AC0112_IBC_LIC
  Pel* m_pcIBCLICRecLeftTemplate[MAX_NUM_COMPONENT];
  Pel* m_pcIBCLICRecAboveTemplate[MAX_NUM_COMPONENT];
#endif

  Pel* m_curLICRefLeftTemplate[2][MAX_NUM_COMPONENT];
  Pel* m_curLICRefAboveTemplate[2][MAX_NUM_COMPONENT];
  Pel* m_curLICRecLeftTemplate[MAX_NUM_COMPONENT];
  Pel* m_curLICRecAboveTemplate[MAX_NUM_COMPONENT];

  int  m_numTemplate[MAX_NUM_COMPONENT][2];
  int  m_shift[2][3] = {{0}}, m_scale[2][3] = {{0}}, m_offset[2][3] = {{0}};
  bool m_fillLicTpl[MAX_NUM_COMPONENT];
#else
  Pel* m_pcLICRefLeftTemplate;
  Pel* m_pcLICRefAboveTemplate;
  Pel* m_pcLICRecLeftTemplate;
  Pel* m_pcLICRecAboveTemplate;
#endif
#endif
#if JVET_AM0157_SGPM_BV_LIC
  int  m_ibcLicMean[2][3] = {{0}};
  int  m_ibcLicShift[2][2][3] = {{{0}}}, m_ibcLicScale[2][2][3] = {{{0}}}, m_ibcLicOffset[2][2][3] = {{{0}}};
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  // buffer size for left/above current templates
  Pel* m_pcCurTplLeft;
  Pel* m_pcCurTplAbove;
  Pel* m_pcRefTplLeft;
  Pel* m_pcRefTplAbove;
#endif
#if TM_AMVP
#if JVET_AD0213_LIC_IMP
public:
#endif
  AMVPInfo m_tplAmvpInfo[NUM_IMV_MODES][NUM_REF_PIC_LIST_01][MAX_NUM_REF];
#if INTER_LIC
  AMVPInfo m_tplAmvpInfoLIC[NUM_IMV_MODES][NUM_REF_PIC_LIST_01][MAX_NUM_REF];
#endif
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  MergeCtx m_pcMergeCtxList0;
  MergeCtx m_pcMergeCtxList1;
#endif
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
  const Picture* m_bmRefPic[2];
  CPelBuf   m_bmRefBuf[2];
  PelBuf    m_bmInterpolationTmpBuf;
  int       m_bmFilterSize;
  int       m_bmInterpolationHOfst;
  int       m_bmInterpolationVOfst;

  ClpRng    m_bmClpRng;
  ChromaFormat m_bmChFmt;
  PelBuf    m_bmPredBuf[2];
  DistParam m_bmDistParam;
  int32_t   m_bmCostShift;   // bilateral matching cost shift

  int32_t   m_bmSubBlkW;
  int32_t   m_bmSubBlkH;
  std::vector<BMSubBlkInfo> m_bmSubBlkList;
#endif
#if JVET_AH0119_SUBBLOCK_TM
  MotionInfo           m_sbMiBuf[3][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
protected:
  InterpolationFilter  m_if;

  Pel*                 m_acYuvPred            [NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMPONENT];
#if JVET_AF0057
  // one vector for each subblock
  Pel* m_dmvrRightBoundary[256];
  Pel* m_dmvrBottomBoundary[256];
#endif

#if JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION
  Pel*                 m_affineDmvrBlockTmp[NUM_REF_PIC_LIST_01];
#endif

#if MULTI_HYP_PRED
  PelStorage           m_additionalHypothesisStorage;
  int                  m_multiHypActive = 0;
#endif

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  PelStorage           m_obmcPelStorage;
#endif

#if JVET_AJ0237_INTERNAL_12BIT
  uint8_t              m_dmvrCostLambda;
#endif

  ChromaFormat         m_currChromaFormat;

  ComponentID          m_maxCompIDToPred;      ///< tells the predictor to only process the components up to (inklusive) this one - useful to skip chroma components during RD-search

  RdCost*              m_pcRdCost;

  int                  m_iRefListIdx;
  PelStorage           m_geoPartBuf[2];
  Mv*                  m_storedMv;

 /*buffers for bilinear Filter data for DMVR refinement*/
  Pel*                 m_cYuvPredTempDMVRL0;
  Pel*                 m_cYuvPredTempDMVRL1;
  int                  m_biLinearBufStride;
  /*buffers for padded data*/
  PelUnitBuf           m_cYuvRefBuffDMVRL0;
  PelUnitBuf           m_cYuvRefBuffDMVRL1;
  Pel*                 m_cRefSamplesDMVRL0[MAX_NUM_COMPONENT];
  Pel*                 m_cRefSamplesDMVRL1[MAX_NUM_COMPONENT];
  Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                             Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                             Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                             Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                             Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
  uint64_t m_SADsArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];
#if MULTI_PASS_DMVR
#if JVET_X0049_BDMVR_SW_OPT
  Mv                   m_searchEnlargeOffsetBilMrg[5][BDMVR_INTME_AREA];
  uint16_t             m_searchEnlargeOffsetToIdx[5][BDMVR_INTME_AREA];
  uint16_t             m_searchEnlargeOffsetNum[5];
  uint64_t             m_sadEnlargeArrayBilMrg[BDMVR_INTME_AREA];
#else
  Mv                   m_searchEnlargeOffsetBilMrg[BDMVR_INTME_AREA];
  uint64_t             m_sadEnlargeArrayBilMrg[BDMVR_INTME_AREA];
  int                  m_searchPriorityBilMrg[BDMVR_INTME_AREA];
#endif
  int                  m_costShiftBilMrg1[BDMVR_INTME_AREA];
  int                  m_costShiftBilMrg2[BDMVR_INTME_AREA];
#endif

  Pel                  m_gradBuf[2][(AFFINE_MIN_BLOCK_SIZE + 2) * (AFFINE_MIN_BLOCK_SIZE + 2)];
  int                  m_dMvBuf[2][16 * 2];
#if JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION
  int                  m_affineSbMvIntX[NUM_REF_PIC_LIST_01][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE];
  int                  m_affineSbMvIntY[NUM_REF_PIC_LIST_01][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE];
  int                  m_affineSbMvFracX[NUM_REF_PIC_LIST_01][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE];
  int                  m_affineSbMvFracY[NUM_REF_PIC_LIST_01][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE][MAX_CU_SIZE / AFFINE_MIN_BLOCK_SIZE];
#endif
  bool                 m_skipPROF;
  bool                 m_encOnly;
  bool                 m_isBi;
#if JVET_AE0046_BI_GPM
  bool                 m_lumaBdofReady;
#endif

  Pel*                 m_gradX0;
  Pel*                 m_gradY0;
  Pel*                 m_gradX1;
  Pel*                 m_gradY1;
#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
  Pel*                 m_absGx;
  Pel*                 m_absGy;
  Pel*                 m_dIx;
  Pel*                 m_dIy;
  Pel*                 m_dI;
  Pel*                 m_signGxGy;
  int*                 m_tmpxSample32bit;
  int*                 m_tmpySample32bit;
  int*                 m_sumAbsGxSample32bit;
  int*                 m_sumAbsGySample32bit;
  int*                 m_sumDIXSample32bit;
  int*                 m_sumDIYSample32bit;
  int*                 m_sumSignGyGxSample32bit;
  bool                 m_bdofMvRefined;
  Mv                   m_bdofSubPuMvOffset[BDOF_SUBPU_MAX_NUM];
#if JVET_AE0091_ITERATIVE_BDOF
  Mv                   m_bdofSubPuMvOffse2[BDOF_SUBPU_MAX_NUM];
#endif
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  uint8_t              m_sbtmvpSubPuDerived[BDOF_SUBPU_MAX_NUM];
  bool                 m_doAffineSubPuBdof;
  bool                 m_skipAffineFirstIterBdof;
#endif
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
  int32_t*             m_piDotProduct1;
  int32_t*             m_piDotProduct2;
  int32_t*             m_piDotProduct3;
  int32_t*             m_piDotProduct5;
  int32_t*             m_piDotProduct6;
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
  Pel*                 m_Gx;
  Pel*                 m_Gy;
#endif
  bool                 m_subPuMC;
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  bool                 m_useHighPrecMv;
  bool                 m_isAffBdofChroma;
  bool                 m_isDeriveOobMask;
#endif
  int                  m_ibcBufferWidth;
#if JVET_Z0153_IBC_EXT_REF
  int                  m_ibcBufferHeight;
#endif
#if JVET_Z0118_GDR
  PelStorage           m_ibcBuffer0; // for dirty
  PelStorage           m_ibcBuffer1; // for clean
#else
  PelStorage           m_ibcBuffer;
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
  Mv                   m_pixelAffineMotionBuf[MAX_CU_SIZE][MAX_CU_SIZE];
#if JVET_AL0079_AFFINE_MC
  Mv                   m_pixelAffineMotion64Buf[MAX_CU_SIZE][MAX_CU_SIZE];
#endif
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  std::vector <int16_t> m_bcwBlendBuf;
  std::vector<Pel>      m_tempPel[2];
#endif

#if JVET_AD0140_MVD_PREDICTION
  Pel m_acYuvRefAMLBiPredTemplateCache[MAX_NUM_REFIDX][NUM_REF_PIC_LIST_01][MAX_NUM_CANDS][2][MAX_CU_SIZE];
  Pel m_acYuvRefAMLBiPredTemplateIdMotionCache[MAX_NUM_REFIDX][NUM_REF_PIC_LIST_01][MAX_NUM_CANDS][2][MAX_CU_SIZE];
#endif

#if JVET_AE0159_FIBC || JVET_AE0078_IBC_LIC_EXTENSION || JVET_AE0059_INTER_CCCM || JVET_AF0073_INTER_CCP_MERGE
  IntraPrediction*  m_pcIntraPred;
  Area m_ibcRefArea;
#endif

  void xIntraBlockCopy          (PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID);
  int             rightShiftMSB(int numer, int    denom);
#if MULTI_PASS_DMVR
  void            xPredInterUni            ( const PredictionUnit &pu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred,
                                             const bool &bi, const bool &bioApplied, const bool luma, const bool chroma,
                                             const bool isBdofMvRefine = false);
  void            xPredInterBiSubPuBDOF    ( PredictionUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma );
#if JVET_Z0136_OOB
  void            applyBiOptFlow           ( const bool isBdofMvRefine, const int bdofBlockOffset, const PredictionUnit &pu,
                                            const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1,
                                            PelUnitBuf &yuvDst, const BitDepths &clipBitDepths, bool *mcMask[2] = NULL, bool *mcMaskChroma[2] = NULL, bool *isOOB = NULL
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
                                            , int ww = 4, int hh = 4
#endif
#if JVET_AE0091_ITERATIVE_BDOF
                                            ,int iter = 0
#endif
  );
#else
  void            applyBiOptFlow           ( const bool isBdofMvRefine, const int bdofBlockOffset, const PredictionUnit &pu,
                                             const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1,
                                             PelUnitBuf &yuvDst, const BitDepths &clipBitDepths );
#endif
#else
#if JVET_Z0136_OOB
  void            applyBiOptFlow           (const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths, bool *mcMask[2] = NULL, bool *mcMaskChroma[2] = NULL, bool *isOOB = NULL);
#else
  void            applyBiOptFlow           (const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths);
#endif
  void            xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi, const bool& bioApplied, const bool luma, const bool chroma );
#endif
#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
#if JVET_Z0136_OOB
  void            subBlockBiOptFlow        ( Pel* dstY, const int dstStride, const Pel* src0, const int src0Stride, const Pel* src1,
                                             const int src1Stride, int bioParamOffset, const int bioParamStride, int width, int height,
                                             const ClpRng& clpRng, const int shiftNum, const int offset, const int limit, bool *mcMask[2], int mcStride, bool *isOOB = NULL
#if JVET_AL0081_BDOF_LDB_MV_REFINE
                                            ,bool noMeanRemove = false
#endif
                                           );
#else
  void            subBlockBiOptFlow        ( Pel* dstY, const int dstStride, const Pel* src0, const int src0Stride, const Pel* src1,
                                             const int src1Stride, int bioParamOffset, const int bioParamStride, int width, int height,
                                             const ClpRng& clpRng, const int shiftNum, const int offset, const int limit );
#endif
#endif
#if ENABLE_OBMC
  PelStorage           m_tmpObmcBufL0;
  PelStorage           m_tmpObmcBufT0;
  PelStorage           m_tmpSubObmcBuf;
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  PelUnitBuf           m_tmpSubBuf1;
  PelUnitBuf           m_tmpSubBuf2;
  PelUnitBuf           m_tmpSubBuf3;
  PelUnitBuf           m_tmpSubBuf4;
  PelUnitBuf           m_tmpSubBuf0;
  PelStorage           m_tmpNbObmcBufA;
  PelStorage           m_tmpNbObmcBufL;
  PelStorage           m_tmpNbIntraObmcBufA;
  PelStorage           m_tmpNbIntraObmcBufL;
#endif
#if JVET_AK0076_EXTENDED_OBMC_IBC && !JVET_AK0212_GPM_OBMC_MODIFICATION
  PelStorage           m_tmpIntraObmcBufL0;
  PelStorage           m_tmpIntraObmcBufT0;
#endif
#endif
#if MULTI_PASS_DMVR
  void xPredInterBiBDMVR        ( PredictionUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma, PelUnitBuf *yuvPredTmp = NULL );
#endif
#if JVET_AE0091_ITERATIVE_BDOF
  void xPredInterBiBDMVR2       ( PredictionUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma, PelUnitBuf *yuvPredTmp = NULL, int iter = 1 );
#endif
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred, const bool luma = true, const bool chroma = true, PelUnitBuf* yuvPredTmp = NULL );
  void xPredInterBlk            ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                 , const bool& bioApplied
                                 , bool isIBC
                                 , const std::pair<int, int> scalingRatio = SCALE_1X
                                 , SizeType dmvrWidth = 0
                                 , SizeType dmvrHeight = 0
                                 , bool bilinearMC = false
                                 , Pel *srcPadBuf = NULL
                                 , int32_t srcPadStride = 0
#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                                 , bool isAML = false
#if INTER_LIC
                                 , bool doLic = false
                                 , Mv   mvCurr = Mv(0, 0)
#endif
#endif
#if JVET_Z0061_TM_OBMC
                                 , bool fastOBMC = false
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
                                 , const bool frac64 = false
#endif
                                 );

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void xPredIBCBlkPadding       (const PredictionUnit& pu, ComponentID compID, const Picture* refPic, const ClpRng& clpRng
                               , CPelBuf& refBufBeforePadding, const Position& refOffsetByIntBv, int xFrac, int yFrac
                               , int width, int height, int filterIdx);
#endif

  void xAddBIOAvg4              (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng);
  void xBioGradFilter           (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth);
  void xCalcBIOPar              (const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth);
  void xCalcBlkGradient         (int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize);
#if MULTI_PASS_DMVR
#if JVET_Z0136_OOB
  void xWeightedAverage         ( const bool isBdofMvRefine, const int bdofBlockOffset, const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied, const bool lumaOnly = false, const bool chromaOnly = false, PelUnitBuf* yuvDstTmp = NULL, bool *mcMask[2] = NULL, int mcStride = -1, bool *mcMaskChroma[2] = NULL, int mcCStride = -1, bool *isOOB = NULL
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
                                 , int ww = 4, int  hh = 4
#endif
#if JVET_AE0091_ITERATIVE_BDOF
                                 ,int iter = 0
#endif
  );
#else
  void xWeightedAverage         ( const bool isBdofMvRefine, const int bdofBlockOffset, const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied, const bool lumaOnly = false, const bool chromaOnly = false, PelUnitBuf* yuvDstTmp = NULL );
#endif
#else
#if JVET_Z0136_OOB
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied, const bool lumaOnly = false, const bool chromaOnly = false, PelUnitBuf* yuvDstTmp = NULL, bool *mcMask[2] = NULL, int mcStride = -1, bool *mcMaskChroma[2] = NULL, int mcCStride = -1, bool *isOOB = NULL );
#else
  void xWeightedAverage         ( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied, const bool lumaOnly = false, const bool chromaOnly = false, PelUnitBuf* yuvDstTmp = NULL );
#endif
#endif
#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if !INTER_LIC
  template <bool trueAfalseL>
  void xGetPredBlkTpl           (const CodingUnit& cu, const ComponentID compID, const CPelBuf& refBuf, const Mv& mv, const int posW, const int posH, const int tplSize, Pel* predBlkTpl
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      , bool AML = false
#endif
                      );
#endif
  void xWeightedAverageY        (const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs);
#endif
#if JVET_W0090_ARMC_TM
#if JVET_AD0140_MVD_PREDICTION
  template <bool exitIfOob, int iAbove1Left2All3>
  bool xPredAffineTpl           (const PredictionUnit& pu, const RefPicList& eRefPicList, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate
#else
  void xPredAffineTpl           (const PredictionUnit &pu, const RefPicList &eRefPicList, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    , AffineMergeCtx affMrgCtx, bool isBilinear
#endif
#if JVET_AF0190_RPR_TMP_REORDER_LIC
    , bool           enableRpr = false
#endif
  );
#endif
#if AFFINE_ENC_OPT
#if JVET_Z0136_OOB
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, RefPicList eRefPicList, const bool genChromaMv = false, const std::pair<int, int> scalingRatio = SCALE_1X, const bool calGradient = false);
#else
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, const bool genChromaMv = false, const std::pair<int, int> scalingRatio = SCALE_1X, const bool calGradient = false);
#endif
#else
#if JVET_Z0136_OOB
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, RefPicList eRefPicList, const bool genChromaMv = false, const std::pair<int, int> scalingRatio = SCALE_1X);
#else
  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng, const bool genChromaMv = false, const std::pair<int, int> scalingRatio = SCALE_1X );
#endif
#endif

  static bool xCheckIdenticalMotion( const PredictionUnit& pu );

  void xSubPuMC                    (PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X, const bool luma = true, const bool chroma = true);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  bool xGetSubPuGroupArea2D        (PredictionUnit& pu, PredictionUnit& subPu, uint8_t* sbtmvpSubPuDerivedPtr, Position& subPuStartPos);
  bool xGetSubPuGroupAreaStartPos  (PredictionUnit& pu, Position& subPuStartPos, uint8_t* sbtmvpSubPuDerivedPtr);
  bool xCheckIdenticalMotionInfo   (MotionInfo orgMotionInfo, MotionInfo targetMotionInfo, MergeType puMergeType);
#endif
#if ENABLE_OBMC
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
#if JVET_AK0076_EXTENDED_OBMC_IBC
  void xSubblockOBMC               (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, PelUnitBuf &pcYuvBefore, std::vector<Pel>* lmcsLut, int iDir, bool bSubMotion = false, bool bIsIntra = false);
#else
  void xSubblockOBMC               (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, PelUnitBuf &pcYuvBefore, int iDir, bool bSubMotion = false, bool bIsIntra = false);
#endif
#else
  void xSubblockOBMC               (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, int iDir, bool bSubMotion = false);
#endif

#if JVET_AK0076_EXTENDED_OBMC_IBC
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void xSubBlockMotionCompensation(PredictionUnit& pu, PelUnitBuf& pcYuvPred, bool lumaOnly = false, bool chromaOnly = false, std::vector<Pel>* lmcsLut = nullptr);
#else
  void xSubBlockMotionCompensation (PredictionUnit &pu, PelUnitBuf &pcYuvPred, bool lumaOnly = false, std::vector<Pel>* lmcsLut = nullptr);
#endif
#else
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void xSubBlockMotionCompensation (PredictionUnit& pu, PelUnitBuf& pcYuvPred, bool lumaOnly = false, bool chromaOnly = false);
#else
  void xSubBlockMotionCompensation (PredictionUnit &pu, PelUnitBuf &pcYuvPred);
#endif
#endif
  void xSubblockOBMCBlending       (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc1, PelUnitBuf &pcYuvPredSrc2, PelUnitBuf &pcYuvPredSrc3, PelUnitBuf &pcYuvPredSrc4, bool isAboveAvail = false, bool isLeftAvail = false, bool isBelowAvail = false, bool isRightAvail = false, bool bSubMotion = false);
#endif
#if !BDOF_RM_CONSTRAINTS
  void xSubPuBio                   (PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X, PelUnitBuf* yuvDstTmp = NULL);
#endif

#if !JVET_AI0183_MVP_EXTENSION
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if JVET_AG0098_AMVP_WITH_SBTMVP
  MotionInfo      m_subPuMiBuf[SUB_BUFFER_SIZE][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#else
  MotionInfo      m_subPuMiBuf[SUB_TMVP_NUM][(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#else
  MotionInfo      m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
#endif
#endif
#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_Z0061_TM_OBMC || JVET_AA0061_IBC_MBVD || (JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV)
  Pel*   m_acYuvCurAMLTemplate[2][MAX_NUM_COMPONENT];   //0: top, 1: left
  bool   m_bAMLTemplateAvailabe[2];
  Pel*   m_acYuvRefAboveTemplate[2][MAX_NUM_COMPONENT];   //0: list0, 1: list1
  Pel*   m_acYuvRefLeftTemplate[2][MAX_NUM_COMPONENT];   //0: list0, 1: list1
  Pel*   m_acYuvRefAMLTemplate[2][MAX_NUM_COMPONENT];   //0: top, 1: left
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  Pel*   m_acYuvRefAMLTemplatePart0[2 + 2];   //0: top, 1: left, 2: top, 3: left
  Pel*   m_acYuvRefAMLTemplatePart1[2 + 2];   //0: top, 1: left, 2: top, 3: left
#endif
#endif
#if JVET_Z0061_TM_OBMC
  Pel *m_acYuvRefAboveTemplateOBMC[2][MAX_NUM_COMPONENT];   // 0: current motion, 1: neighbour motion
  Pel *m_acYuvRefLeftTemplateOBMC[2][MAX_NUM_COMPONENT];    // 0: current motion, 1: neighbour motion
  Pel *m_acYuvBlendTemplateOBMC[2][MAX_NUM_COMPONENT];      // 0: top, 1: left
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  bool   m_tplWeightTblInitialized;
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  Pel*   m_tplWeightTblDict[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_TOTAL_NUM_PARTITION_MODE];
  Pel    m_tplColWeightTblDict[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_TOTAL_NUM_PARTITION_MODE][GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];
#else
  Pel*   m_tplWeightTblDict[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE];
  Pel    m_tplColWeightTblDict[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];
#endif
  Pel**  m_tplWeightTbl;
  Pel    (*m_tplColWeightTbl)[GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];
#endif

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  Distortion*   m_mbvdCandCostList;
  int*          m_mbvdSearchCandsList;
  bool*         m_mbvdTestedCandsList;
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel      *m_cacheModel;
#endif
  PelStorage       m_colorTransResiBuf[3];  // 0-org; 1-act; 2-tmp
#if MULTI_HYP_PRED
  void xAddHypMC(PredictionUnit& pu, PelUnitBuf& predBuf, PelUnitBuf* predBufWOBIO, const bool lumaOnly = false);
#endif

public:
  InterPrediction();
  virtual ~InterPrediction();

#if INTER_LIC || (TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM) || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_Z0061_TM_OBMC
#if JVET_Z0153_IBC_EXT_REF
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
#if JVET_AJ0237_INTERNAL_12BIT
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, Reshape* reshape, const int picWidth, const int picHeight, const int bitDepth);
#else
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, Reshape* reshape, const int picWidth, const int picHeight);
#endif
#else
#if JVET_AJ0237_INTERNAL_12BIT
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, Reshape* reshape, const int picWidth, const int bitDepth);
#else
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, Reshape* reshape, const int picWidth);
#endif
#endif
#else
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, Reshape* reshape);
#endif
#else
  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);
#endif

  void destroy();
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  template <bool trueAfalseL, int compId> Pel* getCurAMLTemplate() { return m_acYuvCurAMLTemplate[trueAfalseL ? 0 : 1][compId]; }
  template <bool trueAfalseL, int compId> Pel* getRefAMLTemplate() { return m_acYuvRefAMLTemplate[trueAfalseL ? 0 : 1][compId]; }
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
  void     setUniRefIdxLC(PredictionUnit &pu);
  void     setUniRefListAndIdx(PredictionUnit &pu);
  void     reorderRefCombList(PredictionUnit &pu, std::vector<RefListAndRefIdx> &refListComb
    , RefPicList currRefList
    , std::vector<MotionInfoPred> &miPredList
#if JVET_AD0140_MVD_PREDICTION
    , std::vector<Mv> (&cMvdDerivedVec)[3]
    , bool &isMvdDerivedVecOrigSpecified
#endif
  );
#if JVET_AG0136_INTRA_TMP_LIC
  std::array<int, 7>& getArrayLicParams() { return m_arrayLicParams; }
#endif


#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
  void      deriveMVDCandVecFromMotionInforPred             (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> &cMvdDerivedVec, bool &isMvdDerivedVecOrigSpecified);
  void      deriveAffineMVDCandVecFromMotionInforPred       (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> cMvdDerivedVec[3], bool &isMvdDerivedVecOrigSpecified);
  void      deriveMVDCandVecFromMotionInforPredGeneral      (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> &cMvdDerivedVec, bool &isMvdDerivedVecOrigSpecified);
  void      deriveAffineMVDCandVecFromMotionInforPredGeneral(const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> cMvdDerivedVec[3], bool &isMvdDerivedVecOrigSpecified);
#else
  void      deriveMVDCandVecFromMotionInforPred             (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> &cMvdDerivedVec);
  void      deriveAffineMVDCandVecFromMotionInforPred       (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> cMvdDerivedVec[3]);
  void      deriveMVDCandVecFromMotionInforPredGeneral      (const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> &cMvdDerivedVec);
  void      deriveAffineMVDCandVecFromMotionInforPredGeneral(const PredictionUnit &pu, std::vector<MotionInfoPred> &miPredList, RefPicList eRefPicList, std::vector<Mv> cMvdDerivedVec[3]);
#endif
#endif
  void     setBiRefPairIdx                                  (PredictionUnit &pu);
  void     setBiRefIdx                                      (PredictionUnit &pu);
  void     reorderRefPairList                               (PredictionUnit &pu, std::vector<RefPicPair> &refPairList, std::vector<MotionInfoPred> &miPredList
#if JVET_AD0140_MVD_PREDICTION
    , std::vector<Mv> (&cMvdDerivedVec)[2][3]
    , bool &isMvdDerivedVecOrigSpecified
#endif
  );
#endif

  // inter
#if JVET_AL0079_AFFINE_MC
  void    setEncOnly(bool b) { m_encOnly = b; }
#endif
#if JVET_AE0046_BI_GPM
  void    setLumaBdofReady    (bool isReady) { m_lumaBdofReady = isReady; }
  void    convert2HighPrec    (PredictionUnit& pu, PelUnitBuf& predBuf, bool lumaOnly, bool chromaOnly, PelUnitBuf* yuvPredTmp = nullptr);
#endif
  void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList = REF_PIC_LIST_X, const bool luma = true, const bool chroma = true, PelUnitBuf* predBufWOBIO = nullptr );
  void    motionCompensation  (PredictionUnit &pu, const RefPicList &eRefPicList = REF_PIC_LIST_X, const bool luma = true, const bool chroma = true );
#if NN_LF_TRYMODE_FIX
  void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X, const bool luma = true, const bool chroma = true, PelUnitBuf *pPredBuf = 0 );
#else
  void    motionCompensation  (CodingUnit &cu,     const RefPicList &eRefPicList = REF_PIC_LIST_X, const bool luma = true, const bool chroma = true );
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS || JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  inline void getPredIBCBlk   (const PredictionUnit& pu, ComponentID comp, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, bool bilinearMC = false
#if JVET_AC0112_IBC_LIC
                          , bool bypassIBCLic = false
#endif
  )
  {
#if JVET_AC0112_IBC_LIC
    bool ibcLicFlag     = pu.cu->ibcLicFlag;
    bool storeBeforeLIC = m_storeBeforeLIC;
    if (bypassIBCLic)
    {
      pu.cu->ibcLicFlag = false;
      m_storeBeforeLIC  = false;
    }
#endif
    xPredInterBlk             (comp, pu, refPic, _mv, dstPic, false, pu.cu->slice->clpRng(comp), false, true, SCALE_1X, 0, 0, bilinearMC);
#if JVET_AC0112_IBC_LIC
    if (bypassIBCLic)
    {
      pu.cu->ibcLicFlag = ibcLicFlag;
      m_storeBeforeLIC  = storeBeforeLIC;
    }
#endif
  }
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  inline void LicItmp         (PredictionUnit &pu, PelBuf &dstBuf
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                               , const int requiredTemplate
#else
                               , const bool isLinearTransformDone
#endif
                               )
  {
    CHECKD(pu.cu->ibcLicFlag!=pu.cu->tmpLicFlag, "InterPrediction::LicItmp: abnormal: pu.cu->ibcLicFlag and pu.cu->tmpLicFlag should be equal");
    CHECKD((pu.cu)->ibcFilterFlag, "`pu.cu->ibcFilterFlag` is not false.");
    if(pu.cu->ibcLicFlag || pu.cu->tmpLicFlag )
    {
      xLocalIlluComp(pu, COMPONENT_Y, pu.mv[0], dstBuf
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                     , false, requiredTemplate
#else
                     , isLinearTransformDone
#endif
                     );
    }
  }
#endif
#if ENABLE_OBMC
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
#if JVET_AK0076_EXTENDED_OBMC_IBC
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void    subBlockOBMC        (PredictionUnit &pu, PelUnitBuf *pDst = nullptr, IntraPrediction *pcIntraPred = nullptr, bool lumaOnly = false, bool chromaOnly = false, bool usePreCheck = false, bool saveMode = false);
#else
  void    subBlockOBMC        (PredictionUnit &pu, PelUnitBuf *pDst = nullptr, IntraPrediction *pcIntraPred = nullptr, bool lumaOnly = false);
#endif
#else
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void    subBlockOBMC        (PredictionUnit &pu, PelUnitBuf *pDst = nullptr, IntraPrediction *pcIntraPred = nullptr, bool lumaOnly = false, bool chromaOnly = false, bool usePreCheck = false, bool saveMode = false);
#else
  void    subBlockOBMC        (PredictionUnit &pu, PelUnitBuf *pDst = nullptr, IntraPrediction *pcIntraPred = nullptr);
#endif
#endif
#else
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void    subBlockOBMC        (PredictionUnit& pu, PelUnitBuf* pDst = nullptr, bool lumaOnly = false, bool chromaOnly = false, bool usePreCheck = false, bool saveMode = false);
#else
  void    subBlockOBMC        (PredictionUnit  &pu, PelUnitBuf *pDst = nullptr);
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool    isSCC                   (const PredictionUnit  &pu);
#if JVET_AK0076_EXTENDED_OBMC_IBC
  bool    skipObmcConditionByPixel(PredictionUnit& pu, ComponentID comp, int width, int height, const Pel* src, int strideSrc, const Pel* dst, int strideDst, int bitDepth, std::vector<Pel>* lmcsLut);
#else
  bool    skipObmcConditionByPixel(PredictionUnit& pu, ComponentID comp, int width, int height, const Pel* src, int strideSrc, const Pel* dst, int strideDst, int bitDepth);
#endif
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void    initTplWeightTable();

  template <bool regular, uint8_t top0Left1TrLeft2 = 0>
  Pel*    getTplWeightTableCU(int splitDir)
  {
    if (regular)
    {
      return m_tplWeightTbl[splitDir];
    }
    else if (top0Left1TrLeft2 == 2)
    {
      return m_tplColWeightTbl[splitDir];
    }
    else
    {
      int16_t angle = g_geoParams[splitDir][0];
      if (g_angle2mirror[angle] == 2)
      {
        return m_tplWeightTbl[splitDir] + (top0Left1TrLeft2 == 0 ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : -GEO_MODE_SEL_TM_SIZE); // Shift to template pos
      }
      else if (g_angle2mirror[angle] == 1)
      {
        return m_tplWeightTbl[splitDir] - (top0Left1TrLeft2 == 0 ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : -GEO_MODE_SEL_TM_SIZE); // Shift to template pos;
      }
      else
      {
        return m_tplWeightTbl[splitDir] - (top0Left1TrLeft2 == 0 ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : GEO_MODE_SEL_TM_SIZE); // Shift to template pos;
      }
    }
  }
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void    deriveGpmSplitMode(PredictionUnit& pu, MergeCtx &geoMrgCtx
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
                           , MergeCtx(&geoTmMrgCtx)[GEO_NUM_TM_MV_CAND]
#endif
#if JVET_AG0164_AFFINE_GPM
                           , AffineMergeCtx& affGeoMrgCtx
#endif
#if JVET_Y0065_GPM_INTRA
                           , IntraPrediction* pcIntraPred
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                           , Mv* geoBvList
#endif
  );
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  std::pair<int8_t,int8_t> getGeoBlendCandIndexes( const int idxCand, std::vector<int8_t>& listMergeCand0, std::vector<int8_t>& listMergeCand1, int8_t* nbZscanPairList = 0 );
  bool    getGeoBlendCand( const CodingUnit& cu, MergeCtx& geoMrgCtx, const int idxCand, GeoBlendInfo& geoBIdst, GeoBlendInfo* geoBlendInfoList=nullptr, int* numGeoBlendInfoList=nullptr );
  void    motionCompensationGeoBlend( CodingUnit& cu, MergeCtx& geoMrgCtx
#if JVET_AE0046_BI_GPM
                              , Mv(&subMvBuf)[MRG_MAX_NUM_CANDS << 1][MAX_NUM_SUBCU_DMVR]
                              , Mv(&subBdofBuf)[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM]
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA || JVET_AK0212_GPM_OBMC_MODIFICATION
                              , IntraPrediction* pcIntraPred
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
                              , std::vector<Pel>* reshapeLUT
#endif
    );

  TplMatchingBuffers  m_tplBuffers;
#if JVET_AK0101_REGRESSION_GPM_INTRA
  std::pair<int8_t, int8_t> getGeoBlendIntraCandIndexes(const int idxCand, std::vector<int8_t>& intraCandList, std::vector<int8_t>& interCandList, int8_t* nbZscanPairList = 0);
  bool    getGeoBlendIntraCand(const CodingUnit& cu, IntraPrediction* pcIntraPred, MergeCtx& geoMrgCtx, const int idxCand, GeoBlendInfo& geoBIdst, uint8_t* mpmList, const int mpmNum, GeoBlendInfo* geoBlendInfoList = nullptr, int* numGeoBlendInfoList = nullptr);
#endif
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void    motionCompensationGeo(CodingUnit &cu, MergeCtx &geoMrgCtx
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
                              , MergeCtx (&geoTmMrgCtx)[GEO_NUM_TM_MV_CAND]
#endif
#if JVET_AG0164_AFFINE_GPM
                              , AffineMergeCtx &gpmAffMrgCtx
#if JVET_AJ0274_GPM_AFFINE_TM
                              , AffineMergeCtx &gpmAffTmMrgCtx
#endif
#endif
#if JVET_AE0046_BI_GPM
                              , Mv(&subMvBuf)[MRG_MAX_NUM_CANDS << 1][MAX_NUM_SUBCU_DMVR]
                              , Mv(&subBdofBuf)[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM]
#endif
#if JVET_Y0065_GPM_INTRA
                              , IntraPrediction* pcIntraPred, std::vector<Pel>* reshapeLUT
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                              , Mv* geoBvList
#endif
  );
#else
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
#if JVET_Y0065_GPM_INTRA
  void    motionCompensationGeo(CodingUnit &cu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, IntraPrediction* pcIntraPred, std::vector<Pel>* reshapeLUT);
#else
  void    motionCompensationGeo(CodingUnit &cu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1);
#endif
#else
#if JVET_Y0065_GPM_INTRA
#if JVET_AE0046_BI_GPM
  void    motionCompensationGeo(CodingUnit& cu, MergeCtx& geoMrgCtx
                               ,Mv(&subMvBuf)[MRG_MAX_NUM_CANDS << 1][MAX_NUM_SUBCU_DMVR]
                               ,Mv(&subBdofBuf)[MRG_MAX_NUM_CANDS][BDOF_SUBPU_MAX_NUM]
                               ,IntraPrediction* pcIntraPred, std::vector<Pel>* reshapeLUT);
#else
  void    motionCompensationGeo( CodingUnit &cu, MergeCtx &geoMrgCtx, IntraPrediction* pcIntraPred, std::vector<Pel>* reshapeLUT );
#endif
#else
  void    motionCompensationGeo(CodingUnit &cu, MergeCtx &GeoMrgCtx);
#endif
#endif
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  void    weightedGeoBlk       (PredictionUnit &pu, const uint8_t splitDir, const uint8_t bldIdx, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#if JVET_Y0065_GPM_INTRA
  void    weightedGeoBlkRounded(PredictionUnit &pu, const uint8_t splitDir, const uint8_t bldIdx, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#else
  void    weightedGeoBlk       (PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#if JVET_Y0065_GPM_INTRA
  void    weightedGeoBlkRounded( PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  void    weightedBlendBlk    ( const PredictionUnit& pu, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, WeightBuf& weightBuf, const int log2WeightBase, const bool roundOutputBD);
  void    weightedBlend       ( const PredictionUnit& pu, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const bool chromaOnly, const bool lumaOnly, const bool roundOutputBD = false);
  void    weightedAffineBlk   ( const PredictionUnit& pu, WeightBuf& weightBuf, const int log2WeightBase, AffineBlendingModel& blendModel );
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void    weightObmcBoundary(Pel* orgDst, Pel* orgSrc, const int strideDst, const int strideSrc, const int width, const int height, const int dir, const ComponentID comp, const int blendMode = -1, const bool subMotion = false);
  void    weightObmcInnerBoundary(const ComponentID comp, Pel* pOrgDst, Pel* pOrgSrc1, Pel* pOrgSrc2, Pel* pOrgSrc3, Pel* pOrgSrc4, const int dstStride, const int srcStride, const int width, const int height, bool isAboveAvail, bool isLeftAvail, bool isBelowAvail, bool isRightAvail);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  void    getBestGeoTMModeList(PredictionUnit &pu, uint8_t& numValidInList, uint8_t(&modeList)[GEO_NUM_SIG_PARTMODE], Pel* (&pRefTopPart0)[GEO_NUM_TM_MV_CAND], Pel* (&pRefLeftPart0)[GEO_NUM_TM_MV_CAND], Pel* (&pRefTopPart1)[GEO_NUM_TM_MV_CAND], Pel* (&pRefLeftPart1)[GEO_NUM_TM_MV_CAND]);
#endif
  void    getBestGeoModeList  (PredictionUnit &pu, uint8_t& numValidInList, uint8_t(&modeList)[GEO_NUM_SIG_PARTMODE], Pel* pRefTopPart0, Pel* pRefLeftPart0, Pel* pRefTopPart1, Pel* pRefLeftPart1
#if JVET_Y0065_GPM_INTRA
                             , Pel** pIntraRefTopPart0 = nullptr, Pel** pIntraRefLeftPart0 = nullptr, Pel** pIntraRefTopPart1 = nullptr, Pel** pIntraRefLeftPart1 = nullptr
#endif
  );
  template <bool trueTFalseL>
  void    weightedGeoTpl      (PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#if JVET_AF0057
  bool     dmvrEnableEncoderCheck;
  void     xDmvrSetEncoderCheckFlag(bool enableFlag) { dmvrEnableEncoderCheck = enableFlag; }
  bool     xDmvrGetEncoderCheckFlag() { return dmvrEnableEncoderCheck; }
#endif
  void xPrefetch              (PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma);
  void xPad                   (PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId);
  void xFinalPaddedMCForDMVR  (PredictionUnit& pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01], bool blockMoved );
  void xBIPMVRefine           (int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray, int width, int height);
  uint64_t xDMVRCost          (int bitDepth, Pel* pRef, uint32_t refStride, const Pel* pOrg, uint32_t orgStride, int width, int height);
  void xinitMC                (PredictionUnit& pu, const ClpRngs &clpRngs);
  void xProcessDMVR           (PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );
#if JVET_AA0061_IBC_MBVD
  void sortIbcMergeMbvdCandidates        (PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t * ibcMbvdLUT, uint32_t * ibcMbvdValidNum, int ibcMbvdIdx= -1);
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  void sortIbcAdaptiveMergeMbvdCandidates(PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t * ibcMbvdLUT, uint32_t * ibcMbvdValidNum, int ibcMbvdIdx= -1);
#endif
#endif
#if JVET_AA0061_IBC_MBVD || (JVET_W0090_ARMC_TM && JVET_Y0058_IBC_LIST_MODIFY)
  bool xAMLIBCGetCurBlkTemplate(PredictionUnit& pu, int nCurBlkWidth, int nCurBlkHeight);
#if JVET_AC0112_IBC_LIC
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
#if JVET_AI0082_GPM_WITH_INTER_IBC
  void getIBCAMLRefTemplate(PredictionUnit &pu, int nCurBlkWidth, int nCurBlkHeight, bool doIbcLic = false, bool checkTmlBvValidaion = true, Pel* pcBufPredRefTop = nullptr, Pel* pcBufPredRefLeft = nullptr);
#else
  void getIBCAMLRefTemplate(PredictionUnit &pu, int nCurBlkWidth, int nCurBlkHeight, bool doIbcLic = false, bool checkTmlBvValidaion = true);
#endif
#else
  void getIBCAMLRefTemplate(PredictionUnit &pu, int nCurBlkWidth, int nCurBlkHeight, bool doIbcLic = false);
#endif
#else
  void getIBCAMLRefTemplate(PredictionUnit &pu, int nCurBlkWidth, int nCurBlkHeight);
#endif

#endif

#if JVET_AC0104_IBC_BVD_PREDICTION || JVET_AD0140_MVD_PREDICTION
  static constexpr bool checkBitMatch(unsigned int value1, unsigned int value2, int bitpos)
  {
    return ((value1 >> bitpos) & 1) == ((value2 >> bitpos) & 1);
  };

  void deriveBvdSignIBC(const Mv& cMvPred, const Mv& cMvdKnownAtDecoder, PredictionUnit& pu, std::vector<Mv>& cMvdDerived, int imv );
  void initOffsets     (Mv& cMvdInput, std::vector<Mv>& cMvdDerived,       MvdSuffixInfo& si, int imv);
  void applyOffsets    (Mv& cMvdInput, std::vector<Mv>& cMvdDerived, const MvdSuffixInfo& si, int imv);
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_Z0054_BLK_REF_PIC_REORDER
  void deriveMVDcand(const PredictionUnit& pu, RefPicList eRefPicList, std::vector<Mv>& cMvdCandList);
  void deriveMVDcandAffine(const PredictionUnit& pu, RefPicList eRefPicList, std::vector<Mv> cMvdCandList[3]);
#endif
#if JVET_AD0140_MVD_PREDICTION
  void deriveMVDcandAffineWithSuffixBins( const PredictionUnit& pu, RefPicList eRefPicList
                                        , std::vector<Mv>& cMvdDerived, std::vector<Mv>& cMvdDerived2, std::vector<Mv>& cMvdDerived3);
  void deriveMVDcandAffineWithSuffixBins( const PredictionUnit& pu, RefPicList eRefPicList
                                        , std::vector<Mv> cMvdDerived[3]) 
  {
    deriveMVDcandAffineWithSuffixBins(pu, eRefPicList, cMvdDerived[0], cMvdDerived[1], cMvdDerived[2]);
  }

  void deriveMVDcandAffineSingleMv(const PredictionUnit& pu, RefPicList eRefPicList, int affineIdx, std::vector<Mv>& cMvdCandList);
  uint16_t deriveMVDcandTrans     (const PredictionUnit& pu, RefPicList eRefList, std::vector<Mv>& cMvdCandList);
#endif
  void deriveMvdSign              (const Mv& cMvPred, const Mv& cMvdKnownAtDecoder, PredictionUnit& pu, RefPicList eRefPicList, int iRefIdx, std::vector<Mv>& cMvdDerived);
#if JVET_AD0140_MVD_PREDICTION
  void initOffsetsMvd             (Mv& cMvdInput, std::vector<Mv>& cMvdDerived, MvdSuffixInfo& si, int imv);
  void defineSignHypMatch         (const Mv &cMvdInput, MvdSuffixInfo &si, const int mvsdIdx);
  void defineSignHypMatchAffine   (PredictionUnit &pu, const RefPicList eRefPicList);
  void applyOffsetsMvd            (Mv& cMvdInput, std::vector<Mv>& cMvdDerived, const MvdSuffixInfo& si, int imv);

  int deriveMVSDIdxFromMVDTransSI (const Mv& cMvd, const std::vector<Mv>& cMvdDerived, const MvdSuffixInfo &si);
  Mv  deriveMVDFromMVSDIdxTransSI (int mvsdIdx, const std::vector<Mv> &cMvdDerived, const MvdSuffixInfo &si);
#endif

  int deriveMVSDIdxFromMVDTrans   (Mv cMvd, std::vector<Mv>& cMvdDerived);
  Mv deriveMVDFromMVSDIdxTrans    (int mvsdIdx, std::vector<Mv>& cMvdDerived);
  void deriveMvdSignSMVD          (const Mv& cMvPred, const Mv& cMvPred2, const Mv& cMvdKnownAtDecoder, PredictionUnit& pu, std::vector<Mv>& cMvdDerived);
  void deriveMvdSignAffine        (const Mv& cMvPred, const Mv& cMvPred2, const Mv& cMvPred3,
#if JVET_Z0054_BLK_REF_PIC_REORDER
                           const Mv cMvdKnownAtDecoder[3],
#else
                           const Mv& cMvdKnownAtDecoder, const Mv& cMvdKnownAtDecoder2, const Mv& cMvdKnownAtDecoder3,
#endif
                           PredictionUnit& pu, RefPicList eRefList, int refIdx, std::vector<Mv>& cMvdDerived, std::vector<Mv>& cMvdDerived2, std::vector<Mv>& cMvdDerived3);
#if JVET_AA0146_WRAP_AROUND_FIX
#if JVET_AD0213_LIC_IMP
  void xGetSublkTemplateCost       (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight,
    const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool wrapRef = false);
#else
  Distortion xGetSublkTemplateCost (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight,
    const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool wrapRef = false);
#endif
#else
#if JVET_AD0213_LIC_IMP
  void xGetSublkTemplateCost       (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight,
    const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate);
#else
  Distortion xGetSublkTemplateCost (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight,
    const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate);
#endif
#endif
#if JVET_AD0140_MVD_PREDICTION
  void initOffsetsAffineMvd        (PredictionUnit &pu, RefPicList eRefList, const std::vector<Mv> (&cMvdDerived)[3]);
  void applyOffsetsAffineMvd       (const PredictionUnit& pu, RefPicList eRefList, std::vector<Mv>& cMvdInput, const  std::vector<Mv>(&cMvdDerived)[3]);

  int deriveMVSDIdxFromMVDAffineSI            (PredictionUnit& pu, RefPicList eRefList, std::vector<Mv>& cMvdDerived, std::vector<Mv>& cMvdDerived2, std::vector<Mv>& cMvdDerived3);
  std::vector<Mv> deriveMVDFromMVSDIdxAffineSI(PredictionUnit& pu, RefPicList eRefList, const std::vector<Mv>& cMvdDerived, const std::vector<Mv>& cMvdDerived2, const std::vector<Mv>& cMvdDerived3, std::vector<Mv>& cMvd);
#endif
  int deriveMVSDIdxFromMVDAffine (PredictionUnit& pu, RefPicList eRefList, std::vector<Mv>& cMvdDerived, std::vector<Mv>& cMvdDerived2, std::vector<Mv>& cMvdDerived3);
  void deriveMVDFromMVSDIdxAffine(PredictionUnit& pu, RefPicList eRefList, std::vector<Mv>& cMvdDerived, std::vector<Mv>& cMvdDerived2, std::vector<Mv>& cMvdDerived3);
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  int deriveMVSDIdxFromMVDTransIBC(const Mv& cMvd, const std::vector<Mv>& cMvdDerived, const MvdSuffixInfo& si) const;
  Mv  deriveMVDFromMVSDIdxTransIBC(int mvsdIdx, const std::vector<Mv>& cMvdDerived, const MvdSuffixInfo& si) const;
#endif
#if JVET_AD0140_MVD_PREDICTION
  static int selectMvdCodingList  (const PredictionUnit& pu, const int numCandL0 = -1);
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void    cacheAssign             ( CacheModel *cache );
#endif
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  static bool isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType );
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  void    sortInterMergeMMVDCandidates(PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t * mmvdLUT, int16_t MMVDIdx = -1);
#else
  void    sortInterMergeMMVDCandidates(PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t * mmvdLUT, uint32_t MMVDIdx = -1);
#endif
#endif
    
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  void    sortAffineMergeCandidates(PredictionUnit pu, AffineMergeCtx& affMrgCtx, uint32_t * affMmvdLUT, int16_t afMMVDIdx = -1, bool fromStart = false);
#else
  void    sortAffineMergeCandidates(PredictionUnit pu, AffineMergeCtx& affMrgCtx, uint32_t * affMmvdLUT, uint32_t afMMVDIdx = -1);
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  void    getBlkAMLRefTemplateSubTMVP                    (PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft);
  static bool xCheckIdenticalMotionSubTMVP               (const PredictionUnit& pu);
  void    adjustMergeCandidatesInOneCandidateGroupSubTMVP(PredictionUnit &pu, MergeCtx& smvpMergeCandCtx, int numRetrievedMergeCand, int mrgCandIdx = -1);
#endif 
#if JVET_W0090_ARMC_TM
  void    adjustInterMergeCandidates(PredictionUnit &pu, MergeCtx& mrgCtx, int mrgCandIdx = -1);
#endif
#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_Z0061_TM_OBMC || JVET_AA0061_IBC_MBVD || JVET_Y0058_IBC_LIST_MODIFY
  bool    xAMLGetCurBlkTemplate     (PredictionUnit& pu, int nCurBlkWidth, int nCurBlkHeight);
  bool    xAMLIsTopTempAvailable    (PredictionUnit& pu);
  bool    xAMLIsLeftTempAvailable   (PredictionUnit& pu); 
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  void xAMLGetSgpmInterBlkTemplate(int nCurBlkWidth, int nCurBlkHeight,int iTempWeidth, int iTempHeight, PelBuf& recBuf);
#endif 
#endif
#if JVET_Z0061_TM_OBMC
  void xOBMCWeightedAverageY    (const PredictionUnit &pu, const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1,
                                 PelUnitBuf &pcYuvDst, const BitDepths &clipBitDepths, const ClpRngs &clpRngs, MotionInfo currMi);
  int  selectOBMCmode           (PredictionUnit &curPu, PredictionUnit &neigPu, bool isAbove, int nLength, uint32_t minCUW, Position curOffset);
  void getBlkOBMCRefTemplate    (PredictionUnit &subblockPu, PelUnitBuf &pcBufPredRef, bool isAbove, MotionInfo currMi);
  bool xCheckIdenticalMotionOBMC(PredictionUnit &pu, MotionInfo tryMi);
  void xSubblockOBMCCopy        (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc, int iDir);
  void xSubblockTMOBMC          (const ComponentID eComp, PredictionUnit &pu, PelUnitBuf &pcYuvPredDst, PelUnitBuf &pcYuvPredSrc,
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
    PelUnitBuf &pcYuvBefore,
#endif
#if JVET_AK0076_EXTENDED_OBMC_IBC
    std::vector<Pel>* lmcsLut,
#endif
    int iDir, int iOBMCmode = 0);
#endif
#if JVET_W0090_ARMC_TM || JVET_AA0070_RRIBC
  void    updateCandList        (uint32_t uiCand, Distortion uiCost, uint32_t uiMrgCandNum, uint32_t* rdCandList, Distortion* candCostList);
#endif
#if JVET_W0090_ARMC_TM
  void    updateCandInfo        (MergeCtx& mrgCtx, uint32_t(*RdCandList)[MRG_MAX_NUM_CANDS], int mrgCandIdx = -1);
#endif
#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  void    getBlkAMLRefTemplate  (PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft, int8_t posList0 = -1, int8_t posList1 = -1, bool load0 = false, bool load1 = false);
#else
  void    getBlkAMLRefTemplate  (PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft);
#endif
#if JVET_AD0213_LIC_IMP
  void    getBlkAMLRefTemplateAlt(PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft, int8_t posList0 = -1, int8_t posList1 = -1, bool load0 = false, bool load1 = false);
#endif
#endif

#if JVET_AD0140_MVD_PREDICTION
  bool    getBlkAMLRefTemplateMvdPredUni(PredictionUnit& pu, PelUnitBuf& pcBufPredRefTop, PelUnitBuf& pcBufPredRefLeft);
  bool    getBlkAMLRefTemplateMvdPred   (PredictionUnit& pu, PelUnitBuf& pcBufPredRefTop, PelUnitBuf& pcBufPredRefLeft);
#endif
#if JVET_W0090_ARMC_TM
  void    adjustAffineMergeCandidates   (PredictionUnit &pu, AffineMergeCtx& affMrgCtx, int mrgCandIdx = -1
#if JVET_Z0139_NA_AFF
    , int sortedCandNum = -1
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    , uint32_t* affCandList = NULL, bool saveTemplate = false
#endif
  );
#if JVET_AG0276_NLIC
  void    adjustAffineMergeCandidates(PredictionUnit &pu, AffineMergeCtx& affMrgCtx, AltLMAffineMergeCtx& altLMAffMrgCtx, AltLMAffineMergeCtx& altLMRMVFMrgCtx);
  void    updateAffineCandInThreeGrp (PredictionUnit &pu, AffineMergeCtx& affMrgCtx, AffineMergeCtx& altLMAffMrgCtx, AffineMergeCtx& altLMAffMrgCtx1, uint32_t* rdCandList, uint32_t* rdCandGrpList, int listsize);
#endif
  void    updateAffineCandInfo(PredictionUnit &pu, AffineMergeCtx& affMrgCtx, 
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if JVET_AI0197_AFFINE_TMVP
    uint32_t (*RdCandList)[AFFINE_MRG_MAX_NUM_CANDS_ALL],
#else
    uint32_t(*RdCandList)[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE],
#endif
#else
    uint32_t(*RdCandList)[AFFINE_MRG_MAX_NUM_CANDS],
#endif
    int mrgCandIdx = -1);
#if JVET_AD0140_MVD_PREDICTION
  template <int iAbove1Left2All3 = 3>
#endif
  void    xGetSublkAMLTemplate(const CodingUnit& cu, const ComponentID compID, const Picture& refPic, 
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
    const Mv& mvAbove,
    const Mv& mvLeft,
#else
    const Mv& mv,
#endif
    const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
     , bool afMMVD = false
#endif
#if JVET_AA0146_WRAP_AROUND_FIX
    , bool wrapRef = false
#endif
#if JVET_AF0190_RPR_TMP_REORDER_LIC
    , const std::pair<int, int>* scalingRatio = NULL
#endif
                               );

#if JVET_AD0140_MVD_PREDICTION
  bool fillAffAMLRefTemplateCache( PredictionUnit& pu, int refList, const int candIdx, const Size& topSize, const Size& leftSize
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                                  , bool isBilinear, AffineMergeCtx affMrgCtx
#endif
                                  );
  bool fillAMLRefTemplateCache   (PredictionUnit& pu, int refList, const int candIdx, const Size& topSize, const Size& leftSize );
#endif


#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
#if JVET_AD0140_MVD_PREDICTION
  template <bool exitIfOob = false>
  bool    getAffAMLRefTemplate(PredictionUnit& pu, PelUnitBuf& pcBufPredRefTop, PelUnitBuf& pcBufPredRefLeft,
#else
  void    getAffAMLRefTemplate(PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft,
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    bool isBilinear, AffineMergeCtx affMrgCtx,
#endif
    int8_t posList0 = -1, int8_t posList1 = -1, bool loadSave0 = false, bool loadSave1 = false);
#else
  void    getAffAMLRefTemplate(PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft);
#endif

#if JVET_AG0164_AFFINE_GPM
  bool getAffAMLRefTemplateImp          (PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                            ,bool isBilinear, AffineMergeCtx affMrgCtx
#endif
);
#endif
#if JVET_AH0119_SUBBLOCK_TM 
  void getAffAndSbtmvpRefTemplate       (PredictionUnit& pu, PelUnitBuf& pcBufPredRefTop, PelUnitBuf& pcBufPredRefLeft,bool isBilinear, AffineMergeCtx affMrgCtx, int interpolationIdx = 2, bool isStore = false, Mv mvOffset = Mv(0, 0), int targetList = 0);
#endif
#if JVET_AD0140_MVD_PREDICTION
  template <int iAbove1Left2All3 = 3>
  bool    getAffAMLRefTemplateMvdPredUni(PredictionUnit& pu, PelUnitBuf& pcBufPredRefTop, PelUnitBuf& pcBufPredRefLeft,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                                         bool isBilinear, AffineMergeCtx affMrgCtx
#endif                               
                                        );
#endif
#if JVET_AD0213_LIC_IMP
  void    getAffAMLRefTemplateAlt       (PredictionUnit &pu, PelUnitBuf &pcBufPredRefTop, PelUnitBuf &pcBufPredRefLeft,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    bool isBilinear, AffineMergeCtx affMrgCtx,
#endif
    int8_t posList0 = -1, int8_t posList1 = -1, bool loadSave0 = false, bool loadSave1 = false);
#endif
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
#if JVET_AN0236_GENERATED_MERGE_CANDIDATES 
  void generateMergeCandidates          (PredictionUnit& pu, MergeCtx& mvpMergeCandCtx, int numRetrievedMergeCand, bool isOppositeLIC = false);
#endif
  void adjustMergeCandidates            (PredictionUnit& pu, MergeCtx& smvpMergeCandCtx, int numRetrievedMergeCand);
#endif
#if JVET_AG0276_NLIC || JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void adjustMergeCandidates            (PredictionUnit& pu, MergeCtx& mvpMergeCandCtx
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                           , AltLMMergeCtx* pMrgCtxAlt
                           , AltLMMergeCtx* pMrgCtxInherit
#else
                          , AltLMMergeCtx& altLMMrgCtx
#endif
                          , int numRetrievedMergeCand);
#endif
#if JVET_AB0079_TM_BCW_MRG
  void adjustMergeCandidatesBcwIdx  (PredictionUnit& pu, MergeCtx& mrgCtx, const int mergeIdx = -1);
#endif
#if JVET_AF0128_LIC_MERGE_TM
  void adjustMergeCandidatesLicFlag (PredictionUnit& pu, MergeCtx& mrgCtx, const int mergeIdx = -1);
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  Distortion  deriveBcwBlending     ( PredictionUnit& pu, bool bUniDir[2] );
  Distortion  deriveBcwBlendingBiDir( PredictionUnit& pu, MvField mvfld0[2], MvField mvfld1[2] 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                    , bool isLicA, bool isLicB
                                    , int scaleA[2], int scaleB[2]
                                    , int offsetA[2], int offsetB[2]
#endif
  );
#if JVET_AK0101_REGRESSION_GPM_INTRA
  Distortion deriveBcwBlendingBiDirIntra(PredictionUnit& pu, IntraPrediction* pcIntraPred, MvField mvfld[2]
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    , bool isLic, int scale[2], int offset[2]
#endif
    , bool isIntra[2], uint8_t intraMode
  );
#endif
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AI0187_TMVP_FOR_CMVP
  void    adjustMergeCandidatesInOneCandidateGroup(PredictionUnit &pu, MergeCtx& mvpMergeCandCtx, int numRetrievedMergeCand, int mrgCandIdx = -1, bool ReduceCandsForSimilarTMCost = false);
#else
  void    adjustMergeCandidatesInOneCandidateGroup(PredictionUnit &pu, MergeCtx& smvpMergeCandCtx, int numRetrievedMergeCand, int mrgCandIdx = -1);
#endif
  void    updateCandInOneCandidateGroup(MergeCtx& mrgCtx, uint32_t* RdCandList, int numCandInCategory = -1);
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  void    updateCandInTwoCandidateGroups(MergeCtx& mrgCtx, uint32_t* rdCandList, int numCandInCategory, MergeCtx mrgCtx2);
#endif
#endif
#if JVET_AG0276_NLIC || JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void    updateCandList(uint32_t uiCand, uint32_t uiCandGrp, Distortion uiCost, uint32_t uiMrgCandNum, uint32_t* rdCandList, uint32_t* rdCandGrpList, Distortion* candCostList);
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void    updateCandInMultiCandidateGroups(uint32_t* rdCandList, uint32_t* rdCandGrpList, int numCandInCategory, MergeCtx& mrgCtx, const MergeCtx** mrgCtx2toN = nullptr, int N = 1);
#else
  void    updateCandInThreeCandidateGroups(MergeCtx& mrgCtx, MergeCtx mrgCtx2, MergeCtx mrgCtx3, uint32_t* rdCandList, uint32_t* rdCandGrpList, int numCandInCategory);
#endif
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  void    adjustMergeCandidatesInOneCandidateGroup(PredictionUnit &pu, MergeCtx& smvpMergeCandCtx, bool* applyBDMVR, Mv** mvBufBDMVR, Mv** mvBufBDMVRTmp, int numRetrievedMergeCand, bool subRefineList[][2] = NULL, bool subRefineListTmp[][2] = NULL, int mrgCandIdx = -1);
  void    updateCandInOneCandidateGroup           (MergeCtx& mrgCtx, uint32_t* rdCandList, bool* applyBDMVR, Mv** mvBufBDMVR, Mv** mvBufBDMVRTmp, bool subRefineList[][2] = NULL, bool subRefineListTmp[][2] = NULL, int numCandInCategory = -1);
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  void    adjustAffineMergeCandidatesOneGroup(PredictionUnit &pu, AffineMergeCtx& affMrgCtx, int listsize
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    , AffineMergeCtx& affOriMrgCtx
#endif
    , int mrgCandIdx = -1);
  void    updateAffineCandInfo2    (PredictionUnit &pu, AffineMergeCtx& affMrgCtx, uint32_t(*rdCandList)[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE], int listsize, int mrgCandIdx = -1);
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  void adjustAffineAMVPCandidates  (PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo, int extCond);
  void tmRefineAffineAMVPCandidates(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo, int extCond);
#endif
#if JVET_Y0058_IBC_LIST_MODIFY
  void    adjustIBCMergeCandidates (PredictionUnit &pu, MergeCtx& mrgCtx, int mrgCandIdx = -1);
  void    updateIBCCandInfo        (PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t(*RdCandList)[IBC_MRG_MAX_NUM_CANDS], int mrgCandIdx = -1);
#endif
#if JVET_Z0075_IBC_HMVP_ENLARGE
  void    adjustIBCMergeCandidates (PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t startPos,uint32_t endPos);
#endif
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  Distortion getTempCost           (const PredictionUnit &pu, const PelBuf &org, const PelBuf &cur);
#endif
#if JVET_AC0112_IBC_GPM
  void    motionCompensationIbcGpm (CodingUnit &cu, MergeCtx &ibcGpmMrgCtx, IntraPrediction* pcIntraPred);
#if JVET_AA0070_RRIBC
  void    adjustIbcMergeRribcCand  (PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t startPos, uint32_t endPos, bool *isSkipThisCand = NULL);
#endif
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE || JVET_AA0070_RRIBC
  void    updateIBCCandInfo        (PredictionUnit &pu, MergeCtx& mrgCtx, uint32_t* RdCandList, uint32_t startPos,uint32_t endPos);
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  template <uint8_t partIdx, bool useDefaultPelBuffer = true>
  void    fillPartGPMRefTemplate   (PredictionUnit &pu, Pel* bufTop = nullptr, Pel* bufLeft = nullptr)
  {
    if (useDefaultPelBuffer)
    {
      bufTop  = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[0] : m_acYuvRefAMLTemplatePart1[0];
      bufLeft = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[1] : m_acYuvRefAMLTemplatePart1[1];
    }
    PelUnitBuf pcBufPredRefTop  = (PelUnitBuf(pu.chromaFormat, PelBuf(bufTop,  pu.lwidth(),          GEO_MODE_SEL_TM_SIZE)));
    PelUnitBuf pcBufPredRefLeft = (PelUnitBuf(pu.chromaFormat, PelBuf(bufLeft, GEO_MODE_SEL_TM_SIZE, pu.lheight()        )));

#if JVET_AG0164_AFFINE_GPM
    if (pu.cu->affine)
    {
      CHECK(pu.mergeType == MRG_TYPE_SUBPU_ATMVP, "Invalid merge type");
      getAffAMLRefTemplateImp(pu, pcBufPredRefTop, pcBufPredRefLeft, false, AffineMergeCtx());
      return;
    }
#endif
    getBlkAMLRefTemplate(pu, pcBufPredRefTop, pcBufPredRefLeft);
  }

  template <uint8_t partIdx, bool useDefaultPelBuffer = true>

#if JVET_AG0164_AFFINE_GPM
  void    fillPartGPMRefTemplate(PredictionUnit &pu, MergeCtx& geoMrgCtx, int candIdx, int geoMmvdIdx = -1, Pel* bufTop = nullptr, Pel* bufLeft = nullptr, const AffineMergeCtx* affGeoMrgCtx = nullptr)
#else
  void    fillPartGPMRefTemplate(PredictionUnit &pu, MergeCtx& geoMrgCtx, int candIdx, int geoMmvdIdx = -1, Pel* bufTop = nullptr, Pel* bufLeft = nullptr)
#endif
  {
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
    if (candIdx >= GEO_MAX_ALL_INTER_UNI_CANDS)
#else
    if (candIdx >= GEO_MAX_NUM_UNI_CANDS)
#endif
    {
      return;
    }
#endif
#if JVET_AG0164_AFFINE_GPM
    if (pu.affineGPM[partIdx])
    {
      CHECK(affGeoMrgCtx == nullptr, "Invalid affine merge ctx!");
      affGeoMrgCtx->setAffMergeInfo( pu, candIdx, geoMmvdIdx);
    }
    else
#endif
#if JVET_W0097_GPM_MMVD_TM
    if (geoMmvdIdx >= 0)
    {
      geoMrgCtx.setGeoMmvdMergeInfo(pu, candIdx, geoMmvdIdx);
    }
    else
#endif
    {
      geoMrgCtx.setMergeInfo(pu, candIdx);
    }
    
    fillPartGPMRefTemplate<partIdx, useDefaultPelBuffer>(pu, bufTop, bufLeft);
#if JVET_AG0164_AFFINE_GPM
    pu.cu->affine = false;
#endif
  }
#if JVET_AI0082_GPM_WITH_INTER_IBC
  template <uint8_t partIdx, bool useDefaultPelBuffer = true>
  void    fillPartGpmInterIbcRefTemplate(PredictionUnit &pu, std::vector<Pel>* lut, Pel* bufTop = nullptr, Pel* bufLeft = nullptr)
  {
    if (useDefaultPelBuffer)
    {
      bufTop  = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[0] : m_acYuvRefAMLTemplatePart1[0];
      bufLeft = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[1] : m_acYuvRefAMLTemplatePart1[1];
    }
    getIBCAMLRefTemplate(pu, pu.lumaSize().width, pu.lumaSize().height, false, true, bufTop, bufLeft);
    if (lut != nullptr)
    {
      if (m_bAMLTemplateAvailabe[0])
      {
        for (int w = 0; w < pu.lwidth(); ++w)
        {
          bufTop[w] = (*lut)[bufTop[w]];
        }
      }

      if (m_bAMLTemplateAvailabe[1])
      {
        for (int h = 0; h < pu.lheight(); ++h)
        {
          bufLeft[h] = (*lut)[bufLeft[h]];
        }
      }
    }
  }

  template <uint8_t partIdx, bool useDefaultPelBuffer = true>
  void    fillPartGpmInterIbcRefTemplate(PredictionUnit &pu, std::vector<Pel>* lut, Mv* geoBvList, int candIdx, int geoMmvdIdx = -1, Pel* bufTop = nullptr, Pel* bufLeft = nullptr)
  {
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
    if (candIdx < GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)
#else
    if (candIdx < GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)
#endif
#else
    if (candIdx < GEO_MAX_NUM_UNI_CANDS)
#endif
    {
      return;
    }
    pu.cu->predMode = MODE_IBC;
#if JVET_AG0164_AFFINE_GPM
    pu.mv[REF_PIC_LIST_0] = geoBvList[candIdx - (GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)];
#else
    pu.mv[REF_PIC_LIST_0] = geoBvList[candIdx - (GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)];
#endif
    pu.bv = pu.mv[REF_PIC_LIST_0];
    pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    fillPartGpmInterIbcRefTemplate<partIdx, useDefaultPelBuffer>(pu, lut, bufTop, bufLeft);
    pu.cu->predMode = MODE_INTER;
  }
#endif
#endif
#if INTER_LIC || JVET_AC0112_IBC_LIC
#if JVET_AE0078_IBC_LIC_EXTENSION
#if JVET_AG0276_LIC_SLOPE_ADJUST
  void xGetLICParamGeneral (const CodingUnit& cu, const ComponentID compID, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, int& shift, int& scale, int& offset, int* shift2 = nullptr, int* scale2 = nullptr, int* offset2 = nullptr, int* mean = nullptr, int *midVal = nullptr);
#else
  void xGetLICParamGeneral (const CodingUnit& cu, const ComponentID compID, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, int& shift, int& scale, int& offset, int* shift2 = nullptr, int* scale2 = nullptr, int* offset2 = nullptr, int* mean = nullptr);
#endif
#else
#if JVET_AG0276_LIC_SLOPE_ADJUST
  void xGetLICParamGeneral (const CodingUnit& cu, const ComponentID compID, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, int& shift, int& scale, int& offset, int *midVal = nullptr);
#else
  void xGetLICParamGeneral (const CodingUnit& cu, const ComponentID compID, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, int& shift, int& scale, int& offset);
#endif
#endif
#endif
#if JVET_AE0159_FIBC
  void xGetIbcFilterRefBuf   (PelBuf& piPred, CodingUnit* pcCU, const ComponentID compID, const Mv& mv, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
  void xCalIbcFilterParam    (PelBuf& piPred, CodingUnit* pcCU, const ComponentID compID, const Mv& mv, unsigned int uiBlkWidth, unsigned int uiBlkHeight ); 
  void xGenerateIbcFilterPred(PelBuf& piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight, const ComponentID compID, CodingUnit* pcCU);
#endif
#if JVET_AE0159_FIBC || JVET_AE0059_INTER_CCCM || JVET_AE0078_IBC_LIC_EXTENSION || JVET_AF0073_INTER_CCP_MERGE
  void setIntraPrediction    ( IntraPrediction* intra );
#endif
#if INTER_LIC
#if JVET_AA0146_WRAP_AROUND_FIX
#if JVET_AF0190_RPR_TMP_REORDER_LIC
  void xGetSublkTemplate   (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool wrapRef = false, const std::pair<int, int>* scalingRatio = NULL);
  void xLocalIlluComp      (const PredictionUnit& pu, const ComponentID compID, const Picture& refPic, const Mv& mv, const bool biPred, PelBuf& dstBuf, bool wrapRef = false, const std::pair<int, int>* scalingRatio = NULL);
#else
  void xGetSublkTemplate   (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool wrapRef = false);
  void xLocalIlluComp      (const PredictionUnit& pu, const ComponentID compID, const Picture& refPic, const Mv& mv, const bool biPred, PelBuf& dstBuf, bool wrapRef = false);
#endif
#else
  void xGetSublkTemplate   (const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate);
  void xLocalIlluComp      (const PredictionUnit& pu, const ComponentID compID, const Picture& refPic, const Mv& mv, const bool biPred, PelBuf& dstBuf);
#endif
#if JVET_AD0213_LIC_IMP
#if JVET_AF0190_RPR_TMP_REORDER_LIC
  void xGetSublkTemplateAndRef(const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool recSample, bool refSample, const std::pair<int, int>* scalingRatio = NULL);
#else
  void xGetSublkTemplateAndRef(const CodingUnit& cu, const ComponentID compID, const Picture& refPic, const Mv& mv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate, bool recSample, bool refSample);
#endif
  void xLicRemHighFreq        (const CodingUnit& cu, int compID, int licIdx);
  void setLicParam            (int refList, int compID, int& licScale, int& licOffset) { licScale = m_scale[refList][compID]; licOffset = m_offset[refList][compID]; }
#if JVET_AM0157_SGPM_BV_LIC
  void setMMLicParam          (int refList, int compID, int& licScale, int& licShift,int& licOffset, int& licScale2, int& licShift2, int& licOffset2, int& mean)
  {
    licScale = m_ibcLicScale[refList][0][compID]; licShift = m_ibcLicShift[refList][0][compID]; licOffset = m_ibcLicOffset[refList][0][compID];
    licScale2 = m_ibcLicScale[refList][1][compID]; licShift2 = m_ibcLicShift[refList][1][compID]; licOffset2 = m_ibcLicOffset[refList][1][compID];
    mean = m_ibcLicMean[refList][compID];
  }
#endif
  void resetFillLicTpl        () { m_fillLicTpl[COMPONENT_Y] = m_fillLicTpl[COMPONENT_Cb] = m_fillLicTpl[COMPONENT_Cr] = false; }
  void xLicCompAdj            (const PredictionUnit& pu, PelUnitBuf& pcYuvPred, const bool lumaOnly, const bool chromaOnly);
#if JVET_AG0276_LIC_BDOF_BDMVR
  void xLicCompAdjBdof        (const PredictionUnit& pu, PelUnitBuf& pcYuvPred, const bool lumaOnly, const bool chromaOnly);
#endif
#endif
  template <bool trueAfalseL>
  void xGetPredBlkTpl         (const CodingUnit& cu, const ComponentID compID, const CPelBuf& refBuf, const Mv& mv, const int posW, const int posH, const int tplSize, Pel* predBlkTpl
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      , bool AML = false
#endif
#if JVET_AF0190_RPR_TMP_REORDER_LIC
                      , const Picture*             refPic       = NULL
                      , const std::pair<int, int>* scalingRatio = NULL
#endif
                      );
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  void xUpdateLicModel         (int &scale, int &offset, int &shift, int midVal, int delta);
#endif

#if JVET_AC0112_IBC_LIC
  void xGetSublkTemplate       (const CodingUnit& cu, const ComponentID compID, const Mv& bv, const int sublkWidth, const int sublkHeight, const int posW, const int posH, int* numTemplate, Pel* refLeftTemplate, Pel* refAboveTemplate, Pel* recLeftTemplate, Pel* recAboveTemplate
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                , const int requiredTemplate
#endif
                                );
  void xLocalIlluComp          (const PredictionUnit& pu, const ComponentID compID, const Mv& bv, PelBuf& dstBuf
#if JVET_AG0136_INTRA_TMP_LIC
                       , const bool isLinearTransformDone
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                , const int requiredTemplate
#endif
                       );
  template <bool trueAfalseL>
  void xGetIbcLicPredBlkTpl     (const CodingUnit& cu, const ComponentID compID, const CPelBuf& refBuf, const Mv& mv, const int posW, const int posH, const int tplSize, Pel* predBlkTpl);
#endif

#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  void       deriveSubTmvpTMMv    (PredictionUnit& pu);
  void       deriveSubTmvpTMMv2Pel(PredictionUnit& pu, int step);
  Distortion deriveTMMv2Pel       (const PredictionUnit& pu, int step, bool fillCurTpl, Distortion curBestCost, RefPicList eRefList, int refIdx, int maxSearchRounds, Mv& mv, const MvField* otherMvf = nullptr);
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  void       deriveTMMv         (PredictionUnit& pu, Distortion* tmCost = NULL, int mergeIdx = -1);
#else
  void       deriveTMMv         (PredictionUnit& pu, Distortion* tmCost = NULL);
#endif
#else
  void       deriveTMMv         (PredictionUnit& pu);
#endif
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  Distortion deriveTMMv         (const PredictionUnit& pu, bool fillCurTpl, Distortion curBestCost, RefPicList eRefList, int refIdx, int maxSearchRounds, Mv& mv, const MvField* otherMvf = nullptr, int mergeIdx = -1);
#else
  Distortion deriveTMMv         (const PredictionUnit& pu, bool fillCurTpl, Distortion curBestCost, RefPicList eRefList, int refIdx, int maxSearchRounds, Mv& mv, const MvField* otherMvf = nullptr);
#endif
#if JVET_AD0213_LIC_IMP
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  template <int tplSize>
  void       deriveTemplateLIC  (TplMatchingCtrl& tplCtrl, RefPicList eRefList);
#else
  void       deriveTemplateLIC  (TplMatchingCtrl& tplCtrl, RefPicList eRefList);
#endif
#endif
#endif // TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
#if TM_AMVP
  void       clearTplAmvpBuffer ();
  void       writeTplAmvpBuffer (const AMVPInfo& src, const CodingUnit& cu, RefPicList eRefList, int refIdx);
  bool       readTplAmvpBuffer  (      AMVPInfo& dst, const CodingUnit& cu, RefPicList eRefList, int refIdx);
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void       writeMergeBuffer   (const MergeCtx& srcList0, const MergeCtx& srcList1, const CodingUnit& cu);
  bool       readMergeBuffer    (      MergeCtx& dstList0,       MergeCtx& dstList1, const CodingUnit& cu);
  void       clearAmvpTmvpBuffer();
#endif
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  void       clearAffineAmvpBuffer ();
  void       writeAffineAmvpBuffer (const AffineAMVPInfo& src, const CodingUnit& cu, RefPicList eRefList, int refIdx);
  bool       readAffineAmvpBuffer  (      AffineAMVPInfo& dst, const CodingUnit& cu, RefPicList eRefList, int refIdx);
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  static Distortion getDecoderSideDerivedMvCost (const Mv& mvStart, const Mv& mvCur, int searchRangeInFullPel, int weight);
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  bool refineTmvpCompareTwoMotionInfo(MotionInfo& srcMotionInfo, const MotionInfo& dstMotionInfo, RefPicList targetRefList, int scaleFactor);
  void refineTmvpCoverSearchForRefinedMv(bool refineTmvpIsSbTmvp, const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv,
      const int &refIdx, bool sbFlag, int col, int* targetRefIdx, int sbTmvpType, RefPicList refineTmvpColRefPicList, int colRefPOC, const MotionInfo& refineTmvpMiAddr);
  void refineTmvpSearchForRefinedMv(bool refineTmvpIsSbTmvp, const PredictionUnit& pu,
      const int srcRefList, const int srcRefIdx, const int dstRefList, const int dstRefIdx,
      Mv& dstMv, int horMin, int horMax, int verMin, int verMax);
#endif
#if MULTI_PASS_DMVR
  void       xBDMVRUpdateSquareSearchCostLog    (Distortion* costLog, int bestDirect);
#endif
#endif
#if MULTI_PASS_DMVR
private:
  void       xBDMVRFillBlkPredPelBuffer        (const PredictionUnit& pu, const Picture& refPic, const Mv &_mv, PelUnitBuf &dstBuf, const ClpRng& clpRng);

#if JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION
  void      xBDMVRFillBlkPredPelBufferAffine   (const PredictionUnit& pu, const Picture& refPic, const Mv(&_mv)[3], PelUnitBuf& dstUnitBuf, const ClpRng& clpRng);
  void      xBDMVRFillBlkPredPelBufferAffineOPT(const PredictionUnit& pu, const Picture& refPic, const RefPicList eRefPicList, const Mv(&_mv)[3], const Mv mvCur, const Mv mvCenter, const bool doInterpolation, PelUnitBuf& dstUnitBuf, const ClpRng& clpRng, const bool profTh,const int blockWidth,const int blockHeight     ,const int memBlockWidthExt,const int memBlockHeight,const int memHeight,const int memStride);
  void      xCalculteAffineParameters          (const PredictionUnit& pu, const Picture& refPic, const Mv(&_mv)[3],int refList, bool& profTH, int& blockWidth, int& blockHeight, int& memBlockWidthExt, int& memBlockHeight, int& memHeight, int& memStride);
#endif
#if JVET_X0049_ADAPT_DMVR
  template <uint8_t dir>
#endif
  void       xBDMVRPreInterpolation    (const PredictionUnit& pu, const Mv (&mvCenter)[2], bool doPreInterpolationFP, bool doPreInterpolationHP);
#if JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION
  Distortion xBDMVRGetMatchingErrorAffine(const PredictionUnit& pu, Mv(&mv)[2][3],Mv(&mvOffset)[2],const Mv(&initialMv)[2],bool& doInterpolation,bool hPel,bool useMR, bool useHadmard, const bool(&profTh)[2], const int(&blockWidth)[2], const int(&blockHeight)[2], const int(&memBlockWidthExt)[2], const int(&memBlockHeight)[2], const int(&memHeight)[2], const int(&memStride)[2]);
#endif
#if JVET_X0049_BDMVR_SW_OPT
  Distortion xBDMVRGetMatchingError    (const PredictionUnit& pu, const Mv (&mv)[2], bool useMR, bool useHadmard = false );
#if JVET_X0049_ADAPT_DMVR
  template <uint8_t dir>
#endif
#else
  Distortion xBDMVRGetMatchingError    (const PredictionUnit& pu, const Mv (&mv)[2], bool useMR );
#endif
  Distortion xBDMVRGetMatchingError    (const PredictionUnit& pu, const Mv (&mv)[2], const int subPuOffset, bool useHadmard, bool useMR
                                      , bool& doPreInterpolation, int32_t searchStepShift, const Mv (&mvCenter)[2]
                                      , const Mv(&mvInitial)[2]  // only used for full-pel MVD
                                      , int nDirect              // only used for half-pel MVD
                                        );
#if JVET_AF0057
  bool isDMVRmvReliable                (Pel* pelBuffer[2], const int stride, const Mv(&initialMv)[2], int horOffset, int verOffset, int xx, int yy, const int widthInSubPu, int theWidth, int theHeight);
#endif
#if JVET_X0049_BDMVR_SW_OPT
  template <bool adaptRange, bool useHadamard>
  Distortion xBDMVRMvIntPelFullSearch  (Mv&mvOffset, Distortion curBestCost,
    const Mv(&initialMv)[2],
    const int32_t maxSearchRounds,
    const int maxHorOffset, const int maxVerOffset,
    const bool earlySkip,
    const Distortion earlyTerminateTh, DistParam &cDistParam, Pel* pelBuffer[2], const int stride);

  template<bool hPel>
  Distortion xBDMVRMvSquareSearch      (Mv(&curBestMv)[2], Distortion curBestCost, PredictionUnit& pu, const Mv(&initialMv)[2], int32_t maxSearchRounds, int32_t searchStepShift, bool useMR, bool useHadmard);
#if JVET_AB0112_AFFINE_DMVR && !JVET_AC0144_AFFINE_DMVR_REGRESSION
  template<bool hPel>
  Distortion xBDMVRMvSquareSearchAffine(Mv(&curBestMv)[2], Distortion curBestCost, PredictionUnit& pu, const Mv(&initialMv)[2], int32_t maxSearchRounds, int32_t searchStepShift, bool useMR, bool useHadmard);
#endif
#if JVET_X0049_ADAPT_DMVR
  template <uint8_t dir>
  Distortion xBDMVRMvOneTemplateHPelSquareSearch(Mv(&curBestMv)[2], Distortion curBestCost, PredictionUnit& pu,
    const Mv(&initialMv)[2], int32_t maxSearchRounds, int32_t searchStepShift,
    bool useMR, bool useHadmard);
#endif
#else
  Distortion xBDMVRMvIntPelFullSearch  (Mv (&curBestMv)[2], Distortion curBestCost, PredictionUnit& pu, const Mv (&initialMv)[2], int32_t maxSearchRounds, int32_t searchStepShift, bool useMR, const int subPuBufOffset );
  Distortion xBDMVRMvSquareSearch      (Mv(&curBestMv)[2], Distortion curBestCost, PredictionUnit& pu, const Mv(&initialMv)[2], int32_t maxSearchRounds, int32_t searchStepShift, bool useMR, bool useHadmard);
#endif

  Mv*       m_bdmvrSubPuMvBuf[2];

#if JVET_X0083_BM_AMVP_MERGE_MODE
public:
#if JVET_AD0213_LIC_IMP
  void      getAmvpMergeModeMergeList(PredictionUnit& pu, MvField* mvFieldAmListCommon, bool* licAmListCommon, const int decAmvpRefIdx = -1);
  void      amvpMergeModeMvRefinement(PredictionUnit& pu, MvField* mvFieldAmListCommon, bool* licAmListCommon, const int mvFieldMergeIdx, const int mvFieldAmvpIdx);
#else
  void      getAmvpMergeModeMergeList(PredictionUnit& pu, MvField* mvFieldAmListCommon, const int decAmvpRefIdx = -1);
  void      amvpMergeModeMvRefinement(PredictionUnit& pu, MvField* mvFieldAmListCommon, const int mvFieldMergeIdx, const int mvFieldAmvpIdx);
#endif
#endif

#if JVET_AC0144_AFFINE_DMVR_REGRESSION
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  void bmAffineInit               (const PredictionUnit& pu, int mergeIdx = -1);
#else
  void bmAffineInit               (const PredictionUnit &pu);
#endif
  void bmInitAffineSubBlocks      (const Position puPos, const int width, const int height, const int dx, const int dy,
    int mvScaleHor[2], int mvScaleVer[2], int deltaMvHorX[2], int deltaMvHorY[2], int deltaMvVerX[2], int deltaMvVerY[2]);
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool processBDMVR4AdaptiveAffine(PredictionUnit& pu, Mv(&mvAffiL0)[2][3], Mv(&mvAffiL1)[2][3], EAffineModel& affTypeL0, EAffineModel& affTypeL1, int mergeIdx = -1);
#else
  bool processBDMVR4AdaptiveAffine(PredictionUnit& pu, Mv(&mvAffiL0)[2][3], Mv(&mvAffiL1)[2][3], EAffineModel& affTypeL0, EAffineModel& affTypeL1);
#endif
  void xDeriveCPMV                (PredictionUnit &pu, const Mv(&curShiftMv)[2][3], int deltaMvHorX, int deltaMvHorY, int deltaMvVerX, int deltaMvVerY, int baseCP, Mv(&cpMV)[2][3]);
  Distortion xBDMVRMv6ParameterSearchAffine(Distortion curBestCost, PredictionUnit& pu);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  void bmAdaptiveAffineIntSearch  (const PredictionUnit& pu, Mv(&mvOffsetL0)[2], Distortion& minCostL0, Mv(&mvOffsetL1)[2], Distortion& minCostL1, int mergeIdx = -1);
#else
  void bmAdaptiveAffineIntSearch  (const PredictionUnit &pu, Mv(&mvOffsetL0)[2], Distortion &minCostL0, Mv(&mvOffsetL1)[2], Distortion &minCostL1);
#endif
  void bmAdaptiveAffineHPelSearch (const PredictionUnit &pu, Mv(&mvOffset)[2], Distortion &minCost, Distortion localCostArray[9], RefPicList refList);
  Distortion xGetBilateralMatchingErrorAdaptiveAffine(const PredictionUnit& pu, Mv(&mvOffset)[2], RefPicList refList, bool skipOtherRef);
  bool bmAdaptiveAffineRegression (PredictionUnit &pu, Distortion &minCost, RefPicList refList);
  Distortion xGetBilateralMatchingErrorAffine(const PredictionUnit& pu, Mv(&mvAffi)[2][3], bool skipInterpolation = false);
  template <bool checkMv>
  Distortion xGetBilateralMatchingErrorAffineCheckMv(const PredictionUnit& pu, Mv(&mvAffi)[2][3]);
#else
  Distortion xGetBilateralMatchingErrorAffine(const PredictionUnit& pu, Mv(&mvAffi)[2][3]);
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  void bmAffineIntSearch     (const PredictionUnit &pu, Mv(&mvOffset)[2], Distortion &minCost, Distortion totalCost[(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1][(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1], int mergeIdx = -1);
#else
  void bmAffineIntSearch     (const PredictionUnit &pu, Mv(&mvOffset)[2], Distortion &minCost, Distortion totalCost[(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1][(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1]);
#endif
  void bmAffineHPelSearch    (const PredictionUnit &pu, Mv(&mvOffset)[2], Distortion &minCost, Distortion localCostArray[9]);

  void xInitBilateralMatching(const int width, const int height, const int bitDepth, const bool useMR, const bool useHadmard);
  Distortion xGetBilateralMatchingErrorAffine(const PredictionUnit& pu, Mv(&mvOffset)[2]);
  bool bmAffineRegression    (PredictionUnit &pu, Distortion &minCost);
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  void bmAffineCpmvSearch    (PredictionUnit &pu, Distortion &minCost);
#endif
#endif
public:
  Mv*       getBdofSubPuMvOffset() {return m_bdofSubPuMvOffset;}
  void      setBdmvrSubPuMvBuf(Mv* mvBuf0, Mv* mvBuf1) { m_bdmvrSubPuMvBuf[0] = mvBuf0; m_bdmvrSubPuMvBuf[1] = mvBuf1; }
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool      processBDMVR              (PredictionUnit& pu, int step = 0, Distortion* tmCost = NULL, int mergeIdx = -1);
#else
  bool      processBDMVR              (PredictionUnit& pu, int step = 0, Distortion* tmCost = NULL);
#endif
#else
  bool      processBDMVR              (PredictionUnit& pu);
#endif
#if JVET_AB0112_AFFINE_DMVR
  bool      processBDMVR4Affine       (PredictionUnit& pu
#if JVET_AH0119_SUBBLOCK_TM
    , AffineMergeCtx &affineMergeCtx, bool doTM
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
      , int mergeIdx = -1
#endif
  );
#endif
#if JVET_AH0119_SUBBLOCK_TM
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool processTM4SbTmvpBaseMV(PredictionUnit& pu, AffineMergeCtx& affineMergeCtx, int uiAffMergeCand, Distortion& uiCostOri, Distortion& uiCostBest, uint32_t targetList, int mergeIdx = -1);
  bool processTM4SbTmvp      (PredictionUnit& pu, AffineMergeCtx& affineMergeCtx, int uiAffMergeCand, bool isEncoder, int mergeIdx = -1);
#else
  bool processTM4SbTmvpBaseMV(PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int uiAffMergeCand, Distortion& uiCostOri, Distortion& uiCostBest, uint32_t targetList);
  bool processTM4SbTmvp      (PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int uiAffMergeCand, bool isEncoder);
#endif
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  bool      processTM4Affine (PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int uiAffMergeCand, bool isEncoder
#if JVET_AH0119_SUBBLOCK_TM
    , bool isTmPara = true
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
    , int mergeIdx = -1
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
    , bool isInt4PosRefine = false
#endif
  );
#if JVET_AH0119_SUBBLOCK_TM
  Distortion xGetTemplateMatchingError(PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int interpolationIdx=2 , bool isStore=false
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
    , int mergeIdx = -1
#endif
  );
  bool processTM4AffineBaseMV(PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int uiAffMergeCand, Mv(&bestCPMV)[2][3], Distortion& uiCostOri, Distortion& uiCostBest, uint32_t targetList
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
    , int mergeIdx = -1
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
    , bool isInt4PosRefine = false
#endif
  );
  void xUpdateCPMV         (PredictionUnit &pu, int32_t targetRefList, const Mv(&curCPMV)[2][3], const int deltaMvHorX, const int deltaMvHorY, const int deltaMvVerX, const int deltaMvVerY, const int baseCP);
  bool processTM4AffinePara(PredictionUnit& pu, AffineMergeCtx &affineMergeCtx, int uiAffMergeCand, int32_t targetRefList, Mv(&cpBestMVF)[2][3], Distortion& uiCostOri, Distortion& uiCostBest
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
    , int mergeIdx = -1
#endif
  );
#endif
#endif
#if JVET_X0049_ADAPT_DMVR
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool      processBDMVRPU2Dir        (PredictionUnit& pu, bool subPURefine[2], Mv(&finalMvDir)[2], int mergeIdx = -1);
  void      processBDMVRSubPU         (PredictionUnit& pu, bool subPURefine, int mergeIdx = -1);
#else
  bool      processBDMVRPU2Dir        (PredictionUnit& pu, bool subPURefine[2], Mv(&finalMvDir)[2]);
  void      processBDMVRSubPU         (PredictionUnit& pu, bool subPURefine);
#endif
#endif
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
private:
  Mv*       m_bdofSubPuMvBuf;
public:
  void      setBdofSubPuMvBuf          (Mv* bdofMvBuf)       { m_bdofSubPuMvBuf   = bdofMvBuf;   }
  Mv*       getBdofSubPuMvBuf          ()                    { return m_bdofSubPuMvBuf;          }
  void      setDoAffineSubPuBdof       (bool doAffineSubPuBdof)  { m_doAffineSubPuBdof = doAffineSubPuBdof; }
  bool      getDoAffineSubPuBdof       ()                        { return m_doAffineSubPuBdof; }
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  void      setAffineBdofChroma        (bool isAffineBdofChroma) { m_isAffBdofChroma = isAffineBdofChroma; }
  bool      getAffineBdofChroma        () { return m_isAffBdofChroma; }
  void      setDeriveOobMask           (bool isDeriveOobMask) { m_isDeriveOobMask = isDeriveOobMask; }
  bool      getDeriveOobMask           () { return m_isDeriveOobMask; }
#endif
  void xFillIBCBuffer                  (CodingUnit &cu);
#if JVET_Z0118_GDR
  void resetCurIBCBuffer               (const ChromaFormat chromaFormatIDC, const Area ctuArea, const int ctuSize, const Pel dirtyPel);  
#endif
  void resetIBCBuffer                  (const ChromaFormat chromaFormatIDC, const int ctuSize);
  void resetVPDUforIBC                 (const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos);

  bool isLumaBvValid                   (const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv);

  bool xPredInterBlkRPR                ( const std::pair<int, int>& scalingRatio, const PPS& pps, const CompArea &blk, const Picture* refPic, const Mv& mv, Pel* dst, const int dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf = false);
#if JVET_Z0118_GDR
  void xPadIBCBuffer                   (const CodingStructure &cs, const UnitArea& ctuArea);
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
private:
  bool m_fillCurTplLeftARMC;
  bool m_fillCurTplAboveARMC;
public:
  void setFillCurTplAboveARMC          (bool b) { m_fillCurTplAboveARMC = b; }
  void setFillCurTplLeftARMC           (bool b) { m_fillCurTplLeftARMC = b; }
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
private:
  bool m_dimdForOBMCFilled;
  int m_modeBuf[2][MAX_CU_SIZE >> MIN_CU_LOG2];
#if JVET_AK0076_EXTENDED_OBMC_IBC && !JVET_AK0212_GPM_OBMC_MODIFICATION
  bool m_intraObmcPred;
#endif
  int m_modeGetCheck[2];
public:
  void setDIMDForOBMC                  (bool b) { m_dimdForOBMCFilled = b; }
  void setModeGetCheck                 (int i, bool b) { m_modeGetCheck[i] = b; }
  void setClearModeBuf                 (int i) { memset(m_modeBuf[i], -1, sizeof(int) * (MAX_CU_SIZE >> MIN_CU_LOG2)); }
#if JVET_AK0076_EXTENDED_OBMC_IBC && !JVET_AK0212_GPM_OBMC_MODIFICATION
  void setIntraObmcPred                (bool b) { m_intraObmcPred = b; }
#endif
#endif

#if JVET_AG0276_NLIC
public:
  void xPredWoRefinement               (PredictionUnit& pu, PelUnitBuf &pcYuvPred, const bool luma = true, const bool chroma = true);
#if JVET_AG0276_LIC_FLAG_SIGNALING
  template <bool isBRcand>
#endif
  void xDevSecLicPara                  (CodingUnit& cu, PelUnitBuf& predBuf, PelUnitBuf& dstBuf);
#endif

#if JVET_AA0096_MC_BOUNDARY_PADDING
  void mcFramePad                      (Picture *pcCurPic, Slice &slice);
#if JVET_Z0118_GDR
  void mcFramePadOneSide               (Picture *pcCurPic, Slice &slice, PadDirection padDir, PelStorage *pPadBuffYUV,
                                        PredictionUnit *blkDataTmp, PelStorage *pPadYUVContainerDyn, const UnitArea blkUnitAreaBuff,
                                        PelStorage *pCurBuffYUV, PictureType pt);
  void mcFramePadRepExt                (Picture *pcCurPic, Slice &slice, PictureType pt);
#else
  void mcFramePadOneSide               (Picture *pcCurPic, Slice &slice, PadDirection padDir, PelStorage *pPadBuffYUV,
                                        PredictionUnit *blkDataTmp, PelStorage *pPadYUVContainerDyn, const UnitArea blkUnitAreaBuff,
                                        PelStorage *pCurBuffYUV);
  void mcFramePadRepExt                (Picture *pcCurPic, Slice &slice);
#endif
#endif
#if JVET_Z0118_GDR && JVET_AD0123_REF_PICTURE_PADDING_FOR_GDR
  void padDirtyArea                    (Picture* pcCurPic, Slice& slice, PictureType pt);
#endif

#if JVET_AE0059_INTER_CCCM
  inline void getNonDownSampledLumaValsOffset( const PredictionUnit* pu, const PelBuf& luma, const int x, const int y, Pel* s, const int offset, const int flipType = 0 );
  inline void getNonDownSampledLumaVals      ( const PredictionUnit* pu, const PelBuf& luma, const int x, const int y, Pel* s, const int flipType = 0 );
  inline int computeOffset                   ( const PelBuf& buf );
#if JVET_AF0073_INTER_CCP_MERGE
  bool deriveInterCccmPrediction             ( TransformUnit* tu, const PelBuf& lumaPrediction, const PelBuf& lumaReconstruction, const PelBuf& inBufCb, const PelBuf& inBufCr, PelBuf& outBufCb, PelBuf& outBufCr );
#else
  bool deriveInterCccmPrediction             ( const TransformUnit* tu, const PelBuf& lumaPrediction, const PelBuf& lumaReconstruction, const PelBuf& inBufCb, const PelBuf& inBufCr, PelBuf& outBufCb, PelBuf& outBufCr );
#endif
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  bool deriveInterCcpMergePrediction         ( TransformUnit* tu, const PelBuf& lumaReconstruction, PelBuf& inBufCb, PelBuf& inBufCr, PelBuf& outBufCb, PelBuf& outBufCr, CCPModelCandidate interCcpMergeList[], int validNum);
#endif
#if JVET_AL0181_ASBT 
private:
  int32_t * m_enerBuffer;
  Pel * m_coordBuffer;
  TCoeff * m_coeffTmp[MAX_NUM_COMPONENT];
  SIGN_PRED_TYPE * m_signPredTypeTmp[MAX_NUM_COMPONENT];
  unsigned * m_signScanIdxTmp[MAX_NUM_COMPONENT];
  uint64_t m_sumMin[MAX_TB_SIZEY];
  uint64_t m_sumMinH[MAX_TB_SIZEY];
public:
  Distortion getDistortionFullBlock          ( TransformUnit& tu, const PelBuf& lumaResi );
  void asbtDownSampling                      ( TransformUnit &tu, ComponentID compID, const PelBuf& lumaPrediction,const PelBuf& lumaResi, PelBuf& downResi );
  void asbtDownSamplingH                     ( TransformUnit &tu, ComponentID compID, const PelBuf& lumaPrediction,const PelBuf& lumaResi, PelBuf& downResi );
  void initEnerPred                          ( TransformUnit& tu, const PelBuf& lumaPrediction );
  void upsampleOneBlock                      ( TransformUnit& tu, PelBuf& image, ComponentID compID );
  void upsampleOneBlockH                     ( TransformUnit& tu, PelBuf& image, ComponentID compID );
  void asbtBorderResi                        ( const TransformUnit& tu, const CPelBuf& recoBuf, const CPelBuf& predBuf, const ComponentID compID, Pel* predResiBorder, const Pel defaultVal );
  void asbtBorderResiH                       ( const TransformUnit& tu, const CPelBuf& recoBuf, const CPelBuf& predBuf, const ComponentID compID, Pel* predResiBorder, const Pel defaultVal );
  void asbtCoord                             ( TransformUnit& tu, ComponentID compID, int dir );
  void asbtCoordPred                         ( TransformUnit& tu, int decim, ComponentID compID );
  void invTransposeTransform                 ( TransformUnit& tu, ComponentID compID );
  int asbtDownSamplingPred                   ( TransformUnit &tu, ComponentID compID, const PelBuf& lumaPrediction, const PelBuf& lumaResi, PelBuf& downResi, int decim );
  void fillCoordBuf                          ( Pel* coordBuf, const uint8_t bestPos, const int height, const int width, const int iCoordStride );
  void fillCoordBufH                         ( Pel* coordBuf, const uint8_t bestPos, const int height, const int width, const int iCoordStride );
  void fillDownResiBuf                       ( PelBuf& downResi, const PelBuf& lumaResi, const uint32_t offset, const int height, const int width, const int iRecStride );
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
#if JVET_AK0076_EXTENDED_OBMC_IBC
  void subBlockIntraForOBMC                  (PredictionUnit &subPu, const int iSub, const bool isAbove, PelUnitBuf &cTmp, IntraPrediction *pcIntraPred, const bool lumaOnly);
#else
  void subBlockIntraForOBMC                  (PredictionUnit &subPu, const int iSub, const bool isAbove, PelUnitBuf &cTmp, IntraPrediction *pcIntraPred);
#endif
#endif
};

#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
struct InterPredResources // Bridge required resource from InterPrediction
{
  Reshape*              m_pcReshape;
  RdCost*               m_pcRdCost;
  InterpolationFilter&  m_if;
  Pel*                  m_ifBuf;       // m_filteredBlockTmp[0][compID]: temp interpolation buffer used to buffer horizontal interpolation output before vertical one performs
  Pel*                  m_preFillBufA; // m_filteredBlock[3][1][0]: Pre-interpolation buffer used to store search area samples of above template
  Pel*                  m_preFillBufL; // m_filteredBlock[3][0][0]: Pre-interpolation buffer used to store search area samples of left  template
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel*           m_cacheModel;
#endif
  InterPredResources( Reshape* pcReshape, RdCost* pcRdCost, InterpolationFilter& ifObj, Pel* ifBuf
                    , Pel* preFillBufA, Pel* preFillBufL
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
                    , CacheModel *cacheModel
#endif
  )
  : m_pcReshape   (pcReshape)
  , m_pcRdCost    (pcRdCost)
  , m_if          (ifObj)
  , m_ifBuf       (ifBuf)
  , m_preFillBufA (preFillBufA)
  , m_preFillBufL (preFillBufL)
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel(cacheModel)
#endif
  {};
};

class TplMatchingCtrl
{
  enum TMSearchMethod
  {
    TMSEARCH_DIAMOND,
    TMSEARCH_CROSS,
    TMSEARCH_NUMBER_OF_METHODS
  };
#if JVET_AD0213_LIC_IMP
public:
#endif
  const CodingUnit&         m_cu;
  const PredictionUnit&     m_pu;
        InterPredResources& m_interRes;

  const Picture&    m_refPic;
  const Mv          m_mvStart;
        Mv          m_mvFinal;
  const Mv*         m_otherRefListMv;
        Distortion  m_minCost;
        bool        m_useWeight;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
        int         m_mergeIdx;
#endif
        int         m_maxSearchRounds;
        ComponentID m_compID;

  PelBuf m_curTplAbove;
  PelBuf m_curTplLeft;
  PelBuf m_refTplAbove;
  PelBuf m_refTplLeft;
  PelBuf m_refSrAbove; // pre-filled samples on search area
  PelBuf m_refSrLeft;  // pre-filled samples on search area

#if JVET_X0056_DMVD_EARLY_TERMINATION
  Distortion m_earlyTerminateTh;
#endif
#if MULTI_PASS_DMVR
  Distortion m_tmCostArrayDiamond[9];
  Distortion m_tmCostArrayCross[5];
#if JVET_AE0091_HIGH_ACCURACY_TEMPLATE_MATCHING
  Distortion m_tmCostArrayDiamond16[17];
#endif
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  bool m_useTop;
  bool m_useLeft;
#endif

public:
  TplMatchingCtrl(const PredictionUnit&     pu,
                        InterPredResources& interRes, // Bridge required resource from InterPrediction
                  const Picture&            refPic,
                  const bool                fillCurTpl,
                  const ComponentID         compID,
                  const bool                useWeight,
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                  const int                 mergeIdx,
#endif
                  const int                 maxSearchRounds,
                        Pel*                curTplAbove,
                        Pel*                curTplLeft,
                        Pel*                refTplAbove,
                        Pel*                refTplLeft,
                  const Mv&                 mvStart,
                  const Mv*                 otherRefListMv,
                  const Distortion          curBestCost
#if JVET_AC0104_IBC_BVD_PREDICTION
                , const int tplSize = TM_TPL_SIZE
                , const bool isForBmvdFlag = false
#endif
                 );

  bool       getTemplatePresentFlag() { return m_curTplAbove.buf != nullptr || m_curTplLeft.buf != nullptr; }
  Distortion getMinCost            () { return m_minCost; }
  Mv         getFinalMv            () { return m_mvFinal; }
  static int getDeltaMean          (const PelBuf& bufCur, const PelBuf& bufRef, const int rowSubShift, const int bd);
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void       inverseCurTemplateLIC (int shift, int scale, int offset);
#endif

  template <int tplSize> void deriveMvUni    ();
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  template <int tplSize> void deriveMvUni2Pel(int step);
#endif
  template <int tplSize> void removeHighFreq (const Picture& otherRefPic, const Mv& otherRefMv, const uint8_t curRefBcwWeight);
#if JVET_AD0213_LIC_IMP
  template <int tplSize> void removeHighFreqLIC(const Picture& otherRefPic, const Mv& otherRefMv, const uint8_t curRefBcwWeight, int shift, int scale, int offset);
#endif
private:
  template <int tplSize, bool trueAfalseL>         bool       xFillCurTemplate    (Pel* tpl);
  template <int tplSize, bool trueAfalseL, int sr> PelBuf     xGetRefTemplate     (const PredictionUnit& curPu, const Picture& refPic, const Mv& _mv, PelBuf& dstBuf);
#if JVET_AC0104_IBC_BVD_PREDICTION
  template <int tplSize, bool trueAfalseL>         bool       xGetCurTemplateAvailable();
  template <int tplSize, bool trueAfalseL>         PelBuf     xGetCurTemplateBvd  (const PredictionUnit& curPu, const Picture& refPic, PelBuf& dstBuf);
  template <int tplSize, bool trueAfalseL>         PelBuf     xGetRefTemplateBvd  (const PredictionUnit& curPu, const Picture& refPic, const Mv& _mv, PelBuf& dstBuf);
#endif
  template <int tplSize, bool trueAfalseL>         void       xRemoveHighFreq     (const Picture& otherRefPic, const Mv& otherRefMv, const uint8_t curRefBcwWeight);
#if JVET_AD0213_LIC_IMP
  template <int tplSize, bool trueAfalseL>         void       xRemoveHighFreqLIC  (const Picture& otherRefPic, const Mv& otherRefMv, const uint8_t curRefBcwWeight, int shift, int scale, int offet);
#endif
  template <int tplSize, int searchPattern>         void       xRefineMvSearch    (int maxSearchRounds, int searchStepShift);
#if MULTI_PASS_DMVR
  template <int searchPattern>                      void       xNextTmCostAarray  (int bestDirect);
  template <int searchPattern>                      void       xDeriveCostBasedMv ();
  template <bool TrueX_FalseY>                      void       xDeriveCostBasedOffset (Distortion costLorA, Distortion costCenter, Distortion costRorB, int log2StepSize);
                                                    int        xBinaryDivision    (int64_t numerator, int64_t denominator, int fracBits);
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
public:
#endif
  template <int tplSize>                            Distortion xGetTempMatchError    (const Mv& mv);
#if JVET_AC0104_IBC_BVD_PREDICTION
  template <int tplSize>                            Distortion xGetTempMatchErrorBvd (const Mv& mv);
  template <int tplSize, bool trueAfalseL, bool useForBvd=false>
                                                    Distortion xGetTempMatchError    (const Mv& mv);
                                                    bool&      getCurTopRefAvailFlag () { return m_useTop; }
                                                    bool&      getCurLeftRefAvailFlag() { return m_useLeft; }
#else
  template <int tplSize, bool trueAfalseL>          Distortion xGetTempMatchError(const Mv& mv);
#endif
#if JVET_AD0213_LIC_IMP
public:
  template <int tplSize>                           Distortion xGetTempMatchLICError(const Mv& mv, int shift, int scale, int offset);
  template <int tplSize, bool trueAfalseL>         Distortion xGetTempMatchLICError(const Mv& mv, int shift, int scale, int offset);
#endif
};
#endif // TM_AMVP || TM_MRG
//! \}
//! 

#endif // __INTERPREDICTION__
