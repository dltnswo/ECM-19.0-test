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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"
#if JVET_W0123_TIMD_FUSION
#include "RdCost.h"
#endif

#include "MatrixIntraPrediction.h"
#if JVET_AK0118_BF_FOR_INTRA_PRED
#include "CommonLib/BilateralFilter.h"
#include "CommonLib/Reshape.h"
#endif
#if JVET_AB0155_SGPM
#include "CommonLib/InterpolationFilter.h"
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED  
#include "IntraPredictionNN.h"
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#include "CommonLib/InterPrediction.h"
class InterPrediction;
#endif
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const uint32_t MAX_INTRA_FILTER_DEPTHS=8;

#if JVET_V0130_INTRA_TMP
class TempLibFast
{
public:
  int   m_pX;           //offset X
  int   m_pY;           //offset Y
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  Mv    m_windTL;       // TL offset of refine window
  Mv    m_windBR;       // BR offset of refine window
  bool  m_isTransferredLeft;
  bool  m_isTransferredTop;
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int   m_rId;
#else
  int   m_pDiff;        //mse
  int   m_diffMax;
#endif

  TempLibFast();
  ~TempLibFast();
#if JVET_AD0086_ENHANCED_INTRA_TMP
  TempLibFast(const int pX, const int pY
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    , const Mv windTL, const Mv windBR, const bool isTransferredLeft, const bool isTransferredTop
#endif
    , const int rId)
  {
    m_pX = pX;
    m_pY = pY;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    m_windTL = windTL;
    m_windBR = windBR;
    m_isTransferredLeft = isTransferredLeft;
    m_isTransferredTop = isTransferredTop;
#endif
    m_rId = rId;
  };
#endif

  void  initTemplateDiff              ( unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int bitDepth );

  int   getX                          ()       { return m_pX;      }
  int   getY                          ()       { return m_pY;      }
#if !JVET_AD0086_ENHANCED_INTRA_TMP
  int   getDiff                       ()       { return m_pDiff;   }
  int   getDiffMax                    ()       { return m_diffMax; }
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  bool operator==(const TempLibFast& in)
  {
    if ((m_pX == in.m_pX) && (m_pY == in.m_pY))
    {
      return 1;
    }
    return 0;
  }
#endif
};

#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AH0200_INTRA_TMP_BV_REORDER
class TempLibFracFast
{
public:
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  int m_subpelX;
  int m_subpelY;
#else
  int   m_subpel;
#endif
  int   m_fracDir;

  TempLibFracFast();
  ~TempLibFracFast();
  TempLibFracFast(
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                  const int pPrecX, const int pPrecY
#else
                  const int pPrec
#endif
                  , const int pDirt)
  {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    m_subpelX = pPrecX, m_subpelY = pPrecY
#else
    m_subpel = pPrec
#endif
    , m_fracDir = pDirt;
  }; 
};
#endif
#endif

typedef short TrainDataType;
#endif

#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA || JVET_AC0119_LM_CHROMA_FUSION || JVET_AG0058_EIP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
typedef int64_t TCccmCoeff;
#if JVET_AJ0237_INTERNAL_12BIT
#define FIXED_MULT(x, y, round, bits) TCccmCoeff((int64_t(x)*(y) + round) >> bits )
#else
#define FIXED_MULT(x, y) TCccmCoeff((int64_t(x)*(y) + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS )
#endif
#if !JVET_AB0174_CCCM_DIV_FREE
#define FIXED_DIV(x, y)  TCccmCoeff((int64_t(x)    << CCCM_DECIM_BITS ) / (y) )
#endif

struct CccmModel
{
  CccmModel( int num, int bitdepth )
  {
    bd = bitdepth;
    midVal = ( 1 << ( bitdepth - 1 ) );
    params.resize( num );
#if JVET_AJ0237_INTERNAL_12BIT
    decimBits = DECIM_BITS(bd);
    decimRound = (1 << (decimBits - 1));
#endif
  }

  ~CccmModel() {}

  std::vector<TCccmCoeff> params;
  int        bd;
  int        midVal;
#if JVET_AJ0237_INTERNAL_12BIT
  int        decimRound;
  int        decimBits;
#endif
  
  const int getNumParams() const
  {
    return (int)params.size();
  }

  void clearModel()
  {
    const int numParams = (int)params.size();

    std::fill( params.begin(), params.end(), 0 );

#if JVET_AJ0237_INTERNAL_12BIT
    params[numParams - 1] = (TCccmCoeff)1 << decimBits; // Default bias to 1
#else
    params[numParams - 1] = 1 << CCCM_DECIM_BITS; // Default bias to 1
#endif
  }

  Pel convolve(Pel* vector)
  {
    TCccmCoeff sum = 0;
    
    for( int i = 0; i < params.size(); i++)
    {
      sum += params[i] * vector[i];
    }

#if JVET_AJ0237_INTERNAL_12BIT
    return Pel( (sum + decimRound) >> decimBits);
#else
    return Pel( (sum + CCCM_DECIM_ROUND ) >> CCCM_DECIM_BITS );
#endif
  }
  
  Pel nonlinear(const Pel val) const { return (val * val + midVal) >> bd; }
  Pel bias     ()              const { return midVal; }
#if JVET_AE0059_INTER_CCCM
  void setBd(const int bitdepth)
  {
    bd = bitdepth;
    midVal = (1 << (bitdepth - 1));
  }
#endif
#if JVET_AL0153_ALF_CCCM
  CccmModel() {}
  bool valid()
  {
#if JVET_AM0216_12BIT_FIX
    for (std::vector<TCccmCoeff>::iterator it = params.begin(); it < params.end(); it++)
    {
      if (abs(*it) > (4 << decimBits))
      {
        return false;
      }
    }
    return true;
#else
    return std::none_of(params.begin(),params.end(),[](TCccmCoeff x) {return abs(x) > (4 << CCCM_DECIM_BITS);});
#endif
  }
#endif
};

struct CccmCovariance
{
  CccmCovariance() {}
  ~CccmCovariance() {}

#if JVET_AB0174_CCCM_DIV_FREE
  void solve1                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, const int chromaOffset, CccmModel& model );
  void solve2                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, const int chromaOffsetCb, const int chromaOffsetCr, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode = false
#endif
  );
#if JVET_AL0153_ALF_CCCM
  void solve3( TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX], TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX], const int sampleNum, const int chromaOffsetCb, const int chromaOffsetCr, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode
#endif
#if JVET_AN0098_ALF_CCCM_REGULARIZATION
    , const bool hasNonLinearTerm
    , const bool regularization
#endif
  );
#endif
#else
  void solve1                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, CccmModel& model );
  void solve2                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode = false
#endif
  );
#endif
#if JVET_AG0058_EIP
#if JVET_AJ0082_MM_EIP
  void solveEip                    ( const TCccmCoeff* A, const TCccmCoeff* Y, const int sampleNum, const int lumaOffset, CccmModel& model, bool bMm = false);
#elif JVET_AB0174_CCCM_DIV_FREE
  void solveEip                    ( const TCccmCoeff* A, const TCccmCoeff* Y, const int sampleNum, const int lumaOffset, CccmModel& model );
#else
  void solveEip                    ( const TCccmCoeff* A, const TCccmCoeff* Y, const int sampleNum, CccmModel& model );
#endif
#endif
#if !JVET_AL0153_ALF_CCCM
private:
#endif
  TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff C[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX + 2];

#if JVET_AL0153_ALF_CCCM
private:
#endif
#if JVET_AC0053_GAUSSIAN_SOLVER
#if JVET_AJ0237_INTERNAL_12BIT
  void gaussBacksubstitution       ( TCccmCoeff* x, int numEq, int col, int round, int bits);
#else
  void gaussBacksubstitution       ( TCccmCoeff* x, int numEq, int col );
#endif
#if JVET_AE0059_INTER_CCCM
  void gaussElimination            ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y0, TCccmCoeff* x0, TCccmCoeff* y1, TCccmCoeff* x1, int numEq, int numFilters, int bd, const bool interCccmMode = false);
#else
  void gaussElimination            ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y0, TCccmCoeff* x0, TCccmCoeff* y1, TCccmCoeff* x1, int numEq, int numFilters, int bd);
#endif
#else
  bool ldlDecompose                ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, int numEq) const;
  void ldlSolve                    ( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, TCccmCoeff* y, TCccmCoeff* x, int numEq, bool decompOk) const;
  void ldlBacksubstitution         ( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* z, TCccmCoeff* x, int numEq) const;
  void ldlTransposeBacksubstitution( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y, TCccmCoeff* z, int numEq) const;
  bool ldlDecomp                   ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* outDiag, int numEq) const;
#endif
};
#endif

#if JVET_AJ0061_TIMD_MERGE
struct TimdMergeInfo
{
  int    timdMode[TIMD_FUSION_NUM];
  int    timdFusionWeight[TIMD_FUSION_NUM];
  bool   timdModeCheckWA[TIMD_FUSION_NUM];
  int8_t timdLocDep[TIMD_FUSION_NUM];
  bool   timdIsBlended;
  int    timdmTrType[2];
};
#endif

class IntraPrediction
{
public:
#if MMLM
  bool m_encPreRDRun;
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
  int  m_intraOBMCNeighState;
  bool m_refSampleForOBMC;
  Pel  m_refSampleForOBMCBuf[MAX_INTRA_SIZE + 3];
  Pel  m_refSampleForOBMCBufFiltered[MAX_INTRA_SIZE + 3];
#endif

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  Pel* m_cccmLumaBuf[CCCM_NUM_PRED_FILTER + 1];
#else
  Pel* m_cccmLumaBuf[2];
#endif
#else
  Pel* m_cccmLumaBuf;
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  CccmCovariance m_cccmSolver;

  Pel m_samples[CCCM_NUM_PARAMS_MAX];
  Pel m_a[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX];
  Pel m_cb[CCCM_REF_SAMPLES_MAX];
  Pel m_cr[CCCM_REF_SAMPLES_MAX];
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  int m_mmlmThreshold2;
#endif
#if JVET_AN0168_REGRESSION_CCP_FUSION
  AffineBlendingModel m_listFusionBlendModel[2][MAX_CCP_FUSION_NUM];  // [Cb/Cr]
  AffineBlendingModel m_ccpmListFusionBlendModel[2][MAX_CCP_FUSION_NUM];  // [Cb/Cr]
#endif

#if SECONDARY_MPM
  uint8_t m_intraMPM[NUM_MOST_PROBABLE_MODES];
#if JVET_AK0059_MDIP
  uint8_t m_intraNonMPM[NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES];
#else
  uint8_t m_intraNonMPM[NUM_NON_MPM_MODES];
#endif
#endif
#if JVET_AK0061_PDP_MPM
  bool m_mpmIncludedPdpMode[NUM_LUMA_MODE] = { false };
#endif
#if JVET_AH0209_PDP
  Pel* m_pdpIntraPredBufIP[NUM_LUMA_MODE];
  bool m_pdpIntraPredReady[NUM_LUMA_MODE];
  Pel  m_ref[ MAX_PDP_SIZE * 6 ];
  Pel  m_refShort[ MAX_PDP_SIZE * 6 ];
  bool m_refAvailable;
#if JVET_AK0061_PDP_MPM
  Pel m_leftRef[256] = { 0 };
  Pel m_leftRefShort[256] = { 0 };
#endif
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  Pel m_refTimd[2][256] = { {0}, {0} };
  Pel m_refShortTimd[2][256] = { {0}, {0} };
#endif
#if JVET_AL0108_BVG_DIMD
  int8_t m_bvgDimdFusionWeight[BVG_DIMD_INTRA_NUM];
  int8_t m_bvgDimdMode[BVG_DIMD_INTRA_NUM];
  int    m_bvDimdNum;
  int    m_bvDimdWeight[BVG_DIMD_BV_LIST_SIZE];
  double m_dSavedRDCostBvgDimdSad;
  uint64_t m_satdCostBvgDimd;
  static_vector<Mv, BVG_DIMD_BV_LIST_SIZE> m_bvListSorted;
#endif
#if JVET_AL0106_BV_EIP
  bool m_tmpSparseSearchCand;
  int  m_tmpSparseSearchXdisp;
  int  m_tmpSparseSearchYdisp;
  void setEipBvInfo(PredictionUnit& pu);
#endif

protected:
#if JVET_AJ0112_REGRESSION_SGPM
  std::vector <int16_t> m_blendBuf;
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  Pel        m_refBuffer[MAX_NUM_COMPONENT][NUM_PRED_BUF][((MAX_CU_SIZE << 3) + 1 + MAX_REF_LINE_IDX) * 2];
#else
  Pel        m_refBuffer[MAX_NUM_COMPONENT][NUM_PRED_BUF][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
#endif
  uint32_t   m_refBufferStride[MAX_NUM_COMPONENT];
#if JVET_AB0155_SGPM
  InterpolationFilter m_if;
#endif
#if JVET_AB0157_INTRA_FUSION
  Pel        m_refBuffer2nd[MAX_NUM_COMPONENT][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
#endif
#if JVET_AG0058_EIP
  Pel        m_eipBuffer[(MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE) * (MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE)];
  Pel        m_eipYBuffer[NUM_EIP_BASE_RECOTYPE][MAX_EIP_SIZE * MAX_EIP_SIZE * 2];
  Pel        m_eipPredTpl[2][MAX_EIP_SIZE * EIP_TPL_SIZE];
  TCccmCoeff ATABuf[NUM_EIP_COMB][((EIP_FILTER_TAP + 1) * EIP_FILTER_TAP) >> 1];
  TCccmCoeff ATYBuf[NUM_EIP_COMB][EIP_FILTER_TAP];
  bool       bSrcBufFilled[NUM_EIP_SHAPE * NUM_EIP_BASE_RECOTYPE];
  bool       bDstBufFilled[NUM_EIP_BASE_RECOTYPE];
  int        numSamplesBuf[NUM_EIP_BASE_RECOTYPE];
#if JVET_AH0086_EIP_BIAS_AND_CLIP
  Pel        m_eipBias;
  int        m_eipClipMin;
  int        m_eipClipMax;
#endif
#if JVET_AJ0082_MM_EIP
  Pel        m_mmEipInputs[2][CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX];
  Pel        m_eipAvg;
  TCccmCoeff mmEipATABuf[2][NUM_EIP_COMB][((EIP_FILTER_TAP + 1) * EIP_FILTER_TAP) >> 1];
  TCccmCoeff mmEipATYBuf[2][NUM_EIP_COMB][EIP_FILTER_TAP];
  Pel        m_mmEipYBuffer[(NUM_EIP_SHAPE * NUM_EIP_BASE_RECOTYPE) << 1][MAX_EIP_SIZE * MAX_EIP_SIZE * 2]; 
  int        numMmSamplesBuf[2][NUM_EIP_SHAPE * NUM_EIP_BASE_RECOTYPE];
  int        m_numSigEip;
#endif 
#if JVET_AL0106_BV_EIP
  int        m_bvEipTempSize[2];
  Pel        m_bvEipBuffer[(MAX_EIP_SIZE + BV_EIP_TEMP_SIZE) * (MAX_EIP_SIZE + BV_EIP_TEMP_SIZE)];
#endif
#endif
private:
#if JVET_AG0136_INTRA_TMP_LIC
  Pel arrayTemp[TMP_FUSION_NUM][(MAX_CU_SIZE + TMP_TEMPLATE_SIZE) * (MAX_CU_SIZE + TMP_TEMPLATE_SIZE)];
#endif

#if !MERGE_ENC_OPT
  Pel* m_yuvExt2[MAX_NUM_COMPONENT][4];
  int  m_yuvExtSize2;
#endif

#if JVET_AA0057_CCCM || JVET_AC0119_LM_CHROMA_FUSION || JVET_AG0058_EIP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  Area m_cccmBlkArea;
#if JVET_AB0174_CCCM_DIV_FREE
  int  m_cccmLumaOffset;
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  Area m_tmpRefArea[MTMP_NUM];
  Pel* m_tmpRefBuf[MTMP_NUM];
#endif

#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  Area m_cccmRefArea;
#if JVET_AE0100_BVGCCCM
  Pel* m_bvgCccmLumaBuf[NUM_BVG_CCCM_CANDS];
  Pel* m_bvgCccmChromaBuf[NUM_BVG_CCCM_CANDS][2];
  Area m_bvgCccmBlkArea;
  Area m_bvgCccmRefArea;
#endif
#endif
  
  static const uint8_t m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS];
#if JVET_W0123_TIMD_FUSION
  static const uint8_t m_aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS];
  RdCost* m_timdSatdCost;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  RdCost* m_itmpSatdCost;
#endif
#if JVET_AC0071_DBV
  RdCost *m_dbvSadCost;
#endif
#if LMS_LINEAR_MODEL
  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication
#endif
#if JVET_AN0158_INTRA_LONGTAP
  static constexpr int g_angTable[32] = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
#if JVET_W0123_TIMD_FUSION
  static constexpr int g_extAngTable[64] = { 0, 1, 2, 3, 4, 5, 6,7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 74, 78, 84, 90, 96, 102, 108, 114, 121, 128, 137, 146, 159, 172, 188, 204, 230, 256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
#endif
#endif
  struct IntraPredParam //parameters of Intra Prediction
  {
    bool refFilterFlag;
    bool applyPDPC;
#if GRAD_PDPC
    bool useGradPDPC;
#endif
    bool isModeVer;
    int  multiRefIndex;
    int  intraPredAngle;
    int  absInvAngle;
    bool interpolationFlag;
    int  angularScale;
#if JVET_AB0157_INTRA_FUSION
    bool fetchRef2nd;
    bool applyFusion;
#endif

#if JVET_AN0158_INTRA_LONGTAP
    int numTapsTIMD;
#endif

    // clang-format off
    IntraPredParam()
      : refFilterFlag(false)
      , applyPDPC(false)
#if GRAD_PDPC
      , useGradPDPC(false)
#endif
      , isModeVer(false)
      , multiRefIndex(-1)
      , intraPredAngle(std::numeric_limits<int>::max())
      , absInvAngle(std::numeric_limits<int>::max())
      , interpolationFlag(false)
      , angularScale(-1)
#if JVET_AB0157_INTRA_FUSION
      , fetchRef2nd(false)
      , applyFusion(false)
#endif
#if JVET_AN0158_INTRA_LONGTAP
      , numTapsTIMD(0)
#endif

    // clang-format on
    {
    }
  };

  IntraPredParam m_ipaParam;

#if JVET_AN0158_INTRA_LONGTAP
  int m_dimdCurrentPredIdx = 0;
  std::pair<int,int> m_numTapsPerModeTIMD;
#endif

#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  Pel* m_pCCFilterTemp;
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  Pel* m_decoderDerivedCcpProbeTemplateT[2];
  Pel* m_decoderDerivedCcpProbeTemplateL[2];
  Pel* m_ddCCPFusionTempCb[MAX_DDCCP_CAND_LIST_SIZE];
  Pel* m_ddCCPFusionTempCr[MAX_DDCCP_CAND_LIST_SIZE];
#if JVET_AN0168_REGRESSION_CCP_FUSION
  Pel* m_ccpStoreFusionTempCb[MAX_CCP_CAND_LIST_SIZE + 1];
  Pel* m_ccpStoreFusionTempCr[MAX_CCP_CAND_LIST_SIZE + 1];
  Pel* m_ccpFusionTempCb[MAX_CCP_CAND_LIST_INIT_SIZE + 1];
  Pel* m_ccpFusionTempCr[MAX_CCP_CAND_LIST_INIT_SIZE + 1];
#else
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  Pel* m_ccpFusionTempCb[MAX_CCP_CAND_LIST_INIT_SIZE];
  Pel* m_ccpFusionTempCr[MAX_CCP_CAND_LIST_INIT_SIZE];
#else
  Pel* m_ccpFusionTempCb[MAX_CCP_CAND_LIST_SIZE];
  Pel* m_ccpFusionTempCr[MAX_CCP_CAND_LIST_SIZE];
#endif
#endif
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  Pel* m_pCcpMerge[2];
#endif
#if JVET_AB0067_MIP_DIMD_LFNST
  Pel* m_pMipTemp;
#endif
  Pel* m_piTemp;
  Pel* m_pMdlmTemp; // for MDLM mode
#if JVET_AA0126_GLM
  Pel* m_glmTempCb[NUM_GLM_IDC];
  Pel* m_glmTempCr[NUM_GLM_IDC];
#if JVET_AB0092_GLM_WITH_LUMA
  Area m_glmRefArea;
  Pel* m_glmGradBuf[NUM_GLM_IDC];
#if JVET_AB0174_CCCM_DIV_FREE
  int  m_glmLumaOffset;
#endif
#endif
#endif
#if JVET_AC0119_LM_CHROMA_FUSION
  Area m_cflmRefArea;
  Pel* m_cflmBuf[3];
#endif
  MatrixIntraPrediction m_matrixIntraPred;
#if JVET_AG0136_INTRA_TMP_LIC
  std::array<std::array<std::array<int, 7>, MTMP_NUM>, 4> m_memLicParams;
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED  
  IntraPredictionNN        m_intraPredNN;
#endif



protected:
  ChromaFormat              m_currChromaFormat;
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  int8_t                    m_evalIdx;
#endif

  int                       m_topRefLength;
  int                       m_leftRefLength;
  ScanElement*              m_scanOrder;
  bool                      m_bestScanRotationMode;
  std::vector<PelStorage>   m_tempBuffer;
#if JVET_AB0155_SGPM
  std::vector<PelStorage>   m_sgpmBuffer;
#endif
#if JVET_AL0134_SGPM_INTER
  Pel*                      m_acSgpmInterYuvRefTemplate[2][MAX_NUM_COMPONENT];   // 0: top, 1: left
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
public:
  bool                      m_sgpmInterSadAvail[GEO_MAX_NUM_UNI_CANDS];
  Distortion                m_sgpmInterSadWholeTM[GEO_MAX_NUM_UNI_CANDS];
  Distortion                m_sgpmInterSadPartsTM[GEO_MAX_NUM_UNI_CANDS][GEO_NUM_PARTITION_MODE];
protected:
#endif 
#endif
  // used in timd tmrl sortedMPM
  std::vector<PelStorage>   m_intraPredBuffer;
  Pel                       m_tempRefAbove[(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];
  Pel                       m_tempRefLeft [(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];
#if JVET_AH0209_PDP
  bool (*m_xPredIntraOpt)  (PelBuf &pDst, const PredictionUnit &pu, const uint32_t modeIdx, const ClpRng& clpRng, Pel* refF, Pel* refS);
  static bool xPredIntraOpt(PelBuf &pDst, const PredictionUnit &pu, const uint32_t modeIdx, const ClpRng& clpRng, Pel* refF, Pel* refS);
#endif
#if JVET_V0130_INTRA_TMP
  int          m_uiPartLibSize;
#if JVET_AD0086_ENHANCED_INTRA_TMP
  static_vector<TempLibFast, MTMP_NUM> m_mtmpCandList;
  static_vector<uint64_t, MTMP_NUM>    m_mtmpCostList;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  static_vector<TempLibFracFast, TMP_BV_REORDER_MAX> m_mtmpFracCandList;
  static_vector<uint64_t, TMP_BV_REORDER_MAX> m_mtmpFracCostList;
#else
  static_vector<TempLibFracFast, TMP_BV_REORDER_MAX> m_mtmpFracCandList[MTMP_NUM];
  static_vector<uint64_t, TMP_BV_REORDER_MAX>        m_mtmpFracCostList[MTMP_NUM];
  int m_log2SizeTop;
  int m_log2SizeLeft;
  int m_sizeTopLeft;
  int m_topMeanTar;
  int m_leftMeanTar;
#endif
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  static_vector<TempLibFast, MTMP_NUM> m_mtmpCandListUseMR;
  static_vector<uint64_t, MTMP_NUM>    m_mtmpCostListUseMR;
#endif
#else
  TempLibFast  m_tempLibFast;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  static_vector<int, MTMP_NUM> m_mtmpRequiredTemplate;
  static_vector<int, MTMP_NUM> m_mtmpRequiredTemplateUseMR;
#endif
  Pel*         m_refPicUsed;
  Picture*     m_refPicBuf;
  unsigned int m_uiPicStride;
  unsigned int m_uiVaildCandiNum;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  Pel*         m_pppTarPatch;
#else
  Pel***       m_pppTarPatch;
#endif
#endif

#if TMP_FAST_ENC
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int                m_tmpXdisp[MTMP_NUM];
  int                m_tmpYdisp[MTMP_NUM];
#if JVET_AG0136_INTRA_TMP_LIC
  int                m_tmpXdispUseMR[MTMP_NUM];
  int                m_tmpYdispUseMR[MTMP_NUM];
#endif
  IntraTMPFusionInfo m_tmpFusionInfo[
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                     TMP_FUS_MODES_NUM
#else
                                     TMP_GROUP_IDX << 1
#endif
                                     ];
#if JVET_AG0136_INTRA_TMP_LIC
  IntraTMPFusionInfo m_tmpFusionInfoUseMR[
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                          TMP_FUS_MODES_NUM
#else
                                          TMP_GROUP_IDX << 1
#endif
                                          ];
#endif
#else
  int            m_tmpXdisp;
  int            m_tmpYdisp;
#endif
  int            m_tmpNumCand;
#if JVET_AG0136_INTRA_TMP_LIC
  int            m_tmpNumCandUseMR;
#endif
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  Pel m_memOffsetsFusionUseMR[
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                              TMP_FUS_MODES_NUM - TMP_GROUP_IDX
#else
                              3
#endif
                              ][TMP_FUSION_NUM];
#endif

#if JVET_AN0158_INTRA_LONGTAP
  std::pair<int, int> xGetNumTapsTIMD(const PredictionUnit& pu, const ComponentID& compID) const;
  using IF_LENGTH_INFO = static_vector<std::pair<int, int>, MAX_MODE_NUM>; // <mode, if_length>
  IF_LENGTH_INFO xGetInterpolationFilteringBudget(const PredictionUnit& pu, const ComponentID& compID) const;

  IF_LENGTH_INFO m_budget;

  using MODE_FUSION_INFO = static_vector<std::pair<int, int>, MAX_MODE_NUM>;
  IF_LENGTH_INFO xAdjustTapBudget(const IF_LENGTH_INFO& initIfTapInfo, const MODE_FUSION_INFO& fusionInfo, const SizeType& blkWidth, const SizeType& blkHeight) const;
#endif

  // prediction
  void xPredIntraPlanar           ( const CPelBuf &pSrc, PelBuf &pDst
#if JVET_AC0105_DIRECTIONAL_PLANAR
    , uint8_t plidx
#endif
  );
  void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter = true );
#if JVET_W0123_TIMD_FUSION
#if JVET_AB0157_INTRA_FUSION
  void xPredIntraAng              (
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
    const PredictionUnit& pu,
#endif
    const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir, const CPelBuf &pSrc2nd,
#if JVET_AK0118_BF_FOR_INTRA_PRED
    bool applyBf, CodingStructure& cs, int blkQp, const PredictionUnit& puReco,
#endif
    bool isISP = true, int weightMode = 4);
#else
  void xPredIntraAng              (
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
    const PredictionUnit& pu,
#endif
    const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir);
#endif
#else
  void xPredIntraAng              (
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
    const PredictionUnit& pu,
#endif
    const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng);
#endif
#if !(JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
#if JVET_AB0155_SGPM
  void initPredIntraParams        (const PredictionUnit &pu, const CompArea compArea, const SPS &sps, const int partIdx = 0);
#else
  void initPredIntraParams        ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );
#endif
#endif

  static bool isIntegerSlope      (const int absAng) { return (0 == (absAng & 0x1F)); }
#if JVET_W0123_TIMD_FUSION
  static bool isIntegerSlopeExt   (const int absAng) { return (0 == (absAng & 0x3F)); }
#endif

  void xPredIntraBDPCM            ( const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng );
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const Size &dstSize );
#if JVET_V0130_INTRA_TMP && !JVET_W0069_TMP_BOUNDARY
  bool isRefTemplateAvailable(CodingUnit& cu, CompArea& area);
#endif

#if JVET_AJ0249_NEURAL_NETWORK_BASED
  void xPredIntraPnn               (const CPelBuf& recoBuf, PelBuf& pDst, const CPelBuf& srcBuf, CodingUnit& cu, const ComponentID compID
#if JVET_AK0118_BF_FOR_INTRA_PRED
  , bool applyBf
#endif
  );
  void xFillReferenceSamples       ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu, const bool forceDeac0 = false, const bool forceDeac1 = false );
#else
  void xFillReferenceSamples       ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
  void xFillReferenceSamplesOBMC   ( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
#endif
#if JVET_AH0136_CHROMA_REORDERING
  void xFillReferenceSamplesForCoLuma(const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu);
#endif
#if JVET_AH0209_PDP
  void xFillReferenceSamples2      (const CPelBuf &recoBuf, const CompArea &area, const CodingUnit &cu);
#endif
  void xFilterReferenceSamples     (const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area, const SPS &sps, int multiRefIdx );

  static int getModifiedWideAngle  ( int width, int height, int predMode );
#if JVET_W0123_TIMD_FUSION
#if JVET_AB0155_SGPM
  static int getWideAngleExt       (int width, int height, int predMode, bool bSgpm = false);
#else
  static int getWideAngleExt       ( int width, int height, int predMode );
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  static int getTimdWideAngleExt   (int width, int height, int predMode);
  static int getTimdRegularAngleExt(int width, int height, int predMode);
#endif
#endif

#if JVET_AC0094_REF_SAMPLES_OPT
  static int getTimdWideAngle      (int width, int height, int predMode);
#endif
  void setReferenceArrayLengths    ( const CompArea &area );

  void destroy                     ();
#if LMS_LINEAR_MODEL
  void xPadMdlmTemplateSample      (Pel*pSrc, Pel*pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum);
  void xGetLMParametersLMS         (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#else
  void xGetLMParameters            (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
public:
#endif
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  std::vector<Mv> m_itmpBvBasedMergeCandidates;
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  std::vector<Mv> m_timdBvBasedMergeCandidates;
#endif
  std::vector<Mv> m_dimdBvBasedMergeCandidates;
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  std::vector<BvInfo> m_sgpmMvBasedMergeCandidates;
#else
  std::vector<Mv> m_sgpmMvBasedMergeCandidates;
#endif
  int itmpSize, itmpSize1;
#else
  std::vector<Mv> m_bvBasedMergeCandidates;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  std::vector<BvInfo> m_sgpmMvBasedMergeCandidates;
#else
  std::vector<Mv> m_sgpmMvBasedMergeCandidates;
#endif
#endif
#endif
#endif
#if JVET_AM0074_INTRA_MERGE
  std::vector<uint64_t> m_bvBasedMergeCostCandidates;
  std::vector<int> m_bvBasedMergeLocDepCandidates;
  uint64_t g_intraModeTmCost[EXT_VDIA_IDX + 1];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  Pel* m_refTarget[TMP_FUSION_NUM];
#endif
#if LMS_LINEAR_MODEL && MMLM
  struct MMLMParameters
  {
    int a;
    int b;
    int shift;
  };
#if JVET_AJ0237_INTERNAL_12BIT
  int xCalcLMParametersGeneralized   (int64_t x, int64_t y, int64_t xx, int64_t xy, int count, int bitDepth, int& a, int& b, int& iShift);
#else
  int xCalcLMParametersGeneralized   (int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b, int &iShift);
#endif
  int xLMSampleClassifiedTraining    (int count, int mean, int meanC, int lumaSamples[], int chrmSamples[], int bitDepth, MMLMParameters parameters[]);
#if JVET_AG0136_INTRA_TMP_LIC
  std::array<int, 7>& getMemLicParams(const int licIdc, const int idx) { return m_memLicParams[licIdc][idx]; }
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  int getRequiredTemplate(const bool useMR, const bool isLicExtension, const int idx) { return isLicExtension ? 0 : (useMR ? m_mtmpRequiredTemplateUseMR : m_mtmpRequiredTemplate)[idx]; }
#endif
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  void setEvalIdx                    (const int8_t evalIdx) { m_evalIdx = evalIdx; }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
protected:
#endif
#if JVET_Z0050_CCLM_SLOPE
  void xUpdateCclmModel              (int &a, int &b, int &iShift, int midLuma, int delta);
#endif

public:
  IntraPrediction();
  virtual ~IntraPrediction();

#if JVET_AJ0249_NEURAL_NETWORK_BASED
#if JVET_AK0118_BF_FOR_INTRA_PRED
  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY, const int nnipMode, Reshape* reshape, BilateralFilter* bilateralFilter );
#else
  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY, const int nnipMode);
#endif
  void resetCuData                () { m_intraPredNN.resetCuData(); }
  bool getIsContextCollectionNeeded(const CompArea& area) { return m_intraPredNN.isContextCollectionNeeded(area); }
#else
  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);
#endif

#if JVET_AJ0146_TIMDSAD
  static_vector<std::pair<uint64_t,int>, NUM_LUMA_MODE> m_timdModeCostList;
#endif
#if JVET_AA0057_CCCM || JVET_AC0119_LM_CHROMA_FUSION || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  Pel    xCccmGetLumaVal          (const PredictionUnit& pu, const CPelBuf pi, const int x, const int y
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
  ) const;
#if JVET_AB0174_CCCM_DIV_FREE
  void   xCccmSetLumaRefValue     (const PredictionUnit& pu);
#endif
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
#if JVET_AD0188_CCP_MERGE
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void   predIntraCCCM            ( PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir, bool ccpModelStorage = true, CCPModelCandidate ccpModel2 = {} );
#else
  void   predIntraCCCM            ( PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir );
#endif
#else
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void   predIntraCCCM            (const PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir, bool ccpModelStorage = true, CCPModelCandidate ccpModel2 = {});
#else
  void   predIntraCCCM            (const PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir);
#endif
#endif

  void   xCccmCalcModels          (const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel& cccmModelCr, int modelId, int modelThr
#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
    , int trainingRange = -1
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool useRefSampOnly = false
#endif
  );

  void   xCccmApplyModel          (const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred);

  void   xCccmCreateLumaRef       (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
  PelBuf xCccmGetLumaRefBuf       (const PredictionUnit& pu, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY
#if JVET_AD0202_CCCM_MDF
    , int cccmDownsamplesFilterIdx = 0, int numBuffer = 0, PelBuf* refLuma1 = NULL, PelBuf* refLuma3 = NULL, PelBuf* refLuma2 = NULL
#endif
  ) const;
  PelBuf xCccmGetLumaPuBuf        (const PredictionUnit& pu
#if JVET_AD0202_CCCM_MDF
    , int cccmDownsamplesFilterIdx = 0, int numBuffer = 0, CPelBuf* refLuma1 = NULL, CPelBuf* refLuma3 = NULL, CPelBuf* refLuma2 = NULL
#endif
  ) const;
  int    xCccmCalcRefAver         (const PredictionUnit& pu
#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
                                    , int trainingRange = -1
#endif
  ) const;
  void   xCccmCalcRefArea         (const PredictionUnit& pu, CompArea chromaArea);
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  void   xCccmCreateLumaNoSubRef  ( const PredictionUnit& pu, CompArea chromaArea 
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
#endif
#if JVET_AE0100_BVGCCCM
  void   xBvgCccmCalcRefArea      (const PredictionUnit& pu, CompArea chromaArea);
  PelBuf xBvgCccmGetLumaPuBuf     (const PredictionUnit& pu, int candIdx = 0) const;
  PelBuf xBvgCccmGetLumaPuBufFul  (const PredictionUnit& pu, int candIdx = 0) const;
  PelBuf xBvgCccmGetChromaPuBuf   (const PredictionUnit& pu, const ComponentID compId, int candIdx = 0) const;
  void   xBvgCccmCreateLumaRef    (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
                                   );
  int    xBvgCccmCalcBlkAver      (const PredictionUnit& pu) const;
  void   xBvgCccmCalcBlkRange     (const PredictionUnit& pu, int& minVal, int&maxVal) const;
  void   xBvgCccmCalcModels       ( const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel& cccmModelCr, int modelId, int modelThr, int minVal, int maxVal );
  void   xBvgCccmApplyModel       ( const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred );
#endif
#endif
#if JVET_AB0092_GLM_WITH_LUMA
  void   xGlmCalcModel            (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel);
  void   xGlmApplyModel           (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel, PelBuf &piPred);
  void   xGlmCreateGradRef        (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
  PelBuf xGlmGetGradRefBuf        (const PredictionUnit& pu, CompArea chromaArea, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY, int glmIdx) const;
  PelBuf xGlmGetGradPuBuf         (const PredictionUnit& pu, CompArea chromaArea, int glmIdx) const;
  Pel    xGlmGetGradVal           (const PredictionUnit& pu, const int glmIdx, const CPelBuf pi, const int x, const int y) const;
  void   xGlmCalcRefArea          (const PredictionUnit& pu, CompArea chromaArea);
#if JVET_AB0174_CCCM_DIV_FREE
  void   xGlmSetLumaRefValue      (const PredictionUnit& pu, CompArea chromaArea);
#endif
#endif
#if JVET_AC0119_LM_CHROMA_FUSION
  void   xCflmCalcModels          (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr);
  void   xCflmApplyModel          (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr, PelBuf& piPred);
  void   xCflmCreateLumaRef       (const PredictionUnit& pu, const CompArea& chromaArea);
  bool   xCflmCreateChromaPred    (const PredictionUnit& pu, const ComponentID compId, PelBuf& piPred
#if JVET_AH0136_CHROMA_REORDERING
    , InterPrediction *pcInterPred
#endif
  );
  PelBuf xCflmGetRefBuf           (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, int& areaWidth, int& areaHeight, int& refSizeX, int& refSizeY, int& refPosPicX, int& refPosPicY) const;
  PelBuf xCflmGetPuBuf            (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea) const;
  int    xCflmCalcRefAver         (const PredictionUnit& pu, const CompArea& chromaArea);
  void   xCflmCalcRefArea         (const PredictionUnit& pu, const CompArea& chromaArea);
#endif

#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void reorderCCPCandidates       ( PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize, int* fusionList );
  int  xGetOneCCPCandCost         ( PredictionUnit &pu, CCPModelCandidate &ccpCand, const int candIdx = 0 );
#else
  void reorderCCPCandidates       ( PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize );
  int  xGetOneCCPCandCost         ( PredictionUnit &pu, CCPModelCandidate &ccpCand );
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  int xGetCostCCCM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr, int lumaOffset, int chromaOffset[2], bool aboveAvailable, bool leftAvailable, PelBuf chromaReco, int type, int refSizeX = 0, int refSizeY = 0, const int cccmMultiFilterIdx = -1, int modelIdx = 0, int* cand0 = nullptr, int* cand1 = nullptr);
  int  xGetOneCCPCandCost         ( PredictionUnit &pu, CCPModelCandidate &ccpCand, CompArea chromaArea, CccmModel cccmModel[], bool aboveAvailable, bool leftAvailable, PelBuf chromaReco, const int candIdx = 0, int compMode = 0, int modelIdx = 0, int* cand0 = nullptr, int* cand1 = nullptr );
  bool deriveCcpMergeAdjustCands (PredictionUnit &pu, CompArea chromaArea, CCPModelCandidate candList[], int& modelNum);
  void adjustCCPCandidates ( PredictionUnit &pu, CCPModelCandidate candList[], int maxCandNum, int candIdx = -1);
#endif
  void predCCPCandidate           ( PredictionUnit &pu, PelBuf &predCb, PelBuf &predCr);

  void xCclmApplyModel            (const PredictionUnit &pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred);
  void xCccmApplyModelOffset      (const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf& piPred, int lumaOffset, int chromaOffset[2], int type, int refSizeX = 0, int refSizeY = 0 );
  void xGlmApplyModelOffset       (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel, int glmIdc, PelBuf& piPred, int lumaOffset, int chromaOffset);  

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
#if JVET_AN0168_REGRESSION_CCP_FUSION
  void storeFusionTemp(int numPos);
  int xComputeBlendFusionModelJoint(const PredictionUnit& pu, const CompArea& chromaAreaCb, const CompArea& chromaAreaCr, int candIdx0, int candIdx1, AffineBlendingModel& blendModel, const int idxTemp);
#endif
  int xGetCostCCPFusion           (const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, int candIdx0, int candIdx1);
  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCLM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CclmModel &cclmModel, int modelNum, int glmIdc, int candIdx = 0);

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCCM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr, int lumaOffset, int chromaOffset[2], int type, int candIdx = 0, int refSizeX = 0, int refSizeY = 0, const int cccmMultiFilterIdx = -1);

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostGLM (const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, CccmModel& glmModel, int glmIdc, int lumaOffset, int& chromaOffset, int candIdx = 0);
#else
  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCLM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CclmModel &cclmModel, int modelNum, int glmIdc);

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCCM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr, int lumaOffset, int chromaOffset[2], int type, int refSizeX = 0, int refSizeY = 0, const int cccmMultiFilterIdx = -1 );

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostGLM (const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, CccmModel& glmModel, int glmIdc, int lumaOffset, int& chromaOffset);
#endif
#endif

#if JVET_AN0168_REGRESSION_CCP_FUSION
  void ccpmComputeTemplateFusionModel( const PredictionUnit& pu, const ComponentID compID, CccmModel cccmModel[2], int modelThr );
  void ccpmComputeBlendFusionModelJoint( const PredictionUnit& pu );
  bool useBlendModelCcpm( const PredictionUnit& pu ) 
  { 
    bool useBlend = pu.cs->sps->getCcpFusionMode();
    return useBlend; 
  }  
  bool useBlendModelDdccp( const PredictionUnit& pu ) 
  { 
    return true;
  }  
#endif

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void filterPredInsideProbeLine         (const ComponentID compID, const PredictionUnit &pu, bool above); // training stage
  int ddccpFusionTemplateCost            (const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, int candIdx0, int candIdx1, int cost0, int cost1);
  int decoderDerivedCccmCost             (const PredictionUnit &pu, int currIdx, const ComponentID compID, int intraDir, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr); // training stage
  int decoderDerivedCclmCost             (const PredictionUnit &pu, int currIdx, const ComponentID compID, int intraDir, const CompArea  &chromaArea, const CclmModel &cclmModel);
  int tmCostDecoderDerivedCcp            (PredictionUnit& pu, int currIdx, int intraDir, bool isCcpMerge = false); // training stage 
  void predDecoderDerivedIntraCCCMFusions(PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, std::vector<DecoderDerivedCcpCandidate> &decoderDerivedCcpList);
  int decoderDerivedCcp                  (PredictionUnit& pu, std::vector<DecoderDerivedCcpCandidate> &decoderDerivedCcpList); // training stage
  void   predDecoderDerivedCcpMergeFusion(PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, CCPModelCandidate decoderDerivedCcp1, CCPModelCandidate decoderDerivedCcp2
#if JVET_AN0168_REGRESSION_CCP_FUSION
    , const int idxDdNonLocalCCPFusion
#endif
  );
#if JVET_AN0168_REGRESSION_CCP_FUSION
  void applyBlendFusion                  (AffineBlendingModel& blendModelCb, PelBuf& predCb, PelBuf& predLmBufferCb, PelBuf& predDst, const ClpRng& clpRng);
#endif
#endif

#if JVET_AF0073_INTER_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void xInterCccmApplyModelOffset        (const PredictionUnit &pu, const ComponentID compId, CccmModel &cccmModel, PelBuf &piPred, int lumaOffset, int chromaOffset);
  int  xGetCostInterCccm                 (const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CccmModel &cccmModel, int lumaOffset, int chromaOffset
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
    , int candIdx = 0
#endif
  );
  void xAddOnTheFlyCalcCCPCands4InterBlk (const PredictionUnit &pu, CompArea chromaArea, CCPModelCandidate candList[], int &validNum);
  void selectCcpMergeCand                (PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize);
  void combineCcpAndInter                (PredictionUnit &pu, PelBuf &inPredCb, PelBuf &inPredCr, PelBuf &outPredCb, PelBuf &outPredCr
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    , bool useExistCcp = false
#endif
  );
#endif

#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
  const IntraPredParam* getIpaParam      () { return &m_ipaParam; };
  bool getGradForOBMC                    (const PredictionUnit pu, const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, const bool isAbove, const int blkSize, int* modeBuf);
#endif

#if JVET_AK0059_MDIP
  void deriveMdipMode                     (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, bool useExcludingMode = true);
#endif
#if ENABLE_DIMD
  static void deriveDimdMode              (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu
#if JVET_AK0059_MDIP
    , bool useExcludingMode = false
#endif
  );
#if JVET_AJ0267_ADAPTIVE_HOG
  static void deriveDimdModeAdaptive      (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu
#if JVET_AK0059_MDIP
    , bool useExcludingMode = false
#endif
  );
#endif
#if JVET_AL0108_BVG_DIMD
  void deriveBvgDimdMode                  (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu);
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  static void deriveDimdChromaMode        (const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr, const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu);
#if JVET_AH0136_CHROMA_REORDERING
   void deriveNonCcpChromaModes           (const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr, const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu, PredictionUnit &pu, InterPrediction *pcInterPred);
   void buildChromaModeList               (const CompArea &area, CodingUnit &cu, PredictionUnit &pu, uint8_t chromaList[NUM_CHROMA_LIST_MODE], int &existNum, bool &hasDBV);
   void predCoLuma                        (const CompArea &area, const CPelBuf &recoBuf, PredictionUnit &pu, uint8_t predMode, PelBuf predBuf, InterPrediction *pcInterPred, CodingUnit &chromaCu);
   void predChromaTM                      (const CompArea &areaCb, const CompArea &areaCr, PredictionUnit &pu, uint8_t predMode, PelBuf predCb, PelBuf predCr, TemplateType eTplType, InterPrediction *pcInterPred);
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
   static void deriveDimdModeList(const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu, static_vector<int, MTSS_LIST_SIZE>& candModeList, static_vector<int, MTSS_LIST_SIZE>& candCostList);
#endif 
#if ENABLE_DIMD && (JVET_AB0067_MIP_DIMD_LFNST || JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST || JVET_AG0058_EIP || JVET_AG0061_INTER_LFNST_NSPT)
  static int deriveIpmForTransform        (CPelBuf predBuf, CodingUnit& cu
#if JVET_AI0050_INTER_MTSS
    , int &secondMode
#endif
#if JVET_AJ0203_DIMD_2X2_EDGE_OP
    , const bool enableAlignedDesign = false
#endif 
  );
#endif
#if JVET_AK0064_CCP_LFNST_NSPT
  static int deriveChromaIpmForTransform   (CPelBuf predBufCb, CPelBuf predBufCr, CodingUnit& cu);
#endif
#if !JVET_AG0061_INTER_LFNST_NSPT
  static int  buildHistogram               ( const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int* piHistogram, int direction, int bw, int bh );
#endif
#endif
#if JVET_W0123_TIMD_FUSION || JVET_AC0119_LM_CHROMA_FUSION
  void xIntraPredTimdHorVerPdpc   (Pel* pDsty,const int dstStride, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, const Pel* refMain, const ClpRng& clpRng);
  void xPredTimdIntraPlanar       (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraDc           ( const PredictionUnit &pu, const CPelBuf &pSrc, Pel* pDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraAng          ( const CPelBuf &pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight, uint32_t dirMode
#if JVET_AC0119_LM_CHROMA_FUSION
    , const ChannelType channelType
#endif
  );
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
  void predTimdIbcItmp            (const ComponentID compId, const PredictionUnit& pu, Mv Bv, Pel* pPred, uint32_t uiStride, uint32_t iWidth, uint32_t iHeight, TemplateType eTempType, int32_t iTemplateWidth, int32_t iTemplateHeight, Pel* piOrg, int orgStride);
  void predUsingBv                (Pel* piPred, unsigned int uiStride, Mv Bv, CodingUnit cu
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  , bool isIntBv = true
#endif  
  );
#endif
  void xIntraPredTimdAngLuma      (Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset);
#if JVET_AC0119_LM_CHROMA_FUSION
  void xIntraPredTimdAngChroma    (Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset);
#endif
  void xIntraPredTimdPlanarDcPdpc (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xIntraPredTimdAngPdpc      (Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height, int xOffset, int yOffset, int scale, int invAngle);
  void xFillTimdReferenceSamples  ( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight );
  Pel  xGetPredTimdValDc          ( const CPelBuf &pSrc, const Size &dstSize, TemplateType eTempType, int iTempHeight, int iTempWidth );
#if JVET_AB0155_SGPM
  void initPredTimdIntraParams    (const PredictionUnit &pu, const CompArea area, int dirMode, bool bSgpm = false
#if JVET_AC0094_REF_SAMPLES_OPT
                                 , bool checkWideAngle = true
#endif
  );
#else
  void initPredTimdIntraParams    (const PredictionUnit & pu, const CompArea area, int dirMode);
#endif
#if JVET_AJ0061_TIMD_MERGE
  void deriveTimdMergeModes       ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu);
  void searchTimdMrgCus           ( const CodingUnit &cu, static_vector<const CodingUnit*, NUM_TIMD_MERGE_CUS> &cuNeighbours );
  void genTimdMrgList             ( const CodingUnit &cu, const static_vector<const CodingUnit*, NUM_TIMD_MERGE_CUS> &cuNeighbours, static_vector<TimdMergeInfo, NUM_TIMD_MERGE_CUS> &timdMrgList );
  template<typename T, size_t N>
  void calcTimdMrgCandCosts       ( const CodingUnit &cu, static_vector<TimdMergeInfo, NUM_TIMD_MERGE_CUS> &timdMrgList, static_vector<T, N>& uiModeList, static_vector<uint64_t, N>& candCostList );
#endif
  void predTimdIntraAng           ( const ComponentID compId, const PredictionUnit &pu, uint32_t uiDirMode, Pel* pPred, uint32_t uiStride, uint32_t iWidth, uint32_t iHeight, TemplateType eTempType, int32_t iTemplateWidth, int32_t iTemplateHeight
#if  JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    , const bool& pdpRefAvailable = false
#endif
  );
#if JVET_AG0146_DIMD_ITMP_IBC
#if JVET_AM0074_INTRA_MERGE
  void deriveIntraMergeCandFromNeighbor(CodingUnit& cu, const CodingUnit* cuNeighbour, int& cnt, IntraMergeCandidate* tempIntraFusionCand);
  int deriveIntraMergeCand(CodingUnit &cu, IntraMergeCandidate* ddIntraFusionCand = NULL, bool isDecoder = false);
  void reorderIntraMergeCand(const CompArea &area, CodingUnit &cu, IntraMergeCandidate *ddIntraFusionCand, int &validCandSize, const bool needDerive = true);
  void generateDimdBuffersForLocDepBlending(const PredictionUnit& pu, PelBuf& piPred, PelBuf* predAngExtra, bool* blendModes, const int8_t* modeWeights, const int* modeLocDep);
  void generateEncIntraMergeBlending(PelBuf &piPred, const PredictionUnit &pu, const IntraMergeCandidate &intraMergeCandidate, PelBuf *piBlock, PelBuf& plnBlock);
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
#if JVET_AM0074_INTRA_MERGE  
  int getBestNonAnglularMode(const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu, std::vector<uint64_t>& bvCosts, std::vector<int>& bvLocDeps);
#else
  int getBestNonAnglularMode      (const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu);
#endif
#elif JVET_AM0074_INTRA_MERGE
  int getBestNonAnglularMode(const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu, std::vector<Mv> BVs, std::vector<uint64_t>& bvCosts, std::vector<int>& bvLocDeps);
#else
  int getBestNonAnglularMode      (const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu, std::vector<Mv> BVs);
#endif
#endif
#if JVET_AH0076_OBIC
  void deriveObicMode             ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu );
  void generateObicBlending       ( PelBuf &piPred, const PredictionUnit &pu, PelBuf predFusion[OBIC_FUSION_NUM - 1], bool blendModes[OBIC_FUSION_NUM - 1], int planarIdx );
#if JVET_AJ0267_ADAPTIVE_HOG
  void generateDimdBlending       ( PelBuf &piPred, const PredictionUnit &pu, PelBuf predFusion[DIMD_FUSION_NUM - 2], PelBuf &plnBlock );
#else
  void generateDimdBlending       ( PelBuf &piPred, const PredictionUnit &pu, PelBuf &piBlock0, PelBuf &piBlock1, PelBuf &piBlock2, PelBuf &piBlock3, PelBuf &plnBlock );
#endif
#endif
#if JVET_AJ0146_TIMDSAD
  int deriveTimdModeSad           (const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu);
#endif
#if JVET_AB0155_SGPM
  int deriveTimdMode              ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, bool bFull = true, bool bHorVer = false 
#if JVET_AJ0146_TIMDSAD
    , bool createList = false
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
    , bool rCalc = false, bool bUseBv = false
#endif
  );
#else
  int deriveTimdMode              ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu );
#endif
  void initTimdIntraPatternLuma   (const CodingUnit &cu, const CompArea &area, int iTemplateWidth, int iTemplateHeight, uint32_t uiRefWidth, uint32_t uiRefHeight
#if  JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    , const bool& pdpRefAvailable = false, const bool& allPDPMode = false
#endif
  );
#if GRAD_PDPC
  void xIntraPredTimdAngGradPdpc  (Pel* pDsty, const int dstStride, Pel* refMain, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, int deltaPos, int intraPredAngle, const ClpRng& clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
    , const bool bExtIntraDir
#endif
  );
#endif
#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
  void xIntraPredPlanarDcPdpc     (const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height, bool ciipPDPC);
#else
  void xIntraPredPlanarDcPdpc     ( const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height );
#endif
#endif
#endif
#if JVET_AB0155_SGPM
  void deriveSgpmModeOrdered      (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, static_vector<SgpmInfo, SGPM_NUM> &candModeList, static_vector<double, SGPM_NUM> &  candCostList);
#if JVET_AJ0112_REGRESSION_SGPM
  int deriveSgpmBlending          (PredictionUnit& pu, PelBuf &predBuf0, PelBuf &predBuf1, PelBuf &recBuf, PelBuf &adBuf, AffineBlendingModel &blendModel);
#endif
#endif
#if JVET_AL0134_SGPM_INTER
  void deriveSgpmInterModeOrdered(MergeCtx &mergeCtx, const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                  static_vector<SgpmInterInfo, SGPM_INTER_NUM> &candModeList,
                                  static_vector<double, SGPM_INTER_NUM> &candCostList, InterPrediction *pcInterPred
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    , int sgpmInterNum, uint32_t m_gpmPartTplCost[GEO_ENC_MMVD_MAX_REFINE_NUM_ADJ][GEO_MAX_ALL_INTER_UNI_CANDS][2][GEO_NUM_PARTITION_MODE] = NULL
#endif
  );
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  void xFillTimdReferenceSamples2(const CPelBuf& recoBuf, const CompArea& area, const CodingUnit& cu, int iTemplateWidth, int iTemplateHeight);
  void deriveSgpmInterIntraModeOrdered(MergeCtx& mergeCtx,
    const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu,
    static_vector<SgpmInterInfo, SGPM_INTER_NUM>& candModeList,
    static_vector<double, SGPM_INTER_NUM>& candCostList,
    InterPrediction* pcInterPred, int sgpmInterNum, IntraPrediction* pcIntraPred, uint8_t* mpmList, const int mpmNum
  );
  void deriveSgpmInterModeEncOrdered(MergeCtx& mergeCtx,
    const CPelBuf& recoBuf, const CompArea& area, CodingUnit& cu,
    static_vector<SgpmInterInfo, SGPM_INTER_NUM>& interCandModeList,
    static_vector<double, SGPM_INTER_NUM>& interCandCostList, InterPrediction* pcInterPred,
    static_vector<SgpmInterInfo, SGPM_INTER_INTRA_NUM>& intraCandModeList,
    static_vector<double, SGPM_INTER_INTRA_NUM>& intraCandCostList, 
    IntraPrediction* pcIntraPred, uint8_t* mpmList, const int mpmNum, uint32_t m_gpmPartTplCost[GEO_ENC_MMVD_MAX_REFINE_NUM_ADJ][GEO_MAX_ALL_INTER_UNI_CANDS][2][GEO_NUM_PARTITION_MODE] = NULL
  );
  void InvLutSgpmIntraTemplate(PredictionUnit& pu, Pel* src, Pel* bufTop, Pel* bufLeft, std::vector<Pel>* lut, int iTempWidth, int iTempHeight, const uint32_t uiPredStride);
  void deriveSgpmAffineModeOrdered(MergeCtx &mergeCtx, AffineMergeCtx &affMrgCtx, const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                  static_vector<SgpmInterInfo, SGPM_AFFINE_NUM> &candModeList,
                                  static_vector<double, SGPM_AFFINE_NUM> &candCostList, InterPrediction *pcInterPred, int sgpmInterNum);
#endif 

  void deriveSgpmTmInterModeOrdered(MergeCtx      &mergeCtxTmOff, MergeCtx (&mergeCtx)[GEO_NUM_TM_MV_CAND],
                                    const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                    static_vector<SgpmInterInfo, SGPM_INTER_NUM> &candModeList,
                                    static_vector<double, SGPM_INTER_NUM> &candCostList, InterPrediction *pcInterPred
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    , int sgpmInterNum
#endif
  );
#endif
#if JVET_AD0085_MPM_SORTING
  void deriveMPMSorted            (
#if JVET_AL0125_IMPROVEMENT_ON_MPM
    bool planarDisable,
#endif
    const PredictionUnit& pu, uint8_t* mpm, int& sortedSize, int iStartIdx
#if JVET_AK0061_PDP_MPM
    , const bool& pdpRefAvailable = false, const bool& allPDPMode = false
#endif
  );
#endif
#if JVET_AK0059_MDIP
  void deriveMdipSorted           (const PredictionUnit& pu, int* list, int& sortedSize, int iStartIdx);
#endif
#if JVET_AK0061_PDP_MPM
  bool determinePDPEnable(const ComponentID compId, const PredictionUnit& pu, const uint32_t  uiDirMode);
  void xFillPDPTempReferenceSamples2(const CPelBuf& recoBuf, const CompArea& area, const CodingUnit& cu);
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  void setBvMvFromMemory          (const CodingUnit& cu, const int idx, const bool useMR);
#endif
#if TMP_FAST_ENC && JVET_AD0086_ENHANCED_INTRA_TMP
  int64_t    m_tmpFlmParams[TMP_FLM_PARAMS][MTMP_NUM];
#endif
#if JVET_AB0157_TMRL
  struct TmrlInfo
  {
    uint32_t uiTemplateAbove;
    uint32_t uiTemplateLeft;
    uint32_t uiWidth;
    uint32_t uiHeight;
    uint32_t uiRefWidth;
    uint32_t uiRefHeight;

  };
  struct TmrlMode
  {
    int8_t  multiRefIdx;
    uint8_t intraDir;
    TmrlMode() : multiRefIdx(0), intraDir(0) {}
    TmrlMode(int8_t _multiRefIdx, uint8_t _intraDir) :
      multiRefIdx(_multiRefIdx), intraDir(_intraDir) {}
  };

  TmrlInfo tmrlInfo;
  TmrlMode m_tmrlList[MRL_LIST_SIZE];
  void xPredTmrlIntraDc          (const CPelBuf& pSrc, Pel* pDst, int iDstStride);
  void xPredTmrlIntraAng         (const CPelBuf& pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride);
  void predTmrlIntraAng          (const PredictionUnit& pu, Pel* pPred, uint32_t uiStride);
  void initTmrlIntraParams       (const PredictionUnit& pu, const CompArea area, const SPS& sps);
  void getTmrlSearchRange        (const PredictionUnit& pu, int8_t* tmrlRefList, uint8_t* tmrlIntraList, uint8_t& sizeRef, uint8_t& sizeMode);
  void getTmrlList               (CodingUnit& cu);
#if JVET_AJ0081_CHROMA_TMRL
  TmrlMode m_chromaTmrlList[CHROMA_TMRL_LIST_SIZE];
  void getChromaTmrlSearchRange  (const PredictionUnit& pu, int8_t* tmrlRefList, uint8_t* tmrlIntraList, uint8_t& sizeRef, uint8_t& sizeMode);
  void predChromaTmrlIntraAng    (PredictionUnit& pu, const ComponentID compID, Pel* pPred, uint32_t uiStride);
  void getChromaTmrlList         (const CPelBuf& recoBufY, const CPelBuf& recoBufCb, const CPelBuf& recoBufCr, const CompArea& areaY, const CompArea& areaCb, const CompArea& areaCr, CodingUnit& cu, PredictionUnit& pu, InterPrediction* pcInterPred);
#endif
#endif
#if JVET_AG0058_EIP
#if JVET_AL0106_BV_EIP
  void initBvEipParams           (const PredictionUnit& pu);
  void getBvEipCands             (const PredictionUnit& pu, static_vector<EipModelCandidate, MAX_BV_EIP>& candList, const ComponentID compId = COMPONENT_Y);
#endif
  void initEipParams             (const PredictionUnit& pu, const ComponentID compId);
  void eipPred                   (const PredictionUnit& pu, PelBuf& piPred, const ComponentID compId = COMPONENT_Y);
  void getCurEipCands            (const PredictionUnit& pu, static_vector<EipModelCandidate, NUM_DERIVED_EIP>& candList, const ComponentID compId = COMPONENT_Y, const bool fastTest = true); 
  int64_t (*m_calcAeipGroupSum)  (const Pel* src1, const Pel* src2, const int numSamples);
  static int64_t calcAeipGroupSum(const Pel* src1, const Pel* src2, const int numSamples);

  void getNeiEipCands            (const PredictionUnit &pu, static_vector<EipModelCandidate, MAX_MERGE_EIP> &candList, const ComponentID compId = COMPONENT_Y);
  void reorderEipCands           (const PredictionUnit &pu, static_vector<EipModelCandidate, MAX_MERGE_EIP> &candList, const ComponentID compId = COMPONENT_Y);
#if JVET_AH0086_EIP_BIAS_AND_CLIP
  void setInputsVec              (Pel *inputs, PelBuf &reco, int w, int h, int filterShape
#if JVET_AL0205_EIP_FILTER_SHAPES
    , int blockWidth, int blockHeight
#endif
  );
#endif
#if JVET_AJ0082_MM_EIP
  inline Pel getEipInputsAvg(Pel *inputs, int filterShape)
  {
    if (filterShape == 0)
    {
      return (inputs[0] + inputs[3] + 1) >> 1;
    }
    else
    {
      return inputs[1];
    }
  }
#endif
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  Reshape*          m_pcReshape;
  BilateralFilter*  m_pcBilateralFilter;

  bool checkBfLmcsApplication ( CodingStructure &cs, Reshape* reshape, ChannelType channelType );
  bool checkBfApplication( int blkQp, int blkWidth, int blkHeight );
  bool checkBfModeApplication( int modeIdx );
  void bilateralFilterPredDiamond5x5Clip(CodingStructure& cs, ChannelType channelType, PelBuf& predBuf, int blkWidth, int blkHeight, int blkQp, const ClpRng& clpRng, bool isIntraMode, CodingUnit &currCu );
  int calcRefSampleTexture(const int16_t* block, int stride, int width, int height, int whlog2);
  bool getApplyBfToPred( CodingStructure& cs, int blkWidth, int blkHeight, ChannelType channel, int blkQp, bool isIntra );
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING && JVET_Y0065_GPM_INTRA
protected:
  bool    m_abFilledIntraGPMRefTpl[NUM_INTRA_MODE];
  uint8_t m_aiGpmIntraMPMLists[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS];   //[][0][]: part0, [][1][]: part1
  Pel     m_acYuvRefGPMIntraTemplate[NUM_INTRA_MODE][2][GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];   //[][0][]: top, [][1][]: left

  template <uint8_t partIdx>
  bool xFillIntraGPMRefTemplateAll             (PredictionUnit& pu, TemplateType eTempType, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut = nullptr, uint8_t candIdx = std::numeric_limits<uint8_t>::max());
  bool xFillIntraGPMRefTemplate                (PredictionUnit& pu, TemplateType eTempType, uint8_t intraMode, bool loadIntraRef, Pel* bufTop, Pel* bufLeft, std::vector<Pel>* lut = nullptr);

public:
  uint8_t    prefillIntraGPMReferenceSamples   (PredictionUnit& pu, int iTempWidth, int iTempHeight);
  bool       fillIntraGPMRefTemplateAll        (PredictionUnit& pu, bool hasAboveTemplate, bool hasLeftTemplate, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut = nullptr, uint8_t candIdx0 = std::numeric_limits<uint8_t>::max(), uint8_t candIdx1 = std::numeric_limits<uint8_t>::max());
  void       setPrefilledIntraGPMMPMModeAll    (const uint8_t (&mpm)[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS]) {memcpy(&m_aiGpmIntraMPMLists[0][0][0], &mpm[0][0][0], sizeof(m_aiGpmIntraMPMLists));}
  uint8_t    getPrefilledIntraGPMMPMMode       (int partIdx, int splitDir, int realCandIdx) { return m_aiGpmIntraMPMLists[splitDir][partIdx][realCandIdx]; }
  Pel*       getPrefilledIntraGPMRefTemplate   (uint8_t intraMode, uint8_t templateIdx) { CHECK(intraMode >= NUM_INTRA_MODE || !m_abFilledIntraGPMRefTpl[intraMode], "Intra GPM reference template not available!"); return m_acYuvRefGPMIntraTemplate[intraMode][templateIdx]; }
  Pel*       getPrefilledIntraGPMRefTemplate   (int partIdx, int splitDir, int realCandIdx, uint8_t templateIdx) { return getPrefilledIntraGPMRefTemplate(getPrefilledIntraGPMMPMMode(partIdx, splitDir, realCandIdx), templateIdx); }
  void       clearPrefilledIntraGPMRefTemplate () {memset(&m_abFilledIntraGPMRefTpl[0], 0, sizeof(m_abFilledIntraGPMRefTpl));}
#endif
  // Angular Intra
#if JVET_AH0209_PDP
  void predIntraAng(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu,
#if JVET_AK0118_BF_FOR_INTRA_PRED
  const bool forceBfOff = false,
#endif
  const bool applyFusion = true, const bool applyPDPFilter = true);
#elif JVET_AB0157_INTRA_FUSION
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool applyFusion = true);
#else
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif
  Pel *getPredictorPtr(const ComponentID compId)
  {
    return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED];
  }

#if JVET_AC0071_DBV
  // Direct Block Vector
  void predIntraDbv                (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                  , InterPrediction *pcInterPred
#endif
  );
  Mv refineChromaBv                (const ComponentID compId, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                  , InterPrediction *pcInterPred
#endif
  );
#endif

  // Cross-component chroma
#if JVET_AD0188_CCP_MERGE
  void predIntraChromaLM           ( const ComponentID compID, PelBuf &piPred, PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel = true, CclmModel *cclmModelStored = nullptr );
#else
  void predIntraChromaLM           (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel = true, CclmModel *cclmModelStored = nullptr);
#endif
  void xGetLumaRecPixels           (const PredictionUnit &pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
#if JVET_AA0126_GLM
  void xGetLumaRecPixelsGlmAll     (const PredictionUnit &pu, CompArea chromaArea);
  Pel xGlmGetLumaVal               (const int s[6], const int c[6], const int glmIdx, const Pel val) const;
#endif
  /// set parameters from CU data for accessing intra data
  
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
#if JVET_AB0155_SGPM
  void initPredIntraParams         (const PredictionUnit &pu, const CompArea compArea, const SPS &sps, const int partIdx = 0);
#else
  void initPredIntraParams         ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );
#endif
#endif

#if JVET_AN0158_INTRA_LONGTAP
  static bool checkModeIsVer(int predMode);
  static int getIntraPredAngle(const bool bExtIntraDir, const bool isModeVer, const int predMode, const PredictionUnit& pu);
#endif 
#if JVET_AB0155_SGPM
  void initIntraPatternChType      (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false,
    const int partIdx = 0
#if JVET_AB0157_INTRA_FUSION
    , bool applyFusion = true
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
    , const bool forceDeac0 = false, const bool forceDeac1 = false
#endif
    );   // use forceRefFilterFlag to get both filtered and unfiltered buffers
#else   // SGPM
#if JVET_AB0157_INTRA_FUSION
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false, bool applyFusion = true ); // use forceRefFilterFlag to get both filtered and unfiltered buffers
#else
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers
#endif
#endif
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
  void initIntraPatternChTypeOBMC(const CodingUnit &cu, const CompArea &area);
#endif
  void initIntraPatternChTypeISP  (const CodingUnit& cu, const CompArea& area, PelBuf& piReco, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers

  // Matrix-based intra prediction
  void initIntraMip               (const PredictionUnit &pu, const CompArea &area);
#if JVET_AB0067_MIP_DIMD_LFNST
  void predIntraMip(const ComponentID compId, PelBuf& piPred, const PredictionUnit& pu, bool useDimd = false);
#else
  void predIntraMip               (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif

#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
#if JVET_AA0057_CCCM
  uint32_t xCalculateCCCMcost     (const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr);
  uint32_t xCalculateCCLMcost     (const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea  &chromaArea, const CclmModel &cclmModel);
  void     applyChromaLM          (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea &chromaArea, int intraDir, const CclmModel &cclmModel);
#endif
  void     filterPredInside       (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu);
#endif
#if JVET_AG0135_AFFINE_CIIP
  template<bool lmcs>
  void geneWeightedCIIPAffinePred(const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT = nullptr);
#endif
  template<bool lmcs>
  void geneWeightedPred           ( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT = nullptr );
  void geneIntrainterPred         (const CodingUnit &cu, PelStorage& pred);
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if JVET_AD0188_CCP_MERGE
  void geneChromaFusionPred       (const ComponentID compId, PelBuf &piPred, PredictionUnit &pu
#if JVET_AH0136_CHROMA_REORDERING
    , InterPrediction *pcInterPred = NULL
#endif
  );
#else
  void geneChromaFusionPred       (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  void geneWeightedPred           ( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred);
  void(*m_ibcCiipBlending)        ( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height );
  static void ibcCiipBlending     ( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height );
#endif
  void reorderPLT                 (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
#if !MERGE_ENC_OPT
  void geneWeightedPred           (const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf);
  Pel* getPredictorPtr2           (const ComponentID compID, uint32_t idx) { return m_yuvExt2[compID][idx]; }
  void switchBuffer               (const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst);
#endif
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
  void(*m_dimdBlending)           ( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height );
  static void dimdBlending        ( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height );
#endif
#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
  void(*m_timdBlending)           ( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height );
  static void timdBlending        ( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height );
#endif
#if JVET_V0130_INTRA_TMP
#if JVET_W0069_TMP_BOUNDARY
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  void (*m_calcTemplateDiffJointSadMrsad)  (const Pel* const ref, const unsigned int uiStride, Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean, const int licShift);
  void(*m_calcTargetMean)                  (Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean);
  static void calcTemplateDiffJointSadMrsad(const Pel* const ref, const unsigned int uiStride, Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean, const int licShift);
  static void calcTargetMean               (Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean);
#else
  void (*m_calcTemplateDiffJointSadMrsad)  (const Pel* const ref, const unsigned int uiStride, Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean);
  void(*m_calcTargetMean)                  (Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean);
  static void calcTemplateDiffJointSadMrsad(const Pel* const ref, const unsigned int uiStride, Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean);
  static void calcTargetMean               (Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean);
#endif //
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  void(*m_calcTemplateDiff)                (Pel* ref, unsigned int uiStride, Pel* tarPatch, int tarStride, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int* diff, int* iMax, RefTemplateType TempType, int requiredTemplate
#else
  void(*m_calcTemplateDiff)                (Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType TempType, int requiredTemplate
#endif
#if JVET_AG0136_INTRA_TMP_LIC
                                  , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
                                  );
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  static void calcTemplateDiff    (Pel* ref, unsigned int uiStride, Pel* tarPatch, int tarStride, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int* diff, int* iMax, RefTemplateType TempType, int requiredTemplate
#if JVET_AG0136_INTRA_TMP_LIC
                               , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
                               );
#else
  static void calcTemplateDiff    (Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                               unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType TempType,
                               int requiredTemplate
#if JVET_AG0136_INTRA_TMP_LIC
                               , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
                               );
#endif
#else
  int( *m_calcTemplateDiff )      ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
  static int calcTemplateDiff     ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
#endif
#else
  int( *m_calcTemplateDiff )      (Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax);
  static int calcTemplateDiff     ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax );
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  Pel* getTargetPatch() { return m_pppTarPatch; }
#else
  Pel** getTargetPatch            ( unsigned int uiDepth )      { return m_pppTarPatch[uiDepth]; }
#endif
  Pel* getRefPicUsed              ()                            { return m_refPicUsed;           }
  void setRefPicUsed              ( Pel* ref )                  { m_refPicUsed = ref;            }
  unsigned int getStride          ()                            { return m_uiPicStride;          }
  void         setStride          ( unsigned int uiPicStride )  { m_uiPicStride = uiPicStride;   }

#if JVET_W0069_TMP_BOUNDARY
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  RefTemplateType getRefTemplateType ( CodingUnit& cu, CompArea& area );
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  void searchCandidateFromOnePicIntra(CodingUnit* pcCU, Pel* tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, RefTemplateType tempType
#else
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, RefTemplateType tempType
#endif
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
                                      , const bool bJointCalc
#endif
                                      );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
                                      , const bool bJointCalc
#endif
                                      );
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  void xPadForFracSearchInterpolation (CodingUnit* pcCU, RefTemplateType tempType);
  void xTmpFracSearchIF               (PredictionUnit& pu, Pel* padbf0, unsigned int padStride, Pel* preTmpbf0, unsigned int predTempStride, Pel* tmp0, unsigned int tmpStride, int extUiWidth, int extUiHeight
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                       , const int fracPrecX, const int fracPrecY
#else
                                       , int fracPrec
#endif
                                       , int fracDir);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  void searchFracCandidate            (CodingUnit* pcCU, Pel* tarPatch, RefTemplateType tempType
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                       , const bool isCalledFromEncSearch
#endif
                                       );
#else
  void searchFracCandidate            ( CodingUnit* pcCU, Pel** tarPatch, RefTemplateType tempType);
#endif
  InterPrediction *m_pcInterPred;
  void setInterPrediction             ( InterPrediction *inter);
#endif
#else
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  void convertDiff2Weight            (int *pDiff, int *weights, const int start, const int foundCandiNum);
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  int  xCalTMPFusionNumber           (const int maxNum, const int numIdx
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
                                      );
#endif
  void xTMPBuildFusionCandidate      (CodingUnit &cu, RefTemplateType tempType
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
                                      );

  void xCalcTmpFlmRefArea            (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, bool& leftPadding, bool& rightPadding, bool& abovePadding, bool& belowPadding);
  void xGetTmpFlmRefBuf              (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType);
  void xCalTmpFlmParam               (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType);
  void xGenerateTmpFlmPred           (PelBuf& piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                      , RefTemplateType tempType
#endif
                                      , CodingUnit* pcCU, bool bDeriveDimdMode = true);

  void xTMPFusionCalcParams          (CodingUnit* cu, CompArea area, CccmModel& tmpFusionModel, int foundCandiNum, RefTemplateType tempType, Pel* curPointTemplate, Pel* refPointTemplate[]
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                      , const int* const ptrLicParamsFusion[TMP_FUSION_NUM]
#else
                                      , const int* const ptrLicParamsFusion[TMP_BEST_CANDIDATES]
#endif
#endif
                                      );
  void xTMPFusionCalcModels          (CodingUnit* cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
                                      );
  void xTMPFusionApplyModel          (PelBuf& piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, CodingUnit* pcCU
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
                                      , bool bDeriveDimdMode = true);

#if !JVET_AH0200_INTRA_TMP_BV_REORDER
  void xPadForInterpolation          (CodingUnit* pcCU);
#endif
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, int& foundCandiNum, PredictionUnit& pu
#if JVET_AG0136_INTRA_TMP_LIC
                                      , const bool useMR
#endif
                                      , bool bDeriveDimdMode = true);
#endif

#elif TMP_FAST_ENC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, CompArea area, int& foundCandiNum, CodingUnit* cu);
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, int& foundCandiNum, PredictionUnit& pu);
#endif
#else
  bool generateTMPrediction          ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int& foundCandiNum );
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel *piPred, unsigned int uiStride, int &foundCandiNum, PredictionUnit &pu);
#endif
#endif
#if JVET_W0069_TMP_BOUNDARY
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
  bool generateTmDcPrediction        (Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val, CodingUnit* cu);
#else
  bool generateTmDcPrediction        ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val );
#endif
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType );
#else
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  void initTmpDisp()
  {
    for (int i = 0; i < MTMP_NUM; i++)
    {
      m_tmpXdisp[i] = 0;
      m_tmpYdisp[i] = 0;
#if JVET_AG0136_INTRA_TMP_LIC
      m_tmpXdispUseMR[i] = 0;
      m_tmpYdispUseMR[i] = 0;
#endif
    }
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
    m_itmpBvBasedMergeCandidates.clear();
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
    m_timdBvBasedMergeCandidates.clear();
#endif
    m_dimdBvBasedMergeCandidates.clear();
    m_sgpmMvBasedMergeCandidates.clear();
    itmpSize = 0; itmpSize1 = 0;
#else
    m_bvBasedMergeCandidates.clear();
#endif
#endif
#if JVET_AM0074_INTRA_MERGE
    m_bvBasedMergeCostCandidates.clear();
    m_bvBasedMergeLocDepCandidates.clear();
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
    m_sgpmMvBasedMergeCandidates.clear();
#endif
#endif
  }
  void initTmpFlmParams()
  {
    for (int j = 0; j < MTMP_NUM; j++)
    {
      for (int i = 0; i < TMP_FLM_PARAMS; i++)
      {
        m_tmpFlmParams[i][j] = -1;
      }
    }
  }
  void initTmpFusionInfo()
  {
    for (int i = 0; i < 
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                        TMP_FUS_MODES_NUM
#else
                        TMP_GROUP_IDX << 1
#endif
                        ; i++)
    {
      m_tmpFusionInfo[i] = IntraTMPFusionInfo{ false, false, 0
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                              , TMP_FUSION_NUM, MAX_INT, MAX_INT, {MAX_INT, MAX_INT, MAX_INT}
#else
                                              , 1
#endif
                                              };
#if JVET_AG0136_INTRA_TMP_LIC
      m_tmpFusionInfoUseMR[i] = IntraTMPFusionInfo{ false, false, 0
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                                   , TMP_FUSION_NUM, MAX_INT, MAX_INT, {MAX_INT, MAX_INT, MAX_INT}
#else
                                                   , 1
#endif
                                                   };
#endif
    }
  }
#elif TMP_FAST_ENC
  void initTmpDisp()
  {
    m_tmpXdisp = 0;
    m_tmpYdisp = 0;
  }
#endif
#if TMP_FAST_ENC
#if JVET_AG0136_INTRA_TMP_LIC
  int getTmpNumCand() const { return m_tmpNumCand; }
  int getTmpNumCandUseMR() const { return m_tmpNumCandUseMR; }
#else
  int getTmpNumCand() { return m_tmpNumCand; }
#endif
#endif

#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  int calcHorCollocatedChroma(PelStorage* pcPicYuvOrg, int bitDepth, RdCost* pcRdCost);
  bool deriveCclmParams(Area& areaY, Area& areaC, PelStorage* pcPicYuvOrg, MMLMParameters param[NUM_HORCOLCHROMA][2]);
  Distortion calcCostColChroma(Area& areaC, PelStorage* pcPicYuvOrg, int col, MMLMParameters param[2], RdCost* pcRdCost, const ClpRng& clpRng);
#endif

#ifdef TARGET_SIMD_X86
  void    initIntraX86();
  template <X86_VEXT vext>
  void    _initIntraX86();
#endif
};
//! \}

#if JVET_W0123_TIMD_FUSION && JVET_AG0092_ENHANCED_TIMD_FUSION
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
void xLocationdepBlending(Pel* pDst, const int strideDst, const Pel* pVer, const int strideVer, const Pel* pHor, const int strideHor, const Pel* pNonLocDep, const int strideNonLocDep, const int width, const int height, const int mode, const int wVer, const int wHor, const int wNonLocDep, const int range=10);
#else
void xLocationdepBlending    (Pel *pDst, int strideDst, Pel *pVer, int strideVer, Pel *pHor, int strideHor,Pel *pNonLocDep, int strideNonLocDep, int width, int height, int mode, int wVer, int wHor, int wNonLocDep, int range = 10);
#endif
#else
#if ENABLE_DIMD
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pVer, int strideVer, Pel *pHor, int strideHor,Pel *pNonLocDep, int strideNonLocDep, int width, int height, int mode, int wVer, int wHor, int wNonLocDep);
#else
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pMainAng, int strideMainAng, Pel *pSecondAng, int strideSecondAng,Pel *pPlanar, int stridePlanar, int width, int height, int sideMain, int sideSecond, int wMain, int wSecond, int wPlanar);
#endif
#endif
#endif
#endif
#endif // __INTRAPREDICTION__
