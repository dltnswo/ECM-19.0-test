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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/ParameterSetManager.h"

#include "CABACWriter.h"
#include "EncCfg.h"
#if JVET_AK0065_TALF
#include "RdCost.h"

struct CandTALF
{
  double                       rate;
  TAlfControl                  talfControl;
  std::vector<TAlfCtbParam>    talfCtbParam;
  std::vector<TAlfFilterParam> params;
  
  CandTALF()
  {
    rate = 0;
    talfControl.reset();
    talfCtbParam.clear();
    params.clear();
  }
};
#endif

#define ALF_PRECISION_VARIETY                             (JVET_AG0158_ALF_LUMA_COEFF_PRECISION || JVET_AK0123_ALF_COEFF_RESTRICTION)
#define ALF_RD_COST_BUG_FIX                               JVET_AK0123_ALF_COEFF_RESTRICTION

struct AlfCovariance
{
  static constexpr int MaxAlfNumClippingValues = AdaptiveLoopFilter::MaxAlfNumClippingValues;

#if JVET_X0071_LONGER_CCALF && !JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && !JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AF0177_ALF_COV_FLOAT
  using TE = float[MAX_NUM_CC_ALF_CHROMA_COEFF][MAX_NUM_CC_ALF_CHROMA_COEFF];
  using Ty = float[MAX_NUM_CC_ALF_CHROMA_COEFF];
#else
  using TE = double[MAX_NUM_CC_ALF_CHROMA_COEFF][MAX_NUM_CC_ALF_CHROMA_COEFF];
  using Ty = double[MAX_NUM_CC_ALF_CHROMA_COEFF];
#endif
#else
#if JVET_AF0177_ALF_COV_FLOAT
  using TE = float[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = float[MAX_NUM_ALF_LUMA_COEFF];
  using DTy = double[MAX_NUM_ALF_LUMA_COEFF];
#else
  using TE = double[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = double[MAX_NUM_ALF_LUMA_COEFF];
#endif
#endif

#if JVET_AF0177_ALF_COV_FLOAT
  std::vector<float> data;
#else
  std::vector<double> data;
#endif

  ptrdiff_t                offsetE;
  ptrdiff_t                stridey;

  size_t sizeY() const { return offsetE; }
  size_t sizeE() const { return data.size() - offsetE; }

  ptrdiff_t getOffsetY(ptrdiff_t i, ptrdiff_t j) const { return stridey * i + j; }

  ptrdiff_t getOffsetEfast(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const
  {
    const ptrdiff_t v0 = getOffsetY(i, k);
    const ptrdiff_t v1 = getOffsetY(j, l);

    CHECKD( v1 > v0, "Wrong v1");

    return offsetE + (v0 * (v0 + 1) >> 1) + v1;
  }

  ptrdiff_t getOffsetE(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const
  {
    const ptrdiff_t v0 = getOffsetY(i, k);
    const ptrdiff_t v1 = getOffsetY(j, l);

    return offsetE + (v1 <= v0 ? (v0 * (v0 + 1) >> 1) + v1 : (v1 * (v1 + 1) >> 1) + v0);
  }
#if JVET_AF0177_ALF_COV_FLOAT
  float       &y(ptrdiff_t i, ptrdiff_t j) { return data[getOffsetY(i, j)]; }
  const float &y(ptrdiff_t i, ptrdiff_t j) const { return data[getOffsetY(i, j)]; }

  float       &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) { return data[getOffsetE(i, j, k, l)]; }
  const float &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const { return data[getOffsetE(i, j, k, l)]; }
#else
  double       &y(ptrdiff_t i, ptrdiff_t j) { return data[getOffsetY(i, j)]; }
  const double &y(ptrdiff_t i, ptrdiff_t j) const { return data[getOffsetY(i, j)]; }

  double       &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) { return data[getOffsetE(i, j, k, l)]; }
  const double &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const { return data[getOffsetE(i, j, k, l)]; }
#endif
  int numCoeff;
  int numBins;
#if JVET_AF0177_ALF_COV_FLOAT
  float pixAcc;
#else
  double pixAcc;
#endif

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create(int size, int _numBins)
  {
    numCoeff = size;
    numBins  = _numBins;
    const int numCols = numCoeff * numBins;
    stridey    = numCoeff;
    offsetE    = stridey * numBins;
    data.resize(offsetE + (numCols * (numCols + 1) >> 1));
#if JVET_AF0177_ALF_COV_FLOAT
    std::fill(data.begin(), data.end(), 0.0f);
#else
    std::fill(data.begin(), data.end(), 0.0);
#endif
  }

  void destroy()
  {
  }

  void reset()
  {
    pixAcc = 0;
#if JVET_AF0177_ALF_COV_FLOAT
    std::fill(data.begin(), data.end(), 0.0f);
#else
    std::fill(data.begin(), data.end(), 0.0);
#endif
  }

  void reset(int _numBins)
  {
    numBins = _numBins;
    reset();
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    numCoeff = src.numCoeff;
    numBins = src.numBins;
    data     = src.data;
    stridey  = src.stridey;
    offsetE  = src.offsetE;
    pixAcc = src.pixAcc;

    return *this;
  }

  bool sameSizeAs(const AlfCovariance &x) { return x.numCoeff == numCoeff && x.numBins == numBins; }

  void add( const AlfCovariance& lhs, const AlfCovariance& rhs )
  {
    numCoeff = lhs.numCoeff;
    numBins = lhs.numBins;
    CHECK(!sameSizeAs(lhs), "AlfCovariance size mismatch");
    CHECK(!sameSizeAs(rhs), "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < data.size(); i++)
    {
      data[i] = lhs.data[i] + rhs.data[i];
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src )
  {
    CHECK(src.numCoeff != numCoeff, "AlfCovariance size mismatch");
    // There may be a case where we accumulate only bin #0
    CHECK(src.numBins != numBins && numBins != 1, "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < sizeY(); i++)
    {
      data[i] += src.data[i];
    }

    if (src.numBins != numBins)
    {
      // numBins == 1 (see CHECK above)
      for (ptrdiff_t i = 0; i < numCoeff; i++)
      {
        for (ptrdiff_t j = 0; j <= i; j++)
        {
          data[getOffsetEfast(0, 0, i, j)] += src.data[src.getOffsetEfast(0, 0, i, j)];
        }
      }
    }
    else
    {
      for (ptrdiff_t i = 0; i < sizeE(); i++)
      {
        data[offsetE + i] += src.data[src.offsetE + i];
      }
    }

    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= ( const AlfCovariance& src )
  {
    CHECK(src.numCoeff != numCoeff, "AlfCovariance size mismatch");
    // There may be a case where we accumulate only bin #0
    CHECK(src.numBins != numBins && numBins != 1, "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < sizeY(); i++)
    {
      data[i] -= src.data[i];
    }
    if (src.numBins != numBins)
    {
      // numBins == 1 (see CHECK above)
      for (ptrdiff_t i = 0; i < numCoeff; i++)
      {
        for (ptrdiff_t j = 0; j <= i; j++)
        {
          data[getOffsetEfast(0, 0, i, j)] -= src.data[src.getOffsetEfast(0, 0, i, j)];
        }
      }
    }
    else
    {
      for (ptrdiff_t i = 0; i < sizeE(); i++)
      {
        data[offsetE + i] -= src.data[src.offsetE + i];
      }
    }

    pixAcc -= src.pixAcc;

    return *this;
  }

  void setEyFromClip(const int* clip, TE _E, Ty _y, int size) const
  {
    CHECK(size != numCoeff, "AlfCovariance size mismatch");

    for (ptrdiff_t k = 0; k < size; k++)
    {
      _y[k] = y(clip[k], k);
      // Upper triangular
      for (ptrdiff_t l = k; l < size; l++)
      {
        _E[k][l] = E(clip[k], clip[l], k, l);
      }
    }
  }
#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
  double optimizeFilter(const int* clip, Ty f, int size, bool encAlfResidualCheck = false) const
#else
  double optimizeFilter(const int* clip, Ty f, int size) const
#endif
#else
  double optimizeFilter(const int* clip, double *f, int size) const
#endif
  {
    gnsSolveByChol( clip, f, size 
#if JVET_AN0086_RESIDUAL_CHECK
    , encAlfResidualCheck
#endif
    );
    return calculateError( clip, f );
  }

#if JVET_AD0222_ALF_RESI_CLASS
#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
  double optimizeFilter(const AlfFilterShape& alfShape, int* clip, Ty f, bool optimize_clip, bool enableLessClip, bool encAlfResidualCheck = false) const;
#else
  double optimizeFilter(const AlfFilterShape& alfShape, int* clip, Ty f, bool optimize_clip, bool enableLessClip) const;
#endif
#else
  double optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip, bool enableLessClip) const;
#endif
#if JVET_AN0086_RESIDUAL_CHECK
  double optimizeFilterClip(const AlfFilterShape& alfShape, int* clip, bool enableLessClip, bool encAlfResidualCheck = false) const
  {
    Ty f;
    return optimizeFilter(alfShape, clip, f, true, enableLessClip, encAlfResidualCheck);
  }
#else
  double optimizeFilterClip(const AlfFilterShape& alfShape, int* clip, bool enableLessClip) const
  {
    Ty f;
    return optimizeFilter(alfShape, clip, f, true, enableLessClip);
  }
#endif
#else 
  double optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip) const;
  double optimizeFilterClip(const AlfFilterShape& alfShape, int* clip) const
  {
    Ty f;
    return optimizeFilter(alfShape, clip, f, true);
  }
#endif

#if JVET_AN0086_RESIDUAL_CHECK
  double calculateError( const int *clip, bool encAlfResidualCheck = false ) const;
#else
  double calculateError( const int *clip ) const;
#endif
#if JVET_AF0177_ALF_COV_FLOAT
  double calculateError( const int *clip, const Ty coeff ) const { return calculateError(clip, coeff, numCoeff); }
  double calculateError( const int *clip, const Ty coeff, const int numCoeff ) const;
#else
  double calculateError( const int *clip, const double *coeff ) const { return calculateError(clip, coeff, numCoeff); }
  double calculateError( const int *clip, const double *coeff, const int numCoeff ) const;
#endif
#if ALF_PRECISION_VARIETY
  void calcInitErrorForCoeffs(double *cAc, double *cA, double *bc, const int *clip, const int *coeff, const int numCoeff, const int bitDepth, const int scaleIdx) const;
#if JVET_AK0065_TALF
  void calcInitErrorForTAlfCoeffs(double* cAc, double* cA, double* bc, const int* clip, const int* coeff, const int numCoeff, const int bitDepth) const;
#endif
  void updateErrorForCoeffsDelta(double *cAc, double *cA, double *bc, const int *clip, const int *coeff, const int numCoeff, const int bitDepth, double delta, int modInd) const;
  double calcErrorForCoeffsDelta(double cAc, double *cA, double bc, const int *clip, const int *coeff, const int numCoeff, const int bitDepth, double delta, int modInd) const;
#endif
  double calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int bitDepth, const int scaleIdx) const;
  double calcErrorForCcAlfCoeffs(const int16_t *coeff, const int numCoeff, const int bitDepth) const;
#if JVET_AK0065_TALF
  double calcErrorForTAlfCoeffs(const int* clip, const int* coeff, const int numCoeff, const int bitDepth) const;
#endif

  void getClipMax(const AlfFilterShape& alfShape, int *clip_max) const;
  void reduceClipCost(const AlfFilterShape& alfShape, int *clip) const;
#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
  int  gnsSolveByChol( TE LHS, Ty rhs, Ty x, int numEq, bool encAlfResidualCheck = false ) const;
#else
  int  gnsSolveByChol( TE LHS, Ty rhs, Ty x, int numEq ) const;
#endif
#else
  int  gnsSolveByChol( TE LHS, double* rhs, double *x, int numEq ) const;
#endif
private:
  // Cholesky decomposition
#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
  int  gnsSolveByChol( const int *clip, Ty x, int numEq, bool encAlfResidualCheck = false) const;
#else
  int  gnsSolveByChol( const int *clip, Ty x, int numEq ) const;
#endif
  void gnsTransposeBacksubstitution( TE U, Ty rhs, Ty x, int order ) const;
  void gnsBacksubstitution( TE R, Ty z, int size, Ty A ) const;
#else
  int  gnsSolveByChol( const int *clip, double *x, int numEq ) const;
  void gnsBacksubstitution( TE R, double* z, int size, double* A ) const;
  void gnsTransposeBacksubstitution( TE U, double* rhs, double* x, int order ) const;
#endif
  int  gnsCholeskyDec( TE inpMatr, TE outMatr, int numEq ) const;
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  inline void           setAlfWSSD(int alfWSSD) { m_alfWSSD = alfWSSD; }
  static std::vector<double>  m_lumaLevelToWeightPLUT;
  inline std::vector<double>& getLumaLevelWeightTable() { return m_lumaLevelToWeightPLUT; }

private:
  int                    m_alfWSSD;
  const EncCfg*          m_encCfg;
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
  AlfCovariance*****       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][fixedFilterSetIdx][classifierInd][classIdx]
#else
  AlfCovariance****       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][fixedFilterSetIdx][classIdx]
#endif
#else
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
#endif
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CHANNEL_TYPE];   // [CHANNEL][shapeIdx][lumaClassIdx/chromaAltIdx]
  uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];
  uint8_t*               m_ctuEnableFlagTmp2[MAX_NUM_COMPONENT];
  uint8_t*               m_ctuAlternativeTmp[MAX_NUM_COMPONENT];
  AlfCovariance**        m_alfCovarianceCcAlf;           // [shapeIdx][filterIdx][ctbAddr]
  AlfCovariance*         m_alfCovarianceFrameCcAlf;      // [shapeIdx][filterIdx]

#if ALF_IMPROVEMENT
  bool classChanged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
  int clipHistory[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  double errorHistory[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
#endif

  //for RDO
  AlfParam               m_alfParamTemp;
  ParameterSetMap<APS>*  m_apsMap;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 2];
#if ALF_PRECISION_VARIETY
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  static const int       m_numBitPrecision = 4;
#else
  static const int       m_numBitPrecision = 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  static const int       m_numBitMantissa = 2;
#else
  static const int       m_numBitMantissa = 1;
#endif
  static constexpr int   m_alfPrecisionVariety = m_numBitPrecision * m_numBitMantissa;
  int                    m_alfClipMerged[ALF_NUM_OF_FILTER_TYPES][m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
#else
  int                    m_alfClipMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
#endif
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_ctxCache;
  double                 m_lambda[MAX_NUM_COMPONENT];

  int**                  m_filterCoeffSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int**                  m_filterClippSet; // [lumaClassIdx/chromaAltIdx][coeffIdx]
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char*                  m_filterScaleIdx; // [lumaClassIdx/chromaAltIdx]
#endif
  int**                  m_diffFilterCoeff;
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];
  unsigned               m_bitsNewFilter[MAX_NUM_CHANNEL_TYPE];
  int&                   m_apsIdStart;
#if JVET_AK0065_TALF
  int&                   m_apsIdStart2;
#endif
  double                 *m_ctbDistortionUnfilter[MAX_NUM_COMPONENT];
  double                 m_unFiltDistCompnent[MAX_NUM_COMPONENT];
#if ALF_IMPROVEMENT
  double                 *m_distCtbApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][2];
  double                 *m_distCtbLumaNewFilt[MAX_NUM_ALF_ALTERNATIVES_LUMA][2];
  double                 *m_ctbDistortionFixedFilter[NUM_FIXED_FILTER_SETS][2];
#else
  double                 *m_distCtbApsLuma[ALF_CTB_MAX_NUM_APS];
  double                 *m_distCtbLumaNewFilt;
  double                 *m_ctbDistortionFixedFilter[NUM_FIXED_FILTER_SETS];
  int                    m_clipDefaultEnc[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if JVET_AD0222_ALF_RESI_CLASS
  bool                   m_enableLessClip;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  bool                   m_alfPrecisonFlag;
#endif
  std::vector<short>     m_alfCtbFilterSetIndexTmp;
  AlfParam               m_alfParamTempNL;
  int                    m_filterTmp[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_clipTmp[MAX_NUM_ALF_LUMA_COEFF];

  int m_apsIdCcAlfStart[2];

  short                  m_bestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  bool                   m_bestFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
  uint8_t                m_bestFilterCount;
  uint8_t*               m_trainingCovControl;
  Pel*                   m_bufOrigin;
  PelBuf*                m_buf;
  uint64_t*              m_trainingDistortion[MAX_NUM_CC_ALF_FILTERS];    // for current block size
  uint64_t*              m_lumaSwingGreaterThanThresholdCount;
  uint64_t*              m_chromaSampleCountNearMidPoint;
  uint8_t*               m_filterControl;         // current iterations filter control
  uint8_t*               m_bestFilterControl;     // best saved filter control
  int                    m_reuseApsId[2];
  bool                   m_limitCcAlf;
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  bool                   m_isLumaSignalNewAps;
  bool                   m_isLowDelayConfig;
  double                 m_chromaFactor;
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  bool                   m_reuseCondition;
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  ScaleAlfEnc            m_scaleAlfEncParam[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif

public:
#if JVET_AK0065_TALF
  EncAdaptiveLoopFilter( int& apsIdStart, int& apsIdStart2 );
#else
  EncAdaptiveLoopFilter( int& apsIdStart );
#endif
  virtual ~EncAdaptiveLoopFilter() {}

  template<bool alfWSSD>
  void  initDistortion(
#if ALF_IMPROVEMENT
    CodingStructure& cs
#endif
  );
  std::vector<int> getAvaiApsIdsLuma(CodingStructure& cs, int &newApsId);
  void  alfEncoderCtb(CodingStructure& cs, AlfParam& alfParamNewFilters
#if ENABLE_QPA
    , const double lambdaChromaWeight
#endif
  );
  void   alfReconstructor(CodingStructure& cs, const PelUnitBuf& recExtBuf);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  ScaleAlfEnc* getAlfScaleEncPtr( const int apsId, const int altNum ) { return &m_scaleAlfEncParam[apsId][altNum]; }
  ScaleAlfEnc& getAlfScaleEnc( const int apsId, const int altNum ) { return m_scaleAlfEncParam[apsId][altNum]; }
  void         alfCorrection( CodingStructure& cs, const PelUnitBuf& origBuf, const PelUnitBuf& recExtBuf, bool mode=false );
  void         alfCorrectionChroma( CodingStructure& cs, PelUnitBuf& recYuvSao );
#endif
#if JVET_AL0153_ALF_CCCM
  PelUnitBuf   *m_newOrgBuf;
  void         setNewOrgBuf(PelUnitBuf *newOrgBuf) { m_newOrgBuf = newOrgBuf; }
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  bool calcOffsetRefinementOnOff( CodingStructure& cs, PelUnitBuf& src0, PelUnitBuf& src1, PelUnitBuf& src2, int& refineIdx );
#endif

  void ALFProcess(CodingStructure& cs, const double *lambdas
#if ENABLE_QPA
    , const double lambdaChromaWeight
#endif
    , Picture* pcPic, uint32_t numSliceSegments
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    , const int intraPreiod
#endif
  );

  void getDistApsFilter( CodingStructure& cs, std::vector<int> apsIds );
  void getDistNewFilter( AlfParam& alfParam );
  int getNewCcAlfApsId( CodingStructure &cs, int cIdx );
  void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice, ParameterSetMap<APS>* apsMap );
  void create( const EncCfg* encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE], bool createEncData = false );
  void destroy( bool destroyEncData = false );
  void setApsIdStart( int i) { m_apsIdStart = i; }
#if JVET_AK0065_TALF
  void setApsIdStart2(int i) { m_apsIdStart2 = i; }
#endif
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  static int lengthHuffman(int coeffVal, HuffmanForALF& huffman);
#endif
  static int lengthGolomb(int coeffVal, int k, bool signed_coeff = true);
#endif

private:
  void   alfEncoder( CodingStructure& cs, AlfParam& alfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                   , const double lambdaChromaWeight = 0.0
#endif
                   );

  void   copyAlfParam( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel );
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if ALF_PRECISION_VARIETY
  double mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits, int altIdx, int classifierIdx, int numFiltersLinear, bool tryImproveBySA);
#else
  double mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits, int altIdx, int classifierIdx, int numFiltersLinear, bool tryImproveBySA);
#endif
  void   getFrameStats( ChannelType channel, int iShapeIdx, int altIdx, int fixedFilterSetIdx, int classifierIdx );
#else
  double mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits
#if ALF_IMPROVEMENT
    , int altIdx
#endif
    , bool tryImproveBySA
  );
  void   getFrameStats( ChannelType channel, int iShapeIdx, int altIdx 
#if ALF_IMPROVEMENT
    , int fixedFilterSetIdx
#endif
  );
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance**** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx, int fixedFilterSetIdx, int classifierIdx );
#else
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance*** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx, int fixedFilterSetIdx);
#endif
#else
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx );
#endif
  template<bool alfWSSD>
  void   deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv, CodingStructure& cs );

#if ALF_IMPROVEMENT
#if JVET_AD0222_ALF_RESI_CLASS
  template<bool m_alfWSSD, bool reuse>
#else
  template<bool m_alfWSSD>
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void   getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, const Pel* org, const int orgStride, const Pel* rec, const int recStride,
#if JVET_AD0222_ALF_RESI_CLASS
    AlfClassifier** classifierNext, AlfCovariance* alfCovarianceNext,
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    const Pel *resi, const int resiStride,
#endif
    const CompArea& areaDst, const CompArea& area, const ChannelType channel, int fixedFilterSetIdx, int classifierIdx);
#else
  void   getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, const Pel* org, const int orgStride, const Pel* rec, const int recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
    const CompArea& areaDst, const CompArea& area, const ChannelType channel, int fixedFilterSetIdx);
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  void   calcCovariance(Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    const Pel *resi, const int resiStride,
#endif
    const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, Pel ***fixedFitlerResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    Pel ***fixedFitlerResiResults,
#endif
    Position posDst, Position pos, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Position posInCtu
#endif
    );
#else  
  void   calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
    const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, Pel ***fixedFitlerResults, Position pos, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Position posInCtu
#endif
    );
#endif
#else
  void   getBlkStats(AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  void   calcCovariance(Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance);
#else
  void   calcCovariance(int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance);
#endif
#endif
  template<bool alfWSSD>
  void   deriveStatsForCcAlfFiltering(const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const int compIdx, CodingStructure &cs);
  template<bool m_alfWSSD>
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  void   getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape, const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area, const ComponentID compID, const int yPos, PelUnitBuf &resiYuv 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , PelUnitBuf& recYuvSAO
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    , PelUnitBuf& recBeforeDbf
#endif
  );
#else
  void   getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape, const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area, const ComponentID compID, const int yPos
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , PelUnitBuf& recYuvSAO
#endif
  );
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if ALF_IMPROVEMENT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  void   calcCovarianceCcAlf( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape, Pel* resiPtr, int resiStride  
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , Pel* recSAO, int saoStride
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
   , Pel* crossChromaPtr, const int crossChromaStride
#endif
  );
#else
  void   calcCovarianceCcAlf( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , Pel* recSAO, int saoStride
#endif
  );
#endif
#else
  void   calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance);
#endif
#else
#if ALF_IMPROVEMENT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  void   calcCovarianceCcAlf( int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape. Pel* resiPtr, int resiStride 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , Pel* recSAO, int saoStride
#endif
  );
#else
  void   calcCovarianceCcAlf( int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , Pel* recSAO, int saoStride
#endif
  );
#endif
#else
  void   calcCovarianceCcAlf(int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance);
#endif
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
   void   mergeClasses(const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES], const int altIdx, int mergedPair[MAX_NUM_ALF_CLASSES][2]);
#else
  void   mergeClasses(const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]
#if ALF_IMPROVEMENT
    , const int altIdx
#endif
  );
#endif

  double getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, 
#if ALF_IMPROVEMENT
    int fixedFilterSetIdx,
#endif
    bool tryImproveBySA, bool onlyFilterCost = false );
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if ALF_PRECISION_VARIETY
  double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam, bool nonLinear, int classifierIdx, bool isMaxNum, int mergedPair[MAX_NUM_ALF_CLASSES][2], int mergedCoeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], char mergedScaleIdx[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES], double mergedErr[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES], bool tryImproveScale);
#else
  double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam, bool nonLinear, int classifierIdx, bool isMaxNum, int mergedPair[MAX_NUM_ALF_CLASSES][2], int mergedCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], char mergedScaleIdx[MAX_NUM_ALF_CLASSES], double mergedErr[MAX_NUM_ALF_CLASSES], bool tryImproveScale);
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
#if ALF_PRECISION_VARIETY
  double tryImproveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, bool nonLinear, int classifierIdx, int bitsNum, int mantissa, double lambda);
#else
  double tryImproveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, bool nonLinear, int classifierIdx, double lambda);
#endif
#endif
#else
  double deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam
#if ALF_IMPROVEMENT
  , bool nonLinear
#endif
  );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  double tryImproveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters,
#if ALF_IMPROVEMENT
    , bool nonLinear
#endif
    , double lambda
  );
#endif
#endif
#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
  void deriveCoeffQuantMultipleBitDepths( int *filterClipp, const AlfCovariance& cov, const AlfFilterShape& shape, const bool optimizeClip, int filtIdx, double errorTabForce0Coeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][2], double lambda, bool tryImproveScale );
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, bool nonLinearFlag, char bitIdx, bool isLuma, char bitNum, char mantissa);
#else
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, bool nonLinearFlag );
#endif
#else
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters );
#endif
  double deriveCoeffQuant( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip, const bool isLuma, const int mantissa, double lambda, char& scaleIdxRef, bool tryImproveScale);
#if JVET_AK0065_TALF
  double deriveCoeffForTAlfQuant(int* filterClipp, int* filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip);
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  double tryImproveCoeffQuant(int* filterClipp, int* filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip, const bool isLuma, const int mantissa, double lambda, char& scaleIdxRef);
#endif
  double deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                  const double chromaWeight,
#endif
                                  const int numClasses, const int numCoeff, double& distUnfilter 
#if ALF_IMPROVEMENT
                                , int fixedFilterSetIdx
#endif
    , bool* flagChanged);
#if JVET_AF0177_ALF_COV_FLOAT
  void   roundFiltCoeff( int *filterCoeffQuant, float *filterCoeff, const int numCoeff, const int factor );
  void   roundFiltCoeffCCALF( int16_t *filterCoeffQuant, float *filterCoeff, const int numCoeff, const int factor );
#else
  void   roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );
  void   roundFiltCoeffCCALF(int16_t *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor);
#endif
  double getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, int zeroBitsVarBin, const int numFilters);
  int    lengthUvlc( int uiCode );
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
  int    getNonFilterCoeffRate( AlfParam& alfParam, int altIdx, int classifierIdx );
#else
  int    getNonFilterCoeffRate( AlfParam& alfParam, int altIdx );
#endif
#if ALF_PRECISION_VARIETY
  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins, int altIdx, char bitIdx, bool isLuma, char bitNum, char mantissa);
  double getDistForce0(AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins, int altIdx, char bitIdx, bool isLuma, char bitNum, char mantissa);
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff, int altIdx, int coeffBits );
#else
  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins, int altIdx);
  double getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins, int altIdx);
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff, int altIdx );
#endif
  void initCtuAlternativeLuma( uint8_t* ctuAlts[MAX_NUM_COMPONENT] );
#else
  int    getNonFilterCoeffRate( AlfParam& alfParam );
  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  double getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff );
#endif
#if ALF_PRECISION_VARIETY
  int    getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, char bitIdx, bool isLuma, char bitNum, char mantissa);
  int    getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, char bitIdx );
  int    lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, char bitIdx, bool isLuma, char bitNum, char mantissa);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  int    lengthFilterCoeffsOneFilter(const AlfFilterShape& alfShape, int* FilterCoeff, bool isLuma, char bitNum, char mantissa);
#endif
#else
  int    getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff ); 
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  int    lengthFilterCoeffsOneFilter(const AlfFilterShape& alfShape, int* FilterCoeff);
#endif
#endif
  int    getChromaCoeffRate( AlfParam& alfParam, int altIdx );
  double getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel );
  double getUnfilteredDistortion( AlfCovariance* cov, const int numClasses );

  void setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, bool val );
  void setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags );
  void setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val );
  void copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel );
  void initCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT] );
  void setCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT], uint8_t val );
  void copyCtuAlternativeChroma( uint8_t* ctuAltsDst[MAX_NUM_COMPONENT], uint8_t* ctuAltsSrc[MAX_NUM_COMPONENT] );
  int getMaxNumAlternativesChroma( );
  int  getCoeffRateCcAlf(short chromaCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], bool filterEnabled[MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, ComponentID compID);
#if JVET_AH0057_CCALF_COEFF_PRECISION
  void deriveCcAlfFilterCoeff( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx, int coeffPrec );
#else
  void deriveCcAlfFilterCoeff( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx );
#endif
  void determineControlIdcValues(CodingStructure &cs, const ComponentID compID, const PelBuf *buf, const int ctuWidthC,
                                 const int ctuHeightC, const int picWidthC, const int picHeightC,
                                 double **unfilteredDistortion, uint64_t *trainingDistortion[MAX_NUM_CC_ALF_FILTERS],
                                 uint64_t *lumaSwingGreaterThanThresholdCount,
                                 uint64_t *chromaSampleCountNearMidPoint,
                                 bool reuseFilterCoeff, uint8_t *trainingCovControl, uint8_t *filterControl,
                                 uint64_t &curTotalDistortion, double &curTotalRate,
                                 bool     filterEnabled[MAX_NUM_CC_ALF_FILTERS],
                                 uint8_t  mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1],
                                 uint8_t &ccAlfFilterCount);
  void deriveCcAlfFilter( CodingStructure& cs, ComponentID compID, const PelUnitBuf& orgYuv, const PelUnitBuf& tempDecYuvBuf, const PelUnitBuf& dstYuv );
  std::vector<int> getAvailableCcAlfApsIds(CodingStructure& cs, ComponentID compID);
  void xSetupCcAlfAPS( CodingStructure& cs );
  void countLumaSwingGreaterThanThreshold(const Pel* luma, int lumaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* lumaSwingGreaterThanThresholdCount, int lumaCountStride);
  void countChromaSampleValueNearMidPoint(const Pel* chroma, int chromaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* chromaSampleCountNearMidPoint, int chromaSampleCountNearMidPointStride);
  void getFrameStatsCcalf(ComponentID compIdx, int filterIdc);
  void initDistortionCcalf(int comp);

#if JVET_AK0065_TALF
  AlfCovariance* m_alfCovarianceTALF[MAX_TALF_FILTER_SHAPE][NUM_TALF_MODE];           // [ctbAddr]
  AlfCovariance  m_alfCovarianceFrameTALF[MAX_TALF_FILTER_SHAPE][NUM_TALF_MODE];
  Pel            m_ELocalStorage[4][NUM_TALF_COEFF + 1][MAX_CU_SIZE * MAX_CU_SIZE];
  Pel            m_yLocalStorage[MAX_CU_SIZE * MAX_CU_SIZE];
  RdCost*        m_sseCost;
  uint64_t*      m_talfDistortion[ALF_CTB_MAX_NUM_APS][MAX_NUM_TALF_FILTERS];   // for current block size
  PelStorage     m_tempSaveTALF[2];

  void determineCtuFlag(CodingStructure &cs, const ComponentID compId, double **unfilteredDistortion
    , uint64_t *trainingDistortion[MAX_NUM_TALF_FILTERS], std::vector<TAlfCtbParam>& filterControl
    , uint64_t &curTotalDistortion, double &curTotalRate, const int filterCount, uint8_t* trainingControl);
  bool determineCtuFlagReuse(CodingStructure &cs, const ComponentID compID, double **unfilteredDistortion
    , uint64_t *trainingDistortion[ALF_CTB_MAX_NUM_APS][MAX_NUM_TALF_FILTERS], TAlfCtbParam *filterControl
    , std::vector<int>& apsIds, std::vector<TAlfFilterParam>& params, uint64_t &curTotalDistortion
    , double &curTotalRate);
  CandTALF deriveTAlfNewFilter(CodingStructure &cs, ComponentID compId, const int mode, const int shapeIdx);
  void deriveTAlfReuseFilter(CodingStructure &cs, ComponentID compId, const int mode
    , static_vector<CandTALF, NUM_TALF_REUSE_CANDS>& reuseCandList);
  void deriveTAlfFilter(CodingStructure &cs, ComponentID compId, std::vector<CandTALF>& talfCandList);
  void deriveStatsForTAlfFilter(CodingStructure &cs, ComponentID compID, const PelUnitBuf &orgYuv
    , const PelUnitBuf &recBeforeALF);
  void getBlkStatsTAlf(AlfCovariance &alfCovariance, const CPelBuf &orgYuv, const CPelBuf &recBeforeALF
    , CodingStructure &cs, const UnitArea& clippedCtu, const ComponentID compId, const int shapeIdx
    , const int mode);
  void getFrameStatsTAlf(const int filterIdc, AlfCovariance* covar, AlfCovariance& covarFrame);
  void deriveCoeffTAlf(const ComponentID compId, const AlfFilterShape& alfFilterShape, const AlfCovariance& covarFrame
    , int filterCoeffQuant[MAX_NUM_ALF_LUMA_COEFF], int filterClipIdx[MAX_NUM_ALF_LUMA_COEFF], const int nonLinearFlag
    , const int bitDepth);
  double getRecoDist(const PelUnitBuf &orgYuv, PelUnitBuf &recYuv, const ComponentID compId, const CodingStructure& cs);
  int  getOneTAlfRate(int nonLinearFlag, int coeff[MAX_NUM_ALF_LUMA_COEFF]);
  int  getSetTAlfRate(uint8_t filterCount, int bestCoeff[MAX_NUM_TALF_FILTERS][MAX_NUM_ALF_LUMA_COEFF]
    , int coeff[MAX_NUM_ALF_LUMA_COEFF], int bestNonLinearFlag[MAX_NUM_TALF_FILTERS], int nonLinearFlag);
  void setTAlfAPS(CodingStructure& cs, CandTALF& cand);
  std::vector<int> getAvailableTAlfApsIds(CodingStructure &cs, int& newApsId);
#endif
};

#endif
