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
 * \file
 * \brief Declaration of InterpolationFilter class
 */

#ifndef __INTERPOLATIONFILTER__
#define __INTERPOLATIONFILTER__

#include "CommonDef.h"
#include "CacheModel.h"

//! \ingroup CommonLib
//! \{

#define IF_INTERNAL_PREC 14 ///< Number of bits for internal precision
#if IF_12TAP
#define IF_FILTER_PREC    8 ///< Log2 of sum of filter taps
#else
#define IF_FILTER_PREC    6 ///< Log2 of sum of filter taps
#endif
#define IF_INTERNAL_OFFS (1<<(IF_INTERNAL_PREC-1)) ///< Offset used internally
#if JVET_AJ0237_INTERNAL_12BIT
#define IF_INTERNAL_PREC_BILINEAR(bd) std::min(IF_INTERNAL_PREC, int(bd))
#else
#define IF_INTERNAL_PREC_BILINEAR 10 ///< Number of bits for internal precision
#endif
#define IF_FILTER_PREC_BILINEAR   4  ///< Bilinear filter coeff precision so that intermediate value will not exceed 16 bit for SIMD - bit exact
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#define IF_INTERNAL_FRAC_BITS(bd) std::max(2, IF_INTERNAL_PREC - int(bd))
#endif
/**
 * \brief Interpolation filter class
 */
class InterpolationFilter
{
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
public:
  static const TFilterCoeff m_lumaFilter64[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS * 4][12];
#endif
#if IF_12TAP
  static const TFilterCoeff m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8];
#else
  static const TFilterCoeff m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA];
#endif
public:
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps
#if JVET_Z0117_CHROMA_IF
  static const TFilterCoeff m_chromaFilter4[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4]; ///< Chroma filter taps
#endif
#if JVET_Z0117_CHROMA_IF
#if JVET_AE0150_SMALL_SCALE_RPR_FILTERS
  static const TFilterCoeff m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR]; ///< Chroma filter taps 1.35x
  static const TFilterCoeff m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR]; ///< Chroma filter taps 1.5x
  static const TFilterCoeff m_chromaFilterRPR3[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR]; ///< Chroma filter taps 2x
#else
  static const TFilterCoeff m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR]; ///< Chroma filter taps 1.5x
  static const TFilterCoeff m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR]; ///< Chroma filter taps 2x
#endif
#else
  static const TFilterCoeff m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps 1.5x
  static const TFilterCoeff m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps 2x
#endif
#if IF_12TAP
  static const TFilterCoeff m_lumaFilter12[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12];     ///< Luma filter taps //+1 added by //kolya
#if JVET_AI0094_SHARP_MC_FILTER_FOR_BIPRED
  static const TFilterCoeff m_lumaFilter12bi[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12];     ///< Luma filter taps for bi-prediction
#endif
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8]; ///< Luma filter taps, for affine 
#if JVET_AA0042_RPR_FILTERS
#if JVET_AE0150_SMALL_SCALE_RPR_FILTERS
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.35x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR3[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 2x
#else
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 2x
#endif
  static const TFilterCoeff m_affineLumaFilterUpRPR[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12];
#if JVET_AE0150_SMALL_SCALE_RPR_FILTERS
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.35x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR3[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 2x
#else
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12]; ///< Luma filter taps 2x
#endif
#else
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8]; ///< Luma filter taps 2x
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8]; ///< Luma filter taps 2x
#endif
private:
  static const TFilterCoeff m_lumaAltHpelIFilter[8]; ///< Luma filter taps
#else
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 2x
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 2x

private:
  static const TFilterCoeff m_lumaAltHpelIFilter[NTAPS_LUMA]; ///< Luma filter taps
#endif
#if INTRA_6TAP
  static const TFilterCoeff m_lumaIntraFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][6]; ///< Chroma filter 6 taps
#if JVET_Z0117_CHROMA_IF
  static const TFilterCoeff m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4]; ///< Weak filter 4 taps
#else
  static const TFilterCoeff m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Weak filter 4 taps
#endif
#if JVET_W0123_TIMD_FUSION
  static const TFilterCoeff m_lumaIntraFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][6]; ///< Chroma filter 6 taps
#endif
#endif
#if JVET_AK0087_INTRA_8TAP
  static const TFilterCoeff m_lumaIntra8tapNonSmoothingFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8];
#if JVET_W0123_TIMD_FUSION
  static const TFilterCoeff m_lumaIntra8tapNonSmoothingFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][8];
#endif
#endif

#if JVET_AN0158_INTRA_LONGTAP
  static const TFilterCoeff m_lumaIntra10tapNonSmoothingFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][10];
#if JVET_W0123_TIMD_FUSION
  static const TFilterCoeff m_lumaIntra10tapNonSmoothingFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][10];
#endif
  static const TFilterCoeff m_lumaIntra12tapNonSmoothingFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][12];
#if JVET_W0123_TIMD_FUSION
  static const TFilterCoeff m_lumaIntra12tapNonSmoothingFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][12];
#endif
  static const TFilterCoeff m_lumaIntra16tapNonSmoothingFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][16];
#if JVET_W0123_TIMD_FUSION
  static const TFilterCoeff m_lumaIntra16tapNonSmoothingFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][16];
#endif
#endif 


#if JVET_W0123_TIMD_FUSION
#if JVET_Z0117_CHROMA_IF
  static const TFilterCoeff g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4]; ///< Chroma filter taps
  static const TFilterCoeff g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4]; ///< Chroma filter taps
#else
  static const TFilterCoeff g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA]; ///< Chroma filter taps
  static const TFilterCoeff g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA]; ///< Chroma filter taps
#endif
#endif
  static const TFilterCoeff m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
  static const TFilterCoeff m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
public:
  static const TFilterCoeff m_bilinearFilterChroma[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps for chroma
#endif
public:
  template<bool isFirst, bool isLast>
  static void filterCopy( const ClpRng& clpRng, const Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height, bool biMCForDMVR);

#if JVET_AC0104_IBC_BVD_PREDICTION
  static void filterCopyWithNoClipping( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height );
  template<bool isVer>
  static void filterReverseCopy( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height );
#endif

  template<int N, bool isVertical, bool isFirst, bool isLast>
  static void filter(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);

  void filter4x4( const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int xFrac, int yFrac, bool isLast
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  , bool useHighPrec = false
#endif
  );

  template<int N>
  void filterHor(const ClpRng& clpRng, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR);

  template<int N>
  void filterVer(const ClpRng& clpRng, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR);

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  static void xWeightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  void weightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#else
  static void xWeightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  void weightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  template <bool trueTFalseL>
  static void xWeightedGeoTpl(const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  template <bool trueTFalseL>
  void weightedGeoTpl (const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
  {
    (trueTFalseL ? m_weightedGeoTplA : m_weightedGeoTplL)(pu, splitDir, predDst, predSrc0, predSrc1);
  }
#endif
#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  static void xWeightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  void weightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#else
  static void xWeightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  void weightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  static void xWeightedBlendBlk(const PredictionUnit& pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, WeightBuf& weightBuf, const int log2WeightBase, const bool roundOutputBD);
  void weightedBlendBlk(const PredictionUnit& pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, WeightBuf& weightBuf, const int log2WeightBase, const bool roundOutputBD);
  static void xWeightAffineBlk(const PredictionUnit& pu, WeightBuf& bufWeight, const int log2WeightBase, AffineBlendingModel& blendModel);
  void weightAffineBlk(const PredictionUnit& pu, WeightBuf& bufWeight, const int log2WeightBase, AffineBlendingModel& blendModel);
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  static void xWeightObmcBoundary(Pel* dst, Pel* src, const int dstStride, const int srcStride, const int width, const int height, const int dir, const ComponentID compID, const int blendMode, const bool subMotion);
  static void xWeightObmcInnerBoundary(const ComponentID comp, Pel* pOrgDst, Pel* pOrgSrc1, Pel* pOrgSrc2, Pel* pOrgSrc3, Pel* pOrgSrc4, const int dstStride, const int srcStride, const int width, const int height, bool isAboveAvail, bool isLeftAvail, bool isBelowAvail, bool isRightAvail);
#endif
protected:
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  static CacheModel* m_cacheModel;
#endif
public:
  InterpolationFilter();
  ~InterpolationFilter() {}
#if IF_12TAP
#if SIMD_4x4_12 && defined(TARGET_SIMD_X86)
  void(*m_filter4x4[2])(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int shiftH, int offsetH, int shiftV, int offsetV, int16_t const *coeffH, int16_t const *coeffV, int ibdimin, int ibdimax); //kolya
#endif

#if JVET_Z0117_CHROMA_IF
  void(*m_filterHor[5][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void(*m_filterVer[5][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
#else
  void(*m_filterHor[4][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void(*m_filterVer[4][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
#endif
#else
#if JVET_Z0117_CHROMA_IF
  void(*m_filterHor[4][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void(*m_filterVer[4][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
#else
  void(*m_filterHor[3][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  void(*m_filterVer[3][2][2])(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
#endif
#endif
  void( *m_filterCopy[2][2] )  ( const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, bool biMCForDMVR);
#if JVET_AC0104_IBC_BVD_PREDICTION
  void( *m_filterCopyWithNoClipping )(const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height);
  void( *m_filterReverseCopy[2] )(const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  void( *m_weightedGeoBlk )(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#else
  void( *m_weightedGeoBlk )(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void( *m_weightedGeoTplA )(const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
  void( *m_weightedGeoTplL )(const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  void( *m_weightedGeoBlkRounded )(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#else
  void( *m_weightedGeoBlkRounded )(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1);
#endif
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  void ( *m_weightedBlendBlk )(const PredictionUnit& pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, WeightBuf& weightBuf, const int log2WeightBase, const bool roundOutputBD);
  void ( *m_weightAffineBlk )(const PredictionUnit& pu, WeightBuf& weightBuf, const int log2WeightBase, AffineBlendingModel& blendModel );
#endif
#if JVET_AB0155_SGPM
  void (*m_weightedSgpm)(const PredictionUnit &pu, const uint32_t width, const uint32_t height,
                         const ComponentID compIdx, const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0,
                         PelBuf &predSrc1);
  static void xWeightedSgpm(const PredictionUnit &pu, const uint32_t width, const uint32_t height,
                         const ComponentID compIdx, const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0,
                         PelBuf &predSrc1);

  int (*m_sadTM)(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                 const int templateHeight, const ComponentID compIdx, PelBuf &predBuf, PelBuf &recBuf, PelBuf &adBuf);
  static int xSadTM(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                    const int templateHeight, const ComponentID compIdx, PelBuf &predBuf, PelBuf &recBuf,
                    PelBuf &adBuf);
  int (*m_sgpmSadTM)(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                     const int templateHeight, const ComponentID compIdx, const uint8_t splitDir, PelBuf &adBuf);
  static int xSgpmSadTM(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                            const int templateHeight, const ComponentID compIdx, const uint8_t splitDir,
                              PelBuf &adBuf);
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void (*m_weightObmcBoundary)(Pel* dst, Pel* src, const int dstStride, const int srcStride, const int width, const int height, const int dir, const ComponentID compID, const int blendMode, const bool subMotion);
  void (*m_weightObmcInnerBoundary)(const ComponentID comp, Pel* pOrgDst, Pel* pOrgSrc1, Pel* pOrgSrc2, Pel* pOrgSrc3, Pel* pOrgSrc4, const int dstStride, const int srcStride, const int width, const int height, bool isAboveAvail, bool isLeftAvail, bool isBelowAvail, bool isRightAvail);
#endif

  void initInterpolationFilter( bool enable );
#ifdef TARGET_SIMD_X86
  void initInterpolationFilterX86();
  template <X86_VEXT vext>
  void _initInterpolationFilterX86();
#endif
#if JVET_AI0094_SHARP_MC_FILTER_FOR_BIPRED
#if JVET_AL0161_4TAP_TM
  void filterHor(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool useBiFilt = 0, bool use4TapTM = false
#else
  void filterHor(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool useBiFilt = 0
#endif
#else
#if JVET_AL0161_4TAP_TM
  void filterHor(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool use4TapTM = false
#else
  void filterHor(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false
#endif
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
               , const bool useCopyWithNoClipping = false
#endif
                );
#if JVET_AI0094_SHARP_MC_FILTER_FOR_BIPRED
#if JVET_AL0161_4TAP_TM
  void filterVer(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool useBiFilt = 0, bool use4TapTM = false);
#else
  void filterVer(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool useBiFilt = 0);
#endif
#else
#if JVET_AL0161_4TAP_TM
  void filterVer(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false, bool use4TapTM = false);
#else
  void filterVer(const ComponentID compID, Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false);
#endif
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void cacheAssign( CacheModel *cache ) { m_cacheModel = cache; }
#endif

#if INTRA_6TAP
  static TFilterCoeff const * const getIntraLumaFilterTable(const int deltaFract) { return m_lumaIntraFilter[deltaFract]; };
  static TFilterCoeff const * const getWeak4TapFilterTable(const int deltaFract) { return m_weak4TapFilter[deltaFract]; };
#if JVET_W0123_TIMD_FUSION
  static TFilterCoeff const * const getIntraLumaFilterTableExt(const int deltaFract) { return m_lumaIntraFilterExt[deltaFract]; };
#endif
#endif
#if JVET_Z0117_CHROMA_IF
  static TFilterCoeff const * const getChromaFilterTable(const int deltaFract) { return m_chromaFilter4[deltaFract]; };
#else
  static TFilterCoeff const * const getChromaFilterTable(const int deltaFract) { return m_chromaFilter[deltaFract]; };
#endif
#if JVET_W0123_TIMD_FUSION
  static TFilterCoeff const * const getExtIntraCubicFilter(const int deltaFract) { return g_aiExtIntraCubicFilter[deltaFract]; };
  static TFilterCoeff const * const getExtIntraGaussFilter(const int deltaFract) { return g_aiExtIntraGaussFilter[deltaFract]; };
#endif
#if JVET_AN0158_INTRA_LONGTAP
  static TFilterCoeff const* const getIntraNtapNonSmoothingFilter(const bool bExtIntraDir, const int deltaFract, const int Ntaps);
  static constexpr int getMaxNumTapsIntra() { return 16; }
#endif

#if JVET_AK0087_INTRA_8TAP
  static TFilterCoeff const* const getIntra8tapCubicFilter(const int deltaFract) { return m_lumaIntra8tapNonSmoothingFilter[deltaFract]; };
#if JVET_W0123_TIMD_FUSION
  static TFilterCoeff const* const getExtIntra8tapCubicFilter(const int deltaFract) { return m_lumaIntra8tapNonSmoothingFilterExt[deltaFract]; };
#endif
#endif
};

//! \}

#endif
