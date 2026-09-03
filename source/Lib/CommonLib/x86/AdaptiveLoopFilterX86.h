/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

#pragma once

#include "CommonDefX86.h"
#include "../AdaptiveLoopFilter.h"
#if ALF_IMPROVEMENT
#include "AlfFixedFilters.h"
#endif

#ifdef TARGET_SIMD_X86
#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <x86intrin.h>
#endif

constexpr uint16_t sh(int x)
{
  return 0x0202 * (x & 7) + 0x0100 + 0x1010 * (x & 8);
}

#if ALF_IMPROVEMENT
static const uint16_t shuffleTab5[4][8] = {
  {
    sh(0), sh(1), sh(2), sh(3), sh(4), sh(5), sh(6), sh(7),
  },
  {
    sh(4), sh(1), sh(5), sh(3), sh(0), sh(2), sh(6), sh(7),
  },
  {
    sh(0), sh(3), sh(2), sh(1), sh(4), sh(5), sh(6), sh(7),
  },
  {
    sh(4), sh(3), sh(5), sh(1), sh(0), sh(2), sh(6), sh(7) ,
  },
};

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
constexpr int p0[FIX_FILTER_NUM_COEFF_13_DB_9] =
{
  0, 2, 6, 12, 20, 30,
  36, 37, 38, 39, 40, 41,
  1, 4, 9, 16, 25, 11, 18, 27,
  3, 8, 15, 24, 35, 13, 22, 33,
  5, 10, 17, 26, 19, 28, 29, 31,
  7, 14, 23, 34, 21, 32,
  42, 44, 48, 54, 58, 59, 60, 61,
  43, 46, 51, 45, 50, 57,
  47, 52, 53, 49, 56, 55, 62, 63
};
static std::array<std::array<std::array<short, FIX_FILTER_NUM_COEFF_13_DB_9>, NUM_FIXED_FILTERS>, NUM_SETS_FIXED_FILTERS> packedDataFixedFilters13Db9;
// Adding +1 to filter size to avoid reading past end of array
static std::array<std::array<std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9 + 1>, NUM_FIXED_FILTERS>, NUM_SETS_FIXED_FILTERS> packedDataFixedFilters9Db9;
static std::array<std::array<std::array<short, FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9 + 1>, NUM_FIXED_FILTERS>, NUM_SETS_FIXED_FILTERS> packedDataFixedFilters9Db9Combine;
constexpr int p0Filter9Db9[FIX_FILTER_NUM_COEFF_9_DB_9] =
{
  0,  1,  4,  9, 16,  3,  8, 15,
  2,  5, 10, 17,  7, 14,
  6, 11, 18, 13, 12, 19,
  20, 21, 24, 29, 36, 23, 28, 35,
  22, 25, 30, 37, 27, 34,
  26, 31, 38, 33, 32, 39, 40
};
constexpr int p0Filter9Db9Combine[FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9] =
{
  20, 21, 24, 29, 36, 23, 28, 35,
  22, 25, 30, 37, 27, 34,
  26, 31, 38, 33, 32, 39, 40
};
#else
constexpr int p0[42] =
{
  0, 2, 6, 12, 20, 30,
  36, 37, 38, 39, 40, 41,
  1, 4, 9, 16, 25, 11, 18, 27,
  3, 8, 15, 24, 35, 13, 22, 33,
  5, 10, 17, 26, 19, 28, 29, 31,
  7, 14, 23, 34, 21, 32
};

static std::array<std::array<std::array<std::array<short, 42>, NUM_FIXED_FILTERS>, NUM_CLASSIFIER>, NUM_SETS_FIXED_FILTERS> packedDataFixedFilters;
#endif
#else
template<X86_VEXT vext>
static void simdDeriveClassificationBlk(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos)
{
  CHECK((blk.height & 7) != 0, "Block height must be a multiple of 8");
  CHECK((blk.width & 7) != 0, "Block width must be a multiple of 8");
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const size_t imgStride = srcLuma.stride;
  const Pel *  srcExt    = srcLuma.buf;

  const int imgHExtended = blk.height + 4;
  const int imgWExtended = blk.width + 4;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  // 18x40 array
  uint16_t colSums[(AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4) >> 1]
                  [AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8];

  for (int i = 0; i < imgHExtended; i += 2)
  {
    const size_t offset = (i + posY - 3) * imgStride + posX - 3;

    const Pel *imgY0 = &srcExt[offset];
    const Pel *imgY1 = &srcExt[offset + imgStride];
    const Pel *imgY2 = &srcExt[offset + imgStride * 2];
    const Pel *imgY3 = &srcExt[offset + imgStride * 3];

    // pixel padding for gradient calculation
    int pos      = blkDst.pos().y - 2 + i;
    int posInCTU = pos & (vbCTUHeight - 1);
    if (pos > 0 && posInCTU == vbPos - 2)
    {
      imgY3 = imgY2;
    }
    else if (pos > 0 && posInCTU == vbPos)
    {
      imgY0 = imgY1;
    }

    __m128i prev = _mm_setzero_si128();

    for (int j = 0; j < imgWExtended; j += 8)
    {
      const __m128i x0 = _mm_loadu_si128((const __m128i *) (imgY0 + j));
      const __m128i x1 = _mm_loadu_si128((const __m128i *) (imgY1 + j));
      const __m128i x2 = _mm_loadu_si128((const __m128i *) (imgY2 + j));
      const __m128i x3 = _mm_loadu_si128((const __m128i *) (imgY3 + j));

      const __m128i x4 = _mm_loadu_si128((const __m128i *) (imgY0 + j + 2));
      const __m128i x5 = _mm_loadu_si128((const __m128i *) (imgY1 + j + 2));
      const __m128i x6 = _mm_loadu_si128((const __m128i *) (imgY2 + j + 2));
      const __m128i x7 = _mm_loadu_si128((const __m128i *) (imgY3 + j + 2));

      const __m128i nw = _mm_blend_epi16(x0, x1, 0xaa);
      const __m128i n  = _mm_blend_epi16(x0, x5, 0x55);
      const __m128i ne = _mm_blend_epi16(x4, x5, 0xaa);
      const __m128i w  = _mm_blend_epi16(x1, x2, 0xaa);
      const __m128i e  = _mm_blend_epi16(x5, x6, 0xaa);
      const __m128i sw = _mm_blend_epi16(x2, x3, 0xaa);
      const __m128i s  = _mm_blend_epi16(x2, x7, 0x55);
      const __m128i se = _mm_blend_epi16(x6, x7, 0xaa);

      __m128i c = _mm_blend_epi16(x1, x6, 0x55);
      c         = _mm_add_epi16(c, c);
      __m128i d = _mm_shuffle_epi8(c, _mm_setr_epi8(2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13));

      const __m128i ver = _mm_abs_epi16(_mm_sub_epi16(c, _mm_add_epi16(n, s)));
      const __m128i hor = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(w, e)));
      const __m128i di0 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(nw, se)));
      const __m128i di1 = _mm_abs_epi16(_mm_sub_epi16(d, _mm_add_epi16(ne, sw)));

      const __m128i hv  = _mm_hadd_epi16(ver, hor);
      const __m128i di  = _mm_hadd_epi16(di0, di1);
      const __m128i all = _mm_hadd_epi16(hv, di);

      const __m128i t = _mm_blend_epi16(all, prev, 0xaa);
      _mm_storeu_si128((__m128i *) &colSums[i >> 1][j], _mm_hadd_epi16(t, all));
      prev = all;
    }
  }

  for (int i = 0; i < (blk.height >> 1); i += 4)
  {
    for (int j = 0; j < blk.width; j += 8)
    {
      __m128i x0, x1, x2, x3, x4, x5, x6, x7;

      const uint32_t z = (2 * i + blkDst.pos().y) & (vbCTUHeight - 1);
      const uint32_t z2 = (2 * i + 4 + blkDst.pos().y) & (vbCTUHeight - 1);

      x0 = (z == vbPos) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 0][j + 4]);
      x1 = _mm_loadu_si128((__m128i *) &colSums[i + 1][j + 4]);
      x2 = _mm_loadu_si128((__m128i *) &colSums[i + 2][j + 4]);
      x3 = (z == vbPos - 4) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 3][j + 4]);

      x4 = (z2 == vbPos) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 2][j + 4]);
      x5 = _mm_loadu_si128((__m128i *) &colSums[i + 3][j + 4]);
      x6 = _mm_loadu_si128((__m128i *) &colSums[i + 4][j + 4]);
      x7 = (z2 == vbPos - 4) ? _mm_setzero_si128() : _mm_loadu_si128((__m128i *) &colSums[i + 5][j + 4]);

      __m128i x0l = _mm_cvtepu16_epi32(x0);
      __m128i x0h = _mm_unpackhi_epi16(x0, _mm_setzero_si128());
      __m128i x1l = _mm_cvtepu16_epi32(x1);
      __m128i x1h = _mm_unpackhi_epi16(x1, _mm_setzero_si128());
      __m128i x2l = _mm_cvtepu16_epi32(x2);
      __m128i x2h = _mm_unpackhi_epi16(x2, _mm_setzero_si128());
      __m128i x3l = _mm_cvtepu16_epi32(x3);
      __m128i x3h = _mm_unpackhi_epi16(x3, _mm_setzero_si128());
      __m128i x4l = _mm_cvtepu16_epi32(x4);
      __m128i x4h = _mm_unpackhi_epi16(x4, _mm_setzero_si128());
      __m128i x5l = _mm_cvtepu16_epi32(x5);
      __m128i x5h = _mm_unpackhi_epi16(x5, _mm_setzero_si128());
      __m128i x6l = _mm_cvtepu16_epi32(x6);
      __m128i x6h = _mm_unpackhi_epi16(x6, _mm_setzero_si128());
      __m128i x7l = _mm_cvtepu16_epi32(x7);
      __m128i x7h = _mm_unpackhi_epi16(x7, _mm_setzero_si128());

      x0l = _mm_add_epi32(x0l, x1l);
      x2l = _mm_add_epi32(x2l, x3l);
      x4l = _mm_add_epi32(x4l, x5l);
      x6l = _mm_add_epi32(x6l, x7l);
      x0h = _mm_add_epi32(x0h, x1h);
      x2h = _mm_add_epi32(x2h, x3h);
      x4h = _mm_add_epi32(x4h, x5h);
      x6h = _mm_add_epi32(x6h, x7h);

      x0l = _mm_add_epi32(x0l, x2l);
      x4l = _mm_add_epi32(x4l, x6l);
      x0h = _mm_add_epi32(x0h, x2h);
      x4h = _mm_add_epi32(x4h, x6h);

      x2l = _mm_unpacklo_epi32(x0l, x4l);
      x2h = _mm_unpackhi_epi32(x0l, x4l);
      x6l = _mm_unpacklo_epi32(x0h, x4h);
      x6h = _mm_unpackhi_epi32(x0h, x4h);

      __m128i sumV  = _mm_unpacklo_epi32(x2l, x6l);
      __m128i sumH  = _mm_unpackhi_epi32(x2l, x6l);
      __m128i sumD0 = _mm_unpacklo_epi32(x2h, x6h);
      __m128i sumD1 = _mm_unpackhi_epi32(x2h, x6h);

      //      uint32_t tempAct = sumV + sumH;
      __m128i tempAct = _mm_add_epi32(sumV, sumH);

      //      const uint32_t activity = std::min<uint32_t>(15, tempAct * scale >> shift);
      //      static const uint8_t th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
      //      uint8_t classIdx = th[activity];
      const uint32_t scale  = (z == vbPos - 4 || z == vbPos) ? 96 : 64;
      const uint32_t scale2 = (z2 == vbPos - 4 || z2 == vbPos) ? 96 : 64;
      __m128i activity = _mm_mullo_epi32(tempAct, _mm_unpacklo_epi64(_mm_set1_epi32(scale), _mm_set1_epi32(scale2)));
      activity         = _mm_srl_epi32(activity, _mm_cvtsi32_si128(shift));
      activity         = _mm_min_epi32(activity, _mm_set1_epi32(15));
      __m128i classIdx = _mm_shuffle_epi8(_mm_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);

      //      if (sumV > sumH)
      //      {
      //        hv1       = sumV;
      //        hv0       = sumH;
      //        dirTempHV = 0;
      //      }
      //      else
      //      {
      //        hv1       = sumH;
      //        hv0       = sumV;
      //        dirTempHV = 1;
      //      }
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i hv1             = _mm_max_epi32(sumV, sumH);
      __m128i hv0             = _mm_min_epi32(sumV, sumH);

      //      if (sumD0 > sumD1)
      //      {
      //        d1       = sumD0;
      //        d0       = sumD1;
      //        dirTempD = 0;
      //      }
      //      else
      //      {
      //        d1       = sumD1;
      //        d0       = sumD0;
      //        dirTempD = 1;
      //      }
      __m128i dirTempDMinus1 = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i d1             = _mm_max_epi32(sumD0, sumD1);
      __m128i d0             = _mm_min_epi32(sumD0, sumD1);

      //      int dirIdx;
      //      if (d1 * hv0 > hv1 * d0)
      //      {
      //        hvd1   = d1;
      //        hvd0   = d0;
      //        dirIdx = 0;
      //      }
      //      else
      //      {
      //        hvd1   = hv1;
      //        hvd0   = hv0;
      //        dirIdx = 2;
      //      }
      __m128i a      = _mm_xor_si128(_mm_mullo_epi32(d1, hv0), _mm_set1_epi32(0x80000000));
      __m128i b      = _mm_xor_si128(_mm_mullo_epi32(hv1, d0), _mm_set1_epi32(0x80000000));
      __m128i dirIdx = _mm_cmpgt_epi32(a, b);
      __m128i hvd1   = _mm_blendv_epi8(hv1, d1, dirIdx);
      __m128i hvd0   = _mm_blendv_epi8(hv0, d0, dirIdx);

      //      if (hvd1 * 2 > 9 * hvd0)
      //      {
      //        classIdx += (dirIdx + 2) * 5;
      //      }
      //      else if (hvd1 > 2 * hvd0)
      //      {
      //        classIdx += (dirIdx + 1) * 5;
      //      }
      __m128i strength1 = _mm_cmpgt_epi32(hvd1, _mm_add_epi32(hvd0, hvd0));
      __m128i strength2 = _mm_cmpgt_epi32(_mm_add_epi32(hvd1, hvd1), _mm_add_epi32(hvd0, _mm_slli_epi32(hvd0, 3)));
      __m128i offset    = _mm_and_si128(strength1, _mm_set1_epi32(5));
      classIdx          = _mm_add_epi32(classIdx, offset);
      classIdx          = _mm_add_epi32(classIdx, _mm_and_si128(strength2, _mm_set1_epi32(5)));
      offset            = _mm_andnot_si128(dirIdx, offset);
      offset            = _mm_add_epi32(offset, offset);
      classIdx          = _mm_add_epi32(classIdx, offset);

      //      uint8_t transposeIdx = 2 * dirTempD + dirTempHV;
      __m128i transposeIdx = _mm_set1_epi32(3);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx         = _mm_add_epi32(transposeIdx, dirTempDMinus1);

      int yOffset = 2 * i + blkDst.pos().y;
      int xOffset = j + blkDst.pos().x;

      static_assert(sizeof(AlfClassifier) == 2, "ALFClassifier type must be 16 bits wide");
      __m128i v;
      v = _mm_unpacklo_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 8, 9, 8, 9, 8, 9, 8, 9));
      _mm_storeu_si128((__m128i *) (classifier[yOffset] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 1] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 2] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 3] + xOffset), v);
      v = _mm_unpackhi_epi8(classIdx, transposeIdx);
      v = _mm_shuffle_epi8(v, _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 8, 9, 8, 9, 8, 9, 8, 9));
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 4] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 5] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 6] + xOffset), v);
      _mm_storeu_si128((__m128i *) (classifier[yOffset + 7] + xOffset), v);
    }
  }
}
#endif

template<X86_VEXT vext>
static void simdFilter5x5Blk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#else  
  const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#endif
#if ALF_IMPROVEMENT
  , Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#else
  , const int vbCTUHeight, int vbPos
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
#if !ALF_IMPROVEMENT
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
  CHECK(!isChroma(compId), "ALF 5x5 filter is for chroma only");
#endif

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
#if !ALF_IMPROVEMENT
  const __m128i mmOffset1 = _mm_set1_epi32((1 << ((shift + 3) - 1)) - round );
#endif

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
#if ALF_IMPROVEMENT
  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");
  size_t stepY = isChroma(compId) ? 4 : 2;
#else
  constexpr size_t stepY = 4;
#endif

  CHECK(blk.y % stepY, "Wrong startHeight in filtering");
  CHECK(blk.x % stepX, "Wrong startWidth in filtering");
  CHECK(height % stepY, "Wrong endHeight in filtering");
  CHECK(width % 4, "Wrong endWidth in filtering");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;



  const __m128i mmOffset = _mm_set1_epi32( round );
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );

#if !ALF_IMPROVEMENT
  __m128i params[2][3];
  __m128i fs   = _mm_loadu_si128((__m128i *) filterSet);
  params[0][0] = _mm_shuffle_epi32(fs, 0x00);
  params[0][1] = _mm_shuffle_epi32(fs, 0x55);
  params[0][2] = _mm_shuffle_epi32(fs, 0xaa);
  __m128i fc   = _mm_loadu_si128((__m128i *) fClipSet);
  params[1][0] = _mm_shuffle_epi32(fc, 0x00);
  params[1][1] = _mm_shuffle_epi32(fc, 0x55);
  params[1][2] = _mm_shuffle_epi32(fc, 0xaa);
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
#if ALF_IMPROVEMENT
    const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
#endif
    for (size_t j = 0; j < width; j += stepX)
    {
#if ALF_IMPROVEMENT
      __m128i params[2][2][3];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; ++k)
      {
        const int transposeIdx = pClass ? (pClass[j + 4 * k] & 0x3)  : 0;
        const int classIdx = pClass ? ( pClass[j + 4 * k] >> 2) : 0;
        const int transposeIdx1 = pClass ? ( pClass[j + 4 * k + 2] & 0x3) : 0;
        const int classIdx1 = pClass ? ( pClass[j + 4 * k + 2] >> 2 ) : 0;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
        scaleFactor[k * 4] = scaleFactor[k * 4 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
        scaleFactor[k * 4 + 2] = scaleFactor[k * 4 + 3] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

        __m128i rawCoeff = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
        __m128i rawClip = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));

        __m128i s0 = _mm_loadu_si128((const __m128i *) shuffleTab5[transposeIdx]);
        __m128i s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));

        rawCoeff = _mm_or_si128(_mm_shuffle_epi8(rawCoeff, s0), _mm_shuffle_epi8(rawCoeff, s1));
        rawClip = _mm_or_si128(_mm_shuffle_epi8(rawClip, s0), _mm_shuffle_epi8(rawClip, s1));

        __m128i rawCoeff1 = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
        __m128i rawClip1 = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));

        s0 = _mm_loadu_si128((const __m128i *) shuffleTab5[transposeIdx1]);
        s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));

        rawCoeff1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff1, s0), _mm_shuffle_epi8(rawCoeff, s1));
        rawClip1 = _mm_or_si128(_mm_shuffle_epi8(rawClip1, s0), _mm_shuffle_epi8(rawClip, s1));

        params[k][0][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff, 0x00), _mm_shuffle_epi32(rawCoeff1, 0x00));
        params[k][0][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff, 0x55), _mm_shuffle_epi32(rawCoeff1, 0x55));
        params[k][0][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff, 0xaa), _mm_shuffle_epi32(rawCoeff1, 0xaa));
        params[k][1][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip, 0x00), _mm_shuffle_epi32(rawClip1, 0x00));
        params[k][1][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip, 0x55), _mm_shuffle_epi32(rawClip1, 0x55));
        params[k][1][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip, 0xaa), _mm_shuffle_epi32(rawClip1, 0xaa));
      }
#endif
      for (size_t ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
#if !ALF_IMPROVEMENT
        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - 2))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + 1))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
        }
#endif
        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();
#else
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);
          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

#if ALF_IMPROVEMENT
          __m128i limit01A = params[0][1][i];
          __m128i limit01B = params[1][1][i];
          val01A = _mm_min_epi16(val01A, limit01A);
          val01B = _mm_min_epi16(val01B, limit01B);
          val01C = _mm_min_epi16(val01C, limit01A);
          val01D = _mm_min_epi16(val01D, limit01B);

          limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
          limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

          val01A = _mm_max_epi16(val01A, limit01A);
          val01B = _mm_max_epi16(val01B, limit01B);
          val01C = _mm_max_epi16(val01C, limit01A);
          val01D = _mm_max_epi16(val01D, limit01B);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff01A = params[0][0][i];
          const __m128i coeff01B = params[1][0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#else
          __m128i limit01A = params[1][i];

          val01A = _mm_min_epi16(val01A, limit01A);
          val01B = _mm_min_epi16(val01B, limit01A);
          val01C = _mm_min_epi16(val01C, limit01A);
          val01D = _mm_min_epi16(val01D, limit01A);

          limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);

          val01A = _mm_max_epi16(val01A, limit01A);
          val01B = _mm_max_epi16(val01B, limit01A);
          val01C = _mm_max_epi16(val01C, limit01A);
          val01D = _mm_max_epi16(val01D, limit01A);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          __m128i coeff01A = params[0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01A));
#endif
        };

        process2coeffs(0, pImg3 + 0, pImg4 + 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(1, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(2, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm_srai_epi32( accumA, shift );
        accumB = _mm_srai_epi32( accumB, shift );
#else

        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);
        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm_srai_epi32(accumA, shift);
          accumB = _mm_srai_epi32(accumB, shift);
        }
        else
        {
          accumA = _mm_srai_epi32(_mm_add_epi32(accumA, mmOffset1), shift + 3);
          accumB = _mm_srai_epi32(_mm_add_epi32(accumB, mmOffset1), shift + 3);
        }
#endif
        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        if (j + stepX <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
        }
        else
        {
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), accumA);
        }
      }

    }

    src += srcStride * stepY;
    dst += dstStride * stepY;
  }
}

static const uint16_t shuffleTab[4][2][8] = {
  {
    { sh(0), sh(1), sh(2), sh(3), sh(4), sh(5), sh(6), sh(7) },
    { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(9), sh(4), sh(10), sh(8), sh(1), sh(5), sh(11), sh(7) },
    { sh(3), sh(0), sh(2), sh(6), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(0), sh(3), sh(2), sh(1), sh(8), sh(7), sh(6), sh(5) },
    { sh(4), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
  },
  {
    { sh(9), sh(8), sh(10), sh(4), sh(3), sh(7), sh(11), sh(5) },
    { sh(1), sh(0), sh(2), sh(6), sh(12), sh(13), sh(14), sh(15) },
  },
};

template<X86_VEXT vext>
static void simdFilter7x7Blk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#else  
  const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#endif
#if ALF_IMPROVEMENT
  , Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
#else
  , const int vbCTUHeight, int vbPos
#endif
)
{
#if !ALF_IMPROVEMENT
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
  CHECK(isChroma(compId), "7x7 ALF filter is meant for luma only");
#endif

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
#if ALF_IMPROVEMENT
  size_t stepY = isChroma(compId) ? 4 : 2;
#else
  constexpr size_t stepY = 4;
#endif

  CHECK(blk.y % stepY, "Wrong startHeight in filtering");
  CHECK(blk.x % stepX, "Wrong startWidth in filtering");
  CHECK(height % stepY, "Wrong endHeight in filtering");
  CHECK(width % stepX, "Wrong endWidth in filtering");


  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

  const __m128i mmOffset = _mm_set1_epi32(round);
#if !ALF_IMPROVEMENT
  const __m128i mmOffset1 = _mm_set1_epi32((1 << ((shift + 3) - 1)) - ROUND);
#endif
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );


  for (size_t i = 0; i < height; i += stepY)
  {
#if ALF_IMPROVEMENT
    const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
#else
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#endif

    for (size_t j = 0; j < width; j += stepX)
    {
      __m128i params[2][2][6];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif

      for (int k = 0; k < 2; ++k)
      {
#if ALF_IMPROVEMENT
        const int transposeIdx = pClass ? ( pClass[j + 4 * k] & 0x3 ) : 0;
        const int classIdx = pClass ? ( pClass[j + 4 * k] >> 2 ) : 0;
        const int transposeIdx1 = pClass ? ( pClass[j + 4 * k + 2] & 0x3 ) : 0;
        const int classIdx1 = pClass ? ( pClass[j + 4 * k + 2] >> 2 ) : 0;
#else
        const AlfClassifier &cl = pClass[j + 4 * k];

        const int transposeIdx = cl.transposeIdx;
        const int classIdx     = cl.classIdx;
#endif

        static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
        static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

        __m128i rawCoeff0, rawCoeff1;
        __m128i rawClip0, rawClip1;

          rawCoeff0 = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoeff1 = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));

          rawClip0 = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip1 = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));

        const __m128i s0 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx][0]);
        const __m128i s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
        const __m128i s2 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx][1]);
        const __m128i s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));

        const __m128i rawCoeffLo = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s0), _mm_shuffle_epi8(rawCoeff1, s1));
        const __m128i rawCoeffHi = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s2), _mm_shuffle_epi8(rawCoeff1, s3));
        const __m128i rawClipLo  = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s0), _mm_shuffle_epi8(rawClip1, s1));
        const __m128i rawClipHi  = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s2), _mm_shuffle_epi8(rawClip1, s3));

#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        scaleFactor[k * 4] = scaleFactor[k * 4 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
        scaleFactor[k * 4 + 2] = scaleFactor[k * 4 + 3] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

        rawCoeff0 = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
        rawCoeff1 = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));

        rawClip0 = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
        rawClip1 = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));

        const __m128i s01 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx1][0]);
        const __m128i s11 = _mm_xor_si128(s01, _mm_set1_epi8((char)0x80));
        const __m128i s21 = _mm_loadu_si128((const __m128i *) shuffleTab[transposeIdx1][1]);
        const __m128i s31 = _mm_xor_si128(s21, _mm_set1_epi8((char)0x80));

        const __m128i rawCoeffLo1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s01), _mm_shuffle_epi8(rawCoeff1, s11));
        const __m128i rawCoeffHi1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff0, s21), _mm_shuffle_epi8(rawCoeff1, s31));
        const __m128i rawClipLo1 = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s01), _mm_shuffle_epi8(rawClip1, s11));
        const __m128i rawClipHi1 = _mm_or_si128(_mm_shuffle_epi8(rawClip0, s21), _mm_shuffle_epi8(rawClip1, s31));

        params[k][0][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffLo, 0x00), _mm_shuffle_epi32(rawCoeffLo1, 0x00));
        params[k][0][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffLo, 0x55), _mm_shuffle_epi32(rawCoeffLo1, 0x55));
        params[k][0][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffLo, 0xaa), _mm_shuffle_epi32(rawCoeffLo1, 0xaa));
        params[k][0][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffLo, 0xff), _mm_shuffle_epi32(rawCoeffLo1, 0xff));
        params[k][0][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffHi, 0x00), _mm_shuffle_epi32(rawCoeffHi1, 0x00));
        params[k][0][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeffHi, 0x55), _mm_shuffle_epi32(rawCoeffHi1, 0x55));
        params[k][1][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipLo, 0x00), _mm_shuffle_epi32(rawClipLo1, 0x00));
        params[k][1][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipLo, 0x55), _mm_shuffle_epi32(rawClipLo1, 0x55));
        params[k][1][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipLo, 0xaa), _mm_shuffle_epi32(rawClipLo1, 0xaa));
        params[k][1][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipLo, 0xff), _mm_shuffle_epi32(rawClipLo1, 0xff));
        params[k][1][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipHi, 0x00), _mm_shuffle_epi32(rawClipHi1, 0x00));
        params[k][1][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClipHi, 0x55), _mm_shuffle_epi32(rawClipHi1, 0x55));
#else

        params[k][0][0] = _mm_shuffle_epi32(rawCoeffLo, 0x00);
        params[k][0][1] = _mm_shuffle_epi32(rawCoeffLo, 0x55);
        params[k][0][2] = _mm_shuffle_epi32(rawCoeffLo, 0xaa);
        params[k][0][3] = _mm_shuffle_epi32(rawCoeffLo, 0xff);
        params[k][0][4] = _mm_shuffle_epi32(rawCoeffHi, 0x00);
        params[k][0][5] = _mm_shuffle_epi32(rawCoeffHi, 0x55);
        params[k][1][0] = _mm_shuffle_epi32(rawClipLo, 0x00);
        params[k][1][1] = _mm_shuffle_epi32(rawClipLo, 0x55);
        params[k][1][2] = _mm_shuffle_epi32(rawClipLo, 0xaa);
        params[k][1][3] = _mm_shuffle_epi32(rawClipLo, 0xff);
        params[k][1][4] = _mm_shuffle_epi32(rawClipHi, 0x00);
        params[k][1][5] = _mm_shuffle_epi32(rawClipHi, 0x55);
#endif
      }

      for (size_t ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
#if !ALF_IMPROVEMENT
        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - 4))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = (yVb >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = (yVb >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + 3))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = (yVb <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = (yVb <= vbPos + 2) ? pImg3 : pImg5;
        }
#endif
        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);

#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();
#else
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

          __m128i limit01A = params[0][1][i];
          __m128i limit01B = params[1][1][i];

          val01A = _mm_min_epi16(val01A, limit01A);
          val01B = _mm_min_epi16(val01B, limit01B);
          val01C = _mm_min_epi16(val01C, limit01A);
          val01D = _mm_min_epi16(val01D, limit01B);

          limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
          limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

          val01A = _mm_max_epi16(val01A, limit01A);
          val01B = _mm_max_epi16(val01B, limit01B);
          val01C = _mm_max_epi16(val01C, limit01A);
          val01D = _mm_max_epi16(val01D, limit01B);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff01A = params[0][0][i];
          const __m128i coeff01B = params[1][0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
        };


        process2coeffs(0, pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1);
        process2coeffs(1, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(2, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
        process2coeffs(3, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(4, pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3);
        process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm_srai_epi32( accumA, shift );
        accumB = _mm_srai_epi32( accumB, shift );
#else
        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);
        if (!(isNearVBabove || isNearVBbelow))
        {
          accumA = _mm_srai_epi32(accumA, shift);
          accumB = _mm_srai_epi32(accumB, shift);
        }
        else
        {
          accumA = _mm_srai_epi32(_mm_add_epi32(accumA, mmOffset1), shift + 3);
          accumB = _mm_srai_epi32(_mm_add_epi32(accumB, mmOffset1), shift + 3);
        }
#endif
        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
      }
    }

    src += srcStride * stepY;
    dst += dstStride * stepY;
  }
}

#if ALF_IMPROVEMENT
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
static const uint16_t shuffleTime9FixedBasedDiamond[4] = { 0, 4, 2, 4 };
static const uint16_t shuffleOp9FixedBasedDiamond[4][4][2] =
{
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //0
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
    { 2, 3 },
  }, //1
  {
    { 0, 1 },
    { 2, 3 },
    { 0, 1 },
    { 0, 1 },
  }, //2
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
    { 2, 3 },
  }, //3
};
static const uint16_t shuffleTab9FixedBasedDiamond[4][4][2][8] =
{
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //0
  {
    {
      { sh(0) , sh(9) , sh(2) , sh(15), sh(4) , sh(10), sh(6) , sh(14) },
      { sh(8) , sh(1) , sh(5) , sh(11), sh(12), sh(13), sh(7) , sh(3) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10) , sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(4), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(5) , sh(9) , sh(7) },
      { sh(4) , sh(6) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //1
  {
    {
      { sh(0) , sh(3) , sh(2) , sh(1) , sh(8) , sh(7) , sh(6) , sh(5) },
      { sh(4) , sh(15), sh(14), sh(13), sh(12), sh(11), sh(10), sh(9) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(7) , sh(6) , sh(5) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //2
  {
    {
      { sh(0) , sh(15), sh(2), sh(9), sh(8) , sh(14), sh(6), sh(10) },
      { sh(4) , sh(3) , sh(7), sh(13), sh(1), sh(11), sh(5), sh(12) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10), sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(4) },
      { sh(8) , sh(9) , sh(10), sh(7), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(7) , sh(9) , sh(5) },
      { sh(4) , sh(6) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //3
};
#endif
static const uint16_t shuffleTime9[4] = { 0, 3, 1, 3 };
static const uint16_t shuffleOp9[4][3][2] =
{
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //0
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  }, //1
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //2
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  }, //3
};

static const uint16_t shuffleTab9[4][3][2][8] =
{
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //0
  {
    {
      { sh(0) , sh(9) , sh(2) , sh(15) , sh(4) , sh(10) , sh(6) , sh(14) },
      { sh(8) , sh(1) , sh(5), sh(11), sh(12), sh(13), sh(7), sh(3) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10) , sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(4), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //1
  {
    {
      { sh(0) , sh(3) , sh(2) , sh(1) , sh(8) , sh(7) , sh(6) , sh(5) },
      { sh(4) , sh(15), sh(14), sh(13), sh(12), sh(11), sh(10), sh(9) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //2
  {
    {
      { sh(0) , sh(15), sh(2), sh(9), sh(8) , sh(14), sh(6), sh(10) },
      { sh(4) , sh(3) , sh(7), sh(13), sh(1), sh(11), sh(5), sh(12) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10), sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(4) },
      { sh(8) , sh(9) , sh(10), sh(7), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //3
};

#if FIXFILTER_CFG
template<X86_VEXT vext>
static void simdFilter9x9BlkNoFix(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = isChroma(compId) ? 4 : 2;

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  CHECK(blk.y % stepY, "Wrong startHeight in filtering");
  CHECK(blk.x % stepX, "Wrong startWidth in filtering");
  CHECK(height % stepY, "Wrong endHeight in filtering");
  CHECK(width % 4, "Wrong endWidth in filtering");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);

    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
        __m256i params[2][2][10];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m128i rawCoeffTmp[2][2][3], rawClipTmp[2][2][3], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
          __m256i rawCoeff[2][3], rawClip[2][3];

          for (int l = 0; l < 2; l++)
          {
            const int transposeIdx0 = pClass ? (pClass[j + 4 * k + 2 * l + 0] & 0x3) : 0;
            const int classIdx0     = pClass ? (pClass[j + 4 * k + 2 * l + 0] >> 2) : 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l * 2] = scaleFactor[k * 8 + l * 2 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoeffTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoeffTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoeffTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));

            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime9[transposeIdx0]; m++)
            {
              int op0 = shuffleOp9[transposeIdx0][m][0];
              int op1 = shuffleOp9[transposeIdx0][m][1];

              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoeffTmp[0][l][op1], s1Tmp[0]));
              __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoeffTmp[0][l][op1], s3Tmp[0]));
              rawCoeffTmp[0][l][op0] = rawTmp0;
              rawCoeffTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }

            const int transposeIdx1 = pClass ? (pClass[j + 4 * k + 2 * l + 8] & 0x3) : 0;
            const int classIdx1     = pClass ? (pClass[j + 4 * k + 2 * l + 8] >> 2) : 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l * 2 + 4] = scaleFactor[k * 8 + l * 2 + 5] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoeffTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoeffTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoeffTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
#
            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime9[transposeIdx1]; m++)
            {
              int op0 = shuffleOp9[transposeIdx1][m][0];
              int op1 = shuffleOp9[transposeIdx1][m][1];

              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0        = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoeffTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1        = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoeffTmp[1][l][op1], s3Tmp[1]));
              rawCoeffTmp[1][l][op0] = rawTmp0;
              rawCoeffTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoeff[l][0] = _mm256_castsi128_si256( rawCoeffTmp[0][l][0]);
            rawCoeff[l][0] = _mm256_insertf128_si256(rawCoeff[l][0], rawCoeffTmp[1][l][0], 1);
            rawCoeff[l][1] = _mm256_castsi128_si256(rawCoeffTmp[0][l][1]);
            rawCoeff[l][1] = _mm256_insertf128_si256(rawCoeff[l][1], rawCoeffTmp[1][l][1], 1);
            rawCoeff[l][2] = _mm256_castsi128_si256(rawCoeffTmp[0][l][2]);
            rawCoeff[l][2] = _mm256_insertf128_si256(rawCoeff[l][2], rawCoeffTmp[1][l][2], 1);

            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);
            rawClip[l][2] = _mm256_castsi128_si256(rawClipTmp[0][l][2]);
            rawClip[l][2] = _mm256_insertf128_si256(rawClip[l][2], rawClipTmp[1][l][2], 1);
          }

          params[k][0][0] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0x00), _mm256_shuffle_epi32(rawCoeff[1][0], 0x00));
          params[k][0][1] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0x55), _mm256_shuffle_epi32(rawCoeff[1][0], 0x55));
          params[k][0][2] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0xaa), _mm256_shuffle_epi32(rawCoeff[1][0], 0xaa));
          params[k][0][3] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0xff), _mm256_shuffle_epi32(rawCoeff[1][0], 0xff));
          params[k][0][4] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0x00), _mm256_shuffle_epi32(rawCoeff[1][1], 0x00));
          params[k][0][5] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0x55), _mm256_shuffle_epi32(rawCoeff[1][1], 0x55));
          params[k][0][6] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0xaa), _mm256_shuffle_epi32(rawCoeff[1][1], 0xaa));
          params[k][0][7] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0xff), _mm256_shuffle_epi32(rawCoeff[1][1], 0xff));
          params[k][0][8] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0x00), _mm256_shuffle_epi32(rawCoeff[1][2], 0x00));
          params[k][0][9] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0x55), _mm256_shuffle_epi32(rawCoeff[1][2], 0x55));

          params[k][1][0] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0x00), _mm256_shuffle_epi32(rawClip[1][0], 0x00));
          params[k][1][1] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0x55), _mm256_shuffle_epi32(rawClip[1][0], 0x55));
          params[k][1][2] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0xaa), _mm256_shuffle_epi32(rawClip[1][0], 0xaa));
          params[k][1][3] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0xff), _mm256_shuffle_epi32(rawClip[1][0], 0xff));
          params[k][1][4] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0x00), _mm256_shuffle_epi32(rawClip[1][1], 0x00));
          params[k][1][5] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0x55), _mm256_shuffle_epi32(rawClip[1][1], 0x55));
          params[k][1][6] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0xaa), _mm256_shuffle_epi32(rawClip[1][1], 0xaa));
          params[k][1][7] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0xff), _mm256_shuffle_epi32(rawClip[1][1], 0xff));
          params[k][1][8] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0x00), _mm256_shuffle_epi32(rawClip[1][2], 0x00));
          params[k][1][9] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0x55), _mm256_shuffle_epi32(rawClip[1][2], 0x55));
        }

        for (size_t ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;

          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          __m256i accumA = _mm256_setzero_si256();
          __m256i accumB = _mm256_setzero_si256();
#else
          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;
#endif

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
            {
              const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
              const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
              const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
              const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

              __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
              __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
              __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
              __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

              __m256i limit01A = params[0][1][i];
              __m256i limit01B = params[1][1][i];

              val01A = _mm256_min_epi16(val01A, limit01A);
              val01B = _mm256_min_epi16(val01B, limit01B);
              val01C = _mm256_min_epi16(val01C, limit01A);
              val01D = _mm256_min_epi16(val01D, limit01B);

              limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
              limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

              val01A = _mm256_max_epi16(val01A, limit01A);
              val01B = _mm256_max_epi16(val01B, limit01B);
              val01C = _mm256_max_epi16(val01C, limit01A);
              val01D = _mm256_max_epi16(val01D, limit01B);

              val01A = _mm256_add_epi16(val01A, val01C);
              val01B = _mm256_add_epi16(val01B, val01D);

              const __m256i coeff01A = params[0][0][i];
              const __m256i coeff01B = params[1][0][i];

              accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
              accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
            };

          process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
          process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
          process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
          process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
          process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
          process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
          process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
          process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
          process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
          process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
          accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

          accumA = _mm256_add_epi32(accumA, mmOffset);
          accumB = _mm256_add_epi32(accumB, mmOffset);
#endif

          accumA = _mm256_srai_epi32(accumA, shift);
          accumB = _mm256_srai_epi32(accumB, shift);

          accumA = _mm256_packs_epi32(accumA, accumB);
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (dst + ii * dstStride + j), accumA);

        }   // for (size_t ii = 0; ii < stepY; ii++)
      } // for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
      dst += dstStride * stepY;
    }
  }
  else
  {

    const __m128i mmOffset = _mm_set1_epi32(round);
    const __m128i mmMin = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif

    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX)
      {
        __m128i params[2][2][10];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[8];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m128i rawCoeff[2][3], rawClip[2][3], s0, s1, s2, s3, rawTmp0, rawTmp1;

          for (int l = 0; l < 2; l++)
          {
            const int transposeIdx = pClass ? (pClass[j + 4 * k + 2 * l] & 0x3 ) : 0;
            const int classIdx = pClass ? (pClass[j + 4 * k + 2 * l] >> 2): 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 4 + l * 2] = scaleFactor[k * 4 + l * 2 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

            rawCoeff[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawCoeff[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoeff[l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));

            rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClip[l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime9[transposeIdx]; m++)
            {
              int op0 = shuffleOp9[transposeIdx][m][0];
              int op1 = shuffleOp9[transposeIdx][m][1];

              s0 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][0]);
              s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
              s2 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][1]);
              s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s0), _mm_shuffle_epi8(rawCoeff[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s2), _mm_shuffle_epi8(rawCoeff[l][op1], s3));
              rawCoeff[l][op0] = rawTmp0;
              rawCoeff[l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
              rawClip[l][op0] = rawTmp0;
              rawClip[l][op1] = rawTmp1;
            }
          }

          params[k][0][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x00), _mm_shuffle_epi32(rawCoeff[1][0], 0x00));
          params[k][0][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x55), _mm_shuffle_epi32(rawCoeff[1][0], 0x55));
          params[k][0][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xaa), _mm_shuffle_epi32(rawCoeff[1][0], 0xaa));
          params[k][0][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xff), _mm_shuffle_epi32(rawCoeff[1][0], 0xff));
          params[k][0][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x00), _mm_shuffle_epi32(rawCoeff[1][1], 0x00));
          params[k][0][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x55), _mm_shuffle_epi32(rawCoeff[1][1], 0x55));
          params[k][0][6] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xaa), _mm_shuffle_epi32(rawCoeff[1][1], 0xaa));
          params[k][0][7] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xff), _mm_shuffle_epi32(rawCoeff[1][1], 0xff));
          params[k][0][8] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x00), _mm_shuffle_epi32(rawCoeff[1][2], 0x00));
          params[k][0][9] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x55), _mm_shuffle_epi32(rawCoeff[1][2], 0x55));

          params[k][1][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x00), _mm_shuffle_epi32(rawClip[1][0], 0x00));
          params[k][1][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x55), _mm_shuffle_epi32(rawClip[1][0], 0x55));
          params[k][1][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xaa), _mm_shuffle_epi32(rawClip[1][0], 0xaa));
          params[k][1][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xff), _mm_shuffle_epi32(rawClip[1][0], 0xff));
          params[k][1][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x00), _mm_shuffle_epi32(rawClip[1][1], 0x00));
          params[k][1][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x55), _mm_shuffle_epi32(rawClip[1][1], 0x55));
          params[k][1][6] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xaa), _mm_shuffle_epi32(rawClip[1][1], 0xaa));
          params[k][1][7] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xff), _mm_shuffle_epi32(rawClip[1][1], 0xff));
          params[k][1][8] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x00), _mm_shuffle_epi32(rawClip[1][2], 0x00));
          params[k][1][9] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x55), _mm_shuffle_epi32(rawClip[1][2], 0x55));
        }

        for (size_t ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;

          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          __m128i accumA = _mm_setzero_si128();
          __m128i accumB = _mm_setzero_si128();
#else
          __m128i accumA = mmOffset;
          __m128i accumB = mmOffset;
#endif

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
            };

          process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
          process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
          process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
          process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
          process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
          process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
          process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
          process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
          process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
          process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
          accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

          accumA = _mm_add_epi32(accumA, mmOffset);
          accumB = _mm_add_epi32(accumB, mmOffset);
#endif

          accumA = _mm_srai_epi32(accumA, shift);
          accumB = _mm_srai_epi32(accumB, shift);

          accumA = _mm_packs_epi32(accumA, accumB);
          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          if (j + stepX <= width)
          {
            _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
          }
          else
          {
            _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), accumA);
          }
        } //for (size_t ii = 0; ii < stepY; ii++)
      } //for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
      dst += dstStride * stepY;
    }
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//use 256 Bit Simd
#endif
}
#endif

template<X86_VEXT vext>
static void simdFilter9x9Blk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
  )
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = isChroma(compId) ? 4 : 2;

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  CHECK(blk.y % stepY, "Wrong startHeight in filtering");
  CHECK(blk.x % stepX, "Wrong startWidth in filtering");
  CHECK(height % stepY, "Wrong endHeight in filtering");
  CHECK(width % 4, "Wrong endWidth in filtering");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif

#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);

    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        __m256i params[2][2][13];
#else
        __m256i params[2][2][10];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          __m128i rawCoeffTmp[2][2][4], rawClipTmp[2][2][4], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
          __m256i rawCoeff[2][4], rawClip[2][4];
#else
          __m128i rawCoeffTmp[2][2][3], rawClipTmp[2][2][3], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
          __m256i rawCoeff[2][3], rawClip[2][3];
#endif

          for (int l = 0; l < 2; l++)
          {
            const int transposeIdx0 = pClass ? (pClass[j + 4 * k + 2 * l + 0] & 0x3) : 0;
            const int classIdx0     = pClass ? (pClass[j + 4 * k + 2 * l + 0] >> 2) : 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l * 2] = scaleFactor[k * 8 + l * 2 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoeffTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoeffTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            rawCoeffTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoeffTmp[0][l][3] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
#else
            rawCoeffTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif
            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            rawClipTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[0][l][3] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
#else
            rawClipTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif

            for (int m = 0; m < shuffleTime9[transposeIdx0]; m++)
            {
              int op0 = shuffleOp9[transposeIdx0][m][0];
              int op1 = shuffleOp9[transposeIdx0][m][1];

              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoeffTmp[0][l][op1], s1Tmp[0]));
              __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoeffTmp[0][l][op1], s3Tmp[0]));
              rawCoeffTmp[0][l][op0] = rawTmp0;
              rawCoeffTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }

            const int transposeIdx1 = pClass ? (pClass[j + 4 * k + 2 * l + 8] & 0x3) : 0;
            const int classIdx1     = pClass ? (pClass[j + 4 * k + 2 * l + 8] >> 2) : 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l * 2 + 4] = scaleFactor[k * 8 + l * 2 + 5] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoeffTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoeffTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            rawCoeffTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoeffTmp[1][l][3] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx1* MAX_NUM_ALF_LUMA_COEFF + 24));
#else
            rawCoeffTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif
            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            rawClipTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[1][l][3] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 24));
#else
            rawClipTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif

            for (int m = 0; m < shuffleTime9[transposeIdx1]; m++)
            {
              int op0 = shuffleOp9[transposeIdx1][m][0];
              int op1 = shuffleOp9[transposeIdx1][m][1];

              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0        = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoeffTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1        = _mm_or_si128(_mm_shuffle_epi8(rawCoeffTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoeffTmp[1][l][op1], s3Tmp[1]));
              rawCoeffTmp[1][l][op0] = rawTmp0;
              rawCoeffTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoeff[l][0] = _mm256_castsi128_si256( rawCoeffTmp[0][l][0]);
            rawCoeff[l][0] = _mm256_insertf128_si256(rawCoeff[l][0], rawCoeffTmp[1][l][0], 1);
            rawCoeff[l][1] = _mm256_castsi128_si256(rawCoeffTmp[0][l][1]);
            rawCoeff[l][1] = _mm256_insertf128_si256(rawCoeff[l][1], rawCoeffTmp[1][l][1], 1);
            rawCoeff[l][2] = _mm256_castsi128_si256(rawCoeffTmp[0][l][2]);
            rawCoeff[l][2] = _mm256_insertf128_si256(rawCoeff[l][2], rawCoeffTmp[1][l][2], 1);
            rawCoeff[l][3] = _mm256_castsi128_si256(rawCoeffTmp[0][l][3]);
            rawCoeff[l][3] = _mm256_insertf128_si256(rawCoeff[l][3], rawCoeffTmp[1][l][3], 1);

            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);
            rawClip[l][2] = _mm256_castsi128_si256(rawClipTmp[0][l][2]);
            rawClip[l][2] = _mm256_insertf128_si256(rawClip[l][2], rawClipTmp[1][l][2], 1);
            rawClip[l][3] = _mm256_castsi128_si256(rawClipTmp[0][l][3]);
            rawClip[l][3] = _mm256_insertf128_si256(rawClip[l][3], rawClipTmp[1][l][3], 1);
          }

          params[k][0][0] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0x00), _mm256_shuffle_epi32(rawCoeff[1][0], 0x00));
          params[k][0][1] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0x55), _mm256_shuffle_epi32(rawCoeff[1][0], 0x55));
          params[k][0][2] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0xaa), _mm256_shuffle_epi32(rawCoeff[1][0], 0xaa));
          params[k][0][3] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][0], 0xff), _mm256_shuffle_epi32(rawCoeff[1][0], 0xff));
          params[k][0][4] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0x00), _mm256_shuffle_epi32(rawCoeff[1][1], 0x00));
          params[k][0][5] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0x55), _mm256_shuffle_epi32(rawCoeff[1][1], 0x55));
          params[k][0][6] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0xaa), _mm256_shuffle_epi32(rawCoeff[1][1], 0xaa));
          params[k][0][7] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][1], 0xff), _mm256_shuffle_epi32(rawCoeff[1][1], 0xff));
          params[k][0][8] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0x00), _mm256_shuffle_epi32(rawCoeff[1][2], 0x00));
          params[k][0][9] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0x55), _mm256_shuffle_epi32(rawCoeff[1][2], 0x55));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          params[k][0][10] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0xaa), _mm256_shuffle_epi32(rawCoeff[1][2], 0xaa));
          params[k][0][11] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][2], 0xff), _mm256_shuffle_epi32(rawCoeff[1][2], 0xff));
          params[k][0][12] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoeff[0][3], 0x00), _mm256_shuffle_epi32(rawCoeff[1][3], 0x00));
#endif

          params[k][1][0] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0x00), _mm256_shuffle_epi32(rawClip[1][0], 0x00));
          params[k][1][1] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0x55), _mm256_shuffle_epi32(rawClip[1][0], 0x55));
          params[k][1][2] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0xaa), _mm256_shuffle_epi32(rawClip[1][0], 0xaa));
          params[k][1][3] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][0], 0xff), _mm256_shuffle_epi32(rawClip[1][0], 0xff));
          params[k][1][4] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0x00), _mm256_shuffle_epi32(rawClip[1][1], 0x00));
          params[k][1][5] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0x55), _mm256_shuffle_epi32(rawClip[1][1], 0x55));
          params[k][1][6] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0xaa), _mm256_shuffle_epi32(rawClip[1][1], 0xaa));
          params[k][1][7] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][1], 0xff), _mm256_shuffle_epi32(rawClip[1][1], 0xff));
          params[k][1][8] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0x00), _mm256_shuffle_epi32(rawClip[1][2], 0x00));
          params[k][1][9] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0x55), _mm256_shuffle_epi32(rawClip[1][2], 0x55));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          params[k][1][10] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0xaa), _mm256_shuffle_epi32(rawClip[1][2], 0xaa));
          params[k][1][11] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][2], 0xff), _mm256_shuffle_epi32(rawClip[1][2], 0xff));
          params[k][1][12] = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][3], 0x00), _mm256_shuffle_epi32(rawClip[1][3], 0x00));
#endif
        }

        for (size_t ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;

          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          const Pel *pImg0FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 0] + blkDst.x + j + padSize;
          const Pel *pImg1FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 1] + blkDst.x + j + padSize;
          const Pel *pImg2FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize - 1] + blkDst.x + j + padSize;
          const Pel *pImg3FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 2] + blkDst.x + j + padSize;
          const Pel *pImg4FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize - 2] + blkDst.x + j + padSize;
#endif
          __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          __m256i accumA = _mm256_setzero_si256();
          __m256i accumB = _mm256_setzero_si256();
#else
          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;
#endif

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };

          process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
          process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
          process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
          process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
          process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
          process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
          process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
          process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
          process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
          process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          process2coeffs(10, pImg3FixedBased + 0, pImg4FixedBased + 0, pImg1FixedBased + 0, pImg2FixedBased + 0);
          process2coeffs(11, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);

          __m256i val00 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize] + blkDst.x + j + padSize)), cur);
          __m256i val10    = _mm256_setzero_si256();
          __m256i val01A   = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B   = _mm256_unpackhi_epi16(val00, val10);
          __m256i limit01A = params[0][1][12];
          __m256i limit01B = params[1][1][12];

          val01A   = _mm256_min_epi16(val01A, limit01A);
          val01B   = _mm256_min_epi16(val01B, limit01B);
          limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
          limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
          val01A   = _mm256_max_epi16(val01A, limit01A);
          val01B   = _mm256_max_epi16(val01B, limit01B);

          __m256i coeff01A = params[0][0][12];
          __m256i coeff01B = params[1][0][12];
          accumA           = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
          accumB           = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
          accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

          accumA = _mm256_add_epi32(accumA, mmOffset);
          accumB = _mm256_add_epi32(accumB, mmOffset);
#endif

          accumA = _mm256_srai_epi32(accumA, shift);
          accumB = _mm256_srai_epi32(accumB, shift);

          accumA = _mm256_packs_epi32(accumA, accumB);
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (dst + ii * dstStride + j), accumA);

        }   // for (size_t ii = 0; ii < stepY; ii++)
      } // for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
      dst += dstStride * stepY;
    }
  }
  else
 {

   const __m128i mmOffset = _mm_set1_epi32(round);
   const __m128i mmMin = _mm_set1_epi16(clpRng.min);
   const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
      __m128i params[2][2][13];
#else
      __m128i params[2][2][10];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif

      for (int k = 0; k < 2; k++)
      {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        __m128i rawCoeff[2][4], rawClip[2][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoeff[2][3], rawClip[2][3], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif

        for (int l = 0; l < 2; l++)
        {
          const int transposeIdx = pClass ? (pClass[j + 4 * k + 2 * l] & 0x3 ) : 0;
          const int classIdx = pClass ? (pClass[j + 4 * k + 2 * l] >> 2): 0;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l * 2] = scaleFactor[k * 4 + l * 2 + 1] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoeff[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoeff[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          rawCoeff[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoeff[l][3] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#else
          rawCoeff[l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#else
          rawClip[l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#endif

          for (int m = 0; m < shuffleTime9[transposeIdx]; m++)
          {
            int op0 = shuffleOp9[transposeIdx][m][0];
            int op1 = shuffleOp9[transposeIdx][m][1];

            s0 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s0), _mm_shuffle_epi8(rawCoeff[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s2), _mm_shuffle_epi8(rawCoeff[l][op1], s3));
            rawCoeff[l][op0] = rawTmp0;
            rawCoeff[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }

        params[k][0][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x00), _mm_shuffle_epi32(rawCoeff[1][0], 0x00));
        params[k][0][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x55), _mm_shuffle_epi32(rawCoeff[1][0], 0x55));
        params[k][0][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xaa), _mm_shuffle_epi32(rawCoeff[1][0], 0xaa));
        params[k][0][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xff), _mm_shuffle_epi32(rawCoeff[1][0], 0xff));
        params[k][0][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x00), _mm_shuffle_epi32(rawCoeff[1][1], 0x00));
        params[k][0][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x55), _mm_shuffle_epi32(rawCoeff[1][1], 0x55));
        params[k][0][6] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xaa), _mm_shuffle_epi32(rawCoeff[1][1], 0xaa));
        params[k][0][7] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xff), _mm_shuffle_epi32(rawCoeff[1][1], 0xff));
        params[k][0][8] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x00), _mm_shuffle_epi32(rawCoeff[1][2], 0x00));
        params[k][0][9] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x55), _mm_shuffle_epi32(rawCoeff[1][2], 0x55));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        params[k][0][10] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0xaa), _mm_shuffle_epi32(rawCoeff[1][2], 0xaa));
        params[k][0][11] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0xff), _mm_shuffle_epi32(rawCoeff[1][2], 0xff));
        params[k][0][12] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][3], 0x00), _mm_shuffle_epi32(rawCoeff[1][3], 0x00));
#endif

        params[k][1][0] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x00), _mm_shuffle_epi32(rawClip[1][0], 0x00));
        params[k][1][1] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x55), _mm_shuffle_epi32(rawClip[1][0], 0x55));
        params[k][1][2] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xaa), _mm_shuffle_epi32(rawClip[1][0], 0xaa));
        params[k][1][3] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xff), _mm_shuffle_epi32(rawClip[1][0], 0xff));
        params[k][1][4] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x00), _mm_shuffle_epi32(rawClip[1][1], 0x00));
        params[k][1][5] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x55), _mm_shuffle_epi32(rawClip[1][1], 0x55));
        params[k][1][6] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xaa), _mm_shuffle_epi32(rawClip[1][1], 0xaa));
        params[k][1][7] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xff), _mm_shuffle_epi32(rawClip[1][1], 0xff));
        params[k][1][8] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x00), _mm_shuffle_epi32(rawClip[1][2], 0x00));
        params[k][1][9] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x55), _mm_shuffle_epi32(rawClip[1][2], 0x55));
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        params[k][1][10] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0xaa), _mm_shuffle_epi32(rawClip[1][2], 0xaa));
        params[k][1][11] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0xff), _mm_shuffle_epi32(rawClip[1][2], 0xff));
        params[k][1][12] = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][3], 0x00), _mm_shuffle_epi32(rawClip[1][3], 0x00));
#endif
      }

      for (size_t ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;

        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;

#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        const Pel *pImg0FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 0] + blkDst.x + j + padSize;
        const Pel *pImg1FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 1] + blkDst.x + j + padSize;
        const Pel *pImg2FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize - 1] + blkDst.x + j + padSize;
        const Pel *pImg3FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize + 2] + blkDst.x + j + padSize;
        const Pel *pImg4FixedBased = fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize - 2] + blkDst.x + j + padSize;
#endif
        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();
#else
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

          __m128i limit01A = params[0][1][i];
          __m128i limit01B = params[1][1][i];

          val01A = _mm_min_epi16(val01A, limit01A);
          val01B = _mm_min_epi16(val01B, limit01B);
          val01C = _mm_min_epi16(val01C, limit01A);
          val01D = _mm_min_epi16(val01D, limit01B);

          limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
          limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

          val01A = _mm_max_epi16(val01A, limit01A);
          val01B = _mm_max_epi16(val01B, limit01B);
          val01C = _mm_max_epi16(val01C, limit01A);
          val01D = _mm_max_epi16(val01D, limit01B);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff01A = params[0][0][i];
          const __m128i coeff01B = params[1][0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
        };

        process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
        process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
        process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
        process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
        process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
        process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
        process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
        process2coeffs(10, pImg3FixedBased, pImg4FixedBased, pImg1FixedBased, pImg2FixedBased );
        process2coeffs(11, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);

        __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[fixedFilterSetIdx][blkDst.y + i + ii + padSize] + blkDst.x + j + padSize)), cur);
        __m128i val10 = _mm_setzero_si128();
        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i limit01A = params[0][1][12];
        __m128i limit01B = params[1][1][12];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);

        __m128i coeff01A = params[0][0][12];
        __m128i coeff01B = params[1][0][12];
        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        if (j + stepX <= width)
        {
          _mm_storeu_si128((__m128i *) (dst + ii * dstStride + j), accumA);
        }
        else
        {
          _mm_storel_epi64((__m128i *) (dst + ii * dstStride + j), accumA);
        }
      } //for (size_t ii = 0; ii < stepY; ii++)
    } //for (size_t j = 0; j < width; j += stepX)
    src += srcStride * stepY;
    dst += dstStride * stepY;
  }
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//use 256 Bit Simd
#endif
}
#endif

#if ALF_IMPROVEMENT
template<X86_VEXT vext>
static void simdFilter9x9BlkExt(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
  )
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = 1;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i params[2][2][14];
#else
      __m128i params[2][2][11];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        __m128i rawCoef[4][4], rawClip[4][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoef[4][3], rawClip[4][3], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#endif
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime9FixedBasedDiamond[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime9[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp9FixedBasedDiamond[transposeIdx][m][0];
            int op1 = shuffleOp9FixedBasedDiamond[transposeIdx][m][1];
#else
            int op0 = shuffleOp9[transposeIdx][m][0];
            int op1 = shuffleOp9[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab9FixedBasedDiamond[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab9FixedBasedDiamond[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }            
        }//for l

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for (unsigned char l = 0; l < 4; l++)
#else
        for (unsigned char l = 0; l < 3; l++)
#endif
        {
          int m = l << 2;

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
          params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
          params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if (l < 3)
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
          params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
          params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if !JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if (l < 2)
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
        }//for l
      }//for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
      pImg0 = src + j;
      pImg1 = pImg0 + srcStride;
      pImg2 = pImg0 - srcStride;
      pImg3 = pImg1 + srcStride;
      pImg4 = pImg2 - srcStride;
      pImg5 = pImg3 + srcStride;
      pImg6 = pImg4 - srcStride;
      pImg7 = pImg5 + srcStride;
      pImg8 = pImg6 - srcStride;

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int filterSetIdx = 2 + fixedFilterSetIdx;
#else
        int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
        const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;

        if(isFixedFilterPaddedPerCtu )
        {
          pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
          pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
          pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
          pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
          pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
        }
        else
        {
          pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
          pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
          pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
          pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
          pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
        }
#endif

      __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
      process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
      process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
      process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize )), cur);
      const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize )), cur);
#else
      const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
      const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i limit01A = params[0][1][13];
      __m128i limit01B = params[1][1][13];
#else
      __m128i limit01A = params[0][1][10];
      __m128i limit01B = params[1][1][10];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      const __m128i coeff01A = params[0][0][13];
      const __m128i coeff01B = params[1][0][13];
#else
      const __m128i coeff01A = params[0][0][10];
      const __m128i coeff01B = params[1][0][10];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }//for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
  }//for i
}

#if JVET_AA0095_ALF_LONGER_FILTER
static const uint16_t shuffleTime13LongLength[4] = { 0, 3, 1, 3 };
static const uint16_t shuffleOp13LongLength[4][3][2] =
{
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //0
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  }, //1
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //2
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  }, //3
};
static const uint16_t shuffleTab13LongLength[4][3][2][8] =
{
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 0
  {
    {
      { sh(14), sh(15), sh(2) , sh(3) , sh(4) , sh(9) , sh(6) , sh(13) },
      { sh(8) , sh(5) , sh(10), sh(11), sh(12), sh(7) , sh(0) , sh(1)  },
    },
    {
      { sh(0) , sh(1) , sh(8), sh(9) , sh(4) , sh(5) , sh(10), sh(7) },
      { sh(2) , sh(3) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2), sh(11) , sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 1
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(7) , sh(6) , sh(5)  },
      { sh(4) , sh(13), sh(12), sh(11), sh(10), sh(9) , sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 2
  {
    {
      { sh(14), sh(15), sh(2) , sh(3) , sh(8) , sh(13), sh(6) , sh(9) },
      { sh(4) , sh(7) , sh(12), sh(11), sh(10), sh(5) , sh(0) , sh(1) },
    },
    {
      { sh(0) , sh(1) , sh(8) , sh(9) , sh(4) , sh(5) , sh(10), sh(7)  },
      { sh(2) , sh(3) , sh(6) , sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 3
};
#if FIXFILTER_CFG
static const uint16_t shuffleTime13NoFixed[4] = {0, 1, 1, 1};
static const uint16_t shuffleOp13NoFixed[4][1][2] =
{
  {
    {0, 1},
  }, //0  
  {
    {0, 1},
  }, //1
  {
    {0, 1},
  }, //2 
  {
    {0, 1},
  }, //3
};
static const uint16_t shuffleTab13NoFixed[4][1][2][8] =
{
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 0

  {
    {
      { sh(6), sh(7), sh(8) , sh(3) , sh(9) , sh(5) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 1
  {
    {
      { sh(0), sh(1), sh(2) , sh(5) , sh(4) , sh(3) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 2
  {
    {
      { sh(6), sh(7), sh(8) , sh(5) , sh(9) , sh(3) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 3
};
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
static const uint16_t shuffleTime13FixedBasedLongLength[4] = { 0, 4, 2, 4 };
static const uint16_t shuffleOp13FixedBasedLongLength[4][4][2] =
{
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  }, //0
#if JVET_AD0222_ALF_LONG_FIXFILTER
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  }, //1
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  }, //2
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  }, //3
#else
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
    { 2, 3 },
  }, //1
  {
    { 0, 1 },
    { 2, 3 },
    { 0, 1 },
    { 0, 1 },
  }, //2
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
    { 2, 3 },
  }, //3
#endif
};
static const uint16_t shuffleTab13FixedBasedLongLength[4][4][2][8] =
{
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 0
#if JVET_AD0222_ALF_LONG_FIXFILTER
  {
    {
      { sh(6), sh(7), sh(8) , sh(3) , sh(9) , sh(5) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1), sh(14), sh(15), sh(4) , sh(5), sh(9), sh(7) },
      { sh(13), sh(6), sh(10), sh(11), sh(12), sh(8), sh(2), sh(3) },
    },
    {
      { sh(0), sh(1), sh(2), sh(3) , sh(8) , sh(9) , sh(6) , sh(10) },
      { sh(4), sh(5), sh(7), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 1
  {
    {
      { sh(0), sh(1), sh(2) , sh(5) , sh(4) , sh(3) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1) , sh(2) , sh(3) , sh(4) , sh(5), sh(8) , sh(7) },
      { sh(6), sh(13), sh(12), sh(11), sh(10), sh(9), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 2
  {
    {
      { sh(6), sh(7), sh(8) , sh(5) , sh(9) , sh(3) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(14), sh(15), sh(4) , sh(5), sh(13), sh(7) },
      { sh(9), sh(8), sh(12), sh(11), sh(10), sh(6), sh(2) , sh(3) },
    },
    {
      { sh(0), sh(1), sh(2), sh(3) , sh(8) , sh(9) , sh(6) , sh(10) },
      { sh(4), sh(5), sh(7), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 3
#else
  {
    {
      { sh(14), sh(15), sh(2) , sh(3) , sh(4) , sh(9) , sh(6) , sh(13) },
      { sh(8) , sh(5) , sh(10), sh(11), sh(12), sh(7) , sh(0) , sh(1)  },
    },
    {
      { sh(0) , sh(1) , sh(8), sh(9) , sh(4) , sh(5) , sh(10), sh(7) },
      { sh(2) , sh(3) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2), sh(11) , sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(5) , sh(9) , sh(7) },
      { sh(4) , sh(6) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 1
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(7) , sh(6) , sh(5)  },
      { sh(4) , sh(13), sh(12), sh(11), sh(10), sh(9) , sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(7) , sh(6) , sh(5)  },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 2
  {
    {
      { sh(14), sh(15), sh(2) , sh(3) , sh(8) , sh(13), sh(6) , sh(9) },
      { sh(4) , sh(7) , sh(12), sh(11), sh(10), sh(5) , sh(0) , sh(1) },
    },
    {
      { sh(0) , sh(1) , sh(8) , sh(9) , sh(4) , sh(5) , sh(10), sh(7)  },
      { sh(2) , sh(3) , sh(6) , sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7)  },
      { sh(8) , sh(9) , sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(8) , sh(7) , sh(9) , sh(5) },
      { sh(4) , sh(6) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //IDX = 3
#endif
};
#endif
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
template<X86_VEXT vext>
static void simdFilter9x9BlkExtDb(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
  )
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = 1;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i params[2][2][16];
#else
      __m128i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoef[4][4], rawClip[4][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime9FixedBasedDiamond[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime9[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp9FixedBasedDiamond[transposeIdx][m][0];
            int op1 = shuffleOp9FixedBasedDiamond[transposeIdx][m][1];
#else
            int op0 = shuffleOp9[transposeIdx][m][0];
            int op1 = shuffleOp9[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab9FixedBasedDiamond[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab9FixedBasedDiamond[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab9[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }//for l

        for (unsigned char l = 0; l < 4; l++)
        {
          int m = l << 2;

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if !JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if (l < 3)
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if !JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
#endif
        }//for l
      }//for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
      pImg0 = src + j;
      pImg1 = pImg0 + srcStride;
      pImg2 = pImg0 - srcStride;
      pImg3 = pImg1 + srcStride;
      pImg4 = pImg2 - srcStride;
      pImg5 = pImg3 + srcStride;
      pImg6 = pImg4 - srcStride;
      pImg7 = pImg5 + srcStride;
      pImg8 = pImg6 - srcStride;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int filterSetIdx = 2 + fixedFilterSetIdx;
#else
      int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
      const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;

      if(isFixedFilterPaddedPerCtu )
      {
        pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
        pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
        pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
        pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
        pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
      }
      else
      {
        pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
        pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
        pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
        pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
        pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
      }
#endif

      __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg7 + 0, pImg8 + 0, pImg5 + 1, pImg6 - 1);
      process2coeffs(1, pImg5 + 0, pImg6 + 0, pImg5 - 1, pImg6 + 1);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 3, pImg2 - 3);
      process2coeffs(5, pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1);
      process2coeffs(6, pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(7, pImg1 - 2, pImg2 + 2, pImg1 - 3, pImg2 + 3);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif

      pImg0 = srcBeforeDb + j;
      pImg1 = pImg0 + srcBeforeDbStride;
      pImg2 = pImg0 - srcBeforeDbStride;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
      process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImg0), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i limit01A = params[0][1][14];
      __m128i limit01B = params[1][1][14];
#else
      __m128i limit01A = params[0][1][11];
      __m128i limit01B = params[1][1][11];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i coeff01A = params[0][0][14];
      __m128i coeff01B = params[1][0][14];
#else
      __m128i coeff01A = params[0][0][11];
      __m128i coeff01B = params[1][0][11];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
      val10 = _mm_sub_epi16(cur, cur);

      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);
#else
      __m128i val = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
      val01A = _mm_shuffle_epi8(val, _mm_setr_epi8(0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7));
      val01B = _mm_shuffle_epi8(val, _mm_setr_epi8(8, 9, 8, 9, 10, 11, 10, 11, 12, 13, 12, 13, 14, 15, 14, 15));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      limit01A = params[0][1][15];
      limit01B = params[1][1][15];
#else
      limit01A = params[0][1][12];
      limit01B = params[1][1][12];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      coeff01A = params[0][0][15];
      coeff01B = params[1][0][15];
#else
      coeff01A = params[0][0][12];
      coeff01B = params[1][0][12];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }//for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
    srcBeforeDb += srcBeforeDbStride * stepY;
  }//for i
}
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
template<X86_VEXT vext>
static void simdFilter13x13BlkExt(AlfClassifier **classifier, const PelUnitBuf &recDst, 
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
  )
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = 1;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i params[2][2][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][17];
#else
      __m128i params[2][2][14];
#endif
#else
      __m128i params[2][2][11];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoef[4][4], rawClip[4][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
#else
        __m128i rawCoef[4][3], rawClip[4][3], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
#endif
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawClip[l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime13LongLength[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];
#else
            int op0 = shuffleOp13LongLength[transposeIdx][m][0];
            int op1 = shuffleOp13LongLength[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));

            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));

            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }//for l
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        limR = 4, lim0 = 4, lim1 = 4, lim2 = 4, lim3 = 3;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#else
        limR = 4, lim0 = 4, lim1 = 4, lim2 = 3, lim3 = 3;
#endif
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for( unsigned char l = 0; l < limR; l++ )
#else
        for( unsigned char l = 0; l < 3; l++ )
#endif
        {
          int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if( l < lim0 )
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim1 )
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
          params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
          params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim2 )
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
          params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
          params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim3 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
#endif
        }//for l
      }//for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif

      pImg0 = src + j;
      pImg1 = pImg0 + srcStride;
      pImg2 = pImg0 - srcStride;
      pImg3 = pImg1 + srcStride;
      pImg4 = pImg2 - srcStride;
      pImg5 = pImg3 + srcStride;
      pImg6 = pImg4 - srcStride;
      pImg7 = pImg5 + srcStride;
      pImg8 = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      pImg9 = pImg7 + srcStride;
      pImg10 = pImg8 - srcStride;
      pImg11 = pImg9 + srcStride;
      pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int filterSetIdx = 2 + fixedFilterSetIdx;
#else
      int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
      const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
      if(isFixedFilterPaddedPerCtu )
      {
        pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
        pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
        pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
        pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
        pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
        pImg6FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
        pImg7FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
        pImg8FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
        pImg9FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
        pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
        pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
        pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
      }
      else
      {
        pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
        pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
        pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
        pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
        pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
        pImg6FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
        pImg7FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
        pImg8FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
        pImg9FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
        pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
        pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
        pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
      }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

      for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;

        }
        else
        {
          pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
        }
      }
#endif
      __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(14, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(15, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);

      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(13, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(14, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);

      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else

      const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
      const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][16];
      __m128i limit01B = params[1][1][16];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i limit01A = params[0][1][14];
      __m128i limit01B = params[1][1][14];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][15];
      __m128i limit01B = params[1][1][15];
#else
      __m128i limit01A = params[0][1][13];
      __m128i limit01B = params[1][1][13];
#endif
#else
      __m128i limit01A = params[0][1][10];
      __m128i limit01B = params[1][1][10];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][16];
      __m128i coeff01B = params[1][0][16];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i coeff01A = params[0][0][14];
      __m128i coeff01B = params[1][0][14];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][15];
      __m128i coeff01B = params[1][0][15];
#else
      const __m128i coeff01A = params[0][0][13];
      const __m128i coeff01B = params[1][0][13];
#endif
#else
      const __m128i coeff01A = params[0][0][10];
      const __m128i coeff01B = params[1][0][10];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      val00    = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
      val10    = _mm_setzero_si128();
      val01A   = _mm_unpacklo_epi16(val00, val10);
      val01B   = _mm_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER
      limit01A = params[0][1][17];
      limit01B = params[1][1][17];
#else
      limit01A = params[0][1][16];
      limit01B = params[1][1][16];
#endif
      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER
      coeff01A = params[0][0][17];
      coeff01B = params[1][0][17];
#else
      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }//for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
  }//for i
}
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
template<X86_VEXT vext>
static void simdFilter13x13BlkExtDb(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    const Pel *fClipSet
#else
    const short *fClipSet
#endif
  , const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
  )
{
  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);
  const size_t width = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t stepY = 1;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel *      dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][19];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i params[2][2][17];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][18];
#else
      __m128i params[2][2][16];
#endif
#else
      __m128i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoef[4][4], rawClip[4][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawClip[l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime13LongLength[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];
#else
            int op0 = shuffleOp13LongLength[transposeIdx][m][0];
            int op1 = shuffleOp13LongLength[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char)0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char)0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }//for l
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 4;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#else
        limR = 4, lim0 = 4, lim1 = 4, lim2 = 4, lim3 = 4;
#endif
        for (unsigned char l = 0; l < limR; l++)
#else
        for (unsigned char l = 0; l < 4; l++)
#endif
        {
          int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if( l < lim0 )
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim1 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim2 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim3 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
#endif
        }//for l
      }//for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
      pImg0 = src + j;
      pImg1 = pImg0 + srcStride;
      pImg2 = pImg0 - srcStride;
      pImg3 = pImg1 + srcStride;
      pImg4 = pImg2 - srcStride;
      pImg5 = pImg3 + srcStride;
      pImg6 = pImg4 - srcStride;
      pImg7 = pImg5 + srcStride;
      pImg8 = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      pImg9  = pImg7 + srcStride;
      pImg10 = pImg8 - srcStride;
      pImg11 = pImg9 + srcStride;
      pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int filterSetIdx = 2 + fixedFilterSetIdx;
#else
      int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
      const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
      if(isFixedFilterPaddedPerCtu )
      {
        pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
        pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
        pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
        pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
        pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
        pImg6FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
        pImg7FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
        pImg8FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
        pImg9FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
        pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
        pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
        pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
      }
      else
      {
        pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
        pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
        pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
        pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
        pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
        pImg6FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
        pImg7FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
        pImg8FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
        pImg9FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
        pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
        pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
        pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
      }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

      for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;
        }
        else
        {
          pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
        }
      }
#endif
      __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(14, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(15, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(13, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(14, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#else
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#endif
      pImg0 = srcBeforeDb + j;
      pImg1 = pImg0 + srcBeforeDbStride;
      pImg2 = pImg0 - srcBeforeDbStride;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(16, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(14, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(15, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
      process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
      process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImg0), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][17];
      __m128i limit01B = params[1][1][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i limit01A = params[0][1][15];
      __m128i limit01B = params[1][1][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][16];
      __m128i limit01B = params[1][1][16];
#else
      __m128i limit01A = params[0][1][14];
      __m128i limit01B = params[1][1][14];
#endif
#else
      __m128i limit01A = params[0][1][11];
      __m128i limit01B = params[1][1][11];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][17];
      __m128i coeff01B = params[1][0][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i coeff01A = params[0][0][15];
      __m128i coeff01B = params[1][0][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][16];
      __m128i coeff01B = params[1][0][16];
#else
      __m128i coeff01A = params[0][0][14];
      __m128i coeff01B = params[1][0][14];
#endif
#else
      __m128i coeff01A = params[0][0][11];
      __m128i coeff01B = params[1][0][11];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
#else
      val10 = _mm_sub_epi16(cur, cur);
#endif
      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);
#else
      __m128i val = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
      val01A = _mm_shuffle_epi8(val, _mm_setr_epi8(0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7));
      val01B = _mm_shuffle_epi8(val, _mm_setr_epi8(8, 9, 8, 9, 10, 11, 10, 11, 12, 13, 12, 13, 14, 15, 14, 15));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      limit01A = params[0][1][18];
      limit01B = params[1][1][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      limit01A = params[0][1][16];
      limit01B = params[1][1][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      limit01A = params[0][1][17];
      limit01B = params[1][1][17];
#else
      limit01A = params[0][1][15];
      limit01B = params[1][1][15];
#endif
#else
      limit01A = params[0][1][12];
      limit01B = params[1][1][12];
#endif
      val01A = _mm_min_epi16(val01A, limit01A);
      val01B = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A = _mm_max_epi16(val01A, limit01A);
      val01B = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      coeff01A = params[0][0][18];
      coeff01B = params[1][0][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      coeff01A = params[0][0][17];
      coeff01B = params[1][0][17];
#else
      coeff01A = params[0][0][15];
      coeff01B = params[1][0][15];
#endif
#else
      coeff01A = params[0][0][12];
      coeff01B = params[1][0][12];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }//for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
    srcBeforeDb += srcBeforeDbStride * stepY;
  }//for i
}
#endif

#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
template<X86_VEXT vext>
static void simdFilter13x13BlkExtDbResiDirect(
  AlfClassifier * *classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi,
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  ,const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  ,Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
  const CPelBuf srcBuffer         = recSrc.get(compId);
  PelBuf        dstBuffer         = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);
  const CPelBuf scrBufferResi     = resi.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
  const size_t srcResiStride     = scrBufferResi.stride;
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int adjustShift = coeffBits - 1;
  const bool  bScalingCorr = isLuma(compId) && fixedFilterSetIdx < 0;
  if ( bScalingCorr )
  {
    fixedFilterSetIdx = -fixedFilterSetIdx - 1;
    adjustShift -= shiftPrecis; // add more precision
  }
  int shift = adjustShift;
#if JVET_AJ0237_INTERNAL_12BIT
  const Pel currBase = 1 << (clpRng.bd - 1);
#else
  const Pel currBase = 512;
#endif

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  __m128i curBase = _mm_set_epi16( currBase, currBase, currBase, currBase, currBase, currBase, currBase, currBase );
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t           stepY = 1;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#endif

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = scrBufferResi.buf + blk.y * srcResiStride + blk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m256i curBase  = _mm256_set1_epi16(currBase);
#endif

    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i params[2][2][20];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        __m256i params[2][2][17];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i params[2][2][19];
#else
        __m256i params[2][2][16];
#endif
#else
        __m256i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          __m256i rawCoef[4][5], rawClip[4][5], s0, s1;
          __m128i rawCoefTmp[2][4][5], rawClipTmp[2][4][5], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
#else
          __m256i rawCoef[4][4], rawClip[4][4], s0, s1;
          __m128i rawCoefTmp[2][4][4], rawClipTmp[2][4][4], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
#endif
          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx0 = pClass[j + 4 * k + l + 0] & 0x3;
            const int classIdx0     = pClass[j + 4 * k + l + 0] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoefTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoefTmp[0][l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            rawCoefTmp[0][l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[0][l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            rawClipTmp[0][l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx0]; m++)
#else
            for (int m = 0; m < shuffleTime13LongLength[transposeIdx0]; m++)
#endif
            {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              int op0 = shuffleOp13FixedBasedLongLength[transposeIdx0][m][0];
              int op1 = shuffleOp13FixedBasedLongLength[transposeIdx0][m][1];
#else
              int op0 = shuffleOp13LongLength[transposeIdx0][m][0];
              int op1 = shuffleOp13LongLength[transposeIdx0][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));
#else
              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));
#endif

             __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s1Tmp[0]));
             __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s3Tmp[0]));
              rawCoefTmp[0][l][op0] = rawTmp0;
              rawCoefTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }

            const int transposeIdx1 = pClass[j + 4 * k + l + 8] & 0x3;
            const int classIdx1     = pClass[j + 4 * k + l + 8] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l + 4] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoefTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoefTmp[1][l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            rawCoefTmp[1][l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[1][l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            rawClipTmp[1][l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx1]; m++)
#else
            for (int m = 0; m < shuffleTime13LongLength[transposeIdx1]; m++)
#endif
            {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              int op0 = shuffleOp13FixedBasedLongLength[transposeIdx1][m][0];
              int op1 = shuffleOp13FixedBasedLongLength[transposeIdx1][m][1];
#else
              int op0 = shuffleOp13LongLength[transposeIdx1][m][0];
              int op1 = shuffleOp13LongLength[transposeIdx1][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));
#else
              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));
#endif

              __m128i rawTmp0       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s3Tmp[1]));
              rawCoefTmp[1][l][op0] = rawTmp0;
              rawCoefTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoef[l][0] = _mm256_castsi128_si256(rawCoefTmp[0][l][0]);
            rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[1][l][0], 1);
            rawCoef[l][1] = _mm256_castsi128_si256(rawCoefTmp[0][l][1]);
            rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[1][l][1], 1);
            rawCoef[l][2] = _mm256_castsi128_si256(rawCoefTmp[0][l][2]);
            rawCoef[l][2] = _mm256_insertf128_si256(rawCoef[l][2], rawCoefTmp[1][l][2], 1);
            rawCoef[l][3] = _mm256_castsi128_si256(rawCoefTmp[0][l][3]);
            rawCoef[l][3] = _mm256_insertf128_si256(rawCoef[l][3], rawCoefTmp[1][l][3], 1);
            rawCoef[l][4] = _mm256_castsi128_si256(rawCoefTmp[0][l][4]);
            rawCoef[l][4] = _mm256_insertf128_si256(rawCoef[l][4], rawCoefTmp[1][l][4], 1);

            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);
            rawClip[l][2] = _mm256_castsi128_si256(rawClipTmp[0][l][2]);
            rawClip[l][2] = _mm256_insertf128_si256(rawClip[l][2], rawClipTmp[1][l][2], 1);
            rawClip[l][3] = _mm256_castsi128_si256(rawClipTmp[0][l][3]);
            rawClip[l][3] = _mm256_insertf128_si256(rawClip[l][3], rawClipTmp[1][l][3], 1);
            rawClip[l][4] = _mm256_castsi128_si256(rawClipTmp[0][l][4]);
            rawClip[l][4] = _mm256_insertf128_si256(rawClip[l][4], rawClipTmp[1][l][4], 1);
          }   // for l
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 5;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
          limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 4;
#else
          limR = 4, lim0 = 4, lim1 = 4, lim2 = 4, lim3 = 4;
#endif
          for (unsigned char l = 0; l < limR; l++)
#else
          for (unsigned char l = 0; l < 4; l++)
#endif
          {
            int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if (l < lim0)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
              params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
              params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim1)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
              params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
              params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim2)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
              params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
              params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim3)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
              params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
              params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
#endif
          }   // for l
        }     // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
        const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
        const Pel *pImgP0;

        pImg0 = src + j;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
        pImg9  = pImg7 + srcStride;
        pImg10 = pImg8 - srcStride;
        pImg11 = pImg9 + srcStride;
        pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        int filterSetIdx = 2 + fixedFilterSetIdx;
#else
        int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
        const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased,
          *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
        if (isFixedFilterPaddedPerCtu)
        {
          pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
          pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
          pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
          pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
          pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
          pImg5FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
          pImg6FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
          pImg7FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
          pImg8FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
          pImg9FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
          pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
          pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
          pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
        }
        else
        {
          pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
          pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
          pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
          pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
          pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
          pImg5FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
          pImg6FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
          pImg7FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
          pImg8FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
          pImg9FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
          pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
          pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
          pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
        }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for (int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
        {
          if (isFixedFilterPaddedPerCtu)
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
          }
        }
#endif
        __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m256i accumA = _mm256_setzero_si256();
        __m256i accumB = _mm256_setzero_si256();
#else
        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          __m256i limit01A = params[0][1][i];
          __m256i limit01B = params[1][1][i];

          val01A = _mm256_min_epi16(val01A, limit01A);
          val01B = _mm256_min_epi16(val01B, limit01B);
          val01C = _mm256_min_epi16(val01C, limit01A);
          val01D = _mm256_min_epi16(val01D, limit01B);

          limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
          limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

          val01A = _mm256_max_epi16(val01A, limit01A);
          val01B = _mm256_max_epi16(val01B, limit01B);
          val01C = _mm256_max_epi16(val01C, limit01A);
          val01D = _mm256_max_epi16(val01D, limit01B);

          val01A = _mm256_add_epi16(val01A, val01C);
          val01B = _mm256_add_epi16(val01B, val01D);

          const __m256i coeff01A = params[0][0][i];
          const __m256i coeff01B = params[1][0][i];

          accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
          accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        };
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
        process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
        process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
        process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
        process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
        process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
        process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
        process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
        process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
        process2coeffs(14, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
        process2coeffs(15, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
        process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
        process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
        process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
        process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
        process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
        process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
        process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
        process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
        process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
        process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
        process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
        process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
        process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
        process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
        process2coeffs(13, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
        process2coeffs(14, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#else
        process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
        process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
        process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
        process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
        process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
        process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
        process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#endif
        pImg0 = srcBeforeDb + j;
        pImg1 = pImg0 + srcBeforeDbStride;
        pImg2 = pImg0 - srcBeforeDbStride;

        pImgP0 = srcResi + j;

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        process2coeffs(16, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        process2coeffs(14, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        process2coeffs(15, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
        process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
        process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        __m256i val00 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)),  cur);
        __m256i val10 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)),  cur);
#else
        __m256i val00 = _mm265_sub_epi16(_mm256_loadu_si256((const __m256i *) pImg0), cur);
        __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
        __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
        __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i limit01A = params[0][1][17];
        __m256i limit01B = params[1][1][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        __m256i limit01A = params[0][1][15];
        __m256i limit01B = params[1][1][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i limit01A = params[0][1][16];
        __m256i limit01B = params[1][1][16];
#else
        __m256i limit01A = params[0][1][14];
        __m256i limit01B = params[1][1][14];
#endif
#else
        __m256i limit01A = params[0][1][11];
        __m256i limit01B = params[1][1][11];
#endif
        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i coeff01A = params[0][0][17];
        __m256i coeff01B = params[1][0][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        __m256i coeff01A = params[0][0][15];
        __m256i coeff01B = params[1][0][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m256i coeff01A = params[0][0][16];
        __m256i coeff01B = params[1][0][16];
#else
        __m256i coeff01A = params[0][0][14];
        __m256i coeff01B = params[1][0][14];
#endif
#else
        __m256i coeff01A = params[0][0][11];
        __m256i coeff01B = params[1][0][11];
#endif
        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));

        // start prediction fixed filter
        __m256i zero = _mm256_setzero_si256();
        val00        = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0)), cur);
        val10        = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) pImgP0), zero);
        val01A       = _mm256_unpacklo_epi16(val00, val10);
        val01B       = _mm256_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limit01A = params[0][1][18];
        limit01B = params[1][1][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        limit01A = params[0][1][16];
        limit01B = params[1][1][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limit01A = params[0][1][17];
        limit01B = params[1][1][17];
#else
        limit01A = params[0][1][15];
        limit01B = params[1][1][15];
#endif

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        coeff01A = params[0][0][18];
        coeff01B = params[1][0][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        coeff01A = params[0][0][16];
        coeff01B = params[1][0][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        coeff01A = params[0][0][17];
        coeff01B = params[1][0][17];
#else
        coeff01A = params[0][0][15];
        coeff01B = params[1][0][15];
#endif

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        // end prediction fixed filter
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        val00  = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0Gauss[0])), cur);
        val10  = _mm256_setzero_si256();
        val01A = _mm256_unpacklo_epi16(val00, val10);
        val01B = _mm256_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER
        limit01A = params[0][1][19];
        limit01B = params[1][1][19];
#else
        limit01A = params[0][1][18];
        limit01B = params[1][1][18];
#endif

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER
        coeff01A = params[0][0][19];
        coeff01B = params[1][0][19];
#else
        coeff01A = params[0][0][18];
        coeff01B = params[1][0][18];
#endif
        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
        accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

        accumA = _mm256_add_epi32(accumA, mmOffset);
        accumB = _mm256_add_epi32(accumB, mmOffset);
#endif
        accumA = _mm256_srai_epi32(accumA, shift);
        accumB = _mm256_srai_epi32(accumB, shift);

        accumA = _mm256_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if( bScalingCorr )
        {
          accumA = _mm256_add_epi16(accumA, curBase);
        }
        else
#endif
        accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        _mm256_storeu_si256((__m256i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
  }
  else
  {

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  const __m128i curBase  = _mm_set1_epi16(currBase);
#endif
#endif//Use AVX2 SIMD
  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][20];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i params[2][2][17];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i params[2][2][19];
#else
      __m128i params[2][2][16];
#endif
#else
      __m128i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoef[4][4], rawClip[4][4], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx     = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          rawClip[l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime13LongLength[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];
#else
            int op0 = shuffleOp13LongLength[transposeIdx][m][0];
            int op1 = shuffleOp13LongLength[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }   // for l
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 5;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 4;
#else
        limR = 4, lim0 = 4, lim1 = 4, lim2 = 4, lim3 = 4;
#endif
        for (unsigned char l = 0; l < limR; l++)
#else
        for (unsigned char l = 0; l < 4; l++)
#endif
        {
          int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if (l < lim0)
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if (l < lim1)
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if (l < lim2)
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if (l < lim3)
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
#endif
        }   // for l
      }     // for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
      const Pel *pImgP0;

      pImg0  = src + j;
      pImg1  = pImg0 + srcStride;
      pImg2  = pImg0 - srcStride;
      pImg3  = pImg1 + srcStride;
      pImg4  = pImg2 - srcStride;
      pImg5  = pImg3 + srcStride;
      pImg6  = pImg4 - srcStride;
      pImg7  = pImg5 + srcStride;
      pImg8  = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      pImg9  = pImg7 + srcStride;
      pImg10 = pImg8 - srcStride;
      pImg11 = pImg9 + srcStride;
      pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int        filterSetIdx = 2 + fixedFilterSetIdx;
#else
      int        filterSetIdx = 0 + fixedFilterSetIdx;
#endif
      const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
      if (isFixedFilterPaddedPerCtu)
      {
        pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
        pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
        pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
        pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
        pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
        pImg6FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
        pImg7FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
        pImg8FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
        pImg9FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
        pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
        pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
        pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
      }
      else
      {
        pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
        pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
        pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
        pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
        pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
        pImg6FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
        pImg7FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
        pImg8FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
        pImg9FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
        pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
        pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
        pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
      }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
      const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

      for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;
        }
        else
        {
          pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
          pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
          pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
          pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
        }
      }
#endif
      __m128i cur    = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(14, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(15, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs(13, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
      process2coeffs(14, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);
#else
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#endif
      pImg0 = srcBeforeDb + j;
      pImg1 = pImg0 + srcBeforeDbStride;
      pImg2 = pImg0 - srcBeforeDbStride;

      pImgP0 = srcResi + j;

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(16, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(14, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(15, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
      process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
      process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)),
        cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)),
        cur);
#else
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImg0), cur);
      __m128i val10 = _mm_sub_epi16(
        _mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][17];
      __m128i limit01B = params[1][1][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i limit01A = params[0][1][15];
      __m128i limit01B = params[1][1][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i limit01A = params[0][1][16];
      __m128i limit01B = params[1][1][16];
#else
      __m128i limit01A = params[0][1][14];
      __m128i limit01B = params[1][1][14];
#endif
#else
      __m128i limit01A = params[0][1][11];
      __m128i limit01B = params[1][1][11];
#endif
      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][17];
      __m128i coeff01B = params[1][0][17];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      __m128i coeff01A = params[0][0][15];
      __m128i coeff01B = params[1][0][15];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      __m128i coeff01A = params[0][0][16];
      __m128i coeff01B = params[1][0][16];
#else
      __m128i coeff01A = params[0][0][14];
      __m128i coeff01B = params[1][0][14];
#endif
#else
      __m128i coeff01A = params[0][0][11];
      __m128i coeff01B = params[1][0][11];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      // start prediction fixed filter
      __m128i zero = _mm_setzero_si128();
      val00        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
      val10        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImgP0), zero);
      val01A       = _mm_unpacklo_epi16(val00, val10);
      val01B       = _mm_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      limit01A = params[0][1][18];
      limit01B = params[1][1][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      limit01A = params[0][1][16];
      limit01B = params[1][1][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      limit01A = params[0][1][17];
      limit01B = params[1][1][17];
#else
      limit01A = params[0][1][15];
      limit01B = params[1][1][15];
#endif

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      coeff01A = params[0][0][18];
      coeff01B = params[1][0][18];
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      coeff01A = params[0][0][17];
      coeff01B = params[1][0][17];
#else
      coeff01A = params[0][0][15];
      coeff01B = params[1][0][15];
#endif

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      // end prediction fixed filter
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      val00    = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
      val10    = _mm_setzero_si128();
      val01A   = _mm_unpacklo_epi16(val00, val10);
      val01B   = _mm_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER
      limit01A = params[0][1][19];
      limit01B = params[1][1][19];
#else
      limit01A = params[0][1][18];
      limit01B = params[1][1][18];
#endif

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER
      coeff01A = params[0][0][19];
      coeff01B = params[1][0][19];
#else
      coeff01A = params[0][0][18];
      coeff01B = params[1][0][18];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
      if ( bScalingCorr )
      {
        accumA = _mm_add_epi16(accumA, curBase);
      }
      else
#endif
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }   // for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
    srcBeforeDb += srcBeforeDbStride * stepY;
    srcResi += srcResiStride * stepY;
  }   // for i
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//Use 256 Bit Simd
 #endif
}

template<X86_VEXT vext>
static void simdFilter13x13BlkExtDbResi(
  AlfClassifier * *classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi,
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  ,const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  ,Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
  const CPelBuf srcBuffer         = recSrc.get(compId);
  PelBuf        dstBuffer         = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);
  const CPelBuf scrBufferResi     = resi.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
  const size_t srcResiStride     = scrBufferResi.stride;
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int adjustShift = coeffBits - 1;
  const bool  bScalingCorr = isLuma(compId) && fixedFilterSetIdx < 0;
  if ( bScalingCorr )
  {
    fixedFilterSetIdx = -fixedFilterSetIdx - 1;
    adjustShift -= shiftPrecis; // add more precision
  }
  int shift = adjustShift;
#if JVET_AJ0237_INTERNAL_12BIT
  const Pel currBase = 1 << (clpRng.bd - 1);
#else
  const Pel currBase = 512;
#endif

#if !( USE_AVX2 && (JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF) )
  __m128i curBase = _mm_set_epi16( currBase, currBase, currBase, currBase, currBase, currBase, currBase, currBase );
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t           stepY = 1;

#if !( USE_AVX2 && (JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF) )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#endif

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = scrBufferResi.buf + blk.y * srcResiStride + blk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  const int padSizeLaplacian = ALF_PADDING_SIZE_LAPLACIAN_RESULTS;
#endif

#if USE_AVX2 && (JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF)
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m256i curBase  = _mm256_set1_epi16(currBase);
#endif
    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m256i params[2][2][21];
#else
        __m256i params[2][2][18];
#endif
#else
        __m256i params[2][2][17];
#endif
#else
        __m256i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          __m256i rawCoef[4][6], rawClip[4][6], s0, s1;
          __m128i rawCoefTmp[2][4][6], rawClipTmp[2][4][6], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
#else
          __m256i rawCoef[4][5], rawClip[4][5], s0, s1;
          __m128i rawCoefTmp[2][4][5], rawClipTmp[2][4][5], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
#endif
          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx0 = pClass[j + 4 * k + l + 0] & 0x3;
            const int classIdx0     = pClass[j + 4 * k + l + 0] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoefTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoefTmp[0][l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
            rawCoefTmp[0][l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawCoefTmp[0][l][5] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[0][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[0][l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 24));
            rawClipTmp[0][l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawClipTmp[0][l][5] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

            const int transposeIdx1 = pClass[j + 4 * k + l + 8] & 0x3;
            const int classIdx1     = pClass[j + 4 * k + l + 8] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l + 4] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoefTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawCoefTmp[1][l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 24));
            rawCoefTmp[1][l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawCoefTmp[1][l][5] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[1][l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));
            rawClipTmp[1][l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 24));
            rawClipTmp[1][l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawClipTmp[1][l][5] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx0]; m++)
#else
            for (int m = 0; m < shuffleTime13LongLength[transposeIdx0]; m++)
#endif
            {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              int op0 = shuffleOp13FixedBasedLongLength[transposeIdx0][m][0];
              int op1 = shuffleOp13FixedBasedLongLength[transposeIdx0][m][1];
#else
              int op0 = shuffleOp13LongLength[transposeIdx0][m][0];
              int op1 = shuffleOp13LongLength[transposeIdx0][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));
#else
              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));
#endif

              __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s1Tmp[0]));
              __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s3Tmp[0]));
              rawCoefTmp[0][l][op0] = rawTmp0;
              rawCoefTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx1]; m++)
#else
            for (int m = 0; m < shuffleTime13LongLength[transposeIdx1]; m++)
#endif
            {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              int op0 = shuffleOp13FixedBasedLongLength[transposeIdx1][m][0];
              int op1 = shuffleOp13FixedBasedLongLength[transposeIdx1][m][1];
#else
              int op0 = shuffleOp13LongLength[transposeIdx1][m][0];
              int op1 = shuffleOp13LongLength[transposeIdx1][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));
#else
              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));
#endif

              __m128i rawTmp0       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s3Tmp[1]));
              rawCoefTmp[1][l][op0] = rawTmp0;
              rawCoefTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoef[l][0] = _mm256_castsi128_si256(rawCoefTmp[0][l][0] );
            rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[1][l][0], 1 );
            rawCoef[l][1] = _mm256_castsi128_si256(rawCoefTmp[0][l][1]);
            rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[1][l][1], 1);
            rawCoef[l][2] = _mm256_castsi128_si256(rawCoefTmp[0][l][2]);
            rawCoef[l][2] = _mm256_insertf128_si256(rawCoef[l][2], rawCoefTmp[1][l][2], 1);
            rawCoef[l][3] = _mm256_castsi128_si256(rawCoefTmp[0][l][3]);
            rawCoef[l][3] = _mm256_insertf128_si256(rawCoef[l][3], rawCoefTmp[1][l][3], 1);
            rawCoef[l][4] = _mm256_castsi128_si256(rawCoefTmp[0][l][4]);
            rawCoef[l][4] = _mm256_insertf128_si256(rawCoef[l][4], rawCoefTmp[1][l][4], 1);
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawCoef[l][5] = _mm256_castsi128_si256(rawCoefTmp[0][l][5]);
            rawCoef[l][5] = _mm256_insertf128_si256(rawCoef[l][5], rawCoefTmp[1][l][5], 1);
#endif

            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);
            rawClip[l][2] = _mm256_castsi128_si256(rawClipTmp[0][l][2]);
            rawClip[l][2] = _mm256_insertf128_si256(rawClip[l][2], rawClipTmp[1][l][2], 1);
            rawClip[l][3] = _mm256_castsi128_si256(rawClipTmp[0][l][3]);
            rawClip[l][3] = _mm256_insertf128_si256(rawClip[l][3], rawClipTmp[1][l][3], 1);
            rawClip[l][4] = _mm256_castsi128_si256(rawClipTmp[0][l][4]);
            rawClip[l][4] = _mm256_insertf128_si256(rawClip[l][4], rawClipTmp[1][l][4], 1);
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            rawClip[l][5] = _mm256_castsi128_si256(rawClipTmp[0][l][5]);
            rawClip[l][5] = _mm256_insertf128_si256(rawClip[l][5], rawClipTmp[1][l][5], 1);
#endif
          }   // for l

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          limR = 6, lim0 = 6, lim1 = 5, lim2 = 5, lim3 = 5;
#else
          limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
          limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#else
          limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#endif
          for (unsigned char l = 0; l < limR; l++)
#else
          for (unsigned char l = 0; l < 5; l++)
#endif
          {
            int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if (l < lim0)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
              params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
              params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim1)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
              params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
              params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim2)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
              params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
              params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
            if (l < lim3)
            {
#endif
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
              params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
              params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            }
#endif
          }   // for l
        } // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
        const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
        const Pel *pImgP0;

        pImg0 = src + j;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
        pImg9  = pImg7 + srcStride;
        pImg10 = pImg8 - srcStride;
        pImg11 = pImg9 + srcStride;
        pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        int filterSetIdx = 2 + fixedFilterSetIdx;
#else
        int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
        const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased,
          *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
        if (isFixedFilterPaddedPerCtu)
        {
          pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
          pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
          pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
          pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
          pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
          pImg5FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
          pImg6FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
          pImg7FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
          pImg8FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
          pImg9FixedBased  = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
          pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
          pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
          pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
        }
        else
        {
          pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
          pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
          pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
          pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
          pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
          pImg5FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
          pImg6FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
          pImg7FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
          pImg8FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
          pImg9FixedBased  = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
          pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
          pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
          pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
        }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for (int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
        {
          if (isFixedFilterPaddedPerCtu)
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          }
        }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      const Pel *pImg0Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
      const Pel *pImg1Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg2Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
      const Pel *pImg3Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg4Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];

      for( int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 0] + j + padSizeLaplacian;
          pImg1Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 1] + j + padSizeLaplacian;
          pImg2Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian - 1] + j + padSizeLaplacian;
          pImg3Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 2] + j + padSizeLaplacian;
          pImg4Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian - 2] + j + padSizeLaplacian;
        }
        else
        {
          pImg0Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 0] + blkDst.x + j + padSizeLaplacian;
          pImg1Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 1] + blkDst.x + j + padSizeLaplacian;
          pImg2Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian - 1] + blkDst.x + j + padSizeLaplacian;
          pImg3Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 2] + blkDst.x + j + padSizeLaplacian;
          pImg4Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian - 2] + blkDst.x + j + padSizeLaplacian;
        }
      }
#endif
        __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m256i accumA = _mm256_setzero_si256();
        __m256i accumB = _mm256_setzero_si256();
#else
        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m256i zero = _mm256_setzero_si256();
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          __m256i limit01A = params[0][1][i];
          __m256i limit01B = params[1][1][i];

          val01A = _mm256_min_epi16(val01A, limit01A);
          val01B = _mm256_min_epi16(val01B, limit01B);
          val01C = _mm256_min_epi16(val01C, limit01A);
          val01D = _mm256_min_epi16(val01D, limit01B);

          limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
          limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

          val01A = _mm256_max_epi16(val01A, limit01A);
          val01B = _mm256_max_epi16(val01B, limit01B);
          val01C = _mm256_max_epi16(val01C, limit01A);
          val01D = _mm256_max_epi16(val01D, limit01B);

          val01A = _mm256_add_epi16(val01A, val01C);
          val01B = _mm256_add_epi16(val01B, val01D);

          const __m256i coeff01A = params[0][0][i];
          const __m256i coeff01B = params[1][0][i];

          accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
          accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        };
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF 
        auto process2coeffs_laplacian = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), zero);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), zero);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), zero);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), zero);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          __m256i limit01A = params[0][1][i];
          __m256i limit01B = params[1][1][i];

          val01A = _mm256_min_epi16(val01A, limit01A);
          val01B = _mm256_min_epi16(val01B, limit01B);
          val01C = _mm256_min_epi16(val01C, limit01A);
          val01D = _mm256_min_epi16(val01D, limit01B);

          limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
          limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

          val01A = _mm256_max_epi16(val01A, limit01A);
          val01B = _mm256_max_epi16(val01B, limit01B);
          val01C = _mm256_max_epi16(val01C, limit01A);
          val01D = _mm256_max_epi16(val01D, limit01B);

          val01A = _mm256_add_epi16(val01A, val01C);
          val01B = _mm256_add_epi16(val01B, val01D);

          const __m256i coeff01A = params[0][0][i];
          const __m256i coeff01B = params[1][0][i];

          accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
          accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        };
#endif
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
        process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
        process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
        process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
        process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
        process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
        process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
        process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
        process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
        process2coeffs_laplacian(14, pImg3Laplacian[0] - 0, pImg4Laplacian[0] + 0, pImg1Laplacian[0] - 0, pImg2Laplacian[0] + 0);
        process2coeffs_laplacian(15, pImg0Laplacian[0] - 2, pImg0Laplacian[0] + 2, pImg0Laplacian[0] - 1, pImg0Laplacian[0] + 1);
#else
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
        process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
        process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
        process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
        process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
        process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
        process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
        process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
        process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
        process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
        process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
        process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
        process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
        process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
        process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
        process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
        process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
        process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
        process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
        process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
        process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
        process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
        process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#else
        process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
        process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
        process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
        process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
        process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
        process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
        process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
        process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#endif
        pImg0  = srcBeforeDb + j;
        pImg1  = pImg0 + srcBeforeDbStride;
        pImg2  = pImg0 - srcBeforeDbStride;
        pImgP0 = srcResi + j;

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        process2coeffs(16, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
        process2coeffs(14, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
        process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
        process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        __m256i val00 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
        __m256i val10 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else
        __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) pImg0), cur);
        __m256i val10 = _mm256_sub_epi16( _mm256_loadu_si256((const __m256i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
        __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
        __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m256i limit01A = params[0][1][17];
        __m256i limit01B = params[1][1][17];
#else
        __m256i limit01A = params[0][1][15];
        __m256i limit01B = params[1][1][15];
#endif
#else
        __m256i limit01A = params[0][1][14];
        __m256i limit01B = params[1][1][14];
#endif
#else
        __m256i limit01A = params[0][1][11];
        __m256i limit01B = params[1][1][11];
#endif
        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m256i coeff01A = params[0][0][17];
        __m256i coeff01B = params[1][0][17];
#else
        __m256i coeff01A = params[0][0][15];
        __m256i coeff01B = params[1][0][15];
#endif
#else
        __m256i coeff01A = params[0][0][14];
        __m256i coeff01B = params[1][0][14];
#endif
#else
        __m256i coeff01A = params[0][0][11];
        __m256i coeff01B = params[1][0][11];
#endif
        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));

        // start residual fixed filter
#if !JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m256i zero = _mm256_setzero_si256();
#endif
        val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (fixedFilterResiResults[1 - fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), zero);
        val10  = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0)), cur);
        val01A = _mm256_unpacklo_epi16(val00, val10);
        val01B = _mm256_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        limit01A = params[0][1][18];
        limit01B = params[1][1][18];
#else
        limit01A = params[0][1][16];
        limit01B = params[1][1][16];
#endif
#else
        limit01A = params[0][1][15];
        limit01B = params[1][1][15];
#endif

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        coeff01A = params[0][0][18];
        coeff01B = params[1][0][18];
#else
        coeff01A = params[0][0][16];
        coeff01B = params[1][0][16];
#endif
#else
        coeff01A = params[0][0][15];
        coeff01B = params[1][0][15];
#endif

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        // end residual fixed filter

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) pImgP0), zero);
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0Gauss[0])), cur);
#else
        val10 = _mm256_sub_epi16(cur, cur);
#endif

        val01A = _mm256_unpacklo_epi16(val00, val10);
        val01B = _mm256_unpackhi_epi16(val00, val10);
#else
        __m256i val = _mm256_sub_epi16( _mm256_loadu_si256( (const __m256i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
        val01A = _mm256_shuffle_epi8(val, _mm256_setr_epi8(0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7));
        val01B = _mm256_shuffle_epi8(val, _mm256_setr_epi8(8, 9, 8, 9, 10, 11, 10, 11, 12, 13, 12, 13, 14, 15, 14, 15, 8, 9, 8, 9, 10, 11, 10, 11, 12, 13, 12, 13, 14, 15, 14, 15));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        limit01A = params[0][1][19];
        limit01B = params[1][1][19];
#else
        limit01A = params[0][1][17];
        limit01B = params[1][1][17];
#endif
#else
        limit01A = params[0][1][16];
        limit01B = params[1][1][16];
#endif
#else
        limit01A = params[0][1][12];
        limit01B = params[1][1][12];
#endif
        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        coeff01A = params[0][0][19];
        coeff01B = params[1][0][19];
#else
        coeff01A = params[0][0][17];
        coeff01B = params[1][0][17];
#endif
#else
        coeff01A = params[0][0][16];
        coeff01B = params[1][0][16];
#endif
#else
        coeff01A = params[0][0][12];
        coeff01B = params[1][0][12];
#endif
        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));

#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        val00    = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0Laplacian[0])), zero);
        val10    = _mm256_setzero_si256();
        val01A   = _mm256_unpacklo_epi16(val00, val10);
        val01B   = _mm256_unpackhi_epi16(val00, val10);

        limit01A = params[0][1][20];
        limit01B = params[1][1][20];

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);

        coeff01A = params[0][0][20];
        coeff01B = params[1][0][20];

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
        accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

        accumA = _mm256_add_epi32(accumA, mmOffset);
        accumB = _mm256_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm256_srai_epi32(accumA, shift);
        accumB = _mm256_srai_epi32(accumB, shift);

        accumA = _mm256_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if( bScalingCorr )
        {
          accumA = _mm256_add_epi16(accumA, curBase);
        }
        else
#endif
        accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        _mm256_storeu_si256((__m256i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
  }
  else
  {

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  const __m128i curBase  = _mm_set1_epi16(currBase);
#endif
#endif //Use AVX2 SIMD
  for (size_t i = 0; i < height; i += stepY)
  {
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += stepX)
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      __m128i params[2][2][21];
#else
      __m128i params[2][2][18];
#endif
#else
      __m128i params[2][2][17];
#endif
#else
      __m128i params[2][2][13];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int32_t scaleFactor[8];
#endif
      for (int k = 0; k < 2; k++)
      {
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        __m128i rawCoef[4][6], rawClip[4][6], s0, s1, s2, s3, rawTmp0, rawTmp1;
#else
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
#endif
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx     = pClass[j + 4 * k + l] >> 2;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
          scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          rawCoef[l][5] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawClip[l][4] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          rawClip[l][5] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 40));
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
#else
          for (int m = 0; m < shuffleTime13LongLength[transposeIdx]; m++)
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];
#else
            int op0 = shuffleOp13LongLength[transposeIdx][m][0];
            int op1 = shuffleOp13LongLength[transposeIdx][m][1];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));
#else
            s0 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
            s2 = _mm_loadu_si128((const __m128i *) shuffleTab13LongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));
#endif

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }   // for l
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        int limR, lim0, lim1, lim2, lim3;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        limR = 6, lim0 = 6, lim1 = 5, lim2 = 5, lim3 = 5;
#else
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#else
        limR = 5, lim0 = 5, lim1 = 4, lim2 = 4, lim3 = 4;
#endif
        for (unsigned char l = 0; l < limR; l++)
#else
        for (unsigned char l = 0; l < 5; l++)
#endif
        {
          int m = l << 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if( l < lim0 )
          {
#endif
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim1 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim2 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
          if( l < lim3 )
          {
#endif
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          }
#endif
        }   // for l
      }     // for k

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
      const Pel *pImgP0;

      pImg0  = src + j;
      pImg1  = pImg0 + srcStride;
      pImg2  = pImg0 - srcStride;
      pImg3  = pImg1 + srcStride;
      pImg4  = pImg2 - srcStride;
      pImg5  = pImg3 + srcStride;
      pImg6  = pImg4 - srcStride;
      pImg7  = pImg5 + srcStride;
      pImg8  = pImg6 - srcStride;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
      pImg9  = pImg7 + srcStride;
      pImg10 = pImg8 - srcStride;
      pImg11 = pImg9 + srcStride;
      pImg12 = pImg10 - srcStride;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int        filterSetIdx = 2 + fixedFilterSetIdx;
#else
      int        filterSetIdx = 0 + fixedFilterSetIdx;
#endif
      const Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;
#if JVET_AD0222_ALF_LONG_FIXFILTER
      const Pel *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
#endif
      if (isFixedFilterPaddedPerCtu)
      {
        pImg0FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 0] + j + padSize;
        pImg1FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 1] + j + padSize;
        pImg2FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 1] + j + padSize;
        pImg3FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 2] + j + padSize;
        pImg4FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 2] + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 3] + j + padSize;
        pImg6FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 3] + j + padSize;
        pImg7FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 4] + j + padSize;
        pImg8FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 4] + j + padSize;
        pImg9FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 5] + j + padSize;
        pImg10FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 5] + j + padSize;
        pImg11FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize + 6] + j + padSize;
        pImg12FixedBased = fixedFilterResultsPerCtu[filterSetIdx][i + padSize - 6] + j + padSize;
#endif
      }
      else
      {
        pImg0FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 0] + blkDst.x + j + padSize;
        pImg1FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 1] + blkDst.x + j + padSize;
        pImg2FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 1] + blkDst.x + j + padSize;
        pImg3FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 2] + blkDst.x + j + padSize;
        pImg4FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 2] + blkDst.x + j + padSize;
#if JVET_AD0222_ALF_LONG_FIXFILTER
        pImg5FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 3] + blkDst.x + j + padSize;
        pImg6FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 3] + blkDst.x + j + padSize;
        pImg7FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 4] + blkDst.x + j + padSize;
        pImg8FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 4] + blkDst.x + j + padSize;
        pImg9FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 5] + blkDst.x + j + padSize;
        pImg10FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 5] + blkDst.x + j + padSize;
        pImg11FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize + 6] + blkDst.x + j + padSize;
        pImg12FixedBased = fixedFilterResults[filterSetIdx][blkDst.y + i + padSize - 6] + blkDst.x + j + padSize;
#endif
      }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];

      for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
        }
        else
        {
          pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
        }
      }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      const Pel *pImg0Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
      const Pel *pImg1Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg2Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
      const Pel *pImg3Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg4Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];

      for( int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++ )
      {
        if( isFixedFilterPaddedPerCtu )
        {
          pImg0Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 0] + j + padSizeLaplacian;
          pImg1Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 1] + j + padSizeLaplacian;
          pImg2Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian - 1] + j + padSizeLaplacian;
          pImg3Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian + 2] + j + padSizeLaplacian;
          pImg4Laplacian[laplacianIdx] = laplacianCtu[laplacianIdx][i + padSizeLaplacian - 2] + j + padSizeLaplacian;
        }
        else
        {
          pImg0Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 0] + blkDst.x + j + padSizeLaplacian;
          pImg1Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 1] + blkDst.x + j + padSizeLaplacian;
          pImg2Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian - 1] + blkDst.x + j + padSizeLaplacian;
          pImg3Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian + 2] + blkDst.x + j + padSizeLaplacian;
          pImg4Laplacian[laplacianIdx] = laplacianPic[laplacianIdx][blkDst.y + i + padSizeLaplacian - 2] + blkDst.x + j + padSizeLaplacian;
        }
      }
#endif
      __m128i cur    = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();
#else
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      __m128i zero = _mm_setzero_si128();
#endif

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF 
      auto process2coeffs_laplacian = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), zero);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), zero);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), zero);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), zero);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };
#endif
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
      process2coeffs_laplacian(14, pImg3Laplacian[0] - 0, pImg4Laplacian[0] + 0, pImg1Laplacian[0] - 0, pImg2Laplacian[0] + 0);
      process2coeffs_laplacian(15, pImg0Laplacian[0] - 2, pImg0Laplacian[0] + 2, pImg0Laplacian[0] - 1, pImg0Laplacian[0] + 1);
#else
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
      process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
      process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(5, pImg12FixedBased - 0, pImg11FixedBased + 0, pImg10FixedBased - 0, pImg9FixedBased + 0);
      process2coeffs(6, pImg8FixedBased - 0, pImg7FixedBased + 0, pImg6FixedBased - 0, pImg5FixedBased + 0);
      process2coeffs(7, pImg4FixedBased - 1, pImg3FixedBased + 1, pImg4FixedBased - 0, pImg3FixedBased + 0);
      process2coeffs(8, pImg4FixedBased + 1, pImg3FixedBased - 1, pImg2FixedBased - 2, pImg1FixedBased + 2);
      process2coeffs(9, pImg2FixedBased - 1, pImg1FixedBased + 1, pImg2FixedBased - 0, pImg1FixedBased + 0);
      process2coeffs(10, pImg2FixedBased + 1, pImg1FixedBased - 1, pImg2FixedBased + 2, pImg1FixedBased - 2);
      process2coeffs(11, pImg0FixedBased - 6, pImg0FixedBased + 6, pImg0FixedBased - 5, pImg0FixedBased + 5);
      process2coeffs(12, pImg0FixedBased - 4, pImg0FixedBased + 4, pImg0FixedBased - 3, pImg0FixedBased + 3);
      process2coeffs(13, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#else
      process2coeffs(0, pImg11 + 0, pImg12 - 0, pImg9 + 0, pImg10 - 0);
      process2coeffs(1, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
      process2coeffs(2, pImg3 + 2, pImg4 - 2, pImg3 + 1, pImg4 - 1);
      process2coeffs(3, pImg3 + 0, pImg4 - 0, pImg3 - 1, pImg4 + 1);
      process2coeffs(4, pImg3 - 2, pImg4 + 2, pImg1 + 2, pImg2 - 2);
      process2coeffs(5, pImg1 + 1, pImg2 - 1, pImg1 + 0, pImg2 - 0);
      process2coeffs(6, pImg1 - 1, pImg2 + 1, pImg1 - 2, pImg2 + 2);
      process2coeffs(7, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
      process2coeffs(8, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
      process2coeffs(9, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      process2coeffs(10, pImg4FixedBased - 0, pImg3FixedBased + 0, pImg2FixedBased - 1, pImg1FixedBased + 1);
      process2coeffs(11, pImg2FixedBased - 0, pImg1FixedBased + 0, pImg2FixedBased + 1, pImg1FixedBased - 1);
      process2coeffs(12, pImg0FixedBased - 2, pImg0FixedBased + 2, pImg0FixedBased - 1, pImg0FixedBased + 1);
#endif
#endif
      pImg0  = srcBeforeDb + j;
      pImg1  = pImg0 + srcBeforeDbStride;
      pImg2  = pImg0 - srcBeforeDbStride;
      pImgP0 = srcResi + j;

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      process2coeffs(16, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#else
      process2coeffs(14, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
      process2coeffs(13, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif
#else
      process2coeffs(10, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);
#endif

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
      __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + padSize] + blkDst.x + j + padSize)), cur);
#else
      __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImg0), cur);
      __m128i val10 = _mm_sub_epi16(
        _mm_loadu_si128((const __m128i *) (fixedFilterResults[fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
#endif
      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      __m128i limit01A = params[0][1][17];
      __m128i limit01B = params[1][1][17];
#else
      __m128i limit01A = params[0][1][15];
      __m128i limit01B = params[1][1][15];
#endif
#else
      __m128i limit01A = params[0][1][14];
      __m128i limit01B = params[1][1][14];
#endif
#else
      __m128i limit01A = params[0][1][11];
      __m128i limit01B = params[1][1][11];
#endif
      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      __m128i coeff01A = params[0][0][17];
      __m128i coeff01B = params[1][0][17];
#else
      __m128i coeff01A = params[0][0][15];
      __m128i coeff01B = params[1][0][15];
#endif
#else
      __m128i coeff01A = params[0][0][14];
      __m128i coeff01B = params[1][0][14];
#endif
#else
      __m128i coeff01A = params[0][0][11];
      __m128i coeff01B = params[1][0][11];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      // start residual fixed filter
#if !JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      __m128i zero = _mm_setzero_si128();
#endif
      val00 = _mm_sub_epi16(
        _mm_loadu_si128((const __m128i *) (fixedFilterResiResults[1 - fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)),
        zero);

      val10  = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      limit01A = params[0][1][18];
      limit01B = params[1][1][18];
#else
      limit01A = params[0][1][16];
      limit01B = params[1][1][16];
#endif
#else
      limit01A = params[0][1][15];
      limit01B = params[1][1][15];
#endif

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      coeff01A = params[0][0][18];
      coeff01B = params[1][0][18];
#else
      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];
#endif
#else
      coeff01A = params[0][0][15];
      coeff01B = params[1][0][15];
#endif

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      // end residual fixed filter

#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImgP0), zero);
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
#else
      val10 = _mm_sub_epi16(cur, cur);
#endif

      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);
#else
      __m128i val = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (fixedFilterResults[EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i] + blkDst.x + j)), cur);
      val01A = _mm_shuffle_epi8(val, _mm_setr_epi8(0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7));
      val01B = _mm_shuffle_epi8(val, _mm_setr_epi8(8, 9, 8, 9, 10, 11, 10, 11, 12, 13, 12, 13, 14, 15, 14, 15));
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      limit01A = params[0][1][19];
      limit01B = params[1][1][19];
#else
      limit01A = params[0][1][17];
      limit01B = params[1][1][17];
#endif
#else
      limit01A = params[0][1][16];
      limit01B = params[1][1][16];
#endif
#else
      limit01A = params[0][1][12];
      limit01B = params[1][1][12];
#endif
      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER 
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      coeff01A = params[0][0][19];
      coeff01B = params[1][0][19];
#else
      coeff01A = params[0][0][17];
      coeff01B = params[1][0][17];
#endif
#else
      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];
#endif
#else
      coeff01A = params[0][0][12];
      coeff01B = params[1][0][12];
#endif
      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      val00    = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Laplacian[0])), zero);
      val10    = _mm_setzero_si128();
      val01A   = _mm_unpacklo_epi16(val00, val10);
      val01B   = _mm_unpackhi_epi16(val00, val10);

      limit01A = params[0][1][20];
      limit01B = params[1][1][20];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      coeff01A = params[0][0][20];
      coeff01B = params[1][0][20];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);
#endif
      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
      if ( bScalingCorr )
      {
        accumA = _mm_add_epi16(accumA, curBase);
      }
      else
#endif
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *) (dst + j), accumA);
    }   // for j
    src += srcStride * stepY;
    dst += dstStride * stepY;
    srcBeforeDb += srcBeforeDbStride * stepY;
    srcResi += srcResiStride * stepY;
  }   // for i
#if USE_AVX2 && (JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF)
  }//Use 256 Bit Simd
 #endif
}
#if FIXFILTER_CFG
template<X86_VEXT vext>
static void simdFilter13x13BlkDbResiDirect(
  AlfClassifier * *classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi,
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  ,const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  ,Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
  const CPelBuf srcBuffer         = recSrc.get(compId);
  PelBuf        dstBuffer         = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);
  const CPelBuf scrBufferResi     = resi.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
  const size_t srcResiStride     = scrBufferResi.stride;
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int adjustShift = coeffBits - 1;
  const bool  bScalingCorr = isLuma(compId) && fixedFilterSetIdx < 0;
  if ( bScalingCorr )
  {
    fixedFilterSetIdx = -fixedFilterSetIdx - 1;
    adjustShift -= shiftPrecis; // add more precision
  }
  int shift = adjustShift;
#if JVET_AJ0237_INTERNAL_12BIT
  const Pel currBase = 1 << (clpRng.bd - 1);
#else
  const Pel currBase = 512;
#endif

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  __m128i curBase = _mm_set_epi16( currBase, currBase, currBase, currBase, currBase, currBase, currBase, currBase );
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t           stepY = 1;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#endif

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = scrBufferResi.buf + blk.y * srcResiStride + blk.x;

#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m256i curBase  = _mm256_set1_epi16(currBase);
#endif

    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
        __m256i params[2][2][10];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m256i rawCoef[4][3], rawClip[4][3], s0, s1;
          __m128i rawCoefTmp[2][4][3], rawClipTmp[2][4][3], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];

          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx0 = pClass[j + 4 * k + l + 0] & 0x3;
            const int classIdx0     = pClass[j + 4 * k + l + 0] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoefTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));

            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[0][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx0]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx0][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx0][m][1];

              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s1Tmp[0]));
              __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s3Tmp[0]));
              rawCoefTmp[0][l][op0] = rawTmp0;
              rawCoefTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }

            const int transposeIdx1 = pClass[j + 4 * k + l + 8] & 0x3;
            const int classIdx1     = pClass[j + 4 * k + l + 8] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l + 4] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoefTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoefTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));

            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClipTmp[1][l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx1]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx1][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx1][m][1];

              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s3Tmp[1]));
              rawCoefTmp[1][l][op0] = rawTmp0;
              rawCoefTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoef[l][0] = _mm256_castsi128_si256(rawCoefTmp[0][l][0]);
            rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[1][l][0], 1);
            rawCoef[l][1] = _mm256_castsi128_si256(rawCoefTmp[0][l][1]);
            rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[1][l][1], 1);
            rawCoef[l][2] = _mm256_castsi128_si256(rawCoefTmp[0][l][2]);
            rawCoef[l][2] = _mm256_insertf128_si256(rawCoef[l][2], rawCoefTmp[1][l][2], 1);
           
            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);
            rawClip[l][2] = _mm256_castsi128_si256(rawClipTmp[0][l][2]);
            rawClip[l][2] = _mm256_insertf128_si256(rawClip[l][2], rawClipTmp[1][l][2], 1);
          }   // for l

          for (unsigned char l = 0; l < 3; l++)
          {
            int m = l << 2;

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            if(l < 2)
            {
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
              params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
              params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
              params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
              params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            }
          }   // for l
        }     // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImgP0;

        pImg0 = src + j;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;

        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for (int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
        {
          if (isFixedFilterPaddedPerCtu)
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
          }
        }
        __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m256i accumA = _mm256_setzero_si256();
        __m256i accumB = _mm256_setzero_si256();
#else
        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };

        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
        process2coeffs(6, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);

        pImg0 = srcBeforeDb + j;
        pImg1 = pImg0 + srcBeforeDbStride;
        pImg2 = pImg0 - srcBeforeDbStride;

        pImgP0 = srcResi + j;

        process2coeffs(7, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);

        // start prediction fixed filter
        __m256i zero = _mm256_setzero_si256();
        __m256i val00        = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0)), cur);
        __m256i val10        = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) pImgP0), zero);
        __m256i val01A       = _mm256_unpacklo_epi16(val00, val10);
        __m256i val01B       = _mm256_unpackhi_epi16(val00, val10);
        __m256i limit01A = params[0][1][8];
        __m256i limit01B = params[1][1][8];

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
        __m256i coeff01A = params[0][0][8];
        __m256i coeff01B = params[1][0][8];

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        
        val00  = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0Gauss[0])), cur);
        val10  = _mm256_setzero_si256();
        val01A = _mm256_unpacklo_epi16(val00, val10);
        val01B = _mm256_unpackhi_epi16(val00, val10);
        limit01A = params[0][1][9];
        limit01B = params[1][1][9];

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
        coeff01A = params[0][0][9];
        coeff01B = params[1][0][9];
        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
        accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

        accumA = _mm256_add_epi32(accumA, mmOffset);
        accumB = _mm256_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm256_srai_epi32(accumA, shift);
        accumB = _mm256_srai_epi32(accumB, shift);

        accumA = _mm256_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if( bScalingCorr )
        {
          accumA = _mm256_add_epi16(accumA, curBase);
        }
        else
#endif
          accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        _mm256_storeu_si256((__m256i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
  }
  else
  {
    const __m128i mmOffset = _mm_set1_epi32(round);
    const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m128i curBase  = _mm_set1_epi16(currBase);
#endif
#endif//Use AVX2 SIMD
    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX)
      {
        __m128i params[2][2][10];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[8];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m128i rawCoef[4][3], rawClip[4][3], s0, s1, s2, s3, rawTmp0, rawTmp1;
          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
            const int classIdx     = pClass[j + 4 * k + l] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

            rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawCoef[l][2] = _mm_loadl_epi64((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));

            rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
            rawClip[l][2] = _mm_loadl_epi64((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));

            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx][m][1];

              s0 = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx][m][0]);
              s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
              s2 = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx][m][1]);
              s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
              rawCoef[l][op0] = rawTmp0;
              rawCoef[l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
              rawClip[l][op0] = rawTmp0;
              rawClip[l][op1] = rawTmp1;
            }
          }   // for l

          for (unsigned char l = 0; l < 3; l++)
          {
            int m = l << 2;
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            if (l < 2)
            {
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
              params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
              params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
              params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
              params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            }
          }   // for l
        }     // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImgP0;

        pImg0  = src + j;
        pImg1  = pImg0 + srcStride;
        pImg2  = pImg0 - srcStride;
        pImg3  = pImg1 + srcStride;
        pImg4  = pImg2 - srcStride;
        pImg5  = pImg3 + srcStride;
        pImg6  = pImg4 - srcStride;
        pImg7  = pImg5 + srcStride;
        pImg8  = pImg6 - srcStride;

        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
        const Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
        {
          if( isFixedFilterPaddedPerCtu )
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 1] + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 1] + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 2] + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss - 2] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
            pImg1Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 1] + blkDst.x + j + padSizeGauss;
            pImg2Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 1] + blkDst.x + j + padSizeGauss;
            pImg3Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 2] + blkDst.x + j + padSizeGauss;
            pImg4Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss - 2] + blkDst.x + j + padSizeGauss;
          }
        }

        __m128i cur    = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();
#else
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
          };

        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        process2coeffs(5, pImg3Gauss[0] - 0, pImg4Gauss[0] + 0, pImg1Gauss[0] - 0, pImg2Gauss[0] + 0);
        process2coeffs(6, pImg0Gauss[0] - 2, pImg0Gauss[0] + 2, pImg0Gauss[0] - 1, pImg0Gauss[0] + 1);

        pImg0 = srcBeforeDb + j;
        pImg1 = pImg0 + srcBeforeDbStride;
        pImg2 = pImg0 - srcBeforeDbStride;

        pImgP0 = srcResi + j;

        process2coeffs(7, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);

        __m128i zero = _mm_setzero_si128();
        __m128i val00        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
        __m128i val10        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImgP0), zero);
        __m128i val01A       = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B       = _mm_unpackhi_epi16(val00, val10);
        __m128i limit01A = params[0][1][8];
        __m128i limit01B = params[1][1][8];

        val01A   = _mm_min_epi16(val01A, limit01A);
        val01B   = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A   = _mm_max_epi16(val01A, limit01A);
        val01B   = _mm_max_epi16(val01B, limit01B);
        __m128i coeff01A = params[0][0][8];
        __m128i coeff01B = params[1][0][8];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

        val00    = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
        val10    = _mm_setzero_si128();
        val01A   = _mm_unpacklo_epi16(val00, val10);
        val01B   = _mm_unpackhi_epi16(val00, val10);
        limit01A = params[0][1][9];
        limit01B = params[1][1][9];

        val01A   = _mm_min_epi16(val01A, limit01A);
        val01B   = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A   = _mm_max_epi16(val01A, limit01A);
        val01B   = _mm_max_epi16(val01B, limit01B);
        coeff01A = params[0][0][9];
        coeff01B = params[1][0][9];
        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if ( bScalingCorr )
        {
          accumA = _mm_add_epi16(accumA, curBase);
        }
        else
#endif
          accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        _mm_storeu_si128((__m128i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//Use 256 Bit Simd
#endif
}

template<X86_VEXT vext>
static void simdFilter13x13BlkDbResi(
  AlfClassifier * *classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi,
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet
#else
  const short *fClipSet
#endif
  ,const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  ,Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel ***gaussPic, Pel ***gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  , const char* scaleIdxSet
#endif
)
{
  const CPelBuf srcBuffer         = recSrc.get(compId);
  PelBuf        dstBuffer         = recDst.get(compId);
  const CPelBuf scrBufferBeforeDb = recBeforeDb.get(compId);
  const CPelBuf scrBufferResi     = resi.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = scrBufferBeforeDb.stride;
  const size_t srcResiStride     = scrBufferResi.stride;
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int adjustShift = coeffBits - 1;
  const bool  bScalingCorr = isLuma(compId) && fixedFilterSetIdx < 0;
  if ( bScalingCorr )
  {
    fixedFilterSetIdx = -fixedFilterSetIdx - 1;
    adjustShift -= shiftPrecis; // add more precision
  }
  int shift = adjustShift;
#if JVET_AJ0237_INTERNAL_12BIT
  const Pel currBase = 1 << (clpRng.bd - 1);
#else
  const Pel currBase = 512;
#endif

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  __m128i curBase = _mm_set_epi16( currBase, currBase, currBase, currBase, currBase, currBase, currBase, currBase );
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  int shift = coeffBits - 1;
#else
  int shift = AdaptiveLoopFilter::m_NUM_BITS - 1;
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  shift += AdaptiveLoopFilter::m_SCALE_SHIFT;
#endif
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t stepX = 8;
  size_t           stepY = 1;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#endif

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = scrBufferBeforeDb.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = scrBufferResi.buf + blk.y * srcResiStride + blk.x;
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax    = _mm256_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m256i curBase  = _mm256_set1_epi16(currBase);
#endif
    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX * 2)
      {
        __m256i params[2][2][8];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[16];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m256i rawCoef[4][2], rawClip[4][2], s0, s1;
          __m128i rawCoefTmp[2][4][2], rawClipTmp[2][4][2], s0Tmp[2], s1Tmp[2], s2Tmp[2], s3Tmp[2];
          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx0 = pClass[j + 4 * k + l + 0] & 0x3;
            const int classIdx0     = pClass[j + 4 * k + l + 0] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx0]];
#endif

            rawCoefTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));

            rawClipTmp[0][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[0][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx0 * MAX_NUM_ALF_LUMA_COEFF + 8));

            const int transposeIdx1 = pClass[j + 4 * k + l + 8] & 0x3;
            const int classIdx1     = pClass[j + 4 * k + l + 8] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 8 + l + 4] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx1]];
#endif

            rawCoefTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawCoefTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));

            rawClipTmp[1][l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF));
            rawClipTmp[1][l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx1 * MAX_NUM_ALF_LUMA_COEFF + 8));

            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx0]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx0][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx0][m][1];
              s0Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx0][m][0]);
              s1Tmp[0] = _mm_xor_si128(s0Tmp[0], _mm_set1_epi8((char) 0x80));
              s2Tmp[0] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx0][m][1]);
              s3Tmp[0] = _mm_xor_si128(s2Tmp[0], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s1Tmp[0]));
              __m128i rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawCoefTmp[0][l][op1], s3Tmp[0]));
              rawCoefTmp[0][l][op0] = rawTmp0;
              rawCoefTmp[0][l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s0Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s1Tmp[0]));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[0][l][op0], s2Tmp[0]), _mm_shuffle_epi8(rawClipTmp[0][l][op1], s3Tmp[0]));
              rawClipTmp[0][l][op0] = rawTmp0;
              rawClipTmp[0][l][op1] = rawTmp1;
            }


            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx1]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx1][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx1][m][1];
              s0Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx1][m][0]);
              s1Tmp[1] = _mm_xor_si128(s0Tmp[1], _mm_set1_epi8((char) 0x80));
              s2Tmp[1] = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx1][m][1]);
              s3Tmp[1] = _mm_xor_si128(s2Tmp[1], _mm_set1_epi8((char) 0x80));

              __m128i rawTmp0       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s1Tmp[1]));
              __m128i rawTmp1       = _mm_or_si128(_mm_shuffle_epi8(rawCoefTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawCoefTmp[1][l][op1], s3Tmp[1]));
              rawCoefTmp[1][l][op0] = rawTmp0;
              rawCoefTmp[1][l][op1] = rawTmp1;

              rawTmp0               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s0Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s1Tmp[1]));
              rawTmp1               = _mm_or_si128(_mm_shuffle_epi8(rawClipTmp[1][l][op0], s2Tmp[1]), _mm_shuffle_epi8(rawClipTmp[1][l][op1], s3Tmp[1]));
              rawClipTmp[1][l][op0] = rawTmp0;
              rawClipTmp[1][l][op1] = rawTmp1;
            }

            rawCoef[l][0] = _mm256_castsi128_si256(rawCoefTmp[0][l][0] );
            rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[1][l][0], 1 );
            rawCoef[l][1] = _mm256_castsi128_si256(rawCoefTmp[0][l][1]);
            rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[1][l][1], 1);
            
            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[0][l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[1][l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[0][l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[1][l][1], 1);            
          }   // for l

          for (unsigned char l = 0; l < 2; l++)
          {
            int m = l << 2;

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
          }   // for l
        } // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImgP0;

        pImg0 = src + j;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;

        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for (int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
        {
          if (isFixedFilterPaddedPerCtu)
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          }
        }
        __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m256i accumA = _mm256_setzero_si256();
        __m256i accumB = _mm256_setzero_si256();
#else
        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };

        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        pImg0  = srcBeforeDb + j;
        pImg1  = pImg0 + srcBeforeDbStride;
        pImg2  = pImg0 - srcBeforeDbStride;
        pImgP0 = srcResi + j;

        process2coeffs(5, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);

        __m256i zero = _mm256_setzero_si256();
        __m256i val00 =  _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0)), cur);
        __m256i val10 =  _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) pImgP0), zero);
        __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
        __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
        __m256i limit01A = params[0][1][6];
        __m256i limit01B = params[1][1][6];

        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
        __m256i coeff01A = params[0][0][6];
        __m256i coeff01B = params[1][0][6];

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));

        val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) (pImg0Gauss[0])), cur);
        val10 = _mm256_sub_epi16(cur, cur);

        val01A = _mm256_unpacklo_epi16(val00, val10);
        val01B = _mm256_unpackhi_epi16(val00, val10);

        limit01A = params[0][1][7];
        limit01B = params[1][1][7];
        val01A   = _mm256_min_epi16(val01A, limit01A);
        val01B   = _mm256_min_epi16(val01B, limit01B);
        limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
        limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);
        val01A   = _mm256_max_epi16(val01A, limit01A);
        val01B   = _mm256_max_epi16(val01B, limit01B);
        coeff01A = params[0][0][7];
        coeff01B = params[1][0][7];

        accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
        accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm256_mullo_epi32(accumA, _mm256_loadu_si256((const __m256i*) scaleFactor));
        accumB = _mm256_mullo_epi32(accumB, _mm256_loadu_si256((const __m256i*) (scaleFactor + 8)));

        accumA = _mm256_add_epi32(accumA, mmOffset);
        accumB = _mm256_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm256_srai_epi32(accumA, shift);
        accumB = _mm256_srai_epi32(accumB, shift);

        accumA = _mm256_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if( bScalingCorr )
        {
          accumA = _mm256_add_epi16(accumA, curBase);
        }
        else
#endif
          accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        _mm256_storeu_si256((__m256i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
  }
  else
  {
    const __m128i mmOffset = _mm_set1_epi32(round);
    const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax    = _mm_set1_epi16(clpRng.max);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    const __m128i curBase  = _mm_set1_epi16(currBase);
#endif
#endif //Use AVX2 SIMD
    for (size_t i = 0; i < height; i += stepY)
    {
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
      for (size_t j = 0; j < width; j += stepX)
      {
        __m128i params[2][2][8];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int32_t scaleFactor[8];
#endif
        for (int k = 0; k < 2; k++)
        {
          __m128i rawCoef[4][2], rawClip[4][2], s0, s1, s2, s3, rawTmp0, rawTmp1;
          for (int l = 0; l < 4; l++)
          {
            const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
            const int classIdx     = pClass[j + 4 * k + l] >> 2;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            scaleFactor[k * 4 + l] = AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];
#endif

            rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));            

            rawClip[l][0] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
            rawClip[l][1] = _mm_loadu_si128((const __m128i *) (fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
            for (int m = 0; m < shuffleTime13NoFixed[transposeIdx]; m++)
            {
              int op0 = shuffleOp13NoFixed[transposeIdx][m][0];
              int op1 = shuffleOp13NoFixed[transposeIdx][m][1];
              s0 = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx][m][0]);
              s1 = _mm_xor_si128(s0, _mm_set1_epi8((char) 0x80));
              s2 = _mm_loadu_si128((const __m128i *) shuffleTab13NoFixed[transposeIdx][m][1]);
              s3 = _mm_xor_si128(s2, _mm_set1_epi8((char) 0x80));

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
              rawCoef[l][op0] = rawTmp0;
              rawCoef[l][op1] = rawTmp1;

              rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
              rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
              rawClip[l][op0] = rawTmp0;
              rawClip[l][op1] = rawTmp1;
            }
          }   // for l
          for (unsigned char l = 0; l < 2; l++)
          {
            int m = l << 2;

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }   // for l
        }     // for k

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImgP0;

        pImg0  = src + j;
        pImg1  = pImg0 + srcStride;
        pImg2  = pImg0 - srcStride;
        pImg3  = pImg1 + srcStride;
        pImg4  = pImg2 - srcStride;
        pImg5  = pImg3 + srcStride;
        pImg6  = pImg4 - srcStride;
        pImg7  = pImg5 + srcStride;
        pImg8  = pImg6 - srcStride;

        const Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];

        for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
        {
          if( isFixedFilterPaddedPerCtu )
          {
            pImg0Gauss[gaussIdx] = gaussCtu[gaussIdx][i + padSizeGauss + 0] + j + padSizeGauss;
          }
          else
          {
            pImg0Gauss[gaussIdx] = gaussPic[gaussIdx][blkDst.y + i + padSizeGauss + 0] + blkDst.x + j + padSizeGauss;
          }
        }
        __m128i cur    = _mm_loadu_si128((const __m128i *) pImg0);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();
#else
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;
#endif

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
          };
        process2coeffs(0, pImg7 + 0, pImg8 - 0, pImg5 + 0, pImg6 - 0);
        process2coeffs(1, pImg3 + 0, pImg4 - 0, pImg1 + 1, pImg2 - 1);
        process2coeffs(2, pImg1 + 0, pImg2 - 0, pImg1 - 1, pImg2 + 1);
        process2coeffs(3, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(4, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);
        
        pImg0  = srcBeforeDb + j;
        pImg1  = pImg0 + srcBeforeDbStride;
        pImg2  = pImg0 - srcBeforeDbStride;
        pImgP0 = srcResi + j;

        process2coeffs(5, pImg1 + 0, pImg2 + 0, pImg0 + 1, pImg0 - 1);

        __m128i zero = _mm_setzero_si128();
        __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0)), cur);
        __m128i val10  = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) pImgP0), zero);
        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i limit01A = params[0][1][6];
        __m128i limit01B = params[1][1][6];

        val01A   = _mm_min_epi16(val01A, limit01A);
        val01B   = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A   = _mm_max_epi16(val01A, limit01A);
        val01B   = _mm_max_epi16(val01B, limit01B);
        __m128i coeff01A = params[0][0][6];
        __m128i coeff01B = params[1][0][6];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

        val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) (pImg0Gauss[0])), cur);
        val10 = _mm_sub_epi16(cur, cur);
        val01A = _mm_unpacklo_epi16(val00, val10);
        val01B = _mm_unpackhi_epi16(val00, val10);
        limit01A = params[0][1][7];
        limit01B = params[1][1][7];

        val01A   = _mm_min_epi16(val01A, limit01A);
        val01B   = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A   = _mm_max_epi16(val01A, limit01A);
        val01B   = _mm_max_epi16(val01B, limit01B);
        coeff01A = params[0][0][7];
        coeff01B = params[1][0][7];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i*) scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i*) (scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);
#endif

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_packs_epi32(accumA, accumB);
#if JVET_AI0084_ALF_RESIDUALS_SCALING
        if ( bScalingCorr )
        {
          accumA = _mm_add_epi16(accumA, curBase);
        }
        else
#endif
          accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        _mm_storeu_si128((__m128i *) (dst + j), accumA);
      }   // for j
      src += srcStride * stepY;
      dst += dstStride * stepY;
      srcBeforeDb += srcBeforeDbStride * stepY;
      srcResi += srcResiStride * stepY;
    }   // for i
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//Use 256 Bit Simd
#endif
}
#endif
#endif

#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
//Gauss Filter
template<X86_VEXT vext>
static void simdGaussFiltering(CodingStructure &cs, Pel ***gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  , bool applyCodingInfo, AlfClassifier** classifierCodingInfo
#endif
  )
{
  int16_t gaussCoefTable[NUM_GAUSS_FILTERED_SOURCE][25] =
  {
    {
      8, 22, 30, 22, 22, 60, 85, 60, 22, 8, 30, 85, 119, 85, 30, 8, 22, 60, 85, 60, 22, 22, 30, 22, 8,
    },
  };

  int16_t gaussClipIdxTable[NUM_GAUSS_FILTERED_SOURCE][25] =
  {
    {
        3, 2, 1, 2, 2, 1, 0, 1, 2, 3, 1, 0, 0, 0, 1, 3, 2, 1, 0, 1, 2, 2, 1, 2, 3,
    },
  };

  int16_t numCoeff = 12;
  int16_t gaussClipTable[25] = {0};
  for(int i = 0; i < numCoeff; i++)
  {
    int clipIdx = gaussClipIdxTable[filterSetIdx][i];
    gaussClipTable[i] = clippingValues[clipIdx];
  }
#if JVET_AJ0237_INTERNAL_12BIT
  int16_t diffTH = 32 << std::max(0, cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 10);
#else
  int16_t diffTH = 32;
#endif

#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool isIntraSlice = cs.slice->isIntra();
  const bool isSpsAdjust  = cs.sps->getAlfLumaFixedFilterAdjust();

  const bool useBounCondition = applyCodingInfo && !(!isSpsAdjust && isIntraSlice);
  const bool useResiCondition = applyCodingInfo && !isIntraSlice && false;
  const int offsetClipValue = 1 << ( clpRng.bd - 1 );
#endif
#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i offsetMax = _mm_set1_epi16(diffTH);
  const __m128i offsetMin = _mm_sub_epi16(_mm_setzero_si128(), offsetMax);
#endif

  const CPelBuf srcBuffer = srcLuma;
  const int srcStride = srcBuffer.stride;

  constexpr int shift = 10;
  constexpr int round = 1 << (shift - 1);

  const int width = blk.width;
  const int height = blk.height;

  constexpr int stepX = 8;
  int stepY = 1;

#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif

  static_assert(sizeof(*gaussCoefTable[0]) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*gaussClipTable   ) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  const int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd )
  {
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m256i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m256i mmOriOffset;
    __m256i mmSignOffsetP, mmSignOffsetN;
    __m256i mmAbsOffset;
    __m256i mmAdjOffset;
    __m256i mmZeroVector = _mm256_set1_epi16( 0 );
    __m256i mm01Vector   = _mm256_set1_epi16( 1 );
    __m256i mm08Vector   = _mm256_set1_epi16( 8 );
    __m256i mm16Vector   = _mm256_set1_epi16( 16 );
    __m256i mmPOffsetClipVector = _mm256_set1_epi16( +offsetClipValue );
    __m256i mmNOffsetClipVector = _mm256_set1_epi16( -offsetClipValue );
    // Set Factor
    __m256i mmBsFactor = isIntraSlice ? _mm256_set1_epi16( 4 + 2 ) : _mm256_set1_epi16( 3 + 2 );
    __m256i mmResiFactor = isIntraSlice ? _mm256_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0)) : _mm256_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0));
#endif
    const __m256i offsetMax = _mm256_set1_epi16(diffTH);
    const __m256i offsetMin = _mm256_sub_epi16(_mm256_set1_epi16( 0 ), offsetMax);
    const __m256i mmOffset  = _mm256_set1_epi32(round);
    const __m256i mmMin     = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax     = _mm256_set1_epi16(clpRng.max);

    for (int i = 0; i < height; i += stepY)
    {
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      AlfClassifier *pClassCodingInfo = nullptr;
      if( useBounCondition || useResiCondition )
      {
        pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
      }
#endif
      for (int j = 0; j < width; j += stepX * 2)
      {
        __m256i params[2][2][6];

        for (int k = 0; k < 2; k++)
        {
          __m256i rawCoef[4][2], rawClip[4][2], s0, s1;
          __m128i rawCoefTmp[4][2], rawClipTmp[4][2];

          for (int l = 0; l < 4; l++)
          {
            rawCoefTmp[l][0] = _mm_loadu_si128((const __m128i *) (gaussCoefTable[filterSetIdx] + 0));
            rawCoefTmp[l][1] = _mm_loadu_si128((const __m128i *) (gaussCoefTable[filterSetIdx] + 8));

            rawClipTmp[l][0] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 0));
            rawClipTmp[l][1] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 8));

            rawCoef[l][0] = _mm256_castsi128_si256( rawCoefTmp[l][0]);
            rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[l][0], 1);
            rawCoef[l][1] = _mm256_castsi128_si256( rawCoefTmp[l][1]);
            rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[l][1], 1);

            rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[l][0]);
            rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[l][0], 1);
            rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[l][1]);
            rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[l][1], 1);
          }   // for l

          for (unsigned char l = 0; l < 2; l++)
          {
            int m = l << 2;

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

            if (l < 1)
            {
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
              params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
              params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
              params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
              params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
            }
          }   // for l
        }     // for k
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        mmClassIdxBsP = _mm256_set1_epi16( 0 );
        mmClassIdxBsN = _mm256_set1_epi16( 0 );
        if( useBounCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256((const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16(mmClassIdxTmp, 1);
          mmClassIdxBsN   = _mm256_sub_epi16( mm01Vector, mmClassIdxBsP);
        }
        mmClassIdxResiP = _mm256_set1_epi16( 0 );
        mmClassIdxResiN = _mm256_set1_epi16( 0 );
        if( useResiCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256((const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16(mmClassIdxTmp, 1);
          mmClassIdxResiP = _mm256_sub_epi16(mmClassIdxTmp, _mm256_add_epi16(mmClassIdxBsP, mmClassIdxBsP));
          mmClassIdxResiN = _mm256_sub_epi16( mm01Vector, mmClassIdxResiP);
        }
#endif

        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
        pImg0 = src + j;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;

        __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
        __m256i accumA = mmOffset;
        __m256i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
          const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
          const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
          const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

          __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
          __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
          __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
          __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

          __m256i limit01A = params[0][1][i];
          __m256i limit01B = params[1][1][i];

          val01A = _mm256_min_epi16(val01A, limit01A);
          val01B = _mm256_min_epi16(val01B, limit01B);
          val01C = _mm256_min_epi16(val01C, limit01A);
          val01D = _mm256_min_epi16(val01D, limit01B);

          limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
          limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

          val01A = _mm256_max_epi16(val01A, limit01A);
          val01B = _mm256_max_epi16(val01B, limit01B);
          val01C = _mm256_max_epi16(val01C, limit01A);
          val01D = _mm256_max_epi16(val01D, limit01B);

          val01A = _mm256_add_epi16(val01A, val01C);
          val01B = _mm256_add_epi16(val01B, val01D);

          const __m256i coeff01A = params[0][0][i];
          const __m256i coeff01B = params[1][0][i];

          accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
          accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
        };

        process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
        process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
        process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
        process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
        process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
        process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

        accumA = _mm256_srai_epi32(accumA, shift);
        accumB = _mm256_srai_epi32(accumB, shift);

        accumA = _mm256_packs_epi32(accumA, accumB);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        if ( useBounCondition )
        {
          accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA);
          accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA);
          // accumA is Ori Offset
          mmOriOffset = accumA;
          // Calc Sign
          // P = 1, N = 0
          mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16(mmOriOffset, mmZeroVector) );
          // P = 0, N = 1
          mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector, mmSignOffsetP));
          // Calc Abs Offset
          mmAbsOffset = _mm256_abs_epi16( mmOriOffset );
          // BS based Adjustment
          mmAdjOffset = _mm256_mullo_epi16(mmAbsOffset, _mm256_add_epi16( mm16Vector, mmBsFactor));
          mmAdjOffset = _mm256_add_epi16(mmAdjOffset, mm08Vector);
          mmAdjOffset = _mm256_srai_epi16(mmAdjOffset, 4);

          __m256i mmTmpAdj = _mm256_mullo_epi16(mmClassIdxBsP, mmAdjOffset);
          __m256i mmTmpOrg = _mm256_mullo_epi16(mmClassIdxBsN, mmAbsOffset);

          __m256i mmTmpFin = _mm256_add_epi16(mmTmpAdj, mmTmpOrg);

          __m256i mmTmpSignP = _mm256_mullo_epi16(mmSignOffsetP, mmTmpFin);
          __m256i mmTmpSignN = _mm256_sub_epi16( mmZeroVector, _mm256_mullo_epi16(mmSignOffsetN, mmTmpFin) );

          accumA = _mm256_add_epi16(mmTmpSignP, mmTmpSignN);
        }

        if ( useResiCondition )
        {
          accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA);
          accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA);
          // accumA is Ori Offset
          mmOriOffset = accumA;
          // Calc Sign
          // P = 1, N = 0
          mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16(mmOriOffset, mmZeroVector));
          // P = 0, N = 1
          mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector, mmSignOffsetP));
          // Calc Abs Offset
          mmAbsOffset = _mm256_abs_epi16(mmOriOffset);
          // Resi based Adjustment
          mmAdjOffset = _mm256_mullo_epi16(mmAbsOffset, _mm256_add_epi16( mm16Vector, mmResiFactor));
          mmAdjOffset = _mm256_add_epi16(mmAdjOffset, mm08Vector);
          mmAdjOffset = _mm256_srai_epi16(mmAdjOffset, 4);

          __m256i mmTmpAdj = _mm256_mullo_epi16(mmClassIdxResiP, mmAdjOffset);
          __m256i mmTmpOrg = _mm256_mullo_epi16(mmClassIdxResiN, mmAbsOffset);

          __m256i mmTmpFin = _mm256_add_epi16(mmTmpAdj, mmTmpOrg);

          __m256i mmTmpSignP = _mm256_mullo_epi16(mmSignOffsetP, mmTmpFin);
          __m256i mmTmpSignN = _mm256_sub_epi16(mmZeroVector, _mm256_mullo_epi16(mmSignOffsetN, mmTmpFin));

          accumA = _mm256_add_epi16(mmTmpSignP, mmTmpSignN);
        }
#endif
        // Clip Offset
        accumA = _mm256_min_epi16(accumA, offsetMax);
        accumA = _mm256_max_epi16(accumA, offsetMin);

        accumA = _mm256_add_epi16(accumA, cur);
        accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

        int curY = blkDst.y + i + padSizeGauss;
        int curX = blkDst.x + j + padSizeGauss;

        _mm256_storeu_si256((__m256i *) (gaussPic[storeIdx][curY] + curX), accumA);
      }   // for j
      src += srcStride * stepY;
    }   // for i
  }
  else //use256BitSimd
  {

  const __m128i offsetMax = _mm_set1_epi16(diffTH);
  const __m128i offsetMin = _mm_sub_epi16(_mm_setzero_si128(), offsetMax);
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
#endif //Use AVX2 SIMD
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  __m128i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
  __m128i mmOriOffset;
  __m128i mmSignOffsetP, mmSignOffsetN;
  __m128i mmAbsOffset;
  __m128i mmAdjOffset;
  __m128i mmZeroVector = _mm_set1_epi16( 0 );
  __m128i mm01Vector   = _mm_set1_epi16( 1 );
  __m128i mm08Vector   = _mm_set1_epi16( 8 );
  __m128i mm16Vector   = _mm_set1_epi16( 16 );
  __m128i mmPOffsetClipVector = _mm_set1_epi16( +offsetClipValue );
  __m128i mmNOffsetClipVector = _mm_set1_epi16( -offsetClipValue );
  //Set Factor
  __m128i mmBsFactor = isIntraSlice ? _mm_set1_epi16( 4 + 2 ) : _mm_set1_epi16( 3 + 2 );
  __m128i mmResiFactor = isIntraSlice ? _mm_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif
  for (int i = 0; i < height; i += stepY)
  {
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    AlfClassifier *pClassCodingInfo = nullptr;
    if( useBounCondition || useResiCondition )
    {
      pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
    }
#endif
    for (int j = 0; j < width; j += stepX)
    {
      __m128i params[2][2][6];

      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoef[4][2], rawClip[4][2], s0, s1;

        for (int l = 0; l < 4; l++)
        {
          rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (gaussCoefTable[filterSetIdx] + 0));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (gaussCoefTable[filterSetIdx] + 8));

          rawClip[l][0] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 0));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 8));
        }//for l

        for (unsigned char l = 0; l < 2; l++)
        {
          int m = l << 2;

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
          params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
          params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

          if (l < 1)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
        }//for l
      }//for k
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      mmClassIdxBsP = _mm_set1_epi16( 0 );
      mmClassIdxBsN = _mm_set1_epi16( 0 );
      if( useBounCondition )
      {
        mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
        mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxBsN   = _mm_sub_epi16( mm01Vector, mmClassIdxBsP );
      }
      mmClassIdxResiP = _mm_set1_epi16( 0 );
      mmClassIdxResiN = _mm_set1_epi16( 0 );
      if( useResiCondition )
      {
        mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
        mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxResiP = _mm_sub_epi16(  mmClassIdxTmp, _mm_add_epi16( mmClassIdxBsP, mmClassIdxBsP) );
        mmClassIdxResiN = _mm_sub_epi16( mm01Vector, mmClassIdxResiP );
      }
#endif

      const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
      pImg0 = src + j;
      pImg1 = pImg0 + srcStride;
      pImg2 = pImg0 - srcStride;
      pImg3 = pImg1 + srcStride;
      pImg4 = pImg2 - srcStride;
      pImg5 = pImg3 + srcStride;
      pImg6 = pImg4 - srcStride;

      __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
      process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
      process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
      process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
      process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
      process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      if( useBounCondition )
      {
        accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
        accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
        //accumA is Ori Offset
        mmOriOffset = accumA;
        //Calc Sign
        //P = 1, N = 0
        mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
        //P = 0, N = 1
        mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
        //Calc Abs Offset
        mmAbsOffset = _mm_abs_epi16( mmOriOffset );
        //BS based Adjustment
        mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmBsFactor ) );
        mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
        mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

        __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
        __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

        __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

        __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
        __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

        accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
      }

      if( useResiCondition )
      {
        accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
        accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
        //accumA is Ori Offset
        mmOriOffset = accumA;
        //Calc Sign
        //P = 1, N = 0
        mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
        //P = 0, N = 1
        mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP));
        //Calc Abs Offset
        mmAbsOffset = _mm_abs_epi16( mmOriOffset );
        //Resi based Adjustment
        mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmResiFactor ) );
        mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
        mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

        __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
        __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

        __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

        __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
        __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

        accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
      }
#endif

      //Clip Offset
      accumA = _mm_min_epi16(accumA, offsetMax);
      accumA = _mm_max_epi16(accumA, offsetMin);

      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      int curY = blkDst.y + i + padSizeGauss;
      int curX = blkDst.x + j + padSizeGauss;
      _mm_storeu_si128((__m128i *) (gaussPic[storeIdx][curY] + curX), accumA);

    }//for j
    src += srcStride * stepY;
  }//for i
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//use256BitSimd
#endif
}
#endif

#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
//Laplacian Filter
template<X86_VEXT vext>
static void simdLaplacianFiltering(CodingStructure &cs, Pel ***laplacianPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx, const CPelBuf &srcCodingInfo)
{
  const int strideSrc = srcLuma.stride;
  const bool isHighRes = cs.pcv->lumaWidth > 1280 && cs.pcv->lumaHeight > 720 ? true : false;

  int16_t laplacianCoefTable[NUM_LAPLACIAN_FILTERED_SOURCE][8] =
  {
    {
      1, 1, -4, 1, 1, 0, 0, 0,
    },
  };
  int16_t laplacianClipIdxTable[NUM_LAPLACIAN_FILTERED_SOURCE][8] =
  {
    {
        0, 0, 0, 0, 0, 0, 0, 0,
    },
  };
  int16_t numCoeff = 2;
  int16_t laplacianClipTable[8] = {0};
  for(int i = 0; i < numCoeff; i++)
  {
    int clipIdx = laplacianClipIdxTable[filterSetIdx][i];
    laplacianClipTable[i] = clippingValues[clipIdx];
  }

#if JVET_AJ0237_INTERNAL_12BIT
  int16_t diffTH = (isHighRes ? 128 : 96) << std::max(0, cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 10);
#else
  int16_t diffTH = (isHighRes ? 128 : 96);
#endif

  int16_t gaussCoefTable[25] =
  {
    8, 22, 30, 22, 22, 60, 85, 60, 22, 8, 30, 85, 119, 85, 30, 8, 22, 60, 85, 60, 22, 22, 30, 22, 8,
  };
  int16_t gaussClipIdxTable[25] =
  {
    3, 2, 1, 2, 2, 1, 0, 1, 2, 3, 1, 0, 0, 0, 1, 3, 2, 1, 0, 1, 2, 2, 1, 2, 3,
  };
  int16_t gaussNumCoeff = 12;
  int16_t gaussClipTable[25] = {0};
  for(int i = 0; i < gaussNumCoeff; i++)
  {
    int clipIdx = gaussClipIdxTable[i];
    gaussClipTable[i] = clippingValues[clipIdx];
  }

#if JVET_AJ0237_INTERNAL_12BIT
  int16_t gaussDiffTH = (isHighRes ? 1024 : 96) << std::max(0, cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 10);
#else
  int16_t gaussDiffTH = (isHighRes ? 1024 : 96);
#endif
 
  constexpr int gaussShift = 10;
  constexpr int gaussRound = 1 << (gaussShift - 1);
  static_assert(sizeof(*gaussCoefTable   ) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*gaussClipTable   ) == 2, "ALF clip values must be 16-bit wide");

#if !( USE_AVX2 && JVET_AK0091_LAPLACIAN_INFO_IN_ALF )
  const __m128i gaussOffsetMax = _mm_set1_epi16(gaussDiffTH);
  const __m128i gaussOffsetMin = _mm_sub_epi16(_mm_setzero_si128(), gaussOffsetMax);

  const __m128i offsetMax = _mm_set1_epi16(diffTH);
  const __m128i offsetMin = _mm_sub_epi16(_mm_setzero_si128(), offsetMax);
#endif

  const CPelBuf srcBuffer = srcLuma;
  const int srcStride = srcBuffer.stride;
  const CPelBuf srcCodingBuffer = srcCodingInfo;
  const int srcCodingStride = srcCodingBuffer.stride;
  const int width = blk.width;
  const int height = blk.height;

  constexpr int stepX = 8;
  int stepY = 1;

#if !( USE_AVX2 && JVET_AK0091_LAPLACIAN_INFO_IN_ALF )
  const __m128i mmGaussOffset = _mm_set1_epi32(gaussRound);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
  int16_t bndNumArr[stepX] = {0};
#endif

  static_assert(sizeof(*laplacianCoefTable[0]) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*laplacianClipTable   ) == 2, "ALF clip values must be 16-bit wide");
  const Pel* src = srcBuffer.buf + blk.y * srcStride + blk.x;
  const Pel* ciPtr = srcCodingBuffer.buf + blk.y * srcCodingStride + blk.x;
  const int padSizeLaplacian = ALF_PADDING_SIZE_LAPLACIAN_RESULTS;

#if USE_AVX2 && JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if(use256BitSimd)
  {
    const __m256i gaussOffsetMax = _mm256_set1_epi16(gaussDiffTH);
    const __m256i gaussOffsetMin = _mm256_sub_epi16(_mm256_set1_epi16( 0 ), gaussOffsetMax);
    const __m256i mmGaussOffset  = _mm256_set1_epi32(gaussRound);

    const __m256i offsetMax = _mm256_set1_epi16(diffTH);
    const __m256i offsetMin = _mm256_sub_epi16(_mm256_set1_epi16( 0 ), offsetMax);
    const __m256i mmMin = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax = _mm256_set1_epi16(clpRng.max);
    int16_t bndNumArr[stepX*2] = {0};

    for (int i = 0; i < height; i += stepY)
    {
      for (int j = 0; j < width; j += stepX * 2)
      {
        __m256i params[2][2][6];

        int16_t bndNum=0;

        for (int dxx = 0; dxx < stepX*2; ++dxx)
        {
          const auto jx = j + dxx;
          const auto ciImg0 = ciPtr + jx;
          
          if(ciImg0[0] == 0)
          {
            bndNumArr[dxx] = 0;
          }
          else
          {
            bndNumArr[dxx] = 1;
            bndNum++;
          }
        }

        if(bndNum != 0)
        {
          for (int k = 0; k < 2; k++)
          {
            __m256i rawCoef[4][2], rawClip[4][2], s0, s1;
            __m128i rawCoefTmp[4][2], rawClipTmp[4][2];

            for (int l = 0; l < 4; l++)
            {
              rawCoefTmp[l][0] = _mm_loadu_si128((const __m128i *) (gaussCoefTable + 0));
              rawCoefTmp[l][1] = _mm_loadu_si128((const __m128i *) (gaussCoefTable + 8));

              rawClipTmp[l][0] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 0));
              rawClipTmp[l][1] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 8));

              rawCoef[l][0] = _mm256_castsi128_si256( rawCoefTmp[l][0]);
              rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[l][0], 1);
              rawCoef[l][1] = _mm256_castsi128_si256( rawCoefTmp[l][1]);
              rawCoef[l][1] = _mm256_insertf128_si256(rawCoef[l][1], rawCoefTmp[l][1], 1);

              rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[l][0]);
              rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[l][0], 1);
              rawClip[l][1] = _mm256_castsi128_si256(rawClipTmp[l][1]);
              rawClip[l][1] = _mm256_insertf128_si256(rawClip[l][1], rawClipTmp[l][1], 1);
            }   // for l

            for (unsigned char l = 0; l < 2; l++)
            {
              int m = l << 2;

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x00), _mm256_shuffle_epi32(rawCoef[3][l], 0x00));
              params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x00), _mm256_shuffle_epi32(rawClip[3][l], 0x00));
              params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x55), _mm256_shuffle_epi32(rawCoef[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0x55), _mm256_shuffle_epi32(rawCoef[3][l], 0x55));
              params[k][0][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x55), _mm256_shuffle_epi32(rawClip[1][l], 0x55));
              s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0x55), _mm256_shuffle_epi32(rawClip[3][l], 0x55));
              params[k][1][1 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

              if (l < 1)
              {
                s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xaa), _mm256_shuffle_epi32(rawCoef[1][l], 0xaa));
                s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xaa), _mm256_shuffle_epi32(rawCoef[3][l], 0xaa));
                params[k][0][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
                s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xaa), _mm256_shuffle_epi32(rawClip[1][l], 0xaa));
                s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xaa), _mm256_shuffle_epi32(rawClip[3][l], 0xaa));
                params[k][1][2 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);

                s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0xff), _mm256_shuffle_epi32(rawCoef[1][l], 0xff));
                s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[2][l], 0xff), _mm256_shuffle_epi32(rawCoef[3][l], 0xff));
                params[k][0][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
                s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0xff), _mm256_shuffle_epi32(rawClip[1][l], 0xff));
                s1 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[2][l], 0xff), _mm256_shuffle_epi32(rawClip[3][l], 0xff));
                params[k][1][3 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s1, 0x88), 0xf0);
              }
            }   // for l
          }     // for k
          __m256i accumA = mmGaussOffset;
          __m256i accumB = mmGaussOffset;

          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
          Pel gaussArr[5][stepX*2];

          // (i,j)
          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm256_srai_epi32(accumA, gaussShift);
          accumB = _mm256_srai_epi32(accumB, gaussShift);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm256_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (&gaussArr[2][0]), accumA);

          //(i+1,j)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + strideSrc + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm256_loadu_si256((const __m256i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm256_srai_epi32(accumA, gaussShift);
          accumB = _mm256_srai_epi32(accumB, gaussShift);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm256_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (&gaussArr[4][0]), accumA);
        
          //(i-1,j)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src - strideSrc + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm256_loadu_si256((const __m256i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm256_srai_epi32(accumA, gaussShift);
          accumB = _mm256_srai_epi32(accumB, gaussShift);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm256_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (&gaussArr[0][0]), accumA);

          //(i,j-1)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + j-1;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm256_loadu_si256((const __m256i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm256_srai_epi32(accumA, gaussShift);
          accumB = _mm256_srai_epi32(accumB, gaussShift);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm256_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (&gaussArr[1][0]), accumA);

          //(i,j+1)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + j+1;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm256_loadu_si256((const __m256i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm256_srai_epi32(accumA, gaussShift);
          accumB = _mm256_srai_epi32(accumB, gaussShift);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm256_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *) (&gaussArr[3][0]), accumA);

          //Laplacian filtering
          for (int k = 0; k < 2; k++)
          {
            __m256i rawCoef[2][2], rawClip[2][2], s0; //, s1;
            __m128i rawCoefTmp[2][2], rawClipTmp[2][2];

            for (int l = 0; l < 2; l++)
            {
              rawCoefTmp[l][0] = _mm_loadu_si128((const __m128i *) (laplacianCoefTable[filterSetIdx] + 0));

              rawClipTmp[l][0] = _mm_loadu_si128((const __m128i *) (laplacianClipTable + 0));

              rawCoef[l][0] = _mm256_castsi128_si256( rawCoefTmp[l][0]);
              rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[l][0], 1);

              rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[l][0]);
              rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[l][0], 1);
            }   // for l

            for (unsigned char l = 0; l < 1; l++)
            {
              int m = l << 2;

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
              params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s0, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
              params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s0, 0x88), 0xf0);
            }//for l
          }//for k
          
          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          cur = _mm256_loadu_si256((const __m256i *) pImg0);
          
          accumA = _mm256_setzero_si256();
          accumB = _mm256_setzero_si256();

          __m256i accumA_1 = _mm256_setzero_si256();
          __m256i accumB_1 = _mm256_setzero_si256();

          auto process2coeffs_laplacian = [&](const int i, Pel *ptr0, Pel *ptr1, Pel *ptr2, Pel *ptr3) {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), _mm256_loadu_si256((const __m256i *)(&gaussArr[2][0])));
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), _mm256_loadu_si256((const __m256i *)(&gaussArr[2][0])));
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), _mm256_loadu_si256((const __m256i *)(&gaussArr[2][0])));
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), _mm256_loadu_si256((const __m256i *)(&gaussArr[2][0])));

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };
          auto process2coeffs_laplacian_1 = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA_1 = _mm256_add_epi32(accumA_1, _mm256_madd_epi16(val01A, coeff01A));
            accumB_1 = _mm256_add_epi32(accumB_1, _mm256_madd_epi16(val01B, coeff01B));
          };

          process2coeffs_laplacian(0, &gaussArr[4][0], &gaussArr[0][0], &gaussArr[3][0], &gaussArr[1][0]);
          process2coeffs_laplacian_1(0, pImg1 + 0, pImg2 - 0, pImg0 + 1, pImg0 - 1);

          accumA = _mm256_packs_epi32(accumA, accumB);
          accumA_1 = _mm256_packs_epi32(accumA_1, accumB_1);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, offsetMax);
          accumA = _mm256_max_epi16(accumA, offsetMin);

          accumA_1 = _mm256_min_epi16(accumA_1, offsetMax);
          accumA_1 = _mm256_max_epi16(accumA_1, offsetMin);

          //Adjust according boundary info
          short* accumA_1_data = (short*)&accumA_1;
          
          for(int ii=0; ii<stepX*2; ii++)
          {
            if(bndNumArr[ii] == 0)
            {
              if(ii == 0)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 0);
              }
              else if(ii == 1)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 1);
              }
              else if(ii == 2)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 2);
              }
              else if(ii == 3)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 3);
              }
              else if(ii == 4)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 4);
              }
              else if(ii == 5)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 5);
              }
              else if(ii == 6)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 6);
              }
              else if(ii == 7)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 7);
              }
              else if(ii == 8)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 8);
              }
              else if(ii == 9)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 9);
              }
              else if(ii == 10)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 10);
              }
              else if(ii == 11)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 11);
              }
              else if(ii == 12)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 12);
              }
              else if(ii == 13)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 13);
              }
              else if(ii == 14)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 14);
              }
              else if(ii == 15)
              {
                accumA = _mm256_insert_epi16(accumA, accumA_1_data[ii], 15);
              }
            }
          }

          int curY = blkDst.y + i + padSizeLaplacian;
          int curX = blkDst.x + j + padSizeLaplacian;
          _mm256_storeu_si256((__m256i *) (laplacianPic[storeIdx][curY] + curX), accumA);
        }
        else
        {
          //Laplacian filtering
          for (int k = 0; k < 2; k++)
          {
            __m256i rawCoef[2][2], rawClip[2][2], s0; //, s1;
            __m128i rawCoefTmp[2][2], rawClipTmp[2][2];

            for (int l = 0; l < 2; l++)
            {
              rawCoefTmp[l][0] = _mm_loadu_si128((const __m128i *) (laplacianCoefTable[filterSetIdx] + 0));

              rawClipTmp[l][0] = _mm_loadu_si128((const __m128i *) (laplacianClipTable + 0));

              rawCoef[l][0] = _mm256_castsi128_si256( rawCoefTmp[l][0]);
              rawCoef[l][0] = _mm256_insertf128_si256(rawCoef[l][0], rawCoefTmp[l][0], 1);

              rawClip[l][0] = _mm256_castsi128_si256(rawClipTmp[l][0]);
              rawClip[l][0] = _mm256_insertf128_si256(rawClip[l][0], rawClipTmp[l][0], 1);
            }   // for l

            for (unsigned char l = 0; l < 1; l++)
            {
              int m = l << 2;

              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawCoef[0][l], 0x00), _mm256_shuffle_epi32(rawCoef[1][l], 0x00));
              params[k][0][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s0, 0x88), 0xf0);
              s0 = _mm256_unpacklo_epi64(_mm256_shuffle_epi32(rawClip[0][l], 0x00), _mm256_shuffle_epi32(rawClip[1][l], 0x00));
              params[k][1][0 + m] = _mm256_blend_epi16(_mm256_shuffle_epi32(s0, 0x88), _mm256_shuffle_epi32(s0, 0x88), 0xf0);
            }//for l
          }//for k

          const Pel *pImg0, *pImg1, *pImg2;
          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;

          __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0);
          __m256i accumA = _mm256_setzero_si256();
          __m256i accumB = _mm256_setzero_si256();

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_unpacklo_epi16(val00, val10);
            __m256i val01B = _mm256_unpackhi_epi16(val00, val10);
            __m256i val01C = _mm256_unpacklo_epi16(val01, val11);
            __m256i val01D = _mm256_unpackhi_epi16(val01, val11);

            __m256i limit01A = params[0][1][i];
            __m256i limit01B = params[1][1][i];

            val01A = _mm256_min_epi16(val01A, limit01A);
            val01B = _mm256_min_epi16(val01B, limit01B);
            val01C = _mm256_min_epi16(val01C, limit01A);
            val01D = _mm256_min_epi16(val01D, limit01B);

            limit01A = _mm256_sub_epi16(_mm256_setzero_si256(), limit01A);
            limit01B = _mm256_sub_epi16(_mm256_setzero_si256(), limit01B);

            val01A = _mm256_max_epi16(val01A, limit01A);
            val01B = _mm256_max_epi16(val01B, limit01B);
            val01C = _mm256_max_epi16(val01C, limit01A);
            val01D = _mm256_max_epi16(val01D, limit01B);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff01A = params[0][0][i];
            const __m256i coeff01B = params[1][0][i];

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff01A));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff01B));
          };

          process2coeffs(0, pImg1 + 0, pImg2 - 0, pImg0 + 1, pImg0 - 1);

          accumA = _mm256_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm256_min_epi16(accumA, offsetMax);
          accumA = _mm256_max_epi16(accumA, offsetMin);

          int curY = blkDst.y + i + padSizeLaplacian;
          int curX = blkDst.x + j + padSizeLaplacian;
          _mm256_storeu_si256((__m256i *) (laplacianPic[storeIdx][curY] + curX), accumA);
        }
      } // for j
      src += srcStride * stepY;
      ciPtr += srcCodingStride * stepY;
    } // for i
  } 
  else //use256BitSimd
  {
    const __m128i gaussOffsetMax = _mm_set1_epi16(gaussDiffTH);
    const __m128i gaussOffsetMin = _mm_sub_epi16(_mm_setzero_si128(), gaussOffsetMax);
    const __m128i mmGaussOffset = _mm_set1_epi32(gaussRound);

    const __m128i offsetMax = _mm_set1_epi16(diffTH);
    const __m128i offsetMin = _mm_sub_epi16(_mm_setzero_si128(), offsetMax);
    const __m128i mmMin = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax = _mm_set1_epi16(clpRng.max);
    int16_t bndNumArr[stepX] = {0};
#endif //Use AVX2 SIMD

    for (int i = 0; i < height; i += stepY)
    {
      for (int j = 0; j < width; j += stepX)
      {
        __m128i params[2][2][6];

        int16_t bndNum=0;
        for (int dxx = 0; dxx < stepX; ++dxx)
        {
          const auto jx = j + dxx;
          const auto ciImg0 = ciPtr + jx;

          if(ciImg0[0] == 0)
          {
            bndNumArr[dxx] = 0;
          }
          else
          {
            bndNumArr[dxx] = 1;
            bndNum++;
          }
        }

        if(bndNum != 0)
        {
          for (int k = 0; k < 2; k++)
          {
            __m128i rawCoef[4][2], rawClip[4][2], s0, s1;

            for (int l = 0; l < 4; l++)
            {
              rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (gaussCoefTable + 0));
              rawCoef[l][1] = _mm_loadu_si128((const __m128i *) (gaussCoefTable + 8));

              rawClip[l][0] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 0));
              rawClip[l][1] = _mm_loadu_si128((const __m128i *) (gaussClipTable + 8));
            }//for l

            for (unsigned char l = 0; l < 2; l++)
            {
              int m = l << 2;

              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
              params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
              params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
              params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
              s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
              params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

              if (l < 1)
              {
                s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
                s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
                params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
                s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
                s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
                params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

                s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
                s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
                params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
                s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
                s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
                params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
              }
            }//for l
          }//for k

          __m128i accumA = mmGaussOffset;
          __m128i accumB = mmGaussOffset;

          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
          Pel gaussArr[5][stepX];

          // (i,j)
          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
          };

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm_srai_epi32(accumA, gaussShift);
          accumB = _mm_srai_epi32(accumB, gaussShift);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *) (&gaussArr[2][0]), accumA);
 
          //(i+1,j)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + strideSrc + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm_loadu_si128((const __m128i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm_srai_epi32(accumA, gaussShift);
          accumB = _mm_srai_epi32(accumB, gaussShift);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *) (&gaussArr[4][0]), accumA);
        
          //(i-1,j)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src - strideSrc + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm_loadu_si128((const __m128i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm_srai_epi32(accumA, gaussShift);
          accumB = _mm_srai_epi32(accumB, gaussShift);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *) (&gaussArr[0][0]), accumA);

          //(i,j-1)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + j-1;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm_loadu_si128((const __m128i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm_srai_epi32(accumA, gaussShift);
          accumB = _mm_srai_epi32(accumB, gaussShift);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *) (&gaussArr[1][0]), accumA);

          //(i,j+1)
          accumA = mmGaussOffset;
          accumB = mmGaussOffset;
          pImg0 = src + j+1;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;

          cur = _mm_loadu_si128((const __m128i *) pImg0);

          process2coeffs(0, pImg6 - 0, pImg5 + 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(1, pImg4 - 0, pImg3 + 0, pImg4 + 1, pImg3 - 1);
          process2coeffs(2, pImg2 - 2, pImg1 + 2, pImg2 - 1, pImg1 + 1);
          process2coeffs(3, pImg2 - 0, pImg1 + 0, pImg2 + 1, pImg1 - 1);
          process2coeffs(4, pImg2 + 2, pImg1 - 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(5, pImg0 - 2, pImg0 + 2, pImg0 - 1, pImg0 + 1);

          accumA = _mm_srai_epi32(accumA, gaussShift);
          accumB = _mm_srai_epi32(accumB, gaussShift);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, gaussOffsetMax);
          accumA = _mm_max_epi16(accumA, gaussOffsetMin);

          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *) (&gaussArr[3][0]), accumA);

          //Laplacian filtering
          for (int k = 0; k < 2; k++)
          {
            __m128i rawCoef[2][2], rawClip[2][2], s0; //, s1;

            for (int l = 0; l < 2; l++)
            {
              rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (laplacianCoefTable[filterSetIdx] + 0));

              rawClip[l][0] = _mm_loadu_si128((const __m128i *) (laplacianClipTable + 0));
            }//for l

            //for (unsigned char l = 0; l < 2; l++)
            for (unsigned char l = 0; l < 1; l++)
            {
              int m = l << 2;

              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
              params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s0, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
              params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s0, 0x88), 0xf0);
            }//for l
          }//for k

          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          cur = _mm_loadu_si128((const __m128i *) pImg0);
          
          accumA = _mm_setzero_si128();
          accumB = _mm_setzero_si128();

          __m128i accumA_1 = _mm_setzero_si128();
          __m128i accumB_1 = _mm_setzero_si128();

          auto process2coeffs_laplacian = [&](const int i, Pel *ptr0, Pel *ptr1, Pel *ptr2, Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), _mm_loadu_si128((const __m128i *)(&gaussArr[2][0])));
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), _mm_loadu_si128((const __m128i *)(&gaussArr[2][0])));
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), _mm_loadu_si128((const __m128i *)(&gaussArr[2][0])));
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), _mm_loadu_si128((const __m128i *)(&gaussArr[2][0])));

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
          };
          auto process2coeffs_laplacian_1 = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA_1 = _mm_add_epi32(accumA_1, _mm_madd_epi16(val01A, coeff01A));
            accumB_1 = _mm_add_epi32(accumB_1, _mm_madd_epi16(val01B, coeff01B));
          };

          process2coeffs_laplacian(0, &gaussArr[4][0], &gaussArr[0][0], &gaussArr[3][0], &gaussArr[1][0]);
          process2coeffs_laplacian_1(0, pImg1 + 0, pImg2 - 0, pImg0 + 1, pImg0 - 1);

          accumA = _mm_packs_epi32(accumA, accumB);
          accumA_1 = _mm_packs_epi32(accumA_1, accumB_1);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, offsetMax);
          accumA = _mm_max_epi16(accumA, offsetMin);

          accumA_1 = _mm_min_epi16(accumA_1, offsetMax);
          accumA_1 = _mm_max_epi16(accumA_1, offsetMin);

          //Adjust according boundary info
          short* accumA_1_data = (short*)&accumA_1;

          for(int ii=0; ii<stepX; ii++)
          {
            if(bndNumArr[ii] == 0)
            {
              if(ii == 0)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 0);
              }
              else if(ii == 1)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 1);
              }
              else if(ii == 2)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 2);
              }
              else if(ii == 3)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 3);
              }
              else if(ii == 4)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 4);
              }
              else if(ii == 5)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 5);
              }
              else if(ii == 6)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 6);
              }
              else if(ii == 7)
              {
                accumA = _mm_insert_epi16(accumA, accumA_1_data[ii], 7);
              }
            }
          }

          int curY = blkDst.y + i + padSizeLaplacian;
          int curX = blkDst.x + j + padSizeLaplacian;
          _mm_storeu_si128((__m128i *) (laplacianPic[storeIdx][curY] + curX), accumA); 
        }
        else
        {
          //Laplacian filtering
          for (int k = 0; k < 2; k++)
          {
            __m128i rawCoef[2][2], rawClip[2][2], s0; //, s1;

            for (int l = 0; l < 2; l++)
            {
              rawCoef[l][0] = _mm_loadu_si128((const __m128i *) (laplacianCoefTable[filterSetIdx] + 0));

              rawClip[l][0] = _mm_loadu_si128((const __m128i *) (laplacianClipTable + 0));
            }//for l

            for (unsigned char l = 0; l < 1; l++)
            {
              int m = l << 2;

              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
              params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s0, 0x88), 0xf0);
              s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
              params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s0, 0x88), 0xf0);
            }//for l
          }//for k

          const Pel *pImg0, *pImg1, *pImg2;
          pImg0 = src + j;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;

          __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
          __m128i accumA = _mm_setzero_si128();
          __m128i accumB = _mm_setzero_si128();

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_unpacklo_epi16(val00, val10);
            __m128i val01B = _mm_unpackhi_epi16(val00, val10);
            __m128i val01C = _mm_unpacklo_epi16(val01, val11);
            __m128i val01D = _mm_unpackhi_epi16(val01, val11);

            __m128i limit01A = params[0][1][i];
            __m128i limit01B = params[1][1][i];

            val01A = _mm_min_epi16(val01A, limit01A);
            val01B = _mm_min_epi16(val01B, limit01B);
            val01C = _mm_min_epi16(val01C, limit01A);
            val01D = _mm_min_epi16(val01D, limit01B);

            limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
            limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

            val01A = _mm_max_epi16(val01A, limit01A);
            val01B = _mm_max_epi16(val01B, limit01B);
            val01C = _mm_max_epi16(val01C, limit01A);
            val01D = _mm_max_epi16(val01D, limit01B);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff01A = params[0][0][i];
            const __m128i coeff01B = params[1][0][i];

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
          };

          process2coeffs(0, pImg1 + 0, pImg2 - 0, pImg0 + 1, pImg0 - 1);

          accumA = _mm_packs_epi32(accumA, accumB);

          //Clip Offset
          accumA = _mm_min_epi16(accumA, offsetMax);
          accumA = _mm_max_epi16(accumA, offsetMin);

          int curY = blkDst.y + i + padSizeLaplacian;
          int curX = blkDst.x + j + padSizeLaplacian;
          _mm_storeu_si128((__m128i *) (laplacianPic[storeIdx][curY] + curX), accumA);
        }
    }//for j
    src += srcStride * stepY;
    ciPtr += srcCodingStride * stepY;
  }//for i
#if USE_AVX2 && JVET_AK0091_LAPLACIAN_INFO_IN_ALF
}//use256BitSimd
#endif
}
#endif

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
static const int8_t shTab9Db9[4][3][16] = {
  {
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9, 12, 13, 14, 15 },
    { 4,  5,  2,  3,  0,  1,  6,  7, 10, 11,  8,  9, 12, 13, 14, 15 }
  },
  {
    { 0,  1, 10, 11, 12, 13, 14, 15,  8,  9,  2,  3,  4,  5,  6,  7 },
    { 0,  1,  8,  9, 10, 11,  6,  7,  2,  3,  4,  5, 12, 13, 14, 15 },
    { 0,  1,  6,  7,  4,  5,  2,  3,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { 8,  9, 14, 15, 12, 13, 10, 11,  0,  1,  6,  7,  4,  5,  2,  3 },
    { 6,  7, 10, 11,  8,  9,  0,  1,  4,  5,  2,  3, 12, 13, 14, 15 },
    { 4,  5,  6,  7,  0,  1,  2,  3, 10, 11,  8,  9, 12, 13, 14, 15 },
  }
};

static const int8_t shTab[4][9][16] = {
  {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9, 12, 13, 14, 15 },
    { 0,  1,  2,  3, 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13 },
    { 8,  9, 10, 11, 12, 13, 14, 15,  0,  1,  2,  3,  4,  5,  6,  7 },
    { 4,  5,  2,  3,  0,  1, 10, 11,  8,  9,  6,  7, 12, 13, 14, 15 },
    { 2,  3,  0,  1,  4,  5,  8,  9,  6,  7, 10, 11, 12, 13, 14, 15 },
  },
  {
    { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 14, 15, 12, 13,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11 },
    { 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,  2,  3,  0,  1 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 13, 14, 15 },
    { 6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 13, 14, 15 }
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 14, 15, 12, 13,  6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9 },
    { 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13,  2,  3,  0,  1 },
    { 8,  9, 10, 11, 12, 13, 14, 15,  0,  1,  2,  3,  4,  5,  6,  7 },
    { 10, 11,  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 12, 13, 14, 15 },
    { 8,  9,  6,  7, 10, 11,  2,  3,  0,  1,  4,  5, 12, 13, 14, 15 }
  }
};
#else
static const int8_t shTab[4][6][16] = {
  {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 }, 
    {  6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9, 12, 13, 14, 15 }, 
    {  0,  1,  2,  3, 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13 } 
  },
  {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }, 
    { 14, 15, 12, 13,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11 }, 
    {  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,  2,  3,  0,  1 } 
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 }, 
    { 14, 15, 12, 13,  6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9 },
    { 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13,  2,  3,  0,  1 }
  }
};
#endif

template<X86_VEXT vext> 
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
static void simdFixFilter13x13Db9Blk( AlfClassifier **classifier, const CPelBuf &srcLuma, const Area& curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  const Area &blkDst,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const CPelBuf &srcLumaBeforeDb,
#endif
  Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
  )
#else
static void simdFilter13x13Blk( AlfClassifier **classifier, const CPelBuf &srcLuma, const Area& curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  const Area &blkDst,
#endif
  Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4] )
#endif
{
  const int srcStride = srcLuma.stride;

  constexpr int shift = AdaptiveLoopFilter::m_NUM_BITS_FIXED_FILTER - 1;
  constexpr int round = 1 << (shift - 1);

  const int width = curBlk.width;
  const int height = curBlk.height;

  constexpr int stepX = 8;
  constexpr int stepY = 2;

  const Pel *src = srcLuma.buf + curBlk.y * srcStride + curBlk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if !USE_AVX2
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
  const __m128i mm11 = _mm_set1_epi8(1);
  const __m128i mm3 = _mm_set1_epi16(3);
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool isIntraSlice = cs.slice->isIntra();
  const bool isSpsAdjust = cs.sps->getAlfLumaFixedFilterAdjust();
  const bool useCodingInfo = true;

  const bool useBounCondition = applyCodingInfo && !( !isSpsAdjust && isIntraSlice ) && useCodingInfo;
  const bool useResiCondition = applyCodingInfo && !isIntraSlice && useCodingInfo;
  const int offsetClipValue = 1 << (clpRng.bd - 1);
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const int srcBeforeDbStride = srcLumaBeforeDb.stride;
  const Pel *srcBeforeDb = srcLumaBeforeDb.buf + curBlk.y * srcBeforeDbStride + curBlk.x;
  const int srcBeforeDbStride2 = srcBeforeDbStride * stepY;
  const std::array<std::array<short, FIX_FILTER_NUM_COEFF_13_DB_9>, NUM_FIXED_FILTERS>& filterCoeffFixed = packedDataFixedFilters13Db9[fixedFiltQpInd];
  const int fixedFiltIndF0 = fixedFiltInd > 1 ? fixedFiltInd - EXT_LENGTH : -1;
#else
  const std::array<std::array<short, 42>, NUM_FIXED_FILTERS>& filterCoeffFixed = packedDataFixedFilters[fixedFiltQpInd][dirWindSize];
#endif
#if USE_AVX2 
  if (vext >= AVX2 && (width % 16) == 0)
  {

    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax = _mm256_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
    __m256i mmClippingValues256 = _mm256_castsi128_si256(mmClippingValues);
    mmClippingValues256 = _mm256_insertf128_si256(mmClippingValues256, mmClippingValues, 1);
    const __m256i mm11 = _mm256_set1_epi8(1);
    const __m256i mm3 = _mm256_set1_epi16(3);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m256i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m256i mmOriOffset;
    __m256i mmSignOffsetP, mmSignOffsetN;
    __m256i mmAbsOffset;
    __m256i mmAdjOffset;
    __m256i mmZeroVector = _mm256_set1_epi16( 0 );
    __m256i mm01Vector   = _mm256_set1_epi16( 1 );
    __m256i mm08Vector   = _mm256_set1_epi16( 8 );
    __m256i mm16Vector   = _mm256_set1_epi16( 16 );
    __m256i mmPOffsetClipVector = _mm256_set1_epi16( +offsetClipValue );
    __m256i mmNOffsetClipVector = _mm256_set1_epi16( -offsetClipValue );
    //Set Factor
    __m256i mmBsFactor = isIntraSlice ? _mm256_set1_epi16( 4 ) : _mm256_set1_epi16( 3 );
    __m256i mmResiFactor = isIntraSlice ? _mm256_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm256_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif
    for (int i = 0; i < height; i += stepY)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
      const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      AlfClassifier *pClassCodingInfo = nullptr;
      if( useBounCondition || useResiCondition )
      {
        pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
      }
#endif

      for (int j = 0; j < width; j += stepX * 2)
      {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        __m256i params[32];
        __m256i rawCoef[4][9];
#else
        __m256i params[21];
        __m256i rawCoef[4][6];
#endif

        for (int m = 0; m < 4; m++)
        {
          int transposeIdx0 = pClass[j + 2 * m] & 0x3;
          const int filterIdx0 = classIndFixed[pClass[j + 2 * m] >> 2];
          int transposeIdx1 = pClass[j + 2 * m + 8] & 0x3;
          const int filterIdx1 = classIndFixed[pClass[j + 2 * m + 8] >> 2];

          __m128i rawCoef00 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data()));
          __m128i rawCoef01 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 6));
          __m128i rawCoef02 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 12));
          __m128i rawCoef03 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 20));
          __m128i rawCoef04 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 28));
          __m128i rawCoef05 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 34));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          __m128i rawCoef06 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 42));
          __m128i rawCoef07 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 50));
          __m128i rawCoef08 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 56));
#endif
          //transpose
          if (transposeIdx0 != 0)
          {
            const __m128i s00 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][0]);
            const __m128i s01 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][1]);
            const __m128i s02 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][2]);
            const __m128i s03 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][3]);
            const __m128i s04 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][4]);
            const __m128i s05 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][5]);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            const __m128i s06 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][6]);
            const __m128i s07 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][7]);
            const __m128i s08 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx0][8]);
#endif

            __m128i rawTmp[6];
            rawTmp[0] = rawCoef00;
            rawTmp[1] = rawCoef01;
            rawTmp[2] = _mm_shuffle_epi8(rawCoef02, s02);
            rawTmp[3] = _mm_shuffle_epi8(rawCoef03, s03);
            rawTmp[4] = _mm_shuffle_epi8(rawCoef04, s04);
            rawTmp[5] = _mm_shuffle_epi8(rawCoef05, s05);
            rawCoef00 = _mm_add_epi16(rawTmp[0], _mm_and_si128(s00, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef01 = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s00, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef02 = _mm_add_epi16(rawTmp[2], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef03 = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef04 = _mm_add_epi16(rawTmp[4], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef05 = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            rawCoef06 = _mm_shuffle_epi8(rawCoef06, s06);
            rawCoef07 = _mm_shuffle_epi8(rawCoef07, s07);
            rawCoef08 = _mm_shuffle_epi8(rawCoef08, s08);
#endif            
          }

          __m128i rawCoef10 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data()));
          __m128i rawCoef11 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 6));
          __m128i rawCoef12 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 12));
          __m128i rawCoef13 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 20));
          __m128i rawCoef14 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 28));
          __m128i rawCoef15 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 34));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          __m128i rawCoef16 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 42));
          __m128i rawCoef17 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 50));
          __m128i rawCoef18 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 56));
#endif

          //transpose
          if (transposeIdx1 != 0)
          {
            const __m128i s10 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][0]);
            const __m128i s11 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][1]);
            const __m128i s12 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][2]);
            const __m128i s13 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][3]);
            const __m128i s14 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][4]);
            const __m128i s15 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][5]);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            const __m128i s16 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][6]);
            const __m128i s17 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][7]);
            const __m128i s18 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx1][8]);
#endif
            __m128i rawTmp[6];
            rawTmp[0] = rawCoef10;
            rawTmp[1] = rawCoef11;
            rawTmp[2] = _mm_shuffle_epi8(rawCoef12, s12);
            rawTmp[3] = _mm_shuffle_epi8(rawCoef13, s13);
            rawTmp[4] = _mm_shuffle_epi8(rawCoef14, s14);
            rawTmp[5] = _mm_shuffle_epi8(rawCoef15, s15);
            rawCoef10 = _mm_add_epi16(rawTmp[0], _mm_and_si128(s10, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef11 = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s10, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef12 = _mm_add_epi16(rawTmp[2], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef13 = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef14 = _mm_add_epi16(rawTmp[4], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef15 = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            rawCoef16 = _mm_shuffle_epi8(rawCoef16, s16);
            rawCoef17 = _mm_shuffle_epi8(rawCoef17, s17);
            rawCoef18 = _mm_shuffle_epi8(rawCoef18, s18);
#endif           
          }
          rawCoef[m][0] = _mm256_castsi128_si256(rawCoef00);
          rawCoef[m][0] = _mm256_insertf128_si256(rawCoef[m][0], rawCoef10, 1);
          rawCoef[m][1] = _mm256_castsi128_si256(rawCoef01);
          rawCoef[m][1] = _mm256_insertf128_si256(rawCoef[m][1], rawCoef11, 1);
          rawCoef[m][2] = _mm256_castsi128_si256(rawCoef02);
          rawCoef[m][2] = _mm256_insertf128_si256(rawCoef[m][2], rawCoef12, 1);
          rawCoef[m][3] = _mm256_castsi128_si256(rawCoef03);
          rawCoef[m][3] = _mm256_insertf128_si256(rawCoef[m][3], rawCoef13, 1);
          rawCoef[m][4] = _mm256_castsi128_si256(rawCoef04);
          rawCoef[m][4] = _mm256_insertf128_si256(rawCoef[m][4], rawCoef14, 1);
          rawCoef[m][5] = _mm256_castsi128_si256(rawCoef05);
          rawCoef[m][5] = _mm256_insertf128_si256(rawCoef[m][5], rawCoef15, 1);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          rawCoef[m][6] = _mm256_castsi128_si256(rawCoef06);
          rawCoef[m][6] = _mm256_insertf128_si256(rawCoef[m][6], rawCoef16, 1);
          rawCoef[m][7] = _mm256_castsi128_si256(rawCoef07);
          rawCoef[m][7] = _mm256_insertf128_si256(rawCoef[m][7], rawCoef17, 1);
          rawCoef[m][8] = _mm256_castsi128_si256(rawCoef08);
          rawCoef[m][8] = _mm256_insertf128_si256(rawCoef[m][8], rawCoef18, 1);
#endif
        }//for(m)

        params[0] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[3] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[4] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[6] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[7] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[18] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        params[21] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]), _mm256_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[22] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]), _mm256_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[23] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]), _mm256_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[24] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]), _mm256_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));

        params[25] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]), _mm256_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[26] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]), _mm256_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[27] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][7], rawCoef[1][7]), _mm256_unpackhi_epi32(rawCoef[2][7], rawCoef[3][7]));

        params[28] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]), _mm256_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[29] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]), _mm256_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[30] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]), _mm256_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[31] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]), _mm256_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        mmClassIdxBsP = _mm256_set1_epi16( 0 );
        mmClassIdxBsN = _mm256_set1_epi16( 0 );
        if( useBounCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256( (const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxBsN   = _mm256_sub_epi16( mm01Vector, mmClassIdxBsP );
        }
        mmClassIdxResiP = _mm256_set1_epi16( 0 );
        mmClassIdxResiN = _mm256_set1_epi16( 0 );
        if( useResiCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256( (const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxResiP = _mm256_sub_epi16( mmClassIdxTmp, _mm256_add_epi16( mmClassIdxBsP, mmClassIdxBsP ) );
          mmClassIdxResiN = _mm256_sub_epi16( mm01Vector, mmClassIdxResiP );
        }
#endif
        for (int ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11, *pImg12;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          const Pel *pImg0Cur = src + j + ii * srcStride;
          if (fixedFiltIndF0 >= 0)
          {
            int curPosY = blkDst.y + ii + i + padSize;
            int curPosX = blkDst.x + j + padSize;
            pImg0 = &fixedFilterResults[fixedFiltIndF0][curPosY][curPosX];;
            pImg1 = &fixedFilterResults[fixedFiltIndF0][curPosY + 1][curPosX];
            pImg2 = &fixedFilterResults[fixedFiltIndF0][curPosY - 1][curPosX];
            pImg3 = &fixedFilterResults[fixedFiltIndF0][curPosY + 2][curPosX];
            pImg4 = &fixedFilterResults[fixedFiltIndF0][curPosY - 2][curPosX];
            pImg5 = &fixedFilterResults[fixedFiltIndF0][curPosY + 3][curPosX];
            pImg6 = &fixedFilterResults[fixedFiltIndF0][curPosY - 3][curPosX];
            pImg7 = &fixedFilterResults[fixedFiltIndF0][curPosY + 4][curPosX];
            pImg8 = &fixedFilterResults[fixedFiltIndF0][curPosY - 4][curPosX];
            pImg9 = &fixedFilterResults[fixedFiltIndF0][curPosY + 5][curPosX];
            pImg10 = &fixedFilterResults[fixedFiltIndF0][curPosY - 5][curPosX];
            pImg11 = &fixedFilterResults[fixedFiltIndF0][curPosY + 6][curPosX];
            pImg12 = &fixedFilterResults[fixedFiltIndF0][curPosY - 6][curPosX];
          }
          else
          {
#endif
            pImg0 = src + j + ii * srcStride;
            pImg1 = pImg0 + srcStride;
            pImg2 = pImg0 - srcStride;
            pImg3 = pImg1 + srcStride;
            pImg4 = pImg2 - srcStride;
            pImg5 = pImg3 + srcStride;
            pImg6 = pImg4 - srcStride;
            pImg7 = pImg5 + srcStride;
            pImg8 = pImg6 - srcStride;
            pImg9 = pImg7 + srcStride;
            pImg10 = pImg8 - srcStride;
            pImg11 = pImg9 + srcStride;
            pImg12 = pImg10 - srcStride;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          }
          const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb, *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
          pImg0BeforeDb = srcBeforeDb + j + ii * srcBeforeDbStride;
          pImg1BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
          pImg2BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
          pImg3BeforeDb = pImg1BeforeDb + srcBeforeDbStride;
          pImg4BeforeDb = pImg2BeforeDb - srcBeforeDbStride;
          pImg5BeforeDb = pImg3BeforeDb + srcBeforeDbStride;
          pImg6BeforeDb = pImg4BeforeDb - srcBeforeDbStride;
          pImg7BeforeDb = pImg5BeforeDb + srcBeforeDbStride;
          pImg8BeforeDb = pImg6BeforeDb - srcBeforeDbStride;
          __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0Cur);
#else
          __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0);
#endif
          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_blend_epi16(val00, _mm256_slli_si256(val10, 2), 0xAA);
            __m256i val01B = _mm256_blend_epi16(_mm256_srli_si256(val00, 2), val10, 0xAA);
            __m256i val01C = _mm256_blend_epi16(val01, _mm256_slli_si256(val11, 2), 0xAA);
            __m256i val01D = _mm256_blend_epi16(_mm256_srli_si256(val01, 2), val11, 0xAA);

            __m256i mmClippingFixed = _mm256_and_si256(params[i], mm3);

            __m256i mmClippingFixed2 = _mm256_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2 = _mm256_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m256i xmm2 = _mm256_add_epi8(mmClippingFixed2, mm11);
            __m256i xmmA = _mm256_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m256i limit = _mm256_shuffle_epi8(mmClippingValues256, xmmA);

            val01A = _mm256_min_epi16(val01A, limit);
            val01B = _mm256_min_epi16(val01B, limit);
            val01C = _mm256_min_epi16(val01C, limit);
            val01D = _mm256_min_epi16(val01D, limit);

            limit = _mm256_sub_epi16(_mm256_setzero_si256(), limit);

            val01A = _mm256_max_epi16(val01A, limit);
            val01B = _mm256_max_epi16(val01B, limit);
            val01C = _mm256_max_epi16(val01C, limit);
            val01D = _mm256_max_epi16(val01D, limit);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff = _mm256_srai_epi16(params[i], 2);

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg11 + 0, pImg12 + 0, pImg9 + 0, pImg10 - 0);
          process2coeffs(1, pImg7 + 0, pImg8 + 0, pImg5 - 0, pImg6 + 0);
          process2coeffs(2, pImg3 + 0, pImg4 - 0, pImg1 + 0, pImg2 - 0);

          process2coeffs(3, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
          process2coeffs(4, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
          process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

          process2coeffs(6, pImg9 + 1, pImg10 - 1, pImg7 + 2, pImg8 - 2);
          process2coeffs(7, pImg5 + 3, pImg6 - 3, pImg3 + 4, pImg4 - 4);
          process2coeffs(8, pImg1 + 5, pImg2 - 5, pImg5 + 1, pImg6 - 1);
          process2coeffs(9, pImg3 + 2, pImg4 - 2, pImg1 + 3, pImg2 - 3);

          process2coeffs(10, pImg9 - 1, pImg10 + 1, pImg7 - 2, pImg8 + 2);
          process2coeffs(11, pImg5 - 3, pImg6 + 3, pImg3 - 4, pImg4 + 4);
          process2coeffs(12, pImg1 - 5, pImg2 + 5, pImg5 - 1, pImg6 + 1);
          process2coeffs(13, pImg3 - 2, pImg4 + 2, pImg1 - 3, pImg2 + 3);

          process2coeffs(14, pImg7 + 1, pImg8 - 1, pImg5 + 2, pImg6 - 2);
          process2coeffs(15, pImg3 + 3, pImg4 - 3, pImg1 + 4, pImg2 - 4);
          process2coeffs(16, pImg3 + 1, pImg4 - 1, pImg1 + 2, pImg2 - 2);

          process2coeffs(17, pImg1 + 1, pImg2 - 1, pImg1 - 1, pImg2 + 1);

          process2coeffs(18, pImg7 - 1, pImg8 + 1, pImg5 - 2, pImg6 + 2);
          process2coeffs(19, pImg3 - 3, pImg4 + 3, pImg1 - 4, pImg2 + 4);
          process2coeffs(20, pImg3 - 1, pImg4 + 1, pImg1 - 2, pImg2 + 2);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          process2coeffs(21, pImg7BeforeDb + 0, pImg8BeforeDb + 0, pImg5BeforeDb + 0, pImg6BeforeDb - 0);
          process2coeffs(22, pImg3BeforeDb + 0, pImg4BeforeDb + 0, pImg1BeforeDb - 0, pImg2BeforeDb + 0);
          process2coeffs(23, pImg0BeforeDb + 4, pImg0BeforeDb - 4, pImg0BeforeDb + 3, pImg0BeforeDb - 3);
          process2coeffs(24, pImg0BeforeDb - 2, pImg0BeforeDb + 2, pImg0BeforeDb - 1, pImg0BeforeDb + 1);
          process2coeffs(25, pImg6BeforeDb - 1, pImg5BeforeDb + 1, pImg4BeforeDb - 2, pImg3BeforeDb + 2);
          process2coeffs(26, pImg2BeforeDb - 3, pImg1BeforeDb + 3, pImg6BeforeDb + 1, pImg5BeforeDb - 1);
          process2coeffs(27, pImg4BeforeDb + 2, pImg3BeforeDb - 2, pImg2BeforeDb + 3, pImg1BeforeDb - 3);
          process2coeffs(28, pImg4BeforeDb - 1, pImg3BeforeDb + 1, pImg2BeforeDb - 2, pImg1BeforeDb + 2);
          process2coeffs(29, pImg2BeforeDb - 1, pImg1BeforeDb + 1, pImg4BeforeDb + 1, pImg3BeforeDb - 1);
          process2coeffs(30, pImg2BeforeDb + 2, pImg1BeforeDb - 2, pImg2BeforeDb + 1, pImg1BeforeDb - 1);
          process2coeffs(31, pImg0BeforeDb, pImg0Cur, pImg0, pImg0Cur);
#endif

          accumA = _mm256_srai_epi32(accumA, shift);
          accumB = _mm256_srai_epi32(accumB, shift);

          accumA = _mm256_blend_epi16(accumA, _mm256_slli_si256(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
          if( useBounCondition )
          {
            accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 1, N = 0
            mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 0, N = 1
            mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector, mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16( mmOriOffset );
            //BS based Adjustment
            mmAdjOffset = _mm256_mullo_epi16( mmAbsOffset, _mm256_add_epi16( mm16Vector, mmBsFactor ) );
            mmAdjOffset = _mm256_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm256_srai_epi16( mmAdjOffset, 4 );

            __m256i mmTmpAdj = _mm256_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
            __m256i mmTmpOrg = _mm256_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

            __m256i mmTmpFin = _mm256_add_epi16( mmTmpAdj, mmTmpOrg );

            __m256i mmTmpSignP = _mm256_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m256i mmTmpSignN = _mm256_sub_epi16( mmZeroVector, _mm256_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm256_add_epi16( mmTmpSignP, mmTmpSignN );
          }

          if( useResiCondition )
          {
            accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 0, N = 1
            mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 1, N = 0
            mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector, mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16( mmOriOffset );
            //Resi based Adjustment
            mmAdjOffset = _mm256_mullo_epi16( mmAbsOffset, _mm256_add_epi16( mm16Vector, mmResiFactor ) );
            mmAdjOffset = _mm256_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm256_srai_epi16( mmAdjOffset, 4 );

            __m256i mmTmpAdj = _mm256_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
            __m256i mmTmpOrg = _mm256_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

            __m256i mmTmpFin = _mm256_add_epi16( mmTmpAdj, mmTmpOrg );

            __m256i mmTmpSignP = _mm256_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m256i mmTmpSignN = _mm256_sub_epi16( mmZeroVector, _mm256_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm256_add_epi16( mmTmpSignP, mmTmpSignN );
          }
#endif
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii + padSize][blkDst.x + j + padSize])), accumA);
#else
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii + padSize][curBlk.x + j + padSize])), accumA);
#else
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
#endif
        } //for (size_t ii = 0; ii < stepY; ii++)
      }//for (size_t j = 0; j < width; j += stepX*2)
      src += srcStride * stepY;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      srcBeforeDb += srcBeforeDbStride2;
#endif
    }
  }
  else
  {
    const __m128i mmOffset = _mm_set1_epi32(round);
    const __m128i mmMin = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax = _mm_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
    const __m128i mm11 = _mm_set1_epi8(1);
    const __m128i mm3 = _mm_set1_epi16(3);
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m128i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m128i mmOriOffset;
    __m128i mmSignOffsetP, mmSignOffsetN;
    __m128i mmAbsOffset;
    __m128i mmAdjOffset;
    __m128i mmZeroVector = _mm_set1_epi16( 0 );
    __m128i mm01Vector   = _mm_set1_epi16( 1 );
    __m128i mm08Vector   = _mm_set1_epi16( 8 );
    __m128i mm16Vector   = _mm_set1_epi16( 16 );
    __m128i mmPOffsetClipVector = _mm_set1_epi16( +offsetClipValue );
    __m128i mmNOffsetClipVector = _mm_set1_epi16( -offsetClipValue );
    //Set Factor
    __m128i mmBsFactor = isIntraSlice ? _mm_set1_epi16( 4 ) : _mm_set1_epi16( 3 );
    __m128i mmResiFactor = isIntraSlice ? _mm_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif
  for (int i = 0; i < height; i += stepY)
  {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
    const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    AlfClassifier *pClassCodingInfo = nullptr;
    if( useBounCondition || useResiCondition )
    {
      pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
    }
#endif

    for (int j = 0; j < width; j += stepX)
    {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      __m128i params[32];
      __m128i rawCoef[4][9];
#else
      __m128i params[21];
      __m128i rawCoef[4][6];
#endif
      for (int m = 0; m < 4; m++)
      {
        int transposeIdx = pClass[j + 2*m] & 0x3;
        const int filterIdx = classIndFixed[pClass[j + 2*m] >> 2];

        rawCoef[m][0] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data()     ));
        rawCoef[m][1] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() +  6));
        rawCoef[m][2] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 12));
        rawCoef[m][3] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 20));
        rawCoef[m][4] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 28));
        rawCoef[m][5] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 34));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        rawCoef[m][6] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 42));
        rawCoef[m][7] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 50));
        rawCoef[m][8] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 56));
        if( transposeIdx != 0 )
#endif
        //transpose
        {
          const __m128i s0 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][0]);
          const __m128i s1 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][1]);
          const __m128i s2 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][2]);
          const __m128i s3 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][3]);
          const __m128i s4 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][4]);
          const __m128i s5 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][5]);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          const __m128i s6 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][6]);
          const __m128i s7 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][7]);
          const __m128i s8 = _mm_loadu_si128((const __m128i*)shTab[transposeIdx][8]);
#endif

          __m128i rawTmp[6];
          rawTmp[0] = rawCoef[m][0];
          rawTmp[1] = rawCoef[m][1];
          rawTmp[2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
          rawTmp[3] = _mm_shuffle_epi8(rawCoef[m][3], s3);
          rawTmp[4] = _mm_shuffle_epi8(rawCoef[m][4], s4);
          rawTmp[5] = _mm_shuffle_epi8(rawCoef[m][5], s5);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          rawCoef[m][6] = _mm_shuffle_epi8(rawCoef[m][6], s6);
          rawCoef[m][7] = _mm_shuffle_epi8(rawCoef[m][7], s7);
          rawCoef[m][8] = _mm_shuffle_epi8(rawCoef[m][8], s8);
#endif

          rawCoef[m][0] = _mm_add_epi16(rawTmp[0], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
          rawCoef[m][1] = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
          rawCoef[m][2] = _mm_add_epi16(rawTmp[2], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
          rawCoef[m][3] = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
          rawCoef[m][4] = _mm_add_epi16(rawTmp[4], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
          rawCoef[m][5] = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
        }
      }//for(m)

      params[ 0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[ 1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[ 2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

      params[ 3] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[ 4] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[ 5] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

      params[ 6] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[ 7] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[ 8] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[ 9] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

      params[10] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[11] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[12] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[13] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

      params[14] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
      params[15] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
      params[16] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

      params[17] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

      params[18] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
      params[19] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
      params[20] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      params[21] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]), _mm_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
      params[22] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]), _mm_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
      params[23] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]), _mm_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));
      params[24] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]), _mm_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));

      params[25] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]), _mm_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
      params[26] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]), _mm_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
      params[27] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][7], rawCoef[1][7]), _mm_unpackhi_epi32(rawCoef[2][7], rawCoef[3][7]));

      params[28] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]), _mm_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
      params[29] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]), _mm_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
      params[30] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]), _mm_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
      params[31] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]), _mm_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      mmClassIdxBsP = _mm_set1_epi16( 0 );
      mmClassIdxBsN = _mm_set1_epi16( 0 );
      if( useBounCondition )
      {
        mmClassIdxTmp = _mm_loadu_si128( (const __m128i *) ( pClassCodingInfo + j ) );
        mmClassIdxBsP = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxBsN = _mm_sub_epi16( mm01Vector, mmClassIdxBsP );
      }
      mmClassIdxResiP = _mm_set1_epi16( 0 );
      mmClassIdxResiN = _mm_set1_epi16( 0 );
      if( useResiCondition )
      {
        mmClassIdxTmp = _mm_loadu_si128( (const __m128i *) ( pClassCodingInfo + j ) );
        mmClassIdxBsP = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxResiP = _mm_sub_epi16( mmClassIdxTmp , _mm_add_epi16( mmClassIdxBsP, mmClassIdxBsP) );
        mmClassIdxResiN = _mm_sub_epi16( mm01Vector, mmClassIdxResiP );
      }
#endif

      for (int ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11, *pImg12;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        const Pel *pImg0Cur = src + j + ii * srcStride;
        if (fixedFiltIndF0 >= 0)
        {
          int curPosY = blkDst.y + ii + i + padSize;
          int curPosX = blkDst.x + j + padSize;
          pImg0 = &fixedFilterResults[fixedFiltIndF0][curPosY][curPosX];;
          pImg1 = &fixedFilterResults[fixedFiltIndF0][curPosY + 1][curPosX];
          pImg2 = &fixedFilterResults[fixedFiltIndF0][curPosY - 1][curPosX];
          pImg3 = &fixedFilterResults[fixedFiltIndF0][curPosY + 2][curPosX];
          pImg4 = &fixedFilterResults[fixedFiltIndF0][curPosY - 2][curPosX];
          pImg5 = &fixedFilterResults[fixedFiltIndF0][curPosY + 3][curPosX];
          pImg6 = &fixedFilterResults[fixedFiltIndF0][curPosY - 3][curPosX];
          pImg7 = &fixedFilterResults[fixedFiltIndF0][curPosY + 4][curPosX];
          pImg8 = &fixedFilterResults[fixedFiltIndF0][curPosY - 4][curPosX];
          pImg9 = &fixedFilterResults[fixedFiltIndF0][curPosY + 5][curPosX];
          pImg10 = &fixedFilterResults[fixedFiltIndF0][curPosY - 5][curPosX];
          pImg11 = &fixedFilterResults[fixedFiltIndF0][curPosY + 6][curPosX];
          pImg12 = &fixedFilterResults[fixedFiltIndF0][curPosY - 6][curPosX];
        }
        else
        {
#endif
        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;
        pImg9 = pImg7 + srcStride;
        pImg10 = pImg8 - srcStride;
        pImg11 = pImg9 + srcStride;
        pImg12 = pImg10 - srcStride;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        }
        const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb, *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
        pImg0BeforeDb = srcBeforeDb + j + ii * srcBeforeDbStride;
        pImg1BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
        pImg2BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
        pImg3BeforeDb = pImg1BeforeDb + srcBeforeDbStride;
        pImg4BeforeDb = pImg2BeforeDb - srcBeforeDbStride;
        pImg5BeforeDb = pImg3BeforeDb + srcBeforeDbStride;
        pImg6BeforeDb = pImg4BeforeDb - srcBeforeDbStride;
        pImg7BeforeDb = pImg5BeforeDb + srcBeforeDbStride;
        pImg8BeforeDb = pImg6BeforeDb - srcBeforeDbStride;
        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0Cur);
#else
        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
#endif
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
          __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
          __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
          __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

          __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);
          
          __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
          mmClippingFixed2 = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
          __m128i xmm2 = _mm_add_epi8(mmClippingFixed2, mm11);
          __m128i xmmA = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
          __m128i limit = _mm_shuffle_epi8(mmClippingValues, xmmA);

          val01A = _mm_min_epi16(val01A, limit);
          val01B = _mm_min_epi16(val01B, limit);
          val01C = _mm_min_epi16(val01C, limit);
          val01D = _mm_min_epi16(val01D, limit);

          limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

          val01A = _mm_max_epi16(val01A, limit);
          val01B = _mm_max_epi16(val01B, limit);
          val01C = _mm_max_epi16(val01C, limit);
          val01D = _mm_max_epi16(val01D, limit);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff = _mm_srai_epi16(params[i], 2);

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
        };

        process2coeffs(0, pImg11 + 0, pImg12 + 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 + 0, pImg5 - 0, pImg6 + 0);
        process2coeffs(2, pImg3 + 0, pImg4 - 0, pImg1 + 0, pImg2 - 0);

        process2coeffs(3, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(4, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        process2coeffs(6, pImg9 + 1, pImg10 - 1, pImg7 + 2, pImg8 - 2);
        process2coeffs(7, pImg5 + 3, pImg6 - 3, pImg3 + 4, pImg4 - 4);
        process2coeffs(8, pImg1 + 5, pImg2 - 5, pImg5 + 1, pImg6 - 1);
        process2coeffs(9, pImg3 + 2, pImg4 - 2, pImg1 + 3, pImg2 - 3);
        
        process2coeffs(10, pImg9 - 1, pImg10 + 1, pImg7 - 2, pImg8 + 2);
        process2coeffs(11, pImg5 - 3, pImg6 + 3, pImg3 - 4, pImg4 + 4);
        process2coeffs(12, pImg1 - 5, pImg2 + 5, pImg5 - 1, pImg6 + 1);
        process2coeffs(13, pImg3 - 2, pImg4 + 2, pImg1 - 3, pImg2 + 3);
        
        process2coeffs(14, pImg7 + 1, pImg8 - 1, pImg5 + 2, pImg6 - 2);
        process2coeffs(15, pImg3 + 3, pImg4 - 3, pImg1 + 4, pImg2 - 4);
        process2coeffs(16, pImg3 + 1, pImg4 - 1, pImg1 + 2, pImg2 - 2);

        process2coeffs(17, pImg1 + 1, pImg2 - 1, pImg1 - 1, pImg2 + 1);
        
        process2coeffs(18, pImg7 - 1, pImg8 + 1, pImg5 - 2, pImg6 + 2);
        process2coeffs(19, pImg3 - 3, pImg4 + 3, pImg1 - 4, pImg2 + 4);
        process2coeffs(20, pImg3 - 1, pImg4 + 1, pImg1 - 2, pImg2 + 2);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        process2coeffs(21, pImg7BeforeDb + 0, pImg8BeforeDb + 0, pImg5BeforeDb + 0, pImg6BeforeDb - 0);
        process2coeffs(22, pImg3BeforeDb + 0, pImg4BeforeDb + 0, pImg1BeforeDb - 0, pImg2BeforeDb + 0);
        process2coeffs(23, pImg0BeforeDb + 4, pImg0BeforeDb - 4, pImg0BeforeDb + 3, pImg0BeforeDb - 3);
        process2coeffs(24, pImg0BeforeDb - 2, pImg0BeforeDb + 2, pImg0BeforeDb - 1, pImg0BeforeDb + 1);
        process2coeffs(25, pImg6BeforeDb - 1, pImg5BeforeDb + 1, pImg4BeforeDb - 2, pImg3BeforeDb + 2);
        process2coeffs(26, pImg2BeforeDb - 3, pImg1BeforeDb + 3, pImg6BeforeDb + 1, pImg5BeforeDb - 1);
        process2coeffs(27, pImg4BeforeDb + 2, pImg3BeforeDb - 2, pImg2BeforeDb + 3, pImg1BeforeDb - 3);
        process2coeffs(28, pImg4BeforeDb - 1, pImg3BeforeDb + 1, pImg2BeforeDb - 2, pImg1BeforeDb + 2);
        process2coeffs(29, pImg2BeforeDb - 1, pImg1BeforeDb + 1, pImg4BeforeDb + 1, pImg3BeforeDb - 1);
        process2coeffs(30, pImg2BeforeDb + 2, pImg1BeforeDb - 2, pImg2BeforeDb + 1, pImg1BeforeDb - 1);
        process2coeffs(31, pImg0BeforeDb, pImg0Cur, pImg0, pImg0Cur);
#endif

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        if( useBounCondition )
        {
          accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
          accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
          //accumA is Ori Offset
          mmOriOffset = accumA;
          //Calc Sign
          //P = 1, N = 0
          mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
          //P = 0, N = 1
          mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
          //Calc Abs Offset
          mmAbsOffset = _mm_abs_epi16( mmOriOffset );
          //BS based Adjustment
          mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmBsFactor ) );
          mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
          mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

          __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
          __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

          __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

          __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
          __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

          accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
        }

        if( useResiCondition )
        {
          accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
          accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
          //accumA is Ori Offset
          mmOriOffset = accumA;
          //Calc Sign
          //P = 1, N = 0
          mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
          //P = 0, N = 1
          mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
          //Calc Abs Offset
          mmAbsOffset = _mm_abs_epi16( mmOriOffset );
          //Resi based Adjustment
          mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmResiFactor ) );
          mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
          mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

          __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
          __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

          __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

          __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
          __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

          accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
        }
#endif
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii + padSize][blkDst.x + j + padSize])), accumA);
#else
        _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii + padSize][curBlk.x + j + padSize])), accumA);
#else
        _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
#endif
      } //for (size_t ii = 0; ii < stepY; ii++)
    }//for (size_t j = 0; j < width; j += stepX)
    src += srcStride * stepY;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    srcBeforeDb += srcBeforeDbStride2;
#endif
  }
#if USE_AVX2
  }
#endif
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
template<X86_VEXT vext>
static void simdFixFilter9x9Db9Blk(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area& curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
  )
{
  const int srcStride = srcLuma.stride;
  constexpr int shift = AdaptiveLoopFilter::m_NUM_BITS_FIXED_FILTER - 1;
  constexpr int round = 1 << (shift - 1);

  const int width = curBlk.width;
  const int height = curBlk.height;

  constexpr int stepX = 8;
  constexpr int stepY = 2;

  const Pel *src = srcLuma.buf + curBlk.y * srcStride + curBlk.x;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  const int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

#if !USE_AVX2 
  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);
  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
  const __m128i mm11 = _mm_set1_epi8(1);
  const __m128i mm3 = _mm_set1_epi16(3);
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool isIntraSlice = cs.slice->isIntra();
  const bool isSpsAdjust = cs.sps->getAlfLumaFixedFilterAdjust();
  const bool useCodingInfo = true;

  const bool useBounCondition = applyCodingInfo && !( !isSpsAdjust && isIntraSlice ) && useCodingInfo;
  const bool useResiCondition = applyCodingInfo && !isIntraSlice && useCodingInfo;
  const int offsetClipValue = 1 << ( clpRng.bd - 1 );
#endif

  const int srcBeforeDbStride = srcLumaBeforeDb.stride;
  const Pel *srcBeforeDb = srcLumaBeforeDb.buf + curBlk.y * srcBeforeDbStride + curBlk.x;
  const int srcBeforeDbStride2 = srcBeforeDbStride * stepY;
  const std::array<std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9 + 1>, NUM_FIXED_FILTERS>& filterCoeffFixed = packedDataFixedFilters9Db9[fixedFiltQpInd];

#if USE_AVX2 
  if (vext >= AVX2 && (width % 16) == 0)
  {
    const __m256i mmOffset = _mm256_set1_epi32(round);
    const __m256i mmMin = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax = _mm256_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
    __m256i mmClippingValues256 = _mm256_castsi128_si256(mmClippingValues);
    mmClippingValues256 = _mm256_insertf128_si256(mmClippingValues256, mmClippingValues, 1);
    const __m256i mm11 = _mm256_set1_epi8(1);
    const __m256i mm3 = _mm256_set1_epi16(3);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m256i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m256i mmOriOffset;
    __m256i mmSignOffsetP, mmSignOffsetN;
    __m256i mmAbsOffset;
    __m256i mmAdjOffset;
    __m256i mmZeroVector = _mm256_set1_epi16( 0 );
    __m256i mm01Vector   = _mm256_set1_epi16( 1 );
    __m256i mm08Vector   = _mm256_set1_epi16( 8 );
    __m256i mm16Vector   = _mm256_set1_epi16( 16 );
    __m256i mmPOffsetClipVector = _mm256_set1_epi16( +offsetClipValue );
    __m256i mmNOffsetClipVector = _mm256_set1_epi16( -offsetClipValue );
    //Set Factor
    __m256i mmBsFactor = isIntraSlice ? _mm256_set1_epi16( 4 ) : _mm256_set1_epi16( 3 );
    __m256i mmResiFactor = isIntraSlice ? _mm256_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm256_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif
    for (int i = 0; i < height; i += stepY)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
      const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      AlfClassifier *pClassCodingInfo = nullptr;
      if( useBounCondition || useResiCondition )
      {
        pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
      }
#endif

      for (int j = 0; j < width; j += stepX * 2)
      {
        __m256i params[21];
        __m256i rawCoef[4][6];

        for (int m = 0; m < 4; m++)
        {
          int transposeIdx0 = pClass[j + 2 * m] & 0x3;
          const int filterIdx0 = classIndFixed[pClass[j + 2 * m] >> 2];
          int transposeIdx1 = pClass[j + 2 * m + 8] & 0x3;
          const int filterIdx1 = classIndFixed[pClass[j + 2 * m + 8] >> 2];

          __m128i rawCoef00 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data()));
          __m128i rawCoef01 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 8));
          __m128i rawCoef02 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 14));
          __m128i rawCoef03 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 20));
          __m128i rawCoef04 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 28));
          __m128i rawCoef05 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx0].data() + 34));


          //transpose
          if (transposeIdx0 != 0)
          {
            const __m128i s00 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx0][0]);
            const __m128i s01 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx0][1]);
            const __m128i s02 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx0][2]);

            rawCoef00 = _mm_shuffle_epi8(rawCoef00, s00);
            rawCoef01 = _mm_shuffle_epi8(rawCoef01, s01);
            rawCoef02 = _mm_shuffle_epi8(rawCoef02, s02);
            rawCoef03 = _mm_shuffle_epi8(rawCoef03, s00);
            rawCoef04 = _mm_shuffle_epi8(rawCoef04, s01);
            rawCoef05 = _mm_shuffle_epi8(rawCoef05, s02);
          }

          __m128i rawCoef10 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data()));
          __m128i rawCoef11 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 8));
          __m128i rawCoef12 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 14));
          __m128i rawCoef13 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 20));
          __m128i rawCoef14 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 28));
          __m128i rawCoef15 = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx1].data() + 34));


          //transpose
          if (transposeIdx1 != 0)
          {
            const __m128i s10 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx1][0]);
            const __m128i s11 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx1][1]);
            const __m128i s12 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx1][2]);

            rawCoef10 = _mm_shuffle_epi8(rawCoef10, s10);
            rawCoef11 = _mm_shuffle_epi8(rawCoef11, s11);
            rawCoef12 = _mm_shuffle_epi8(rawCoef12, s12);
            rawCoef13 = _mm_shuffle_epi8(rawCoef13, s10);
            rawCoef14 = _mm_shuffle_epi8(rawCoef14, s11);
            rawCoef15 = _mm_shuffle_epi8(rawCoef15, s12);
          }
          rawCoef[m][0] = _mm256_castsi128_si256(rawCoef00);
          rawCoef[m][0] = _mm256_insertf128_si256(rawCoef[m][0], rawCoef10, 1);
          rawCoef[m][1] = _mm256_castsi128_si256(rawCoef01);
          rawCoef[m][1] = _mm256_insertf128_si256(rawCoef[m][1], rawCoef11, 1);
          rawCoef[m][2] = _mm256_castsi128_si256(rawCoef02);
          rawCoef[m][2] = _mm256_insertf128_si256(rawCoef[m][2], rawCoef12, 1);
          rawCoef[m][3] = _mm256_castsi128_si256(rawCoef03);
          rawCoef[m][3] = _mm256_insertf128_si256(rawCoef[m][3], rawCoef13, 1);
          rawCoef[m][4] = _mm256_castsi128_si256(rawCoef04);
          rawCoef[m][4] = _mm256_insertf128_si256(rawCoef[m][4], rawCoef14, 1);
          rawCoef[m][5] = _mm256_castsi128_si256(rawCoef05);
          rawCoef[m][5] = _mm256_insertf128_si256(rawCoef[m][5], rawCoef15, 1);
        }//for(m)

        params[0] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[3] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[4] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[6] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[7] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[18] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        mmClassIdxBsP = _mm256_set1_epi16( 0 );
        mmClassIdxBsN = _mm256_set1_epi16( 0 );
        if( useBounCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256( (const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxBsN   = _mm256_sub_epi16( mm01Vector, mmClassIdxBsP );
        }
        mmClassIdxResiP = _mm256_set1_epi16( 0 );
        mmClassIdxResiN = _mm256_set1_epi16( 0 );
        if( useResiCondition )
        {
          mmClassIdxTmp   = _mm256_loadu_si256( (const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxResiP = _mm256_sub_epi16( mmClassIdxTmp, _mm256_add_epi16( mmClassIdxBsP, mmClassIdxBsP ) );
          mmClassIdxResiN = _mm256_sub_epi16( mm01Vector, mmClassIdxResiP );
        }
#endif
        for (int ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb, *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
          pImg0BeforeDb = srcBeforeDb + j + ii * srcBeforeDbStride;
          pImg1BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
          pImg2BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
          pImg3BeforeDb = pImg1BeforeDb + srcBeforeDbStride;
          pImg4BeforeDb = pImg2BeforeDb - srcBeforeDbStride;
          pImg5BeforeDb = pImg3BeforeDb + srcBeforeDbStride;
          pImg6BeforeDb = pImg4BeforeDb - srcBeforeDbStride;
          pImg7BeforeDb = pImg5BeforeDb + srcBeforeDbStride;
          pImg8BeforeDb = pImg6BeforeDb - srcBeforeDbStride;

          __m256i cur = _mm256_loadu_si256((const __m256i *) pImg0);

          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_blend_epi16(val00, _mm256_slli_si256(val10, 2), 0xAA);
            __m256i val01B = _mm256_blend_epi16(_mm256_srli_si256(val00, 2), val10, 0xAA);
            __m256i val01C = _mm256_blend_epi16(val01, _mm256_slli_si256(val11, 2), 0xAA);
            __m256i val01D = _mm256_blend_epi16(_mm256_srli_si256(val01, 2), val11, 0xAA);

            __m256i mmClippingFixed = _mm256_and_si256(params[i], mm3);

            __m256i mmClippingFixed2 = _mm256_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2 = _mm256_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m256i xmm2 = _mm256_add_epi8(mmClippingFixed2, mm11);
            __m256i xmmA = _mm256_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m256i limit = _mm256_shuffle_epi8(mmClippingValues256, xmmA);

            val01A = _mm256_min_epi16(val01A, limit);
            val01B = _mm256_min_epi16(val01B, limit);
            val01C = _mm256_min_epi16(val01C, limit);
            val01D = _mm256_min_epi16(val01D, limit);

            limit = _mm256_sub_epi16(_mm256_setzero_si256(), limit);

            val01A = _mm256_max_epi16(val01A, limit);
            val01B = _mm256_max_epi16(val01B, limit);
            val01C = _mm256_max_epi16(val01C, limit);
            val01D = _mm256_max_epi16(val01D, limit);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff = _mm256_srai_epi16(params[i], 2);

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg8 + 0, pImg7 + 0, pImg6 - 1, pImg5 + 1);
          process2coeffs(1, pImg4 - 2, pImg3 + 2, pImg2 - 3, pImg1 + 3);
          process2coeffs(2, pImg0 - 4, pImg0 + 4, pImg6 + 1, pImg5 - 1);
          process2coeffs(3, pImg4 + 2, pImg3 - 2, pImg2 + 3, pImg1 - 3);

          process2coeffs(4, pImg6 + 0, pImg5 - 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(5, pImg2 - 2, pImg1 + 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(6, pImg4 + 1, pImg3 - 1, pImg2 + 2, pImg1 - 2);

          process2coeffs(7, pImg4 + 0, pImg3 - 0, pImg2 - 1, pImg1 + 1);
          process2coeffs(8, pImg0 - 2, pImg0 + 2, pImg2 + 1, pImg1 - 1);
          process2coeffs(9, pImg2 + 0, pImg1 - 0, pImg0 - 1, pImg0 + 1);

          process2coeffs(10, pImg8BeforeDb + 0, pImg7BeforeDb + 0, pImg6BeforeDb - 1, pImg5BeforeDb + 1);
          process2coeffs(11, pImg4BeforeDb - 2, pImg3BeforeDb + 2, pImg2BeforeDb - 3, pImg1BeforeDb + 3);
          process2coeffs(12, pImg0BeforeDb - 4, pImg0BeforeDb + 4, pImg6BeforeDb + 1, pImg5BeforeDb - 1);
          process2coeffs(13, pImg4BeforeDb + 2, pImg3BeforeDb - 2, pImg2BeforeDb + 3, pImg1BeforeDb - 3);

          process2coeffs(14, pImg6BeforeDb + 0, pImg5BeforeDb - 0, pImg4BeforeDb - 1, pImg3BeforeDb + 1);
          process2coeffs(15, pImg2BeforeDb - 2, pImg1BeforeDb + 2, pImg0BeforeDb - 3, pImg0BeforeDb + 3);
          process2coeffs(16, pImg4BeforeDb + 1, pImg3BeforeDb - 1, pImg2BeforeDb + 2, pImg1BeforeDb - 2);

          process2coeffs(17, pImg4BeforeDb + 0, pImg3BeforeDb - 0, pImg2BeforeDb - 1, pImg1BeforeDb + 1);
          process2coeffs(18, pImg0BeforeDb - 2, pImg0BeforeDb + 2, pImg2BeforeDb + 1, pImg1BeforeDb - 1);
          process2coeffs(19, pImg2BeforeDb + 0, pImg1BeforeDb - 0, pImg0BeforeDb - 1, pImg0BeforeDb + 1);
          process2coeffs(20, pImg0BeforeDb, pImg0, pImg0, pImg0);

          accumA = _mm256_srai_epi32(accumA, shift);
          accumB = _mm256_srai_epi32(accumB, shift);

          accumA = _mm256_blend_epi16(accumA, _mm256_slli_si256(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
          if( useBounCondition )
          {
            accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 1, N = 0
            mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 0, N = 1
            mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector,  mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16( mmOriOffset );
            //BS based Adjustment
            mmAdjOffset = _mm256_mullo_epi16( mmAbsOffset, _mm256_add_epi16( mm16Vector, mmBsFactor ) );
            mmAdjOffset = _mm256_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm256_srai_epi16( mmAdjOffset, 4 );

            __m256i mmTmpAdj = _mm256_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
            __m256i mmTmpOrg = _mm256_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

            __m256i mmTmpFin = _mm256_add_epi16( mmTmpAdj, mmTmpOrg );

            __m256i mmTmpSignP = _mm256_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m256i mmTmpSignN = _mm256_sub_epi16( mmZeroVector, _mm256_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm256_add_epi16( mmTmpSignP, mmTmpSignN );
          }

          if( useResiCondition )
          {
            accumA = _mm256_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm256_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 1, N = 0
            mmSignOffsetP = _mm256_abs_epi16( _mm256_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 0, N = 1
            mmSignOffsetN = _mm256_abs_epi16( _mm256_sub_epi16( mm01Vector, mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16( mmOriOffset );
            //Resi based Adjustment
            mmAdjOffset = _mm256_mullo_epi16( mmAbsOffset, _mm256_add_epi16( mm16Vector, mmResiFactor ) );
            mmAdjOffset = _mm256_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm256_srai_epi16( mmAdjOffset, 4 );

            __m256i mmTmpAdj = _mm256_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
            __m256i mmTmpOrg = _mm256_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

            __m256i mmTmpFin = _mm256_add_epi16( mmTmpAdj, mmTmpOrg );

            __m256i mmTmpSignP = _mm256_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m256i mmTmpSignN = _mm256_sub_epi16( mmZeroVector, _mm256_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm256_add_epi16( mmTmpSignP, mmTmpSignN );
          }
#endif
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii + padSize][blkDst.x + j + padSize])), accumA);
#else
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm256_storeu_si256((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii + padSize][curBlk.x + j + padSize])), accumA);
#else
          _mm256_storeu_si256((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
#endif
        } //for (size_t ii = 0; ii < stepY; ii++)
      }//for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
      srcBeforeDb += srcBeforeDbStride2;
    }
  }
  else
  {
    const __m128i mmOffset = _mm_set1_epi32(round);
    const __m128i mmMin = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax = _mm_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues);
    const __m128i mm11 = _mm_set1_epi8(1);
    const __m128i mm3 = _mm_set1_epi16(3);
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m128i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m128i mmOriOffset;
    __m128i mmSignOffsetP, mmSignOffsetN;
    __m128i mmAbsOffset;
    __m128i mmAdjOffset;
    __m128i mmZeroVector = _mm_set1_epi16( 0 );
    __m128i mm01Vector   = _mm_set1_epi16( 1 );
    __m128i mm08Vector   = _mm_set1_epi16( 8 );
    __m128i mm16Vector   = _mm_set1_epi16( 16 );
    __m128i mmPOffsetClipVector = _mm_set1_epi16( +offsetClipValue );
    __m128i mmNOffsetClipVector = _mm_set1_epi16( -offsetClipValue );
    //Set Factor
    __m128i mmBsFactor = isIntraSlice ? _mm_set1_epi16( 4 ) : _mm_set1_epi16( 3 );
    __m128i mmResiFactor = isIntraSlice ? _mm_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif
    for (int i = 0; i < height; i += stepY)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
      const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      AlfClassifier *pClassCodingInfo = nullptr;
      if( useBounCondition || useResiCondition )
      {
        pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
      }
#endif

      for (int j = 0; j < width; j += stepX)
      {
        __m128i params[21];
        __m128i rawCoef[4][6];

        for (int m = 0; m < 4; m++)
        {
          int transposeIdx = pClass[j + 2 * m] & 0x3;
          const int filterIdx = classIndFixed[pClass[j + 2 * m] >> 2];

          rawCoef[m][0] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data()));
          rawCoef[m][1] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 8));
          rawCoef[m][2] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 14));
          rawCoef[m][3] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 20));
          rawCoef[m][4] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 28));
          rawCoef[m][5] = _mm_loadu_si128((const __m128i*)(filterCoeffFixed[filterIdx].data() + 34));


          //transpose
          if (transposeIdx != 0)
          {
            const __m128i s0 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx][0]);
            const __m128i s1 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx][1]);
            const __m128i s2 = _mm_loadu_si128((const __m128i*)shTab9Db9[transposeIdx][2]);

            rawCoef[m][0] = _mm_shuffle_epi8(rawCoef[m][0], s0);
            rawCoef[m][1] = _mm_shuffle_epi8(rawCoef[m][1], s1);
            rawCoef[m][2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
            rawCoef[m][3] = _mm_shuffle_epi8(rawCoef[m][3], s0);
            rawCoef[m][4] = _mm_shuffle_epi8(rawCoef[m][4], s1);
            rawCoef[m][5] = _mm_shuffle_epi8(rawCoef[m][5], s2);
          }
        }//for(m)

        params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[3] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[4] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[6] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[7] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]), _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]), _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[18] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]), _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        mmClassIdxBsP = _mm_set1_epi16( 0 );
        mmClassIdxBsN = _mm_set1_epi16( 0 );
        if( useBounCondition )
        {
          mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxBsN   = _mm_sub_epi16( mm01Vector, mmClassIdxBsP );
        }
        mmClassIdxResiP = _mm_set1_epi16( 0 );
        mmClassIdxResiN = _mm_set1_epi16( 0 );
        if( useResiCondition )
        {
          mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
          mmClassIdxResiP = _mm_sub_epi16(  mmClassIdxTmp, _mm_add_epi16( mmClassIdxBsP, mmClassIdxBsP) );
          mmClassIdxResiN = _mm_sub_epi16( mm01Vector, mmClassIdxResiP );
        }
#endif
        for (int ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb, *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
          pImg0BeforeDb = srcBeforeDb + j + ii * srcBeforeDbStride;
          pImg1BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
          pImg2BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
          pImg3BeforeDb = pImg1BeforeDb + srcBeforeDbStride;
          pImg4BeforeDb = pImg2BeforeDb - srcBeforeDbStride;
          pImg5BeforeDb = pImg3BeforeDb + srcBeforeDbStride;
          pImg6BeforeDb = pImg4BeforeDb - srcBeforeDbStride;
          pImg7BeforeDb = pImg5BeforeDb + srcBeforeDbStride;
          pImg8BeforeDb = pImg6BeforeDb - srcBeforeDbStride;

          __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);

          __m128i accumA = mmOffset;
          __m128i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3) {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

            __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
            __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
            __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
            __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

            __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

            __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2 = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m128i xmm2 = _mm_add_epi8(mmClippingFixed2, mm11);
            __m128i xmmA = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m128i limit = _mm_shuffle_epi8(mmClippingValues, xmmA);

            val01A = _mm_min_epi16(val01A, limit);
            val01B = _mm_min_epi16(val01B, limit);
            val01C = _mm_min_epi16(val01C, limit);
            val01D = _mm_min_epi16(val01D, limit);

            limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

            val01A = _mm_max_epi16(val01A, limit);
            val01B = _mm_max_epi16(val01B, limit);
            val01C = _mm_max_epi16(val01C, limit);
            val01D = _mm_max_epi16(val01D, limit);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff = _mm_srai_epi16(params[i], 2);

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg8 + 0, pImg7 + 0, pImg6 - 1, pImg5 + 1);
          process2coeffs(1, pImg4 - 2, pImg3 + 2, pImg2 - 3, pImg1 + 3);
          process2coeffs(2, pImg0 - 4, pImg0 + 4, pImg6 + 1, pImg5 - 1);
          process2coeffs(3, pImg4 + 2, pImg3 - 2, pImg2 + 3, pImg1 - 3);

          process2coeffs(4, pImg6 + 0, pImg5 - 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(5, pImg2 - 2, pImg1 + 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(6, pImg4 + 1, pImg3 - 1, pImg2 + 2, pImg1 - 2);

          process2coeffs(7, pImg4 + 0, pImg3 - 0, pImg2 - 1, pImg1 + 1);
          process2coeffs(8, pImg0 - 2, pImg0 + 2, pImg2 + 1, pImg1 - 1);
          process2coeffs(9, pImg2 + 0, pImg1 - 0, pImg0 - 1, pImg0 + 1);

          process2coeffs(10, pImg8BeforeDb + 0, pImg7BeforeDb + 0, pImg6BeforeDb - 1, pImg5BeforeDb + 1);
          process2coeffs(11, pImg4BeforeDb - 2, pImg3BeforeDb + 2, pImg2BeforeDb - 3, pImg1BeforeDb + 3);
          process2coeffs(12, pImg0BeforeDb - 4, pImg0BeforeDb + 4, pImg6BeforeDb + 1, pImg5BeforeDb - 1);
          process2coeffs(13, pImg4BeforeDb + 2, pImg3BeforeDb - 2, pImg2BeforeDb + 3, pImg1BeforeDb - 3);

          process2coeffs(14, pImg6BeforeDb + 0, pImg5BeforeDb - 0, pImg4BeforeDb - 1, pImg3BeforeDb + 1);
          process2coeffs(15, pImg2BeforeDb - 2, pImg1BeforeDb + 2, pImg0BeforeDb - 3, pImg0BeforeDb + 3);
          process2coeffs(16, pImg4BeforeDb + 1, pImg3BeforeDb - 1, pImg2BeforeDb + 2, pImg1BeforeDb - 2);

          process2coeffs(17, pImg4BeforeDb + 0, pImg3BeforeDb - 0, pImg2BeforeDb - 1, pImg1BeforeDb + 1);
          process2coeffs(18, pImg0BeforeDb - 2, pImg0BeforeDb + 2, pImg2BeforeDb + 1, pImg1BeforeDb - 1);
          process2coeffs(19, pImg2BeforeDb + 0, pImg1BeforeDb - 0, pImg0BeforeDb - 1, pImg0BeforeDb + 1);
          process2coeffs(20, pImg0BeforeDb, pImg0, pImg0, pImg0);

          accumA = _mm_srai_epi32(accumA, shift);
          accumB = _mm_srai_epi32(accumB, shift);

          accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
          if( useBounCondition )
          {
            accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 1, N = 0
            mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 0, N = 1
            mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm_abs_epi16( mmOriOffset );
            //BS based Adjustment
            mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmBsFactor ) );
            mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

            __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
            __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

            __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

            __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
          }

          if( useResiCondition )
          {
            accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
            accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
            //accumA is Ori Offset
            mmOriOffset = accumA;
            //Calc Sign
            //P = 1, N = 0
            mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
            //P = 0, N = 1
            mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
            //Calc Abs Offset
            mmAbsOffset = _mm_abs_epi16( mmOriOffset );
            //Resi based Adjustment
            mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmResiFactor ) );
            mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
            mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

            __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
            __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

            __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

            __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
            __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

            accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
          }
#endif
          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii + padSize][blkDst.x + j + padSize])), accumA);
#else
          _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii + padSize][curBlk.x + j + padSize])), accumA);
#else
          _mm_storeu_si128((__m128i *) (&(fixedFilterResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
#endif
        } //for (size_t ii = 0; ii < stepY; ii++)
      }//for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
      srcBeforeDb += srcBeforeDbStride2;
    }
#if USE_AVX2 
  }
#endif
}

#if JVET_AJ0237_INTERNAL_12BIT
template<X86_VEXT vext>
static void simdDeriveVariance(const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, uint32_t ***variance, int bits)
#else
template<X86_VEXT vext>
static void simdDeriveVariance(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t ***variance)
#endif
{
#if JVET_AJ0237_INTERNAL_12BIT
  // temporary buffer, could be optimized
  int64_t tempData[2][(256 + 10) >> 1][((256 + 16) >> 1) + 8] = { { { 0 } } };
  int bdShift = 2 * std::max(0, bits - 10);
#endif
  const size_t imgStride = srcLuma.stride;
  const Pel *  srcExt = srcLuma.buf;
  int fl = DIST_CLASS;
  const size_t fl2 = fl << 1;
  const size_t imgHExtended = blk.height + fl2;
  //const size_t imgWExtended = blk.width + fl2;

  int numSample = (fl * 2 + 2) * (fl * 2 + 2); 
  int numSample2 = 128 * 128;
  int offset = numSample2 >> 1;

#if !JVET_AJ0237_INTERNAL_12BIT
  int num[8]{ numSample, numSample, numSample, numSample, numSample, numSample, numSample, numSample };
  int mul[8]{ 13, 13, 13, 13, 13, 13, 13, 13 };
  int off[8]{ offset, offset, offset, offset, offset, offset, offset, offset };
#endif
#if USE_AVX2
  if (vext >= AVX2 && (blk.width % 32) == 0)
  {
#if !JVET_AJ0237_INTERNAL_12BIT
    __m256i n = _mm256_loadu_si256((__m256i *) num);
    __m256i m13 = _mm256_loadu_si256((__m256i *) mul);
    __m256i o = _mm256_loadu_si256((__m256i *) off);
#endif
    const int posX = blk.pos().x;
    const int posY = blk.pos().y;

    //const __m256i zeros = _mm256_setzero_si256();
    const __m256i ones = _mm256_set1_epi16(1);
    int iOffset = 0;
    size_t offsetPos = (posY - fl) * imgStride + posX - fl;
    size_t imgStride2 = imgStride << 1;
    const Pel *imgY0 = &srcExt[offsetPos];
    const Pel *imgY1 = &srcExt[offsetPos + imgStride];

    for (int i = 0; i < imgHExtended; i += 2, iOffset += 1, imgY0 += imgStride2, imgY1 += imgStride2)
    {

      for (int j = 0; j < blk.width; j += 32)
      {
        int jOffset = j >> 1;

        __m256i x0 = _mm256_loadu_si256((__m256i *)(imgY0 + j));
        __m256i y0 = _mm256_loadu_si256((__m256i *)(imgY1 + j));

        __m256i x8 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 8));
        __m256i y8 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 8));

        //__m256i s0 = _mm256_hadd_epi16(x0, y0);
        //__m256i s8 = _mm256_hadd_epi16(x8, y8);

        __m256i xx0 = _mm256_madd_epi16(x0, x0);
        __m256i xx8 = _mm256_madd_epi16(x8, x8);
        __m256i yy0 = _mm256_madd_epi16(y0, y0);
        __m256i yy8 = _mm256_madd_epi16(y8, y8);

        x0 = _mm256_add_epi16(x0, y0);
        __m256i s8 = _mm256_add_epi16(x8, y8);

        x0 = _mm256_madd_epi16(x0, ones);
        s8 = _mm256_madd_epi16(s8, ones);

        xx0 = _mm256_add_epi32(xx0, yy0);
        xx8 = _mm256_add_epi32(xx8, yy8);

        __m256i xx2 = _mm256_alignr_epi8(xx8, xx0, 4);
        __m256i xx4 = _mm256_alignr_epi8(xx8, xx0, 8);
        __m256i xx6 = _mm256_alignr_epi8(xx8, xx0, 12);

        __m256i x2 = _mm256_alignr_epi8(s8, x0, 4);
        __m256i x4 = _mm256_alignr_epi8(s8, x0, 8);
        __m256i x6 = _mm256_alignr_epi8(s8, x0, 12);

        yy0 = _mm256_add_epi32(xx0, xx2);
        xx0 = _mm256_add_epi32(xx4, xx6);
        yy0 = _mm256_add_epi32(yy0, xx8);

        y0 = _mm256_add_epi32(x0, x2);
        x4 = _mm256_add_epi32(x4, x6);
        y0 = _mm256_add_epi32(y0, s8);

        __m256i sum2 = _mm256_add_epi32(yy0, xx0);
        __m256i sum = _mm256_add_epi32(y0, x4);

        x0 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 16));
        y0 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 16));

        _mm256_storeu_si256((__m256i *) &variance[2][iOffset][jOffset], sum);
        _mm256_storeu_si256((__m256i *) &variance[3][iOffset][jOffset], sum2);
        //__m128i suml = _mm256_castsi256_si128(sum);
        //__m128i sum2l = _mm256_castsi256_si128(sum2);
        //_mm_storeu_si128((__m128i *) &variance[2][iOffset][jOffset], suml);
        //_mm_storeu_si128((__m128i *) &variance[3][iOffset][jOffset], sum2l);
        //suml = _mm256_castsi256_si128(_mm256_permute2f128_si256(sum,sum,1));
        //sum2l = _mm256_castsi256_si128(_mm256_permute2f128_si256(sum2,sum2,1));
        //_mm_storeu_si128((__m128i *) &variance[2][iOffset][jOffset+4], suml);
        //_mm_storeu_si128((__m128i *) &variance[3][iOffset][jOffset+4], sum2l);

        x8 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 24));
        y8 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 24));

        //__m256i s0 = _mm256_hadd_epi16(x0, y0);
        //__m256i s8 = _mm256_hadd_epi16(x8, y8);

        xx0 = _mm256_madd_epi16(x0, x0);
        xx8 = _mm256_madd_epi16(x8, x8);
        yy0 = _mm256_madd_epi16(y0, y0);
        yy8 = _mm256_madd_epi16(y8, y8);

        x0 = _mm256_add_epi16(x0, y0);
        s8 = _mm256_add_epi16(x8, y8);

        x0 = _mm256_madd_epi16(x0, ones);
        s8 = _mm256_madd_epi16(s8, ones);

        xx0 = _mm256_add_epi32(xx0, yy0);
        xx8 = _mm256_add_epi32(xx8, yy8);

        xx2 = _mm256_alignr_epi8(xx8, xx0, 4);
        xx4 = _mm256_alignr_epi8(xx8, xx0, 8);
        xx6 = _mm256_alignr_epi8(xx8, xx0, 12);

        x2 = _mm256_alignr_epi8(s8, x0, 4);
        x4 = _mm256_alignr_epi8(s8, x0, 8);
        x6 = _mm256_alignr_epi8(s8, x0, 12);

        yy0 = _mm256_add_epi32(xx0, xx2);
        xx0 = _mm256_add_epi32(xx4, xx6);
        yy0 = _mm256_add_epi32(yy0, xx8);

        y0 = _mm256_add_epi32(x0, x2);
        x4 = _mm256_add_epi32(x4, x6);
        y0 = _mm256_add_epi32(y0, s8);

        __m256i summ2 = _mm256_add_epi32(yy0, xx0);
        __m256i summ = _mm256_add_epi32(y0, x4);

        //x0 = x8;
        //y0 = y8;

        _mm256_storeu_si256((__m256i *) &variance[2][iOffset][jOffset + 8], summ);
        _mm256_storeu_si256((__m256i *) &variance[3][iOffset][jOffset + 8], summ2);
        //suml = _mm256_castsi256_si128(summ);
        //sum2l = _mm256_castsi256_si128(summ2);
        //_mm_storeu_si128((__m128i *) &variance[2][iOffset][jOffset+8], suml);
        //_mm_storeu_si128((__m128i *) &variance[3][iOffset][jOffset+8], sum2l);
        //suml = _mm256_castsi256_si128(_mm256_permute2f128_si256(summ,summ,1));
        //sum2l = _mm256_castsi256_si128(_mm256_permute2f128_si256(summ2,summ2,1));
        //_mm_storeu_si128((__m128i *) &variance[2][iOffset][jOffset+12], suml);
        //_mm_storeu_si128((__m128i *) &variance[3][iOffset][jOffset+12], sum2l);

        if (i == 8)
        {
#if JVET_AJ0237_INTERNAL_12BIT
          for (int kk = 0; kk < 4; kk++)
          {
            __m256i x8Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 4][jOffset + kk * 4]));
            __m256i y8Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 4][jOffset + kk * 4]));
            __m256i x6Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 3][jOffset + kk * 4]));
            __m256i y6Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 3][jOffset + kk * 4]));
            __m256i x4Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 2][jOffset + kk * 4]));
            __m256i y4Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 2][jOffset + kk * 4]));
            __m256i x2Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 1][jOffset + kk * 4]));
            __m256i y2Low   = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 1][jOffset + kk * 4]));
            __m256i sumLow  = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset    ][jOffset + kk * 4]));
            __m256i sum2Low = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset    ][jOffset + kk * 4]));

            x8Low = _mm256_add_epi64(sumLow, x8Low);
            y8Low = _mm256_add_epi64(sum2Low, y8Low);

            x4Low = _mm256_add_epi64(x6Low, x4Low);
            y4Low = _mm256_add_epi64(y6Low, y4Low);

            x2Low = _mm256_add_epi64(x8Low, x2Low);
            y2Low = _mm256_add_epi64(y8Low, y2Low);

            sumLow  = _mm256_add_epi64(x4Low, x2Low);
            sum2Low = _mm256_add_epi64(y4Low, y2Low);

            _mm256_storeu_si256((__m256i*) &tempData[0][iOffset - 4][jOffset + kk * 4], sumLow);
            _mm256_storeu_si256((__m256i*) &tempData[1][iOffset - 4][jOffset + kk * 4], sum2Low);

            variance[VARIANCE][iOffset - 4][jOffset + kk * 4] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4] - tempData[0][iOffset - 4][jOffset + kk * 4] * tempData[0][iOffset - 4][jOffset + kk * 4] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 1] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 1] - tempData[0][iOffset - 4][jOffset + kk * 4 + 1] * tempData[0][iOffset - 4][jOffset + kk * 4 + 1] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 2] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 2] - tempData[0][iOffset - 4][jOffset + kk * 4 + 2] * tempData[0][iOffset - 4][jOffset + kk * 4 + 2] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 3] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 3] - tempData[0][iOffset - 4][jOffset + kk * 4 + 3] * tempData[0][iOffset - 4][jOffset + kk * 4 + 3] + offset) >> 3)) >> (14 + bdShift));
          }
#else
          x8 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 4][jOffset]);
          y8 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 4][jOffset]);
          x6 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 3][jOffset]);
          __m256i y6 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 3][jOffset]);
          x4 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 2][jOffset]);
          __m256i y4 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 2][jOffset]);
          x2 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 1][jOffset]);
          __m256i y2 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 1][jOffset]);

          xx8 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 4][jOffset + 8]);
          yy8 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 4][jOffset + 8]);
          xx6 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 3][jOffset + 8]);
          __m256i yy6 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 3][jOffset + 8]);
          xx4 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 2][jOffset + 8]);
          __m256i yy4 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 2][jOffset + 8]);
          xx2 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 1][jOffset + 8]);
          __m256i yy2 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 1][jOffset + 8]);

          x8 = _mm256_add_epi32(sum, x8);
          y8 = _mm256_add_epi32(sum2, y8);
          xx8 = _mm256_add_epi32(summ, xx8);
          yy8 = _mm256_add_epi32(summ2, yy8);

          x4 = _mm256_add_epi32(x6, x4);
          y4 = _mm256_add_epi32(y6, y4);
          xx4 = _mm256_add_epi32(xx6, xx4);
          yy4 = _mm256_add_epi32(yy6, yy4);

          x2 = _mm256_add_epi32(x8, x2);
          y2 = _mm256_add_epi32(y8, y2);
          xx2 = _mm256_add_epi32(xx8, xx2);
          yy2 = _mm256_add_epi32(yy8, yy2);

          sum = _mm256_add_epi32(x4, x2);
          sum2 = _mm256_add_epi32(y4, y2);
          summ = _mm256_add_epi32(xx4, xx2);
          summ2 = _mm256_add_epi32(yy4, yy2);
          _mm256_storeu_si256((__m256i *) &variance[0][iOffset - 4][jOffset], sum);
          _mm256_storeu_si256((__m256i *) &variance[1][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *) &variance[0][iOffset - 4][jOffset + 8], summ);
          _mm256_storeu_si256((__m256i *) &variance[1][iOffset - 4][jOffset + 8], summ2);

          sum2 = _mm256_mullo_epi32(sum2, n);
          summ2 = _mm256_mullo_epi32(summ2, n);
          sum = _mm256_mullo_epi32(sum, sum);
          summ = _mm256_mullo_epi32(summ, summ);
          sum2 = _mm256_add_epi32(sum2, o);
          summ2 = _mm256_add_epi32(summ2, o);
          sum2 = _mm256_sub_epi32(sum2, sum);
          summ2 = _mm256_sub_epi32(summ2, summ);
          sum2 = _mm256_srli_epi32(sum2, 3);
          sum2 = _mm256_mullo_epi32(sum2, m13);
          sum2 = _mm256_srli_epi32(sum2, 14);
          summ2 = _mm256_srli_epi32(summ2, 3);
          summ2 = _mm256_mullo_epi32(summ2, m13);
          summ2 = _mm256_srli_epi32(summ2, 14);
          _mm256_storeu_si256((__m256i *) &variance[VARIANCE][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *) &variance[VARIANCE][iOffset - 4][jOffset + 8], summ2);
#endif
        }
        else if (i > 8)
        {
#if JVET_AJ0237_INTERNAL_12BIT
          for (int kk = 0; kk < 4; kk++)
          {
            __m256i x8Low = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) &variance[2][iOffset - 5][jOffset + kk * 4]));
            __m256i y8Low = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) &variance[3][iOffset - 5][jOffset + kk * 4]));

            __m256i x6Low = _mm256_loadu_si256((__m256i*) &tempData[0][iOffset - 5][jOffset + kk * 4]);
            __m256i y6Low = _mm256_loadu_si256((__m256i*) &tempData[1][iOffset - 5][jOffset + kk * 4]);

            __m256i sumLow  = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) &variance[2][iOffset][jOffset + kk * 4]));
            __m256i sum2Low = _mm256_cvtepi32_epi64(_mm_loadu_si128((__m128i*) &variance[3][iOffset][jOffset + kk * 4]));

            x6Low = _mm256_sub_epi64(x6Low, x8Low);
            y6Low = _mm256_sub_epi64(y6Low, y8Low);

            sumLow  = _mm256_add_epi64(x6Low, sumLow);
            sum2Low = _mm256_add_epi64(y6Low, sum2Low);

            _mm256_storeu_si256((__m256i*)& tempData[0][iOffset - 4][jOffset + kk * 4], sumLow);
            _mm256_storeu_si256((__m256i*)& tempData[1][iOffset - 4][jOffset + kk * 4], sum2Low);

            variance[VARIANCE][iOffset - 4][jOffset + kk * 4] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4] - tempData[0][iOffset - 4][jOffset + kk * 4] * tempData[0][iOffset - 4][jOffset + kk * 4] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 1] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 1] - tempData[0][iOffset - 4][jOffset + kk * 4 + 1] * tempData[0][iOffset - 4][jOffset + kk * 4 + 1] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 2] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 2] - tempData[0][iOffset - 4][jOffset + kk * 4 + 2] * tempData[0][iOffset - 4][jOffset + kk * 4 + 2] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 4 + 3] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 4 + 3] - tempData[0][iOffset - 4][jOffset + kk * 4 + 3] * tempData[0][iOffset - 4][jOffset + kk * 4 + 3] + offset) >> 3)) >> (14 + bdShift));
          }
#else
          x8 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 5][jOffset]);
          xx8 = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 5][jOffset + 8]);
          y8 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 5][jOffset]);
          yy8 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 5][jOffset + 8]);
          x6 = _mm256_loadu_si256((__m256i *)&variance[0][iOffset - 5][jOffset]);
          xx6 = _mm256_loadu_si256((__m256i *)&variance[0][iOffset - 5][jOffset + 8]);
          __m256i y6 = _mm256_loadu_si256((__m256i *)&variance[1][iOffset - 5][jOffset]);
          __m256i yy6 = _mm256_loadu_si256((__m256i *)&variance[1][iOffset - 5][jOffset + 8]);

          x6 = _mm256_sub_epi32(x6, x8);
          xx6 = _mm256_sub_epi32(xx6, xx8);
          y6 = _mm256_sub_epi32(y6, y8);
          yy6 = _mm256_sub_epi32(yy6, yy8);

          sum = _mm256_add_epi32(x6, sum);
          sum2 = _mm256_add_epi32(y6, sum2);
          summ = _mm256_add_epi32(xx6, summ);
          summ2 = _mm256_add_epi32(yy6, summ2);
          _mm256_storeu_si256((__m256i *) &variance[0][iOffset - 4][jOffset], sum);
          _mm256_storeu_si256((__m256i *) &variance[1][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *) &variance[0][iOffset - 4][jOffset + 8], summ);
          _mm256_storeu_si256((__m256i *) &variance[1][iOffset - 4][jOffset + 8], summ2);

          sum2 = _mm256_mullo_epi32(sum2, n);
          summ2 = _mm256_mullo_epi32(summ2, n);
          sum = _mm256_mullo_epi32(sum, sum);
          summ = _mm256_mullo_epi32(summ, summ);
          sum2 = _mm256_add_epi32(sum2, o);
          summ2 = _mm256_add_epi32(summ2, o);
          sum2 = _mm256_sub_epi32(sum2, sum);
          summ2 = _mm256_sub_epi32(summ2, summ);
          sum2 = _mm256_srli_epi32(sum2, 3);
          summ2 = _mm256_srli_epi32(summ2, 3);
          sum2 = _mm256_mullo_epi32(sum2, m13);
          summ2 = _mm256_mullo_epi32(summ2, m13);
          sum2 = _mm256_srli_epi32(sum2, 14);
          summ2 = _mm256_srli_epi32(summ2, 14);
          _mm256_storeu_si256((__m256i *) &variance[VARIANCE][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *) &variance[VARIANCE][iOffset - 4][jOffset + 8], summ2);
#endif
        }
      }

    }
  }
  else
  {
#endif
#if !JVET_AJ0237_INTERNAL_12BIT
    __m128i n = _mm_loadu_si128((__m128i *) num);
    __m128i m13 = _mm_loadu_si128((__m128i *) mul);
    __m128i o = _mm_loadu_si128((__m128i *) off);
#endif
    const int posX = blk.pos().x;
    const int posY = blk.pos().y;

    const __m128i zeros = _mm_setzero_si128();
    int iOffset = 0;
    size_t offsetPos = (posY - fl) * imgStride + posX - fl;
    size_t imgStride2 = imgStride << 1;
    const Pel *imgY0 = &srcExt[offsetPos];
    const Pel *imgY1 = &srcExt[offsetPos + imgStride];

    for (int i = 0; i < imgHExtended; i += 2, iOffset += 1, imgY0 += imgStride2, imgY1 += imgStride2)
    {

      __m128i x0 = _mm_loadu_si128((__m128i *)(imgY0));
      __m128i y0 = _mm_loadu_si128((__m128i *)(imgY1));

      for (int j = 0; j < blk.width; j += 8)
      {
        int jOffset = j >> 1;

        __m128i x8 = _mm_loadu_si128((__m128i *)(imgY0 + j + 8));
        __m128i y8 = _mm_loadu_si128((__m128i *)(imgY1 + j + 8));

        __m128i s0 = _mm_hadd_epi16(x0, y0);
        __m128i s8 = _mm_hadd_epi16(x8, y8);

        __m128i xx0 = _mm_madd_epi16(x0, x0);
        __m128i xx8 = _mm_madd_epi16(x8, x8);
        __m128i yy0 = _mm_madd_epi16(y0, y0);
        __m128i yy8 = _mm_madd_epi16(y8, y8);

        x0 = _mm_unpacklo_epi16(s0, zeros);
        y0 = _mm_unpackhi_epi16(s0, zeros);
        s0 = _mm_unpacklo_epi16(s8, zeros);
        __m128i s2 = _mm_unpackhi_epi16(s8, zeros);

        xx0 = _mm_add_epi32(xx0, yy0);
        xx8 = _mm_add_epi32(xx8, yy8);

        __m128i xx2 = _mm_alignr_epi8(xx8, xx0, 4);
        __m128i xx4 = _mm_alignr_epi8(xx8, xx0, 8);
        __m128i xx6 = _mm_alignr_epi8(xx8, xx0, 12);

        x0 = _mm_add_epi32(x0, y0);
        s0 = _mm_add_epi32(s0, s2);

        __m128i x2 = _mm_alignr_epi8(s0, x0, 4);
        __m128i x4 = _mm_alignr_epi8(s0, x0, 8);
        __m128i x6 = _mm_alignr_epi8(s0, x0, 12);

        yy0 = _mm_add_epi32(xx0, xx2);
        xx0 = _mm_add_epi32(xx4, xx6);
        yy0 = _mm_add_epi32(yy0, xx8);

        y0 = _mm_add_epi32(x0, x2);
        x4 = _mm_add_epi32(x4, x6);
        y0 = _mm_add_epi32(y0, s0);

        __m128i sum2 = _mm_add_epi32(yy0, xx0);
        __m128i sum = _mm_add_epi32(y0, x4);

        x0 = x8;
        y0 = y8;

        _mm_storeu_si128((__m128i *) &variance[2][iOffset][jOffset], sum);
        _mm_storeu_si128((__m128i *) &variance[3][iOffset][jOffset], sum2);

        if (i == 8)
        {
#if JVET_AJ0237_INTERNAL_12BIT
          for (int kk = 0; kk < 2; kk++)
          {
            __m128i x8Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 4][jOffset + kk * 2]));
            __m128i y8Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 4][jOffset + kk * 2]));
            __m128i x6Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 3][jOffset + kk * 2]));
            __m128i y6Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 3][jOffset + kk * 2]));
            __m128i x4Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 2][jOffset + kk * 2]));
            __m128i y4Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 2][jOffset + kk * 2]));
            __m128i x2Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 1][jOffset + kk * 2]));
            __m128i y2Low   = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 1][jOffset + kk * 2]));
            __m128i sumLow  = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset    ][jOffset + kk * 2]));
            __m128i sum2Low = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset    ][jOffset + kk * 2]));

            x8Low   = _mm_add_epi64(sumLow, x8Low);
            y8Low   = _mm_add_epi64(sum2Low, y8Low);

            x4Low   = _mm_add_epi64(x6Low, x4Low);
            y4Low   = _mm_add_epi64(y6Low, y4Low);

            x2Low   = _mm_add_epi64(x8Low, x2Low);
            y2Low   = _mm_add_epi64(y8Low, y2Low);

            sumLow  = _mm_add_epi64(x4Low, x2Low);
            sum2Low = _mm_add_epi64(y4Low, y2Low);

            _mm_storeu_si128((__m128i*) &tempData[0][iOffset - 4][jOffset + kk * 2], sumLow);
            _mm_storeu_si128((__m128i*) &tempData[1][iOffset - 4][jOffset + kk * 2], sum2Low);

            variance[VARIANCE][iOffset - 4][jOffset + kk * 2] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 2] - tempData[0][iOffset - 4][jOffset + kk * 2] * tempData[0][iOffset - 4][jOffset + kk * 2] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 2 + 1] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 2 + 1] - tempData[0][iOffset - 4][jOffset + kk * 2 + 1] * tempData[0][iOffset - 4][jOffset + kk * 2 + 1] + offset) >> 3)) >> (14 + bdShift));
          }
#else
          x8 = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 4][jOffset]);
          y8 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 4][jOffset]);
          x6 = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 3][jOffset]);
          __m128i y6 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 3][jOffset]);
          x4 = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 2][jOffset]);
          __m128i y4 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 2][jOffset]);
          x2 = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 1][jOffset]);
          __m128i y2 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 1][jOffset]);

          x8 = _mm_add_epi32(sum, x8);
          y8 = _mm_add_epi32(sum2, y8);

          x4 = _mm_add_epi32(x6, x4);
          y4 = _mm_add_epi32(y6, y4);

          x2 = _mm_add_epi32(x8, x2);
          y2 = _mm_add_epi32(y8, y2);

          sum = _mm_add_epi32(x4, x2);
          sum2 = _mm_add_epi32(y4, y2);
          _mm_storeu_si128((__m128i *) &variance[0][iOffset - 4][jOffset], sum);
          _mm_storeu_si128((__m128i *) &variance[1][iOffset - 4][jOffset], sum2);

          sum2 = _mm_mullo_epi32(sum2, n);
          sum = _mm_mullo_epi32(sum, sum);
          sum2 = _mm_add_epi32(sum2, o);
          sum2 = _mm_sub_epi32(sum2, sum);
          sum2 = _mm_srli_epi32(sum2, 3);
          sum2 = _mm_mullo_epi32(sum2, m13);
          sum2 = _mm_srli_epi32(sum2, 14);
          _mm_storeu_si128((__m128i *) &variance[VARIANCE][iOffset - 4][jOffset], sum2);
#endif
        }
        else if (i > 8)
        {
#if JVET_AJ0237_INTERNAL_12BIT
          for (int kk = 0; kk < 2; kk++)
          {
            __m128i x8Low = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset - 5][jOffset + kk * 2]));
            __m128i y8Low = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset - 5][jOffset + kk * 2]));
            __m128i x6Low = _mm_loadu_si128((__m128i*) &tempData[0][iOffset - 5][jOffset + kk * 2]);
            __m128i y6Low = _mm_loadu_si128((__m128i*) &tempData[1][iOffset - 5][jOffset + kk * 2]);
            __m128i sumLow  = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[2][iOffset][jOffset + kk * 2]));
            __m128i sum2Low = _mm_cvtepi32_epi64(_mm_loadu_si128((__m128i*) & variance[3][iOffset][jOffset + kk * 2]));

            x6Low = _mm_sub_epi32(x6Low, x8Low);
            y6Low = _mm_sub_epi32(y6Low, y8Low);

            sumLow  = _mm_add_epi32(x6Low, sumLow);
            sum2Low = _mm_add_epi32(y6Low, sum2Low);

            _mm_storeu_si128((__m128i*) & tempData[0][iOffset - 4][jOffset + kk * 2], sumLow);
            _mm_storeu_si128((__m128i*) & tempData[1][iOffset - 4][jOffset + kk * 2], sum2Low);

            variance[VARIANCE][iOffset - 4][jOffset + kk * 2] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 2] - tempData[0][iOffset - 4][jOffset + kk * 2] * tempData[0][iOffset - 4][jOffset + kk * 2] + offset) >> 3)) >> (14 + bdShift));
            variance[VARIANCE][iOffset - 4][jOffset + kk * 2 + 1] = (uint32_t)((13 * ((numSample * tempData[1][iOffset - 4][jOffset + kk * 2 + 1] - tempData[0][iOffset - 4][jOffset + kk * 2 + 1] * tempData[0][iOffset - 4][jOffset + kk * 2 + 1] + offset) >> 3)) >> (14 + bdShift));
          }
#else
          x8 = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 5][jOffset]);
          y8 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 5][jOffset]);
          x6 = _mm_loadu_si128((__m128i *)&variance[0][iOffset - 5][jOffset]);
          __m128i y6 = _mm_loadu_si128((__m128i *)&variance[1][iOffset - 5][jOffset]);

          x6 = _mm_sub_epi32(x6, x8);
          y6 = _mm_sub_epi32(y6, y8);

          sum = _mm_add_epi32(x6, sum);
          sum2 = _mm_add_epi32(y6, sum2);
          _mm_storeu_si128((__m128i *) &variance[0][iOffset - 4][jOffset], sum);
          _mm_storeu_si128((__m128i *) &variance[1][iOffset - 4][jOffset], sum2);

          sum2 = _mm_mullo_epi32(sum2, n);
          sum = _mm_mullo_epi32(sum, sum);
          sum2 = _mm_add_epi32(sum2, o);
          sum2 = _mm_sub_epi32(sum2, sum);
          sum2 = _mm_srli_epi32(sum2, 3);
          sum2 = _mm_mullo_epi32(sum2, m13);
          sum2 = _mm_srli_epi32(sum2, 14);
          _mm_storeu_si128((__m128i *) &variance[VARIANCE][iOffset - 4][jOffset], sum2);
#endif
        }
      }

    }
#if USE_AVX2
  }
#endif
}
#endif

#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
template<X86_VEXT vext>
static void simdFilterResi9x9Db9Blk(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &curBlk, const Area &blkDst, Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
  )
{
  const int srcStride = srcResiLuma.stride;
  constexpr int shift = AdaptiveLoopFilter::m_NUM_BITS_FIXED_FILTER - 1;
  constexpr int round = 1 << (shift - 1);

  const int width = curBlk.width;
  const int height = curBlk.height;

  constexpr int stepX = 8;
  constexpr int stepY = 2;

  const Pel *src = srcResiLuma.buf + curBlk.y * srcStride + curBlk.x;
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool isIntraSlice = cs.slice->isIntra();
  const bool isSpsAdjust = cs.sps->getAlfLumaFixedFilterAdjust();
  const bool useCodingInfo = isSpsAdjust ? true : false;
  const bool useBounCondition = applyCodingInfo && !( !isSpsAdjust && isIntraSlice ) && useCodingInfo;
  const bool useResiCondition = applyCodingInfo && !isIntraSlice && useCodingInfo;
  const int offsetClipValue = 1 << ( clpRng.bd - 1 );
#endif
#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmOffset = _mm_set1_epi32(round);
#endif

  const int     clpRngmin = -clpRng.max;
  const int     clpRngmax = clpRng.max;
#if !( USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION )
  const __m128i mmMin = _mm_set1_epi16(clpRngmin);
  const __m128i mmMax = _mm_set1_epi16(clpRngmax);

  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *) clippingValues);
  const __m128i mm11 = _mm_set1_epi8(1);
  const __m128i mm3 = _mm_set1_epi16(3);
#endif
  const std::array<std::array<short, FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9 + 1>, NUM_FIXED_FILTERS>& filterCoeffFixed = packedDataFixedFilters9Db9Combine[fixedFiltQpInd];
  const Pel zeros[8] = { 0 };

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0;

  if( use256BitSimd )
  {
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    __m256i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
    __m256i mmOriOffset;
    __m256i mmSignOffsetP, mmSignOffsetN;
    __m256i mmAbsOffset;
    __m256i mmAdjOffset;
    __m256i mmZeroVector = _mm256_set1_epi16(0);
    __m256i mm01Vector   = _mm256_set1_epi16(1);
    __m256i mm08Vector   = _mm256_set1_epi16(8);
    __m256i mm16Vector   = _mm256_set1_epi16(16);
    __m256i mmPOffsetClipVector = _mm256_set1_epi16(+offsetClipValue);
    __m256i mmNOffsetClipVector = _mm256_set1_epi16(-offsetClipValue);
    // Set Factor
    __m256i mmBsFactor = isIntraSlice ? _mm256_set1_epi16( 4 ) : _mm256_set1_epi16( 3 );
    __m256i mmResiFactor = isIntraSlice ? _mm256_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0)) : _mm256_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif

    const __m256i mmOffset = _mm256_set1_epi32(round);

    const int     clpRngmin = -clpRng.max;
    const int     clpRngmax = clpRng.max;
    const __m256i mmMin     = _mm256_set1_epi16(clpRngmin);
    const __m256i mmMax     = _mm256_set1_epi16(clpRngmax);

    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *) clippingValues);
    __m256i mmClippingValues256 = _mm256_castsi128_si256(mmClippingValues);
    mmClippingValues256               = _mm256_insertf128_si256(mmClippingValues256, mmClippingValues, 1);
    const __m256i mm11             = _mm256_set1_epi8(1);
    const __m256i mm3              = _mm256_set1_epi16(3);
    const std::array<std::array<short, FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9 + 1>, NUM_FIXED_FILTERS>
             &filterCoeffFixed = packedDataFixedFilters9Db9Combine[fixedFiltQpInd];
    const Pel zeros[16]         = { 0 };

    for (int i = 0; i < height; i += stepY)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
      const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      AlfClassifier *pClassCodingInfo = nullptr;
      if (useBounCondition || useResiCondition)
      {
        pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
      }
#endif

      for (int j = 0; j < width; j += stepX * 2)
      {
        __m256i params[11];
        __m256i rawCoef[4][3];
        for (int m = 0; m < 4; m++)
        {
          int transposeIdx0 = pClass[j + 2 * m] & 0x3;
          const int filterIdx0    = classIndFixed[pClass[j + 2 * m] >> 2];

          __m128i rawCoef00 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx0].data()));
          __m128i rawCoef01 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx0].data() + 8));
          __m128i rawCoef02 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx0].data() + 14));
          // transpose0
          if (transposeIdx0 != 0)
          {
            const __m128i s00 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx0][0]);
            const __m128i s01 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx0][1]);
            const __m128i s02 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx0][2]);

            rawCoef00 = _mm_shuffle_epi8(rawCoef00, s00);
            rawCoef01 = _mm_shuffle_epi8(rawCoef01, s01);
            rawCoef02 = _mm_shuffle_epi8(rawCoef02, s02);
          }

          int transposeIdx1 = pClass[j + 2 * m + 8] & 0x3;
          const int filterIdx1    = classIndFixed[pClass[j + 2 * m + 8] >> 2];

          __m128i rawCoef10 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx1].data()));
          __m128i rawCoef11 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx1].data() + 8));
          __m128i rawCoef12 = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx1].data() + 14));
          // transpose1
          if (transposeIdx1 != 0)
          {
            const __m128i s10 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx1][0]);
            const __m128i s11 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx1][1]);
            const __m128i s12 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx1][2]);

            rawCoef10 = _mm_shuffle_epi8(rawCoef10, s10);
            rawCoef11 = _mm_shuffle_epi8(rawCoef11, s11);
            rawCoef12 = _mm_shuffle_epi8(rawCoef12, s12);
          }

          rawCoef[m][0] = _mm256_castsi128_si256(rawCoef00);
          rawCoef[m][0] = _mm256_insertf128_si256(rawCoef[m][0], rawCoef10, 1);
          rawCoef[m][1] = _mm256_castsi128_si256(rawCoef01);
          rawCoef[m][1] = _mm256_insertf128_si256(rawCoef[m][1], rawCoef11, 1);
          rawCoef[m][2] = _mm256_castsi128_si256(rawCoef02);
          rawCoef[m][2] = _mm256_insertf128_si256(rawCoef[m][2], rawCoef12, 1);
        }   // for(m)

        params[0] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),  _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),  _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[3] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[4] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[6] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm256_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[7]  = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8]  = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9]  = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[10] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        mmClassIdxBsP = _mm256_set1_epi16( 0 );
        mmClassIdxBsN = _mm256_set1_epi16( 0 );
        if (useBounCondition)
        {
          mmClassIdxTmp = _mm256_loadu_si256((const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP = _mm256_srai_epi16(mmClassIdxTmp, 1);
          mmClassIdxBsN = _mm256_sub_epi16(mm01Vector, mmClassIdxBsP);
        }
        mmClassIdxResiP = _mm256_set1_epi16( 0 );
        mmClassIdxResiN = _mm256_set1_epi16( 0 );
        if (useResiCondition)
        {
          mmClassIdxTmp   = _mm256_loadu_si256((const __m256i *) (pClassCodingInfo + j));
          mmClassIdxBsP   = _mm256_srai_epi16(mmClassIdxTmp, 1);
          mmClassIdxResiP = _mm256_sub_epi16(mmClassIdxTmp, _mm256_add_epi16(mmClassIdxBsP, mmClassIdxBsP));
          mmClassIdxResiN = _mm256_sub_epi16(mm01Vector, mmClassIdxResiP);
        }
#endif
        for (int ii = 0; ii < stepY; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          __m256i cur    = _mm256_loadu_si256((const __m256i *) pImg0);
          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *) ptr3), cur);

            __m256i val01A = _mm256_blend_epi16(val00, _mm256_slli_si256(val10, 2), 0xAA);
            __m256i val01B = _mm256_blend_epi16(_mm256_srli_si256(val00, 2), val10, 0xAA);
            __m256i val01C = _mm256_blend_epi16(val01, _mm256_slli_si256(val11, 2), 0xAA);
            __m256i val01D = _mm256_blend_epi16(_mm256_srli_si256(val01, 2), val11, 0xAA);

            __m256i mmClippingFixed = _mm256_and_si256(params[i], mm3);

            __m256i mmClippingFixed2 = _mm256_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2         = _mm256_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m256i xmm2             = _mm256_add_epi8(mmClippingFixed2, mm11);
            __m256i xmmA             = _mm256_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m256i limit            = _mm256_shuffle_epi8(mmClippingValues256, xmmA);

            val01A = _mm256_min_epi16(val01A, limit);
            val01B = _mm256_min_epi16(val01B, limit);
            val01C = _mm256_min_epi16(val01C, limit);
            val01D = _mm256_min_epi16(val01D, limit);

            limit = _mm256_sub_epi16(_mm256_setzero_si256(), limit);

            val01A = _mm256_max_epi16(val01A, limit);
            val01B = _mm256_max_epi16(val01B, limit);
            val01C = _mm256_max_epi16(val01C, limit);
            val01D = _mm256_max_epi16(val01D, limit);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff = _mm256_srai_epi16(params[i], 2);

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg8 + 0, pImg7 + 0, pImg6 - 1, pImg5 + 1);
          process2coeffs(1, pImg4 - 2, pImg3 + 2, pImg2 - 3, pImg1 + 3);
          process2coeffs(2, pImg0 - 4, pImg0 + 4, pImg6 + 1, pImg5 - 1);
          process2coeffs(3, pImg4 + 2, pImg3 - 2, pImg2 + 3, pImg1 - 3);

          process2coeffs(4, pImg6 + 0, pImg5 - 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(5, pImg2 - 2, pImg1 + 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(6, pImg4 + 1, pImg3 - 1, pImg2 + 2, pImg1 - 2);

          process2coeffs(7, pImg4 + 0, pImg3 - 0, pImg2 - 1, pImg1 + 1);
          process2coeffs(8, pImg0 - 2, pImg0 + 2, pImg2 + 1, pImg1 - 1);
          process2coeffs(9, pImg2 + 0, pImg1 - 0, pImg0 - 1, pImg0 + 1);
          process2coeffs(10, zeros, pImg0, pImg0, pImg0);

          accumA = _mm256_srai_epi32(accumA, shift);
          accumB = _mm256_srai_epi32(accumB, shift);

          accumA = _mm256_blend_epi16(accumA, _mm256_slli_si256(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
          if (useBounCondition)
          {
            accumA = _mm256_min_epi16(mmPOffsetClipVector, accumA);
            accumA = _mm256_max_epi16(mmNOffsetClipVector, accumA);
            // accumA is Ori Offset
            mmOriOffset = accumA;
            // Calc Sign
            // P = 1, N = 0
            mmSignOffsetP = _mm256_abs_epi16(_mm256_cmpgt_epi16(mmOriOffset, mmZeroVector));
            // P = 0, N = 1
            mmSignOffsetN = _mm256_abs_epi16(_mm256_sub_epi16(mm01Vector, mmSignOffsetP));
            // Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16(mmOriOffset);
            // BS based Adjustment
            mmAdjOffset = _mm256_mullo_epi16(mmAbsOffset, _mm256_add_epi16(mm16Vector, mmBsFactor));
            mmAdjOffset = _mm256_add_epi16(mmAdjOffset, mm08Vector);
            mmAdjOffset = _mm256_srai_epi16(mmAdjOffset, 4);

            __m256i mmTmpAdj = _mm256_mullo_epi16(mmClassIdxBsP, mmAdjOffset);
            __m256i mmTmpOrg = _mm256_mullo_epi16(mmClassIdxBsN, mmAbsOffset);

            __m256i mmTmpFin = _mm256_add_epi16(mmTmpAdj, mmTmpOrg);

            __m256i mmTmpSignP = _mm256_mullo_epi16(mmSignOffsetP, mmTmpFin);
            __m256i mmTmpSignN = _mm256_sub_epi16(mmZeroVector, _mm256_mullo_epi16(mmSignOffsetN, mmTmpFin));

            accumA = _mm256_add_epi16(mmTmpSignP, mmTmpSignN);
          }

          if (useResiCondition)
          {
            accumA = _mm256_min_epi16(mmPOffsetClipVector, accumA);
            accumA = _mm256_max_epi16(mmNOffsetClipVector, accumA);
            // accumA is Ori Offset
            mmOriOffset = accumA;
            // Calc Sign
            // P = 1, N = 0
            mmSignOffsetP = _mm256_abs_epi16(_mm256_cmpgt_epi16(mmOriOffset, mmZeroVector));
            // P = 0, N = 1
            mmSignOffsetN = _mm256_abs_epi16(_mm256_sub_epi16(mm01Vector, mmSignOffsetP));
            // Calc Abs Offset
            mmAbsOffset = _mm256_abs_epi16(mmOriOffset);
            // Resi based Adjustment
            mmAdjOffset = _mm256_mullo_epi16(mmAbsOffset, _mm256_add_epi16(mm16Vector, mmResiFactor));
            mmAdjOffset = _mm256_add_epi16(mmAdjOffset, mm08Vector);
            mmAdjOffset = _mm256_srai_epi16(mmAdjOffset, 4);

            __m256i mmTmpAdj = _mm256_mullo_epi16(mmClassIdxResiP, mmAdjOffset);
            __m256i mmTmpOrg = _mm256_mullo_epi16(mmClassIdxResiN, mmAbsOffset);

            __m256i mmTmpFin = _mm256_add_epi16(mmTmpAdj, mmTmpOrg);

            __m256i mmTmpSignP = _mm256_mullo_epi16(mmSignOffsetP, mmTmpFin);
            __m256i mmTmpSignN = _mm256_sub_epi16(mmZeroVector, _mm256_mullo_epi16(mmSignOffsetN, mmTmpFin));

            accumA = _mm256_add_epi16(mmTmpSignP, mmTmpSignN);
          }
#endif
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResiResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#else
          _mm256_storeu_si256((__m256i *) (&(fixedFilterResiResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
        }   // for (size_t ii = 0; ii < stepY; ii++)
      }     // for (size_t j = 0; j < width; j += stepX)
      src += srcStride * stepY;
    }

  }
  else
  {

  const __m128i mmOffset = _mm_set1_epi32(round);

  const __m128i mmMin = _mm_set1_epi16(clpRngmin);
  const __m128i mmMax = _mm_set1_epi16(clpRngmax);

  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *) clippingValues);
  const __m128i mm11 = _mm_set1_epi8(1);
  const __m128i mm3 = _mm_set1_epi16(3);
#endif //Use Avx2 Simd

#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  __m128i mmClassIdxBsP, mmClassIdxResiP, mmClassIdxBsN, mmClassIdxResiN, mmClassIdxTmp;
  __m128i mmOriOffset;
  __m128i mmSignOffsetP, mmSignOffsetN;
  __m128i mmAbsOffset;
  __m128i mmAdjOffset;
  __m128i mmZeroVector = _mm_set1_epi16( 0 );
  __m128i mm01Vector   = _mm_set1_epi16( 1 );
  __m128i mm08Vector   = _mm_set1_epi16( 8 );
  __m128i mm16Vector   = _mm_set1_epi16( 16 );
  __m128i mmPOffsetClipVector = _mm_set1_epi16( +offsetClipValue );
  __m128i mmNOffsetClipVector = _mm_set1_epi16( -offsetClipValue );
  //Set Factor
  __m128i mmBsFactor = isIntraSlice ? _mm_set1_epi16( 4 ) : _mm_set1_epi16( 3 );
  __m128i mmResiFactor = isIntraSlice ? _mm_set1_epi16( 0 >> (!isSpsAdjust ? 1 : 0) ) : _mm_set1_epi16( 3 >> (!isSpsAdjust ? 1 : 0) );
#endif

  for (int i = 0; i < height; i += stepY)
  {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
    const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    AlfClassifier *pClassCodingInfo = nullptr;
    if( useBounCondition || useResiCondition )
    {
      pClassCodingInfo = classifierCodingInfo[blkDst.y + i] + blkDst.x;
    }
#endif

    for (int j = 0; j < width; j += stepX)
    {
      __m128i params[11];
      __m128i rawCoef[4][3];
      for (int m = 0; m < 4; m++)
      {
        int       transposeIdx = pClass[j + 2 * m] & 0x3;
        const int filterIdx = classIndFixed[pClass[j + 2 * m] >> 2];

        rawCoef[m][0] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data()));
        rawCoef[m][1] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 8));
        rawCoef[m][2] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 14));
        // transpose
        if (transposeIdx != 0)
        {
          const __m128i s0 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx][0]);
          const __m128i s1 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx][1]);
          const __m128i s2 = _mm_loadu_si128((const __m128i *) shTab9Db9[transposeIdx][2]);

          rawCoef[m][0] = _mm_shuffle_epi8(rawCoef[m][0], s0);
          rawCoef[m][1] = _mm_shuffle_epi8(rawCoef[m][1], s1);
          rawCoef[m][2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
        }
      }   // for(m)

      params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[3] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]), _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

      params[4] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[5] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[6] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]), _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

      params[7] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[8] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[9] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[10] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]), _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
      mmClassIdxBsP = _mm_set1_epi16( 0 );
      mmClassIdxBsN = _mm_set1_epi16( 0 );
      if( useBounCondition )
      {
        mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
        mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxBsN   = _mm_sub_epi16( mm01Vector, mmClassIdxBsP );
      }
      mmClassIdxResiP = _mm_set1_epi16( 0 );
      mmClassIdxResiN = _mm_set1_epi16( 0 );
      if( useResiCondition )
      {
        mmClassIdxTmp   = _mm_loadu_si128( (const __m128i *) (pClassCodingInfo + j));
        mmClassIdxBsP   = _mm_srai_epi16( mmClassIdxTmp, 1 );
        mmClassIdxResiP = _mm_sub_epi16(  mmClassIdxTmp, _mm_add_epi16( mmClassIdxBsP, mmClassIdxBsP) );
        mmClassIdxResiN = _mm_sub_epi16( mm01Vector, mmClassIdxResiP );
      }
#endif
      for (int ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
        pImg0 = src + j + ii * srcStride;
        pImg1 = pImg0 + srcStride;
        pImg2 = pImg0 - srcStride;
        pImg3 = pImg1 + srcStride;
        pImg4 = pImg2 - srcStride;
        pImg5 = pImg3 + srcStride;
        pImg6 = pImg4 - srcStride;
        pImg7 = pImg5 + srcStride;
        pImg8 = pImg6 - srcStride;

        __m128i cur = _mm_loadu_si128((const __m128i *) pImg0);
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
          __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
          __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
          __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

          __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

          __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
          mmClippingFixed2 = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
          __m128i xmm2 = _mm_add_epi8(mmClippingFixed2, mm11);
          __m128i xmmA = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
          __m128i limit = _mm_shuffle_epi8(mmClippingValues, xmmA);

          val01A = _mm_min_epi16(val01A, limit);
          val01B = _mm_min_epi16(val01B, limit);
          val01C = _mm_min_epi16(val01C, limit);
          val01D = _mm_min_epi16(val01D, limit);

          limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

          val01A = _mm_max_epi16(val01A, limit);
          val01B = _mm_max_epi16(val01B, limit);
          val01C = _mm_max_epi16(val01C, limit);
          val01D = _mm_max_epi16(val01D, limit);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff = _mm_srai_epi16(params[i], 2);

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
        };

        process2coeffs(0, pImg8 + 0, pImg7 + 0, pImg6 - 1, pImg5 + 1);
        process2coeffs(1, pImg4 - 2, pImg3 + 2, pImg2 - 3, pImg1 + 3);
        process2coeffs(2, pImg0 - 4, pImg0 + 4, pImg6 + 1, pImg5 - 1);
        process2coeffs(3, pImg4 + 2, pImg3 - 2, pImg2 + 3, pImg1 - 3);

        process2coeffs(4, pImg6 + 0, pImg5 - 0, pImg4 - 1, pImg3 + 1);
        process2coeffs(5, pImg2 - 2, pImg1 + 2, pImg0 - 3, pImg0 + 3);
        process2coeffs(6, pImg4 + 1, pImg3 - 1, pImg2 + 2, pImg1 - 2);

        process2coeffs(7, pImg4 + 0, pImg3 - 0, pImg2 - 1, pImg1 + 1);
        process2coeffs(8, pImg0 - 2, pImg0 + 2, pImg2 + 1, pImg1 - 1);
        process2coeffs(9, pImg2 + 0, pImg1 - 0, pImg0 - 1, pImg0 + 1);
        process2coeffs(10, zeros, pImg0, pImg0, pImg0);

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        if( useBounCondition )
        {
          accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
          accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
          //accumA is Ori Offset
          mmOriOffset = accumA;
          //Calc Sign
          //P = 1, N = 0
          mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
          //P = 0, N = 1
          mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
          //Calc Abs Offset
          mmAbsOffset = _mm_abs_epi16( mmOriOffset );
          //BS based Adjustment
          mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmBsFactor ) );
          mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector );
          mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

          __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxBsP, mmAdjOffset );
          __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxBsN, mmAbsOffset );

          __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

          __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
          __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

          accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
        }

        if( useResiCondition )
        {
          accumA = _mm_min_epi16( mmPOffsetClipVector, accumA );
          accumA = _mm_max_epi16( mmNOffsetClipVector, accumA );
          //accumA is Ori Offset
          mmOriOffset = accumA;
          //Calc Sign
          //P = 1, N = 0
          mmSignOffsetP = _mm_abs_epi16( _mm_cmpgt_epi16( mmOriOffset , mmZeroVector ));
          //P = 0, N = 1
          mmSignOffsetN = _mm_abs_epi16( _mm_sub_epi16( mm01Vector, mmSignOffsetP ));
          //Calc Abs Offset
          mmAbsOffset = _mm_abs_epi16( mmOriOffset );
          //Resi based Adjustment
          mmAdjOffset = _mm_mullo_epi16( mmAbsOffset, _mm_add_epi16( mm16Vector, mmResiFactor ) );
          mmAdjOffset = _mm_add_epi16( mmAdjOffset, mm08Vector);
          mmAdjOffset = _mm_srai_epi16( mmAdjOffset, 4 );

          __m128i mmTmpAdj = _mm_mullo_epi16( mmClassIdxResiP, mmAdjOffset );
          __m128i mmTmpOrg = _mm_mullo_epi16( mmClassIdxResiN, mmAbsOffset );

          __m128i mmTmpFin = _mm_add_epi16( mmTmpAdj, mmTmpOrg );

          __m128i mmTmpSignP = _mm_mullo_epi16( mmSignOffsetP, mmTmpFin );
          __m128i mmTmpSignN = _mm_sub_epi16( mmZeroVector, _mm_mullo_epi16( mmSignOffsetN, mmTmpFin ));

          accumA = _mm_add_epi16( mmTmpSignP, mmTmpSignN );
        }
#endif
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        _mm_storeu_si128((__m128i *) (&(fixedFilterResiResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#else
        _mm_storeu_si128((__m128i *) (&(fixedFilterResiResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
      }   // for (size_t ii = 0; ii < stepY; ii++)
    }     // for (size_t j = 0; j < width; j += stepX)
    src += srcStride * stepY;
  }
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//Use 256 Bit Simd
#endif
}
#else
template<X86_VEXT vext>
static void simdFilterResi13x13Blk(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                   const Area &blkDst,
#endif
                                   Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd,
                                   const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                                   const ClpRng &clpRng, const Pel clippingValues[4])
{
  const int srcStride = srcResiLuma.stride;

  constexpr int shift = AdaptiveLoopFilter::m_NUM_BITS_FIXED_FILTER - 1;
  constexpr int round = 1 << (shift - 1);

  const int width  = curBlk.width;
  const int height = curBlk.height;

  constexpr int stepX = 8;
  constexpr int stepY = 2;

  const Pel *src = srcResiLuma.buf + curBlk.y * srcStride + curBlk.x;

  const __m128i mmOffset = _mm_set1_epi32(round);

  const int     clpRngmin = -1024;
  const int     clpRngmax = 1024;
  const __m128i mmMin     = _mm_set1_epi16(clpRngmin);
  const __m128i mmMax     = _mm_set1_epi16(clpRngmax);

  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *) clippingValues);
  const __m128i mm11             = _mm_set1_epi8(1);
  const __m128i mm3              = _mm_set1_epi16(3);

  const std::array<std::array<short, 42>, NUM_FIXED_FILTERS> &filterCoeffFixed =
    packedDataFixedFilters[fixedFiltQpInd][dirWindSize];

  for (int i = 0; i < height; i += stepY)
  {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    const AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
#else
    const AlfClassifier *pClass = classifier[curBlk.y + i] + curBlk.x;
#endif

    for (int j = 0; j < width; j += stepX)
    {
      __m128i params[21];
      __m128i rawCoef[4][6];
      for (int m = 0; m < 4; m++)
      {
        int       transposeIdx = pClass[j + 2 * m] & 0x3;
        const int filterIdx    = classIndFixed[pClass[j + 2 * m] >> 2];

        rawCoef[m][0] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data()));
        rawCoef[m][1] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 6));
        rawCoef[m][2] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 12));
        rawCoef[m][3] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 20));
        rawCoef[m][4] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 28));
        rawCoef[m][5] = _mm_loadu_si128((const __m128i *) (filterCoeffFixed[filterIdx].data() + 34));

        // transpose
        {
          const __m128i s0 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][0]);
          const __m128i s1 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][1]);
          const __m128i s2 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][2]);
          const __m128i s3 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][3]);
          const __m128i s4 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][4]);
          const __m128i s5 = _mm_loadu_si128((const __m128i *) shTab[transposeIdx][5]);

          __m128i rawTmp[6];
          rawTmp[0] = rawCoef[m][0];
          rawTmp[1] = rawCoef[m][1];
          rawTmp[2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
          rawTmp[3] = _mm_shuffle_epi8(rawCoef[m][3], s3);
          rawTmp[4] = _mm_shuffle_epi8(rawCoef[m][4], s4);
          rawTmp[5] = _mm_shuffle_epi8(rawCoef[m][5], s5);

          rawCoef[m][0] = _mm_add_epi16(rawTmp[0], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
          rawCoef[m][1] = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
          rawCoef[m][2] = _mm_add_epi16(rawTmp[2], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
          rawCoef[m][3] = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
          rawCoef[m][4] = _mm_add_epi16(rawTmp[4], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
          rawCoef[m][5] = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
        }
      }   // for(m)

      params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

      params[3] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[4] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[5] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

      params[6] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                     _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[7] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                     _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[8] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                     _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[9] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                     _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

      params[10] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                      _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[11] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                      _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[12] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                      _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
      params[13] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                      _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

      params[14] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                      _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
      params[15] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                      _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
      params[16] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                      _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

      params[17] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                      _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

      params[18] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                      _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
      params[19] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                      _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
      params[20] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                      _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

      for (int ii = 0; ii < stepY; ii++)
      {
        const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11,
          *pImg12;
        pImg0  = src + j + ii * srcStride;
        pImg1  = pImg0 + srcStride;
        pImg2  = pImg0 - srcStride;
        pImg3  = pImg1 + srcStride;
        pImg4  = pImg2 - srcStride;
        pImg5  = pImg3 + srcStride;
        pImg6  = pImg4 - srcStride;
        pImg7  = pImg5 + srcStride;
        pImg8  = pImg6 - srcStride;
        pImg9  = pImg7 + srcStride;
        pImg10 = pImg8 - srcStride;
        pImg11 = pImg9 + srcStride;
        pImg12 = pImg10 - srcStride;

        __m128i cur    = _mm_loadu_si128((const __m128i *) pImg0);
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *) ptr3), cur);

          __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
          __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
          __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
          __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

          __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

          __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
          mmClippingFixed2         = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
          __m128i xmm2             = _mm_add_epi8(mmClippingFixed2, mm11);
          __m128i xmmA             = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
          __m128i limit            = _mm_shuffle_epi8(mmClippingValues, xmmA);

          val01A = _mm_min_epi16(val01A, limit);
          val01B = _mm_min_epi16(val01B, limit);
          val01C = _mm_min_epi16(val01C, limit);
          val01D = _mm_min_epi16(val01D, limit);

          limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

          val01A = _mm_max_epi16(val01A, limit);
          val01B = _mm_max_epi16(val01B, limit);
          val01C = _mm_max_epi16(val01C, limit);
          val01D = _mm_max_epi16(val01D, limit);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff = _mm_srai_epi16(params[i], 2);

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
        };

        process2coeffs(0, pImg11 + 0, pImg12 + 0, pImg9 + 0, pImg10 - 0);
        process2coeffs(1, pImg7 + 0, pImg8 + 0, pImg5 - 0, pImg6 + 0);
        process2coeffs(2, pImg3 + 0, pImg4 - 0, pImg1 + 0, pImg2 - 0);

        process2coeffs(3, pImg0 + 6, pImg0 - 6, pImg0 + 5, pImg0 - 5);
        process2coeffs(4, pImg0 + 4, pImg0 - 4, pImg0 + 3, pImg0 - 3);
        process2coeffs(5, pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1);

        process2coeffs(6, pImg9 + 1, pImg10 - 1, pImg7 + 2, pImg8 - 2);
        process2coeffs(7, pImg5 + 3, pImg6 - 3, pImg3 + 4, pImg4 - 4);
        process2coeffs(8, pImg1 + 5, pImg2 - 5, pImg5 + 1, pImg6 - 1);
        process2coeffs(9, pImg3 + 2, pImg4 - 2, pImg1 + 3, pImg2 - 3);

        process2coeffs(10, pImg9 - 1, pImg10 + 1, pImg7 - 2, pImg8 + 2);
        process2coeffs(11, pImg5 - 3, pImg6 + 3, pImg3 - 4, pImg4 + 4);
        process2coeffs(12, pImg1 - 5, pImg2 + 5, pImg5 - 1, pImg6 + 1);
        process2coeffs(13, pImg3 - 2, pImg4 + 2, pImg1 - 3, pImg2 + 3);

        process2coeffs(14, pImg7 + 1, pImg8 - 1, pImg5 + 2, pImg6 - 2);
        process2coeffs(15, pImg3 + 3, pImg4 - 3, pImg1 + 4, pImg2 - 4);
        process2coeffs(16, pImg3 + 1, pImg4 - 1, pImg1 + 2, pImg2 - 2);

        process2coeffs(17, pImg1 + 1, pImg2 - 1, pImg1 - 1, pImg2 + 1);

        process2coeffs(18, pImg7 - 1, pImg8 + 1, pImg5 - 2, pImg6 + 2);
        process2coeffs(19, pImg3 - 3, pImg4 + 3, pImg1 - 4, pImg2 + 4);
        process2coeffs(20, pImg3 - 1, pImg4 + 1, pImg1 - 2, pImg2 + 2);

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        _mm_storeu_si128((__m128i *) (&(fixedFilterResiResults[fixedFiltInd][blkDst.y + i + ii][blkDst.x + j])), accumA);
#else
        _mm_storeu_si128((__m128i *) (&(fixedFilterResiResults[fixedFiltInd][curBlk.y + i + ii][curBlk.x + j])), accumA);
#endif
      }   // for (size_t ii = 0; ii < stepY; ii++)
    }     // for (size_t j = 0; j < width; j += stepX)
    src += srcStride * stepY;
  }
}
#endif
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
static void simdDeriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS], const int side)
#else
static void simdDeriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS])
#endif
{
  const size_t imgStride = srcLuma.stride;
  const Pel *  srcExt = srcLuma.buf;
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  const int flP1 = side + 1;
  const int fl2 = side << 1;
#else
  const size_t flP1 = ALF_CLASSIFIER_FL + 1;
  const size_t fl2 = ALF_CLASSIFIER_FL << 1;
#endif
  const int imgHExtended = blk.height + fl2;
  const int imgWExtended = blk.width + fl2;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  const size_t stride2 = imgStride * 2;
  const size_t stride3 = imgStride * 3;

  uint16_t colSums2x2[NUM_DIRECTIONS][(AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 10) >> 1]
    [((AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 16) >> 1) + 4];

  for (int i = 0; i < imgHExtended; i += 2)
  {
    const size_t offset = (i + posY - flP1) * imgStride + posX - flP1;

    const Pel *imgY0 = &srcExt[offset];
    const Pel *imgY1 = &srcExt[offset + imgStride];
    const Pel *imgY2 = &srcExt[offset + stride2];
    const Pel *imgY3 = &srcExt[offset + stride3];

    for (int j = 0; j < imgWExtended; j += 8)
    {
      const __m128i x0 = _mm_loadu_si128((const __m128i *) (imgY0 + j));
      const __m128i x1 = _mm_loadu_si128((const __m128i *) (imgY1 + j));
      const __m128i x2 = _mm_loadu_si128((const __m128i *) (imgY2 + j));
      const __m128i x3 = _mm_loadu_si128((const __m128i *) (imgY3 + j));
      const __m128i x0next = _mm_loadu_si128((const __m128i *) (imgY0 + j + 8));
      const __m128i x1next = _mm_loadu_si128((const __m128i *) (imgY1 + j + 8));
      const __m128i x2next = _mm_loadu_si128((const __m128i *) (imgY2 + j + 8));
      const __m128i x3next = _mm_loadu_si128((const __m128i *) (imgY3 + j + 8));

      const __m128i pixel1 = _mm_slli_epi16(_mm_alignr_epi8(x1next, x1, 2), 1);
      const __m128i pixel2 = _mm_slli_epi16(_mm_alignr_epi8(x2next, x2, 2), 1);

      //ver
      __m128i ver1 = _mm_add_epi16(_mm_alignr_epi8(x0next, x0, 2), _mm_alignr_epi8(x2next, x2, 2));
      ver1 = _mm_sub_epi16(pixel1, ver1);
      ver1 = _mm_abs_epi16(ver1);
      __m128i ver2 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 2), _mm_alignr_epi8(x3next, x3, 2));
      ver2 = _mm_sub_epi16(pixel2, ver2);
      ver2 = _mm_abs_epi16(ver2);
      ver1 = _mm_add_epi16(ver1, ver2);  //8 ver (each is 2 values in a col)

      //hor
      __m128i hor1 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 4), x1);
      hor1 = _mm_sub_epi16(pixel1, hor1);
      hor1 = _mm_abs_epi16(hor1);
      __m128i hor2 = _mm_add_epi16(_mm_alignr_epi8(x2next, x2, 4), x2);
      hor2 = _mm_sub_epi16(pixel2, hor2);
      hor2 = _mm_abs_epi16(hor2);
      hor1 = _mm_add_epi16(hor1, hor2);

      //dig0
      __m128i di01 = _mm_add_epi16(_mm_alignr_epi8(x2next, x2, 4), x0);
      di01 = _mm_sub_epi16(pixel1, di01);
      di01 = _mm_abs_epi16(di01);
      __m128i di02 = _mm_add_epi16(_mm_alignr_epi8(x3next, x3, 4), x1);
      di02 = _mm_sub_epi16(pixel2, di02);
      di02 = _mm_abs_epi16(di02);
      di01 = _mm_add_epi16(di01, di02);

      //dig1
      __m128i di11 = _mm_add_epi16(_mm_alignr_epi8(x0next, x0, 4), x2);
      di11 = _mm_sub_epi16(pixel1, di11);
      di11 = _mm_abs_epi16(di11);
      __m128i di12 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 4), x3);
      di12 = _mm_sub_epi16(pixel2, di12);
      di12 = _mm_abs_epi16(di12);
      di11 = _mm_add_epi16(di11, di12);

      __m128i vh = _mm_hadd_epi16(ver1, hor1);   //ver: 2x2, 2x2, 2x2, 2x2; hor: 2x2, 2x2, 2x2, 2x2
      __m128i di = _mm_hadd_epi16(di01, di11);

      _mm_storel_epi64((__m128i *) &colSums2x2[VER][i >> 1][j >> 1], vh);
      _mm_storel_epi64((__m128i *) &colSums2x2[HOR][i >> 1][j >> 1], _mm_srli_si128(vh, 8));
      _mm_storel_epi64((__m128i *) &colSums2x2[DIAG0][i >> 1][j >> 1], di);
      _mm_storel_epi64((__m128i *) &colSums2x2[DIAG1][i >> 1][j >> 1], _mm_srli_si128(di, 8));
    }//(int j = 0; j < imgWExtended; j += 8)
  }//for (int i = 0; i < imgHExtended; i += 2)

  //get 4x4 sums
  for (int i = 0; i < (imgHExtended >> 1); i++)
  {
    for (int j = 0; j < (imgWExtended >> 1); j+=4)
    {   
      __m128i x0v = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[VER][i][j]));
      __m128i x1v = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[VER][i][j + 1]));
      __m128i x0h = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[HOR][i][j]));
      __m128i x1h = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[HOR][i][j + 1]));
      __m128i x0d0 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[DIAG0][i][j]));
      __m128i x1d0 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[DIAG0][i][j + 1]));
      __m128i x0d1 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[DIAG1][i][j]));
      __m128i x1d1 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[DIAG1][i][j + 1]));

      x0v = _mm_add_epi32(x0v, x1v);
      x0h = _mm_add_epi32(x0h, x1h);
      x0d0 = _mm_add_epi32(x0d0, x1d0);
      x0d1 = _mm_add_epi32(x0d1, x1d1);
      _mm_storeu_si128((__m128i *) &laplacian[VER][i][j], x0v);
      _mm_storeu_si128((__m128i *) &laplacian[HOR][i][j], x0h);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG0][i][j], x0d0);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG1][i][j], x0d1); //2x4

      if (i > 0) //4x4
      {
        x1v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i-1][j]);
        x1h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i-1][j]);
        x1d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i-1][j]);
        x1d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i-1][j]);
        x0v = _mm_add_epi32(x0v, x1v);
        x0h = _mm_add_epi32(x0h, x1h);
        x0d0 = _mm_add_epi32(x0d0, x1d0);
        x0d1 = _mm_add_epi32(x0d1, x1d1);
        _mm_storeu_si128((__m128i *) &laplacian[VER][i - 1][j], x0v);
        _mm_storeu_si128((__m128i *) &laplacian[HOR][i - 1][j], x0h);
        _mm_storeu_si128((__m128i *) &laplacian[DIAG0][i - 1][j], x0d0);
        _mm_storeu_si128((__m128i *) &laplacian[DIAG1][i - 1][j], x0d1);
      }
    }//for (int j = 0; j < (imgWExtended >> 1); j+=4)
  }// for (int i = 0; i < (imgHExtended >> 1); i++)
}

static void simdDeriveClassificationLaplacianBig(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS])
{
  //get 12x12 sums, laplacian stores 4x4 sums
  int fl2 = ALF_CLASSIFIER_FL << 1;
  const int imgHExtended = curBlk.height + fl2;
  const int imgWExtended = curBlk.width + fl2;

  //4x12 sums
  for (int i = 0; i < (imgHExtended >> 1); i++)
  {
    __m128i x0v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i][0]);
    __m128i x0h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i][0]);
    __m128i x0d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i][0]);
    __m128i x0d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i][0]);
    for (int j = 4; j < (imgWExtended >> 1); j += 4)
    {      
      __m128i x2v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i][j]);
      __m128i x1v = _mm_shuffle_epi32(_mm_blend_epi16(x0v, x2v, 0x0f), 0x4e);
      x0v = _mm_add_epi32(x0v, x1v);
      x0v = _mm_add_epi32(x0v, x2v);
      
      __m128i x2h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i][j]);
      __m128i x1h = _mm_shuffle_epi32(_mm_blend_epi16(x0h, x2h, 0x0f), 0x4e);
      x0h = _mm_add_epi32(x0h, x1h);
      x0h = _mm_add_epi32(x0h, x2h);

      __m128i x2d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i][j]);
      __m128i x1d0 = _mm_shuffle_epi32(_mm_blend_epi16(x0d0, x2d0, 0x0f), 0x4e);
      x0d0 = _mm_add_epi32(x0d0, x1d0);
      x0d0 = _mm_add_epi32(x0d0, x2d0);

      __m128i x2d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i][j]);
      __m128i x1d1 = _mm_shuffle_epi32(_mm_blend_epi16(x0d1, x2d1, 0x0f), 0x4e);
      x0d1 = _mm_add_epi32(x0d1, x1d1);
      x0d1 = _mm_add_epi32(x0d1, x2d1);   

      _mm_storeu_si128((__m128i *) &laplacian[VER][i][j-4], x0v);
      _mm_storeu_si128((__m128i *) &laplacian[HOR][i][j-4], x0h);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG0][i][j-4], x0d0);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG1][i][j-4], x0d1);
      x0v = x2v;
      x0h = x2h;
      x0d0 = x2d0;
      x0d1 = x2d1;
    }
  }

  for (int i = 0; i < (imgHExtended >> 1) - 5; i++)
  {
    for (int j = 0; j < (imgWExtended >> 1) - 5; j += 4)
    {
      __m128i x0v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i][j]);
      __m128i x1v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i + 2][j]);
      __m128i x2v = _mm_loadu_si128((const __m128i *) &laplacian[VER][i + 4][j]);
      x0v = _mm_add_epi32(x0v, x1v);
      x0v = _mm_add_epi32(x0v, x2v);

      __m128i x0h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i][j]);
      __m128i x1h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i + 2][j]);
      __m128i x2h = _mm_loadu_si128((const __m128i *) &laplacian[HOR][i + 4][j]);
      x0h = _mm_add_epi32(x0h, x1h);
      x0h = _mm_add_epi32(x0h, x2h);

      __m128i x0d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i][j]);
      __m128i x1d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i + 2][j]);
      __m128i x2d0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][i + 4][j]);
      x0d0 = _mm_add_epi32(x0d0, x1d0);
      x0d0 = _mm_add_epi32(x0d0, x2d0);

      __m128i x0d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i][j]);
      __m128i x1d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i + 2][j]);
      __m128i x2d1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][i + 4][j]);
      x0d1 = _mm_add_epi32(x0d1, x1d1);
      x0d1 = _mm_add_epi32(x0d1, x2d1);

      _mm_storeu_si128((__m128i *) &laplacian[VER][i][j], x0v);
      _mm_storeu_si128((__m128i *) &laplacian[HOR][i][j], x0h);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG0][i][j], x0d0);
      _mm_storeu_si128((__m128i *) &laplacian[DIAG1][i][j], x0d1);
    }
  }
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
template<X86_VEXT vext>
#endif
static void simdCalcClass0(AlfClassifier **classifier, const Area &blkDst, const Area &curBlk, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS])
{
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const uint8_t divShift2[16] = { 0, 0, 2, 2, 4, 4, 4, 4,  6,  6,  6,  6, 8, 8, 8, 8 };
  const uint8_t sqrtSum[32] = { 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 };
#endif
  const __m128i shift = _mm_cvtsi32_si128(9 + bitDepth - 4);
  const int multTab[] = { 5628, 1407, 624, 351, 225, 156 };

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && blkDst.width % 16 == 0 ? true : false;

  if( use256BitSimd)
  {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
    const __m256i mult = _mm256_set1_epi32(multTab[dirWindSize % 10]);
#else
    const __m256i mult = _mm256_set1_epi32(multTab[dirWindSize]);
#endif
    const __m256i dirOff = _mm256_set1_epi32(noDir * (noDir + 1));
    const __m256i ones   = _mm256_set1_epi32(1);
    const __m256i zeros  = _mm256_setzero_si256();
    const __m256i scale  = _mm256_set1_epi32(192);

    int lapOffset = (dirWindSize == 1) ? 2 : 0;
    for (int i = 0; i < curBlk.height; i += 2)
    {
      int iOffset = (i >> 1) + lapOffset;
      for (int j = 0; j < curBlk.width; j += 16)
      {
        int jOffset = (j >> 1) + lapOffset;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        int iOffsetV = i >> 1;
        int jOffsetV = j >> 1;
#endif
        __m256i sumV  = _mm256_loadu_si256((const __m256i *) &laplacian[VER][iOffset][jOffset]);   // 8 32-bit values
        __m256i sumH  = _mm256_loadu_si256((const __m256i *) &laplacian[HOR][iOffset][jOffset]);
        __m256i sumD0 = _mm256_loadu_si256((const __m256i *) &laplacian[DIAG0][iOffset][jOffset]);
        __m256i sumD1 = _mm256_loadu_si256((const __m256i *) &laplacian[DIAG1][iOffset][jOffset]);

        // sum += sumV + sumH;
        __m256i tempAct  = _mm256_add_epi32(sumV, sumH);
        __m256i activity = _mm256_mullo_epi32(tempAct, mult);
        activity         = _mm256_srl_epi32(activity, shift);
        activity         = _mm256_min_epi32(activity, scale);

        __m256i xmm2  = activity;
        __m256i xmm0  = _mm256_setzero_si256();
        __m256i xmm15 = _mm256_cmpeq_epi32(xmm0, xmm0);
        __m256i xmm1  = _mm256_srli_epi32(xmm15, 31);
        __m256i xmm7  = _mm256_srli_epi32(xmm15, 29);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        __m256i xmm8 = _mm256_srli_epi32(xmm15, 28);
#endif
        __m256i xmm9 = _mm256_add_epi32(_mm256_slli_epi32(xmm7, 2), xmm1);

        __m256i LUT192 = _mm256_set_epi32(0x0C020A00, 0x0E040608, 0x0E040608, 0x0C020A00, 0x0C020A00, 0x0E040608, 0x0E040608, 0x0C020A00);

        xmm2 = _mm256_or_si256(xmm2, _mm256_srli_epi32(xmm2, 1));
        xmm2 = _mm256_or_si256(xmm2, _mm256_srli_epi32(xmm2, 2));
        xmm2 = _mm256_or_si256(xmm2, _mm256_srli_epi32(xmm2, 4));
        xmm2 = _mm256_mullo_epi16(xmm2, xmm9);
        xmm2 = _mm256_and_si256(_mm256_srli_epi32(xmm2, 5), xmm7);
        xmm2 = _mm256_shuffle_epi8(LUT192, xmm2);

        __m256i xmm4 = _mm256_xor_si256(activity, _mm256_srli_epi32(activity, 1));
//        xmm4         = _mm256_cmplt_epi32(xmm4, activity);
        xmm4         = _mm256_cmpgt_epi32(xmm4, _mm256_sub_epi32( activity, _mm256_set1_epi32(1) ) );
        xmm4         = _mm256_add_epi32( _mm256_abs_epi32(xmm4), _mm256_set1_epi32( -1 ) );

        xmm4         = _mm256_or_si256(_mm256_cmpeq_epi32(activity, xmm1), xmm4);
        xmm4         = _mm256_and_si256(xmm4, xmm1);

        activity = _mm256_or_si256(xmm2, xmm4);

        __m256i hv1 = _mm256_max_epi32(sumV, sumH);
        __m256i hv0 = _mm256_min_epi32(sumV, sumH);

        __m256i d1 = _mm256_max_epi32(sumD0, sumD1);
        __m256i d0 = _mm256_min_epi32(sumD0, sumD1);

        // edgeStrengthHV, to optimize
        __m256i hv0Two   = _mm256_slli_epi32(hv0, 1);
        __m256i hv0Eight = _mm256_slli_epi32(hv0, 3);
        __m256i hv1Two   = _mm256_slli_epi32(hv1, 1);
        __m256i strength = _mm256_cmpgt_epi32(_mm256_slli_epi32(hv1, 2), _mm256_add_epi32(hv0, _mm256_slli_epi32(hv0, 2)));   // 4, 5
        __m256i edgeStrengthHV = _mm256_and_si256(strength, ones);

        strength       = _mm256_cmpgt_epi32(hv1Two, _mm256_add_epi32(hv0, hv0Two));   // 2, 3
        edgeStrengthHV = _mm256_add_epi32(edgeStrengthHV, _mm256_and_si256(strength, ones));

        strength       = _mm256_cmpgt_epi32(hv1, hv0Two);   // 1, 2
        edgeStrengthHV = _mm256_add_epi32(edgeStrengthHV, _mm256_and_si256(strength, ones));

        strength       = _mm256_cmpgt_epi32(hv1, _mm256_add_epi32(hv0, hv0Two));   // 1, 3
        edgeStrengthHV = _mm256_add_epi32(edgeStrengthHV, _mm256_and_si256(strength, ones));

        strength       = _mm256_cmpgt_epi32(hv1Two, _mm256_add_epi32(hv0, hv0Eight));   // 2, 9
        edgeStrengthHV = _mm256_add_epi32(edgeStrengthHV, _mm256_and_si256(strength, ones));

        strength       = _mm256_cmpgt_epi32(hv1, hv0Eight);   // 1, 8
        edgeStrengthHV = _mm256_add_epi32(edgeStrengthHV, _mm256_and_si256(strength, ones));

        // edgeStrengthD, to optimize
        __m256i d0Two   = _mm256_slli_epi32(d0, 1);
        __m256i d0Eight = _mm256_slli_epi32(d0, 3);
        __m256i d1Two   = _mm256_slli_epi32(d1, 1);
        strength        = _mm256_cmpgt_epi32(_mm256_slli_epi32(d1, 2), _mm256_add_epi32(d0, _mm256_slli_epi32(d0, 2)));   // 4, 5
        __m256i edgeStrengthD = _mm256_and_si256(strength, ones);

        strength      = _mm256_cmpgt_epi32(d1Two, _mm256_add_epi32(d0, d0Two));   // 2, 3
        edgeStrengthD = _mm256_add_epi32(edgeStrengthD, _mm256_and_si256(strength, ones));

        strength      = _mm256_cmpgt_epi32(d1, d0Two);   // 1, 2
        edgeStrengthD = _mm256_add_epi32(edgeStrengthD, _mm256_and_si256(strength, ones));

        strength      = _mm256_cmpgt_epi32(d1, _mm256_add_epi32(d0, d0Two));   // 1, 3
        edgeStrengthD = _mm256_add_epi32(edgeStrengthD, _mm256_and_si256(strength, ones));

        strength      = _mm256_cmpgt_epi32(d1Two, _mm256_add_epi32(d0, d0Eight));   // 2, 9
        edgeStrengthD = _mm256_add_epi32(edgeStrengthD, _mm256_and_si256(strength, ones));

        strength      = _mm256_cmpgt_epi32(d1, d0Eight);   // 1, 8
        edgeStrengthD = _mm256_add_epi32(edgeStrengthD, _mm256_and_si256(strength, ones));

        const __m256i hv1Xd0e = _mm256_mul_epi32(hv1, d0);
        const __m256i hv0Xd1e = _mm256_mul_epi32(hv0, d1);
        const __m256i hv1Xd0o = _mm256_mul_epi32(_mm256_srli_si256(hv1, 4), _mm256_srli_si256(d0, 4));
        const __m256i hv0Xd1o = _mm256_mul_epi32(_mm256_srli_si256(hv0, 4), _mm256_srli_si256(d1, 4));

        const __m256i xmme = _mm256_sub_epi64(hv0Xd1e, hv1Xd0e);
        const __m256i xmmo = _mm256_sub_epi64(hv0Xd1o, hv1Xd0o);

        __m256i dirCondition = _mm256_srai_epi32(_mm256_blend_epi16(_mm256_srli_si256(xmme, 4), xmmo, 0xCC), 31);

        __m256i cx        = _mm256_blendv_epi8(edgeStrengthHV, edgeStrengthD, dirCondition);   // x
        __m256i cy        = _mm256_blendv_epi8(edgeStrengthD, edgeStrengthHV, dirCondition);   // y
        __m256i dirOffset = _mm256_blendv_epi8(_mm256_set1_epi32(28), zeros, dirCondition);
        // direction = (y*(y+1))/2 + x
        __m256i direction = _mm256_mullo_epi32(cy, cy);
        direction         = _mm256_add_epi32(direction, cy);
        direction         = _mm256_srli_epi32(direction, 1);
        direction         = _mm256_add_epi32(direction, cx);
        direction         = _mm256_andnot_si256(_mm256_cmpgt_epi32(cx, cy), direction);
        direction         = _mm256_add_epi32(direction, dirOffset);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        __m256i sum2     = _mm256_loadu_si256((const __m256i *) &laplacian[VARIANCE][iOffsetV][jOffsetV]);
        __m256i shiftLut = _mm256_set_m128i( _mm_loadu_si128( (const __m128i *) divShift2),  _mm_loadu_si128( (const __m128i *) divShift2) );
        __m256i shiftVal = _mm256_shuffle_epi8(shiftLut, activity);
        shiftVal         = _mm256_add_epi32(shiftVal, xmm1);
        shiftVal         = _mm256_add_epi32(shiftVal, xmm1);
        if (vext >= AVX2)
        {
          sum2 = _mm256_srlv_epi32(sum2, shiftVal);
        }
        else
        {
          __m128i sum2Tmp0 = _mm256_extracti128_si256(sum2, 0);
          __m128i sum2Tmp1 = _mm256_extracti128_si256(sum2, 1);
          __m128i shiftValTmp0 = _mm256_extracti128_si256(shiftVal, 0);
          __m128i shiftValTmp1 = _mm256_extracti128_si256(shiftVal, 1);

          uint64_t tmpVal0[4];
          int32_t *pVal0 = (int32_t *) tmpVal0;
          _mm_storeu_si128((__m128i *) tmpVal0, sum2Tmp0);
          _mm_storeu_si128((__m128i *) (tmpVal0 + 2), shiftValTmp0);
          pVal0[0] >>= pVal0[4];
          pVal0[1] >>= pVal0[5];
          pVal0[2] >>= pVal0[6];
          pVal0[3] >>= pVal0[7];

          uint64_t tmpVal1[4];
          int32_t *pVal1 = (int32_t *) tmpVal1;
          _mm_storeu_si128((__m128i *) tmpVal1, sum2Tmp1);
          _mm_storeu_si128((__m128i *) (tmpVal1 + 2), shiftValTmp1);
          pVal1[0] >>= pVal1[4];
          pVal1[1] >>= pVal1[5];
          pVal1[2] >>= pVal1[6];
          pVal1[3] >>= pVal1[7];

          sum2 = _mm256_set_m128i(  _mm_loadu_si128((const __m128i *) pVal1 ), _mm_loadu_si128((const __m128i *) pVal0) );
        }

        __m256i LUT0 = _mm256_set_m128i( _mm_loadu_si128((const __m128i *) sqrtSum), _mm_loadu_si128((const __m128i *) sqrtSum) );
        __m256i LUT1  = _mm256_set_m128i(_mm_loadu_si128((const __m128i *) &sqrtSum[16]), _mm_loadu_si128((const __m128i *) &sqrtSum[16]) );
        __m256i xmm16 = _mm256_set_epi32(16, 16, 16, 16, 16, 16, 16, 16);
        __m256i xmm35 = _mm256_set_epi32(35, 35, 35, 35, 35, 35, 35, 35);
        __m256i xmm48 = _mm256_set_epi32(48, 48, 48, 48, 48, 48, 48, 48);

        __m256i use1 = _mm256_cmpgt_epi32(sum2, xmm8);

        __m256i idx0 = _mm256_and_si256(sum2, xmm8);
        __m256i idx1 = _mm256_sub_epi32(sum2, xmm16);
        idx1         = _mm256_min_epi32(idx1, xmm8);

        idx0 = _mm256_shuffle_epi8(LUT0, idx0);
        idx1 = _mm256_shuffle_epi8(LUT1, idx1);

        idx1 = _mm256_add_epi32(idx1, _mm256_slli_epi32(xmm1, 2));

        idx0 = _mm256_andnot_si256(use1, idx0);
        idx1 = _mm256_and_si256(use1, idx1);
        idx0 = _mm256_add_epi32(idx0, idx1);

        xmm35 = _mm256_cmpgt_epi32(sum2, xmm35);
        xmm48 = _mm256_cmpgt_epi32(sum2, xmm48);

        xmm35 = _mm256_and_si256(xmm35, xmm1);
        xmm48 = _mm256_and_si256(xmm48, xmm1);

        xmm35 = _mm256_add_epi32(xmm35, xmm48);

        xmm2     = _mm256_add_epi32(idx0, xmm35);
        xmm2     = _mm256_slli_epi32(xmm2, 4);
        activity = _mm256_add_epi32(activity, xmm2);
#endif
        __m256i classIdx = _mm256_mullo_epi32(dirOff, activity);
        classIdx         = _mm256_add_epi32(classIdx, direction);

        // transpose
        __m256i dirTempHVMinus1 = _mm256_cmpgt_epi32(sumV, sumH);
        __m256i dirTempDMinus1  = _mm256_cmpgt_epi32(sumD0, sumD1);
        __m256i transposeIdx    = _mm256_set1_epi32(3);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempHVMinus1);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempDMinus1);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempDMinus1);

        classIdx = _mm256_slli_epi16(classIdx, 2);
        classIdx = _mm256_add_epi16(classIdx, transposeIdx);
        classIdx = _mm256_shuffle_epi8(classIdx, _mm256_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13, 0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        _mm256_storeu_si256((__m256i *) &classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
        _mm256_storeu_si256((__m256i *) &classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
#else
        _mm256_storeu_si256((__m256i *) &classifier[curBlk.pos().y + i][curBlk.pos().x + j], classIdx);
        _mm256_storeu_si256((__m256i *) &classifier[curBlk.pos().y + i + 1][curBlk.pos().x + j], classIdx);
#endif

      }   // for (int j = 0; j < curBlk.width; j += 16)
    }     // for (int i = 0; i < curBlk.height; i += 2)

  }
  else
  {
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  const __m128i mult = _mm_set1_epi32(multTab[dirWindSize % 10]);
#else  
  const __m128i mult = _mm_set1_epi32(multTab[dirWindSize]);
#endif
  const __m128i dirOff = _mm_set1_epi32(noDir * (noDir + 1));
  const __m128i ones = _mm_set1_epi32(1);
  const __m128i zeros = _mm_setzero_si128();
  const __m128i scale = _mm_set1_epi32(192);

  int lapOffset = (dirWindSize == 1) ? 2 : 0;
  for (int i = 0; i < curBlk.height; i += 2)
  {
    int iOffset = (i >> 1) + lapOffset;
    for (int j = 0; j < curBlk.width; j += 8)
    {
      int jOffset = (j >> 1) + lapOffset;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      int iOffsetV = i >> 1;
      int jOffsetV = j >> 1;
#endif
      __m128i sumV = _mm_loadu_si128((const __m128i *) &laplacian[VER][iOffset][jOffset]);  //4 32-bit values
      __m128i sumH = _mm_loadu_si128((const __m128i *) &laplacian[HOR][iOffset][jOffset]);
      __m128i sumD0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][iOffset][jOffset]);
      __m128i sumD1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][iOffset][jOffset]);

      //sum += sumV + sumH;
      __m128i tempAct = _mm_add_epi32(sumV, sumH);
      __m128i activity = _mm_mullo_epi32(tempAct, mult);
      activity = _mm_srl_epi32(activity, shift);
      activity = _mm_min_epi32(activity, scale);

      __m128i xmm2 = activity;
      __m128i xmm0 = _mm_setzero_si128();
      __m128i xmm15 = _mm_cmpeq_epi32(xmm0, xmm0);
      __m128i xmm1 = _mm_srli_epi32(xmm15, 31);
      __m128i xmm7 = _mm_srli_epi32(xmm15, 29);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      __m128i xmm8 = _mm_srli_epi32(xmm15, 28);
#endif
      __m128i xmm9 = _mm_add_epi32(_mm_slli_epi32(xmm7, 2), xmm1);

      __m128i LUT192 = _mm_set_epi32(0x0C020A00, 0x0E040608, 0x0E040608, 0x0C020A00);

      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 1));
      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 2));
      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 4));
      xmm2 = _mm_mullo_epi16(xmm2, xmm9);
      xmm2 = _mm_and_si128(_mm_srli_epi32(xmm2, 5), xmm7);
      xmm2 = _mm_shuffle_epi8(LUT192, xmm2);

       __m128i xmm4 = _mm_xor_si128(activity, _mm_srli_epi32(activity, 1));
      xmm4 = _mm_cmplt_epi32(xmm4, activity);
      xmm4 = _mm_or_si128(_mm_cmpeq_epi32(activity, xmm1), xmm4);
      xmm4 = _mm_and_si128(xmm4, xmm1);

      activity = _mm_or_si128(xmm2, xmm4);
      
      __m128i hv1 = _mm_max_epi32(sumV, sumH);
      __m128i hv0 = _mm_min_epi32(sumV, sumH);

      __m128i d1 = _mm_max_epi32(sumD0, sumD1);
      __m128i d0 = _mm_min_epi32(sumD0, sumD1);

      //edgeStrengthHV, to optimize
      __m128i hv0Two = _mm_slli_epi32(hv0, 1);
      __m128i hv0Eight = _mm_slli_epi32(hv0, 3);
      __m128i hv1Two = _mm_slli_epi32(hv1, 1);
      __m128i strength = _mm_cmpgt_epi32(_mm_slli_epi32(hv1, 2), _mm_add_epi32(hv0, _mm_slli_epi32(hv0, 2)));  //4, 5
      __m128i edgeStrengthHV = _mm_and_si128(strength, ones);

      strength = _mm_cmpgt_epi32(hv1Two, _mm_add_epi32(hv0, hv0Two)); //2, 3
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(hv1, hv0Two); //1, 2
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(hv1, _mm_add_epi32(hv0, hv0Two)); //1, 3
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(hv1Two, _mm_add_epi32(hv0, hv0Eight)); //2, 9
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(hv1, hv0Eight); //1, 8
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      //edgeStrengthD, to optimize
      __m128i d0Two = _mm_slli_epi32(d0, 1);
      __m128i d0Eight = _mm_slli_epi32(d0, 3);
      __m128i d1Two = _mm_slli_epi32(d1, 1);
      strength = _mm_cmpgt_epi32(_mm_slli_epi32(d1, 2), _mm_add_epi32(d0, _mm_slli_epi32(d0, 2))); //4, 5
      __m128i edgeStrengthD = _mm_and_si128(strength, ones);

      strength = _mm_cmpgt_epi32(d1Two, _mm_add_epi32(d0, d0Two)); //2, 3
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(d1, d0Two); //1, 2
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones)); 

      strength = _mm_cmpgt_epi32(d1, _mm_add_epi32(d0, d0Two)); //1, 3
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(d1Two, _mm_add_epi32(d0, d0Eight));//2, 9
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength = _mm_cmpgt_epi32(d1, d0Eight);//1, 8
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      const __m128i hv1Xd0e = _mm_mul_epi32(hv1, d0); 
      const __m128i hv0Xd1e = _mm_mul_epi32(hv0, d1);
      const __m128i hv1Xd0o = _mm_mul_epi32(_mm_srli_si128(hv1, 4), _mm_srli_si128(d0, 4));
      const __m128i hv0Xd1o = _mm_mul_epi32(_mm_srli_si128(hv0, 4), _mm_srli_si128(d1, 4));

      const __m128i xmme = _mm_sub_epi64(hv0Xd1e, hv1Xd0e);
      const __m128i xmmo = _mm_sub_epi64(hv0Xd1o, hv1Xd0o);

      __m128i dirCondition = _mm_srai_epi32(_mm_blend_epi16(_mm_srli_si128(xmme, 4), xmmo, 0xCC), 31);

      __m128i cx = _mm_blendv_epi8(edgeStrengthHV, edgeStrengthD, dirCondition);  //x
      __m128i cy = _mm_blendv_epi8(edgeStrengthD, edgeStrengthHV, dirCondition);  //y
      __m128i dirOffset = _mm_blendv_epi8(_mm_set1_epi32(28), zeros, dirCondition);
      //direction = (y*(y+1))/2 + x
      __m128i direction = _mm_mullo_epi32(cy, cy);
      direction = _mm_add_epi32(direction, cy);
      direction = _mm_srli_epi32(direction, 1);
      direction = _mm_add_epi32(direction, cx);
      direction = _mm_andnot_si128(_mm_cmpgt_epi32(cx, cy), direction);
      direction = _mm_add_epi32(direction, dirOffset);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      __m128i sum2 = _mm_loadu_si128((const __m128i *) &laplacian[VARIANCE][iOffsetV][jOffsetV]);
      __m128i shiftLut = _mm_loadu_si128((const __m128i *) divShift2);
      __m128i shiftVal = _mm_shuffle_epi8(shiftLut, activity);
      shiftVal = _mm_add_epi32(shiftVal, xmm1);
      shiftVal = _mm_add_epi32(shiftVal, xmm1);
      if (vext >= AVX2)
      {
        sum2 = _mm_srlv_epi32(sum2, shiftVal);
      }
      else
      {
        uint64_t tmpVal[4];
        int32_t  *pVal = (int32_t *)tmpVal;
        _mm_storeu_si128((__m128i *) tmpVal, sum2);
        _mm_storeu_si128((__m128i *) (tmpVal + 2), shiftVal);
        pVal[0] >>= pVal[4];
        pVal[1] >>= pVal[5];
        pVal[2] >>= pVal[6];
        pVal[3] >>= pVal[7];
        sum2 = _mm_loadu_si128((const __m128i *) pVal);
      }

      __m128i LUT0 = _mm_loadu_si128((const __m128i *) sqrtSum);
      __m128i LUT1 = _mm_loadu_si128((const __m128i *) &sqrtSum[16]);
      __m128i xmm16 = _mm_set_epi32(16, 16, 16, 16);
      __m128i xmm35 = _mm_set_epi32(35, 35, 35, 35);
      __m128i xmm48 = _mm_set_epi32(48, 48, 48, 48);

      __m128i use1 = _mm_cmpgt_epi32(sum2, xmm8);

      __m128i idx0 = _mm_and_si128(sum2, xmm8);
      __m128i idx1 = _mm_sub_epi32(sum2, xmm16);
      idx1 = _mm_min_epi32(idx1, xmm8);

      idx0 = _mm_shuffle_epi8(LUT0, idx0);
      idx1 = _mm_shuffle_epi8(LUT1, idx1);

      idx1 = _mm_add_epi32(idx1, _mm_slli_epi32(xmm1, 2));

      idx0 = _mm_andnot_si128(use1, idx0);
      idx1 = _mm_and_si128(use1, idx1);
      idx0 = _mm_add_epi32(idx0, idx1);

      xmm35 = _mm_cmpgt_epi32(sum2, xmm35);
      xmm48 = _mm_cmpgt_epi32(sum2, xmm48);

      xmm35 = _mm_and_si128(xmm35, xmm1);
      xmm48 = _mm_and_si128(xmm48, xmm1);

      xmm35 = _mm_add_epi32(xmm35, xmm48);


      xmm2 = _mm_add_epi32(idx0, xmm35);
      xmm2 = _mm_slli_epi32(xmm2, 4);
      activity = _mm_add_epi32(activity, xmm2);
#endif
      __m128i classIdx = _mm_mullo_epi32(dirOff, activity);
      classIdx = _mm_add_epi32(classIdx, direction);     

      //transpose
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i dirTempDMinus1 = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i transposeIdx = _mm_set1_epi32(3);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempDMinus1);

      classIdx = _mm_slli_epi16(classIdx, 2);
      classIdx = _mm_add_epi16(classIdx, transposeIdx);
      classIdx = _mm_shuffle_epi8(classIdx, _mm_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      _mm_storeu_si128((__m128i *) &classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *) &classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
#else
      _mm_storeu_si128((__m128i *) &classifier[curBlk.pos().y + i][curBlk.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *) &classifier[curBlk.pos().y + i + 1][curBlk.pos().x + j], classIdx);
#endif

    }//for (int j = 0; j < curBlk.width; j += 8)
  }//for (int i = 0; i < curBlk.height; i += 2)
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }// Use 256 Bit Simd
 #endif
}

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
template <X86_VEXT vext>
#endif
static void simdCalcClass1(AlfClassifier **classifier, const Area &blkDst, const Area &curBlk, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS])
{
  const __m128i shift = _mm_cvtsi32_si128(9 + bitDepth);
#if JVET_X0071_ALF_BAND_CLASSIFIER
  const int multTab[] = { 16884, 4221, 1872, 1053, 675, 468 };
#else
  const int multTab[] = { 5628, 1407, 624, 351, 225, 156 };
#endif

#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  const bool use256BitSimd = vext >= AVX2 && curBlk.width % 16 == 0 ? true : false;
  if( use256BitSimd )
  {
    const __m256i mult  = _mm256_set1_epi32(multTab[dirWindSize]);
    const __m256i scale = _mm256_set1_epi32(15);

    for (int i = 0; i < curBlk.height; i += 2)
    {
      int iOffset = i >> 1;
      for (int j = 0; j < curBlk.width; j += 16)
      {
        int     jOffset = j >> 1;
        __m256i sumV    = _mm256_loadu_si256((const __m256i *) &laplacian[VER][iOffset][jOffset]);   // 8 32-bit values
        __m256i sumH    = _mm256_loadu_si256((const __m256i *) &laplacian[HOR][iOffset][jOffset]);
        __m256i sumD0   = _mm256_loadu_si256((const __m256i *) &laplacian[DIAG0][iOffset][jOffset]);
        __m256i sumD1   = _mm256_loadu_si256((const __m256i *) &laplacian[DIAG1][iOffset][jOffset]);

        // sum += sumV + sumH;
        __m256i tempAct  = _mm256_add_epi32(sumV, sumH);
        __m256i activity = _mm256_mullo_epi32(tempAct, mult);
        activity         = _mm256_srl_epi32(activity, shift);
        activity         = _mm256_min_epi32(activity, scale);
        __m256i classIdx = _mm256_shuffle_epi8(_mm256_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);
        classIdx         = _mm256_add_epi32(classIdx, _mm256_slli_epi32(classIdx, 2));   // activity * 5

        __m256i hv1 = _mm256_max_epi32(sumV, sumH);
        __m256i hv0 = _mm256_min_epi32(sumV, sumH);

        __m256i d1 = _mm256_max_epi32(sumD0, sumD1);
        __m256i d0 = _mm256_min_epi32(sumD0, sumD1);

        const __m256i hv1Xd0e = _mm256_mul_epi32(hv1, d0);
        const __m256i hv0Xd1e = _mm256_mul_epi32(hv0, d1);
        const __m256i hv1Xd0o = _mm256_mul_epi32(_mm256_srli_si256(hv1, 4), _mm256_srli_si256(d0, 4));
        const __m256i hv0Xd1o = _mm256_mul_epi32(_mm256_srli_si256(hv0, 4), _mm256_srli_si256(d1, 4));

        const __m256i xmme = _mm256_sub_epi64(hv1Xd0e, hv0Xd1e);
        const __m256i xmmo = _mm256_sub_epi64(hv1Xd0o, hv0Xd1o);

        __m256i dirCondition = _mm256_srai_epi32(_mm256_blend_epi16(_mm256_srli_si256(xmme, 4), xmmo, 0xCC), 31);

        __m256i hvd1      = _mm256_blendv_epi8(hv1, d1, dirCondition);
        __m256i hvd0      = _mm256_blendv_epi8(hv0, d0, dirCondition);
        __m256i strength1 = _mm256_cmpgt_epi32(hvd1, _mm256_add_epi32(hvd0, hvd0));
        __m256i strength2 = _mm256_cmpgt_epi32(_mm256_add_epi32(hvd1, hvd1), _mm256_add_epi32(hvd0, _mm256_slli_epi32(hvd0, 3)));
        __m256i offset    = _mm256_and_si256(strength1, _mm256_set1_epi32(1));
        __m256i direction = _mm256_add_epi32(offset, _mm256_and_si256(strength2, _mm256_set1_epi32(1)));
        direction         = _mm256_add_epi32(direction, _mm256_andnot_si256(dirCondition, _mm256_set1_epi32(2)));
        direction         = _mm256_and_si256(direction, strength1);
        classIdx          = _mm256_add_epi32(direction, classIdx);

        // transpose
        __m256i dirTempHVMinus1 = _mm256_cmpgt_epi32(sumV, sumH);
        __m256i dirTempDMinus1  = _mm256_cmpgt_epi32(sumD0, sumD1);
        __m256i transposeIdx    = _mm256_set1_epi32(3);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempHVMinus1);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempDMinus1);
        transposeIdx            = _mm256_add_epi32(transposeIdx, dirTempDMinus1);
        classIdx                = _mm256_slli_epi16(classIdx, 2);
        classIdx                = _mm256_add_epi16(classIdx, transposeIdx);
        classIdx = _mm256_shuffle_epi8(classIdx, _mm256_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13, 0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        _mm256_storeu_si256((__m256i *) &classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
        _mm256_storeu_si256((__m256i *) &classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
#else
        _mm256_storeu_si256((__m256i *) &classifier[curBlk.pos().y + i][curBlk.pos().x + j], classIdx);
        _mm256_storeu_si256((__m256i *) &classifier[curBlk.pos().y + i + 1][curBlk.pos().x + j], classIdx);
#endif
      }   // for (int j = 0; j < curBlk.width; j += 16)
    }     // for (int i = 0; i < curBlk.height; i += 2)
  }
  else
  {
#endif
  const __m128i mult = _mm_set1_epi32(multTab[dirWindSize]);
  const __m128i scale = _mm_set1_epi32(15);

  for (int i = 0; i < curBlk.height; i += 2)
  {
    int iOffset = i >> 1;
    for (int j = 0; j < curBlk.width; j += 8)
    {
      int jOffset = j >> 1;
      __m128i sumV = _mm_loadu_si128((const __m128i *) &laplacian[VER][iOffset][jOffset]);  //4 32-bit values
      __m128i sumH = _mm_loadu_si128((const __m128i *) &laplacian[HOR][iOffset][jOffset]);
      __m128i sumD0 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG0][iOffset][jOffset]);
      __m128i sumD1 = _mm_loadu_si128((const __m128i *) &laplacian[DIAG1][iOffset][jOffset]);

      //sum += sumV + sumH;
      __m128i tempAct = _mm_add_epi32(sumV, sumH);
      __m128i activity = _mm_mullo_epi32(tempAct, mult);
      activity = _mm_srl_epi32(activity, shift);
      activity = _mm_min_epi32(activity, scale);
      __m128i classIdx = _mm_shuffle_epi8(_mm_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);
      classIdx = _mm_add_epi32(classIdx, _mm_slli_epi32(classIdx, 2));  // activity * 5

      __m128i hv1 = _mm_max_epi32(sumV, sumH);
      __m128i hv0 = _mm_min_epi32(sumV, sumH);

      __m128i d1 = _mm_max_epi32(sumD0, sumD1);
      __m128i d0 = _mm_min_epi32(sumD0, sumD1);

      const __m128i hv1Xd0e = _mm_mul_epi32(hv1, d0);
      const __m128i hv0Xd1e = _mm_mul_epi32(hv0, d1);
      const __m128i hv1Xd0o = _mm_mul_epi32(_mm_srli_si128(hv1, 4), _mm_srli_si128(d0, 4));
      const __m128i hv0Xd1o = _mm_mul_epi32(_mm_srli_si128(hv0, 4), _mm_srli_si128(d1, 4));

      const __m128i xmme = _mm_sub_epi64(hv1Xd0e, hv0Xd1e);
      const __m128i xmmo = _mm_sub_epi64(hv1Xd0o, hv0Xd1o);

      __m128i dirCondition = _mm_srai_epi32(_mm_blend_epi16(_mm_srli_si128(xmme, 4), xmmo, 0xCC), 31);

      __m128i hvd1 = _mm_blendv_epi8(hv1, d1, dirCondition);
      __m128i hvd0 = _mm_blendv_epi8(hv0, d0, dirCondition);
      __m128i strength1 = _mm_cmpgt_epi32(hvd1, _mm_add_epi32(hvd0, hvd0));
      __m128i strength2 = _mm_cmpgt_epi32(_mm_add_epi32(hvd1, hvd1), _mm_add_epi32(hvd0, _mm_slli_epi32(hvd0, 3)));
      __m128i offset = _mm_and_si128(strength1, _mm_set1_epi32(1));
      __m128i direction = _mm_add_epi32(offset, _mm_and_si128(strength2, _mm_set1_epi32(1)));
      direction = _mm_add_epi32(direction, _mm_andnot_si128(dirCondition, _mm_set1_epi32(2)));      
      direction = _mm_and_si128(direction, strength1);      
      classIdx = _mm_add_epi32(direction, classIdx);

      //transpose
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i dirTempDMinus1 = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i transposeIdx = _mm_set1_epi32(3);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      classIdx = _mm_slli_epi16(classIdx, 2);
      classIdx = _mm_add_epi16(classIdx, transposeIdx);
      classIdx = _mm_shuffle_epi8(classIdx, _mm_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      _mm_storeu_si128((__m128i *) &classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *) &classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
#else
      _mm_storeu_si128((__m128i *) &classifier[curBlk.pos().y + i][curBlk.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *) &classifier[curBlk.pos().y + i + 1][curBlk.pos().x + j], classIdx);
#endif
    }//for (int j = 0; j < curBlk.width; j += 8)
  }//for (int i = 0; i < curBlk.height; i += 2)
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  }//Use 256 Bit Simd
#endif
}
#endif
#if JVET_AK0065_TALF && USE_AVX2
static void simdSetBiInput(Pel input[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const CodingStructure& cs, const ComponentID compId
  , const CPelBuf& recBuf, const Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF], const Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF]
  , const Position curPos, const int shapeIdx, const int picWidth, const int picHeight, const int mode, std::vector<refComb>& refCombs
  , MvField* mvField, const int numOfClips)
{
  bool isMv = isMvTAlf(mode);
  const CPelBuf& refBufA = isMv ? cs.slice->getRefPic(REF_PIC_LIST_0, mvField[0].refIdx)->unscaledPic->getRecoBuf(compId)
    : cs.slice->getRefPic(refCombs[0].rplId, refCombs[0].refId)->unscaledPic->getRecoBuf(compId);
  const CPelBuf& refBufB = isMv ? cs.slice->getRefPic(REF_PIC_LIST_1, mvField[1].refIdx)->unscaledPic->getRecoBuf(compId)
    : cs.slice->getRefPic(refCombs[1].rplId, refCombs[1].refId)->unscaledPic->getRecoBuf(compId);
  const Position posOffsetA(mvField[0].mv.getHor(), mvField[0].mv.getVer());
  const Position posOffsetB(mvField[1].mv.getHor(), mvField[1].mv.getVer());
  const Position sbbPosA(curPos.offset(posOffsetA));
  const Position sbbPosB(curPos.offset(posOffsetB));
  const int refAStride = refBufA.stride;
  const int refBStride = refBufB.stride;
  const Position* relPos = shapeIdx > 0 ? templateShape1 : templateShape0;
  Pel buf[TALF_SBB_SIZE * TALF_SBB_SIZE];

  // 1. save rec data.
  const Pel* rec = recBuf.bufAt(curPos);
  const int recStride = recBuf.stride;
  Pel* recData = buf;
  for(int y = 0; y < TALF_SBB_SIZE; y++)
  {
    memcpy(recData, rec, sizeof(Pel) * TALF_SBB_SIZE);
    rec += recStride;
    recData += TALF_SBB_SIZE;
  }
  __m256i sub = _mm256_loadu_si256((const __m256i *) buf );
  if (cs.pcv->isEncoder)
  {
    for (int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
    {
      _mm256_storeu_si256((__m256i *) input[clipIdx][NUM_TALF_COEFF][0], sub);
    }
  }

  // 2. save center refDataA, refDataB
  int cIdx = 0;
  const Pel* refACenter = refBufA.bufAt(sbbPosA);
  Pel* refDataACenter = buf;
  for(int y = 0; y < TALF_SBB_SIZE; y++)
  {
    memcpy(refDataACenter, refACenter, sizeof(Pel) * TALF_SBB_SIZE);
    refACenter += refAStride;
    refDataACenter += TALF_SBB_SIZE;
  }
  __m256i refCentermmA = _mm256_loadu_si256((const __m256i *) buf );

  const Pel* refBCenter = refBufB.bufAt(sbbPosB);
  Pel* refDataBCenter = buf;
  for(int y = 0; y < TALF_SBB_SIZE; y++)
  {
    memcpy(refDataBCenter, refBCenter, sizeof(Pel) * TALF_SBB_SIZE);
    refBCenter += refBStride;
    refDataBCenter += TALF_SBB_SIZE;
  }
  __m256i refCentermmB = _mm256_loadu_si256((const __m256i *) buf );

  __m256i refCentermm = _mm256_srai_epi16(_mm256_add_epi16(refCentermmA, refCentermmB), 1);
  refCentermm = _mm256_sub_epi16(refCentermm, sub);

  for(int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
  {
    __m256i clipMinVal = _mm256_set1_epi16(clipMin[clipIdx][cIdx]);
    __m256i clipMaxVal = _mm256_set1_epi16(clipMax[clipIdx][cIdx]);
    __m256i clipCenter = _mm256_max_epi16(_mm256_min_epi16(refCentermm, clipMaxVal), clipMinVal);
    _mm256_storeu_si256((__m256i *) input[clipIdx][cIdx][0], clipCenter);
  }

  // 3. save paired refData
  for (cIdx = 1; cIdx < NUM_TALF_COEFF; cIdx++)
  {
    const Pel* refUpperA = refBufA.bufAt(sbbPosA.offset(relPos[cIdx]));
    Pel* refDataA = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refDataA, refUpperA, sizeof(Pel) * TALF_SBB_SIZE);
      refUpperA += refAStride;
      refDataA  += TALF_SBB_SIZE;
    }
    __m256i refUppermmA = _mm256_loadu_si256((const __m256i *) buf );

    const Pel* refLowerA = refBufA.bufAt(sbbPosA - relPos[cIdx]);
    refDataA = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refDataA, refLowerA, sizeof(Pel) * TALF_SBB_SIZE);
      refLowerA += refAStride;
      refDataA  += TALF_SBB_SIZE;
    }
    __m256i refLowermmA = _mm256_loadu_si256((const __m256i *) buf );

    const Pel* refUpperB = refBufB.bufAt(sbbPosB.offset(relPos[cIdx]));
    Pel* refDataB = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refDataB, refUpperB, sizeof(Pel) * TALF_SBB_SIZE);
      refUpperB += refAStride;
      refDataB  += TALF_SBB_SIZE;
    }
    __m256i refUppermmB = _mm256_loadu_si256((const __m256i *) buf );

    const Pel* refLowerB = refBufB.bufAt(sbbPosB - relPos[cIdx]);
    refDataB = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refDataB, refLowerB, sizeof(Pel) * TALF_SBB_SIZE);
      refLowerB += refBStride;
      refDataB  += TALF_SBB_SIZE;
    }
    __m256i refLowermmB = _mm256_loadu_si256((const __m256i *) buf );

    __m256i refUppermm = _mm256_srai_epi16(_mm256_add_epi16(refUppermmA, refUppermmB), 1);
    __m256i refLowermm = _mm256_srai_epi16(_mm256_add_epi16(refLowermmA, refLowermmB), 1);
    refUppermm = _mm256_sub_epi16(refUppermm, sub);
    refLowermm = _mm256_sub_epi16(refLowermm, sub);

    for(int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
    {
      __m256i clipMinVal = _mm256_set1_epi16(clipMin[clipIdx][cIdx]);
      __m256i clipMaxVal = _mm256_set1_epi16(clipMax[clipIdx][cIdx]);
      __m256i clipUpper  = _mm256_max_epi16(_mm256_min_epi16(refUppermm, clipMaxVal), clipMinVal);
      __m256i clipLower  = _mm256_max_epi16(_mm256_min_epi16(refLowermm, clipMaxVal), clipMinVal);
      __m256i clipPaired = _mm256_add_epi16(clipUpper, clipLower);
      _mm256_storeu_si256((__m256i *) input[clipIdx][cIdx][0], clipPaired);
    }
  }
}
static void simdSetUniInput(Pel input[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const CodingStructure& cs, const ComponentID compId
  , const CPelBuf& recBuf, const Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF], const Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF]
  , const Position curPos, const int shapeIdx, const int picWidth, const int picHeight, const int mode, std::vector<refComb>& refCombs
  , MvField* mvField, const int numOfClips)
{
  bool refList = isFwdTAlf(mode) ? 0 : 1;
  bool isMv = isMvTAlf(mode);
  const CPelBuf& refBuf = isMv ? cs.slice->getRefPic(RefPicList(refList), mvField[refList].refIdx)->unscaledPic->getRecoBuf(compId)
    : cs.slice->getRefPic(refCombs[refList].rplId, refCombs[refList].refId)->unscaledPic->getRecoBuf(compId);
  const int refStride = refBuf.stride;
  const Position posOffset(mvField[refList].mv.getHor(), mvField[refList].mv.getVer());
  const Position sbbPos(curPos.offset(posOffset));
  const Position* relPos = shapeIdx > 0 ? templateShape1 : templateShape0;
  Pel buf[TALF_SBB_SIZE * TALF_SBB_SIZE];

  // 1. save rec data.
  const Pel* rec = recBuf.bufAt(curPos);
  const int recStride = recBuf.stride;
  Pel* recData = buf;
  for(int y = 0; y < TALF_SBB_SIZE; y++)
  {
    memcpy(recData, rec, sizeof(Pel) * TALF_SBB_SIZE);
    rec += recStride;
    recData += TALF_SBB_SIZE;
  }
  __m256i sub = _mm256_loadu_si256((const __m256i *) buf );
  if (cs.pcv->isEncoder)
  {
    for (int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
    {
      _mm256_storeu_si256((__m256i *) input[clipIdx][NUM_TALF_COEFF][0], sub);
    }
  }

  // 2. save center refData
  int cIdx = 0;
  const Pel* refCenter = refBuf.bufAt(sbbPos.offset(relPos[cIdx]));
  Pel* refDataCenter = buf;
  for(int y = 0; y < TALF_SBB_SIZE; y++)
  {
    memcpy(refDataCenter, refCenter, sizeof(Pel) * TALF_SBB_SIZE);
    refCenter += refStride;
    refDataCenter += TALF_SBB_SIZE;
  }
  __m256i refCentermm = _mm256_loadu_si256((const __m256i *) buf );
  refCentermm = _mm256_sub_epi16(refCentermm, sub);
  for(int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
  {
    __m256i clipMinVal = _mm256_set1_epi16(clipMin[clipIdx][cIdx]);
    __m256i clipMaxVal = _mm256_set1_epi16(clipMax[clipIdx][cIdx]);
    __m256i clipCenter = _mm256_max_epi16(_mm256_min_epi16(refCentermm, clipMaxVal), clipMinVal);
    _mm256_storeu_si256((__m256i *) input[clipIdx][cIdx][0], clipCenter);
  }

  // 3. save paired refData
  for (cIdx = 1; cIdx < NUM_TALF_COEFF; cIdx++)
  {
    const Pel* refUpper = refBuf.bufAt(sbbPos.offset(relPos[cIdx]));
    Pel* refData = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refData, refUpper, sizeof(Pel) * TALF_SBB_SIZE);
      refUpper += refStride;
      refData  += TALF_SBB_SIZE;
    }
    __m256i refUppermm = _mm256_loadu_si256((const __m256i *) buf );
    refUppermm = _mm256_sub_epi16(refUppermm, sub);

    const Pel* refLower = refBuf.bufAt(sbbPos - relPos[cIdx]);
    refData = buf;
    for(int y = 0; y < TALF_SBB_SIZE; y++)
    {
      memcpy(refData, refLower, sizeof(Pel) * TALF_SBB_SIZE);
      refLower += refStride;
      refData  += TALF_SBB_SIZE;
    }
    __m256i refLowermm = _mm256_loadu_si256((const __m256i *) buf );
    refLowermm = _mm256_sub_epi16(refLowermm, sub);
    for(int clipIdx = 0; clipIdx < numOfClips; clipIdx++)
    {
      __m256i clipMinVal = _mm256_set1_epi16(clipMin[clipIdx][cIdx]);
      __m256i clipMaxVal = _mm256_set1_epi16(clipMax[clipIdx][cIdx]);
      __m256i clipUpper = _mm256_max_epi16(_mm256_min_epi16(refUppermm, clipMaxVal), clipMinVal);
      __m256i clipLower = _mm256_max_epi16(_mm256_min_epi16(refLowermm, clipMaxVal), clipMinVal);
      __m256i clipPaired = _mm256_add_epi16(clipUpper, clipLower);
      _mm256_storeu_si256((__m256i *) input[clipIdx][cIdx][0], clipPaired);
    }
  }
}
static int simdGroupSumTAlf(Pel* a, Pel* b)
{
  __m256i vsum32 = _mm256_setzero_si256();
  __m256i vsrc1 = _mm256_lddqu_si256((__m256i*)a);
  __m256i vsrc2 = _mm256_lddqu_si256((__m256i*)b);
  __m256i vsumtemp = _mm256_madd_epi16(vsrc1, vsrc2);
  vsum32 = _mm256_add_epi32(vsum32, vsumtemp);
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
  return (_mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11))));
}
static void simdFilterBatchTAlf(Pel inputBatch[NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const int *filterCoeff,const Position pos
  , PelBuf &dstBuf, PelBuf &recBuf, const int numCoeff, const int offset, const int shift, const ClpRng& clpRng)
{
  __m256i toAdd0 = _mm256_setzero_si256();
  __m256i toAdd1 = _mm256_setzero_si256();
  __m256i offSet = _mm256_set1_epi32(offset);

  for(int i = 0; i < numCoeff; i++)
  {
    __m256i coef = _mm256_set1_epi32(filterCoeff[i]);
    __m256i input0 = _mm256_cvtepi16_epi32(_mm_loadu_si128((__m128i const*)inputBatch[i][0]));
    __m256i input1 = _mm256_cvtepi16_epi32(_mm_loadu_si128((__m128i const*)inputBatch[i][2]));
    toAdd0 = _mm256_add_epi32(toAdd0, _mm256_mullo_epi32(coef, input0));
    toAdd1 = _mm256_add_epi32(toAdd1, _mm256_mullo_epi32(coef, input1));
  }
  __m256i absToAdd0 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_abs_epi32(toAdd0), offSet), shift);
  __m256i absToAdd1 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_abs_epi32(toAdd1), offSet), shift);
  toAdd0 = _mm256_sign_epi32(absToAdd0, toAdd0);
  toAdd1 = _mm256_sign_epi32(absToAdd1, toAdd1);
  int talfCorr[TALF_SBB_SIZE][TALF_SBB_SIZE];
  _mm256_storeu_si256((__m256i*)talfCorr[0], toAdd0);
  _mm256_storeu_si256((__m256i*)talfCorr[2], toAdd1);
  Pel* dst = dstBuf.bufAt(pos);
  const int stride = dstBuf.stride;
  for (int y = 0; y < TALF_SBB_SIZE; y++)
  {
    for (int x = 0; x < TALF_SBB_SIZE; x++)
    {
#if JVET_AG0145_ADAPTIVE_CLIPPING
      dst[x] += talfCorr[y][x];
#else
      dst[x] = ClipPel( dst[x] + talfCorr[y][x], clpRng );
#endif
    }
    dst += stride;
  }
}
#endif

#if ENABLE_SIMD_OPT_ALF_CHOLESKY
template<X86_VEXT vext>
static int simdFastCholeskyDec(AdaptiveLoopFilter::cholesky_matrix inpMatr, AdaptiveLoopFilter::cholesky_matrix outMatr, int numEq)
{
  for (int i = 0; i < numEq; i++)
  {
    AdaptiveLoopFilter::cholesky_float_t scale = inpMatr[i][i];

#if JVET_AK0123_ALF_COEFF_RESTRICTION
    for (int k = 0; k < i; k++)
#else
    for (int k = i - 1; k >= 0; k--)
#endif
    {
      scale -= outMatr[k][i] * outMatr[k][i];
    }

    if (scale <= AdaptiveLoopFilter::cholesky_reg_sqr) // inpMatr is singular
    {
      return 0;
    }

    outMatr[i][i] = sqrt(scale);
    AdaptiveLoopFilter::cholesky_float_t tmp = (AdaptiveLoopFilter::cholesky_float_t)1 / outMatr[i][i];

    int j = i + 1;
    for (; j + 4 < numEq; j += 4)
    {
      __m128 scale = _mm_loadu_ps(inpMatr[i] + j);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      for (int k = 0; k < i; k++)
#else
      for (int k = i - 1; k >= 0; k--)
#endif
      {
        __m128 outMatrJ = _mm_loadu_ps(outMatr[k] + j);
        __m128 outMatrI = _mm_set1_ps(outMatr[k][i]);
        scale = _mm_sub_ps(scale, _mm_mul_ps(outMatrJ, outMatrI));
      }
      scale = _mm_mul_ps(scale, _mm_set1_ps(tmp));
      _mm_storeu_ps(outMatr[i] + j, scale); // Upper triangular
    }

    for (; j < numEq; j++)
    {
      scale = inpMatr[i][j];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      for (int k = 0; k < i; k++)
#else
      for (int k = i - 1; k >= 0; k--)
#endif
      {
        scale -= outMatr[k][j] * outMatr[k][i];
      }

      outMatr[i][j] = scale * tmp; // Upper triangular
    }
  }

  return 1; // Signal that Cholesky factorization is successfully performed
}
#endif

template <X86_VEXT vext>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86()
{
  m_filter5x5Blk = simdFilter5x5Blk<vext>;
  m_filter7x7Blk = simdFilter7x7Blk<vext>;
#if ALF_IMPROVEMENT
#if FIXFILTER_CFG
  m_filter9x9BlkNoFix = simdFilter9x9BlkNoFix<vext>;
#endif
  m_filter9x9Blk = simdFilter9x9Blk<vext>;
  m_filter9x9BlkExt = simdFilter9x9BlkExt<vext>;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
  m_filter13x13BlkExtDb = simdFilter13x13BlkExtDb<vext>;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filter13x13BlkExtDbResiDirect = simdFilter13x13BlkExtDbResiDirect<vext>;
  m_filter13x13BlkExtDbResi       = simdFilter13x13BlkExtDbResi<vext>;
#if FIXFILTER_CFG
  m_filter13x13BlkDbResiDirect = simdFilter13x13BlkDbResiDirect<vext>;
  m_filter13x13BlkDbResi       = simdFilter13x13BlkDbResi<vext>;
#endif
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  m_filter9x9BlkExtDb = simdFilter9x9BlkExtDb<vext>;
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
  m_filter13x13BlkExt = simdFilter13x13BlkExt<vext>;
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_fixFilter9x9Db9Blk = simdFixFilter9x9Db9Blk<vext>;
  m_fixFilter13x13Db9Blk = simdFixFilter13x13Db9Blk<vext>;
#else
  m_filter13x13Blk = simdFilter13x13Blk<vext>;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_filterResi9x9Blk = simdFilterResi9x9Db9Blk<vext>;
#else
  m_filterResi13x13Blk = simdFilterResi13x13Blk<vext>;
#endif
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  m_gaussFiltering = simdGaussFiltering<vext>;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  m_laplacianFiltering = simdLaplacianFiltering<vext>;
#endif
  m_deriveClassificationLaplacian = simdDeriveClassificationLaplacian;
  m_deriveClassificationLaplacianBig = simdDeriveClassificationLaplacianBig;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_deriveVariance = simdDeriveVariance<vext>;
  m_calcClass0 = simdCalcClass0<vext>;
#else
  m_calcClass0 = simdCalcClass0;
#endif
#if USE_AVX2 && JVET_AJ0188_CODING_INFO_CLASSIFICATION
  m_calcClass1 = simdCalcClass1<vext>;
#else
  m_calcClass1 = simdCalcClass1;
#endif

  for( int i = 0; i < NUM_SETS_FIXED_FILTERS; i++ )
  {
    for( int j = 0; j < NUM_CLASSIFIER; j++ )
    {
      for( int k = 0; k < NUM_FIXED_FILTERS; k++ )
      {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        if( j == 0 )
        {
          for (int m = 0; m < FIX_FILTER_NUM_COEFF_9_DB_9; m++)
          {
            packedDataFixedFilters9Db9[i][k][m] = (m_filterCoeffFixed9Db9[i][k][p0Filter9Db9[m]] << 2) | m_clippingFixed9Db9[i][k][p0Filter9Db9[m]];
          }
          for (int m = 0; m < 20; m++)
          {
            int combineIdx = p0Filter9Db9Combine[m];
            if (combineIdx == -1)
            {
              packedDataFixedFilters9Db9Combine[i][k][m] = packedDataFixedFilters9Db9[i][k][m];
            }
            else
            {
              packedDataFixedFilters9Db9Combine[i][k][m] = ((m_filterCoeffFixed9Db9[i][k][p0Filter9Db9[m]] + m_filterCoeffFixed9Db9[i][k][combineIdx]) << 2) | ((m_clippingFixed9Db9[i][k][p0Filter9Db9[m]] + m_clippingFixed9Db9[i][k][combineIdx] + 1) >> 1);
            }
          }
          for (int m = 20; m < FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9; m++)
          {
            int combineIdx = p0Filter9Db9Combine[m];
            packedDataFixedFilters9Db9Combine[i][k][m] = (m_filterCoeffFixed9Db9[i][k][combineIdx] << 2) | m_clippingFixed9Db9[i][k][combineIdx];
          }
        }
        else
        {
          for( int m = 0; m < FIX_FILTER_NUM_COEFF_13_DB_9; m++ )
          {
            packedDataFixedFilters13Db9[i][k][m] = ( m_filterCoeffFixed13Db9[i][k][p0[m]] << 2 ) | m_clippingFixed13Db9[i][k][p0[m]];
          }
        }
#else
        for( int m = 0; m < 42; m++ )
        {
          packedDataFixedFilters[i][j][k][m] = ( m_filterCoeffFixed[i][j][k][p0[m]] << 2 ) | m_clippingFixed[i][j][k][p0[m]];
        }
#endif
      }
    }
  }
#else
  m_deriveClassificationBlk = simdDeriveClassificationBlk<vext>;
#endif
#if JVET_AK0065_TALF && USE_AVX2
  if (vext >= AVX2)
  {
    m_setTAlfInput[1] = simdSetBiInput;
    m_setTAlfInput[0] = simdSetUniInput;
    m_groupSumTAlf    = simdGroupSumTAlf;
    m_filterBatchTAlf = simdFilterBatchTAlf;
  }
#endif
#if ENABLE_SIMD_OPT_ALF_CHOLESKY
  m_fastCholeskyDec = simdFastCholeskyDec<vext>;
#endif
}

template void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86<SIMDX86>();
#endif   // TARGET_SIMD_X86
