/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
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
 * \brief Implementation of IbcHashMap class
 */

#include "CommonDefX86.h"
#include "../IntraPrediction.h"

#ifdef TARGET_SIMD_X86

#include <nmmintrin.h>

#if ENABLE_SIMD_TMP
#if JVET_AD0086_ENHANCED_INTRA_TMP
// Calculation with step of 4
#if JVET_AG0136_INTRA_TMP_LIC
template<X86_VEXT vext>
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
inline int calcDiff4(const short* pSrc1, const short* pSrc2, const int start, const int end)
#else
inline uint32_t calcDiff4(const short* pSrc1, const short* pSrc2, const int start, const int end)
#endif
{
#if JVET_AG0136_INTRA_TMP_LIC
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])));
  const __m128i vzero = _mm_setzero_si128();
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 4)
  {
    __m128i vsrc1 = _mm_loadl_epi64((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_loadl_epi64((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
#endif
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
template<X86_VEXT vext>
inline int calcDiffExtraction4(const short* pSrc1, const short* pSrc2, const int start, const int end, int& last)
{
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(_mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e)), _mm_shuffle_epi32(vsum32, 0xb1));
  last = _mm_extract_epi16(vsum16, 3);
  return _mm_cvtsi128_si32(vsum32);
}
#endif
// Calculation with step of 8
#if JVET_AG0136_INTRA_TMP_LIC
template<X86_VEXT vext>
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
inline int calcDiff8(const short* pSrc1, const short* pSrc2, const int start, const int end)
#else
inline uint32_t calcDiff8(const short* pSrc1, const short* pSrc2, const int start, const int end)
#endif
{
#if JVET_AG0136_INTRA_TMP_LIC && USE_AVX2
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start])));
  const __m128i vzero = _mm_setzero_si128();
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    __m128i vsrc1 = _mm_loadu_si128((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
#if JVET_AG0136_INTRA_TMP_LIC
#if USE_AVX2
template<X86_VEXT vext>
inline int calcDiff16(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    vsum16 = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2)));
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
inline int summation4(const short* pSrc, const int start, const int end)
{
  const __m128i vsum16 = _mm_loadl_epi64((const __m128i*) &pSrc[start]);
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

template<X86_VEXT vext>
inline int summation8(const short* pSrc, const int start, const int end)
{
#if USE_AVX2
  const __m128i vsum16 = _mm_loadu_si128((const __m128i*) &pSrc[start]);
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc = _mm_loadu_si128((const __m128i*) &pSrc[iX]);
    vsum16 = _mm_add_epi16(vsum16, vsrc);
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

#if USE_AVX2
template<X86_VEXT vext>
inline int summation16(const short* pSrc, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc = _mm256_loadu_si256((const __m256i*) &pSrc[iX]);
    vsum16 = _mm256_add_epi16(vsum16, vsrc);
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
inline void calcDiffDelta4Joint(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& sad, int& mrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                , const int licShift
#endif
                                )
{
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const __m128i referenceVec = _mm_loadl_epi64((const __m128i*) &pSrc2[start]);
  const __m128i difference = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), referenceVec);
  const __m128i differenceLic = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start + licShift]), referenceVec);
#else
  const __m128i difference = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start]));
#endif
  const __m128i vsumSad16 = _mm_abs_epi16(difference);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(differenceLic, delta));
#else
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(difference, delta));
#endif
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsumSad32 = _mm_unpacklo_epi16(vsumSad16, vzero);
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0xb1));
  __m128i vsumMrsad32 = _mm_unpacklo_epi16(vsumMrsad16, vzero);
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm_cvtsi128_si32(vsumSad32);
  mrsad = _mm_cvtsi128_si32(vsumMrsad32);
}

template<X86_VEXT vext>
inline int calcDiffDelta4(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end)
{
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])), delta));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
template<X86_VEXT vext>
inline void calcDiffDeltaExtraction4Joint(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& sad, int& mrsad, const int licShift, int& lastSad, int& lastMrsad)
{
  const __m128i referenceVec = _mm_loadl_epi64((const __m128i*) &pSrc2[start]);
  const __m128i difference = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), referenceVec);
  const __m128i differenceLic = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start + licShift]), referenceVec);
  const __m128i vsumSad16 = _mm_abs_epi16(difference);
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(differenceLic, delta));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsumSad32 = _mm_unpacklo_epi16(vsumSad16, vzero);
  vsumSad32 = _mm_add_epi32(_mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0x4e)), _mm_shuffle_epi32(vsumSad32, 0xb1));
  __m128i vsumMrsad32 = _mm_unpacklo_epi16(vsumMrsad16, vzero);
  vsumMrsad32 = _mm_add_epi32(_mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0x4e)), _mm_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm_cvtsi128_si32(vsumSad32);
  mrsad = _mm_cvtsi128_si32(vsumMrsad32);
  lastSad = _mm_extract_epi16(vsumSad16, 3);
  lastMrsad = _mm_extract_epi16(vsumMrsad16, 3);
}

template<X86_VEXT vext>
inline int calcDiffExtractionDelta4(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& last)
{
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])), delta));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(_mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e)), _mm_shuffle_epi32(vsum32, 0xb1));
  last = _mm_extract_epi16(vsum16, 3);
  return _mm_cvtsi128_si32(vsum32);
}
#endif

template<X86_VEXT vext>
inline void calcDiffDelta8Joint(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& sad, int& mrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                , const int licShift
#endif
                                )
{
#if USE_AVX2
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const __m128i referenceVec = _mm_lddqu_si128((const __m128i*) &pSrc2[start]);
  const __m128i difference = _mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), referenceVec);
  const __m128i differenceLic = _mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start + licShift]), referenceVec);
#else
  const __m128i difference = _mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start]));
#endif
  const __m128i vsumSad16 = _mm_abs_epi16(difference);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(differenceLic, delta));
#else
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(difference, delta));
#endif
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsumSad16 = vzero;
  __m128i vsumMrsad16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc1 = _mm_loadu_si128((const __m128i*) &pSrc1[iX]);
    const __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) &pSrc2[iX]);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const __m128i vsrcLic1 = _mm_loadu_si128((const __m128i*) & pSrc1[iX + licShift]);
    const __m128i differenceLic = _mm_sub_epi16(vsrcLic1, vsrc2);
#endif
    const __m128i difference = _mm_sub_epi16(vsrc1, vsrc2);
    vsumSad16 = _mm_add_epi16(vsumSad16, _mm_abs_epi16(difference));
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    vsumMrsad16 = _mm_add_epi16(vsumMrsad16, _mm_abs_epi16(_mm_sub_epi16(differenceLic, delta)));
#else
    vsumMrsad16 = _mm_add_epi16(vsumMrsad16, _mm_abs_epi16(_mm_sub_epi16(difference, delta)));
#endif
  }
#endif
  __m128i vsumSad32 = _mm_add_epi32(_mm_unpacklo_epi16(vsumSad16, vzero), _mm_unpackhi_epi16(vsumSad16, vzero));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0xb1));
  __m128i vsumMrsad32 = _mm_add_epi32(_mm_unpacklo_epi16(vsumMrsad16, vzero), _mm_unpackhi_epi16(vsumMrsad16, vzero));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm_cvtsi128_si32(vsumSad32);
  mrsad = _mm_cvtsi128_si32(vsumMrsad32);
}

template<X86_VEXT vext>
inline int calcDiffDelta8(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end)
{
#if USE_AVX2
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start])), delta));
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc1 = _mm_loadu_si128((const __m128i*) &pSrc1[iX]);
    const __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) &pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(vsrc1, vsrc2), delta)));
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

#if USE_AVX2
template<X86_VEXT vext>
inline void calcDiffDelta16Joint(const short* pSrc1, const short* pSrc2, const __m256i delta, const int start, const int end, int& sad, int& mrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                 , const int licShift
#endif
                                 )
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsumSad16 = vzero;
  __m256i vsumMrsad16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const __m256i vsrcLic1 = _mm256_loadu_si256((const __m256i*) & pSrc1[iX + licShift]);
#endif
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    const __m256i difference = _mm256_sub_epi16(vsrc1, vsrc2);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const __m256i differenceLic = _mm256_sub_epi16(vsrcLic1, vsrc2);
#endif
    vsumSad16 = _mm256_add_epi16(vsumSad16, _mm256_abs_epi16(difference));
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    vsumMrsad16 = _mm256_add_epi16(vsumMrsad16, _mm256_abs_epi16(_mm256_sub_epi16(differenceLic, delta)));
#else
    vsumMrsad16 = _mm256_add_epi16(vsumMrsad16, _mm256_abs_epi16(_mm256_sub_epi16(difference, delta)));
#endif
  }
  __m256i vsumSad32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsumSad16, vzero), _mm256_unpackhi_epi16(vsumSad16, vzero));
  vsumSad32 = _mm256_add_epi32(vsumSad32, _mm256_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm256_add_epi32(vsumSad32, _mm256_shuffle_epi32(vsumSad32, 0xb1));
  __m256i vsumMrsad32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsumMrsad16, vzero), _mm256_unpackhi_epi16(vsumMrsad16, vzero));
  vsumMrsad32 = _mm256_add_epi32(vsumMrsad32, _mm256_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm256_add_epi32(vsumMrsad32, _mm256_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm256_extract_epi32(vsumSad32, 0) + _mm256_extract_epi32(vsumSad32, 4);
  mrsad = _mm256_extract_epi32(vsumMrsad32, 0) + _mm256_extract_epi32(vsumMrsad32, 4);
}

template<X86_VEXT vext>
inline int calcDiffDelta16(const short* pSrc1, const short* pSrc2, const __m256i delta, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    vsum16 = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(_mm256_sub_epi16(vsrc1, vsrc2), delta)));
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
void calcTargetMeanSIMD(Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean)
#else
void calcTargetMeanSIMD(Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean)
#endif
{
  topTargetMean = 0;
  leftTargetMean = 0;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const Pel* tmpBuf = tarPatch;
#endif
  if (tempType == L_SHAPE_TEMPLATE)
  {
    if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 1)
    {
#if  USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          topTargetMean += summation16<vext>((const short*)tmpBuf, TMP_TEMPLATE_SIZE, uiPatchWidth);
          tmpBuf += tarStride;
#else
          topTargetMean += summation16<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          topTargetMean += summation8<vext>((const short*)tmpBuf, TMP_TEMPLATE_SIZE, uiPatchWidth);
          tmpBuf += tarStride;
#else
          topTargetMean += summation8<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          topTargetMean += summation4<vext>((const short*)tmpBuf, TMP_TEMPLATE_SIZE, uiPatchWidth);
          tmpBuf += tarStride;
#else
          topTargetMean += summation4<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
        }
      }
      topTargetMean >>= log2SizeTop;
    }
    if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 2)
    {
      for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        leftTargetMean += summation4<vext>((const short*)tmpBuf, 0, TMP_TEMPLATE_SIZE);
        tmpBuf += tarStride;
#else
        leftTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, TMP_TEMPLATE_SIZE);
#endif
      }
      leftTargetMean >>= log2SizeLeft;
    }
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        topTargetMean += summation16<vext>((const short*)tmpBuf, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        tmpBuf += tarStride;
#else
        topTargetMean += summation16<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
#endif
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        topTargetMean += summation8<vext>((const short*)tmpBuf, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        tmpBuf += tarStride;
#else
        topTargetMean += summation8<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
#endif
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        topTargetMean += summation4<vext>((const short*)tmpBuf, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        tmpBuf += tarStride;
#else
        topTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
#endif
      }
    }
    topTargetMean >>= log2SizeTop;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      leftTargetMean += summation4<vext>((const short*)tmpBuf, 0, TMP_TEMPLATE_SIZE);
      tmpBuf += tarStride;
#else
      leftTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, TMP_TEMPLATE_SIZE);
#endif
    }
    leftTargetMean >>= log2SizeLeft;
  }
}

template<X86_VEXT vext>
inline __m128i calcMeanRefLeftSIMD(const Pel* const ref, const unsigned int uiPatchHeight, const unsigned int uiStride, const int log2SizeLeft, const int leftTargetMean)
{
  int leftMeanRef = 0;
  const Pel* refPatchRowTemp = ref - TMP_TEMPLATE_SIZE;
  for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRowTemp += uiStride)
  {
    leftMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, TMP_TEMPLATE_SIZE);
  }
  leftMeanRef >>= log2SizeLeft;
  return _mm_set1_epi16(leftMeanRef - leftTargetMean);
}

template<X86_VEXT vext>
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
void calcTemplateDiffJointSadMrsadSIMD(const Pel* const ref, const unsigned int uiStride, Pel* tarPatch, int tarStride, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean, const int licShift)
#else
void calcTemplateDiffJointSadMrsadSIMD(const Pel* const ref, const unsigned int uiStride, Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean)
#endif
{
  int diffSumSad = 0;
  int diffSumMrsad = 0;
  int topDiffSad = MAX_INT;
  int topDiffMrsad = MAX_INT;
  int leftDiffSad = MAX_INT;
  int leftDiffMrsad = MAX_INT;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const Pel* const refLic = ref + licShift;
  const Pel* tmpBuf = tarPatch;
#endif
  const Pel* refPatchRow = tempType == L_SHAPE_TEMPLATE ? ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE : (tempType == ABOVE_TEMPLATE ? ref - TMP_TEMPLATE_SIZE * uiStride : ref - TMP_TEMPLATE_SIZE);
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  const Pel* refPatchRowLic = tempType == L_SHAPE_TEMPLATE ? refLic - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE : (tempType == ABOVE_TEMPLATE ? refLic - TMP_TEMPLATE_SIZE * uiStride : refLic - TMP_TEMPLATE_SIZE);
#endif
  int topMeanRef = 0;
  __m128i topMeanDelta = _mm_setzero_si128();
#if USE_AVX2
  __m256i topMeanDelta256 = _mm256_setzero_si256();
#endif
  if (tempType == L_SHAPE_TEMPLATE)
  {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const Pel* refPatchRowTemp = refPatchRowLic;
#else
    const Pel* refPatchRowTemp = refPatchRow;
#endif
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation16<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation8<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation4<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    topMeanRef >>= log2SizeTop;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const Pel* refPatchRowTemp = refPatchRowLic;
#else
    const Pel* refPatchRowTemp = refPatchRow;
#endif
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation16<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation8<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    topMeanRef >>= log2SizeTop;
  }
  if (tempType == L_SHAPE_TEMPLATE || tempType == ABOVE_TEMPLATE)
  {
    topMeanDelta = _mm_set1_epi16(topMeanRef - topTargetMean);
#if USE_AVX2
    topMeanDelta256 = _mm256_set1_epi16(topMeanRef - topTargetMean);
#endif
  }
  int iSumSad = 0;
  int iSumMrsad = 0;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    topDiffSad = 0;
    topDiffMrsad = 0;
    leftDiffSad = 0;
    leftDiffMrsad = 0;
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        const short* const pSrc2 = (const short*)tmpBuf;
        tmpBuf += tarStride;
#else
        const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                  , licShift
#endif
                                  );
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta16Joint<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                   , licShift
#endif
                                   );
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        const short* const pSrc2 = (const short*)tmpBuf;
        tmpBuf += tarStride;
#else
        const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                  , licShift
#endif
                                  );
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta8Joint<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                  , licShift
#endif
                                  );
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        const short* const pSrc2 = (const short*)tmpBuf;
        tmpBuf += tarStride;
#else
        const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                  , licShift
#endif
                                  );
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
                                  , licShift
#endif
                                  );
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
#endif
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(refLic, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#else
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif	
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    int lastSad;
    int lastMrsad;
#endif
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
    {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      calcDiffDeltaExtraction4Joint<vext>((const short*) refPatchRow, (const short*) tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad, licShift, lastSad, lastMrsad);
#else
      calcDiffDelta4Joint<vext>((const short*)refPatchRow, (const short*)tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad, licShift);
#endif
#else
      calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      leftDiffSad += iSumSad + lastSad;
      leftDiffMrsad += iSumMrsad + lastMrsad;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      const short* pSrc2 = tmpBuf;
#endif
      tmpBuf += tarStride;
#else
      const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      iSumSad +=
        (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1]) * ((1 <<TMP_TEMPLATE_COST_SHIFT)-1));
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      iSumMrsad += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1 + licShift] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal) * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#else
      iSumMrsad += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                    * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      diffSumSad += iSumSad + (lastSad << TMP_TEMPLATE_COST_SHIFT);
      diffSumMrsad += iSumMrsad + (lastMrsad << TMP_TEMPLATE_COST_SHIFT);
#else
      diffSumSad += iSumSad;
      diffSumMrsad += iSumMrsad;
      leftDiffSad += iSumSad;
      leftDiffMrsad += iSumMrsad;
#endif
      if (diffSumSad > iMaxSad[0] && leftDiffSad > iMaxSad[2] && diffSumMrsad > iMaxMrsad[0] && leftDiffMrsad > iMaxMrsad[2])
      {
        break;
      }
    }
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
    if (vext >= AVX2 && (iCols & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        calcDiffDelta16Joint<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta256, 0, iCols, iSumSad, iSumMrsad, licShift);
        tmpBuf += tarStride;
#else
        calcDiffDelta16Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, 0, iCols, iSumSad, iSumMrsad);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
    else if ((iCols & 7) == 0)
#else
    if ((iCols & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        calcDiffDelta8Joint<vext>((const short*)refPatchRow, (const short*)tmpBuf, topMeanDelta, 0, iCols, iSumSad, iSumMrsad, licShift);
        tmpBuf += tarStride;
#else
        calcDiffDelta8Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols, iSumSad, iSumMrsad);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        calcDiffDelta4Joint<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta, 0, iCols, iSumSad, iSumMrsad, licShift);
        tmpBuf += tarStride;
#else
        calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols, iSumSad, iSumMrsad);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          iSumSad <<= TMP_TEMPLATE_COST_SHIFT;
          iSumMrsad <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(refLic, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#else
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif	
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    int lastSad;
    int lastMrsad;
#endif
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
    {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      calcDiffDeltaExtraction4Joint<vext>((const short*) refPatchRow, (const short*) tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad, licShift, lastSad, lastMrsad);
#else
      calcDiffDelta4Joint<vext>((const short*)refPatchRow, (const short*)tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad, licShift);
#endif
#else
      calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      const short* pSrc2 = tmpBuf;
#endif
      tmpBuf += tarStride;
#else
      const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      iSumSad += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1])
                  * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      iSumMrsad += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1 + licShift] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal) * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#else
      iSumMrsad += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                    * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      diffSumSad += iSumSad + (lastSad << TMP_TEMPLATE_COST_SHIFT);
      diffSumMrsad += iSumMrsad + (lastMrsad << TMP_TEMPLATE_COST_SHIFT);
#else
      diffSumSad += iSumSad;
      diffSumMrsad += iSumMrsad;
#endif
      if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
      {
        break;
      }
    }
  }
  diffSad[0] = diffSumSad;
  diffSad[1] = topDiffSad;
  diffSad[2] = leftDiffSad;
  diffMrsad[0] = diffSumMrsad;
  diffMrsad[1] = topDiffMrsad;
  diffMrsad[2] = leftDiffMrsad;
}
#endif
template<X86_VEXT vext>
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
void calcTemplateDiffSIMD(Pel* ref, unsigned int uiStride, Pel* tarPatch, int tarStride, unsigned int uiPatchWidth,
  unsigned int uiPatchHeight, int* diff, int* iMax, RefTemplateType tempType,
  int requiredTemplate
#if JVET_AG0136_INTRA_TMP_LIC
  , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
)
#else
void calcTemplateDiffSIMD(Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                          unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType tempType,
                          int requiredTemplate
#if JVET_AG0136_INTRA_TMP_LIC
                          , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
                          )
#endif
{
  int diffSum  = 0;
  int topDiff  = MAX_INT;
  int leftDiff = MAX_INT;
  int iY;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
  Pel* tmpBuf = tarPatch;
#endif
#if JVET_W0069_TMP_BOUNDARY
  Pel *refPatchRow;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel *refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel *    tarPatchRow;
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  int uiSum;
#else
  uint32_t uiSum;
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  if (isMrSad)
  {
    __m128i topMeanDelta = _mm_setzero_si128();
#if USE_AVX2
    __m256i topMeanDelta256 = _mm256_setzero_si256();
#endif
    int topMeanRef = 0;
    if (tempType == L_SHAPE_TEMPLATE)
    {
      if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 1)
      {
        const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation16<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation8<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation4<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        topMeanRef >>= log2SizeTop;
      }
    }
    else if (tempType == ABOVE_TEMPLATE)
    {
      const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation16<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation8<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      topMeanRef >>= log2SizeTop;
    }
    if ((tempType == L_SHAPE_TEMPLATE && requiredTemplate != 2) || tempType == ABOVE_TEMPLATE)
    {
      topMeanDelta = _mm_set1_epi16(topMeanRef - topTargetMean);
#if USE_AVX2
      topMeanDelta256 = _mm256_set1_epi16(topMeanRef - topTargetMean);
#endif
    }
    if (tempType == L_SHAPE_TEMPLATE)
    {
      topDiff = 0;
      leftDiff = 0;
      if (requiredTemplate == 3)
      {
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            const short* const pSrc2 = (const short*)tmpBuf;
            tmpBuf += tarStride;
#else
            const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
            uiSum = calcDiffDelta16<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
#endif
            diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            const short* const pSrc2 = (const short*)tmpBuf;
            tmpBuf += tarStride;
#else
            const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
            uiSum = calcDiffDelta8<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
#endif
            diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            const short* const pSrc2 = (const short*)tmpBuf;
            tmpBuf += tarStride;
#else
            const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
#endif
            diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += uiSum;
#endif
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif
        refPatchRow = ref - TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        int last;
#endif
        for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          uiSum = calcDiffExtractionDelta4<vext>((const short*) refPatchRow, (const short*) tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, last);
#else
          uiSum = calcDiffDelta4<vext>((const short*)refPatchRow, (const short*)tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#else
          uiSum = calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          leftDiff += uiSum + last;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          const short* pSrc2 = tmpBuf;
#endif
          tmpBuf += tarStride;
#else
          const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          uiSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                    * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          diffSum += uiSum + (last << TMP_TEMPLATE_COST_SHIFT);
#else
          diffSum += uiSum;
          leftDiff += uiSum;
#endif
          if (diffSum > iMax[0] && leftDiff > iMax[2])
          {
            break;
          }
        }
      }
      else if (requiredTemplate == 0)
      {
        bool isAboveLeftSkipped = false;
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRow;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            const short* const pSrc2 = tmpBuf;
            tmpBuf += tarStride;
#else
            const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
#else
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#endif
            uiSum = calcDiffDelta16<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            diffSum += calcDiffDelta16<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRow;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            const short* const pSrc2 = tmpBuf;
            tmpBuf += tarStride;
#else
            const short* const pSrc2 = (const short*) tarPatch[iY];
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
#else
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#endif
            uiSum = calcDiffDelta8<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            diffSum += calcDiffDelta8<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, tmpBuf, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            uiSum = calcDiffDelta4<vext>((const short*) refPatchRow, tmpBuf, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            tmpBuf += tarStride;
#else
            uiSum = calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, uiPatchWidth);
#endif
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            diffSum += uiSum;
#else
            diffSum += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, uiPatchWidth);
#endif
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        if (!isAboveLeftSkipped)
        {
          const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif
          refPatchRow = ref - TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          int last;
#endif
          for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
          {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            diffSum += calcDiffExtractionDelta4<vext>((const short*) refPatchRow, tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, last);
#else
            diffSum += calcDiffDelta4<vext>((const short*)refPatchRow, tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#else
            diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            const short* pSrc2 = tmpBuf;
#endif
            tmpBuf += tarStride;
#else
            const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            diffSum += last << TMP_TEMPLATE_COST_SHIFT;
#else
            diffSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                        * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
            if (diffSum > iMax[0])
            {
              break;
            }
          }
        }
      }
      else if (requiredTemplate == 1)
      {
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += calcDiffDelta16<vext>((const short*) refPatchRow, tmpBuf, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
            uiSum = calcDiffDelta16<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            tmpBuf += tarStride;
#else
            uiSum = calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            topDiff += uiSum;
#endif
#else
            topDiff += calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += calcDiffDelta8<vext>((const short*) refPatchRow, tmpBuf, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
            uiSum = calcDiffDelta8<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            tmpBuf += tarStride;
#else
            uiSum = calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            topDiff += uiSum;
#endif
#else
            topDiff += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            topDiff += calcDiffDelta4<vext>((const short*) refPatchRow, tmpBuf, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
            uiSum = calcDiffDelta4<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            tmpBuf += tarStride;
#else
            uiSum = calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            if (iY == (TMP_TEMPLATE_SIZE-1)) 
            {
              uiSum <<= TMP_TEMPLATE_COST_SHIFT;
            }
            topDiff += uiSum;
#endif
#else
            topDiff += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
      }
      else
      {
        const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif
        refPatchRow = ref - TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif
        for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          leftDiff += calcDiffDelta4<vext>((const short*) refPatchRow, tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#else
          leftDiff += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          const short* pSrc2 = tmpBuf;
#endif
          tmpBuf += tarStride;
#else
          const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          leftDiff += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                       * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
          if (leftDiff > iMax[2])
          {
            break;
          }
        }
      }
    }
    else if (tempType == ABOVE_TEMPLATE)
    {
      const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
      if (vext >= AVX2 && (iCols & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiffDelta16<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta256, 0, iCols);
          tmpBuf += tarStride;
#else
          uiSum = calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, 0, iCols);
#endif
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, 0, iCols);
#endif
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
      else if ((iCols & 7) == 0)
#else
      if ((iCols & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiffDelta8<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta, 0, iCols);
          tmpBuf += tarStride;
#else
          uiSum = calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
#endif
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
#endif
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiffDelta4<vext>((const short*)refPatchRow, tmpBuf, topMeanDelta, 0, iCols);
          tmpBuf += tarStride;
#else
          uiSum = calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
#endif
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
#endif
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
    }
    else if (tempType == LEFT_TEMPLATE)
    {
      const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      const short leftMeanDeltaVal = (short)_mm_cvtsi128_si32(leftMeanDelta);
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      int last;
#endif
      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        diffSum += calcDiffExtractionDelta4<vext>((const short*) refPatchRow, tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE, last);
#else
        diffSum += calcDiffDelta4<vext>((const short*)refPatchRow, tmpBuf, leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#else
        diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        const short *pSrc1 = (const short *) refPatchRow;
#endif
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        const short* pSrc2 = tmpBuf;
#endif
        tmpBuf += tarStride;
#else
        const short *pSrc2 = (const short *) tarPatch[iY];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        diffSum += last << TMP_TEMPLATE_COST_SHIFT;
#else
        diffSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1] - leftMeanDeltaVal)
                    * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif
#endif
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
  }
  else
  {
#endif

  // horizontal difference
#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif
    topDiff  = 0;
    leftDiff = 0;
    if (requiredTemplate == 3)
    {
#if JVET_AG0136_INTRA_TMP_LIC 
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          const short* pSrc1 = tmpBuf;
          tmpBuf += tarStride;
#else
          const short* pSrc1 = (const short*) tarPatch[iY];
#endif
          const short* pSrc2 = (const short*) refPatchRow;
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#endif
          uiSum = calcDiff16<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
#endif
          diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          const short* pSrc1 = tmpBuf;
          tmpBuf += tarStride;
#else
          const short* pSrc1 = (const short*) tarPatch[iY];
#endif
          const short* pSrc2 = (const short*) refPatchRow;
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#endif
          uiSum = calcDiff8<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
#endif
          diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          const short* pSrc1 = tmpBuf;
          tmpBuf += tarStride;
#else
          const short* pSrc1 = (const short*) tarPatch[iY];
#endif
          const short* pSrc2 = (const short*) refPatchRow;
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#endif
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
#endif
          diffSum += uiSum;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += uiSum;
#endif
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
#else
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        int iCols = uiPatchWidth;

        // TopLeft
        uiSum = calcDiff4(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE-1)) 
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSum += uiSum;

        if (((iCols - TMP_TEMPLATE_SIZE) & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSum += uiSum;
        topDiff += uiSum;

        if (diffSum > iMax[0] && topDiff > iMax[1])
        {
          break;
        }

        // update location
        refPatchRow += uiStride;
      }
#endif
      // TopDiff = top_diff;
      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif

#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      int last;
#endif
      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        tarPatchRow = tmpBuf;
        tmpBuf += tarStride;
#else
        tarPatchRow        = tarPatch[iY];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        uiSum = calcDiffExtraction4<vext>((const short*) tarPatchRow, (const short*) refPatchRow, 0, iCols, last);
        leftDiff += uiSum + last;
        diffSum += uiSum + (last << TMP_TEMPLATE_COST_SHIFT);
#else
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        uiSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1])
                  * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));

        diffSum += uiSum;
        leftDiff += uiSum;
#endif
#endif

        if (diffSum > iMax[0] && leftDiff > iMax[2])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }
    else if (requiredTemplate == 0)
    {
#if JVET_AG0136_INTRA_TMP_LIC 
      bool isAboveLeftSkipped = false;
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          const short* const pSrc1 = (const short*)tmpBuf;
          tmpBuf += tarStride;
#else
          const short* pSrc1 = (const short*) tarPatch[iY];
#endif
          const short* pSrc2 = (const short*) refPatchRow;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#else
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#endif
          uiSum = calcDiff16<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          diffSum += calcDiff16<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          const short* const pSrc1 = (const short*)tmpBuf;
          tmpBuf += tarStride;
#else
          const short* pSrc1 = (const short*) tarPatch[iY];
#endif
          const short* pSrc2 = (const short*) refPatchRow;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
#else
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#endif
          uiSum = calcDiff8<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          diffSum += calcDiff8<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
          diffSum += calcDiff4<vext>(tmpBuf, (const short*)refPatchRow, 0, TMP_TEMPLATE_SIZE);
          uiSum = calcDiff4<vext>(tmpBuf, (const short*)refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
          tmpBuf += tarStride;
#else
          uiSum = calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, uiPatchWidth);
#endif
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          diffSum += uiSum;
#else
          diffSum += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, uiPatchWidth);
#endif
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      if (!isAboveLeftSkipped)
      {
#else
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        // int  iRows = uiPatchHeight;
        int iCols = uiPatchWidth;
        if ((iCols & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, 0, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);
        }
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        diffSum += uiSum;

        if (diffSum > iMax[0])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
#endif

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif

#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      int last;
#endif
      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        tarPatchRow = tmpBuf;
        tmpBuf += tarStride;
#else
        tarPatchRow        = tarPatch[iY];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        uiSum = calcDiffExtraction4<vext>((const short*) tarPatchRow, (const short*) refPatchRow, 0, iCols, last);
        diffSum += uiSum + (last << TMP_TEMPLATE_COST_SHIFT);
#else
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        uiSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1])
                  * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));

        diffSum += uiSum;
#endif
#endif

        if (diffSum > iMax[0])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
#if JVET_AG0136_INTRA_TMP_LIC 
      }
#endif
    }
    else if (requiredTemplate == 1)
    {
#if JVET_AG0136_INTRA_TMP_LIC 
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += calcDiff16<vext>(tmpBuf, (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
          uiSum = calcDiff16<vext>(tmpBuf, (const short*)refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          tmpBuf += tarStride;
#else
          uiSum = calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          topDiff += uiSum;
#endif
#else
          topDiff += calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += calcDiff8<vext>(tmpBuf, (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
          uiSum = calcDiff8<vext>(tmpBuf, (const short*)refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          tmpBuf += tarStride;
#else
          uiSum = calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          topDiff += uiSum;
#endif
#else
          topDiff += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          topDiff += calcDiff4<vext>(tmpBuf, (const short*)refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#else
          uiSum = calcDiff4<vext>(tmpBuf, (const short*)refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          tmpBuf += tarStride;
#else
          uiSum = calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          if (iY == (TMP_TEMPLATE_SIZE-1)) 
          {
            uiSum <<= TMP_TEMPLATE_COST_SHIFT;
          }
          topDiff += uiSum;
#endif
#else
          topDiff += calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
#endif
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
#else
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        int iCols = uiPatchWidth;

        if (((iCols - TMP_TEMPLATE_SIZE) & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if (iY == (TMP_TEMPLATE_SIZE - 1))
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
#endif
        topDiff += uiSum;

        if (topDiff > iMax[1])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
#endif
    }
    else   // L
    {
      refPatchRow = ref - TMP_TEMPLATE_SIZE;
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      tmpBuf = tarPatch + TMP_TEMPLATE_SIZE * tarStride;
#endif

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;

      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        tarPatchRow = tmpBuf;
        tmpBuf += tarStride;
#else
        tarPatchRow        = tarPatch[iY];
#endif
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference

#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        leftDiff += calcDiff4
#else
        uiSum = calcDiff4
#endif
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        uiSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1])
                  * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));
#endif

        leftDiff += uiSum;
#endif

        if (leftDiff > iMax[2])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }

#if JVET_W0069_TMP_BOUNDARY
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
#if JVET_AG0136_INTRA_TMP_LIC
    const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
    if (vext >= AVX2 && (iCols & 15) == 0)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        uiSum = calcDiff16<vext>(tmpBuf, (const short*)refPatchRow, 0, iCols);
        tmpBuf += tarStride;
#else
        uiSum = calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (iY == (TMP_TEMPLATE_SIZE-1)) 
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
        diffSum += uiSum;
#else
        diffSum += calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
    else if ((iCols & 7) == 0)
#else
    if ((iCols & 7) == 0)
#endif
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        uiSum = calcDiff8<vext>(tmpBuf, (const short*)refPatchRow, 0, iCols);
        tmpBuf += tarStride;
#else
        uiSum = calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (iY == (TMP_TEMPLATE_SIZE-1)) 
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
        diffSum += uiSum;
#else
        diffSum += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
        uiSum = calcDiff4<vext>(tmpBuf, (const short*)refPatchRow, 0, iCols);
        tmpBuf += tarStride;
#else
        uiSum = calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (iY == (TMP_TEMPLATE_SIZE-1)) 
        {
          uiSum <<= TMP_TEMPLATE_COST_SHIFT;
        }
        diffSum += uiSum;
#else
        diffSum += calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
#endif
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
#else
    // horizontal difference
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
      if ((iCols & 7) == 0)
      {
        uiSum = calcDiff8(pSrc1, pSrc2, 0, iCols);
      }
      else
      {
        uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);
      }
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      if (iY == (TMP_TEMPLATE_SIZE - 1))
      {
        uiSum <<= TMP_TEMPLATE_COST_SHIFT;
      }
#endif
      diffSum += uiSum;

      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }

      // update location
      refPatchRow += uiStride;
    }
#endif
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
   int last;
#endif
    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
      tarPatchRow = tmpBuf;
      tmpBuf += tarStride;
#else
      tarPatchRow        = tarPatch[iY];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      uiSum = calcDiffExtraction4<vext>((const short*) tarPatchRow, (const short*) refPatchRow, 0, iCols, last);
      diffSum += uiSum + (last << TMP_TEMPLATE_COST_SHIFT);
#else
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                        <vext>
#endif
                        (pSrc1, pSrc2, 0, iCols);
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      uiSum += (abs(pSrc1[TMP_TEMPLATE_SIZE - 1] - pSrc2[TMP_TEMPLATE_SIZE - 1])
                * ((1 << TMP_TEMPLATE_COST_SHIFT) - 1));

      diffSum += uiSum;
#endif
#endif

      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }

      refPatchRow += uiStride;
    }
  }
#endif

#if JVET_AG0136_INTRA_TMP_LIC
  }
#endif
  diff[0] = diffSum;
  diff[1] = topDiff;
  diff[2] = leftDiff;
}
#else
template< X86_VEXT vext >
#if JVET_W0069_TMP_BOUNDARY
int calcTemplateDiffSIMD(Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType tempType)
#else
int calcTemplateDiffSIMD( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax )
#endif
{
  int diffSum = 0;
  int iY;
#if JVET_W0069_TMP_BOUNDARY
  Pel* refPatchRow;
  if( tempType == L_SHAPE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == LEFT_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == ABOVE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel* refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel* tarPatchRow;
  uint32_t uiSum;

  // horizontal difference
#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth;
      if ((iCols & 7) == 0)
      {
        // Do with step of 8
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      else
      {
        // Do with step of 4
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 4)
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }

    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      // Do with step of 4
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      __m128i vsum16 = vzero;
      for (int iX = 0; iX < iCols; iX += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum            = _mm_cvtsi128_si32(vsum32);

      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
#if JVET_W0069_TMP_BOUNDARY
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    // horizontal difference
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
      if ((iCols & 7) == 0)
      {
        // Do with step of 8
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      else
      {
        // Do with step of 4
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 4)
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      // Do with step of 4
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      __m128i vsum16 = vzero;
      for (int iX = 0; iX < iCols; iX += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum            = _mm_cvtsi128_si32(vsum32);

      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
  }
#endif

  return diffSum;
}
#endif 
#endif

#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void dimdBlendingSIMD( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height )
{
  CHECK( (width % 4) != 0, "width should be multiple of 4" );
  __m128i vw0 = _mm_set1_epi32( w0 );
  __m128i vw1 = _mm_set1_epi32( w1 );
  __m128i vw2 = _mm_set1_epi32( w2 );
  const int shift = 6;

  for( int i = 0; i < height; i++ )
  {
    for( int j = 0; j < width; j += 4 )
    {
      __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
      __m128i vsrc0 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc0 + j) ) );
      __m128i vsrc1 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc1 + j) ) );

      vdst = _mm_mullo_epi32( vdst, vw0 );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc0, vw1 ) );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc1, vw2 ) );

      vdst = _mm_srai_epi32( vdst, shift );
      vdst = _mm_packs_epi32( vdst, vdst );
      _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
    }
    pDst += strideDst;
    pSrc0 += strideSrc0;
    pSrc1 += strideSrc1;
  }
}
#endif

#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void timdBlendingSIMD( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height )
{
  CHECK( (width % 4) != 0, "width should be multiple of 4" );
#if JVET_AL0108_BVG_DIMD && USE_AVX2
  if ((vext >= AVX2) && (width & 0x7) == 0)
  {
    const int shift = 6;
    __m256i mw = _mm256_unpacklo_epi16(_mm256_set1_epi16(w0), _mm256_set1_epi16(w1));
    __m256i msrc0, msrc1, msum0, msum1;

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        msrc0 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pDst[col])));
        msrc1 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pSrc[col])));
        msum0 = _mm256_unpacklo_epi16(msrc0, msrc1);
        msum1 = _mm256_unpackhi_epi16(msrc0, msrc1);
        msum0 = _mm256_madd_epi16(msum0, mw);
        msum1 = _mm256_madd_epi16(msum1, mw);
        msum0 = _mm256_srai_epi32(msum0, shift);
        msum1 = _mm256_srai_epi32(msum1, shift);
        msum0 = _mm256_packs_epi32(msum0, msum1);
        _mm_storeu_si128((__m128i *)&pDst[col], _mm256_castsi256_si128(msum0));
      }
      pDst += strideDst;
      pSrc += strideSrc;
    }
  }
  else
  {
#endif  
  __m128i vw0 = _mm_set1_epi32( w0 );
  __m128i vw1 = _mm_set1_epi32( w1 );
  const int shift = 6;

  for( int i = 0; i < height; i++ )
  {
    for( int j = 0; j < width; j += 4 )
    {
      __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
      __m128i vsrc = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc + j) ) );

      vdst = _mm_mullo_epi32( vdst, vw0 );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc, vw1 ) );

      vdst = _mm_srai_epi32( vdst, shift );
      vdst = _mm_packs_epi32( vdst, vdst );
      _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
    }
    pDst += strideDst;
    pSrc += strideSrc;
  }
#if JVET_AL0108_BVG_DIMD && USE_AVX2
  }
#endif
}
#endif

#if JVET_AC0112_IBC_CIIP && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void ibcCiipBlendingSIMD( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height )
{
#if USE_AVX2
  if ((vext >= AVX2) && (width & 0x7) == 0)
  {
    const int offset = 1 << (shift - 1);
    __m256i mw = _mm256_unpacklo_epi16(_mm256_set1_epi16(w0), _mm256_set1_epi16(w1));
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i msrc0, msrc1, msum0, msum1;

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        msrc0 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pSrc0[col])));
        msrc1 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pSrc1[col])));
        msum0 = _mm256_unpacklo_epi16(msrc0, msrc1);
        msum1 = _mm256_unpackhi_epi16(msrc0, msrc1);
        msum0 = _mm256_madd_epi16(msum0, mw);
        msum1 = _mm256_madd_epi16(msum1, mw);
        msum0 = _mm256_add_epi32(msum0, voffset);
        msum1 = _mm256_add_epi32(msum1, voffset);
        msum0 = _mm256_srai_epi32(msum0, shift);
        msum1 = _mm256_srai_epi32(msum1, shift);
        msum0 = _mm256_packs_epi32(msum0, msum1);
        _mm_storeu_si128((__m128i *)&pDst[col], _mm256_castsi256_si128(msum0));
      }
      pSrc0 += strideSrc0;
      pSrc1 += strideSrc1;
      pDst += strideDst;
    }
  }
  else
  {
#endif
    __m128i vw0 = _mm_set1_epi32( w0 );
    __m128i vw1 = _mm_set1_epi32( w1 );
    const int offset = 1 << (shift - 1);
    __m128i voffset  = _mm_set1_epi32( offset );

    for( int i = 0; i < height; i++ )
    {
      for( int j = 0; j < width; j += 4 )
      {
        __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
        __m128i vsrc0 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc0 + j) ) );
        __m128i vsrc1 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc1 + j) ) );

        vdst = _mm_mullo_epi32( vsrc0, vw0 );
        vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc1, vw1 ) );

        vdst = _mm_add_epi32( vdst, voffset );
        vdst = _mm_srai_epi32( vdst, shift );
        vdst = _mm_packs_epi32( vdst, vdst );
        _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
      }
      pDst += strideDst;
      pSrc0 += strideSrc0;
      pSrc1 += strideSrc1;
    }
#if USE_AVX2
  }
#endif
}
#endif

#if JVET_AH0209_PDP
template< X86_VEXT vext >
bool xPredIntraOpt_SIMD(PelBuf &pDst, const PredictionUnit &pu, const uint32_t modeIdx, const ClpRng& clpRng, Pel* refF, Pel* refS)
{
#if JVET_AK0061_PDP_MPM
  uint32_t width = pu.lumaSize().width;
  uint32_t height = pu.lumaSize().height;
#else
  const uint32_t width = pDst.width;
  const uint32_t height = pDst.height;
#endif 

  const int sizeKey = (width << 8) + height;
  const int sizeIdx = g_size.find( sizeKey ) != g_size.end() ? g_size[sizeKey] : -1;

  if( sizeIdx < 0 )
  {
    return false;
  }
#if JVET_AI0208_PDP_MIP
  if ( !pu.cu->mipFlag && sizeIdx > 12 && modeIdx > 1 && (modeIdx % 4 != 2))
#else
  if (sizeIdx > 12 && modeIdx > 1 && (modeIdx % 4 != 2))
#endif
  {
    return false;
  }

  const uint32_t stride = pDst.stride;
  Pel*           pred = pDst.buf;
  const int xShift = g_sizeData[sizeIdx][8], yShift = g_sizeData[sizeIdx][9];

  bool shortLen = modeIdx < PDP_SHORT_TH[0] || (modeIdx >= PDP_SHORT_TH[1] && modeIdx <= PDP_SHORT_TH[2]);
  const int refLen = shortLen ? g_sizeData[sizeIdx][10] : g_sizeData[sizeIdx][7];
  auto ref = shortLen ? refS : refF;

#if JVET_AI0208_PDP_MIP
  int16_t*** filter;
  if(pu.cu->mipFlag)
  {
    if(pu.mipTransposedFlag)
    {
      filter = g_pdpFiltersMip[modeIdx+16][sizeIdx];
    }
    else
    {
      filter = g_pdpFiltersMip[modeIdx][sizeIdx];
    }
  }
  else
  {
    filter = g_pdpFilters[modeIdx][sizeIdx];
  }
#else
  int16_t*** filter = g_pdpFilters[modeIdx][sizeIdx];
#endif
  const int addShift = 1 << 13;
#if !JVET_AK0061_PDP_MPM
  const __m128i offset = _mm_set1_epi32( addShift );
#if JVET_AJ0237_INTERNAL_12BIT
  const __m128i max = _mm_set1_epi32((1 << clpRng.bd) - 1);
#else
  const __m128i max = _mm_set1_epi32( 1023 );
#endif
  const __m128i zeros = _mm_setzero_si128();
  __m128i vmat[ 4 ], vcoef[ 4 ], vsrc;
#endif
#if JVET_AI0208_PDP_MIP
  bool isHeightLarge = (height == 32) && !pu.cu->mipFlag;
#else
  bool isHeightLarge = (height == 32);
#endif
  int startY = isHeightLarge? 1 : 0;
  int strideOffset = isHeightLarge ? (stride << 1) : stride;
  int offsetY = isHeightLarge ? 2 : 1;
  if (isHeightLarge)
  {
    pred += stride;
  }
#if JVET_AK0061_PDP_MPM && !JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  width = pDst.width;
  height = isHeightLarge ? pu.lumaSize().height : pDst.height;
#endif 
#if JVET_AK0061_PDP_MPM && USE_AVX2
  if (vext >= AVX2 && refLen >= 16) 
  {
    __m256i vmat[4], vcoef[4], vsrc;
    const __m128i offset128 = _mm_set1_epi32(addShift);
#if JVET_AJ0237_INTERNAL_12BIT
    const __m128i max128 = _mm_set1_epi32((1 << clpRng.bd) - 1);
#else
    const __m128i max = _mm_set1_epi32(1023);
#endif
    const __m128i zeros128 = _mm_setzero_si128();
    __m128i vmat128[4], vcoef128[4], vsrc128;
    __m128i result;
    const int length = (refLen >> 4) << 4;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    for (int y = startY; y < pDst.height; y += offsetY, pred += strideOffset)
#else
    for (int y = startY; y < height; y += offsetY, pred += strideOffset) 
#endif
    {
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
      for (int x = 0; x < pDst.width; x += 4)
#else
      for (int x = 0; x < width; x += 4) 
#endif
      {
        const int16_t* f0 = filter[y >> yShift][(x >> xShift) >> 2];
        
        vcoef[0] = _mm256_setzero_si256();
        vcoef[1] = _mm256_setzero_si256();
        vcoef[2] = _mm256_setzero_si256();
        vcoef[3] = _mm256_setzero_si256();
        result = _mm_setzero_si128();
        int i;
        ///< 256 bit process
        for (i = 0; i < length; i += 16)
        {
          vsrc = _mm256_lddqu_si256((const __m256i*) & ref[i]);
          vmat[0] = _mm256_set_m128i(_mm_loadu_si128((const __m128i*)(f0 + 32)), _mm_loadu_si128((const __m128i*)f0));
          vmat[1] = _mm256_set_m128i(_mm_loadu_si128((const __m128i*)(f0 + 40)), _mm_loadu_si128((const __m128i*)(f0 + 8)));
          vmat[2] = _mm256_set_m128i(_mm_loadu_si128((const __m128i*)(f0 + 48)), _mm_loadu_si128((const __m128i*)(f0 + 16)));
          vmat[3] = _mm256_set_m128i(_mm_loadu_si128((const __m128i*)(f0 + 56)), _mm_loadu_si128((const __m128i*)(f0 + 24)));

          vcoef[0] = _mm256_add_epi32(_mm256_madd_epi16(vsrc, vmat[0]), vcoef[0]);
          vcoef[1] = _mm256_add_epi32(_mm256_madd_epi16(vsrc, vmat[1]), vcoef[1]);
          vcoef[2] = _mm256_add_epi32(_mm256_madd_epi16(vsrc, vmat[2]), vcoef[2]);
          vcoef[3] = _mm256_add_epi32(_mm256_madd_epi16(vsrc, vmat[3]), vcoef[3]);

          f0 += 64;
        }
        vcoef[0] = _mm256_hadd_epi32(vcoef[0], vcoef[1]);
        vcoef[2] = _mm256_hadd_epi32(vcoef[2], vcoef[3]);
        vcoef[0] = _mm256_hadd_epi32(vcoef[0], vcoef[2]);
        result = _mm_add_epi32(_mm256_extractf128_si256(vcoef[0], 0), _mm256_extractf128_si256(vcoef[0], 1));
        
        if (length != refLen) 
        {
          f0 = filter[y >> yShift][(x >> xShift) >> 2] + (length >> 4) * 64;
          vcoef128[0] = _mm_setzero_si128();
          vcoef128[1] = _mm_setzero_si128();
          vcoef128[2] = _mm_setzero_si128();
          vcoef128[3] = _mm_setzero_si128();
          for (; i < refLen; i += 8)
          {
            vsrc128 = _mm_loadu_si128((const __m128i*) & ref[i]);

            vmat128[0] = _mm_loadu_si128((const __m128i*)f0);
            vmat128[1] = _mm_loadu_si128((const __m128i*)(f0 + 8));
            vmat128[2] = _mm_loadu_si128((const __m128i*)(f0 + 16));
            vmat128[3] = _mm_loadu_si128((const __m128i*)(f0 + 24));

            vcoef128[0] = _mm_add_epi32(_mm_madd_epi16(vsrc128, vmat128[0]), vcoef128[0]);
            vcoef128[1] = _mm_add_epi32(_mm_madd_epi16(vsrc128, vmat128[1]), vcoef128[1]);
            vcoef128[2] = _mm_add_epi32(_mm_madd_epi16(vsrc128, vmat128[2]), vcoef128[2]);
            vcoef128[3] = _mm_add_epi32(_mm_madd_epi16(vsrc128, vmat128[3]), vcoef128[3]);

            f0 += 32;
          }
          vcoef128[0] = _mm_hadd_epi32(vcoef128[0], vcoef128[1]);
          vcoef128[2] = _mm_hadd_epi32(vcoef128[2], vcoef128[3]);
          vcoef128[0] = _mm_hadd_epi32(vcoef128[0], vcoef128[2]);
          result = _mm_add_epi32(result, vcoef128[0]);
        }
        result = _mm_srai_epi32(_mm_add_epi32(result, offset128), 14);
        result = _mm_min_epi32(result, max128);
        result = _mm_max_epi32(result, zeros128);
        *((int64_t*)&pred[x]) = _mm_cvtsi128_si64(_mm_packs_epi32(result, result));
      }
    }
  }
  else {
#endif
#if JVET_AK0061_PDP_MPM
    const __m128i offset = _mm_set1_epi32(addShift);
#if JVET_AJ0237_INTERNAL_12BIT
    const __m128i max = _mm_set1_epi32((1 << clpRng.bd) - 1);
#else
    const __m128i max = _mm_set1_epi32(1023);
#endif
    const __m128i zeros = _mm_setzero_si128();
    __m128i vmat[4], vcoef[4], vsrc;
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    for (int y = startY; y < pDst.height; y += offsetY, pred += strideOffset)
#else
    for (int y = startY; y < height; y += offsetY, pred += strideOffset)
#endif
    {
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
      for (int x = 0; x < pDst.width; x += 4)
#else
      for (int x = 0; x < width; x += 4)
#endif
    {
      const int16_t* f0 = filter[y >> yShift][(x >> xShift)>>2];

      vcoef[0] = _mm_setzero_si128();
      vcoef[1] = _mm_setzero_si128();
      vcoef[2] = _mm_setzero_si128();
      vcoef[3] = _mm_setzero_si128();
      int i;
      for (i = 0; i < refLen; i+=8)
      {
        vsrc = _mm_loadu_si128( ( const __m128i* )&ref[i] );
        //vsrc = _mm_unpacklo_epi64( vsrc, vsrc);

        vmat[0] = _mm_loadu_si128( ( const __m128i* )f0 );
        vmat[1] = _mm_loadu_si128( ( const __m128i* )(f0+8 ) );
        vmat[2] = _mm_loadu_si128( ( const __m128i* )(f0+16) );
        vmat[3] = _mm_loadu_si128( ( const __m128i* )(f0+24) );

        vcoef[0] = _mm_add_epi32( _mm_madd_epi16( vsrc, vmat[0] ), vcoef[0] );
        vcoef[1] = _mm_add_epi32( _mm_madd_epi16( vsrc, vmat[1] ), vcoef[1] );
        vcoef[2] = _mm_add_epi32( _mm_madd_epi16( vsrc, vmat[2] ), vcoef[2] );
        vcoef[3] = _mm_add_epi32( _mm_madd_epi16( vsrc, vmat[3] ), vcoef[3] );

        f0 += 32;
      }
      vcoef[0] = _mm_hadd_epi32( vcoef[0], vcoef[1] );
      vcoef[2] = _mm_hadd_epi32( vcoef[2], vcoef[3] );

      vcoef[0] = _mm_hadd_epi32( vcoef[0], vcoef[2] );
      //vcoef[2] = _mm_hadd_epi32( vcoef[2], vcoef[3] );
      //vcoef[0] =  _mm_unpacklo_epi64(vcoef[0], vcoef[2]);
      vcoef[0] = _mm_srai_epi32( _mm_add_epi32( vcoef[0], offset ), 14 );
      vcoef[0] = _mm_min_epi32(vcoef[0], max);
      vcoef[0] = _mm_max_epi32(vcoef[0], zeros);
      //_mm_storeu_si128( ( __m128i * )&pred[x], vcoef[0] );
      *((int64_t *)&pred[x]) = _mm_cvtsi128_si64(_mm_packs_epi32(vcoef[0], vcoef[0]));
      //pred[x] = (Pel)Clip3(clpRng.min, clpRng.max, (std::max(0, sum) + addShift) >> 14);

    }
  }
#if JVET_AK0061_PDP_MPM && USE_AVX2
  }
#endif
#if JVET_AI0208_PDP_MIP
  if (sizeIdx > 12 && !pu.cu->mipFlag)
#else
  if (sizeIdx > 12)
#endif
  {
    
    int numMRLLeft = g_sizeData[sizeIdx][5];
    int numMRLTop = g_sizeData[sizeIdx][6];
#if JVET_AK0061_PDP_MPM
    int sampFacHor = pu.lumaSize().width / 16;
    int sampFacVer = pu.lumaSize().height / 16;
    int strideDst = (pu.lumaSize().width << 1) + numMRLLeft; //fetching from g_ref always.
#else
    int sampFacHor = pDst.width / 16;
    int sampFacVer = pDst.height / 16;
    int strideDst = (pDst.width << 1) + numMRLLeft; //fetching from g_ref always.
#endif
    Pel* ptrSrc = refF + ( strideDst * numMRLTop ) + numMRLLeft - 1;
    Pel* ref2 = refF + ( strideDst*( numMRLTop - 1 ) ) + numMRLLeft;

    pred = pDst.buf;
    for( int y = 0; y < pDst.height; y++ )
    {
      if( 0 != ( sampFacHor - 1 ) && ( y%sampFacVer == ( sampFacVer - 1 ) ) )
      {
        pred[0] = ( ptrSrc[0] + pred[1] + 1 ) >> 1;
      }

      for( int x = 1; x < pDst.width; x++ )
      {
        if( ( x%sampFacHor != ( sampFacHor - 1 ) ) && ( y%sampFacVer == ( sampFacVer - 1 ) ) )
        {
          pred[x] = ( pred[x - 1] + pred[x + 1] + 1 ) >> 1;
        }
      }

      pred += stride;
      ptrSrc += numMRLLeft;
    }

    pred = pDst.buf;
    Pel* pred1 = pred + stride;
    if( 0 != ( sampFacVer - 1 ) )
    {
      for( int x = 0; x < pDst.width; x++ )
      {
        pred[x] = ( ref2[x] + pred1[x] + 1 ) >> 1;
      }
    }

    pred += stride;
    Pel* pred0 = pred - stride;
    pred1 = pred + stride;

    for( int y = 1; y < pDst.height; y++ )
    {
      if( ( y%sampFacVer != ( sampFacVer - 1 ) ) )
      {
        for( int x = 0; x < pDst.width; x++ )
        {
          pred[x] = ( pred0[x] + pred1[x] + 1 ) >> 1;
        }
      }

      pred += stride;
      pred0 += stride;
      pred1 += stride;
    }
  }

  return true;
}
#endif

#if JVET_AG0058_EIP && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
int64_t calcAeipGroupSumSIMD(const Pel* src1, const Pel* src2, const int numSamples)
{
  int i = 0;
#if USE_AVX2
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum32 = vzero;
  const int samplesBySIMD = (numSamples >> 4) << 4;
#if JVET_AJ0237_INTERNAL_12BIT
  int64_t sum = 0;
  const int simdSampleBatchCnt = (samplesBySIMD >> 4) >> 2;
  for (int batchIdx = 0; batchIdx < simdSampleBatchCnt; batchIdx++)
  {
    vsum32 = vzero;
    for (; i < ((batchIdx + 1) * 64); i += 16)
    {
      __m256i vsrc1 = _mm256_lddqu_si256((__m256i*)(&src1[i]));
      __m256i vsrc2 = _mm256_lddqu_si256((__m256i*)(&src2[i]));
      __m256i vsumtemp = _mm256_madd_epi16(vsrc1, vsrc2);
      vsum32 = _mm256_add_epi32(vsum32, vsumtemp);
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
    vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
    sum += (_mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11))));
  }
  if (i < samplesBySIMD)
  {
    vsum32 = vzero;
#endif
  for (; i < samplesBySIMD; i += 16)
  {
    __m256i vsrc1 = _mm256_lddqu_si256((__m256i*)(&src1[i]));
    __m256i vsrc2 = _mm256_lddqu_si256((__m256i*)(&src2[i]));
    __m256i vsumtemp = _mm256_madd_epi16(vsrc1, vsrc2);
    vsum32 = _mm256_add_epi32(vsum32, vsumtemp);
  }
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
#if JVET_AJ0237_INTERNAL_12BIT
  sum += (_mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11))));
  }
#else
  int64_t sum = (_mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11))));
#endif
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = vzero;
  const int samplesBySIMD = (numSamples >> 4) << 4;
#if JVET_AJ0237_INTERNAL_12BIT
  int64_t sum = 0;
  const int simdSampleBatchCnt = (samplesBySIMD >> 4) >> 2;
  for (int batchIdx = 0; batchIdx < simdSampleBatchCnt; batchIdx++)
  {
    vsum32 = vzero;
    for (; i < ((batchIdx + 1) * 64); i += 8)
    {
      __m128i vsrc1 = _mm_loadu_si128((__m128i*)(&src1[i]));
      __m128i vsrc2 = _mm_loadu_si128((__m128i*)(&src2[i]));
      __m128i vsumtemp = _mm_madd_epi16(vsrc1, vsrc2);
      vsum32 = _mm_add_epi32(vsum32, vsumtemp);
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
    sum += _mm_cvtsi128_si32(vsum32);
  }
  if (i < samplesBySIMD)
  {
    vsum32 = vzero;
#endif
  for (; i < samplesBySIMD; i += 8)
  {
    __m128i vsrc1 = _mm_loadu_si128((__m128i*)(&src1[i]));
    __m128i vsrc2 = _mm_loadu_si128((__m128i*)(&src2[i]));
    __m128i vsumtemp = _mm_madd_epi16(vsrc1, vsrc2);
    vsum32 = _mm_add_epi32(vsum32, vsumtemp);
  }
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
#if JVET_AJ0237_INTERNAL_12BIT
  sum += _mm_cvtsi128_si32(vsum32);
  }
#else
  int64_t sum = _mm_cvtsi128_si32(vsum32);
#endif
#endif
  for (; i < numSamples; i++)
  {
    sum += src1[i] * src2[i];
  }

  return sum;
}
#endif

template <X86_VEXT vext>
void IntraPrediction::_initIntraX86()
{
#if ENABLE_SIMD_TMP
  m_calcTemplateDiff = calcTemplateDiffSIMD<vext>;
#if JVET_AG0136_INTRA_TMP_LIC
  m_calcTemplateDiffJointSadMrsad = calcTemplateDiffJointSadMrsadSIMD<vext>;
  m_calcTargetMean = calcTargetMeanSIMD<vext>;
#endif
#endif
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
  m_dimdBlending = dimdBlendingSIMD<vext>;
#endif
#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
  m_timdBlending = timdBlendingSIMD<vext>;
#endif
#if JVET_AC0112_IBC_CIIP && INTRA_TRANS_ENC_OPT
  m_ibcCiipBlending = ibcCiipBlendingSIMD<vext>;
#endif
#if JVET_AH0209_PDP
  m_xPredIntraOpt = xPredIntraOpt_SIMD<vext>;
#endif
#if JVET_AG0058_EIP && INTRA_TRANS_ENC_OPT
  m_calcAeipGroupSum = calcAeipGroupSumSIMD<vext>;
#endif
}

template void IntraPrediction::_initIntraX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
