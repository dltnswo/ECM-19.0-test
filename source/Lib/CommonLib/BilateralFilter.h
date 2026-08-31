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
#ifndef BILATERALFILTER_H
#define BILATERALFILTER_H

#include "CommonDef.h"

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER

#include "Unit.h"
#include "Buffer.h"
#ifdef TARGET_SIMD_X86
#include <tmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
#endif
#if JVET_V0094_BILATERAL_FILTER
class BIFCabacEst
{
public:
  virtual ~BIFCabacEst() {};
#if JVET_AG0196_CABAC_RETRAIN
  virtual uint64_t getBits( const ComponentID compID, Slice& slice, const BifParams& htdfParams ) = 0;
#else
  virtual uint64_t getBits( const ComponentID compID, const Slice& slice, const BifParams& htdfParams) = 0;
#endif
};
#endif
class BilateralFilter
{
#if JVET_AK0118_BF_FOR_INTRA_PRED
public:
#else
private:
#endif
  // 128x128 is the max TU size, 4 is the padding for the considered neighborhood, 16 is the AVX buffer size.
  // (128 + 4 + 16) * (128 + 4) = 19536.
  Pel tempblockSIMD[19536];
  Pel tempblockFilteredSIMD[19536];

  Pel *tempblock = (Pel*)tempblockSIMD;
  Pel* tempblockFiltered = (Pel*)tempblockFilteredSIMD;

#if JVET_AJ0237_INTERNAL_12BIT
  void (*m_bilateralFilterDiamond5x5)(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum, int bdShift
#if JVET_AK0118_BF_FOR_INTRA_PRED
    , bool isIntraPredBf
#endif
    );
  static void blockBilateralFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum, int bdShift
#if JVET_AK0118_BF_FOR_INTRA_PRED
    , bool isIntraPredBf
#endif
    );
#else
  void (*m_bilateralFilterDiamond5x5)(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum);
  static void blockBilateralFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum);
#endif

#if JVET_AF0112_BIF_DYNAMIC_SCALING
  int (*m_calcMAD)(int16_t* block, int stride, int width, int height, int whlog2);
  static int calcMAD(int16_t* block, int stride, int width, int height, int whlog2);
#endif

#if JVET_V0094_BILATERAL_FILTER
  char m_wBIF[26][16] = {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, /* 17 */
  { 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, }, /* 18 */
  { 0, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, }, /* 19 */
  { 0, 3, 4, 4, 4, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, }, /* 20 */
  { 0, 4, 5, 5, 5, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, }, /* 21 */
  { 0, 5, 6, 6, 6, 4, 3, 3, 1, 1, 1, 1, 0, 0, 0, -1, }, /* 22 */
  { 0, 6, 7, 7, 7, 5, 4, 4, 2, 2, 1, 1, 1, 0, 0, -1, }, /* 23 */
  { 0, 6, 8, 8, 8, 6, 5, 4, 3, 3, 2, 2, 1, 1, 0, -2, }, /* 24 */
  { 0, 7, 8, 9, 9, 7, 7, 5, 4, 4, 2, 2, 2, 1, 1, -2, }, /* 25 */
  { 0, 7, 9, 10, 10, 8, 8, 5, 5, 5, 3, 3, 2, 2, 1, -3, }, /* 26 */
  { 0, 8, 10, 11, 11, 9, 9, 6, 6, 6, 3, 3, 3, 2, 1, -3, }, /* 27 */
  { 0, 9, 11, 13, 13, 11, 11, 8, 8, 8, 5, 4, 4, 3, 2, -3, }, /* 28 */
  { 0, 10, 12, 14, 14, 13, 13, 10, 9, 9, 6, 5, 5, 4, 3, -4, }, /* 29 */
  { 0, 11, 13, 16, 16, 15, 15, 13, 11, 11, 8, 6, 6, 4, 4, -4, }, /* 30 */
  { 0, 12, 14, 17, 17, 17, 17, 15, 12, 12, 9, 7, 7, 5, 5, -5, }, /* 31 */
  { 0, 13, 15, 19, 19, 19, 19, 17, 14, 14, 11, 8, 8, 6, 6, -5, }, /* 32 */
  { 0, 14, 17, 20, 21, 21, 21, 19, 17, 16, 13, 10, 10, 7, 7, -5, }, /* 33 */
  { 0, 15, 19, 22, 23, 24, 23, 22, 20, 18, 15, 12, 11, 9, 7, -6, }, /* 34 */
  { 0, 17, 20, 23, 26, 26, 26, 24, 23, 20, 18, 15, 13, 10, 8, -6, }, /* 35 */
  { 0, 18, 22, 25, 28, 29, 28, 27, 26, 22, 20, 17, 14, 12, 8, -7, }, /* 36 */
  { 0, 19, 24, 26, 30, 31, 30, 29, 29, 24, 22, 19, 16, 13, 9, -7, }, /* 37 */
  { 0, 20, 26, 29, 32, 33, 32, 31, 31, 27, 24, 21, 17, 14, 11, -8, }, /* 38 */
  { 0, 21, 28, 31, 34, 35, 35, 34, 34, 30, 26, 23, 19, 15, 13, -8, }, /* 39 */
  { 0, 21, 30, 34, 36, 38, 37, 36, 36, 32, 29, 25, 20, 17, 15, -9, }, /* 40 */
  { 0, 22, 32, 36, 38, 40, 40, 39, 39, 35, 31, 27, 22, 18, 17, -9, }, /* 41 */
  { 0, 23, 34, 39, 40, 42, 42, 41, 41, 38, 33, 29, 23, 19, 19, -10, }, /* 42 */
#else
  {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   2,   2,   2,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   3,   3,   3,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   4,   4,   4,   3,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   5,   5,   5,   4,   3,   2,   2,   2,   2,   2,   1,   0,   1,   1,  -1, },
  {  0,   6,   7,   7,   5,   3,   3,   3,   3,   2,   2,   1,   1,   1,   1,  -1, },
  {  0,   6,   8,   8,   5,   4,   3,   3,   3,   3,   3,   2,   1,   2,   2,  -2, },
  {  0,   7,  10,  10,   6,   4,   4,   4,   4,   3,   3,   2,   2,   2,   2,  -2, },
  {  0,   8,  11,  11,   7,   5,   5,   4,   5,   4,   4,   2,   2,   2,   2,  -2, },
  {  0,   8,  12,  13,  10,   8,   8,   6,   6,   6,   5,   3,   3,   3,   3,  -2, },
  {  0,   8,  13,  14,  13,  12,  11,   8,   8,   7,   7,   5,   5,   4,   4,  -2, },
  {  0,   9,  14,  16,  16,  15,  14,  11,   9,   9,   8,   6,   6,   5,   6,  -3, },
  {  0,   9,  15,  17,  19,  19,  17,  13,  11,  10,  10,   8,   8,   6,   7,  -3, },
  {  0,   9,  16,  19,  22,  22,  20,  15,  12,  12,  11,   9,   9,   7,   8,  -3, },
  {  0,  10,  17,  21,  24,  25,  24,  20,  18,  17,  15,  12,  11,   9,   9,  -3, },
  {  0,  10,  18,  23,  26,  28,  28,  25,  23,  22,  18,  14,  13,  11,  11,  -3, },
  {  0,  11,  19,  24,  29,  30,  32,  30,  29,  26,  22,  17,  15,  13,  12,  -3, },
  {  0,  11,  20,  26,  31,  33,  36,  35,  34,  31,  25,  19,  17,  15,  14,  -3, },
  {  0,  12,  21,  28,  33,  36,  40,  40,  40,  36,  29,  22,  19,  17,  15,  -3, },
  {  0,  13,  21,  29,  34,  37,  41,  41,  41,  38,  32,  23,  20,  17,  15,  -3, },
  {  0,  14,  22,  30,  35,  38,  42,  42,  42,  39,  34,  24,  20,  17,  15,  -3, },
  {  0,  15,  22,  31,  35,  39,  42,  42,  43,  41,  37,  25,  21,  17,  15,  -3, },
  {  0,  16,  23,  32,  36,  40,  43,  43,  44,  42,  39,  26,  21,  17,  15,  -3, },
  {  0,  17,  23,  33,  37,  41,  44,  44,  45,  44,  42,  27,  22,  17,  15,  -3, },
#endif
  };
  char m_lut[26][3 * 16];
  char m_distFactor[3] = { 16, 12, 11, };
  char m_tuSizeFactor[2][64] = { 
    { 10,10,10,9,8,6,5,4,10,10,10,9,8,6,5,4,10,10,10,9,8,6,5,4,9,9,9,7,6,5,5,4,8,8,8,6,5,5,5,4,6,6,6,5,5,5,5,4,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4, },
    { 10,10,10,8,6,6,4,0,10,10,10,8,6,6,4,0,10,10,10,8,6,6,4,0,8,8,8,6,5,4,4,0,6,6,6,5,4,4,4,0,6,6,6,4,4,0,0,0,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0, },
  };
  char m_tuMADFactor[2][16] = {
    { 0,0,0,1,2,3,4,5,6,6,6,6,7,7,7,7, },
    { 0,0,0,0,0,0,1,2,2,4,4,4,4,5,5,5, },
  };
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  char m_wBIFChroma[26][16] = {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, }, /* 17 */
  { 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, }, /* 18 */
  { 0, 2, 3, 3, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 0, }, /* 19 */
  { 0, 3, 5, 5, 6, 5, 5, 5, 5, 5, 4, 2, 2, 2, 2, 1, }, /* 20 */
  { 0, 4, 6, 6, 8, 6, 6, 6, 6, 6, 6, 3, 3, 3, 3, 1, }, /* 21 */
  { 0, 5, 8, 8, 10, 8, 8, 8, 8, 8, 7, 4, 4, 4, 4, 1, }, /* 22 */
  { 0, 5, 8, 8, 10, 9, 9, 9, 9, 9, 8, 5, 5, 5, 4, 1, }, /* 23 */
  { 0, 5, 8, 8, 10, 9, 10, 10, 9, 9, 9, 6, 6, 6, 4, 2, }, /* 24 */
  { 0, 5, 8, 8, 10, 10, 10, 10, 10, 10, 9, 8, 6, 6, 5, 2, }, /* 25 */
  { 0, 5, 8, 8, 10, 10, 11, 11, 10, 10, 10, 9, 7, 7, 5, 3, }, /* 26 */
  { 0, 5, 8, 8, 10, 11, 12, 12, 11, 11, 11, 10, 8, 8, 5, 3, }, /* 27 */
  { 0, 5, 8, 9, 11, 12, 13, 13, 13, 13, 13, 13, 11, 11, 8, 5, }, /* 28 */
  { 0, 5, 8, 9, 11, 13, 14, 15, 15, 15, 16, 15, 13, 13, 11, 6, }, /* 29 */
  { 0, 6, 9, 10, 12, 13, 16, 16, 16, 16, 18, 18, 16, 16, 15, 8, }, /* 30 */
  { 0, 6, 9, 10, 12, 14, 17, 18, 18, 18, 21, 20, 18, 18, 18, 9, }, /* 31 */
  { 0, 6, 9, 11, 13, 15, 18, 19, 20, 20, 23, 23, 21, 21, 21, 11, }, /* 32 */
  { 0, 6, 9, 12, 14, 16, 18, 20, 21, 21, 24, 24, 22, 24, 21, 13, }, /* 33 */
  { 0, 6, 9, 13, 14, 17, 19, 21, 21, 21, 25, 25, 23, 27, 22, 14, }, /* 34 */
  { 0, 6, 10, 13, 15, 17, 19, 21, 22, 22, 25, 25, 25, 29, 22, 16, }, /* 35 */
  { 0, 6, 10, 14, 15, 18, 20, 22, 22, 22, 26, 26, 26, 32, 23, 17, }, /* 36 */
  { 0, 6, 10, 15, 16, 19, 20, 23, 23, 23, 27, 27, 27, 35, 23, 19, }, /* 37 */
  { 0, 6, 11, 15, 17, 20, 21, 24, 25, 25, 28, 29, 29, 36, 26, 21, }, /* 38 */
  { 0, 6, 11, 15, 18, 21, 22, 25, 26, 26, 29, 30, 32, 37, 29, 24, }, /* 39 */
  { 0, 7, 12, 16, 18, 21, 22, 27, 28, 28, 29, 32, 34, 37, 33, 26, }, /* 40 */
  { 0, 7, 12, 16, 19, 22, 23, 28, 29, 29, 30, 33, 37, 38, 36, 29, }, /* 41 */
  { 0, 7, 13, 16, 20, 23, 24, 29, 31, 31, 31, 35, 39, 39, 39, 31, }, /* 42 */
#else
  {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   2,   2,   2,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   2,   2,   2,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   3,   3,   3,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   4,   4,   4,   3,   2,   2,   2,   2,   2,   2,   1,   0,   1,   1,   -1, },
  {   0,   5,   5,   5,   4,   2,   2,   2,   2,   2,   2,   1,   1,   1,   1,   -1, },
  {   0,   5,   6,   6,   4,   3,   2,   2,   2,   2,   2,   2,   1,   2,   2,   -2, },
  {   0,   5,   8,   8,   5,   3,   3,   3,   3,   2,   2,   2,   2,   2,   2,   -2, },
  {   0,   6,   9,   9,   5,   4,   4,   3,   4,   3,   3,   2,   2,   2,   2,   -2, },
  {   0,   6,   9,  10,   8,   6,   6,   5,   5,   5,   4,   2,   2,   2,   2,   -2, },
  {   0,   6,  10,  11,  10,   9,   9,   6,   6,   5,   5,   4,   4,   3,   3,   -2, },
  {   0,   7,  11,  12,  12,  12,  11,   9,   7,   7,   6,   5,   5,   4,   5,   -2, },
  {   0,   7,  12,  13,  15,  15,  13,  10,   9,   8,   8,   6,   6,   5,   5,   -2, },
  {   0,   7,  12,  15,  17,  17,  16,  12,   9,   9,   9,   7,   7,   5,   6,   -2, },
  {   0,   8,  13,  16,  19,  20,  19,  16,  14,  13,  12,   9,   9,   7,   7,   -2, },
  {   0,   8,  14,  18,  20,  22,  22,  20,  18,  17,  14,  11,  10,   9,   9,   -2, },
  {   0,   9,  15,  19,  23,  23,  25,  23,  23,  20,  17,  13,  12,  10,   9,   -2, },
  {   0,   9,  16,  20,  24,  26,  28,  27,  27,  24,  20,  15,  13,  12,  11,   -2, },
  {   0,   9,  16,  22,  26,  28,  31,  31,  31,  28,  23,  17,  15,  13,  12,   -2, },
  {   0,  10,  16,  23,  27,  29,  32,  32,  32,  30,  25,  18,  16,  13,  12,   -2, },
  {   0,  11,  17,  23,  27,  30,  33,  33,  33,  30,  27,  19,  16,  13,  12,   -2, },
  {   0,  12,  17,  24,  27,  30,  33,  33,  34,  32,  29,  20,  16,  13,  12,   -2, },
  {   0,  12,  18,  25,  28,  31,  34,  34,  34,  33,  30,  20,  16,  13,  12,   -2, },
  {   0,  13,  18,  26,  29,  32,  34,  34,  35,  34,  33,  21,  17,  13,  12,   -2, },
#endif
  };
  char m_lutChroma[26][3 * 16];
  char m_distFactorChroma[3] = { 16, 8, 6, };
  char m_tuSizeFactorChroma[2][64] = {
    { 9,9,9,9,8,8,8,0,9,9,9,9,8,8,8,0,9,9,8,8,8,8,8,0,9,9,8,8,8,8,8,0,8,8,8,8,8,8,8,0,8,8,8,8,8,8,8,0,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0, },
    { 13,13,13,13,9,9,9,0,13,13,13,13,9,9,9,0,13,13,13,13,9,9,9,0,13,13,13,9,9,9,9,0,9,9,9,9,9,9,9,0,9,9,9,9,9,0,0,0,9,9,9,9,9,0,0,0,0,0,0,0,0,0,0,0, },
  };
  char m_tuMADFactorChroma[2][16] = {
    { 0,1,3,4,5,7,9,11,11,11,11,11,11,11,11,11, },
    { 0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1, },
  };
#endif

#if JVET_AJ0237_INTERNAL_12BIT
  int internalBitDepth;
#endif
public:
  BilateralFilter();
  ~BilateralFilter();

  void create();
  void destroy();
#if JVET_AJ0237_INTERNAL_12BIT
  void setInternalBitDepth(int bdDepth) { internalBitDepth = bdDepth; }
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  int getInternalBitDepth() { return internalBitDepth; }
#endif
#if JVET_V0094_BILATERAL_FILTER
  void bilateralFilterRDOdiamond5x5(const ComponentID compID, PelBuf& resiBuf, const CPelBuf& predBuf, PelBuf& recoBuf, int32_t qp, const CPelBuf& recIPredBuf, const ClpRng& clpRng, TransformUnit & currTU, bool useReco, bool doReshape = false, std::vector<Pel>* pLUT = nullptr);
  void bilateralFilterPicRDOperCTU(const ComponentID compID, CodingStructure& cs, PelUnitBuf& src,BIFCabacEst* bifCABACEstimator);
  void bilateralFilterDiamond5x5(const ComponentID compID, const CPelUnitBuf& src, PelUnitBuf& rec, int32_t qp, const ClpRng& clpRng, TransformUnit & currTU, bool noClip
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
    , bool clipTop, bool clipBottom, bool clipLeft, bool clipRight
#endif
  );

  const char* getFilterLutParameters(int16_t* block, const int stride, const int width, const int height, const PredMode predMode, const int qp, int& bfac
#if JVET_AK0118_BF_FOR_INTRA_PRED
    , int& madValue
#endif
    );
  void clipNotBilaterallyFilteredBlocks(const ComponentID compID, const CPelUnitBuf& src, PelUnitBuf& rec, const ClpRng& clpRng, TransformUnit & currTU);
#endif

#if JVET_X0071_CHROMA_BILATERAL_FILTER
  const char* getFilterLutParametersChroma(int16_t* block, const int stride, const int width, const int height, const PredMode predMode, const int qp, int& bfac, int widthForStrength, int heightForStrength, bool isLumaValid);
#endif

#if JVET_AF0112_BIF_DYNAMIC_SCALING
  bool getApplyBIF(const TransformUnit& currTU, ComponentID compID);
#endif

#if ENABLE_SIMD_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER_ENABLE_SIMD
#ifdef TARGET_SIMD_X86
#if JVET_AJ0237_INTERNAL_12BIT
  template<X86_VEXT vext>
  static void simdFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum, int bdShift
#if JVET_AK0118_BF_FOR_INTRA_PRED
    , bool isIntraPredBf
#endif
    );
#else
  template<X86_VEXT vext>
  static void simdFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip, int cutBitsNum);
#endif
#if JVET_AF0112_BIF_DYNAMIC_SCALING
  template<X86_VEXT vext>
  static int simdCalcMAD(int16_t* block, int stride, int width, int height, int whlog2);
#endif

  void    initBilateralFilterX86();
  template <X86_VEXT vext>
  void    _initBilateralFilterX86();
#endif
#endif
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  bool isCrossedByVirtualBoundaries(const CodingStructure &cs, const int xPos, const int yPos, const int width,
                                    const int height, bool &clipTop, bool &clipBottom, bool &clipLeft, bool &clipRight,
                                    int &numHorVirBndry, int &numVerVirBndry, int horVirBndryPos[],
                                    int verVirBndryPos[], bool isEncoderRDO = false);
#endif
};

#endif

#endif /* BILATERALFILTER_H */
