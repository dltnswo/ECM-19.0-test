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

/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration
#include "Unit.h"
#include "Buffer.h"
#include "InterpolationFilter.h"
#include "Reshape.h"

#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
FrameCtrlInfo g_pictureControlInfo[ALF_CTB_MAX_NUM_APS];
bool g_pictureControlInfoInit = false;
#endif

void applyPROFCore(Pel* dst, int dstStride, const Pel* src, int srcStride, int width, int height, const Pel* gradX, const Pel* gradY, int gradStride, const int* dMvX, const int* dMvY, int dMvStride, const bool& bi, int shiftNum, Pel offset, const ClpRng& clpRng)
{
  int idx = 0;

  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);
  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      int32_t dI = dMvX[idx] * gradX[w] + dMvY[idx] * gradY[w];
      dI = Clip3(-dILimit, dILimit - 1, dI);
      dst[w] = src[w] + dI;
      if (!bi)
      {
        dst[w] = (dst[w] + offset) >> shiftNum;
        dst[w] = ClipPel(dst[w], clpRng);
      }

      idx++;
    }
    gradX += gradStride;
    gradY += gradStride;
    dst += dstStride;
    src += srcStride;
  }
}

#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
int64_t getSumOfDifferenceCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int rowSubShift, int bitDepth)
{
  height     >>= rowSubShift;
  src0Stride <<= rowSubShift;
  src1Stride <<= rowSubShift;

  int64_t sum = 0;
#define GET_SUM_DIFF_CORE_OP( ADDR ) sum += ( src0[ADDR] - src1[ADDR] )
#define GET_SUM_DIFF_CORE_INC    \
  src0 += src0Stride;            \
  src1 += src1Stride;            \

  SIZE_AWARE_PER_EL_OP(GET_SUM_DIFF_CORE_OP, GET_SUM_DIFF_CORE_INC);

#undef GET_SUM_DIFF_CORE_OP
#undef GET_SUM_DIFF_CORE_INC

  return sum;
}
#endif

#if JVET_AG0067_DMVR_EXTENSIONS
int getMean(int sum, int div)
{
  int sign = 1;
  if (sum < 0 )
  {
    sum  = -sum;
    sign = -1;
  }
  int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
  int x            = floorLog2(div);
  int normNum1     = (div << 4 >> x) & 15;
  int v            = divTable[normNum1] | 8;
  x += (normNum1 != 0);
  int shift  = 13 - x;
  int retVal = 0;
  if (shift < 0)
  {
    shift   = -shift;
    int add = (1 << (shift - 1));
    retVal  = (sum * v + add) >> shift;
  }
  else
  {
    retVal = (sum * v) << shift;
  }
  return sign * (retVal >> 16);
}
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void getAbsoluteDifferencePerSampleCore(Pel* dst, int dstStride, const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height)
{
#define GET_ABS_DIFF_PER_SAMPLE_CORE_OP( ADDR ) dst[ADDR] = std::abs( src0[ADDR] - src1[ADDR] )
#define GET_ABS_DIFF_PER_SAMPLE_CORE_INC    \
  src0 += src0Stride;                       \
  src1 += src1Stride;                       \
  dst  += dstStride;                        \

  SIZE_AWARE_PER_EL_OP(GET_ABS_DIFF_PER_SAMPLE_CORE_OP, GET_ABS_DIFF_PER_SAMPLE_CORE_INC);

#undef GET_ABS_DIFF_PER_SAMPLE_CORE_OP
#undef GET_ABS_DIFF_PER_SAMPLE_CORE_INC
}

template <uint8_t maskType>
int64_t getMaskedSampleSumCore(Pel* src, int srcStride, int width, int height, int bitDepth, short* weightMask, int maskStepX, int maskStride, int maskStride2)
{
  const Pel* mask      = weightMask;
  const int  cols      = width;
        int  rows      = height;

  int64_t sum = 0;
  if (maskType == 1) // 1: Use mask
  {
    for (; rows != 0; rows--)
    {
      for (int n = 0; n < cols; n++)
      {
        sum  += (src[n]) * (*mask);
        mask += maskStepX;
      }
      src  += srcStride;
      mask += (maskStride + maskStride2);
    }
  }
  else if (maskType == 2 || maskType == 3) // 2: Use binary mask that contains only 0's and 1's, 3: Inverse the input binary mask before use
  {
    for (; rows != 0; rows--)
    {
      for (int n = 0; n < cols; n++)
      {
        sum += (src[n]) & (maskType == 3 ? ((*mask) - 1) : (-(*mask)));
        mask += maskStepX;
      }
      src  += srcStride;
      mask += (maskStride + maskStride2);
    }
  }
  else // No mask
  {
    for (; rows != 0; rows--)
    {
      for (int n = 0; n < cols; n++)
      {
        sum += src[n];
      }
      src  += srcStride;
    }
  }

  return sum;
}
#endif

#if JVET_W0097_GPM_MMVD_TM
void roundBDCore(const Pel* srcp, const int srcStride, Pel* dest, const int destStride, int width, int height, const ClpRng& clpRng)
{
  const int32_t clipbd = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int32_t shiftDefault = IF_INTERNAL_FRAC_BITS(clipbd);
#else
  const int32_t shiftDefault = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
#endif
  const int32_t offsetDefault = (1 << (shiftDefault - 1)) + IF_INTERNAL_OFFS;

  if (width == 1)
  {
    THROW("Blocks of width = 1 not supported");
  }
  else
  {
#define RND_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( srcp[ADDR] + offsetDefault, shiftDefault), clpRng )
#define RND_INC        \
    srcp += srcStride;  \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP(RND_OP, RND_INC);

#undef RND_OP
#undef RND_INC
  }
}

void weightedAvgCore(const Pel* src0, const unsigned src0Stride, const Pel* src1, const unsigned src1Stride, Pel* dest, const unsigned destStride, const int8_t w0, const int8_t w1, int width, int height, const ClpRng& clpRng)
{
  const int8_t log2WeightBase = g_bcwLog2WeightBase;
  const int clipbd = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
#else
  const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
#endif
  const int offset = (1 << (shiftNum - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR]*w0 + src1[ADDR]*w1 + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src0Stride; \
    src1 += src1Stride; \
    dest += destStride; \

  SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
}

void copyClipCore(const Pel* srcp, const unsigned srcStride, Pel* dest, const unsigned destStride, int width, int height, const ClpRng& clpRng)
{
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( srcp[ADDR], clpRng )
#define RECO_INC        \
  srcp += srcStride;  \
  dest += destStride; \

  SIZE_AWARE_PER_EL_OP(RECO_OP, RECO_INC);

#undef RECO_OP
#undef RECO_INC
}
#endif
template< typename T >
#if JVET_Z0136_OOB
void addAvgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool *isOOB)
#else
void addAvgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
#endif
{
#define ADD_AVG_CORE_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src1[ADDR] + src2[ADDR] + offset ), rshift ), clpRng )
#define ADD_AVG_CORE_INC    \
  src1 += src1Stride;       \
  src2 += src2Stride;       \
  dest +=  dstStride;       \

  SIZE_AWARE_PER_EL_OP( ADD_AVG_CORE_OP, ADD_AVG_CORE_INC );

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
}

#if JVET_AE0169_BIPREDICTIVE_IBC
template< typename T >
void avgCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height )
{
#define ADD_AVG_CORE_OP( ADDR ) dest[ADDR] = ( ( src1[ADDR] + src2[ADDR] + 1 ) >> 1 )
#define ADD_AVG_CORE_INC    \
  src1 += src1Stride;       \
  src2 += src2Stride;       \
  dest +=  dstStride;       \

  SIZE_AWARE_PER_EL_OP( ADD_AVG_CORE_OP, ADD_AVG_CORE_INC );

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
}
#endif

#if JVET_AD0213_LIC_IMP
template< typename T >
void toLastCore(T* src, int srcStride, int width, int height, int shiftNum, int offset, const ClpRng& clpRng)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      src[x] = ClipPel(rightShift((src[x] + offset), shiftNum), clpRng);
    }
    src += srcStride;
  }
}

template< typename T >
void licRemoveWeightHighFreqCore(T* src0, T* src1, T* dst, int length, int w0, int w1, int offset, const ClpRng& clpRng)
{
  for (int w = 0; w < length; w++)
  {
    T iTemp = ClipPel(T((int(src0[w])*w0 - int(src1[w])*w1 + offset) >> 16), clpRng);
    dst[w] = iTemp;
  }
}
#endif

void addBIOAvgCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  int b = 0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      b = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#else
      dst[x] = ClipPel((int16_t)rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#endif

      b = tmpx * (gradX0[x + 1] - gradX1[x + 1]) + tmpy * (gradY0[x + 1] - gradY1[x + 1]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      dst[x + 1] = ClipPel(rightShift((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);
#else
      dst[x + 1] = ClipPel((int16_t)rightShift((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);
#endif

      b = tmpx * (gradX0[x + 2] - gradX1[x + 2]) + tmpy * (gradY0[x + 2] - gradY1[x + 2]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      dst[x + 2] = ClipPel(rightShift((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);
#else
      dst[x + 2] = ClipPel((int16_t)rightShift((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);
#endif

      b = tmpx * (gradX0[x + 3] - gradX1[x + 3]) + tmpy * (gradY0[x + 3] - gradY1[x + 3]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      dst[x + 3] = ClipPel(rightShift((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
#else
      dst[x + 3] = ClipPel((int16_t)rightShift((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
#endif
    }
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
}

#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
void calcBIOParameterCoreHighPrecision(const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int width, int height, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, Pel* dI
#if JVET_AG0067_DMVR_EXTENSIONS
                                       ,Pel* gX, Pel* gY
#endif
)
{
  width -= 2;
  height -= 2;
  const int bioParamOffset = widthG + 1;
  srcY0Tmp += src0Stride + 1;
  srcY1Tmp += src1Stride + 1;
  gradX0 += bioParamOffset;  gradX1 += bioParamOffset;
  gradY0 += bioParamOffset;  gradY1 += bioParamOffset;
  s1  += bioParamOffset;  s2  += bioParamOffset;
  s3  += bioParamOffset;  s5  += bioParamOffset;
  s6  += bioParamOffset;
#if JVET_AG0067_DMVR_EXTENSIONS
  gX  += bioParamOffset;
  gY  += bioParamOffset;
#endif
  const int shift4 = 4;
  dI += bioParamOffset;
  int32_t  temp=0, tempGX=0, tempGY=0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      temp = (int32_t) ((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4)) ;
      tempGX = (int32_t) (gradX0[x] + gradX1[x]);
      tempGY = (int32_t) (gradY0[x] + gradY1[x]);
      dI[x] = (Pel) temp;
      s1[x] =  tempGX * tempGX;
      s2[x] =  tempGX * tempGY;
      s5[x] =  tempGY * tempGY;
      s3[x] = tempGX * temp;
      s6[x] = tempGY * temp;
#if JVET_AG0067_DMVR_EXTENSIONS
      gX[x] = tempGX;
      gY[x] = tempGY;
#endif
      
    }

    srcY0Tmp += src0Stride;
    srcY1Tmp += src1Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
    s1 += widthG;
    s2 += widthG;
    s3 += widthG;
    s5 += widthG;
    s6 += widthG;
    dI += widthG;
#if JVET_AG0067_DMVR_EXTENSIONS
    gX += widthG;
    gY += widthG;
#endif
  }
  
  return;
}

void calcBIOParamSum4CoreHighPrecision(int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, int width, int height, const int widthG, int32_t* sumS1, int32_t* sumS2, int32_t* sumS3, int32_t* sumS5, int32_t* sumS6
#if JVET_AG0067_DMVR_EXTENSIONS
                                       ,Pel* dI, Pel* gX, Pel* gY , bool isGPM = false, bool isSub = false
#endif
)
{
#if JVET_AG0067_DMVR_EXTENSIONS
  int meanDiff = 0;
  int absmeanDiff = 0;
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        meanDiff += dI[x];
        absmeanDiff += abs(dI[x]);
      }
      dI += widthG;
    }
    
    if (isSub )
    {
      meanDiff = getMean( meanDiff + (height*width >> 1), height*width);
    }
    else
    {
      if (isGPM || (absmeanDiff > 2 * abs(meanDiff)))
      {
        meanDiff = 0;
      }
      meanDiff = getMean( meanDiff + height*width, height*width << 1);
    }
  }
#endif
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int w = 1;
      w = (x >= (width/2) ? width - x : x + 1) * (y >= (height/2) ? height - y : y + 1);
      *sumS1 += w * s1[x];
      *sumS2 += w * s2[x];
      *sumS3 += w * s3[x];
      *sumS5 += w * s5[x];
      *sumS6 += w * s6[x];
#if JVET_AG0067_DMVR_EXTENSIONS
      *sumS3 -= w * gX[x]*meanDiff;
      *sumS6 -= w * gY[x]*meanDiff;
#endif
    }

    s1 += widthG;
    s2 += widthG;
    s3 += widthG;
    s5 += widthG;
    s6 += widthG;
#if JVET_AG0067_DMVR_EXTENSIONS
    gX += widthG;
    gY += widthG;
#endif
  }
}
#endif

#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
void calcBIOParameterCore(const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int width, int height, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, Pel* dI)
{
  width -= 2;
  height -= 2;
  const int bioParamOffset = widthG + 1;
  srcY0Tmp += src0Stride + 1;
  srcY1Tmp += src1Stride + 1;
  gradX0 += bioParamOffset;  gradX1 += bioParamOffset;
  gradY0 += bioParamOffset;  gradY1 += bioParamOffset;
  absGX  += bioParamOffset;  absGY  += bioParamOffset;
  dIX    += bioParamOffset;  dIY    += bioParamOffset;
  signGyGx += bioParamOffset;
  const int shift4 = 4;
  const int shift5 = 1;

  if (dI)
  {
    dI += bioParamOffset;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        int tmpGX = (gradX0[x] + gradX1[x]) >> shift5;
        int tmpGY = (gradY0[x] + gradY1[x]) >> shift5;
        int tmpDI = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
        dI[x] = tmpDI;
        absGX[x] = (tmpGX < 0 ? -tmpGX : tmpGX);
        absGY[x] = (tmpGY < 0 ? -tmpGY : tmpGY);
        dIX[x] = (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
        dIY[x] = (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
        signGyGx[x] = (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));
      }
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      absGX += widthG;
      absGY += widthG;
      dI += widthG;
      dIX += widthG;
      dIY += widthG;
      signGyGx += widthG;
    }

    return;
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int tmpGX = (gradX0[x] + gradX1[x]) >> shift5;
      int tmpGY = (gradY0[x] + gradY1[x]) >> shift5;
      int tmpDI = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      absGX[x] = (tmpGX < 0 ? -tmpGX : tmpGX);
      absGY[x] = (tmpGY < 0 ? -tmpGY : tmpGY);
      dIX[x] = (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
      dIY[x] = (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
      signGyGx[x] = (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));
    }

    srcY0Tmp += src0Stride;
    srcY1Tmp += src1Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
    absGX += widthG;
    absGY += widthG;
    dIX += widthG;
    dIY += widthG;
    signGyGx += widthG;
  }
}

void calcBIOParamSum5Core(Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, const int widthG, const int width, const int height, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      const int sampleIdx = y * width + x;
      sumAbsGX[sampleIdx] = 0;
      sumAbsGY[sampleIdx] = 0;
      sumDIX[sampleIdx] = 0;
      sumDIY[sampleIdx] = 0;
      sumSignGyGx[sampleIdx] = 0;
      for (int yy = 0; yy < 5; yy++)
      {
        for (int xx = 0; xx < 5; xx++)
        {
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
          int w = 1;
          w = (xx >= 2 ? 5 - xx : xx + 1) * (yy >= 2 ? 5 - yy : yy + 1);
          sumAbsGX[sampleIdx] += w * absGX[xx];
          sumAbsGY[sampleIdx] += w * absGY[xx];
          sumDIX[sampleIdx] += w * dIX[xx];
          sumDIY[sampleIdx] += w * dIY[xx];
          sumSignGyGx[sampleIdx] += w * signGyGx[xx];
#else
          sumAbsGX[sampleIdx] += absGX[xx];
          sumAbsGY[sampleIdx] += absGY[xx];
          sumDIX[sampleIdx] += dIX[xx];
          sumDIY[sampleIdx] += dIY[xx];
          sumSignGyGx[sampleIdx] += signGyGx[xx];
#endif
        }

        absGX += widthG;
        absGY += widthG;
        dIX += widthG;
        dIY += widthG;
        signGyGx += widthG;
      }

      sumDIX[sampleIdx] <<= 2;
      sumDIY[sampleIdx] <<= 2;
#if JVET_AE0091_ITERATIVE_BDOF
      int regVxVy = (1 << 8);
      sumAbsGX[sampleIdx] += regVxVy;
      sumAbsGY[sampleIdx] += regVxVy;
#endif
      absGX += (1 - 5 * widthG);
      absGY += (1 - 5 * widthG);
      dIX += (1 - 5 * widthG);
      dIY += (1 - 5 * widthG);
      signGyGx += (1 - 5 * widthG);
    }

    absGX += (widthG - width);
    absGY += (widthG - width);
    dIX += (widthG - width);
    dIY += (widthG - width);
    signGyGx += (widthG - width);
  }
}

#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
void calcBIOParamSum5NOSIMCore(int32_t* absGX, int32_t* absGY, int32_t* dIX, int32_t* dIY, int32_t* signGyGx, const int widthG, const int width, const int height, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx ,Pel* dI
#if JVET_AG0067_DMVR_EXTENSIONS
  , Pel* gX, Pel* gY
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  , bool noMeanRemove
#endif
#endif
)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      const int sampleIdx = y * width + x;
      sumAbsGX[sampleIdx] = 0;
      sumAbsGY[sampleIdx] = 0;
      sumDIX[sampleIdx] = 0;
      sumDIY[sampleIdx] = 0;
      sumSignGyGx[sampleIdx] = 0;
#if JVET_AG0067_DMVR_EXTENSIONS      
      int meanDiff = 0;
      int absmeanDiff = 0;
      int sX0 = 0, sX1 = 0;
#endif
      int w = 1, a = 1, b = 2, c = 4, d = 4, e = 8, f = 16;
      int weight[5][5] = {{a, b, c, b, a}, {b, d, e, d, b}, {c, e, f, e, c}, {b, d, e, d, b}, {a, b, c, b, a}};
      const int regVxVy = 2528; // = ((1 << 11) * 100)/81. 100 is summation of the new weights, 81 was the summation of the old weights

      for (int yy = 0; yy < 5; yy++)
      {
        for (int xx = 0; xx < 5; xx++)
        {
          w = weight[yy][xx];
          sumAbsGX[sampleIdx] += w * absGX[xx];
          sumAbsGY[sampleIdx] += w * absGY[xx];
          sumDIX[sampleIdx] += w * dIX[xx];
          sumDIY[sampleIdx] += w * dIY[xx];
          sumSignGyGx[sampleIdx] += w * signGyGx[xx];
#if JVET_AG0067_DMVR_EXTENSIONS          
          meanDiff    +=  dI[xx];
          absmeanDiff += abs(dI[xx]);
          sX0 -= w * gX[xx];
          sX1 -= w * gY[xx];
#endif
        }
        absGX += widthG;
        absGY += widthG;
        dIX += widthG;
        dIY += widthG;
        signGyGx += widthG;
        dI += widthG;
#if JVET_AG0067_DMVR_EXTENSIONS
        gX += widthG;
        gY += widthG;
#endif
      }

#if JVET_AG0067_DMVR_EXTENSIONS
#if JVET_AL0081_BDOF_LDB_MV_REFINE
      meanDiff = noMeanRemove ? 0 : ((absmeanDiff > 2 * abs(meanDiff)) ? 0 : (meanDiff + 32) >> 6);
#else
      meanDiff = (absmeanDiff > 2 * abs(meanDiff))  ? 0 : (meanDiff + 32) >> 6;
#endif
      sumDIX[sampleIdx] += sX0*meanDiff;
      sumDIY[sampleIdx] += sX1*meanDiff;
#endif
      sumDIX[sampleIdx] += (sumDIX[sampleIdx] + 2) >> 2;
      sumDIY[sampleIdx] += (sumDIY[sampleIdx] + 2) >> 2;
      sumAbsGX[sampleIdx] += regVxVy;
      sumAbsGY[sampleIdx] += regVxVy;
      absGX += (1 - 5 * widthG);
      absGY += (1 - 5 * widthG);
      dIX += (1 - 5 * widthG);
      dIY += (1 - 5 * widthG);
      signGyGx += (1 - 5 * widthG);
      dI += (1 - 5 * widthG);
#if JVET_AG0067_DMVR_EXTENSIONS
      gX += (1 - 5 * widthG);
      gY += (1 - 5 * widthG);
#endif
    }
    absGX += (widthG - width);
    absGY += (widthG - width);
    dIX += (widthG - width);
    dIY += (widthG - width);
    signGyGx += (widthG - width);
    dI += (widthG - width);
#if JVET_AG0067_DMVR_EXTENSIONS
    gX += (widthG - width);
    gY += (widthG - width);
#endif
  }
}
#endif
void calcBIOParamSum4Core(Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, int width, int height, const int widthG, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *sumAbsGX += absGX[x];
      *sumAbsGY += absGY[x];
      *sumDIX += dIX[x];
      *sumDIY += dIY[x];
      *sumSignGyGx += signGyGx[x];
    }

    absGX += widthG;
    absGY += widthG;
    dIX += widthG;
    dIY += widthG;
    signGyGx += widthG;
  }
}

void calcBIOClippedVxVyCore(int* sumDIXSample32bit, int* sumAbsGxSample32bit, int* sumDIYSample32bit, int* sumAbsGySample32bit, int* sumSignGyGxSample32bit, const int limit, const int bioSubblockSize, int* tmpxSample32bit, int* tmpySample32bit)
{
  for (int idx = 0; idx < bioSubblockSize; idx++)
  {
    *tmpxSample32bit = Clip3(-limit, limit, (*sumDIXSample32bit) >> (*sumAbsGxSample32bit));
    int tmpData = ((*sumSignGyGxSample32bit) * (*tmpxSample32bit)) >> 1;
    *tmpySample32bit = Clip3(-limit, limit, (((*sumDIYSample32bit) - tmpData) >> (*sumAbsGySample32bit)));
    sumDIXSample32bit++;
    sumAbsGxSample32bit++;
    sumDIYSample32bit++;
    sumAbsGySample32bit++;
    sumSignGyGxSample32bit++;
    tmpxSample32bit++;
    tmpySample32bit++;
  }
}
#if JVET_Z0136_OOB
void addBIOAvgNCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride, int width, int height, int* tmpx, int* tmpy, int shift, int offset, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool *isOOB)
#else
void addBIOAvgNCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride, int width, int height, int* tmpx, int* tmpy, int shift, int offset, const ClpRng& clpRng)
#endif
{
  int b = 0;
#if JVET_Z0136_OOB
  int offset2 = offset >> 1;
  int shift2 = shift - 1;
  bool *pMcMask0 = mcMask[0];
  bool *pMcMask1 = mcMask[1];
#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
  int pX = 0, pY = 0;
  const int tt = 16;
#endif
  if (isOOB[0] || isOOB[1])
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        b = (int)tmpx[x] * (gradX0[x] - gradX1[x]) + (int)tmpy[x] * (gradY0[x] - gradY1[x]);
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dst[x] = ClipPel(rightShift(src1[x] + offset2, shift2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dst[x] = ClipPel(rightShift(src0[x] + offset2, shift2), clpRng);
        }
        else
        {
          dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
        }
      }

      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      tmpx += width;
      tmpy += width;
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
        pX = (tmpx[x] >  tt)  ?  1 : ((tmpx[x] < -tt) ? -1 : 0);
        pY = (tmpy[x] >  tt)  ?  1 : ((tmpy[x] < -tt) ? -1 : 0);
        int xX = tmpx[x] - 32 * pX;
        int yY = tmpy[x] - 32 * pY;
        b = (int)xX * (gradX0[x + pY * gradStride + pX] - gradX1[x - pY * gradStride - pX]) + (int)yY * (gradY0[x + pY * gradStride + pX] - gradY1[x - pY * gradStride - pX]);
        dst[x] = ClipPel(rightShift((src0[x + pY * src0Stride + pX] + src1[x - pY * src1Stride - pX] + b + offset), shift), clpRng);
#else
        b = (int)tmpx[x] * (gradX0[x] - gradX1[x]) + (int)tmpy[x] * (gradY0[x] - gradY1[x]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
        dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#else
        dst[x] = ClipPel((int16_t)rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#endif
#endif
      }

      tmpx += width;
      tmpy += width;
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
    }
  }
#else
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      b = (int)tmpx[x] * (gradX0[x] - gradX1[x]) + (int)tmpy[x] * (gradY0[x] - gradY1[x]);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#else
      dst[x] = ClipPel((int16_t)rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
#endif
    }

    tmpx += width;    tmpy += width;
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
#endif
  return;
}

void calAbsSumCore(const Pel* diff, int stride, int width, int height, int* absSum)
{
  *absSum = 0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *absSum += ::abs(diff[x]);
    }

    diff += stride;
  }
}
#endif

template<bool pad = true>
void gradFilterCore(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth)
{
  Pel* srcTmp = pSrc + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;
  const int  shift1 = 6;

#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
  for (int y = 0; y < (height - 2); y++)
  {
    for (int x = 0; x < (width - 2); x++)
#else
  for (int y = 0; y < (height - 2 * BIO_EXTEND_SIZE); y++)
  {
    for (int x = 0; x < (width - 2 * BIO_EXTEND_SIZE); x++)
#endif
    {
      gradYTmp[x] = ( srcTmp[x + srcStride] >> shift1 ) - ( srcTmp[x - srcStride] >> shift1 );
      gradXTmp[x] = ( srcTmp[x + 1] >> shift1 ) - ( srcTmp[x - 1] >> shift1 );
    }

    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }

#if !MULTI_PASS_DMVR && !SAMPLE_BASED_BDOF
  if (pad)
  {
  gradXTmp = gradX + gradStride + 1;
  gradYTmp = gradY + gradStride + 1;
  for (int y = 0; y < (height - 2 * BIO_EXTEND_SIZE); y++)
  {
    gradXTmp[-1] = gradXTmp[0];
    gradXTmp[width - 2 * BIO_EXTEND_SIZE] = gradXTmp[width - 2 * BIO_EXTEND_SIZE - 1];
    gradXTmp += gradStride;

    gradYTmp[-1] = gradYTmp[0];
    gradYTmp[width - 2 * BIO_EXTEND_SIZE] = gradYTmp[width - 2 * BIO_EXTEND_SIZE - 1];
    gradYTmp += gradStride;
  }

  gradXTmp = gradX + gradStride;
  gradYTmp = gradY + gradStride;
  ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
  ::memcpy(gradXTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradXTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
  ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
  ::memcpy(gradYTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradYTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
  }
#endif
}

void calcBIOSumsCore(const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx)
{
  const int shift4 = 4;
  const int shift5 = 1;

  for (int y = 0; y < 6; y++)
  {
    for (int x = 0; x < 6; x++)
    {
      int tmpGX = (gradX0[x] + gradX1[x]) >> shift5;
      int tmpGY = (gradY0[x] + gradY1[x]) >> shift5;
      int tmpDI = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      *sumAbsGX += (tmpGX < 0 ? -tmpGX : tmpGX);
      *sumAbsGY += (tmpGY < 0 ? -tmpGY : tmpGY);
      *sumDIX += (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
      *sumDIY += (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
      *sumSignGyGx += (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));

    }

    srcY1Tmp += src1Stride;
    srcY0Tmp += src0Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }
}


void calcBlkGradientCore(int sx, int sy, int     *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
  int     *Gx2 = arraysGx2;
  int     *Gy2 = arraysGy2;
  int     *GxGy = arraysGxGy;
  int     *GxdI = arraysGxdI;
  int     *GydI = arraysGydI;

  // set to the above row due to JVET_K0485_BIO_EXTEND_SIZE
  Gx2 -= (BIO_EXTEND_SIZE*width);
  Gy2 -= (BIO_EXTEND_SIZE*width);
  GxGy -= (BIO_EXTEND_SIZE*width);
  GxdI -= (BIO_EXTEND_SIZE*width);
  GydI -= (BIO_EXTEND_SIZE*width);

  for (int y = -BIO_EXTEND_SIZE; y < unitSize + BIO_EXTEND_SIZE; y++)
  {
    for (int x = -BIO_EXTEND_SIZE; x < unitSize + BIO_EXTEND_SIZE; x++)
    {
      sGx2 += Gx2[x];
      sGy2 += Gy2[x];
      sGxGy += GxGy[x];
      sGxdI += GxdI[x];
      sGydI += GydI[x];
    }

    Gx2 += width;
    Gy2 += width;
    GxGy += width;
    GxdI += width;
    GydI += width;
  }
}

#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
void removeWeightHighFreq(int16_t* dst, int dstStride, const int16_t* src, int srcStride, int width, int height, int shift, int bcwWeight)
{
  int normalizer = ((1 << 16) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;
  int weight0 = normalizer << g_bcwLog2WeightBase;
  int weight1 = (g_bcwWeightBase - bcwWeight)*normalizer;
#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP( ADDR )      dst[ADDR] =             (dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16

  SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
}

void removeHighFreq(int16_t* dst, int dstStride, const int16_t* src, int srcStride, int width, int height)
{
#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP( ADDR )      dst[ADDR] =             2 * dst[ADDR] - src[ADDR]

  SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
}
#endif

template<typename T>
void reconstructCore( const T* src1, int src1Stride, const T* src2, int src2Stride, T* dest, int dstStride, int width, int height, const ClpRng& clpRng )
{
#define RECO_CORE_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_CORE_INC     \
  src1 += src1Stride;     \
  src2 += src2Stride;     \
  dest +=  dstStride;     \

  SIZE_AWARE_PER_EL_OP( RECO_CORE_OP, RECO_CORE_INC );

#undef RECO_CORE_OP
#undef RECO_CORE_INC
}


template<typename T>
void linTfCore( const T* src, int srcStride, Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip )
{
#define LINTF_CORE_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_CORE_INC  \
  src += srcStride;     \
  dst += dstStride;     \

  SIZE_AWARE_PER_EL_OP( LINTF_CORE_OP, LINTF_CORE_INC );

#undef LINTF_CORE_OP
#undef LINTF_CORE_INC
}

#if JVET_Z0136_OOB
bool isMvOOBCore(const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, bool *mcMaskChroma, bool lumaOnly, ChromaFormat componentID)
{
  int chromaScale = getComponentScaleX(COMPONENT_Cb, componentID);
  const int mvstep = 1 << MV_FRACTIONAL_BITS_INTERNAL;
  const int mvstepHalf = mvstep >> 1;

  int horMax = (((int)pps->getPicWidthInLumaSamples() - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int horMin = -mvstepHalf;
  int verMax = (((int)pps->getPicHeightInLumaSamples() - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int verMin = -mvstepHalf;

  int offsetX = (pos.x << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getHor();
  int offsetY = (pos.y << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getVer();
  bool isOOB = false;
  if ((offsetX <= horMin)
    || ((offsetX + ((size.width - 1) << MV_FRACTIONAL_BITS_INTERNAL) ) >= horMax)
    || (offsetY <= verMin)
    || ((offsetY + ((size.height - 1) << MV_FRACTIONAL_BITS_INTERNAL)) >= verMax))
  {
    isOOB = true;
  }
  if (isOOB)
  {
    int baseOffsetX = offsetX;
    bool *pMcMask = mcMask;

    for (int y = 0; y < size.height; y++, offsetY += mvstep)
    {
      offsetX = baseOffsetX;
      bool checkY = (offsetY <= verMin) || (offsetY >= verMax);
      for (int x = 0; x < size.width; x++, offsetX += mvstep)
      {
        pMcMask[x] = (offsetX <= horMin) || (offsetX >= horMax) || checkY;
      }
      pMcMask += size.width;
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma = mcMaskChroma;
      pMcMask = mcMask;
      int widthChroma = (size.width) >> chromaScale;
      int heightChroma = (size.height) >> chromaScale;
      int widthLuma2 = size.width << chromaScale;
      for (int y = 0; y < heightChroma; y++)
      {
        for (int x = 0; x < widthChroma; x++)
        {
          pMcMaskChroma[x] = pMcMask[x << chromaScale];
        }
        pMcMaskChroma += widthChroma;
        pMcMask += widthLuma2;
      }
    }
  }
  else
  {
    bool *pMcMask = mcMask;
    memset(pMcMask, false, size.width * size.height);

    bool *pMcMaskChroma = mcMaskChroma;
    int widthChroma = (size.width) >> chromaScale;
    int heightChroma = (size.height) >> chromaScale;
    memset(pMcMaskChroma, false, widthChroma * heightChroma);
  }
  return isOOB;
}

bool isMvOOBSubBlkCore(const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly, ChromaFormat componentID)
{
  int chromaScale = getComponentScaleX(COMPONENT_Cb, componentID);
  const int mvstep = 1 << MV_FRACTIONAL_BITS_INTERNAL;
  const int mvstepHalf = mvstep >> 1;

  int horMax = (((int)pps->getPicWidthInLumaSamples() - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int horMin = -mvstepHalf;
  int verMax = (((int)pps->getPicHeightInLumaSamples() - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int verMin = -mvstepHalf;

  int offsetX = (pos.x << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getHor();
  int offsetY = (pos.y << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getVer();
  bool isOOB = false;
  if ((offsetX <= horMin)
    || ((offsetX + ((size.width - 1) << MV_FRACTIONAL_BITS_INTERNAL) ) >= horMax)
    || (offsetY <= verMin)
    || ((offsetY + ((size.height - 1) << MV_FRACTIONAL_BITS_INTERNAL)) >= verMax))
  {
    isOOB = true;
  }
  if (isOOB)
  {
    int baseOffsetX = offsetX;
    bool *pMcMask = mcMask;
    for (int y = 0; y < size.height; y++, offsetY += mvstep)
    {
      offsetX = baseOffsetX;
      bool checkY = (offsetY <= verMin) || (offsetY >= verMax);;
      for (int x = 0; x < size.width; x++, offsetX += mvstep)
      {
        pMcMask[x] = (offsetX <= horMin) || (offsetX >= horMax) || checkY;
      }
      pMcMask += mcStride;
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma = mcMaskChroma;
      pMcMask = mcMask;
      int widthChroma = (size.width) >> chromaScale;
      int heightChroma = (size.height) >> chromaScale;
      int strideLuma2 = mcStride << chromaScale;
      for (int y = 0; y < heightChroma; y++)
      {
        for (int x = 0; x < widthChroma; x++)
        {
          pMcMaskChroma[x] = pMcMask[x << chromaScale];
        }
        pMcMaskChroma += mcCStride;
        pMcMask += strideLuma2;
      }
    }
  }
  else
  {
    bool *pMcMask = mcMask;
    for (int y = 0; y < size.height; y++)
    {
      memset(pMcMask, false, size.width);
      pMcMask += mcStride;
    }

    bool *pMcMaskChroma = mcMaskChroma;
    int widthChroma = (size.width) >> chromaScale;
    int heightChroma = (size.height) >> chromaScale;
    for (int y = 0; y < heightChroma; y++)
    {
      memset(pMcMaskChroma, false, widthChroma);
      pMcMaskChroma += mcCStride;
    }
  }
  return isOOB;
}
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
void computeDeltaAndShiftCore(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri)
{
  for (int i = 0; i < int(mvpInfoVecOri.size()); i++)
  {
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
    mvpInfoVecOri[i].mvp.hor = mvpInfoVecOri[i].mvp.hor >= 0 ? mvpInfoVecOri[i].mvp.hor << 2 : -(-mvpInfoVecOri[i].mvp.hor << 2);
    mvpInfoVecOri[i].mvp.ver = mvpInfoVecOri[i].mvp.ver >= 0 ? mvpInfoVecOri[i].mvp.ver << 2 : -(-mvpInfoVecOri[i].mvp.ver << 2);
#endif
    mvpInfoVecOri[i].pos.x = mvpInfoVecOri[i].pos.x - posLT.x;
    mvpInfoVecOri[i].pos.y = mvpInfoVecOri[i].pos.y - posLT.y;
    mvpInfoVecOri[i].mvp.set(mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor(), mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer());
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    mvpInfoVecOri[i].mvp.set(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.hor), Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.ver));
#endif
  }
}
void computeDeltaAndShiftCoreAddi(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri, std::vector<RMVFInfo> &mvpInfoVecRes)
{
  int offset = (int)mvpInfoVecRes.size();
  for (int i = 0; i < int(mvpInfoVecOri.size()); i++)
  {
    mvpInfoVecRes.push_back(RMVFInfo());
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
    mvpInfoVecOri[i].mvp.hor = mvpInfoVecOri[i].mvp.hor >= 0 ? mvpInfoVecOri[i].mvp.hor << 2 : -(-mvpInfoVecOri[i].mvp.hor << 2);
    mvpInfoVecOri[i].mvp.ver = mvpInfoVecOri[i].mvp.ver >= 0 ? mvpInfoVecOri[i].mvp.ver << 2 : -(-mvpInfoVecOri[i].mvp.ver << 2);
#endif
    mvpInfoVecRes[offset + i].pos.x = mvpInfoVecOri[i].pos.x - posLT.x;
    mvpInfoVecRes[offset + i].pos.y = mvpInfoVecOri[i].pos.y - posLT.y;
    mvpInfoVecRes[offset + i].mvp.set(mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor(), mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer());
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    mvpInfoVecRes[offset + i].mvp.set(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecRes[offset + i].mvp.hor), Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecRes[offset + i].mvp.ver));
#endif
  }
}
void buildRegressionMatrixCore(std::vector<RMVFInfo> &mvpInfoVecOri, 
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int64_t sumbb[2][3][3], int64_t sumeb[2][3],
#else
  int sumbb[2][3][3], int sumeb[2][3],
#endif
  uint16_t addedSize)
{
  int iNum = (int)mvpInfoVecOri.size();
  int b[3];
  int e[2];
  for (int ni = addedSize ? iNum - addedSize : 0; ni < iNum; ni++)//for all neighbor PUs
  {
    // to avoid big values in matrix, it is better to use delta_x and delta_y value, ie.e. use the x,y with respect to the top,left corner of current PU
    b[0] = mvpInfoVecOri[ni].pos.x;
    b[1] = mvpInfoVecOri[ni].pos.y;
    b[2] = 1;

    e[0] = mvpInfoVecOri[ni].mvp.getHor();
    e[1] = mvpInfoVecOri[ni].mvp.getVer();

    for (int c = 0; c < 2; c++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumeb[c][d] += (e[c] * b[d]);
      }
      for (int d1 = 0; d1 < 3; d1++)
      {
        for (int d = 0; d < 3; d++)
        {
          sumbb[c][d1][d] += (b[d1] * b[d]);
        }
      }
    }
  }
}
#endif
PelBufferOps::PelBufferOps()
{
#if JVET_W0097_GPM_MMVD_TM
  roundBD = roundBDCore;
  weightedAvg = weightedAvgCore;
  copyClip = copyClipCore;
#endif
  addAvg4 = addAvgCore<Pel>;
  addAvg8 = addAvgCore<Pel>;
#if JVET_AE0169_BIPREDICTIVE_IBC
  avg = avgCore<Pel>;
#endif
#if JVET_AD0213_LIC_IMP
  toLast2 = toLastCore<Pel>;
  toLast4 = toLastCore<Pel>;
  licRemoveWeightHighFreq2 = licRemoveWeightHighFreqCore<Pel>;
  licRemoveWeightHighFreq4 = licRemoveWeightHighFreqCore<Pel>;
#endif
  reco4 = reconstructCore<Pel>;
  reco8 = reconstructCore<Pel>;

  linTf4 = linTfCore<Pel>;
  linTf8 = linTfCore<Pel>;

  addBIOAvg4      = addBIOAvgCore;
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
  calcBIOParameterHighPrecision   = calcBIOParameterCoreHighPrecision;
  calcBIOParamSum4HighPrecision   = calcBIOParamSum4CoreHighPrecision;
#if JVET_AG0067_DMVR_EXTENSIONS
  calcBIOParamSum4HighPrecision4  = calcBIOParamSum4CoreHighPrecision;
  calcBIOParamSum4HighPrecision8  = calcBIOParamSum4CoreHighPrecision;
  calcBIOParamSum4HighPrecision16 = calcBIOParamSum4CoreHighPrecision;
#endif
#endif
#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
  calcBIOParameter   = calcBIOParameterCore;
  calcBIOParamSum5   = calcBIOParamSum5Core;
#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
  calcBIOParamSum5NOSIM4     = calcBIOParamSum5NOSIMCore;
  calcBIOParamSum5NOSIM8     = calcBIOParamSum5NOSIMCore;
#endif
  calcBIOParamSum4   = calcBIOParamSum4Core;
  calcBIOClippedVxVy = calcBIOClippedVxVyCore;
  addBIOAvgN         = addBIOAvgNCore;
  calAbsSum          = calAbsSumCore;
  bioGradFilter      = gradFilterCore <false>;
#else
  bioGradFilter   = gradFilterCore;
#endif
  calcBIOSums = calcBIOSumsCore;

  copyBuffer = copyBufferCore;
  padding = paddingCore;
#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  removeWeightHighFreq8 = removeWeightHighFreq;
  removeWeightHighFreq4 = removeWeightHighFreq;
  removeHighFreq8 = removeHighFreq;
  removeHighFreq4 = removeHighFreq;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  removeWeightHighFreq1 = removeWeightHighFreq;
  removeHighFreq1 = removeHighFreq;
#endif
#endif

  profGradFilter = gradFilterCore <false>;
  applyPROF      = applyPROFCore;
  roundIntVector = nullptr;
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  getSumOfDifference = getSumOfDifferenceCore;
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  getAbsoluteDifferencePerSample = getAbsoluteDifferencePerSampleCore;
  getSampleSumFunc[0] = getMaskedSampleSumCore<0>;
  getSampleSumFunc[1] = getMaskedSampleSumCore<1>;
  getSampleSumFunc[2] = getMaskedSampleSumCore<2>;
  getSampleSumFunc[3] = getMaskedSampleSumCore<3>;
#endif
#if JVET_Z0136_OOB
  isMvOOB = isMvOOBCore;
  isMvOOBSubBlk = isMvOOBSubBlkCore;
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  computeDeltaAndShift = computeDeltaAndShiftCore;
  computeDeltaAndShiftAddi = computeDeltaAndShiftCoreAddi;
  buildRegressionMatrix = buildRegressionMatrixCore;
#endif
}

PelBufferOps g_pelBufOP = PelBufferOps();

void copyBufferCore(Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height)
{
  int numBytes = width * sizeof(Pel);
  for (int i = 0; i < height; i++)
  {
    memcpy(dst + i * dstStride, src + i * srcStride, numBytes);
  }
}

void paddingCore(Pel *ptr, int stride, int width, int height, int padSize)
{
  /*left and right padding*/
  Pel *ptrTemp1 = ptr;
  Pel *ptrTemp2 = ptr + (width - 1);
  int offset = 0;
  for (int i = 0; i < height; i++)
  {
    offset = stride * i;
    for (int j = 1; j <= padSize; j++)
    {
      *(ptrTemp1 - j + offset) = *(ptrTemp1 + offset);
      *(ptrTemp2 + j + offset) = *(ptrTemp2 + offset);
    }
  }
  /*Top and Bottom padding*/
  int numBytes = (width + padSize + padSize) * sizeof(Pel);
  ptrTemp1 = (ptr - padSize);
  ptrTemp2 = (ptr + (stride * (height - 1)) - padSize);
  for (int i = 1; i <= padSize; i++)
  {
    memcpy(ptrTemp1 - (i * stride), (ptrTemp1), numBytes);
    memcpy(ptrTemp2 + (i * stride), (ptrTemp2), numBytes);
  }
}

#if MULTI_HYP_PRED
template<>
void AreaBuf<Pel>::addHypothesisAndClip(const AreaBuf<const Pel> &other, const int weight, const ClpRng& clpRng)
{
  CHECK(width != other.width, "Incompatible size");
  CHECK(height != other.height, "Incompatible size");

  Pel* dest = buf;
  const Pel* src = other.buf;
  const int counterweight = (1 << MULTI_HYP_PRED_WEIGHT_BITS) - weight;
  const int add = 1 << (MULTI_HYP_PRED_WEIGHT_BITS - 1);

#define ADD_HYP_OP( ADDR ) dest[ADDR] = ClipPel( ( counterweight*dest[ADDR] + weight*src[ADDR] + add ) >> MULTI_HYP_PRED_WEIGHT_BITS, clpRng )
#define ADD_HYP_INC     \
    dest += stride; \
    src += other.stride;

  SIZE_AWARE_PER_EL_OP(ADD_HYP_OP, ADD_HYP_INC);

#undef ADD_HYP_OP
#undef ADD_HYP_INC
}
#endif

template<>
#if JVET_Z0136_OOB
void AreaBuf<Pel>::addWeightedAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng, const int8_t bcwIdx, bool *mcMask[2], int mcStride, bool* isOOB)
#else
void AreaBuf<Pel>::addWeightedAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng, const int8_t bcwIdx)
#endif
{
#if JVET_W0097_GPM_MMVD_TM
#if JVET_Z0136_OOB
  int8_t w0 = getBcwWeight(bcwIdx, REF_PIC_LIST_0);
  int8_t w1 = getBcwWeight(bcwIdx, REF_PIC_LIST_1);

  const int8_t log2WeightBase = g_bcwLog2WeightBase;
  const Pel* src1 = other1.buf;
  const Pel* src2 = other2.buf;
  Pel* dest = buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride = stride;
  const int clipbd = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
#else
  const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
#endif
  const int offset = (1 << (shiftNum - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
  if (!isOOB[0] && !isOOB[1])
  {
    g_pelBufOP.weightedAvg(src1, src1Stride, src2, src2Stride, dest, destStride, w0, w1, width, height, clpRng);
  }
  else
  {
    int shiftNum2 = IF_INTERNAL_FRAC_BITS(clipbd);
    const int offset2 = (1 << (shiftNum2 - 1)) + IF_INTERNAL_OFFS;
    bool *pMcMask0 = mcMask[0];
    bool *pMcMask1 = mcMask[1];

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dest[x] = ClipPel(rightShift(src2[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dest[x] = ClipPel(rightShift(src1[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          dest[x + 0] = ClipPel(rightShift((src1[x] * w0 + src2[x + 0] * w1 + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
#else
  const int8_t w0 = getBcwWeight(bcwIdx, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwIdx, REF_PIC_LIST_1);

  const Pel*            src0 = other1.buf;
  const Pel*            src1 = other2.buf;
  Pel*                 dest = buf;
  const unsigned src0Stride = other1.stride;
  const unsigned src1Stride = other2.stride;
  const unsigned destStride = stride;

  g_pelBufOP.weightedAvg(src0, src0Stride, src1, src1Stride, dest, destStride, w0, w1, width, height, clpRng);
#endif
#else
  const int8_t w0 = getBcwWeight(bcwIdx, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwIdx, REF_PIC_LIST_1);
  const int8_t log2WeightBase = g_bcwLog2WeightBase;

  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
  Pel* dest = buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride = stride;
  const int clipbd = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
#else
  const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
#endif
  const int offset = (1 << (shiftNum - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR]*w0 + src2[ADDR]*w1 + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

  SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
#endif
}

template<>
void AreaBuf<Pel>::rspSignal(std::vector<Pel>& pLUT)
{
  Pel* dst = buf;
  Pel* src = buf;
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        dst[x] = pLUT[src[x]];
      }
      dst += stride;
      src += stride;
    }
}

template<>
void AreaBuf<Pel>::rspSignal(const AreaBuf<const Pel>& other, std::vector<Pel>& pLUT)
{
  CHECK( width != other.width, "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

  Pel* dst = buf;
  const Pel* src = other.buf;
  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
    {
      dst[x] = pLUT[src[x]];
    }
    dst += stride;
    src += other.stride;
  }
}

template<>
void AreaBuf<Pel>::rspSignal( const AreaBuf<Pel> &toReshape, std::vector<Pel>& pLUT )
{
  CHECK( width != toReshape.width, "Incompatible size" );
  CHECK( height != toReshape.height, "Incompatible size" );

  Pel* dst = buf;
  Pel* src = toReshape.buf;
  const int srcStride = toReshape.stride;

  for( unsigned y = 0; y < height; y++ )
  {
    for( unsigned x = 0; x < width; x++ )
    {
      dst[x] = pLUT[src[x]];
    }
    dst += stride;
    src += srcStride;
  }
}

#if JVET_AA0070_RRIBC
template<>
void AreaBuf<Pel>::flipSignal(bool isFlipHor)
{
  Pel *tempPel;
  Size tSize(width, height);
  tempPel       = new Pel[tSize.area()];
  PelBuf tmpBuf = PelBuf(tempPel, tSize);
  copyBufferCore(buf, stride, tmpBuf.buf, tmpBuf.stride, tmpBuf.width, tmpBuf.height);

  Pel *dstbuf = buf;
  Pel *srcbuf = tmpBuf.buf;
  if (isFlipHor)
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        dstbuf[x] = srcbuf[width - 1 - x];
      }
      dstbuf += stride;
      srcbuf += tmpBuf.stride;
    }
  }
  else
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        dstbuf[x] = srcbuf[(height - 1 - y) * tmpBuf.stride + x];
      }
      dstbuf += stride;
    }
  }

  delete[] tempPel;
}
#endif

template<>
void AreaBuf<Pel>::rspSignalAllAndSubtract( const AreaBuf<Pel> &buffer1, const AreaBuf<Pel> &buffer2, std::vector<Pel>& pLUT )
{
  CHECK( width != buffer1.width, "Incompatible size in buffer1" );
  CHECK( height != buffer1.height, "Incompatible size in buffer1" );
  CHECK( width != buffer2.width, "Incompatible size in buffer2" );
  CHECK( height != buffer2.height, "Incompatible size in buffer2" );

  Pel* dest = buf;
  const Pel* buf1 = buffer1.buf;
  const Pel* buf2 = buffer2.buf;

#define SUBS_INC           \
  dest +=          stride; \
  buf1 +=  buffer1.stride; \
  buf2 +=  buffer2.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] = pLUT[buf1[ADDR]] - pLUT[buf2[ADDR]]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<>
void AreaBuf<Pel>::rspSignalAndSubtract( const AreaBuf<Pel> &buffer1, const AreaBuf<Pel> &buffer2, std::vector<Pel>& pLUT )
{
  CHECK( width != buffer1.width, "Incompatible size in buffer1" );
  CHECK( height != buffer1.height, "Incompatible size in buffer1" );
  CHECK( width != buffer2.width, "Incompatible size in buffer2" );
  CHECK( height != buffer2.height, "Incompatible size in buffer2" );

  Pel* dest = buf;
  const Pel* buf1 = buffer1.buf;
  const Pel* buf2 = buffer2.buf;

#define SUBS_INC           \
  dest +=          stride; \
  buf1 +=  buffer1.stride; \
  buf2 +=  buffer2.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] = pLUT[buf1[ADDR]] - buf2[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<>
void AreaBuf<Pel>::scaleSignal(const int scale, const bool dir, const ClpRng& clpRng)
{
  Pel* dst = buf;
  Pel* src = buf;
  int sign, absval;
  int maxAbsclipBD = (1<<clpRng.bd) - 1;

  if (dir) // forward
  {
    if (width == 1)
    {
      THROW("Blocks of width = 1 not supported");
    }
    else
    {
      for (unsigned y = 0; y < height; y++)
      {
        for (unsigned x = 0; x < width; x++)
        {
          sign = src[x] >= 0 ? 1 : -1;
          absval = sign * src[x];
          dst[x] = (Pel)Clip3(-maxAbsclipBD, maxAbsclipBD, sign * (((absval << CSCALE_FP_PREC) + (scale >> 1)) / scale));
        }
        dst += stride;
        src += stride;
      }
    }
  }
  else // inverse
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        dst[x] = Reshape::scalePel(src[x], scale, maxAbsclipBD);
      }
      dst += stride;
      src += stride;
    }
  }
}

template<>
#if JVET_Z0136_OOB
void AreaBuf<Pel>::addAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool* isOOB)
#else
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng)
#endif
{
  const Pel* src0 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride =        stride;
  const int     clipbd      = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd) + 1;
#else
  const int     shiftNum    = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
#endif
  const int     offset      = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;

#if JVET_Z0136_OOB
  if (mcMask == NULL || (!isOOB[0] && !isOOB[1]))
  {
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
    if ((width & 7) == 0)
    {
      g_pelBufOP.addAvg8(src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng, mcMask, mcStride, isOOB);
    }
    else if ((width & 3) == 0)
    {
      g_pelBufOP.addAvg4(src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng, mcMask, mcStride, isOOB);
    }
    else
#endif
    {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR] + src2[ADDR] + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
    }
  }
  else
  {
    int shiftNum2 = IF_INTERNAL_FRAC_BITS(clipbd);
    const int offset2 = (1 << (shiftNum2 - 1)) + IF_INTERNAL_OFFS;
    bool *pMcMask0 = mcMask[0];
    bool *pMcMask1 = mcMask[1];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dest[x] = ClipPel(rightShift(src2[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dest[x] = ClipPel(rightShift(src0[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          dest[x] = ClipPel(rightShift((src0[x] + src2[x] + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src0 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
#else
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.addAvg8( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.addAvg4( src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng );
  }
  else
#endif
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( ( src0[ADDR] + src2[ADDR] + offset ), shiftNum ), clpRng )
#define ADD_AVG_INC     \
    src0 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
#endif
}

#if JVET_AE0169_BIPREDICTIVE_IBC
template<>
void AreaBuf<Pel>::avg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2)
{
  const Pel* src1 = other1.buf;
  const Pel* src2 = other2.buf;
        Pel* dest =        buf;

  const unsigned src1Stride = other1.stride;
  const unsigned src2Stride = other2.stride;
  const unsigned destStride =        stride;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if ((width & 3) == 0)
  {
    g_pelBufOP.avg(src1, src1Stride, src2, src2Stride, dest, destStride, width, height);
  }
  else
#endif
  {
#define ADD_AVG_OP( ADDR ) dest[ADDR] = rightShift( ( src1[ADDR] + src2[ADDR] + 1 ), 1 )
#define ADD_AVG_INC     \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( ADD_AVG_OP, ADD_AVG_INC );

#undef ADD_AVG_OP
#undef ADD_AVG_INC
  }
}
#endif

template<>
void AreaBuf<Pel>::toLast( const ClpRng& clpRng )
{
        Pel* src       = buf;
  const uint32_t srcStride = stride;

  const int  clipbd    = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd);
#else
  const int  shiftNum  = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
#endif
  const int  offset    = ( 1 << ( shiftNum - 1 ) ) + IF_INTERNAL_OFFS;

  if (width == 1)
  {
#if JVET_AD0213_LIC_IMP
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        src[x] = ClipPel(rightShift((src[x] + offset), shiftNum), clpRng);
      }
      src += srcStride;
    }
#else
    THROW( "Blocks of width = 1 not supported" );
#endif
  }
#if JVET_AD0213_LIC_IMP
  else if ((width & 3) == 0)
  {
    g_pelBufOP.toLast4(src, srcStride, width, height, shiftNum, offset, clpRng);
  }
  else if ((width & 1) == 0)
  {
    g_pelBufOP.toLast2(src, srcStride, width, height, shiftNum, offset, clpRng);
  }
  else
  {
    THROW("Unsupported size!");
  }
#else
  else if (width&2)
  {
    for ( int y = 0; y < height; y++ )
    {
      for (int x=0 ; x < width; x+=2 )
      {
        src[x + 0] = ClipPel( rightShift( ( src[x + 0] + offset ), shiftNum ), clpRng );
        src[x + 1] = ClipPel( rightShift( ( src[x + 1] + offset ), shiftNum ), clpRng );
      }
      src += srcStride;
    }
  }
  else
  {
    for ( int y = 0; y < height; y++ )
    {
      for (int x=0 ; x < width; x+=4 )
      {
        src[x + 0] = ClipPel( rightShift( ( src[x + 0] + offset ), shiftNum ), clpRng );
        src[x + 1] = ClipPel( rightShift( ( src[x + 1] + offset ), shiftNum ), clpRng );
        src[x + 2] = ClipPel( rightShift( ( src[x + 2] + offset ), shiftNum ), clpRng );
        src[x + 3] = ClipPel( rightShift( ( src[x + 3] + offset ), shiftNum ), clpRng );

      }
      src += srcStride;
    }
  }
#endif
}


template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel> &src, const ClpRng& clpRng )
{
  const Pel* srcp = src.buf;
        Pel* dest =     buf;

  const unsigned srcStride  = src.stride;
  const unsigned destStride = stride;

#if !JVET_W0090_ARMC_TM && !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if( width == 1 )
  {
    THROW( "Blocks of width = 1 not supported" );
  }
  else
#endif
  {
#if JVET_W0097_GPM_MMVD_TM
    g_pelBufOP.copyClip(srcp, srcStride, dest, destStride, width, height, clpRng);
#else
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( srcp[ADDR], clpRng )
#define RECO_INC        \
    srcp += srcStride;  \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
#endif
  }
}

template<>
void AreaBuf<Pel>::roundToOutputBitdepth( const AreaBuf<const Pel> &src, const ClpRng& clpRng )
{
  const Pel* srcp = src.buf;
        Pel* dest =     buf;
  const unsigned srcStride  = src.stride;
  const unsigned destStride = stride;
#if !JVET_W0097_GPM_MMVD_TM
  const int32_t clipbd            = clpRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int32_t shiftDefault      = IF_INTERNAL_FRAC_BITS(clipbd);
#else
  const int32_t shiftDefault      = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
#endif
  const int32_t offsetDefault     = (1<<(shiftDefault-1)) + IF_INTERNAL_OFFS;
#endif
  if( width == 1 )
  {
    THROW( "Blocks of width = 1 not supported" );
  }
  else
  {
#if JVET_W0097_GPM_MMVD_TM
    g_pelBufOP.roundBD(srcp, srcStride, dest, destStride, width, height, clpRng);
#else
#define RND_OP( ADDR ) dest[ADDR] = ClipPel( rightShift( srcp[ADDR] + offsetDefault, shiftDefault), clpRng )
#define RND_INC        \
    srcp += srcStride;  \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RND_OP, RND_INC );

#undef RND_OP
#undef RND_INC
#endif
  }
}


template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng )
{
  const Pel* src1 = pred.buf;
  const Pel* src2 = resi.buf;
        Pel* dest =      buf;

  const unsigned src1Stride = pred.stride;
  const unsigned src2Stride = resi.stride;
  const unsigned destStride =      stride;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.reco8( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.reco4( src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng );
  }
  else
#endif
  {
#define RECO_OP( ADDR ) dest[ADDR] = ClipPel( src1[ADDR] + src2[ADDR], clpRng )
#define RECO_INC        \
    src1 += src1Stride; \
    src2 += src2Stride; \
    dest += destStride; \

    SIZE_AWARE_PER_EL_OP( RECO_OP, RECO_INC );

#undef RECO_OP
#undef RECO_INC
  }
}

#if JVET_AE0078_IBC_LIC_EXTENSION
template<>
void AreaBuf<Pel>::linearTransforms(const int scale, const int shift, const int offset, const int scale2, const int shift2, const int offset2, const int yThres, bool bClip, const ClpRng& clpRng)
{
  const Pel* src = buf;
  Pel* dst = buf;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      if (src[j] <= yThres)
      {
        dst[j] = (Pel)ClipPel(((scale * src[j]) >> shift) + offset, clpRng);
      }
      else
      {
        dst[j] = (Pel)ClipPel(((scale2 * src[j]) >> shift2) + offset2, clpRng);
      }
    }
    src += stride;
    dst += stride;
  }
}
#endif

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  const Pel* src = buf;
        Pel* dst = buf;
#if JVET_AG0276_NLIC
  const uint32_t areaT = area();
#endif

#if JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if (width == 0)
  {
    THROW("Blocks of width = 0 not supported");
  }
#else
  if( width == 1 )
  {
    THROW( "Blocks of width = 1 not supported" );
  }
#endif
#if JVET_AG0276_NLIC
  else if (width == stride && (areaT & 7) == 0)
  {
    g_pelBufOP.linTf8(src, areaT, dst, areaT, areaT, 1, scale, shift, offset, clpRng, bClip);
  }
  else if (width == stride && (areaT & 3) == 0)
  {
    g_pelBufOP.linTf4(src, areaT, dst, areaT, areaT, 1, scale, shift, offset, clpRng, bClip);
  }
#endif
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  else if( ( width & 7 ) == 0 )
  {
    g_pelBufOP.linTf8( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
  else if( ( width & 3 ) == 0 )
  {
    g_pelBufOP.linTf4( src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip );
  }
#endif
  else
  {
#define LINTF_OP( ADDR ) dst[ADDR] = ( Pel ) bClip ? ClipPel( rightShift( scale * src[ADDR], shift ) + offset, clpRng ) : ( rightShift( scale * src[ADDR], shift ) + offset )
#define LINTF_INC        \
    src += stride;       \
    dst += stride;       \

    SIZE_AWARE_PER_EL_OP( LINTF_OP, LINTF_INC );

#undef LINTF_OP
#undef LINTF_INC
  }
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<>
void AreaBuf<Pel>::subtract( const Pel val )
{
  ClpRng clpRngDummy;
  linearTransform( 1, 0, -val, false, clpRngDummy );
}
#endif


PelStorage::PelStorage()
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_origin[i] = nullptr;
  }
}

PelStorage::~PelStorage()
{
  destroy();
}

void PelStorage::create( const UnitArea &_UnitArea )
{
  create( _UnitArea.chromaFormat, _UnitArea.blocks[0] );
}

void PelStorage::create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize, const unsigned _margin, const unsigned _alignment, const bool _scaleChromaMargin )
{
  CHECK( !bufs.empty(), "Trying to re-create an already initialized buffer" );

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  bool      lumaEmpty   = _chromaFormat == CHROMA_ONLY_420 ? true : false;
  chromaFormat          = _chromaFormat == CHROMA_ONLY_420 ? CHROMA_420 : _chromaFormat;
#else
  chromaFormat = _chromaFormat;
#endif

  const uint32_t numCh = getNumberValidComponents( _chromaFormat );

  unsigned extHeight = _area.height;
  unsigned extWidth  = _area.width;

  if( _maxCUSize )
  {
    extHeight = ( ( _area.height + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
    extWidth  = ( ( _area.width  + _maxCUSize - 1 ) / _maxCUSize ) * _maxCUSize;
  }

  for( uint32_t i = 0; i < numCh; i++ )
  {
    const ComponentID compID = ComponentID( i );
    const unsigned scaleX = ::getComponentScaleX( compID, chromaFormat );
    const unsigned scaleY = ::getComponentScaleY( compID, chromaFormat );
    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth  >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin?scaleY:0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin?scaleX:0);
    unsigned totalWidth   = scaledWidth + 2*xmargin;
    unsigned totalHeight  = scaledHeight +2*ymargin;

    if( _alignment )
    {
      // make sure buffer lines are align
      CHECK( _alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment" );
      totalWidth = ( ( totalWidth + _alignment - 1 ) / _alignment ) * _alignment;
    }
    uint32_t area = totalWidth * totalHeight;
    CHECK( !area, "Trying to create a buffer with zero area" );

#if JVET_AI0084_ALF_RESIDUALS_SCALING
    Pel* topLeft  = nullptr;
    if ( lumaEmpty && i == 0 ) 
    {
      m_origin[i]   = (Pel*)xMalloc( Pel, 1 );  // this component is dummy
    }
    else
    {
      m_origin[i] = (Pel*)xMalloc( Pel, area );
      topLeft     = m_origin[i] + totalWidth * ymargin + xmargin;
    }
#else
    m_origin[i] = ( Pel* ) xMalloc( Pel, area );
    Pel* topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
#endif
    bufs.push_back( PelBuf( topLeft, totalWidth, _area.width >> scaleX, _area.height >> scaleY ) );
  }
}

void PelStorage::createFromBuf( PelUnitBuf buf )
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  bufs.resize(numCh);

  for( uint32_t i = 0; i < numCh; i++ )
  {
    PelBuf cPelBuf = buf.get( ComponentID( i ) );
    bufs[i] = PelBuf( cPelBuf.bufAt( 0, 0 ), cPelBuf.stride, cPelBuf.width, cPelBuf.height );
  }
}

void PelStorage::swap( PelStorage& other )
{
  const uint32_t numCh = ::getNumberValidComponents( chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    // check this otherwise it would turn out to get very weird
    CHECK( chromaFormat                   != other.chromaFormat                  , "Incompatible formats" );
    CHECK( get( ComponentID( i ) )        != other.get( ComponentID( i ) )       , "Incompatible formats" );
    CHECK( get( ComponentID( i ) ).stride != other.get( ComponentID( i ) ).stride, "Incompatible formats" );

    std::swap( bufs[i].buf,    other.bufs[i].buf );
    std::swap( bufs[i].stride, other.bufs[i].stride );
    std::swap( m_origin[i],    other.m_origin[i] );
  }
}

void PelStorage::destroy()
{
  chromaFormat = NUM_CHROMA_FORMAT;
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( m_origin[i] )
    {
      xFree( m_origin[i] );
      m_origin[i] = nullptr;
    }
  }
  bufs.clear();
}

PelBuf PelStorage::getBuf( const ComponentID CompID )
{
  return bufs[CompID];
}

const CPelBuf PelStorage::getBuf( const ComponentID CompID ) const
{
  return bufs[CompID];
}

PelBuf PelStorage::getBuf( const CompArea &blk )
{
  const PelBuf& r = bufs[blk.compID];

  CHECKD( rsAddr( blk.bottomRight(), r.stride ) >= ( ( r.height - 1 ) * r.stride + r.width ), "Trying to access a buf outside of bound!" );

  return PelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

const CPelBuf PelStorage::getBuf( const CompArea &blk ) const
{
  const PelBuf& r = bufs[blk.compID];
  return CPelBuf( r.buf + rsAddr( blk, r.stride ), r.stride, blk );
}

PelUnitBuf PelStorage::getBuf( const UnitArea &unit )
{
  return ( chromaFormat == CHROMA_400 ) ? PelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : PelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

const CPelUnitBuf PelStorage::getBuf( const UnitArea &unit ) const
{
  return ( chromaFormat == CHROMA_400 ) ? CPelUnitBuf( chromaFormat, getBuf( unit.Y() ) ) : CPelUnitBuf( chromaFormat, getBuf( unit.Y() ), getBuf( unit.Cb() ), getBuf( unit.Cr() ) );
}

template<>
void UnitBuf<Pel>::colorSpaceConvert(const UnitBuf<Pel> &other, const bool forward, const ClpRng& clpRng)
{
  const Pel* pOrg0 = bufs[COMPONENT_Y].buf;
  const Pel* pOrg1 = bufs[COMPONENT_Cb].buf;
  const Pel* pOrg2 = bufs[COMPONENT_Cr].buf;
  const int  strideOrg = bufs[COMPONENT_Y].stride;

  Pel* pDst0 = other.bufs[COMPONENT_Y].buf;
  Pel* pDst1 = other.bufs[COMPONENT_Cb].buf;
  Pel* pDst2 = other.bufs[COMPONENT_Cr].buf;
  const int strideDst = other.bufs[COMPONENT_Y].stride;

  int width = bufs[COMPONENT_Y].width;
  int height = bufs[COMPONENT_Y].height;
  int maxAbsclipBD = (1 << (clpRng.bd + 1)) - 1;
  int r, g, b;
  int y0, cg, co;

  CHECK(bufs[COMPONENT_Y].stride != bufs[COMPONENT_Cb].stride || bufs[COMPONENT_Y].stride != bufs[COMPONENT_Cr].stride, "unequal stride for 444 content");
  CHECK(other.bufs[COMPONENT_Y].stride != other.bufs[COMPONENT_Cb].stride || other.bufs[COMPONENT_Y].stride != other.bufs[COMPONENT_Cr].stride, "unequal stride for 444 content");
  CHECK(bufs[COMPONENT_Y].width != other.bufs[COMPONENT_Y].width || bufs[COMPONENT_Y].height != other.bufs[COMPONENT_Y].height, "unequal block size")

    if (forward)
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          r = pOrg2[x];
          g = pOrg0[x];
          b = pOrg1[x];

          co = r - b;
          int t = b + (co >> 1);
          cg = g - t;
          pDst0[x] = t + (cg >> 1);
          pDst1[x] = cg;
          pDst2[x] = co;
        }
        pOrg0 += strideOrg;
        pOrg1 += strideOrg;
        pOrg2 += strideOrg;
        pDst0 += strideDst;
        pDst1 += strideDst;
        pDst2 += strideDst;
      }
    }
    else
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          y0 = pOrg0[x];
          cg = pOrg1[x];
          co = pOrg2[x];

          y0 = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, y0);
          cg = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, cg);
          co = Clip3((-maxAbsclipBD - 1), maxAbsclipBD, co);

          int t = y0 - (cg >> 1);
          pDst0[x] = cg + t;
          pDst1[x] = t - (co >> 1);
          pDst2[x] = co + pDst1[x];
        }

        pOrg0 += strideOrg;
        pOrg1 += strideOrg;
        pOrg2 += strideOrg;
        pDst0 += strideDst;
        pDst1 += strideDst;
        pDst2 += strideDst;
      }
    }
}
