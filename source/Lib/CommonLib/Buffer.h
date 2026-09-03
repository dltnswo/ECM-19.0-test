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

/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#ifndef __BUFFER__
#define __BUFFER__

#include "Common.h"
#include "CommonDef.h"
#include "ChromaFormat.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <typeinfo>

// ---------------------------------------------------------------------------
// AreaBuf struct
// ---------------------------------------------------------------------------

struct PelBufferOps
{
  PelBufferOps();

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  void initPelBufOpsX86();
  template<X86_VEXT vext>
  void _initPelBufOpsX86();
#endif
#if JVET_W0097_GPM_MMVD_TM
  void(*roundBD)       (const Pel* srcp, const int srcStride, Pel* dest, const int destStride, int width, int height, const ClpRng& clpRng);
  void(*weightedAvg)   (const Pel* src0, const unsigned src0Stride, const Pel* src1, const unsigned src1Stride, Pel* dest, const unsigned destStride, const int8_t w0, const int8_t w1, int width, int height, const ClpRng& clpRng);
  void(*copyClip)      (const Pel* srcp, const unsigned srcStride, Pel* dest, const unsigned destStride, int width, int height, const ClpRng& clpRng);
#endif
#if JVET_Z0136_OOB
  void(*addAvg4)       (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool * isOOB);
  void(*addAvg8)       (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool * isOOB);
#else
  void ( *addAvg4 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,            int shift, int offset, const ClpRng& clpRng );
  void ( *addAvg8 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,            int shift, int offset, const ClpRng& clpRng );
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  void ( *avg )        (const Pel* src1, int src1Stride, const Pel* src2, int src2Stride, Pel *dst, int dstStride, int width, int height);
#endif
#if JVET_AD0213_LIC_IMP
  void(*toLast2)       (Pel* src, int srcStride, int width, int height, int shiftNum, int offset, const ClpRng& clpRng);
  void(*toLast4)       (Pel* src, int srcStride, int width, int height, int shiftNum, int offset, const ClpRng& clpRng);
  void(*licRemoveWeightHighFreq2) (Pel* src0, Pel* src1, Pel* dst, int length, int w0, int w1, int offset, const ClpRng& clpRng);
  void(*licRemoveWeightHighFreq4) (Pel* src0, Pel* src1, Pel* dst, int length, int w0, int w1, int offset, const ClpRng& clpRng);
#endif
  void ( *reco4 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *reco8 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *linTf4 )        ( const Pel* src0, int src0Stride,                                  Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip );
  void ( *linTf8 )        ( const Pel* src0, int src0Stride,                                  Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip );
  void(*addBIOAvg4)    (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng);
  void(*bioGradFilter) (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth);
  void(*calcBIOPar)    (const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, const int bitDepth);
#if JVET_AD0195_HIGH_PRECISION_BDOF_CORE
  void(*calcBIOParameterHighPrecision)   (const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int width, int height, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, Pel* dI
#if JVET_AG0067_DMVR_EXTENSIONS
                                          ,Pel* gX, Pel* gY
#endif
                                          );
  void(*calcBIOParamSum4HighPrecision)   (int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, int width, int height, const int widthG, int32_t* sumS1, int32_t* sumS2, int32_t* sumS3, int32_t* sumS5, int32_t* sumS6
#if JVET_AG0067_DMVR_EXTENSIONS
                                          ,Pel* dI ,Pel* gX, Pel* gY, bool isGPM, bool isSub
#endif
                                          );
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
  void(*calcBIOParamSum4HighPrecision4)   (int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, int width, int height, const int widthG, int32_t* sumS1, int32_t* sumS2, int32_t* sumS3, int32_t* sumS5, int32_t* sumS6, Pel* dI, Pel* gX, Pel* gY, bool isGPM, bool isSub);
  void(*calcBIOParamSum4HighPrecision8)   (int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, int width, int height, const int widthG, int32_t* sumS1, int32_t* sumS2, int32_t* sumS3, int32_t* sumS5, int32_t* sumS6, Pel* dI  ,Pel* gX, Pel* gY, bool isGPM, bool isSub);
  void(*calcBIOParamSum4HighPrecision16)   (int32_t* s1, int32_t* s2, int32_t* s3, int32_t* s5, int32_t* s6, int width, int height, const int widthG, int32_t* sumS1, int32_t* sumS2, int32_t* sumS3, int32_t* sumS5, int32_t* sumS6, Pel* dI   ,Pel* gX, Pel* gY, bool isGPM, bool isSub);
#endif
#if MULTI_PASS_DMVR || SAMPLE_BASED_BDOF
  void(*calcBIOParameter)   (const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int width, int height, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, Pel* dI);
  void(*calAbsSum)          (const Pel* diff, int stride, int width, int height, int* absDiff);
  void(*calcBIOParamSum5)   (Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, const int widthG, const int width, const int height, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx);
#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
  void(*calcBIOParamSum5NOSIM4)   (int32_t* absGX, int32_t* absGY, int32_t* dIX, int32_t* dIY, int32_t* signGyGx, const int widthG, const int width, const int height, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx ,Pel* dI
#if JVET_AG0067_DMVR_EXTENSIONS
    , Pel* gX, Pel* gY
#if JVET_AL0081_BDOF_LDB_MV_REFINE
    , bool noMeanRemove
#endif
#endif
    );
  void(*calcBIOParamSum5NOSIM8)   (int32_t* absGX, int32_t* absGY, int32_t* dIX, int32_t* dIY, int32_t* signGyGx, const int widthG, const int width, const int height, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx, Pel* dI
#if JVET_AG0067_DMVR_EXTENSIONS
    , Pel* gX, Pel* gY
#if JVET_AL0081_BDOF_LDB_MV_REFINE
    , bool noMeanRemove
#endif
#endif
    );
#endif
  void(*calcBIOParamSum4)   (Pel* absGX, Pel* absGY, Pel* dIX, Pel* dIY, Pel* signGyGx, int width, int height, const int widthG, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx);
#if JVET_Z0136_OOB
  void(*addBIOAvgN)         (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride, int width, int height, int* tmpx, int* tmpy, int shift, int offset, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool *isOOB);
#else
  void(*addBIOAvgN)         (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride, int width, int height, int* tmpx, int* tmpy, int shift, int offset, const ClpRng& clpRng);
#endif
  void(*calcBIOClippedVxVy) (int* sumDIXSample32bit, int* sumAbsGxSample32bit, int* sumDIYSample32bit, int* sumAbsGySample32bit, int* sumSignGyGxSample32bit, const int limit, const int bioSubblockSize, int* tmpxSample32bit, int* tmpySample32bit);
#endif
  void(*calcBIOSums)   (const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGyGx);
  void(*calcBlkGradient)(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize);
  void(*copyBuffer)(Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height);
  void(*padding)(Pel *dst, int stride, int width, int height, int padSize);
#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  void ( *removeWeightHighFreq8)  ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int shift, int bcwWeight);
  void ( *removeWeightHighFreq4)  ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int shift, int bcwWeight);
  void ( *removeHighFreq8)        ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
  void ( *removeHighFreq4)        ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  void ( *removeWeightHighFreq1)  ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int shift, int bcwWeight);
  void ( *removeHighFreq1)        ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
#endif
#endif
  void (*profGradFilter) (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth);
  void (*applyPROF)      (Pel* dst, int dstStride, const Pel* src, int srcStride, int width, int height, const Pel* gradX, const Pel* gradY, int gradStride, const int* dMvX, const int* dMvY, int dMvStride, const bool& bi, int shiftNum, Pel offset, const ClpRng& clpRng);
  void (*roundIntVector) (int* v, int size, unsigned int nShift, const int dmvLimit);
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  int64_t (*getSumOfDifference) ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int rowSubShift, int bitDepth );
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void (*getAbsoluteDifferencePerSample) ( Pel* dst, int dstStride, const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height );
  int64_t(*getSampleSumFunc[4]) (Pel* src, int srcStride, int width, int height, int bitDepth, short* weightMask, int maskStepX, int maskStride, int maskStride2);
#endif
#if JVET_Z0136_OOB
  bool(*isMvOOB)  (const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, bool *mcMaskChroma, bool lumaOnly, ChromaFormat componentID);
  bool(*isMvOOBSubBlk) (const Mv& rcMv, const struct Position pos, const struct Size size, const SPS* sps, const PPS* pps, bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly, ChromaFormat componentID);
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  void(*computeDeltaAndShift)(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri);
  void(*computeDeltaAndShiftAddi)(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri, std::vector<RMVFInfo> &mvpInfoVecRes);
  void(*buildRegressionMatrix)(std::vector<RMVFInfo> &mvpInfoVecOri, 
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
    int64_t sumbb[2][3][3], int64_t sumeb[2][3],
#else
    int sumbb[2][3][3], int sumeb[2][3],
#endif
    uint16_t addedSize);
#endif
};
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#define GetAbsDiffPerSample(dstBuf, srcBuf0, srcBuf1)  g_pelBufOP.getAbsoluteDifferencePerSample((dstBuf).buf, (dstBuf).stride, (srcBuf0).buf, (srcBuf0).stride, (srcBuf1).buf, (srcBuf1).stride, (dstBuf).width, (dstBuf).height)
#define GetSampleSumFunc(fIdx,  srcBuf, bitDepth, mask, maskStepX, maskStride, maskStride2)  g_pelBufOP.getSampleSumFunc[fIdx] ((srcBuf).buf, (srcBuf).stride, (srcBuf).width, (srcBuf.height), (bitDepth), (mask), (maskStepX), (maskStride), (maskStride2))
#define GetSampleSum(           srcBuf, bitDepth                                          )  GetSampleSumFunc(0, (srcBuf), (bitDepth), nullptr, (        0), (         0), (          0))
#define GetMaskedSampleSum(     srcBuf, bitDepth, mask, maskStepX, maskStride, maskStride2)  GetSampleSumFunc(1, (srcBuf), (bitDepth),  (mask), (maskStepX), (maskStride), (maskStride2))
#define Get01MaskedSampleSum(   srcBuf, bitDepth, mask, maskStepX, maskStride, maskStride2)  GetSampleSumFunc(2, (srcBuf), (bitDepth),  (mask), (maskStepX), (maskStride), (maskStride2))
#define Get01InvMaskedSampleSum(srcBuf, bitDepth, mask, maskStepX, maskStride, maskStride2)  GetSampleSumFunc(3, (srcBuf), (bitDepth),  (mask), (maskStepX), (maskStride), (maskStride2))
#endif

extern PelBufferOps g_pelBufOP;

void paddingCore(Pel *ptr, int stride, int width, int height, int padSize);
void copyBufferCore(Pel *src, int srcStride, Pel *Dst, int dstStride, int width, int height);
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
int64_t getSumOfDifferenceCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int rowSubShift, int bitDepth);
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
int getMean(int sum, int div);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void getAbsoluteDifferencePerSampleCore(Pel* dst, int dstStride, const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
template <uint8_t maskType> // 0: No mask, 1: Use mask, 2: Use binary mask that contains only 0's and 1's, 3: Inverse the input binary mask before use
int64_t getMaskedSampleSumCore(Pel* src, int srcStride, int width, int height, int bitDepth, short* weightMask, int maskStepX, int maskStride, int maskStride2);
#endif

template<typename T>
struct AreaBuf : public Size
{
  T*        buf;
  int       stride;
  // the proper type causes awful lot of errors
  //ptrdiff_t stride;

  AreaBuf()                                                                               : Size(),                  buf( NULL ), stride( 0 )          { }
  AreaBuf( T *_buf, const Size &size )                                                    : Size( size ),            buf( _buf ), stride( size.width ) { }
  AreaBuf( T *_buf, const int &_stride, const Size &size )                                : Size( size ),            buf( _buf ), stride( _stride )    { }
  AreaBuf( T *_buf, const SizeType &_width, const SizeType &_height )                     : Size( _width, _height ), buf( _buf ), stride( _width )     { }
  AreaBuf( T *_buf, const int &_stride, const SizeType &_width, const SizeType &_height ) : Size( _width, _height ), buf( _buf ), stride( _stride )    { }

  operator AreaBuf<const T>() const { return AreaBuf<const T>( buf, stride, width, height ); }

  void fill                 ( const T &val, const int padSize = 0);
  void memset               ( const int val );

  void copyFrom             ( const AreaBuf<const T> &other );
#if JVET_AH0209_PDP
  void padCopyFrom(const AreaBuf<const T> &other, int w, int h, int pw, int ph);
#if JVET_AI0208_PDP_MIP
  void copyTranspose           ( const AreaBuf<const T> &other );
  void copyFromFill             ( const AreaBuf<const T> &other, int w, int h, T fill);
#endif
#endif
  void roundToOutputBitdepth(const AreaBuf<const T> &src, const ClpRng& clpRng);

  void reconstruct          ( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng);
  void copyClip             ( const AreaBuf<const T> &src, const ClpRng& clpRng);
#if MULTI_HYP_PRED
  void addHypothesisAndClip(const AreaBuf<const T> &other, const int weight, const ClpRng& clpRng);
#endif

  void subtract             ( const AreaBuf<const T> &other );
  void extendSingleBorderPel();
  void extendBorderPel      (  unsigned margin );
  void extendBorderPel(unsigned marginX, unsigned marginY);
  void padBorderPel         ( unsigned marginX, unsigned marginY, int dir );
#if JVET_Z0136_OOB
  void addWeightedAvg(const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, const int8_t bcwIdx, bool *mcMask[2], int mcStride, bool *isOOB);
#else
  void addWeightedAvg       ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, const int8_t bcwIdx);
#endif
  void removeWeightHighFreq ( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng, const int8_t iBcwWeight);
#if JVET_Z0136_OOB
  void addAvg               ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool* isOOB);
#else
  void addAvg               ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng );
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  void avg                  (const AreaBuf<const T> &other1, const AreaBuf<const T> &other2);
#endif
  void removeHighFreq       ( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng);
  void updateHistogram      ( std::vector<int32_t>& hist ) const;
#if INTER_LIC
  void subtractHistogram    (std::vector<int32_t>& hist) const;
#endif

  T    meanDiff             ( const AreaBuf<const T> &other ) const;
  void subtract             ( const T val );
  void subtract             ( const AreaBuf<const T> &buffer1, const AreaBuf<const T> &buffer2 );
#if JVET_AE0078_IBC_LIC_EXTENSION
  void linearTransforms     ( const int scale, const int shift, const int offset, const int scale2, const int shift2, const int offset2, const int yThres, bool bClip, const ClpRng& clpRng );
#endif
  void linearTransform      ( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

  void transposedFrom       ( const AreaBuf<const T> &other );

  void toLast               ( const ClpRng& clpRng );

#if JVET_AA0070_RRIBC
  void flipSignal(bool isFlipHor);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void flip                 ( const int flipType = 0 );
#endif
#endif

  void rspSignal            ( std::vector<Pel>& pLUT );
  void rspSignal            ( const AreaBuf<const Pel>& other, std::vector<Pel>& pLUT );
  void rspSignal            ( const AreaBuf<Pel> &toReshape, std::vector<Pel>& pLUT );
  void rspSignalAllAndSubtract ( const AreaBuf<Pel> &buffer1, const AreaBuf<Pel> &buffer2, std::vector<Pel>& pLUT );
  void rspSignalAndSubtract ( const AreaBuf<Pel> &buffer1, const AreaBuf<Pel> &buffer2, std::vector<Pel>& pLUT );
  void scaleSignal          ( const int scale, const bool dir , const ClpRng& clpRng);
  T    computeAvg           ( ) const;

        T& at( const int &x, const int &y )          { return buf[y * stride + x]; }
  const T& at( const int &x, const int &y ) const    { return buf[y * stride + x]; }

        T& at( const Position &pos )                 { return buf[pos.y * stride + pos.x]; }
  const T& at( const Position &pos ) const           { return buf[pos.y * stride + pos.x]; }


        T* bufAt( const int &x, const int &y )       { return &at( x, y ); }
  const T* bufAt( const int &x, const int &y ) const { return &at( x, y ); }

        T* bufAt( const Position& pos )              { return &at( pos ); }
  const T* bufAt( const Position& pos ) const        { return &at( pos ); }

  AreaBuf<      T> subBuf( const Position &pos, const Size &size )                                    { return AreaBuf<      T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<const T> subBuf( const Position &pos, const Size &size )                              const { return AreaBuf<const T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<      T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h )       { return AreaBuf<      T>( bufAt( x, y ), stride, _w, _h ); }
  AreaBuf<const T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h ) const { return AreaBuf<const T>( bufAt( x, y ), stride, _w, _h ); }
};

typedef AreaBuf<      Pel>  PelBuf;
typedef AreaBuf<const Pel> CPelBuf;

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
typedef AreaBuf<      int16_t> WeightBuf;
#endif

#if JVET_Y0141_SIGN_PRED_IMPROVE
typedef AreaBuf<      unsigned> IdxBuf;
typedef AreaBuf<const unsigned> CIdxBuf;
#endif

typedef AreaBuf<      TCoeff>  CoeffBuf;
typedef AreaBuf<const TCoeff> CCoeffBuf;

typedef AreaBuf<      MotionInfo>  MotionBuf;
typedef AreaBuf<const MotionInfo> CMotionBuf;
#if JVET_W0123_TIMD_FUSION
typedef AreaBuf<      uint8_t> IpmBuf;
typedef AreaBuf<const uint8_t> CIpmBuf;
#endif

#if JVET_AE0043_CCP_MERGE_TEMPORAL
typedef AreaBuf<      int>   CCPModelIdxBuf;
typedef AreaBuf<const int>  CCCPModelIdxBuf;
#endif
#if JVET_AG0058_EIP
typedef AreaBuf<      int>   EipModelIdxBuf;
typedef AreaBuf<const int>  CEipModelIdxBuf;
#endif
typedef AreaBuf<      TCoeff>  PLTescapeBuf;
typedef AreaBuf<const TCoeff> CPLTescapeBuf;

typedef AreaBuf<      bool>  PLTtypeBuf;
typedef AreaBuf<const bool> CPLTtypeBuf;

#if JVET_AH0135_TEMPORAL_PARTITIONING
typedef AreaBuf<      SplitPred>  QTDepthBuf;
typedef AreaBuf<const SplitPred> CQTDepthBuf;
#endif

#define SIZE_AWARE_PER_EL_OP( OP, INC )                     \
if( ( width & 7 ) == 0 )                                    \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 8 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
      OP( x + 4 );                                          \
      OP( x + 5 );                                          \
      OP( x + 6 );                                          \
      OP( x + 7 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 3 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 4 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 1 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 2 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else                                                        \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x++ )                        \
    {                                                       \
      OP( x );                                              \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}

template<typename T>
void AreaBuf<T>::fill(const T & val, const int padSize )
{
  if( width == stride )
  {
    std::fill_n( buf, width * height, val );
  }
  else
  {
    if(padSize == 0)
    {
      T* dest = buf;

      for( unsigned y = 0; y < height; y++ )
      {
        std::fill_n( dest, width, val );

        dest += stride;
      }
    }
    else
    {
      T* dest = buf;
      dest -= padSize * stride + padSize;
      for (int y = -padSize; y < (int)(height)+padSize; y++)
      {
        std::fill_n(dest, (int)(width)+2 * padSize, val);

        dest += stride;
      }
    }
  }
}

template<typename T>
void AreaBuf<T>::memset( const int val )
{
  if( width == stride )
  {
    std::fill_n( reinterpret_cast< char * >(buf), width * height * sizeof( T ), val );
  }
  else
  {
    T *dest = buf;

    for( int y = 0; y < height; y++ )
    {
      std::fill_n( reinterpret_cast< char * >(dest), width * sizeof( T ), val );

      dest += stride;
    }
  }
}

template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other )
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  CHECK( width  != other.width,  "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

  if( buf == other.buf )
  {
    return;
  }

  if( width == stride && stride == other.stride )
  {
    memcpy( buf, other.buf, width * height * sizeof( T ) );
  }
  else
  {
          T* dst         = buf;
    const T* src         = other.buf;
    const unsigned srcStride = other.stride;

    for( unsigned y = 0; y < height; y++ )
    {
      memcpy( dst, src, width * sizeof( T ) );

      dst += stride;
      src += srcStride;
    }
  }
}
#if JVET_AH0209_PDP
template<typename T>
void AreaBuf<T>::padCopyFrom(const AreaBuf<const T> &other, int w, int h, int pw, int ph)
{
  subBuf(Position(0, 0), Size(w - pw, h - ph)).copyFrom(other.subBuf(Position(0, 0), Size(w - pw, h - ph)));

  if (pw)
  {
    for (auto i = 0; i < h - ph; ++i)
    {
      subBuf(Position(w - pw, i), Size(pw, 1)).fill(other.at(w - pw - 1, i));
    }
  }
  if (ph)
  {
    for (auto i = 0; i < w - pw; ++i)
    {
      subBuf(Position(i, h - ph), Size(1, ph)).fill(other.at(i, h - ph - 1));
    }
  }
  if (pw && ph)
  {
    subBuf(Position(w - pw, h - ph), Size(pw, ph)).fill(other.at(w - pw - 1, h - ph - 1));
  }
}
#if JVET_AI0208_PDP_MIP
template<typename T>
void AreaBuf<T>::copyTranspose( const AreaBuf<const T> &other )
{
  int tw = width;
  int th = height;
  int sw = other.width;
  int sh = other.height;

  if( tw != sh || th != sw )
  {
    CHECK(1, "Size not compatible");
  }

  for( auto i = 0; i < th; ++i)
  {
    for( auto j = 0; j < tw; ++j)
    {
      at(j,i) = other.at(i,j);
    }
  }
}
template<typename T>
void AreaBuf<T>::copyFromFill( const AreaBuf<const T> &other, int w, int h, T fill)
{
  int pw = w - (int)other.width;
  int ph = h - (int)other.height;

  CHECK(pw < 0 || ph < 0, "Bad source/target buffer specified!" );

  subBuf( Position(0,0), Size(w -pw, h -ph)).copyFrom(other.subBuf( Position(0,0), Size(w -pw, h -ph)));

  if(pw)
  {
    for( auto i = 0; i < h - ph; ++i)
    {
      subBuf( Position(w-pw,i), Size(pw, 1)).fill(fill);
    }
  }
  if(ph)
  {
    for( auto i = 0; i < w - pw; ++i)
    {
      subBuf( Position(i,h-ph), Size(1, ph)).fill(fill);
    }
  }
  if(pw && ph)
  {
    subBuf( Position(w-pw,h-ph), Size(pw, ph)).fill(fill);
  }
}
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
template<typename T>
void AreaBuf<T>::flip(int flipType) // flipType = [0] no flip, [1] hor, [2] ver
{
  if (buf == nullptr)
  {
    return;
  }

  if (flipType == 2)
  {
    T* dstA = buf;
    T* dstB = buf + (height - 1) * stride;
    for (int h = (int)(height >> 1); h > 0; --h)
    {
      std::swap_ranges(dstA, dstA + width, dstB);
      dstA += stride;
      dstB -= stride;
    }
  }
  else if (flipType == 1)
  {
    int length = (int)(width >> 1);
    T* dstL = buf;
    T* dstR = buf + length;
    for (int h = (int)height; h > 0; --h)
    {
      std::swap_ranges(dstL, dstL + length, dstR);
      std::reverse(dstL, dstL + length);
      std::reverse(dstR, dstR + length);
      dstL += stride;
      dstR += stride;
    }
  }
}
#endif

#if MULTI_HYP_PRED
template<>
void AreaBuf<Pel>::addHypothesisAndClip(const AreaBuf<const Pel> &other, const int weight, const ClpRng& clpRng);
#endif

template<typename T>
void AreaBuf<T>::subtract( const AreaBuf<const T> &other )
{
  CHECK( width  != other.width,  "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

        T* dest =       buf;
  const T* subs = other.buf;

#define SUBS_INC        \
  dest +=       stride; \
  subs += other.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] -= subs[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<typename T>
void AreaBuf<T>::subtract( const AreaBuf<const T> &buffer1, const AreaBuf<const T> &buffer2 )
{
  CHECK( width != buffer1.width, "Incompatible size in buffer1" );
  CHECK( height != buffer1.height, "Incompatible size in buffer1" );
  CHECK( width != buffer2.width, "Incompatible size in buffer2" );
  CHECK( height != buffer2.height, "Incompatible size in buffer2" );

  T* dest = buf;
  const T* buf1 = buffer1.buf;
  const T* buf2 = buffer2.buf;

#define SUBS_INC          \
  dest +=         stride; \
  buf1 += buffer1.stride; \
  buf2 += buffer2.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] = buf1[ADDR] - buf2[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<typename T>
void AreaBuf<T>::copyClip( const AreaBuf<const T> &src, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel> &src, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::reconstruct( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng );


template<typename T>
#if JVET_Z0136_OOB
void AreaBuf<T>::addAvg( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool* isOOB)
#else
void AreaBuf<T>::addAvg( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng )
#endif
{
  THROW( "Type not supported" );
}

template<>
#if JVET_Z0136_OOB
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng, bool *mcMask[2], int mcStride, bool* isOOB);
#else
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng );
#endif

template<typename T>
void AreaBuf<T>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::toLast( const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::toLast( const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::removeWeightHighFreq(const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng, const int8_t bcwWeight)
{
  const int8_t bcwWeightOther = g_bcwWeightBase - bcwWeight;
  const int8_t log2WeightBase = g_bcwLog2WeightBase;

  const Pel* src = other.buf;
  const int  srcStride = other.stride;

  Pel* dst = buf;
  const int  dstStride = stride;

#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  if(!bClip)
  {
    if( !( width & 7 ) )
    {
      g_pelBufOP.removeWeightHighFreq8( dst, dstStride, src, srcStride, width, height, 16, bcwWeight );
    }
    else if( !( width & 3 ) )
    {
      g_pelBufOP.removeWeightHighFreq4( dst, dstStride, src, srcStride, width, height, 16, bcwWeight );
    }
    else
    {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      g_pelBufOP.removeWeightHighFreq1( dst, dstStride, src, srcStride, width, height, 16, bcwWeight );
#else
      CHECK( true, "Not supported" );
#endif
    }
  }
  else
  {
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    Intermediate_Int normalizer = ((1 << 16) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;
#else
    int normalizer = ((1 << 16) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    Intermediate_Int weight0 = normalizer << log2WeightBase;
    Intermediate_Int weight1 = bcwWeightOther * normalizer;
#else
    int weight0 = normalizer << log2WeightBase;
    int weight1 = bcwWeightOther * normalizer;
#endif
#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( T((dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16), clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             T((dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16)
#else
#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( (dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16, clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             (dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16
#endif

    if(bClip)
    {
      SIZE_AWARE_PER_EL_OP(REM_HF_OP_CLIP, REM_HF_INC);
    }
    else
    {
      SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);
    }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  }
#endif
}

template<typename T>
void AreaBuf<T>::removeHighFreq( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng )
{
  const T*  src       = other.buf;
  const int srcStride = other.stride;

        T*  dst       = buf;
  const int dstStride = stride;

#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  if (!bClip)
  {
    if( !( width & 7 ) )
    {
      g_pelBufOP.removeHighFreq8( dst, dstStride, src, srcStride, width, height );
    }
    else if( !( width & 3 ) )
    {
      g_pelBufOP.removeHighFreq4( dst, dstStride, src, srcStride, width, height );
    }
    else
    {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      g_pelBufOP.removeHighFreq1(dst, dstStride, src, srcStride, width, height);
#else
      CHECK(true, "Not supported");
#endif
    }
  }
  else
  {
#endif

#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( 2 * dst[ADDR] - src[ADDR], clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             2 * dst[ADDR] - src[ADDR]

  if( bClip )
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP_CLIP, REM_HF_INC );
  }
  else
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP,      REM_HF_INC );
  }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP

#if ENABLE_SIMD_OPT_BCW && defined(TARGET_SIMD_X86)
  }
#endif
}


template<typename T>
void AreaBuf<T>::updateHistogram( std::vector<int32_t>& hist ) const
{
  const T* data = buf;
  for( std::size_t y = 0; y < height; y++, data += stride )
  {
    for( std::size_t x = 0; x < width; x++ )
    {
      hist[ data[x] ]++;
    }
  }
}

#if INTER_LIC
template<typename T>
void AreaBuf<T>::subtractHistogram(std::vector<int32_t>& hist) const
{
  const T* data = buf;
  for (std::size_t y = 0; y < height; y++, data += stride)
  {
    for (std::size_t x = 0; x < width; x++)
    {
      hist[data[x]]--;
    }
  }
}
#endif

template<typename T>
void AreaBuf<T>::extendBorderPel(unsigned marginX, unsigned marginY)
{
  T* p = buf;
  int h = height;
  int w = width;
  int s = stride;

  CHECK((w + 2 * marginX) > s, "Size of buffer too small to extend");
  // do left and right margins
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < marginX; x++)
    {
      *(p - marginX + x) = p[0];
      p[w + x] = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= (s + marginX);
  // p is now the (-margin, height-1)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p + (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }

  // p is still (-marginX, height-1)
  p -= ((h - 1) * s);
  // p is now (-marginX, 0)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p - (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }
}

template<typename T>
void AreaBuf<T>::padBorderPel( unsigned marginX, unsigned marginY, int dir )
{
  T*  p = buf;
  int s = stride;
  int h = height;
  int w = width;

  CHECK( w  > s, "Size of buffer too small to extend" );

  // top-left margin
  if ( dir == 1 )
  {
    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[marginX];
      }
      p += s;
    }
  }

  // bottom-right margin
  if ( dir == 2 )
  {
    p = buf + s * ( h - marginY ) + w - marginX;

    for( int y = 0; y < marginY; y++ )
    {
      for( int x = 0; x < marginX; x++ )
      {
        p[x] = p[-1];
      }
      p += s;
    }
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPel( unsigned margin )
{
  T*  p = buf;
  int h = height;
  int w = width;
  int s = stride;

  CHECK( ( w + 2 * margin ) > s, "Size of buffer too small to extend" );
  // do left and right margins
  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < margin; x++ )
    {
      *( p - margin + x ) = p[0];
      p[w + x] = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= ( s + margin );
  // p is now the (-margin, height-1)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p + ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }

  // pi is still (-marginX, height-1)
  p -= ( ( h - 1 ) * s );
  // pi is now (-marginX, 0)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p - ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }
}

template<typename T>
T AreaBuf<T>::meanDiff( const AreaBuf<const T> &other ) const
{
  int64_t acc = 0;

  CHECK( width  != other.width,  "Incompatible size" );
  CHECK( height != other.height, "Incompatible size" );

  const T* src1 =       buf;
  const T* src2 = other.buf;

#define MEAN_DIFF_INC   \
  src1 +=       stride; \
  src2 += other.stride; \

#define MEAN_DIFF_OP(ADDR) acc += src1[ADDR] - src2[ADDR]

  SIZE_AWARE_PER_EL_OP( MEAN_DIFF_OP, MEAN_DIFF_INC );

#undef MEAN_DIFF_INC
#undef MEAN_DIFF_OP

  return T( acc / area() );
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::subtract( const Pel val );
#endif

template<typename T>
void AreaBuf<T>::subtract( const T val )
{
  T* dst = buf;

#define OFFSET_INC       dst       += stride
#define OFFSET_OP(ADDR)  dst[ADDR] -= val

  SIZE_AWARE_PER_EL_OP( OFFSET_OP, OFFSET_INC );

#undef OFFSET_INC
#undef OFFSET_OP
}

template<typename T>
void AreaBuf<T>::transposedFrom( const AreaBuf<const T> &other )
{
  CHECK( width * height != other.width * other.height, "Incompatible size" );

        T* dst  =       buf;
  const T* src  = other.buf;
  width         = other.height;
  height        = other.width;
  stride        = stride < width ? width : stride;

  for( unsigned y = 0; y < other.height; y++ )
  {
    for( unsigned x = 0; x < other.width; x++ )
    {
      dst[y + x*stride] = src[x + y*other.stride];
    }
  }
}

template<typename T>
T AreaBuf <T> ::computeAvg() const
{
    const T* src = buf;
#if ENABLE_QPA
    int64_t  acc = 0; // for picture-wise use in getGlaringColorQPOffset() and applyQPAdaptationChroma()
#else
    int32_t  acc = 0;
#endif
#define AVG_INC      src += stride
#define AVG_OP(ADDR) acc += src[ADDR]
    SIZE_AWARE_PER_EL_OP(AVG_OP, AVG_INC);
#undef AVG_INC
#undef AVG_OP
    return T ((acc + (area() >> 1)) / area());
}

#ifndef DONT_UNDEF_SIZE_AWARE_PER_EL_OP
#undef SIZE_AWARE_PER_EL_OP
#endif // !DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// ---------------------------------------------------------------------------
// UnitBuf struct
// ---------------------------------------------------------------------------

struct UnitArea;

template<typename T>
struct UnitBuf
{
  typedef static_vector<AreaBuf<T>,       MAX_NUM_COMPONENT> UnitBufBuffers;
  typedef static_vector<AreaBuf<const T>, MAX_NUM_COMPONENT> ConstUnitBufBuffers;

  ChromaFormat chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat( NUM_CHROMA_FORMAT ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const UnitBufBuffers&  _bufs ) : chromaFormat( _chromaFormat ), bufs( _bufs ) { }
  UnitBuf( const ChromaFormat &_chromaFormat,       UnitBufBuffers&& _bufs ) : chromaFormat( _chromaFormat ), bufs( std::forward<UnitBufBuffers>( _bufs ) ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY ) : chromaFormat( _chromaFormat ), bufs{ blkY } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY) } { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY, const AreaBuf<T>  &blkCb, const AreaBuf<T>  &blkCr ) : chromaFormat( _chromaFormat ), bufs{ blkY, blkCb, blkCr } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY,       AreaBuf<T> &&blkCb,       AreaBuf<T> &&blkCr ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY), std::forward<AreaBuf<T> >(blkCb), std::forward<AreaBuf<T> >(blkCr) } { }

  operator UnitBuf<const T>() const
  {
    return UnitBuf<const T>( chromaFormat, ConstUnitBufBuffers( bufs.begin(), bufs.end() ) );
  }

        AreaBuf<T>& get( const ComponentID comp )        { return bufs[comp]; }
  const AreaBuf<T>& get( const ComponentID comp )  const { return bufs[comp]; }

        AreaBuf<T>& Y()        { return bufs[0]; }
  const AreaBuf<T>& Y()  const { return bufs[0]; }
        AreaBuf<T>& Cb()       { return bufs[1]; }
  const AreaBuf<T>& Cb() const { return bufs[1]; }
        AreaBuf<T>& Cr()       { return bufs[2]; }
  const AreaBuf<T>& Cr() const { return bufs[2]; }

  void fill                 ( const T &val );
  void copyFrom             ( const UnitBuf<const T> &other, const bool lumaOnly = false, const bool chromaOnly = false );
  void roundToOutputBitdepth(const UnitBuf<const T> &src, const ClpRngs& clpRngs);
  void reconstruct          ( const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs );
  void copyClip             ( const UnitBuf<const T> &src, const ClpRngs& clpRngs, const bool lumaOnly = false, const bool chromaOnly = false );
#if MULTI_HYP_PRED
  void addHypothesisAndClip(const UnitBuf<const T> &other, const int weight, const ClpRngs& clpRngs, const bool lumaOnly = false);
#endif
  void subtract             ( const UnitBuf<const T> &other );
#if JVET_Z0136_OOB
  void addWeightedAvg       ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false, bool *mcMask[2] = NULL, int mcStride = -1, bool *mcMaskChroma[2] = NULL, int mcCStride = -1, bool *isOOB = NULL);
  void addAvg               ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false, bool *mcMask[2] = NULL, int mcStride = -1, bool *mcMaskChroma[2] = NULL, int mcCStride = -1, bool *isOOB = NULL);
#else
  void addWeightedAvg       ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false);
  void addAvg               ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false);
#endif
  void extendSingleBorderPel();
  void extendBorderPel(unsigned marginX, unsigned marginY);
  void padBorderPel         ( unsigned margin, int dir );
  void extendBorderPel      ( unsigned margin );
  void removeHighFreq       ( const UnitBuf<T>& other, const bool bClip, const ClpRngs& clpRngs
                            , const int8_t bcwWeight = g_bcwWeights[BCW_DEFAULT]
                            );

        UnitBuf<      T> subBuf (const UnitArea& subArea);
  const UnitBuf<const T> subBuf (const UnitArea& subArea) const;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void copyFromComponent    ( const UnitBuf<const T> &other, ComponentID startCompId, ComponentID lastCompId );
#endif
  void colorSpaceConvert(const UnitBuf<T> &other, const bool forward, const ClpRng& clpRng);
};

typedef UnitBuf<      Pel>  PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<      TCoeff>  CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

template<typename T>
void UnitBuf<T>::fill( const T &val )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].fill( val );
  }
}

template<typename T>
void UnitBuf<T>::copyFrom(const UnitBuf<const T> &other, const bool lumaOnly, const bool chromaOnly )
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECK( lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected" );
  const size_t compStart = chromaOnly ? 1 : 0;
  const size_t compEnd   = lumaOnly ? 1 : (unsigned) bufs.size();
  for( size_t i = compStart; i < compEnd; i++ )
  {
    bufs[i].copyFrom( other.bufs[i] );
  }
}

#if MULTI_HYP_PRED
template<typename T>
void UnitBuf<T>::addHypothesisAndClip(const UnitBuf<const T> &other, const int weight, const ClpRngs& clpRngs, const bool lumaOnly)
{
  CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");

  for (unsigned i = 0; i < (lumaOnly ? 1 : bufs.size()); i++)
  {
    bufs[i].addHypothesisAndClip(other.bufs[i], weight, clpRngs.comp[i]);
  }
}
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
template<typename T>
void UnitBuf<T>::copyFromComponent( const UnitBuf<const T> &other, ComponentID startCompId, ComponentID lastCompId )
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = startCompId; i <= lastCompId; i++ )
  {
    bufs[i].copyFrom( other.bufs[i] );
  }
}
#endif

template<typename T>
void UnitBuf<T>::subtract( const UnitBuf<const T> &other )
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].subtract( other.bufs[i] );
  }
}

template<typename T>
void UnitBuf<T>::copyClip(const UnitBuf<const T> &src, const ClpRngs &clpRngs, const bool lumaOnly, const bool chromaOnly )
{
  CHECK( chromaFormat != src.chromaFormat, "Incompatible formats" );

  CHECK( lumaOnly && chromaOnly, "Not allowed to have both lumaOnly and chromaOnly selected" );
  const size_t compStart = chromaOnly ? 1 : 0;
  const size_t compEnd   = lumaOnly ? 1 : bufs.size();
  for( size_t i = compStart; i < compEnd; i++ )
  {
    bufs[i].copyClip( src.bufs[i], clpRngs.comp[i] );
  }
}


template<typename T>
void UnitBuf<T>::roundToOutputBitdepth(const UnitBuf<const T> &src, const ClpRngs& clpRngs)
{
  CHECK(chromaFormat != src.chromaFormat, "Incompatible formats");

  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].roundToOutputBitdepth(src.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T>
void UnitBuf<T>::reconstruct(const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs)
{
  CHECK( chromaFormat != pred.chromaFormat, "Incompatible formats" );
  CHECK( chromaFormat != resi.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].reconstruct( pred.bufs[i], resi.bufs[i], clpRngs.comp[i] );
  }
}

template<typename T>
#if JVET_Z0136_OOB
void UnitBuf<T>::addWeightedAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx /* = BCW_DEFAULT */, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */, bool *mcMask[2], int mcStride, bool *mcMaskChroma[2], int mcCStride, bool *isOOB)
#else
void UnitBuf<T>::addWeightedAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t bcwIdx /* = BCW_DEFAULT */, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
#endif
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend = lumaOnly ? 1 : bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  for(size_t i = istart; i < iend; i++)
  {
#if JVET_Z0136_OOB
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], bcwIdx, i == 0 ? mcMask : mcMaskChroma, i == 0 ? mcStride : mcCStride, isOOB);
#else
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], bcwIdx);
#endif
  }
}

template<typename T>
#if JVET_Z0136_OOB
void UnitBuf<T>::addAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */, bool *mcMask[2], int mcStride, bool *mcMaskChroma[2], int mcCStride, bool *isOOB)
#else
void UnitBuf<T>::addAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
#endif
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly   ? 1 : bufs.size();

  CHECK( lumaOnly && chromaOnly, "should not happen" );

  for( size_t i = istart; i < iend; i++)
  {
#if JVET_Z0136_OOB
    bufs[i].addAvg( other1.bufs[i], other2.bufs[i], clpRngs.comp[i], i == 0 ? mcMask : mcMaskChroma, i == 0 ? mcStride : mcCStride, isOOB);
#else
    bufs[i].addAvg( other1.bufs[i], other2.bufs[i], clpRngs.comp[i]);
#endif
  }
}

template<typename T>
void UnitBuf<T>::colorSpaceConvert(const UnitBuf<T> &other, const bool forward, const ClpRng& clpRng)
{
  THROW("Type not supported");
}

template<>
void UnitBuf<Pel>::colorSpaceConvert(const UnitBuf<Pel> &other, const bool forward, const ClpRng& clpRng);

template<typename T>
void UnitBuf<T>::extendSingleBorderPel()
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendSingleBorderPel();
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPel(unsigned marginX, unsigned marginY)
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].extendBorderPel(marginX >> getComponentScaleX(ComponentID(i), chromaFormat), marginY >> getComponentScaleY(ComponentID(i), chromaFormat));
  }
}

template<typename T>
void UnitBuf<T>::padBorderPel( unsigned margin, int dir )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].padBorderPel( margin >> getComponentScaleX( ComponentID( i ), chromaFormat ), margin >> getComponentScaleY( ComponentID( i ), chromaFormat ), dir );
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPel( unsigned margin )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendBorderPel( margin );
  }
}

template<typename T>
void UnitBuf<T>::removeHighFreq( const UnitBuf<T>& other, const bool bClip, const ClpRngs& clpRngs
                               , const int8_t bcwWeight
                               )
{
  if(bcwWeight != g_bcwWeights[BCW_DEFAULT])
  {
    bufs[0].removeWeightHighFreq(other.bufs[0], bClip, clpRngs.comp[0], bcwWeight);
    return;
  }
  bufs[0].removeHighFreq(other.bufs[0], bClip, clpRngs.comp[0]);

}

template<typename T>
UnitBuf<T> UnitBuf<T>::subBuf( const UnitArea& subArea )
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}


template<typename T>
const UnitBuf<const T> UnitBuf<T>::subBuf( const UnitArea& subArea ) const
{
  UnitBuf<const T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( const auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}

// ---------------------------------------------------------------------------
// PelStorage struct (PelUnitBuf which allocates its own memory)
// ---------------------------------------------------------------------------

struct UnitArea;
struct CompArea;

struct PelStorage : public PelUnitBuf
{
  PelStorage();
  ~PelStorage();

  void swap( PelStorage& other );
  void createFromBuf( PelUnitBuf buf );
  void create( const UnitArea &_unit );
  void create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize = 0, const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true );
  void destroy();

         PelBuf getBuf( const CompArea &blk );
  const CPelBuf getBuf( const CompArea &blk ) const;

         PelBuf getBuf( const ComponentID CompID );
  const CPelBuf getBuf( const ComponentID CompID ) const;

         PelUnitBuf getBuf( const UnitArea &unit );
  const CPelUnitBuf getBuf( const UnitArea &unit ) const;
  Pel *getOrigin( const int id ) const { return m_origin[id]; }

private:

  Pel *m_origin[MAX_NUM_COMPONENT];
};

struct CompStorage : public PelBuf
{
  CompStorage () { m_memory = nullptr; }
  ~CompStorage() { if (valid()) delete [] m_memory; }

  void create( const Size& size )
  {
    CHECK( m_memory, "Trying to re-create an already initialized buffer" );
    m_memory = new Pel [ size.area() ];
    *static_cast<PelBuf*>(this) = PelBuf( m_memory, size );
  }
  void destroy()
  {
    if (valid()) delete [] m_memory;
    m_memory = nullptr;
  }
  bool valid() { return m_memory != nullptr; }
private:
  Pel* m_memory;
};

#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
struct FrameCtrlInfo
{
  int8_t alfTemporalLayer[MAX_NUM_COMPONENT] = { 100, 100, 100 };
  std::vector<uint8_t> bufAlfFlag[MAX_NUM_COMPONENT];
  std::vector<uint8_t> bufAlfFilter[MAX_NUM_COMPONENT];
  std::vector<uint8_t> bufCcalfFilter[MAX_NUM_COMPONENT];
  int8_t ccalfTemporalLayer[MAX_NUM_COMPONENT] = { 100, 100, 100 };
  FrameCtrlInfo() { reset(); }
  void resize(int ctuSize)
  {
    bufAlfFlag[COMPONENT_Y].resize(1);
    bufAlfFilter[COMPONENT_Y].resize(1);
    bufCcalfFilter[COMPONENT_Y].resize(1);
    for (int comp = 1; comp < MAX_NUM_COMPONENT; comp++)
    {
      bufAlfFlag[comp].resize(ctuSize);
      bufAlfFilter[comp].resize(ctuSize);
      bufCcalfFilter[comp].resize(ctuSize);
    }
    reset();
  }

  void reset()
  {
    for (int comp = 1; comp < MAX_NUM_COMPONENT; comp++)
    {
      memset(bufAlfFlag[comp].data(), 0, sizeof(uint8_t) * bufAlfFlag[comp].size());
      memset(bufAlfFilter[comp].data(), 0, sizeof(uint8_t) * bufAlfFilter[comp].size());
      alfTemporalLayer[comp] = 100;
      memset(bufCcalfFilter[comp].data(), 0, sizeof(uint8_t) * bufCcalfFilter[comp].size());
    }
    ccalfTemporalLayer[COMPONENT_Y] = ccalfTemporalLayer[COMPONENT_Cb] = ccalfTemporalLayer[COMPONENT_Cr] = 100;
  }
};
extern FrameCtrlInfo g_pictureControlInfo[ALF_CTB_MAX_NUM_APS];
extern bool g_pictureControlInfoInit;
#endif
#endif