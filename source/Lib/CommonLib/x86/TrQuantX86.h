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
#include "../TrQuant.h"

#ifdef TARGET_SIMD_X86

#include <nmmintrin.h>

#if TRANSFORM_SIMD_OPT
template <TransType transType, int transSize>
void TrQuant::fastForwardTransform_SIMD( const TCoeff *block, TCoeff *coeff, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  int i, j, k, sum;
  int rndFactor = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = m_forwardTransformKernels[transType][g_aucLog2[transSize] - 1];
  const int zoRow = line - iSkipLine;
  const int zoCol = transSize - iSkipLine2;

  TCoeff *pCoef;
  if( ( transSize & 0x03 ) == 0 && sizeof( TCoeff ) == 4 && sizeof( TMatrixCoeff ) == 2 )
  {
    if( iSkipLine ) // zeroout true
    {
      // zo represents number of lines to be processed
      // colShift: representes number of cols to processed
      for( i = 0; i < zoRow; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < zoCol; j++ )
        {
          __m128i tmpSum = _mm_setzero_si128();
          for( k = 0; k < transSize; k += 4 )
          {
            __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( block + k ) );
            __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
          }
          tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
          tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
          sum = _mm_cvtsi128_si32( tmpSum );
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
    }
    else
    {
      if( ( transType == 0 ) && ( ( transSize & 0xF ) == 0 ) )
      {
        for( i = 0; i < line; i++ )
        {
          TCoeff    splitBlockE[2048];
          const int halfSize = transSize >> 1;
          const int quarterSize = transSize >> 2;
          TCoeff  * splitBlockO = splitBlockE + halfSize;
          TCoeff  * splitBlockF = splitBlockE + quarterSize;
          TCoeff  * splitBlockG = splitBlockE + transSize;

          for( int m = 0, n = transSize - 1; m < n; m++, n-- )
          {
            splitBlockG[m] = block[m] + block[n];
            splitBlockO[m] = block[m] - block[n];
          }
          for( int m = 0, n = halfSize - 1; m < n; m++, n-- )
          {
            splitBlockE[m] = splitBlockG[m] + splitBlockG[n];
            splitBlockF[m] = splitBlockG[m] - splitBlockG[n];
          }

          pCoef = coeff;
          const TMatrixCoeff *curT = iT;
          for( j = 0; j < transSize; j++ )
          {
            const int    arraySize = ( j & 1 ? halfSize : quarterSize );
            const TCoeff * selPart = ( j & 1 ? splitBlockO : ( j & 2 ? splitBlockF : splitBlockE ) );
            __m128i tmpSum = _mm_setzero_si128();

            for( k = 0; k < arraySize; k += 4 )
            {
              __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( selPart + k ) );
              __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
              tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
            }

            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
            sum = _mm_cvtsi128_si32( tmpSum );
            pCoef[i] = ( sum + rndFactor ) >> shift;
            pCoef += line;
            curT += transSize;
          }
          block += transSize;
        }
      }
      else
      {
        for( i = 0; i < line; i++ )
        {
          pCoef = coeff;
          const TMatrixCoeff *curT = iT;
          for( j = 0; j < transSize; j++ )
          {
            __m128i tmpSum = _mm_setzero_si128();
            for( k = 0; k < transSize; k += 4 )
            {
              __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( block + k ) );
              __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
              tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
            }
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
            sum = _mm_cvtsi128_si32( tmpSum );
            pCoef[i] = ( sum + rndFactor ) >> shift;
            pCoef += line;
            curT += transSize;
          }
          block += transSize;
        }
      }
    }
  }
  else
  {
    if( iSkipLine )
    {
      TCoeff *tmp = coeff;
      int     numCoeff = line * transSize; // total number of coefficients
      for( i = 0; i < zoRow; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < zoCol; j++ )
        {
          sum = 0;
          for( k = 0; k < transSize; k++ )
          {
            sum += block[k] * curT[k];
          }
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
      coeff += zoRow;
      int zeOutNumCoeffCols = line - zoRow;
      if( zoRow )
      {
        for( j = 0; j < zoCol; j++ )
        {
          memset( coeff, 0, sizeof( TCoeff ) * ( zeOutNumCoeffCols ) );
          coeff += line;
        }
      }
      coeff = tmp + line * zoCol;
      int zeOutNumCoeffRows = numCoeff - ( line * zoCol );
      memset( coeff, 0, sizeof( TCoeff ) * zeOutNumCoeffRows );
    }
    else
    {
      for( i = 0; i < line; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < transSize; j++ )
        {
          sum = 0;
          for( k = 0; k < transSize; k++ )
          {
            sum += block[k] * curT[k];
          }
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
    }
  }
}

template <TransType transType, int transSize>
void TrQuant::fastInverseTransform_SIMD( const TCoeff *coeff, TCoeff *block, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  int i, j, k;
  int rndFactor = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = m_inverseTransformKernels[transType][g_aucLog2[transSize] - 1];

  const int zoRow = line - iSkipLine;
  const int zoCol = transSize - iSkipLine2;

  if( !( transSize & 0x03 ) && sizeof( TCoeff ) == 4 && sizeof( TMatrixCoeff ) == 2 )
  {
    const __m128i minCoeff = _mm_set1_epi32( outputMinimum );
    const __m128i maxCoeff = _mm_set1_epi32( outputMaximum );
    const __m128i round = _mm_set1_epi32( rndFactor );

    if( transType == DCT2 && !( transSize & 0xF ) )
    {
      for( i = 0; i < line; i++, block += transSize )
      {
        TCoeff    splitBlockE[2048];
        const int halfSize = transSize >> 1;
        const int quarterSize = transSize >> 2;
        TCoeff  * splitBlockO = splitBlockE + halfSize;
        TCoeff  * splitBlockF = splitBlockE + quarterSize;
        TCoeff  * splitBlockG = splitBlockE + transSize;

        memset( splitBlockE, 0, 3 * halfSize * sizeof( TCoeff ) );

        const TCoeff *      curCoeff = coeff;
        const TMatrixCoeff *curIT = iT;
        for( k = 0; k < transSize; k++, curCoeff += line, curIT += transSize )
        {
          const int arraySize = ( k & 1 ? halfSize : quarterSize );
          __m128i    tmpCoeff = _mm_set1_epi32( curCoeff[i] );
          __m128i  * tmpBlock = ( __m128i * )( k & 1 ? splitBlockO : ( k & 2 ? splitBlockF : splitBlockE ) );

          for( j = 0; j < arraySize; j += 4, tmpBlock++ )
          {
            __m128i tmp = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curIT + j ) ) );
            __m128i sum = _mm_loadu_si128( tmpBlock );
            tmp = _mm_mullo_epi32( tmpCoeff, tmp );
            tmp = _mm_add_epi32( tmp, sum );
            _mm_storeu_si128( tmpBlock, tmp );
          }
        }

        for( int m = 0, n = halfSize - 1; m < n; m++, n-- )
        {
          splitBlockG[m] = splitBlockE[m] + splitBlockF[m];
          splitBlockG[n] = splitBlockE[m] - splitBlockF[m];
        }
        for( int m = 0, n = transSize - 1; m < n; m++, n-- )
        {
          block[m] = ( splitBlockG[m] + splitBlockO[m] );
          block[n] = ( splitBlockG[m] - splitBlockO[m] );
        }

        __m128i *tmpBlock = ( __m128i * )block;
        for( j = 0; j < transSize; j += 4, tmpBlock++ )
        {
          __m128i tmp = _mm_loadu_si128( tmpBlock );
          tmp = _mm_srai_epi32( _mm_add_epi32( tmp, round ), shift );
          tmp = _mm_min_epi32( _mm_max_epi32( tmp, minCoeff ), maxCoeff );
          _mm_storeu_si128( tmpBlock, tmp );
        }
      }
    }
    else
    {
      for( i = 0; i < line; i++, block += transSize )
      {
        memset( block, 0, transSize << 2 );
        const TCoeff *      curCoeff = coeff;
        const TMatrixCoeff *curIT = iT;
        for( k = 0; k < transSize; k++, curCoeff += line, curIT += transSize )
        {
          __m128i  tmpCoeff = _mm_set1_epi32( curCoeff[i] );
          __m128i *tmpBlock = ( __m128i * )block;
          for( j = 0; j < transSize; j += 4, tmpBlock++ )
          {
            __m128i tmp = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curIT + j ) ) );
            __m128i sum = _mm_loadu_si128( tmpBlock );
            tmp = _mm_mullo_epi32( tmpCoeff, tmp );
            tmp = _mm_add_epi32( tmp, sum );
            _mm_storeu_si128( tmpBlock, tmp );
          }
        }
        __m128i *tmpBlock = ( __m128i * )block;
        for( j = 0; j < transSize; j += 4, tmpBlock++ )
        {
          __m128i tmp = _mm_loadu_si128( tmpBlock );
          tmp = _mm_srai_epi32( _mm_add_epi32( tmp, round ), shift );
          tmp = _mm_min_epi32( _mm_max_epi32( tmp, minCoeff ), maxCoeff );
          _mm_storeu_si128( tmpBlock, tmp );
        }
      }
    }
  }
  else
  {
    for( i = 0; i < zoRow; i++ )
    {
      for( j = 0; j < transSize; j++ )
      {
        int sum = 0;
        for( k = 0; k < zoCol; k++ )
        {
          sum += coeff[k * line + i] * iT[k * transSize + j];
        }
        block[i * transSize + j] = Clip3( outputMinimum, outputMaximum, ( int ) ( sum + rndFactor ) >> shift );
      }
    }

    if( iSkipLine )
    {
      memset( block + ( zoRow*transSize ), 0, ( iSkipLine*transSize ) * sizeof( TCoeff ) );
    }
  }
}
#endif

#if INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void forwardLfnst_SIMD(TCoeff* src, TCoeff*& dst, const int8_t*& trMat, const int trSize, const int zeroOutSize)
{
  CHECK((trSize & 0x7) || (zeroOutSize & 0x7), "trSize and zeroOutSize should be multiple of 8");

  const __m128i vrnd = _mm_set1_epi32(64);
  __m128i vmat[4], vcoef[4], vsrc;

  int trSize2 = (trSize << 1);
  int trSize3 = trSize2 + trSize;
  int trSize4 = (trSize << 2);
  for (int j = 0; j < zeroOutSize; j += 4)
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    const int8_t* trMatTmp[4] = { trMat, trMat + trSize, trMat + trSize2, trMat + trSize3 };
    vcoef[0] = vcoef[1] = vcoef[2] = vcoef[3] = _mm_setzero_si128();
    for (int i = 0; i < trSize; i += 4)
    {
      vsrc = _mm_loadu_si128((const __m128i*)srcPtr);
      for (int idx = 0; idx < 4; idx++)
      {
        vmat[idx] = _mm_cvtepi8_epi32(_mm_cvtsi32_si128(*(unsigned int const*)trMatTmp[idx]));
        vcoef[idx] = _mm_add_epi32(_mm_mullo_epi32(vsrc, vmat[idx]), vcoef[idx]);
      }
      srcPtr += 4;
      trMatTmp[0] += 4; trMatTmp[1] += 4; trMatTmp[2] += 4; trMatTmp[3] += 4;
    }
    vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[1]);
    vcoef[2] = _mm_hadd_epi32(vcoef[2], vcoef[3]);
    vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[2]);
    vcoef[0] = _mm_srai_epi32(_mm_add_epi32(vcoef[0], vrnd), 7);
    _mm_storeu_si128((__m128i *)dst, vcoef[0]);

    dst += 4;
    trMat += trSize4;
  }
}

template <X86_VEXT vext>
void inverseLfnst_SIMD(TCoeff* src, TCoeff*  dst, const int8_t*  trMat, const int trSize, const int zeroOutSize, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  CHECK((trSize & 0x7) || (zeroOutSize & 0x7), "trSize and zeroOutSize should be multiple of 8");

  const __m128i vrnd = _mm_set1_epi32(64);
  __m128i vmat[4], vcoef[4], vsrc;
  __m128i vmin = _mm_set1_epi32(outputMinimum);
  __m128i vmax = _mm_set1_epi32(outputMaximum);
	
  int trSize2 = (trSize << 1);
  int trSize3 = trSize2 + trSize;
  int trSize4 = (trSize << 2);
  for (int j = 0; j < trSize; j += 4)
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    const int8_t* trMatTmp[4] = { trMat, trMat + 1, trMat + 2, trMat + 3 };
    vcoef[0] = vcoef[1] = vcoef[2] = vcoef[3] = _mm_setzero_si128();
    for (int i = 0; i < zeroOutSize; i += 4)
    {
      vsrc = _mm_loadu_si128((const __m128i*)srcPtr);
      for (int idx = 0; idx < 4; idx++)
      {
        vmat[idx] = _mm_set_epi32(*(trMatTmp[idx] + trSize3), *(trMatTmp[idx] + trSize2), *(trMatTmp[idx] + trSize), *trMatTmp[idx]);
        vcoef[idx] = _mm_add_epi32(_mm_mullo_epi32(vsrc, vmat[idx]), vcoef[idx]);
      }
      srcPtr += 4;
      trMatTmp[0] += trSize4; trMatTmp[1] += trSize4; trMatTmp[2] += trSize4; trMatTmp[3] += trSize4;
    }
    vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[1]);
    vcoef[2] = _mm_hadd_epi32(vcoef[2], vcoef[3]);
    vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[2]);
    vcoef[0] = _mm_srai_epi32(_mm_add_epi32(vcoef[0], vrnd), 7);
    vcoef[0] = _mm_min_epi32(vmax, _mm_max_epi32(vmin, vcoef[0]));
    _mm_storeu_si128((__m128i *)dst, vcoef[0]);

    dst += 4;
    trMat += 4;
  }
}
#endif
#if JVET_AC0130_NSPT && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void computeFwdNspt_SIMD( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx
#else
void computeFwdNspt_SIMD( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
                             , int bktIdx
#endif
)
{
  int trSize = width * height;

#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  const int8_t* trMat = TrQuant::getNsptMatrix(mode, width, height, nsptIdx, bktIdx);
#else
  const int8_t* trMat = g_nspt4x4[ mode ][ nsptIdx ][ 0 ];
  if( width == 8 && height == 8 )
  {
    trMat = g_nspt8x8[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 4 && height == 8 )
  {
    trMat = g_nspt4x8[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 4 )
  {
    trMat = g_nspt8x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 4 && height == 16 )
  {
    trMat = g_nspt4x16[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 16 && height == 4 )
  {
    trMat = g_nspt16x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 16 )
  {
    trMat = g_nspt8x16[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 16 && height == 8 )
  {
    trMat = g_nspt16x8[ mode ][ nsptIdx ][ 0 ];
  }
#if JVET_AE0086_LARGE_NSPT
  else if( width == 4 && height == 32 )
  {
    trMat = g_nspt4x32[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 32 && height == 4 )
  {
    trMat = g_nspt32x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 32 )
  {
    trMat = g_nspt8x32[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 32 && height == 8 )
  {
    trMat = g_nspt32x8[ mode ][ nsptIdx ][ 0 ];
  }
#endif
#endif

  int shift = shift_1st + shift_2nd - 7;
  int rnd = 1 << ( shift - 1 );

  const __m128i vrnd = _mm_set1_epi32( rnd );
  __m128i vmat[ 4 ], vcoef[ 4 ], vsrc;

  int trSize2 = ( trSize << 1 );
  int trSize3 = trSize2 + trSize;
  int trSize4 = ( trSize << 2 );
  for( int j = 0; j < zeroOutSize; j += 4 )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    const int8_t* trMatTmp[ 4 ] = { trMat, trMat + trSize, trMat + trSize2, trMat + trSize3 };
    vcoef[ 0 ] = vcoef[ 1 ] = vcoef[ 2 ] = vcoef[ 3 ] = _mm_setzero_si128();
    for( int i = 0; i < trSize; i += 4 )
    {
      vsrc = _mm_loadu_si128( ( const __m128i* )srcPtr );
      for( int idx = 0; idx < 4; idx++ )
      {
        vmat[ idx ] = _mm_cvtepi8_epi32( _mm_cvtsi32_si128( *( unsigned int const* ) trMatTmp[ idx ] ) );
        vcoef[ idx ] = _mm_add_epi32( _mm_mullo_epi32( vsrc, vmat[ idx ] ), vcoef[ idx ] );
      }
      srcPtr += 4;
      trMatTmp[ 0 ] += 4; trMatTmp[ 1 ] += 4; trMatTmp[ 2 ] += 4; trMatTmp[ 3 ] += 4;
    }
    vcoef[ 0 ] = _mm_hadd_epi32( vcoef[ 0 ], vcoef[ 1 ] );
    vcoef[ 2 ] = _mm_hadd_epi32( vcoef[ 2 ], vcoef[ 3 ] );
    vcoef[ 0 ] = _mm_hadd_epi32( vcoef[ 0 ], vcoef[ 2 ] );
    vcoef[ 0 ] = _mm_srai_epi32( _mm_add_epi32( vcoef[ 0 ], vrnd ), shift );
    _mm_storeu_si128( ( __m128i * )dst, vcoef[ 0 ] );

    dst += 4;
    trMat += trSize4;
  }
}

template <X86_VEXT vext>
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void computeInvNspt_SIMD( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd,
  const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
                             , int bktIdx
#endif
)
{
#else
void TrQuant::computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
                             , int bktIdx
#endif
{
  int             maxLog2TrDynamicRange = 15;
#endif
  int trSize = width * height;

  const TCoeff    outputMinimum = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum = ( 1 << maxLog2TrDynamicRange ) - 1;

#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  const int8_t* trMat = TrQuant::getNsptMatrix(mode, width, height, nsptIdx, bktIdx);
#else
  const int8_t* trMat = g_nspt4x4[ mode ][ nsptIdx ][ 0 ];
  if( width == 8 && height == 8 )
  {
    trMat = g_nspt8x8[ mode ][ nsptIdx ][ 0 ];
  }
  if( width == 4 && height == 8 )
  {
    trMat = g_nspt4x8[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 4 )
  {
    trMat = g_nspt8x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 4 && height == 16 )
  {
    trMat = g_nspt4x16[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 16 && height == 4 )
  {
    trMat = g_nspt16x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 16 )
  {
    trMat = g_nspt8x16[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 16 && height == 8 )
  {
    trMat = g_nspt16x8[ mode ][ nsptIdx ][ 0 ];
  }
#if JVET_AE0086_LARGE_NSPT
  else if( width == 4 && height == 32 )
  {
    trMat = g_nspt4x32[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 32 && height == 4 )
  {
    trMat = g_nspt32x4[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 8 && height == 32 )
  {
    trMat = g_nspt8x32[ mode ][ nsptIdx ][ 0 ];
  }
  else if( width == 32 && height == 8 )
  {
    trMat = g_nspt32x8[ mode ][ nsptIdx ][ 0 ];
  }
#endif
#endif

  int shift = shift_1st + shift_2nd - 7;
  int rnd = 1 << ( shift - 1 );

  const __m128i vrnd = _mm_set1_epi32( rnd );
  __m128i vmat[ 4 ], vcoef[ 4 ], vsrc;
  __m128i vmin = _mm_set1_epi32( outputMinimum );
  __m128i vmax = _mm_set1_epi32( outputMaximum );

  int trSize2 = ( trSize << 1 );
  int trSize3 = trSize2 + trSize;
  int trSize4 = ( trSize << 2 );
  for( int j = 0; j < trSize; j += 4 )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    const int8_t* trMatTmp[ 4 ] = { trMat, trMat + 1, trMat + 2, trMat + 3 };
    vcoef[ 0 ] = vcoef[ 1 ] = vcoef[ 2 ] = vcoef[ 3 ] = _mm_setzero_si128();
    for( int i = 0; i < zeroOutSize; i += 4 )
    {
      vsrc = _mm_loadu_si128( ( const __m128i* )srcPtr );
      for( int idx = 0; idx < 4; idx++ )
      {
        vmat[ idx ] = _mm_set_epi32( *( trMatTmp[ idx ] + trSize3 ), *( trMatTmp[ idx ] + trSize2 ), *( trMatTmp[ idx ] + trSize ), *trMatTmp[ idx ] );
        vcoef[ idx ] = _mm_add_epi32( _mm_mullo_epi32( vsrc, vmat[ idx ] ), vcoef[ idx ] );
      }
      srcPtr += 4;
      trMatTmp[ 0 ] += trSize4; trMatTmp[ 1 ] += trSize4; trMatTmp[ 2 ] += trSize4; trMatTmp[ 3 ] += trSize4;
    }
    vcoef[ 0 ] = _mm_hadd_epi32( vcoef[ 0 ], vcoef[ 1 ] );
    vcoef[ 2 ] = _mm_hadd_epi32( vcoef[ 2 ], vcoef[ 3 ] );
    vcoef[ 0 ] = _mm_hadd_epi32( vcoef[ 0 ], vcoef[ 2 ] );
    vcoef[ 0 ] = _mm_srai_epi32( _mm_add_epi32( vcoef[ 0 ], vrnd ), shift );
    vcoef[ 0 ] = _mm_min_epi32( vmax, _mm_max_epi32( vmin, vcoef[ 0 ] ) );
    _mm_storeu_si128( ( __m128i * )dst, vcoef[ 0 ] );

    dst += 4;
    trMat += 4;
  }
}
#endif
#if TRANSFORM_SIMD_OPT
template <X86_VEXT vext>
void TrQuant::_initTrQuantX86()
{
#if INTRA_TRANS_ENC_OPT 
  m_fwdLfnst = forwardLfnst_SIMD<vext>;
  m_invLfnst = inverseLfnst_SIMD<vext>;
#endif
#if JVET_AC0130_NSPT && INTRA_TRANS_ENC_OPT
  m_computeFwdNspt = computeFwdNspt_SIMD<vext>;
  m_computeInvNspt = computeInvNspt_SIMD<vext>;
#endif
#if TRANSFORM_SIMD_OPT
#if TU_256
  m_forwardTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_FORWARD][0], g_trCoreDCT2P4[TRANSFORM_FORWARD][0], g_trCoreDCT2P8[TRANSFORM_FORWARD][0], g_trCoreDCT2P16[TRANSFORM_FORWARD][0], g_trCoreDCT2P32[TRANSFORM_FORWARD][0], g_trCoreDCT2P64[TRANSFORM_FORWARD][0], g_trCoreDCT2P128[TRANSFORM_FORWARD][0], g_trCoreDCT2P256[0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_FORWARD][0], g_trCoreDCT8P8[TRANSFORM_FORWARD][0], g_trCoreDCT8P16[TRANSFORM_FORWARD][0], g_trCoreDCT8P32[TRANSFORM_FORWARD][0], g_trCoreDCT8P64[TRANSFORM_FORWARD][0], g_trCoreDCT8P128[TRANSFORM_FORWARD][0], g_trCoreDCT8P256[0] },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_FORWARD][0], g_trCoreDST7P8[TRANSFORM_FORWARD][0], g_trCoreDST7P16[TRANSFORM_FORWARD][0], g_trCoreDST7P32[TRANSFORM_FORWARD][0], g_trCoreDST7P64[TRANSFORM_FORWARD][0], g_trCoreDST7P128[TRANSFORM_FORWARD][0], g_trCoreDST7P256[0] },
#if JVET_W0103_INTRA_MTS
    { nullptr,                              g_aiTr4[DCT5][0], g_aiTr8[DCT5][0], g_aiTr16[DCT5][0], g_aiTr32[DCT5][0], g_aiTr64[DCT5][0], g_aiTr128[DCT5][0], g_aiTr256[DCT5][0] },
    { nullptr,                              g_aiTr4[DST4][0], g_aiTr8[DST4][0], g_aiTr16[DST4][0], g_aiTr32[DST4][0], g_aiTr64[DST4][0], g_aiTr128[DST4][0], g_aiTr256[DST4][0] },
    { nullptr,                              g_aiTr4[DST1][0], g_aiTr8[DST1][0], g_aiTr16[DST1][0], g_aiTr32[DST1][0], g_aiTr64[DST1][0], g_aiTr128[DST1][0], g_aiTr256[DST1][0] },
    { nullptr,                              g_aiTr4[IDTR][0], g_aiTr8[IDTR][0], g_aiTr16[IDTR][0], g_aiTr32[IDTR][0], g_aiTr64[IDTR][0], g_aiTr128[IDTR][0], g_aiTr256[IDTR][0] },
#if JVET_AA0133_INTER_MTS_OPT
    { nullptr,                              g_aiTr4[KLT0][0], g_aiTr8[KLT0][0], g_aiTr16[KLT0][0], nullptr, nullptr, nullptr, nullptr },
    { nullptr,                              g_aiTr4[KLT1][0], g_aiTr8[KLT1][0], g_aiTr16[KLT1][0], nullptr, nullptr, nullptr, nullptr },
#endif
#endif
  } };

  m_inverseTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_INVERSE][0], g_trCoreDCT2P4[TRANSFORM_INVERSE][0], g_trCoreDCT2P8[TRANSFORM_INVERSE][0], g_trCoreDCT2P16[TRANSFORM_INVERSE][0], g_trCoreDCT2P32[TRANSFORM_INVERSE][0], g_trCoreDCT2P64[TRANSFORM_INVERSE][0], g_trCoreDCT2P128[TRANSFORM_INVERSE][0], g_trCoreDCT2P256[0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_INVERSE][0], g_trCoreDCT8P8[TRANSFORM_INVERSE][0], g_trCoreDCT8P16[TRANSFORM_INVERSE][0], g_trCoreDCT8P32[TRANSFORM_INVERSE][0], g_trCoreDCT8P64[TRANSFORM_INVERSE][0], g_trCoreDCT8P128[TRANSFORM_INVERSE][0], g_trCoreDCT8P256[0] },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_INVERSE][0], g_trCoreDST7P8[TRANSFORM_INVERSE][0], g_trCoreDST7P16[TRANSFORM_INVERSE][0], g_trCoreDST7P32[TRANSFORM_INVERSE][0], g_trCoreDST7P64[TRANSFORM_INVERSE][0], g_trCoreDST7P128[TRANSFORM_INVERSE][0], g_trCoreDST7P256[0] },
#if JVET_W0103_INTRA_MTS
    { nullptr,                              g_aiTr4[DCT5][0], g_aiTr8[DCT5][0], g_aiTr16[DCT5][0], g_aiTr32[DCT5][0], g_aiTr64[DCT5][0], g_aiTr128[DCT5][0], g_aiTr256[DCT5][0] },
    { nullptr,                              g_aiTr4[DST4][0], g_aiTr8[DST4][0], g_aiTr16[DST4][0], g_aiTr32[DST4][0], g_aiTr64[DST4][0], g_aiTr128[DST4][0], g_aiTr256[DST4][0] },
    { nullptr,                              g_aiTr4[DST1][0], g_aiTr8[DST1][0], g_aiTr16[DST1][0], g_aiTr32[DST1][0], g_aiTr64[DST1][0], g_aiTr128[DST1][0], g_aiTr256[DST1][0] },
    { nullptr,                              g_aiTr4[IDTR][0], g_aiTr8[IDTR][0], g_aiTr16[IDTR][0], g_aiTr32[IDTR][0], g_aiTr64[IDTR][0], g_aiTr128[IDTR][0], g_aiTr256[IDTR][0] },
#if JVET_AA0133_INTER_MTS_OPT
    { nullptr,                              g_aiTr4[KLT0][0], g_aiTr8[KLT0][0], g_aiTr16[KLT0][0], nullptr, nullptr, nullptr, nullptr },
    { nullptr,                              g_aiTr4[KLT1][0], g_aiTr8[KLT1][0], g_aiTr16[KLT1][0], nullptr, nullptr, nullptr, nullptr },
#endif
#endif
  } };
  
  fastFwdTrans[0][0] = fastForwardTransform_SIMD<DCT2, 2>;
  fastFwdTrans[0][1] = fastForwardTransform_SIMD<DCT2, 4>;
  fastFwdTrans[0][2] = fastForwardTransform_SIMD<DCT2, 8>;
  fastFwdTrans[0][3] = fastForwardTransform_SIMD<DCT2, 16>;
  fastFwdTrans[0][4] = fastForwardTransform_SIMD<DCT2, 32>;
  fastFwdTrans[0][5] = fastForwardTransform_SIMD<DCT2, 64>;
  fastFwdTrans[0][6] = fastForwardTransform_SIMD<DCT2, 128>;
  fastFwdTrans[0][7] = fastForwardTransform_SIMD<DCT2, 256>;

  fastFwdTrans[1][0] = nullptr;
  fastFwdTrans[1][1] = fastForwardTransform_SIMD<DCT8, 4>;
  fastFwdTrans[1][2] = fastForwardTransform_SIMD<DCT8, 8>;
  fastFwdTrans[1][3] = fastForwardTransform_SIMD<DCT8, 16>;
  fastFwdTrans[1][4] = fastForwardTransform_SIMD<DCT8, 32>;
  fastFwdTrans[1][5] = fastForwardTransform_SIMD<DCT8, 64>;
  fastFwdTrans[1][6] = fastForwardTransform_SIMD<DCT8, 128>;
  fastFwdTrans[1][7] = fastForwardTransform_SIMD<DCT8, 256>;

  fastFwdTrans[2][0] = nullptr;
  fastFwdTrans[2][1] = fastForwardTransform_SIMD<DST7, 4>;
  fastFwdTrans[2][2] = fastForwardTransform_SIMD<DST7, 8>;
  fastFwdTrans[2][3] = fastForwardTransform_SIMD<DST7, 16>;
  fastFwdTrans[2][4] = fastForwardTransform_SIMD<DST7, 32>;
  fastFwdTrans[2][5] = fastForwardTransform_SIMD<DST7, 64>;
  fastFwdTrans[2][6] = fastForwardTransform_SIMD<DST7, 128>;
  fastFwdTrans[2][7] = fastForwardTransform_SIMD<DST7, 256>;

#if JVET_W0103_INTRA_MTS
  fastFwdTrans[3][0] = nullptr;
  fastFwdTrans[3][1] = fastForwardTransform_SIMD<DCT5, 4>;
  fastFwdTrans[3][2] = fastForwardTransform_SIMD<DCT5, 8>;
  fastFwdTrans[3][3] = fastForwardTransform_SIMD<DCT5, 16>;
  fastFwdTrans[3][4] = fastForwardTransform_SIMD<DCT5, 32>;
  fastFwdTrans[3][5] = fastForwardTransform_SIMD<DCT5, 64>;
  fastFwdTrans[3][6] = fastForwardTransform_SIMD<DCT5, 128>;
  fastFwdTrans[3][7] = fastForwardTransform_SIMD<DCT5, 256>;

  fastFwdTrans[4][0] = nullptr;
  fastFwdTrans[4][1] = fastForwardTransform_SIMD<DST4, 4>;
  fastFwdTrans[4][2] = fastForwardTransform_SIMD<DST4, 8>;
  fastFwdTrans[4][3] = fastForwardTransform_SIMD<DST4, 16>;
  fastFwdTrans[4][4] = fastForwardTransform_SIMD<DST4, 32>;
  fastFwdTrans[4][5] = fastForwardTransform_SIMD<DST4, 64>;
  fastFwdTrans[4][6] = fastForwardTransform_SIMD<DST4, 128>;
  fastFwdTrans[4][7] = fastForwardTransform_SIMD<DST4, 256>;

  fastFwdTrans[5][0] = nullptr;
  fastFwdTrans[5][1] = fastForwardTransform_SIMD<DST1, 4>;
  fastFwdTrans[5][2] = fastForwardTransform_SIMD<DST1, 8>;
  fastFwdTrans[5][3] = fastForwardTransform_SIMD<DST1, 16>;
  fastFwdTrans[5][4] = fastForwardTransform_SIMD<DST1, 32>;
  fastFwdTrans[5][5] = fastForwardTransform_SIMD<DST1, 64>;
  fastFwdTrans[5][6] = fastForwardTransform_SIMD<DST1, 128>;
  fastFwdTrans[5][7] = fastForwardTransform_SIMD<DST1, 256>;

  fastFwdTrans[6][0] = nullptr;
  fastFwdTrans[6][1] = fastForwardTransform_SIMD<IDTR, 4>;
  fastFwdTrans[6][2] = fastForwardTransform_SIMD<IDTR, 8>;
  fastFwdTrans[6][3] = fastForwardTransform_SIMD<IDTR, 16>;
  fastFwdTrans[6][4] = fastForwardTransform_SIMD<IDTR, 32>;
  fastFwdTrans[6][5] = fastForwardTransform_SIMD<IDTR, 64>;
  fastFwdTrans[6][6] = fastForwardTransform_SIMD<IDTR, 128>;
  fastFwdTrans[6][7] = fastForwardTransform_SIMD<IDTR, 256>;
#if JVET_AA0133_INTER_MTS_OPT
  fastFwdTrans[7][0] = nullptr;
  fastFwdTrans[7][1] = fastForwardTransform_SIMD<KLT0, 4>;
  fastFwdTrans[7][2] = fastForwardTransform_SIMD<KLT0, 8>;
  fastFwdTrans[7][3] = fastForwardTransform_SIMD<KLT0, 16>;
  fastFwdTrans[7][4] = nullptr;
  fastFwdTrans[7][5] = nullptr;
  fastFwdTrans[7][6] = nullptr;
  fastFwdTrans[7][7] = nullptr;

  fastFwdTrans[8][0] = nullptr;
  fastFwdTrans[8][1] = fastForwardTransform_SIMD<KLT1, 4>;
  fastFwdTrans[8][2] = fastForwardTransform_SIMD<KLT1, 8>;
  fastFwdTrans[8][3] = fastForwardTransform_SIMD<KLT1, 16>;
  fastFwdTrans[8][4] = nullptr;
  fastFwdTrans[8][5] = nullptr;
  fastFwdTrans[8][6] = nullptr;
  fastFwdTrans[8][7] = nullptr;
#endif
#endif

  fastInvTrans[0][0] = fastInverseTransform_SIMD<DCT2, 2>;
  fastInvTrans[0][1] = fastInverseTransform_SIMD<DCT2, 4>;
  fastInvTrans[0][2] = fastInverseTransform_SIMD<DCT2, 8>;
  fastInvTrans[0][3] = fastInverseTransform_SIMD<DCT2, 16>;
  fastInvTrans[0][4] = fastInverseTransform_SIMD<DCT2, 32>;
  fastInvTrans[0][5] = fastInverseTransform_SIMD<DCT2, 64>;
  fastInvTrans[0][6] = fastInverseTransform_SIMD<DCT2, 128>;
  fastInvTrans[0][7] = fastInverseTransform_SIMD<DCT2, 256>;

  fastInvTrans[1][0] = nullptr;
  fastInvTrans[1][1] = fastInverseTransform_SIMD<DCT8, 4>;
  fastInvTrans[1][2] = fastInverseTransform_SIMD<DCT8, 8>;
  fastInvTrans[1][3] = fastInverseTransform_SIMD<DCT8, 16>;
  fastInvTrans[1][4] = fastInverseTransform_SIMD<DCT8, 32>;
  fastInvTrans[1][5] = fastInverseTransform_SIMD<DCT8, 64>;
  fastInvTrans[1][6] = fastInverseTransform_SIMD<DCT8, 128>;
  fastInvTrans[1][7] = fastInverseTransform_SIMD<DCT8, 256>;

  fastInvTrans[2][0] = nullptr;
  fastInvTrans[2][1] = fastInverseTransform_SIMD<DST7, 4>;
  fastInvTrans[2][2] = fastInverseTransform_SIMD<DST7, 8>;
  fastInvTrans[2][3] = fastInverseTransform_SIMD<DST7, 16>;
  fastInvTrans[2][4] = fastInverseTransform_SIMD<DST7, 32>;
  fastInvTrans[2][5] = fastInverseTransform_SIMD<DST7, 64>;
  fastInvTrans[2][6] = fastInverseTransform_SIMD<DST7, 128>;
  fastInvTrans[2][7] = fastInverseTransform_SIMD<DST7, 256>;

#if JVET_W0103_INTRA_MTS
  fastInvTrans[3][0] = nullptr;
  fastInvTrans[3][1] = fastInverseTransform_SIMD<DCT5, 4>;
  fastInvTrans[3][2] = fastInverseTransform_SIMD<DCT5, 8>;
  fastInvTrans[3][3] = fastInverseTransform_SIMD<DCT5, 16>;
  fastInvTrans[3][4] = fastInverseTransform_SIMD<DCT5, 32>;
  fastInvTrans[3][5] = fastInverseTransform_SIMD<DCT5, 64>;
  fastInvTrans[3][6] = fastInverseTransform_SIMD<DCT5, 128>;
  fastInvTrans[3][7] = fastInverseTransform_SIMD<DCT5, 256>;

  fastInvTrans[4][0] = nullptr;
  fastInvTrans[4][1] = fastInverseTransform_SIMD<DST4, 4>;
  fastInvTrans[4][2] = fastInverseTransform_SIMD<DST4, 8>;
  fastInvTrans[4][3] = fastInverseTransform_SIMD<DST4, 16>;
  fastInvTrans[4][4] = fastInverseTransform_SIMD<DST4, 32>;
  fastInvTrans[4][5] = fastInverseTransform_SIMD<DST4, 64>;
  fastInvTrans[4][6] = fastInverseTransform_SIMD<DST4, 128>;
  fastInvTrans[4][7] = fastInverseTransform_SIMD<DST4, 256>;

  fastInvTrans[5][0] = nullptr;
  fastInvTrans[5][1] = fastInverseTransform_SIMD<DST1, 4>;
  fastInvTrans[5][2] = fastInverseTransform_SIMD<DST1, 8>;
  fastInvTrans[5][3] = fastInverseTransform_SIMD<DST1, 16>;
  fastInvTrans[5][4] = fastInverseTransform_SIMD<DST1, 32>;
  fastInvTrans[5][5] = fastInverseTransform_SIMD<DST1, 64>;
  fastInvTrans[5][6] = fastInverseTransform_SIMD<DST1, 128>;
  fastInvTrans[5][7] = fastInverseTransform_SIMD<DST1, 256>;

  fastInvTrans[6][0] = nullptr;
  fastInvTrans[6][1] = fastInverseTransform_SIMD<IDTR, 4>;
  fastInvTrans[6][2] = fastInverseTransform_SIMD<IDTR, 8>;
  fastInvTrans[6][3] = fastInverseTransform_SIMD<IDTR, 16>;
  fastInvTrans[6][4] = fastInverseTransform_SIMD<IDTR, 32>;
  fastInvTrans[6][5] = fastInverseTransform_SIMD<IDTR, 64>;
  fastInvTrans[6][6] = fastInverseTransform_SIMD<IDTR, 128>;
  fastInvTrans[6][7] = fastInverseTransform_SIMD<IDTR, 256>;
#if JVET_AA0133_INTER_MTS_OPT
  fastInvTrans[7][0] = nullptr;
  fastInvTrans[7][1] = fastInverseTransform_SIMD<KLT0, 4>;
  fastInvTrans[7][2] = fastInverseTransform_SIMD<KLT0, 8>;
  fastInvTrans[7][3] = fastInverseTransform_SIMD<KLT0, 16>;
  fastInvTrans[7][4] = nullptr;
  fastInvTrans[7][5] = nullptr;
  fastInvTrans[7][6] = nullptr;
  fastInvTrans[7][7] = nullptr;

  fastInvTrans[8][0] = nullptr;
  fastInvTrans[8][1] = fastInverseTransform_SIMD<KLT1, 4>;
  fastInvTrans[8][2] = fastInverseTransform_SIMD<KLT1, 8>;
  fastInvTrans[8][3] = fastInverseTransform_SIMD<KLT1, 16>;
  fastInvTrans[8][4] = nullptr;
  fastInvTrans[8][5] = nullptr;
  fastInvTrans[8][6] = nullptr;
  fastInvTrans[8][7] = nullptr;
#endif
#endif
#else
  m_forwardTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_FORWARD][0], g_trCoreDCT2P4[TRANSFORM_FORWARD][0], g_trCoreDCT2P8[TRANSFORM_FORWARD][0], g_trCoreDCT2P16[TRANSFORM_FORWARD][0], g_trCoreDCT2P32[TRANSFORM_FORWARD][0], g_trCoreDCT2P64[TRANSFORM_FORWARD][0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_FORWARD][0], g_trCoreDCT8P8[TRANSFORM_FORWARD][0], g_trCoreDCT8P16[TRANSFORM_FORWARD][0], g_trCoreDCT8P32[TRANSFORM_FORWARD][0], nullptr },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_FORWARD][0], g_trCoreDST7P8[TRANSFORM_FORWARD][0], g_trCoreDST7P16[TRANSFORM_FORWARD][0], g_trCoreDST7P32[TRANSFORM_FORWARD][0], nullptr },
  } };

  m_inverseTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_INVERSE][0], g_trCoreDCT2P4[TRANSFORM_INVERSE][0], g_trCoreDCT2P8[TRANSFORM_INVERSE][0], g_trCoreDCT2P16[TRANSFORM_INVERSE][0], g_trCoreDCT2P32[TRANSFORM_INVERSE][0], g_trCoreDCT2P64[TRANSFORM_INVERSE][0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_INVERSE][0], g_trCoreDCT8P8[TRANSFORM_INVERSE][0], g_trCoreDCT8P16[TRANSFORM_INVERSE][0], g_trCoreDCT8P32[TRANSFORM_INVERSE][0], nullptr },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_INVERSE][0], g_trCoreDST7P8[TRANSFORM_INVERSE][0], g_trCoreDST7P16[TRANSFORM_INVERSE][0], g_trCoreDST7P32[TRANSFORM_INVERSE][0], nullptr },
  } };

  fastFwdTrans[0][0] = fastForwardTransform_SIMD<DCT2, 2>;
  fastFwdTrans[0][1] = fastForwardTransform_SIMD<DCT2, 4>;
  fastFwdTrans[0][2] = fastForwardTransform_SIMD<DCT2, 8>;
  fastFwdTrans[0][3] = fastForwardTransform_SIMD<DCT2, 16>;
  fastFwdTrans[0][4] = fastForwardTransform_SIMD<DCT2, 32>;
  fastFwdTrans[0][5] = fastForwardTransform_SIMD<DCT2, 64>;

  fastFwdTrans[1][0] = nullptr;
  fastFwdTrans[1][1] = fastForwardTransform_SIMD<DCT8, 4>;
  fastFwdTrans[1][2] = fastForwardTransform_SIMD<DCT8, 8>;
  fastFwdTrans[1][3] = fastForwardTransform_SIMD<DCT8, 16>;
  fastFwdTrans[1][4] = fastForwardTransform_SIMD<DCT8, 32>;
  fastFwdTrans[1][5] = fastForwardTransform_SIMD<DCT8, 64>;

  fastFwdTrans[2][0] = nullptr;
  fastFwdTrans[2][1] = fastForwardTransform_SIMD<DST7, 4>;
  fastFwdTrans[2][2] = fastForwardTransform_SIMD<DST7, 8>;
  fastFwdTrans[2][3] = fastForwardTransform_SIMD<DST7, 16>;
  fastFwdTrans[2][4] = fastForwardTransform_SIMD<DST7, 32>;
  fastFwdTrans[2][5] = fastForwardTransform_SIMD<DST7, 64>;

  fastInvTrans[0][0] = fastInverseTransform_SIMD<DCT2, 2>;
  fastInvTrans[0][1] = fastInverseTransform_SIMD<DCT2, 4>;
  fastInvTrans[0][2] = fastInverseTransform_SIMD<DCT2, 8>;
  fastInvTrans[0][3] = fastInverseTransform_SIMD<DCT2, 16>;
  fastInvTrans[0][4] = fastInverseTransform_SIMD<DCT2, 32>;
  fastInvTrans[0][5] = fastInverseTransform_SIMD<DCT2, 64>;

  fastInvTrans[1][0] = nullptr;
  fastInvTrans[1][1] = fastInverseTransform_SIMD<DCT8, 4>;
  fastInvTrans[1][2] = fastInverseTransform_SIMD<DCT8, 8>;
  fastInvTrans[1][3] = fastInverseTransform_SIMD<DCT8, 16>;
  fastInvTrans[1][4] = fastInverseTransform_SIMD<DCT8, 32>;
  fastInvTrans[1][5] = fastInverseTransform_SIMD<DCT8, 64>;

  fastInvTrans[2][0] = nullptr;
  fastInvTrans[2][1] = fastInverseTransform_SIMD<DST7, 4>;
  fastInvTrans[2][2] = fastInverseTransform_SIMD<DST7, 8>;
  fastInvTrans[2][3] = fastInverseTransform_SIMD<DST7, 16>;
  fastInvTrans[2][4] = fastInverseTransform_SIMD<DST7, 32>;
  fastInvTrans[2][5] = fastInverseTransform_SIMD<DST7, 64>;
#endif
#endif
}

template void TrQuant::_initTrQuantX86<SIMDX86>();
#endif

#endif //#ifdef TARGET_SIMD_X86
//! \}
