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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "Reshape.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "QuantRDOQ.h"
#include "DepQuant.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};

//! \ingroup CommonLib
//! \{

static inline int64_t square( const int d ) { return d * (int64_t)d; }

template<int signedMode> std::pair<int64_t,int64_t> fwdTransformCbCr( const PelBuf &resCb, const PelBuf &resCr, PelBuf& resC1, PelBuf& resC2 )
{
  const Pel*  cb  = resCb.buf;
  const Pel*  cr  = resCr.buf;
  Pel*        c1  = resC1.buf;
  Pel*        c2  = resC2.buf;
  int64_t     d1  = 0;
  int64_t     d2  = 0;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride, c1 += resC1.stride, c2 += resC2.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      int cbx = cb[x], crx = cr[x];
      if      ( signedMode ==  1 )
      {
        c1[x] = Pel( ( 4*cbx + 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (c1[x]>>1) );
      }
      else if ( signedMode == -1 )
      {
        c1[x] = Pel( ( 4*cbx - 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (-c1[x]>>1) );
      }
      else if ( signedMode ==  2 )
      {
        c1[x] = Pel( ( cbx + crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx - c1[x] );
      }
      else if ( signedMode == -2 )
      {
        c1[x] = Pel( ( cbx - crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx + c1[x] );
      }
      else if ( signedMode ==  3 )
      {
        c2[x] = Pel( ( 4*crx + 2*cbx ) / 5 );
        d1   += square( cbx - (c2[x]>>1) ) + square( crx - c2[x] );
      }
      else if ( signedMode == -3 )
      {
        c2[x] = Pel( ( 4*crx - 2*cbx ) / 5 );
        d1   += square( cbx - (-c2[x]>>1) ) + square( crx - c2[x] );
      }
      else
      {
        d1   += square( cbx );
        d2   += square( crx );
      }
    }
  }
  return std::make_pair(d1,d2);
}

template<int signedMode> void invTransformCbCr( PelBuf &resCb, PelBuf &resCr )
{
  Pel*  cb  = resCb.buf;
  Pel*  cr  = resCr.buf;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      if (signedMode == 1)
      {
        cr[x] = cb[x] >> 1;
      }
      else if (signedMode == -1)
      {
        cr[x] = -cb[x] >> 1;
      }
      else if (signedMode == 2)
      {
        cr[x] = cb[x];
      }
      else if (signedMode == -2)
      {
        // non-normative clipping to prevent 16-bit overflow
        cr[x] = (cb[x] == -32768 && sizeof(Pel) == 2) ? 32767 : -cb[x];
      }
      else if (signedMode == 3)
      {
        cb[x] = cr[x] >> 1;
      }
      else if (signedMode == -3)
      {
        cb[x] = -cr[x] >> 1;
      }
    }
  }
}

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
#if TRANSFORM_SIMD_OPT
std::array<std::array<const TMatrixCoeff*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> TrQuant::m_forwardTransformKernels;
std::array<std::array<const TMatrixCoeff*, g_numTransformMatrixSizes>, NUM_TRANS_TYPE> TrQuant::m_inverseTransformKernels;
#endif

TrQuant::TrQuant() : m_quant( nullptr )
{
  // allocate temporary buffers
  {
    m_invICT      = m_invICTMem + maxAbsIctMode;
    m_invICT[ 0]  = invTransformCbCr< 0>;
    m_invICT[ 1]  = invTransformCbCr< 1>;
    m_invICT[-1]  = invTransformCbCr<-1>;
    m_invICT[ 2]  = invTransformCbCr< 2>;
    m_invICT[-2]  = invTransformCbCr<-2>;
    m_invICT[ 3]  = invTransformCbCr< 3>;
    m_invICT[-3]  = invTransformCbCr<-3>;
    m_fwdICT      = m_fwdICTMem + maxAbsIctMode;
    m_fwdICT[ 0]  = fwdTransformCbCr< 0>;
    m_fwdICT[ 1]  = fwdTransformCbCr< 1>;
    m_fwdICT[-1]  = fwdTransformCbCr<-1>;
    m_fwdICT[ 2]  = fwdTransformCbCr< 2>;
    m_fwdICT[-2]  = fwdTransformCbCr<-2>;
    m_fwdICT[ 3]  = fwdTransformCbCr< 3>;
    m_fwdICT[-3]  = fwdTransformCbCr<-3>;
  }
}

TrQuant::~TrQuant()
{
  if( m_quant )
  {
    delete m_quant;
    m_quant = nullptr;
  }
}

#if ENABLE_SPLIT_PARALLELISM
void TrQuant::copyState( const TrQuant& other )
{
  m_quant->copyState( *other.m_quant );
}
#endif

void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  m_quant->dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( const Quant* otherQuant,
                    const uint32_t uiMaxTrSize,
                    const bool bUseRDOQ,
                    const bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ,
#endif
                    const bool bEnc
)
{
  delete m_quant;
  m_quant = nullptr;

  m_quant = new DepQuant(otherQuant, bEnc);

  if( m_quant )
  {
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ );
  }

#if TU_256
  fastFwdTrans =
  { {
    { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64, fastForwardDCT2_B128, fastForwardDCT2_B256 },
    { nullptr,            fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, fastForwardDCT8_B64, fastForwardDCT8_B128, fastForwardDCT8_B256 },
    { nullptr,            fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, fastForwardDST7_B64, fastForwardDST7_B128, fastForwardDST7_B256 },
#if JVET_W0103_INTRA_MTS
    { nullptr,            fastForwardDCT5_B4, fastForwardDCT5_B8, fastForwardDCT5_B16, fastForwardDCT5_B32, fastForwardDCT5_B64, fastForwardDCT5_B128, fastForwardDCT5_B256 },
    { nullptr,            fastForwardDST4_B4, fastForwardDST4_B8, fastForwardDST4_B16, fastForwardDST4_B32, fastForwardDST4_B64, fastForwardDST4_B128, fastForwardDST4_B256 },
    { nullptr,            fastForwardDST1_B4, fastForwardDST1_B8, fastForwardDST1_B16, fastForwardDST1_B32, fastForwardDST1_B64, fastForwardDST1_B128, fastForwardDST1_B256 },
    { nullptr,            fastForwardIDTR_B4, fastForwardIDTR_B8, fastForwardIDTR_B16, fastForwardIDTR_B32, fastForwardIDTR_B64, fastForwardIDTR_B128, fastForwardIDTR_B256 },
#if JVET_AA0133_INTER_MTS_OPT
    {nullptr,             fastForwardKLT0_B4, fastForwardKLT0_B8, fastForwardKLT0_B16,  nullptr,             nullptr,             nullptr,              nullptr             },
    {nullptr,             fastForwardKLT1_B4, fastForwardKLT1_B8, fastForwardKLT1_B16,  nullptr,             nullptr,             nullptr,              nullptr             },
#endif
#endif
  } };

  fastInvTrans =
  { {
    { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64, fastInverseDCT2_B128, fastInverseDCT2_B256 },
    { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, fastInverseDCT8_B64, fastInverseDCT8_B128, fastInverseDCT8_B256 },
    { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, fastInverseDST7_B64, fastInverseDST7_B128, fastInverseDST7_B256 },
#if JVET_W0103_INTRA_MTS
    { nullptr,            fastInverseDCT5_B4, fastInverseDCT5_B8, fastInverseDCT5_B16, fastInverseDCT5_B32, fastInverseDCT5_B64, fastInverseDCT5_B128, fastInverseDCT5_B256 },
    { nullptr,            fastInverseDST4_B4, fastInverseDST4_B8, fastInverseDST4_B16, fastInverseDST4_B32, fastInverseDST4_B64, fastInverseDST4_B128, fastInverseDST4_B256 },
    { nullptr,            fastInverseDST1_B4, fastInverseDST1_B8, fastInverseDST1_B16, fastInverseDST1_B32, fastInverseDST1_B64, fastInverseDST1_B128, fastInverseDST1_B256 },
    { nullptr,            fastInverseIDTR_B4, fastInverseIDTR_B8, fastInverseIDTR_B16, fastInverseIDTR_B32, fastInverseIDTR_B64, fastInverseIDTR_B128, fastInverseIDTR_B256 },
#if JVET_AA0133_INTER_MTS_OPT
    {nullptr,             fastInverseKLT0_B4, fastInverseKLT0_B8, fastInverseKLT0_B16,  nullptr,             nullptr,             nullptr,              nullptr             },
    {nullptr,             fastInverseKLT1_B4, fastInverseKLT1_B8, fastInverseKLT1_B16,  nullptr,             nullptr,             nullptr,              nullptr             },
#endif
#endif
  } };
#else
  fastFwdTrans =
  { {
    { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64 },
    { nullptr,            fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, nullptr },
    { nullptr,            fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, nullptr },
#if JVET_W0103_INTRA_MTS
    { nullptr,            fastForwardDCT5_B4, fastForwardDCT5_B8, fastForwardDCT5_B16, fastForwardDCT5_B32, nullptr },
    { nullptr,            fastForwardDST4_B4, fastForwardDST4_B8, fastForwardDST4_B16, fastForwardDST4_B32, nullptr },
    { nullptr,            fastForwardDST1_B4, fastForwardDST1_B8, fastForwardDST1_B16, fastForwardDST1_B32, nullptr },
    { nullptr,            fastForwardIDTR_B4, fastForwardIDTR_B8, fastForwardIDTR_B16, fastForwardIDTR_B32, nullptr },
#endif
  } };

  fastInvTrans =
  { {
    { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
    { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
    { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
#if JVET_W0103_INTRA_MTS
    { nullptr,            fastInverseDCT5_B4, fastInverseDCT5_B8, fastInverseDCT5_B16, fastInverseDCT5_B32, nullptr },
    { nullptr,            fastInverseDST4_B4, fastInverseDST4_B8, fastInverseDST4_B16, fastInverseDST4_B32, nullptr },
    { nullptr,            fastInverseDST1_B4, fastInverseDST1_B8, fastInverseDST1_B16, fastInverseDST1_B32, nullptr },
    { nullptr,            fastInverseIDTR_B4, fastInverseIDTR_B8, fastInverseIDTR_B16, fastInverseIDTR_B32, nullptr },
#endif
  } };
#endif

#if INTRA_TRANS_ENC_OPT
  m_fwdLfnst = forwardLfnst;
  m_invLfnst = inverseLfnst;
#endif
#if JVET_AC0130_NSPT && INTRA_TRANS_ENC_OPT
  m_computeFwdNspt = computeFwdNspt;
  m_computeInvNspt = computeInvNspt;
#endif
#if TRANSFORM_SIMD_OPT
#ifdef TARGET_SIMD_X86
  initTrQuantX86();
#endif
#endif
}

#if AHG7_LN_TOOLOFF_CFG
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::fwdLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag )
#else
void TrQuant::fwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag )
#endif
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::fwdLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
#else
void TrQuant::fwdLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
#endif
#endif
{
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
  const int8_t* trMat  = lfnstExtFlag ? ( ( size > 8 ) ? g_lfnst16x16[ mode ][ index ][ 0 ] : ( ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ] ) )
                                      : ( ( size > 4 ) ? g_vvcLfnst8x8[ mode ][ index ][ 0 ] : g_vvcLfnst4x4[ mode ][ index ][ 0 ] );
  const int     trSize = lfnstExtFlag ? ( ( size > 8 ) ? L16W_ZO : ( ( size > 4 ) ? L8W_ZO : 16 ) )
                                      : ( ( size > 4 ) ? 48 : 16 );
#else
  const int8_t* trMat  = ( size > 8 ) ? g_lfnst16x16[ mode ][ index ][ 0 ] : ( ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ] );
  const int     trSize = ( size > 8 ) ? L16W_ZO : ( ( size > 4 ) ? L8W_ZO : 16 );
#endif
#else
  const int8_t* trMat  = ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
#if EXTENDED_LFNST
  const int     trSize = ( size > 4 ) ? 64 : 16;
#else
  const int     trSize = ( size > 4 ) ? 48 : 16;
#endif
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if !INTRA_TRANS_ENC_OPT 
  TCoeff        coef;
#endif
  TCoeff*       out    = dst;
#else
  int           coef;
  int*          out    = dst;
#endif  
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  CHECK( index >= 4, "Wrong index");
#else
  CHECK( index >= 3, "Wrong index" );
#endif

#if INTRA_TRANS_ENC_OPT
  m_fwdLfnst(src, out, trMat, trSize, zeroOutSize);
#else
  for( int j = 0; j < zeroOutSize; j++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*          srcPtr   = src;
#else
    int*          srcPtr   = src;
#endif
    const int8_t* trMatTmp = trMat;
    coef = 0;
    for( int i = 0; i < trSize; i++ )
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *out++ = ( coef + 64 ) >> 7;
    trMat += trSize;
  }
#endif
  ::memset( out, 0, ( trSize - zeroOutSize ) * sizeof( int ) );
}

#if AHG7_LN_TOOLOFF_CFG
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::invLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, const int maxLog2TrDynamicRange, bool lfnstExtFlag )
{
#else
void TrQuant::invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, bool lfnstExtFlag )
{
  int             maxLog2TrDynamicRange = 15;
#endif
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::invLfnstNxN( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize, const int maxLog2TrDynamicRange )
{
#else
void TrQuant::invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  int             maxLog2TrDynamicRange =  15;
#endif
#endif
  const TCoeff    outputMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;

#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
  const int8_t*   trMat  = lfnstExtFlag ? ( ( size > 8 ) ? g_lfnst16x16[ mode ][ index ][ 0 ] : ( ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ] ) )
                                        : ( ( size > 4 ) ? g_vvcLfnst8x8[ mode ][ index ][ 0 ] : g_vvcLfnst4x4[ mode ][ index ][ 0 ] );
  const int       trSize = lfnstExtFlag ? ( ( size > 8 ) ? L16W_ZO : ( ( size > 4 ) ? L8W_ZO : 16 ) )
                                        : ( ( size > 4 ) ? 48 : 16 );
#else
  const int8_t*   trMat  = ( size > 8 ) ? g_lfnst16x16[ mode ][ index ][ 0 ] : ( ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ] );
  const int       trSize = ( size > 8 ) ? L16W_ZO : ( ( size > 4 ) ? L8W_ZO : 16 );
#endif
#else
  const int8_t*   trMat                 =  ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
#if EXTENDED_LFNST
  const int       trSize                =  ( size > 4 ) ? 64 : 16;
#else
  const int       trSize                =  ( size > 4 ) ? 48 : 16;
#endif
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if !INTRA_TRANS_ENC_OPT 
  TCoeff          resi;
#endif
  TCoeff*         out                   =  dst;
#else
  int             resi;
  int*            out                   =  dst;
#endif  

#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  CHECK( index >= 4, "Wrong index" );
#else
  CHECK( index >= 3, "Wrong index" );
#endif
#if INTRA_TRANS_ENC_OPT 
  m_invLfnst(src, out, trMat, trSize, zeroOutSize, outputMinimum, outputMaximum);
#else
  for( int j = 0; j < trSize; j++ )
  {
    resi = 0;
    const int8_t* trMatTmp = trMat;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr   = src;
#else
    int*          srcPtr   = src;
#endif
    for( int i = 0; i < zeroOutSize; i++ )
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    *out++ = Clip3<TCoeff>( outputMinimum, outputMaximum, ( resi + 64 ) >> 7 );
#else
    *out++ = Clip3( outputMinimum, outputMaximum, ( int ) ( resi + 64 ) >> 7 );
#endif
    trMat++;
  }
#endif
}

bool TrQuant::getTransposeFlag( uint32_t intraMode )
{
  return ( ( intraMode >= NUM_LUMA_MODE ) && ( intraMode >= ( NUM_LUMA_MODE + ( NUM_EXT_LUMA_MODE >> 1 ) ) ) ) ||
         ( ( intraMode <  NUM_LUMA_MODE ) && ( intraMode >  DIA_IDX ) );
}

void TrQuant::xInvLfnst( const TransformUnit &tu, const ComponentID compID )
{
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
#endif
#if JVET_AL0181_ASBT
  const CompArea& area     = tu.blocksResidual[ compID ];
#else
  const CompArea& area     = tu.blocks[ compID ];
#endif
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (tu.cu->isSepTree() ? true : isLuma(compID)) )
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (tu.cu->separateTree ? true : isLuma(compID)))
#else
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (CS::isDualITree(*tu.cs) ? true : isLuma(compID)))
#endif
#endif
  {
#if JVET_AC0130_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
    bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                  ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
    bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
    bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, CU::isIntra( *( tu.cu ) ) );
#endif
    if( allowNSPT )
    {
      return;
    }
#endif
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    const bool whge4         = tu.cu->cs->sps->getUseLFNSTExt() ? PU::getUseLFNST16( width, height ) : false;
#else
    const bool whge4         = PU::getUseLFNST16( width, height );
#endif
    const bool whge3         = PU::getUseLFNST8 ( width, height );
    int widthIdx             = gp_sizeIdxInfo->idxFrom(  width );
    int heightIdx            = gp_sizeIdxInfo->idxFrom( height );
    const ScanElement * scan = whge4 ? g_coefTopLeftDiagScan16x16[ widthIdx ] : ( whge3 ? g_coefTopLeftDiagScan8x8[ widthIdx ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ widthIdx ][ heightIdx ] );
#else
    const bool whge3 = width >= 8 && height >= 8;
    const ScanElement * scan = whge3 ? g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
#endif
#if JVET_AK0217_INTRA_MTSS 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID ).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID );
#endif
#endif
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    if( lfnstIdx < ( tu.cu->cs->sps->getUseLFNSTExt() ? 4 : 3 ) )
#else
    if (lfnstIdx < 4)
#endif
#else
    if( lfnstIdx < 3 )
#endif
    {
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
      CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType { STATS__TOOL_LFNST, width, height, compID } );
#endif      
      bool          transposeFlag   = getTransposeFlag( intraMode );
#if JVET_W0119_LFNST_EXTENSION
      const int     sbSize          = whge4 ? 16 : ( whge3 ? 8 : 4 );
#else
      const int     sbSize          = whge3 ? 8 : 4;
#endif
#if !EXTENDED_LFNST && !JVET_W0119_LFNST_EXTENSION
      bool          tu4x4Flag       = ( width == 4 && height == 4 );
      bool          tu8x8Flag       = ( width == 8 && height == 8 );
#endif
      TCoeff*       lfnstTemp;
      TCoeff*       coeffTemp;
      int           y;
      lfnstTemp   = m_tempInMatrix;   // inverse spectral rearrangement
      coeffTemp   = m_tempCoeff;
      TCoeff *dst = lfnstTemp;

      const ScanElement *scanPtr = scan;
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
      int numLfnstCoeff = tu.cu->cs->sps->getUseLFNSTExt() ? ( whge4 ? L16H : ( whge3 ? L8H : 16 ) ) : 16;
#else
      int numLfnstCoeff = whge4 ? L16H : ( whge3 ? L8H : 16 );
#endif
      for( y = 0; y < numLfnstCoeff; y++ )
#else
#if EXTENDED_LFNST
      const int nSamples = sbSize * sbSize;
      for( y = 0; y < nSamples; y++ )
#else
      for (y = 0; y < 16; y++)
#endif
#endif
      {
        *dst++ = coeffTemp[ scanPtr->idx ];
        scanPtr++;
      }
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
      int zeroOutSize = PU::getLFNSTMatrixDim( width, height, tu.cu->cs->sps->getUseLFNSTExt() );
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, tu.cu->cs->sps->getUseLFNSTExt() ? g_lfnstLut[ intraMode ] : g_vvcLfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize, maxLog2TrDynamicRange, tu.cu->cs->sps->getUseLFNSTExt() );
#else
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, tu.cu->cs->sps->getUseLFNSTExt() ? g_lfnstLut[ intraMode ] : g_vvcLfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize, tu.cu->cs->sps->getUseLFNSTExt() );
#endif
#else
      int zeroOutSize = PU::getLFNSTMatrixDim( width, height );
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize, maxLog2TrDynamicRange );
#else
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize );
#endif
#endif
#else
#if EXTENDED_LFNST
      const int trSize = whge3 ? 64 : 16;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, trSize, maxLog2TrDynamicRange );
#else
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, trSize );
#endif          
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16, maxLog2TrDynamicRange );
#else
      invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );
#endif          
#endif
#endif
          lfnstTemp = m_tempOutMatrix; // inverse spectral rearrangement

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[4];
            coeffTemp[2] = lfnstTemp[8];
            coeffTemp[3] = lfnstTemp[12];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#if JVET_W0119_LFNST_EXTENSION
        else if( sbSize == 8 )
#else
        else   // ( sbSize == 8 )
#endif
        {
          for (y = 0; y < 8; y++)
          {
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[8];
            coeffTemp[2] = lfnstTemp[16];
            coeffTemp[3] = lfnstTemp[24];
            coeffTemp[4] = lfnstTemp[32];
            coeffTemp[5] = lfnstTemp[40];
            coeffTemp[6] = lfnstTemp[48];
            coeffTemp[7] = lfnstTemp[56];
#else
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[8];
            coeffTemp[2] = lfnstTemp[16];
            coeffTemp[3] = lfnstTemp[24];
            if (y < 4)
            {
              coeffTemp[4] = lfnstTemp[32];
              coeffTemp[5] = lfnstTemp[36];
              coeffTemp[6] = lfnstTemp[40];
              coeffTemp[7] = lfnstTemp[44];
            }
#endif
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#if JVET_W0119_LFNST_EXTENSION
        else // (sbSize == 16)
        {
          for( y = 0; y < 12; y++ )
          {
            coeffTemp[ 0 ] = lfnstTemp[  0 ];  coeffTemp[ 1 ] = lfnstTemp[ 12 ];
            coeffTemp[ 2 ] = lfnstTemp[ 24 ];  coeffTemp[ 3 ] = lfnstTemp[ 36 ];

            if( y < 8 )
            {
              coeffTemp[ 4 ] = lfnstTemp[ 48 ];  coeffTemp[ 5 ] = lfnstTemp[ 56 ];
              coeffTemp[ 6 ] = lfnstTemp[ 64 ];  coeffTemp[ 7 ] = lfnstTemp[ 72 ];
            }

            if( y < 4 )
            {
              coeffTemp[  8 ] = lfnstTemp[ 80 ];  coeffTemp[  9 ] = lfnstTemp[ 84 ];
              coeffTemp[ 10 ] = lfnstTemp[ 88 ];  coeffTemp[ 11 ] = lfnstTemp[ 92 ];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#endif
      }
      else
      {
#if JVET_W0119_LFNST_EXTENSION
        if( sbSize == 16 )
        {
          for( y = 0; y < 12; y++ )
          {
            uint32_t uiStride = ( y < 4 ) ? 12 : ( ( y < 8 ) ? 8 : 4 );
            ::memcpy( coeffTemp, lfnstTemp, uiStride * sizeof( TCoeff ) );
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
        else
        {
#endif
        for (y = 0; y < sbSize; y++)
        {
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
          uint32_t uiStride = tu.cu->cs->sps->getUseLFNSTExt() ? sbSize : ( ( y < 4 ) ? sbSize : 4 );
#else
          uint32_t uiStride = sbSize;
#endif
#else
          uint32_t uiStride = (y < 4) ? sbSize : 4;
#endif
          ::memcpy(coeffTemp, lfnstTemp, uiStride * sizeof(TCoeff));
          lfnstTemp += uiStride;
          coeffTemp += width;
        }
#if JVET_W0119_LFNST_EXTENSION
        }
#endif
      }
    }
  }
}

void TrQuant::xFwdLfnst( const TransformUnit &tu, const ComponentID compID, const bool loadTr )
{
#if JVET_AL0181_ASBT
  const CompArea& area     = tu.blocksResidual[ compID ];
#else
  const CompArea& area     = tu.blocks[ compID ];
#endif
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;
#if JVET_AI0050_INTER_MTSS
  const uint32_t  lfnstIntra = tu.cu->lfnstIntra;
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (tu.cu->isSepTree() ? true : isLuma(compID)) )
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (tu.cu->separateTree ? true : isLuma(compID)))
#else
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (CS::isDualITree(*tu.cs) ? true : isLuma(compID)))
#endif
#endif
  {
#if JVET_AC0130_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
    bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                  ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
    bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
    bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, CU::isIntra( *( tu.cu ) ) );
#endif
    if( allowNSPT )
    {
      return;
    }
#endif
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    const bool whge4         = tu.cu->cs->sps->getUseLFNSTExt() ? PU::getUseLFNST16( width, height ) : false;  // width >= 16 && height >= 16;
#else
    const bool whge4         = PU::getUseLFNST16( width, height );  // width >= 16 && height >= 16;
#endif
    const bool whge3         = PU::getUseLFNST8( width, height );
    int widthIdx             = gp_sizeIdxInfo->idxFrom( width );
    int heightIdx            = gp_sizeIdxInfo->idxFrom( height );
    const ScanElement * scan = whge4 ? g_coefTopLeftDiagScan16x16[ widthIdx ] : ( whge3 ? g_coefTopLeftDiagScan8x8[ widthIdx ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ widthIdx ][ heightIdx ] );
#else
    const bool whge3 = width >= 8 && height >= 8;
    const ScanElement * scan = whge3 ? g_coefTopLeftDiagScan8x8[ gp_sizeIdxInfo->idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
#endif
#if JVET_AK0217_INTRA_MTSS 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
      uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID).first;
#else
      uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
      uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID ).first;
#else
      uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID );
#endif
#endif
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    if( lfnstIdx < ( tu.cu->cs->sps->getUseLFNSTExt() ? 4 : 3 ) )
#else
    if ( lfnstIdx < 4 )
#endif
#else
    if( lfnstIdx < 3 )
#endif
    {

      bool            transposeFlag   = getTransposeFlag( intraMode );
#if JVET_W0119_LFNST_EXTENSION
      const int       sbSize          = whge4 ? 16 : ( whge3 ? 8 : 4 );
#else
      const int       sbSize          = whge3 ? 8 : 4;
#endif
#if !EXTENDED_LFNST && !JVET_W0119_LFNST_EXTENSION
      bool            tu4x4Flag       = ( width == 4 && height == 4 );
      bool            tu8x8Flag       = ( width == 8 && height == 8 );
#endif
      TCoeff*         lfnstTemp;
      TCoeff*         coeffTemp;
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
      const int       kerCandNum = ( tu.cu->cs->sps->getUseLFNSTExt() || ( tu.cu->cs->sps->getUseNSPT() && CU::isNSPTAllowed( width, height ) ) ) ? 3 : 2;
      TCoeff *        tempCoeff = loadTr ? m_mtsCoeffs[ lfnstIdx ? lfnstIdx + NUM_TRAFO_MODES_MTS + kerCandNum * lfnstIntra - 1 : tu.mtsIdx[ compID ] ] : m_tempCoeff;
#else
      TCoeff *        tempCoeff = loadTr ? m_mtsCoeffs[lfnstIdx ? lfnstIdx + NUM_TRAFO_MODES_MTS + 3 * lfnstIntra - 1 : tu.mtsIdx[compID]] : m_tempCoeff;
#endif
#else
      TCoeff *        tempCoeff = loadTr ? m_mtsCoeffs[lfnstIdx ? lfnstIdx + NUM_TRAFO_MODES_MTS - 1 : tu.mtsIdx[compID]] : m_tempCoeff;
#endif
#else
      TCoeff *        tempCoeff = loadTr ? m_mtsCoeffs[tu.mtsIdx[compID]] : m_tempCoeff;
#endif
      int y;
      lfnstTemp = m_tempInMatrix;   // forward low frequency non-separable transform
      coeffTemp = tempCoeff;

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[4]  = coeffTemp[1];
            lfnstTemp[8]  = coeffTemp[2];
            lfnstTemp[12] = coeffTemp[3];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#if JVET_W0119_LFNST_EXTENSION
        else if( sbSize == 8 )
#else
        else   // ( sbSize == 8 )
#endif
        {
          for (y = 0; y < 8; y++)
          {
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
            lfnstTemp[  0 ] = coeffTemp[ 0 ];
            lfnstTemp[  8 ] = coeffTemp[ 1 ];
            lfnstTemp[ 16 ] = coeffTemp[ 2 ];
            lfnstTemp[ 24 ] = coeffTemp[ 3 ];
            lfnstTemp[ 32 ] = coeffTemp[ 4 ];
            lfnstTemp[ 40 ] = coeffTemp[ 5 ];
            lfnstTemp[ 48 ] = coeffTemp[ 6 ];
            lfnstTemp[ 56 ] = coeffTemp[ 7 ];
#else
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[8]  = coeffTemp[1];
            lfnstTemp[16] = coeffTemp[2];
            lfnstTemp[24] = coeffTemp[3];
            if (y < 4)
            {
              lfnstTemp[32] = coeffTemp[4];
              lfnstTemp[36] = coeffTemp[5];
              lfnstTemp[40] = coeffTemp[6];
              lfnstTemp[44] = coeffTemp[7];
            }
#endif
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#if JVET_W0119_LFNST_EXTENSION
        else // (sbSize == 16)
        {
          for( y = 0; y < 12; y++ )
          {
            lfnstTemp[  0 ] = coeffTemp[ 0 ]; lfnstTemp[ 12 ] = coeffTemp[ 1 ];
            lfnstTemp[ 24 ] = coeffTemp[ 2 ]; lfnstTemp[ 36 ] = coeffTemp[ 3 ];

            if( y < 8 )
            {
              lfnstTemp[ 48 ] = coeffTemp[ 4 ];  lfnstTemp[ 56 ] = coeffTemp[ 5 ];
              lfnstTemp[ 64 ] = coeffTemp[ 6 ];  lfnstTemp[ 72 ] = coeffTemp[ 7 ];
            }

            if( y < 4 )
            {
              lfnstTemp[ 80 ] = coeffTemp[  8 ];  lfnstTemp[ 84 ] = coeffTemp[  9 ];
              lfnstTemp[ 88 ] = coeffTemp[ 10 ];  lfnstTemp[ 92 ] = coeffTemp[ 11 ];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
#endif
      }
      else
      {
#if JVET_W0119_LFNST_EXTENSION
        if( sbSize == 16 )
        {
          for( y = 0; y < 16; y++ )
          {
            uint32_t uiStride = ( y < 4 ) ? 12 : ( ( y < 8 ) ? 8 : 4 );
            ::memcpy( lfnstTemp, coeffTemp, uiStride * sizeof( TCoeff ) );
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
        else
        {
#endif
        for( y = 0; y < sbSize; y++ )
        {
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
          uint32_t uiStride = tu.cu->cs->sps->getUseLFNSTExt() ? sbSize : ( ( y < 4 ) ? sbSize : 4 );
#else
          uint32_t uiStride = sbSize;
#endif
#else
          uint32_t uiStride = ( y < 4 ) ? sbSize : 4;
#endif
          ::memcpy( lfnstTemp, coeffTemp, uiStride * sizeof( TCoeff ) );
          lfnstTemp += uiStride;
          coeffTemp += width;
        }
#if JVET_W0119_LFNST_EXTENSION
        }
#endif
      }

#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
      int zeroOutSize = PU::getLFNSTMatrixDim( width, height, tu.cu->cs->sps->getUseLFNSTExt() );
#else
      int zeroOutSize = PU::getLFNSTMatrixDim( width, height );
#endif

#if AHG7_LN_TOOLOFF_CFG
      fwdLfnstNxN( m_tempInMatrix, m_tempOutMatrix, tu.cu->cs->sps->getUseLFNSTExt() ? g_lfnstLut[ intraMode ] : g_vvcLfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize, tu.cu->cs->sps->getUseLFNSTExt() );
#else
      fwdLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, zeroOutSize );
#endif
#else
#if EXTENDED_LFNST
      const int trSize = whge3 ? 64 : 16;
      fwdLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, trSize );
#else
      fwdLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );
#endif
#endif
      lfnstTemp = m_tempOutMatrix; // forward spectral rearrangement
      coeffTemp = tempCoeff;
          
      const ScanElement *scanPtr = scan;

#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
      int lfnstCoeffNum = tu.cu->cs->sps->getUseLFNSTExt() ? ( ( sbSize > 8 ) ? L16W_ZO : ( ( sbSize > 4 ) ? L8W_ZO : 16 ) )
                                                           : ( ( sbSize == 4 ) ? sbSize * sbSize : 48 );
#else
      int lfnstCoeffNum = ( sbSize > 8 ) ? L16W_ZO : ( ( sbSize > 4 ) ? L8W_ZO : 16 );
#endif
#else
#if EXTENDED_LFNST
      int lfnstCoeffNum = sbSize * sbSize;
#else
      int lfnstCoeffNum = ( sbSize == 4 ) ? sbSize * sbSize : 48;
#endif
#endif

      for (y = 0; y < lfnstCoeffNum; y++)
      {
        coeffTemp[scanPtr->idx] = *lfnstTemp++;
        scanPtr++;
      }
    }
  }
}


void TrQuant::invTransformNxN( TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQP )
{
#if JVET_AL0181_ASBT
  const CompArea &area    = tu.blocksResidual[compID];
#else
  const CompArea &area    = tu.blocks[compID];
#endif
  const uint32_t uiWidth      = area.width;
  const uint32_t uiHeight     = area.height;

  CHECK( uiWidth > tu.cs->sps->getMaxTbSize() || uiHeight > tu.cs->sps->getMaxTbSize(), "Maximal allowed transformation size exceeded!" );
  CoeffBuf tempCoeff = CoeffBuf(m_tempCoeff, area);
  xDeQuant(tu, tempCoeff, compID, cQP);

  DTRACE_COEFF_BUF(D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID);

#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  if( ( spsIntraLfnstEnabled && CU::isIntra( *tu.cu ) ) || ( tu.cs->sps->getUseInterLFNST() && CU::isInter( *tu.cu ) ) )
#else
  if (tu.cs->sps->getUseLFNST())
#endif
  {
    xInvLfnst(tu, compID);
  }

  if (tu.mtsIdx[compID] == MTS_SKIP)
  {
    xITransformSkip(tempCoeff, pResi, tu, compID);
  }
  else
  {
    xIT(tu, compID, tempCoeff, pResi);
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
}

std::pair<int64_t,int64_t> TrQuant::fwdTransformICT( const TransformUnit &tu, const PelBuf &resCb, const PelBuf &resCr, PelBuf &resC1, PelBuf &resC2, int jointCbCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  CHECK( Size(resCb) != Size(resC1), "resCb and resC1 have different sizes" );
  CHECK( Size(resCb) != Size(resC2), "resCb and resC2 have different sizes" );
  return (*m_fwdICT[ TU::getICTMode(tu, jointCbCr) ])( resCb, resCr, resC1, resC2 );
}

void TrQuant::invTransformICT( const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  (*m_invICT[ TU::getICTMode(tu) ])( resCb, resCr );
}

std::vector<int> TrQuant::selectICTCandidates( const TransformUnit &tu, CompStorage* resCb, CompStorage* resCr )
{
  CHECK( !resCb[0].valid() || !resCr[0].valid(), "standard components are not valid" );

  if( !CU::isIntra( *tu.cu ) )
  {
    int cbfMask = 3;
#if JVET_AL0181_ASBT
    resCb[cbfMask].create( tu.blocksResidual[COMPONENT_Cb] );
    resCr[cbfMask].create( tu.blocksResidual[COMPONENT_Cr] );
#else
    resCb[cbfMask].create( tu.blocks[COMPONENT_Cb] );
    resCr[cbfMask].create( tu.blocks[COMPONENT_Cr] );
#endif
    fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
    std::vector<int> cbfMasksToTest;
    cbfMasksToTest.push_back( cbfMask );
    return cbfMasksToTest;
  }

  std::pair<int64_t,int64_t> pairDist[4];
  for( int cbfMask = 0; cbfMask < 4; cbfMask++ )
  {
    if( cbfMask )
    {
      CHECK( resCb[cbfMask].valid() || resCr[cbfMask].valid(), "target components for cbfMask=" << cbfMask << " are already present" );
#if JVET_AL0181_ASBT
      resCb[cbfMask].create( tu.blocksResidual[COMPONENT_Cb] );
      resCr[cbfMask].create( tu.blocksResidual[COMPONENT_Cr] );
#else
      resCb[cbfMask].create( tu.blocks[COMPONENT_Cb] );
      resCr[cbfMask].create( tu.blocks[COMPONENT_Cr] );
#endif
    }
    pairDist[cbfMask] = fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
  }

  std::vector<int> cbfMasksToTest;
  int64_t minDist1  = std::min<int64_t>( pairDist[0].first, pairDist[0].second );
  int64_t minDist2  = std::numeric_limits<int64_t>::max();
  int     cbfMask1  = 0;
  int     cbfMask2  = 0;
  for( int cbfMask : { 1, 2, 3 } )
  {
    if( pairDist[cbfMask].first < minDist1 )
    {
      cbfMask2  = cbfMask1; minDist2  = minDist1;
      cbfMask1  = cbfMask;  minDist1  = pairDist[cbfMask1].first;
    }
    else if( pairDist[cbfMask].first < minDist2 )
    {
      cbfMask2  = cbfMask;  minDist2  = pairDist[cbfMask2].first;
    }
  }
  if( cbfMask1 )
  {
    cbfMasksToTest.push_back( cbfMask1 );
  }
  if( cbfMask2 && ( ( minDist2 < (9*minDist1)/8 ) || ( !cbfMask1 && minDist2 < (3*minDist1)/2 ) ) )
  {
    cbfMasksToTest.push_back( cbfMask2 );
  }

  return cbfMasksToTest;
}



// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::getTrTypes(const TransformUnit tu, const ComponentID compID, int &trTypeHor, int &trTypeVer)
{
  const bool isExplicitMTS = (CU::isIntra(*tu.cu) ? tu.cs->sps->getUseIntraMTS() : tu.cs->sps->getUseInterMTS() && CU::isInter(*tu.cu)) && isLuma(compID);
#if JVET_AJ0257_IMPLICIT_MTS_LUT
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->getUseImplicitMTS() && isLuma(compID) && !tu.cu->lfnstIdx;
#else
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->getUseImplicitMTS() && isLuma(compID) && !tu.cu->lfnstIdx && !tu.cu->mipFlag && !tu.cu->eipFlag && !tu.cu->tmpFlag && !tu.cu->sgpm;
#endif
#else
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->getUseImplicitMTS() && isLuma(compID) && tu.cu->lfnstIdx == 0 && tu.cu->mipFlag == 0;
#endif
  const bool isISP = CU::isIntra(*tu.cu) && tu.cu->ispMode && isLuma(compID);
  const bool isSBT = CU::isInter(*tu.cu) && tu.cu->sbtInfo && isLuma(compID);

  trTypeHor = DCT2;
  trTypeVer = DCT2;
#if JVET_AI0050_SBT_LFNST
  if (isSBT && tu.cu->lfnstIdx)
  {
    return;
  }
#endif

  if (isISP && tu.cu->lfnstIdx)
  {
    return;
  }

  if (!tu.cs->sps->getUseMTS())
  {
    return;
  }

#if JVET_V0130_INTRA_TMP
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  if (isImplicitMTS || isISP)
#else
  if (isImplicitMTS || isISP || tu.cu->tmpFlag)
#endif
#else
  if (isImplicitMTS || isISP)
#endif
  {
#if JVET_AL0181_ASBT
    int  width = tu.blocksResidual[compID].width;
    int  height = tu.blocksResidual[compID].height;
#else
    int  width = tu.blocks[compID].width;
    int  height = tu.blocks[compID].height;
#endif    
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
    if (isISP || width < 4 || height < 4)
    {
      bool widthDstOk  = width >= 4 && width <= 16;
      bool heightDstOk = height >= 4 && height <= 16;

      if (widthDstOk)
      {
        trTypeHor = DST7;
      }
      if (heightDstOk)
      {
        trTypeVer = DST7;
      }
      return;
    }

    int             intraBlMode = 0;
    const CompArea &area        = tu.blocks[compID];
    if (tu.cu->dimd)
    {
      intraBlMode = 0;
    }
    else if (tu.cu->timd)
    {
      intraBlMode = 1;
    }
    else if (tu.cu->mipFlag)
    {
      intraBlMode = 2;
    }
    else if (tu.cu->tmpFlag)
    {
      intraBlMode = 3;
    }
    else if (tu.cu->sgpm)
    {
      intraBlMode = 4;
    }
    else if (tu.cu->eipFlag)
    {
      intraBlMode = 5;
    }
    else if (PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID)) == PNN_IDX)
    {
      intraBlMode = 6;
    }
    else
    {
      int predMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID));
#if JVET_AD0085_TMRL_EXTENSION
      if (tu.cu->tmrlFlag)
      {
        predMode = MAP131TO67(predMode);
      }
#endif

      predMode = PU::getWideAngle(tu, (uint32_t) predMode, compID);
      CHECK(predMode < -(NUM_EXT_LUMA_MODE >> 1) || predMode >= NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1),
            "luma mode out of range");

#if JVET_AC0105_DIRECTIONAL_PLANAR
      if (predMode == PLANAR_IDX)
      {
        if (tu.cu->plIdx == 2)
        {
          predMode = HOR_IDX;
        }
        else if (tu.cu->plIdx == 1)
        {
          predMode = VER_IDX;
        }
      }
#endif
      int     modeImplicit   = predMode < 0                ? predMode + NUM_LUMA_MODE
                               : predMode >= NUM_LUMA_MODE ? predMode - NUM_LUMA_MODE + 2
                                                           : predMode;
      int     modeIdx        = modeImplicit > DIA_IDX ? (NUM_LUMA_MODE + 1 - modeImplicit) : modeImplicit;
      bool    isTrTransposed = modeImplicit > DIA_IDX ? true : false;
      uint8_t nSzIdxW        = std::min(3, (floorLog2(width) - 2));
      uint8_t nSzIdxH        = std::min(3, (floorLog2(height) - 2));
      uint8_t nSzIdx         = isTrTransposed ? (nSzIdxH * 4 + nSzIdxW) : (nSzIdxW * 4 + nSzIdxH);
      int     nTrType        = g_aucImplicitToTrSet[nSzIdx][modeIdx];

      trTypeHor = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 1 : 0];
      trTypeVer = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 0 : 1];

      return;
    }

    int  predMode        = tu.intraDirStat.first;
    int  predModeSecond = tu.intraDirStat.second;
    int  idxMap[18]      = { 0, 1, 2, 3, -1, 4, 5, 6, -1, -1, 7, 8, -1, -1, -1, 9, 10, 11 };
    bool blockSym        = (height > width);
    bool predModeSym     = blockSym && predMode > 1;
    int  log2BlockWidth  = floorLog2(blockSym ? height : width) - 2;
    int  log2BlockHeight = floorLog2(blockSym ? width : height) - 2;
    int  blIndSize;
    if (log2BlockWidth < 4 && log2BlockHeight < 4)
    {
      blIndSize = log2BlockHeight * 4 + log2BlockWidth;
    }
    else
    {
      blIndSize = 16 + (log2BlockWidth - 4);
    }
    blIndSize = idxMap[blIndSize];
    int predModeDiff, temp = abs(predMode - predModeSecond);
    if (temp <= 8)
    {
      predModeDiff = 0;
    }
    else if (temp <= 16)
    {
      predModeDiff = 1;
    }
    else
    {
      predModeDiff = 2;
    }
    int TrIdx = 0;
    switch (intraBlMode)
    {
    case 0: TrIdx = g_aucIpmToTrSetModDimd[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 1: TrIdx = g_aucIpmToTrSetModTimd[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 2: TrIdx = g_aucIpmToTrSetModMip[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 3: TrIdx = g_aucIpmToTrSetModTmp[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 4: TrIdx = g_aucIpmToTrSetModSgpm[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 5: TrIdx = g_aucIpmToTrSetModEip[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    case 6: TrIdx = g_aucIpmToTrSetModPnn[predModeDiff][blIndSize][predModeSym ? 34 * 2 - predMode : predMode]; break;
    default:
      CHECK(true,"Invalid ImplicitMTS Transform set!");
      break;
    }
    trTypeVer = (blockSym || predModeSym) ? TrIdx / 6 : TrIdx % 6;
    trTypeHor = (blockSym || predModeSym) ? TrIdx % 6 : TrIdx / 6;
    return;
#else
#if JVET_AJ0257_IMPLICIT_MTS_LUT
    const CompArea &area = tu.blocks[compID];
    int predMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID));
#if JVET_AJ0249_NEURAL_NETWORK_BASED
    if (predMode == PNN_IDX)
    {
      return;
    }
#endif

    if (isISP || width < 4 || height < 4 || tu.cu->dimd || tu.cu->timd)
    {
      bool widthDstOk  = width >= 4 && width <= 16;
      bool heightDstOk = height >= 4 && height <= 16;

      if (widthDstOk)
      {
        trTypeHor = DST7;
      }
      if (heightDstOk)
      {
        trTypeVer = DST7;
      }
      return;
    }

#if JVET_AD0085_TMRL_EXTENSION
    if (tu.cu->tmrlFlag)
    {
      predMode = MAP131TO67(predMode);
    }
#endif

    predMode = PU::getWideAngle(tu, (uint32_t) predMode, compID);
    CHECK(predMode < -(NUM_EXT_LUMA_MODE >> 1) || predMode >= NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1),
          "luma mode out of range");

#if JVET_AC0105_DIRECTIONAL_PLANAR
    if (predMode == PLANAR_IDX)
    {
      if (tu.cu->plIdx == 2)
      {
        predMode = HOR_IDX;
      }
      else if (tu.cu->plIdx == 1)
      {
        predMode = VER_IDX;
      }
    }
#endif
    int     modeImplicit   = predMode < 0                ? predMode + NUM_LUMA_MODE
                             : predMode >= NUM_LUMA_MODE ? predMode - NUM_LUMA_MODE + 2
                                                         : predMode;
    int     modeIdx        = modeImplicit > DIA_IDX ? (NUM_LUMA_MODE + 1 - modeImplicit) : modeImplicit;
    bool    isTrTransposed = modeImplicit > DIA_IDX ? true : false;
    uint8_t nSzIdxW        = std::min(3, (floorLog2(width) - 2));
    uint8_t nSzIdxH        = std::min(3, (floorLog2(height) - 2));
    uint8_t nSzIdx         = isTrTransposed ? (nSzIdxH * 4 + nSzIdxW) : (nSzIdxW * 4 + nSzIdxH);
    int     nTrType        = g_aucImplicitToTrSet[nSzIdx][modeIdx];

    trTypeHor = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 1 : 0];
    trTypeVer = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 0 : 1];

    return;
#else
    bool widthDstOk  = width >= 4 && width <= 16;
    bool heightDstOk = height >= 4 && height <= 16;

    if (widthDstOk)
    {
      trTypeHor = DST7;
    }
    if (heightDstOk)
    {
      trTypeVer = DST7;
    }
    return;
#endif
#endif
  }


  if (isSBT)
  {
    uint8_t sbtIdx = tu.cu->getSbtIdx();
    uint8_t sbtPos = tu.cu->getSbtPos();

#if JVET_AJ0260_SBT_CORNER_MODE
    if( sbtIdx == SBT_QUAD || sbtIdx == SBT_QUARTER )
    {
      if( tu.lwidth() > MTS_INTER_MAX_CU_SIZE || tu.lheight() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else if( sbtPos == 0 )
      {
        trTypeHor = DCT8;
        trTypeVer = DCT8;
      }
      else if( sbtPos == 1 )
      {
        trTypeHor = DST7;
        trTypeVer = DCT8;
      }
      else if( sbtPos == 2 )
      {
        trTypeHor = DCT8;
        trTypeVer = DST7;
      }
      else if( sbtPos == 3 )
      {
        trTypeHor = DST7;
        trTypeVer = DST7;
      }
      else
      {
        CHECK( true, "Wrong SBT QUAD position" );
      }
    }   
    else
#endif
    if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD )
    {
      CHECK( tu.lwidth() > MTS_INTER_MAX_CU_SIZE, "Unsupported width");

      if( tu.lheight() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if (sbtPos == SBT_POS0)
        {
          trTypeHor = DCT8;
          trTypeVer = DST7;
        }
        else
        {
          trTypeHor = DST7;
          trTypeVer = DST7;
        }
      }
    }
    else
    {
      CHECK( tu.lheight() > MTS_INTER_MAX_CU_SIZE, "Unsupported height");

      if( tu.lwidth() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if (sbtPos == SBT_POS0)
        {
          trTypeHor = DST7;
          trTypeVer = DCT8;
        }
        else
        {
          trTypeHor = DST7;
          trTypeVer = DST7;
        }
      }
    }
    return;
  }

  if (isExplicitMTS)
  {
#if JVET_W0103_INTRA_MTS
    if (tu.mtsIdx[compID] > MTS_SKIP && CU::isIntra(*tu.cu)
#if AHG7_MTS_TOOLOFF_CFG
      && tu.cs->sps->getUseMTSExt()
#endif
      )
    {
      CHECK(compID != COMPONENT_Y, " MTS activated for chroma");
#if JVET_AL0181_ASBT
      uint32_t width = tu.blocksResidual[compID].width;
      uint32_t height = tu.blocksResidual[compID].height;
#else
      uint32_t width = tu.blocks[compID].width;
      uint32_t height = tu.blocks[compID].height;
#endif
      int TrIdx = (tu.mtsIdx[compID] - MTS_DST7_DST7);
      CHECK(width < 4 || height < 4, "width < 4 || height < 4 for MTS");
      uint8_t nSzIdxW = std::min(3, (floorLog2(width) - 2));
      uint8_t nSzIdxH = std::min(3, (floorLog2(height) - 2));
#if JVET_AL0181_ASBT
      const CompArea& area = tu.blocksResidual[compID];
#else
      const CompArea& area = tu.blocks[compID];
#endif
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
      int predMode;
      if (tu.cu->tmpFlag)
      {
        predMode = tu.cu->intraTmpDimdMode;
      }
      else
      {
        predMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID));
      }
#else
      int predMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID));
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
      if (predMode == PNN_IDX)
      {
        const ComponentID compIdEffective = !isLuma(compID) && tu.jointCbCr ? COMPONENT_Cb : compID;
        predMode = (tu.cu)->indicesRepresentationPnn[compIdEffective][0];
      }
#endif
#if JVET_W0123_TIMD_FUSION
      if (tu.cu->timd && compID == COMPONENT_Y)
      {
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if (PU::isBvTimdApplied(tu.cu))
        {
          int secondMode = 0;
          PU::getTimdVimp(predMode, secondMode, tu.cu);
        }
        else
        {
          predMode = MAP131TO67(predMode);
        }
#else
        predMode = MAP131TO67(predMode);
#endif
      }
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
#if JVET_AL0108_BVG_DIMD
      if (tu.cu->dimd && !tu.cu->bvgDimdFlag && compID == COMPONENT_Y
#if JVET_AM0074_INTRA_MERGE
        && !tu.cu->intraMergeMode
#endif
        )
#else
      if (tu.cu->dimd && compID == COMPONENT_Y)
#endif
      {
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if (PU::isBvDimdApplied(tu.cu))
        {
          int secondMode = 0;
          PU::getDimdVimp(predMode, secondMode, tu.cu);
        }
        else
        {
          // predMode = MAP131TO67(predMode);
        }
#endif
      }
#endif
#if JVET_AB0155_SGPM
      if (tu.cu->sgpm)
      {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        CHECK(tu.cu->sgpmSplitDir >= SGPM_TOTAL_NUM_PARTITIONS, "Invalid splitDir for SGPM");
#if JVET_AJ0112_REGRESSION_SGPM
        predMode = PU::isRegressionSgpm(*tu.cs->getPU(area.pos(), toChannelType(compID))) ? tu.cu->sgpmDimdMode : g_geoAngle2IntraAng[g_geoParams[g_sgpmSplitDir[tu.cu->sgpmSplitDir]][0]];
#else
        predMode = g_geoAngle2IntraAng[g_geoParams[g_sgpmSplitDir[tu.cu->sgpmSplitDir]][0]];
#endif
#else
#if JVET_AJ0112_REGRESSION_SGPM
        predMode = PU::isRegressionSgpm(*tu.cs->getPU(area.pos(), toChannelType(compID))) ? tu.cu->sgpmDimdMode : g_geoAngle2IntraAng[g_geoParams[tu.cu->sgpmSplitDir][0]];
#else
        predMode = g_geoAngle2IntraAng[g_geoParams[tu.cu->sgpmSplitDir][0]];
#endif
#endif
      }
#endif

#if JVET_AD0085_TMRL_EXTENSION
      if (tu.cu->tmrlFlag && compID == COMPONENT_Y)
      {
        predMode = MAP131TO67(predMode);
      }
#endif
#if JVET_AG0058_EIP
      if (tu.cu->eipFlag && compID == COMPONENT_Y)
      {
        predMode = tu.cu->eipModel.eipDimdMode;
      }
#endif
      int ucMode;
      int nMdIdx;
      bool isTrTransposed = false;
      if (tu.cu->mipFlag) //MIP is treated as planar.
      {
        ucMode = 0;
        nMdIdx = 35;
        isTrTransposed = (tu.cs->getPU(area.pos(), toChannelType(compID)))->mipTransposedFlag;
      }
      else
      {
        ucMode = predMode; //"ucMode" is the signaled Mode.
        predMode = PU::getWideAngle(tu, (uint32_t)predMode, compID);
        CHECK(predMode < -(NUM_EXT_LUMA_MODE >> 1) || predMode >= NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1), "luma mode out of range");
        predMode = (predMode < 0) ? 2 : (predMode >= NUM_LUMA_MODE) ? 66 : predMode;
#if JVET_AC0105_DIRECTIONAL_PLANAR
        if (compID == COMPONENT_Y && predMode == PLANAR_IDX)
        {
          if (tu.cu->plIdx == 2)
          {
            predMode = HOR_IDX;
          }
          else if (tu.cu->plIdx == 1)
          {
            predMode = VER_IDX;
          }
        }
#endif
        nMdIdx = predMode > DIA_IDX ? (NUM_LUMA_MODE + 1 - predMode) : predMode;
        isTrTransposed = (predMode > DIA_IDX) ? true : false;
      }
      uint8_t nSzIdx = isTrTransposed ? (nSzIdxH * 4 + nSzIdxW) : (nSzIdxW * 4 + nSzIdxH);
      CHECK(nSzIdx >= 16, "nSzIdx >= 16");
      CHECK(nMdIdx >= 36, "nMdIdx >= 36");
      uint8_t nTrSet = g_aucIpmToTrSet[nSzIdx][nMdIdx];
      CHECK(nTrSet >= 80, "nTrSet >= 80");
      trTypeVer = g_aucTrIdxToTr[g_aucTrSet[nTrSet][TrIdx]][predMode > DIA_IDX ? 1 : 0];
      trTypeHor = g_aucTrIdxToTr[g_aucTrSet[nTrSet][TrIdx]][predMode > DIA_IDX ? 0 : 1];
      predMode = ucMode; //to Check IDTR criteria, signaled mode should be used to check the difference
      if (TrIdx == 3 && width <= 16 && height <= 16)
      {
        if (abs(predMode - HOR_IDX) <= g_aiIdLut[floorLog2(width) - 2][floorLog2(height) - 2])
        {
          trTypeVer = IDTR;
        }
        if (abs(predMode - VER_IDX) <= g_aiIdLut[floorLog2(width) - 2][floorLog2(height) - 2])
        {
          trTypeHor = IDTR;
        }
      }
    }
    else
#endif
    if (tu.mtsIdx[compID] > MTS_SKIP)
    {
      int indHor = (tu.mtsIdx[compID] - MTS_DST7_DST7) & 1;
      int indVer = (tu.mtsIdx[compID] - MTS_DST7_DST7) >> 1;
      trTypeHor = indHor ? DCT8 : DST7;
      trTypeVer = indVer ? DCT8 : DST7;
#if JVET_AA0133_INTER_MTS_OPT
#if AHG7_MTS_TOOLOFF_CFG
      if (tu.cs->sps->getUseMTSExt())
      {
#endif
#if JVET_AL0181_ASBT
        uint32_t width = tu.blocksResidual[compID].width;
        uint32_t height = tu.blocksResidual[compID].height;
#else
        uint32_t width = tu.blocks[compID].width;
        uint32_t height = tu.blocks[compID].height;
#endif
        CHECK(width < 4 || height < 4, "width < 4 || height < 4 for KLT");
        if (width <= 16 && height <= 16)
        {
          trTypeHor = indHor ? KLT1 : KLT0;
          trTypeVer = indVer ? KLT1 : KLT0;
        }
#if AHG7_MTS_TOOLOFF_CFG
      }
#endif
#endif
    }
  }
}

void TrQuant::xT( const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int width, const int height )
{
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const uint32_t transformWidthIndex    = floorLog2(width ) - 1;  // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = floorLog2(height) - 1;  // nLog2HeightMinus1, since transform start from 2-point

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

#if JVET_AJ0061_TIMD_MERGE
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  if (tu.cu->timdMrg && !tu.cu->lfnstIdx && !tu.cs->sps->getUseImplicitMTS())
#else
  if (tu.cu->timdMrg && !tu.cu->lfnstIdx)
#endif
  {
    // Timd-Mrg CUs inherit transform type from their cands
    int implicitDst7 = PU::canTimdMergeImplicitDst7(tu);
    trTypeHor = (implicitDst7 & 2) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][0];
    trTypeVer = (implicitDst7 & 1) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][1];
#if AHG7_MTS_TOOLOFF_CFG
    trTypeHor = (width > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeHor;
    trTypeVer = (height > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeVer;
#endif
  }
#endif
#if JVET_AM0074_INTRA_MERGE
#if JVET_AJ0061_TIMD_MERGE
  else
#endif
  if (tu.cu->intraMergeMode && !tu.cu->lfnstIdx)
  {
    int implicitDst7 = PU::canIntraMergeImplicitDst7(tu);
    trTypeHor = (implicitDst7 & 2) ? DST7 : DCT2;
    trTypeVer = (implicitDst7 & 1) ? DST7 : DCT2;
#if AHG7_MTS_TOOLOFF_CFG
    trTypeHor = (width > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeHor;
    trTypeVer = (height > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeVer;
#endif
  }
#endif
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
  else
  {
#endif
  getTrTypes ( tu, compID, trTypeHor, trTypeVer );
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
  }
#endif
#if TU_256
  int  skipWidth  =  width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int  skipHeight =  height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#else
  int  skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int  skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if JVET_AC0130_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
  const bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, CU::isIntra( *(tu.cu) ) );
#endif
#endif

#if EXTENDED_LFNST
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx && width >= 4 && height >= 4)
  {
    const bool whge3 = width >= 8 && height >= 8;
    const int lfnst_threshold = whge3 ? 8 : 4;
    skipWidth  = width  - lfnst_threshold;
    skipHeight = height - lfnst_threshold;
  }
#else
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
#if JVET_AC0130_NSPT
  if( ( ( spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) ) || ( tu.cs->sps->getUseInterLFNST() && CU::isInter( *( tu.cu ) ) ) ) && tu.cu->lfnstIdx && !allowNSPT )
#else
  if( ( ( spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) ) || ( tu.cs->sps->getUseInterLFNST() && CU::isInter( *( tu.cu ) ) ) ) && tu.cu->lfnstIdx )
#endif
#else
#if JVET_AC0130_NSPT
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx && !allowNSPT )
#else
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx )
#endif
#endif
  {
    if( ( width == 4 && height > 16 ) || ( width > 16 && height == 4 ) )
    {
      skipWidth  = width  - 4;
      skipHeight = height - 4;
    }
#if JVET_W0119_LFNST_EXTENSION
    else if( width >= 16 && height >= 16 )
    {
      skipWidth  = width  - 16;
      skipHeight = height - 16;
    }
#endif
    else if( ( width == 8 && height > 16 ) || ( width > 16 && height == 8 ) )
    {
      skipWidth  = width  - 8;
      skipHeight = height - 8;
    }
  }
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  if ( trTypeHor != DCT2 )
  {
    CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType{ STATS__TOOL_EMT, uint32_t( width ), uint32_t( height ), compID } );
  }
#endif

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TB_SIZEY * MAX_TB_SIZEY] );

  const Pel *resiBuf    = resi.buf;
  const int  resiStride = resi.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      block[( y * width ) + x] = resiBuf[( y * resiStride ) + x];
    }
  }

  dstCoeff.fill( 0 );

  if( width > 1 && height > 1 ) // 2-D transform
  {
    const int      shift_1st              = ((floorLog2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    const int      shift_2nd              =  (floorLog2(height))            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp = (TCoeff *) alloca(width * height * sizeof(TCoeff));

#if JVET_AC0130_NSPT
    if( CU::nsptApplyCond( tu, compID, allowNSPT ) )
    {
      xFwdNspt( tu, block, dstCoeff.buf, compID, shift_1st, shift_2nd, tu.cu->lfnstIdx );
    }
    else
    {
#endif
    fastFwdTrans[trTypeHor][transformWidthIndex](block, tmp, shift_1st, height, 0, skipWidth);
    fastFwdTrans[trTypeVer][transformHeightIndex](tmp, dstCoeff.buf, shift_2nd, width, skipWidth, skipHeight);
#if JVET_AC0130_NSPT
    }
#endif
  }
  else if( height == 1 ) //1-D horizontal transform
  {
    const int      shift              = ((floorLog2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastFwdTrans[trTypeHor][transformWidthIndex]( block, dstCoeff.buf, shift, 1, 0, skipWidth );
  }
  else //if (iWidth == 1) //1-D vertical transform
  {
    int shift = ( ( floorLog2(height) ) + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastFwdTrans[trTypeVer][transformHeightIndex]( block, dstCoeff.buf, shift, 1, 0, skipHeight );
  }
}

void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const uint32_t transformWidthIndex    = floorLog2(width ) - 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = floorLog2(height) - 1;                                // nLog2HeightMinus1, since transform start from 2-point

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

#if JVET_AJ0061_TIMD_MERGE
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  if (tu.cu->timdMrg && !tu.cu->lfnstIdx && !tu.cs->sps->getUseImplicitMTS())
#else
  if (tu.cu->timdMrg && !tu.cu->lfnstIdx)
#endif
  {
    // Timd-Mrg CUs inherit transform type from their cands
    int implicitDst7 = PU::canTimdMergeImplicitDst7(tu);
    trTypeHor = (implicitDst7 & 2) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][0];
    trTypeVer = (implicitDst7 & 1) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][1];
#if AHG7_MTS_TOOLOFF_CFG
    trTypeHor = (width > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeHor;
    trTypeVer = (height > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeVer;
#endif
  }
#endif
#if JVET_AM0074_INTRA_MERGE
#if JVET_AJ0061_TIMD_MERGE
  else
#endif
  if (tu.cu->intraMergeMode && !tu.cu->lfnstIdx)
  {
    int implicitDst7 = PU::canIntraMergeImplicitDst7(tu);
    trTypeHor = (implicitDst7 & 2) ? DST7 : DCT2;
    trTypeVer = (implicitDst7 & 1) ? DST7 : DCT2;
#if AHG7_MTS_TOOLOFF_CFG
    trTypeHor = (width > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeHor;
    trTypeVer = (height > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trTypeVer;
#endif
  }
#endif
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
  else
  {
#endif
  getTrTypes ( tu, compID, trTypeHor, trTypeVer );
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
  }
#endif
#if TU_256
  int skipWidth  =  width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int skipHeight =  height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#else
  int skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if JVET_AC0130_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
  const bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, CU::isIntra( *(tu.cu) ) );
#endif
#endif

#if EXTENDED_LFNST
  if (tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx && width >= 4 && height >= 4)
  {
    const bool whge3 = width >= 8 && height >= 8;
    const int lfnst_threshold = whge3 ? 8 : 4;
    skipWidth = width - lfnst_threshold;
    skipHeight = height - lfnst_threshold;
  }
#else
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
#if JVET_AC0130_NSPT
  if( ( ( spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) ) || ( tu.cs->sps->getUseInterLFNST() && CU::isInter( *( tu.cu ) ) ) ) && tu.cu->lfnstIdx && !allowNSPT )
#else
  if( ( ( spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) ) || ( tu.cs->sps->getUseInterLFNST() && CU::isInter( *( tu.cu ) ) ) ) && tu.cu->lfnstIdx )
#endif
#else
#if JVET_AC0130_NSPT
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx && !allowNSPT )
#else
  if( tu.cs->sps->getUseLFNST() && tu.cu->lfnstIdx )
#endif
#endif
  {
    if( ( width == 4 && height > 16 ) || ( width > 16 && height == 4 ) )
    {
      skipWidth  = width  - 4;
      skipHeight = height - 4;
    }
#if JVET_W0119_LFNST_EXTENSION
    else if( ( width >= 16 && height >= 16 ) )
    {
      skipWidth  = width  - 16;
      skipHeight = height - 16;
    }
#endif
    else if( ( width == 8 && height > 16 ) || ( width > 16 && height == 8 ) )
    {
      skipWidth  = width  - 8;
      skipHeight = height - 8;
    }
  }
#endif

  TCoeff *block = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

  if( width > 1 && height > 1 ) //2-D transform
  {
    const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

#if JVET_AC0130_NSPT
    if( CU::nsptApplyCond( tu, compID, allowNSPT ) )
    {
      xInvNspt( tu, pCoeff.buf, block, compID, shift_1st, shift_2nd, tu.cu->lfnstIdx );
    }
    else
    {
#endif
  fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, clipMinimum, clipMaximum);
  fastInvTrans[trTypeHor][transformWidthIndex] (tmp,      block, shift_2nd, height,         0, skipWidth, clipMinimum, clipMaximum);
#if JVET_AC0130_NSPT
    }
#endif
  }
  else if( width == 1 ) //1-D vertical transform
  {
    int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastInvTrans[trTypeVer][transformHeightIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipHeight, clipMinimum, clipMaximum );
  }
  else //if(iHeight == 1) //1-D horizontal transform
  {
    const int      shift              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastInvTrans[trTypeHor][transformWidthIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipWidth, clipMinimum, clipMaximum );
  }

  Pel *resiBuf    = pResidual.buf;
  int  resiStride = pResidual.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      resiBuf[( y * resiStride ) + x] = Pel( block[( y * width ) + x] );
    }
  }
}

#if JVET_AC0130_NSPT
void TrQuant::xFwdNspt( const TransformUnit &tu, TCoeff* src, TCoeff* dst, const ComponentID compID, const int shift_1st, const int shift_2nd, int lfnstIdx )
{
#if JVET_AL0181_ASBT
  const CompArea&   area = tu.blocksResidual[ compID ];
#else
  const CompArea&   area = tu.blocks[ compID ];
#endif
  const uint32_t   width = area.width;
  const uint32_t  height = area.height;

  const ScanElement * scan = g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
#if JVET_AK0217_INTRA_MTSS 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID ).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID );
#endif
#endif

  bool transposeFlag = getTransposeFlag( intraMode );

  TCoeff*          nsptIn = m_nsptTempInMatrix;
  TCoeff *        nsptOut = m_nsptTempOutMatrix;

  if( transposeFlag )
  {
    TCoeff* srcTemp = src;
    for( int y = 0; y < height; y++ )
    {
      TCoeff* nsptInTemp = nsptIn++;
      for( int x = 0; x < width; x++ )
      {
        *nsptInTemp = *srcTemp++;
        nsptInTemp += height;
      }
    }
  }
  else // transposeFlag = 0
  {
    ::memcpy( nsptIn, src, width * height * sizeof( TCoeff ) );
  }

  int         nsptIdx = lfnstIdx - 1;
  uint8_t  nsptSetIdx = g_nsptLut[ intraMode ];
  int     zeroOutSize = PU::getNSPTMatrixDim( transposeFlag ? height : width, transposeFlag ? width : height );

#if INTRA_TRANS_ENC_OPT
  m_computeFwdNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, zeroOutSize, nsptIdx
#else
  computeFwdNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, zeroOutSize, nsptIdx
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    , PU::getNSPTBucket(tu, secondBucket, compID)
#else
    , PU::getNSPTBucket(tu, secondBucket)
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    , PU::getNSPTBucket(tu, compID)
#else
    , PU::getNSPTBucket(tu)
#endif
#endif
#endif
  );

  int nsptCoeffNum = PU::getNSPTMatrixDim( width, height );
  const ScanElement *scanPtr = scan;

  for( int y = 0; y < nsptCoeffNum; y++ )
  {
    dst[ scanPtr->idx ] = *nsptOut++;
    scanPtr++;
  } 
}

void TrQuant::xInvNspt( const TransformUnit &tu, const TCoeff* src, TCoeff* dst, const ComponentID compID, const int shift_1st, const int shift_2nd, int lfnstIdx )
{
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
#endif
#if JVET_AL0181_ASBT
  const CompArea&   area = tu.blocksResidual[ compID ];
#else
  const CompArea&   area = tu.blocks[ compID ];
#endif
  const uint32_t   width = area.width;
  const uint32_t  height = area.height;
 
  const ScanElement * scan = g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ gp_sizeIdxInfo->idxFrom( width ) ][ gp_sizeIdxInfo->idxFrom( height ) ];
#if JVET_AK0217_INTRA_MTSS 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID ).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID );
#endif
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType { STATS__TOOL_LFNST, width, height, compID } );
#endif
  bool transposeFlag = getTransposeFlag( intraMode );
  TCoeff  *invNsptIn = m_nsptTempInMatrix;
  int   nsptCoeffNum = PU::getNSPTMatrixDim( transposeFlag ? height : width, transposeFlag ? width : height );

  const ScanElement *scanPtr = scan;
  for( int i = 0; i < nsptCoeffNum; i++ )
  {
    *invNsptIn++ = src[ scanPtr->idx ];
    scanPtr++;
  }

  int        nsptIdx = lfnstIdx - 1;
  uint8_t nsptSetIdx = g_nsptLut[ intraMode ];
  int    zeroOutSize = PU::getNSPTMatrixDim( transposeFlag ? height : width, transposeFlag ? width : height );

#if INTRA_TRANS_ENC_OPT
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  m_computeInvNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, maxLog2TrDynamicRange, zeroOutSize, nsptIdx
#else
  m_computeInvNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, zeroOutSize, nsptIdx
#endif
#else
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  computeInvNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, maxLog2TrDynamicRange, zeroOutSize, nsptIdx
#else
  computeInvNspt( m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width, transposeFlag ? width : height, shift_1st, shift_2nd, zeroOutSize, nsptIdx
#endif
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    , PU::getNSPTBucket(tu, secondBucket, compID)
#else
    , PU::getNSPTBucket(tu, secondBucket)
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    , PU::getNSPTBucket(tu, compID)
#else
    , PU::getNSPTBucket(tu)
#endif
#endif
#endif
  );

  TCoeff*       nsptOut = m_nsptTempOutMatrix;

  if( transposeFlag )
  {
    TCoeff* dstTemp = dst;
    for( int y = 0; y < height; y++ )
    {
      TCoeff* nsptOutTemp = nsptOut++;
      for( int x = 0; x < width; x++ )
      {
        *dstTemp++ = *nsptOutTemp;
        nsptOutTemp += height;
      }
    }
  }
  else
  {
    ::memcpy( dst, nsptOut, width * height * sizeof( TCoeff ) );
  }
}

#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
uint8_t TrQuant::getNsptKernelCluster(const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx, int bktIdx)
{
  CHECK(bktIdx < 0 || bktIdx >= NUM_NSPT_BLOCK_TYPES, "bktIdx outside range");
  if (width == 4 && height == 4)
  {
    return g_nsptIdx4x4[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 8 && height == 8)
  {
    return g_nsptIdx8x8[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 4 && height == 8)
  {
    return g_nsptIdx4x8[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 8 && height == 4)
  {
    return g_nsptIdx8x4[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 4 && height == 16)
  {
    return g_nsptIdx4x16[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 16 && height == 4)
  {
    return g_nsptIdx16x4[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 8 && height == 16)
  {
    return g_nsptIdx8x16[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 16 && height == 8)
  {
    return g_nsptIdx16x8[ mode ][ bktIdx ][ nsptIdx ];
  }
#if JVET_AE0086_LARGE_NSPT
  else if (width == 4 && height == 32)
  {
    return g_nsptIdx4x32[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 32 && height == 4)
  {
    return g_nsptIdx32x4[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 8 && height == 32)
  {
    return g_nsptIdx8x32[ mode ][ bktIdx ][ nsptIdx ];
  }
  else if (width == 32 && height == 8)
  {
    return g_nsptIdx32x8[ mode ][ bktIdx ][ nsptIdx ];
  }
#endif
  return false;
}

const int8_t *TrQuant::getNsptMatrix(const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx, int bktIdx)
{
  uint8_t clusterIdx = TrQuant::getNsptKernelCluster(mode, width, height, nsptIdx, bktIdx);
  const int8_t *trMat = g_nspt4x4[ clusterIdx ][ 0 ];
  if (width == 8 && height == 8)
  {
    trMat = g_nspt8x8[ clusterIdx ][ 0 ];
  }
  else if (width == 4 && height == 8)
  {
    trMat = g_nspt4x8[ clusterIdx ][ 0 ];
  }
  else if (width == 8 && height == 4)
  {
    trMat = g_nspt8x4[ clusterIdx ][ 0 ];
  }
  else if (width == 4 && height == 16)
  {
    trMat = g_nspt4x16[ clusterIdx ][ 0 ];
  }
  else if (width == 16 && height == 4)
  {
    trMat = g_nspt16x4[ clusterIdx ][ 0 ];
  }
  else if (width == 8 && height == 16)
  {
    trMat = g_nspt8x16[ clusterIdx ][ 0 ];
  }
  else if (width == 16 && height == 8)
  {
    trMat = g_nspt16x8[ clusterIdx ][ 0 ];
  }
#if JVET_AE0086_LARGE_NSPT
  else if (width == 4 && height == 32)
  {
    trMat = g_nspt4x32[ clusterIdx ][ 0 ];
  }
  else if (width == 32 && height == 4)
  {
    trMat = g_nspt32x4[ clusterIdx ][ 0 ];
  }
  else if (width == 8 && height == 32)
  {
    trMat = g_nspt8x32[ clusterIdx ][ 0 ];
  }
  else if (width == 32 && height == 8)
  {
    trMat = g_nspt32x8[ clusterIdx ][ 0 ];
  }
#endif 
  return trMat;
}
#endif

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::computeFwdNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx
#else
void TrQuant::computeFwdNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
                             , int bktIdx
#endif
)
{
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  const int8_t* trMat = getNsptMatrix(mode, width, height, nsptIdx, bktIdx);
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

  int     trSize = width * height;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  TCoeff  coef;
  TCoeff* out = dst;
#else
  int  coef;
  int* out = dst;
#endif

  for( int j = 0; j < zeroOutSize; j++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*          srcPtr = src;
#else
    int*            srcPtr = src;
#endif
    const int8_t* trMatTmp = trMat;
    coef = 0;
    for( int i = 0; i < trSize; i++ )
    {
      coef += *srcPtr++ * *trMatTmp++;
    }

    int shift = shift_1st + shift_2nd - 7;
    int rnd = 1 << ( shift - 1 );
    *out++ = ( coef + rnd ) >> shift;

    trMat += trSize;
  }

  std::fill_n( out, trSize - zeroOutSize, 0 );
}

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
void TrQuant::computeInvNspt( TCoeff* src, TCoeff* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd,
  const int maxLog2TrDynamicRange, int zeroOutSize, int nsptIdx
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
                             , int bktIdx
#endif
)
{
#else
void TrQuant::computeInvNspt( int* src, int* dst, const uint32_t mode, const uint32_t width, const uint32_t height, const int shift_1st, const int shift_2nd, int zeroOutSize, int nsptIdx )
{
  int             maxLog2TrDynamicRange = 15;
#endif
  const TCoeff    outputMinimum = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum = ( 1 << maxLog2TrDynamicRange ) - 1;

#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  const int8_t* trMat = getNsptMatrix(mode, width, height, nsptIdx, bktIdx);
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

  int trSize = width * height;

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  TCoeff  resi;
  TCoeff* out = dst;
#else
  int  resi;
  int* out = dst;
#endif

  for( int j = 0; j < trSize; j++ )
  {
    resi = 0;
    const int8_t* trMatTmp = trMat;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif

    for( int i = 0; i < zeroOutSize; i++ )
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }

    int shift = shift_1st + shift_2nd - 7;
    int rnd = 1 << ( shift - 1 );
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    *out++ = Clip3<TCoeff>( outputMinimum, outputMaximum, ( resi + rnd ) >> shift );
#else
    *out++ = Clip3( outputMinimum, outputMaximum, ( int ) ( resi + rnd ) >> shift );
#endif
    trMat++;
  }
}
#endif

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
#if JVET_AL0181_ASBT
  const CompArea &area      = tu.blocksResidual[compID];
#else
  const CompArea &area      = tu.blocks[compID];
#endif
  const int width           = area.width;
  const int height          = area.height;

  const TCoeff *coeff = pCoeff.buf;
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      pResidual.at(x, y) = coeff[x];
    }
    coeff += pCoeff.stride;
  }
}

void TrQuant::xQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  m_quant->quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
}

void TrQuant::transformNxN( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, std::vector<TrMode>* trModes, const int maxCand )
{
        CodingStructure &cs = *tu.cs;
#if JVET_AL0181_ASBT
  const CompArea &rect      = tu.blocksResidual[compID];
#else
  const CompArea &rect      = tu.blocks[compID];
#endif
  const uint32_t width      = rect.width;
  const uint32_t height     = rect.height;

  const CPelBuf  resiBuf    = cs.getResiBuf(rect);

  CHECK( cs.sps->getMaxTbSize() < width, "Unsupported transformation size" );

  int pos = 0;
  std::vector<TrCost> trCosts;
  std::vector<TrMode>::iterator it = trModes->begin();
#if TU_256
  const double facBB[] = { 1.2, 1.3, 1.3, 1.4, 1.5, 1.5, 1.5 };
#else
  const double facBB[] = { 1.2, 1.3, 1.3, 1.4, 1.5 };
#endif

  while( it != trModes->end() )
  {
#if JVET_AG0061_INTER_LFNST_NSPT
    tu.mtsIdx[compID]   = it->first < NUM_TRAFO_MODES_MTS ? it->first : 0;
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
    int kerCandNum = ( tu.cu->cs->sps->getUseLFNSTExt() || ( tu.cu->cs->sps->getUseNSPT() && CU::isNSPTAllowed( width, height ) ) ) ? 3 : 2;
    int factor = ( it->first - NUM_TRAFO_MODES_MTS ) / kerCandNum;
    tu.lfnstIdx[ compID ] = it->first < NUM_TRAFO_MODES_MTS ? 0 : it->first - NUM_TRAFO_MODES_MTS - kerCandNum * factor + 1;
    tu.lfnstIntra[ compID ] = it->first < ( NUM_TRAFO_MODES_MTS + kerCandNum ) ? 0 : ( it->first < ( NUM_TRAFO_MODES_MTS + ( kerCandNum << 1 ) ) ? 1 : 2 );
#else
    int factor = (it->first - NUM_TRAFO_MODES_MTS) / 3;
    tu.lfnstIdx[compID] = it->first < NUM_TRAFO_MODES_MTS ? 0 : it->first - NUM_TRAFO_MODES_MTS - 3 * factor + 1;
    tu.lfnstIntra[compID] = it->first < (NUM_TRAFO_MODES_MTS + 3) ? 0 : (it->first < (NUM_TRAFO_MODES_MTS + 6) ? 1 : 2);
#endif
#else
    tu.lfnstIdx[compID] = it->first < NUM_TRAFO_MODES_MTS ? 0 : it->first - NUM_TRAFO_MODES_MTS + 1;
#endif
#if JVET_AI0050_SBT_LFNST
    if ((compID == COMPONENT_Y && !tu.cu->sbtInfo) || (!tu.noResidual && compID == COMPONENT_Y && tu.cu->sbtInfo))
#else
    if (compID == COMPONENT_Y)
#endif
    {
      tu.cu->lfnstIdx = tu.lfnstIdx[compID];
#if JVET_AI0050_INTER_MTSS
      tu.cu->lfnstIntra = tu.lfnstIntra[compID];
#endif
    }
    CoeffBuf tempCoeff(m_mtsCoeffs[it->first], rect);
#else
    tu.mtsIdx[compID] = it->first;
    CoeffBuf tempCoeff( m_mtsCoeffs[tu.mtsIdx[compID]], rect);
#endif

    if( tu.noResidual )
    {
      int sumAbs = 0;
      trCosts.push_back( TrCost( sumAbs, pos++ ) );
      it++;
      continue;
    }
    if ( tu.mtsIdx[compID] == MTS_SKIP )
    {
      xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
    }
    else
    {
      xT( tu, compID, resiBuf, tempCoeff, width, height );
    }
#if JVET_AG0061_INTER_LFNST_NSPT
    xFwdLfnst(tu, compID, true);
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff sumAbs = 0;
#else
    int sumAbs = 0;
#endif
    for( int pos = 0; pos < width*height; pos++ )
    {
      sumAbs += abs( tempCoeff.buf[pos] );
    }

    double scaleSAD=1.0;
    if ( tu.mtsIdx[compID] == MTS_SKIP && ((floorLog2(width) + floorLog2(height)) & 1) == 1)
    {
      scaleSAD=1.0/1.414213562; // compensate for not scaling transform skip coefficients by 1/sqrt(2)
    }
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      int trShift = getTransformShift(tu.cu->slice->getSPS()->getBitDepth(toChannelType(compID)), rect.size(),
                                      tu.cu->slice->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID)));
      scaleSAD *= pow(2, trShift);
    }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    trCosts.push_back( TrCost( int(std::min<double>(sumAbs*scaleSAD, std::numeric_limits<int>::max())), pos++ ) );
#else
    trCosts.push_back( TrCost( int(sumAbs*scaleSAD), pos++ ) );
#endif
    it++;
  }
#if JVET_AA0133_INTER_MTS_OPT
#if JVET_AG0061_INTER_LFNST_NSPT
  if (CU::isInter(*tu.cu) && (tu.cu->mtsFlag || tu.cu->lfnstFlag) && compID == COMPONENT_Y)
#else
  if (CU::isInter(*tu.cu) && tu.cu->mtsFlag && compID == COMPONENT_Y)
#endif
  {
    std::stable_sort(trCosts.begin(), trCosts.end(), [](const TrCost  l, const TrCost r) {return l.first < r.first; });
    std::vector<TrMode> trModesTemp;
    trModesTemp.resize(trModes->size());
    for (int i = 0; i < trModes->size(); i++)
    {
      trModesTemp[i] = trModes->at(i);
    }
    for (int i = 0; i < trModes->size(); i++)
    {
      int index = trCosts[i].second;
      trModes->at(i) = trModesTemp[index];
    }
    trModesTemp.resize(0);
    return;
  }
#endif
  int numTests = 0;
  std::vector<TrCost>::iterator itC = trCosts.begin();
  const double fac   = facBB[std::max(0, floorLog2(std::max(width, height)) - 2)];
  const double thr   = fac * trCosts.begin()->first;
  const double thrTS = trCosts.begin()->first;
  while( itC != trCosts.end() )
  {
#if JVET_Y0142_ADAPT_INTRA_MTS
    const bool testTr = itC->first <= ( trModes->at(itC->second).first == 1 ? thrTS : thr) && numTests <= maxCand;
#else
    const bool testTr = itC->first <= ( itC->second == 1 ? thrTS : thr ) && numTests <= maxCand;
#endif
    trModes->at( itC->second ).second = testTr;
    numTests += testTr;
    itC++;
  }
}
#if JVET_W0103_INTRA_MTS
// does transform for MTS candidates and return absSum of unquant Coeffs.
uint64_t TrQuant::transformNxN(TransformUnit& tu)
{
  CHECK(!tu.cu->mtsFlag, "mtsFlag should be on for selection");
  CodingStructure &cs = *tu.cs;
#if JVET_AL0181_ASBT
  const CompArea &rect = tu.blocksResidual[COMPONENT_Y];
#else
  const CompArea &rect = tu.blocks[COMPONENT_Y];
#endif
  const uint32_t uiWidth = rect.width;
  const uint32_t uiHeight = rect.height;

  const CPelBuf resiBuf = cs.getResiBuf(rect);
  CoeffBuf tempCoeff(m_mtsCoeffs[tu.mtsIdx[0]], rect);
  xT(tu, COMPONENT_Y, resiBuf, tempCoeff, uiWidth, uiHeight);


  const TCoeff *dstCoeffBuf = tempCoeff.buf;
  const int  dstCoeffStride = tempCoeff.stride;
  uint64_t coeffAbsSum = 0;

  for (int y = 0; y < uiHeight; y++)
  {
    for (int x = 0; x < uiWidth; x++)
    {
      coeffAbsSum += abs(dstCoeffBuf[(y * dstCoeffStride) + x]);
    }
  }
  return coeffAbsSum;
}
#endif
void TrQuant::transformNxN( TransformUnit& tu, const ComponentID& compID, const QpParam& cQP, TCoeff& uiAbsSum, const Ctx& ctx, const bool loadTr )
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
#if JVET_AL0181_ASBT
  const CompArea &rect = tu.blocksResidual[compID];
#else
  const CompArea &rect      = tu.blocks[compID];
#endif
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);

  if( tu.noResidual )
  {
    uiAbsSum = 0;
    TU::setCbfAtDepth( tu, compID, tu.depth, uiAbsSum > 0 );
    return;
  }

  if ((tu.cu->bdpcmMode && isLuma(compID)) || (!isLuma(compID) && tu.cu->bdpcmModeChroma))
  {
    tu.mtsIdx[compID] = MTS_SKIP;
  }

  uiAbsSum = 0;

  // transform and quantize
  CHECK(cs.sps->getMaxTbSize() < uiWidth, "Unsupported transformation size");

#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
  const int       kerCandNum = ( tu.cu->cs->sps->getUseLFNSTExt() || ( tu.cu->cs->sps->getUseNSPT() && CU::isNSPTAllowed( uiWidth, uiHeight ) ) ) ? 3 : 2;
  CoeffBuf tempCoeff( loadTr ? m_mtsCoeffs[ tu.lfnstIdx[ compID ] ? ( tu.lfnstIdx[ compID ] + NUM_TRAFO_MODES_MTS + kerCandNum * tu.lfnstIntra[ compID ] - 1 ) : tu.mtsIdx[ compID ] ] : m_tempCoeff, rect );
#else
  CoeffBuf tempCoeff(loadTr ? m_mtsCoeffs[tu.lfnstIdx[compID] ? (tu.lfnstIdx[compID] + NUM_TRAFO_MODES_MTS + 3 * tu.lfnstIntra[compID] - 1) : tu.mtsIdx[compID]] : m_tempCoeff, rect);
#endif
#else
  CoeffBuf tempCoeff(loadTr ? m_mtsCoeffs[tu.lfnstIdx[compID] ? tu.lfnstIdx[compID] + NUM_TRAFO_MODES_MTS - 1 : tu.mtsIdx[compID]] : m_tempCoeff, rect);
#endif
#else
  CoeffBuf tempCoeff(loadTr ? m_mtsCoeffs[tu.mtsIdx[compID]] : m_tempCoeff, rect);
#endif
  DTRACE_PEL_BUF(D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID);

  if (!loadTr)
  {
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      xTransformSkip(tu, compID, resiBuf, tempCoeff.buf);
    }
    else
    {
      xT(tu, compID, resiBuf, tempCoeff, uiWidth, uiHeight);
    }
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
    bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                  ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
    if( ( spsIntraLfnstEnabled && CU::isIntra( *tu.cu ) ) || ( sps.getUseInterLFNST() && CU::isInter( *tu.cu ) ) )
#else
    if (sps.getUseLFNST())
#endif
    {
      xFwdLfnst(tu, compID);
    }
#endif
  }
#if !JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  if( ( spsIntraLfnstEnabled && CU::isIntra( *tu.cu ) ) || ( sps.getUseInterLFNST() && CU::isInter( *tu.cu ) ) )
#else
  if (sps.getUseLFNST())
#endif
  {
    xFwdLfnst(tu, compID, loadTr);
  }
#endif

  DTRACE_COEFF_BUF(D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID);

  xQuant(tu, compID, tempCoeff, uiAbsSum, cQP, ctx);

  DTRACE_COEFF_BUF(D_TCOEFF, tu.getCoeffs(compID), tu, tu.cu->predMode, compID);

  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
}

void TrQuant::xTransformSkip(const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff)
{
#if JVET_AL0181_ASBT
  const CompArea &rect = tu.blocksResidual[compID];
#else
  const CompArea &rect = tu.blocks[compID];
#endif
  const uint32_t width = rect.width;
  const uint32_t height = rect.height;
  const Pel *pelResi = resi.buf;

  for (uint32_t y = 0, coefficientIndex = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++, coefficientIndex++)
    {
      psCoeff[coefficientIndex] = TCoeff(pelResi[x]);
    }
    pelResi += resi.stride;
  }
}

#if SIGN_PREDICTION
#if JVET_AL0181_ASBT
void TrQuant::predCoeffSigns(TransformUnit &tu, const ComponentID compID, const bool reshapeChroma, const Pel * tmpPredResiBorder)
#else
void TrQuant::predCoeffSigns(TransformUnit &tu, const ComponentID compID, const bool reshapeChroma)
#endif
{
  bool bIsJCCR = tu.jointCbCr && isChroma(compID);
  ComponentID residCompID = compID;
  bool bJccrWithCr = bIsJCCR && !(tu.jointCbCr >> 1);
#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
#if JVET_AJ0237_INTERNAL_12BIT
  int signPredShift = SIGN_PRED_RESIDUAL_BITS;
#else
  const int signPredShift = 10 + SIGN_PRED_RESIDUAL_BITS  - tu.cs->sps->getBitDepth(toChannelType(COMPONENT_Y));
#endif
  const int signPredOffset = 1 << (signPredShift - 1);
#endif
  if(bJccrWithCr)
  {
    residCompID = COMPONENT_Cr;
  }
  if( !( TU::getUseSignPred( tu, residCompID ) && ( TU::getCbf( tu, compID ) || bIsJCCR ) ) )
  {
    return;
  }

  if( bIsJCCR && compID == COMPONENT_Cr )
  {
    return;
  }

#if JVET_Y0141_SIGN_PRED_IMPROVE
  static_vector<Position, SIGN_PRED_MAX_NUM> predSignsXY;
  DepQuant::getPredictedSigns(tu, residCompID, predSignsXY, m_signsBuf, !tu.cs->pcv->isEncoder);
  int32_t numPredSigns = (int32_t)predSignsXY.size();

  if (!numPredSigns)
  {
    return;
  }
  auto setCoeffSign = [](CoeffBuf &buff, uint8_t *signBuf, static_vector<Position, SIGN_PRED_MAX_NUM> &pos) -> void
  {
    for (int i = 0; i < pos.size(); ++i)
    {
      bool bit_value = signBuf[i];
      TCoeff &coeff = buff.at(pos[i]);
      coeff = std::abs(coeff) * (bit_value ? -1 : 1);
    }
  };
  auto extractCoeffSign = [](CoeffBuf &buff, uint8_t *signBuf, static_vector<Position, SIGN_PRED_MAX_NUM> &pos) -> void
  {
    for (int i = 0; i < pos.size(); ++i)
    {
      uint32_t coeffSign = buff.at(pos[i]) < 0 ? 1 : 0;
      signBuf[i] = (uint8_t)coeffSign;
    }
  };
  auto setCoeffSignPositive = [](CoeffBuf &buff, static_vector<Position, SIGN_PRED_MAX_NUM> &pos) -> void
  {
    for (int i = 0; i < pos.size(); ++i)
    {
      TCoeff &coeff = buff.at(pos[i]);
      if (coeff < 0)
      {
        coeff = -coeff;
      }
    }
  };
#else
  auto setCoeffSign = [](CoeffBuf &buff, uint32_t signMask, static_vector<Position, SIGN_PRED_MAX_NUM> &pos) -> void
  {
    for( int i = 0, j = (int)pos.size() - 1; i < pos.size(); ++i, j-- )
    {
      bool bit_value = ( signMask >> j ) & 0x1;
      TCoeff &coeff = buff.at( pos[i] );
      coeff = std::abs( coeff ) * ( bit_value ? -1 : 1 );
    }
  };
  auto extractCoeffSign = [](CoeffBuf &buff, static_vector<Position, SIGN_PRED_MAX_NUM> &pos) -> uint32_t
  {
    uint32_t signMask = 0;
    for( int i = 0, j = (int)pos.size() - 1; i < pos.size(); ++i, j-- )
    {
      uint32_t coeffSign = buff.at( pos[i] ) < 0 ? 1 : 0;
      signMask |= coeffSign << j;
    }
    return signMask;
  };
#endif

#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
  auto createTemplate = [this, tu, signPredShift](ComponentID comp, uint32_t width, uint32_t height, uint32_t mtsIdx) -> void
#else
  auto createTemplate = [this,tu](ComponentID comp, uint32_t width, uint32_t height, uint32_t mtsIdx) -> void
#endif
  {
    // This is the function used to generate template values stored in g_initRomSignPred[]
    TCoeff *memCoeff = (TCoeff *)xMalloc(TCoeff, width*height);
    Pel      *memTmpResid = (Pel *)xMalloc(Pel,    width*height);
    CoeffBuf coeff(memCoeff, width, height);
    PelBuf   resi(memTmpResid, width, height);
    coeff.fill(0);

    const uint32_t stride = width + height;
    const uint32_t length = width + height;

#if JVET_Y0141_SIGN_PRED_IMPROVE
    int h = (height > SIGN_PRED_FREQ_RANGE) ? SIGN_PRED_FREQ_RANGE : height;
    int w = (width > SIGN_PRED_FREQ_RANGE) ? SIGN_PRED_FREQ_RANGE : width;
    int spArea = tu.cs->sps->getSignPredArea();
    int signPredWidth = std::min((int)width, spArea);
    int signPredHeight = std::min((int)height, spArea);
#if JVET_AJ0237_INTERNAL_12BIT
    int16_t* pTemplate = (int16_t*)xMalloc(int16_t, stride * h * w);
    AreaBuf<int16_t> templateBuf(pTemplate, stride, length, h * w);
#else
    int8_t         *pTemplate      = (int8_t *) xMalloc(int8_t, stride * h * w);
    AreaBuf<int8_t> templateBuf(pTemplate, stride, length, h * w);
#endif
#else
    int8_t *pTemplate = (int8_t *) xMalloc(int8_t, stride * SIGN_PRED_FREQ_RANGE * SIGN_PRED_FREQ_RANGE);
    AreaBuf<int8_t> templateBuf(pTemplate, stride, length, SIGN_PRED_FREQ_RANGE * SIGN_PRED_FREQ_RANGE);
#endif
    Position prev(0,0);
#if JVET_AJ0237_INTERNAL_12BIT
    int16_t* templ = templateBuf.buf;
#else
    int8_t *templ = templateBuf.buf;
#endif
#if JVET_Y0141_SIGN_PRED_IMPROVE
    for (int j = 0; j < signPredHeight*signPredWidth; ++j)
    {
      Position curr(j%signPredWidth, j / signPredWidth);
      int idx = curr.y * w + curr.x;
      templ = templateBuf.buf + templateBuf.stride * idx;
#else
    for( int j = 0; j < SIGN_PRED_FREQ_RANGE*SIGN_PRED_FREQ_RANGE; ++j)
    {
      Position curr(j%SIGN_PRED_FREQ_RANGE, j/SIGN_PRED_FREQ_RANGE);
#endif
      coeff.at(prev) = 0;
#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
      coeff.at(curr) = 1 << signPredShift;
#else
      // Is this the correct value to use? Shouldn't it depend on bit depth?
      coeff.at(curr) = 1 << SIGN_PRED_SHIFT;
#endif

      xIT( tu, comp, coeff, resi);

      Pel* pelResi = resi.bufAt(0, height - 1);

      for (uint32_t i = 0; i < height; i++)
      {
#if JVET_AJ0237_INTERNAL_12BIT
        templ[i] = (int16_t)(*pelResi);
#else
        CHECK(*pelResi < -128 || *pelResi > 127, "value exceeds 8-bit range");
        templ[i] = (int8_t)(*pelResi);
#endif
        pelResi -= resi.stride;
      }

      pelResi = resi.buf;

      for (uint32_t i = 0; i < width; i++)
      {
#if JVET_AJ0237_INTERNAL_12BIT
        templ[i + height] = (int16_t)pelResi[i];
#else
        CHECK(pelResi[i] < -128 || pelResi[i] > 127, "value exceeds 8-bit range");
        templ[i + height] = (int8_t) pelResi[i];
#endif
      }
#if !JVET_Y0141_SIGN_PRED_IMPROVE
      templ += templateBuf.stride;
#endif
      prev = curr;
    }

    int log2Width = floorLog2(width);
    int log2Height = floorLog2(height);
    g_resiBorderTemplate[log2Width-2][log2Height-2][mtsIdx] = templateBuf.buf;

    xFree(memCoeff);
    xFree(memTmpResid);
  };

#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
  auto createTemplateLFNST = [this, tu, signPredShift](ComponentID comp, uint32_t width, uint32_t height, uint32_t lfnstIdx) -> void
#else
  auto createTemplateLFNST = [this, tu](ComponentID comp, uint32_t width, uint32_t height, uint32_t lfnstIdx) -> void
#endif
  {
    const uint32_t stride = width + height;
    const uint32_t length = width + height;

    Pel      *memTmpResid = (Pel *)xMalloc(Pel, width*height);
    CoeffBuf coeff(m_tempCoeff, width, height);
    PelBuf   resi(memTmpResid, width, height);
    int signPredHeight = 4;
    int signPredWidth = 4;
#if JVET_AJ0237_INTERNAL_12BIT
    int16_t* pTemplate = (int16_t*)xMalloc(int16_t, stride * signPredHeight * signPredWidth);
    AreaBuf<int16_t> templateBuf(pTemplate, stride, length, signPredHeight* signPredWidth);
    int16_t* templ = templateBuf.buf;
#else
    int8_t         *pTemplate      = (int8_t *) xMalloc(int8_t, stride * signPredHeight * signPredWidth);
    AreaBuf<int8_t> templateBuf(pTemplate, stride, length, signPredHeight * signPredWidth);
    int8_t *templ = templateBuf.buf;
#endif
    for (int j = 0; j < signPredHeight*signPredWidth; ++j)
    {
      coeff.fill(0);
      Position curr((j%signPredWidth), (j / signPredWidth));
#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
      coeff.at(curr) = 1 << signPredShift;
#else
      coeff.at(curr) = 1 << SIGN_PRED_SHIFT;
#endif

      xInvLfnst(tu, comp);
      xIT(tu, comp, coeff, resi);

      Pel* pelResi = resi.bufAt(0, height - 1);

      for (uint32_t i = 0; i < height; i++)
      {
#if JVET_AJ0237_INTERNAL_12BIT
        templ[i] = (int16_t)(*pelResi);
#else
        CHECK(*pelResi < -128 || *pelResi > 127, "value exceeds 8-bit range");
        templ[i] = (int8_t)(*pelResi);
#endif
        pelResi -= resi.stride;
      }

      pelResi = resi.buf;

      for (uint32_t i = 0; i < width; i++)
      {
#if JVET_AJ0237_INTERNAL_12BIT
        templ[i + height] = (int16_t)pelResi[i];
#else
        CHECK(pelResi[i] < -128 || pelResi[i] > 127, "value exceeds 8-bit range");
        templ[i + height] = (int8_t) pelResi[i];
#endif
      }
      templ += templateBuf.stride;
    }

    int log2Width = floorLog2(width);
    int log2Height = floorLog2(height);
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
    bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
    bool allowNSPT = CU::isNSPTAllowed(tu, comp, width, height, spsIntraLfnstEnabled && CU::isIntra(*(tu.cu)));
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket, comp) : 0;
#else
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket) : 0;
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, comp) : 0;
#else
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu) : 0;
#endif
#endif
    g_resiBorderTemplateLFNST[nsptBucketIdx][log2Width - 2][log2Height - 2][lfnstIdx] = templateBuf.buf;
#else
    g_resiBorderTemplateLFNST[log2Width - 2][log2Height - 2][lfnstIdx] = templateBuf.buf;
#endif

    xFree(memTmpResid);
  };
#endif

  const QpParam cQP( tu, residCompID );
  CodingStructure &cs = *tu.cs;
  PelBuf recoBuf = cs.picture->getRecoBuf(tu.blocks[residCompID]);
  PelBuf predBuf = cs.getPredBuf(tu.blocks[residCompID]);
  Pel              predResiBorder[2 * SIGN_PRED_MAX_BS];
#if JVET_AL0181_ASBT
  if(tmpPredResiBorder == NULL)
  {
    TU::predBorderResi(tu.blocks[residCompID], recoBuf, predBuf, residCompID, tu.blocks[residCompID].width, tu.blocks[residCompID].height, predResiBorder, (1 << (tu.cs->sps->getBitDepth(toChannelType(residCompID)) - 1)));
  }
  else
  {
    memcpy(predResiBorder,tmpPredResiBorder, 2 * SIGN_PRED_MAX_BS * sizeof(Pel));
  }
  const uint32_t     uiWidth  = tu.blocksResidual[residCompID].width;
  const uint32_t     uiHeight = tu.blocksResidual[residCompID].height;
#else
  TU::predBorderResi(tu.blocks[residCompID], recoBuf, predBuf, residCompID, tu.blocks[residCompID].width, tu.blocks[residCompID].height, predResiBorder, (1 << (tu.cs->sps->getBitDepth(toChannelType(residCompID)) - 1)));

  const uint32_t     uiWidth  = tu.blocks[residCompID].width;
  const uint32_t     uiHeight = tu.blocks[residCompID].height;
#endif
  const uint32_t     stride   = uiWidth + uiHeight;
  const uint32_t     length   = uiWidth + uiHeight;

#if JVET_Y0141_SIGN_PRED_IMPROVE
  int log2Width = floorLog2(uiWidth);
  int log2Height = floorLog2(uiHeight);
  int actualTrIdx = 0, actualLfnstIdx = 0;
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  bool lfnstEnabled = ( ( ( spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) ) || ( tu.cu->cs->sps->getUseInterLFNST() && CU::isInter( *( tu.cu ) ) ) ) && tu.checkLFNSTApplied( residCompID ) );
#else
  bool lfnstEnabled = tu.checkLFNSTApplied(residCompID);
#endif
  if (lfnstEnabled)
  {
    actualLfnstIdx = getLfnstIdx(tu, residCompID);
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
    bool allowNSPT = CU::isNSPTAllowed(tu, compID, uiWidth, uiHeight, spsIntraLfnstEnabled && CU::isIntra(*(tu.cu)));
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket, compID) : 0;
#else
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket) : 0;
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, compID) : 0;
#else
    int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu) : 0;
#endif
#endif
    if (!g_resiBorderTemplateLFNST[nsptBucketIdx][log2Width - 2][log2Height - 2][actualLfnstIdx])
    {
      createTemplateLFNST(residCompID, uiWidth, uiHeight, actualLfnstIdx);
    }
#else
    if (!g_resiBorderTemplateLFNST[log2Width - 2][log2Height - 2][actualLfnstIdx])
    {
      createTemplateLFNST(residCompID, uiWidth, uiHeight, actualLfnstIdx);
    }
#endif
  }
  else
  {
    int trHor, trVer;
#if JVET_AJ0061_TIMD_MERGE
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
    if (tu.cu->timdMrg && !tu.cu->lfnstIdx && !tu.cs->sps->getUseImplicitMTS())
#else
    if (tu.cu->timdMrg && !tu.cu->lfnstIdx)
#endif
    {
      // Timd-Mrg CUs inherit transform type from their cands
      int implicitDst7 = PU::canTimdMergeImplicitDst7(tu);
      trHor = (implicitDst7 & 2) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][0];
      trVer = (implicitDst7 & 1) ? DST7 : tu.cu->timdmTrType[tu.cu->timdMrg - 1][1];
#if AHG7_MTS_TOOLOFF_CFG
      trHor = (uiWidth > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trHor;
      trVer = (uiHeight > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trVer;
#endif
    }
#endif
#if JVET_AM0074_INTRA_MERGE
#if JVET_AJ0061_TIMD_MERGE
    else
#endif
    if (tu.cu->intraMergeMode && !tu.cu->lfnstIdx)
    {
      int implicitDst7 = PU::canIntraMergeImplicitDst7(tu);
      trHor = (implicitDst7 & 2) ? DST7 : DCT2;
      trVer = (implicitDst7 & 1) ? DST7 : DCT2;
#if AHG7_MTS_TOOLOFF_CFG
      trHor = (uiWidth > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trHor;
      trVer = (uiHeight > tu.cs->sps->getIntraMTSMaxSize()) ? DCT2 : trVer;
#endif
    }
#endif
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
    else
    {
#endif
    getTrTypes(tu, residCompID, trHor, trVer);
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
    }
#endif
#if JVET_W0103_INTRA_MTS
    actualTrIdx = trHor * NUM_TRANS_TYPE + trVer;
#else
    actualTrIdx = trHor * 3 + trVer;
#endif
    if (!g_resiBorderTemplate[log2Width - 2][log2Height - 2][actualTrIdx])
    {
      createTemplate(residCompID, uiWidth, uiHeight, actualTrIdx);
    }
  }
#else
  int trHor, trVer;
  getTrTypes(tu, residCompID, trHor, trVer);
#if JVET_W0103_INTRA_MTS
  int actualTrIdx = trHor * NUM_TRANS_TYPE + trVer;
#else
  int actualTrIdx = trHor * 3 + trVer;
#endif
  int log2Width = floorLog2(uiWidth);
  int log2Height = floorLog2(uiHeight);
  if(!g_resiBorderTemplate[log2Width-2][log2Height-2][actualTrIdx])
  {
    createTemplate(residCompID, uiWidth, uiHeight, actualTrIdx);
  }
#endif
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_AL0181_ASBT
  const bool possibleTransposition = ((tu.asbtDecimH[compID] || tu.asbtDecimW[compID])
                                      && !((tu.asbtDecimH[compID] + tu.asbtDecimW[compID]) == 2 && (tu.blocks[compID].width <= 8 || tu.blocks[compID].height <= 8))
                                      && !(tu.blocks[compID].width <= 4) && !(tu.blocks[compID].height <= 4));
  const uint32_t spSize = ((lfnstEnabled || possibleTransposition) ? 4 : tu.cs->sps->getSignPredArea());
#else
  const uint32_t spSize = (lfnstEnabled ? 4 : tu.cs->sps->getSignPredArea());
#endif
  const uint32_t signPredWidth = std::min(uiWidth, spSize);
  const uint32_t signPredHeight = std::min(uiHeight, spSize);
  const uint32_t w = std::min(uiWidth, (uint32_t)SIGN_PRED_FREQ_RANGE);
  const uint32_t h = std::min(uiHeight, (uint32_t)SIGN_PRED_FREQ_RANGE);

#if JVET_AJ0237_INTERNAL_12BIT
  AreaBuf<const int16_t> templateNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int16_t>()
                  : AreaBuf<const int16_t>(g_resiBorderTemplate[log2Width - 2][log2Height - 2][actualTrIdx], stride,
                                          length, w * h));
#else
  AreaBuf<const int8_t> templateNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int8_t>()
                  : AreaBuf<const int8_t>(g_resiBorderTemplate[log2Width - 2][log2Height - 2][actualTrIdx], stride,
                                          length, w * h));
#endif
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  bool allowNSPT = CU::isNSPTAllowed( tu, compID, uiWidth, uiHeight, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#if JVET_AK0217_INTRA_MTSS
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
  int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket, compID) : 0;
#else
  int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, secondBucket) : 0;
#endif
#else
#if JVET_AL0215_NSPT_SET_FOR_INTRANN
  int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu, compID) : 0;
#else
  int  nsptBucketIdx = allowNSPT ? PU::getNSPTBucket(tu) : 0;
#endif
#endif
#if JVET_AJ0237_INTERNAL_12BIT
  AreaBuf<const int16_t> templateLfnstNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int16_t>(g_resiBorderTemplateLFNST[nsptBucketIdx][log2Width - 2][log2Height - 2][actualLfnstIdx],
                                          stride, length, signPredWidth * signPredHeight)
                  : AreaBuf<const int16_t>());
#else
  AreaBuf<const int8_t> templateLfnstNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int8_t>(g_resiBorderTemplateLFNST[nsptBucketIdx][log2Width - 2][log2Height - 2][actualLfnstIdx],
                                          stride, length, signPredWidth * signPredHeight)
                  : AreaBuf<const int8_t>());
#endif
#else
#if JVET_AJ0237_INTERNAL_12BIT
  AreaBuf<const int16_t> templateLfnstNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int16_t>(g_resiBorderTemplateLFNST[log2Width - 2][log2Height - 2][actualLfnstIdx],
                                          stride, length, signPredWidth * signPredHeight)
                  : AreaBuf<const int16_t>());
#else
  AreaBuf<const int8_t> templateLfnstNormalizedBuf =
    (lfnstEnabled ? AreaBuf<const int8_t>(g_resiBorderTemplateLFNST[log2Width - 2][log2Height - 2][actualLfnstIdx],
                                          stride, length, signPredWidth * signPredHeight)
                  : AreaBuf<const int8_t>());
#endif
#endif
#else
  AreaBuf<const int8_t> templateNormalizedBuf(g_resiBorderTemplate[log2Width - 2][log2Height - 2][actualTrIdx], stride,
                                              length, SIGN_PRED_FREQ_RANGE * SIGN_PRED_FREQ_RANGE);
  static_vector<Position, SIGN_PRED_MAX_NUM> predSignsXY;
  DepQuant::getPredictedSigns( tu, residCompID, predSignsXY );
  int32_t numPredSigns = (int32_t)predSignsXY.size();

  if( !numPredSigns )
  {
    return;
  }
#endif
  Pel m_signPredTemplate[SIGN_PRED_MAX_NUM * SIGN_PRED_MAX_BS * 2];

#if JVET_AL0181_ASBT
  CoeffBuf bufTmpQuant = CoeffBuf(m_tempCoeff, tu.blocksResidual[residCompID]);
#else
  CoeffBuf bufTmpQuant = CoeffBuf(m_tempCoeff, tu.blocks[residCompID]);
#endif
  PelBuf   piResi(m_tempSignPredResid, uiWidth, uiHeight);
  PelBuf   piResiCr(m_tempSignPredResid + uiWidth*uiHeight, uiWidth, uiHeight);

  CoeffBuf quantedCoeffBuff = tu.getCoeffs(residCompID);
  AreaBuf<SIGN_PRED_TYPE> bufSigns         = tu.getCoeffSigns(residCompID);

#if JVET_Y0141_SIGN_PRED_IMPROVE
  extractCoeffSign(quantedCoeffBuff, m_signsBuf, predSignsXY);
  setCoeffSignPositive(quantedCoeffBuff, predSignsXY);
#else
  uint32_t bufferSigns = extractCoeffSign(quantedCoeffBuff, predSignsXY);
  setCoeffSign(quantedCoeffBuff, 0, predSignsXY);
#endif

  xDeQuant( tu, bufTmpQuant, residCompID, cQP );

#if JVET_Y0141_SIGN_PRED_IMPROVE
  setCoeffSign(quantedCoeffBuff, m_signsBuf, predSignsXY);
#else
  setCoeffSign(quantedCoeffBuff, bufferSigns, predSignsXY);
#endif

  const bool useLeft = tu.blocks[residCompID].x != 0 || tu.blocks[residCompID].y == 0;
  const bool useTop  = tu.blocks[residCompID].y != 0 || tu.blocks[residCompID].x == 0;

  const int first = useLeft ? 0 : uiHeight;
  const int last  = useTop ? length : uiHeight;

  TCoeff *tmpQuant = bufTmpQuant.bufAt( 0, 0 );
  const ptrdiff_t tmpQuantStride = bufTmpQuant.stride;

  // Compute impact on template area of each coeff where sign is to be determined
  for (int predSignIdx = 0; predSignIdx < predSignsXY.size(); predSignIdx++)
  {
    const auto &xy = predSignsXY[predSignIdx];

    Pel         *templateVec = m_signPredTemplate + predSignIdx * stride;
    const TCoeff coeffVal    = tmpQuant[xy.y * tmpQuantStride + xy.x];

#if JVET_Y0141_SIGN_PRED_IMPROVE
    CHECK(coeffVal <= 0, "coefficient value should be positive");
#else
    CHECK(coeffVal == 0, "coefficient value should be nonzero");
#endif

#if JVET_AJ0237_INTERNAL_12BIT
    const int16_t* templateBasisVec;
#else
    const int8_t *templateBasisVec;
#endif

#if JVET_Y0141_SIGN_PRED_IMPROVE
    if (lfnstEnabled)
    {
      const int pos = xy.y * signPredWidth + xy.x;

      templateBasisVec = templateLfnstNormalizedBuf.bufAt(0, pos);
    }
    else
#endif
    {
      const int idx = xy.y * w + xy.x;

      templateBasisVec = templateNormalizedBuf.bufAt(0, idx);
    }

    for (int j = first; j < last; j++)
    {
      // coeffVal should be in -32768..32767 range and templateBasisVec[j] in -63..63
      // output range should be about -8064..8064
#if JVET_AI0096_SIGN_PRED_BIT_DEPTH_FIX
      templateVec[j] = (coeffVal * templateBasisVec[j] + signPredOffset) >> signPredShift;
#else
      templateVec[j] = (coeffVal * templateBasisVec[j] + SIGN_PRED_OFFSET) >> SIGN_PRED_SHIFT;
#endif
    }
  }

#if JVET_Y0141_SIGN_PRED_IMPROVE
  if (lfnstEnabled)
  {
    xInvLfnst(tu, residCompID);
  }
#endif
  if( bJccrWithCr )
  {
    xIT( tu, COMPONENT_Cr, bufTmpQuant, piResiCr );
  }
  else
  {
    xIT( tu, residCompID, bufTmpQuant, piResi );
  }

  Pel *pelResi = piResi.buf;
  ptrdiff_t resiStride = piResi.stride;

  if( bIsJCCR )
  {
    invTransformICT( tu, piResi, piResiCr );
    if( bJccrWithCr )
    {
      pelResi = piResiCr.buf;
      resiStride = piResiCr.stride;
    }
  }

  Pel predResiTemplate[2 * SIGN_PRED_MAX_BS];

  // Copy residual before any sign change
  Pel *resiTemplate = predResiTemplate + uiHeight;
  if (useLeft)
  {
    for (int i = 0; i < uiHeight; i++)
    {
      resiTemplate[~i] = pelResi[resiStride * i];
    }
  }

  if (useTop)
  {
    for (int i = 0; i < uiWidth; i++)
    {
      resiTemplate[i] = pelResi[i];
    }
  }

  int16_t chromaScale = 0;
  Pel     maxVal      = 0;
  if (reshapeChroma)
  {
    CHECK(tu.getChromaAdj() > std::numeric_limits<int16_t>::max(), "scale out of range");
    chromaScale = tu.getChromaAdj();
    maxVal      = (1 << tu.cu->cs->slice->clpRng(residCompID).bd) - 1;
  }

  // Compute SADs for all possible combinations of sign changes
  uint32_t cost = 0;

  if (reshapeChroma)
  {
    for (uint32_t i = first; i < last; i++)
    {
      cost += abs(predResiBorder[i] - Reshape::scalePel(predResiTemplate[i], chromaScale, maxVal));
    }
  }
  else
  {
    for (uint32_t i = first; i < last; i++)
    {
      cost += abs(predResiBorder[i] - predResiTemplate[i]);
    }
  }

  uint32_t costs[1 << SIGN_PRED_MAX_NUM];

  costs[0] = cost << SIGN_PRED_MAX_NUM;

  for (uint32_t idx = 1; idx < (1 << numPredSigns); idx++)
  {
    const uint32_t signXor  = idx & ~(idx - 1);
    const uint32_t curSigns = idx ^ (idx >> 1);

    const int16_t scale        = (curSigns & signXor) != 0 ? -2 : 2;
    const int  predSignIdx  = numPredSigns - 1 - floorLog2(signXor);

    const Pel *templ = m_signPredTemplate + predSignIdx * stride;

    // Compute cost of modification
    uint32_t cost = 0;

    if (reshapeChroma)
    {
      for (uint32_t i = first; i < last; i++)
      {
        predResiTemplate[i] += templ[i] * scale;
        cost += abs(predResiBorder[i] - Reshape::scalePel(predResiTemplate[i], chromaScale, maxVal));
      }
    }
    else
    {
      for (uint32_t i = first; i < last; i++)
      {
        predResiTemplate[i] += templ[i] * scale;
        cost += abs(predResiBorder[i] - predResiTemplate[i]);
      }
    }

    costs[curSigns] = (cost << SIGN_PRED_MAX_NUM) | curSigns;
  }

  uint32_t  minIdx            = 0;
  uint32_t  base              = 0;
  bool      resiSign          = true;
  uint32_t  signMask          = 1 << numPredSigns;

  for (uint32_t predSignIdx = 0; predSignIdx < numPredSigns; predSignIdx++)
  {
    const Position &xyPos = predSignsXY[predSignIdx];

    // Find predicted sign value
    if( resiSign )
    {
      uint32_t minVal = ~0u;
      for (uint32_t idx = base; idx < base + signMask; idx++)
      {
        minVal = std::min(minVal, costs[idx]);
      }
      minIdx = minVal & ~(~0u << SIGN_PRED_MAX_NUM);
    }

    signMask >>= 1;

    const bool predSign = (minIdx & signMask) != 0;

    bool realSign;

    if( tu.cs->pcv->isEncoder )
    {
      realSign = quantedCoeffBuff.at(xyPos) < 0;
      resiSign = predSign ^ realSign;
    }
    else
    {
      resiSign = quantedCoeffBuff.at(xyPos) < 0;
      realSign = predSign ^ resiSign;
      if (predSign)
      {
        quantedCoeffBuff.at(xyPos) = -quantedCoeffBuff.at(xyPos);
      }
    }

    bufSigns.at( xyPos ) = predSign ? SIGN_PRED_NEGATIVE : SIGN_PRED_POSITIVE;

    if( realSign )
    {
      base += signMask;
    }
  }
}

#if JVET_Y0141_SIGN_PRED_IMPROVE
int TrQuant::getLfnstIdx(const TransformUnit &tu, ComponentID compID)
{
  const CompArea& area = tu.blocks[compID];
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;
#if JVET_AK0217_INTRA_MTSS 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform(secondBucket, tu, compID);
#endif
#else
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID ).first;
#else
  uint32_t intraMode = PU::getFinalIntraModeForTransform( tu, compID );
#endif
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
  bool allowNSPT = CU::isNSPTAllowed( tu, compID, area.width, area.height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
  bool allowNSPT = CU::isNSPTAllowed( tu, compID, area.width, area.height, CU::isIntra( *( tu.cu ) ) );
#endif
  bool transposeFlag = getTransposeFlag(intraMode);
#if JVET_AC0130_NSPT
  int mode = allowNSPT ? g_nsptLut[ intraMode ] : g_lfnstLut[ intraMode ];
#else
  int mode = g_lfnstLut[intraMode];
#endif
  int index = lfnstIdx - 1;
#if JVET_W0119_LFNST_EXTENSION || EXTENDED_LFNST
  int result = (transposeFlag * 105) + (index * 35) + mode;
  CHECK(!((result >= 0) && (result <= 209)), "invalid index output");
#else
  int result = (transposeFlag << 3) + (index << 2) + mode;
  CHECK(!((result >= 0) && (result <= 15)), "invalid index output");
#endif

  return result;
}
#endif
#endif
#if INTRA_TRANS_ENC_OPT
void TrQuant::forwardLfnst(TCoeff* src, TCoeff*& dst, const int8_t*& trMat, const int trSize, const int zeroOutSize)
{
  for (int j = 0; j < zeroOutSize; j++)
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*          srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    const int8_t* trMatTmp = trMat;
    TCoeff coef = 0;
    for (int i = 0; i < trSize; i++)
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *dst++ = (coef + 64) >> 7;
    trMat += trSize;
  }
}

void TrQuant::inverseLfnst(TCoeff* src, TCoeff*  dst, const int8_t*  trMat, const int trSize, const int zeroOutSize, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  for (int j = 0; j < trSize; j++)
  {
    TCoeff resi = 0;
    const int8_t* trMatTmp = trMat;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    TCoeff*       srcPtr = src;
#else
    int*          srcPtr = src;
#endif
    for (int i = 0; i < zeroOutSize; i++)
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    *dst++ = Clip3<TCoeff>(outputMinimum, outputMaximum, (resi + 64) >> 7);
#else
    *dst++ = Clip3(outputMinimum, outputMaximum, (int)(resi + 64) >> 7);
#endif
    trMat++;
  }
}
#endif

//! \}
