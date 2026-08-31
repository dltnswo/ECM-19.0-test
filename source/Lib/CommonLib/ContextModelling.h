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

/** \file     ContextModelling.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTMODELLING__
#define __CONTEXTMODELLING__


#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"
#if JVET_AG0143_INTER_INTRA
#include "CodingStructure.h"
#endif


#include <bitset>

struct CoeffCodingContext
{
public:
#if JVET_AL0181_ASBT
  CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm = false , bool isTranspose = false );
#else
  CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm = false );
#endif
#if SIGN_PREDICTION
  int  getPredSignsQualified() { return m_bSignPredQualified;}
#endif
public:
  void  initSubblock     ( int SubsetId, bool sigGroupFlag = false );
public:
  void  resetSigGroup   ()                      { m_sigCoeffGroupFlag.reset( m_subSetPos ); }
  void  setSigGroup     ()                      { m_sigCoeffGroupFlag.set( m_subSetPos ); }
  bool  noneSigGroup    ()                      { return m_sigCoeffGroupFlag.none(); }
  int   lastSubSet      ()                      { return ( maxNumCoeff() - 1 ) >> log2CGSize(); }
  bool  isLastSubSet    ()                      { return lastSubSet() == m_subSetId; }
  bool  only1stSigGroup ()                      { return m_sigCoeffGroupFlag.count()-m_sigCoeffGroupFlag[lastSubSet()]==0; }
  void  setScanPosLast  ( int       posLast )   { m_scanPosLast = posLast; }
#if JVET_AG0143_INTER_INTRA
  static inline bool getSwitchCondition(const CodingUnit &cu,ChannelType channelType) {
    bool            condition     =
      cu.predMode == MODE_INTRA && cu.slice->getSliceType() != I_SLICE &&
      ((!cu.tmpFlag && channelType == CHANNEL_TYPE_LUMA)||(channelType == CHANNEL_TYPE_CHROMA 
#if JVET_AH0136_CHROMA_REORDERING
        && !(cu.firstPU->intraDir[1] >= DBV_CHROMA_IDX && cu.firstPU->intraDir[1] <= DBV_CHROMA_IDX9)
#else
        && cu.firstPU->intraDir[1] != DBV_CHROMA_IDX
#endif
        ));
    assert(cu.slice->getSliceType() != I_SLICE || channelType == cu.chType);
    int tmpMaxSize = cu.cs->sps->getIntraTMPMaxSize();
    condition = condition || (cu.predMode == MODE_IBC && channelType == CHANNEL_TYPE_LUMA && cu.slice->getSliceType() == I_SLICE)
                || (cu.predMode == MODE_INTRA && ((channelType == CHANNEL_TYPE_LUMA && cu.tmpFlag && !cu.bdpcmMode && !cu.dimd
                                                && cu.lwidth() <= tmpMaxSize && cu.lheight() <= tmpMaxSize)
#if !JVET_AH0136_CHROMA_REORDERING
                  || (channelType == CHANNEL_TYPE_CHROMA && cu.firstPU->intraDir[1] == DBV_CHROMA_IDX)
#endif
                  ) && cu.slice->getSliceType() == I_SLICE);
    return condition;
  }
#endif
public:
  ComponentID     compID          ()                        const { return m_compID; }
  int             subSetId        ()                        const { return m_subSetId; }
  int             subSetPos       ()                        const { return m_subSetPos; }
  int             cgPosY          ()                        const { return m_subSetPosY; }
  int             cgPosX          ()                        const { return m_subSetPosX; }
  unsigned        width           ()                        const { return m_width; }
  unsigned        height          ()                        const { return m_height; }
  unsigned        log2CGWidth     ()                        const { return m_log2CGWidth; }
  unsigned        log2CGHeight    ()                        const { return m_log2CGHeight; }
  unsigned        log2CGSize      ()                        const { return m_log2CGSize; }
  bool            extPrec         ()                        const { return m_extendedPrecision; }
  int             maxLog2TrDRange ()                        const { return m_maxLog2TrDynamicRange; }
  unsigned        maxNumCoeff     ()                        const { return m_maxNumCoeff; }
  int             scanPosLast     ()                        const { return m_scanPosLast; }
  int             minSubPos       ()                        const { return m_minSubPos; }
  int             maxSubPos       ()                        const { return m_maxSubPos; }
  bool            isLast          ()                        const { return ( ( m_scanPosLast >> m_log2CGSize ) == m_subSetId ); }
  bool            isNotFirst      ()                        const { return ( m_subSetId != 0 ); }
  bool            isSigGroup(int scanPosCG) const { return m_sigCoeffGroupFlag[m_scanCG[scanPosCG].idx]; }
  bool            isSigGroup      ()                        const { return m_sigCoeffGroupFlag[ m_subSetPos ]; }
  bool            signHiding      ()                        const { return m_signHiding; }
  bool            hideSign        ( int       posFirst,
                                    int       posLast   )   const { return ( m_signHiding && ( posLast - posFirst >= SBH_THRESHOLD ) ); }
  CoeffScanType   scanType        ()                        const { return m_scanType; }
  unsigned        blockPos(int scanPos) const { return m_scan[scanPos].idx; }
  unsigned        posX(int scanPos) const { return m_scan[scanPos].x; }
  unsigned        posY(int scanPos) const { return m_scan[scanPos].y; }
  unsigned        maxLastPosX     ()                        const { return m_maxLastPosX; }
  unsigned        maxLastPosY     ()                        const { return m_maxLastPosY; }
  unsigned        lastXCtxId      ( unsigned  posLastX  )   const { return m_ctxSetLastX( m_lastOffsetX + ( posLastX >> m_lastShiftX ) ); }
  unsigned        lastYCtxId      ( unsigned  posLastY  )   const { return m_ctxSetLastY( m_lastOffsetY + ( posLastY >> m_lastShiftY ) ); }
  int             numCtxBins      ()                        const { return   m_remainingContextBins;      }
  void            setNumCtxBins   ( int n )                       {          m_remainingContextBins  = n; }
#if JVET_AG0143_INTER_INTRA
  unsigned sigGroupCtxId(bool ts=false) const
  {
    if (m_switchCondition)
    {
        return ts ?
          m_sigGroupCtxIdTSSwitch
                  : 
          m_sigGroupCtxIdSwitch;
    }
    else
    {
      return ts ? m_sigGroupCtxIdTS : m_sigGroupCtxId;
    }
  }
#else
  unsigned        sigGroupCtxId   ( bool ts = false     )   const { return ts ? m_sigGroupCtxIdTS : m_sigGroupCtxId; }
#endif
  bool            bdpcm           ()                        const { return m_bdpcm; }

  void            decimateNumCtxBins(int n) { m_remainingContextBins -= n; }
  void            increaseNumCtxBins(int n) { m_remainingContextBins += n; }

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  TCoeff          minCoeff()                                const { return m_minCoeff; }
  TCoeff          maxCoeff()                                const { return m_maxCoeff; }
#endif

#if JVET_AE0102_LFNST_CTX 
  unsigned sigCtxIdAbs(int scanPos, const TCoeff* coeff, const int state, int lfnstIdx = -1)
#else
  unsigned sigCtxIdAbs( int scanPos, const TCoeff* coeff, const int state )
#endif
  {
    const uint32_t posY      = m_scan[scanPos].y;
    const uint32_t posX      = m_scan[scanPos].x;
    const TCoeff* pData     = coeff + posX + posY * m_width;
    const int     diag      = posX + posY;
    int           numPos    = 0;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff        sumAbs    = 0;
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
#define UPDATE(x) {TCoeff a=abs(x);sumAbs+=std::min(GTN_LEVEL+(a&1),a);numPos+=int(!!a);}
#else
#define UPDATE(x) {TCoeff a=abs(x);sumAbs+=std::min(4+(a&1),a);numPos+=int(!!a);}
#endif
#else
    int           sumAbs    = 0;
#define UPDATE(x) {int a=abs(x);sumAbs+=std::min(4+(a&1),a);numPos+=!!a;}
#endif
#if JVET_AE0102_LFNST_CTX
    if (lfnstIdx == 0)
    {
#endif
    if( posX < m_width-1 )
    {
      UPDATE( pData[1] );
      if( posX < m_width-2 )
      {
        UPDATE( pData[2] );
      }
      if( posY < m_height-1 )
      {
        UPDATE( pData[m_width+1] );
      }
    }
    if( posY < m_height-1 )
    {
      UPDATE( pData[m_width] );
      if( posY < m_height-2 )
      {
        UPDATE( pData[m_width<<1] );
      }
    }
#if JVET_AE0102_LFNST_CTX
    }
    else if (lfnstIdx > 0)
    {
      for (int k = 1; k <= std::min(5, scanPosLast() - scanPos); k++)
      {
        UPDATE( coeff[blockPos(scanPos + k)] );
      }
    }
#endif
#undef UPDATE


#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    int ctxOfs = int(std::min<TCoeff>((sumAbs + 1) >> 1, NSIGCTX-1)) + (diag < 2 ? NSIGCTX : 0);
#else
    int ctxOfs = int(std::min<TCoeff>((sumAbs+1)>>1, 3)) + ( diag < 2 ? 4 : 0 );
#endif
#else
    int ctxOfs = std::min((sumAbs+1)>>1, 3) + ( diag < 2 ? 4 : 0 );
#endif

    if( m_chType == CHANNEL_TYPE_LUMA )
    {
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      ctxOfs += diag < 5 ? NSIGCTX : 0;
#else
      ctxOfs += diag < 5 ? 4 : 0;
#endif
    }

    m_tmplCpDiag = diag;
    m_tmplCpSum1 = sumAbs - numPos;

#if TCQ_8STATES
    return m_sigFlagCtxSet[state & 3](ctxOfs);
#else
    return m_sigFlagCtxSet[std::max(0, state - 1)](ctxOfs);
#endif
  }

  uint8_t ctxOffsetAbs()
  {
    int offset = 0;
    if( m_tmplCpDiag != -1 )
    {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      offset = int(std::min<TCoeff>(m_tmplCpSum1, NGTXCTX-1)) + 1;
#else
      offset  = int(std::min<TCoeff>( m_tmplCpSum1, 4 )) + 1;
#endif
#else
      offset  = std::min( m_tmplCpSum1, 4 ) + 1;
#endif
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      offset += (!m_tmplCpDiag ? (m_chType == CHANNEL_TYPE_LUMA ? NGTXCTX*3 : NGTXCTX) : m_chType == CHANNEL_TYPE_LUMA ? m_tmplCpDiag < 3 ? NGTXCTX*2 : (m_tmplCpDiag < 10 ? NGTXCTX : 0) : 0);
#else
      offset += ( !m_tmplCpDiag ? ( m_chType == CHANNEL_TYPE_LUMA ? 15 : 5 ) : m_chType == CHANNEL_TYPE_LUMA ? m_tmplCpDiag < 3 ? 10 : ( m_tmplCpDiag < 10 ? 5 : 0 ) : 0 );
#endif
    }
    return uint8_t(offset);
  }


  unsigned parityCtxIdAbs   ( uint8_t offset )  const { return m_parFlagCtxSet   ( offset ); }


#if !JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  unsigned greater1CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[1]( offset ); }
  unsigned greater2CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[0]( offset ); }
#else
  unsigned greater1CtxIdAbs(uint8_t offset)  const { return m_gtxFlagCtxSet[1](offset); }
  unsigned greater2CtxIdAbs(uint8_t offset)  const { return m_gtxFlagCtxSet[2](offset); }
  unsigned greater3CtxIdAbs(uint8_t offset)  const { return m_gtxFlagCtxSet[0](offset); }
  unsigned greater4CtxIdAbs(uint8_t offset)  const { return m_gtxFlagCtxSet[3](offset); }
#endif


#if JVET_AE0102_LFNST_CTX
  void updateCtxSets()
  {
#if JVET_AG0143_INTER_INTRA
    if(m_switchCondition)
    {
      m_sigFlagCtxSet[0] = Ctx::SigFlagCtxSetSwitch[m_chType];
      m_sigFlagCtxSet[1] = Ctx::SigFlagCtxSetSwitch[m_chType + 2];
      m_sigFlagCtxSet[2] = Ctx::SigFlagCtxSetSwitch[m_chType];
      m_sigFlagCtxSet[3] = Ctx::SigFlagCtxSetSwitch[m_chType + 4];
      m_parFlagCtxSet = Ctx::ParFlagCtxSetSwitch[m_chType];
      m_gtxFlagCtxSet[0] = Ctx::GtxFlagCtxSetSwitch[m_chType];
      m_gtxFlagCtxSet[1] = Ctx::GtxFlagCtxSetSwitch[m_chType + 2];
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      m_gtxFlagCtxSet[2] = Ctx::GtxFlagCtxSetSwitch[m_chType + 4];
      m_gtxFlagCtxSet[3] = Ctx::GtxFlagCtxSetSwitch[m_chType + 6];
#endif
    }
    else
    {
      m_sigFlagCtxSet[0] = Ctx::SigFlag[m_chType];
      m_sigFlagCtxSet[1] = Ctx::SigFlag[m_chType + 2];
      m_sigFlagCtxSet[2] = Ctx::SigFlag[m_chType];
      m_sigFlagCtxSet[3] = Ctx::SigFlag[m_chType + 4];
      m_parFlagCtxSet = Ctx::ParFlag[m_chType];
      m_gtxFlagCtxSet[0] = Ctx::GtxFlag[m_chType];
      m_gtxFlagCtxSet[1] = Ctx::GtxFlag[m_chType + 2];
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      m_gtxFlagCtxSet[2] = Ctx::GtxFlag[m_chType + 4];
      m_gtxFlagCtxSet[3] = Ctx::GtxFlag[m_chType + 6];
#endif
    }
#else
    m_sigFlagCtxSet[0] = Ctx::SigFlag[m_chType];
    m_sigFlagCtxSet[1] = Ctx::SigFlag[m_chType + 2];
    m_sigFlagCtxSet[2] = Ctx::SigFlag[m_chType];
    m_sigFlagCtxSet[3] = Ctx::SigFlag[m_chType + 4];
    m_parFlagCtxSet    = Ctx::ParFlag[m_chType];
    m_gtxFlagCtxSet[0] = Ctx::GtxFlag[m_chType];
    m_gtxFlagCtxSet[1] = Ctx::GtxFlag[m_chType + 2];
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    m_gtxFlagCtxSet[2] = Ctx::GtxFlag[m_chType + 4];
    m_gtxFlagCtxSet[3] = Ctx::GtxFlag[m_chType + 6];
#endif
#endif

  }

  unsigned templateAbsSum(int scanPos, const TCoeff* coeff, int baseLevel, int lfnstIdx = 0)
#else
  unsigned templateAbsSum(int scanPos, const TCoeff* coeff, int baseLevel)
#endif
  {
    const uint32_t  posY  = m_scan[scanPos].y;
    const uint32_t  posX  = m_scan[scanPos].x;
    const TCoeff*   pData = coeff + posX + posY * m_width;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff          sum   = 0;
#else
    int             sum   = 0;
#endif
#if JVET_AE0102_LFNST_CTX
    if (lfnstIdx == 0 )
    {
#endif
    if (posX < m_width - 1)
    {
      sum += abs(pData[1]);
      if (posX < m_width - 2)
      {
        sum += abs(pData[2]);
      }
      if (posY < m_height - 1)
      {
        sum += abs(pData[m_width + 1]);
      }
    }
    if (posY < m_height - 1)
    {
      sum += abs(pData[m_width]);
      if (posY < m_height - 2)
      {
        sum += abs(pData[m_width << 1]);
      }
    }
#if JVET_AE0102_LFNST_CTX
    }
    else if (lfnstIdx > 0)
    {
      for (int k = 1; k <= std::min(5, scanPosLast() - scanPos); k++)
      {
        sum += abs(coeff[blockPos(scanPos + k)]);
      }
    }
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    return unsigned(std::max<TCoeff>(std::min<TCoeff>(sum - 5 * baseLevel, 31), 0));
#else
    return std::max(std::min(sum - 5 * baseLevel, 31), 0);
#endif
  }

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  unsigned templateAbsSum2(int scanPos, const TCoeff* coeff, int baseLevel, int lfnstIdx = 0)
  {
    const uint32_t  posY = m_scan[scanPos].y;
    const uint32_t  posX = m_scan[scanPos].x;
    const TCoeff* pData = coeff + posX + posY * m_width;
    TCoeff          sum = 0;
    TCoeff c = 0;

#if JVET_AE0102_LFNST_CTX
    if (lfnstIdx == 0)
    {
#endif
      if (posX < m_width - 1)
      {
        c = abs(pData[1]);
        sum += (c-(TCoeff)!!c);
        if (posX < m_width - 2)
        {
          c = abs(pData[2]);
          sum += (c - (TCoeff)!!c);
        }
        if (posY < m_height - 1)
        {
          c = abs(pData[m_width + 1]);
          sum += (c - (TCoeff)!!c);
        }
      }
      if (posY < m_height - 1)
      {
        c = abs(pData[m_width]);
        sum += (c - (TCoeff)!!c);
        if (posY < m_height - 2)
        {
          c= abs(pData[m_width << 1]);
          sum += (c - (TCoeff)!!c);
        }
      }
#if JVET_AE0102_LFNST_CTX
    }
    else if (lfnstIdx > 0)
    {
      for (int k = 1; k <= std::min(5, scanPosLast() - scanPos); k++)
      {
        c = abs(coeff[blockPos(scanPos + k)]);
        sum += (c - (TCoeff)!!c);
      }
    }
#endif
    return unsigned(std::max<TCoeff>(std::min<TCoeff>(sum , GTN_MAXSUM-1), 0));
  }
#endif
  
  unsigned sigCtxIdAbsTS(int scanPos, const TCoeff *coeff)
  {
    const uint32_t posY   = m_scan[scanPos].y;
    const uint32_t posX   = m_scan[scanPos].x;
    const TCoeff * posC   = coeff + posX + posY * m_width;
    int            numPos = 0;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
#define UPDATE(x) {TCoeff a=abs(x);numPos+=int(!!a);}
#else
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}
#endif
    if (posX > 0)
    {
      UPDATE(posC[-1]);
    }
    if (posY > 0)
    {
      UPDATE(posC[-(int) m_width]);
    }
#undef UPDATE

    return m_tsSigFlagCtxSet(numPos);
  }

  unsigned parityCtxIdAbsTS() const { return m_tsParFlagCtxSet(0); }

  unsigned greaterXCtxIdAbsTS(uint8_t offset) const { return m_tsGtxFlagCtxSet(offset); }


  unsigned        lrg1CtxIdAbsTS(int scanPos, const TCoeff *coeff, int bdpcm)
  {
    const uint32_t posY = m_scan[scanPos].y;
    const uint32_t posX = m_scan[scanPos].x;
    const TCoeff * posC = coeff + posX + posY * m_width;

    int             numPos = 0;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
#define UPDATE(x) {TCoeff a=abs(x);numPos+=int(!!a);}
#else
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}
#endif
    if (bdpcm)
    {
      numPos = 3;
    }
    else
    {
      if (posX > 0)
      {
        UPDATE(posC[-1]);
      }
      if (posY > 0)
      {
        UPDATE(posC[-(int) m_width]);
      }
    }
#undef UPDATE
    return m_tsLrg1FlagCtxSet(numPos);
  }

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  template <typename T> int sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }
#endif


  unsigned signCtxIdAbsTS(int scanPos, const TCoeff *coeff, int bdpcm)
  {
    const uint32_t posY  = m_scan[scanPos].y;
    const uint32_t posX  = m_scan[scanPos].x;
    const TCoeff * pData = coeff + posX + posY * m_width;

    int      rightSign = 0, belowSign = 0;
    unsigned signCtx = 0;

    if (posX > 0)
    {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      rightSign = sgn(pData[-1]);
#else
      rightSign = pData[-1];
#endif
    }
    if (posY > 0)
    {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      belowSign = sgn(pData[-(int) m_width]);
#else
      belowSign = pData[-(int) m_width];
#endif
    }

    if ((rightSign == 0 && belowSign == 0) || ((rightSign * belowSign) < 0))
    {
      signCtx = 0;
    }
    else if (rightSign >= 0 && belowSign >= 0)
    {
      signCtx = 1;
    }
    else
    {
      signCtx = 2;
    }
    if (bdpcm)
    {
      signCtx += 3;
    }
    return m_tsSignFlagCtxSet(signCtx);
  }

  void neighTS(int &rightPixel, int &belowPixel, int scanPos, const TCoeff* coeff)
  {
    const uint32_t  posY = m_scan[scanPos].y;
    const uint32_t  posX = m_scan[scanPos].x;
    const TCoeff*   data = coeff + posX + posY * m_width;

    rightPixel = belowPixel = 0;

    if (posX > 0)
    {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
      rightPixel = int(data[-1]);
#else
      rightPixel = data[-1];
#endif
    }
    if (posY > 0)
    {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
      belowPixel = int(data[-(int)m_width]);
#else
      belowPixel = data[-(int)m_width];
#endif
    }
  }

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  int deriveModCoeff(int rightPixel, int belowPixel, TCoeff absCoeff, int bdpcm = 0)
#else
  int deriveModCoeff(int rightPixel, int belowPixel, int absCoeff, int bdpcm = 0)
#endif
  {

    if (absCoeff == 0)
      return 0;
    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    int absCoeffMod = int(absCoeff);
#else
    int absCoeffMod = absCoeff;
#endif

    if (bdpcm == 0)
    {
      pred1 = std::max(absBelow, absRight);

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
      if (absCoeffMod == pred1)
#else
      if (absCoeff == pred1)
#endif
      {
        absCoeffMod = 1;
      }
      else
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
        absCoeffMod = absCoeffMod < pred1 ? absCoeffMod + 1 : absCoeffMod;
#else
        absCoeffMod = absCoeff < pred1 ? absCoeff + 1 : absCoeff;
#endif
      }
    }

    return(absCoeffMod);
  }

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  TCoeff decDeriveModCoeff(int rightPixel, int belowPixel, TCoeff absCoeff)
#else
  int decDeriveModCoeff(int rightPixel, int belowPixel, int absCoeff)
#endif
  {

    if (absCoeff == 0)
      return 0;

    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);
    pred1 = std::max(absBelow, absRight);

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff absCoeffMod;
#else
    int absCoeffMod;
#endif

    if (absCoeff == 1 && pred1 > 0)
    {
      absCoeffMod = pred1;
    }
    else
    {
      absCoeffMod = absCoeff - (absCoeff <= pred1);
    }
    return(absCoeffMod);
  }

  unsigned templateAbsSumTS( int scanPos, const TCoeff* coeff )
  {
    return 1;
  }

  int                       regBinLimit;

private:
  // constant
  const ComponentID         m_compID;
  const ChannelType         m_chType;
  const unsigned            m_width;
  const unsigned            m_height;
  const unsigned            m_log2CGWidth;
  const unsigned            m_log2CGHeight;
  const unsigned            m_log2CGSize;
  const unsigned            m_widthInGroups;
  const unsigned            m_heightInGroups;
  const unsigned            m_log2BlockWidth;
  const unsigned            m_log2BlockHeight;
  const unsigned            m_maxNumCoeff;
  const bool                m_signHiding;
  const bool                m_extendedPrecision;
  const int                 m_maxLog2TrDynamicRange;
  CoeffScanType             m_scanType;
  const ScanElement *       m_scan;
  const ScanElement *       m_scanCG;
#if JVET_AG0143_INTER_INTRA
  const bool                m_switchCondition;
#endif
  const CtxSet              m_ctxSetLastX;
  const CtxSet              m_ctxSetLastY;
  const unsigned            m_maxLastPosX;
  const unsigned            m_maxLastPosY;
  const int                 m_lastOffsetX;
  const int                 m_lastOffsetY;
  const int                 m_lastShiftX;
  const int                 m_lastShiftY;
  const bool                m_TrafoBypass;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const TCoeff              m_minCoeff;
  const TCoeff              m_maxCoeff;
#endif
  // modified
  int                       m_scanPosLast;
  int                       m_subSetId;
  int                       m_subSetPos;
  int                       m_subSetPosX;
  int                       m_subSetPosY;
  int                       m_minSubPos;
  int                       m_maxSubPos;
  unsigned                  m_sigGroupCtxId;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  TCoeff                    m_tmplCpSum1;
#else
  int                       m_tmplCpSum1;
#endif
  int                       m_tmplCpDiag;
#if JVET_AG0143_INTER_INTRA
  unsigned                  m_sigGroupCtxIdSwitch;
  unsigned                  m_sigGroupCtxIdTSSwitch;
#endif
#if TCQ_8STATES
  CtxSet                    m_sigFlagCtxSet[4];
#else
  CtxSet                    m_sigFlagCtxSet[3];
#endif
  CtxSet                    m_parFlagCtxSet;
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  CtxSet                    m_gtxFlagCtxSet[4];
#else
  CtxSet                    m_gtxFlagCtxSet[2];
#endif
  unsigned                  m_sigGroupCtxIdTS;
  CtxSet                    m_tsSigFlagCtxSet;
  CtxSet                    m_tsParFlagCtxSet;
  CtxSet                    m_tsGtxFlagCtxSet;
  CtxSet                    m_tsLrg1FlagCtxSet;
  CtxSet                    m_tsSignFlagCtxSet;
  int                       m_remainingContextBins;
  std::bitset<MLS_GRP_NUM>  m_sigCoeffGroupFlag;
  const bool                m_bdpcm;
#if SIGN_PREDICTION
  int                       m_bSignPredQualified;
#endif
};


class CUCtx
{
public:
  CUCtx()              : isDQPCoded(false), isChromaQpAdjCoded(false),
                         qgStart(false)
                         {
                           violatesLfnstConstrained[CHANNEL_TYPE_LUMA  ] = false;
                           violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
                           lfnstLastScanPos                              = false;
                           violatesMtsCoeffConstraint                    = false;
                           mtsLastScanPos                                = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
                           mtsCoeffAbsSum                                = 0;
#endif
                         }
  CUCtx(int _qp)       : isDQPCoded(false), isChromaQpAdjCoded(false),
                         qgStart(false),
                         qp(_qp)
                         {
                           violatesLfnstConstrained[CHANNEL_TYPE_LUMA  ] = false;
                           violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
                           lfnstLastScanPos                              = false;
                           violatesMtsCoeffConstraint                    = false;
                           mtsLastScanPos                                = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
                           mtsCoeffAbsSum                                = 0;
#endif
                         }
  ~CUCtx() {}
public:
  bool      isDQPCoded;
  bool      isChromaQpAdjCoded;
  bool      qgStart;
  bool      lfnstLastScanPos;
  int8_t    qp;                   // used as a previous(last) QP and for QP prediction
  bool      violatesLfnstConstrained[MAX_NUM_CHANNEL_TYPE];
  bool      violatesMtsCoeffConstraint;
  bool      mtsLastScanPos;
#if JVET_Y0142_ADAPT_INTRA_MTS
  int64_t   mtsCoeffAbsSum;
#endif
};

class MergeCtx
{
public:
#if JVET_AM0106_CHAIN_AMVP_MERGE
  Position      posNeighbours[NUM_MERGE_CANDS];
#endif
#if (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM) || JVET_Z0075_IBC_HMVP_ENLARGE
  MvField       mvFieldNeighbours[NUM_MERGE_CANDS << 1]; // double length for mv of both lists
  uint8_t       bcwIdx[NUM_MERGE_CANDS];
#if INTER_LIC
  bool          licFlags[NUM_MERGE_CANDS];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool          licInheritPara[NUM_MERGE_CANDS];
  int16_t       licScale[NUM_MERGE_CANDS][2][3];
  int16_t       licOffset[NUM_MERGE_CANDS][2][3];
#endif
#endif
#if JVET_AC0112_IBC_LIC
  bool          ibcLicFlags[NUM_MERGE_CANDS];
#if JVET_AE0078_IBC_LIC_EXTENSION
  int           ibcLicIndex[NUM_MERGE_CANDS];
#endif
#endif
#if JVET_AE0159_FIBC
  bool          ibcFilterFlags[NUM_MERGE_CANDS];
#endif
#if JVET_AA0070_RRIBC
  int           rribcFlipTypes[NUM_MERGE_CANDS];
#endif
  unsigned char interDirNeighbours[NUM_MERGE_CANDS];
#if MULTI_HYP_PRED
  MultiHypVec   addHypNeighbours[NUM_MERGE_CANDS];
#endif
  Distortion    candCost[NUM_MERGE_CANDS];
#if JVET_AG0276_NLIC
  bool          altLMFlag[NUM_MERGE_CANDS];
  AltLMInterUnit altLMParaNeighbours[NUM_MERGE_CANDS];
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
  int           candtype[NUM_MERGE_CANDS];
#endif
#else
  MergeCtx() : numValidMergeCand( 0 ), hasMergedCandList( false ) { }
  ~MergeCtx() {}
public:
  MvField       mvFieldNeighbours [ MRG_MAX_NUM_CANDS << 1 ]; // double length for mv of both lists
  uint8_t       bcwIdx            [ MRG_MAX_NUM_CANDS      ];
#if INTER_LIC
  bool          licFlags          [ MRG_MAX_NUM_CANDS      ];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool          licInheritPara[MRG_MAX_NUM_CANDS];
  int16_t       licScale[MRG_MAX_NUM_CANDS][2][3];
  int16_t       licOffset[MRG_MAX_NUM_CANDS][2][3];
#endif
#endif
#if JVET_AC0112_IBC_LIC
  bool          ibcLicFlags       [ MRG_MAX_NUM_CANDS      ];
#if JVET_AE0078_IBC_LIC_EXTENSION
  int           ibcLicIndex       [ MRG_MAX_NUM_CANDS      ];
#endif
#endif
#if JVET_AE0159_FIBC
  bool          ibcFilterFlags    [ MRG_MAX_NUM_CANDS      ];
#endif
#if JVET_AA0070_RRIBC
  int rribcFlipTypes[MRG_MAX_NUM_CANDS];
#endif
  unsigned char interDirNeighbours[ MRG_MAX_NUM_CANDS      ];
#if MULTI_HYP_PRED
  MultiHypVec   addHypNeighbours[MRG_MAX_NUM_CANDS];
#endif
#endif
  int           numValidMergeCand;
#if JVET_X0049_ADAPT_DMVR || JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
  int           numCandToTestEnc;
#endif
  bool          hasMergedCandList;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  int numAMVPMergeCand;
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if JVET_AG0098_AMVP_WITH_SBTMVP
  MotionBuf     subPuMvpMiBuf[SUB_BUFFER_SIZE];
#else
  MotionBuf     subPuMvpMiBuf[SUB_TMVP_NUM];
#endif
#else
  MotionBuf     subPuMvpMiBuf;
#endif
#if JVET_AL0160_SBSMVP
  MotionBuf     subSpatialPuMvpMiBuf[NUM_SUB_SMVP];
#else
  MotionBuf     subPuMvpExtMiBuf;
#endif

  MvField mmvdBaseMv[MMVD_BASE_MV_NUM][2];
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  void setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped = -1);
#else
  void setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx);
#endif
#if JVET_AA0061_IBC_MBVD
  MvField ibcMbvdBaseBv[IBC_MBVD_BASE_NUM][2];
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  MvField ibcMbvdBaseBvFrac[IBC_MBVD_BASE_NUM][2];
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  RefineTmvpCtxModellingParams refineTmvpParams;
  void copyRefineTmvpCtxModellingParams(MergeCtx &orgMergeCtx, uint32_t srcMrgIdxL0, uint32_t srcMrgIdxL1, uint32_t dstMrgIdxL0, uint32_t dstMrgIdxL1);
#endif
  bool setIbcMbvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped = -1, int candIdx1 = -1, int candIdxMaped1 = -1);
#else
  bool setIbcMbvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped = -1);
#endif
#endif
  bool          mmvdUseAltHpelIf  [ MMVD_BASE_MV_NUM ];
#if (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM) || JVET_Z0075_IBC_HMVP_ENLARGE
  bool          useAltHpelIf      [ NUM_MERGE_CANDS ];
#else
  bool          useAltHpelIf      [ MRG_MAX_NUM_CANDS ];
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void saveMergeInfo(PredictionUnit& puTmp, PredictionUnit pu);
#endif
  void setMergeInfo( PredictionUnit& pu, int candIdx );
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  template <typename MergeCtxType> static void setLICParamToPu         (const MergeCtxType& src,       PredictionUnit& pu, int candIdx, bool hasLIC);
  template <typename MergeCtxType> static void loadLICParamFromPu      (      MergeCtxType& dst, const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC);
  template <typename MergeCtxType> static void loadLICParamFromMotInfo (      MergeCtxType& dst, const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC);
  template <typename MergeCtxType> static void copyLICParamFromCtx     (      MergeCtxType& dst, int candIdxDst, const MergeCtxType& src, int candIdxSrc);
  template <typename MergeCtxType> static void setDefaultLICParamToCtx (      MergeCtxType& dst, int candIdx);
  template <typename MergeCtxType> static void setInheritAndLICFlags   (      MergeCtxType& dst, int candIdx);
#if JVET_AG0276_NLIC
  template <typename MergeCtxType> static void setLICParamUsingAltLM   (      MergeCtxType& dst, int candIdx);
#endif

  void setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC);
  void loadLICParamFromPu      (const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC);
  void loadLICParamFromMotInfo (const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC);
  void copyLICParamFromCtx     (int candIdx, const MergeCtx& src, int candIdxSrc);
  void setDefaultLICParamToCtx (int candIdx);
  void setInheritAndLICFlags   (int candIdx);
#if JVET_AG0276_NLIC
  void setLICParamUsingAltLM   (int candIdx);
#endif
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  int8_t getDir( Slice* slice, int candIdx, MvField* mvField 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
               , int* scale, int* offset
#endif
  );
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  void setIbcL1Info( PredictionUnit& pu, int candIdx );
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM || (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM) || JVET_Y0058_IBC_LIST_MODIFY || JVET_AE0046_BI_GPM
#if JVET_Z0075_IBC_HMVP_ENLARGE
  bool xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1, int compareNum = -1) const;
#else
  bool xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  bool xCheckSimilarMotion2Lists(int mergeCandIndex, MergeCtx *mrgCtx, uint32_t mvdSimilarityThresh = 1) const;
#endif
#endif
#if JVET_AD0213_LIC_IMP
  void initMrgCand(int cnt);
#endif
#if JVET_Z0084_IBC_TM
#if JVET_Z0075_IBC_HMVP_ENLARGE
  bool xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1, int compareNum = -1) const;
#else
  bool xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION                                
  bool xCheckSimilarMotionSubTMVP(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if TM_MRG
  void copyRegularMergeCand( int dstCandIdx, MergeCtx& srcCtx, int srcCandIdx );
  void convertRegularMergeCandToBi(int candIdx);
#endif
#if JVET_W0097_GPM_MMVD_TM
  void setGeoMmvdMergeInfo(PredictionUnit& pu, int mergeIdx, int mmvdIdx);
  void copyMergeCtx(MergeCtx &orgMergeCtx);
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  int   pocMrg[NUM_MERGE_CANDS];
  bool  mrgDuplicated[NUM_MERGE_CANDS];
  void  setGeoMrgDuplicate( const PredictionUnit& pu );
#endif
#if JVET_AN0236_GENERATED_MERGE_CANDIDATES 
  void copyToMergeCtx(const MergeCtx& other) // create new from existing
  {
    // create a copy of the list
    numValidMergeCand = other.numValidMergeCand;
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
    numCandToTestEnc = other.numCandToTestEnc;
#endif
    memcpy(bcwIdx, other.bcwIdx, numValidMergeCand * sizeof(uint8_t));
    memcpy(interDirNeighbours, other.interDirNeighbours, numValidMergeCand * sizeof(unsigned char));
    memcpy(posNeighbours, other.posNeighbours, numValidMergeCand * sizeof(Position));
    memcpy(mvFieldNeighbours, other.mvFieldNeighbours, (numValidMergeCand *2) * sizeof(MvField));
    memcpy(useAltHpelIf, other.useAltHpelIf, numValidMergeCand * sizeof(bool));
#if JVET_AG0276_NLIC
    memcpy(altLMFlag, other.altLMFlag, numValidMergeCand * sizeof(bool));
#endif
#if INTER_LIC 
    memcpy(licFlags,other.licFlags, numValidMergeCand * sizeof(bool));
    memcpy(candCost, other.candCost, numValidMergeCand * sizeof(Distortion));
    memcpy(candtype, other.candtype, numValidMergeCand * sizeof(int));
#if MULTI_HYP_PRED
    memcpy(addHypNeighbours, other.addHypNeighbours, numValidMergeCand * sizeof(MultiHypVec));
#endif
#endif 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    for (int ui = 0; ui < numValidMergeCand; ui++)
    {
      copyLICParamFromCtx(ui, other, ui);
#if JVET_AG0276_NLIC
      altLMParaNeighbours[ui]= other.altLMParaNeighbours[ui];
#endif
    }
#endif
  }
#endif
};
#if JVET_AG0164_AFFINE_GPM
class InterPrediction;
#endif

#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if JVET_AI0197_AFFINE_TMVP
class AffineMergeCtx
{
public:
  AffineMergeCtx()
    : numValidMergeCand(0)
#if JVET_AG0164_AFFINE_GPM
    , m_indexOffset(0)
    , m_isGPMAff(0)
#endif
  {
    for (unsigned i = 0; i < AFFINE_MRG_MAX_NUM_CANDS_ALL; i++)
    {
      affineType[i] = AFFINEMODEL_4PARAM;
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
      m_isConstructed[i] = false;
#endif
    }
  }
  ~AffineMergeCtx() {}

public:
  MvField       mvFieldNeighbours[AFFINE_MRG_MAX_NUM_CANDS_ALL << 1][3];   // double length for mv of both lists
  unsigned char interDirNeighbours[AFFINE_MRG_MAX_NUM_CANDS_ALL];
  Distortion    candCost[AFFINE_MRG_MAX_NUM_CANDS_ALL];
  EAffineModel  affineType[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#if JVET_AG0276_NLIC
  bool           altLMFlag[AFFINE_MRG_MAX_NUM_CANDS_ALL];
  AltLMInterUnit altLMParaNeighbours[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#endif
#if INTER_LIC
  bool licFlags[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool    licInheritPara[AFFINE_MRG_MAX_NUM_CANDS_ALL];
  int16_t licScale[AFFINE_MRG_MAX_NUM_CANDS_ALL][2][3];
  int16_t licOffset[AFFINE_MRG_MAX_NUM_CANDS_ALL][2][3];
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  int16_t licScaleBuf[AFFINE_MRG_MAX_NUM_CANDS_ALL][2][3];
  int16_t licOffsetBuf[AFFINE_MRG_MAX_NUM_CANDS_ALL][2][3];
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool obmcFlags[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#endif
  uint8_t bcwIdx[AFFINE_MRG_MAX_NUM_CANDS_ALL];
  int     numValidMergeCand;
  int     numAffCandToTestEnc;
  int     maxNumMergeCand;

  MergeCtx *mrgCtx;
  MergeType mergeType[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  bool m_isConstructed[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int colIdx[AFFINE_MRG_MAX_NUM_CANDS_ALL];
#endif
#if JVET_AB0112_AFFINE_DMVR
  bool xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AG0164_AFFINE_GPM
  int  m_indexOffset;
  int  m_isGPMAff;
  void setAffMergeInfo(PredictionUnit &pu, int candIdx, int8_t mmvdIdx = -1) const;
#endif
#if JVET_AG0276_NLIC
  bool xCheckSimilarMotion1(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1, bool isAltLM = false) const;
#endif
#if JVET_AH0119_SUBBLOCK_TM
  bool xCheckSimilarSbTMVP(PredictionUnit pu, int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void setLICParamToPu(PredictionUnit &pu, int candIdx, bool hasLIC);
  void setLICParamToPu(PredictionUnit &pu, int candIdx, bool hasLIC) const;
  void loadLICParamFromPu(const PredictionUnit *pu, int candIdx, bool allowAltModel, bool hasLIC);
  void loadLICParamFromMotInfo(const MotionInfo *mi, int candIdx, bool allowAltModel, bool hasLIC);
  void copyLICParamFromCtx(int candIdx, const AffineMergeCtx &src, int candIdxSrc);
  void setDefaultLICParamToCtx(int candIdx);
  void setInheritAndLICFlags(int candIdx);
#if JVET_AG0276_NLIC
  void setLICParamUsingAltLM(int candIdx);
#endif
#endif
};
#else
class AffineMergeCtx
{
public:
  AffineMergeCtx() : numValidMergeCand(0) 
#if JVET_AG0164_AFFINE_GPM
               , m_indexOffset(0)
               , m_isGPMAff(0)
#endif
  { for (unsigned i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++) affineType[i] = AFFINEMODEL_4PARAM; }
  ~AffineMergeCtx() {}
public:
  MvField       mvFieldNeighbours[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE << 1][3]; // double length for mv of both lists
  unsigned char interDirNeighbours[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  Distortion    candCost[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  EAffineModel  affineType[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#if JVET_AG0276_NLIC
  bool          altLMFlag[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  AltLMInterUnit altLMParaNeighbours[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#endif
#if INTER_LIC
  bool          licFlags[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool          licInheritPara[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  int16_t       licScale[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE][2][3];
  int16_t       licOffset[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE][2][3];
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool          obmcFlags[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#endif
  uint8_t       bcwIdx[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  int           numValidMergeCand;
  int           numAffCandToTestEnc;
  int           maxNumMergeCand;

  MergeCtx     *mrgCtx;
  MergeType     mergeType[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int           colIdx[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
#endif
#if JVET_AB0112_AFFINE_DMVR
  bool          xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AG0164_AFFINE_GPM
  int           m_indexOffset;
  int           m_isGPMAff;
  void          setAffMergeInfo(PredictionUnit &pu, int candIdx, int8_t mmvdIdx = -1) const;
#endif
#if JVET_AG0276_NLIC
  bool          xCheckSimilarMotion1(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1, bool isAltLM = false) const;
#endif
#if JVET_AH0119_SUBBLOCK_TM
  bool          xCheckSimilarSbTMVP(PredictionUnit pu, int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void          setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC);
  void          setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC) const;
  void          loadLICParamFromPu      (const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC);
  void          loadLICParamFromMotInfo (const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC);
  void          copyLICParamFromCtx     (int candIdx, const AffineMergeCtx& src, int candIdxSrc);
  void          setDefaultLICParamToCtx (int candIdx);
  void          setInheritAndLICFlags   (int candIdx);
#if JVET_AG0276_NLIC
  void          setLICParamUsingAltLM   (int candIdx);
#endif
#endif
};
#endif
#else
class AffineMergeCtx
{
public:
  AffineMergeCtx() : numValidMergeCand( 0 ) { for ( unsigned i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ ) affineType[i] = AFFINEMODEL_4PARAM; }
  ~AffineMergeCtx() {}
public:
  MvField       mvFieldNeighbours[AFFINE_MRG_MAX_NUM_CANDS << 1][3]; // double length for mv of both lists
  unsigned char interDirNeighbours[AFFINE_MRG_MAX_NUM_CANDS];
  EAffineModel  affineType[AFFINE_MRG_MAX_NUM_CANDS];
#if INTER_LIC
  bool          licFlags[AFFINE_MRG_MAX_NUM_CANDS];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool          licInheritPara[AFFINE_MRG_MAX_NUM_CANDS];
  int16_t       licScale[AFFINE_MRG_MAX_NUM_CANDS][2][3];
  int16_t       licOffset[AFFINE_MRG_MAX_NUM_CANDS][2][3];
#endif
#endif
  uint8_t       bcwIdx[AFFINE_MRG_MAX_NUM_CANDS];
  int           numValidMergeCand;
  int           maxNumMergeCand;

  MergeCtx     *mrgCtx;
  MergeType     mergeType[AFFINE_MRG_MAX_NUM_CANDS];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int           colIdx[AFFINE_MRG_MAX_NUM_CANDS];
#endif
#if JVET_AB0112_AFFINE_DMVR
  bool          xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  void          setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC);
  void          loadLICParamFromPu      (const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC);
  void          loadLICParamFromMotInfo (const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC);
  void          copyLICParamFromCtx     (int candIdx, const AffineMergeCtx& src, int candIdxSrc);
  void          setDefaultLICParamToCtx (int candIdx);
  void          setInheritAndLICFlags   (int candIdx);
#if JVET_AG0276_NLIC
  void          setLICParamUsingAltLM   (int candIdx);
#endif
#endif
};
#endif

#if JVET_AG0276_NLIC
class AltLMMergeCtx
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
: public MergeCtx
#endif
{
public:
  AltLMInterUnit altLMParaNeighbours[ALT_MRG_MAX_NUM_CANDS];
#if !JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  MvField        mvFieldNeighbours[ALT_MRG_MAX_NUM_CANDS << 1];
  uint8_t        bcwIdx[ALT_MRG_MAX_NUM_CANDS];
  unsigned char  interDirNeighbours[ALT_MRG_MAX_NUM_CANDS];
  bool           useAltHpelIf[ALT_MRG_MAX_NUM_CANDS];
  int            numValidMergeCand;
#endif

  void           initAltLMMergeCtx(int idx);
  bool           xCheckSameMotion(int cnt, uint32_t mvdSimilarityThresh);
};

class AltLMAffineMergeCtx
{
public:
  AltLMInterUnit  altLMParaNeighbours[ALT_AFF_MRG_MAX_NUM_CANDS];
  MvField         mvFieldNeighbours[ALT_AFF_MRG_MAX_NUM_CANDS << 1][3]; // double length for mv of both lists
  unsigned char   interDirNeighbours[ALT_AFF_MRG_MAX_NUM_CANDS];
  EAffineModel    affineType[ALT_AFF_MRG_MAX_NUM_CANDS];
  uint8_t         bcwIdx[ALT_AFF_MRG_MAX_NUM_CANDS];
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool            obmcFlags[ALT_AFF_MRG_MAX_NUM_CANDS];
#endif
  int             numValidMergeCand;
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  bool            m_isConstructed[ALT_AFF_MRG_MAX_NUM_CANDS];
#endif

  bool            xCheckSameAffMotion(const PredictionUnit& pu, int cnt);
  void            initAltLMAffMergeCtx(int idx);
  void            init();
  bool            xCheckSameAffMotion(const PredictionUnit& pu, int cnt, AltLMAffineMergeCtx& altAffMrgCtx1);
};
#endif

namespace DeriveCtx
{
void     CtxSplit     ( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* canSplit = nullptr 
#if JVET_AI0087_BTCUS_RESTRICTION
  ,bool disableBTV = false, bool disableBTH = false
#endif
);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
unsigned CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner );
#endif
unsigned CtxQtCbf     ( const ComponentID compID, const bool prevCbf = false, const int ispIdx = 0 );
unsigned CtxInterDir  ( const PredictionUnit& pu );
unsigned CtxSkipFlag  ( const CodingUnit& cu );
#if JVET_AG0276_LIC_SLOPE_ADJUST
unsigned CtxLicFlag  ( const CodingUnit& cu );
#endif
unsigned CtxAffineFlag( const CodingUnit& cu );
#if JVET_AG0135_AFFINE_CIIP
unsigned CtxCiipAffineFlag(const CodingUnit& cu);
#endif
#if JVET_AG0164_AFFINE_GPM
unsigned CtxGPMAffineFlag( const CodingUnit& cu );
#endif
#if JVET_X0049_ADAPT_DMVR
unsigned CtxBMMrgFlag(const CodingUnit& cu);
#endif
#if JVET_AA0070_RRIBC
unsigned CtxRribcFlipType(const CodingUnit& cu);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
unsigned CtxbvOneZeroComp(const CodingUnit &cu);
#endif
unsigned CtxPredModeFlag( const CodingUnit& cu );
unsigned CtxIBCFlag(const CodingUnit& cu);
unsigned CtxMipFlag   ( const CodingUnit& cu );
#if JVET_AD0086_ENHANCED_INTRA_TMP
unsigned CtxTmpFusionFlag( const CodingUnit& cu );
#endif
#if JVET_V0130_INTRA_TMP
unsigned CtxTmpFlag(const CodingUnit& cu);
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
uint16_t CtxPnnLuminanceFlag(const CodingUnit& cu);
#endif
unsigned CtxPltCopyFlag( const unsigned prevRunType, const unsigned dist );
#if ENABLE_DIMD
unsigned CtxDIMDFlag(const CodingUnit& cu);
#endif
#if JVET_W0123_TIMD_FUSION
unsigned CtxTimdFlag( const CodingUnit& cu );
#endif
#if JVET_AB0155_SGPM
unsigned CtxSgpmFlag(const CodingUnit &cu);
#endif
#if JVET_AD0140_MVD_PREDICTION
int ctxSmMvdBin(const int iPreviousBinIsCorrect2, const int iPreviousBinIsCorrect, const int isHor, const int significance, const MotionModel& motionModel);
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
int CtxSmBvdBin(const int iPreviousBinIsCorrect2, const int iPreviousBinIsCorrect, const int isHor, const int significance);
#endif
#if JVET_AE0159_FIBC
unsigned ctxIbcFilterFlag(const CodingUnit& cu);
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
unsigned CtxCUSeparateTree ( const CodingStructure& cs, Partitioner& partitioner );
#endif
}

#endif // __CONTEXTMODELLING__
