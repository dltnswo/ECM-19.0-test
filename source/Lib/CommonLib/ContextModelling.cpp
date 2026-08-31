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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"


#if JVET_AL0181_ASBT
CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm, bool isTranspose )
#else
CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm )
#endif
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
#if JVET_AL0181_ASBT
  , m_width                     (isTranspose ? (tu.blocks[m_compID].width >> tu.asbtDecimH[m_compID]) : tu.blocksResidual[m_compID].width)
#else
  , m_width                     (tu.block(m_compID).width)
#endif
#if JVET_AL0181_ASBT
  , m_height                    (isTranspose ? tu.blocks[m_compID].height : tu.blocksResidual[m_compID].height)
#else
  , m_height                    (tu.block(m_compID).height)
#endif
  , m_log2CGWidth               ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][0] )
  , m_log2CGHeight              ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][1] )
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) >> m_log2CGWidth)
  , m_heightInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) >> m_log2CGHeight)
  , m_log2BlockWidth            ((unsigned)floorLog2(m_width))
  , m_log2BlockHeight           ((unsigned)floorLog2(m_height))
  , m_maxNumCoeff               (m_width * m_height)
  , m_signHiding                (signHide)
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scanType                  (SCAN_DIAG)
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanCG                    (g_scanOrder     [SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
#if JVET_AG0143_INTER_INTRA
  , m_switchCondition           (getSwitchCondition(*tu.cu, m_chType))
  , m_ctxSetLastX               (m_switchCondition ? Ctx::LastXCtxSetSwitch[m_chType] : Ctx::LastX[m_chType])
  , m_ctxSetLastY               (m_switchCondition ? Ctx::LastYCtxSetSwitch[m_chType] : Ctx::LastY[m_chType])
#else
  , m_ctxSetLastX               (Ctx::LastX[m_chType])
  , m_ctxSetLastY               (Ctx::LastY[m_chType])
#endif
  , m_maxLastPosX(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) - 1])
  , m_maxLastPosY(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() && (tu.mtsIdx[m_compID] == MTS_SKIP))
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  , m_minCoeff                  (-(1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)))
  , m_maxCoeff                  ((1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)) - 1)
#endif
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
#if TCQ_8STATES
#if JVET_AG0143_INTER_INTRA
  , m_sigGroupCtxIdSwitch(-1)
  , m_sigGroupCtxIdTSSwitch(-1)
#if JVET_AE0102_LFNST_CTX
  , m_sigFlagCtxSet{ (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType]     : Ctx::SigFlag[m_chType])     : Ctx::SigFlagL[m_chType],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType + 2] : Ctx::SigFlag[m_chType + 2]) : Ctx::SigFlagL[m_chType + 2] ,
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType]     : Ctx::SigFlag[m_chType])     : Ctx::SigFlagL[m_chType],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType + 4] : Ctx::SigFlag[m_chType + 4]) : Ctx::SigFlagL[m_chType + 4]  }

#else
  , m_sigFlagCtxSet{m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType]     : Ctx::SigFlag[m_chType],
                    m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType + 2] : Ctx::SigFlag[m_chType + 2],
                    m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType]     : Ctx::SigFlag[m_chType],
                    m_switchCondition ? Ctx::SigFlagCtxSetSwitch[m_chType + 4] : Ctx::SigFlag[m_chType + 4] }
#endif
#else
#if JVET_AE0102_LFNST_CTX
  , m_sigFlagCtxSet             { (tu.cu->lfnstIdx == 0) ? Ctx::SigFlag[m_chType]     : Ctx::SigFlagL[m_chType],
                                  (tu.cu->lfnstIdx == 0) ? Ctx::SigFlag[m_chType + 2] : Ctx::SigFlagL[m_chType + 2],
                                  (tu.cu->lfnstIdx == 0) ? Ctx::SigFlag[m_chType]     : Ctx::SigFlagL[m_chType],
                                  (tu.cu->lfnstIdx == 0) ? Ctx::SigFlag[m_chType + 4] : Ctx::SigFlagL[m_chType + 4] }
#else
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+4] }
#endif
#endif
#else
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
#endif

#if JVET_AG0143_INTER_INTRA
#if JVET_AE0102_LFNST_CTX
  , m_parFlagCtxSet{ (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::ParFlagCtxSetSwitch[m_chType]     : Ctx::ParFlag[m_chType])     : Ctx::ParFlagL[m_chType]}
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  , m_gtxFlagCtxSet{ (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType]     : Ctx::GtxFlag[m_chType])     : Ctx::GtxFlagL[m_chType],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType + 2] : Ctx::GtxFlag[m_chType + 2]) : Ctx::GtxFlagL[m_chType + 2],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType + 4] : Ctx::GtxFlag[m_chType + 4]) : Ctx::GtxFlagL[m_chType + 4],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType + 6] : Ctx::GtxFlag[m_chType + 6]) : Ctx::GtxFlagL[m_chType + 6]}
#else
  , m_gtxFlagCtxSet{ (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType]     : Ctx::GtxFlag[m_chType])     : Ctx::GtxFlagL[m_chType],
                     (tu.cu->lfnstIdx == 0) ? (m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType + 2] : Ctx::GtxFlag[m_chType + 2]) : Ctx::GtxFlagL[m_chType + 2]}
#endif
#else
  , m_parFlagCtxSet{m_switchCondition ? Ctx::ParFlagCtxSetSwitch[m_chType]     : Ctx::ParFlag[m_chType]}
  , m_gtxFlagCtxSet{m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType]     : Ctx::GtxFlag[m_chType],
                    m_switchCondition ? Ctx::GtxFlagCtxSetSwitch[m_chType + 2] : Ctx::GtxFlag[m_chType + 2]}
#endif
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet  (m_switchCondition ? Ctx::TsSigFlagCtxSetSwitch       : Ctx::TsSigFlag)
  , m_tsParFlagCtxSet  (m_switchCondition ? Ctx::TsParFlagCtxSetSwitch       : Ctx::TsParFlag)
  , m_tsGtxFlagCtxSet  (m_switchCondition ? Ctx::TsGtxFlagCtxSetSwitch       : Ctx::TsGtxFlag)
  , m_tsLrg1FlagCtxSet (m_switchCondition ? Ctx::TsLrg1FlagCtxSetSwitch      : Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet (m_switchCondition ? Ctx::TsResidualSignCtxSetSwitch  : Ctx::TsResidualSign)
#else
#if JVET_AE0102_LFNST_CTX
  , m_parFlagCtxSet             { (tu.cu->lfnstIdx == 0) ? Ctx::ParFlag[m_chType] : Ctx::ParFlagL[m_chType] }
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  , m_gtxFlagCtxSet
{ 
  (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType] : Ctx::GtxFlagL[m_chType] , 
  (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType + 2] : Ctx::GtxFlagL[m_chType + 2],
  (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType + 4] : Ctx::GtxFlagL[m_chType + 4],
  (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType + 6] : Ctx::GtxFlagL[m_chType + 6],
}
#else
  , m_gtxFlagCtxSet             { (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType] : Ctx::GtxFlagL[m_chType] , (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType + 2] : Ctx::GtxFlagL[m_chType + 2] }
#endif
#else
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType] : Ctx::GtxFlagL[m_chType] , (tu.cu->lfnstIdx == 0) ? Ctx::GtxFlag[m_chType + 2] : Ctx::GtxFlagL[m_chType + 2] }
#endif
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet           ( Ctx::TsSigFlag )
  , m_tsParFlagCtxSet           ( Ctx::TsParFlag )
  , m_tsGtxFlagCtxSet           ( Ctx::TsGtxFlag )
  , m_tsLrg1FlagCtxSet          (Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet          (Ctx::TsResidualSign)
#endif
  , m_sigCoeffGroupFlag         ()
  , m_bdpcm                     (bdpcm)
#if SIGN_PREDICTION
  , m_bSignPredQualified        (TU::getDelayedSignCoding(tu, component))
#endif
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
    const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
  }
  else
  {
#if TU_256
    static const int prefix_ctx[]   = { 0, 0, 0, 3, 6, 10, 15, 21, 28 };
#else
    static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
#endif
    const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
    const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[m_subSetId].idx;
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  unsigned  sigLeft   = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
  unsigned  sigAbove  = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
  m_sigGroupCtxIdTS   = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
#if JVET_AG0143_INTER_INTRA 
  m_sigGroupCtxIdSwitch = Ctx::SigCoeffGroupCtxSetSwitch[m_chType](sigRight | sigLower);
  m_sigGroupCtxIdTSSwitch = Ctx::TsSigCoeffGroupCtxSetSwitch(sigLeft + sigAbove);
#endif
}

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
unsigned DeriveCtx::CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner )
{
  assert( partitioner.chType == CHANNEL_TYPE_LUMA );
  const Position pos = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;
  return ctxId;
}
#endif

void DeriveCtx::CtxSplit( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* _canSplit 
#if JVET_AI0087_BTCUS_RESTRICTION
  , bool disableBTV, bool disableBTH
#endif

/*= nullptr */ )
{
  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  // get left depth
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  // get above depth
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  bool canSplit[NUMBER_SPLITS];

  if( _canSplit == nullptr )
  {
#if JVET_AH0135_TEMPORAL_PARTITIONING
    unsigned maxMtt;
    partitioner.canSplit(cs, canSplit[NOSPLIT], canSplit[QTSPLIT], canSplit[BHSPLIT], canSplit[BVSPLIT], canSplit[THSPLIT], canSplit[TVSPLIT], maxMtt
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );
#else
    partitioner.canSplit( cs, canSplit[NOSPLIT], canSplit[QTSPLIT], canSplit[BHSPLIT], canSplit[BVSPLIT], canSplit[THSPLIT], canSplit[TVSPLIT]
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );
#endif
  }
  else
  {
    memcpy( canSplit, _canSplit, NUMBER_SPLITS * sizeof( bool ) );
  }

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().blocks[partitioner.chType].width;
  const unsigned heightCurr = partitioner.currArea().blocks[partitioner.chType].height;

  ctxSpl = 0;

#if JVET_AN0121_CONTEXT_CHANGE 
  if (cuLeft)
  {
    const unsigned widthtLeft = cuLeft->blocks[partitioner.chType].width;
    const unsigned heightLeft = cuLeft->blocks[partitioner.chType].height;
    ctxSpl += ((widthtLeft * heightLeft) < (widthCurr * heightCurr) ? 1 : 0);
  }
  if (cuAbove)
  {
    const unsigned heightAbove = cuAbove->blocks[partitioner.chType].height;
    const unsigned widthAbove  = cuAbove->blocks[partitioner.chType].width;
    ctxSpl += ((heightAbove * widthAbove) < (heightCurr * widthCurr) ? 1 : 0);
  }
#else
  if (cuLeft)
  {
    const unsigned heightLeft = cuLeft->blocks[partitioner.chType].height;
    ctxSpl += (heightLeft < heightCurr ? 1 : 0);
  }
  if (cuAbove)
  {
    const unsigned widthAbove = cuAbove->blocks[partitioner.chType].width;
    ctxSpl += (widthAbove < widthCurr ? 1 : 0);
  }
#endif

  unsigned numSplit = 0;
  if (canSplit[QTSPLIT])
  {
    numSplit += 2;
  }
  if (canSplit[BHSPLIT])
  {
    numSplit += 1;
  }
  if (canSplit[BVSPLIT])
  {
    numSplit += 1;
  }
  if (canSplit[THSPLIT])
  {
    numSplit += 1;
  }
  if (canSplit[TVSPLIT])
  {
    numSplit += 1;
  }

  if (numSplit > 0)
  {
    numSplit--;
  }

  ctxSpl += 3 * ( numSplit >> 1 );

#if JVET_AI0087_BTCUS_RESTRICTION
  if ((disableBTV || disableBTH) && partitioner.chType == CHANNEL_TYPE_LUMA)
  {
    ctxSpl = 9;
  }
#endif

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( canSplit[BHSPLIT] ? 1 : 0 ) + ( canSplit[THSPLIT] ? 1 : 0 );
  const unsigned numVer = ( canSplit[BVSPLIT] ? 1 : 0 ) + ( canSplit[TVSPLIT] ? 1 : 0 );

  if( numVer == numHor )
  {
    const Area& area = partitioner.currArea().blocks[partitioner.chType];

    const unsigned wAbove       = cuAbove ? cuAbove->blocks[partitioner.chType].width  : 1;
    const unsigned hLeft        = cuLeft  ? cuLeft ->blocks[partitioner.chType].height : 1;

    const unsigned depAbove     = area.width / wAbove;
    const unsigned depLeft      = area.height / hLeft;

    if (depAbove == depLeft || !cuLeft || !cuAbove)
    {
      ctxHv = 0;
    }
    else if (depAbove < depLeft)
    {
      ctxHv = 1;
    }
    else
    {
      ctxHv = 2;
    }
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = ( partitioner.currMtDepth <= 1 ? 1 : 0 );
  ctxVerBt = ( partitioner.currMtDepth <= 1 ? 3 : 2 );
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbf, const int ispIdx )
{
  if( ispIdx && isLuma( compID ) )
  {
    return 2 + (int)prevCbf;
  }
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbf ? 1 : 0 );
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
#if CTU_256
  return ( MAX_CU_DEPTH - ( ( floorLog2( pu.lumaSize().width ) + floorLog2( pu.lumaSize().height ) + 1 ) >> 1 ) );
#else
  return ( 7 - ((floorLog2(pu.lumaSize().width) + floorLog2(pu.lumaSize().height) + 1) >> 1) );
#endif
}

#if JVET_X0049_ADAPT_DMVR
unsigned DeriveCtx::CtxBMMrgFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && (cuLeft->firstPU->bmMergeFlag || (!cuLeft->firstPU->mergeFlag && cuLeft->firstPU->interDir == 3))) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && (cuAbove->firstPU->bmMergeFlag || (!cuAbove->firstPU->mergeFlag && cuAbove->firstPU->interDir == 3))) ? 1 : 0;

  return ctxId;
}
#endif

#if JVET_AA0070_RRIBC
unsigned DeriveCtx::CtxRribcFlipType(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->predMode == MODE_IBC && !cuLeft->firstPU->mergeFlag && cuLeft->rribcFlipType) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->predMode == MODE_IBC && !cuAbove->firstPU->mergeFlag && cuAbove->rribcFlipType) ? 1 : 0;

  return ctxId;
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
unsigned DeriveCtx::CtxbvOneZeroComp(const CodingUnit &cu)
{
  const CodingStructure *cs    = cu.cs;
  unsigned               ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->predMode == MODE_IBC && !cuLeft->firstPU->mergeFlag && cuLeft->bvOneZeroComp) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->predMode == MODE_IBC && !cuAbove->firstPU->mergeFlag && cuAbove->bvOneZeroComp) ? 1 : 0;

  return ctxId;
}
#endif
#if JVET_AG0135_AFFINE_CIIP
unsigned DeriveCtx::CtxCiipAffineFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && (cuLeft->affine || cuLeft->firstPU->ciipAffine)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && (cuAbove->affine || cuAbove->firstPU->ciipAffine)) ? 1 : 0;

  return ctxId;
}
#endif
unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

#if JVET_AG0164_AFFINE_GPM
  const CodingUnit* cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  ctxId = (cuLeft && ((cuLeft->affine && !cuLeft->firstPU->ciipAffine) || (cuLeft->geoFlag && (cuLeft->firstPU->affineGPM[0] || cuLeft->firstPU->affineGPM[1] || cuLeft->firstPU->sgpmAffine)))) ? 1 : 0;
#elif JVET_AG0135_AFFINE_CIIP
  ctxId = (cuLeft && ((cuLeft->affine && !cuLeft->firstPU->ciipAffine) || (cuLeft->geoFlag && (cuLeft->firstPU->affineGPM[0] || cuLeft->firstPU->affineGPM[1])))) ? 1 : 0;
#else
  ctxId = (cuLeft && (cuLeft->affine || (cuLeft->geoFlag && (cuLeft->firstPU->affineGPM[0] || cuLeft->firstPU->affineGPM[1])))) ? 1 : 0;
#endif

  const CodingUnit* cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  ctxId += (cuAbove && ((cuAbove->affine && !cuAbove->firstPU->ciipAffine) || (cuAbove->geoFlag && (cuAbove->firstPU->affineGPM[0] || cuAbove->firstPU->affineGPM[1] || cuAbove->firstPU->sgpmAffine)))) ? 1 : 0;
#elif JVET_AG0135_AFFINE_CIIP
  ctxId += (cuAbove && ((cuAbove->affine && !cuAbove->firstPU->ciipAffine) || (cuAbove->geoFlag && (cuAbove->firstPU->affineGPM[0] || cuAbove->firstPU->affineGPM[1])))) ? 1 : 0;
#else
  ctxId += (cuAbove && (cuAbove->affine || (cuAbove->geoFlag && (cuAbove->firstPU->affineGPM[0] || cuAbove->firstPU->affineGPM[1])))) ? 1 : 0;
#endif
#else
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
#if JVET_AG0135_AFFINE_CIIP
  ctxId = (cuLeft && (cuLeft->affine && !cuLeft->firstPU->ciipAffine)) ? 1 : 0;
#else
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;
#endif
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
#if JVET_AG0135_AFFINE_CIIP
  ctxId += (cuAbove && (cuAbove->affine && !cuAbove->firstPU->ciipAffine)) ? 1 : 0;
#else
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;
#endif
#endif
  return ctxId;
}

#if JVET_AG0164_AFFINE_GPM
unsigned DeriveCtx::CtxGPMAffineFlag(const CodingUnit& cu)
{
  const CodingStructure* cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit* cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && (cuLeft->affine || (cuLeft->geoFlag && ( cuLeft->firstPU->affineGPM[0] || cuLeft->firstPU->affineGPM[1])))) ? 1 : 0;

  const CodingUnit* cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && (cuAbove->affine || (cuAbove->geoFlag && (cuAbove->firstPU->affineGPM[0] || cuAbove->firstPU->affineGPM[1])))) ? 1 : 0;

  return ctxId;
}
#endif

unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}

#if JVET_AG0276_LIC_SLOPE_ADJUST
unsigned DeriveCtx::CtxLicFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && !cuLeft->firstPU->mergeFlag && !cuLeft->firstPU->amvpMergeModeFlag[0] && !cuLeft->firstPU->amvpMergeModeFlag[1] ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && !cuAbove->firstPU->mergeFlag && !cuAbove->firstPU->amvpMergeModeFlag[0] && !cuAbove->firstPU->amvpMergeModeFlag[1]  ) ? 1 : 0;

  return ctxId;
}
#endif

#if ENABLE_DIMD 
unsigned DeriveCtx::CtxDIMDFlag(const CodingUnit& cu)
{
#if 1 // one context
  return 0;
#else
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->dimd) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->dimd) ? 1 : 0;

  return ctxId;
#endif
}
#endif

#if JVET_W0123_TIMD_FUSION
unsigned DeriveCtx::CtxTimdFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = (cuLeft && cuLeft->timd) ? 1 : 0;
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += (cuAbove && cuAbove->timd) ? 1 : 0;
  return ctxId;
}
#endif

#if JVET_AB0155_SGPM
unsigned DeriveCtx::CtxSgpmFlag(const CodingUnit &cu)
{
  const CodingStructure *cs     = cu.cs;
  unsigned               ctxId  = 0;
  const CodingUnit *     cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId                         = (cuLeft && cuLeft->sgpm) ? 1 : 0;
  const CodingUnit *cuAbove     = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->sgpm) ? 1 : 0;
  return ctxId;
}
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
int DeriveCtx::CtxSmBvdBin(const int iPreviousBinIsCorrect2, const int iPreviousBinIsCorrect, const int isHor, const int significance)
{
  bool constexpr checkPrev2 = false;
  const int ctxNumInGroup = checkPrev2 ? 5 : 2;
  CHECK(iPreviousBinIsCorrect2 < -1, "Illegal iPreviousBinIsCorrect2");
  int iCtxIdx = (0 == iPreviousBinIsCorrect) ? 0 : 1;
  if (checkPrev2)
  {
    if (iCtxIdx == 0)
    {
      iCtxIdx = iPreviousBinIsCorrect2 < 0 ? 2 : iPreviousBinIsCorrect2; // # 0, 1 and 2 for cases 00, 10 and X0
    }
    else // (iCtxIdx == 1) 
    {
      iCtxIdx = 2 + (iPreviousBinIsCorrect2 < 0 ? 2 : iPreviousBinIsCorrect2); // # 2, 3 and 4 for cases 01, 11 and X1
    }
  }
  if (significance < 5)
  {
    return iCtxIdx;
  }
  return ctxNumInGroup + (0 == isHor ? 0 : ctxNumInGroup) + iCtxIdx;
}
#endif

#if JVET_AD0140_MVD_PREDICTION 
int DeriveCtx::ctxSmMvdBin(const int iPreviousBinIsCorrect2, const int iPreviousBinIsCorrect, const int isHor, const int significance, const MotionModel& motionModel)
{
  const bool isAffine = MotionModel::BiAffine == motionModel || MotionModel::UniAffine == motionModel;
  int ctxOffset = Clip3(0, 5, significance); 
  return (isAffine ? 5 : 0) + ctxOffset;
}
#endif

unsigned DeriveCtx::CtxPredModeFlag( const CodingUnit& cu )
{
  const CodingUnit *cuLeft  = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  const CodingUnit *cuAbove = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxIBCFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const Position pos = cu.chType == CHANNEL_TYPE_CHROMA ? cu.chromaPos() : cu.lumaPos();
  const CodingUnit *cuLeft = cs->getCURestricted(pos.offset(-1, 0), cu, cu.chType);
  ctxId += (cuLeft && CU::isIBC(*cuLeft)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(pos.offset(0, -1), cu, cu.chType);
  ctxId += (cuAbove && CU::isIBC(*cuAbove)) ? 1 : 0;
  return ctxId;
}

#if TM_MRG
void MergeCtx::copyRegularMergeCand(int dstCandIdx, MergeCtx& srcCtx, int srcCandIdx)
{
  if (this == (&srcCtx) && dstCandIdx == srcCandIdx)
  {
    return;
  }

  mvFieldNeighbours[ dstCandIdx << 1     ] = srcCtx.mvFieldNeighbours[ srcCandIdx << 1     ];
  mvFieldNeighbours[(dstCandIdx << 1) + 1] = srcCtx.mvFieldNeighbours[(srcCandIdx << 1) + 1];
  bcwIdx            [dstCandIdx] = srcCtx.bcwIdx            [srcCandIdx];
#if JVET_AG0276_NLIC
  altLMFlag[dstCandIdx] = srcCtx.altLMFlag[srcCandIdx];
  altLMParaNeighbours[dstCandIdx] = srcCtx.altLMParaNeighbours[srcCandIdx];
#endif
#if INTER_LIC
  licFlags          [dstCandIdx] = srcCtx.licFlags          [srcCandIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  copyLICParamFromCtx(dstCandIdx, srcCtx, srcCandIdx);
#endif
#endif
  interDirNeighbours[dstCandIdx] = srcCtx.interDirNeighbours[srcCandIdx];
  useAltHpelIf      [dstCandIdx] = srcCtx.useAltHpelIf      [srcCandIdx];
#if MULTI_HYP_PRED
  addHypNeighbours  [dstCandIdx] = srcCtx.addHypNeighbours  [srcCandIdx];
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
  candtype[dstCandIdx] = srcCtx.candtype[srcCandIdx];
#endif
}

void MergeCtx::convertRegularMergeCandToBi(int candIdx)
{
  if( interDirNeighbours[candIdx] < 3
#if INTER_LIC
      && !licFlags[candIdx]
#endif
    )
  {
    MvField& mvfL0 = mvFieldNeighbours[ candIdx << 1     ];
    MvField& mvfL1 = mvFieldNeighbours[(candIdx << 1) + 1];

    if (mvfL0.refIdx >= 0 && mvfL1.refIdx < 0)
    {
      mvfL1.mv = Mv() - mvfL0.mv;
      mvfL1.refIdx = 0;
    }
    else if (mvfL1.refIdx >= 0 && mvfL0.refIdx < 0)
    {
      mvfL0.mv = Mv() - mvfL1.mv;
      mvfL0.refIdx = 0;
    }
    else
    {
      CHECK(true, "Invalid merge candidate");
    }

    interDirNeighbours[candIdx] = 3;
    bcwIdx            [candIdx] = BCW_DEFAULT;
  } 
}
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void MergeCtx::saveMergeInfo(PredictionUnit& puTmp, PredictionUnit pu)
{
  puTmp.mergeIdx = pu.mergeIdx;
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  CHECK(candIdx >= numValidMergeCand, "Merge candidate does not exist");
#endif

  puTmp.regularMergeFlag = pu.regularMergeFlag;
  puTmp.mergeFlag = pu.mergeFlag;
  puTmp.mmvdMergeFlag = pu.mmvdMergeFlag;
  puTmp.interDir = pu.interDir;
  puTmp.cu->imv = pu.cu->imv;
  puTmp.mergeType = pu.mergeType;
  puTmp.mv[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0];
  puTmp.mv[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1];
#if MULTI_PASS_DMVR
  puTmp.bdmvrRefine = pu.bdmvrRefine;
#endif
  puTmp.refIdx[REF_PIC_LIST_0] = pu.refIdx[REF_PIC_LIST_0];
  puTmp.refIdx[REF_PIC_LIST_1] = pu.refIdx[REF_PIC_LIST_1];

  puTmp.bv = pu.bv;

  puTmp.cu->bcwIdx = pu.cu->bcwIdx;
  puTmp.addHypData = pu.addHypData;
  puTmp.numMergedAddHyps = pu.numMergedAddHyps;

  puTmp.mmvdEncOptMode = pu.mmvdEncOptMode;
  puTmp.cu->licFlag = pu.cu->licFlag;
#if JVET_AG0276_NLIC
  puTmp.cu->altLMFlag = pu.cu->altLMFlag;
  puTmp.cu->altLMParaUnit = pu.cu->altLMParaUnit;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  puTmp.cu->altLMBRParaUnit = pu.cu->altLMBRParaUnit;
#endif
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  puTmp.cu->licDelta = pu.cu->licDelta;
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  puTmp.cu->licInheritPara = pu.cu->licInheritPara;
  for (int list = 0; list < 2; list++)
  {
    for (int comp = 0; comp < 3; comp++)
    {
      puTmp.cu->licScale[list][comp] = pu.cu->licScale[list][comp];
      puTmp.cu->licOffset[list][comp] = pu.cu->licOffset[list][comp];
    }
  }
#endif
}
#endif
void MergeCtx::setMergeInfo(PredictionUnit& pu, int candIdx)
{
#if JVET_X0049_ADAPT_DMVR
  pu.mergeIdx = candIdx;
  if (pu.bmMergeFlag && pu.bmDir == 2)
  {
    candIdx -= BM_MRG_MAX_NUM_CANDS;
  }
#endif
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  CHECK(candIdx >= numValidMergeCand, "Merge candidate does not exist");
#endif

  pu.regularMergeFlag = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag = true;
  pu.mmvdMergeFlag = false;
  pu.interDir = interDirNeighbours[candIdx];
  pu.cu->imv = (!pu.cu->geoFlag && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
#if !JVET_X0049_ADAPT_DMVR
  pu.mergeIdx = candIdx;
#endif
  pu.mergeType = CU::isIBC(*pu.cu) ? MRG_TYPE_IBC : MRG_TYPE_DEFAULT_N;
  pu.mv[REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv[REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
  pu.refIdx[REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].refIdx;
  pu.refIdx[REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
#if JVET_AG0276_NLIC
  pu.cu->altLMFlag = altLMFlag[candIdx];
  pu.cu->altLMParaUnit = altLMParaNeighbours[candIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool initializeAltLM = !pu.cu->altLMFlag
                      && !pu.cu->geoFlag;
  if (initializeAltLM)
  {
    pu.cu->altLMParaUnit.resetAltLinearModel();
  }
#endif
#endif

  if (CU::isIBC(*pu.cu))
  {
    pu.bv = pu.mv[REF_PIC_LIST_0];
    pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (!pu.cs->sps->getIBCFracFlag())
#endif
      pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
#if MULTI_HYP_PRED
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
#endif
#if JVET_AC0112_IBC_LIC
    pu.cu->ibcLicFlag = ibcLicFlags[candIdx];
#if JVET_AE0078_IBC_LIC_EXTENSION
    pu.cu->ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
    pu.cu->ibcFilterFlag = ibcFilterFlags[candIdx];
#endif
#if JVET_AA0070_RRIBC
    pu.cu->rribcFlipType = rribcFlipTypes[candIdx];
  }
  else
  {
    pu.cu->rribcFlipType = 0;
#endif
  }
  pu.cu->bcwIdx = (interDirNeighbours[candIdx] == 3) ? bcwIdx[candIdx] : BCW_DEFAULT;
#if MULTI_HYP_PRED
  if (pu.ciipFlag
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    || pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
    || pu.bmMergeFlag
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    || pu.affBMMergeFlag
#endif
    )
  {
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
  }
#if MULTI_HYP_PRED
  else if (pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(pu.Y().width, pu.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
  }
#endif
  else
  {
    pu.addHypData = addHypNeighbours[candIdx];
    pu.numMergedAddHyps = int(addHypNeighbours[candIdx].size());
  }
#endif

#if !INTER_RM_SIZE_CONSTRAINTS  
  PU::restrictBiPredMergeCandsOne(pu);
#endif
  pu.mmvdEncOptMode = 0;

#if INTER_LIC
  pu.cu->licFlag = pu.cs->slice->getUseLIC() ? licFlags[candIdx] : false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  CHECK(!licFlags[candIdx] && licInheritPara[candIdx], "!licFlags[candIdx] && licInheritPara[candIdx]");
  setLICParamToPu(pu, candIdx, licInheritPara[candIdx]);
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  pu.cu->licDelta = 0;
#endif
#if !JVET_AD0213_LIC_IMP
  if (pu.interDir == 3)
  {
    CHECK(pu.cu->licFlag, "LIC is not used with bi-prediction in merge");
  }
#endif
#endif
}

#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
template <typename MergeCtxType>
void MergeCtx::setLICParamToPu(const MergeCtxType& src, PredictionUnit& pu, int candIdx, bool hasLIC)
{
  hasLIC &= pu.cs->slice->getUseLIC();
  if(hasLIC)
  {
    pu.cu->licInheritPara = src.licInheritPara[candIdx];
    for (int list = 0; list < 2; list++)
    {
      if (src.interDirNeighbours[candIdx] & (list + 1))
      {
        for (int comp = 0; comp < 3; comp++)
        {
          pu.cu->licScale [list][comp] = src.licScale [candIdx][list][comp];
          pu.cu->licOffset[list][comp] = src.licOffset[candIdx][list][comp];
        }
      }
      else
      {
        for (int comp = 0; comp < 3; comp++)
        {
          pu.cu->licScale [list][comp] = 32;
          pu.cu->licOffset[list][comp] = 0;
        }
      }
    }
  }
  else
  {
    pu.cu->licInheritPara = false;
    for (int list = 0; list < 2; list++)
    {
      for (int comp = 0; comp < 3; comp++)
      {
        pu.cu->licScale [list][comp] = 32;
        pu.cu->licOffset[list][comp] = 0;
      }
    }
  }
}

template <typename MergeCtxType>
void MergeCtx::loadLICParamFromPu(MergeCtxType& dst, const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC)
{
  hasLIC &= (pu != nullptr && pu->cs->slice->getUseLIC());
#if JVET_AG0276_NLIC
  allowAltModel &= (pu != nullptr);
  if (pu->cu->altLMFlag && !allowAltModel)
  {
    hasLIC = false;
  }
#endif

  if (!hasLIC)
  {
    MergeCtx::setDefaultLICParamToCtx(dst, candIdx);
#if JVET_AG0276_NLIC
    dst.altLMParaNeighbours[candIdx].resetAltLinearModel();
#endif
    return;
  }

  CHECK(pu == nullptr, "pu cannot be a null pointer");
  dst.licInheritPara[candIdx] = true;
#if JVET_AG0276_NLIC
  if (allowAltModel)
  {
    dst.altLMParaNeighbours[candIdx] = pu->cu->altLMParaUnit;
  }
  else
  {
    dst.altLMParaNeighbours[candIdx].resetAltLinearModel();
  }

#endif

#if JVET_AG0276_NLIC
  if (allowAltModel && pu->cu->altLMFlag)
  {
    setLICParamUsingAltLM(dst, candIdx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
    dst.altLMParaNeighbours[candIdx] = pu->cu->altLMBRParaUnit;
#endif
    return;
  }
#endif
  
  for (int list = 0; list < 2; list++)
  {
    for (int comp = 0; comp < 3; comp++)
    {
      dst.licScale [candIdx][list][comp] = pu->cu->licScale [list][comp];
      dst.licOffset[candIdx][list][comp] = pu->cu->licOffset[list][comp];
    }
  }
}

template <typename MergeCtxType>
void MergeCtx::loadLICParamFromMotInfo(MergeCtxType& dst, const MotionInfo* mi, int candIdx, bool allowAltModel, bool hasLIC)
{
  CHECK(true, "Invalid function");
}

template <typename MergeCtxType>
void MergeCtx::copyLICParamFromCtx(MergeCtxType& dst, int candIdxDst, const MergeCtxType& src, int candIdxSrc)
{
  dst.licInheritPara[candIdxDst] = src.licInheritPara[candIdxSrc];
  for (int list = 0; list < 2; list++)
  {
    if (src.interDirNeighbours[candIdxSrc] & (list + 1))
    {
      for (int comp = 0; comp < 3; comp++)
      {
        dst.licScale [candIdxDst][list][comp] = src.licScale [candIdxSrc][list][comp];
        dst.licOffset[candIdxDst][list][comp] = src.licOffset[candIdxSrc][list][comp];
      }
    }
    else
    {
      for (int comp = 0; comp < 3; comp++)
      {
        dst.licScale [candIdxDst][list][comp] = 32;
        dst.licOffset[candIdxDst][list][comp] = 0;
      }
    }
  }
}

template <typename MergeCtxType>
void MergeCtx::setDefaultLICParamToCtx(MergeCtxType& dst, int candIdx)
{
  dst.licInheritPara[candIdx] = false;
  for (int list = 0; list < 2; list++)
  {
    for (int comp = 0; comp < 3; comp++)
    {
      dst.licScale [candIdx][list][comp] = 32;
      dst.licOffset[candIdx][list][comp] = 0;
    }
  }
}

template <typename MergeCtxType>
void MergeCtx::setInheritAndLICFlags(MergeCtxType& dst, int candIdx)
{
  bool licL0 = false;
  bool licL1 = false;

  if (dst.interDirNeighbours[candIdx] & 1)
  {
    licL0 = dst.licScale [candIdx][0][0] != 32 || dst.licScale [candIdx][0][1] != 32 || dst.licScale [candIdx][0][2] != 32 ||
            dst.licOffset[candIdx][0][0] !=  0 || dst.licOffset[candIdx][0][1] !=  0 || dst.licOffset[candIdx][0][2] !=  0;
  }

  if (dst.interDirNeighbours[candIdx] & 2)
  {
    licL1 = dst.licScale [candIdx][1][0] != 32 || dst.licScale [candIdx][1][1] != 32 || dst.licScale [candIdx][1][2] != 32 ||
            dst.licOffset[candIdx][1][0] !=  0 || dst.licOffset[candIdx][1][1] !=  0 || dst.licOffset[candIdx][1][2] !=  0;
  }

  dst.licInheritPara[candIdx] = (licL0 | licL1);
  dst.licFlags[candIdx] = dst.licInheritPara[candIdx];
}

#if JVET_AG0276_NLIC
template <typename MergeCtxType>
void MergeCtx::setLICParamUsingAltLM(MergeCtxType& dst, int candIdx)
{
  dst.licInheritPara[candIdx] = true;
  for (int list = 0; list < 2; list++)
  {
    if (dst.interDirNeighbours[candIdx] & (list + 1))
    {
      for (int comp = 0; comp < 3; comp++)
      {
        dst.licScale [candIdx][list][comp] = dst.altLMParaNeighbours[candIdx].scale [comp];
        dst.licOffset[candIdx][list][comp] = dst.altLMParaNeighbours[candIdx].offset[comp];
      }
    }
    else
    {
      for (int comp = 0; comp < 3; comp++)
      {
        dst.licScale [candIdx][list][comp] = 32;
        dst.licOffset[candIdx][list][comp] = 0;
      }
    }
  }
}
#endif

void MergeCtx::setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC)                     { MergeCtx::setLICParamToPu         (*this, pu, candIdx, hasLIC);                }
void MergeCtx::loadLICParamFromPu      (const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC) { MergeCtx::loadLICParamFromPu      (*this, pu, candIdx, allowAltModel, hasLIC); }
void MergeCtx::loadLICParamFromMotInfo (const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC) { MergeCtx::loadLICParamFromMotInfo (*this, mi, candIdx, allowAltModel, hasLIC); }
void MergeCtx::copyLICParamFromCtx     (int candIdx, const MergeCtx& src, int candIdxSrc)                       { MergeCtx::copyLICParamFromCtx     (*this, candIdx, src, candIdxSrc);           }
void MergeCtx::setDefaultLICParamToCtx (int candIdx)                                                            { MergeCtx::setDefaultLICParamToCtx (*this, candIdx);                            }
void MergeCtx::setInheritAndLICFlags   (int candIdx)                                                            { MergeCtx::setInheritAndLICFlags   (*this, candIdx);                            }
#if JVET_AG0276_NLIC
void MergeCtx::setLICParamUsingAltLM   (int candIdx)                                                            { MergeCtx::setLICParamUsingAltLM   (*this, candIdx);                            }
#endif
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
int8_t  MergeCtx::getDir( Slice* slice, int candIdx, MvField *mvField 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                        , int* scale, int* offset
#endif
)
{
  if (candIdx < 0)
  {
    return (-1);
  }

  mvField[REF_PIC_LIST_0].refIdx = mvFieldNeighbours[(candIdx << 1) + 0].refIdx;
  mvField[REF_PIC_LIST_1].refIdx = mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
  mvField[REF_PIC_LIST_0].mv = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  mvField[REF_PIC_LIST_1].mv = mvFieldNeighbours[(candIdx << 1) + 1].mv;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  if (licFlags[candIdx])
  {
    scale [REF_PIC_LIST_0] = licScale [candIdx][0][COMPONENT_Y];
    scale [REF_PIC_LIST_1] = licScale [candIdx][1][COMPONENT_Y];
    offset[REF_PIC_LIST_0] = licOffset[candIdx][0][COMPONENT_Y];
    offset[REF_PIC_LIST_1] = licOffset[candIdx][1][COMPONENT_Y];
  }
  else
  {
    scale [REF_PIC_LIST_0] = 32;
    scale [REF_PIC_LIST_1] = 32;
    offset[REF_PIC_LIST_0] = 0;
    offset[REF_PIC_LIST_1] = 0;
  }
#endif

  int8_t  refIdx[2] = { -1, -1 };
  Mv      mv[2];
  refIdx[REF_PIC_LIST_0]  = mvField[REF_PIC_LIST_0].refIdx;
  refIdx[REF_PIC_LIST_1]  = mvField[REF_PIC_LIST_1].refIdx;
  mv[REF_PIC_LIST_0]      = mvField[REF_PIC_LIST_0].mv;
  mv[REF_PIC_LIST_1]      = mvField[REF_PIC_LIST_1].mv;
  if ( refIdx[0] == (-1) && refIdx[1] == (-1) )
  {
    return (-1);
  }

  if ( refIdx[0] == (-1) || refIdx[1] == (-1) ) 
  {
    return (refIdx[0] == (-1)) ? 1 : 0 ;
  }
  int poc0 = -1, poc1 = -1;
  if ( refIdx[0] != (-1) && refIdx[1] != (-1) ) 
  {
    poc0 = slice->getRefPic(REF_PIC_LIST_0, refIdx[0])->getPOC();
    poc1 = slice->getRefPic(REF_PIC_LIST_1, refIdx[1])->getPOC();
    if ( poc0 == poc1 && mv[0] == mv[1] 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      && scale [0] == scale[1] && offset[0] == offset[1]
#endif
      ) 
    {
      mvField[REF_PIC_LIST_1].refIdx = -1;
      mvField[REF_PIC_LIST_1].mv.setZero();
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      scale [REF_PIC_LIST_1] = 32;
      offset[REF_PIC_LIST_1] = 0;
#endif
      return 0;
    }
  }
  return 2; // bi-dir
}
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
void MergeCtx::setIbcL1Info( PredictionUnit& pu, int candIdx )
{
  pu.interDir = 3;
  pu.ibcMergeIdx1 = candIdx;
  pu.mv[REF_PIC_LIST_1] = mvFieldNeighbours[candIdx<<1].mv;
  pu.refIdx[REF_PIC_LIST_1] = MAX_NUM_REF;
  pu.cu->ibcLicFlag = 0;
  pu.cu->rribcFlipType = 0;
#if JVET_AE0159_FIBC
  pu.cu->ibcFilterFlag = false;
#endif
}
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION                                
#if JVET_AH0119_SUBBLOCK_TM
bool AffineMergeCtx::xCheckSimilarSbTMVP(PredictionUnit pu, int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx < 0)
  {
    return true;
  }
#if JVET_AI0183_MVP_EXTENSION
  Size      puSize = pu.lumaSize();
  int       numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int       numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int       puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int       puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  const int incrRow = (puWidth >> ATMVP_SUB_BLOCK_SIZE);
  const int incrColumn = mrgCtx->subPuMvpMiBuf[0].stride - (puSize.width >> ATMVP_SUB_BLOCK_SIZE);
#endif
  if (mvdSimilarityThresh > 1)
  {
    int mvdTh = mvdSimilarityThresh;
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        bool similarCheck = false;
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx
            && mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;

            if (mvDiffL0.getAbsHor() < mvdTh && mvDiffL0.getAbsVer() < mvdTh && mvDiffL1.getAbsHor() < mvdTh
              && mvDiffL1.getAbsVer() < mvdTh)
            {
              similarCheck = true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            if (mvDiff.getAbsHor() < mvdTh && mvDiff.getAbsVer() < mvdTh)
            {
              similarCheck = true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;
            if (mvDiff.getAbsHor() < mvdTh && mvDiff.getAbsVer() < mvdTh)
            {
              similarCheck = true;
            }
          }
        }
        if (similarCheck)
        {
#if JVET_AI0183_MVP_EXTENSION
#if JVET_AL0160_SBSMVP
          MotionInfo* mi0 = colIdx[ui] >= 0 ? mrgCtx->subPuMvpMiBuf[colIdx[ui]].buf: mrgCtx->subSpatialPuMvpMiBuf[-1 - colIdx[ui]].buf;
          MotionInfo* mi1 = colIdx[mergeCandIndex] >= 0 ? mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]].buf: mrgCtx->subSpatialPuMvpMiBuf[-1 - colIdx[mergeCandIndex]].buf;
#else
          MotionInfo* mi0 = mrgCtx->subPuMvpMiBuf[colIdx[ui]].buf;
          MotionInfo* mi1 = mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]].buf;
#endif
          for (int h = 0; h < puSize.height && similarCheck; h += puHeight)
          {
            for (int w = 0; w < puSize.width && similarCheck; w += puWidth)
            {
              if (mi0->interDir != mi1->interDir)
#else
          Size      puSize = pu.lumaSize();
          int       numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
          int       numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
          int       puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
          int       puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
          MotionBuf mb0 = mrgCtx->subPuMvpMiBuf[colIdx[ui]];
          MotionBuf mb1 = mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]];
          for (int h = 0; h < puSize.height && similarCheck; h += puHeight)
          {
            for (int w = 0; w < puSize.width && similarCheck; w += puWidth)
            {
              MotionInfo mi0 = mb0.buf[(w >> ATMVP_SUB_BLOCK_SIZE) + (h >> ATMVP_SUB_BLOCK_SIZE) * (mrgCtx->subPuMvpMiBuf[0].stride)];
              MotionInfo mi1 = mb1.buf[(w >> ATMVP_SUB_BLOCK_SIZE) + (h >> ATMVP_SUB_BLOCK_SIZE) * (mrgCtx->subPuMvpMiBuf[0].stride)];
              char interDir0 = mi0.interDir;
              char interDir1 = mi1.interDir;
              if (interDir1 != interDir0)
#endif
              {
                similarCheck = false;
              }
              else
              {
#if JVET_AI0183_MVP_EXTENSION
                if (mi0->interDir == 1)
                {
                  if (mi1->refIdx[0] == mi0->refIdx[0])
                  {
                    Mv mvDiff = mi0->mv[0] - mi1->mv[0];
#else
                if (interDir1 == 1)
                {
                  if (mi1.refIdx[0] == mi0.refIdx[0])
                  {
                    Mv mvDiff = mi0.mv[0] - mi1.mv[0];
#endif
                    if (mvDiff.getAbsHor() >= mvdTh || mvDiff.getAbsVer() >= mvdTh)
                    {
                      similarCheck = false;
                    }
                  }
                  else
                  {
                    similarCheck = false;
                  }
                }
#if JVET_AI0183_MVP_EXTENSION
                else if (mi0->interDir == 2)
                {
                  if (mi1->refIdx[1] == mi0->refIdx[1])
                  {
                    Mv mvDiff = mi0->mv[1] - mi1->mv[1];
#else
                else if (interDir1 == 2)
                {
                  if (mi1.refIdx[1] == mi0.refIdx[1])
                  {
                    Mv mvDiff = mi0.mv[1] - mi1.mv[1];
#endif
                    if (mvDiff.getAbsHor() >= mvdTh || mvDiff.getAbsVer() >= mvdTh)
                    {
                      similarCheck = false;
                    }
                  }
                  else
                  {
                    similarCheck = false;
                  }
                }
#if JVET_AI0183_MVP_EXTENSION
                else
                {
                  if (mi1->refIdx[1] == mi0->refIdx[1] && mi1->refIdx[0] == mi0->refIdx[0])
                  {
                    Mv mvDiffL0 = mi0->mv[1] - mi1->mv[1];
                    Mv mvDiffL1 = mi0->mv[0] - mi1->mv[0];
                    if (mvDiffL0.getAbsHor() >= mvdTh || mvDiffL0.getAbsVer() >= mvdTh || mvDiffL1.getAbsHor() >= mvdTh || mvDiffL1.getAbsVer() >= mvdTh)
#else
                else if (interDir1 == 3)
                {
                  if (mi1.refIdx[1] == mi0.refIdx[1] && mi1.refIdx[0] == mi0.refIdx[0])
                  {
                    Mv mvDiffL0 = mi0.mv[1] - mi1.mv[1];
                    Mv mvDiffL1 = mi0.mv[0] - mi1.mv[0];
                    if (mvDiffL0.getAbsHor() >= mvdTh || mvDiffL0.getAbsVer() >= mvdTh || mvDiffL1.getAbsHor() >= mvdTh
                      || mvDiffL1.getAbsVer() >= mvdTh)
#endif
                    {
                      similarCheck = false;
                    }
                  }
                  else
                  {
                    similarCheck = false;
                  }
                }
              }
#if JVET_AI0183_MVP_EXTENSION
              mi0 += incrRow;
              mi1 += incrRow;
            }
            mi0 += incrColumn;
            mi1 += incrColumn;
#else
            }
#endif
          }
          if (similarCheck)
          {
            return true;
          }
        }
      }
    }
    return false;
  }
  else
  {
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
#if JVET_AL0160_SBSMVP
      if (colIdx[mergeCandIndex] < 0)
      {
        if (colIdx[mergeCandIndex] == (-1 - SUB_SMVP_TYPE_TR))
        {
          if (colIdx[ui] >= (-1 - SUB_SMVP_TYPE_TL))
          {
            continue;
          }
        }
        else
        {
          if (colIdx[ui] < 0)
          {
            continue;
          }
        }
      }
#endif
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        bool similarCheck = false;
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx
            && mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx
            && mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv
            && mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv)
          {
            similarCheck = true;
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx
            && mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv)
          {
            similarCheck = true;
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx
            && mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv)
          {
            similarCheck = true;
          }
        }
        if (similarCheck)
        {
#if JVET_AI0183_MVP_EXTENSION
#if JVET_AL0160_SBSMVP
          MotionInfo* mi0 = colIdx[ui] >= 0 ? mrgCtx->subPuMvpMiBuf[colIdx[ui]].buf : mrgCtx->subSpatialPuMvpMiBuf[-1 - colIdx[ui]].buf;
          MotionInfo* mi1 = colIdx[mergeCandIndex] >= 0 ? mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]].buf : mrgCtx->subSpatialPuMvpMiBuf[-1 - colIdx[mergeCandIndex]].buf;
#else
          MotionInfo* mi0 = mrgCtx->subPuMvpMiBuf[colIdx[ui]].buf;
          MotionInfo* mi1 = mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]].buf;
#endif
          for (int h = 0; h < puSize.height && similarCheck; h += puHeight)
          {
            for (int w = 0; w < puSize.width && similarCheck; w += puWidth)
            {
              if (mi0->interDir != mi1->interDir)
#else
          Size      puSize = pu.lumaSize();
          int       numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
          int       numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
          int       puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
          int       puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
          MotionBuf mb0 = mrgCtx->subPuMvpMiBuf[colIdx[ui]];
          MotionBuf mb1 = mrgCtx->subPuMvpMiBuf[colIdx[mergeCandIndex]];
          for (int h = 0; h < puSize.height && similarCheck; h += puHeight)
          {
            for (int w = 0; w < puSize.width && similarCheck; w += puWidth)
            {
              MotionInfo mi0 = mb0.buf[(w >> ATMVP_SUB_BLOCK_SIZE) + (h >> ATMVP_SUB_BLOCK_SIZE) * (mrgCtx->subPuMvpMiBuf[0].stride)];
              MotionInfo mi1 = mb1.buf[(w >> ATMVP_SUB_BLOCK_SIZE) + (h >> ATMVP_SUB_BLOCK_SIZE) * (mrgCtx->subPuMvpMiBuf[0].stride)];
              char interDir0 = mi0.interDir;
              char interDir1 = mi1.interDir;
              if (interDir1 != interDir0)
#endif
              {
                similarCheck = false;
              }
              else
              {
#if JVET_AI0183_MVP_EXTENSION
                if (mi0->interDir == 1)
                {
                  if (mi1->refIdx[0] != mi0->refIdx[0] || mi1->mv[0] != mi0->mv[0])
#else
                if (interDir1 == 1)
                {
                  if (mi1.refIdx[0] != mi0.refIdx[0] || mi1.mv[0] != mi0.mv[0])
#endif
                  {
                    similarCheck = false;
                  }
                }
#if JVET_AI0183_MVP_EXTENSION
                else if (mi0->interDir == 2)
                {
                  if (mi1->refIdx[1] != mi0->refIdx[1] || mi1->mv[1] != mi0->mv[1])
#else
                else if (interDir1 == 2)
                {
                  if (mi1.refIdx[1] != mi0.refIdx[1] || mi1.mv[1] != mi0.mv[1])
#endif
                  {
                    similarCheck = false;
                  }
                }
#if JVET_AI0183_MVP_EXTENSION
                else
                {
                  if (mi1->refIdx[1] != mi0->refIdx[1] || mi1->mv[1] != mi0->mv[1] || mi1->refIdx[0] != mi0->refIdx[0] || mi1->mv[0] != mi0->mv[0])
#else
                else if (interDir1 == 3)
                {
                  if (mi1.refIdx[1] != mi0.refIdx[1] || mi1.mv[1] != mi0.mv[1] || mi1.refIdx[0] != mi0.refIdx[0]
                    || mi1.mv[0] != mi0.mv[0])
#endif
                  {
                    similarCheck = false;
                  }
                }
              }
#if JVET_AI0183_MVP_EXTENSION
              mi0 += incrRow;
              mi1 += incrRow;
            }
            mi0 += incrColumn;
            mi1 += incrColumn;
#else
            }
#endif
          }
          if (similarCheck)
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}
#endif
bool MergeCtx::xCheckSimilarMotionSubTMVP(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (interDirNeighbours[mergeCandIndex] == 0)
  {
    return true;
  }
  Mv cVector;

  CHECK(interDirNeighbours[mergeCandIndex] != 1 && interDirNeighbours[mergeCandIndex] != 2, "Wrong interDir.");

  cVector = (interDirNeighbours[mergeCandIndex] == 1) ? mvFieldNeighbours[(mergeCandIndex << 1)].mv : mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
  cVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    Mv cTempVector = (interDirNeighbours[ui] == 1) ? mvFieldNeighbours[(ui << 1)].mv : mvFieldNeighbours[(ui << 1) + 1].mv;
    cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
    Mv mvDiff = cTempVector - cVector;
    if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
    {
      return true;
    }
  }
  return false;
}
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM || (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM) || JVET_Y0058_IBC_LIST_MODIFY || JVET_AE0046_BI_GPM
#if JVET_Z0075_IBC_HMVP_ENLARGE
bool MergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh, int compareNum) const
#else
bool MergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
#endif
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx < 0)
  {
    return true;
  }

#if JVET_Z0075_IBC_HMVP_ENLARGE
  if (compareNum == -1)
  {
    compareNum = mergeCandIndex;
  }
#endif
  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      if(licInheritPara[ui] == licInheritPara[mergeCandIndex])
#endif
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)    ].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)    ].refIdx &&
              mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx )
          {
            Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)    ].mv - mvFieldNeighbours[(mergeCandIndex << 1)    ].mv;
            Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;

            if (mvDiffL0.getAbsHor() < mvdSimilarityThresh && mvDiffL0.getAbsVer() < mvdSimilarityThresh
             && mvDiffL1.getAbsHor() < mvdSimilarityThresh && mvDiffL1.getAbsVer() < mvdSimilarityThresh)
            {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
              if (licInheritPara[ui])
              {
                for (int comp = 0; comp < 1; comp++)
                {
                  if (licScale[ui][0][comp] != licScale[mergeCandIndex][0][comp]) return false;
                  if (licOffset[ui][0][comp] != licOffset[mergeCandIndex][0][comp]) return false;
                  if (licScale[ui][1][comp] != licScale[mergeCandIndex][1][comp]) return false;
                  if (licOffset[ui][1][comp] != licOffset[mergeCandIndex][1][comp]) return false;
                }
              }
#endif
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)].refIdx )
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1)].mv - mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
              if (licInheritPara[ui])
              {
                for (int comp = 0; comp < 1; comp++)
                {
                  if (licScale[ui][0][comp] != licScale[mergeCandIndex][0][comp]) return false;
                  if (licOffset[ui][0][comp] != licOffset[mergeCandIndex][0][comp]) return false;
                }
              }
#endif
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx )
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
              if (licInheritPara[ui])
              {
                for (int comp = 0; comp < 1; comp++)
                {
                  if (licScale[ui][1][comp] != licScale[mergeCandIndex][1][comp]) return false;
                  if (licOffset[ui][1][comp] != licOffset[mergeCandIndex][1][comp]) return false;
                }
              }
#endif
              return true;
            }
          }
        }
      }
    }

    return false;
  }

#if JVET_Z0075_IBC_HMVP_ENLARGE
  for (uint32_t ui = 0; ui < compareNum; ui++)
#else
  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
#endif
  {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    if(licInheritPara[ui] == licInheritPara[mergeCandIndex])
#endif
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)    ].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)    ].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
            mvFieldNeighbours[(ui << 1)    ].mv     == mvFieldNeighbours[(mergeCandIndex << 1)    ].mv     &&
            mvFieldNeighbours[(ui << 1) + 1].mv     == mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
          if (licInheritPara[ui])
          {
            for (int comp = 0; comp < 1; comp++)
            {
              if (licScale[ui][0][comp] != licScale[mergeCandIndex][0][comp]) return false;
              if (licOffset[ui][0][comp] != licOffset[mergeCandIndex][0][comp]) return false;
              if (licScale[ui][1][comp] != licScale[mergeCandIndex][1][comp]) return false;
              if (licOffset[ui][1][comp] != licOffset[mergeCandIndex][1][comp]) return false;
            }
          }
#endif
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
            mvFieldNeighbours[(ui << 1)].mv     == mvFieldNeighbours[(mergeCandIndex << 1)].mv)
        {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
          if (licInheritPara[ui])
          {
            for (int comp = 0; comp < 1; comp++)
            {
              if (licScale[ui][0][comp] != licScale[mergeCandIndex][0][comp]) return false;
              if (licOffset[ui][0][comp] != licOffset[mergeCandIndex][0][comp]) return false;
            }
          }
#endif
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].mv     == mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
          if (licInheritPara[ui])
          {
            for (int comp = 0; comp < 1; comp++)
            {
              if (licScale[ui][1][comp] != licScale[mergeCandIndex][1][comp]) return false;
              if (licOffset[ui][1][comp] != licOffset[mergeCandIndex][1][comp]) return false;
            }
          }
#endif
          return true;
        }
      }
    }
  }

  return false;
}
#endif
#if JVET_AD0213_LIC_IMP
void MergeCtx::initMrgCand(int cnt)
{
#if JVET_AM0106_CHAIN_AMVP_MERGE
  posNeighbours[cnt] = Position(0, 0); // position from which candidate is derived - apply this only for spatial candidates
#endif
  bcwIdx[cnt] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
  altLMFlag[cnt] = false;
  altLMParaNeighbours[cnt].resetAltLinearModel();
#endif
#if INTER_LIC
  licFlags[cnt] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  setDefaultLICParamToCtx(cnt);
#endif
#endif
#if JVET_AC0112_IBC_LIC
  ibcLicFlags[cnt] = false;
#if JVET_AE0078_IBC_LIC_EXTENSION
  ibcLicIndex[cnt] = 0;
#endif
#endif
#if JVET_AE0159_FIBC
  ibcFilterFlags[cnt] = false;
#endif
#if JVET_AA0070_RRIBC
  rribcFlipTypes[cnt] = 0;
#endif
  interDirNeighbours[cnt] = 0;
  mvFieldNeighbours[(cnt << 1)].refIdx = NOT_VALID;
  mvFieldNeighbours[(cnt << 1) + 1].refIdx = NOT_VALID;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  mvFieldNeighbours[(cnt << 1) + 0].mv.setZero();
  mvFieldNeighbours[(cnt << 1) + 1].mv.setZero();
#endif
  useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
  addHypNeighbours[cnt].clear();
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  candCost[cnt] = MAX_UINT64;
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
  candtype[cnt] = -1;
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  refineTmvpParams.refineTmvpRefList[(cnt << 1)] = REF_PIC_LIST_X;
  refineTmvpParams.refineTmvpRefList[(cnt << 1) + 1] = REF_PIC_LIST_X;
#endif
}
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
bool MergeCtx::xCheckSimilarMotion2Lists(int mergeCandIndex, MergeCtx *mrgCtx, uint32_t mvdSimilarityThresh ) const
{
  if (mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx < 0 && mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < numValidMergeCand; ui++)
    {
      if (interDirNeighbours[ui] == mrgCtx->interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx)
          {
            Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;

            if (mvDiffL0.getAbsHor() < mvdSimilarityThresh && mvDiffL0.getAbsVer() < mvdSimilarityThresh
              && mvDiffL1.getAbsHor() < mvdSimilarityThresh && mvDiffL1.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1)].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }
  for (uint32_t ui = 0; ui < numValidMergeCand; ui++)
  {
    if (interDirNeighbours[ui] == mrgCtx->interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
          mvFieldNeighbours[(ui << 1)].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv &&
          mvFieldNeighbours[(ui << 1) + 1].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
          mvFieldNeighbours[(ui << 1)].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
    }
  }
  return false;
}
#endif
#if JVET_Z0084_IBC_TM
#if JVET_Z0075_IBC_HMVP_ENLARGE
bool MergeCtx::xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh, int compareNum) const
#else
bool MergeCtx::xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
#endif
{
  if (mvFieldNeighbours[mergeCandIndex << 1].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      Mv mvDiff = mvFieldNeighbours[ui << 1].mv - mvFieldNeighbours[mergeCandIndex << 1].mv;
      if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
      {
        return true;
      }
    }
  }
  else
  {
#if JVET_Z0075_IBC_HMVP_ENLARGE
    if (compareNum == -1)
    {
      compareNum = mergeCandIndex;
    }

    for (uint32_t ui = 0; ui < compareNum; ui++)
#else
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
#endif
    {
      if (mvFieldNeighbours[ui << 1].mv == mvFieldNeighbours[mergeCandIndex << 1].mv)
      {
        return true;
      }
    }
  }

  return false;
}
#endif
#if JVET_W0097_GPM_MMVD_TM
void MergeCtx::setGeoMmvdMergeInfo(PredictionUnit& pu, int mergeIdx, int mmvdIdx)
{
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  CHECK(mergeIdx >= numValidMergeCand, "Merge candidate does not exist");
  CHECK(mmvdIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "GPM MMVD index is invalid");
  CHECK(!pu.cu->geoFlag || CU::isIBC(*pu.cu), "incorrect GPM setting")

    pu.regularMergeFlag = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag = true;
  pu.mmvdMergeFlag = false;
#if JVET_AE0046_BI_GPM
  int desiredDir = interDirNeighbours[mergeIdx];

  if (interDirNeighbours[mergeIdx] == 3)
  {
    if (!pu.cs->slice->getCheckLDC())
    {
      desiredDir = (mergeIdx % 2) + 1;
    }
  }
  pu.interDir = desiredDir;
#else
  pu.interDir = interDirNeighbours[mergeIdx];
#endif
  pu.cu->imv = 0;
  pu.mergeIdx = mergeIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif

  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  const int refExtMvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  int fPosStep = (extMMVD ? (mmvdIdx >> 3) : (mmvdIdx >> 2));
  int fPosPosition = (extMMVD ? (mmvdIdx - (fPosStep << 3)) : (mmvdIdx - (fPosStep << 2)));
  int offset = (extMMVD ? refExtMvdCands[fPosStep] : refMvdCands[fPosStep]);
  Mv  mvOffset;

  if (fPosPosition == 0)
  {
    mvOffset = Mv(offset, 0);
  }
  else if (fPosPosition == 1)
  {
    mvOffset = Mv(-offset, 0);
  }
  else if (fPosPosition == 2)
  {
    mvOffset = Mv(0, offset);
  }
  else if (fPosPosition == 3)
  {
    mvOffset = Mv(0, -offset);
  }
  else if (fPosPosition == 4)
  {
    mvOffset = Mv(offset, offset);
  }
  else if (fPosPosition == 5)
  {
    mvOffset = Mv(offset, -offset);
  }
  else if (fPosPosition == 6)
  {
    mvOffset = Mv(-offset, offset);
  }
  else if (fPosPosition == 7)
  {
    mvOffset = Mv(-offset, -offset);
  }

#if JVET_AE0046_BI_GPM
  if (desiredDir == 3)
  {
    pu.refIdx[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].refIdx;
    pu.refIdx[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].refIdx;
  }
  else
  {
    int listTarget = desiredDir - 1;
    int listEmpty = 1 - listTarget;

    pu.refIdx[ RefPicList(listTarget) ] = mvFieldNeighbours[(mergeIdx << 1) + listTarget].refIdx;
    pu.refIdx[ RefPicList(listEmpty) ]  = -1;
  }

#else
  pu.refIdx[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].refIdx;
  pu.refIdx[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].refIdx;
#endif

#if JVET_AE0046_BI_GPM
  if (pu.refIdx[REF_PIC_LIST_0] >= 0 && pu.refIdx[REF_PIC_LIST_1] >= 0)
  {
    Mv tempMv[2];

    const int refListIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refListIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const int poc0 = pu.cs->slice->getRefPOC(REF_PIC_LIST_0, refListIdx0);
    const int poc1 = pu.cs->slice->getRefPOC(REF_PIC_LIST_1, refListIdx1);
    const int currPoc = pu.cs->slice->getPOC();

    tempMv[0] = mvOffset;

    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1] = tempMv[0];

      const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
      const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;

      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      {
        tempMv[0] = tempMv[1].scaleMv(scale);
      }
    }
    else
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
      const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
        }
      }
      else
      {
        tempMv[1] = tempMv[0].scaleMv(scale);
      }
    }

    pu.mv[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].mv + tempMv[0];
    pu.mv[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].mv + tempMv[1];
  }
  else
  {
#endif
    if (pu.refIdx[REF_PIC_LIST_0] >= 0)
    {
      pu.mv[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].mv + mvOffset;
    }
    else
    {
      pu.mv[REF_PIC_LIST_0] = Mv();
    }

    if (pu.refIdx[REF_PIC_LIST_1] >= 0)
    {
      pu.mv[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].mv + mvOffset;
    }
    else
    {
      pu.mv[REF_PIC_LIST_1] = Mv();
    }
#if JVET_AE0046_BI_GPM
  }
#endif
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->bcwIdx = (interDirNeighbours[mergeIdx] == 3) ? bcwIdx[mergeIdx] : BCW_DEFAULT;

#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif

#if !INTER_RM_SIZE_CONSTRAINTS
  PU::restrictBiPredMergeCandsOne(pu);
#endif
  pu.mmvdEncOptMode = 0;

#if JVET_AG0276_NLIC
  pu.cu->altLMFlag = altLMFlag[mergeIdx];
  pu.cu->altLMParaUnit = altLMParaNeighbours[mergeIdx];

#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool initializeAltLM = !pu.cu->altLMFlag
                      && !pu.cu->geoFlag;
  if (initializeAltLM)
  {
    pu.cu->altLMParaUnit.resetAltLinearModel();
  }
#endif
#endif
#if INTER_LIC
  pu.cu->licFlag = pu.cs->slice->getUseLIC() ? licFlags[mergeIdx] : false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  setLICParamToPu(pu, mergeIdx, licInheritPara[mergeIdx]);
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  pu.cu->licDelta = 0;
#endif
#if !JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  if (pu.interDir == 3)
  {
    CHECK(pu.cu->licFlag, "LIC is not used with bi-prediction in merge");
  }
#endif
#endif
}
void MergeCtx::copyMergeCtx(MergeCtx & orgMergeCtx)
{
  memcpy(interDirNeighbours, orgMergeCtx.interDirNeighbours, MRG_MAX_NUM_CANDS * sizeof(unsigned char));
  memcpy(mvFieldNeighbours, orgMergeCtx.mvFieldNeighbours, (MRG_MAX_NUM_CANDS << 1) * sizeof(MvField));
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  for (int i = 0; i < orgMergeCtx.numValidMergeCand; ++i)
  {
    copyLICParamFromCtx(i, orgMergeCtx, i);
#if JVET_AG0276_NLIC
    licFlags[i] = orgMergeCtx.licFlags[i];
    altLMParaNeighbours[i] = orgMergeCtx.altLMParaNeighbours[i];
#endif
  }
#endif
}
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped)
#else
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx)
#endif
{
  const Slice &slice = *pu.cs->slice;
  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED && !JVET_AA0132_CONFIGURABLE_TM_TOOLS
  const int refMvdCands[] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift ,  32 << mvShift };
#else
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
#endif
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv[2];

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if(candIdxMaped == -1)
  {
    candIdxMaped = candIdx;
  }
  tempIdx = candIdxMaped;
#else
  tempIdx = candIdx;
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  fPosGroup = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  fPosStep = tempIdx / MMVD_MAX_DIR;
  fPosPosition = tempIdx - fPosStep * MMVD_MAX_DIR;
#else
  fPosStep = tempIdx / 4;
  fPosPosition = tempIdx - fPosStep * (4);
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  }
  else
  {
    fPosGroup = tempIdx / (VVC_MMVD_BASE_MV_NUM * VVC_MMVD_MAX_REFINE_NUM);
    tempIdx = tempIdx - fPosGroup * (VVC_MMVD_BASE_MV_NUM * VVC_MMVD_MAX_REFINE_NUM);
    fPosBaseIdx = tempIdx / VVC_MMVD_MAX_REFINE_NUM;
    tempIdx = tempIdx - fPosBaseIdx * (VVC_MMVD_MAX_REFINE_NUM);
    fPosStep = tempIdx / VVC_MMVD_MAX_DIR;
    fPosPosition = tempIdx - fPosStep * VVC_MMVD_MAX_DIR;
  }
#endif
  int offset = refMvdCands[fPosStep];
  if ( pu.cu->slice->getPicHeader()->getDisFracMMVD() )
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
                    //0   1   2   3   4   5   6   7  8   9  10  11 12  13  14  15
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2, -2,  2, 1, -1, -1,  1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1, -1,  1, -1, 2, -2,  2, -2};
#else
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2,  2, -2, 1,  1, -1, -1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1,  1, -1, -1, 2, -2,  2, -2};
#endif
#endif
   
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  if ((refList0 != -1) && (refList1 != -1)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    && pu.cs->sps->getUseTMMMVD()
#endif
    )
  {
    tempMv[0] = Mv(0,0);
    tempMv[1] = Mv(0,0);
    int cutOff1 = 2 * MMVD_MAX_DIR_UNI;
    int cutOff2 = MMVD_MAX_DIR_UNI;
    if (fPosPosition >= cutOff1)
    {
      fPosPosition -= cutOff1;
      const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
      const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
      const int currPoc = slice.getPOC();
      tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
      if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
      {
        tempMv[1] = tempMv[0];
      }
      else
      {
        tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
      }
    }
    else if (fPosPosition >= cutOff2)
    {
      fPosPosition -= cutOff2;
      tempMv[1] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
    }
    else
    {
      tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
    }
    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  else
#endif
#endif
#if !JVET_AA0093_ENHANCED_MMVD_EXTENSION || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_ENHANCED_MMVD_EXTENSION)
  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
#endif
    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1] = tempMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      tempMv[0] = tempMv[1].scaleMv(scale);
    }
    else
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
        }
      }
      else
      tempMv[1] = tempMv[0].scaleMv(scale);
    }

    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }
#endif
  else if (refList0 != -1)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
#endif
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
  }
  else if (refList1 != -1)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[1] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[1] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[1] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[1] = Mv(0, offset);
    }
    else
    {
      tempMv[1] = Mv(0, -offset);
    }
#endif
    pu.interDir = 2;
    pu.mv[REF_PIC_LIST_0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0] = -1;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }

  pu.mmvdMergeFlag = true;
  pu.mmvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.regularMergeFlag = true;
  pu.mergeIdx = candIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine             = false;
#endif
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->imv = mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0;
#if JVET_AG0276_NLIC
  pu.cu->altLMFlag = altLMFlag[fPosBaseIdx];
  pu.cu->altLMParaUnit = altLMParaNeighbours[fPosBaseIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool initializeAltLM = !pu.cu->altLMFlag
                      && !pu.cu->geoFlag;
  if (initializeAltLM)
  {
    pu.cu->altLMParaUnit.resetAltLinearModel();
  }
#endif
#endif
#if INTER_LIC
  pu.cu->licFlag = licFlags[fPosBaseIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  setLICParamToPu(pu, fPosBaseIdx, licInheritPara[fPosBaseIdx]);
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  pu.cu->licDelta = 0;
#endif
#endif
  pu.cu->bcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? bcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }

#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif

  PU::restrictBiPredMergeCandsOne(pu);
}

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
void MergeCtx::setGeoMrgDuplicate( const PredictionUnit& pu )
{
  CHECK(GEO_MAX_NUM_UNI_CANDS > NUM_MERGE_CANDS, "setGeoMrgDuplicate() failed");
  uint8_t maxNumMergeCandidates = pu.cs->sps->getMaxNumGeoCand();
  Slice* slice = pu.cs->slice;

  Mv   mrgMv[GEO_MAX_NUM_UNI_CANDS];
  maxNumMergeCandidates = std::min((int)maxNumMergeCandidates, numValidMergeCand);
  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    int mrgList        = mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    int mrgRefIdx      = mvFieldNeighbours[(mergeCand << 1) + mrgList].refIdx;
#if JVET_AJ0274_REGRESSION_GPM_TM
    if (mrgRefIdx < 0 || mrgRefIdx > MAX_NUM_REF)
    {
      mrgDuplicated[mergeCand] = true;
      pocMrg[mergeCand] = -1;
      continue;
    }
#endif
    pocMrg[mergeCand]  = slice->getRefPic((RefPicList)mrgList, mrgRefIdx)->getPOC();
    mrgMv[mergeCand]   = mvFieldNeighbours[(mergeCand << 1) + mrgList].mv;
    mrgDuplicated[mergeCand] = false;
    if (mergeCand)
    {
      for (int i = 0; i < mergeCand; i++)
      {
        if (pocMrg[mergeCand] == pocMrg[i] && mrgMv[mergeCand] == mrgMv[i])
        {
          mrgDuplicated[mergeCand] = true;
          break;
        }
      }
    }
  }
}
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
void MergeCtx::copyRefineTmvpCtxModellingParams(MergeCtx &orgMergeCtx, uint32_t srcMrgIdxL0, uint32_t srcMrgIdxL1, uint32_t dstMrgIdxL0, uint32_t dstMrgIdxL1)
{
  refineTmvpParams.refineTmvpRefList    [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpRefList    [srcMrgIdxL0];
  refineTmvpParams.refineTmvpRefList    [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpRefList    [srcMrgIdxL1];
  if (orgMergeCtx.refineTmvpParams.refineTmvpRefList[srcMrgIdxL0] == REF_PIC_LIST_0)
  {
    refineTmvpParams.refineTmvpCol          [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpCol          [srcMrgIdxL0];
    refineTmvpParams.refineTmvpRefIdx       [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpRefIdx       [srcMrgIdxL0];
    refineTmvpParams.refineTmvpTargetRefIdx [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpTargetRefIdx [srcMrgIdxL0];
    refineTmvpParams.refineTmvpSbFlag       [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpSbFlag       [srcMrgIdxL0];
    refineTmvpParams.refineTmvpPosition     [dstMrgIdxL0][0] = orgMergeCtx.refineTmvpParams.refineTmvpPosition     [srcMrgIdxL0][0];
    refineTmvpParams.refineTmvpPosition     [dstMrgIdxL0][1] = orgMergeCtx.refineTmvpParams.refineTmvpPosition     [srcMrgIdxL0][1];
    refineTmvpParams.refineTmvpColRefPicList[dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpColRefPicList[srcMrgIdxL0];
    refineTmvpParams.refineTmvpColRefPOC    [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpColRefPOC    [srcMrgIdxL0];
    refineTmvpParams.refineTmvpMiPtr        [dstMrgIdxL0]    = orgMergeCtx.refineTmvpParams.refineTmvpMiPtr        [srcMrgIdxL0];
  }
  else
  {
    CHECK(orgMergeCtx.refineTmvpParams.refineTmvpRefList[srcMrgIdxL0] != REF_PIC_LIST_X, "this is not possible");
  }
  if (orgMergeCtx.refineTmvpParams.refineTmvpRefList[srcMrgIdxL1] == REF_PIC_LIST_1)
  {
    refineTmvpParams.refineTmvpCol          [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpCol          [srcMrgIdxL1];
    refineTmvpParams.refineTmvpRefIdx       [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpRefIdx       [srcMrgIdxL1];
    refineTmvpParams.refineTmvpTargetRefIdx [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpTargetRefIdx [srcMrgIdxL1];
    refineTmvpParams.refineTmvpSbFlag       [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpSbFlag       [srcMrgIdxL1];
    refineTmvpParams.refineTmvpPosition     [dstMrgIdxL1][0] = orgMergeCtx.refineTmvpParams.refineTmvpPosition     [srcMrgIdxL1][0];
    refineTmvpParams.refineTmvpPosition     [dstMrgIdxL1][1] = orgMergeCtx.refineTmvpParams.refineTmvpPosition     [srcMrgIdxL1][1];
    refineTmvpParams.refineTmvpColRefPicList[dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpColRefPicList[srcMrgIdxL1];
    refineTmvpParams.refineTmvpColRefPOC    [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpColRefPOC    [srcMrgIdxL1];
    refineTmvpParams.refineTmvpMiPtr        [dstMrgIdxL1]    = orgMergeCtx.refineTmvpParams.refineTmvpMiPtr        [srcMrgIdxL1];
  }
  else
  {
    CHECK(orgMergeCtx.refineTmvpParams.refineTmvpRefList[srcMrgIdxL1] != REF_PIC_LIST_X, "this is not possible");
  }
}
#endif
#if JVET_AB0112_AFFINE_DMVR
bool AffineMergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    int mvdTh = mvdSimilarityThresh;
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
              mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0L0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff0L1 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;

            Mv mvDiff1L0 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff1L1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;

            Mv mvDiff2L0 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            Mv mvDiff2L1 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0L0.getAbsHor() < mvdTh && mvDiff0L0.getAbsVer() < mvdTh
              && mvDiff0L1.getAbsHor() < mvdTh && mvDiff0L1.getAbsVer() < mvdTh
              &&mvDiff1L0.getAbsHor() < mvdTh && mvDiff1L0.getAbsVer() < mvdTh
              && mvDiff1L1.getAbsHor() < mvdTh && mvDiff1L1.getAbsVer() < mvdTh
              &&mvDiff2L0.getAbsHor() < mvdTh && mvDiff2L0.getAbsVer() < mvdTh
              && mvDiff2L1.getAbsHor() < mvdTh && mvDiff2L1.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
              &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
              &&mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
              &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
              && mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
          mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv&&
          mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv&&
          mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
          )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
          mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv&&
          mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv&&
          mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv
          )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv &&
          mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv &&
          mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
          )
        {
          return true;
        }
      }
    }
  }
  return false;
}
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
void AffineMergeCtx::setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC)                               { MergeCtx::setLICParamToPu         (*this, pu, candIdx, hasLIC);                }
void AffineMergeCtx::setLICParamToPu         (      PredictionUnit& pu, int candIdx, bool hasLIC)                         const { MergeCtx::setLICParamToPu         (*this, pu, candIdx, hasLIC);                }
void AffineMergeCtx::loadLICParamFromPu      (const PredictionUnit* pu, int candIdx, bool allowAltModel, bool hasLIC)           { MergeCtx::loadLICParamFromPu      (*this, pu, candIdx, allowAltModel, hasLIC); }
void AffineMergeCtx::loadLICParamFromMotInfo (const MotionInfo*     mi, int candIdx, bool allowAltModel, bool hasLIC)           { MergeCtx::loadLICParamFromMotInfo (*this, mi, candIdx, allowAltModel, hasLIC); }
void AffineMergeCtx::copyLICParamFromCtx     (int candIdx, const AffineMergeCtx& src, int candIdxSrc)                           { MergeCtx::copyLICParamFromCtx     (*this, candIdx, src, candIdxSrc);           }
void AffineMergeCtx::setDefaultLICParamToCtx (int candIdx)                                                                      { MergeCtx::setDefaultLICParamToCtx (*this, candIdx);                            }
void AffineMergeCtx::setInheritAndLICFlags   (int candIdx)                                                                      { MergeCtx::setInheritAndLICFlags   (*this, candIdx);                            }
#if JVET_AG0276_NLIC
void AffineMergeCtx::setLICParamUsingAltLM   (int candIdx)                                                                      { MergeCtx::setLICParamUsingAltLM   (*this, candIdx);                            }
#endif
#endif

#if JVET_AG0276_NLIC
bool AffineMergeCtx::xCheckSimilarMotion1(int mergeCandIndex, uint32_t mvdSimilarityThresh, bool isAlt) const
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    int mvdTh = mvdSimilarityThresh;
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      if (!isAlt && altLMFlag[ui])
      {
        continue;
      }
      if (isAlt && !altLMFlag[ui])
      {
        continue;
      }
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
              mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0L0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff0L1 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;

            Mv mvDiff1L0 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff1L1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;

            Mv mvDiff2L0 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            Mv mvDiff2L1 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0L0.getAbsHor() < mvdTh && mvDiff0L0.getAbsVer() < mvdTh
                && mvDiff0L1.getAbsHor() < mvdTh && mvDiff0L1.getAbsVer() < mvdTh
                &&mvDiff1L0.getAbsHor() < mvdTh && mvDiff1L0.getAbsVer() < mvdTh
                && mvDiff1L1.getAbsHor() < mvdTh && mvDiff1L1.getAbsVer() < mvdTh
                &&mvDiff2L0.getAbsHor() < mvdTh && mvDiff2L0.getAbsVer() < mvdTh
                && mvDiff2L1.getAbsHor() < mvdTh && mvDiff2L1.getAbsVer() < mvdTh
                )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
                &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
                &&mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
                )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
                &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
                && mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
                )
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    if (!isAlt && altLMFlag[ui])
    {
      continue;
    }
    if (isAlt && !altLMFlag[ui])
    {
      continue;
    }
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
            mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv     &&
            mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv&&
            mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv     &&
            mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv&&
            mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv     &&
            mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
            )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
            mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv&&
            mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv&&
            mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv
            )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv &&
            mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv &&
            mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
            )
        {
          return true;
        }
      }
    }
  }
  return false;
}
#endif
#if JVET_AA0061_IBC_MBVD
#if JVET_AE0169_BIPREDICTIVE_IBC
bool MergeCtx::setIbcMbvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped, int candIdx1, int candIdxMaped1)
#else
bool MergeCtx::setIbcMbvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped)
#endif
{
  const int mvShift = MV_FRACTIONAL_BITS_DIFF + 2;
  const int refMvdCands[IBC_MBVD_STEP_NUM] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift , 24 << mvShift , 32 << mvShift , 40 << mvShift , 48 << mvShift , 56 << mvShift ,
    64 << mvShift , 72 << mvShift , 80 << mvShift , 88 << mvShift , 96 << mvShift , 104 << mvShift , 112 << mvShift , 120 << mvShift , 128 << mvShift };
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv;

  if(candIdxMaped == -1)
  {
    candIdxMaped = candIdx;
  }
  tempIdx = candIdxMaped;

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  int offset = 0;
  if (pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch())
  {
    fPosBaseIdx = tempIdx / IBC_MBVD_AD_MAX_REFINE_NUM;
    tempIdx = tempIdx - fPosBaseIdx * (IBC_MBVD_AD_MAX_REFINE_NUM);
    fPosStep = tempIdx / IBC_MBVD_OFFSET_DIR;
    fPosPosition = tempIdx - fPosStep * (IBC_MBVD_OFFSET_DIR);
    offset = g_ibcMbvdCandOffsets[fPosStep];
  }
  else
  {
#endif
  fPosGroup = tempIdx / (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / IBC_MBVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (IBC_MBVD_MAX_REFINE_NUM);
  fPosStep = tempIdx / IBC_MBVD_OFFSET_DIR;
  fPosPosition = tempIdx - fPosStep * (IBC_MBVD_OFFSET_DIR);
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  offset = refMvdCands[fPosStep];
  }
#else
  int offset = refMvdCands[fPosStep];
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (!pu.cu->slice->getPicHeader()->getDisFracMBVD() || !pu.cs->slice->getSPS()->getPLTMode())
  {
    offset >>= 2;
  }
#endif

  const int refList0 = ibcMbvdBaseBv[fPosBaseIdx][0].refIdx;
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2,  2, -2, 1,  1, -1, -1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1,  1, -1, -1, 2, -2,  2, -2};

  if (refList0 != -1)
  {
    tempMv = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#if JVET_AA0070_RRIBC
    //check the BVD direction and base BV flip direction
    if ((rribcFlipTypes[fPosBaseIdx] == 1) && (yDir[fPosPosition] != 0))
    {
      return true;
    }
    if ((rribcFlipTypes[fPosBaseIdx] == 2) && (xDir[fPosPosition] != 0))
    {
      return true;
    }
#endif
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = ibcMbvdBaseBv[fPosBaseIdx][0].mv + tempMv;
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
  }

  pu.ibcMbvdMergeFlag = true;
  pu.ibcMbvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.mergeType = MRG_TYPE_IBC;

#if JVET_AE0169_BIPREDICTIVE_IBC
  pu.ibcMergeIdx1 = MAX_INT;
  if(candIdxMaped1 == -1)
  {
    candIdxMaped1 = candIdx1;
  }
  if (candIdxMaped1 != -1)
  {
    if (candIdx1 < IBC_MRG_MAX_NUM_CANDS)
    {
      fPosBaseIdx = candIdx1;
      tempMv = Mv();
#if JVET_AA0070_RRIBC
      //check the BVD direction and base BV flip direction
      if (rribcFlipTypes[fPosBaseIdx] == 1)
      {
        return true;
      }
      if (rribcFlipTypes[fPosBaseIdx] == 2)
      {
        return true;
      }
#endif
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
      pu.mv[REF_PIC_LIST_1] = pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch() ? ibcMbvdBaseBvFrac[fPosBaseIdx][0].mv : ibcMbvdBaseBv[fPosBaseIdx][0].mv;
#endif
    }
    else
    {
      tempIdx = candIdxMaped1;

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
      int offset = 0;
      if (pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch())
      {
        fPosBaseIdx = tempIdx / IBC_MBVD_AD_MAX_REFINE_NUM;
        tempIdx = tempIdx - fPosBaseIdx * (IBC_MBVD_AD_MAX_REFINE_NUM);
        fPosStep = tempIdx / IBC_MBVD_OFFSET_DIR;
        fPosPosition = tempIdx - fPosStep * (IBC_MBVD_OFFSET_DIR);
        offset = g_ibcMbvdCandOffsets[fPosStep];
      }
      else
      {
#endif
      fPosGroup = tempIdx / (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
      tempIdx = tempIdx - fPosGroup * (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
      fPosBaseIdx = tempIdx / IBC_MBVD_MAX_REFINE_NUM;
      tempIdx = tempIdx - fPosBaseIdx * (IBC_MBVD_MAX_REFINE_NUM);
      fPosStep = tempIdx / IBC_MBVD_OFFSET_DIR;
      fPosPosition = tempIdx - fPosStep * (IBC_MBVD_OFFSET_DIR);
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
      offset = refMvdCands[fPosStep];
      }
#else
      int offset = refMvdCands[fPosStep];
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (!pu.cu->slice->getPicHeader()->getDisFracMBVD() || !pu.cs->slice->getSPS()->getPLTMode())
      {
        offset >>= 2;
      }
#endif

      tempMv = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#if JVET_AA0070_RRIBC
      //check the BVD direction and base BV flip direction
      if ((rribcFlipTypes[fPosBaseIdx] == 1) && (yDir[fPosPosition] != 0))
      {
        return true;
      }
      if ((rribcFlipTypes[fPosBaseIdx] == 2) && (xDir[fPosPosition] != 0))
      {
        return true;
      }
#endif
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
      pu.mv[REF_PIC_LIST_1] = ibcMbvdBaseBv[fPosBaseIdx][0].mv + tempMv;
#endif
    }
    pu.interDir = 3;
#if !JVET_AE0169_IBC_MBVD_LIST_DERIVATION
    pu.mv[REF_PIC_LIST_1] = ibcMbvdBaseBv[fPosBaseIdx][0].mv + tempMv;
#endif
    pu.refIdx[REF_PIC_LIST_1] = MAX_NUM_REF;
    pu.ibcMergeIdx1 = candIdx1;
  }
#endif

  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

#if JVET_AE0169_BIPREDICTIVE_IBC
  pu.cu->bcwIdx = BCW_DEFAULT;
#else
  pu.cu->bcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? bcwIdx[fPosBaseIdx] : BCW_DEFAULT;
#endif

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }
  pu.bv = pu.mv[REF_PIC_LIST_0];
  pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pu.cs->sps->getIBCFracFlag())
  {
    pu.cu->imv = (!pu.cu->geoFlag && useAltHpelIf[fPosBaseIdx]) ? IMV_HPEL : 0;
  }
  else
#endif
  pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AC0112_IBC_LIC
  pu.cu->ibcLicFlag = (pu.interDir == 3) ? false : ibcLicFlags[fPosBaseIdx];
#if JVET_AE0078_IBC_LIC_EXTENSION
  pu.cu->ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
  pu.cu->ibcFilterFlag = (pu.interDir == 3) ? false : ibcFilterFlags[fPosBaseIdx];
#endif
#if JVET_AA0070_RRIBC
  pu.cu->rribcFlipType = (pu.interDir == 3) ? 0 : rribcFlipTypes[fPosBaseIdx];
#endif
#else
#if JVET_AC0112_IBC_LIC
  pu.cu->ibcLicFlag = ibcLicFlags[fPosBaseIdx];
#if JVET_AE0078_IBC_LIC_EXTENSION
  pu.cu->ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
  pu.cu->ibcFilterFlag = ibcFilterFlags[fPosBaseIdx];
#endif
#if JVET_AA0070_RRIBC
  pu.cu->rribcFlipType = rribcFlipTypes[fPosBaseIdx];
#endif
#endif
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif
  return false;
}
#endif

#if JVET_AG0276_NLIC
void AltLMMergeCtx::initAltLMMergeCtx(int idx)
{
  altLMParaNeighbours[idx].resetAltLinearModel();
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  initMrgCand(idx);
#else
  mvFieldNeighbours[(idx << 1) + 0].setMvField(Mv(), -1);
  mvFieldNeighbours[(idx << 1) + 1].setMvField(Mv(), -1);
  interDirNeighbours[idx] = 0;
  bcwIdx[idx] = BCW_DEFAULT;
  useAltHpelIf[idx] = false;
#endif
}

bool AltLMMergeCtx::xCheckSameMotion(int mrgCandIdx, uint32_t mvdSimilarityThresh)
{
  for (int ui = 0; ui < mrgCandIdx; ui++)
  {
    bool sameMotion = false;
    bool sameInterDir = (interDirNeighbours[ui] == interDirNeighbours[mrgCandIdx]);
    if (sameInterDir && interDirNeighbours[mrgCandIdx] == 3)
    {
      sameInterDir = (bcwIdx[ui] == bcwIdx[mrgCandIdx]);
    }

    if (sameInterDir)
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mrgCandIdx << 1)].refIdx     &&
          mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mrgCandIdx << 1) + 1].refIdx)
        {
          Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)].mv - mvFieldNeighbours[(mrgCandIdx << 1)].mv;
          Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mrgCandIdx << 1) + 1].mv;

          if (mvDiffL0.getAbsHor() < mvdSimilarityThresh && mvDiffL0.getAbsVer() < mvdSimilarityThresh
            && mvDiffL1.getAbsHor() < mvdSimilarityThresh && mvDiffL1.getAbsVer() < mvdSimilarityThresh)
          {
            sameMotion = true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mrgCandIdx << 1)].refIdx)
        {
          Mv mvDiff = mvFieldNeighbours[(ui << 1)].mv - mvFieldNeighbours[(mrgCandIdx << 1)].mv;
          if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
          {
            sameMotion = true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mrgCandIdx << 1) + 1].refIdx)
        {
          Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mrgCandIdx << 1) + 1].mv;
          if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
          {
            sameMotion = true;
          }
        }
      }
    }

    if (sameMotion)
    {
      return true;
    }
  }
  return false;
}
#endif

#if JVET_AG0276_NLIC
bool AltLMAffineMergeCtx::xCheckSameAffMotion(const PredictionUnit& pu, int cnt)
{
  if (cnt < 1)
  {
    return true;
  }

  const int CPMV_SIMILARITY_THREH = 1;
  const int PARA_SIMILARITY_THREH = 1;

  const int lastIdx = cnt;
  for (int idx = 0; idx < cnt; idx++)
  {
    if (affineType[idx] != affineType[lastIdx])
    {
      continue;
    }

    if (interDirNeighbours[idx] != interDirNeighbours[lastIdx])
    {
      continue;
    }

    if (interDirNeighbours[idx] == 3 && bcwIdx[idx] != bcwIdx[lastIdx])
    {
      continue;
    }

    if ((interDirNeighbours[lastIdx] & 1) != 0)
    {
      if (mvFieldNeighbours[(idx << 1)][0].refIdx != mvFieldNeighbours[(lastIdx << 1)][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = mvFieldNeighbours[(idx << 1)][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(idx << 1)][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(idx << 1)][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affinePara, affineType[idx]);
      acMvTemp[0] = mvFieldNeighbours[(lastIdx << 1)][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(lastIdx << 1)][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(lastIdx << 1)][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affineType[idx]);

      if (
        abs(mvFieldNeighbours[(idx << 1)][0].mv.getHor() - mvFieldNeighbours[(lastIdx << 1)][0].mv.getHor()) > CPMV_SIMILARITY_THREH ||
        abs(mvFieldNeighbours[(idx << 1)][0].mv.getVer() - mvFieldNeighbours[(lastIdx << 1)][0].mv.getVer()) > CPMV_SIMILARITY_THREH ||
        abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
        abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH
        )
      {
        continue;
      }


      if (affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
          abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH)
        {
          continue;
        }
      }
    }

    if ((interDirNeighbours[lastIdx] & 2) != 0)
    {
      if (mvFieldNeighbours[(idx << 1) + 1][0].refIdx != mvFieldNeighbours[(lastIdx << 1) + 1][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = mvFieldNeighbours[(idx << 1) + 1][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(idx << 1) + 1][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(idx << 1) + 1][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affinePara, affineType[idx]);
      acMvTemp[0] = mvFieldNeighbours[(lastIdx << 1) + 1][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(lastIdx << 1) + 1][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(lastIdx << 1) + 1][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affineType[idx]);

      if (
        abs(mvFieldNeighbours[(idx << 1) + 1][0].mv.getHor() - mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getHor()) > CPMV_SIMILARITY_THREH ||
        abs(mvFieldNeighbours[(idx << 1) + 1][0].mv.getVer() - mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getVer()) > CPMV_SIMILARITY_THREH ||
        abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
        abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH)
      {
        continue;
      }

      if (affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
          abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH)
        {
          continue;
        }
      }
    }
    return false;
  }
  return true;
}

void AltLMAffineMergeCtx::initAltLMAffMergeCtx(int idx)
{
  altLMParaNeighbours[idx].resetAltLinearModel();
  for (int mvNum = 0; mvNum < 3; mvNum++)
  {
    mvFieldNeighbours[(idx << 1) + 0][mvNum].setMvField(Mv(), -1);
    mvFieldNeighbours[(idx << 1) + 1][mvNum].setMvField(Mv(), -1);
  }
  interDirNeighbours[idx] = 0;
  affineType[idx] = AFFINEMODEL_4PARAM;
  bcwIdx[idx] = BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  obmcFlags[idx] = true;
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  m_isConstructed[idx] = false;
#endif
}

void AltLMAffineMergeCtx::init()
{
  for (int i = 0; i < ALT_AFF_MRG_MAX_NUM_CANDS; i++)
  {
    initAltLMAffMergeCtx(i);
  }
  numValidMergeCand = 0;
}

bool AltLMAffineMergeCtx::xCheckSameAffMotion(const PredictionUnit& pu, int cnt, AltLMAffineMergeCtx& altAffMrgCtx1)
{
  const int cpmvSimilarityThresh = 1;
  const int paraSimilarityThresh = 1;

  const int lastIdx = cnt;
  for (int idx = 0; idx < altAffMrgCtx1.numValidMergeCand; idx++)
  {
    if (altAffMrgCtx1.affineType[idx] != affineType[lastIdx])
    {
      continue;
    }

    if (altAffMrgCtx1.interDirNeighbours[idx] != interDirNeighbours[lastIdx])
    {
      continue;
    }

    if (altAffMrgCtx1.interDirNeighbours[idx] == 3 && altAffMrgCtx1.bcwIdx[idx] != bcwIdx[lastIdx])
    {
      continue;
    }

    if ((interDirNeighbours[lastIdx] & 1) != 0)
    {
      if (altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][0].refIdx != mvFieldNeighbours[(lastIdx << 1)][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][0].mv;
      acMvTemp[1] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][1].mv;
      acMvTemp[2] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affinePara, affineType[idx]);
      acMvTemp[0] = mvFieldNeighbours[(lastIdx << 1)][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(lastIdx << 1)][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(lastIdx << 1)][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affineType[idx]);

      if (abs(altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][0].mv.getHor() - mvFieldNeighbours[(lastIdx << 1)][0].mv.getHor()) > cpmvSimilarityThresh ||
        abs(altAffMrgCtx1.mvFieldNeighbours[(idx << 1)][0].mv.getVer() - mvFieldNeighbours[(lastIdx << 1)][0].mv.getVer()) > cpmvSimilarityThresh ||
        abs(affinePara[0] - affineParaLast[0]) > paraSimilarityThresh ||
        abs(affinePara[2] - affineParaLast[2]) > paraSimilarityThresh)
      {
        continue;
      }


      if (affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > paraSimilarityThresh ||
          abs(affinePara[3] - affineParaLast[3]) > paraSimilarityThresh)
        {
          continue;
        }
      }
    }

    if ((interDirNeighbours[lastIdx] & 2) != 0)
    {
      if (altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][0].refIdx != mvFieldNeighbours[(lastIdx << 1) + 1][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][0].mv;
      acMvTemp[1] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][1].mv;
      acMvTemp[2] = altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affinePara, affineType[idx]);
      acMvTemp[0] = mvFieldNeighbours[(lastIdx << 1) + 1][0].mv;
      acMvTemp[1] = mvFieldNeighbours[(lastIdx << 1) + 1][1].mv;
      acMvTemp[2] = mvFieldNeighbours[(lastIdx << 1) + 1][2].mv;
      PU::deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affineType[idx]);

      if (abs(altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][0].mv.getHor() - mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getHor()) > cpmvSimilarityThresh ||
        abs(altAffMrgCtx1.mvFieldNeighbours[(idx << 1) + 1][0].mv.getVer() - mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getVer()) > cpmvSimilarityThresh ||
        abs(affinePara[0] - affineParaLast[0]) > paraSimilarityThresh ||
        abs(affinePara[2] - affineParaLast[2]) > paraSimilarityThresh)
      {
        continue;
      }

      if (affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > paraSimilarityThresh ||
          abs(affinePara[3] - affineParaLast[3]) > paraSimilarityThresh)
        {
          continue;
        }
      }
    }
    return false;
  }
  return true;
}
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
unsigned DeriveCtx::CtxTmpFusionFlag(const CodingUnit& cu)
{
  const CodingStructure* cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit* cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = cuLeft && cuLeft->tmpFusionFlag ? 1 : 0;

  const CodingUnit* cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += cuAbove && cuAbove->tmpFusionFlag ? 1 : 0;

  return ctxId;
}
#endif

#if JVET_V0130_INTRA_TMP
unsigned DeriveCtx::CtxTmpFlag(const CodingUnit& cu)
{
	const CodingStructure* cs = cu.cs;
	unsigned ctxId = 0;

	const CodingUnit* cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
	ctxId = (cuLeft && cuLeft->tmpFlag) ? 1 : 0;

	const CodingUnit* cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
	ctxId += (cuAbove && cuAbove->tmpFlag) ? 1 : 0;

	ctxId = (cu.lwidth() > 2 * cu.lheight() || cu.lheight() > 2 * cu.lwidth()) ? 3 : ctxId;

	return ctxId;
}
#endif

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId  = (cu.lwidth() > 2*cu.lheight() || cu.lheight() > 2*cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}
#if JVET_AJ0249_NEURAL_NETWORK_BASED
uint16_t DeriveCtx::CtxPnnLuminanceFlag(const CodingUnit& cu)
{
  const CodingStructure* const cs = cu.cs;
  uint16_t ctxId = 0;
  const CodingUnit* const cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CHANNEL_TYPE_LUMA);
  if (cuLeft)
  {
    const uint32_t indexModeLeft = PU::getFinalIntraMode(*cuLeft->firstPU, CHANNEL_TYPE_LUMA);
    if (indexModeLeft == PNN_IDX)
    {
      return 1;
    }
  }
  const CodingUnit* const cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CHANNEL_TYPE_LUMA);
  if (cuAbove)
  {
    const uint32_t indexModeAbove = PU::getFinalIntraMode(*cuAbove->firstPU, CHANNEL_TYPE_LUMA);
    if (indexModeAbove == PNN_IDX)
    {
      ctxId = 1;
    }
  }
  return ctxId;
}
#endif

unsigned DeriveCtx::CtxPltCopyFlag( const unsigned prevRunType, const unsigned dist )
{
  uint8_t *ucCtxLut = (prevRunType == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  if ( dist <= RUN_IDX_THRE )
  {
     return ucCtxLut[dist];
  }
  else
  {
    return ucCtxLut[RUN_IDX_THRE];
  }
}

#if JVET_AE0159_FIBC 
unsigned DeriveCtx::ctxIbcFilterFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const Position pos = cu.chType == CHANNEL_TYPE_CHROMA ? cu.chromaPos() : cu.lumaPos();
  const CodingUnit *cuLeft = cs->getCURestricted(pos.offset(-1, 0), cu, cu.chType);
  ctxId += (cuLeft && !cuLeft->firstPU->mergeFlag && (cuLeft->ibcLicFlag || cuLeft->ibcFilterFlag)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(pos.offset(0, -1), cu, cu.chType);
  ctxId += (cuAbove && !cuAbove->firstPU->mergeFlag && (cuAbove->ibcLicFlag || cuAbove->ibcFilterFlag)) ? 1 : 0;
  return ctxId;
}
#endif

#if JVET_AG0164_AFFINE_GPM
void AffineMergeCtx::setAffMergeInfo(PredictionUnit &pu, int candIdx, int8_t mmvdIdx) const
{
  if (candIdx >= m_indexOffset)
  {
    candIdx -= m_indexOffset;
  }
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  CHECK(mmvdIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "GPM MMVD index is invalid");
  CHECK(!pu.cu->geoFlag || CU::isIBC(*pu.cu), "incorrect GPM setting")

  Mv  mvOffset(0, 0);

  if (mmvdIdx >= 0)
  {
    const int mvShift = MV_FRACTIONAL_BITS_DIFF;
    const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
    const int refExtMvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
    int fPosStep = (extMMVD ? (mmvdIdx >> 3) : (mmvdIdx >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx - (fPosStep << 3)) : (mmvdIdx - (fPosStep << 2)));
    int offset = (extMMVD ? refExtMvdCands[fPosStep] : refMvdCands[fPosStep]);

    if (fPosPosition == 0)
    {
      mvOffset = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      mvOffset = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      mvOffset = Mv(0, offset);
    }
    else if (fPosPosition == 3)
    {
      mvOffset = Mv(0, -offset);
    }
    else if (fPosPosition == 4)
    {
      mvOffset = Mv(offset, offset);
    }
    else if (fPosPosition == 5)
    {
      mvOffset = Mv(offset, -offset);
    }
    else if (fPosPosition == 6)
    {
      mvOffset = Mv(-offset, offset);
    }
    else if (fPosPosition == 7)
    {
      mvOffset = Mv(-offset, -offset);
    }
  }
  pu.cu->affine = true;
  pu.mergeFlag  = true;
  pu.interDir       = interDirNeighbours[candIdx];
  pu.cu->affineType = affineType[candIdx];
  pu.cu->bcwIdx     = bcwIdx[candIdx];
#if JVET_AG0276_NLIC
  pu.cu->altLMFlag = altLMFlag[candIdx];
  pu.cu->altLMParaUnit = altLMParaNeighbours[candIdx];
#endif
#if INTER_LIC
  pu.cu->licFlag = licFlags[candIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  setLICParamToPu(pu, candIdx, licInheritPara[candIdx]);
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  pu.colIdx = colIdx[candIdx];
#endif
  pu.mergeType    = mergeType[candIdx];

  pu.cu->obmcFlag = true;
  //pu.cu->obmcFlag = obmcFlags[candIdx];

  if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    THROW("Invalid merge type");
  }
  else
  {
    for (int i = 0; i < 2; ++i)
    {
      if (pu.cs->slice->getNumRefIdx(RefPicList(i)) > 0)
      {
        const MvField *mvField = mvFieldNeighbours[(candIdx << 1) + i];
        pu.mvpIdx[i]     = 0;
        pu.mvpNum[i]     = 0;
        pu.mvd[i]        = Mv();
        pu.refIdx[i]     = mvField[0].refIdx;
        pu.mvAffi[i][0]  = mvField[0].mv + mvOffset;
        pu.mvAffi[i][1]  = mvField[1].mv + mvOffset;
        pu.mvAffi[i][2]  = mvField[2].mv + mvOffset;

        pu.mv[i] = pu.mvAffi[i][0];// For split reordering
      }
      else
      {
        pu.mvpIdx[i]     = 0;
        pu.mvpNum[i]     = 0;
        pu.mvd[i]        = Mv();
        pu.refIdx[i]     = -1;
        pu.mvAffi[i][0]  = Mv();
        pu.mvAffi[i][1]  = Mv();
        pu.mvAffi[i][2]  = Mv();

        pu.mv[i] = Mv();// For split reordering
      }
    }
  }
}
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
unsigned DeriveCtx::CtxCUSeparateTree( const CodingStructure& cs, Partitioner& partitioner )
{
  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );
#if HEVC_TILES_WPP
  const unsigned curTileIdx  = cs.picture->tileMap->getTileIdxMap( pos );
#endif

#if HEVC_TILES_WPP
  const CodingUnit *cuLeft = cs.getCURestricted( pos.offset(-1, 0), curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset(0, -1), curSliceIdx, curTileIdx, partitioner.chType );
#else
  const CodingUnit *cuLeft = cs.getCURestricted( pos.offset(-1, 0), pos, curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit *cuAbove = cs.getCURestricted( pos.offset(0, -1), pos, curSliceIdx, curTileIdx, partitioner.chType );
#endif

  unsigned ctxId = ( cuLeft && cuLeft->separateTree ) ? 1 : 0;
  ctxId += ( cuAbove && cuAbove->separateTree ) ? 1 : 0;

  return ctxId;
}
#endif
