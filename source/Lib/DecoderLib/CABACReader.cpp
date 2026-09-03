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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)           const CodingStatisticsClassType CSCT(x);                       m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)        const CodingStatisticsClassType CSCT(x,y);                     m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)    const CodingStatisticsClassType CSCT(x, s.width, s.height);    m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z) const CodingStatisticsClassType CSCT(x, s.width, s.height, z); m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)                  m_BinDecoder.set( x );
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif

#if JVET_AG0196_CABAC_RETRAIN
namespace CabacRetrain
{
  extern SliceType sliceReport;
}
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
Partitioner *g_decPartitionerSST;
#endif

void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.getSliceType();
  int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }

#if JVET_AH0176_LOW_DELAY_B_CTX
  if (sliceType == B_SLICE && slice.getCheckLDC())
  {
    sliceType = L_SLICE;
  }
#endif

  m_BinDecoder.reset(qp, (int)sliceType);

#if JVET_AG0196_CABAC_RETRAIN
  slice.setCabacInitSliceType( sliceType );
#endif
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  if( slice.getSPS()->getTempCabacInitMode() )
  {
    m_CABACDataStore->loadCtxStates( &slice, getCtx() );
  }
#endif
#if JVET_AG0196_CABAC_RETRAIN
  CabacRetrain::sliceReport = slice.getCabacInitSliceType();
#endif
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP( STATS__TRAILING_BITS, m_Bitstream->readOutTrailingBits(), 0 );
#else
    m_Bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

#if JVET_AI0087_BTCUS_RESTRICTION 
bool CABACReader::isLumaNonBoundaryCu(const Partitioner& partitioner, SizeType picWidth, SizeType picHeight )
{
  bool validCU = false;
  if (isLuma(partitioner.chType))
  {
    int maxWidthHeight = std::max(partitioner.currArea().lwidth(), partitioner.currArea().lheight()) - 1;
    if ((partitioner.currArea().Y().x + maxWidthHeight < picWidth)
      && (partitioner.currArea().Y().y + maxWidthHeight < picHeight))
    {
      validCU = true;
    }
  }
  return validCU;
}
void CABACReader::setBtFirstPart(const Partitioner& partitioner, SizeType blockSize, CodingStructure& cs, PartSplit setValue)
{
  if (blockSize == 128)
  {
    cs.btFirstPartDecs[0] = setValue;
  }
  else if (blockSize == 64)
  {
    cs.btFirstPartDecs[1] = setValue;
  }
  else if (blockSize == 32)
  {
    cs.btFirstPartDecs[2] = setValue;
  }
  else if (blockSize == 16)
  {
    cs.btFirstPartDecs[3] = setValue;
  }  
}
#endif

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    void  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

void CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_tree_unit() pos=(%d,%d)\n", area.lx(), area.ly());
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  g_decPartitionerSST = new QTBTPartitioner; //PartitionerFactory::get( *cs.slice );
#endif

  partitioner.initCtu(area, CH_L, *cs.slice);
  #if JVET_AI0087_BTCUS_RESTRICTION 
  std::memset(cs.btFirstPartDecs,0, sizeof(cs.btFirstPartDecs));
#endif
  #if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  cs.treeType = partitioner.treeType = TREE_D;
  cs.modeType = partitioner.modeType = MODE_TYPE_ALL;
#endif

#if NN_LF_UNIFIED
  if (cs.sps->getNnlfUnifiedEnabledFlag() && ctuRsAddr == 0)
  {
    readNnlfUnifiedParameters(cs);
  }
#endif

  sao( cs, ctuRsAddr );
#if JVET_W0066_CCSAO
  if (cs.sps->getCCSAOEnabledFlag())
  {
    for ( int compIdx = 0; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
#if JVET_AL0142_CCSAO_REUSE_CTU
      if (cs.slice->m_ccSaoComParam.enabled[compIdx] && cs.slice->m_ccSaoComParam.reusePrv[compIdx] != CCSAO_REUSE_PARAM_CTU)
#else
      if (cs.slice->m_ccSaoComParam.enabled[compIdx])
#endif
      {
        const int setNum = cs.slice->m_ccSaoComParam.setNum[compIdx];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccSaoControlIdc(cs, ComponentID(compIdx), ctuRsAddr, cs.slice->m_ccSaoControl[compIdx], lumaPos, setNum);
#if JVET_AL0142_CCSAO_REUSE_CTU
        // Allocated ccSaoControl in ParseCcSao, assigned here
        g_ccSaoPrvParam[compIdx][0].ccSaoControl[ctuRsAddr] = cs.slice->m_ccSaoControl[compIdx][ctuRsAddr];
#endif
      }
#if JVET_AL0142_CCSAO_REUSE_CTU
      else if (cs.slice->m_ccSaoComParam.enabled[compIdx])  // && CCSAO_REUSE_PARAM_CTU
      {
        // load ccSaoControl
        int prvId = cs.slice->m_ccSaoComParam.reusePrvId[compIdx];
        cs.slice->m_ccSaoControl[compIdx][ctuRsAddr] = g_ccSaoPrvParam[compIdx][prvId].ccSaoControl[ctuRsAddr];
      }
      else
      {
        cs.slice->m_ccSaoControl[compIdx][ctuRsAddr] = 0;
      }
#endif
    }
  }
#endif
#if JVET_AL0153_ALF_CCCM
  lfCccm(cs, ctuRsAddr);
#endif
  if (cs.sps->getALFEnabledFlag() && (cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y)))
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t          curTileIdx = cs.pps->getTileIdx( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {

      if (cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx))
      {
        uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
        int ctx = 0;
        ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
        ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;

        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ALF);
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
        if (compIdx == COMPONENT_Y || (compIdx != COMPONENT_Y && !cs.slice->getTileGroupAlfReuseFlag((ComponentID)compIdx)))
        {
#endif
        ctbAlfFlag[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
        }
#endif

        if (isLuma((ComponentID)compIdx) && ctbAlfFlag[ctuRsAddr])
        {
          readAlfCtuFilterIndex(cs, ctuRsAddr);
#if ALF_IMPROVEMENT
          int apsIdxCtu = cs.slice->getPic()->getAlfCtbFilterIndex()[ctuRsAddr] - NUM_FIXED_FILTER_SETS;
          if (apsIdxCtu >= 0)
          {
            int apsIdx = cs.slice->getTileGroupApsIdLuma()[apsIdxCtu];
            CHECK(cs.slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized");
            const AlfParam& alfParam = cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
            const int numAlts = alfParam.numAlternativesLuma;
            uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData(compIdx);
            ctbAlfAlternative[ctuRsAddr] = 0;
            uint8_t decoded = 0;
            while( decoded < numAlts - 1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( COMPONENT_Y ) ) )
            {
              ++decoded;
            }
            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
#endif
        }
        if( isChroma( (ComponentID)compIdx ) )
        {
          int apsIdx = cs.slice->getTileGroupApsIdChroma();
          CHECK(cs.slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized");
          const AlfParam& alfParam = cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
          const int numAlts = alfParam.numAlternativesChroma;
          uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
          ctbAlfAlternative[ctuRsAddr] = 0;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          if(!cs.slice->getTileGroupAlfReuseFlag((ComponentID)compIdx))
          {
#endif
          if( ctbAlfFlag[ctuRsAddr] )
          {
            uint8_t decoded = 0;
#if ALF_IMPROVEMENT
            while( decoded < numAlts-1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx ) ) )
#else
            while( decoded < numAlts - 1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx - 1 ) ) )
#endif
            {
              ++decoded;
            }

            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          }
#endif
        }
      }
    }
  }
  if (cs.sps->getCCALFEnabledFlag())
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      if (cs.slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1] && (!cs.slice->getTileGroupCcalfReuseFlag(ComponentID(compIdx))))
#else
      if (cs.slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
#endif
      {
        const int filterCount   = cs.slice->m_ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccAlfFilterControlIdc(cs, ComponentID(compIdx), ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos,
                              filterCount);
      }
    }
  }

#if JVET_AK0065_TALF
  const TAlfControl talfControl = cs.slice->getTileGroupTAlfControl();
  if (cs.sps->getUseTAlf() && talfControl.enabledFlag)
  {
    const int    ry = ctuRsAddr / cs.pcv->widthInCtus;
    const int    rx = ctuRsAddr % cs.pcv->widthInCtus;
    const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);
    readTAlfFilterControlIdc(cs, COMPONENT_Y, ctuRsAddr, cs.slice->m_tAlfCtbControl, lumaPos);
  }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  /*bool isLast =*/ coding_tree( cs, partitioner, cuCtx, qps );
  cs.setLumaPointers( cs );
#else

#if TU_256
  if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE ) )
#else
  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
#endif
  {
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    CUCtx cuCtxChroma(qps[CH_C]);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &cuCtxChroma);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxChroma.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  delete g_decPartitionerSST;
#endif

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->getSliceQpBase() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->getSliceQpBase() );

}

#if NN_LF_UNIFIED
void CABACReader::readNnlfUnifiedParameters( CodingStructure& cs )
{
  // parse parameter id of each block
  NNFilterUnified::FilterParameters& prm = cs.picture->m_picprm;
  const NNFilterUnified::SliceParameters& sprm = cs.picture->slices[0]->getNnlfUnifiedParameters();
  int cpt = 0;
  int prmNum = prm.prmNum;
  for( int y = 0; y < prm.numBlocksHeight; ++y )
  {
    for( int x = 0; x < prm.numBlocksWidth; ++x, ++cpt )
    {
      if( sprm.mode < prmNum )
      {
        prm.prmId[cpt] = sprm.mode;
        continue;
      }

      bool useNnlf, useFirstParam = false;
      useNnlf = m_BinDecoder.decodeBin( Ctx::nnlfUnifiedParams( 0 ) );

      if( prmNum == 1 )
      {
        prm.prmId[cpt] = useNnlf ? 0 : -1;
      }
      else if( !useNnlf )
      {
        prm.prmId[cpt] = -1;
      }
      else
      {
        useFirstParam = m_BinDecoder.decodeBin( Ctx::nnlfUnifiedParams( 1 ) );
        if( prmNum == 2 )
        {
          prm.prmId[cpt] = useFirstParam ? 0 : 1;
        }
        else if( useFirstParam )
        {
          prm.prmId[cpt] = 0;
        }
        else
        {
          uint32_t nnlfPrmIdMinus1 = 0;
          xReadTruncBinCode( nnlfPrmIdMinus1, prmNum - 1 );
          prm.prmId[cpt] = nnlfPrmIdMinus1 + 1;
        }
      }
    }
  }
}
#endif

void CABACReader::readAlfCtuFilterIndex(CodingStructure& cs, unsigned ctuRsAddr)
{
  short* alfCtbFilterSetIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    unsigned usePrevFilt = m_BinDecoder.decodeBin(Ctx::AlfUseTemporalFilt());
    if (usePrevFilt)
    {
      if (numAps > 1)
      {
        xReadTruncBinCode(filtIndex, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
      filtIndex += (unsigned)(NUM_FIXED_FILTER_SETS);
    }
    else
    {
      xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
  }
  alfCtbFilterSetIndex[ctuRsAddr] = filtIndex;
}
void CABACReader::ccAlfFilterControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,
                                        uint8_t *filterControlIdc, Position lumaPos, int filterCount)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_ALF_BLOCK_LEVEL_IDC );

  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
#if JVET_X0071_LONGER_CCALF
    ctxt += (filterControlIdc[curIdx - 1] != 1) ? 1 : 0;
#else
    ctxt += ( filterControlIdc[curIdx - 1] ) ? 1 : 0;
#endif
  }
  if (aboveAvail)
  {
#if JVET_X0071_LONGER_CCALF
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus] != 1) ? 1 : 0;
#else
    ctxt += ( filterControlIdc[curIdx - cs.pcv->widthInCtus] ) ? 1 : 0;
#endif
  }
  ctxt += ( compID == COMPONENT_Cr ) ? 3 : 0;

  int idcVal  = m_BinDecoder.decodeBin( Ctx::CcAlfFilterControlFlag( ctxt ) );
  if ( idcVal )
  {
    while ( ( idcVal != filterCount ) && m_BinDecoder.decodeBinEP() )
    {
      idcVal++;
    }
  }

#if JVET_X0071_LONGER_CCALF
  int pos0 = 1;
  idcVal = (idcVal == pos0 ? 0 : idcVal < pos0 ? idcVal + 1 : idcVal);
#endif

  filterControlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "cc_alf_filter_control_idc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}
#if JVET_AK0065_TALF
void CABACReader::readTAlfFilterControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,  TAlfCtbParam *filterControlIdc, Position lumaPos)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_ALF_BLOCK_LEVEL_IDC );

  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxId           = 0;

  if (leftAvail)
  {
    ctxId += ( filterControlIdc[curIdx - 1].enabledFlag ) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxId += ( filterControlIdc[curIdx - cs.pcv->widthInCtus].enabledFlag ) ? 1 : 0;
  }
  auto talfControl = cs.slice->getTileGroupTAlfControl();
  if (talfControl.newFilters)
  {
    ctxId += 3;
  }
  filterControlIdc[curIdx].reset();
  filterControlIdc[curIdx].enabledFlag = m_BinDecoder.decodeBin( Ctx::TAlfFilterControlFlag( ctxId ) );
  if (filterControlIdc[curIdx].enabledFlag)
  {
    int      numSets = int(talfControl.apsIds.size());
    if (numSets > 1 && !talfControl.newFilters)
    {
      filterControlIdc[curIdx].setIdx = unary_max_eqprob(numSets - 1);
    }
    int       apsId       = talfControl.apsIds[filterControlIdc[curIdx].setIdx];
    const int filterCount = cs.slice->getTAlfAPSs()[apsId]->getTAlfAPSParam().filterCount;
    if (filterCount > 1)
    {
      filterControlIdc[curIdx].filterIdx = unary_max_eqprob(filterCount - 1);
    }
  }
}
#endif
//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(!(cs.pps->getUseBIF() || cs.sps->getSAOEnabledFlag() || cs.pps->getUseChromaBIF()))
#else
  // If neither BIF nor SAO is enabled, we can return.
  if(!(cs.pps->getUseBIF() || cs.sps->getSAOEnabledFlag()))
#endif
  {
    return;
  }
  // At least one is enabled, it is safe to assume we can do getSAO().
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(!(cs.pps->getUseChromaBIF() || cs.sps->getSAOEnabledFlag()))
  {
        return;
  }
    SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#endif
#endif
  
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  // The getSAO() function call has been moved up.
#else
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#endif
  bool              slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool              slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = cs.slice->getIndependentSliceIdx();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SAO );

  const unsigned  curTileIdx  = cs.pps->getTileIdx( pos );
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth( toChannelType(compID) ) );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

#if JVET_V0094_BILATERAL_FILTER
void CABACReader::bif( const ComponentID compID, CodingStructure& cs)
{
  int width = cs.picture->lwidth();
  int height = cs.picture->lheight();

  int blockWidth = cs.pcv->maxCUWidth;
  int blockHeight = cs.pcv->maxCUHeight;

  int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
  int heightInBlocks = height / blockHeight + (height % blockHeight != 0);

  for (int i = 0; i < widthInBlocks * heightInBlocks; i++)
  {
    bif(compID, cs, i);
  }
}

void CABACReader::bif( const ComponentID compID, CodingStructure& cs, unsigned ctuRsAddr )
{
//  const SPS&   sps = *cs.sps;
  const PPS&   pps = *cs.pps;

  if( isLuma(compID) && !pps.getUseBIF())
  {
    return;
  }

#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if( isChroma( compID ) && !pps.getUseChromaBIF() )
  {
    return;
  }
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2( STATS__CABAC_BITS__BIF, compID );

  BifParams& bifParams = cs.picture->getBifParam( compID );

  if( ctuRsAddr == 0 )
  {
    int width = cs.picture->lwidth();
    int height = cs.picture->lheight();
    int blockWidth = cs.pcv->maxCUWidth;
    int blockHeight = cs.pcv->maxCUHeight;

    int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
    int heightInBlocks = height / blockHeight + (height % blockHeight != 0);
    bifParams.numBlocks = widthInBlocks * heightInBlocks;
    bifParams.ctuOn.resize( bifParams.numBlocks );
    std::fill(bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 0);

    bifParams.allCtuOn = m_BinDecoder.decodeBinEP();

    if( bifParams.allCtuOn == 0 )
    {
      bifParams.frmOn = m_BinDecoder.decodeBinEP();
    }
    else
    {
      bifParams.frmOn = 0;
    }
  }

  if( bifParams.allCtuOn )
  {
    bifParams.ctuOn[ctuRsAddr] = 1;
  }
  else
  {
    if( bifParams.frmOn )
    {
      bifParams.ctuOn[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::BifCtrlFlags[compID]() );
    }
    else
    {
      bifParams.ctuOn[ctuRsAddr] = 0;
    }
  }
}
#endif

#if JVET_W0066_CCSAO
void CABACReader::ccSaoControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,
                                  uint8_t *controlIdc, Position lumaPos, int setNum)
{
  //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_SAO_BLOCK_LEVEL_IDC );

  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( controlIdc[curIdx - 1] ) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += ( controlIdc[curIdx - cs.pcv->widthInCtus] ) ? 1 : 0;
  }
  ctxt += ( compID == COMPONENT_Y  ) ? 0 
        : ( compID == COMPONENT_Cb ) ? 3 : 6;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__SAO, compID);
  int idcVal  = m_BinDecoder.decodeBin( Ctx::CcSaoControlIdc( ctxt ) );
  if ( idcVal )
  {
    while ( ( idcVal != setNum ) && m_BinDecoder.decodeBinEP() )
    {
      idcVal++;
    }
  }
  controlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "cc_sao_control_idc() compID=%d pos=(%d,%d) ctxt=%d, setNum=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, setNum, idcVal);
}
#endif

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, int (&qps)[2] )
#else
void CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
#endif
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pps.getUseDQP() && partitioner.currQgEnable())
#else
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma(partitioner.chType) )
#endif
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
    cs.chromaQpAdj = 0;
  }

#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
      cs.chromaQpAdj = 0;
    }
  }
#endif

  const PartSplit splitMode = split_cu_mode( cs, partitioner );

#if JVET_AI0087_BTCUS_RESTRICTION
  CHECK(!partitioner.canSplit(splitMode, cs, false, false), "Got an invalid split!");
#else
  CHECK(!partitioner.canSplit(splitMode, cs), "Got an invalid split!");
#endif

  if( splitMode != CU_DONT_SPLIT )
  {
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
#if TU_256
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );

    if (CS::isDualITree(cs) && pPartitionerChroma != nullptr
        && (partitioner.currArea().lwidth() >= maxSize || partitioner.currArea().lheight() >= maxSize))
#else
    if (CS::isDualITree(cs) && pPartitionerChroma != nullptr
      && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
#endif
    {
      partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
      pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
      bool beContinue     = true;
      bool lumaContinue   = true;
      bool chromaContinue = true;

      while (beContinue)
      {
#if TU_256
        if( partitioner.currArea().lwidth() > maxSize || partitioner.currArea().lheight() > maxSize )
#else
        if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
#endif
        {
          if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
          {
            coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
          }
          lumaContinue   = partitioner.nextPart(cs);
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
        else
        {
          // dual tree coding under 64x64 block
          if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
          {
            coding_tree(cs, partitioner, cuCtx);
          }
          lumaContinue = partitioner.nextPart(cs);
          if (cs.area.blocks[pPartitionerChroma->chType].contains(
                pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
          {
            coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
          }
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
      }
      partitioner.exitCurrSplit();
      pPartitionerChroma->exitCurrSplit();

      // cat the chroma CUs together
      CodingUnit *currentCu        = cs.getCU(partitioner.currArea().lumaPos(), CHANNEL_TYPE_LUMA);
      CodingUnit *nextCu           = nullptr;
      CodingUnit *tempLastLumaCu   = nullptr;
      CodingUnit *tempLastChromaCu = nullptr;
      ChannelType currentChType    = currentCu->chType;
      while (currentCu->next != nullptr)
      {
        nextCu = currentCu->next;
        if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_LUMA)
        {
          tempLastLumaCu = currentCu;
          if (tempLastChromaCu != nullptr)   // swap
          {
            tempLastChromaCu->next = nextCu;
          }
        }
        else if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_CHROMA)
        {
          tempLastChromaCu = currentCu;
          if (tempLastLumaCu != nullptr)   // swap
          {
            tempLastLumaCu->next = nextCu;
          }
        }
        currentCu     = nextCu;
        currentChType = currentCu->chType;
      }

      CodingUnit *chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), CHANNEL_TYPE_CHROMA);
      tempLastLumaCu->next      = chromaFirstCu;
    }
    else
    {
#endif // JVET_AI0136_ADAPTIVE_DUAL_TREE
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      const ModeType modeTypeParent = partitioner.modeType;
      cs.modeType = partitioner.modeType = mode_constraint(cs, partitioner, splitMode);   // change for child nodes
      // decide chroma split or not
      bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTRA;
      CHECK(chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma");
      if (partitioner.treeType == TREE_D)
      {
        cs.treeType = partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
      }
#endif
      partitioner.splitCurrArea( splitMode, cs );
      do
      {
        if( cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          coding_tree( cs, partitioner, cuCtx, qps );
#else
          coding_tree( cs, partitioner, cuCtx );
#endif
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( chromaNotSplit )
      {
        CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        cs.treeType = partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }

        //recover treeType
        partitioner.chType = CHANNEL_TYPE_LUMA;
        cs.treeType = partitioner.treeType = TREE_D;
      }

      //recover ModeType
      cs.modeType = partitioner.modeType = modeTypeParent;
#endif
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
    }
#endif
    return;
  }


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType, implicitSplit);
#else
  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType );
#endif

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
  cu.tileIdx = cs.pps->getTileIdx( currArea.lumaPos() );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( cu.cs->treeType != partitioner.treeType, "treeType mismatch" );
  int lumaQPinLocalDualTree = -1;
#endif


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  cu.isSST        = ( cu.slice->getSeparateTreeEnabled() && cu.slice->getProcessingIntraRegion() ) ? true : false;
  cu.separateTree = ( cu.isSST && cu.slice->getProcessingSeparateTrees() ) ? true : false;
  cu.predMode     = ( cu.isSST ) ? MODE_INTRA : cu.predMode;
  if ( cu.isSST )
  {
    cu.chType = cu.slice->getProcessingChannelType();
    cu.slice->setCUIntraRegionRoot( &cu );
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (!cu.slice->getSeparateTreeEnabled() || !cu.slice->getProcessingIntraRegion() || (cu.slice->isIntra() && cu.slice->getUseIBC()))
#else
  if (!cu.slice->getSeparateTreeEnabled() || !cu.slice->getProcessingIntraRegion() || (cu.slice->isIntra() && cu.slice->getSPS()->getIBCFlag()))
#endif
  {
    coding_unit_pred_mode( cu, partitioner );
  }
  if ( !cu.slice->getSeparateTreeEnabled() || !cu.slice->getProcessingIntraRegion() )
  {
    separate_tree_cu_flag( cu, partitioner );
  }

  if ( cu.slice->getSeparateTreeEnabled() && CU::isIntra( cu ) && !cu.slice->getProcessingIntraRegion() && cu.separateTree )
  {
    CHECK( !cu.isSST, "Separate tree root not found" );
    const bool separateTree = cu.separateTree;
    const PartSplit implicitSplit = partitioner.getImplicitSplit( cs ); // FLL to check
    cs.popLastCU( implicitSplit );
    cs.slice->setProcessingIntraRegion   ( true );
    cs.slice->setProcessingSeparateTrees ( separateTree );
    cs.slice->setProcessingChannelType   ( CH_L );
    cs.slice->setIntraRegionRoot         ( &partitioner );

    if ( cu.isSST )
    {
      cu.slice->setProcessingIntraRegion   ( true );
      cu.slice->setCUIntraRegionRoot( &cu );
    }

    CHECK( separateTree && ( partitioner.currArea().lwidth() > 256 || partitioner.currArea().lheight() > 256 ), "Separate tree cannot have width or height greater than 256" );

    Partitioner *& partitionerSST = g_decPartitionerSST;
    partitionerSST->copyState( partitioner );
    partitionerSST->chType = CH_L;
    /*lastSegment =*/ coding_tree( cs, *partitionerSST, cuCtx, qps );
    cs.setLumaPointers( cs );

    if ( separateTree )
    {
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
      cs.chromaTreePred(cs, partitioner);
#endif
      CUCtx cuCtxChroma( qps[CH_C] );
      cs.slice->setProcessingChannelType ( CH_C );
      partitionerSST->chType = CH_C;
      /*lastSegment =*/ coding_tree( cs, *partitionerSST, cuCtxChroma, qps );
      qps[CH_C] = cuCtxChroma.qp;
    }

    cs.slice->setProcessingIntraRegion( false );
    return ;// lastSegment;
  }
#endif

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pps.getUseDQP() && CS::isDualITree(cs) && isChroma(cu.chType))
#else
  if (pps.getUseDQP() && partitioner.isSepTree(cs) && isChroma(cu.chType))
#endif
  {
    const Position chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, cu.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, cu.chromaFormat));
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    const CodingUnit* colLumaCu = cs.getCU(lumaRefPos, CHANNEL_TYPE_LUMA);
#else
    //derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    //therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma cu qp
    const CodingUnit* colLumaCu = cs.getLumaCU( lumaRefPos );
    CHECK( colLumaCu == nullptr, "colLumaCU shall exist" );
    lumaQPinLocalDualTree = cuCtx.qp;
#endif
    if (colLumaCu)
    {
      cuCtx.qp = colLumaCu->qp;
    }
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level
#if JVET_AC0094_REF_SAMPLES_OPT
  cu.areAboveRightUnavail = cs.getCURestricted(cu.Y().topRight().offset(1, -1), cu, partitioner.chType) == nullptr;
  cu.areBelowLeftUnavail = cs.getCURestricted(cu.Y().bottomLeft().offset(-1, 1), cu, partitioner.chType) == nullptr;
#endif

  // coding unit
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() pos=(%d,%d) size=%dx%d chType=%d depth=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.blocks[cu.chType].width, cu.blocks[cu.chType].height, cu.chType, cu.depth);

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // If context data collection is active and if on bottom of the CTU, start the counters
  if ( m_BinDecoder.getBinBuffer() )
  {
    m_BinDecoder.setBinBufferActive( CU::isOnCtuBottom( cu ) );
  }
#endif

  coding_unit( cu, partitioner, cuCtx );

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // Done with the data collection for this CU
  if ( m_BinDecoder.getBinBuffer() )
  {
    m_BinDecoder.setBinBufferActive( false );
  }
#endif

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  //recover cuCtx.qp to luma qp after decoding the chroma CU
  if( pps.getUseDQP() && partitioner.isSepTree( cs ) && isChroma( cu.chType ) )
  {
    cuCtx.qp = lumaQPinLocalDualTree;
  }
#endif
  uint32_t compBegin;
  uint32_t numComp;
  bool jointPLT = false;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs))
#else
  if (cu.isSepTree())
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isLocalSepTree() )
    {
      compBegin = COMPONENT_Y;
      numComp = (cu.chromaFormat != CHROMA_400)?3: 1;
      jointPLT = true;
    }
    else
    {
#endif
      if (isLuma(partitioner.chType))
      {
        compBegin = COMPONENT_Y;
        numComp   = 1;
      }
      else
      {
        compBegin = COMPONENT_Cb;
        numComp   = 2;
      }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    }
#endif
  }
  else
  {
    compBegin = COMPONENT_Y;
    numComp = (cu.chromaFormat != CHROMA_400) ? 3 : 1;
    jointPLT = true;
  }
  if (CU::isPLT(cu))
  {
    cs.reorderPrevPLT(cs.prevPLT, cu.curPLTSize, cu.curPLT, cu.reuseflag, compBegin, numComp, jointPLT);
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE( g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
#endif
    DTRACE(g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height,
           cu.qp);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  }
#endif
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
ModeType CABACReader::mode_constraint( CodingStructure& cs, Partitioner &partitioner, PartSplit splitMode )
{
  int val = cs.signalModeCons( splitMode, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MODE_CONSTRAINT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );
    bool flag = m_BinDecoder.decodeBin( Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
    return flag ? MODE_TYPE_INTRA : MODE_TYPE_INTER;
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    return MODE_TYPE_INTRA;
  }
  else
  {
    return partitioner.modeType;
  }
}
#endif
PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );
#if JVET_AI0087_BTCUS_RESTRICTION
  bool disableBTV = false;
  bool disableBTH = false;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (CABACReader::isLumaNonBoundaryCu(partitioner, cs.picture->lwidth(), cs.picture->lheight()) && (!(cs.slice->getProcessingIntraRegion() && cs.slice->getProcessingSeparateTrees()) || cs.slice->isIntra()) )
#else
  if (CABACReader::isLumaNonBoundaryCu(partitioner, cs.picture->lwidth(), cs.picture->lheight()) )
#endif
  {
    if ((partitioner.currBtDepth == 0) && (partitioner.currArea().lwidth() == partitioner.currArea().lheight()))
    {
      CABACReader::setBtFirstPart(partitioner, partitioner.currArea().lwidth(), cs, CTU_LEVEL);
    }

    if ((partitioner.currBtDepth == 1) && (partitioner.currPartIdx() == 1))
    {
      if (partitioner.currPartLevel().split == CU_HORZ_SPLIT)
      {
        if (partitioner.currArea().lwidth() == 128 && cs.btFirstPartDecs[0] == CU_VERT_SPLIT)
        {
          disableBTV = true;
        }
        else if (partitioner.currArea().lwidth() == 64 && cs.btFirstPartDecs[1] == CU_VERT_SPLIT)
        {
          disableBTV = true;
        }
        else if (partitioner.currArea().lwidth() == 32 && cs.btFirstPartDecs[2] == CU_VERT_SPLIT)
        {
          disableBTV = true;
        }
        else if (partitioner.currArea().lwidth() == 16 && cs.btFirstPartDecs[3] == CU_VERT_SPLIT)
        {
          disableBTV = true;
        }
      }
      else if (partitioner.currPartLevel().split == CU_VERT_SPLIT)   // BTV Case
      {
        if (partitioner.currArea().lheight() == 128 && cs.btFirstPartDecs[0] == CU_HORZ_SPLIT)
        {
          disableBTH = true;
        }
        else if (partitioner.currArea().lheight() == 64 && cs.btFirstPartDecs[1] == CU_HORZ_SPLIT)
        {
          disableBTH = true;
        }
        else if (partitioner.currArea().lheight() == 32 && cs.btFirstPartDecs[2] == CU_HORZ_SPLIT)
        {
          disableBTH = true;
        }
        else if (partitioner.currArea().lheight() == 16 && cs.btFirstPartDecs[3] == CU_HORZ_SPLIT)
        {
          disableBTH = true;
        }
      }
    }
  }
#endif
  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  unsigned maxMtt;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv, maxMtt 
#if JVET_AI0087_BTCUS_RESTRICTION
    , disableBTV, disableBTH
#endif
  );
#else
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv
#if JVET_AI0087_BTCUS_RESTRICTION
    , disableBTV, disableBTH
#endif
  );
#endif

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl
#if JVET_AI0087_BTCUS_RESTRICTION
    , disableBTV, disableBTH
#endif
  );

  bool isSplit = canBh || canBv || canTh || canTv || canQt;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt = canQt;
#endif

  if( canNo && isSplit )
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE

    if (cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && (partitioner.currArea().lwidth() <= 256 && partitioner.currArea().lheight() <= 256))
    {
      isSplit = false;
    }
    else
    {
#if JVET_AH0135_TEMPORAL_PARTITIONING
      bool      colSplitPredExist = false;
      SplitPred currentSplitPred;

      Picture *pColPic = cs.slice->getRefPic(RefPicList(cs.slice->isInterB() ? 1 - cs.slice->getColFromL0Flag() : 0),
                                             cs.slice->getColRefIdx());
      if (!cs.slice->isIntra() && pColPic != NULL && !pColPic->isRefScaled(cs.pps) && pColPic->cs->slice != NULL
          && (pColPic->cs->area.Y().contains(partitioner.currArea().blocks[partitioner.chType].pos().offset(
            (partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
            ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)))))
      {
        currentSplitPred  = pColPic->cs->getQtDepthInfo(partitioner.currArea().blocks[partitioner.chType].pos().offset(
          (partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
          ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)));
        colSplitPredExist = true;
      }

      if (colSplitPredExist && (partitioner.currQtDepth < currentSplitPred.qtDetphCol))
      {
        if (isQt && canBtt)
        {
          isQt = m_BinDecoder.decodeBin(Ctx::SplitQtFlag(ctxQtSplit));

          if (isQt)
          {
            return CU_QUAD_SPLIT;
          }
          canBtt = false;

          if (!isQt)
          {
            isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
          }
        }
        else
        {
          isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
        }
      }
      else
#endif
        isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
      DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit);
    }
#else   // JVET_AI0136_ADAPTIVE_DUAL_TREE
#if JVET_AH0135_TEMPORAL_PARTITIONING
    bool colSplitPredExist = false;
    SplitPred currentSplitPred;

    Picture* pColPic = cs.slice->getRefPic(RefPicList(cs.slice->isInterB() ? 1 - cs.slice->getColFromL0Flag() : 0), cs.slice->getColRefIdx());
    if (!cs.slice->isIntra() && pColPic != NULL && pColPic->cs->slice != NULL 
      && (pColPic->cs->area.Y().contains(partitioner.currArea().blocks[partitioner.chType].pos().offset((partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
        ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)))))
    {
      currentSplitPred = pColPic->cs->getQtDepthInfo(partitioner.currArea().blocks[partitioner.chType].pos().offset((partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
                                                     ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)));
      colSplitPredExist = true;
    }

    if (colSplitPredExist && (partitioner.currQtDepth < currentSplitPred.qtDetphCol))
    {
      if (isQt && canBtt)
      {
        isQt = m_BinDecoder.decodeBin(Ctx::SplitQtFlag(ctxQtSplit));

        if (isQt)
        {
          return CU_QUAD_SPLIT;
        }
        canBtt = false;

        if (!isQt)
        {
          isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
        }
      }
      else
      {
        isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
      }
    }
    else
    {
#endif
      isSplit = m_BinDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
#if JVET_AH0135_TEMPORAL_PARTITIONING
    }
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );
#endif // JVET_AI0136_ADAPTIVE_DUAL_TREE
  }

#if ENABLE_TRACING
  const CompArea& block = partitioner.currArea().blocks[partitioner.chType];
#endif

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctx=%d split=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxSplit, isSplit );


  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

#if !JVET_AH0135_TEMPORAL_PARTITIONING
  const bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt   = canQt;
#endif

  if( isQt && canBtt )
  {
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctx=%d qt=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool        isVer = canBv || canTv;

  if( isVer && canHor )
  {
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

  if( is12 && can14 )
  {
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  if (isVer && is12)
  {
    mode = CU_VERT_SPLIT;
  }
  else if (isVer && !is12)
  {
    mode = CU_TRIV_SPLIT;
  }
  else if (!isVer && is12)
  {
    mode = CU_HORZ_SPLIT;
  }
  else
  {
    mode = CU_TRIH_SPLIT;
  }

#if JVET_AI0087_BTCUS_RESTRICTION
  if (CABACReader::isLumaNonBoundaryCu(partitioner, cs.picture->lwidth(), cs.picture->lheight())
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      && (!(cs.slice->getProcessingIntraRegion() && cs.slice->getProcessingSeparateTrees()) || cs.slice->isIntra())
#endif
      )
  {
    if ((partitioner.currBtDepth == 1) && (partitioner.currPartIdx() == 0))
    {
      if (partitioner.currPartLevel().split == CU_HORZ_SPLIT)   // BTH Case
      {
        if (mode == CU_VERT_SPLIT)
        {
          CABACReader::setBtFirstPart(partitioner, partitioner.currArea().lwidth(), cs, CU_VERT_SPLIT);
        }
        else
        {
          CABACReader::setBtFirstPart(partitioner, partitioner.currArea().lwidth(), cs, CTU_LEVEL);
        }
      }
      else if (partitioner.currPartLevel().split == CU_VERT_SPLIT)   // BTV Case
      {
        if (mode == CU_HORZ_SPLIT)
        {
          CABACReader::setBtFirstPart(partitioner, partitioner.currArea().lheight(), cs, CU_HORZ_SPLIT);
        }
        else
        {
          CABACReader::setBtFirstPart(partitioner, partitioner.currArea().lheight(), cs, CTU_LEVEL);
        }
      }
    }
  }
#endif

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctxHv=%d ctx12=%d mode=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACReader::coding_unit_pred_mode( CodingUnit &cu, Partitioner &partitioner )
{
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // If context data collection is active and if on bottom of the CTU, start the counters
  if (m_BinDecoder.getBinBuffer())
  {
    m_BinDecoder.setBinBufferActive(CU::isOnCtuBottom(cu));
  }
#endif

  // skip flag
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((!cu.slice->isIntra() || cu.slice->getUseIBC()) && cu.Y().valid())
#else
  if ((!cu.slice->isIntra() || cu.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
#endif
  {
    cu_skip_flag( cu );
  }

  if ( !cu.skip )
  {
    pred_mode( cu );
  }

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // Done with the data collection for this CU
  if ( m_BinDecoder.getBinBuffer() )
  {
    m_BinDecoder.setBinBufferActive( false );
  }
#endif
}
#endif

void CABACReader::coding_unit(CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx)
{
  CodingStructure& cs = *cu.cs;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( cu.treeType != partitioner.treeType || cu.modeType != partitioner.modeType, "treeType or modeType mismatch" );
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  
  PredictionUnit *tpu=nullptr;
  if (cu.firstPU == nullptr)
  {
    tpu = & (cs.addPU(cu, partitioner.chType));
  }
  else
  {
    tpu = cu.firstPU;
  }
  PredictionUnit& pu=*tpu;
#else
  PredictionUnit&    pu = cs.addPU(cu, partitioner.chType);
#endif
  // skip flag

#if !JVET_AI0136_ADAPTIVE_DUAL_TREE

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((!cs.slice->isIntra() || cs.slice->getUseIBC()) && cu.Y().valid())
#else
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
#endif
  {
    cu_skip_flag( cu );
  }
#endif

  // skip data
  if( cu.skip )
  {
    cu.colorTransform = false;
    cs.addEmptyTUs( partitioner );
    MergeCtx           mrgCtx;
    prediction_unit  ( pu, mrgCtx );
#if INTER_LIC
    cu_lic_flag      ( cu );
#endif
#if ENABLE_OBMC
    // it just sets the OBMC flag and does not parse any syntax.
    obmc_flag        ( cu );
#endif
    end_of_ctu( cu, cuCtx );
    return;
  }

  // prediction mode and partitioning data
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
  pred_mode ( cu );
#endif

#if ENABLE_DIMD
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  if (!cu.slice->getPnnMode())
  {
    cu_dimd_flag(cu);
  }
#else
  cu_dimd_flag( cu );
#endif
#endif
  if (CU::isIntra(cu))
  {
    adaptive_color_transform(cu);
  }
  if (CU::isPLT(cu))
  {
    cu.colorTransform = false;
    cs.addTU(cu, partitioner.chType);
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs))
#else
    if (cu.isSepTree())
#endif
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      if( cu.chromaFormat != CHROMA_400 )
      {
        cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
      }
      else
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
    }
    end_of_ctu(cu, cuCtx);
    return;
  }

  // --> create PUs

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  end_of_ctu( cu, cuCtx );
}

void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__SKIP_FLAG, cu.lumaSize());
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (cu.slice->isIntra() && cu.cs->slice->getUseIBC())
#else
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
#endif
#else
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getUseIBC())
#else
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
#endif
#endif
  {
    cu.skip = false;
    cu.rootCbf = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if( !cu.slice->getSPS()->getUseIbcMerge() )
    {
      return;
    }
#endif

    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
    {
      unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
      unsigned skip  = m_BinDecoder.decodeBin(Ctx::SkipFlag(ctxId));

      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0);

      if (skip)
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
      }
    }
    return;
  }
#if !INTER_RM_SIZE_CONSTRAINTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ( !cu.cs->slice->getUseIBC() && cu.lwidth() == 4 && cu.lheight() == 4 )
#else
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
#endif
  {
    return;
  }
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if( !cu.cs->slice->getUseIBC() && cu.isConsIntra() )
#else
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
#endif
  {
    return;
  }
#endif
  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (skip && cu.cs->slice->getUseIBC())
#else
  if (skip && cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if( !cu.slice->getSPS()->getUseIbcMerge() )
    {
      cu.predMode = MODE_INTER;
    }
    else
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#endif
    {
#if !INTER_RM_SIZE_CONSTRAINTS
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        return;
      }
#endif
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);

      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        CHECK(cu.firstPU != nullptr, "abnormal situation (cu_skip_flag)");
        cu.cs->addPU(cu, cu.chType);
#endif
        cu.skip                      = true;
        cu.rootCbf                   = false;
        cu.predMode                  = MODE_IBC;
        cu.mmvdSkip                  = false;
        cu.firstPU->regularMergeFlag = false;
      }
      else
      {
        cu.predMode = MODE_INTER;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((skip && CU::isInter(cu) && cu.cs->slice->getUseIBC()) ||
    (skip && !cu.cs->slice->getUseIBC()))
#else
  if ((skip && CU::isInter(cu) && cu.cs->slice->getSPS()->getIBCFlag()) ||
    (skip && !cu.cs->slice->getSPS()->getIBCFlag()))
#endif
  {
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  if (skip && CU::interCcpMergeZeroRootCbfAllowed(cu))
  {
    inter_ccp_merge_root_cbf_zero(cu);
  }
#endif
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__IMV_FLAG, cu.lumaSize());

  if( !cu.cs->sps->getAMVREnabledFlag() )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    cu.imv = CU::isIBC(cu) && !cu.firstPU->mergeFlag ? 1 : cu.imv;
#endif
    return;
  }

#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (!CU::isIBC(cu) && (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]))
#else
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
#endif
  {
    return;
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool useIBCFrac = CU::isIBC(cu) && !cu.firstPU->mergeFlag && cu.cs->sps->getIBCFracFlag()
#if JVET_AA0070_RRIBC
                 && (cu.rribcFlipType == 0)
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                 && (cu.bvOneZeroComp == 0)
#endif
    ;
#endif
  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    cu.imv = CU::isIBC(cu) && !cu.firstPU->mergeFlag ? (useIBCFrac ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : 1) : cu.imv;
#endif
    return;
  }

  if ( cu.affine )
  {
    return;
  }


#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag && !pu.cs->slice->getAmvpSbTmvpAmvrEnabledFlag())
  {
    cu.imv = 0;
    return;
  }
#endif

  const SPS *sps = cu.cs->sps;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const CtxSet& imvCtx = CU::isIBC(cu) && pu.cs->sps->getIBCFracFlag() ? Ctx::ImvFlagIBC : Ctx::ImvFlag;
#endif

  unsigned value = 0;
  if (CU::isIBC(cu)
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      && !useIBCFrac
#endif
    )
  {
    value = 1;
  }
  else
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    value = m_BinDecoder.decodeBin(imvCtx(0));
#else
    value = m_BinDecoder.decodeBin(Ctx::ImvFlag(0));
#endif
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );

  cu.imv = value;

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif
  if( sps->getAMVREnabledFlag() && value )
  {
    if (!CU::isIBC(cu))
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      value = m_BinDecoder.decodeBin(imvCtx(4));
#else
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(4));
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4);
      cu.imv = value ? 1 : IMV_HPEL;
    }
    if (value)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      value = m_BinDecoder.decodeBin(imvCtx(1));
#else
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(1));
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1);
      value++;
      cu.imv = value;
    }
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (CU::isIBC(cu))
  {
    CHECK(pu.cs->sps->getIBCFracFlag() && cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
    CHECK(!useIBCFrac && (cu.imv == IMV_OFF || cu.imv == IMV_HPEL), "Fractiona IBC is not enabled to support fractional BVD coding");
  }
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}
void CABACReader::affine_amvr_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AFFINE_AMVR, cu.lumaSize());

  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );

  if( value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACReader::separate_tree_cu_flag( CodingUnit& cu, Partitioner& partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );
  unsigned ctxId       = 0;
  bool inferredSeparateTreeFlag = false;
  if ( cu.slice->getSeparateTreeEnabled() && CU::isIntra( cu ) && !cu.cs->slice->getProcessingIntraRegion() )
  {
    bool canSplit = partitioner.canSplit(CU_QUAD_SPLIT, *cu.cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );
    canSplit = canSplit || partitioner.canSplit( CU_MT_SPLIT, *cu.cs 
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );

    int separateTreeFlag = -1;
    cu.cs->deriveSeparateTreeFlagInference( separateTreeFlag, inferredSeparateTreeFlag, partitioner.currArea().lumaSize().width, partitioner.currArea().lumaSize().height, canSplit );
    if ( inferredSeparateTreeFlag )
    {
      CHECK( separateTreeFlag == -1, "Invalid separate tree flag value\n" );
      cu.separateTree = ( separateTreeFlag ) ? true : false;
    }
    else
    {
#if JVET_AG0117_CABAC_SPATIAL_TUNING && JVET_AI0136_ADAPTIVE_DUAL_TREE
      // If context data collection is active and if on bottom of the CTU, start the counters
      if ( m_BinDecoder.getBinBuffer() )
      {
        m_BinDecoder.setBinBufferActive( CU::isOnCtuBottom( cu ) );
      }
#endif

      ctxId = DeriveCtx::CtxCUSeparateTree(*cu.cs, partitioner);
      cu.separateTree = m_BinDecoder.decodeBin( Ctx::SeparateTree(ctxId) );

#if JVET_AG0117_CABAC_SPATIAL_TUNING && JVET_AI0136_ADAPTIVE_DUAL_TREE
      // Done with the data collection for this CU
      if ( m_BinDecoder.getBinBuffer() )
      {
        m_BinDecoder.setBinBufferActive( false );
      }
#endif
    }
    cu.isSST = true;
    DTRACE( g_trace_ctx, D_SYNTAX, "separate_tree_cu_flag() pred_mode=%d, separateTree=%d, ctxId=%d, inferredSeparateTreeFlag=%d, processingIntraRegion=%d, (x=%d, y=%d, w=%d, h=%d)\n", cu.predMode ? 1 : 0, cu.separateTree, ctxId, inferredSeparateTreeFlag ? 1 : 0, cu.cs->slice->getProcessingIntraRegion() ? 1: 0, cu.lumaPos().x, cu.lumaPos().y, partitioner.currArea().lumaSize().width, partitioner.currArea().lumaSize().height );
  }
}
#endif


void CABACReader::pred_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__PRED_MODE, cu.block((!CS::isDualITree(*cu.cs) || isLuma(cu.chType))? COMPONENT_Y: COMPONENT_Cb).lumaSize(), (!CS::isDualITree(*cu.cs) || isLuma(cu.chType)) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (cu.cs->slice->getUseIBC() && cu.chType != CHANNEL_TYPE_CHROMA)
#else
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#endif
#endif
    {
      cu.predMode = MODE_INTRA;

      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
      }
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
      {
        cu.predMode = MODE_INTRA;
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
        {
          if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
          {
            cu.predMode = MODE_PLT;
          }
        }
      }
      else
      {
        cu.predMode = MODE_INTER;

        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
          unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
          if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
          {
            cu.predMode = MODE_IBC;
          }
        }
      }
    }
  }
  else
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4) || cu.isConsIntra() )
#endif
#endif
    {
      cu.predMode = MODE_INTRA;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  )  )
#endif
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      cu.predMode = m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))) ? MODE_INTRA : MODE_INTER;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if (CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  )  )
#endif
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pred_mode() pred_mode=%d\n", cu.predMode);
}

void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{
  if (!CU::bdpcmAllowed(cu, compID))
  {
    if (isLuma(compID))
    {
      cu.bdpcmMode = 0;
      if (!CS::isDualITree(*cu.cs))
      {
        cu.bdpcmModeChroma = 0;
      }
    }
    else
    {
      cu.bdpcmModeChroma = 0;
    }
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__BDPCM_MODE, cu.block(compID).lumaSize(), compID );

  int bdpcmMode;
  unsigned ctxId = isLuma( compID ) ? 0 : 2;
  bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId) );
  if (bdpcmMode)
  {
    bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId+1) );
  }
  if (isLuma(compID))
  {
    cu.bdpcmMode = bdpcmMode;
  }
  else
  {
    cu.bdpcmModeChroma = bdpcmMode;
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_LUMA, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_CHROMA, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma);
  }
}

void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
    {
      bdpcm_mode(cu, COMPONENT_Y );
    }
    intra_luma_pred_modes( cu );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if ((!cu.Y().valid() || (!CS::isDualITree(*cu.cs) && cu.Y().valid())) && isChromaEnabled(cu.chromaFormat))
#else
    if( ( !cu.Y().valid() || (!cu.isSepTree() && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
#endif
    {
      bdpcm_mode(cu, ComponentID(CHANNEL_TYPE_CHROMA));
    }
#if JVET_AK0076_EXTENDED_OBMC_IBC
    cu.obmcFlag = isLuma(cu.chType) && cu.lumaSize().area() >= 32;
#endif
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }
  MergeCtx mrgCtx;
  for( auto &pu : CU::traversePUs( cu ) )
  {
#if JVET_AD0140_MVD_PREDICTION
    pu.mvdSuffixInfo.clear();
#endif
    prediction_unit( pu, mrgCtx );
  }

  imv_mode   ( cu, mrgCtx );
  affine_amvr_mode( cu, mrgCtx );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  for (auto &pu : CU::traversePUs(cu))
  {
    mvsd_data(pu);
  }
#endif
#if INTER_LIC
  cu_lic_flag( cu ); // local illumination compensation
#endif
#if !(JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION) || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION))
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION)
  if (!cu.cs->sps->getUseMvdPred())
  {
#endif
  cu_bcw_flag(cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION)
  }
#endif
#endif
#if ENABLE_OBMC
  obmc_flag  ( cu );
#endif
}

void CABACReader::cu_bcw_flag(CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return;
  }
#endif

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__BCW_IDX, cu.lumaSize());

  uint32_t idx = 0;

  uint32_t symbol = m_BinDecoder.decodeBin(Ctx::BcwIdx(0));

  int32_t numBcw = (cu.slice->getCheckLDC()) ? 5 : 3;
  if(symbol == 1)
  {
    uint32_t prefixNumBits = numBcw - 2;
    uint32_t step = 1;

    idx = 1;

    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      symbol = m_BinDecoder.decodeBinEP();
      if (symbol == 0)
      {
        break;
      }
      idx += step;
    }
  }

  uint8_t bcwIdx = (uint8_t)g_bcwParsingOrder[idx];
  CU::setBcwIdx(cu, bcwIdx);

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.bcwIdx ? 1 : 0);
#if MULTI_HYP_PRED
  mh_pred_data(*cu.firstPU);
#endif
}

#if ENABLE_OBMC
void CABACReader::obmc_flag(CodingUnit& cu)
{
#if JVET_AK0076_EXTENDED_OBMC_IBC
  if (!cu.cs->sps->getUseOBMC() || cu.predMode == MODE_INTRA
#else
  if (!cu.cs->sps->getUseOBMC() || CU::isIBC(cu) || cu.predMode == MODE_INTRA
#endif
#if INTER_LIC && !JVET_AD0213_LIC_IMP
    || cu.licFlag
#endif
#if JVET_AK0076_EXTENDED_OBMC_IBC
    || cu.rribcFlipType != 0
#if JVET_AC0112_IBC_LIC && !JVET_AD0213_LIC_IMP
    || cu.ibcLicFlag
#endif
#endif
    || cu.lwidth() * cu.lheight() < 32
    )
  {
    cu.obmcFlag = false;
    return;
  }
#if JVET_AK0076_EXTENDED_OBMC_IBC
  if (CU::isIBC(cu))
  {
    cu.obmcFlag = true;
    return;
  }
#endif
  if (cu.firstPU->mergeFlag)
  {
    cu.obmcFlag = true;
    return;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    cu.obmcFlag = true;
    return;
  }
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__OBMC, cu.lumaSize());

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool ctxCond = cu.affine == false || cu.firstPU->addHypData.size() > 0;
  cu.obmcFlag = (bool)m_BinDecoder.decodeBin(ctxCond ? Ctx::ObmcFlag(0) : Ctx::ObmcFlag(1));
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) ctx=%d obmc_flag=%d\n", cu.lx(), cu.ly(), ctxCond ? 0 : 1, cu.obmcFlag);
#else
  cu.obmcFlag = (bool)m_BinDecoder.decodeBin(Ctx::ObmcFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) obmc_flag=%d\n", cu.lx(), cu.ly(), cu.obmcFlag);
#endif
}
#endif

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line(CodingUnit& cu)
{
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode
#if ENABLE_DIMD
    || cu.dimd
#endif
#if JVET_AJ0146_TIMDSAD
    || (cu.timd && cu.timdSad)
#endif
#if JVET_AB0155_SGPM
    || cu.sgpm
#endif
#if JVET_AK0059_MDIP
    || cu.mdip
#endif
    )
  {
    cu.firstPU->multiRefIdx = 0;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MULTI_REF_LINE, cu.lumaSize());

  const int numBlocks = CU::getNumPUs(cu);
  PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    if( !cu.cs->sps->getUseMRL() )
    {
      pu->multiRefIdx = 0;
      pu = pu->next;
      continue;
    }
#if JVET_AH0065_RELAX_LINE_BUFFER
    bool isFirstLineOfCtu = cu.block(COMPONENT_Y).y == 0;
#else
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
#endif
    if (isFirstLineOfCtu)
    {
      pu->multiRefIdx = 0;
      continue;
    }
    int multiRefIdx = 0;

#if JVET_Y0116_EXTENDED_MRL_LIST
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(5) : Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#else
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(6) : Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#else
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#endif
        if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1]
#if JVET_W0123_TIMD_FUSION
        && !cu.timd
#endif
          )
        {
          multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(2)) == 1 ? MULTI_REF_LINE_IDX[3] : MULTI_REF_LINE_IDX[2];
          if (MRL_NUM_REF_LINES > 4 && multiRefIdx != MULTI_REF_LINE_IDX[2])
          {
            multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(3)) == 1 ? MULTI_REF_LINE_IDX[4] : MULTI_REF_LINE_IDX[3];
            if (MRL_NUM_REF_LINES > 5 && multiRefIdx != MULTI_REF_LINE_IDX[3])
            {
              multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(4)) == 1 ? MULTI_REF_LINE_IDX[5] : MULTI_REF_LINE_IDX[4];
            }
          }
        }
      }
    }
#else
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(2) : Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#else
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(3) : Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#else
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#endif
      }

    }
#endif
    pu->multiRefIdx = multiRefIdx;
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
    pu = pu->next;
  }
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
#if JVET_AM0074_INTRA_MERGE
  if ((cu.intraMergeMode > 0) && !cu.slice->getPnnMode())
  {
    return;
  }
#endif
#if JVET_AH0076_OBIC
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.obicFlag && !cu.slice->getPnnMode())
#else
  if (cu.obicFlag)
#endif
  {
    return;
  }
#endif
#if JVET_AL0108_BVG_DIMD
  if (cu.bvgDimdFlag)
  {
    return;
  }
#endif
  if( cu.bdpcmMode )
  {
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
    return;
  }
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  cu_pnn_flag(cu);
  if ((cu.firstPU)->intraDir[CHANNEL_TYPE_LUMA] == PNN_IDX)
  {
    return;
  }
#if ENABLE_DIMD
  if (cu.slice->getPnnMode())
  {
    cu_dimd_flag(cu);
  }
  if (cu.dimd)
  {
    return;
  }
#endif
#endif
#if JVET_V0130_INTRA_TMP
  int tmpMaxSize=cu.cs->sps->getIntraTMPMaxSize();
  if (cu.lwidth() <= tmpMaxSize && cu.lheight() <= tmpMaxSize)
  {
	  tmp_flag(cu);
    if( cu.tmpFlag )
    {
      return;
    }
  }
  else
  {
    cu.tmpFlag = 0;
  }
#endif
  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
#if JVET_W0123_TIMD_FUSION
  cu_timd_flag(cu);
#if JVET_AJ0061_TIMD_MERGE
  if (cu.timdMrg)
  {
    return;
  }
#endif
#endif
#if JVET_AG0058_EIP
  cu_eip_flag(cu);
  if (cu.eipFlag)
  {
    return;
  }
#endif
#if JVET_AK0059_MDIP
  mdip_flag(cu);
#endif
#if JVET_AB0155_SGPM
  sgpm_flag(cu);
  if (cu.sgpm)
  {
    return;
  }
#endif
#if JVET_AD0082_TMRL_CONFIG
  if (CU::allowTmrl(cu))
  {
    cuTmrlFlag(cu);
    if (cu.tmrlFlag)
    {
      return;
    }
  }
  else
  {
    extend_ref_line(cu);
  }
#else
#if JVET_AB0157_TMRL
  cuTmrlFlag(cu);
  if (cu.tmrlFlag)
  {
    return;
  }
#else
  extend_ref_line( cu );
#endif
#endif
  isp_mode( cu );
#if ENABLE_DIMD && !JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.dimd)
  {
    return;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    return;
  }
#endif
#if JVET_AK0059_MDIP
  if (cu.mdip)
  {
    return;
  }
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA );

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs( cu );
  int mpmFlag[4];
  for( int k = 0; k < numBlocks; k++ )
  {
    CHECK(numBlocks != 1, "not supported yet");
    if ( cu.firstPU->multiRefIdx )
    {
      mpmFlag[0] = true;
    }
    else
    {
      mpmFlag[k] = m_BinDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
    }
  }

  PredictionUnit *pu = cu.firstPU;

#if !JVET_AJ0249_NEURAL_NETWORK_BASED
#if !SECONDARY_MPM
  unsigned int mpmPred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
#endif
#endif

  for( int k = 0; k < numBlocks; k++ )
  {
#if !JVET_AJ0249_NEURAL_NETWORK_BASED
#if !JVET_AD0085_TMRL_EXTENSION
#if SECONDARY_MPM
    PU::getIntraMPMs( *pu, mpmPred, nonMpmPred
#if JVET_AC0094_REF_SAMPLES_OPT
                     , false
#endif
    );
#else
    PU::getIntraMPMs(*pu, mpmPred);
#endif
#endif
#endif
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
    pu->parseLumaMode = true;
    pu->mpmFlag = mpmFlag[k];
#endif
    if( mpmFlag[k] )
    {
      uint32_t ipredIdx = 0;

      unsigned ctx = ( pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0 );
#if SECONDARY_MPM
      unsigned ctx2 = ( ctx ? ( cu.firstPU->multiRefIdx == 0 ? 2 : 1 ) : 0 );
#endif
#if JVET_AL0125_IMPROVEMENT_ON_MPM
      bool plannar = false;
      if (pu->cu->slice->getPnnMode() && IntraPredictionNN::hasPnnPrediction(cu))
      {
        plannar = true;
      }
#endif
      if( pu->multiRefIdx == 0 )
      {
#if JVET_AL0125_IMPROVEMENT_ON_MPM
        if (plannar)
        {
          ipredIdx = m_BinDecoder.decodeBin(Ctx::IntraLumaMPMfistIdxFlag());
        }
        else
        {
          ipredIdx = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(ctx));
        }
#else
        ipredIdx = m_BinDecoder.decodeBin( Ctx::IntraLumaPlanarFlag( ctx ) );
#endif
      }
      else
      {
        ipredIdx = 1;
      }

      if( ipredIdx )
      {
#if SECONDARY_MPM
        ipredIdx += m_BinDecoder.decodeBin( Ctx::IntraLumaMPMIdx( 0 + ctx2 ) );
#else
        ipredIdx += m_BinDecoder.decodeBinEP();
#endif
      }
      if( ipredIdx > 1 )
      {
        ipredIdx += m_BinDecoder.decodeBinEP();
      }
      if( ipredIdx > 2 )
      {
        ipredIdx += m_BinDecoder.decodeBinEP();
      }
      if( ipredIdx > 3 )
      {
        ipredIdx += m_BinDecoder.decodeBinEP();
      }

      pu->secondMpmFlag = false;
      pu->ipredIdx = ipredIdx;
    }
    else
    {
      unsigned ipredMode = 0;

#if SECONDARY_MPM
      if( m_BinDecoder.decodeBin( Ctx::IntraLumaSecondMpmFlag() ) )
      {
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
#if JVET_AD0085_MPM_SORTING
        int idx = 0;
        if (cu.cs->sps->getUseMpmSorting())
        {
          const int maxNumCtxBins = (NUM_SECONDARY_MOST_PROBABLE_MODES / 4) - 1;
          int prefixIdx = 0;
          int ctxId = 0;
          for (int val = 0; val < maxNumCtxBins; val++)
          {
            unsigned int bin = m_BinDecoder.decodeBin(Ctx::IntraLumaSecondMpmIdx(ctxId++));
            prefixIdx += bin;
            if (!bin)
            {
              break;
            }
          }

          idx = m_BinDecoder.decodeBinsEP(2) + prefixIdx * 4;
          idx += NUM_PRIMARY_MOST_PROBABLE_MODES;
        }
        else
        {
          idx = m_BinDecoder.decodeBinsEP(4) + NUM_PRIMARY_MOST_PROBABLE_MODES;
        }
#else
        int idx = m_BinDecoder.decodeBinsEP( 4 ) + NUM_PRIMARY_MOST_PROBABLE_MODES;
#endif
        pu->ipredIdx = idx;
#else
        pu->ipredIdx = m_BinDecoder.decodeBinsEP( 4 ) + NUM_PRIMARY_MOST_PROBABLE_MODES;
#endif
        pu->secondMpmFlag = true;
      }
      else
      {
#if JVET_AK0059_MDIP
        if (cu.cs->sps->getUseMdip() && (cu.cs->sps->getUseDimd() || (!cu.cs->sps->getUseDimd() && CU::allowMdip(cu))))
        {
          const int numNonMpm = CU::allowMdip(cu) ? NUM_NON_MPM_MODES : NUM_NON_MPM_MODES + MDIP_NUM;
          xReadTruncBinCode( ipredMode, numNonMpm );
        }
        else
        {
          xReadTruncBinCode( ipredMode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES );
        }
#else
        xReadTruncBinCode( ipredMode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES );
#endif
        pu->secondMpmFlag = false;
        pu->ipredIdx = ipredMode;
      }
#else
      xReadTruncBinCode( ipredMode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES );
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
      pu->ipredIdx = ipredMode;
#endif
#endif
#if !SECONDARY_MPM && !JVET_AJ0249_NEURAL_NETWORK_BASED
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpmPred, mpmPred + NUM_MOST_PROBABLE_MODES );

      for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        ipredMode += (ipredMode >= mpmPred[i]);
      }
#endif

#if !JVET_AJ0249_NEURAL_NETWORK_BASED
      pu->intraDir[0] = ipredMode;
#endif
    }

#if JVET_AC0105_DIRECTIONAL_PLANAR
#if JVET_AK0061_PDP_MPM
    const bool& enablePlanarSort = PU::determinePDPTemp(*pu);
#if JVET_AL0125_IMPROVEMENT_ON_MPM
    bool planarDisable = false;
    if (cu.slice->getPnnMode() && IntraPredictionNN::hasPnnPrediction(cu))
    {
      planarDisable = true;
    }
    if (CU::isDirectionalPlanarAvailable(cu) && !(enablePlanarSort || planarDisable) && pu->ipredIdx == 0 && mpmFlag[k])
#else
    if (CU::isDirectionalPlanarAvailable(cu) && !enablePlanarSort && pu->ipredIdx == 0 && mpmFlag[k])
#endif
#else
    if (CU::isDirectionalPlanarAvailable(cu) && pu->ipredIdx == 0 && mpmFlag[k])
#endif
    {
      uint8_t plIdx = 0;
      plIdx         = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(2));
      if (plIdx)
      {
        plIdx += m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(3));
#if JVET_AK0061_PDP_MPM
        pu->parseLumaMode = false;
#endif
      }
      cu.plIdx = plIdx;
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) pl_idx=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, cu.plIdx);
    }
    else
    {
      cu.plIdx = 0;
    }
#endif

#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) predIdx=%d mpm=%d secondmpm=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->ipredIdx, pu->mpmFlag, pu->secondMpmFlag);
#else
    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) predIdx=%d mpm=%d secondmpm=%d \n", k, pu->lumaPos().x, pu->lumaPos().y, pu->ipredIdx, pu->mpmFlag, pu->secondMpmFlag);
#endif
    pu = pu->next;
  }
}
#if JVET_AJ0249_NEURAL_NETWORK_BASED
void CABACReader::cu_pnn_flag(CodingUnit& cu)
{
  if (!cu.Y().valid())
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_PNN_FLAG, cu.lumaSize(), CHANNEL_TYPE_LUMA );
  if (cu.slice->getPnnMode() && IntraPredictionNN::hasPnnPrediction(cu))
  {
    const uint16_t ctxId = DeriveCtx::CtxPnnLuminanceFlag(cu);
    if (m_BinDecoder.decodeBin(Ctx::PnnLuminanceFlag(ctxId)))
    {
      (cu.firstPU)->intraDir[CHANNEL_TYPE_LUMA] = PNN_IDX;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_pnn_flag() ctx=%d pos=(%d,%d) pnn=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, (cu.firstPU)->intraDir[CHANNEL_TYPE_LUMA] == PNN_IDX ? 1 : 0);
  }
}
#endif
#if JVET_AM0074_INTRA_MERGE
void CABACReader::cu_derived_intra_flag(CodingUnit& cu)
{
  cu.dimd = m_BinDecoder.decodeBin(Ctx::GeneralIntraDerivationFlag(0));

  if (cu.dimd)
  {
    cu.intraMergeMode = 0;
    if (CU::allowIntraMergeMode(cu))
    {
      cu.allowIntraMergeMode = 2;
      for (int i = 0; i < MAX_NUM_INTRA_MERGE_CAND; i++)
      {
        if (m_BinDecoder.decodeBin(Ctx::IntraMergeModeFlag(i)))
        {
          cu.intraMergeMode++;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      cu.allowIntraMergeMode = 1;
    }
#if JVET_AH0076_OBIC
    if (!cu.intraMergeMode)
    {
      cu_obic_flag(cu);
    }
#endif
  }
}
#endif
#if ENABLE_DIMD
void CABACReader::cu_dimd_flag(CodingUnit& cu)
{
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || !cu.slice->getSPS()->getUseDimd())
  {
    cu.dimd = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIMD, cu.lumaSize(), COMPONENT_Y);

  unsigned ctxId = DeriveCtx::CtxDIMDFlag(cu);
  cu.dimd = m_BinDecoder.decodeBin(Ctx::DimdFlag(ctxId));
#if JVET_AL0108_BVG_DIMD
  cu.obicFlag = false;
  cu.bvgDimdFlag = false;
  if(!cu.dimd)
  {
    return;
  }
  const bool bvgDimdAvail = PU::isBvgDimdAvail(*cu.firstPU);
  if (bvgDimdAvail)
  {
    const bool bDimd = m_BinDecoder.decodeBin(Ctx::RegularDimdFlag(0));
    if (bDimd)
    {
      return;
    }
#if JVET_AM0074_INTRA_MERGE
    cu.intraMergeMode = 0;
    if (CU::allowIntraMergeMode(cu))
    {
      cu.allowIntraMergeMode = 2;
      for (int i = 0; i < MAX_NUM_INTRA_MERGE_CAND; i++)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MERGE, cu.lumaSize(), COMPONENT_Y);

        if (m_BinDecoder.decodeBin(Ctx::IntraMergeModeFlag(i)))
        {
          cu.intraMergeMode++;
        }
        else
        {
          break;
        }
      }
      if (cu.intraMergeMode)
      {
        return;
      }
    }
    else
    {
      cu.allowIntraMergeMode = 1;
    }
#else
    // isObicAvail is already done in cu_obic_flag(cu);
    const bool bObicAvail = PU::isObicAvail(*cu.firstPU);
    if (bObicAvail)
#endif
    {
      cu_obic_flag(cu);
      cu.bvgDimdFlag = !cu.obicFlag;
    }
#if !JVET_AM0074_INTRA_MERGE
    else
    {
      cu.bvgDimdFlag = true;
    }
#endif
  }
  else
  {
    cu.bvgDimdFlag = false;
#endif
#if JVET_AM0074_INTRA_MERGE
    cu_derived_intra_flag(cu);
#else
#if JVET_AH0076_OBIC
  cu_obic_flag(cu);
#endif
#endif
#if JVET_AL0108_BVG_DIMD
  }
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  if (cu.bvgDimdFlag)
  {
    cu.tempType = CU::getRefTemplateType(cu);
  }
#endif
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_dimd_flag() ctx=%d pos=(%d,%d) dimd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.dimd);
}
#endif

#if JVET_AH0076_OBIC
void CABACReader::cu_obic_flag(CodingUnit& cu )
{
  cu.obicFlag = false;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_OBIC, cu.lumaSize(), COMPONENT_Y);

#if JVET_AJ0249_NEURAL_NETWORK_BASED
  if (!cu.Y().valid() || !cu.dimd)
#else
  if (!cu.dimd || !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
#endif
  {
    return;
  }
  if (!PU::isObicAvail(*cu.firstPU))
  {
    return;
  }
  cu.obicFlag = m_BinDecoder.decodeBin( Ctx::obicFlag(0) );
}
#endif

#if JVET_AG0058_EIP
void CABACReader::cu_eip_flag(CodingUnit& cu)
{
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  if (!cu.Y().valid() || cu.timd)
#else
  if (cu.timd || cu.dimd || !cu.Y().valid() || !isLuma(cu.chType))
#endif
  {
    cu.eipFlag = false;
    return;
  }

  const bool bCanUseEip = getAllowedEip(cu, COMPONENT_Y) || getAllowedEipMerge(cu, COMPONENT_Y);
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_EIP, cu.lumaSize(), COMPONENT_Y);

  if (bCanUseEip)
  {
    cu.eipFlag = m_BinDecoder.decodeBin(Ctx::EipFlag(0));
    if (cu.eipFlag)
    {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      cu.tempType = CU::getRefTemplateType(cu);
#endif
#if JVET_AL0106_BV_EIP
      if (getAllowedEip(cu, COMPONENT_Y) && getAllowedEipMerge(cu, COMPONENT_Y))
      {
        cu.eipMerge = m_BinDecoder.decodeBin(Ctx::EipFlag(1));
      }
      else if (getAllowedEipMerge(cu, COMPONENT_Y))
      {
        cu.eipMerge = true;
      }
      else
      {
        cu.eipMerge = false;
      }

      if (cu.eipMerge)
      {
        cu.firstPU->intraDir[0] = unary_max_eqprob(NUM_EIP_MERGE_SIGNAL - 1);
      }
      else
      {
        unsigned int symbol = 0;
        static_vector<EIPInfo, NUM_DERIVED_EIP> eipInfoList;
#if JVET_AJ0082_MM_EIP
        cu.eipMmFlag = m_BinDecoder.decodeBin(Ctx::EipFlag(2));
        if (!cu.eipMmFlag && allowBvEip(cu))
        {
          cu.bvEip = m_BinDecoder.decodeBin(Ctx::EipFlag(3));
          if (cu.bvEip)
          {
            cu.firstPU->intraDir[0] = 0;
            return;
          }
        }

        int eipNum = getAllowedCurEip(cu, COMPONENT_Y, eipInfoList, cu.eipMmFlag);
        if (eipNum > 1)
        {
          xReadTruncBinCode(symbol, eipNum);
        }
#else
#if JVET_AL0106_BV_EIP
        if (allowBvEip(cu))
        {
          cu.bvEip = m_BinDecoder.decodeBin(Ctx::EipFlag(2));
          if (cu.bvEip)
          {
            cu.firstPU->intraDir[0] = 0;
            return;
          }
        }
#endif
        xReadTruncBinCode(symbol, getAllowedCurEip(cu, COMPONENT_Y, eipInfoList));
#endif
        cu.firstPU->intraDir[0] = symbol;
      }
#else
      if (getAllowedEip(cu, COMPONENT_Y) && getAllowedEipMerge(cu, COMPONENT_Y))
      {
        cu.eipMerge = m_BinDecoder.decodeBin(Ctx::EipFlag(1));
      }
      else if (getAllowedEipMerge(cu, COMPONENT_Y))
      {
        cu.eipMerge = true;
      }
      else
      {
        cu.eipMerge = false;
      }
      if(cu.eipMerge)
      {
        cu.firstPU->intraDir[0] = unary_max_eqprob(NUM_EIP_MERGE_SIGNAL - 1);
      }
      else
      {
        unsigned int symbol = 0;
        static_vector<EIPInfo, NUM_DERIVED_EIP> eipInfoList;
#if JVET_AJ0082_MM_EIP
        cu.eipMmFlag = m_BinDecoder.decodeBin(Ctx::EipFlag(2));
        int eipNum = getAllowedCurEip(cu, COMPONENT_Y, eipInfoList, cu.eipMmFlag);
        if (eipNum > 1)
        {
          xReadTruncBinCode(symbol, eipNum);
        }
#else
        xReadTruncBinCode(symbol, getAllowedCurEip(cu, COMPONENT_Y, eipInfoList));
#endif
        cu.firstPU->intraDir[0] = symbol;
      }
#endif
    }
  }
  else
  {
    cu.eipFlag = false;
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "eip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.eipFlag ? 1 : 0);
}
#endif

#if JVET_AK0059_MDIP
void CABACReader::mdip_flag(CodingUnit& cu)
{
  cu.mdip = false;
  if(!cu.cs->sps->getUseMdip())
  {
    return;
  }
#if ENABLE_DIMD && !JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.dimd)
  {
    cu.mdip = false;
    return;
  }
#endif
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType)
#if JVET_W0123_TIMD_FUSION
      || cu.timd
#endif
   )
  {
    cu.mdip = false;
    return;
  }
  
  if (CU::allowMdip(cu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MDIP, cu.lumaSize(), COMPONENT_Y);

    cu.mdip = m_BinDecoder.decodeBin( Ctx::MdipFlag() );
  }
}
#endif

#if JVET_W0123_TIMD_FUSION
void CABACReader::cu_timd_flag( CodingUnit& cu )
{
  if (!cu.cs->sps->getUseTimd())
  {
    cu.timd = false;
    return;
  }
  if (cu.lwidth() * cu.lheight() > 1024 && cu.slice->getSliceType() == I_SLICE)
  {
    cu.timd = false;
    return;
  }
#if ENABLE_DIMD && !JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.dimd)
  {
    cu.timd = false;
    return;
  }
#endif
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    cu.timd = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_TIMD, cu.lumaSize(), COMPONENT_Y);

  unsigned ctxId = DeriveCtx::CtxTimdFlag( cu );
  cu.timd = m_BinDecoder.decodeBin( Ctx::TimdFlag(ctxId) );
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_timd_flag() ctx=%d pos=(%d,%d) timd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.timd);

#if JVET_AJ0146_TIMDSAD
  if (cu.timd && CU::allowTimdSad(cu))
  {
    cu.timdSad = m_BinDecoder.decodeBin( Ctx::TimdFlagSad() );

    DTRACE( g_trace_ctx, D_SYNTAX, "cu_timd_flag() ctx=%d pos=(%d,%d) timdSad=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.timdSad );
  }
  else
  {
    cu.timdSad = false;
  }
#endif
#if JVET_AJ0061_TIMD_MERGE
  cu_timd_merge_flag(cu);
#endif
}

#if JVET_AJ0061_TIMD_MERGE
void CABACReader::cu_timd_merge_flag( CodingUnit& cu)
{
  cu.timdMrg = 0;
  if (!cu.timd || cu.dimd)
  {
    return;
  }
#if JVET_AJ0146_TIMDSAD
  if (cu.timdSad)
  {
    return;
  }
#endif  
  if (!PU::canTimdMerge(*cu.firstPU))
  {
    return;
  }
  int ctxId = cu.lwidth() * cu.lheight() >= 64 ? 0 : 1;
  cu.timdMrg = m_BinDecoder.decodeBin( Ctx::TimdMrgFlag(ctxId) ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_timd_merge_flag() ctx=%d pos=(%d,%d) timdMrg=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.timdMrg );
}
#endif
#endif

#if JVET_AB0155_SGPM
void CABACReader::sgpm_flag(CodingUnit &cu)
{
  if (!cu.cs->sps->getUseSgpm())
  {
    cu.sgpm = false;
    return;
  }
  if (!(cu.lwidth() >= GEO_MIN_CU_SIZE_EX && cu.lheight() >= GEO_MIN_CU_SIZE_EX && cu.lwidth() <= GEO_MAX_CU_SIZE_EX
        && cu.lheight() <= GEO_MAX_CU_SIZE_EX && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth()
        && cu.lwidth() * cu.lheight() >= SGPM_MIN_PIX))
  {
    cu.sgpm = false;
    return;
  }

  if( cu.mipFlag
#if ENABLE_DIMD && !JVET_AJ0249_NEURAL_NETWORK_BASED
    || cu.dimd
#endif
#if JVET_W0123_TIMD_FUSION
    || cu.timd
#endif
#if JVET_V0130_INTRA_TMP
    || cu.tmpFlag
#endif
#if JVET_AK0059_MDIP
    || cu.mdip
#endif
    )
  {
    cu.sgpm = false;
    return;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    cu.sgpm = false;
    return;
  }
  if (!(cu.lx() && cu.ly()))
  {
    cu.sgpm = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_SGPM, cu.lumaSize(), COMPONENT_Y);


  unsigned ctxId = DeriveCtx::CtxSgpmFlag(cu);
  cu.sgpm        = m_BinDecoder.decodeBin(Ctx::SgpmFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_flag() pos=(%d,%d) ctx=%d sgpm_flag=%d\n", cu.lumaPos().x, cu.lumaPos().y, ctxId, cu.sgpm);

  if (cu.sgpm)
  {
    uint32_t sgpmIdx = 0;
    xReadTruncBinCode(sgpmIdx, SGPM_NUM);
    cu.sgpmIdx = sgpmIdx;
    DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_flag() pos=(%d,%d) sgpm_idx=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.sgpmIdx);
  }
}
#endif

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA))
#else
  if( cu.chromaFormat == CHROMA_400 || ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA ) )
#endif
  {
    return;
  }

  PredictionUnit* pu = cu.firstPU;
  if( cu.bdpcmModeChroma )
  {
    pu->intraDir[1] = cu.bdpcmModeChroma == 2 ? VER_IDX : HOR_IDX;
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
      pu->blocks[CHANNEL_TYPE_CHROMA].x, pu->blocks[CHANNEL_TYPE_CHROMA].y, pu->intraDir[CHANNEL_TYPE_CHROMA]);
    return;
  }

  CHECK(pu->cu != &cu, "Inconsistent PU-CU mapping");
  intra_chroma_pred_mode(*pu);
  DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) fusion_mode=%d cccm_mode=%d\n",
    pu->blocks[CHANNEL_TYPE_CHROMA].x, pu->blocks[CHANNEL_TYPE_CHROMA].y, pu->isChromaFusion, pu->cccmFlag);
}

#if JVET_Z0050_CCLM_SLOPE
void CABACReader::cclmDelta(PredictionUnit& pu, int8_t &delta)
{
  if ( delta )
  {
    int flag = 1;
    
    while (flag && delta < 4)
    {
      flag   = m_BinDecoder.decodeBinEP();
      delta += flag;
    }

    delta *= m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(4)) ? -1 : +1;
  }
}

void CABACReader::cclmDeltaSlope(PredictionUnit& pu)
{
#if JVET_AA0057_CCCM
  if ( pu.cccmFlag )
  {
    return;
  }
#endif

  if ( PU::hasCclmDeltaFlag( pu ) )
  {
    bool deltaActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(0));

    if ( deltaActive )
    {
      bool bothActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

      pu.cclmOffsets.cb0 = bothActive;
      pu.cclmOffsets.cr0 = bothActive;
      
      if ( !bothActive )
      {
        pu.cclmOffsets.cb0 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

#if MMLM
        if ( PU::isMultiModeLM( pu.intraDir[1] ) && !pu.cclmOffsets.cb0 )
        {
          pu.cclmOffsets.cr0 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
        }
        else
#endif
        {
          pu.cclmOffsets.cr0 = !pu.cclmOffsets.cb0;
        }
      }

      cclmDelta( pu, pu.cclmOffsets.cb0 );
      cclmDelta( pu, pu.cclmOffsets.cr0 );
    
#if MMLM
      // Now the same for the second model (if applicable)
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        bool bothActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

        pu.cclmOffsets.cb1 = bothActive;
        pu.cclmOffsets.cr1 = bothActive;
        
        if ( !bothActive )
        {
          pu.cclmOffsets.cb1 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

          if ( pu.cclmOffsets.cb1 )
          {
            pu.cclmOffsets.cr1 = 0;
          }
          else if ( pu.cclmOffsets.cb0 == 0 && pu.cclmOffsets.cr0 == 0 && pu.cclmOffsets.cb1 == 0 )
          {
            pu.cclmOffsets.cr1 = 1;
          }
          else
          {
            pu.cclmOffsets.cr1 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
          }
        }

        cclmDelta( pu, pu.cclmOffsets.cb1 );
        cclmDelta( pu, pu.cclmOffsets.cr1 );
      }
#endif
    }
  }
}
#endif

#if JVET_AA0126_GLM
void CABACReader::glmIdc(PredictionUnit& pu)
{
  if ( PU::hasGlmFlag( pu ) )
  {
    bool glmActive = m_BinDecoder.decodeBin(Ctx::GlmFlags(0));

    if ( glmActive )
    {
#if JVET_AB0092_GLM_WITH_LUMA
      pu.glmIdc.cb0 = 1;
#if NUM_GLM_WEIGHT
      pu.glmIdc.cb0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(1)) ? NUM_GLM_PATTERN : 0;
#endif
#if JVET_AA0057_CCCM
      if (m_BinDecoder.decodeBin(Ctx::GlmFlags(2)))
      {
        pu.glmIdc.cb0 += 1;
        if (m_BinDecoder.decodeBin(Ctx::GlmFlags(3)))
        {
          pu.glmIdc.cb0 += 1;
          if (m_BinDecoder.decodeBin(Ctx::GlmFlags(4)))
          {
            pu.glmIdc.cb0 += 1;
          }
        }
      }
#else
      pu.glmIdc.cb0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
#endif
      pu.glmIdc.cr0 = pu.glmIdc.cb0;
#else
      bool bothActive = m_BinDecoder.decodeBin(Ctx::GlmFlags(3));

      pu.glmIdc.cb0 = bothActive;
      pu.glmIdc.cr0 = bothActive;
      
      if ( !bothActive )
      {
        pu.glmIdc.cb0 = m_BinDecoder.decodeBin(Ctx::GlmFlags(1));
        pu.glmIdc.cr0 = !pu.glmIdc.cb0;
      }

      if ( pu.glmIdc.cb0 )
      {
#if NUM_GLM_WEIGHT
        pu.glmIdc.cb0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(2)) ? NUM_GLM_PATTERN : 0;
#endif
        pu.glmIdc.cb0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
      }

      if ( pu.glmIdc.cr0 )
      {
#if NUM_GLM_WEIGHT
        pu.glmIdc.cr0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(4)) ? NUM_GLM_PATTERN : 0;
#endif
        pu.glmIdc.cr0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
      }
#endif

#if MMLM
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        pu.glmIdc.cb1 = pu.glmIdc.cb0;
        pu.glmIdc.cr1 = pu.glmIdc.cr0;
      }
#endif
    }
  }
}
#endif

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
void CABACReader::decoderDerivedCcpModes(PredictionUnit &pu)
{
  pu.decoderDerivedCcpMode = 0;
  if (PU::hasDecoderDerivedCCP(pu))
  {
    pu.decoderDerivedCcpMode = m_BinDecoder.decodeBin(Ctx::decoderDerivedCCP(0));
    pu.intraDir[1] = LM_CHROMA_IDX;
  }
}
#endif

#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
void CABACReader::nonLocalCCPIndex(PredictionUnit &pu)
{
  pu.idxNonLocalCCP = 0;

  if (PU::hasNonLocalCCP(pu))
  {
    pu.idxNonLocalCCP = m_BinDecoder.decodeBin(Ctx::nonLocalCCP(0));
    if (pu.idxNonLocalCCP)
    {
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
      if (pu.cu->slice->getSPS()->getUseDdCcpFusion())
      {
        pu.ddNonLocalCCPFusion = m_BinDecoder.decodeBin(Ctx::ddNonLocalCCP(0));
      }
      if (pu.ddNonLocalCCPFusion)
      {
#if JVET_AN0168_REGRESSION_CCP_FUSION
        const int maxNumFusionCcpModel = PU::getMaxNumCcpFusionModel(pu);
        pu.ddNonLocalCCPFusion += unary_max_eqprob(maxNumFusionCcpModel - 1);
#else
        pu.ddNonLocalCCPFusion += unary_max_eqprob(MAX_CCP_FUSION_NUM - 1);
#endif
      }
      else
      {
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
        if (PU::hasCcpMergeAdjustFlag(pu))
        {
          pu.ccpMergeAdjustFlag = m_BinDecoder.decodeBin( Ctx::ccpMergeAdjust(0));
          if (pu.ccpMergeAdjustFlag)
          {
            pu.ccpMergeAdjustFlag += m_BinDecoder.decodeBin( Ctx::ccpMergeAdjust(1));
          }
        }
        if (pu.ccpMergeAdjustFlag)
        {
          pu.idxNonLocalCCP += unary_max_eqprob(pu.cs->sps->getMaxNumCcpMergeAdjustCand() - 1);
        }
        else
#endif
      pu.idxNonLocalCCP += unary_max_eqprob(MAX_CCP_CAND_LIST_SIZE - 1);
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
      if (PU::hasCCPMergeFusionFlag(pu) && pu.idxNonLocalCCP == 1)
#else
      if (PU::hasCCPMergeFusionFlag(pu))
#endif
      {
        pu.ccpMergeFusionFlag = m_BinDecoder.decodeBin(Ctx::CCPMergeFusionFlag(0));
        if (pu.ccpMergeFusionFlag)
        {
          pu.ccpMergeFusionType = m_BinDecoder.decodeBin(Ctx::CCPMergeFusionType(0));
        }
      }
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
      }
#endif
      pu.cccmFlag    = 0;
      pu.intraDir[1] = LM_CHROMA_IDX;
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    DTRACE(g_trace_ctx, D_SYNTAX, "nonLocalCCPIndex() pos=(%d, %d) idxNonLocalCCP=%d\n", pu.blocks[pu.chType].pos().x, pu.blocks[pu.chType].pos().y, pu.idxNonLocalCCP);
#endif
  }
}
#endif

bool CABACReader::intra_chroma_lmc_mode(PredictionUnit& pu)
{
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  decoderDerivedCcpModes(pu);
  if (pu.decoderDerivedCcpMode)
  {
    return true;
  }
#endif
#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  nonLocalCCPIndex(pu);
  if (pu.idxNonLocalCCP)
  {
    return true;
  }
#endif
#if MMLM
  int lmModeList[NUM_CHROMA_MODE];
#else 
  int lmModeList[10];
#endif
  PU::getLMSymbolList(pu, lmModeList);

  int symbol = m_BinDecoder.decodeBin(Ctx::CclmModeIdx(0));

  if (symbol == 0)
  {
    pu.intraDir[1] = lmModeList[symbol];
    CHECK(pu.intraDir[1] != LM_CHROMA_IDX, "should be LM_CHROMA");
  }
  else
  {
#if MMLM // Modifying signalling may get minor gain
    int modeIdx = 1; // MMLM_Chroma
    modeIdx += m_BinDecoder.decodeBin(Ctx::MMLMFlag(0));

    if (modeIdx > 1) // = 2 means MDLM_L
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    }

    if (modeIdx > 2) // == 3 means MDLM_T
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    }
    if (modeIdx > 3) // == 4 MMLM_L
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    } // == 5 mean MMLM_T
    pu.intraDir[1] = lmModeList[modeIdx];
#else
    symbol += m_BinDecoder.decodeBinEP();
    pu.intraDir[1] = lmModeList[symbol];
#endif
  }
  
#if JVET_AA0057_CCCM
  cccmFlag( pu );
#endif
#if JVET_AA0126_GLM
  glmIdc( pu );
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmDeltaSlope( pu );
#endif
#if JVET_AD0120_LBCCP
  ccInsideFilterFlag(pu);
#endif

  return true; //it will only enter this function for LMC modes, so always return true ;
}

#if JVET_AA0057_CCCM
void CABACReader::cccmFlag(PredictionUnit& pu)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return;
  }
#endif
  const unsigned intraDir = pu.intraDir[1];
  
#if JVET_AE0100_BVGCCCM
  if ((intraDir == LM_CHROMA_IDX || intraDir == MMLM_CHROMA_IDX) && PU::hasBvgCccmFlag(pu) && PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX) && PU::bvgCccmMultiModeAvail(pu, intraDir))
  {
    pu.bvgCccmFlag = m_BinDecoder.decodeBin( Ctx::BvgCccmFlag( 0 ) );
    if (pu.bvgCccmFlag)
    {
      pu.cccmFlag = 1;
      return;
    }
  }
#endif
#if JVET_AB0143_CCCM_TS
  bool isCCCMEnabled = false;
  if (intraDir == LM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX);
  }
  else if (intraDir == MDLM_L_IDX)
  {
    isCCCMEnabled = PU::isLeftCccmMode(pu, MDLM_L_IDX);
  }
  else if (intraDir == MDLM_T_IDX)
  {
    isCCCMEnabled = PU::isTopCccmMode(pu, MDLM_T_IDX);
  }
#if MMLM
  else if (intraDir == MMLM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX);
  }
  else if (intraDir == MMLM_L_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_L_IDX);
  }
  else if (intraDir == MMLM_T_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_T_IDX);
  }
#endif

  if (isCCCMEnabled)
#else
  if ( PU::cccmSingleModeAvail(pu, intraDir) || PU::cccmMultiModeAvail(pu, intraDir) )
#endif
  {
    pu.cccmFlag = m_BinDecoder.decodeBin( Ctx::CccmFlag( 0 ) );
#if JVET_AB0143_CCCM_TS
    if (pu.cccmFlag)
    {
#if MMLM
      pu.cccmFlag = (intraDir == MDLM_T_IDX || intraDir == MMLM_T_IDX) ? 3 : (intraDir == MDLM_L_IDX || intraDir == MMLM_L_IDX) ? 2 : 1;
#else
      pu.cccmFlag = (intraDir == MDLM_T_IDX) ? 3 : (intraDir == MDLM_L_IDX) ? 2 : 1;
#endif
#if JVET_AD0202_CCCM_MDF
      bool isCccmWithMdfEnabled = true;
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
      if (!(intraDir == MMLM_CHROMA_IDX || intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX))
      {
        isCccmWithMdfEnabled = false;
      }
      else
#endif
      if (intraDir == MMLM_CHROMA_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_CHROMA_IDX);
      }
      else if (intraDir == MMLM_L_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_L_IDX);
      }
      else if (intraDir == MMLM_T_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_T_IDX);
      }

      if (isCccmWithMdfEnabled)
      {
        pu.cccmMultiFilterIdx = m_BinDecoder.decodeBin(Ctx::CccmMpfFlag(0));
        if (pu.cccmMultiFilterIdx > 0)
        {
          pu.cccmMultiFilterIdx += m_BinDecoder.decodeBin(Ctx::CccmMpfFlag(1));
          if (pu.cccmMultiFilterIdx > 1)
          {
            pu.cccmMultiFilterIdx += m_BinDecoder.decodeBin(Ctx::CccmMpfFlag(2));
          }
        }
      }
      else
      {
        pu.cccmMultiFilterIdx = 0;
      }

      if (!pu.cccmMultiFilterIdx)
      {
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
      pu.cccmNoSubFlag = 0;
      if ( pu.cs->sps->getUseCccm() == 2 )
      {    
        pu.cccmNoSubFlag += m_BinDecoder.decodeBin( Ctx::CccmFlag( 1 ) );
      }
#endif
#if JVET_AC0054_GLCCCM
      pu.glCccmFlag = 0;
#if !JVET_AC0147_CCCM_NO_SUBSAMPLING
      unsigned ctxId = 1;
#else
      unsigned ctxId = 2;
      if (!pu.cccmNoSubFlag)
#endif
      {
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        if (intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX || intraDir == MMLM_CHROMA_IDX)
        {
#endif
        pu.glCccmFlag = m_BinDecoder.decodeBin( Ctx::CccmFlag( ctxId ) );
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        }
#endif
      }
#endif
#if JVET_AD0202_CCCM_MDF
      }
#endif
    }
#endif
  }
}
#endif

#if JVET_AD0120_LBCCP
void CABACReader::ccInsideFilterFlag(PredictionUnit &pu)
{
#if JVET_AE0100_BVGCCCM
  if (pu.bvgCccmFlag)
  {
    return;
  }
#endif
  const unsigned intraDir = pu.intraDir[1];
  pu.ccInsideFilter       = 0;

  if (PU::hasCcInsideFilterFlag(pu, intraDir))
  {
    pu.ccInsideFilter = m_BinDecoder.decodeBin(Ctx::CcInsideFilterFlag(0));
  }
}
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
void CABACReader::intraChromaFusionMode(PredictionUnit& pu)
{
  int symbol = m_BinDecoder.decodeBin(Ctx::ChromaFusionMode());

  if (symbol > 0)
  {
    symbol += m_BinDecoder.decodeBin(Ctx::ChromaFusionType()); // Default=1

#if MMLM
    if (symbol > 1)
    {
      symbol += m_BinDecoder.decodeBin(Ctx::ChromaFusionCclm());  // LM=2
    }
#endif
  }
  pu.isChromaFusion = symbol;
}
#endif

#if JVET_AJ0081_CHROMA_TMRL
void CABACReader::intraChromaTmrl(PredictionUnit& pu)
{
  pu.chromaTmrlIdx = 0;
  pu.chromaTmrlFlag = bool(m_BinDecoder.decodeBin(Ctx::ChromaTmrlFlag()));
  if (pu.chromaTmrlFlag)
  {
    pu.chromaTmrlIdx = unary_max_eqprob(CHROMA_TMRL_LIST_SIZE - 1);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "intraChromaTmrl() ctx=%d pos=(%d,%d) chromaTmrlFlag=%d chromaTmrlIdx=%d\n", 0, pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.chromaTmrlFlag ? 1 : 0, pu.chromaTmrlIdx);
}
#endif

void CABACReader::intra_chroma_pred_mode(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA);
  if (pu.cu->colorTransform)
  {
    pu.intraDir[CHANNEL_TYPE_CHROMA] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
#if CCLM_LATENCY_RESTRICTION_RMV
  if (pu.cs->sps->getUseLMChroma() )
#else
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
#endif  
  {
    bool isLMCMode = m_BinDecoder.decodeBin(Ctx::CclmModeFlag(0)) ? true : false;
    if (isLMCMode)
    {
      intra_chroma_lmc_mode(pu);
      DTRACE( g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() LM pos=(%d,%d) dir=%d\n", pu.blocks[ CHANNEL_TYPE_CHROMA ].x, pu.blocks[ CHANNEL_TYPE_CHROMA ].y, pu.intraDir[ CHANNEL_TYPE_CHROMA ] );
      return;
    }
  }

#if JVET_AJ0081_CHROMA_TMRL
  if (PU::hasChromaTmrl(pu) && pu.cs->sps->getUseTmrl())
  {
    intraChromaTmrl(pu);
    if (pu.chromaTmrlFlag)
    {
      CHECK(pu.chromaTmrlIdx >= CHROMA_TMRL_LIST_SIZE, "Chroma tmrl index out of bounds");
      return;
    }
  }
#endif

#if JVET_AC0071_DBV
  if (PU::hasChromaBvFlag(pu))
  {
    if (m_BinDecoder.decodeBin(Ctx::DbvChromaMode()) == 0)
    {
      pu.intraDir[1] = DBV_CHROMA_IDX;
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AH0136_CHROMA_REORDERING
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        if (PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX, true))
        {
#endif
          intraChromaFusionMode(pu);
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        }
#endif
#else
        if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
        {
          pu.isChromaFusion = true;
        }
#endif
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() ChromaBv pos=(%d,%d) dir=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
      return;
    }
  }
#endif

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    pu.intraDir[1] = DM_CHROMA_IDX;
#if JVET_Z0050_DIMD_CHROMA_FUSION
    if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
    {
#if JVET_AC0119_LM_CHROMA_FUSION
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
      if (PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX, true))
      {
#endif
        intraChromaFusionMode(pu);
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
      }
#endif
#else
      if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
      {
        pu.isChromaFusion = true;
      }
#endif
    }
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() DM pos=(%d,%d) dir=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
    return;
  }

#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  if (pu.cu->slice->getSPS()->getUseDimd())
  {
    if (m_BinDecoder.decodeBin(Ctx::DimdChromaMode()) == 0)
    {
      pu.intraDir[1] = DIMD_CHROMA_IDX;
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AC0119_LM_CHROMA_FUSION
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        if (PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX, true))
        {
#endif
        intraChromaFusionMode(pu);
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
        }
#endif
#else
        if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
        {
          pu.isChromaFusion = true;
        }
#endif
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() DIMD pos=(%d,%d) dir=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, DIMD_CHROMA_IDX /*pu.intraDir[CHANNEL_TYPE_CHROMA]*/);
      return;
    }
  }
#endif

#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
  pu.parseChromaMode = true;
#endif
  unsigned candId = m_BinDecoder.decodeBinsEP(2);
  DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) cand_idx=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, candId);
#if JVET_Z0050_DIMD_CHROMA_FUSION
  if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
  {
#if JVET_AC0119_LM_CHROMA_FUSION
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
    if (PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX, true))
    {
#endif
      intraChromaFusionMode(pu);
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
    }
#endif
#else
    if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
    {
      pu.isChromaFusion = true;
    }
#endif
  }
#endif
#if JVET_AH0136_CHROMA_REORDERING || JVET_AJ0249_NEURAL_NETWORK_BASED
  pu.intraDir[1] = PLANAR_IDX;
  pu.candId = candId;
#else
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(PU::isLMCMode(chromaCandModes[candId]), "The intra dir cannot be LM_CHROMA for this path");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  CHECK(chromaCandModes[candId] == DIMD_CHROMA_IDX, "The intra dir cannot be DIMD_CHROMA for this path");
#endif

  pu.intraDir[1] = chromaCandModes[candId];
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  pu.candId = candId;
#endif
#endif
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
#if MULTI_HYP_PRED
    if (!(pu.mergeFlag && pu.numMergedAddHyps == pu.addHypData.size()))
#else
    if( !pu.mergeFlag )
#endif
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf )
    {
      cu.colorTransform = false;
      cu.cs->addEmptyTUs( partitioner );
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
      cu.interCcpMergeZeroRootCbfIdc = 0;
      if (CU::interCcpMergeZeroRootCbfAllowed(cu))
      {
        inter_ccp_merge_root_cbf_zero(cu);
      }
#endif
      return;
    }
  }

  if (CU::isInter(cu) || CU::isIBC(cu))
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
  cuCtx.lfnstLastScanPos                              = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
  cuCtx.mtsCoeffAbsSum                                = 0;
#endif
  ChromaCbfs chromaCbfs;
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, CU::getISPType(cu, getFirstComponentOfChannel(partitioner.chType)), 0);
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx);
  }

  residual_lfnst_mode( cu, cuCtx );
#if JVET_AE0102_LFNST_CTX
  // call coeff coding, check the ordering of tus
  for (auto &currTU : CU::traverseTUs(cu))
  {
    transform_unit(currTU, cuCtx, partitioner, -1, true);
  }
#endif
#if JVET_AL0181_ASBT 
  bool isAsbt = false;
  for (auto& currTU : CU::traverseTUs(cu))
  {
    isAsbt = isAsbt || (currTU.asbtDecimH[COMPONENT_Y] || currTU.asbtDecimW[COMPONENT_Y]);
  }
  if (!isAsbt)
  {
    mts_idx          (cu, cuCtx);
  }
#else
  mts_idx            ( cu, cuCtx );
#endif
#if SIGN_PREDICTION
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    for( int compIdx = COMPONENT_Y; compIdx < MAX_NUM_COMPONENT; ++compIdx)
    {
      ComponentID compID = (ComponentID)compIdx;
      if(compIdx >= currTU.blocks.size())
      {
        continue;
      }

      if(currTU.jointCbCr)
      {
        if( !( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cb )
        {
          continue;
        }
        if( ( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cr )
        {
          continue;
        }
      }

      if(currTU.blocks[compID].valid() && TU::getCbf( currTU, compID ) && TU::getDelayedSignCoding(currTU, compID))
      {
        parsePredictedSigns(const_cast<TransformUnit &>(currTU), compID);
      }
    }
  }
#endif
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__QT_ROOT_CBF, cu.lumaSize());
  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
void CABACReader::inter_ccp_merge_root_cbf_zero(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__QT_ROOT_CBF, cu.lumaSize());
  cu.interCcpMergeZeroRootCbfIdc = unary_max_symbol(Ctx::InterCcpMergeZeroRootCbfIdc(0), Ctx::InterCcpMergeZeroRootCbfIdc(1),
    MAX_CCP_MERGE_WEIGHT_IDX);
  DTRACE(g_trace_ctx, D_SYNTAX, "inter_ccp_merge_root_cbf_zero() pos=(%d,%d) inter_ccp_merge_root_cbf_zero_flag=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.interCcpMergeZeroRootCbfIdc);
}
#endif

void CABACReader::adaptive_color_transform(CodingUnit& cu)
{
  if (!cu.slice->getSPS()->getUseColorTrans())
  {
    return;
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs))
#else
  if (cu.isSepTree())
#endif
  {
    return;
  }

  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__ACT, cu.lumaSize());
    cu.colorTransform = (m_BinDecoder.decodeBin(Ctx::ACTFlag()));
    DTRACE(g_trace_ctx, D_SYNTAX, "adaptive_color_transform() act_flag=(%d,%d)\n", cu.lumaPos().x, cu.lumaPos().y, cu.colorTransform);
  }
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__SBT_MODE, cu.lumaSize());
  //bin - flag
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  bool sbtFlag = m_BinDecoder.decodeBin( Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

#if JVET_AJ0260_SBT_CORNER_MODE
  const uint8_t sbtQuadModeAllowed = CU::targetSbtAllowed( SBT_QUAD, sbtAllowed );
  const uint8_t sbtQuarterModeAllowed = CU::targetSbtAllowed( SBT_QUARTER, sbtAllowed );

  auto sbtQuadMode = [&]( const bool isLast, int& modeIdx )
  {
    if( sbtQuadModeAllowed )
    {
      modeIdx++;

      CHECK( !sbtVerHalfAllow && !sbtHorHalfAllow, "Wrong SBT QUAD setting");

      // SBT QUAD restriction
      CHECK( cuWidth < SBT_QUAD_MIN_BLOCK_SIZE, "Width must be SBT_QUAD_MIN_BLOCK_SIZE or larger for SBT QUAD" );
      CHECK( cuHeight < SBT_QUAD_MIN_BLOCK_SIZE, "Height must be SBT_QUAD_MIN_BLOCK_SIZE or larger for SBT QUAD" );

      bool sbtQuad = isLast ? true : m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 2 ) );

      if( sbtQuad )
      {
        cu.setSbtIdx( SBT_QUAD );
        //pos
        uint8_t horIdx = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 1 ) );
        uint8_t verIdx = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 2 ) );
        cu.setSbtPos( ( verIdx << 1 ) + horIdx );
        DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), ( int ) cu.sbtInfo );
        return true;
      }
    }

    return false;
  };

  auto sbtQuarterMode = [&]( const bool isLast, int& modeIdx )
  {
    if( sbtQuarterModeAllowed )
    {
      modeIdx++;

      // SBT QUARTER restriction
      CHECK( cuWidth < SBT_QUAD_MIN_BLOCK_SIZE, "Width must be SBT_QUAD_MIN_BLOCK_SIZE or larger for SBT QUAD" );
      CHECK( cuHeight < SBT_QUAD_MIN_BLOCK_SIZE, "Height must be SBT_QUAD_MIN_BLOCK_SIZE or larger for SBT QUAD" );

      bool bSbtQuarter = isLast ? true : m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 3 ) );

      if( bSbtQuarter )
      {
        cu.setSbtIdx( SBT_QUARTER );
        //pos
        uint8_t horIdx = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 3 ) );
        uint8_t verIdx = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 4 ) );
        cu.setSbtPos( ( verIdx << 1 ) + horIdx );
        DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), ( int ) cu.sbtInfo );
        return true;
      }
    }

    return false;
  };

  auto sbtRectMode = [&]( const bool isLast, int& modeIdx )
  {
    modeIdx++;

    const bool sbtRect = isLast ? true : m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 1 ) );

    if( !sbtRect )
    {
      return false;
    }
#endif

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }
  cu.setSbtIdx( sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF ) );

  //bin - pos
  bool sbtPosFlag = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 0 ) );
  cu.setSbtPos( sbtPosFlag ? SBT_POS1 : SBT_POS0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbt_info=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );

#if JVET_AJ0260_SBT_CORNER_MODE
  return true;

  };

  int numSbtModesM1 = 1 - ( ( sbtVerHalfAllow + sbtHorHalfAllow + sbtVerQuadAllow + sbtHorQuadAllow ) ? 1 : 0 ) + ( sbtQuadModeAllowed ? 1 : 0 ) + ( sbtQuarterModeAllowed ? 1 : 0 );

  int modeIdx = 0;

  if( sbtRectMode( modeIdx == numSbtModesM1, modeIdx ) )
  {
    return;
  }
  else if( sbtQuadMode( modeIdx == numSbtModesM1, modeIdx ) )
  {
    return;
  }
  else if( sbtQuarterMode( modeIdx == numSbtModesM1, modeIdx ) )
  {
    return;
  }
#endif
}


void CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

  if (((rbPos.x & cu.cs->pcv->maxCUWidthMask) == 0 || rbPos.x == cu.cs->pps->getPicWidthInLumaSamples())
      && ((rbPos.y & cu.cs->pcv->maxCUHeightMask) == 0 || rbPos.y == cu.cs->pps->getPicHeightInLumaSamples())
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS      
      && (!CS::isDualITree(*cu.cs) || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType)))
#else
      && (!cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType)))
#endif
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );
  }
}

void CABACReader::cu_palette_info(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__PLT_MODE, cu.block((!CS::isDualITree(*cu.cs) || isLuma(cu.chType))? COMPONENT_Y: COMPONENT_Cb).lumaSize(), (!CS::isDualITree(*cu.cs) || isLuma(cu.chType)) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA);

  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  int curPLTidx = 0;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
    cu.cs->prevPLT.curPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[COMPONENT_Y];
#endif
  cu.lastPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[compBegin];
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int maxPltSize = CS::isDualITree(*cu.cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#else
  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#endif
  if (cu.lastPLTSize[compBegin])
  {
    xDecodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        for( int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++ )
        {
          cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
        }
      }
      else
      {
#endif
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
        }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      }
#endif
      curPLTidx++;
    }
  }
  cu.reusePLTSize[compBegin] = curPLTidx;

  uint32_t recievedPLTnum = 0;
  if (curPLTidx < maxPltSize)
  {
    recievedPLTnum = exp_golomb_eqprob(0);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() recieved_plt_num=%d\n", recievedPLTnum);

  cu.curPLTSize[compBegin] = curPLTidx + recievedPLTnum;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
    cu.curPLTSize[COMPONENT_Y] = cu.curPLTSize[compBegin];
#endif
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = curPLTidx; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      cu.curPLT[compID][idx] = m_BinDecoder.decodeBinsEP(channelBitDepth);
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() comp=%d idx=%d cur_plt=%d\n", comp, idx, cu.curPLT[compID][idx]);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        if( isLuma( cu.chType ) )
        {
          cu.curPLT[COMPONENT_Cb][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
          cu.curPLT[COMPONENT_Cr][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
        }
        else
        {
          cu.curPLT[COMPONENT_Y][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
        }
      }
#endif
    }
  }
  cu.useEscape[compBegin] = true;
  if (cu.curPLTSize[compBegin] > 0)
  {
    uint32_t escCode = 0;
    escCode = m_BinDecoder.decodeBinEP();
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() esc_code=%d\n", escCode);
    cu.useEscape[compBegin] = (escCode != 0);
  }
  uint32_t    indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  //encode index map
  uint32_t    height = cu.block(compBegin).height;
  uint32_t    width = cu.block(compBegin).width;

  uint32_t total = height * width;
  if (indexMaxSize > 1)
  {
    parseScanRotationModeFlag(cu, compBegin);
  }
  else
  {
    cu.useRotation[compBegin] = false;
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
#else
    if (!cu.isSepTree() || isLuma(tu.chType))
#endif
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
#else
    if (!cu.isSepTree() || isChroma(tu.chType))
#endif
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t prevRunPos = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}

void CABACReader::cuPaletteSubblockInfo(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType)
{
  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&  tu = *cu.firstTU;
  PLTtypeBuf      runType = tu.getrunType(compBegin);
  PelBuf          curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t        indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  uint32_t        totalPel = cu.block(compBegin).height*cu.block(compBegin).width;

  int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
  int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
  maxSubPos = (maxSubPos > totalPel) ? totalPel : maxSubPos; // if last position is out of the current CU size

  unsigned runCopyFlag[(1 << LOG2_PALETTE_CG_SIZE)];
  for (int i = 0; i < (1 << LOG2_PALETTE_CG_SIZE); i++)
  {
    runCopyFlag[i] = MAX_INT;
  }
  if (minSubPos == 0)
  {
    runCopyFlag[0] = 0;
  }

// PLT runCopy flag and runType - context coded
  int curPos = minSubPos;
  for (; curPos < maxSubPos && indexMaxSize > 1; curPos++)
  {
    uint32_t posy = m_scanOrder[curPos].y;
    uint32_t posx = m_scanOrder[curPos].x;
    uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    unsigned identityFlag = 1;

    const CtxSet&   ctxSet = (prevRunType == PLT_RUN_INDEX) ? Ctx::IdxRunModel : Ctx::CopyRunModel;
    if (curPos > 0)
    {
      int dist = curPos - prevRunPos - 1;
      const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      identityFlag = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
      runCopyFlag[curPos - minSubPos] = identityFlag;
    }

    if ( identityFlag == 0 || curPos == 0 )
    {
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else
      {
        runType.at(posx, posy) = (m_BinDecoder.decodeBin(Ctx::RunTypeFlag()));
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
      prevRunType = runType.at(posx, posy);
      prevRunPos  = curPos;
    }
    else //assign run information
    {
      runType.at(posx, posy) = runType.at(posxprev, posyprev);
    }
  }

// PLT index values - bypass coded
  uint32_t adjust;
  uint32_t symbol = 0;
  curPos = minSubPos;
  if (indexMaxSize > 1)
  {
    for (; curPos < maxSubPos; curPos++)
    {
      if (curPos > 0)
      {
        adjust = 1;
      }
      else
      {
        adjust = 0;
      }

      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      if ( runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX )
      {
        xReadTruncBinCode(symbol, indexMaxSize - adjust);
        xAdjustPLTIndex(cu, symbol, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
      else if (runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
      }
      else
      {
        curPLTIdx.at(posx, posy) = (cu.useRotation[compBegin]) ? curPLTIdx.at(posx - 1, posy) : curPLTIdx.at(posx, posy - 1);
      }
    }
  }
  else
  {
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      runType.at(posx, posy) = PLT_RUN_INDEX;
      if (runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = 0;
      }
      else
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
      }
    }
  }

// Quantized escape colors - bypass coded
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (curPos = minSubPos; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            escapeValue.at(posx, posy) = exp_golomb_eqprob(5);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            assert(escapeValue.at(posx, posy) < (TCoeff(1) << (cu.cs->sps->getBitDepth(toChannelType((ComponentID)comp)) + 1)));
#else
            assert(escapeValue.at(posx, posy) < (1 << (cu.cs->sps->getBitDepth(toChannelType((ComponentID)comp)) + 1)));
#endif
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            escapeValue.at(posxC, posyC) = exp_golomb_eqprob(5);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            assert(escapeValue.at(posxC, posyC) < (TCoeff(1) << (cu.cs->sps->getBitDepth(toChannelType(compID)) + 1)));
#else
            assert(escapeValue.at(posxC, posyC) < (1 << (cu.cs->sps->getBitDepth(toChannelType(compID)) + 1)));
#endif
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
      }
    }
  }
}

void CABACReader::parseScanRotationModeFlag(CodingUnit& cu, ComponentID compBegin)
{
  cu.useRotation[compBegin] = m_BinDecoder.decodeBin(Ctx::RotationFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() use_rotation=%d\n", cu.useRotation[compBegin]);
}

void CABACReader::xDecodePLTPredIndicator(CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  uint32_t symbol, numPltPredicted = 0, idx = 0;

  symbol = exp_golomb_eqprob(0);

  if (symbol != 1)
  {
    while (idx < cu.lastPLTSize[compBegin] && numPltPredicted < maxPLTSize)
    {
      if (idx > 0)
      {
        symbol = exp_golomb_eqprob(0);
      }
      if (symbol == 1)
      {
        break;
      }

      if (symbol)
      {
        idx += symbol - 1;
      }
      cu.reuseflag[compBegin][idx] = 1;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        cu.reuseflag[COMPONENT_Y][idx] = 1;
      }
#endif
      numPltPredicted++;
      idx++;
    }
  }
}
void CABACReader::xAdjustPLTIndex(CodingUnit& cu, Pel curLevel, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t symbol;
  int refLevel = MAX_INT;
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  if (idx)
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      refLevel = paletteIdx.at(prevposx, prevposy);
      if (paletteIdx.at(prevposx, prevposy) == cu.curPLTSize[compBegin]) // escape
      {
        refLevel = maxSymbol - 1;
      }
    }
    else
    {
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        refLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        refLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
    }
    maxSymbol--;
  }
  symbol = curLevel;
  if (curLevel >= refLevel) // include escape mode
  {
    symbol++;
  }
  paletteIdx.at(posx, posy) = symbol;
}

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu, mrgCtx );
//    void  merge_flag      ( pu );
//    void  merge_data      ( pu, mrgCtx );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACReader::prediction_unit( PredictionUnit& pu, MergeCtx& mrgCtx )
{
  if( pu.cu->skip )
  {
    pu.mergeFlag = true;
  }
  else
  {
    merge_flag( pu );
  }

#if !JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AA0070_RRIBC
  rribcData(*pu.cu);
#endif
#else
  if (pu.isBvpClusterApplicable())
  {
    bvOneZeroComp(*pu.cu);
  }
#if JVET_AA0070_RRIBC
  else
  {
    rribcData(*pu.cu);
  }
#endif
#endif
#if JVET_AC0112_IBC_LIC
  cuIbcLicFlag(*pu.cu);
#endif

  if( pu.mergeFlag )
  {
    merge_data(pu);
#if MULTI_HYP_PRED
    if( !pu.cu->skip && !CU::isIBC( *pu.cu ) )
    {
      CHECK(pu.cu->imv != 0, "Multi Hyp: pu.cu->imv != 0");
      mh_pred_data(pu);
    }
#endif
  }
  else if (CU::isIBC(*pu.cu))
  {
#if JVET_AE0169_BIPREDICTIVE_IBC
    ibcBiPredictionFlag(pu);
    if (pu.interDir == 3)
    {
      pu.amvpMergeModeFlag[REF_PIC_LIST_1] = true;
    }
#endif
#if JVET_AC0112_IBC_CIIP
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
#if !JVET_AE0169_BIPREDICTIVE_IBC
    pu.interDir = 1;
#endif
    pu.cu->affine = false;
    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;

    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_AA0070_RRIBC
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    pu.bvdSuffixInfo.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.isBvdPredApplicable(), pu.isBvpClusterApplicable(),
              pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir, pu.cu->rribcFlipType);
#else
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.isBvdPredApplicable(), pu.cu->rribcFlipType);
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir,
              pu.cu->rribcFlipType);
#else
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.cu->rribcFlipType);
#endif
#endif
#else
#if JVET_AC0104_IBC_BVD_PREDICTION
#error Not implemented  
#endif
    mvd_coding(pu.mvd[REF_PIC_LIST_0], true, pu.cu->rribcFlipType);
#endif
#elif JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    pu.bvdSuffixInfo.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.isBvdPredApplicable(), pu.isBvpClusterApplicable(),
              pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir);
#else
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.isBvdPredApplicable());
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir);
#else
    bvdCoding(pu.mvd[REF_PIC_LIST_0]);
#endif
#endif
#else
    mvd_coding(pu.mvd[REF_PIC_LIST_0]);
#endif

    if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
    {
      pu.mvpIdx[REF_PIC_LIST_0] = 0;
    }
    else
    {
      mvp_flag(pu, REF_PIC_LIST_0);
    }
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
    if (pu.isBvpClusterApplicable())
    {
      if (pu.mvpIdx[REF_PIC_LIST_0] == 0)
      {
        if (pu.cu->bvZeroCompDir == 1)
        {
          pu.mvd[REF_PIC_LIST_0].hor = -pu.mvd[REF_PIC_LIST_0].hor;
        }
        else if (pu.cu->bvZeroCompDir == 2)
        {
          pu.mvd[REF_PIC_LIST_0].ver = -pu.mvd[REF_PIC_LIST_0].ver;
        }
      }
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
    {
      merge_idx(pu);
      pu.mvd[REF_PIC_LIST_1].set(0, 0);
    }
#endif
  }
  else
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMerge_mode( pu );
    if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
    {
      pu.cu->affine = 0;
      pu.cu->smvdMode = 0;
      CHECK(pu.interDir != 3, "this is not possible");
      CHECK(CU::isIBC(*pu.cu) != 0, "this is not possible");
    }
    else
    {
#endif
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
#if JVET_AG0098_AMVP_WITH_SBTMVP
    amvpSbTmvpFlag(pu);
    if (!pu.amvpSbTmvpFlag)
    {
#endif
    smvd_mode( pu );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if(pu.cs->sps->getUseMvdPred())
    {
#endif
    cu_bcw_flag(*pu.cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    }
#endif
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    refPairIdx(pu);
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
    }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    }
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
    refIdxLC(pu);
#endif
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!pu.amvpMergeModeFlag[REF_PIC_LIST_0])
      {
#endif
      ref_idx     ( pu, REF_PIC_LIST_0 );
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
      if( pu.cu->affine )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
        const auto& motionModel = (pu.interDir == 3) ? MotionModel::BiAffine : MotionModel::UniAffine;
        pu.mvdSuffixInfo.setMotionModel(motionModel);
        {
          auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0];
          si.setMotionModel(motionModel);
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0], &si, !pu.isMvdPredApplicable());
        }
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0], !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0]);
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0]);
#endif
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
        auto& si1 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][1];
        si1.setMotionModel(motionModel);
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1], &si1, !pu.isMvdPredApplicable());
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][1]);
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1]);
#endif
#endif
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          auto& si2 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][2];
          si2.setMotionModel(motionModel);
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2], &si2, !pu.isMvdPredApplicable());
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][2]);
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2]);
#endif
#endif
        }
      }
#if JVET_AG0098_AMVP_WITH_SBTMVP
      else if (pu.amvpSbTmvpFlag)
      {
        amvpSbTmvpMvdCoding(pu, pu.mvd[REF_PIC_LIST_0]);
      }
#endif
      else
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (pu.amvpMergeModeFlag[REF_PIC_LIST_1] == true && pu.mvpIdx[REF_PIC_LIST_0] < 2)
        {
          pu.mvd[REF_PIC_LIST_0].setZero();
        }
        else
        {
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto motionModel = pu.cu->smvdMode ? MotionModel::BiTranslationalSmvd :
                                                     ( 3 == pu.interDir ) ? MotionModel::BiTranslational : MotionModel::UniTranslational;
          pu.mvdSuffixInfo.setMotionModel(motionModel);
          auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0];
          si.setMotionModel(motionModel);
          mvd_coding(pu.mvd[REF_PIC_LIST_0], &si, !pu.isMvdPredApplicable());
#else
          mvd_coding(pu.mvd[REF_PIC_LIST_0] ,  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
        mvd_coding(pu.mvd[REF_PIC_LIST_0], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0]);
#else
        mvd_coding(pu.mvd[REF_PIC_LIST_0] );
#endif
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
    }

    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
#if JVET_X0083_BM_AMVP_MERGE_MODE
        if (!pu.amvpMergeModeFlag[REF_PIC_LIST_1])
        {
#endif
        ref_idx(pu, REF_PIC_LIST_1);
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
        if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */)
        {
          pu.mvd[REF_PIC_LIST_1]        = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][0] = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][1] = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][2] = Mv();
        }
        else if (pu.cu->affine)
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto& motionModel = (pu.interDir == 3) ? MotionModel::BiAffine : MotionModel::UniAffine;
          pu.mvdSuffixInfo.setMotionModel(motionModel);
          auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0];
          si.setMotionModel(motionModel);
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0], &si, !pu.isMvdPredApplicable());
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0]);
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0]);
#endif
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          auto& si1 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][1];
          si1.setMotionModel(motionModel);
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1], &si1, !pu.isMvdPredApplicable());
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][1]);
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1]);
#endif
#endif
          if (pu.cu->affineType == AFFINEMODEL_6PARAM)
          {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
            auto& si2 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][2];
            si2.setMotionModel(motionModel);
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2], &si2, !pu.isMvdPredApplicable());
#else
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][2]);
#else
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2]);
#endif
#endif
          }
        }
#if JVET_AG0098_AMVP_WITH_SBTMVP
        else if (pu.amvpSbTmvpFlag)
        {
          amvpSbTmvpMvdCoding(pu, pu.mvd[REF_PIC_LIST_1]);
        }
#endif
        else
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] == true && pu.mvpIdx[REF_PIC_LIST_1] < 2)
          {
            pu.mvd[REF_PIC_LIST_1].setZero();
          }
          else
          {
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
            const auto motionModel = pu.cu->smvdMode ? MotionModel::BiTranslationalSmvd :
                                                       ( 3 == pu.interDir ) ? MotionModel::BiTranslational : MotionModel::UniTranslational;
            pu.mvdSuffixInfo.setMotionModel(motionModel);
            auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0];
            si.setMotionModel(motionModel);
            mvd_coding(pu.mvd[REF_PIC_LIST_1], &si, !pu.isMvdPredApplicable());
#else
            mvd_coding(pu.mvd[REF_PIC_LIST_1],  !pu.isMvdPredApplicable());
#endif
#else
#if JVET_AD0140_MVD_PREDICTION
          mvd_coding(pu.mvd[REF_PIC_LIST_1], pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0]);
#else
          mvd_coding(pu.mvd[REF_PIC_LIST_1]);
#endif
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          }
#endif
        }
#if JVET_X0083_BM_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      else
      {
        pu.refIdx[REF_PIC_LIST_1] = pu.cs->slice->getSymRefIdx(REF_PIC_LIST_1);
        mvp_flag    ( pu, REF_PIC_LIST_1 );
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
    }
  }
#if MULTI_HYP_PRED
  CHECK(PU::isBipredRestriction(pu) && !pu.addHypData.empty(), "additional hypotheseis signalled in restricted bi-pred mode");
#endif
  if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
  {
    pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
    pu.interDir               =  1;
    pu.cu->bcwIdx = BCW_DEFAULT;
  }

  if ( pu.cu->smvdMode )
  {
    RefPicList eCurRefList = (RefPicList)(pu.cu->smvdMode - 1);
    pu.mvd[1 - eCurRefList].set( -pu.mvd[eCurRefList].hor, -pu.mvd[eCurRefList].ver );
    CHECK(!((pu.mvd[1 - eCurRefList].getHor() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getHor() <= MVD_MAX)) || !((pu.mvd[1 - eCurRefList].getVer() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getVer() <= MVD_MAX)), "Illegal MVD value");
    pu.refIdx[1 - eCurRefList] = pu.cs->slice->getSymRefIdx( 1 - eCurRefList );
  }
}
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
void CABACReader::mvsd_data(PredictionUnit&  pu)
{
  CHECK(pu.cu->slice->getSliceType() == I_SLICE && !CU::isIBC(*pu.cu), "cannot be I Slice");
#if !JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
    return;
  }
#endif

  if (pu.cu->skip || pu.mergeFlag 
#if !JVET_AC0104_IBC_BVD_PREDICTION
    || CU::isIBC(*pu.cu)
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
    || !pu.isMvdPredApplicable()
#endif
     )
  {
    return;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif

#if JVET_AD0140_MVD_PREDICTION
  const int numAffinesInRpl = 2 + (pu.cu->affineType == AFFINEMODEL_6PARAM);
  for (int i = REF_PIC_LIST_0; i < NUM_REF_PIC_LIST_01; ++i)
  {
    const RefPicList& eRefList = static_cast<RefPicList>(i);
    if (pu.cu->affine)
    {
      const auto& motionModel = (3 == pu.interDir) ? MotionModel::BiAffine : MotionModel::UniAffine ;
      for (int j = 0; j < numAffinesInRpl; ++j)
      {
        pu.mvdSuffixInfo.initSuffixesAndSignsMvd( j, eRefList, pu.mvdAffi[i][j], pu.cu->imv, motionModel );
      }
    }
    else
    {
      const auto& motionModel = ( 0 != pu.cu->smvdMode ) ? MotionModel::BiTranslationalSmvd : 
                                                          (3 == pu.interDir) ? MotionModel::BiTranslational : MotionModel::UniTranslational;
      pu.mvdSuffixInfo.initSuffixesAndSignsMvd(0, static_cast<RefPicList>(i), pu.mvd[i], pu.cu->imv, motionModel);

      if ( pu.cu->smvdMode )
      {
        break;
      }
    }
  }
  pu.mvdSuffixInfo.getMergedBinBudgetForMv( MvdSuffixInfoMv::getBinBudgetForPrediction(pu.Y().width, pu.Y().height, pu.cu->imv));
#endif
  if (pu.interDir != 2 /* PRED_L1 */)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_0);
    }
    else
#endif

    {
      mvsdIdxFunc(pu, REF_PIC_LIST_0);
    }
  }

#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
    return;
  }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.interDir != 1 /* PRED_L0 */ && pu.cu->smvdMode != 1)
  {
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_1);
    }
    else
    {
      mvsdIdxFunc(pu, REF_PIC_LIST_1);
    }
  }
#endif
}
#endif
void CABACReader::smvd_mode( PredictionUnit& pu )
{
  pu.cu->smvdMode = 0;
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__SYMMVD_FLAG, pu.lumaSize());

  pu.cu->smvdMode = m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

#if JVET_AG0098_AMVP_WITH_SBTMVP 
void CABACReader::amvpSbTmvpFlag(PredictionUnit& pu)
{
  pu.amvpSbTmvpFlag = false;
  pu.colIdx = -1;
  if (!pu.cs->sps->getSbTMVPEnabledFlag())
  {
    return;
  }

  if (!pu.cs->slice->getAmvpSbTmvpEnabledFlag())
  {
    return;
  }

  if (pu.cu->affine)
  {
    return;
  }
  if (pu.interDir == 3)
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AMVP_SBTMVP, pu.lumaSize());
  
  pu.amvpSbTmvpFlag = m_BinDecoder.decodeBin(Ctx::amvpSbTmvpFlag(0)) ? 1 : 0;
  pu.colIdx = 0;
  
  if (pu.amvpSbTmvpFlag)
  {
    pu.colIdx = 0;
    RefPicList colRefList = (pu.colIdx == 0 ? RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag() : 0) : RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag2nd() : 0));
    pu.interDir = (1 << colRefList);
    if (pu.cs->slice->getAmvpSbTmvpNumColPic() > 1)
    {
      pu.interDir = 1;
      if (m_BinDecoder.decodeBin(Ctx::amvpSbTmvpFlag(1)))
      {
        pu.interDir = 2;
      }

      colRefList = RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag() : 0);
      if ((pu.interDir == 1 && colRefList == REF_PIC_LIST_0) || (pu.interDir == 2 && colRefList == REF_PIC_LIST_1))
      {
        pu.colIdx = 0;
      }
      else
      {
        pu.colIdx = 1;
        colRefList = RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag2nd() : 0);
      }
    }
    pu.refIdx[colRefList] = pu.colIdx == 0 ? pu.cs->slice->getColRefIdx() : pu.cs->slice->getColRefIdx2nd();
    pu.cu->smvdMode = 0;
    g_picAmvpSbTmvpEnabledArea += pu.lwidth() * pu.lheight();
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpFlag() amvpSbTmvpFlag=%d colIdx:%d pos=(%d,%d) size=%dx%d\n", pu.amvpSbTmvpFlag ? 1 : 0, pu.colIdx ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}

void CABACReader::amvpSbTmvpMvdCoding(PredictionUnit &pu, Mv &rMvd)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AMVP_SBTMVP, pu.lumaSize());
  
  if (m_BinDecoder.decodeBin(Ctx::amvpSbTmvpMvdIdx(0)))
  {
    pu.amvpSbTmvpMvdIdx = -1;
    rMvd.setZero();
    DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpMvdCoding() pos=(%d,%d) size=(%d,%d) amvpSbTmvpMvdIdx:%d\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), -1);
  }
  else
  {
    unsigned int uiUnaryIdx = 0;
    int numStepCandMinus1 = pu.cs->slice->getAmvpSbTmvpNumOffset() - 1;
    int temp = 0;
    temp = m_BinDecoder.decodeBinsEP(2);
    for (; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
    {
      if (!m_BinDecoder.decodeBin(Ctx::amvpSbTmvpMvdIdx(uiUnaryIdx + 1)))
      {
        break;
      }
    }
    uiUnaryIdx <<= 2;
    uiUnaryIdx += temp;
    pu.amvpSbTmvpMvdIdx = uiUnaryIdx;
    rMvd.set(1, 1);
    DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpMvdCoding() pos=(%d,%d) size=(%d,%d) amvpSbTmvpMvdIdx:%d numStepCandMinus1:%d\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), pu.amvpSbTmvpMvdIdx, numStepCandMinus1);
  }
}
#endif

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
  cu.affine = false;

  if ( !cu.cs->slice->isIntra() && (cu.slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && 
#if JVET_AJ0085_SUBBLOCK_MERGE_MODE_EXTENSION
    CU::isAffineAllowed(cu)
#else
    cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 
#endif
    )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AFFINE_FLAG, cu.lumaSize());

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
#if JVET_AJ0085_SUBBLOCK_MERGE_MODE_EXTENSION
    if(CU::affineCtxInc(cu))
    {
      ctxId += 3;
    }
#endif

    cu.affine = m_BinDecoder.decodeBin( Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
#if INTER_RM_SIZE_CONSTRAINTS
  if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8)
#else
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AFFINE_FLAG, cu.lumaSize());

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      ctxId = 0;
      cu.affineType = m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      cu.affineType = AFFINEMODEL_4PARAM;
    }
  }
}

#if AFFINE_MMVD
void CABACReader::affine_mmvd_data(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseAffineMmvdMode() || !pu.mergeFlag || !pu.cu->affine)
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AFFINE_MMVD, pu.lumaSize());
  
  pu.afMmvdFlag = (m_BinDecoder.decodeBin(Ctx::AfMmvdFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_flag() af_mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.afMmvdFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  
  if (!pu.afMmvdFlag)
  {
    return;
  }
  
  // Base affine merge candidate idx
  uint8_t afMmvdBaseIdx = 0;
  int numCandMinus1Base = AF_MMVD_BASE_NUM - 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  unsigned ctxId = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  static_assert(ECM3_AF_MMVD_BASE_NUM == 1, "The value of ECM3_AF_MMVD_BASE_NUM must be 1");
  if (pu.cs->sps->getUseTMMMVD())
#endif
  {
    const CodingStructure *cs = pu.cu->cs;
    const CodingUnit *cuLeft = cs->getCURestricted(pu.cu->lumaPos().offset(-1, 0), *pu.cu, CH_L);
    ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;
    const CodingUnit *cuAbove = cs->getCURestricted(pu.cu->lumaPos().offset(0, -1), *pu.cu, CH_L);
    ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;
  }
  numCandMinus1Base = (ctxId == 0) ? 0 : ((ctxId == 1) ? 1 : AF_MMVD_BASE_NUM-1);
#endif
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    int ctx2 =  (numCandMinus1Base == 1) ? 1 : 0;
    if (m_BinDecoder.decodeBin(Ctx::AfMmvdIdx(ctx2)))
#else
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdIdx()))
#endif
      {
        afMmvdBaseIdx++;
        for (; afMmvdBaseIdx < numCandMinus1Base; afMmvdBaseIdx++)
        {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
          if (!m_BinDecoder.decodeBin(Ctx::AfMmvdIdx(afMmvdBaseIdx + 1)))
#else
            if (!m_BinDecoder.decodeBinEP())
#endif
            {
              break;
            }
        }
      }
  }
  pu.afMmvdBaseIdx = afMmvdBaseIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_base_idx() af_mmvd_base_idx=%d\n", afMmvdBaseIdx);
  
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  (AF_MMVD_MAX_REFINE_NUM >> (ricePar+AFFINE_MMVD_SIZE_SHIFT)) / AFFINE_BI_DIR  - 1;
#else
  int numStepCandMinus1 =  (AF_MMVD_MAX_REFINE_NUM >> (ricePar + AFFINE_MMVD_SIZE_SHIFT))  - 1;
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int temp = m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep(5));
#else
  int temp = (ricePar == 0) ? 0 : m_BinDecoder.decodeBinsEP(ricePar);
#endif
  int uiUnaryIdx = 0;
  for (; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uiUnaryIdx += afMmvdBaseIdx * AF_MMVD_MAX_REFINE_NUM;
#endif
  pu.afMmvdMergeIdx = uiUnaryIdx;
  pu.afMmvdStep = 0;
  pu.afMmvdDir = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  {
    // Decode Step Value
    uint8_t stepOffset = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    int numStepCandMinus1 = ECM3_AF_MMVD_STEP_NUM - 1;
#else
    int numStepCandMinus1 = AF_MMVD_STEP_NUM - 1;
#endif
    if (numStepCandMinus1 > 0)
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStepECM3()))
#else
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep()))
#endif
      {
        stepOffset++;
        for (; stepOffset < numStepCandMinus1; stepOffset++)
        {
          if (!m_BinDecoder.decodeBinEP())
          {
            break;
          }
        }
      }
    }
    pu.afMmvdStep = stepOffset;
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_offset_step() af_mmvd_offset_step=%d\n", stepOffset);
    
    // Decode offset direction
    uint8_t b0 = 0, b1 = 0;
    b0 = m_BinDecoder.decodeBinEP();
    b1 = m_BinDecoder.decodeBinEP();
    uint8_t offDir = (b1 << 1) | b0;
    pu.afMmvdDir = offDir;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    pu.afMmvdMergeIdx = (uint8_t)(pu.afMmvdBaseIdx * ECM3_AF_MMVD_MAX_REFINE_NUM + pu.afMmvdStep * ECM3_AF_MMVD_OFFSET_DIR + pu.afMmvdDir);
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_offset_dir() af_mmvd_offset_dir=%d\n", offDir);
  }
#endif
}
#endif

#if JVET_AA0061_IBC_MBVD
void CABACReader::ibcMbvdData(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_BVD, pu.lumaSize(), COMPONENT_Y);

  if (!pu.cs->sps->getUseIbcMbvd() || !pu.mergeFlag || !CU::isIBC(*pu.cu))
  {
    return;
  }

  pu.ibcMbvdMergeFlag = (m_BinDecoder.decodeBin(Ctx::IbcMbvdFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_mbvd_flag=%d pos=(%d,%d) size=%dx%d\n", pu.ibcMbvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.ibcMbvdMergeFlag)
  {
    return;
  }
#if JVET_AE0169_BIPREDICTIVE_IBC
  int biMbvdMode = 0;
  if (pu.interDir == 3)
  {
    biMbvdMode = m_BinDecoder.decodeBin(Ctx::IbcMbvdFlag(1)) + 1;
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() bi_mbvd_mode=%d\n", biMbvdMode);
  }
#endif

  // Base IBC merge candidate idx
  uint8_t var0 = 0;
#if JVET_AE0169_BIPREDICTIVE_IBC
  int numBaseCandMinus1 = std::min(int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1, IBC_MBVD_BASE_NUM - 1);
#else
  int numBaseCandMinus1 = IBC_MBVD_BASE_NUM - 1;
#endif
  if (numBaseCandMinus1 > 0)
  {
    // to support more base candidates
    if (m_BinDecoder.decodeBin(Ctx::IbcMbvdMergeIdx()))
    {
      var0++;
      for (; var0 < numBaseCandMinus1; var0++)
      {
        if (!m_BinDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() base_idx=%d\n", var0);

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  const int mbvdsPerBase = pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch() ? IBC_MBVD_SIZE_ENC : IBC_MBVD_MAX_REFINE_NUM;
#else
  const int mbvdsPerBase = IBC_MBVD_MAX_REFINE_NUM;
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
  int ibcMbvdSizeEnc = IBC_MBVD_SIZE_ENC;
  uint8_t var1 = var0;
  if (biMbvdMode == 2)
  {
    // Base IBC merge candidate idx
    if (var1 < numBaseCandMinus1 && numBaseCandMinus1 > 0)
    {
      if (m_BinDecoder.decodeBinEP())
      {
        var1++;
        // to support more base candidates
        if (var1 < numBaseCandMinus1 && m_BinDecoder.decodeBin(Ctx::IbcMbvdMergeIdx()))
        {
          var1++;
          for (; var1 < numBaseCandMinus1; var1++)
          {
            if (!m_BinDecoder.decodeBinEP())
            {
              break;
            }
          }
        }
      }
    }
    if (var1 == var0)
    {
      ibcMbvdSizeEnc--;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() base_idx1=%d\n", var1);
  }

  unsigned int ricePar = 1;
  unsigned int riceParVal = 1<<ricePar;
  int numCandStepMinus1 = ((ibcMbvdSizeEnc+riceParVal-1) >> ricePar) - 1;
  unsigned int uiUnaryIdx = 0;
  int remain = ibcMbvdSizeEnc;
  for (; uiUnaryIdx < numCandStepMinus1; ++uiUnaryIdx, remain-=riceParVal)
  {
    if (!m_BinDecoder.decodeBin(Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  int length = (biMbvdMode != 2 || uiUnaryIdx < numCandStepMinus1) ? ricePar : ceilLog2(remain);
  uiUnaryIdx <<= ricePar;
  int temp = 0;
  if (length > 0)
  {
    temp = m_BinDecoder.decodeBinsEP(length);
  }
  uiUnaryIdx += temp;
  pu.ibcMbvdMergeIdx = var0 * mbvdsPerBase + uiUnaryIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_merge_idx=%d\n", pu.ibcMbvdMergeIdx);

  if (biMbvdMode == 1)
  {
    pu.ibcMergeIdx1 = -1;
  }
  else if (biMbvdMode == 2)
  {
    if (var0 == var1)
    {
      ibcMbvdSizeEnc = IBC_MBVD_SIZE_ENC - (uiUnaryIdx+1);
    }
    else
    {
      ibcMbvdSizeEnc = IBC_MBVD_SIZE_ENC;
    }
    numCandStepMinus1 = ((ibcMbvdSizeEnc+riceParVal-1) >> ricePar) - 1;
    unsigned int uiUnaryIdx1 = 0;

    int remain1 = ibcMbvdSizeEnc;
    for (; uiUnaryIdx1 < numCandStepMinus1; ++uiUnaryIdx1, remain1-=riceParVal)
    {
      if (!m_BinDecoder.decodeBin(Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx1 > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx1))))
      {
        break;
      }
    }
    int length1 = ((var0 != var1) || (uiUnaryIdx1 < numCandStepMinus1)) ? ricePar : ceilLog2(remain1);
    uiUnaryIdx1 <<= ricePar;
    int temp1 = 0;
    if (length1 > 0)
    {
      temp1 = m_BinDecoder.decodeBinsEP(length1);
    }
    uiUnaryIdx1 += temp1;
    if (var0 == var1)
    {
      uiUnaryIdx1 += (uiUnaryIdx+1);
    }
    pu.ibcMergeIdx1 = var1 * mbvdsPerBase + uiUnaryIdx1 + IBC_MRG_MAX_NUM_CANDS;
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_merge_idx1=%d\n", pu.ibcMergeIdx1 - IBC_MRG_MAX_NUM_CANDS);
  }
#else
  unsigned int uiUnaryIdx = 0;
  unsigned int ricePar = 1;
  int numCandStepMinus1 = (IBC_MBVD_SIZE_ENC >> ricePar) - 1;
  int temp = 0;
  temp = m_BinDecoder.decodeBinsEP(ricePar);

  for (; uiUnaryIdx < numCandStepMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
  pu.ibcMbvdMergeIdx = var0 * mbvdsPerBase + uiUnaryIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() merge_idx=%d\n", pu.ibcMbvdMergeIdx);
#endif
}
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
void CABACReader::tm_merge_flag(PredictionUnit& pu)
{
  pu.tmMergeFlag = false;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (CU::isIBC(*pu.cu) && !pu.cs->sps->getUseTMIbc())
  {
    return;
  }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag)
  {
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      pu.tmMergeFlag = m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
    }
    return;
  }
#endif

#if (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG) || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG
  if (pu.cu->geoFlag)
  {
    if (!pu.cs->slice->getSPS()->getUseGPMTMMode())
    {
      return;
    }
  }
  else
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG
  if (pu.regularMergeFlag
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    && !CU::isIBC(*pu.cu)
#endif
    )
  {
    if (!pu.cs->slice->getSPS()->getUseTMMrgMode())
    {
      return;
    }
  }
  else
#endif
  if (!pu.cs->slice->getSPS()->getUseDMVDMode())
  {
#if (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG)
#if !(JVET_Z0084_IBC_TM && IBC_TM_MRG)
    CHECK(true, "Unknown mode to read tm_merge_flag");
#endif
#endif
    return;
  }
#else
  if (pu.cs->slice->getSPS()->getUseDMVDMode())
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::TMMergeFlag(CU::isIBC(*pu.cu) ? 1 : 0)));
#else
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::TMMergeFlag()));
#endif
#if JVET_X0049_ADAPT_DMVR
    DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tm_merge_flag || bm_merge_flag=%d\n", pu.tmMergeFlag);
#else
    DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tm_merge_flag=%d\n", pu.tmMergeFlag ? 1 : 0);
#endif
  }
}
#endif

#if JVET_AC0112_IBC_CIIP
void CABACReader::ibcCiipFlag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcCiip() || ( pu.lx() == 0 && pu.ly() == 0))
  {
    pu.ibcCiipFlag = false;
    return;
  }
  if (pu.lwidth() * pu.lheight() < 32 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    pu.ibcCiipFlag = false;
    return;
  }
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.interDir == 3)
  {
    pu.ibcCiipFlag = false;
    return;
  }
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_CIIP_FLAG, pu.lumaSize(), COMPONENT_Y);

  if (pu.mergeFlag)
  {
    if (pu.cu->skip)
    {
      pu.ibcCiipFlag = false;
      return;
    }
    pu.ibcCiipFlag = m_BinDecoder.decodeBin(Ctx::IbcCiipFlag(0));
  }
  else
  {
#if JVET_AA0070_RRIBC
    if (pu.cu->rribcFlipType)
    {
      pu.ibcCiipFlag = false;
      return;
    }
#endif
#if JVET_AC0112_IBC_LIC
    if (pu.cu->ibcLicFlag)
    {
      pu.ibcCiipFlag = false;
      return;
    }
#endif
    if (pu.cs->slice->getSliceType() != I_SLICE)
    {
      pu.ibcCiipFlag = false;
      return;
    }
    pu.ibcCiipFlag = m_BinDecoder.decodeBin(Ctx::IbcCiipFlag(1));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_ciip_flag() ibc_ciip_flag=%d\n", pu.ibcCiipFlag);
}

void CABACReader::ibcCiipIntraIdx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_CIIP_FLAG, pu.lumaSize(), COMPONENT_Y);

  pu.ibcCiipIntraIdx = m_BinDecoder.decodeBin( Ctx::IbcCiipIntraIdx() );
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_ciip_intra_idx() ibc_ciip_intra_idx=%d\n", pu.ibcCiipIntraIdx);
}
#endif

#if JVET_AC0112_IBC_GPM
void CABACReader::ibcGpmFlag(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_GPM_FLAG, pu.lumaSize(), COMPONENT_Y);

  if (!pu.cs->sps->getUseIbcGpm() || (pu.lx() == 0 && pu.ly() == 0))
  {
    pu.ibcGpmFlag = false;
    return;
  }
  if (pu.lwidth() < 8 || pu.lheight() < 8 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    pu.ibcGpmFlag = false;
    return;
  }
  pu.ibcGpmFlag = (m_BinDecoder.decodeBin(Ctx::IbcGpmFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_flag() ibc_gpm_flag=%d\n", pu.ibcGpmFlag);
}

void CABACReader::ibcGpmMergeIdx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_MERGEIDX, pu.lumaSize(), COMPONENT_Y);

  uint32_t splitDir = 0;
  bool splitDirFirstSet = m_BinDecoder.decodeBin( Ctx::IbcGpmSplitDirSetFlag() );
  if (splitDirFirstSet)
  {
    int splitDirIdx = m_BinDecoder.decodeBinsEP(3);
#if JVET_AJ0107_GPM_SHAPE_ADAPT 
    splitDir = splitDirIdx;
    for(int i = 0; IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM+ IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM; i++)
    {
      if(g_ibcGpmSplitDirFirstSetRank[i] == splitDirIdx+1)
      {
        splitDir = i;
        break;
      }
    }
#else
    splitDir = g_ibcGpmFirstSetSplitDir[splitDirIdx];
#endif
  }
  else
  {
    xReadTruncBinCode(splitDir, IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM);
    uint8_t prefix = splitDir;
    splitDir++;
    for (uint8_t i = 0; i < GEO_NUM_PARTITION_MODE && splitDir != 0; i++)
    {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
      if (g_ibcGpmSplitDirFirstSetRank[i] != 0)
#else
      if (!g_ibcGpmSecondSetSplitDir[i])
#endif
      {
        prefix++;
      }
      else
      {
        splitDir--;
      }
    }
    splitDir = prefix;
  }
  pu.ibcGpmSplitDir = splitDir;

  bool isIntra0 = m_BinDecoder.decodeBin( Ctx::IbcGpmIntraFlag() ) ? true : false;
  bool isIntra1 = isIntra0 ? false : true;
#if JVET_AE0169_GPM_IBC_IBC
  if (!isIntra0 && pu.cs->sps->getMaxNumIBCMergeCand() > 1)
  {
    isIntra1 = m_BinDecoder.decodeBin(Ctx::IbcGpmIntraFlag(1)) ? true : false;
  }
#endif

  const int maxNumIbcGpmCand = pu.cs->sps->getMaxNumIBCMergeCand();
  int numCandminus2 = maxNumIbcGpmCand - 2;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
  if (isIntra0)
  {
    mergeCand0 = IBC_GPM_MAX_NUM_UNI_CANDS + unary_max_eqprob(IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
    }
  }
  if (isIntra1)
  {
    mergeCand1 = IBC_GPM_MAX_NUM_UNI_CANDS + unary_max_eqprob(IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
#if JVET_AE0169_GPM_IBC_IBC
  else
  {
    if (isIntra0)
    {
      if (numCandminus2 >= 0)
      {
        if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
        {
          mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
        }
      }
    }
    else
    {
      if (numCandminus2 > 0)
      {
        if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
        {
          mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
        }
      }
      mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
    }
  }
#else
  else if (numCandminus2 >= 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
    }
  }
#endif
  pu.ibcGpmMergeIdx0 = mergeCand0;
  pu.ibcGpmMergeIdx1 = mergeCand1;
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_merge_idx() ibc_gpm_splt_dir=%d merge_idx0=%d merge_idx1=%d\n", pu.ibcGpmSplitDir, pu.ibcGpmMergeIdx0, pu.ibcGpmMergeIdx1);
}

void CABACReader::ibcGpmAdaptBlendIdx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_GPM_BLENDIDX, pu.lumaSize(), COMPONENT_Y);

  
#if JVET_AE0169_GPM_IBC_IBC
  if ((IBC_GPM_NUM_BLENDING == 1) || pu.cs->slice->getSliceType() != I_SLICE)
#else
  if (IBC_GPM_NUM_BLENDING == 1)
#endif
  {
    pu.ibcGpmBldIdx = 0;
    return;
  }
  int bin0 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(0));
  if (bin0 == 1)
  {
    pu.ibcGpmBldIdx = 0;
  }
  else
  {
    int bin1 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(1));
    if (bin1 == 0)
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(3));
      if (bin2 == 0)
      {
        pu.ibcGpmBldIdx = 4;
      }
      else
      {
        pu.ibcGpmBldIdx = 3;
      }
    }
    else
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(2));
      if (bin2 == 0)
      {
        pu.ibcGpmBldIdx = 1;
      }
      else
      {
        pu.ibcGpmBldIdx = 2;
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_adapt_blend_idx() ibc_gpm_bld_idx=%d\n", pu.ibcGpmBldIdx);
}
#endif

#if JVET_AC0112_IBC_LIC
void CABACReader::cuIbcLicFlag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_LIC_FLAG, cu.lumaSize(), COMPONENT_Y);

#if JVET_AE0159_FIBC
  if (!(cu.cs->sps->getUseIbcLic() || cu.cs->sps->getUseIbcFilter()) || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    cu.ibcLicFlag = false;
    return;
  }
#if JVET_AA0070_RRIBC
  if (cu.rribcFlipType > 0)
  {
    cu.ibcLicFlag = false;
    return;
  }
#endif
  if (cu.lwidth() * cu.lheight() < 32)
  {
    cu.ibcLicFlag = false;
    return;
  }
  if ((cu.lx() >= FIBC_TEMPLATE_SIZE || cu.ly() >= FIBC_TEMPLATE_SIZE) && (cu.cs->slice->getSliceType() == I_SLICE) && cu.cs->sps->getUseIbcFilter() )
  {
    unsigned ctxIdx = 1 + DeriveCtx::ctxIbcFilterFlag(cu);
    cu.ibcFilterFlag = m_BinDecoder.decodeBin(Ctx::IbcLicFlag(ctxIdx));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() filter_flag=%d\n", cu.ibcFilterFlag);
  }
  else
  {
    cu.ibcFilterFlag = false;
  }
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (!cu.ibcFilterFlag && cu.cs->sps->getUseIbcLic())
#else
  if (!cu.ibcFilterFlag && cu.cs->sps->getUseIbcLic() && (cu.lwidth() * cu.lheight() <= 256))
#endif
  {
    cu.ibcLicFlag = m_BinDecoder.decodeBin(Ctx::IbcLicFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_flag=%d\n", cu.ibcLicFlag);
#if JVET_AE0078_IBC_LIC_EXTENSION
    if (cu.ibcLicFlag)
    {
      const int bin1 = m_BinDecoder.decodeBin(Ctx::IbcLicIndex(0));
      const int bin2 = m_BinDecoder.decodeBin(Ctx::IbcLicIndex(1));
      if (bin1)
      {
        cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_L : IBC_LIC_IDX_T;
      }
      else
      {
        cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_M : IBC_LIC_IDX;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_idx=%d\n", cu.ibcLicIdx);
    }
    else
    {
      cu.ibcLicIdx = 0;
    }
#endif
  }
#if !JVET_AE0078_IBC_LIC_EXTENSION
  else if (!cu.ibcFilterFlag && cu.cs->sps->getUseIbcLic()) // (cu.lwidth() * cu.lheight() > 256)
  {
    cu.ibcLicFlag = false;
  }
#endif
  else 
  {
    cu.ibcLicFlag = cu.cs->sps->getUseIbcLic()? true : false;
#if JVET_AE0078_IBC_LIC_EXTENSION
    cu.ibcLicIdx = 0;
#endif
  }
#else
  if (!cu.cs->sps->getUseIbcLic() || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    cu.ibcLicFlag = false;
    return;
  }
#if JVET_AA0070_RRIBC
  if (cu.rribcFlipType > 0)
  {
    cu.ibcLicFlag = false;
    return;
  }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (cu.lwidth() * cu.lheight() < 32)
#else
  if (cu.lwidth() * cu.lheight() < 32 || cu.lwidth() * cu.lheight() > 256)
#endif
  {
    cu.ibcLicFlag = false;
    return;
  }
  cu.ibcLicFlag = m_BinDecoder.decodeBin( Ctx::IbcLicFlag() );
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_flag=%d\n", cu.ibcLicFlag);
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (cu.ibcLicFlag)
  {
    const int bin1 = m_BinDecoder.decodeBin(Ctx::IbcLicIndex(0));
    const int bin2 = m_BinDecoder.decodeBin(Ctx::IbcLicIndex(1));
    if (bin1)
    {
      cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_L : IBC_LIC_IDX_T;
    }
    else
    {
      cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_M : IBC_LIC_IDX;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_idx=%d\n", cu.ibcLicIdx);
  }
  else
  {
    cu.ibcLicIdx = 0;
  }
#endif
#endif
}
#endif

#if JVET_X0049_ADAPT_DMVR
void CABACReader::bm_merge_flag(PredictionUnit& pu)
{
  pu.bmDir = 0;
  pu.bmMergeFlag = false;
  if (!PU::isBMMergeFlagCoded(pu))
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

  unsigned ctxId = DeriveCtx::CtxBMMrgFlag(*pu.cu);
  pu.bmMergeFlag = (m_BinDecoder.decodeBin(Ctx::BMMergeFlag(ctxId)));
  if (pu.bmMergeFlag)
  {
    pu.bmDir = 1 << m_BinDecoder.decodeBin(Ctx::BMMergeFlag(3));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "bm_merge_flag() bm_merge_flag=%d, bmDir = %d\n", pu.bmMergeFlag ? 1 : 0, pu.bmDir);
}
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
void CABACReader::affBmFlag(PredictionUnit& pu)
{
  pu.affBMDir = 0;
  pu.affBMMergeFlag = false;
  if (!PU::isAffBMMergeFlagCoded(pu))
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AFFINE_BM_FLAG_DIR, pu.lumaSize());
  pu.affBMMergeFlag = (m_BinDecoder.decodeBin(Ctx::affBMFlag(0)));
  DTRACE(g_trace_ctx, D_SYNTAX, "aff_bm_flag() aff_bm_flag=%d\n", pu.affBMMergeFlag);
  if (pu.affBMMergeFlag)
  {
    pu.affBMDir = 1 << m_BinDecoder.decodeBin(Ctx::affBMFlag(1));
    DTRACE(g_trace_ctx, D_SYNTAX, "aff_bm_flag() aff_bm_dir=%d\n", pu.affBMDir);
  }
}
#endif
void CABACReader::merge_flag( PredictionUnit& pu )
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if( CU::isIBC( *pu.cu ) && !pu.cu->slice->getSPS()->getUseIbcMerge() )
  {
    pu.mergeFlag = false;
    return;
  }
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

  pu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    return;
  }
}


void CABACReader::merge_data( PredictionUnit& pu )
{
  if (CU::isIBC(*pu.cu))
  {
#if JVET_AE0169_BIPREDICTIVE_IBC
    ibcBiPredictionFlag(pu);
#else
    pu.interDir = 1;
#endif
#if JVET_AA0061_IBC_MBVD
    ibcMbvdData(pu);
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
    {
#endif
      tm_merge_flag(pu);
#if JVET_AA0061_IBC_MBVD
    }
#endif
#endif
#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir != 3)
#endif
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
#if JVET_AC0112_IBC_GPM
#if JVET_AC0112_IBC_CIIP && JVET_AA0061_IBC_MBVD
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (!pu.ibcMbvdMergeFlag && !pu.ibcCiipFlag && pu.interDir != 3)
#else
    if (!pu.ibcMbvdMergeFlag && !pu.ibcCiipFlag)
#endif
#else
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
#else
#if JVET_AC0112_IBC_CIIP
    if (!pu.ibcCiipFlag)
#endif
#endif
#endif
    {
      ibcGpmFlag(pu);
      if (pu.ibcGpmFlag)
      {
        ibcGpmMergeIdx(pu);
        ibcGpmAdaptBlendIdx(pu);
      }
    }
#endif
    merge_idx(pu);
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3 && !pu.ibcMbvdMergeFlag)
    {
      ibcMergeIdx1(pu);
    }
#endif
    return;
  }
  else
  {
    CodingUnit cu = *pu.cu;
    subblock_merge_flag(*pu.cu);
    if (pu.cu->affine)
    {
#if AFFINE_MMVD
      affine_mmvd_data(pu);
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      pu.affBMMergeFlag = false;
      if (!pu.afMmvdFlag)
      {
        affBmFlag(pu);
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      pu.affineOppositeLic = false;
      if (PU::hasOppositeLICFlag(pu) && !pu.afMmvdFlag && !pu.affBMMergeFlag && pu.cs->sps->getUseAffMergeOppositeLic())
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MERGE_FLAG_OPPLIC, pu.lumaSize());
        pu.affineOppositeLic = m_BinDecoder.decodeBin(Ctx::AffineFlagOppositeLic(0));
      }
#endif
      merge_idx(pu);
      cu.firstPU->regularMergeFlag = false;
      return;
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

#if CIIP_RM_BLOCK_SIZE_CONSTRAINTS
#if CTU_256
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );
    
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32 && pu.cu->lwidth() <= maxSize && pu.cu->lheight() <= maxSize;
#else
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32;
#endif
#else
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE && pu.cu->lwidth() * pu.cu->lheight() >= 64;
#endif

#if JVET_Y0065_GPM_INTRA
    const bool geoAvailable  = pu.cu->cs->slice->getSPS()->getUseGeo() && !pu.cu->cs->slice->isIntra()
                              && pu.cs->sps->getMaxNumGeoCand() > 0 && pu.cu->lwidth() >= GEO_MIN_CU_SIZE
#else
    const bool geoAvailable  = pu.cu->cs->slice->getSPS()->getUseGeo() && pu.cu->cs->slice->isInterB()
                              && pu.cs->sps->getMaxNumGeoCand() > 1 && pu.cu->lwidth() >= GEO_MIN_CU_SIZE
#endif
                              && pu.cu->lheight() >= GEO_MIN_CU_SIZE && pu.cu->lwidth() <= GEO_MAX_CU_SIZE
                              && pu.cu->lheight() <= GEO_MAX_CU_SIZE && pu.cu->lwidth() < 8 * pu.cu->lheight()
                              && pu.cu->lheight() < 8 * pu.cu->lwidth();

    if (geoAvailable || ciipAvailable)
    {
      cu.firstPU->regularMergeFlag = m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() regular_merge=%d pos=(%d,%d) size=%dx%d\n", cu.firstPU->regularMergeFlag ? 1 : 0, cu.firstPU->lumaPos().x, cu.firstPU->lumaPos().y, cu.firstPU->lumaSize().width, cu.firstPU->lumaSize().height);
    }
    else
    {
      cu.firstPU->regularMergeFlag = true;
    }
    if (cu.firstPU->regularMergeFlag)
    {
#if TM_MRG
      tm_merge_flag(pu);
#endif
#if JVET_X0049_ADAPT_DMVR
#if TM_MRG
      if (pu.tmMergeFlag
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        || !pu.cs->slice->getSPS()->getUseTMMrgMode()
#endif
        )
#endif
      {
        bm_merge_flag(pu);
#if TM_MRG
        if (pu.bmMergeFlag)
        {
          pu.tmMergeFlag = false;
        }
#endif
      }
#endif
      if (cu.cs->slice->getSPS()->getUseMMVD()
#if TM_MRG
        && !pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
        && !pu.bmMergeFlag
#endif
        )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
        unsigned  ctxId =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                          !pu.cs->sps->getUseTMMMVD() ||
#endif
                          pu.cu->skip ? 0 : 1;
        cu.firstPU->mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(ctxId));
#else
        cu.firstPU->mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
#endif
        DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", cu.firstPU->mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
      }
      else
      {
        cu.firstPU->mmvdMergeFlag = false;
      }
      if (cu.skip)
      {
        cu.mmvdSkip = cu.firstPU->mmvdMergeFlag;
      }
    }
    else
    {
      pu.mmvdMergeFlag = false;
      pu.cu->mmvdSkip = false;
#if TM_MRG
      pu.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      pu.bmMergeFlag = false;
#endif
      if (geoAvailable && ciipAvailable)
      {
        Ciip_flag(pu);
      }
      else if (ciipAvailable)
      {
        pu.ciipFlag = true;
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AG0135_AFFINE_CIIP
          ciipAffineFlag(pu);
          if (!pu.ciipAffine)
          {
#endif
            tm_merge_flag(pu);
#if JVET_AG0135_AFFINE_CIIP
          }
          else
          {
            pu.cu->affine = true;
          }
#endif
#endif
#else
        if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());
          pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
          DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
        }
#endif
#if CIIP_PDPC
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__INTRA_PDPC_FLAG, pu.lumaSize()); 
#if JVET_AG0135_AFFINE_CIIP
        if (!pu.ciipAffine)
        {
#endif
          pu.ciipPDPC = (m_BinDecoder.decodeBin(Ctx::CiipFlag(1)));
#if JVET_AG0135_AFFINE_CIIP
        }
        else
        {
          pu.ciipPDPC = false;
        }
#endif
        DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() ciip_pdpc_flag=%d\n", pu.ciipPDPC);
#endif
      }
      else
      {
        pu.ciipFlag = false;
      }
      if (pu.ciipFlag)
      {
        pu.intraDir[0] = PLANAR_IDX;
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else
      {
        pu.cu->geoFlag = true;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
        pu.cu->geoBlendFlag = false;
#if JVET_AJ0274_REGRESSION_GPM_TM
        pu.geoBlendTmFlag = false;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
        pu.geoBlendIntraFlag = false;
#endif
#endif
#if JVET_AE0046_BI_GPM
        PU::setGpmDirMode(pu);
#endif
      }
    }
  }
  if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
  {
    mmvd_merge_idx(pu);
  }
  else
  {
#if JVET_AG0276_LIC_FLAG_SIGNALING
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG_OPPLIC, pu.lumaSize());
    pu.mergeOppositeLic = false;
    pu.tmMergeFlagOppositeLic = false;
    if (pu.regularMergeFlag && PU::hasOppositeLICFlag(pu) && !pu.bmMergeFlag)
    {
      if (pu.tmMergeFlag && pu.cs->sps->getUseTMMergeOppositeLic())
      {
        pu.tmMergeFlagOppositeLic = m_BinDecoder.decodeBin(Ctx::TmMergeFlagOppositeLic(0));
      }
      else if (pu.cs->sps->getUseMergeOppositeLic())
      {
        pu.mergeOppositeLic = m_BinDecoder.decodeBin(Ctx::MergeFlagOppositeLic(0));
      }
    }
#endif
    merge_idx(pu);
  }
}


void CABACReader::merge_idx( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize());

  if ( pu.cu->affine )
  {
#if AFFINE_MMVD
    if (pu.afMmvdFlag)
    {
      return;
    }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    if (pu.affBMMergeFlag)
    {
      pu.mergeIdx = 0;
      return;
    }
#endif
    int numCandminus1 = int( pu.cs->picHeader->getMaxNumAffineMergeCand() ) - 1;
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (pu.affineOppositeLic)
    {
      numCandminus1 = int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) - 1;
    }
#endif
    pu.mergeIdx = 0;
    if ( numCandminus1 > 0 )
    {
#if JVET_AA0128_AFFINE_MERGE_CTX_INC
      unsigned int unaryIdx = 0;
      for (; unaryIdx < numCandminus1; ++unaryIdx)
      {
        if (!m_BinDecoder.decodeBin(Ctx::AffMergeIdx((unaryIdx > 2 ? 2 : unaryIdx))))
        {
          break;
        }
      }
      pu.mergeIdx = unaryIdx;
#else
      if ( m_BinDecoder.decodeBin( Ctx::AffMergeIdx() ) )
      {
        pu.mergeIdx++;
        for ( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
        {
          if (!m_BinDecoder.decodeBinEP())
          {
            break;
          }
        }
      }
#endif
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    int numCandminus1 = int(pu.cs->sps->getMaxNumMergeCand()) - 1;
    pu.mergeIdx       = 0;

    if (pu.cu->geoFlag)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
      bool bUseOnlyOneVector1 = pu.cs->slice->isInterP() || pu.cs->sps->getMaxNumGeoCand() == 1;
      CHECK(bUseOnlyOneVector1, "CABACReader::merge_idx( bUseOnlyOneVector=1 ) failed.");

      pu.cu->geoBlendFlag = false;
      if ( CU::isGeoBlendAvailable(*pu.cu) )
      {
        pu.cu->geoBlendFlag = m_BinDecoder.decodeBin(Ctx::GeoBlendFlag());
        DTRACE( g_trace_ctx, D_SYNTAX, "geoBlendFlag() geoBlendFlag=%d\n", pu.cu->geoBlendFlag ? 1 : 0 );
      }

      if ( pu.cu->geoBlendFlag )
      {
        if ( m_BinDecoder.decodeBin(Ctx::MergeIdx()) )
        {
          const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoBlendCand() - 2;
          pu.mergeIdx += unary_max_eqprob(maxNumGeoCand) + 1;
        }
        DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );

        pu.geoMergeIdx0 = (uint8_t)pu.mergeIdx;
        pu.geoMergeIdx1 = (uint8_t)pu.mergeIdx;
        pu.gpmIntraFlag = false;
#if JVET_AI0082_GPM_WITH_INTER_IBC
        pu.gpmInterIbcFlag = false;
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
#if JVET_AN0203_RGPM_NO_TM
        pu.geoBlendTmFlag = false;
        if ( CU::isGeoBlendTmAvail(*pu.cu) )
#endif
        {
          pu.geoBlendTmFlag = m_BinDecoder.decodeBin(Ctx::GeoBlendTMFlag());
        }
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
        if (!pu.geoBlendTmFlag && CU::isGeoBlendIntraAvailable(*pu.cu))
        {
          pu.geoBlendIntraFlag = m_BinDecoder.decodeBin(Ctx::GeoBlendIntraFlag());
        }
#endif
        return;
      }
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      geoAdaptiveBlendingIdx(pu);
#endif
#if JVET_AL0134_SGPM_INTER
      sgpmInterFlag(pu);
      if (!pu.sgpmInter)
      {
#endif
#if JVET_AG0164_AFFINE_GPM
      bool isAffGPMValid = PU::isAffineGPMValid(pu);
      int  affGPMFlagCtxOffset = 0;

      affGPMFlagCtxOffset = PU::getAffGPMCtxOffset(pu);
#endif
#if JVET_W0097_GPM_MMVD_TM
#if JVET_Y0065_GPM_INTRA
      bool isIntra0 = false;
      bool isIntra1 = false;
      bool bUseOnlyOneVector = pu.cs->slice->isInterP() || pu.cs->sps->getMaxNumGeoCand() == 1;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
      bool isIbc0 = false;
      bool isIbc1 = false;
      bool isGpmInterIbcEnabled = pu.cs->sps->getUseGeoInterIbc();
#endif
      pu.geoMMVDFlag0 = m_BinDecoder.decodeBin(Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag0)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_0);
      }
#if JVET_AG0164_AFFINE_GPM
      else
      if (isAffGPMValid)
      {
        pu.affineGPM[0] = m_BinDecoder.decodeBin(Ctx::AffineFlag(affGPMFlagCtxOffset));
      }
      if (pu.geoMMVDFlag0 || pu.affineGPM[0])
      {
        isIntra0 = false;
      }
#endif
#if JVET_Y0065_GPM_INTRA
      else
      {
        isIntra0 = m_BinDecoder.decodeBin( Ctx::GPMIntraFlag() ) ? true : false;
#if JVET_AI0082_GPM_WITH_INTER_IBC
        if (isIntra0 && isGpmInterIbcEnabled)
        {
          isIbc0 = m_BinDecoder.decodeBin( Ctx::GpmInterIbcFlag() ) ? true : false;
        }
#endif
      }

      if (!bUseOnlyOneVector || isIntra0)
      {
#endif
      pu.geoMMVDFlag1 = m_BinDecoder.decodeBin(Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag1)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_1);
      }
#if JVET_AG0164_AFFINE_GPM
      else
      if (isAffGPMValid)
      {
        pu.affineGPM[1] = m_BinDecoder.decodeBin(Ctx::AffineFlag(affGPMFlagCtxOffset));
      }
      if (pu.geoMMVDFlag1 || pu.affineGPM[1])
      {
        isIntra1 = false;
      }
#endif
#if JVET_Y0065_GPM_INTRA
      else if (!isIntra0)
      {
        isIntra1 = m_BinDecoder.decodeBin( Ctx::GPMIntraFlag() ) ? true : false;
#if JVET_AI0082_GPM_WITH_INTER_IBC
        if (isIntra1 && isGpmInterIbcEnabled)
        {
          isIbc1 = m_BinDecoder.decodeBin( Ctx::GpmInterIbcFlag() ) ? true : false;
        }
#endif
      }
      }
      else
      {
        isIntra1 = true;
      }
      pu.gpmIntraFlag = (isIntra0 || isIntra1);
#if JVET_AI0082_GPM_WITH_INTER_IBC
      pu.gpmInterIbcFlag = (isIbc0 || isIbc1);
#endif
#endif
#if JVET_AJ0274_GPM_AFFINE_TM
      bool affGpmTmValid = isAffGPMValid && PU::isAffineGpmTmValid(pu);
#endif

#if TM_MRG
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if (isIntra0 || isIntra1)
        {
          pu.tmMergeFlag = false;
        }
        else
#endif
#if JVET_AJ0274_GPM_AFFINE_TM
        if (!affGpmTmValid && (pu.affineGPM[0] || pu.affineGPM[1]))
        {
          pu.tmMergeFlag = false;
        }
        else
#else
#if JVET_AG0164_AFFINE_GPM
        if (pu.affineGPM[0] || pu.affineGPM[1])
        {
          pu.tmMergeFlag = false;
        }
        else
#endif
#endif
        tm_merge_flag(pu);
        if (pu.tmMergeFlag)
        {
          pu.geoTmFlag0 = true;
          pu.geoTmFlag1 = true;
          geo_merge_idx(pu);
        }
        else
        {
          pu.geoTmFlag0 = false;
          pu.geoTmFlag1 = false;
#if JVET_Y0065_GPM_INTRA
          if (isIntra0 || isIntra1)
          {
#if JVET_AI0082_GPM_WITH_INTER_IBC
            geo_merge_idx1(pu, isIntra0, isIntra1, isIbc0, isIbc1);
#else
            geo_merge_idx1(pu, isIntra0, isIntra1);
#endif
          }
          else
#endif
          geo_merge_idx(pu);
        }
      }
#else
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if( isIntra0 || isIntra1 )
        {
          geo_merge_idx1( pu, isIntra0, isIntra1 );
        }
        else
#endif
        geo_merge_idx(pu);
      }
#endif
      else if (pu.geoMMVDFlag0 && pu.geoMMVDFlag1)
      {
        if (pu.geoMMVDIdx0 == pu.geoMMVDIdx1)
        {
          geo_merge_idx(pu);
        }
        else
        {
#if JVET_Y0065_GPM_INTRA
#if JVET_AI0082_GPM_WITH_INTER_IBC
          geo_merge_idx1(pu, isIntra0, isIntra1, isIbc0, isIbc1);
#else
          geo_merge_idx1(pu, isIntra0, isIntra1);
#endif
#else
          geo_merge_idx1(pu);
#endif
        }
      }
      else
      {
#if JVET_Y0065_GPM_INTRA
#if JVET_AI0082_GPM_WITH_INTER_IBC
        geo_merge_idx1(pu, isIntra0, isIntra1, isIbc0, isIbc1);
#else
        geo_merge_idx1(pu, isIntra0, isIntra1);
#endif
#else
        geo_merge_idx1(pu);
#endif
      }
#else
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      geoModeIdx(pu);
#else
      uint32_t splitDir = 0;
      xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
      pu.geoSplitDir          = splitDir;
#endif
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
      int numCandminus2 = maxNumGeoCand - 2;
      pu.mergeIdx       = 0;
      int mergeCand0    = 0;
      int mergeCand1    = 0;
      if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
      {
        mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
      }
      if (numCandminus2 > 0)
      {
        if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
        {
          mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
        }
      }
      mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
      pu.geoMergeIdx0 = mergeCand0;
      pu.geoMergeIdx1 = mergeCand1;
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir);
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", mergeCand0);
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", mergeCand1);
#endif
#if JVET_AL0134_SGPM_INTER
      }
#endif
      return;
    }

  if (pu.cu->predMode == MODE_IBC)
  {
#if JVET_AA0061_IBC_MBVD
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.ibcMbvdMergeFlag && (pu.interDir == 1 || pu.ibcMergeIdx1 >= 0))
#else
    if (pu.ibcMbvdMergeFlag)
#endif
    {
      return;
    }
#endif
#if JVET_AC0112_IBC_GPM
    if (pu.ibcGpmFlag)
    {
      pu.mergeIdx = pu.ibcGpmMergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS ? pu.ibcGpmMergeIdx0 : pu.ibcGpmMergeIdx1;
      return;
    }
#endif
    numCandminus1 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1;
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3 && pu.mergeFlag)
    {
      if (pu.ibcMbvdMergeFlag)
      {
        numCandminus1 = std::min(numCandminus1, IBC_MBVD_BASE_NUM - 1);
      }
      else
      {
        numCandminus1--;
      }
    }
#endif
  }
#if TM_MRG
  else if (pu.tmMergeFlag)
  {
#if JVET_X0141_CIIP_TIMD_TM
    if (pu.ciipFlag)
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumCiipTMMergeCand()) - 1;
    }
    else
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
    }
#else
    numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
#endif
  }
#endif
#if JVET_X0049_ADAPT_DMVR
  else if (pu.bmMergeFlag)
  {
    numCandminus1 = int(pu.cs->sps->getMaxNumBMMergeCand()) - 1;
  }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  if (pu.mergeOppositeLic)
  {
    numCandminus1 = int(pu.cs->sps->getMaxNumOppositeLicMergeCand()) - 1;
  }
  else if (pu.tmMergeFlagOppositeLic)
  {
    numCandminus1 = int(pu.cs->sps->getMaxNumTMOppositeLicMergeCand()) - 1;
  }
#endif
  if( numCandminus1 > 0 )
  {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    const CtxSet mrgIdxCtxSet = pu.tmMergeFlag ? Ctx::TmMergeIdx : Ctx::MergeIdx;
#endif
#if NON_ADJACENT_MRG_CAND
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
    {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#else
      if (!m_BinDecoder.decodeBin(Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#endif
      {
        break;
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.ibcMbvdMergeFlag)
    {
      pu.ibcMergeIdx1 = uiUnaryIdx;
    }
    else
#endif
    pu.mergeIdx = uiUnaryIdx;
#else
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    if (m_BinDecoder.decodeBin(mrgIdxCtxSet()))
#else
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
#endif
    {
      pu.mergeIdx++;
      for (; pu.mergeIdx < numCandminus1; pu.mergeIdx++)
      {
        if( !m_BinDecoder.decodeBinEP() )
        {
          break;
        }
      }
    }
#endif
  }

#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.ibcMbvdMergeFlag) 
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() ibc_merge_idx1=%d\n", pu.ibcMergeIdx1 ); 
  else 
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
#if JVET_X0049_ADAPT_DMVR
  if (pu.bmMergeFlag && pu.bmDir == 2)
  {
    pu.mergeIdx += BM_MRG_MAX_NUM_CANDS;
  }
#endif
  }
}

#if JVET_AE0169_BIPREDICTIVE_IBC
void CABACReader::ibcBiPredictionFlag( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_BIPRED, pu.lumaSize(), COMPONENT_Y);

  if (!pu.cs->slice->getBiPredictionIBCFlag())
  {
    pu.interDir = 1;
    return;
  }
  if (!pu.mergeFlag && (pu.cu->ibcLicFlag || pu.cu->rribcFlipType))
  {
    pu.interDir = 1;
    return;
  }
  pu.interDir = m_BinDecoder.decodeBin(Ctx::BiPredIbcFlag(pu.mergeFlag ? 0 : 1)) ? 3 : 1;

  DTRACE( g_trace_ctx, D_SYNTAX, "ibc_bi_prediction_flag() inter_dir=%d\n", pu.interDir );
}

void CABACReader::ibcMergeIdx1( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_MERGEIDX, pu.lumaSize(), COMPONENT_Y);

  if (pu.interDir != 3)
  {
    return;
  }

  
  int numCandminus2 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - pu.mergeIdx - 2;
  pu.ibcMergeIdx1   = pu.mergeIdx + 1;
  if( numCandminus2 > 0 )
  {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    const CtxSet mrgIdxCtxSet = pu.tmMergeFlag ? Ctx::TmMergeIdx : Ctx::MergeIdx;
#endif
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus2; ++uiUnaryIdx)
    {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#else
      if (!m_BinDecoder.decodeBin(Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#endif
      {
        break;
      }
    }
    pu.ibcMergeIdx1 += uiUnaryIdx;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "ibc_merge_idx1() ibc_merge_idx1=%d\n", pu.ibcMergeIdx1 );
}
#endif

#if JVET_W0097_GPM_MMVD_TM
void CABACReader::geo_mmvd_idx(PredictionUnit& pu, RefPicList eRefPicList)
{
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  int numStepCandMinus1 = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  int step = 0;
  if (m_BinDecoder.decodeBin(Ctx::GeoMmvdStepMvpIdx()))
  {
    step++;
    for (; step < numStepCandMinus1; step++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }

  int idxToMMVDStep[GPM_EXT_MMVD_REFINE_STEP] = { 1, 2, 3, 4, 5, 0, 6, 7, 8 };
  step = idxToMMVDStep[step];

  int direction = 0;
  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  direction = m_BinDecoder.decodeBinsEP(maxMMVDDir > 4 ? 3 : 2);
  int mvpIdx = (step * maxMMVDDir + direction);
  if (eRefPicList == REF_PIC_LIST_0)
  {
    pu.geoMMVDIdx0 = mvpIdx;
  }
  else
  {
    pu.geoMMVDIdx1 = mvpIdx;
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_idx() geo_mmvd_idx%d=%d\n", eRefPicList, mvpIdx);
}


void CABACReader::geo_merge_idx(PredictionUnit& pu)
{
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  uint32_t splitDir = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  pu.geoSplitDir = splitDir;
#endif
#if JVET_AG0164_AFFINE_GPM
  int maxNumGeoCand = pu.affineGPM[0] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
#else
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
#endif
#if JVET_AJ0274_GPM_AFFINE_TM
  if (pu.affineGPM[0] && pu.tmMergeFlag)
  {
    maxNumGeoCand = pu.cs->sps->getMaxNumGpmAffTmCand();
  }
#endif
  CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
  CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
  int numCandminus2 = maxNumGeoCand - 2;
  pu.mergeIdx = 0;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
#if JVET_AG0164_AFFINE_GPM
  CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
  mrgIdxCtxSet = pu.affineGPM[0]? Ctx::GpmAffMergeIdx: Ctx::GpmMergeIdx;
  for (; mergeCand0 < numCandminus2 + 1; ++mergeCand0)
  {
    if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((mergeCand0 > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : mergeCand0))))
    {
      break;
    }
  }
#else
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
  }
#endif

#if JVET_AG0164_AFFINE_GPM
  maxNumGeoCand = pu.affineGPM[1] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
#if JVET_AJ0274_GPM_AFFINE_TM
  if (pu.affineGPM[1] && pu.tmMergeFlag)
  {
    maxNumGeoCand = pu.cs->sps->getMaxNumGpmAffTmCand();
  }
#endif
  numCandminus2 = maxNumGeoCand - 2;
#endif

  if (numCandminus2 > 0
#if JVET_AG0164_AFFINE_GPM
    || (pu.affineGPM[0] != pu.affineGPM[1])
#endif
    )
  {
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[1] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    for (; mergeCand1 < numCandminus2 + ((pu.affineGPM[0] != pu.affineGPM[1]) ? 1 : 0); ++mergeCand1)
    {
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((mergeCand1 > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : mergeCand1))))
      {
        break;
      }
    }
#else
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
#if JVET_AG0164_AFFINE_GPM
      mergeCand1 += unary_max_eqprob(numCandminus2 - ((pu.affineGPM[0] != pu.affineGPM[1]) ? 0 : 1)) + 1;
#else
      mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
#endif
    }
#endif
  }
#if JVET_AG0164_AFFINE_GPM
  mergeCand1 += ((mergeCand1 >= mergeCand0) && (pu.affineGPM[0] == pu.affineGPM[1]) )? 1 : 0;
#else
  mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
#endif
  pu.geoMergeIdx0 = mergeCand0;
  pu.geoMergeIdx1 = mergeCand1;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx() geo_merge_idx0=%d geo_merge_idx1=%d\n", pu.geoMergeIdx0, pu.geoMergeIdx1);
}

#if JVET_Y0065_GPM_INTRA
#if JVET_AI0082_GPM_WITH_INTER_IBC
void CABACReader::geo_merge_idx1(PredictionUnit& pu, bool isIntra0, bool isIntra1, bool isIbc0, bool isIbc1)
#else
void CABACReader::geo_merge_idx1(PredictionUnit& pu, bool isIntra0, bool isIntra1)
#endif
#else
void CABACReader::geo_merge_idx1(PredictionUnit& pu)
#endif
{
  
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  uint32_t splitDir = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  pu.geoSplitDir = splitDir;
#endif
#if JVET_AG0164_AFFINE_GPM
  int maxNumGeoCand = pu.affineGPM[0] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
#if JVET_AJ0274_GPM_AFFINE_TM
  if (pu.affineGPM[0] && pu.tmMergeFlag)
  {
    maxNumGeoCand = pu.cs->sps->getMaxNumGpmAffTmCand();
  }
#endif
#else
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
#endif
#if JVET_Y0065_GPM_INTRA
  CHECK(maxNumGeoCand < 1, "Incorrect max number of geo candidates");
#else
  CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
#endif
  CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  int numCandminus2 = maxNumGeoCand - 2;
  pu.mergeIdx = 0;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
#if JVET_Y0065_GPM_INTRA
  if (isIntra0)
  {
#if JVET_AI0082_GPM_WITH_INTER_IBC
    if (isIbc0)
    {
      unsigned int uiUnaryIdx = 0;
      int numCandminus1 = GEO_MAX_NUM_IBC_CANDS - 1;
      for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
      {
        if (!m_BinDecoder.decodeBin(Ctx::GpmInterIbcIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
        {
          break;
        }
      }
#if JVET_AG0164_AFFINE_GPM
      mergeCand0 = GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS + uiUnaryIdx;
#else
      mergeCand0 = GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS + uiUnaryIdx;
#endif
    }
    else
    {
#if JVET_AG0164_AFFINE_GPM
      mergeCand0 = GEO_MAX_ALL_INTER_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
#else
      mergeCand0 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
#endif
    }
#else
#if JVET_AG0164_AFFINE_GPM
    mergeCand0 = GEO_MAX_ALL_INTER_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
#else
    mergeCand0 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
#endif
#endif
  }
  else if (numCandminus2 >= 0)
  {
#endif
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[0] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    for (; mergeCand0 < numCandminus2 + 1; ++mergeCand0)
    {
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((mergeCand0 > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : mergeCand0))))
      {
        break;
      }
    }
#else
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
  }
#endif
#if JVET_Y0065_GPM_INTRA
  }

#if JVET_AG0164_AFFINE_GPM
  maxNumGeoCand = pu.affineGPM[1] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
#if JVET_AJ0274_GPM_AFFINE_TM
  if (pu.affineGPM[1] && pu.tmMergeFlag)
  {
    maxNumGeoCand = pu.cs->sps->getMaxNumGpmAffTmCand();
  }
#endif
  numCandminus2 = maxNumGeoCand - 2;
#endif

  if (isIntra1)
  {
#if JVET_AI0082_GPM_WITH_INTER_IBC
    if (isIbc1)
    {
      unsigned int uiUnaryIdx = 0;
      int numCandminus1 = GEO_MAX_NUM_IBC_CANDS - 1;
      for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
      {
        if (!m_BinDecoder.decodeBin(Ctx::GpmInterIbcIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
        {
          break;
        }
      }
#if JVET_AG0164_AFFINE_GPM
      mergeCand1 = GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS + uiUnaryIdx;
#else
      mergeCand1 = GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS + uiUnaryIdx;
#endif
    }
    else
    {
#if JVET_AG0164_AFFINE_GPM
      mergeCand1 = GEO_MAX_ALL_INTER_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
#else
      mergeCand1 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
#endif
    }
#else
#if JVET_AG0164_AFFINE_GPM
    mergeCand1 = GEO_MAX_ALL_INTER_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
#else
    mergeCand1 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
#endif
#endif
  }
  else if (numCandminus2 >= 0)
  {
#endif
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[1] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    for (; mergeCand1 < numCandminus2 + 1; ++mergeCand1)
    {
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((mergeCand1 > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : mergeCand1))))
      {
        break;
      }
    }
#else
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
  }
#endif
#if JVET_Y0065_GPM_INTRA
  }
#endif
  pu.geoMergeIdx0 = mergeCand0;
  pu.geoMergeIdx1 = mergeCand1;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx1() geo_merge_idx0=%d geo_merge_idx1=%d\n", pu.geoMergeIdx0, pu.geoMergeIdx1);
}
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
void CABACReader::geoAdaptiveBlendingIdx( PredictionUnit& pu )
{
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
  int blkSizeSmall = pu.lwidth() < pu.lheight() ? pu.lwidth() : pu.lheight();
  int offset = (blkSizeSmall < GPM_BLENDING_SIZE_THRESHOLD) ? 0 : 4;
  int bin0 = m_BinDecoder.decodeBin(Ctx::GeoBldFlag(0 + offset));
  if (bin0 == 1)
  {
    pu.geoBldIdx = 2; //1
  }
  else
  {
    int bin1 = m_BinDecoder.decodeBin(Ctx::GeoBldFlag(1 + offset));
    if (bin1 == 0)
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::GeoBldFlag(3 + offset));
      if (bin2 == 0)
      {
        pu.geoBldIdx = 4; //000
      }
      else
      {
        pu.geoBldIdx = 3; //001
      }
    }
    else
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::GeoBldFlag(2 + offset));
      if (bin2 == 0)
      {
        pu.geoBldIdx = 1; //010
      }
      else
      {
        pu.geoBldIdx = 0; //011
      }
    }
  }
#else
  int bin0 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 0 ) );
  if( bin0 == 1 )
  {
    pu.geoBldIdx = 2; //1
  }
  else
  {
    int bin1 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 1 ) );
    if( bin1 == 0 )
    {
      int bin2 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 3 ) );
      if( bin2 == 0 )
      {
        pu.geoBldIdx = 4; //000
      }
      else
      {
        pu.geoBldIdx = 3; //001
      }
    }
    else
    {
      int bin2 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 2 ) );
      if( bin2 == 0 )
      {
        pu.geoBldIdx = 1; //010
      }
      else
      {
        pu.geoBldIdx = 0; //011
      }
    }
  }
#endif
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
  if (blkSizeSmall >= GPM_BLENDING_SIZE_THRESHOLD)
  {
    pu.geoBldIdx++;
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_adaptive_blending_idx() geo_bld_idx=%d\n", pu.geoBldIdx);
}
#endif

#if JVET_AL0134_SGPM_INTER
void CABACReader::sgpmInterFlag(PredictionUnit &pu)
{
  if (!(pu.cs->slice->getSPS()->getUseSgpmInter() && pu.lx() && pu.ly()))
  {
    pu.sgpmInter = false;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    pu.sgpmAffine = false;
    pu.sgpmIntra = false;
#endif
    return;
  }

  pu.sgpmInter = m_BinDecoder.decodeBin(Ctx::SgpmInterFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_inter_flag() pos=(%d,%d) sgpm_inter_flag=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmInter);
  if (pu.sgpmInter)
  {
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    int  affGPMFlagCtxOffset = PU::getAffGPMCtxOffset(pu);
    pu.sgpmAffine = false;
    if (CU::isSgpmAffineAllowed(*pu.cu))
    {
      pu.sgpmAffine = m_BinDecoder.decodeBin(Ctx::SgpmAffineFlag(affGPMFlagCtxOffset));
      DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_affine_flag() pos=(%d,%d) sgpm_inter_flag=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmAffine);
    }
    if (pu.sgpmAffine)
    {
      uint32_t sgpmAffineIdx = 0;
      const int maxSgpmAffineNum = pu.cs->slice->getCheckLDB() ? SGPM_AFFINE_INTRA_NUM_LD: SGPM_AFFINE_NUM;
      xReadTruncBinCode(sgpmAffineIdx, maxSgpmAffineNum);
      pu.sgpmInterIdx = sgpmAffineIdx;
      DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_affine_flag() pos=(%d,%d) sgpm_inter_idx=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmInterIdx);
    }
    else
    {
      pu.sgpmIntra = false;
      if (CU::isSgpmIntraAllowed(*pu.cu))
      {
        pu.sgpmIntra = m_BinDecoder.decodeBin(Ctx::SgpmIntraFlag());
        DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_intra_flag() pos=(%d,%d) sgpm_inter_flag=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmIntra);
      }
      if (!pu.cs->slice->getCheckLDB() && !pu.sgpmIntra)
#else
    if (!pu.cs->slice->getCheckLDB())
#endif
    {
      pu.sgpmInterTm = m_BinDecoder.decodeBin(Ctx::SgpmInterTmFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_inter_tm_flag() pos=(%d,%d) sgpm_inter_tm_flag=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmInterTm);
    }
    else
    {
      pu.sgpmInterTm        = false;
    }

    uint32_t sgpmInterIdx = 0;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    const int maxSgpmNum = pu.sgpmIntra ? (pu.cs->slice->getCheckLDB() ? SGPM_AFFINE_INTRA_NUM_LD: SGPM_INTER_INTRA_NUM): SGPM_INTER_NUM;
    xReadTruncBinCode(sgpmInterIdx, maxSgpmNum);
#else
    xReadTruncBinCode(sgpmInterIdx, SGPM_INTER_NUM);
#endif
    pu.sgpmInterIdx = sgpmInterIdx;
    DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_inter_flag() pos=(%d,%d) sgpm_inter_idx=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.sgpmInterIdx);
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
    }
#endif
  }
}
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void CABACReader::geoModeIdx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  if (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode())
  {
    uint32_t geoMode = 0;
    xReadTruncBinCode(geoMode, GEO_NUM_PARTITION_MODE);
    pu.geoSplitDir = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%u\n", geoMode);
    return;
  }

  const int maxNumBins    = (GEO_NUM_SIG_PARTMODE / GEO_SPLIT_MODE_RICE_CODE_DIVISOR) - 1;
  const int maxNumCtxBins = 5;
        int geoModePrefix = 0;
  for (int binIdx = 0; binIdx < maxNumBins; ++binIdx, ++geoModePrefix)
  {
    if (binIdx < maxNumCtxBins)
    {
      if (!m_BinDecoder.decodeBin(Ctx::GeoSubModeIdx(binIdx)))
      {
        break;
      }
    }
    else
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }

  if (GEO_SPLIT_MODE_RICE_CODE_DIVISOR > 1)
  {
    uint32_t geoModeSuffix = 0;
    xReadTruncBinCode(geoModeSuffix, GEO_SPLIT_MODE_RICE_CODE_DIVISOR);
    uint32_t geoMode = ((uint32_t)(geoModePrefix * GEO_SPLIT_MODE_RICE_CODE_DIVISOR)) + geoModeSuffix;
    pu.geoSyntaxMode = geoMode;
    pu.geoSplitDir   = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d(prefix=%d suffix=%u)\n", geoMode, geoModePrefix, geoModeSuffix);
  }
  else
  {
    uint32_t geoMode = (uint32_t)geoModePrefix;
    pu.geoSyntaxMode = geoMode;
    pu.geoSplitDir   = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%u\n", geoMode);
  }
}
#endif

void CABACReader::mmvd_merge_idx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  unsigned int uiUnaryIdx = 0;
  int var0 = 0;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numCandMinus1Base = std::min<int>(MMVD_BASE_MV_NUM, pu.cs->sps->getMaxNumMergeCand()) - 1;
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
    if (m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx(0)))
    {
      var0++;
      for (; var0 < numCandMinus1Base; var0++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx(var0)))
        {
          break;
        }
      }
    }
  }
#else
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() base_mvp_idx=%d\n", var0);
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  ((MMVD_MAX_REFINE_NUM >> (ricePar+MMVD_SIZE_SHIFT)))/MMVD_BI_DIR  - 1;
#else
  int numStepCandMinus1 =  (MMVD_MAX_REFINE_NUM >> (ricePar+MMVD_SIZE_SHIFT))  - 1;
#endif
  int temp = 0;
  temp = m_BinDecoder.decodeBinsEP(ricePar);
  
  for (; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
  pu.mmvdMergeIdx = var0 * MMVD_MAX_REFINE_NUM  +uiUnaryIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  int var0 = 0;
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
#if !JVET_AA0093_ENHANCED_MMVD_EXTENSION
    static_assert(MMVD_BASE_MV_NUM == 2, "");
#endif
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);
  int numStepCandMinus1 = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                          VVC_MMVD_REFINE_STEP - 1;
#else
                          MMVD_REFINE_STEP - 1;
#endif
  int var1 = 0;

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdxECM3()))
#else
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
#endif
  {
    var1++;
    for (; var1 < numStepCandMinus1; var1++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_step_mvp_idx() mmvd_step_mvp_idx=%d\n", var1);
  int var2 = 0;
  if (m_BinDecoder.decodeBinEP())
  {
    var2 += 2;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  else
  {
    var2 += 0;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  int mvpIdx = (var0 * VVC_MMVD_MAX_REFINE_NUM + var1 * VVC_MMVD_MAX_DIR + var2);
#else
  int mvpIdx = (var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2);
#endif
  pu.mmvdMergeIdx = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#endif
}

void CABACReader::inter_pred_idc( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__INTER_DIR, pu.lumaSize());

  if( pu.cs->slice->isInterP() )
  {
    pu.interDir = 1;
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, pu.lumaPos().x, pu.lumaPos().y );
      pu.interDir = 3;
      return;
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (pu.cs->sps->getUseARL())
  {
    pu.interDir = 1; // temporally set as 1 for enabling refIdxLC()
    return;
  }
#endif
#if CTU_256
  if( m_BinDecoder.decodeBin( Ctx::InterDir( 7 ) ) )
#else
  if( m_BinDecoder.decodeBin( Ctx::InterDir(6) ) )
#endif
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", 2, pu.lumaPos().x, pu.lumaPos().y );
    pu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
  pu.interDir = 1;
  return;
}

#if JVET_Z0054_BLK_REF_PIC_REORDER
void CABACReader::refIdxLC(PredictionUnit& pu)
{  
  if (!PU::useRefCombList(pu))
  {
    return;
  }
 
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize());
  int numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedList().size() - 1;
  int refIdxLC = 0;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedListAmvpMerge().size() - 1;
  }
#endif
  if (numRefMinus1 > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::RefPicLC(0)))
    {
      refIdxLC++;
      for (; refIdxLC < numRefMinus1; refIdxLC++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::RefPicLC(std::min(refIdxLC, 2))))
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refIdxLC() value=%d pos=(%d,%d)\n", refIdxLC, pu.lumaPos().x, pu.lumaPos().y);
  pu.refIdxLC = refIdxLC;
  // temporally set interDir and refIdx (has to be done)
  if (pu.interDir != 3)
  {
    pu.interDir = 1;
  }
  pu.refIdx[0] = 0;
}

void CABACReader::refPairIdx(PredictionUnit& pu)
{
  if (!PU::useRefPairList(pu))
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize());
  int numRefMinus1 = (int)pu.cs->slice->getRefPicPairList().size() - 1;
  int refPairIdx = 0;
  if (numRefMinus1 > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::RefPicLC(0)))
    {
      refPairIdx++;
      for (; refPairIdx < numRefMinus1; refPairIdx++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::RefPicLC(std::min(refPairIdx, 2))))
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refPairIdx() value=%d pos=(%d,%d)\n", refPairIdx, pu.lumaPos().x, pu.lumaPos().y);
  pu.refPairIdx = refPairIdx;
  // temporally set refIdx
  pu.refIdx[0] = 0;
  pu.refIdx[1] = 0;
}
#endif

void CABACReader::ref_idx( PredictionUnit &pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize());

  if ( pu.cu->smvdMode )
  {
    pu.refIdx[eRefList] = pu.cs->slice->getSymRefIdx( eRefList );
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (PU::useRefCombList(pu) || PU::useRefPairList(pu))
  {
    return;
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[1 - eRefList])
  {
#if JVET_Y0128_NON_CTC
    if (pu.cs->slice->getAmvpMergeModeOnlyOneValidRefIdx(eRefList) >= 0)
    {
      pu.refIdx[eRefList] = pu.cs->slice->getAmvpMergeModeOnlyOneValidRefIdx(eRefList);
      return;
    }
#else
    const RefPicList refListAmvp = eRefList;
    const RefPicList refListMerge = RefPicList(1 - eRefList);
    const int curPoc = pu.cs->slice->getPOC();
    const int numRefAmvp = pu.cs->slice->getNumRefIdx(refListAmvp);
    const int numRefMerge = pu.cs->slice->getNumRefIdx(refListMerge);
    int candidateRefIdxCount = 0;
    int onlyOneValidRefIdxAmvp = -1;
    for (int refIdxAmvp = 0; refIdxAmvp < numRefAmvp; refIdxAmvp++)
    {
      const int amvpPoc = pu.cs->slice->getRefPOC(refListAmvp, refIdxAmvp);
      bool validCandidate = false;
      for (int refIdxMerge = 0; refIdxMerge < numRefMerge; refIdxMerge++)
      {
        const int mergePoc = pu.cs->slice->getRefPOC(refListMerge, refIdxMerge);
        if ((amvpPoc - curPoc) * (mergePoc - curPoc) < 0)
        {
          validCandidate = true;
        }
      }
      if (validCandidate)
      {
        onlyOneValidRefIdxAmvp = refIdxAmvp;
        candidateRefIdxCount++;
      }
    }
    CHECK(candidateRefIdxCount == 0, "this is not possible");
    if (candidateRefIdxCount == 1)
    {
      pu.refIdx[eRefList] = onlyOneValidRefIdxAmvp;
      return;
    }
#endif
  }
#endif

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    pu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
    pu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      pu.refIdx[eRefList] = (signed char)(idx - 1);
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
  }
}

#if MULTI_HYP_PRED
int CABACReader::ref_idx_mh(const int numRef)
{

  if (numRef <= 1 || !m_BinDecoder.decodeBin(Ctx::MHRefPic()))
  {
    return 0;
  }
  if (numRef <= 2 || !m_BinDecoder.decodeBin(Ctx::MHRefPic(1)))
  {
    return 1;
  }
  for (int idx = 3; ; idx++)
  {
    if (numRef <= idx || !m_BinDecoder.decodeBinEP())
    {
      return(idx - 1);
    }
  }
}
#endif


void CABACReader::mvp_flag( PredictionUnit& pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MVP_IDX, pu.lumaSize());

#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[eRefList])
  {
    return;
  }
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.amvpMergeModeFlag[1 - eRefList] == true && !CU::isIBC(*pu.cu))
#else
  if (pu.amvpMergeModeFlag[1 - eRefList] == true)
#endif
  {
    unsigned mvpIdx = 0;
#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (pu.cs->sps->getUseARL())
    {
      RefListAndRefIdx refListComb = pu.cs->slice->getRefPicCombinedListAmvpMerge()[pu.refIdxLC];
      if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(refListComb.refList, refListComb.refIdx)) == false)
      {
        mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
      }
    }
    else
#endif
      if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
#else
    if (!pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu))
#endif
#endif
    {
      mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
    }
    if (mvpIdx == 0)
    {
      pu.mvpIdx [eRefList] = mvpIdx + m_BinDecoder.decodeBinEP();
    }
    else
    {
      pu.mvpIdx [eRefList] = mvpIdx + 1;
    }
    return;
  }
#endif
#if TM_AMVP
#if JVET_Y0128_NON_CTC || JVET_AA0132_CONFIGURABLE_TM_TOOLS
  unsigned mvpIdx = 0;
  if (pu.cu->affine || CU::isIBC(*pu.cu))
  {
    mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  else if (PU::useRefCombList(pu))
  {
    mvpIdx = pu.refIdxLC >= pu.cs->slice->getNumNonScaledRefPic()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          || !pu.cs->sps->getUseTMAmvpMode()
#else
          || !pu.cs->sps->getUseDMVDMode()
#endif
           ? m_BinDecoder.decodeBin(Ctx::MVPIdx()) : 0;
  }
  else if (PU::useRefPairList(pu))
  {
    mvpIdx = pu.refPairIdx >= pu.cs->slice->getNumNonScaledRefPicPair()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          || !pu.cs->sps->getUseTMAmvpMode()
#else
          || !pu.cs->sps->getUseDMVDMode()
#endif
           ? m_BinDecoder.decodeBin(Ctx::MVPIdx()) : 0;
  }
#endif
  else if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
  {
    mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  }
#else
  unsigned mvpIdx = !pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu) ? m_BinDecoder.decodeBin( Ctx::MVPIdx() ) : 0;
#endif
#else
  unsigned mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvpIdx, pu.lumaPos().x, pu.lumaPos().y );
  pu.mvpIdx [eRefList] = mvpIdx;
#if !JVET_Z0054_BLK_REF_PIC_REORDER
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvpIdx );
#endif
}
#if JVET_AG0135_AFFINE_CIIP
void CABACReader::ciipAffineFlag(PredictionUnit& pu)
{
  if (!pu.cu->slice->isIntra() && pu.cs->sps->getUseCiipAffine() && (pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && 
#if JVET_AJ0085_SUBBLOCK_MERGE_MODE_EXTENSION
    CU::isAffineAllowed(*pu.cu)
#else
    pu.cu->lumaSize().width >= 8 && pu.cu->lumaSize().height >= 8
#endif
    )
  {
    unsigned ctxId = DeriveCtx::CtxCiipAffineFlag(*pu.cu);
#if JVET_AJ0085_SUBBLOCK_MERGE_MODE_EXTENSION
    if (CU::affineCtxInc(*pu.cu))
    {
      ctxId += 3;
    }
#endif
    pu.ciipAffine = m_BinDecoder.decodeBin(Ctx::CiipAffineFlag(ctxId));
  }
  else
  {
    pu.ciipAffine = false;
  }
}
#endif
void CABACReader::Ciip_flag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseCiip())
  {
    pu.ciipFlag = false;
#if CIIP_PDPC
    pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
    pu.ciipAffine = false;
#endif
    return;
  }
  if (pu.cu->skip)
  {
    pu.ciipFlag = false;
#if CIIP_PDPC
    pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
    pu.ciipAffine = false;
#endif
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MH_INTRA_FLAG, pu.lumaSize());

  pu.ciipFlag = (m_BinDecoder.decodeBin(Ctx::CiipFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=%d pos=(%d,%d) size=%dx%d\n", pu.ciipFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
#if CIIP_PDPC
  if( pu.ciipFlag )
  {
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AG0135_AFFINE_CIIP
      ciipAffineFlag(pu);
      if (!pu.ciipAffine)
      {
#endif
        tm_merge_flag(pu);
#if JVET_AG0135_AFFINE_CIIP
      }
      else
      {
        pu.cu->affine = true;
      }
#endif
#else
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
      DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
    }
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
    if (!pu.ciipAffine)
    {
#endif
      pu.ciipPDPC = (m_BinDecoder.decodeBin(Ctx::CiipFlag(1)));
#if JVET_AG0135_AFFINE_CIIP
    }
    else
    {
      pu.ciipPDPC = false;
    }
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_pdpc_flag=%d pos=(%d,%d) size=%dx%d\n", pu.ciipPDPC ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  }
#else
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag && pu.cs->slice->getSPS()->getUseCiipTmMrg())
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    tm_merge_flag(pu);
#else
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
#endif
  }
#endif
#endif
}

#if MULTI_HYP_PRED
void CABACReader::mh_pred_data(PredictionUnit& pu)
{
  CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
  if (!pu.cs->sps->getUseInterMultiHyp() || !pu.cs->slice->isInterB())
  {
    return;
  }
  if (pu.ciipFlag)
  {
    return;
  }
  if (pu.cu->geoFlag)
  {
    return;
  }
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  if (pu.tmMergeFlag)
  {
    return;
  }
#endif
#if JVET_X0049_ADAPT_DMVR
  if (pu.bmMergeFlag)
  {
    return;
  }
#endif
#if !JVET_Z0083_PARSINGERROR_FIX
  if (!pu.mergeFlag && pu.cu->affine && pu.cu->imv)
  {
    return;
  }
#endif
  if (!pu.mergeFlag && pu.cu->bcwIdx == BCW_DEFAULT)
  {
    return;
  }
  if( CU::isIBC( *pu.cu ) )
  {
    return;
  }
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif

  if (pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(pu.Y().width, pu.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    return;
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mh(STATS__CABAC_BITS__MH_INTER, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mvpIdx(STATS__CABAC_BITS__MVP_IDX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mergeIdx(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_refIdx(STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mvd(STATS__CABAC_BITS__MVD, pu.lumaSize().width, pu.lumaSize().height);
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mh);

  const int numMHRef = pu.cs->slice->getNumMultiHypRefPics();
  CHECK(numMHRef <= 0, "Multi Hyp: numMHRef <= 0");
  int hypIdx = 0;
  const size_t maxNumAddHyps = pu.cs->sps->getMaxNumAddHyps();
  while (pu.addHypData.size() < maxNumAddHyps && m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(hypIdx)) == 1)
  {
    if (hypIdx < 1)
    {
      hypIdx++;
    }
    MultiHypPredictionData mhData;
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mh);
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
    const int maxNumMHPCand = pu.cs->sps->getMaxNumMHPCand();
    if (maxNumMHPCand > 0 && m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(2)))
    {
      mhData.isMrg = true;
      int numCandminus2 = maxNumMHPCand - 2;
#else
    if (m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(2)))
    {
      mhData.isMrg = true;
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      int numCandminus2 = maxNumGeoCand - 2;
#endif
      int mergeCand0 = 0;
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mergeIdx);
      if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
      {
        mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
      }
      mhData.mrgIdx = mergeCand0;
      mhData.weightIdx = unary_max_symbol(Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
      pu.addHypData.push_back(mhData);
      continue;
    }
    mhData.isMrg = false;

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_refIdx);
    mhData.refIdx = ref_idx_mh(numMHRef);
    CHECK(mhData.refIdx < 0, "Multi Hyp: mhData.refIdx < 0");
    CHECK(mhData.refIdx >= numMHRef, "Multi Hyp: mhData.refIdx >= numMHRef");
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvd);
#if JVET_AD0140_MVD_PREDICTION 
    mvd_coding(mhData.mvd, nullptr, true);
#else
    mvd_coding(mhData.mvd);
#endif
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvpIdx);
    mhData.mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
    mhData.weightIdx = unary_max_symbol(Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
    pu.addHypData.push_back(mhData);
  }
}
#endif



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx
  , const PartSplit ispType, const int subTuIdx
#if JVET_AE0102_LFNST_CTX
  , const bool codeTuCoeff
#endif
)
{
  const UnitArea&   area = partitioner.currArea();
  CodingUnit&         cu = *cs.getCU(area.blocks[partitioner.chType], partitioner.chType);
  int       subTuCounter = subTuIdx;

  // split_transform_flag
  bool split = partitioner.canSplit(TU_MAX_TR_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  );
  const unsigned  trDepth = partitioner.currTrDepth;

  if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs 
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  ) )
  {
    split = true;
  }

  if( !split && cu.ispMode )
  {
    split = partitioner.canSplit( ispType, cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );
  }

  if( split )
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ))
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE(g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n",
             partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height);

#endif
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if (cu.ispMode)
    {
      partitioner.splitCurrArea(ispType, cs);
    }
    else if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ))
    {
      partitioner.splitCurrArea(PartSplit(cu.getSbtTuSplit()), cs);
    }
    else
    {
      THROW("Implicit TU split not available!");
    }

    do
    {
      transform_tree( cs, partitioner, cuCtx,          ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit &tu = cs.addTU( CS::getArea( cs, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *cs.pcv );
    tu.checkTuNoResidual( partitioner.currPartIdx() );

    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
#if !REMOVE_PCM
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
#endif
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit(tu, cuCtx, partitioner, subTuCounter);
  }
}

bool CABACReader::cbf_comp( CodingStructure& cs, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  unsigned  ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
  const CtxSet &ctxSet = Ctx::QtCbf[area.compID];
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__QT_CBF, area.size(), area.compID);
  unsigned  cbf = 0;
  if( (area.compID == COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmMode)
   || (area.compID != COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmModeChroma))
  {
    if (area.compID == COMPONENT_Y)
    {
      ctxId = 1;
    }
    else if (area.compID == COMPONENT_Cb)
    {
      ctxId = 1;
    }
    else
    {
      ctxId = 2;
    }
    cbf = m_BinDecoder.decodeBin(ctxSet(ctxId));
  }
  else
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================

#if JVET_AD0140_MVD_PREDICTION
void CABACReader::mvd_coding( Mv &rMvd, MvdSuffixInfo* const pSi
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                            , bool codeSign
#endif
)
{
  const bool ctxCoding = nullptr != pSi && !codeSign;

  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = 0, verAbs = 0;
  
  horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  const int iEgcOffset = (!codeSign && ctxCoding) ? pSi->getEGCOffset() : 1;
  if ( iEgcOffset != 0 )
  {
    // abs_mvd_greater1_flag[ 0 | 1 ]
    if (horAbs)
    {
      horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
    }
    if (verAbs)
    {
      verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
    }
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  int horParam = -1;
  int verParam = -1;
  if (!codeSign && ctxCoding)
  {    
    horParam = horAbs > iEgcOffset ? xReadMvdPrefix(MVD_CODING_GOLOMB_ORDER) : -1;
    verParam = verAbs > iEgcOffset ? xReadMvdPrefix(MVD_CODING_GOLOMB_ORDER) : -1;
#if ENABLE_TRACING
    if (horParam == -1)
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_hor) = %d \n", horAbs);
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() mvd_hor prefix=%d \n", horParam);
    }
    if (verParam == -1)
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_ver) = %d \n", verAbs);
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() mvd_ver prefix=%d \n", verParam);
    }
#endif
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() code_sign=%d \n", codeSign);
    if (horAbs)
    {
      if (horAbs > 1)
      {
        horAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
      }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (codeSign)
#endif
      if (m_BinDecoder.decodeBinEP())
      {
        horAbs = -horAbs;
      }
    }
    if (verAbs)
    {
      if (verAbs > 1)
      {
        verAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
      }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (codeSign)
#endif
      if (m_BinDecoder.decodeBinEP())
      {
        verAbs = -verAbs;
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_hor)=%d \n", horAbs > 0 ? horAbs : -horAbs);
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_ver)=%d \n", verAbs > 0 ? verAbs : -verAbs);
  }
  rMvd = Mv(horAbs, verAbs);
  if (ctxCoding)
  {
    pSi->horPrefix = horParam;
    pSi->verPrefix = verParam;
  }
}

unsigned CABACReader::xReadMvdPrefix( int param )
{
  unsigned bit    = 1;
  unsigned uiIdx  = 0;

  while (bit)
  {
    bit = m_BinDecoder.decodeBinEP();
    uiIdx++;
  }

  --uiIdx;

  return uiIdx;
}

unsigned CABACReader::xReadMvdContextSuffix(int symbol, int param)
{
  unsigned bit = 0;
  ++param;
  CHECK(param < 0, "param < 0");
  if (0 != param)
  {
    bit = m_BinDecoder.decodeBinsEP(param);
    symbol += bit;
  }
  return symbol;
}

void CABACReader::mvdCodingRemainder(Mv& rMvd, MvdSuffixInfo& si, const int imv)
{
  int horAbs = rMvd.getAbsHor();
  int verAbs = rMvd.getAbsVer();

  const int horParam = si.horPrefix;
  const int verParam = si.verPrefix;

  unsigned int& horOffsetPrediction = si.horOffsetPrediction;
  unsigned int& verOffsetPrediction = si.verOffsetPrediction;
  horOffsetPrediction = 0;
  verOffsetPrediction = 0;

  const int numMSBhor = std::max(0, si.horOffsetPredictionNumBins);
  const int numMSBver = std::max(0, si.verOffsetPredictionNumBins);

  unsigned horSuffix = 0;
  unsigned verSuffix = 0;

  if (horParam >= 0 || verParam >= 0) 
  {    
    const int iEgcOffset = si.getEGCOffset() + 1;

    if (horParam >= 0)
    {  
      for (int i = numMSBhor - 1; i >= 0; --i)
      {
        const int prev2Bin = (i + 1 > numMSBhor - 1) ? -1
                           : (i + 1 == numMSBhor - 1) ? si.horSignHypMatch
                                       : /*otherwise*/ (horOffsetPrediction >> 1) & 1;
        const int prevBin  = (i == numMSBhor - 1) ? si.horSignHypMatch : (horOffsetPrediction & 1);
        const int imvShift = MotionModelCheck::isAffine(si.m_motionModel) ? Mv::getImvPrecShiftAffineMvd(imv) : Mv::getImvPrecShiftMvd(imv);
        const int iCtxIdx  = DeriveCtx::ctxSmMvdBin(prev2Bin, prevBin, true, i + imvShift, si.m_motionModel);

        horOffsetPrediction <<= 1;

        int bin = (int)m_BinDecoder.decodeBin(Ctx::MvsdIdxMVDMSB(iCtxIdx));
        bin = (0 == bin) ? 1 : 0;
        horOffsetPrediction |= bin;
      }

      DTRACE(g_trace_ctx, D_SYNTAX, "Codeword for MVD suffix prediction bins for horizontal component: %d \n", horOffsetPrediction);
      DTRACE(g_trace_ctx, D_SYNTAX, "Number of MVD suffix prediction bins for horizontal component: %d \n", numMSBhor);

      CHECK(horParam < 0, "horParam < 0");
      CHECK(horParam + 1 < numMSBhor, "horParam + 1 < numMSBhor");

      DTRACE(g_trace_ctx, D_SYNTAX, "Number of explicitly coded MVD horizontal suffix bins: %d \n", horParam - numMSBhor + 1);
      horSuffix = xReadMvdContextSuffix(si.horPrefixGroupStartValue, horParam - numMSBhor );
      horAbs = horSuffix + iEgcOffset;
    }
    if (verParam >= 0)
    {
      for (int i = numMSBver - 1; i >= 0; --i)
      {
        const int prev2Bin = (i + 1 > numMSBver - 1) ? -1
                           : (i + 1 == numMSBver - 1) ? si.verSignHypMatch
                                       : /*otherwise*/ (verOffsetPrediction >> 1) & 1;
        const int prevBin  = (i == numMSBver - 1) ? si.verSignHypMatch : (verOffsetPrediction & 1);
        const int imvShift = MotionModelCheck::isAffine(si.m_motionModel) ? Mv::getImvPrecShiftAffineMvd(imv) : Mv::getImvPrecShiftMvd(imv);
        const int iCtxIdx  = DeriveCtx::ctxSmMvdBin(prev2Bin, prevBin, false, i + imvShift, si.m_motionModel);

        verOffsetPrediction <<= 1;

        int bin = (int)m_BinDecoder.decodeBin(Ctx::MvsdIdxMVDMSB(iCtxIdx));
        bin = (0 == bin) ? 1 : 0;
        verOffsetPrediction |= bin;
      }

      DTRACE(g_trace_ctx, D_SYNTAX, "Codeword for MVD suffix prediction bins for vertical component: %d \n", verOffsetPrediction);
      DTRACE(g_trace_ctx, D_SYNTAX, "Number of MVD suffix prediction bins for vertical component: %d \n", numMSBver);

      CHECK(verParam < 0, "verParam < 0");
      CHECK(verParam + 1 < numMSBver, "verParam + 1 < numMSBver");

      DTRACE(g_trace_ctx, D_SYNTAX, "Number of explicitly coded MVD vertical suffix bins: %d \n", verParam - numMSBver + 1);
      verSuffix = xReadMvdContextSuffix(si.verPrefixGroupStartValue, verParam - numMSBver );
      verAbs = verSuffix + iEgcOffset;
    }
    rMvd = Mv(horAbs, verAbs);

    CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal MVD value");
  }
  else
  {
    unsigned horSuffix = 0;
    unsigned verSuffix = 0;
    if (horAbs != 0)
    {
      int horGolombMin = si.horPrefixGroupStartValue;

      horSuffix = xReadMvdContextSuffix(horGolombMin, horParam );
      horAbs = horSuffix + 2;
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() hor_suffix=%d \n", horSuffix);
    }

    if (verAbs != 0)
    {
      int verGolombMin = si.verPrefixGroupStartValue;

      verSuffix = xReadMvdContextSuffix(verGolombMin, verParam );
      verAbs = verSuffix + 2;
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() ver_uffix=%d \n", verSuffix);
    }

    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() abs(mvd)=(%d,%d) \n", horAbs, verAbs);
    rMvd = Mv(horAbs, verAbs);
  }
}

#else

#if JVET_AA0070_RRIBC
void CABACReader::mvd_coding( Mv &rMvd
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  , bool codeSign
#endif
  , const int &rribcFlipType
)
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = 0, verAbs = 0;
  if (rribcFlipType != 2)
  {
    horAbs = (int) m_BinDecoder.decodeBin(Ctx::Mvd());
  }
  if (rribcFlipType != 1)
  {
    verAbs = (int) m_BinDecoder.decodeBin(Ctx::Mvd());
  }
#else
void CABACReader::mvd_coding( Mv &rMvd
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  , bool codeSign
#endif
)
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
#endif

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal MVD value");
}

#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
unsigned CABACReader::xReadBvdContextPrefix(unsigned ctxT, int offset, int param )
{
  unsigned bit    = 1;
  unsigned uiIdx = 0;

  while (bit)
  {
    if (uiIdx >= ctxT)
    {
      bit = m_BinDecoder.decodeBinEP();
    }
    else
    {
      bit = m_BinDecoder.decodeBin(Ctx::Bvd( offset + uiIdx + 1));
    }

    uiIdx++;
  }

  --uiIdx;

  return uiIdx;
}

unsigned CABACReader::xReadBvdContextSuffix(int symbol, int param )
{
  unsigned bit = 0;
  ++param;
  CHECK(param < 0,"param < 0");
  if (0!=param)
  {
    CHECK(param == 0, "param == 0");

    bit = m_BinDecoder.decodeBinsEP(param);    

    symbol += bit;
  }
  return symbol;
}
#endif

unsigned CABACReader::xReadBvdContext(unsigned ctxT, int offset, int param )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  unsigned uiIdx = 0;
  while( bit )
  {
    if (uiIdx >= ctxT)
    {
      bit = m_BinDecoder.decodeBinEP();
    }
    else
    {
      bit = m_BinDecoder.decodeBin(Ctx::Bvd( offset + uiIdx + 1));
    }
    uiIdx++;
    symbol += bit << param++;
  }
  if( --param )
  {
    bit = m_BinDecoder.decodeBinsEP(param);
    symbol += bit;
  }
  return symbol;
}

#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
void CABACReader::bvdCodingRemainder(Mv& rMvd, MvdSuffixInfo& si, const int imv )
{
  int horAbs = rMvd.getAbsHor();
  int verAbs = rMvd.getAbsVer();

  const int horParam = si.horPrefix;
  const int verParam = si.verPrefix;

  unsigned int& horOffsetPrediction = si.horOffsetPrediction;
  unsigned int& verOffsetPrediction = si.verOffsetPrediction;
  horOffsetPrediction = 0;
  verOffsetPrediction = 0;

  const int numMSBhor = si.horOffsetPredictionNumBins;
  const int numMSBver = si.verOffsetPredictionNumBins;

  unsigned horSuffix = 0;
  unsigned verSuffix = 0;

  if (horParam >= 0 || verParam >= 0) 
  {    

    if (horParam >= 0)
    {
      for (int i = numMSBhor - 1; i >= 0; --i)
      {
        const int prev2Bin = (i + 1 > numMSBhor  - 1) ? -1 : 
                             (i + 1 == numMSBhor - 1) ? si.horSignHypMatch : 
                                     /*otherwise*/      (horOffsetPrediction>>1) & 1;
        const int prevBin  = (i == numMSBhor - 1) ? si.horSignHypMatch : (horOffsetPrediction & 1);
        const int imvShift = Mv::getImvPrecShift(imv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                               , si.isFracBvEnabled
#endif
        );
        const int iCtxIdx = DeriveCtx::CtxSmBvdBin(prev2Bin, prevBin, true, i + imvShift);

        horOffsetPrediction <<= 1;

        int bin = (int)m_BinDecoder.decodeBin(Ctx::MvsdIdxBVDMSB(iCtxIdx));

        bin = (0 == bin) ? 1 : 0;
        horOffsetPrediction |= bin;
      }

      CHECK(horParam < 0, "horParam < 0");
      CHECK(horParam + 1 < numMSBhor, "horParam + 1 < numMSBhor");

      horSuffix = xReadBvdContextSuffix(si.horPrefixGroupStartValue, horParam - numMSBhor );
      horAbs = horSuffix + 1;
    }
    if (verParam >= 0)
    {
      for (int i = numMSBver - 1; i >= 0; --i)
      {
        const int prev2Bin = (i + 1 > numMSBver  - 1) ? -1 : 
                             (i + 1 == numMSBver - 1) ? si.verSignHypMatch : 
                                     /*otherwise*/      (verOffsetPrediction>>1) & 1;
        const int prevBin = (i == numMSBver - 1) ? si.verSignHypMatch : (verOffsetPrediction & 1);
        const int imvShift = Mv::getImvPrecShift(imv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                               , si.isFracBvEnabled
#endif
        );
        const int iCtxIdx = DeriveCtx::CtxSmBvdBin(prev2Bin, prevBin, false, i + imvShift);

        verOffsetPrediction <<= 1;

        int bin = (int)m_BinDecoder.decodeBin(Ctx::MvsdIdxBVDMSB(iCtxIdx));

        bin = (0 == bin) ? 1 : 0;

        verOffsetPrediction |= bin;
      }


      int verPrefix = si.verPrefixGroupStartValue;


      verSuffix = xReadBvdContextSuffix(verPrefix, verParam - numMSBver );
      verAbs = verSuffix + 1;
    }

    rMvd = Mv(horAbs, verAbs);

    CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal BVD value");
  }
  else
  {
    unsigned horSuffix = 0;
    unsigned verSuffix = 0;
    if (horAbs != 0)
    {
      int horGolombMin = si.horPrefixGroupStartValue;

      horSuffix = xReadBvdContextSuffix(horGolombMin, horParam );
      horAbs = horSuffix + 1;
    }

    if (verAbs != 0)
    {
      int verGolombMin = si.verPrefixGroupStartValue;

      verSuffix = xReadBvdContextSuffix(verGolombMin, verParam );
      verAbs = verSuffix + 1;
    }

    rMvd = Mv(horAbs, verAbs);
  }
}
#endif

#if JVET_AA0070_RRIBC
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACReader::bvdCoding(Mv &rMvd, MvdSuffixInfo &si, const bool useBvdPred, const bool useBvpCluster,
                            int bvOneZeroComp, int bvZeroCompDir, const int &rribcFlipType)
#else
void CABACReader::bvdCoding(Mv& rMvd, MvdSuffixInfo& si, const bool useBvdPred, const int& rribcFlipType)
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACReader::bvdCoding(Mv &rMvd, const bool useBvpCluster, int bvOneZeroComp, int bvZeroCompDir,
                            const int &rribcFlipType)
#else
void CABACReader::bvdCoding(Mv& rMvd, const int& rribcFlipType)
#endif
#endif
{
  int horAbs = 0, verAbs = 0;

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  if (useBvpCluster)
  {
    if (bvOneZeroComp)
    {
      if (bvZeroCompDir == 1)
      {
        horAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
      }

      if (bvZeroCompDir == 2)
      {
        verAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
      }
    }
    else
    {
      horAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
      verAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
    }
  }
  else
  {
    if (rribcFlipType != 2)
    {
      horAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
    }
    if (rribcFlipType != 1)
    {
      verAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
    }
  }
#else
  if (rribcFlipType != 2)
  {
    horAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  }
  if (rribcFlipType != 1)
  {
    verAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
  }
#endif
#else
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACReader::bvdCoding(Mv &rMvd, MvdSuffixInfo &si, const bool useBvdPred, const bool useBvpCluster,
                            int bvOneZeroComp, int bvZeroCompDir)
#else
void CABACReader::bvdCoding(Mv & rMvd, MvdSuffixInfo & si, const bool useBvdPred)
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACReader::bvdCoding(Mv &rMvd, const bool useBvpCluster, int bvOneZeroComp, int bvZeroCompDir)
#else
void CABACReader::bvdCoding(Mv & rMvd)
#endif
#endif
{
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  if (useBvdPred)
  {
    int horParam = (0 != horAbs) ?
      xReadBvdContextPrefix(NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER) : -1;
    int verParam = (0 != verAbs) ?
      xReadBvdContextPrefix(NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER) : -1;
    si.horPrefix = horParam;
    si.verPrefix = verParam;
  }
  else
  {
#endif
    if (horAbs)
    {
      horAbs += xReadBvdContext(NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      if (useBvpCluster)
      {
        if (!bvOneZeroComp)
        {
          if (m_BinDecoder.decodeBinEP())
          {
            horAbs = -horAbs;
          }
        }
      }
      else 
      {
        if (m_BinDecoder.decodeBinEP())
        {
          horAbs = -horAbs;
        }
      }
#else
      if (m_BinDecoder.decodeBinEP())
      {
        horAbs = -horAbs;
      }
#endif
    }
    if (verAbs)
    {
      verAbs += xReadBvdContext(NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      if (useBvpCluster)
      {
        if (!bvOneZeroComp)
        {
          if (m_BinDecoder.decodeBinEP())
          {
            verAbs = -verAbs;
          }
        }
      }
      else
      {
        if (m_BinDecoder.decodeBinEP())
        {
          verAbs = -verAbs;
        }
      }
#else
      if (m_BinDecoder.decodeBinEP())
      {
        verAbs = -verAbs;
      }
#endif
    }
#if JVET_AC0104_IBC_BVD_PREDICTION
  }
#endif
  rMvd = Mv(horAbs, verAbs);

  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal BVD value");

}
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
void CABACReader::mvsdIdxFunc(PredictionUnit &pu, RefPicList eRefList)
{
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  if (!pu.isMvdPredApplicable())
  {
    return;
  }
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu) && !pu.isBvdPredApplicable())
  {
    return;
  }
#endif
  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }
  if (pu.cu->smvdMode && eRefList == REF_PIC_LIST_1)
  {
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER && !JVET_AD0140_MVD_PREDICTION
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 && (pu.mvd[0].getHor() || pu.mvd[0].getVer()))
    {
      if (pu.mvd[eRefList].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvd[eRefList].setHor(-pu.mvd[eRefList].getHor());
        }
      }
      if (pu.mvd[eRefList].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvd[eRefList].setVer(-pu.mvd[eRefList].getVer());
        }
      }
      return;
    }
  }
#endif
  
  Mv trMv = pu.mvd[eRefList].getAbsMv();
  trMv.changeTransPrecAmvr2Internal(pu.cu->imv);
  int Thres = THRES_TRANS;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVS, pu.lumaSize());
  
  int mvsdIdx = 0;
  int shift = 0;
#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    pu.bvdSuffixInfo.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
    pu.bvdSuffixInfo.initSuffixesAndSigns(pu.mvd[REF_PIC_LIST_0], pu.cu->imv);

    const int horPrefix = pu.bvdSuffixInfo.horPrefix;
    const int verPrefix = pu.bvdSuffixInfo.verPrefix;

    trMv = pu.mvd[eRefList].getAbsMv();
    trMv.changeTransPrecAmvr2Internal(pu.cu->imv);

    pu.bvdSuffixInfo.horSignHypMatch = -1;
    pu.bvdSuffixInfo.verSignHypMatch = -1;

    if (horPrefix < 0 && verPrefix < 0)
    {
      return;
    }

    bool setHorSignToNegative = false;
    bool setVerSignToNegative = false;
    Mv trMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
                 verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));
    trMv.changeTransPrecAmvr2Internal(pu.cu->imv);

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
    if (pu.isBvpClusterApplicable())
    {
      if (0 != pu.cu->rribcFlipType)
      {
        pu.mvsdIdx[eRefList] = mvsdIdx;
        bvdCodingRemainder(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.cu->imv);
        return;
      }
    }
#endif
    if (pu.mvd[eRefList].getHor())
    {
      if (pu.bvdSuffixInfo.horEncodeSignInEP)
      {
        setHorSignToNegative = m_BinDecoder.decodeBinEP();
      }
      else
      {
        uint8_t ctxId = (trMv.getHor() <= Thres) ? 0 : 1;
#if JVET_AD0140_MVD_PREDICTION
        uint8_t bin   = m_BinDecoder.decodeBin(Ctx::MvsdIdxIBC(ctxId));
#else
        uint8_t bin   = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));

#endif
        pu.bvdSuffixInfo.horSignHypMatch = 0==bin;

        mvsdIdx += (bin << shift);
        shift++;
      }
    } // if (pu.mvd[eRefList].getHor())
    if (pu.mvd[eRefList].getVer())
    {
      if (pu.bvdSuffixInfo.verEncodeSignInEP)
      {
        setVerSignToNegative = m_BinDecoder.decodeBinEP();
      }
      else
      {
        uint8_t ctxId = (trMv.getVer() <= Thres) ? 0 : 1;
#if JVET_AD0140_MVD_PREDICTION
        uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdxIBC(ctxId));
#else
        uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
#endif
        pu.bvdSuffixInfo.verSignHypMatch = 0 == bin;

        mvsdIdx += (bin << shift);
        shift++;
      }
    } // if (pu.mvd[eRefList].getVer())

    pu.mvsdIdx[eRefList] = mvsdIdx;
    bvdCodingRemainder(pu.mvd[REF_PIC_LIST_0], pu.bvdSuffixInfo, pu.cu->imv);

    if (setHorSignToNegative)
    {
      pu.mvd[REF_PIC_LIST_0].setHor(-pu.mvd[REF_PIC_LIST_0].getHor());
    }
    if (setVerSignToNegative)
    {
      pu.mvd[REF_PIC_LIST_0].setVer(-pu.mvd[REF_PIC_LIST_0].getVer());
    }
    return;
  }
#endif

#if JVET_AD0140_MVD_PREDICTION
  auto& si = pu.mvdSuffixInfo.mvBins[eRefList][0];
  bool setHorSignToNegative = false;
  bool setVerSignToNegative = false;

  si.horSignHypMatch = -1;
  si.verSignHypMatch = -1;

  const int horPrefix = si.horPrefix;
  const int verPrefix = si.verPrefix;

  trMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
            verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));
  trMv.changeTransPrecAmvr2Internal(pu.cu->imv);

  if (pu.mvd[eRefList].getHor())
  {
    if( si.horEncodeSignInEP )
    {
      setHorSignToNegative = m_BinDecoder.decodeBinEP();
    }
    else
    {
      uint8_t ctxId = ( trMv.getHor() <= Thres ) ? 0 : 1;
      uint8_t bin = m_BinDecoder.decodeBin( Ctx::MvsdIdx( ctxId ) );
      si.horSignHypMatch = 0 == bin;
      DTRACE( g_trace_ctx, D_SYNTAX, "mvsd hor: bin=%d, ctx=%d \n", bin, ctxId );
      mvsdIdx += ( bin << shift );
      shift++;
    }
  }
  if (pu.mvd[eRefList].getVer())
  {
    if( si.verEncodeSignInEP )
    {
      setVerSignToNegative = m_BinDecoder.decodeBinEP();
    }
    else
    {
      uint8_t ctxId = ( trMv.getVer() <= Thres ) ? 0 : 1;
      uint8_t bin = m_BinDecoder.decodeBin( Ctx::MvsdIdx( ctxId ) );
      si.verSignHypMatch = 0 == bin;
      DTRACE( g_trace_ctx, D_SYNTAX, "mvsd ver: bin=%d, ctx=%d \n", bin, ctxId );
      mvsdIdx += ( bin << shift );
      shift++;
    }
  }

  pu.mvsdIdx[eRefList] = mvsdIdx;

  if (setHorSignToNegative)
  {
    pu.mvd[eRefList].setHor(-pu.mvd[eRefList].getHor());
  }
  if (setVerSignToNegative)
  {
    pu.mvd[eRefList].setVer(-pu.mvd[eRefList].getVer());
  }

  if (eRefList == REF_PIC_LIST_1 || pu.interDir == 1
      || (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_0 && pu.interDir == 3)
      || pu.cu->smvdMode
     )
  {
    bool readL0Suffixes = pu.interDir != 2;
    bool readL1Suffixes = pu.interDir != 1 && !(pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3);

    if (pu.cu->smvdMode) 
    {
      CHECK(pu.interDir != 3, "SMVD mode should be B-prediction");
      readL0Suffixes &= (eRefList == 0);
      readL1Suffixes = false;
    }

    for (int i = REF_PIC_LIST_0; i < NUM_REF_PIC_LIST_01; ++i)
    {
      if ((i == 0 && !readL0Suffixes) || (i == 1 && !readL1Suffixes) || (!pu.mvd[i].getHor() && !pu.mvd[i].getVer()))
      {
          continue;
      }

      auto& si = pu.mvdSuffixInfo.mvBins[i][0];
      const int horPrefix = si.horPrefix;
      const int verPrefix = si.verPrefix;

      if (horPrefix >= 0 || verPrefix >= 0)
      {
        setHorSignToNegative = pu.mvd[i].getHor() < 0;
        setVerSignToNegative = pu.mvd[i].getVer() < 0;

        mvdCodingRemainder(pu.mvd[i], si, pu.cu->imv );

        if (setHorSignToNegative)
        {
            pu.mvd[i].setHor(-pu.mvd[i].getHor());
        }
        if (setVerSignToNegative)
        {
            pu.mvd[i].setVer(-pu.mvd[i].getVer());
        }
      }
    }
  }
#elif JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.mvd[eRefList].getHor())
  {
    uint8_t ctxId = (trMv.getHor() <= Thres) ? 0 : 1;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvd[eRefList].getVer())
  {
    uint8_t ctxId = (trMv.getVer() <= Thres) ? 0 : 1;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  pu.mvsdIdx[eRefList] = mvsdIdx;
#endif
}
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void CABACReader::mvsdAffineIdxFunc(PredictionUnit &pu, RefPicList eRefList)
{
  if (!pu.cu->affine)
  {
    return;
  }
  if (!pu.isMvdPredApplicable())
  {
    return;
  }
  
  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }

#if JVET_Z0054_BLK_REF_PIC_REORDER && !JVET_AD0140_MVD_PREDICTION
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 &&
      (pu.mvdAffi[0][0].getHor() || pu.mvdAffi[0][0].getVer() ||
        pu.mvdAffi[0][1].getHor() || pu.mvdAffi[0][1].getVer() ||
        (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvdAffi[0][2].getHor() || pu.mvdAffi[0][2].getVer()))
        )
      )
    {
      if (pu.mvdAffi[eRefList][0].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][0].setHor(-pu.mvdAffi[eRefList][0].getHor());
        }
      }
      if (pu.mvdAffi[eRefList][0].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][0].setVer(-pu.mvdAffi[eRefList][0].getVer());
        }
      }
      if (pu.mvdAffi[eRefList][1].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][1].setHor(-pu.mvdAffi[eRefList][1].getHor());
        }
      }
      if (pu.mvdAffi[eRefList][1].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][1].setVer(-pu.mvdAffi[eRefList][1].getVer());
        }
      }

      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        if (pu.mvdAffi[eRefList][2].getHor())
        {
          if (m_BinDecoder.decodeBinEP())
          {
            pu.mvdAffi[eRefList][2].setHor(-pu.mvdAffi[eRefList][2].getHor());
          }
        }
        if (pu.mvdAffi[eRefList][2].getVer())
        {
          if (m_BinDecoder.decodeBinEP())
          {
            pu.mvdAffi[eRefList][2].setVer(-pu.mvdAffi[eRefList][2].getVer());
          }
        }
      }
      return;
    }
  }
#endif
  
  Mv AffMv[3];
  AffMv[0] = pu.mvdAffi[eRefList][0].getAbsMv();
  AffMv[1] = pu.mvdAffi[eRefList][1].getAbsMv();
  AffMv[2] = pu.mvdAffi[eRefList][2].getAbsMv();
  AffMv[0].changeAffinePrecAmvr2Internal(pu.cu->imv);
  AffMv[1].changeAffinePrecAmvr2Internal(pu.cu->imv);
  AffMv[2].changeAffinePrecAmvr2Internal(pu.cu->imv);
  int Thres = THRES_AFFINE;
  
  int mvsdIdx = 0;
  int shift = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVS, pu.lumaSize());
  
#if JVET_AD0140_MVD_PREDICTION
  const int numAffinesInRpl = 2 + (pu.cu->affineType == AFFINEMODEL_6PARAM);

  for (int i = 0; i < numAffinesInRpl; ++i)
  {
    auto& si = pu.mvdSuffixInfo.mvBins[eRefList][i];
    bool setHorSignToNegative = false;
    bool setVerSignToNegative = false;

    si.horSignHypMatch = -1;
    si.verSignHypMatch = -1;

    const int horPrefix = si.horPrefix;
    const int verPrefix = si.verPrefix;

    Mv TrMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
                  verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));
    TrMv.changeAffinePrecAmvr2Internal(pu.cu->imv);

    if (pu.mvdAffi[eRefList][i].getHor())
    {
      if (si.horEncodeSignInEP)
      {
        setHorSignToNegative = m_BinDecoder.decodeBinEP();
      }
      else
      {
        uint8_t ctxId = (TrMv.getHor() <= Thres) ? 2 : 3;
        uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
        si.horSignHypMatch = 0 == bin;
        mvsdIdx += (bin << shift);
        shift++;
      }
    }
    if (pu.mvdAffi[eRefList][i].getVer())
    {
      if (si.verEncodeSignInEP)
      {
        setVerSignToNegative = m_BinDecoder.decodeBinEP();
      }
      else
      {
        uint8_t ctxId = (TrMv.getVer() <= Thres) ? 2 : 3;
        uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
        si.verSignHypMatch = 0 == bin;
        mvsdIdx += (bin << shift);
        shift++;
      }
    }

    if (setHorSignToNegative)
    {
      pu.mvdAffi[eRefList][i].setHor(-pu.mvdAffi[eRefList][i].getHor());
    }
    if (setVerSignToNegative)
    {
      pu.mvdAffi[eRefList][i].setVer(-pu.mvdAffi[eRefList][i].getVer());
    }
  }

  if (eRefList == REF_PIC_LIST_1 || pu.interDir == 1
    || (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_0 && pu.interDir == 3)) // current RPL = L1 or there is only L0: now we can predict suffix bins
  {
    bool readL0Suffixes = pu.interDir != 2;
    bool readL1Suffixes = pu.interDir != 1 && !(pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3);

    // read suffixes
    for (int i = REF_PIC_LIST_0; i < NUM_REF_PIC_LIST_01; ++i)   // read MVD suffixes
    {
      const RefPicList& eRefList = static_cast<RefPicList>(i);

      for (int j = 0; j < numAffinesInRpl; ++j)
      {
        auto& si = pu.mvdSuffixInfo.mvBins[eRefList][j];

        if ((i == 0 && !readL0Suffixes) || (i == 1 && !readL1Suffixes) || (!pu.mvdAffi[i][j].getHor() && !pu.mvdAffi[i][j].getVer()))
        {
          continue;
        }

        const int horPrefix = si.horPrefix;
        const int verPrefix = si.verPrefix;
        if (horPrefix >= 0 || verPrefix >= 0)
        {
          const bool horSignIsNegative = pu.mvdAffi[i][j].getHor() < 0;
          const bool verSignIsNegative = pu.mvdAffi[i][j].getVer() < 0;
          mvdCodingRemainder( pu.mvdAffi[i][j], si, pu.cu->imv );

          if (horSignIsNegative)
          {
            pu.mvdAffi[i][j].setHor(-pu.mvdAffi[i][j].getHor());
          }
          if (verSignIsNegative)
          {
            pu.mvdAffi[i][j].setVer(-pu.mvdAffi[i][j].getVer());
          }
        }
      }
    }
  }

#else
  if (pu.mvdAffi[eRefList][0].getHor())
  {
    uint8_t ctxId = (AffMv[0].getHor() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][0].getVer())
  {
    uint8_t ctxId = (AffMv[0].getVer() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][1].getHor())
  {
    uint8_t ctxId = (AffMv[1].getHor() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][1].getVer())
  {
    uint8_t ctxId = (AffMv[1].getVer() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    if (pu.mvdAffi[eRefList][2].getHor())
    {
      uint8_t ctxId = (AffMv[2].getHor() <= Thres) ? 2 : 3;
      uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
      mvsdIdx += (bin << shift);
      shift++;
    }
    if (pu.mvdAffi[eRefList][2].getVer())
    {
      uint8_t ctxId = (AffMv[2].getVer() <= Thres) ? 2 : 3;
      uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
      mvsdIdx += (bin << shift);
      shift++;
    }
  }
#endif
  pu.mvsdIdx[eRefList] = mvsdIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mvsd_affine_idx_func() mvsdIdx=%d\n", pu.mvsdIdx[eRefList]); // eRefList can have enc-dec mismatch
}
#endif
//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACReader::transform_unit(TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter
#if JVET_AE0102_LFNST_CTX  
  , const bool codeTuCoeff
#endif    
)
{
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;

  CodingStructure&  cs = *tu.cs;
  CodingUnit&       cu = *tu.cu;
  ChromaCbfs        chromaCbfs;
  chromaCbfs.Cb = chromaCbfs.Cr = false;

  const bool chromaCbfISP = area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && cu.ispMode;
#if JVET_AE0102_LFNST_CTX
  if ( codeTuCoeff == false)
  {
#endif
  // cbf_cb & cbf_cr
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!CS::isDualITree(cs) || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#else
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#endif
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cb = cbf_comp(cs, area.blocks[COMPONENT_Cb], cbfDepth);
    }

    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cr = cbf_comp(cs, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
    }
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  else if (CS::isDualITree(cs))
#else
  else if (cu.isSepTree())
#endif
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 0);
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      CHECK(tu.noResidual, "noResidual should be 0");
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else
    {
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      bool lastCbfIsInferred = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < nTus - 1; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, trDepth);
        }
      }
      bool cbfY = lastCbfIsInferred ? true : cbf_comp(cs, tu.Y(), trDepth, previousCbf, cu.ispMode);
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, (cbfY ? 1 : 0));
    }
  }
  if (area.chromaFormat != CHROMA_400 && (!cu.ispMode || chromaCbfISP))
  {
    TU::setCbfAtDepth(tu, COMPONENT_Cb, trDepth, (chromaCbfs.Cb ? 1 : 0));
    TU::setCbfAtDepth(tu, COMPONENT_Cr, trDepth, (chromaCbfs.Cr ? 1 : 0));
  }

  bool lumaOnly  = (cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid());
  bool cbfLuma   = (tu.cbf[COMPONENT_Y] != 0);
  bool cbfChroma = (lumaOnly ? false : (chromaCbfs.Cb || chromaCbfs.Cr));

#if TU_256
  if ((cu.lwidth() > MAX_TB_SIZEY || cu.lheight() > MAX_TB_SIZEY || cbfLuma || cbfChroma) &&
#else
  if ((cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma) &&
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  (!CS::isDualITree(*tu.cs) || isLuma(tu.chType)))
#else
    (!tu.cu->isSepTree() || isLuma(tu.chType)))
#endif
  {
    if (cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#else
  if (!cu.isSepTree() || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#endif
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    SizeType channelWidth = !CS::isDualITree(*tu.cs) ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !CS::isDualITree(*tu.cs) ? cu.lheight() : cu.chromaSize().height;
#else
    SizeType channelWidth = !cu.isSepTree() ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !cu.isSepTree() ? cu.lheight() : cu.chromaSize().height;
#endif
#if TU_256
    if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > MAX_TB_SIZEY || channelHeight > MAX_TB_SIZEY || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
#else
    if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > 64 || channelHeight > 64 || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
#endif
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }
#if JVET_AL0181_ASBT
  tu.blocksResidual[COMPONENT_Y].size() = tu.blocks[COMPONENT_Y].size();
  tu.blocksResidual[COMPONENT_Cr].size() = tu.blocks[COMPONENT_Cr].size();
  tu.blocksResidual[COMPONENT_Cb].size() = tu.blocks[COMPONENT_Cb].size();
  tu.blocksResForDbf[COMPONENT_Y].size() = tu.blocks[COMPONENT_Y].size();
  tu.blocksResForDbf[COMPONENT_Cr].size() = tu.blocks[COMPONENT_Cr].size();
  tu.blocksResForDbf[COMPONENT_Cb].size() = tu.blocks[COMPONENT_Cb].size();

  for (unsigned i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    tu.asbtDecimW[i] = 0;
    tu.asbtDecimH[i] = 0;
  }

  if (cbfLuma != 0 && tu.cu->rootCbf)
  {
    interAsbt(tu);
    for (unsigned i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      if (tu.asbtDecimW[i] > 0)
      {
        tu.blocksResidual[i].width = tu.blocks[i].width >> tu.asbtDecimW[i];
      }

      if (tu.asbtDecimH[i] > 0)
      {
        tu.blocksResidual[i].height = tu.blocks[i].height >> tu.asbtDecimH[i];
      }
    }
    if (chromaCbfs.Cb == 0)
    {
      tu.asbtDecimW[COMPONENT_Cb] = 0;
      tu.asbtDecimH[COMPONENT_Cb] = 0;
    }
    if (chromaCbfs.Cr == 0)
    {
      tu.asbtDecimW[COMPONENT_Cr] = 0;
      tu.asbtDecimH[COMPONENT_Cr] = 0;
    }
  }
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  if ( !lumaOnly )
  {
    interCcpMerge( tu );
  }
#endif
#if JVET_AE0059_INTER_CCCM
  if ( !lumaOnly )
  {
    interCccm( tu );
  }
#endif
  if( !lumaOnly )
  {
    joint_cb_cr(tu, (tu.cbf[COMPONENT_Cb] ? 2 : 0) + (tu.cbf[COMPONENT_Cr] ? 1 : 0));
  }
#if JVET_AE0102_LFNST_CTX
}
#endif

#if JVET_AE0102_LFNST_CTX
  if (tu.cbf[COMPONENT_Y] != 0)
#else
  if (cbfLuma)
#endif
  {
    residual_coding(tu, COMPONENT_Y, cuCtx
#if JVET_AE0102_LFNST_CTX
      , codeTuCoeff
#endif    
    );
  }
#if JVET_AE0102_LFNST_CTX
  if (!(cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid()))
#else
  if (!lumaOnly)
#endif
  {
    for (ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID(compID + 1))
    {
      if (tu.cbf[compID])
      {
        residual_coding(tu, compID, cuCtx
#if JVET_AE0102_LFNST_CTX
                        , codeTuCoeff
#endif
        );
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__DELTA_QP_EP, cu.lumaSize());

  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, cu.blocks[cu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_chroma_qp_offset() chroma_qp_adj=%d\n", cu.chromaQpAdj);
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::joint_cb_cr( TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }

  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMPONENT_Cr].lumaSize(), CHANNEL_TYPE_CHROMA );
    tu.jointCbCr = ( m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( cbfMask-1 ) ) ? cbfMask : 0 );
  }
}

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID, CUCtx& cuCtx
#if JVET_AE0102_LFNST_CTX  
  , const bool codeTuCoeff
#endif    
)
{
  const CodingUnit& cu = *tu.cu;
#if JVET_AE0102_LFNST_CTX
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() codeTuCoeff=%d etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", codeTuCoeff, tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );
#else
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );
#endif

  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
  {
    return;
  }
#if JVET_AE0102_LFNST_CTX
  if (codeTuCoeff == false)
  {
#endif
    ts_flag(tu, compID);
#if JVET_AE0102_LFNST_CTX
  }
#endif
    if (tu.mtsIdx[compID] == MTS_SKIP && !tu.cs->slice->getTSResidualCodingDisabledFlag())
    {
#if JVET_AE0102_LFNST_CTX
      if ( codeTuCoeff )
      {
#endif
        residual_codingTS(tu, compID);
#if JVET_AE0102_LFNST_CTX
      }
#endif
      return;
    }

  // determine sign hiding
  bool signHiding = cu.cs->slice->getSignDataHidingEnabledFlag();

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;
#if JVET_AE0102_LFNST_CTX
  if (codeTuCoeff == false)
  {
#endif
  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );
#if JVET_AE0102_LFNST_CTX
  tu.lastPosition[compID] = cctx.scanPosLast();
#endif
#if !EXTENDED_LFNST
#if JVET_AL0181_ASBT
  if (tu.mtsIdx[compID] != MTS_SKIP && tu.blocksResidual[compID].height >= 4 && tu.blocksResidual[compID].width >= 4)
#else
  if (tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
#endif
  {
#if JVET_AC0130_NSPT
#if JVET_AL0181_ASBT
    uint32_t  width = tu.blocksResidual[ compID ].width;
    uint32_t height = tu.blocksResidual[ compID ].height;
#else
    uint32_t  width = tu.blocks[ compID ].width;
    uint32_t height = tu.blocks[ compID ].height;
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
    bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                  ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
    bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
    bool  allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, CU::isIntra( *( tu.cu ) ) );
#endif

#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    const int maxLfnstPos = ( allowNSPT ? PU::getNSPTMatrixDim( width, height ) : PU::getLFNSTMatrixDim( width, height, tu.cu->cs->sps->getUseLFNSTExt() ) ) - 1;
#else
    const int maxLfnstPos = ( allowNSPT ? PU::getNSPTMatrixDim( width, height ) : PU::getLFNSTMatrixDim( width, height ) ) - 1;
#endif
#else
    const int maxLfnstPos = allowNSPT ? PU::getNSPTMatrixDim( width, height ) - 1: ( ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15 );
#endif
#else
#if JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    const int maxLfnstPos = PU::getLFNSTMatrixDim( tu.blocks[ compID ].width, tu.blocks[ compID ].height, tu.cu->cs->sps->getUseLFNSTExt() ) - 1;
#else
    const int maxLfnstPos = PU::getLFNSTMatrixDim( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) - 1;
#endif
#else
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
#endif
#endif
    cuCtx.violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
#endif

#if JVET_AL0181_ASBT
  if( tu.mtsIdx[compID] != MTS_SKIP && tu.blocksResidual[compID].height >= 4 && tu.blocksResidual[compID].width >= 4 )
#else
  if( tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
#endif
  {
#if JVET_AG0061_INTER_LFNST_NSPT
    const int lfnstLastScanPosTh = isLuma( compID ) ? ( CU::isIntra( *( tu.cu ) ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_LUMA_INTER ) : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= ( CU::isIntra( *( tu.cu ) ) || isLuma( compID ) ) ? ( cctx.scanPosLast() >= lfnstLastScanPosTh ) : false;
#else
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
#endif
  }
  if (isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
  {
    cuCtx.mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }

#if EXTENDED_LFNST
    int subSetId = cctx.scanPosLast() >> cctx.log2CGSize();
    cctx.initSubblock(subSetId);
    if (tu.blocks[compID].width >= 4 && tu.blocks[compID].height >= 4)
    {
      const bool whge3 = tu.blocks[compID].width >= 8 && tu.blocks[compID].height >= 8;
      const bool isLfnstViolated = whge3 ? ((cctx.cgPosY() > 1 || cctx.cgPosX() > 1)) : ((cctx.cgPosY() > 0 || cctx.cgPosX() > 0));
      cuCtx.violatesLfnstConstrained[toChannelType(compID)] |= isLfnstViolated;
    }
#endif

#if JVET_AE0102_LFNST_CTX
    return;
  }
  else
  {
    cctx.setScanPosLast(tu.lastPosition[compID]);
  }
#endif

#if SIGN_PREDICTION
  AreaBuf<SIGN_PRED_TYPE> signBuff = tu.getCoeffSigns(compID);
  tu.initSignBuffers( compID );
#endif

  // parse subblocks
#if TCQ_8STATES
  const uint64_t stateTransTab = g_stateTransTab[ tu.cs->slice->getDepQuantEnabledIdc() ];
#else
  const int stateTransTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
#endif
  int       state         = 0;

  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );

#if !TU_256
      if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 && compID == COMPONENT_Y )
      {
        if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( tu.blocks[ compID ].width == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
        {
          continue;
        }
      }
#endif
#if SIGN_PREDICTION
#if JVET_AE0102_LFNST_CTX 
      int lfnstidx =  cu.lfnstIdx;
      residual_coding_subblock(cctx, coeff, signBuff.buf, stateTransTab, state, lfnstidx);
#else
      residual_coding_subblock(cctx, coeff, signBuff.buf, stateTransTab, state);
#endif
#else
      residual_coding_subblock(cctx, coeff, stateTransTab, state);
#endif
#if !JVET_AE0102_LFNST_CTX
#if EXTENDED_LFNST
      if ( tu.blocks[compID].width >= 4 && tu.blocks[compID].height >= 4 )
      {
        const bool whge3 = tu.blocks[compID].width >= 8 && tu.blocks[compID].height >= 8;
        const bool isLfnstViolated = whge3 ? (cctx.isSigGroup() && (cctx.cgPosY() > 1 || cctx.cgPosX() > 1)) : (cctx.isSigGroup() && (cctx.cgPosY() > 0 || cctx.cgPosX() > 0));
        cuCtx.violatesLfnstConstrained[toChannelType(compID)] |= isLfnstViolated;
      }
#endif
#endif
#if !TU_256
      if ( isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
      {
        cuCtx.violatesMtsCoeffConstraint = true;
      }
#endif
    }
#if JVET_Y0142_ADAPT_INTRA_MTS
    if (isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP
#if AHG7_MTS_TOOLOFF_CFG
      && tu.cu->cs->sps->getUseMTSExt()
#endif
      )
    {
      const int  coeffStride = tu.getCoeffs(compID).stride;
      const int  uiWidth = tu.getCoeffs(compID).width;
      const int  uiHeight = tu.getCoeffs(compID).height;
      uint64_t coeffAbsSum = 0;

      for (int y = 0; y < uiHeight; y++)
      {
        for (int x = 0; x < uiWidth; x++)
        {
          coeffAbsSum += abs(coeff[(y * coeffStride) + x]);
        }
      }
      cuCtx.mtsCoeffAbsSum = (int64_t)coeffAbsSum;
    }
#endif
}

void CABACReader::ts_flag( TransformUnit& tu, ComponentID compID )
{
  int tsFlag = ( (tu.cu->bdpcmMode && isLuma(compID)) || (tu.cu->bdpcmModeChroma && isChroma(compID)) ) ? 1 : tu.mtsIdx[compID] == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[compID], compID );
    tsFlag = m_BinDecoder.decodeBin( Ctx::TransformSkipFlag( ctxIdx ) );
  }

  tu.mtsIdx[compID] = tsFlag ? MTS_SKIP : MTS_DCT2_DCT2;

  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.blocks[compID].x, tu.blocks[compID].y, tsFlag);
}

void CABACReader::mts_idx( CodingUnit& cu, CUCtx& cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMPONENT_Y]; // Transform skip flag has already been decoded

  if( CU::isMTSAllowed( cu, COMPONENT_Y ) && !cuCtx.violatesMtsCoeffConstraint &&
      cuCtx.mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[COMPONENT_Y], COMPONENT_Y );
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
    int ctxIdx = 0;
    if (CU::isIntra(cu)
#if AHG7_MTS_TOOLOFF_CFG
       && tu.cu->cs->sps->getUseMTSExt()
#endif
      )
    {
      ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
    }
#else
    int ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
#endif
#else
    int ctxIdx = (cu.mipFlag) ? 3 : 0;
#endif
#else
    int ctxIdx = 0;
#endif
    int symbol = m_BinDecoder.decodeBin( Ctx::MTSIdx(ctxIdx));

    if( symbol )
    {
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
      int nCands = CU::isIntra(cu)
#if AHG7_MTS_TOOLOFF_CFG
        && tu.cu->cs->sps->getUseMTSExt()
#endif
        ? MTS_NCANDS[ctxIdx] : 4;
#else
      int nCands = MTS_NCANDS[ctxIdx];
#endif
      uint32_t val;
      if (nCands > 1)
      {
        xReadTruncBinCode(val, nCands);
      }
      else
      {
        val = 0;
      }
      mtsIdx = MTS_DST7_DST7 + val;
#else
      int bins[2];
      bins[0] = m_BinDecoder.decodeBin(Ctx::MTSIdx(1));
      bins[1] = m_BinDecoder.decodeBin(Ctx::MTSIdx(2));
      mtsIdx = MTS_DST7_DST7 + (bins[0] << 1) + bins[1];
#endif
#else
      ctxIdx = 1;
      mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol  = m_BinDecoder.decodeBin( Ctx::MTSIdx(ctxIdx));
        mtsIdx += symbol;

        if( !symbol )
        {
          break;
        }
      }
#endif
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
  }

#if TU_256
  for( auto &tmpTu : CU::traverseTUs( cu ) )
  {
    tmpTu.mtsIdx[COMPONENT_Y] = mtsIdx;
  }
#else
  tu.mtsIdx[COMPONENT_Y] = mtsIdx;
#endif

}

void CABACReader::isp_mode( CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) || cu.colorTransform 
#if ENABLE_DIMD && JVET_V0087_DIMD_NO_ISP && !JVET_AJ0249_NEURAL_NETWORK_BASED
      || cu.dimd
#endif
#if JVET_AJ0146_TIMDSAD
    || (cu.timd && cu.timdSad)
#endif
#if JVET_AB0155_SGPM
      || cu.sgpm
#endif
    )
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__ISP_MODE_FLAG, cu.lumaSize());

#if JVET_W0123_TIMD_FUSION
  int symbol = m_BinDecoder.decodeBin(cu.timd ? Ctx::ISPMode(2) : Ctx::ISPMode(0));
#else
  int symbol = m_BinDecoder.decodeBin(Ctx::ISPMode(0));
#endif

  if( symbol )
  {
    cu.ispMode = 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subpartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACReader::residual_lfnst_mode( CodingUnit& cu,  CUCtx& cuCtx  )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int chIdx = CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#else
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( cu.slice->getSliceType() == I_SLICE && cu.cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( cu.slice->getSliceType() != I_SLICE && cu.cs->sps->getUseIntraLFNSTPBSlice() ) );
#endif
  if ((cu.ispMode && !CU::canUseLfnstWithISP(cu, cu.chType))
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
#if JVET_V0130_INTRA_TMP
    || ( spsIntraLfnstEnabled && CU::isIntra( cu ) && ( ( cu.mipFlag && !allowLfnstWithMip( cu.firstPU->lumaSize() ) ) || ( cu.tmpFlag && !allowLfnstWithTmp() ) ) )
#else
    || ( spsIntraLfnstEnabled && CU::isIntra( cu ) && cu.mipFlag && !allowLfnstWithMip( cu.firstPU->lumaSize() ) )
#endif
#else
#if JVET_V0130_INTRA_TMP
    || (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && ((cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) || (cu.tmpFlag && !allowLfnstWithTmp())))
#else
    || (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize()))
#endif
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA && std::min(cu.blocks[1].width, cu.blocks[1].height) < 4)
#else
    || (cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min(cu.blocks[1].width, cu.blocks[1].height) < 4)
#endif
#if JVET_AG0061_INTER_LFNST_NSPT && JVET_AI0050_SBT_LFNST
    || ( !cu.cs->sps->getUseSbtLFNST() && cu.sbtInfo && CU::isInter( cu ) )
#elif JVET_AG0061_INTER_LFNST_NSPT
    || ( cu.sbtInfo && CU::isInter( cu ) ) //JVET-AG0208 (EE2-related: On LFNST/NSPT index signalling)
#endif
    || (cu.blocks[chIdx].lumaSize().width > cu.cs->sps->getMaxTbSize()
          || cu.blocks[chIdx].lumaSize().height > cu.cs->sps->getMaxTbSize()))
  {
    return;
  }
#if JVET_W0123_TIMD_FUSION
  if (cu.timd && (cu.ispMode || cu.firstPU->multiRefIdx))
  {
    cu.lfnstIdx = 0;
    return;
  }
#endif

#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
#if JVET_AL0181_ASBT
  bool isAsbt = false;

  for (auto &currTU : CU::traverseTUs(cu))
  {
    isAsbt = isAsbt || ( currTU.asbtDecimW[COMPONENT_Y] ||currTU.asbtDecimH[COMPONENT_Y] );
  }
  if( ( spsIntraLfnstEnabled && CU::isIntra( cu ) ) || ( cu.cs->sps->getUseInterLFNST() && (CU::isInter(cu) && !isAsbt) ) )
#else
  if( ( spsIntraLfnstEnabled && CU::isIntra( cu ) ) || ( cu.cs->sps->getUseInterLFNST() && CU::isInter( cu ) ) )
#endif
#else
  if (cu.cs->sps->getUseLFNST() && (CU::isIntra(cu) || CU::isInter(cu)))
#endif
#else
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  if( spsIntraLfnstEnabled && CU::isIntra( cu ) )
#else
  if (cu.cs->sps->getUseLFNST() && CU::isIntra(cu))
#endif
#endif
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    const bool lumaFlag              = CS::isDualITree(*cu.cs) ? (isLuma(cu.chType) ? true : false) : true;
    const bool chromaFlag            = CS::isDualITree(*cu.cs) ? (isChroma(cu.chType) ? true : false) : true;
#else
    const bool lumaFlag              = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
#endif
    bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
    bool isTrSkip = false;
    for (auto &currTU : CU::traverseTUs(cu))
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMPONENT_Y; compID < numValidComp; compID++)
      {
        if (currTU.blocks[compID].valid() && TU::getCbf(currTU, (ComponentID)compID) && currTU.mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if ((!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip)
    {
      cu.lfnstIdx = 0;
      return;
    }
  }
  else
  {
    cu.lfnstIdx = 0;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__LFNST, cu.blocks[chIdx].lumaSize(), ChannelType(chIdx));
#if JVET_AG0061_INTER_LFNST_NSPT
  if (CU::isInter(cu))
  {
    uint32_t idxLFNST = m_BinDecoder.decodeBin(Ctx::InterLFNSTIdx(0));
    if (idxLFNST)
    {
      idxLFNST += m_BinDecoder.decodeBin(Ctx::InterLFNSTIdx(1));
#if AHG7_LN_TOOLOFF_CFG
      int width = cu.blocks[ chIdx ].width;
      int height = cu.blocks[ chIdx ].height;

#if JVET_AI0050_SBT_LFNST
      if( cu.sbtInfo )
      {
        for( auto &currTU : CU::traverseTUs( cu ) )
        {
          if( !currTU.noResidual )
          {
            width = currTU.lwidth();
            height = currTU.lheight();
            break;
          }
        }
      }
#endif

      if( cu.cs->sps->getUseLFNSTExt() || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( width, height ) ) )
      {
#endif
      if (idxLFNST == 2)
      {
        idxLFNST += m_BinDecoder.decodeBin(Ctx::InterLFNSTIdx(2));
      }
#if AHG7_LN_TOOLOFF_CFG
      }
#endif
    }
    cu.lfnstIdx = idxLFNST;
#if JVET_AI0050_INTER_MTSS
    if (cu.cs->sps->getUseInterMTSS() && cu.lfnstIdx > 0 && cu.geoFlag)
    {
      cu.lfnstIntra = m_BinDecoder.decodeBin(Ctx::InterLFNSTIntraIdx());
    }
#endif
  }
  else
  {
#endif

    unsigned cctx = 0;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs)) cctx++;
#else
    if (cu.isSepTree()) cctx++;
#endif

    
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if AHG7_LN_TOOLOFF_CFG
    uint32_t idxLFNST = 0;

    Size tuSize = ( cu.ispMode == HOR_INTRA_SUBPARTITIONS ) ? Size( cu.blocks[ chIdx ].width, CU::getISPSplitDim( cu.blocks[ chIdx ].width, cu.blocks[ chIdx ].height, TU_1D_HORZ_SPLIT ) ) :
                                                              Size( CU::getISPSplitDim( cu.blocks[ chIdx ].width, cu.blocks[ chIdx ].height, TU_1D_VERT_SPLIT ), cu.blocks[ chIdx ].height );
    int width  = ( cu.ispMode == NOT_INTRA_SUBPARTITIONS ) ? cu.blocks[ chIdx ].width  : tuSize.width;
    int height = ( cu.ispMode == NOT_INTRA_SUBPARTITIONS ) ? cu.blocks[ chIdx ].height : tuSize.height;

    if( cu.cs->sps->getUseLFNSTExt() || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( width, height ) ) )
    {
#endif
    uint32_t firstBit  = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(cctx));
    cctx               = 2 + firstBit;
    uint32_t secondBit = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(cctx));
#if AHG7_LN_TOOLOFF_CFG
    idxLFNST = firstBit + ( secondBit << 1 );
#else
    uint32_t idxLFNST  = firstBit + (secondBit << 1);
#endif
#if AHG7_LN_TOOLOFF_CFG
    }
    else
    {
      idxLFNST = m_BinDecoder.decodeBin( Ctx::VvcLFNSTIdx( cctx ) );
      if( idxLFNST )
      {
        idxLFNST += m_BinDecoder.decodeBin( Ctx::VvcLFNSTIdx( 2 ) );
      }
    }
#endif
#else
    uint32_t idxLFNST = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(cctx));
    if (idxLFNST)
    {
      idxLFNST += m_BinDecoder.decodeBin(Ctx::LFNSTIdx(2));
    }
#endif
    cu.lfnstIdx = idxLFNST;
#if JVET_AJ0249_NEURAL_NETWORK_BASED
    if (isLuma(cu.chType) && cu.lfnstIdx && isAllowedMultiple(cu.lwidth(), cu.lheight()))
    {
      const uint32_t intraMode = PU::getFinalIntraMode(*cu.firstPU, cu.chType);
      if (intraMode == PNN_IDX)
      {
        cu.lfnstSecFlag = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(4));
      }
    }
#endif
#if JVET_AK0217_INTRA_MTSS
    if (CU::isMdirAllowed(cu) && idxLFNST <= (cu.lwidth() * cu.lheight() < 256 ? MTSS_CAND_NUM[0] : MTSS_CAND_NUM[1]))
    {
      int mdirIdx = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(4));
      for (auto& tmpTu : CU::traverseTUs(cu))
      {
        tmpTu.mdirIdx[COMPONENT_Y] = mdirIdx;
      }
    }
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  }
#endif

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__LAST_SIG_X_Y, Size( cctx.width(), cctx.height() ), cctx.compID() );

  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if !TU_256
  if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 && compID == COMPONENT_Y )
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }
#endif

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if (!m_BinDecoder.decodeBin(cctx.lastYCtxId(PosLastY)))
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
#if JVET_AK0097_LAST_POS_SIGNALING
    bool isMaxX=false;
    bool isMaxXisSignaled=false;

    if ( (cctx.width() >= SECONDARY_PREFIX_START_SIZE) && PosLastX == maxLastPosX )
    {
      int ctxId = compID==0 ? g_aucLog2[cctx.width()/SECONDARY_PREFIX_START_SIZE] : (5 + compID - 1);
      isMaxX = (m_BinDecoder.decodeBin(Ctx::lastXSecondaryPrefix(ctxId)) == 1 ? true : false);
      isMaxXisSignaled = true;
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxX=%d\n", isMaxX ? 1 : 0);
    }

    if( isMaxX )
    {
      CHECKD(PosLastX >= (LAST_SIGNIFICANT_GROUPS - 1), "Abnormal situation");
      PosLastX = g_uiMinInGroup[PosLastX + 1] - 1;
    }
    else
    {
#endif
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
#if JVET_AK0097_LAST_POS_SIGNALING
    bool allTo1=true;
    for ( int i = uiCount - 1; i >= 1; i-- )
#else
    for ( int i = uiCount - 1; i >= 0; i-- )
#endif
    {
#if JVET_AK0097_LAST_POS_SIGNALING
      if( i==(uiCount-1) )
      {
        int ctxOf = g_aucLog2[(cctx.width()/ID_SUFFIX_CTX_START_SIZE)];
        CHECKD(ctxOf<0 || ctxOf>g_aucLog2[128/ID_SUFFIX_CTX_START_SIZE],"Abnormal value of ctxOf");
        uiTemp += m_BinDecoder.decodeBin( Ctx::lastXSuffix[compID](ctxOf) ) << i;
      }
      else
      {
        uiTemp += m_BinDecoder.decodeBinEP( ) << i;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastXSuffixBin(%d)=%d\n", i, (uiTemp>>i)&1);
      allTo1 &= ( ( (uiTemp >> i)&1 ) ==1 );
#else
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
#endif
    }
#if JVET_AK0097_LAST_POS_SIGNALING
    if( !isMaxXisSignaled || !allTo1 )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << 0;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() uiTempX=%d\n", uiTemp);
#endif
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
#if JVET_AK0097_LAST_POS_SIGNALING
    }
#endif

  }
  if( PosLastY > 3 )
  {
#if JVET_AK0097_LAST_POS_SIGNALING
    bool isMaxY=false;
    bool isMaxYisSignaled=false;

    if (cctx.height() >= SECONDARY_PREFIX_START_SIZE && PosLastY == maxLastPosY)
    {
      int ctxId = compID==0 ? g_aucLog2[cctx.height()/SECONDARY_PREFIX_START_SIZE] : (5 + compID - 1);
      isMaxY = (m_BinDecoder.decodeBin(Ctx::lastYSecondaryPrefix(ctxId)) == 1 ? true : false);
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxY=%d\n", isMaxY ? 1 : 0);
      isMaxYisSignaled=true;
    }

    if( isMaxY )
    {
      CHECK(PosLastY >= (LAST_SIGNIFICANT_GROUPS - 1), "Abnormal situation");
      CHECKD(PosLastY >= (LAST_SIGNIFICANT_GROUPS - 1), "Abnormal situation");
      PosLastY = g_uiMinInGroup[PosLastY + 1] - 1;
    }
    else
    {
#endif
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
#if JVET_AK0097_LAST_POS_SIGNALING
    bool allTo1=true;
    for ( int i = uiCount - 1; i >= 1; i-- )
#else
    for ( int i = uiCount - 1; i >= 0; i-- )
#endif
    {
#if JVET_AK0097_LAST_POS_SIGNALING
      if( i==(uiCount-1) )
      {
        int ctxOf = g_aucLog2[(cctx.height()/ID_SUFFIX_CTX_START_SIZE)];
        CHECK(ctxOf<0 || ctxOf>g_aucLog2[128/ID_SUFFIX_CTX_START_SIZE],"Abnormal value of ctxOf");
        uiTemp += m_BinDecoder.decodeBin( Ctx::lastYSuffix[compID](ctxOf) ) << i;
      }
      else
      {
        uiTemp += m_BinDecoder.decodeBinEP( ) << i;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastYSuffixBin(%d)=%d\n", i, (uiTemp>>i)&1);
      allTo1 &= ( ( (uiTemp >> i)&1 ) ==1 );
#else
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
#endif
    }
#if JVET_AK0097_LAST_POS_SIGNALING
    if ( !isMaxYisSignaled || !allTo1 )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << 0;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() uiTempY=%d\n", uiTemp);
#endif
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
#if JVET_AK0097_LAST_POS_SIGNALING
    }
#endif
  }

  int blkPos;
  blkPos = PosLastX + (PosLastY * cctx.width());

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX_RESI, "last_sig_coeff() scan_pos_last=%d\n", scanPos);
  return scanPos;
}

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
static void check_coeff_conformance(const CoeffCodingContext& cctx, const TCoeff coeff)
#else
static void check_coeff_conformance(TCoeff coeff)
#endif
{
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  CHECK( coeff < cctx.minCoeff() || coeff > cctx.maxCoeff(),
         "TransCoeffLevel outside allowable range" );
#else
  CHECK( coeff < COEFF_MIN || coeff > COEFF_MAX,
         "TransCoeffLevel should be in the range [-32768, 32767]" );
#endif
}
#if TCQ_8STATES
#if SIGN_PREDICTION
#if JVET_AE0102_LFNST_CTX 
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, SIGN_PRED_TYPE* sign, const uint64_t stateTransTable, int& state, int lfnstIdx )
#else
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, SIGN_PRED_TYPE* sign, const uint64_t stateTransTable, int& state )
#endif
#else
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const uint64_t stateTransTable, int& state )
#endif
#else
#if SIGN_PREDICTION
void CABACReader::residual_coding_subblock(CoeffCodingContext &cctx, TCoeff *coeff, SIGN_PRED_TYPE *sign,
  const int stateTransTable, int &state)
#else
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const int stateTransTable, int& state )
#endif
#endif
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  const int   cgStartPosX = cctx.cgPosX() << cctx.log2CGWidth();
  const int   cgStartPosY = cctx.cgPosY() << cctx.log2CGHeight();
  const bool  isSPArea = (cgStartPosX < SIGN_PRED_FREQ_RANGE) && (cgStartPosY < SIGN_PRED_FREQ_RANGE);
  const bool  signPredQualified = cctx.getPredSignsQualified() > 0 && isSPArea && cctx.width() >= 4 && cctx.height() >= 4;
#else
  const bool  isFirst      = !cctx.isNotFirst();
  const bool  signPredQualified = cctx.getPredSignsQualified() && isFirst && cctx.width() >= 4 && cctx.height() >= 4 ;
#endif
#endif
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "sig_group() bin=%d\n", sigGroup);
  }
  if (sigGroup)
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }
  uint8_t   ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
#if JVET_AM0056_PRED_TRANSFORM_COEFF_CODING
    unsigned gt1Flag    = 0;
    bool     isPTCC     = false;
#endif
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
#if JVET_AE0102_LFNST_CTX 
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state, lfnstIdx);
#else
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state);
#endif
      
#if JVET_AM0056_PRED_TRANSFORM_COEFF_CODING
      bool needDecSign   = true;
      isPTCC           =  (cctx.compID() == COMPONENT_Y) && m_BinDecoder.getProb(sigCtxId) > PTCC_SIG_THR;
      if (isPTCC)
      {
        uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
        ctxOff           = cctx.ctxOffsetAbs();
        gt1Flag = m_BinDecoder.decodeBin(cctx.greater1CtxIdAbs(ctxOff));
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
        remRegBins--;
        
        needDecSign = !gt1Flag;
      }
    
      if (needDecSign)
      {
#endif
        sigFlag = m_BinDecoder.decodeBin( sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
        remRegBins--;
#if JVET_AM0056_PRED_TRANSFORM_COEFF_CODING
      }
      else
      {
        sigFlag = 1;    // inferred significant flags
      }
#endif
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
#if JVET_AE0102_LFNST_CTX 
      cctx.sigCtxIdAbs(nextSigPos, coeff, state, lfnstIdx); // required for setting variables that are needed for gtx/par context selection
#else
      cctx.sigCtxIdAbs(nextSigPos, coeff, state); // required for setting variables that are needed for gtx/par context selection
#endif
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff           = cctx.ctxOffsetAbs();
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
#if JVET_AM0056_PRED_TRANSFORM_COEFF_CODING
      if (!isPTCC)
      {
        gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
#else
        unsigned gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
#endif
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff) );
        remRegBins--;
#if JVET_AM0056_PRED_TRANSFORM_COEFF_CODING
      }
#endif
      unsigned parFlag = 0;
      unsigned gt2Flag = 0;
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      unsigned gtX = 0;
      unsigned gtN = 0;
#endif
      if( gt1Flag )
      {
#if !JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin(cctx.parityCtxIdAbs(ctxOff));
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs( ctxOff ) );
        remRegBins--;
#endif
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);

        gt2Flag = m_BinDecoder.decodeBin(cctx.greater2CtxIdAbs(ctxOff));
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
        if (gt2Flag)
        {
          gtX = 1;
          for (int i = 1; i < GTN - 1; i++)
          {
            if (gtX)
            {
              unsigned int ctxId = (i == 1) ? cctx.greater3CtxIdAbs(ctxOff) : cctx.greater4CtxIdAbs(ctxOff);
              gtX = m_BinDecoder.decodeBin(ctxId);
              DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt%d_flag() bin=%d ctx=%d\n", i + 2, gtX, ctxId);
              gtN += gtX;
              remRegBins--;
            }
            else
            {
              gtX = 0;
              break;
            }
          }

          if (gtX)
          {
            parFlag = m_BinDecoder.decodeBinEP();
            DTRACE(g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d \n", parFlag);
          }
        }
#endif
      }
#if !JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      coeff[ blkPos ] += 1 + parFlag + gt1Flag + (gt2Flag << 1);
#else
      coeff[blkPos] += 1 + parFlag + gt1Flag + gt2Flag + gtN;
#endif
    }
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((coeff[blkPos]&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((coeff[blkPos]&1)<<1)) ) & 3;
#endif
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
#if JVET_AE0102_LFNST_CTX
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    int       sumAll = cctx.templateAbsSum2(scanPos, coeff, GTN_LEVEL);
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4, lfnstIdx);
#endif
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
#endif
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    ricePar = g_auiGoRiceParsCoeffGTN[sumAll];
#else
    ricePar = g_auiGoRiceParsCoeff[sumAll];
#endif
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    if (tcoeff >= GTN_LEVEL)
#else
    if( tcoeff >= 4 )
#endif
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
      int       rem     = m_BinDecoder.decodeRemAbsEP( ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      tcoeff += (rem<<1);
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
#if JVET_AE0102_LFNST_CTX
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0, lfnstIdx);
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
#endif
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeff    tcoeff  = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((tcoeff&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((tcoeff&1)<<1)) ) & 3;
#endif
    if( tcoeff )
    {
      int        blkPos         = cctx.blockPos( scanPos );
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = scanPos;
      lastNZPos  = std::max<int>( lastNZPos, scanPos );
      coeff[blkPos] = tcoeff;
    }
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
#if SIGN_PREDICTION
  unsigned        signPattern;
  if(!signPredQualified)
  {
    signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );
  }
#else
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );
#endif

  //===== set final coefficents =====
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  TCoeff sumAbs = 0;
#else
  int sumAbs = 0;
#endif

#if SIGN_PREDICTION
  if( !signPredQualified)
  {
#endif
  for( unsigned k = 0; k < numSigns; k++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif
  }
#if SIGN_PREDICTION
  }
#endif
  if( numNonZero > numSigns )
  {
    int k                 = numSigns;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif

#if SIGN_PREDICTION
    sign[sigBlkPos[k]] = SIGN_PRED_HIDDEN;
#endif
  }
}

void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding_ts() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma);
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;
  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS(cctx, coeff);
  }
}
void CABACReader::residual_coding_subblockTS(CoeffCodingContext &cctx, TCoeff *coeff)
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
#if TR_ONLY_COEFF_STATS
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG_TS, cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS_TS,        cctx.width(), cctx.height(), cctx.compID() );
#else
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

#endif

  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin(cctx.sigGroupCtxId(true));
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sig_group() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId());
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS(nextSigPos, coeff);
      sigFlag                 = m_BinDecoder.decodeBin(sigCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== decode sign's =====
#if TR_ONLY_COEFF_STATS
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT_TS, Size(cctx.width(), cctx.height()), cctx.compID());
#else
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
#endif
      int sign;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      sign                     = m_BinDecoder.decodeBin(signCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sign() bin=%d ctx=%d\n", sign, signCtxId);
      cctx.decimateNumCtxBins(1);

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      gt1Flag                 = m_BinDecoder.decodeBin(gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId);
      cctx.decimateNumCtxBins(1);

      unsigned parFlag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin(cctx.parityCtxIdAbsTS());
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS());
        cctx.decimateNumCtxBins(1);
      }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (TCoeff)(1 + parFlag + gt1Flag);
#else
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (1 + parFlag + gt1Flag);
#endif
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  const int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    TCoeff& tcoeff = coeff[cctx.blockPos(scanPos)];
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if( tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
      if (tcoeff >= cutoffVal)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        unsigned gt2Flag;
        gt2Flag = m_BinDecoder.decodeBin(cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        tcoeff += (gt2Flag << 1);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, tcoeff);
        cctx.decimateNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }
  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );

    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    if (tcoeff < 0)
    {
      tcoeff = -tcoeff;
    }
    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += (scanPos <= lastScanPosPass1) ? (rem << 1) : rem;
      if (tcoeff && scanPos > lastScanPosPass1)
      {
        int      blkPos = cctx.blockPos(scanPos);
        int sign = m_BinDecoder.decodeBinEP();
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_rice_sign() bin=%d\n", sign);
        signPattern += (sign << numNonZero);
        sigBlkPos[numNonZero++] = blkPos;
      }
    }
    if (!cctx.bdpcm() && cutoffVal)
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    coeff[ sigBlkPos[k] ] = ( signPattern & 1 ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif
  }
}

//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}

unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}

unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}
#if JVET_V0130_INTRA_TMP
void CABACReader::tmp_flag(CodingUnit& cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_TMP_FLAG, cu.lumaSize(), COMPONENT_Y);

  if (!cu.Y().valid())
  {
    return;
  }

#if JVET_X0124_TMP_SIGNAL && !JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.dimd)
  {
    cu.tmpFlag = false;
    return;
  }
#endif

  if (!cu.cs->sps->getUseIntraTMP())
  {
    cu.tmpFlag = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxTmpFlag(cu);
  cu.tmpFlag     = m_BinDecoder.decodeBin(Ctx::TmpFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "tmp_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpFlag ? 1 : 0);
#if JVET_AD0086_ENHANCED_INTRA_TMP
  if (cu.tmpFlag)
  {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
    cu.tempType = CU::getRefTemplateType(cu);
#endif
    unsigned ctxId_fusion = DeriveCtx::CtxTmpFusionFlag(cu);
    cu.tmpFusionFlag      = m_BinDecoder.decodeBin(Ctx::TmpFusion(ctxId_fusion));
    DTRACE(g_trace_ctx, D_SYNTAX, "tmp_fusion_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
           cu.tmpFusionFlag ? 1 : 0);
    if (cu.tmpFusionFlag)
    {
      cu.tmpIdx = 0;
      int tmpFusionIdx = m_BinDecoder.decodeBin(Ctx::TmpFusion(4))? TMP_GROUP_IDX : 0;

      cu.tmpIdx =  m_BinDecoder.decodeBin(Ctx::TmpFusion(5));
      if(cu.tmpIdx)
      {
        cu.tmpIdx += m_BinDecoder.decodeBinEP();
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        if (cu.tmpIdx == TMP_GROUP_IDX - 2 && tmpFusionIdx == 0)
        {
          cu.tmpIdx += m_BinDecoder.decodeBinEP();
        }
#endif
      }
      cu.tmpIdx += tmpFusionIdx;
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_fusion_idx() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIdx);
#if JVET_AG0136_INTRA_TMP_LIC
      cu.tmpLicFlag = m_BinDecoder.decodeBin(Ctx::TmpLic(0));
      cu.ibcLicFlag = cu.tmpLicFlag;
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      cu.ibcLicIdx = IBC_LIC_IDX;
#else
      cu.ibcLicIdx = 0;
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "tmpLicFlag pos=(%d,%d) value=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpLicFlag);
#endif
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::TmpIdx(0)))
      {
        if (m_BinDecoder.decodeBin(Ctx::TmpIdx(1)))
        {
          cu.tmpIdx = 0;
        }
        else
        {
          if (m_BinDecoder.decodeBin(Ctx::TmpIdx(2)))
          {
            cu.tmpIdx = 1;
          }
          else
          {
            cu.tmpIdx = 2;
          }
        }
      }
      else
      {
        uint32_t tmpIdx = 0;
        xReadTruncBinCode(tmpIdx, MTMP_NUM - 3);
        cu.tmpIdx = tmpIdx + 3;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_idx() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIdx);

      cu.tmpFlmFlag = m_BinDecoder.decodeBin(Ctx::TmpFusion(3));
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_flm_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
             cu.tmpFlmFlag);
#if JVET_AG0136_INTRA_TMP_LIC
      if (!cu.tmpFlmFlag)
      {
        cu.tmpLicFlag = m_BinDecoder.decodeBin(Ctx::TmpLic(0));
        cu.ibcLicFlag = cu.tmpLicFlag;
        DTRACE(g_trace_ctx, D_SYNTAX, "tmpLicFlag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpLicFlag);

        if (cu.slice->getSPS()->getItmpLicExtension() && cu.ibcLicFlag)
        {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          int bin1 = 0;
          if (cu.tempType == L_SHAPE_TEMPLATE)
          {
            bin1 = m_BinDecoder.decodeBin(Ctx::ItmpLicIndex(0));
          }
#else
          const int bin1 = m_BinDecoder.decodeBin(Ctx::ItmpLicIndex(0));
#endif
          const int bin2 = m_BinDecoder.decodeBin(Ctx::ItmpLicIndex(1));
          if (bin1)
          {
            cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_L : IBC_LIC_IDX_T;
          }
          else
          {
            cu.ibcLicIdx = bin2 ? IBC_LIC_IDX_M : IBC_LIC_IDX;
          }
          DTRACE(g_trace_ctx, D_SYNTAX, "tmp_lic_idx=%d\n", cu.ibcLicIdx);
        }
        else
        {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          cu.ibcLicIdx = IBC_LIC_IDX;
#else
          cu.ibcLicIdx = 0;
#endif
        }
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      }

#if !JVET_AH0200_INTRA_TMP_BV_REORDER
      if (!cu.tmpFlmFlag && !cu.tmpLicFlag)
#else
      if (!cu.tmpFlmFlag)
#endif
#endif
#else
      if (!cu.tmpFlmFlag)
#endif
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING || !JVET_AG0136_INTRA_TMP_LIC
      {
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        if(cu.lwidth() * cu.lheight() > TMP_SKIP_REFINE_THRESHOLD)
        {
#endif
          cu.tmpFracIdx = 0;
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        if (m_BinDecoder.decodeBin(Ctx::TmpFlag(4)))
#else
        }
        else if(m_BinDecoder.decodeBin(Ctx::TmpFlag(7)))
#endif
        {
          cu.tmpFracIdx = 1;
        }
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        else
        {
          cu.tmpFracIdx = 0;
        }
#endif
        cu.tmpIsSubPel  = 0;
        cu.tmpSubPelIdx = -1;
        DTRACE(g_trace_ctx, D_SYNTAX, "tmpFracIdx pos=(%d,%d) value=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpFracIdx);
#else
        cu.tmpIsSubPel = m_BinDecoder.decodeBin(Ctx::TmpFlag(4));
        if (cu.tmpIsSubPel)
        {
          cu.tmpIsSubPel += m_BinDecoder.decodeBin(Ctx::TmpFlag(5));
          if (cu.tmpIsSubPel == 2)
          {
            cu.tmpIsSubPel += m_BinDecoder.decodeBin(Ctx::TmpFlag(6));
          }
          cu.tmpSubPelIdx = m_BinDecoder.decodeBinsEP(3);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "tmp_is_subpel() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIsSubPel);
#endif
      }
      else
      {
        cu.tmpIsSubPel  = 0;
        cu.tmpSubPelIdx = -1;
      }
    }
  }
#endif
}
#endif

void CABACReader::mip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MIP, cu.lumaSize(), COMPONENT_Y);
#if ENABLE_DIMD && !JVET_AJ0249_NEURAL_NETWORK_BASED
  if (cu.dimd)
  {
    cu.mipFlag = false;
    return;
  }
#endif
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    cu.mipFlag = false;
    return;
  }


  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.mipFlag = m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACReader::mip_pred_modes( CodingUnit &cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MIP, cu.lumaSize(), COMPONENT_Y);

  if( !cu.Y().valid() )
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACReader::mip_pred_mode( PredictionUnit &pu )
{
  pu.mipTransposedFlag = bool(m_BinDecoder.decodeBinEP());

  uint32_t mipMode;
  const int numModes = getNumModesMip( pu.Y() );
  xReadTruncBinCode( mipMode, numModes );
  pu.intraDir[CHANNEL_TYPE_LUMA] = mipMode;
  CHECKD( pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode" );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA], pu.mipTransposedFlag ? 1 : 0 );
}

#if INTER_LIC
void CABACReader::cu_lic_flag( CodingUnit& cu )
{
  if( CU::isLICFlagPresent( cu ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__LIC_FLAG, cu.lumaSize() );
#if JVET_AG0276_LIC_SLOPE_ADJUST
    unsigned ctxId = DeriveCtx::CtxLicFlag( cu );
    cu.licFlag = m_BinDecoder.decodeBin( Ctx::LICFlag( ctxId ) );
#else
    cu.licFlag = m_BinDecoder.decodeBin( Ctx::LICFlag( 0 ) );
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.licFlag ? 1 : 0 );
#if JVET_AG0276_LIC_SLOPE_ADJUST
    if (cu.licFlag && CU::isLicSlopeAllowed(cu) && cu.firstPU->interDir != 3)
    {
      int delta = m_BinDecoder.decodeBin(Ctx::LicDelta(0)) ? 1 : 0;
      if (delta)
      {
        delta *= m_BinDecoder.decodeBin(Ctx::LicDelta(1)) ? -1 : +1;
      }
      cu.licDelta = delta;
    }
#endif
  }
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACReader::bvOneZeroComp(CodingUnit &cu)
{
  if (!CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    return;
  }
  cu.bvZeroCompDir = 0;
#if JVET_AA0070_RRIBC
  cu.rribcFlipType = 0;
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__IBC_BVD, cu.lumaSize(), COMPONENT_Y);

  unsigned ctxId   = DeriveCtx::CtxbvOneZeroComp(cu);
  cu.bvOneZeroComp = m_BinDecoder.decodeBin(Ctx::bvOneZeroComp(ctxId));
  if (cu.bvOneZeroComp)
  {
    // Read the BV direction
    cu.bvZeroCompDir = cu.bvOneZeroComp;
    cu.bvZeroCompDir += m_BinDecoder.decodeBin(Ctx::bvOneZeroComp(3));

#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (cu.cs->sps->getUseRRIbc())
    {
#endif
    ctxId            = DeriveCtx::CtxRribcFlipType(cu);
    cu.rribcFlipType = m_BinDecoder.decodeBin(Ctx::rribcFlipType(ctxId));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif
    if (cu.rribcFlipType)
    {
      cu.rribcFlipType = cu.bvZeroCompDir;
    }
#endif
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "rribc_data() rribc_flip_type = %d\n", cu.rribcFlipType);
}
#endif

#if JVET_AA0070_RRIBC
void CABACReader::rribcData(CodingUnit &cu)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (!cu.cs->sps->getUseRRIbc() || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
#else
  if (!CU::isIBC(cu) || cu.firstPU->mergeFlag)
#endif
  {
    return;
  }

  cu.rribcFlipType = 0;

  unsigned ctxId   = DeriveCtx::CtxRribcFlipType(cu);
  cu.rribcFlipType = (m_BinDecoder.decodeBin(Ctx::rribcFlipType(ctxId)));
  if (cu.rribcFlipType)
  {
    cu.rribcFlipType += m_BinDecoder.decodeBin(Ctx::rribcFlipType(3));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "rribc_data() rribc_flip_type = %d\n", cu.rribcFlipType);
}
#endif


#if SIGN_PREDICTION
void CABACReader::parsePredictedSigns( TransformUnit &tu, ComponentID compID )
{
  uint32_t bin;
  const CtxSet* ctx = &Ctx::signPred[toChannelType( compID )];
  int ctxOffset = CU::isIntra( *tu.cu ) ? 0 : 2;
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_AL0181_ASBT
  const bool lfnstEnabled = ((tu.checkLFNSTApplied(compID)) || ((tu.asbtDecimH[compID] || tu.asbtDecimW[compID])
                            && !((tu.asbtDecimH[compID] + tu.asbtDecimW[compID]) == 2 && (tu.blocks[compID].width <= 8 || tu.blocks[compID].height <= 8))
                            && !(tu.blocks[compID].width <= 4) && !(tu.blocks[compID].height <= 4)));
#else
  bool lfnstEnabled = tu.checkLFNSTApplied(compID);
#endif
#if JVET_AL0181_ASBT
  uint32_t minAreaSize = ((tu.asbtDecimH[compID] || tu.asbtDecimW[compID]) && compID > 0) ?
    std::min(tu.blocks[compID].height << (tu.asbtDecimH[compID] + tu.asbtDecimW[compID]), (tu.blocks[compID].width << (tu.asbtDecimH[compID] + tu.asbtDecimW[compID])))
    : 4;

  minAreaSize = std::min(minAreaSize, (uint32_t)4);

  const int32_t maxNumPredSigns = lfnstEnabled ? std::min<int>(minAreaSize, tu.cs->sps->getNumPredSigns()) : tu.cs->sps->getNumPredSigns();
#else
  const int32_t maxNumPredSigns = lfnstEnabled ? std::min<int>(4, tu.cs->sps->getNumPredSigns()) : tu.cs->sps->getNumPredSigns();
#endif
#else
  const int32_t maxNumPredSigns = tu.cs->sps->getNumPredSigns();
#endif
  const bool useSignPred = TU::getUseSignPred( tu, compID );
  int numSignPred = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT, tu.cu->block(compID).lumaSize(), compID);

  CoeffBuf buff = tu.getCoeffs( compID );
  AreaBuf<SIGN_PRED_TYPE> signBuff = tu.getCoeffSigns(compID);
  TCoeff *coeff = buff.buf;
  SIGN_PRED_TYPE         *signs    = signBuff.buf;
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_AL0181_ASBT
  uint32_t extAreaWidth = std::min(tu.blocksResidual[compID].width, (uint32_t)SIGN_PRED_FREQ_RANGE);
  uint32_t extAreaHeight = std::min(tu.blocksResidual[compID].height, (uint32_t)SIGN_PRED_FREQ_RANGE);
#else
  uint32_t extAreaWidth = std::min(tu.blocks[compID].width, (uint32_t)SIGN_PRED_FREQ_RANGE);
  uint32_t extAreaHeight = std::min(tu.blocks[compID].height, (uint32_t)SIGN_PRED_FREQ_RANGE);
#endif
#if JVET_AL0181_ASBT
  uint32_t extAreaSize = (lfnstEnabled ? minAreaSize : tu.cs->sps->getSignPredArea());
#else
  uint32_t extAreaSize = (lfnstEnabled ? 4 : tu.cs->sps->getSignPredArea());
#endif
#if JVET_AC0130_NSPT
#if JVET_AL0181_ASBT
  uint32_t spAreaWidth = useSignPred ? std::min( tu.blocksResidual[compID].width, extAreaSize ) : extAreaWidth;
  uint32_t spAreaHeight = useSignPred ? std::min( tu.blocksResidual[compID].height, extAreaSize ) : extAreaHeight;
#else
  uint32_t spAreaWidth = useSignPred ? std::min( tu.blocks[compID].width, extAreaSize ) : extAreaWidth;
  uint32_t spAreaHeight = useSignPred ? std::min( tu.blocks[compID].height, extAreaSize ) : extAreaHeight;
#endif
#else
  uint32_t spAreaWidth = useSignPred ? std::min(tu.blocks[compID].width, extAreaSize) : extAreaWidth;
  uint32_t spAreaHeight = useSignPred ? std::min(tu.blocks[compID].height, extAreaSize) : extAreaHeight;
#endif

  for ( uint32_t y = 0; y < spAreaHeight; y++ )
#else
  for ( uint32_t y = 0; y < SIGN_PRED_FREQ_RANGE; y++ )
#endif
  {
#if JVET_Y0141_SIGN_PRED_IMPROVE
    for ( uint32_t x = 0; x < spAreaWidth; x++ )
#else
    for( uint32_t x = 0; x < SIGN_PRED_FREQ_RANGE; x++ )
#endif
    {
      TCoeff coef = coeff[x];

      if( coef )
      {
        if (signs[x] != SIGN_PRED_HIDDEN)
        {
          if( !useSignPred || numSignPred >= maxNumPredSigns )
          {
            bin = m_BinDecoder.decodeBinEP();
          }
          else
          {
#if JVET_Y0141_SIGN_PRED_IMPROVE
            int levOffset = (coef < 2) ? 0 : 1;
            bin = m_BinDecoder.decodeBin((*ctx)(ctxOffset + levOffset));
#else
            uint32_t ctxId = ( x || y ) ? 1 : 0;
            bin = m_BinDecoder.decodeBin( ( *ctx )( ctxId + ctxOffset ) );
#endif
            numSignPred++;
          }

          if( bin )
          {
            coeff[x] = -coef;
          }
        }
      }
    }
    coeff += buff.stride;
    signs += signBuff.stride;
  }
#if JVET_Y0141_SIGN_PRED_IMPROVE
  if ( spAreaWidth != extAreaWidth || spAreaHeight != extAreaHeight )
  {
    coeff = buff.buf;
    for ( uint32_t y = 0; y < extAreaHeight; y++ )
    {
      uint32_t startX = (y < spAreaHeight) ? spAreaWidth : 0;
      uint32_t endX = extAreaWidth - 1;
      for ( uint32_t x = startX; x <= endX; x++ )
      {
        TCoeff coef = coeff[x];
        if (coef)
        {
          bin = m_BinDecoder.decodeBinEP();
          if (bin)
          {
            coeff[x] = -coef;
          }
        }
      }
      coeff += buff.stride;
    }
  }
#endif
}
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
void CABACReader::amvpMerge_mode( PredictionUnit& pu )
{
  if (PU::isBipredRestriction(pu))
  {
    CHECK(1, "this is not possible");
    return;
  }
  unsigned useAmMode = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AMVP_MERGE_FLAG, pu.lumaSize());
#if JVET_Y0128_NON_CTC
  if (pu.cu->slice->getUseAmvpMergeMode() == true)
#else
  if (!pu.cu->slice->getCheckLDC())
#endif
  {
    useAmMode = m_BinDecoder.decodeBin(Ctx::amFlagState());
    DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() am_flag=%d\n", useAmMode);
  }
  if (useAmMode)
  {
    pu.interDir = 3; // implicit BI prediction
    unsigned mergeDir = 0;
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (pu.cs->sps->getUseARL())
    {
      mergeDir = 1;
    }
    else
#endif
    if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() == false)
    {
      mergeDir = m_BinDecoder.decodeBinEP();
      DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() dir=%d\n", mergeDir);
    }
    else
    {
      // when L1ZeroFlag is true, only L1 can be mergeDir
      mergeDir = 1;
    }
    pu.mvpIdx[mergeDir] = 2;
    pu.amvpMergeModeFlag[mergeDir] = true;
    pu.amvpMergeModeFlag[1 - mergeDir] = false;
  }
}
#endif

#if JVET_AB0157_TMRL
void CABACReader::cuTmrlFlag(CodingUnit& cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_TMRL, cu.lumaSize(), COMPONENT_Y);

#if !JVET_AD0082_TMRL_CONFIG
  if (!CU::allowTmrl(cu))
  {
    cu.firstPU->multiRefIdx = 0;
    cu.tmrlFlag = false;
    cu.tmrlListIdx = 0;
    return;
  }
  PredictionUnit* pu = cu.firstPU;
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    CHECK(cu.tmrlFlag, "TMRL cannot combine with TIMD.");
    uint8_t bin = 0;
    bin = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(5)); // TIMD MRL
    if (bin)
    {
      bin += m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(6)); // which line
    }
    pu->multiRefIdx = MULTI_REF_LINE_IDX[bin];
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", bin, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
  }
  else
  {
#endif
#endif
    int ctxId = 0;
    cu.tmrlFlag = bool(m_BinDecoder.decodeBin(Ctx::TmrlDerive(ctxId++)));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_flag() ctx=%d pos=(%d,%d) tmrl=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlFlag);
    if (cu.tmrlFlag)
    {
      const int maxNumCtxBins = (MRL_LIST_SIZE / MRL_IDX_RICE_CODE_DIVISOR) - 1;
      int mrlIdxPrefix = 0;
      for (int val = 0; val < maxNumCtxBins; val++)
      {
        unsigned int bin = m_BinDecoder.decodeBin(Ctx::TmrlDerive(ctxId++));
        mrlIdxPrefix += bin;
        if (!bin)
        {
          break;
        }
      }
      uint32_t mrlSuffixBin0 = 0, mrlSuffixBin1 = 0;
      mrlSuffixBin0 = m_BinDecoder.decodeBin(Ctx::TmrlDerive(maxNumCtxBins + 1));
      mrlSuffixBin1 = m_BinDecoder.decodeBin(Ctx::TmrlDerive(maxNumCtxBins + 2));
      cu.tmrlListIdx = mrlIdxPrefix * MRL_IDX_RICE_CODE_DIVISOR + (mrlSuffixBin0 + (mrlSuffixBin1 << 1));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_idx() ctx=%d pos=(%d,%d) tmrlidx=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlListIdx);
    }
    else
    {
      cu.tmrlListIdx = 0;
    }
#if !JVET_AD0082_TMRL_CONFIG
#if JVET_W0123_TIMD_FUSION
  }
#endif
#endif
}
#endif

#if JVET_AE0059_INTER_CCCM
void CABACReader::interCccm(TransformUnit& tu)
{
  if (TU::interCccmAllowed(tu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTER_CCCM, tu.blocks[tu.chType].lumaSize(), tu.chType);
    tu.interCccm = m_BinDecoder.decodeBin(Ctx::InterCccmFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "inter_cccm() pos=(%d,%d) inter_cccm_flag=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.interCccm);
  }
}
#endif
#if JVET_AL0181_ASBT
void CABACReader::interAsbt(TransformUnit& tu)
{
  for (unsigned i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    tu.asbtDecimW[i] = 0;
    tu.asbtDecimH[i] = 0;
  }
  bool isDecimation = false;
  
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__ASBT_MODE, tu.cu->block(COMPONENT_Y).lumaSize());
  if (TU::isInterAsbt(tu))
  {
    bool interAsbtW = TU::isInterAsbtW(tu);
    bool interAsbtH = TU::isInterAsbtH(tu);

    if (interAsbtW || interAsbtH)
    {
      isDecimation = m_BinDecoder.decodeBin(Ctx::AsbtEnabledFlag(((tu.cu->sbtInfo != 0) ? 1 : 0)));
    }
    if (isDecimation)
    {
      if (interAsbtW && interAsbtH)
      {
        tu.asbtDecimW[COMPONENT_Y] = 1;
        tu.asbtDecimW[COMPONENT_Cr] = 1;
        tu.asbtDecimW[COMPONENT_Cb] = 1;
        tu.asbtDecimH[COMPONENT_Y] = tu.asbtDecimW[COMPONENT_Y] ? 0 : 1;
        tu.asbtDecimH[COMPONENT_Cr] = tu.asbtDecimW[COMPONENT_Cr] ? 0 : 1;
        tu.asbtDecimH[COMPONENT_Cb] = tu.asbtDecimW[COMPONENT_Cb] ? 0 : 1;
      }
      else
      {
        tu.asbtDecimW[COMPONENT_Y] = interAsbtW;
        tu.asbtDecimH[COMPONENT_Y] = interAsbtH;
        tu.asbtDecimW[COMPONENT_Cr] = interAsbtW;
        tu.asbtDecimH[COMPONENT_Cr] = interAsbtH;
        tu.asbtDecimW[COMPONENT_Cb] = interAsbtW;
        tu.asbtDecimH[COMPONENT_Cb] = interAsbtH;
      }

      if ((interAsbtW && interAsbtH) ? (std::max(tu.blocks[0].width, tu.blocks[0].height) > MIN_ASBT_DECIM_4_SIZE) : (tu.asbtDecimW[COMPONENT_Y] ? (tu.blocks[0].width > MIN_ASBT_DECIM_4_SIZE) : (tu.blocks[0].height > MIN_ASBT_DECIM_4_SIZE)))
      {
        if (m_BinDecoder.decodeBin(Ctx::AsbtSecondDecim(((tu.cu->sbtInfo != 0) ? 1 : 0))))
        {
          if (tu.asbtDecimW[COMPONENT_Y])
          {
            if ((tu.blocks[COMPONENT_Y].width >> 2) < 4)
            {
              tu.asbtDecimW[COMPONENT_Y] = 0;
              tu.asbtDecimH[COMPONENT_Y] = 2;
              tu.asbtDecimW[COMPONENT_Cr] = 0;
              tu.asbtDecimH[COMPONENT_Cr] = 2;
              tu.asbtDecimW[COMPONENT_Cb] = 0;
              tu.asbtDecimH[COMPONENT_Cb] = 2;
            }
            else
            {
              tu.asbtDecimW[COMPONENT_Y]++;
              tu.asbtDecimW[COMPONENT_Cr]++;
              tu.asbtDecimW[COMPONENT_Cb]++;
            }
          }
          else
          {
            tu.asbtDecimH[COMPONENT_Y]++;
            tu.asbtDecimH[COMPONENT_Cr]++;
            tu.asbtDecimH[COMPONENT_Cb]++;
          }
        }
      }
    }
  }
}
#endif

#if JVET_AF0073_INTER_CCP_MERGE
void CABACReader::interCcpMerge(TransformUnit& tu)
{
  if (TU::interCcpMergeAllowed(tu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTER_CCPMERGE, tu.blocks[tu.chType].lumaSize(), tu.chType);
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    tu.interCcpMerge = m_BinDecoder.decodeBin(Ctx::InterCcpMergeFlag(tu.cbf[COMPONENT_Y] ? 0 : 1));
#else
    tu.interCcpMerge = m_BinDecoder.decodeBin(Ctx::InterCcpMergeFlag(0));
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "inter_ccp_merge() pos=(%d,%d) inter_ccp_merge_flag=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.interCcpMerge);
  }
}
#endif
#if JVET_AL0153_ALF_CCCM
void CABACReader::lfCccm(CodingStructure& cs, const uint32_t ctuRsAddr)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_ALF_BLOCK_LEVEL_IDC );
  if(!cs.sps->getLfCccmEnabledFlag())
  {
    return;
  }
  if(!cs.slice->getLfCccmEnabledFlag())
  {
    return;
  }
  if(ctuRsAddr==0 && !cs.slice->isIntra() && cs.slice->lfCccmGetReferencePicture())
  {
    cs.slice->m_lfCccmFrameLevelInherit = m_BinDecoder.decodeBin(Ctx::LfCccmFlag(1));
  }
  if(cs.slice->m_lfCccmFrameLevelInherit)
  {
    return;
  }
  cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = m_BinDecoder.decodeBin(Ctx::LfCccmFlag(0));
  if (cs.slice->m_lfCccmEnabled.at(ctuRsAddr))
  {
    const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
    if(nCand)
    {
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr)= m_BinDecoder.decodeBin(Ctx::LfCccmFlag(2));
      if(cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) && nCand>1)
      {
        cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) += 2*m_BinDecoder.decodeBin(Ctx::LfCccmFlag(3));
      }
      if(cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr))
      {
        cs.slice->lfCccmMerge(ctuRsAddr);
        return;
      }
    }
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = 0;

    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += m_BinDecoder.decodeBin(Ctx::LfCccmFlag(4));
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += 2*m_BinDecoder.decodeBin(Ctx::LfCccmFlag(5));
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += 4*m_BinDecoder.decodeBin(Ctx::LfCccmFlag(6));

    cs.slice->m_lfCccmModelType.at(ctuRsAddr) = m_BinDecoder.decodeBin(Ctx::LfCccmFlag(7));
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) += 2*m_BinDecoder.decodeBin(Ctx::LfCccmFlag(8));
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) += 4*m_BinDecoder.decodeBin(Ctx::LfCccmFlag(9));
  }
}
#endif

