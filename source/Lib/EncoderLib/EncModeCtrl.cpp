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

/** \file     EncModeCtrl.cpp
    \brief    Encoder controller for trying out specific modes
*/

#include "EncModeCtrl.h"

#include "AQp.h"
#include "RateCtrl.h"

#include "CommonLib/RdCost.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"

#include <cmath>

void EncModeCtrl::init( EncCfg *pCfg, RateCtrl *pRateCtrl, RdCost* pRdCost )
{
  m_pcEncCfg      = pCfg;
  m_pcRateCtrl    = pRateCtrl;
  m_pcRdCost      = pRdCost;
  m_fastDeltaQP   = false;
#if SHARP_LUMA_DELTA_QP
  m_lumaQPOffset  = 0;

  initLumaDeltaQpLUT();
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_avoidComplexIntraInInterSlice = false;
  m_savedInterHad                 = MAX_UINT;
  m_treeIdx                       = 0;
#endif
}

bool EncModeCtrl::tryModeMaster( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
#if ENABLE_SPLIT_PARALLELISM
  if( m_ComprCUCtxList.back().isLevelSplitParallel )
  {
    if( !parallelJobSelector( encTestmode, cs, partitioner ) )
    {
      return false;
    }
  }
#endif
  return tryMode( encTestmode, cs, partitioner );
}

void EncModeCtrl::setEarlySkipDetected()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().earlySkip = true;
#else
  m_ComprCUCtxList.back().earlySkip = true;
#endif
}

void EncModeCtrl::xExtractFeatures( const EncTestMode encTestmode, CodingStructure& cs )
{
  CHECK( cs.features.size() < NUM_ENC_FEATURES, "Features vector is not initialized" );

  cs.features[ENC_FT_DISTORTION     ] = double( cs.dist              );
  cs.features[ENC_FT_FRAC_BITS      ] = double( cs.fracBits          );
  cs.features[ENC_FT_RD_COST        ] = double( cs.cost              );
  cs.features[ENC_FT_ENC_MODE_TYPE  ] = double( encTestmode.type     );
  cs.features[ENC_FT_ENC_MODE_OPTS  ] = double( encTestmode.opts     );
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
bool EncModeCtrl::nextMode( const CodingStructure &cs, Partitioner &partitioner )
{
  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().lastTestMode = m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.back();

  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.pop_back();

  while( !m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.empty() && !tryModeMaster( currTestMode(), cs, partitioner ) )
  {
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.pop_back();
  }

  return !m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.empty();
}
#else
bool EncModeCtrl::nextMode( const CodingStructure &cs, Partitioner &partitioner )
{
  m_ComprCUCtxList.back().lastTestMode = m_ComprCUCtxList.back().testModes.back();

  m_ComprCUCtxList.back().testModes.pop_back();

  while( !m_ComprCUCtxList.back().testModes.empty() && !tryModeMaster( currTestMode(), cs, partitioner ) )
  {
    m_ComprCUCtxList.back().testModes.pop_back();
  }

  return !m_ComprCUCtxList.back().testModes.empty();
}
#endif


EncTestMode EncModeCtrl::currTestMode() const
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_ComprCUCtxList[m_treeIdx].back().testModes.back();
#else
  return m_ComprCUCtxList.back().testModes.back();
#endif
}

EncTestMode EncModeCtrl::lastTestMode() const
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_ComprCUCtxList[m_treeIdx].back().lastTestMode;
#else
  return m_ComprCUCtxList.back().lastTestMode;
#endif
}

bool EncModeCtrl::anyMode() const
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return !m_ComprCUCtxList[m_treeIdx].back().testModes.empty();
#else
  return !m_ComprCUCtxList.back().testModes.empty();
#endif
}

void EncModeCtrl::setBest( CodingStructure& cs )
{
  if( cs.cost != MAX_DOUBLE && !cs.cus.empty() )
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[m_treeIdx].back().bestCS = &cs;
    m_ComprCUCtxList[m_treeIdx].back().bestCU = cs.cus[0];
    m_ComprCUCtxList[m_treeIdx].back().bestTU = cs.cus[0]->firstTU;
    m_ComprCUCtxList[m_treeIdx].back().lastTestMode = getCSEncMode( cs );
#else
    m_ComprCUCtxList.back().bestCS = &cs;
    m_ComprCUCtxList.back().bestCU = cs.cus[0];
    m_ComprCUCtxList.back().bestTU = cs.cus[0]->firstTU;
    m_ComprCUCtxList.back().lastTestMode = getCSEncMode( cs );
#endif
  }
}

#if JVET_AI0087_BTCUS_RESTRICTION 
bool EncModeCtrl::isLumaNonBoundaryCu(const Partitioner& partitioner, SizeType picWidth, SizeType picHeight)
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
#endif

void EncModeCtrl::xGetMinMaxQP( int& minQP, int& maxQP, const CodingStructure& cs, const Partitioner &partitioner, const int baseQP, const SPS& sps, const PPS& pps, const PartSplit splitMode )
{
  if( m_pcEncCfg->getUseRateCtrl() )
  {
    minQP = m_pcRateCtrl->getRCQP();
    maxQP = m_pcRateCtrl->getRCQP();
    return;
  }

  const unsigned subdivIncr = (splitMode == CU_QUAD_SPLIT) ? 2 : (splitMode == CU_BT_SPLIT) ? 1 : 0;
  const bool qgEnable = partitioner.currQgEnable(); // QG possible at current level
  const bool qgEnableChildren = qgEnable && ((partitioner.currSubdiv + subdivIncr) <= cs.slice->getCuQpDeltaSubdiv()) && (subdivIncr > 0); // QG possible at next level
  const bool isLeafQG = (qgEnable && !qgEnableChildren);

  if( isLeafQG ) // QG at deepest level
  {
    int deltaQP = m_pcEncCfg->getMaxDeltaQP();
    minQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP - deltaQP );
    maxQP = Clip3( -sps.getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, baseQP + deltaQP );
#if JVET_Y0240_BIM
    Position pos = partitioner.currQgPos;
    const int ctuSize = sps.getCTUSize();
    const int ctuId = ( pos.y / ctuSize ) * ( ( cs.picture->lwidth() + ctuSize - 1 ) / ctuSize ) + ( pos.x / ctuSize );
    const int bimOffset = getBIMOffset( m_slice->getPOC(), ctuId );
    minQP += bimOffset;
    maxQP += bimOffset;
#endif
  }
  else if( qgEnableChildren ) // more splits and not the deepest QG level
  {
    minQP = baseQP;
    maxQP = baseQP;
  }
  else // deeper than QG
  {
    minQP = cs.currQP[partitioner.chType];
    maxQP = cs.currQP[partitioner.chType];
  }
}


int EncModeCtrl::xComputeDQP( const CodingStructure &cs, const Partitioner &partitioner )
{
  Picture* picture    = cs.picture;
  unsigned uiAQDepth  = std::min( partitioner.currSubdiv/2, ( uint32_t ) picture->aqlayer.size() - 1 );
  AQpLayer* pcAQLayer = picture->aqlayer[uiAQDepth];

  double dMaxQScale   = pow( 2.0, m_pcEncCfg->getQPAdaptationRange() / 6.0 );
  double dAvgAct      = pcAQLayer->getAvgActivity();
  double dCUAct       = pcAQLayer->getActivity( cs.area.Y().topLeft() );
  double dNormAct     = ( dMaxQScale*dCUAct + dAvgAct ) / ( dCUAct + dMaxQScale*dAvgAct );
  double dQpOffset    = log( dNormAct ) / log( 2.0 ) * 6.0;
  int    iQpOffset    = int( floor( dQpOffset + 0.49999 ) );
  return iQpOffset;
}


#if SHARP_LUMA_DELTA_QP
void EncModeCtrl::initLumaDeltaQpLUT()
{
  const LumaLevelToDeltaQPMapping &mapping = m_pcEncCfg->getLumaLevelToDeltaQPMapping();

  if( !mapping.isEnabled() )
  {
    return;
  }

  // map the sparse LumaLevelToDeltaQPMapping.mapping to a fully populated linear table.

  int         lastDeltaQPValue = 0;
  std::size_t nextSparseIndex = 0;
  for( int index = 0; index < LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; index++ )
  {
    while( nextSparseIndex < mapping.mapping.size() && index >= mapping.mapping[nextSparseIndex].first )
    {
      lastDeltaQPValue = mapping.mapping[nextSparseIndex].second;
      nextSparseIndex++;
    }
    m_lumaLevelToDeltaQPLUT[index] = lastDeltaQPValue;
  }
}

int EncModeCtrl::calculateLumaDQP( const CPelBuf& rcOrg )
{
  double avg = 0;

  // Get QP offset derived from Luma level
#if !WCG_EXT
  if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_AVG_METHOD )
#else
  CHECK( m_pcEncCfg->getLumaLevelToDeltaQPMapping().mode != LUMALVL_TO_DQP_AVG_METHOD, "invalid delta qp mode" );
#endif
  {
    // Use average luma value
    avg = (double) rcOrg.computeAvg();
  }
#if !WCG_EXT
  else
  {
    // Use maximum luma value
    int maxVal = 0;
    for( uint32_t y = 0; y < rcOrg.height; y++ )
    {
      for( uint32_t x = 0; x < rcOrg.width; x++ )
      {
        const Pel& v = rcOrg.at( x, y );
        if( v > maxVal )
        {
          maxVal = v;
        }
      }
    }
    // use a percentage of the maxVal
    avg = ( double ) maxVal * m_pcEncCfg->getLumaLevelToDeltaQPMapping().maxMethodWeight;
  }
#endif
  int lumaBD = m_pcEncCfg->getBitDepth(CHANNEL_TYPE_LUMA);
  int lumaIdxOrg = Clip3<int>(0, int(1 << lumaBD) - 1, int(avg + 0.5));
  int lumaIdx = lumaBD < 10 ? lumaIdxOrg << (10 - lumaBD) : lumaBD > 10 ? lumaIdxOrg >> (lumaBD - 10) : lumaIdxOrg;
  int QP = m_lumaLevelToDeltaQPLUT[lumaIdx];
  return QP;
}
#endif

#if ENABLE_SPLIT_PARALLELISM
void EncModeCtrl::copyState( const EncModeCtrl& other, const UnitArea& area )
{
  m_slice          = other.m_slice;
  m_fastDeltaQP    = other.m_fastDeltaQP;
  m_lumaQPOffset   = other.m_lumaQPOffset;
  m_runNextInParallel
                   = other.m_runNextInParallel;
  m_ComprCUCtxList = other.m_ComprCUCtxList;
}

#endif
void CacheBlkInfoCtrl::create(const int maxCuSize)
{
  const unsigned numPos = maxCuSize >> MIN_CU_LOG2;
  m_maxCuSize = maxCuSize;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for (unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
    for( unsigned x = 0; x < numPos; x++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        m_codedCUInfo[treeIdx][x][y] = new CodedCUInfo**[m_numWidths];

        for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
        {
          if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( wIdx ) ) && x + ( gp_sizeIdxInfo->sizeFrom( wIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
          {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
            if (treeIdx > 0 && gp_sizeIdxInfo->sizeFrom(wIdx) > 64)
            {
              m_codedCUInfo[treeIdx][x][y][wIdx] = nullptr;
              continue;
            }
#endif
            m_codedCUInfo[treeIdx][x][y][wIdx] = new CodedCUInfo*[gp_sizeIdxInfo->numHeights()];

            for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              if (treeIdx > 0 && gp_sizeIdxInfo->sizeFrom(hIdx) > 64)
              {
                m_codedCUInfo[treeIdx][x][y][wIdx][hIdx] = nullptr;
                continue;
              }
#endif

              if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( hIdx ) ) && y + ( gp_sizeIdxInfo->sizeFrom( hIdx ) >> MIN_CU_LOG2 ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
              {
                m_codedCUInfo[treeIdx][x][y][wIdx][hIdx] = new CodedCUInfo;
              }
              else
              {
                m_codedCUInfo[treeIdx][x][y][wIdx][hIdx] = nullptr;
              }
            }
          }
          else
          {
            m_codedCUInfo[treeIdx][x][y][wIdx] = nullptr;
          }
        }
      }
    }
  }
#else
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      m_codedCUInfo[x][y] = new CodedCUInfo**[m_numWidths];

      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( wIdx ) ) && x + ( gp_sizeIdxInfo->sizeFrom( wIdx ) >> MIN_CU_LOG2 ) <= (maxCuSize >> MIN_CU_LOG2 ) )
        {
          m_codedCUInfo[x][y][wIdx] = new CodedCUInfo*[gp_sizeIdxInfo->numHeights()];

          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( hIdx ) ) && y + ( gp_sizeIdxInfo->sizeFrom( hIdx ) >> MIN_CU_LOG2 ) <= (maxCuSize >> MIN_CU_LOG2 ) )
            {
              m_codedCUInfo[x][y][wIdx][hIdx] = new CodedCUInfo;
            }
            else
            {
              m_codedCUInfo[x][y][wIdx][hIdx] = nullptr;
            }
          }
        }
        else
        {
          m_codedCUInfo[x][y][wIdx] = nullptr;
        }
      }
    }
  }
#endif
}

void CacheBlkInfoCtrl::destroy()
{
  const unsigned numPos = m_maxCuSize >> MIN_CU_LOG2;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for( unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
    for( unsigned x = 0; x < numPos; x++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
        {
          if( m_codedCUInfo[treeIdx][x][y][wIdx] )
          {
            for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
            {
              if( m_codedCUInfo[treeIdx][x][y][wIdx][hIdx] )
              {
                delete m_codedCUInfo[treeIdx][x][y][wIdx][hIdx];
              }
            }

            delete[] m_codedCUInfo[treeIdx][x][y][wIdx];
          }
        }

        delete[] m_codedCUInfo[treeIdx][x][y];
      }
    }
  }
#else
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_codedCUInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_codedCUInfo[x][y][wIdx][hIdx] )
            {
              delete m_codedCUInfo[x][y][wIdx][hIdx];
            }
          }

          delete[] m_codedCUInfo[x][y][wIdx];
        }
      }

      delete[] m_codedCUInfo[x][y];
    }
  }
#endif
}

void CacheBlkInfoCtrl::init( const Slice &slice )
{
  const unsigned numPos = m_maxCuSize >> MIN_CU_LOG2;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for( unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
    for( unsigned x = 0; x < numPos; x++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        if (!m_codedCUInfo[treeIdx][x][y])
        {
          continue;
        }
        for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
        {
          if( m_codedCUInfo[treeIdx][x][y][wIdx] )
          {
            for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
            {
              if( m_codedCUInfo[treeIdx][x][y][wIdx][hIdx] )
              {
                std::fill_n( reinterpret_cast< char * >(m_codedCUInfo[treeIdx][x][y][wIdx][hIdx]), sizeof( CodedCUInfo ), 0 );
              }
            }
          }
        }
      }
    }
  }
#else
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_codedCUInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_codedCUInfo[x][y][wIdx][hIdx] )
            {
              std::fill_n( reinterpret_cast< char * >(m_codedCUInfo[x][y][wIdx][hIdx]), sizeof( CodedCUInfo ), 0 );
            }
          }
        }
      }
    }
  }
#endif

  m_slice_chblk = &slice;
#if ENABLE_SPLIT_PARALLELISM

  m_currTemporalId = 0;
#endif
}
#if ENABLE_SPLIT_PARALLELISM

void CacheBlkInfoCtrl::touch( const UnitArea& area )
{
  CodedCUInfo& cuInfo = getBlkInfo( area );
  cuInfo.temporalId = m_currTemporalId;
}

void CacheBlkInfoCtrl::copyState( const CacheBlkInfoCtrl &other, const UnitArea& area )
{
  m_slice_chblk = other.m_slice_chblk;

  m_currTemporalId = other.m_currTemporalId;

  if( m_slice_chblk->isIntra() ) return;

  const int cuSizeMask = m_slice_chblk->getSPS()->getMaxCUWidth() - 1;

  const int minPosX = ( area.lx() & cuSizeMask ) >> MIN_CU_LOG2;
  const int minPosY = ( area.ly() & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosX = ( area.Y().bottomRight().x & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosY = ( area.Y().bottomRight().y & cuSizeMask ) >> MIN_CU_LOG2;

  for( unsigned x = minPosX; x <= maxPosX; x++ )
  {
    for( unsigned y = minPosY; y <= maxPosY; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        const int width = gp_sizeIdxInfo->sizeFrom( wIdx );

        if( m_codedCUInfo[x][y][wIdx] && width <= area.lwidth() && x + ( width >> MIN_CU_LOG2 ) <= ( maxPosX + 1 ) )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            const int height = gp_sizeIdxInfo->sizeFrom( hIdx );

            if( gp_sizeIdxInfo->isCuSize( height ) && height <= area.lheight() && y + ( height >> MIN_CU_LOG2 ) <= ( maxPosY + 1 ) )
            {
              if( other.m_codedCUInfo[x][y][wIdx][hIdx]->temporalId > m_codedCUInfo[x][y][wIdx][hIdx]->temporalId )
              {
                *m_codedCUInfo[x][y][wIdx][hIdx] = *other.m_codedCUInfo[x][y][wIdx][hIdx];
                m_codedCUInfo[x][y][wIdx][hIdx]->temporalId = m_currTemporalId;
              }
            }
            else if( y + ( height >> MIN_CU_LOG2 ) > maxPosY + 1 )
            {
              break;;
            }
          }
        }
        else if( x + ( width >> MIN_CU_LOG2 ) > maxPosX + 1 )
        {
          break;
        }
      }
    }
  }
}
#endif

CodedCUInfo& CacheBlkInfoCtrl::getBlkInfo( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return *m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4];
#else
  return *m_codedCUInfo[idx1][idx2][idx3][idx4];
#endif
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
CodedCUInfo* CacheBlkInfoCtrl::getBlkInfoPtr( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;

  if (m_treeIdx > 0 && (area.lwidth() > 64 || area.lheight() > 64))
  {
    return nullptr;
  }

  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4];
}
#endif

bool CacheBlkInfoCtrl::isSkip( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;

  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->isSkip;
#else
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->isSkip;
#endif
}

char CacheBlkInfoCtrl::getSelectColorSpaceOption(const UnitArea& area)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4);

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->selectColorSpaceOption;
#else
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->selectColorSpaceOption;
#endif
}

bool CacheBlkInfoCtrl::isMMVDSkip(const UnitArea& area)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4);

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->isMMVDSkip;
#else
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->isMMVDSkip;
#endif
}

void CacheBlkInfoCtrl::setMv( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, const Mv& rMv )
{
  if( iRefIdx >= MAX_STORED_CU_INFO_REFS ) return;

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->saveMv [refPicList][iRefIdx] = rMv;
  m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx] = true;
#else
  m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv [refPicList][iRefIdx] = rMv;
  m_codedCUInfo[idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx] = true;
#endif
#if ENABLE_SPLIT_PARALLELISM

  touch( area );
#endif
}

bool CacheBlkInfoCtrl::getMv( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, Mv& rMv ) const
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4 );

  if( iRefIdx >= MAX_STORED_CU_INFO_REFS )
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    rMv = m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->saveMv[refPicList][0];
#else
    rMv = m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv[refPicList][0];
#endif
    return false;
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  rMv = m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->saveMv[refPicList][iRefIdx];
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx];
#else
  rMv = m_codedCUInfo[idx1][idx2][idx3][idx4]->saveMv[refPicList][iRefIdx];
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->validMv[refPicList][iRefIdx];
#endif
}

void SaveLoadEncInfoSbt::init( const Slice &slice )
{
  m_sliceSbt = &slice;
}

void SaveLoadEncInfoSbt::create(const int maxCuSize)
{
  int numSizeIdx = gp_sizeIdxInfo->idxFrom( SBT_MAX_SIZE ) - MIN_CU_LOG2 + 1;
  const unsigned numPosIdx = maxCuSize >> MIN_CU_LOG2;
  m_maxCuSize = maxCuSize;

  m_saveLoadSbt = new SaveLoadStructSbt***[numPosIdx];

  for( int xIdx = 0; xIdx < numPosIdx; xIdx++ )
  {
    m_saveLoadSbt[xIdx] = new SaveLoadStructSbt**[numPosIdx];
    for( int yIdx = 0; yIdx < numPosIdx; yIdx++ )
    {
      m_saveLoadSbt[xIdx][yIdx] = new SaveLoadStructSbt*[numSizeIdx];
      for( int wIdx = 0; wIdx < numSizeIdx; wIdx++ )
      {
        m_saveLoadSbt[xIdx][yIdx][wIdx] = new SaveLoadStructSbt[numSizeIdx];
      }
    }
  }
}

void SaveLoadEncInfoSbt::destroy()
{
  int numSizeIdx = gp_sizeIdxInfo->idxFrom( SBT_MAX_SIZE ) - MIN_CU_LOG2 + 1;
  const int numPosIdx = m_maxCuSize >> MIN_CU_LOG2;

  for( int xIdx = 0; xIdx < numPosIdx; xIdx++ )
  {
    for( int yIdx = 0; yIdx < numPosIdx; yIdx++ )
    {
      for( int wIdx = 0; wIdx < numSizeIdx; wIdx++ )
      {
        delete[] m_saveLoadSbt[xIdx][yIdx][wIdx];
      }
      delete[] m_saveLoadSbt[xIdx][yIdx];
    }
    delete[] m_saveLoadSbt[xIdx];
  }
  delete[] m_saveLoadSbt;
}

uint16_t SaveLoadEncInfoSbt::findBestSbt( const UnitArea& area, const uint32_t curPuSse )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_sliceSbt->getPPS()->pcv, idx1, idx2, idx3, idx4 );
  SaveLoadStructSbt* pSbtSave = &m_saveLoadSbt[idx1][idx2][idx3 - MIN_CU_LOG2][idx4 - MIN_CU_LOG2];

  for( int i = 0; i < pSbtSave->numPuInfoStored; i++ )
  {
    if( curPuSse == pSbtSave->puSse[i] )
    {
      return pSbtSave->puSbt[i] + ( pSbtSave->puTrs[i] << 8 );
    }
  }

  return MAX_UCHAR + ( MAX_UCHAR << 8 );
}

bool SaveLoadEncInfoSbt::saveBestSbt( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt, const uint8_t curPuTrs )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( area.Y(), *m_sliceSbt->getPPS()->pcv, idx1, idx2, idx3, idx4 );
  SaveLoadStructSbt* pSbtSave = &m_saveLoadSbt[idx1][idx2][idx3 - MIN_CU_LOG2][idx4 - MIN_CU_LOG2];

  if( pSbtSave->numPuInfoStored == SBT_NUM_SL )
  {
    return false;
  }

  pSbtSave->puSse[pSbtSave->numPuInfoStored] = curPuSse;
  pSbtSave->puSbt[pSbtSave->numPuInfoStored] = curPuSbt;
  pSbtSave->puTrs[pSbtSave->numPuInfoStored] = curPuTrs;
  pSbtSave->numPuInfoStored++;
  return true;
}

#if ENABLE_SPLIT_PARALLELISM
void SaveLoadEncInfoSbt::copyState(const SaveLoadEncInfoSbt &other)
{
  m_sliceSbt = other.m_sliceSbt;
}
#endif

void SaveLoadEncInfoSbt::resetSaveloadSbt( int maxSbtSize )
{
  int numSizeIdx = gp_sizeIdxInfo->idxFrom( maxSbtSize ) - MIN_CU_LOG2 + 1;
  int numPosIdx = m_maxCuSize >> MIN_CU_LOG2;

  for( int xIdx = 0; xIdx < numPosIdx; xIdx++ )
  {
    for( int yIdx = 0; yIdx < numPosIdx; yIdx++ )
    {
      for( int wIdx = 0; wIdx < numSizeIdx; wIdx++ )
      {
        memset( m_saveLoadSbt[xIdx][yIdx][wIdx], 0, numSizeIdx * sizeof( SaveLoadStructSbt ) );
      }
    }
  }
}

bool CacheBlkInfoCtrl::getInter(const UnitArea& area)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4);

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->isInter;
#else
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->isInter;
#endif
}
void CacheBlkInfoCtrl::setBcwIdx(const UnitArea& area, uint8_t gBiIdx)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4);
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->bcwIdx = gBiIdx;
#else
  m_codedCUInfo[idx1][idx2][idx3][idx4]->bcwIdx = gBiIdx;
#endif
}
uint8_t CacheBlkInfoCtrl::getBcwIdx(const UnitArea& area)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_chblk->getPPS()->pcv, idx1, idx2, idx3, idx4);

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  return m_codedCUInfo[m_treeIdx][idx1][idx2][idx3][idx4]->bcwIdx;
#else
  return m_codedCUInfo[idx1][idx2][idx3][idx4]->bcwIdx;
#endif
}

#if REUSE_CU_RESULTS
static bool isTheSameNbHood( const CodingUnit &cu, const CodingStructure& cs, const Partitioner &partitioner
                            , const PredictionUnit &pu, int picW, int picH
                           )
{
  if( cu.chType != partitioner.chType )
  {
    return false;
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cu.isSST && ( !cu.slice->getProcessingIntraRegion() || cu.separateTree != cu.slice->getProcessingSeparateTrees() ) )
  {
    return false;
  }
#endif

  const PartitioningStack &ps = partitioner.getPartStack();

  int i = 1;

  for( ; i < ps.size(); i++ )
  {
    if( ps[i].split != CU::getSplitAtDepth( cu, i - 1 ) )
    {
      break;
    }
  }

  const UnitArea &cmnAnc = ps[i - 1].parts[ps[i - 1].idx];
  const UnitArea cuArea  = CS::getArea( cs, cu, partitioner.chType );
//#endif

  for( int i = 0; i < cmnAnc.blocks.size(); i++ )
  {
    if( i < cuArea.blocks.size() && cuArea.blocks[i].valid() && cuArea.blocks[i].pos() != cmnAnc.blocks[i].pos() )
    {
      return false;
    }
  }

  return true;
}

#if CONVERT_NUM_TU_SPLITS_TO_CFG
void BestEncInfoCache::create( const ChromaFormat chFmt, const int maxNumTUs, const int maxCuSize )
#else
void BestEncInfoCache::create( const ChromaFormat chFmt )
#endif
{
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  m_maxNumTUs = maxNumTUs;
#endif
  const unsigned numPos = maxCuSize >> MIN_CU_LOG2;
  m_maxCuSize = maxCuSize;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for (unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
#endif
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_bestEncInfo[treeIdx][x][y] = new BestEncodingInfo**[m_numWidths];
#else
      m_bestEncInfo[x][y] = new BestEncodingInfo**[m_numWidths];
#endif
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( wIdx ) ) && x + ( gp_sizeIdxInfo->sizeFrom( wIdx ) >> MIN_CU_LOG2 ) <= (maxCuSize >> MIN_CU_LOG2 ) )
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_bestEncInfo[treeIdx][x][y][wIdx] = new BestEncodingInfo*[gp_sizeIdxInfo->numHeights()];
#else
          m_bestEncInfo[x][y][wIdx] = new BestEncodingInfo*[gp_sizeIdxInfo->numHeights()];
#endif
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
            if (treeIdx > 0 )// && (gp_sizeIdxInfo->sizeFrom(wIdx) > 64 || gp_sizeIdxInfo->sizeFrom(wIdx)>64))
            {
              m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] = nullptr;
              continue;
            }
            else
#endif
            if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( hIdx ) ) && y + ( gp_sizeIdxInfo->sizeFrom( hIdx ) >> MIN_CU_LOG2 ) <= (maxCuSize >> MIN_CU_LOG2 ) )
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] = new BestEncodingInfo;
#else
              m_bestEncInfo[x][y][wIdx][hIdx] = new BestEncodingInfo;
#endif
              int w = gp_sizeIdxInfo->sizeFrom( wIdx );
              int h = gp_sizeIdxInfo->sizeFrom( hIdx );

              const UnitArea area( chFmt, Area( 0, 0, w, h ) );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              new ( &m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->cu ) CodingUnit    ( area );
              new ( &m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->pu ) PredictionUnit( area );
#else
              new ( &m_bestEncInfo[x][y][wIdx][hIdx]->cu ) CodingUnit    ( area );
              new ( &m_bestEncInfo[x][y][wIdx][hIdx]->pu ) PredictionUnit( area );
#endif
#if CONVERT_NUM_TU_SPLITS_TO_CFG
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->tus.clear();
#else
              m_bestEncInfo[x][y][wIdx][hIdx]->tus.clear();
#endif
              for( int i = 0; i < maxNumTUs; i++ )
              {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
                m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->tus.push_back( TransformUnit( area ) );
#else
                m_bestEncInfo[x][y][wIdx][hIdx]->tus.push_back( TransformUnit( area ) );
#endif
              }
#elif REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
              m_bestEncInfo[x][y][wIdx][hIdx]->numTus = 0;
              for( int i = 0; i < MAX_NUM_TUS; i++ )
              {
                new ( &m_bestEncInfo[x][y][wIdx][hIdx]->tus[i] ) TransformUnit( area );
              }
#else
              new ( &m_bestEncInfo[x][y][wIdx][hIdx]->tu ) TransformUnit( area );
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->poc      = -1;
#else
              m_bestEncInfo[x][y][wIdx][hIdx]->poc      = -1;
#endif
            }
            else
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] = nullptr;
#else
              m_bestEncInfo[x][y][wIdx][hIdx] = nullptr;
#endif
            }
          }
        }
        else
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_bestEncInfo[treeIdx][x][y][wIdx] = nullptr;
#else
          m_bestEncInfo[x][y][wIdx] = nullptr;
#endif
        }
      }
    }
  }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  }
#endif

}

void BestEncInfoCache::destroy()
{
  const unsigned numPos = m_maxCuSize >> MIN_CU_LOG2;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for (unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
    for( unsigned x = 0; x < numPos; x++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
        {
          if( m_bestEncInfo[treeIdx][x][y][wIdx] )
          {
            for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
            {
              if( m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] )
              {
                delete m_bestEncInfo[treeIdx][x][y][wIdx][hIdx];
              }
            }

            delete[] m_bestEncInfo[treeIdx][x][y][wIdx];
          }
        }

        delete[] m_bestEncInfo[treeIdx][x][y];
      }
    }
    delete[] m_pCoeff[treeIdx];
#if SIGN_PREDICTION
    delete[] m_pCoeffSign[treeIdx];
#if JVET_Y0141_SIGN_PRED_IMPROVE
    delete[] m_pCoeffSignScanIdx[treeIdx];
#endif
#endif

    delete[] m_pPcmBuf[treeIdx];

    if (m_runType[treeIdx] != nullptr)
    {
      delete[] m_runType[treeIdx];
      m_runType[treeIdx] = nullptr;
    }

  }


#else
  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        if( m_bestEncInfo[x][y][wIdx] )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            if( m_bestEncInfo[x][y][wIdx][hIdx] )
            {
              delete m_bestEncInfo[x][y][wIdx][hIdx];
            }
          }

          delete[] m_bestEncInfo[x][y][wIdx];
        }
      }

      delete[] m_bestEncInfo[x][y];
    }
  }

  delete[] m_pCoeff;
#if SIGN_PREDICTION
  delete[] m_pCoeffSign;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  delete[] m_pCoeffSignScanIdx;
#endif
#endif
  delete[] m_pPcmBuf;

  if (m_runType != nullptr)
  {
    delete[] m_runType;
    m_runType = nullptr;
  }

#endif

}

void BestEncInfoCache::init( const Slice &slice )
{
  bool isInitialized = m_slice_bencinf;

  m_slice_bencinf = &slice;

  if( isInitialized ) return;

  const unsigned numPos = slice.getSPS()->getMaxCUWidth() >> MIN_CU_LOG2;

  m_numWidths  = gp_sizeIdxInfo->numWidths();
  m_numHeights = gp_sizeIdxInfo->numHeights();

  size_t numCoeff = 0;



#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_treeIdx = 0;
  for (unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
    numCoeff = 0;
#endif

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if( m_bestEncInfo[treeIdx][x][y][wIdx] ) for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
        {
          if( m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] )
          {
            for( const CompArea& blk : m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->cu.blocks )
#else
        if (m_bestEncInfo[x][y][wIdx]) for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
        {
          if (m_bestEncInfo[x][y][wIdx][hIdx])
          {
            for (const CompArea& blk : m_bestEncInfo[x][y][wIdx][hIdx]->cu.blocks)
#endif
            {
              numCoeff += blk.area();
            }
          }
        }
      }
    }
  }


#if CONVERT_NUM_TU_SPLITS_TO_CFG

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_pCoeff[treeIdx] = new TCoeff[numCoeff*m_maxNumTUs];
#if SIGN_PREDICTION
  m_pCoeffSign[treeIdx] = new SIGN_PRED_TYPE[numCoeff*m_maxNumTUs];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  m_pCoeffSignScanIdx[treeIdx] = new unsigned[numCoeff*m_maxNumTUs];
#endif
#endif
  m_pPcmBuf[treeIdx] = new Pel[numCoeff*m_maxNumTUs];

  if( slice.getSPS()->getPLTMode() )
  {
    m_runType[treeIdx] = new bool[numCoeff*m_maxNumTUs];
  }
#else
  m_pCoeff = new TCoeff[numCoeff*m_maxNumTUs];
#if SIGN_PREDICTION
  m_pCoeffSign = new SIGN_PRED_TYPE[numCoeff*m_maxNumTUs];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  m_pCoeffSignScanIdx = new unsigned[numCoeff*m_maxNumTUs];
#endif
#endif
  m_pPcmBuf = new Pel[numCoeff*m_maxNumTUs];

  if( slice.getSPS()->getPLTMode() )
  {
    m_runType = new bool[numCoeff*m_maxNumTUs];
  }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  }
#endif


#elif REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  m_pCoeff  = new TCoeff[numCoeff*MAX_NUM_TUS];
#if SIGN_PREDICTION
  m_pCoeffSign = new SIGN_PRED_TYPE[numCoeff * MAX_NUM_TUS];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  m_pCoeffSignScanIdx = new unsigned[numCoeff*MAX_NUM_TUS];
#endif
#endif
  m_pPcmBuf = new Pel   [numCoeff*MAX_NUM_TUS];
  if (slice.getSPS()->getPLTMode())
  {
    m_runType   = new bool[numCoeff*MAX_NUM_TUS];
  }
#else
  m_pCoeff  = new TCoeff[numCoeff];
#if SIGN_PREDICTION
  m_pCoeffSign = new SIGN_PRED_TYPE[numCoeff];
#endif
  m_pPcmBuf = new Pel   [numCoeff];
  if (slice.getSPS()->getPLTMode())
  {
    m_runType   = new bool[numCoeff];
  }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for (unsigned treeIdx = 0; treeIdx < MAX_TREE_TYPE; treeIdx++ )
  {
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  TCoeff *coeffPtr = m_pCoeff[treeIdx];
#if SIGN_PREDICTION
  SIGN_PRED_TYPE *coeffSignPtr = m_pCoeffSign[treeIdx];
  
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned *coeffSignScanIdx = m_pCoeffSignScanIdx[treeIdx];
#endif
#endif
  Pel    *pcmPtr   = m_pPcmBuf[treeIdx];
  bool   *runTypePtr   = m_runType[treeIdx];
#else
    TCoeff *coeffPtr = m_pCoeff;
#if SIGN_PREDICTION
    SIGN_PRED_TYPE *coeffSignPtr = m_pCoeffSign;

#if JVET_Y0141_SIGN_PRED_IMPROVE
    unsigned *coeffSignScanIdx = m_pCoeffSignScanIdx;
#endif
#endif
    Pel    *pcmPtr   = m_pPcmBuf;
    bool   *runTypePtr   = m_runType;
#endif


  m_dummyCS.pcv = m_slice_bencinf->getPPS()->pcv;

  for( unsigned x = 0; x < numPos; x++ )
  {
    for( unsigned y = 0; y < numPos; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if( m_bestEncInfo[treeIdx][x][y][wIdx] ) for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
        {
          if( m_bestEncInfo[treeIdx][x][y][wIdx][hIdx] )
#else
        if( m_bestEncInfo[x][y][wIdx] ) for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
        {
          if( m_bestEncInfo[x][y][wIdx][hIdx] )
#endif
          {
            TCoeff *coeff[MAX_NUM_TBLOCKS] = { 0, };
#if SIGN_PREDICTION
            SIGN_PRED_TYPE *sign[MAX_NUM_TBLOCKS] = {
              0,
            };
#if JVET_Y0141_SIGN_PRED_IMPROVE
            unsigned *sign_scanIdx[MAX_NUM_TBLOCKS] = { 0, };
#endif
#endif
            Pel    *pcmbf[MAX_NUM_TBLOCKS] = { 0, };
            bool   *runType[MAX_NUM_TBLOCKS - 1] = { 0, };

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
#if CONVERT_NUM_TU_SPLITS_TO_CFG
            for( int j = 0; j < m_maxNumTUs; j++ )
#else
            for( int j = 0; j < MAX_NUM_TUS; j++ )
#endif
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              TransformUnit &tu = m_bestEncInfo[treeIdx][x][y][wIdx][hIdx]->tus[j];
#else
              TransformUnit &tu = m_bestEncInfo[x][y][wIdx][hIdx]->tus[j];
#endif
              const UnitArea &area = tu;

              for( int i = 0; i < area.blocks.size(); i++ )
              {
                coeff[i] = coeffPtr; coeffPtr += area.blocks[i].area();
#if SIGN_PREDICTION
                sign[i] = coeffSignPtr; coeffSignPtr += area.blocks[i].area();
#if JVET_Y0141_SIGN_PRED_IMPROVE
                sign_scanIdx[i] = coeffSignScanIdx; coeffSignScanIdx += area.blocks[i].area();
#endif
#endif
                pcmbf[i] = pcmPtr;   pcmPtr += area.blocks[i].area();
                if (i < 2)
                {
                  runType[i]   = runTypePtr;   runTypePtr   += area.blocks[i].area();
                }
              }

              tu.cs = &m_dummyCS;
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
              tu.init(coeff, sign, sign_scanIdx, pcmbf, runType);
#else
              tu.init(coeff, sign, pcmbf, runType);
#endif
#else
              tu.init(coeff, pcmbf, runType);
#endif
            }
#else
            const UnitArea &area = m_bestEncInfo[x][y][wIdx][hIdx]->tu;

            for( int i = 0; i < area.blocks.size(); i++ )
            {
              coeff[i] = coeffPtr; coeffPtr += area.blocks[i].area();
              pcmbf[i] =   pcmPtr;   pcmPtr += area.blocks[i].area();
              if (i < 2)
              {
                runType[i] = runTypePtr;   runTypePtr += area.blocks[i].area();
              }
#if SIGN_PREDICTION
              sign[i] = coeffSignPtr; coeffSignPtr += area.blocks[i].area();
#endif
              //runLength[i] = runLengthPtr; runLengthPtr += area.blocks[i].area();
            }

            m_bestEncInfo[x][y][wIdx][hIdx]->tu.cs = &m_dummyCS;
#if SIGN_PREDICTION
            m_bestEncInfo[x][y][wIdx][hIdx]->tu.init(coeff, sign, pcmbf, runType);
#else
            m_bestEncInfo[x][y][wIdx][hIdx]->tu.init(coeff, pcmbf, runType);
#endif
#endif
          }
        }
      }
    }
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  }
#endif

#if ENABLE_SPLIT_PARALLELISM

  m_currTemporalId = 0;
#endif
}

bool BestEncInfoCache::setFromCs( const CodingStructure& cs, const Partitioner& partitioner )
{
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  if( cs.cus.size() != 1 || ( m_maxNumTUs == 1 && cs.tus.size() != 1 ) || cs.pus.size() != 1 )
#elif REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  if( cs.cus.size() != 1 || cs.pus.size() != 1 )
#else
  if( cs.cus.size() != 1 || cs.tus.size() != 1 || cs.pus.size() != 1 )
#endif
  {
    return false;
  }

  unsigned idx1, idx2, idx3, idx4;

  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  BestEncodingInfo& encInfo = *m_bestEncInfo[m_treeIdx][idx1][idx2][idx3][idx4];
#else
  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (!(m_bestEncInfo[m_treeIdx][idx1][idx2][idx3][idx4]))
  {
    return false;
  }
#endif
  encInfo.poc            =  cs.picture->poc;
  encInfo.cu.repositionTo( *cs.cus.front() );
  encInfo.pu.repositionTo( *cs.pus.front() );
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS && !CONVERT_NUM_TU_SPLITS_TO_CFG
  encInfo.tu.repositionTo( *cs.tus.front() );
#endif
  encInfo.cu             = *cs.cus.front();
  encInfo.pu             = *cs.pus.front();
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
  int tuIdx = 0;
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  CHECK( encInfo.tus.size() < cs.tus.size(), "Wrong number of TUs" );
#endif

  for( auto tu : cs.tus )
  {
    encInfo.tus[tuIdx].repositionTo( *tu );
    encInfo.tus[tuIdx].resizeTo( *tu );
    for( auto &blk : tu->blocks )
    {
      if( blk.valid() )
      {
        encInfo.tus[tuIdx].copyComponentFrom( *tu, blk.compID );
      }
    }
    tuIdx++;
  }

#if CONVERT_NUM_TU_SPLITS_TO_CFG
  CHECKD( cs.tus.size() > m_maxNumTUs, "Exceeding tus array boundaries" );
#else
  CHECKD( cs.tus.size() > MAX_NUM_TUS, "Exceeding tus array boundaries" );
#endif

#else
  for( auto &blk : cs.tus.front()->blocks )
  {
    if( blk.valid() ) encInfo.tu.copyComponentFrom( *cs.tus.front(), blk.compID );
  }
#endif

  return true;
}

bool BestEncInfoCache::isValid( const CodingStructure& cs, const Partitioner& partitioner, int qp )
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( partitioner.treeType == TREE_C )
  {
    return false; //if save & load is allowed for chroma CUs, we should check whether luma info (pred, recon, etc) is the same, which is quite complex
  }
#endif
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  BestEncodingInfo& encInfo = *m_bestEncInfo[m_treeIdx][idx1][idx2][idx3][idx4];
#else
  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];
#endif

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( encInfo.cu.treeType != partitioner.treeType || encInfo.cu.modeType != partitioner.modeType )
  {
    return false;
  }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (!(m_bestEncInfo[m_treeIdx][idx1][idx2][idx3][idx4]))
  {
    return false;
  }
#endif

  if( encInfo.cu.qp != qp )
    return false;
  if( cs.picture->poc != encInfo.poc || CS::getArea( cs, cs.area, partitioner.chType ) != CS::getArea( cs, encInfo.cu, partitioner.chType ) || !isTheSameNbHood( encInfo.cu, cs, partitioner
    , encInfo.pu, (cs.picture->Y().width), (cs.picture->Y().height)
)
    || CU::isIBC(encInfo.cu)
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    || (CU::isIntra(encInfo.cu) && (!cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled()))
#endif
    || partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp
    )
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool BestEncInfoCache::setCsFrom( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx( cs.area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4 );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  BestEncodingInfo& encInfo = *m_bestEncInfo[m_treeIdx][idx1][idx2][idx3][idx4];
#else
  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];
#endif

  if( cs.picture->poc != encInfo.poc || CS::getArea( cs, cs.area, partitioner.chType ) != CS::getArea( cs, encInfo.cu, partitioner.chType ) || !isTheSameNbHood( encInfo.cu, cs, partitioner
    , encInfo.pu, (cs.picture->Y().width), (cs.picture->Y().height)
    )
    || partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp
    )
  {
    return false;
  }

  CodingUnit     &cu = cs.addCU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );
  PredictionUnit &pu = cs.addPU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS && !CONVERT_NUM_TU_SPLITS_TO_CFG
  TransformUnit  &tu = cs.addTU( CS::getArea( cs, cs.area, partitioner.chType ), partitioner.chType );
#endif

  cu          .repositionTo( encInfo.cu );
  pu          .repositionTo( encInfo.pu );
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS && !CONVERT_NUM_TU_SPLITS_TO_CFG
  tu          .repositionTo( encInfo.tu );
#endif

  cu          = encInfo.cu;
  pu          = encInfo.pu;
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
  CHECKD( !( encInfo.tus.size() > 0), "Empty tus array");
  for( int i = 0; i < encInfo.tus.size(); i++ )
  {
    TransformUnit  &tu = cs.addTU( encInfo.tus[i], partitioner.chType );

    for( auto &blk : tu.blocks )
    {
      if( blk.valid() ) tu.copyComponentFrom( encInfo.tus[i], blk.compID );
    }
  }
#else
  for( auto &blk : tu.blocks )
  {
    if( blk.valid() ) tu.copyComponentFrom( encInfo.tu, blk.compID );
  }
#endif


  return true;
}

#if ENABLE_SPLIT_PARALLELISM
void BestEncInfoCache::copyState(const BestEncInfoCache &other, const UnitArea &area)
{
  m_slice_bencinf  = other.m_slice_bencinf;
  m_currTemporalId = other.m_currTemporalId;

  if( m_slice_bencinf->isIntra() ) return;

  const int cuSizeMask = m_slice_bencinf->getSPS()->getMaxCUWidth() - 1;

  const int minPosX = ( area.lx() & cuSizeMask ) >> MIN_CU_LOG2;
  const int minPosY = ( area.ly() & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosX = ( area.Y().bottomRight().x & cuSizeMask ) >> MIN_CU_LOG2;
  const int maxPosY = ( area.Y().bottomRight().y & cuSizeMask ) >> MIN_CU_LOG2;

  for( unsigned x = minPosX; x <= maxPosX; x++ )
  {
    for( unsigned y = minPosY; y <= maxPosY; y++ )
    {
      for( int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++ )
      {
        const int width = gp_sizeIdxInfo->sizeFrom( wIdx );

        if( m_bestEncInfo[x][y][wIdx] && width <= area.lwidth() && x + ( width >> MIN_CU_LOG2 ) <= ( maxPosX + 1 ) )
        {
          for( int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++ )
          {
            const int height = gp_sizeIdxInfo->sizeFrom( hIdx );

            if( gp_sizeIdxInfo->isCuSize( height ) && height <= area.lheight() && y + ( height >> MIN_CU_LOG2 ) <= ( maxPosY + 1 ) )
            {
              if( other.m_bestEncInfo[x][y][wIdx][hIdx]->temporalId > m_bestEncInfo[x][y][wIdx][hIdx]->temporalId )
              {
                m_bestEncInfo[x][y][wIdx][hIdx]->cu       = other.m_bestEncInfo[x][y][wIdx][hIdx]->cu;
                m_bestEncInfo[x][y][wIdx][hIdx]->pu       = other.m_bestEncInfo[x][y][wIdx][hIdx]->pu;
                m_bestEncInfo[x][y][wIdx][hIdx]->numTus   = other.m_bestEncInfo[x][y][wIdx][hIdx]->numTus;
                m_bestEncInfo[x][y][wIdx][hIdx]->poc      = other.m_bestEncInfo[x][y][wIdx][hIdx]->poc;
                m_bestEncInfo[x][y][wIdx][hIdx]->testMode = other.m_bestEncInfo[x][y][wIdx][hIdx]->testMode;

                for( int i = 0; i < m_bestEncInfo[x][y][wIdx][hIdx]->numTus; i++ )
                  m_bestEncInfo[x][y][wIdx][hIdx]->tus[i] = other.m_bestEncInfo[x][y][wIdx][hIdx]->tus[i];
              }
            }
            else if( y + ( height >> MIN_CU_LOG2 ) > maxPosY + 1 )
            {
              break;;
            }
          }
        }
        else if( x + ( width >> MIN_CU_LOG2 ) > maxPosX + 1 )
        {
          break;
        }
      }
    }
  }
}

void BestEncInfoCache::touch(const UnitArea &area)
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(area.Y(), *m_slice_bencinf->getPPS()->pcv, idx1, idx2, idx3, idx4);
  BestEncodingInfo &encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  encInfo.temporalId = m_currTemporalId;
}

#endif

#endif

static bool interHadActive( const ComprCUCtx& ctx )
{
  return ctx.interHad != 0;
}

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlQTBT
//////////////////////////////////////////////////////////////////////////

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void EncModeCtrlMTnoRQT::setTreeIdx()
{
  EncModeCtrl::m_treeIdx = 0;
  if ( m_slice->getSeparateTreeEnabled() && m_slice->getProcessingIntraRegion() && m_slice->getProcessingSeparateTrees() && !m_slice->isIntra())
  {
    EncModeCtrl::m_treeIdx = ( m_slice->getProcessingChannelType() == CH_L ) ? 1 : 2;
  } 
  CacheBlkInfoCtrl::m_treeIdx = EncModeCtrl::m_treeIdx;
  BestEncInfoCache::m_treeIdx = EncModeCtrl::m_treeIdx;
}
#endif

void EncModeCtrlMTnoRQT::create( const EncCfg& cfg )
{
#if JVET_Z0118_GDR
  m_encCfg = cfg;
#endif
  CacheBlkInfoCtrl::create(cfg.getCTUSize());
#if REUSE_CU_RESULTS
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  BestEncInfoCache::create( cfg.getChromaFormatIdc(), cfg.getMaxNumTUs(), cfg.getCTUSize() );
#else
  BestEncInfoCache::create( cfg.getChromaFormatIdc() );
#endif
#endif
  SaveLoadEncInfoSbt::create(cfg.getCTUSize());
}

void EncModeCtrlMTnoRQT::destroy()
{
  CacheBlkInfoCtrl::destroy();
#if REUSE_CU_RESULTS
  BestEncInfoCache::destroy();
#endif
  SaveLoadEncInfoSbt::destroy();
}

void EncModeCtrlMTnoRQT::initCTUEncoding( const Slice &slice )
{
  CacheBlkInfoCtrl::init( slice );
#if REUSE_CU_RESULTS
  BestEncInfoCache::init( slice );
#endif
  SaveLoadEncInfoSbt::init( slice );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CHECK( !m_ComprCUCtxList[0].empty(), "Mode list is not empty at the beginning of a CTU" );
#else
  CHECK( !m_ComprCUCtxList.empty(), "Mode list is not empty at the beginning of a CTU" );
#endif

  m_slice             = &slice;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  setTreeIdx();
#endif
#if ENABLE_SPLIT_PARALLELISM
  m_runNextInParallel      = false;
#endif

  if( m_pcEncCfg->getUseE0023FastEnc() )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    const int thres = m_pcEncCfg->getUseCompositeRef() ? 2 * PICTURE_DISTANCE_TH : PICTURE_DISTANCE_TH;
    m_skipThreshold = ((slice.getMinPictureDistance(m_pcEncCfg->getIBCFastMethod()) <= thres) ? FAST_SKIP_DEPTH : SKIP_DEPTH);
#else
    if (m_pcEncCfg->getUseCompositeRef())
      m_skipThreshold = ( ( slice.getMinPictureDistance() <= PICTURE_DISTANCE_TH * 2 ) ? FAST_SKIP_DEPTH : SKIP_DEPTH );
    else
      m_skipThreshold = ((slice.getMinPictureDistance() <= PICTURE_DISTANCE_TH) ? FAST_SKIP_DEPTH : SKIP_DEPTH);
#endif

  }
  else
  {
    m_skipThreshold = SKIP_DEPTH;
  }
}

void EncModeCtrlMTnoRQT::initCULevel( Partitioner &partitioner, const CodingStructure& cs )
{
  // Min/max depth
  unsigned minDepth = 0;
  unsigned maxDepth = floorLog2(cs.sps->getCTUSize()) - floorLog2(cs.sps->getMinQTSize( m_slice->getSliceType(), partitioner.chType ));
  if( m_pcEncCfg->getUseFastLCTU() )
  {
    if( auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
    {
      // LARGE CTU
      adPartitioner->setMaxMinDepth( minDepth, maxDepth, cs );
    }
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  setTreeIdx();
  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].push_back( ComprCUCtx( cs, minDepth, maxDepth, NUM_EXTRA_FEATURES ) );
#else
  m_ComprCUCtxList.push_back( ComprCUCtx( cs, minDepth, maxDepth, NUM_EXTRA_FEATURES ) );
#endif

#if JVET_AI0087_BTCUS_RESTRICTION
  bool disableBTV = false;
  bool disableBTH = false;
  if (EncModeCtrl::isLumaNonBoundaryCu(partitioner, cs.picture->lwidth(), cs.picture->lheight()) 
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    && (!(cs.slice->getProcessingIntraRegion() && cs.slice->getProcessingSeparateTrees()) || cs.slice->isIntra()) 
#endif
    )
  {
    if ((partitioner.currBtDepth == 1) && (partitioner.currPartIdx() == 1))
    {
      if (partitioner.currPartLevel().split == CU_HORZ_SPLIT)   // BTH Case
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
        if (partitioner.currArea().lheight() == 16 && cs.btFirstPartDecs[3] == CU_HORZ_SPLIT)
        {
          disableBTH = true;
        }
      }
    }
  }
#endif

#if ENABLE_SPLIT_PARALLELISM
  if( m_runNextInParallel )
  {
    for( auto &level : m_ComprCUCtxList )
    {
      CHECK( level.isLevelSplitParallel, "Tring to parallelize a level within parallel execution!" );
    }
    CHECK( cs.picture->scheduler.getSplitJobId() == 0, "Trying to run a parallel level although jobId is 0!" );
    m_runNextInParallel                          = false;
    m_ComprCUCtxList[m_treeIdx].back().isLevelSplitParallel = true;
  }

#endif

#if JVET_AH0135_TEMPORAL_PARTITIONING
  const CodingUnit* cuLeft = cs.getCU(cs.area.blocks[partitioner.chType].pos().offset(-1, 0), partitioner.chType);
  const CodingUnit* cuAbove = cs.getCU(cs.area.blocks[partitioner.chType].pos().offset(0, -1), partitioner.chType);

  bool qtBeforeBt = ((cuLeft && cuAbove && cuLeft->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth)
    || (cuLeft && !cuAbove && cuLeft->qtDepth > partitioner.currQtDepth)
    || (!cuLeft && cuAbove && cuAbove->qtDepth > partitioner.currQtDepth)
    || (!cuAbove && !cuLeft && cs.area.lwidth() >= (32 << cs.slice->getDepth()))
    )
    && (cs.area.lwidth() > (cs.pcv->getMinQtSize(*cs.slice, partitioner.chType) << 1));

  unsigned maxBTD;
  bool canNo, canQt, canBh, canBv, canTh, canTv;

  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv, maxBTD 
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  );

  if (!canBh && !canBv)
  {
    qtBeforeBt = true;
  }
#else
  const CodingUnit* cuLeft  = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( -1, 0 ), partitioner.chType );
  const CodingUnit* cuAbove = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( 0, -1 ), partitioner.chType );

  const bool qtBeforeBt = ( (  cuLeft  &&  cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth )
                         || (  cuLeft  && !cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth )
                         || ( !cuLeft  &&  cuAbove  && cuAbove->qtDepth > partitioner.currQtDepth )
                         || ( !cuAbove && !cuLeft   && cs.area.lwidth() >= ( 32 << cs.slice->getDepth() ) ) )
                         && ( cs.area.lwidth() > ( cs.pcv->getMinQtSize( *cs.slice, partitioner.chType ) << 1 ) );

#endif

  // set features
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  ComprCUCtx &cuECtx  = m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back();
  bool isReusingCu    = false;
  if ( !cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() && cs.slice->getProcessingIntraRegion() && 
    cs.slice->isIntraRegionRoot( &partitioner ) && !cs.slice->getProcessingSeparateTrees() && 
    m_nonIntraCUECtx->bestCU && !m_nonIntraCUECtx->bestCU->isSST )
  {
    cuECtx = *m_nonIntraCUECtx;
    cuECtx.testModes.clear();
    isReusingCu = cuECtx.get<bool>( IS_REUSING_CU );
  }
  else
  {
#else
  ComprCUCtx &cuECtx  = m_ComprCUCtxList.back();
#endif
  cuECtx.set( BEST_NON_SPLIT_COST,  MAX_DOUBLE );
  cuECtx.set( BEST_VERT_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_HORZ_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_TRIH_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( BEST_TRIV_SPLIT_COST, MAX_DOUBLE );
  cuECtx.set( DO_TRIH_SPLIT,        1 );
  cuECtx.set( DO_TRIV_SPLIT,        1 );
  cuECtx.set( BEST_IMV_COST,        MAX_DOUBLE * .5 );
  cuECtx.set( BEST_NO_IMV_COST,     MAX_DOUBLE * .5 );
#if JVET_W0097_GPM_MMVD_TM
  cuECtx.set( BEST_GPM_COST,        MAX_DOUBLE * .5);
#endif
#if JVET_AD0213_LIC_IMP
  cuECtx.set(BEST_LIC_COST,         MAX_DOUBLE * .5);
#endif
  cuECtx.set( QT_BEFORE_BT,         qtBeforeBt );
  cuECtx.set( DID_QUAD_SPLIT,       false );
  cuECtx.set( IS_BEST_NOSPLIT_SKIP, false );
  cuECtx.set( MAX_QT_SUB_DEPTH,     0 );
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  }
#endif

  // QP
  int baseQP = cs.baseQP;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!CS::isDualITree(cs) || isLuma(partitioner.chType))
#else
  if (!partitioner.isSepTree(cs) || isLuma(partitioner.chType))
#endif
  {
    if (m_pcEncCfg->getUseAdaptiveQP())
    {
      baseQP = Clip3(-cs.sps->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, baseQP + xComputeDQP(cs, partitioner));
    }
#if ENABLE_QPA_SUB_CTU
    else if (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && cs.pps->getUseDQP() && cs.slice->getCuQpDeltaSubdiv() > 0)
    {
      const PreCalcValues &pcv = *cs.pcv;

      if ((partitioner.currArea().lwidth() < pcv.maxCUWidth) && (partitioner.currArea().lheight() < pcv.maxCUHeight) && cs.picture)
      {
        const Position    &pos = partitioner.currQgPos;
        const unsigned mtsLog2 = (unsigned)floorLog2(std::min (cs.sps->getMaxTbSize(), pcv.maxCUWidth));
        const unsigned  stride = pcv.maxCUWidth >> mtsLog2;

        baseQP = cs.picture->m_subCtuQP[((pos.x & pcv.maxCUWidthMask) >> mtsLog2) + stride * ((pos.y & pcv.maxCUHeightMask) >> mtsLog2)];
      }
    }
#endif
#if SHARP_LUMA_DELTA_QP
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      if (partitioner.currQgEnable())
      {
        m_lumaQPOffset = calculateLumaDQP (cs.getOrgBuf (clipArea (cs.area.Y(), cs.picture->Y())));
      }
      baseQP = Clip3 (-cs.sps->getQpBDOffset (CHANNEL_TYPE_LUMA), MAX_QP, baseQP - m_lumaQPOffset);
    }
#endif
  }
  int minQP = baseQP;
  int maxQP = baseQP;

  xGetMinMaxQP( minQP, maxQP, cs, partitioner, baseQP, *cs.sps, *cs.pps, CU_QUAD_SPLIT );
  bool checkIbc = true;
  if (partitioner.chType == CHANNEL_TYPE_CHROMA)
  {
    checkIbc = false;
  }

  // Add coding modes here
  // NOTE: Working back to front, as a stack, which is more efficient with the container
  // NOTE: First added modes will be processed at the end.

  //////////////////////////////////////////////////////////////////////////
  // Add unit split modes

  if( !cuECtx.get<bool>( QT_BEFORE_BT ) )
  {
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_QT, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_QT, ETO_STANDARD, qp } );
#endif
    }
  }

#if JVET_AH0135_TEMPORAL_PARTITIONING 
  if ( canTv )
#else
  if( partitioner.canSplit( CU_TRIV_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  ) )
#endif
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_TT_V, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_TT_V, ETO_STANDARD, qp } );
#endif
    }
  }

#if JVET_AH0135_TEMPORAL_PARTITIONING
  if ( canTh )
#else
  if( partitioner.canSplit( CU_TRIH_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  ) )
#endif
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_TT_H, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_TT_H, ETO_STANDARD, qp } );
#endif
    }
  }

  int minQPq = minQP;
  int maxQPq = maxQP;
  xGetMinMaxQP( minQP, maxQP, cs, partitioner, baseQP, *cs.sps, *cs.pps, CU_BT_SPLIT );
#if JVET_AH0135_TEMPORAL_PARTITIONING
  if ( canBv
#if JVET_AI0087_BTCUS_RESTRICTION 
    && !(disableBTV)
#endif
    )
#else
  if( partitioner.canSplit( CU_VERT_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
    , disableBTV, disableBTH
#endif
  ) )
#endif
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_BT_V, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_BT_V, ETO_STANDARD, qp } );
#endif
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().set( DID_VERT_SPLIT, true );
#else
    m_ComprCUCtxList.back().set( DID_VERT_SPLIT, true );
#endif
  }
  else
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().set( DID_VERT_SPLIT, false );
#else
    m_ComprCUCtxList.back().set( DID_VERT_SPLIT, false );
#endif
  }

#if JVET_AH0135_TEMPORAL_PARTITIONING
  if ( canBh
#if JVET_AI0087_BTCUS_RESTRICTION 
    && !(disableBTH)
#endif
    )
#else
  if( partitioner.canSplit( CU_HORZ_SPLIT, cs
#if JVET_AI0087_BTCUS_RESTRICTION
    , disableBTV, disableBTH
#endif
  ) )
#endif
  {
    // add split modes
    for( int qp = maxQP; qp >= minQP; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_BT_H, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_BT_H, ETO_STANDARD, qp } );
#endif
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().set( DID_HORZ_SPLIT, true );
#else
    m_ComprCUCtxList.back().set( DID_HORZ_SPLIT, true );
#endif
  }
  else
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().set( DID_HORZ_SPLIT, false );
#else
    m_ComprCUCtxList.back().set( DID_HORZ_SPLIT, false );
#endif
  }

  if( cuECtx.get<bool>( QT_BEFORE_BT ) )
  {
#if JVET_AH0135_TEMPORAL_PARTITIONING
    if ( canQt )
#endif
    for( int qp = maxQPq; qp >= minQPq; qp-- )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SPLIT_QT, ETO_STANDARD, qp } );
#else
      m_ComprCUCtxList.back().testModes.push_back( { ETM_SPLIT_QT, ETO_STANDARD, qp } );
#endif
    }
  }

#if JVET_AH0135_TEMPORAL_PARTITIONING
  if ( canNo )
  {
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_POST_DONT_SPLIT } );
#else
    m_ComprCUCtxList.back().testModes.push_back( { ETM_POST_DONT_SPLIT } );
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  }
  else
  {
    return;
  }
#endif
  xGetMinMaxQP( minQP, maxQP, cs, partitioner, baseQP, *cs.sps, *cs.pps, CU_DONT_SPLIT );

  int  lowestQP = minQP;

  //////////////////////////////////////////////////////////////////////////
  // Add unit coding modes: Intra, InterME, InterMerge ...
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool tryIntraRdo = true;
  bool tryInterRdo = true;
  bool tryIBCRdo   = true;
  if( partitioner.isConsIntra() )
  {
    tryInterRdo = false;
  }
  else if( partitioner.isConsInter() )
  {
    tryIntraRdo = tryIBCRdo = false;
  }
  checkIbc &= tryIBCRdo;
#endif
  for( int qpLoop = maxQP; qpLoop >= minQP; qpLoop-- )
  {
    const int  qp       = std::max( qpLoop, lowestQP );
#if REUSE_CU_RESULTS
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    isReusingCu = isValid( cs, partitioner, qp );
#else
    const bool isReusingCu = isValid( cs, partitioner, qp );
#endif
    cuECtx.set( IS_REUSING_CU, isReusingCu );
    if( isReusingCu )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( {ETM_RECO_CACHED, ETO_STANDARD, qp} );
#else
      m_ComprCUCtxList.back().testModes.push_back( {ETM_RECO_CACHED, ETO_STANDARD, qp} );
#endif
    }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (cs.slice->getSliceType() == I_SLICE && cs.slice->getUseIBC() && checkIbc && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC ))
    {
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (m_pcEncCfg->getIbcMerge() && partitioner.chType == CHANNEL_TYPE_LUMA)
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#else
        m_ComprCUCtxList.back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#endif
      }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_IBC,         ETO_STANDARD,  qp });
#else
      m_ComprCUCtxList.back().testModes.push_back({ ETM_IBC,         ETO_STANDARD,  qp });
#endif
#else
      m_ComprCUCtxList.back().testModes.push_back({ ETM_IBC,         ETO_STANDARD,  qp });
      if (m_pcEncCfg->getIbcMerge() && partitioner.chType == CHANNEL_TYPE_LUMA)
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#else
        m_ComprCUCtxList[m_treeIdx].back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#endif
      }
#endif
    }
#endif
    // add intra modes
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cs.slice->getSPS()->getPLTMode() && ( cs.slice->isIntra() || (cs.area.lwidth() == 4 && cs.area.lheight() == 4)) && getPltEnc())
#else
    if( tryIntraRdo )
    {
    if (cs.slice->getSPS()->getPLTMode() && (partitioner.treeType != TREE_D || cs.slice->isIntra() || (cs.area.lwidth() == 4 && cs.area.lheight() == 4)) && getPltEnc())
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_PALETTE, ETO_STANDARD, qp });
#else
      m_ComprCUCtxList.back().testModes.push_back({ ETM_PALETTE, ETO_STANDARD, qp });
#endif
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_SEPARATE_TREE_INTRA, ETO_STANDARD, qp });

    m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_INTRA, ETO_STANDARD, qp } );
#else
    m_ComprCUCtxList.back().testModes.push_back( { ETM_INTRA, ETO_STANDARD, qp } );
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cs.slice->getSPS()->getPLTMode() && !cs.slice->isIntra() && !(cs.area.lwidth() == 4 && cs.area.lheight() == 4) && getPltEnc())
#else
    if (cs.slice->getSPS()->getPLTMode() && partitioner.treeType == TREE_D && !cs.slice->isIntra() && !(cs.area.lwidth() == 4 && cs.area.lheight() == 4) && getPltEnc())
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_PALETTE,  ETO_STANDARD, qp });
#else
      m_ComprCUCtxList.back().testModes.push_back({ ETM_PALETTE,  ETO_STANDARD, qp });
#endif
    }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    }
#endif
    // add ibc mode to intra path
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (cs.slice->getUseIBC() && checkIbc && (cs.slice->getSliceType() != I_SLICE || !(m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC )))
#else
    if (cs.sps->getIBCFlag() && checkIbc)
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_IBC,         ETO_STANDARD,  qp });
#else
      m_ComprCUCtxList.back().testModes.push_back({ ETM_IBC,         ETO_STANDARD,  qp });
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if( m_pcEncCfg->getIbcMerge() )
#endif
      if (partitioner.chType == CHANNEL_TYPE_LUMA)
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#else
        m_ComprCUCtxList.back().testModes.push_back({ ETM_IBC_MERGE,   ETO_STANDARD,  qp });
#endif
      }
    }
  }

  // add first pass modes
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!m_slice->isIntra() )
#else
  if ( !m_slice->isIntra() && tryInterRdo )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!m_slice->isIntra() && !(cs.area.lwidth() == 4 && cs.area.lheight() == 4))
#else
  if ( !m_slice->isIntra() && !( cs.area.lwidth() == 4 && cs.area.lheight() == 4 ) && tryInterRdo )
#endif
#endif
  {
    for( int qpLoop = maxQP; qpLoop >= minQP; qpLoop-- )
    {
      const int  qp       = std::max( qpLoop, lowestQP );
#if INTER_LIC
      bool useLIC         = m_slice->getUseLIC();
#endif
#if MULTI_HYP_PRED
      if (cs.sps->getUseInterMultiHyp() && m_slice->isInterB())
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_MULTIHYP, ETO_STANDARD, qp });
#else
        m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_MULTIHYP, ETO_STANDARD, qp });
#endif
      }
#endif
      if (m_pcEncCfg->getIMV())
      {
#if INTER_LIC
        // inter with imv and illumination compensation
        if (useLIC)
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((4 << ETO_IMV_SHIFT) | ETO_LIC), qp });
#else
          m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((4 << ETO_IMV_SHIFT) | ETO_LIC), qp });
#endif
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_ME,  EncTestModeOpts( 4 << ETO_IMV_SHIFT ), qp });
#else
        m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_ME,  EncTestModeOpts( 4 << ETO_IMV_SHIFT ), qp });
#endif
      }
      if( m_pcEncCfg->getIMV() || m_pcEncCfg->getUseAffineAmvr() )
      {
        int imv = m_pcEncCfg->getIMV4PelFast() ? 3 : 2;
#if INTER_LIC
        // inter with imv and illumination compensation
        if (useLIC)
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((imv << ETO_IMV_SHIFT) | ETO_LIC), qp });
#else
          m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((imv << ETO_IMV_SHIFT) | ETO_LIC), qp });
#endif
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_INTER_ME, EncTestModeOpts( imv << ETO_IMV_SHIFT ), qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME, EncTestModeOpts( imv << ETO_IMV_SHIFT ), qp } );
#endif
#if INTER_LIC
        // inter with imv and illumination compensation
        if (useLIC)
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((1 << ETO_IMV_SHIFT) | ETO_LIC), qp });
#else
          m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_ME, EncTestModeOpts((1 << ETO_IMV_SHIFT) | ETO_LIC), qp });
#endif
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_INTER_ME, EncTestModeOpts( 1 << ETO_IMV_SHIFT ), qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME, EncTestModeOpts( 1 << ETO_IMV_SHIFT ), qp } );
#endif
      }

#if INTER_LIC
      // inter with illumination compensation
      if (useLIC)
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_INTER_ME,    ETO_LIC,      qp });
#else
        m_ComprCUCtxList.back().testModes.push_back({ ETM_INTER_ME,    ETO_LIC,      qp });
#endif
      }
#endif

      // add inter modes
      if( m_pcEncCfg->getUseEarlySkipDetection() )
      {
#if JVET_Y0065_GPM_INTRA
        if( cs.sps->getUseGeo() && !cs.slice->isIntra() )
#else
        if( cs.sps->getUseGeo() && cs.slice->isInterB() )
#endif
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_MERGE_GEO, ETO_STANDARD, qp } );
#else
          m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_GEO, ETO_STANDARD, qp } );
#endif
        }
#if TM_MRG && !MERGE_ENC_OPT
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        if (cs.sps->getUseTMMrgMode())
#else
        if (cs.sps->getUseDMVDMode())
#endif
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back({ ETM_MERGE_TM,     ETO_STANDARD, qp });
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_MERGE_SKIP,  ETO_STANDARD, qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_SKIP,  ETO_STANDARD, qp } );
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
        if (cs.sps->getUseAffineMmvdMode())
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back({ ETM_AF_MMVD,    ETO_STANDARD, qp });
        }
#endif
#if !MERGE_ENC_OPT
        if ( cs.sps->getUseAffine() || cs.sps->getSbTMVPEnabledFlag() )
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back( { ETM_AFFINE,    ETO_STANDARD, qp } );
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_INTER_ME,    ETO_STANDARD, qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME,    ETO_STANDARD, qp } );
#endif
      }
      else
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_INTER_ME,    ETO_STANDARD, qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_INTER_ME,    ETO_STANDARD, qp } );
#endif
#if JVET_Y0065_GPM_INTRA
        if( cs.sps->getUseGeo() && !cs.slice->isIntra() )
#else
        if( cs.sps->getUseGeo() && cs.slice->isInterB() )
#endif
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_MERGE_GEO, ETO_STANDARD, qp } );
#else
          m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_GEO, ETO_STANDARD, qp } );
#endif
        }
#if TM_MRG && !MERGE_ENC_OPT
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        if (cs.sps->getUseTMMrgMode())
#else
        if (cs.sps->getUseDMVDMode())
#endif
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back( { ETM_MERGE_TM,     ETO_STANDARD, qp } );
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back( { ETM_MERGE_SKIP,  ETO_STANDARD, qp } );
#else
        m_ComprCUCtxList.back().testModes.push_back( { ETM_MERGE_SKIP,  ETO_STANDARD, qp } );
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
        if (cs.sps->getUseAffineMmvdMode())
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back({ ETM_AF_MMVD,    ETO_STANDARD, qp });
        }
#endif
#if !MERGE_ENC_OPT
        if ( cs.sps->getUseAffine() || cs.sps->getSbTMVPEnabledFlag() )
        {
          m_ComprCUCtxList[m_treeIdx].back().testModes.push_back( { ETM_AFFINE,    ETO_STANDARD, qp } );
        }
#endif
      }
      if (m_pcEncCfg->getUseHashME())
      {
        int minSize = min(cs.area.lwidth(), cs.area.lheight());
        if (minSize < 128 && minSize >= 4)
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.push_back({ ETM_HASH_INTER, ETO_STANDARD, qp });
#else
          m_ComprCUCtxList.back().testModes.push_back({ ETM_HASH_INTER, ETO_STANDARD, qp });
#endif
        }
      }
    }
  }

  // ensure to skip unprobable modes
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if( !tryModeMaster( m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().testModes.back(), cs, partitioner ) )
#else
  if( !tryModeMaster( m_ComprCUCtxList.back().testModes.back(), cs, partitioner ) )
#endif
  {
    nextMode( cs, partitioner );
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back().lastTestMode = EncTestMode();
#else
  m_ComprCUCtxList.back().lastTestMode = EncTestMode();
#endif

}

void EncModeCtrlMTnoRQT::finishCULevel( Partitioner &partitioner )
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_ComprCUCtxList[EncModeCtrl::m_treeIdx].pop_back();
#else
  m_ComprCUCtxList.pop_back();
#endif
}



#if JVET_AI0136_ADAPTIVE_DUAL_TREE
static bool doNotTest(  const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner  )
{
  if ( cs.slice->getSeparateTreeEnabled() && encTestmode.type != ETM_POST_DONT_SPLIT && encTestmode.type != ETM_RECO_CACHED )
  {
    // When processing SST only test ETM_INTRA/split
    if ( cs.slice->getProcessingIntraRegion() && encTestmode.type != ETM_INTRA && encTestmode.type != ETM_IBC  && encTestmode.type != ETM_IBC_MERGE && 
      (!cs.slice->getSPS()->getPLTMode() || encTestmode.type != ETM_PALETTE) && 
      !isModeSplit( encTestmode.type ) )
    {
      return true;
    }
    // When not processing SST, do not test ETM_INTRA (since intra is tested under SST)
    if ( !cs.slice->isIntra() && !cs.slice->getProcessingIntraRegion() && encTestmode.type == ETM_INTRA )
    {
      return true;
    }
    // When processing SST, do not split shared tree
    if ( cs.slice->getProcessingIntraRegion() && !cs.slice->getProcessingSeparateTrees() && isModeSplit( encTestmode.type ) )
    {
      return true;
    }

    const bool isBlInPic = cs.picture->Y().contains( partitioner.currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->Y().contains( partitioner.currArea().Y().topRight() );
    if( isBlInPic && isTrInPic ) // not boundary
    {
      // inter + processing SST
      if ( !cs.slice->isIntra() && cs.slice->getProcessingIntraRegion() )
      {
        if ( encTestmode.type != ETM_INTRA && !isModeSplit( encTestmode ) )
        {
          return true;
        }
        if ( cs.slice->getIntraRegionNoSplitTest() && isModeSplit( encTestmode ) )
        {
          return true;
        }
      }
    }
  }

  return false;
}
#endif



bool EncModeCtrlMTnoRQT::tryMode( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && ( partitioner.currArea().lwidth() <= 256 && partitioner.currArea().lheight() <= 256 ) )
  {
    if ( encTestmode.type == ETM_SEPARATE_TREE_INTRA )
    {
      return true;
    }
    else
    {
      return false;
    }
}
#else
  if ( cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() )
  {
    if ( encTestmode.type == ETM_SEPARATE_TREE_INTRA )
    {
      return false;
    }
  }
#endif

  if ( doNotTest( encTestmode, cs, partitioner ) )
  {
    return false;
  }

  ComprCUCtx& cuECtx = m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back();
#else
  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();
#endif     

#if JVET_AJ0226_MTT_SKIP   
  if (m_pcEncCfg->getUseMttSkip() && partitioner.currBtDepth == 0 && (partitioner.currArea().lwidth() == partitioner.currArea().lheight()) &&
      (partitioner.currArea().lwidth() == 64 || partitioner.currArea().lwidth() == 32 ))
  {
    if (((partitioner.currArea().Y().x + partitioner.currArea().lwidth()-1  < cs.picture->lwidth())
      && (partitioner.currArea().Y().y + partitioner.currArea().lwidth()-1  < cs.picture->lheight()))
      && (encTestmode.type == ETM_SPLIT_BT_H || encTestmode.type == ETM_SPLIT_BT_V
        || encTestmode.type == ETM_SPLIT_TT_H || encTestmode.type == ETM_SPLIT_TT_V)
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
      && (partitioner.chType == CHANNEL_TYPE_LUMA || (partitioner.chType == CHANNEL_TYPE_CHROMA && cs.slice->getSliceType() == I_SLICE && cs.pcv->getMaxTtSize(*cs.slice, CHANNEL_TYPE_CHROMA) < 128))
#else
      && partitioner.chType == CHANNEL_TYPE_LUMA 
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      && (!(cs.slice->getProcessingIntraRegion() && cs.slice->getProcessingSeparateTrees()) || cs.slice->isIntra())
#endif
      )
    {
      double splitSignalCostScaling = Clip3(0.0, 4.0, (4.0 * pow(0.5, (m_pcEncCfg->getBaseQP() - 22.0) / 5)));
      int modeCabacCost = 0;

      if (encTestmode.type == ETM_SPLIT_TT_H)
      {
        modeCabacCost = partitioner.currArea().lwidth() == 64 ? m_cabacBitsforTTH[2] : partitioner.currArea().lwidth() == 32 ? m_cabacBitsforTTH[3] : 0;
      }
      else if (encTestmode.type == ETM_SPLIT_TT_V)
      {
        modeCabacCost = partitioner.currArea().lwidth() == 64 ? m_cabacBitsforTTV[2] : partitioner.currArea().lwidth() == 32 ? m_cabacBitsforTTV[3] : 0;
      }
      else if (encTestmode.type == ETM_SPLIT_BT_H)
      {
        modeCabacCost = partitioner.currArea().lwidth() == 64 ? m_cabacBitsforBTH[2] : partitioner.currArea().lwidth() == 32 ? m_cabacBitsforBTH[3] : 0;
      }
      else if (encTestmode.type == ETM_SPLIT_BT_V)
      {
        modeCabacCost = partitioner.currArea().lwidth() == 64 ? m_cabacBitsforBTV[2] : partitioner.currArea().lwidth() == 32 ? m_cabacBitsforBTV[3] : 0;
      }

      int thresholdMTT = Clip3(0, MAX_INT, (140 - ((cs.slice->getSliceQp() - 22) * 3)) * (1000000 - (int)(modeCabacCost * splitSignalCostScaling) ));
      double noSplitCost = partitioner.currArea().lwidth() == 64 ? m_noSplitIntraRdCost64CU : partitioner.currArea().lwidth() == 32 ? m_noSplitIntraRdCost32CU :  0;

      if (noSplitCost > thresholdMTT)
      {
        const PartSplit split = getPartSplit(encTestmode);
        if (split == CU_HORZ_SPLIT)
        {
          cuECtx.set(DID_HORZ_SPLIT, false);
        }
        if (split == CU_VERT_SPLIT)
        {
          cuECtx.set(DID_VERT_SPLIT, false);
        }
        if (split == CU_TRIH_SPLIT)
        {
          cuECtx.set(DO_TRIH_SPLIT, false);
        }
        if (split == CU_TRIV_SPLIT)
        {
          cuECtx.set(DO_TRIV_SPLIT, false);
        }
        return false;
      }
    }
  }
#endif

  // Fast checks, partitioning depended
#if MERGE_ENC_OPT
  if (cuECtx.isHashPerfectMatch && encTestmode.type != ETM_MERGE_SKIP && encTestmode.type != ETM_INTER_ME 
    && encTestmode.type != ETM_MERGE_GEO
#if AFFINE_MMVD && !MERGE_ENC_OPT
    && encTestmode.type != ETM_AF_MMVD
#endif
#if TM_MRG && !MERGE_ENC_OPT
    && encTestmode.type != ETM_MERGE_TM
#endif
    )
#else
  if (cuECtx.isHashPerfectMatch && encTestmode.type != ETM_MERGE_SKIP && encTestmode.type != ETM_INTER_ME && encTestmode.type != ETM_AFFINE && encTestmode.type != ETM_MERGE_GEO
#if AFFINE_MMVD
     && encTestmode.type != ETM_AF_MMVD
#endif
#if TM_MRG
     && encTestmode.type != ETM_MERGE_TM
#endif
    )
#endif
  {
#if JVET_AG0276_NLIC
    if (encTestmode.type != ETM_POST_DONT_SPLIT)
#endif
    return false;
  }

  // if early skip detected, skip all modes checking but the splits
  if( cuECtx.earlySkip && m_pcEncCfg->getUseEarlySkipDetection() && !isModeSplit( encTestmode ) && !( isModeInter( encTestmode ) ) )
  {
    return false;
  }

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
  const bool isBoundary         = implicitSplit != CU_DONT_SPLIT;

  if( isBoundary && encTestmode.type != ETM_SPLIT_QT )
  {
    return getPartSplit( encTestmode ) == implicitSplit;
  }
  else if( isBoundary && encTestmode.type == ETM_SPLIT_QT )
  {
    return partitioner.canSplit( CU_QUAD_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    );
  }

#if REUSE_CU_RESULTS
  if( cuECtx.get<bool>( IS_REUSING_CU ) )
  {
    if( encTestmode.type == ETM_RECO_CACHED )
    {
      return true;
    }

    if( isModeNoSplit( encTestmode ) )
    {
      return false;
    }
  }

#endif
  const Slice&           slice       = *m_slice;
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const SPS&             sps         = *slice.getSPS();
#endif
  const uint32_t             numComp     = getNumberValidComponents( slice.getSPS()->getChromaFormatIdc() );
  const uint32_t             width       = partitioner.currArea().lumaSize().width;
  const CodingStructure *bestCS      = cuECtx.bestCS;
#if JVET_AG0276_NLIC
  CodingUnit            *bestCU = cuECtx.bestCU;
#else
  const CodingUnit      *bestCU      = cuECtx.bestCU;
#endif
  const EncTestMode      bestMode    = bestCS ? getCSEncMode( *bestCS ) : EncTestMode();


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CodedCUInfo    *relatedCU          = getBlkInfoPtr( partitioner.currArea() );
#else
  CodedCUInfo    &relatedCU          = getBlkInfo( partitioner.currArea() );
#endif

  if( cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit( CU_QUAD_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif

  ) )
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    bool doIntra = encTestmode.type == ETM_INTRA && cs.slice->getSeparateTreeEnabled()
                   && cs.slice->getIntraRegionNoSplitTest() && !bestCS;
    if (!doIntra)
#endif
    {
      // enforce QT
      return encTestmode.type == ETM_SPLIT_QT;
    }
  }
  else if (encTestmode.type == ETM_SPLIT_QT && cuECtx.maxDepth <= partitioner.currQtDepth)
  {
#if JVET_AI0087_BTCUS_RESTRICTION  
        if (!(partitioner.chType == CHANNEL_TYPE_LUMA && partitioner.currBtDepth == 0 && (partitioner.currArea().lwidth() == 128 || partitioner.currArea().lwidth() == 64 || partitioner.currArea().lwidth() == 32)))
#endif      
        {
          // don't check this QT depth
          return false;
        }      
  }
  if( bestCS && bestCS->cus.size() == 1 )
  {
    // update the best non-split cost
    cuECtx.set( BEST_NON_SPLIT_COST, bestCS->cost );
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS  
  if (partitioner.currArea().Y().valid() && lastTestMode().type == ETM_INTRA && bestCS && bestCS->cus.size() == 1)
  {
    int cnt = 0;
    for (auto tu : bestCS->tus)
    {
      cnt += tu->countNonZero();
    }
    cuECtx.set(BEST_INTRA_NZ_CNT, cnt);
  }
#endif
  if( encTestmode.type == ETM_INTRA )
  {
    if( getFastDeltaQp() )
    {
      if( cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize )
      {
        return false; // only check necessary 2Nx2N Intra in fast delta-QP mode
      }
    }

#if TU_256
#if JVET_AL0274_MAX_TB_SIZE_FIX
    int maxTbSize = cs.sps->getMaxTbSize();
    const int maxSize = std::min<int>(maxTbSize, MAX_INTRA_SIZE);
#else
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );
#endif

    if( m_pcEncCfg->getUseFastLCTU() && partitioner.currArea().lumaSize().area() > maxSize * maxSize )
    {
      return ( m_pcEncCfg->getDualITree() == 0 && m_pcEncCfg->getMaxMTTHierarchyDepthI() == 0 && cs.sps->getMinQTSize( cs.slice->getSliceType(), partitioner.chType ) > maxSize );
    }

    // intra block can't be larger than MAX_INTRA_SIZE, in general such split should not be allowed in canSplit function, but there is no prediction mode info there
    if( partitioner.currArea().lumaSize().width > maxSize || partitioner.currArea().lumaSize().height > maxSize )
    {
      return false;
    }
#else
    if( m_pcEncCfg->getUseFastLCTU() && partitioner.currArea().lumaSize().area() > 4096 )
    {
      return (m_pcEncCfg->getDualITree() == 0 && m_pcEncCfg->getMaxMTTHierarchyDepthI() == 0 && cs.sps->getMinQTSize(cs.slice->getSliceType(), partitioner.chType) > 64);
    }

    if (CS::isDualITree(cs) && (partitioner.currArea().lumaSize().width > 64 || partitioner.currArea().lumaSize().height > 64))
    {
      return false;
    }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (m_pcEncCfg->getUsePbIntraFast() && (!cs.slice->isIntra() || cs.slice->getUseIBC()) && !interHadActive(cuECtx) && cuECtx.bestCU && !CU::isIntra(*cuECtx.bestCU))
#else
    if (m_pcEncCfg->getUsePbIntraFast() && (!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && !interHadActive(cuECtx) && cuECtx.bestCU && !CU::isIntra(*cuECtx.bestCU))
#endif
    {
      return false;
    }

    // INTRA MODES
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (cs.slice->getUseIBC() && !cuECtx.bestTU)
#else
    if (cs.sps->getIBCFlag() && !cuECtx.bestTU)
#endif
      return true;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( partitioner.isConsIntra() && !cuECtx.bestTU )
    {
      return true;
    }
#endif
    if ( partitioner.currArea().lumaSize().width == 4 && partitioner.currArea().lumaSize().height == 4 && !slice.isIntra() && !cuECtx.bestTU )
    {
      return true;
    }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if ( slice.getSeparateTreeEnabled() )
    {
      if ( !slice.isIntra() && m_avoidComplexIntraInInterSlice )
      {
        return false;
      }
    }
    else
    {
#endif
    if( !( slice.isIntra() || bestMode.type == ETM_INTRA || !cuECtx.bestTU ||
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    ((!m_pcEncCfg->getDisableIntraPUsInInterSlices()) && (relatedCU && (!relatedCU->isInter || !relatedCU->isIBC)) && (
#else
      ((!m_pcEncCfg->getDisableIntraPUsInInterSlices()) && (!relatedCU.isInter || !relatedCU.isIBC) && (
#endif
                                         ( cuECtx.bestTU->cbf[0] != 0 ) ||
           ( ( numComp > COMPONENT_Cb ) && cuECtx.bestTU->cbf[1] != 0 ) ||
           ( ( numComp > COMPONENT_Cr ) && cuECtx.bestTU->cbf[2] != 0 )  // avoid very complex intra if it is unlikely
         ) ) ) )
    {
      return false;
    }
    if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NOINTRA_IBCCBF0)
      && (bestMode.type == ETM_IBC || bestMode.type == ETM_IBC_MERGE)
      && (!cuECtx.bestCU->Y().valid() || cuECtx.bestTU->cbf[0] == 0)
      && (!cuECtx.bestCU->Cb().valid() || cuECtx.bestTU->cbf[1] == 0)
      && (!cuECtx.bestCU->Cr().valid() || cuECtx.bestTU->cbf[2] == 0))
    {
      return false;
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    }

    if ( slice.getSeparateTreeEnabled() )
    {
      if ( slice.getProcessingIntraRegion() && slice.isIntraRegionRoot( &partitioner ) )
      {
        cuECtx.interHad = m_savedInterHad;
      }
    }
    else
    {
#endif
    if( lastTestMode().type != ETM_INTRA && cuECtx.bestCS && cuECtx.bestCU && interHadActive( cuECtx ) )
    {
      // Get SATD threshold from best Inter-CU
      if (!cs.slice->isIntra() && m_pcEncCfg->getUsePbIntraFast() && !cs.slice->getDisableSATDForRD())
      {
        CodingUnit* bestCU = cuECtx.bestCU;
        if (bestCU && !CU::isIntra(*bestCU))
        {
          DistParam distParam;
          const bool useHad = true;
          m_pcRdCost->setDistParam( distParam, cs.getOrgBuf( COMPONENT_Y ), cuECtx.bestCS->getPredBuf( COMPONENT_Y ), cs.sps->getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, useHad );
          cuECtx.interHad = distParam.distFunc( distParam );
        }
      }
    }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (bestMode.type == ETM_PALETTE && !slice.isIntra() && partitioner.treeType == TREE_D && !(partitioner.currArea().lumaSize().width == 4 && partitioner.currArea().lumaSize().height == 4)) // inter slice
    {
      return false;
    }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if ( m_pcEncCfg->getUseFastISP() && (relatedCU && relatedCU->relatedCuIsValid) )
#else
    if ( m_pcEncCfg->getUseFastISP() && relatedCU.relatedCuIsValid )
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      cuECtx.ispPredModeVal     = relatedCU->ispPredModeVal;
      cuECtx.bestDCT2NonISPCost = relatedCU->bestDCT2NonISPCost;
      cuECtx.relatedCuIsValid   = relatedCU->relatedCuIsValid;
      cuECtx.bestNonDCT2Cost    = relatedCU->bestNonDCT2Cost;
      cuECtx.bestISPIntraMode   = relatedCU->bestISPIntraMode;
#else
      cuECtx.ispPredModeVal     = relatedCU.ispPredModeVal;
      cuECtx.bestDCT2NonISPCost = relatedCU.bestDCT2NonISPCost;
      cuECtx.relatedCuIsValid   = relatedCU.relatedCuIsValid;
      cuECtx.bestNonDCT2Cost    = relatedCU.bestNonDCT2Cost;
      cuECtx.bestISPIntraMode   = relatedCU.bestISPIntraMode;
#endif
    }

    return true;
  }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  else if ( encTestmode.type == ETM_SEPARATE_TREE_INTRA )
  {
    if( getFastDeltaQp() )
    {
      if( cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize )
      {
        return false; // only check necessary 2Nx2N Intra in fast delta-QP mode
      }
    }

    if( m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && !interHadActive( cuECtx ) && cuECtx.bestCU && CU::isInter( *cuECtx.bestCU ) )
    {
      return false;
    }

    if ( !cs.slice->isIntra() && m_pcEncCfg->getUseFastLCTU() && partitioner.currArea().lumaSize().area() > 4096 )
    {
      return false;
    }

    // INTRA MODES
    m_avoidComplexIntraInInterSlice = false;
    if( !( slice.isIntra() || bestMode.type == ETM_INTRA ||
        ( ( !m_pcEncCfg->getDisableIntraPUsInInterSlices() ) && (relatedCU && !relatedCU->isInter) && cuECtx.bestTU != nullptr && (
        ( cuECtx.bestTU->cbf[0] != 0 ) ||
        ( ( numComp > COMPONENT_Cb ) && cuECtx.bestTU->cbf[1] != 0 ) ||
        ( ( numComp > COMPONENT_Cr ) && cuECtx.bestTU->cbf[2] != 0 )  // avoid very complex intra if it is unlikely
        ) ) ) )
    {
      m_avoidComplexIntraInInterSlice = true;
    }

    if( lastTestMode().type != ETM_INTRA && lastTestMode().type != ETM_SEPARATE_TREE_INTRA && cuECtx.bestCS && cuECtx.bestCU && interHadActive( cuECtx ) )
    {
      // Get SATD threshold from best Inter-CU
      if( !cs.slice->isIntra() && m_pcEncCfg->getUsePbIntraFast() )
      {
        CodingUnit* bestCU = cuECtx.bestCU;
        if( bestCU && CU::isInter( *bestCU ) )
        {
          DistParam distParam;
          const bool useHad = true;
          m_pcRdCost->setDistParam( distParam, cs.getOrgBuf( COMPONENT_Y ), cuECtx.bestCS->getPredBuf( COMPONENT_Y ), cs.sps->getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, useHad );
          cuECtx.interHad = distParam.distFunc( distParam );
        }
      }
    }
    m_savedInterHad = cuECtx.interHad;

    if ( cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && !m_avoidComplexIntraInInterSlice )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
#endif
  else if (encTestmode.type == ETM_PALETTE)
  {
    if (partitioner.currArea().lumaSize().width > 64 || partitioner.currArea().lumaSize().height > 64
        || ((partitioner.currArea().lumaSize().width * partitioner.currArea().lumaSize().height <= 16) && (isLuma(partitioner.chType)) )
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        || ((partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) && (!isLuma(partitioner.chType)) && CS::isDualITree(cs)) )
#else
        || ((partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) && (!isLuma(partitioner.chType)) && partitioner.isSepTree(cs) ) 
        || (partitioner.isLocalSepTree(cs)  && (!isLuma(partitioner.chType)) ) )
#endif
    {
      return false;
    }
    const Area curr_cu = CS::getArea(cs, cs.area, partitioner.chType).blocks[getFirstComponentOfChannel(partitioner.chType)];
    try
    {
      double stored_cost = slice.m_mapPltCost[isChroma(partitioner.chType)].at(curr_cu.pos()).at(curr_cu.size());
      if (bestMode.type != ETM_INVALID && stored_cost > cuECtx.bestCS->cost)
      {
        return false;
      }
    }
    catch (const std::out_of_range &)
    {
      // do nothing if no stored cost value was found.
    }
    return true;
  }
  else if (encTestmode.type == ETM_IBC || encTestmode.type == ETM_IBC_MERGE)
  {
    // IBC MODES
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) && cuECtx.get<int>(BEST_INTRA_NZ_CNT) < IBC_NONSCC_ENC_RD_NZ_COUNT)
    {
      return false;
    }
    return slice.getUseIBC() && (partitioner.currArea().lumaSize().width < 128 && partitioner.currArea().lumaSize().height < 128);
#else
    return sps.getIBCFlag() && (partitioner.currArea().lumaSize().width < 128 && partitioner.currArea().lumaSize().height < 128);
#endif
  }
  else if( isModeInter( encTestmode ) )
  {
    // INTER MODES (ME + MERGE/SKIP)
    CHECK( slice.isIntra(), "Inter-mode should not be in the I-Slice mode list!" );

    if( getFastDeltaQp() )
    {
      if( encTestmode.type == ETM_MERGE_SKIP )
      {
        return false;
      }
#if TM_MRG && !MERGE_ENC_OPT
      if (encTestmode.type == ETM_MERGE_TM)
      {
        return false;
      }
#endif
      if( cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize )
      {
        return false; // only check necessary 2Nx2N Inter in fast deltaqp mode
      }
    }

    // --- Check if we can quit current mode using SAVE/LOAD coding history

#if MULTI_HYP_PRED
    if (encTestmode.type == ETM_INTER_ME
      || encTestmode.type == ETM_INTER_MULTIHYP)
#else
    if( encTestmode.type == ETM_INTER_ME )
#endif
    {
      if( encTestmode.opts == ETO_STANDARD )
      {
        // NOTE: ETO_STANDARD is always done when early SKIP mode detection is enabled
        if( !m_pcEncCfg->getUseEarlySkipDetection() )
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          if( relatedCU && (relatedCU->isSkip || relatedCU->isIntra) )
#else
          if( relatedCU.isSkip || relatedCU.isIntra )
#endif
          {
            return false;
          }
        }
      }
      else if ((encTestmode.opts & ETO_IMV) != 0)
      {
        int imvOpt = (encTestmode.opts & ETO_IMV) >> ETO_IMV_SHIFT;

        if (imvOpt == 3 && cuECtx.get<double>(BEST_NO_IMV_COST) * 1.06 < cuECtx.get<double>(BEST_IMV_COST))
        {
          if ( !m_pcEncCfg->getUseAffineAmvr() )
          return false;
        }
      }
    }
#if MULTI_HYP_PRED
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (encTestmode.type == ETM_INTER_MULTIHYP && (relatedCU && relatedCU->isInter && relatedCU->numAddHyp == 0) )
#else
    if (encTestmode.type == ETM_INTER_MULTIHYP && relatedCU.isInter && relatedCU.numAddHyp == 0)
#endif
    {
      return false;
    }
#endif
#if !MERGE_ENC_OPT
    if ( encTestmode.type == ETM_AFFINE && relatedCU.isIntra )
    {
      return false;
    }
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
    if (encTestmode.type == ETM_AF_MMVD && relatedCU.isIntra)
    {
      return false;
    }
#endif
    if( encTestmode.type == ETM_MERGE_GEO && ( partitioner.currArea().lwidth() < GEO_MIN_CU_SIZE || partitioner.currArea().lheight() < GEO_MIN_CU_SIZE
                                            || partitioner.currArea().lwidth() > GEO_MAX_CU_SIZE || partitioner.currArea().lheight() > GEO_MAX_CU_SIZE
                                            || partitioner.currArea().lwidth() >= 8 * partitioner.currArea().lheight()
                                            || partitioner.currArea().lheight() >= 8 * partitioner.currArea().lwidth() ) )
    {
      return false;
    }
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (encTestmode.type == ETM_MERGE_GEO && relatedCU && relatedCU->skipGPM)
#else
    if (encTestmode.type == ETM_MERGE_GEO && relatedCU.skipGPM)
#endif
    {
      return false;
    }
#endif
#if JVET_AD0213_LIC_IMP
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (encTestmode.type == ETM_INTER_ME && (encTestmode.opts & ETO_LIC) && relatedCU && relatedCU->skipLIC)
#else
    if (encTestmode.type == ETM_INTER_ME && (encTestmode.opts & ETO_LIC) && relatedCU.skipLIC)
#endif
    {
      return false;
    }
#endif
    return true;
  }
  else if( isModeSplit( encTestmode ) )
  {
    //////////////////////////////////////////////////////////////////////////
    // skip-history rule - don't split further if at least for three past levels
    //                     in the split tree it was found that skip is the best mode
    //////////////////////////////////////////////////////////////////////////
    int skipScore = 0;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if ((!slice.isIntra() || slice.getUseIBC()) && cuECtx.get<bool>(IS_BEST_NOSPLIT_SKIP))
#else
    if ((!slice.isIntra() || slice.getSPS()->getIBCFlag()) && cuECtx.get<bool>(IS_BEST_NOSPLIT_SKIP))
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      for( int i = 2; i < m_ComprCUCtxList[EncModeCtrl::m_treeIdx].size(); i++ )
      {
        if( ( m_ComprCUCtxList[EncModeCtrl::m_treeIdx].end() - i )->get<bool>( IS_BEST_NOSPLIT_SKIP ) )
#else
      for( int i = 2; i < m_ComprCUCtxList.size(); i++ )
      {
        if( ( m_ComprCUCtxList.end() - i )->get<bool>( IS_BEST_NOSPLIT_SKIP ) )
#endif
        {
          skipScore += 1;
        }
        else
        {
          break;
        }
      }
    }

    const PartSplit split = getPartSplit( encTestmode );
    if( !partitioner.canSplit( split, cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif    
    ) || skipScore >= 2 )
    {
      if( split == CU_HORZ_SPLIT ) cuECtx.set( DID_HORZ_SPLIT, false );
      if( split == CU_VERT_SPLIT ) cuECtx.set( DID_VERT_SPLIT, false );
      if( split == CU_QUAD_SPLIT ) cuECtx.set( DID_QUAD_SPLIT, false );

      return false;
    }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    // speedups
    if ( !cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && cuECtx.skipSplits )
    {
      return false;
    }
#endif
    if( m_pcEncCfg->getUseContentBasedFastQtbt() )
    {
      const CompArea& currArea = partitioner.currArea().Y();
      int cuHeight  = currArea.height;
      int cuWidth   = currArea.width;

      const bool condIntraInter = m_pcEncCfg->getIntraPeriod() == 1 ? ( partitioner.currBtDepth == 0 ) : ( cuHeight > 32 && cuWidth > 32 );

      if( cuWidth == cuHeight && condIntraInter && getPartSplit( encTestmode ) != CU_QUAD_SPLIT )
      {
        const CPelBuf bufCurrArea = cs.getOrgBuf( partitioner.currArea().block( COMPONENT_Y ) );

        double horVal = 0;
        double verVal = 0;
        double dupVal = 0;
        double dowVal = 0;

        const double th = m_pcEncCfg->getIntraPeriod() == 1 ? 1.2 : 1.0;

        unsigned j, k;

        for( j = 0; j < cuWidth - 1; j++ )
        {
          for( k = 0; k < cuHeight - 1; k++ )
          {
            horVal += abs( bufCurrArea.at( j + 1, k     ) - bufCurrArea.at( j, k ) );
            verVal += abs( bufCurrArea.at( j    , k + 1 ) - bufCurrArea.at( j, k ) );
            dowVal += abs( bufCurrArea.at( j + 1, k )     - bufCurrArea.at( j, k + 1 ) );
            dupVal += abs( bufCurrArea.at( j + 1, k + 1 ) - bufCurrArea.at( j, k ) );
          }
        }
        if( horVal > th * verVal && sqrt( 2 ) * horVal > th * dowVal && sqrt( 2 ) * horVal > th * dupVal && ( getPartSplit( encTestmode ) == CU_HORZ_SPLIT || getPartSplit( encTestmode ) == CU_TRIH_SPLIT ) )
        {
          return false;
        }
        if( th * dupVal < sqrt( 2 ) * verVal && th * dowVal < sqrt( 2 ) * verVal && th * horVal < verVal && ( getPartSplit( encTestmode ) == CU_VERT_SPLIT || getPartSplit( encTestmode ) == CU_TRIV_SPLIT ) )
        {
          return false;
        }
      }

      if( m_pcEncCfg->getIntraPeriod() == 1 && cuWidth <= 32 && cuHeight <= 32 && bestCS && bestCS->tus.size() == 1 && bestCU && bestCU->depth == partitioner.currDepth && partitioner.currBtDepth > 1 && isLuma( partitioner.chType ) )
      {
        if( !bestCU->rootCbf )
        {
          return false;
        }
      }
    }

    if( bestCU && bestCU->skip && bestCU->mtDepth >= m_skipThreshold && !isModeSplit( cuECtx.lastTestMode ) )
    {
      return false;
    }

    int featureToSet = -1;

    switch( getPartSplit( encTestmode ) )
    {
      case CU_QUAD_SPLIT:
        {
#if ENABLE_SPLIT_PARALLELISM
          if( !cuECtx.isLevelSplitParallel )
#endif
          if( !cuECtx.get<bool>( QT_BEFORE_BT ) && bestCU )
          {
#if JVET_AH0135_TEMPORAL_PARTITIONING
            bool canNo, canQt, canBh, canBv, canTh, canTv;
            unsigned maxBTD;
            partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv, maxBTD 
#if JVET_AI0087_BTCUS_RESTRICTION
              , false, false
#endif
            );
#else 
            unsigned maxBTD        = cs.pcv->getMaxBtDepth( slice, partitioner.chType );
#endif
            const CodingUnit *cuBR = bestCS->cus.back();
            unsigned height        = partitioner.currArea().lumaSize().height;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (bestCU && ((bestCU->btDepth == 0 && maxBTD >= ((slice.isIntra() && !slice.getUseIBC()) ? 3 : 2))
              || (bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 && maxBTD >= ((slice.isIntra() && !slice.getUseIBC()) ? 4 : 3)))
#else
            if (bestCU && ((bestCU->btDepth == 0 && maxBTD >= ((slice.isIntra() && !slice.getSPS()->getIBCFlag()) ? 3 : 2))
              || (bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 && maxBTD >= ((slice.isIntra() && !slice.getSPS()->getIBCFlag()) ? 4 : 3)))
#endif
              && (width <= MAX_TB_SIZEY && height <= MAX_TB_SIZEY)
              && cuECtx.get<bool>(DID_HORZ_SPLIT) && cuECtx.get<bool>(DID_VERT_SPLIT))
            {
              return false;
            }
          }
          if( m_pcEncCfg->getUseEarlyCU() && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip )
          {
            return false;
          }
          if( getFastDeltaQp() && width <= slice.getPPS()->pcv->fastDeltaQPCuMaxSize )
          {
            return false;
          }
        }
        break;
      case CU_HORZ_SPLIT:
        featureToSet = DID_HORZ_SPLIT;
        break;
      case CU_VERT_SPLIT:
        featureToSet = DID_VERT_SPLIT;
        break;
      case CU_TRIH_SPLIT:
        if( cuECtx.get<bool>( DID_HORZ_SPLIT ) && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
        {
          return false;
        }

#if JVET_AL0181_ASBT
        if ( bestCU && bestCU->rootCbf && ( (slice.getPic()->temporalId >= 5) && (bestCU->mtDepth < 1) ) )
        {
          if ( bestCU->firstTU->asbtDecimH[COMPONENT_Y] )
          {
            return false;
          }
        }
#endif
        if( !cuECtx.get<bool>( DO_TRIH_SPLIT ) )
        {
          return false;
        }
        break;
      case CU_TRIV_SPLIT:
        if( cuECtx.get<bool>( DID_VERT_SPLIT ) && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
        {
          return false;
        }

#if JVET_AL0181_ASBT
        if ( bestCU && bestCU->rootCbf && ( (slice.getPic()->temporalId >= 5) && (bestCU->mtDepth < 1) ) )
        {
          if ( bestCU->firstTU->asbtDecimW[COMPONENT_Y] )
          {
            return false;
          }
        }
#endif
        if( !cuECtx.get<bool>( DO_TRIV_SPLIT ) )
        {
          return false;
        }
        break;
      default:
        THROW( "Only CU split modes are governed by the EncModeCtrl" );
        return false;
        break;
    }

    switch( split )
    {
      case CU_HORZ_SPLIT:
      case CU_TRIH_SPLIT:
        if( cuECtx.get<bool>( QT_BEFORE_BT ) && cuECtx.get<bool>( DID_QUAD_SPLIT ) )
        {
          if( cuECtx.get<int>( MAX_QT_SUB_DEPTH ) > partitioner.currQtDepth + 1 )
          {
            if( featureToSet >= 0 ) cuECtx.set( featureToSet, false );
            return false;
          }
        }
#if JVET_Y0152_TT_ENC_SPEEDUP
        if (m_pcEncCfg->getFastTTskip() && split == CU_TRIH_SPLIT)
        {
          bool skipTtSplitMode = xSkipTreeCandidate(getPartSplit(encTestmode), cs.splitRdCostBest, m_slice->getSliceType());
          if (skipTtSplitMode) 
          {
            return false;
          }
        }
#endif
        break;
      case CU_VERT_SPLIT:
      case CU_TRIV_SPLIT:
        if( cuECtx.get<bool>( QT_BEFORE_BT ) && cuECtx.get<bool>( DID_QUAD_SPLIT ) )
        {
          if( cuECtx.get<int>( MAX_QT_SUB_DEPTH ) > partitioner.currQtDepth + 1 )
          {
            if( featureToSet >= 0 ) cuECtx.set( featureToSet, false );
            return false;
          }
        }
#if JVET_Y0152_TT_ENC_SPEEDUP
        if (m_pcEncCfg->getFastTTskip() && split == CU_TRIV_SPLIT)
        {
          bool skipTtSplitMode = xSkipTreeCandidate(getPartSplit(encTestmode), cs.splitRdCostBest, m_slice->getSliceType());
          if (skipTtSplitMode)
          {
            return false;
          }
        }
#endif
        break;
      default:
        break;
    }

    if( split == CU_QUAD_SPLIT ) cuECtx.set( DID_QUAD_SPLIT, true );
    if (cs.sps->getLog2ParallelMergeLevelMinus2())
    {
      const CompArea& area = partitioner.currArea().Y();
      const SizeType size = 1 << (cs.sps->getLog2ParallelMergeLevelMinus2() + 2);
      if (!cs.slice->isIntra() && (area.width > size || area.height > size))
      {
        if (area.height <= size && split == CU_HORZ_SPLIT) return false;
        if (area.width <= size && split == CU_VERT_SPLIT) return false;
        if (area.height <= 2 * size && split == CU_TRIH_SPLIT) return false;
        if (area.width <= 2 * size && split == CU_TRIV_SPLIT) return false;
      }
    }
    return true;
  }
  else
  {
    CHECK( encTestmode.type != ETM_POST_DONT_SPLIT, "Unknown mode" );
    if ((cuECtx.get<double>(BEST_NO_IMV_COST) == (MAX_DOUBLE * .5) || cuECtx.get<bool>(IS_REUSING_CU)) && !slice.isIntra())
    {
      unsigned idx1, idx2, idx3, idx4;
      getAreaIdx(partitioner.currArea().Y(), *slice.getPPS()->pcv, idx1, idx2, idx3, idx4);
      if (g_isReusedUniMVsFilled[idx1][idx2][idx3][idx4])
      {
        m_pcInterSearch->insertUniMvCands(partitioner.currArea().Y(), g_reusedUniMVs[idx1][idx2][idx3][idx4]);
      }
#if INTER_LIC
      if (cs.slice->getUseLIC() && g_isReusedUniMVsFilledLIC[idx1][idx2][idx3][idx4])
      {
        m_pcInterSearch->swapUniMvBuffer();
        m_pcInterSearch->insertUniMvCands(partitioner.currArea().Y(), g_reusedUniMVsLIC[idx1][idx2][idx3][idx4]);
        m_pcInterSearch->swapUniMvBuffer();
      }
#endif
    }
    if( !bestCS || ( bestCS && isModeSplit( bestMode ) ) )
    {
      return false;
    }
    else
    {
#if REUSE_CU_RESULTS
      setFromCs( *bestCS, partitioner );

#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( partitioner.modeType == MODE_TYPE_INTRA && partitioner.chType == CHANNEL_TYPE_LUMA )
      {
        return false; //not set best coding mode for intra coding pass
      }
#endif
      // assume the non-split modes are done and set the marks for the best found mode
      if( bestCS && bestCU )
      {
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if (!slice.isIntra() && relatedCU)
        {
          double gpmCost = cuECtx.get<double>(BEST_GPM_COST);
          if (gpmCost != (MAX_DOUBLE * .5))
          {
            if (gpmCost > (bestCS->cost * 1.2))
            {
              relatedCU->skipGPM = true;
            }
            else if (gpmCost > bestCS->cost && bestCU->skip)
            {
              relatedCU->skipGPM = true;
            }
            else if (gpmCost > bestCS->cost && (!cuECtx.bestTU->cbf[0] && !cuECtx.bestTU->cbf[1] && !cuECtx.bestTU->cbf[2]))
            {
              relatedCU->skipGPM = true;
            }
          }
        }
#else
        if (!slice.isIntra())
        {
          double gpmCost = cuECtx.get<double>(BEST_GPM_COST);
          if (gpmCost != (MAX_DOUBLE * .5))
          {
            if (gpmCost > (bestCS->cost * 1.2))
            {
              relatedCU.skipGPM = true;
            }
            else if (gpmCost > bestCS->cost && bestCU->skip)
            {
              relatedCU.skipGPM = true;
            }
            else if (gpmCost > bestCS->cost && (!cuECtx.bestTU->cbf[0] && !cuECtx.bestTU->cbf[1] && !cuECtx.bestTU->cbf[2]))
            {
              relatedCU.skipGPM = true;
            }
          }
        }
#endif
#endif
#if JVET_AD0213_LIC_IMP
        int fastLic = m_pcEncCfg->getFastLic();
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if(relatedCU)
#endif
        if (fastLic)
        {
          double licCost = cuECtx.get<double>(BEST_LIC_COST);
          int c1 = (fastLic & 0x03);
          int c2 = (fastLic & 0x04);
          double r = ((c1 == 0x01) ? 1.0 : ((c1 == 0x02) ? 1.1 : 1.2));
          if (licCost != (MAX_DOUBLE * .5))
          {
            if (licCost > (bestCS->cost * r) && bestCU->skip)
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              relatedCU->skipLIC = true;
#else
              relatedCU.skipLIC = true;
#endif
            }
            else if (c2 && licCost > (bestCS->cost * 1.1))
            {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
              relatedCU->skipLIC = true;
#else
              relatedCU.skipLIC = true;
#endif
            }
          }
        }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if (relatedCU && CU::isInter(*bestCU))
#else
        if( CU::isInter( *bestCU ) )
#endif
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          relatedCU->isInter   = true;
          relatedCU->isSkip   |= bestCU->skip;
          relatedCU->isMMVDSkip |= bestCU->mmvdSkip;
          relatedCU->bcwIdx    = bestCU->bcwIdx;
#if MULTI_HYP_PRED
          relatedCU->numAddHyp = (uint8_t)bestCU->firstPU->addHypData.size();
#endif
          if (bestCU->slice->getSPS()->getUseColorTrans())
          {
            if (m_pcEncCfg->getRGBFormatFlag())
            {
              if (bestCU->colorTransform && bestCU->rootCbf)
              {
                relatedCU->selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU->selectColorSpaceOption = 2;
              }
            }
            else
            {
              if (!bestCU->colorTransform || !bestCU->rootCbf)
              {
                relatedCU->selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU->selectColorSpaceOption = 2;
              }
            }
          }
#else
          relatedCU.isInter   = true;
          relatedCU.isSkip   |= bestCU->skip;
          relatedCU.isMMVDSkip |= bestCU->mmvdSkip;
          relatedCU.bcwIdx    = bestCU->bcwIdx;
#if MULTI_HYP_PRED
          relatedCU.numAddHyp = (uint8_t)bestCU->firstPU->addHypData.size();
#endif
          if (bestCU->slice->getSPS()->getUseColorTrans())
          {
            if (m_pcEncCfg->getRGBFormatFlag())
            {
              if (bestCU->colorTransform && bestCU->rootCbf)
              {
                relatedCU.selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU.selectColorSpaceOption = 2;
              }
            }
            else
            {
              if (!bestCU->colorTransform || !bestCU->rootCbf)
              {
                relatedCU.selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU.selectColorSpaceOption = 2;
              }
            }
          }
#endif
        }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        else if (relatedCU && CU::isIBC(*bestCU))
#else
        else if (CU::isIBC(*bestCU))
#endif
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
#if JVET_AA0070_RRIBC
          relatedCU->isRribcCoded = bestCU->rribcFlipType > 0;
#endif
          relatedCU->isIBC = true;
          relatedCU->isSkip |= bestCU->skip;
          if (bestCU->slice->getSPS()->getUseColorTrans())
          {
            if (m_pcEncCfg->getRGBFormatFlag())
            {
              if (bestCU->colorTransform && bestCU->rootCbf)
              {
                relatedCU->selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU->selectColorSpaceOption = 2;
              }
            }
            else
            {
              if (!bestCU->colorTransform || !bestCU->rootCbf)
              {
                relatedCU->selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU->selectColorSpaceOption = 2;
              }
            }
          }
#else
#if JVET_AA0070_RRIBC
          relatedCU.isRribcCoded = bestCU->rribcFlipType > 0;
#endif
          relatedCU.isIBC = true;
          relatedCU.isSkip |= bestCU->skip;
          if (bestCU->slice->getSPS()->getUseColorTrans())
          {
            if (m_pcEncCfg->getRGBFormatFlag())
            {
              if (bestCU->colorTransform && bestCU->rootCbf)
              {
                relatedCU.selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU.selectColorSpaceOption = 2;
              }
            }
            else
            {
              if (!bestCU->colorTransform || !bestCU->rootCbf)
              {
                relatedCU.selectColorSpaceOption = 1;
              }
              else
              {
                relatedCU.selectColorSpaceOption = 2;
              }
            }
          }
#endif
        }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        else if( relatedCU && CU::isIntra( *bestCU ) )
#else
        else if( CU::isIntra( *bestCU ) )
#endif
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          relatedCU->isIntra   = true;
          if ( m_pcEncCfg->getUseFastISP() && cuECtx.ispWasTested && ( !relatedCU->relatedCuIsValid || bestCS->cost < relatedCU->bestCost ) )
#else
          relatedCU.isIntra   = true;
          if ( m_pcEncCfg->getUseFastISP() && cuECtx.ispWasTested && ( !relatedCU.relatedCuIsValid || bestCS->cost < relatedCU.bestCost ) )
#endif
          {
            // Compact data
            int bit0 = true;
            int bit1 = cuECtx.ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0;
            int bit2 = cuECtx.ispMode == VER_INTRA_SUBPARTITIONS;
            int bit3 = cuECtx.ispLfnstIdx > 0;
            int bit4 = cuECtx.ispLfnstIdx == 2;
            int bit5 = cuECtx.mipFlag;
            int bit6 = cuECtx.bestCostIsp < cuECtx.bestNonDCT2Cost * 0.95;
#if JVET_V0130_INTRA_TMP
            int bit7 = cuECtx.tmpFlag;
#endif
#if JVET_AG0058_EIP
            int bit8 = cuECtx.eipFlag;
#endif
            int val =
              (bit0) |
              (bit1 << 1) |
              (bit2 << 2) |
              (bit3 << 3) |
              (bit4 << 4) |
              (bit5 << 5) |
              (bit6 << 6) |
#if JVET_V0130_INTRA_TMP
              (bit7 << 7) |
#endif
#if JVET_AG0058_EIP
              (bit8 << 8) |
#endif
              ( cuECtx.bestPredModeDCT2 << 9 );
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
            relatedCU->ispPredModeVal     = val;
            relatedCU->bestDCT2NonISPCost = cuECtx.bestDCT2NonISPCost;
            relatedCU->bestCost           = bestCS->cost;
            relatedCU->bestNonDCT2Cost    = cuECtx.bestNonDCT2Cost;
            relatedCU->bestISPIntraMode   = cuECtx.bestISPIntraMode;
            relatedCU->relatedCuIsValid   = true;
#else
            relatedCU.ispPredModeVal     = val;
            relatedCU.bestDCT2NonISPCost = cuECtx.bestDCT2NonISPCost;
            relatedCU.bestCost           = bestCS->cost;
            relatedCU.bestNonDCT2Cost    = cuECtx.bestNonDCT2Cost;
            relatedCU.bestISPIntraMode   = cuECtx.bestISPIntraMode;
            relatedCU.relatedCuIsValid   = true;
#endif
          }
#if INTRA_TRANS_ENC_OPT
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          if ((m_pcEncCfg->getIntraPeriod() == 1) && isLuma(partitioner.chType) && cuECtx.isLfnstTested() && (!relatedCU->relatedCuLfnstIsValid || bestCS->cost < relatedCU->bestCostForLfnst))
          {
            relatedCU->bestCostForLfnst = bestCS->cost;
            relatedCU->relatedCuLfnstIsValid = true;

            if (cuECtx.bestLfnstCost[1] > (cuECtx.bestLfnstCost[0] * 1.4))
            {
              relatedCU->skipLfnstTest = true;
            }
#if JVET_AK0217_INTRA_MTSS
            else if (partitioner.currArea().lwidth() * partitioner.currArea().lheight() >= MIN_MTSS_SIZE && cuECtx.bestLfnstCost[1] > (cuECtx.bestLfnstCost[0] * 1.09))
            {
              relatedCU->skipLfnstTest = true;
            }
#endif 
            else
            {
              relatedCU->skipLfnstTest = false;
            }
          }
#else
          if ((m_pcEncCfg->getIntraPeriod() == 1) && isLuma(partitioner.chType) && cuECtx.isLfnstTested() && (!relatedCU.relatedCuLfnstIsValid || bestCS->cost < relatedCU.bestCostForLfnst))
          {
            relatedCU.bestCostForLfnst = bestCS->cost;
            relatedCU.relatedCuLfnstIsValid = true;

            if (cuECtx.bestLfnstCost[1] > (cuECtx.bestLfnstCost[0] * 1.4))
            {
              relatedCU.skipLfnstTest = true;
            }
            else
            {
              relatedCU.skipLfnstTest = false;
            }
          }
#endif
#endif
#if JVET_AB0092_GLM_WITH_LUMA
          if (!isLuma(partitioner.chType) && !(bestCU->firstPU->glmIdc.isActive()))
          {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
            relatedCU->skipGLM = true;
#else
            relatedCU.skipGLM = true;
#endif
          }
#endif
        }
#if ENABLE_SPLIT_PARALLELISM
#if REUSE_CU_RESULTS
        BestEncInfoCache::touch(partitioner.currArea());
#endif
        CacheBlkInfoCtrl::touch(partitioner.currArea());
#endif
        cuECtx.set( IS_BEST_NOSPLIT_SKIP, bestCU->skip );
#if JVET_AG0276_NLIC
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if (!cs.slice->getProcessingIntraRegion())
        {
#endif
          if (cuECtx.bestCU->altLMFlag)
          {
            cuECtx.bestCU->secAltLMParaUnit = cuECtx.bestCU->altLMParaUnit;
          }
          else
          {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            cuECtx.bestCU->secAltLMParaUnit.resetAltLinearModel();
#else
            cuECtx.bestCU->secAltLMParaUnit = cuECtx.bestCU->altLMParaUnit;
#endif
          }
          cuECtx.bestCU->altLMParaUnit.resetAltLinearModel();
#if JVET_AG0276_LIC_FLAG_SIGNALING
          cuECtx.bestCU->altLMBRParaUnit.resetAltLinearModel();
#endif
          if (CU::isSecLicParaNeeded(*bestCU))
          {
            UnitArea   localUnitArea(bestCU->chromaFormat, Area(0, 0, bestCU->lumaSize().width, bestCU->lumaSize().height));
            PelUnitBuf predBeforeMCAdjBuffer = m_pcInterSearch->m_acPredBeforeLICBuffer[REF_PIC_LIST_0].getBuf(localUnitArea);

            bool isPredSampleRefined = CU::isPredRefined(*bestCU);
            if (!isPredSampleRefined)
            {
              bool obmcApplied = true;
              if (bestCU->cs->sps->getUseOBMC() == false || bestCU->obmcFlag == false || bestCU->lwidth() * bestCU->lheight() < 32)
              {
                obmcApplied = false;
              }
              if (obmcApplied && bestCU->firstPU->mergeFlag)
              {
                if (m_pcInterSearch->isSCC(*bestCU->firstPU))
                {
                  obmcApplied = false;
                }
              }
              if (obmcApplied)
              {
                isPredSampleRefined = true;
              }
            }
            if (CU::isAllowSecLicPara(*bestCU))
            {
              if (isPredSampleRefined)
              {
#if INTER_LIC
                bool orgLicFlag = bestCU->licFlag;
                bestCU->licFlag = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                bool orgLicInheritedFlag = bestCU->licInheritPara;
                bestCU->licInheritPara = false;
#endif
#endif
                bool orgCiipFlag = bestCU->firstPU->ciipFlag;
                bestCU->firstPU->ciipFlag = false;
                m_pcInterSearch->xPredWoRefinement(*bestCU->firstPU, predBeforeMCAdjBuffer);
#if INTER_LIC
                bestCU->licFlag = orgLicFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                bestCU->licInheritPara = orgLicInheritedFlag;
#endif
#endif
                bestCU->firstPU->ciipFlag = orgCiipFlag;
              }
              else
              {
#if NN_LF_TRYMODE_FIX
                m_pcInterSearch->motionCompensation(*cuECtx.bestCU, REF_PIC_LIST_X, true, true, &predBeforeMCAdjBuffer);
#else
                m_pcInterSearch->motionCompensation(*cuECtx.bestCU);
                predBeforeMCAdjBuffer.copyFrom(cuECtx.bestCU->cs->getPredBuf(*cuECtx.bestCU->firstPU));
#endif
#if NN_LF_CCCM_CCPMERGE_ALT_FIX
                for (auto& currTU : CU::traverseTUs(*cuECtx.bestCU))
                {
                  if (currTU.interCccm || currTU.interCcpMerge || cuECtx.bestCU->interCcpMergeZeroRootCbfIdc)
                  {
                    cuECtx.bestCU->cs->getPredBuf(currTU.blocks[COMPONENT_Cb]).copyFrom(cuECtx.bestCU->cs->getPredBufCustom(currTU.blocks[COMPONENT_Cb]));
                    cuECtx.bestCU->cs->getPredBuf(currTU.blocks[COMPONENT_Cr]).copyFrom(cuECtx.bestCU->cs->getPredBufCustom(currTU.blocks[COMPONENT_Cr]));
                  }
                }
#endif
              }
            }
            else
            {
              if (cuECtx.bestCU->affine && cuECtx.bestCU->firstPU->mergeFlag && cuECtx.bestCU->cs->sps->getUseOBMC() && cuECtx.bestCU->obmcFlag && (cuECtx.bestCU->cs->sps->getUseAltLM() || cuECtx.bestCU->cs->sps->getUseAffAltLM()))
              {
                if (cuECtx.bestCU->obmcFlag && m_pcInterSearch->isSCC(*cuECtx.bestCU->firstPU) && !CU::isTLCond(*cuECtx.bestCU))
                {
                  cuECtx.bestCU->obmcFlag = false;
                }
              }
            }
            if (CU::isAllowSecLicPara(*bestCU))
            {
              PelUnitBuf recBuf = m_pcInterSearch->m_acPredBeforeLICBuffer[REF_PIC_LIST_1].getBuf(localUnitArea);
              recBuf.copyFrom(cuECtx.bestCU->cs->getRecoBuf(*cuECtx.bestCU->firstPU));
              if (bestCS->slice->getLmcsEnabledFlag() && m_pcInterSearch->m_pcReshape->getCTUFlag())
              {
                recBuf.Y().rspSignal(m_pcInterSearch->m_pcReshape->getInvLUT());
              }
#if JVET_AG0276_LIC_FLAG_SIGNALING
              m_pcInterSearch->xDevSecLicPara<false>(*cuECtx.bestCU, predBeforeMCAdjBuffer, recBuf);
#else
              m_pcInterSearch->xDevSecLicPara(*cuECtx.bestCU, predBeforeMCAdjBuffer, recBuf);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
              m_pcInterSearch->xDevSecLicPara<true>(*cuECtx.bestCU, predBeforeMCAdjBuffer, recBuf);
              if ((cuECtx.bestCU->altLMBRParaUnit.scale[COMPONENT_Y] == cuECtx.bestCU->altLMParaUnit.scale[COMPONENT_Y]) && (cuECtx.bestCU->altLMBRParaUnit.offset[COMPONENT_Y] == cuECtx.bestCU->altLMParaUnit.offset[COMPONENT_Y]))
              {
                cuECtx.bestCU->altLMBRParaUnit.resetAltLinearModel();
              }
#endif
            }
          }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE 
        }
#endif

#endif
      }
    }

    return false;
  }
}

#if JVET_Y0152_TT_ENC_SPEEDUP
bool EncModeCtrlMTnoRQT::xSkipTreeCandidate(const PartSplit split, const double* splitRdCostBest, const SliceType& sliceType) const
{
  if (!splitRdCostBest)
  {
    return false;
  }
  double ttEncSpeedupRate = m_pcEncCfg->getFastTTskipThr();
  double horXorVerRate = m_pcEncCfg->getFastTTskipThr();

  if (!(m_pcEncCfg->getFastTTskip() & FAST_METHOD_TT_ENC_SPEEDUP_ISLICE))
  {
    if (sliceType == I_SLICE)
    {
      return false;
    }
  }

  if (!(m_pcEncCfg->getFastTTskip() & FAST_METHOD_TT_ENC_SPEEDUP_BSLICE))
  {
    if (sliceType == B_SLICE)
    {
      return false;
    }
  }
  bool res = false;

  if (split == CU_TRIH_SPLIT)
  {
    if (m_pcEncCfg->getFastTTskip() & FAST_METHOD_ENC_SPEEDUP_BT_BASED)
    {
      if (splitRdCostBest[CTU_LEVEL] < MAX_DOUBLE && splitRdCostBest[CU_HORZ_SPLIT] < MAX_DOUBLE)
      {
        if (splitRdCostBest[CU_HORZ_SPLIT] > ttEncSpeedupRate * splitRdCostBest[CTU_LEVEL])
        {
          res = true;
        }
      }
    }

    if (m_pcEncCfg->getFastTTskip() & FAST_METHOD_HOR_XOR_VER)
    {
      if (splitRdCostBest[CU_HORZ_SPLIT] < MAX_DOUBLE && splitRdCostBest[CU_VERT_SPLIT] < MAX_DOUBLE)
      {
        if (splitRdCostBest[CU_HORZ_SPLIT] > horXorVerRate * splitRdCostBest[CU_VERT_SPLIT])
        {
          res = true;
        }
      }
    }
  }
  if (split == CU_TRIV_SPLIT)
  {
    if (m_pcEncCfg->getFastTTskip() & FAST_METHOD_ENC_SPEEDUP_BT_BASED)
    {
      if (splitRdCostBest[CTU_LEVEL] < MAX_DOUBLE && splitRdCostBest[CU_VERT_SPLIT] < MAX_DOUBLE)
      {
        if (splitRdCostBest[CU_VERT_SPLIT] > ttEncSpeedupRate * splitRdCostBest[CTU_LEVEL])
        {
          res = true;
        }
      }
    }

    if (m_pcEncCfg->getFastTTskip() & FAST_METHOD_HOR_XOR_VER)
    {
      if (splitRdCostBest[CU_HORZ_SPLIT] < MAX_DOUBLE && splitRdCostBest[CU_VERT_SPLIT] < MAX_DOUBLE)
      {
        if (splitRdCostBest[CU_VERT_SPLIT] > horXorVerRate * splitRdCostBest[CU_HORZ_SPLIT])
        {
          res = true;
        }
      }
    }
  }
  return res;
}
#endif

bool EncModeCtrlMTnoRQT::checkSkipOtherLfnst( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner )
{
  xExtractFeatures( encTestmode, *tempCS );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  ComprCUCtx& cuECtx  = m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back();
#else
  ComprCUCtx& cuECtx  = m_ComprCUCtxList.back();
#endif
  bool skipOtherLfnst = false;

  if( encTestmode.type == ETM_INTRA )
  {
    if( !cuECtx.bestCS || ( tempCS->cost >= cuECtx.bestCS->cost && cuECtx.bestCS->cus.size() == 1 && CU::isIntra( *cuECtx.bestCS->cus[ 0 ] ) )
      || ( tempCS->cost <  cuECtx.bestCS->cost && CU::isIntra( *tempCS->cus[ 0 ] ) ) )
    {
      skipOtherLfnst = !tempCS->cus[ 0 ]->rootCbf;
    }
#if JVET_AM0074_INTRA_MERGE
  if (cuECtx.bestCS && !skipOtherLfnst)
  {
    bool skipLfsnt23 = (tempCS->cus[0]->lfnstIdx > 1) && (tempCS->cost > 1.5 * cuECtx.bestCS->cost);
    bool skipLfsnt123 = (tempCS->cus[0]->lfnstIdx > 0) && (tempCS->cost > 1.7 * cuECtx.bestCS->cost);
    if(skipLfsnt23 || skipLfsnt123)
    {
      skipOtherLfnst = true;
    }
  }
#endif
  }

  return skipOtherLfnst;
}

bool EncModeCtrlMTnoRQT::useModeResult( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner )
{
  xExtractFeatures( encTestmode, *tempCS );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  ComprCUCtx& cuECtx = m_ComprCUCtxList[EncModeCtrl::m_treeIdx].back();
#else
  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();
#endif

  if(      encTestmode.type == ETM_SPLIT_BT_H )
  {
    cuECtx.set( BEST_HORZ_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    cuECtx.set( BEST_VERT_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_TT_H )
  {
    cuECtx.set( BEST_TRIH_SPLIT_COST, tempCS->cost );
  }
  else if( encTestmode.type == ETM_SPLIT_TT_V )
  {
    cuECtx.set( BEST_TRIV_SPLIT_COST, tempCS->cost );
  }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  else if( ( encTestmode.type == ETM_INTRA || ( encTestmode.type == ETM_SEPARATE_TREE_INTRA && EncModeCtrl::m_treeIdx==0 ) )  )
#else
  else if( encTestmode.type == ETM_INTRA )
#endif
  {
    const CodingUnit& cu = *tempCS->getCU( partitioner.chType );

    if( !cu.mtsFlag )
    {
      cuECtx.bestMtsSize2Nx2N1stPass   = tempCS->cost;
    }
    if( !cu.ispMode )
    {
      cuECtx.bestCostMtsFirstPassNoIsp = tempCS->cost;
    }
#if INTRA_TRANS_ENC_OPT
    if (cu.lfnstIdx)
    {
      if (tempCS->cost < cuECtx.bestLfnstCost[1])
      {
        cuECtx.bestLfnstCost[1] = tempCS->cost;
      }
    }
    else
    {
      if (tempCS->cost < cuECtx.bestLfnstCost[0])
      {
        cuECtx.bestLfnstCost[0] = tempCS->cost;
      }
    }
#endif
  }

  if( m_pcEncCfg->getIMV4PelFast() && m_pcEncCfg->getIMV() && encTestmode.type == ETM_INTER_ME )
  {
    int imvMode = ( encTestmode.opts & ETO_IMV ) >> ETO_IMV_SHIFT;

    if( imvMode == 1 )
    {
      if( tempCS->cost < cuECtx.get<double>( BEST_IMV_COST ) )
      {
        cuECtx.set( BEST_IMV_COST, tempCS->cost );
      }
    }
    else if( imvMode == 0 )
    {
      if( tempCS->cost < cuECtx.get<double>( BEST_NO_IMV_COST ) )
      {
        cuECtx.set( BEST_NO_IMV_COST, tempCS->cost );
      }
    }
  }
#if JVET_W0097_GPM_MMVD_TM
  if (encTestmode.type == ETM_MERGE_GEO)
  {
    if (tempCS->cost < cuECtx.get<double>(BEST_GPM_COST))
    {
      cuECtx.set(BEST_GPM_COST, tempCS->cost);
    }
  }
#endif
#if JVET_AD0213_LIC_IMP
  if (encTestmode.type == ETM_INTER_ME && (encTestmode.opts & ETO_LIC))
  {
    if (tempCS->cost < cuECtx.get<double>(BEST_LIC_COST))
    {
      cuECtx.set(BEST_LIC_COST, tempCS->cost);
    }
  }
#endif
  if( encTestmode.type == ETM_SPLIT_QT )
  {
    int maxQtD = 0;
    for( const auto& cu : tempCS->cus )
    {
      maxQtD = std::max<int>( maxQtD, cu->qtDepth );
    }
    cuECtx.set( MAX_QT_SUB_DEPTH, maxQtD );
  }

  int maxMtD = tempCS->pcv->getMaxBtDepth( *tempCS->slice, partitioner.chType ) + partitioner.currImplicitBtDepth;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  unsigned maxBTD;
  partitioner.canSplit( *tempCS, canNo, canQt, canBh, canBv, canTh, canTv, maxBTD
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  );
  maxMtD = (int)maxBTD;
#endif

  if( encTestmode.type == ETM_SPLIT_BT_H )
  {
    if( tempCS->cus.size() > 2 )
    {
      int h_2   = tempCS->area.blocks[partitioner.chType].height / 2;
      int cu1_h = tempCS->cus.front()->blocks[partitioner.chType].height;
      int cu2_h = tempCS->cus.back() ->blocks[partitioner.chType].height;

      cuECtx.set( DO_TRIH_SPLIT, cu1_h < h_2 || cu2_h < h_2 || partitioner.currMtDepth + 1 == maxMtD );
    }
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    if( tempCS->cus.size() > 2 )
    {
      int w_2   = tempCS->area.blocks[partitioner.chType].width / 2;
      int cu1_w = tempCS->cus.front()->blocks[partitioner.chType].width;
      int cu2_w = tempCS->cus.back() ->blocks[partitioner.chType].width;

      cuECtx.set( DO_TRIV_SPLIT, cu1_w < w_2 || cu2_w < w_2 || partitioner.currMtDepth + 1 == maxMtD );
    }
  }

  // for now just a simple decision based on RD-cost or choose tempCS if bestCS is not yet coded
  if( tempCS->features[ENC_FT_RD_COST] != MAX_DOUBLE && ( !cuECtx.bestCS || ( ( tempCS->features[ENC_FT_RD_COST] + ( tempCS->useDbCost ? tempCS->costDbOffset : 0 ) ) < ( cuECtx.bestCS->features[ENC_FT_RD_COST] + ( tempCS->useDbCost ? cuECtx.bestCS->costDbOffset : 0 ) ) ) ) )
  {
    cuECtx.bestCS = tempCS;
    cuECtx.bestCU = tempCS->cus[0];
    cuECtx.bestTU = cuECtx.bestCU->firstTU;

    if( isModeInter( encTestmode ) )
    {
      //Here we take the best cost of both inter modes. We are assuming only the inter modes (and all of them) have come before the intra modes!!!
      cuECtx.bestInterCost = cuECtx.bestCS->cost;
    }

    return true;
  }
  else
  {
    return false;
  }
}

#if ENABLE_SPLIT_PARALLELISM
void EncModeCtrlMTnoRQT::copyState( const EncModeCtrl& other, const UnitArea& area )
{
  const EncModeCtrlMTnoRQT* pOther = dynamic_cast<const EncModeCtrlMTnoRQT*>( &other );

  CHECK( !pOther, "Trying to copy state from a different type of controller" );

  this->EncModeCtrl        ::copyState( *pOther, area );
  this->CacheBlkInfoCtrl   ::copyState( *pOther, area );
#if REUSE_CU_RESULTS
  this->BestEncInfoCache   ::copyState( *pOther, area );
#endif
  this->SaveLoadEncInfoSbt ::copyState( *pOther );

  m_skipThreshold = pOther->m_skipThreshold;
}

int EncModeCtrlMTnoRQT::getNumParallelJobs( const CodingStructure &cs, Partitioner& partitioner ) const
{
  int numJobs = 0;

  if(      partitioner.canSplit( CU_TRIH_SPLIT, cs ) )
  {
    numJobs = 6;
  }
  else if( partitioner.canSplit( CU_TRIV_SPLIT, cs ) )
  {
    numJobs = 5;
  }
  else if( partitioner.canSplit( CU_HORZ_SPLIT, cs ) )
  {
    numJobs = 4;
  }
  else if( partitioner.canSplit( CU_VERT_SPLIT, cs ) )
  {
    numJobs = 3;
  }
  else if( partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    numJobs = 2;
  }
  else if( partitioner.canSplit( CU_DONT_SPLIT, cs ) )
  {
    numJobs = 1;
  }

  CHECK( numJobs >= NUM_RESERVERD_SPLIT_JOBS, "More jobs specified than allowed" );

  return numJobs;
}

bool EncModeCtrlMTnoRQT::isParallelSplit( const CodingStructure &cs, Partitioner& partitioner ) const
{
  if( partitioner.getImplicitSplit( cs ) != CU_DONT_SPLIT || cs.picture->scheduler.getSplitJobId() != 0 ) return false;
  if( cs.pps->getUseDQP() && partitioner.currQgEnable() ) return false;
  const int numJobs = getNumParallelJobs( cs, partitioner );
  const int numPxl  = partitioner.currArea().Y().area();
  const int parlAt  = m_pcEncCfg->getNumSplitThreads() <= 3 ? 1024 : 256;
  if(  cs.slice->isIntra() && numJobs > 2 && ( numPxl == parlAt || !partitioner.canSplit( CU_QUAD_SPLIT, cs ) ) ) return true;
  if( !cs.slice->isIntra() && numJobs > 1 && ( numPxl == parlAt || !partitioner.canSplit( CU_QUAD_SPLIT, cs ) ) ) return true;
  return false;
}

bool EncModeCtrlMTnoRQT::parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const
{
  // Job descriptors
  //  - 1: all non-split modes
  //  - 2: QT-split
  //  - 3: all vertical modes but TT_V
  //  - 4: all horizontal modes but TT_H
  //  - 5: TT_V
  //  - 6: TT_H
  switch( cs.picture->scheduler.getSplitJobId() )
  {
  case 1:
    // be sure to execute post dont split
    return !isModeSplit( encTestmode );
    break;
  case 2:
    return encTestmode.type == ETM_SPLIT_QT;
    break;
  case 3:
    return encTestmode.type == ETM_SPLIT_BT_V;
    break;
  case 4:
    return encTestmode.type == ETM_SPLIT_BT_H;
    break;
  case 5:
    return encTestmode.type == ETM_SPLIT_TT_V;
    break;
  case 6:
    return encTestmode.type == ETM_SPLIT_TT_H;
    break;
  default:
    THROW( "Unknown job-ID for parallelization of EncModeCtrlMTnoRQT: " << cs.picture->scheduler.getSplitJobId() );
    break;
  }
}

#endif


