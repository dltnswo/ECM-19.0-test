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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"


XUCache g_globalUnitCache = XUCache();

const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMPONENT] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure(CUCache& cuCache, PUCache& puCache, TUCache& tuCache)
  : area      ()
  , picture   ( nullptr )
  , parent    ( nullptr )
  , bestCS    ( nullptr )
  , m_isTopLayer(false)
  , m_isTuEnc ( false )
  , m_cuCache ( cuCache )
  , m_puCache ( puCache )
  , m_tuCache ( tuCache )
  , bestParent ( nullptr )
  , tmpColorSpaceCost(MAX_DOUBLE)
  , firstColorSpaceSelected(true)
#if JVET_Z0153_IBC_EXT_REF
  , resetIBCBuffer (true)
#else
  , resetIBCBuffer (false)
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  , m_lumaCUs       ( nullptr )      
  , m_lumaPUs       ( nullptr )      
  , m_lumaTUs       ( nullptr )      
  , m_lumaUnitScale ( nullptr )
  , m_lumaArea      ( nullptr )     
  , m_lumaCuIdx     ( nullptr )    
  , m_lumaPuIdx     ( nullptr )    
  , m_lumaTuIdx     ( nullptr )    
  , m_lumaParent    ( nullptr )    
  , m_bestCU        ( nullptr )
  , m_lastCodedCU   ( nullptr )
  , m_savedCost     ( MAX_DOUBLE )
  , m_lumaFracBits  ( MAX_UINT )
  , m_lumaDist      ( MAX_UINT )
  , m_lumaCost      ( MAX_DOUBLE )
#endif
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_coeffs[ i ] = nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[ i ] = nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[ i ] = nullptr;
#endif
    m_offsets[ i ] = 0;
  }

  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = nullptr;
#endif
    m_runType[i] = nullptr;
  }

  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_cuIdx   [ i ] = nullptr;
    m_puIdx   [ i ] = nullptr;
    m_tuIdx   [ i ] = nullptr;
    m_isDecomp[ i ] = nullptr;
  }

#if JVET_Z0118_GDR
  m_motionBuf0    = nullptr;
  m_motionBuf1    = nullptr;
  picHeader       = nullptr;
#else
  m_motionBuf     = nullptr;
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  currQtDepthBuf  = nullptr;
#endif

#if JVET_AE0043_CCP_MERGE_TEMPORAL
  m_ccpmIdxBuf = nullptr;
  m_ccpModelLUT.clear();
#endif
#if JVET_AG0058_EIP
  m_eipIdxBuf = nullptr;
  m_eipModelLUT.clear();
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_Z0118_GDR
  m_ipmBuf0 = nullptr;
  m_ipmBuf1 = nullptr;  
#else
  m_ipmBuf = nullptr;
#endif
#endif

#if JVET_Z0136_OOB
  for (uint32_t i = 0; i < 2; i++)
  {
    mcMask[i] = nullptr;
    mcMaskChroma[i] = nullptr;
  }
#endif
  features.resize( NUM_ENC_FEATURES );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  treeType = TREE_D;
  modeType = MODE_TYPE_ALL;
#endif
  tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
  tmpColorSpaceIntraCost[1] = MAX_DOUBLE;
  firstColorSpaceTestOnly = false;

#if JVET_Z0118_GDR
  picHeader = nullptr;
  m_gdrEnabled = false;
#endif
#if JVET_AI0183_MVP_EXTENSION
  m_intersectingMvBuf = nullptr;
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  for (int refineTmvpColIdx = 0; refineTmvpColIdx < 2; refineTmvpColIdx++)
  {
    for (int refineTmvpColRefList = 0; refineTmvpColRefList < 2; refineTmvpColRefList++)
    {
      for (int refineTmvpCurRefList = 0; refineTmvpCurRefList < 2; refineTmvpCurRefList++)
      {
        for (int refineTmvpCurRefIdx = 0; refineTmvpCurRefIdx < MAX_NUM_REF; refineTmvpCurRefIdx++)
        {
          for (int refineTmvpModeType = 0; refineTmvpModeType < 2; refineTmvpModeType++)
          {
            tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0] = nullptr;
            tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1] = nullptr;
          }
        }
      }
    }
    tmvpRefinedAvailable[refineTmvpColIdx] = nullptr;
  }
#endif
}

void CodingStructure::destroy()
{
  picture   = nullptr;
  parent    = nullptr;

  m_pred.destroy();
#if NNVC_USE_PRED
  m_predCustom.destroy();
#endif
  m_resi.destroy();
#if JVET_Z0118_GDR
  m_reco0.destroy();
  if (m_gdrEnabled)
  {
    m_reco1.destroy();
  }
#else
  m_reco.destroy();
#endif
  m_orgr.destroy();

  destroyTemporaryCsData();


#if JVET_Z0118_GDR
  delete[] m_motionBuf0;
  if (m_gdrEnabled)
  {
    delete[] m_motionBuf1;
  }
  m_motionBuf0 = nullptr;
  m_motionBuf1 = nullptr;

  if (picHeader && m_gdrEnabled)
  {
    delete picHeader;
  }

  picHeader = nullptr;  
#else
  delete[] m_motionBuf;
  m_motionBuf = nullptr;
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  delete[] currQtDepthBuf;
  currQtDepthBuf = nullptr;
#endif 

#if JVET_AE0043_CCP_MERGE_TEMPORAL
  if (m_ccpmIdxBuf)
  {
    delete [] m_ccpmIdxBuf;
  }
  m_ccpmIdxBuf = nullptr;
  m_ccpModelLUT.clear();
#endif
#if JVET_AG0058_EIP
  if (m_eipIdxBuf)
  {
    delete [] m_eipIdxBuf;
  }
  m_eipIdxBuf = nullptr;
  m_eipModelLUT.clear();
#endif

#if JVET_W0123_TIMD_FUSION
#if JVET_Z0118_GDR
  delete[] m_ipmBuf0;
  if (m_gdrEnabled)
  {
    delete[] m_ipmBuf1;
  }
  m_ipmBuf0 = nullptr;
  m_ipmBuf1 = nullptr;
#else
  delete[] m_ipmBuf;
  m_ipmBuf = nullptr;
#endif
#endif

}

void CodingStructure::destroyTemporaryCsData()
{
  destroyCoeffs();
  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    delete[] m_isDecomp[i];
    m_isDecomp[i] = nullptr;

    delete[] m_cuIdx[i];
    m_cuIdx[i] = nullptr;

    delete[] m_puIdx[i];
    m_puIdx[i] = nullptr;

    delete[] m_tuIdx[i];
    m_tuIdx[i] = nullptr;
  }

#if JVET_Z0136_OOB
  for (uint32_t i = 0; i < 2; i++)
  {
    if (mcMask[i])
    {
      xFree(mcMask[i]);
      mcMask[i] = nullptr;
    }

    if (mcMaskChroma[i])
    {
      xFree(mcMaskChroma[i]);
      mcMaskChroma[i] = nullptr;
    }
  }
#endif

  for (auto& pcu : cus)
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }
  m_tuCache.cache(tus);
  m_numTUs = 0;
  m_puCache.cache(pus);
  m_numPUs = 0;
  for (auto& pcu : cus)
  {
    pcu->firstPU = pcu->lastPU = nullptr;
  }
  m_cuCache.cache(cus);
  m_numCUs = 0;
#if JVET_AI0183_MVP_EXTENSION
  if (m_intersectingMvBuf != nullptr)
  {
    delete[] m_intersectingMvBuf;
    m_intersectingMvBuf = nullptr;
  }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  for (int refineTmvpColIdx = 0; refineTmvpColIdx < 2; refineTmvpColIdx++)
  {
    for (int refineTmvpColRefList = 0; refineTmvpColRefList < 2; refineTmvpColRefList++)
    {
      for (int refineTmvpCurRefList = 0; refineTmvpCurRefList < 2; refineTmvpCurRefList++)
      {
        for (int refineTmvpCurRefIdx = 0; refineTmvpCurRefIdx < MAX_NUM_REF; refineTmvpCurRefIdx++)
        {
          for (int refineTmvpModeType = 0; refineTmvpModeType < 2; refineTmvpModeType++)
          {
            if (tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0] != nullptr)
            {
              delete [] tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0];
              tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0] = nullptr;
            }
            if (tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1] != nullptr)
            {
              delete [] tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1];
              tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1] = nullptr;
            }
          }
        }
      }
    }
    if (tmvpRefinedAvailable[refineTmvpColIdx] != nullptr)
    {
      delete [] tmvpRefinedAvailable[refineTmvpColIdx];
      tmvpRefinedAvailable[refineTmvpColIdx] = nullptr;
    }
  }
#endif
}

void CodingStructure::createTemporaryCsData(const bool isPLTused)
{
  const unsigned numCh = ::getNumberValidChannels(area.chromaFormat);
  createCoeffs(isPLTused);
  for (unsigned i = 0; i < numCh; i++)
  {
    unsigned _area = unitScale[i].scale(area.blocks[i].size()).area();

    CHECK(m_cuIdx[i] != nullptr, "m_cuIdx[i] != nullptr");
    CHECK(m_puIdx[i] != nullptr, "m_puIdx[i] != nullptr");
    CHECK(m_tuIdx[i] != nullptr, "m_tuIdx[i] != nullptr");
    CHECK(m_isDecomp[i] != nullptr, "m_isDecomp[i] != nullptr");
    m_cuIdx[i] = _area > 0 ? new unsigned[_area] : nullptr;
    m_puIdx[i] = _area > 0 ? new unsigned[_area] : nullptr;
    m_tuIdx[i] = _area > 0 ? new unsigned[_area] : nullptr;
    m_isDecomp[i] = _area > 0 ? new bool[_area] : nullptr;
  }

#if JVET_Z0136_OOB
  int extendLumaArea = area.lumaSize().area();
  for (unsigned i = 0; i < 2; i++)
  {
    CHECK(mcMask[i] != nullptr, "mcMask[i] != nullptr");
    mcMask[i] = (bool*)xMalloc(bool, extendLumaArea);
    mcMaskChroma[i] = (bool*)xMalloc(bool, extendLumaArea);
  }
#endif
#if JVET_AI0183_MVP_EXTENSION && !JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  m_intersectingMvBuf = new IntersectingMvData[unitScale[0].scale(area.blocks[0].size()).area()];
#endif
}
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
void CodingStructure::createTemporaryRefineTmvpCsData(bool isIntra, int* numRefPic, bool isDec)
{
  if (isIntra)
  {
    return;
  }
#if JVET_AI0183_MVP_EXTENSION
  m_intersectingMvBuf = new IntersectingMvData[unitScale[0].scale(area.blocks[0].size()).area()];
#endif
  int refineTmvpBufSize = (area.lumaSize().area() >> 4);
  size_t refineTmvpTempBufSize = sizeof(int) * refineTmvpBufSize;
  tmvpRefinedAvailable[0] = new int[refineTmvpBufSize];
  for (int refineTmvpIdx = 0; refineTmvpIdx < refineTmvpBufSize; refineTmvpIdx++)
  {
    tmvpRefinedAvailable[0][refineTmvpIdx] = MAX_INT;
  }
  tmvpRefinedAvailable[1] = new int[refineTmvpBufSize];
  std::memcpy(tmvpRefinedAvailable[1], tmvpRefinedAvailable[0], refineTmvpTempBufSize);
  if (isDec)
  {
    return;
  }
  for (int refineTmvpColIdx = 0; refineTmvpColIdx < 2; refineTmvpColIdx++)
  {
    for (int refineTmvpColRefList = 0; refineTmvpColRefList < 2; refineTmvpColRefList++)
    {
      for (int refineTmvpCurRefList = 0; refineTmvpCurRefList < 2; refineTmvpCurRefList++)
      {
        for (int refineTmvpCurRefIdx = 0; refineTmvpCurRefIdx < numRefPic[refineTmvpCurRefList]; refineTmvpCurRefIdx++)
        {
          for (int refineTmvpModeType = 0; refineTmvpModeType < 2; refineTmvpModeType++)
          {
            tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0] = new int[refineTmvpBufSize];
            tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1] = new int[refineTmvpBufSize];
            std::memcpy(tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][0], tmvpRefinedAvailable[0], refineTmvpTempBufSize);
            std::memcpy(tmvpRefinedMv[refineTmvpColIdx][refineTmvpColRefList][refineTmvpCurRefList][refineTmvpCurRefIdx][refineTmvpModeType][1], tmvpRefinedAvailable[0], refineTmvpTempBufSize);
          }
        }
      }
    }
  }
}
#endif

void CodingStructure::releaseIntermediateData()
{
  clearTUs();
  clearPUs();
  clearCUs();
}

#if JVET_Z0118_GDR
bool CodingStructure::isCuCrossIRA(int begX) const
{
  if ((area.lx() < begX) && (begX < (area.lx() + area.lwidth())))
  {
    return true;
  }

  return false;
}

bool CodingStructure::isCuCrossVB(int endX) const
{
  if ((area.lx() < endX) && (endX < (area.lx() + area.lwidth())))
  {
    return true;
  }

  return false;
}

bool CodingStructure::containRefresh(int begX, int endX) const
{
  if (begX == endX)
  {
    return false;
  }
  
  if ((area.lx() <= begX) && (endX <= (area.lx() + area.lwidth())))
  {
    return true;
  }

  return false;
}

bool CodingStructure::overlapRefresh(int begX, int endX) const
{
  if (begX == endX)
  {
    return false;
  }

  if ((begX < (area.lx() + area.lwidth())) || (area.lx() < endX))
  {
    return true;
  }

  return false;
}

bool CodingStructure::withinRefresh(int begX, int endX) const
{
  if (begX == endX)
  {
    return false;
  }

  if ((begX <= area.lx()) && ((area.lx() + area.lwidth()) <= endX))
  {  
    return true;
  }

  return false;
}

Area CodingStructure::findOverlappedArea(const Area &a1, const Area &a2) const
{
  Area intersectArea = Area(Position(0, 0), Size(0, 0));

  if (
    ((a1.x <= a2.x && a2.x <= a1.topRight().x)    || (a2.x <= a1.x && a1.x <= a2.topRight().x)) &&
    ((a1.y <= a2.y && a2.y <= a1.bottomRight().y) || (a2.y <= a1.y && a1.y <= a2.bottomRight().y))
    ) 
  {
    int xArray[4] = { a1.x, a1.topRight().x,    a2.x, a2.topRight().x };
    int yArray[4] = { a1.y, a1.bottomRight().y, a2.y, a2.bottomRight().y };
    std::sort(xArray, xArray + 4);
    std::sort(yArray, yArray + 4);

    intersectArea = Area(Position(xArray[1], yArray[1]), Size(xArray[2] - xArray[1] + 1, yArray[2] - yArray[1] + 1));
  }
  
  return intersectArea;
}
#endif



#if JVET_Z0118_GDR
bool CodingStructure::isInGdrIntervalOrRecoveryPoc() const
{
  if( !m_gdrEnabled )
  {
    return false;
  }

  PicHeader     *curPh = picHeader;
  bool isCurGdrIntervalPic = curPh->getInGdrInterval();
  bool isCurGdrRecoveryPocPic = curPh->getIsGdrRecoveryPocPic();

  if (isCurGdrIntervalPic || isCurGdrRecoveryPocPic)
  {
    return true;
  }

  return false;
}

bool CodingStructure::isClean(const ChannelType effChType) const
{
  if( !m_gdrEnabled )
  {
    return false;
  }

  bool ret = isClean(area.Y(), effChType);

  return ret;
}

bool CodingStructure::isClean(const Position &IntPos, RefPicList e, int refIdx) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */
  const Picture* const refPic = slice->getRefPic(e, refIdx);

  if (!refPic || refIdx < 0)
  {
    return false;
  }

  PicHeader     *refPh = refPic->cs->picHeader;
  bool isRefGdrIntervalPic = refPh->getInGdrInterval();
  bool isRefGdrRecoveryPocPic = refPh->getIsGdrRecoveryPocPic();

  if (isInGdrIntervalOrRecoveryPoc())
  {
    int virboundaryEndx = 0;

    if (isRefGdrIntervalPic)
    {
      virboundaryEndx = refPh->getVirtualBoundariesPosX(0);
    }

    if (isRefGdrRecoveryPocPic)
    {
      virboundaryEndx = slice->getPPS()->getPicWidthInLumaSamples();
    }

    if (IntPos.x < virboundaryEndx)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    // refPic is normal picture
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);

    if (isCurGdrPicture)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

bool CodingStructure::isClean(const Position &IntPos, const Picture* const refPic) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  if (!refPic)
  {
    return false;
  }

  PicHeader     *refPh = refPic->cs->picHeader;
  bool isRefGdrIntervalPic    = refPh->getInGdrInterval();
  bool isRefGdrRecoveryPocPic = refPh->getIsGdrRecoveryPocPic();

  if (isInGdrIntervalOrRecoveryPoc())
  {
    int virboundaryEndx = 0;

    if (isRefGdrIntervalPic)
    {
      virboundaryEndx = refPh->getVirtualBoundariesPosX(0);
    }

    if (isRefGdrRecoveryPocPic)
    {
      virboundaryEndx = slice->getPPS()->getPicWidthInLumaSamples();
    }

    if (IntPos.x < virboundaryEndx)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    // refPic is normal picture
    bool isCurGdrPicture = (slice->getPicHeader()->getNumVerVirtualBoundaries() > 0);

    if (isCurGdrPicture)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
}

bool CodingStructure::isClean(const int Intx, const int Inty, const ChannelType effChType) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  /*
    1. non gdr picture --> false;
    2. gdr picture
         pos in clean area -> true
         pos in dirty area -> false
  */
  PicHeader     *curPh = picHeader;
  bool isCurGdrIntervalPic    = curPh->getInGdrInterval();
  bool isCurGdrRecoveryPocPic = curPh->getIsGdrRecoveryPocPic();
  
  if (isInGdrIntervalOrRecoveryPoc())
  {
    int virboundaryEndx = 0;
    
    if (isCurGdrIntervalPic)
    {
      virboundaryEndx = curPh->getVirtualBoundariesPosX(0);
    }

    if (isCurGdrRecoveryPocPic)
    {
      virboundaryEndx = slice->getPPS()->getPicWidthInLumaSamples();
    }

    virboundaryEndx = virboundaryEndx >> effChType;
    if (Intx < virboundaryEndx)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}

bool CodingStructure::isClean(const Position &IntPos, const ChannelType effChType) const
{
  if( !m_gdrEnabled)
  {
    return false;
  }

  bool ret = isClean(IntPos.x, IntPos.y, effChType);

  return ret;
}

bool CodingStructure::isClean(const Area &area, const ChannelType effChType) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  Position pTopLeft  = area.topLeft();
  Position pTopRight = area.topRight();
  Position pBotLeft  = area.bottomLeft();
  Position pBotRight = area.bottomRight();

  bool bTopLeft  = isClean(pTopLeft,  effChType);
  bool bTopRight = isClean(pTopRight, effChType);
  bool bBotLeft  = isClean(pBotLeft,  effChType);
  bool bBotRight = isClean(pBotRight, effChType);

  return bTopLeft && bTopRight && bBotLeft && bBotRight;
}

bool CodingStructure::isClean(const CodingUnit &cu) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  bool ret = cu.Y().valid() ? isClean(cu.Y().bottomRight(), CHANNEL_TYPE_LUMA) : isClean(cu.Cb().bottomRight(), CHANNEL_TYPE_CHROMA);

  return ret;
}

bool CodingStructure::isClean(const PredictionUnit &pu) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  bool ret = pu.Y().valid() ? isClean(pu.Y().bottomRight(), CHANNEL_TYPE_LUMA) : isClean(pu.Cb().bottomRight(), CHANNEL_TYPE_CHROMA);

  return ret;
}

bool CodingStructure::isClean(const TransformUnit &tu) const
{
  if (!m_gdrEnabled)
  {
    return false;
  }

  bool ret = tu.Y().valid() ? isClean(tu.Y().bottomRight(), CHANNEL_TYPE_LUMA) : isClean(tu.Cb().bottomRight(), CHANNEL_TYPE_CHROMA);

  return ret;
}
#endif

#if JVET_Z0118_GDR
void CodingStructure::updateReconMotIPM(const UnitArea &uarea) const
{
  if (!m_gdrEnabled)
  {
    picture->getRecoBuf(uarea).copyFrom(getRecoBuf(uarea));
    return;
  }

  updateReconMotIPM(uarea, getRecoBuf(uarea));
}

void CodingStructure::updateReconMotIPM(const CompArea &carea) const
{
  if (!m_gdrEnabled)
  {
    picture->getRecoBuf(carea).copyFrom(getRecoBuf(carea));
    return;
  }

  updateReconMotIPM(carea, getRecoBuf(carea));
}

void CodingStructure::updateReconMotIPM(const UnitArea &uarea, const CPelUnitBuf &pbuf) const
{  
  if (!m_gdrEnabled)
  {
    picture->getRecoBuf(uarea).copyFrom(pbuf);
    return;
  }

  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    ComponentID compID = (ComponentID)i;
    if (uarea.block(compID).valid())
    {
      updateReconMotIPM(uarea.block(compID), pbuf.get(compID));
    }
  }
}

void CodingStructure::updateReconMotIPM(const CompArea &carea, const CPelBuf &pbuf) const
{
  if (!m_gdrEnabled)
  {
    picture->getRecoBuf(carea).copyFrom(pbuf);
    return;
  }

  const ComponentID compID = carea.compID;

  if (!isInGdrIntervalOrRecoveryPoc())
  {
    picture->getRecoBuf(carea).copyFrom(pbuf);
    if (compID == COMPONENT_Y)
    {
#if JVET_W0123_TIMD_FUSION
      picture->cs->getIpmBuf(carea).copyFrom(getIpmBuf(carea));
#endif
      picture->cs->getMotionBuf(carea).copyFrom(getMotionBuf(carea));
    }

    return;
  }

  picture->getBuf(carea, PIC_RECONSTRUCTION_0).copyFrom(pbuf);
  if (compID == COMPONENT_Y)
  {
#if JVET_W0123_TIMD_FUSION
    picture->cs->getIpmBuf(carea, PIC_RECONSTRUCTION_0).copyFrom(getIpmBuf(carea));
#endif
    picture->cs->getMotionBuf(carea, PIC_RECONSTRUCTION_0).copyFrom(getMotionBuf(carea));
  }

  ChromaFormat chromaFormat = sps->getChromaFormatIdc();
  int gdrEndX = picHeader->getGdrEndX()          >> getComponentScaleX((ComponentID)compID, chromaFormat);
  int gdrEndY = pps->getPicHeightInLumaSamples() >> getComponentScaleY((ComponentID)compID, chromaFormat);;

  CompArea cleanArea      = CompArea((ComponentID)compID, chromaFormat, Area(Position(0, 0), Size(gdrEndX, gdrEndY)));
  CompArea overlappedArea = CompArea((ComponentID)compID, chromaFormat, findOverlappedArea(cleanArea, carea));
  CompArea subArea        = CompArea((ComponentID)compID, chromaFormat, Area(overlappedArea.pos() - carea.pos(), overlappedArea.size()));

  if (subArea.area() > 0)
  {  
    picture->getBuf(overlappedArea, PIC_RECONSTRUCTION_1).copyFrom(pbuf.subBuf(subArea.pos(), subArea.size()));
    if (compID == COMPONENT_Y)
    {
#if JVET_W0123_TIMD_FUSION
      picture->cs->getIpmBuf(overlappedArea, PIC_RECONSTRUCTION_1).copyFrom(getIpmBuf(overlappedArea));
#endif
      picture->cs->getMotionBuf(overlappedArea, PIC_RECONSTRUCTION_1).copyFrom(getMotionBuf(overlappedArea));
    }
  }
}
#endif
    
#if JVET_Z0118_GDR
void CodingStructure::rspSignalPicture(const UnitArea &uarea, std::vector<Pel>& pLUT, bool usePred) const
{
  for (int i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    ComponentID compID = (ComponentID) i;
    if (uarea.block(compID).valid())
    {
      CompArea carea = uarea.block(compID);
      rspSignalPicture(carea, pLUT, usePred);
    }
  }
}

void CodingStructure::rspSignalPicture(const CompArea &carea, std::vector<Pel>& pLUT, bool usePred) const
{
  ComponentID compID = carea.compID;
    
  // 1. normal picture
  if (!isInGdrIntervalOrRecoveryPoc())
  {   
    PelBuf picRecoBuff = picture->getRecoBuf(carea);
    CPelBuf predBuf = usePred ? getPredBuf(carea) : picture->getRecoBuf(carea);

    picRecoBuff.rspSignal(predBuf, pLUT);

    return;
  }

  // 2.1. gdr interval dirty picture
  PelBuf picRecoBuff0 = picture->getBuf(carea, PIC_RECONSTRUCTION_0);
  CPelBuf predBuf0 = usePred ? getPredBuf(carea) : picture->getBuf(carea, PIC_RECONSTRUCTION_0);
  
  picRecoBuff0.rspSignal(predBuf0, pLUT);

  // 2.2. gdr interval clean picture
  ChromaFormat chromaFormat = sps->getChromaFormatIdc();
  int gdrEndX = picHeader->getGdrEndX()          >> getComponentScaleX((ComponentID)compID, chromaFormat);
  int gdrEndY = pps->getPicHeightInLumaSamples() >> getComponentScaleY((ComponentID)compID, chromaFormat);;

  CompArea cleanArea      = CompArea((ComponentID)compID, chromaFormat, Area(Position(0, 0), Size(gdrEndX, gdrEndY)));
  CompArea overlappedArea = CompArea((ComponentID)compID, chromaFormat, findOverlappedArea(cleanArea, carea));

  if (overlappedArea.area() > 0)
  {
    PelBuf picRecoBuff1 = picture->getBuf(overlappedArea, PIC_RECONSTRUCTION_1);    
    CPelBuf predBuf1 = usePred ? getPredBuf(overlappedArea) : picture->getBuf(overlappedArea, PIC_RECONSTRUCTION_1);

    picRecoBuff1.rspSignal(predBuf1, pLUT);
  }
}


void CodingStructure::reconstructPicture(const CompArea &carea, std::vector<Pel>& pLUT, CodingStructure *resiCS, bool lmcsEnable) const
{  
  ComponentID compID = carea.compID;
  
#if JVET_AG0145_ADAPTIVE_CLIPPING
  ClpRng clpRng = slice->clpRng(compID);
  if (compID == COMPONENT_Y)
  {
    if (lmcsEnable)
    {
      clpRng.min = pLUT[slice->getLumaPelMin()];
      clpRng.max = pLUT[slice->getLumaPelMax()];
    }
    else
    {
      if (slice->getSPS()->getUseLmcs() && slice->getLmcsEnabledFlag())
      {
        clpRng.min = pLUT[slice->getLumaPelMin()];
        clpRng.max = pLUT[slice->getLumaPelMax()];
      }
      else
      {
        clpRng.min = slice->getLumaPelMin();
        clpRng.max = slice->getLumaPelMax();
      }
    }
  }
#endif

  // 1. normal picture
  if (!isInGdrIntervalOrRecoveryPoc())  
  {
    PelBuf picRecoBuff = picture->getRecoBuf(carea);

    if (lmcsEnable)
    {
      picRecoBuff.rspSignal(getPredBuf(carea), pLUT);
#if JVET_AG0145_ADAPTIVE_CLIPPING
      picRecoBuff.reconstruct(picRecoBuff, resiCS->getResiBuf(carea), clpRng);
#else
      picRecoBuff.reconstruct(picRecoBuff, resiCS->getResiBuf(carea), slice->clpRng(compID));
#endif
    }
    else
    {
#if JVET_AG0145_ADAPTIVE_CLIPPING
      picRecoBuff.reconstruct(getPredBuf(carea), resiCS->getResiBuf(carea), clpRng);
#else
      picRecoBuff.reconstruct(getPredBuf(carea), resiCS->getResiBuf(carea), slice->clpRng(compID));
#endif
    }

    return;
  }
  
  // 2.1. gdr interval dirty picture
  PelBuf picRecoBuff0 = picture->getBuf(carea, PIC_RECONSTRUCTION_0);

  if (lmcsEnable)
  {
    picRecoBuff0.rspSignal(getPredBuf(carea), pLUT);
#if JVET_AG0145_ADAPTIVE_CLIPPING
    picRecoBuff0.reconstruct(picRecoBuff0, resiCS->getResiBuf(carea), clpRng);
#else
    picRecoBuff0.reconstruct(picRecoBuff0, resiCS->getResiBuf(carea), slice->clpRng(compID));
#endif
  }
  else
  {
#if JVET_AG0145_ADAPTIVE_CLIPPING
    picRecoBuff0.reconstruct(getPredBuf(carea), resiCS->getResiBuf(carea), clpRng);
#else
    picRecoBuff0.reconstruct(getPredBuf(carea), resiCS->getResiBuf(carea), slice->clpRng(compID));
#endif
  }

  // 2.2. gdr interval clean picture
  ChromaFormat chromaFormat = sps->getChromaFormatIdc();
  int gdrEndX = picHeader->getGdrEndX()          >> getComponentScaleX((ComponentID)compID, chromaFormat);
  int gdrEndY = pps->getPicHeightInLumaSamples() >> getComponentScaleY((ComponentID)compID, chromaFormat);;

  CompArea cleanArea      = CompArea((ComponentID)compID, chromaFormat, Area(Position(0, 0), Size(gdrEndX, gdrEndY)));
  CompArea overlappedArea = CompArea((ComponentID)compID, chromaFormat, findOverlappedArea(cleanArea, carea));  

  if (overlappedArea.area() > 0)
  {
    PelBuf picRecoBuff1 = picture->getBuf(overlappedArea, PIC_RECONSTRUCTION_1);

    if (lmcsEnable)
    {
      picRecoBuff1.rspSignal(getPredBuf(overlappedArea), pLUT);
#if JVET_AG0145_ADAPTIVE_CLIPPING
      picRecoBuff1.reconstruct(picRecoBuff1, resiCS->getResiBuf(overlappedArea), clpRng);
#else
      picRecoBuff1.reconstruct(picRecoBuff1, resiCS->getResiBuf(overlappedArea), slice->clpRng(compID));
#endif
    }
    else
    {
#if JVET_AG0145_ADAPTIVE_CLIPPING
      picRecoBuff1.reconstruct(getPredBuf(overlappedArea), resiCS->getResiBuf(overlappedArea), clpRng);
#else
      picRecoBuff1.reconstruct(getPredBuf(overlappedArea), resiCS->getResiBuf(overlappedArea), slice->clpRng(compID));
#endif
    }
  }
}
#endif


bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType )
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType ) const
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

void CodingStructure::setDecomp(const CompArea &_area, const bool _isCoded /*= true*/)
{
  const UnitScale& scale = unitScale[_area.compID];

  AreaBuf<bool> isCodedBlk( m_isDecomp[toChannelType( _area.compID )] + rsAddr( _area, area.blocks[_area.compID].pos(), area.blocks[_area.compID].width, scale ),
                            area.blocks[_area.compID].width >> scale.posx,
                            _area.width                     >> scale.posx,
                            _area.height                    >> scale.posy);
  isCodedBlk.fill( _isCoded );
}

void CodingStructure::setDecomp(const UnitArea &_area, const bool _isCoded /*= true*/)
{
  for( uint32_t i = 0; i < _area.blocks.size(); i++ )
  {
    if (_area.blocks[i].valid())
    {
      setDecomp(_area.blocks[i], _isCoded);
    }
  }
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const int CodingStructure::signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const
{
  if (CS::isDualITree(*this) || modeTypeParent != MODE_TYPE_ALL || partitioner.currArea().chromaFormat == CHROMA_444 || partitioner.currArea().chromaFormat == CHROMA_400 )
  {
    return LDT_MODE_TYPE_INHERIT;
  }
  int minLumaArea = partitioner.currArea().lumaSize().area();
  if (split == CU_QUAD_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT) // the area is split into 3 or 4 parts
  {
    minLumaArea = minLumaArea >> 2;
  }
  else if (split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT) // the area is split into 2 parts
  {
    minLumaArea = minLumaArea >> 1;
  }
  int minChromaBlock = minLumaArea >> (getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat) + getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat));

  bool is2xNChroma = (partitioner.currArea().chromaSize().width == 4 && split == CU_VERT_SPLIT) || (partitioner.currArea().chromaSize().width == 8 && split == CU_TRIV_SPLIT);
  return minChromaBlock >= 16 && !is2xNChroma ? LDT_MODE_TYPE_INHERIT : ((minLumaArea < 32) || slice->isIntra()) ? LDT_MODE_TYPE_INFER : LDT_MODE_TYPE_SIGNAL;
}

void CodingStructure::clearCuPuTuIdxMap(const UnitArea &_area, uint32_t numCu, uint32_t numPu, uint32_t numTu, uint32_t* pOffset )
{
  UnitArea clippedArea = clipArea( _area, *picture );
  uint32_t numCh = ::getNumberValidChannels( _area.chromaFormat );
  for( uint32_t i = 0; i < numCh; i++ )
  {
    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = clippedArea.blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf = scale.scale( _selfBlk );
    const Area scaledBlk = scale.scale( _blk );
    const size_t offset = rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    unsigned *idxPtrCU = m_cuIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrCU, scaledSelf.width, scaledBlk.size() ).fill( 0 );

    unsigned *idxPtrPU = m_puIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrPU, scaledSelf.width, scaledBlk.size() ).fill( 0 );

    unsigned *idxPtrTU = m_tuIdx[i] + offset;
    AreaBuf<uint32_t>( idxPtrTU, scaledSelf.width, scaledBlk.size() ).fill( 0 );
  }

  //pop cu/pu/tus
  for( int i = m_numTUs; i > numTu; i-- )
  {
    m_tuCache.cache( tus.back() );
    tus.pop_back();
    m_numTUs--;
  }
  for( int i = m_numPUs; i > numPu; i-- )
  {
    m_puCache.cache( pus.back() );
    pus.pop_back();
    m_numPUs--;
  }
  for( int i = m_numCUs; i > numCu; i-- )
  {
    m_cuCache.cache( cus.back() );
    cus.pop_back();
    m_numCUs--;
  }
  for( int i = 0; i < 3; i++ )
  {
    m_offsets[i] = pOffset[i];
  }
}

CodingUnit* CodingStructure::getLumaCU( const Position &pos )
{
  const ChannelType effChType = CHANNEL_TYPE_LUMA;
  const CompArea &_blk = area.blocks[effChType];
  CHECK( !_blk.contains( pos ), "must contain the pos" );

  const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

  if (idx != 0)
  {
    return cus[idx - 1];
  }
  else
  {
    return nullptr;
  }
}
#endif
CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!_blk.contains(pos))
#else
  if( !_blk.contains( pos ) || (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA) )
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    //keep this check, which is helpful to identify bugs
    if( treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
      CHECK( parent->treeType != TREE_D, "wrong parent treeType " );
    }
#endif
    if (parent)
    {
      return parent->getCU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)
    {
      return cus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

#if JVET_AH0135_TEMPORAL_PARTITIONING
CodingUnit* CodingStructure::getCUTempo(const Position& pos, const ChannelType effChType)
{
  const CompArea& _blk = area.blocks[effChType];
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!_blk.contains(pos))
#else
  if (!_blk.contains(pos) || (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA))
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    //keep this check, which is helpful to identify bugs
    if (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA)
    {
      CHECK(parent == nullptr, "parent shall be valid; consider using function getLumaCU()");
      CHECK(parent->treeType != TREE_D, "wrong parent treeType ");
    }
#endif
    if (parent)
    {
      return parent->getCU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_cuIdx[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[effChType])];
    if (idx != 0)
    {
      return cus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}
#endif
const CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType ) const
{
#if JVET_Z0118_GDR
  if (m_gdrEnabled)
  {
    Size lumaSize = slice->getPic()->Y().lumaSize();
    if( area.lumaSize() != lumaSize )
    {
      const Position posRB = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().bottomRight() : area.Cb().bottomRight();
      const Position posTL = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().topLeft() : area.Cb().topLeft();

      bool isTLClean = isClean( posTL, effChType );
      bool isRBClean = isClean( posRB, effChType );
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean( pos, effChType );

      if( isSrcClean && !isTarClean )
      {
        return nullptr;
      }
    }
  }
#endif  

  const CompArea &_blk = area.blocks[effChType];

#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!_blk.contains(pos))
#else
  if( !_blk.contains( pos ) || (treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA) )
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( treeType == TREE_C && effChType == CHANNEL_TYPE_LUMA )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
      CHECK( parent->treeType != TREE_D, "wrong parent treeType" );
    }
#endif
    if (parent)
    {
      return parent->getCU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)
    {
      return cus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

PredictionUnit* CodingStructure::getPU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getPU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)
    {
      return pus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

const PredictionUnit * CodingStructure::getPU( const Position &pos, const ChannelType effChType ) const
{
#if JVET_Z0118_GDR
  if (m_gdrEnabled)
  {
    Size lumaSize = slice->getPic()->Y().lumaSize();
    if( area.lumaSize() != lumaSize )
    {
      const Position posRB = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().bottomRight() : area.Cb().bottomRight();
      const Position posTL = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().topLeft() : area.Cb().topLeft();

      bool isTLClean = isClean( posTL, effChType );
      bool isRBClean = isClean( posRB, effChType );
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean( pos, effChType );

      if( isSrcClean && !isTarClean )
      {
        return nullptr;
      }
    }
  }
#endif  

  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getPU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if (idx != 0)
    {
      return pus[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

TransformUnit* CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx )
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];

        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains( pos ) )
            {
              extraIdx++;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
              CHECK( tus[idx - 1 + extraIdx]->cu->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
#endif
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if (m_isTuEnc)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
}

const TransformUnit * CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx ) const
{
#if JVET_Z0118_GDR
  if (m_gdrEnabled)
  {
    Size lumaSize = slice->getPic()->Y().lumaSize();
    if( area.lumaSize() != lumaSize )
    {
      const Position posRB = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().bottomRight() : area.Cb().bottomRight();
      const Position posTL = ( effChType == CHANNEL_TYPE_LUMA ) ? area.Y().topLeft() : area.Cb().topLeft();

      bool isTLClean = isClean( posTL, effChType );
      bool isRBClean = isClean( posRB, effChType );
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean( pos, effChType );

      if( isSrcClean && !isTarClean )
      {
        return nullptr;
      }
    }
  }
#endif  

  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if (parent)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];
        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while ( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains(pos) )
            {
              extraIdx++;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
              CHECK( tus[idx - 1 + extraIdx]->cu->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
#endif
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if (m_isTuEnc)
    {
      return parent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType, const PartSplit& implicitSplit )
#else
CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType )
#endif
{
  CodingUnit *cu = m_cuCache.get();

  cu->UnitArea::operator=( unit );
  cu->initData();
  cu->cs        = this;
  cu->slice     = nullptr;
  cu->next      = nullptr;
  cu->firstPU   = nullptr;
  cu->lastPU    = nullptr;
  cu->firstTU   = nullptr;
  cu->lastTU    = nullptr;
  cu->chType    = chType;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  cu->treeType = treeType;
  cu->modeType = modeType;
#endif
  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    prevCU->next = cu;
#if ENABLE_SPLIT_PARALLELISM

    CHECK( prevCU->cacheId != cu->cacheId, "Inconsintent cacheId between previous and current CU" );
#endif
  }

  cus.push_back( cu );

  uint32_t idx = ++m_numCUs;
  cu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  for( uint32_t i = 0; i < numCh && implicitSplit == CU_DONT_SPLIT; i++ )
#else
  for( uint32_t i = 0; i < numCh; i++ )
#endif
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = cu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_cuIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    CHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *cu;
}

PredictionUnit& CodingStructure::addPU( const UnitArea &unit, const ChannelType chType )
{
  PredictionUnit *pu = m_puCache.get();

  pu->UnitArea::operator=( unit );
  pu->initData();
  pu->next   = nullptr;
  pu->cs     = this;
  pu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  pu->chType = chType;
#if ENABLE_SPLIT_PARALLELISM

  CHECK( pu->cacheId != pu->cu->cacheId, "Inconsintent cacheId between the PU and assigned CU" );
  CHECK( pu->cu->firstPU != nullptr, "Without an RQT the firstPU should be null" );
#endif

  PredictionUnit *prevPU = m_numPUs > 0 ? pus.back() : nullptr;

  if( prevPU && prevPU->cu == pu->cu )
  {
    prevPU->next = pu;
#if ENABLE_SPLIT_PARALLELISM

    CHECK( prevPU->cacheId != pu->cacheId, "Inconsintent cacheId between previous and current PU" );
#endif
  }

  pus.push_back( pu );

  if( pu->cu->firstPU == nullptr )
  {
    pu->cu->firstPU = pu;
  }
  pu->cu->lastPU = pu;

  uint32_t idx = ++m_numPUs;
  pu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );
  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !pu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = pu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_puIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    CHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *pu;
}

TransformUnit& CodingStructure::addTU( const UnitArea &unit, const ChannelType chType )
{
  TransformUnit *tu = m_tuCache.get();

  tu->UnitArea::operator=( unit );
  tu->initData();
  tu->next   = nullptr;
  tu->prev   = nullptr;
  tu->cs     = this;
  tu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  tu->chType = chType;
#if ENABLE_SPLIT_PARALLELISM

  if( tu->cu )
  {
    CHECK(tu->cacheId != tu->cu->cacheId, "Inconsintent cacheId between the TU and assigned CU");
  }
#endif


  TransformUnit *prevTU = m_numTUs > 0 ? tus.back() : nullptr;

  if( prevTU && prevTU->cu == tu->cu )
  {
    prevTU->next = tu;
    tu->prev     = prevTU;
#if ENABLE_SPLIT_PARALLELISM

    CHECK( prevTU->cacheId != tu->cacheId, "Inconsintent cacheId between previous and current TU" );
#endif
  }

  tus.push_back( tu );

  if( tu->cu )
  {
    if( tu->cu->firstTU == nullptr )
    {
      tu->cu->firstTU = tu;
    }
    tu->cu->lastTU = tu;
  }

  uint32_t idx = ++m_numTUs;
  tu->idx  = idx;

  TCoeff *coeffs[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
#if SIGN_PREDICTION
  SIGN_PRED_TYPE *signs[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned *signsScanIdx[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
#endif
#endif
#if REMOVE_PCM
  Pel    *pltIdx[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
#else
  Pel    *pcmbuf[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
#endif
  bool   *runType[5]   = { nullptr, nullptr, nullptr, nullptr, nullptr };

  uint32_t numCh = ::getNumberValidComponents( area.chromaFormat );

  for (uint32_t i = 0; i < numCh; i++)
  {
    if (!tu->blocks[i].valid())
    {
      continue;
    }

    if (i < ::getNumberValidChannels(area.chromaFormat))
    {
      const CompArea &_selfBlk = area.blocks[i];
      const CompArea     &_blk = tu->blocks[i];

      bool isIspTu = tu->cu != nullptr && tu->cu->ispMode && isLuma(_blk.compID);

      bool isFirstIspTu = false;
      if (isIspTu)
      {
        isFirstIspTu = CU::isISPFirst(*tu->cu, _blk, getFirstComponentOfChannel(ChannelType(i)));
      }
      if (!isIspTu || isFirstIspTu)
      {
        const UnitScale& scale = unitScale[_blk.compID];

        const Area scaledSelf = scale.scale(_selfBlk);
        const Area scaledBlk = isIspTu ? scale.scale(tu->cu->blocks[i]) : scale.scale(_blk);
        unsigned *idxPtr = m_tuIdx[i] + rsAddr(scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width);
        CHECK(*idxPtr, "Overwriting a pre-existing value, should be '0'!");
        AreaBuf<uint32_t>(idxPtr, scaledSelf.width, scaledBlk.size()).fill(idx);
      }
    }

    coeffs[i] = m_coeffs[i] + m_offsets[i];
#if SIGN_PREDICTION
    signs[i]  = m_coeffSigns[i] + m_offsets[i];
#if JVET_Y0141_SIGN_PRED_IMPROVE
    signsScanIdx[i] = m_coeffSignsIdx[i] + m_offsets[i];
#endif
#endif
#if !REMOVE_PCM
    pcmbuf[i] = m_pcmbuf[i] + m_offsets[i];
#endif

    if (i < MAX_NUM_CHANNEL_TYPE)
    {
#if REMOVE_PCM
      if (m_pltIdx[i] != nullptr)
      {
        pltIdx[i] = m_pltIdx[i] + m_offsets[i];
      }
#endif
      if (m_runType[i] != nullptr)
      {
        runType[i] = m_runType[i] + m_offsets[i];
      }
    }

    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }
#if REMOVE_PCM
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  tu->init(coeffs, signs, signsScanIdx, pltIdx, runType);
#else
  tu->init(coeffs, signs, pltIdx, runType);
#endif
#else
  tu->init(coeffs, pltIdx, runType);
#endif
#else
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  tu->init(coeffs, signs, signsScanIdx, pcmbuf, runType);
#else
  tu->init(coeffs, signs, pcmbuf, runType);
#endif
#else
  tu->init(coeffs, pcmbuf, runType);
#endif
#endif

  return *tu;
}

void CodingStructure::addEmptyTUs( Partitioner &partitioner )
{
  const UnitArea& area    = partitioner.currArea();
  bool            split   = partitioner.canSplit(TU_MAX_TR_SPLIT, *this
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif  
  );


  const unsigned  trDepth = partitioner.currTrDepth;

  if( split )
  {
    partitioner.splitCurrArea( TU_MAX_TR_SPLIT, *this );
    do
    {
      addEmptyTUs( partitioner );
    } while( partitioner.nextPart( *this ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit &tu = this->addTU( CS::getArea( *this, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *this->pcv );
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
  }
}

CUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType )
{
  CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  CodingUnit* lastCU = firstCU;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( !CS::isDualITree( *this ) ) //for a more generalized separate tree
  {
    bool bContinue = true;
    CodingUnit* currCU = firstCU;
    while( bContinue )
    {
      if( currCU == nullptr )
      {
        bContinue = false;
        lastCU = currCU;
      }
      else if( currCU->chType != effChType )
      {
        lastCU = currCU;
        currCU = currCU->next;
      }
      else
      {
        if( unit.contains( *currCU ) )
        {
          lastCU = currCU;
          currCU = currCU->next;
        }
        else
        {
          bContinue = false;
          lastCU = currCU;
        }
      }
    }
  }
  else
  {
#endif
    do
    {
    } while (lastCU && (lastCU = lastCU->next) && unit.contains(*lastCU));
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
}
#endif
  return CUTraverser( firstCU, lastCU );
}

PUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType )
{
  PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  PredictionUnit* lastPU  = firstPU;

  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );

  return PUTraverser( firstPU, lastPU );
}

TUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType )
{
  TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );

  return TUTraverser( firstTU, lastTU );
}

cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const CodingUnit* lastCU  = firstCU;

  do { } while( lastCU && ( lastCU = lastCU->next ) && unit.contains( *lastCU ) );

  return cCUTraverser( firstCU, lastCU );
}

cPUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const PredictionUnit* lastPU  = firstPU;

  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );

  return cPUTraverser( firstPU, lastPU );
}

cTUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );

  return cTUTraverser( firstTU, lastTU );
}

// coding utilities

void CodingStructure::allocateVectorsAtPicLevel()
{
  const int  twice = ( !pcv->ISingleTree && slice->isIRAP() && pcv->chrFormat != CHROMA_400 ) ? 2 : 1;
  size_t allocSize = twice * unitScale[0].scale( area.blocks[0].size() ).area();

  cus.reserve( allocSize );
  pus.reserve( allocSize );
  tus.reserve( allocSize );
}

#if JVET_Z0118_GDR
void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabled)
#else
void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused)
#endif
{
#if JVET_Z0118_GDR
  m_gdrEnabled = isGdrEnabled;
#endif

  createInternals(UnitArea(_chromaFormat, _area), isTopLayer, isPLTused);

  if (isTopLayer)
  {
    return;
  }

#if JVET_Z0118_GDR
  m_pt = PIC_RECONSTRUCTION_0;
  m_reco0.create(area);
  if (m_gdrEnabled)
  {
    m_reco1.create(area);
    picHeader = new PicHeader();
    picHeader->initPicHeader();
  }
#else
  m_reco.create( area );
#endif

  m_pred.create( area );
#if NNVC_USE_PRED
  m_predCustom.create( area );
#endif
  m_resi.create( area );
  m_orgr.create( area );
}

#if JVET_Z0118_GDR
void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabled)
#else
void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused)
#endif
{
#if JVET_Z0118_GDR
  m_gdrEnabled = isGdrEnabled;
#endif

  createInternals(_unit, isTopLayer, isPLTused);
  
  if (isTopLayer)
  {
    return;
  }

#if JVET_Z0118_GDR
  m_pt = PIC_RECONSTRUCTION_0;
  m_reco0.create(area);
  if (m_gdrEnabled)
  {
    m_reco1.create(area);
    picHeader = new PicHeader();
    picHeader->initPicHeader();
  }  
#else
  m_reco.create( area );
#endif

  m_pred.create( area );
#if NNVC_USE_PRED
  m_predCustom.create( area );
#endif
  m_resi.create( area );
  m_orgr.create( area );
}

void CodingStructure::createInternals(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused)
{
  area = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
  parent  = nullptr;

  unsigned numCh = ::getNumberValidChannels(area.chromaFormat);


  for (unsigned i = 0; i < numCh; i++)
  {
    m_offsets[i] = 0;
  }

  m_isTopLayer = isTopLayer;
  if( !isTopLayer ) createTemporaryCsData(isPLTused);

  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
#if JVET_Z0118_GDR
  m_motionBuf0 = new MotionInfo[_lumaAreaScaled];
  if (m_gdrEnabled)
  {
    m_motionBuf1 = new MotionInfo[_lumaAreaScaled];
  }
#else
  m_motionBuf       = new MotionInfo[_lumaAreaScaled];
#endif

#if JVET_AH0135_TEMPORAL_PARTITIONING
  currQtDepthBuf = new SplitPred[_lumaAreaScaled];
#endif 
#if JVET_AE0043_CCP_MERGE_TEMPORAL
  m_ccpmIdxBuf = new int[_lumaAreaScaled];
  m_ccpModelLUT.resize(0);
#endif
#if JVET_AG0058_EIP
  m_eipIdxBuf = new int[_lumaAreaScaled];
  m_eipModelLUT.resize(0);
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_Z0118_GDR
  m_ipmBuf0 = new uint8_t[_lumaAreaScaled];
  if (m_gdrEnabled)
  {
    m_ipmBuf1 = new uint8_t[_lumaAreaScaled];
  }
#else
  m_ipmBuf          = new uint8_t[_lumaAreaScaled];
#endif
#endif // JVET_W0123_TIMD_FUSION
  if (!isTopLayer) initStructData();
}

void CodingStructure::addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> &lut, const MotionInfo &mi)
{
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}

#if JVET_Z0139_HIST_AFF
void CodingStructure::addAffMiToLut(static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutSet, const AffineMotionInfo addMi[2], int refIdx[2])
{
  for (int reflist = 0; reflist < 2; reflist++)
  {
    if (refIdx[reflist] != -1 && addMi[reflist].oneSetAffineParametersPattern != 0)
    {
      int idxInLUT = reflist * MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min(refIdx[reflist], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);

      static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>& lut = lutSet[idxInLUT];


      size_t currCnt = lut.size();

      bool pruned = false;
      int  sameCandIdx = 0;

      for (int idx = 0; idx < currCnt; idx++)
      {
        if (lut[idx] == addMi[reflist])
        {
          sameCandIdx = idx;
          pruned = true;
          break;
        }
      }

      if (pruned || currCnt == lut.capacity())
      {
        lut.erase(lut.begin() + sameCandIdx);
      }

      lut.push_back(addMi[reflist]);
    }
  }
}
void CodingStructure::addAffInheritToLut(static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS>& lut, const AffineInheritInfo& mi)
{
  size_t currCnt = lut.size();

  bool pruned = false;
  int  sameCandIdx = 0;
  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
void CodingStructure::addMiToLutIBC(static_vector<MotionInfo, MAX_NUM_HMVP_IBC_CANDS> &lut, const MotionInfo &mi)
{
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (mi.interDir == 3)
  {
    for (int l = 1; l >= 0; l--)
    {
      MotionInfo saveMi = mi;
      if (l == 1)
      {
        saveMi.mv[0] = saveMi.mv[1];
      }
      saveMi.interDir = 1;
      saveMi.mv[1] = Mv();
      saveMi.refIdx[1] = -1;
      saveMi.bv = saveMi.mv[0];
      saveMi.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);

      size_t currCnt = lut.size();

      bool pruned      = false;
      int  sameCandIdx = 0;

      for (int idx = 0; idx < currCnt; idx++)
      {
        if (lut[idx] == saveMi)
        {
          sameCandIdx = idx;
          pruned      = true;
          break;
        }
      }

      if (pruned || currCnt == lut.capacity())
      {
        lut.erase(lut.begin() + sameCandIdx);
      }

      lut.push_back(saveMi);
    }
    return;
  }
#endif
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}
#endif

void CodingStructure::resetPrevPLT(PLTBuf& prevPLT)
{
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    prevPLT.curPLTSize[comp] = 0;
  }

  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memset(prevPLT.curPLT[comp], 0, MAXPLTPREDSIZE * sizeof(Pel));
  }
}

void CodingStructure::reorderPrevPLT(PLTBuf& prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE], Pel curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE], bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin, uint32_t numComp, bool jointPLT)
{
  Pel stuffedPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE];
  uint8_t tempCurPLTsize[MAX_NUM_CHANNEL_TYPE];
  uint8_t stuffPLTsize[MAX_NUM_COMPONENT];

  uint32_t maxPredPltSize = jointPLT ? MAXPLTPREDSIZE : MAXPLTPREDSIZE_DUALTREE;

  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    tempCurPLTsize[comID] = curPLTSize[comID];
    stuffPLTsize[i] = 0;
    memcpy(stuffedPLT[i], curPLT[i], curPLTSize[comID] * sizeof(Pel));
  }

  for (int ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((ch > 0) ? COMPONENT_Cb : COMPONENT_Y);
    if (ch > 1) break;
    for (int i = 0; i < prevPLT.curPLTSize[comID]; i++)
    {
      if (tempCurPLTsize[comID] + stuffPLTsize[ch] >= maxPredPltSize)
      {
        break;
      }

      if (!reuseflag[comID][i])
      {
        if (ch == COMPONENT_Y)
        {
          stuffedPLT[0][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[0][i];
        }
        else
        {
          stuffedPLT[1][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[1][i];
          stuffedPLT[2][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[2][i];
        }
        stuffPLTsize[ch]++;
      }
    }
  }

  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    prevPLT.curPLTSize[comID] = curPLTSize[comID] + stuffPLTsize[comID];
    memcpy(prevPLT.curPLT[i], stuffedPLT[i], prevPLT.curPLTSize[comID] * sizeof(Pel));
    CHECK(prevPLT.curPLTSize[comID] > maxPredPltSize, " Maximum palette predictor size exceed limit");
  }
}

void CodingStructure::setPrevPLT(PLTBuf predictor)
{
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    prevPLT.curPLTSize[comp] = predictor.curPLTSize[comp];
  }
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memcpy(prevPLT.curPLT[comp], predictor.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}
void CodingStructure::storePrevPLT(PLTBuf& predictor)
{
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    predictor.curPLTSize[comp] = prevPLT.curPLTSize[comp];
  }
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    memcpy(predictor.curPLT[comp], prevPLT.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}

void CodingStructure::rebindPicBufs()
{
  CHECK( parent, "rebindPicBufs can only be used for the top level CodingStructure" );

#if JVET_Z0118_GDR
  if (!picture->M_BUFS(0, PIC_RECONSTRUCTION).bufs.empty())
  {
    m_pt = PIC_RECONSTRUCTION_0;
    m_reco0.createFromBuf(picture->M_BUFS(0, PIC_RECONSTRUCTION_0));
    if (m_gdrEnabled)
    {
      m_reco1.createFromBuf(picture->M_BUFS(0, PIC_RECONSTRUCTION_1));
    }
  }
  else
  {
    m_reco0.destroy();
    if (m_gdrEnabled)
    {
      m_reco1.destroy();
    }
  }
#else
  if (!picture->M_BUFS(0, PIC_RECONSTRUCTION).bufs.empty())
  {
    m_reco.createFromBuf(picture->M_BUFS(0, PIC_RECONSTRUCTION));
  }
  else
  {
    m_reco.destroy();
  }
#endif
  if (!picture->M_BUFS(0, PIC_PREDICTION).bufs.empty())
  {
    m_pred.createFromBuf(picture->M_BUFS(0, PIC_PREDICTION));
  }
  else
  {
    m_pred.destroy();
  }
#if NNVC_USE_PRED
  if( !picture->M_BUFS( 0, PIC_PREDICTION_CUSTOM ).bufs.empty() )
  {
    m_predCustom.createFromBuf( picture->M_BUFS( 0, PIC_PREDICTION_CUSTOM ) );
  }
  else
  {
    m_predCustom.destroy();
  }
#endif
  if (!picture->M_BUFS(0, PIC_RESIDUAL).bufs.empty())
  {
    m_resi.createFromBuf(picture->M_BUFS(0, PIC_RESIDUAL));
  }
  else
  {
    m_resi.destroy();
  }
  if( pcv->isEncoder )
  {
    if (!picture->M_BUFS(0, PIC_ORG_RESI).bufs.empty())
    {
      m_orgr.create(area.chromaFormat, area.blocks[0], pcv->maxCUWidth);
    }
    else
    {
      m_orgr.destroy();
    }
  }
}

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
void CodingStructure::createTMBuf(const int cbWidth, const int cbHeight)
{
  m_pCurrTmTop  = (Pel *) xMalloc(Pel, AML_MERGE_TEMPLATE_SIZE * cbWidth);
  m_pCurrTmLeft = (Pel *) xMalloc(Pel, AML_MERGE_TEMPLATE_SIZE * cbHeight);
  m_pRefTmTop   = (Pel *) xMalloc(Pel, AML_MERGE_TEMPLATE_SIZE * cbWidth);
  m_pRefTmLeft  = (Pel *) xMalloc(Pel, AML_MERGE_TEMPLATE_SIZE * cbHeight);

  m_pcBufPredCurTop  = PelBuf(m_pCurrTmTop, cbWidth, AML_MERGE_TEMPLATE_SIZE);
  m_pcBufPredCurLeft = PelBuf(m_pCurrTmLeft, AML_MERGE_TEMPLATE_SIZE, cbHeight);
  m_pcBufPredRefTop  = PelBuf(m_pRefTmTop, cbWidth, AML_MERGE_TEMPLATE_SIZE);
  m_pcBufPredRefLeft = PelBuf(m_pRefTmLeft, AML_MERGE_TEMPLATE_SIZE, cbHeight);
}

void CodingStructure::destroyTMBuf()
{
  xFree(m_pCurrTmTop);
  m_pCurrTmTop = nullptr;
  xFree(m_pCurrTmLeft);
  m_pCurrTmLeft = nullptr;
  xFree(m_pRefTmTop);
  m_pRefTmTop = nullptr;
  xFree(m_pRefTmLeft);
  m_pRefTmLeft = nullptr;
}
#endif

void CodingStructure::createCoeffs(const bool isPLTused)
{
  const unsigned numCh = getNumberValidComponents( area.chromaFormat );

  for( unsigned i = 0; i < numCh; i++ )
  {
    unsigned _area = area.blocks[i].area();

    m_coeffs[i] = _area > 0 ? ( TCoeff* ) xMalloc( TCoeff, _area ) : nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[i] = _area > 0 ? (SIGN_PRED_TYPE *) xMalloc(SIGN_PRED_TYPE, _area) : nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = _area > 0 ? (unsigned*)xMalloc(unsigned, _area) : nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = _area > 0 ? ( Pel*    ) xMalloc( Pel,    _area ) : nullptr;
#endif
  }

  if (isPLTused)
  {
    for (unsigned i = 0; i < (isChromaEnabled(area.chromaFormat) ? 2 : 1); i++)
    {
      unsigned _area = area.blocks[i].area();

#if REMOVE_PCM
      m_pltIdx[i] = _area > 0 ? (Pel*)xMalloc(Pel, _area) : nullptr;
#endif
      m_runType[i] = _area > 0 ? (bool*)xMalloc(bool, _area) : nullptr;
    }
  }
}

void CodingStructure::destroyCoeffs()
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if (m_coeffs[i])
    {
      xFree(m_coeffs[i]);
      m_coeffs[i] = nullptr;
    }
#if SIGN_PREDICTION
    if( m_coeffSigns[i] )
    { 
      xFree( m_coeffSigns[i] );
      m_coeffSigns[i] = nullptr;
    }
#if JVET_Y0141_SIGN_PRED_IMPROVE
    if (m_coeffSignsIdx[i])
    {
      xFree(m_coeffSignsIdx[i]);
      m_coeffSignsIdx[i] = nullptr;
    }
#endif
#endif
#if !REMOVE_PCM
    if (m_pcmbuf[i])
    {
      xFree(m_pcmbuf[i]);
      m_pcmbuf[i] = nullptr;
    }
#endif
  }

  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
#if REMOVE_PCM
    if (m_pltIdx[i])
    {
      xFree(m_pltIdx[i]);
      m_pltIdx[i] = nullptr;
    }
#endif
    if (m_runType[i])
    {
      xFree(m_runType[i]);
      m_runType[i] = nullptr;
    }
  }
}

void CodingStructure::initSubStructure( CodingStructure& subStruct, const ChannelType _chType, const UnitArea &subArea, const bool &isTuEnc )
{
  CHECK( this == &subStruct, "Trying to init self as sub-structure" );

  subStruct.useDbCost = false;
  subStruct.costDbOffset = 0;

  for( uint32_t i = 0; i < subStruct.area.blocks.size(); i++ )
  {
    CHECKD( subStruct.area.blocks[i].size() != subArea.blocks[i].size(), "Trying to init sub-structure of incompatible size" );

    subStruct.area.blocks[i].pos() = subArea.blocks[i].pos();
  }

  if( parent )
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD( !area.contains( subStruct.area ), "Trying to init sub-structure not contained in the parent" );
  }

  subStruct.parent    = this;
  subStruct.picture   = picture;

  subStruct.sps       = sps;
  subStruct.vps       = vps;
  subStruct.pps       = pps;
#if JVET_Z0118_GDR
  if (m_gdrEnabled)
  {
    if (!subStruct.picHeader)
    {      
      subStruct.picHeader = new PicHeader;
      subStruct.picHeader->initPicHeader();
    }
    *subStruct.picHeader = *picHeader;
  }
  else
  {
    subStruct.picHeader = picHeader;
  }
#else
  subStruct.picHeader = picHeader;
#endif

  memcpy(subStruct.alfApss, alfApss, sizeof(alfApss));

#if JVET_AK0065_TALF
  memcpy(subStruct.talfApss, talfApss, sizeof(talfApss));
#endif
  subStruct.lmcsAps = lmcsAps;
  subStruct.scalinglistAps = scalinglistAps;

  subStruct.slice     = slice;
  subStruct.baseQP    = baseQP;
  subStruct.prevQP[_chType]
                      = prevQP[_chType];
  subStruct.pcv       = pcv;

  subStruct.m_isTuEnc = isTuEnc;

  subStruct.motionLut = motionLut;
#if JVET_AD0188_CCP_MERGE
  subStruct.ccpLut    = ccpLut;
#endif
#if JVET_AG0058_EIP
  subStruct.eipLut = eipLut;
#endif
  subStruct.prevPLT = prevPLT;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  subStruct.treeType  = treeType;
  subStruct.modeType  = modeType;
#endif
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
  subStruct.splitPredLuma = splitPredLuma;
#endif
  subStruct.initStructData( currQP[_chType] );
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( slice->shouldCopyLumaCUs() )
  {
    copyLumaPointers( subStruct );
  }
#endif

  if( isTuEnc )
  {
    CHECKD( area != subStruct.area, "Trying to init sub-structure for TU-encoding of incompatible size" );

    for( const auto &pcu : cus )
    {
      CodingUnit &cu = subStruct.addCU( *pcu, _chType );

      cu = *pcu;
    }

    for( const auto &ppu : pus )
    {
      PredictionUnit &pu = subStruct.addPU( *ppu, _chType );

      pu = *ppu;
    }

    unsigned numComp = ::getNumberValidChannels( area.chromaFormat );
    for( unsigned i = 0; i < numComp; i++)
    {
      ::memcpy( subStruct.m_isDecomp[i], m_isDecomp[i], (unitScale[i].scale( area.blocks[i].size() ).area() * sizeof( bool ) ) );
    }
  }
#if JVET_Z0118_GDR
  subStruct.m_pt = m_pt;
#endif
}

void CodingStructure::useSubStructure( const CodingStructure& subStruct, const ChannelType chType, const UnitArea &subArea, const bool cpyPred /*= true*/, const bool cpyReco /*= true*/, const bool cpyOrgResi /*= true*/, const bool cpyResi /*= true*/, const bool updateCost /*= true*/ )
{
  UnitArea clippedArea = clipArea( subArea, *picture );

  setDecomp( clippedArea );

  CPelUnitBuf subPredBuf = cpyPred ? subStruct.getPredBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subResiBuf = cpyResi ? subStruct.getResiBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subRecoBuf = cpyReco ? subStruct.getRecoBuf( clippedArea ) : CPelUnitBuf();

  if( parent )
  {
    // copy data to picture
#if NNVC_USE_PRED
    getPredBufCustom              ( clippedArea ).copyFrom( subStruct.getPredBuf( clippedArea ) );
    getPredBuf                    ( clippedArea ).copyFrom( subStruct.getPredBuf( clippedArea ) );
#endif	
    if( cpyPred )
    {
      getPredBuf( clippedArea ).copyFrom( subPredBuf );
    }

    if (cpyResi)
    {
      getResiBuf(clippedArea).copyFrom(subResiBuf);
    }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    else
    {
      getResiBuf(clippedArea).copyFrom(subStruct.getResiBuf(clippedArea), true);
    }
#endif

    if( cpyReco )
    {
      getRecoBuf( clippedArea ).copyFrom( subRecoBuf );
    }

    if( cpyOrgResi )
    {
      getOrgResiBuf( clippedArea ).copyFrom( subStruct.getOrgResiBuf( clippedArea ) );
    }
  }
#if NNVC_USE_PRED
  getPredBufCustom                 ( clippedArea ).copyFrom( subStruct.getPredBuf( clippedArea ) );
  getPredBuf                       ( clippedArea ).copyFrom( subStruct.getPredBuf( clippedArea ) );
#endif

  if( cpyPred )
  {
    picture->getPredBuf( clippedArea ).copyFrom( subPredBuf );
  }

  if (cpyResi)
  {
    picture->getResiBuf(clippedArea).copyFrom(subResiBuf);
  }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  else
  {
    picture->getResiBuf(clippedArea).copyFrom(subStruct.getResiBuf(clippedArea), true);
  }
#endif

#if JVET_Z0118_GDR
  if (isInGdrIntervalOrRecoveryPoc())
  {
    if (cpyReco)
    {
      updateReconMotIPM(clippedArea, subRecoBuf); // xcomrpessCU - need
    }
  }
  else
  {
    if (cpyReco)
    {
      picture->getRecoBuf(clippedArea).copyFrom(subRecoBuf);
    }
  }
#else
  if (cpyReco)
  {
    picture->getRecoBuf(clippedArea).copyFrom(subRecoBuf);
  }
#endif  

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AB0061_ITMP_BV_FOR_IBC
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool processingIntraRegion = ( slice->getSeparateTreeEnabled() && slice->getProcessingIntraRegion() ) ? true : false;
  if( !subStruct.m_isTuEnc && (((!slice->isIntra()&& !processingIntraRegion ) || slice->getUseIBC() || slice->getSPS()->getUseIntraTMP()) && chType != CHANNEL_TYPE_CHROMA) )
#else
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getUseIBC() || slice->getSPS()->getUseIntraTMP()) && chType != CHANNEL_TYPE_CHROMA))
#endif
#else
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getUseIBC()) && chType != CHANNEL_TYPE_CHROMA))
#endif
#else
#if JVET_AB0061_ITMP_BV_FOR_IBC
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getSPS()->getIBCFlag() || slice->getSPS()->getUseIntraTMP()) && chType != CHANNEL_TYPE_CHROMA))
#else
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && chType != CHANNEL_TYPE_CHROMA))
#endif
#endif
  {
    // copy motion buffer
    MotionBuf ownMB  = getMotionBuf          ( clippedArea );
    CMotionBuf subMB = subStruct.getMotionBuf( clippedArea );

    ownMB.copyFrom( subMB );

    motionLut = subStruct.motionLut;
  }
#if JVET_AD0188_CCP_MERGE
  ccpLut = subStruct.ccpLut;
#endif
#if JVET_AG0058_EIP
  if ((slice->isIntra() || (!slice->isIntra() && !subStruct.m_isTuEnc)) && chType != CHANNEL_TYPE_CHROMA)
  {
    eipLut = subStruct.eipLut;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (!subStruct.m_isTuEnc && chType != CHANNEL_TYPE_CHROMA)
  {
    IpmBuf ownIB  = getIpmBuf          ( clippedArea );
    CIpmBuf subIB = subStruct.getIpmBuf( clippedArea );
    ownIB.copyFrom( subIB );
  }
#endif
  prevPLT = subStruct.prevPLT;


  if ( updateCost )
  {
    fracBits += subStruct.fracBits;
    dist     += subStruct.dist;
    cost     += subStruct.cost;
    costDbOffset += subStruct.costDbOffset;
  }
  if( parent )
  {
    // allow this to be false at the top level
    CHECKD( !area.contains( subArea ), "Trying to use a sub-structure not contained in self" );
  }

  // copy the CUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &pcu : subStruct.cus )
    {
      // add an analogue CU into own CU store
      const UnitArea &cuPatch = *pcu;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      CodingUnit &cu = addCU( cuPatch, pcu->chType );
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      CodingUnit &cu = addCU( cuPatch, pcu->chType );
#else
      CodingUnit &cu = addCU(cuPatch, chType);
#endif
#endif

      // copy the CU info from subPatch
      cu = *pcu;
    }
  }

  // copy the PUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &ppu : subStruct.pus )
    {
      // add an analogue PU into own PU store
      const UnitArea &puPatch = *ppu;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      PredictionUnit &pu = addPU( puPatch, ppu->chType );
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      PredictionUnit &pu = addPU( puPatch, ppu->chType );
#else
      PredictionUnit &pu = addPU(puPatch, chType);
#endif
#endif
      // copy the PU info from subPatch
      pu = *ppu;
    }
  }
  // copy the TUs over
  for( const auto &ptu : subStruct.tus )
  {
    // add an analogue TU into own TU store
    const UnitArea &tuPatch = *ptu;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    TransformUnit &tu = addTU( tuPatch, ptu->chType );
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    TransformUnit &tu = addTU( tuPatch, ptu->chType );
#else
    TransformUnit &tu = addTU(tuPatch, chType);
#endif
#endif
    // copy the TU info from subPatch
    tu = *ptu;
  }
}

void CodingStructure::copyStructure( const CodingStructure& other, const ChannelType chType, const bool copyTUs, const bool copyRecoBuf )
{
  fracBits = other.fracBits;
  dist     = other.dist;
  cost     = other.cost;
  costDbOffset = other.costDbOffset;
  CHECKD( area != other.area, "Incompatible sizes" );

  const UnitArea dualITreeArea = CS::getArea( *this, this->area, chType );

#if JVET_Z0118_GDR
  m_pt = other.m_pt;
#endif

  // copy the CUs over
  for (const auto &pcu : other.cus)
  {
    if( !dualITreeArea.contains( *pcu ) )
    {
      continue;
    }
    // add an analogue CU into own CU store
    const UnitArea &cuPatch = *pcu;

    CodingUnit &cu = addCU(cuPatch, pcu->chType);

    // copy the CU info from subPatch
    cu = *pcu;
  }

  // copy the PUs over
  for (const auto &ppu : other.pus)
  {
    if( !dualITreeArea.contains( *ppu ) )
    {
      continue;
    }
    // add an analogue PU into own PU store
    const UnitArea &puPatch = *ppu;

    PredictionUnit &pu = addPU(puPatch, ppu->chType);
    // copy the PU info from subPatch
    pu = *ppu;
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AB0061_ITMP_BV_FOR_IBC
  if (!other.slice->isIntra() || other.slice->getUseIBC() || other.slice->getSPS()->getUseIntraTMP() )
#else
  if (!other.slice->isIntra() || other.slice->getUseIBC())
#endif
#else
#if JVET_AB0061_ITMP_BV_FOR_IBC
  if (!other.slice->isIntra() || other.slice->getSPS()->getIBCFlag() || other.slice->getSPS()->getUseIntraTMP() )
#else
  if (!other.slice->isIntra() || other.slice->getSPS()->getIBCFlag())
#endif
#endif
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf();
    CMotionBuf subMB = other.getMotionBuf();

    ownMB.copyFrom( subMB );

    motionLut = other.motionLut;
  }

#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
  splitPredLuma = other.splitPredLuma;
#endif
#if JVET_AD0188_CCP_MERGE
  ccpLut = other.ccpLut;
#endif
#if JVET_AE0043_CCP_MERGE_TEMPORAL
  CCPModelIdxBuf  ownIdxB = getCcpmIdxBuf();
  CCCPModelIdxBuf subIdxB = other.getCcpmIdxBuf();
  ownIdxB.copyFrom( subIdxB );
  m_ccpModelLUT = other.m_ccpModelLUT;
#endif
#if JVET_AG0058_EIP
  eipLut = other.eipLut;
#endif
#if JVET_AG0058_EIP
  EipModelIdxBuf ownEipIdxB = getEipIdxBuf();
  CEipModelIdxBuf subEipIdxB = other.getEipIdxBuf();
  ownEipIdxB.copyFrom(subEipIdxB);
  m_eipModelLUT = other.m_eipModelLUT;
#endif

#if JVET_W0123_TIMD_FUSION
  IpmBuf  ownIB = getIpmBuf();
  CIpmBuf subIB = other.getIpmBuf();
  ownIB.copyFrom( subIB );
#endif
  prevPLT = other.prevPLT;

  if( copyTUs )
  {
    // copy the TUs over
    for( const auto &ptu : other.tus )
    {
      if( !dualITreeArea.contains( *ptu ) )
      {
        continue;
      }
      // add an analogue TU into own TU store
      const UnitArea &tuPatch = *ptu;
      TransformUnit &tu = addTU( tuPatch, ptu->chType );
      // copy the TU info from subPatch
      tu = *ptu;
    }
  }

  if( copyRecoBuf )
  {
    CPelUnitBuf recoBuf = other.getRecoBuf( area );

    if( parent )
    {
      // copy data to self for neighbors
      getRecoBuf( area ).copyFrom( recoBuf );
    }

    // copy data to picture
#if JVET_Z0118_GDR
    if (isInGdrIntervalOrRecoveryPoc())
    {
      updateReconMotIPM(area, recoBuf); // xcomrpessCU - need
    }
    else
    {
      picture->getRecoBuf(area).copyFrom(recoBuf);
    }
#else
    picture->getRecoBuf(area).copyFrom(recoBuf);
#endif
    
    if (other.pcv->isEncoder)
    {
      CPelUnitBuf predBuf = other.getPredBuf(area);
      if (parent)
      {
        getPredBuf(area).copyFrom(predBuf);
      }
      picture->getPredBuf(area).copyFrom(predBuf);
    }

    // required for DebugCTU
    int numCh = ::getNumberValidChannels( area.chromaFormat );
    for( int i = 0; i < numCh; i++ )
    {
      const size_t _area = unitScale[i].scaleArea( area.blocks[i].area() );

      memcpy( m_isDecomp[i], other.m_isDecomp[i], sizeof( *m_isDecomp[0] ) * _area );
    }
  }
}

void CodingStructure::initStructData( const int &QP, const bool &skipMotBuf )
{
  clearPUs();
  clearTUs();
  clearCUs();

  if( QP < MAX_INT )
  {
    currQP[0] = currQP[1] = QP;
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AB0061_ITMP_BV_FOR_IBC
  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getUseIBC() || slice->getSPS()->getUseIntraTMP()) && !m_isTuEnc)))
#else
  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getUseIBC()) && !m_isTuEnc)))
#endif
#else
#if JVET_AB0061_ITMP_BV_FOR_IBC
  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getSPS()->getIBCFlag() || slice->getSPS()->getUseIntraTMP()) && !m_isTuEnc)))
#else
  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && !m_isTuEnc)))
#endif
#endif
  {
#if JVET_Z0118_GDR
  getMotionBuf(PIC_RECONSTRUCTION_0).memset(0);
  if (m_gdrEnabled)
  {
    getMotionBuf(PIC_RECONSTRUCTION_1).memset(0);
  }
#else
    getMotionBuf().memset(0);
#endif
  }

#if JVET_AH0135_TEMPORAL_PARTITIONING
  getQTDepthBuf().memset(0);
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_Z0118_GDR
  getIpmBuf(PIC_RECONSTRUCTION_0).memset(0);
  if (m_gdrEnabled)
  {
    getIpmBuf(PIC_RECONSTRUCTION_1).memset(0);
  }
#else
  getIpmBuf().memset(0);
#endif
#endif

  fracBits = 0;
  dist     = 0;
  cost     = MAX_DOUBLE;
  lumaCost = MAX_DOUBLE;
  costDbOffset = 0;
  useDbCost = false;
  interHad = std::numeric_limits<Distortion>::max();

#if JVET_Z0118_GDR
  m_pt = PIC_RECONSTRUCTION_0;
#endif

#if MULTI_HYP_PRED
  m_meResults.clear();
#endif
}


void CodingStructure::clearTUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    size_t _area = ( area.blocks[i].area() >> unitScale[i].area );

    memset( m_isDecomp[i], false, sizeof( *m_isDecomp[0] ) * _area );
    memset( m_tuIdx   [i],     0, sizeof( *m_tuIdx   [0] ) * _area );
  }

  numCh = getNumberValidComponents( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    m_offsets[i] = 0;
  }

  for( auto &pcu : cus )
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }

  m_tuCache.cache( tus );
  m_numTUs = 0;
}

void CodingStructure::clearPUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_puIdx[i], 0, sizeof( *m_puIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_puCache.cache( pus );
  m_numPUs = 0;

  for( auto &pcu : cus )
  {
    pcu->firstPU = pcu->lastPU = nullptr;
  }
}

void CodingStructure::clearCUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_cuIdx[i], 0, sizeof( *m_cuIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_cuCache.cache( cus );
  m_numCUs = 0;
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
#if JVET_Z0118_GDR
  return getMotionBuf(_area, m_pt);
#else
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
#endif
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
#if JVET_Z0118_GDR
  return getMotionBuf(_area, m_pt);
#else
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
#endif
}

#if JVET_AH0135_TEMPORAL_PARTITIONING
void CodingStructure::setSplitPred()
{
  const Picture* pic = picture->unscaledPic;
  const int width = pic->getPicWidthInLumaSamples();
  const int height = pic->getPicHeightInLumaSamples();
  Position pos;
  CodingUnit* cuColAll = NULL;
  QTDepthBuf mb; 
  SplitPred sp;

  int offset = slice->getPPS()->getCtuSize() >> 1;
  picture->maxTemporalBtDepth = slice->getPicHeader()->getMaxMTTHierarchyDepth(slice->getSliceType(), CHANNEL_TYPE_LUMA);
  const uint32_t roundVal = ((slice->getPPS()->getCtuSize() >> 2) * (slice->getPPS()->getCtuSize() >> 2))
   * (slice->getPPS()->getCtuSize()  >> LOG2__BLOCK_RESOLUTION) * (slice->getPPS()->getCtuSize() >> LOG2__BLOCK_RESOLUTION);

  uint32_t sumQtdetphColArea = 0;
  uint32_t nbSamples = 0;
  uint32_t nbSamplesMTT = 0;

  const uint8_t nbPos = (offset << 1) >> LOG2__BLOCK_RESOLUTION;
  bool parsedPositions[256 >> LOG2__BLOCK_RESOLUTION][256 >> LOG2__BLOCK_RESOLUTION];
  for (int h = 0; h < height; h = h + QTBTTT_TEMPO_PRED_BUFFER_SIZE)
  {
    for (int w = 0; w < width; w = w + QTBTTT_TEMPO_PRED_BUFFER_SIZE)
    {
      sumQtdetphColArea = 0;
      nbSamples = 0;

      uint8_t maxBtdetphColArea = 0;
      uint32_t sumMttdetphColArea = 0;
      nbSamplesMTT = 0;
      uint8_t minQtdetphColArea = UINT8_MAX;

      pos = Position{ PosType(w), PosType(h) };

      int topLeftWindowx = pos.x - offset;
      int topLeftWindowy = pos.y - offset;
      int bottomRightWindowx = pos.x + offset;
      int bottomRightWindowy = pos.y + offset;
      for (int i = 0; i < nbPos; i++)
      {
        for (int j = 0; j < nbPos; j++)
        {
          parsedPositions[i][j] = false;
        }
      }

      if ((topLeftWindowx < 0)
        || (topLeftWindowy < 0)
        || (bottomRightWindowx >= width)
        || (bottomRightWindowy >= height)
        )
      {
        for (int i = 0; i < nbPos; i++)
        {
          for (int j = 0; j < nbPos; j++)
          {
            if (topLeftWindowx + i * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION < 0)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowy + j * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION < 0)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowx + i * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION >= width)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowy + j * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION >= height)
            {
              parsedPositions[i][j] = true;
            }
          }
        }
      }

      for (int x = 0; x < 0 + (offset << 1); x = x + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION)
      {
        for (int y = 0; y < 0 + (offset << 1); y = y + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION)
        {
          if (!parsedPositions[x >> LOG2__BLOCK_RESOLUTION][y >> LOG2__BLOCK_RESOLUTION])
          {
            cuColAll = getCUTempo(pos.offset(x - offset, y - offset), CHANNEL_TYPE_LUMA);
                   
            int widthBlockRes  = (cuColAll->lumaPos().x - (pos.x + x - offset) + cuColAll->lwidth() + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION - 1 ) >> LOG2__BLOCK_RESOLUTION;
            int heightBlockRes = (cuColAll->lumaPos().y - (pos.y + y - offset) + cuColAll->lheight() + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION - 1) >> LOG2__BLOCK_RESOLUTION;

            if (((x >> LOG2__BLOCK_RESOLUTION) + widthBlockRes > ((offset << 1) >> LOG2__BLOCK_RESOLUTION)))
            {
              widthBlockRes = ((offset << 1) >> LOG2__BLOCK_RESOLUTION) - (x >> LOG2__BLOCK_RESOLUTION);
            }
            if(((y >> LOG2__BLOCK_RESOLUTION) + heightBlockRes > ((offset << 1) >> LOG2__BLOCK_RESOLUTION)))
            {
              heightBlockRes = ((offset << 1) >> LOG2__BLOCK_RESOLUTION) - (y >> LOG2__BLOCK_RESOLUTION);
            }
                
            for (int i = x >> LOG2__BLOCK_RESOLUTION; i < (x >> LOG2__BLOCK_RESOLUTION)  + widthBlockRes; i++)
            {
              for (int j = y >> LOG2__BLOCK_RESOLUTION; j < (y >> LOG2__BLOCK_RESOLUTION) + heightBlockRes; j++)
              {
                parsedPositions[i][j] = true;
              }
            }

            int sizeBlock = (cuColAll->lheight() ) * (cuColAll->lwidth() );
            sumQtdetphColArea += cuColAll->qtDepth * ((roundVal * (heightBlockRes * widthBlockRes) )/ sizeBlock);
            nbSamples += ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);

            if (cuColAll->mtDepth - cuColAll->mtImplicitDepth > maxBtdetphColArea)
            {
              maxBtdetphColArea = cuColAll->mtDepth - cuColAll->mtImplicitDepth;
            }

            if (cuColAll->mtImplicitDepth == 0)
            {
              sumMttdetphColArea += (cuColAll->mtDepth - cuColAll->mtImplicitDepth) * ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);
              nbSamplesMTT += ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);
            }

            if (cuColAll->qtDepth < minQtdetphColArea)
            {
              minQtdetphColArea = cuColAll->qtDepth;
            }
          }
        }
      }
      if (nbSamples > 0)
      {
        sumQtdetphColArea = (sumQtdetphColArea + (nbSamples >> 1)) / nbSamples;
      }
      sp.qtDetphCol = (uint8_t)sumQtdetphColArea;
      sp.maxBtDetphCol = maxBtdetphColArea;

      if (nbSamplesMTT > 0)
      {
        sumMttdetphColArea = (sumMttdetphColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
      }
      sp.mttDetphCol = (uint8_t)sumMttdetphColArea;

      sp.minqtDetphCol = minQtdetphColArea;

      mb = getQtDepthBuf(Area(pos.getX(), pos.getY(), std::min(width - w, QTBTTT_TEMPO_PRED_BUFFER_SIZE), std::min(height - h, QTBTTT_TEMPO_PRED_BUFFER_SIZE)));
      mb.fill(sp);
    }
  }
}

QTDepthBuf CodingStructure::getQtDepthBuf(const Area& _area)
{
  const CompArea& _luma = area.Y();
  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return QTDepthBuf(currQtDepthBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

#endif 

#if JVET_W0123_TIMD_FUSION && RPR_ENABLE
bool  CodingStructure::picContain(const Position _pos)
{
#if JVET_Z0118_GDR
  if (!picture)
  {
    return false;
  }
#endif

  const Picture* pic = picture->unscaledPic;
  const int width   = pic->getPicWidthInLumaSamples();
  const int height  = pic->getPicHeightInLumaSamples();
  return (_pos.x >= 0) && (_pos.x < width) && (_pos.y >= 0) && (_pos.y < height);
}
#endif

MotionInfo& CodingStructure::getMotionInfo( const Position& pos )
{
#if JVET_Z0118_GDR
  return getMotionInfo(pos, m_pt);
#else
#if JVET_W0123_TIMD_FUSION && RPR_ENABLE
  CHECKD( !picContain( pos ), "Trying to access motion information outside of this coding structure");
#else
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );
#endif

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
#endif
}

const MotionInfo& CodingStructure::getMotionInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  return getMotionInfo(pos, m_pt);;
#else
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
#endif
}
#if JVET_AH0135_TEMPORAL_PARTITIONING
SplitPred& CodingStructure::getQtDepthInfo(const Position& pos)
{
  CHECKD(!area.Y().contains(pos), "Trying to access TEMPORAL partitionning information outside of this coding structure");
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos = g_miScaling.scale(pos - area.lumaPos());

  return *(currQtDepthBuf + miPos.y * stride + miPos.x);
}
#endif 
#if JVET_Z0118_GDR
MotionBuf CodingStructure::getMotionBuf(const Area& _area, PictureType pt)
{
  const CompArea& _luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return MotionBuf(((pt == PIC_RECONSTRUCTION_0) ? m_motionBuf0 : m_motionBuf1) + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

const CMotionBuf CodingStructure::getMotionBuf(const Area& _area, PictureType pt) const
{
  const CompArea& _luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return MotionBuf(((pt == PIC_RECONSTRUCTION_0) ? m_motionBuf0 : m_motionBuf1) + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

MotionInfo& CodingStructure::getMotionInfo(const Position& pos, PictureType pt)
{
  CHECKD(!area.Y().contains(pos), "Trying to access motion information outside of this coding structure");

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos = g_miScaling.scale(pos - area.lumaPos());

  return *(((pt == PIC_RECONSTRUCTION_0) ? m_motionBuf0 : m_motionBuf1) + miPos.y * stride + miPos.x);
}

const MotionInfo& CodingStructure::getMotionInfo(const Position& pos, PictureType pt) const
{
  CHECKD(!area.Y().contains(pos), "Trying to access motion information outside of this coding structure");

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos = g_miScaling.scale(pos - area.lumaPos());

  return *(((pt == PIC_RECONSTRUCTION_0) ? m_motionBuf0 : m_motionBuf1) + miPos.y * stride + miPos.x);
}
#endif

#if JVET_AE0043_CCP_MERGE_TEMPORAL
CCPModelIdxBuf CodingStructure::getCcpmIdxBuf( const Area& bufArea)
{
  const CompArea& chromaArea = area.Cb();
  CHECK( !chromaArea.contains( bufArea ), "Trying to access CC model information outside of this coding structure");

  int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  UnitScale scaling(MIN_CU_LOG2 - lumaScaleX, MIN_CU_LOG2 - lumaScaleY);
  const Area miArea   = scaling.scale( bufArea );
  const Area selfArea = scaling.scale( chromaArea );
  
  return CCPModelIdxBuf( m_ccpmIdxBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

const CCCPModelIdxBuf CodingStructure::getCcpmIdxBuf( const Area& bufArea ) const
{
  const CompArea& chromaArea = area.Cb();
  CHECK( !chromaArea.contains( bufArea ), "Trying to access CC model information outside of this coding structure");

  int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  UnitScale scaling(MIN_CU_LOG2 - lumaScaleX, MIN_CU_LOG2 - lumaScaleY);
  const Area miArea   = scaling.scale( bufArea );
  const Area selfArea = scaling.scale( chromaArea );
  
  return CCCPModelIdxBuf( m_ccpmIdxBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

int& CodingStructure::getCcpmIdxInfo( const Position& pos )
{
  CHECK( !area.Cb().contains( pos ), "Trying to access CC model information outside of this coding structure");

  int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  UnitScale scaling(MIN_CU_LOG2 - lumaScaleX, MIN_CU_LOG2 - lumaScaleY);
  const unsigned stride = scaling.scaleHor(area.Cb().width);
  const Position miPos = scaling.scale(pos - area.chromaPos());

  return *( m_ccpmIdxBuf + miPos.y * stride + miPos.x );
}

const int& CodingStructure::getCcpmIdxInfo( const Position& pos ) const
{
  CHECK( !area.Cb().contains( pos ), "Trying to access CC model information outside of this coding structure");

  int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );
  int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, sps->getChromaFormatIdc() );

  UnitScale scaling(MIN_CU_LOG2 - lumaScaleX, MIN_CU_LOG2 - lumaScaleY);
  const unsigned stride = scaling.scaleHor(area.Cb().width);
  const Position miPos = scaling.scale(pos - area.chromaPos());

  return *( m_ccpmIdxBuf + miPos.y * stride + miPos.x );
}
#endif

#if JVET_AG0058_EIP
EipModelIdxBuf CodingStructure::getEipIdxBuf( const Area& bufArea)
{
  const CompArea& lumaArea = area.Y();
  CHECK( !lumaArea.contains( bufArea ), "Trying to access Eip model information outside of this coding structure");
  const Area miArea   = g_miScaling.scale( bufArea );
  const Area selfArea = g_miScaling.scale( lumaArea );
  
  return EipModelIdxBuf( m_eipIdxBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

const CEipModelIdxBuf CodingStructure::getEipIdxBuf( const Area& bufArea ) const
{
  const CompArea& lumaArea = area.Y();
  CHECK( !lumaArea.contains( bufArea ), "Trying to access Eip model information outside of this coding structure");
  const Area miArea   = g_miScaling.scale( bufArea );
  const Area selfArea = g_miScaling.scale( lumaArea );
  
  return EipModelIdxBuf( m_eipIdxBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

int& CodingStructure::getEipIdxInfo( const Position& pos )
{
  CHECK( !area.Y().contains( pos ), "Trying to access Eip model information outside of this coding structure");

  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_eipIdxBuf + miPos.y * stride + miPos.x );
}

const int& CodingStructure::getEipIdxInfo( const Position& pos ) const
{
  CHECK( !area.Y().contains( pos ), "Trying to access Eip model information outside of this coding structure");

  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_eipIdxBuf + miPos.y * stride + miPos.x );
}
#endif

#if JVET_W0123_TIMD_FUSION
IpmBuf CodingStructure::getIpmBuf( const Area& _area )
{
#if JVET_Z0118_GDR
  return getIpmBuf(_area, m_pt);
#else
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return IpmBuf( m_ipmBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
#endif
}

const CIpmBuf CodingStructure::getIpmBuf( const Area& _area ) const
{
#if JVET_Z0118_GDR
  return getIpmBuf(_area, m_pt);
#else
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return IpmBuf( m_ipmBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
#endif
}

uint8_t& CodingStructure::getIpmInfo( const Position& pos )
{
#if JVET_Z0118_GDR
  return getIpmInfo(pos, m_pt);
#else
#if RPR_ENABLE
  CHECKD( !picContain( pos ), "Trying to access motion information outside of this coding structure");
#else
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );
#endif

  //return getIpmBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the intra prediction mode buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_ipmBuf + miPos.y * stride + miPos.x );
#endif
}

const uint8_t& CodingStructure::getIpmInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  return getIpmInfo(pos, m_pt);
#else
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getIpmBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the intra prediction mode buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_ipmBuf + miPos.y * stride + miPos.x );
#endif
}
#endif

#if JVET_W0123_TIMD_FUSION && JVET_Z0118_GDR
IpmBuf CodingStructure::getIpmBuf(const Area& _area, PictureType pt)
{
  const CompArea& _luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return IpmBuf(((pt == PIC_RECONSTRUCTION_0) ? m_ipmBuf0 : m_ipmBuf1) + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

const CIpmBuf CodingStructure::getIpmBuf(const Area& _area, PictureType pt) const
{
  const CompArea& _luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return IpmBuf(((pt == PIC_RECONSTRUCTION_0) ? m_ipmBuf0 : m_ipmBuf1) + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

uint8_t& CodingStructure::getIpmInfo(const Position& pos, PictureType pt)
{
#if JVET_Z0118_GDR
  static uint8_t constIpm = 0;
  
  if (!picContain(pos))
  {
    return constIpm;
  }
#endif

#if RPR_ENABLE
  CHECKD( !picContain( pos ), "Trying to access motion information outside of this coding structure");
#else
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );
#endif

  //return getIpmBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the intra prediction mode buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *(((pt == PIC_RECONSTRUCTION_0) ? m_ipmBuf0 : m_ipmBuf1) + miPos.y * stride + miPos.x);
}

const uint8_t& CodingStructure::getIpmInfo(const Position& pos, PictureType pt) const
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getIpmBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the intra prediction mode buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *(((pt == PIC_RECONSTRUCTION_0) ? m_ipmBuf0 : m_ipmBuf1) + miPos.y * stride + miPos.x);
}
#endif


// data accessors
       PelBuf     CodingStructure::getPredBuf(const CompArea &blk)           { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     CodingStructure::getPredBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)          { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_PREDICTION); }

#if NNVC_USE_PRED
       PelBuf     CodingStructure::getPredBufCustom(const CompArea &blk)           { return getBuf(blk,  PIC_PREDICTION_CUSTOM); }
const CPelBuf     CodingStructure::getPredBufCustom(const CompArea &blk)     const { return getBuf(blk,  PIC_PREDICTION_CUSTOM); }
       PelUnitBuf CodingStructure::getPredBufCustom(const UnitArea &unit)          { return getBuf(unit, PIC_PREDICTION_CUSTOM); }
const CPelUnitBuf CodingStructure::getPredBufCustom(const UnitArea &unit)    const { return getBuf(unit, PIC_PREDICTION_CUSTOM); }
#endif

       PelBuf     CodingStructure::getResiBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     CodingStructure::getResiBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RESIDUAL); }

#if JVET_Z0118_GDR
       PelBuf     CodingStructure::getRecoBuf(const CompArea &blk)           { return getBuf(blk, m_pt);  }
const CPelBuf     CodingStructure::getRecoBuf(const CompArea &blk)     const { return getBuf(blk, m_pt);  }
       PelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)          { return getBuf(unit, m_pt); }
const CPelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)    const { return getBuf(unit, m_pt); }
#else
       PelBuf     CodingStructure::getRecoBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RECONSTRUCTION); }
const CPelBuf     CodingStructure::getRecoBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RECONSTRUCTION); }
       PelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RECONSTRUCTION); }
const CPelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RECONSTRUCTION); }
#endif
       PelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORG_RESI); }
const CPelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORG_RESI); }
       PelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORG_RESI); }
const CPelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORG_RESI); }

       PelBuf     CodingStructure::getOrgBuf(const CompArea &blk)            { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const CompArea &blk)      const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)           { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)     const { return getBuf(unit, PIC_ORIGINAL); }

       PelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)      { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)const { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf()                               { return picture->getBuf(area, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf()                         const { return picture->getBuf(area, PIC_ORIGINAL); }
#if ALF_SAO_TRUE_ORG
       PelUnitBuf CodingStructure::getTrueOrgBuf()                           { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }
const CPelUnitBuf CodingStructure::getTrueOrgBuf()                     const { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }
#endif

PelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type )
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;
  
#if JVET_Z0118_GDR
  PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : (type == PIC_RECONSTRUCTION_0 ? &m_reco0 : (type == PIC_RECONSTRUCTION_1 ? &m_reco1 : (type == PIC_ORG_RESI ? &m_orgr : nullptr ))));
#else
  PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );
#endif  
#if NNVC_USE_PRED
  if (type == PIC_PREDICTION_CUSTOM)
  {
    buf = &m_predCustom;
  }
#endif

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  if (!parent && (type == PIC_PREDICTION))
#else
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
#endif
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

const CPelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;
#if JVET_Z0118_GDR
  const PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : (type == PIC_RECONSTRUCTION_0 ? &m_reco0 : (type == PIC_RECONSTRUCTION_1 ? &m_reco1 : (type == PIC_ORG_RESI ? &m_orgr : nullptr ))));
#else
  const PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );
#endif
#if NNVC_USE_PRED
  if (type == PIC_PREDICTION_CUSTOM)
  {
    buf = &m_predCustom;
  }
#endif

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  if (!parent && (type == PIC_PREDICTION))
#else
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
#endif
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

PelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type )
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}



const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const CodingUnit& curCu, const ChannelType _chType ) const
{
  const CodingUnit* cu = getCU( pos, _chType );
  // exists       same slice and tile                  cu precedes curCu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curCu.slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curCu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curCu.chromaFormat );
  int xCurr = curCu.blocks[_chType].x << getChannelTypeScaleX( _chType, curCu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;

  if( cu && CU::isSameSliceAndTile( *cu, curCu ) && ( cu->cs != curCu.cs || cu->idx <= curCu.idx ) && addCheck)
  {
#if JVET_Z0118_GDR
    if (m_gdrEnabled)
    {
      const Position posRB = (_chType == CHANNEL_TYPE_LUMA) ? curCu.Y().bottomRight() : curCu.Cb().bottomRight();
      const Position posTL = (_chType == CHANNEL_TYPE_LUMA) ? curCu.Y().topLeft() : curCu.Cb().topLeft();
      bool isTLClean = isClean(posTL, _chType);
      bool isRBClean = isClean(posRB, _chType);
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean(pos, _chType);

      if (isSrcClean && !isTarClean)
      {
        return nullptr;
      }
    }
#endif
    return cu;
  }
  else
  {
    return nullptr;
  }
}

const CodingUnit* CodingStructure::getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const
{
  const CodingUnit* cu = getCU(pos, _chType);
  const bool wavefrontsEnabled = this->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(this->sps->getMaxCUWidth());
  int xNbY = pos.x << getChannelTypeScaleX(_chType, this->area.chromaFormat);
  int xCurr = curPos.x << getChannelTypeScaleX(_chType, this->area.chromaFormat);
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1) ? false : true;

#if JVET_Z0118_GDR
  if (m_gdrEnabled)
  {
    bool isSrcClean = isClean(curPos, _chType);
    bool isTarClean = isClean(pos, _chType);

    if (isSrcClean && !isTarClean)
    {
      return nullptr;
    }
  }
#endif

  return ( cu && cu->slice->getIndependentSliceIdx() == curSliceIdx && cu->tileIdx == curTileIdx && addCheck ) ? cu : nullptr;
}

const PredictionUnit* CodingStructure::getPURestricted( const Position &pos, const PredictionUnit& curPu, const ChannelType _chType ) const
{
  const PredictionUnit* pu = getPU( pos, _chType );
  // exists       same slice and tile                  pu precedes curPu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curPu.cu->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curPu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curPu.chromaFormat );
  int xCurr = curPu.blocks[_chType].x << getChannelTypeScaleX( _chType, curPu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  if (pu && CU::isSameSliceAndTile(*pu->cu, *curPu.cu) && (pu->cs != curPu.cs || pu->idx <= curPu.idx) && addCheck)
  {
#if JVET_Z0118_GDR
    if (m_gdrEnabled)
    {
      const Position posRB = (_chType == CHANNEL_TYPE_LUMA) ? curPu.Y().bottomRight() : curPu.Cb().bottomRight();
      const Position posTL = (_chType == CHANNEL_TYPE_LUMA) ? curPu.Y().topLeft() : curPu.Cb().topLeft();
      bool isTLClean = isClean(posTL, _chType);
      bool isRBClean = isClean(posRB, _chType);
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean(pos, _chType);

      if (isSrcClean && !isTarClean)
      {
        return nullptr;
      }
    }
#endif
    return pu;
  }
  else
  {
    return nullptr;
  }
}

const TransformUnit* CodingStructure::getTURestricted( const Position &pos, const TransformUnit& curTu, const ChannelType _chType ) const
{
  const TransformUnit* tu = getTU( pos, _chType );
  // exists       same slice and tile                  tu precedes curTu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  const bool wavefrontsEnabled = curTu.cu->slice->getSPS()->getEntropyCodingSyncEnabledFlag();
  int ctuSizeBit = floorLog2(curTu.cs->sps->getMaxCUWidth());
  int xNbY  = pos.x << getChannelTypeScaleX( _chType, curTu.chromaFormat );
  int xCurr = curTu.blocks[_chType].x << getChannelTypeScaleX( _chType, curTu.chromaFormat );
  bool addCheck = (wavefrontsEnabled && (xNbY >> ctuSizeBit) >= (xCurr >> ctuSizeBit) + 1 ) ? false : true;
  if (tu && CU::isSameSliceAndTile(*tu->cu, *curTu.cu) && (tu->cs != curTu.cs || tu->idx <= curTu.idx) && addCheck)
  {
#if JVET_Z0118_GDR
    if (m_gdrEnabled)
    {
      const Position posRB = (_chType == CHANNEL_TYPE_LUMA) ? curTu.Y().bottomRight() : curTu.Cb().bottomRight();
      const Position posTL = (_chType == CHANNEL_TYPE_LUMA) ? curTu.Y().topLeft() : curTu.Cb().topLeft();
      bool isTLClean = isClean(posTL, _chType);
      bool isRBClean = isClean(posRB, _chType);
      bool isSrcClean = isTLClean || isRBClean;
      bool isTarClean = isClean(pos, _chType);

      if (isSrcClean && !isTarClean)
      {
        return nullptr;
      }
    }
#endif
    return tu;
  }
  else
  {
    return nullptr;
  }
}


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CodingStructure::copyLumaPointers( CodingStructure& cs )
{
  cs.m_lumaCUs       = m_lumaCUs;      
  cs.m_lumaPUs       = m_lumaPUs;      
  cs.m_lumaTUs       = m_lumaTUs;      
  cs.m_lumaUnitScale = m_lumaUnitScale;
  cs.m_lumaArea      = m_lumaArea;     
  cs.m_lumaCuIdx     = m_lumaCuIdx;    
  cs.m_lumaPuIdx     = m_lumaPuIdx;    
  cs.m_lumaTuIdx     = m_lumaTuIdx;    
  cs.m_lumaParent    = m_lumaParent; 
  cs.m_lumaFracBits  = m_lumaFracBits;
  cs.m_lumaDist      = m_lumaDist;
  cs.m_lumaCost      = m_lumaCost;
};

void CodingStructure::setLumaPointers( CodingStructure& cs )
{
  cs.m_lumaCUs       = &(cus);
  cs.m_lumaPUs       = &(pus);
  cs.m_lumaTUs       = &(tus);
  cs.m_lumaUnitScale = &(unitScale[COMPONENT_Y]);
  cs.m_lumaArea      = &(area);
  cs.m_lumaCuIdx     = m_cuIdx[CH_L];
  cs.m_lumaPuIdx     = m_puIdx[CH_L];
  cs.m_lumaTuIdx     = m_tuIdx[CH_L];
  cs.m_lumaParent    = parent;
  cs.m_lumaFracBits  = fracBits;
  cs.m_lumaDist      = dist;
  cs.m_lumaCost      = cost;
};

/*const*/ CodingUnit *CodingStructure::getLumaCU(const Position &pos, const ChannelType effChType) const
{
  const CompArea &_blk = m_lumaArea->blocks[effChType];

  if (!_blk.contains(pos))
  {
    if (m_lumaParent)
    {
      return m_lumaParent->getCU(pos, effChType);   // retrieve using getCU()
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_lumaCuIdx[rsAddr(pos, _blk.pos(), _blk.width, (*m_lumaUnitScale))];
    if (idx != 0)
    {
      return (*m_lumaCUs)[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

const PredictionUnit *CodingStructure::getLumaPU(const Position &pos, const ChannelType effChType) const
{
  const CompArea &_blk = m_lumaArea->blocks[effChType];

  if (!_blk.contains(pos))
  {
    if (m_lumaParent)
    {
      return m_lumaParent->getPU(pos, effChType);   // retrieve using getPU()
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_lumaPuIdx[rsAddr(pos, _blk.pos(), _blk.width, (*m_lumaUnitScale))];
    if (idx != 0)
    {
      return (*m_lumaPUs)[idx - 1];
    }
    else
    {
      return nullptr;
    }
  }
}

const TransformUnit *CodingStructure::getLumaTU(const Position &pos, const ChannelType effChType) const
{
  const CompArea &_blk = m_lumaArea->blocks[effChType];

  if (!_blk.contains(pos))
  {
    if (m_lumaParent)
    {
      return m_lumaParent->getTU(pos, effChType);   // retrieve using getTU()
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    const unsigned idx = m_lumaTuIdx[rsAddr(pos, _blk.pos(), _blk.width, (*m_lumaUnitScale))];
    if (idx != 0)
    {
      return (*m_lumaTUs)[idx - 1];
    }
    else if (m_isTuEnc)
    {
      return m_lumaParent->getTU(pos, effChType);
    }
    else
    {
      return nullptr;
    }
  }
}

void CodingStructure::popLastCU( const PartSplit& implicitSplit )
{
  CodingUnit* cu = cus.back();
  cu->idx        = 0;
  m_cuCache.cache( cu );

  cus.pop_back();
  m_numCUs--;
  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    prevCU->next = nullptr;
  }

  uint32_t idx = 0;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );

  for( uint32_t i = 0; i < numCh && implicitSplit == CU_DONT_SPLIT; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = cu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_cuIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }
}

void CodingStructure::deriveSeparateTreeFlagInference( int& separateTreeFlag, bool& inferredSeparateTreeFlag, int width, int height, bool canSplit )
{
  if (!slice->isIntra())
  {
    if (!canSplit)   // at maximal leaf of partition tree always use shared tree
    {
      inferredSeparateTreeFlag = true;
      separateTreeFlag         = 0;
    }
    else if (g_aucLog2[width] > 6 || g_aucLog2[height] > 6)   // JVET-K0230
    {
      inferredSeparateTreeFlag = true;
      separateTreeFlag         = 0;
    }
    else if ((g_aucLog2[width] + g_aucLog2[height]) >= ID_SEP_TREE_BLK_SIZE_LIMIT1)
    {
      inferredSeparateTreeFlag = true;
      separateTreeFlag         = 1;
    }
    else if ((g_aucLog2[width] + g_aucLog2[height]) <= ID_SEP_TREE_BLK_SIZE_LIMIT2)
    {
      inferredSeparateTreeFlag = true;
      separateTreeFlag         = 0;
    }
    else
    {
      inferredSeparateTreeFlag = false;
    }
  }
  else
  {
    inferredSeparateTreeFlag = true;
    separateTreeFlag         = 1;
  }
}

void CodingStructure::determineIfSeparateTreeFlagInferred(bool &inferredSeparateTreeFlag, int width, int height,
                                                          bool canSplit)
{
  if (!slice->isIntra())
  {
    if (!canSplit)
    {
      inferredSeparateTreeFlag = true;
    }
    else if (g_aucLog2[width] > 6 || g_aucLog2[height] > 6)   // JVET-K0230
    {
      inferredSeparateTreeFlag = true;
    }
    else if ((g_aucLog2[width] + g_aucLog2[height]) >= ID_SEP_TREE_BLK_SIZE_LIMIT1)
    {
      inferredSeparateTreeFlag = true;
    }
    else if ((g_aucLog2[width] + g_aucLog2[height]) <= ID_SEP_TREE_BLK_SIZE_LIMIT2)
    {
      inferredSeparateTreeFlag = true;
    }
    else
    {
      inferredSeparateTreeFlag = false;
    }
  }
  else
  {
    inferredSeparateTreeFlag = true;
  }
}
#endif
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
void CodingStructure::chromaTreePred(CodingStructure& cs, Partitioner& partitioner)
{
  UnitArea curArea = partitioner.currArea();
  uint32_t nbSamplesMTT = 0;
  uint32_t sumMttdetphColArea = 0;
  uint32_t sumBtdetphColArea = 0;
  uint32_t horBlkColArea = 0;
  uint32_t verBlkColArea = 0;

  for (auto &currCU : cs.traverseCUs(curArea, CHANNEL_TYPE_LUMA))
  {
    if (!currCU.Y().valid())
    {
      continue;
    }

    int sizeBlock = currCU.lwidth() * currCU.lheight();

    if (currCU.mtImplicitDepth == 0)
    {
      sumMttdetphColArea += currCU.mtDepth * sizeBlock;
      sumBtdetphColArea += currCU.btDepth * sizeBlock;

      if (currCU.lwidth() > currCU.lheight())
      {
        horBlkColArea += 16 * sizeBlock;
      }
      if (currCU.lheight() > currCU.lwidth())
      {
        verBlkColArea += 16 * sizeBlock;
      }

      nbSamplesMTT += sizeBlock;
    }
  }

  if (nbSamplesMTT > 0)
  {
    sumMttdetphColArea = (sumMttdetphColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
    sumBtdetphColArea = (sumBtdetphColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
    horBlkColArea = (horBlkColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
    verBlkColArea = (verBlkColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
  }
  splitPredLuma.mttDetphCol = (uint8_t)sumMttdetphColArea;
  splitPredLuma.btDetphCol = (uint8_t)sumBtdetphColArea;
  splitPredLuma.horblkCol = (uint8_t)horBlkColArea;
  splitPredLuma.verblkCol = (uint8_t)verBlkColArea;
}
#endif
