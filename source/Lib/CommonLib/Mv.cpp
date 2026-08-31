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

/** \file     Mv.cpp
    \brief    motion vector class
*/

#include "Mv.h"

#include "Common.h"
#include "Slice.h"

const MvPrecision Mv::m_amvrPrecision[4] = { MV_PRECISION_QUARTER, MV_PRECISION_INT, MV_PRECISION_4PEL, MV_PRECISION_HALF }; // for cu.imv=0, 1, 2 and 3
const MvPrecision Mv::m_amvrPrecAffine[3] = { MV_PRECISION_QUARTER, MV_PRECISION_SIXTEENTH, MV_PRECISION_INT }; // for cu.imv=0, 1 and 2
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const MvPrecision Mv::m_amvrPrecIbc[4] = { MV_PRECISION_QUARTER, MV_PRECISION_INT, MV_PRECISION_4PEL, MV_PRECISION_HALF }; // for cu.imv=0, 1, 2 and 3
#else
const MvPrecision Mv::m_amvrPrecIbc[3] = { MV_PRECISION_INT, MV_PRECISION_INT, MV_PRECISION_4PEL }; // for cu.imv=0, 1 and 2
#endif

void roundAffineMv( int& mvx, int& mvy, int nShift )
{
  const int nOffset = 1 << (nShift - 1);
  mvx = (mvx + nOffset - (mvx >= 0)) >> nShift;
  mvy = (mvy + nOffset - (mvy >= 0)) >> nShift;
}

void (*clipMv) ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );

void clipMvInPic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps )
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
#if JVET_AA0096_MC_BOUNDARY_PADDING
  offset += MC_PAD_SIZE;
#endif
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;

  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

void clipMvInSubpic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps )
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;
  const SubPic& curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = ((curSubPic.getSubPicRight() + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.getMaxCUWidth() - offset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << mvShift;

    verMax = ((curSubPic.getSubPicBottom() + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.getMaxCUHeight() - offset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << mvShift;
  }
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
void(*clipMv2) (Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift);

void clipMvInPic2(Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift)
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps, mvShift);
    return;
  }

  int offset = 8;
#if JVET_AA0096_MC_BOUNDARY_PADDING
  offset += MC_PAD_SIZE;
#endif
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;

  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

void clipMvInSubpic2(Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift)
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps, mvShift);
    return;
  }

  int offset = 8;
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;
  const SubPic& curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = ((curSubPic.getSubPicRight() + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.getMaxCUWidth() - offset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << mvShift;

    verMax = ((curSubPic.getSubPicBottom() + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.getMaxCUHeight() - offset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << mvShift;
  }
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

bool wrapClipMv(Mv& rcMv, const Position& pos, const struct Size& size, const SPS *sps, const PPS *pps, const int iMvShift)
{
  bool wrapRef = true;
#else
bool wrapClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const SPS *sps, const PPS *pps )
{
  bool wrapRef = true;
  int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
#endif
  int iOffset = 8;
  int iHorMax = ( pps->getPicWidthInLumaSamples() + sps->getMaxCUWidth() - size.width + iOffset - (int)pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) sps->getMaxCUWidth()                                      - iOffset - ( int ) pos.x + 1 ) << iMvShift;

  int iVerMax = ( pps->getPicHeightInLumaSamples() + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) sps->getMaxCUHeight() - iOffset - ( int ) pos.y + 1 ) << iMvShift;

  const SubPic& curSubPic = pps->getSubPicFromPos( pos );
  if( curSubPic.getTreatedAsPicFlag() )
  {
    iVerMax = ( ( curSubPic.getSubPicBottom() + 1 ) + iOffset - ( int ) pos.y - 1 ) << iMvShift;
    iVerMin = ( -( int ) sps->getMaxCUHeight() - iOffset - ( ( int ) pos.y - curSubPic.getSubPicTop() ) + 1 ) << iMvShift;
  }
  int mvX = rcMv.getHor();

  if(mvX > iHorMax)
  {
    mvX -= ( pps->getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }
  if(mvX < iHorMin)
  {
    mvX += ( pps->getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }

  rcMv.setHor( mvX );
  rcMv.setVer( std::min( iVerMax, std::max( iVerMin, rcMv.getVer() ) ) );
  return wrapRef;
}


#if JVET_AD0140_MVD_PREDICTION
bool MvdSuffixInfoMv::getMergedBinBudgetForMv(const unsigned int curBinBudget)
{
  bool       useMergedBinBudget = true;
  RefPicList rplIdx = REF_PIC_LIST_X;
  constexpr int rplNumber = static_cast<int>(NUM_REF_PIC_LIST_01); //to be updated for MHP ???
  for (int rplCnt = 0; rplCnt < rplNumber; ++rplCnt)
  {
    const int& curActualMvCompNum = actualMvCompNum[rplCnt];
    CHECK(curActualMvCompNum > maxNumMvComp, "actualMvCompNum > maxNumMvComp");

    if (0 >= curActualMvCompNum)
    {
      useMergedBinBudget = false;
    }
    else
    {
      rplIdx = static_cast<RefPicList>(rplCnt);
    }
  };

  if (rplIdx == REF_PIC_LIST_X)
  {
    return false;
  }
  else if (!useMergedBinBudget)
  {
    actualRpl = rplIdx;
    return MvdSuffixInfoMv::getBinBudgetForMv(curBinBudget, rplIdx);
  }


  AuxMvdBins curAuxMvdBins[rplNumber * maxNumMvComp];
  int totalActualMvCompNum = 0;
  for (int rplCnt = 0; rplCnt < rplNumber; ++rplCnt)
  {
    const int& curActualMvCompNum = actualMvCompNum[rplCnt];
    std::memcpy(curAuxMvdBins + totalActualMvCompNum, auxMvdBins[rplCnt], curActualMvCompNum * sizeof(AuxMvdBins));
    totalActualMvCompNum += curActualMvCompNum;
  }

  auto compAuxMvdBins = [this](const AuxMvdBins& entry0, const AuxMvdBins& entry1)
  {
    const int hypBinDiff = entry0.numHypBins - entry1.numHypBins;
    const auto& mvd0Info = mvBins[entry0.rplIdx][entry0.mvIdx];
    const int sign0 = static_cast<int>(entry0.isHor ? mvd0Info.horEncodeSignInEP : mvd0Info.verEncodeSignInEP);
    const auto& mvd1Info = mvBins[entry1.rplIdx][entry1.mvIdx];
    const int sign1 = static_cast<int>(entry1.isHor ? mvd1Info.horEncodeSignInEP : mvd1Info.verEncodeSignInEP);

    if (0 == hypBinDiff)
    {
      return sign0 > sign1;
    }
    else if (std::abs(hypBinDiff) <= MVD_PREDICTION_SIGN_SUFFIX_BIN_THR && entry0.numHypBins > entry1.numHypBins)
    {
      return sign0 < sign1 ? false : true;
    }
    else if (std::abs(hypBinDiff) <= MVD_PREDICTION_SIGN_SUFFIX_BIN_THR && entry0.numHypBins < entry1.numHypBins)
    {
      return sign0 > sign1 ? true : false;
    }

    return entry0.numHypBins > entry1.numHypBins;
  };

  for (int binCnt = 0; binCnt < curBinBudget; ++binCnt)
  {

    int bestIdx = 0;
    for (int entryCnt = 1; entryCnt < totalActualMvCompNum; ++entryCnt)
    {
      if (compAuxMvdBins(curAuxMvdBins[entryCnt], curAuxMvdBins[bestIdx]))
      {
        bestIdx = entryCnt;
      }
    }

    if (0 != bestIdx)
    {
      std::swap(*curAuxMvdBins, curAuxMvdBins[bestIdx]);
    }

    int& mvCompWithMaxBinNum = curAuxMvdBins[0].numHypBins;
    if (mvCompWithMaxBinNum > 0)
    {
      const int& mvIdx = curAuxMvdBins[0].mvIdx;
      const int& rplSelected = curAuxMvdBins[0].rplIdx;

      if (curAuxMvdBins[0].isHor)
      {
        if (mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins < 0) //get a hor sign counted and initialize the number of used suffix bins for a hor component
        {
          mvBins[rplSelected][mvIdx].horEncodeSignInEP = false;
          mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins = 0;
        }
        else
        {
          ++mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins;
        }
      }
      else
      {
        if (mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins < 0) //get a ver sign counted and initialize the number of used suffix bins for a ver component
        {
          mvBins[rplSelected][mvIdx].verEncodeSignInEP = false;
          mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins = 0;
        }
        else
        {
          ++mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins;
        }
      };

      --mvCompWithMaxBinNum;
    }
    else
    {
      break;
    }
    }

  return true;
  }

bool MvdSuffixInfoMv::getBinBudgetForMv(const unsigned int curBinBudget, const RefPicList rplIdx)
{
  const size_t rplSelected        = static_cast<size_t>(rplIdx);
  const int&   curActualMvCompNum = actualMvCompNum[rplSelected];
  if (0 >= curActualMvCompNum)
  {
    return false;
  };
  AuxMvdBins curAuxMvdBins[maxNumMvComp];
  CHECK(rplSelected > 1, "rplSelected > 1");
  CHECK(curActualMvCompNum > maxNumMvComp, "actualMvCompNum > maxNumMvComp");
  std::memcpy(curAuxMvdBins, auxMvdBins[rplSelected], curActualMvCompNum * sizeof(AuxMvdBins));

  auto compAuxMvdBins = [this, rplSelected](const AuxMvdBins& entry0, const AuxMvdBins& entry1)
  {
    const int hypBinDiff = entry0.numHypBins - entry1.numHypBins;
    const auto& mvd0Info = mvBins[rplSelected][entry0.mvIdx];
    const int sign0      = static_cast<int>(entry0.isHor ? mvd0Info.horEncodeSignInEP : mvd0Info.verEncodeSignInEP);
    const auto& mvd1Info = mvBins[rplSelected][entry1.mvIdx];
    const int sign1      = static_cast<int>(entry1.isHor ? mvd1Info.horEncodeSignInEP : mvd1Info.verEncodeSignInEP);

    if (0 == hypBinDiff)
    {
      return sign0 > sign1;
    }
    else if (std::abs(hypBinDiff) <= MVD_PREDICTION_SIGN_SUFFIX_BIN_THR && entry0.numHypBins > entry1.numHypBins)
    {
      return sign0 < sign1 ? false : true;
    }
    else if (std::abs(hypBinDiff) <= MVD_PREDICTION_SIGN_SUFFIX_BIN_THR && entry0.numHypBins < entry1.numHypBins)
    {
      return sign0 > sign1 ? true : false;
    }
    
    return entry0.numHypBins > entry1.numHypBins;
  };

  for (int binCnt = 0; binCnt < curBinBudget; ++binCnt)
  {

    int bestIdx = 0;
    for (int entryCnt = 1; entryCnt < curActualMvCompNum; ++entryCnt)
    {
      if (compAuxMvdBins(curAuxMvdBins[entryCnt], curAuxMvdBins[bestIdx]))
      {
        bestIdx = entryCnt;
      }
    }

    if (0 != bestIdx)
    {
      std::swap(*curAuxMvdBins, curAuxMvdBins[bestIdx]);
    }

    int& mvCompWithMaxBinNum = curAuxMvdBins[0].numHypBins;
    if (mvCompWithMaxBinNum > 0)
    {
      const int& mvIdx = curAuxMvdBins[0].mvIdx;

      if (curAuxMvdBins[0].isHor)
      {
        if (mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins < 0) //get a hor sign counted and initialize the number of used suffix bins for a hor component
        {
          mvBins[rplSelected][mvIdx].horEncodeSignInEP          = false;
          mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins = 0;
        }
        else
        {
          ++mvBins[rplSelected][mvIdx].horOffsetPredictionNumBins;
        }
      }
      else
      {
        if (mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins < 0) //get a ver sign counted and initialize the number of used suffix bins for a ver component
        {
          mvBins[rplSelected][mvIdx].verEncodeSignInEP          = false;
          mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins = 0;
        }
        else
        {
          ++mvBins[rplSelected][mvIdx].verOffsetPredictionNumBins;
        }
      };

      --mvCompWithMaxBinNum;
    }
    else
    {
      break;
    }
  }

  return true;
}

RefPicList MvdSuffixInfoMv::selectRplForMvdCoding(const unsigned int curBinBudget) const
{
  constexpr size_t rpl0Idx   = static_cast<size_t>(REF_PIC_LIST_0);
  constexpr size_t rpl1Idx   = static_cast<size_t>(REF_PIC_LIST_1);
  constexpr size_t rplLength = static_cast<size_t>(NUM_REF_PIC_LIST_01);

  int binSum[rplLength] = {0, 0};
  for (int rplCnt = 0; rplCnt < rplLength; ++rplCnt)
  {
    for (int mvCompCnt = 0; mvCompCnt < actualMvCompNum[rplCnt]; ++mvCompCnt)
    {
      binSum[rplCnt] += auxMvdBins[rplCnt][mvCompCnt].numHypBins;
      CHECK(binSum[rplCnt] < 0, "A negative value of a suffix bin sum");
    }
  }
  
  if (0 == binSum[rpl0Idx] && 0 == binSum[rpl1Idx])
  {
    return REF_PIC_LIST_0;
  }

  const bool rplCoversBinBudget[rplLength] = { curBinBudget <= binSum[rpl0Idx], curBinBudget <= binSum[rpl1Idx] };
  if (rplCoversBinBudget[rpl0Idx] != rplCoversBinBudget[rpl1Idx])
  {
    return rplCoversBinBudget[rpl0Idx] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
  }

  if (!rplCoversBinBudget[rpl0Idx] && !rplCoversBinBudget[rpl1Idx])
  {
    if (binSum[rpl0Idx] > binSum[rpl1Idx])
    {
      return REF_PIC_LIST_0;
    }

    if (binSum[rpl0Idx] < binSum[rpl1Idx])
    {
      return REF_PIC_LIST_1;
    }
  }


  auto compAuxMvdBins = [](const AuxMvdBins& entry0, const AuxMvdBins& entry1) { return entry0.numHypBins > entry1.numHypBins; };
  auto getBinWeight   = [](const int binPos)
  { 
    constexpr int binExistOffset = 10;
    const     int result         = binExistOffset + binPos - 1;
    return result;
  };

  int binWeightedSum[rplLength] = { 0, 0 };
  AuxMvdBins auxMvdBinsCopy[NUM_REF_PIC_LIST_01][maxNumMvComp];
  std::memcpy(auxMvdBinsCopy, auxMvdBins, rplLength * maxNumMvComp * sizeof(AuxMvdBins));
  
  for (int binCnt = 0; binCnt < curBinBudget; ++binCnt)
  {
    std::nth_element(auxMvdBinsCopy[rpl0Idx], auxMvdBinsCopy[rpl0Idx], &auxMvdBinsCopy[rpl0Idx][actualMvCompNum[rpl0Idx]], compAuxMvdBins);
    int& maxBinRpl0 = auxMvdBinsCopy[rpl0Idx][0].numHypBins;
    if (maxBinRpl0 > 0)
    {
      binWeightedSum[rpl0Idx] += getBinWeight(maxBinRpl0);
      --maxBinRpl0;
    }

    std::nth_element(auxMvdBinsCopy[rpl1Idx], auxMvdBinsCopy[rpl1Idx], &auxMvdBinsCopy[rpl1Idx][actualMvCompNum[rpl1Idx]], compAuxMvdBins);
    int& maxBinRpl1 = auxMvdBinsCopy[rpl1Idx][0].numHypBins;
    if (maxBinRpl1 > 0)
    {
      binWeightedSum[rpl1Idx] += getBinWeight(maxBinRpl1);
      --maxBinRpl1;
    }
  }

  return binWeightedSum[rpl0Idx] >= binWeightedSum[rpl1Idx] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
}
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
void MvdSuffixInfo::initPrefixes(const Mv& mv, const int imv, const bool isInternalPrecision)
{
  Mv tmp = mv;
  if (isInternalPrecision)
  {
    tmp.changeIbcPrecInternal2Amvr(imv);
  }

  int       horMvd = tmp.getHor();
  int       verMvd = tmp.getVer();

  unsigned  absHor = unsigned(horMvd < 0 ? -horMvd : horMvd);
  unsigned  absVer = unsigned(verMvd < 0 ? -verMvd : verMvd);
  const auto horRemainderNonZero = (absHor > 0);
  const auto verRemainderNonZero = (absVer > 0);

  horPrefix = -1;
  if (horRemainderNonZero)
  {
    horPrefix = getPrefixLength(absHor - 1 );
  }

  verPrefix = -1;
  if (verRemainderNonZero)
  {
    verPrefix = getPrefixLength(absVer - 1 );
  }

  initSuffixesAndSigns(mv, imv);
}


void MvdSuffixInfo::initSuffixesAndSigns(const Mv& mv, const int imv)
{
  if (horPrefix >= 0)
  {
    horPrefixGroupStartValue = xGetGolombGroupMinValue(horPrefix);
    iBinsInHorSuffix = horPrefix+1;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  else
  {
    horPrefixGroupStartValue = -1;
    iBinsInHorSuffix = -1;
  }
#endif
  if (verPrefix >= 0)
  {
    verPrefixGroupStartValue = xGetGolombGroupMinValue(verPrefix);
    iBinsInVerSuffix = verPrefix + 1;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  else
  {
    verPrefixGroupStartValue = -1;
    iBinsInVerSuffix = -1;
  }
#endif

  defineNumberOfPredictedBinsInSuffix(horPrefix, verPrefix, imv);
}

void MvdSuffixInfo::defineNumberOfPredictedBinsInSuffix(const int iHorPrefix, const int iVerPrefix, const uint8_t imv)
{
  const int iNumberOfMSBins = IBC_BVD_PREDICTION_MAX_BIN_NUM;
  CHECK(iNumberOfMSBins < 0, "Incorrect/negative value of bins to be predicted in suffixes of exp-Golomb code");



  const unsigned int uiExpGolombParam = BVD_CODING_GOLOMB_ORDER;

  const int iBinsInHorSuffix = iHorPrefix < 0 ? 0 : iHorPrefix + uiExpGolombParam;
  const int iBinsInVerSuffix = iVerPrefix < 0 ? 0 : iVerPrefix + uiExpGolombParam;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const int skipFracbins          = isFracBvEnabled ? (imv == IMV_OFF ? 2 : (imv == IMV_HPEL ? 1 : 0)) : 0;  
  const int iAvailBinsInHorSuffix = std::max(0, iBinsInHorSuffix - skipFracbins);
  const int iAvailBinsInVerSuffix = std::max(0, iBinsInVerSuffix - skipFracbins);
#else
  const int iPrecShift = Mv::getImvPrecShift(imv);
  constexpr int iMaxNumberOfInsignLSBins = 0;

  const int iNumberOfInsignLSBins = std::max(0, iMaxNumberOfInsignLSBins - iPrecShift);
  const int iAvailBinsInHorSuffix = std::max(0, iBinsInHorSuffix - iNumberOfInsignLSBins);
  const int iAvailBinsInVerSuffix = std::max(0, iBinsInVerSuffix - iNumberOfInsignLSBins);
#endif
  const int iTotalNumberOfPredBins = std::min(iNumberOfMSBins, iAvailBinsInHorSuffix + iAvailBinsInVerSuffix);

  horEncodeSignInEP = false;
  verEncodeSignInEP = false;

  if (0 >= iTotalNumberOfPredBins)
  {
    horOffsetPredictionNumBins = 0;
    verOffsetPredictionNumBins = 0;
    return;
  }

  int iNumberOfHorMSBins = 0;
  int iNumberOfVerMSBins = 0;
  if (iHorPrefix > iVerPrefix)
  {
    if (iVerPrefix < 0)
    {
      iNumberOfHorMSBins = iTotalNumberOfPredBins;
    }
    else
    {
      const int iDiffHorVer = iAvailBinsInHorSuffix - iAvailBinsInVerSuffix;
      const int iNumberOfEqSignBins = iTotalNumberOfPredBins - iDiffHorVer;
      if (iNumberOfEqSignBins <= 0)
      {

        const bool magnitudeCheck = 2 * (verPrefixGroupStartValue + (1 << (iBinsInVerSuffix - 1))) < (1 << std::max(0, iBinsInHorSuffix - iTotalNumberOfPredBins - 1));
        if (iAvailBinsInHorSuffix > iTotalNumberOfPredBins && magnitudeCheck)
        {
          iNumberOfHorMSBins = iTotalNumberOfPredBins + 1; //1 bin instead of VER sign
          verEncodeSignInEP = true;
        }
        else
        {
          iNumberOfHorMSBins = iTotalNumberOfPredBins;
        }
      }
      else
      {
        iNumberOfVerMSBins = iNumberOfEqSignBins >> 1;
        CHECK(iAvailBinsInHorSuffix + iAvailBinsInVerSuffix == iTotalNumberOfPredBins && iNumberOfEqSignBins - (iNumberOfVerMSBins << 1) != 0, "iNumberOfEqSignBins is odd for iHorPrefix > iVerPrefix");
        iNumberOfHorMSBins = iNumberOfEqSignBins - iNumberOfVerMSBins + iDiffHorVer;
      }
    }
  }
  else if (iVerPrefix > iHorPrefix)
  {
    if (iHorPrefix < 0)
    {
      iNumberOfVerMSBins = iTotalNumberOfPredBins;
    }
    else
    {
      const int iDiffVerHor = iAvailBinsInVerSuffix - iAvailBinsInHorSuffix;
      const int iNumberOfEqSignBins = iTotalNumberOfPredBins - iDiffVerHor;
      if (iNumberOfEqSignBins <= 0)
      {
        const bool magnitudeCheck = 2 * (horPrefixGroupStartValue + (1 << (iBinsInHorSuffix - 1))) < (1 << std::max(0, iBinsInVerSuffix - iTotalNumberOfPredBins - 1));
        if (iAvailBinsInVerSuffix > iTotalNumberOfPredBins && magnitudeCheck)
        {
          iNumberOfVerMSBins = iTotalNumberOfPredBins + 1; //1 bin instead of HOR sign
          horEncodeSignInEP = true;
        }
        else
        {
          iNumberOfVerMSBins = iTotalNumberOfPredBins;
        }
      }
      else
      {
        iNumberOfHorMSBins = iNumberOfEqSignBins >> 1;
        CHECK(iAvailBinsInHorSuffix + iAvailBinsInVerSuffix == iTotalNumberOfPredBins && iNumberOfEqSignBins - (iNumberOfHorMSBins << 1) != 0, "iNumberOfEqSignBins is odd for iVerPrefix > iHorPrefix");
        iNumberOfVerMSBins = iNumberOfEqSignBins - iNumberOfHorMSBins + iDiffVerHor;
      }
    }
  }
  else
  {
    iNumberOfVerMSBins = std::min(iAvailBinsInVerSuffix, (iTotalNumberOfPredBins >> 1));
    iNumberOfHorMSBins = iTotalNumberOfPredBins - iNumberOfVerMSBins;
  };

  CHECK(iNumberOfHorMSBins < 0, "iNumberOfHorMSBins < 0");
  CHECK(iNumberOfVerMSBins < 0, "iNumberOfVerMSBins < 0");

  horOffsetPredictionNumBins = std::min(iAvailBinsInHorSuffix, iNumberOfHorMSBins);
  verOffsetPredictionNumBins = std::min(iAvailBinsInVerSuffix, iNumberOfVerMSBins);
}
#endif

#if JVET_AD0140_MVD_PREDICTION
void MvdSuffixInfoMv::initPrefixesMvd(const int mvIdx, const RefPicList curRpl, const Mv& mv, const int imv, const bool isInternalPrecision, const MotionModel& motionModel)
{
  CHECK(mvIdx < 0, "mvIdx < 0");
  CHECK(mvIdx > 5, "mvIdx > 5");

  auto& horPrefix = mvBins[curRpl][mvIdx].horPrefix;
  auto& verPrefix = mvBins[curRpl][mvIdx].verPrefix;

  Mv tmp = mv;
  if (isInternalPrecision)
  {
    const auto transPrecFxn = MotionModelCheck::isAffine(motionModel) ? &Mv::changeAffinePrecInternal2Amvr : &Mv::changeTransPrecInternal2Amvr;
    (tmp.*transPrecFxn)(imv);
  }

  int       horMvd = tmp.getHor();
  int       verMvd = tmp.getVer();

  unsigned  absHor = unsigned(horMvd < 0 ? -horMvd : horMvd);
  unsigned  absVer = unsigned(verMvd < 0 ? -verMvd : verMvd);

  const int iEgcOffset = MvdSuffixInfo::getEGCOffset(motionModel) + 1;
  const auto horRemainderNonZero = (absHor >= iEgcOffset);
  const auto verRemainderNonZero = (absVer >= iEgcOffset);

  horPrefix = -1;
  if (horRemainderNonZero)
  {
    horPrefix = MvdSuffixInfo::getPrefixLength(absHor - iEgcOffset);
  }

  verPrefix = -1;
  if (verRemainderNonZero)
  {
    verPrefix = MvdSuffixInfo::getPrefixLength(absVer - iEgcOffset);
  }
  if (-1 == actualMvCompNum[curRpl]) // init
  {
    actualMvCompNum[curRpl] = 0;
  } 

  initSuffixesAndSignsMvd(mvIdx, curRpl, mv, imv, motionModel);
}

void MvdSuffixInfoMv::clear()
{
  m_motionModel = MotionModel::Undefined;
  for (int rplIdx = static_cast<int>(REF_PIC_LIST_0); rplIdx < static_cast<int>(NUM_REF_PIC_LIST_01); ++rplIdx)
  { 
    actualMvCompNum[rplIdx] = -1;
    for (int i = 0; i < maxNumMv; ++i)
    {
      mvBins[rplIdx][i] = MvdSuffixInfo();
    }
    for (int i = 0; i < maxNumMvComp; ++i)
    {
      auxMvdBins[rplIdx][i] = AuxMvdBins();
    }
  }
}

void MvdSuffixInfoMv::initSuffixesAndSignsMvd(const int mvIdx, const RefPicList rpl, const Mv& mvd, const int imv, const MotionModel& motionModel)
{
  const size_t rplIdx = static_cast<size_t>(rpl);
  CHECK(rplIdx >= static_cast<int>(NUM_REF_PIC_LIST_01), "Uninitialized value of actualRpl is used");

  if (actualMvCompNum[rplIdx] < 0)
  {
    actualMvCompNum[rplIdx] = 0;
  }

  mvBins[rplIdx][mvIdx].m_motionModel = motionModel;

  auto& isHorMagnZero = mvBins[rplIdx][mvIdx].isHorMagnZero;
  auto& isVerMagnZero = mvBins[rplIdx][mvIdx].isVerMagnZero;

  isHorMagnZero = mvd.getHor() == 0;
  isVerMagnZero = mvd.getVer() == 0;

  auto& horPrefix = mvBins[rplIdx][mvIdx].horPrefix;
  auto& verPrefix = mvBins[rplIdx][mvIdx].verPrefix;
  auto& si = mvBins[rplIdx][mvIdx];

  int iPrecShift = MotionModelCheck::isAffine(motionModel) ? Mv::getImvPrecShiftAffineMvd(imv) : Mv::getImvPrecShiftMvd(imv);
  constexpr int iMaxNumberOfInsignLSBins = 0;
  const int iNumberOfInsignLSBins = std::max(0, iMaxNumberOfInsignLSBins - iPrecShift);

  if (!isHorMagnZero)
  {
    if (horPrefix >= 0)
    {
      si.horPrefixGroupStartValue = MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix);
      si.iBinsInHorSuffix = horPrefix + 1;
    }
    // set bin prediction budget for horizontal component of MVD (including the sign bin)
    int suffixBins = std::max(0, (horPrefix >= 0) ? mvBins[rplIdx][mvIdx].iBinsInHorSuffix - iNumberOfInsignLSBins : 0);
    auxMvdBins[rplIdx][actualMvCompNum[rplIdx]++] = AuxMvdBins( mvIdx, true, suffixBins + (isHorMagnZero ? 0 : 1) /*sign*/, static_cast<int>(rplIdx));
  }

  if (!isVerMagnZero)
  {
    if (verPrefix >= 0)
    {
      si.verPrefixGroupStartValue = MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix);
      si.iBinsInVerSuffix = verPrefix + 1;
    }
    // set bin prediction budget for vertical component of MVD (including the sign bin)
    int suffixBins = std::max(0, (verPrefix >= 0) ? mvBins[rplIdx][mvIdx].iBinsInVerSuffix - iNumberOfInsignLSBins : 0);
    auxMvdBins[rplIdx][actualMvCompNum[rplIdx]++] = AuxMvdBins( mvIdx, false, suffixBins + (isVerMagnZero ? 0 : 1) /*sign*/, static_cast<int>(rplIdx));
  }

  if (MotionModel::UniAffine != m_motionModel && MotionModel::BiAffine != m_motionModel)
  {
    CHECK(actualMvCompNum[rplIdx] != (0 != mvd.getAbsHor() ? 1 : 0) + (0 != mvd.getAbsVer() ? 1 : 0), "actualMvCompNum invalid");
  }

  if (MotionModel::Undefined == m_motionModel)
  {
    m_motionModel = motionModel;
  }
  else
  {
    CHECK(m_motionModel != motionModel, "Inconsistent assignment of motion model");
  }
}


int MvdSuffixInfo::getEGCOffset(const MotionModel& motionModel)
{
  CHECK(MotionModel::Undefined == motionModel, "Undefined motion model");

  return 1;
}

bool MotionModelCheck::isAffine(const MotionModel& mm)
{
  return (mm == MotionModel::UniAffine) || (mm == MotionModel::BiAffine);
}
#endif
//! \}
