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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif
#include "EncReshape.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class EncModeCtrl;

enum PLTScanMode
{
  PLT_SCAN_HORTRAV = 0,
  PLT_SCAN_VERTRAV = 1,
  NUM_PLT_SCAN = 2
};
class SortingElement
{
public:
  SortingElement()
  {
    cnt[0] = cnt[1] = cnt[2] = cnt[3] = 0;
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    data[0] = data[1] = data[2] = 0;
    sumData[0] = sumData[1] = sumData[2] = 0;
  }
  uint32_t  getCnt(int idx) const         { return cnt[idx]; }
  void      setCnt(uint32_t val, int idx) { cnt[idx] = val; }
  int       getSumData (int id) const   { return sumData[id]; }

  void resetAll(ComponentID compBegin, uint32_t numComp)
  {
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = 0;
      sumData[ch] = 0;
    }
  }
  void setAll(uint32_t* ui, ComponentID compBegin, uint32_t numComp)
  {
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = ui[ch];
    }
  }
  bool almostEqualData(SortingElement element, int errorLimit, const BitDepths& bitDepths, ComponentID compBegin, uint32_t numComp, bool lossless)
  {
    bool almostEqual = true;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      if (lossless)
      {
        if ((std::abs(data[comp] - element.data[comp])) > errorLimit)
        {
          almostEqual = false;
          break;
        }
      }
      else
      {
      uint32_t absError = 0;
      if (isChroma((ComponentID) comp))
      {
        absError += int(double(std::abs(data[comp] - element.data[comp])) * PLT_CHROMA_WEIGHTING) >> (bitDepths.recon[CHANNEL_TYPE_CHROMA] - PLT_ENCBITDEPTH);
      }
      else
      {
        absError += (std::abs(data[comp] - element.data[comp]))>> (bitDepths.recon[CHANNEL_TYPE_LUMA] - PLT_ENCBITDEPTH);
      }
      if (absError > errorLimit)
      {
        almostEqual = false;
        break;
      }
      }
    }
    return almostEqual;
  }
  uint32_t getSAD(SortingElement element, const BitDepths &bitDepths, ComponentID compBegin, uint32_t numComp, bool lossless)
  {
    uint32_t sumAd = 0;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      ChannelType chType = (comp > 0) ? CHANNEL_TYPE_CHROMA : CHANNEL_TYPE_LUMA;
      if (lossless)
      {
        sumAd += (std::abs(data[comp] - element.data[comp]));
      }
      else
      {
      sumAd += (std::abs(data[comp] - element.data[comp]) >> (bitDepths.recon[chType] - PLT_ENCBITDEPTH));
      }
    }
    return sumAd;
  }
  void copyDataFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      data[comp] = element.data[comp];
      sumData[comp] = data[comp];
      shift[comp] = 0;
      lastCnt[comp] = 1;
    }
  }
  void copyAllFrom(SortingElement element, ComponentID compBegin, uint32_t numComp)
  {
    copyDataFrom(element, compBegin, numComp);
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      sumData[comp] = element.sumData[comp];
      cnt[comp]     = element.cnt[comp];
      shift[comp]   = element.shift[comp];
      lastCnt[comp] = element.lastCnt[comp];
    }
    cnt[MAX_NUM_COMPONENT] = element.cnt[MAX_NUM_COMPONENT];
  }
  void addElement(const SortingElement& element, ComponentID compBegin, uint32_t numComp)
  {
    for (int i = compBegin; i<(compBegin + numComp); i++)
    {
      sumData[i] += element.data[i];
      cnt[i]++;
      if( cnt[i] > 1 && cnt[i] == 2 * lastCnt[i] )
      {
        uint32_t rnd = 1 << shift[i];
        shift[i]++;
        data[i] = (sumData[i] + rnd) >> shift[i];
        lastCnt[i] = cnt[i];
      }
    }
  }
private:
  uint32_t cnt[MAX_NUM_COMPONENT+1];
  int shift[3], lastCnt[3], data[3], sumData[3];
};
/// encoder search class
class IntraSearch : public IntraPrediction
{
private:
  EncModeCtrl    *m_modeCtrl;
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
  int             m_testAMTForFullRD[6];
  bool            m_validMTSReturn;
#else
  int             m_testAMTForFullRD[4];
#endif
  int             m_numCandAMTForFullRD;
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  std::array<std::array<uint32_t, NUM_INDICES_REP>, MAX_NUM_COMPONENT> m_indicesRepresentationPnn;
#endif
  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool            m_saveCuCostInSCIPU;
  uint8_t         m_numCuInSCIPU;
  Area            m_cuAreaInSCIPU[NUM_INTER_CU_INFO_SAVE];
  double          m_cuCostInSCIPU[NUM_INTER_CU_INFO_SAVE];
#endif
  struct ModeInfo
  {
    bool     mipFlg; // CU::mipFlag
    bool     mipTrFlg; // PU::mipTransposedFlag
    int      mRefId; // PU::multiRefIdx
    uint8_t  ispMod; // CU::ispMode
    uint32_t modeId; // PU::intraDir[CHANNEL_TYPE_LUMA]
#if JVET_V0130_INTRA_TMP
    bool     tmpFlag; // CU::tmpFlag
#if JVET_AD0086_ENHANCED_INTRA_TMP
    int      tmpIdx;
    bool     tmpFusionFlag;
    bool     tmpFlmFlag;
#if JVET_AG0136_INTRA_TMP_LIC
    bool     tmpLicFlag;
    uint8_t  tmpLicIdc;
#endif
    int      tmpIsSubPel;
    int      tmpSubPelIdx;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
    int      tmpFracIdx;
#endif
#endif
#endif
#if JVET_AB0155_SGPM
    bool     sgpmFlag;   // CU::sgpmFlag
    int      sgpmSplitDir;
    int      sgpmMode0;
    int      sgpmMode1;
    int      sgpmIdx;
#if JVET_AG0152_SGPM_ITMP_IBC
#if JVET_AL0188_SGPM_FLIPAWARE_BV
    BvInfo   sgpmBv0;
    BvInfo   sgpmBv1;
#else
    Mv       sgpmBv0;
    Mv       sgpmBv1;
#endif
#endif
#if JVET_AJ0112_REGRESSION_SGPM
    bool sgpmIsRegression;
    AffineBlendingModel sgpmBlendModel;
#endif
#endif
#if JVET_AM0074_INTRA_MERGE
    uint8_t     dimd;
    uint8_t     timd;
#endif
#if JVET_AB0155_SGPM
#if JVET_V0130_INTRA_TMP
    ModeInfo() : mipFlg( false ), mipTrFlg( false ), mRefId( 0 ), ispMod( NOT_INTRA_SUBPARTITIONS ), modeId( 0 ), tmpFlag( 0 )
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
      , tmpIdx(0) , tmpFusionFlag(false) , tmpFlmFlag(false) , tmpLicFlag(false) , tmpLicIdc(0), tmpIsSubPel(0), tmpSubPelIdx(0)
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      , tmpFracIdx(-1)
#endif
#else
      , tmpIdx(0) , tmpFusionFlag(false) , tmpFlmFlag(false) , tmpIsSubPel(0), tmpSubPelIdx(0)
#endif
#endif
	, sgpmFlag( 0 ), sgpmSplitDir( 0 ), sgpmMode0( 0 ), sgpmMode1( 0 ), sgpmIdx( 0 ) 
#if JVET_AG0152_SGPM_ITMP_IBC
#if JVET_AL0188_SGPM_FLIPAWARE_BV
      , sgpmBv0(Mv(0, 0), 0), sgpmBv1(Mv(0, 0), 0)
#else
  , sgpmBv0(0, 0), sgpmBv1(0, 0)
#endif
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  , sgpmIsRegression(false)
  , sgpmBlendModel(AffineBlendingModel(5,1,31))
#endif
#if JVET_AM0074_INTRA_MERGE
  , dimd (0)
  , timd (0)
#endif
{}
    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode,
             const bool tmpf = 0
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
      , const int tmpi = 0 , const bool tmpff = 0  , const int tmpflmf = 0 , const int tmpLic = 0 , const int tmpLicIdc = 0 , const int tmpsp = 0, const int tmpspi = 0
#if JVET_AH0200_INTRA_TMP_BV_REORDER
    , const int tmpfi = -1
#endif
#else
      , const int tmpi = 0 , const bool tmpff = 0  , const int tmpflmf = 0 , const int tmpsp = 0, const int tmpspi = 0
#endif
#endif
	  , const bool sf = 0, const int sd = 0, const int sm0 = 0, const int sm1 = 0, const int si = 0
#if JVET_AM0074_INTRA_MERGE
    , const uint8_t dimd = 0, const uint8_t timd = 0
#endif
#if JVET_AG0152_SGPM_ITMP_IBC
#if JVET_AL0188_SGPM_FLIPAWARE_BV
      , const BvInfo sbv0 = BvInfo(Mv(0, 0), 0), const BvInfo sbv1 = BvInfo(Mv(0, 0), 0)
#else
    , const Mv sbv0 = Mv(0, 0), const Mv sbv1 = Mv(0, 0)
#endif
#endif
#if JVET_AJ0112_REGRESSION_SGPM
      , const bool sir = false
      , const AffineBlendingModel sbm = AffineBlendingModel(5, 1, 31)
#endif
)
#else
    ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0), sgpmFlag(0), sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0), sgpmIdx(0){}
    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode,
             const bool sf = 0, const int sd = 0, const int sm0 = 0, const int sm1 = 0, const int si = 0)
#endif
      : mipFlg(mipf)
      , mipTrFlg(miptf)
      , mRefId(mrid)
      , ispMod(ispm)
      , modeId(mode)
#if JVET_V0130_INTRA_TMP
      , tmpFlag(tmpf)
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
      , tmpIdx(tmpi) , tmpFusionFlag(tmpff) , tmpFlmFlag(tmpflmf) , tmpLicFlag(tmpLic) , tmpLicIdc(tmpLicIdc) ,tmpIsSubPel(tmpsp) , tmpSubPelIdx(tmpspi)
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      , tmpFracIdx(tmpfi)
#endif
#else
      , tmpIdx(tmpi) , tmpFusionFlag(tmpff) , tmpFlmFlag(tmpflmf) , tmpIsSubPel(tmpsp) , tmpSubPelIdx(tmpspi)
#endif
#endif
      , sgpmFlag(sf)
      , sgpmSplitDir(sd)
      , sgpmMode0(sm0)
      , sgpmMode1(sm1)
      , sgpmIdx(si)
#if JVET_AG0152_SGPM_ITMP_IBC
      , sgpmBv0(sbv0)
      , sgpmBv1(sbv1)
#endif
#if JVET_AJ0112_REGRESSION_SGPM
      , sgpmIsRegression(sir)
      , sgpmBlendModel(sbm)
#endif
#if JVET_AM0074_INTRA_MERGE
      , dimd(dimd)
      , timd(timd)
#endif
    {
    }
    ModeInfo &operator=(const ModeInfo &other)
    {
      mipFlg       = other.mipFlg;     // CU::mipFlag
      mipTrFlg     = other.mipTrFlg;   // PU::mipTransposedFlag
      mRefId       = other.mRefId;     // PU::multiRefIdx
      ispMod       = other.ispMod;     // CU::ispMode
      modeId       = other.modeId;     // PU::intraDir[CHANNEL_TYPE_LUMA]
#if JVET_V0130_INTRA_TMP
      tmpFlag      = other.tmpFlag;    // CU::tmpFlag
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
      tmpIdx        = other.tmpIdx;
      tmpFusionFlag = other.tmpFusionFlag;
      tmpFlmFlag    = other.tmpFlmFlag;
#if JVET_AG0136_INTRA_TMP_LIC
      tmpLicFlag    = other.tmpLicFlag;
      tmpLicIdc     = other.tmpLicIdc;
#endif
      tmpIsSubPel   = other.tmpIsSubPel;    // CU::tmpIsSubPel
      tmpSubPelIdx  = other.tmpSubPelIdx;   // CU::tmpSubPelIdx
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      tmpFracIdx   = other.tmpFracIdx;
#endif
#endif
      sgpmFlag     = other.sgpmFlag;   // CU::sgpmFlag
      sgpmSplitDir = other.sgpmSplitDir;
      sgpmMode0    = other.sgpmMode0;
      sgpmMode1    = other.sgpmMode1;
      sgpmIdx      = other.sgpmIdx;
#if JVET_AG0152_SGPM_ITMP_IBC
      sgpmBv0 = other.sgpmBv0;
      sgpmBv1 = other.sgpmBv1;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
      sgpmIsRegression = other.sgpmIsRegression;
      sgpmBlendModel = other.sgpmBlendModel;
#endif
#if JVET_AM0074_INTRA_MERGE
      dimd = other.dimd;
      timd = other.timd;
#endif
      return *this;
    }
    bool operator==(const ModeInfo cmp) const
    {
      return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod
                && modeId == cmp.modeId 
#if JVET_V0130_INTRA_TMP
                && tmpFlag == cmp.tmpFlag
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
                && tmpIdx == cmp.tmpIdx
                && tmpFusionFlag == cmp.tmpFusionFlag
                && tmpFlmFlag == cmp.tmpFlmFlag
#if JVET_AG0136_INTRA_TMP_LIC
                && tmpLicFlag == cmp.tmpLicFlag
                && tmpLicIdc == cmp.tmpLicIdc
#endif
                && tmpIsSubPel == cmp.tmpIsSubPel
                && tmpSubPelIdx == cmp.tmpSubPelIdx
#if JVET_AH0200_INTRA_TMP_BV_REORDER
                && tmpFracIdx == cmp.tmpFracIdx
#endif
#endif
                && sgpmFlag == cmp.sgpmFlag
                && sgpmSplitDir == cmp.sgpmSplitDir // sgpmMode0 and sgpmMode1 seems no need
#if JVET_AM0074_INTRA_MERGE
                && dimd == cmp.dimd
                && timd == cmp.timd
#endif
);
    }
#elif JVET_V0130_INTRA_TMP
	  ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0), tmpFlag(0) {}
	  ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode, const bool tpmf = 0) : mipFlg(mipf), mipTrFlg(miptf), mRefId(mrid), ispMod(ispm), modeId(mode), tmpFlag(tpmf) {}
	  bool operator==(const ModeInfo cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId && tmpFlag == cmp.tmpFlag); }
#else
    ModeInfo() : mipFlg(false), mipTrFlg(false), mRefId(0), ispMod(NOT_INTRA_SUBPARTITIONS), modeId(0) {}
    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode) : mipFlg(mipf), mipTrFlg(miptf), mRefId(mrid), ispMod(ispm), modeId(mode) {}
    bool operator==(const ModeInfo cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId); }
#endif
#if JVET_AM0074_INTRA_MERGE
    void setDimdParam(uint8_t in) { dimd = in; }
    void setTimdParam(uint8_t in) { timd = in; }
#endif
  };
  struct ModeInfoWithCost : public ModeInfo
  {
    double rdCost;
    ModeInfoWithCost() : ModeInfo(), rdCost(MAX_DOUBLE) {}
#if JVET_AB0155_SGPM && JVET_V0130_INTRA_TMP
    ModeInfoWithCost(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode, const bool tpmf, 
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
      const int tmpi, const bool tmpff, const int tmpflmf,  const int tmpLicItmp, const int tmpLicIdc, const int tmpsp, const int tmpspi,
#if JVET_AH0200_INTRA_TMP_BV_REORDER
                     const int tmpfi,
#endif
#else
                     const int tmpi, const bool tmpff, const int tmpflmf,  const int tmpsp, const int tmpspi,
#endif
#endif
					 double cost, const bool sf = 0, const int sd = 0, const int sm0 = 0, const int sm1 = 0
#if JVET_AM0074_INTRA_MERGE
           , const uint8_t dimd = 0
           , const uint8_t timd = 0
#endif
)
      : ModeInfo(mipf, miptf, mrid, ispm, mode, tpmf
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
        ,tmpi ,tmpff ,tmpflmf , tmpLicItmp, tmpLicIdc, tmpsp, tmpspi
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        ,tmpfi
#endif
#else
        ,tmpi ,tmpff ,tmpflmf , tmpsp, tmpspi
#endif
#endif
	  , sf, sd, sm0, sm1
#if JVET_AM0074_INTRA_MERGE
	  , dimd
	  , timd
#endif
      ), rdCost(cost)
    {
    }
    bool operator==(const ModeInfoWithCost cmp) const
    {
      return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod
              && modeId == cmp.modeId && tmpFlag == cmp.tmpFlag
#if JVET_AD0086_ENHANCED_INTRA_TMP
      && tmpIdx == cmp.tmpIdx
      && tmpFusionFlag == cmp.tmpFusionFlag
      && tmpFlmFlag == cmp.tmpFlmFlag
#if JVET_AG0136_INTRA_TMP_LIC
      && tmpLicFlag == cmp.tmpLicFlag
      && tmpLicIdc == cmp.tmpLicIdc
#endif
      && tmpIsSubPel == cmp.tmpIsSubPel
      && tmpSubPelIdx == cmp.tmpSubPelIdx
#if JVET_AH0200_INTRA_TMP_BV_REORDER
      && tmpFracIdx == cmp.tmpFracIdx
#endif
#endif
	  && rdCost == cmp.rdCost && sgpmFlag == cmp.sgpmFlag
      && sgpmSplitDir == cmp.sgpmSplitDir
#if JVET_AM0074_INTRA_MERGE
      && dimd == cmp.dimd
      && timd == cmp.timd
#endif
);   // sgpmMode0 and sgpmMode1 seems no need
    }
#elif JVET_V0130_INTRA_TMP
	  ModeInfoWithCost(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode, const bool tpmf, double cost) : ModeInfo(mipf, miptf, mrid, ispm, mode, tpmf), rdCost(cost) {}
	  bool operator==(const ModeInfoWithCost cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId && tmpFlag == cmp.tmpFlag && rdCost == cmp.rdCost); }
#else
    ModeInfoWithCost(const bool mipf, const bool miptf, const int mrid, const uint8_t ispm, const uint32_t mode, double cost) : ModeInfo(mipf, miptf, mrid, ispm, mode), rdCost(cost) {}
    bool operator==(const ModeInfoWithCost cmp) const { return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && mRefId == cmp.mRefId && ispMod == cmp.ispMod && modeId == cmp.modeId && rdCost == cmp.rdCost); }
#endif
    static bool compareModeInfoWithCost(ModeInfoWithCost a, ModeInfoWithCost b) { return a.rdCost < b.rdCost; }
  };

  struct ISPTestedModeInfo
  {
    int    numCompSubParts;
    double rdCost;

    ISPTestedModeInfo() {}

    void setMode(int numParts, double cost)
    {
      numCompSubParts = numParts;
      rdCost = cost;
    }
    void clear()
    {
      numCompSubParts = -1;
      rdCost = MAX_DOUBLE;
    }
  };
  struct ISPTestedModesInfo
  {
    std::map<int, ISPTestedModeInfo>            intraMode[2];
    std::map<int, bool>                         modeHasBeenTested[2];
    int                                         numTotalParts[2];
    static_vector<int, FAST_UDI_MAX_RDMODE_NUM> testedModes[2];
    int                                         bestModeSoFar;
    ISPType                                     bestSplitSoFar;
    int                                         bestMode[2];
    double                                      bestCost[2];
    int                                         numTestedModes[2];
    int                                         candIndexInList[2];
    bool                                        splitIsFinished[2];
    int                                         numOrigModesToTest;

    // set a tested mode results
    void setModeResults(ISPType splitType, int iModeIdx, int numCompletedParts, double rdCost, double currentBestCost)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      CHECK( iModeIdx < 0, "The modeIdx is invalid" );
      const int maxNumParts = numTotalParts[st];
      intraMode[st][iModeIdx].setMode(numCompletedParts, numCompletedParts == maxNumParts ? rdCost : MAX_DOUBLE);
      testedModes[st].push_back(iModeIdx);
      numTestedModes[st]++;
      modeHasBeenTested[st][iModeIdx] = true;
      if (numCompletedParts == maxNumParts && rdCost < bestCost[st])   // best mode update
      {
        bestMode[st] = iModeIdx;
        bestCost[st] = rdCost;
      }
      if (numCompletedParts == maxNumParts && rdCost < currentBestCost)   // best mode update
      {
        bestModeSoFar = iModeIdx;
        bestSplitSoFar = splitType;
      }
    }

    int getNumCompletedSubParts(ISPType splitType, int iModeIdx)
    {
      const unsigned st = splitType - 1;
      CHECK(st < 0 || st > 1, "The split type is invalid!");
      CHECK(iModeIdx < 0, "The modeIdx is invalid");
      return modeHasBeenTested[st].count( iModeIdx ) > 0 ? intraMode[st][iModeIdx].numCompSubParts : -1;
    }

    double getRDCost(ISPType splitType, int iModeIdx)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return modeHasBeenTested[st].count( iModeIdx ) > 0 ? intraMode[st][iModeIdx].rdCost : MAX_DOUBLE;
    }

    // get a tested intra mode index
    int getTestedIntraMode(ISPType splitType, int pos)
    {
      const unsigned st = splitType - 1;
      CHECKD(st > 1, "The split type is invalid!");
      return pos < testedModes[st].size() ? testedModes[st].at(pos) : -1;
    }

    // set everything to default values
    void clear()
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        numTestedModes [splitIdx] = 0;
        candIndexInList[splitIdx] = 0;
        numTotalParts  [splitIdx] = 0;
        splitIsFinished[splitIdx] = false;
        testedModes    [splitIdx].clear();
        bestCost       [splitIdx] = MAX_DOUBLE;
        bestMode       [splitIdx] = -1;
      }
      bestModeSoFar = -1;
      bestSplitSoFar = NOT_INTRA_SUBPARTITIONS;
      numOrigModesToTest = -1;
      modeHasBeenTested[0].clear();
      modeHasBeenTested[1].clear();
    }
    void clearISPModeInfo(int idx)
    {
      intraMode[0].clear();
      intraMode[1].clear();
    }
    void init(const int numTotalPartsHor, const int numTotalPartsVer)
    {
      clear();
      const int horSplit = HOR_INTRA_SUBPARTITIONS - 1, verSplit = VER_INTRA_SUBPARTITIONS - 1;
      numTotalParts  [horSplit] = numTotalPartsHor;
      numTotalParts  [verSplit] = numTotalPartsVer;
      splitIsFinished[horSplit] = (numTotalParts[horSplit] == 0);
      splitIsFinished[verSplit] = (numTotalParts[verSplit] == 0);
    }
  };

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_ispCandListHor, m_ispCandListVer;
  static_vector<ModeInfoWithCost, FAST_UDI_MAX_RDMODE_NUM> m_regIntraRDListWithCosts;

  ISPTestedModesInfo m_ispTestedModes[NUM_LFNST_NUM_PER_SET];
  int m_curIspLfnstIdx;

  //cost variables for the EMT algorithm and new modes list
  double     m_bestModeCostStore[ NUM_LFNST_NUM_PER_SET ];                                    // RD cost of the best mode for each PU using DCT2
  bool       m_bestModeCostValid[ NUM_LFNST_NUM_PER_SET ];
  double     m_modeCostStore[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];                   // RD cost of each mode for each PU using DCT2
  ModeInfo   m_savedRdModeList[ NUM_LFNST_NUM_PER_SET ][ NUM_LUMA_MODE ];
  int32_t    m_savedNumRdModes[ NUM_LFNST_NUM_PER_SET ];
#if JVET_W0103_INTRA_MTS
  double     m_globalBestCostStore;
  bool       m_globalBestCostValid;
  int        m_numModesISPRDO; //full modes for ISP testing.
#if JVET_Y0142_ADAPT_INTRA_MTS
  static_vector<ModeInfo, NUM_LUMA_MODE> m_modesForMTS;
  static_vector<int64_t, NUM_LUMA_MODE> m_modesCoeffAbsSumDCT2;
  int64_t m_coeffAbsSumDCT2;
#endif
#if JVET_AL0108_BVG_DIMD
  double     m_intraBestCostStore;
  bool       m_intraBestCostValid;
#endif
#endif
#if JVET_AJ0061_TIMD_MERGE
  static_vector<ModeInfo, NumTimdMode> m_uiSavedRdModeListTimd;
  static_vector<double,   NumTimdMode> m_uiSavedModeCostTimd;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  double     m_bestIntraSADHADCost;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  double     m_bestIntraSADCost;
#endif
  ModeInfo                                           m_savedRdModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  char                                               m_savedBDPCMModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  double                                             m_savedRdCostFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2][FAST_UDI_MAX_RDMODE_NUM];
  int                                                m_numSavedRdModeFirstColorSpace[4 * NUM_LFNST_NUM_PER_SET * 2];
  int                                                m_savedRdModeIdx;

#if SECONDARY_MPM
  int m_mpmListSize;
#endif

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedRdModeListLFNST;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiSavedHadModeListLFNST;
  uint32_t                                         m_uiSavedNumRdModesLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedModeCostLFNST;
  static_vector<double,   FAST_UDI_MAX_RDMODE_NUM> m_dSavedHadListLFNST;

#if JVET_AB0155_SGPM
  static_vector<ModeInfo, SGPM_NUM> m_uiSavedRdModeListSGPM;
  static_vector<ModeInfo, SGPM_NUM> m_uiSavedHadModeListSGPM;
  static_vector<double, SGPM_NUM>   m_dSavedModeCostSGPM;
  static_vector<double, SGPM_NUM>   m_dSavedHadListSGPM;

#if JVET_AG0152_SGPM_ITMP_IBC
  Pel* m_intraPredBuf[NUM_LUMA_MODE + SGPM_NUM_BVS];
  Pel* m_sgpmPredBuf[SGPM_NUM];
  uint8_t         m_intraModeReady[NUM_LUMA_MODE + SGPM_NUM_BVS];
#else
  Pel*            m_intraPredBuf[NUM_LUMA_MODE];
  Pel*            m_sgpmPredBuf[SGPM_NUM];
  uint8_t         m_intraModeReady[NUM_LUMA_MODE];
#endif
#endif
#if JVET_AH0209_PDP
  Pel* m_pdpIntraPredBuf[NUM_LUMA_MODE];
#if JVET_AK0061_PDP_MPM
  static_vector<ModeInfo, NUM_MOST_PROBABLE_MODES> m_mpmSavedPdpModeList;
  static_vector<double, NUM_MOST_PROBABLE_MODES> m_mpmSavedPdpRdList;
  std::pair<ModeInfo, double> m_timdMergeRdModeList;
  
#endif
#endif
#if JVET_AG0058_EIP
  Pel* m_eipPredBuf[NUM_DERIVED_EIP];
  Pel* m_eipMergePredBuf[MAX_MERGE_EIP];
#if JVET_AL0106_BV_EIP
  Pel* m_eipBvPredBuf[MAX_BV_EIP];
#endif
  static_vector<ModeInfo, NUM_DERIVED_EIP + MAX_MERGE_EIP> m_uiSavedRdModeListEip;
  static_vector<ModeInfo, NUM_DERIVED_EIP + MAX_MERGE_EIP> m_uiSavedHadModeListEip;
  static_vector<double, NUM_DERIVED_EIP + MAX_MERGE_EIP>   m_dSavedModeCostEip;
  static_vector<double, NUM_DERIVED_EIP + MAX_MERGE_EIP>   m_dSavedHadListEip;
#if JVET_AJ0082_MM_EIP
  double m_encBestEipCost;
#endif
#endif
#if JVET_AJ0061_TIMD_MERGE
  Pel* m_timdPredBuf[NumTimdMode];
#endif
#if JVET_AJ0146_TIMDSAD 
  Pel* m_timdSadPredBuf;
#if !JVET_AJ0061_TIMD_MERGE
  Pel* m_timdPredBuf;
#endif
  double m_dSavedRDCostTimdSad;
  double m_dSavedHadTimdSad;
#endif
#if JVET_AH0076_OBIC
  Pel* m_dimdPredBuf;
  Pel* m_obicPredBuf;
#endif
#if JVET_AK0059_MDIP
  Distortion m_mpm0SadHad;
  Pel* m_mdipPredBuf;
  double m_dSavedSadHadRdCostMdip;
  Distortion m_dSavedSadHadMdip;
  Distortion m_dSavedSadHadPdp;
  Distortion m_dSavedSadPdp;
  bool m_includeExcludingMode[NUM_LUMA_MODE];
#endif
#if JVET_AL0108_BVG_DIMD
  Pel *m_bvgDimdPredBuf;
#endif
  PelStorage      m_tmpStorageLCU;
  PelStorage      m_colorTransResiBuf;
#if JVET_AB0143_CCCM_TS
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  PelStorage      m_cccmStorage[2][TOTAL_NUM_CCCM_MODES];
#else
  PelStorage      m_cccmStorage[2][CCCM_NUM_MODES];
#endif
#else
  PelStorage      m_cccmStorage[CCCM_NUM_MODES];
#endif
#endif
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
  PelStorage      m_ccpFusionStorage[MAX_CCP_FUSION_NUM];
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  PelStorage      m_ccpMergeAdjustStorage[MAX_CCP_CAND_LIST_SIZE<<1];
#endif

#if JVET_AD0188_CCP_MERGE
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  CCPModelCandidate m_ccmParamsStorage[2][TOTAL_NUM_CCCM_MODES];
#else
  CCPModelCandidate m_ccmParamsStorage[2][CCCM_NUM_MODES];
#endif
#else
  CCPModelCandidate m_ccmParamsStorage[CCCM_NUM_MODES];
#endif
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  PelStorage      m_predStorage[2];
  PelStorage      m_fusionStorage[6];
#endif

#if JVET_AJ0081_CHROMA_TMRL
  PelStorage     m_chromaMrlStorage[CHROMA_TMRL_LIST_SIZE];
#endif

#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
  struct crossComponentPredictionModeInfo
  {
    int    bufIdx;
    int    intraModeIndex; // indicate which intra mode
    int    ccpModeIndex;   // 1:regular cccm, 2~4:cccm-mdf
                           // 12:noSubCccm, 13:glcccm, 14:BvgCccm, 15:ddccp, 16:glm
                           // 21:ccpMergeMode, 22:ccpMergeMode fusions
                           // 31:slopeAdj
                           // 41:intraFusionMode
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
                           // 51:ccpMergeMode with adjustment
#endif
    int    isLbccp;
    int    isMrl;
    double cost = MAX_DOUBLE;
  };
#endif

#if JVET_AD0120_LBCCP
  PelStorage      m_lmPredFiltStorage[LBCCP_FILTER_MMLMNUM];
  struct lmPredFiltModeInfo
  {
    int    bufIdx;
    int    isCccm;
    int    isCccmNoSub;
    int    isGlcccm;
    int    cccmMdfIdx;
    double cost;
  };
#endif

#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  PelStorage      m_predCCPFusionStorage[2];
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  Distortion m_backupMinSadHad[MTMP_NUM];
  Distortion m_backupSadCost[MTMP_NUM];
  Distortion m_backupLicMinSadHad[MTMP_NUM][4];
  Distortion m_backupLicSadCost[MTMP_NUM][4];
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiRdModeListFracTmp;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_candCostListFracTmp;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiRdModeListLicFracTmp;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_candCostListLicFracTmp;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiRdModeListTmp;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_candCostListTmp;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> m_uiRdModeListTmpLic;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_candCostListTmpLic;
#endif

protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  BilateralFilter* m_bilateralFilter;
#endif
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
  EncReshape*     m_pcReshape;

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_ctxCache;

  bool            m_isInitialized;
#if !JVET_AJ0237_INTERNAL_12BIT
  uint32_t        m_symbolSize;
  uint16_t**      m_truncBinBits;
  uint16_t*       m_escapeNumBins;
#endif
  bool            m_bestEscape;
  double*         m_indexError[MAXPLTSIZE + 1];
  uint8_t*        m_minErrorIndexMap; // store the best index in terms of distortion for each pixel
  uint8_t         m_indexMapRDOQ   [2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  bool            m_runMapRDOQ     [2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  uint8_t*        m_statePtRDOQ    [NUM_TRELLIS_STATE];
  bool            m_prevRunTypeRDOQ[2][NUM_TRELLIS_STATE];
  int             m_prevRunPosRDOQ [2][NUM_TRELLIS_STATE];
  double          m_stateCostRDOQ  [2][NUM_TRELLIS_STATE];
public:
#if INTRA_TRANS_ENC_OPT
  bool            m_skipTimdLfnstMtsPass;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  bool            m_skipSgpmLfnstMtsPass;
#endif
#if JVET_AJ0061_TIMD_MERGE || JVET_AM0074_INTRA_MERGE
  bool            m_skipTimdMrgLfnstMtsPass;
  bool            m_skipObicMode;
  bool            m_skipDimdMode;
  uint64_t        m_satdCostOBIC;
  uint64_t        m_satdCostDIMD;
  bool            m_skipTimdMode[NumTimdMode];
  uint64_t        m_satdCostTIMD[NumTimdMode][2]; 
#endif
#if JVET_AM0074_INTRA_MERGE
  Pel* m_intraMergePredBuf[MAX_NUM_INTRA_MERGE_CAND];
  Pel* m_immBvPredBuf[DIMD_FUSION_NUM];
  int m_intraMergeCandListSize;
  IntraMergeCandidate m_intraMergeCand[MAX_NUM_INITIAL_INTRA_MERGE_CAND];
  bool            m_skipIntraMergeMode;
  uint64_t        m_satdCostIntraMerge;
  double          m_dSavedSadHadRdCostIntraMerge[MAX_NUM_INTRA_MERGE_CAND];
  double          m_dSavedSadHadRdCostObic{ MAX_DOUBLE };
  bool            m_skipIntraMergeLfnstMtsPass;
#endif
#if JVET_AH0076_OBIC
  bool            m_skipObicLfnstMtsPass;
  bool            m_skipDimdLfnstMtsPass;
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool            m_skipNnLfnstMtsPass;
#endif
#if JVET_AL0108_BVG_DIMD
  bool            m_skipBvgDimdLfnstMtsPass;
  bool            m_skipBvgDimd;
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  bool            m_skipCCCMSATD;
  int             m_isCccmNoSubModeEnabledInRdo[MMLM_T_IDX + 1];
#endif 
#if JVET_AJ0082_MM_EIP
  bool            m_skipEipLfnstMtsPass;
#endif
#if JVET_AD0202_CCCM_MDF
  bool            m_skipCCCMwithMdfSATD;
  int             m_isCccmWithMdfEnabledInRdo[5][MMLM_T_IDX + 1];
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  bool            m_skipDdCcpListConstruction;
  bool            firstTransformDdccp;
  PelStorage      m_ddCcpStorage;
  PelUnitBuf      m_ddCcpStorageTemp;
  std::vector<DecoderDerivedCcpCandidate> m_decoderDerivedCcpList;
  bool            m_skipDdCcpMergeFusionList;
  int             m_numCcpMergefusionRdo;
#if JVET_AL0191_INTRA_CHROMA_ENCOPT_CCP_CONSTRAINTS
  double m_ddccpMergeFusionCost[5];
  int m_ddCcpMergeFusionModeIndex[5];
  PelStorage      m_ddCcpFusionStorage[5];
  PelUnitBuf      m_ddCcpFusionStorageTemp[5];
#else
  double          m_ddccpMergeFusionCost[2];
  int             m_ddCcpMergeFusionModeIndex[2];
  PelStorage      m_ddCcpFusionStorage[2];
  PelUnitBuf      m_ddCcpFusionStorageTemp[2];
#endif
#endif  
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool            m_isDimdPredictionSaved;
  bool            m_isObicPredictionSaved;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  bool            m_skipCcpMergeAdjustSatd;
  int             m_isCcpMergeAdjustEnabledInRdo[MAX_CCP_CAND_LIST_SIZE << 1];
  bool m_skipCcpMergeAdjustListConstruction;
  int m_ccpMergeAdjustListSize;
  CCPModelCandidate m_candListAdjust[MAX_CCP_CAND_LIST_INIT_SIZE << 1];

  bool m_skipCcpMergeListConstruction;
  int m_ccpMergeListSize;
  CCPModelCandidate m_candListCcpMerge[MAX_CCP_CAND_LIST_SIZE];
  int m_fusionList[MAX_CCP_FUSION_NUM * 2];
#endif
  IntraSearch();
  ~IntraSearch();

  void init                       ( EncCfg*        pcEncCfg,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                                   BilateralFilter* bilateralFilter,
#endif
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const uint32_t     maxCUWidth,
                                    const uint32_t     maxCUHeight,
                                    const uint32_t     maxTotalCUDepth
                                  , EncReshape*   m_pcReshape
                                  , const unsigned bitDepthY
                                  );

  void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

  void setModeCtrl                ( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl; }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool getSaveCuCostInSCIPU       ()               { return m_saveCuCostInSCIPU; }
  void setSaveCuCostInSCIPU       ( bool b )       { m_saveCuCostInSCIPU = b;  }
  void setNumCuInSCIPU            ( uint8_t i )    { m_numCuInSCIPU = i; }
  void saveCuAreaCostInSCIPU      ( Area area, double cost );
  void initCuAreaCostInSCIPU      ();
  double findInterCUCost          ( CodingUnit &cu );
#endif
public:
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  void resetIndicesRepresentationPnnMemories();
  void resetAreDimdObicPredictionsSaved() { m_isDimdPredictionSaved = false; m_isObicPredictionSaved = false; }
#endif
  bool estIntraPredLumaQT(CodingUnit &cu, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false, CodingStructure* bestCS = NULL
#if JVET_AG0136_INTRA_TMP_LIC
    , InterPrediction* pcInterPred = nullptr
#endif
  );
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm, const double maxCostAllowed = MAX_DOUBLE 
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                  , InterPrediction* pcInterPred = nullptr
#endif
  );
  void PLTSearch                  ( CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  uint64_t xFracModeBitsIntra     (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);
  void invalidateBestModeCost     () { for( int i = 0; i < NUM_LFNST_NUM_PER_SET; i++ ) m_bestModeCostValid[ i ] = false; };

  void sortRdModeListFirstColorSpace(ModeInfo mode, double cost, char bdpcmMode, ModeInfo* rdModeList, double* rdCostList, char* bdpcmModeList, int& candNum);
  void invalidateBestRdModeFirstColorSpace();
  void setSavedRdModeIdx(int idx) { m_savedRdModeIdx = idx; }
#if JVET_AD0120_LBCCP
  void fillLmPredFiltList(PredictionUnit pu, const PelUnitBuf& lmPredFilt, int &lmPredFiltIdx, std::vector<lmPredFiltModeInfo> &miLmPredFiltList);
#endif

#if SECONDARY_MPM
  int& getMpmListSize()           { return m_mpmListSize; }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  double getBestIntraSADHADCost() { return m_bestIntraSADHADCost; }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  double getBestIntraSADCost()    { return m_bestIntraSADCost; }
#endif
  void setLumaIntraPredIdx(PredictionUnit& pu);
#if JVET_AG0058_EIP
  EipModelCandidate m_eipModel[NUM_DERIVED_EIP];
  EipModelCandidate m_eipMergeModel[MAX_MERGE_EIP];
#if JVET_AL0106_BV_EIP
  EipModelCandidate m_eipBvModel[MAX_BV_EIP];
#endif
#endif
protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------


  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  void     xEncIntraHeader                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1 );
  void     xEncSubdivCbfQT                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQT                     ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, CUCtx * cuCtx = nullptr  );
  uint64_t xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner& pm, const ComponentID compID );

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
  void xEncCoeffQT                                 ( CodingStructure &cs, Partitioner& pm, const ComponentID compID, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, CUCtx * cuCtx = nullptr );

  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr, std::vector<TrMode>* trModes=nullptr, const bool loadTr=false 
#if JVET_AG0136_INTRA_TMP_LIC
    , InterPrediction* pcInterPred=nullptr
#endif
  );
  void xIntraCodingACTTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, std::vector<TrMode>* trModes = nullptr, const bool loadTr = false);
#if JVET_W0103_INTRA_MTS
  void xSelectAMTForFullRD(TransformUnit &tu
#if JVET_AG0136_INTRA_TMP_LIC
    , InterPrediction* pcInterPred=nullptr
#endif
  );
  bool testISPforCurrCU(const CodingUnit &cu);
#endif
  ChromaCbfs xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE,                          const PartSplit ispType = TU_NO_ISP 
#if JVET_AB0143_CCCM_TS || JVET_AC0119_LM_CHROMA_FUSION
    , const PelUnitBuf& predStorage = UnitBuf<Pel>()
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    , InterPrediction* pcInterPred = nullptr
#endif
  );
  bool       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP, const bool ispIsCurrentWinner = false, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false 
#if JVET_AG0136_INTRA_TMP_LIC
    , InterPrediction* pcInterPred=NULL
#endif
  );
  bool       xRecurIntraCodingACTQT(CodingStructure &cs, Partitioner& pm, bool mtsCheckRangeFlag = false, int mtsFirstCheckId = 0, int mtsLastCheckId = 0, bool moreProbMTSIdxFirst = false);
  bool       xIntraCodingLumaISP      ( CodingStructure& cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE );

  template<typename T, size_t N>
  void reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip
#if JVET_AB0157_TMRL
    , const double* tmrlCostList
#endif
#if JVET_AC0105_DIRECTIONAL_PLANAR
    , const double* dirPlanarCostList
#endif
#if JVET_AJ0146_TIMDSAD
    , int addOne
#endif
  );
  void   derivePLTLossy  (      CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void   calcPixelPred   (      CodingStructure& cs, Partitioner& partitioner, uint32_t    yPos,      uint32_t xPos,             ComponentID compBegin, uint32_t  numComp);
  void     preCalcPLTIndexRD      (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
  void     calcPixelPredRD        (CodingStructure& cs, Partitioner& partitioner, Pel* orgBuf, Pel* pixelValue, Pel* recoValue, ComponentID compBegin, uint32_t numComp);
  void     deriveIndexMap         (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, double& dCost, bool* idxExist);
  bool     deriveSubblockIndexMap(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, PLTScanMode pltScanMode, int minSubPos, int maxSubPos, const BinFracBits& fracBitsPltRunType, const BinFracBits* fracBitsPltIndexINDEX, const BinFracBits* fracBitsPltIndexCOPY, const double minCost, bool useRotate);
  double   rateDistOptPLT         (bool RunType, uint8_t RunIndex, bool prevRunType, uint8_t prevRunIndex, uint8_t aboveRunIndex, bool& prevCodedRunType, int& prevCodedRunPos, int scanPos, uint32_t width, int dist, int indexMaxValue, const BinFracBits* IndexfracBits, const BinFracBits& TypefracBits);
#if !JVET_AJ0237_INTERNAL_12BIT
  void     initTBCTable           (int bitDepth);
#endif
  uint32_t getTruncBinBits        (uint32_t symbol, uint32_t maxSymbol);
  uint32_t getEpExGolombNumBins   (uint32_t symbol, uint32_t count);
#if AHG7_LN_TOOLOFF_CFG
  void xGetNextISPMode                    ( ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize, bool lfnstExtFlag, bool nsptFlag );
#else
  void xGetNextISPMode                    ( ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize );
#endif
  bool xSortISPCandList                   ( double bestCostSoFar, double bestNonISPCost, ModeInfo bestNonISPMode );
  void xSortISPCandListLFNST              ( );
  void xFindAlreadyTestedNearbyIntraModes ( int currentLfnstIdx, int currentIntraMode, int* refLfnstIdx, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize );
  bool updateISPStatusFromRelCU           ( double bestNonISPCostCurrCu, ModeInfo bestNonISPModeCurrCu, int& bestISPModeInRelCU );
  void xFinishISPModes                    ( );
#if JVET_Z0050_CCLM_SLOPE
  void xFindBestCclmDeltaSlopeSATD        ( PredictionUnit &pu, ComponentID compID, int cclmModel, int &deltaBest, int64_t &satdBest );
#endif
#if JVET_AA0126_GLM
  void xFindBestGlmIdcSATD                ( PredictionUnit &pu, ComponentID compID, int &idcBest, int64_t &satdBest );
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  void getPredForCCPMrgFusion(PredictionUnit& pu, PelBuf& predCb, PelBuf& predCr);
  void xCalcCcpMrgPred(const PredictionUnit& pu, const ComponentID compID, PelBuf& piPredNonLm, PelBuf& piPredLm);
#endif
#if JVET_AM0074_INTRA_MERGE
  void updateIntraMergeModeTest(static_vector<std::pair<int32_t, double>, FAST_UDI_MAX_RDMODE_NUM> m_uiRdModeCostList);
#endif
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
