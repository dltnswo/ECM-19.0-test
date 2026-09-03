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

/** \file     MotionInfo.h
    \brief    motion information handling classes (header)
    \todo     MvField seems to be better to be inherited from Mv
*/

#ifndef __MOTIONINFO__
#define __MOTIONINFO__

#include "CommonDef.h"
#include "Mv.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Type definition
// ====================================================================================================================
#if JVET_AL0188_SGPM_FLIPAWARE_BV
struct BvInfo
{
  Mv   mv;
  int  flipType;
#if JVET_AM0157_SGPM_BV_LIC
  bool licFlag;
  bool mmLicFlag;
  int  mean;
  int  licScale[2];
  int  licShift[2];
  int  licOffset[2];
  BvInfo() :mv(0, 0), flipType(0), licFlag(false), mmLicFlag(false), mean(0)
  {
    for (int i = 0; i < 2; i++)
    {
      licScale[i] = 0;
      licShift[i] = 0;
      licOffset[i] = 0;
    }
  }
  BvInfo(Mv bv, int flip, bool lic = false, int licScale0 = 0, int licShift0 = 0,int licOffset0 = 0, int licScale1 = 0, int licShift1 = 0, int licOffset1 = 0, bool mmLic = false, int meanValue = 0) :mv(bv), flipType(flip), licFlag(lic), mmLicFlag(mmLic), mean(meanValue)
  {
    licScale[0] = licScale0;
    licShift[0] = licShift0;
    licOffset[0] = licOffset0;
    licScale[1] = licScale1;
    licShift[1] = licShift1;
    licOffset[1] = licOffset1;
  }
  bool operator!=(const BvInfo& other) const
  {
    if ((mv != other.mv) || (flipType != other.flipType) || (licFlag != other.licFlag) || (mmLicFlag != other.mmLicFlag))
    {
      return true;
    }
    if ((licScale[0] != other.licScale[0]) || (licScale[1] != other.licScale[1]) || (licOffset[0] != other.licOffset[0]) || (licOffset[1] != other.licOffset[1]) || (licShift[0] != other.licShift[0]) || (licShift[1] != other.licShift[1]) || mean != other.mean)
    {
      return true;
    }
    return false;
  }
  bool operator==(const BvInfo& other) const
  {
    if ((mv != other.mv) || (flipType != other.flipType) || (licFlag != other.licFlag) || (mmLicFlag != other.mmLicFlag))
    {
      return false;
    }
    if ((licScale[0] != other.licScale[0]) || (licScale[1] != other.licScale[1]) || (licOffset[0] != other.licOffset[0]) || (licOffset[1] != other.licOffset[1]) || (licShift[0] != other.licShift[0]) || (licShift[1] != other.licShift[1]) || mean != other.mean)
    {
      return false;
    }
    return true;
  }
#else
  BvInfo() :mv(0, 0), flipType(0) {}
  BvInfo(Mv bv, int flip) :mv(bv), flipType(flip) {}
  bool operator!=(const BvInfo& other)      const { return (mv != other.mv) || (flipType != other.flipType); }
  bool operator==(const BvInfo& other)      const { return (mv == other.mv) && (flipType == other.flipType); }
#endif
};
#endif

#if MULTI_HYP_PRED 
struct MultiHypPredictionData
{
  int8_t   imv;
#if INTER_LIC
  bool     licFlag;
#endif
  bool     isMrg;
  int8_t   mrgIdx;
  int8_t   refList;
  int      mvpIdx;
  int8_t   refIdx;
  int8_t   weightIdx;
  Mv       mv;
  Mv       mvd;
};

typedef static_vector<MultiHypPredictionData, MULTI_HYP_PRED_MAX_CANDS> MultiHypVec;
#endif

#if JVET_AI0183_MVP_EXTENSION
struct IntersectingMvData
{
  int8_t intersectingNr;
  int8_t intersectingRefIdx[INTERSECTING_MV_MAX_NR][2];
  Mv     intersectingMv[INTERSECTING_MV_MAX_NR][2];
};
#endif

/// parameters for AMVP
struct AMVPInfo
{
#if TM_AMVP || (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  Mv       mvCand[REGULAR_AMVP_MAX_NUM_CANDS + 1];  ///< array of motion vector predictor candidates
#else
  Mv       mvCand[AMVP_MAX_NUM_CANDS_MEM];  ///< array of motion vector predictor candidates
#endif
  unsigned numCand;                       ///< number of motion vector predictor candidates
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  Distortion mvCost[REGULAR_AMVP_MAX_NUM_CANDS + 1];
#endif
#if TM_AMVP || (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  unsigned maxStorageSize;
  unsigned maxSimilarityThreshold;

  AMVPInfo() : numCand(0), maxStorageSize(AMVP_MAX_NUM_CANDS), maxSimilarityThreshold(0) {}

  bool xCheckSimilarMotion(const int mvpIdx)
  {
    if (maxSimilarityThreshold == 1)
    {
      for (uint32_t ui = 0; ui < mvpIdx; ui++)
      {
        if (mvCand[ui] == mvCand[mvpIdx])
        {
          return true;
        }
      }
    }
    else if (maxSimilarityThreshold > 1)
    {
      for (uint32_t ui = 0; ui < mvpIdx; ui++)
      {
        Mv mvDiff = mvCand[ui] - mvCand[mvpIdx];
        if (mvDiff.getAbsHor() < maxSimilarityThreshold && mvDiff.getAbsVer() < maxSimilarityThreshold)
        {
          return true;
        }
      }
    }
   
    return false;
  }
#endif
};

struct AffineAMVPInfo
{
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  unsigned maxStorageSize;
  Mv       mvCandLT[ AFFINE_AMVP_MAX_CAND ];  ///< array of affine motion vector predictor candidates for left-top corner
  Mv       mvCandRT[ AFFINE_AMVP_MAX_CAND ];  ///< array of affine motion vector predictor candidates for right-top corner
  Mv       mvCandLB[ AFFINE_AMVP_MAX_CAND ];  ///< array of affine motion vector predictor candidates for left-bottom corner
#else
  Mv       mvCandLT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-top corner
  Mv       mvCandRT[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for right-top corner
  Mv       mvCandLB[ AMVP_MAX_NUM_CANDS_MEM ];  ///< array of affine motion vector predictor candidates for left-bottom corner
#endif
  unsigned numCand;                       ///< number of motion vector predictor candidates
};
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
struct RMVFInfo
{
  Mv mvp;
  Position pos;
  int8_t refIdx;
  RMVFInfo() : mvp(Mv(0, 0)), pos(Position(0, 0)), refIdx(-1) {};
  RMVFInfo(Mv const cMv, Position const cPos, int8_t const cIdx) : mvp(cMv), pos(cPos), refIdx(cIdx) {};
};
#endif
// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// class for motion vector with reference index
struct MvField
{
  Mv    mv;
  int8_t refIdx;

  MvField()                                    :            refIdx( NOT_VALID ) {}
  MvField( Mv const & cMv, const int iRefIdx ) : mv( cMv ), refIdx(   iRefIdx ) {}

  void setMvField( Mv const & cMv, const int iRefIdx )
  {
    CHECK( iRefIdx == -1 && cMv != Mv(0,0), "Must not happen." );
    mv     = cMv;
    refIdx = iRefIdx;
  }

  bool operator==( const MvField& other ) const
  {
    CHECK( refIdx == -1 && mv != Mv(0,0), "Error in operator== of MvField." );
    CHECK( other.refIdx == -1 && other.mv != Mv(0,0), "Error in operator== of MvField." );
    return refIdx == other.refIdx && mv == other.mv;
  }
  bool operator!=( const MvField& other ) const
  {
    CHECK( refIdx == -1 && mv != Mv(0,0), "Error in operator!= of MvField." );
    CHECK( other.refIdx == -1 && other.mv != Mv(0,0), "Error in operator!= of MvField." );
    return refIdx != other.refIdx || mv != other.mv;
  }
};

struct MotionInfo
{
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool isSCC;
  bool isRefSCC;
  bool isRefRefSCC;
#endif
  bool     isInter;
  bool     isIBCmot;
  char     interDir;
  bool     useAltHpelIf;
  uint16_t   sliceIdx;
  Mv      mv     [ NUM_REF_PIC_LIST_01 ];
  int8_t   refIdx [ NUM_REF_PIC_LIST_01 ];
  uint8_t         bcwIdx;
#if INTER_LIC
  bool     usesLIC;
#endif
#if JVET_AC0112_IBC_LIC
  bool     useIbcLic;
#if JVET_AE0078_IBC_LIC_EXTENSION
  int      ibcLicIdx;
#endif
#endif
#if JVET_AE0159_FIBC
  bool      useIbcFilter;
#endif
#if JVET_AA0070_RRIBC
  int  rribcFlipType;
  Position centerPos;
#endif
#if MULTI_HYP_PRED  
  MultiHypVec addHypData;
#endif
  Mv      bv;
#if JVET_AI0197_AFFINE_TMVP
  bool   isAffine;
  int8_t affineType;
  int    y;
  int    x;
  int    height;
  int    width;
#endif
#if JVET_AG0164_AFFINE_GPM
  int8_t     gpmPartIdx;
#if JVET_AI0197_AFFINE_TMVP
  MotionInfo()
    : isSCC(false)
    , isRefSCC(false)
    , isRefRefSCC(false)
    , isInter(false)
    , isIBCmot(false)
    , interDir(0)
    , useAltHpelIf(false)
    , sliceIdx(0)
    , refIdx{ NOT_VALID, NOT_VALID }
    , bcwIdx(0)
    , usesLIC(false)
    , isAffine(false)
    , affineType(0)
    , y(-1)
    , x(-1)
    , height(0)
    , width(0)
    , gpmPartIdx(-1)
  {
  }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this
  // constructor for anything else
  MotionInfo(int i)
    : isSCC(false)
    , isRefSCC(false)
    , isRefRefSCC(false)
    , isInter(i != 0)
    , isIBCmot(false)
    , interDir(0)
    , useAltHpelIf(false)
    , sliceIdx(0)
    , refIdx{ 0, 0 }
    , bcwIdx(0)
    , usesLIC(false)
    , isAffine(false)
    , affineType(0)
    , y(-1)
    , x(-1)
    , height(0)
    , width(0)
    , gpmPartIdx(-1)
  {
    CHECKD(i != 0, "The argument for this constructor has to be '0'");
  }
#else
  MotionInfo() : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(false), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ NOT_VALID, NOT_VALID }, bcwIdx(0), usesLIC(false),gpmPartIdx(-1)
  {
  }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this constructor for anything else
  MotionInfo(int i) : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(i != 0), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ 0,         0 }, bcwIdx(0), usesLIC(false), gpmPartIdx(-1){ CHECKD(i != 0, "The argument for this constructor has to be '0'"); }
#endif
#else
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if INTER_LIC
  MotionInfo() : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(false), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ NOT_VALID, NOT_VALID }, bcwIdx(0), usesLIC(false)
  { 
  }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this constructor for anything else
  MotionInfo(int i) : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(i != 0),isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ 0,         0 }, bcwIdx(0), usesLIC(false) { CHECKD(i != 0, "The argument for this constructor has to be '0'"); }
#else
  MotionInfo() : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(false), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ NOT_VALID, NOT_VALID }, bcwIdx(0) { }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this constructor for anything else
  MotionInfo(int i) : isSCC(false), isRefSCC(false), isRefRefSCC(false), isInter(i != 0), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ 0,         0 }, bcwIdx(0) { CHECKD(i != 0, "The argument for this constructor has to be '0'"); }
#endif
#else
#if INTER_LIC
  MotionInfo() : isInter(false), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ NOT_VALID, NOT_VALID }, bcwIdx(0), usesLIC(false)
  {
  }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this constructor for anything else
  MotionInfo(int i) : isInter(i != 0), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ 0,         0 }, bcwIdx(0), usesLIC(false) { CHECKD(i != 0, "The argument for this constructor has to be '0'"); }
#else
  MotionInfo() : isInter(false), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ NOT_VALID, NOT_VALID }, bcwIdx(0) { }
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this constructor for anything else
  MotionInfo(int i) : isInter(i != 0), isIBCmot(false), interDir(0), useAltHpelIf(false), sliceIdx(0), refIdx{ 0,         0 }, bcwIdx(0) { CHECKD(i != 0, "The argument for this constructor has to be '0'"); }
#endif
#endif
#endif
  bool operator==( const MotionInfo& mi ) const
  {
    if( isInter != mi.isInter  ) return false;
    if (isIBCmot != mi.isIBCmot) return false;
    if( isInter )
    {
      if( sliceIdx != mi.sliceIdx ) return false;
      if( interDir != mi.interDir ) return false;

      if( interDir != 2 )
      {
        if( refIdx[0] != mi.refIdx[0] ) return false;
        if( mv[0]     != mi.mv[0]     ) return false;
      }

      if( interDir != 1 )
      {
        if( refIdx[1] != mi.refIdx[1] ) return false;
        if( mv[1]     != mi.mv[1]     ) return false;
      }
    }

    return true;
  }

  bool operator!=( const MotionInfo& mi ) const
  {
    return !( *this == mi );
  }
};

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
struct RefineTmvpSbTmvpParams
{
  bool     refineTmvpValid;
  int16_t  refineTmvpCol;
  int16_t  refineTmvpSubIdx;
  int16_t  refineTmvpFixRefIdx;
  int16_t  refineTmvpSbTmvpType;
  bool     refineTmvpIsRefined;
};
struct RefineTmvpSbTmvpSubParams
{
  RefPicList         refineTmvpSubRefList[NUM_REF_PIC_LIST_01];
  int16_t            refineTmvpSubPosition[NUM_REF_PIC_LIST_01][2];
  int16_t            refineTmvpSubTargetRefIdx[NUM_REF_PIC_LIST_01];
  RefPicList         refineTmvpSubColRefPicList[NUM_REF_PIC_LIST_01];
  int16_t            refineTmvpSubColRefPOC[NUM_REF_PIC_LIST_01];
  const MotionInfo*  refineTmvpSubMi[NUM_REF_PIC_LIST_01];
};
struct RefineTmvpTmvpFirstPassParams
{
  bool                refineTmvpNeedDoRefine;
  RefPicList          refineTmvpColRefPicList;
  int16_t             refineTmvpColRefPOC;
  const MotionInfo*   refineTmvpMiPtr;
};
struct RefineTmvpCtxModellingParams
{
  RefPicList  refineTmvpRefList[NUM_MERGE_CANDS << 1];
  bool        refineTmvpSbFlag[NUM_MERGE_CANDS << 1];
  int16_t     refineTmvpPosition[NUM_MERGE_CANDS << 1][2];
  int16_t     refineTmvpRefIdx[NUM_MERGE_CANDS << 1];
  int16_t     refineTmvpTargetRefIdx[NUM_MERGE_CANDS << 1];
  int16_t     refineTmvpCol[NUM_MERGE_CANDS << 1];
  RefPicList  refineTmvpColRefPicList[NUM_MERGE_CANDS << 1];
  int16_t     refineTmvpColRefPOC[NUM_MERGE_CANDS << 1];
  const MotionInfo* refineTmvpMiPtr[NUM_MERGE_CANDS << 1];
};
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
struct MotionInfoPred
{
  int8_t     interDir;
  int8_t     refIdx[2];
  Mv         mv[2];
  Mv         mvd[2];
  Mv         mvAffi[2][3];
  Mv         mvdAffi[2][3];
  Distortion cost;
#if JVET_AD0140_MVD_PREDICTION
  bool       aboveCostCalculated;
  bool       leftCostCalculated;
#endif
};
#endif

class BcwMotionParam
{
  bool       m_readOnly[2][33];       // 2 RefLists, 33 RefFrams
  Mv         m_mv[2][33];
  Distortion m_dist[2][33];

  bool       m_readOnlyAffine[2][2][33];
  Mv         m_mvAffine[2][2][33][3];
  Distortion m_distAffine[2][2][33];
  int        m_mvpIdx[2][2][33];

public:

  void reset()
  {
    Mv* pMv = &(m_mv[0][0]);
    for (int ui = 0; ui < 1 * 2 * 33; ++ui, ++pMv)
    {
      pMv->set(std::numeric_limits<int16_t>::max(), std::numeric_limits<int16_t>::max());
    }

    Mv* pAffineMv = &(m_mvAffine[0][0][0][0]);
    for (int ui = 0; ui < 2 * 2 * 33 * 3; ++ui, ++pAffineMv)
    {
      pAffineMv->set(0, 0);
    }

    memset(m_readOnly, false, 2 * 33 * sizeof(bool));
    memset(m_dist, -1, 2 * 33 * sizeof(Distortion));
    memset(m_readOnlyAffine, false, 2 * 2 * 33 * sizeof(bool));
    memset(m_distAffine, -1, 2 * 2 * 33 * sizeof(Distortion));
    memset( m_mvpIdx, 0, 2 * 2 * 33 * sizeof( int ) );
  }

  void setReadMode(bool b, uint32_t uiRefList, uint32_t uiRefIdx) { m_readOnly[uiRefList][uiRefIdx] = b; }
  bool isReadMode(uint32_t uiRefList, uint32_t uiRefIdx) { return m_readOnly[uiRefList][uiRefIdx]; }

  void setReadModeAffine(bool b, uint32_t uiRefList, uint32_t uiRefIdx, int bP4) { m_readOnlyAffine[bP4][uiRefList][uiRefIdx] = b; }
  bool isReadModeAffine(uint32_t uiRefList, uint32_t uiRefIdx, int bP4) { return m_readOnlyAffine[bP4][uiRefList][uiRefIdx]; }

  Mv&  getMv(uint32_t uiRefList, uint32_t uiRefIdx) { return m_mv[uiRefList][uiRefIdx]; }

  void copyFrom(Mv& rcMv, Distortion uiDist, uint32_t uiRefList, uint32_t uiRefIdx)
  {
    m_mv[uiRefList][uiRefIdx] = rcMv;
    m_dist[uiRefList][uiRefIdx] = uiDist;
  }

  void copyTo(Mv& rcMv, Distortion& ruiDist, uint32_t uiRefList, uint32_t uiRefIdx)
  {
    rcMv = m_mv[uiRefList][uiRefIdx];
    ruiDist = m_dist[uiRefList][uiRefIdx];
  }

  Mv& getAffineMv(uint32_t uiRefList, uint32_t uiRefIdx, uint32_t uiAffineMvIdx, int bP4) { return m_mvAffine[bP4][uiRefList][uiRefIdx][uiAffineMvIdx]; }

  void copyAffineMvFrom(Mv(&racAffineMvs)[3], Distortion uiDist, uint32_t uiRefList, uint32_t uiRefIdx, int bP4
                        , const int mvpIdx
  )
  {
    memcpy(m_mvAffine[bP4][uiRefList][uiRefIdx], racAffineMvs, 3 * sizeof(Mv));
    m_distAffine[bP4][uiRefList][uiRefIdx] = uiDist;
    m_mvpIdx[bP4][uiRefList][uiRefIdx]     = mvpIdx;
  }

  void copyAffineMvTo(Mv acAffineMvs[3], Distortion& ruiDist, uint32_t uiRefList, uint32_t uiRefIdx, int bP4
                      , int& mvpIdx
  )
  {
    memcpy(acAffineMvs, m_mvAffine[bP4][uiRefList][uiRefIdx], 3 * sizeof(Mv));
    ruiDist = m_distAffine[bP4][uiRefList][uiRefIdx];
    mvpIdx  = m_mvpIdx[bP4][uiRefList][uiRefIdx];
  }
};

#if JVET_Z0139_HIST_AFF
struct AffineMotionInfo
{
  union
  {
    uint64_t oneSetAffineParametersPattern;
    short oneSetAffineParameters[4];
  };
  AffineMotionInfo(uint64_t p) { oneSetAffineParametersPattern = p; };
  AffineMotionInfo() { oneSetAffineParametersPattern = 0; };

  bool operator == (const AffineMotionInfo& motInfo)
  {
    return oneSetAffineParametersPattern == motInfo.oneSetAffineParametersPattern;
  }
};

struct AffineInheritInfo
{
  Position basePos;
  MvField  baseMV[2];

  union
  {
    uint64_t oneSetAffineParametersPattern0;
    short oneSetAffineParameters0[4];
  };
  union
  {
    uint64_t oneSetAffineParametersPattern1;
    short oneSetAffineParameters1[4];
  };
  AffineInheritInfo() { baseMV[0].refIdx = baseMV[1].refIdx = -1; };

  bool operator == (const AffineInheritInfo& motInfo)
  {
    if (baseMV[0].refIdx != motInfo.baseMV[0].refIdx)
    {
      return false;
    }
    if (baseMV[1].refIdx != motInfo.baseMV[1].refIdx)
    {
      return false;
    }
    if (baseMV[0].refIdx != -1 && oneSetAffineParametersPattern0 != motInfo.oneSetAffineParametersPattern0)
    {
      return false;
    }
    if (baseMV[1].refIdx != -1 && oneSetAffineParametersPattern1 != motInfo.oneSetAffineParametersPattern1)
    {
      return false;
    }
    return true;
  }

};
#endif

struct LutMotionCand
{
#if JVET_Z0118_GDR
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lut0;
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lut1;

#if JVET_Z0075_IBC_HMVP_ENLARGE
  static_vector<MotionInfo, MAX_NUM_HMVP_IBC_CANDS> lutIbc0;
  static_vector<MotionInfo, MAX_NUM_HMVP_IBC_CANDS> lutIbc1;
#else
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lutIbc0;
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lutIbc1;
#endif

#if JVET_Z0139_HIST_AFF
  static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> lutAff0[2 * MAX_NUM_AFFHMVP_ENTRIES_ONELIST];
  static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> lutAff1[2 * MAX_NUM_AFFHMVP_ENTRIES_ONELIST];
  static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> lutAffInherit0;
  static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> lutAffInherit1;
#endif
#else
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lut;
#if JVET_Z0075_IBC_HMVP_ENLARGE
  static_vector<MotionInfo, MAX_NUM_HMVP_IBC_CANDS> lutIbc;
#else
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> lutIbc;
#endif

#if JVET_Z0139_HIST_AFF
  static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> lutAff[2 * MAX_NUM_AFFHMVP_ENTRIES_ONELIST];
  static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> lutAffInherit;
#endif
#endif
};

struct PatentBvCand
{
  Mv m_bvCands[IBC_NUM_CANDIDATES];
  int currCnt;
};
#endif // __MOTIONINFO__
