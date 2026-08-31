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

/** \file     Mv.h
    \brief    motion vector class (header)
*/

#ifndef __MV__
#define __MV__

#include "CommonDef.h"
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0147_CCCM_NO_SUBSAMPLING
#include <limits.h>
#endif
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

enum MvPrecision
{
  MV_PRECISION_4PEL     = 0,      // 4-pel
  MV_PRECISION_INT      = 2,      // 1-pel, shift 2 bits from 4-pel
  MV_PRECISION_HALF     = 3,      // 1/2-pel
  MV_PRECISION_QUARTER  = 4,      // 1/4-pel (the precision of regular MV difference signaling), shift 4 bits from 4-pel
  MV_PRECISION_SIXTEENTH = 6,     // 1/16-pel (the precision of internal MV), shift 6 bits from 4-pel
  MV_PRECISION_INTERNAL = 2 + MV_FRACTIONAL_BITS_INTERNAL,
};

/// basic motion vector class
class Mv
{
private:
  static const MvPrecision m_amvrPrecision[4];
  static const MvPrecision m_amvrPrecAffine[3];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  static const MvPrecision m_amvrPrecIbc[4];
#else
  static const MvPrecision m_amvrPrecIbc[3];
#endif

  static const int mvClipPeriod = (1 << MV_BITS);
  static const int halMvClipPeriod = (1 << (MV_BITS - 1));

public:
  int   hor;     ///< horizontal component of motion vector
  int   ver;     ///< vertical component of motion vector

  // ------------------------------------------------------------------------------------------------------------------
  // constructors
  // ------------------------------------------------------------------------------------------------------------------

  Mv(                    ) : hor( 0    ), ver( 0    ) {}
  Mv( int iHor, int iVer ) : hor( iHor ), ver( iVer ) {}

  // ------------------------------------------------------------------------------------------------------------------
  // set
  // ------------------------------------------------------------------------------------------------------------------

  void  set       ( int iHor, int iVer)     { hor = iHor;  ver = iVer; }
  void  setHor    ( int i )                 { hor = i;                 }
  void  setVer    ( int i )                 { ver = i;                 }
  void  setZero   ()                        { hor = ver = 0;           }

  // ------------------------------------------------------------------------------------------------------------------
  // get
  // ------------------------------------------------------------------------------------------------------------------

  int   getHor    () const { return hor;          }
  int   getVer    () const { return ver;          }
  int   getAbsHor () const { return abs( hor );   }
  int   getAbsVer () const { return abs( ver );   }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  template <           int fracBits = MV_FRACTIONAL_BITS_INTERNAL> inline int   getTrHor ()                 const { return (hor >> (fracBits)) << (fracBits); }
  template <           int fracBits = MV_FRACTIONAL_BITS_INTERNAL> inline int   getTrVer ()                 const { return (ver >> (fracBits)) << (fracBits); }
  template <bool luma, int fracBits = MV_FRACTIONAL_BITS_INTERNAL> inline int   getTrHor (int chromaFormat) const { return luma || chromaFormat == CHROMA_444 ? getTrHor<fracBits>() : getTrHor<fracBits + 1>(); }
  template <bool luma, int fracBits = MV_FRACTIONAL_BITS_INTERNAL> inline int   getTrVer (int chromaFormat) const { return luma || chromaFormat != CHROMA_420 ? getTrVer<fracBits>() : getTrVer<fracBits + 1>(); }

private:
                       inline bool  isFracChromaMv420 ()                 const { return hor != getTrHor<MV_FRACTIONAL_BITS_INTERNAL + 1>() || ver != getTrVer<MV_FRACTIONAL_BITS_INTERNAL + 1>(); }
                       inline bool  isFracChromaMv422 ()                 const { return hor != getTrHor<MV_FRACTIONAL_BITS_INTERNAL + 1>() || ver != getTrVer<MV_FRACTIONAL_BITS_INTERNAL    >(); }
public:
                       inline bool  isFracMv          ()                 const { return hor != getTrHor<MV_FRACTIONAL_BITS_INTERNAL    >() || ver != getTrVer<MV_FRACTIONAL_BITS_INTERNAL    >(); }
  template <bool luma> inline int   isFracMv          (int chromaFormat) const { return !luma && chromaFormat == CHROMA_420 ? isFracChromaMv420() : (!luma && chromaFormat == CHROMA_422 ? isFracChromaMv422() : isFracMv()); }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // operations
  // ------------------------------------------------------------------------------------------------------------------

  const Mv& operator += (const Mv& _rcMv)
  {
    hor += _rcMv.hor;
    ver += _rcMv.ver;

    return  *this;
  }

  const Mv& operator-= (const Mv& _rcMv)
  {
    hor -= _rcMv.hor;
    ver -= _rcMv.ver;

    return  *this;
  }


  //! shift right with rounding
  void divideByPowerOf2 (const int i)
  {
    if (i != 0)
    {
      const int offset = (1 << (i - 1));
      hor = (hor + offset - (hor >= 0)) >> i;
      ver = (ver + offset - (ver >= 0)) >> i;
    }
  }

  const Mv& operator<<= (const int i)
  {
    hor <<= i;
    ver <<= i;
    return  *this;
  }

  const Mv& operator>>= ( const int i )
  {
    if (i != 0)
    {
      const int offset = (1 << (i - 1));
      hor = (hor + offset - (hor >= 0)) >> i;
      ver = (ver + offset - (ver >= 0)) >> i;
    }
    return  *this;
  }

  const Mv operator - ( const Mv& rcMv ) const
  {
    return Mv( hor - rcMv.hor, ver - rcMv.ver );
  }

  const Mv operator + ( const Mv& rcMv ) const
  {
    return Mv( hor + rcMv.hor, ver + rcMv.ver );
  }

  bool operator== ( const Mv& rcMv ) const
  {
    return ( hor == rcMv.hor && ver == rcMv.ver );
  }

  bool operator!= ( const Mv& rcMv ) const
  {
    return !( *this == rcMv );
  }

  const Mv scaleMv( int iScale ) const
  {
    const int mvx = Clip3(MV_MIN, MV_MAX, (iScale * getHor() + 128 - (iScale * getHor() >= 0)) >> 8);
    const int mvy = Clip3(MV_MIN, MV_MAX, (iScale * getVer() + 128 - (iScale * getVer() >= 0)) >> 8);
    return Mv( mvx, mvy );
  }

  void changePrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    const int shift = (int)dst - (int)src;
    if (shift >= 0)
    {
      *this <<= shift;
    }
    else
    {
      const int rightShift = -shift;
      const int nOffset = 1 << (rightShift - 1);
      hor = hor >= 0 ? (hor + nOffset - 1) >> rightShift : (hor + nOffset) >> rightShift;
      ver = ver >= 0 ? (ver + nOffset - 1) >> rightShift : (ver + nOffset) >> rightShift;
    }
  }
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  void roundRightShift(int shift)
  {
    CHECK(shift < 0, "shift < 0 for roundRightShift");
    const int nOffset = 1 << (shift - 1);
    hor = hor >= 0 ? (hor + nOffset - 1) >> shift : (hor + nOffset) >> shift;
    ver = ver >= 0 ? (ver + nOffset - 1) >> shift : (ver + nOffset) >> shift;
  }
#endif

  void roundToPrecision(const MvPrecision& src, const MvPrecision& dst)
  {
    changePrecision(src, dst);
    changePrecision(dst, src);
  }

  // translational MV
  void changeTransPrecInternal2Amvr(const int amvr)
  {
    changePrecision(MV_PRECISION_INTERNAL, m_amvrPrecision[amvr]);
  }

  void changeTransPrecAmvr2Internal(const int amvr)
  {
    changePrecision(m_amvrPrecision[amvr], MV_PRECISION_INTERNAL);
  }

  void roundTransPrecInternal2Amvr(const int amvr)
  {
    roundToPrecision(MV_PRECISION_INTERNAL, m_amvrPrecision[amvr]);
  }

  // affine MV
  void changeAffinePrecInternal2Amvr(const int amvr)
  {
    changePrecision(MV_PRECISION_INTERNAL, m_amvrPrecAffine[amvr]);
  }

  void changeAffinePrecAmvr2Internal(const int amvr)
  {
    changePrecision(m_amvrPrecAffine[amvr], MV_PRECISION_INTERNAL);
  }

  void roundAffinePrecInternal2Amvr(const int amvr)
  {
    roundToPrecision(MV_PRECISION_INTERNAL, m_amvrPrecAffine[amvr]);
  }

  // IBC block vector
  void changeIbcPrecInternal2Amvr(const int amvr)
  {
    changePrecision(MV_PRECISION_INTERNAL, m_amvrPrecIbc[amvr]);
  }

  void changeIbcPrecAmvr2Internal(const int amvr)
  {
    changePrecision(m_amvrPrecIbc[amvr], MV_PRECISION_INTERNAL);
  }

  void roundIbcPrecInternal2Amvr(const int amvr)
  {
    roundToPrecision(MV_PRECISION_INTERNAL, m_amvrPrecIbc[amvr]);
  }

#if JVET_AC0104_IBC_BVD_PREDICTION
  static int getImvPrecShift(const uint8_t imv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                           , bool supportFracBv
#endif
  )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    return supportFracBv ? (imv == IMV_OFF || imv == IMV_HPEL ? 0 : 2) : MV_PRECISION_4PEL == m_amvrPrecIbc[imv] ? 2 : 0;
#else
    return MV_PRECISION_4PEL == m_amvrPrecIbc[imv] ? 2 : 0;
#endif
  }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
  void regulateMv(int mvDirType)
  {
    if (mvDirType == 1)
    {
      ver = 0;
    }
    else if (mvDirType == 2)
    {
      hor = 0;
    }
  }
#endif

#if JVET_AD0140_MVD_PREDICTION
  static int getImvPrecShiftMvd(const uint8_t imv)
  {
    return MV_PRECISION_4PEL == m_amvrPrecision[imv] ? 4 
            : (MV_PRECISION_INT == m_amvrPrecision[imv] ? 2
                : (MV_PRECISION_HALF == m_amvrPrecision[imv] ? 1 : 0));
  }

  static int getImvPrecShiftAffineMvd(const uint8_t imv)
  {
    return MV_PRECISION_INT == m_amvrPrecAffine[imv] ? 4 
             : (MV_PRECISION_QUARTER == m_amvrPrecAffine[imv] ? 2 : 0);
  }
#endif

  Mv getSymmvdMv(const Mv& curMvPred, const Mv& tarMvPred)
  {
    return Mv(tarMvPred.hor - hor + curMvPred.hor, tarMvPred.ver - ver + curMvPred.ver);
  }

  void clipToStorageBitDepth()
  {
    hor = Clip3( -(1 << 17), (1 << 17) - 1, hor );
    ver = Clip3( -(1 << 17), (1 << 17) - 1, ver );
  }
  void mvCliptoStorageBitDepth()  // periodic clipping
  {
    hor = (hor + mvClipPeriod) & (mvClipPeriod - 1);
    hor = (hor >= halMvClipPeriod) ? (hor - mvClipPeriod) : hor;
    ver = (ver + mvClipPeriod) & (mvClipPeriod - 1);
    ver = (ver >= halMvClipPeriod) ? (ver - mvClipPeriod) : ver;
  }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  bool isMvdPredApplicable() const
  {
    return (getAbsHor() + getAbsVer()) >= 1;
  }
  
  Mv getAbsMv() { return Mv(abs(hor), abs(ver)); }
#endif
};// END CLASS DEFINITION MV

#if JVET_AC0104_IBC_BVD_PREDICTION || JVET_AD0140_MVD_PREDICTION
#if JVET_AD0140_MVD_PREDICTION
enum class MotionModel 
{ 
  Undefined = -1, 
  UniTranslational = 0, 
  BiTranslational, 
  BiTranslationalSmvd, 
  UniAffine, 
  BiAffine, 
  NumberOfMotionModels 
};
namespace MotionModelCheck
{
  bool isAffine(const MotionModel& mm);
}
#endif

struct MvdSuffixInfo
{
  static const int paramOfGolombCode = 1;
#if JVET_AD0140_MVD_PREDICTION
  MotionModel m_motionModel = MotionModel::Undefined;
#endif
  unsigned horOffsetPrediction = 0;
  unsigned verOffsetPrediction = 0;

  int      horPrefix = -1;
  int      verPrefix = -1;

  int      iBinsInHorSuffix = -1;
  int      iBinsInVerSuffix = -1;

  int      horOffsetPredictionNumBins = -1;
  int      verOffsetPredictionNumBins = -1;

  int      horPrefixGroupStartValue = -1;
  int      verPrefixGroupStartValue = -1;

#if JVET_AD0140_MVD_PREDICTION
  bool     horEncodeSignInEP = true;
  bool     verEncodeSignInEP = true;
#else
  bool     horEncodeSignInEP = false;
  bool     verEncodeSignInEP = false;
#endif

  int      horSignHypMatch = -1;
  int      verSignHypMatch = -1;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool     isFracBvEnabled = false;
#endif

#if JVET_AD0140_MVD_PREDICTION
  bool     isHorMagnZero = true;
  bool     isVerMagnZero = true;
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION 
  void initPrefixes(const Mv &mv, const int imv, const bool isInternalPrecision);

  void initSuffixesAndSigns(const Mv &mv, const int imv);

  void defineNumberOfPredictedBinsInSuffix(const int iHorPrefix, const int iVerPrefix, const uint8_t imv);
#endif

#if JVET_AD0140_MVD_PREDICTION
  void       setMotionModel(const MotionModel& motionModel) { m_motionModel = motionModel; }
  static int getEGCOffset(const MotionModel& motionModel);
  int        getEGCOffset() const { return getEGCOffset(m_motionModel); }

  int getNumBinsOfSignsAndSuffixes() const
  {
    return ((!isHorMagnZero && !horEncodeSignInEP) ? 1 : 0) + // hor sign bin
           ((!isVerMagnZero && !verEncodeSignInEP) ? 1 : 0) + // ver sign bin
           std::max(0,horOffsetPredictionNumBins) + std::max(0, verOffsetPredictionNumBins);
  }

  void initPrefixesMvd(const Mv& mv, const int imv, const bool isInternalPrecision);
  void initSuffixesAndSignsMvd(const Mv &mv, const int imv);
  void initPrefixesAffineMvd(const Mv& affineMv, const int imv, const bool isInternalPrecision, int& binBudget);
  void initSuffixesAndSignsAffineMvd(const Mv& affineMv, const int imv, int& binBudget);
  void defineNumberOfPredictedBinsInSuffixAffineMvd(const int iHorPrefix, const int iVerPrefix, const uint8_t imv, int& binBudget);
#endif

  static int getMaxSuffix(const int prefix, const int maxAbs, const int maxGroupStartValue)
  {
    if (maxAbs <= 0)
    {
      return -1;
    }
    const int maxPrefix = getPrefixLength(maxAbs - 1);
    return maxPrefix != prefix ? -1 : 1 + floorLog2(getSuffix(maxAbs - 1, maxGroupStartValue));
  }

  static unsigned int getSuffix(const int absVal, const int groupStartValue)
  {
    CHECK(absVal < 0, "Incorrect absVal in getSuffix()");
    int suffix = absVal - groupStartValue;
    return suffix;
  }

  static unsigned int getPrefixLength(int iSymbol)
  {
    if (iSymbol < 0)
    {
      return -1;
    }
    int param = BVD_CODING_GOLOMB_ORDER;
    while (iSymbol >= (unsigned)(1 << param))
    {
      iSymbol -= 1 << param;
      param++;
    }
    return param - 1; // "-1" is to compensate for separator
  }


  static unsigned     xGetGolombGroupMinValue(const int prefix)
  {
    CHECK(prefix < 0, "final_param < 0");
    const int golomb_param = BVD_CODING_GOLOMB_ORDER;
    return (1 << (prefix + golomb_param)) - 1 - ((1 << golomb_param) - 1);// (final_param > golomb_param + 1 ? ((1 << golomb_param) - 1) : 0);
  }
};
#endif

#if JVET_AD0140_MVD_PREDICTION
struct MvdSuffixInfoMv
{
  static constexpr int suffixPosThreshold = 0; // least significant postition of predicted bin in a suffix 
  static constexpr int maxNumMv           = 3;
  static constexpr int maxNumMvComp       = 2 * maxNumMv; //MV has 2 components: horizontal and vertical ones

  MotionModel   m_motionModel = MotionModel::Undefined;

  MvdSuffixInfo mvBins[NUM_REF_PIC_LIST_01][maxNumMv];
  struct AuxMvdBins
  {
    int  mvIdx      = -1;
    bool isHor      = true;
    int  numHypBins = -1; // Number of hypotheses bins (siffix + sign bin)
    int  rplIdx     = static_cast<int>(REF_PIC_LIST_X);
    AuxMvdBins() {};
    AuxMvdBins(int _mvIdxIn, bool _isHorIn, int _numHypBins, int _rplIdx) : mvIdx(_mvIdxIn), isHor(_isHorIn), numHypBins(_numHypBins), rplIdx(_rplIdx) {};
  };

  AuxMvdBins auxMvdBins[NUM_REF_PIC_LIST_01][maxNumMvComp]; //one entry for one MV (MVD) component
  int actualMvCompNum[NUM_REF_PIC_LIST_01] = { -1, -1 };
  RefPicList actualRpl                     = REF_PIC_LIST_X;

  static unsigned getBinBudgetForPrediction(const unsigned int width, const unsigned int height, const uint8_t imv)
  {
    return (width > 4 && height > 4) ? 6 : 2;
  }

  void clear();

  void       setMotionModel         ( const MotionModel& motionModel ) { m_motionModel = motionModel;}
  bool       getMergedBinBudgetForMv( const unsigned int curBinBudget );
  bool       getBinBudgetForMv      ( const unsigned int curBinBudget, const RefPicList rplIdx );
  RefPicList selectRplForMvdCoding  ( const unsigned int curBinBudget ) const;
  void       initPrefixesMvd        ( const int mvIdx, const RefPicList curRpl, const Mv& mvd, const int imv, const bool isInternalPrecision, const MotionModel& motionModel );
  void       initSuffixesAndSignsMvd( const int mvIdx, const RefPicList rpl,    const Mv& mvd, const int imv, const MotionModel& motionModel );
};
#endif

namespace std
{
  template<> struct hash<Mv>
  {
    size_t operator()( const Mv &value ) const { return ( ( (size_t)value.hor << 32 ) + value.ver ); }
  };
};

extern void(*clipMv) ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );
void clipMvInPic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );
void clipMvInSubpic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
extern void(*clipMv2) (Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift);
void clipMvInPic2(Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift);
void clipMvInSubpic2(Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps, const int mvShift);
#endif
bool wrapClipMv( Mv& rcMv, const Position& pos,
                 const struct Size& size,
                 const SPS *sps
               , const PPS* pps
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
               , const int mvShift = MV_FRACTIONAL_BITS_INTERNAL
#endif
);

void roundAffineMv( int& mvx, int& mvy, int nShift );

//! \}

#endif // __MV__
