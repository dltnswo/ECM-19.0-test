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

/** \file     RdCost.h
    \brief    RD cost computation classes (header)
*/

#ifndef __RDCOST__
#define __RDCOST__

#include "CommonDef.h"
#include "Mv.h"
#include "Unit.h"
#include "Buffer.h"
#include "Slice.h"
#include "RdCostWeightPrediction.h"
#include <math.h>

//! \ingroup CommonLib
//! \{

#if JVET_Z0131_IBC_BVD_BINARIZATION
struct EstBvdBitsStruct
{
  uint32_t bitsGt0FlagH[2];
  uint32_t bitsGt0FlagV[2];

  uint32_t bitsH[BVD_IBC_MAX_PREFIX];
  uint32_t bitsV[BVD_IBC_MAX_PREFIX];

  uint32_t bitsIdx[2];
  uint32_t bitsImv[2];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  uint32_t bitsFracImv[4];
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  uint32_t bitsRribc;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  uint32_t bitsMerge[IBC_MRG_MAX_NUM_CANDS];
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  uint32_t bitsBvType[3];
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  uint32_t bitsUseFlip[2];
#endif
#endif
};
#endif

class DistParam;
class EncCfg;

// ====================================================================================================================
// Type definition
// ====================================================================================================================

// for function pointer
typedef Distortion (*FpDistFunc) (const DistParam&);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// distortion parameter class
class DistParam
{
public:
  CPelBuf               org;
  CPelBuf               cur;
#if WCG_EXT
  CPelBuf               orgLuma;
#endif
  const Pel*            mask;
  int                   maskStride;
  int                   stepX;
  int                   maskStride2;
  int                   step;
  FpDistFunc            distFunc;
  int                   bitDepth;

  bool                  useMR;
  bool                  applyWeight;     // whether weighted prediction is used or not
  bool                  isBiPred;

  const WPScalingParam *wpCur;           // weighted prediction scaling parameters for current ref
  ComponentID           compID;
  Distortion            maximumDistortionForEarlyExit; /// During cost calculations, if distortion exceeds this value, cost calculations may early-terminate.

  // (vertical) subsampling shift (for reducing complexity)
  // - 0 = no subsampling, 1 = even rows, 2 = every 4th, etc.
  int                   subShift;
  int                   cShiftX;
  int                   cShiftY;
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  int                   tmWeightIdx;
#endif
  DistParam() :
  org(), cur(),
  mask( nullptr ),
  maskStride( 0 ),
  stepX(0),
  maskStride2(0),
  step( 1 ), bitDepth( 0 ), useMR( false ), applyWeight( false ), isBiPred( false ), wpCur( nullptr ), compID( MAX_NUM_COMPONENT ), maximumDistortionForEarlyExit( std::numeric_limits<Distortion>::max() ), subShift( 0 )
  , cShiftX(-1), cShiftY(-1)
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  , tmWeightIdx(-1)
#endif
  { }
};

/// RD cost computation class
class RdCost
{
private:
  // for distortion

  static FpDistFunc       m_afpDistortFunc[DF_TOTAL_FUNCTIONS]; // [eDFunc]
  CostMode                m_costMode;
  double                  m_distortionWeight[MAX_NUM_COMPONENT]; // only chroma values are used.
  double                  m_dLambda;
  bool                   m_isLosslessRDCost;

#if WCG_EXT
  double                  m_dLambda_unadjusted; // TODO: check is necessary
  double                  m_DistScaleUnadjusted;
  static std::vector<double> m_reshapeLumaLevelToWeightPLUT;
  static std::vector<double> m_lumaLevelToWeightPLUT;
  static uint32_t         m_signalType;
  static double           m_chromaWeight;
  static int              m_lumaBD;
  ChromaFormat            m_cf;
#endif
  double                  m_DistScale;
  double                  m_dLambdaMotionSAD;
  double                  m_lambdaStore[2][3];   // 0-org; 1-act
  double                  m_DistScaleStore[2][3]; // 0-org; 1-act
  bool                    m_resetStore;
  int                     m_pairCheck;

  // for motion cost
  Mv                      m_mvPredictor;
#if JVET_AA0070_RRIBC
  Mv                      m_bvPredictors[3][2];
#else
  Mv                      m_bvPredictors[2];
#endif
  double                  m_motionLambda;
  int                     m_iCostScale;
  double                  m_dCost; // for ibc
#if JVET_Z0131_IBC_BVD_BINARIZATION
  EstBvdBitsStruct        m_cBvdBitCosts;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool                    m_useFullPelImvForZeroBvd;
#endif

public:
  RdCost();
  virtual ~RdCost();

#if WCG_EXT
  void          setChromaFormat       ( const ChromaFormat & _cf) { m_cf = _cf; }
  double        calcRdCost            ( uint64_t fracBits, Distortion distortion, bool useUnadjustedLambda = true );
#else
  double        calcRdCost            ( uint64_t fracBits, Distortion distortion );
#endif

  void          setDistortionWeight   ( const ComponentID compID, const double distortionWeight ) { m_distortionWeight[compID] = distortionWeight; }
  void          setLambda             ( double dLambda, const BitDepths &bitDepths );

#if WCG_EXT
  double        getLambda( bool unadj = false )
                                      { return unadj ? m_dLambda_unadjusted : m_dLambda; }
#else
  double        getLambda()           { return m_dLambda; }
#endif
  double        getChromaWeight()     { return ((m_distortionWeight[COMPONENT_Cb] + m_distortionWeight[COMPONENT_Cr]) / 2.0); }
#if RDOQ_CHROMA_LAMBDA
  double        getDistortionWeight   ( const ComponentID compID ) const { return m_distortionWeight[compID % MAX_NUM_COMPONENT]; }
#endif

  void          setCostMode(CostMode m) { m_costMode = m; }
  void          setLosslessRDCost(bool m) { m_isLosslessRDCost = m; }

  // Distortion Functions
  void          init();
#ifdef TARGET_SIMD_X86
  void          initRdCostX86();
  template <X86_VEXT vext>
  void          _initRdCostX86();
#endif

  void           setDistParam( DistParam &rcDP, const CPelBuf &org, const Pel* piRefY , int iRefStride, int bitDepth, ComponentID compID, int subShiftMode = 0, int step = 1, bool useHadamard = false );
  void           setDistParam( DistParam &rcDP, const CPelBuf &org, const CPelBuf &cur, int bitDepth, ComponentID compID, bool useHadamard = false );
  void           setDistParam( DistParam &rcDP, const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShiftMode = 0, int step = 1, bool useHadamard = false, bool bioApplied = false );
#if JVET_W0123_TIMD_FUSION
  void           setTimdDistParam( DistParam &rcDP, const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShiftMode = 0, int step = 1, bool useHadamard = false );
#endif
  void           setDistParam( DistParam &rcDP, const CPelBuf &org, const Pel* piRefY, int iRefStride, const Pel* mask01, int iMaskStride, int stepX, int iMaskStride2, int bitDepth, ComponentID compID);
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  void           setDistParam( DistParam &rcDP, const CPelBuf &org, const CPelBuf &cur, int bitDepth, bool trueAfalseL, int wIdx, int subShift, ComponentID compID);
#endif

  double         getMotionLambda          ( )  { return m_dLambdaMotionSAD; }
  void           selectMotionLambda       ( )  { m_motionLambda = getMotionLambda( ); }
  void           setPredictor             ( const Mv& rcMv )
  {
    m_mvPredictor = rcMv;
  }
  void           setCostScale             ( int iCostScale )           { m_iCostScale = iCostScale; }
  Distortion     getCost                  ( uint32_t b )                   { return Distortion( m_motionLambda * b ); }
  // for ibc
  void           getMotionCost(int add) { m_dCost = m_dLambdaMotionSAD + add; }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  inline Distortion getBvCostSingle(Mv mv, AMVPInfo& amvpInfo, uint8_t imv, uint8_t imvForZeroBvd, bool zeroMvdCheckX, bool zeroMvdCheckY, uint32_t addExtraBits, uint8_t& mvpIdx)
  {
    Mv bv = mv;
    bv.changeIbcPrecInternal2Amvr(imv);

    auto getMvBinCost = [&](int mvpIdx)
    {
      Mv bvp = amvpInfo.mvCand[mvpIdx];
      bvp.changeIbcPrecInternal2Amvr(imv);
      Mv bvd = bv - bvp;

      if (imv != imvForZeroBvd && (zeroMvdCheckX || bvd.getHor() == 0) && (zeroMvdCheckY || bvd.getVer() == 0)) // note: zero mvd is allowed only for default IMV mode
      {
        return std::numeric_limits<uint32_t>::max();
      }
      else
      {
        uint32_t binCost = (zeroMvdCheckX ? 0 : xGetExpGolombNumberOfBitsIBCH(bvd.getHor()))
                         + (zeroMvdCheckY ? 0 : xGetExpGolombNumberOfBitsIBCV(bvd.getVer()))
                         + m_cBvdBitCosts.bitsIdx[mvpIdx];
        if (!(imv == imvForZeroBvd && (zeroMvdCheckX || bvd.getHor() == 0) && (zeroMvdCheckY || bvd.getVer() == 0)))
        {
          binCost += m_cBvdBitCosts.bitsFracImv[imv];
        }
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        if (addExtraBits >= 1 && addExtraBits <= 5)
        {
          const static int bvTypeTbl[] = { NOT_VALID,         0, 1, 2, 1, 2 }; // 0: 2D, 1: 1-D Hor, 2: 1-D Ver
          const static int bvFlipTbl[] = { NOT_VALID, NOT_VALID, 0, 0, 1, 1 }; // 0: no flip, 1: flip

          binCost += m_cBvdBitCosts.bitsBvType[bvTypeTbl[addExtraBits]];
          if (addExtraBits > 1)
          {
            binCost += m_cBvdBitCosts.bitsUseFlip[bvFlipTbl[addExtraBits]];
          }
        }
#else
        if (addExtraBits >= 1 && addExtraBits <= 3)
        {
          binCost += m_cBvdBitCosts.bitsBvType[addExtraBits - 1];
        }
#endif
#endif
        return binCost;
      }
    };

    uint32_t b0       = getMvBinCost(0);
    uint32_t b1       = getMvBinCost(1);
    uint32_t bBest    = (b1 < b0) ? b1 : b0;
    int      bBestIdx = (b1 < b0) ? 1 : 0;

    mvpIdx = bBestIdx;
    return Distortion(m_dCost * bBest) >> SCALE_BITS;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
  inline Distortion getBvCost(Mv mvd, uint8_t imv, uint8_t imvForZeroBvd, bool zeroMvdCheckX, bool zeroMvdCheckY,
                              uint32_t addExtraBits, uint8_t mvpIdx)
  {
    Mv bvd = mvd;
    bvd.changeIbcPrecInternal2Amvr(imv);

    uint32_t binCost = (zeroMvdCheckX ? 0 : xGetExpGolombNumberOfBitsIBCH(bvd.getHor()))
                       + (zeroMvdCheckY ? 0 : xGetExpGolombNumberOfBitsIBCV(bvd.getVer()))
                       + m_cBvdBitCosts.bitsIdx[mvpIdx];
    if (!(imv == imvForZeroBvd && (zeroMvdCheckX || bvd.getHor() == 0) && (zeroMvdCheckY || bvd.getVer() == 0)))
    {
      binCost += m_cBvdBitCosts.bitsFracImv[imv];
    }
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    if (addExtraBits >= 1 && addExtraBits <= 5)
    {
      const int bvTypeTbl[] = { NOT_VALID, 0, 1, 2, 1, 2 };           // 0: 2D, 1: 1-D Hor, 2: 1-D Ver
      const int bvFlipTbl[] = { NOT_VALID, NOT_VALID, 0, 0, 1, 1 };   // 0: no flip, 1: flip

      binCost += m_cBvdBitCosts.bitsBvType[bvTypeTbl[addExtraBits]];
      if (addExtraBits > 1)
      {
        binCost += m_cBvdBitCosts.bitsUseFlip[bvFlipTbl[addExtraBits]];
      }
    }
#else
    if (addExtraBits >= 1 && addExtraBits <= 3)
    {
      binCost += m_cBvdBitCosts.bitsBvType[addExtraBits - 1];
    }
#endif
#endif
    return Distortion(m_dCost * binCost) >> SCALE_BITS;
  }
#endif

  void setFullPelImvForZeroBvd(bool b) { m_useFullPelImvForZeroBvd = b; }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  inline Mv getBvPred()
  {
#if JVET_AA0070_RRIBC
    return Mv(m_bvPredictors[0][0].getHor(), m_bvPredictors[0][0].getVer());
#else
    return Mv(m_bvPredictors[0].getHor(), m_bvPredictors[0].getVer());
#endif
  }
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  inline Distortion getBvVerZeroCompCost(int x, bool useIMV, int *tempImv, int *tempIdx, Mv bvp1Pel[2])
  {
    uint32_t b0 = xGetExpGolombNumberOfBitsIBCH(x - bvp1Pel[0].getHor()) + m_cBvdBitCosts.bitsIdx[0];
    uint32_t b1 = xGetExpGolombNumberOfBitsIBCH(x - bvp1Pel[1].getHor()) + m_cBvdBitCosts.bitsIdx[1];
    if (useIMV)
    {
      b0 += (x != bvp1Pel[0].getHor()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      b1 += (x != bvp1Pel[1].getHor()) ? m_cBvdBitCosts.bitsImv[0] : 0;
    }
    uint32_t bestCost = (b1 < b0) ? b1 : b0;
    int      bestIdx  = (b1 < b0) ? 1 : 0;
    int      imv      = (useIMV && (x != bvp1Pel[bestIdx].getHor())) ? 1 : 0;
    if (imv && (x % 4 == 0))
    {
      int      x4Pel = x >> 2;
      uint32_t bQ0, bQ1;
      bQ0 = (x4Pel == (bvp1Pel[0].getHor() >> 2))
              ? std::numeric_limits<uint32_t>::max()
              : (xGetExpGolombNumberOfBitsIBCH(x4Pel - (bvp1Pel[0].getHor() >> 2)) + m_cBvdBitCosts.bitsIdx[0]);
      bQ1 = (x4Pel == (bvp1Pel[1].getHor() >> 2))
              ? std::numeric_limits<uint32_t>::max()
              : (xGetExpGolombNumberOfBitsIBCH(x4Pel - (bvp1Pel[1].getHor() >> 2)) + m_cBvdBitCosts.bitsIdx[1]);

      uint32_t bestCost4Pel = (bQ1 < bQ0) ? bQ1 : bQ0;
      bestCost4Pel += (bestCost4Pel < std::numeric_limits<uint32_t>::max()) ? m_cBvdBitCosts.bitsImv[1] : 0;
      if (bestCost4Pel < bestCost)
      {
        imv      = 2;
        bestCost = bestCost4Pel;
        bestIdx  = (bQ1 < bQ0) ? 1 : 0;
      }
    }
    *tempImv = imv;
    *tempIdx = bestIdx;
    return Distortion(m_dCost * bestCost) >> SCALE_BITS;
  }

  inline Distortion getBvHorZeroCompCost(int y, bool useIMV, int *tempImv, int *tempIdx, Mv bvp1Pel[2])
  {
    uint32_t b0 = xGetExpGolombNumberOfBitsIBCV(y - bvp1Pel[0].getVer()) + m_cBvdBitCosts.bitsIdx[0];
    uint32_t b1 = xGetExpGolombNumberOfBitsIBCV(y - bvp1Pel[1].getVer()) + m_cBvdBitCosts.bitsIdx[1];
    if (useIMV)
    {
      b0 += (y != bvp1Pel[0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      b1 += (y != bvp1Pel[1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
    }
    uint32_t bestCost = (b1 < b0) ? b1 : b0;
    int      bestIdx  = (b1 < b0) ? 1 : 0;
    int      imv      = (useIMV && (y != bvp1Pel[bestIdx].getVer())) ? 1 : 0;
    if (imv && (y % 4 == 0))
    {
      int      y4Pel = y >> 2;
      uint32_t bQ0, bQ1;
      bQ0 = (y4Pel == (bvp1Pel[0].getVer() >> 2))
              ? std::numeric_limits<uint32_t>::max()
              : (xGetExpGolombNumberOfBitsIBCV(y4Pel - (bvp1Pel[0].getVer() >> 2)) + m_cBvdBitCosts.bitsIdx[0]);
      bQ1 = (y4Pel == (bvp1Pel[1].getVer() >> 2))
              ? std::numeric_limits<uint32_t>::max()
              : (xGetExpGolombNumberOfBitsIBCV(y4Pel - (bvp1Pel[1].getVer() >> 2)) + m_cBvdBitCosts.bitsIdx[1]);

      uint32_t bestCost4Pel = (bQ1 < bQ0) ? bQ1 : bQ0;
      bestCost4Pel += (bestCost4Pel < std::numeric_limits<uint32_t>::max()) ? m_cBvdBitCosts.bitsImv[1] : 0;

      if (bestCost4Pel < bestCost)
      {
        imv      = 2;
        bestCost = bestCost4Pel;
        bestIdx  = (bQ1 < bQ0) ? 1 : 0;
      }
    }
    *tempImv = imv;
    *tempIdx = bestIdx;
    return Distortion(m_dCost * bestCost) >> SCALE_BITS;
  }
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
  inline Distortion getBvpMergeCost(int idx)
  {
    return Distortion(m_dCost * m_cBvdBitCosts.bitsMerge[idx]) >> SCALE_BITS;
  }
#endif

#if JVET_AA0070_RRIBC
  void setPredictors(Mv pcMv[3][2]);
#if JVET_Z0131_IBC_BVD_BINARIZATION || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  EstBvdBitsStruct *getBvdBitCosts() { return &m_cBvdBitCosts; }

#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, int rribcFlipType, uint8_t *bvImvResBest = NULL,
                                           int *bvpIdxBest = NULL, bool flag = false, AMVPInfo *amvpInfo4Pel = NULL,
                                           const bool useBvpCluster = true)
#else
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, int rribcFlipType, uint8_t *bvImvResBest = NULL, int *bvpIdxBest = NULL)
#endif
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && !JVET_AE0169_BIPREDICTIVE_IBC
    const uint32_t fullPelImvCostForZeroBvd = m_useFullPelImvForZeroBvd || rribcFlipType > 0 ? 0 : std::numeric_limits<uint32_t>::max();

    uint32_t b0 = 0;
    uint32_t b1 = 0;
    if (useIMV)
    {
      if (rribcFlipType == 0)
      {
        b0 = (x != m_bvPredictors[rribcFlipType][0].getHor() || y != m_bvPredictors[rribcFlipType][0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
        b1 = (x != m_bvPredictors[rribcFlipType][1].getHor() || y != m_bvPredictors[rribcFlipType][1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
      }
      else if (rribcFlipType == 1)
      {
        b0 = (x != m_bvPredictors[rribcFlipType][0].getHor()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
        b1 = (x != m_bvPredictors[rribcFlipType][1].getHor()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
      }
      else
      {
        b0 = (y != m_bvPredictors[rribcFlipType][0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
        b1 = (y != m_bvPredictors[rribcFlipType][1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : fullPelImvCostForZeroBvd;
      }
    }

    if (b0 != std::numeric_limits<uint32_t>::max())
    {
      b0 += (rribcFlipType == 1) ? 0 : xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[rribcFlipType][0].getVer());
      b0 += (rribcFlipType == 2) ? 0 : xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[rribcFlipType][0].getHor());
      b0 += m_cBvdBitCosts.bitsIdx[0];
    }

    if (b1 != std::numeric_limits<uint32_t>::max())
    {
      b1 += (rribcFlipType == 1) ? 0 : xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[rribcFlipType][1].getVer());
      b1 += (rribcFlipType == 2) ? 0 : xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[rribcFlipType][1].getHor());
      b1 += m_cBvdBitCosts.bitsIdx[1];
    }
#else
    uint32_t b0 = (rribcFlipType == 1) ? 0 : xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[rribcFlipType][0].getVer());
    b0 += (rribcFlipType == 2) ? 0 : xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[rribcFlipType][0].getHor());
    b0 += m_cBvdBitCosts.bitsIdx[0];

    uint32_t b1 = (rribcFlipType == 1) ? 0 : xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[rribcFlipType][1].getVer());
    b1 += (rribcFlipType == 2) ? 0 : xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[rribcFlipType][1].getHor());
    b1 += m_cBvdBitCosts.bitsIdx[1];

    if (useIMV)
    {
      if (rribcFlipType == 0)
      {
        b0 += (x != m_bvPredictors[rribcFlipType][0].getHor() || y != m_bvPredictors[rribcFlipType][0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
        b1 += (x != m_bvPredictors[rribcFlipType][1].getHor() || y != m_bvPredictors[rribcFlipType][1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      }
      else if (rribcFlipType == 1)
      {
        b0 += (x != m_bvPredictors[rribcFlipType][0].getHor()) ? m_cBvdBitCosts.bitsImv[0] : 0;
        b1 += (x != m_bvPredictors[rribcFlipType][1].getHor()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      }
      else
      {
        b0 += (y != m_bvPredictors[rribcFlipType][0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
        b1 += (y != m_bvPredictors[rribcFlipType][1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      }
    }
#endif
    uint32_t bBest    = (b1 < b0) ? b1 : b0;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    if (useBvpCluster)
    {
      bBest += (rribcFlipType) ? m_cBvdBitCosts.bitsRribc : 0;
    }
#endif
    int      bBestIdx = (b1 < b0) ? 1 : 0;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    uint8_t  bestRes  = IMV_FPEL;
    if(m_useFullPelImvForZeroBvd)
    {
#else
    uint8_t  bestRes;
#endif
    if (rribcFlipType == 0)
    {
      bestRes = (useIMV && (x != m_bvPredictors[rribcFlipType][bBestIdx].getHor() || y != m_bvPredictors[rribcFlipType][bBestIdx].getVer())) ? 1 : 0;
    }
    else if (rribcFlipType == 1)
    {
      bestRes = (useIMV && (x != m_bvPredictors[rribcFlipType][bBestIdx].getHor())) ? 1 : 0;
    }
    else
    {
      bestRes = (useIMV && (y != m_bvPredictors[rribcFlipType][bBestIdx].getVer())) ? 1 : 0;
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif

    if (bvImvResBest)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      *bvImvResBest = IMV_FPEL;
#else
      *bvImvResBest = bestRes;
#endif
      *bvpIdxBest   = bBestIdx;
    }

#if JVET_AE0169_BIPREDICTIVE_IBC
    if (useIMV && bestRes && x % 4 == 0 && y % 4 == 0)
#else
    if (bestRes && x % 4 == 0 && y % 4 == 0)
#endif
    {
      Mv cMv(x >> 2, y >> 2);

#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV 
      Mv tmpBv0;
      Mv tmpBv1;
      if (flag)
      {
        tmpBv0 = amvpInfo4Pel->mvCand[0];
        tmpBv1 = amvpInfo4Pel->mvCand[1];
        tmpBv0.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);
        tmpBv1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);
      }
      else
      {
        tmpBv0 = m_bvPredictors[rribcFlipType][0];
        tmpBv1 = m_bvPredictors[rribcFlipType][1];
        tmpBv0.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
        tmpBv1.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
      }
#else
      Mv tmpBv0 = m_bvPredictors[rribcFlipType][0];
      Mv tmpBv1 = m_bvPredictors[rribcFlipType][1];
      tmpBv0.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
      tmpBv1.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
#endif

      uint32_t bQ0, bQ1;
      if (rribcFlipType == 0)
      {
        bQ0 = (cMv == tmpBv0) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv0.getHor()) + xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv0.getVer()) + m_cBvdBitCosts.bitsIdx[0]);
        bQ1 = (cMv == tmpBv1) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv1.getHor()) + xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv1.getVer()) + m_cBvdBitCosts.bitsIdx[1]);
      }
      else if (rribcFlipType == 1)
      {
        bQ0 = (cMv.getHor() == tmpBv0.getHor()) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv0.getHor()) + m_cBvdBitCosts.bitsIdx[0]);
        bQ1 = (cMv.getHor() == tmpBv1.getHor()) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv1.getHor()) + m_cBvdBitCosts.bitsIdx[1]);
      }
      else
      {
        bQ0 = (cMv.getVer() == tmpBv0.getVer()) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv0.getVer()) + m_cBvdBitCosts.bitsIdx[0]);
        bQ1 = (cMv.getVer() == tmpBv1.getVer()) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv1.getVer()) + m_cBvdBitCosts.bitsIdx[1]);
      }
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      if (useBvpCluster)
      {
        bQ0 += (rribcFlipType && (cMv.getHor() != tmpBv0.getHor()) && (cMv.getVer() != tmpBv0.getVer()))
                 ? m_cBvdBitCosts.bitsRribc
                 : 0;
        bQ1 += (rribcFlipType && (cMv.getHor() != tmpBv1.getHor()) && (cMv.getVer() != tmpBv1.getVer()))
                 ? m_cBvdBitCosts.bitsRribc
                 : 0;
      }
#endif
      uint32_t bQBest = (bQ1 < bQ0) ? bQ1 : bQ0;
      bQBest += (bQBest < std::numeric_limits<uint32_t>::max()) ? m_cBvdBitCosts.bitsImv[1] : 0;

      if (bQBest < bBest)
      {
        if (bvImvResBest)
        {
          *bvImvResBest = 2;
          *bvpIdxBest   = (bQ1 < bQ0) ? 1 : 0;
        }
        bBest = bQBest;
      }
    }
    return Distortion(m_dCost * bBest) >> SCALE_BITS;
  }
#else
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, int rribcFlipType)
  {
    return Distortion(m_dCost * getBitsMultiplePreds(x, y, useIMV, rribcFlipType));
  }
  unsigned int getBitsMultiplePreds(int x, int y, bool useIMV, int rribcFlipType)
  {
    int rmvH[2];
    int rmvV[2];
    rmvH[0] = x - m_bvPredictors[rribcFlipType][0].getHor();
    rmvH[1] = x - m_bvPredictors[rribcFlipType][1].getHor();

    rmvV[0] = y - m_bvPredictors[rribcFlipType][0].getVer();
    rmvV[1] = y - m_bvPredictors[rribcFlipType][1].getVer();

    int absCand[2];
    absCand[0] = abs(rmvH[0]) + abs(rmvV[0]);
    absCand[1] = abs(rmvH[1]) + abs(rmvV[1]);

    int rmvHQP[2];
    int rmvVQP[2];
    if (x % 4 == 0 && y % 4 == 0 && useIMV)
    {
      int imvShift = 2;
      int offset   = 1 << (imvShift - 1);

      rmvHQP[0] = (x >> 2) - ((m_bvPredictors[rribcFlipType][0].getHor() + offset) >> 2);
      rmvHQP[1] = (x >> 2) - ((m_bvPredictors[rribcFlipType][1].getHor() + offset) >> 2);
      rmvVQP[0] = (y >> 2) - ((m_bvPredictors[rribcFlipType][0].getVer() + offset) >> 2);
      rmvVQP[1] = (y >> 2) - ((m_bvPredictors[rribcFlipType][1].getVer() + offset) >> 2);

      int absCandQP[2];
      absCandQP[0] = abs(rmvHQP[0]) + abs(rmvVQP[0]);
      absCandQP[1] = abs(rmvHQP[1]) + abs(rmvVQP[1]);
      unsigned int candBits0QP, candBits1QP;
      if (absCand[0] < absCand[1])
      {
        unsigned int candBits0 = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvV[0]);
        candBits0 += (rribcFlipType == 2) ? 0 : getIComponentBits(rmvH[0]);
        if (absCandQP[0] < absCandQP[1])
        {
          candBits0QP = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvVQP[0]);
          candBits0QP += (rribcFlipType == 2) ? 0 : getIComponentBits(rmvHQP[0]);
          return candBits0QP < candBits0 ? candBits0QP : candBits0;
        }
        else
        {
          candBits1QP = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvVQP[1]);
          candBits1QP = (rribcFlipType == 2) ? 0 : getIComponentBits(rmvHQP[1]);
          return candBits1QP < candBits0 ? candBits1QP : candBits0;
        }
      }
      else
      {
        unsigned int candBits1 = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvV[1]);
        candBits1 += (rribcFlipType == 2) ? 0 : getIComponentBits(rmvH[1]);
        if (absCandQP[0] < absCandQP[1])
        {
          candBits0QP = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvVQP[0]);
          candBits0QP += (rribcFlipType == 2) ? 0 : getIComponentBits(rmvHQP[0]);
          return candBits0QP < candBits1 ? candBits0QP : candBits1;
        }
        else
        {
          candBits1QP = (rribcFlipType == 1) ? 0 : getIComponentBits(rmvVQP[1]);
          candBits1QP = (rribcFlipType == 2) ? 0 : getIComponentBits(rmvHQP[1]);
          return candBits1QP < candBits1 ? candBits1QP : candBits1;
        }
      }
    }
    else
    {
      if (absCand[0] < absCand[1])
      {
        return ((rribcFlipType == 1) ? (getIComponentBits(rmvH[0])) : ((rribcFlipType == 2) ? (getIComponentBits(rmvV[0])) : (getIComponentBits(rmvH[0]) + getIComponentBits(rmvV[0]))));
      }
      else
      {
        return ((rribcFlipType == 1) ? (getIComponentBits(rmvH[1])) : ((rribcFlipType == 2) ? (getIComponentBits(rmvV[1])) : (getIComponentBits(rmvH[1]) + getIComponentBits(rmvV[1]))));
      }
    }
  }

  unsigned int getIComponentBits(int val)
  {
    if (!val)
    {
      return 1;
    }

    unsigned int length = 1;
    unsigned int temp   = (val <= 0) ? (-val << 1) + 1 : (val << 1);

    while (1 != temp)
    {
      temp >>= 1;
      length += 2;
    }

    return length;
  }
#endif
#else
  void            setPredictors(Mv *pcMv);

#if JVET_Z0131_IBC_BVD_BINARIZATION
  EstBvdBitsStruct*  getBvdBitCosts()
  {
    return &m_cBvdBitCosts;
  }
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, uint8_t *bvImvResBest = NULL, int *bvpIdxBest = NULL, bool flag = false, AMVPInfo* amvpInfo4Pel = NULL)
#else
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, uint8_t *bvImvResBest = NULL, int *bvpIdxBest = NULL)
#endif
  {
    uint32_t b0 = xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[0].getHor()) + xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[0].getVer()) + m_cBvdBitCosts.bitsIdx[0];
    uint32_t b1 = xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[1].getHor()) + xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[1].getVer()) + m_cBvdBitCosts.bitsIdx[1];

    if (useIMV)
    {
      b0 += (x != m_bvPredictors[0].getHor() || y != m_bvPredictors[0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      b1 += (x != m_bvPredictors[1].getHor() || y != m_bvPredictors[1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
    }
    uint32_t bBest = (b1 < b0) ? b1 : b0;
    int bBestIdx = (b1 < b0) ? 1 : 0;
    uint8_t bestRes = (useIMV && (x != m_bvPredictors[bBestIdx].getHor() || y != m_bvPredictors[bBestIdx].getVer())) ? 1 : 0;
    if (bvImvResBest)
    {
      *bvImvResBest = bestRes;
      *bvpIdxBest = bBestIdx;
    }

    if (bestRes && x % 4 == 0 && y % 4 == 0)
    {
      Mv cMv(x >> 2, y >> 2);
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
      Mv tmpBv0;
      Mv tmpBv1;
      if (flag)
      {
        tmpBv0 = amvpInfo4Pel->mvCand[0];
        tmpBv1 = amvpInfo4Pel->mvCand[1];
        tmpBv0.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);
        tmpBv1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);
      }
      else
      {
        tmpBv0 = m_bvPredictors[0];
        tmpBv1 = m_bvPredictors[1];
        tmpBv0.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
        tmpBv1.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
      }
#else
      Mv tmpBv0 = m_bvPredictors[0];
      Mv tmpBv1 = m_bvPredictors[1];
      tmpBv0.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
      tmpBv1.changePrecision(MV_PRECISION_INT, MV_PRECISION_4PEL);
#endif

      uint32_t bQ0 = (cMv == tmpBv0) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv0.getHor()) + xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv0.getVer()) + m_cBvdBitCosts.bitsIdx[0] );
      uint32_t bQ1 = (cMv == tmpBv1) ? std::numeric_limits<uint32_t>::max() : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv1.getHor()) + xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv1.getVer()) + m_cBvdBitCosts.bitsIdx[1] );

      uint32_t bQBest = (bQ1 < bQ0) ? bQ1 : bQ0;
      bQBest += (bQBest < std::numeric_limits<uint32_t>::max()) ? m_cBvdBitCosts.bitsImv[1] : 0;

      if (bQBest < bBest)
      {
        if (bvImvResBest)
        {
          *bvImvResBest = 2;
          *bvpIdxBest = (bQ1 < bQ0) ? 1 : 0;
        }
        bBest = bQBest;
      }
    }
    return Distortion(m_dCost * bBest) >> SCALE_BITS;
  }
#else
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV)
  {
    return Distortion(m_dCost * getBitsMultiplePreds(x, y, useIMV));
  }

  unsigned int    getBitsMultiplePreds(int x, int y, bool useIMV)
  {
    int rmvH[2];
    int rmvV[2];
    rmvH[0] = x - m_bvPredictors[0].getHor();
    rmvH[1] = x - m_bvPredictors[1].getHor();

    rmvV[0] = y - m_bvPredictors[0].getVer();
    rmvV[1] = y - m_bvPredictors[1].getVer();
    int absCand[2];
    absCand[0] = abs(rmvH[0]) + abs(rmvV[0]);
    absCand[1] = abs(rmvH[1]) + abs(rmvV[1]);

    int rmvHQP[2];
    int rmvVQP[2];
    if (x % 4 == 0 && y % 4 == 0 && useIMV)
    {
      int imvShift = 2;
      int offset = 1 << (imvShift - 1);

      rmvHQP[0] = (x >> 2) - ((m_bvPredictors[0].getHor() + offset) >> 2);
      rmvHQP[1] = (x >> 2) - ((m_bvPredictors[1].getHor() + offset) >> 2);
      rmvVQP[0] = (y >> 2) - ((m_bvPredictors[0].getVer() + offset) >> 2);
      rmvVQP[1] = (y >> 2) - ((m_bvPredictors[1].getVer() + offset) >> 2);

      int absCandQP[2];
      absCandQP[0] = abs(rmvHQP[0]) + abs(rmvVQP[0]);
      absCandQP[1] = abs(rmvHQP[1]) + abs(rmvVQP[1]);
      unsigned int candBits0QP, candBits1QP;
      if (absCand[0] < absCand[1])
      {
        unsigned int candBits0 = getIComponentBits(rmvH[0]) + getIComponentBits(rmvV[0]);
        if (absCandQP[0] < absCandQP[1])
        {
          candBits0QP = getIComponentBits(rmvHQP[0]) + getIComponentBits(rmvVQP[0]);
          return candBits0QP <candBits0 ? candBits0QP : candBits0;
        }
        else
        {
          candBits1QP = getIComponentBits(rmvHQP[1]) + getIComponentBits(rmvVQP[1]);
          return candBits1QP < candBits0 ? candBits1QP : candBits0;
        }
      }
      else
      {
        unsigned int candBits1 = getIComponentBits(rmvH[1]) + getIComponentBits(rmvV[1]);
        if (absCandQP[0] < absCandQP[1])
        {
          candBits0QP = getIComponentBits(rmvHQP[0]) + getIComponentBits(rmvVQP[0]);
          return candBits0QP < candBits1 ? candBits0QP : candBits1;
        }
        else
        {
          candBits1QP = getIComponentBits(rmvHQP[1]) + getIComponentBits(rmvVQP[1]);
          return candBits1QP < candBits1 ? candBits1QP : candBits1;
        }
      }
    }
    else

    {
      if (absCand[0] < absCand[1])
      {
        return getIComponentBits(rmvH[0]) + getIComponentBits(rmvV[0]);
      }
      else
      {
        return getIComponentBits(rmvH[1]) + getIComponentBits(rmvV[1]);
      }
    }
  }

  unsigned int getIComponentBits(int val)
  {
    if (!val)
    {
      return 1;
    }

    unsigned int length = 1;
    unsigned int temp = (val <= 0) ? (-val << 1) + 1 : (val << 1);

    while (1 != temp)
    {
      temp >>= 1;
      length += 2;
    }

    return length;
  }
#endif
#endif

#if ENABLE_SPLIT_PARALLELISM
  void copyState( const RdCost& other );
#endif

  // for motion cost
  static uint32_t    xGetExpGolombNumberOfBits( int iVal )
  {
    CHECKD( iVal == std::numeric_limits<int>::min(), "Wrong value" );
    unsigned uiLength2 = 1, uiTemp2 = ( iVal <= 0 ) ? ( unsigned( -iVal ) << 1 ) + 1 : unsigned( iVal << 1 );

    while( uiTemp2 > MAX_CU_SIZE )
    {
      uiLength2 += ( MAX_CU_DEPTH << 1 );
      uiTemp2  >>=   MAX_CU_DEPTH;
    }

    return uiLength2 + ( floorLog2(uiTemp2) << 1 );
  }

  Distortion     getCostOfVectorWithPredictor( const int x, const int y, const unsigned imvShift )  { return Distortion( m_motionLambda * getBitsOfVectorWithPredictor(x, y, imvShift )); }
#if JVET_AA0070_RRIBC && !JVET_Z0131_IBC_BVD_BINARIZATION
  uint32_t getBitsOfVectorWithPredictor( const int x, const int y, const unsigned imvShift, const int &rribcFlipType = 0)
  {
    return ((rribcFlipType == 1) ? (xGetExpGolombNumberOfBits(((x << m_iCostScale) - m_mvPredictor.getHor()) >> imvShift)) : ((rribcFlipType == 2) ? xGetExpGolombNumberOfBits(((y << m_iCostScale) - m_mvPredictor.getVer()) >> imvShift)
                  : (xGetExpGolombNumberOfBits(((x << m_iCostScale) - m_mvPredictor.getHor()) >> imvShift) + xGetExpGolombNumberOfBits(((y << m_iCostScale) - m_mvPredictor.getVer()) >> imvShift))));
  }
#else
  uint32_t       getBitsOfVectorWithPredictor( const int x, const int y, const unsigned imvShift )  { return xGetExpGolombNumberOfBits(((x << m_iCostScale) - m_mvPredictor.getHor())>>imvShift) + xGetExpGolombNumberOfBits(((y << m_iCostScale) - m_mvPredictor.getVer())>>imvShift); }
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
  Distortion     getAmvpSbTmvpCostOfVectorWithPredictor(const int amvpSbTmvpMvdOffset, const int numOffset)
  {
    return Distortion(m_motionLambda * getAmvpSbTmvpBitsOfVectorWithPredictor(amvpSbTmvpMvdOffset, numOffset));
  }
  const int m_estAmvpSbTmvpIdxBits[3][3] = { { 3, 0, 0 },
                                             { 4, 4, 0 },
                                             { 4, 5, 5 } };
  uint32_t       getAmvpSbTmvpBitsOfVectorWithPredictor(const int amvpSbTmvpMvdOffset, const int numOffset)
  {
    if (amvpSbTmvpMvdOffset < 0)
    {
      return 1;
    }
    else
    {
      return m_estAmvpSbTmvpIdxBits[numOffset - 1][amvpSbTmvpMvdOffset];
    }
  }
#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
  // for block vector cost
  uint32_t    xGetExpGolombNumberOfBitsIBCH( int iVal )
  {
    CHECKD( iVal == std::numeric_limits<int>::min(), "Wrong value" );
    
    unsigned int temp = (iVal <= 0) ? -iVal : iVal;
    if (!temp) 
    {
      return m_cBvdBitCosts.bitsGt0FlagH[0];
    }
    else
    {
      unsigned int order = BVD_CODING_GOLOMB_ORDER;
      unsigned int bins = 0;
      temp -= 1;

      while (temp >= (1<<order))
      {
        temp -= (1<<order);
        order += 1;
        bins += 1;
      }
      CHECKD( bins >= BVD_IBC_MAX_PREFIX, "Prefix is too large" );
      return m_cBvdBitCosts.bitsGt0FlagH[1]
             + m_cBvdBitCosts.bitsH[bins] + (1<<SCALE_BITS);
    }
  }

  uint32_t    xGetExpGolombNumberOfBitsIBCV( int iVal )
  {
    CHECKD( iVal == std::numeric_limits<int>::min(), "Wrong value" );
    
    unsigned int temp = (iVal <= 0) ? -iVal : iVal;
    if (!temp) 
    {
      return m_cBvdBitCosts.bitsGt0FlagV[0];
    }
    else
    {
      unsigned int order = BVD_CODING_GOLOMB_ORDER;
      unsigned int bins = 0;
      temp -= 1;

      while (temp >= (1<<order))
      {
        temp -= (1<<order);
        order += 1;
        bins += 1;
      }
      CHECKD( bins >= BVD_IBC_MAX_PREFIX, "Prefix is too large" );
      return m_cBvdBitCosts.bitsGt0FlagV[1]
             + m_cBvdBitCosts.bitsV[bins] + (1<<SCALE_BITS);
    }
  }

#if !JVET_AA0070_RRIBC
  Distortion     getCostOfVectorWithPredictorIBC( const int x, const int y, const unsigned imvShift )  
  { 
    return Distortion( m_motionLambda * getBitsOfVectorWithPredictorIBC(x, y, imvShift )) >> SCALE_BITS; 
  }
  uint32_t           getBitsOfVectorWithPredictorIBC( const int x, const int y, const unsigned imvShift )  
  { 
    return xGetExpGolombNumberOfBitsIBCH(((x << m_iCostScale) - m_mvPredictor.getHor())>>imvShift) + xGetExpGolombNumberOfBitsIBCV(((y << m_iCostScale) - m_mvPredictor.getVer())>>imvShift); 
  }
#endif
#endif
#if WCG_EXT
         void    saveUnadjustedLambda       ();
         void    initLumaLevelToWeightTable ();
  inline double  getWPSNRLumaLevelWeight    (int val) { return m_lumaLevelToWeightPLUT[val]; }
  void           initLumaLevelToWeightTableReshape();
  void           updateReshapeLumaLevelToWeightTableChromaMD (std::vector<Pel>& ILUT);
  void           restoreReshapeLumaLevelToWeightTable        ();
  inline double  getWPSNRReshapeLumaLevelWeight              (int val)                   { return m_reshapeLumaLevelToWeightPLUT[val]; }
  void           setReshapeInfo                              (uint32_t type, int lumaBD) { m_signalType = type; m_lumaBD = lumaBD; }
  void           updateReshapeLumaLevelToWeightTable         (SliceReshapeInfo &sliceReshape, Pel *wtTable, double cwt);
  inline std::vector<double>& getLumaLevelWeightTable        ()                   { return m_lumaLevelToWeightPLUT; }
#endif

#if JVET_S0234_ACT_CRS_FIX
  void           lambdaAdjustColorTrans(bool forward, ComponentID compID, bool applyChromaScale = false, int* resScaleInv = NULL);
#else
  void           lambdaAdjustColorTrans(bool forward, ComponentID compID);
#endif
  void           resetStore() { m_resetStore = true; }

private:

  static Distortion xGetSSE           ( const DistParam& pcDtParam );
  static Distortion xGetSSE4          ( const DistParam& pcDtParam );
  static Distortion xGetSSE8          ( const DistParam& pcDtParam );
  static Distortion xGetSSE16         ( const DistParam& pcDtParam );
  static Distortion xGetSSE32         ( const DistParam& pcDtParam );
  static Distortion xGetSSE64         ( const DistParam& pcDtParam );
  static Distortion xGetSSE16N        ( const DistParam& pcDtParam );

#if WCG_EXT
  static Distortion getWeightedMSE    (int compIdx, const Pel org, const Pel cur, const uint32_t uiShift, const Pel orgLuma);
  static Distortion xGetSSE_WTD       ( const DistParam& pcDtParam );
  static Distortion xGetSSE2_WTD      ( const DistParam& pcDtParam );
  static Distortion xGetSSE4_WTD      ( const DistParam& pcDtParam );
  static Distortion xGetSSE8_WTD      ( const DistParam& pcDtParam );
  static Distortion xGetSSE16_WTD     ( const DistParam& pcDtParam );
  static Distortion xGetSSE32_WTD     ( const DistParam& pcDtParam );
  static Distortion xGetSSE64_WTD     ( const DistParam& pcDtParam );
  static Distortion xGetSSE16N_WTD    ( const DistParam& pcDtParam );
#endif

  static Distortion xGetSAD           ( const DistParam& pcDtParam );
  static Distortion xGetSAD4          ( const DistParam& pcDtParam );
  static Distortion xGetSAD8          ( const DistParam& pcDtParam );
  static Distortion xGetSAD16         ( const DistParam& pcDtParam );
  static Distortion xGetSAD32         ( const DistParam& pcDtParam );
  static Distortion xGetSAD64         ( const DistParam& pcDtParam );
  static Distortion xGetSAD16N        ( const DistParam& pcDtParam );

  static Distortion xGetSAD12         ( const DistParam& pcDtParam );
  static Distortion xGetSAD24         ( const DistParam& pcDtParam );
  static Distortion xGetSAD48         ( const DistParam& pcDtParam );

  static Distortion xGetSAD_full      ( const DistParam& pcDtParam );
  static Distortion xGetSADwMask      ( const DistParam& pcDtParam );

  static Distortion xGetMRSAD         ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD4        ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD8        ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD16       ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD32       ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD64       ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD16N      ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD12       ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD24       ( const DistParam& pcDtParam );
  static Distortion xGetMRSAD48       ( const DistParam& pcDtParam );
  static Distortion xGetMRHADs        ( const DistParam& pcDtParam );

  static Distortion xGetHADs          ( const DistParam& pcDtParam );
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  static Distortion xCalcHADs1xN      ( const Pel* piOrg, const Pel* piCurr, int iStrideOrg, int iStrideCur, int iRows, int iCols);
#endif
#if JVET_AJ0096_SATD_REORDER_INTRA || JVET_AJ0096_SATD_REORDER_INTER
  static Distortion xCalcHADs1x16     ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iRows, int iCols);
  static Distortion xCalcHADs1x8      ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur, int iRows, int iCols);
#endif
  static Distortion xCalcHADs2x2      ( const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );
  static Distortion xCalcHADs4x4      ( const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );
  static Distortion xCalcHADs8x8      ( const Pel *piOrg, const Pel *piCurr, int iStrideOrg, int iStrideCur, int iStep );

  static Distortion xCalcHADs16x8     ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur );
  static Distortion xCalcHADs8x16     ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur );
  static Distortion xCalcHADs4x8      ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur );
  static Distortion xCalcHADs8x4      ( const Pel *piOrg, const Pel *piCur, int iStrideOrg, int iStrideCur );

#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  template < int tplSize, bool trueAfalseL, bool mr >
  static Distortion xGetTMErrorFull   ( const DistParam& rcDtParam );
#endif

#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  static Distortion xGetSSE_SIMD    ( const DistParam& pcDtParam );
  template<int iWidth, X86_VEXT vext>
  static Distortion xGetSSE_NxN_SIMD( const DistParam& pcDtParam );
#if DIST_SSE_ENABLE && CTU_256
  template<X86_VEXT vext>
  static Distortion xGetSSE_NxN_SIMD( const DistParam &pcDtParam );
#endif

  template<X86_VEXT vext>
  static Distortion xGetSAD_SIMD    ( const DistParam& pcDtParam );
  template<int iWidth, X86_VEXT vext>
  static Distortion xGetSAD_NxN_SIMD( const DistParam& pcDtParam );
  template<X86_VEXT vext>
  static Distortion xGetSAD_IBD_SIMD( const DistParam& pcDtParam );

  template<X86_VEXT vext>
  static Distortion xGetHADs_SIMD   ( const DistParam& pcDtParam );

#if INTER_LIC || (TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM)
  template< X86_VEXT vext >
  static Distortion xGetMRSAD_SIMD(const DistParam &rcDtParam);
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  template < X86_VEXT vext, int tplSize, bool trueAfalseL, bool mr >
  static Distortion xGetTMErrorFull_SIMD(const DistParam& rcDtParam);
#endif

  template< X86_VEXT vext >
  static Distortion xGetSADwMask_SIMD( const DistParam& pcDtParam );
#endif

public:

#if WCG_EXT
  Distortion   getDistPart( const CPelBuf &org, const CPelBuf &cur, int bitDepth, const ComponentID compID, DFunc eDFunc, const CPelBuf *orgLuma = NULL );
#else
  Distortion   getDistPart( const CPelBuf &org, const CPelBuf &cur, int bitDepth, const ComponentID compID, DFunc eDFunc );
#endif

  Distortion   getDistPart(const CPelBuf &org, const CPelBuf &cur, const Pel* mask, int bitDepth, const ComponentID compID, DFunc eDFunc);
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  Distortion   getDistColChroma( const CPelBuf &org, const CPelBuf &cur, int bitDepth, const ComponentID compID, DFunc eDFunc );
#endif
};// END CLASS DEFINITION RdCost

//! \}

#endif // __RDCOST__
