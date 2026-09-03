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

/** \file     EncModeCtrl.h
    \brief    Encoder controller for trying out specific modes
*/

#ifndef __ENCMODECTRL__
#define __ENCMODECTRL__

// Include files
#include "EncCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"
#include "InterSearch.h"

#include <typeinfo>
#include <vector>

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////


enum EncTestModeType
{
  ETM_HASH_INTER,
  ETM_MERGE_SKIP,
  ETM_INTER_ME,
#if !MERGE_ENC_OPT
  ETM_AFFINE,
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
  ETM_AF_MMVD,
#endif
#if TM_MRG && !MERGE_ENC_OPT
  ETM_MERGE_TM,
#endif
  ETM_MERGE_GEO,
  ETM_INTRA,
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  ETM_SEPARATE_TREE_INTRA,
#endif
  ETM_PALETTE,
  ETM_SPLIT_QT,
  ETM_SPLIT_BT_H,
  ETM_SPLIT_BT_V,
  ETM_SPLIT_TT_H,
  ETM_SPLIT_TT_V,
  ETM_POST_DONT_SPLIT, // dummy mode to collect the data from the unsplit coding
#if REUSE_CU_RESULTS
  ETM_RECO_CACHED,
#endif
  ETM_TRIGGER_IMV_LIST,
  ETM_IBC,    // ibc mode
  ETM_IBC_MERGE, // ibc merge mode
#if MULTI_HYP_PRED
  ETM_INTER_MULTIHYP,
#endif
  ETM_INVALID
};

enum EncTestModeOpts
{
  ETO_STANDARD    =  0,                   // empty      (standard option)
  ETO_FORCE_MERGE =  1<<0,                // bit   0    (indicates forced merge)
  ETO_IMV_SHIFT   =     1,                // bits  1-3  (imv parameter starts at bit 1)
  ETO_IMV         =  7<<ETO_IMV_SHIFT,    // bits  1-3  (imv parameter uses 3 bits)
#if INTER_LIC
  ETO_LIC         = 1 << 4,               // bit   4    (local illumination compensation)
#endif
  ETO_DUMMY       =  1<<5,                // bit   5    (dummy)
  ETO_INVALID     = 0xffffffff            // bits 0-31  (invalid option)
};

static void getAreaIdx(const Area& area, const PreCalcValues &pcv, unsigned &idx1, unsigned &idx2, unsigned &idx3, unsigned &idx4)
{
  idx1 = (area.x & pcv.maxCUWidthMask)  >> MIN_CU_LOG2;
  idx2 = (area.y & pcv.maxCUHeightMask) >> MIN_CU_LOG2;
  idx3 = gp_sizeIdxInfo->idxFrom( area.width  );
  idx4 = gp_sizeIdxInfo->idxFrom( area.height );
}

struct EncTestMode
{
  EncTestMode()
    : type( ETM_INVALID ), opts( ETO_INVALID  ), qp( -1  ) {}
  EncTestMode( EncTestModeType _type )
    : type( _type       ), opts( ETO_STANDARD ), qp( -1  ) {}
  EncTestMode( EncTestModeType _type, int _qp )
    : type( _type       ), opts( ETO_STANDARD ), qp( _qp ) {}
  EncTestMode( EncTestModeType _type, EncTestModeOpts _opts, int _qp )
    : type( _type       ), opts( _opts        ), qp( _qp ) {}

  EncTestModeType type;
  EncTestModeOpts opts;
  int             qp;
  double          maxCostAllowed;
};


inline bool isModeSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     :
  case ETM_SPLIT_BT_H   :
  case ETM_SPLIT_BT_V   :
  case ETM_SPLIT_TT_H   :
  case ETM_SPLIT_TT_V   :
    return true;
  default:
    return false;
  }
}

inline bool isModeNoSplit( const EncTestMode& encTestmode )
{
  return !isModeSplit( encTestmode ) && encTestmode.type != ETM_POST_DONT_SPLIT;
}

inline bool isModeInter( const EncTestMode& encTestmode ) // perhaps remove
{
  return (   encTestmode.type == ETM_INTER_ME
          || encTestmode.type == ETM_MERGE_SKIP
#if !MERGE_ENC_OPT
          || encTestmode.type == ETM_AFFINE
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
          || encTestmode.type == ETM_AF_MMVD
#endif
#if TM_MRG && !MERGE_ENC_OPT
          || encTestmode.type == ETM_MERGE_TM
#endif
          || encTestmode.type == ETM_MERGE_GEO
          || encTestmode.type == ETM_HASH_INTER
#if MULTI_HYP_PRED
          || encTestmode.type == ETM_INTER_MULTIHYP
#endif
         );
}

inline PartSplit getPartSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     : return CU_QUAD_SPLIT;
  case ETM_SPLIT_BT_H   : return CU_HORZ_SPLIT;
  case ETM_SPLIT_BT_V   : return CU_VERT_SPLIT;
  case ETM_SPLIT_TT_H   : return CU_TRIH_SPLIT;
  case ETM_SPLIT_TT_V   : return CU_TRIV_SPLIT;
  default:                return CU_DONT_SPLIT;
  }
}

inline EncTestMode getCSEncMode( const CodingStructure& cs )
{
  return EncTestMode( EncTestModeType( (unsigned)cs.features[ENC_FT_ENC_MODE_TYPE] ),
                      EncTestModeOpts( (unsigned)cs.features[ENC_FT_ENC_MODE_OPTS] ),
                      false);
}



//////////////////////////////////////////////////////////////////////////
// EncModeCtrl controls if specific modes should be tested
//////////////////////////////////////////////////////////////////////////

struct ComprCUCtx
{
  ComprCUCtx() : testModes(), extraFeatures()
  {
  }

  ComprCUCtx( const CodingStructure& cs, const uint32_t _minDepth, const uint32_t _maxDepth, const uint32_t numExtraFeatures )
    : minDepth      ( _minDepth  )
    , maxDepth      ( _maxDepth  )
    , testModes     (            )
    , lastTestMode  (            )
    , earlySkip     ( false      )
    , isHashPerfectMatch
                    ( false      )
    , bestCS        ( nullptr    )
    , bestCU        ( nullptr    )
    , bestTU        ( nullptr    )
    , extraFeatures (            )
    , extraFeaturesd(            )
    , bestInterCost ( MAX_DOUBLE )
    , bestMtsSize2Nx2N1stPass
                    ( MAX_DOUBLE )
    , skipSecondMTSPass
                    ( false )
    , interHad      (std::numeric_limits<Distortion>::max())

#if ENABLE_SPLIT_PARALLELISM
    , isLevelSplitParallel
                    ( false )
#endif
    , bestCostWithoutSplitFlags( MAX_DOUBLE )
    , bestCostMtsFirstPassNoIsp( MAX_DOUBLE )
    , bestCostIsp   ( MAX_DOUBLE )
    , ispWasTested  ( false )
    , bestPredModeDCT2
                    ( UINT8_MAX )
    , relatedCuIsValid
                    ( false )
    , ispPredModeVal( 0 )
    , bestDCT2NonISPCost
                    ( MAX_DOUBLE )
    , bestNonDCT2Cost
                    ( MAX_DOUBLE )
    , bestISPIntraMode
                    ( UINT8_MAX )
#if JVET_V0130_INTRA_TMP
	  , tmpFlag       (false)
#endif
    , mipFlag       ( false )
    , ispMode       ( NOT_INTRA_SUBPARTITIONS )
    , ispLfnstIdx   ( 0 )
    , stopNonDCT2Transforms
                    ( false )
#if JVET_AG0058_EIP
    , eipFlag      ( false )
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    , skipSplits    ( false )
#endif
  {
    getAreaIdx( cs.area.Y(), *cs.pcv, cuX, cuY, cuW, cuH );
    partIdx = ( ( cuX << 8 ) | cuY );

    extraFeatures.reserve( numExtraFeatures );
    extraFeatures.resize ( numExtraFeatures, 0 );

    extraFeaturesd.reserve( numExtraFeatures );
    extraFeaturesd.resize ( numExtraFeatures, 0.0 );
#if INTRA_TRANS_ENC_OPT
    bestLfnstCost[0] = bestLfnstCost[1] = MAX_DOUBLE;
#endif
  }

  unsigned                          minDepth;
  unsigned                          maxDepth;
  unsigned                          cuX, cuY, cuW, cuH, partIdx;
  std::vector<EncTestMode>          testModes;
  EncTestMode                       lastTestMode;
  bool                              earlySkip;
  bool                              isHashPerfectMatch;
  CodingStructure                  *bestCS;
  CodingUnit                       *bestCU;
  TransformUnit                    *bestTU;
  static_vector<int64_t,  30>         extraFeatures;
  static_vector<double, 30>         extraFeaturesd;
  double                            bestInterCost;
  double                            bestMtsSize2Nx2N1stPass;
  bool                              skipSecondMTSPass;
  Distortion                        interHad;
#if ENABLE_SPLIT_PARALLELISM
  bool                              isLevelSplitParallel;
#endif
  double                            bestCostWithoutSplitFlags;
  double                            bestCostMtsFirstPassNoIsp;
  double                            bestCostIsp;
#if INTRA_TRANS_ENC_OPT
  double                            bestLfnstCost[2];
#endif
  bool                              ispWasTested;
  uint16_t                          bestPredModeDCT2;
  bool                              relatedCuIsValid;
  uint16_t                          ispPredModeVal;
  double                            bestDCT2NonISPCost;
  double                            bestNonDCT2Cost;
  uint8_t                           bestISPIntraMode;
#if JVET_V0130_INTRA_TMP
  bool							              	tmpFlag;
#endif
  bool                              mipFlag;
  uint8_t                           ispMode;
  uint8_t                           ispLfnstIdx;
  bool                              stopNonDCT2Transforms;
#if JVET_AG0058_EIP
  bool                              eipFlag;
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool                              skipSplits;
#endif

  template<typename T> T    get( int ft )       const { return typeid(T) == typeid(double) ? (T&)extraFeaturesd[ft] : T(extraFeatures[ft]); }
  template<typename T> void set( int ft, T val )      { extraFeatures [ft] = int64_t( val ); }
  void                      set( int ft, double val ) { extraFeaturesd[ft] = val; }
#if INTRA_TRANS_ENC_OPT
  bool   isLfnstTested()                        const { return (bestLfnstCost[0] != MAX_DOUBLE && bestLfnstCost[1] != MAX_DOUBLE); }
#endif
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl - abstract class specifying the general flow of mode control
//////////////////////////////////////////////////////////////////////////

class EncModeCtrl
{
protected:

  const EncCfg         *m_pcEncCfg;
  const class RateCtrl *m_pcRateCtrl;
        class RdCost   *m_pcRdCost;
  const Slice          *m_slice;
#if SHARP_LUMA_DELTA_QP
  int                   m_lumaLevelToDeltaQPLUT[LUMA_LEVEL_TO_DQP_LUT_MAXSIZE];
  int                   m_lumaQPOffset;
#endif
#if JVET_Y0240_BIM
  std::map<int, int*>  *m_bimQPMap;
#endif
  bool                  m_fastDeltaQP;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList[MAX_TREE_TYPE];
  ComprCUCtx           *m_nonIntraCUECtx;
#else
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList;
#endif
#if ENABLE_SPLIT_PARALLELISM
  int                   m_runNextInParallel;
#endif
  InterSearch*          m_pcInterSearch;

  bool                  m_doPlt;
#if JVET_AJ0226_MTT_SKIP
  double                m_noSplitIntraRdCost64CU;
  double                m_noSplitIntraRdCost32CU;
  int                   m_cabacBitsforBTH[4];
  int                   m_cabacBitsforBTV[4];
  int                   m_cabacBitsforTTH[4];
  int                   m_cabacBitsforTTV[4];
  int                   m_cabacBitsforQT[4];
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool                  m_avoidComplexIntraInInterSlice;
  Distortion            m_savedInterHad;
  unsigned              m_treeIdx;
#endif

public:

  virtual ~EncModeCtrl              () {}

  virtual void create               ( const EncCfg& cfg )                                                                   = 0;
  virtual void destroy              ()                                                                                      = 0;
  virtual void initCTUEncoding      ( const Slice &slice )                                                                  = 0;
  virtual void initCULevel          ( Partitioner &partitioner, const CodingStructure& cs )                                 = 0;
  virtual void finishCULevel        ( Partitioner &partitioner )                                                            = 0;

protected:

  virtual bool tryMode              ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) = 0;

public:

  virtual bool useModeResult        ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner ) = 0;
  virtual bool checkSkipOtherLfnst  ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner ) = 0;
#if ENABLE_SPLIT_PARALLELISM
  virtual void copyState            ( const EncModeCtrl& other, const UnitArea& area );
  virtual int  getNumParallelJobs   ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return 1;     }
  virtual bool isParallelSplit      ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return false; }
  virtual bool parallelJobSelector  ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const { return true;  }
          void setParallelSplit     ( bool val ) { m_runNextInParallel = val; }
#endif

  void         init                 ( EncCfg *pCfg, RateCtrl *pRateCtrl, RdCost *pRdCost );
  bool         tryModeMaster        ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool         nextMode             ( const CodingStructure &cs, Partitioner &partitioner );
  EncTestMode  currTestMode         () const;
  EncTestMode  lastTestMode         () const;
  void         setEarlySkipDetected ();
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void         setIsHashPerfectMatch( bool b ) { m_ComprCUCtxList[m_treeIdx].back().isHashPerfectMatch = b; }
  bool         getIsHashPerfectMatch() { return m_ComprCUCtxList[m_treeIdx].back().isHashPerfectMatch; }
#else
  void         setIsHashPerfectMatch( bool b ) { m_ComprCUCtxList.back().isHashPerfectMatch = b; }
  bool         getIsHashPerfectMatch() { return m_ComprCUCtxList.back().isHashPerfectMatch; }
#endif
  virtual void setBest              ( CodingStructure& cs );
  bool         anyMode              () const;
#if JVET_AI0087_BTCUS_RESTRICTION
  bool         isLumaNonBoundaryCu(const Partitioner& partitioner, SizeType picWidth, SizeType picHeight);
#endif
#if JVET_AJ0226_MTT_SKIP
  void         setNoSplitIntraCost64CU(double cost) { m_noSplitIntraRdCost64CU = cost; }
  void         setNoSplitIntraCost32CU  (double cost) { m_noSplitIntraRdCost32CU = cost;}

  void         setBthCabacBits(int bitsCabac, int pos) { m_cabacBitsforBTH[pos]  = bitsCabac; }
  void         setBtvCabacBits(int bitsCabac, int pos) { m_cabacBitsforBTV[pos]  = bitsCabac; }
  void         setTthCabacBits(int bitsCabac, int pos) { m_cabacBitsforTTH[pos] = bitsCabac; }
  void         setTtvCabacBits(int bitsCabac, int pos) { m_cabacBitsforTTV[pos] = bitsCabac; }
  void         setQtCabacBits (int bitsCabac, int pos) { m_cabacBitsforQT[pos] = bitsCabac; }


  int         getBthCabacBits(int pos) { return m_cabacBitsforBTH[pos] ; }
  int         getBtvCabacBits(int pos) { return m_cabacBitsforBTV[pos] ; }
  int         getTthCabacBits(int pos) { return m_cabacBitsforTTH[pos] ; }
  int         getTtvCabacBits(int pos) { return m_cabacBitsforTTV[pos] ; }
  int         getQtCabacBits (int pos)  { return m_cabacBitsforQT[pos]  ; }

#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  ComprCUCtx&       getComprCUCtx    ()                      { CHECK( m_ComprCUCtxList[m_treeIdx].empty(), "Accessing empty list!"); return m_ComprCUCtxList[m_treeIdx].back(); }
  void              setNonIntraCUECtx( ComprCUCtx& c )       { m_nonIntraCUECtx = &c; }
  const Slice*      getSlice()                               { return m_slice; }
  virtual void      setTreeIdx() = 0;
#else
  const ComprCUCtx& getComprCUCtx   () { CHECK( m_ComprCUCtxList.empty(), "Accessing empty list!"); return m_ComprCUCtxList.back(); }
#endif

#if SHARP_LUMA_DELTA_QP
  void                  initLumaDeltaQpLUT();
  int                   calculateLumaDQP  ( const CPelBuf& rcOrg );
#endif
  void setFastDeltaQp                 ( bool b )                {        m_fastDeltaQP = b;                               }
  bool getFastDeltaQp                 ()                  const { return m_fastDeltaQP;                                   }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void popLastCmprCUCtx               () { m_ComprCUCtxList[m_treeIdx].pop_back(); }
  void skipSplits                     () { m_ComprCUCtxList[m_treeIdx].back().skipSplits = true; }

  double getBestInterCost             ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestInterCost;           }
  Distortion getInterHad              ()                  const { return m_ComprCUCtxList[m_treeIdx].back().interHad;                }
  void enforceInterHad                ( Distortion had )        {        m_ComprCUCtxList[m_treeIdx].back().interHad = had;          }
  double getMtsSize2Nx2NFirstPassCost ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestMtsSize2Nx2N1stPass; }
  bool   getSkipSecondMTSPass         ()                  const { return m_ComprCUCtxList[m_treeIdx].back().skipSecondMTSPass;       }
  void   setSkipSecondMTSPass         ( bool b )                { m_ComprCUCtxList[m_treeIdx].back().skipSecondMTSPass = b;          }
  double getBestCostWithoutSplitFlags ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestCostWithoutSplitFlags;         }
  void   setBestCostWithoutSplitFlags ( double cost )           { m_ComprCUCtxList[m_treeIdx].back().bestCostWithoutSplitFlags = cost;         }
  double getMtsFirstPassNoIspCost     ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestCostMtsFirstPassNoIsp;         }
  void   setMtsFirstPassNoIspCost     ( double cost )           { m_ComprCUCtxList[m_treeIdx].back().bestCostMtsFirstPassNoIsp = cost;         }
  double getIspCost                   ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestCostIsp; }
  void   setIspCost                   ( double val )            { m_ComprCUCtxList[m_treeIdx].back().bestCostIsp = val; }
#if INTRA_TRANS_ENC_OPT
  void   resetLfnstCost               ()                        { m_ComprCUCtxList[m_treeIdx].back().bestLfnstCost[0] = m_ComprCUCtxList[m_treeIdx].back().bestLfnstCost[1] = MAX_DOUBLE; }
#endif
  bool   getISPWasTested              ()                  const { return m_ComprCUCtxList[m_treeIdx].back().ispWasTested; }
  void   setISPWasTested              ( bool val )              { m_ComprCUCtxList[m_treeIdx].back().ispWasTested = val; }
  void   setBestPredModeDCT2          ( uint16_t val )          { m_ComprCUCtxList[m_treeIdx].back().bestPredModeDCT2 = val; }
  uint16_t getBestPredModeDCT2        ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestPredModeDCT2; }
  bool   getRelatedCuIsValid          ()                  const { return m_ComprCUCtxList[m_treeIdx].back().relatedCuIsValid; }
  void   setRelatedCuIsValid          ( bool val )              { m_ComprCUCtxList[m_treeIdx].back().relatedCuIsValid = val; }
  uint16_t getIspPredModeValRelCU     ()                  const { return m_ComprCUCtxList[m_treeIdx].back().ispPredModeVal; }
  void   setIspPredModeValRelCU       ( uint16_t val )          { m_ComprCUCtxList[m_treeIdx].back().ispPredModeVal = val; }
  double getBestDCT2NonISPCostRelCU   ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestDCT2NonISPCost; }
  void   setBestDCT2NonISPCostRelCU   ( double val )            { m_ComprCUCtxList[m_treeIdx].back().bestDCT2NonISPCost = val; }
  double getBestNonDCT2Cost           ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestNonDCT2Cost; }
  void   setBestNonDCT2Cost           ( double val )            { m_ComprCUCtxList[m_treeIdx].back().bestNonDCT2Cost = val; }
  uint8_t getBestISPIntraModeRelCU    ()                  const { return m_ComprCUCtxList[m_treeIdx].back().bestISPIntraMode; }
  void   setBestISPIntraModeRelCU     ( uint8_t val )           { m_ComprCUCtxList[m_treeIdx].back().bestISPIntraMode = val; }
#if JVET_V0130_INTRA_TMP
  void   setTPMFlagISPPass            (bool val)                { m_ComprCUCtxList[m_treeIdx].back().tmpFlag = val; }
#endif
#if JVET_AG0058_EIP
  void   setAeipFlagISPPass           (bool val)                { m_ComprCUCtxList[m_treeIdx].back().eipFlag = val; }
#endif
  void   setMIPFlagISPPass            ( bool val )              { m_ComprCUCtxList[m_treeIdx].back().mipFlag = val; }
  void   setISPMode                   ( uint8_t val )           { m_ComprCUCtxList[m_treeIdx].back().ispMode = val; }
  void   setISPLfnstIdx               ( uint8_t val )           { m_ComprCUCtxList[m_treeIdx].back().ispLfnstIdx = val; }
  bool   getStopNonDCT2Transforms     ()                  const { return m_ComprCUCtxList[m_treeIdx].back().stopNonDCT2Transforms; }
  void   setStopNonDCT2Transforms     ( bool val )              { m_ComprCUCtxList[m_treeIdx].back().stopNonDCT2Transforms = val; }

#else
  double getBestInterCost             ()                  const { return m_ComprCUCtxList.back().bestInterCost;           }
  Distortion getInterHad              ()                  const { return m_ComprCUCtxList.back().interHad;                }
  void enforceInterHad                ( Distortion had )        {        m_ComprCUCtxList.back().interHad = had;          }
  double getMtsSize2Nx2NFirstPassCost ()                  const { return m_ComprCUCtxList.back().bestMtsSize2Nx2N1stPass; }
  bool   getSkipSecondMTSPass         ()                  const { return m_ComprCUCtxList.back().skipSecondMTSPass;       }
  void   setSkipSecondMTSPass         ( bool b )                { m_ComprCUCtxList.back().skipSecondMTSPass = b;          }
  double getBestCostWithoutSplitFlags ()                  const { return m_ComprCUCtxList.back().bestCostWithoutSplitFlags;         }
  void   setBestCostWithoutSplitFlags ( double cost )           { m_ComprCUCtxList.back().bestCostWithoutSplitFlags = cost;         }
  double getMtsFirstPassNoIspCost     ()                  const { return m_ComprCUCtxList.back().bestCostMtsFirstPassNoIsp;         }
  void   setMtsFirstPassNoIspCost     ( double cost )           { m_ComprCUCtxList.back().bestCostMtsFirstPassNoIsp = cost;         }
  double getIspCost                   ()                  const { return m_ComprCUCtxList.back().bestCostIsp; }
  void   setIspCost                   ( double val )            { m_ComprCUCtxList.back().bestCostIsp = val; }
#if INTRA_TRANS_ENC_OPT
  void   resetLfnstCost               ()                        { m_ComprCUCtxList.back().bestLfnstCost[0] = m_ComprCUCtxList.back().bestLfnstCost[1] = MAX_DOUBLE; }
#endif
  bool   getISPWasTested              ()                  const { return m_ComprCUCtxList.back().ispWasTested; }
  void   setISPWasTested              ( bool val )              { m_ComprCUCtxList.back().ispWasTested = val; }
  void   setBestPredModeDCT2          ( uint16_t val )          { m_ComprCUCtxList.back().bestPredModeDCT2 = val; }
  uint16_t getBestPredModeDCT2        ()                  const { return m_ComprCUCtxList.back().bestPredModeDCT2; }
  bool   getRelatedCuIsValid          ()                  const { return m_ComprCUCtxList.back().relatedCuIsValid; }
  void   setRelatedCuIsValid          ( bool val )              { m_ComprCUCtxList.back().relatedCuIsValid = val; }
  uint16_t getIspPredModeValRelCU     ()                  const { return m_ComprCUCtxList.back().ispPredModeVal; }
  void   setIspPredModeValRelCU       ( uint16_t val )          { m_ComprCUCtxList.back().ispPredModeVal = val; }
  double getBestDCT2NonISPCostRelCU   ()                  const { return m_ComprCUCtxList.back().bestDCT2NonISPCost; }
  void   setBestDCT2NonISPCostRelCU   ( double val )            { m_ComprCUCtxList.back().bestDCT2NonISPCost = val; }
  double getBestNonDCT2Cost           ()                  const { return m_ComprCUCtxList.back().bestNonDCT2Cost; }
  void   setBestNonDCT2Cost           ( double val )            { m_ComprCUCtxList.back().bestNonDCT2Cost = val; }
  uint8_t getBestISPIntraModeRelCU    ()                  const { return m_ComprCUCtxList.back().bestISPIntraMode; }
  void   setBestISPIntraModeRelCU     ( uint8_t val )           { m_ComprCUCtxList.back().bestISPIntraMode = val; }
#if JVET_V0130_INTRA_TMP
  void   setTPMFlagISPPass            (bool val)                { m_ComprCUCtxList.back().tmpFlag = val; }
#endif
#if JVET_AG0058_EIP
  void   setAeipFlagISPPass           (bool val)                { m_ComprCUCtxList.back().eipFlag = val; }
#endif
  void   setMIPFlagISPPass            ( bool val )              { m_ComprCUCtxList.back().mipFlag = val; }
  void   setISPMode                   ( uint8_t val )           { m_ComprCUCtxList.back().ispMode = val; }
  void   setISPLfnstIdx               ( uint8_t val )           { m_ComprCUCtxList.back().ispLfnstIdx = val; }
  bool   getStopNonDCT2Transforms     ()                  const { return m_ComprCUCtxList.back().stopNonDCT2Transforms; }
  void   setStopNonDCT2Transforms     ( bool val )              { m_ComprCUCtxList.back().stopNonDCT2Transforms = val; }
#endif 


#if JVET_AJ0226_MTT_SKIP 
  void resetSplitSignalCostParams()
  {
    std::memset(m_cabacBitsforTTH, 0, sizeof(m_cabacBitsforTTH));
    std::memset(m_cabacBitsforTTV, 0, sizeof(m_cabacBitsforTTV));
    std::memset(m_cabacBitsforBTH, 0, sizeof(m_cabacBitsforBTH));
    std::memset(m_cabacBitsforBTV, 0, sizeof(m_cabacBitsforBTV));
    std::memset(m_cabacBitsforQT, 0, sizeof(m_cabacBitsforQT));
  }
#endif

  void setInterSearch                 (InterSearch* pcInterSearch)   { m_pcInterSearch = pcInterSearch; }
  void   setPltEnc                    ( bool b )                { m_doPlt = b; }
  bool   getPltEnc()                                      const { return m_doPlt; }
#if JVET_Y0240_BIM
  void   setBIMQPMap                  ( std::map<int, int*> *qpMap ) { m_bimQPMap = qpMap; }
  int    getBIMOffset                 ( int poc, int ctuId )
  {
    auto it = m_bimQPMap->find(poc);
    return (it == m_bimQPMap->end()) ? 0 : (*m_bimQPMap)[poc][ctuId];
  }
#endif

#if JVET_Z0118_GDR
void forceIntraMode()
{ 
  // remove all inter or split to force make intra
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();   
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size(); 
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (isModeInter(etm.type))
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;          
    }
  }  
}

void forceIntraNoSplit()
{
  // remove all inter or split to force make intra        
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif

  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (isModeInter(etm.type) || isModeSplit(etm.type)) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }  
}

// Note: ForceInterMode
void forceInterMode()
{    
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_INTRA) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;        
    }
  }  
}

void removeHashInter()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_HASH_INTER) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void removeMergeSkip()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_MERGE_SKIP) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void removeInterME()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_INTER_ME) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

#if !MERGE_ENC_OPT
void removeAffine()
{
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
  for (int j = 0; j < n; j++) 
  {
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
    if (etm.type == ETM_AFFINE) 
    {
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
      j--;
      n--;
    }
  }
}
#endif

void removeMergeGeo()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_MERGE_GEO) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void removeIntra()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type == ETM_INTRA) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void removeBadMode()
{  
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_INTER_ME && ((etm.opts & ETO_IMV) >> ETO_IMV_SHIFT) > 2) 
    {  
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
      break;
    }
  }  
}

bool anyPredModeLeft()
{ 
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_HASH_INTER ||
        etm.type == ETM_MERGE_SKIP || 
        etm.type == ETM_INTER_ME   || 
#if !MERGE_ENC_OPT
        etm.type == ETM_AFFINE     || 
#endif
        etm.type == ETM_MERGE_GEO  || 
        etm.type == ETM_INTRA      ||
        etm.type == ETM_PALETTE    || 
        etm.type == ETM_IBC        ||
        etm.type == ETM_IBC_MERGE) {
      return true;
    }
  }

  return false;
}

bool anyIntraIBCMode()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif

  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_INTRA || etm.type == ETM_IBC) 
    {
      return true;
    }
  }

  return false;
}

void forceRemovePredMode()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif

  for (int j = 0; j < n; j++)
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (
      etm.type == ETM_HASH_INTER
      || etm.type == ETM_MERGE_SKIP
      || etm.type == ETM_INTER_ME
#if !MERGE_ENC_OPT
      || etm.type == ETM_AFFINE
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
      || etm.type == ETM_AF_MMVD
#endif
#if TM_MRG && !MERGE_ENC_OPT
      || etm.type == ETM_MERGE_TM
#endif
      || etm.type == ETM_MERGE_GEO
      || etm.type == ETM_INTRA
      || etm.type == ETM_PALETTE
      || etm.type == ETM_IBC
      || etm.type == ETM_IBC_MERGE
#if MULTI_HYP_PRED
      || etm.type == ETM_INTER_MULTIHYP
#endif
      )
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveDontSplit()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif

  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_POST_DONT_SPLIT) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceVerSplitOnly()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];       
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];       
#endif

    if (etm.type == ETM_SPLIT_BT_H || etm.type == ETM_SPLIT_TT_H)
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }   
}

void forceRemoveTTH()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++)
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_TT_H)
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveTTV()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    
    if (etm.type == ETM_SPLIT_TT_V) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveBTH()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++)
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_BT_H)
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveBTV()
{  
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_BT_V) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveQT()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif

  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_QT) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }  
}

void forceRemoveHT()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_BT_H || etm.type == ETM_SPLIT_TT_H) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif

      j--;
      n--;
    }
  }
}

void forceRemoveQTHT()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_QT || etm.type == ETM_SPLIT_BT_H || etm.type == ETM_SPLIT_TT_H) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceRemoveAllSplit()
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif

    if (etm.type == ETM_SPLIT_QT || etm.type == ETM_SPLIT_BT_H || etm.type == ETM_SPLIT_BT_V || etm.type == ETM_SPLIT_TT_H || etm.type == ETM_SPLIT_TT_V) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;
    }
  }
}

void forceQTonlyMode()
{
  // remove all split except QT  
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];
#endif
    if (etm.type != ETM_SPLIT_QT) 
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      m_ComprCUCtxList[m_treeIdx].back().testModes.erase(m_ComprCUCtxList[m_treeIdx].back().testModes.begin() + j);
#else
      m_ComprCUCtxList.back().testModes.erase(m_ComprCUCtxList.back().testModes.begin() + j);
#endif
      j--;
      n--;        
    }
  }    
}

const char* printType(EncTestModeType type)
{
  char *ret;

  switch (type)
  {
  case  ETM_HASH_INTER:      ret = strdup("Hash"); break;
  case  ETM_MERGE_SKIP:      ret = strdup("Mkip"); break;
  case  ETM_INTER_ME:        ret = strdup("InterMe"); break;
#if !MERGE_ENC_OPT
  case  ETM_AFFINE:          ret = strdup("Affi"); break;
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
  case  ETM_AF_MMVD:         ret = strdup("AfMMVD"); break;
#endif
#if TM_MRG && !MERGE_ENC_OPT
  case  ETM_MERGE_TM:        ret = strdup("MergeTM"); break;
#endif
  case  ETM_MERGE_GEO:       ret = strdup("MergeGeo"); break;
  case  ETM_INTRA:           ret = strdup("Intra"); break;
  case  ETM_PALETTE:         ret = strdup("Palet"); break;

  case  ETM_SPLIT_QT:        ret = strdup("QT"); break;
  case  ETM_SPLIT_BT_H:      ret = strdup("BTH"); break;
  case  ETM_SPLIT_BT_V:      ret = strdup("BTV"); break;
  case  ETM_SPLIT_TT_H:      ret = strdup("TTH"); break;
  case  ETM_SPLIT_TT_V:      ret = strdup("TTV"); break;
  case  ETM_POST_DONT_SPLIT: ret = strdup("|"); break;
#if REUSE_CU_RESULTS    
  case ETM_RECO_CACHED:      ret = strdup("CACHE"); break;
#endif
  case ETM_TRIGGER_IMV_LIST: ret = strdup("TrigIMVList"); break;  
  case ETM_IBC:              ret = strdup("IBC"); break;
  case ETM_IBC_MERGE:        ret = strdup("IBCMerge"); break;
#if MULTI_HYP_PRED
  case ETM_INTER_MULTIHYP:   ret = strdup("MulHyp"); break;
#endif
  default:
    ret = strdup("INVALID");
  }

  return ret;
}

void printMode()
{
  // remove all inter or split to force make intra        
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  int n = (int)m_ComprCUCtxList[m_treeIdx].back().testModes.size();
#else
  int n = (int)m_ComprCUCtxList.back().testModes.size();
#endif
  printf("-:[");
  for (int j = 0; j < n; j++) 
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    const EncTestMode etm = m_ComprCUCtxList[m_treeIdx].back().testModes[j];      
#else
    const EncTestMode etm = m_ComprCUCtxList.back().testModes[j];      
#endif
    printf(" %s", printType(etm.type));      
  }
  printf("]\n");   
}
#endif

protected:
  void xExtractFeatures ( const EncTestMode encTestmode, CodingStructure& cs );
  void xGetMinMaxQP     ( int& iMinQP, int& iMaxQP, const CodingStructure& cs, const Partitioner &pm, const int baseQP, const SPS& sps, const PPS& pps, const PartSplit splitMode );
  int  xComputeDQP      ( const CodingStructure &cs, const Partitioner &pm );
};


//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////
struct SaveLoadStructSbt
{
  uint8_t  numPuInfoStored;
  uint32_t puSse[SBT_NUM_SL];
  uint8_t  puSbt[SBT_NUM_SL];
  uint8_t  puTrs[SBT_NUM_SL];
};

class SaveLoadEncInfoSbt
{
protected:
#if ENABLE_SPLIT_PARALLELISM
public:
#endif
  void init( const Slice &slice );
#if ENABLE_SPLIT_PARALLELISM
protected:
#endif
  int m_maxCuSize;
  void create(const int maxCuSize);
  void destroy();

private:
  SaveLoadStructSbt ****m_saveLoadSbt;
  Slice const       *m_sliceSbt;

public:
  virtual  ~SaveLoadEncInfoSbt() { }
  void     resetSaveloadSbt( int maxSbtSize );
  uint16_t findBestSbt( const UnitArea& area, const uint32_t curPuSse );
  bool     saveBestSbt( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt, const uint8_t curPuTrs );
#if ENABLE_SPLIT_PARALLELISM
  void     copyState(const SaveLoadEncInfoSbt& other);
#endif
};

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool     isInter;
  bool     isIntra;
  bool     isSkip;
  bool     isMMVDSkip;
  bool     isIBC;
#if JVET_W0097_GPM_MMVD_TM
  bool     skipGPM;
  char     isGPMTested;

#if JVET_AG0164_AFFINE_GPM
  int      geoDirCandList[GEO_MAX_TRY_WEIGHTED_SATD + 3];
  int      numGeoDirCand;
  int      geoMrgIdx0List[GEO_MAX_TRY_WEIGHTED_SATD + 3];
  int      geoMrgIdx1List[GEO_MAX_TRY_WEIGHTED_SATD + 3];
#else
  int      geoDirCandList[GEO_MAX_TRY_WEIGHTED_SATD];
  int      numGeoDirCand;
  int      geoMrgIdx0List[GEO_MAX_TRY_WEIGHTED_SATD];
  int      geoMrgIdx1List[GEO_MAX_TRY_WEIGHTED_SATD];
#endif

#endif
#if JVET_AD0213_LIC_IMP
  bool    skipLIC;
#endif
#if JVET_AA0070_RRIBC
  bool     isRribcCoded;
  bool     isRribcTested;
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  bool     skipFracTmp;
#endif
#if JVET_AJ0082_MM_EIP
  bool     skipEip;
#endif
#if JVET_AL0108_BVG_DIMD
  bool     skipBvgDimd;
#endif

  bool     validMv[NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv       saveMv [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];

#if MULTI_HYP_PRED
  uint8_t  numAddHyp;
#endif
  uint8_t  bcwIdx;
  char     selectColorSpaceOption;  // 0 - test both two color spaces; 1 - only test the first color spaces; 2 - only test the second color spaces
  uint16_t ispPredModeVal;
  double   bestDCT2NonISPCost;
  double   bestCost;
  double   bestNonDCT2Cost;
  bool     relatedCuIsValid;
  uint8_t  bestISPIntraMode;
#if INTRA_TRANS_ENC_OPT
  double   bestCostForLfnst;
  bool     relatedCuLfnstIsValid;
  bool     skipLfnstTest;
#endif
#if ENABLE_SPLIT_PARALLELISM
 uint64_t  temporalId;
#endif
#if JVET_AB0092_GLM_WITH_LUMA
  bool     skipGLM;
#endif
};

class CacheBlkInfoCtrl
{
private:

  unsigned         m_numWidths, m_numHeights;
  Slice const     *m_slice_chblk;
  // x in CTU, y in CTU, width, height
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CodedCUInfo   ***m_codedCUInfo[MAX_TREE_TYPE][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
#else
  CodedCUInfo   ***m_codedCUInfo[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
#endif

protected:

  int m_maxCuSize;
  void create   (const int maxCuSize);
  void destroy  ();
#if ENABLE_SPLIT_PARALLELISM
public:
#endif
  void init     ( const Slice &slice );
#if ENABLE_SPLIT_PARALLELISM
private:
  uint64_t
       m_currTemporalId;
public:
  void tick     () { m_currTemporalId++; CHECK( m_currTemporalId <= 0, "Problem with integer overflow!" ); }
  // mark the state of the blk as changed within the current temporal id
  void copyState( const CacheBlkInfoCtrl &other, const UnitArea& area );
protected:
  void touch    ( const UnitArea& area );
#endif
#if JVET_W0097_GPM_MMVD_TM || INTRA_TRANS_ENC_OPT
public:
#endif
  CodedCUInfo& getBlkInfo( const UnitArea& area );
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CodedCUInfo* getBlkInfoPtr(const UnitArea& area);
#endif
public:

  virtual ~CacheBlkInfoCtrl() {}

  bool isSkip ( const UnitArea& area );
  bool isMMVDSkip(const UnitArea& area);
  bool getMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx,       Mv& rMv ) const;
  void setMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, const Mv& rMv );

  bool  getInter( const UnitArea& area );
  void  setBcwIdx( const UnitArea& area, uint8_t gBiIdx );
  uint8_t getBcwIdx( const UnitArea& area );

  char  getSelectColorSpaceOption(const UnitArea& area);
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  unsigned  m_treeIdx;
#endif
};

#if REUSE_CU_RESULTS
struct BestEncodingInfo
{
  CodingUnit     cu;
  PredictionUnit pu;
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  std::vector<TransformUnit> tus;
#elif REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  TransformUnit  tus[MAX_NUM_TUS];
  size_t         numTus;
#else
  TransformUnit  tu;
#endif

  int            poc;

#if ENABLE_SPLIT_PARALLELISM
  int64_t        temporalId;
#endif
};

class BestEncInfoCache
{
private:

  unsigned            m_numWidths, m_numHeights;
  const Slice        *m_slice_bencinf;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  BestEncodingInfo ***m_bestEncInfo[MAX_TREE_TYPE][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  TCoeff             *m_pCoeff[MAX_TREE_TYPE];
#if SIGN_PREDICTION
  SIGN_PRED_TYPE     *m_pCoeffSign[MAX_TREE_TYPE];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned           *m_pCoeffSignScanIdx[MAX_TREE_TYPE];
#endif
#endif
  Pel                *m_pPcmBuf[MAX_TREE_TYPE];

  bool               *m_runType[MAX_TREE_TYPE];
#else
  BestEncodingInfo ***m_bestEncInfo[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  TCoeff             *m_pCoeff;
#if SIGN_PREDICTION
  SIGN_PRED_TYPE *m_pCoeffSign;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned           *m_pCoeffSignScanIdx;
#endif
#endif
  Pel                *m_pPcmBuf;

  bool               *m_runType;
#endif

  CodingStructure     m_dummyCS;
  XUCache             m_dummyCache;
#if ENABLE_SPLIT_PARALLELISM
  int64_t m_currTemporalId;
#endif
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  int                 m_maxNumTUs;
#endif

protected:

#if CONVERT_NUM_TU_SPLITS_TO_CFG
  int m_maxCuSize;
  void create(const ChromaFormat chFmt, const int maxNumTUs, const int maxCuSize);
#else
  void create   ( const ChromaFormat chFmt );
#endif
  void destroy  ();

  bool setFromCs( const CodingStructure& cs, const Partitioner& partitioner );
  bool isValid  ( const CodingStructure &cs, const Partitioner &partitioner, int qp );

#if ENABLE_SPLIT_PARALLELISM
  void touch    ( const UnitArea& area );
#endif
public:

  BestEncInfoCache() : m_slice_bencinf( nullptr ), m_dummyCS( m_dummyCache.cuCache, m_dummyCache.puCache, m_dummyCache.tuCache ) {}
  virtual ~BestEncInfoCache() {}

#if ENABLE_SPLIT_PARALLELISM
  void     copyState( const BestEncInfoCache &other, const UnitArea &area );
  void     tick     () { m_currTemporalId++; CHECK( m_currTemporalId <= 0, "Problem with integer overflow!" ); }
#endif
  void     init     ( const Slice &slice );
  bool     setCsFrom( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  unsigned m_treeIdx;
#endif
};

#endif
//////////////////////////////////////////////////////////////////////////
// EncModeCtrlMTnoRQT - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////
#if JVET_W0097_GPM_MMVD_TM
enum ExtraFeatures
{
  DID_HORZ_SPLIT = 0,
  DID_VERT_SPLIT,
  DID_QUAD_SPLIT,
  BEST_HORZ_SPLIT_COST,
  BEST_VERT_SPLIT_COST,
  BEST_TRIH_SPLIT_COST,
  BEST_TRIV_SPLIT_COST,
  DO_TRIH_SPLIT,
  DO_TRIV_SPLIT,
  BEST_NON_SPLIT_COST,
  BEST_NO_IMV_COST,
  BEST_IMV_COST,
  BEST_GPM_COST,
#if JVET_AD0213_LIC_IMP
  BEST_LIC_COST,
#endif
  QT_BEFORE_BT,
  IS_BEST_NOSPLIT_SKIP,
  MAX_QT_SUB_DEPTH,
#if REUSE_CU_RESULTS
  IS_REUSING_CU,
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  BEST_INTRA_NZ_CNT,
#endif
  NUM_EXTRA_FEATURES
};
#endif
class EncModeCtrlMTnoRQT : public EncModeCtrl, public CacheBlkInfoCtrl
#if REUSE_CU_RESULTS
  , public BestEncInfoCache
#endif
  , public SaveLoadEncInfoSbt
{
#if !JVET_W0097_GPM_MMVD_TM
  enum ExtraFeatures
  {
    DID_HORZ_SPLIT = 0,
    DID_VERT_SPLIT,
    DID_QUAD_SPLIT,
    BEST_HORZ_SPLIT_COST,
    BEST_VERT_SPLIT_COST,
    BEST_TRIH_SPLIT_COST,
    BEST_TRIV_SPLIT_COST,
    DO_TRIH_SPLIT,
    DO_TRIV_SPLIT,
    BEST_NON_SPLIT_COST,
    BEST_NO_IMV_COST,
    BEST_IMV_COST,
#if JVET_AD0213_LIC_IMP
    BEST_LIC_COST,
#endif
    QT_BEFORE_BT,
    IS_BEST_NOSPLIT_SKIP,
    MAX_QT_SUB_DEPTH,
#if REUSE_CU_RESULTS
    IS_REUSING_CU,
#endif
    NUM_EXTRA_FEATURES
  };
#endif
  unsigned m_skipThreshold;
#if JVET_Z0118_GDR
  EncCfg m_encCfg;
#endif

public:

  virtual void create             ( const EncCfg& cfg );
  virtual void destroy            ();
  virtual void initCTUEncoding    ( const Slice &slice );
  virtual void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs );
  virtual void finishCULevel      ( Partitioner &partitioner );

  virtual bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  virtual bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner );

#if ENABLE_SPLIT_PARALLELISM
  virtual void copyState          ( const EncModeCtrl& other, const UnitArea& area );

  virtual int  getNumParallelJobs ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool isParallelSplit    ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const;
#endif
  virtual bool checkSkipOtherLfnst( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner );
#if JVET_Y0152_TT_ENC_SPEEDUP
  bool xSkipTreeCandidate(const PartSplit split, const double* splitRdCostBest, const SliceType& sliceType) const;
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void setTreeIdx();
#endif
};


//! \}

#endif // __ENCMODECTRL__
