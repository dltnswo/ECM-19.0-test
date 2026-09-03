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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"


//! \ingroup EncoderLib
//! \{


class EncCu;
class CABACWriter
{
public:
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  CABACWriter( BinEncIf& binEncoder, CABACDataStore* cabacDataStore ) : m_BinEncoder( binEncoder ), m_Bitstream( 0 )
  {
    m_TestCtx = m_BinEncoder.getCtx(); m_EncCu = NULL;
    m_CABACDataStore = cabacDataStore;
  }
#else
  CABACWriter(BinEncIf& binEncoder)   : m_BinEncoder(binEncoder), m_Bitstream(0) { m_TestCtx = m_BinEncoder.getCtx(); m_EncCu = NULL; }
#endif
  virtual ~CABACWriter() {}

public:
#if JVET_AG0196_CABAC_RETRAIN
  void initCtxModels(Slice &slice);
#else
  void        initCtxModels(const Slice &slice);
#endif
#if JVET_AI0087_BTCUS_RESTRICTION
  bool         isLumaNonBoundaryCu(const Partitioner& partitioner, SizeType picWidth, SizeType picHeight);
  void         setBtFirstPart(Partitioner& partitioner, SizeType blockSize, PartSplit setValue);
#endif
  void        setEncCu(EncCu* pcEncCu) { m_EncCu = pcEncCu; }
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  void                  setBinBufferActive (bool b)      { m_BinEncoder.setBinBufferActive(b); }
  void                  setBinBuffer(BinStoreVector *bb) { m_BinEncoder.setBinBuffer(bb);      }
  const BinStoreVector *getBinBuffer()             const { return m_BinEncoder.getBinBuffer(); }
  void                  updateCtxs  (BinStoreVector *bb) { m_BinEncoder.updateCtxs(bb);        }
#endif

public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          (       CodingStructure&        cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false, bool skipAlf = false
#if JVET_AL0153_ALF_CCCM
    , bool skipLfCccm = false
#endif
  );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
#if JVET_V0094_BILATERAL_FILTER
  void        bif                      ( const ComponentID compID, const Slice&                   slice, const BifParams& bifParams);
  void        bif                      ( const ComponentID compID, const Slice& slice, const BifParams& bifParams, unsigned ctuRsAddr);
#endif
#if JVET_W0066_CCSAO
  void        codeCcSaoControlIdc       ( uint8_t idcVal, CodingStructure &cs, const ComponentID compID, const int curIdx,
                                          const uint8_t *controlIdc, Position lumaPos, const int setNum );
#endif
  // coding (quad)tree (clause 7.3.8.4)
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
  void        coding_tree               (CodingStructure&        cs, Partitioner&      pm, CUCtx& cuCtx, int(&qps)[2], Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
#else
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, int (&qps)[2], Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
#endif
  void        split_cu_mode             ( const PartSplit               split,    const CodingStructure& cs,     Partitioner& pm, const CodingUnit* cu
#if JVET_AI0087_BTCUS_RESTRICTION
    , bool btUpdateInfo
#endif
  );
#else
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
  void        split_cu_mode             ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm
#if JVET_AI0087_BTCUS_RESTRICTION
    , bool btUpdateInfo
#endif
  );
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  void        mode_constraint           ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm,    const ModeType modeType );
#endif
  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void        cu_skip_flag              ( const CodingUnit&             cu , Partitioner& partitioner);
#else
  void        cu_skip_flag              ( const CodingUnit&             cu );
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void        pred_mode                 ( const CodingUnit&             cu,       Partitioner&      pm );
#else
  void        pred_mode                 ( const CodingUnit&             cu );
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void        separate_tree_cu_flag     ( const CodingUnit&             cu,       Partitioner&      pm );
#endif
  void        bdpcm_mode                ( const CodingUnit&             cu,       const ComponentID compID );

  void        cu_pred_data              ( const CodingUnit&             cu );
#if ENABLE_OBMC
  void        obmc_flag                 (const CodingUnit&              cu);
#endif
  void        cu_bcw_flag               ( const CodingUnit&             cu );
  void        extend_ref_line           (const PredictionUnit&          pu );
  void        extend_ref_line           (const CodingUnit&              cu );
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const PredictionUnit&         pu );
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  void cu_pnn_flag(const CodingUnit& cu);
#endif
#if JVET_AM0074_INTRA_MERGE
  void        cu_derived_intra_flag     (const CodingUnit&                   cu);
#endif
#if ENABLE_DIMD
  void        cu_dimd_flag              ( const CodingUnit&             cu );
#endif
#if JVET_AH0076_OBIC
  void        cu_obic_flag              ( const CodingUnit&             cu );
#endif
#if JVET_AK0059_MDIP
  void        mdip_flag                 ( const CodingUnit&             cu );
#endif
#if JVET_W0123_TIMD_FUSION
  void        cu_timd_flag              ( const CodingUnit&             cu );
#if JVET_AJ0061_TIMD_MERGE
  void        cu_timd_merge_flag        ( const CodingUnit&             cu );
#endif
#endif
#if JVET_AB0155_SGPM
  void        sgpm_flag                 (const CodingUnit&              cu );
#endif
#if JVET_AB0157_TMRL
  void        cuTmrlFlag                ( const CodingUnit&             cu );
#if JVET_AJ0081_CHROMA_TMRL
  void        intraChromaTmrl           ( const PredictionUnit&         pu );
#endif
#endif
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
  void        intra_chroma_lmc_mode     ( const PredictionUnit&         pu );
  void        intra_chroma_pred_mode    ( const PredictionUnit&         pu );
#if JVET_AC0119_LM_CHROMA_FUSION
  void        intraChromaFusionMode     ( const PredictionUnit&         pu );
#endif
#if JVET_AA0057_CCCM
  void        cccmFlag                  ( const PredictionUnit&         pu );
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void        decoderDerivedCcpModes    ( const PredictionUnit&         pu );
#endif
#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void        nonLocalCCPIndex          ( const PredictionUnit&         pu );
#endif
#if JVET_AD0120_LBCCP
  void        ccInsideFilterFlag(const PredictionUnit &pu);
#endif

  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  void        inter_ccp_merge_root_cbf_zero(const CodingUnit&           cu);
#endif
  void        adaptive_color_transform(const CodingUnit&             cu);
  void        sbt_mode                  ( const CodingUnit&             cu );
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );
#if JVET_V0130_INTRA_TMP
  void        tmp_flag                  ( const CodingUnit& cu );
#endif
#if JVET_X0049_ADAPT_DMVR
  void        bm_merge_flag             ( const PredictionUnit&         pu);
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  void        affBmFlag                 (const PredictionUnit&         pu);
#endif
  void        mip_flag                  ( const CodingUnit&             cu );
  void        mip_pred_modes            ( const CodingUnit&             cu );
  void        mip_pred_mode             ( const PredictionUnit&         pu );
  void        cu_palette_info           ( const CodingUnit&             cu,       ComponentID       compBegin,     uint32_t numComp,          CUCtx&       cuCtx);
  void        cuPaletteSubblockInfo     ( const CodingUnit&             cu,       ComponentID       compBegin,     uint32_t numComp,          int subSetId,               uint32_t& prevRunPos,        unsigned& prevRunType );
  Pel         writePLTIndex             ( const CodingUnit&             cu,       uint32_t          idx,           PelBuf&  paletteIdx,       PLTtypeBuf&  paletteRunType, int         maxSymbol,   ComponentID compBegin );
#if JVET_AG0058_EIP
  void        cu_eip_flag              ( const CodingUnit&             cu );
#endif
  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const PredictionUnit&         pu );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  void        mvsd_data                 ( const PredictionUnit&         pu );
#endif
  void        merge_flag                ( const PredictionUnit&         pu );
  void        merge_data                ( const PredictionUnit&         pu );
  void        affine_flag               ( const CodingUnit&             cu );
  void        subblock_merge_flag       ( const CodingUnit&             cu );
  void        merge_idx                 ( const PredictionUnit&         pu );
  void        mmvd_merge_idx(const PredictionUnit&         pu);
#if AFFINE_MMVD
  void        affine_mmvd_data          ( const PredictionUnit&         pu );
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  void        ibcBiPredictionFlag     ( const PredictionUnit&         pu );
  void        ibcMergeIdx1            ( const PredictionUnit&         pu );
#endif
#if JVET_AA0061_IBC_MBVD
  void        ibcMbvdData             ( const PredictionUnit&         pu );
#endif
#if JVET_AC0112_IBC_CIIP
  void        ibcCiipFlag               ( const PredictionUnit&         pu );
  void        ibcCiipIntraIdx           ( const PredictionUnit&         pu );
#endif
#if JVET_AC0112_IBC_GPM
  void        ibcGpmFlag                ( const PredictionUnit&         pu );
  void        ibcGpmMergeIdx            ( const PredictionUnit&         pu );
  void        ibcGpmAdaptBlendIdx       ( const int idx );
#endif
#if JVET_AC0112_IBC_LIC
  void        cuIbcLicFlag              (const CodingUnit& cu );
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  void        tm_merge_flag             ( const PredictionUnit&         pu);
#endif
#if JVET_W0097_GPM_MMVD_TM
  void        geo_mmvd_idx(const PredictionUnit&         pu, RefPicList eRefPicList);
  void        geo_merge_idx(const PredictionUnit&         pu);
  void        geo_merge_idx1(const PredictionUnit&         pu);

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  uint64_t    geo_blend_est(const TempCtx& ctxStart, const int flag);
#endif
  uint64_t    geo_mode_est(const TempCtx& ctxStart, const int geoMode
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                         , const uint8_t altCodeIdx = 0
#endif
  );
  uint64_t    geo_mergeIdx_est(const TempCtx& ctxStart, const int candIdx, const int maxNumGeoCand
#if JVET_AG0164_AFFINE_GPM
    , int isAffine = 0
#endif
  );
#if JVET_Y0065_GPM_INTRA
  uint64_t    geo_intraFlag_est         ( const TempCtx& ctxStart, const int flag);
#if JVET_AI0082_GPM_WITH_INTER_IBC
  uint64_t    geo_intraIdx_est          ( const int intraIdx, const bool isGeoIbc);
#else
  uint64_t    geo_intraIdx_est          ( const int intraIdx);
#endif
#endif
  uint64_t    geo_mmvdFlag_est(const TempCtx& ctxStart, const int flag);
  uint64_t    geo_mmvdIdx_est(const TempCtx& ctxStart, const int mmvdIdx, const bool extMMVD);
#if TM_MRG
  uint64_t    geo_tmFlag_est(const TempCtx& ctxStart, const int flag);
#endif
#endif
#if JVET_AG0164_AFFINE_GPM
  uint64_t    geo_affFlag_est(const TempCtx& ctxStart, const int flag, int ctxOffset);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
  uint64_t    geoBldFlagEst             (const PredictionUnit& pu, const TempCtx& ctxStart, const int flag);
  void        geoAdaptiveBlendingIdx    (const PredictionUnit& pu, const int idx);
#else
  uint64_t    geoBldFlagEst             (const TempCtx& ctxStart, const int flag);
  void        geoAdaptiveBlendingIdx    (const int idx);
#endif
#endif
#if JVET_AL0134_SGPM_INTER
  void        sgpmInterFlag             (const PredictionUnit &pu);
  uint64_t    sgpmInterFlagEst          (const TempCtx &ctxStart, const int flag);
  uint64_t    sgpmInterTmFlagEst        (const TempCtx &ctxStart, const int flag);
  uint64_t    sgpmInterIdxEst           (const TempCtx &ctxStart, const int idx
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  , const bool& sgpmIntraFlag = false, const bool& bLd = false
#endif 
  );
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA 
  uint64_t    sgpmIntraFlagEst(const TempCtx& ctxStart, const int flag);
  uint64_t    sgpmAffineFlagEst         ( const TempCtx &ctxStart, const int flag, const int ctxOffset = -1);
  uint64_t    sgpmAffineIdxEst          ( const TempCtx &ctxStart, const int idx, const int maxNum);
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void        geoModeIdx                ( const PredictionUnit&         pu);
  void        geoModeIdx                ( const uint8_t geoMode, const uint8_t altCodeIdx = 0);
#endif
  void        imv_mode                  ( const CodingUnit&             cu );
  void        affine_amvr_mode          ( const CodingUnit&             cu );
  void        inter_pred_idc            ( const PredictionUnit&         pu );
#if JVET_Z0054_BLK_REF_PIC_REORDER && JVET_AD0213_LIC_IMP
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList eRefList, bool forceRefIdx = false);
#else
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList eRefList );
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  void        refIdxLC                  ( const PredictionUnit&         pu );
  void        refPairIdx                ( const PredictionUnit&         pu );
#endif
  void        mvp_flag                  ( const PredictionUnit&         pu,       RefPicList eRefList );

  void        Ciip_flag                 ( const PredictionUnit&         pu );
#if JVET_AG0135_AFFINE_CIIP
  void        ciipAffineFlag            ( const PredictionUnit&         pu);
#endif
  void        smvd_mode                 ( const PredictionUnit&         pu );
#if JVET_AG0098_AMVP_WITH_SBTMVP
  void        amvpSbTmvpFlag            ( const PredictionUnit&         pu );
  void        amvpSbTmvpMvdCoding       ( const PredictionUnit&         pu );
#endif

#if MULTI_HYP_PRED
  void        ref_idx_mh                (const int numRef, const int refIdx);
  void        mh_pred_data              (const PredictionUnit&         pu);
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  void        amvpMerge_mode            ( const PredictionUnit&         pu );
#endif
#if JVET_Z0050_CCLM_SLOPE
  void        cclmDelta                 ( const PredictionUnit&         pu, int8_t delta);
  void        cclmDeltaSlope            ( const PredictionUnit&         pu );
#endif
#if JVET_AA0126_GLM
  void        glmIdc                    ( const PredictionUnit&         pu );
#endif

#if JVET_AL0153_ALF_CCCM
  void        lfCccm( const CodingStructure& cs, const uint32_t ctuRsAddr );
#endif
  // transform tree (clause 7.3.8.8)
  void        transform_tree(const CodingStructure&        cs, Partitioner&      pm, CUCtx& cuCtx, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1
#if JVET_AE0102_LFNST_CTX
    , const bool codeTuCoeff = false
#endif  
  );

  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbf = false, const bool useISP = false );

  // mvd coding (clause 7.3.8.9)
#if JVET_AD0140_MVD_PREDICTION
  void        mvd_coding            ( const Mv& rMvd, int8_t imv, const MvdSuffixInfo* const pSi = nullptr
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                        , bool codeSign = true
#endif
                                    );
  void        mvdCodingRemainder    ( const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv );
  unsigned    xWriteMvdPrefix       ( unsigned uiSymbol, int param);
  void        xWriteMvdContextSuffix( unsigned uiSymbol, int param, int param_updated, int numSkipMSB );
#elif JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  void        mvd_coding                ( const Mv &rMvd, int8_t imv, bool ctxCoding = true 
#if JVET_AA0070_RRIBC
                                        , const int& rribcFlipType = 0
#endif
                                        );
#else
  void        mvd_coding                ( const Mv &rMvd, int8_t imv, const int &rribcFlipType = 0
#if JVET_AA0070_RRIBC
                                        , const int& rribcFlipType = 0
#endif
                                        );
#endif

#if JVET_AA0070_RRIBC
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  void bvdCoding(const Mv &rMvd, const bool useBvdPred = true, const bool useBvpCluster = true, int bvOneZeroComp = 0,
                 int bvZeroCompDir = 0, const int &rribcFlipType = 0 );
#else                                     
  void        bvdCoding                 ( const Mv& rMvd, const bool useBvdPred = true, const int& rribcFlipType = 0 );
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  void bvdCoding(const Mv &rMvd, const bool useBvpCluster = true, int bvOneZeroComp = 0, int bvZeroCompDir = 0,
                 const int &rribcFlipType = 0 );
#else                                     
  void        bvdCoding                 ( const Mv& rMvd, const int& rribcFlipType = 0 );
#endif
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  void        bvdCodingRemainder        (const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv);
  unsigned    xWriteBvdContextPrefix    (unsigned uiSymbol, unsigned ctxT, int offset, int param);
  void        xWriteBvdContextSuffix    (unsigned uiSymbol, int param, int paramUpdated, int numSkipMSB);
#endif
  void        xWriteBvdContext          (unsigned uiSymbol, unsigned ctxT, int offset, int param);
#endif
#else
  void        mvd_coding                ( const Mv &rMvd, int8_t imv 
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    , bool codeSign = true
#endif
  );

#if JVET_AD0140_MVD_PREDICTION
  void        mvdCodingRemainder        ( const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv, const bool isAffine
  );
  unsigned    xWriteMvdPrefix           ( unsigned uiSymbol, int param );
  void        xWriteMvdContextSuffix    ( unsigned uiSymbol, int param, int param_updated, int numSkipMSB = 0 );
#endif
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  void bvdCoding(const Mv &rMvd, const bool useBvdPred = true, const bool useBvpCluster = true, int bvOneZeroComp = 0,
                 int bvZeroCompDir = 0);
#else                                     
  void        bvdCoding                 ( const Mv& rMvd, const bool useBvdPred = true );
#endif                                    
  void        bvdCodingRemainder        ( const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv );
                                          
  unsigned    xWriteBvdContextPrefix    ( unsigned uiSymbol, unsigned ctxT, int offset, int param );
  void        xWriteBvdContextSuffix    ( unsigned uiSymbol, int param, int paramUpdated, int numSkipMSB = 0 );
#else
  void        bvdCoding                 ( const Mv &rMvd);
#endif
  void        xWriteBvdContext(unsigned uiSymbol, unsigned ctxT, int offset, int param);
#endif
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  void mvsdIdxFunc(const PredictionUnit &pu, RefPicList eRefList);
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  void mvsdAffineIdxFunc(const PredictionUnit &pu, RefPicList eRefList);
#endif
  // transform unit (clause 7.3.8.10)
  void        transform_unit( const TransformUnit&          tu, CUCtx&            cuCtx, Partitioner& pm, const int subTuCounter = -1
#if JVET_AE0102_LFNST_CTX
    , const bool codeTuCoeff = false
#endif
  );


  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const int8_t qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID, CUCtx* cuCtx = nullptr
#if JVET_AE0102_LFNST_CTX  
    , const bool codeTuCoeff = false
#endif      
  );
  void        ts_flag                   ( const TransformUnit&          tu,       ComponentID       compID );
  void        mts_idx                   ( const CodingUnit&             cu,       CUCtx*            cuCtx  );
  void        residual_lfnst_mode       ( const CodingUnit&             cu,       CUCtx&            cuCtx );
  void        isp_mode                  ( const CodingUnit&             cu );
  void        last_sig_coeff            ( CoeffCodingContext&           cctx,     const TransformUnit& tu, ComponentID       compID );
#if TCQ_8STATES
#if JVET_AE0102_LFNST_CTX 
  void        residual_coding_subblock(CoeffCodingContext& cctx, const TCoeff* coeff, const uint64_t stateTransTable, int& state, int lfnstIdx = -1);
#else
  void        residual_coding_subblock(CoeffCodingContext& cctx, const TCoeff* coeff, const uint64_t stateTransTable, int& state);
#endif
#else
  void        residual_coding_subblock(CoeffCodingContext &cctx, const TCoeff *coeff, const int stateTransTable, int &state);
#endif
#if SIGN_PREDICTION
  void        codePredictedSigns ( TransformUnit &tu, ComponentID compID);
#endif
  void        residual_codingTS         ( const TransformUnit&          tu,       ComponentID       compID );
  void        residual_coding_subblockTS(CoeffCodingContext &cctx, const TCoeff *coeff);
  void        joint_cb_cr               ( const TransformUnit&          tu,       const int cbfMask );

  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam);
  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam);
  void        codeAlfCtuEnableFlag      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam );
  void        codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma);

  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ChannelType channel, AlfParam* alfParam);
  void        codeAlfCtuAlternatives     ( CodingStructure& cs, ComponentID compID, AlfParam* alfParam);
  void        codeAlfCtuAlternative      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam = NULL 
#if ALF_IMPROVEMENT
    , int numAltLuma = -1
#endif
  );
  void codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID, const int curIdx,
                                 const uint8_t *filterControlIdc, Position lumaPos, const int filterCount);
  #if NN_LF_UNIFIED
  void writeNnlfUnifiedParameters(const CodingStructure &cs);
#if JVET_AK0093_NON_NORMATIVE_TDO
  void computeCostNnlfUnifiedParameters(const CodingStructure& cs, const std::vector<int>& bsFilteringBlocks);
#endif
  #endif
#if JVET_AK0065_TALF
  void codeTAlfFilterControlIdc(TAlfCtbParam curControl, CodingStructure &cs, const ComponentID compID,
                                const int curIdx, const TAlfCtbParam *filterControlIdc, Position lumaPos,
                                const int filterCount, const int numSets, const bool newFilters);
#endif

#if INTER_LIC
  void        cu_lic_flag               ( const CodingUnit& cu );
#endif

#if JVET_AA0070_RRIBC
  void        rribcData                ( const CodingUnit &cu);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  void        bvOneZeroComp            ( const CodingUnit &cu );
#endif
#if JVET_AE0059_INTER_CCCM
  void        interCccm                ( const TransformUnit& tu );
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  void        interCcpMerge            ( const TransformUnit& tu );
#endif
#if JVET_AL0181_ASBT
  void        interAsbt                ( const TransformUnit& tu );
#endif
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  CABACDataStore*         m_CABACDataStore;
#endif


private:
  void        unary_max_symbol          ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob          ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob         ( unsigned symbol, unsigned count );
  void        code_unary_fixed          ( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed );

  // statistic
  unsigned    get_num_written_bits()    { return m_BinEncoder.getNumWrittenBits(); }

  void  xWriteTruncBinCode(uint32_t uiSymbol, uint32_t uiMaxSymbol);
  void        codeScanRotationModeFlag   ( const CodingUnit& cu,     ComponentID compBegin);
  void        xEncodePLTPredIndicator    ( const CodingUnit& cu,     uint32_t    maxPltSize, ComponentID compBegin);
#if JVET_AL0181_ASBT  
  void        writeTransposeCoeff        ( const TransformUnit& tu,  ComponentID compID );
#endif  
private:
  BinEncIf&         m_BinEncoder;
  OutputBitstream*  m_Bitstream;
  Ctx               m_TestCtx;
  EncCu*            m_EncCu;
  ScanElement*      m_scanOrder;
#if JVET_AL0181_ASBT    
  TCoeff            m_coeffTmp[MAX_NUM_COMPONENT][MAX_CU_SIZE * MAX_CU_SIZE];
  SIGN_PRED_TYPE    m_signPredTypeTmp[MAX_NUM_COMPONENT][MAX_CU_SIZE * MAX_CU_SIZE];
  unsigned          m_signScanIdxTmp[MAX_NUM_COMPONENT][MAX_CU_SIZE * MAX_CU_SIZE];
#endif  
};



class CABACEncoder
{
public:
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  CABACEncoder()
    : m_CABACWriterStd( m_BinEncoderStd, nullptr )
    , m_CABACEstimatorStd( m_BitEstimatorStd, nullptr )
    , m_CABACWriter{ &m_CABACWriterStd }
    , m_CABACEstimator{ &m_CABACEstimatorStd }
    , m_CABACDataStore( nullptr )
  {
    m_CABACDataStore = new CABACDataStore;

    m_CABACWriterStd.m_CABACDataStore = m_CABACDataStore;
    m_CABACEstimatorStd.m_CABACDataStore = m_CABACDataStore;

    for( int i = 0; i < BPM_NUM - 1; i++ )
    {
      m_CABACWriter[i]->m_CABACDataStore = m_CABACDataStore;
      m_CABACEstimator[i]->m_CABACDataStore = m_CABACDataStore;
    }

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    m_BinEncoderStd.initBufferer  (CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
    m_BitEstimatorStd.initBufferer(CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
#endif
  }

  virtual ~CABACEncoder()
  {
    if( m_CABACDataStore )
    {
      delete m_CABACDataStore;
    }
  }
#else
  CABACEncoder()
    : m_CABACWriterStd      ( m_BinEncoderStd )
    , m_CABACEstimatorStd   ( m_BitEstimatorStd )
    , m_CABACWriter         { &m_CABACWriterStd,   }
    , m_CABACEstimator      { &m_CABACEstimatorStd }
  {
#if JVET_AG0117_CABAC_SPATIAL_TUNING
    m_BinEncoderStd.initBufferer  (CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
    m_BitEstimatorStd.initBufferer(CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
#endif
  }
#endif

  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [0]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[0]; }
private:
  BinEncoder_Std      m_BinEncoderStd;
  BitEstimator_Std    m_BitEstimatorStd;
  CABACWriter         m_CABACWriterStd;
  CABACWriter         m_CABACEstimatorStd;
  CABACWriter*        m_CABACWriter   [BPM_NUM-1];
  CABACWriter*        m_CABACEstimator[BPM_NUM-1];

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  CABACDataStore*         m_CABACDataStore;
#endif
};

//! \}

#endif //__CABACWRITER__
