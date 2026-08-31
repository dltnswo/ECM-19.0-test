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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

#include "ChromaFormat.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

void CompArea::xRecalcLumaToChroma()
{
  const uint32_t csx = getComponentScaleX(compID, chromaFormat);
  const uint32_t csy = getComponentScaleY(compID, chromaFormat);

  x      >>= csx;
  y      >>= csy;
  width  >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if( isLuma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos() : chromaPos();
}

Position CompArea::chanPos( const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos() : chromaPos();
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area) : chromaFormat(_chromaFormat), blocks(getNumberValidComponents(_chromaFormat))
{
  const uint32_t numCh = getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,      CompArea &&blkCb,      CompArea &&blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

bool UnitArea::contains(const UnitArea& other) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( toChannelType( blk.compID ) == chType && blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
void UnitArea::resizeTo( const UnitArea& unitArea )
{
  for( uint32_t i = 0; i < blocks.size(); i++ )
  {
    blocks[i].resizeTo( unitArea.blocks[i] );
  }
}
#endif

void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (blk.compID == compID)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType(blk.compID) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea &unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }

CodingUnit& CodingUnit::operator=( const CodingUnit& other )
{
  slice             = other.slice;
  predMode          = other.predMode;
  qtDepth           = other.qtDepth;
  depth             = other.depth;
  btDepth           = other.btDepth;
  mtDepth           = other.mtDepth;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  mtImplicitDepth   = other.mtImplicitDepth;
#endif
  splitSeries       = other.splitSeries;
  skip              = other.skip;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  interCcpMergeZeroRootCbfIdc = other.interCcpMergeZeroRootCbfIdc;
#endif
  mmvdSkip = other.mmvdSkip;
  affine            = other.affine;
  affineType        = other.affineType;
  colorTransform = other.colorTransform;
  geoFlag           = other.geoFlag;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  geoBlendFlag      = other.geoBlendFlag;
  blendModel.copy( other.blendModel );
#endif
  bdpcmMode         = other.bdpcmMode;
  bdpcmModeChroma   = other.bdpcmModeChroma;
  qp                = other.qp;
  chromaQpAdj       = other.chromaQpAdj;
  rootCbf           = other.rootCbf;
  sbtInfo           = other.sbtInfo;
  mtsFlag           = other.mtsFlag;
#if JVET_AG0061_INTER_LFNST_NSPT
  lfnstFlag         = other.lfnstFlag;
#if JVET_AI0050_INTER_MTSS
  lfnstIntra        = other.lfnstIntra;
#endif
#endif
  lfnstIdx          = other.lfnstIdx;
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  indicesRepresentationPnn = other.indicesRepresentationPnn;
  lfnstSecFlag = other.lfnstSecFlag;
#endif
  tileIdx           = other.tileIdx;
#if JVET_AC0105_DIRECTIONAL_PLANAR
  plIdx = other.plIdx;
#endif
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION 
  candModeListForTransform = other.candModeListForTransform;
#endif 
#if JVET_AK0217_INTRA_MTSS 
  candModeListForTransformMtss = other.candModeListForTransformMtss;
  candCostListForTransformMtss = other.candCostListForTransformMtss;
#endif 
#if JVET_AM0074_INTRA_MERGE
  intraMergeMode = other.intraMergeMode;
  allowIntraMergeMode = other.allowIntraMergeMode;
#if JVET_AB0157_INTRA_FUSION
  for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
  {
    dimdLocDepMerge[i] = other.dimdLocDepMerge[i];
    dimdBlendModeMerge[i] = other.dimdBlendModeMerge[i];
  }
  for (int i = 0; i < DIMD_FUSION_NUM; i++)
  {
    dimdRelWeightMerge[i] = other.dimdRelWeightMerge[i];
  }
#else
  for (int i = 0; i < 2; i++)
  {
    dimdLocDepMerge[i] = other.dimdLocDepMerge[i];
    dimdBlendModeMerge[i] = other.dimdBlendModeMerge[i];
  }
  for (int i = 0; i < 3; i++)
  {
    dimdRelWeightMerge[i] = other.dimdRelWeightMerge[i];
  }
#endif
  dimdModeMerge = other.dimdModeMerge;
  isBvDimdExtAvail = other.isBvDimdExtAvail;
  for (int i = 0; i < DIMD_FUSION_NUM; i++)
  {
    isBvDimdExt[i] = other.isBvDimdExt[i];
    bvDimdExt[i]   = other.bvDimdExt[i];
  }
#endif
#if ENABLE_DIMD
#if JVET_AH0076_OBIC
  obicFlag = other.obicFlag;
  obicIsBlended = other.obicIsBlended;
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    obicMode[i] = other.obicMode[i];
    obicFusionWeight[i] = other.obicFusionWeight[i];
#if JVET_AK0056_WEIGHTED_OBIC
    obicLocDep[i] = other.obicLocDep[i];
#endif
  }
#endif
#if JVET_AL0108_BVG_DIMD
  bvgDimdFlag      = other.bvgDimdFlag;
  for (int i = 0; i < BVG_DIMD_INTRA_NUM; i++)
  {
    bvgDimdMode[i] = other.bvgDimdMode[i];
    bvgDimdFusionWeight[i] = other.bvgDimdFusionWeight[i];
  }
  bvDimdNum = other.bvDimdNum;
  for (int i = 0; i < BVG_DIMD_BV_LIST_SIZE; i++)
  {
    bvDimdList[i] = other.bvDimdList[i];
    bvDimdWeight[i] = other.bvDimdWeight[i];
  }
#endif
#if JVET_AG0146_DIMD_ITMP_IBC
  isBvDimd = other.isBvDimd;
  bvDimd = other.bvDimd;
#endif
  dimd = other.dimd;
  dimdBlending = other.dimdBlending;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
  for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
  {
    dimdLocDep[i] = other.dimdLocDep[i];
  }
#else
  dimdLocDep[0] = other.dimdLocDep[0];
  dimdLocDep[1] = other.dimdLocDep[1];
#endif
#endif
  dimdMode = other.dimdMode;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  dimdChromaMode = other.dimdChromaMode;
#if JVET_AC0094_REF_SAMPLES_OPT
  dimdChromaModeSecond = other.dimdChromaModeSecond;
#endif
#endif
#if JVET_AK0064_CCP_LFNST_NSPT
  for (int i = 0; i < 4; i++)
  {
    ccpChromaDimdMode[i] = other.ccpChromaDimdMode[i];
  }
#endif
#if JVET_AB0157_INTRA_FUSION
  for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
  {
    dimdBlendMode[i] = other.dimdBlendMode[i];
  }

  for( int i = 0; i < DIMD_FUSION_NUM; i++ )
  {
    dimdRelWeight[i] = other.dimdRelWeight[i];
  }
#else
  for( int i = 0; i < 2; i++ )
  {
    dimdBlendMode[i] = other.dimdBlendMode[i];
  }

  for( int i = 0; i < 3; i++ )
  {
    dimdRelWeight[i] = other.dimdRelWeight[i];
  }
#endif
#endif
#if JVET_AH0136_CHROMA_REORDERING
  for (uint32_t i = 0; i < 7; i++)
  {
    chromaList[i] = other.chromaList[i];
  }
  for (int i = 0; i < 5; i++)
  {
    dimdBlendModeChroma[i] = other.dimdBlendModeChroma[i];
  }
  for (int i = 0; i < 10; i++)
  {
    mvs[i] = other.mvs[i];
    bvs[i] = other.bvs[i];
    rribcTypes[i] = other.rribcTypes[i];
  }
  mvsNum = other.mvsNum;
#endif
#if TMP_FAST_ENC
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  tmpXdisp = other.tmpXdisp;
  tmpYdisp = other.tmpYdisp;
#endif
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int i = 0; i < MTMP_NUM; i++)
  {
    tmpXYdisp[i]      = other.tmpXYdisp[i];
    tmpXYdispUseMR[i] = other.tmpXYdispUseMR[i];
  }
#endif
  tmpIdx        = other.tmpIdx;
  tmpFusionFlag = other.tmpFusionFlag;
  tmpFlmFlag    = other.tmpFlmFlag;
#if JVET_AG0136_INTRA_TMP_LIC
  tmpLicFlag    = other.tmpLicFlag;
#endif
  tmpIsSubPel  = other.tmpIsSubPel;
  tmpSubPelIdx = other.tmpSubPelIdx;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  tmpFracIdx    = other.tmpFracIdx;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  tempType      = other.tempType;
#endif
#endif
#endif
#if JVET_AK0059_MDIP
  mdip              = other.mdip;
  mdipMode          = other.mdipMode;
  isModeExcluded    = other.isModeExcluded;
  std::memcpy( excludingMode, other.excludingMode, sizeof( excludingMode ) );
#endif
#if JVET_W0123_TIMD_FUSION
  timd              = other.timd;
  timdMode          = other.timdMode;
  timdModeSecondary = other.timdModeSecondary;
#if JVET_AC0094_REF_SAMPLES_OPT
  timdModeCheckWA          = other.timdModeCheckWA;
  timdModeSecondaryCheckWA = other.timdModeSecondaryCheckWA;
#endif
  timdIsBlended     = other.timdIsBlended;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  timdModeNonAng    = other.timdModeNonAng;
  for( int i = 0; i < TIMD_FUSION_NUM; i++ )
  {
    timdFusionWeight[i] = other.timdFusionWeight[i];
    timdLocDep[i]       = other.timdLocDep[i];
  }
#else
  timdFusionWeight[0] = other.timdFusionWeight[0];
  timdFusionWeight[1] = other.timdFusionWeight[1];
#endif
#if JVET_AJ0146_TIMDSAD
  timdSad              = other.timdSad;
  timdModeSad          = other.timdModeSad;
  timdModeSecondarySad = other.timdModeSecondarySad;
#if JVET_AC0094_REF_SAMPLES_OPT
  timdModeCheckWASad          = other.timdModeCheckWASad;
  timdModeSecondaryCheckWASad = other.timdModeSecondaryCheckWASad;
#endif
  timdIsBlendedSad     = other.timdIsBlendedSad;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  timdModeNonAngSad    = other.timdModeNonAngSad  ;
  for( int i = 0; i < TIMD_FUSION_NUM; i++ )
  {
    timdFusionWeightSad[i] = other.timdFusionWeightSad[i];
    timdLocDepSad[i]       = other.timdLocDepSad[i];
  }
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int i = 0; i < TIMD_FUSION_NUM; i++)
  {
    timdBv[i]      = other.timdBv[i];
    isBvTimd[i]    = other.isBvTimd[i];
    timdBvSad[i]   = other.timdBvSad[i];
    isBvTimdSad[i] = other.isBvTimdSad[i];
  }
#endif
#else
  timdFusionWeightSad[0] = other.timdFusionWeightSad[0];
  timdFusionWeightSad[1] = other.timdFusionWeightSad[1];
#endif
#endif
#if JVET_AJ0061_TIMD_MERGE
  timdMrg = other.timdMrg;
  for (int i = 0; i <= NUM_TIMD_MERGE_MODES; i++)
  {
    timdmTrType[i][0] = other.timdmTrType[i][0];
    timdmTrType[i][1] = other.timdmTrType[i][1];
  }
  for (int i = 0; i < NUM_TIMD_MERGE_MODES; i++)
  {
    timdMrgIsBlended[i] = other.timdMrgIsBlended[i];
    for (int j = 0; j < TIMD_FUSION_NUM; j++)
    {
      timdMrgList[i][j] = other.timdMrgList[i][j];
      timdMrgFusionWeight[i][j] = other.timdMrgFusionWeight[i][j];
      timdMrgModeCheckWA[i][j] = other.timdMrgModeCheckWA[i][j];
      timdMrgLocDep[i][j] = other.timdMrgLocDep[i][j];
    }
  }
  timdMrgCand = other.timdMrgCand;
#endif
#endif
#if JVET_AB0155_SGPM
  timdHor      = other.timdHor;
  timdVer      = other.timdVer;
#if JVET_AG0152_SGPM_ITMP_IBC
  sgpmBv0       = other.sgpmBv0;
  sgpmBv1       = other.sgpmBv1;
#endif
  sgpm         = other.sgpm;
  sgpmIdx      = other.sgpmIdx;
  sgpmSplitDir = other.sgpmSplitDir;
  sgpmMode0    = other.sgpmMode0;
  sgpmMode1    = other.sgpmMode1;
#endif
#if JVET_AG0058_EIP
  eipFlag = other.eipFlag;
  eipMerge = other.eipMerge;
  eipModel = other.eipModel;
#if JVET_AJ0082_MM_EIP
  eipMmFlag = other.eipMmFlag;
#endif
#if JVET_AL0106_BV_EIP
  bvEip = other.bvEip;
#endif
#endif
#if ENABLE_OBMC
  obmcFlag          = other.obmcFlag;
  isobmcMC          = other.isobmcMC;
#endif
  imv               = other.imv;
  imvNumCand        = other.imvNumCand;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  isSST                          = other.isSST;
  separateTree                   = other.separateTree;
  intraRegionRootDepth           = other.intraRegionRootDepth;  
  intraRegionRootQtDepth         = other.intraRegionRootQtDepth;
  intraRegionRootBtDepth         = other.intraRegionRootBtDepth;
  intraRegionRootMtDepth         = other.intraRegionRootMtDepth;
  intraRegionRootImplicitBtDepth = other.intraRegionRootImplicitBtDepth;
#endif

  bcwIdx            = other.bcwIdx;
  for (int i = 0; i<2; i++)
    refIdxBi[i] = other.refIdxBi[i];

  smvdMode        = other.smvdMode;
  ispMode           = other.ispMode;
  mipFlag           = other.mipFlag;
#if JVET_AB0067_MIP_DIMD_LFNST
  mipDimdMode       = other.mipDimdMode;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  sgpmDimdMode = other.sgpmDimdMode;
#endif
#if JVET_V0130_INTRA_TMP
  tmpFlag           = other.tmpFlag;
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
  intraTmpDimdMode = other.intraTmpDimdMode;
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int iTimd = 0; iTimd < NumTimdMode; iTimd++)
  {
    timdDimdMode[iTimd]       = other.timdDimdMode[iTimd];
    timdSecondDimdMode[iTimd] = other.timdSecondDimdMode[iTimd];
  }
  timdSadDimdMode       = other.timdSadDimdMode;
  timdSecondSadDimdMode = other.timdSecondSadDimdMode;
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  dimdDimdMode       = other.dimdDimdMode;
  dimdSecondDimdMode = other.dimdSecondDimdMode;
  obicDimdMode       = other.obicDimdMode;
  obicSecondDimdMode = other.obicSecondDimdMode;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  dimdDerivedIntraDir = other.dimdDerivedIntraDir;
#if JVET_AI0050_INTER_MTSS
  dimdDerivedIntraDir2nd = other.dimdDerivedIntraDir2nd;
#endif
#endif
#endif
#if JVET_AG0276_NLIC
  altLMFlag         = other.altLMFlag;
  altLMParaUnit     = other.altLMParaUnit;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  altLMBRParaUnit   = other.altLMBRParaUnit;
#endif
#if ENABLE_OBMC
  secAltLMParaUnit  = other.secAltLMParaUnit;
#endif
#endif
#if INTER_LIC
  licFlag           = other.licFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  licInheritPara    = other.licInheritPara;
#endif
#if JVET_AD0213_LIC_IMP
  for (int i = 0; i < 2; i++)
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      licScale[i][comp] = other.licScale[i][comp];
      licOffset[i][comp] = other.licOffset[i][comp];
    }
  }
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  licDelta         = other.licDelta;
#endif
#endif
#if JVET_AC0112_IBC_LIC
  ibcLicFlag = other.ibcLicFlag;
#if JVET_AE0078_IBC_LIC_EXTENSION
  ibcLicIdx = other.ibcLicIdx;
#if JVET_AM0157_SGPM_BV_LIC
  for (int i = 0; i < 2; i++)
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      ibcLicMean[i][comp] = other.ibcLicMean[i][comp];
    }
  }
  for (int i = 0; i < 2; i++)
  {
    for (int modeIdx = 0; modeIdx < 2; modeIdx++)
    {
      for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ibcLicScale[i][modeIdx][comp] = other.ibcLicScale[i][modeIdx][comp];
        ibcLicShift[i][modeIdx][comp] = other.ibcLicShift[i][modeIdx][comp];
        ibcLicOffset[i][modeIdx][comp] = other.ibcLicOffset[i][modeIdx][comp];
      }
    }
  }
#endif
#endif
#endif
#if JVET_AE0159_FIBC
  ibcFilterFlag  = other.ibcFilterFlag;
  if (slice->getSPS()->getUseIbcFilter())
  {
    memcpy(ibcFilterParams, other.ibcFilterParams, FIBC_PARAMS * sizeof(int64_t));
  }
#endif
#if JVET_AA0070_RRIBC
  rribcFlipType = other.rribcFlipType;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bvOneZeroComp = other.bvOneZeroComp;
  bvZeroCompDir = other.bvZeroCompDir;
#endif
#if JVET_AB0157_TMRL
  tmrlFlag = other.tmrlFlag;
  tmrlListIdx = other.tmrlListIdx;
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  areAboveRightUnavail = other.areAboveRightUnavail;
  areBelowLeftUnavail  = other.areBelowLeftUnavail;
#endif

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = other.curPLTSize[idx];
    useEscape[idx]    = other.useEscape[idx];
    useRotation[idx]  = other.useRotation[idx];
    reusePLTSize[idx] = other.reusePLTSize[idx];
    lastPLTSize[idx]  = other.lastPLTSize[idx];
    if (slice->getSPS()->getPLTMode())
    {
      memcpy(reuseflag[idx], other.reuseflag[idx], MAXPLTPREDSIZE * sizeof(bool));
    }
  }

  if (slice->getSPS()->getPLTMode())
  {
    for (int idx = 0; idx < MAX_NUM_COMPONENT; idx++)
    {
      memcpy(curPLT[idx], other.curPLT[idx], MAXPLTSIZE * sizeof(Pel));
    }
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  treeType          = other.treeType;
  modeType          = other.modeType;
  modeTypeSeries    = other.modeTypeSeries;
#endif
  return *this;
}

void CodingUnit::initData()
{
  predMode          = NUMBER_OF_PREDICTION_MODES;
  qtDepth           = 0;
  depth             = 0;
  btDepth           = 0;
  mtDepth           = 0;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  mtImplicitDepth   = 0;
#endif
  splitSeries       = 0;
  skip              = false;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  interCcpMergeZeroRootCbfIdc = 0;
#endif
  mmvdSkip = false;
  affine            = false;
  affineType        = 0;
  colorTransform = false;
  geoFlag           = false;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  geoBlendFlag      = false;
#endif
  bdpcmMode         = 0;
  bdpcmModeChroma   = 0;
  qp                = 0;
  chromaQpAdj       = 0;
  rootCbf           = true;
  sbtInfo           = 0;
  mtsFlag           = 0;
#if JVET_AG0061_INTER_LFNST_NSPT
  lfnstFlag         = 0;
#if JVET_AI0050_INTER_MTSS
  lfnstIntra        = 0;
#endif
#endif
  lfnstIdx          = 0;
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  candModeListForTransform.resize(2);
  candModeListForTransform[0] = 0;
  candModeListForTransform[1] = 0;
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  for (auto it{indicesRepresentationPnn.begin()}; it != indicesRepresentationPnn.end(); it++)
  {
    std::fill(it->begin(), it->end(), MAX_INT);
  }
  lfnstSecFlag = false;
#endif
  tileIdx           = 0;
#if JVET_AC0105_DIRECTIONAL_PLANAR
  plIdx = 0;
#endif
#if JVET_AM0074_INTRA_MERGE
  intraMergeMode = 0;
  allowIntraMergeMode = 0;
#if JVET_AB0157_INTRA_FUSION
  for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
  {
    dimdLocDepMerge[i] = 0;
    dimdBlendModeMerge[i] = -1;
  }
  for (int i = 0; i < DIMD_FUSION_NUM; i++)
  {
    dimdRelWeightMerge[i] = -1;
  }
#else
  for (int i = 0; i < 2; i++)
  {
    dimdLocDepMerge[i] = 0;
    dimdBlendModeMerge[i] = -1;
  }
  for (int i = 0; i < 3; i++)
  {
    dimdRelWeightMerge[i] = -1;
  }
#endif
  dimdModeMerge = -1;
  isBvDimdExtAvail = false;
  for (int i = 0; i < DIMD_FUSION_NUM; i++)
  {
    isBvDimdExt[i] = false;
    bvDimdExt[i]   = Mv(0, 0);
  }
#endif
#if ENABLE_DIMD
#if JVET_AH0076_OBIC
  obicFlag = false;
  obicIsBlended = false;
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    obicMode[i] = -1;
    obicFusionWeight[i] = 0;
#if JVET_AK0056_WEIGHTED_OBIC
    obicLocDep[i] = 0;
#endif
  }
#endif
#if JVET_AL0108_BVG_DIMD
  bvgDimdFlag      = false;
  for (int i = 0; i < BVG_DIMD_INTRA_NUM; i++)
  {
    bvgDimdMode[i] = 0;
    bvgDimdFusionWeight[i] = 0;
  }
  bvDimdNum = 0;
  for (int i = 0; i < BVG_DIMD_BV_LIST_SIZE; i++)
  {
    bvDimdList[i] = Mv(0, 0);
    bvDimdWeight[i] = 0;
  }
#endif
#if JVET_AG0146_DIMD_ITMP_IBC
  isBvDimd  = 0;
  bvDimd    = Mv(0, 0);
#endif
  dimd = false;
  dimdBlending = false;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
  for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
  {
    dimdLocDep[i] = 0;
  }
#else
  dimdLocDep[0] = 0;
  dimdLocDep[1] = 0;
#endif
#endif
  dimdMode = -1;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  dimdChromaMode   = -1;
#if JVET_AC0094_REF_SAMPLES_OPT
  dimdChromaModeSecond = -1;
#endif
#endif
#if JVET_AK0064_CCP_LFNST_NSPT
  for (int i = 0; i < 4; i++)
  {
    ccpChromaDimdMode[i] = -1;
  }
#endif
#if JVET_AB0157_INTRA_FUSION
  for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
  {
    dimdBlendMode[i] = -1;
  }

  for( int i = 0; i < DIMD_FUSION_NUM; i++ )
  {
    dimdRelWeight[i] = -1;
  }
#else
  for( int i = 0; i < 2; i++ )
  {
    dimdBlendMode[i] = -1;
  }

  for( int i = 0; i < 3; i++ )
  {
    dimdRelWeight[i] = -1;
  }
#endif
#endif
#if JVET_AH0136_CHROMA_REORDERING
  for (uint32_t i = 0; i < 7; i++)
  {
    chromaList[i] = -1;
  }
  for (int i = 0; i < 5; i++)
  {
    dimdBlendModeChroma[i] = -1;
  }
  for (int i = 0; i < 10; i++)
  {
    mvs[i].setZero();
    bvs[i].setZero();
    rribcTypes[i] = 0;
  }
  mvsNum = 0;
#endif
#if TMP_FAST_ENC
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  tmpXdisp = 0;
  tmpYdisp = 0;
#endif
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int i = 0; i < MTMP_NUM; i++)
  {
    tmpXYdisp[i]      = Mv(0, 0);
    tmpXYdispUseMR[i] = Mv(0, 0);
  }
#endif
  tmpIdx        = 0;
  tmpFusionFlag = false;
  tmpFlmFlag    = false;
  tmpIsSubPel  = 0;
  tmpSubPelIdx = -1;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  tmpFracIdx    = -1;
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  tmpLicFlag   = false;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  tempType = UNDEF_TEMPLATE;
#endif
#endif
#endif
#if JVET_AK0059_MDIP
  mdip              = false;
  mdipMode          = -1;
  isModeExcluded    = true;
  for (int i=0; i < EXCLUDING_MODE_NUM; i++)
  {
    excludingMode[i] = -1;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  timd                     = false;
#if JVET_AC0094_REF_SAMPLES_OPT
  timdMode                 = INVALID_TIMD_IDX;
  timdModeSecondary        = INVALID_TIMD_IDX;
  timdModeCheckWA          = true;
  timdModeSecondaryCheckWA = true;
#else
  timdMode          = -1;
  timdModeSecondary = -1;
#endif
  timdIsBlended     = false;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  timdModeNonAng    = INVALID_TIMD_IDX;
  for( int i = 0; i < TIMD_FUSION_NUM; i++ )
  {
    timdFusionWeight[i] = -1;
    timdLocDep[i]       = 0;
  }
#else
  timdFusionWeight[0] = -1;
  timdFusionWeight[1] = -1;
#endif
#if JVET_AJ0146_TIMDSAD
  timdSad                     = false;
#if JVET_AC0094_REF_SAMPLES_OPT 
  timdModeSad                 = INVALID_TIMD_IDX;
  timdModeSecondarySad        = INVALID_TIMD_IDX;
  timdModeCheckWASad          = true;
  timdModeSecondaryCheckWASad = true;
#else
  timdModeSad                 = -1;
  timdModeSecondarySad        = -1;
#endif
  timdIsBlendedSad     = false;
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int i = 0; i < TIMD_FUSION_NUM; i++)
  {
    timdBv[i]      = Mv(0, 0);
    isBvTimd[i]    = false;
    timdBvSad[i]   = Mv(0, 0);
    isBvTimdSad[i] = false;
  }
#endif
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  timdModeNonAngSad    = INVALID_TIMD_IDX;
  for( int i = 0; i < TIMD_FUSION_NUM; i++ )
  {
    timdFusionWeightSad[i] = -1;
    timdLocDepSad[i]       = 0;
  }
#else
  timdFusionWeightSad[0] = -1;
  timdFusionWeightSad[1] = -1;
#endif
#endif
#if JVET_AJ0061_TIMD_MERGE
  timdMrg = 0;
  for (int i = 0; i <= NUM_TIMD_MERGE_MODES; i++)
  {
    timdmTrType[i][0] = TransType::DCT2;
    timdmTrType[i][1] = TransType::DCT2;
  }
  for (int i = 0; i < NUM_TIMD_MERGE_MODES; i++)
  {
    timdMrgIsBlended[i] = false;
    for (int j = 0; j < TIMD_FUSION_NUM; j++)
    {
      timdMrgList[i][j] = INVALID_TIMD_IDX;
      timdMrgFusionWeight[i][j] = -1;
      timdMrgModeCheckWA[i][j] = true;
      timdMrgLocDep[i][j] = 0;
    }
  }
  timdMrgCand = -1;
#endif
#endif
#if JVET_AB0155_SGPM
  timdHor      = -1;
  timdVer      = -1;
  sgpm         = false;
  sgpmIdx      = -1;
  sgpmSplitDir = -1;
  sgpmMode0    = -1;
  sgpmMode1    = -1;
#if JVET_AG0152_SGPM_ITMP_IBC
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  sgpmBv0 = BvInfo(Mv(0, 0), 0);
  sgpmBv1 = BvInfo(Mv(0, 0), 0);
#else
  sgpmBv0      = Mv(0,0);
  sgpmBv1      = Mv(0,0);
#endif
#endif
#endif
#if JVET_AG0058_EIP
  eipFlag = false;
  eipMerge = false;
#if JVET_AJ0082_MM_EIP
  eipMmFlag = false;
#endif
#if JVET_AL0106_BV_EIP
  bvEip = false;
#endif
#endif
#if ENABLE_OBMC
  obmcFlag          = true;
  isobmcMC          = false;
#endif
  imv               = 0;
  imvNumCand        = 0;
  bcwIdx            = BCW_DEFAULT;
  for (int i = 0; i < 2; i++)
    refIdxBi[i] = -1;
  smvdMode        = 0;
  ispMode           = 0;
  mipFlag           = false;
#if JVET_AB0067_MIP_DIMD_LFNST
  mipDimdMode       = 0;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  sgpmDimdMode = 0;
#endif
#if JVET_V0130_INTRA_TMP
  tmpFlag = false;
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
  intraTmpDimdMode = -1;
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  for (int iTimd = 0; iTimd < NumTimdMode; iTimd++)
  {
    timdDimdMode[iTimd]       = -1;
    timdSecondDimdMode[iTimd] = -1;
  }
  timdSadDimdMode       = -1;
  timdSecondSadDimdMode = -1;
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  dimdDimdMode       = -1;
  dimdSecondDimdMode = -1;
  obicDimdMode       = -1;
  obicSecondDimdMode = -1;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  dimdDerivedIntraDir = 0;
#if JVET_AI0050_INTER_MTSS
  dimdDerivedIntraDir2nd = 0;
#endif
#endif
#endif
#if JVET_AG0276_NLIC
  altLMFlag = false;
  altLMParaUnit.resetAltLinearModel();
#if JVET_AG0276_LIC_FLAG_SIGNALING
  altLMBRParaUnit.resetAltLinearModel();
#endif
#if ENABLE_OBMC
  secAltLMParaUnit.resetAltLinearModel();
#endif
#endif
#if INTER_LIC
  licFlag = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  licInheritPara = false;
#endif
#if JVET_AD0213_LIC_IMP
  for (int i = 0; i < 2; i++)
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      licScale[i][comp] = 32;
      licOffset[i][comp] = 0;
#else
      licScale[i][comp] = MAX_INT;
      licOffset[i][comp] = MAX_INT;
#endif
    }
  }
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  licDelta = 0;
#endif
#endif
#if JVET_AC0112_IBC_LIC
  ibcLicFlag = false;
#if JVET_AE0078_IBC_LIC_EXTENSION
  ibcLicIdx = 0;
#if JVET_AM0157_SGPM_BV_LIC
  for (int i = 0; i < 2; i++)
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      ibcLicMean[i][comp] = 0;
    }
  }
  for (int i = 0; i < 2; i++)
  {
    for (int modeIdx = 0; modeIdx < 2; modeIdx++)
    {
      for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ibcLicScale[i][modeIdx][comp] = 0;
        ibcLicShift[i][modeIdx][comp] = 0;
        ibcLicOffset[i][modeIdx][comp] = 0;
      }
    }
  }
#endif
#endif
#endif
#if JVET_AE0159_FIBC
  for (int i = 0; i < FIBC_PARAMS; i++)
  {
    ibcFilterParams[i] = -1;
  }
  ibcFilterFlag = false;
#endif
#if JVET_AA0070_RRIBC
  rribcFlipType = 0;
#endif
#if JVET_AB0157_TMRL
  tmrlFlag = false;
  tmrlListIdx = 0;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bvOneZeroComp = 0;
  bvZeroCompDir = 0;
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  areAboveRightUnavail = false;
  areBelowLeftUnavail  = false;
#endif

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = 0;
    reusePLTSize[idx] = 0;
    lastPLTSize[idx]  = 0;
    useEscape[idx]    = false;
    useRotation[idx]  = false;
    memset(reuseflag[idx], false, MAXPLTPREDSIZE * sizeof(bool));
  }

  for (int idx = 0; idx < MAX_NUM_COMPONENT; idx++)
  {
    memset(curPLT[idx], 0, MAXPLTSIZE * sizeof(Pel));
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  treeType          = TREE_D;
  modeType          = MODE_TYPE_ALL;
  modeTypeSeries    = 0;
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  isSST                          = false;
  separateTree                   = false;
  intraRegionRootDepth           = -1;
  intraRegionRootQtDepth         = -1;
  intraRegionRootBtDepth         = -1;
  intraRegionRootMtDepth         = -1;
  intraRegionRootImplicitBtDepth = -1;
#endif
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const bool CodingUnit::isSepTree() const
{
  return treeType != TREE_D || CS::isDualITree( *cs );
}

const bool CodingUnit::isLocalSepTree() const
{
  return treeType != TREE_D && !CS::isDualITree(*cs);
}
#endif
#if !CCLM_LATENCY_RESTRICTION_RMV
const bool CodingUnit::checkCCLMAllowed() const
{
  bool allowCCLM = false;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( chType != CHANNEL_TYPE_CHROMA ) //single tree I slice or non-I slice
#else
  if( !CS::isDualITree( *cs ) ) //single tree I slice or non-I slice (Note: judging chType is no longer equivalent to checking dual-tree I slice since the local dual-tree is introduced)
#endif
  {
    allowCCLM = true;
  }
  else if( slice->getSPS()->getCTUSize() <= 32 ) //dual tree, CTUsize < 64
  {
    allowCCLM = true;
  }
  else //dual tree, CTU size 64 or 128
  {
#if TU_256
    int depthFor64x64Node = 0;
#else
    int depthFor64x64Node = slice->getSPS()->getCTUSize() == 128 ? 1 : 0;
#endif
    const PartSplit cuSplitTypeDepth1 = CU::getSplitAtDepth( *this, depthFor64x64Node );
    const PartSplit cuSplitTypeDepth2 = CU::getSplitAtDepth( *this, depthFor64x64Node + 1 );

    //allow CCLM if 64x64 chroma tree node uses QT split or HBT+VBT split combination
    if( cuSplitTypeDepth1 == CU_QUAD_SPLIT || (cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_VERT_SPLIT) )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width <= 32 && blocks[COMPONENT_Cb].height <= 32 ), "chroma cu size shall be <= 16x16 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width <= 16 && blocks[COMPONENT_Cb].height <= 16), "chroma cu size shall be <= 16x16 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x64 chroma tree node uses NS (No Split) and becomes a chroma CU containing 32x32 chroma blocks
    else if( cuSplitTypeDepth1 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width >= 32 && blocks[COMPONENT_Cb].height >= 32 ), "chroma cu size shall be 64x64 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width == 32 && blocks[COMPONENT_Cb].height == 32), "chroma cu size shall be 32x32 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x32 chroma tree node uses NS and becomes a chroma CU containing 32x16 chroma blocks
    else if( cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width >= 32 && blocks[COMPONENT_Cb].height >= 16 ), "chroma cu size shall be 64x32 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width == 32 && blocks[COMPONENT_Cb].height == 16), "chroma cu size shall be 32x16 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }

    //further check luma conditions
    if( allowCCLM )
    {
      //disallow CCLM if luma 64x64 block uses BT or TT or NS with ISP
      const Position lumaRefPos( chromaPos().x << getComponentScaleX( COMPONENT_Cb, chromaFormat ), chromaPos().y << getComponentScaleY( COMPONENT_Cb, chromaFormat ) );
      const CodingUnit* colLumaCu = cs->picture->cs->getCU( lumaRefPos, CHANNEL_TYPE_LUMA );

#if TU_256
      if( colLumaCu->lwidth() < MAX_TB_SIZEY || colLumaCu->lheight() < MAX_TB_SIZEY ) //further split at CTU luma node
#else
      if( colLumaCu->lwidth() < 64 || colLumaCu->lheight() < 64 ) //further split at 64x64 luma node
#endif
      {
        const PartSplit cuSplitTypeDepth1Luma = CU::getSplitAtDepth( *colLumaCu, depthFor64x64Node );
        CHECK( !(cuSplitTypeDepth1Luma >= CU_QUAD_SPLIT && cuSplitTypeDepth1Luma <= CU_TRIV_SPLIT), "split mode shall be BT, TT or QT" );
        if( cuSplitTypeDepth1Luma != CU_QUAD_SPLIT )
        {
          allowCCLM = false;
        }
      }
#if TU_256
      else if( colLumaCu->lwidth() == MAX_TB_SIZEY && colLumaCu->lheight() == MAX_TB_SIZEY && colLumaCu->ispMode ) //not split at CTU luma node and use ISP mode
#else
      else if( colLumaCu->lwidth() == 64 && colLumaCu->lheight() == 64 && colLumaCu->ispMode ) //not split at 64x64 luma node and use ISP mode
#endif
      {
        allowCCLM = false;
      }
    }
  }

  return allowCCLM;
}
#endif
const uint8_t CodingUnit::checkAllowedSbt() const
{
  if( !slice->getSPS()->getUseSBT() )
  {
    return 0;
  }

  //check on prediction mode
  if (predMode == MODE_INTRA || predMode == MODE_IBC || predMode == MODE_PLT ) //intra, palette or IBC
  {
    return 0;
  }
  if( firstPU->ciipFlag )
  {
    return 0;
  }
#if JVET_Y0065_GPM_INTRA && !JVET_AI0050_SBT_LFNST
  if (firstPU->gpmIntraFlag)
  {
    return 0;
  }
#endif

  uint8_t sbtAllowed = 0;
  int cuWidth  = lwidth();
  int cuHeight = lheight();
  bool allowType[NUMBER_SBT_IDX];
  memset( allowType, false, NUMBER_SBT_IDX * sizeof( bool ) );

  //parameter
  int maxSbtCUSize = cs->sps->getMaxTbSize();
  int minSbtCUSize = 1 << ( MIN_CU_LOG2 + 1 );

  //check on size
  if( cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize )
  {
    return 0;
  }

  allowType[SBT_VER_HALF] = cuWidth  >= minSbtCUSize;
  allowType[SBT_HOR_HALF] = cuHeight >= minSbtCUSize;
  allowType[SBT_VER_QUAD] = cuWidth  >= ( minSbtCUSize << 1 );
  allowType[SBT_HOR_QUAD] = cuHeight >= ( minSbtCUSize << 1 );
#if JVET_AJ0260_SBT_CORNER_MODE
  allowType[ SBT_QUAD ]    = cuWidth >= minSbtCUSize && cuHeight >= minSbtCUSize && cuWidth >= SBT_QUAD_MIN_BLOCK_SIZE    && cuHeight >= SBT_QUAD_MIN_BLOCK_SIZE;
  allowType[ SBT_QUARTER ] = cuWidth >= minSbtCUSize && cuHeight >= minSbtCUSize && cuWidth >= SBT_QUARTER_MIN_BLOCK_SIZE && cuHeight >= SBT_QUARTER_MIN_BLOCK_SIZE;
#endif

  for( int i = 0; i < NUMBER_SBT_IDX; i++ )
  {
    sbtAllowed += (uint8_t)allowType[i] << i;
  }

  return sbtAllowed;
}

uint8_t CodingUnit::getSbtTuSplit() const
{
  uint8_t sbtTuSplitType = 0;

  switch( getSbtIdx() )
  {
  case SBT_VER_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_HALF_POS0_SPLIT; break;
  case SBT_HOR_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_HALF_POS0_SPLIT; break;
  case SBT_VER_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_QUAD_POS0_SPLIT; break;
  case SBT_HOR_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_QUAD_POS0_SPLIT; break;
#if JVET_AJ0260_SBT_CORNER_MODE
  case SBT_QUAD:     sbtTuSplitType = getSbtPos() + SBT_QUAD_POSTL_SPLIT;    break;
  case SBT_QUARTER:  sbtTuSplitType = getSbtPos() + SBT_QUARTER_POSTL_SPLIT; break;
#endif
  default: assert( 0 );  break;
  }

#if JVET_AJ0260_SBT_CORNER_MODE
  CHECK( sbtTuSplitType < SBT_VER_HALF_POS0_SPLIT || sbtTuSplitType >= NUM_PART_SPLIT, "Wrong SBT split type" );
#else
  CHECK( sbtTuSplitType < SBT_VER_HALF_POS0_SPLIT || sbtTuSplitType > SBT_HOR_QUAD_POS1_SPLIT, "Wrong SBT split type" );
#endif
  return sbtTuSplitType;
}


#if JVET_AJ0260_SBT_CORNER_MODE
int CodingUnit::getSbtTuIdx() const
{
  int idx = 0;

  if( sbtInfo )
  {
    const int sbtIdx = getSbtIdx();
    const int sbtPos = getSbtPos();

    if( sbtIdx == SBT_QUAD )
    {
      idx = sbtPos;
    }
    else if( sbtIdx == SBT_QUARTER )
    {
      if( sbtPos == 0 )
      {
        idx = 0;
      }
      else if( sbtPos == 1 )
      {
        idx = 3;
      }
      else if( sbtPos == 2 )
      {
        idx = 12;
      }
      else if( sbtPos == 3 )
      {
        idx = 15;
      }
      else
      {
        CHECK( true, "Wrong SBT position" );
      }
    }
    else
    {
      idx = sbtPos;
    }
  }

  return idx;
}
#endif

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

PredictionUnit::PredictionUnit(const UnitArea &unit)                                : UnitArea(unit)                , cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }
PredictionUnit::PredictionUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }

void PredictionUnit::initData()
{
  // intra data - need this default initialization for PCM

  intraDir[0] = DC_IDX;
  intraDir[1] = PLANAR_IDX;
#if JVET_AB0155_SGPM
  intraDir1[0] = DC_IDX;
  intraDir1[1] = PLANAR_IDX;
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if JVET_AC0119_LM_CHROMA_FUSION
  isChromaFusion = 0;
#else
  isChromaFusion = false;
#endif
#endif
  mipTransposedFlag = false;
  multiRefIdx = 0;
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  decoderDerivedCcpMode = 0;
  ddNonLocalCCPFusion = 0;
#endif
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
  parseLumaMode = false;
  candId = -1;
  parseChromaMode = false;
#endif
  mpmFlag = false;
  ipredIdx = -1;
  secondMpmFlag = false;
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = {};
#endif
#if JVET_AA0126_GLM
  glmIdc      = {};
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  cccmFlag    = 0;
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  cccmNoSubFlag = 0;
#endif
#if JVET_AC0054_GLCCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  glCccmFlag = 0;
#endif
#if JVET_AD0202_CCCM_MDF
  cccmMultiFilterIdx = 0;
#endif
#if JVET_AE0100_BVGCCCM
  bvgCccmFlag = 0;
  numBvgCands = 0;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = Mv(0, 0);
    rrIbcList[candIdx] = 0;
  }
#endif
#endif
#if JVET_AD0188_CCP_MERGE
  idxNonLocalCCP = 0;
  curCand = {};
  curCand.type = CCP_TYPE_NONE;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  ccpMergeAdjustFlag = 0;
#endif
#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  ccInsideFilter = 0;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  ccpMergeFusionFlag = 0;
  ccpMergeFusionType = 0;
#endif
#if JVET_AJ0081_CHROMA_TMRL
  chromaMrlIdx = 0;
  chromaTmrlFlag = false;
  chromaTmrlIdx = 0;
#endif
  // inter data
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  colIdx = 0;
#endif
  mergeFlag   = false;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  mergeOppositeLic = false;
  affineOppositeLic = false;
  tmMergeFlagOppositeLic = false;
#endif
  regularMergeFlag = false;
  mergeIdx    = MAX_UCHAR;
  geoSplitDir  = MAX_UCHAR;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = MAX_UCHAR;
#endif
  geoMergeIdx0 = MAX_UCHAR;
  geoMergeIdx1 = MAX_UCHAR;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = false;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  geoBlendIntraFlag = false;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  gpmInterIbcFlag = false;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = false;
  geoMMVDIdx0 = MAX_UCHAR;
  geoMMVDFlag1 = false;
  geoMMVDIdx1 = MAX_UCHAR;
#if TM_MRG
  geoTmFlag0 = false;
  geoTmFlag1 = false;
#endif
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
  geoBlendTmFlag = false;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  geoBldIdx = MAX_UCHAR;
#endif
#if JVET_AL0134_SGPM_INTER
  sgpmInter          = false;
  sgpmInterTm        = false;
  sgpmInterIdx       = MAX_UCHAR;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  sgpmIntra          = false;
  sgpmAffine         = false;
#endif
#endif
  mmvdMergeFlag = false;
  mmvdMergeIdx = MAX_UCHAR;
#if JVET_AE0169_BIPREDICTIVE_IBC
  ibcMergeIdx1 = MAX_INT;
#endif
#if JVET_AA0061_IBC_MBVD
  ibcMbvdMergeFlag = false;
  ibcMbvdMergeIdx = MAX_INT;
#endif
#if AFFINE_MMVD
  afMmvdFlag    = false;
  afMmvdBaseIdx = UINT8_MAX;
  afMmvdStep    = UINT8_MAX;
  afMmvdDir     = UINT8_MAX;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = UINT8_MAX;
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = false;
  bmDir = 0;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  affBMMergeFlag = false;
  affBMDir = 0;
#endif
  interDir    = MAX_UCHAR;
  mergeType   = MRG_TYPE_DEFAULT_N;
  bv.setZero();
  bvd.setZero();
  mvRefine = false;
#if MULTI_PASS_DMVR
  bdmvrRefine = false;
#else
  ::memset(mvdL0SubPu, 0, sizeof(mvdL0SubPu));
#endif
#if JVET_AF0057
  dmvrImpreciseMv = false;
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  availableBdofRefinedMv = 0;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  reduceTplSize = false;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  amvpSbTmvpFlag = false;
  amvpSbTmvpMvdIdx = -1;
#endif
#if JVET_AC0112_IBC_GPM
  ibcGpmFlag = false;
  ibcGpmSplitDir = MAX_UCHAR;
  ibcGpmMergeIdx0 = MAX_UCHAR;
  ibcGpmMergeIdx1 = MAX_UCHAR;
  ibcGpmBldIdx = MAX_UCHAR;
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = -1;
  refPairIdx = -1;
#endif
  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = false;
#endif
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i]     .setZero();
    mvd[i]    .setZero();
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
    mvsdIdx[i] = -1;
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j].setZero();
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j].setZero();
#if JVET_AG0164_AFFINE_GPM
      gpmPartmvAffi[0][i][j].setZero();
      gpmPartmvAffi[1][i][j].setZero();
#endif
    }
  }
#if JVET_AG0164_AFFINE_GPM
  affineGPM[0] = affineGPM[1] = 0;
  gpmPartRefIdx[0][0]  = gpmPartRefIdx[0][1]  = gpmPartRefIdx[1][0] = gpmPartRefIdx[1][1] = -1;
  gpmPartAffType[0] = gpmPartAffType[1] = AFFINE_MODEL_NUM;
#endif

  ciipFlag = false;
#if CIIP_PDPC
  ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  ciipAffine = false;
#endif
#if JVET_AC0112_IBC_CIIP
  ibcCiipFlag = false;
  ibcCiipIntraIdx = 0;
#endif
  mmvdEncOptMode = 0;
#if MULTI_HYP_PRED
  addHypData.clear();
  numMergedAddHyps = 0;
#endif

#if JVET_AE0046_BI_GPM
  gpmDirMode = 0;
  gpmDmvrRefinePart0 = false;
  gpmDmvrRefinePart1 = false;
#endif
}

PredictionUnit& PredictionUnit::operator=(const IntraPredictionData& predData)
{

  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    intraDir[i] = predData.intraDir[i];
#if JVET_AB0155_SGPM
    intraDir1[i] = predData.intraDir1[i];
#endif
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION
  isChromaFusion = predData.isChromaFusion;
#endif
  mipTransposedFlag = predData.mipTransposedFlag;
  multiRefIdx = predData.multiRefIdx;
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  decoderDerivedCcpMode = predData.decoderDerivedCcpMode;
  ddNonLocalCCPFusion = predData.ddNonLocalCCPFusion;
#endif
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
  parseLumaMode = predData.parseLumaMode;
  candId = predData.candId;
  parseChromaMode = predData.parseChromaMode;
#endif
  mpmFlag = predData.mpmFlag;
  ipredIdx = predData.ipredIdx;
  secondMpmFlag = predData.secondMpmFlag;
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = predData.cclmOffsets;
#endif
#if JVET_AA0126_GLM
  glmIdc      = predData.glmIdc;
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  cccmFlag    = predData.cccmFlag;
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  cccmNoSubFlag = predData.cccmNoSubFlag;
#endif
#if JVET_AC0054_GLCCCM
  glCccmFlag = predData.glCccmFlag;
#endif
#if JVET_AD0202_CCCM_MDF
  cccmMultiFilterIdx = predData.cccmMultiFilterIdx;
#endif
#if JVET_AE0100_BVGCCCM
  bvgCccmFlag = predData.bvgCccmFlag;
  numBvgCands = predData.numBvgCands;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = predData.bvList[candIdx];
    rrIbcList[candIdx] = predData.rrIbcList[candIdx];
  }
#endif
#endif
#if JVET_AD0188_CCP_MERGE
  idxNonLocalCCP  = predData.idxNonLocalCCP;
  curCand = predData.curCand;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  ccpMergeAdjustFlag = predData.ccpMergeAdjustFlag;
#endif
#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  ccInsideFilter = predData.ccInsideFilter;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  ccpMergeFusionFlag = predData.ccpMergeFusionFlag;
  ccpMergeFusionType = predData.ccpMergeFusionType;
#endif
#if JVET_AJ0081_CHROMA_TMRL
  chromaMrlIdx = predData.chromaMrlIdx;
  chromaTmrlFlag = predData.chromaTmrlFlag;
  chromaTmrlIdx = predData.chromaTmrlIdx;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=(const InterPredictionData& predData)
{
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  colIdx = predData.colIdx;
#endif
  mergeFlag   = predData.mergeFlag;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  mergeOppositeLic = predData.mergeOppositeLic;
  affineOppositeLic = predData.affineOppositeLic;
  tmMergeFlagOppositeLic = predData.tmMergeFlagOppositeLic;
#endif
  regularMergeFlag = predData.regularMergeFlag;
  mergeIdx    = predData.mergeIdx;
  geoSplitDir  = predData.geoSplitDir;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = predData.geoSyntaxMode;
#endif
  geoMergeIdx0 = predData.geoMergeIdx0;
  geoMergeIdx1 = predData.geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = predData.gpmIntraFlag;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  geoBlendIntraFlag = predData.geoBlendIntraFlag;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  gpmInterIbcFlag = predData.gpmInterIbcFlag;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = predData.geoMMVDFlag0;
  geoMMVDIdx0 = predData.geoMMVDIdx0;
  geoMMVDFlag1 = predData.geoMMVDFlag1;
  geoMMVDIdx1 = predData.geoMMVDIdx1;
#if TM_MRG
  geoTmFlag0 = predData.geoTmFlag0;
  geoTmFlag1 = predData.geoTmFlag1;
#endif
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
  geoBlendTmFlag = predData.geoBlendTmFlag;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  geoBldIdx = predData.geoBldIdx;
#endif
#if JVET_AL0134_SGPM_INTER
  sgpmInter          = predData.sgpmInter;
  sgpmInterTm        = predData.sgpmInterTm;
  sgpmInterIdx       = predData.sgpmInterIdx;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  sgpmIntra          = predData.sgpmIntra;
  sgpmAffine         = predData.sgpmAffine;
#endif
#endif
  mmvdMergeFlag = predData.mmvdMergeFlag;
  mmvdMergeIdx = predData.mmvdMergeIdx;
#if JVET_AE0169_BIPREDICTIVE_IBC
  ibcMergeIdx1 = predData.ibcMergeIdx1;
#endif
#if JVET_AA0061_IBC_MBVD
  ibcMbvdMergeFlag = predData.ibcMbvdMergeFlag;
  ibcMbvdMergeIdx = predData.ibcMbvdMergeIdx;
#endif
#if AFFINE_MMVD
  afMmvdFlag    = predData.afMmvdFlag;
  afMmvdBaseIdx = predData.afMmvdBaseIdx;
  afMmvdStep    = predData.afMmvdStep;
  afMmvdDir     = predData.afMmvdDir;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = predData.afMmvdMergeIdx;
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  tmMergeFlag = predData.tmMergeFlag;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = predData.bmMergeFlag;
  bmDir = predData.bmDir;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  affBMMergeFlag = predData.affBMMergeFlag;
  affBMDir = predData.affBMDir;
#endif
  interDir    = predData.interDir;
  mergeType   = predData.mergeType;
  bv          = predData.bv;
  bvd         = predData.bvd;
  mvRefine = predData.mvRefine;
#if MULTI_PASS_DMVR
  bdmvrRefine = predData.bdmvrRefine;
#else
  ::memcpy(mvdL0SubPu, predData.mvdL0SubPu, sizeof(mvdL0SubPu));
#endif
#if JVET_AF0057
  dmvrImpreciseMv = predData.dmvrImpreciseMv;
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  availableBdofRefinedMv = predData.availableBdofRefinedMv;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  reduceTplSize = predData.reduceTplSize;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  amvpSbTmvpFlag = predData.amvpSbTmvpFlag;
  amvpSbTmvpMvdIdx = predData.amvpSbTmvpMvdIdx;
#endif
#if JVET_AC0112_IBC_GPM
  ibcGpmFlag = predData.ibcGpmFlag;
  ibcGpmSplitDir = predData.ibcGpmSplitDir;
  ibcGpmMergeIdx0 = predData.ibcGpmMergeIdx0;
  ibcGpmMergeIdx1 = predData.ibcGpmMergeIdx1;
  ibcGpmBldIdx = predData.ibcGpmBldIdx;
#endif

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = predData.amvpMergeModeFlag[i];
#endif
    mvpIdx[i]   = predData.mvpIdx[i];
    mvpNum[i]   = predData.mvpNum[i];
    mv[i]       = predData.mv[i];
    mvd[i]      = predData.mvd[i];
    refIdx[i]   = predData.refIdx[i];
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
    mvsdIdx[i] = predData.mvsdIdx[i];
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = predData.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = predData.mvAffi[i][j];
#if JVET_AG0164_AFFINE_GPM
      gpmPartmvAffi[0][i][j] = predData.gpmPartmvAffi[0][i][j];
      gpmPartmvAffi[1][i][j] = predData.gpmPartmvAffi[1][i][j];
#endif
    }
  }
#if JVET_AG0164_AFFINE_GPM
  affineGPM[0] = predData.affineGPM[0];
  affineGPM[1] = predData.affineGPM[1];

  gpmPartRefIdx[0][0]  = predData.gpmPartRefIdx[0][0];
  gpmPartRefIdx[0][1]  = predData.gpmPartRefIdx[0][1];
  gpmPartRefIdx[1][0]  = predData.gpmPartRefIdx[1][0];
  gpmPartRefIdx[1][1]  = predData.gpmPartRefIdx[1][1];

  gpmPartAffType[0] = predData.gpmPartAffType[0];
  gpmPartAffType[1] = predData.gpmPartAffType[1];
#endif

#if JVET_AD0140_MVD_PREDICTION
  mvdSuffixInfo = predData.mvdSuffixInfo;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  bvdSuffixInfo = predData.bvdSuffixInfo;
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = predData.refIdxLC;
  refPairIdx = predData.refPairIdx;
#endif
  ciipFlag = predData.ciipFlag;
#if CIIP_PDPC
  ciipPDPC = predData.ciipPDPC;
#endif
#if JVET_AG0135_AFFINE_CIIP
  ciipAffine = predData.ciipAffine;
#endif
#if JVET_AC0112_IBC_CIIP
  ibcCiipFlag = predData.ibcCiipFlag;
  ibcCiipIntraIdx = predData.ibcCiipIntraIdx;
#endif
#if MULTI_HYP_PRED
  addHypData = predData.addHypData;
  numMergedAddHyps = predData.numMergedAddHyps;
#endif
#if JVET_AE0046_BI_GPM
  gpmDirMode = predData.gpmDirMode;
  gpmDmvrRefinePart0 = predData.gpmDmvrRefinePart0;
  gpmDmvrRefinePart1 = predData.gpmDmvrRefinePart1;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const PredictionUnit& other )
{


  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    intraDir[ i ] = other.intraDir[ i ];
#if JVET_AB0155_SGPM
    intraDir1[i] = other.intraDir1[i];
#endif
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION
  isChromaFusion = other.isChromaFusion;
#endif
  mipTransposedFlag = other.mipTransposedFlag;
  multiRefIdx = other.multiRefIdx;
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  decoderDerivedCcpMode = other.decoderDerivedCcpMode;
  ddNonLocalCCPFusion = other.ddNonLocalCCPFusion;
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = other.cclmOffsets;
#endif
#if JVET_AA0126_GLM
  glmIdc      = other.glmIdc;
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  cccmFlag    = other.cccmFlag;
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  cccmNoSubFlag = other.cccmNoSubFlag;
#endif
#if JVET_AC0054_GLCCCM
  glCccmFlag = other.glCccmFlag;
#endif
#if JVET_AD0202_CCCM_MDF
  cccmMultiFilterIdx = other.cccmMultiFilterIdx;
#endif
#if JVET_AE0100_BVGCCCM
  bvgCccmFlag = other.bvgCccmFlag;
  numBvgCands = other.numBvgCands;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = other.bvList[candIdx];
    rrIbcList[candIdx] = other.rrIbcList[candIdx];
  }
#endif
#endif
#if JVET_AD0188_CCP_MERGE
  idxNonLocalCCP  = other.idxNonLocalCCP;
  curCand = other.curCand;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  ccpMergeAdjustFlag = other.ccpMergeAdjustFlag;
#endif

#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  ccInsideFilter = other.ccInsideFilter;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  ccpMergeFusionFlag = other.ccpMergeFusionFlag;
  ccpMergeFusionType = other.ccpMergeFusionType;
#endif
#if JVET_AJ0081_CHROMA_TMRL
  chromaMrlIdx = other.chromaMrlIdx;
  chromaTmrlFlag = other.chromaTmrlFlag;
  chromaTmrlIdx = other.chromaTmrlIdx;
#endif
  mergeFlag   = other.mergeFlag;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  mergeOppositeLic = other.mergeOppositeLic;
  affineOppositeLic = other.affineOppositeLic;
  tmMergeFlagOppositeLic = other.tmMergeFlagOppositeLic;
#endif
  regularMergeFlag = other.regularMergeFlag;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  colIdx = other.colIdx;
#endif
  mergeIdx    = other.mergeIdx;
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
  parseLumaMode = other.parseLumaMode;
  candId = other.candId;
  parseChromaMode = other.parseChromaMode;
#endif
  mpmFlag = other.mpmFlag;
  ipredIdx = other.ipredIdx;
  secondMpmFlag = other.secondMpmFlag;
  geoSplitDir  = other.geoSplitDir;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = other.geoSyntaxMode;
#endif
  geoMergeIdx0 = other.geoMergeIdx0;
  geoMergeIdx1 = other.geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = other.gpmIntraFlag;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  geoBlendIntraFlag = other.geoBlendIntraFlag;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  gpmInterIbcFlag = other.gpmInterIbcFlag;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = other.geoMMVDFlag0;
  geoMMVDIdx0 = other.geoMMVDIdx0;
  geoMMVDFlag1 = other.geoMMVDFlag1;
  geoMMVDIdx1 = other.geoMMVDIdx1;
#if TM_MRG
  geoTmFlag0 = other.geoTmFlag0;
  geoTmFlag1 = other.geoTmFlag1;
#endif
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
  geoBlendTmFlag = other.geoBlendTmFlag;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  geoBldIdx = other.geoBldIdx;
#endif
#if JVET_AL0134_SGPM_INTER
  sgpmInter          = other.sgpmInter;
  sgpmInterTm        = other.sgpmInterTm;
  sgpmInterIdx       = other.sgpmInterIdx;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  sgpmIntra          = other.sgpmIntra;
  sgpmAffine         = other.sgpmAffine;   
#endif
#endif
  mmvdMergeFlag = other.mmvdMergeFlag;
  mmvdMergeIdx = other.mmvdMergeIdx;
#if JVET_AE0169_BIPREDICTIVE_IBC
  ibcMergeIdx1 = other.ibcMergeIdx1;
#endif
#if JVET_AA0061_IBC_MBVD
  ibcMbvdMergeFlag = other.ibcMbvdMergeFlag;
  ibcMbvdMergeIdx = other.ibcMbvdMergeIdx;
#endif
#if AFFINE_MMVD
  afMmvdFlag    = other.afMmvdFlag;
  afMmvdBaseIdx = other.afMmvdBaseIdx;
  afMmvdStep    = other.afMmvdStep;
  afMmvdDir     = other.afMmvdDir;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = other.afMmvdMergeIdx;
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  tmMergeFlag = other.tmMergeFlag;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = other.bmMergeFlag;
  bmDir = other.bmDir;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  affBMMergeFlag = other.affBMMergeFlag;
  affBMDir = other.affBMDir;
#endif
  interDir    = other.interDir;
  mergeType   = other.mergeType;
  bv          = other.bv;
  bvd         = other.bvd;
  mvRefine = other.mvRefine;
#if MULTI_PASS_DMVR
  bdmvrRefine = other.bdmvrRefine;
#else
  ::memcpy(mvdL0SubPu, other.mvdL0SubPu, sizeof(mvdL0SubPu));
#endif
#if JVET_AF0057
  dmvrImpreciseMv = other.dmvrImpreciseMv;
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  availableBdofRefinedMv = other.availableBdofRefinedMv;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  reduceTplSize = other.reduceTplSize;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  amvpSbTmvpFlag = other.amvpSbTmvpFlag;
  amvpSbTmvpMvdIdx = other.amvpSbTmvpMvdIdx;
#endif
#if JVET_AC0112_IBC_GPM
  ibcGpmFlag = other.ibcGpmFlag;
  ibcGpmSplitDir = other.ibcGpmSplitDir;
  ibcGpmMergeIdx0 = other.ibcGpmMergeIdx0;
  ibcGpmMergeIdx1 = other.ibcGpmMergeIdx1;
  ibcGpmBldIdx = other.ibcGpmBldIdx;
#endif

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = other.amvpMergeModeFlag[i];
#endif
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    mv[i]       = other.mv[i];
    mvd[i]      = other.mvd[i];
    refIdx[i]   = other.refIdx[i];
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
    mvsdIdx[i] = other.mvsdIdx[i];
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = other.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = other.mvAffi[i][j];
#if JVET_AG0164_AFFINE_GPM
      gpmPartmvAffi[0][i][j] = other.gpmPartmvAffi[0][i][j];
      gpmPartmvAffi[1][i][j] = other.gpmPartmvAffi[1][i][j];
#endif
    }
  }

#if JVET_AG0164_AFFINE_GPM
  affineGPM[0] = other.affineGPM[0];
  affineGPM[1] = other.affineGPM[1];

  gpmPartRefIdx[0][0] = other.gpmPartRefIdx[0][0];
  gpmPartRefIdx[0][1] = other.gpmPartRefIdx[0][1];
  gpmPartRefIdx[1][0] = other.gpmPartRefIdx[1][0];
  gpmPartRefIdx[1][1] = other.gpmPartRefIdx[1][1];

  gpmPartAffType[0] = other.gpmPartAffType[0];
  gpmPartAffType[1] = other.gpmPartAffType[1];
#endif

#if JVET_AD0140_MVD_PREDICTION
  mvdSuffixInfo = other.mvdSuffixInfo;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  bvdSuffixInfo = other.bvdSuffixInfo;
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = other.refIdxLC;
  refPairIdx = other.refPairIdx;
#endif
  ciipFlag = other.ciipFlag;
#if CIIP_PDPC
  ciipPDPC = other.ciipPDPC;
#endif
#if JVET_AG0135_AFFINE_CIIP
  ciipAffine = other.ciipAffine;
#endif
#if JVET_AC0112_IBC_CIIP
  ibcCiipFlag = other.ibcCiipFlag;
  ibcCiipIntraIdx = other.ibcCiipIntraIdx;
#endif
#if MULTI_HYP_PRED
  addHypData = other.addHypData;
  numMergedAddHyps = other.numMergedAddHyps;
#endif
#if JVET_AE0046_BI_GPM
  gpmDirMode = other.gpmDirMode;
  gpmDmvrRefinePart0 = other.gpmDmvrRefinePart0;
  gpmDmvrRefinePart1 = other.gpmDmvrRefinePart1;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const MotionInfo& mi )
{
  interDir = mi.interDir;

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv    [i] = mi.mv[i];
  }
#if MULTI_HYP_PRED
  addHypData.clear();
  numMergedAddHyps = 0;
#endif

  return *this;
}

#if JVET_Z0139_HIST_AFF
#if JVET_AG0164_AFFINE_GPM
void PredictionUnit::getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2], MvField baseMv[2]) const
#else
void PredictionUnit::getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2]) const
#endif
{
#if JVET_AG0164_AFFINE_GPM
  baseMv[0] = MvField( Mv(), -1);
  baseMv[1] = MvField( Mv(), -1);

  if (cu->geoFlag)
  {
    CHECK( !affineGPM[0] && !affineGPM[1], "Not GPM-affine");
    for (int list = 0; list < 2; list++)
    {
      refIdxOut[list] = NOT_VALID;
      affineMiOut[list].oneSetAffineParametersPattern = 0;

      for (int gpmPartIdx = 0; gpmPartIdx < 2; gpmPartIdx++)
      {
        if (affineGPM[gpmPartIdx])
        {
          CHECK(gpmPartAffType[gpmPartIdx] == AFFINE_MODEL_NUM, "Invalid affine type");

          if (gpmPartRefIdx[gpmPartIdx][list] == -1)
          {
            continue;
          }

          refIdxOut[list] = gpmPartRefIdx[gpmPartIdx][list];

          int affpara[4];
          PU::deriveAffineParametersFromMVs(*this, gpmPartmvAffi[gpmPartIdx][list], affpara, gpmPartAffType[gpmPartIdx]);
          PU::storeAffParas(affpara);

          affineMiOut[list].oneSetAffineParameters[0] = (short)(affpara[0]);
          affineMiOut[list].oneSetAffineParameters[1] = (short)(affpara[1]);
          affineMiOut[list].oneSetAffineParameters[2] = (short)(affpara[2]);
          affineMiOut[list].oneSetAffineParameters[3] = (short)(affpara[3]);

          baseMv[list] = MvField(gpmPartmvAffi[gpmPartIdx][list][0], gpmPartRefIdx[gpmPartIdx][list]);
          break;
        }
      }
    }
    return;
  }
#endif
  for (int list = 0; list < 2; list++)
  {
    RefPicList eRefList = (list == 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    refIdxOut[list] = NOT_VALID;
    affineMiOut[list].oneSetAffineParametersPattern = 0;

    if ((interDir == 1 && list == 1) || (interDir == 2 && list == 0))
    {
      continue;
    }
    refIdxOut[list] = refIdx[list];

    int affpara[4];
    PU::deriveAffineParametersFromMVs(*this, mvAffi[eRefList], affpara, (EAffineModel)this->cu->affineType);
    PU::storeAffParas(affpara);

    affineMiOut[list].oneSetAffineParameters[0] = (short)(affpara[0]);
    affineMiOut[list].oneSetAffineParameters[1] = (short)(affpara[1]);
    affineMiOut[list].oneSetAffineParameters[2] = (short)(affpara[2]);
    affineMiOut[list].oneSetAffineParameters[3] = (short)(affpara[3]);

#if JVET_AG0164_AFFINE_GPM
    baseMv[list] = MvField(mvAffi[list][0], refIdxOut[list]);
#endif
  }
}
#endif

const MotionInfo& PredictionUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& PredictionUnit::getMotionInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  if( cs->sps->getGDREnabledFlag() )
  {
    bool isSrcClean = cs->isClean(Y().bottomRight(), CHANNEL_TYPE_LUMA);
    bool isTarClean = cs->isClean(pos, CHANNEL_TYPE_LUMA);

    if (isSrcClean && !isTarClean)
    {
      return constMotionIntra;
    }
  }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CHECKD( Y().valid() && !Y().contains(pos), "Trying to access motion info outsied of PU");
#else
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
#endif
  return cs->getMotionInfo( pos );
}

MotionBuf PredictionUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf PredictionUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
bool PredictionUnit::isMvdPredApplicable() const
{
#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*cu) && cs->sps->getUseBvdPred())
  {
    return true;
  }
#endif

  if (!cs->sps->getUseMvdPred())
  {
    return false;
  }
  if (cu->firstPU->addHypData.size() - cu->firstPU->numMergedAddHyps > 0)
  {
    return false;
  }
  return true;
}
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
bool PredictionUnit::isBvdPredApplicable() const
{
  if (!cs->sps->getUseBvdPred())
  {
    return false;
  }

  return true;
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
bool PredictionUnit::isBvpClusterApplicable() const
{
  if (!cs->sps->getUseBvpCluster())
  {
    return false;
  }

  return true;
}
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
uint32_t PredictionUnit::getBvType() const
{
#if JVET_AA0070_RRIBC
  int bvType = cu->rribcFlipType;
#else
  int bvType = 0;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bvType = isBvpClusterApplicable() ? cu->bvZeroCompDir : bvType;
#endif

  // 0: 2-D; 1: Hor 1-D; 2: Ver 1-D
  return bvType;
}
#endif

#if JVET_W0123_TIMD_FUSION
const uint8_t& PredictionUnit::getIpmInfo() const
{
  return cs->getIpmInfo( lumaPos() );
}

const uint8_t& PredictionUnit::getIpmInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  if( cs->sps->getGDREnabledFlag() )
  {
    bool isSrcClean = cs->isClean(Y().bottomRight(), CHANNEL_TYPE_LUMA);
    bool isTarClean = cs->isClean(pos, CHANNEL_TYPE_LUMA);

    if (isSrcClean && !isTarClean)
    {
      return constIpm;    
    }
  }
#endif

  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getIpmInfo( pos );
}

IpmBuf PredictionUnit::getIpmBuf()
{
  return cs->getIpmBuf( *this );
}

CIpmBuf PredictionUnit::getIpmBuf() const
{
  return cs->getIpmBuf( *this );
}
#endif


// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea& unit) : UnitArea(unit), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[i] = nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = nullptr;
#endif
  }

  for (unsigned i = 0; i < MAX_NUM_TBLOCKS - 1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = nullptr;
#endif
    m_runType[i] = nullptr;
  }

#if JVET_AL0181_ASBT
  for (uint32_t i = 0; i < getNumberValidComponents( chromaFormat ); i++)
  {
    blocksResidual[i] = CompArea(ComponentID(i), chromaFormat, blocks[i].x, blocks[i].y, blocks[i].width, blocks[i].height, (COMPONENT_Y == i));
    blocksResForDbf[i] = CompArea(ComponentID(i), chromaFormat, blocks[i].x, blocks[i].y, blocks[i].width, blocks[i].height, (COMPONENT_Y == i));
  }
#endif
  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
#if JVET_AL0181_ASBT
  for (uint32_t i = 0; i <getNumberValidComponents(chromaFormat); i++)
  {
    blocksResidual[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
    blocksResForDbf[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
#endif
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[i] = nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = nullptr;
#endif
  }

  for (unsigned i = 0; i < MAX_NUM_TBLOCKS - 1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = nullptr;
#endif
    m_runType[i] = nullptr;
  }

  initData();
}

void TransformUnit::initData()
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    cbf[i]           = 0;
    mtsIdx[i]        = MTS_DCT2_DCT2;
#if JVET_AL0181_ASBT
    blocksResidual[i] = block((ComponentID) i);
    blocksResForDbf[i] = block((ComponentID)i);
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
    lfnstIdx[i]      = 0;
#if JVET_AI0050_INTER_MTSS
    lfnstIntra[i]    = 0;
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
    mdirIdx[i] = 0;
    secondNSPTSet[i] = false;
#endif 
  }
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
    intraDirStat.first = 0;
    intraDirStat.second = 0;
#endif
  depth              = 0;
  noResidual         = false;
  jointCbCr          = 0;
  m_chromaResScaleInv = 0;
#if JVET_AE0059_INTER_CCCM
  interCccm          = 0;
#endif
#if JVET_AL0181_ASBT
  for (unsigned i = 0; i < MAX_NUM_COMPONENT; i++)
  {
    asbtDecimW[i]    = 0;
    asbtDecimH[i]    = 0;
  }
  asbtPosx           = 0;
  asbtPosy           = 0;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  interCcpMerge      = 0;
  curCand            = {};
  curCand.type       = CCP_TYPE_NONE;
#endif
}
#if REMOVE_PCM
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
void TransformUnit::init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, unsigned **signsScanIdx, Pel **pltIdx, bool **runType)
#else
void TransformUnit::init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, Pel **pltIdx, bool **runType)
#endif
#else
void TransformUnit::init(TCoeff **coeffs, Pel **pltIdx, bool **runType)
#endif
#else
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
void TransformUnit::init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, unsigned **signsScanIdx, Pel **pcmbuf, bool **runType)
#else
void TransformUnit::init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, Pel **pcmbuf, bool **runType)
#endif
#else
void TransformUnit::init(TCoeff **coeffs, Pel **pcmbuf, bool **runType)
#endif
#endif
{
  uint32_t numBlocks = getNumberValidTBlocks(*cs->pcv);

  for (uint32_t i = 0; i < numBlocks; i++)
  {
    m_coeffs[i] = coeffs[i];
#if SIGN_PREDICTION
    m_coeffSigns[i] = signs[i];
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = signsScanIdx[i];
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = pcmbuf[i];
#endif
  }

  // numBlocks is either 1 for 4:0:0, or 3 otherwise. It would perhaps be better to loop over getNumberValidChannels(*cs->pcv.chrFormat) for m_runType.
  for (uint32_t i = 0; i < std::max<uint32_t>(2, numBlocks)-1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = pltIdx[i];
#endif
    m_runType[i] = runType[i];
  }
}

TransformUnit& TransformUnit::operator=(const TransformUnit& other)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  unsigned numBlocks = ::getNumberValidTBlocks(*cs->pcv);
  for( unsigned i = 0; i < numBlocks; i++ )
  {
    CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

#if JVET_AL0181_ASBT
    blocksResidual[i] = other.blocksResidual[i];
    uint32_t area = blocksResidual[i].area();
    blocksResForDbf[i] = other.blocksResForDbf[i];
#else
    uint32_t area = blocks[i].area();
#endif

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
#if SIGN_PREDICTION
    if (m_coeffSigns[i] && other.m_coeffSigns[i] && m_coeffSigns[i] != other.m_coeffSigns[i])
      std::copy_n(other.m_coeffSigns[i], area, m_coeffSigns[i]);
#if JVET_Y0141_SIGN_PRED_IMPROVE
    if (m_coeffSignsIdx[i] && other.m_coeffSignsIdx[i] && m_coeffSignsIdx[i] != other.m_coeffSignsIdx[i]) memcpy(m_coeffSignsIdx[i], other.m_coeffSignsIdx[i], sizeof(unsigned) * area);
#endif
#endif
#if !REMOVE_PCM
    if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);
#endif
    if (cu->slice->getSPS()->getPLTMode() && i < 2)
    {
#if REMOVE_PCM
      if (m_pltIdx[i] && other.m_pltIdx[i] && m_pltIdx[i] != other.m_pltIdx[i]) memcpy(m_pltIdx[i], other.m_pltIdx[i], sizeof(Pel) * area);
#endif
      if (m_runType[i]   && other.m_runType[i]   && m_runType[i]   != other.m_runType[i]  ) memcpy(m_runType[i],   other.m_runType[i],   sizeof(bool) * area);
    }
    cbf[i]           = other.cbf[i];
    mtsIdx[i]        = other.mtsIdx[i];
#if JVET_AG0061_INTER_LFNST_NSPT
    lfnstIdx[i]      = other.lfnstIdx[i];
#if JVET_AI0050_INTER_MTSS
    lfnstIntra[i]    = other.lfnstIntra[i];
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
    mdirIdx[i] = other.mdirIdx[i];
    secondNSPTSet[i] = other.secondNSPTSet[i];
#endif
  }
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
    intraDirStat.first = other.intraDirStat.first;
    intraDirStat.second = other.intraDirStat.second;
#endif
  depth              = other.depth;
  noResidual         = other.noResidual;
  jointCbCr          = other.jointCbCr;
#if JVET_AE0059_INTER_CCCM
  interCccm          = other.interCccm;
#endif
#if JVET_AL0181_ASBT
  for (unsigned i = 0; i < numBlocks; i++)
  {
    asbtDecimW[i]    = other.asbtDecimW[i];
    asbtDecimH[i]    = other.asbtDecimH[i];
  }
  asbtPosx           = other.asbtPosx;
  asbtPosy           = other.asbtPosy;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  interCcpMerge      = other.interCcpMerge;
  curCand            = other.curCand;
#endif
  return *this;
}

void TransformUnit::copyComponentFrom(const TransformUnit& other, const ComponentID i)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

#if JVET_AL0181_ASBT
    blocksResidual[i] = other.blocksResidual[i];
    uint32_t area = blocksResidual[i].area();
    blocksResForDbf[i] = other.blocksResForDbf[i];
#else
  uint32_t area = blocks[i].area();
#endif

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
#if SIGN_PREDICTION
  if (m_coeffSigns[i] && other.m_coeffSigns[i] && m_coeffSigns[i] != other.m_coeffSigns[i])
    std::copy_n(other.m_coeffSigns[i], area, m_coeffSigns[i]);
#if JVET_Y0141_SIGN_PRED_IMPROVE
  if (m_coeffSignsIdx[i] && other.m_coeffSignsIdx[i] && m_coeffSignsIdx[i] != other.m_coeffSignsIdx[i]) memcpy(m_coeffSignsIdx[i], other.m_coeffSignsIdx[i], sizeof(unsigned) * area);
#endif
#endif
#if !REMOVE_PCM
  if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);
#endif
  if ((i == COMPONENT_Y || i == COMPONENT_Cb))
  {
#if REMOVE_PCM
    if (m_pltIdx[i] && other.m_pltIdx[i] && m_pltIdx[i] != other.m_pltIdx[i]) memcpy(m_pltIdx[i], other.m_pltIdx[i], sizeof(Pel) * area);
#endif
    if (m_runType[i] && other.m_runType[i] && m_runType[i] != other.m_runType[i])   memcpy(m_runType[i], other.m_runType[i], sizeof(bool) * area);
  }

  cbf[i]           = other.cbf[i];
  depth            = other.depth;
  mtsIdx[i]        = other.mtsIdx[i];
#if JVET_AG0061_INTER_LFNST_NSPT
  lfnstIdx[i]      = other.lfnstIdx[i];
#if JVET_AI0050_INTER_MTSS
  lfnstIntra[i]    = other.lfnstIntra[i];
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
  mdirIdx[i] = other.mdirIdx[i];
  secondNSPTSet[i] = other.secondNSPTSet[i];
#endif 
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
    intraDirStat.first = other.intraDirStat.first;
    intraDirStat.second = other.intraDirStat.second;
#endif
  noResidual       = other.noResidual;
  jointCbCr        = isChroma( i ) ? other.jointCbCr : jointCbCr;
#if JVET_AE0059_INTER_CCCM
  interCccm        = other.interCccm;
#endif
#if JVET_AL0181_ASBT
  asbtDecimW[i]    = other.asbtDecimW[i];
  asbtDecimH[i]    = other.asbtDecimH[i];
  asbtPosx         = other.asbtPosx;
  asbtPosy         = other.asbtPosy;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  interCcpMerge    = other.interCcpMerge;
  curCand          = other.curCand;
#endif
}
#if JVET_AL0181_ASBT
       CoeffBuf TransformUnit::getCoeffs(const ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocksResidual[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocksResidual[id]); }

#if SIGN_PREDICTION
AreaBuf<SIGN_PRED_TYPE> TransformUnit::getCoeffSigns(const ComponentID id)
{
  return AreaBuf<SIGN_PRED_TYPE>(m_coeffSigns[id], blocksResidual[id]);
}

const AreaBuf<SIGN_PRED_TYPE> TransformUnit::getCoeffSigns(const ComponentID id) const
{
  return AreaBuf<SIGN_PRED_TYPE>(m_coeffSigns[id], blocksResidual[id]);
}

void TransformUnit::fillBlocksResForDbf(const ComponentID id)
{
  fillBlocksResForDbfSize(this->blocksResForDbf, id);
  fillBlocksResForDbfPos(id);
}

void TransformUnit::fillBlocksResForDbfPos(const ComponentID id)
{
  if (isLuma(id))
  {
    blocksResForDbf[id].x = blocks[id].x + asbtPosx;
    blocksResForDbf[id].y = blocks[id].y + asbtPosy;
  }
  else
  {
    blocksResForDbf[id].x = blocks[id].x + (asbtPosx >> 1);
    blocksResForDbf[id].y = blocks[id].y + (asbtPosy >> 1);
  }
}

void TransformUnit::fillBlocksResForDbfSize(CompArea* blocksRes, const ComponentID id)
{
  blocksRes[id].width = blocks[id].width >> asbtDecimW[id];
  blocksRes[id].height = blocks[id].height >> asbtDecimH[id];
}

void TransformUnit::initBlockRes(CompArea* blocksRes, const ComponentID id)
{
  if (id == MAX_NUM_COMPONENT)
  {
    blocksRes[COMPONENT_Y] = blocks[COMPONENT_Y];
    blocksRes[COMPONENT_Y] = blocks[COMPONENT_Y];
    blocksRes[COMPONENT_Cr] = blocks[COMPONENT_Cr];
    blocksRes[COMPONENT_Cr] = blocks[COMPONENT_Cr];
    blocksRes[COMPONENT_Cb] = blocks[COMPONENT_Cb];
    blocksRes[COMPONENT_Cb] = blocks[COMPONENT_Cb];
  }
  else
  {
    blocksRes[id] = blocks[id];
  }
}

void TransformUnit::alignAsbtDecim(const ComponentID id)
{
  asbtDecimH[id] = asbtDecimH[COMPONENT_Y];
  asbtDecimW[id] = asbtDecimW[COMPONENT_Y];
}
#if JVET_Y0141_SIGN_PRED_IMPROVE
      IdxBuf    TransformUnit::getCoeffSignsScanIdx(const ComponentID id) { return  IdxBuf(m_coeffSignsIdx[id], blocksResidual[id]); }
const CIdxBuf   TransformUnit::getCoeffSignsScanIdx(const ComponentID id) const { return CIdxBuf(m_coeffSignsIdx[id], blocksResidual[id]); }
#endif
#endif
#else

       CoeffBuf TransformUnit::getCoeffs(const ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

#if SIGN_PREDICTION
AreaBuf<SIGN_PRED_TYPE> TransformUnit::getCoeffSigns(const ComponentID id)
{
  return AreaBuf<SIGN_PRED_TYPE>(m_coeffSigns[id], blocks[id]);
}
#if JVET_Y0141_SIGN_PRED_IMPROVE
      IdxBuf    TransformUnit::getCoeffSignsScanIdx(const ComponentID id) { return  IdxBuf(m_coeffSignsIdx[id], blocks[id]); }
const CIdxBuf   TransformUnit::getCoeffSignsScanIdx(const ComponentID id) const { return CIdxBuf(m_coeffSignsIdx[id], blocks[id]); }
#endif
#endif
#endif

#if JVET_AF0073_INTER_CCP_MERGE
const MotionInfo& TransformUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& TransformUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outside of TU" );
  return cs->getMotionInfo( pos );
}

const int& TransformUnit::getCcpmIdxInfo() const
{
  return cs->getCcpmIdxInfo( chromaPos() );
}

const int& TransformUnit::getCcpmIdxInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outside of TU" );
  return cs->getCcpmIdxInfo( pos );
}
#endif

#if REMOVE_PCM
       PelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)         { return        PelBuf(m_pltIdx[id], blocks[id]); }
const CPelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)   const { return       CPelBuf(m_pltIdx[id], blocks[id]); }
#else
       PelBuf   TransformUnit::getPcmbuf(const ComponentID id)       { return  PelBuf  (m_pcmbuf[id], blocks[id]); }
const CPelBuf   TransformUnit::getPcmbuf(const ComponentID id) const { return CPelBuf  (m_pcmbuf[id], blocks[id]); }

       PelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)         { return        PelBuf(m_pcmbuf[id], blocks[id]); }
const CPelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)   const { return       CPelBuf(m_pcmbuf[id], blocks[id]); }
#endif

       PLTtypeBuf   TransformUnit::getrunType  (const ComponentID id)         { return   PLTtypeBuf(m_runType[id], blocks[id]); }
const CPLTtypeBuf   TransformUnit::getrunType  (const ComponentID id)   const { return  CPLTtypeBuf(m_runType[id], blocks[id]); }

       PLTescapeBuf TransformUnit::getescapeValue(const ComponentID id)       { return  PLTescapeBuf(m_coeffs[id], blocks[id]); }
const CPLTescapeBuf TransformUnit::getescapeValue(const ComponentID id) const { return CPLTescapeBuf(m_coeffs[id], blocks[id]); }

#if REMOVE_PCM
      Pel*          TransformUnit::getPLTIndex   (const ComponentID id)       { return  m_pltIdx[id];    }
#else
      Pel*          TransformUnit::getPLTIndex   (const ComponentID id)       { return  m_pcmbuf[id];    }
#endif
      bool*         TransformUnit::getRunTypes   (const ComponentID id)       { return  m_runType[id];   }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
int TransformUnit::countNonZero()
{
  if (!cbf[0])
  {
    return 0;
  }
  int count = 0;
  auto areaY = Y();
  auto coeffY = m_coeffs[0];
  int blksize = areaY.width * areaY.height;
  for (auto i = 0; i < blksize; ++i)
  {
    if (coeffY[i])
    {
      count++;
    }
  }
  return count;
}
#endif
void TransformUnit::checkTuNoResidual( unsigned idx )
{
  if( CU::getSbtIdx( cu->sbtInfo ) == SBT_OFF_DCT )
  {
    return;
  }

  if( ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS0 && idx == 1 ) || ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS1 && idx == 0 ) )
  {
    noResidual = true;
  }
#if JVET_AJ0260_SBT_CORNER_MODE
  else if( CU::getSbtIdx( cu->sbtInfo ) == SBT_QUAD )
  {
    noResidual = CU::getSbtPos( cu->sbtInfo ) != idx ? true : false;
  }
  else if( CU::getSbtIdx( cu->sbtInfo ) == SBT_QUARTER )
  {
    if( CU::getSbtPos( cu->sbtInfo ) == 0 && idx == 0 )
    {
      noResidual = false;
    }
    else if( CU::getSbtPos( cu->sbtInfo ) == 1 && idx == 3 )
    {
      noResidual = false;
    }
    else if( CU::getSbtPos( cu->sbtInfo ) == 2 && idx == 12 )
    {
      noResidual = false;
    }
    else if( CU::getSbtPos( cu->sbtInfo ) == 3 && idx == 15 )
    {
      noResidual = false;
    }
    else
    {
      noResidual = true;
    }
  }
#endif
}

int TransformUnit::getTbAreaAfterCoefZeroOut(ComponentID compID) const
{
#if JVET_AL0181_ASBT
  int tbArea = blocksResidual[compID].width * blocksResidual[compID].height;
  int tbZeroOutWidth = blocksResidual[compID].width;
  int tbZeroOutHeight = blocksResidual[compID].height;
#else
  int tbArea = blocks[compID].width * blocks[compID].height;
  int tbZeroOutWidth = blocks[compID].width;
  int tbZeroOutHeight = blocks[compID].height;
#endif

#if !TU_256
  if ( cs->sps->getUseMTS() && cu->sbtInfo != 0 && blocks[compID].width <= 32 && blocks[compID].height <= 32 && compID == COMPONENT_Y )
  {
    tbZeroOutWidth = (blocks[compID].width == 32) ? 16 : tbZeroOutWidth;
    tbZeroOutHeight = (blocks[compID].height == 32) ? 16 : tbZeroOutHeight;
  }
#endif
  tbZeroOutWidth = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutWidth);
  tbZeroOutHeight = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutHeight);
  tbArea = tbZeroOutWidth * tbZeroOutHeight;
  return tbArea;
}
#if JVET_Y0141_SIGN_PRED_IMPROVE
bool TransformUnit::checkLFNSTApplied(ComponentID compID)
{
  const uint32_t  lfnstIdx = this->cu->lfnstIdx;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool  lfnstApplied = lfnstIdx && this->mtsIdx[compID] != MTS_SKIP && (this->cu->isSepTree() ? true : isLuma(compID));
#else
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool  lfnstApplied = lfnstIdx && this->mtsIdx[compID] != MTS_SKIP && (cu->separateTree ? true : isLuma(compID));
#else
  bool  lfnstApplied = lfnstIdx && this->mtsIdx[compID] != MTS_SKIP && (CS::isDualITree(*this->cs) ? true : isLuma(compID));
#endif
#endif
  return lfnstApplied;
}
#endif
int          TransformUnit::getChromaAdj()                     const { return m_chromaResScaleInv; }
void         TransformUnit::setChromaAdj(int i)                      { m_chromaResScaleInv = i;    }

#if SIGN_PREDICTION
void TransformUnit::initSignBuffers( const ComponentID compID )
{  
#if JVET_AL0181_ASBT
  uint32_t uiWidth = blocksResidual[compID].width;
  uint32_t uiHeight = blocksResidual[compID].height;
#else
  uint32_t uiWidth = blocks[compID].width;
  uint32_t uiHeight = blocks[compID].height;
#endif

  if( cs->sps->getNumPredSigns() > 0 && uiHeight >= 4 && uiWidth >= 4 )
  {
    AreaBuf<SIGN_PRED_TYPE> signBuff = getCoeffSigns(compID);
    SIGN_PRED_TYPE         *coeff    = signBuff.buf;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    IdxBuf signScanIdxBuff = getCoeffSignsScanIdx( compID );
    uint32_t spArea = std::max( cs->sps->getSignPredArea(), SIGN_PRED_FREQ_RANGE );
    unsigned int *coeffIdx = signScanIdxBuff.buf;
    uint32_t spWidth = std::min( uiWidth, spArea );
    uint32_t spHeight = std::min( uiHeight, spArea );
    CHECK(SIGN_PRED_BYPASS, "SIGN_PRED_BYPASS should be equal to 0");

    for( uint32_t y = 0; y < spHeight; y++ )
#else
    for( uint32_t y = 0; y < SIGN_PRED_FREQ_RANGE; y++ )
#endif
    {
#if JVET_Y0141_SIGN_PRED_IMPROVE
      std::fill_n(coeff, spWidth, SIGN_PRED_BYPASS);
      memset( coeffIdx, MAX_UINT, sizeof( unsigned int ) * spWidth );
      coeffIdx += signScanIdxBuff.stride;
#else
      coeff[0] = SIGN_PRED_BYPASS;
      coeff[1] = SIGN_PRED_BYPASS;
      coeff[2] = SIGN_PRED_BYPASS;
      coeff[3] = SIGN_PRED_BYPASS;
#endif
      coeff += signBuff.stride;
    }
  }
}
#endif
