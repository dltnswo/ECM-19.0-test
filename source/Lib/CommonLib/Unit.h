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

/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#ifndef __UNIT__
#define __UNIT__

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"
#include "ChromaFormat.h"
#if JVET_AJ0249_NEURAL_NETWORK_BASED
#include <array>
#endif

// ---------------------------------------------------------------------------
// tools
// ---------------------------------------------------------------------------

struct PLTBuf {
  uint8_t        curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel            curPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE];
};
inline Position recalcPosition(const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Position &pos)
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return pos;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Position(pos.x >> getComponentScaleX(dstCId, _cf), pos.y >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Position(pos.x << getComponentScaleX(srcCId, _cf), pos.y << getComponentScaleY(srcCId, _cf));
  }
}

inline Position recalcPosition( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Position &pos )
{
  if( srcCHt == dstCHt )
  {
    return pos;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Position( pos.x >> getChannelTypeScaleX( dstCHt, _cf ), pos.y >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Position( pos.x << getChannelTypeScaleX( srcCHt, _cf ), pos.y << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Size &size )
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return size;
  }
  else if( isLuma( srcCId ) && isChroma( dstCId ) )
  {
    return Size( size.width >> getComponentScaleX( dstCId, _cf ), size.height >> getComponentScaleY( dstCId, _cf ) );
  }
  else
  {
    return Size( size.width << getComponentScaleX( srcCId, _cf ), size.height << getComponentScaleY( srcCId, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Size &size )
{
  if( srcCHt == dstCHt )
  {
    return size;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Size( size.width >> getChannelTypeScaleX( dstCHt, _cf ), size.height >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Size( size.width << getChannelTypeScaleX( srcCHt, _cf ), size.height << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

// ---------------------------------------------------------------------------
// block definition
// ---------------------------------------------------------------------------

struct CompArea : public Area
{
  CompArea() : Area(), chromaFormat(NUM_CHROMA_FORMAT), compID(MAX_NUM_TBLOCKS)                                                                                                                                 { }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Area &_area, const bool isLuma = false)                                          : Area(_area),          chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Position& _pos, const Size& _size, const bool isLuma = false)                    : Area(_pos, _size),    chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const uint32_t _x, const uint32_t _y, const uint32_t _w, const uint32_t _h, const bool isLuma = false) : Area(_x, _y, _w, _h), chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }

  ChromaFormat chromaFormat;
  ComponentID compID;

  Position chromaPos() const;
  Position lumaPos()   const;

  Size     chromaSize() const;
  Size     lumaSize()   const;

  Position compPos( const ComponentID compID ) const;
  Position chanPos( const ChannelType chType ) const;

  Position topLeftComp    (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, *this);                                                     }
  Position topRightComp   (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), y                          }); }
  Position bottomLeftComp (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { x                        , (PosType) (y + height - 1 )}); }
  Position bottomRightComp(const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), (PosType) (y + height - 1 )}); }

  bool valid() const { return chromaFormat < NUM_CHROMA_FORMAT && compID < MAX_NUM_TBLOCKS && width != 0 && height != 0; }

  const bool operator==(const CompArea &other) const
  {
    if (chromaFormat != other.chromaFormat) return false;
    if (compID       != other.compID)       return false;

    return Position::operator==(other) && Size::operator==(other);
  }

  const bool operator!=(const CompArea &other) const { return !(operator==(other)); }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
  void     resizeTo          (const Size& newSize)          { Size::resizeTo(newSize); }
#endif
  void     repositionTo      (const Position& newPos)       { Position::repositionTo(newPos); }
  void     positionRelativeTo(const CompArea& origCompArea) { Position::relativeTo(origCompArea); }

private:

  void xRecalcLumaToChroma();
};

inline CompArea clipArea(const CompArea &compArea, const Area &boundingBox)
{
  return CompArea(compArea.compID, compArea.chromaFormat, clipArea((const Area&) compArea, boundingBox));
}

// ---------------------------------------------------------------------------
// unit definition
// ---------------------------------------------------------------------------

typedef static_vector<CompArea, MAX_NUM_TBLOCKS> UnitBlocksType;

struct UnitArea
{
  ChromaFormat chromaFormat;
  UnitBlocksType blocks;

  UnitArea() : chromaFormat(NUM_CHROMA_FORMAT) { }
  UnitArea(const ChromaFormat _chromaFormat);
  UnitArea(const ChromaFormat _chromaFormat, const Area &area);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY, const CompArea  &blkCb, const CompArea  &blkCr);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,       CompArea &&blkCb,       CompArea &&blkCr);

        CompArea& Y()                                  { return blocks[COMPONENT_Y];  }
  const CompArea& Y()                            const { return blocks[COMPONENT_Y];  }
        CompArea& Cb()                                 { return blocks[COMPONENT_Cb]; }
  const CompArea& Cb()                           const { return blocks[COMPONENT_Cb]; }
        CompArea& Cr()                                 { return blocks[COMPONENT_Cr]; }
  const CompArea& Cr()                           const { return blocks[COMPONENT_Cr]; }

        CompArea& block(const ComponentID comp)       { return blocks[comp]; }
  const CompArea& block(const ComponentID comp) const { return blocks[comp]; }

  bool contains(const UnitArea& other) const;
  bool contains(const UnitArea& other, const ChannelType chType) const;

        CompArea& operator[]( const int n )       { return blocks[n]; }
  const CompArea& operator[]( const int n ) const { return blocks[n]; }

  const bool operator==(const UnitArea &other) const
  {
    if (chromaFormat != other.chromaFormat)   return false;
    if (blocks.size() != other.blocks.size()) return false;

    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      if (blocks[i] != other.blocks[i]) return false;
    }

    return true;
  }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
  void resizeTo    (const UnitArea& unit);
#endif
  void repositionTo(const UnitArea& unit);

  const bool operator!=(const UnitArea &other) const { return !(*this == other); }

  const Position& lumaPos () const { return Y(); }
  const Size&     lumaSize() const { return Y(); }

  const Position& chromaPos () const { return Cb(); }
  const Size&     chromaSize() const { return Cb(); }

  const UnitArea  singleComp(const ComponentID compID) const;
  const UnitArea  singleChan(const ChannelType chType) const;

  const SizeType  lwidth()  const { return Y().width; }  /*! luma width  */
  const SizeType  lheight() const { return Y().height; } /*! luma height */

  const PosType   lx() const { return Y().x; }           /*! luma x-pos */
  const PosType   ly() const { return Y().y; }           /*! luma y-pos */

  bool valid() const { return chromaFormat != NUM_CHROMA_FORMAT && blocks.size() > 0; }
};

inline UnitArea clipArea(const UnitArea &area, const UnitArea &boundingBox)
{
  UnitArea ret(area.chromaFormat);

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    ret.blocks.push_back(clipArea(area.blocks[i], boundingBox.blocks[i]));
  }

  return ret;
}

struct UnitAreaRelative : public UnitArea
{
  UnitAreaRelative(const UnitArea& origUnit, const UnitArea& unit)
  {
    *((UnitArea*)this) = unit;
    for(uint32_t i = 0; i < blocks.size(); i++)
    {
      blocks[i].positionRelativeTo(origUnit.blocks[i]);
    }
  }
};

class SPS;
class VPS;
class DCI;
class PPS;
class Slice;

// ---------------------------------------------------------------------------
// coding unit
// ---------------------------------------------------------------------------

#include "Buffer.h"

struct TransformUnit;
struct PredictionUnit;
class  CodingStructure;

struct CodingUnit : public UnitArea
{
  CodingStructure *cs;
  Slice *slice;
  ChannelType    chType;

  PredMode       predMode;

  uint8_t        depth;   // number of all splits, applied with generalized splits
  uint8_t        qtDepth; // number of applied quad-splits, before switching to the multi-type-tree (mtt)
  // a triple split would increase the mtDepth by 1, but the qtDepth by 2 in the first and last part and by 1 in the middle part (because of the 1-2-1 split proportions)
  uint8_t        btDepth; // number of applied binary splits, after switching to the mtt (or it's equivalent)
  uint8_t        mtDepth; // the actual number of splits after switching to mtt (equals btDepth if only binary splits are allowed)
#if JVET_AH0135_TEMPORAL_PARTITIONING
  uint8_t        mtImplicitDepth;
#endif
  int8_t         chromaQpAdj;
  int8_t         qp;
  SplitSeries    splitSeries;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  TreeType       treeType;
  ModeType       modeType;
  ModeTypeSeries modeTypeSeries;
#endif
  bool           skip;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  int8_t         interCcpMergeZeroRootCbfIdc;
#endif
  bool           mmvdSkip;
  bool           affine;
  int8_t         affineType;
  bool           colorTransform;
  bool           geoFlag;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  bool           geoBlendFlag;
#endif
  int8_t         bdpcmMode;
  int8_t         bdpcmModeChroma;
  uint8_t        imv;
  bool           rootCbf;
  uint8_t        sbtInfo;
  uint32_t       tileIdx;
#if JVET_AC0105_DIRECTIONAL_PLANAR
  uint8_t        plIdx;
#endif
#if JVET_AM0074_INTRA_MERGE
  int intraMergeMode{ 0 };
  uint8_t allowIntraMergeMode{ 0 };
  int8_t dimdModeMerge{ -1 };
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
  int dimdLocDepMerge[DIMD_FUSION_NUM - 1];
#else
  int dimdLocDepMerge[2];
#endif
#endif
#if JVET_AB0157_INTRA_FUSION
  int8_t dimdBlendModeMerge[DIMD_FUSION_NUM - 1];
  int8_t dimdRelWeightMerge[DIMD_FUSION_NUM];
#else
  int8_t dimdBlendModeMerge[2];
  int8_t dimdRelWeightMerge[3];
#endif
  bool   isBvDimdExtAvail;
  bool   isBvDimdExt[DIMD_FUSION_NUM];
  Mv     bvDimdExt[DIMD_FUSION_NUM];
#endif
#if ENABLE_DIMD
#if JVET_AH0076_OBIC
  bool obicFlag;
  bool obicIsBlended;
  int obicMode[OBIC_FUSION_NUM];
  int obicFusionWeight[OBIC_FUSION_NUM];
#if JVET_AK0056_WEIGHTED_OBIC
  int obicLocDep[OBIC_FUSION_NUM];
#endif
#endif
#if JVET_AL0108_BVG_DIMD
  bool           bvgDimdFlag;
  int8_t         bvgDimdFusionWeight[BVG_DIMD_INTRA_NUM];
  int8_t         bvgDimdMode[BVG_DIMD_INTRA_NUM];
  int            bvDimdNum;
  Mv             bvDimdList[BVG_DIMD_BV_LIST_SIZE];
  int            bvDimdWeight[BVG_DIMD_BV_LIST_SIZE];
#endif
#if JVET_AG0146_DIMD_ITMP_IBC
  bool           isBvDimd;
  Mv             bvDimd;
#endif
  bool           dimd;
  bool           dimdBlending;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
  int            dimdLocDep[DIMD_FUSION_NUM-1];
#else
  int            dimdLocDep[2];
#endif
#endif
  int8_t         dimdMode;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  int8_t         dimdChromaMode;
#if JVET_AC0094_REF_SAMPLES_OPT
  int8_t         dimdChromaModeSecond;
#endif
#endif
#if JVET_AK0064_CCP_LFNST_NSPT
  int8_t         ccpChromaDimdMode[4]; // jointCbCr: 0,1,2,3
#endif
#if JVET_AB0157_INTRA_FUSION
  int8_t         dimdBlendMode[DIMD_FUSION_NUM-1]; // max number of blend modes (the main mode is not counter) --> incoherent with dimdRelWeight
  int8_t         dimdRelWeight[DIMD_FUSION_NUM]; // max number of predictions to blend
#else
  int8_t         dimdBlendMode[2]; // max number of blend modes (the main mode is not counter) --> incoherent with dimdRelWeight
  int8_t         dimdRelWeight[3]; // max number of predictions to blend
#endif
#endif
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  static_vector<int, 2> candModeListForTransform;
#endif
#if JVET_AK0217_INTRA_MTSS 
  static_vector<int, MTSS_LIST_SIZE> candModeListForTransformMtss;
  static_vector<int, MTSS_LIST_SIZE> candCostListForTransformMtss;
#endif 
#if JVET_AH0136_CHROMA_REORDERING
  int8_t         dimdBlendModeChroma[DIMD_FUSION_NUM - 1];
  uint8_t        chromaList[7];
  Mv             mvs[10];
  Mv             bvs[10];
  int            rribcTypes[10];
  int            mvsNum;
#endif
#if TMP_FAST_ENC
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  int                tmpXdisp;
  int                tmpYdisp;
#endif
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  Mv                 tmpXYdisp[MTMP_NUM];
  Mv                 tmpXYdispUseMR[MTMP_NUM];
#endif
  bool               tmpFlmFlag;
#if JVET_AG0136_INTRA_TMP_LIC
  bool               tmpLicFlag;
#endif
  uint8_t            tmpIdx;
  bool               tmpFusionFlag;
  int                tmpIsSubPel;
  int                tmpSubPelIdx;
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  int                tmpFracIdx;
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
  RefTemplateType    tempType;
#endif
#endif
#endif
#if JVET_AK0059_MDIP
  bool           mdip;
  int            mdipMode;
  bool           isModeExcluded;
  int            excludingMode[EXCLUDING_MODE_NUM];
#endif
#if JVET_W0123_TIMD_FUSION
  bool           timd;
  int            timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
  bool           timdModeCheckWA;
  bool           timdModeSecondaryCheckWA;
#endif
  int            timdModeSecondary;
  bool           timdIsBlended;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  int            timdModeNonAng;
  int8_t         timdFusionWeight[TIMD_FUSION_NUM];
  int8_t         timdLocDep[TIMD_FUSION_NUM];
#else
  int8_t         timdFusionWeight[2];
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  Mv             timdBv		[TIMD_FUSION_NUM];
  bool           isBvTimd	[TIMD_FUSION_NUM];
  Mv             timdBvSad	[TIMD_FUSION_NUM];
  bool           isBvTimdSad[TIMD_FUSION_NUM];
#endif
#if JVET_AJ0146_TIMDSAD
  bool           timdSad;
  int            timdModeSad;
#if JVET_AC0094_REF_SAMPLES_OPT
  bool           timdModeCheckWASad;
  bool           timdModeSecondaryCheckWASad;
#endif
  int            timdModeSecondarySad;
  bool           timdIsBlendedSad;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  int            timdModeNonAngSad;
  int8_t         timdFusionWeightSad[TIMD_FUSION_NUM];
  int8_t         timdLocDepSad[TIMD_FUSION_NUM];
#else
  int8_t         timdFusionWeightSad[2];
#endif
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  int            timdDimdMode[NumTimdMode];
  int            timdSadDimdMode;
  int            timdSecondDimdMode[NumTimdMode];
  int            timdSecondSadDimdMode;
#endif
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
  int            dimdDimdMode;
  int            dimdSecondDimdMode;
  int            obicDimdMode;
  int            obicSecondDimdMode;
#endif

#if JVET_AJ0061_TIMD_MERGE
  int           timdMrg;
  int           timdMrgList[NUM_TIMD_MERGE_MODES][TIMD_FUSION_NUM];
  bool          timdMrgIsBlended[NUM_TIMD_MERGE_MODES];
  int           timdMrgFusionWeight[NUM_TIMD_MERGE_MODES][TIMD_FUSION_NUM];
  bool          timdMrgModeCheckWA[NUM_TIMD_MERGE_MODES][TIMD_FUSION_NUM];
  int8_t        timdMrgLocDep[NUM_TIMD_MERGE_MODES][TIMD_FUSION_NUM];
  int           timdMrgCand;
  int           timdmTrType[NUM_TIMD_MERGE_MODES + 1][2];
#endif
#endif
#if JVET_AB0155_SGPM
  int            timdHor;
  int            timdVer;
#if JVET_AG0152_SGPM_ITMP_IBC
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  BvInfo         sgpmBv0;
  BvInfo         sgpmBv1;
#else
  Mv             sgpmBv0;
  Mv             sgpmBv1;
#endif
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  AffineBlendingModel sgpmBlendModel;
#endif
  bool           sgpm;
  int            sgpmIdx;
  int            sgpmSplitDir;
  int            sgpmMode0;
  int            sgpmMode1;
#endif
#if JVET_AG0058_EIP
  bool eipFlag;
  bool              eipMerge;
  EipModelCandidate eipModel;
#if JVET_AJ0082_MM_EIP
  bool              eipMmFlag;
#endif
#if JVET_AL0106_BV_EIP
  bool              bvEip;
#endif
#endif
#if ENABLE_OBMC
  bool           obmcFlag;
  bool           isobmcMC;
#endif
  uint8_t        mtsFlag;
#if JVET_AG0061_INTER_LFNST_NSPT
  uint8_t        lfnstFlag;
#if JVET_AI0050_INTER_MTSS
  uint8_t        lfnstIntra;
#endif
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  int            dimdDerivedIntraDir;
#if JVET_AI0050_INTER_MTSS
  int            dimdDerivedIntraDir2nd;
#endif
#endif
  uint8_t        lfnstIdx;
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  std::array<std::array<uint32_t, NUM_INDICES_REP>, MAX_NUM_COMPONENT> indicesRepresentationPnn;
  bool           lfnstSecFlag;
#endif
  uint8_t        bcwIdx;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  AffineBlendingModel blendModel;
#endif
  int8_t         refIdxBi[2];
  bool           mipFlag;
#if JVET_AB0067_MIP_DIMD_LFNST
  int            mipDimdMode;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  int            sgpmDimdMode;
#endif
#if JVET_V0130_INTRA_TMP
  bool		    	 tmpFlag;
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
  int            intraTmpDimdMode;
#endif
#endif
#if JVET_AG0276_NLIC
  bool           altLMFlag;
  AltLMInterUnit altLMParaUnit;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  AltLMInterUnit altLMBRParaUnit;
#endif
#if ENABLE_OBMC
  AltLMInterUnit secAltLMParaUnit;
#endif
#endif
#if INTER_LIC
  bool           licFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  bool           licInheritPara;
#endif
#if JVET_AD0213_LIC_IMP
  int            licScale[2][3];
  int            licOffset[2][3];
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
  int            licDelta;
#endif
#endif
#if JVET_AC0112_IBC_LIC
  bool           ibcLicFlag;
#if JVET_AE0078_IBC_LIC_EXTENSION
  int            ibcLicIdx;
#if JVET_AM0157_SGPM_BV_LIC
  int            ibcLicMean[2][3];
  int            ibcLicScale[2][2][3];
  int            ibcLicOffset[2][2][3];
  int            ibcLicShift[2][2][3];
#endif
#endif
#endif
#if JVET_AE0159_FIBC
  int64_t ibcFilterParams[FIBC_PARAMS];
  bool           ibcFilterFlag;
#endif
#if JVET_AA0070_RRIBC
  int            rribcFlipType;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  int            bvOneZeroComp;   // IBC BV has one null component
  int            bvZeroCompDir;   // 1: vertical null comp.  2: horizontal null comp.
#endif
#if JVET_AB0157_TMRL
  bool           tmrlFlag;
  uint8_t        tmrlListIdx;
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  bool           areAboveRightUnavail;
  bool           areBelowLeftUnavail;
#endif
  // needed for fast imv mode decisions
  int8_t         imvNumCand;
  uint8_t        smvdMode;
  uint8_t        ispMode;
  bool           useEscape[MAX_NUM_CHANNEL_TYPE];
  bool           useRotation[MAX_NUM_CHANNEL_TYPE];
  bool           reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE];
  uint8_t        lastPLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t        reusePLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t        curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel            curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE];

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool           isSST;
  bool           separateTree;
  int            intraRegionRootDepth;  
  int            intraRegionRootQtDepth;
  int            intraRegionRootBtDepth;
  int            intraRegionRootMtDepth;
  int            intraRegionRootImplicitBtDepth;

  CodingUnit() : chType( CH_L ), isSST( false ) { }
#else
  CodingUnit() : chType( CH_L ) { }
#endif
  CodingUnit(const UnitArea &unit);
  CodingUnit(const ChromaFormat _chromaFormat, const Area &area);

  CodingUnit& operator=( const CodingUnit& other );

  void initData();

  unsigned    idx;
  CodingUnit *next;

  PredictionUnit *firstPU;
  PredictionUnit *lastPU;

  TransformUnit *firstTU;
  TransformUnit *lastTU;
#if ENABLE_SPLIT_PARALLELISM

  int64_t cacheId;
  bool    cacheUsed;
#endif
  const uint8_t     getSbtIdx() const { CHECKD( ( ( sbtInfo >> 0 ) & 0xf ) >= NUMBER_SBT_IDX, "Wrong sbtInfo"); return ( sbtInfo >> 0 ) & 0xf; }
  const uint8_t     getSbtPos() const { return ( sbtInfo >> 4 ) & 0x3; }
  void              setSbtIdx( uint8_t idx ) { CHECKD( idx >= NUMBER_SBT_IDX, "sbt_idx wrong" ); sbtInfo = ( idx << 0 ) + ( sbtInfo & 0xf0 ); }
  void              setSbtPos( uint8_t pos ) { CHECKD( pos >= 4, "sbt_pos wrong" ); sbtInfo = ( pos << 4 ) + ( sbtInfo & 0xcf ); }
  uint8_t           getSbtTuSplit() const;
  const uint8_t     checkAllowedSbt() const;
#if !CCLM_LATENCY_RESTRICTION_RMV
  const bool        checkCCLMAllowed() const;
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const bool        isSepTree() const;

  const bool        isLocalSepTree() const;

  const bool        isConsInter() const { return modeType == MODE_TYPE_INTER; }
  const bool        isConsIntra() const { return modeType == MODE_TYPE_INTRA; }
#endif
#if JVET_AJ0260_SBT_CORNER_MODE
  int                getSbtTuIdx() const;
#endif
};

// ---------------------------------------------------------------------------
// prediction unit
// ---------------------------------------------------------------------------

struct IntraPredictionData
{
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION || JVET_AH0136_CHROMA_REORDERING || JVET_AJ0249_NEURAL_NETWORK_BASED
  bool      parseLumaMode = false;
  int8_t    candId = -1;
  bool      parseChromaMode = false;
#endif
  bool      mpmFlag = false;
  int8_t    ipredIdx = -1;
  bool      secondMpmFlag = false;
  uint8_t  intraDir[MAX_NUM_CHANNEL_TYPE];
#if JVET_AB0155_SGPM
  uint8_t intraDir1[MAX_NUM_CHANNEL_TYPE];
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if JVET_AC0119_LM_CHROMA_FUSION
  uint8_t   isChromaFusion;
#else
  bool      isChromaFusion;
#endif
#endif
  bool      mipTransposedFlag;
  int8_t    multiRefIdx;
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  int       decoderDerivedCcpMode;
  int       ddNonLocalCCPFusion;
#endif
#if JVET_Z0050_CCLM_SLOPE
  CclmOffsets cclmOffsets;
#endif
#if JVET_AA0126_GLM
  GlmIdc    glmIdc;
#endif
#if JVET_AA0057_CCCM || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  int       cccmFlag;
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  int       cccmNoSubFlag;
#endif
#if JVET_AC0054_GLCCCM
  int       glCccmFlag;
#endif
#if JVET_AD0202_CCCM_MDF
  int       cccmMultiFilterIdx;
#endif
#if JVET_AE0100_BVGCCCM
  int       bvgCccmFlag;
  int       numBvgCands;
  Mv        bvList[NUM_BVG_CCCM_CANDS];
  int       rrIbcList[NUM_BVG_CCCM_CANDS];
#endif
#endif
#if JVET_AD0188_CCP_MERGE
  int       idxNonLocalCCP;
  CCPModelCandidate curCand;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  int       ccpMergeAdjustFlag;
#endif
#if JVET_AD0120_LBCCP || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  int       ccInsideFilter;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  int       ccpMergeFusionFlag;
  int       ccpMergeFusionType;
#endif
#if JVET_AJ0081_CHROMA_TMRL
  int       chromaMrlIdx;
  bool      chromaTmrlFlag;
  int       chromaTmrlIdx;
#endif
};

struct InterPredictionData
{
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
#if JVET_AL0160_SBSMVP
  int8_t      colIdx;
#else
  uint8_t     colIdx;
#endif
#endif
  bool        mergeFlag;
  bool        regularMergeFlag;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uint16_t    mergeIdx;
#else
  uint8_t     mergeIdx;
#endif
  uint8_t     geoSplitDir;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  uint8_t     geoSyntaxMode;
#endif
  uint8_t     geoMergeIdx0;
  uint8_t     geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
  bool        gpmIntraFlag;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool        geoBlendIntraFlag;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  bool        gpmInterIbcFlag;
#endif
#if JVET_W0097_GPM_MMVD_TM
  bool        geoMMVDFlag0;
  uint8_t     geoMMVDIdx0;
  bool        geoMMVDFlag1;
  uint8_t     geoMMVDIdx1;
#if TM_MRG
  bool        geoTmFlag0;
  bool        geoTmFlag1;
  uint8_t     geoTmType;
#endif
#endif
#if JVET_AJ0274_REGRESSION_GPM_TM
  bool        geoBlendTmFlag;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  uint8_t     geoBldIdx;
#endif
#if JVET_AL0134_SGPM_INTER
  bool        sgpmInter;
  bool        sgpmInterTm;
  uint8_t     sgpmInterIdx;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  bool        sgpmIntra;
  bool        sgpmAffine;
#endif 
#endif
  bool        mmvdMergeFlag;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uint16_t    mmvdMergeIdx;
#else
  uint8_t     mmvdMergeIdx;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  int         ibcMergeIdx1;
#endif
#if JVET_AA0061_IBC_MBVD
  bool        ibcMbvdMergeFlag;
  int         ibcMbvdMergeIdx;
#endif
#if JVET_AC0112_IBC_CIIP
  bool        ibcCiipFlag;
  int         ibcCiipIntraIdx;
#endif
#if JVET_AC0112_IBC_GPM
  bool        ibcGpmFlag;
  uint8_t     ibcGpmSplitDir;
  uint8_t     ibcGpmMergeIdx0;
  uint8_t     ibcGpmMergeIdx1;
  uint8_t     ibcGpmBldIdx;
#endif
#if AFFINE_MMVD
  bool        afMmvdFlag;
  uint8_t     afMmvdBaseIdx; // base vector's merge index at the affine merge list, excluding sbTmvp
  uint8_t     afMmvdStep;
  uint8_t     afMmvdDir;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uint16_t    afMmvdMergeIdx;
#else
  uint8_t     afMmvdMergeIdx;
#endif
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  bool        tmMergeFlag;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool        tmMergeFlagOppositeLic;
  bool        mergeOppositeLic;
  bool        affineOppositeLic;
#endif
#if JVET_X0049_ADAPT_DMVR
  uint8_t     bmMergeFlag;
  uint8_t     bmDir;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  uint8_t     affBMMergeFlag;
  uint8_t     affBMDir;
#endif
  uint8_t     interDir;
  uint8_t     mvpIdx  [NUM_REF_PIC_LIST_01];
  uint8_t     mvpNum  [NUM_REF_PIC_LIST_01];
  Mv        mvd     [NUM_REF_PIC_LIST_01];
  Mv        mv      [NUM_REF_PIC_LIST_01];
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  Mv        mvHighPrec[NUM_REF_PIC_LIST_01];
#endif
#if MULTI_PASS_DMVR
  bool      bdmvrRefine;
#else
  Mv        mvdL0SubPu[MAX_NUM_SUBCU_DMVR];
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  int       availableBdofRefinedMv;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool      reduceTplSize;
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  bool      amvpMergeModeFlag[NUM_REF_PIC_LIST_01];
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  bool      amvpSbTmvpFlag;
  int8_t    amvpSbTmvpMvdIdx;
#endif
  int8_t     refIdx  [NUM_REF_PIC_LIST_01];
#if JVET_Z0054_BLK_REF_PIC_REORDER
  int8_t    refIdxLC;
  int8_t    refPairIdx;
#endif
  MergeType mergeType;
  bool      mvRefine;
  Mv        mvdAffi [NUM_REF_PIC_LIST_01][3];
  Mv        mvAffi[NUM_REF_PIC_LIST_01][3];

#if JVET_AG0164_AFFINE_GPM
  uint8_t     affineGPM[2];

  int8_t       gpmPartRefIdx[2][2];
  EAffineModel gpmPartAffType[2]; 
  Mv           gpmPartmvAffi[2][NUM_REF_PIC_LIST_01][3];

#endif

  bool      ciipFlag;
#if JVET_AF0057
  bool      dmvrImpreciseMv;
#endif
#if CIIP_PDPC
  bool      ciipPDPC;
#endif
#if JVET_AG0135_AFFINE_CIIP
  bool      ciipAffine;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  int       mvsdIdx[NUM_REF_PIC_LIST_01];
#endif
#if JVET_AD0140_MVD_PREDICTION
  MvdSuffixInfoMv mvdSuffixInfo;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  MvdSuffixInfo bvdSuffixInfo;
#endif

  Mv        bv;                             // block vector for IBC
  Mv        bvd;                            // block vector difference for IBC
  uint8_t   mmvdEncOptMode;                  // 0: no action 1: skip chroma MC for MMVD candidate pre-selection 2: skip chroma MC and BIO for MMVD candidate pre-selection
#if MULTI_HYP_PRED
  MultiHypVec addHypData;
  int8_t      numMergedAddHyps;
#endif

#if JVET_AE0046_BI_GPM
  int8_t   gpmDirMode; // 0: use uni list gen, 1 : use bi list gen
  bool     gpmDmvrRefinePart0;
  bool     gpmDmvrRefinePart1;
#endif
};

struct PredictionUnit : public UnitArea, public IntraPredictionData, public InterPredictionData
{
  CodingUnit      *cu;
  CodingStructure *cs;
  ChannelType      chType;
#if JVET_Z0118_GDR
  const MotionInfo constMotionIntra = { false };
  const uint8_t    constIpm = 0;
#endif

  // constructors
#if MULTI_PASS_DMVR
  PredictionUnit(): chType( CH_L ) { initData(); }
#else
  PredictionUnit(): chType( CH_L ) { }
#endif
  PredictionUnit(const UnitArea &unit);
  PredictionUnit(const ChromaFormat _chromaFormat, const Area &area);

  void initData();

  PredictionUnit& operator=(const IntraPredictionData& predData);
  PredictionUnit& operator=(const InterPredictionData& predData);
  PredictionUnit& operator=(const PredictionUnit& other);
  PredictionUnit& operator=(const MotionInfo& mi);

  unsigned        idx;

  PredictionUnit *next;

#if JVET_Z0139_HIST_AFF
#if JVET_AG0164_AFFINE_GPM
  void getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2], MvField baseMv[2]) const;
#else
  void getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2]) const;
#endif
#endif

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing neighboring motion information)
  const MotionInfo& getMotionInfo() const;
  const MotionInfo& getMotionInfo( const Position& pos ) const;
  MotionBuf         getMotionBuf();
  CMotionBuf        getMotionBuf() const;

#if JVET_W0123_TIMD_FUSION
  const uint8_t& getIpmInfo() const;
  const uint8_t& getIpmInfo( const Position& pos ) const;
  IpmBuf         getIpmBuf();
  CIpmBuf        getIpmBuf() const;
#endif

#if ENABLE_SPLIT_PARALLELISM

  int64_t cacheId;
  bool    cacheUsed;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  bool              isMvdPredApplicable() const;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  bool              isBvdPredApplicable() const;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bool isBvpClusterApplicable() const;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
  uint32_t          getBvType() const;
#endif
};

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
struct GeoBlendInfo
{
  AffineBlendingModel blendModel;
  Distortion  uiCostTmp;  // uicost template
  MvField     mvFieldA[2];
  MvField     mvFieldB[2];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  int         scaleA[2], scaleB[2];
  int         offsetA[2], offsetB[2];
#endif
  int         dir[2];     // 0,1 or 2 (bi-dir)
  int         mergeCand[2];
  Distortion  sad = 0;    // sad of CU prediction
  double      uiCost = 0;
  int         idxBufGeo;  // index of storage in m_acMergeBuffer[]
  int         iOrder;     // before re-ordering
  int         iMergeIdx;
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool        isIntra[2] = { false , false };
#endif

  bool isSame( MvField mvOtherFieldA[2], MvField mvOtherFieldB[2] 
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
             , int scaleOtherA [2], int scaleOtherB [2]
             , int offsetOtherA[2], int offsetOtherB[2]
#endif
  )
  {
    bool  bSameA = (mvFieldA[0] == mvOtherFieldA[0]) && (mvFieldA[1] == mvOtherFieldA[1]);
    bool  bSameB = (mvFieldB[0] == mvOtherFieldB[0]) && (mvFieldB[1] == mvOtherFieldB[1]);
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    bSameA &= (scaleA[0] == scaleOtherA[0] && offsetA[0] == offsetOtherA[0]) && (scaleA[1] == scaleOtherA[1] && offsetA[1] == offsetOtherA[1]);
    bSameB &= (scaleB[0] == scaleOtherB[0] && offsetB[0] == offsetOtherB[0]) && (scaleB[1] == scaleOtherB[1] && offsetB[1] == offsetOtherB[1]);
#endif
    return bSameA && bSameB;
  }
};
#endif

// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit      *cu;
  CodingStructure *cs;
  ChannelType      chType;
  int              m_chromaResScaleInv;
  uint8_t        depth;
  uint8_t        mtsIdx     [ MAX_NUM_TBLOCKS ];
#if JVET_AG0061_INTER_LFNST_NSPT
  uint8_t        lfnstIdx   [ MAX_NUM_TBLOCKS ];
#if JVET_AI0050_INTER_MTSS
  uint8_t        lfnstIntra [ MAX_NUM_TBLOCKS ];
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
  uint8_t        mdirIdx    [MAX_NUM_TBLOCKS];
  bool           secondNSPTSet[MAX_NUM_TBLOCKS];
#endif
  bool           noResidual;
  uint8_t        jointCbCr;
  uint8_t        cbf        [ MAX_NUM_TBLOCKS ];
#if JVET_AE0102_LFNST_CTX
  int            lastPosition[ MAX_NUM_TBLOCKS ];
#endif
#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  std::pair<uint32_t, uint32_t> intraDirStat;
#endif
#if JVET_AE0059_INTER_CCCM
  int8_t         interCccm;
#endif
#if JVET_AL0181_ASBT
  int8_t         asbtDecimW[MAX_NUM_COMPONENT];
  int8_t         asbtDecimH[MAX_NUM_COMPONENT];
  int8_t         asbtPosx;
  int8_t         asbtPosy;
  CompArea       blocksResidual[MAX_NUM_COMPONENT];
  CompArea       blocksResForDbf[MAX_NUM_COMPONENT];
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  int8_t         interCcpMerge;
  CCPModelCandidate curCand;
#endif
  TransformUnit() : chType( CH_L ) { }
  TransformUnit(const UnitArea& unit);
  TransformUnit(const ChromaFormat _chromaFormat, const Area &area);

  void initData();

  unsigned       idx;
  TransformUnit *next;
  TransformUnit *prev;
#if REMOVE_PCM
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  void init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, unsigned **signsScanIdx, Pel **pltIdx, bool **runType);
#else
  void init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, Pel **pltIdx, bool **runType);
#endif
#else
  void init(TCoeff **coeffs, Pel **pltIdx, bool **runType);
#endif
#else
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  void init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, unsigned **signsScanIdx, Pel **pcmbuf, bool **runType);
#else
  void init(TCoeff **coeffs, SIGN_PRED_TYPE **signs, Pel **pcmbuf, bool **runType);
#endif
#else
  void init(TCoeff **coeffs, Pel **pcmbuf, bool **runType);
#endif
#endif

  TransformUnit& operator=(const TransformUnit& other);
  void copyComponentFrom  (const TransformUnit& other, const ComponentID compID);
  void checkTuNoResidual( unsigned idx );
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int  countNonZero();
#endif
  int  getTbAreaAfterCoefZeroOut(ComponentID compID) const;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  bool checkLFNSTApplied(ComponentID compID);
#endif
         CoeffBuf getCoeffs(const ComponentID id);
  const CCoeffBuf getCoeffs(const ComponentID id) const;
#if SIGN_PREDICTION
  AreaBuf<SIGN_PRED_TYPE> getCoeffSigns(const ComponentID id);
#if JVET_AL0181_ASBT
  const AreaBuf<SIGN_PRED_TYPE> getCoeffSigns(const ComponentID id) const;
  void fillBlocksResForDbf(const ComponentID id);
  void fillBlocksResForDbfPos(const ComponentID id);
  void fillBlocksResForDbfSize(CompArea* blocksRes, const ComponentID id);
  void initBlockRes(CompArea* blocksRes, const ComponentID id);
  void alignAsbtDecim(const ComponentID id);
#endif
#if JVET_Y0141_SIGN_PRED_IMPROVE
  IdxBuf          getCoeffSignsScanIdx(const ComponentID id);
  const CIdxBuf   getCoeffSignsScanIdx(const ComponentID id) const;
#endif
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  const MotionInfo& getMotionInfo() const;
  const MotionInfo& getMotionInfo( const Position& pos ) const;
  
  const int& getCcpmIdxInfo() const;
  const int& getCcpmIdxInfo( const Position& pos ) const;
#endif
#if !REMOVE_PCM
         PelBuf   getPcmbuf(const ComponentID id);
  const CPelBuf   getPcmbuf(const ComponentID id) const;
#endif
        int       getChromaAdj( )                 const;
        void      setChromaAdj(int i);
         PelBuf   getcurPLTIdx(const ComponentID id);
  const CPelBuf   getcurPLTIdx(const ComponentID id) const;
         PLTtypeBuf   getrunType(const ComponentID id);
  const CPLTtypeBuf   getrunType(const ComponentID id) const;
         PLTescapeBuf getescapeValue(const ComponentID id);
  const CPLTescapeBuf getescapeValue(const ComponentID id) const;
        Pel*      getPLTIndex(const ComponentID id);
        bool*     getRunTypes(const ComponentID id);

#if ENABLE_SPLIT_PARALLELISM
  int64_t cacheId;
  bool    cacheUsed;

#endif
#if SIGN_PREDICTION
  void initSignBuffers( const ComponentID compID );
#endif

private:
  TCoeff *m_coeffs[ MAX_NUM_TBLOCKS ];
#if SIGN_PREDICTION
  SIGN_PRED_TYPE *m_coeffSigns[MAX_NUM_TBLOCKS];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned *m_coeffSignsIdx[MAX_NUM_TBLOCKS];
#endif
#endif
#if REMOVE_PCM
  Pel    *m_pltIdx[MAX_NUM_TBLOCKS - 1];
#else
  Pel    *m_pcmbuf[ MAX_NUM_TBLOCKS ];
#endif
  bool   *m_runType[ MAX_NUM_TBLOCKS - 1 ];
};

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

#include <iterator>

template<typename T>
class UnitIterator
{
private:
  T* m_punit;

public:
  UnitIterator(           ) : m_punit( nullptr ) { }
  UnitIterator( T* _punit ) : m_punit( _punit  ) { }

  typedef T&       reference;
  typedef T const& const_reference;
  typedef T*       pointer;
  typedef T const* const_pointer;

  reference        operator*()                                      { return *m_punit; }
  const_reference  operator*()                                const { return *m_punit; }
  pointer          operator->()                                     { return  m_punit; }
  const_pointer    operator->()                               const { return  m_punit; }

  UnitIterator<T>& operator++()                                     { m_punit = m_punit->next; return *this; }
  UnitIterator<T>  operator++( int )                                { auto x = *this; ++( *this ); return x; }
  bool             operator!=( const UnitIterator<T>& other ) const { return m_punit != other.m_punit; }
  bool             operator==( const UnitIterator<T>& other ) const { return m_punit == other.m_punit; }
};

template<typename T>
class UnitTraverser
{
private:
  T* m_begin;
  T* m_end;

public:
  UnitTraverser(                    ) : m_begin( nullptr ), m_end( nullptr ) { }
  UnitTraverser( T* _begin, T* _end ) : m_begin( _begin  ), m_end( _end    ) { }

  typedef T                     value_type;
  typedef size_t                size_type;
  typedef T&                    reference;
  typedef T const&              const_reference;
  typedef T*                    pointer;
  typedef T const*              const_pointer;
  typedef UnitIterator<T>       iterator;
  typedef UnitIterator<const T> const_iterator;

  iterator        begin()        { return UnitIterator<T>( m_begin ); }
  const_iterator  begin()  const { return UnitIterator<T>( m_begin ); }
  const_iterator  cbegin() const { return UnitIterator<T>( m_begin ); }
  iterator        end()          { return UnitIterator<T>( m_end   ); }
  const_iterator  end()    const { return UnitIterator<T>( m_end   ); }
  const_iterator  cend()   const { return UnitIterator<T>( m_end   ); }
};

typedef UnitTraverser<CodingUnit>     CUTraverser;
typedef UnitTraverser<PredictionUnit> PUTraverser;
typedef UnitTraverser<TransformUnit>  TUTraverser;

typedef UnitTraverser<const CodingUnit>     cCUTraverser;
typedef UnitTraverser<const PredictionUnit> cPUTraverser;
typedef UnitTraverser<const TransformUnit>  cTUTraverser;

#if MULTI_HYP_PRED
struct MEResult
{
  CodingUnit cu;
  PredictionUnit pu;
  uint32_t bits;
  Distortion cost;
  PelUnitBuf* predBuf;
  int8_t predBufIdx;
  MEResult() { predBuf = nullptr; predBufIdx = -1; }
};
typedef std::vector<MEResult> MEResultVec;
#endif

#if JVET_AG0152_SGPM_ITMP_IBC
struct SgpmInfo
{
  int sgpmSplitDir;
  int sgpmMode0;
  int sgpmMode1;
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  BvInfo   sgpmBv0;
  BvInfo   sgpmBv1;
#else
  Mv   sgpmBv0;
  Mv   sgpmBv1;
#endif
#if JVET_AJ0112_REGRESSION_SGPM
  bool isRegression;
  AffineBlendingModel blendModel;
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0), sgpmBv0(Mv(0, 0), 0), sgpmBv1(Mv(0, 0), 0), isRegression(false), blendModel(AffineBlendingModel(5, 1, 31)) {}
  SgpmInfo(const int sd, const int sm0, const int sm1, const BvInfo sbv0, const BvInfo sbv1, bool isR, AffineBlendingModel bM) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1), sgpmBv0(sbv0), sgpmBv1(sbv1), isRegression(isR), blendModel(bM) {}
#else
  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0), sgpmBv0(0, 0), sgpmBv1(0, 0), isRegression(false), blendModel(AffineBlendingModel(5, 1, 31)) {}
  SgpmInfo(const int sd, const int sm0, const int sm1, const Mv sbv0, const Mv sbv1, bool isR, AffineBlendingModel bM) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1), sgpmBv0(sbv0), sgpmBv1(sbv1), isRegression(isR), blendModel(bM){}
#endif
#else
#if JVET_AL0188_SGPM_FLIPAWARE_BV
  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0), sgpmBv0(Mv(0, 0), 0), sgpmBv1(Mv(0, 0), 0) {}
  SgpmInfo(const int sd, const int sm0, const int sm1, const BvInfo sbv0, const BvInfo sbv1) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1), sgpmBv0(sbv0), sgpmBv1(sbv1) {}
#else
  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0), sgpmBv0(0, 0), sgpmBv1(0, 0) {}
  SgpmInfo(const int sd, const int sm0, const int sm1, const Mv sbv0, const Mv sbv1) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1), sgpmBv0(sbv0), sgpmBv1(sbv1) {}
#endif
#endif

  SgpmInfo& operator=(const SgpmInfo& other)
  {
    sgpmSplitDir = other.sgpmSplitDir;
    sgpmMode0 = other.sgpmMode0;
    sgpmMode1 = other.sgpmMode1;
    sgpmBv0 = other.sgpmBv0;
    sgpmBv1 = other.sgpmBv1;
#if JVET_AJ0112_REGRESSION_SGPM
    isRegression = other.isRegression;
    blendModel = other.blendModel;
#endif
    return *this;
  }
};
#endif
#endif

