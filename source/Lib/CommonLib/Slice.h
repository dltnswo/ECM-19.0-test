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

/** \file     Slice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __SLICE__
#define __SLICE__

#include <cstring>
#include <list>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "Rom.h"
#include "ChromaFormat.h"
#include "Common.h"
#include "HRD.h"
#include <unordered_map>
#include "AlfParameters.h"
#include "Unit.h"
#if MULTI_HYP_PRED
#include "MotionInfo.h"
#endif

//! \ingroup CommonLib
//! \{
#include "CommonLib/MotionInfo.h"
#if NN_LF_UNIFIED
#include "CommonLib/NNFilterUnified.h"
#endif

struct MotionInfo;


struct Picture;
class Pic;
class TrQuant;
// ====================================================================================================================
// Constants
// ====================================================================================================================
class PreCalcValues;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
class Partitioner;
#endif

static const uint32_t REF_PIC_LIST_NUM_IDX=32;

typedef std::list<Picture*> PicList;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct DpbParameters
{
  int m_maxDecPicBuffering[MAX_TLAYER] = { 0 };
  int m_numReorderPics[MAX_TLAYER] = { 0 };
  int m_maxLatencyIncreasePlus1[MAX_TLAYER] = { 0 };
};

class ReferencePictureList
{
private:
  int   m_numberOfShorttermPictures;
  int   m_numberOfLongtermPictures;
  int   m_isLongtermRefPic[MAX_NUM_REF_PICS];
  int   m_refPicIdentifier[MAX_NUM_REF_PICS];  //This can be delta POC for STRP or POC LSB for LTRP
  int   m_POC[MAX_NUM_REF_PICS];
  int   m_numberOfActivePictures;
  bool  m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];
  int   m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  bool  m_ltrp_in_slice_header_flag;
  bool  m_interLayerPresentFlag;
  bool  m_isInterLayerRefPic[MAX_NUM_REF_PICS];
  int   m_interLayerRefPicIdx[MAX_NUM_REF_PICS];
  int   m_numberOfInterLayerPictures;

public:
  ReferencePictureList( const bool interLayerPicPresentFlag = false );
  virtual ~ReferencePictureList();

  void    setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx );
  int     getRefPicIdentifier(int idx) const;
  bool    isRefPicLongterm(int idx) const;

  void    setNumberOfShorttermPictures(int numberOfStrp);
  int     getNumberOfShorttermPictures() const;

  void    setNumberOfLongtermPictures(int numberOfLtrp);
  int     getNumberOfLongtermPictures() const;

  void    setLtrpInSliceHeaderFlag(bool flag) { m_ltrp_in_slice_header_flag = flag; }
  bool    getLtrpInSliceHeaderFlag() const { return m_ltrp_in_slice_header_flag; }

  void    setNumberOfInterLayerPictures( const int numberOfIlrp ) { m_numberOfInterLayerPictures = numberOfIlrp; }
  int     getNumberOfInterLayerPictures() const { return m_numberOfInterLayerPictures; }

  int     getNumRefEntries() const { return m_numberOfShorttermPictures + m_numberOfLongtermPictures + m_numberOfInterLayerPictures; }

  void    setPOC(int idx, int POC);
  int     getPOC(int idx) const;

  void    setNumberOfActivePictures(int numberOfLtrp);
  int     getNumberOfActivePictures() const;

  int     getDeltaPocMSBCycleLT(int i) const { return m_deltaPOCMSBCycleLT[i]; }
  void    setDeltaPocMSBCycleLT(int i, int x) { m_deltaPOCMSBCycleLT[i] = x; }
  bool    getDeltaPocMSBPresentFlag(int i) const { return m_deltaPocMSBPresentFlag[i]; }
  void    setDeltaPocMSBPresentFlag(int i, bool x) { m_deltaPocMSBPresentFlag[i] = x; }

  void    printRefPicInfo() const;

  bool      getInterLayerPresentFlag()                   const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                 { m_interLayerPresentFlag = b; }
  bool      isInterLayerRefPic( int idx )                const { return m_isInterLayerRefPic[idx]; }
  int       getInterLayerRefPicIdx( int idx )            const { return m_interLayerRefPicIdx[idx]; }
  void      setInterLayerRefPicIdx( int idx, int layerIdc )    { m_interLayerRefPicIdx[idx] = layerIdc; }
};

/// Reference Picture List set class
class RPLList
{
private:
  std::vector<ReferencePictureList> m_referencePictureLists;

public:
  RPLList() { }
  virtual                        ~RPLList() { }

  void                           create(int numberOfEntries) { m_referencePictureLists.resize(numberOfEntries); }
  void                           destroy() { }


  ReferencePictureList*          getReferencePictureList(int referencePictureListIdx) { return &m_referencePictureLists[referencePictureListIdx]; }
  const ReferencePictureList*    getReferencePictureList(int referencePictureListIdx) const { return &m_referencePictureLists[referencePictureListIdx]; }

  int                            getNumberOfReferencePictureLists() const { return int(m_referencePictureLists.size()); }
};

/// SCALING_LIST class
class ScalingList
{
public:
             ScalingList();
  virtual    ~ScalingList()                                                 { }
  
  int*       getScalingListAddress(uint32_t scalingListId)                    { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  const int* getScalingListAddress(uint32_t scalingListId) const              { return &(m_scalingListCoef[scalingListId][0]);            } //!< get matrix coefficient
  void       checkPredMode(uint32_t scalingListId);

  void       setRefMatrixId(uint32_t scalingListId, uint32_t u)               { m_refMatrixId[scalingListId] = u;                         } //!< set reference matrix ID
  uint32_t       getRefMatrixId(uint32_t scalingListId) const                 { return m_refMatrixId[scalingListId];                      } //!< get reference matrix ID

  static const int* getScalingListDefaultAddress(uint32_t scalinListId);                                                                           //!< get default matrix coefficient
  void       processDefaultMatrix(uint32_t scalinListId);

  void       setScalingListDC(uint32_t scalinListId, uint32_t u)              { m_scalingListDC[scalinListId] = u;                        } //!< set DC value
  int        getScalingListDC(uint32_t scalinListId) const                    { return m_scalingListDC[scalinListId];                     } //!< get DC value

  void       setScalingListCopyModeFlag(uint32_t scalinListId, bool bIsCopy)  { m_scalingListPredModeFlagIsCopy[scalinListId] = bIsCopy;  }
  bool       getScalingListCopyModeFlag(uint32_t scalinListId) const          { return m_scalingListPredModeFlagIsCopy[scalinListId];     } //getScalingListPredModeFlag
  void       processRefMatrix(uint32_t scalingListId, uint32_t refListId);

  int        lengthUvlc(int uiCode);
  int        lengthSvlc(int uiCode);
  void       CheckBestPredScalingList(int scalingListId, int predListIdx, int& BitsCount);
  void       codePredScalingList(int* scalingList, const int* scalingListPred, int scalingListDC, int scalingListPredDC, int scalinListId, int& bitsCost);
  void       codeScalingList(int* scalingList, int scalingListDC, int scalinListId, int& bitsCost);
  void       setScalingListPreditorModeFlag(uint32_t scalingListId, bool bIsPred) { m_scalingListPreditorModeFlag[scalingListId] = bIsPred; }
  bool       getScalingListPreditorModeFlag(uint32_t scalingListId) const { return m_scalingListPreditorModeFlag[scalingListId]; }
  bool       getChromaScalingListPresentFlag() const {return m_chromaScalingListPresentFlag;}
  void       setChromaScalingListPresentFlag( bool flag) { m_chromaScalingListPresentFlag = flag;}
  bool       isLumaScalingList( int scalingListId) const;
  void       checkDcOfMatrix();
  bool       xParseScalingList(const std::string &fileName);
  void       setDefaultScalingList();
  bool       isNotDefaultScalingList();

  bool operator==( const ScalingList& other )
  {
    if (memcmp(m_scalingListPredModeFlagIsCopy, other.m_scalingListPredModeFlagIsCopy, sizeof(m_scalingListPredModeFlagIsCopy)))
    {
      return false;
    }
    if( memcmp( m_scalingListDC, other.m_scalingListDC, sizeof( m_scalingListDC ) ) )
    {
      return false;
    }
    if( memcmp( m_refMatrixId, other.m_refMatrixId, sizeof( m_refMatrixId ) ) )
    {
      return false;
    }
    if( memcmp( m_scalingListCoef, other.m_scalingListCoef, sizeof( m_scalingListCoef ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const ScalingList& other )
  {
    return !( *this == other );
  }

private:
  void             outputScalingLists(std::ostream &os) const;
  bool             m_scalingListPredModeFlagIsCopy [30]; //!< reference list index
  int              m_scalingListDC                 [30]; //!< the DC value of the matrix coefficient for 16x16
  uint32_t         m_refMatrixId                   [30]; //!< RefMatrixID
  bool             m_scalingListPreditorModeFlag   [30]; //!< reference list index
  std::vector<int> m_scalingListCoef               [30]; //!< quantization matrix
  bool             m_chromaScalingListPresentFlag;
};

class ConstraintInfo
{
#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
  bool              m_gciPresentFlag;
#endif
#if !JVET_S0266_VUI_length
  bool              m_nonPackedConstraintFlag;
  bool              m_nonProjectedConstraintFlag;
#endif
#if JVET_Q0114_ASPECT5_GCI_FLAG
  bool              m_noRprConstraintFlag;
#endif
  bool              m_noResChangeInClvsConstraintFlag;
  bool              m_oneTilePerPicConstraintFlag;
  bool              m_picHeaderInSliceHeaderConstraintFlag;
  bool              m_oneSlicePerPicConstraintFlag;
#if JVET_S0113_S0195_GCI
  bool              m_noIdrRplConstraintFlag;
  bool              m_noRectSliceConstraintFlag;
  bool              m_oneSlicePerSubpicConstraintFlag;
  bool              m_noSubpicInfoConstraintFlag;
#else
  bool              m_oneSubpicPerPicConstraintFlag;
#endif
#if !JVET_S0138_GCI_PTL
  bool              m_frameOnlyConstraintFlag;
#endif
  bool              m_intraOnlyConstraintFlag;
  uint32_t          m_maxBitDepthConstraintIdc;
  int               m_maxChromaFormatConstraintIdc;
  bool              m_onePictureOnlyConstraintFlag;
  bool              m_lowerBitRateConstraintFlag;
#if !JVET_S0138_GCI_PTL
  bool              m_singleLayerConstraintFlag;
#endif
  bool              m_allLayersIndependentConstraintFlag;
  bool              m_noMrlConstraintFlag;
  bool              m_noIspConstraintFlag;
  bool              m_noMipConstraintFlag;
  bool              m_noLfnstConstraintFlag;
  bool              m_noMmvdConstraintFlag;
  bool              m_noSmvdConstraintFlag;
  bool              m_noProfConstraintFlag;
  bool              m_noPaletteConstraintFlag;
  bool              m_noActConstraintFlag;
  bool              m_noLmcsConstraintFlag;

#if JVET_S0050_GCI
  bool              m_noExplicitScaleListConstraintFlag;
  bool              m_noVirtualBoundaryConstraintFlag;
#endif
#if JVET_S0058_GCI
  bool              m_noMttConstraintFlag;
#endif
#if JVET_R0341_GCI
  bool              m_noChromaQpOffsetConstraintFlag;
#endif
  bool              m_noQtbttDualTreeIntraConstraintFlag;
#if JVET_S0066_GCI
  int               m_maxLog2CtuSizeConstraintIdc;
#endif
  bool              m_noPartitionConstraintsOverrideConstraintFlag;
  bool              m_noSaoConstraintFlag;
#if JVET_W0066_CCSAO
  bool              m_noCCSaoConstraintFlag;
#endif
  bool              m_noAlfConstraintFlag;
  bool              m_noCCAlfConstraintFlag;
#if JVET_S0058_GCI
  bool              m_noWeightedPredictionConstraintFlag;
#endif
  bool              m_noRefWraparoundConstraintFlag;
  bool              m_noTemporalMvpConstraintFlag;
  bool              m_noSbtmvpConstraintFlag;
  bool              m_noAmvrConstraintFlag;
  bool              m_noBdofConstraintFlag;
  bool              m_noDmvrConstraintFlag;
  bool              m_noCclmConstraintFlag;
  bool              m_noMtsConstraintFlag;
  bool              m_noSbtConstraintFlag;
  bool              m_noAffineMotionConstraintFlag;
  bool              m_noBcwConstraintFlag;
  bool              m_noIbcConstraintFlag;
#if ENABLE_DIMD
  bool              m_noDimdConstraintFlag;
#endif
#if JVET_W0123_TIMD_FUSION
  bool              m_noTimdConstraintFlag;
#endif
#if JVET_AB0155_SGPM
  bool              m_noSgpmConstraintFlag;
#endif
#if JVET_AD0082_TMRL_CONFIG
  bool              m_noTmrlConstraintFlag;
#endif
#if JVET_AG0058_EIP
  bool              m_noEipConstraintFlag;
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  bool              m_noIntraPredBfConstraintFlag;
#endif
#if ENABLE_OBMC
  bool              m_noObmcConstraintFlag;
#endif
  bool              m_noCiipConstraintFlag;
  bool              m_noGeoConstraintFlag;
  bool              m_noLadfConstraintFlag;
  bool              m_noTransformSkipConstraintFlag;
#if JVET_S0066_GCI
  bool              m_noLumaTransformSize64ConstraintFlag;
#endif
  bool              m_noBDPCMConstraintFlag;
  bool              m_noJointCbCrConstraintFlag;
  bool              m_noQpDeltaConstraintFlag;
  bool              m_noDepQuantConstraintFlag;
  bool              m_noSignDataHidingConstraintFlag;
  bool              m_noMixedNaluTypesInPicConstraintFlag;
  bool              m_noTrailConstraintFlag;
  bool              m_noStsaConstraintFlag;
  bool              m_noRaslConstraintFlag;
  bool              m_noRadlConstraintFlag;
  bool              m_noIdrConstraintFlag;
  bool              m_noCraConstraintFlag;
  bool              m_noGdrConstraintFlag;
  bool              m_noApsConstraintFlag;

public:
  ConstraintInfo()
#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
    : m_gciPresentFlag(true)
#if !JVET_S0266_VUI_length
    , m_nonPackedConstraintFlag(false)
    , m_nonProjectedConstraintFlag(false)
#endif
#else
    : m_nonPackedConstraintFlag (false)
    , m_nonProjectedConstraintFlag(false)
#endif
#if JVET_Q0114_ASPECT5_GCI_FLAG
    , m_noRprConstraintFlag(false)
#endif
    , m_noResChangeInClvsConstraintFlag(false)
    , m_oneTilePerPicConstraintFlag(false)
    , m_picHeaderInSliceHeaderConstraintFlag(false)
    , m_oneSlicePerPicConstraintFlag(false)
#if JVET_S0113_S0195_GCI
    , m_noIdrRplConstraintFlag(false)
    , m_noRectSliceConstraintFlag(false)
    , m_oneSlicePerSubpicConstraintFlag(false)
    , m_noSubpicInfoConstraintFlag(false)
#else
    , m_oneSubpicPerPicConstraintFlag(false)
#endif
#if !JVET_S0138_GCI_PTL
    , m_frameOnlyConstraintFlag  (false)
#endif
    , m_intraOnlyConstraintFlag  (false)
#if JVET_S0094_CHROMAFORMAT_BITDEPTH_CONSTRAINT
    , m_maxBitDepthConstraintIdc  (  16)
    , m_maxChromaFormatConstraintIdc(CHROMA_444)
#else
    , m_maxBitDepthConstraintIdc  (  0)
    , m_maxChromaFormatConstraintIdc(CHROMA_420)
#endif
    , m_onePictureOnlyConstraintFlag (false)
    , m_lowerBitRateConstraintFlag (false )
#if !JVET_S0138_GCI_PTL
    , m_singleLayerConstraintFlag(false)
#endif
    , m_allLayersIndependentConstraintFlag(false)
    , m_noMrlConstraintFlag(false)
    , m_noIspConstraintFlag(false)
    , m_noMipConstraintFlag(false)
    , m_noLfnstConstraintFlag(false)
    , m_noMmvdConstraintFlag(false)
    , m_noSmvdConstraintFlag(false)
    , m_noProfConstraintFlag(false)
    , m_noPaletteConstraintFlag(false)
    , m_noActConstraintFlag(false)
    , m_noLmcsConstraintFlag(false)
#if JVET_S0050_GCI
    , m_noExplicitScaleListConstraintFlag(false)
    , m_noVirtualBoundaryConstraintFlag(false)
#endif
#if JVET_S0058_GCI
    , m_noMttConstraintFlag(false)
#endif
#if JVET_R0341_GCI
    , m_noChromaQpOffsetConstraintFlag(false)
#endif
    , m_noQtbttDualTreeIntraConstraintFlag(false)
#if JVET_S0066_GCI
    , m_maxLog2CtuSizeConstraintIdc(8)
#endif
    , m_noPartitionConstraintsOverrideConstraintFlag(false)
    , m_noSaoConstraintFlag      (false)
#if JVET_W0066_CCSAO
    , m_noCCSaoConstraintFlag      (false)
#endif
    , m_noAlfConstraintFlag      (false)
    , m_noCCAlfConstraintFlag      (false)
#if JVET_S0058_GCI
    , m_noWeightedPredictionConstraintFlag(false)
#endif
    , m_noRefWraparoundConstraintFlag(false)
    , m_noTemporalMvpConstraintFlag(false)
    , m_noSbtmvpConstraintFlag   (false)
    , m_noAmvrConstraintFlag     (false)
    , m_noBdofConstraintFlag     (false)
    , m_noDmvrConstraintFlag     (false)
    , m_noCclmConstraintFlag     (false)
    , m_noMtsConstraintFlag      (false)
    , m_noSbtConstraintFlag      (false)
    , m_noAffineMotionConstraintFlag(false)
    , m_noBcwConstraintFlag      (false)
    , m_noIbcConstraintFlag      (false)
#if ENABLE_DIMD
    , m_noDimdConstraintFlag     (false)
#endif
#if JVET_W0123_TIMD_FUSION
    , m_noTimdConstraintFlag     (false)
#endif
#if ENABLE_OBMC
    , m_noObmcConstraintFlag     (false)
#endif
    , m_noCiipConstraintFlag  (false)
    , m_noGeoConstraintFlag      (false)
    , m_noLadfConstraintFlag     (false)
    , m_noTransformSkipConstraintFlag(false)
#if JVET_S0066_GCI
    , m_noLumaTransformSize64ConstraintFlag(false)
#endif
    , m_noBDPCMConstraintFlag    (false)
    , m_noJointCbCrConstraintFlag (false)
    , m_noQpDeltaConstraintFlag  (false)
    , m_noDepQuantConstraintFlag (false)
    , m_noSignDataHidingConstraintFlag(false)
    , m_noMixedNaluTypesInPicConstraintFlag(false)
    , m_noTrailConstraintFlag (false)
    , m_noStsaConstraintFlag (false)
    , m_noRaslConstraintFlag (false)
    , m_noRadlConstraintFlag (false)
    , m_noIdrConstraintFlag (false)
    , m_noCraConstraintFlag (false)
    , m_noGdrConstraintFlag (false)
    , m_noApsConstraintFlag (false)
  {}


#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
  bool          getGciPresentFlag() const { return m_gciPresentFlag; }
  void          setGciPresentFlag(bool b) { m_gciPresentFlag = b; }
#endif

#if !JVET_S0266_VUI_length
  bool          getNonPackedConstraintFlag() const { return m_nonPackedConstraintFlag; }
  void          setNonPackedConstraintFlag(bool b) { m_nonPackedConstraintFlag = b; }
#endif
#if !JVET_S0138_GCI_PTL
  bool          getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  void          setFrameOnlyConstraintFlag(bool b) { m_frameOnlyConstraintFlag = b; }
#endif
  uint32_t      getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
  void          setMaxBitDepthConstraintIdc(uint32_t bitDepth) { m_maxBitDepthConstraintIdc = bitDepth; }

  int           getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
  void          setMaxChromaFormatConstraintIdc(int fmt) { m_maxChromaFormatConstraintIdc = fmt; }

#if !JVET_S0266_VUI_length
  bool          getNonProjectedConstraintFlag() const { return m_nonProjectedConstraintFlag; }
  void          setNonProjectedConstraintFlag(bool b) { m_nonProjectedConstraintFlag = b; }
#endif

#if JVET_Q0114_ASPECT5_GCI_FLAG
  bool          getNoRprConstraintFlag() const { return m_noRprConstraintFlag; }
  void          setNoRprConstraintFlag(bool b) { m_noRprConstraintFlag = b; }
#endif

  bool          getNoResChangeInClvsConstraintFlag() const { return m_noResChangeInClvsConstraintFlag; }
  void          setNoResChangeInClvsConstraintFlag(bool b) { m_noResChangeInClvsConstraintFlag = b; }

  bool          getOneTilePerPicConstraintFlag() const { return m_oneTilePerPicConstraintFlag; }
  void          setOneTilePerPicConstraintFlag(bool b) { m_oneTilePerPicConstraintFlag = b; }

  bool          getPicHeaderInSliceHeaderConstraintFlag() const { return m_picHeaderInSliceHeaderConstraintFlag; }
  void          setPicHeaderInSliceHeaderConstraintFlag(bool b) { m_picHeaderInSliceHeaderConstraintFlag = b; }

  bool          getOneSlicePerPicConstraintFlag() const { return m_oneSlicePerPicConstraintFlag; }
  void          setOneSlicePerPicConstraintFlag(bool b) { m_oneSlicePerPicConstraintFlag = b; }

#if JVET_S0113_S0195_GCI
  bool          getNoIdrRplConstraintFlag() const          { return m_noIdrRplConstraintFlag; }
  void          setNoIdrRplConstraintFlag(bool b)          { m_noIdrRplConstraintFlag = b; }

  bool          getNoRectSliceConstraintFlag() const       { return m_noRectSliceConstraintFlag; }
  void          setNoRectSliceConstraintFlag(bool b)       { m_noRectSliceConstraintFlag = b; }

  bool          getOneSlicePerSubpicConstraintFlag() const { return m_oneSlicePerSubpicConstraintFlag; }
  void          setOneSlicePerSubpicConstraintFlag(bool b) { m_oneSlicePerSubpicConstraintFlag = b; }

  bool          getNoSubpicInfoConstraintFlag() const      { return m_noSubpicInfoConstraintFlag; }
  void          setNoSubpicInfoConstraintFlag(bool b)      { m_noSubpicInfoConstraintFlag = b; }
#else
  bool          getOneSubpicPerPicConstraintFlag() const { return m_oneSubpicPerPicConstraintFlag; }
  void          setOneSubpicPerPicConstraintFlag(bool b) { m_oneSubpicPerPicConstraintFlag = b; }
#endif

  bool          getIntraOnlyConstraintFlag() const { return m_intraOnlyConstraintFlag; }
  void          setIntraOnlyConstraintFlag(bool b) { m_intraOnlyConstraintFlag = b; }

  bool          getOnePictureOnlyConstraintFlag() const { return m_onePictureOnlyConstraintFlag; }
  void          setOnePictureOnlyConstraintFlag(bool b) { m_onePictureOnlyConstraintFlag = b; }

  bool          getLowerBitRateConstraintFlag() const { return m_lowerBitRateConstraintFlag; }
  void          setLowerBitRateConstraintFlag(bool b) { m_lowerBitRateConstraintFlag = b; }
#if !JVET_S0138_GCI_PTL
  bool          getSingleLayerConstraintFlag() const { return m_singleLayerConstraintFlag; }
  void          setSingleLayerConstraintFlag(bool b) { m_singleLayerConstraintFlag = b; }
#endif
  bool          getAllLayersIndependentConstraintFlag() const { return m_allLayersIndependentConstraintFlag; }
  void          setAllLayersIndependentConstraintFlag(bool b) { m_allLayersIndependentConstraintFlag = b; }
  bool          getNoMrlConstraintFlag() const { return m_noMrlConstraintFlag; }
  void          setNoMrlConstraintFlag(bool b) { m_noMrlConstraintFlag = b; }
  bool          getNoIspConstraintFlag() const { return m_noIspConstraintFlag; }
  void          setNoIspConstraintFlag(bool b) { m_noIspConstraintFlag = b; }
  bool          getNoMipConstraintFlag() const { return m_noMipConstraintFlag; }
  void          setNoMipConstraintFlag(bool b) { m_noMipConstraintFlag = b; }
  bool          getNoLfnstConstraintFlag() const { return m_noLfnstConstraintFlag; }
  void          setNoLfnstConstraintFlag(bool b) { m_noLfnstConstraintFlag = b; }
  bool          getNoMmvdConstraintFlag() const { return m_noMmvdConstraintFlag; }
  void          setNoMmvdConstraintFlag(bool b) { m_noMmvdConstraintFlag = b; }
  bool          getNoSmvdConstraintFlag() const { return m_noSmvdConstraintFlag; }
  void          setNoSmvdConstraintFlag(bool b) { m_noSmvdConstraintFlag = b; }
  bool          getNoProfConstraintFlag() const { return m_noProfConstraintFlag; }
  void          setNoProfConstraintFlag(bool b) { m_noProfConstraintFlag = b; }
  bool          getNoPaletteConstraintFlag() const { return m_noPaletteConstraintFlag; }
  void          setNoPaletteConstraintFlag(bool b) { m_noPaletteConstraintFlag = b; }
  bool          getNoActConstraintFlag() const { return m_noActConstraintFlag; }
  void          setNoActConstraintFlag(bool b) { m_noActConstraintFlag = b; }
  bool          getNoLmcsConstraintFlag() const { return m_noLmcsConstraintFlag; }
  void          setNoLmcsConstraintFlag(bool b) { m_noLmcsConstraintFlag = b; }
#if JVET_S0050_GCI
  bool          getNoExplicitScaleListConstraintFlag() const { return m_noExplicitScaleListConstraintFlag; }
  void          setNoExplicitScaleListConstraintFlag(bool b) { m_noExplicitScaleListConstraintFlag = b; }
  bool          getNoVirtualBoundaryConstraintFlag() const { return m_noVirtualBoundaryConstraintFlag; }
  void          setNoVirtualBoundaryConstraintFlag(bool b) { m_noVirtualBoundaryConstraintFlag = b; }
#endif
#if JVET_S0058_GCI
  bool          getNoMttConstraintFlag() const { return m_noMttConstraintFlag; }
  void          setNoMttConstraintFlag(bool bVal) { m_noMttConstraintFlag = bVal; }
#endif
#if JVET_R0341_GCI
  bool          getNoChromaQpOffsetConstraintFlag() const { return m_noChromaQpOffsetConstraintFlag; }
  void          setNoChromaQpOffsetConstraintFlag(bool b) { m_noChromaQpOffsetConstraintFlag = b; }
#endif
  bool          getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
  void          setNoQtbttDualTreeIntraConstraintFlag(bool bVal) { m_noQtbttDualTreeIntraConstraintFlag = bVal; }
#if JVET_S0066_GCI
  int           getMaxLog2CtuSizeConstraintIdc() const { return m_maxLog2CtuSizeConstraintIdc; }
  void          setMaxLog2CtuSizeConstraintIdc(int idc) { m_maxLog2CtuSizeConstraintIdc = idc; }
#endif
  bool          getNoPartitionConstraintsOverrideConstraintFlag() const { return m_noPartitionConstraintsOverrideConstraintFlag; }
  void          setNoPartitionConstraintsOverrideConstraintFlag(bool bVal) { m_noPartitionConstraintsOverrideConstraintFlag = bVal; }
  bool          getNoSaoConstraintFlag() const { return m_noSaoConstraintFlag; }
  void          setNoSaoConstraintFlag(bool bVal) { m_noSaoConstraintFlag = bVal; }
#if JVET_W0066_CCSAO
  bool          getNoCCSaoConstraintFlag() const { return m_noCCSaoConstraintFlag; }
  void          setNoCCSaoConstraintFlag(bool val) { m_noCCSaoConstraintFlag = val; }
#endif
  bool          getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; }
  void          setNoAlfConstraintFlag(bool bVal) { m_noAlfConstraintFlag = bVal; }
  bool          getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  void          setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
  bool          getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  void          setNoJointCbCrConstraintFlag(bool bVal) { m_noJointCbCrConstraintFlag = bVal; }
#if JVET_S0058_GCI
  bool          getNoWeightedPredictionConstraintFlag() const { return m_noWeightedPredictionConstraintFlag; }
  void          setNoWeightedPredictionConstraintFlag(bool bVal) { m_noWeightedPredictionConstraintFlag = bVal; }
#endif
  bool          getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  void          setNoRefWraparoundConstraintFlag(bool bVal) { m_noRefWraparoundConstraintFlag = bVal; }
  bool          getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  void          setNoTemporalMvpConstraintFlag(bool bVal) { m_noTemporalMvpConstraintFlag = bVal; }
  bool          getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  void          setNoSbtmvpConstraintFlag(bool bVal) { m_noSbtmvpConstraintFlag = bVal; }
  bool          getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  void          setNoAmvrConstraintFlag(bool bVal) { m_noAmvrConstraintFlag = bVal; }
  bool          getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  void          setNoBdofConstraintFlag(bool bVal) { m_noBdofConstraintFlag = bVal; }
  bool          getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  void          setNoDmvrConstraintFlag(bool bVal) { m_noDmvrConstraintFlag = bVal; }
  bool          getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  void          setNoCclmConstraintFlag(bool bVal) { m_noCclmConstraintFlag = bVal; }
  bool          getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  void          setNoMtsConstraintFlag(bool bVal) { m_noMtsConstraintFlag = bVal; }
  bool          getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  void          setNoSbtConstraintFlag(bool bVal) { m_noSbtConstraintFlag = bVal; }
  bool          getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  void          setNoAffineMotionConstraintFlag(bool bVal) { m_noAffineMotionConstraintFlag = bVal; }
  bool          getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  void          setNoBcwConstraintFlag(bool bVal) { m_noBcwConstraintFlag = bVal; }
  bool          getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  void          setNoIbcConstraintFlag(bool bVal) { m_noIbcConstraintFlag = bVal; }
#if ENABLE_DIMD
  bool          getNoDimdConstraintFlag() const { return m_noDimdConstraintFlag; }
  void          setNoDimdConstraintFlag(bool bVal) { m_noDimdConstraintFlag = bVal; }
#endif
#if JVET_W0123_TIMD_FUSION
  bool          getNoTimdConstraintFlag() const { return m_noTimdConstraintFlag; }
  void          setNoTimdConstraintFlag(bool bVal) { m_noTimdConstraintFlag = bVal; }
#endif
#if JVET_AB0155_SGPM
  bool          getNoSgpmConstraintFlag() const { return m_noSgpmConstraintFlag; }
  void          setNoSgpmConstraintFlag(bool bVal) { m_noSgpmConstraintFlag = bVal; }
#endif
#if JVET_AD0082_TMRL_CONFIG
  bool          getNoTmrlConstraintFlag() const { return m_noTmrlConstraintFlag; }
  void          setNoTmrlConstraintFlag(bool bVal) { m_noTmrlConstraintFlag = bVal; }
#endif
#if JVET_AG0058_EIP
  bool          getNoEipConstraintFlag() const { return m_noEipConstraintFlag; }
  void          setNoEipConstraintFlag(bool bVal) { m_noEipConstraintFlag = bVal; }
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  bool          getNoIntraPredBfConstraintFlag() const { return m_noIntraPredBfConstraintFlag; }
  void          setNoIntraPredBfConstraintFlag(bool bVal) { m_noIntraPredBfConstraintFlag = bVal; }
#endif
#if ENABLE_OBMC
  bool          getNoObmcConstraintFlag() const { return m_noObmcConstraintFlag; }
  void          setNoObmcConstraintFlag(bool bVal) { m_noObmcConstraintFlag = bVal; }
#endif
  bool          getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  void          setNoCiipConstraintFlag(bool bVal) { m_noCiipConstraintFlag = bVal; }
  bool          getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  void          setNoGeoConstraintFlag(bool bVal) { m_noGeoConstraintFlag = bVal; }
  bool          getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  void          setNoLadfConstraintFlag(bool bVal) { m_noLadfConstraintFlag = bVal; }
  bool          getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  void          setNoTransformSkipConstraintFlag(bool bVal) { m_noTransformSkipConstraintFlag = bVal; }
#if JVET_S0066_GCI
  bool          getNoLumaTransformSize64ConstraintFlag() const { return m_noLumaTransformSize64ConstraintFlag; }
  void          setNoLumaTransformSize64ConstraintFlag(bool bVal) { m_noLumaTransformSize64ConstraintFlag = bVal; }
#endif
  bool          getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  void          setNoBDPCMConstraintFlag(bool bVal) { m_noBDPCMConstraintFlag = bVal; }
  bool          getNoQpDeltaConstraintFlag() const { return m_noQpDeltaConstraintFlag; }
  void          setNoQpDeltaConstraintFlag(bool bVal) { m_noQpDeltaConstraintFlag = bVal; }
  bool          getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  void          setNoDepQuantConstraintFlag(bool bVal) { m_noDepQuantConstraintFlag = bVal; }
  bool          getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  void          setNoSignDataHidingConstraintFlag(bool bVal) { m_noSignDataHidingConstraintFlag = bVal; }
  bool          getNoMixedNaluTypesInPicConstraintFlag() const    { return m_noMixedNaluTypesInPicConstraintFlag; }
  void          setNoMixedNaluTypesInPicConstraintFlag(bool bVal) { m_noMixedNaluTypesInPicConstraintFlag = bVal; }
  bool          getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  void          setNoTrailConstraintFlag(bool bVal) { m_noTrailConstraintFlag = bVal; }
  bool          getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  void          setNoStsaConstraintFlag(bool bVal) { m_noStsaConstraintFlag = bVal; }
  bool          getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  void          setNoRaslConstraintFlag(bool bVal) { m_noRaslConstraintFlag = bVal; }
  bool          getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  void          setNoRadlConstraintFlag(bool bVal) { m_noRadlConstraintFlag = bVal; }
  bool          getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  void          setNoIdrConstraintFlag(bool bVal) { m_noIdrConstraintFlag = bVal; }
  bool          getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  void          setNoCraConstraintFlag(bool bVal) { m_noCraConstraintFlag = bVal; }
  bool          getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  void          setNoGdrConstraintFlag(bool bVal) { m_noGdrConstraintFlag = bVal; }
  bool          getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  void          setNoApsConstraintFlag(bool bVal) { m_noApsConstraintFlag = bVal; }

  friend bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2);
  friend bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2);
};

class ProfileTierLevel
{
  Level::Tier       m_tierFlag;
  Profile::Name     m_profileIdc;
  uint8_t           m_numSubProfile;
  std::vector<uint32_t>          m_subProfileIdc;
  Level::Name       m_levelIdc;
#if JVET_S0138_GCI_PTL
  bool              m_frameOnlyConstraintFlag;
  bool              m_multiLayerEnabledFlag;
#endif
  ConstraintInfo    m_constraintInfo;
  bool              m_subLayerLevelPresentFlag[MAX_TLAYER - 1];
  Level::Name       m_subLayerLevelIdc[MAX_TLAYER];

public:
                ProfileTierLevel();

  Level::Tier   getTierFlag() const                         { return m_tierFlag;                    }
  void          setTierFlag(Level::Tier x)                  { m_tierFlag = x;                       }

  Profile::Name getProfileIdc() const                       { return m_profileIdc;                  }
  void          setProfileIdc(Profile::Name x)              { m_profileIdc = x;                     }

  uint32_t      getSubProfileIdc(int i) const               { return m_subProfileIdc[i]; }
  void          setSubProfileIdc(int i, uint32_t x)         { m_subProfileIdc[i] = x; }

  uint8_t       getNumSubProfile() const                    { return m_numSubProfile; }
  void          setNumSubProfile(uint8_t x)                 { m_numSubProfile = x; m_subProfileIdc.resize(m_numSubProfile); }

  Level::Name   getLevelIdc() const                         { return m_levelIdc;                    }
  void          setLevelIdc(Level::Name x)                  { m_levelIdc = x;                       }

#if JVET_S0138_GCI_PTL
  bool                    getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  void                    setFrameOnlyConstraintFlag(bool x) { m_frameOnlyConstraintFlag = x; }

  bool                    getMultiLayerEnabledFlag() const { return m_multiLayerEnabledFlag; }
  void                    setMultiLayerEnabledFlag(bool x) { m_multiLayerEnabledFlag = x; }
#endif

  ConstraintInfo*         getConstraintInfo()              { return &m_constraintInfo; }
  const ConstraintInfo*   getConstraintInfo() const        { return &m_constraintInfo; }

  bool                    getSubLayerLevelPresentFlag(int i) const     { return m_subLayerLevelPresentFlag[i];   }
  void                    setSubLayerLevelPresentFlag(int i, bool x)   { m_subLayerLevelPresentFlag[i] = x;      }

  Level::Name             getSubLayerLevelIdc(int i) const             { return m_subLayerLevelIdc[i];   }
  void                    setSubLayerLevelIdc(int i, Level::Name x)    { m_subLayerLevelIdc[i] = x;      }
  friend bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
  friend bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2);
};



class SliceReshapeInfo
{
public:
  bool      sliceReshaperEnableFlag;
  bool      sliceReshaperModelPresentFlag;
  unsigned  enableChromaAdj;
  uint32_t  reshaperModelMinBinIdx;
  uint32_t  reshaperModelMaxBinIdx;
  int       reshaperModelBinCWDelta[PIC_CODE_CW_BINS];
  int       maxNbitsNeededDeltaCW;
  int       chrResScalingOffset;
  void      setUseSliceReshaper(bool b)                                { sliceReshaperEnableFlag = b;            }
  bool      getUseSliceReshaper() const                                { return sliceReshaperEnableFlag;         }
  void      setSliceReshapeModelPresentFlag(bool b)                    { sliceReshaperModelPresentFlag = b;      }
  bool      getSliceReshapeModelPresentFlag() const                    { return   sliceReshaperModelPresentFlag; }
  void      setSliceReshapeChromaAdj(unsigned adj)                     { enableChromaAdj = adj;                  }
  unsigned  getSliceReshapeChromaAdj() const                           { return enableChromaAdj;                 }

  bool operator==( const SliceReshapeInfo& other )
  {
    if( sliceReshaperEnableFlag != other.sliceReshaperEnableFlag )
    {
      return false;
    }
    if( sliceReshaperModelPresentFlag != other.sliceReshaperModelPresentFlag )
    {
      return false;
    }
    if( enableChromaAdj != other.enableChromaAdj )
    {
      return false;
    }
    if( reshaperModelMinBinIdx != other.reshaperModelMinBinIdx )
    {
      return false;
    }
    if( reshaperModelMaxBinIdx != other.reshaperModelMaxBinIdx )
    {
      return false;
    }
    if( maxNbitsNeededDeltaCW != other.maxNbitsNeededDeltaCW )
    {
      return false;
    }
    if (chrResScalingOffset != other.chrResScalingOffset)
    {
      return false;
    }
    if( memcmp( reshaperModelBinCWDelta, other.reshaperModelBinCWDelta, sizeof( reshaperModelBinCWDelta ) ) )
    {
      return false;
    }

    return true;
  }

  bool operator!=( const SliceReshapeInfo& other )
  {
    return !( *this == other );
  }
};

struct ReshapeCW
{
  std::vector<uint32_t> binCW;
  int       updateCtrl;
  int       adpOption;
  uint32_t  initialCW;
  int rspPicSize;
  int rspFps;
  int rspBaseQP;
  int rspTid;
  int rspSliceQP;
  int rspFpsToIp;
};

struct ChromaQpAdj
{
  union
  {
    struct {
      int CbOffset;
      int CrOffset;
      int JointCbCrOffset;
    } comp;
    int offset[3];
  } u;
};
struct ChromaQpMappingTableParams {
  int               m_qpBdOffset;
  bool              m_sameCQPTableForAllChromaFlag;
  int               m_numQpTables;
  int               m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES];
  int               m_numPtsInCQPTableMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpInValMinus1[MAX_NUM_CQP_MAPPING_TABLES];
  std::vector<int>  m_deltaQpOutVal[MAX_NUM_CQP_MAPPING_TABLES];

  ChromaQpMappingTableParams()
  {
    m_qpBdOffset = 12;
    m_sameCQPTableForAllChromaFlag = true;
    m_numQpTables = 1;
    m_numPtsInCQPTableMinus1[0] = 0;
    m_qpTableStartMinus26[0] = 0;
    m_deltaQpInValMinus1[0] = { 0 };
    m_deltaQpOutVal[0] = { 0 };
  }

  void      setSameCQPTableForAllChromaFlag(bool b)                             { m_sameCQPTableForAllChromaFlag = b; }
  bool      getSameCQPTableForAllChromaFlag()                             const { return m_sameCQPTableForAllChromaFlag; }
  void      setNumQpTables(int n)                                     { m_numQpTables = n; }
  int       getNumQpTables()                                     const { return m_numQpTables; }
  void      setQpTableStartMinus26(int tableIdx, int n)                         { m_qpTableStartMinus26[tableIdx] = n; }
  int       getQpTableStartMinus26(int tableIdx)                          const { return m_qpTableStartMinus26[tableIdx]; }
  void      setNumPtsInCQPTableMinus1(int tableIdx, int n)                      { m_numPtsInCQPTableMinus1[tableIdx] = n; }
  int       getNumPtsInCQPTableMinus1(int tableIdx)                       const { return m_numPtsInCQPTableMinus1[tableIdx]; }
  void      setDeltaQpInValMinus1(int tableIdx, std::vector<int> &inVals)       { m_deltaQpInValMinus1[tableIdx] = inVals; }
  void      setDeltaQpInValMinus1(int tableIdx, int idx, int n)                 { m_deltaQpInValMinus1[tableIdx][idx] = n; }
  int       getDeltaQpInValMinus1(int tableIdx, int idx)                  const { return m_deltaQpInValMinus1[tableIdx][idx]; }
  void      setDeltaQpOutVal(int tableIdx, std::vector<int> &outVals)           { m_deltaQpOutVal[tableIdx] = outVals; }
  void      setDeltaQpOutVal(int tableIdx, int idx, int n)                      { m_deltaQpOutVal[tableIdx][idx] = n; }
  int       getDeltaQpOutVal(int tableIdx, int idx)                       const { return m_deltaQpOutVal[tableIdx][idx]; }
};
struct ChromaQpMappingTable : ChromaQpMappingTableParams
{
  std::map<int, int> m_chromaQpMappingTables[MAX_NUM_CQP_MAPPING_TABLES];

  int       getMappedChromaQpValue(ComponentID compID, const int qpVal)  const { return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal); }
  void      derivedChromaQPMappingTables();
  void      setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset);
};

class SliceMap
{
private:
  uint32_t               m_sliceID;                           //!< slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t               m_numTilesInSlice;                   //!< number of tiles in slice (raster-scan slices only)
  uint32_t               m_numCtuInSlice;                     //!< number of CTUs in the slice
  std::vector<uint32_t>  m_ctuAddrInSlice;                    //!< raster-scan addresses of all the CTUs in the slice

public:
  SliceMap();
  virtual ~SliceMap();

  void                   setSliceID( uint32_t u )             { m_sliceID = u;            }
  uint32_t               getSliceID() const                   { return m_sliceID;         }
  void                   setNumTilesInSlice( uint32_t u )     { m_numTilesInSlice = u;    }
  uint32_t               getNumTilesInSlice() const           { return m_numTilesInSlice; }
  void                   setNumCtuInSlice( uint32_t u )       { m_numCtuInSlice = u;      }
  uint32_t               getNumCtuInSlice() const             { return m_numCtuInSlice;   }
  std::vector<uint32_t>  getCtuAddrList( ) const              { return m_ctuAddrInSlice;  }
  uint32_t               getCtuAddrInSlice( int idx ) const   { CHECK(idx >= m_ctuAddrInSlice.size(), "CTU index exceeds number of CTUs in slice."); return m_ctuAddrInSlice[idx]; }
  void                   pushToCtuAddrInSlice( uint32_t u )   { m_ctuAddrInSlice.push_back(u); m_numCtuInSlice++;}

  void  initSliceMap()
  {
    m_sliceID = 0;
    m_numTilesInSlice = 0;
    m_numCtuInSlice = 0;
    m_ctuAddrInSlice.clear();
  }

  void  addCtusToSlice( uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY )
  {
    CHECK( startX >= stopX || startY >= stopY, "Invalid slice definition");
    for( uint32_t ctbY = startY; ctbY < stopY; ctbY++ )
    {
      for( uint32_t ctbX = startX; ctbX < stopX; ctbX++ )
      {
        m_ctuAddrInSlice.push_back( ctbY * picWidthInCtbsY + ctbX );
        m_numCtuInSlice++;
      }
    }
  }
};

class RectSlice
{
private:
  uint32_t         m_tileIdx;                           //!< tile index corresponding to the first CTU in the slice
  uint32_t         m_sliceWidthInTiles;                 //!< slice width in units of tiles
  uint32_t         m_sliceHeightInTiles;                //!< slice height in units of tiles
  uint32_t         m_numSlicesInTile;                   //!< number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t         m_sliceHeightInCtu;                  //!< slice height in units of CTUs for the special case of multiple slices inside a single tile

public:
  RectSlice();
  virtual ~RectSlice();

  void             setSliceWidthInTiles( uint32_t u )   { m_sliceWidthInTiles = u;      }
  uint32_t         getSliceWidthInTiles( ) const        { return  m_sliceWidthInTiles;  }
  void             setSliceHeightInTiles( uint32_t u )  { m_sliceHeightInTiles = u;     }
  uint32_t         getSliceHeightInTiles( ) const       { return  m_sliceHeightInTiles; }
  void             setNumSlicesInTile( uint32_t u )     { m_numSlicesInTile = u;        }
  uint32_t         getNumSlicesInTile( ) const          { return  m_numSlicesInTile;    }
  void             setSliceHeightInCtu( uint32_t u )    { m_sliceHeightInCtu = u;       }
  uint32_t         getSliceHeightInCtu( ) const         { return  m_sliceHeightInCtu;   }
  void             setTileIdx( uint32_t u )             { m_tileIdx = u;                }
  uint32_t         getTileIdx( ) const                  { return  m_tileIdx;            }

};

class SubPic
{
private:
  uint32_t         m_subPicID;                                  //!< ID of subpicture
  uint32_t         m_subPicIdx;                                 //!< Index of subpicture
  uint32_t         m_numCTUsInSubPic;                           //!< number of CTUs contained in this sub-picture
  uint32_t         m_subPicCtuTopLeftX;                         //!< horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicCtuTopLeftY;                         //!< vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t         m_subPicWidth;                               //!< the width of subpicture in units of CTU
  uint32_t         m_subPicHeight;                              //!< the height of subpicture in units of CTU
  uint32_t         m_subPicWidthInLumaSample;                   //!< the width of subpicture in units of luma sample
  uint32_t         m_subPicHeightInLumaSample;                  //!< the height of subpicture in units of luma sample
  uint32_t         m_firstCtuInSubPic;                          //!< the raster scan index of the first CTU in a subpicture
  uint32_t         m_lastCtuInSubPic;                           //!< the raster scan index of the last CTU in a subpicture
  uint32_t         m_subPicLeft;                                //!< the position of left boundary
  uint32_t         m_subPicRight;                               //!< the position of right boundary
  uint32_t         m_subPicTop;                                 //!< the position of top boundary
  uint32_t         m_subPicBottom;                              //!< the position of bottom boundary
  std::vector<uint32_t> m_ctuAddrInSubPic;                      //!< raster scan addresses of all the CTUs in the slice

  bool             m_treatedAsPicFlag;                          //!< whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool             m_loopFilterAcrossSubPicEnabledFlag;         //!< whether in-loop filtering operations may be performed across the boundaries of the subpicture
  uint32_t         m_numSlicesInSubPic;                         //!< Number of slices contained in this subpicture

public:
  SubPic();
  virtual ~SubPic();

  void             setSubPicID (uint32_t u)                {         m_subPicID = u;       }
  uint32_t         getSubPicID   ()                  const { return  m_subPicID;           }
  void             setSubPicIdx (uint32_t u)               {         m_subPicIdx = u;      }
  uint32_t         getSubPicIdx ()                   const { return  m_subPicIdx;          }
  void             setNumCTUsInSubPic   (uint32_t u)       {         m_numCTUsInSubPic = u;       }
  uint32_t         getNumCTUsInSubPic   ()           const { return  m_numCTUsInSubPic;           }
  void             setSubPicCtuTopLeftX (uint32_t u)       {         m_subPicCtuTopLeftX = u;     }
  uint32_t         getSubPicCtuTopLeftX ()           const { return  m_subPicCtuTopLeftX;         }
  void             setSubPicCtuTopLeftY (uint32_t u)       {         m_subPicCtuTopLeftY = u;     }
  uint32_t         getSubPicCtuTopLeftY ()           const { return  m_subPicCtuTopLeftY;         }
  void             setSubPicWidthInCTUs (uint32_t u)       {         m_subPicWidth = u;           }
  uint32_t         getSubPicWidthInCTUs ()           const { return  m_subPicWidth;               }
  void             setSubPicHeightInCTUs(uint32_t u)       {         m_subPicHeight = u;          }
  uint32_t         getSubPicHeightInCTUs()           const { return  m_subPicHeight;              }
  void             setFirstCTUInSubPic  (uint32_t u)       {         m_firstCtuInSubPic = u;      }
  uint32_t         getFirstCTUInSubPic  ()           const { return  m_firstCtuInSubPic;          }
  void             setLastCTUInSubPic   (uint32_t u)       {         m_lastCtuInSubPic = u;       }
  uint32_t         getLastCTUInSubPic   ()           const { return  m_lastCtuInSubPic;           }
  void             setSubPicLeft        (uint32_t u)       {         m_subPicLeft = u;            }
  uint32_t         getSubPicLeft        ()           const { return  m_subPicLeft;                }
  void             setSubPicRight       (uint32_t u)       {         m_subPicRight = u;           }
  uint32_t         getSubPicRight       ()           const { return  m_subPicRight;               }
  void             setSubPicTop         (uint32_t u)       {         m_subPicTop = u;             }
  uint32_t         getSubPicTop         ()           const { return  m_subPicTop;                 }
  void             setSubPicBottom      (uint32_t u)       {         m_subPicBottom = u;          }
  uint32_t         getSubPicBottom      ()           const { return  m_subPicBottom;              }

  void             setSubPicWidthInLumaSample (uint32_t u) {         m_subPicWidthInLumaSample = u;   }
  uint32_t         getSubPicWidthInLumaSample()      const { return  m_subPicWidthInLumaSample;       }
  void             setSubPicHeightInLumaSample(uint32_t u) {         m_subPicHeightInLumaSample = u;  }
  uint32_t         getSubPicHeightInLumaSample()     const { return  m_subPicHeightInLumaSample;      }

  std::vector<uint32_t> getCtuAddrList  ()           const { return  m_ctuAddrInSubPic;           }
  void                  clearCTUAddrList()                 { m_ctuAddrInSubPic.clear(); }
  void                  addCTUsToSubPic(std::vector<uint32_t> ctuAddrInSlice)
  {
    for (auto ctu:ctuAddrInSlice)
      m_ctuAddrInSubPic.push_back(ctu);
  }
  void  addAllCtusInPicToSubPic(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY)
  {
    CHECK(startX >= stopX || startY >= stopY, "Invalid slice definition");
    for (uint32_t ctbY = startY; ctbY < stopY; ctbY++)
    {
      for (uint32_t ctbX = startX; ctbX < stopX; ctbX++)
      {
        m_ctuAddrInSubPic.push_back(ctbY * picWidthInCtbsY + ctbX);
      }
    }
  }
  bool                 isContainingPos(const Position& pos) const
  {
    return pos.x >= m_subPicLeft && pos.x <= m_subPicRight && pos.y >= m_subPicTop  && pos.y <= m_subPicBottom;
  }
  void             setTreatedAsPicFlag           (bool u)  {         m_treatedAsPicFlag = u;   }
  bool             getTreatedAsPicFlag           ()  const { return  m_treatedAsPicFlag;       }
  void             setloopFilterAcrossEnabledFlag(bool u)  {         m_loopFilterAcrossSubPicEnabledFlag = u; }
  bool             getloopFilterAcrossEnabledFlag()  const { return  m_loopFilterAcrossSubPicEnabledFlag;     }

  bool             isFirstCTUinSubPic(uint32_t ctuAddr) const { return  ctuAddr == m_firstCtuInSubPic;  }
  bool              isLastCTUinSubPic(uint32_t ctuAddr) const { return  ctuAddr == m_lastCtuInSubPic;   }
  void             setNumSlicesInSubPic( uint32_t val )    { m_numSlicesInSubPic = val; }
  uint32_t         getNumSlicesInSubPic() const            { return m_numSlicesInSubPic; }
  bool             containsCtu(const Position& pos) const
  {
    return pos.x >= m_subPicCtuTopLeftX && pos.x < m_subPicCtuTopLeftX + m_subPicWidth &&
           pos.y >= m_subPicCtuTopLeftY && pos.y < m_subPicCtuTopLeftY + m_subPicHeight;
  }
  bool             containsCtu(int ctuAddr) const
  {
    for (auto & addr : m_ctuAddrInSubPic)
    {
      if (addr == ctuAddr)
      {
        return true;
      }
    }
    return false;
  }
};

class DCI
{
private:
  int m_maxSubLayersMinus1;
  std::vector<ProfileTierLevel> m_profileTierLevel;

public:
  DCI()
    : m_maxSubLayersMinus1(0)
  {};

  virtual ~DCI() {};

  int  getMaxSubLayersMinus1() const { return m_maxSubLayersMinus1; }
  void setMaxSubLayersMinus1(int val) { m_maxSubLayersMinus1 = val; }

  size_t getNumPTLs() const { return m_profileTierLevel.size(); }
  void  setProfileTierLevel(const std::vector<ProfileTierLevel>& val) { m_profileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx) const { return m_profileTierLevel[idx]; }
  bool  IsIndenticalDCI(const DCI& comparedDCI) const
  {
    if(m_maxSubLayersMinus1 != comparedDCI.m_maxSubLayersMinus1) return false;
    if(m_profileTierLevel != comparedDCI.m_profileTierLevel) return false;
    return true;
  }
};


class VPS
{
private:
  int                   m_VPSId;
  uint32_t              m_uiMaxLayers;

  uint32_t              m_vpsMaxSubLayers;
  uint32_t              m_vpsLayerId[MAX_VPS_LAYERS];
  bool                  m_vpsAllLayersSameNumSubLayersFlag;
  bool                  m_vpsAllIndependentLayersFlag;
  uint32_t              m_vpsCfgPredDirection[MAX_VPS_SUBLAYERS];
  bool                  m_vpsIndependentLayerFlag[MAX_VPS_LAYERS];
  bool                  m_vpsDirectRefLayerFlag[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint32_t              m_vpsMaxTidIlRefPicsPlus1[MAX_VPS_LAYERS];
  bool                  m_vpsEachLayerIsAnOlsFlag;
  uint32_t              m_vpsOlsModeIdc;
  uint32_t              m_vpsNumOutputLayerSets;
  bool                  m_vpsOlsOutputLayerFlag[MAX_NUM_OLSS][MAX_VPS_LAYERS];
  uint32_t              m_directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  uint32_t              m_generalLayerIdx[MAX_VPS_LAYERS];

  uint32_t              m_vpsNumPtls;
  bool                  m_ptPresentFlag[MAX_NUM_OLSS];
  uint32_t              m_ptlMaxTemporalId[MAX_NUM_OLSS];
  std::vector<ProfileTierLevel> m_vpsProfileTierLevel;
  uint32_t              m_olsPtlIdx[MAX_NUM_OLSS];

  // stores index ( ilrp_idx within 0 .. NumDirectRefLayers ) of the dependent reference layers
  uint32_t              m_interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS];
  bool                  m_vpsExtensionFlag;
  bool                  m_vpsGeneralHrdParamsPresentFlag;
  bool                  m_vpsSublayerCpbParamsPresentFlag;
  uint32_t              m_numOlsHrdParamsMinus1;
  uint32_t              m_hrdMaxTid[MAX_NUM_OLSS];
  uint32_t              m_olsHrdIdx[MAX_NUM_OLSS];
  GeneralHrdParams      m_generalHrdParams;
  std::vector<Size>             m_olsDpbPicSize;
  std::vector<int>              m_olsDpbParamsIdx;
  std::vector<std::vector<int>> m_outputLayerIdInOls;
  std::vector<std::vector<int>> m_numSubLayersInLayerInOLS;

#if JVET_S0100_ASPECT3
  std::vector<int> m_multiLayerOlsIdxToOlsIdx; // mapping from multi-layer OLS index to OLS index. Initialized in deriveOutputLayerSets()
                                               // m_multiLayerOlsIdxToOlsIdx[n] is the OLSidx of the n-th multi-layer OLS.
#endif
public:
  std::vector<std::vector<OlsHrdParams>> m_olsHrdParams;
  int                           m_totalNumOLSs;
  int                           m_numMultiLayeredOlss;
  uint32_t                      m_multiLayerOlsIdx[MAX_NUM_OLSS];
  int                           m_numDpbParams;
  std::vector<DpbParameters>    m_dpbParameters;
  bool                          m_sublayerDpbParamsPresentFlag;
  std::vector<int>              m_dpbMaxTemporalId;
  std::vector<int>              m_targetOutputLayerIdSet;          ///< set of LayerIds to be outputted
  std::vector<int>              m_targetLayerIdSet;                ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int                           m_targetOlsIdx;
  std::vector<int>              m_numOutputLayersInOls;
  std::vector<int>              m_numLayersInOls;
  std::vector<std::vector<int>> m_layerIdInOls;
  std::vector<int>              m_olsDpbChromaFormatIdc;
  std::vector<int>              m_olsDpbBitDepthMinus8;

public:
                    VPS();

  virtual           ~VPS();

  int               getVPSId() const                                     { return m_VPSId;                                                  }
  void              setVPSId(int i)                                      { m_VPSId = i;                                                     }

  uint32_t          getMaxLayers() const                                 { return m_uiMaxLayers;                                            }
  void              setMaxLayers(uint32_t l)                             { m_uiMaxLayers = l;                                               }

  uint32_t          getMaxSubLayers() const                              { return m_vpsMaxSubLayers;                                        }
  void              setMaxSubLayers(uint32_t value)                      { m_vpsMaxSubLayers = value;                                       }

  bool              getAllLayersSameNumSublayersFlag() const { return m_vpsAllLayersSameNumSubLayersFlag; }
  void              setAllLayersSameNumSublayersFlag(bool t) { m_vpsAllLayersSameNumSubLayersFlag = t; }

  uint32_t          getLayerId(uint32_t layerIdx) const { return m_vpsLayerId[layerIdx]; }
  void              setLayerId(uint32_t layerIdx, uint32_t layerId) { m_vpsLayerId[layerIdx] = layerId; }

  bool              getAllIndependentLayersFlag() const { return m_vpsAllIndependentLayersFlag; }
  void              setAllIndependentLayersFlag(bool t) { m_vpsAllIndependentLayersFlag = t; }
  uint32_t          getPredDirection(uint32_t tmplayer) const { return m_vpsCfgPredDirection[tmplayer]; }
  void              setPredDirection(uint32_t tmplayer, uint32_t t) { m_vpsCfgPredDirection[tmplayer] = t; }

  bool              getIndependentLayerFlag(uint32_t layerIdx) const { return m_vpsIndependentLayerFlag[layerIdx]; }
  void              setIndependentLayerFlag(uint32_t layerIdx, bool t) { m_vpsIndependentLayerFlag[layerIdx] = t; }

  uint32_t          getMaxTidIlRefPicsPlus1(uint32_t layerIdx) const { return m_vpsMaxTidIlRefPicsPlus1[layerIdx]; }
  void              setMaxTidIlRefPicsPlus1(uint32_t layerIdx, uint32_t i) { m_vpsMaxTidIlRefPicsPlus1[layerIdx] = i; }

  bool              getDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx) const { return m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx]; }
  void              setDirectRefLayerFlag(uint32_t layerIdx, uint32_t refLayerIdx, bool t) { m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx] = t; }

  uint32_t          getDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc ) const { return m_directRefLayerIdx[layerIdx][refLayerIdc]; }
  void              setDirectRefLayerIdx( uint32_t layerIdx, uint32_t refLayerIdc, uint32_t refLayerIdx ) { m_directRefLayerIdx[layerIdx][refLayerIdc] = refLayerIdx; }

  uint32_t          getInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx ) const { return m_interLayerRefIdx[layerIdx][refLayerIdx]; }
  void              setInterLayerRefIdc( uint32_t layerIdx, uint32_t refLayerIdx, uint32_t refLayerIdc ) { m_interLayerRefIdx[layerIdx][refLayerIdx] = refLayerIdc; }

  uint32_t          getGeneralLayerIdx(uint32_t layerId) const { return m_generalLayerIdx[layerId]; }
  void              setGeneralLayerIdx(uint32_t layerId, uint32_t layerIdc) { m_generalLayerIdx[layerId] = layerIdc; }

  bool              getEachLayerIsAnOlsFlag() const { return m_vpsEachLayerIsAnOlsFlag; }
  void              setEachLayerIsAnOlsFlag(bool t) { m_vpsEachLayerIsAnOlsFlag = t; }

  uint32_t          getOlsModeIdc() const { return m_vpsOlsModeIdc; }
  void              setOlsModeIdc(uint32_t t) { m_vpsOlsModeIdc = t; }

  uint32_t          getNumOutputLayerSets() const { return m_vpsNumOutputLayerSets; }
  void              setNumOutputLayerSets(uint8_t t) { m_vpsNumOutputLayerSets = t; }

  bool              getOlsOutputLayerFlag(uint32_t ols, uint32_t layer) const { return m_vpsOlsOutputLayerFlag[ols][layer]; }
  void              setOlsOutputLayerFlag(uint32_t ols, uint32_t layer, bool t) { m_vpsOlsOutputLayerFlag[ols][layer] = t; }

  uint32_t          getNumPtls()                                   const { return m_vpsNumPtls; }
  void              setNumPtls(uint32_t val)                             { m_vpsNumPtls = val; }

  bool              getPtPresentFlag(int idx)                      const { return m_ptPresentFlag[idx]; }
  void              setPtPresentFlag(int idx, bool val)                  { m_ptPresentFlag[idx] = val; }

  uint32_t          getPtlMaxTemporalId(int idx)                   const { return m_ptlMaxTemporalId[idx]; }
  void              setPtlMaxTemporalId(int idx, uint32_t val)           { m_ptlMaxTemporalId[idx] = val; }

  void              setProfileTierLevel(const std::vector<ProfileTierLevel> &val)   { m_vpsProfileTierLevel = val; }
  const ProfileTierLevel& getProfileTierLevel(int idx)             const { return m_vpsProfileTierLevel[idx]; }

  uint32_t          getOlsPtlIdx(int idx)                          const { return m_olsPtlIdx[idx]; }
  void              setOlsPtlIdx(int idx, uint32_t val)                  { m_olsPtlIdx[idx] = val; }

  bool              getVPSExtensionFlag() const                          { return m_vpsExtensionFlag;                                 }
  void              setVPSExtensionFlag(bool t)                          { m_vpsExtensionFlag = t;                                    }
  bool              getVPSGeneralHrdParamsPresentFlag() const { return m_vpsGeneralHrdParamsPresentFlag; }
  void              setVPSGeneralHrdParamsPresentFlag(bool t) { m_vpsGeneralHrdParamsPresentFlag = t; }
  bool              getVPSSublayerCpbParamsPresentFlag() const { return m_vpsSublayerCpbParamsPresentFlag; }
  void              setVPSSublayerCpbParamsPresentFlag(bool t) { m_vpsSublayerCpbParamsPresentFlag = t; }
  uint32_t          getNumOlsHrdParamsMinus1()                                   const { return m_numOlsHrdParamsMinus1; }
  void              setNumOlsHrdParamsMinus1(uint32_t val) { m_numOlsHrdParamsMinus1 = val; }

  uint32_t          getHrdMaxTid(int olsIdx)                   const { return m_hrdMaxTid[olsIdx]; }
  void              setHrdMaxTid(int olsIdx, uint32_t val)           { m_hrdMaxTid[olsIdx] = val; }
  uint32_t          getOlsHrdIdx(int olsIdx)                   const { return m_olsHrdIdx[olsIdx]; }
  void              setOlsHrdIdx(int olsIdx, uint32_t val)           { m_olsHrdIdx[olsIdx] = val; }

  OlsHrdParams*          getOlsHrdParameters(int olsIdx) { return &m_olsHrdParams[olsIdx][0]; }
  const OlsHrdParams*    getOlsHrdParameters(int olsIdx) const { return &m_olsHrdParams[olsIdx][0]; }

  GeneralHrdParams*          getGeneralHrdParameters() { return &m_generalHrdParams; }
  const GeneralHrdParams*    getGeneralHrdParameters() const { return &m_generalHrdParams; }
  int               getTargetOlsIdx() { return m_targetOlsIdx; }
  void              setTargetOlsIdx(uint32_t t) { m_targetOlsIdx = t; }

  int               getMaxDecPicBuffering( int temporalId ) const        { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_maxDecPicBuffering[temporalId]; }
  int               getNumReorderPics( int temporalId ) const            { return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].m_numReorderPics[temporalId]; }
  int               getTotalNumOLSs() const                              { return m_totalNumOLSs; }
  int               getNumMultiLayeredOlss() const                       { return m_numMultiLayeredOlss; }
  Size              getOlsDpbPicSize( int olsIdx ) const                 { return m_olsDpbPicSize[olsIdx];          }
  void              setOlsDpbPicSize( int olsIdx, Size size )            { m_olsDpbPicSize[olsIdx] = size;          }
  void              setOlsDpbPicWidth( int olsIdx, int width )           { m_olsDpbPicSize[olsIdx].width = width;   }
  void              setOlsDpbPicHeight( int olsIdx, int height )         { m_olsDpbPicSize[olsIdx].height = height; }
  int               getOlsDpbChromaFormatIdc(int olsIdx) const           { return m_olsDpbChromaFormatIdc[olsIdx];  }
  int               getOlsDpbBitDepthMinus8(int olsIdx) const            { return m_olsDpbBitDepthMinus8[olsIdx];   }
  void              setOlsDpbChromaFormatIdc(int olsIdx, int chromaFormatIdc)  { m_olsDpbChromaFormatIdc[olsIdx] = chromaFormatIdc; }
  void              setOlsDpbBitDepthMinus8(int olsIdx, int bitDepthMinus8) { m_olsDpbBitDepthMinus8[olsIdx] = bitDepthMinus8; }

  int               getOlsDpbParamsIdx( int olsIdx ) const               { return m_olsDpbParamsIdx[olsIdx];        }
  void              setOlsDpbParamsIdx( int olsIdx, int paramIdx )       { m_olsDpbParamsIdx[olsIdx] = paramIdx;    }

  void              deriveOutputLayerSets();
  void              deriveTargetOutputLayerSet( int targetOlsIdx );

#if JVET_S0100_ASPECT3
  void              checkVPS();
#endif

  void              setNumLayersInOls(int olsIdx, int numLayers)         { m_numLayersInOls[olsIdx]  = numLayers; }
  int               getNumLayersInOls(int olsIdx)      const             { return m_numLayersInOls[olsIdx]; }

  void              setLayerIdInOls  (int olsIdx, int layerIdx, int layerId)    { m_layerIdInOls[olsIdx][layerIdx] = layerId; }
  uint32_t          getLayerIdInOls  (int olsIdx, int layerIdx) const           { return m_layerIdInOls[olsIdx][layerIdx]   ; }
  std::vector<int>  getLayerIdsInOls(int targetOlsIdx)                    { return m_layerIdInOls[targetOlsIdx];     }

  int               getNumSubLayersInLayerInOLS (int olsIdx, int layerIdx) const    { return m_numSubLayersInLayerInOLS[olsIdx][layerIdx]   ; }
};

class Window
{
private:
  bool m_enabledFlag;
  int  m_winLeftOffset;
  int  m_winRightOffset;
  int  m_winTopOffset;
  int  m_winBottomOffset;
public:
  Window()
  : m_enabledFlag    (false)
  , m_winLeftOffset  (0)
  , m_winRightOffset (0)
  , m_winTopOffset   (0)
  , m_winBottomOffset(0)
  { }

  bool getWindowEnabledFlag() const   { return m_enabledFlag;                          }
  int  getWindowLeftOffset() const    { return m_enabledFlag ? m_winLeftOffset : 0;    }
  void setWindowLeftOffset(int val)   { m_winLeftOffset = val; m_enabledFlag |=  (val!=0);   }
  int  getWindowRightOffset() const   { return m_enabledFlag ? m_winRightOffset : 0;   }
  void setWindowRightOffset(int val)  { m_winRightOffset = val; m_enabledFlag |= (val!=0);  }
  int  getWindowTopOffset() const     { return m_enabledFlag ? m_winTopOffset : 0;     }
  void setWindowTopOffset(int val)    { m_winTopOffset = val; m_enabledFlag |= (val!=0);    }
  int  getWindowBottomOffset() const  { return m_enabledFlag ? m_winBottomOffset: 0;   }
  void setWindowBottomOffset(int val) { m_winBottomOffset = val; m_enabledFlag |= (val!=0); }

  void setWindow(int offsetLeft, int offsetRight, int offsetTop, int offsetBottom)
  {
    m_enabledFlag     = (offsetLeft || offsetRight || offsetTop || offsetBottom);
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetRight;
    m_winTopOffset    = offsetTop;
    m_winBottomOffset = offsetBottom;
  }
};


class VUI
{
private:
#if JVET_S0266_VUI_length
  bool       m_progressiveSourceFlag;
  bool       m_interlacedSourceFlag;
  bool       m_nonPackedFlag;
  bool       m_nonProjectedFlag;
  bool       m_aspectRatioInfoPresentFlag;
  bool       m_aspectRatioConstantFlag;
  int        m_aspectRatioIdc;
  int        m_sarWidth;
  int        m_sarHeight;
  bool       m_overscanInfoPresentFlag;
  bool       m_overscanAppropriateFlag;
  bool       m_colourDescriptionPresentFlag;
  int        m_colourPrimaries;
  int        m_transferCharacteristics;
  int        m_matrixCoefficients;
  bool       m_videoFullRangeFlag;
  bool       m_chromaLocInfoPresentFlag;
  int        m_chromaSampleLocTypeTopField;
  int        m_chromaSampleLocTypeBottomField;
  int        m_chromaSampleLocType;
#else
  bool       m_aspectRatioInfoPresentFlag;
  bool       m_aspectRatioConstantFlag;
  int        m_aspectRatioIdc;
  int        m_sarWidth;
  int        m_sarHeight;
  bool       m_colourDescriptionPresentFlag;
  int        m_colourPrimaries;
  int        m_transferCharacteristics;
  int        m_matrixCoefficients;
  bool       m_progressiveSourceFlag;
  bool       m_interlacedSourceFlag;
  bool       m_chromaLocInfoPresentFlag;
  int        m_chromaSampleLocTypeTopField;
  int        m_chromaSampleLocTypeBottomField;
  int        m_chromaSampleLocType;
  bool       m_overscanInfoPresentFlag;
  bool       m_overscanAppropriateFlag;
  bool       m_videoFullRangeFlag;
#endif

public:
  VUI()
#if JVET_S0266_VUI_length
    : m_progressiveSourceFlag             (false) // Default values as documented in VVC D10 are used
    , m_interlacedSourceFlag              (false)
    , m_nonPackedFlag                     (false)
    , m_nonProjectedFlag                  (false)
    , m_aspectRatioInfoPresentFlag        (false) 
    , m_aspectRatioConstantFlag           (false)
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_videoFullRangeFlag                (false)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (6)
    , m_chromaSampleLocTypeBottomField    (6)
    , m_chromaSampleLocType               (6)
#else
    : m_aspectRatioInfoPresentFlag        (false) //TODO: This initialiser list contains magic numbers
    , m_aspectRatioConstantFlag           (true)
    , m_aspectRatioIdc                    (0)
    , m_sarWidth                          (0)
    , m_sarHeight                         (0)
    , m_colourDescriptionPresentFlag      (false)
    , m_colourPrimaries                   (2)
    , m_transferCharacteristics           (2)
    , m_matrixCoefficients                (2)
    , m_progressiveSourceFlag             (false)
    , m_interlacedSourceFlag              (false)
    , m_chromaLocInfoPresentFlag          (false)
    , m_chromaSampleLocTypeTopField       (0)
    , m_chromaSampleLocTypeBottomField    (0)
    , m_chromaSampleLocType               (0)
    , m_overscanInfoPresentFlag           (false)
    , m_overscanAppropriateFlag           (false)
    , m_videoFullRangeFlag                (false)
#endif
  {}

  virtual           ~VUI() {}

  bool              getAspectRatioInfoPresentFlag() const                  { return m_aspectRatioInfoPresentFlag;           }
  void              setAspectRatioInfoPresentFlag(bool i)                  { m_aspectRatioInfoPresentFlag = i;              }
  bool              getAspectRatioConstantFlag() const                     { return m_aspectRatioConstantFlag;           }
  void              setAspectRatioConstantFlag(bool b)                     { m_aspectRatioConstantFlag = b;              }

  int               getAspectRatioIdc() const                              { return m_aspectRatioIdc;                       }
  void              setAspectRatioIdc(int i)                               { m_aspectRatioIdc = i;                          }

  int               getSarWidth() const                                    { return m_sarWidth;                             }
  void              setSarWidth(int i)                                     { m_sarWidth = i;                                }

  int               getSarHeight() const                                   { return m_sarHeight;                            }
  void              setSarHeight(int i)                                    { m_sarHeight = i;                               }

  bool              getColourDescriptionPresentFlag() const                { return m_colourDescriptionPresentFlag;         }
  void              setColourDescriptionPresentFlag(bool i)                { m_colourDescriptionPresentFlag = i;            }

  int               getColourPrimaries() const                             { return m_colourPrimaries;                      }
  void              setColourPrimaries(int i)                              { m_colourPrimaries = i;                         }

  int               getTransferCharacteristics() const                     { return m_transferCharacteristics;              }
  void              setTransferCharacteristics(int i)                      { m_transferCharacteristics = i;                 }

  int               getMatrixCoefficients() const                          { return m_matrixCoefficients;                   }
  void              setMatrixCoefficients(int i)                           { m_matrixCoefficients = i;                      }

  bool              getProgressiveSourceFlag() const                       { return m_progressiveSourceFlag; }
  void              setProgressiveSourceFlag(bool b)                       { m_progressiveSourceFlag = b; }

  bool              getInterlacedSourceFlag() const                        { return m_interlacedSourceFlag; }
  void              setInterlacedSourceFlag(bool b)                        { m_interlacedSourceFlag = b; }

#if JVET_S0266_VUI_length
  bool              getNonPackedFlag() const                               { return m_nonPackedFlag; }
  void              setNonPackedFlag(bool b)                               { m_nonPackedFlag = b; }

  bool              getNonProjectedFlag() const                            { return m_nonProjectedFlag; }
  void              setNonProjectedFlag(bool b)                            { m_nonProjectedFlag = b; }
#endif

  bool              getChromaLocInfoPresentFlag() const                    { return m_chromaLocInfoPresentFlag;             }
  void              setChromaLocInfoPresentFlag(bool i)                    { m_chromaLocInfoPresentFlag = i;                }

  int               getChromaSampleLocTypeTopField() const                 { return m_chromaSampleLocTypeTopField;          }
  void              setChromaSampleLocTypeTopField(int i)                  { m_chromaSampleLocTypeTopField = i;             }

  int               getChromaSampleLocTypeBottomField() const              { return m_chromaSampleLocTypeBottomField;       }
  void              setChromaSampleLocTypeBottomField(int i)               { m_chromaSampleLocTypeBottomField = i;          }

  int               getChromaSampleLocType() const                         { return m_chromaSampleLocType;          }
  void              setChromaSampleLocType(int i)                          { m_chromaSampleLocType = i;             }

  bool              getOverscanInfoPresentFlag() const                     { return m_overscanInfoPresentFlag;              }
  void              setOverscanInfoPresentFlag(bool i)                     { m_overscanInfoPresentFlag = i;                 }

  bool              getOverscanAppropriateFlag() const                     { return m_overscanAppropriateFlag;              }
  void              setOverscanAppropriateFlag(bool i)                     { m_overscanAppropriateFlag = i;                 }

  bool              getVideoFullRangeFlag() const                          { return m_videoFullRangeFlag;                   }
  void              setVideoFullRangeFlag(bool i)                          { m_videoFullRangeFlag = i;                      }

};

/// SPS RExt class
class SPSRExt // Names aligned to text specification
{
private:
  bool             m_transformSkipRotationEnabledFlag;
  bool             m_transformSkipContextEnabledFlag;
  bool             m_extendedPrecisionProcessingFlag;
  bool             m_intraSmoothingDisabledFlag;
  bool             m_highPrecisionOffsetsEnabledFlag;
  bool             m_persistentRiceAdaptationEnabledFlag;
  bool             m_cabacBypassAlignmentEnabledFlag;

public:
  SPSRExt();

  bool settingsDifferFromDefaults() const
  {
    return getTransformSkipRotationEnabledFlag()
        || getTransformSkipContextEnabledFlag()
        || getExtendedPrecisionProcessingFlag()
        || getIntraSmoothingDisabledFlag()
        || getHighPrecisionOffsetsEnabledFlag()
        || getPersistentRiceAdaptationEnabledFlag()
        || getCabacBypassAlignmentEnabledFlag();
  }


  bool getTransformSkipRotationEnabledFlag() const                                     { return m_transformSkipRotationEnabledFlag;     }
  void setTransformSkipRotationEnabledFlag(const bool value)                           { m_transformSkipRotationEnabledFlag = value;    }

  bool getTransformSkipContextEnabledFlag() const                                      { return m_transformSkipContextEnabledFlag;      }
  void setTransformSkipContextEnabledFlag(const bool value)                            { m_transformSkipContextEnabledFlag = value;     }

  bool getExtendedPrecisionProcessingFlag() const                                      { return m_extendedPrecisionProcessingFlag;      }
  void setExtendedPrecisionProcessingFlag(bool value)                                  { m_extendedPrecisionProcessingFlag = value;     }

  bool getIntraSmoothingDisabledFlag() const                                           { return m_intraSmoothingDisabledFlag;           }
  void setIntraSmoothingDisabledFlag(bool bValue)                                      { m_intraSmoothingDisabledFlag=bValue;           }

  bool getHighPrecisionOffsetsEnabledFlag() const                                      { return m_highPrecisionOffsetsEnabledFlag;      }
  void setHighPrecisionOffsetsEnabledFlag(bool value)                                  { m_highPrecisionOffsetsEnabledFlag = value;     }

  bool getPersistentRiceAdaptationEnabledFlag() const                                  { return m_persistentRiceAdaptationEnabledFlag;  }
  void setPersistentRiceAdaptationEnabledFlag(const bool value)                        { m_persistentRiceAdaptationEnabledFlag = value; }

  bool getCabacBypassAlignmentEnabledFlag() const                                      { return m_cabacBypassAlignmentEnabledFlag;      }
  void setCabacBypassAlignmentEnabledFlag(const bool value)                            { m_cabacBypassAlignmentEnabledFlag = value;     }
};


/// SPS class
class SPS
{
private:
  int               m_SPSId;
  int               m_VPSId;
  int               m_layerId;
  bool              m_affineAmvrEnabledFlag;
  bool              m_DMVR;
  bool              m_MMVD;
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool              m_affineParaRefinement;
#endif
#if AFFINE_MMVD
  bool              m_AffineMmvdMode;
#endif
#if JVET_AA0061_IBC_MBVD
  bool              m_ibcMbvd;
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  bool              m_ibcMbvdAdSearch;
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool              m_rribc;
  bool              m_tmibc;
  bool              m_ibcMerge;
#endif
#if JVET_AC0112_IBC_CIIP
  bool              m_ibcCiip;
#endif
#if JVET_AC0112_IBC_GPM
  bool              m_ibcGpm;
#endif
#if JVET_AC0112_IBC_LIC
  bool              m_ibcLic;
#endif
#if JVET_AE0159_FIBC
  bool              m_ibcFilter;
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  bool              m_ibcNonAdjCand;
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  bool              m_itmpLicExtension;
  bool              m_itmpLicMode;
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  bool              m_disableRefFilter;
  bool              m_disablePdpc;
  bool              m_disableIntraFusion;
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  bool              m_DMVDMode;
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  bool              m_tmToolsEnableFlag;
#if TM_AMVP
  bool              m_tmAmvpMode;
#endif
#if TM_MRG
  bool              m_tmMrgMode;
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  bool              m_tmGPMMode;
#endif
#if JVET_Z0061_TM_OBMC && ENABLE_OBMC
  bool              m_tmOBMCMode;
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  bool              m_useTmvpNmvpReorder;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  bool              m_useTMMMVD;
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  bool              m_altGPMSplitModeCode;
#endif
#if JVET_AL0134_SGPM_INTER
  bool              m_useSgpmInter;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  bool              m_mvdPred;
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  bool              m_bvdPred;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bool              m_bvpCluster;
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  bool              m_useARL;
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  bool              m_useGeoBlend;
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool              m_useGeoBlendIntra;
#endif
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool              m_enableMaxMttIncrease;
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int               m_alfScaleMode;
  bool              m_alfScalePrevEnabled;
#endif
#if JVET_AK0065_TALF
  bool              m_talf;
#endif
  bool              m_SBT;
#if JVET_AI0050_INTER_MTSS
  bool              m_interMTSS; 
#endif
#if JVET_AI0050_SBT_LFNST
  bool              m_sbtLFNST;
#endif
#if JVET_AL0181_ASBT
  bool              m_asbt;
  bool              m_asbtPh;
#endif
  bool              m_ISP;
  ChromaFormat      m_chromaFormatIdc;
#if !JVET_S0052_RM_SEPARATE_COLOUR_PLANE
  bool              m_separateColourPlaneFlag;     //!< separate colour plane flag
#endif

  uint32_t              m_uiMaxTLayers;           // maximum number of temporal layers

  bool              m_ptlDpbHrdParamsPresentFlag;
  bool              m_SubLayerDpbParamsFlag;

  // Structure
  uint32_t              m_maxWidthInLumaSamples;
  uint32_t              m_maxHeightInLumaSamples;
  Window                m_conformanceWindow;
  bool                  m_subPicInfoPresentFlag;                // indicates the presence of sub-picture info
  uint32_t              m_numSubPics;                        //!< number of sub-pictures used
  bool                  m_independentSubPicsFlag;
#if JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT
  bool                  m_subPicSameSizeFlag;
#endif
  std::vector<uint32_t> m_subPicCtuTopLeftX;
  std::vector<uint32_t> m_subPicCtuTopLeftY;
  std::vector<uint32_t> m_subPicWidth;
  std::vector<uint32_t> m_subPicHeight;
  std::vector<bool>     m_subPicTreatedAsPicFlag;
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag;
  bool                  m_subPicIdMappingExplicitlySignalledFlag;
  bool                  m_subPicIdMappingInSpsFlag;
  uint32_t              m_subPicIdLen;                       //!< sub-picture ID length in bits
  std::vector<uint16_t> m_subPicId;                          //!< sub-picture ID for each sub-picture in the sequence

  int               m_log2MinCodingBlockSize;
  unsigned    m_CTUSize;
  unsigned    m_partitionOverrideEnalbed;       // enable partition constraints override function
  unsigned    m_minQT[3];   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned    m_maxMTTHierarchyDepth[3];
  unsigned    m_maxBTSize[3];
  unsigned    m_maxTTSize[3];
  bool        m_idrRefParamList;
  unsigned    m_dualITree;
  uint32_t              m_uiMaxCUWidth;
  uint32_t              m_uiMaxCUHeight;

  RPLList           m_RPLList0;
  RPLList           m_RPLList1;
  uint32_t          m_numRPL0;
  uint32_t          m_numRPL1;
  
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  uint32_t          m_numLambda;
  uint32_t          m_lambdaVal[MAX_GOP];
  int               m_qpOffsets[MAX_GOP];
  int               m_maxbitsLambdaVal;
#endif

  bool              m_rpl1CopyFromRpl0Flag;
  bool              m_rpl1IdxPresentFlag;
  bool              m_allRplEntriesHasSameSignFlag;
  bool              m_bLongTermRefsPresent;
  bool              m_SPSTemporalMVPEnabledFlag;
  int               m_numReorderPics[MAX_TLAYER];

  // Tool list

#if JVET_AL0160_SBSMVP
  bool              m_spatialMVPEnabledFlag;
#endif
  bool              m_transformSkipEnabledFlag;
  int               m_log2MaxTransformSkipBlockSize;
  bool              m_BDPCMEnabledFlag;
  bool              m_JointCbCrEnabledFlag;
#if JVET_AK0085_TM_BOUNDARY_PADDING
  bool              m_templateMatchingBoundaryPrediction;
#endif

  // Parameter
  BitDepths         m_bitDepths;
  bool              m_entropyCodingSyncEnabledFlag;                    //!< Flag for enabling WPP
  bool              m_entryPointPresentFlag;                           //!< Flag for indicating the presence of entry points
  int               m_qpBDOffset[MAX_NUM_CHANNEL_TYPE];
  int               m_internalMinusInputBitDepth[MAX_NUM_CHANNEL_TYPE]; //  max(0, internal bitdepth - input bitdepth);                                          }

  bool              m_sbtmvpEnabledFlag;
  bool              m_bdofEnabledFlag;
  bool              m_fpelMmvdEnabledFlag;
  bool              m_BdofControlPresentFlag;
  bool              m_DmvrControlPresentFlag;
  bool              m_ProfControlPresentFlag;
  uint32_t          m_uiBitsForPOC;
  bool              m_pocMsbFlag;
  uint32_t          m_pocMsbLen;
  int               m_numExtraPHBitsBytes;
  int               m_numExtraSHBitsBytes;
  std::vector<bool> m_extraPHBitPresentFlag;
  std::vector<bool> m_extraSHBitPresentFlag;
  uint32_t          m_numLongTermRefPicSPS;
  uint32_t          m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool              m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  uint32_t          m_log2MaxTbSize;
  bool              m_useWeightPred;                     //!< Use of Weighting Prediction (P_SLICE)
  bool              m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)

  bool              m_saoEnabledFlag;
#if JVET_W0066_CCSAO
  bool              m_ccSaoEnabledFlag;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  bool              m_alfPrecisionFlag;
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION
  bool              m_ccalfPrecisionFlag;
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool              m_nnipMode;
#endif
#if JVET_AN0168_REGRESSION_CCP_FUSION
  int               m_ccpFusionMode;
#endif
  bool              m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  bool              m_scalingListEnabledFlag;
  bool              m_depQuantEnabledFlag;            //!< dependent quantization enabled flag
  bool              m_signDataHidingEnabledFlag;      //!< sign data hiding enabled flag
  bool              m_virtualBoundariesEnabledFlag;   //!< Enable virtual boundaries tool
  bool              m_virtualBoundariesPresentFlag;   //!< disable loop filtering across virtual boundaries
  unsigned          m_numVerVirtualBoundaries;                         //!< number of vertical virtual boundaries
  unsigned          m_numHorVirtualBoundaries;                         //!< number of horizontal virtual boundaries
  unsigned          m_virtualBoundariesPosX[3];                        //!< horizontal position of each vertical virtual boundary
  unsigned          m_virtualBoundariesPosY[3];                        //!< vertical position of each horizontal virtual boundary
  uint32_t          m_uiMaxDecPicBuffering[MAX_TLAYER];
  uint32_t          m_uiMaxLatencyIncreasePlus1[MAX_TLAYER];


  bool              m_generalHrdParametersPresentFlag;
  GeneralHrdParams m_generalHrdParams;
  OlsHrdParams     m_olsHrdParams[MAX_TLAYER];

  bool              m_fieldSeqFlag;
  bool              m_vuiParametersPresentFlag;
#if JVET_S0266_VUI_length
  unsigned          m_vuiPayloadSize;
#endif
  VUI               m_vuiParameters;

  SPSRExt           m_spsRangeExtension;

  static const int  m_winUnitX[NUM_CHROMA_FORMAT];
  static const int  m_winUnitY[NUM_CHROMA_FORMAT];
  ProfileTierLevel  m_profileTierLevel;

#if NN_COMMON_SPS
  bool              m_nnlfEnabledFlag;
  NNLFUnifiedID     m_nnlfid;
  bool              m_nnlfUnifiedEnabledFlag;
  uint32_t          m_nnlfUnifiedInferSize[MAX_NUM_NNLF_UNIFIED_INFER_GRANULARITY];
  uint32_t          m_nnlfUnifiedInfSizeExt;
  uint32_t          m_nnlfUnifiedMaxNumPrms;
#endif
  

  bool              m_alfEnabledFlag;
  bool              m_ccalfEnabledFlag;
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  bool              m_alfLumaFixedFilterAdjust;
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  bool              m_inloopOffsetRefineFlag;
  bool              m_inloopOffsetRefineFunc;
#endif
#if JVET_AL0153_ALF_CCCM
  bool              m_lfCccmEnabledFlag;
#endif
  bool              m_wrapAroundEnabledFlag;
  unsigned          m_IBCFlag;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  unsigned          m_IBCFracFlag;
  unsigned          m_IBCFlagInterSlice;
#endif
  bool              m_useColorTrans;
  unsigned          m_PLTMode;

  bool              m_lmcsEnabled;
  bool              m_AMVREnabledFlag;
  bool              m_LMChroma;
  bool              m_horCollocatedChromaFlag;
  bool              m_verCollocatedChromaFlag;
  bool              m_MTS;
  bool              m_IntraMTS;                   // 18
  bool              m_InterMTS;                   // 19
#if AHG7_MTS_TOOLOFF_CFG
  bool              m_MTSExt;
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool              m_intraLFNSTISlice;
  bool              m_intraLFNSTPBSlice;
  bool              m_interLFNST;
#else
  bool              m_LFNST;
#endif
#if AHG7_LN_TOOLOFF_CFG
  bool              m_NSPT;
  bool              m_LFNSTExt;
#endif
  bool              m_SMVD;
  bool              m_Affine;
  bool              m_AffineType;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  bool              m_useAltCost;
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  bool              m_useExtAmvp;
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  bool              m_useAffineTM;
#if JVET_AG0276_NLIC
  bool              m_useAffAltLMTM;
#endif
#if JVET_AH0119_SUBBLOCK_TM
  bool              m_useSbTmvpTM;
#endif
#endif
  bool              m_PROF;
  bool              m_bcw;                        //
#if ENABLE_DIMD
  bool              m_dimd;
#endif
#if JVET_AL0108_BVG_DIMD
  bool              m_bvgDimd;
#endif
#if JVET_W0123_TIMD_FUSION
  bool              m_timd;
#if JVET_AJ0061_TIMD_MERGE
  bool              m_timdMrg;
#endif
#endif
#if JVET_AB0155_SGPM
  bool              m_sgpm;
#endif
#if JVET_AD0082_TMRL_CONFIG
  bool              m_tmrl;
#endif
#if JVET_AN0095_QUANTIZATION_CENTER_SHIFT
  bool              m_qcs;
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  bool              m_tmNoninterToolsEnableFlag;
#endif
#if JVET_AG0058_EIP
  bool              m_eip;
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  bool              m_intraPredBf;
#endif
#if JVET_AD0085_MPM_SORTING
  bool              m_mpmSorting;
#endif
#if JVET_AK0059_MDIP
  bool              m_mdip;
#endif
#if JVET_AH0136_CHROMA_REORDERING
  bool              m_chromaReordering;
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  int               m_cccm;
#endif
#if JVET_AE0100_BVGCCCM
  bool              m_bvgCccm;
#endif
#if JVET_AD0188_CCP_MERGE
  bool              m_ccpMerge;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  int               m_maxNumCcpMergeAdjustCand;
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  bool              m_ddCcpFusion;
#endif
#if JVET_V0130_INTRA_TMP
  bool              m_intraTMP;                                       ///< intra Template Matching 
  unsigned          m_intraTmpMaxSize;                               ///< max CU size for which intra TMP is allowed
#endif
#if JVET_AC0071_DBV
  bool              m_intraDBV;
#endif
#if ENABLE_OBMC
  bool              m_OBMC;
#endif
  bool              m_ciip;
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  bool              m_ciipTimd;
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  bool              m_ciipTmMrg;
#endif
#if JVET_AG0135_AFFINE_CIIP
  bool              m_ciipAffine;
#endif
  bool              m_Geo;
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  bool              m_geoShapeAdapt;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  bool              m_geoInterIbc;
#endif
#if INTER_LIC
  bool              m_licEnabledFlag;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  bool              m_licSlopeAdjustEnabledFlag;
#endif
#endif

#if JVET_AE0059_INTER_CCCM
  bool              m_interCccm;
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  bool              m_interCcpMerge;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  bool              m_interCcpMergeZeroLumaCbf;
#endif
#endif

#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  bool              m_largeIBCLSR;
#endif

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool              m_LadfEnabled;
  int               m_LadfNumIntervals;
  int               m_LadfQpOffset[MAX_LADF_INTERVALS];
  int               m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif

#if JVET_AA0133_INTER_MTS_OPT
  int               m_interMTSMaxSize;
#endif
#if AHG7_MTS_TOOLOFF_CFG
  int              m_intraMTSMaxSize;
#endif
#if MULTI_HYP_PRED
  bool              m_InterMultiHyp;              // multi hypothesis inter prediction
  int               m_maxNumAddHyps;
  int               m_numAddHypWeights;
  int               m_maxNumAddHypRefFrames;
#endif
  bool              m_MRL;
  bool              m_MIP;
#if JVET_W0090_ARMC_TM || JVET_Y0058_IBC_LIST_MODIFY || JVET_Z0075_IBC_HMVP_ENLARGE
  bool              m_AML;
#endif
#if JVET_AG0276_NLIC
  bool              m_altLM;
  bool              m_affAltLM;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool              m_mergeOppositeLic;
  bool              m_mergeTMOppositeLic;
  bool              m_mergeAffOppositeLic;
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  bool              m_fastSubTmvp;
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  bool              m_useTemporalAffineOpt;
  bool              m_useSyntheticAffine;
#endif
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  bool              m_mvLdbRefineSwitch;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool              m_armcRefinedMotion;
#endif
  ChromaQpMappingTable m_chromaQpMappingTable;
  bool m_GDREnabledFlag;
  bool              m_SubLayerCbpParametersPresentFlag;

  bool              m_rprEnabledFlag;
  bool              m_resChangeInClvsEnabledFlag;
  bool              m_interLayerPresentFlag;
#if JVET_AC0096
  bool              m_rprFunctionalityTestingEnabledFlag;
  int               m_rprSwitchingResolutionOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE];
  int               m_rprSwitchingQPOffsetOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE];
#endif
#if JVET_AG0116
  bool              m_gopBasedRPREnabledFlag;
#endif
  uint32_t          m_log2ParallelMergeLevelMinus2;
  bool              m_ppsValidFlag[64];
  Size              m_scalingWindowSizeInPPS[64];
  uint32_t          m_maxNumMergeCand;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t          m_maxNumOppositeLicMergeCand;
#endif
#if JVET_X0049_ADAPT_DMVR
  uint32_t          m_maxNumBMMergeCand;
#endif
  uint32_t          m_maxNumAffineMergeCand;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t          m_maxNumAffineOppositeLicMergeCand;
#endif
  uint32_t          m_maxNumIBCMergeCand;
  uint32_t          m_maxNumGeoCand;
#if JVET_AG0164_AFFINE_GPM
  uint32_t          m_maxNumGpmAffCand;
#if JVET_AJ0274_GPM_AFFINE_TM
  uint32_t          m_maxNumGpmAffTmCand;
#endif
#endif
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  uint32_t          m_maxNumMHPCand;
#endif
  bool              m_scalingMatrixAlternativeColourSpaceDisabledFlag;
  bool              m_scalingMatrixDesignatedColourSpaceFlag;

  bool              m_disableScalingMatrixForLfnstBlks; 

#if SIGN_PREDICTION
  int               m_numPredSign;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  int               m_log2SignPredArea;
#endif
#endif
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  unsigned int      m_tempCabacInitMode;
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool        m_interSliceSeparateTree;
#endif

#if JVET_AH0209_PDP
  bool              m_pdp;
#endif

#if JVET_AI0183_MVP_EXTENSION
  bool              m_scaledMvExtTmvp;
  bool              m_scaledMvExtBiTmvp;
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  bool m_sbTmvpMvExt;
#endif
#if JVET_AL0157_MERGE_CANDIDATE_EXTENSION
  bool m_boundaryMvpExt;
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  unsigned int      m_refineTmvpCfgIdx;
#endif

public:

  SPS();
  virtual                 ~SPS();

  int                     getSPSId() const                                                                { return m_SPSId;                                                      }
  void                    setSPSId(int i)                                                                 { m_SPSId = i;                                                         }
  int                     getVPSId() const                                                                { return m_VPSId; }
  void                    setVPSId(int i)                                                                 { m_VPSId = i; }
  void                    setLayerId( int i )                                                             { m_layerId = i;                                                       }
  int                     getLayerId() const                                                              { return m_layerId;                                                    }
  ChromaFormat            getChromaFormatIdc () const                                                     { return m_chromaFormatIdc;                                            }
  void                    setChromaFormatIdc (ChromaFormat i)                                             { m_chromaFormatIdc = i;                                               }
#if !JVET_S0052_RM_SEPARATE_COLOUR_PLANE
  void                    setSeparateColourPlaneFlag ( bool b )                                           { m_separateColourPlaneFlag = b;                                       }
  bool                    getSeparateColourPlaneFlag () const                                             { return m_separateColourPlaneFlag;                                    }
#endif

  static int              getWinUnitX (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitX[chromaFormatIdc]; }
  static int              getWinUnitY (int chromaFormatIdc)                                               { CHECK(chromaFormatIdc < 0 || chromaFormatIdc >= NUM_CHROMA_FORMAT, "Invalid chroma format parameter"); return m_winUnitY[chromaFormatIdc]; }

  // structure
  void                    setMaxPicWidthInLumaSamples( uint32_t u )                                       { m_maxWidthInLumaSamples = u; }
  uint32_t                getMaxPicWidthInLumaSamples() const                                             { return  m_maxWidthInLumaSamples; }
  void                    setMaxPicHeightInLumaSamples( uint32_t u )                                      { m_maxHeightInLumaSamples = u; }
  uint32_t                getMaxPicHeightInLumaSamples() const                                            { return  m_maxHeightInLumaSamples; }
  Window&                 getConformanceWindow()                                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )                               { m_conformanceWindow = conformanceWindow; }

  void      setSubPicInfoPresentFlag(bool b)                                                { m_subPicInfoPresentFlag = b;            }
  bool      getSubPicInfoPresentFlag() const                                                { return m_subPicInfoPresentFlag;         }

  void      setNumSubPics( uint32_t u )                                                     { CHECK( u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded" );
                                                                                              m_numSubPics = u;
                                                                                              m_subPicCtuTopLeftX.resize(m_numSubPics);
                                                                                              m_subPicCtuTopLeftY.resize(m_numSubPics);
                                                                                              m_subPicWidth.resize(m_numSubPics);
                                                                                              m_subPicHeight.resize(m_numSubPics);
                                                                                              m_subPicTreatedAsPicFlag.resize(m_numSubPics);
                                                                                              m_loopFilterAcrossSubpicEnabledFlag.resize(m_numSubPics);
                                                                                              m_subPicId.resize(m_numSubPics);
                                                                                            }
  void      setIndependentSubPicsFlag(bool b)                                                { m_independentSubPicsFlag = b;                    }
  bool      getIndependentSubPicsFlag() const                                                { return m_independentSubPicsFlag;                 }
#if JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT
  void      setSubPicSameSizeFlag(bool b)                                                   { m_subPicSameSizeFlag = b;                       }
  bool      getSubPicSameSizeFlag() const                                                   { return m_subPicSameSizeFlag;                    }
#endif
  uint32_t  getNumSubPics( ) const                                                          { return  m_numSubPics;                           }
  void      setSubPicCtuTopLeftX( int i, uint32_t u )                                       { m_subPicCtuTopLeftX[i] = u;                     }
  uint32_t  getSubPicCtuTopLeftX( int i ) const                                             { return  m_subPicCtuTopLeftX[i];                 }
  void      setSubPicCtuTopLeftY( int i, uint32_t u )                                       { m_subPicCtuTopLeftY[i] = u;                     }
  uint32_t  getSubPicCtuTopLeftY( int i ) const                                             { return  m_subPicCtuTopLeftY[i];                 }
  void      setSubPicWidth( int i, uint32_t u )                                             { m_subPicWidth[i] = u;                           }
  uint32_t  getSubPicWidth( int i ) const                                                   { return  m_subPicWidth[i];                       }
  void      setSubPicHeight( int i, uint32_t u )                                            { m_subPicHeight[i] = u;                          }
  uint32_t  getSubPicHeight( int i ) const                                                  { return  m_subPicHeight[i];                      }
  void      setSubPicTreatedAsPicFlag( int i, bool u )                                      { m_subPicTreatedAsPicFlag[i] = u;                }
  bool      getSubPicTreatedAsPicFlag( int i ) const                                        { return  m_subPicTreatedAsPicFlag[i];            }
  void      setLoopFilterAcrossSubpicEnabledFlag( int i, bool u )                           { m_loopFilterAcrossSubpicEnabledFlag[i] = u;     }
  bool      getLoopFilterAcrossSubpicEnabledFlag( int i ) const                             { return  m_loopFilterAcrossSubpicEnabledFlag[i]; }

  void      setSubPicCtuTopLeftX                        (const std::vector<uint32_t> &v)   { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicCtuTopLeftX = v; }
  void      setSubPicCtuTopLeftY                        (const std::vector<uint32_t> &v)   { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicCtuTopLeftY = v; }
  void      setSubPicWidth                              (const std::vector<uint32_t> &v)   { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicWidth = v; }
  void      setSubPicHeight                             (const std::vector<uint32_t> &v)   { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicHeight = v; }
  void      setSubPicTreatedAsPicFlag                   (const std::vector<bool> &v)       { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_subPicTreatedAsPicFlag = v; }
  void      setLoopFilterAcrossSubpicEnabledFlag        (const std::vector<bool> &v)       { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ;m_loopFilterAcrossSubpicEnabledFlag = v; }

  bool       getDisableScalingMatrixForLfnstBlks() const { return m_disableScalingMatrixForLfnstBlks; }
  void       setDisableScalingMatrixForLfnstBlks(bool flag) { m_disableScalingMatrixForLfnstBlks = flag; }

  void                    setSubPicIdMappingExplicitlySignalledFlag( bool b )                             { m_subPicIdMappingExplicitlySignalledFlag = b;    }
  bool                    getSubPicIdMappingExplicitlySignalledFlag() const                               { return m_subPicIdMappingExplicitlySignalledFlag; }
  void                    setSubPicIdMappingInSpsFlag( bool b )                                           { m_subPicIdMappingInSpsFlag = b;                  }
  bool                    getSubPicIdMappingInSpsFlag() const                                             { return  m_subPicIdMappingInSpsFlag;              }
  void                    setSubPicIdLen( uint32_t u )                                                    { m_subPicIdLen = u;                       }
  uint32_t                getSubPicIdLen() const                                                          { return  m_subPicIdLen;                   }
  void                    setSubPicId( int i, uint16_t u )                                                { m_subPicId[i] = u;     }
  uint16_t                getSubPicId( int i ) const                                                      { return  m_subPicId[i]; }
  void                    setSubPicId(const std::vector<uint16_t> &v)                                     { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ; m_subPicId = v; }
  const std::vector<uint16_t> getSubPicIds() const                                                        { return  m_subPicId; }
#if JVET_AK0085_TM_BOUNDARY_PADDING
  void                    setTMBP( int val)                                                               {m_templateMatchingBoundaryPrediction = val;};
  bool                    getTMBP() const                                                                 {return m_templateMatchingBoundaryPrediction;};
#endif

  uint32_t                getNumLongTermRefPicSPS() const                                                 { return m_numLongTermRefPicSPS;                                       }
  void                    setNumLongTermRefPicSPS(uint32_t val)                                           { m_numLongTermRefPicSPS = val;                                        }

  uint32_t                getLtRefPicPocLsbSps(uint32_t index) const                                      { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_ltRefPicPocLsbSps[index]; }
  void                    setLtRefPicPocLsbSps(uint32_t index, uint32_t val)                              { CHECK( index >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_ltRefPicPocLsbSps[index] = val;  }

  bool                    getUsedByCurrPicLtSPSFlag(int i) const                                          { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); return m_usedByCurrPicLtSPSFlag[i];    }
  void                    setUsedByCurrPicLtSPSFlag(int i, bool x)                                        { CHECK( i >= MAX_NUM_LONG_TERM_REF_PICS, "Index exceeds boundary" ); m_usedByCurrPicLtSPSFlag[i] = x;       }

  int                     getLog2MinCodingBlockSize() const                                               { return m_log2MinCodingBlockSize;                                     }
  void                    setLog2MinCodingBlockSize(int val)                                              { m_log2MinCodingBlockSize = val;                                      }
  void                    setCTUSize(unsigned    ctuSize)                                                 { m_CTUSize = ctuSize; }
  unsigned                getCTUSize()                                                              const { return  m_CTUSize; }
  void                    setSplitConsOverrideEnabledFlag(bool b)                                         { m_partitionOverrideEnalbed = b; }
  bool                    getSplitConsOverrideEnabledFlag()                                         const { return m_partitionOverrideEnalbed; }
  void                    setMinQTSizes(unsigned*   minQT)                                                { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2]; }
  unsigned                getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA)
                                                                                                    const { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1]; }
  void                    setMaxMTTHierarchyDepth(unsigned    maxMTTHierarchyDepth,
                                        unsigned    maxMTTHierarchyDepthI,
                                        unsigned    maxMTTHierarchyDepthIChroma)
                                                                                                          { m_maxMTTHierarchyDepth[1] = maxMTTHierarchyDepth; m_maxMTTHierarchyDepth[0] = maxMTTHierarchyDepthI; m_maxMTTHierarchyDepth[2] = maxMTTHierarchyDepthIChroma; }
  unsigned                getMaxMTTHierarchyDepth()                                                 const { return m_maxMTTHierarchyDepth[1]; }
  unsigned                getMaxMTTHierarchyDepthI()                                                const { return m_maxMTTHierarchyDepth[0]; }
  unsigned                getMaxMTTHierarchyDepthIChroma()                                          const { return m_maxMTTHierarchyDepth[2]; }
  void                    setMaxBTSize(unsigned    maxBTSize,
                                       unsigned    maxBTSizeI,
                                       unsigned    maxBTSizeC)
                                                                                                          { m_maxBTSize[1] = maxBTSize; m_maxBTSize[0] = maxBTSizeI; m_maxBTSize[2] = maxBTSizeC; }
  unsigned                getMaxBTSize()                                                            const { return m_maxBTSize[1]; }
  unsigned                getMaxBTSizeI()                                                           const { return m_maxBTSize[0]; }
  unsigned                getMaxBTSizeIChroma()                                                     const { return m_maxBTSize[2]; }
  void                    setMaxTTSize(unsigned    maxTTSize,
                                       unsigned    maxTTSizeI,
                                       unsigned    maxTTSizeC)
                                                                                                          { m_maxTTSize[1] = maxTTSize; m_maxTTSize[0] = maxTTSizeI; m_maxTTSize[2] = maxTTSizeC; }
  unsigned                getMaxTTSize()                                                            const { return m_maxTTSize[1]; }
  unsigned                getMaxTTSizeI()                                                           const { return m_maxTTSize[0]; }
  unsigned                getMaxTTSizeIChroma()                                                     const { return m_maxTTSize[2]; }
  unsigned*               getMinQTSizes()                                                           const { return (unsigned *)m_minQT;                }
  unsigned*               getMaxMTTHierarchyDepths()                                                const { return (unsigned *)m_maxMTTHierarchyDepth; }
  unsigned*               getMaxBTSizes()                                                           const { return (unsigned *)m_maxBTSize;            }
  unsigned*               getMaxTTSizes()                                                           const { return (unsigned *)m_maxTTSize;            }
  void                    setIDRRefParamListPresent(bool b)                                               { m_idrRefParamList = b; }
  bool                    getIDRRefParamListPresent()                                               const { return m_idrRefParamList; }
  void                    setUseDualITree(bool b)                                                         { m_dualITree = b; }
  bool                    getUseDualITree()                                                         const { return m_dualITree; }
#if SIGN_PREDICTION
  void                    setNumPredSigns(int num)                                                        { m_numPredSign = num; }
  int                     getNumPredSigns()                                                         const { return m_numPredSign; }
#if JVET_Y0141_SIGN_PRED_IMPROVE
  void                    setLog2SignPredArea(int val)                                                    { m_log2SignPredArea = val; }
  int                     getLog2SignPredArea()                                                     const { return m_log2SignPredArea; }
  int                     getSignPredArea()                                                         const { return (1 << m_log2SignPredArea); }
#endif
#endif

  void                    setMaxCUWidth( uint32_t u )                                                         { m_uiMaxCUWidth = u;                                                  }
  uint32_t                    getMaxCUWidth() const                                                           { return  m_uiMaxCUWidth;                                              }
  void                    setMaxCUHeight( uint32_t u )                                                        { m_uiMaxCUHeight = u;                                                 }
  uint32_t                    getMaxCUHeight() const                                                          { return  m_uiMaxCUHeight;                                             }
  bool                    getTransformSkipEnabledFlag() const                                                 { return m_transformSkipEnabledFlag;                                   }
  void                    setTransformSkipEnabledFlag( bool b )                                               { m_transformSkipEnabledFlag = b;                                      }
  uint32_t                getLog2MaxTransformSkipBlockSize() const                                            { return m_log2MaxTransformSkipBlockSize;                              }
  void                    setLog2MaxTransformSkipBlockSize(uint32_t u)                                        { m_log2MaxTransformSkipBlockSize = u;                                 }
  bool                    getBDPCMEnabledFlag() const                                                         { return m_BDPCMEnabledFlag;                                           }
  void                    setBDPCMEnabledFlag( bool b )                                                       { m_BDPCMEnabledFlag = b;                                              }
  void                    setBitsForPOC( uint32_t u )                                                         { m_uiBitsForPOC = u;                                                  }
  uint32_t                    getBitsForPOC() const                                                           { return m_uiBitsForPOC;                                               }
  void                    setPocMsbFlag(bool b)                                                               { m_pocMsbFlag = b;                                                    }
  bool                    getPocMsbFlag() const                                                               { return m_pocMsbFlag;                                                 }
  void                    setPocMsbLen(uint32_t u)                                                            { m_pocMsbLen = u;                                                     }
  uint32_t                getPocMsbLen() const                                                                { return m_pocMsbLen;                                                  }
  void                    setNumExtraPHBitsBytes(int i)                                                       { m_numExtraPHBitsBytes = i;                                           }
  int                     getNumExtraPHBitsBytes() const                                                      { return m_numExtraPHBitsBytes;                                        }
  void                    setNumExtraSHBitsBytes(int i)                                                       { m_numExtraSHBitsBytes = i;                                           }
  int                     getNumExtraSHBitsBytes() const                                                      { return m_numExtraSHBitsBytes;                                        }
  void                    setExtraPHBitPresentFlags(const std::vector<bool> &b)                               { m_extraPHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraPHBitPresentFlags() const                                                   { return m_extraPHBitPresentFlag;                                      }
  void                    setExtraSHBitPresentFlags(const std::vector<bool> &b)                               { m_extraSHBitPresentFlag = b;                                         }
  const std::vector<bool> getExtraSHBitPresentFlags() const                                                   { return m_extraSHBitPresentFlag;                                      }
  void                    setNumReorderPics(int i, uint32_t tlayer)                                           { m_numReorderPics[tlayer] = i;                                        }
  int                     getNumReorderPics(uint32_t tlayer) const                                            { return m_numReorderPics[tlayer];                                     }
  void                    createRPLList0(int numRPL);
  void                    createRPLList1(int numRPL);
  const RPLList*          getRPLList( bool b ) const                                                          { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  RPLList*                getRPLList( bool b )                                                                { return b==1 ? &m_RPLList1 : &m_RPLList0;                             }
  uint32_t                getNumRPL( bool b ) const                                                           { return b==1 ? m_numRPL1   : m_numRPL0;                               }
  const RPLList*          getRPLList0() const                                                                 { return &m_RPLList0;                                                  }
  RPLList*                getRPLList0()                                                                       { return &m_RPLList0;                                                  }
  const RPLList*          getRPLList1() const                                                                 { return &m_RPLList1;                                                  }
  RPLList*                getRPLList1()                                                                       { return &m_RPLList1;                                                  }
  uint32_t                getNumRPL0() const                                                                  { return m_numRPL0;                                                    }
  uint32_t                getNumRPL1() const                                                                  { return m_numRPL1;                                                    }
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  uint32_t                getNumLambda() const                                                                { return m_numLambda;                                                  }
  int                     getIdx(uint32_t val) const;
  uint32_t                getLambdaVal(int idx) const                                                         { return m_lambdaVal[idx];                                             }
  void                    setNumLambda(uint32_t numL)                                                         { m_numLambda = numL;                                                  }
  void                    setLambdaVal(int idx, uint32_t val)                                                 { m_lambdaVal[idx] = val;                                              }
  uint32_t                getMaxbitsLambdaVal() const                                                         { return m_maxbitsLambdaVal;                                           }
  void                    setMaxbitsLambdaVal(int numL)                                                       { m_maxbitsLambdaVal = numL;                                           }
  void                    setQPOffsets(int idx, int val)                                                      { m_qpOffsets[idx] = val;                                              }
  int                     getQPOffsets(int idx) const                                                         { return m_qpOffsets[idx];                                             }
  int                     getQPOffsetsIdx(int val) const;
#endif 
  void                    setRPL1CopyFromRPL0Flag(bool isCopy)                                                { m_rpl1CopyFromRpl0Flag = isCopy;                                     }
  bool                    getRPL1CopyFromRPL0Flag() const                                                     { return m_rpl1CopyFromRpl0Flag;                                       }
  bool                    getRPL1IdxPresentFlag() const                                                       { return m_rpl1IdxPresentFlag;                                         }
  void                    setAllActiveRplEntriesHasSameSignFlag(bool isAllSame)                               { m_allRplEntriesHasSameSignFlag = isAllSame;                          }
  bool                    getAllActiveRplEntriesHasSameSignFlag() const                                       { return m_allRplEntriesHasSameSignFlag;                               }
  bool                    getLongTermRefsPresent() const                                                  { return m_bLongTermRefsPresent;                                       }
  void                    setLongTermRefsPresent(bool b)                                                  { m_bLongTermRefsPresent=b;                                            }
  bool                    getSPSTemporalMVPEnabledFlag() const                                            { return m_SPSTemporalMVPEnabledFlag;                                  }
  void                    setSPSTemporalMVPEnabledFlag(bool b)                                            { m_SPSTemporalMVPEnabledFlag=b;                                       }
#if JVET_AL0160_SBSMVP
  bool                    getSpatialMVPEnabledFlag() const                                                { return m_spatialMVPEnabledFlag;                                      }
  void                    setSpatialMVPEnabledFlag(bool b)                                                { m_spatialMVPEnabledFlag=b;                                           }
#endif
  void                    setLog2MaxTbSize( uint32_t u )                                                  { m_log2MaxTbSize = u;                                                 }
  uint32_t                getLog2MaxTbSize() const                                                        { return  m_log2MaxTbSize;                                             }
  uint32_t                getMaxTbSize() const                                                            { return  1 << m_log2MaxTbSize;                                        }
  // Bit-depth
  int                     getBitDepth(ChannelType type) const                                             { return m_bitDepths.recon[type];                                      }
  void                    setBitDepth(ChannelType type, int u )                                           { m_bitDepths.recon[type] = u;                                         }
  const BitDepths&        getBitDepths() const                                                            { return m_bitDepths;                                                  }

  bool                    getEntropyCodingSyncEnabledFlag() const                                         { return m_entropyCodingSyncEnabledFlag;                               }
  void                    setEntropyCodingSyncEnabledFlag(bool val)                                       { m_entropyCodingSyncEnabledFlag = val;                                }
  bool                    getEntryPointsPresentFlag() const                                               { return m_entryPointPresentFlag;                                      }
  void                    setEntryPointsPresentFlag(bool val)                                             { m_entryPointPresentFlag = val;                                       }
  int                     getMaxLog2TrDynamicRange(ChannelType channelType) const                         { return getSpsRangeExtension().getExtendedPrecisionProcessingFlag() ? std::max<int>(15, int(m_bitDepths.recon[channelType] + 6)) : 15; }

  int                     getDifferentialLumaChromaBitDepth() const                                       { return int(m_bitDepths.recon[CHANNEL_TYPE_LUMA]) - int(m_bitDepths.recon[CHANNEL_TYPE_CHROMA]); }
  int                     getQpBDOffset(ChannelType type) const                                           { return m_qpBDOffset[type];                                           }
  void                    setQpBDOffset(ChannelType type, int i)                                          { m_qpBDOffset[type] = i;                                              }
  int                     getInternalMinusInputBitDepth(ChannelType type) const                           { return m_internalMinusInputBitDepth[type];                                           }
  void                    setInternalMinusInputBitDepth(ChannelType type, int i)                          { m_internalMinusInputBitDepth[type] = i;                                              }

  void                    setSAOEnabledFlag(bool bVal)                                                    { m_saoEnabledFlag = bVal;                                                    }
  bool                    getSAOEnabledFlag() const                                                       { return m_saoEnabledFlag;                                                    }
#if NN_COMMON_SPS
  bool                    getNnlfEnabledFlag() const                                                      { return m_nnlfEnabledFlag; }
  void                    setNnlfEnabledFlag(bool b)                                                      { m_nnlfEnabledFlag = b; }
  NNLFUnifiedID           getNnlfId() const                                                               { return m_nnlfid; }
  void                    setNnlfId(NNLFUnifiedID ui)                                                     { m_nnlfid = ui; }
  bool                    getNnlfUnifiedEnabledFlag() const                                               { return m_nnlfUnifiedEnabledFlag; }
  void                    setNnlfUnifiedEnabledFlag(bool b)                                               { m_nnlfUnifiedEnabledFlag = b; }
  void                    setNnlfUnifiedInferSize(uint32_t* nnlfUnifiedInferSize)                         { m_nnlfUnifiedInferSize[0] = nnlfUnifiedInferSize[0]; m_nnlfUnifiedInferSize[1] = nnlfUnifiedInferSize[1]; m_nnlfUnifiedInferSize[2] = nnlfUnifiedInferSize[2]; }
  uint32_t                getNnlfUnifiedInferSize(NnlfUnifiedInferGranularity nnlfUnifiedInferGranularity) const { return m_nnlfUnifiedInferSize[nnlfUnifiedInferGranularity]; }
  uint32_t                getNnlfUnifiedInfSizeExt() const                                                { return  m_nnlfUnifiedInfSizeExt; }
  void                    setNnlfUnifiedInfSizeExt(uint32_t i)                                            { m_nnlfUnifiedInfSizeExt = i; }
  uint32_t                getNnlfUnifiedMaxNumPrms() const                                                { return  m_nnlfUnifiedMaxNumPrms; }
  void                    setNnlfUnifiedMaxNumPrms(uint32_t i)                                            { m_nnlfUnifiedMaxNumPrms = i; }
#endif


#if JVET_W0066_CCSAO
  bool                    getCCSAOEnabledFlag() const                                                     { return m_ccSaoEnabledFlag; }
  void                    setCCSAOEnabledFlag( bool b )                                                   { m_ccSaoEnabledFlag = b;    }
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  bool                    getAlfPrecisionFlag() const                                                 { return m_alfPrecisionFlag; }
  void                    setAlfPrecisionFlag( bool b )                                               { m_alfPrecisionFlag = b;    }
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION
  bool                    getCCALFPrecisionFlag() const                                                   { return m_ccalfPrecisionFlag; }
  void                    setCCALFPrecisionFlag( bool b )                                                 { m_ccalfPrecisionFlag = b; }
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool                    getNnipMode() const { return m_nnipMode; }
  void                    setNnipMode(const bool nnipMode) { m_nnipMode = nnipMode; }
#endif
#if JVET_AN0168_REGRESSION_CCP_FUSION
 int                      getCcpFusionMode() const                                                        { return m_ccpFusionMode; }
 void                     setCcpFusionMode( const int fMode )                                             { m_ccpFusionMode = fMode; }
#endif
#if JVET_AL0153_ALF_CCCM
  bool                    getLfCccmEnabledFlag() const                                                    { return m_lfCccmEnabledFlag; }
  void                    setLfCccmEnabledFlag( bool b )                                                  { m_lfCccmEnabledFlag = b; }
#endif
  bool                    getALFEnabledFlag() const                                                       { return m_alfEnabledFlag; }
  void                    setALFEnabledFlag( bool b )                                                     { m_alfEnabledFlag = b; }
bool                    getCCALFEnabledFlag() const                                                       { return m_ccalfEnabledFlag; }
void                    setCCALFEnabledFlag( bool b )                                                     { m_ccalfEnabledFlag = b; }
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  bool                    getAlfLumaFixedFilterAdjust() const                                             { return m_alfLumaFixedFilterAdjust; }
  void                    setAlfLumaFixedFilterAdjust( bool b )                                           { m_alfLumaFixedFilterAdjust = b; }
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  bool                    getInloopOffsetRefineFlag() const                                               { return m_inloopOffsetRefineFlag; }
  void                    setInloopOffsetRefineFlag( bool b )                                             { m_inloopOffsetRefineFlag = b; }
  bool                    getInloopOffsetRefineFunc() const                                               { return m_inloopOffsetRefineFunc; }
  void                    setInloopOffsetRefineFunc( bool b )                                             { m_inloopOffsetRefineFunc = b; }
#endif
  void                    setJointCbCrEnabledFlag(bool bVal)                                              { m_JointCbCrEnabledFlag = bVal; }
  bool                    getJointCbCrEnabledFlag() const                                                 { return m_JointCbCrEnabledFlag; }

  bool getSbTMVPEnabledFlag() const { return m_sbtmvpEnabledFlag; }
  void setSbTMVPEnabledFlag(bool b) { m_sbtmvpEnabledFlag = b; }

  void                    setBDOFEnabledFlag(bool b)                                                      { m_bdofEnabledFlag = b; }
  bool                    getBDOFEnabledFlag() const                                                      { return m_bdofEnabledFlag; }

  bool                    getFpelMmvdEnabledFlag() const                                                  { return m_fpelMmvdEnabledFlag; }
  void                    setFpelMmvdEnabledFlag( bool b )                                                { m_fpelMmvdEnabledFlag = b;    }
  bool                    getUseDMVR()const                                                               { return m_DMVR; }
  void                    setUseDMVR(bool b)                                                              { m_DMVR = b;    }
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool                    getUseAffineParaRefinement()const                                               { return m_affineParaRefinement; }
  void                    setUseAffineParaRefinement(bool b)                                              { m_affineParaRefinement = b; }
#endif
  bool                    getUseMMVD()const                                                               { return m_MMVD; }
  void                    setUseMMVD(bool b)                                                              { m_MMVD = b;    }
#if AFFINE_MMVD
  void                    setUseAffineMmvdMode(bool b)                                                    { m_AffineMmvdMode = b; }
  bool                    getUseAffineMmvdMode() const                                                    { return m_AffineMmvdMode; }
#endif
#if JVET_AA0061_IBC_MBVD
  void                    setUseIbcMbvd(bool b)                                                           { m_ibcMbvd = b; }
  bool                    getUseIbcMbvd() const                                                           { return m_ibcMbvd; }
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  void                    setUseIbcMbvdAdSearch(bool b)                                                   { m_ibcMbvdAdSearch = b; }
  bool                    getUseIbcMbvdAdSearch() const                                                   { return m_ibcMbvdAdSearch; }
#endif
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void                    setUseRRIbc(bool b)                                                             { m_rribc = b; }
  bool                    getUseRRIbc() const                                                             { return m_rribc; }

  void                    setUseTMIbc(bool b)                                                             { m_tmibc = b; }
  bool                    getUseTMIbc() const                                                             { return m_tmibc; }

  void                    setUseIbcMerge( bool b )                                                        { m_ibcMerge = b; }
  bool                    getUseIbcMerge() const                                                          { return m_ibcMerge; }
#endif

#if JVET_AC0112_IBC_CIIP
  void                    setUseIbcCiip(bool b)                                                           { m_ibcCiip = b; }
  bool                    getUseIbcCiip() const                                                           { return m_ibcCiip; }
#endif
#if JVET_AC0112_IBC_GPM
  void                    setUseIbcGpm(bool b)                                                            { m_ibcGpm = b; }
  bool                    getUseIbcGpm() const                                                            { return m_ibcGpm; }
#endif
#if JVET_AC0112_IBC_LIC
  void                    setUseIbcLic(bool b)                                                            { m_ibcLic = b; }
  bool                    getUseIbcLic() const                                                            { return m_ibcLic; }
#endif
#if JVET_AE0159_FIBC
  void                    setUseIbcFilter(bool b)                                                         { m_ibcFilter = b; }
  bool                    getUseIbcFilter() const                                                         { return m_ibcFilter; }
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  void                    setUseIbcNonAdjCand(bool b)                                                     { m_ibcNonAdjCand = b; }
  bool                    getUseIbcNonAdjCand() const                                                     { return m_ibcNonAdjCand; }
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  void                    setItmpLicExtension(bool b)                                                     { m_itmpLicExtension = b; }
  bool                    getItmpLicExtension() const                                                     { return m_itmpLicExtension; }
  void                    setItmpLicMode(bool b)                                                     { m_itmpLicMode = b; }
  bool                    getItmpLicMode() const                                                     { return m_itmpLicMode; }
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  void                    setDisableRefFilter(bool b)                                                     { m_disableRefFilter = b; }
  bool                    getDisableRefFilter() const                                                     { return m_disableRefFilter; }
  void                    setDisablePdpc(bool b)                                                          { m_disablePdpc = b; }
  bool                    getDisablePdpc() const                                                          { return m_disablePdpc; }
  void                    setDisableIntraFusion(bool b)                                                   { m_disableIntraFusion = b; }
  bool                    getDisableIntraFusion() const                                                   { return m_disableIntraFusion; }
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  void                    setUseDMVDMode(bool b)                                                          { m_DMVDMode = b; }
  bool                    getUseDMVDMode() const                                                          { return m_DMVDMode; }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  void                    setTMToolsEnableFlag(bool b)                                                    { m_tmToolsEnableFlag = b; }
  bool                    getTMToolsEnableFlag() const                                                    { return m_tmToolsEnableFlag; }
#if TM_AMVP
  void                    setUseTMAmvpMode(bool b)                                                        { m_tmAmvpMode = b; }
  bool                    getUseTMAmvpMode() const                                                        { return m_tmAmvpMode; }
#endif
#if TM_MRG
  void                    setUseTMMrgMode(bool b)                                                         { m_tmMrgMode = b; }
  bool                    getUseTMMrgMode() const                                                         { return m_tmMrgMode; } // Controls regular merge mode with TM, while others (e.g., GPM-TM, CIIP-TM, OBMC-TM) are controlled separately
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  void                    setUseGPMTMMode(bool b)                                                         { m_tmGPMMode = b; }
  bool                    getUseGPMTMMode() const                                                         { return m_tmGPMMode; }
#endif
#if JVET_Z0061_TM_OBMC && ENABLE_OBMC
  void                    setUseOBMCTMMode(bool b)                                                        { m_tmOBMCMode = b; }
  bool                    getUseOBMCTMMode() const                                                        { return m_tmOBMCMode; }
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  void                    setUseTmvpNmvpReordering(bool b)                                                { m_useTmvpNmvpReorder = b; }
  bool                    getUseTmvpNmvpReordering() const                                                { return m_useTmvpNmvpReorder; }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  void                    setUseTMMMVD(bool b)                                                            { m_useTMMMVD = b; }
  bool                    getUseTMMMVD() const                                                            { return m_useTMMMVD; }
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  void                    setUseAltGPMSplitModeCode(bool b)                                               { m_altGPMSplitModeCode = b; }
  bool                    getUseAltGPMSplitModeCode() const                                               { return m_altGPMSplitModeCode; }
#endif
#if JVET_AL0134_SGPM_INTER
  void                    setUseSgpmInter(bool b)                                                         { m_useSgpmInter = b; }
  bool                    getUseSgpmInter() const                                                         { return m_useSgpmInter; }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  void                    setUseMvdPred(bool b)                                                           { m_mvdPred = b; }
  bool                    getUseMvdPred() const                                                           { return m_mvdPred; }
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  void                    setUseBvdPred(bool b)                                                           { m_bvdPred = b; }
  bool                    getUseBvdPred() const                                                           { return m_bvdPred; }
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  void                    setUseBvpCluster(bool b)                                                        { m_bvpCluster = b; }
  bool                    getUseBvpCluster() const                                                        { return m_bvpCluster; }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  void                    setUseARL(bool b) { m_useARL = b; }
  bool                    getUseARL()const { return m_useARL; }
#endif
  bool                    getBdofControlPresentFlag()const                                                { return m_BdofControlPresentFlag; }
  void                    setBdofControlPresentFlag(bool b)                                               { m_BdofControlPresentFlag = b;    }

  bool                    getDmvrControlPresentFlag()const                                                { return m_DmvrControlPresentFlag; }
  void                    setDmvrControlPresentFlag(bool b)                                               { m_DmvrControlPresentFlag = b;    }

  bool                    getProfControlPresentFlag()const                                                { return m_ProfControlPresentFlag; }
  void                    setProfControlPresentFlag(bool b)                                               { m_ProfControlPresentFlag = b;    }
  uint32_t                getMaxTLayers() const                                                           { return m_uiMaxTLayers; }
  void                    setMaxTLayers( uint32_t uiMaxTLayers )                                          { CHECK( uiMaxTLayers > MAX_TLAYER, "Invalid number T-layers" ); m_uiMaxTLayers = uiMaxTLayers; }

  bool                    getPtlDpbHrdParamsPresentFlag()  const                                          { return m_ptlDpbHrdParamsPresentFlag;     }
  void                    setPtlDpbHrdParamsPresentFlag(bool b)                                           {        m_ptlDpbHrdParamsPresentFlag = b; }
  bool                    getSubLayerDpbParamsFlag()  const                                               { return m_SubLayerDpbParamsFlag;          }
  void                    setSubLayerDpbParamsFlag(bool b)                                                {        m_SubLayerDpbParamsFlag = b;      }
  bool                    getTemporalIdNestingFlag() const                                                { return m_bTemporalIdNestingFlag;                                     }
  void                    setTemporalIdNestingFlag( bool bValue )                                         { m_bTemporalIdNestingFlag = bValue;                                   }

  bool                    getScalingListFlag() const                                                      { return m_scalingListEnabledFlag;                                     }
  void                    setScalingListFlag( bool b )                                                    { m_scalingListEnabledFlag  = b;                                       }
  void                    setDepQuantEnabledFlag(bool b)                                                  { m_depQuantEnabledFlag = b; }
  bool                    getDepQuantEnabledFlag() const                                                  { return m_depQuantEnabledFlag; }
  void                    setSignDataHidingEnabledFlag(bool b)                                            { m_signDataHidingEnabledFlag = b; }
  bool                    getSignDataHidingEnabledFlag() const                                            { return m_signDataHidingEnabledFlag; }
  void                    setVirtualBoundariesEnabledFlag( bool b )                                       { m_virtualBoundariesEnabledFlag = b;                                  }
  bool                    getVirtualBoundariesEnabledFlag() const                                         { return m_virtualBoundariesEnabledFlag;                               }
  void                    setVirtualBoundariesPresentFlag( bool b )                                       { m_virtualBoundariesPresentFlag = b; }
  bool                    getVirtualBoundariesPresentFlag() const                                         { return m_virtualBoundariesPresentFlag; }
  void                    setNumVerVirtualBoundaries(unsigned u)                                          { m_numVerVirtualBoundaries = u;                                       }
  unsigned                getNumVerVirtualBoundaries() const                                              { return m_numVerVirtualBoundaries;                                    }
  void                    setNumHorVirtualBoundaries(unsigned u)                                          { m_numHorVirtualBoundaries = u;                                       }
  unsigned                getNumHorVirtualBoundaries() const                                              { return m_numHorVirtualBoundaries;                                    }
  void                    setVirtualBoundariesPosX(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;    }
  unsigned                getVirtualBoundariesPosX(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx]; }
  void                    setVirtualBoundariesPosY(unsigned u, unsigned idx)                              { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;    }
  unsigned                getVirtualBoundariesPosY(unsigned idx) const                                    { CHECK( idx >= 3, "vitrual boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx]; }
  uint32_t                getMaxDecPicBuffering(uint32_t tlayer) const                                    { return m_uiMaxDecPicBuffering[tlayer];                               }
  void                    setMaxDecPicBuffering( uint32_t ui, uint32_t tlayer )                           { CHECK(tlayer >= MAX_TLAYER, "Invalid T-layer"); m_uiMaxDecPicBuffering[tlayer] = ui;    }
  uint32_t                getMaxLatencyIncreasePlus1(uint32_t tlayer) const                               { return m_uiMaxLatencyIncreasePlus1[tlayer];                          }
  void                    setMaxLatencyIncreasePlus1( uint32_t ui , uint32_t tlayer)                      { m_uiMaxLatencyIncreasePlus1[tlayer] = ui;                            }
  uint32_t                getMaxNumMergeCand() const { return m_maxNumMergeCand; }
  void                    setMaxNumMergeCand(uint32_t u) { m_maxNumMergeCand = u; }
#if TM_MRG
  uint32_t                getMaxNumTMMergeCand() const { return std::min((uint32_t)TM_MRG_MAX_NUM_CANDS, m_maxNumMergeCand); }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t                getMaxNumOppositeLicMergeCand() const { return m_maxNumOppositeLicMergeCand; }
  void                    setMaxNumOppositeLicMergeCand(uint32_t u) { m_maxNumOppositeLicMergeCand = u; }
#endif
#if TM_MRG
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t                getMaxNumTMOppositeLicMergeCand() const { return std::min((uint32_t)TM_MRG_MAX_NUM_CANDS_OPPOSITELIC, m_maxNumOppositeLicMergeCand); }
#endif
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  uint32_t                getMaxNumCiipTMMergeCand() const { return std::min((uint32_t)CIIP_TM_MRG_MAX_NUM_CANDS, m_maxNumMergeCand); }
#endif
#if JVET_X0049_ADAPT_DMVR
  void                    setMaxNumBMMergeCand(uint32_t u) { m_maxNumBMMergeCand = u; }
  uint32_t                getMaxNumBMMergeCand() const { return m_maxNumBMMergeCand; }
#endif
  uint32_t                getMaxNumAffineMergeCand() const { return m_maxNumAffineMergeCand; }
  void                    setMaxNumAffineMergeCand(uint32_t u) { m_maxNumAffineMergeCand = u; }
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t                getMaxNumAffineOppositeLicMergeCand() const { return m_maxNumAffineOppositeLicMergeCand; }
  void                    setMaxNumAffineOppositeLicMergeCand(uint32_t u) { m_maxNumAffineOppositeLicMergeCand = u; }
#endif
  uint32_t                getMaxNumIBCMergeCand() const { return m_maxNumIBCMergeCand; }
  void                    setMaxNumIBCMergeCand(uint32_t u) { m_maxNumIBCMergeCand = u; }
  uint32_t                getMaxNumGeoCand() const                                                        { CHECK( m_maxNumGeoCand >= GEO_MAX_NUM_UNI_CANDS, "Number of GEO candidates exceed GEO_MAX_NUM_CANDS" ); return m_maxNumGeoCand; }
  void                    setMaxNumGeoCand(uint32_t u)                                                    { CHECK( m_maxNumGeoCand >= GEO_MAX_NUM_UNI_CANDS, "Number of GEO candidates exceed GEO_MAX_NUM_CANDS" ); m_maxNumGeoCand = u; }
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  uint32_t                getMaxNumGeoBlendCand() const                                                   { return getMaxNumGeoCand(); }
#endif
#if JVET_AG0164_AFFINE_GPM
  uint32_t                getMaxNumGpmAffCand() const                                                     { CHECK(m_maxNumGpmAffCand > GEO_MAX_NUM_UNI_AFF_CANDS, "Number of GPM Affine candidates exceed GEO_MAX_NUM_UNI_AFF_CANDS"); return m_maxNumGpmAffCand; }
  void                    setMaxNumGpmAffCand(uint32_t u)                                                 { CHECK(m_maxNumGpmAffCand > GEO_MAX_NUM_UNI_AFF_CANDS, "Number of GPM Affine  candidates exceed GEO_MAX_NUM_UNI_AFF_CANDS"); m_maxNumGpmAffCand = u; }
#if JVET_AJ0274_GPM_AFFINE_TM
  uint32_t                getMaxNumGpmAffTmCand() const                                                   { return m_maxNumGpmAffTmCand; }
  void                    setMaxNumGpmAffTmCand(uint32_t u)                                               { m_maxNumGpmAffTmCand = u; }
#endif
#endif
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  uint32_t                getMaxNumMHPCand() const                                                        { CHECK( m_maxNumMHPCand >= GEO_MAX_NUM_UNI_CANDS, "Number of MHP candidates exceed GEO_MAX_NUM_CANDS" ); return m_maxNumMHPCand; }
  void                    setMaxNumMHPCand(uint32_t u)                                                    { CHECK( m_maxNumMHPCand >= GEO_MAX_NUM_UNI_CANDS, "Number of MHP candidates exceed GEO_MAX_NUM_CANDS" ); m_maxNumMHPCand = u; }
#endif
  void                    setAffineAmvrEnabledFlag( bool val )                                            { m_affineAmvrEnabledFlag = val;                                       }
  bool                    getAffineAmvrEnabledFlag() const                                                { return m_affineAmvrEnabledFlag;                                      }
  bool                    getGeneralHrdParametersPresentFlag() const { return m_generalHrdParametersPresentFlag; }
  void                    setGeneralHrdParametersPresentFlag(bool b) { m_generalHrdParametersPresentFlag = b; }
  OlsHrdParams*          getOlsHrdParameters() { return &m_olsHrdParams[0]; }
  const OlsHrdParams*    getOlsHrdParameters() const { return &m_olsHrdParams[0]; }

  GeneralHrdParams*          getGeneralHrdParameters() { return &m_generalHrdParams; }
  const GeneralHrdParams*    getGeneralHrdParameters() const { return &m_generalHrdParams; }
  bool                    getFieldSeqFlag() const                                                         { return m_fieldSeqFlag;                         }
  void                    setFieldSeqFlag(bool i)                                                         { m_fieldSeqFlag = i;                            }
  bool                    getVuiParametersPresentFlag() const                                             { return m_vuiParametersPresentFlag;                                   }
  void                    setVuiParametersPresentFlag(bool b)                                             { m_vuiParametersPresentFlag = b;                                      }
#if JVET_S0266_VUI_length
  unsigned                getVuiPayloadSize() const                                                       { return m_vuiPayloadSize; }
  void                    setVuiPayloadSize(unsigned i)                                                   { m_vuiPayloadSize = i; }
#endif
  VUI*                    getVuiParameters()                                                              { return &m_vuiParameters;                                             }
  const VUI*              getVuiParameters() const                                                        { return &m_vuiParameters;                                             }
  const ProfileTierLevel* getProfileTierLevel() const                                                     { return &m_profileTierLevel; }
  ProfileTierLevel*       getProfileTierLevel()                                                           { return &m_profileTierLevel; }

  const SPSRExt&          getSpsRangeExtension() const                                                    { return m_spsRangeExtension;                                          }
  SPSRExt&                getSpsRangeExtension()                                                          { return m_spsRangeExtension;                                          }

  void                    setWrapAroundEnabledFlag(bool b)                                                { m_wrapAroundEnabledFlag = b;                                         }
  bool                    getWrapAroundEnabledFlag() const                                                { return m_wrapAroundEnabledFlag;                                      }
  void                    setUseLmcs(bool b)                                                              { m_lmcsEnabled = b;                                                   }
  bool                    getUseLmcs() const                                                              { return m_lmcsEnabled;                                                }
  void                    setIBCFlag(unsigned IBCFlag)                                                    { m_IBCFlag = IBCFlag; }
  unsigned                getIBCFlag() const                                                              { return m_IBCFlag; }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void                    setIBCFracFlag(unsigned b)                                                      { m_IBCFracFlag = b;                                                   }
  unsigned                getIBCFracFlag() const                                                          { return m_IBCFracFlag;                                                }
  void                    setIBCFlagInterSlice(unsigned IBCFlag)                                          { m_IBCFlagInterSlice = IBCFlag; }
  unsigned                getIBCFlagInterSlice() const                                                    { return m_IBCFlagInterSlice; }
#endif
  void                    setUseColorTrans(bool value) { m_useColorTrans = value; }
  bool                    getUseColorTrans() const { return m_useColorTrans; }
  void                    setPLTMode(unsigned PLTMode)                                                    { m_PLTMode = PLTMode; }
  unsigned                getPLTMode() const                                                              { return m_PLTMode; }
  void                    setUseSBT( bool b )                                                             { m_SBT = b; }
  bool                    getUseSBT() const                                                               { return m_SBT; }
#if JVET_AI0050_INTER_MTSS
  void                    setUseInterMTSS(bool b)                                                         { m_interMTSS = b; }
  bool                    getUseInterMTSS() const                                                         { return m_interMTSS; }
#endif
#if JVET_AI0050_SBT_LFNST
  void                    setUseSbtLFNST(bool b)                                                          { m_sbtLFNST = b; }
  bool                    getUseSbtLFNST() const                                                          { return m_sbtLFNST; }
#endif
#if JVET_AL0181_ASBT
  void                    setUseASBT(bool b)                                                              { m_asbt = b; }
  bool                    getUseASBT() const                                                              { return m_asbt; }
  void                    setUseASBTphSignaling(bool b)                                                   { m_asbtPh = b; }
  bool                    getUseASBTphSignaling() const                                                   { return m_asbtPh; }
#endif
  void                    setUseISP( bool b )                                                             { m_ISP = b; }
  bool                    getUseISP() const                                                               { return m_ISP; }

  void      setAMVREnabledFlag    ( bool b )                                        { m_AMVREnabledFlag = b; }
  bool      getAMVREnabledFlag    ()                                      const     { return m_AMVREnabledFlag; }
  void      setUseAffine          ( bool b )                                        { m_Affine = b; }
  bool      getUseAffine          ()                                      const     { return m_Affine; }
  void      setUseAffineType      ( bool b )                                        { m_AffineType = b; }
  bool      getUseAffineType      ()                                      const     { return m_AffineType; }
  void      setUsePROF            ( bool b )                                        { m_PROF = b; }
  bool      getUsePROF            ()                                      const     { return m_PROF; }
  void      setUseLMChroma        ( bool b )                                        { m_LMChroma = b; }
  bool      getUseLMChroma        ()                                      const     { return m_LMChroma; }
  void      setHorCollocatedChromaFlag( bool b )                                    { m_horCollocatedChromaFlag = b;    }
  bool      getHorCollocatedChromaFlag()                                  const     { return m_horCollocatedChromaFlag; }
  void      setVerCollocatedChromaFlag( bool b )                                    { m_verCollocatedChromaFlag = b;    }
  bool      getVerCollocatedChromaFlag()                                  const     { return m_verCollocatedChromaFlag; }
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  bool      getCclmHorCollocatedChromaFlag()                              const     { return m_horCollocatedChromaFlag; }
  bool      getCclmVerCollocatedChromaFlag()                              const     { return m_verCollocatedChromaFlag; }
#else
  bool      getCclmCollocatedChromaFlag()                                 const     { return m_verCollocatedChromaFlag; }
#endif
  void      setUseMTS             ( bool b )                                        { m_MTS = b; }
  bool      getUseMTS             ()                                      const     { return m_MTS; }
  bool      getUseImplicitMTS     ()                                      const     { return m_MTS && !m_IntraMTS; }
  void      setUseIntraMTS        ( bool b )                                        { m_IntraMTS = b; }
  bool      getUseIntraMTS        ()                                      const     { return m_IntraMTS; }
  void      setUseInterMTS        ( bool b )                                        { m_InterMTS = b; }
  bool      getUseInterMTS        ()                                      const     { return m_InterMTS; }
#if AHG7_MTS_TOOLOFF_CFG
  void      setUseMTSExt(bool b)                                                    { m_MTSExt = b; }
  bool      getUseMTSExt()                                                const     { return m_MTSExt; }
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  void      setUseIntraLFNSTISlice  ( bool b )                                      { m_intraLFNSTISlice = b; }
  bool      getUseIntraLFNSTISlice  ()                                    const     { return m_intraLFNSTISlice; }
  void      setUseIntraLFNSTPBSlice ( bool b )                                      { m_intraLFNSTPBSlice = b; }
  bool      getUseIntraLFNSTPBSlice ()                                    const     { return m_intraLFNSTPBSlice; }
  void      setUseInterLFNST      ( bool b )                                        { m_interLFNST = b; }
  bool      getUseInterLFNST      ()                                      const     { return m_interLFNST; }
#else
  void      setUseLFNST           ( bool b )                                        { m_LFNST = b; }
  bool      getUseLFNST           ()                                      const     { return m_LFNST; }
#endif
#if AHG7_LN_TOOLOFF_CFG
  void      setUseNSPT            ( bool b )                                        { m_NSPT = b; }
  bool      getUseNSPT            ()                                      const     { return m_NSPT; }
  void      setUseLFNSTExt        ( bool b )                                        { m_LFNSTExt = b; }
  bool      getUseLFNSTExt        ()                                      const     { return m_LFNSTExt; }
#endif
  void      setUseSMVD(bool b)                                                      { m_SMVD = b; }
  bool      getUseSMVD()                                                  const     { return m_SMVD; }
  void      setUseBcw             ( bool b )                                        { m_bcw = b; }
  bool      getUseBcw             ()                                      const     { return m_bcw; }
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  void      setLadfEnabled        ( bool b )                                        { m_LadfEnabled = b; }
  bool      getLadfEnabled        ()                                      const     { return m_LadfEnabled; }
  void      setLadfNumIntervals   ( int i )                                         { m_LadfNumIntervals = i; }
  int       getLadfNumIntervals   ()                                      const     { return m_LadfNumIntervals; }
  void      setLadfQpOffset       ( int value, int idx )                            { m_LadfQpOffset[ idx ] = value; }
  int       getLadfQpOffset       ( int idx )                             const     { return m_LadfQpOffset[ idx ]; }
  void      setLadfIntervalLowerBound( int value, int idx )                         { m_LadfIntervalLowerBound[ idx ] = value; }
  int       getLadfIntervalLowerBound( int idx )                          const     { return m_LadfIntervalLowerBound[ idx ]; }
#endif
#if JVET_AA0133_INTER_MTS_OPT
  void      setInterMTSMaxSize(int size)                                            { m_interMTSMaxSize = size; }
  int       getInterMTSMaxSize()                                          const     { return m_interMTSMaxSize; }
#endif
#if AHG7_MTS_TOOLOFF_CFG
  void      setIntraMTSMaxSize(int size)                                            { m_intraMTSMaxSize = size; }
  int       getIntraMTSMaxSize()                                          const     { return m_intraMTSMaxSize; }
#endif
#if MULTI_HYP_PRED
  bool      getUseInterMultiHyp()                                      const { return m_InterMultiHyp; }
  int       getMaxNumAddHyps()                                      const { return m_maxNumAddHyps; }
  void      setMaxNumAddHyps(int i) { m_maxNumAddHyps = i; m_InterMultiHyp = (m_maxNumAddHyps != 0); }
  void      setNumAddHypWeights(int i) { m_numAddHypWeights = i; }
  int       getNumAddHypWeights()                                      const { return m_numAddHypWeights; }
  void      setMaxNumAddHypRefFrames(int i) { m_maxNumAddHypRefFrames = i; }
  int       getMaxNumAddHypRefFrames()                                    const { return m_maxNumAddHypRefFrames; }
#endif
#if ENABLE_DIMD
  void      setUseDimd         ( bool b )                                        { m_dimd = b; }
  bool      getUseDimd         ()                                      const     { return m_dimd; }
#endif
#if JVET_AL0108_BVG_DIMD
  void      setUseBvgDimd      ( bool b )                                        { m_bvgDimd = b; }
  bool      getUseBvgDimd      ()                                      const     { return m_bvgDimd; }
#endif
#if JVET_V0130_INTRA_TMP
  void      setUseIntraTMP     (bool b)                                          { m_intraTMP = b; }
  bool      getUseIntraTMP     ()                                      const     { return m_intraTMP; }
  void      setIntraTMPMaxSize (unsigned n)                                      { m_intraTmpMaxSize = n; }
  unsigned  getIntraTMPMaxSize ()                                      const     { return m_intraTmpMaxSize; }
#endif
#if JVET_AC0071_DBV
  void setUseIntraDBV(bool b) { m_intraDBV = b; }
  bool getUseIntraDBV() const { return m_intraDBV; }
#endif
#if JVET_W0123_TIMD_FUSION
  void      setUseTimd         ( bool b )                                        { m_timd = b; }
  bool      getUseTimd         ()                                      const     { return m_timd; }
#if JVET_AJ0061_TIMD_MERGE
  void      setUseTimdMrg      ( bool b )                                        { m_timdMrg = b; }
  bool      getUseTimdMrg      ()                                      const     { return m_timdMrg; }
#endif
#endif
#if JVET_AB0155_SGPM
  void      setUseSgpm         (bool b)                                          { m_sgpm = b; }
  bool      getUseSgpm         ()                                      const     { return m_sgpm; }
#endif
#if JVET_AD0082_TMRL_CONFIG
  void      setUseTmrl         (bool b)                                          { m_tmrl = b; }
  bool      getUseTmrl         ()                                      const     { return m_tmrl; }
#endif
#if JVET_AN0095_QUANTIZATION_CENTER_SHIFT
  void      setUseQcs          (bool b)                                          { m_qcs = b; }
  bool      getUseQcs          ()                                      const     { return m_qcs; }
#endif

#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  void      setTMnoninterToolsEnableFlag         (bool b)                                          { m_tmNoninterToolsEnableFlag = b; }
  bool      getTMnoninterToolsEnableFlag          ()                                      const     { return m_tmNoninterToolsEnableFlag; }
#endif
#if JVET_AG0058_EIP
  void      setUseEip          (bool b)                                          { m_eip = b; }
  bool      getUseEip()                                                const     { return m_eip; }
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  void      setUseIntraPredBf  (bool b)                                          { m_intraPredBf = b; }
  bool      getUseIntraPredBf()                                        const     { return m_intraPredBf; }
#endif
#if JVET_AD0085_MPM_SORTING
  void      setUseMpmSorting   (bool b)                                          { m_mpmSorting = b; }
  bool      getUseMpmSorting   ()                                      const     { return m_mpmSorting; }
#endif
#if JVET_AK0059_MDIP
  void      setUseMdip         (bool b)                                          { m_mdip = b; }
  bool      getUseMdip         ()                                      const     { return m_mdip; }
#endif
#if JVET_AH0136_CHROMA_REORDERING
  void      setUseChromaReordering(bool b)                                       { m_chromaReordering = b; }
  bool      getUseChromaReordering()                                   const     { return m_chromaReordering; }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  void      setUseCccm( int i )                                                  { m_cccm = i; }
  int       getUseCccm()                                               const     { return m_cccm; }
#endif
#if JVET_AE0100_BVGCCCM
  void      setUseBvgCccm(bool b)                                                { m_bvgCccm = b; }
  bool      getUseBvgCccm()                                            const     { return m_bvgCccm; }
#endif
#if JVET_AD0188_CCP_MERGE
  void      setUseCcpMerge     ( bool i )                                        { m_ccpMerge = i; }
  bool      getUseCcpMerge     ()                                      const     { return m_ccpMerge; }
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  void      setMaxNumCcpMergeAdjustCand ( int i )                                { m_maxNumCcpMergeAdjustCand = i; }
  int       getMaxNumCcpMergeAdjustCand     ()                         const     { return m_maxNumCcpMergeAdjustCand; }
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  void      setUseDdCcpFusion  ( bool i )                                        { m_ddCcpFusion = i; }
  bool      getUseDdCcpFusion  ()                                      const     { return m_ddCcpFusion; }
#endif
#if ENABLE_OBMC
  void      setUseOBMC         ( bool b )                                        { m_OBMC = b; }
  bool      getUseOBMC         ()                                      const     { return m_OBMC; }
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  bool      getUseGeoBlend     ()                                      const     { return m_useGeoBlend; }
  void      setUseGeoBlend     ( bool b )                                        { m_useGeoBlend = b; }
#if JVET_AK0101_REGRESSION_GPM_INTRA
  bool      getUseGeoBlendIntra()                                      const     { return m_useGeoBlendIntra; }
  void      setUseGeoBlendIntra(bool b)                                          { m_useGeoBlendIntra = b; }
#endif
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool      getEnableMaxMttIncrease()                                  const     { return m_enableMaxMttIncrease; }
  void      setEnableMaxMttIncrease(bool b)                                      { m_enableMaxMttIncrease = b; }
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int       getAlfScaleMode    ()                                      const     { return m_alfScaleMode; }
  void      setAlfScaleMode    ( int m )                                         { m_alfScaleMode = m; }
  int       getAlfScaleNbCorr  ()                                      const     { return ( m_alfScaleMode ? ((m_alfScaleMode == 1) ? 5 : 9) : 0 ); }
  bool      getAlfScalePrevEnabled()                                   const     { return m_alfScalePrevEnabled; }
  void      setAlfScalePrevEnabled(bool b)                                       { m_alfScalePrevEnabled = b;    }
#endif
#if JVET_AK0065_TALF
  bool      getUseTAlf         ()                                      const     { return m_talf; }
  void      setUseTAlf         ( bool b )                                        { m_talf = b; }
#endif
  void      setUseCiip         ( bool b )                                        { m_ciip = b; }
  bool      getUseCiip         ()                                      const     { return m_ciip; }
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  void      setUseCiipTimd     (bool b)                                          { m_ciipTimd = b; }
  bool      getUseCiipTimd     ()                                      const     { return m_ciipTimd; }
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  void      setUseCiipTmMrg         ( bool b )                                        { m_ciipTmMrg = b; }
  bool      getUseCiipTmMrg         ()                                      const     { return m_ciipTmMrg; }
#endif
  void      setUseGeo             ( bool b )                                        { m_Geo = b; }
  bool      getUseGeo             ()                                      const     { return m_Geo; }
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  void      setUseGeoShapeAdapt   ( bool b )                                        { m_geoShapeAdapt = b; }
  bool      getUseGeoShapeAdapt   ()                                      const     { return m_geoShapeAdapt; }
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  void      setUseGeoInterIbc     ( bool b )                                        { m_geoInterIbc = b; }
  bool      getUseGeoInterIbc     ()                                      const     { return m_geoInterIbc; }
#endif
#if INTER_LIC
  void      setLicEnabledFlag     ( bool b )                                        { m_licEnabledFlag = b; }
  bool      getLicEnabledFlag     ()                                     const      { return m_licEnabledFlag; }
#if JVET_AG0276_LIC_SLOPE_ADJUST
  void      setLicSlopeAdjustEnabledFlag     ( bool b )                             { m_licSlopeAdjustEnabledFlag = b; }
  bool      getLicSlopeAdjustEnabledFlag     ()                          const      { return m_licSlopeAdjustEnabledFlag; }
#endif
#endif
#if JVET_AE0059_INTER_CCCM
  void      setUseInterCccm       ( bool b )                                         { m_interCccm = b; }
  bool      getUseInterCccm       ()                                      const      { return m_interCccm; }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  void      setUseInterCcpMerge   ( bool b )                                         { m_interCcpMerge = b; }
  bool      getUseInterCcpMerge   ()                                      const      { return m_interCcpMerge; }
#endif
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  void      setUseInterCcpMergeZeroLumaCbf   ( bool b )                              { m_interCcpMergeZeroLumaCbf = b; }
  bool      getUseInterCcpMergeZeroLumaCbf   ()                           const      { return m_interCcpMergeZeroLumaCbf; }
#endif
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  void      setUseLargeIBCLSR   ( bool b )                                           { m_largeIBCLSR = b; }
  bool      getUseLargeIBCLSR   ()                                        const      { return m_largeIBCLSR; }
#endif
  void      setUseMRL             ( bool b )                                        { m_MRL = b; }
  bool      getUseMRL             ()                                      const     { return m_MRL; }
  void      setUseMIP             ( bool b )                                        { m_MIP = b; }
  bool      getUseMIP             ()                                      const     { return m_MIP; }
#if JVET_AG0135_AFFINE_CIIP
  void      setUseCiipAffine      ( bool b )                                        { m_ciipAffine = b; }
  bool      getUseCiipAffine      ()                                      const     { return m_ciipAffine; }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void      setUseMergeOppositeLic( bool b )                                        { m_mergeOppositeLic = b; }
  bool      getUseMergeOppositeLic()                                      const     { return m_mergeOppositeLic; }
  void      setUseTMMergeOppositeLic( bool b )                                      { m_mergeTMOppositeLic = b; }
  bool      getUseTMMergeOppositeLic()                                    const     { return m_mergeTMOppositeLic; }
  void      setUseAffMergeOppositeLic( bool b )                                     { m_mergeAffOppositeLic = b; }
  bool      getUseAffMergeOppositeLic()                                   const     { return m_mergeAffOppositeLic; }
#endif
#if JVET_W0090_ARMC_TM || JVET_Y0058_IBC_LIST_MODIFY || JVET_Z0075_IBC_HMVP_ENLARGE
  void      setUseAML             ( bool b )                                        { m_AML = b; }
  bool      getUseAML             ()                                      const     { return m_AML; }
#if JVET_AG0276_NLIC
  void      setUseAltLM           ( bool b )                                        { m_altLM = b; }
  bool      getUseAltLM           ()                                      const     { return m_altLM; }
  void      setUseAffAltLM        ( bool b )                                        { m_affAltLM = b; }
  bool      getUseAffAltLM        ()                                      const     { return m_affAltLM; }
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  void      setUseFastSubTmvp     ( bool b )                                        { m_fastSubTmvp = b; }
  bool      getUseFastSubTmvp     ()                                      const     { return m_fastSubTmvp; }
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
  void      setUseTemporalAffineOpt( bool b )                                       { m_useTemporalAffineOpt = b; }
  bool      getUseTemporalAffineOpt()                                     const     { return m_useTemporalAffineOpt; }
  void      setUseSyntheticAffine  ( bool b )                                       { m_useSyntheticAffine = b; }
  bool      getUseSyntheticAffine  ()                                     const     { return m_useSyntheticAffine; }
#endif
#if JVET_AL0081_BDOF_LDB_MV_REFINE
  void      setMvLdbRefineSwitch( bool b )                                          { m_mvLdbRefineSwitch = b; }
  bool      getMvLdbRefineSwitch()                                        const     { return m_mvLdbRefineSwitch; }
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  void      setUseExtAmvp         ( bool b )                                        { m_useExtAmvp = b; }
  bool      getUseExtAmvp          ()                                      const     { return m_useExtAmvp; }
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  void      setUseAltCost         ( bool b )                                        { m_useAltCost = b; }
  bool      getUseAltCost         ()                                      const     { return m_useAltCost; }
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  void      setUseAffineTM        ( bool b )                                        { m_useAffineTM = b; }
  bool      getUseAffineTM        ()                                     const      { return  m_useAffineTM; }
#if JVET_AG0276_NLIC
  void      setUseAffAltLMTM      ( bool b )                                        { m_useAffAltLMTM = b; }
  bool      getUseAffAltLMTM      ()                                      const     { return m_useAffAltLMTM; }
#endif
#if JVET_AH0119_SUBBLOCK_TM
  void      setUseSbTmvpTM        ( bool b )                                        { m_useSbTmvpTM = b; }
  bool      getUseSbTmvpTM        ()                                      const     { return m_useSbTmvpTM; }
#endif
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  void      setUseArmcRefinedMotion ( bool b )                                      { m_armcRefinedMotion = b; }
  bool      getUseArmcRefinedMotion  ()                                   const     { return m_armcRefinedMotion; }
#endif
  bool      getUseWP              ()                                      const     { return m_useWeightPred; }
  bool      getUseWPBiPred        ()                                      const     { return m_useWeightedBiPred; }
  void      setUseWP              ( bool b )                                        { m_useWeightPred = b; }
  void      setUseWPBiPred        ( bool b )                                        { m_useWeightedBiPred = b; }
  void      setChromaQpMappingTableFromParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)   { m_chromaQpMappingTable.setParams(params, qpBdOffset); }
  void      derivedChromaQPMappingTables()                                          { m_chromaQpMappingTable.derivedChromaQPMappingTables(); }
  const ChromaQpMappingTable& getChromaQpMappingTable()                   const     { return m_chromaQpMappingTable;}
  int       getMappedChromaQpValue(ComponentID compID, int qpVal)         const     { return m_chromaQpMappingTable.getMappedChromaQpValue(compID, qpVal); }
  void      setGDREnabledFlag(bool flag)                                            { m_GDREnabledFlag = flag; }
  bool      getGDREnabledFlag() const                                               { return m_GDREnabledFlag; }
  void      setSubLayerParametersPresentFlag(bool flag)                             { m_SubLayerCbpParametersPresentFlag = flag; }
  bool      getSubLayerParametersPresentFlag()                            const     { return m_SubLayerCbpParametersPresentFlag;  }

  bool      getRprEnabledFlag()                                           const     { return m_rprEnabledFlag; }
  void      setRprEnabledFlag( bool flag )                                          { m_rprEnabledFlag = flag; }
  bool      getInterLayerPresentFlag()                                        const { return m_interLayerPresentFlag; }
  void      setInterLayerPresentFlag( bool b )                                      { m_interLayerPresentFlag = b; }
  bool      getResChangeInClvsEnabledFlag()                               const     { return m_resChangeInClvsEnabledFlag; }
  void      setResChangeInClvsEnabledFlag(bool flag)                                { m_resChangeInClvsEnabledFlag = flag; }
#if JVET_AC0096
  bool      getRprFunctionalityTestingEnabledFlag()                       const { return m_rprFunctionalityTestingEnabledFlag; }
  void      setRprFunctionalityTestingEnabledFlag(bool flag)                        { m_rprFunctionalityTestingEnabledFlag = flag; }
  void      setRprSwitchingResolutionOrderList(int value, int idx)                  { m_rprSwitchingResolutionOrderList[idx] = value; }
  int       getRprSwitchingResolutionOrderList(int idx)                  const { return m_rprSwitchingResolutionOrderList[idx]; }
  void      setRprSwitchingQPOffsetOrderList(int value, int idx)                    { m_rprSwitchingQPOffsetOrderList[idx] = value; }
  int       getRprSwitchingQPOffsetOrderList(int idx)                    const { return m_rprSwitchingQPOffsetOrderList[idx]; }
#endif
#if JVET_AG0116
  bool      getGOPBasedRPREnabledFlag()                                   const { return m_gopBasedRPREnabledFlag; }
  void      setGOPBasedRPREnabledFlag(bool flag)                                { m_gopBasedRPREnabledFlag = flag; }
#endif
  uint32_t  getLog2ParallelMergeLevelMinus2() const { return m_log2ParallelMergeLevelMinus2; }
  void      setLog2ParallelMergeLevelMinus2(uint32_t mrgLevel) { m_log2ParallelMergeLevelMinus2 = mrgLevel; }
  void          setPPSValidFlag(int i, bool b) { m_ppsValidFlag[i] = b; }
  bool          getPPSValidFlag(int i)         { return m_ppsValidFlag[i]; }
  void          setScalingWindowSizeInPPS(int i, int scWidth, int scHeight) { m_scalingWindowSizeInPPS[i].width = scWidth; m_scalingWindowSizeInPPS[i].height = scHeight;}
  const Size&   getScalingWindowSizeInPPS(int i)                            { return m_scalingWindowSizeInPPS[i]; }
  void      setScalingMatrixForAlternativeColourSpaceDisabledFlag(bool b)           { m_scalingMatrixAlternativeColourSpaceDisabledFlag = b; }
  bool      getScalingMatrixForAlternativeColourSpaceDisabledFlag()           const { return m_scalingMatrixAlternativeColourSpaceDisabledFlag; }
  void      setScalingMatrixDesignatedColourSpaceFlag(bool b)                       { m_scalingMatrixDesignatedColourSpaceFlag = b; }
  bool      getScalingMatrixDesignatedColourSpaceFlag()                       const { return m_scalingMatrixDesignatedColourSpaceFlag; }

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  void         setTempCabacInitMode( unsigned n )                                { m_tempCabacInitMode = n; }
  unsigned int getTempCabacInitMode()                                      const { return m_tempCabacInitMode; }
#endif

#if JVET_AH0209_PDP
  void      setUsePDP( bool b )                                                 { m_pdp = b; }
  bool      getUsePDP()                                                   const { return m_pdp; }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void     setUseInterSliceSeparateTree     ( bool b )                              { m_interSliceSeparateTree = b;    }
  bool     getInterSliceSeparateTreeEnabled ()                            const     { return m_interSliceSeparateTree; }
#endif

#if JVET_AI0183_MVP_EXTENSION
  void      setConfigScaledMvExtTmvp( bool n )                     { m_scaledMvExtTmvp = n; }
  bool      getConfigScaledMvExtTmvp()                       const { return m_scaledMvExtTmvp; }
  void      setConfigScaledMvExtBiTmvp( bool n )                   { m_scaledMvExtBiTmvp = n; }
  bool      getConfigScaledMvExtBiTmvp()                     const { return m_scaledMvExtBiTmvp; }
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  void      setConfigSbTmvpMvExt( bool n )                     { m_sbTmvpMvExt = n; }
  bool      getConfigSbTmvpMvExt()                       const { return m_sbTmvpMvExt; }
#endif
#if JVET_AL0157_MERGE_CANDIDATE_EXTENSION
  void      setConfigBoundaryMvpExt(bool n)                       { m_boundaryMvpExt = n; }
  bool      getConfigBoundaryMvpExt()                       const { return m_boundaryMvpExt; }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void          setRefineTmvpCfgIdx( unsigned n )                 { m_refineTmvpCfgIdx = n; }
  unsigned int  getRefineTmvpCfgIdx()                       const { return m_refineTmvpCfgIdx; }
#endif
};


/// PPS class
class PPS
{
private:
  int              m_PPSId;                    // pic_parameter_set_id
  int              m_SPSId;                    // seq_parameter_set_id
  int              m_picInitQPMinus26;
  bool             m_useDQP;
  bool             m_usePPSChromaTool;
  bool             m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  int              m_layerId;
  int              m_temporalId;
  int              m_puCounter;

  // access channel

  int              m_chromaCbQpOffset;
  int              m_chromaCrQpOffset;
  bool             m_chromaJointCbCrQpOffsetPresentFlag;
  int              m_chromaCbCrQpOffset;

  // Chroma QP Adjustments
  int              m_chromaQpOffsetListLen; // size (excludes the null entry used in the following array).
  ChromaQpAdj      m_ChromaQpAdjTableIncludingNullEntry[1+MAX_QP_OFFSET_LIST_SIZE]; //!< Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise

  uint32_t             m_numRefIdxL0DefaultActive;
  uint32_t             m_numRefIdxL1DefaultActive;

  bool             m_rpl1IdxPresentFlag;

  bool             m_bUseWeightPred;                    //!< Use of Weighting Prediction (P_SLICE)
  bool             m_useWeightedBiPred;                 //!< Use of Weighting Bi-Prediction (B_SLICE)
  bool             m_OutputFlagPresentFlag;             //!< Indicates the presence of output_flag in slice header
  uint32_t         m_numSubPics;                        //!< number of sub-pictures used - must match SPS
  bool             m_subPicIdMappingInPpsFlag;
  uint32_t         m_subPicIdLen;                       //!< sub-picture ID length in bits
  std::vector<uint16_t> m_subPicId;                     //!< sub-picture ID for each sub-picture in the sequence
  bool             m_noPicPartitionFlag;                //!< no picture partitioning flag - single slice, single tile
  uint8_t          m_log2CtuSize;                       //!< log2 of the CTU size - required to match corresponding value in SPS
#if CTU_256
  uint16_t         m_ctuSize;                           //!< CTU size
#else
  uint8_t          m_ctuSize;                           //!< CTU size
#endif
  uint32_t         m_picWidthInCtu;                     //!< picture width in units of CTUs
  uint32_t         m_picHeightInCtu;                    //!< picture height in units of CTUs
  uint32_t         m_numExpTileCols;                    //!< number of explicitly specified tile columns
  uint32_t         m_numExpTileRows;                    //!< number of explicitly specified tile rows
  uint32_t         m_numTileCols;                       //!< number of tile columns
  uint32_t         m_numTileRows;                       //!< number of tile rows
  std::vector<uint32_t> m_tileColWidth;                 //!< tile column widths in units of CTUs
  std::vector<uint32_t> m_tileRowHeight;                //!< tile row heights in units of CTUs
  std::vector<uint32_t> m_tileColBd;                    //!< tile column left-boundaries in units of CTUs
  std::vector<uint32_t> m_tileRowBd;                    //!< tile row top-boundaries in units of CTUs
  std::vector<uint32_t> m_ctuToTileCol;                 //!< mapping between CTU horizontal address and tile column index
  std::vector<uint32_t> m_ctuToTileRow;                 //!< mapping between CTU vertical address and tile row index
  bool             m_rectSliceFlag;                     //!< rectangular slice flag
  bool             m_singleSlicePerSubPicFlag;          //!< single slice per sub-picture flag
  std::vector<uint32_t> m_ctuToSubPicIdx;               //!< mapping between CTU and Sub-picture index
  uint32_t         m_numSlicesInPic;                    //!< number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool             m_tileIdxDeltaPresentFlag;           //!< tile index delta present flag
  std::vector<RectSlice> m_rectSlices;                  //!< list of rectangular slice signalling parameters
  std::vector<SliceMap>  m_sliceMap;                    //!< list of CTU maps for each slice in the picture
  std::vector<SubPic>      m_subPics;                   //!< list of subpictures in the picture
  bool             m_loopFilterAcrossTilesEnabledFlag;  //!< loop filtering applied across tiles flag
  bool             m_loopFilterAcrossSlicesEnabledFlag; //!< loop filtering applied across slices flag


  bool             m_cabacInitPresentFlag;

  bool             m_pictureHeaderExtensionPresentFlag;   //< picture header extension flags present in picture headers or not
  bool             m_sliceHeaderExtensionPresentFlag;
  bool             m_deblockingFilterControlPresentFlag;
  bool             m_deblockingFilterOverrideEnabledFlag;
  bool             m_ppsDeblockingFilterDisabledFlag;
#if DB_PARAM_TID
  std::vector<int> m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  std::vector<int> m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
#else
  int              m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int              m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
#endif
  int              m_deblockingFilterCbBetaOffsetDiv2;    //< beta offset for Cb deblocking filter
  int              m_deblockingFilterCbTcOffsetDiv2;      //< tc offset for Cb deblocking filter
  int              m_deblockingFilterCrBetaOffsetDiv2;    //< beta offset for Cr deblocking filter
  int              m_deblockingFilterCrTcOffsetDiv2;      //< tc offset for Cr deblocking filter
  bool             m_listsModificationPresentFlag;

  bool             m_rplInfoInPhFlag;
  bool             m_dbfInfoInPhFlag;
  bool             m_saoInfoInPhFlag;
  bool             m_alfInfoInPhFlag;
  bool             m_wpInfoInPhFlag;
  bool             m_qpDeltaInfoInPhFlag;
  bool             m_mixedNaluTypesInPicFlag;

  bool             m_conformanceWindowFlag;
  uint32_t         m_picWidthInLumaSamples;
  uint32_t         m_picHeightInLumaSamples;
  Window           m_conformanceWindow;
  Window           m_scalingWindow;

  bool             m_wrapAroundEnabledFlag;               //< reference wrap around enabled or not
  unsigned         m_picWidthMinusWrapAroundOffset;          // <pic_width_in_minCbSizeY - wraparound_offset_in_minCbSizeY
  unsigned         m_wrapAroundOffset;                    //< reference wrap around offset in luma samples
#if JVET_AC0189_SGPM_NO_BLENDING
  bool             m_sgpmNoBlend;
#endif
#if JVET_V0094_BILATERAL_FILTER
  bool             m_BIF;
  int              m_BIFStrength;
  int              m_BIFQPOffset;
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  bool             m_chromaBIF;
  int              m_chromaBIFStrength;
  int              m_chromaBIFQPOffset;
#endif

public:
  PreCalcValues   *pcv;

public:
                         PPS();
  virtual                ~PPS();

  int                    getPPSId() const                                                 { return m_PPSId;                               }
  void                   setPPSId(int i)                                                  { m_PPSId = i;                                  }
  int                    getSPSId() const                                                 { return m_SPSId;                               }
  void                   setSPSId(int i)                                                  { m_SPSId = i;                                  }

  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setPuCounter(int i)                                              { m_puCounter = i;                              }
  int                    getPuCounter()                                             const { return m_puCounter;                           }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }

  int                    getPicInitQPMinus26() const                                      { return  m_picInitQPMinus26;                   }
  void                   setPicInitQPMinus26( int i )                                     { m_picInitQPMinus26 = i;                       }
  bool                   getUseDQP() const                                                { return m_useDQP;                              }
  void                   setUseDQP( bool b )                                              { m_useDQP   = b;                               }
  bool                   getPPSChromaToolFlag()                                     const { return  m_usePPSChromaTool;                   }
  void                   setPPSChromaToolFlag(bool b)                                     { m_usePPSChromaTool = b;                       }
  bool                   getSliceChromaQpFlag() const                                     { return  m_bSliceChromaQpFlag;                 }
  void                   setSliceChromaQpFlag( bool b )                                   { m_bSliceChromaQpFlag = b;                     }


  bool                   getJointCbCrQpOffsetPresentFlag() const                          { return m_chromaJointCbCrQpOffsetPresentFlag;   }
  void                   setJointCbCrQpOffsetPresentFlag(bool b)                          { m_chromaJointCbCrQpOffsetPresentFlag = b;      }

  void                   setQpOffset(ComponentID compID, int i )
  {
    if      (compID==COMPONENT_Cb)
    {
      m_chromaCbQpOffset = i;
    }
    else if (compID==COMPONENT_Cr)
    {
      m_chromaCrQpOffset = i;
    }
    else if (compID==JOINT_CbCr)
    {
      m_chromaCbCrQpOffset = i;
    }
    else
    {
      THROW( "Invalid chroma QP offset" );
    }
  }
  int                    getQpOffset(ComponentID compID) const
  {
    return (compID==COMPONENT_Y) ? 0 : (compID==COMPONENT_Cb ? m_chromaCbQpOffset : compID==COMPONENT_Cr ? m_chromaCrQpOffset : m_chromaCbCrQpOffset );
  }

  bool                   getCuChromaQpOffsetListEnabledFlag() const                       { return getChromaQpOffsetListLen()>0;            }
  int                    getChromaQpOffsetListLen() const                                 { return m_chromaQpOffsetListLen;                 }
  void                   clearChromaQpOffsetList()                                        { m_chromaQpOffsetListLen = 0;                    }

  const ChromaQpAdj&     getChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1 ) const
  {
    CHECK(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen+1, "Invalid chroma QP offset");
    return m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1]; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  }

  void                   setChromaQpOffsetListEntry( int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset )
  {
    CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE, "Invalid chroma QP offset");
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CbOffset = cbOffset; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.CrOffset = crOffset;
    m_ChromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.JointCbCrOffset = jointCbCrOffset;
    m_chromaQpOffsetListLen = std::max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
  }

  void                   setNumRefIdxL0DefaultActive(uint32_t ui)                             { m_numRefIdxL0DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL0DefaultActive() const                              { return m_numRefIdxL0DefaultActive;            }
  void                   setNumRefIdxL1DefaultActive(uint32_t ui)                             { m_numRefIdxL1DefaultActive=ui;                }
  uint32_t                   getNumRefIdxL1DefaultActive() const                              { return m_numRefIdxL1DefaultActive;            }

  void                   setRpl1IdxPresentFlag(bool isPresent)                            { m_rpl1IdxPresentFlag = isPresent;             }
  uint32_t               getRpl1IdxPresentFlag() const                                    { return m_rpl1IdxPresentFlag;                  }

  bool                   getUseWP() const                                                 { return m_bUseWeightPred;                      }
  bool                   getWPBiPred() const                                              { return m_useWeightedBiPred;                   }
  void                   setUseWP( bool b )                                               { m_bUseWeightPred = b;                         }
  void                   setWPBiPred( bool b )                                            { m_useWeightedBiPred = b;                      }

  void                   setWrapAroundEnabledFlag(bool b)                                 { m_wrapAroundEnabledFlag = b;                  }
  bool                   getWrapAroundEnabledFlag() const                                 { return m_wrapAroundEnabledFlag;               }  
  void                   setPicWidthMinusWrapAroundOffset(unsigned offset)                { m_picWidthMinusWrapAroundOffset = offset;     }
  unsigned               getPicWidthMinusWrapAroundOffset() const                         { return m_picWidthMinusWrapAroundOffset;       }
  void                   setWrapAroundOffset(unsigned offset)                             { m_wrapAroundOffset = offset;                  }
  unsigned               getWrapAroundOffset() const                                      { return m_wrapAroundOffset;                    }
  void                   setOutputFlagPresentFlag( bool b )                               { m_OutputFlagPresentFlag = b;                  }
  bool                   getOutputFlagPresentFlag() const                                 { return m_OutputFlagPresentFlag;               }
  void                   setNumSubPics(uint32_t u )                                       { CHECK( u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded" );
                                                                                            m_numSubPics = u;
                                                                                            m_subPicId.resize(m_numSubPics);
                                                                                          }
  uint32_t               getNumSubPics( ) const                                           { return  m_numSubPics;                         }
  void                   setSubPicIdMappingInPpsFlag( bool b )                            { m_subPicIdMappingInPpsFlag = b;               }
  bool                   getSubPicIdMappingInPpsFlag() const                              { return m_subPicIdMappingInPpsFlag;            }
  void                   setSubPicIdLen( uint32_t u )                                     { m_subPicIdLen = u;                            }
  uint32_t               getSubPicIdLen() const                                           { return  m_subPicIdLen;                        }
  void                   setSubPicId( int i, uint16_t u )                                 { m_subPicId[i] = u;     }
  void                   setSubPicId(const std::vector<uint16_t> &v)                      { CHECK(v.size()!=m_numSubPics, "number of vector entries must be equal to numSubPics") ; m_subPicId = v; }
  uint16_t               getSubPicId( int i ) const                                       { return  m_subPicId[i]; }
  const std::vector<uint16_t> getSubPicIds() const                                        { return  m_subPicId; }
  uint32_t               getSubPicIdxFromSubPicId( uint32_t subPicId ) const;
  void                   setNoPicPartitionFlag( bool b )                                  { m_noPicPartitionFlag = b;                     }
  bool                   getNoPicPartitionFlag( ) const                                   { return  m_noPicPartitionFlag;                 }
  void                   setLog2CtuSize( uint8_t u )                                      { m_log2CtuSize = u; m_ctuSize = 1 << m_log2CtuSize;
                                                                                            m_picWidthInCtu = (m_picWidthInLumaSamples  + m_ctuSize - 1) / m_ctuSize;
                                                                                            m_picHeightInCtu = (m_picHeightInLumaSamples  + m_ctuSize - 1) / m_ctuSize; }
  uint8_t                getLog2CtuSize( ) const                                          { return  m_log2CtuSize;                        }
#if CTU_256
  uint16_t               getCtuSize( ) const                                              { return  m_ctuSize;                            }
#else
  uint8_t                getCtuSize( ) const                                              { return  m_ctuSize;                            }
#endif
  uint32_t               getPicWidthInCtu( ) const                                        { return  m_picWidthInCtu;                      }
  uint32_t               getPicHeightInCtu( ) const                                       { return  m_picHeightInCtu;                     }
  void                   setNumExpTileColumns( uint32_t u )                               { m_numExpTileCols = u;                         }
  uint32_t               getNumExpTileColumns( ) const                                    { return  m_numExpTileCols;                     }
  void                   setNumExpTileRows( uint32_t u )                                  { m_numExpTileRows = u;                         }
  uint32_t               getNumExpTileRows( ) const                                       { return  m_numExpTileRows;                     }
  void                   setNumTileColumns( uint32_t u )                                  { m_numTileCols = u;                            }
  uint32_t               getNumTileColumns( ) const                                       { return  m_numTileCols;                        }
  void                   setNumTileRows( uint32_t u )                                     { m_numTileRows = u;                            }
  uint32_t               getNumTileRows( ) const                                          { return  m_numTileRows;                        }
  uint32_t               getNumTiles( ) const                                             { return  m_numTileCols * m_numTileRows;        }
  void                   setTileColumnWidths( std::vector<uint32_t> widths )              { m_tileColWidth = widths;                      }
  void                   setTileRowHeights( std::vector<uint32_t> heights )               { m_tileRowHeight = heights;                    }
  void                   addTileColumnWidth( uint32_t u )                                 { CHECK( m_tileColWidth.size()  >= MAX_TILE_COLS, "Number of tile columns exceeds valid range" ); m_tileColWidth.push_back(u);    }
#if JVET_S0156_LEVEL_DEFINITION
  void                   addTileRowHeight( uint32_t u )                                   { m_tileRowHeight.push_back(u);   }
#else
  void                   addTileRowHeight( uint32_t u )                                   { CHECK( m_tileRowHeight.size() >= MAX_TILE_ROWS, "Number of tile rows exceeds valid range" );    m_tileRowHeight.push_back(u);   }
#endif
  uint32_t               getTileColumnWidth( int idx ) const                              { CHECK( idx >= m_tileColWidth.size(), "Tile column index exceeds valid range" );                 return  m_tileColWidth[idx];    }
  uint32_t               getTileRowHeight( int idx ) const                                { CHECK( idx >= m_tileRowHeight.size(), "Tile row index exceeds valid range" );                   return  m_tileRowHeight[idx];   }
  uint32_t               getTileColumnBd( int idx ) const                                 { CHECK( idx >= m_tileColBd.size(), "Tile column index exceeds valid range" );                    return  m_tileColBd[idx];       }
  uint32_t               getTileRowBd( int idx ) const                                    { CHECK( idx >= m_tileRowBd.size(), "Tile row index exceeds valid range" );                       return  m_tileRowBd[idx];       }
  uint32_t               ctuToTileCol( int ctuX ) const                                   { CHECK( ctuX >= m_ctuToTileCol.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileCol[ctuX];                  }
  uint32_t               ctuToTileRow( int ctuY ) const                                   { CHECK( ctuY >= m_ctuToTileRow.size(), "CTU address index exceeds valid range" ); return  m_ctuToTileRow[ctuY];                  }
  uint32_t               ctuToTileColBd( int ctuX ) const                                 { return  getTileColumnBd(ctuToTileCol( ctuX ));                                                                                  }
  uint32_t               ctuToTileRowBd( int ctuY ) const                                 { return  getTileRowBd(ctuToTileRow( ctuY ));                                                                                     }
  bool                   ctuIsTileColBd( int ctuX ) const                                 { return  ctuX == ctuToTileColBd( ctuX );                                                                                         }
  bool                   ctuIsTileRowBd( int ctuY ) const                                 { return  ctuY == ctuToTileRowBd( ctuY );                                                                                         }
  uint32_t               getTileIdx( uint32_t ctuX, uint32_t ctuY ) const                 { return (ctuToTileRow( ctuY ) * getNumTileColumns()) + ctuToTileCol( ctuX );                                                     }
  uint32_t               getTileIdx( uint32_t ctuRsAddr) const                            { return getTileIdx( ctuRsAddr % m_picWidthInCtu,  ctuRsAddr / m_picWidthInCtu );                                                 }
  uint32_t               getTileIdx( const Position& pos ) const                          { return getTileIdx( pos.x / m_ctuSize, pos.y / m_ctuSize );                                                                      }
  void                   setRectSliceFlag( bool b )                                       { m_rectSliceFlag = b;                                                                                                            }
  bool                   getRectSliceFlag( ) const                                        { return  m_rectSliceFlag;                                                                                                        }
  void                   setSingleSlicePerSubPicFlag( bool b )                            { m_singleSlicePerSubPicFlag = b;                                                                                                 }
  bool                   getSingleSlicePerSubPicFlag( ) const                             { return  m_singleSlicePerSubPicFlag;                                                                                             }
  uint32_t               getCtuToSubPicIdx( int idx ) const                               { CHECK( idx >= m_ctuToSubPicIdx.size(), "CTU address index exceeds valid range" ); CHECK( getNumSubPics() < 1, "Number of subpicture cannot be 0" ); return  m_ctuToSubPicIdx[ idx ]; }
  void                   setNumSlicesInPic( uint32_t u )                                  { CHECK( u > MAX_SLICES, "Number of slices in picture exceeds valid range" ); m_numSlicesInPic = u;                               }
  uint32_t               getNumSlicesInPic( ) const                                       { return  m_numSlicesInPic;                                                                                                       }
  void                   setTileIdxDeltaPresentFlag( bool b )                             { m_tileIdxDeltaPresentFlag = b;                                                                                                  }
  bool                   getTileIdxDeltaPresentFlag( ) const                              { return  m_tileIdxDeltaPresentFlag;                                                                                              }
  void                   setSliceWidthInTiles( int idx, uint32_t u )                      { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceWidthInTiles( u );            }
  uint32_t               getSliceWidthInTiles( int idx ) const                            { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceWidthInTiles( );      }
  void                   setSliceHeightInTiles( int idx, uint32_t u )                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInTiles( u );           }
  uint32_t               getSliceHeightInTiles( int idx ) const                           { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInTiles( );     }
  void                   setNumSlicesInTile( int idx, uint32_t u )                        { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setNumSlicesInTile( u );              }
  uint32_t               getNumSlicesInTile( int idx ) const                              { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getNumSlicesInTile( );        }
  void                   setSliceHeightInCtu( int idx, uint32_t u )                       { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setSliceHeightInCtu( u );             }
  uint32_t               getSliceHeightInCtu( int idx ) const                             { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getSliceHeightInCtu( );       }
  void                   setSliceTileIdx(  int idx, uint32_t u )                          { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    m_rectSlices[idx].setTileIdx( u );                      }
  uint32_t               getSliceTileIdx( int idx ) const                                 { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return  m_rectSlices[idx].getTileIdx( );                }
  void                   setRectSlices( std::vector<RectSlice> rectSlices )               { m_rectSlices = rectSlices;                                                                                                      }
  void                   setLoopFilterAcrossTilesEnabledFlag( bool b )                    { m_loopFilterAcrossTilesEnabledFlag = b;                                                                                         }
  bool                   getLoopFilterAcrossTilesEnabledFlag( ) const                     { return  m_loopFilterAcrossTilesEnabledFlag;                                                                                     }
  void                   setLoopFilterAcrossSlicesEnabledFlag( bool b )                   { m_loopFilterAcrossSlicesEnabledFlag = b;                                                                                        }
  bool                   getLoopFilterAcrossSlicesEnabledFlag( ) const                    { return  m_loopFilterAcrossSlicesEnabledFlag;                                                                                    }
  void                   resetTileSliceInfo();
  void                   initTiles();
  void                   initRectSlices();
  void                   initRectSliceMap(const SPS  *sps);
  std::vector<SubPic>    getSubPics()  const                                              {return m_subPics;          };
  SubPic                 getSubPic(uint32_t idx) const                                    { return m_subPics[idx]; }
  void                   initSubPic(const SPS &sps);
  const SubPic&          getSubPicFromPos(const Position& pos)  const;
  const SubPic&          getSubPicFromCU (const CodingUnit& cu) const;
  void                   initRasterSliceMap( std::vector<uint32_t> sizes );
  void                   checkSliceMap();
  SliceMap               getSliceMap( int idx ) const                                     { CHECK( idx >= m_numSlicesInPic, "Slice index exceeds valid range" );    return m_sliceMap[idx];                             }



  void                   setCabacInitPresentFlag( bool flag )                             { m_cabacInitPresentFlag = flag;                }
  bool                   getCabacInitPresentFlag() const                                  { return m_cabacInitPresentFlag;                }
#if JVET_AC0189_SGPM_NO_BLENDING
  void                   setUseSgpmNoBlend( bool b)                                               { m_sgpmNoBlend = b;                                    }
  bool                   getUseSgpmNoBlend() const                                                { return m_sgpmNoBlend;                                 }
#endif
#if JVET_V0094_BILATERAL_FILTER
  void                   setUseBIF( bool b)                                               { m_BIF = b;                                    }
  bool                   getUseBIF() const                                                { return m_BIF;                                 }
  void                   setBIFStrength( int val)                                         { m_BIFStrength = val;                          }
  int                    getBIFStrength() const                                           { return m_BIFStrength;                         }
  void                   setBIFQPOffset( int val)                                         { m_BIFQPOffset = val;                          }
  int                    getBIFQPOffset() const                                           { return m_BIFQPOffset;                         }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  void                   setUseChromaBIF( bool b)                                         { m_chromaBIF = b;             }
  bool                   getUseChromaBIF() const                                          { return m_chromaBIF;          }
  void                   setChromaBIFStrength( int val)                                   { m_chromaBIFStrength = val;   }
  int                    getChromaBIFStrength() const                                     { return m_chromaBIFStrength;  }
  void                   setChromaBIFQPOffset( int val)                                   { m_chromaBIFQPOffset = val;   }
  int                    getChromaBIFQPOffset() const                                     { return m_chromaBIFQPOffset;  }
#endif
  void                   setDeblockingFilterControlPresentFlag( bool val )                { m_deblockingFilterControlPresentFlag = val;   }
  bool                   getDeblockingFilterControlPresentFlag() const                    { return m_deblockingFilterControlPresentFlag;  }
  void                   setDeblockingFilterOverrideEnabledFlag( bool val )               { m_deblockingFilterOverrideEnabledFlag = val;  }
  bool                   getDeblockingFilterOverrideEnabledFlag() const                   { return m_deblockingFilterOverrideEnabledFlag; }
  void                   setPPSDeblockingFilterDisabledFlag(bool val)                     { m_ppsDeblockingFilterDisabledFlag = val;      } //!< set offset for deblocking filter disabled
  bool                   getPPSDeblockingFilterDisabledFlag() const                       { return m_ppsDeblockingFilterDisabledFlag;     } //!< get offset for deblocking filter disabled
#if DB_PARAM_TID
  void                   setDeblockingFilterBetaOffsetDiv2(std::vector<int> val)          { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  std::vector<int>       getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  void                   setDeblockingFilterTcOffsetDiv2(std::vector<int> val)            { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  std::vector<int>       getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
#else
  void                   setDeblockingFilterBetaOffsetDiv2(int val)                       { m_deblockingFilterBetaOffsetDiv2 = val;       } //!< set beta offset for deblocking filter
  int                    getDeblockingFilterBetaOffsetDiv2() const                        { return m_deblockingFilterBetaOffsetDiv2;      } //!< get beta offset for deblocking filter
  void                   setDeblockingFilterTcOffsetDiv2(int val)                         { m_deblockingFilterTcOffsetDiv2 = val;         } //!< set tc offset for deblocking filter
  int                    getDeblockingFilterTcOffsetDiv2() const                          { return m_deblockingFilterTcOffsetDiv2;        } //!< get tc offset for deblocking filter
#endif

  void                   setDeblockingFilterCbBetaOffsetDiv2(int val)                     { m_deblockingFilterCbBetaOffsetDiv2 = val;     } //!< set beta offset for Cb deblocking filter
  int                    getDeblockingFilterCbBetaOffsetDiv2() const                      { return m_deblockingFilterCbBetaOffsetDiv2;    } //!< get beta offset for Cb deblocking filter
  void                   setDeblockingFilterCbTcOffsetDiv2(int val)                       { m_deblockingFilterCbTcOffsetDiv2 = val;       } //!< set tc offset for Cb deblocking filter
  int                    getDeblockingFilterCbTcOffsetDiv2() const                        { return m_deblockingFilterCbTcOffsetDiv2;      } //!< get tc offset for Cb deblocking filter
  void                   setDeblockingFilterCrBetaOffsetDiv2(int val)                     { m_deblockingFilterCrBetaOffsetDiv2 = val;     } //!< set beta offset for Cr deblocking filter
  int                    getDeblockingFilterCrBetaOffsetDiv2() const                      { return m_deblockingFilterCrBetaOffsetDiv2;    } //!< get beta offset for Cr deblocking filter
  void                   setDeblockingFilterCrTcOffsetDiv2(int val)                       { m_deblockingFilterCrTcOffsetDiv2 = val;       } //!< set tc offset for Cr deblocking filter
  int                    getDeblockingFilterCrTcOffsetDiv2() const                        { return m_deblockingFilterCrTcOffsetDiv2;      } //!< get tc offset for Cr deblocking filter
  bool                   getListsModificationPresentFlag() const                          { return m_listsModificationPresentFlag;        }
  void                   setListsModificationPresentFlag( bool b )                        { m_listsModificationPresentFlag = b;           }
  bool                   getPictureHeaderExtensionPresentFlag() const                     { return m_pictureHeaderExtensionPresentFlag;     }
  void                   setPictureHeaderExtensionPresentFlag(bool val)                   { m_pictureHeaderExtensionPresentFlag = val;      }
  bool                   getSliceHeaderExtensionPresentFlag() const                       { return m_sliceHeaderExtensionPresentFlag;     }
  void                   setSliceHeaderExtensionPresentFlag(bool val)                     { m_sliceHeaderExtensionPresentFlag = val;      }

  void                   setRplInfoInPhFlag(bool flag)                                    { m_rplInfoInPhFlag = flag;                     }
  bool                   getRplInfoInPhFlag() const                                       { return m_rplInfoInPhFlag;                     }
  void                   setDbfInfoInPhFlag(bool flag)                                    { m_dbfInfoInPhFlag = flag;                     }
  bool                   getDbfInfoInPhFlag() const                                       { return m_dbfInfoInPhFlag;                     }
  void                   setSaoInfoInPhFlag(bool flag)                                    { m_saoInfoInPhFlag = flag;                     }
  bool                   getSaoInfoInPhFlag() const                                       { return m_saoInfoInPhFlag;                     }
  void                   setAlfInfoInPhFlag(bool flag)                                    { m_alfInfoInPhFlag = flag;                     }
  bool                   getAlfInfoInPhFlag() const                                       { return m_alfInfoInPhFlag;                     }
  void                   setWpInfoInPhFlag(bool flag)                                     { m_wpInfoInPhFlag = flag;                      }
  bool                   getWpInfoInPhFlag() const                                        { return m_wpInfoInPhFlag;                      }
  void                   setQpDeltaInfoInPhFlag(bool flag)                                { m_qpDeltaInfoInPhFlag = flag;                 }
  bool                   getQpDeltaInfoInPhFlag() const                                   { return m_qpDeltaInfoInPhFlag; }


  void                    setPicWidthInLumaSamples( uint32_t u )                          { m_picWidthInLumaSamples = u; }
  uint32_t                getPicWidthInLumaSamples() const                                { return  m_picWidthInLumaSamples; }
  void                    setPicHeightInLumaSamples( uint32_t u )                         { m_picHeightInLumaSamples = u; }
  uint32_t                getPicHeightInLumaSamples() const                               { return  m_picHeightInLumaSamples; }

  void                    setConformanceWindowFlag(bool flag)                             { m_conformanceWindowFlag = flag; }
  bool                    getConformanceWindowFlag() const                                { return m_conformanceWindowFlag; }
  Window&                 getConformanceWindow()                                          { return  m_conformanceWindow; }
  const Window&           getConformanceWindow() const                                    { return  m_conformanceWindow; }
  void                    setConformanceWindow( Window& conformanceWindow )               { m_conformanceWindow = conformanceWindow; }

  Window&                 getScalingWindow()                                              { return  m_scalingWindow; }
  const Window&           getScalingWindow()                                        const { return  m_scalingWindow; }
  void                    setScalingWindow( Window& scalingWindow )                       { m_scalingWindow = scalingWindow; }

  int                     getMixedNaluTypesInPicFlag() const                              { return m_mixedNaluTypesInPicFlag; }
  void                    setMixedNaluTypesInPicFlag( const bool flag )                   { m_mixedNaluTypesInPicFlag = flag; }
#if JVET_AB0171_ASYMMETRIC_DB_FOR_GDR
private:
  bool                         m_asymmetricILF;
public:
  bool getAsymmetricILF() const  { return m_asymmetricILF; }
  void setAsymmetricILF(bool b)  { m_asymmetricILF = b; }
#endif
};

class APS
{
private:
  int                    m_APSId;                    // adaptation_parameter_set_id
  int                    m_temporalId;
  int                    m_puCounter;
  int                    m_layerId;
  ApsType                m_APSType;                  // aps_params_type
  AlfParam               m_alfAPSParam;
  SliceReshapeInfo       m_reshapeAPSInfo;
  ScalingList            m_scalingListApsInfo;
  CcAlfFilterParam       m_ccAlfAPSParam;
  bool                   m_hasPrefixNalUnitType;

#if JVET_AK0065_TALF
  TAlfFilterParam        m_talfAPSParam;
#endif
public:
  APS();
  virtual                ~APS();

  int                    getAPSId() const                                                 { return m_APSId;                               }
  void                   setAPSId(int i)                                                  { m_APSId = i;                                  }

  ApsType                getAPSType() const                                               { return m_APSType;                             }
  void                   setAPSType( ApsType type )                                       { m_APSType = type;                             }

  void                   setAlfAPSParam(AlfParam& alfAPSParam)                            { m_alfAPSParam = alfAPSParam;                  }
  void                   setTemporalId( int i )                                           { m_temporalId = i;                             }
  int                    getTemporalId()                                            const { return m_temporalId;                          }
  void                   setPuCounter(int i)                                              { m_puCounter = i;                              }
  int                    getPuCounter()                                             const { return m_puCounter;                           }
  void                   setLayerId( int i )                                              { m_layerId = i;                                }
  int                    getLayerId()                                               const { return m_layerId;                             }
  AlfParam&              getAlfAPSParam()  { return m_alfAPSParam; }

  void                   setReshaperAPSInfo(SliceReshapeInfo& reshapeAPSInfo)             { m_reshapeAPSInfo = reshapeAPSInfo;            }
  SliceReshapeInfo&      getReshaperAPSInfo()                                             { return m_reshapeAPSInfo;                      }
  void                   setScalingList( ScalingList& scalingListAPSInfo )                { m_scalingListApsInfo = scalingListAPSInfo;    }
  ScalingList&           getScalingList()                                                 { return m_scalingListApsInfo;                  }
  void                   setCcAlfAPSParam(CcAlfFilterParam& ccAlfAPSParam)                { m_ccAlfAPSParam = ccAlfAPSParam;              }
  CcAlfFilterParam&      getCcAlfAPSParam()  { return m_ccAlfAPSParam; }
  void                   setHasPrefixNalUnitType( bool b )                                { m_hasPrefixNalUnitType = b;                   }
  bool                   getHasPrefixNalUnitType() const                                  { return m_hasPrefixNalUnitType;                }
#if JVET_R0433
  bool chromaPresentFlag;
#endif
#if JVET_AK0065_TALF
  void                   setTAlfAPSParam(TAlfFilterParam &param)                          { m_talfAPSParam = param;                }
  TAlfFilterParam&       getTAlfAPSParam()                                                { return m_talfAPSParam;                        }
#endif
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool     presentFlag;
  uint32_t log2WeightDenom;
  int      codedWeight;
  int      codedOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int  w;
  int  o;
  int  offset;
  int  shift;
  int  round;

  static bool isWeighted(const WPScalingParam *wp);
};

inline bool WPScalingParam::isWeighted(const WPScalingParam *wp)
{
  return wp != nullptr && (wp[COMPONENT_Y].presentFlag || wp[COMPONENT_Cb].presentFlag || wp[COMPONENT_Cr].presentFlag);
}

struct WPACDCParam
{
  int64_t iAC;
  int64_t iDC;
};

// picture header class
class PicHeader
{
private:
  bool                        m_valid;                                                  //!< picture header is valid yet or not
  Picture*                    m_pcPic;                                                  //!< pointer to picture structure
  int                         m_pocLsb;                                                 //!< least significant bits of picture order count
  bool                        m_nonReferencePictureFlag;                                //!< non-reference picture flag
  bool                        m_gdrOrIrapPicFlag;                                       //!< gdr or irap picture flag
  bool                        m_gdrPicFlag;                                             //!< gradual decoding refresh picture flag
#if !JVET_S0193_NO_OUTPUT_PRIOR_PIC
  bool                        m_noOutputOfPriorPicsFlag;                                //!< no output of prior pictures flag
#endif
#if JVET_Z0118_GDR
  bool                        m_inGdrInterval;
  bool                        m_gdrRecoveryPocPic;
  int                         m_lastGdrIntervalPoc;
  int                         m_gdrBegX;
  int                         m_gdrEndX;
#endif  
  uint32_t                    m_recoveryPocCnt;                                         //!< recovery POC count
  bool                        m_noOutputBeforeRecoveryFlag;                             //!< NoOutputBeforeRecoveryFlag
  bool                        m_handleCraAsCvsStartFlag;                                //!< HandleCraAsCvsStartFlag
  bool                        m_handleGdrAsCvsStartFlag;                                //!< HandleGdrAsCvsStartFlag
  int                         m_spsId;                                                  //!< sequence parameter set ID
  int                         m_ppsId;                                                  //!< picture parameter set ID
  bool                        m_pocMsbPresentFlag;                                      //!< ph_poc_msb_present_flag
  int                         m_pocMsbVal;                                              //!< poc_msb_val
  bool                        m_virtualBoundariesEnabledFlag;                           //!< loop filtering across virtual boundaries disabled
  bool                        m_virtualBoundariesPresentFlag;                           //!< loop filtering across virtual boundaries disabled
  unsigned                    m_numVerVirtualBoundaries;                                //!< number of vertical virtual boundaries
  unsigned                    m_numHorVirtualBoundaries;                                //!< number of horizontal virtual boundaries
  unsigned                    m_virtualBoundariesPosX[3];                               //!< horizontal virtual boundary positions
  unsigned                    m_virtualBoundariesPosY[3];                               //!< vertical virtual boundary positions
  bool                        m_picOutputFlag;                                          //!< picture output flag
  const ReferencePictureList
    *m_pRPL0;   //!< pointer to RPL for L0, either in the SPS or the local RPS in the picture header
  const ReferencePictureList* m_pRPL1;                                                  //!< pointer to RPL for L1, either in the SPS or the local RPS in the picture header
  ReferencePictureList        m_localRPL0;                                              //!< RPL for L0 when present in picture header
  ReferencePictureList        m_localRPL1;                                              //!< RPL for L1 when present in picture header
  int                         m_rpl0Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  int                         m_rpl1Idx;                                                //!< index of used RPL in the SPS or -1 for local RPL in the picture header
  bool                        m_picInterSliceAllowedFlag;                               //!< inter slice allowed flag in PH
  bool                        m_picIntraSliceAllowedFlag;                               //!< intra slice allowed flag in PH
  bool                        m_splitConsOverrideFlag;                                  //!< partitioning constraint override flag
  uint32_t                    m_cuQpDeltaSubdivIntra;                                   //!< CU QP delta maximum subdivision for intra slices
  uint32_t                    m_cuQpDeltaSubdivInter;                                   //!< CU QP delta maximum subdivision for inter slices
  uint32_t                    m_cuChromaQpOffsetSubdivIntra;                            //!< CU chroma QP offset maximum subdivision for intra slices
  uint32_t                    m_cuChromaQpOffsetSubdivInter;                            //!< CU chroma QP offset maximum subdivision for inter slices
  bool                        m_enableTMVPFlag;                                         //!< enable temporal motion vector prediction
  bool                        m_picColFromL0Flag;                                       //!< syntax element collocated_from_l0_flag
  uint32_t                    m_colRefIdx;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  bool                        m_picColFromL0Flag2nd;                                    //!< syntax element collocated_from_l0_flag
  uint32_t                    m_colRefIdx2nd;
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  uint32_t                    m_costForARMC;                                            //!< Cost for diversity criterion
#endif
  bool                        m_mvdL1ZeroFlag;                                          //!< L1 MVD set to zero flag
  uint32_t                    m_maxNumAffineMergeCand;                                  //!< max number of sub-block merge candidates
#if JVET_AG0276_LIC_FLAG_SIGNALING
  uint32_t                    m_maxNumAffineOppositeLicMergeCand;                       //!< max number of sub-block merge candidates with opposite LIC flag
#endif
  bool                        m_disFracMMVD;                                            //!< fractional MMVD offsets disabled flag
  bool                        m_disBdofFlag;                                            //!< picture level BDOF disable flag
  bool                        m_disDmvrFlag;                                            //!< picture level DMVR disable flag
  bool                        m_disProfFlag;                                            //!< picture level PROF disable flag
  bool                        m_jointCbCrSignFlag;                                      //!< joint Cb/Cr residual sign flag
#if JVET_W0097_GPM_MMVD_TM
  bool                        m_gpmMMVDTableFlag;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool                        m_disFracMBVD;
#endif
  int                         m_qpDelta;                                                //!< value of Qp delta
  bool                        m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];                   //!< sao enabled flags for each channel
#if JVET_W0066_CCSAO
  bool                        m_ccSaoEnabledFlag[MAX_NUM_COMPONENT];
#endif
#if ALF_IMPROVEMENT
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  bool                        m_alfReuseFlag[MAX_NUM_COMPONENT];
  bool                        m_ccalfReuseFlag[MAX_NUM_COMPONENT];
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  int                         m_alfFixedFilterSetIdx[MAX_NUM_COMPONENT];
#else
  int                         m_alfFixedFilterSetIdx;
#endif
#endif
  bool                        m_alfEnabledFlag[MAX_NUM_COMPONENT];                      //!< alf enabled flags for each component
  int                         m_numAlfAps;                                              //!< number of alf aps active for the picture
  std::vector<int>            m_alfApsId;                                               //!< list of alf aps for the picture
  int                         m_alfChromaApsId;                                         //!< chroma alf aps ID
  bool m_ccalfEnabledFlag[MAX_NUM_COMPONENT];
  int  m_ccalfCbApsId;
  int  m_ccalfCrApsId;
#if JVET_AK0065_TALF
  TAlfControl m_talfControl;
#endif
  bool                        m_deblockingFilterOverrideFlag;                           //!< deblocking filter override controls enabled
  bool                        m_deblockingFilterDisable;                                //!< deblocking filter disabled flag
  int                         m_deblockingFilterBetaOffsetDiv2;                         //!< beta offset for deblocking filter
  int                         m_deblockingFilterTcOffsetDiv2;                           //!< tc offset for deblocking filter
  int                         m_deblockingFilterCbBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCbTcOffsetDiv2;                         //!< tc offset for deblocking filter
  int                         m_deblockingFilterCrBetaOffsetDiv2;                       //!< beta offset for deblocking filter
  int                         m_deblockingFilterCrTcOffsetDiv2;                         //!< tc offset for deblocking filter
  bool                        m_lmcsEnabledFlag;                                        //!< lmcs enabled flag
  int                         m_lmcsApsId;                                              //!< lmcs APS ID
  APS*                        m_lmcsAps;                                                //!< lmcs APS
  bool                        m_lmcsChromaResidualScaleFlag;                            //!< lmcs chroma residual scale flag
  bool                        m_explicitScalingListEnabledFlag;                         //!< explicit quantization scaling list enabled
  int                         m_scalingListApsId;                                       //!< quantization scaling list APS ID
  APS*                        m_scalingListAps;                                         //!< quantization scaling list APS
  unsigned                    m_minQT[3];                                               //!< minimum quad-tree size  0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned                    m_maxMTTHierarchyDepth[3];                                //!< maximum MTT depth
  unsigned                    m_maxBTSize[3];                                           //!< maximum BT size
  unsigned                    m_maxTTSize[3];                                           //!< maximum TT size

  WPScalingParam              m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT];   // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  int                         m_numL0Weights;                                           //!< number of weights for L0 list
  int                         m_numL1Weights;                                           //!< number of weights for L1 list
#if JVET_AL0181_ASBT
  bool                        m_asbt;
#endif

public:
                              PicHeader();
  virtual                     ~PicHeader();
  void                        initPicHeader();
  bool                        isValid()                                                 { return m_valid;                                                                              }
  void                        setValid()                                                { m_valid = true;                                                                              }
  void                        setPic( Picture* p )                                      { m_pcPic = p;                                                                                 }
  Picture*                    getPic()                                                  { return m_pcPic;                                                                              }
  const Picture*              getPic() const                                            { return m_pcPic;                                                                              }
  void                        setPocLsb(int i)                                          { m_pocLsb = i;                                                                                }
  int                         getPocLsb()                                               { return m_pocLsb;                                                                             }
  void                        setNonReferencePictureFlag( bool b )                      { m_nonReferencePictureFlag = b;                                                               }
  bool                        getNonReferencePictureFlag() const                        { return m_nonReferencePictureFlag;                                                            }
  void                        setGdrOrIrapPicFlag( bool b )                             { m_gdrOrIrapPicFlag = b;                                                                      }
  bool                        getGdrOrIrapPicFlag() const                               { return m_gdrOrIrapPicFlag;                                                                   }
  void                        setGdrPicFlag( bool b )                                   { m_gdrPicFlag = b;                                                                            }
  bool                        getGdrPicFlag() const                                     { return m_gdrPicFlag;                                                                         }
#if !JVET_S0193_NO_OUTPUT_PRIOR_PIC
  void                        setNoOutputOfPriorPicsFlag( bool b )                      { m_noOutputOfPriorPicsFlag = b;                                                               }
  bool                        getNoOutputOfPriorPicsFlag() const                        { return m_noOutputOfPriorPicsFlag;                                                            }
#endif
#if JVET_Z0118_GDR
  void                        setInGdrInterval(bool b)                                  { m_inGdrInterval = b;                                                                         }
  bool                        getInGdrInterval() const                                  { return m_inGdrInterval;                                                                      }  
  void                        setGdrBegX(int x)                                         { m_gdrBegX = x; }
  int                         getGdrBegX()                                              { return m_gdrBegX; }
  void                        setGdrEndX(int x)                                         { m_gdrEndX = x; }
  int                         getGdrEndX()                                              { return m_gdrEndX; }
  void                        setIsGdrRecoveryPocPic(bool b)                            { m_gdrRecoveryPocPic = b; }
  bool                        getIsGdrRecoveryPocPic() const                            { return m_gdrRecoveryPocPic; }
#endif
  void                        setRecoveryPocCnt( uint32_t u )                           { m_recoveryPocCnt = u;                                                                        }
  uint32_t                    getRecoveryPocCnt() const                                 { return m_recoveryPocCnt;                                                                     }
  void                        setSPSId( uint32_t u )                                    { m_spsId = u;                                                                                 }
  uint32_t                    getSPSId() const                                          { return m_spsId;                                                                              }
  void                        setPPSId( uint32_t u )                                    { m_ppsId = u;                                                                                 }
  uint32_t                    getPPSId() const                                          { return m_ppsId;                                                                              }
  void                        setPocMsbPresentFlag(bool b)                              { m_pocMsbPresentFlag = b;                                                                     }
  bool                        getPocMsbPresentFlag() const                              { return m_pocMsbPresentFlag;                                                                  }
  void                        setPocMsbVal(int i)                                       { m_pocMsbVal = i;                                                                             }
  int                         getPocMsbVal()                                            { return m_pocMsbVal;                                                                          }
  void                        setVirtualBoundariesPresentFlag( bool b )                 { m_virtualBoundariesPresentFlag = b;                                                          }
  bool                        getVirtualBoundariesPresentFlag() const                   { return m_virtualBoundariesPresentFlag;                                                       }
  void                        setNumVerVirtualBoundaries(unsigned u)                    { m_numVerVirtualBoundaries = u;                                                               }
  unsigned                    getNumVerVirtualBoundaries() const                        { return m_numVerVirtualBoundaries;                                                            }
  void                        setNumHorVirtualBoundaries(unsigned u)                    { m_numHorVirtualBoundaries = u;                                                               }
  unsigned                    getNumHorVirtualBoundaries() const                        { return m_numHorVirtualBoundaries;                                                            }
  void                        setVirtualBoundariesPosX(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosX[idx] = u;   }
  unsigned                    getVirtualBoundariesPosX(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosX[idx];}
  void                        setVirtualBoundariesPosY(unsigned u, unsigned idx)        { CHECK( idx >= 3, "boundary index exceeds valid range" ); m_virtualBoundariesPosY[idx] = u;   }
  unsigned                    getVirtualBoundariesPosY(unsigned idx) const              { CHECK( idx >= 3, "boundary index exceeds valid range" ); return m_virtualBoundariesPosY[idx];}
  void                        setPicOutputFlag( bool b )                                { m_picOutputFlag = b;                                                                         }
  bool                        getPicOutputFlag() const                                  { return m_picOutputFlag;                                                                      }
  void                        setRPL( bool b, const ReferencePictureList *pcRPL)        { if(b==1) { m_pRPL1 = pcRPL; } else { m_pRPL0 = pcRPL; }                                      }
  const ReferencePictureList* getRPL( bool b )                                          { return b==1 ? m_pRPL1 : m_pRPL0;                                                             }
  ReferencePictureList*       getLocalRPL( bool b )                                     { return b==1 ? &m_localRPL1 : &m_localRPL0;                                                   }
  void                        setRPLIdx( bool b, int rplIdx)                            { if(b==1) { m_rpl1Idx = rplIdx; } else { m_rpl0Idx = rplIdx; }                                }
  int                         getRPLIdx( bool b ) const                                 { return b==1 ? m_rpl1Idx : m_rpl0Idx;                                                         }
  void                        setRPL0(const ReferencePictureList *pcRPL)                { m_pRPL0 = pcRPL;                                                                             }
  void                        setRPL1(const ReferencePictureList *pcRPL)                { m_pRPL1 = pcRPL;                                                                             }
  const ReferencePictureList* getRPL0()                                                 { return m_pRPL0;                                                                              }
  const ReferencePictureList* getRPL1()                                                 { return m_pRPL1;                                                                              }
  ReferencePictureList*       getLocalRPL0()                                            { return &m_localRPL0;                                                                         }
  ReferencePictureList*       getLocalRPL1()                                            { return &m_localRPL1;                                                                         }
  void                        setRPL0idx(int rplIdx)                                    { m_rpl0Idx = rplIdx;                                                                          }
  void                        setRPL1idx(int rplIdx)                                    { m_rpl1Idx = rplIdx;                                                                          }
  int                         getRPL0idx() const                                        { return m_rpl0Idx;                                                                            }
  int                         getRPL1idx() const                                        { return m_rpl1Idx;                                                                            }
  void                        setPicInterSliceAllowedFlag(bool b)                       { m_picInterSliceAllowedFlag = b; }
  bool                        getPicInterSliceAllowedFlag() const                       { return m_picInterSliceAllowedFlag; }
  void                        setPicIntraSliceAllowedFlag(bool b)                       { m_picIntraSliceAllowedFlag = b; }
  bool                        getPicIntraSliceAllowedFlag() const                       { return m_picIntraSliceAllowedFlag; }
  void                        setSplitConsOverrideFlag( bool b )                        { m_splitConsOverrideFlag = b;                                                                 }
  bool                        getSplitConsOverrideFlag() const                          { return m_splitConsOverrideFlag;                                                              }
  void                        setCuQpDeltaSubdivIntra( uint32_t u )                     { m_cuQpDeltaSubdivIntra = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivIntra() const                           { return m_cuQpDeltaSubdivIntra;                                                               }
  void                        setCuQpDeltaSubdivInter( uint32_t u )                     { m_cuQpDeltaSubdivInter = u;                                                                  }
  uint32_t                    getCuQpDeltaSubdivInter() const                           { return m_cuQpDeltaSubdivInter;                                                               }
  void                        setCuChromaQpOffsetSubdivIntra( uint32_t u )              { m_cuChromaQpOffsetSubdivIntra = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivIntra() const                    { return m_cuChromaQpOffsetSubdivIntra;                                                        }
  void                        setCuChromaQpOffsetSubdivInter( uint32_t u )              { m_cuChromaQpOffsetSubdivInter = u;                                                           }
  uint32_t                    getCuChromaQpOffsetSubdivInter() const                    { return m_cuChromaQpOffsetSubdivInter;                                                        }
  void                        setEnableTMVPFlag( bool b )                               { m_enableTMVPFlag = b;                                                                        }
  bool                        getEnableTMVPFlag() const                                 { return m_enableTMVPFlag;                                                                     }
  void                        setPicColFromL0Flag(bool val)                             { m_picColFromL0Flag = val;                                                                     }
  bool                        getPicColFromL0Flag() const                               { return m_picColFromL0Flag;                                                                    }
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                                                       }
  uint32_t                    getColRefIdx()                                             { return m_colRefIdx;                                                                         }
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AJ0237_INTERNAL_12BIT
  void                        setCostForARMC(uint32_t cost, int bitDepth)               { m_costForARMC = (cost << (std::max(0, bitDepth - 10)));                                      }
#else
  void                        setCostForARMC(uint32_t cost)                             { m_costForARMC = cost;                                                                        }
#endif
  uint32_t                    getCostForARMC()                                          { return m_costForARMC;                                                                        }
#if JVET_AJ0237_INTERNAL_12BIT
  uint32_t                    getCostForARMC(int bitDepth)                              { return m_costForARMC >> (std::max(0, bitDepth - 10));                                        } // for header parsing/writing purpose
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void                        setPicColFromL0Flag2nd(bool val)                          { m_picColFromL0Flag2nd = val;                                                                 }
  bool                        getPicColFromL0Flag2nd() const                            { return m_picColFromL0Flag2nd;                                                                }
  void                        setColRefIdx2nd(uint32_t refIdx)                          { m_colRefIdx2nd = refIdx;                                                                     }
  uint32_t                    getColRefIdx2nd()                                         { return m_colRefIdx2nd;                                                                       }
#endif
  void                        setMvdL1ZeroFlag( bool b )                                { m_mvdL1ZeroFlag = b;                                                                         }
  bool                        getMvdL1ZeroFlag() const                                  { return m_mvdL1ZeroFlag;                                                                      }
  void                        setMaxNumAffineMergeCand( uint32_t val )                  { m_maxNumAffineMergeCand = val;                                                               }
  uint32_t                    getMaxNumAffineMergeCand() const                          { return m_maxNumAffineMergeCand;                                                              }
#if JVET_AG0276_LIC_FLAG_SIGNALING
  void                        setMaxNumAffineOppositeLicMergeCand( uint32_t val )       { m_maxNumAffineOppositeLicMergeCand = val; }
  uint32_t                    getMaxNumAffineOppositeLicMergeCand() const               { return m_maxNumAffineOppositeLicMergeCand; }
#endif
  void                        setDisFracMMVD( bool val )                                { m_disFracMMVD = val;                                                                         }
  bool                        getDisFracMMVD() const                                    { return m_disFracMMVD;                                                                        }
  void                        setDisBdofFlag( bool val )                                { m_disBdofFlag = val;                                                                         }
  bool                        getDisBdofFlag() const                                    { return m_disBdofFlag;                                                                        }
  void                        setDisDmvrFlag( bool val )                                { m_disDmvrFlag = val;                                                                         }
  bool                        getDisDmvrFlag() const                                    { return m_disDmvrFlag;                                                                        }
  void                        setDisProfFlag( bool val )                                { m_disProfFlag = val;                                                                         }
  bool                        getDisProfFlag() const                                    { return m_disProfFlag;                                                                        }
  void                        setJointCbCrSignFlag( bool b )                            { m_jointCbCrSignFlag = b;                                                                     }
  bool                        getJointCbCrSignFlag() const                              { return m_jointCbCrSignFlag;                                                                  }
#if JVET_W0097_GPM_MMVD_TM
  void                        setGPMMMVDTableFlag(bool b)                               { m_gpmMMVDTableFlag = b;                                                                      }
  bool                        getGPMMMVDTableFlag() const                               { return m_gpmMMVDTableFlag;                                                                   }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void                        setDisFracMBVD(bool b)                                    { m_disFracMBVD = b;                                                                           }
  bool                        getDisFracMBVD() const                                    { return m_disFracMBVD;                                                                        }
#endif
  void                        setQpDelta(int b)                                         { m_qpDelta = b;                                                                               }
  int                         getQpDelta() const                                        { return m_qpDelta;                                                                            }
  void                        setSaoEnabledFlag(ChannelType chType, bool b)             { m_saoEnabledFlag[chType] = b;                                                                }
  bool                        getSaoEnabledFlag(ChannelType chType) const               { return m_saoEnabledFlag[chType];                                                             }
#if JVET_W0066_CCSAO
  void                        setCcSaoEnabledFlag(ComponentID compId, bool b)           { m_ccSaoEnabledFlag[compId] = b;                                                              }
  bool                        getCcSaoEnabledFlag(ComponentID compId) const             { return m_ccSaoEnabledFlag[compId];                                                           }
#endif
#if ALF_IMPROVEMENT
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  void                        setAlfReuseFlag( ComponentID compId, bool b ) { m_alfReuseFlag[compId] = b; }
  bool                        getAlfReuseFlag( ComponentID compId )  { return m_alfReuseFlag[compId]; }
  void                        setCcalfReuseFlag( ComponentID compId, bool b ) { m_ccalfReuseFlag[compId] = b; }
  bool                        getCcalfReuseFlag( ComponentID compId )  { return m_ccalfReuseFlag[compId]; }
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  void                        setAlfFixedFilterSetIdx( ComponentID compId, int i )                            { m_alfFixedFilterSetIdx[compId] = i;                                                                  }
  int                         getAlfFixedFilterSetIdx( ComponentID compId ) const                           { return m_alfFixedFilterSetIdx[compId];                                                               }
#else
  void                        setAlfFixedFilterSetIdx(int i)                            { m_alfFixedFilterSetIdx = i;                                                                  }
  int                         getAlfFixedFilterSetIdx() const                           { return m_alfFixedFilterSetIdx;                                                               }
#endif
#endif
  void                        setAlfEnabledFlag(ComponentID compId, bool b)             { m_alfEnabledFlag[compId] = b;                                                                }
  bool                        getAlfEnabledFlag(ComponentID compId) const               { return m_alfEnabledFlag[compId];                                                             }
  void                        setNumAlfAps(int i)                                       { m_numAlfAps = i;                                                                             }
  int                         getNumAlfAps() const                                      { return m_numAlfAps;                                                                          }
  void                        setAlfApsIdChroma(int i)                                  { m_alfChromaApsId = i;                                                                        }
  int                         getAlfApsIdChroma() const                                 { return m_alfChromaApsId;                                                                     }
  void setCcAlfEnabledFlag(ComponentID compId, bool b) { m_ccalfEnabledFlag[compId] = b; }
  bool getCcAlfEnabledFlag(ComponentID compId) const { return m_ccalfEnabledFlag[compId]; }

  void setCcAlfCbApsId(int i) { m_ccalfCbApsId = i; }
  int  getCcAlfCbApsId() const { return m_ccalfCbApsId; }
  void setCcAlfCrApsId(int i) { m_ccalfCrApsId = i; }
  int  getCcAlfCrApsId() const { return m_ccalfCrApsId; }
#if JVET_AK0065_TALF
  void setTAlfControl(TAlfControl c) { m_talfControl = c;    }
  TAlfControl getTAlfControl() const { return m_talfControl; }
#endif
  void                        setDeblockingFilterOverrideFlag( bool b )                 { m_deblockingFilterOverrideFlag = b;                                                          }
  bool                        getDeblockingFilterOverrideFlag() const                   { return m_deblockingFilterOverrideFlag;                                                       }
  void                        setDeblockingFilterDisable( bool b )                      { m_deblockingFilterDisable= b;                                                                }
  bool                        getDeblockingFilterDisable() const                        { return m_deblockingFilterDisable;                                                            }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )                { m_deblockingFilterBetaOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const                  { return m_deblockingFilterBetaOffsetDiv2;                                                     }
  void                        setDeblockingFilterTcOffsetDiv2( int i )                  { m_deblockingFilterTcOffsetDiv2 = i;                                                          }
  int                         getDeblockingFilterTcOffsetDiv2() const                   { return m_deblockingFilterTcOffsetDiv2;                                                       }
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )              { m_deblockingFilterCbBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const                { return m_deblockingFilterCbBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )                { m_deblockingFilterCbTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCbTcOffsetDiv2() const                 { return m_deblockingFilterCbTcOffsetDiv2;                                                     }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )              { m_deblockingFilterCrBetaOffsetDiv2 = i;                                                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const                { return m_deblockingFilterCrBetaOffsetDiv2;                                                   }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )                { m_deblockingFilterCrTcOffsetDiv2 = i;                                                        }
  int                         getDeblockingFilterCrTcOffsetDiv2() const                 { return m_deblockingFilterCrTcOffsetDiv2;                                                     }
  void                        setLmcsEnabledFlag(bool b)                                { m_lmcsEnabledFlag = b;                                                                       }
  bool                        getLmcsEnabledFlag()                                      { return m_lmcsEnabledFlag;                                                                    }
  const bool                  getLmcsEnabledFlag() const                                { return m_lmcsEnabledFlag;                                                                    }
  void                        setLmcsAPS(APS* aps)                                      { m_lmcsAps = aps; m_lmcsApsId = (aps) ? aps->getAPSId() : -1;                                 }
  APS*                        getLmcsAPS() const                                        { return m_lmcsAps;                                                                            }
  void                        setLmcsAPSId(int id)                                      { m_lmcsApsId = id;                                                                            }
  int                         getLmcsAPSId() const                                      { return m_lmcsApsId;                                                                          }
  void                        setLmcsChromaResidualScaleFlag(bool b)                    { m_lmcsChromaResidualScaleFlag = b;                                                           }
  bool                        getLmcsChromaResidualScaleFlag()                          { return m_lmcsChromaResidualScaleFlag;                                                        }
  const bool                  getLmcsChromaResidualScaleFlag() const                    { return m_lmcsChromaResidualScaleFlag;                                                        }
  void                        setScalingListAPS( APS* aps )                             { m_scalingListAps = aps; m_scalingListApsId = ( aps ) ? aps->getAPSId() : -1;                 }
  APS*                        getScalingListAPS() const                                 { return m_scalingListAps;                                                                     }
  void                        setScalingListAPSId( int id )                             { m_scalingListApsId = id;                                                                     }
  int                         getScalingListAPSId() const                               { return m_scalingListApsId;                                                                   }
  void                        setExplicitScalingListEnabledFlag( bool b )               { m_explicitScalingListEnabledFlag = b;                                                        }
  bool                        getExplicitScalingListEnabledFlag()                       { return m_explicitScalingListEnabledFlag;                                                     }
  const bool                  getExplicitScalingListEnabledFlag() const                 { return m_explicitScalingListEnabledFlag;                                                     }

  unsigned*                   getMinQTSizes() const                                     { return (unsigned *)m_minQT;                                                                  }
  unsigned*                   getMaxMTTHierarchyDepths() const                          { return (unsigned *)m_maxMTTHierarchyDepth;                                                   }
  unsigned*                   getMaxBTSizes() const                                     { return (unsigned *)m_maxBTSize;                                                              }
  unsigned*                   getMaxTTSizes() const                                     { return (unsigned *)m_maxTTSize;                                                              }

  void                        setMinQTSize(unsigned idx, unsigned minQT)                { m_minQT[idx] = minQT;                                                                        }
  void                        setMaxMTTHierarchyDepth(unsigned idx, unsigned maxMTT)    { m_maxMTTHierarchyDepth[idx] = maxMTT;                                                        }
  void                        setMaxBTSize(unsigned idx, unsigned maxBT)                { m_maxBTSize[idx] = maxBT;                                                                    }
  void                        setMaxTTSize(unsigned idx, unsigned maxTT)                { m_maxTTSize[idx] = maxTT;                                                                    }

  void                        setMinQTSizes(unsigned*   minQT)                          { m_minQT[0] = minQT[0]; m_minQT[1] = minQT[1]; m_minQT[2] = minQT[2];                                                 }
  void                        setMaxMTTHierarchyDepths(unsigned*   maxMTT)              { m_maxMTTHierarchyDepth[0] = maxMTT[0]; m_maxMTTHierarchyDepth[1] = maxMTT[1]; m_maxMTTHierarchyDepth[2] = maxMTT[2]; }
  void                        setMaxBTSizes(unsigned*   maxBT)                          { m_maxBTSize[0] = maxBT[0]; m_maxBTSize[1] = maxBT[1]; m_maxBTSize[2] = maxBT[2];                                     }
  void                        setMaxTTSizes(unsigned*   maxTT)                          { m_maxTTSize[0] = maxTT[0]; m_maxTTSize[1] = maxTT[1]; m_maxTTSize[2] = maxTT[2];                                     }

  unsigned                    getMinQTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_minQT[0] : m_minQT[2]) : m_minQT[1];                                              }
  unsigned                    getMaxMTTHierarchyDepth(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxMTTHierarchyDepth[0] : m_maxMTTHierarchyDepth[2]) : m_maxMTTHierarchyDepth[1]; }
  unsigned                    getMaxBTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxBTSize[0] : m_maxBTSize[2]) : m_maxBTSize[1];                                  }
  unsigned                    getMaxTTSize(SliceType   slicetype,
                                       ChannelType chType = CHANNEL_TYPE_LUMA) const    { return slicetype == I_SLICE ? (chType == CHANNEL_TYPE_LUMA ? m_maxTTSize[0] : m_maxTTSize[2]) : m_maxTTSize[1];                                  }

#if JVET_AL0181_ASBT
  void                        setUseASBT(bool b)                                        { m_asbt = b;                                                                                  }
  bool                        getUseASBT() const                                        { return m_asbt;                                                                               }
#endif
  void                        setAlfAPSs(std::vector<int> apsIDs)                       { m_alfApsId.resize(m_numAlfAps);
                                                                                          for (int i = 0; i < m_numAlfAps; i++)
                                                                                          {
                                                                                            m_alfApsId[i] = apsIDs[i];
                                                                                          }
                                                                                        }

  std::vector<int>            getAlfAPSs() const                                        { return m_alfApsId; }

  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  const WPScalingParam *      getWpScaling(const RefPicList refPicList, const int refIdx) const;
  WPScalingParam *            getWpScaling(const RefPicList refPicList, const int refIdx);
  WPScalingParam*             getWpScalingAll()                                        { return (WPScalingParam *) m_weightPredTable; }
  void                        resetWpScaling();
  void                        setNumL0Weights(int b)                                   { m_numL0Weights = b;                          }
  int                         getNumL0Weights()                                        { return m_numL0Weights;                       }
  void                        setNumL1Weights(int b)                                   { m_numL1Weights = b;                          }
  int                         getNumL1Weights()                                        { return m_numL1Weights;                       }

  void                        setNoOutputBeforeRecoveryFlag( bool val )                { m_noOutputBeforeRecoveryFlag = val;  }
  bool                        getNoOutputBeforeRecoveryFlag() const                    { return m_noOutputBeforeRecoveryFlag; }
  void                        setHandleCraAsCvsStartFlag( bool val )                   { m_handleCraAsCvsStartFlag = val;     }
  bool                        getHandleCraAsCvsStartFlag() const                       { return m_handleCraAsCvsStartFlag;    }
  void                        setHandleGdrAsCvsStartFlag( bool val )                   { m_handleGdrAsCvsStartFlag = val;     }
  bool                        getHandleGdrAsCvsStartFlag() const                       { return m_handleGdrAsCvsStartFlag;    }
};

/// slice header class
class Slice
{

private:
  //  Bitstream writing
  bool                       m_saoEnabledFlag[MAX_NUM_CHANNEL_TYPE];
#if NN_LF_UNIFIED
  NNFilterUnified::SliceParameters m_nnlfUnifiedParam;
  NnlfUnifiedInferGranularity      m_nnlfUnifiedInferGranularity;
#endif
#if JVET_W0066_CCSAO
  bool                       m_ccSaoEnabledFlag[MAX_NUM_COMPONENT];
#endif
  int                        m_iPOC;
  int                        m_iLastIDR;
  int                        m_prevGDRInSameLayerPOC;  //< the previous GDR in the same layer
  int                        m_iAssociatedIRAP;
  NalUnitType                m_iAssociatedIRAPType;
  int                        m_prevGDRSubpicPOC;
  int                        m_prevIRAPSubpicPOC;
  NalUnitType                m_prevIRAPSubpicType;
  bool                       m_enableDRAPSEI;
  bool                       m_useLTforDRAP;
  bool                       m_isDRAP;
  int                        m_latestDRAPPOC;
  const ReferencePictureList* m_pRPL0;                //< pointer to RPL for L0, either in the SPS or the local RPS in the same slice header
  const ReferencePictureList* m_pRPL1;                //< pointer to RPL for L1, either in the SPS or the local RPS in the same slice header
  ReferencePictureList        m_localRPL0;            //< RPL for L0 when present in slice header
  ReferencePictureList        m_localRPL1;            //< RPL for L1 when present in slice header
  int                         m_rpl0Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
  int                         m_rpl1Idx;              //< index of used RPL in the SPS or -1 for local RPL in the slice header
#if !JVET_S0052_RM_SEPARATE_COLOUR_PLANE
  int                        m_colourPlaneId;                         //!< 4:4:4 colour plane ID
#endif
  NalUnitType                m_eNalUnitType;         ///< Nal unit type for the slice
  bool                       m_pictureHeaderInSliceHeader;
  uint32_t                   m_nuhLayerId;           ///< Nal unit layer id
  SliceType                  m_eSliceType;
#if JVET_S0193_NO_OUTPUT_PRIOR_PIC
  bool                       m_noOutputOfPriorPicsFlag;           //!< no output of prior pictures flag
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool                       m_biPredictionIBCFlag;
#endif
  int                        m_iSliceQp;
  int                        m_iSliceQpBase;
  bool                       m_ChromaQpAdjEnabled;
  bool                       m_lmcsEnabledFlag;
  bool                       m_explicitScalingListUsed;
  bool                       m_deblockingFilterDisable;
  bool                       m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  int                        m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  int                        m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  int                        m_deblockingFilterCbBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCbTcOffsetDiv2;    //< tc offset for deblocking filter
  int                        m_deblockingFilterCrBetaOffsetDiv2;  //< beta offset for deblocking filter
  int                        m_deblockingFilterCrTcOffsetDiv2;    //< tc offset for deblocking filter
#if TCQ_8STATES
  int                        m_depQuantEnabledIdc;               //!< dependent quantization enabled idc
#else
  bool                       m_depQuantEnabledFlag;               //!< dependent quantization enabled flag
#endif
  bool                       m_signDataHidingEnabledFlag;         //!< sign data hiding enabled flag
  bool                       m_tsResidualCodingDisabledFlag;
  int                        m_list1IdxToList0Idx[MAX_NUM_REF];
  int                        m_aiNumRefIdx   [NUM_REF_PIC_LIST_01];    //  for multiple reference of current slice
#if JVET_Z0054_BLK_REF_PIC_REORDER
  std::vector<RefListAndRefIdx> m_refPicCombinedList;
  int8_t                     m_iRefIdxOfLC[2][MAX_NUM_REF];
  int8_t                     m_numNonScaledRefPic;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  std::vector<RefListAndRefIdx> m_refPicCombinedListAmvpMerge;
  int8_t                     m_iRefIdxOfLCAmvpMerge[2][MAX_NUM_REF];
#endif
  std::vector<RefPicPair>    m_refPicPairList;
  int8_t                     m_iRefPicPairIdx[MAX_NUM_REF][MAX_NUM_REF];
  int8_t                     m_numNonScaledRefPicPair;
#endif
  bool                       m_pendingRasInit;

  bool                       m_bCheckLDC;
#if JVET_AF0128_LIC_MERGE_TM
  bool                       m_bCheckLDB;
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  bool                       m_bSepObmc4GPM;
#endif

  bool                       m_biDirPred;
  int                        m_symRefIdx[2];
#if JVET_Y0128_NON_CTC
  bool                       m_useBM;
  int                        m_bmDefaultRefIdx[2];
  bool                       m_useAmvpMergeMode;
  int                        m_amvpMergeModeOnlyOneValidRefIdx[2];
  bool                       m_amvpMergeModeValidRefIdx[2][MAX_NUM_REF];
  bool                       m_amvpMergeModeValidCandPair[MAX_NUM_REF][MAX_NUM_REF];
#endif

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  bool                       m_pairEqualPocDist[MAX_NUM_REF][MAX_NUM_REF];
#endif
  //  Data
  int                        m_iSliceQpDelta;
  int                        m_iSliceChromaQpDelta[MAX_NUM_COMPONENT+1];
  Picture*                   m_apcRefPicList [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_aiRefPOCList  [NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
#if JVET_AH0069_CMVP
  int                        m_aiRefRefIdxList[NUM_REF_PIC_LIST_01][MAX_NUM_REF][NUM_REF_PIC_LIST_01][MAX_NUM_REF+1][NUM_REF_PIC_LIST_01];
#endif
  bool                       m_bIsUsedAsLongTerm[NUM_REF_PIC_LIST_01][MAX_NUM_REF+1];
  int                        m_iDepth;
  Picture*                   m_scaledRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  Picture*                   m_savedRefPicList[NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  std::pair<int, int>        m_scalingRatio[NUM_REF_PIC_LIST_01][MAX_NUM_REF_PICS];


  // access channel
  const VPS*                 m_pcVPS;
  const SPS*                 m_pcSPS;
  const PPS*                 m_pcPPS;
  Picture*                   m_pcPic;
  const PicHeader*           m_pcPicHeader;    //!< pointer to picture header structure
  bool                       m_colFromL0Flag;  // collocated picture from List0 flag


  uint32_t                   m_colRefIdx;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  bool                       m_colFromL0Flag2nd;  // collocated picture from List0 flag
  uint32_t                   m_colRefIdx2nd;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  uint8_t                    m_amvpSbTmvpNumOffset;
  uint8_t                    m_amvpSbTmvpNumColPic;
  bool                       m_amvpSbTmvpAmvrEnabledFlag;
  bool                       m_amvpSbTmvpEnabledFlag;
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  int                        m_extAmvpLevel;
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  uint32_t                   m_costForARMC;
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  std::vector<int>           m_implicitRefIdx[2][NUM_REF_PIC_LIST_01][NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
#if JVET_AI0183_MVP_EXTENSION
  std::vector<int>           m_implicitRefIdx2nd[2][NUM_REF_PIC_LIST_01][NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
  std::vector<int>           m_implicitRefIdx3rd[2][NUM_REF_PIC_LIST_01][NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
#endif
#else
  std::vector<int>           m_implicitRefIdx[NUM_REF_PIC_LIST_01][NUM_REF_PIC_LIST_01][MAX_NUM_REF + 1];
#endif
#endif
  double                     m_lambdas[MAX_NUM_COMPONENT];
#if INTER_LIC
  bool                       m_UseLIC;
#endif 
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool                       m_useIBC;
#endif

  bool                       m_abEqualRef  [NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_REF];
  uint32_t                   m_uiTLayer;
  bool                       m_bTLayerSwitchingFlag;

  SliceMap                   m_sliceMap;                     //!< list of CTUs in current slice - raster scan CTU addresses
  uint32_t                   m_independentSliceIdx;
  bool                       m_nextSlice;
  uint32_t                   m_sliceBits;
  bool                       m_bFinalized;


  bool                       m_bTestWeightPred;
  bool                       m_bTestWeightBiPred;
  WPScalingParam             m_weightPredTable[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  WPACDCParam                m_weightACDCParam[MAX_NUM_COMPONENT];
  ClpRngs                    m_clpRngs;
  std::vector<uint32_t>      m_substreamSizes;
  uint32_t                   m_numEntryPoints;
  uint32_t                   m_numSubstream;

  bool                       m_cabacInitFlag;
#if JVET_AG0196_CABAC_RETRAIN
  SliceType                  m_cabacInitSliceType;
#endif

  uint32_t                   m_sliceSubPicId;


  SliceType                  m_encCABACTableIdx;           // Used to transmit table selection across slices.

  clock_t                    m_iProcessingStartTime;
  double                     m_dProcessingTime;

  int                        m_rpPicOrderCntVal;
#if ALF_IMPROVEMENT                   
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  bool                       m_tileGroupAlfReuseFlag[MAX_NUM_COMPONENT];
  bool                       m_tileGroupCcalfReuseFlag[MAX_NUM_COMPONENT];
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  int                        m_tileGroupAlfFixedFilterSetIdx[MAX_NUM_COMPONENT];
#else
  int                        m_tileGroupAlfFixedFilterSetIdx;
#endif
#endif
#if JVET_AK0065_TALF
  APS*                       m_talfApss[ALF_CTB_MAX_NUM_APS];
#endif
  APS*                       m_alfApss[ALF_CTB_MAX_NUM_APS];
  bool                       m_tileGroupAlfEnabledFlag[MAX_NUM_COMPONENT];
  int                        m_tileGroupNumAps;
  std::vector<int>           m_tileGroupLumaApsId;
  int                        m_tileGroupChromaApsId;
  bool                       m_tileGroupCcAlfCbEnabledFlag;
  bool                       m_tileGroupCcAlfCrEnabledFlag;
  int                        m_tileGroupCcAlfCbApsId;
  int                        m_tileGroupCcAlfCrApsId;
  bool                       m_disableSATDForRd;
  bool                       m_isLossless;
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  bool                       m_useScaleAlf; // at least one APS uses alf-residuals-rescaling
  ScaleAlf                   m_scaleAlfParam[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
  int                        m_idxCorrChroma[3];
#endif

#if JVET_AK0065_TALF
  TAlfControl   m_tileGroupTAlfControl; //slice controls
#endif
#if JVET_AG0145_ADAPTIVE_CLIPPING
  int                        m_lumaPelMax;
  int                        m_lumaPelMin;
  bool                       m_adaptiveClipQuant;
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  bool                       m_offsetRefinementDbf;
  bool                       m_offsetRefinementAlf;
  uint8_t                    m_offsetRefinementDbfIdx;
  uint8_t                    m_offsetRefinementAlfIdx;
#endif
#if MULTI_HYP_PRED
  int                        m_numMultiHypRefPics = 0;

#if !JVET_Z0054_BLK_REF_PIC_REORDER
  struct RefListAndRefIdx
  {
    RefPicList refList;
    int8_t     refIdx;
  };
#endif
  std::vector<RefListAndRefIdx> m_multiHypRefPics;
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool                       m_separateTreeEnabled;
  bool                       m_processingIntraRegion;
  bool                       m_processingSeparateTrees;
  ChannelType                m_processingChannelType;
  int                        m_intraRegionRootDepth;
  int                        m_intraRegionRootQtDepth;
  int                        m_intraRegionRootBtDepth;
  int                        m_intraRegionRootMtDepth;
  int                        m_intraRegionRootImplicitBtDepth;
  bool                       m_intraRegionNoSplitTest;
#endif

#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool m_pnnMode;
#endif
public:
                              Slice();
  virtual                     ~Slice();
  void                        initSlice();
  void                        inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps );
  void                        setPicHeader( const PicHeader* pcPicHeader )           { m_pcPicHeader = pcPicHeader;                                  }
  const PicHeader*            getPicHeader() const                                   { return m_pcPicHeader;                                         }
  int                         getRefIdx4MVPair( RefPicList eCurRefPicList, int nCurRefIdx );


  void                        setSPS( const SPS* pcSPS )                             { m_pcSPS = pcSPS;                                              }
  const SPS*                  getSPS() const                                         { return m_pcSPS;                                               }
  void                        setVPS( const VPS* pcVPS )                             { m_pcVPS = pcVPS;                                              }
  const VPS*                  getVPS() const                                         { return m_pcVPS;                                               }

  void                        setPPS( const PPS* pcPPS )                             { m_pcPPS = pcPPS;                                              }
  const PPS*                  getPPS() const                                         { return m_pcPPS;                                               }

  void                        setAlfAPSs(APS** apss)                                 { memcpy(m_alfApss, apss, sizeof(m_alfApss));                   }
  APS**                       getAlfAPSs()                                           { return m_alfApss;                                             }
#if JVET_AK0065_TALF
  void                        setTAlfAPSs(APS **apss)                                { memcpy(m_talfApss, apss, sizeof(m_talfApss)); }
  APS**                       getTAlfAPSs()                                          { return m_talfApss; }
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  bool                        getUseAlfScale()                                       { return m_useScaleAlf; }
  void                        setUseAlfScale( bool s )                               { m_useScaleAlf = s; }
  void                        resetAlfScale()
  {
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      for (int j = 0; j < MAX_NUM_ALF_ALTERNATIVES_LUMA; j++)
      {
        m_scaleAlfParam[i][j].reset();
      }
    }
  }
  ScaleAlf*                   getAlfScalePtr( const int apsId, const int altNum )   { return &m_scaleAlfParam[m_tileGroupLumaApsId[apsId]][altNum]; }
  ScaleAlf&                   getAlfScale( const int apsId, const int altNum )      { return m_scaleAlfParam[m_tileGroupLumaApsId[apsId]][altNum];  }
  void  copyAlfScale( Slice& slice )
  {
    m_useScaleAlf = slice.m_useScaleAlf;
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      for (int j = 0; j < MAX_NUM_ALF_ALTERNATIVES_LUMA; j++)
      {
        ScaleAlf& scaleAlfParam = slice.m_scaleAlfParam[i][j];
        if ( scaleAlfParam.initDone ) 
        {
          scaleAlfParam.setMinMax( slice.getLumaPelMin(), slice.getLumaPelMax() );
          scaleAlfParam.setGroupSize( scaleAlfParam.groupShift );
          scaleAlfParam.fillIdxCorr();

          m_scaleAlfParam[i][j] = scaleAlfParam;
        }
      }
    }
    for (int c = 0; c < MAX_NUM_COMPONENT; c++)
    {
      m_idxCorrChroma[c] = slice.m_idxCorrChroma[c];
    }
  }
  void                        setAlfScaleChroma( int compIdx, const int s )          { m_idxCorrChroma[compIdx] = s;    }
  int                         getAlfScaleChroma( int compIdx ) const                 { return m_idxCorrChroma[compIdx]; }
#endif

  void                        setSaoEnabledFlag(ChannelType chType, bool s)          {m_saoEnabledFlag[chType] =s;                                   }
  bool                        getSaoEnabledFlag(ChannelType chType) const            { return m_saoEnabledFlag[chType];                              }
#if NN_LF_UNIFIED
  void                        setNnlfUnifiedParameters(const NNFilterUnified::SliceParameters &prm) { m_nnlfUnifiedParam = prm; }
  const NNFilterUnified::SliceParameters getNnlfUnifiedParameters() const                           { return m_nnlfUnifiedParam; }
  void                        setNnlfUnifiedInferGranularity(NnlfUnifiedInferGranularity nnlfUnifiedInferGranularity){ m_nnlfUnifiedInferGranularity = nnlfUnifiedInferGranularity; }
  NnlfUnifiedInferGranularity getNnlfUnifiedInferGranularity() const                                { return m_nnlfUnifiedInferGranularity; }
#endif

#if JVET_W0066_CCSAO
  void                        resetCcSaoEnabledFlag()                                { memset(m_ccSaoEnabledFlag, 0, sizeof(m_ccSaoEnabledFlag));    }
  void                        setCcSaoEnabledFlag(ComponentID compID, bool b)        { m_ccSaoEnabledFlag[compID] = b;                               }
  bool                        getCcSaoEnabledFlag(ComponentID compID)                { return m_ccSaoEnabledFlag[compID];                            }
#endif
  void                        setRPL0(const ReferencePictureList *pcRPL)             { m_pRPL0 = pcRPL;                                             }
  void                        setRPL1(const ReferencePictureList *pcRPL)             { m_pRPL1 = pcRPL;                                             }
  const ReferencePictureList* getRPL0()                                              { return m_pRPL0;                                              }
  const ReferencePictureList* getRPL1()                                              { return m_pRPL1;                                              }
  ReferencePictureList*       getLocalRPL0()                                         { return &m_localRPL0;                                         }
  ReferencePictureList*       getLocalRPL1()                                         { return &m_localRPL1;                                         }
  void                        setRPL0idx(int rplIdx)                                 { m_rpl0Idx = rplIdx;                                          }
  void                        setRPL1idx(int rplIdx)                                 { m_rpl1Idx = rplIdx;                                          }
  int                         getRPL0idx() const                                     { return m_rpl0Idx;                                            }
  int                         getRPL1idx() const                                     { return m_rpl1Idx;                                            }
  void                        setLastIDR(int iIDRPOC)                                { m_iLastIDR = iIDRPOC;                                         }
  int                         getLastIDR() const                                     { return m_iLastIDR;                                            }
  void                        setPrevGDRInSameLayerPOC(int prevGDRInSameLayerPOC)    { m_prevGDRInSameLayerPOC = prevGDRInSameLayerPOC;              }
  int                         getPrevGDRInSameLayerPOC() const                       { return m_prevGDRInSameLayerPOC;                               }
  void                        setAssociatedIRAPPOC(int iAssociatedIRAPPOC)           { m_iAssociatedIRAP = iAssociatedIRAPPOC;                       }
  int                         getAssociatedIRAPPOC() const                           { return m_iAssociatedIRAP;                                     }
  void                        setAssociatedIRAPType(NalUnitType associatedIRAPType)  { m_iAssociatedIRAPType = associatedIRAPType;                   }
  NalUnitType                 getAssociatedIRAPType() const                          { return m_iAssociatedIRAPType;                                 }
  void                        setPrevGDRSubpicPOC(int poc)                           { m_prevGDRSubpicPOC = poc;                                     }
  int                         getPrevGDRSubpicPOC() const                            { return m_prevGDRSubpicPOC;                                    }
  void                        setPrevIRAPSubpicPOC(int poc)                          { m_prevIRAPSubpicPOC = poc;                                    }
  int                         getPrevIRAPSubpicPOC() const                           { return m_prevIRAPSubpicPOC;                                   }
  void                        setPrevIRAPSubpicType(NalUnitType type)                { m_prevIRAPSubpicType = type;                                  }
  NalUnitType                 getPrevIRAPSubpicType() const                          { return m_prevIRAPSubpicType;                                  }
  void                        checkSubpicTypeConstraints(PicList& rcListPic, const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int prevIRAPSubpicDecOrderNo);
  SliceType                   getSliceType() const                                   { return m_eSliceType;                                          }
#if JVET_S0193_NO_OUTPUT_PRIOR_PIC
  void                        setNoOutputOfPriorPicsFlag(bool b)                     { m_noOutputOfPriorPicsFlag = b;                                }
  bool                        getNoOutputOfPriorPicsFlag() const                     { return m_noOutputOfPriorPicsFlag;                             }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  void                        setBiPredictionIBCFlag(bool b)                         { m_biPredictionIBCFlag = b;                                }
  bool                        getBiPredictionIBCFlag() const                         { return m_biPredictionIBCFlag;                             }
#endif
  int                         getPOC() const                                         { return m_iPOC;                                                }
  int                         getSliceQp() const                                     { return m_iSliceQp;                                            }
  bool                        getUseWeightedPrediction() const                       { return( (m_eSliceType==P_SLICE && testWeightPred()) || (m_eSliceType==B_SLICE && testWeightBiPred()) ); }
  int                         getSliceQpDelta() const                                { return m_iSliceQpDelta;                                       }
  int                         getSliceChromaQpDelta(ComponentID compID) const        { return isLuma(compID) ? 0 : m_iSliceChromaQpDelta[compID];    }
  bool                        getUseChromaQpAdj() const                              { return m_ChromaQpAdjEnabled;                                  }
  bool                        getDeblockingFilterDisable() const                     { return m_deblockingFilterDisable;                             }
  bool                        getDeblockingFilterOverrideFlag() const                { return m_deblockingFilterOverrideFlag;                        }
  int                         getDeblockingFilterBetaOffsetDiv2()const               { return m_deblockingFilterBetaOffsetDiv2;                      }
  int                         getDeblockingFilterTcOffsetDiv2() const                { return m_deblockingFilterTcOffsetDiv2;                        }
  int                         getDeblockingFilterCbBetaOffsetDiv2()const             { return m_deblockingFilterCbBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCbTcOffsetDiv2() const              { return m_deblockingFilterCbTcOffsetDiv2;                      }
  int                         getDeblockingFilterCrBetaOffsetDiv2()const             { return m_deblockingFilterCrBetaOffsetDiv2;                    }
  int                         getDeblockingFilterCrTcOffsetDiv2() const              { return m_deblockingFilterCrTcOffsetDiv2;                      }
  bool                        getPendingRasInit() const                              { return m_pendingRasInit;                                      }
  void                        setPendingRasInit( bool val )                          { m_pendingRasInit = val;                                       }

  void                        setLmcsEnabledFlag(bool b)                              { m_lmcsEnabledFlag = b;                                       }
  bool                        getLmcsEnabledFlag()                                    { return m_lmcsEnabledFlag;                                    }
  const bool                  getLmcsEnabledFlag() const                              { return m_lmcsEnabledFlag;                                    }

  void                        setExplicitScalingListUsed(bool b)                      { m_explicitScalingListUsed = b;                               }
  bool                        getExplicitScalingListUsed()                            { return m_explicitScalingListUsed;                            }

  int                         getNumRefIdx( RefPicList e ) const                     { return m_aiNumRefIdx[e];                                      }
  Picture*                    getPic()                                               { return m_pcPic;                                               }
  const Picture*              getPic() const                                         { return m_pcPic;                                               }
        Picture*              getRefPic( RefPicList e, int iRefIdx) const            { CHECK( iRefIdx < 0 || iRefIdx >= MAX_NUM_REF, "refIdx is out of range" ); return m_apcRefPicList[e][iRefIdx]; }
#if JVET_Z0118_GDR
        Picture*              getReferencePicture(RefPicList e, int iRefIdx)         { return m_apcRefPicList[e][iRefIdx];                           }
#endif
  int                         getRefPOC( RefPicList e, int iRefIdx) const            { return m_aiRefPOCList[e][iRefIdx];                            }
#if JVET_AH0069_CMVP
  const int*                  getRefRefIdx( RefPicList e, int iRefIdx, RefPicList refe, int iRefRefIdx ) const { return m_aiRefRefIdxList[e][iRefIdx][refe][iRefRefIdx]; }
#endif
  int                         getDepth() const                                       { return m_iDepth;                                              }
  bool                        getColFromL0Flag() const                               { return m_colFromL0Flag;                                       }
  uint32_t                    getColRefIdx() const                                   { return m_colRefIdx;                                           }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  bool                        getColFromL0Flag2nd() const                            { return m_colFromL0Flag2nd;                                    }
  uint32_t                    getColRefIdx2nd() const                                { return m_colRefIdx2nd;                                        }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  uint8_t                     getAmvpSbTmvpNumOffset() const                         { return m_amvpSbTmvpNumOffset;                                     }
  uint8_t                     getAmvpSbTmvpNumColPic() const                         { return m_amvpSbTmvpNumColPic;                                     }
  bool                        getAmvpSbTmvpAmvrEnabledFlag() const                   { return m_amvpSbTmvpAmvrEnabledFlag;                               }
  bool                        getAmvpSbTmvpEnabledFlag() const                       { return m_amvpSbTmvpEnabledFlag;                                   }
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  int                         getExtAmvpLevel() const                                 { return m_extAmvpLevel;                                   }
#endif
  void                        checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic);
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AJ0237_INTERNAL_12BIT
  uint32_t                    getCostForARMC(int bitDepth) const                                 { return m_costForARMC >> (std::max(0, bitDepth - 10)); } // for header parsing/writing purpose
#endif
  uint32_t                    getCostForARMC() const                                 { return m_costForARMC;                                         }
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void resizeImBuf(int numSlices, int col)
  {
    for (int refIdx = 0; refIdx < MAX_NUM_REF + 1; refIdx++)
    {
      m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].begin(), m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].begin(), m_implicitRefIdx[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].end(), -1);

      m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].begin(), m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].begin(), m_implicitRefIdx[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].end(), -1);
#if JVET_AI0183_MVP_EXTENSION
      m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].begin(),
                m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].begin(),
                m_implicitRefIdx2nd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].end(), -1);

      m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].begin(),
                m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].begin(),
                m_implicitRefIdx2nd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].end(), -1);
      m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].begin(),
                m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].begin(),
                m_implicitRefIdx3rd[col][REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].end(), -1);

      m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].begin(),
                m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].begin(),
                m_implicitRefIdx3rd[col][REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].end(), -1);
#endif
    }
  }
  void                        setImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int curRefIdx, int col) { m_implicitRefIdx[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx] = curRefIdx; }
  int                         getImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) { return m_implicitRefIdx[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
  int                         getImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) const { return m_implicitRefIdx[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
#if JVET_AI0183_MVP_EXTENSION
  void                        setImRefIdx2nd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int curRefIdx, int col) { m_implicitRefIdx2nd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx] = curRefIdx; }
  int                         getImRefIdx2nd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) { return m_implicitRefIdx2nd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
  int                         getImRefIdx2nd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) const { return m_implicitRefIdx2nd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
  void                        setImRefIdx3rd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int curRefIdx, int col) { m_implicitRefIdx3rd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx] = curRefIdx; }
  int                         getImRefIdx3rd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) { return m_implicitRefIdx3rd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
  int                         getImRefIdx3rd(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int col = 0) const { return m_implicitRefIdx3rd[col][colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
#endif

#else
  void resizeImBuf(int numSlices)
  {
    for (int refIdx = 0; refIdx < MAX_NUM_REF + 1; refIdx++)
    {
      m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].begin(), m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].begin(), m_implicitRefIdx[REF_PIC_LIST_0][REF_PIC_LIST_1][refIdx].end(), -1);

      m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].begin(), m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_0][refIdx].end(), -1);

      m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].resize(numSlices);
      std::fill(m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].begin(), m_implicitRefIdx[REF_PIC_LIST_1][REF_PIC_LIST_1][refIdx].end(), -1);
    }
  }
  void                        setImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx, int curRefIdx) { m_implicitRefIdx[colRefPicList][curRefPicList][colRefIdx][sliceIdx] = curRefIdx; }
  int                         getImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx) { return m_implicitRefIdx[colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
  int                         getImRefIdx(int sliceIdx, RefPicList colRefPicList, RefPicList curRefPicList, int colRefIdx) const { return m_implicitRefIdx[colRefPicList][curRefPicList][colRefIdx][sliceIdx]; }
#endif
#endif
  bool                        getIsUsedAsLongTerm(int i, int j) const                { return m_bIsUsedAsLongTerm[i][j];                             }
  void                        setIsUsedAsLongTerm(int i, int j, bool value)          { m_bIsUsedAsLongTerm[i][j] = value;                            }
  bool                        getCheckLDC() const                                    { return m_bCheckLDC;                                           }
#if JVET_AF0128_LIC_MERGE_TM
  bool                        getCheckLDB() const                                    { return m_bCheckLDB;                                           }
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  bool                        getCheckUseSepOBMC() const                             { return m_bSepObmc4GPM;                                        }
#endif
  int                         getList1IdxToList0Idx( int list1Idx ) const            { return m_list1IdxToList0Idx[list1Idx];                        }
  void                        setPOC( int i )                                        { m_iPOC              = i;                                      }
  bool                        getPictureHeaderInSliceHeader() const                  { return m_pictureHeaderInSliceHeader;                         }
  void                        setPictureHeaderInSliceHeader( bool e )                { m_pictureHeaderInSliceHeader = e;                            }
  void                        setNalUnitType( NalUnitType e )                        { m_eNalUnitType      = e;                                      }
  NalUnitType                 getNalUnitType() const                                 { return m_eNalUnitType;                                        }
  void                        setNalUnitLayerId( uint32_t i )                        { m_nuhLayerId = i;                                             }
  uint32_t                    getNalUnitLayerId() const                              { return m_nuhLayerId;                                          }
  bool                        getRapPicFlag() const;
  bool                        getIdrPicFlag() const                                  { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  bool                        isIRAP() const { return (getNalUnitType() >= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA); }
  // CLVSS PU is either an IRAP PU with NoOutputBeforeRecoveryFlag equal to 1 or a GDR PU with NoOutputBeforeRecoveryFlag equal to 1.
  bool                        isClvssPu() const                                      { return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR && !m_pcPPS->getMixedNaluTypesInPicFlag() && m_pcPicHeader->getNoOutputBeforeRecoveryFlag(); }
  bool                        isIDRorBLA() const { return (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  void                        checkCRA(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int pocCRA, PicList& rcListPic);
  void                        checkSTSA(PicList& rcListPic);
  void                        checkRPL(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int associatedIRAPDecodingOrderNumber, PicList& rcListPic);
  void                        decodingRefreshMarking(int& pocCRA, bool& bRefreshPending, PicList& rcListPic, const bool bEfficientFieldIRAPEnabled);
#if JVET_AG0196_CABAC_RETRAIN
  void                        setSliceType( SliceType e )                            { m_eSliceType        = e; m_cabacInitSliceType = m_eSliceType; }
#else
  void                        setSliceType( SliceType e )                            { m_eSliceType        = e;                                      }
#endif
  void                        setSliceQp( int i )                                    { m_iSliceQp          = i;                                      }
  void                        setSliceQpDelta( int i )                               { m_iSliceQpDelta     = i;                                      }
  void                        setSliceChromaQpDelta( ComponentID compID, int i )     { m_iSliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i;       }
  void                        setUseChromaQpAdj( bool b )                            { m_ChromaQpAdjEnabled = b;                                     }
  void                        setDeblockingFilterDisable( bool b )                   { m_deblockingFilterDisable= b;                                 }
  void                        setDeblockingFilterOverrideFlag( bool b )              { m_deblockingFilterOverrideFlag = b;                           }
  void                        setDeblockingFilterBetaOffsetDiv2( int i )             { m_deblockingFilterBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterTcOffsetDiv2( int i )               { m_deblockingFilterTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCbBetaOffsetDiv2( int i )           { m_deblockingFilterCbBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCbTcOffsetDiv2( int i )             { m_deblockingFilterCbTcOffsetDiv2 = i;                           }
  void                        setDeblockingFilterCrBetaOffsetDiv2( int i )           { m_deblockingFilterCrBetaOffsetDiv2 = i;                         }
  void                        setDeblockingFilterCrTcOffsetDiv2( int i )             { m_deblockingFilterCrTcOffsetDiv2 = i;                           }
#if TCQ_8STATES
  void                        setDepQuantEnabledIdc( int b )                        { m_depQuantEnabledIdc = b;                                                                   }
  int                         getDepQuantEnabledIdc() const                         { return m_depQuantEnabledIdc;                                                                }  
#else
  void                        setDepQuantEnabledFlag( bool b )                       { m_depQuantEnabledFlag = b;                                                                   }
  bool                        getDepQuantEnabledFlag() const                         { return m_depQuantEnabledFlag;                                                                }  
#endif
  void                        setSignDataHidingEnabledFlag( bool b )                 { m_signDataHidingEnabledFlag = b;                                                             }
  bool                        getSignDataHidingEnabledFlag() const                   { return m_signDataHidingEnabledFlag;                                                          }  
  void                        setTSResidualCodingDisabledFlag(bool b) { m_tsResidualCodingDisabledFlag = b; }
  bool                        getTSResidualCodingDisabledFlag() const { return m_tsResidualCodingDisabledFlag; }

  void                        setNumRefIdx( RefPicList e, int i )                    { m_aiNumRefIdx[e]    = i;                                      }
  void                        setPic( Picture* p )                                   { m_pcPic             = p;                                      }
  void                        setDepth( int iDepth )                                 { m_iDepth            = iDepth;                                 }

  void                        constructRefPicList(PicList& rcListPic);
  void                        setRefPOCList();
#if JVET_AH0069_CMVP
  void                        setRefRefIdxList();
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  void                        generateCombinedList();
  int8_t                      getRefIdxOfLC(RefPicList e, int i) { return m_iRefIdxOfLC[e][i]; }
  int8_t                      getNumNonScaledRefPic() { return m_numNonScaledRefPic; }
  const std::vector<RefListAndRefIdx> &getRefPicCombinedList() const { return m_refPicCombinedList; }
  int8_t                      getRefIdxOfLCAmvpMerge(RefPicList e, int i) { return m_iRefIdxOfLCAmvpMerge[e][i]; }
  const std::vector<RefListAndRefIdx> &getRefPicCombinedListAmvpMerge() const { return m_refPicCombinedListAmvpMerge; }
  void                        generateRefPicPairList();
  int8_t                      getRefPicPairIdx(int i, int j) { return m_iRefPicPairIdx[i][j]; }
  int8_t                      getNumNonScaledRefPicPair() { return m_numNonScaledRefPicPair; }
  const std::vector<RefPicPair> &getRefPicPairList() const { return m_refPicPairList; }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  void                        generateEqualPocDist();
  bool                        getPairEqualPocDist(int refIdx0, int refIdx1) const { return m_pairEqualPocDist[refIdx0][refIdx1]; };
#endif
#if JVET_AI0183_MVP_EXTENSION
  void                        generateIntersectingMv();
  void                        getPuIntersectingMv(Position topLeftPuPos, Size puSize, IntersectingMvData* resultIntersectingMvDataPtr);
  bool                        getIntersectingPositionMv(Position& intersectingScaledPosTL, Mv& srcIntersectingMv, Mv& dstIntersectingMv, Position srcScaledPosTL,
                                                        Mv srcRefMv, Picture* srcRefPic, Picture* dstRefPic, const Position boundaryPosMin, const Position boundaryPosMax, const int scaleFactorToCurrent);
  bool                        checkValidIntersectingMv(IntersectingMvData* curIntersectingMvDataBufPtr, RefPicList srcRefListIdx, int8_t srcRefIdx, Mv srcMv, RefPicList dstRefListIdx, int8_t dstRefIdx, Mv dstMv);
#endif
#if MULTI_HYP_PRED
  void                        setMultiHypRefPicList();
  const std::vector<RefListAndRefIdx> &getMultiHypRefPicList() const { return m_multiHypRefPics; }
  void                        setNumMultiHypRefPics(int i) { m_numMultiHypRefPics = i; }
  int                         getNumMultiHypRefPics() const { return m_numMultiHypRefPics; }
#endif

  void                        setColFromL0Flag( bool colFromL0 )                     { m_colFromL0Flag = colFromL0;                                  }
  void                        setColRefIdx( uint32_t refIdx)                             { m_colRefIdx = refIdx;                                         }
  void                        setCheckLDC( bool b )                                  { m_bCheckLDC = b;                                              }
#if JVET_AF0128_LIC_MERGE_TM
  void                        setCheckLDB( bool b )                                  { m_bCheckLDB = b;                                              }
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
  void                        setCheckUseSepOBMC( bool b )                           { m_bSepObmc4GPM = b;                                           }
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  void                        setColFromL0Flag2nd(bool colFromL0)                    { m_colFromL0Flag2nd = colFromL0;                               }
  void                        setColRefIdx2nd(uint32_t refIdx)                       { m_colRefIdx2nd = refIdx;                                      }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  void                        setAmvpSbTmvpEnabledFlag(bool b)                       { m_amvpSbTmvpEnabledFlag = b;                                      }
  void                        setAmvpSbTmvpNumOffset(uint8_t numOffset)              { m_amvpSbTmvpNumOffset = numOffset;                                }
  void                        setAmvpSbTmvpNumColPic(uint8_t numPic)                 { m_amvpSbTmvpNumColPic = numPic;                                   }
  void                        setAmvpSbTmvpAmvrEnabledFlag(bool b)                   { m_amvpSbTmvpAmvrEnabledFlag = b;                                  }
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  void                        setExtAmvpLevel(int b)                            { m_extAmvpLevel = b;                                    }
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AJ0237_INTERNAL_12BIT
  void                        setCostForARMC(uint32_t cost, int bitDepth)                          { m_costForARMC = (cost << (std::max(0, bitDepth - 10))); }
#else
  void                        setCostForARMC(uint32_t cost)                          { m_costForARMC = cost;                                         }
#endif
#endif
  void                        setBiDirPred( bool b, int refIdx0, int refIdx1 ) { m_biDirPred = b; m_symRefIdx[0] = refIdx0; m_symRefIdx[1] = refIdx1; }
  bool                        getBiDirPred() const { return m_biDirPred; }
  int                         getSymRefIdx( int refList ) const { return m_symRefIdx[refList]; }
#if JVET_Y0128_NON_CTC
  void                        checkBMAvailability(Slice* pcSlice);
  void                        setUseBM( bool b, int refIdx0, int refIdx1 ) { m_useBM = b; m_bmDefaultRefIdx[0] = refIdx0; m_bmDefaultRefIdx[1] = refIdx1; }
  bool                        getUseBM() const { return m_useBM; }
  int                         getBMDefaultRefIdx( int refList ) const { return m_bmDefaultRefIdx[refList]; }
  void                        checkAmvpMergeModeAvailability(Slice* pcSlice);
  bool                        getUseAmvpMergeMode() const { return m_useAmvpMergeMode; }
  int                         getAmvpMergeModeOnlyOneValidRefIdx(RefPicList refList) const { return m_amvpMergeModeOnlyOneValidRefIdx[refList]; }
  bool                        getAmvpMergeModeValidRefIdx(RefPicList refList, int refIdx) { return m_amvpMergeModeValidRefIdx[refList][refIdx]; }
  bool                        getAmvpMergeModeValidCandPair(int refIdxInRefList0, int refIdxInRefList1) const { return m_amvpMergeModeValidCandPair[refIdxInRefList0][refIdxInRefList1]; }
#endif

  bool                        isIntra() const                                        { return m_eSliceType == I_SLICE;                               }
  bool                        isInterB() const                                       { return m_eSliceType == B_SLICE;                               }
  bool                        isInterP() const                                       { return m_eSliceType == P_SLICE;                               }
#if JVET_Z0118_GDR
  bool                        isInterGDR() const                                     { return (m_eSliceType == B_SLICE && m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR); }  
#endif

  bool                        getEnableDRAPSEI () const                              { return m_enableDRAPSEI;                                       }
  void                        setEnableDRAPSEI ( bool b )                            { m_enableDRAPSEI = b;                                          }
  bool                        getUseLTforDRAP () const                               { return m_useLTforDRAP;                                        }
  void                        setUseLTforDRAP ( bool b )                             { m_useLTforDRAP = b;                                           }
  bool                        isDRAP () const                                        { return m_isDRAP;                                              }
  void                        setDRAP ( bool b )                                     { m_isDRAP = b;                                                 }
  void                        setLatestDRAPPOC ( int i )                             { m_latestDRAPPOC = i;                                          }
  int                         getLatestDRAPPOC () const                              { return m_latestDRAPPOC;                                       }
  bool                        cvsHasPreviousDRAP() const                             { return m_latestDRAPPOC != MAX_INT;                            }
  bool                        isPocRestrictedByDRAP( int poc, bool precedingDRAPinDecodingOrder );
  bool                        isPOCInRefPicList( const ReferencePictureList *rpl, int poc );
  void                        checkConformanceForDRAP( uint32_t temporalId );
#if !JVET_S0052_RM_SEPARATE_COLOUR_PLANE
  void                        setColourPlaneId( int id )                             { m_colourPlaneId = id;                                         }
  int                         getColourPlaneId() const                               { return m_colourPlaneId;                                       }
#endif

  void                        setLambdas( const double lambdas[MAX_NUM_COMPONENT] )  { for (int component = 0; component < MAX_NUM_COMPONENT; component++) m_lambdas[component] = lambdas[component]; }
  const double*               getLambdas() const                                     { return m_lambdas;                                             }

  void                        setSliceSubPicId(int i)                               { m_sliceSubPicId = i;   }
  uint32_t                    getSliceSubPicId() const                              { return m_sliceSubPicId; }
  uint32_t                    getCuQpDeltaSubdiv() const                             { return this->isIntra() ? m_pcPicHeader->getCuQpDeltaSubdivIntra() : m_pcPicHeader->getCuQpDeltaSubdivInter(); }
  uint32_t                    getCuChromaQpOffsetSubdiv() const                      { return this->isIntra() ? m_pcPicHeader->getCuChromaQpOffsetSubdivIntra() : m_pcPicHeader->getCuChromaQpOffsetSubdivInter(); }
  void                        initEqualRef();
  bool                        isEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2 )
  {
    CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid reference picture list");
    if (iRefIdx1 < 0 || iRefIdx2 < 0)
    {
      return false;
    }
    else
    {
      return m_abEqualRef[e][iRefIdx1][iRefIdx2];
    }
  }

  void                        setEqualRef( RefPicList e, int iRefIdx1, int iRefIdx2, bool b)
  {
    CHECK( e >= NUM_REF_PIC_LIST_01, "Invalid reference picture list" );
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }

  static void                 sortPicList( PicList& rcListPic );
  void                        setList1IdxToList0Idx();

  uint32_t                    getTLayer() const                                      { return m_uiTLayer;                                            }
  void                        setTLayer( uint32_t uiTLayer )                             { m_uiTLayer = uiTLayer;                                        }

  void                        checkLeadingPictureRestrictions( PicList& rcListPic, const PPS& pps)                                         const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList* pRPL, int rplIdx, bool printErrors, int* refPicIndex, int numActiveRefPics) const;

  void                        applyReferencePictureListBasedMarking( PicList& rcListPic, const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int layerId, const PPS& pps )  const;
  bool                        isTemporalLayerSwitchingPoint( PicList& rcListPic )                                           const;
  bool                        isStepwiseTemporalLayerSwitchingPointCandidate( PicList& rcListPic )                          const;
  int                         checkThatAllRefPicsAreAvailable(PicList& rcListPic, const ReferencePictureList *pRPL, int rplIdx, bool printErrors)                const;

  void                        setNumTilesInSlice( uint32_t u )                       { m_sliceMap.setNumTilesInSlice( u );                                       }
  uint32_t                    getNumTilesInSlice() const                             { return m_sliceMap.getNumTilesInSlice();                                   }
  void                        setSliceMap( SliceMap map )                            { m_sliceMap = map;                                                         }
  uint32_t                    getFirstCtuRsAddrInSlice() const                       { return m_sliceMap.getCtuAddrInSlice(0);                                   }
  void                        setSliceID( uint32_t u )                               { m_sliceMap.setSliceID( u );                                               }
  uint32_t                    getSliceID() const                                     { return m_sliceMap.getSliceID();                                           }
  uint32_t                    getNumCtuInSlice() const                               { return m_sliceMap.getNumCtuInSlice();                                     }
  uint32_t                    getCtuAddrInSlice( int idx ) const                     { return m_sliceMap.getCtuAddrInSlice( idx );                               }
  void                        initSliceMap()                                         { m_sliceMap.initSliceMap();                                                }
  void                        addCtusToSlice( uint32_t startX, uint32_t stopX,
                                              uint32_t startY, uint32_t stopY,
                                              uint32_t picWidthInCtbsY )             { m_sliceMap.addCtusToSlice(startX, stopX, startY, stopY, picWidthInCtbsY); }
  void                        setIndependentSliceIdx( uint32_t i)                        { m_independentSliceIdx = i;                                    }
  uint32_t                        getIndependentSliceIdx() const                         { return  m_independentSliceIdx;                                }
  void                        copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);
  void                        setSliceBits( uint32_t uiVal )                             { m_sliceBits = uiVal;                                          }
  uint32_t                        getSliceBits() const                                   { return m_sliceBits;                                           }
  void                        setFinalized( bool uiVal )                             { m_bFinalized = uiVal;                                         }
  bool                        getFinalized() const                                   { return m_bFinalized;                                          }
  bool                        testWeightPred( ) const                                { return m_bTestWeightPred;                                     }
  void                        setTestWeightPred( bool bValue )                       { m_bTestWeightPred = bValue;                                   }
  bool                        testWeightBiPred( ) const                              { return m_bTestWeightBiPred;                                   }
  void                        setTestWeightBiPred( bool bValue )                     { m_bTestWeightBiPred = bValue;                                 }
  void                        setWpScaling( WPScalingParam  wp[NUM_REF_PIC_LIST_01][MAX_NUM_REF][MAX_NUM_COMPONENT] )
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam)*NUM_REF_PIC_LIST_01*MAX_NUM_REF*MAX_NUM_COMPONENT);
  }
  void                        setWpScaling(WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_REF_PIC_LIST_01 * MAX_NUM_REF * MAX_NUM_COMPONENT);
  }
  WPScalingParam *            getWpScalingAll()                                      { return (WPScalingParam *) m_weightPredTable;                  }
  WPScalingParam *            getWpScaling(const RefPicList refPicList, const int refIdx);
  const WPScalingParam *      getWpScaling(const RefPicList refPicList, const int refIdx) const;

  void                        resetWpScaling();
  void                        initWpScaling(const SPS *sps);

  void                        setWpAcDcParam( WPACDCParam wp[MAX_NUM_COMPONENT] )    { memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam)*MAX_NUM_COMPONENT); }

  void                        getWpAcDcParam( const WPACDCParam *&wp ) const;
  void                        initWpAcDcParam();

  void                        clearSubstreamSizes( )                                 { return m_substreamSizes.clear();                              }
  uint32_t                        getNumberOfSubstreamSizes( )                           { return (uint32_t) m_substreamSizes.size();                        }
  void                        addSubstreamSize( uint32_t size )                          { m_substreamSizes.push_back(size);                             }
  uint32_t                        getSubstreamSize( uint32_t idx )                           { CHECK(idx>=getNumberOfSubstreamSizes(),"Invalid index"); return m_substreamSizes[idx]; }
  void                        resetNumberOfSubstream()                               { m_numSubstream = 0;                                           }
  uint32_t                    getNumberOfSubstream()                                 { return (uint32_t) m_numSubstream;                             }
  void                        increaseNumberOfSubstream()                            { m_numSubstream++;                                             }

  void                        setCabacInitFlag( bool val )                           { m_cabacInitFlag = val;                                        } //!< set CABAC initial flag
  bool                        getCabacInitFlag()                               const { return m_cabacInitFlag;                                       } //!< get CABAC initial flag
#if JVET_AG0196_CABAC_RETRAIN
  void                        setCabacInitSliceType( SliceType val )                 { m_cabacInitSliceType = val; }
  SliceType                   getCabacInitSliceType()                          const { return m_cabacInitSliceType; }
#endif

  void                        setEncCABACTableIdx( SliceType idx )                   { m_encCABACTableIdx = idx;                                     }
  SliceType                   getEncCABACTableIdx() const                            { return m_encCABACTableIdx;                                    }


  void                        setSliceQpBase( int i )                                { m_iSliceQpBase = i;                                           }
  int                         getSliceQpBase()                                 const { return m_iSliceQpBase;                                        }

  void                        setDefaultClpRng( const SPS& sps );
  const ClpRngs&              clpRngs()                                         const { return m_clpRngs;}
  const ClpRng&               clpRng( ComponentID id)                           const { return m_clpRngs.comp[id];}
  ClpRngs&                    getClpRngs()                                            { return m_clpRngs;}
  unsigned                    getMinPictureDistance(
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                                     unsigned ibcFastMethod
#endif
                              )                           const ;
  void startProcessingTimer();
  void stopProcessingTimer();
  void resetProcessingTime()       { m_dProcessingTime = m_iProcessingStartTime = 0; }
  double getProcessingTime() const { return m_dProcessingTime; }


#if ALF_IMPROVEMENT  
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  bool                        getTileGroupAlfReuseFlag(ComponentID compId) const { return m_tileGroupAlfReuseFlag[compId]; }
  void                        setTileGroupAlfReuseFlag(ComponentID compId, bool b) { m_tileGroupAlfReuseFlag[compId] = b; }
  bool                        getTileGroupCcalfReuseFlag( ComponentID compId ) const { return m_tileGroupCcalfReuseFlag[compId]; }
  void                        setTileGroupCcalfReuseFlag(ComponentID compId, bool b) { m_tileGroupCcalfReuseFlag[compId] = b; }
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  int                         getTileGroupAlfFixedFilterSetIdx( ComponentID compId ) const { return m_tileGroupAlfFixedFilterSetIdx[compId]; }
  void                        setTileGroupAlfFixedFilterSetIdx( ComponentID compId, int i ) { m_tileGroupAlfFixedFilterSetIdx[compId] = i; }
#else
  int                         getTileGroupAlfFixedFilterSetIdx() const { return m_tileGroupAlfFixedFilterSetIdx; }
  void                        setTileGroupAlfFixedFilterSetIdx(int i) { m_tileGroupAlfFixedFilterSetIdx = i; }
#endif
#endif
  void                        resetTileGroupAlfEnabledFlag() { memset(m_tileGroupAlfEnabledFlag, 0, sizeof(m_tileGroupAlfEnabledFlag)); }
  bool                        getTileGroupAlfEnabledFlag(ComponentID compId) const { return m_tileGroupAlfEnabledFlag[compId]; }
  void                        setTileGroupAlfEnabledFlag(ComponentID compId, bool b) { m_tileGroupAlfEnabledFlag[compId] = b; }
  int                         getTileGroupNumAps() const { return m_tileGroupNumAps; }
  void                        setTileGroupNumAps(int i) { m_tileGroupNumAps = i; }
  int                         getTileGroupApsIdChroma() const { return m_tileGroupChromaApsId; }
  void                        setTileGroupApsIdChroma(int i) { m_tileGroupChromaApsId = i; }
  std::vector<int32_t>        getTileGroupApsIdLuma() const { return m_tileGroupLumaApsId; }
  void                        setAlfAPSs(std::vector<int> ApsIDs)
  {
    m_tileGroupLumaApsId.resize(m_tileGroupNumAps);
    for (int i = 0; i < m_tileGroupNumAps; i++)
    {
      m_tileGroupLumaApsId[i] = ApsIDs[i];
    }
  }
  void resetTileGroupCcAlCbfEnabledFlag() { m_tileGroupCcAlfCbEnabledFlag = 0; }
  void resetTileGroupCcAlCrfEnabledFlag() { m_tileGroupCcAlfCrEnabledFlag = 0; }

  void setTileGroupCcAlfCbEnabledFlag(bool b) { m_tileGroupCcAlfCbEnabledFlag = b; }
  void setTileGroupCcAlfCrEnabledFlag(bool b) { m_tileGroupCcAlfCrEnabledFlag = b; }
  void setTileGroupCcAlfCbApsId(int i) { m_tileGroupCcAlfCbApsId = i; }
  void setTileGroupCcAlfCrApsId(int i) { m_tileGroupCcAlfCrApsId = i; }

  bool getTileGroupCcAlfCbEnabledFlag() { return m_tileGroupCcAlfCbEnabledFlag; }
  bool getTileGroupCcAlfCrEnabledFlag() { return m_tileGroupCcAlfCrEnabledFlag; }
  int  getTileGroupCcAlfCbApsId() { return m_tileGroupCcAlfCbApsId; }
  int  getTileGroupCcAlfCrApsId() { return m_tileGroupCcAlfCrApsId; }
#if JVET_AK0065_TALF
  TAlfCtbParam* m_tAlfCtbControl;  // ctb controls
  void        setTileGroupTAlfControl(TAlfControl c) { m_tileGroupTAlfControl = c;    }
  TAlfControl getTileGroupTAlfControl() const         { return m_tileGroupTAlfControl; }
#endif
  void                        setDisableSATDForRD(bool b) { m_disableSATDForRd = b; }
  bool                        getDisableSATDForRD() { return m_disableSATDForRd; }
  void                        setLossless(bool b) { m_isLossless = b; }
  bool                        isLossless() const { return m_isLossless; }
#if JVET_Y0128_NON_CTC
#if JVET_AK0065_TALF
  bool                        scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS** apss2, APS* lmcsAps, APS* scalingListAps, const bool isDecoder );
#else
  bool                        scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder );
#endif
#else
  void                        scaleRefPicList( Picture *scaledRefPic[ ], PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps, const bool isDecoder );
#endif
  void                        freeScaledRefPicList( Picture *scaledRefPic[] );
  bool                        checkRPR();
  const std::pair<int, int>&  getScalingRatio( const RefPicList refPicList, const int refIdx )  const { CHECK( refIdx < 0, "Invalid reference index" ); return m_scalingRatio[refPicList][refIdx]; }
  void                        setNumSubstream( const SPS *sps, const PPS *pps );
  void                        setNumEntryPoints( const SPS *sps, const PPS *pps );
  uint32_t                    getNumEntryPoints( ) const { return m_numEntryPoints;  }
#if JVET_Q0406_CABAC_ZERO
  bool                        isLastSliceInSubpic();
#endif

#if JVET_W0066_CCSAO
  CcSaoComParam               m_ccSaoComParam;
  uint8_t*                    m_ccSaoControl[MAX_NUM_COMPONENT];
#endif
  CcAlfFilterParam            m_ccAlfFilterParam;
  uint8_t*                    m_ccAlfFilterControl[2];

#if INTER_LIC
  bool                        getUseLIC()                                   const { return m_UseLIC; }
  void                        setUseLIC(bool b) { m_UseLIC = b; }
  void                        setUseLICOnPicLevel(bool fastPicDecision);
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool                        getUseIBC()                                   const { return m_useIBC; }
  void                        setUseIBC(bool b) { m_useIBC = b; }
#endif
#if JVET_AG0145_ADAPTIVE_CLIPPING
  void                        setLumaPelMax(int u)                                { m_lumaPelMax = u; }
  int                         getLumaPelMax()                               const { return  m_lumaPelMax; }
  void                        setLumaPelMin(int u)                                { m_lumaPelMin = u; }
  int                         getLumaPelMin()                               const { return  m_lumaPelMin; }
  void                        setAdaptiveClipQuant(bool b)                        { m_adaptiveClipQuant = b; };
  bool                        getAdaptiveClipQuant()                        const { return m_adaptiveClipQuant; };
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  void                        setOffsetRefinementDbf(bool b)                      { m_offsetRefinementDbf = b; }
  bool                        getOffsetRefinementDbf()                      const { return m_offsetRefinementDbf; }
  void                        setOffsetRefinementAlf(bool b)                      { m_offsetRefinementAlf = b; }
  bool                        getOffsetRefinementAlf()                      const { return m_offsetRefinementAlf; }
  void                        setOffsetRefinementDbfIdx(uint8_t b)                { m_offsetRefinementDbfIdx = b; }
  uint8_t                     getOffsetRefinementDbfIdx()                   const { return m_offsetRefinementDbfIdx; }
  void                        setOffsetRefinementAlfIdx(uint8_t b)                { m_offsetRefinementAlfIdx = b; }
  uint8_t                     getOffsetRefinementAlfIdx()                   const { return m_offsetRefinementAlfIdx; }
#endif

#if JVET_AJ0249_NEURAL_NETWORK_BASED
  bool getPnnMode() const { return m_pnnMode; }
  void setPnnMode(const bool pnnMode) { m_pnnMode = pnnMode; }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  void                        setSeparateTreeEnabled( const bool& b )                 { m_separateTreeEnabled = b;        }
  bool                        getSeparateTreeEnabled()                          const { return m_separateTreeEnabled;     }
  void                        setProcessingIntraRegion( const bool& b )               { m_processingIntraRegion = b;      }
  bool                        getProcessingIntraRegion()                        const { return m_processingIntraRegion;   }
  void                        setProcessingSeparateTrees( const bool& b )             { m_processingSeparateTrees = b;    }
  bool                        getProcessingSeparateTrees()                      const { return m_processingSeparateTrees; }
  void                        setProcessingChannelType( const ChannelType& ch )       { m_processingChannelType = ch;     }
  ChannelType                 getProcessingChannelType()                        const { return m_processingChannelType;   }
  void                        setIntraRegionRoot( Partitioner* p );
  void                        setCUIntraRegionRoot ( CodingUnit*cu );
  bool                        isIntraRegionRoot( Partitioner* p )               const ;
  void                        exitIntraRegionTesting();
  bool                        shouldCopyLumaCUs()                               const { return ( m_separateTreeEnabled && m_processingIntraRegion && m_processingSeparateTrees && m_processingChannelType == CH_C ); }
  bool                        processingIntraRegionChroma()                     const { return ( m_separateTreeEnabled && m_processingIntraRegion && m_processingSeparateTrees && m_processingChannelType == CH_C ); }
  void                        setIntraRegionNoSplitTest( const bool& b )              { m_intraRegionNoSplitTest = b;     }
  bool                        getIntraRegionNoSplitTest()                       const { return m_intraRegionNoSplitTest;  }
#endif

#if JVET_AL0153_ALF_CCCM
public:
  bool                    m_lfCccmEnabledFlag;
  bool                    getLfCccmEnabledFlag() const                                                    { return m_lfCccmEnabledFlag; }
  void                    setLfCccmEnabledFlag( bool b )                                                  { m_lfCccmEnabledFlag = b; }
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  bool                    m_lfCccmImpEnabledFlag;
  bool                    getLfCccmImpEnabledFlag() const                                                 { return m_lfCccmImpEnabledFlag; }
  void                    setLfCccmImpEnabledFlag(bool b)                                                 { m_lfCccmImpEnabledFlag = b; }
  int                     m_lfCccmImpFactorIdx;
  int                     getLfCccmImpFactorIdx() const                                                   { return m_lfCccmImpFactorIdx; }
  void                    setLfCccmImpFactorIdx(int b)                                                    { m_lfCccmImpFactorIdx = b; }
#endif
  std::vector<int8_t>     m_lfCccmEnabled;
  std::vector<int8_t>     m_lfCccmWindowSizeIndex;
  std::vector<int8_t>     m_lfCccmModelType;
  std::vector<int8_t>     m_lfCccmCTUMerge;
  int8_t                  m_lfCccmFrameLevelInherit;
  void                    lfCccmClearControlInformation(const int ctuRsAddr = -1);
  void                    lfCccmMerge(const int ctuRsAddr);
  const Picture*          lfCccmGetReferencePicture() const;
  lfCccmCand              lfCccmGetCandidate(const int ctuRsAddr) const;
  std::vector<lfCccmCand> lfCccmGetMergeCandidates(const int ctuRsAddr) const;
#endif
protected:
  Picture*              xGetRefPic( PicList& rcListPic, int poc, const int layerId );
  Picture*              xGetLongTermRefPic( PicList& rcListPic, int poc, bool pocHasMsb, const int layerId );
public:
  std::unordered_map< Position, std::unordered_map< Size, double> > m_mapPltCost[2];
private:

};// END CLASS DEFINITION Slice


class PreCalcValues
{
public:
  PreCalcValues( const SPS& sps, const PPS& pps, bool _isEncoder )
    : chrFormat           ( sps.getChromaFormatIdc() )
    , multiBlock422       ( false )
    , maxCUWidth          ( sps.getMaxCUWidth() )
    , maxCUHeight         ( sps.getMaxCUHeight() )
    , maxCUWidthMask      ( maxCUWidth  - 1 )
    , maxCUHeightMask     ( maxCUHeight - 1 )
    , maxCUWidthLog2      ( floorLog2( maxCUWidth  ) )
    , maxCUHeightLog2     ( floorLog2( maxCUHeight ) )
    , minCUWidth          ( 1 << MIN_CU_LOG2 )
    , minCUHeight         ( 1 << MIN_CU_LOG2 )
    , minCUWidthLog2      ( floorLog2( minCUWidth  ) )
    , minCUHeightLog2     ( floorLog2( minCUHeight ) )
    , partsInCtuWidth     ( maxCUWidth >> MIN_CU_LOG2)
    , partsInCtuHeight    ( maxCUHeight >> MIN_CU_LOG2)
    , partsInCtu          ( partsInCtuWidth * partsInCtuHeight )
    , widthInCtus         ( (pps.getPicWidthInLumaSamples () + sps.getMaxCUWidth () - 1) / sps.getMaxCUWidth () )
    , heightInCtus        ( (pps.getPicHeightInLumaSamples() + sps.getMaxCUHeight() - 1) / sps.getMaxCUHeight() )
    , sizeInCtus          ( widthInCtus * heightInCtus )
#if NN_LF_UNIFIED
    , widthInNnlfunifiedInferSize {(pps.getPicWidthInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_SMALL) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_SMALL), (pps.getPicWidthInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_BASE) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_BASE), (pps.getPicWidthInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_LARGE) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_LARGE)}
    , heightInNnlfUnifiedInferSize {(pps.getPicHeightInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_SMALL) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_SMALL), (pps.getPicHeightInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_BASE) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_BASE), (pps.getPicHeightInLumaSamples () + sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_LARGE) - 1) / sps.getNnlfUnifiedInferSize(NNLF_UNIFIED_INFER_GRANULARITY_LARGE)}
    , sizeInNnlfUnifiedInferSize {widthInNnlfunifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_SMALL] * heightInNnlfUnifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_SMALL], widthInNnlfunifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_BASE] * heightInNnlfUnifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_BASE], widthInNnlfunifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_LARGE] * heightInNnlfUnifiedInferSize[NNLF_UNIFIED_INFER_GRANULARITY_LARGE]}
#endif
    , lumaWidth           ( pps.getPicWidthInLumaSamples() )
    , lumaHeight          ( pps.getPicHeightInLumaSamples() )
    , fastDeltaQPCuMaxSize( Clip3(1u << sps.getLog2MinCodingBlockSize(), sps.getMaxCUHeight(), 32u) )
    , noChroma2x2         (  false )
    , isEncoder           ( _isEncoder )
    , ISingleTree         ( !sps.getUseDualITree() )
    , maxBtDepth          { sps.getMaxMTTHierarchyDepthI(), sps.getMaxMTTHierarchyDepth(), sps.getMaxMTTHierarchyDepthIChroma() }
    , minBtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxBtSize           { sps.getMaxBTSizeI(), sps.getMaxBTSize(), sps.getMaxBTSizeIChroma() }
    , minTtSize           { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() }
    , maxTtSize           { sps.getMaxTTSizeI(), sps.getMaxTTSize(), sps.getMaxTTSizeIChroma() }
    , minQtSize           { sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(B_SLICE, CHANNEL_TYPE_LUMA), sps.getMinQTSize(I_SLICE, CHANNEL_TYPE_CHROMA) }
  {}

  const ChromaFormat chrFormat;
  const bool         multiBlock422;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtuHeight;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
#if NN_LF_UNIFIED
  const uint32_t     widthInNnlfunifiedInferSize[MAX_NUM_NNLF_UNIFIED_INFER_GRANULARITY];
  const uint32_t     heightInNnlfUnifiedInferSize[MAX_NUM_NNLF_UNIFIED_INFER_GRANULARITY];
  const uint32_t     sizeInNnlfUnifiedInferSize[MAX_NUM_NNLF_UNIFIED_INFER_GRANULARITY];
#endif
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;

private:
  const unsigned     maxBtDepth[3];
  const unsigned     minBtSize [3];
  const unsigned     maxBtSize [3];
  const unsigned     minTtSize [3];
  const unsigned     maxTtSize [3];
  const unsigned     minQtSize [3];

  unsigned getValIdx    ( const Slice &slice, const ChannelType chType ) const;

public:
  unsigned getMaxBtDepth( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxBtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMaxTtSize ( const Slice &slice, const ChannelType chType ) const;
  unsigned getMinQtSize ( const Slice &slice, const ChannelType chType ) const;
};

#if ENABLE_TRACING
void xTraceVPSHeader();
void xTraceDCIHeader();
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceAPSHeader();
void xTracePictureHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
#endif

#endif // __SLICE__
