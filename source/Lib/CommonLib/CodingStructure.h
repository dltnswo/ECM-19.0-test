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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#ifndef __CODINGSTRUCTURE__
#define __CODINGSTRUCTURE__

#include "Unit.h"
#include "Buffer.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "Slice.h"
#include <vector>


struct Picture;


enum PictureType
{
  PIC_RECONSTRUCTION = 0,
#if JVET_Z0118_GDR
  PIC_RECONSTRUCTION_0 = 0,
  PIC_RECONSTRUCTION_1 = 1,
  PIC_RECONSTRUCTION_SAVE = 2,
#endif
  PIC_ORIGINAL,
  PIC_TRUE_ORIGINAL,
  PIC_FILTERED_ORIGINAL,
  PIC_PREDICTION,
  PIC_RESIDUAL,
  PIC_ORG_RESI,
  PIC_RECON_WRAP,
  PIC_ORIGINAL_INPUT,
  PIC_TRUE_ORIGINAL_INPUT,
  PIC_FILTERED_ORIGINAL_INPUT,
#if NNVC_USE_BS
  PIC_BS_MAP,
#endif
#if NNVC_USE_PRED
  PIC_PREDICTION_CUSTOM,
#endif
#if NNVC_USE_REC_BEFORE_DBF
  PIC_REC_BEFORE_DBF,
#endif
#if JVET_AC0089_NNVC_USE_BPM_INFO
  PIC_BLOCK_PRED_MODE,
#endif
#if JVET_AJ0124_QP_BLOCK
  PIC_BLOCK_QP,
#endif
  NUM_PIC_TYPES
};
extern XUCache g_globalUnitCache;

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture         *picture;
  CodingStructure *parent;
  CodingStructure *bestCS;
  Slice           *slice;

  UnitScale        unitScale[MAX_NUM_COMPONENT];
#if JVET_AI0087_BTCUS_RESTRICTION 
  int         btFirstPartDecs[4];
#endif
  int         baseQP;
  int         prevQP[MAX_NUM_CHANNEL_TYPE];
  int         currQP[MAX_NUM_CHANNEL_TYPE];
  int         chromaQpAdj;
  const SPS *sps;
  const PPS *pps;
  PicHeader *picHeader;
#if JVET_AK0065_TALF
  APS*       talfApss[ALF_CTB_MAX_NUM_APS];
#endif
  APS*       alfApss[ALF_CTB_MAX_NUM_APS];
  APS *      lmcsAps;
  APS *      scalinglistAps;
  const VPS *vps;
  const PreCalcValues* pcv;

  CodingStructure(CUCache&, PUCache&, TUCache&);

#if JVET_Z0118_GDR
  bool isGdrEnabled() { return m_gdrEnabled; }       
  void create(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabled = false);
  void create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused, const bool isGdrEnabeld = false);
#else
  void create(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused);
  void create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer, const bool isPLTused);
#endif

  void destroy();
  void releaseIntermediateData();
  bool m_isTopLayer;
  void destroyTemporaryCsData();
  void createTemporaryCsData(const bool isPLTused);
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  void createTemporaryRefineTmvpCsData(bool isIntra, int* numRefPic, bool isDec);
#endif
#if JVET_Z0118_GDR
  bool isCuCrossIRA(int begX) const;
  bool isCuCrossVB(int endX) const;
  
  bool containRefresh(int begX, int endX) const;  
  bool overlapRefresh(int begX, int endX) const;  
  bool withinRefresh(int begX, int endX) const;  
  
  Area findOverlappedArea(const Area &a1, const Area &a2) const;
#endif

#if JVET_Z0118_GDR
  bool isInGdrIntervalOrRecoveryPoc() const;

  bool isClean(const ChannelType effChType) const;
  bool isClean(const Position &IntPos, RefPicList e, int refIdx) const;
  bool isClean(const Position &IntPos, const Picture* const ref_pic) const;
  bool isClean(const Position &IntPos, const ChannelType effChType) const;  
  bool isClean(const int x, const int y, const ChannelType effChType) const;  
  bool isClean(const Area &area, const ChannelType effChType) const;
  
  bool isClean(const CodingUnit &cu) const;
  bool isClean(const PredictionUnit &pu) const;  
  bool isClean(const TransformUnit &tu) const;
#endif

#if JVET_Z0118_GDR
  void updateReconMotIPM(const UnitArea &uarea) const;
  void updateReconMotIPM(const CompArea &carea) const;
  void updateReconMotIPM(const UnitArea &uarea, const CPelUnitBuf &pbuf) const;  
  void updateReconMotIPM(const CompArea &carea, const CPelBuf &pbuf) const;
#endif

#if JVET_Z0118_GDR
  void rspSignalPicture(const UnitArea &uarea, std::vector<Pel>& pLUT, const bool usePred = true) const;
  void rspSignalPicture(const CompArea &carea, std::vector<Pel>& pLUT, const bool usePred = true) const;
  void reconstructPicture(const CompArea &carea, std::vector<Pel>& pLUT, CodingStructure *resiCS, bool lmcsEnable) const;
#endif

  void rebindPicBufs();
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  void createTMBuf(const int cbWidth, const int cbHeight);
  void destroyTMBuf();
#endif

  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  bool isDecomp (const Position &pos, const ChannelType _chType) const;
  bool isDecomp (const Position &pos, const ChannelType _chType);
  void setDecomp(const CompArea &area, const bool _isCoded = true);
  void setDecomp(const UnitArea &area, const bool _isCoded = true);

  const CodingUnit     *getCU(const Position &pos, const ChannelType _chType) const;
  const PredictionUnit *getPU(const Position &pos, const ChannelType _chType) const;
  const TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1) const;

#if JVET_AH0135_TEMPORAL_PARTITIONING
  CodingUnit     *getCUTempo(const Position& pos, const ChannelType effChType);
#endif
  CodingUnit     *getCU(const Position &pos, const ChannelType _chType);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CodingUnit     *getLumaCU( const Position &pos );
#endif  
  PredictionUnit *getPU(const Position &pos, const ChannelType _chType);
  TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1);

  const CodingUnit     *getCU(const ChannelType &_chType) const { return getCU(area.blocks[_chType].pos(), _chType); }
  const PredictionUnit *getPU(const ChannelType &_chType) const { return getPU(area.blocks[_chType].pos(), _chType); }
  const TransformUnit  *getTU(const ChannelType &_chType) const { return getTU(area.blocks[_chType].pos(), _chType); }

  CodingUnit     *getCU(const ChannelType &_chType ) { return getCU(area.blocks[_chType].pos(), _chType); }
  PredictionUnit *getPU(const ChannelType &_chType ) { return getPU(area.blocks[_chType].pos(), _chType); }
  TransformUnit  *getTU(const ChannelType &_chType ) { return getTU(area.blocks[_chType].pos(), _chType); }

  const CodingUnit     *getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const;
  const CodingUnit     *getCURestricted(const Position &pos, const CodingUnit& curCu,                               const ChannelType _chType) const;
  const PredictionUnit *getPURestricted(const Position &pos, const PredictionUnit& curPu,                           const ChannelType _chType) const;
  const TransformUnit  *getTURestricted(const Position &pos, const TransformUnit& curTu,                            const ChannelType _chType) const;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType, const PartSplit& implicitSplit = CU_DONT_SPLIT);
#else
  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType);
#endif
  PredictionUnit& addPU(const UnitArea &unit, const ChannelType _chType);
  TransformUnit&  addTU(const UnitArea &unit, const ChannelType _chType);
  void            addEmptyTUs(Partitioner &partitioner);

  CUTraverser     traverseCUs(const UnitArea& _unit, const ChannelType _chType);
  PUTraverser     traversePUs(const UnitArea& _unit, const ChannelType _chType);
  TUTraverser     traverseTUs(const UnitArea& _unit, const ChannelType _chType);

  cCUTraverser    traverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cPUTraverser    traversePUs(const UnitArea& _unit, const ChannelType _chType) const;
  cTUTraverser    traverseTUs(const UnitArea& _unit, const ChannelType _chType) const;

  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  static_vector<double, NUM_ENC_FEATURES> features;

#if JVET_Y0152_TT_ENC_SPEEDUP
  double     *splitRdCostBest; //[Partition::NUM_PART_SPLIT];
#endif
  double      cost;
  bool        useDbCost;
  double      costDbOffset;
  double      lumaCost;
  uint64_t      fracBits;
  Distortion  dist;
  Distortion  interHad;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  TreeType    treeType; //because partitioner can not go deep to tu and cu coding (e.g., addCU()), need another variable for indicating treeType
  ModeType    modeType;
#endif
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
  SplitPred   splitPredLuma;
#endif
  void initStructData  (const int &QP = MAX_INT, const bool &skipMotBuf = false);
  void initSubStructure(      CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool &isTuEnc);

  void copyStructure   (const CodingStructure& cs, const ChannelType chType, const bool copyTUs = false, const bool copyRecoBuffer = false);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi, const bool updateCost);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType,                          const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi, const bool updateCost) { useSubStructure(cs, chType, cs.area, cpyPred, cpyReco, cpyOrgResi, cpyResi, updateCost); }

  void clearTUs();
  void clearPUs();
  void clearCUs();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const int signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const;
  void clearCuPuTuIdxMap  ( const UnitArea &_area, uint32_t numCu, uint32_t numPu, uint32_t numTu, uint32_t* pOffset );
  void getNumCuPuTuOffset ( uint32_t* pArray )
  {
    pArray[0] = m_numCUs;     pArray[1] = m_numPUs;     pArray[2] = m_numTUs;
    pArray[3] = m_offsets[0]; pArray[4] = m_offsets[1]; pArray[5] = m_offsets[2];
  }
#endif
#if MULTI_HYP_PRED
  MEResultVec m_meResults;
#endif

private:
  void createCoeffs(const bool isPLTused);
  void createInternals(const UnitArea& _unit, const bool isTopLayer, const bool isPLTused);

public:

  std::vector<    CodingUnit*> cus;
  std::vector<PredictionUnit*> pus;
  std::vector< TransformUnit*> tus;



  LutMotionCand motionLut;

#if JVET_AD0188_CCP_MERGE
  LutCCP                 ccpLut;
  template<class T> void addCCPToLut(static_vector<T, MAX_NUM_HCCP_CANDS> &lut, const T &model, int reusePos);
  template<class T> void getOneModelFromCCPLut(const static_vector<T, MAX_NUM_HCCP_CANDS> &lut, T &model, int pos);
#endif
#if JVET_AG0058_EIP
  LutEIP                 eipLut;
  template<class T> void addEipToLut(static_vector<T, MAX_NUM_HEIP_CANDS> &lut, const T &model, int reusePos);
  template<class T> void getOneModelFromEipLut(const static_vector<T, MAX_NUM_HEIP_CANDS> &lut, T &model, int pos);
#endif
  void addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS>& lut, const MotionInfo &mi);
#if JVET_Z0075_IBC_HMVP_ENLARGE
  void addMiToLutIBC(static_vector<MotionInfo, MAX_NUM_HMVP_IBC_CANDS>& lut, const MotionInfo &mi);
#endif

#if JVET_Z0139_HIST_AFF
  void addAffMiToLut(static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutSet, const AffineMotionInfo addMi[2], int refIdx[2]);
  void addAffInheritToLut(static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS>& lut, const AffineInheritInfo& mi);
#endif

  PLTBuf prevPLT;
  void resetPrevPLT(PLTBuf& prevPLT);
  void reorderPrevPLT(PLTBuf& prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE], Pel curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE], bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin, uint32_t numComp, bool jointPLT);
  void setPrevPLT(PLTBuf predictor);
  void storePrevPLT(PLTBuf& predictor);

private:

  // needed for TU encoding
  bool m_isTuEnc;

  unsigned *m_cuIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_puIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_tuIdx   [MAX_NUM_CHANNEL_TYPE];
  bool     *m_isDecomp[MAX_NUM_CHANNEL_TYPE];

  unsigned m_numCUs;
  unsigned m_numPUs;
  unsigned m_numTUs;

  CUCache& m_cuCache;
  PUCache& m_puCache;
  TUCache& m_tuCache;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_pred;
#if NNVC_USE_PRED
  PelStorage m_predCustom;
#endif  
#if JVET_AC0089_NNVC_USE_BPM_INFO
  PelStorage m_block_pred_mode;
#endif
  PelStorage m_resi;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
public:
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  Pel   *m_pCurrTmTop;
  Pel   *m_pCurrTmLeft;
  Pel   *m_pRefTmTop;
  Pel   *m_pRefTmLeft;
#endif
  PelBuf m_pcBufPredCurTop;
  PelBuf m_pcBufPredCurLeft;
  PelBuf m_pcBufPredRefTop;
  PelBuf m_pcBufPredRefLeft;
#endif
#if JVET_AE0043_CCP_MERGE_TEMPORAL
public:
  int    *m_ccpmIdxBuf;
  std::vector<CCPModelCandidate> m_ccpModelLUT;
#endif
#if JVET_AG0058_EIP
public:
  int    *m_eipIdxBuf;
  std::vector<EipModelCandidate> m_eipModelLUT;
#endif
#if JVET_Z0118_GDR
public:
  PictureType m_pt; 
private:
  PelStorage m_reco0; // for GDR dirty
  PelStorage m_reco1; // for GDR clean
#else
  PelStorage m_reco;
#endif
  PelStorage m_orgr;

  TCoeff *m_coeffs [ MAX_NUM_COMPONENT ];
#if SIGN_PREDICTION
  SIGN_PRED_TYPE *m_coeffSigns[MAX_NUM_COMPONENT];
#if JVET_Y0141_SIGN_PRED_IMPROVE
  unsigned *m_coeffSignsIdx[MAX_NUM_COMPONENT];
#endif
#endif
#if REMOVE_PCM
  Pel    *m_pltIdx[MAX_NUM_CHANNEL_TYPE];
#else
  Pel    *m_pcmbuf [ MAX_NUM_COMPONENT ];
#endif
  bool   *m_runType[ MAX_NUM_CHANNEL_TYPE ];
  int     m_offsets[ MAX_NUM_COMPONENT ];

#if JVET_Z0118_GDR
  MotionInfo *m_motionBuf0;
  MotionInfo *m_motionBuf1;
#else
  MotionInfo *m_motionBuf;
#endif
#if JVET_AH0135_TEMPORAL_PARTITIONING
  SplitPred *currQtDepthBuf;
#endif

#if JVET_W0123_TIMD_FUSION
#if JVET_Z0118_GDR
  uint8_t *m_ipmBuf0;
  uint8_t *m_ipmBuf1;
#else
  uint8_t *m_ipmBuf;
#endif  
#endif

#if JVET_Z0118_GDR
  bool m_gdrEnabled;
#endif

#if JVET_AI0183_MVP_EXTENSION
  IntersectingMvData *m_intersectingMvBuf;
public:
  IntersectingMvData* getIntersectingMvBuf()     { return m_intersectingMvBuf; };
#else
public:
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  int* tmvpRefinedMv[2/*colIdx*/][NUM_REF_PIC_LIST_01/*colRefList*/][NUM_REF_PIC_LIST_01/*cureRefList*/][MAX_NUM_REF/*cureRefIdx*/][2/*non-SbTmvp, SbTmvp*/][2/*hor, ver*/];
  int* tmvpRefinedAvailable[2/*colIdx*/];
#endif
  CodingStructure *bestParent;
  double        tmpColorSpaceCost;
  bool          firstColorSpaceSelected;
  double        tmpColorSpaceIntraCost[2];
  bool          firstColorSpaceTestOnly;
  bool resetIBCBuffer;
#if JVET_Z0136_OOB
  bool          *mcMask[2];
  bool          *mcMaskChroma[2];
#endif
  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }
  MotionBuf getMotionBuf()                        { return getMotionBuf(  area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }
  const CMotionBuf getMotionBuf()                        const { return getMotionBuf(  area.Y() ); }

  MotionInfo& getMotionInfo( const Position& pos );
  const MotionInfo& getMotionInfo( const Position& pos ) const;

#if JVET_AH0135_TEMPORAL_PARTITIONING
  void setSplitPred();

  QTDepthBuf getQtDepthBuf(const Area& _area);
  QTDepthBuf getQTDepthBuf()                      { return getQtDepthBuf(area.Y()); }

  SplitPred& getQtDepthInfo(const Position& pos);
#endif
#if JVET_AL0143_CHROMA_PARTITION_PREDICTION
  void chromaTreePred(CodingStructure& cs, Partitioner& partitioner);
#endif
#if JVET_AE0043_CCP_MERGE_TEMPORAL
  CCPModelIdxBuf getCcpmIdxBuf( const     Area& bufArea);
  CCPModelIdxBuf getCcpmIdxBuf( const UnitArea& bufArea) { return getCcpmIdxBuf( bufArea.Cb() ); }
  CCPModelIdxBuf getCcpmIdxBuf()                         { return getCcpmIdxBuf( area.Cb() ); }
  const CCCPModelIdxBuf getCcpmIdxBuf( const     Area& bufArea ) const;
  const CCCPModelIdxBuf getCcpmIdxBuf( const UnitArea& bufArea ) const { return getCcpmIdxBuf( bufArea.Cb() ); }
  const CCCPModelIdxBuf getCcpmIdxBuf()                          const { return getCcpmIdxBuf( area.Cb() ); }

  int& getCcpmIdxInfo( const Position& pos );
  const int& getCcpmIdxInfo( const Position& pos ) const;
#endif

#if JVET_AG0058_EIP
  EipModelIdxBuf getEipIdxBuf( const     Area& bufArea);
  EipModelIdxBuf getEipIdxBuf( const UnitArea& bufArea) { return getEipIdxBuf( bufArea.Y() ); }
  EipModelIdxBuf getEipIdxBuf()                         { return getEipIdxBuf( area.Y() ); }
  const CEipModelIdxBuf getEipIdxBuf( const     Area& bufArea ) const;
  const CEipModelIdxBuf getEipIdxBuf( const UnitArea& bufArea ) const { return getEipIdxBuf( bufArea.Y() ); }
  const CEipModelIdxBuf getEipIdxBuf()                          const { return getEipIdxBuf( area.Y() ); }

  int& getEipIdxInfo( const Position& pos );
  const int& getEipIdxInfo( const Position& pos ) const;
#endif

#if JVET_W0123_TIMD_FUSION && RPR_ENABLE
  bool  picContain( const Position _pos );
#endif

#if JVET_Z0118_GDR
  MotionBuf getMotionBuf(const     Area& _area, PictureType pt);
  MotionBuf getMotionBuf(const UnitArea& _area, PictureType pt) { return getMotionBuf(_area.Y(), pt); }
  MotionBuf getMotionBuf(PictureType pt) { return getMotionBuf(area.Y(), pt); }

  const CMotionBuf getMotionBuf(const     Area& _area, PictureType pt) const;
  const CMotionBuf getMotionBuf(const UnitArea& _area, PictureType pt) const { return getMotionBuf(_area.Y(), pt); }
  const CMotionBuf getMotionBuf(PictureType pt)                        const { return getMotionBuf(area.Y(), pt); }

  MotionInfo& getMotionInfo(const Position& pos, PictureType pt);
  const MotionInfo& getMotionInfo(const Position& pos, PictureType pt) const;
#endif

#if JVET_W0123_TIMD_FUSION
  IpmBuf getIpmBuf( const     Area& _area );
  IpmBuf getIpmBuf( const UnitArea& _area ) { return getIpmBuf( _area.Y() ); }
  IpmBuf getIpmBuf()                        { return getIpmBuf(  area.Y() ); }

  const CIpmBuf getIpmBuf( const     Area& _area ) const;
  const CIpmBuf getIpmBuf( const UnitArea& _area ) const { return getIpmBuf( _area.Y() ); }
  const CIpmBuf getIpmBuf()                        const { return getIpmBuf(  area.Y() ); }

  uint8_t& getIpmInfo( const Position& pos );
  const uint8_t& getIpmInfo( const Position& pos ) const;
#endif

#if JVET_W0123_TIMD_FUSION && JVET_Z0118_GDR
  IpmBuf getIpmBuf(const     Area& _area, PictureType pt);
  IpmBuf getIpmBuf(const UnitArea& _area, PictureType pt) { return getIpmBuf(_area.Y(), pt); }
  IpmBuf getIpmBuf(PictureType pt)                        { return getIpmBuf(area.Y(), pt); }

  const CIpmBuf getIpmBuf(const     Area& _area, PictureType pt) const;
  const CIpmBuf getIpmBuf(const UnitArea& _area, PictureType pt) const { return getIpmBuf(_area.Y(), pt); }
  const CIpmBuf getIpmBuf(PictureType pt)                        const { return getIpmBuf(area.Y(), pt); }

  uint8_t& getIpmInfo(const Position& pos, PictureType pt);
  const uint8_t& getIpmInfo(const Position& pos, PictureType pt) const;
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  std::vector<    CodingUnit*> *m_lumaCUs;
  std::vector<PredictionUnit*> *m_lumaPUs;
  std::vector< TransformUnit*> *m_lumaTUs;
  UnitScale                    *m_lumaUnitScale;
  UnitArea                     *m_lumaArea;
  unsigned                     *m_lumaCuIdx;
  unsigned                     *m_lumaPuIdx;
  unsigned                     *m_lumaTuIdx;
  CodingStructure              *m_lumaParent;
  CodingUnit                   *m_bestCU;
  CodingUnit                   *m_lastCodedCU;
  double                        m_savedCost;
  uint64_t                      m_lumaFracBits;
  Distortion                    m_lumaDist;
  double                        m_lumaCost;

  void copyLumaPointers( CodingStructure& cs );
  void setLumaPointers ( CodingStructure& cs );
  void popLastCU( const PartSplit& implicitSplit );
  void deriveSeparateTreeFlagInference( int& separateTreeFlag, bool& inferredSeparateTreeFlag, int width, int height, bool canSplit );
  void determineIfSeparateTreeFlagInferred( bool& inferredSeparateTreeFlag, int width, int height, bool canSplit );

  /*const*/ CodingUnit     *getLumaCU(const Position &pos, const ChannelType _chType) const;
  const PredictionUnit *getLumaPU(const Position &pos, const ChannelType _chType) const;
  const TransformUnit  *getLumaTU(const Position &pos, const ChannelType _chType) const;
#endif

public:
  // ---------------------------------------------------------------------------
  // temporary (shadowed) data accessors
  // ---------------------------------------------------------------------------
         PelBuf       getPredBuf(const CompArea &blk);
  const CPelBuf       getPredBuf(const CompArea &blk) const;
         PelUnitBuf   getPredBuf(const UnitArea &unit);
  const CPelUnitBuf   getPredBuf(const UnitArea &unit) const;

#if NNVC_USE_PRED
         PelBuf       getPredBufCustom(const CompArea &blk);
  const CPelBuf       getPredBufCustom(const CompArea &blk) const;
         PelUnitBuf   getPredBufCustom(const UnitArea &unit);
  const CPelUnitBuf   getPredBufCustom(const UnitArea &unit) const;
#endif

         PelBuf       getResiBuf(const CompArea &blk);
  const CPelBuf       getResiBuf(const CompArea &blk) const;
         PelUnitBuf   getResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getResiBuf(const UnitArea &unit) const;

         PelBuf       getRecoBuf(const CompArea &blk);
  const CPelBuf       getRecoBuf(const CompArea &blk) const;
         PelUnitBuf   getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf   getRecoBuf(const UnitArea &unit) const;
#if JVET_Z0118_GDR
         PelUnitBuf&  getRecoBufRef() { return (m_pt == PIC_RECONSTRUCTION_0) ? m_reco0 : m_reco1; }
#else
         PelUnitBuf&  getRecoBufRef() { return m_reco; }
#endif

         PelBuf       getOrgResiBuf(const CompArea &blk);
  const CPelBuf       getOrgResiBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgResiBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const CompArea &blk);
  const CPelBuf       getOrgBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const ComponentID &compID);
  const CPelBuf       getOrgBuf(const ComponentID &compID) const;
         PelUnitBuf   getOrgBuf();
  const CPelUnitBuf   getOrgBuf() const;
#if ALF_SAO_TRUE_ORG
         PelUnitBuf   getTrueOrgBuf();
  const CPelUnitBuf   getTrueOrgBuf() const;
#endif

  // pred buffer
         PelBuf       getPredBuf(const ComponentID &compID)       { return m_pred.get(compID); }
  const CPelBuf       getPredBuf(const ComponentID &compID) const { return m_pred.get(compID); }
         PelUnitBuf   getPredBuf()                                { return m_pred; }
  const CPelUnitBuf   getPredBuf()                          const { return m_pred; }

  // resi buffer
         PelBuf       getResiBuf(const ComponentID compID)        { return m_resi.get(compID); }
  const CPelBuf       getResiBuf(const ComponentID compID)  const { return m_resi.get(compID); }
         PelUnitBuf   getResiBuf()                                { return m_resi; }
  const CPelUnitBuf   getResiBuf()                          const { return m_resi; }

  // org-resi buffer
         PelBuf       getOrgResiBuf(const ComponentID &compID)       { return m_orgr.get(compID); }
  const CPelBuf       getOrgResiBuf(const ComponentID &compID) const { return m_orgr.get(compID); }
         PelUnitBuf   getOrgResiBuf()                                { return m_orgr; }
  const CPelUnitBuf   getOrgResiBuf()                          const { return m_orgr; }

  // reco buffer
#if JVET_Z0118_GDR
        PelBuf        getRecoBuf(const ComponentID compID)         { return (m_pt == PIC_RECONSTRUCTION_0) ? m_reco0.get(compID) : m_reco1.get(compID); }         
  const CPelBuf       getRecoBuf(const ComponentID compID)   const { return (m_pt == PIC_RECONSTRUCTION_0) ? m_reco0.get(compID) : m_reco1.get(compID); }       
        PelUnitBuf    getRecoBuf()                                 { return (m_pt == PIC_RECONSTRUCTION_0) ? m_reco0 : m_reco1; }         
  const CPelUnitBuf   getRecoBuf()                           const { return (m_pt == PIC_RECONSTRUCTION_0) ? m_reco0 : m_reco1; }                
#else
         PelBuf       getRecoBuf(const ComponentID compID)         { return m_reco.get(compID); }
  const CPelBuf       getRecoBuf(const ComponentID compID)   const { return m_reco.get(compID); }
         PelUnitBuf   getRecoBuf()                                 { return m_reco; }
  const CPelUnitBuf   getRecoBuf()                           const { return m_reco; }
#endif

#if JVET_AC0089_NNVC_USE_BPM_INFO
  // block prediction mode buffer
  PelBuf            getBlockPredModeBuf(const ComponentID compID) { return m_block_pred_mode.get(compID); }
  const CPelBuf     getBlockPredModeBuf(const ComponentID compID) const { return m_block_pred_mode.get(compID); }
  PelUnitBuf        getBlockPredModeBuf() { return m_block_pred_mode; }
  const CPelUnitBuf getBlockPredModeBuf() const { return m_block_pred_mode; }
#endif

#if JVET_Z0118_GDR // setReconBuf
  void setReconBuf(const PictureType pt)
  {
    m_pt = pt;
  }

  PictureType checkReconBuf()
  {
    return m_pt;
  }
#endif
private:

  inline        PelBuf       getBuf(const CompArea &blk,  const PictureType &type);
  inline const CPelBuf       getBuf(const CompArea &blk,  const PictureType &type) const;
  inline        PelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type);
  inline const CPelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type) const;
};


static inline uint32_t getNumberValidTBlocks(const PreCalcValues& pcv) { return (pcv.chrFormat==CHROMA_400) ? 1 : ( pcv.multiBlock422 ? MAX_NUM_TBLOCKS : MAX_NUM_COMPONENT ); }

#if JVET_AD0188_CCP_MERGE
template<class T>
void CodingStructure::addCCPToLut(static_vector<T, MAX_NUM_HCCP_CANDS> &lut, const T &model, int reusePos)
{
  int currCnt = (int) lut.size();

  int erasePos = 0;

  if (reusePos == -1)
  {
    for (int j = 0; j < currCnt; j++)
    {
      if (lut[currCnt - j - 1] == model)
      {
        reusePos = j;
        break;
      }
    }
  }
  if (reusePos != -1)
  {
    erasePos = currCnt - 1 - reusePos;   // reverse the order
  }
  if (reusePos != -1 || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + erasePos);
  }
  lut.push_back(model);
}

template<class T>
void CodingStructure::getOneModelFromCCPLut(const static_vector<T, MAX_NUM_HCCP_CANDS> &lut, T &model, int pos)
{
  size_t currCnt = lut.size();
  CHECK(pos >= currCnt, "Invalid entry in CCP LUT");
  model = lut[currCnt - pos - 1];
}

#endif

#if JVET_AG0058_EIP
template<class T>
void CodingStructure::addEipToLut(static_vector<T, MAX_NUM_HEIP_CANDS> &lut, const T &model, int reusePos)
{
  int currCnt = (int) lut.size();

  int erasePos = 0;

  if (reusePos == -1)
  {
    for (int j = 0; j < currCnt; j++)
    {
      if (lut[currCnt - j - 1] == model)
      {
        reusePos = j;
        break;
      }
    }
  }
  if (reusePos != -1)
  {
    erasePos = currCnt - 1 - reusePos;   // reverse the order
  }
  if (reusePos != -1 || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + erasePos);
  }
  lut.push_back(model);
}

template<class T>
void CodingStructure::getOneModelFromEipLut(const static_vector<T, MAX_NUM_HEIP_CANDS> &lut, T &model, int pos)
{
  size_t currCnt = lut.size();
  CHECK(pos >= currCnt, "Invalid entry in EIP LUT");
  model = lut[currCnt - pos - 1];
}

#endif
#endif

