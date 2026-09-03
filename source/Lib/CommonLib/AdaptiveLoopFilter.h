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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

#if ALF_IMPROVEMENT
typedef       short           AlfClassifier;         
#else
struct AlfClassifier
{
  AlfClassifier() {}
  AlfClassifier( uint8_t cIdx, uint8_t tIdx )
    : classIdx( cIdx ), transposeIdx( tIdx )
  {
  }
  uint8_t classIdx;
  uint8_t transposeIdx;
};
#endif

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  VARIANCE,
#endif
  NUM_DIRECTIONS
};

class AdaptiveLoopFilter
{
public:
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  static inline Pel clipALF(const Pel clip, const Pel ref, const Pel val0, const Pel val1)
  {
    return Clip3<Pel>(-clip, +clip, val0-ref) + Clip3<Pel>(-clip, +clip, val1-ref);
  }
#else
  static inline int clipALF(const int clip, const short ref, const short val0, const short val1)
  {
    return Clip3<int>(-clip, +clip, val0-ref) + Clip3<int>(-clip, +clip, val1-ref);
  }
#endif
#if ALF_IMPROVEMENT
  static inline int clipALF( const int clip, const short ref, const short val )
  {
    return Clip3<int>( -clip, +clip, val - ref );
  }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER || JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  static inline int numFixedFilters( AlfFilterType filterType )
  {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
    return ( filterType >= ALF_FILTER_9 ) ? 2 : 1 ;
#else
    return ( filterType >= ALF_FILTER_9_EXT ) ? 2 : 1 ;
#endif
  }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  static bool isCoeffRestricted(short coeff, bool luma);
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
  void mirroredPaddingForAlf(CodingStructure& cs, const PelUnitBuf& src, int paddingSize, bool enableLuma, bool enableChroma);
#endif
#if NN_LF_UNIFIED
  void copyPreNNVCDataToTemp(CodingStructure &cs) { m_tempBufPreNNVC.copyFrom(cs.getRecoBuf()); } // copy Reco buffer into alf.tempBuffPreNN
  void copyAfterNNVCDataToTemp(CodingStructure &cs) { m_tempBufAfterNNVC.copyFrom(cs.getRecoBuf()); }// copy Reco buffer into alf.tempBuffAfterNN
  void copyPreNNVCDataFromTemp(CodingStructure& cs) {cs.getRecoBuf().copyFrom(m_tempBufPreNNVC); } // copy alf.tempBuffPreNN to Reco
  void copyNNVCData(Picture &pic)
  { 
    PelUnitBuf recBuf = pic.getRecoBuf();
    m_tempBufBeforeDb.copyFrom(recBuf); 
  }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  void copyDbData( CodingStructure& cs ) { m_tempBufBeforeDb.copyFrom( cs.getRecoBuf() ); }
#else
  void copyDbData( CodingStructure& cs ) { m_tempBufBeforeDb.bufs[COMPONENT_Y].copyFrom( cs.getRecoBuf().bufs[COMPONENT_Y] ); }
#endif
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  void copyResiData(CodingStructure &cs) { m_tempBufResi.bufs[COMPONENT_Y].copyFrom(cs.getResiBuf().bufs[COMPONENT_Y]); }
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  PelUnitBuf callCodingInfoBuf( CodingStructure &cs ) { return m_tempBufCodingInfo; }
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  PelUnitBuf callRecAfterSaoBuf() { return m_tempBufSAO; }
  PelUnitBuf callRecBeforeAlfBuf() { return m_tempBuf; }
  PelUnitBuf callRecBeforeDbfBuf(){ return  m_tempBufBeforeDb; }
#endif

  static constexpr int AlfNumClippingValues[MAX_NUM_CHANNEL_TYPE] = { 4, 4 };
  static constexpr int MaxAlfNumClippingValues = 4;

#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  static constexpr int   m_NUM_BITS_CHROMA = 8;
#else
  static constexpr int   m_NUM_BITS = 8;
#endif
#if ALF_IMPROVEMENT
  static constexpr int   m_NUM_BITS_FIXED_FILTER = 12;
  static constexpr int   m_CLASSIFICATION_BLK_SIZE = 256;  //non-normative, local buffer size
#else
  static constexpr int   m_CLASSIFICATION_BLK_SIZE = 32;  //non-normative, local buffer size
#endif
  static constexpr int m_ALF_UNUSED_CLASSIDX = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX = 255;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
  static constexpr int m_SCALE_BITS_NUM = 4;
  static constexpr int m_SCALE_FACTOR[1 << m_SCALE_BITS_NUM] = {16, 17, 18, 19, 21, 23, 25, 27, 29, 15, 14, 13, 12, 11, 10, 9,};
  static constexpr int m_SCALE_SHIFT = 4;
#endif

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}
  void reconstructCoeffAPSs(CodingStructure& cs, bool luma, bool chroma, bool isRdo);
  void reconstructCoeff(AlfParam& alfParam, ChannelType channel, const bool isRdo, const bool isRedo = false);
  void ALFProcess(CodingStructure& cs);
#if FIXFILTER_CFG 
  void create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], bool useFixedFilter = true);
#else
  void create( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] );
#endif
  void destroy();
#if JVET_AK0065_TALF
  void TAlfProcess(CodingStructure &cs);
#endif
#if RPR_ENABLE
  Size  getAlfSize() { return Size(m_tempBuf.get(COMPONENT_Y).width, m_tempBuf.get(COMPONENT_Y).height); }
#endif

#if ALF_IMPROVEMENT
  void  copyFixedFilterResults(const PelUnitBuf &recDst, const Area &blkDst, ComponentID COMPONENT_Y, Pel*** fixedFilterResults, const int fixedFilterSetIdx, const int classifierIdx);
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  static void fixedFiltering(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &cu,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                             const Area &blkDst,
#endif
                             Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd,
                             const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                             const ClpRng &clpRng, const Pel clippingValues[4]);
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  static void fixedFilteringResi(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &cu,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                 const Area &blkDst,
#endif
                                 Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd,
                                 const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                                 const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
  );
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  void paddingFixedFilterResultsPic(Pel*** fixedFilterResultsPic[3], const int fixedFilterSetIdx, ComponentID compID);
#else
  void paddingFixedFilterResultsPic(Pel*** fixedFilterResultsPic, const int fixedFilterSetIdx);
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  void paddingFixedFilterResultsCtu( Pel*** fixedFilterResultsPic, Pel*** fixedFilterResultsCtu, const int fixedFilterSetIdx, const Area &blk, int classifierIdx );
  void deriveFixedFilterResultsBlk( AlfClassifier*** classifier, const CPelBuf& srcLuma, const CPelBuf& srcLumaBeforeDb, const Area& blkDst, const Area& blk, const int bits, CodingStructure &cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int winIdx, int fixedFilterSetIdx );
  void deriveFixedFilterResults( AlfClassifier*** classifier, const CPelBuf& srcLuma, const CPelBuf& srcLumaBeforeDb, const Area& blkDst, const Area& blk, CodingStructure &cs, int winIdx, int fixedFilterSetIdx );
  static void calcClass0Var( AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS] );
#if JVET_AJ0237_INTERNAL_12BIT
  static void deriveVariance(const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, uint32_t ***laplacian, int bd);
#else
  static void deriveVariance( const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t ***laplacian );
#endif
  void deriveFixedFilterResultsCtuBoundary( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkDst, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx, int classifierIdx
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , const CPelBuf& srcCodingInfo, const CPelBuf& srcResi
#endif
    );
  void deriveFixedFilterResultsPerBlk( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkCur, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx );
#if JVET_AJ0237_INTERNAL_12BIT
  void(*m_deriveVariance)(const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, uint32_t ***variance, int bd);
#else
  void(*m_deriveVariance)(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t ***variance);
#endif
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  void deriveFixedFilterResultsCtuBoundaryChroma(AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkDst, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], uint8_t* ctuEnableFlag, int ctuIdx);
  void deriveFixedFilterResultsPerBlkChroma(AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
  void deriveFixFilterResultsBlkChroma( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS] );
  void deriveFixedFilterChroma(AlfClassifier*** classifier, const PelUnitBuf& src, const PelUnitBuf& srcBeforeDb, const Area& blkDst, const Area& blk, CodingStructure &cs, const int classifierIdx, ComponentID compID);
  void alfFixedFilterBlkNonSimd(AlfClassifier **classifier, const CPelBuf &src, const Area &curBlk, const Area &blkDst, const CPelBuf &srcBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4], bool isLuma
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , CodingStructure &cs
#endif
    );
  void alfFixedFilterBlk(AlfClassifier **classifier, const CPelBuf &src, const Area &curBlk, const Area &blkDst, const CPelBuf &srcBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4], bool isLuma
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , CodingStructure &cs
#endif
    );
  template<AlfFixedFilterType filtType>
#else
  void alfFixedFilterBlkNonSimd(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]);
  void alfFixedFilterBlk(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]);
  template<AlfFixedFilterType filtType>
#endif
  static void fixedFilterBlk(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
    );
  void(*m_fixFilter13x13Db9Blk)(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
    );
  void(*m_fixFilter9x9Db9Blk)(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
    );
#else
  void paddingFixedFilterResultsCtu(Pel*** fixedFilterResultsPic, Pel*** fixedFilterResultsCtu, const int fixedFilterSetIdx, const Area &blk);
  void deriveFixedFilterResultsCtuBoundary(AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx);
  void deriveFixedFilterResultsPerBlk(AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkCur, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx );
#endif
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  void paddingGaussResultsPic(Pel*** gaussPic, const int storeIdx);
  void paddingGaussResultsCtu(Pel*** gaussPic, Pel*** gaussCtu, const int storeIdx, const Area &blkDst);
  void deriveGaussResultsCtuBoundary(Pel*** gaussPic, const CPelBuf &srcLuma, const Area &blkDst, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx, const int filterSetIdx, const int storeIdx 
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION && FIXFILTER_CFG
    , const CPelBuf& srcCodingInfo, const CPelBuf& srcResi, const bool useFixedFilter
#endif
  );
  void deriveGaussResultsBlk( Pel*** gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, const int storeIdx);
  void deriveGaussResults(const CPelBuf& srcLumaDb, const Area& blkDst, const Area& blk, CodingStructure &cs, const int filterSetIdx, const int storeIdx );

  static void gaussFiltering(CodingStructure &cs, Pel ***gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, AlfClassifier** classifierCodingInfo
#endif
    );
  void(*m_gaussFiltering)   (CodingStructure &cs, Pel ***gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, AlfClassifier** classifierCodingInfo
#endif
    );
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  static void textureClassMapping(AlfClassifier **classifier, const Area& blk, int classifierIdx, int subBlkSize, AlfClassifier **classifierCodingInfo);
  void( *m_textureClassMapping ) (AlfClassifier **classifier, const Area& blk, int classifierIdx, int subBlkSize, AlfClassifier **classifierCodingInfo);

  static void calcAlfLumaCodingInfoBlk( CodingStructure& cs, AlfClassifier** classifier, const Area &blkDst, const Area &blkSrc, const CPelBuf& srcLuma, int subBlkSize, int classifierIdx, int bitDepth, const CPelBuf& srcLumaResi, uint32_t **buffer, const CPelBuf& srcCodingInfo );
  void(  *m_calcAlfLumaCodingInfoBlk )( CodingStructure& cs, AlfClassifier** classifier, const Area &blkDst, const Area &blkSrc, const CPelBuf& srcLuma, int subBlkSize, int classifierIdx, int bitDepth, const CPelBuf& srcLumaResi, uint32_t **buffer, const CPelBuf& srcCodingInfo );
#endif
#if NNLF_ALF_POS_INTERFACE
  static void bypassGaussFiltering(CodingStructure& cs, Pel*** gaussPic, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const ClpRng& clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx);
#endif
//#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  void calcOffsetRefinement(CodingStructure& cs, PelUnitBuf& src0, PelUnitBuf& src1, PelUnitBuf& dst, int stageIdx, int refineIdx);
  void copyOffsetRefinement(CodingStructure& cs, PelUnitBuf& src, PelUnitBuf& dst );
  void copyOffsetRefinementBlk(CodingStructure& cs, PelUnitBuf& src, PelUnitBuf& dst, const Area& blk );

  static void calcOffsetRefinementBlk(CodingStructure& cs, PelUnitBuf& src0, PelUnitBuf& src1, PelUnitBuf& dst, PelUnitBuf& srcCodingInfo, int stageIdx, const Area& blk, int refineIdx, PelUnitBuf& codingInfo);
  void ( *m_calcOffsetRefinementBlk )(CodingStructure& cs, PelUnitBuf& src0, PelUnitBuf& src1, PelUnitBuf& dst, PelUnitBuf& srcCodingInfo, int stageIdx, const Area& blk, int refineIdx, PelUnitBuf& codingInfo);
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  void paddingLaplacianResultsPic(Pel ***laplacianPic, const int storeIdx);
  void paddingLaplacianResultsCtu(Pel ***laplacianPic, Pel ***laplacianCtu, const int storeIdx, const Area &blkDst);
  void deriveLaplacianResultsCtuBoundary(Pel*** laplacianPic, const CPelBuf &srcLuma, const Area &blkDst, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx, const int filterSetIdx, const int storeIdx, const CPelBuf &srcCodingInfo);
  void deriveLaplacianResultsBlk( Pel*** laplacianPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, const int storeIdx, const CPelBuf &srcCodingInfo );
  void deriveLaplacianResults(const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, CodingStructure &cs, const int filterSetIdx, const int storeIdx, const CPelBuf &srcCodingInfo);

  static void laplacianFiltering(CodingStructure &cs, Pel ***laplacianPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx, const CPelBuf &srcCodingInfo);
  void(*m_laplacianFiltering)   (CodingStructure &cs, Pel ***laplacianPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx, const CPelBuf &srcCodingInfo);
  static void localGaussianFiltering(CodingStructure &cs, const Pel* srcPtrGauss, int strideSrc, int i , int j, const ClpRng &clpRng, const Pel clippingValues[4], Pel gaussOutput[], int gaussOutputLoc);
#endif
  int assignAct(int avg_varPrec, int shift, int noAct);
  static void calcClass(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
  static void deriveClassificationLaplacianBig(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS]);
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  static void deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS], const int side);
#else
  static void deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS]);
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void deriveClassificationAndFixFilterResultsBlk( AlfClassifier ***classifier, Pel ***fixedFilterResults, 
#if FIXFILTER_CFG
    bool useFixedFilter,
#endif
    const CPelBuf &srcLuma, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    const bool bResiFixed, Pel ***fixedFilterResiResults, const CPelBuf &srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    const CPelBuf &srcLumaBeforeDb, const uint8_t ctuPadFlag,
#endif
    const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int qpIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx, const int multipleClassifierIdx );
  static void calcClassNew( AlfClassifier **classifier, const Area &blkDst, const Area &cu, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth
#if JVET_AD0222_ALF_RESI_CLASS
    , const CPelBuf& srcResiLuma, uint32_t **buffer
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
   , AlfClassifier **classifierCodingInfo
#endif
  );
#else
  void deriveClassificationAndFixFilterResultsBlk( AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int qpIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx );
#endif
  template<AlfFilterType filtTypeCcAlf>
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
#if JVET_AH0057_CCALF_COEFF_PRECISION
  static void filterBlkCcAlf(const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc, const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs, CodingStructure& cs, const CPelUnitBuf& resiSrc, const Pel clippingValues[4], const int coeffPrec 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSAOSrc
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    , const CPelUnitBuf& recBeforeDbf
#endif
  );
#else
  static void filterBlkCcAlf(const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc, const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs, CodingStructure& cs, const CPelUnitBuf& resiSrc, const Pel clippingValues[4] 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSAOSrc
#endif
  );
#endif
#else
#if JVET_AH0057_CCALF_COEFF_PRECISION
  static void filterBlkCcAlf(const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc, const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs, CodingStructure& cs, const int coeffPrec
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSAOSrc
#endif
  );
#else
  static void filterBlkCcAlf(const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc, const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs, CodingStructure& cs
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSAOSrc
#endif
  );
#endif
#endif
#else
  static void deriveClassificationBlk(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
    template<AlfFilterType filtTypeCcAlf>
  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos);
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void deriveClassification( AlfClassifier*** classifier, const CPelBuf& srcLuma, 
#if FIXFILTER_CFG
                            bool useFixedFilter,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                            const bool bResiFixed, const CPelBuf &srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
                            const CPelBuf& srcLumaBeforeDb, const uint8_t ctuPadFlag,
#endif
    const Area& blkDst, const Area& blk, CodingStructure &cs, const int classifierIdx, const int multipleClassifierIdx );
#else
  void deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk
#if ALF_IMPROVEMENT
  , CodingStructure &cs, const int classifierIdx
#endif
  );
#endif

#if ALF_IMPROVEMENT
  void(*m_calcClass0)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
  void(*m_calcClass1)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AD0222_ALF_RESI_CLASS
  void(*m_calcClass2)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth, const CPelBuf& srcLumaResi, uint32_t **buffer
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , AlfClassifier **classifierCodingInfo
#endif
    );
#else
  void(*m_calcClass2)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth);
#endif
#endif
  void(*m_deriveClassificationLaplacianBig)(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS]);
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  void(*m_deriveClassificationLaplacian)(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS], const int side);
#else
  void(*m_deriveClassificationLaplacian)(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS]);
#endif
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  void (*m_filter13x13Blk)(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           const Area &blkDst,
#endif
                           Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd,
                           const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                           const ClpRng &clpRng, const Pel clippingValues[4]);
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  void(*m_filterResi9x9Blk)(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    const Area &blkDst,
#endif
    Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
    , bool applyCodingInfo, CodingStructure &cs, AlfClassifier** classifierCodingInfo
#endif
    );
#else
  void (*m_filterResi13x13Blk)(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                               const Area &blkDst,
#endif
                               Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4]);
#endif
#endif
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
#if JVET_AH0057_CCALF_COEFF_PRECISION
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, const CPelUnitBuf &resiSrc, const Pel clippingValues[4], const int coeffPrec 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSrcSAO
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    , const CPelUnitBuf& recBeforeDbf
#endif
    );
#else
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, const CPelUnitBuf &resiSrc, const Pel clippingValues[4] 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSrcSAO
#endif
    );
#endif
#else
#if JVET_AH0057_CCALF_COEFF_PRECISION
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, const int coeffPrec
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSrcSAO
#endif
    );
#else
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    , const CPelUnitBuf& recSrcSAO
#endif
    );
#endif
#endif
  template<AlfFilterType filtType>
  static void filterBlk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    const PelUnitBuf &resi,
#endif
    const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#else
    const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#endif
                        , Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                        Pel ***fixedFilterResiResults,
#endif
    int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
    , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
    , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
   );
#else
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos);
  template<AlfFilterType filtType>
  static void filterBlk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT                        
                        const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
#else
                        const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
#endif
                        int vbPos);
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if ALF_IMPROVEMENT
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  void (*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                         , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                         , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                         , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void (*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                         , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                         , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                         , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                   , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                   , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  void    alfAddCorrect( AlfClassifier** classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blk, const ComponentID compId, char coeffBits, int* idxCorr );
  void    alfAddCorrectChroma( CodingStructure& cs, PelUnitBuf& tmpYuvRecSAO );
  void    setAlfScaleMode( const int mode );
  int     getScaleCorrInt   ( const int s );
  double  getScaleCorrDouble( const int s );
  void    storeAlfScalePrev( CodingStructure& cs, Slice& slice );
  void    backupAlfScalePrev( std::vector<int> (&alfScalePrevBckup)[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA] ) const;
  void    restoreAlfScalePrev( const std::vector<int> (&alfScalePrevBckup)[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA] );
  void    getAlfScalePrev( CodingStructure& cs, Slice& slice );
  std::vector<int>&  getAlfScalePrev( const int apsIdx, const int alt ) { return m_idxCorrPrev[apsIdx][alt]; }
  void    resetAlfScalePrev( Slice& slice );
#endif
  void (*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                         , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                         , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                         , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void (*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                         , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                         , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                         , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void (*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                         , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                         , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                         , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void (*m_filter9x9BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                        , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                        , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                        , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
#if JVET_AA0095_ALF_LONGER_FILTER
  void (*m_filter13x13BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet,  const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                       , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                       , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                       , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
  void (*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                       , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                       , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                       , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
#endif
  void (*m_filter13x13BlkExtDbResiDirect)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                       , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                       , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                       , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
#if FIXFILTER_CFG
  void (*m_filter13x13BlkDbResiDirect)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
    , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
    );
  void (*m_filter13x13BlkDbResi)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
    , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
    );
  void (*m_filter9x9BlkNoFix)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
    , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                                 , Pel ***laplacianPic, Pel ***laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
    );
#endif
  void (*m_filter13x13BlkExtDbResi)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const PelUnitBuf &resi, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, Pel ***fixedFilterResiResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                       , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                       , Pel*** gaussPic, Pel*** gaussCtu
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                       , Pel*** laplacianPic, Pel*** laplacianCtu
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    , char coeffBits
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    , const char* scaleIdxSet
#endif
  );
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#endif
#else
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#endif
#endif
#endif
#else
  void (*m_deriveClassificationBlk)(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
  void (*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
  void (*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
#endif
#else
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#endif
#else
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
    );
#endif
#endif
#else
  void (*m_deriveClassificationBlk)(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
  void (*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
  void (*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
#endif
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION  
  void applyCcAlfFilter(CodingStructure &cs, ComponentID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt, uint8_t *filterControl, const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const int   selectedFilterIdx, const int coeffPrec);
#else
  void applyCcAlfFilter(CodingStructure &cs, ComponentID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt, uint8_t *filterControl, const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const int   selectedFilterIdx);
#endif
  CcAlfFilterParam &getCcAlfFilterParam() { return m_ccAlfFilterParam; }
  uint8_t* getCcAlfControlIdc(const ComponentID compID)   { return m_ccAlfFilterControl[compID-1]; }

# if JVET_AK0065_TALF
  TAlfCtbParam*        m_tAlfCtbControl;
  std::vector<refComb> m_refCombs;
  TAlfCtbParam*        getTAlfControl () { return m_tAlfCtbControl; }
  void getRefPics(const CodingStructure &cs);
  bool getMotionOffset(const CodingStructure &cs, const Position pos, MvField* mvField, const int mode, const int shapeIdx);
  static void setBiInput(Pel input[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const CodingStructure& cs
    , const ComponentID compId, const CPelBuf& recBuf, const Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF]
    , const Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF], const Position curPos, const int shapeIdx, const int picWidth
    , const int picHeight, const int mode, std::vector<refComb>& refCombs, MvField* mvField, const int numOfClips);
  static void setUniInput(Pel input[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const CodingStructure& cs
    , const ComponentID compId, const CPelBuf& recBuf, const Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF]
    , const Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF], const Position curPos, const int shapeIdx, const int picWidth
    , const int picHeight, const int mode, std::vector<refComb>& refCombs, MvField* mvField, const int numOfClips);
  static void filterBatchTAlf(Pel inputBatch[NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const int *filterCoeff
    , const Position pos, PelBuf &dstBuf, PelBuf &recBuf, const int numCoeff, const int offset, const int shift
    , const ClpRng& clpRng);
  static int groupSumTAlf(Pel* a, Pel* b);
  void filterBlkTAlf(CodingStructure &cs, const ComponentID compId, PelBuf &recBuf0, PelBuf &recBuf1, const UnitArea& ctu
    , TAlfCtbParam& ctbControl, std::vector<TAlfFilterParam>& params, const TAlfControl talfControl);
  void applyTAlfFilter(CodingStructure &cs, const ComponentID compId, PelUnitBuf &recAfterALF, PelUnitBuf &recBeforeALF
    , std::vector<TAlfFilterParam>& params, TAlfCtbParam* tAlfControl, const TAlfControl talfControl);
  void (*m_setTAlfInput[2])(Pel input[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const CodingStructure& cs
    , const ComponentID compId, const CPelBuf& recBuf, const Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF]
    , const Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF], const Position curPos, const int shapeIdx, const int picWidth
    , const int picHeight, const int mode, std::vector<refComb>& refCombs, MvField* mvField, const int numOfClips);
  int  (*m_groupSumTAlf)(Pel *a, Pel *b);
  void (*m_filterBatchTAlf)(Pel inputBatch[NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE], const int *filterCoeff
    , const Position pos, PelBuf &dstBuf, PelBuf &recBuf, const int numCoeff, const int offset, const int shift
    , const ClpRng& clpRng);
#endif
#ifdef TARGET_SIMD_X86  
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

protected:
  bool isCrossedByVirtualBoundaries( const CodingStructure& cs, const int xPos, const int yPos, const int width, const int height, bool& clipTop, bool& clipBottom, bool& clipLeft, bool& clipRight, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], int& rasterSliceAlfPad );
#if !JVET_AH0057_CCALF_COEFF_PRECISION
  static constexpr int   m_scaleBits = 7; // 8-bits
#endif
  CcAlfFilterParam       m_ccAlfFilterParam;
  uint8_t*               m_ccAlfFilterControl[2];
#if !ALF_IMPROVEMENT
  static const int             m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  static const int             m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];
  short                        m_fixedFilterSetCoeffDec[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_clipDefault[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
  bool                         m_created = false;
  short                        m_chromaCoeffFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char                         m_chromaScaleIdxFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][1];
#endif
  AlfParam*                    m_alfParamChroma;
  Pel                          m_alfClippingValues[MAX_NUM_CHANNEL_TYPE][MaxAlfNumClippingValues];
  std::vector<AlfFilterShape>  m_filterShapesCcAlf;
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CHANNEL_TYPE];
#if ALF_IMPROVEMENT
  AlfFilterType                m_filterTypeApsLuma[ALF_CTB_MAX_NUM_APS];
  AlfFilterType                m_filterTypeApsChroma;
  static const int             alfNumCoeff[ALF_NUM_OF_FILTER_TYPES];
  bool                         m_filterTypeTest[MAX_NUM_CHANNEL_TYPE][ALF_NUM_OF_FILTER_TYPES];
  int                          m_filterTypeToStatIndex[MAX_NUM_CHANNEL_TYPE][ALF_NUM_OF_FILTER_TYPES];
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  AlfClassifier**              m_classifier[ALF_NUM_CLASSIFIER + 2];
#else
  AlfClassifier**              m_classifier[ALF_NUM_CLASSIFIER + 1];
#endif
#else
  AlfClassifier**              m_classifier[ALF_NUM_CLASSIFIER];
#endif
  char                         m_classifierIdxApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
  char                         m_classifierFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  char                         m_coeffBitsApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
  char                         m_coeffBitsFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif
#else
  AlfClassifier**              m_classifier;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  AlfClassifier**              m_classifierCodingInfo[1];
#endif
#if ALF_IMPROVEMENT
  int                          m_numLumaAltAps[ALF_CTB_MAX_NUM_APS];
  short                        m_coeffApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#else  
  short                        m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#endif
  short                        m_coeffFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char                         m_scaleIdxApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES];
  char                         m_scaleIdxFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#else
  short                        m_clippFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
#else
  short                        m_coeffApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#else
  short                        m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#endif
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#else
  short                        m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
#endif
  short                        m_chromaClippFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];
#if ALF_IMPROVEMENT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  Pel***                       m_fixFilterResult[3];
#else
  Pel***                       m_fixFilterResult;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel***                       m_fixFilterResiResult;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  Pel***                       m_fixedFilterResultPerCtu;
  bool                         m_isFixedFilterPaddedPerCtu;
  uint8_t*                     m_ctuEnableOnlineLumaFlag;
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  uint8_t*                     m_ctuPadFlag;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  Pel***                       m_gaussPic;
  Pel***                       m_gaussCtu;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  Pel***                       m_laplacianPic;
  Pel***                       m_laplacianCtu;
#endif
  const int                    usedWindowIdx[NUM_CLASSIFIER] = { 1, 5 };
  int                          m_mappingDir[NUM_DIR_FIX][NUM_DIR_FIX];
  uint32_t**                   m_laplacian[NUM_DIRECTIONS];
  uint32_t *                   m_laplacianPtr[NUM_DIRECTIONS][(m_CLASSIFICATION_BLK_SIZE + 10) >> 1];
  uint32_t                     m_laplacianData[NUM_DIRECTIONS][(m_CLASSIFICATION_BLK_SIZE + 10)>>1][((m_CLASSIFICATION_BLK_SIZE + 16)>>1) + 8];
#else
  int**                        m_laplacian[NUM_DIRECTIONS];
  int *                        m_laplacianPtr[NUM_DIRECTIONS][m_CLASSIFICATION_BLK_SIZE + 5];
  int                          m_laplacianData[NUM_DIRECTIONS][m_CLASSIFICATION_BLK_SIZE + 5][m_CLASSIFICATION_BLK_SIZE + 5];
#endif
  uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t*                     m_ctuAlternative[MAX_NUM_COMPONENT];
  PelStorage                   m_tempBuf;
  PelStorage                   m_tempBuf2;
#if NN_LF_UNIFIED
  PelStorage                   m_tempBufAfterNNVC;
  PelStorage                   m_tempBufPreNNVC;
#endif
  
#if JVET_AI0084_ALF_RESIDUALS_SCALING
  PelStorage                   m_tempBuf3;
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  PelStorage                   m_tempBufBeforeDb;
  PelStorage                   m_tempBufBeforeDb2;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  PelStorage                   m_tempBufResi;
  PelStorage                   m_tempBufResi2;
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  PelStorage                   m_tempBufSAO;
  PelStorage                   m_tempBufSAO2;
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  PelStorage                   m_tempBufCodingInfo;
  PelStorage                   m_tempBufCodingInfo2;
#endif
  int                          m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
  int                          m_picWidth;
  int                          m_picHeight;
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  int                          m_picWidthChroma;
  int                          m_picHeightChroma;
#endif
  int                          m_maxCUWidth;
  int                          m_maxCUHeight;
  int                          m_maxCUDepth;
  int                          m_numCTUsInWidth;
  int                          m_numCTUsInHeight;
  int                          m_numCTUsInPic;
#if !ALF_IMPROVEMENT
  int                          m_alfVBLumaPos;
  int                          m_alfVBChmaPos;
  int                          m_alfVBLumaCTUHeight;
  int                          m_alfVBChmaCTUHeight;
#endif
  ChromaFormat                 m_chromaFormat;
  ClpRngs                      m_clpRngs;

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  int                          m_nbCorr;
  std::vector<int>             m_scaleCorr;
  int                          m_nbCorrChroma;
  std::vector<int>             m_scaleCorrChroma;
  std::vector<int>             m_idxCorrPrev[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif

#if ENABLE_SIMD_OPT_ALF_CHOLESKY
  public:
#if JVET_AF0177_ALF_COV_FLOAT
  using cholesky_float_t = float;
#else
  using cholesky_float_t = double;
#endif
  using cholesky_matrix = cholesky_float_t[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  static constexpr cholesky_float_t cholesky_reg_sqr = cholesky_float_t(0.0000001);
  static int(*m_fastCholeskyDec)(cholesky_matrix inpMatr, cholesky_matrix outMatr, int numEq);
  static int fastCholeskyDec(cholesky_matrix inpMatr, cholesky_matrix outMatr, int numEq);
#endif
};

#if JVET_AK0123_ALF_COEFF_RESTRICTION

class ALFCoeffRestriction
{
public:
  struct InputParam
  {
    int8_t bitWidth;
    int8_t mantissa;

    InputParam(int8_t bitWidth, int8_t mantissa) : bitWidth(bitWidth), mantissa(mantissa)
    {
    }

    bool operator<(const InputParam& right) const
    {
      if (bitWidth != right.bitWidth)
      {
        return bitWidth < right.bitWidth;
      }
      return mantissa < right.mantissa;
    }
  };

  struct Param
  {
    int16_t minValue, maxValue;
    std::vector<int16_t> idxToCoeff;
    std::vector<int16_t> coeffToIdx;
  };

private:
  static std::map<InputParam, Param> m_cache;

  InputParam m_inputParam;
  Param* m_param;

  void build();
public:
  ALFCoeffRestriction(int bitWidth, int mantissa);
  void init();

  const InputParam& getInputParam() { return m_inputParam; }
  const Param& getParam() { return *m_param; }
};

class HuffmanForALF
{
public:
  struct InputParam
  {
    bool isLuma;
    int8_t bitWidth;
    int8_t mantissa;
    int8_t coeffGroup;

    InputParam(bool isLuma, int8_t bitWidth, int8_t mantissa, int8_t coeffGroup) : isLuma(isLuma), bitWidth(bitWidth), mantissa(mantissa), coeffGroup(coeffGroup)
    {
    }

    bool operator<(const InputParam& right) const
    {
      if (isLuma != right.isLuma)
      {
        return isLuma < right.isLuma;
      }
      if (bitWidth != right.bitWidth)
      {
        return bitWidth < right.bitWidth;
      }
      if (mantissa != right.mantissa)
      {
        return mantissa < right.mantissa;
      }
      return coeffGroup < right.coeffGroup;
    }
  };

  struct Node
  {
    int16_t coeffIdx;
    Node* zero;
    Node* one;
  };

  struct Param
  {
    Node* root;
    std::vector<std::pair<uint32_t, int8_t>> codeTableAndLength;

    Param() : root(nullptr)
    {
    }

    void dfsDelete(Node* node)
    {
      if (node)
      {
        dfsDelete(node->zero);
        dfsDelete(node->one);
        delete node;
      }
    }

    ~Param()
    {
      dfsDelete(root);
    }
  };

private:
  static std::map<InputParam, std::vector<int>> m_stats;
  static std::map<InputParam, Param> m_cache;

  InputParam m_inputParam;
  ALFCoeffRestriction m_coeff;
  Param* m_param;

  void build();
  void buildHuffmanTree();
  void buildCodeTable();
  void buildCodeTable(Node* node, uint32_t code, int level);
public:
  HuffmanForALF(bool isLuma, int bitWidth, int mantissa, int coeffGroup);
  void init();

  const InputParam& getInputParam() { return m_inputParam; }
  const Param& getParam() { return *m_param; }

  void setGroup(int coeffGroup);

  bool encodeCoeff(int16_t coeff, uint32_t& symbol, int& length);
  bool decodeBit(int8_t bit, Node*& node);
  int16_t getCoeff(Node* node);
};

#endif

#endif
