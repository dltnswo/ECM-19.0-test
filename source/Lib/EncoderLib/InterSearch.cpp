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

/** \file     EncSearch.cpp
 *  \brief    encoder inter search class
 */

#include "InterSearch.h"


#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif
#include "CommonLib/MCTS.h"

#include "EncModeCtrl.h"
#include "EncLib.h"

#include <math.h>
#include <limits>


 //! \ingroup EncoderLib
 //! \{

static const Mv s_acMvRefineH[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

static const Mv s_acMvRefineQ[9] =
{
  Mv(  0,  0 ), // 0
  Mv(  0, -1 ), // 1
  Mv(  0,  1 ), // 2
  Mv( -1, -1 ), // 5
  Mv(  1, -1 ), // 6
  Mv( -1,  0 ), // 3
  Mv(  1,  0 ), // 4
  Mv( -1,  1 ), // 7
  Mv(  1,  1 )  // 8
};

#if JVET_Z0131_IBC_BVD_BINARIZATION
void InterSearch::xEstBvdBitCosts(EstBvdBitsStruct *p
#if JVET_AE0169_BIPREDICTIVE_IBC
                                , bool bi
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                , unsigned useIBCFrac
#if JVET_AA0070_RRIBC
                                , int ctxIdRrIBC
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                , int ctxIdOneComp
#endif
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                , const bool useBvpCluster
#endif
)
{
  const FracBitsAccess& fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();

  p->bitsGt0FlagH[0] = fracBits.getFracBitsArray(Ctx::Bvd(HOR_BVD_CTX_OFFSET)).intBits[0];
  p->bitsGt0FlagH[1] = fracBits.getFracBitsArray(Ctx::Bvd(HOR_BVD_CTX_OFFSET)).intBits[1];;

  p->bitsGt0FlagV[0] = fracBits.getFracBitsArray(Ctx::Bvd(VER_BVD_CTX_OFFSET)).intBits[0];
  p->bitsGt0FlagV[1] = fracBits.getFracBitsArray(Ctx::Bvd(VER_BVD_CTX_OFFSET)).intBits[1];

  const int epBitCost = 1 << SCALE_BITS;
  const int horCtxThre = NUM_HOR_BVD_CTX;
  const int verCtxThre = NUM_VER_BVD_CTX;

  const int horCtxOs = HOR_BVD_CTX_OFFSET;
  const int verCtxOs = VER_BVD_CTX_OFFSET;

  uint32_t singleBitH[2];
  uint32_t singleBitV[2];
  int bitsX = 0, bitsY = 0;

  for (int i = 0; i < BVD_IBC_MAX_PREFIX; i++)
  {
    if (i < horCtxThre)
    {
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray(Ctx::Bvd(horCtxOs + i + 1));
      singleBitH[0] = fracBitsPar.intBits[0];
      singleBitH[1] = fracBitsPar.intBits[1];
    }
    else
    {
      singleBitH[0] = epBitCost;
      singleBitH[1] = epBitCost;
    }
    p->bitsH[i] = bitsX + singleBitH[0] + (i+BVD_CODING_GOLOMB_ORDER) * epBitCost;
    bitsX += singleBitH[1];
  }

  for (int i = 0; i < BVD_IBC_MAX_PREFIX; i++)
  {
    if (i < verCtxThre)
    {
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray(Ctx::Bvd(verCtxOs + i + 1));
      singleBitV[0] = fracBitsPar.intBits[0];
      singleBitV[1] = fracBitsPar.intBits[1];
    }
    else
    {
      singleBitV[0] = epBitCost;
      singleBitV[1] = epBitCost;
    }
    p->bitsV[i] = bitsY + singleBitV[0] + (i+BVD_CODING_GOLOMB_ORDER) * epBitCost;
    bitsY += singleBitV[1];
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const CtxSet& imvCtx = useIBCFrac ? Ctx::ImvFlagIBC : Ctx::ImvFlag;
#endif

  p->bitsIdx[0] = fracBits.getFracBitsArray(Ctx::MVPIdx()).intBits[0];
  p->bitsIdx[1] = fracBits.getFracBitsArray(Ctx::MVPIdx()).intBits[1];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  p->bitsImv[0] = fracBits.getFracBitsArray(imvCtx(1)).intBits[0];
  p->bitsImv[1] = fracBits.getFracBitsArray(imvCtx(1)).intBits[1];
#else
  p->bitsImv[0] = fracBits.getFracBitsArray(Ctx::ImvFlag(1)).intBits[0];
  p->bitsImv[1] = fracBits.getFracBitsArray(Ctx::ImvFlag(1)).intBits[1];
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  p->bitsFracImv[IMV_OFF ] = fracBits.getFracBitsArray(imvCtx(0)).intBits[0];
  p->bitsFracImv[IMV_FPEL] = fracBits.getFracBitsArray(imvCtx(0)).intBits[1]
                           + p->bitsImv[0];
  p->bitsFracImv[IMV_4PEL] = fracBits.getFracBitsArray(imvCtx(0)).intBits[1]
                           + p->bitsImv[1];
  p->bitsFracImv[IMV_HPEL] = std::numeric_limits<uint32_t>::max();
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
  if (useBvpCluster)
  {
    p->bitsRribc  = fracBits.getFracBitsArray(Ctx::rribcFlipType(0)).intBits[1];
  }
  else
  {
    p->bitsRribc = 0;
  }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  p->bitsMerge[0] = 0;
  if (bi)
  {
    memset(&p->bitsMerge[0], 0, sizeof(int32_t)*m_pcEncCfg->getMaxNumIBCMergeCand());
    for (int i = 0; i < m_pcEncCfg->getMaxNumIBCMergeCand()-1; i++)
    {
      int ctxIdx = i > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : i;
      p->bitsMerge[i] += fracBits.getFracBitsArray(Ctx::MergeIdx(ctxIdx)).intBits[0];
      for (int j = i+1; j < m_pcEncCfg->getMaxNumIBCMergeCand(); j++)
      {
        p->bitsMerge[j] += fracBits.getFracBitsArray(Ctx::MergeIdx(ctxIdx)).intBits[1];
      }
    }
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  p->bitsBvType[0] = 0;
  p->bitsBvType[1] = 0;
  p->bitsBvType[2] = 0;
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  p->bitsBvType[0] = 0;
  p->bitsBvType[1] = 0;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  if (useBvpCluster)
  {
    if (ctxIdOneComp != NOT_VALID)
    {
      p->bitsBvType[0] = fracBits.getFracBitsArray(Ctx::bvOneZeroComp(ctxIdOneComp)).intBits[0];
      p->bitsBvType[1] = fracBits.getFracBitsArray(Ctx::bvOneZeroComp(ctxIdOneComp)).intBits[1]
                       + fracBits.getFracBitsArray(Ctx::bvOneZeroComp(3           )).intBits[0];
      p->bitsBvType[2] = fracBits.getFracBitsArray(Ctx::bvOneZeroComp(ctxIdOneComp)).intBits[1]
                       + fracBits.getFracBitsArray(Ctx::bvOneZeroComp(3           )).intBits[1];
    }

#if JVET_AA0070_RRIBC
    if (ctxIdRrIBC != NOT_VALID)
    {
      p->bitsUseFlip[0] = fracBits.getFracBitsArray(Ctx::rribcFlipType(ctxIdRrIBC)).intBits[0];
      p->bitsUseFlip[1] = fracBits.getFracBitsArray(Ctx::rribcFlipType(ctxIdRrIBC)).intBits[1];
    }
#endif
  }
#if JVET_AA0070_RRIBC
  else
#endif
#endif
#if JVET_AA0070_RRIBC
  {
    if (ctxIdRrIBC != NOT_VALID)
    {
      p->bitsBvType[0] = fracBits.getFracBitsArray(Ctx::rribcFlipType(ctxIdRrIBC)).intBits[0];
      p->bitsBvType[1] = fracBits.getFracBitsArray(Ctx::rribcFlipType(ctxIdRrIBC)).intBits[1]
                       + fracBits.getFracBitsArray(Ctx::rribcFlipType(3         )).intBits[0];
      p->bitsBvType[2] = fracBits.getFracBitsArray(Ctx::rribcFlipType(ctxIdRrIBC)).intBits[1]
                       + fracBits.getFracBitsArray(Ctx::rribcFlipType(3         )).intBits[1];
    }
  }
#endif
#endif
}
#endif

InterSearch::InterSearch()
  : m_modeCtrl                    (nullptr)
  , m_pSplitCS                    (nullptr)
  , m_pFullCS                     (nullptr)
  , m_pcEncCfg                    (nullptr)
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
, m_bilateralFilter             (nullptr)
#endif
  , m_pcTrQuant                   (nullptr)
  , m_pcReshape                   (nullptr)
  , m_iSearchRange                (0)
  , m_bipredSearchRange           (0)
  , m_motionEstimationSearchMethod(MESEARCH_FULL)
  , m_CABACEstimator              (nullptr)
  , m_ctxCache                    (nullptr)
  , m_pTempPel                    (nullptr)
  , m_isInitialized               (false)
{
  for (int i=0; i<MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset (m_aaiAdaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof (int));
  }
  for (int i=0; i<AMVP_MAX_NUM_CANDS+1; i++)
  {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1+1) * sizeof (uint32_t) );
#else
    memset (m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS+1) * sizeof (uint32_t) );
#endif
  }

  setWpScalingDistParam( -1, REF_PIC_LIST_X, nullptr );
  m_affMVList = nullptr;
  m_affMVListSize = 0;
  m_affMVListIdx = 0;
  m_uniMvList = nullptr;
  m_uniMvListSize = 0;
  m_uniMvListIdx = 0;
#if INTER_LIC
  m_uniMvListLIC = nullptr;
  m_uniMvListSizeLIC = 0;
  m_uniMvListIdxLIC = 0;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  m_amvpSbTmvpBufValid = nullptr;
  m_amvpSbTmvpMotionBuf = nullptr;
  m_amvpSbTmvpBufTLPos = Position(0, 0);
#endif
  m_histBestSbt    = MAX_UCHAR;
  m_histBestMtsIdx = MAX_UCHAR;

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  m_tplWeightTblInitialized = false;
  initTplWeightTable();
#endif
}


void InterSearch::destroy()
{
  CHECK(!m_isInitialized, "Not initialized");
  if ( m_pTempPel )
  {
    delete [] m_pTempPel;
    m_pTempPel = NULL;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pSaveCS = nullptr;

  for(uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_tmpPredStorage[i].destroy();
  }
  m_tmpStorageLCU.destroy();
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_tmpStorageCUflipH.destroy();
  m_tmpStorageCUflipV.destroy();
#endif
  m_tmpAffiStorage.destroy();
#if JVET_AC0112_IBC_CIIP
  m_ibcCiipBuffer.destroy();
#endif

  if ( m_tmpAffiError != NULL )
  {
    delete[] m_tmpAffiError;
  }
  if ( m_tmpAffiDeri[0] != NULL )
  {
    delete[] m_tmpAffiDeri[0];
  }
  if ( m_tmpAffiDeri[1] != NULL )
  {
    delete[] m_tmpAffiDeri[1];
  }
  if (m_affMVList)
  {
    delete[] m_affMVList;
    m_affMVList = nullptr;
  }
  m_affMVListIdx = 0;
  m_affMVListSize = 0;
  if (m_uniMvList)
  {
    delete[] m_uniMvList;
    m_uniMvList = nullptr;
  }
  m_uniMvListIdx = 0;
  m_uniMvListSize = 0;
#if INTER_LIC
  if (m_uniMvListLIC)
  {
    delete[] m_uniMvListLIC;
    m_uniMvListLIC = nullptr;
  }
  m_uniMvListIdxLIC = 0;
  m_uniMvListSizeLIC = 0;
#endif
#if JVET_AE0059_INTER_CCCM
  for (int i = 0; i < 12; i++)
  {
    delete[] m_interCccmStorage[i];
  }
  delete[] m_interCccmStorage;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  for (int i = 0; i < 5; i++)
  {
    delete[] m_interCcpMergeStorage[i];
  }
  delete[] m_interCcpMergeStorage;
#endif
#if JVET_AL0181_ASBT
  for (int i = 0; i < 10; i++)
  {
    delete[] m_interAsbt[i];
  }
  delete[] m_interAsbt;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (m_amvpSbTmvpBufValid)
  {
    delete[] m_amvpSbTmvpBufValid;
    m_amvpSbTmvpBufValid = nullptr;
    delete[] m_amvpSbTmvpMotionBuf;
    m_amvpSbTmvpMotionBuf = nullptr;
  }
#endif
  m_isInitialized = false;
}

void InterSearch::setTempBuffers( CodingStructure ****pSplitCS, CodingStructure ****pFullCS, CodingStructure **pSaveCS )
{
  m_pSplitCS = pSplitCS;
  m_pFullCS  = pFullCS;
  m_pSaveCS  = pSaveCS;
}

#if ENABLE_SPLIT_PARALLELISM
void InterSearch::copyState( const InterSearch& other )
{
  memcpy( m_aaiAdaptSR, other.m_aaiAdaptSR, sizeof( m_aaiAdaptSR ) );
}
#endif

InterSearch::~InterSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}

void InterSearch::init( EncCfg*        pcEncCfg,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                      BilateralFilter* bilateralFilter,
#endif
                        TrQuant*       pcTrQuant,
                        int            iSearchRange,
                        int            bipredSearchRange,
                        MESearchMethod motionEstimationSearchMethod,
                        bool           useCompositeRef,
                        const uint32_t     maxCUWidth,
                        const uint32_t     maxCUHeight,
                        const uint32_t     maxTotalCUDepth,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache
                      , EncReshape*    pcReshape
#if JVET_Z0153_IBC_EXT_REF
                      , const uint32_t curPicWidthY 
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
                      , const uint32_t curPicHeightY 
#endif
#endif
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_numBVs = 0;
  for (int i = 0; i < IBC_NUM_CANDIDATES; i++)
  {
    m_defaultCachedBvs.m_bvCands[i].setZero();
  }
  m_defaultCachedBvs.currCnt = 0;
  m_pcEncCfg                     = pcEncCfg;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter              = bilateralFilter;
#endif
  m_pcTrQuant                    = pcTrQuant;
  m_iSearchRange                 = iSearchRange;
  m_bipredSearchRange            = bipredSearchRange;
  m_motionEstimationSearchMethod = motionEstimationSearchMethod;
  m_CABACEstimator               = CABACEstimator;
  m_ctxCache                     = ctxCache;
  m_useCompositeRef              = useCompositeRef;
  m_pcReshape                    = pcReshape;

  for( uint32_t iDir = 0; iDir < MAX_NUM_REF_LIST_ADAPT_SR; iDir++ )
  {
    for( uint32_t iRefIdx = 0; iRefIdx < MAX_IDX_ADAPT_SR; iRefIdx++ )
    {
      m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange;
    }
  }

  // initialize motion cost
  for( int iNum = 0; iNum < AMVP_MAX_NUM_CANDS + 1; iNum++ )
  {
    for( int iIdx = 0; iIdx < AMVP_MAX_NUM_CANDS; iIdx++ )
    {
      if( iIdx < iNum )
      {
        m_auiMVPIdxCost[iIdx][iNum] = xGetMvpIdxBits( iIdx, iNum );
      }
      else
      {
        m_auiMVPIdxCost[iIdx][iNum] = MAX_UINT;
      }
    }
  }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  m_auiMVPIdxCost[0][3] = m_auiMVPIdxCost[0][2];
  m_auiMVPIdxCost[1][3] = m_auiMVPIdxCost[1][2];
#endif

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();
#if INTER_LIC || (TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM) || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_Z0153_IBC_EXT_REF
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
#if JVET_AJ0237_INTERNAL_12BIT
  InterPrediction::init( pcRdCost, cform, maxCUHeight, m_pcReshape, curPicWidthY, curPicHeightY, pcEncCfg->getBitDepth(CHANNEL_TYPE_LUMA));
#else
  InterPrediction::init( pcRdCost, cform, maxCUHeight, m_pcReshape, curPicWidthY, curPicHeightY );
#endif
#else
#if JVET_AJ0237_INTERNAL_12BIT
  InterPrediction::init( pcRdCost, cform, maxCUHeight, m_pcReshape, curPicWidthY, pcEncCfg->getBitDepth(CHANNEL_TYPE_LUMA));
#else
  InterPrediction::init( pcRdCost, cform, maxCUHeight, m_pcReshape, curPicWidthY );
#endif
#endif
#else
  InterPrediction::init( pcRdCost, cform, maxCUHeight, m_pcReshape );
#endif
#else 
  InterPrediction::init( pcRdCost, cform, maxCUHeight );
#endif

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_tmpPredStorage[i].create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  }
  m_tmpStorageLCU.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_tmpStorageCUflipH.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
  m_tmpStorageCUflipV.create( UnitArea( cform, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) ) );
#endif
  m_tmpAffiStorage.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_tmpAffiError = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
#if AFFINE_ENC_OPT
  m_tmpAffiDeri[0] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
#else
  m_tmpAffiDeri[0] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1] = new int[MAX_CU_SIZE * MAX_CU_SIZE];
#endif
#if JVET_AC0112_IBC_CIIP
  m_ibcCiipBuffer.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#endif
  m_pTempPel = new Pel[maxCUWidth*maxCUHeight];
  m_affMVListMaxSize = (pcEncCfg->getIntraPeriod() == (uint32_t)-1) ? AFFINE_ME_LIST_SIZE_LD : AFFINE_ME_LIST_SIZE;
  if (!m_affMVList)
    m_affMVList = new AffineMVInfo[m_affMVListMaxSize];
  m_affMVListIdx = 0;
  m_affMVListSize = 0;
  m_uniMvListMaxSize = 15;
  if (!m_uniMvList)
  {
    m_uniMvList = new BlkUniMvInfo[m_uniMvListMaxSize];
  }
  m_uniMvListIdx = 0;
  m_uniMvListSize = 0;
#if INTER_LIC
  if (!m_uniMvListLIC)
  {
    m_uniMvListLIC = new BlkUniMvInfo[m_uniMvListMaxSize];
  }
  m_uniMvListIdxLIC = 0;
  m_uniMvListSizeLIC = 0;
#endif
#if JVET_AE0059_INTER_CCCM
  m_interCccmStorage = new Pel * [12];
  for (int i = 0; i < 12; i++)
  {
    m_interCccmStorage[i] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }
#endif
#if JVET_AL0181_ASBT
  m_interAsbt = new Pel * [10];
  for (int i = 0; i < 10; i++)
  {
    m_interAsbt[i] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  m_interCcpMergeStorage = new Pel * [5];
  for (int i = 0; i < 5; i++)
  {
    m_interCcpMergeStorage[i] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  m_amvpMergeBuffer = (Pel**)m_filteredBlock;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (!m_amvpSbTmvpBufValid)
  {
    m_amvpSbTmvpBufValid = new bool[AMVP_SBTMVP_BUF_SIZE];
    m_amvpSbTmvpMotionBuf = new MotionInfo[AMVP_SBTMVP_BUF_SIZE];
  }
#endif
  m_isInitialized = true;
}

void InterSearch::resetSavedAffineMotion()
{
  for ( int i = 0; i < 2; i++ )
  {
    for ( int j = 0; j < 2; j++ )
    {
      m_affineMotion.acMvAffine4Para[i][j] = Mv( 0, 0 );
      m_affineMotion.acMvAffine6Para[i][j] = Mv( 0, 0 );
    }
    m_affineMotion.acMvAffine6Para[i][2] = Mv( 0, 0 );

    m_affineMotion.affine4ParaRefIdx[i] = -1;
    m_affineMotion.affine6ParaRefIdx[i] = -1;
  }
  for ( int i = 0; i < 3; i++ )
  {
    m_affineMotion.hevcCost[i] = std::numeric_limits<Distortion>::max();
  }
  m_affineMotion.affine4ParaAvail = false;
  m_affineMotion.affine6ParaAvail = false;
}

void InterSearch::storeAffineMotion( Mv acAffineMv[2][3], int8_t affineRefIdx[2], EAffineModel affineType, int bcwIdx )
{
  if ( ( bcwIdx == BCW_DEFAULT || !m_affineMotion.affine6ParaAvail ) && affineType == AFFINEMODEL_6PARAM )
  {
    for ( int i = 0; i < 2; i++ )
    {
      for ( int j = 0; j < 3; j++ )
      {
        m_affineMotion.acMvAffine6Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine6ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine6ParaAvail = true;
  }

  if ( ( bcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail ) && affineType == AFFINEMODEL_4PARAM )
  {
    for ( int i = 0; i < 2; i++ )
    {
      for ( int j = 0; j < 2; j++ )
      {
        m_affineMotion.acMvAffine4Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine4ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine4ParaAvail = true;
  }
}

inline void InterSearch::xTZSearchHelp( IntTZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance )
{
  Distortion  uiSad = 0;

//  CHECK(!( !( rcStruct.searchRange.left > iSearchX || rcStruct.searchRange.right < iSearchX || rcStruct.searchRange.top > iSearchY || rcStruct.searchRange.bottom < iSearchY )), "Unspecified error");

  const Pel* const  piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iRefStride + iSearchX;

  m_cDistParam.cur.buf = piRefSrch;

  if( 1 == rcStruct.subShiftMode )
  {
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      Distortion uiTempSad = m_cDistParam.distFunc( m_cDistParam );

      if((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        // it's not supposed that any member of DistParams is manipulated beside cur.buf
        int subShift = m_cDistParam.subShift;
        const Pel* pOrgCpy = m_cDistParam.org.buf;
        uiSad += uiTempSad >> m_cDistParam.subShift;

        while( m_cDistParam.subShift > 0 )
        {
          int isubShift           = m_cDistParam.subShift -1;
          m_cDistParam.org.buf = rcStruct.pcPatternKey->buf + (rcStruct.pcPatternKey->stride << isubShift);
          m_cDistParam.cur.buf = piRefSrch + (rcStruct.iRefStride << isubShift);
          uiTempSad            = m_cDistParam.distFunc( m_cDistParam );
          uiSad               += uiTempSad >> m_cDistParam.subShift;

          if(((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.subShift--;
        }

        if(m_cDistParam.subShift == 0)
        {
          uiSad += uiBitCost;

          if( uiSad < rcStruct.uiBestSad )
          {
            rcStruct.uiBestSad      = uiSad;
            rcStruct.iBestX         = iSearchX;
            rcStruct.iBestY         = iSearchY;
            rcStruct.uiBestDistance = uiDistance;
            rcStruct.uiBestRound    = 0;
            rcStruct.ucPointNr      = ucPointNr;
            m_cDistParam.maximumDistortionForEarlyExit = uiSad;
          }
        }

        // restore org ptr
        m_cDistParam.org.buf  = pOrgCpy;
        m_cDistParam.subShift = subShift;
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.distFunc( m_cDistParam );

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if( uiSad < rcStruct.uiBestSad )
    {
      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( iSearchX, iSearchY, rcStruct.imvShift );

      if( uiSad < rcStruct.uiBestSad )
      {
        rcStruct.uiBestSad      = uiSad;
        rcStruct.iBestX         = iSearchX;
        rcStruct.iBestY         = iSearchY;
        rcStruct.uiBestDistance = uiDistance;
        rcStruct.uiBestRound    = 0;
        rcStruct.ucPointNr      = ucPointNr;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}



inline void InterSearch::xTZ2PointSearch( IntTZSearchStruct& rcStruct )
{
  const SearchRange& sr = rcStruct.searchRange;

  static const int xOffset[2][9] = { {  0, -1, -1,  0, -1, +1, -1, -1, +1 }, {  0,  0, +1, +1, -1, +1,  0, +1,  0 } };
  static const int yOffset[2][9] = { {  0,  0, -1, -1, +1, -1,  0, +1,  0 }, {  0, -1, -1,  0, -1, +1, +1, +1, +1 } };

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  const int iX1 = rcStruct.iBestX + xOffset[0][rcStruct.ucPointNr];
  const int iX2 = rcStruct.iBestX + xOffset[1][rcStruct.ucPointNr];

  const int iY1 = rcStruct.iBestY + yOffset[0][rcStruct.ucPointNr];
  const int iY2 = rcStruct.iBestY + yOffset[1][rcStruct.ucPointNr];

  if( iX1 >= sr.left && iX1 <= sr.right && iY1 >= sr.top && iY1 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX1, iY1, 0, 2 );
  }

  if( iX2 >= sr.left && iX2 <= sr.right && iY2 >= sr.top && iY2 <= sr.bottom )
  {
    xTZSearchHelp( rcStruct, iX2, iY2, 0, 2 );
  }
}


inline void InterSearch::xTZ8PointSquareSearch( IntTZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0 , "Invalid distance");
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iTop >= sr.top ) // check top
  {
    if ( iLeft >= sr.left ) // check top left
    {
      xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
    }
    // top middle
    xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );

    if ( iRight <= sr.right ) // check top right
    {
      xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
    }
  } // check top
  if ( iLeft >= sr.left ) // check middle left
  {
    xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
  }
  if ( iRight <= sr.right ) // check middle right
  {
    xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
  }
  if ( iBottom <= sr.bottom ) // check bottom
  {
    if ( iLeft >= sr.left ) // check bottom left
    {
      xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
    }
    // check bottom middle
    xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );

    if ( iRight <= sr.right ) // check bottom right
    {
      xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
    }
  } // check bottom
}




inline void InterSearch::xTZ8PointDiamondSearch( IntTZSearchStruct& rcStruct,
                                                 const int iStartX,
                                                 const int iStartY,
                                                 const int iDist,
                                                 const bool bCheckCornersAtDist1 )
{
  const SearchRange& sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK( iDist == 0, "Invalid distance" );
  const int iTop        = iStartY - iDist;
  const int iBottom     = iStartY + iDist;
  const int iLeft       = iStartX - iDist;
  const int iRight      = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if ( iDist == 1 )
  {
    if ( iTop >= sr.top ) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iTop, 1, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iTop, 3, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
      }
    }
    if ( iLeft >= sr.left ) // check middle left
    {
      xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
    }
    if ( iRight <= sr.right ) // check middle right
    {
      xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
    }
    if ( iBottom <= sr.bottom ) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if ( iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp( rcStruct, iLeft, iBottom, 6, iDist );
        }
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        if ( iRight <= sr.right ) // check middle right
        {
          xTZSearchHelp( rcStruct, iRight, iBottom, 8, iDist );
        }
      }
      else
      {
        xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
      }
    }
  }
  else
  {
    if ( iDist <= 8 )
    {
      const int iTop_2      = iStartY - (iDist>>1);
      const int iBottom_2   = iStartY + (iDist>>1);
      const int iLeft_2     = iStartX - (iDist>>1);
      const int iRight_2    = iStartX + (iDist>>1);

      if (  iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX,  iTop,      2, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iTop_2,    1, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iTop_2,    3, iDist>>1 );
        xTZSearchHelp( rcStruct, iLeft,    iStartY,   4, iDist    );
        xTZSearchHelp( rcStruct, iRight,   iStartY,   5, iDist    );
        xTZSearchHelp( rcStruct, iLeft_2,  iBottom_2, 6, iDist>>1 );
        xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, iDist>>1 );
        xTZSearchHelp( rcStruct, iStartX,  iBottom,   7, iDist    );
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 2, iDist );
        }
        if ( iTop_2 >= sr.top ) // check half top
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iTop_2, 1, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iTop_2, 3, (iDist>>1) );
          }
        } // check half top
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 4, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 5, iDist );
        }
        if ( iBottom_2 <= sr.bottom ) // check half bottom
        {
          if ( iLeft_2 >= sr.left ) // check half left
          {
            xTZSearchHelp( rcStruct, iLeft_2, iBottom_2, 6, (iDist>>1) );
          }
          if ( iRight_2 <= sr.right ) // check half right
          {
            xTZSearchHelp( rcStruct, iRight_2, iBottom_2, 8, (iDist>>1) );
          }
        } // check half bottom
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 7, iDist );
        }
      } // check border
    }
    else // iDist > 8
    {
      if ( iTop >= sr.top && iLeft >= sr.left &&
           iRight <= sr.right && iBottom <= sr.bottom ) // check border
      {
        xTZSearchHelp( rcStruct, iStartX, iTop,    0, iDist );
        xTZSearchHelp( rcStruct, iLeft,   iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iRight,  iStartY, 0, iDist );
        xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);
          xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
          xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
        }
      }
      else // check border
      {
        if ( iTop >= sr.top ) // check top
        {
          xTZSearchHelp( rcStruct, iStartX, iTop, 0, iDist );
        }
        if ( iLeft >= sr.left ) // check left
        {
          xTZSearchHelp( rcStruct, iLeft, iStartY, 0, iDist );
        }
        if ( iRight <= sr.right ) // check right
        {
          xTZSearchHelp( rcStruct, iRight, iStartY, 0, iDist );
        }
        if ( iBottom <= sr.bottom ) // check bottom
        {
          xTZSearchHelp( rcStruct, iStartX, iBottom, 0, iDist );
        }
        for ( int index = 1; index < 4; index++ )
        {
          const int iPosYT = iTop    + ((iDist>>2) * index);
          const int iPosYB = iBottom - ((iDist>>2) * index);
          const int iPosXL = iStartX - ((iDist>>2) * index);
          const int iPosXR = iStartX + ((iDist>>2) * index);

          if ( iPosYT >= sr.top ) // check top
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYT, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYT, 0, iDist );
            }
          } // check top
          if ( iPosYB <= sr.bottom ) // check bottom
          {
            if ( iPosXL >= sr.left ) // check left
            {
              xTZSearchHelp( rcStruct, iPosXL, iPosYB, 0, iDist );
            }
            if ( iPosXR <= sr.right ) // check right
            {
              xTZSearchHelp( rcStruct, iPosXR, iPosYB, 0, iDist );
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion InterSearch::xPatternRefinement( const CPelBuf* pcPatternKey,
                                            Mv baseRefMv,
                                            int iFrac, Mv& rcMvFrac,
                                            bool bAllowUseOfHadamard )
{
  Distortion  uiDist;
  Distortion  uiDistBest  = std::numeric_limits<Distortion>::max();
  uint32_t        uiDirecBest = 0;

  Pel*  piRefPos;
  int iRefStride = pcPatternKey->width + 1;
  m_pcRdCost->setDistParam( m_cDistParam, *pcPatternKey, m_filteredBlock[0][0][0], iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && bAllowUseOfHadamard );

  const Mv* pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  for (uint32_t i = 0; i < 9; i++)
  {
    if (m_skipFracME && i > 0)
    {
      break;
    }
    Mv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    int horVal = cMvTest.getHor() * iFrac;
    int verVal = cMvTest.getVer() * iFrac;
    piRefPos = m_filteredBlock[verVal & 3][horVal & 3][0];

    if (horVal == 2 && (verVal & 1) == 0)
    {
      piRefPos += 1;
    }
    if ((horVal & 1) == 0 && verVal == 2)
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;


    m_cDistParam.cur.buf   = piRefPos;
    uiDist = m_cDistParam.distFunc( m_cDistParam );
    uiDist += m_pcRdCost->getCostOfVectorWithPredictor( cMvTest.getHor(), cMvTest.getVer(), 0 );

    if ( uiDist < uiDistBest )
    {
      uiDistBest  = uiDist;
      uiDirecBest = i;
      m_cDistParam.maximumDistortionForEarlyExit = uiDist;
    }
  }

  rcMvFrac = pcMvRefine[uiDirecBest];

  return uiDistBest;
}

Distortion InterSearch::xGetInterPredictionError( PredictionUnit& pu, PelUnitBuf& origBuf, const RefPicList &eRefPicList )
{
  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  motionCompensation( pu, predBuf, eRefPicList );

  DistParam cDistParam;
  cDistParam.applyWeight = false;

  m_pcRdCost->setDistParam(cDistParam, origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, m_pcEncCfg->getUseHADME() && !pu.cu->slice->getDisableSATDForRD());

  return (Distortion)cDistParam.distFunc( cDistParam );
}

/// add ibc search functions here

void InterSearch::xIBCSearchMVCandUpdate(Distortion  sad, int x, int y, Distortion* sadBestCand, Mv* cMVCand)
{
  int j = CHROMA_REFINEMENT_CANDIDATES - 1;

  if (sad < sadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
  {
    for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (m_bestSrchCostIntBv.enableFracIBC && (cMVCand[t].hor == x && cMVCand[t].ver == y))
      {
        CHECK(sad != sadBestCand[t], "This should not happen");
        return;
      }
#endif
      if (sad < sadBestCand[t])
        j = t;
    }

    for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
    {
      sadBestCand[k] = sadBestCand[k - 1];

      cMVCand[k].set(cMVCand[k - 1].getHor(), cMVCand[k - 1].getVer());
    }
    sadBestCand[j] = sad;
    cMVCand[j].set(x, y);
  }
}

#if JVET_AA0070_RRIBC
int InterSearch::xIBCSearchMVChromaRefine( PredictionUnit& pu, int roiWidth, int roiHeight, int cuPelX, int cuPelY, Distortion* sadBestCand, Mv* cMVCand, int rribcFlipType )
#else
int InterSearch::xIBCSearchMVChromaRefine(PredictionUnit& pu,
  int         roiWidth,
  int         roiHeight,
  int         cuPelX,
  int         cuPelY,
  Distortion* sadBestCand,
  Mv*     cMVCand
)
#endif
{
  if ( (!isChromaEnabled(pu.chromaFormat)) || (!pu.Cb().valid()) )
  {
    return 0;
  }

  int bestCandIdx = 0;
  Distortion  sadBest = std::numeric_limits<Distortion>::max();
  Distortion  tempSad;

  Pel* pRef;
  Pel* pOrg;
  int refStride, orgStride;
  int width, height;

  int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();

  UnitArea allCompBlocks(pu.chromaFormat, (Area)pu.block(COMPONENT_Y));
  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
    {
      continue;
    }
#if JVET_AA0070_RRIBC
    if ((rribcFlipType == 1 && cMVCand[cand].getVer() != 0) || (rribcFlipType == 2 && cMVCand[cand].getHor() != 0))
    {
      continue;
    }
#endif

    if ((!cMVCand[cand].getHor()) && (!cMVCand[cand].getVer()))
      continue;

    if (((int)(cuPelY + cMVCand[cand].getVer() + roiHeight) >= picHeight) || ((cuPelY + cMVCand[cand].getVer()) < 0))
      continue;

    if (((int)(cuPelX + cMVCand[cand].getHor() + roiWidth) >= picWidth) || ((cuPelX + cMVCand[cand].getHor()) < 0))
      continue;

    tempSad = sadBestCand[cand];

    pu.mv[0] = cMVCand[cand];
    pu.mv[0].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    pu.interDir = 1;
    pu.refIdx[0] = pu.cs->slice->getNumRefIdx(REF_PIC_LIST_0); // last idx in the list

    PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_0].getBuf(UnitAreaRelative(*pu.cu, pu));
    motionCompensation(pu, predBufTmp, REF_PIC_LIST_0);

    for (unsigned int ch = COMPONENT_Cb; ch < ::getNumberValidComponents(pu.chromaFormat); ch++)
    {
      width = roiWidth >> ::getComponentScaleX(ComponentID(ch), pu.chromaFormat);
      height = roiHeight >> ::getComponentScaleY(ComponentID(ch), pu.chromaFormat);

      PelUnitBuf origBuf = pu.cs->getOrgBuf(allCompBlocks);
      PelUnitBuf* pBuf = &origBuf;
#if JVET_AA0070_RRIBC
      PelBuf tmpPattern;
      if (rribcFlipType)
      {
        CompArea tmpArea(ComponentID(ch), pu.chromaFormat, Position(0, 0), Size(pBuf->get(ComponentID(ch)).width, pBuf->get(ComponentID(ch)).height));
        tmpPattern = m_tmpStorageLCU.getBuf(tmpArea);
        tmpPattern.copyFrom(pBuf->get(ComponentID(ch)));
        tmpPattern.flipSignal(rribcFlipType == 1);
      }
      else
      {
        tmpPattern = pBuf->get(ComponentID(ch));
      }
#else
      CPelBuf  tmpPattern = pBuf->get(ComponentID(ch));
#endif
      pOrg = (Pel*)tmpPattern.buf;

      Picture* refPic = pu.cu->slice->getPic();
      const CPelBuf refBuf = refPic->getRecoBuf(allCompBlocks.blocks[ComponentID(ch)]);
      pRef = (Pel*)refBuf.buf;

      refStride = refBuf.stride;
      orgStride = tmpPattern.stride;

      //ComponentID compID = (ComponentID)ch;
      PelUnitBuf* pBufRef = &predBufTmp;
      CPelBuf  tmpPatternRef = pBufRef->get(ComponentID(ch));
      pRef = (Pel*)tmpPatternRef.buf;
      refStride = tmpPatternRef.stride;


      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          tempSad += ((abs(pRef[col] - pOrg[col])) >> (pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 8));
        }
        pRef += refStride;
        pOrg += orgStride;
      }
    }

    if (tempSad < sadBest)
    {
      sadBest = tempSad;
      bestCandIdx = cand;
    }
  }

  return bestCandIdx;
}

static unsigned int xMergeCandLists(Mv *dst, unsigned int dn, unsigned int dstTotalLength, Mv *src, unsigned int sn)
{
  for (unsigned int cand = 0; cand < sn && dn < dstTotalLength; cand++)
  {
    if (src[cand] == Mv())
    {
      continue;
    }
    bool found = false;
    for (int j = 0; j<dn; j++)
    {
      if (src[cand] == dst[j])
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      dst[dn] = src[cand];
      dn++;
    }
  }

  return dn;
}
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
void InterSearch::xIntraPatternSearchOpt(PredictionUnit &pu, IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiCost, Mv *pcMvSrchRngLT, Mv *pcMvSrchRngRB, Mv *pcMvPred
#if JVET_AA0070_RRIBC
                                       , int rribcFlipType
#endif
)
{
  const int   srchRngHorLeft = pcMvSrchRngLT->getHor();
  const int   srchRngHorRight = pcMvSrchRngRB->getHor();
  const int   srchRngVerTop = pcMvSrchRngLT->getVer();
  const int   srchRngVerBottom = pcMvSrchRngRB->getVer();

  const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  const int   puPelOffsetX = 0;
  const int   puPelOffsetY = 0;
  const int   cuPelX = pu.Y().x;
  const int   cuPelY = pu.Y().y;

  int          roiWidth = pu.lwidth();
  int          roiHeight = pu.lheight();

  Distortion  sad;
  Distortion  sadBest = std::numeric_limits<Distortion>::max();
  int         bestX = 0;
  int         bestY = 0;

  const Pel*        piRefSrch = cStruct.piRefY;

  int         bestCandIdx = 0;

  Distortion  sadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  Mv      cMVCand[CHROMA_REFINEMENT_CANDIDATES];


  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    sadBestCand[cand] = std::numeric_limits<Distortion>::max();
    cMVCand[cand].set(0, 0);
  }
  m_cDistParam.useMR = false;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_pcRdCost->setFullPelImvForZeroBvd(!pu.cs->sps->getIBCFracFlag());
#endif

  const int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();

  {
    m_cDistParam.subShift = 0;
    
    int srLeft = srchRngHorLeft, srRight = srchRngHorRight, srTop = srchRngVerTop, srBottom = srchRngVerBottom;
    m_numBVs = 0;
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt);

    Mv cMvPredEncOnly[IBC_NUM_CANDIDATES];
    int nbPreds = 0;
    PU::getIbcMVPsEncOnly(pu, cMvPredEncOnly, nbPreds);
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), cMvPredEncOnly, nbPreds);

    for (unsigned int cand = 0; cand < m_numBVs; cand++)
    {
      int xPred = m_acBVs[cand].getHor();
      int yPred = m_acBVs[cand].getVer();

#if JVET_AA0070_RRIBC
      if ((rribcFlipType == 1 && yPred != 0) || (rribcFlipType == 2 && xPred != 0))
      {
        continue;
      }
#endif

      if (!(xPred == 0 && yPred == 0)
        && !((yPred < srTop) || (yPred > srBottom))
        && !((xPred < srLeft) || (xPred > srRight)))
      {
#if JVET_Z0084_IBC_TM
        bool validCand = PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);
#else
        bool validCand = searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);
#endif

        if (validCand)
        {
#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, xPred, yPred, sadBestCand, cMVCand);
        }
      }
    }

    bestX = cMVCand[0].getHor();
    bestY = cMVCand[0].getVer();
    rcMv.set(bestX, bestY);
    sadBest = sadBestCand[0];

    const int boundY = (0 - roiHeight - puPelOffsetY);
#if JVET_AE0169_BIPREDICTIVE_IBC
    for (int y = std::max(srchRngVerTop, 0 - cuPelY); y <= boundY; y+=2)
#else
    for (int y = std::max(srchRngVerTop, 0 - cuPelY); y <= boundY; ++y)
#endif
    {
#if JVET_AA0070_RRIBC
      if (rribcFlipType == 1 && y != 0)
      {
        continue;
      }
#endif
#if JVET_Z0084_IBC_TM
      if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#else
      if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#endif
      {
        continue;
      }

#if JVET_AA0070_RRIBC
      sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
      sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag());
#endif
      m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y;
      sad += m_cDistParam.distFunc(m_cDistParam);

      xIBCSearchMVCandUpdate(sad, 0, y, sadBestCand, cMVCand);
      
      if (sadBestCand[0] <= 3)
      {
#if JVET_AE0169_BIPREDICTIVE_IBC
        break;
#else
        bestX = cMVCand[0].getHor();
        bestY = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
#endif
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (sadBestCand[0] < sadBest && rribcFlipType != 1)
    {
      bestY = cMVCand[0].getVer();
      for (int y = bestY-1; y <= bestY+1; y+=2)
      {
#if JVET_Z0084_IBC_TM
        if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#else
        if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#endif
        {
          continue;
        }

#if JVET_AA0070_RRIBC
        sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
        sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag());
#endif
        m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y;
        sad += m_cDistParam.distFunc(m_cDistParam);

        xIBCSearchMVCandUpdate(sad, 0, y, sadBestCand, cMVCand);
      }
    }
    if (sadBestCand[0] <= 3)
    {
      bestX = cMVCand[0].getHor();
      bestY = cMVCand[0].getVer();
      sadBest = sadBestCand[0];
      rcMv.set(bestX, bestY);
      ruiCost = sadBest;
      goto end;
    }
#endif

    const int boundX = std::max(srchRngHorLeft, -cuPelX);
#if JVET_AE0169_BIPREDICTIVE_IBC
    sadBest = sadBestCand[0];
    for (int x = 0 - roiWidth - puPelOffsetX; x >= boundX; x-=2)
#else
    for (int x = 0 - roiWidth - puPelOffsetX; x >= boundX; --x)
#endif
    {
#if JVET_AA0070_RRIBC
      if (rribcFlipType == 2 && x != 0)
      {
        continue;
      }
#endif
#if JVET_Z0084_IBC_TM
      if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#else
      if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#endif
      {
        continue;
      }

#if JVET_AA0070_RRIBC
      sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
      sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag());
#endif
      m_cDistParam.cur.buf = piRefSrch + x;
      sad += m_cDistParam.distFunc(m_cDistParam);


      xIBCSearchMVCandUpdate(sad, x, 0, sadBestCand, cMVCand);

      if (sadBestCand[0] <= 3)
      {
#if JVET_AE0169_BIPREDICTIVE_IBC
        break;
#else
        bestX = cMVCand[0].getHor();
        bestY = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
#endif
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (sadBestCand[0] < sadBest && rribcFlipType != 2)
    {
      bestX = cMVCand[0].getHor();
      for (int x = bestX+1; x >= bestX-1; x-=2)
      {
#if JVET_Z0084_IBC_TM
        if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#else
        if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#endif
        {
          continue;
        }

#if JVET_AA0070_RRIBC
        sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
        sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag());
#endif
        m_cDistParam.cur.buf = piRefSrch + x;
        sad += m_cDistParam.distFunc(m_cDistParam);


        xIBCSearchMVCandUpdate(sad, x, 0, sadBestCand, cMVCand);
      }
      if (sadBestCand[0] <= 3)
      {
        bestX = cMVCand[0].getHor();
        bestY = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }
#endif

    bestX = cMVCand[0].getHor();
    bestY = cMVCand[0].getVer();
    sadBest = sadBestCand[0];
#if JVET_AA0070_RRIBC
    if ((!bestX && !bestY) || (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType) <= 32))
    {
      //chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
    if ((!bestX && !bestY) || (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag()) <= 32))
    {
      //chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
      bestX = cMVCand[bestCandIdx].getHor();
      bestY = cMVCand[bestCandIdx].getVer();
      sadBest = sadBestCand[bestCandIdx];
      rcMv.set(bestX, bestY);
      ruiCost = sadBest;
      goto end;
    }

    if (pu.lwidth() < 128 && pu.lheight() < 128)
    {
#if JVET_Z0153_IBC_EXT_REF
#if JVET_AE0169_BIPREDICTIVE_IBC
      int ibcSearchSizeX = bestX == 0 ? (IBC_SEARCH_RANGE>>1) : IBC_SEARCH_RANGE;
      int ibcSearchSizeY = bestY == 0 ? (IBC_SEARCH_RANGE>>1) : IBC_SEARCH_RANGE;
      int verTop    = bestY - ibcSearchSizeY;
      int verBottom = bestY + ibcSearchSizeY;
      int horLeft   = bestX - ibcSearchSizeX;
      int horRight  = bestX + ibcSearchSizeX;
#else
      int verTop    = bestY - IBC_SEARCH_RANGE;
      int verBottom = bestY + IBC_SEARCH_RANGE;
      int horLeft   = bestX - IBC_SEARCH_RANGE;
      int horRight  = bestX + IBC_SEARCH_RANGE;
#endif

      for (int y = std::max(verTop, -cuPelY); y <= verBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }
        for (int x = std::max(horLeft, -cuPelX); x <= horRight; x += 2)
        {
#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
#else
      for (int y = std::max(srchRngVerTop, -cuPelY); y <= srchRngVerBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }

        for (int x = std::max(srchRngHorLeft, -cuPelX); x <= srchRngHorRight; x++)
        {
#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
#endif
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
          {
            continue;
          }

#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#else
          if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#endif
          {
            continue;
          }
#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
        }
      }

      bestX = cMVCand[0].getHor();
      bestY = cMVCand[0].getVer();
      sadBest = sadBestCand[0];
#if JVET_AA0070_RRIBC
      if (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType) <= 16)
      {
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
      if (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag()) <= 16)
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
        bestX = cMVCand[bestCandIdx].getHor();
        bestY = cMVCand[bestCandIdx].getVer();
        sadBest = sadBestCand[bestCandIdx];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
      }

    //refinement (+/-1) around best 8 candidates so far
    //copy the MVCandList, as the main list is updating
    Mv cMvCandTemp[CHROMA_REFINEMENT_CANDIDATES];
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
      cMvCandTemp[cand] = cMVCand[cand];
    }
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
      int centerX = cMvCandTemp[cand].getHor();
      int centerY = cMvCandTemp[cand].getVer();
      for (int y = centerY - 1; y < centerY + 2; y++)
      {
        for (int x = centerX - 1; x < centerX + 2; x++)
        {
          if (x == centerX && y == centerY)
          {
            continue;
          }
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
          {
            continue;
          }
#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#else
          if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#endif
          {
            continue;
          }
#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
        }
      }
    }

  }
#if JVET_AA0070_RRIBC
  bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
  bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif

  bestX = cMVCand[bestCandIdx].getHor();
  bestY = cMVCand[bestCandIdx].getVer();
  sadBest = sadBestCand[bestCandIdx];
  rcMv.set(bestX, bestY);
  ruiCost = sadBest;

end:
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pu.cs->sps->getIBCFracFlag()
#if JVET_AA0070_RRIBC
    && rribcFlipType == 0
#endif
    )
  {
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
      if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
      {
        break;
      }
      m_bestSrchCostIntBv.insert(sadBestCand[cand], cMVCand[cand].getHor(), cMVCand[cand].getVer()
#if JVET_AE0169_BIPREDICTIVE_IBC
                               , MAX_UCHAR
#endif
#if JVET_AA0070_RRIBC
                               , rribcFlipType
#endif
      );
    }
  }
#endif

  m_numBVs = 0;
  m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt);

  m_defaultCachedBvs.currCnt = 0;
  m_defaultCachedBvs.currCnt = xMergeCandLists(m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt, IBC_NUM_CANDIDATES, cMVCand, CHROMA_REFINEMENT_CANDIDATES);
  m_defaultCachedBvs.currCnt = xMergeCandLists(m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt, IBC_NUM_CANDIDATES, m_acBVs, m_numBVs);

  for (unsigned int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
#if JVET_AA0070_RRIBC
    if ((rribcFlipType == 1 && cMVCand[cand].getVer() != 0) || (rribcFlipType == 2 && cMVCand[cand].getHor() != 0))
    {
      continue;
    }
    if (m_pcEncCfg->getIntraPeriod() < 1 && m_pcEncCfg->getIBCFastMethod())
    {
      if (sadBestCand[cand] >= minCostProj)
      {
        break;
      }
      else if (!cand && rribcFlipType)
      {
        minCostProj = 3 * sadBestCand[0];
      }
    }
#endif
    if (cMVCand[cand].getHor() == 0 && cMVCand[cand].getVer() == 0)
    {
      continue;
    }

    m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord[cMVCand[cand]] = sadBestCand[cand];
  }

  return;
 }
#endif
#if JVET_AA0070_RRIBC
void InterSearch::xIntraPatternSearch( PredictionUnit &pu, IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiCost, Mv *pcMvSrchRngLT, Mv *pcMvSrchRngRB, Mv *pcMvPred, int rribcFlipType )
#else
void InterSearch::xIntraPatternSearch(PredictionUnit &pu, IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiCost, Mv *pcMvSrchRngLT, Mv *pcMvSrchRngRB, Mv *pcMvPred)
#endif
{
  const int   srchRngHorLeft = pcMvSrchRngLT->getHor();
  const int   srchRngHorRight = pcMvSrchRngRB->getHor();
  const int   srchRngVerTop = pcMvSrchRngLT->getVer();
  const int   srchRngVerBottom = pcMvSrchRngRB->getVer();

  const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  const int   puPelOffsetX = 0;
  const int   puPelOffsetY = 0;
  const int   cuPelX = pu.Y().x;
  const int   cuPelY = pu.Y().y;

  int          roiWidth = pu.lwidth();
  int          roiHeight = pu.lheight();

  Distortion  sad;
  Distortion  sadBest = std::numeric_limits<Distortion>::max();
  int         bestX = 0;
  int         bestY = 0;

  const Pel*        piRefSrch = cStruct.piRefY;

  int         bestCandIdx = 0;

  Distortion  sadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  Mv      cMVCand[CHROMA_REFINEMENT_CANDIDATES];


  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    sadBestCand[cand] = std::numeric_limits<Distortion>::max();
    cMVCand[cand].set(0, 0);
  }

  m_cDistParam.useMR = false;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_pcRdCost->setFullPelImvForZeroBvd(!pu.cs->sps->getIBCFracFlag());
#endif

  const int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();


  {
    m_cDistParam.subShift = 0;

    Distortion tempSadBest = 0;

    int srLeft = srchRngHorLeft, srRight = srchRngHorRight, srTop = srchRngVerTop, srBottom = srchRngVerBottom;
    m_numBVs = 0;
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt);

    Mv cMvPredEncOnly[IBC_NUM_CANDIDATES];
    int nbPreds = 0;
    PU::getIbcMVPsEncOnly(pu, cMvPredEncOnly, nbPreds);
    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), cMvPredEncOnly, nbPreds);

    for (unsigned int cand = 0; cand < m_numBVs; cand++)
    {
      int xPred = m_acBVs[cand].getHor();
      int yPred = m_acBVs[cand].getVer();

#if JVET_AA0070_RRIBC
      if ((rribcFlipType == 1 && yPred != 0) || (rribcFlipType == 2 && xPred != 0))
      {
        continue;
      }
#endif

      if (!(xPred == 0 && yPred == 0)
        && !((yPred < srTop) || (yPred > srBottom))
        && !((xPred < srLeft) || (xPred > srRight)))
      {
#if JVET_Z0084_IBC_TM
        bool validCand = PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);
#else
        bool validCand = searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);
#endif

        if (validCand)
        {
#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, xPred, yPred, sadBestCand, cMVCand);
        }
      }
    }

    bestX = cMVCand[0].getHor();
    bestY = cMVCand[0].getVer();
    rcMv.set(bestX, bestY);
    sadBest = sadBestCand[0];

    const int boundY = (0 - roiHeight - puPelOffsetY);
    for (int y = std::max(srchRngVerTop, 0 - cuPelY); y <= boundY; ++y)
    {
#if JVET_AA0070_RRIBC
      if (rribcFlipType == 1 && y != 0)
      {
        continue;
      }
#endif
#if JVET_Z0084_IBC_TM
      if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#else
      if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
#endif
      {
        continue;
      }

#if JVET_AA0070_RRIBC
      sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
      sad = m_pcRdCost->getBvCostMultiplePreds(0, y, pu.cs->sps->getAMVREnabledFlag());
#endif
      m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y;
      sad += m_cDistParam.distFunc(m_cDistParam);

      xIBCSearchMVCandUpdate(sad, 0, y, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX = cMVCand[0].getHor();
        bestY = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    const int boundX = std::max(srchRngHorLeft, -cuPelX);
    for (int x = 0 - roiWidth - puPelOffsetX; x >= boundX; --x)
    {
#if JVET_AA0070_RRIBC
      if (rribcFlipType == 2 && x != 0)
      {
        continue;
      }
#endif
#if JVET_Z0084_IBC_TM
      if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#else
      if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
#endif
      {
        continue;
      }

#if JVET_AA0070_RRIBC
      sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
      sad = m_pcRdCost->getBvCostMultiplePreds(x, 0, pu.cs->sps->getAMVREnabledFlag());
#endif
      m_cDistParam.cur.buf = piRefSrch + x;
      sad += m_cDistParam.distFunc(m_cDistParam);


      xIBCSearchMVCandUpdate(sad, x, 0, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX = cMVCand[0].getHor();
        bestY = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    bestX = cMVCand[0].getHor();
    bestY = cMVCand[0].getVer();
    sadBest = sadBestCand[0];
#if JVET_AA0070_RRIBC
    if ((!bestX && !bestY) || (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType) <= 32))
    {
      //chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
    if ((!bestX && !bestY) || (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag()) <= 32))
    {
      //chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
      bestX = cMVCand[bestCandIdx].getHor();
      bestY = cMVCand[bestCandIdx].getVer();
      sadBest = sadBestCand[bestCandIdx];
      rcMv.set(bestX, bestY);
      ruiCost = sadBest;
      goto end;
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    int blkSize = (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) ? 128 : 16;
    if (pu.lwidth() < blkSize && pu.lheight() < blkSize)
#else
    if (pu.lwidth() < 16 && pu.lheight() < 16)
#endif
    {
#if JVET_Z0153_IBC_EXT_REF
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
      int verTop = pu.cu->slice->getSPS()->getUseLargeIBCLSR()? -(int)lcuWidth * 2:- (int)lcuWidth;
#else
      int verTop = - (int)lcuWidth;
#endif
      int verBottom = std::min((int)(lcuWidth>>2), (int)(lcuWidth - (cuPelY % lcuWidth) - roiHeight));
      int horLeft = - (int)lcuWidth*2;
      int horRight = lcuWidth>>2;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC
      if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) && pu.cu->predMode == MODE_IBC && rribcFlipType == 0)
#else
      if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) && pu.cu->predMode == MODE_IBC)
#endif
      {
        verTop    = bestY - IBC_SEARCH_RANGE;
        verBottom = bestY + IBC_SEARCH_RANGE;
        horLeft   = bestX - IBC_SEARCH_RANGE;
        horRight  = bestX + IBC_SEARCH_RANGE;
      }
#endif
      for (int y = std::max(verTop, -cuPelY); y <= verBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }        
        for (int x = std::max(horLeft, -cuPelX); x <= horRight; x++)
        {
#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
#else
      for (int y = std::max(srchRngVerTop, -cuPelY); y <= srchRngVerBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
          continue;

        for (int x = std::max(srchRngHorLeft, -cuPelX); x <= srchRngHorRight; x++)
        {
#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
#endif
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
            continue;

#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#else
          if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#endif
          {
            continue;
          }

#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
        }
      }

      bestX = cMVCand[0].getHor();
      bestY = cMVCand[0].getVer();
      sadBest = sadBestCand[0];
#if JVET_AA0070_RRIBC
      if (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType) <= 16)
      {
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
      if (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag()) <= 16)
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
        bestX = cMVCand[bestCandIdx].getHor();
        bestY = cMVCand[bestCandIdx].getVer();
        sadBest = sadBestCand[bestCandIdx];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }

#if JVET_Z0153_IBC_EXT_REF
      for (int y = (std::max(verTop, -cuPelY) + 1); y <= verBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }

        for (int x = std::max(horLeft, -cuPelX); x <= horRight; x += 2)
#else
      for (int y = (std::max(srchRngVerTop, -cuPelY) + 1); y <= srchRngVerBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
          continue;

        for (int x = std::max(srchRngHorLeft, -cuPelX); x <= srchRngHorRight; x += 2)
#endif
        {
#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
            continue;

#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#else
          if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#endif
          {
            continue;
          }

#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag());
#endif

          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);


          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
          if (sadBestCand[0] <= 5)
          {
            // chroma refine & return
#if JVET_AA0070_RRIBC
            bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
            bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
            bestX = cMVCand[bestCandIdx].getHor();
            bestY = cMVCand[bestCandIdx].getVer();
            sadBest = sadBestCand[bestCandIdx];
            rcMv.set(bestX, bestY);
            ruiCost = sadBest;
            goto end;
          }
        }
      }

      bestX = cMVCand[0].getHor();
      bestY = cMVCand[0].getVer();
      sadBest = sadBestCand[0];

#if JVET_AA0070_RRIBC
      if ((sadBest >= tempSadBest) || ((sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType)) <= 32))
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
      if ((sadBest >= tempSadBest) || ((sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, pu.cs->sps->getAMVREnabledFlag())) <= 32))
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
        bestX = cMVCand[bestCandIdx].getHor();
        bestY = cMVCand[bestCandIdx].getVer();
        sadBest = sadBestCand[bestCandIdx];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }

      tempSadBest = sadBestCand[0];

#if JVET_Z0153_IBC_EXT_REF
      for (int y = (std::max(verTop, -cuPelY) + 1); y <= verBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }
        for (int x = (std::max(horLeft, -cuPelX) + 1); x <= horRight; x += 2)
#else
      for (int y = (std::max(srchRngVerTop, -cuPelY) + 1); y <= srchRngVerBottom; y += 2)
      {
#if JVET_AA0070_RRIBC
        if (rribcFlipType == 1 && y != 0)
        {
          continue;
        }
#endif
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
          continue;

        for (int x = (std::max(srchRngHorLeft, -cuPelX) + 1); x <= srchRngHorRight; x += 2)
#endif
        {

#if JVET_AA0070_RRIBC
          if (rribcFlipType == 2 && x != 0)
          {
            continue;
          }
#endif
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
            continue;

#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#else
          if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
#endif
          {
            continue;
          }

#if JVET_AA0070_RRIBC
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#else
          sad = m_pcRdCost->getBvCostMultiplePreds(x, y, pu.cs->sps->getAMVREnabledFlag());
#endif
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);


          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
          if (sadBestCand[0] <= 5)
          {
            // chroma refine & return
#if JVET_AA0070_RRIBC
            bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
            bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif
            bestX = cMVCand[bestCandIdx].getHor();
            bestY = cMVCand[bestCandIdx].getVer();
            sadBest = sadBestCand[bestCandIdx];
            rcMv.set(bestX, bestY);
            ruiCost = sadBest;
            goto end;
          }
        }
      }
    }
  }

#if JVET_AA0070_RRIBC
  bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand, rribcFlipType);
#else
  bestCandIdx = xIBCSearchMVChromaRefine(pu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
#endif

  bestX = cMVCand[bestCandIdx].getHor();
  bestY = cMVCand[bestCandIdx].getVer();
  sadBest = sadBestCand[bestCandIdx];
  rcMv.set(bestX, bestY);
  ruiCost = sadBest;

end:
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pu.cs->sps->getIBCFracFlag()
#if JVET_AA0070_RRIBC
    && rribcFlipType == 0
#endif
    )
  {
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
      if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
      {
        break;
      }
      m_bestSrchCostIntBv.insert(sadBestCand[cand], cMVCand[cand].getHor(), cMVCand[cand].getVer()
#if JVET_AE0169_BIPREDICTIVE_IBC
                               , MAX_UCHAR
#endif
#if JVET_AA0070_RRIBC
                               , rribcFlipType
#endif
      );
    }
  }
#endif

  m_numBVs = 0;
  m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, (2 * IBC_NUM_CANDIDATES), m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt);

  m_defaultCachedBvs.currCnt = 0;
  m_defaultCachedBvs.currCnt = xMergeCandLists(m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt, IBC_NUM_CANDIDATES, cMVCand, CHROMA_REFINEMENT_CANDIDATES);
  m_defaultCachedBvs.currCnt = xMergeCandLists(m_defaultCachedBvs.m_bvCands, m_defaultCachedBvs.currCnt, IBC_NUM_CANDIDATES, m_acBVs, m_numBVs);

  for (unsigned int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
#if JVET_AA0070_RRIBC
    if ((rribcFlipType == 1 && cMVCand[cand].getVer() != 0) || (rribcFlipType == 2 && cMVCand[cand].getHor() != 0))
    {
      continue;
    }
    if (m_pcEncCfg->getIntraPeriod() < 1 && m_pcEncCfg->getIBCFastMethod())
    {
      if (sadBestCand[cand] >= minCostProj)
      {
        break;
      }
      else if (!cand && rribcFlipType)
      {
        minCostProj =  3 * sadBestCand[0];
      }
    }
#endif
    if (cMVCand[cand].getHor() == 0 && cMVCand[cand].getVer() == 0)
    {
      continue;
    }

    m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord[cMVCand[cand]] = sadBestCand[cand];
  }

  return;
}


// based on xMotionEstimation
#if JVET_AA0070_RRIBC
#if JVET_AC0112_IBC_CIIP
void InterSearch::xIBCEstimation(PredictionUnit &pu, PelUnitBuf &origBuf, PelBuf &ibcCiipIntraBuf, Mv pcMvPred[3][2], Mv &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY, int numRribcType)
#else
void InterSearch::xIBCEstimation(PredictionUnit &pu, PelUnitBuf &origBuf, Mv pcMvPred[3][2], Mv &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY, int numRribcType)
#endif
#else
#if JVET_AC0112_IBC_CIIP
void InterSearch::xIBCEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, PelBuf &ibcCiipIntraBuf, Mv *pcMvPred, Mv &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY)
#else
void InterSearch::xIBCEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, Mv *pcMvPred, Mv &rcMv, Distortion &ruiCost, const int localSearchRangeX, const int localSearchRangeY)
#endif
#endif
{
  const int iPicWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int iPicHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
  const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  const int           cuPelX = pu.Y().x;
  const int           cuPelY = pu.Y().y;
  int                 iRoiWidth = pu.lwidth();
  int                 iRoiHeight = pu.lheight();

  PelUnitBuf* pBuf = &origBuf;

  //  Search key pattern initialization
  CPelBuf  tmpPattern = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;
  PelBuf tmpOrgLuma;
#if JVET_AA0070_RRIBC
  const CompArea &area = pu.blocks[COMPONENT_Y];
  CompArea        tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
  if ((pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
  {
    tmpOrgLuma = m_tmpStorageLCU.getBuf(tmpArea);
    tmpOrgLuma.rspSignal(tmpPattern, m_pcReshape->getFwdLUT());
    pcPatternKey = (CPelBuf *) &tmpOrgLuma;
  }

  CPelBuf   *pcPatternKeyFlipH;
  CPelBuf   *pcPatternKeyFlipV;
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  PelStorage m_tmpStorageCUflipH;
  PelStorage m_tmpStorageCUflipV;
#endif
  PelBuf tmpOrgLumaFlipH, tmpOrgLumaFlipV;

  if (numRribcType > 1)
  {
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    m_tmpStorageCUflipH.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#endif
    tmpOrgLumaFlipH = m_tmpStorageCUflipH.getBuf(tmpArea);

#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    m_tmpStorageCUflipV.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
#endif
    tmpOrgLumaFlipV = m_tmpStorageCUflipV.getBuf(tmpArea);

    if ((pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      tmpOrgLumaFlipH.copyFrom(tmpOrgLuma);
      tmpOrgLumaFlipV.copyFrom(tmpOrgLuma);
    }
    else
    {
      tmpOrgLumaFlipH.copyFrom(tmpPattern);
      tmpOrgLumaFlipV.copyFrom(tmpPattern);
    }
    tmpOrgLumaFlipH.flipSignal(true);
    pcPatternKeyFlipH = (CPelBuf *) &tmpOrgLumaFlipH;

    tmpOrgLumaFlipV.flipSignal(false);
    pcPatternKeyFlipV = (CPelBuf *) &tmpOrgLumaFlipV;
  }
#else
  if ((pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
  {
    const CompArea &area = pu.blocks[COMPONENT_Y];
    CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
    tmpOrgLuma = m_tmpStorageLCU.getBuf(tmpArea);
    tmpOrgLuma.rspSignal( tmpPattern, m_pcReshape->getFwdLUT() );
    pcPatternKey = (CPelBuf*)&tmpOrgLuma;
  }
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
  int sadShift = 0;
#endif
#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.ibcCiipFlag && pu.interDir == 1)
#else
  if (pu.ibcCiipFlag)
#endif
  {
    const Pel *pelOrig = pcPatternKey->buf;
    Pel *pelIntra = ibcCiipIntraBuf.buf;
    int height = ibcCiipIntraBuf.height;
    int width = ibcCiipIntraBuf.width;
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        pelIntra[x] = (pelOrig[x] << 1) - pelIntra[x];
      }
      pelIntra += ibcCiipIntraBuf.stride;
      pelOrig += pcPatternKey->stride;
    }
    pcPatternKey = (CPelBuf*)&ibcCiipIntraBuf;
#if JVET_AE0169_BIPREDICTIVE_IBC
    sadShift = 1;
#endif
  }
#endif

  m_lumaClpRng = pu.cs->slice->clpRng(COMPONENT_Y);
  Picture* refPic = pu.cu->slice->getPic();
  const CPelBuf refBuf = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);

  IntTZSearchStruct cStruct;
#if !JVET_AA0070_RRIBC
  cStruct.pcPatternKey = pcPatternKey;
#endif
  cStruct.iRefStride = refBuf.stride;
  cStruct.piRefY = refBuf.buf;
  CHECK(pu.cu->imv == IMV_HPEL, "IF_IBC");
  cStruct.imvShift = pu.cu->imv << 1;
  cStruct.subShiftMode = 0; // used by intra pattern search function

#if JVET_AE0169_BIPREDICTIVE_IBC
  PelUnitBuf orgBufForAmvpMerge[IBC_MRG_MAX_NUM_CANDS];
  uint8_t checkAmvpMerge[IBC_MRG_MAX_NUM_CANDS] = {0, };
  int maxCheckMerge = 1;
  if (pu.interDir == 3)
  {
    maxCheckMerge = m_amvpMergeCtx.numValidMergeCand;
    for (int idx = 0; idx < maxCheckMerge; idx++)
    {
      if (m_amvpMergeCtx.rribcFlipTypes[idx])
      {
        continue;
      }
      int xPred = m_amvpMergeCtx.mvFieldNeighbours[idx<<1].mv.getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
      int yPred = m_amvpMergeCtx.mvFieldNeighbours[idx<<1].mv.getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
#if JVET_Z0084_IBC_TM
      if (PU::searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#else
      if (searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#endif
      {
        orgBufForAmvpMerge[idx] = PelUnitBuf(pu.chromaFormat, PelBuf(m_amvpMergeBuffer[idx], iRoiWidth, iRoiHeight));
        getBvpMergeOrgBuf(pu, idx, pcPatternKey, refBuf, ibcCiipIntraBuf, orgBufForAmvpMerge[idx]);
        checkAmvpMerge[idx] = 1;
      }
    }
    sadShift = pu.ibcCiipFlag ? 2 : 1;
  }
  else
  {
    checkAmvpMerge[0] = 1;
  }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  int sadAdd = sadShift > 0 ? (1<<(sadShift-1)) : 0;
#endif

  // disable weighted prediction
  setWpScalingDistParam(-1, REF_PIC_LIST_X, pu.cs->slice);

  m_pcRdCost->getMotionCost(0);
  m_pcRdCost->setPredictors(pcMvPred);
  m_pcRdCost->setCostScale(0);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_pcRdCost->setFullPelImvForZeroBvd(!pu.cs->sps->getIBCFracFlag());
#endif

  m_cDistParam.useMR = false;
#if JVET_AC0112_IBC_LIC
#if JVET_AE0159_FIBC
  PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
  if (!pu.cs->sps->getUseIbcFilter())
  {
    m_cDistParam.useMR = pu.cu->ibcLicFlag ? true : false ;
  }
#else
  m_cDistParam.useMR = pu.cu->ibcLicFlag;
#endif
#endif
#if !JVET_AA0070_RRIBC
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#endif
  bool buffered = false;
#if JVET_AA0070_RRIBC
  minCostProj = MAX_INT;
  if (m_pcEncCfg->getIntraPeriod() < 1 && m_pcEncCfg->getIBCFastMethod())
  {
    ruiCost  = MAX_UINT;
    std::unordered_map<Mv, Distortion> &history = m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord;
    if (history.size())
    {
      std::vector<std::pair<Mv, Distortion>> historyOrdered(history.size());
      int                                    i = 0;
      for (std::unordered_map<Mv, Distortion>::iterator p = history.begin(); p != history.end(); p++)
      {
        historyOrdered[i] = { p->first, p->second };
        i++;
      }
     std::stable_sort(historyOrdered.begin(), historyOrdered.end(),[](const std::pair<Mv, Distortion> &l, const std::pair<Mv, Distortion> &r)
                     { return (l.second < r.second) || (l.second == r.second && (l.first.getHor() < r.first.getHor() || (l.first.getHor() == r.first.getHor() && l.first.getVer() < r.first.getVer()))); });
     minCostProj = 3 * historyOrdered[0].second;

      Distortion cand0Cost[3] = { MAX_UINT, MAX_UINT, MAX_UINT };
      bool       isCalc[3] = { false }, isSkip[3] = { false }, checkDone[3] = { false };
      int        map[6] = { 1, 2, 0, 2, 0, 1 };
      for (int i = 0; i < history.size(); i++)
      {
        if (historyOrdered[i].second >= minCostProj)
        {
          break;
        }
        const Mv &bv  = historyOrdered[i].first;
        int       xBv = bv.hor;
        int       yBv = bv.ver;

        for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
        {
          if (isSkip[rribcFlipType])
          {
            continue;
          }
          if (!checkDone[rribcFlipType] && isCalc[rribcFlipType] && (isCalc[map[rribcFlipType * 2]] || isCalc[map[rribcFlipType * 2 + 1]]))
          {
            if (isCalc[map[rribcFlipType * 2]] && cand0Cost[rribcFlipType] >= 1.1 * cand0Cost[map[rribcFlipType * 2]])
            {
              isSkip[rribcFlipType] = true;
            }
            else if (isCalc[map[rribcFlipType * 2]] && cand0Cost[rribcFlipType] >= 1.1 * cand0Cost[map[rribcFlipType * 2 + 1]])
            {
              isSkip[rribcFlipType] = true;
            }
            checkDone[rribcFlipType] = true;
          }
          if ((rribcFlipType == 1 && yBv != 0) || (rribcFlipType == 2 && xBv != 0))
          {
            continue;
          }
#if JVET_Z0084_IBC_TM
          if (PU::searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
#else
          if (searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
#endif
          {
            buffered = true;
#if JVET_AE0169_BIPREDICTIVE_IBC
            for (int mergeIdx = 0; mergeIdx < maxCheckMerge; mergeIdx++)
            {
              if (!checkAmvpMerge[mergeIdx])
              {
                continue;
              }
              pu.amvpMergeModeFlag[REF_PIC_LIST_1] = (pu.interDir == 3);
              if ((pu.amvpMergeModeFlag[REF_PIC_LIST_1] && ((xBv<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getHor()) &&
                ((yBv<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getVer())))
              {
                continue;
              }
#endif
            Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xBv, yBv, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#if JVET_AE0169_BIPREDICTIVE_IBC
            sad += m_pcRdCost->getBvpMergeCost(pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? mergeIdx : 0);
            cStruct.pcPatternKey = pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? (CPelBuf*)&orgBufForAmvpMerge[mergeIdx].Y() : (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#else
            cStruct.pcPatternKey = (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#endif
            m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
            m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
#if JVET_AE0169_BIPREDICTIVE_IBC
            sad += ((m_cDistParam.distFunc(m_cDistParam)+sadAdd)>>sadShift);
#else
            sad += m_cDistParam.distFunc(m_cDistParam);
#endif
            if (!isCalc[rribcFlipType])
            {
              cand0Cost[rribcFlipType] = sad;
              isCalc[rribcFlipType]    = true;
            }
            if (sad < ruiCost)
            {
              rcMv                 = bv;
              ruiCost              = sad;
              pu.cu->rribcFlipType = rribcFlipType;
#if JVET_AE0169_BIPREDICTIVE_IBC
              pu.mergeIdx = mergeIdx;
#endif
            }
            else if (sad == ruiCost)
            {
              // stabilise the search through the unordered list
              if (abs(bv.hor) < abs(rcMv.getHor()) || (bv.hor == rcMv.getHor() && abs(bv.ver) < abs(rcMv.getVer())))
              {
                // update the vector.
                rcMv                 = bv;
                pu.cu->rribcFlipType = rribcFlipType;
#if JVET_AE0169_BIPREDICTIVE_IBC
                pu.mergeIdx = mergeIdx;
#endif
              }
            }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (pu.cs->sps->getIBCFracFlag())
            {
#if JVET_AE0169_BIPREDICTIVE_IBC
              m_bestSrchCostIntBv.insert(sad, xBv, yBv, (pu.interDir == 3) ? mergeIdx : MAX_UCHAR, rribcFlipType);
#else
              m_bestSrchCostIntBv.insert(sad, xBv, yBv, rribcFlipType);
#endif
            }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
            }
#endif
          }
        }
      }

      if (buffered)
      {
        Mv  cMvPredEncOnly[IBC_NUM_CANDIDATES];
        int nbPreds = 0;
        PU::getIbcMVPsEncOnly(pu, cMvPredEncOnly, nbPreds);

        for (unsigned int cand = 0; cand < nbPreds; cand++)
        {
          int xPred = cMvPredEncOnly[cand].getHor();
          int yPred = cMvPredEncOnly[cand].getVer();

          for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
          {
            if (isSkip[rribcFlipType])
            {
              continue;
            }
            if ((rribcFlipType == 1 && yPred != 0) || (rribcFlipType == 2 && xPred != 0))
            {
              continue;
            }
#if JVET_Z0084_IBC_TM
            if (PU::searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#else
            if (searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#endif
            {
#if JVET_AE0169_BIPREDICTIVE_IBC
              for (int mergeIdx = 0; mergeIdx < maxCheckMerge; mergeIdx++)
              {
                if (!checkAmvpMerge[mergeIdx])
                {
                  continue;
                }
                pu.amvpMergeModeFlag[REF_PIC_LIST_1] = (pu.interDir == 3);
                if ((pu.amvpMergeModeFlag[REF_PIC_LIST_1] && ((xPred<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getHor()) &&
                  ((yPred<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getVer())))
                {
                  continue;
                }
#endif
              Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#if JVET_AE0169_BIPREDICTIVE_IBC
              sad += m_pcRdCost->getBvpMergeCost(pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? mergeIdx : 0);
              cStruct.pcPatternKey = pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? (CPelBuf*)&orgBufForAmvpMerge[mergeIdx].Y() : (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#else
              cStruct.pcPatternKey = (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#endif
              m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
              m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
#if JVET_AE0169_BIPREDICTIVE_IBC
              sad += ((m_cDistParam.distFunc(m_cDistParam)+sadAdd)>>sadShift);
#else
              sad += m_cDistParam.distFunc(m_cDistParam);
#endif
              if (sad >= minCostProj)
              {
                continue;
              }
              if (sad < ruiCost)
              {
                rcMv.set(xPred, yPred);
                ruiCost              = sad;
                pu.cu->rribcFlipType = rribcFlipType;
#if JVET_AE0169_BIPREDICTIVE_IBC
                pu.mergeIdx = mergeIdx;
#endif
              }
              else if (sad == ruiCost)
              {
                // stabilise the search through the unordered list
                if (abs(xPred) < abs(rcMv.getHor()) || (xPred == rcMv.getHor() && abs(yPred) < abs(rcMv.getVer())))
                {
                  // update the vector.
                  rcMv.set(xPred, yPred);
                  pu.cu->rribcFlipType = rribcFlipType;
#if JVET_AE0169_BIPREDICTIVE_IBC
                  pu.mergeIdx = mergeIdx;
#endif
                }
              }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              if (pu.cs->sps->getIBCFracFlag())
              {
#if JVET_AE0169_BIPREDICTIVE_IBC
                m_bestSrchCostIntBv.insert(sad, xPred, yPred, (pu.interDir == 3) ? mergeIdx : MAX_UCHAR, rribcFlipType);
#else
                m_bestSrchCostIntBv.insert(sad, xPred, yPred, rribcFlipType);
#endif
              }
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
              if (pu.interDir == 1)
#endif
              m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord[Mv(xPred, yPred)] = sad;
#if JVET_AE0169_BIPREDICTIVE_IBC
              }
#endif
            }
          }
        }
      }
    }
  }
  else if (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_BUFFERBV)
#else
  if (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_BUFFERBV)
#endif
  {
    ruiCost = MAX_UINT;
    std::unordered_map<Mv, Distortion>& history = m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord;
#if JVET_AA0070_RRIBC
    std::vector<std::pair<Mv, Distortion>> historyOrdered(history.size());
    int                                    i = 0;
    for (std::unordered_map<Mv, Distortion>::iterator p = history.begin(); p != history.end(); p++)
    {
      historyOrdered[i] = { p->first, p->second };
      i++;
    }
    std::stable_sort(historyOrdered.begin(), historyOrdered.end(),[](const std::pair<Mv, Distortion> &l, const std::pair<Mv, Distortion> &r)
                     { return (l.second < r.second) || (l.second == r.second && (l.first.getHor() < r.first.getHor() || (l.first.getHor() == r.first.getHor() && l.first.getVer() < r.first.getVer()))); });

    for (int i = 0; i < history.size(); i++)
    {
        const Mv &bv  = historyOrdered[i].first;
#else
    for (std::unordered_map<Mv, Distortion>::iterator p = history.begin(); p != history.end(); p++)
    {
      const Mv& bv = p->first;
#endif

      int xBv = bv.hor;
      int yBv = bv.ver;
#if JVET_Z0084_IBC_TM
      if (PU::searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
#else
      if (searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
#endif
      {
#if JVET_AA0070_RRIBC
        for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
        {
          if ((rribcFlipType == 1 && yBv != 0) || (rribcFlipType == 2 && xBv != 0))
          {
            continue;
          }
          buffered = true;
#if JVET_AE0169_BIPREDICTIVE_IBC
          for (int mergeIdx = 0; mergeIdx < maxCheckMerge; mergeIdx++)
          {
            if (!checkAmvpMerge[mergeIdx])
            {
              continue;
            }
            pu.amvpMergeModeFlag[REF_PIC_LIST_1] = (pu.interDir == 3);
            if ((pu.amvpMergeModeFlag[REF_PIC_LIST_1] && ((xBv<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getHor()) &&
              ((yBv<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getVer())))
            {
              continue;
            }
#endif
          Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xBv, yBv, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#if JVET_AE0169_BIPREDICTIVE_IBC
          sad += m_pcRdCost->getBvpMergeCost(pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? mergeIdx : 0);
          cStruct.pcPatternKey = pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? (CPelBuf*)&orgBufForAmvpMerge[mergeIdx].Y() : (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#else
          cStruct.pcPatternKey = (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#endif
          m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#else
        buffered = true;
        Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xBv, yBv, pu.cs->sps->getAMVREnabledFlag());
#endif
#if JVET_AE0159_FIBC
        if (pu.cs->sps->getUseIbcFilter())
        {
          if (pu.cu->ibcLicFlag)
          {
            pu.mv[0] = Mv((xBv << MV_FRACTIONAL_BITS_INTERNAL), (yBv << MV_FRACTIONAL_BITS_INTERNAL));
            getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predBuf, false);
            m_cDistParam.cur.buf = predBuf.Y().buf;
            m_cDistParam.cur.stride = predBuf.Y().stride;
          }
          else
          {
            m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
            m_cDistParam.cur.stride = cStruct.iRefStride;
          }
        }
        else
        {
          m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
        }
#else
        m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
        sad += ((m_cDistParam.distFunc(m_cDistParam)+sadAdd)>>sadShift);
#else
        sad += m_cDistParam.distFunc(m_cDistParam);
#endif
        if (sad < ruiCost)
        {
          rcMv = bv;
          ruiCost = sad;
#if JVET_AA0070_RRIBC
          pu.cu->rribcFlipType = rribcFlipType;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
          pu.mergeIdx = mergeIdx;
#endif
        }
        else if (sad == ruiCost)
        {
          // stabilise the search through the unordered list
#if JVET_AA0070_RRIBC
          if (abs(bv.hor) < abs(rcMv.getHor()) || (bv.hor == rcMv.getHor() && abs(bv.ver) < abs(rcMv.getVer())))
          {
            // update the vector.
            rcMv                 = bv;
            pu.cu->rribcFlipType = rribcFlipType;
#if JVET_AE0169_BIPREDICTIVE_IBC
            pu.mergeIdx = mergeIdx;
#endif
          }
        }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        if (pu.cs->sps->getIBCFracFlag())
        {
#if JVET_AE0169_BIPREDICTIVE_IBC
          m_bestSrchCostIntBv.insert(sad, xBv, yBv, (pu.interDir == 3) ? mergeIdx : MAX_UCHAR, rribcFlipType);
#else
          m_bestSrchCostIntBv.insert(sad, xBv, yBv, rribcFlipType);
#endif
        }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
          }
#endif
        }
      }
#else
          if (bv.hor < rcMv.getHor() || (bv.hor == rcMv.getHor() && bv.ver < rcMv.getVer()))
          {
            // update the vector.
            rcMv = bv;
#if JVET_AE0169_BIPREDICTIVE_IBC
            pu.mergeIdx = mergeIdx;
#endif
          }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          if (pu.cs->sps->getIBCFracFlag())
          {
#if JVET_AE0169_BIPREDICTIVE_IBC
            m_bestSrchCostIntBv.insert(sad, xBv, yBv, (pu.interDir == 3) ? mergeIdx : MAX_UCHAR);
#else
            m_bestSrchCostIntBv.insert(sad, xBv, yBv);
#endif
          }
#endif
        }
#if JVET_AE0169_BIPREDICTIVE_IBC
          }
#endif
      }
#endif
    }

    if (buffered)
    {
      Mv cMvPredEncOnly[IBC_NUM_CANDIDATES];
      int nbPreds = 0;
      PU::getIbcMVPsEncOnly(pu, cMvPredEncOnly, nbPreds);

      for (unsigned int cand = 0; cand < nbPreds; cand++)
      {
        int xPred = cMvPredEncOnly[cand].getHor();
        int yPred = cMvPredEncOnly[cand].getVer();

#if JVET_Z0084_IBC_TM
        if (PU::searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#else
        if (searchBv(pu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
#endif
        {
#if JVET_AA0070_RRIBC
          for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
          {
            if ((rribcFlipType == 1 && yPred != 0) || (rribcFlipType == 2 && xPred != 0))
            {
              continue;
            }
#if JVET_AE0169_BIPREDICTIVE_IBC
            for (int mergeIdx = 0; mergeIdx < maxCheckMerge; mergeIdx++)
            {
              if (!checkAmvpMerge[mergeIdx])
              {
                continue;
              }
              pu.amvpMergeModeFlag[REF_PIC_LIST_1] = (pu.interDir == 3);
              if ((pu.amvpMergeModeFlag[REF_PIC_LIST_1] && ((xPred<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getHor()) &&
                ((yPred<<MV_FRACTIONAL_BITS_INTERNAL) == m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv.getVer())))
              {
                continue;
              }
#endif
            Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag(), rribcFlipType);
#if JVET_AE0169_BIPREDICTIVE_IBC
            sad += m_pcRdCost->getBvpMergeCost(pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? mergeIdx : 0);
            cStruct.pcPatternKey = pu.amvpMergeModeFlag[REF_PIC_LIST_1] ? (CPelBuf*)&orgBufForAmvpMerge[mergeIdx].Y() : (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#else
            cStruct.pcPatternKey = (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
#endif
            m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#else
          Distortion sad = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, pu.cs->sps->getAMVREnabledFlag());
#endif
#if JVET_AE0159_FIBC
          if (pu.cs->sps->getUseIbcFilter())
          {
            if (pu.cu->ibcLicFlag)
            {
              pu.mv[0] = Mv((xPred << MV_FRACTIONAL_BITS_INTERNAL), (yPred << MV_FRACTIONAL_BITS_INTERNAL));
              getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predBuf, false);
              m_cDistParam.cur.buf = predBuf.Y().buf;
              m_cDistParam.cur.stride = predBuf.Y().stride;
            }
            else
            {
              m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
              m_cDistParam.cur.stride = cStruct.iRefStride;
            }
          }
          else
          {
            m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
          }
#else
          m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
          sad += ((m_cDistParam.distFunc(m_cDistParam)+sadAdd)>>sadShift);
#else
          sad += m_cDistParam.distFunc(m_cDistParam);
#endif
          if (sad < ruiCost)
          {
            rcMv.set(xPred, yPred);
            ruiCost = sad;
#if JVET_AA0070_RRIBC
            pu.cu->rribcFlipType = rribcFlipType;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
            pu.mergeIdx = mergeIdx;
#endif
          }
          else if (sad == ruiCost)
          {
            // stabilise the search through the unordered list
            if (xPred < rcMv.getHor() || (xPred == rcMv.getHor() && yPred < rcMv.getVer()))
            {
              // update the vector.
              rcMv.set(xPred, yPred);
#if JVET_AA0070_RRIBC
              pu.cu->rribcFlipType = rribcFlipType;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
              pu.mergeIdx = mergeIdx;
#endif
            }
          }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          if (pu.cs->sps->getIBCFracFlag())
          {
            m_bestSrchCostIntBv.insert(sad, xPred, yPred
#if JVET_AE0169_BIPREDICTIVE_IBC
                                     , (pu.interDir == 3) ? mergeIdx : MAX_UCHAR
#endif
#if JVET_AA0070_RRIBC
                                     , rribcFlipType
#endif
            );
          }
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
          if (pu.interDir == 1)
#endif
          m_ctuRecord[pu.lumaPos()][pu.lumaSize()].bvRecord[Mv(xPred, yPred)] = sad;
#if JVET_AE0169_BIPREDICTIVE_IBC
            }
#endif
#if JVET_AA0070_RRIBC
          }
#endif
        }
      }
    }
  }

  if (!buffered)
  {
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3)
    {
      rcMv.set(0, 0);
      return;
    }
    pu.cu->imv = IMV_FPEL;
#endif
    Mv        cMvSrchRngLT;
    Mv        cMvSrchRngRB;

    // assume that intra BV is integer-pel precision
    xSetIntraSearchRange(pu, pu.lwidth(), pu.lheight(), localSearchRangeX, localSearchRangeY, cMvSrchRngLT, cMvSrchRngRB);

    //  Do integer search
#if JVET_AA0070_RRIBC
    Distortion minCost = MAX_UINT;
    Mv         tempMv;
    tempMv.setZero();
    for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
    {
      cStruct.pcPatternKey = (rribcFlipType == 0) ? pcPatternKey : ((rribcFlipType == 1) ? pcPatternKeyFlipH : pcPatternKeyFlipV);
      m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC && !rribcFlipType)
      {
        xIntraPatternSearchOpt(pu, cStruct, tempMv, ruiCost, &cMvSrchRngLT, &cMvSrchRngRB, pcMvPred[rribcFlipType], rribcFlipType);
      }
      else
#endif
      xIntraPatternSearch(pu, cStruct, tempMv, ruiCost, &cMvSrchRngLT, &cMvSrchRngRB, pcMvPred[rribcFlipType], rribcFlipType);
      if (ruiCost < minCost)
      {
        minCost = ruiCost;
        rcMv.set(tempMv.getHor(), tempMv.getVer());
        pu.cu->rribcFlipType = rribcFlipType;
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    ruiCost = minCost;
#endif
  }
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (numRribcType > 1)
  {
    m_tmpStorageCUflipH.destroy();
    m_tmpStorageCUflipV.destroy();
  }
#endif
#else
    xIntraPatternSearch(pu, cStruct, rcMv, ruiCost, &cMvSrchRngLT, &cMvSrchRngRB, pcMvPred);
  }
#endif
}

#if JVET_AE0169_BIPREDICTIVE_IBC
void InterSearch::getBvpMergeOrgBuf(const PredictionUnit& pu, int mergeIdx, CPelBuf* pcPatternKey, const CPelBuf& refBuf, const PelBuf& ibcCiipIntraBuf, PelUnitBuf& orgBufForAmvpMerge)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  auto isFracBv = [&pu](Mv mv)
  {
    return pu.cs->sps->getIBCFracFlag()
       && ((mv.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0 || (mv.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0);
  };
#endif

  int                 iRoiWidth = pu.lwidth();
  int                 iRoiHeight = pu.lheight();
  const Pel *pelOrig = pcPatternKey->buf;
  Mv& _mv = m_amvpMergeCtx.mvFieldNeighbours[mergeIdx<<1].mv;
  int xPred = _mv.getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
  int yPred = _mv.getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
  const Pel *pelMerge = refBuf.buf + yPred * refBuf.stride + xPred;
  Pel* pelAmvpMerge = orgBufForAmvpMerge.Y().buf;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool foundFracBV = isFracBv(_mv);
  if (foundFracBV)
  {
#if JVET_AC0112_IBC_LIC && JVET_AC0112_IBC_CIIP
    m_storeBeforeLIC = false;
#endif
    getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), _mv, orgBufForAmvpMerge);
    pelMerge = orgBufForAmvpMerge.Y().buf;
  }
  int mergeStride = foundFracBV ? iRoiWidth : refBuf.stride;
#else
  int mergeStride = refBuf.stride;
#endif
  if (pu.ibcCiipFlag)
  {
    Pel *pelIntra = ibcCiipIntraBuf.buf;
    for( int y = 0; y < iRoiHeight; y++ )
    {
      for( int x = 0; x < iRoiWidth; x++ )
      {
        pelAmvpMerge[x] = (pelOrig[x] << 2) - (pelIntra[x] << 1) - pelMerge[x];
      }
      pelAmvpMerge += iRoiWidth;
      pelOrig += pcPatternKey->stride;
      pelIntra += ibcCiipIntraBuf.stride;
      pelMerge += mergeStride;
    }
  }
  else
  {
    for( int y = 0; y < iRoiHeight; y++ )
    {
      for( int x = 0; x < iRoiWidth; x++ )
      {
        pelAmvpMerge[x] = (pelOrig[x] << 1) - pelMerge[x];
      }
      pelAmvpMerge += iRoiWidth;
      pelOrig += pcPatternKey->stride;
      pelMerge += mergeStride;
    }
  }
}
#endif

// based on xSetSearchRange
void InterSearch::xSetIntraSearchRange(PredictionUnit& pu, int iRoiWidth, int iRoiHeight, const int localSearchRangeX, const int localSearchRangeY, Mv& rcMvSrchRngLT, Mv& rcMvSrchRngRB)
{
  const SPS &sps = *pu.cs->sps;

  int srLeft, srRight, srTop, srBottom;

  const int cuPelX = pu.Y().x;
  const int cuPelY = pu.Y().y;

  const int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
#if JVET_Z0153_IBC_EXT_REF
  const int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();

  srLeft = -cuPelX;
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  srTop = -cuPelY;
#else
  srTop = - 2 * lcuWidth - (cuPelY % lcuWidth);
#if JVET_AA0106_IBCBUF_CTU256
  if (256 == lcuWidth)
  {
    srTop = -lcuWidth - (cuPelY % lcuWidth);
  }
#endif
#endif
  srRight = picWidth - cuPelX - iRoiWidth;
  srBottom = lcuWidth - (cuPelY % lcuWidth) - iRoiHeight;  
#else
  const int ctuSizeLog2 = floorLog2(lcuWidth);
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);

  srLeft = -(numLeftCTUs * lcuWidth + (cuPelX % lcuWidth));
  srTop = -(cuPelY % lcuWidth);

  srRight = lcuWidth - (cuPelX % lcuWidth) - iRoiWidth;
  srBottom = lcuWidth - (cuPelY % lcuWidth) - iRoiHeight;
#endif

  rcMvSrchRngLT.setHor(srLeft);
  rcMvSrchRngLT.setVer(srTop);
  rcMvSrchRngRB.setHor(srRight);
  rcMvSrchRngRB.setVer(srBottom);

  rcMvSrchRngLT <<= 2;
  rcMvSrchRngRB <<= 2;
  bool temp = m_clipMvInSubPic;
  m_clipMvInSubPic = true;
  xClipMv(rcMvSrchRngLT, pu.cu->lumaPos(),
         pu.cu->lumaSize(),
         sps
      , *pu.cs->pps
  );
  xClipMv(rcMvSrchRngRB, pu.cu->lumaPos(),
         pu.cu->lumaSize(),
         sps
      , *pu.cs->pps
  );
  m_clipMvInSubPic = temp;
  rcMvSrchRngLT >>= 2;
  rcMvSrchRngRB >>= 2;
}

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
bool InterSearch::predIBCSearch( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap
#if JVET_AC0112_IBC_LIC || JVET_AC0112_IBC_CIIP
                               , Distortion* bvSearchCost
#endif
#if JVET_AC0112_IBC_CIIP
                               , PelBuf* ibcCiipIntraBuf
#endif
#if JVET_AA0070_RRIBC
                               , bool isSecondPass
#endif
#if JVET_AC0112_IBC_LIC || JVET_AC0112_IBC_CIIP
                               , bool* searchedByHash
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_CIIP
                               , bool isCiipFirstPass
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
                               , double costTh
#endif
)
#else
#if JVET_AA0070_RRIBC
#if JVET_AC0112_IBC_CIIP
bool InterSearch::predIBCSearch( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap, Distortion* bvSearchCost, PelBuf* ibcCiipIntraBuf, bool isSecondPass, bool* searchedByHash)
#else
#if JVET_AC0112_IBC_LIC
bool InterSearch::predIBCSearch( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap, Distortion* bvSearchCost, bool isSecondPass, bool* searchedByHash)
#else
bool InterSearch::predIBCSearch( CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap, bool isSecondPass)
#endif
#endif
#else
#if JVET_AC0112_IBC_CIIP
bool InterSearch::predIBCSearch(CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap, Distortion* bvSearchCost, PelBuf* ibcCiipIntraBuf, bool* searchedByHash)
#else
#if JVET_AC0112_IBC_LIC
bool InterSearch::predIBCSearch(CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap, Distortion* bvSearchCost, bool* searchedByHash)
#else
bool InterSearch::predIBCSearch(CodingUnit& cu, Partitioner& partitioner, const int localSearchRangeX, const int localSearchRangeY, IbcHashMap& ibcHashMap)
#endif
#endif
#endif
#endif
{
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;

  Mv           cMv;
  Mv           cMvPred;

#if JVET_Z0131_IBC_BVD_BINARIZATION
  xEstBvdBitCosts(m_pcRdCost->getBvdBitCosts()
#if JVET_AE0169_BIPREDICTIVE_IBC
                , cu.firstPU->interDir == 3
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                , cu.cs->sps->getIBCFracFlag()
#if JVET_AA0070_RRIBC
                , cu.cs->sps->getIBCFracFlag() ? (int)DeriveCtx::CtxRribcFlipType(cu) : NOT_VALID
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                , cu.cs->sps->getIBCFracFlag() ? (int)DeriveCtx::CtxbvOneZeroComp(cu) : NOT_VALID
#endif
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                , cu.firstPU->isBvpClusterApplicable()
#endif
  );
#endif
#if JVET_AA0070_RRIBC
  CodedCUInfo& relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(partitioner.currArea());
#if JVET_AC0112_IBC_LIC
  if (isSecondPass && relatedCU.isRribcCoded && cu.ibcLicFlag)
  {
    return false;
  }
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  uint8_t imvForZeroMvd = (cu.cs->sps->getIBCFracFlag() ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : IMV_FPEL);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  cu.bvOneZeroComp = 0;
  cu.bvZeroCompDir = 0;
#endif
#endif

  for (auto &pu : CU::traversePUs(cu))
  {
    m_maxCompIDToPred = MAX_NUM_COMPONENT;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    m_bestSrchCostIntBv.init(false, cu.cs->sps->getIBCFracFlag());
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
    bool bAmvpListDerived = false;
#if JVET_AC0112_IBC_LIC
    bAmvpListDerived |= pu.cu->ibcLicFlag;
#endif
#if JVET_AC0112_IBC_CIIP
    bAmvpListDerived |= pu.ibcCiipFlag;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    bAmvpListDerived |= (pu.interDir == 3);
#endif
#endif
#endif

    CHECK(pu.cu != &cu, "PU is contained in another CU");
#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    int numRribcType = (cu.cs->sps->getUseRRIbc()) ? 3 : 1;
#else
    int numRribcType = 3;
#endif
    if (isSecondPass)
    {
      if (!relatedCU.isRribcCoded)
      {
        numRribcType = 1;
      }
    }

#if JVET_AC0112_IBC_CIIP
    if (pu.ibcCiipFlag)
    {
      numRribcType = 1;
    }
#endif
#if JVET_AC0112_IBC_LIC
    if (pu.cu->ibcLicFlag)
    {
      numRribcType = 1;
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3)
    {
      numRribcType = 1;
    }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    int numBvTypes = numRribcType;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    numBvTypes = std::max(numBvTypes, (int)(cu.cs->sps->getIBCFracFlag() && pu.isBvpClusterApplicable() ? 3 : 1));
#endif
#endif

    Mv       cMv;
    cMv.setZero();
    int iBvpNum    = 2;
    int bvpIdxBest = 0;
    Distortion cost = 0;

    AMVPInfo amvpInfo4Pel[3];
    AMVPInfo amvpInfo[3];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    AMVPInfo  amvpInfoQPel[3];
    AMVPInfo  amvpInfoHPel[3];
    AMVPInfo* amvpInfoList[NUM_IMV_MODES] = { amvpInfoQPel, amvpInfo, amvpInfo4Pel, amvpInfoHPel };
#endif
    Mv       cMvPred[3][2];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    for (int i = 0; i < numBvTypes; i++)
#else
    for (int i = 0; i < numRribcType; i++)
#endif
    {
      pu.cu->rribcFlipType = i;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
      if (!bAmvpListDerived)
      {
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      MergeCtx mergeCtx;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (cu.cs->sps->getIBCFracFlag() && i == 0)
      {
        // prepare imv = 0 accuracy predictor info
        pu.cu->imv = IMV_OFF;
        PU::fillIBCMvpCand(pu, amvpInfoQPel[i]
#if JVET_AE0169_BIPREDICTIVE_IBC
                         , i == 0 ? m_amvpMergeCtx : mergeCtx
#endif
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                         , this
#endif
        );
      }
#endif

      // prepare imv = 2 accuracy predictor info
      pu.cu->imv      = 2;
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_LIC
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag && pu.interDir == 1)
#else
      if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag)
#endif
      {
#else
      if (!pu.ibcCiipFlag)
      {
#endif
#else
#if JVET_AC0112_IBC_LIC
      if (!pu.cu->ibcLicFlag)
      {
#endif
#endif
#endif
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AE0169_BIPREDICTIVE_IBC
      PU::fillIBCMvpCand(pu, amvpInfo4Pel[i], mergeCtx, this);
#else
      PU::fillIBCMvpCand(pu, amvpInfo4Pel[i], this);
#endif
#else
      PU::fillIBCMvpCand(pu, amvpInfo4Pel[i]);
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      m_amvpInfo4Pel[i] = amvpInfo4Pel[i];
      }
      else
      {
        amvpInfo4Pel[i] = m_amvpInfo4Pel[i];
      }
#endif
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      // prepare imv = 1 accuracy predictor info
      pu.cu->imv = 1;
#else
      // prepare imv = 0 accuracy predictor info
      pu.cu->imv = 0;
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_LIC
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag && pu.interDir == 1)
#else
      if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag)
#endif
      {
#else
      if (!pu.ibcCiipFlag)
      {
#endif
#else
#if JVET_AC0112_IBC_LIC
      if (!pu.cu->ibcLicFlag)
      {
#endif
#endif
#endif
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      PU::fillIBCMvpCand(pu, amvpInfo[i], (!pu.cs->sps->getIBCFracFlag() && i == 0) ? m_amvpMergeCtx : mergeCtx, this);
#else
      PU::fillIBCMvpCand(pu, amvpInfo[i], i == 0 ? m_amvpMergeCtx : mergeCtx, this);
#endif
#else
      PU::fillIBCMvpCand(pu, amvpInfo[i], this);
#endif
#else
      PU::fillIBCMvpCand(pu, amvpInfo[i]);
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      m_amvpInfo[i] = amvpInfo[i];
      }
      else
      {
        amvpInfo[i] = m_amvpInfo[i];
      }
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
      }

      if (bAmvpListDerived)
      {
        if (cu.cs->sps->getIBCFracFlag())
        {
          amvpInfoQPel[i] = m_amvpInfoQPel[i];
          amvpInfoHPel[i] = m_amvpInfoHPel[i];
        }
        amvpInfo4Pel[i] = m_amvpInfo4Pel[i];
        amvpInfo    [i] = m_amvpInfo    [i];
        pu.cu->imv = 1;
      }
      else
      {
        if (cu.cs->sps->getIBCFracFlag())
        {
          m_amvpInfoQPel[i] = amvpInfoQPel[i];
          m_amvpInfoHPel[i] = amvpInfoHPel[i];
        }
        m_amvpInfo4Pel[i] = amvpInfo4Pel[i];
        m_amvpInfo    [i] = amvpInfo    [i];
      }
#endif

      // store in full pel accuracy, shift before use in search
      cMvPred[i][0] = amvpInfo[i].mvCand[0];
      cMvPred[i][0].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      cMvPred[i][1] = amvpInfo[i].mvCand[1];
      cMvPred[i][1].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
      {
        iBvpNum    = 1;
        cMvPred[i][1] = cMvPred[i][0];
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC && JVET_Y0058_IBC_LIST_MODIFY && JVET_W0090_ARMC_TM
    if (pu.cs->slice->getBiPredictionIBCFlag() && !pu.ibcCiipFlag && !pu.cu->ibcLicFlag && pu.interDir == 1 && m_amvpMergeCtx.numValidMergeCand > 0)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (!pu.isBvpClusterApplicable() || !cu.cs->sps->getIBCFracFlag())
      {
        pu.cu->rribcFlipType = 0;
        pu.mergeFlag = false;
        PU::getIBCMergeCandidates(pu, m_amvpMergeCtx);
      }
#endif
      if (pu.cs->sps->getUseAML())
      {
#if JVET_Z0075_IBC_HMVP_ENLARGE
        adjustIBCMergeCandidates(pu, m_amvpMergeCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#else
        pcInter->adjustIBCMergeCandidates(pu, mergeCtx);
#endif
      }
      adjustIbcMergeRribcCand(pu, m_amvpMergeCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
      pu.interDir = 1;
      pu.mergeFlag = false;
      pu.cu->ibcLicFlag = 0;
#if JVET_AE0159_FIBC
      pu.cu->ibcFilterFlag = false;
#endif    
    }
#endif
    pu.cu->rribcFlipType = 0;
#else
    //////////////////////////////////////////////////////////
    /// ibc search
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    AMVPInfo  amvpInfo, amvpInfo4Pel, amvpInfoQPel, amvpInfoHPel;
    Mv cMv, cMvPred[2];

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
    if (!bAmvpListDerived)
    {
#endif
    if (cu.cs->sps->getIBCFracFlag())
    {
      pu.cu->imv = IMV_OFF;
      PU::fillIBCMvpCand(pu, amvpInfoQPel
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
                       , this
#endif
      );
    }
#endif

    pu.cu->imv = 2;
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    AMVPInfo amvpInfo4Pel;
#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_LIC
    if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag)
    {
#else
    if (!pu.ibcCiipFlag)
    {
#endif
#else
#if JVET_AC0112_IBC_LIC
    if (!pu.cu->ibcLicFlag)
    {
#endif
#endif
#endif
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    PU::fillIBCMvpCand(pu, amvpInfo4Pel, this);
#else
    PU::fillIBCMvpCand(pu, amvpInfo4Pel);
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      m_amvpInfo4Pel = amvpInfo4Pel;
    }
    else
    {
      amvpInfo4Pel = m_amvpInfo4Pel;
    }
#endif
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    pu.cu->imv = 1;
#else
    pu.cu->imv = 0;// (Int)cu.cs->sps->getUseIMV(); // set as IMV=0 initially
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    Mv    cMv, cMvPred[2];
    AMVPInfo amvpInfo;
#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_LIC
    if (!pu.ibcCiipFlag && !pu.cu->ibcLicFlag)
    {
#else
    if (!pu.ibcCiipFlag)
    {
#endif
#else
#if JVET_AC0112_IBC_LIC
    if (!pu.cu->ibcLicFlag)
    {
#endif
#endif
#endif
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    PU::fillIBCMvpCand(pu, amvpInfo, this);
#else
    PU::fillIBCMvpCand(pu, amvpInfo);
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      m_amvpInfo = amvpInfo;
    }
    else
    {
      amvpInfo = m_amvpInfo;
    }
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
    }

    if (bAmvpListDerived)
    {
      if (cu.cs->sps->getIBCFracFlag())
      {
        amvpInfoQPel = m_amvpInfoQPel;
        amvpInfoHPel = m_amvpInfoHPel;
      }
      amvpInfo4Pel = m_amvpInfo4Pel;
      amvpInfo     = m_amvpInfo;
      pu.cu->imv = 1;
    }
    else
    {
      if (cu.cs->sps->getIBCFracFlag())
      {
        m_amvpInfoQPel = amvpInfoQPel;
        m_amvpInfoHPel = amvpInfoHPel;
      }
      m_amvpInfo4Pel = amvpInfo4Pel;
      m_amvpInfo     = amvpInfo;
    }
#endif

    // store in full pel accuracy, shift before use in search
    cMvPred[0] = amvpInfo.mvCand[0];
    cMvPred[0].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    cMvPred[1] = amvpInfo.mvCand[1];
    cMvPred[1].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);

    int iBvpNum = 2;
    int bvpIdxBest = 0;
    cMv.setZero();
    Distortion cost = 0;
    if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
    {
      iBvpNum = 1;
      cMvPred[1] = cMvPred[0];
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    AMVPInfo* amvpInfoList[NUM_IMV_MODES] = { &amvpInfoQPel, &amvpInfo, &amvpInfo4Pel, &amvpInfoHPel };
#endif
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    pu.mergeIdx = MAX_UCHAR;
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->getIBCFracFlag());
#if JVET_AC0112_IBC_CIIP
    m_bestSrchCostIntBv.enableMultiCandSrch = cu.cs->sps->getIBCFracFlag() && pu.ibcCiipFlag ? false : m_bestSrchCostIntBv.enableMultiCandSrch;
#endif
#if JVET_AC0112_IBC_LIC
    m_bestSrchCostIntBv.enableMultiCandSrch = cu.cs->sps->getIBCFracFlag() && cu.ibcLicFlag && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) > 0 ? false : m_bestSrchCostIntBv.enableMultiCandSrch;
#endif
#endif

#if JVET_AC0112_IBC_CIIP
    const CompArea &area = pu.blocks[COMPONENT_Y];
    const UnitArea localUnitArea(area.chromaFormat, Area(0, 0, area.width, area.height));
    PelBuf ibcCiipIntraBuff = m_ibcCiipBuffer.getBuf(localUnitArea.Y());
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraBuff.copyFrom(*ibcCiipIntraBuf);
    }
#endif

#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (m_pcEncCfg->getIBCHashSearch() && !pu.ibcCiipFlag && (pu.interDir == 1))
#else
    if (m_pcEncCfg->getIBCHashSearch() && !pu.ibcCiipFlag)
#endif
#else
    if (m_pcEncCfg->getIBCHashSearch())
#endif
    {
#if JVET_AA0070_RRIBC
      xxIBCHashSearch(pu, cMvPred, iBvpNum, cMv, bvpIdxBest, ibcHashMap, amvpInfo4Pel, numRribcType);
#else
      xxIBCHashSearch(pu, cMvPred, iBvpNum, cMv, bvpIdxBest, ibcHashMap);
#endif
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      if (searchedByHash)
      {
        *searchedByHash = true;
      }
#endif
    }

    if (cMv.getHor() == 0 && cMv.getVer() == 0)
    {
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      if (searchedByHash)
      {
        *searchedByHash = false;
      }
#endif
      // if hash search does not work or is not enabled
      PelUnitBuf origBuf = pu.cs->getOrgBuf(pu);
#if JVET_AA0070_RRIBC
#if JVET_AC0112_IBC_CIIP
      xIBCEstimation(pu, origBuf, ibcCiipIntraBuff, cMvPred, cMv, cost, localSearchRangeX, localSearchRangeY, numRribcType);
#else
      xIBCEstimation(pu, origBuf, cMvPred, cMv, cost, localSearchRangeX, localSearchRangeY, numRribcType);
#endif
#else
#if JVET_AC0112_IBC_CIIP
      xIBCEstimation(pu, origBuf, ibcCiipIntraBuff, cMvPred, cMv, cost, localSearchRangeX, localSearchRangeY);
#else
      xIBCEstimation(pu, origBuf, cMvPred, cMv, cost, localSearchRangeX, localSearchRangeY);
#endif
#endif
    }

    if (cMv.getHor() == 0 && cMv.getVer() == 0)
    {
      return false;
    }
    /// ibc search
    /////////////////////////////////////////////////////////
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->getIBCFracFlag());
#endif
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
    if (bvSearchCost)
    {
      *bvSearchCost = cost;
    }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_CIIP
    if (isCiipFirstPass)
    {
      return false;
    }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
    if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) && (((double)cost) > costTh))
    {
      return false;
    }
#endif
#if JVET_Z0131_IBC_BVD_BINARIZATION
    m_pcRdCost->setPredictors(cMvPred);
    m_pcRdCost->setCostScale(0);
#if JVET_AA0070_RRIBC
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    if (pu.isBvpClusterApplicable())
    {
#if !JVET_AC0112_IBC_CIIP && !JVET_AC0112_IBC_LIC
    Distortion initCost = 
#endif
        m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(),
                                           pu.cu->rribcFlipType, &pu.cu->imv, &bvpIdxBest, true,
                                           &amvpInfo4Pel[pu.cu->rribcFlipType], pu.isBvpClusterApplicable());

    if (pu.cu->rribcFlipType)
    {
      pu.cu->bvOneZeroComp = 1;
      pu.cu->bvZeroCompDir = pu.cu->rribcFlipType;
    }
#if !JVET_AC0112_IBC_CIIP && !JVET_AC0112_IBC_LIC
    else
    {
      getBestBvpBvOneZeroComp(pu, cMv, initCost, &bvpIdxBest , &amvpInfo[0], &amvpInfo4Pel[0]);
    }
#endif
    }
    else
    {
      m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(),
                                         pu.cu->rribcFlipType, &pu.cu->imv, &bvpIdxBest, true,
                                         &amvpInfo4Pel[pu.cu->rribcFlipType]);
    }
#else
    m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(),
                                       pu.cu->rribcFlipType, &pu.cu->imv, &bvpIdxBest, true,
                                       &amvpInfo4Pel[pu.cu->rribcFlipType]);
#endif
#else
    m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), pu.cu->rribcFlipType, &pu.cu->imv, &bvpIdxBest);
#endif
#else
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
    m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), &pu.cu->imv, &bvpIdxBest, true, &amvpInfo4Pel);
#else
    m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), &pu.cu->imv, &bvpIdxBest);
#endif
#endif
#else
    unsigned int bitsBVPBest, bitsBVPTemp;
    bitsBVPBest = MAX_INT;
    m_pcRdCost->setCostScale(0);

    for (int bvpIdxTemp = 0; bvpIdxTemp<iBvpNum; bvpIdxTemp++)
    {
#if JVET_AA0070_RRIBC
      Mv ccMvPred = cMvPred[pu.cu->rribcFlipType][bvpIdxTemp];
      m_pcRdCost->setPredictor(ccMvPred);
      bitsBVPTemp = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0, cu.rribcFlipType);
#else
      m_pcRdCost->setPredictor(cMvPred[bvpIdxTemp]);

      bitsBVPTemp = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0);
#endif

      if (bitsBVPTemp < bitsBVPBest)
      {
        bitsBVPBest = bitsBVPTemp;
        bvpIdxBest = bvpIdxTemp;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        pu.cu->imv = IMV_FPEL;
#else
#if JVET_AA0070_RRIBC
        bool diffMV = false;
        if ((pu.cu->rribcFlipType == 1 && cMv.getHor() != ccMvPred.getHor()) || (pu.cu->rribcFlipType == 2 && cMv.getVer() != ccMvPred.getVer()) || (pu.cu->rribcFlipType == 0 && cMv != ccMvPred))
        {
          diffMV = true;
        }
        if (cu.cs->sps->getAMVREnabledFlag() && diffMV)
#else
        if (cu.cs->sps->getAMVREnabledFlag() && cMv != cMvPred[bvpIdxTemp])
#endif
          pu.cu->imv = 1; // set as full-pel
        else
          pu.cu->imv = 0; // set as fractional-pel
#endif

      }

      unsigned int bitsBVPQP = MAX_UINT;


      Mv mvPredQuadPel;
      if ((cMv.getHor() % 4 == 0) && (cMv.getVer() % 4 == 0) && (pu.cs->sps->getAMVREnabledFlag()))
      {
#if JVET_AA0070_RRIBC
        mvPredQuadPel = amvpInfo4Pel[pu.cu->rribcFlipType].mvCand[bvpIdxTemp];
#else
        mvPredQuadPel = amvpInfo4Pel.mvCand[bvpIdxTemp];// cMvPred[bvpIdxTemp];

#endif

        mvPredQuadPel.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_4PEL);

        m_pcRdCost->setPredictor(mvPredQuadPel);

#if JVET_AA0070_RRIBC
        bitsBVPQP = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor() >> 2, cMv.getVer() >> 2, 0, cu.rribcFlipType);
#else
        bitsBVPQP = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor() >> 2, cMv.getVer() >> 2, 0);
#endif

      }
      mvPredQuadPel.changePrecision(MV_PRECISION_4PEL, MV_PRECISION_INT);
#if JVET_AA0070_RRIBC
      bool diffMV = false;
      if ((pu.cu->rribcFlipType == 1 && cMv.getHor() != mvPredQuadPel.getHor()) || (pu.cu->rribcFlipType == 2 && cMv.getVer() != mvPredQuadPel.getVer()) || (pu.cu->rribcFlipType == 0 && cMv != mvPredQuadPel))
      {
        diffMV = true;
      }
      if ((bitsBVPQP < bitsBVPBest) && diffMV)
#else
      if (bitsBVPQP < bitsBVPBest && cMv != mvPredQuadPel)
#endif
      {
        bitsBVPBest = bitsBVPQP;
        bvpIdxBest = bvpIdxTemp;

        if (cu.cs->sps->getAMVREnabledFlag())
          pu.cu->imv = 2; // set as quad-pel
      }

    }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    Mv curBestBv = cMv;
#endif
    pu.bv = cMv; // bv is always at integer accuracy
    cMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    pu.mv[REF_PIC_LIST_0] = cMv; // store in fractional pel accuracy
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir != 3)
    {
      pu.mergeIdx = MAX_UCHAR;
    }
#endif

    pu.mvpIdx[REF_PIC_LIST_0] = bvpIdxBest;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
    int intBvType = pu.getBvType();
    if (pu.cu->imv == IMV_4PEL && cMv != amvpInfoList[IMV_4PEL][intBvType].mvCand[bvpIdxBest])
    {
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfoList[IMV_4PEL][intBvType].mvCand[bvpIdxBest];
    }
    else
    {
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfoList[IMV_FPEL][intBvType].mvCand[bvpIdxBest];
    }
    pu.mvd[REF_PIC_LIST_0].regulateMv(intBvType);
#else
#if JVET_AA0070_RRIBC
    if (pu.cu->imv == 2 && cMv != amvpInfo4Pel[pu.cu->rribcFlipType].mvCand[bvpIdxBest])
    {
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfo4Pel[pu.cu->rribcFlipType].mvCand[bvpIdxBest];
    }
    else
    {
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfo[pu.cu->rribcFlipType].mvCand[bvpIdxBest];
    }
    if (pu.cu->rribcFlipType == 1)
    {
      pu.mvd[REF_PIC_LIST_0].setVer(0);
    }
    else if (pu.cu->rribcFlipType == 2)
    {
      pu.mvd[REF_PIC_LIST_0].setHor(0);
    }
#else
    if (pu.cu->imv == 2 && cMv != amvpInfo4Pel.mvCand[bvpIdxBest])
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfo4Pel.mvCand[bvpIdxBest];
    else
      pu.mvd[REF_PIC_LIST_0] = cMv - amvpInfo.mvCand[bvpIdxBest];
#endif
#endif

    if( pu.mvd[REF_PIC_LIST_0] == Mv( 0, 0 ) )
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
      pu.cu->imv = intBvType == 0 ? imvForZeroMvd : IMV_FPEL;
      pu.mv[0] = amvpInfoList[pu.cu->imv][intBvType].mvCand[bvpIdxBest];
      pu.mv[0].regulateMv(intBvType);
#else
      pu.cu->imv = imvForZeroMvd;
      pu.mv[0] = amvpInfoList[pu.cu->imv][0].mvCand[bvpIdxBest];
#endif

      pu.bv = pu.mv[0];
      pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // bv is always set at integer precision
#else
      pu.cu->imv = 0;
#endif
    }

    if( pu.cu->imv == 2 )
    {
      CHECKD( ( cMv.getHor() % 16 ) || ( cMv.getVer() % 16 ), "Wrong MV");
    }

#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if( cu.cs->sps->getAMVREnabledFlag() )
    {
      assert( pu.cu->imv > 0 || pu.mvd[REF_PIC_LIST_0] == Mv() );
    }
#endif

    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (pu.cs->sps->getIBCFracFlag()
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
      && intBvType == 0
#endif
      )
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      if (pu.isBvpClusterApplicable() && pu.cu->rribcFlipType)
      {
        pu.cu->bvOneZeroComp = 1;
        pu.cu->bvZeroCompDir = pu.cu->rribcFlipType;
      }
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
      int bvType = pu.getBvType();
#endif

      if (NOT_VALID == m_bestSrchCostIntBv.find(curBestBv.getHor(), curBestBv.getVer()
#if JVET_AE0169_BIPREDICTIVE_IBC
                                              , pu.mergeIdx
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
                                              , bvType
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
                                              , (pu.cu->rribcFlipType > 0)
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
                                              , pu.cu->ibcLicIdx
#endif
      ))
      {
        m_bestSrchCostIntBv.insert(0, curBestBv.getHor(), curBestBv.getVer()
#if JVET_AE0169_BIPREDICTIVE_IBC
                                 , pu.mergeIdx
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
                                 , bvType
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
                                 , (pu.cu->rribcFlipType > 0)
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
                                 , pu.cu->ibcLicIdx
#endif
        );
      }

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      Distortion fracCost =
#endif
      xPredIBCFracPelSearch(pu, m_bestSrchCostIntBv, amvpInfoList[IMV_OFF], amvpInfoList[IMV_HPEL], amvpInfoList[IMV_FPEL], amvpInfoList[IMV_4PEL]
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
                          , numBvTypes
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
                          , numRribcType > 1
#endif
      );
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      bvpIdxBest = pu.mvpIdx[REF_PIC_LIST_0];
#endif

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      if (bvSearchCost)
      {
        *bvSearchCost = fracCost;
      }
#endif
    }
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION 
    pu.mvsdIdx[REF_PIC_LIST_0] = 0;
    if (pu.isBvdPredApplicable() && pu.mvd[REF_PIC_LIST_0].isMvdPredApplicable())
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      pu.bvdSuffixInfo.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
      pu.bvdSuffixInfo.initPrefixes(pu.mvd[REF_PIC_LIST_0], pu.cu->imv, true);

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      static std::vector<Mv> cMvdDerivedVec;
      cMvdDerivedVec.resize(0);
#else
      std::vector<Mv> cMvdDerivedVec;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      int bvType = pu.getBvType();
#else
      int bvType = 0;
#endif
      Mv cMvPred2 = amvpInfoList[pu.cu->imv][bvType].mvCand[bvpIdxBest];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      cMvPred2.regulateMv(bvType);
#endif
#else
#if JVET_AA0070_RRIBC
      Mv cMvPred2 = ((pu.cu->imv == 2 && cMv != amvpInfo4Pel[pu.cu->rribcFlipType].mvCand[bvpIdxBest]) ? amvpInfo4Pel[pu.cu->rribcFlipType] : amvpInfo[pu.cu->rribcFlipType]).mvCand[bvpIdxBest];
      if (pu.cu->rribcFlipType == 1)
      {
        cMvPred2.setVer(0);
      }
      else if (pu.cu->rribcFlipType == 2)
      {
        cMvPred2.setHor(0);
      }
#else
      const Mv cMvPred2 = ((pu.cu->imv == 2 && cMv != amvpInfo4Pel.mvCand[bvpIdxBest]) ? amvpInfo4Pel : amvpInfo).mvCand[bvpIdxBest];
#endif
#endif

      const Mv cMvdKnownAtDecoder = pu.mvd[REF_PIC_LIST_0];// .getAbsMv();

      deriveBvdSignIBC(cMvPred2, cMvdKnownAtDecoder, pu, cMvdDerivedVec, pu.cu->imv);

      int idx = deriveMVSDIdxFromMVDTransIBC(pu.mvd[REF_PIC_LIST_0], cMvdDerivedVec, pu.bvdSuffixInfo);

      initOffsets(pu.mvd[REF_PIC_LIST_0], cMvdDerivedVec, pu.bvdSuffixInfo, pu.cu->imv);

      CHECK(idx == -1, "pu.mvsdIdx[REF_PIC_LIST_0] = -1");

      pu.mvsdIdx[REF_PIC_LIST_0] = idx;
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
    {
      m_amvpMergeCtx.setIbcL1Info(pu, pu.mergeIdx);
    }
#endif
  }

  return true;
}

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
template <int N>
Distortion InterSearch::xPredIBCFracPelSearch(PredictionUnit&              pu
                                            , SrchCostBv<N>&               intBvList
                                            , AMVPInfo*                    amvpInfoQPel
                                            , AMVPInfo*                    amvpInfoHPel
                                            , AMVPInfo*                    amvpInfoFPel
                                            , AMVPInfo*                    amvpInfo4Pel
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                            , int                          numBvTypes
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                            , bool                         testRrIBC
#endif
)
{
  if (intBvList.cnt == 0)
  {
    return std::numeric_limits<Distortion>::max();
  }
  intBvList.cnt = intBvList.enableMultiCandSrch ? intBvList.cnt : 1;

  AMVPInfo* amvpInfoList[NUM_IMV_MODES] = { amvpInfoQPel, amvpInfoFPel, amvpInfo4Pel, amvpInfoHPel };
#if JVET_AA0070_RRIBC && !JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bool testRrIBC = numBvTypes > 1;
#endif
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  {
    // Frac BV only for 2-D modes
    numBvTypes = 1;
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    testRrIBC = false;
#endif
  }
#endif

  // Get original samples
  PelUnitBuf origBuf = pu.cs->getOrgBuf(pu);
  const CompArea& area = pu.blocks[COMPONENT_Y];
  CompArea  tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());

#if JVET_AA0070_RRIBC
  PelBuf tmpOrgLuma[3];
#else
  PelBuf tmpOrgLuma[1];
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  PelBuf tmpOrgBvpMerge[N];
  if (pu.interDir == 3)
  {
    for (int i = 0; i < intBvList.cnt; ++i)
    {
      int idx = intBvList.mergeIdxList[i];
      tmpOrgBvpMerge[i] = PelBuf(m_amvpMergeBuffer[idx], pu.lwidth(), pu.lheight());
    }
    tmpOrgLuma[0] = tmpOrgBvpMerge[0];
  }
  else
#endif
#if JVET_AC0112_IBC_CIIP
  if (pu.ibcCiipFlag)
  {
    tmpOrgLuma[0] = m_ibcCiipBuffer.getBuf(tmpArea);
  }
  else
#endif
  {
    if ((pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      tmpOrgLuma[0] = m_tmpStorageLCU.getBuf(tmpArea);
      tmpOrgLuma[0].rspSignal(origBuf.Y(), m_pcReshape->getFwdLUT());
    }
    else
    {
      tmpOrgLuma[0] = origBuf.Y();
    }
  }

#if JVET_AA0070_RRIBC
  if (testRrIBC)
  {
    tmpOrgLuma[1] = m_tmpStorageCUflipH.getBuf(tmpArea);
    tmpOrgLuma[2] = m_tmpStorageCUflipV.getBuf(tmpArea);
    tmpOrgLuma[1].copyFrom(tmpOrgLuma[0]);
    tmpOrgLuma[2].copyFrom(tmpOrgLuma[0]);
    tmpOrgLuma[1].flip(1);
    tmpOrgLuma[2].flip(2);
  }
#endif

  // Compute cost for integer BV
  uint8_t imvForZeroMvd = IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD;
  bool    useBilinearMC = false;
  PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
  DistParam distParam;
#if JVET_AC0112_IBC_LIC
#if JVET_AE0159_FIBC
  if (!pu.cs->sps->getUseIbcFilter())
  {
    distParam.useMR = pu.cu->ibcLicFlag  ? true : false ;
  }
#else
  distParam.useMR = pu.cu->ibcLicFlag;
#endif
#endif
  m_pcRdCost->setDistParam(distParam, tmpOrgLuma[0], predBuf.Y().buf, predBuf.Y().stride, pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, 0, 1, false);
  m_pcRdCost->getMotionCost(0);
  m_pcRdCost->setCostScale(0);

#if JVET_AE0078_IBC_LIC_EXTENSION
  Distortion bestIntBvCost = MAX_UINT64;
#endif
  for (int i = 0; i < intBvList.cnt; ++i)
  {
    pu.cu->imv = IMV_FPEL;
    pu.bv      = intBvList.mvList[i];
    pu.mv[0]   = intBvList.mvList[i];
    pu.mv[0].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
#if JVET_AE0159_FIBC
    if (m_pcEncCfg->getIntraPeriod() != 1) //non-AI
    {    
#if JVET_AE0078_IBC_LIC_EXTENSION
      const int ibcLicLoopNum = pu.cu->ibcLicFlag && !pu.cu->ibcFilterFlag ? 4 : 1;
      int bestLicIdc = 0;
      Distortion bestLicCost = MAX_UINT64;
      for (int licIdc = 0; licIdc < ibcLicLoopNum; licIdc++)
      {
        pu.cu->ibcLicIdx = licIdc;
#endif
#if JVET_AC0112_IBC_LIC
      if (pu.cu->ibcLicFlag)
      {
        getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predBuf, useBilinearMC);
        distParam.cur.buf = predBuf.Y().buf;
        distParam.cur.stride = predBuf.Y().stride;
      }
      else
#endif
      {
        Position offset = pu.Y().pos().offset(pu.bv.getHor(), pu.bv.getVer());
        CPelBuf  refBuf = pu.cu->slice->getPic()->getRecoBuf(CompArea(COMPONENT_Y, pu.chromaFormat, offset, Size(pu.lwidth(), pu.lheight())), false);
        distParam.cur.buf = refBuf.buf;
        distParam.cur.stride = refBuf.stride;
      }
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
      int bvType = intBvList.bvTypeList[i];
#else
      int bvType = 0;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
#if JVET_AA0070_RRIBC 
        tmpOrgLuma[0] = tmpOrgBvpMerge[i];
#else
        distParam.org = tmpOrgBvpMerge[i];
#endif
      }
#endif
#if JVET_AA0070_RRIBC 
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      distParam.org = tmpOrgLuma[intBvList.bvFlipList[i] ? bvType : 0];
#else
      distParam.org = tmpOrgLuma[bvType];
#endif
#endif

      uint32_t   addExtraBits = 1 + bvType
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
        + (bvType == 0 ? 0 : (intBvList.bvFlipList[i] ? 2 : 0))
#endif
        ;
      bool       has4PelIMV = (pu.bv.getHor() & 3) == 0 && (pu.bv.getVer() & 3) == 0;
#if JVET_AE0169_BIPREDICTIVE_IBC
        uint8_t    tempMvpIdx[3] = { 0, 0, 0 };
        Distortion tempBvCost[3] =
        {
          m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_OFF][bvType], IMV_OFF, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[0]),
          m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_FPEL][bvType], IMV_FPEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[1]),
          has4PelIMV ? m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_4PEL][bvType], IMV_4PEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[2]) : std::numeric_limits<Distortion>::max()
        };
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        tempBvCost[0] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        tempBvCost[1] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        if (has4PelIMV)
        {
          tempBvCost[2] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        }
      }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
      if (pu.cu->ibcLicFlag)
      {
        uint8_t bestIdx = (tempBvCost[2] < tempBvCost[1] && tempBvCost[2] < tempBvCost[0]) ? 2 : (tempBvCost[1] < tempBvCost[0]) ? 1 : 0;
        m_bestSrchCostIbcFilter.imvList[licIdc] = ImvMode(bestIdx);
        m_bestSrchCostIbcFilter.mvpIdxList[licIdc] = tempMvpIdx[bestIdx];
        m_bestSrchCostIbcFilter.costList[licIdc] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
        m_bestSrchCostIbcFilter.mvList[licIdc] = pu.mv[0];
        Distortion currCost = licIdc == 0 ? Distortion(m_bestSrchCostIbcFilter.costList[licIdc] * 0.95) : m_bestSrchCostIbcFilter.costList[licIdc];
        if (currCost < bestLicCost)
        {
          bestLicIdc = licIdc;
          bestLicCost = currCost;
        }
      }
      else
      {
#endif
      uint8_t bestIdx = (tempBvCost[2] < tempBvCost[1] && tempBvCost[2] < tempBvCost[0]) ? 2 : (tempBvCost[1] < tempBvCost[0]) ? 1 : 0;
      intBvList.imvList   [i] = ImvMode(bestIdx);
#else
      uint8_t    tempMvpIdx[2] = { 0, 0 };
      Distortion tempBvCost[2] =
      {
        m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_FPEL][bvType], IMV_FPEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[0]),
        has4PelIMV ? m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_4PEL][bvType], IMV_4PEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[1]) : std::numeric_limits<Distortion>::max()
      };
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        tempBvCost[0] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        if (has4PelIMV)
        {
          tempBvCost[1] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        }
      }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
      if (pu.cu->ibcLicFlag)
      {
        uint8_t bestIdx = tempBvCost[1] < tempBvCost[0] ? 1 : 0;
        m_bestSrchCostIbcFilter.imvList[licIdc] = ImvMode(bestIdx);
        m_bestSrchCostIbcFilter.mvpIdxList[licIdc] = tempMvpIdx[bestIdx];
        m_bestSrchCostIbcFilter.costList[licIdc] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
        m_bestSrchCostIbcFilter.mvList[licIdc] = pu.mv[0];
        Distortion currCost = licIdc == 0 ? Distortion(m_bestSrchCostIbcFilter.costList[licIdc] * 0.95)
                                          : m_bestSrchCostIbcFilter.costList[licIdc];
        if (currCost < bestLicCost)
        {
          bestLicIdc = licIdc;
          bestLicCost = currCost;
        }
      }
      else
      {
#endif
      uint8_t bestIdx = tempBvCost[1] < tempBvCost[0] ? 1 : 0;
      intBvList.imvList[i] = bestIdx == 0 ? IMV_FPEL : IMV_4PEL;
#endif
      intBvList.mvpIdxList[i] = tempMvpIdx[bestIdx];
      intBvList.costList[i] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
      intBvList.mvList[i] = pu.mv[0];
#if JVET_AE0078_IBC_LIC_EXTENSION
        }
      }
      if (pu.cu->ibcLicFlag)
      {
        intBvList.imvList[i] = m_bestSrchCostIbcFilter.imvList[bestLicIdc];
        intBvList.mvpIdxList[i] = m_bestSrchCostIbcFilter.mvpIdxList[bestLicIdc];
        intBvList.costList[i] = m_bestSrchCostIbcFilter.costList[bestLicIdc];
        intBvList.mvList[i] = m_bestSrchCostIbcFilter.mvList[bestLicIdc];
        intBvList.bvLicIdx[i] = bestLicIdc;
      }
      bestIntBvCost = std::min(intBvList.costList[i], bestIntBvCost);
#endif
    }
    else
    {
#if JVET_AE0078_IBC_LIC_EXTENSION
      const int ibcLicLoopNum = pu.cu->ibcLicFlag && pu.cs->sps->getUseIbcFilter() && (pu.cu->slice->getSliceType() == I_SLICE) ? 5 : (pu.cu->ibcLicFlag ? 4 : 1);
#else
      const int ibcLicLoopNum = pu.cu->ibcLicFlag && pu.cs->sps->getUseIbcFilter() && (pu.cu->slice->getSliceType() == I_SLICE)  ? 2 : 1;
#endif
      int bestLicIdc = 0;
      Distortion bestLicCost = MAX_UINT64;
      for (int licIdc = 0; licIdc < ibcLicLoopNum; licIdc++)
      {
#if JVET_AE0078_IBC_LIC_EXTENSION
        pu.cu->ibcFilterFlag = (licIdc == 4) ? true : false;
        pu.cu->ibcLicIdx = licIdc < 4 ? licIdc : 0;
#else
        pu.cu->ibcFilterFlag = licIdc > 0 ? true: false;
#endif
#else
#if JVET_AE0078_IBC_LIC_EXTENSION
      const int ibcLicLoopNum = pu.cu->ibcLicFlag ? 4 : 1;
      int bestLicIdc = 0;
      Distortion bestLicCost = MAX_UINT64;
      for (int licIdc = 0; licIdc < ibcLicLoopNum; licIdc++)
      {
        pu.cu->ibcLicIdx = licIdc;
#endif
#endif

#if JVET_AC0112_IBC_LIC
    if (pu.cu->ibcLicFlag)
    {
      getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predBuf, useBilinearMC);
      distParam.cur.buf = predBuf.Y().buf;
      distParam.cur.stride = predBuf.Y().stride;
    }
    else
#endif
    {
      Position offset = pu.Y().pos().offset(pu.bv.getHor(), pu.bv.getVer());
      CPelBuf  refBuf = pu.cu->slice->getPic()->getRecoBuf(CompArea(COMPONENT_Y, pu.chromaFormat, offset, Size(pu.lwidth(), pu.lheight())), false);
      distParam.cur.buf = refBuf.buf;
      distParam.cur.stride = refBuf.stride;
    }
    
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC
      int bvType = intBvList.bvTypeList[i];
#else
      int bvType = 0;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
#if JVET_AA0070_RRIBC 
        tmpOrgLuma[0] = tmpOrgBvpMerge[i];
#else
        distParam.org = tmpOrgBvpMerge[i];
#endif
      }
#endif
#if JVET_AA0070_RRIBC 
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      distParam.org = tmpOrgLuma[intBvList.bvFlipList[i] ? bvType : 0];
#else
      distParam.org = tmpOrgLuma[bvType];
#endif
#endif

      uint32_t   addExtraBits  = 1 + bvType
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
                               + (bvType == 0 ? 0 : (intBvList.bvFlipList[i] ? 2 : 0))
#endif
        ;
      bool       has4PelIMV    = (pu.bv.getHor() & 3) == 0 && (pu.bv.getVer() & 3) == 0;
#if JVET_AE0169_BIPREDICTIVE_IBC
      uint8_t    tempMvpIdx[3] = { 0, 0, 0 };
      Distortion tempBvCost[3] =
      {
                     m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_OFF][bvType], IMV_OFF, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[0]),
                     m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_FPEL][bvType], IMV_FPEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[1]),
        has4PelIMV ? m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_4PEL][bvType], IMV_4PEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[2]) : std::numeric_limits<Distortion>::max()
      };
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        tempBvCost[0] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        tempBvCost[1] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        if (has4PelIMV)
        {
          tempBvCost[2] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        }
      }
#endif
#if JVET_AE0159_FIBC || JVET_AE0078_IBC_LIC_EXTENSION
      if (pu.cu->ibcLicFlag )
      {
        uint8_t bestIdx = (tempBvCost[2] < tempBvCost[1] && tempBvCost[2] < tempBvCost[0]) ? 2 : (tempBvCost[1] < tempBvCost[0]) ? 1 : 0;
        m_bestSrchCostIbcFilter.imvList[licIdc] = ImvMode(bestIdx);
        m_bestSrchCostIbcFilter.mvpIdxList[licIdc] = tempMvpIdx[bestIdx];
        m_bestSrchCostIbcFilter.costList[licIdc] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
        m_bestSrchCostIbcFilter.mvList[licIdc] = pu.mv[0];
        Distortion currCost = licIdc == 0 ? Distortion(m_bestSrchCostIbcFilter.costList[licIdc] * 0.95) : m_bestSrchCostIbcFilter.costList[licIdc];
        if (currCost < bestLicCost)
        {
          bestLicIdc = licIdc;
          bestLicCost = currCost;
        }
      }
      else 
      {
#endif
      uint8_t bestIdx = (tempBvCost[2] < tempBvCost[1] && tempBvCost[2] < tempBvCost[0]) ? 2 : (tempBvCost[1] < tempBvCost[0]) ? 1 : 0;
      intBvList.imvList   [i] = ImvMode(bestIdx);
#else
      uint8_t    tempMvpIdx[2] = { 0, 0 };
      Distortion tempBvCost[2] =
      {
                     m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_FPEL][bvType], IMV_FPEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[0]),
        has4PelIMV ? m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_4PEL][bvType], IMV_4PEL, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[1]) : std::numeric_limits<Distortion>::max()
      };
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        tempBvCost[0] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        if (has4PelIMV)
        {
          tempBvCost[1] += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[i]);
        }
      }
#endif
#if JVET_AE0159_FIBC || JVET_AE0078_IBC_LIC_EXTENSION
      if (pu.cu->ibcLicFlag)
      {
        uint8_t bestIdx = tempBvCost[1] < tempBvCost[0] ? 1 : 0;
        m_bestSrchCostIbcFilter.imvList[licIdc] = ImvMode(bestIdx);
        m_bestSrchCostIbcFilter.mvpIdxList[licIdc] = tempMvpIdx[bestIdx];
        m_bestSrchCostIbcFilter.costList[licIdc] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
        m_bestSrchCostIbcFilter.mvList[licIdc] = pu.mv[0];
        Distortion currCost = licIdc == 0 ? Distortion(m_bestSrchCostIbcFilter.costList[licIdc] * 0.95)
                                          : m_bestSrchCostIbcFilter.costList[licIdc];
        if (currCost < bestLicCost)
        {
          bestLicIdc = licIdc;
          bestLicCost = currCost;
        }
      }
      else
      {
#endif
      uint8_t bestIdx = tempBvCost[1] < tempBvCost[0] ? 1 : 0;
      intBvList.imvList   [i] = bestIdx == 0 ? IMV_FPEL : IMV_4PEL;
#endif
      intBvList.mvpIdxList[i] = tempMvpIdx[bestIdx];
      intBvList.costList  [i] = tempBvCost[bestIdx] + distParam.distFunc(distParam);
      intBvList.mvList    [i] = pu.mv[0];
#if JVET_AE0159_FIBC
    }
#endif
    }
#if JVET_AE0159_FIBC || JVET_AE0078_IBC_LIC_EXTENSION
    }
      if (pu.cu->ibcLicFlag)
      {
        intBvList.imvList[i] = m_bestSrchCostIbcFilter.imvList[bestLicIdc];
        intBvList.mvpIdxList[i] = m_bestSrchCostIbcFilter.mvpIdxList[bestLicIdc];
        intBvList.costList[i] = m_bestSrchCostIbcFilter.costList[bestLicIdc];
        intBvList.mvList[i] = m_bestSrchCostIbcFilter.mvList[bestLicIdc];
#if JVET_AE0078_IBC_LIC_EXTENSION
#if JVET_AE0159_FIBC
        intBvList.bvFilter[i] = bestLicIdc == 4 ? true : false;
#endif
        intBvList.bvLicIdx[i] = bestLicIdc < 4 ? bestLicIdc : 0;
#else
        intBvList.bvFilter[i] = bestLicIdc;
#endif
      }
#if JVET_AE0078_IBC_LIC_EXTENSION
      bestIntBvCost = std::min(intBvList.costList[i], bestIntBvCost);
#endif
  }
#endif
  }
#if JVET_AE0078_IBC_LIC_EXTENSION
  for (int i = 0; i < intBvList.cnt; ++i)
  {
    intBvList.skipLicSrch[i] = (intBvList.costList[i] > Distortion(2.5 * bestIntBvCost)) ? true : false;
  }
#endif
  distParam.cur.buf    = predBuf.Y().buf;
  distParam.cur.stride = predBuf.Y().stride;

  // Frac-pel search
  const static Mv mvOffsetH[] = { Mv(-8, 0), Mv(8, 0), Mv(0, -8), Mv(0, 8), Mv(-8, -8), Mv(8, -8), Mv(-8, 8), Mv(8, 8), };
  const static Mv mvOffsetQ[] = { Mv(-4, 0), Mv(4, 0), Mv(0, -4), Mv(0, 4), Mv(-4, -4), Mv(4, -4), Mv(-4, 4), Mv(4, 4), };

  auto ibcFracPelSquareSearch = [&](uint8_t imv, int candIdx, const Mv* mvOffset, int numOffset)
  {
    CHECK(imv != IMV_HPEL && imv != IMV_OFF, "Error on IMV value in IBC fractional-pel search");
    Distortion& bestCost     = intBvList.costList  [candIdx];
    uint8_t&    bestImv      = intBvList.imvList   [candIdx];
    uint8_t&    bestMvpIdx   = intBvList.mvpIdxList[candIdx];
    Mv&         bestMv       = intBvList.mvList    [candIdx];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    int&        bestBvType   = intBvList.bvTypeList[candIdx];
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bool&       bestFlipMode = intBvList.bvFlipList[candIdx];
#endif
#if JVET_AE0159_FIBC
    if (pu.cs->sps->getUseIbcFilter() && m_pcEncCfg->getIntraPeriod() == 1)
    {
      pu.cu->ibcFilterFlag = intBvList.bvFilter[candIdx];
    }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
    pu.cu->ibcLicIdx = intBvList.bvLicIdx[candIdx];
#endif

    pu.cu->imv = imv;
    const Mv  centerMv = bestMv;
    for (int j = 0; j < numOffset; ++j)
    {
      pu.mv[0] = centerMv + mvOffset[j];
      uint32_t validType = PU::checkValidBvPU(pu, COMPONENT_Y, pu.mv[0], true, 0);
      if (validType == IBC_BV_INVALID)
      {
        continue;
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if ((pu.amvpMergeModeFlag[REF_PIC_LIST_1] && (pu.mv[0].getHor() == m_amvpMergeCtx.mvFieldNeighbours[intBvList.mergeIdxList[candIdx]<<1].mv.getHor()) &&
        (pu.mv[0].getVer() == m_amvpMergeCtx.mvFieldNeighbours[intBvList.mergeIdxList[candIdx]<<1].mv.getVer())))
      {
        continue;
      }
#endif
      getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predBuf, useBilinearMC);

      // Check cost
      uint32_t addExtraBits  = 1;
      distParam.org = tmpOrgLuma[0];
      uint8_t tempMvpIdx = 0;
      Distortion tempCost = m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[imv][0], imv, imvForZeroMvd, false, false, addExtraBits, tempMvpIdx);
      int        tempBvType = 0;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      if (pu.isBvpClusterApplicable())
      {
        // Check cost for hor bv and ver bv
        for (int bvType = 1; bvType < numBvTypes; ++bvType)
        {
          if ((bvType == 1 && pu.mv[0].ver != 0) || (bvType == 2 && pu.mv[0].hor != 0))
          {
            continue;
          }

          addExtraBits = 1 + bvType;
          Distortion tempCost1D = m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[imv][bvType], imv, imvForZeroMvd, bvType == 2, bvType == 1, addExtraBits, tempMvpIdx);
          if (tempCost1D < tempCost)
          {
            tempCost   = tempCost1D;
            tempBvType = bvType;
          }
        }
      }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        tempCost += m_pcRdCost->getBvpMergeCost(intBvList.mergeIdxList[candIdx]);
      }
#endif
      tempCost += (tempCost < bestCost ? distParam.distFunc(distParam) : 0);

      if (tempCost < bestCost)
      {
        bestCost     = tempCost;
        bestImv      = imv;
        bestMvpIdx   = tempMvpIdx;
        bestMv       = pu.mv[0];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        bestBvType   = tempBvType;
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        bestFlipMode = false;
#endif
      }

#if JVET_AA0070_RRIBC
      // Check cost for hor flip and ver flip
      if(testRrIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        && pu.cs->sps->getUseRRIbc()
#endif
        )
      {
        for (int rrType = 1; rrType < numBvTypes; ++rrType)
        {
          if ((rrType == 1 && pu.mv[0].ver != 0) || (rrType == 2 && pu.mv[0].hor != 0))
          {
            continue;
          }
          
          addExtraBits = 1 + rrType
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                       + 2
#endif
            ;
          distParam.org = tmpOrgLuma[rrType];
          tempMvpIdx = 0;
          tempCost = m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[imv][rrType], imv, imvForZeroMvd, rrType == 2, rrType == 1, addExtraBits, tempMvpIdx);
          tempCost += (tempCost < bestCost ? distParam.distFunc(distParam) : 0);
          
          if (tempCost < bestCost)
          {
            bestCost     = tempCost;
            bestImv      = imv;
            bestMvpIdx   = tempMvpIdx;
            bestMv       = pu.mv[0];
            bestBvType   = rrType;
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            bestFlipMode = true;
#endif
          }
        }
      }
#endif
    }
  };

  if (!intBvList.enableMultiCandSrch && intBvList.mvList[intBvList.maxSize] != Mv())
  {
    if( NOT_VALID == intBvList.find(intBvList.mvList[intBvList.maxSize].hor
                                  , intBvList.mvList[intBvList.maxSize].ver
#if JVET_AE0169_BIPREDICTIVE_IBC
                                  , intBvList.mergeIdxList[intBvList.maxSize]
#endif
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                                  , intBvList.bvTypeList[intBvList.maxSize]
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
                                  , intBvList.bvFlipList[intBvList.maxSize]
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
                                  , intBvList.bvLicIdx[intBvList.maxSize]
#endif
      ))
    {
      intBvList.replaceAt(intBvList.maxSize, 1); // Add history best frac BV
      intBvList.cnt = 2;
    }
  }

  for (int i = 0; i < intBvList.cnt; ++i)
  {
#if JVET_AE0078_IBC_LIC_EXTENSION
    if (pu.cu->ibcLicFlag && intBvList.skipLicSrch[i])
    {
      continue;
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3)
    {
      tmpOrgLuma[0] = tmpOrgBvpMerge[i];
    }
#endif
    ibcFracPelSquareSearch(IMV_OFF, i, mvOffsetH, 8);
    ibcFracPelSquareSearch(IMV_OFF, i, mvOffsetQ, 8);
  }

  // Find best
  int bestCandIdx = 0;
  if (intBvList.cnt > 1)
  {
    for (int i = 1; i < intBvList.cnt; ++i)
    {
      if (intBvList.costList[i] < intBvList.costList[bestCandIdx])
      {
        bestCandIdx = i;
      }
    }
  }

  if (intBvList.enableMultiCandSrch)
  {  
    // stored for next round
    intBvList.replaceAt(bestCandIdx, intBvList.maxSize);

    // Further refinement when best bv is fractional
    if ((intBvList.imvList[bestCandIdx] == IMV_OFF || intBvList.imvList[bestCandIdx] == IMV_HPEL))
    {
      intBvList.replaceAt(bestCandIdx, 0);
      intBvList.cnt = 1;
      bestCandIdx   = 0;

      ibcFracPelSquareSearch(IMV_OFF, 0, mvOffsetH, 8);
      ibcFracPelSquareSearch(IMV_OFF, 0, mvOffsetQ, 8);
    }
  }

  // Set best mode
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  int bestBvType = intBvList.bvTypeList[bestCandIdx];
#else
  int bestBvType = 0;
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  pu.cu->bvZeroCompDir = bestBvType;
  pu.cu->bvOneZeroComp = bestBvType > 0;
#endif
#if JVET_AA0070_RRIBC
  pu.cu->rribcFlipType = bestBvType;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  pu.cu->rribcFlipType = pu.cs->sps->getUseRRIbc() ? pu.cu->rribcFlipType : 0;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  pu.cu->rribcFlipType = intBvList.bvFlipList[bestCandIdx] ? pu.cu->rribcFlipType : 0;
#endif
#endif
#if JVET_AE0159_FIBC
  if (pu.cs->sps->getUseIbcFilter() && m_pcEncCfg->getIntraPeriod() == 1)
  {
    pu.cu->ibcFilterFlag = pu.cu->ibcLicFlag ? intBvList.bvFilter[bestCandIdx] : 0;
  }
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
  pu.cu->ibcLicIdx = pu.cu->ibcLicFlag ? intBvList.bvLicIdx[bestCandIdx] : 0;
#endif

  pu.cu->imv   = intBvList.imvList   [bestCandIdx];
  pu.mvpIdx[0] = intBvList.mvpIdxList[bestCandIdx];
  pu.mv    [0] = intBvList.mvList    [bestCandIdx];
  pu.mvd   [0] = pu.mv[0] - amvpInfoList[pu.cu->imv][bestBvType].mvCand[pu.mvpIdx[0]];

  pu.bv = pu.mv[0];
  pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // bv is always stored at integer precision

#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  if (bestBvType == 1)
  {
    pu.mvd[REF_PIC_LIST_0].setVer(0);
    CHECK(pu.mv[0].ver != 0, "Invalid bv for horizontal IBC mode");
  }
  else if (bestBvType == 2)
  {
    pu.mvd[REF_PIC_LIST_0].setHor(0);
    CHECK(pu.mv[0].hor != 0, "Invalid bv for vertical IBC mode");
  }
#endif

  if (pu.mvd[REF_PIC_LIST_0] == Mv(0, 0))
  {
    pu.cu->imv = imvForZeroMvd;

    pu.mv[0] = amvpInfoList[pu.cu->imv][bestBvType].mvCand[pu.mvpIdx[0]];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    pu.mv[0].regulateMv(bestBvType);
#endif

    pu.bv = pu.mv[0];
    pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // pu.bv is always stored at integer precision
  }

  pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
#if JVET_AE0169_BIPREDICTIVE_IBC
  return (pu.interDir == 3 && pu.ibcCiipFlag) ? ((intBvList.costList[bestCandIdx]+2)>>2) : (pu.interDir == 3 || pu.ibcCiipFlag) ? ((intBvList.costList[bestCandIdx]+1)>>1) : intBvList.costList[bestCandIdx];
#else
  return intBvList.costList[bestCandIdx];
#endif
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
inline void InterSearch::getBestBvpBvOneZeroComp(PredictionUnit &pu, Mv cMv, Distortion initCost,
                                                 int *bvpIdxBest, AMVPInfo *amvp1Pel, AMVPInfo *amvp4Pel)
{
  Mv         bvpCand[2];
  int        tempImv = 0, tempIdx = 0;
  Distortion bvOneZeroCompCost = std::numeric_limits<uint32_t>::max();
  if (cMv.getVer() == 0)
  {
    bvpCand[0] = Mv(std::max(-(int) pu.lwidth(), -pu.Y().x), 0);
    bvpCand[1] = Mv(-pu.Y().x, 0);
    bvOneZeroCompCost = m_pcRdCost->getBvVerZeroCompCost(cMv.getHor(), pu.cs->sps->getAMVREnabledFlag(), &tempImv, &tempIdx, bvpCand);
  }
  else if (cMv.getHor() == 0)
  {
#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
    bvpCand[0] = Mv(0, std::max(-(int) pu.lheight(), -pu.Y().y));
    bvpCand[1] = Mv(0, -pu.Y().y);
    bvOneZeroCompCost = m_pcRdCost->getBvHorZeroCompCost(cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), &tempImv, &tempIdx, bvpCand);
#else
    const int ctbSize     = pu.cs->sps->getCTUSize();
    const int numCurrCtuY = (pu.Y().y >> (floorLog2(ctbSize)));
    unsigned int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
    int rrTop;
    if (256 == lcuWidth)
    {
      rrTop = (numCurrCtuY < 2) ? -pu.Y().y : -((pu.Y().y & (ctbSize - 1)) + ctbSize);
    }
    else
    {
      rrTop = (numCurrCtuY < 3) ? -pu.Y().y : -((pu.Y().y & (ctbSize - 1)) + 2 * ctbSize);
    }
    bvpCand[0] = Mv(0, std::max(-(int) pu.lheight(), rrTop));
    bvpCand[1] = Mv(0, rrTop);
    bvOneZeroCompCost = m_pcRdCost->getBvHorZeroCompCost(cMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), &tempImv, &tempIdx, bvpCand);
#endif
  }

  if (bvOneZeroCompCost < initCost)
  {
    pu.cu->bvOneZeroComp = 1;
    pu.cu->bvZeroCompDir = (cMv.getVer() == 0) ? 1 : cMv.getHor() == 0 ? 2 : 0;
    pu.cu->imv           = tempImv;
    *bvpIdxBest          = tempIdx;
    if (pu.cu->imv == 2)
    {
      amvp4Pel->mvCand[tempIdx] = bvpCand[tempIdx];
      amvp4Pel->mvCand[tempIdx].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    }
    else
    {
      amvp1Pel->mvCand[tempIdx] = bvpCand[tempIdx];
      amvp1Pel->mvCand[tempIdx].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    }
  }
}
#endif

#if JVET_AA0070_RRIBC
void InterSearch::xxIBCHashSearch(PredictionUnit &pu, Mv mvPred[3][2], int numMvPred, Mv &mv, int &idxMvPred, IbcHashMap &ibcHashMap, AMVPInfo amvpInfo4Pel[3], int numRribcType)
#else
void InterSearch::xxIBCHashSearch(PredictionUnit &pu, Mv *mvPred, int numMvPred, Mv &mv, int &idxMvPred, IbcHashMap &ibcHashMap)
#endif
{
  mv.setZero();
  m_pcRdCost->setCostScale(0);

#if JVET_AA0070_RRIBC
  const unsigned int lcuWidth  = pu.cs->slice->getSPS()->getMaxCUWidth();
  const int          cuPelX    = pu.Y().x;
  const int          cuPelY    = pu.Y().y;
  const int          picWidth  = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int          picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
  int                roiWidth  = pu.lwidth();
  int                roiHeight = pu.lheight();
#if JVET_Z0131_IBC_BVD_BINARIZATION
  Distortion minCost = MAX_UINT64;
  m_pcRdCost->setPredictors(mvPred);
#else
  unsigned int minCost = MAX_UINT;
#endif

  std::vector<Position> candPos[3];
  bool                  isHashMatch[3] = { false };
  for (int rribcFlipType = 0; rribcFlipType < numRribcType; rribcFlipType++)
  {
    if(rribcFlipType && m_pcEncCfg->getIntraPeriod() <= 1)
    {
      Distortion th = m_pcEncCfg->getIntraPeriod() == -1 ? 50 : 200;
      if ((minCost < th) || (!isHashMatch[rribcFlipType - 1]))
      {
        continue;
      }
    }
    else if (rribcFlipType)
    {
      if (minCost < 100 || (rribcFlipType == 1 && !isHashMatch[0]) || (rribcFlipType == 2 && !isHashMatch[0] && !isHashMatch[1]))
      {
        continue;
      }
    }
    isHashMatch[rribcFlipType] = ibcHashMap.ibcHashMatch(pu, pu.Y(), candPos[rribcFlipType], *pu.cs, m_pcEncCfg->getIBCHashSearchMaxCand(), m_pcEncCfg->getIBCHashSearchRange4SmallBlk(), rribcFlipType);
    if (!isHashMatch[rribcFlipType])
    {
      continue;
    }
    for (std::vector<Position>::iterator pos = candPos[rribcFlipType].begin(); pos != candPos[rribcFlipType].end(); pos++)
    {
      if ((rribcFlipType == 1 && (*pos - pu.Y().pos()).y != 0) || (rribcFlipType == 2 && (*pos - pu.Y().pos()).x != 0))
      {
        continue;
      }
#else
  std::vector<Position> candPos;
  if (ibcHashMap.ibcHashMatch(pu.Y(), candPos, *pu.cs, m_pcEncCfg->getIBCHashSearchMaxCand(), m_pcEncCfg->getIBCHashSearchRange4SmallBlk()))
  {
#if JVET_Z0131_IBC_BVD_BINARIZATION
    Distortion minCost = MAX_UINT64;
    m_pcRdCost->setPredictors(mvPred);
#else
    unsigned int minCost = MAX_UINT;
#endif

    const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
    const int   cuPelX = pu.Y().x;
    const int   cuPelY = pu.Y().y;
    const int   picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
    const int   picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
    int         roiWidth = pu.lwidth();
    int         roiHeight = pu.lheight();

    for (std::vector<Position>::iterator pos = candPos.begin(); pos != candPos.end(); pos++)
    {
#endif
      Position bottomRight = pos->offset(pu.Y().width - 1, pu.Y().height - 1);
      if (pu.cs->isDecomp(*pos, CHANNEL_TYPE_LUMA) && pu.cs->isDecomp(bottomRight, CHANNEL_TYPE_LUMA))
      {
        Position tmp = *pos - pu.Y().pos();
        Mv candMv;
        candMv.set(tmp.x, tmp.y);

#if JVET_Z0084_IBC_TM
        if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, candMv.getHor(), candMv.getVer(), lcuWidth))
#else
        if (!searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, candMv.getHor(), candMv.getVer(), lcuWidth))
#endif
        {
          continue;
        }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        m_pcRdCost->setFullPelImvForZeroBvd(!pu.cs->sps->getIBCFracFlag());
#endif
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AA0070_RRIBC
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        Distortion cost = m_pcRdCost->getBvCostMultiplePreds(candMv.getHor(), candMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), rribcFlipType, &pu.cu->imv, &idxMvPred, true, &amvpInfo4Pel[rribcFlipType]);
#else
        Distortion cost = m_pcRdCost->getBvCostMultiplePreds(candMv.getHor(), candMv.getVer(), pu.cs->sps->getAMVREnabledFlag(), rribcFlipType, &pu.cu->imv, &idxMvPred);
#endif
#else
        Distortion cost = m_pcRdCost->getBvCostMultiplePreds(candMv.getHor(), candMv.getVer(), pu.cs->sps->getAMVREnabledFlag());
#endif
        if (cost < minCost)
        {
          mv = candMv;
          minCost = cost;
#if JVET_AA0070_RRIBC
          pu.cu->rribcFlipType = rribcFlipType;
#endif
        }
#else
        for (int n = 0; n < numMvPred; n++)
        {
#if JVET_AA0070_RRIBC
          m_pcRdCost->setPredictor(mvPred[rribcFlipType][n]);
          unsigned int cost = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor(), candMv.getVer(), 0, rribcFlipType);
#else
          m_pcRdCost->setPredictor(mvPred[n]);
          unsigned int cost = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor(), candMv.getVer(), 0);
#endif

          if (cost < minCost)
          {
            mv = candMv;
            idxMvPred = n;
            minCost = cost;
#if JVET_AA0070_RRIBC
            pu.cu->rribcFlipType = rribcFlipType;
#endif
          }

          int costQuadPel = MAX_UINT;
          if ((candMv.getHor() % 4 == 0) && (candMv.getVer() % 4 == 0) && (pu.cs->sps->getAMVREnabledFlag()))
          {
            Mv mvPredQuadPel;
            int imvShift = 2;
            int offset = 1 << (imvShift - 1);

#if JVET_AA0070_RRIBC
            int x = (mvPred[rribcFlipType][n].hor + offset - (mvPred[rribcFlipType][n].hor >= 0)) >> 2;
            int y = (mvPred[rribcFlipType][n].ver + offset - (mvPred[rribcFlipType][n].ver >= 0)) >> 2;
#else
            int x = (mvPred[n].hor + offset - (mvPred[n].hor >= 0)) >> 2;
            int y = (mvPred[n].ver + offset - (mvPred[n].ver >= 0)) >> 2;
#endif
            mvPredQuadPel.set(x, y);

            m_pcRdCost->setPredictor(mvPredQuadPel);

#if JVET_AA0070_RRIBC
            costQuadPel = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor() >> 2, candMv.getVer() >> 2, 0, rribcFlipType);
#else
            costQuadPel = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor() >> 2, candMv.getVer() >> 2, 0);
#endif

          }
          if (costQuadPel < minCost)
          {
            mv = candMv;
            idxMvPred = n;
            minCost = costQuadPel;
#if JVET_AA0070_RRIBC
            pu.cu->rribcFlipType = rribcFlipType;
#endif
          }

        }
#endif
      }
    }
  }
}


void InterSearch::addToSortList(std::list<BlockHash>& listBlockHash, std::list<int>& listCost, int cost, const BlockHash& blockHash)
{
  std::list<BlockHash>::iterator itBlockHash = listBlockHash.begin();
  std::list<int>::iterator itCost = listCost.begin();

  while (itCost != listCost.end())
  {
    if (cost < (*itCost))
    {
      listCost.insert(itCost, cost);
      listBlockHash.insert(itBlockHash, blockHash);
      return;
    }

    ++itCost;
    ++itBlockHash;
  }

  listCost.push_back(cost);
  listBlockHash.push_back(blockHash);
}

void InterSearch::selectMatchesInter(const MapIterator& itBegin, int count, std::list<BlockHash>& listBlockHash, const BlockHash& currBlockHash)
{
  const int maxReturnNumber = 5;

  listBlockHash.clear();
  std::list<int> listCost;
  listCost.clear();

  MapIterator it = itBegin;
  for (int i = 0; i < count; i++, it++)
  {
    if ((*it).hashValue2 != currBlockHash.hashValue2)
    {
      continue;
    }

    int currCost = RdCost::xGetExpGolombNumberOfBits((*it).x - currBlockHash.x) +
      RdCost::xGetExpGolombNumberOfBits((*it).y - currBlockHash.y);

    if (listBlockHash.size() < maxReturnNumber)
    {
      addToSortList(listBlockHash, listCost, currCost, (*it));
    }
    else if (!listCost.empty() && currCost < listCost.back())
    {
      listCost.pop_back();
      listBlockHash.pop_back();
      addToSortList(listBlockHash, listCost, currCost, (*it));
    }
  }
}
void InterSearch::selectRectangleMatchesInter(const MapIterator& itBegin, int count, std::list<BlockHash>& listBlockHash, const BlockHash& currBlockHash, int width, int height, int idxNonSimple, unsigned int* &hashValues, int baseNum, int picWidth, int picHeight, bool isHorizontal, uint16_t* curHashPic)
{
  const int maxReturnNumber = 5;
  int baseSize = min(width, height);
  unsigned int crcMask = 1 << 16;
  crcMask -= 1;

  listBlockHash.clear();
  std::list<int> listCost;
  listCost.clear();

  MapIterator it = itBegin;

  for (int i = 0; i < count; i++, it++)
  {
    if ((*it).hashValue2 != currBlockHash.hashValue2)
    {
      continue;
    }
    int xRef = (*it).x;
    int yRef = (*it).y;
    if (isHorizontal)
    {
      xRef -= idxNonSimple * baseSize;
    }
    else
    {
      yRef -= idxNonSimple * baseSize;
    }
    if (xRef < 0 || yRef < 0 || xRef + width >= picWidth || yRef + height >= picHeight)
    {
      continue;
    }
    //check Other baseSize hash values
    uint16_t* refHashValue = curHashPic + yRef * picWidth + xRef;
    bool isSame = true;

    for (int k = 0; k < baseNum; k++)
    {
      if ((*refHashValue) != (uint16_t)(hashValues[k] & crcMask))
      {
        isSame = false;
        break;
      }
      refHashValue += (isHorizontal ? baseSize : (baseSize*picWidth));
    }
    if (!isSame)
    {
      continue;
    }

    int currCost = RdCost::xGetExpGolombNumberOfBits(xRef - currBlockHash.x) +
      RdCost::xGetExpGolombNumberOfBits(yRef - currBlockHash.y);

    BlockHash refBlockHash;
    refBlockHash.hashValue2 = (*it).hashValue2;
    refBlockHash.x = xRef;
    refBlockHash.y = yRef;

    if (listBlockHash.size() < maxReturnNumber)
    {
      addToSortList(listBlockHash, listCost, currCost, refBlockHash);
    }
    else if (!listCost.empty() && currCost < listCost.back())
    {
      listCost.pop_back();
      listBlockHash.pop_back();
      addToSortList(listBlockHash, listCost, currCost, refBlockHash);
    }
  }
}

bool InterSearch::xRectHashInterEstimation(PredictionUnit& pu, RefPicList& bestRefPicList, int& bestRefIndex, Mv& bestMv, Mv& bestMvd, int& bestMVPIndex, bool& isPerfectMatch)
{
  int width = pu.cu->lumaSize().width;
  int height = pu.cu->lumaSize().height;

  int baseSize = min(width, height);
  bool isHorizontal = true;;
  int baseNum = 0;
  if (height < width)
  {
    isHorizontal = true;
    baseNum = 1 << (floorLog2(width) - floorLog2(height));
  }
  else
  {
    isHorizontal = false;
    baseNum = 1 << (floorLog2(height) - floorLog2(width));
  }

  int xPos = pu.cu->lumaPos().x;
  int yPos = pu.cu->lumaPos().y;
  const int currStride = pu.cs->picture->getOrigBuf().get(COMPONENT_Y).stride;
  const Pel* curPel = pu.cs->picture->getOrigBuf().get(COMPONENT_Y).buf + yPos * currStride + xPos;
  int picWidth = pu.cu->slice->getPPS()->getPicWidthInLumaSamples();
  int picHeight = pu.cu->slice->getPPS()->getPicHeightInLumaSamples();

  int xBase = xPos;
  int yBase = yPos;
  const Pel* basePel = curPel;
  int idxNonSimple = -1;
  unsigned int* hashValue1s = new unsigned int[baseNum];
  unsigned int* hashValue2s = new unsigned int[baseNum];

  for (int k = 0; k < baseNum; k++)
  {
    if (isHorizontal)
    {
      xBase = xPos + k * baseSize;
      basePel = curPel + k * baseSize;
    }
    else
    {
      yBase = yPos + k * baseSize;
      basePel = curPel + k * baseSize * currStride;
    }

    if (idxNonSimple == -1 && !TComHash::isHorizontalPerfectLuma(basePel, currStride, baseSize, baseSize) && !TComHash::isVerticalPerfectLuma(basePel, currStride, baseSize, baseSize))
    {
      idxNonSimple = k;
    }
    TComHash::getBlockHashValue((pu.cs->picture->getOrigBuf()), baseSize, baseSize, xBase, yBase, pu.cu->slice->getSPS()->getBitDepths(), hashValue1s[k], hashValue2s[k]);
  }
  if (idxNonSimple == -1)
  {
    idxNonSimple = 0;
  }

  Distortion bestCost = UINT64_MAX;

  BlockHash currBlockHash;
  currBlockHash.x = xPos;//still use the first base block location
  currBlockHash.y = yPos;

  currBlockHash.hashValue2 = hashValue2s[idxNonSimple];

  m_pcRdCost->setDistParam(m_cDistParam, pu.cs->getOrgBuf(pu).Y(), 0, 0, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, false);

  int imvBest = 0;
  int numPredDir = pu.cu->slice->isInterP() ? 1 : 2;
  for (int refList = 0; refList < numPredDir; refList++)
  {
    RefPicList eRefPicList = (refList == 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    int refPicNumber = pu.cu->slice->getNumRefIdx(eRefPicList);

    for (int refIdx = 0; refIdx < refPicNumber; refIdx++)
    {
      int bitsOnRefIdx = 1;
      if (refPicNumber > 1)
      {
        bitsOnRefIdx += refIdx + 1;
        if (refIdx == refPicNumber - 1)
        {
          bitsOnRefIdx--;
        }
      }
      m_numHashMVStoreds[eRefPicList][refIdx] = 0;

      const std::pair<int, int>& scaleRatio = pu.cu->slice->getScalingRatio( eRefPicList, refIdx );
      if( scaleRatio != SCALE_1X )
      {
        continue;
      }

      CHECK( pu.cu->slice->getRefPic( eRefPicList, refIdx )->getHashMap() == nullptr, "Hash table is not initialized" );

      if (refList == 0 || pu.cu->slice->getList1IdxToList0Idx(refIdx) < 0)
      {
        int count = static_cast<int>(pu.cu->slice->getRefPic(eRefPicList, refIdx)->getHashMap()->count(hashValue1s[idxNonSimple]));
        if (count == 0)
        {
          continue;
        }

        list<BlockHash> listBlockHash;
        selectRectangleMatchesInter(pu.cu->slice->getRefPic(eRefPicList, refIdx)->getHashMap()->getFirstIterator(hashValue1s[idxNonSimple]), count, listBlockHash, currBlockHash, width, height, idxNonSimple, hashValue2s, baseNum, picWidth, picHeight, isHorizontal, pu.cu->slice->getRefPic(eRefPicList, refIdx)->getHashMap()->getHashPic(baseSize));

        m_numHashMVStoreds[eRefPicList][refIdx] = int(listBlockHash.size());
        if (listBlockHash.empty())
        {
          continue;
        }
        AMVPInfo currAMVPInfoPel;
        AMVPInfo currAMVPInfo4Pel;
        AMVPInfo currAMVPInfoQPel;
        pu.cu->imv = 2;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfo4Pel
#if TM_AMVP
                      , this
#endif
        );
        pu.cu->imv = 1;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfoPel
#if TM_AMVP
                      , this
#endif
        );
        pu.cu->imv = 0;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfoQPel
#if TM_AMVP
                      , this
#endif
        );
#if TM_AMVP
        CHECK(currAMVPInfoPel.numCand != currAMVPInfoQPel.numCand, "The number of full-Pel AMVP candidates and that of Q-Pel should be identical");
        CHECK(currAMVPInfoPel.numCand != currAMVPInfo4Pel.numCand, "The number of full-Pel AMVP candidates and that of 4-Pel should be identical");
        const uint8_t amvpNumCand = currAMVPInfoPel.numCand;
        for (int mvpIdxTemp = 0; mvpIdxTemp < amvpNumCand; mvpIdxTemp++)
#else
        for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
#endif
        {
          currAMVPInfoQPel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
          currAMVPInfoPel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
          currAMVPInfo4Pel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
        }

        bool wrap = pu.cu->slice->getRefPic(eRefPicList, refIdx)->isWrapAroundEnabled( pu.cs->pps );
        const Pel* refBufStart = pu.cu->slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMPONENT_Y).buf;
        const int refStride = pu.cu->slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMPONENT_Y).stride;
        m_cDistParam.cur.stride = refStride;

        m_pcRdCost->selectMotionLambda( );
        m_pcRdCost->setCostScale(0);

        list<BlockHash>::iterator it;
        int countMV = 0;
        for (it = listBlockHash.begin(); it != listBlockHash.end(); ++it)
        {
          int curMVPIdx = 0;
          unsigned int curMVPbits = MAX_UINT;
          Mv cMv((*it).x - currBlockHash.x, (*it).y - currBlockHash.y);
          m_hashMVStoreds[eRefPicList][refIdx][countMV++] = cMv;
          cMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_QUARTER);

#if TM_AMVP
          for (int mvpIdxTemp = 0; mvpIdxTemp < amvpNumCand; mvpIdxTemp++)
#else
          for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
#endif
          {
            Mv cMvPredPel = currAMVPInfoQPel.mvCand[mvpIdxTemp];
            m_pcRdCost->setPredictor(cMvPredPel);

            unsigned int tempMVPbits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0);

            if (tempMVPbits < curMVPbits)
            {
              curMVPbits = tempMVPbits;
              curMVPIdx = mvpIdxTemp;
              pu.cu->imv = 0;
            }

            if (pu.cu->slice->getSPS()->getAMVREnabledFlag())
            {
              unsigned int bitsMVP1Pel = MAX_UINT;
              Mv mvPred1Pel = currAMVPInfoPel.mvCand[mvpIdxTemp];
              m_pcRdCost->setPredictor(mvPred1Pel);
              bitsMVP1Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 2);
              if (bitsMVP1Pel < curMVPbits)
              {
                curMVPbits = bitsMVP1Pel;
                curMVPIdx = mvpIdxTemp;
                pu.cu->imv = 1;
              }

              if ((cMv.getHor() % 16 == 0) && (cMv.getVer() % 16 == 0))
              {
                unsigned int bitsMVP4Pel = MAX_UINT;
                Mv mvPred4Pel = currAMVPInfo4Pel.mvCand[mvpIdxTemp];
                m_pcRdCost->setPredictor(mvPred4Pel);
                bitsMVP4Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 4);
                if (bitsMVP4Pel < curMVPbits)
                {
                  curMVPbits = bitsMVP4Pel;
                  curMVPIdx = mvpIdxTemp;
                  pu.cu->imv = 2;
                }
              }
            }
          }
          curMVPbits += bitsOnRefIdx;

          m_cDistParam.cur.buf = refBufStart + (*it).y*refStride + (*it).x;
          Distortion currSad = m_cDistParam.distFunc(m_cDistParam);
          Distortion currCost = currSad + m_pcRdCost->getCost(curMVPbits);

          if (!isPerfectMatch)
          {
            if (pu.cu->slice->getRefPic(eRefPicList, refIdx)->slices[0]->getSliceQp() <= pu.cu->slice->getSliceQp())
            {
              isPerfectMatch = true;
            }
          }

          if (currCost < bestCost)
          {
            bestCost = currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex = refIdx;
            bestMv = cMv;
            bestMVPIndex = curMVPIdx;
            imvBest = pu.cu->imv;
            if (pu.cu->imv == 2)
            {
              bestMvd = cMv - currAMVPInfo4Pel.mvCand[curMVPIdx];
            }
            else if (pu.cu->imv == 1)
            {
              bestMvd = cMv - currAMVPInfoPel.mvCand[curMVPIdx];
            }
            else
            {
              bestMvd = cMv - currAMVPInfoQPel.mvCand[curMVPIdx];
            }
          }
        }
      }
    }
  }
  delete[] hashValue1s;
  delete[] hashValue2s;
  pu.cu->imv = imvBest;
  if (bestMvd == Mv(0, 0))
  {
    pu.cu->imv = 0;
    return false;
  }
  return (bestCost < MAX_INT);
}

bool InterSearch::xHashInterEstimation(PredictionUnit& pu, RefPicList& bestRefPicList, int& bestRefIndex, Mv& bestMv, Mv& bestMvd, int& bestMVPIndex, bool& isPerfectMatch)
{
  int width = pu.cu->lumaSize().width;
  int height = pu.cu->lumaSize().height;
  if (width != height)
  {
    return xRectHashInterEstimation(pu, bestRefPicList, bestRefIndex, bestMv, bestMvd, bestMVPIndex, isPerfectMatch);
  }
  int xPos = pu.cu->lumaPos().x;
  int yPos = pu.cu->lumaPos().y;

  uint32_t hashValue1;
  uint32_t hashValue2;
  Distortion bestCost = UINT64_MAX;

  if (!TComHash::getBlockHashValue((pu.cs->picture->getOrigBuf()), width, height, xPos, yPos, pu.cu->slice->getSPS()->getBitDepths(), hashValue1, hashValue2))
  {
    return false;
  }

  BlockHash currBlockHash;
  currBlockHash.x = xPos;
  currBlockHash.y = yPos;
  currBlockHash.hashValue2 = hashValue2;

  m_pcRdCost->setDistParam(m_cDistParam, pu.cs->getOrgBuf(pu).Y(), 0, 0, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, false);

  int imvBest = 0;

  int numPredDir = pu.cu->slice->isInterP() ? 1 : 2;
  for (int refList = 0; refList < numPredDir; refList++)
  {
    RefPicList eRefPicList = (refList == 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    int refPicNumber = pu.cu->slice->getNumRefIdx(eRefPicList);


    for (int refIdx = 0; refIdx < refPicNumber; refIdx++)
    {
      int bitsOnRefIdx = 1;
      if (refPicNumber > 1)
      {
        bitsOnRefIdx += refIdx + 1;
        if (refIdx == refPicNumber - 1)
        {
          bitsOnRefIdx--;
        }
      }
      m_numHashMVStoreds[eRefPicList][refIdx] = 0;

      const std::pair<int, int>& scaleRatio = pu.cu->slice->getScalingRatio( eRefPicList, refIdx );
      if( scaleRatio != SCALE_1X )
      {
        continue;
      }

      CHECK( pu.cu->slice->getRefPic( eRefPicList, refIdx )->getHashMap() == nullptr, "Hash table is not initialized" );

      if (refList == 0 || pu.cu->slice->getList1IdxToList0Idx(refIdx) < 0)
      {
        int count = static_cast<int>(pu.cu->slice->getRefPic(eRefPicList, refIdx)->getHashMap()->count(hashValue1));
        if (count == 0)
        {
          continue;
        }

        list<BlockHash> listBlockHash;
        selectMatchesInter(pu.cu->slice->getRefPic(eRefPicList, refIdx)->getHashMap()->getFirstIterator(hashValue1), count, listBlockHash, currBlockHash);
        m_numHashMVStoreds[eRefPicList][refIdx] = (int)listBlockHash.size();
        if (listBlockHash.empty())
        {
          continue;
        }
        AMVPInfo currAMVPInfoPel;
        AMVPInfo currAMVPInfo4Pel;
        pu.cu->imv = 2;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfo4Pel
#if TM_AMVP
                      , this
#endif
        );
        pu.cu->imv = 1;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfoPel
#if TM_AMVP
                      , this
#endif
        );
        AMVPInfo currAMVPInfoQPel;
        pu.cu->imv = 0;
        PU::fillMvpCand(pu, eRefPicList, refIdx, currAMVPInfoQPel
#if TM_AMVP
                      , this
#endif
        );
#if TM_AMVP
        CHECK(currAMVPInfoPel.numCand != currAMVPInfoQPel.numCand, "The number of full-Pel AMVP candidates and that of Q-Pel should be identical");
        CHECK(currAMVPInfoPel.numCand != currAMVPInfo4Pel.numCand, "The number of full-Pel AMVP candidates and that of 4-Pel should be identical");
        const uint8_t amvpNumCand = currAMVPInfoPel.numCand;
        CHECK(currAMVPInfoPel.numCand == 0, "Wrong")
        for (int mvpIdxTemp = 0; mvpIdxTemp < amvpNumCand; mvpIdxTemp++)
#else
        CHECK(currAMVPInfoPel.numCand <= 1, "Wrong")
        for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
#endif
        {
          currAMVPInfoQPel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
          currAMVPInfoPel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
          currAMVPInfo4Pel.mvCand[mvpIdxTemp].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
        }

        bool wrap = pu.cu->slice->getRefPic(eRefPicList, refIdx)->isWrapAroundEnabled( pu.cs->pps );
        const Pel* refBufStart = pu.cu->slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMPONENT_Y).buf;
        const int refStride = pu.cu->slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMPONENT_Y).stride;

        m_cDistParam.cur.stride = refStride;

        m_pcRdCost->selectMotionLambda( );
        m_pcRdCost->setCostScale(0);

        list<BlockHash>::iterator it;
        int countMV = 0;
        for (it = listBlockHash.begin(); it != listBlockHash.end(); ++it)
        {
          int curMVPIdx = 0;
          unsigned int curMVPbits = MAX_UINT;
          Mv cMv((*it).x - currBlockHash.x, (*it).y - currBlockHash.y);
          m_hashMVStoreds[eRefPicList][refIdx][countMV++] = cMv;
          cMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_QUARTER);

#if TM_AMVP
          for (int mvpIdxTemp = 0; mvpIdxTemp < amvpNumCand; mvpIdxTemp++)
#else
          for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
#endif
          {
            Mv cMvPredPel = currAMVPInfoQPel.mvCand[mvpIdxTemp];
            m_pcRdCost->setPredictor(cMvPredPel);

            unsigned int tempMVPbits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0);

            if (tempMVPbits < curMVPbits)
            {
              curMVPbits = tempMVPbits;
              curMVPIdx = mvpIdxTemp;
              pu.cu->imv = 0;
            }

            if (pu.cu->slice->getSPS()->getAMVREnabledFlag())
            {
              unsigned int bitsMVP1Pel = MAX_UINT;
              Mv mvPred1Pel = currAMVPInfoPel.mvCand[mvpIdxTemp];
              m_pcRdCost->setPredictor(mvPred1Pel);
              bitsMVP1Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 2);
              if (bitsMVP1Pel < curMVPbits)
              {
                curMVPbits = bitsMVP1Pel;
                curMVPIdx = mvpIdxTemp;
                pu.cu->imv = 1;
              }

              if ((cMv.getHor() % 16 == 0) && (cMv.getVer() % 16 == 0))
              {
                unsigned int bitsMVP4Pel = MAX_UINT;
                Mv mvPred4Pel = currAMVPInfo4Pel.mvCand[mvpIdxTemp];
                m_pcRdCost->setPredictor(mvPred4Pel);
                bitsMVP4Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 4);
                if (bitsMVP4Pel < curMVPbits)
                {
                  curMVPbits = bitsMVP4Pel;
                  curMVPIdx = mvpIdxTemp;
                  pu.cu->imv = 2;
                }
              }
            }
          }

          curMVPbits += bitsOnRefIdx;

          m_cDistParam.cur.buf = refBufStart + (*it).y*refStride + (*it).x;
          Distortion currSad = m_cDistParam.distFunc(m_cDistParam);
          Distortion currCost = currSad + m_pcRdCost->getCost(curMVPbits);

          if (!isPerfectMatch)
          {
            if (pu.cu->slice->getRefPic(eRefPicList, refIdx)->slices[0]->getSliceQp() <= pu.cu->slice->getSliceQp())
            {
              isPerfectMatch = true;
            }
          }

          if (currCost < bestCost)
          {
            bestCost = currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex = refIdx;
            bestMv = cMv;
            bestMVPIndex = curMVPIdx;
            imvBest = pu.cu->imv;
            if (pu.cu->imv == 2)
            {
              bestMvd = cMv - currAMVPInfo4Pel.mvCand[curMVPIdx];
            }
            else if (pu.cu->imv == 1)
            {
              bestMvd = cMv - currAMVPInfoPel.mvCand[curMVPIdx];
            }
            else
            {
              bestMvd = cMv - currAMVPInfoQPel.mvCand[curMVPIdx];
            }
          }
        }
      }
    }
  }
  pu.cu->imv = imvBest;
  if (bestMvd == Mv(0, 0))
  {
    pu.cu->imv = 0;
    return false;
  }
  return (bestCost < MAX_INT);
}

bool InterSearch::predInterHashSearch(CodingUnit& cu, Partitioner& partitioner, bool& isPerfectMatch)
{
  Mv       bestMv, bestMvd;
  RefPicList   bestRefPicList;
  int          bestRefIndex;
  int          bestMVPIndex;

  auto &pu = *cu.firstPU;

  Mv cMvZero;
  pu.mv[REF_PIC_LIST_0] = Mv();
  pu.mv[REF_PIC_LIST_1] = Mv();
  pu.mvd[REF_PIC_LIST_0] = cMvZero;
  pu.mvd[REF_PIC_LIST_1] = cMvZero;
  pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

  if (xHashInterEstimation(pu, bestRefPicList, bestRefIndex, bestMv, bestMvd, bestMVPIndex, isPerfectMatch))
  {
    pu.interDir = static_cast<int>(bestRefPicList) + 1;
    pu.mv[bestRefPicList] = bestMv;
    pu.mv[bestRefPicList].changePrecision(MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL);

    pu.mvd[bestRefPicList] = bestMvd;
    pu.mvd[bestRefPicList].changePrecision(MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (!PU::useRefPairList(pu) && !PU::useRefCombList(pu))
#endif
    if (pu.isMvdPredApplicable())
    {
      std::vector<Mv> cMvdDerivedVec;
      Mv cMvPred = pu.mv[bestRefPicList] - pu.mvd[bestRefPicList];
      Mv cMvdKnownAtDecoder = Mv(pu.mvd[bestRefPicList].getAbsHor(), pu.mvd[bestRefPicList].getAbsVer());
#if JVET_AD0140_MVD_PREDICTION
      const auto& motionModel = ( 0 != pu.cu->smvdMode ) ? MotionModel::BiTranslationalSmvd : 
                                                          (3 == pu.interDir) ? MotionModel::BiTranslational : MotionModel::UniTranslational;
      CHECK(pu.cu->affine != 0, "Affine motion model is specified in the encoder Hash Search ");
      pu.mvdSuffixInfo.initPrefixesMvd( 0, bestRefPicList, pu.mvd[bestRefPicList], pu.cu->imv, true, motionModel );
      pu.mvdSuffixInfo.getBinBudgetForMv(MvdSuffixInfoMv::getBinBudgetForPrediction(pu.Y().width, pu.Y().height, pu.cu->imv), bestRefPicList);
      deriveMvdSign(cMvPred, cMvdKnownAtDecoder, pu, bestRefPicList, bestRefIndex, cMvdDerivedVec);
      int idx = deriveMVSDIdxFromMVDTransSI(pu.mvd[bestRefPicList], cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[bestRefPicList][0]);
      initOffsetsMvd(pu.mvd[bestRefPicList], cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[bestRefPicList][0], pu.cu->imv);
#else
      deriveMvdSign(cMvPred, cMvdKnownAtDecoder, pu, bestRefPicList, bestRefIndex, cMvdDerivedVec);
      int idx = deriveMVSDIdxFromMVDTrans(pu.mvd[bestRefPicList], cMvdDerivedVec);
#endif
      CHECK(idx == -1, "");
      pu.mvsdIdx[bestRefPicList] = idx;
#if JVET_AD0140_MVD_PREDICTION
      defineSignHypMatch(pu.mvd[bestRefPicList], pu.mvdSuffixInfo.mvBins[bestRefPicList][0], pu.mvsdIdx[bestRefPicList]);
#endif
    }
#endif
    pu.refIdx[bestRefPicList] = bestRefIndex;
    pu.mvpIdx[bestRefPicList] = bestMVPIndex;

#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
    pu.mvpNum[bestRefPicList] = PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(bestRefPicList, bestRefIndex)) ? 1 : 2;
#else
    pu.mvpNum[bestRefPicList] = 1;
#endif
#else
    pu.mvpNum[bestRefPicList] = 2;
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (PU::useRefCombList(pu))
    {
      setUniRefIdxLC(pu);
    }
    else if (PU::useRefPairList(pu))
    {
      setBiRefPairIdx(pu);
    }
#endif

    PU::spanMotionInfo(pu);
    PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
    motionCompensation(pu, predBuf, REF_PIC_LIST_X);
    return true;
  }
  else
  {
    return false;
  }

  return true;
}


//! search of the best candidate for inter prediction
#if JVET_X0083_BM_AMVP_MERGE_MODE
void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner, bool& bdmvrAmMergeNotValid,
#if JVET_AD0213_LIC_IMP
    MvField* mvFieldAmListCommon, bool* licAmListCommon, Mv* mvBufEncAmBDmvrL0, Mv* mvBufEncAmBDmvrL1)
#else
    MvField* mvFieldAmListCommon, Mv* mvBufEncAmBDmvrL0, Mv* mvBufEncAmBDmvrL1)
#endif
#else
void InterSearch::predInterSearch(CodingUnit& cu, Partitioner& partitioner)
#endif
{
  CodingStructure& cs = *cu.cs;

  AMVPInfo     amvp[2];
  Mv           cMvSrchRngLT;
  Mv           cMvSrchRngRB;

  Mv           cMvZero;

  Mv           cMv[2];
  Mv           cMvBi[2];
  Mv           cMvTemp[2][33];
  Mv           cMvHevcTemp[2][33];
  int          iNumPredDir = cs.slice->isInterP() ? 1 : 2;

  Mv           cMvPred[2][33];

  Mv           cMvPredBi[2][33];
  int          aaiMvpIdxBi[2][33];

  int          aaiMvpIdx[2][33];
  int          aaiMvpNum[2][33];

  AMVPInfo     aacAMVPInfo[2][33];

  int          iRefIdx[2]={0,0}; //If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int          iRefIdxBi[2] = { -1, -1 };

  uint32_t         uiMbBits[3] = {1, 1, 0};

  uint32_t         uiLastMode = 0;
  uint32_t         uiLastModeTemp = 0;
  int          iRefStart, iRefEnd;

  int          symMode = 0;

  int          bestBiPRefIdxL1 = 0;
  int          bestBiPMvpL1    = 0;
  Distortion   biPDistTemp     = std::numeric_limits<Distortion>::max();

  uint8_t      bcwIdx          = (cu.cs->slice->isInterB() ? cu.bcwIdx : BCW_DEFAULT);
  bool         enforceBcwPred = false;
  MergeCtx     mergeCtx;

  // Loop over Prediction Units
  CHECK(!cu.firstPU, "CU does not contain any PUs");
  uint32_t         puIdx = 0;
  auto &pu = *cu.firstPU;
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int tryBipred = 0;
  bool checkAffine    = (pu.cu->imv == 0 || pu.cu->slice->getSPS()->getAffineAmvrEnabledFlag()) && pu.cu->imv != IMV_HPEL;
  bool checkNonAffine = pu.cu->imv == 0 || pu.cu->imv == IMV_HPEL || (pu.cu->slice->getSPS()->getAMVREnabledFlag() &&
                                            pu.cu->imv <= (pu.cu->slice->getSPS()->getAMVREnabledFlag() ? IMV_4PEL : 0));
  CodingUnit *bestCU  = pu.cu->cs->bestCS != nullptr ? pu.cu->cs->bestCS->getCU( CHANNEL_TYPE_LUMA ) : nullptr;
  bool trySmvd        = ( bestCU != nullptr && pu.cu->imv == 2 && checkAffine ) ? ( !bestCU->firstPU->mergeFlag && !bestCU->affine ) : true;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  trySmvd &= pu.cu->licDelta == 0;
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  bool bestSubTmvp = (bestCU != NULL && (bestCU->firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP || bestCU->firstPU->amvpSbTmvpFlag)) ? true : false;
  bool tryAmvpSbTmvp = cs.sps->getSbTMVPEnabledFlag() && cs.slice->getAmvpSbTmvpEnabledFlag() && !cu.licFlag && checkNonAffine && bcwIdx == BCW_DEFAULT ? true : false;
  if ( !cs.slice->getAmvpSbTmvpAmvrEnabledFlag() )
  {
    tryAmvpSbTmvp &= pu.cu->imv ? false : true;
  }
  else
  {
    tryAmvpSbTmvp &= pu.cu->imv == 0 || pu.cu->imv == IMV_FPEL ? true : false;
  }
#endif 
  if ( pu.cu->imv && bestCU != nullptr && checkAffine )
  {
    checkAffine = !( bestCU->firstPU->mergeFlag || !bestCU->affine );
  }

#if JVET_X0083_BM_AMVP_MERGE_MODE
  const bool amvpMergeModeFlag = pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1];
  RefPicList refListAmvp       = REF_PIC_LIST_X;
  RefPicList refListMerge      = REF_PIC_LIST_X;
  int candidateRefIdxCount     = 0;
  if (amvpMergeModeFlag)
  {
#if JVET_Y0128_NON_CTC
    if (pu.cu->slice->getUseAmvpMergeMode() == false)
    {
      m_skipPROF = false;
      m_encOnly = false;
      bdmvrAmMergeNotValid = true;
      return;
    }
#endif
    trySmvd = false;
    checkAffine = false;
#if JVET_AG0098_AMVP_WITH_SBTMVP
    tryAmvpSbTmvp = false;
#endif
    refListMerge = pu.amvpMergeModeFlag[0] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    refListAmvp = RefPicList(1 - refListMerge);
#if JVET_AD0213_LIC_IMP
    getAmvpMergeModeMergeList(pu, mvFieldAmListCommon, licAmListCommon);
#else
    getAmvpMergeModeMergeList(pu, mvFieldAmListCommon);
#endif
    for (int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->getNumRefIdx(refListAmvp); iRefIdxTemp++)
    {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      if (mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM].refIdx < 0
          && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + 1].refIdx < 0
          && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + 2].refIdx < 0)
#else
      if (mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS].refIdx < 0 && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS + 1].refIdx < 0)
#endif
      {
        continue;
      }
      candidateRefIdxCount++;
    }
  }
#if JVET_Y0128_NON_CTC || JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  if ( amvpMergeModeFlag && !candidateRefIdxCount )
  {
    m_skipPROF = false;
    m_encOnly = false;
    bdmvrAmMergeNotValid = true;
    return;
  }
#endif
#endif
  if ( pu.cu->imv == 2 && checkNonAffine && pu.cu->slice->getSPS()->getAffineAmvrEnabledFlag() )
  {
#if AMVR_ENC_OPT
    checkNonAffine = m_affineMotion.hevcCost[1] < m_affineMotion.hevcCost[0];
#else
    checkNonAffine = m_affineMotion.hevcCost[1] < m_affineMotion.hevcCost[0] * 1.06f;
#endif
  }

#if JVET_AD0213_LIC_IMP && TM_AMVP
  resetLicEncCtrlPara();
#endif
#if MULTI_HYP_PRED
  const bool saveMeResultsForMHP = cs.sps->getUseInterMultiHyp()
    && bcwIdx != BCW_DEFAULT
    && (pu.Y().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE && std::min(pu.Y().width, pu.Y().height) >= MULTI_HYP_PRED_RESTRICT_MIN_WH)
      ;
#endif
  {
#if JVET_AD0213_LIC_IMP
    m_encMotionEstimation = true;
#endif
    if (pu.cu->cs->bestParent != nullptr && pu.cu->cs->bestParent->getCU(CHANNEL_TYPE_LUMA) != nullptr && pu.cu->cs->bestParent->getCU(CHANNEL_TYPE_LUMA)->affine == false)
    {
      m_skipPROF = true;
    }
    m_encOnly = true;
    // motion estimation only evaluates luma component
    m_maxCompIDToPred = MAX_NUM_COMPONENT;
//    m_maxCompIDToPred = COMPONENT_Y;

    CHECK(pu.cu != &cu, "PU is contained in another CU");

    if (cu.cs->sps->getSbTMVPEnabledFlag())
    {
      Size bufSize = g_miScaling.scale(pu.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if JVET_AG0098_AMVP_WITH_SBTMVP
      for (int i = 0; i < SUB_BUFFER_SIZE; i++)
#else
      for (int i = 0; i < SUB_TMVP_NUM; i++)
#endif
      {
        mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
      }
#else
      mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
    }
#if JVET_AL0160_SBSMVP
    if(cu.cs->sps->getSpatialMVPEnabledFlag())
    {
      Size bufSize = g_miScaling.scale(pu.lumaSize());

      for (int i = 0; i < NUM_SUB_SMVP; i++)
      {
        mergeCtx.subSpatialPuMvpMiBuf[i] = MotionBuf(m_subSpatialPuMiBuf[i], bufSize);
      }
    }
#endif
    Distortion   uiHevcCost = std::numeric_limits<Distortion>::max();
    Distortion   uiAffineCost = std::numeric_limits<Distortion>::max();
    Distortion   uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion   uiCostBi  =   std::numeric_limits<Distortion>::max();
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    Distortion   uiCostTemp = std::numeric_limits<Distortion>::max();
#else
    Distortion   uiCostTemp;
#endif

    uint32_t         uiBits[3];
    uint32_t         uiBitsTemp;
    Distortion   bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion   uiCostTempL0[MAX_NUM_REF];
    for (int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    uint32_t         uiBitsTempL0[MAX_NUM_REF];

    Mv           mvValidList1;
    int          refIdxValidList1 = 0;
    uint32_t         bitsValidList1   = MAX_UINT;
    Distortion   costValidList1   = std::numeric_limits<Distortion>::max();

    PelUnitBuf origBuf = pu.cs->getOrgBuf( pu );

    xGetBlkBits( cs.slice->isInterP(), puIdx, uiLastMode, uiMbBits );

    m_pcRdCost->selectMotionLambda( );

    unsigned imvShift = pu.cu->imv == IMV_HPEL ? 1 : (pu.cu->imv << 1);
    if ( checkNonAffine )
    {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!amvpMergeModeFlag)
      {
#endif
      //  Uni-directional prediction
      for ( int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
      {
        RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
        for (int iRefIdxTemp = 0; iRefIdxTemp < cs.slice->getNumRefIdx(eRefPicList); iRefIdxTemp++)
        {
          uiBitsTemp = uiMbBits[iRefList];
          if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
          {
            uiBitsTemp += iRefIdxTemp+1;
            if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
            {
              uiBitsTemp--;
            }
          }
          xEstimateMvPredAMVP( pu, origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], amvp[eRefPicList], false, &biPDistTemp);

          aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
          aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];

          if(cs.picHeader->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist)
          {
            bestBiPDist = biPDistTemp;
            bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
            bestBiPRefIdxL1 = iRefIdxTemp;
          }

#if TM_AMVP
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][aaiMvpNum[iRefList][iRefIdxTemp]];
#else
          uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
#endif

          if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )    // list 1
          {
            if ( cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) >= 0 )
            {
              cMvTemp[1][iRefIdxTemp] = cMvTemp[0][cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              uiCostTemp = uiCostTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )];
              /*first subtract the bit-rate part of the cost of the other list*/
              uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[cs.slice->getList1IdxToList0Idx( iRefIdxTemp )] );
              /*correct the bit-rate part of the current ref*/
              m_pcRdCost->setPredictor  ( cMvPred[iRefList][iRefIdxTemp] );
              uiBitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( cMvTemp[1][iRefIdxTemp].getHor(), cMvTemp[1][iRefIdxTemp].getVer(), imvShift + MV_FRACTIONAL_BITS_DIFF );
              /*calculate the correct cost*/
              uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
            }
            else
            {
              xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
            }
          }
          else
          {
            xMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList] );
          }
          if( cu.cs->sps->getUseBcw() && cu.bcwIdx == BCW_DEFAULT && cu.cs->slice->isInterB() )
          {
            const bool checkIdentical = true;
            m_uniMotions.setReadMode(checkIdentical, (uint32_t)iRefList, (uint32_t)iRefIdxTemp);
            m_uniMotions.copyFrom(cMvTemp[iRefList][iRefIdxTemp], uiCostTemp - m_pcRdCost->getCost(uiBitsTemp), (uint32_t)iRefList, (uint32_t)iRefIdxTemp);
          }
          xCopyAMVPInfo( &amvp[eRefPicList], &aacAMVPInfo[iRefList][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
          xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp, pu.cu->imv );

          if ( iRefList == 0 )
          {
            uiCostTempL0[iRefIdxTemp] = uiCostTemp;
            uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
          }
          if ( uiCostTemp < uiCost[iRefList] )
          {
            uiCost[iRefList] = uiCostTemp;
            uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

            // set motion
            cMv    [iRefList] = cMvTemp[iRefList][iRefIdxTemp];
            iRefIdx[iRefList] = iRefIdxTemp;
          }
#if JVET_Z0054_BLK_REF_PIC_REORDER
          if (cu.cs->sps->getUseARL() && iRefList == 1 && cs.slice->getList1IdxToList0Idx(iRefIdxTemp) >= 0)
          {
            uiCostTemp = MAX_UINT;
          }
#endif

          if ( iRefList == 1 && uiCostTemp < costValidList1 && cs.slice->getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
          {
            costValidList1 = uiCostTemp;
            bitsValidList1 = uiBitsTemp;

            // set motion
            mvValidList1     = cMvTemp[iRefList][iRefIdxTemp];
            refIdxValidList1 = iRefIdxTemp;
          }
        }
      }

      ::memcpy(cMvHevcTemp, cMvTemp, sizeof(cMvTemp));
      if (cu.imv == 0 && (!cu.slice->getSPS()->getUseBcw() || bcwIdx == BCW_DEFAULT))
      {
        insertUniMvCands(pu.Y(), cMvTemp);

        unsigned idx1, idx2, idx3, idx4;
        getAreaIdx(cu.Y(), *cu.slice->getPPS()->pcv, idx1, idx2, idx3, idx4);
#if INTER_LIC
        if (cu.slice->getUseLIC() && cu.licFlag)
        {
          ::memcpy(&(g_reusedUniMVsLIC[idx1][idx2][idx3][idx4][0][0]), cMvTemp, 2 * 33 * sizeof(Mv));
          g_isReusedUniMVsFilledLIC[idx1][idx2][idx3][idx4] = true;
        }
        else
        {
#endif
        ::memcpy(&(g_reusedUniMVs[idx1][idx2][idx3][idx4][0][0]), cMvTemp, 2 * 33 * sizeof(Mv));
        g_isReusedUniMVsFilled[idx1][idx2][idx3][idx4] = true;
#if INTER_LIC
        }
#endif
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
      //  Bi-predictive Motion estimation
#if JVET_AD0213_LIC_IMP
      bool condOn = cu.slice->getCheckLDC() || bcwIdx == BCW_DEFAULT || !m_affineModeSelected || !m_pcEncCfg->getUseBcwFast() || (cu.licFlag && bcwIdx != BCW_DEFAULT);
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
      condOn &= cu.licDelta == 0;
#endif
      if( ( cs.slice->isInterB() ) && ( PU::isBipredRestriction( pu ) == false )
#if JVET_AD0213_LIC_IMP
        && condOn
#else
        && (cu.slice->getCheckLDC() || bcwIdx == BCW_DEFAULT || !m_affineModeSelected || !m_pcEncCfg->getUseBcwFast())
#endif
#if INTER_LIC && !JVET_AD0213_LIC_IMP
        && !cu.licFlag
#endif
        )
      {
        bool doBiPred = true;
        tryBipred = 1;
        cMvBi[0] = cMv[0];
        cMvBi[1] = cMv[1];
        iRefIdxBi[0] = iRefIdx[0];
        iRefIdxBi[1] = iRefIdx[1];

        ::memcpy( cMvPredBi,   cMvPred,   sizeof( cMvPred   ) );
        ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof( aaiMvpIdx ) );

        uint32_t uiMotBits[2];

#if JVET_X0083_BM_AMVP_MERGE_MODE
        if(cs.picHeader->getMvdL1ZeroFlag() && !pu.amvpMergeModeFlag[1])
#else
        if(cs.picHeader->getMvdL1ZeroFlag())
#endif
        {
          xCopyAMVPInfo(&aacAMVPInfo[1][bestBiPRefIdxL1], &amvp[REF_PIC_LIST_1]);
          aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;
          cMvPredBi  [1][bestBiPRefIdxL1] = amvp[REF_PIC_LIST_1].mvCand[bestBiPMvpL1];

          cMvBi    [1] = cMvPredBi[1][bestBiPRefIdxL1];
          iRefIdxBi[1] = bestBiPRefIdxL1;
          pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
          pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
          pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;

          if( m_pcEncCfg->getMCTSEncConstraint() )
          {
            Mv restrictedMv = pu.mv[REF_PIC_LIST_1];
            Area curTileAreaRestricted;
            curTileAreaRestricted = pu.cs->picture->mctsInfo.getTileAreaSubPelRestricted( pu );
            MCTSHelper::clipMvToArea( restrictedMv, pu.cu->Y(), curTileAreaRestricted, *pu.cs->sps );
            // If sub-pel filter samples are not inside of allowed area
            if( restrictedMv != pu.mv[REF_PIC_LIST_1] )
            {
              uiCostBi = std::numeric_limits<Distortion>::max();
              doBiPred = false;
            }
          }
          PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf( UnitAreaRelative(cu, pu) );
          motionCompensation( pu, predBufTmp, REF_PIC_LIST_1 );

          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiMbBits[1];

          if ( cs.slice->getNumRefIdx(REF_PIC_LIST_1) > 1 )
          {
            uiMotBits[1] += bestBiPRefIdxL1 + 1;
            if ( bestBiPRefIdxL1 == cs.slice->getNumRefIdx(REF_PIC_LIST_1)-1 )
            {
              uiMotBits[1]--;
            }
          }

#if TM_AMVP
          uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][amvp[REF_PIC_LIST_1].numCand];
#else
          uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];
#endif

          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];

          cMvTemp[1][bestBiPRefIdxL1] = cMvBi[1];
        }
        else
        {
          uiMotBits[0] = uiBits[0] - uiMbBits[0];
          uiMotBits[1] = uiBits[1] - uiMbBits[1];
          uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
        }
        if( doBiPred )
        {
        // 4-times iteration (default)
        int iNumIter = 4;

        // fast encoder setting: only one iteration
        if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 || cs.picHeader->getMvdL1ZeroFlag() )
        {
          iNumIter = 1;
        }


#if JVET_X0083_BM_AMVP_MERGE_MODE
        if (amvpMergeModeFlag)
        {
          iNumIter = 1;
        }
#endif

        enforceBcwPred = (bcwIdx != BCW_DEFAULT);
        for ( int iIter = 0; iIter < iNumIter; iIter++ )
        {
          int         iRefList    = iIter % 2;

#if JVET_X0083_BM_AMVP_MERGE_MODE
          if (amvpMergeModeFlag)
          {
            iRefList = pu.amvpMergeModeFlag[1] ? 0 : 1;
          }
          else
#endif
          if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 )
          {
            if( uiCost[0] <= uiCost[1] )
            {
              iRefList = 1;
            }
            else
            {
              iRefList = 0;
            }
            if( bcwIdx != BCW_DEFAULT )
            {
              iRefList = ( abs( getBcwWeight(bcwIdx, REF_PIC_LIST_0 ) ) > abs( getBcwWeight(bcwIdx, REF_PIC_LIST_1 ) ) ? 1 : 0 );
            }
          }
          else if ( iIter == 0 )
          {
            iRefList = 0;
          }
#if JVET_X0083_BM_AMVP_MERGE_MODE
          if ( iIter == 0 && !cs.picHeader->getMvdL1ZeroFlag() && !amvpMergeModeFlag)
#else
          if ( iIter == 0 && !cs.picHeader->getMvdL1ZeroFlag())
#endif
          {
            pu.mv    [1 - iRefList] = cMv    [1 - iRefList];
            pu.refIdx[1 - iRefList] = iRefIdx[1 - iRefList];

            PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(cu, pu) );
            motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
          }

          RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

          if(cs.picHeader->getMvdL1ZeroFlag())
          {
            iRefList = 0;
            eRefPicList = REF_PIC_LIST_0;
          }

          bool bChanged = false;
          iRefStart = 0;
          iRefEnd   = cs.slice->getNumRefIdx(eRefPicList)-1;
          for (int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++)
          {
#if JVET_X0083_BM_AMVP_MERGE_MODE
          int numberBestMvpIdxLoop = 1;
          int selectedBestMvpIdx = -1;
          Mv selectedBestMv;
          if (amvpMergeModeFlag)
          {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
            if (mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM].refIdx < 0
                && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + 1].refIdx < 0
                && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + 2].refIdx < 0)
#else
            if (mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS].refIdx < 0 && mvFieldAmListCommon[iRefIdxTemp * AMVP_MAX_NUM_CANDS + 1].refIdx < 0)
#endif
            {
              continue;
            }
            xEstimateMvPredAMVP( pu, origBuf, refListAmvp, iRefIdxTemp, cMvPred[refListAmvp][iRefIdxTemp], amvp[refListAmvp], false, &biPDistTemp, mvFieldAmListCommon);
            xCopyAMVPInfo( &amvp[refListAmvp], &aacAMVPInfo[refListAmvp][iRefIdxTemp]); // must always be done ( also when AMVP_MODE = AM_NONE )
            numberBestMvpIdxLoop = amvp[eRefPicList].numCand;
          }

          for (int bestMvpIdxLoop = 0; bestMvpIdxLoop < numberBestMvpIdxLoop; bestMvpIdxLoop++)
          {
            if (amvpMergeModeFlag)
            {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
              const int mvFieldMergeIdx = iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + bestMvpIdxLoop;
#else
              const int mvFieldMergeIdx = iRefIdxTemp * AMVP_MAX_NUM_CANDS + bestMvpIdxLoop;
#endif
              aaiMvpIdxBi[iRefList][iRefIdxTemp] = bestMvpIdxLoop;
              unsigned idx1, idx2, idx3, idx4;
              getAreaIdx(cu.Y(), *cu.slice->getPPS()->pcv, idx1, idx2, idx3, idx4);
              CHECK(g_isReusedUniMVsFilled[idx1][idx2][idx3][idx4] == false, "this is not possible");
              if (g_isReusedUniMVsFilled[idx1][idx2][idx3][idx4])
              {
                cMvTemp[iRefList][iRefIdxTemp] = g_reusedUniMVs[idx1][idx2][idx3][idx4][refListAmvp][iRefIdxTemp];
              }
              else
              {
                cMvTemp[iRefList][iRefIdxTemp] = amvp[eRefPicList].mvCand[bestMvpIdxLoop];
              }
              cMvPredBi[iRefList][iRefIdxTemp] = amvp[eRefPicList].mvCand[bestMvpIdxLoop];
              // set merge dir mv info and MC
              pu.mv[1 - iRefList] = mvFieldAmListCommon[mvFieldMergeIdx].mv;
              pu.refIdx[1 - iRefList] = mvFieldAmListCommon[mvFieldMergeIdx].refIdx;
            }
#endif
#if !JVET_AD0213_LIC_IMP
            if( m_pcEncCfg->getUseBcwFast() && (bcwIdx != BCW_DEFAULT)
              && (pu.cu->slice->getRefPic(eRefPicList, iRefIdxTemp)->getPOC() == pu.cu->slice->getRefPic(RefPicList(1 - iRefList), pu.refIdx[1 - iRefList])->getPOC())
              && (!pu.cu->imv && pu.cu->slice->getTLayer()>1)
#if INTER_LIC
              && !cu.licFlag
#endif
              )
            {
              continue;
            }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
            if (amvpMergeModeFlag)
            {
              uiBitsTemp = uiMbBits[2];
            }
            else
            {
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
              if (cu.cs->sps->getUseARL())
              {
                int refIdxTemp[2];
                refIdxTemp[iRefList] = iRefIdxTemp;
                refIdxTemp[1 - iRefList] = iRefIdxBi[1 - iRefList];
                if (pu.cu->slice->getRefPicPairIdx(refIdxTemp[0], refIdxTemp[1]) < 0)
                {
                  continue;
                }
              }
#endif
            uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
            uiBitsTemp += ((cs.slice->getSPS()->getUseBcw() == true) ? getWeightIdxBits(bcwIdx) : 0);
#if JVET_X0083_BM_AMVP_MERGE_MODE
            }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
            if (( cs.slice->getNumRefIdx(eRefPicList) > 1 ) && !(amvpMergeModeFlag && candidateRefIdxCount <= 1))
#else
            if ( cs.slice->getNumRefIdx(eRefPicList) > 1 )
#endif
            {
              uiBitsTemp += iRefIdxTemp+1;
              if ( iRefIdxTemp == cs.slice->getNumRefIdx(eRefPicList)-1 )
              {
                uiBitsTemp--;
              }
            }
#if TM_AMVP
            uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][aacAMVPInfo[iRefList][iRefIdxTemp].numCand];
#else
            uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
#endif
            if ( cs.slice->getBiDirPred() )
            {
#if JVET_X0083_BM_AMVP_MERGE_MODE
              if (!amvpMergeModeFlag)
#endif
              uiBitsTemp += 1; // add one bit for symmetrical MVD mode
            }
#if MULTI_HYP_PRED
            if (saveMeResultsForMHP)
              uiBitsTemp++; // terminating 0 mh_flag
#endif
            // call ME
#if JVET_X0083_BM_AMVP_MERGE_MODE
            if (amvpMergeModeFlag)
            {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
              if (bestMvpIdxLoop < 2)
              {
                MvField amvpMvField, mergeMvField;
                amvpMvField.setMvField(cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp);
                mergeMvField.setMvField(cMvPredBi[iRefList][iRefIdxTemp].getSymmvdMv(cMvPredBi[iRefList][iRefIdxTemp], pu.mv[1 - iRefList]), pu.refIdx[1 - iRefList]);
#if JVET_AD0213_LIC_IMP
                int mvFieldMergeIdx = iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + bestMvpIdxLoop;
                CHECK(licAmListCommon[mvFieldMergeIdx] != licAmListCommon[MAX_NUM_AMVP_CANDS_MAX_REF + mvFieldMergeIdx], "inconsistent LIC value");
                int orgLicFlag = pu.cu->licFlag;
                pu.cu->licFlag = licAmListCommon[mvFieldMergeIdx];
#endif
                uiCostTemp = xGetSymmetricCost( pu, origBuf, eRefPicList, amvpMvField, mergeMvField, bcwIdx );
#if JVET_AD0213_LIC_IMP
                pu.cu->licFlag = orgLicFlag;
#endif
                uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
                cMvTemp[iRefList][iRefIdxTemp] = amvpMvField.mv;
              }
              else
              {
#endif
              PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(cu, pu) );
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
              motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList), true, false );
#else
              motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
#endif
#if MULTI_HYP_PRED
              CHECK(pu.addHypData.empty() == false, "this is not possible");
#endif
#if JVET_AD0213_LIC_IMP
              int mvFieldMergeIdx = iRefIdxTemp * AMVP_MAX_NUM_CANDS_MEM + bestMvpIdxLoop;
              int orgLicFlag = pu.cu->licFlag;
              pu.cu->licFlag = licAmListCommon[mvFieldMergeIdx];
#endif
              xMotionEstimation ( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList], true );
#if JVET_AD0213_LIC_IMP
              pu.cu->licFlag = orgLicFlag;
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
              }
#endif
            }
            else
            {
#endif
            xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[eRefPicList] );
            xMotionEstimation ( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp, amvp[eRefPicList], true );
            xCheckBestMVP( eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], amvp[eRefPicList], uiBitsTemp, uiCostTemp, pu.cu->imv);
#if JVET_X0083_BM_AMVP_MERGE_MODE
            }
#endif
#if MULTI_HYP_PRED
            if (saveMeResultsForMHP)
            {
              // AMVP bi
              MEResult biPredResult;
              biPredResult.cu = cu;
              biPredResult.pu = pu;
              biPredResult.pu.interDir = 3;
              biPredResult.pu.mv[iRefList] = cMvTemp[iRefList][iRefIdxTemp];
              biPredResult.pu.mv[1 - iRefList] = cMvBi[1 - iRefList];
              biPredResult.pu.mv[0].mvCliptoStorageBitDepth();
              biPredResult.pu.mv[1].mvCliptoStorageBitDepth();

              biPredResult.pu.mvd[iRefList] = cMvTemp[iRefList][iRefIdxTemp] - cMvPredBi[iRefList][iRefIdxTemp];
              biPredResult.pu.mvd[1 - iRefList] = cMvBi[1 - iRefList] - cMvPredBi[1 - iRefList][iRefIdxBi[1 - iRefList]];
              biPredResult.pu.refIdx[iRefList] = iRefIdxTemp;
              biPredResult.pu.refIdx[1 - iRefList] = iRefIdxBi[1 - iRefList];
              biPredResult.pu.mvpIdx[iRefList] = aaiMvpIdxBi[iRefList][iRefIdxTemp];
              biPredResult.pu.mvpIdx[1 - iRefList] = aaiMvpIdxBi[1 - iRefList][iRefIdxBi[1 - iRefList]];
              biPredResult.pu.mvpNum[iRefList] = aaiMvpNum[iRefList][iRefIdxTemp];
              biPredResult.pu.mvpNum[1 - iRefList] = aaiMvpNum[1 - iRefList][iRefIdxBi[1 - iRefList]];
              biPredResult.cost = uiCostTemp;
              biPredResult.bits = uiBitsTemp;

              if (!(cu.imv != 0 && biPredResult.pu.mvd[0] == Mv(0, 0) && biPredResult.pu.mvd[1] == Mv(0, 0)))
              {
                cs.m_meResults.push_back(biPredResult);
              }

            }
#endif
            if ( uiCostTemp < uiCostBi )
            {
              bChanged = true;

              cMvBi[iRefList]     = cMvTemp[iRefList][iRefIdxTemp];
              iRefIdxBi[iRefList] = iRefIdxTemp;
#if JVET_X0083_BM_AMVP_MERGE_MODE
              if (amvpMergeModeFlag)
              {
                selectedBestMvpIdx = bestMvpIdxLoop;
                selectedBestMv = cMvTemp[iRefList][iRefIdxTemp];
              }
#endif

              uiCostBi            = uiCostTemp;
#if JVET_X0083_BM_AMVP_MERGE_MODE
              if (amvpMergeModeFlag)
              {
                uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2];
              }
              else
              {
#endif
              uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
              uiMotBits[iRefList] -= ((cs.slice->getSPS()->getUseBcw() == true) ? getWeightIdxBits(bcwIdx) : 0);
#if JVET_X0083_BM_AMVP_MERGE_MODE
              }
#endif
              uiBits[2]           = uiBitsTemp;

              if(iNumIter!=1)
              {
                //  Set motion
                pu.mv    [eRefPicList] = cMvBi    [iRefList];
                pu.refIdx[eRefPicList] = iRefIdxBi[iRefList];

                PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf( UnitAreaRelative(cu, pu) );
                motionCompensation( pu, predBufTmp, eRefPicList );
              }
            }
#if JVET_X0083_BM_AMVP_MERGE_MODE
          } // for loop-bestMvpIdxLoop

          if (amvpMergeModeFlag && selectedBestMvpIdx >= 0)
          {
            aaiMvpIdxBi[iRefList][iRefIdxTemp] = selectedBestMvpIdx;
            xCopyAMVPInfo(&aacAMVPInfo[iRefList][iRefIdxTemp], &amvp[eRefPicList] );
            cMvTemp[iRefList][iRefIdxTemp] = selectedBestMv;
            cMvPredBi[iRefList][iRefIdxTemp] = amvp[eRefPicList].mvCand[selectedBestMvpIdx];
          }
#endif
          } // for loop-iRefIdxTemp

          if ( !bChanged )
          {
            if ((uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) || enforceBcwPred)
            {
              xCopyAMVPInfo(&aacAMVPInfo[0][iRefIdxBi[0]], &amvp[REF_PIC_LIST_0]);
              xCheckBestMVP( REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], amvp[REF_PIC_LIST_0], uiBits[2], uiCostBi, pu.cu->imv);
              if(!cs.picHeader->getMvdL1ZeroFlag())
              {
                xCopyAMVPInfo(&aacAMVPInfo[1][iRefIdxBi[1]], &amvp[REF_PIC_LIST_1]);
                xCheckBestMVP( REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], amvp[REF_PIC_LIST_1], uiBits[2], uiCostBi, pu.cu->imv);
              }
            }
            break;
          }
        } // for loop-iter
        }
        cu.refIdxBi[0] = iRefIdxBi[0];
        cu.refIdxBi[1] = iRefIdxBi[1];
        if ( cs.slice->getBiDirPred() && trySmvd )
        {
          Distortion symCost;
          Mv cMvPredSym[2];
          int mvpIdxSym[2];

          int curRefList = REF_PIC_LIST_0;
          int tarRefList = 1 - curRefList;
          RefPicList eCurRefList = (curRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
          int refIdxCur = cs.slice->getSymRefIdx( curRefList );
          int refIdxTar = cs.slice->getSymRefIdx( tarRefList );
          CHECK (refIdxCur==-1 || refIdxTar==-1, "Uninitialized reference index not allowed");

#if TM_AMVP
          unsigned amvpNumCandCur = aacAMVPInfo[curRefList][refIdxCur].numCand;
          unsigned amvpNumCandTar = aacAMVPInfo[tarRefList][refIdxTar].numCand;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          if (!pu.cs->sps->getUseTMAmvpMode())
#else
          if (!pu.cs->sps->getUseDMVDMode())
#endif
          {
            amvpNumCandCur = AMVP_MAX_NUM_CANDS;
            amvpNumCandTar = AMVP_MAX_NUM_CANDS;
          }
#endif
          if ( aacAMVPInfo[curRefList][refIdxCur].mvCand[0] == aacAMVPInfo[curRefList][refIdxCur].mvCand[1] )
            aacAMVPInfo[curRefList][refIdxCur].numCand = 1;
          if ( aacAMVPInfo[tarRefList][refIdxTar].mvCand[0] == aacAMVPInfo[tarRefList][refIdxTar].mvCand[1] )
            aacAMVPInfo[tarRefList][refIdxTar].numCand = 1;

          MvField cCurMvField, cTarMvField;
          Distortion costStart = std::numeric_limits<Distortion>::max();
          for ( int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand; i++ )
          {
            for ( int j = 0; j < aacAMVPInfo[tarRefList][refIdxTar].numCand; j++ )
            {
              cCurMvField.setMvField( aacAMVPInfo[curRefList][refIdxCur].mvCand[i], refIdxCur );
              cTarMvField.setMvField( aacAMVPInfo[tarRefList][refIdxTar].mvCand[j], refIdxTar );
              Distortion cost = xGetSymmetricCost( pu, origBuf, eCurRefList, cCurMvField, cTarMvField, bcwIdx );
              if ( cost < costStart )
              {
                costStart = cost;
                cMvPredSym[curRefList] = aacAMVPInfo[curRefList][refIdxCur].mvCand[i];
                cMvPredSym[tarRefList] = aacAMVPInfo[tarRefList][refIdxTar].mvCand[j];
                mvpIdxSym[curRefList] = i;
                mvpIdxSym[tarRefList] = j;
              }
            }
          }
          cCurMvField.mv = cMvPredSym[curRefList];
          cTarMvField.mv = cMvPredSym[tarRefList];

          m_pcRdCost->setCostScale(0);
          Mv pred = cMvPredSym[curRefList];
          pred.changeTransPrecInternal2Amvr(pu.cu->imv);
          m_pcRdCost->setPredictor(pred);
          Mv mv = cCurMvField.mv;
          mv.changeTransPrecInternal2Amvr(pu.cu->imv);
          uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
#if TM_AMVP
          bits += m_auiMVPIdxCost[mvpIdxSym[curRefList]][amvpNumCandCur];
          bits += m_auiMVPIdxCost[mvpIdxSym[tarRefList]][amvpNumCandTar];
#else
          bits += m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS];
          bits += m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS];
#endif
          costStart += m_pcRdCost->getCost(bits);

          std::vector<Mv> symmvdCands;
          auto smmvdCandsGen = [&](Mv mvCand, bool mvPrecAdj)
          {
            if (mvPrecAdj && pu.cu->imv)
            {
              mvCand.roundTransPrecInternal2Amvr(pu.cu->imv);
            }

            bool toAddMvCand = true;
            for (std::vector<Mv>::iterator pos = symmvdCands.begin(); pos != symmvdCands.end(); pos++)
            {
              if (*pos == mvCand)
              {
                toAddMvCand = false;
                break;
              }
            }

            if (toAddMvCand)
            {
              symmvdCands.push_back(mvCand);
            }
          };

          smmvdCandsGen(cMvHevcTemp[curRefList][refIdxCur], false);
          smmvdCandsGen(cMvTemp[curRefList][refIdxCur], false);
          if (iRefIdxBi[curRefList] == refIdxCur)
          {
            smmvdCandsGen(cMvBi[curRefList], false);
          }
          for (int i = 0; i < m_uniMvListSize; i++)
          {
            if ( symmvdCands.size() >= 5 )
              break;
            BlkUniMvInfo* curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
            smmvdCandsGen(curMvInfo->uniMvs[curRefList][refIdxCur], true);
          }

          for (auto mvStart : symmvdCands)
          {
            bool checked = false; //if it has been checkin in the mvPred.
            for (int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand && !checked; i++)
            {
              checked |= (mvStart == aacAMVPInfo[curRefList][refIdxCur].mvCand[i]);
            }
            if (checked)
            {
              continue;
            }

            Distortion bestCost = costStart;
            symmvdCheckBestMvp(pu, origBuf, mvStart, (RefPicList)curRefList, aacAMVPInfo, bcwIdx, cMvPredSym, mvpIdxSym, costStart);
            if (costStart < bestCost)
            {
              cCurMvField.setMvField(mvStart, refIdxCur);
              cTarMvField.setMvField(mvStart.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);
            }
          }
          Mv startPtMv = cCurMvField.mv;

#if TM_AMVP
          Distortion mvpCost = m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdxSym[curRefList]][amvpNumCandCur] + m_auiMVPIdxCost[mvpIdxSym[tarRefList]][amvpNumCandTar]);
#else
          Distortion mvpCost = m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS] + m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS]);
#endif
          symCost = costStart - mvpCost;

          // ME
          xSymmetricMotionEstimation( pu, origBuf, cMvPredSym[curRefList], cMvPredSym[tarRefList], eCurRefList, cCurMvField, cTarMvField, symCost, bcwIdx );

          symCost += mvpCost;

          if (startPtMv != cCurMvField.mv)
          { // if ME change MV, run a final check for best MVP.
            symmvdCheckBestMvp(pu, origBuf, cCurMvField.mv, (RefPicList)curRefList, aacAMVPInfo, bcwIdx, cMvPredSym, mvpIdxSym, symCost, true);
          }

          bits = uiMbBits[2];
          bits += 1; // add one bit for #symmetrical MVD mode
          bits += ((cs.slice->getSPS()->getUseBcw() == true) ? getWeightIdxBits(bcwIdx) : 0);
          symCost += m_pcRdCost->getCost(bits);
          cTarMvField.setMvField(cCurMvField.mv.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);

          if( m_pcEncCfg->getMCTSEncConstraint() )
          {
            if( !( MCTSHelper::checkMvForMCTSConstraint( pu, cCurMvField.mv ) && MCTSHelper::checkMvForMCTSConstraint( pu, cTarMvField.mv ) ) )
              symCost = std::numeric_limits<Distortion>::max();
          }
#if MULTI_HYP_PRED
          if (saveMeResultsForMHP)
          {
            // SMVD
            MEResult biPredResult;
            biPredResult.cu = cu;
            biPredResult.pu = pu;
            biPredResult.pu.interDir = 3;
            biPredResult.cu.smvdMode = 1 + curRefList;

            biPredResult.pu.mv[curRefList] = cCurMvField.mv;
            biPredResult.pu.mv[tarRefList] = cTarMvField.mv;
            biPredResult.pu.mv[curRefList].mvCliptoStorageBitDepth();
            biPredResult.pu.mv[tarRefList].mvCliptoStorageBitDepth();
            biPredResult.pu.mvd[curRefList] = cCurMvField.mv - cMvPredSym[curRefList];
            biPredResult.pu.mvd[tarRefList] = cTarMvField.mv - cMvPredSym[tarRefList];
            biPredResult.pu.refIdx[curRefList] = cCurMvField.refIdx;
            biPredResult.pu.refIdx[tarRefList] = cTarMvField.refIdx;
            biPredResult.pu.mvpIdx[curRefList] = mvpIdxSym[curRefList];
            biPredResult.pu.mvpIdx[tarRefList] = mvpIdxSym[tarRefList];
            biPredResult.pu.mvpNum[curRefList] = aaiMvpNum[curRefList][cCurMvField.refIdx];
            biPredResult.pu.mvpNum[tarRefList] = aaiMvpNum[tarRefList][cTarMvField.refIdx];

            biPredResult.cost = symCost;
            biPredResult.bits = bits;

            if (!(cu.imv != 0 && biPredResult.pu.mvd[0] == Mv(0, 0) && biPredResult.pu.mvd[1] == Mv(0, 0)))
            {
              cs.m_meResults.push_back(biPredResult);
            }

          }
#endif
          // save results
          if ( symCost < uiCostBi )
          {
            uiCostBi = symCost;
            symMode = 1 + curRefList;

            cMvBi[curRefList] = cCurMvField.mv;
            iRefIdxBi[curRefList] = cCurMvField.refIdx;
            aaiMvpIdxBi[curRefList][cCurMvField.refIdx] = mvpIdxSym[curRefList];
            cMvPredBi[curRefList][iRefIdxBi[curRefList]] = cMvPredSym[curRefList];

            cMvBi[tarRefList] = cTarMvField.mv;
            iRefIdxBi[tarRefList] = cTarMvField.refIdx;
            aaiMvpIdxBi[tarRefList][cTarMvField.refIdx] = mvpIdxSym[tarRefList];
            cMvPredBi[tarRefList][iRefIdxBi[tarRefList]] = cMvPredSym[tarRefList];
          }
        }
      } // if (B_SLICE)
#if JVET_AG0098_AMVP_WITH_SBTMVP
      bool       useAmvpSbTmvpBuf = false;
      int        amvpSbTmvpMvdIdx = -1;
      int        amvpSbTmvpColIdx = -1;
      Mv         amvpSbTmvpMv = Mv(0, 0);
      int        amvpSbTmvpRefIdx = -1;
      int        amvpSbTmvpInterDir = -1;
      Distortion amvpSbTmvpCost = std::numeric_limits<Distortion>::max();

      Distortion normalCost = ((uiCostBi <= uiCost[0]) && (uiCostBi <= uiCost[1])) ? uiCostBi : (uiCost[0] <= uiCost[1] ? uiCost[0] : uiCost[1]);
      if (tryAmvpSbTmvp && pu.cu->imv)
      {
        tryAmvpSbTmvp = normalCost < m_affineMotion.hevcCost[0];
      }

      if (tryAmvpSbTmvp)
      {
        pu.amvpSbTmvpFlag = true;
        for (int idx = 0; idx < cs.slice->getAmvpSbTmvpNumColPic(); idx++)
        {
          pu.colIdx = idx;

          RefPicList tmpAmvpSbTmvpRefList = idx == 0 ? RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag() : 0) : RefPicList(pu.cs->slice->isInterB() ? 1 - pu.cs->slice->getColFromL0Flag2nd() : 0);
          int tmpAmvpSbTmvpRefIdx = idx == 0 ? pu.cs->slice->getColRefIdx() : pu.cs->slice->getColRefIdx2nd();
          pu.interDir = tmpAmvpSbTmvpRefList == REF_PIC_LIST_0 ? 1 : 2;
          uint32_t tmpAmvpSbTmvpBits = cs.slice->getAmvpSbTmvpNumColPic() > 1 ? 1 : 0;
          Distortion tmpAmvpSbTmvpCost = std::numeric_limits<Distortion>::max();
          Mv       tmpAmvpSbTmvpMv = Mv(0, 0);
          int      tmpAmvpSbTmvpMvdIdx = -1;
          useAmvpSbTmvpBuf = false;

          xEstimateMvPredAMVP(pu, origBuf, tmpAmvpSbTmvpRefList, tmpAmvpSbTmvpRefIdx, cMvPred[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx], amvp[tmpAmvpSbTmvpRefList], false, &biPDistTemp);
          aaiMvpIdx[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx] = pu.mvpIdx[tmpAmvpSbTmvpRefList];
          aaiMvpNum[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx] = pu.mvpNum[tmpAmvpSbTmvpRefList];
#if TM_AMVP
          tmpAmvpSbTmvpBits += m_auiMVPIdxCost[aaiMvpIdx[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx]][aaiMvpNum[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx]];
#else
          tmpAmvpSbTmvpBits += m_auiMVPIdxCost[aaiMvpIdx[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx]][AMVP_MAX_NUM_CANDS];
#endif

          if (aaiMvpNum[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx] == 1)
          {
            useAmvpSbTmvpBuf = true;
            memset(m_amvpSbTmvpBufValid, 0, sizeof(bool) * AMVP_SBTMVP_BUF_SIZE);
            const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
            const unsigned mask = ~(scale - 1);
            Mv intMvShift = Mv(-1 * g_amvpSbTmvp_mvd_offset[cs.slice->getAmvpSbTmvpNumOffset() - 1 + (cu.imv ? AMVP_SBTMVP_NUM_OFFSET : 0)], -1 * g_amvpSbTmvp_mvd_offset[cs.slice->getAmvpSbTmvpNumOffset() - 1 + (cu.imv ? AMVP_SBTMVP_NUM_OFFSET : 0)]);
            intMvShift.changePrecision(MV_PRECISION_INT, MV_PRECISION_SIXTEENTH);
            intMvShift += cMvPred[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx];
            intMvShift.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            m_amvpSbTmvpBufTLPos.x = pu.lx() + intMvShift.hor;
            m_amvpSbTmvpBufTLPos.y = pu.ly() + intMvShift.ver;
            PU::clipColPos(m_amvpSbTmvpBufTLPos.x, m_amvpSbTmvpBufTLPos.y, pu);
            m_amvpSbTmvpBufTLPos = Position{ PosType(m_amvpSbTmvpBufTLPos.x & mask) >> ATMVP_SUB_BLOCK_SIZE, PosType(m_amvpSbTmvpBufTLPos.y & mask) >> ATMVP_SUB_BLOCK_SIZE };
          }
          
          xAmvpSbTmvpMotionEstimation(pu, origBuf, tmpAmvpSbTmvpRefList, cMvPred[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx], tmpAmvpSbTmvpRefIdx, tmpAmvpSbTmvpMv, tmpAmvpSbTmvpMvdIdx, useAmvpSbTmvpBuf, mergeCtx, aaiMvpIdx[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx], tmpAmvpSbTmvpBits, tmpAmvpSbTmvpCost, normalCost);

          xCopyAMVPInfo(&amvp[tmpAmvpSbTmvpRefList], &aacAMVPInfo[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx]); // must always be done ( also when AMVP_MODE = AM_NONE )
          xCheckBestMVP(tmpAmvpSbTmvpRefList, tmpAmvpSbTmvpMv, cMvPred[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx], aaiMvpIdx[tmpAmvpSbTmvpRefList][tmpAmvpSbTmvpRefIdx], amvp[tmpAmvpSbTmvpRefList], tmpAmvpSbTmvpBits, tmpAmvpSbTmvpCost, pu.cu->imv, true, tmpAmvpSbTmvpMvdIdx, pu.cs->slice->getAmvpSbTmvpNumOffset());

          if (tmpAmvpSbTmvpCost < amvpSbTmvpCost)
          {
            amvpSbTmvpCost = tmpAmvpSbTmvpCost;

            amvpSbTmvpMvdIdx   = tmpAmvpSbTmvpMvdIdx;
            amvpSbTmvpMv       = tmpAmvpSbTmvpMv;
            amvpSbTmvpRefIdx   = tmpAmvpSbTmvpRefIdx;
            amvpSbTmvpColIdx   = idx;
            amvpSbTmvpInterDir = pu.interDir;
          }
        } 
        pu.amvpSbTmvpFlag = false;
      }
#endif

      //  Clear Motion Field
    pu.mv    [REF_PIC_LIST_0] = Mv();
    pu.mv    [REF_PIC_LIST_1] = Mv();
    pu.mvd   [REF_PIC_LIST_0] = cMvZero;
    pu.mvd   [REF_PIC_LIST_1] = cMvZero;
    pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
    pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;


    // Set Motion Field

    cMv    [1] = mvValidList1;
    iRefIdx[1] = refIdxValidList1;
    uiBits [1] = bitsValidList1;
    uiCost [1] = costValidList1;
    if (cu.cs->pps->getWPBiPred() == true && tryBipred && (bcwIdx != BCW_DEFAULT))
    {
      CHECK(iRefIdxBi[0]<0, "Invalid picture reference index");
      CHECK(iRefIdxBi[1]<0, "Invalid picture reference index");
      wp0 = cu.cs->slice->getWpScaling(REF_PIC_LIST_0, iRefIdxBi[0]);
      wp1 = cu.cs->slice->getWpScaling(REF_PIC_LIST_1, iRefIdxBi[1]);
      if (WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1))
      {
        uiCostBi = MAX_UINT;
        enforceBcwPred = false;
      }
    }
    if( enforceBcwPred )
    {
      uiCost[0] = uiCost[1] = MAX_UINT;
    }

      uiLastModeTemp = uiLastMode;
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (amvpMergeModeFlag)
      {
        if (uiCostBi > ((m_amvpOnlyCost * 5) >> 2))
        {
#if JVET_Y0128_NON_CTC || JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          m_skipPROF = false;
          m_encOnly = false;
#endif
          bdmvrAmMergeNotValid = true;
#if JVET_AD0213_LIC_IMP
          m_encMotionEstimation = false;
#endif
          return;
        }
        m_amvpOnlyCost = (uiCostBi < m_amvpOnlyCost) ? uiCostBi : m_amvpOnlyCost;
      }
      if (((uiCostBi <= uiCost[0]) && (uiCostBi <= uiCost[1])) || amvpMergeModeFlag)
      {
        uiLastMode = 2;
        if (pu.amvpMergeModeFlag[1])
        {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          const int mvFieldMergeIdx = iRefIdxBi[0] * AMVP_MAX_NUM_CANDS_MEM + aaiMvpIdxBi[0][iRefIdxBi[0]];
#else
          const int mvFieldMergeIdx = iRefIdxBi[0] * AMVP_MAX_NUM_CANDS + aaiMvpIdxBi[0][iRefIdxBi[0]];
#endif
          pu.mv[REF_PIC_LIST_1] = mvFieldAmListCommon[mvFieldMergeIdx].mv;
          pu.refIdx[REF_PIC_LIST_1] = mvFieldAmListCommon[mvFieldMergeIdx].refIdx;
          pu.mvpIdx[REF_PIC_LIST_1] = 2;
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          pu.mvd[REF_PIC_LIST_1].setZero();
#endif
#if JVET_AD0213_LIC_IMP
          pu.cu->licFlag = licAmListCommon[mvFieldMergeIdx];
#endif
        }
        if (pu.amvpMergeModeFlag[0])
        {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          const int mvFieldMergeIdx = iRefIdxBi[1] * AMVP_MAX_NUM_CANDS_MEM + aaiMvpIdxBi[1][iRefIdxBi[1]];
#else
          const int mvFieldMergeIdx = iRefIdxBi[1] * AMVP_MAX_NUM_CANDS + aaiMvpIdxBi[1][iRefIdxBi[1]];
#endif
          pu.mv[REF_PIC_LIST_0] = mvFieldAmListCommon[mvFieldMergeIdx].mv;
          pu.refIdx[REF_PIC_LIST_0] = mvFieldAmListCommon[mvFieldMergeIdx].refIdx;
          pu.mvpIdx[REF_PIC_LIST_0] = 2;
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          pu.mvd[REF_PIC_LIST_0].setZero();
#endif
#if JVET_AD0213_LIC_IMP
          pu.cu->licFlag = licAmListCommon[mvFieldMergeIdx];
#endif
        }
        pu.interDir = 3;
        if (!pu.amvpMergeModeFlag[0])
        {
          pu.mv[REF_PIC_LIST_0] = cMvBi[0];
          pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
          pu.mvd[REF_PIC_LIST_0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
          pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
          pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
        }
        if (!pu.amvpMergeModeFlag[1])
        {
          pu.mv[REF_PIC_LIST_1] = cMvBi[1];
          pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
          pu.mvd[REF_PIC_LIST_1] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
          pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
          pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
        }
        pu.cu->smvdMode = symMode;
      }
#else
      if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1])
      {
        uiLastMode = 2;
        pu.mv    [REF_PIC_LIST_0] = cMvBi[0];
        pu.mv    [REF_PIC_LIST_1] = cMvBi[1];
        pu.mvd   [REF_PIC_LIST_0] = cMvBi[0] - cMvPredBi[0][iRefIdxBi[0]];
        pu.mvd   [REF_PIC_LIST_1] = cMvBi[1] - cMvPredBi[1][iRefIdxBi[1]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
        pu.interDir = 3;

        pu.cu->smvdMode = symMode;
      }
#endif
      else if ( uiCost[0] <= uiCost[1] )
      {
        uiLastMode = 0;
        pu.mv    [REF_PIC_LIST_0] = cMv[0];
        pu.mvd   [REF_PIC_LIST_0] = cMv[0] - cMvPred[0][iRefIdx[0]];
        pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
        pu.interDir = 1;
      }
      else
      {
        uiLastMode = 1;
        pu.mv    [REF_PIC_LIST_1] = cMv[1];
        pu.mvd   [REF_PIC_LIST_1] = cMv[1] - cMvPred[1][iRefIdx[1]];
        pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
        pu.interDir = 2;
      }

      if( bcwIdx != BCW_DEFAULT )
      {
        cu.bcwIdx = BCW_DEFAULT; // Reset to default for the Non-NormalMC modes.
      }

    uiHevcCost = ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] ) ? uiCostBi : ( ( uiCost[0] <= uiCost[1] ) ? uiCost[0] : uiCost[1] );
#if JVET_AG0098_AMVP_WITH_SBTMVP
    pu.amvpSbTmvpFlag = false;
    if (amvpSbTmvpCost < uiHevcCost)
    {
      uiHevcCost = amvpSbTmvpCost;
      pu.amvpSbTmvpFlag = true;
      pu.cu->smvdMode = false;
      pu.amvpSbTmvpMvdIdx = amvpSbTmvpMvdIdx;
      pu.colIdx = amvpSbTmvpColIdx;
      if (amvpSbTmvpInterDir == 1)
      {
        uiLastMode = 0;
        pu.mv[REF_PIC_LIST_0] = amvpSbTmvpMv;
        pu.mvd[REF_PIC_LIST_0] = amvpSbTmvpMv - cMvPred[0][amvpSbTmvpRefIdx];
        pu.refIdx[REF_PIC_LIST_0] = amvpSbTmvpRefIdx;
        pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][amvpSbTmvpRefIdx];
        pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][amvpSbTmvpRefIdx];
        pu.interDir = 1;

        pu.mv[REF_PIC_LIST_1] = Mv();
        pu.mvd[REF_PIC_LIST_1] = cMvZero;
        pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
        pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
      }
      else
      {
        uiLastMode = 1;
        pu.mv[REF_PIC_LIST_1] = amvpSbTmvpMv;
        pu.mvd[REF_PIC_LIST_1] = amvpSbTmvpMv - cMvPred[1][amvpSbTmvpRefIdx];
        pu.refIdx[REF_PIC_LIST_1] = amvpSbTmvpRefIdx;
        pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][amvpSbTmvpRefIdx];
        pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][amvpSbTmvpRefIdx];
        pu.interDir = 2;

        pu.mv[REF_PIC_LIST_0] = Mv();
        pu.mvd[REF_PIC_LIST_0] = cMvZero;
        pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
      }
    }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (!amvpMergeModeFlag && (m_amvpOnlyCost > uiHevcCost))
    {
      m_amvpOnlyCost = uiHevcCost;
    }
#endif
    }
#if JVET_AD0213_LIC_IMP
    bool condOn = (bcwIdx == BCW_DEFAULT || m_affineModeSelected || !m_pcEncCfg->getUseBcwFast() || (cu.licFlag && bcwIdx != BCW_DEFAULT));
    if (cu.licFlag && !m_doAffineLic)
    {
      condOn = false;
    }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
    if (bestSubTmvp && pu.amvpSbTmvpFlag)
    {
      checkAffine = false;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
    if (cu.Y().width >= 8 && cu.Y().height >= 8 && cu.slice->getSPS()->getUseAffine()
#else
    if (cu.Y().width > 8 && cu.Y().height > 8 && cu.slice->getSPS()->getUseAffine()
#endif
      && checkAffine
      && m_pcEncCfg->getUseAffineAmvp()
#if JVET_AD0213_LIC_IMP
      && condOn
#else
      && (bcwIdx == BCW_DEFAULT || m_affineModeSelected || !m_pcEncCfg->getUseBcwFast())
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
      && !amvpMergeModeFlag
#endif
      )
    {
      m_hevcCost = uiHevcCost;
      // save normal hevc result
      uint32_t uiMRGIndex = pu.mergeIdx;
      bool bMergeFlag = pu.mergeFlag;
      uint32_t uiInterDir = pu.interDir;
      int  iSymMode = cu.smvdMode;
#if JVET_AG0098_AMVP_WITH_SBTMVP
      bool isAmvpSbTmvpMode = pu.amvpSbTmvpFlag;
      pu.amvpSbTmvpFlag = false;
      int tmpAmvpSbTmvpMvdIdx = pu.amvpSbTmvpMvdIdx;
      pu.amvpSbTmvpMvdIdx = -1;
      int tmpAmvpSbTmvpColIdx = pu.colIdx;
      pu.colIdx = -1;
#endif

      Mv cMvd[2];
      uint32_t uiMvpIdx[2], uiMvpNum[2];
      uiMvpIdx[0] = pu.mvpIdx[REF_PIC_LIST_0];
      uiMvpIdx[1] = pu.mvpIdx[REF_PIC_LIST_1];
      uiMvpNum[0] = pu.mvpNum[REF_PIC_LIST_0];
      uiMvpNum[1] = pu.mvpNum[REF_PIC_LIST_1];
      cMvd[0]     = pu.mvd[REF_PIC_LIST_0];
      cMvd[1]     = pu.mvd[REF_PIC_LIST_1];

      MvField cHevcMvField[2];
      cHevcMvField[0].setMvField( pu.mv[REF_PIC_LIST_0], pu.refIdx[REF_PIC_LIST_0] );
      cHevcMvField[1].setMvField( pu.mv[REF_PIC_LIST_1], pu.refIdx[REF_PIC_LIST_1] );

      // do affine ME & Merge
      cu.affineType = AFFINEMODEL_4PARAM;
      Mv acMvAffine4Para[2][33][3];
      int refIdx4Para[2] = { -1, -1 };

      xPredAffineInterSearch(pu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, acMvAffine4Para, refIdx4Para, bcwIdx, enforceBcwPred,
        ((cu.slice->getSPS()->getUseBcw() == true) ? getWeightIdxBits(bcwIdx) : 0));

      if ( pu.cu->imv == 0 )
      {
        storeAffineMotion( pu.mvAffi, pu.refIdx, AFFINEMODEL_4PARAM, bcwIdx );
      }

      if ( cu.slice->getSPS()->getUseAffineType() )
      {
#if AFFINE_ENC_OPT
        if (uiAffineCost < uiHevcCost * 0.95) ///< condition for 6 parameter affine ME
#else
        if (uiAffineCost < uiHevcCost * 1.05) ///< condition for 6 parameter affine ME
#endif
        {
          // save 4 parameter results
          Mv bestMv[2][3], bestMvd[2][3];
          int bestMvpIdx[2], bestMvpNum[2], bestRefIdx[2];
          uint8_t bestInterDir;

          bestInterDir = pu.interDir;
          bestRefIdx[0] = pu.refIdx[0];
          bestRefIdx[1] = pu.refIdx[1];
          bestMvpIdx[0] = pu.mvpIdx[0];
          bestMvpIdx[1] = pu.mvpIdx[1];
          bestMvpNum[0] = pu.mvpNum[0];
          bestMvpNum[1] = pu.mvpNum[1];

          for ( int refList = 0; refList < 2; refList++ )
          {
            bestMv[refList][0] = pu.mvAffi[refList][0];
            bestMv[refList][1] = pu.mvAffi[refList][1];
            bestMv[refList][2] = pu.mvAffi[refList][2];
            bestMvd[refList][0] = pu.mvdAffi[refList][0];
            bestMvd[refList][1] = pu.mvdAffi[refList][1];
            bestMvd[refList][2] = pu.mvdAffi[refList][2];
          }

          refIdx4Para[0] = bestRefIdx[0];
          refIdx4Para[1] = bestRefIdx[1];

          Distortion uiAffine6Cost = std::numeric_limits<Distortion>::max();
          cu.affineType = AFFINEMODEL_6PARAM;
          xPredAffineInterSearch(pu, origBuf, puIdx, uiLastModeTemp, uiAffine6Cost, cMvHevcTemp, acMvAffine4Para, refIdx4Para, bcwIdx, enforceBcwPred,
            ((cu.slice->getSPS()->getUseBcw() == true) ? getWeightIdxBits(bcwIdx) : 0));

          if ( pu.cu->imv == 0 )
          {
            storeAffineMotion( pu.mvAffi, pu.refIdx, AFFINEMODEL_6PARAM, bcwIdx );
          }

          // reset to 4 parameter affine inter mode
          if ( uiAffineCost <= uiAffine6Cost )
          {
            cu.affineType = AFFINEMODEL_4PARAM;
            pu.interDir = bestInterDir;
            pu.refIdx[0] = bestRefIdx[0];
            pu.refIdx[1] = bestRefIdx[1];
            pu.mvpIdx[0] = bestMvpIdx[0];
            pu.mvpIdx[1] = bestMvpIdx[1];
            pu.mvpNum[0] = bestMvpNum[0];
            pu.mvpNum[1] = bestMvpNum[1];
            pu.mv[0].setZero();
            pu.mv[1].setZero();

            for ( int verIdx = 0; verIdx < 3; verIdx++ )
            {
              pu.mvdAffi[REF_PIC_LIST_0][verIdx] = bestMvd[0][verIdx];
              pu.mvdAffi[REF_PIC_LIST_1][verIdx] = bestMvd[1][verIdx];
              pu.mvAffi[REF_PIC_LIST_0][verIdx] = bestMv[0][verIdx];
              pu.mvAffi[REF_PIC_LIST_1][verIdx] = bestMv[1][verIdx];
            }
          }
          else
          {
            uiAffineCost = uiAffine6Cost;
          }
        }

        uiAffineCost += m_pcRdCost->getCost( 1 ); // add one bit for affine_type
      }

      if( uiAffineCost < uiHevcCost )
      {
        if( m_pcEncCfg->getMCTSEncConstraint() && !MCTSHelper::checkMvBufferForMCTSConstraint( pu ) )
        {
          uiAffineCost = std::numeric_limits<Distortion>::max();
        }
      }
      if ( uiHevcCost <= uiAffineCost )
      {
        // set hevc me result
        cu.affine = false;
        pu.mergeFlag = bMergeFlag;
        pu.regularMergeFlag = false;
        pu.mergeIdx = uiMRGIndex;
        pu.interDir = uiInterDir;
        cu.smvdMode = iSymMode;
#if JVET_AG0098_AMVP_WITH_SBTMVP
        pu.amvpSbTmvpFlag = isAmvpSbTmvpMode;
        pu.amvpSbTmvpMvdIdx = tmpAmvpSbTmvpMvdIdx;
        pu.colIdx = tmpAmvpSbTmvpColIdx;
#endif
        pu.mv    [REF_PIC_LIST_0] = cHevcMvField[0].mv;
        pu.refIdx[REF_PIC_LIST_0] = cHevcMvField[0].refIdx;
        pu.mv    [REF_PIC_LIST_1] = cHevcMvField[1].mv;
        pu.refIdx[REF_PIC_LIST_1] = cHevcMvField[1].refIdx;
        pu.mvpIdx[REF_PIC_LIST_0] = uiMvpIdx[0];
        pu.mvpIdx[REF_PIC_LIST_1] = uiMvpIdx[1];
        pu.mvpNum[REF_PIC_LIST_0] = uiMvpNum[0];
        pu.mvpNum[REF_PIC_LIST_1] = uiMvpNum[1];
        pu.mvd[REF_PIC_LIST_0] = cMvd[0];
        pu.mvd[REF_PIC_LIST_1] = cMvd[1];
      }
      else
      {
        cu.smvdMode = 0;
#if JVET_AG0098_AMVP_WITH_SBTMVP
        pu.amvpSbTmvpFlag = false;
        pu.amvpSbTmvpMvdIdx = -1;
        pu.colIdx = -1;
#endif
        CHECK( !cu.affine, "Wrong." );
        uiLastMode = uiLastModeTemp;
      }
    }
    if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
    {
      if (bcwIdx != BCW_DEFAULT)
      {
        cu.bcwIdx = bcwIdx;
      }
    }
    m_maxCompIDToPred = MAX_NUM_COMPONENT;

#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (amvpMergeModeFlag && PU::checkBDMVRCondition(pu))
    {
      setBdmvrSubPuMvBuf(mvBufEncAmBDmvrL0, mvBufEncAmBDmvrL1);
      pu.bdmvrRefine = true;
      // span motion to subPU
      for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
      {
        mvBufEncAmBDmvrL0[subPuIdx] = pu.mv[0];
        mvBufEncAmBDmvrL1[subPuIdx] = pu.mv[1];
      }
    }
    if (!pu.bdmvrRefine)
#endif
    {
#if JVET_AG0098_AMVP_WITH_SBTMVP
      if (pu.amvpSbTmvpFlag)
      {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
        PU::getAmvpSbTmvp(this, pu, mergeCtx, pu.interDir == 1 ? pu.mv[0] : pu.mv[1]);
#else
        PU::getAmvpSbTmvp(pu, mergeCtx, pu.interDir == 1 ? pu.mv[0] : pu.mv[1]);
#endif
      }
#endif
      PU::spanMotionInfo(pu, mergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif
      );
    }

    m_skipPROF = false;
    m_encOnly = false;
    //  MC
    PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (( bcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail || !m_affineMotion.affine6ParaAvail ) && !amvpMergeModeFlag)
#else
    if ( bcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail || !m_affineMotion.affine6ParaAvail )
#endif
    {
      m_affineMotion.hevcCost[pu.cu->imv] = uiHevcCost;
    }
#if JVET_AC0158_PIXEL_AFFINE_MC && JVET_AD0213_LIC_IMP
    cu.obmcFlag = true;
    if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
    {
      cu.obmcFlag = false;
    }
#endif
#if INTER_LIC

#if JVET_X0083_BM_AMVP_MERGE_MODE && JVET_AD0213_LIC_IMP
#if JVET_AG0098_AMVP_WITH_SBTMVP
    if (cu.licFlag && !amvpMergeModeFlag && !pu.amvpSbTmvpFlag)
#else
    if (cu.licFlag && !amvpMergeModeFlag)
#endif
#else
#if JVET_AG0098_AMVP_WITH_SBTMVP
    if (cu.licFlag && !pu.amvpSbTmvpFlag)
#else
    if (cu.licFlag)
#endif
#endif
    {
#if JVET_AD0213_LIC_IMP && TM_AMVP
      if (pu.interDir != MAX_UCHAR)
      {
        setEncCtrlParaLicOff(cu);
      }
#else
#if !TM_AMVP
#if JVET_AD0213_LIC_IMP
      m_storeBeforeLIC = (pu.interDir == 1 || pu.interDir == 2 || pu.cu->affine || pu.cu->smvdMode) ? true : false;
#else
      m_storeBeforeLIC = true;
#endif
#else
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP
      m_storeBeforeLIC = pu.cs->sps->getUseTMAmvpMode() ? false : true;
#endif
#endif
#endif
      m_predictionBeforeLIC = m_tmpStorageLCU.getBuf(UnitAreaRelative(*pu.cu, pu));
    }
#if JVET_AD0213_LIC_IMP
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_AG0098_AMVP_WITH_SBTMVP
    if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
    if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR)
#endif
#else
#if JVET_AG0098_AMVP_WITH_SBTMVP
    if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
    if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && pu.interDir != MAX_UCHAR)
#endif
#endif
    {
      setEncCtrlParaLicOn(cu);
    }
#endif
#endif
#if JVET_AD0213_LIC_IMP
    m_encMotionEstimation = false;
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, this))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
      setDoAffineSubPuBdof(false);
#if JVET_AG0098_AMVP_WITH_SBTMVP
      if (pu.amvpSbTmvpFlag)
      {
        Mv* buf = getBdofSubPuMvBuf();
        int bioSubPuIdx = 0;
        const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
        for (int yy = 0; yy < pu.lumaSize().height; yy += 4)
        {
          for (int xx = 0; xx < pu.lumaSize().width; xx += 4)
          {
            buf[bioSubPuIdx].setZero();
            bioSubPuIdx++;
          }
          bioSubPuIdx += bioSubPuStrideIncr;
        }
      }
#endif
    }
    motionCompensation( pu, predBuf, REF_PIC_LIST_X );
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    if (getDoAffineSubPuBdof() == true && (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1]))
    {
      PU::setAffineBdofRefinedMotion(pu, getBdofSubPuMvBuf());
      setDoAffineSubPuBdof(false);
    }
#else
    motionCompensation( pu, predBuf, REF_PIC_LIST_X );
#endif
#if JVET_AD0213_LIC_IMP
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    if(!pu.cu->licInheritPara)
#endif
    if (pu.cu->licFlag)
    {
      for (int list = 0; list < 2; list++)
      {
        if (pu.refIdx[list] >= 0)
        {
          for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
          {
            setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
          }
        }
      }
    }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (pu.bdmvrRefine)
    {
#if JVET_AG0276_LIC_BDOF_BDMVR
      if (cu.licFlag == true)
      {
        memset((void*)getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
      }
#endif
      PU::spanMotionInfo(*cu.firstPU, MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        pu.colIdx,
#endif
        mvBufEncAmBDmvrL0, mvBufEncAmBDmvrL1, getBdofSubPuMvOffset());
    }
#endif
#if INTER_LIC && (!TM_AMVP || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP) || JVET_AD0213_LIC_IMP)
    m_storeBeforeLIC = false;
#endif
    puIdx++;
  }

#if INTER_LIC
#if JVET_AD0213_LIC_IMP && TM_AMVP
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
  if (cu.licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR)
#endif
#else
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.licFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
  if (cu.licFlag && pu.interDir != MAX_UCHAR)
#endif
#endif
  {
    checkEncLicOff(cu, mergeCtx);
  }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  else
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
  if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && !amvpMergeModeFlag && pu.interDir != MAX_UCHAR)
#endif
#else
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && pu.interDir != MAX_UCHAR && !pu.amvpSbTmvpFlag)
#else
  if ((cu.slice->getUseLIC() && (cu.Y().area() >= LIC_MIN_CU_PIXELS)) && !pu.cu->licFlag && pu.interDir != MAX_UCHAR)
#endif
#endif
  {
    checkEncLicOn(cu, mergeCtx);
  }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  else
  {
    if (getDoAffineSubPuBdof())
    {
      setDoAffineSubPuBdof(false);
      PU::setAffineBdofRefinedMotion(pu, getBdofSubPuMvBuf());
    }
  }
#endif
#else
#if !TM_AMVP || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP) // This LIC optimization must be off; otherwise, enc/dec mismatching will result. Because the cost metrics (MRSAD or SAD) of TM mode is adaptive to LIC flag, refined MVs would change when LIC flag is 1 or 0.
  if (cu.licFlag && pu.interDir != MAX_UCHAR
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP
    && !pu.cs->sps->getUseTMAmvpMode()
#endif
    ) // xCheckRDCostInterIMV initializes pu.interDir by using 10. When checkAffine and checkNonAffine are both false, pu.interDir remains 10 which should be avoided
  {
#if JVET_AD0213_LIC_IMP
    CHECK(pu.interDir != 1 && pu.interDir != 2 && pu.interDir != 3, "Invalid InterDir for LIC");
#else
    CHECK(pu.interDir != 1 && pu.interDir != 2, "Invalid InterDir for LIC");
#endif

    PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
    DistParam distParam;
    m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), predBuf.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
    Distortion distLicOn = distParam.distFunc(distParam);

#if JVET_AD0213_LIC_IMP
    if (!(pu.interDir == 1 || pu.interDir == 2 || pu.cu->affine || pu.cu->smvdMode))
    {
      pu.cu->licFlag = false;
      motionCompensation(pu, m_predictionBeforeLIC, REF_PIC_LIST_X);
      pu.cu->licFlag = true;
    }
#endif
    m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), m_predictionBeforeLIC.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
    Distortion distLicOff = distParam.distFunc(distParam);
    if (distLicOn >= distLicOff)
    {
      pu.cu->licFlag = false;
      PU::spanLicFlags(pu, false);
      predBuf.copyFrom(m_predictionBeforeLIC);
    }
  }
#endif
#endif
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (cu.imv && !CU::hasSubCUNonZeroMVd(cu) && !CU::hasSubCUNonZeroAffineMVd(cu))
  {
    setWpScalingDistParam(-1, REF_PIC_LIST_X, cu.cs->slice);
    return;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if ((!PU::useRefCombList(pu) && !PU::useRefPairList(pu)) || (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]))
#else
  if(!PU::useRefPairList(pu) && !PU::useRefCombList(pu))
#endif
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.isMvdPredApplicable() && !pu.amvpSbTmvpFlag)
#else
  if (pu.isMvdPredApplicable())
#endif
  {

#if JVET_AD0140_MVD_PREDICTION
    pu.mvdSuffixInfo.clear();
#endif

#if !JVET_AD0213_LIC_IMP
    bool bi = pu.interDir == 3;
#endif
    if (cu.affine)
    {
#if JVET_AD0140_MVD_PREDICTION
      const auto& motionModel = (3 == pu.interDir) ? MotionModel::BiAffine : MotionModel::UniAffine;
      for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
      {
        RefPicList eRefPicList = RefPicList(uiRefListIdx);
        Mv absMvd[3];
        absMvd[0] = Mv(pu.mvdAffi[uiRefListIdx][0].getAbsMv());
        absMvd[1] = Mv(pu.mvdAffi[uiRefListIdx][1].getAbsMv());
        absMvd[2] = (cu.affineType == AFFINEMODEL_6PARAM) ? Mv(pu.mvdAffi[uiRefListIdx][2].getAbsMv()) : Mv(0, 0);
        if (pu.cs->slice->getNumRefIdx(eRefPicList) > 0
          && (pu.interDir & (1 << uiRefListIdx)) && (absMvd[0] != Mv(0, 0) || absMvd[1] != Mv(0, 0) || absMvd[2] != Mv(0, 0)) && pu.isMvdPredApplicable())
        {
          for (int i = 0; i < 2 + (pu.cu->affine); ++i)
          {
            pu.mvdSuffixInfo.initPrefixesMvd(i, eRefPicList, pu.mvdAffi[uiRefListIdx][i].getAbsMv(), pu.cu->imv, true, motionModel);
          }

        }
        pu.mvdSuffixInfo.getBinBudgetForMv(MvdSuffixInfoMv::getBinBudgetForPrediction(pu.Y().width, pu.Y().height, pu.cu->imv), eRefPicList);
      }
#endif
      for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
      {
        RefPicList eRefPicList = RefPicList(uiRefListIdx);
        Mv absMvd[3];
        absMvd[0] = Mv(pu.mvdAffi[uiRefListIdx][0].getAbsMv());
        absMvd[1] = Mv(pu.mvdAffi[uiRefListIdx][1].getAbsMv());
        absMvd[2] = (cu.affineType == AFFINEMODEL_6PARAM) ? Mv(pu.mvdAffi[uiRefListIdx][2].getAbsMv()) : Mv(0, 0);
        if (pu.cs->slice->getNumRefIdx(eRefPicList) > 0
          && (pu.interDir & (1 << uiRefListIdx)) && (absMvd[0] != Mv(0, 0) || absMvd[1] != Mv(0, 0) || absMvd[2] != Mv(0, 0)) && pu.isMvdPredApplicable())
        {
          AffineAMVPInfo affineAMVPInfo;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
          PU::fillAffineMvpCand(this, pu, eRefPicList, pu.refIdx[uiRefListIdx], affineAMVPInfo
#else
          PU::fillAffineMvpCand(pu, eRefPicList, pu.refIdx[uiRefListIdx], affineAMVPInfo
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
            , this
#endif    
          );

          const unsigned mvpIdx = pu.mvpIdx[eRefPicList];

#if JVET_AD0140_MVD_PREDICTION
          std::vector<Mv> cMvdDerived[3];
          deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx], 
            absMvd, pu, eRefPicList, pu.refIdx[eRefPicList], cMvdDerived[0], cMvdDerived[1], cMvdDerived[2]);
#else
          std::vector<Mv> cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3;
#if JVET_Z0054_BLK_REF_PIC_REORDER
          deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx],
            absMvd, pu, eRefPicList, pu.refIdx[eRefPicList], cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#else
          deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx],
            absMvd[0], absMvd[1], absMvd[2], pu, eRefPicList, pu.refIdx[eRefPicList], cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#endif
#endif
          int idx = -1;
#if JVET_AD0140_MVD_PREDICTION
          idx = deriveMVSDIdxFromMVDAffineSI(pu, eRefPicList, cMvdDerived[0], cMvdDerived[1], cMvdDerived[2]);
          initOffsetsAffineMvd(pu, eRefPicList, cMvdDerived);
#else
          idx = deriveMVSDIdxFromMVDAffine(pu, eRefPicList, cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#endif
          CHECK(idx == -1, "no match for mvsdIdx at Encoder");
          pu.mvsdIdx[eRefPicList] = idx;
#if JVET_AD0140_MVD_PREDICTION
          defineSignHypMatchAffine(pu, eRefPicList);
#endif
        }
      }
    }
    else
    {
#if JVET_AD0140_MVD_PREDICTION
      pu.mvdSuffixInfo.clear();
      const auto& motionModel = (0 != cu.smvdMode) ? MotionModel::BiTranslationalSmvd :
        (3 == pu.interDir) ? MotionModel::BiTranslational : MotionModel::UniTranslational;
      for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
      {
        RefPicList eRefPicList = RefPicList(uiRefListIdx);
        Mv cMvd = pu.mvd[eRefPicList];

        if (pu.cs->slice->getNumRefIdx(eRefPicList) > 0
          && (pu.interDir & (1 << uiRefListIdx))
          && pu.isMvdPredApplicable()
          && cMvd.isMvdPredApplicable()
          )
        {
          const RefPicList curRefList = cu.smvdMode ? REF_PIC_LIST_0 : (RefPicList)uiRefListIdx;
          if ((!cu.smvdMode) || REF_PIC_LIST_0 == eRefPicList)
          {
            pu.mvdSuffixInfo.initPrefixesMvd(0, curRefList, pu.mvd[curRefList].getAbsMv(), pu.cu->imv, true, motionModel);
          }

          pu.mvdSuffixInfo.getBinBudgetForMv(MvdSuffixInfoMv::getBinBudgetForPrediction(pu.Y().width, pu.Y().height, pu.cu->imv), eRefPicList);
          if (cu.smvdMode)
          {
            pu.mvdSuffixInfo.selectRplForMvdCoding(MvdSuffixInfoMv::getBinBudgetForPrediction(pu.Y().width, pu.Y().height, pu.cu->imv));
            break;
          }
        }
      }
#endif
      for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
      {
        RefPicList eRefPicList = RefPicList(uiRefListIdx);
        Mv cMvd = pu.mvd[eRefPicList];
        if (pu.cs->slice->getNumRefIdx(eRefPicList) > 0
          && (pu.interDir & (1 << uiRefListIdx))
          && pu.isMvdPredApplicable()
          && cMvd.isMvdPredApplicable()
          )
        {
#if JVET_AD0213_LIC_IMP
          int      iRefIdx = pu.refIdx[uiRefListIdx];
          AMVPInfo* amvpCand = cu.licFlag ? &(m_tplAmvpInfoLIC[cu.imv][uiRefListIdx][iRefIdx]) : &(m_tplAmvpInfo[cu.imv][uiRefListIdx][iRefIdx]);
          if (amvpCand->numCand == 0)
          {
            PU::fillMvpCand(pu, RefPicList(uiRefListIdx), iRefIdx, *amvpCand, this);
          }

          Mv cMvPred2 = amvpCand->mvCand[pu.mvpIdx[uiRefListIdx]];
#else
          auto aMvPred = bi ? cMvPredBi : cMvPred;
          auto aRefIdx = bi ? iRefIdxBi : iRefIdx;
          auto aMv = bi ? cMvBi : cMv;
          Mv cMvPred2 = aMvPred[uiRefListIdx][aRefIdx[uiRefListIdx]];
          CHECK(cMvd != aMv[uiRefListIdx] - cMvPred2, "");
          int iRefIdx = pu.refIdx[uiRefListIdx];
#endif
          Mv cMvdKnownAtDecoder = Mv(cMvd.getAbsHor(), cMvd.getAbsVer());
          std::vector<Mv> cMvdDerivedVec;

#if JVET_AD0140_MVD_PREDICTION
          const RefPicList curRefList = cu.smvdMode ? REF_PIC_LIST_0 : (RefPicList)uiRefListIdx;
#endif
          if (cu.smvdMode)
          {
            if (uiRefListIdx == 1)
            {
              cMvd = pu.mvd[REF_PIC_LIST_0];
              CHECK((pu.mvd[REF_PIC_LIST_0].hor != -pu.mvd[REF_PIC_LIST_1].hor) || (pu.mvd[REF_PIC_LIST_0].ver != -pu.mvd[REF_PIC_LIST_1].ver), "not mirrored MVD for SMVD at Enc");
              CHECK(cs.slice->getSymRefIdx(REF_PIC_LIST_0) != pu.refIdx[REF_PIC_LIST_0], "ref Idx for List 0 does not match for SMVD at Enc");
              CHECK(cs.slice->getSymRefIdx(REF_PIC_LIST_1) != pu.refIdx[REF_PIC_LIST_1], "ref Idx for List 1 does not match for SMVD at Enc");

#if JVET_AD0213_LIC_IMP
              AMVPInfo* amvpCand0 = cu.licFlag ? &(m_tplAmvpInfoLIC[cu.imv][0][iRefIdx]) : &(m_tplAmvpInfo[cu.imv][0][iRefIdx]);
              CHECK(amvpCand0->numCand == 0, "amvp candiate is not available");
              deriveMvdSignSMVD(amvpCand0->mvCand[pu.mvpIdx[0]], amvpCand->mvCand[pu.mvpIdx[uiRefListIdx]], cMvdKnownAtDecoder, pu, cMvdDerivedVec);
#else
              deriveMvdSignSMVD(aMvPred[0][aRefIdx[0]], aMvPred[1][aRefIdx[1]], cMvdKnownAtDecoder, pu, cMvdDerivedVec);
#endif
#if JVET_AD0140_MVD_PREDICTION
              int idx = deriveMVSDIdxFromMVDTransSI(cMvd, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[curRefList][0]);
              initOffsetsMvd(pu.mvd[curRefList], cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[curRefList][0], pu.cu->imv);
#else
              int idx = deriveMVSDIdxFromMVDTrans(cMvd, cMvdDerivedVec);
#endif
              CHECK(idx == -1, "");
              pu.mvsdIdx[REF_PIC_LIST_0] = idx;
#if JVET_AD0140_MVD_PREDICTION
              defineSignHypMatch(pu.mvd[curRefList], pu.mvdSuffixInfo.mvBins[curRefList][0], pu.mvsdIdx[curRefList]);
#endif
            }
          }
          else
          {
            deriveMvdSign(cMvPred2, cMvdKnownAtDecoder, pu, eRefPicList, iRefIdx, cMvdDerivedVec);
#if JVET_AD0140_MVD_PREDICTION
            int idx = deriveMVSDIdxFromMVDTransSI(cMvd, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[uiRefListIdx][0]);
            initOffsetsMvd(pu.mvd[uiRefListIdx], cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[uiRefListIdx][0], pu.cu->imv);
#else
            int idx = deriveMVSDIdxFromMVDTrans(cMvd, cMvdDerivedVec);
#endif
            CHECK(idx == -1, "");
            pu.mvsdIdx[eRefPicList] = idx;
#if JVET_AD0140_MVD_PREDICTION
            defineSignHypMatch(pu.mvd[uiRefListIdx], pu.mvdSuffixInfo.mvBins[uiRefListIdx][0], pu.mvsdIdx[uiRefListIdx]);
#endif
          }
        }
      } //loop end for non-affine
    }
  }
#endif
  setWpScalingDistParam( -1, REF_PIC_LIST_X, cu.cs->slice );

  return;
}

uint32_t InterSearch::xCalcAffineMVBits( PredictionUnit& pu, Mv acMvTemp[3], Mv acMvPred[3] )
{
  int mvNum  = pu.cu->affineType ? 3 : 2;
  m_pcRdCost->setCostScale( 0 );
  uint32_t bitsTemp = 0;

  for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
  {
    Mv pred = verIdx == 0 ? acMvPred[verIdx] : acMvPred[verIdx] + acMvTemp[0] - acMvPred[0];
    pred.changeAffinePrecInternal2Amvr(pu.cu->imv);
    m_pcRdCost->setPredictor( pred );
    Mv mv = acMvTemp[verIdx];
    mv.changeAffinePrecInternal2Amvr(pu.cu->imv);

    bitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor( mv.getHor(), mv.getVer(), 0 );
  }

  return bitsTemp;
}

#if MULTI_HYP_PRED
void InterSearch::predInterSearchAdditionalHypothesis(PredictionUnit& pu, const MEResult& x, MEResultVec& out)
{
  const SPS &sps = *pu.cs->sps;
  CHECK(!sps.getUseInterMultiHyp(), "Multi Hyp is not active");
  CHECK(!pu.cs->slice->isInterB(), "Multi Hyp only allowed in B slices");
  CHECK(pu.cu->predMode != MODE_INTER, "Multi Hyp: pu.cu->predMode != MODE_INTER");
  CHECK(pu.addHypData.size() > sps.getMaxNumAddHyps(), "Multi Hyp: too many hypotheseis");
  if( pu.addHypData.size() == sps.getMaxNumAddHyps() )
  {
    return;
  }

  CHECK(!pu.mergeFlag && pu.cu->bcwIdx == BCW_DEFAULT, "!pu.mergeFlag && pu.cu->bcwIdx == BCW_DEFAULT");
  // get first prediction hypothesis
  PelUnitBuf tempPredBuf;
  if (x.predBuf != nullptr)
  {
    tempPredBuf = *x.predBuf;
  }
  else
  {
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, this))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_WITHOUT_STORE_MV;
      m_doAffineSubPuBdof = false;
#if JVET_AG0098_AMVP_WITH_SBTMVP
      if (pu.mergeType != MRG_TYPE_SUBPU_ATMVP && !pu.amvpSbTmvpFlag)
#else
      if (pu.mergeType != MRG_TYPE_SUBPU_ATMVP)
#endif
      {
        PU::spanMotionInfo(pu);
      }
    }
#endif
    tempPredBuf = pu.cs->getPredBuf(pu);
    pu.mvRefine = true;
    motionCompensation(pu, REF_PIC_LIST_X, true, false);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    m_doAffineSubPuBdof = false;
#endif
    pu.mvRefine = false;
  }
  const auto &MHRefPics = pu.cs->slice->getMultiHypRefPicList();
  const int iNumMHRefPics = int(MHRefPics.size());
  CHECK(iNumMHRefPics <= 0, "Multi Hyp: iNumMHRefPics <= 0");

  PelUnitBuf origBuf = pu.cs->getOrgBuf(pu);

  const UnitArea unitAreaFromPredBuf(origBuf.chromaFormat, Area(Position(0, 0), origBuf.Y()));
  // NOTE: tempOrigBuf share the same buffer with tempBuf that is used in xAddHypMC.
  PelUnitBuf tempOrigBuf = m_additionalHypothesisStorage.getBuf(unitAreaFromPredBuf);


  MultiHypPredictionData tempMHPredData;

  m_pcRdCost->selectMotionLambda();

  const int numWeights = sps.getNumAddHypWeights();
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdx(pu.Y(), *pu.cs->slice->getPPS()->pcv, idx1, idx2, idx3, idx4);
#if JVET_AG0276_NLIC
  bool savedAltLMFlag = pu.cu->altLMFlag;
  AltLMInterUnit savedAltLMParaUnit = pu.cu->altLMParaUnit;
#endif
#if INTER_LIC
  auto savedLICFlag = pu.cu->licFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
  auto savedLICInheritePara = pu.cu->licInheritPara;
#endif
#endif
  tempMHPredData.isMrg = true;
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  uint8_t maxNumMergeCandidates = pu.cs->sps->getMaxNumMHPCand();
  CHECK(maxNumMergeCandidates >= GEO_MAX_NUM_UNI_CANDS, "");
  if (maxNumMergeCandidates > 0)
  {
#else
  {
    uint8_t maxNumMergeCandidates = pu.cs->sps->getMaxNumGeoCand();
    CHECK(maxNumMergeCandidates >= GEO_MAX_NUM_UNI_CANDS, "");
#endif
    DistParam distParam;
    const bool bUseHadamard = !pu.cs->slice->getDisableSATDForRD();
    m_pcRdCost->setDistParam(distParam, origBuf.Y(), tempOrigBuf.Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

#if JVET_AD0213_LIC_IMP
    bool mhpMrgListGenerate = ((!savedLICFlag && ((m_mhpMrgTempBufSet & 0x1) == 0)) || (savedLICFlag && ((m_mhpMrgTempBufSet & 0x2) == 0)));
    if (mhpMrgListGenerate)
    {
      PredictionUnit fakePredData = pu;
      fakePredData.mergeFlag = false;
      fakePredData.mergeType = MRG_TYPE_DEFAULT_N;
      fakePredData.mmvdMergeFlag = false;
      fakePredData.ciipFlag = false;
      fakePredData.addHypData.clear();
      fakePredData.regularMergeFlag = false;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      fakePredData.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      fakePredData.bmMergeFlag = false;
#endif
#if MULTI_PASS_DMVR
      fakePredData.bdmvrRefine = false;
#endif
      if (!m_mhpMrgTempBufSet)
      {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
        PU::getGeoMergeCandidates(fakePredData, m_geoMrgCtx, this);
#else
        PU::getGeoMergeCandidates(fakePredData, m_geoMrgCtx);
#endif
      }
#if JVET_W0097_GPM_MMVD_TM
      maxNumMergeCandidates = min((int)maxNumMergeCandidates, m_geoMrgCtx.numValidMergeCand);
#endif
      const auto savedAffine = pu.cu->affine;
      const auto savedIMV = pu.cu->imv;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      m_isAddHypMC = true;
#endif
      for (int i = 0; i < maxNumMergeCandidates; i++)
      {
        // get prediction for the additional hypothesis
        int refList = m_geoMrgCtx.interDirNeighbours[i] - 1; CHECK(refList != 0 && refList != 1, "");
        fakePredData.interDir = refList + 1;
        fakePredData.mv[refList] = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].mv;
        fakePredData.refIdx[refList] = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].refIdx;
        fakePredData.refIdx[1 - refList] = -1;
        fakePredData.cu->affine = false;
        fakePredData.cu->imv = m_geoMrgCtx.useAltHpelIf[i] ? IMV_HPEL : 0;
#if JVET_AG0276_NLIC
        fakePredData.cu->altLMFlag = m_geoMrgCtx.altLMFlag[i];
        fakePredData.cu->altLMParaUnit = m_geoMrgCtx.altLMParaNeighbours[i];
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        fakePredData.cu->licInheritPara = false;
#endif
        fakePredData.mvRefine = true;
        if (savedLICFlag)
        {
          motionCompensation(fakePredData, m_mhpMrgTempBufLic[i], REF_PIC_LIST_X, true, false);
        }
        else
        {
          motionCompensation(fakePredData, m_mhpMrgTempBuf[i], REF_PIC_LIST_X, true, false);
        }
        fakePredData.mvRefine = false;
        // the restore of affine flag and imv flag has to be here
#if JVET_AG0276_NLIC
        fakePredData.cu->altLMFlag = savedAltLMFlag;
        fakePredData.cu->altLMParaUnit = savedAltLMParaUnit;
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        fakePredData.cu->licInheritPara = savedLICInheritePara;
#endif
        fakePredData.cu->imv = savedIMV;
        fakePredData.cu->affine = savedAffine;
      }
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      m_isAddHypMC = false;
#endif
      if (savedLICFlag)
      {
        setGeoTmpBuffer(m_mhpMrgTempBufSet + 0x2);
      }
      else
      {
        setGeoTmpBuffer(m_mhpMrgTempBufSet + 0x1);
      }
    }
#else
    if (!(pu.addHypData.size() > pu.numMergedAddHyps && m_mhpMrgTempBufSet)) // non 1st addHyp check should already have the MC results stored
    {
      PredictionUnit fakePredData = pu;
      fakePredData.mergeFlag = false;
      fakePredData.mergeType = MRG_TYPE_DEFAULT_N;
      fakePredData.mmvdMergeFlag = false;
      fakePredData.ciipFlag = false;
      fakePredData.addHypData.clear();
      fakePredData.regularMergeFlag = false;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      fakePredData.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      fakePredData.bmMergeFlag = false;
#endif
#if MULTI_PASS_DMVR
      fakePredData.bdmvrRefine = false;
#endif
      if (!m_mhpMrgTempBufSet)
      {
        PU::getGeoMergeCandidates(fakePredData, m_geoMrgCtx);
      }
#if JVET_W0097_GPM_MMVD_TM
      maxNumMergeCandidates = min((int)maxNumMergeCandidates, m_geoMrgCtx.numValidMergeCand);
#endif
      const auto savedAffine = pu.cu->affine;
      const auto savedIMV = pu.cu->imv;
      for (int i = 0; i < maxNumMergeCandidates; i++)
      {
        if (m_mhpMrgTempBufSet // MC results already stored when checking GEO RD cost
#if INTER_LIC
          && (fakePredData.cu->licFlag == m_geoMrgCtx.licFlags[i])
#endif
          )
        {
          continue;
        }
        // get prediction for the additional hypothesis
        int refList = m_geoMrgCtx.interDirNeighbours[i] - 1; CHECK(refList != 0 && refList != 1, "");
        fakePredData.interDir = refList + 1;
        fakePredData.mv[refList] = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].mv;
        fakePredData.refIdx[refList] = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].refIdx;
        fakePredData.refIdx[1 - refList] = -1;
        fakePredData.cu->affine = false;
        fakePredData.cu->imv = m_geoMrgCtx.useAltHpelIf[i] ? IMV_HPEL : 0;
        fakePredData.mvRefine = true;
        motionCompensation(fakePredData, m_mhpMrgTempBuf[i], REF_PIC_LIST_X, true, false);
        fakePredData.mvRefine = false;
        // the restore of affine flag and imv flag has to be here
        fakePredData.cu->imv = savedIMV;
        fakePredData.cu->affine = savedAffine;
      }
      setGeoTmpBuffer();
    }
#endif
#if JVET_W0097_GPM_MMVD_TM
    else
    {
      maxNumMergeCandidates = min((int)maxNumMergeCandidates, m_geoMrgCtx.numValidMergeCand);
    }
#endif
    for (int i = 0; i < maxNumMergeCandidates; i++)
    {
      int refList = m_geoMrgCtx.interDirNeighbours[i] - 1; CHECK(refList != 0 && refList != 1, "");
      tempMHPredData.mrgIdx = i;
      tempMHPredData.isMrg = true;
      tempMHPredData.refIdx = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].refIdx;
      tempMHPredData.mv = m_geoMrgCtx.mvFieldNeighbours[(i << 1) + refList].mv;
      tempMHPredData.imv = m_geoMrgCtx.useAltHpelIf[i] ? IMV_HPEL : 0;
#if INTER_LIC 
      tempMHPredData.licFlag = savedLICFlag;
#endif
      tempMHPredData.refList = refList;
      for (tempMHPredData.weightIdx = 0; tempMHPredData.weightIdx < numWeights; ++tempMHPredData.weightIdx)
      {
        tempOrigBuf.copyFrom(tempPredBuf, true);

#if JVET_AD0213_LIC_IMP
        if (savedLICFlag)
        {
          tempOrigBuf.addHypothesisAndClip(m_mhpMrgTempBufLic[i], g_addHypWeight[tempMHPredData.weightIdx], pu.cs->slice->clpRngs(), true);
        }
        else
#endif
        tempOrigBuf.addHypothesisAndClip(m_mhpMrgTempBuf[i], g_addHypWeight[tempMHPredData.weightIdx], pu.cs->slice->clpRngs(), true);
        Distortion uiSad = distParam.distFunc(distParam);
        uint32_t uiBits = x.bits + (i + 1);
        if (i == pu.cs->sps->getMaxNumGeoCand() - 1)
        {
          uiBits--;
        }
        uiBits += tempMHPredData.weightIdx + 1;
        if (tempMHPredData.weightIdx == numWeights - 1)
          uiBits--;
        Distortion uiCostTemp = uiSad + m_pcRdCost->getCost(uiBits);
        if (uiCostTemp < x.cost)
        {
          MEResult result;
          result.cu = *pu.cu;
          result.pu = pu;
          CHECK(tempMHPredData.mrgIdx >= maxNumMergeCandidates, "");
          result.pu.addHypData.push_back(tempMHPredData);
          result.cost = uiCostTemp;
          result.bits = uiBits;
          // store MHP MC result for next additonal hypothesis test
          if (pu.addHypData.size() < sps.getMaxNumAddHyps() && m_mhpTempBufCounter < GEO_MAX_TRY_WEIGHTED_SAD)
          {
            result.predBuf = &m_mhpTempBuf[m_mhpTempBufCounter];
            result.predBufIdx = m_mhpTempBufCounter;
            m_mhpTempBufCounter++;
            result.predBuf->copyFrom(tempOrigBuf, true);
          }
          out.push_back(result);
        }
      } // weightIdx
    } // i
  }
  tempMHPredData.isMrg = false;
#if INTER_LIC 
  tempMHPredData.licFlag = pu.cu->licFlag;
#endif
  tempMHPredData.imv = pu.cu->imv;
  for (tempMHPredData.weightIdx = 0; tempMHPredData.weightIdx < numWeights; ++tempMHPredData.weightIdx)
  {
    tempOrigBuf.copyFrom(origBuf, true);
    tempOrigBuf.removeHighFreq(tempPredBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs(), g_addHypWeight[tempMHPredData.weightIdx]);
    for (tempMHPredData.refIdx = 0; tempMHPredData.refIdx < iNumMHRefPics; ++tempMHPredData.refIdx)
    {
      tempMHPredData.mvpIdx = 0;
      {
        const int iRefPicList = MHRefPics[tempMHPredData.refIdx].refList;
        const int iRefIdxPred = MHRefPics[tempMHPredData.refIdx].refIdx;
        const RefPicList eRefPicList = RefPicList(iRefPicList);
        uint32_t uiBits = x.bits + getAdditionalHypothesisInitialBits(tempMHPredData, numWeights, iNumMHRefPics);
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
        auto amvpInfo = PU::getMultiHypMVPCands(this, pu, tempMHPredData);
#else
        auto amvpInfo = PU::getMultiHypMVPCands(pu, tempMHPredData);
#endif
        Mv cMvPred = amvpInfo.mvCand[tempMHPredData.mvpIdx];
        if ((pu.addHypData.size() + 1 - pu.numMergedAddHyps) < sps.getMaxNumAddHyps())
          uiBits++;
        Mv cMv(0, 0);
        if (g_isReusedUniMVsFilled[idx1][idx2][idx3][idx4])
        {
          cMv = g_reusedUniMVs[idx1][idx2][idx3][idx4][iRefPicList][iRefIdxPred];
          uint32_t bitsDummy = 0;
          Distortion uiCostDummy = 0;
          xCheckBestMVP(eRefPicList, cMv, cMvPred, tempMHPredData.mvpIdx, amvpInfo, bitsDummy, uiCostDummy, pu.cu->imv);
        }
        else
        {
          cMv = cMvPred;
        }
        Distortion uiCostTemp = 0;
#if JVET_AG0276_NLIC
        pu.cu->altLMFlag = false;
        pu.cu->altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
        pu.cu->licFlag = tempMHPredData.licFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        pu.cu->licInheritPara = false;
#endif
#endif
        xMotionEstimation(pu, tempOrigBuf, eRefPicList, cMvPred, iRefIdxPred, cMv, tempMHPredData.mvpIdx, uiBits, uiCostTemp, amvpInfo, false, g_addHypWeight[tempMHPredData.weightIdx]);
        xCheckBestMVP(eRefPicList, cMv, cMvPred, tempMHPredData.mvpIdx, amvpInfo, uiBits, uiCostTemp, pu.cu->imv);
#if JVET_AG0276_NLIC
        pu.cu->altLMFlag = savedAltLMFlag;
        pu.cu->altLMParaUnit = savedAltLMParaUnit;
#endif
#if INTER_LIC
        pu.cu->licFlag = savedLICFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        pu.cu->licInheritPara = savedLICInheritePara;
#endif
#endif
        tempMHPredData.mv = cMv;
        tempMHPredData.mv.mvCliptoStorageBitDepth();

        tempMHPredData.mvd = cMv - cMvPred;

        if (uiCostTemp < x.cost)
        {
          MEResult result;
          result.cu = *pu.cu;
          result.pu = pu;
          result.pu.addHypData.push_back(tempMHPredData);
          result.cost = uiCostTemp;
          result.bits = uiBits;
          out.push_back(result);
        }
      }
    }
    }

  // buffer recycling
  if (m_pcEncCfg->getNumMHPCandsToTest() > 4 && x.predBufIdx >= 0 && m_mhpTempBufCounter > x.predBufIdx + 1)
  {
#if !JVET_AG0164_AFFINE_GPM
    if (x.predBufIdx < GEO_MAX_TRY_WEIGHTED_SAD - 1)
    {
#endif
      ::memcpy(m_mhpTempBuf + x.predBufIdx, m_mhpTempBuf + x.predBufIdx + 1, (m_mhpTempBufCounter - x.predBufIdx - 1) * sizeof(PelUnitBuf));
#if !JVET_AG0164_AFFINE_GPM
    }
#endif
    m_mhpTempBufCounter--;
  }
}

inline unsigned InterSearch::getAdditionalHypothesisInitialBits(const MultiHypPredictionData& mhData,
  const int iNumWeights,
  const int iNumMHRefPics)
{
  unsigned uiBits = 0;

  // weight idx
  uiBits += mhData.weightIdx + 1;
  if (mhData.weightIdx == iNumWeights - 1)
    uiBits--;

  // AMVP flag
  uiBits++;

  // ref idx
  uiBits += mhData.refIdx + 1;
  if (mhData.refIdx == iNumMHRefPics - 1)
  {
    uiBits--;
  }

  return uiBits;
}
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void InterSearch::initGeoAngleSelection(PredictionUnit& pu
#if JVET_Y0065_GPM_INTRA
                                      , IntraPrediction* pcIntraPred, const uint8_t (&mpm)[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS]
#endif
)
{
  xAMLGetCurBlkTemplate(pu, pu.lwidth(), pu.lheight());
  memset(&m_gpmacsSplitModeTmSelAvail[0][0][0], 0, sizeof(m_gpmacsSplitModeTmSelAvail));
  memset(&m_gpmPartTplCost[0][0][0][0], -1, sizeof(m_gpmPartTplCost));

  int16_t wIdx = floorLog2((uint32_t)pu.lwidth ()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2((uint32_t)pu.lheight()) - GEO_MIN_CU_LOG2;
  m_tplWeightTbl    = m_tplWeightTblDict   [hIdx][wIdx];
  m_tplColWeightTbl = m_tplColWeightTblDict[hIdx][wIdx];

#if JVET_Y0065_GPM_INTRA
  pcIntraPred->clearPrefilledIntraGPMRefTemplate();
  pcIntraPred->prefillIntraGPMReferenceSamples(pu, GEO_MODE_SEL_TM_SIZE, GEO_MODE_SEL_TM_SIZE);
  pcIntraPred->setPrefilledIntraGPMMPMModeAll(mpm);
#endif
}

void InterSearch::setGeoSplitModeToSyntaxTable(PredictionUnit& pu, MergeCtx& mergeCtx0, int mergeCand0, MergeCtx& mergeCtx1, int mergeCand1
#if JVET_AG0164_AFFINE_GPM
                                             , const AffineMergeCtx& affMergeCtx
#endif
#if JVET_Y0065_GPM_INTRA
                                             , IntraPrediction* pcIntraPred
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                                             , Mv* geoBvList
#endif
                                             , int mmvdCand0, int mmvdCand1)
{
#if JVET_Y0065_GPM_INTRA
  bool isIntra[2];
#if JVET_AI0082_GPM_WITH_INTER_IBC
  bool isIbc[2];
  xRemapMrgIndexAndMmvdIdx(mergeCand0, mergeCand1, mmvdCand0, mmvdCand1, isIntra[0], isIntra[1], isIbc[0], isIbc[1]);
#else
  xRemapMrgIndexAndMmvdIdx(mergeCand0, mergeCand1, mmvdCand0, mmvdCand1, isIntra[0], isIntra[1]);
#endif
#endif
  const int idx0 = mmvdCand0 + 1;
  const int idx1 = mmvdCand1 + 1;

  if ((m_gpmacsSplitModeTmSelAvail[idx0][idx1][mergeCand0] & ((uint16_t)1 << mergeCand1)) == 0)
  {
    uint8_t numValidInList = 0;
    uint8_t modeList[GEO_NUM_SIG_PARTMODE];
    selectGeoSplitModes(pu
#if JVET_AG0164_AFFINE_GPM
                      , affMergeCtx
#endif
#if JVET_Y0065_GPM_INTRA
                      , pcIntraPred
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                      , geoBvList
#endif
                      , m_gpmPartTplCost[idx0][mergeCand0]
                      , m_gpmPartTplCost[idx1][mergeCand1]
                      , mergeCtx0
                      , mergeCand0
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
                      + (isIntra[0] ? GEO_MAX_ALL_INTER_UNI_CANDS : 0)
#else
                      + (isIntra[0] ? GEO_MAX_NUM_UNI_CANDS : 0)
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                      + (isIbc[0] ? GEO_MAX_NUM_INTRA_CANDS : 0)
#endif
#endif
                      , mergeCtx1
                      , mergeCand1
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
                      + (isIntra[1] ? GEO_MAX_ALL_INTER_UNI_CANDS : 0)
#else
                      + (isIntra[1] ? GEO_MAX_NUM_UNI_CANDS : 0)
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                      + (isIbc[1] ? GEO_MAX_NUM_INTRA_CANDS : 0)
#endif
#endif
                      , numValidInList
                      , modeList
#if JVET_W0097_GPM_MMVD_TM
                      , (mmvdCand0 >= GPM_EXT_MMVD_MAX_REFINE_NUM ? -1 : mmvdCand0)
                      , (mmvdCand1 >= GPM_EXT_MMVD_MAX_REFINE_NUM ? -1 : mmvdCand1)
#endif
    );

    xSetGpmModeToSyntaxModeTable(numValidInList, modeList, m_gpmacsSplitModeTmSel[idx0][idx1][mergeCand0][mergeCand1]);
    m_gpmacsSplitModeTmSelAvail[idx0][idx1][mergeCand0] |= ((uint16_t)1 << mergeCand1);
  }
}

#if JVET_W0097_GPM_MMVD_TM && TM_MRG
#if JVET_AJ0274_GPM_AFFINE_TM
void InterSearch::setGeoTMSplitModeToSyntaxTable(PredictionUnit& pu, MergeCtx(&mergeCtx)[GEO_NUM_TM_MV_CAND], const AffineMergeCtx& affMergeCtx, int mergeCand0, int mergeCand1, int mmvdCand0, int mmvdCand1)
#else
void InterSearch::setGeoTMSplitModeToSyntaxTable(PredictionUnit& pu, MergeCtx(&mergeCtx)[GEO_NUM_TM_MV_CAND], int mergeCand0, int mergeCand1, int mmvdCand0, int mmvdCand1)
#endif
{
  const int idx0 = mmvdCand0 + 1;
  const int idx1 = mmvdCand1 + 1;

  if ((m_gpmacsSplitModeTmSelAvail[idx0][idx1][mergeCand0] & ((uint16_t)1 << mergeCand1)) == 0)
  {
    uint8_t numValidInList = 0;
    uint8_t modeList[GEO_NUM_SIG_PARTMODE];
    selectGeoTMSplitModes(pu
                        , m_gpmPartTplCost[idx0][mergeCand0]
                        , m_gpmPartTplCost[idx1][mergeCand1]
                        , mergeCtx
#if JVET_AJ0274_GPM_AFFINE_TM
                        , affMergeCtx
#endif
                        , mergeCand0
                        , mergeCand1
                        , numValidInList
                        , modeList
    );

    xSetGpmModeToSyntaxModeTable(numValidInList, modeList, m_gpmacsSplitModeTmSel[idx0][idx1][mergeCand0][mergeCand1]);
    m_gpmacsSplitModeTmSelAvail[idx0][idx1][mergeCand0] |= ((uint16_t)1 << mergeCand1);
  }
}
#endif

int InterSearch::convertGeoSplitModeToSyntax(int splitDir, int mergeCand0, int mergeCand1, int mmvdCand0, int mmvdCand1)
{
#if JVET_Y0065_GPM_INTRA
  bool isIntra[2];
#if JVET_AI0082_GPM_WITH_INTER_IBC
  bool isIbc[2];
  xRemapMrgIndexAndMmvdIdx(mergeCand0, mergeCand1, mmvdCand0, mmvdCand1, isIntra[0], isIntra[1], isIbc[0], isIbc[1]);
#else
  xRemapMrgIndexAndMmvdIdx(mergeCand0, mergeCand1, mmvdCand0, mmvdCand1, isIntra[0], isIntra[1]);
#endif
#endif
  return m_gpmacsSplitModeTmSel[mmvdCand0 + 1][mmvdCand1 + 1][mergeCand0][mergeCand1][splitDir];
}

bool InterSearch::selectGeoSplitModes(PredictionUnit &pu,
#if JVET_AG0164_AFFINE_GPM
                                      const AffineMergeCtx& affMergeCtx,
#endif
#if JVET_Y0065_GPM_INTRA
                                      IntraPrediction* pcIntraPred,
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                                      Mv* geoBvList,
#endif
                                      uint32_t (&gpmTplCostPart0)[2][GEO_NUM_PARTITION_MODE],
                                      uint32_t (&gpmTplCostPart1)[2][GEO_NUM_PARTITION_MODE],
                                      MergeCtx& mergeCtx0, int mergeCand0, MergeCtx& mergeCtx1, int mergeCand1, uint8_t& numValidInList, uint8_t (&modeList)[GEO_NUM_SIG_PARTMODE], int mmvdCand0, int mmvdCand1)
{
  if (!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1])
  {
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    return false;
  }
#if JVET_AG0164_AFFINE_GPM
  if (PU::checkRprRefExistingInGpm(pu, mergeCtx0, mergeCand0, mergeCtx1, mergeCand1, affMergeCtx))
#else
  if (PU::checkRprRefExistingInGpm(pu, mergeCtx0, mergeCand0, mergeCtx1, mergeCand1))
#endif
  {
    bool backupTplValid[2] = {m_bAMLTemplateAvailabe[0], m_bAMLTemplateAvailabe[1]};
    m_bAMLTemplateAvailabe[0] = false;
    m_bAMLTemplateAvailabe[1] = false;
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    m_bAMLTemplateAvailabe[0] = backupTplValid[0];
    m_bAMLTemplateAvailabe[1] = backupTplValid[1];
    return false;
  }

  bool fillRefTplPart0 = gpmTplCostPart0[0][0] == std::numeric_limits<uint32_t>::max();
  bool fillRefTplPart1 = gpmTplCostPart1[1][0] == std::numeric_limits<uint32_t>::max();
  Pel* pRefTopPart0    = m_acYuvRefAMLTemplatePart0[0];
  Pel* pRefLeftPart0   = m_acYuvRefAMLTemplatePart0[1];
  Pel* pRefTopPart1    = m_acYuvRefAMLTemplatePart1[0];
  Pel* pRefLeftPart1   = m_acYuvRefAMLTemplatePart1[1];
#if JVET_AI0082_GPM_WITH_INTER_IBC
  std::vector<Pel>* lut = m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() ? &m_pcReshape->getInvLUT() : nullptr;
#endif

  // First partition
  if (fillRefTplPart0)
  {
#if JVET_AG0164_AFFINE_GPM
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx0, mergeCand0, mmvdCand0, pRefTopPart0, pRefLeftPart0, &affMergeCtx);
#else
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx0, mergeCand0, mmvdCand0, pRefTopPart0, pRefLeftPart0);
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
    fillPartGpmInterIbcRefTemplate<0, false>(pu, lut, geoBvList, mergeCand0, mmvdCand0, pRefTopPart0, pRefLeftPart0);
#endif
#if JVET_Y0065_GPM_INTRA
    xCollectIntraGeoPartCost<0>(pu, pcIntraPred, mergeCand0, gpmTplCostPart0[0]);
#endif
  }

  // Second
  if (fillRefTplPart1)
  {
#if JVET_AG0164_AFFINE_GPM
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx1, mergeCand1, mmvdCand1, pRefTopPart1, pRefLeftPart1, &affMergeCtx);
#else
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx1, mergeCand1, mmvdCand1, pRefTopPart1, pRefLeftPart1);
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
    fillPartGpmInterIbcRefTemplate<1, false>(pu, lut, geoBvList, mergeCand1, mmvdCand1, pRefTopPart1, pRefLeftPart1);
#endif
#if JVET_Y0065_GPM_INTRA
    xCollectIntraGeoPartCost<1>(pu, pcIntraPred, mergeCand1, gpmTplCostPart1[1]);
#endif
  }

  // Get mode lists
  getBestGeoModeListEncoder(pu, numValidInList, modeList, pRefTopPart0, pRefLeftPart0, pRefTopPart1, pRefLeftPart1, gpmTplCostPart0, gpmTplCostPart1);
  return true;
}

#if JVET_W0097_GPM_MMVD_TM && TM_MRG
#if JVET_AJ0274_GPM_AFFINE_TM
bool InterSearch::selectGeoTMSplitModes (PredictionUnit &pu,
                                         uint32_t (&gpmTplCostPart0)[2][GEO_NUM_PARTITION_MODE],
                                         uint32_t (&gpmTplCostPart1)[2][GEO_NUM_PARTITION_MODE],
                                         MergeCtx (&mergeCtx)[GEO_NUM_TM_MV_CAND], const AffineMergeCtx& affMergeCtx,int mergeCand0, int mergeCand1, uint8_t& numValidInList, uint8_t (&modeList)[GEO_NUM_SIG_PARTMODE])
#else
bool InterSearch::selectGeoTMSplitModes (PredictionUnit &pu,
                                         uint32_t (&gpmTplCostPart0)[2][GEO_NUM_PARTITION_MODE],
                                         uint32_t (&gpmTplCostPart1)[2][GEO_NUM_PARTITION_MODE],
                                         MergeCtx (&mergeCtx)[GEO_NUM_TM_MV_CAND], int mergeCand0, int mergeCand1, uint8_t& numValidInList, uint8_t (&modeList)[GEO_NUM_SIG_PARTMODE])
#endif
{
  if (!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1])
  {
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    return false;
  }

#if JVET_AJ0274_GPM_AFFINE_TM
  if (PU::checkRprRefExistingInGpm(pu, mergeCtx[GEO_TM_OFF], mergeCand0, mergeCtx[GEO_TM_OFF], mergeCand1, affMergeCtx))
#else
  if (PU::checkRprRefExistingInGpm(pu, mergeCtx[GEO_TM_OFF], mergeCand0, mergeCtx[GEO_TM_OFF], mergeCand1))
#endif
  {
    bool backupTplValid[2] = {m_bAMLTemplateAvailabe[0], m_bAMLTemplateAvailabe[1]};
    m_bAMLTemplateAvailabe[0] = false;
    m_bAMLTemplateAvailabe[1] = false;
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    m_bAMLTemplateAvailabe[0] = backupTplValid[0];
    m_bAMLTemplateAvailabe[1] = backupTplValid[1];
    return false;
  }

  bool fillRefTplPart0  = gpmTplCostPart0[0][0] == std::numeric_limits<uint32_t>::max();
  bool fillRefTplPart1  = gpmTplCostPart1[1][0] == std::numeric_limits<uint32_t>::max();
  Pel* pRefTopPart0 [GEO_NUM_TM_MV_CAND] = {nullptr, m_acYuvRefAMLTemplatePart0[0], m_acYuvRefAMLTemplatePart0[2], nullptr                      }; // For mergeCtx[GEO_TM_SHAPE_AL] and mergeCtx[GEO_TM_SHAPE_A]
  Pel* pRefLeftPart0[GEO_NUM_TM_MV_CAND] = {nullptr, m_acYuvRefAMLTemplatePart0[1], m_acYuvRefAMLTemplatePart0[3], nullptr                      }; // For mergeCtx[GEO_TM_SHAPE_AL] and mergeCtx[GEO_TM_SHAPE_A]
  Pel* pRefTopPart1 [GEO_NUM_TM_MV_CAND] = {nullptr, m_acYuvRefAMLTemplatePart1[0], nullptr,                       m_acYuvRefAMLTemplatePart1[2]}; // For mergeCtx[GEO_TM_SHAPE_AL] and mergeCtx[GEO_TM_SHAPE_L]
  Pel* pRefLeftPart1[GEO_NUM_TM_MV_CAND] = {nullptr, m_acYuvRefAMLTemplatePart1[1], nullptr,                       m_acYuvRefAMLTemplatePart1[3]}; // For mergeCtx[GEO_TM_SHAPE_AL] and mergeCtx[GEO_TM_SHAPE_L]

  // First partition
  if (fillRefTplPart0)
  {
#if JVET_AJ0274_GPM_AFFINE_TM
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx[GEO_TM_SHAPE_AL], mergeCand0, -1, pRefTopPart0[GEO_TM_SHAPE_AL], pRefLeftPart0[GEO_TM_SHAPE_AL], &affMergeCtx);
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx[GEO_TM_SHAPE_A ], mergeCand0, -1, pRefTopPart0[GEO_TM_SHAPE_A ], pRefLeftPart0[GEO_TM_SHAPE_A ], &affMergeCtx);
#else
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx[GEO_TM_SHAPE_AL], mergeCand0, -1, pRefTopPart0[GEO_TM_SHAPE_AL], pRefLeftPart0[GEO_TM_SHAPE_AL]);
    fillPartGPMRefTemplate<0, false>(pu, mergeCtx[GEO_TM_SHAPE_A ], mergeCand0, -1, pRefTopPart0[GEO_TM_SHAPE_A ], pRefLeftPart0[GEO_TM_SHAPE_A ]);
#endif
  }

  // Second
  if (fillRefTplPart1)
  {
#if JVET_AJ0274_GPM_AFFINE_TM
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx[GEO_TM_SHAPE_AL], mergeCand1, -1, pRefTopPart1[GEO_TM_SHAPE_AL], pRefLeftPart1[GEO_TM_SHAPE_AL], &affMergeCtx);
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx[GEO_TM_SHAPE_L ], mergeCand1, -1, pRefTopPart1[GEO_TM_SHAPE_L ], pRefLeftPart1[GEO_TM_SHAPE_L ], &affMergeCtx);
#else
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx[GEO_TM_SHAPE_AL], mergeCand1, -1, pRefTopPart1[GEO_TM_SHAPE_AL], pRefLeftPart1[GEO_TM_SHAPE_AL]);
    fillPartGPMRefTemplate<1, false>(pu, mergeCtx[GEO_TM_SHAPE_L ], mergeCand1, -1, pRefTopPart1[GEO_TM_SHAPE_L ], pRefLeftPart1[GEO_TM_SHAPE_L ]);
#endif
  }

  // Get mode lists
  getBestGeoTMModeListEncoder(pu, numValidInList, modeList, pRefTopPart0, pRefLeftPart0, pRefTopPart1, pRefLeftPart1, gpmTplCostPart0, gpmTplCostPart1);
  return true;
}
#endif

void InterSearch::getBestGeoModeListEncoder(PredictionUnit &pu, uint8_t& numValidInList,
                                            uint8_t(&modeList)[GEO_NUM_SIG_PARTMODE],
                                            Pel* pRefTopPart0, Pel* pRefLeftPart0,
                                            Pel* pRefTopPart1, Pel* pRefLeftPart1,
                                            uint32_t(&gpmTplCostPart0)[2][GEO_NUM_PARTITION_MODE],
                                            uint32_t(&gpmTplCostPart1)[2][GEO_NUM_PARTITION_MODE])
{
  if (!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1])
  {
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    return;
  }

  // Check mode
  bool filledRefTplPart0 = gpmTplCostPart0[0][0] == std::numeric_limits<uint32_t>::max();
  bool filledRefTplPart1 = gpmTplCostPart1[1][0] == std::numeric_limits<uint32_t>::max();
  int bitDepth = pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
    int whIdx = !pu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(pu.lwidth()) - floorLog2(pu.lheight()) + GEO_SQUARE_IDX);
#endif
  if (m_bAMLTemplateAvailabe[0])
  {
    SizeType   szPerLine            = pu.lwidth();
    PelUnitBuf pcBufPredCurTop      = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[0][0], szPerLine, GEO_MODE_SEL_TM_SIZE));
    PelUnitBuf pcBufPredRefTopPart0 = PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart0,                szPerLine, GEO_MODE_SEL_TM_SIZE));
    PelUnitBuf pcBufPredRefTopPart1 = PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart1,                szPerLine, GEO_MODE_SEL_TM_SIZE));

    const int maskStride2[3] = { -(int)szPerLine, (int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { GEO_WEIGHT_MASK_SIZE_EXT, GEO_WEIGHT_MASK_SIZE_EXT, -GEO_WEIGHT_MASK_SIZE_EXT }; // mask stride
    const int stepX[3] = { 1, -1, 1 };

    // Cost of partition 0
    if(filledRefTplPart0)
    {
      GetAbsDiffPerSample(pcBufPredRefTopPart0.Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart0.Y());
      uint32_t fullCostPart0 = (uint32_t)GetSampleSum(pcBufPredRefTopPart0.Y(), bitDepth);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 0>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefTopPart0.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart0[0][splitDirIdx] = tempDist;
        gpmTplCostPart0[1][splitDirIdx] = fullCostPart0 - tempDist; // pre-calculated
#else
        gpmTplCostPart0[0][splitDir] = tempDist;
        gpmTplCostPart0[1][splitDir] = fullCostPart0 - tempDist; // pre-calculated
#endif
      }
    }

    // Cost of partition 1
    if(filledRefTplPart1)
    {
      GetAbsDiffPerSample(pcBufPredRefTopPart1.Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart1.Y());
      uint32_t fullCostPart1 = (uint32_t)GetSampleSum(pcBufPredRefTopPart1.Y(), bitDepth);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 0>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefTopPart1.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart1[0][splitDirIdx] = tempDist;  // pre-calculated
        gpmTplCostPart1[1][splitDirIdx] = fullCostPart1 - tempDist;
#else
        gpmTplCostPart1[0][splitDir] = tempDist;  // pre-calculated
        gpmTplCostPart1[1][splitDir] = fullCostPart1 - tempDist;
#endif
      }
    }
  }
  else
  {
    if (filledRefTplPart0)
    {
      memset(gpmTplCostPart0[0], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
      memset(gpmTplCostPart0[1], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
    }
    if (filledRefTplPart1)
    {
      memset(gpmTplCostPart1[1], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
      memset(gpmTplCostPart1[0], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
    }
  }

  if (m_bAMLTemplateAvailabe[1])
  {
    SizeType   szPerLine             = pu.lheight();
    PelUnitBuf pcBufPredCurLeft      = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[1][0], szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation
    PelUnitBuf pcBufPredRefLeftPart0 = PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart0,               szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation
    PelUnitBuf pcBufPredRefLeftPart1 = PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart1,               szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation

    const int maskStride2[3] = { -(int)szPerLine, -(int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { (int)szPerLine, (int)szPerLine, (int)szPerLine }; // mask stride
    const int stepX[3] = { 1, 1, 1 };

#if JVET_AJ0107_GPM_SHAPE_ADAPT
    int whIdx = !pu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(pu.lwidth()) - floorLog2(pu.lheight()) + GEO_SQUARE_IDX);
#endif
    // Cost of partition 0
    if (filledRefTplPart0)
    {
      GetAbsDiffPerSample(pcBufPredRefLeftPart0.Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart0.Y());
      uint32_t fullCostPart0 = (uint32_t)GetSampleSum(pcBufPredRefLeftPart0.Y(), bitDepth);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 2>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefLeftPart0.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart0[0][splitDirIdx] += tempDist;
        gpmTplCostPart0[1][splitDirIdx] += fullCostPart0 - tempDist; // pre-calculated
#else
        gpmTplCostPart0[0][splitDir] += tempDist;
        gpmTplCostPart0[1][splitDir] += fullCostPart0 - tempDist; // pre-calculated
#endif
      }
    }

    // Cost of partition 1
    if (filledRefTplPart1)
    {
      GetAbsDiffPerSample(pcBufPredRefLeftPart1.Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart1.Y());
      uint32_t fullCostPart1 = (uint32_t)GetSampleSum(pcBufPredRefLeftPart1.Y(), bitDepth);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 2>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefLeftPart1.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart1[0][splitDirIdx] += tempDist;  // pre-calculated
        gpmTplCostPart1[1][splitDirIdx] += fullCostPart1 - tempDist;
#else
        gpmTplCostPart1[0][splitDir] += tempDist;  // pre-calculated
        gpmTplCostPart1[1][splitDir] += fullCostPart1 - tempDist;
#endif
      }
    }
  }

  // Check split mode cost
  uint32_t uiCost[GEO_NUM_PARTITION_MODE];
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
  {
    uiCost[splitDir] = gpmTplCostPart0[0][splitDir] + gpmTplCostPart1[1][splitDir];
  }

  // Find best N candidates
  numValidInList = (uint8_t)getIndexMappingTableToSortedArray1D<uint32_t, GEO_NUM_PARTITION_MODE, uint8_t, GEO_NUM_SIG_PARTMODE>(uiCost, modeList);

}

#if JVET_W0097_GPM_MMVD_TM && TM_MRG
void InterSearch::getBestGeoTMModeListEncoder(PredictionUnit &pu, uint8_t& numValidInList,
                                              uint8_t(&modeList)[GEO_NUM_SIG_PARTMODE],
                                              Pel* (&pRefTopPart0)[GEO_NUM_TM_MV_CAND], Pel* (&pRefLeftPart0)[GEO_NUM_TM_MV_CAND],
                                              Pel* (&pRefTopPart1)[GEO_NUM_TM_MV_CAND], Pel* (&pRefLeftPart1)[GEO_NUM_TM_MV_CAND],
                                              uint32_t(&gpmTplCostPart0)[2][GEO_NUM_PARTITION_MODE],
                                              uint32_t(&gpmTplCostPart1)[2][GEO_NUM_PARTITION_MODE])
{
  if (!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1])
  {
    getBestGeoModeList(pu, numValidInList, modeList, nullptr, nullptr, nullptr, nullptr);
    return;
  }

  // Check mode
  bool filledRefTplPart0 = gpmTplCostPart0[0][0] == std::numeric_limits<uint32_t>::max();
  bool filledRefTplPart1 = gpmTplCostPart1[1][0] == std::numeric_limits<uint32_t>::max();
  int bitDepth = pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA);

#if JVET_AJ0107_GPM_SHAPE_ADAPT
    int whIdx = !pu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(pu.lwidth()) - floorLog2(pu.lheight()) + GEO_SQUARE_IDX);
#endif
  if (m_bAMLTemplateAvailabe[0])
  {
    SizeType   szPerLine       = pu.lwidth();
    PelUnitBuf pcBufPredCurTop = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[0][0], szPerLine, GEO_MODE_SEL_TM_SIZE));
    PelUnitBuf pcBufPredRefTopPart0[GEO_NUM_TM_MV_CAND] = {PelUnitBuf(), 
                                                           PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart0[GEO_TM_SHAPE_AL], szPerLine, GEO_MODE_SEL_TM_SIZE)),
                                                           PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart0[GEO_TM_SHAPE_A ], szPerLine, GEO_MODE_SEL_TM_SIZE)),
                                                           PelUnitBuf()};
    PelUnitBuf pcBufPredRefTopPart1[GEO_NUM_TM_MV_CAND] = {PelUnitBuf(),
                                                           PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart1[GEO_TM_SHAPE_AL], szPerLine, GEO_MODE_SEL_TM_SIZE)),
                                                           PelUnitBuf(),
                                                           PelUnitBuf(pu.chromaFormat, PelBuf(pRefTopPart1[GEO_TM_SHAPE_L ], szPerLine, GEO_MODE_SEL_TM_SIZE))};

    const int maskStride2[3] = { -(int)szPerLine, (int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { GEO_WEIGHT_MASK_SIZE_EXT, GEO_WEIGHT_MASK_SIZE_EXT, -GEO_WEIGHT_MASK_SIZE_EXT }; // mask stride
    const int stepX[3] = { 1, -1, 1 };

    // Cost of partition 0
    if(filledRefTplPart0)
    {
      GetAbsDiffPerSample(pcBufPredRefTopPart0[GEO_TM_SHAPE_AL].Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart0[GEO_TM_SHAPE_AL].Y());
      GetAbsDiffPerSample(pcBufPredRefTopPart0[GEO_TM_SHAPE_A ].Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart0[GEO_TM_SHAPE_A ].Y());

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        uint8_t shapeIdx  = g_geoTmShape[0][g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 0>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefTopPart0[shapeIdx].Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart0[0][splitDirIdx] = tempDist;
#else
        gpmTplCostPart0[0][splitDir] = tempDist;
#endif
      }
    }

    // Cost of partition 1
    if(filledRefTplPart1)
    {
      GetAbsDiffPerSample(pcBufPredRefTopPart1[GEO_TM_SHAPE_AL].Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart1[GEO_TM_SHAPE_AL].Y());
      GetAbsDiffPerSample(pcBufPredRefTopPart1[GEO_TM_SHAPE_L ].Y(), pcBufPredCurTop.Y(), pcBufPredRefTopPart1[GEO_TM_SHAPE_L ].Y());

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        uint8_t shapeIdx  = g_geoTmShape[1][g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 0>(splitDir);
        uint32_t tempDist = (uint32_t)Get01InvMaskedSampleSum(pcBufPredRefTopPart1[shapeIdx].Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart1[1][splitDirIdx] = tempDist;
#else
        gpmTplCostPart1[1][splitDir] = tempDist;
#endif
      }
    }
  }
  else
  {
    if (filledRefTplPart0)
    {
      memset(gpmTplCostPart0[0], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
    }
    if (filledRefTplPart1)
    {
      memset(gpmTplCostPart1[1], 0, sizeof(uint32_t) * GEO_NUM_PARTITION_MODE);
    }
  }

  if (m_bAMLTemplateAvailabe[1])
  {
    SizeType   szPerLine        = pu.lheight();
    PelUnitBuf pcBufPredCurLeft = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[1][0], szPerLine, GEO_MODE_SEL_TM_SIZE)); // reordered to make it 1 row to enable SIMD
    PelUnitBuf pcBufPredRefLeftPart0[GEO_NUM_TM_MV_CAND] = {PelUnitBuf(),
                                                            PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart0[GEO_TM_SHAPE_AL], szPerLine, GEO_MODE_SEL_TM_SIZE)), // To enable SIMD for cost computation
                                                            PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart0[GEO_TM_SHAPE_A ], szPerLine, GEO_MODE_SEL_TM_SIZE)), // To enable SIMD for cost computation
                                                            PelUnitBuf()};
    PelUnitBuf pcBufPredRefLeftPart1[GEO_NUM_TM_MV_CAND] = { PelUnitBuf(),
                                                            PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart1[GEO_TM_SHAPE_AL], szPerLine, GEO_MODE_SEL_TM_SIZE)), // To enable SIMD for cost computation
                                                            PelUnitBuf(),
                                                            PelUnitBuf(pu.chromaFormat, PelBuf(pRefLeftPart1[GEO_TM_SHAPE_L ], szPerLine, GEO_MODE_SEL_TM_SIZE))}; // To enable SIMD for cost computation

    const int maskStride2[3] = { -(int)szPerLine, -(int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { (int)szPerLine, (int)szPerLine, (int)szPerLine }; // mask stride
    const int stepX[3] = { 1, 1, 1 };

    // Cost of partition 0
    if (filledRefTplPart0)
    {
      GetAbsDiffPerSample(pcBufPredRefLeftPart0[GEO_TM_SHAPE_AL].Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart0[GEO_TM_SHAPE_AL].Y());
      GetAbsDiffPerSample(pcBufPredRefLeftPart0[GEO_TM_SHAPE_A ].Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart0[GEO_TM_SHAPE_A ].Y());

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        uint8_t shapeIdx  = g_geoTmShape[0][g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 2>(splitDir);
        uint32_t tempDist = (uint32_t)Get01MaskedSampleSum(pcBufPredRefLeftPart0[shapeIdx].Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart0[0][splitDirIdx] += tempDist;
#else
        gpmTplCostPart0[0][splitDir] += tempDist;
#endif
      }
    }

    // Cost of partition 1
    if (filledRefTplPart1)
    {
      GetAbsDiffPerSample(pcBufPredRefLeftPart1[GEO_TM_SHAPE_AL].Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart1[GEO_TM_SHAPE_AL].Y());
      GetAbsDiffPerSample(pcBufPredRefLeftPart1[GEO_TM_SHAPE_L ].Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeftPart1[GEO_TM_SHAPE_L ].Y());

#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for (int splitDirIdx = 0; splitDirIdx < GEO_NUM_PARTITION_MODE; ++splitDirIdx)
      {
        int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
      {
#endif
        int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
        uint8_t shapeIdx  = g_geoTmShape[1][g_geoParams[splitDir][0]];
        Pel* mask = getTplWeightTableCU<false, 2>(splitDir);
        uint32_t tempDist = (uint32_t)Get01InvMaskedSampleSum(pcBufPredRefLeftPart1[shapeIdx].Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
        gpmTplCostPart1[1][splitDirIdx] += tempDist;
#else
        gpmTplCostPart1[1][splitDir] += tempDist;
#endif
      }
    }
  }

  // Check split mode cost
  uint32_t uiCost[GEO_NUM_PARTITION_MODE];
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
  {
    uiCost[splitDir] = gpmTplCostPart0[0][splitDir] + gpmTplCostPart1[1][splitDir];
  }

  // Find best N candidates
  numValidInList = (uint8_t)getIndexMappingTableToSortedArray1D<uint32_t, GEO_NUM_PARTITION_MODE, uint8_t, GEO_NUM_SIG_PARTMODE>(uiCost, modeList);

}
#endif

#if JVET_Y0065_GPM_INTRA
template <uint8_t partIdx>
void InterSearch::xCollectIntraGeoPartCost(PredictionUnit &pu, IntraPrediction* pcIntraPred, int mergeCand, uint32_t(&gpmTplCost)[GEO_NUM_PARTITION_MODE])
{
#if JVET_AG0164_AFFINE_GPM
  if ((!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1]) || gpmTplCost[0] != std::numeric_limits<uint32_t>::max() || mergeCand < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
  if ((!m_bAMLTemplateAvailabe[0] && !m_bAMLTemplateAvailabe[1]) || gpmTplCost[0] != std::numeric_limits<uint32_t>::max() || mergeCand < GEO_MAX_NUM_UNI_CANDS)
#endif
  {
    return;
  }
#if JVET_AI0082_GPM_WITH_INTER_IBC
#if JVET_AG0164_AFFINE_GPM
  if (mergeCand >= GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)
#else
  if (mergeCand >= GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS)
#endif
  {
    return;
  }
#endif

  std::vector<Pel>* LUT = m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && m_pcReshape->getCTUFlag() ? &m_pcReshape->getInvLUT() : nullptr;
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  pcIntraPred->fillIntraGPMRefTemplateAll(pu, m_bAMLTemplateAvailabe[0], m_bAMLTemplateAvailabe[1], false, false, false, LUT, (partIdx == 0 ? mergeCand : 0), (partIdx == 1 ? mergeCand : 0));
#else
  pcIntraPred->fillIntraGPMRefTemplateAll(pu, m_bAMLTemplateAvailabe[0], m_bAMLTemplateAvailabe[1], true, false, false, LUT, (partIdx == 0 ? mergeCand : 0), (partIdx == 1 ? mergeCand : 0));
#endif
#if JVET_AG0164_AFFINE_GPM
  int  realCandIdx = mergeCand - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
  int  realCandIdx = mergeCand - GEO_MAX_NUM_UNI_CANDS;
#endif
  int  bitDepth    = pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA);
  Pel* pDiffTop    = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[0] : m_acYuvRefAMLTemplatePart1[0];
  Pel* pDiffLeft   = partIdx == 0 ? m_acYuvRefAMLTemplatePart0[1] : m_acYuvRefAMLTemplatePart1[1];

  static_vector<int, GEO_NUM_PARTITION_MODE> intraModeToSplitDirAll[NUM_INTRA_MODE];
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  int whIdx = !pu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(pu.lwidth()) - floorLog2(pu.lheight()) + GEO_SQUARE_IDX);
#endif
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; ++splitDir)
  {
    uint8_t intraMode = pcIntraPred->getPrefilledIntraGPMMPMMode(partIdx, splitDir, realCandIdx);
    intraModeToSplitDirAll[intraMode].push_back(splitDir);
  }

  if (m_bAMLTemplateAvailabe[0])
  {
    SizeType   szPerLine        = pu.lwidth();
    PelUnitBuf pcBufPredCurTop  = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[0][0], szPerLine, GEO_MODE_SEL_TM_SIZE));
    PelUnitBuf pcBufPredRefTop  = PelUnitBuf(pu.chromaFormat, PelBuf(nullptr,                     szPerLine, GEO_MODE_SEL_TM_SIZE));
    PelUnitBuf pcBufDiffTop     = PelUnitBuf(pu.chromaFormat, PelBuf(pDiffTop,                    szPerLine, GEO_MODE_SEL_TM_SIZE));

    const int maskStride2[3] = { -(int)szPerLine, (int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { GEO_WEIGHT_MASK_SIZE_EXT, GEO_WEIGHT_MASK_SIZE_EXT, -GEO_WEIGHT_MASK_SIZE_EXT }; // mask stride
    const int stepX[3] = { 1, -1, 1 };

    for (uint8_t intraMode = 0; intraMode < NUM_INTRA_MODE; ++intraMode)
    {
      static_vector<int, GEO_NUM_PARTITION_MODE>& toSplitDir = intraModeToSplitDirAll[intraMode];
      if (toSplitDir.size() > 0)
      {
        pcBufPredRefTop.Y().buf = pcIntraPred->getPrefilledIntraGPMRefTemplate(intraMode, 0);
        GetAbsDiffPerSample(pcBufDiffTop.Y(), pcBufPredCurTop.Y(), pcBufPredRefTop.Y());

        for (int i = 0; i < toSplitDir.size(); ++i)
        {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
          int splitDirIdx = toSplitDir[i];
          CHECK(splitDirIdx >= GEO_NUM_PARTITION_MODE, "Invalid GPM partition");
          int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
          int splitDir = toSplitDir[i];
#endif
          int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
          Pel* mask = getTplWeightTableCU<false, 0>(splitDir);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
          gpmTplCost[splitDirIdx] = (uint32_t)GetSampleSumFunc(partIdx + 2, pcBufDiffTop.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#else          
          gpmTplCost[splitDir] = (uint32_t)GetSampleSumFunc(partIdx + 2, pcBufDiffTop.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#endif
        }
      }
    }
  }
  else
  {
    memset(gpmTplCost, 0, sizeof(gpmTplCost));
  }

  if (m_bAMLTemplateAvailabe[1])
  {
    SizeType   szPerLine        = pu.lheight();
    PelUnitBuf pcBufPredCurLeft = PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvCurAMLTemplate[1][0], szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation
    PelUnitBuf pcBufPredRefLeft = PelUnitBuf(pu.chromaFormat, PelBuf(nullptr,                     szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation
    PelUnitBuf pcBufDiffLeft    = PelUnitBuf(pu.chromaFormat, PelBuf(pDiffLeft,                   szPerLine, GEO_MODE_SEL_TM_SIZE)); // To enable SIMD for cost computation

    const int maskStride2[3] = { -(int)szPerLine, -(int)szPerLine, -(int)szPerLine }; // template length
    const int maskStride[3] = { (int)szPerLine, (int)szPerLine, (int)szPerLine }; // mask stride
    const int stepX[3] = { 1, 1, 1 };

    for (uint8_t intraMode = 0; intraMode < NUM_INTRA_MODE; ++intraMode)
    {
      static_vector<int, GEO_NUM_PARTITION_MODE>& toSplitDir = intraModeToSplitDirAll[intraMode];
      if (toSplitDir.size() > 0)
      {
        pcBufPredRefLeft.Y().buf = pcIntraPred->getPrefilledIntraGPMRefTemplate(intraMode, 1);
        GetAbsDiffPerSample(pcBufDiffLeft.Y(), pcBufPredCurLeft.Y(), pcBufPredRefLeft.Y());

        for (int i = 0; i < toSplitDir.size(); ++i)
        {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
          int splitDirIdx = toSplitDir[i];
          CHECK(splitDirIdx >= GEO_NUM_PARTITION_MODE, "Invalid GPM partition");
          int splitDir = g_gpmSplitDir[whIdx][splitDirIdx];
#else
          int splitDir = toSplitDir[i];
#endif
          int16_t mirrorIdx = g_angle2mirror[g_geoParams[splitDir][0]];
          Pel* mask = getTplWeightTableCU<false, 2>(splitDir);
#if JVET_AJ0107_GPM_SHAPE_ADAPT
          gpmTplCost[splitDirIdx] += (uint32_t)GetSampleSumFunc(partIdx + 2, pcBufDiffLeft.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#else
          gpmTplCost[splitDir] += (uint32_t)GetSampleSumFunc(partIdx + 2, pcBufDiffLeft.Y(), bitDepth, mask, stepX[mirrorIdx], maskStride[mirrorIdx], maskStride2[mirrorIdx]);
#endif
        }
      }
    }
  }
}
#endif
#endif

// AMVP
#if JVET_X0083_BM_AMVP_MERGE_MODE
void InterSearch::xEstimateMvPredAMVP( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, bool bFilled, Distortion* puiDistBiP, MvField* mvFieldAmListCommon )
#else
void InterSearch::xEstimateMvPredAMVP( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& rAMVPInfo, bool bFilled, Distortion* puiDistBiP )
#endif
{
  Mv         cBestMv;
  int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  int        i;

  AMVPInfo*  pcAMVPInfo = &rAMVPInfo;

  // Fill the MV Candidates
  if (!bFilled)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (pu.amvpMergeModeFlag[1 - eRefPicList] == true)
    {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      const int mvFieldAmvpIdx0 = MAX_NUM_AMVP_CANDS_MAX_REF + iRefIdx * AMVP_MAX_NUM_CANDS_MEM;
      CHECK(mvFieldAmListCommon[mvFieldAmvpIdx0].refIdx != iRefIdx, "this is not possible");
#else
      const int mvFieldAmvpIdx0 = MAX_NUM_AMVP_CANDS_MAX_REF + iRefIdx * AMVP_MAX_NUM_CANDS;
#endif
      pcAMVPInfo->mvCand[0] = mvFieldAmListCommon[mvFieldAmvpIdx0].mv;
      pcAMVPInfo->numCand = 1;
#if !TM_AMVP || JVET_Y0128_NON_CTC || JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE || JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (TM_AMVP && !JVET_Y0128_NON_CTC && !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE)
      if(!pu.cu->cs->sps->getUseTMAmvpMode())
      {
#endif
      const int mvFieldAmvpIdx1 = mvFieldAmvpIdx0 + 1;
      if (mvFieldAmListCommon[mvFieldAmvpIdx1].refIdx >= 0)
      {
        pcAMVPInfo->mvCand[1] = mvFieldAmListCommon[mvFieldAmvpIdx1].mv;
        pcAMVPInfo->numCand = 2;
      }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (TM_AMVP && !JVET_Y0128_NON_CTC && !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE)
      }
#endif
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      const int mvFieldAmvpIdx2 = mvFieldAmvpIdx0 + 2;
      if (mvFieldAmListCommon[mvFieldAmvpIdx2].refIdx >= 0)
      {
        pcAMVPInfo->mvCand[2] = mvFieldAmListCommon[mvFieldAmvpIdx2].mv;
        pcAMVPInfo->numCand = 3;
      }
#endif
      return;
    }
#endif
    PU::fillMvpCand( pu, eRefPicList, iRefIdx, *pcAMVPInfo
#if TM_AMVP
                   , this
#endif
    );
  }
#if INTER_LIC && RPR_ENABLE
  // xPredInterBlk may call PU::checkRprLicCondition()
#if JVET_Y0128_NON_CTC
  pu.interDir = (uint8_t)(eRefPicList + 1);
#endif
  pu.refIdx[eRefPicList]      = iRefIdx;
  pu.refIdx[1 - eRefPicList]  = NOT_VALID;
#endif

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0];

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  //-- Check Minimum Cost.
  for( i = 0 ; i < pcAMVPInfo->numCand; i++)
  {
#if TM_AMVP
    Distortion uiTmpCost = xGetTemplateCost( pu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, pcAMVPInfo->numCand, eRefPicList, iRefIdx );
#else
    Distortion uiTmpCost = xGetTemplateCost( pu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );
#endif
    if( uiBestCost > uiTmpCost )
    {
      uiBestCost     = uiTmpCost;
      cBestMv        = pcAMVPInfo->mvCand[i];
      iBestIdx       = i;
      (*puiDistBiP)  = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred = cBestMv;
  pu.mvpIdx[eRefPicList] = iBestIdx;
  pu.mvpNum[eRefPicList] = pcAMVPInfo->numCand;

  return;
}

uint32_t InterSearch::xGetMvpIdxBits(int iIdx, int iNum)
{
  CHECK(iIdx < 0 || iNum < 0 || iIdx >= iNum, "Invalid parameters");

  if (iNum == 1)
  {
    return 0;
  }

  uint32_t uiLength = 1;
  int iTemp = iIdx;
  if ( iTemp == 0 )
  {
    return uiLength;
  }

  bool bCodeLast = ( iNum-1 > iTemp );

  uiLength += (iTemp-1);

  if( bCodeLast )
  {
    uiLength++;
  }

  return uiLength;
}

void InterSearch::xGetBlkBits( bool bPSlice, int iPartIdx, uint32_t uiLastMode, uint32_t uiBlkBit[3])
{
  uiBlkBit[0] = (! bPSlice) ? 3 : 1;
  uiBlkBit[1] = 3;
  uiBlkBit[2] = 5;
}

void InterSearch::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->numCand = pSrc->numCand;
  for (int i = 0; i < pSrc->numCand; i++)
  {
    pDst->mvCand[i] = pSrc->mvCand[i];
  }
#if TM_AMVP
  pDst->maxSimilarityThreshold = pSrc->maxSimilarityThreshold;
#endif
}

#if JVET_AG0098_AMVP_WITH_SBTMVP
void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost, const uint8_t imv, const bool amvpSbTmvp, const int amvpSbTmvpMvdIdx, const int numAmvpSbTmvpOffset)
#else
void InterSearch::xCheckBestMVP ( RefPicList eRefPicList, Mv cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t& ruiBits, Distortion& ruiCost, const uint8_t imv )
#endif
{
  if ( imv > 0 && imv < 3 )
  {
    return;
  }

  AMVPInfo* pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)
  {
    return;
  }

  m_pcRdCost->setCostScale ( 0    );

  int iBestMVPIdx = riMVPIdx;

  Mv pred = rcMvPred;
  pred.changeTransPrecInternal2Amvr(imv);
  m_pcRdCost->setPredictor( pred );
  Mv mv = cMv;
  mv.changeTransPrecInternal2Amvr(imv);
#if JVET_AG0098_AMVP_WITH_SBTMVP
  int iOrgMvBits = amvpSbTmvp ? m_pcRdCost->getAmvpSbTmvpBitsOfVectorWithPredictor(amvpSbTmvpMvdIdx < 0 ? -1 : (amvpSbTmvpMvdIdx >> 2), numAmvpSbTmvpOffset) : m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
#else
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
#endif
#if TM_AMVP
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][pcAMVPInfo->numCand];
#else
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
#endif
  int iBestMvBits = iOrgMvBits;

  for (int iMVPIdx = 0; iMVPIdx < pcAMVPInfo->numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }

    pred = pcAMVPInfo->mvCand[iMVPIdx];
    pred.changeTransPrecInternal2Amvr(imv);
    m_pcRdCost->setPredictor( pred );
#if JVET_AG0098_AMVP_WITH_SBTMVP
    int iMvBits = amvpSbTmvp ? m_pcRdCost->getAmvpSbTmvpBitsOfVectorWithPredictor(amvpSbTmvpMvdIdx < 0 ? -1 : (amvpSbTmvpMvdIdx >> 2), numAmvpSbTmvpOffset) : m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
#else
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
#endif
#if TM_AMVP
    iMvBits += m_auiMVPIdxCost[iMVPIdx][pcAMVPInfo->numCand];
#else
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];
#endif

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  //if changed
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits ))  + m_pcRdCost->getCost( ruiBits );
  }
}


Distortion InterSearch::xGetTemplateCost( const PredictionUnit& pu,
                                          PelUnitBuf& origBuf,
                                          PelUnitBuf& predBuf,
                                          Mv          cMvCand,
                                          int         iMVPIdx,
                                          int         iMVPNum,
                                          RefPicList  eRefPicList,
                                          int         iRefIdx
)
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture* picRef = pu.cu->slice->getRefPic( eRefPicList, iRefIdx );
  clipMv( cMvCand, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );

  // prediction pattern
  const bool bi = pu.cu->slice->testWeightPred() && pu.cu->slice->getSliceType()==P_SLICE
#if INTER_LIC
    && !pu.cu->licFlag
#endif
    ;


  xPredInterBlk( COMPONENT_Y, pu, picRef, cMvCand, predBuf, bi, pu.cu->slice->clpRng( COMPONENT_Y )
                , false
                , false
                );

  if ( bi )
  {
    xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, iRefIdx, m_maxCompIDToPred );
  }

  // calc distortion

  uiCost = m_pcRdCost->getDistPart(origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD);
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );

  return uiCost;
}

Distortion InterSearch::xGetAffineTemplateCost( PredictionUnit& pu, PelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList eRefPicList, int iRefIdx )
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture* picRef = pu.cu->slice->getRefPic( eRefPicList, iRefIdx );
#if INTER_LIC && RPR_ENABLE
  // xPredAffineBlk may call PU::checkRprLicCondition()
#if JVET_Y0128_NON_CTC
  pu.interDir = (uint8_t)(eRefPicList + 1);
#endif
  pu.refIdx[eRefPicList]    = iRefIdx;
  pu.refIdx[1-eRefPicList]  = NOT_VALID;
#endif

  // prediction pattern
  const bool bi = pu.cu->slice->testWeightPred() && pu.cu->slice->getSliceType()==P_SLICE
#if INTER_LIC
    && !pu.cu->licFlag
#endif
    ;
  Mv mv[3];
  memcpy(mv, acMvCand, sizeof(mv));
  m_iRefListIdx = eRefPicList;
#if JVET_Z0136_OOB
  xPredAffineBlk(COMPONENT_Y, pu, picRef, mv, predBuf, bi, pu.cu->slice->clpRng(COMPONENT_Y), eRefPicList);
#else
  xPredAffineBlk(COMPONENT_Y, pu, picRef, mv, predBuf, bi, pu.cu->slice->clpRng(COMPONENT_Y));
#endif
  if( bi )
  {
    xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, iRefIdx, m_maxCompIDToPred );
  }

  // calc distortion
  enum DFunc distFunc = (pu.cs->slice->getDisableSATDForRD()) ? DF_SAD : DF_HAD;
  uiCost  = m_pcRdCost->getDistPart( origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y
    , distFunc
  );
  uiCost += m_pcRdCost->getCost( m_auiMVPIdxCost[iMVPIdx][iMVPNum] );
  DTRACE( g_trace_ctx, D_COMMON, " (%d) affineTemplateCost=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCost );
  return uiCost;
}

#if MULTI_HYP_PRED
void InterSearch::xMotionEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Mv& rcMvPred, int iRefIdxPred, Mv& rcMv, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, bool bBi, int weight)
#else
void InterSearch::xMotionEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Mv& rcMvPred, int iRefIdxPred, Mv& rcMv, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, bool bBi)
#endif
{
#if MULTI_HYP_PRED
  if (!weight)
#endif
  if( pu.cu->cs->sps->getUseBcw() && pu.cu->bcwIdx != BCW_DEFAULT && !bBi && xReadBufferedUniMv(pu, eRefPicList, iRefIdxPred, rcMvPred, rcMv, ruiBits, ruiCost) )
  {
    return;
  }

  Mv cMvHalf, cMvQter;
  CHECK(eRefPicList >= MAX_NUM_REF_LIST_ADAPT_SR || iRefIdxPred>=int(MAX_IDX_ADAPT_SR), "Invalid reference picture list");
  m_iSearchRange = m_aaiAdaptSR[eRefPicList][iRefIdxPred];
#if MULTI_HYP_PRED 
  if (weight)
  {
    m_iSearchRange = std::min(m_iSearchRange, MULTI_HYP_PRED_SEARCH_RANGE);
  }
#endif

  int    iSrchRng   = (bBi ? m_bipredSearchRange : m_iSearchRange);
  double fWeight    = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );
  PelUnitBuf* pBuf       = &origBuf;

  if(bBi) // Bi-predictive ME
  {
#if MULTI_HYP_PRED
    CHECK(weight, "Multi Hyp: bBi");
#endif
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf( UnitAreaRelative(*pu.cu, pu ));
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq( otherBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs()
                              ,getBcwWeight( pu.cu->bcwIdx, eRefPicList )
                              );
    pBuf = &origBufTmp;

    fWeight = xGetMEDistortionWeight( pu.cu->bcwIdx, eRefPicList );
  }
#if MULTI_HYP_PRED
  else if (weight)
  {
    CHECK(bBi, "Multi Hyp: bBi");
    fWeight = fabs(double(weight) / double(1 << MULTI_HYP_PRED_WEIGHT_BITS));
  }
#endif
  m_cDistParam.isBiPred = bBi;
#if INTER_LIC
  m_cDistParam.useMR = pu.cu->licFlag;
#endif

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf* pcPatternKey = &tmpPattern;

  m_lumaClpRng = pu.cs->slice->clpRng( COMPONENT_Y );

  bool wrap =  pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred)->isWrapAroundEnabled( pu.cs->pps );
  CPelBuf buf = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred)->getRecoBuf(pu.blocks[COMPONENT_Y], wrap);

  IntTZSearchStruct cStruct;
  cStruct.pcPatternKey  = pcPatternKey;
  cStruct.iRefStride    = buf.stride;
  cStruct.piRefY        = buf.buf;
  cStruct.imvShift = pu.cu->imv == IMV_HPEL ? 1 : (pu.cu->imv << 1);
  cStruct.useAltHpelIf = pu.cu->imv == IMV_HPEL;
  cStruct.inCtuSearch = false;
  cStruct.zeroMV = false;
  {
    if (m_useCompositeRef && pu.cs->slice->getRefPic(eRefPicList, iRefIdxPred)->longTerm)
    {
      cStruct.inCtuSearch = true;
    }
  }

  auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl );

  bool bQTBTMV  = false;
  bool bQTBTMV2 = false;
  Mv cIntMv;
#if MULTI_HYP_PRED 
  if (!bBi && !weight)
#else
  if (!bBi)
#endif
  {
    bool bValid = blkCache && blkCache->getMv( pu, eRefPicList, iRefIdxPred, cIntMv );
    if( bValid )
    {
      bQTBTMV2 = true;
      cIntMv.changePrecision( MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    }
  }

  Mv predQuarter = rcMvPred;
  predQuarter.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  m_pcRdCost->setPredictor( predQuarter );

  m_pcRdCost->setCostScale(2);

#if INTER_LIC
  if (pu.cu->licFlag)
  {
    m_cDistParam.applyWeight = false;
  }
  else
#endif
  {
    setWpScalingDistParam(iRefIdxPred, eRefPicList, pu.cu->slice);
  }
  m_currRefPicList = eRefPicList;
  m_currRefPicIndex = iRefIdxPred;
  m_skipFracME = false;
  //  Do integer search
  if( ( m_motionEstimationSearchMethod == MESEARCH_FULL ) || bBi || bQTBTMV )
  {
    cStruct.subShiftMode = m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ? 2 : 0;
    m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode);

    Mv bestInitMv = (bBi ? rcMv : rcMvPred);
    Mv cTmpMv = bestInitMv;

    clipMv( cTmpMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;
    Distortion uiBestSad = m_cDistParam.distFunc(m_cDistParam);
    uiBestSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
    {
      cTmpMv = rcMvPred;
      clipMv( cTmpMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
      cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;
      Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
      if (uiSad < uiBestSad)
      {
        uiBestSad = uiSad;
        bestInitMv = rcMvPred;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
#endif

#if AMVR_ENC_OPT
    const MvPrecision tmpIntMvPrec = (pu.cu->imv == IMV_4PEL ? MV_PRECISION_4PEL : MV_PRECISION_INT);
#endif
    for (int i = 0; i < m_uniMvListSize; i++)
    {
      BlkUniMvInfo* curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
      Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
      tmpCurMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
#endif

      int j = 0;
      for (; j < i; j++)
      {
        BlkUniMvInfo *prevMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
        Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][iRefIdxPred];
        tmpPrevMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);

        if (tmpCurMv == tmpPrevMv)
#else
        if (curMvInfo->uniMvs[eRefPicList][iRefIdxPred] == prevMvInfo->uniMvs[eRefPicList][iRefIdxPred])
#endif
        {
          break;
        }
      }
      if (j < i)
        continue;

      cTmpMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
      clipMv( cTmpMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if AMVR_ENC_OPT
      cTmpMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
      if (tmpIntMvPrec != MV_PRECISION_INT)
      {
        cTmpMv.changePrecision(tmpIntMvPrec, MV_PRECISION_INT);
      }
#else
      cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
      m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

      Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
      if (uiSad < uiBestSad)
      {
        uiBestSad = uiSad;
        bestInitMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }

    if( !bQTBTMV )
    {
      xSetSearchRange(pu, bestInitMv, iSrchRng, cStruct.searchRange, cStruct);
    }
    xPatternSearch( cStruct, rcMv, ruiCost);
  }
  else if( bQTBTMV2 )
  {
    rcMv = cIntMv;

    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
    xTZSearch(pu, eRefPicList, iRefIdxPred, cStruct, rcMv, ruiCost, NULL, false, true);
  }
  else
  {
    cStruct.subShiftMode = ( !m_pcEncCfg->getRestrictMESampling() && m_pcEncCfg->getMotionEstimationSearchMethod() == MESEARCH_SELECTIVE ) ? 1 :
                            ( m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode() == FASTINTERSEARCH_MODE3 ) ? 2 : 0;
#if MULTI_HYP_PRED
    if (weight == 0)
#endif
    rcMv = rcMvPred;
    const Mv *pIntegerMv2Nx2NPred = 0;
#if MULTI_HYP_PRED
    const auto savedMEMethod = m_motionEstimationSearchMethod;
    if( weight )
    {
      m_motionEstimationSearchMethod = MESEARCH_DIAMOND_ENHANCED;
    }
#endif
    xPatternSearchFast(pu, eRefPicList, iRefIdxPred, cStruct, rcMv, ruiCost, pIntegerMv2Nx2NPred);
#if MULTI_HYP_PRED
    if( weight )
    {
      m_motionEstimationSearchMethod = savedMEMethod;
    }
    else
    {
#endif
    if( blkCache )
    {
      blkCache->setMv( pu.cs->area, eRefPicList, iRefIdxPred, rcMv );
    }
    else
    {
      m_integerMv2Nx2N[eRefPicList][iRefIdxPred] = rcMv;
    }
#if MULTI_HYP_PRED
    }
#endif
  }
  DTRACE( g_trace_ctx, D_ME, "%d %d %d :MECostFPel<L%d,%d>: %d,%d,%dx%d, %d", DTRACE_GET_COUNTER( g_trace_ctx, D_ME ), pu.cu->slice->getPOC(), 0, ( int ) eRefPicList, ( int ) bBi, pu.Y().x, pu.Y().y, pu.Y().width, pu.Y().height, ruiCost );
  // sub-pel refinement for sub-pel resolution
  if ( pu.cu->imv == 0 || pu.cu->imv == IMV_HPEL )
  {
    if( m_pcEncCfg->getMCTSEncConstraint() )
    {
      Area curTileAreaSubPelRestricted = pu.cs->picture->mctsInfo.getTileAreaSubPelRestricted( pu );
      // Area adjustment, because subpel refinement is going to (x-1;y-1) direction
      curTileAreaSubPelRestricted.x += 1;
      curTileAreaSubPelRestricted.y += 1;
      curTileAreaSubPelRestricted.width -= 1;
      curTileAreaSubPelRestricted.height -= 1;
      if( ! MCTSHelper::checkMvIsNotInRestrictedArea( pu, rcMv, curTileAreaSubPelRestricted, MV_PRECISION_INT ) )
      {
        MCTSHelper::clipMvToArea( rcMv, pu.Y(), curTileAreaSubPelRestricted, *pu.cs->sps, 0 );
      }
    }
    xPatternSearchFracDIF( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, cMvHalf, cMvQter, ruiCost);
    m_pcRdCost->setCostScale( 0 );
    rcMv <<= 2;
    rcMv  += ( cMvHalf <<= 1 );
    rcMv  += cMvQter;
    uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( rcMv.getHor(), rcMv.getVer(), cStruct.imvShift );
    ruiBits += uiMvBits;
    ruiCost = ( Distortion ) ( floor( fWeight * ( ( double ) ruiCost - ( double ) m_pcRdCost->getCost( uiMvBits ) ) ) + ( double ) m_pcRdCost->getCost( ruiBits ) );
    rcMv.changePrecision(MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL);
  }
  else // integer refinement for integer-pel and 4-pel resolution
  {
    rcMv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    xPatternSearchIntRefine( pu, cStruct, rcMv, rcMvPred, riMVPIdx, ruiBits, ruiCost, amvpInfo, fWeight);
  }

#if INTER_LIC
  if (pu.cu->licFlag)
  {
#if JVET_AD0213_LIC_IMP
    PelUnitBuf predTempBuf = m_tmpAffiStorage.getBuf(UnitAreaRelative(*pu.cu, pu));
#else
    PelUnitBuf predTempBuf = m_tmpStorageLCU.getBuf(UnitAreaRelative(*pu.cu, pu));
#endif
    const Picture* picRef = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred);
#if JVET_AA0096_MC_BOUNDARY_PADDING
    Mv rcMvClipped(rcMv);
    clipMv(rcMvClipped, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps);
    xPredInterBlk(COMPONENT_Y, pu, picRef, rcMvClipped, predTempBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), false,
                  false);
#else
    xPredInterBlk(COMPONENT_Y, pu, picRef, rcMv, predTempBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), false, false);
#endif

    DistParam distParam;
#if JVET_AD0213_LIC_IMP
    m_pcRdCost->setDistParam(distParam, pBuf->Y(), predTempBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, !pu.cs->slice->getDisableSATDForRD());
    ruiCost = (Distortion)floor(fWeight * (double)distParam.distFunc(distParam)) + m_pcRdCost->getCost(ruiBits);
#else
    m_pcRdCost->setDistParam(distParam, origBuf.Y(), predTempBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, !pu.cs->slice->getDisableSATDForRD());
#if MULTI_HYP_PRED
    ruiCost = (Distortion)floor(fWeight * (double)distParam.distFunc(distParam)) + m_pcRdCost->getCost(ruiBits);
#else
    ruiCost = distParam.distFunc(distParam) + m_pcRdCost->getCost(ruiBits);
#endif
#endif
  }
#endif
  DTRACE(g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", (int)eRefPicList, (int)bBi, ruiCost, ruiBits, rcMv.getHor() << 2, rcMv.getVer() << 2);
}

#if JVET_AG0098_AMVP_WITH_SBTMVP
void InterSearch::xAmvpSbTmvpMotionEstimation(PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eRefPicList, Mv& rcMvPred, int iRefIdxPred, Mv& rcMv, int& amvpSbTmvpIdx, bool useAmvpSbTmvpBuf, MergeCtx& mrgCtx, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const Distortion normalCost)
{
  ruiCost = std::numeric_limits<Distortion>::max();
  Mv predMv = rcMvPred;
  predMv.changeTransPrecInternal2Amvr(pu.cu->imv);
  m_pcRdCost->setPredictor(predMv);
  m_pcRdCost->setCostScale(0);
  rcMv = rcMvPred;

  PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
  DistParam distParam;
  m_pcRdCost->setDistParam(distParam, origBuf.Y(), predBuf.Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, !pu.cs->slice->getDisableSATDForRD());

  // calculate cost for mvd zero 
  amvpSbTmvpIdx = -1;
  Mv mvPred = rcMv;
  Mv mvCand = rcMv;

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  PU::getAmvpSbTmvp(this, pu, mrgCtx, mvCand, useAmvpSbTmvpBuf, m_amvpSbTmvpBufTLPos, m_amvpSbTmvpBufValid, m_amvpSbTmvpMotionBuf);
#else
  PU::getAmvpSbTmvp(pu, mrgCtx, mvCand, useAmvpSbTmvpBuf, m_amvpSbTmvpBufTLPos, m_amvpSbTmvpBufValid, m_amvpSbTmvpMotionBuf);
#endif
  if (!useAmvpSbTmvpBuf)
  {
    PU::spanMotionInfo(pu, mrgCtx);
  }

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
#endif
  xSubPuMC(pu, predBuf, REF_PIC_LIST_X, true, false);

  // get MVD cost
  ruiCost = m_pcRdCost->getAmvpSbTmvpCostOfVectorWithPredictor(-1, pu.cs->slice->getAmvpSbTmvpNumOffset());
#if MULTI_HYP_PRED
  ruiCost += (Distortion)floor((double)distParam.distFunc(distParam));
#else
  ruiCost += distParam.distFunc(distParam);
#endif
  int mvPrec = MV_PRECISION_INTERNAL - MV_PRECISION_INT;

  int mvdDirOffset = 0, step = 0, tmpDir = 0;
  Distortion uiCost, tmpSatd, idxCost;
  Mv tmpMv;
  for (int mvdStep = 0; mvdStep < pu.cs->slice->getAmvpSbTmvpNumOffset(); mvdStep++)
  {
    if (pu.cu->imv)
    {
      mvdDirOffset = ((mvdStep & 1) == 1) ? 0 : AMVP_SBTMVP_NUM_DIR;
      step = g_amvpSbTmvp_mvd_offset[mvdStep + AMVP_SBTMVP_NUM_OFFSET] << mvPrec;
    }
    else
    {
      mvdDirOffset = ((mvdStep & 1) == 0) ? 0 : AMVP_SBTMVP_NUM_DIR;
      step = g_amvpSbTmvp_mvd_offset[mvdStep] << mvPrec;
    }
    tmpSatd = std::numeric_limits<Distortion>::max();
    idxCost = m_pcRdCost->getAmvpSbTmvpCostOfVectorWithPredictor(mvdStep, pu.cs->slice->getAmvpSbTmvpNumOffset());
    if (idxCost >= ruiCost || idxCost >= normalCost)
    {
      break;
    }
    for (int mvdDir = 0; mvdDir < AMVP_SBTMVP_NUM_DIR; mvdDir++)
    {
      mvCand.set(g_amvpSbTmvp_mvd_dir[0][mvdDir + mvdDirOffset] * step, g_amvpSbTmvp_mvd_dir[1][mvdDir + mvdDirOffset] * step);
      mvCand += mvPred;

      if (m_pcEncCfg->getMCTSEncConstraint())
      {
        if (!(MCTSHelper::checkMvForMCTSConstraint(pu, mvCand)))
          continue; // Skip this this pos
      }

#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
      PU::getAmvpSbTmvp(this, pu, mrgCtx, mvCand, useAmvpSbTmvpBuf, m_amvpSbTmvpBufTLPos, m_amvpSbTmvpBufValid, m_amvpSbTmvpMotionBuf);
#else
      PU::getAmvpSbTmvp(pu, mrgCtx, mvCand, useAmvpSbTmvpBuf, m_amvpSbTmvpBufTLPos, m_amvpSbTmvpBufValid, m_amvpSbTmvpMotionBuf);
#endif
      if (!useAmvpSbTmvpBuf)
      {
        PU::spanMotionInfo(pu, mrgCtx);
      }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
#endif
      xSubPuMC(pu, predBuf, REF_PIC_LIST_X, true, false);

#if MULTI_HYP_PRED
      uiCost = (Distortion)floor((double)distParam.distFunc(distParam));
#else
      uiCost = distParam.distFunc(distParam);
#endif
      if (uiCost < tmpSatd)
      {
        tmpSatd = uiCost;
        tmpMv = mvCand;
        tmpDir = mvdDir;
      }
    }
    // get MVD cost
    uiCost = idxCost + tmpSatd;
    if (uiCost < ruiCost)
    {
      ruiCost = uiCost;
      rcMv = tmpMv;
      amvpSbTmvpIdx = mvdStep * AMVP_SBTMVP_NUM_DIR + tmpDir;
    }
  } 

  uint32_t uiMvBits = m_pcRdCost->getAmvpSbTmvpBitsOfVectorWithPredictor(amvpSbTmvpIdx < 0 ? -1 : (amvpSbTmvpIdx >> 2), pu.cs->slice->getAmvpSbTmvpNumOffset());
  ruiBits += uiMvBits;
  ruiCost = (Distortion)(floor(1.0 * ((double)ruiCost - (double)m_pcRdCost->getCost(uiMvBits))) + (double)m_pcRdCost->getCost(ruiBits));
}
#endif

void InterSearch::xSetSearchRange ( const PredictionUnit& pu,
                                    const Mv& cMvPred,
                                    const int iSrchRng,
                                    SearchRange& sr
                                  , IntTZSearchStruct& cStruct
)
{
  const int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  Mv cFPMvPred = cMvPred;
  clipMv( cFPMvPred, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
  
  Mv mvTL(cFPMvPred.getHor() - (iSrchRng << iMvShift), cFPMvPred.getVer() - (iSrchRng << iMvShift));
  Mv mvBR(cFPMvPred.getHor() + (iSrchRng << iMvShift), cFPMvPred.getVer() + (iSrchRng << iMvShift));

  if (m_pcEncCfg->getMCTSEncConstraint())
  {
    MCTSHelper::clipMvToArea( mvTL, pu.Y(), pu.cs->picture->mctsInfo.getTileArea(), *pu.cs->sps );
    MCTSHelper::clipMvToArea( mvBR, pu.Y(), pu.cs->picture->mctsInfo.getTileArea(), *pu.cs->sps );
  }
  else
  {
    xClipMv( mvTL, pu.cu->lumaPos(),
            pu.cu->lumaSize(),
            *pu.cs->sps
          , *pu.cs->pps
    );
    xClipMv( mvBR, pu.cu->lumaPos(),
            pu.cu->lumaSize(),
            *pu.cs->sps
          , *pu.cs->pps
    );
  }

  mvTL.divideByPowerOf2( iMvShift );
  mvBR.divideByPowerOf2( iMvShift );

  sr.left   = mvTL.hor;
  sr.top    = mvTL.ver;
  sr.right  = mvBR.hor;
  sr.bottom = mvBR.ver;

  if (m_useCompositeRef && cStruct.inCtuSearch)
  {
    Position posRB = pu.Y().bottomRight();
    Position posTL = pu.Y().topLeft();
    const PreCalcValues *pcv = pu.cs->pcv;
    Position posRBinCTU(posRB.x & pcv->maxCUWidthMask, posRB.y & pcv->maxCUHeightMask);
    Position posLTinCTU = Position(posTL.x & pcv->maxCUWidthMask, posTL.y & pcv->maxCUHeightMask).offset(-4, -4);
    if (sr.left < -posLTinCTU.x)
      sr.left = -posLTinCTU.x;
    if (sr.top < -posLTinCTU.y)
      sr.top = -posLTinCTU.y;
    if (sr.right >((int)pcv->maxCUWidth - 4 - posRBinCTU.x))
      sr.right = (int)pcv->maxCUWidth - 4 - posRBinCTU.x;
    if (sr.bottom >((int)pcv->maxCUHeight - 4 - posRBinCTU.y))
      sr.bottom = (int)pcv->maxCUHeight - 4 - posRBinCTU.y;
    if (posLTinCTU.x == -4 || posLTinCTU.y == -4)
    {
      sr.left = sr.right = sr.bottom = sr.top = 0;
      cStruct.zeroMV = 1;
    }
    if (posRBinCTU.x == pcv->maxCUWidthMask || posRBinCTU.y == pcv->maxCUHeightMask)
    {
      sr.left = sr.right = sr.bottom = sr.top = 0;
      cStruct.zeroMV = 1;
    }
  }
}


void InterSearch::xPatternSearch( IntTZSearchStruct&    cStruct,
                                  Mv&            rcMv,
                                  Distortion&    ruiSAD )
{
  Distortion  uiSad;
  Distortion  uiSadBest = std::numeric_limits<Distortion>::max();
  int         iBestX = 0;
  int         iBestY = 0;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  const SearchRange& sr = cStruct.searchRange;

  const Pel* piRef = cStruct.piRefY + (sr.top * cStruct.iRefStride);
  for ( int y = sr.top; y <= sr.bottom; y++ )
  {
    for ( int x = sr.left; x <= sr.right; x++ )
    {
      //  find min. distortion position
      m_cDistParam.cur.buf = piRef + x;

      uiSad = m_cDistParam.distFunc( m_cDistParam );

      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor( x, y, cStruct.imvShift );

      if ( uiSad < uiSadBest )
      {
        uiSadBest = uiSad;
        iBestX    = x;
        iBestY    = y;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
    piRef += cStruct.iRefStride;
  }
  rcMv.set( iBestX, iBestY );

  cStruct.uiBestSad = uiSadBest; // th for testing
  ruiSAD = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor( iBestX, iBestY, cStruct.imvShift );
  return;
}


void InterSearch::xPatternSearchFast( const PredictionUnit& pu,
                                      RefPicList            eRefPicList,
                                      int                   iRefIdxPred,
                                      IntTZSearchStruct&    cStruct,
                                      Mv&                   rcMv,
                                      Distortion&           ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  switch ( m_motionEstimationSearchMethod )
  {
  case MESEARCH_DIAMOND:
    xTZSearch         ( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false );
    break;

  case MESEARCH_SELECTIVE:
    xTZSearchSelective( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred );
    break;

  case MESEARCH_DIAMOND_ENHANCED:
    xTZSearch         ( pu, eRefPicList, iRefIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true );
    break;

  case MESEARCH_FULL: // shouldn't get here.
  default:
    break;
  }
}


void InterSearch::xTZSearch( const PredictionUnit& pu,
                             RefPicList            eRefPicList,
                             int                   iRefIdxPred,
                             IntTZSearchStruct&    cStruct,
                             Mv&                   rcMv,
                             Distortion&           ruiSAD,
                             const Mv* const       pIntegerMv2Nx2NPred,
                             const bool            bExtendedSettings,
                             const bool            bFastSettings)
{
  const bool bUseRasterInFastMode                    = true; //toggle this to further reduce runtime

  const bool bUseAdaptiveRaster                      = bExtendedSettings;
  const int  iRaster                                 = (bFastSettings && bUseRasterInFastMode) ? 8 : 5;
  const bool bTestZeroVector                         = true && !bFastSettings;
  const bool bTestZeroVectorStart                    = bExtendedSettings;
  const bool bTestZeroVectorStop                     = false;
  const bool bFirstSearchDiamond                     = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bFirstCornersForDiamondDist1            = bExtendedSettings;
  const bool bFirstSearchStop                        = m_pcEncCfg->getFastMEAssumingSmootherMVEnabled();
  const uint32_t uiFirstSearchRounds                     = bFastSettings ? (bUseRasterInFastMode?3:2) : 3;     // first search stop X rounds after best match (must be >=1)
  const bool bEnableRasterSearch                     = bFastSettings ? bUseRasterInFastMode : true;
  const bool bAlwaysRasterSearch                     = bExtendedSettings;  // true: BETTER but factor 2 slower
  const bool bRasterRefinementEnable                 = false; // enable either raster refinement or star refinement
  const bool bRasterRefinementDiamond                = false; // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bRasterRefinementCornersForDiamondDist1 = bExtendedSettings;
  const bool bStarRefinementEnable                   = true;  // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond                  = true;  // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementCornersForDiamondDist1   = bExtendedSettings;
  const bool bStarRefinementStop                     = false || bFastSettings;
  const uint32_t uiStarRefinementRounds                  = 2;  // star refinement stop X rounds after best match (must be >=1)
  const bool bNewZeroNeighbourhoodTest               = bExtendedSettings;

  int iSearchRange = m_iSearchRange;
  if( m_pcEncCfg->getMCTSEncConstraint() )
  {
    MCTSHelper::clipMvToArea( rcMv, pu.Y(), pu.cs->picture->mctsInfo.getTileArea(), *pu.cs->sps );
  }
  else
  {
    clipMv( rcMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
  }
  rcMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();

  //
  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );

  // distortion


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    if ((rcMv.getHor() != 0 || rcMv.getVer() != 0) &&
      (0 != cStruct.iBestX || 0 != cStruct.iBestY))
    {
      // only test 0-vector if not obviously previously tested.
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
    }
  }

  SearchRange& sr = cStruct.searchRange;

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    if( m_pcEncCfg->getMCTSEncConstraint() )
    {
      MCTSHelper::clipMvToArea( integerMv2Nx2NPred, pu.Y(), pu.cs->picture->mctsInfo.getTileArea(), *pu.cs->sps );
    }
    else
    {
      clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    }
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    integerMv2Nx2NPred.divideByPowerOf2(2);

    if ((rcMv != integerMv2Nx2NPred) &&
      (integerMv2Nx2NPred.getHor() != cStruct.iBestX || integerMv2Nx2NPred.getVer() != cStruct.iBestY))
    {
      // only test integerMv2Nx2NPred if not obviously previously tested.
      xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);
    }
  }

#if AMVR_ENC_OPT
  const MvPrecision tmpIntMvPrec = (pu.cu->imv == IMV_4PEL ? MV_PRECISION_4PEL : MV_PRECISION_INT);
#endif
  for (int i = 0; i < m_uniMvListSize; i++)
  {
    BlkUniMvInfo* curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
    Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
    tmpCurMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
#endif

    int j = 0;
    for (; j < i; j++)
    {
      BlkUniMvInfo *prevMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
      Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][iRefIdxPred];
      tmpPrevMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);

      if (tmpCurMv == tmpPrevMv)
#else
      if (curMvInfo->uniMvs[eRefPicList][iRefIdxPred] == prevMvInfo->uniMvs[eRefPicList][iRefIdxPred])
#endif
      {
        break;
      }
    }
    if (j < i)
      continue;

    Mv cTmpMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
    clipMv( cTmpMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if AMVR_ENC_OPT
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
    if (tmpIntMvPrec != MV_PRECISION_INT)
    {
      cTmpMv.changePrecision(tmpIntMvPrec, MV_PRECISION_INT);
    }
#else
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad = uiSad;
      cStruct.iBestX = cTmpMv.hor;
      cStruct.iBestY = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= MV_FRACTIONAL_BITS_INTERNAL;
    xSetSearchRange(pu, currBestMv, m_iSearchRange >> (bFastSettings ? 1 : 0), sr, cStruct);
  }
  if (m_pcEncCfg->getUseHashME() && (m_currRefPicList == 0 || pu.cu->slice->getList1IdxToList0Idx(m_currRefPicIndex) < 0))
  {
    int minSize = min(pu.cu->lumaSize().width, pu.cu->lumaSize().height);
    if (minSize < 128 && minSize >= 4)
    {
      int numberOfOtherMvps = m_numHashMVStoreds[m_currRefPicList][m_currRefPicIndex];
      for (int i = 0; i < numberOfOtherMvps; i++)
      {
        xTZSearchHelp(cStruct, m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getHor(), m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getVer(), 0, 0);
      }
      if (numberOfOtherMvps > 0)
      {
        // write out best match
        rcMv.set(cStruct.iBestX, cStruct.iBestY);
        ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
        m_skipFracME = true;
        return;
      }
    }
  }

  // start search
  int  iDist = 0;
  int  iStartX = cStruct.iBestX;
  int  iStartY = cStruct.iBestY;

  const bool bBestCandidateZero = (cStruct.iBestX == 0) && (cStruct.iBestY == 0);

  // first search around best position up to now.
  // The following works as a "subsampled/log" window search around the best candidate
  for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
  {
    if ( bFirstSearchDiamond == 1 )
    {
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bFirstCornersForDiamondDist1 );
    }
    else
    {
      xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
    }

    if ( bFirstSearchStop && ( cStruct.uiBestRound >= uiFirstSearchRounds ) ) // stop criterion
    {
      break;
    }
  }

  if (!bNewZeroNeighbourhoodTest)
  {
    // test whether zero Mv is a better start point than Median predictor
    if ( bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)) )
    {
      xTZSearchHelp( cStruct, 0, 0, 0, 0 );
      if ( (cStruct.iBestX == 0) && (cStruct.iBestY == 0) )
      {
        // test its neighborhood
        for ( iDist = 1; iDist <= iSearchRange; iDist*=2 )
        {
          xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
          if ( bTestZeroVectorStop && (cStruct.uiBestRound > 0) ) // stop criterion
          {
            break;
          }
        }
      }
    }
  }
  else
  {
    // Test also zero neighbourhood but with half the range
    // It was reported that the original (above) search scheme using bTestZeroVectorStart did not
    // make sense since one would have already checked the zero candidate earlier
    // and thus the conditions for that test would have not been satisfied
    if (bTestZeroVectorStart == true && bBestCandidateZero != true)
    {
      for ( iDist = 1; iDist <= (iSearchRange >> 1); iDist*=2 )
      {
        xTZ8PointDiamondSearch( cStruct, 0, 0, iDist, false );
        if ( bTestZeroVectorStop && (cStruct.uiBestRound > 2) ) // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if ( cStruct.uiBestDistance == 1 )
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch( cStruct );
  }

  // raster search if distance is too big
  if (bUseAdaptiveRaster)
  {
    int iWindowSize     = iRaster;
    SearchRange localsr = sr;

    if (!(bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster))))
    {
      iWindowSize ++;
      localsr.left   /= 2;
      localsr.right  /= 2;
      localsr.top    /= 2;
      localsr.bottom /= 2;
    }
    cStruct.uiBestDistance = iWindowSize;
    for ( iStartY = localsr.top; iStartY <= localsr.bottom; iStartY += iWindowSize )
    {
      for ( iStartX = localsr.left; iStartX <= localsr.right; iStartX += iWindowSize )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, iWindowSize );
      }
    }
  }
  else
  {
    if ( bEnableRasterSearch && ( ((int)(cStruct.uiBestDistance) >= iRaster) || bAlwaysRasterSearch ) )
    {
      cStruct.uiBestDistance = iRaster;
      for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += iRaster )
      {
        for ( iStartX = sr.left; iStartX <= sr.right; iStartX += iRaster )
        {
          xTZSearchHelp( cStruct, iStartX, iStartY, 0, iRaster );
        }
      }
    }
  }

  // raster refinement

  if ( bRasterRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if ( cStruct.uiBestDistance > 1 )
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if ( bRasterRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bRasterRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // star refinement
  if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, bStarRefinementCornersForDiamondDist1 );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
}


void InterSearch::xTZSearchSelective( const PredictionUnit& pu,
                                      RefPicList            eRefPicList,
                                      int                   iRefIdxPred,
                                      IntTZSearchStruct&    cStruct,
                                      Mv                    &rcMv,
                                      Distortion            &ruiSAD,
                                      const Mv* const       pIntegerMv2Nx2NPred )
{
  const bool bTestZeroVector          = true;
  const bool bEnableRasterSearch      = true;
  const bool bAlwaysRasterSearch      = false;  // 1: BETTER but factor 15x slower
  const bool bStarRefinementEnable    = true;   // enable either star refinement or raster refinement
  const bool bStarRefinementDiamond   = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool bStarRefinementStop      = false;
  const uint32_t uiStarRefinementRounds   = 2;  // star refinement stop X rounds after best match (must be >=1)
  const int  iSearchRange             = m_iSearchRange;
  const int  iSearchRangeInitial      = m_iSearchRange >> 2;
  const int  uiSearchStep             = 4;
  const int  iMVDistThresh            = 8;

  int   iStartX                 = 0;
  int   iStartY                 = 0;
  int   iDist                   = 0;

  clipMv( rcMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
  rcMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
  rcMv.divideByPowerOf2(2);

  // init TZSearchStruct
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();
  cStruct.iBestX = 0;
  cStruct.iBestY = 0;

  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam( m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, cStruct.subShiftMode );


  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp( cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0 );

  // test whether zero Mv is better start point than Median predictor
  if ( bTestZeroVector )
  {
    xTZSearchHelp( cStruct, 0, 0, 0, 0 );
  }

  SearchRange& sr = cStruct.searchRange;

  if ( pIntegerMv2Nx2NPred != 0 )
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    clipMv( integerMv2Nx2NPred, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    integerMv2Nx2NPred.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    integerMv2Nx2NPred.divideByPowerOf2(2);

    xTZSearchHelp( cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);

  }

#if AMVR_ENC_OPT
  const MvPrecision tmpIntMvPrec = (pu.cu->imv == IMV_4PEL ? MV_PRECISION_4PEL : MV_PRECISION_INT);
#endif
  for (int i = 0; i < m_uniMvListSize; i++)
  {
    BlkUniMvInfo* curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
    Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
    tmpCurMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
#endif

    int j = 0;
    for (; j < i; j++)
    {
      BlkUniMvInfo *prevMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
#if AMVR_ENC_OPT
      Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][iRefIdxPred];
      tmpPrevMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);

      if (tmpCurMv == tmpPrevMv)
#else
      if (curMvInfo->uniMvs[eRefPicList][iRefIdxPred] == prevMvInfo->uniMvs[eRefPicList][iRefIdxPred])
#endif
      {
        break;
      }
    }
    if (j < i)
      continue;

    Mv cTmpMv = curMvInfo->uniMvs[eRefPicList][iRefIdxPred];
    clipMv( cTmpMv, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if AMVR_ENC_OPT
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, tmpIntMvPrec);
    if (tmpIntMvPrec != MV_PRECISION_INT)
    {
      cTmpMv.changePrecision(tmpIntMvPrec, MV_PRECISION_INT);
    }
#else
    cTmpMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad = uiSad;
      cStruct.iBestX = cTmpMv.hor;
      cStruct.iBestY = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY );
    currBestMv <<= 2;
    xSetSearchRange( pu, currBestMv, m_iSearchRange, sr, cStruct );
  }
  if (m_pcEncCfg->getUseHashME() && (m_currRefPicList == 0 || pu.cu->slice->getList1IdxToList0Idx(m_currRefPicIndex) < 0))
  {
    int minSize = min(pu.cu->lumaSize().width, pu.cu->lumaSize().height);
    if (minSize < 128 && minSize >= 4)
    {
      int numberOfOtherMvps = m_numHashMVStoreds[m_currRefPicList][m_currRefPicIndex];
      for (int i = 0; i < numberOfOtherMvps; i++)
      {
        xTZSearchHelp(cStruct, m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getHor(), m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getVer(), 0, 0);
      }
      if (numberOfOtherMvps > 0)
      {
        // write out best match
        rcMv.set(cStruct.iBestX, cStruct.iBestY);
        ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
        m_skipFracME = true;
        return;
      }
    }
  }

  // Initial search
  int iBestX = cStruct.iBestX;
  int iBestY = cStruct.iBestY;
  int iFirstSrchRngHorLeft    = ((iBestX - iSearchRangeInitial) > sr.left)   ? (iBestX - iSearchRangeInitial) : sr.left;
  int iFirstSrchRngVerTop     = ((iBestY - iSearchRangeInitial) > sr.top)    ? (iBestY - iSearchRangeInitial) : sr.top;
  int iFirstSrchRngHorRight   = ((iBestX + iSearchRangeInitial) < sr.right)  ? (iBestX + iSearchRangeInitial) : sr.right;
  int iFirstSrchRngVerBottom  = ((iBestY + iSearchRangeInitial) < sr.bottom) ? (iBestY + iSearchRangeInitial) : sr.bottom;

  for ( iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep )
  {
    for ( iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep )
    {
      xTZSearchHelp( cStruct, iStartX, iStartY, 0, 0 );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 1, false );
      xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, 2, false );
    }
  }

  int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  //full search with early exit if MV is distant from predictors
  if ( bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch) )
  {
    for ( iStartY = sr.top; iStartY <= sr.bottom; iStartY += 1 )
    {
      for ( iStartX = sr.left; iStartX <= sr.right; iStartX += 1 )
      {
        xTZSearchHelp( cStruct, iStartX, iStartY, 0, 1 );
      }
    }
  }
  //Smaller MV, refine around predictor
  else if ( bStarRefinementEnable && cStruct.uiBestDistance > 0 )
  {
    // start refinement
    while ( cStruct.uiBestDistance > 0 )
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr = 0;
      for ( iDist = 1; iDist < iSearchRange + 1; iDist*=2 )
      {
        if ( bStarRefinementDiamond == 1 )
        {
          xTZ8PointDiamondSearch ( cStruct, iStartX, iStartY, iDist, false );
        }
        else
        {
          xTZ8PointSquareSearch  ( cStruct, iStartX, iStartY, iDist );
        }
        if ( bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds) ) // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if ( cStruct.uiBestDistance == 1 )
      {
        cStruct.uiBestDistance = 0;
        if ( cStruct.ucPointNr != 0 )
        {
          xTZ2PointSearch( cStruct );
        }
      }
    }
  }

  // write out best match
  rcMv.set( cStruct.iBestX, cStruct.iBestY );
  ruiSAD = cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor( cStruct.iBestX, cStruct.iBestY, cStruct.imvShift );
}

void InterSearch::xPatternSearchIntRefine(PredictionUnit& pu, IntTZSearchStruct&  cStruct, Mv& rcMv, Mv& rcMvPred, int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost, const AMVPInfo& amvpInfo, double fWeight)
{

  CHECK( pu.cu->imv == 0 || pu.cu->imv == IMV_HPEL , "xPatternSearchIntRefine(): Sub-pel MV used.");
  CHECK( amvpInfo.mvCand[riMVPIdx] != rcMvPred, "xPatternSearchIntRefine(): MvPred issue.");

  const SPS &sps = *pu.cs->sps;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && !pu.cs->slice->getDisableSATDForRD());

  // -> set MV scale for cost calculation to QPEL (0)
  m_pcRdCost->setCostScale ( 0 );

  Distortion  uiDist, uiSATD = 0;
  Distortion  uiBestDist  = std::numeric_limits<Distortion>::max();
  // subtract old MVP costs because costs for all newly tested MVPs are added in here
#if TM_AMVP
  ruiBits -= m_auiMVPIdxCost[riMVPIdx][amvpInfo.numCand];
#else
  ruiBits -= m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
#endif

  Mv cBestMv = rcMv;
  Mv cBaseMvd[2];
  int iBestBits = 0;
  int iBestMVPIdx = riMVPIdx;
  Mv testPos[9] = { { 0, 0}, { -1, -1},{ -1, 0},{ -1, 1},{ 0, -1},{ 0, 1},{ 1, -1},{ 1, 0},{ 1, 1} };


  cBaseMvd[0] = (rcMv - amvpInfo.mvCand[0]);
  cBaseMvd[1] = (rcMv - amvpInfo.mvCand[1]);

  CHECK( (cBaseMvd[0].getHor() & 0x03) != 0 || (cBaseMvd[0].getVer() & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 0 Mvd issue.");
  CHECK( (cBaseMvd[1].getHor() & 0x03) != 0 || (cBaseMvd[1].getVer() & 0x03) != 0 , "xPatternSearchIntRefine(): AMVP cand 1 Mvd issue.");

  cBaseMvd[0].roundTransPrecInternal2Amvr(pu.cu->imv);
  cBaseMvd[1].roundTransPrecInternal2Amvr(pu.cu->imv);

  // test best integer position and all 8 neighboring positions
  for (int pos = 0; pos < 9; pos ++)
  {
    Mv cTestMv[2];
    // test both AMVP candidates for each position
    for (int iMVPIdx = 0; iMVPIdx < amvpInfo.numCand; iMVPIdx++)
    {
      cTestMv[iMVPIdx] = testPos[pos];
      cTestMv[iMVPIdx].changeTransPrecAmvr2Internal(pu.cu->imv);
      cTestMv[iMVPIdx] += cBaseMvd[iMVPIdx];
      cTestMv[iMVPIdx] += amvpInfo.mvCand[iMVPIdx];

      // MCTS and IMV
      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        Mv cTestMVRestr = cTestMv[iMVPIdx];
        MCTSHelper::clipMvToArea( cTestMVRestr, pu.cu->Y(), pu.cs->picture->mctsInfo.getTileAreaIntPelRestricted( pu ), *pu.cs->sps );

        if( cTestMVRestr != cTestMv[iMVPIdx] )
        {
          // Skip this IMV pos, cause clipping affects IMV scaling
          continue;
        }
      }
      if ( iMVPIdx == 0 || cTestMv[0] != cTestMv[1])
      {
        Mv cTempMV = cTestMv[iMVPIdx];
        if( !m_pcEncCfg->getMCTSEncConstraint() )
        {
          clipMv( cTempMV, pu.cu->lumaPos(), pu.cu->lumaSize(), sps, *pu.cs->pps );
        }
        m_cDistParam.cur.buf = cStruct.piRefY  + cStruct.iRefStride * (cTempMV.getVer() >>  MV_FRACTIONAL_BITS_INTERNAL) + (cTempMV.getHor() >> MV_FRACTIONAL_BITS_INTERNAL);
        uiDist = uiSATD = (Distortion) (m_cDistParam.distFunc( m_cDistParam ) * fWeight);
      }
      else
      {
        uiDist = uiSATD;
      }

#if TM_AMVP
      int iMvBits = m_auiMVPIdxCost[iMVPIdx][amvpInfo.numCand];
#else
      int iMvBits = m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];
#endif
      Mv pred = amvpInfo.mvCand[iMVPIdx];
      pred.changeTransPrecInternal2Amvr(pu.cu->imv);
      m_pcRdCost->setPredictor( pred );
      Mv mv = cTestMv[iMVPIdx];
      mv.changeTransPrecInternal2Amvr(pu.cu->imv);
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor( mv.getHor(), mv.getVer(), 0 );
      uiDist += m_pcRdCost->getCost(iMvBits);

      if (uiDist < uiBestDist)
      {
        uiBestDist = uiDist;
        cBestMv = cTestMv[iMVPIdx];
        iBestMVPIdx = iMVPIdx;
        iBestBits = iMvBits;
      }
    }
  }
  if( uiBestDist == std::numeric_limits<Distortion>::max() )
  {
    ruiCost = std::numeric_limits<Distortion>::max();
    return;
  }

  rcMv = cBestMv;
  rcMvPred = amvpInfo.mvCand[iBestMVPIdx];
  riMVPIdx = iBestMVPIdx;
  m_pcRdCost->setPredictor( rcMvPred );

  ruiBits += iBestBits;
  // taken from JEM 5.0
  // verify since it makes no sence to subtract Lamda*(Rmvd+Rmvpidx) from D+Lamda(Rmvd)
  // this would take the rate for the MVP idx out of the cost calculation
  // however this rate is always 1 so impact is small
  ruiCost = uiBestDist - m_pcRdCost->getCost(iBestBits) + m_pcRdCost->getCost(ruiBits);
  // taken from JEM 5.0
  // verify since it makes no sense to add rate for MVDs twicce

  return;
}

void InterSearch::xPatternSearchFracDIF(
  const PredictionUnit& pu,
  RefPicList            eRefPicList,
  int                   iRefIdx,
  IntTZSearchStruct&    cStruct,
  const Mv&             rcMvInt,
  Mv&                   rcMvHalf,
  Mv&                   rcMvQter,
  Distortion&           ruiCost
)
{

  //  Reference pattern initialization (integer scale)
  int         iOffset    = rcMvInt.getHor() + rcMvInt.getVer() * cStruct.iRefStride;
  CPelBuf cPatternRoi(cStruct.piRefY + iOffset, cStruct.iRefStride, *cStruct.pcPatternKey);
  if (m_skipFracME)
  {
    Mv baseRefMv(0, 0);
    rcMvHalf.setZero();
    m_pcRdCost->setCostScale(0);
    xExtDIFUpSamplingH(&cPatternRoi, cStruct.useAltHpelIf);
    rcMvQter = rcMvInt;   rcMvQter <<= 2;    // for mv-cost
    ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, !pu.cs->slice->getDisableSATDForRD());
    return;
  }


  if (cStruct.imvShift > IMV_FPEL || (m_useCompositeRef && cStruct.zeroMV))
  {
    m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY + iOffset, cStruct.iRefStride, m_lumaClpRng.bd, COMPONENT_Y, 0, 1, m_pcEncCfg->getUseHADME() && !pu.cs->slice->getDisableSATDForRD());
    ruiCost = m_cDistParam.distFunc( m_cDistParam );
    ruiCost += m_pcRdCost->getCostOfVectorWithPredictor( rcMvInt.getHor(), rcMvInt.getVer(), cStruct.imvShift );
    return;
  }

  //  Half-pel refinement
  m_pcRdCost->setCostScale(1);
  xExtDIFUpSamplingH(&cPatternRoi, cStruct.useAltHpelIf);

  rcMvHalf = rcMvInt;   rcMvHalf <<= 1;    // for mv-cost
  Mv baseRefMv(0, 0);
  ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 2, rcMvHalf, (!pu.cs->slice->getDisableSATDForRD()));

  //  quarter-pel refinement
  if (cStruct.imvShift == IMV_OFF)
  {
  m_pcRdCost->setCostScale( 0 );
  xExtDIFUpSamplingQ ( &cPatternRoi, rcMvHalf);
  baseRefMv = rcMvHalf;
  baseRefMv <<= 1;

  rcMvQter = rcMvInt;    rcMvQter <<= 1;    // for mv-cost
  rcMvQter += rcMvHalf;  rcMvQter <<= 1;
  ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, (!pu.cs->slice->getDisableSATDForRD()));
  }
}

Distortion InterSearch::xGetSymmetricCost( PredictionUnit& pu, PelUnitBuf& origBuf, RefPicList eCurRefPicList, const MvField& cCurMvField, MvField& cTarMvField, int bcwIdx )
{
  Distortion cost = std::numeric_limits<Distortion>::max();
  RefPicList eTarRefPicList = (RefPicList)(1 - (int)eCurRefPicList);

  // get prediction of eCurRefPicList
  PelUnitBuf predBufA = m_tmpPredStorage[eCurRefPicList].getBuf( UnitAreaRelative( *pu.cu, pu ) );
  const Picture* picRefA = pu.cu->slice->getRefPic( eCurRefPicList, cCurMvField.refIdx );
  Mv mvA = cCurMvField.mv;
  clipMv( mvA, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if JVET_AD0213_LIC_IMP
  if ((mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 && !pu.cu->licFlag)
#else
  if ( (mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 )
#endif
  {
    Position offset = pu.blocks[COMPONENT_Y].pos().offset( mvA.getHor() >> 4, mvA.getVer() >> 4 );
    CPelBuf pelBufA = picRefA->getRecoBuf( CompArea( COMPONENT_Y, pu.chromaFormat, offset, pu.blocks[COMPONENT_Y].size() ), false );
    predBufA.bufs[0].buf = const_cast<Pel *>(pelBufA.buf);
    predBufA.bufs[0].stride = pelBufA.stride;
    predBufA.bufs[0].width = pelBufA.width;
    predBufA.bufs[0].height = pelBufA.height;
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pu, picRefA, mvA, predBufA, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false );
  }

  // get prediction of eTarRefPicList
  PelUnitBuf predBufB = m_tmpPredStorage[eTarRefPicList].getBuf( UnitAreaRelative( *pu.cu, pu ) );
  const Picture* picRefB = pu.cu->slice->getRefPic( eTarRefPicList, cTarMvField.refIdx );
  Mv mvB = cTarMvField.mv;
  clipMv( mvB, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if JVET_AD0213_LIC_IMP
  if ((mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 && !pu.cu->licFlag)
#else
  if ( (mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 )
#endif
  {
    Position offset = pu.blocks[COMPONENT_Y].pos().offset( mvB.getHor() >> 4, mvB.getVer() >> 4 );
    CPelBuf pelBufB = picRefB->getRecoBuf( CompArea( COMPONENT_Y, pu.chromaFormat, offset, pu.blocks[COMPONENT_Y].size() ), false );
    predBufB.bufs[0].buf = const_cast<Pel *>(pelBufB.buf);
    predBufB.bufs[0].stride = pelBufB.stride;
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pu, picRefB, mvB, predBufB, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false );
  }

  PelUnitBuf bufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative( *pu.cu, pu ) );
  bufTmp.copyFrom( origBuf );
  bufTmp.removeHighFreq( predBufA, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs(), getBcwWeight( pu.cu->bcwIdx, eTarRefPicList ) );
  double fWeight = xGetMEDistortionWeight( pu.cu->bcwIdx, eTarRefPicList );

  // calc distortion
  DFunc distFunc = (!pu.cu->slice->getDisableSATDForRD()) ? DF_HAD : DF_SAD;
  cost = (Distortion)floor( fWeight * (double)m_pcRdCost->getDistPart( bufTmp.Y(), predBufB.Y(), pu.cs->sps->getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, distFunc ) );
  return(cost);
}

Distortion InterSearch::xSymmeticRefineMvSearch( PredictionUnit &pu, PelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred
  , RefPicList eRefPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion uiMinCost, int SearchPattern, int nSearchStepShift, uint32_t uiMaxSearchRounds, int bcwIdx )
{
  const Mv mvSearchOffsetCross[4] = { Mv( 0 , 1 ) , Mv( 1 , 0 ) , Mv( 0 , -1 ) , Mv( -1 ,  0 ) };
  const Mv mvSearchOffsetSquare[8] = { Mv( -1 , 1 ) , Mv( 0 , 1 ) , Mv( 1 ,  1 ) , Mv( 1 ,  0 ) , Mv( 1 , -1 ) , Mv( 0 , -1 ) , Mv( -1 , -1 ) , Mv( -1 , 0 ) };
  const Mv mvSearchOffsetDiamond[8] = { Mv( 0 , 2 ) , Mv( 1 , 1 ) , Mv( 2 ,  0 ) , Mv( 1 , -1 ) , Mv( 0 , -2 ) , Mv( -1 , -1 ) , Mv( -2 ,  0 ) , Mv( -1 , 1 ) };
  const Mv mvSearchOffsetHexagon[6] = { Mv( 2 , 0 ) , Mv( 1 , 2 ) , Mv( -1 ,  2 ) , Mv( -2 ,  0 ) , Mv( -1 , -2 ) , Mv( 1 , -2 ) };

  int nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const Mv * pSearchOffset;
  if ( SearchPattern == 0 )
  {
    nDirectEnd = 3;
    nDirectRounding = 4;
    nDirectMask = 0x03;
    pSearchOffset = mvSearchOffsetCross;
  }
  else if ( SearchPattern == 1 )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetSquare;
  }
  else if ( SearchPattern == 2 )
  {
    nDirectEnd = 7;
    nDirectRounding = 8;
    nDirectMask = 0x07;
    pSearchOffset = mvSearchOffsetDiamond;
  }
  else if ( SearchPattern == 3 )
  {
    nDirectEnd = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    THROW( "Invalid search pattern" );
  }

  int nBestDirect;
  for ( uint32_t uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++ )
  {
    nBestDirect = -1;
    MvField mvCurCenter = rCurMvField;
    for ( int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++ )
    {
      int nDirect;
      if ( SearchPattern == 3 )
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        nDirect = (nIdx + nDirectRounding) & nDirectMask;
      }

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      MvField mvCand = mvCurCenter, mvPair;
      mvCand.mv += mvOffset;

      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        if( !( MCTSHelper::checkMvForMCTSConstraint( pu, mvCand.mv ) ) )
          continue; // Skip this this pos
      }
      // get MVD cost
      Mv pred = rcMvCurPred;
      pred.changeTransPrecInternal2Amvr(pu.cu->imv);
      m_pcRdCost->setPredictor( pred );
      m_pcRdCost->setCostScale( 0 );
      Mv mv = mvCand.mv;
      mv.changeTransPrecInternal2Amvr(pu.cu->imv);
      uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor( mv.getHor(), mv.getVer(), 0 );
      Distortion uiCost = m_pcRdCost->getCost( uiMvBits );

      // get MVD pair and set target MV
      mvPair.refIdx = rTarMvField.refIdx;
      mvPair.mv.set( rcMvTarPred.hor - (mvCand.mv.hor - rcMvCurPred.hor), rcMvTarPred.ver - (mvCand.mv.ver - rcMvCurPred.ver) );
      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        if( !( MCTSHelper::checkMvForMCTSConstraint( pu, mvPair.mv ) ) )
          continue; // Skip this this pos
      }
      uiCost += xGetSymmetricCost( pu, origBuf, eRefPicList, mvCand, mvPair, bcwIdx );
      if ( uiCost < uiMinCost )
      {
        uiMinCost = uiCost;
        rCurMvField = mvCand;
        rTarMvField = mvPair;
        nBestDirect = nDirect;
      }
    }

    if ( nBestDirect == -1 )
    {
      break;
    }
    int nStep = 1;
    if ( SearchPattern == 1 || SearchPattern == 2 )
    {
      nStep = 2 - (nBestDirect & 0x01);
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd = nBestDirect + nStep;
  }

  return(uiMinCost);
}


void InterSearch::xSymmetricMotionEstimation( PredictionUnit& pu, PelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList eRefPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion& ruiCost, int bcwIdx )
{
  // Refine Search
  int nSearchStepShift = MV_FRACTIONAL_BITS_DIFF;
  int nDiamondRound = 8;
  int nCrossRound = 1;

  nSearchStepShift += pu.cu->imv == IMV_HPEL ? 1 : (pu.cu->imv << 1);
  nDiamondRound >>= pu.cu->imv;

  ruiCost = xSymmeticRefineMvSearch( pu, origBuf, rcMvCurPred, rcMvTarPred, eRefPicList, rCurMvField, rTarMvField, ruiCost, 2, nSearchStepShift, nDiamondRound, bcwIdx );
  ruiCost = xSymmeticRefineMvSearch( pu, origBuf, rcMvCurPred, rcMvTarPred, eRefPicList, rCurMvField, rTarMvField, ruiCost, 0, nSearchStepShift, nCrossRound, bcwIdx );
}

void InterSearch::xPredAffineInterSearch( PredictionUnit&       pu,
                                          PelUnitBuf&           origBuf,
                                          int                   puIdx,
                                          uint32_t&                 lastMode,
                                          Distortion&           affineCost,
                                          Mv                    hevcMv[2][33]
                                        , Mv                    mvAffine4Para[2][33][3]
                                        , int                   refIdx4Para[2]
                                        , uint8_t               bcwIdx
                                        , bool                  enforceBcwPred
                                        , uint32_t              bcwIdxBits
                                         )
{
  const Slice &slice = *pu.cu->slice;

  affineCost = std::numeric_limits<Distortion>::max();

  Mv        cMvZero;
  Mv        aacMv[2][3];
  Mv        cMvBi[2][3];
  Mv        cMvTemp[2][33][3];

  int       iNumPredDir = slice.isInterP() ? 1 : 2;

  int mvNum = 2;
  mvNum = pu.cu->affineType ? 3 : 2;

  // Mvp
  Mv        cMvPred[2][33][3];
  Mv        cMvPredBi[2][33][3];
  int       aaiMvpIdxBi[2][33];
  int       aaiMvpIdx[2][33];
  int       aaiMvpNum[2][33];

  AffineAMVPInfo aacAffineAMVPInfo[2][33];
  AffineAMVPInfo affiAMVPInfoTemp[2];

  int           iRefIdx[2]={0,0}; // If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int           iRefIdxBi[2];

  uint32_t          uiMbBits[3] = {1, 1, 0};

  int           iRefStart, iRefEnd;

  int           bestBiPRefIdxL1 = 0;
  int           bestBiPMvpL1 = 0;
  Distortion biPDistTemp = std::numeric_limits<Distortion>::max();

  Distortion    uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
  Distortion    uiCostBi  = std::numeric_limits<Distortion>::max();
  Distortion    uiCostTemp;

  uint32_t          uiBits[3] = { 0 };
  uint32_t          uiBitsTemp;
  Distortion    bestBiPDist = std::numeric_limits<Distortion>::max();

  Distortion    uiCostTempL0[MAX_NUM_REF];
  for (int iNumRef=0; iNumRef < MAX_NUM_REF; iNumRef++)
  {
    uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
  }
  uint32_t uiBitsTempL0[MAX_NUM_REF];

  Mv            mvValidList1[4];
  int           refIdxValidList1 = 0;
  uint32_t          bitsValidList1 = MAX_UINT;
  Distortion costValidList1 = std::numeric_limits<Distortion>::max();
  Mv            mvHevc[3];
  const bool affineAmvrEnabled = pu.cu->slice->getSPS()->getAffineAmvrEnabledFlag();
  int tryBipred = 0;
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  xGetBlkBits( slice.isInterP(), puIdx, lastMode, uiMbBits);

  pu.cu->affine = true;
  pu.mergeFlag = false;
  pu.regularMergeFlag = false;
  if( bcwIdx != BCW_DEFAULT )
  {
    pu.cu->bcwIdx = bcwIdx;
  }
#if MULTI_HYP_PRED
  const bool saveMeResultsForMHP = pu.cs->sps->getUseInterMultiHyp()
    && pu.cu->imv == 0
    && bcwIdx != BCW_DEFAULT
    && (pu.Y().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE && std::min(pu.Y().width, pu.Y().height) >= MULTI_HYP_PRED_RESTRICT_MIN_WH)
    ;
#endif

  // Uni-directional prediction
  for ( int iRefList = 0; iRefList < iNumPredDir; iRefList++ )
  {
    RefPicList  eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    pu.interDir = ( iRefList ? 2 : 1 );
    for (int iRefIdxTemp = 0; iRefIdxTemp < slice.getNumRefIdx(eRefPicList); iRefIdxTemp++)
    {
      // Get RefIdx bits
      uiBitsTemp = uiMbBits[iRefList];
      if ( slice.getNumRefIdx(eRefPicList) > 1 )
      {
        uiBitsTemp += iRefIdxTemp+1;
        if ( iRefIdxTemp == slice.getNumRefIdx(eRefPicList)-1 )
        {
          uiBitsTemp--;
        }
      }

      // Do Affine AMVP
      xEstimateAffineAMVP( pu, affiAMVPInfoTemp[eRefPicList], origBuf, eRefPicList, iRefIdxTemp, cMvPred[iRefList][iRefIdxTemp], &biPDistTemp );
      if ( affineAmvrEnabled )
      {
        biPDistTemp += m_pcRdCost->getCost( xCalcAffineMVBits( pu, cMvPred[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp] ) );
      }
      aaiMvpIdx[iRefList][iRefIdxTemp] = pu.mvpIdx[eRefPicList];
      aaiMvpNum[iRefList][iRefIdxTemp] = pu.mvpNum[eRefPicList];;
      if ( pu.cu->affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp )
      {
        xCopyAffineAMVPInfo( affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp] );
        continue;
      }

      // set hevc ME result as start search position when it is best than mvp
      for ( int i=0; i<3; i++ )
      {
        mvHevc[i] = hevcMv[iRefList][iRefIdxTemp];
        mvHevc[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      }
      PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

      Distortion uiCandCost = xGetAffineTemplateCost(pu, origBuf, predBuf, mvHevc, aaiMvpIdx[iRefList][iRefIdxTemp],
                                                     AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp);

      if ( affineAmvrEnabled )
      {
        uiCandCost += m_pcRdCost->getCost( xCalcAffineMVBits( pu, mvHevc, cMvPred[iRefList][iRefIdxTemp] ) );
      }

      //check stored affine motion
      bool affine4Para    = pu.cu->affineType == AFFINEMODEL_4PARAM;
      bool savedParaAvail = pu.cu->imv && ( ( m_affineMotion.affine4ParaRefIdx[iRefList] == iRefIdxTemp && affine4Para && m_affineMotion.affine4ParaAvail ) ||
                                            ( m_affineMotion.affine6ParaRefIdx[iRefList] == iRefIdxTemp && !affine4Para && m_affineMotion.affine6ParaAvail ) );

      if ( savedParaAvail )
      {
        Mv mvFour[3];
        for ( int i = 0; i < mvNum; i++ )
        {
          mvFour[i] = affine4Para ? m_affineMotion.acMvAffine4Para[iRefList][i] : m_affineMotion.acMvAffine6Para[iRefList][i];
          mvFour[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
        }

        Distortion candCostInherit = xGetAffineTemplateCost( pu, origBuf, predBuf, mvFour, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp );
        candCostInherit += m_pcRdCost->getCost( xCalcAffineMVBits( pu, mvFour, cMvPred[iRefList][iRefIdxTemp] ) );

        if ( candCostInherit < uiCandCost )
        {
          uiCandCost = candCostInherit;
          memcpy( mvHevc, mvFour, 3 * sizeof( Mv ) );
        }
      }

      if (pu.cu->affineType == AFFINEMODEL_4PARAM && m_affMVListSize
        && (!pu.cu->cs->sps->getUseBcw() || bcwIdx == BCW_DEFAULT)
        )
      {
        int shift = MAX_CU_DEPTH;
        for (int i = 0; i < m_affMVListSize; i++)
        {
          AffineMVInfo *mvInfo = m_affMVList + ((m_affMVListIdx - i - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
          //check;
          int j = 0;
          for (; j < i; j++)
          {
            AffineMVInfo *prevMvInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
            if ((mvInfo->affMVs[iRefList][iRefIdxTemp][0] == prevMvInfo->affMVs[iRefList][iRefIdxTemp][0]) &&
              (mvInfo->affMVs[iRefList][iRefIdxTemp][1] == prevMvInfo->affMVs[iRefList][iRefIdxTemp][1])
              && (mvInfo->x == prevMvInfo->x) && (mvInfo->y == prevMvInfo->y)
              && (mvInfo->w == prevMvInfo->w)
              )
            {
              break;
            }
          }
          if (j < i)
            continue;

          Mv mvTmp[3], *nbMv = mvInfo->affMVs[iRefList][iRefIdxTemp];
          int vx, vy;
          int dMvHorX, dMvHorY, dMvVerX, dMvVerY;
          int mvScaleHor = nbMv[0].getHor() << shift;
          int mvScaleVer = nbMv[0].getVer() << shift;
          Mv dMv = nbMv[1] - nbMv[0];
          dMvHorX = dMv.getHor() << (shift - floorLog2(mvInfo->w));
          dMvHorY = dMv.getVer() << (shift - floorLog2(mvInfo->w));
          dMvVerX = -dMvHorY;
          dMvVerY = dMvHorX;
          vx = mvScaleHor + dMvHorX * (pu.Y().x - mvInfo->x) + dMvVerX * (pu.Y().y - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (pu.Y().x - mvInfo->x) + dMvVerY * (pu.Y().y - mvInfo->y);
          roundAffineMv(vx, vy, shift);
          mvTmp[0] = Mv(vx, vy);
          mvTmp[0].clipToStorageBitDepth();
          clipMv( mvTmp[0], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
          mvTmp[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
          vx = mvScaleHor + dMvHorX * (pu.Y().x + pu.Y().width - mvInfo->x) + dMvVerX * (pu.Y().y - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (pu.Y().x + pu.Y().width - mvInfo->x) + dMvVerY * (pu.Y().y - mvInfo->y);
          roundAffineMv(vx, vy, shift);
          mvTmp[1] = Mv(vx, vy);
          mvTmp[1].clipToStorageBitDepth();
          clipMv( mvTmp[1], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
          mvTmp[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
          mvTmp[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
          Distortion tmpCost = xGetAffineTemplateCost(pu, origBuf, predBuf, mvTmp, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp);
          if ( affineAmvrEnabled )
          {
            tmpCost += m_pcRdCost->getCost( xCalcAffineMVBits( pu, mvTmp, cMvPred[iRefList][iRefIdxTemp] ) );
          }
          if (tmpCost < uiCandCost)
          {
            uiCandCost = tmpCost;
            std::memcpy(mvHevc, mvTmp, 3 * sizeof(Mv));
          }
        }
      }
      if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
      {
        Mv mvFour[3];
        mvFour[0] = mvAffine4Para[iRefList][iRefIdxTemp][0];
        mvFour[1] = mvAffine4Para[iRefList][iRefIdxTemp][1];
        mvAffine4Para[iRefList][iRefIdxTemp][0].roundAffinePrecInternal2Amvr(pu.cu->imv);
        mvAffine4Para[iRefList][iRefIdxTemp][1].roundAffinePrecInternal2Amvr(pu.cu->imv);

        int shift = MAX_CU_DEPTH;
        int vx2 = (mvFour[0].getHor() << shift) - ((mvFour[1].getVer() - mvFour[0].getVer()) << (shift + floorLog2(pu.lheight()) - floorLog2(pu.lwidth())));
        int vy2 = (mvFour[0].getVer() << shift) + ((mvFour[1].getHor() - mvFour[0].getHor()) << (shift + floorLog2(pu.lheight()) - floorLog2(pu.lwidth())));
        int offset = (1 << (shift - 1));
        vx2 = (vx2 + offset - (vx2 >= 0)) >> shift;
        vy2 = (vy2 + offset - (vy2 >= 0)) >> shift;
        mvFour[2].hor = vx2;
        mvFour[2].ver = vy2;
        mvFour[2].clipToStorageBitDepth();
        mvFour[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
        mvFour[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
        mvFour[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
        Distortion uiCandCostInherit = xGetAffineTemplateCost( pu, origBuf, predBuf, mvFour, aaiMvpIdx[iRefList][iRefIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdxTemp );
        if ( affineAmvrEnabled )
        {
          uiCandCostInherit += m_pcRdCost->getCost( xCalcAffineMVBits( pu, mvFour, cMvPred[iRefList][iRefIdxTemp] ) );
        }
        if ( uiCandCostInherit < uiCandCost )
        {
          uiCandCost = uiCandCostInherit;
          for ( int i = 0; i < 3; i++ )
          {
            mvHevc[i] = mvFour[i];
          }
        }
      }

      if ( uiCandCost < biPDistTemp )
      {
        ::memcpy( cMvTemp[iRefList][iRefIdxTemp], mvHevc, sizeof(Mv)*3 );
      }
      else
      {
        ::memcpy( cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
      }

      // GPB list 1, save the best MvpIdx, RefIdx and Cost
      if ( slice.getPicHeader()->getMvdL1ZeroFlag() && iRefList==1 && biPDistTemp < bestBiPDist )
      {
        bestBiPDist = biPDistTemp;
        bestBiPMvpL1 = aaiMvpIdx[iRefList][iRefIdxTemp];
        bestBiPRefIdxL1 = iRefIdxTemp;
      }

      // Update bits
      uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdx[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];

      if ( m_pcEncCfg->getFastMEForGenBLowDelayEnabled() && iRefList == 1 )   // list 1
      {
        if ( slice.getList1IdxToList0Idx( iRefIdxTemp ) >= 0 && (pu.cu->affineType != AFFINEMODEL_6PARAM || slice.getList1IdxToList0Idx( iRefIdxTemp ) == refIdx4Para[0]) )
        {
          int iList1ToList0Idx = slice.getList1IdxToList0Idx( iRefIdxTemp );
          ::memcpy( cMvTemp[1][iRefIdxTemp], cMvTemp[0][iList1ToList0Idx], sizeof(Mv)*3 );
          uiCostTemp = uiCostTempL0[iList1ToList0Idx];

          uiCostTemp -= m_pcRdCost->getCost( uiBitsTempL0[iList1ToList0Idx] );
          uiBitsTemp += xCalcAffineMVBits( pu, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp] );
          /*calculate the correct cost*/
          uiCostTemp += m_pcRdCost->getCost( uiBitsTemp );
          DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp );
        }
        else
        {
          xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp
                                   , aaiMvpIdx[iRefList][iRefIdxTemp], affiAMVPInfoTemp[eRefPicList]
          );
        }
      }
      else
      {
        xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPred[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp
                                 , aaiMvpIdx[iRefList][iRefIdxTemp], affiAMVPInfoTemp[eRefPicList]
        );
      }
      if(pu.cu->cs->sps->getUseBcw() && pu.cu->bcwIdx == BCW_DEFAULT && pu.cu->slice->isInterB())
      {
        m_uniMotions.setReadModeAffine(true, (uint8_t)iRefList, (uint8_t)iRefIdxTemp, pu.cu->affineType);
        m_uniMotions.copyAffineMvFrom(cMvTemp[iRefList][iRefIdxTemp], uiCostTemp - m_pcRdCost->getCost(uiBitsTemp), (uint8_t)iRefList, (uint8_t)iRefIdxTemp, pu.cu->affineType
                                      , aaiMvpIdx[iRefList][iRefIdxTemp]
        );
      }
      // Set best AMVP Index
      xCopyAffineAMVPInfo( affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[iRefList][iRefIdxTemp] );
      if ( pu.cu->imv != 2 || !m_pcEncCfg->getUseAffineAmvrEncOpt() )
      xCheckBestAffineMVP( pu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPred[iRefList][iRefIdxTemp], aaiMvpIdx[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );

      if ( iRefList == 0 )
      {
        uiCostTempL0[iRefIdxTemp] = uiCostTemp;
        uiBitsTempL0[iRefIdxTemp] = uiBitsTemp;
      }
      DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d, uiCost[iRefList]=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp, uiCost[iRefList] );
      if ( uiCostTemp < uiCost[iRefList] )
      {
        uiCost[iRefList] = uiCostTemp;
        uiBits[iRefList] = uiBitsTemp; // storing for bi-prediction

        // set best motion
        ::memcpy( aacMv[iRefList], cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv) * 3 );
        iRefIdx[iRefList] = iRefIdxTemp;
      }
#if JVET_Z0054_BLK_REF_PIC_REORDER
      if (pu.cs->sps->getUseARL() && iRefList == 1 && slice.getList1IdxToList0Idx(iRefIdxTemp) >= 0)
      {
        uiCostTemp = MAX_UINT;
      }
#endif

      if ( iRefList == 1 && uiCostTemp < costValidList1 && slice.getList1IdxToList0Idx( iRefIdxTemp ) < 0 )
      {
        costValidList1 = uiCostTemp;
        bitsValidList1 = uiBitsTemp;

        // set motion
        memcpy( mvValidList1, cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
        refIdxValidList1 = iRefIdxTemp;
      }
    } // End refIdx loop
  } // end Uni-prediction

  if ( pu.cu->affineType == AFFINEMODEL_4PARAM )
  {
    ::memcpy( mvAffine4Para, cMvTemp, sizeof( cMvTemp ) );
    if ( pu.cu->imv == 0 && ( !pu.cu->cs->sps->getUseBcw() || bcwIdx == BCW_DEFAULT ) )
    {
      AffineMVInfo *affMVInfo = m_affMVList + m_affMVListIdx;

      //check;
      int j = 0;
      for (; j < m_affMVListSize; j++)
      {
        AffineMVInfo *prevMvInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
        if ((pu.Y().x == prevMvInfo->x) && (pu.Y().y == prevMvInfo->y) && (pu.Y().width == prevMvInfo->w) && (pu.Y().height == prevMvInfo->h))
        {
          break;
        }
      }
      if (j < m_affMVListSize)
        affMVInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));

      ::memcpy(affMVInfo->affMVs, cMvTemp, sizeof(cMvTemp));

      if (j == m_affMVListSize)
      {
        affMVInfo->x = pu.Y().x;
        affMVInfo->y = pu.Y().y;
        affMVInfo->w = pu.Y().width;
        affMVInfo->h = pu.Y().height;
        m_affMVListSize = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
        m_affMVListIdx = (m_affMVListIdx + 1) % (m_affMVListMaxSize);
      }
    }
  }

  // Bi-directional prediction
  if ( slice.isInterB() && !PU::isBipredRestriction(pu) 
#if AFFINE_ENC_OPT || MULTI_HYP_PRED 
    // In case refIdx4Para[i] is NOT_VALID, uiMotBits[i] would be undefined since list i will not be searched in 6-para model.
    // Therefore, the undefined bits would be stored in MHP candidates.
    && !(pu.cu->affineType == AFFINEMODEL_6PARAM && (refIdx4Para[0] == NOT_VALID || refIdx4Para[1] == NOT_VALID))
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
    && pu.cu->licDelta == 0
#endif
#if INTER_LIC && !JVET_AD0213_LIC_IMP
    && !pu.cu->licFlag
#endif
    )
  {
    tryBipred = 1;
    pu.interDir = 3;
    m_isBi = true;
    // Set as best list0 and list1
    iRefIdxBi[0] = iRefIdx[0];
    iRefIdxBi[1] = iRefIdx[1];

    ::memcpy( cMvBi,       aacMv,     sizeof(aacMv)     );
    ::memcpy( cMvPredBi,   cMvPred,   sizeof(cMvPred)   );
    ::memcpy( aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx) );

    uint32_t uiMotBits[2];
    bool doBiPred = true;

    if ( slice.getPicHeader()->getMvdL1ZeroFlag() ) // GPB, list 1 only use Mvp
    {
      xCopyAffineAMVPInfo( aacAffineAMVPInfo[1][bestBiPRefIdxL1], affiAMVPInfoTemp[REF_PIC_LIST_1] );
      pu.mvpIdx[REF_PIC_LIST_1] = bestBiPMvpL1;
      aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;

      // Set Mv for list1
      Mv pcMvTemp[3] = { affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandRT[bestBiPMvpL1],
                         affiAMVPInfoTemp[REF_PIC_LIST_1].mvCandLB[bestBiPMvpL1] };
      ::memcpy( cMvPredBi[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv)*3 );
      ::memcpy( cMvBi[1],                      pcMvTemp, sizeof(Mv)*3 );
      ::memcpy( cMvTemp[1][bestBiPRefIdxL1],   pcMvTemp, sizeof(Mv)*3 );
      iRefIdxBi[1] = bestBiPRefIdxL1;

      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        Area curTileAreaRestricted;
        curTileAreaRestricted = pu.cs->picture->mctsInfo.getTileAreaSubPelRestricted( pu );
        for( int i = 0; i < mvNum; i++ )
        {
          Mv restrictedMv = pcMvTemp[i];
          MCTSHelper::clipMvToArea( restrictedMv, pu.cu->Y(), curTileAreaRestricted, *pu.cs->sps );

          // If sub-pel filter samples are not inside of allowed area
          if( restrictedMv != pcMvTemp[i] )
          {
            uiCostBi = std::numeric_limits<Distortion>::max();
            doBiPred = false;
          }
        }
      }
      // Get list1 prediction block
      PU::setAllAffineMv( pu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], REF_PIC_LIST_1);
      pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

      PelUnitBuf predBufTmp = m_tmpPredStorage[REF_PIC_LIST_1].getBuf( UnitAreaRelative(*pu.cu, pu) );
      motionCompensation( pu, predBufTmp, REF_PIC_LIST_1 );

      // Update bits
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiMbBits[1];

      if( slice.getNumRefIdx(REF_PIC_LIST_1) > 1 )
      {
        uiMotBits[1] += bestBiPRefIdxL1+1;
        if( bestBiPRefIdxL1 == slice.getNumRefIdx(REF_PIC_LIST_1)-1 )
        {
          uiMotBits[1]--;
        }
      }
      uiMotBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }
    else
    {
      uiMotBits[0] = uiBits[0] - uiMbBits[0];
      uiMotBits[1] = uiBits[1] - uiMbBits[1];
      uiBits[2] = uiMbBits[2] + uiMotBits[0] + uiMotBits[1];
    }

    if( doBiPred )
    {
    // 4-times iteration (default)
    int iNumIter = 4;

    // fast encoder setting or GPB: only one iteration
    if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 || slice.getPicHeader()->getMvdL1ZeroFlag() )
    {
      iNumIter = 1;
    }

    for ( int iIter = 0; iIter < iNumIter; iIter++ )
    {
      // Set RefList
      int iRefList = iIter % 2;
      if ( m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE1 || m_pcEncCfg->getFastInterSearchMode()==FASTINTERSEARCH_MODE2 )
      {
        if( uiCost[0] <= uiCost[1] )
        {
          iRefList = 1;
        }
        else
        {
          iRefList = 0;
        }
        if( bcwIdx != BCW_DEFAULT )
        {
          iRefList = ( abs( getBcwWeight( bcwIdx, REF_PIC_LIST_0 ) ) > abs( getBcwWeight( bcwIdx, REF_PIC_LIST_1 ) ) ? 1 : 0 );
        }
      }
      else if ( iIter == 0 )
      {
        iRefList = 0;
      }

      // First iterate, get prediction block of opposite direction
      if( iIter == 0 && !slice.getPicHeader()->getMvdL1ZeroFlag() )
      {
        PU::setAllAffineMv( pu, aacMv[1-iRefList][0], aacMv[1-iRefList][1], aacMv[1-iRefList][2], RefPicList(1-iRefList));
        pu.refIdx[1-iRefList] = iRefIdx[1-iRefList];

        PelUnitBuf predBufTmp = m_tmpPredStorage[1 - iRefList].getBuf( UnitAreaRelative(*pu.cu, pu) );
        motionCompensation( pu, predBufTmp, RefPicList(1 - iRefList) );
      }

      RefPicList eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      if ( slice.getPicHeader()->getMvdL1ZeroFlag() ) // GPB, fix List 1, search List 0
      {
        iRefList = 0;
        eRefPicList = REF_PIC_LIST_0;
      }

      bool bChanged = false;

      iRefStart = 0;
      iRefEnd   = slice.getNumRefIdx(eRefPicList) - 1;
      for ( int iRefIdxTemp = iRefStart; iRefIdxTemp <= iRefEnd; iRefIdxTemp++ )
      {
#if JVET_Z0054_BLK_REF_PIC_REORDER
        if (pu.cs->sps->getUseARL())
        {
          int refIdxTemp[2];
          refIdxTemp[iRefList] = iRefIdxTemp;
          refIdxTemp[1 - iRefList] = iRefIdxBi[1 - iRefList];
          if (pu.cu->slice->getRefPicPairIdx(refIdxTemp[0], refIdxTemp[1]) < 0)
          {
            continue;
          }
        }
#endif
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM && refIdx4Para[iRefList] != iRefIdxTemp )
        {
          continue;
        }
#if !JVET_AD0213_LIC_IMP
        if(m_pcEncCfg->getUseBcwFast() && (bcwIdx != BCW_DEFAULT)
          && (pu.cu->slice->getRefPic(eRefPicList, iRefIdxTemp)->getPOC() == pu.cu->slice->getRefPic(RefPicList(1 - iRefList), pu.refIdx[1 - iRefList])->getPOC())
          && (pu.cu->affineType == AFFINEMODEL_4PARAM && pu.cu->slice->getTLayer()>1))
        {
          continue;
        }
#endif
        // update bits
        uiBitsTemp = uiMbBits[2] + uiMotBits[1-iRefList];
        uiBitsTemp += ((pu.cu->slice->getSPS()->getUseBcw() == true) ? bcwIdxBits : 0);
        if( slice.getNumRefIdx(eRefPicList) > 1 )
        {
          uiBitsTemp += iRefIdxTemp+1;
          if ( iRefIdxTemp == slice.getNumRefIdx(eRefPicList)-1 )
          {
            uiBitsTemp--;
          }
        }
        uiBitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[iRefList][iRefIdxTemp]][AMVP_MAX_NUM_CANDS];
        // call Affine ME
        xAffineMotionEstimation( pu, origBuf, eRefPicList, cMvPredBi[iRefList][iRefIdxTemp], iRefIdxTemp, cMvTemp[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp,
                                 aaiMvpIdxBi[iRefList][iRefIdxTemp], aacAffineAMVPInfo[iRefList][iRefIdxTemp],
          true );
        xCopyAffineAMVPInfo( aacAffineAMVPInfo[iRefList][iRefIdxTemp], affiAMVPInfoTemp[eRefPicList] );
        if ( pu.cu->imv != 2 || !m_pcEncCfg->getUseAffineAmvrEncOpt() )
        xCheckBestAffineMVP( pu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[iRefList][iRefIdxTemp], cMvPredBi[iRefList][iRefIdxTemp], aaiMvpIdxBi[iRefList][iRefIdxTemp], uiBitsTemp, uiCostTemp );

#if MULTI_HYP_PRED
        if(saveMeResultsForMHP)
        {
          // Affine bi
          MEResult biPredResult;
          biPredResult.cu = *pu.cu;
          biPredResult.cu.smvdMode = 0;
          biPredResult.pu = pu;
          biPredResult.cost = uiCostTemp;
          biPredResult.bits = uiBitsTemp;

          biPredResult.pu.mv[REF_PIC_LIST_0] = Mv();
          biPredResult.pu.mv[REF_PIC_LIST_1] = Mv();
          biPredResult.pu.mvd[REF_PIC_LIST_0] = cMvZero;
          biPredResult.pu.mvd[REF_PIC_LIST_1] = cMvZero;

          for (int i = 0; i < 3; ++i)
          {
            biPredResult.pu.mvAffi[iRefList][i] = cMvTemp[iRefList][iRefIdxTemp][i];
            biPredResult.pu.mvAffi[1 - iRefList][i] = cMvBi[1 - iRefList][i];
            biPredResult.pu.mvAffi[0][i].roundAffinePrecInternal2Amvr(pu.cu->imv);
            biPredResult.pu.mvAffi[1][i].roundAffinePrecInternal2Amvr(pu.cu->imv);
          }

          biPredResult.pu.refIdx[iRefList] = iRefIdxTemp;
          biPredResult.pu.refIdx[1 - iRefList] = iRefIdxBi[1 - iRefList];

          for (int verIdx = 0; verIdx < mvNum; verIdx++)
          {
            biPredResult.pu.mvdAffi[iRefList][verIdx] = cMvTemp[iRefList][iRefIdxTemp][verIdx] - cMvPredBi[iRefList][iRefIdxTemp][verIdx];
            biPredResult.pu.mvdAffi[1 - iRefList][verIdx] = cMvBi[1 - iRefList][verIdx] - cMvPredBi[1 - iRefList][iRefIdxBi[1 - iRefList]][verIdx];
            if (verIdx != 0)
            {
              biPredResult.pu.mvdAffi[0][verIdx] = biPredResult.pu.mvdAffi[0][verIdx] - biPredResult.pu.mvdAffi[0][0];
              biPredResult.pu.mvdAffi[1][verIdx] = biPredResult.pu.mvdAffi[1][verIdx] - biPredResult.pu.mvdAffi[1][0];
            }
          }

          biPredResult.pu.interDir = 3;
          biPredResult.pu.mvpIdx[iRefList] = aaiMvpIdxBi[iRefList][iRefIdxTemp];
          biPredResult.pu.mvpIdx[1 - iRefList] = aaiMvpIdxBi[1 - iRefList][iRefIdxBi[1 - iRefList]];
          biPredResult.pu.mvpNum[iRefList] = aaiMvpNum[iRefList][iRefIdxTemp];
          biPredResult.pu.mvpNum[1 - iRefList] = aaiMvpNum[1 - iRefList][iRefIdxBi[1 - iRefList]];

          pu.cs->m_meResults.push_back(biPredResult);
        }
#endif
        if ( uiCostTemp < uiCostBi )
        {
          bChanged = true;
          ::memcpy( cMvBi[iRefList], cMvTemp[iRefList][iRefIdxTemp], sizeof(Mv)*3 );
          iRefIdxBi[iRefList] = iRefIdxTemp;

          uiCostBi            = uiCostTemp;
          uiMotBits[iRefList] = uiBitsTemp - uiMbBits[2] - uiMotBits[1-iRefList];
          uiMotBits[iRefList] -= ((pu.cu->slice->getSPS()->getUseBcw() == true) ? bcwIdxBits : 0);
          uiBits[2]           = uiBitsTemp;

          if ( iNumIter != 1 ) // MC for next iter
          {
            //  Set motion
            PU::setAllAffineMv( pu, cMvBi[iRefList][0], cMvBi[iRefList][1], cMvBi[iRefList][2], eRefPicList);
            pu.refIdx[eRefPicList] = iRefIdxBi[eRefPicList];
            PelUnitBuf predBufTmp = m_tmpPredStorage[iRefList].getBuf( UnitAreaRelative(*pu.cu, pu) );
            motionCompensation( pu, predBufTmp, eRefPicList );
          }
        }
      } // for loop-iRefIdxTemp

      if ( !bChanged )
      {
        if ((uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1]) || enforceBcwPred)
        {
          xCopyAffineAMVPInfo( aacAffineAMVPInfo[0][iRefIdxBi[0]], affiAMVPInfoTemp[REF_PIC_LIST_0] );
          xCheckBestAffineMVP( pu, affiAMVPInfoTemp[REF_PIC_LIST_0], REF_PIC_LIST_0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]], aaiMvpIdxBi[0][iRefIdxBi[0]], uiBits[2], uiCostBi );

          if ( !slice.getPicHeader()->getMvdL1ZeroFlag() )
          {
            xCopyAffineAMVPInfo( aacAffineAMVPInfo[1][iRefIdxBi[1]], affiAMVPInfoTemp[REF_PIC_LIST_1] );
            xCheckBestAffineMVP( pu, affiAMVPInfoTemp[REF_PIC_LIST_1], REF_PIC_LIST_1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]], aaiMvpIdxBi[1][iRefIdxBi[1]], uiBits[2], uiCostBi );
          }
        }
        break;
      }
    } // for loop-iter
    }
    m_isBi = false;
  } // if (B_SLICE)

  pu.mv    [REF_PIC_LIST_0] = Mv();
  pu.mv    [REF_PIC_LIST_1] = Mv();
  pu.mvd   [REF_PIC_LIST_0] = cMvZero;
  pu.mvd   [REF_PIC_LIST_1] = cMvZero;
  pu.refIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.refIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

  for ( int verIdx = 0; verIdx < 3; verIdx++ )
  {
    pu.mvdAffi[REF_PIC_LIST_0][verIdx] = cMvZero;
    pu.mvdAffi[REF_PIC_LIST_1][verIdx] = cMvZero;
  }

  // Set Motion Field
  memcpy( aacMv[1], mvValidList1, sizeof(Mv)*3 );
  iRefIdx[1] = refIdxValidList1;
  uiBits[1]  = bitsValidList1;
  uiCost[1]  = costValidList1;
  if (pu.cs->pps->getWPBiPred() == true && tryBipred && (bcwIdx != BCW_DEFAULT))
  {
    CHECK(iRefIdxBi[0]<0, "Invalid picture reference index");
    CHECK(iRefIdxBi[1]<0, "Invalid picture reference index");
    wp0 = pu.cs->slice->getWpScaling(REF_PIC_LIST_0, iRefIdxBi[0]);
    wp1 = pu.cs->slice->getWpScaling(REF_PIC_LIST_1, iRefIdxBi[1]);

    if (WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1))
    {
      uiCostBi = MAX_UINT;
      enforceBcwPred = false;
    }
  }
  if( enforceBcwPred )
  {
    uiCost[0] = uiCost[1] = MAX_UINT;
  }

  // Affine ME result set
  if ( uiCostBi <= uiCost[0] && uiCostBi <= uiCost[1] ) // Bi
  {
    lastMode = 2;
    affineCost = uiCostBi;
    pu.interDir = 3;

    pu.refIdx[REF_PIC_LIST_0] = iRefIdxBi[0];
    pu.refIdx[REF_PIC_LIST_1] = iRefIdxBi[1];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvAffi[REF_PIC_LIST_0][verIdx] = cMvBi[0][verIdx];
      pu.mvAffi[REF_PIC_LIST_1][verIdx] = cMvBi[1][verIdx];
      pu.mvdAffi[REF_PIC_LIST_0][verIdx] = cMvBi[0][verIdx] - cMvPredBi[0][iRefIdxBi[0]][verIdx];
      pu.mvdAffi[REF_PIC_LIST_1][verIdx] = cMvBi[1][verIdx] - cMvPredBi[1][iRefIdxBi[1]][verIdx];

      if ( verIdx != 0 )
      {
        pu.mvdAffi[0][verIdx] = pu.mvdAffi[0][verIdx] - pu.mvdAffi[0][0];
        pu.mvdAffi[1][verIdx] = pu.mvdAffi[1][verIdx] - pu.mvdAffi[1][0];
      }
    }


    pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
    pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdxBi[0]];
    pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
    pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdxBi[1]];
  }
  else if ( uiCost[0] <= uiCost[1] ) // List 0
  {
    lastMode = 0;
    affineCost = uiCost[0];
    pu.interDir = 1;
    pu.mv[1].setZero();
    pu.refIdx[REF_PIC_LIST_0] = iRefIdx[0];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvAffi[REF_PIC_LIST_0][verIdx] = aacMv[0][verIdx];
      pu.mvdAffi[REF_PIC_LIST_0][verIdx] = aacMv[0][verIdx] - cMvPred[0][iRefIdx[0]][verIdx];
      if ( verIdx != 0 )
      {
        pu.mvdAffi[0][verIdx] = pu.mvdAffi[0][verIdx] - pu.mvdAffi[0][0];
      }
    }

    pu.mvpIdx[REF_PIC_LIST_0] = aaiMvpIdx[0][iRefIdx[0]];
    pu.mvpNum[REF_PIC_LIST_0] = aaiMvpNum[0][iRefIdx[0]];
  }
  else
  {
    lastMode = 1;
    affineCost = uiCost[1];
    pu.interDir = 2;
    pu.mv[0].setZero();
    pu.refIdx[REF_PIC_LIST_1] = iRefIdx[1];

    for ( int verIdx = 0; verIdx < mvNum; verIdx++ )
    {
      pu.mvAffi[REF_PIC_LIST_1][verIdx] = aacMv[1][verIdx];
      pu.mvdAffi[REF_PIC_LIST_1][verIdx] = aacMv[1][verIdx] - cMvPred[1][iRefIdx[1]][verIdx];
      if ( verIdx != 0 )
      {
        pu.mvdAffi[1][verIdx] = pu.mvdAffi[1][verIdx] - pu.mvdAffi[1][0];
      }
    }

    pu.mvpIdx[REF_PIC_LIST_1] = aaiMvpIdx[1][iRefIdx[1]];
    pu.mvpNum[REF_PIC_LIST_1] = aaiMvpNum[1][iRefIdx[1]];
  }
  if( bcwIdx != BCW_DEFAULT )
  {
    pu.cu->bcwIdx = BCW_DEFAULT;
  }
}

// Ax = b, m = {A, b}
#if AFFINE_ENC_OPT
void solveGaussElimination( double( *m )[7], double *x, int num )
{
#define NEARZERO(x) x == 0.

  const int numM1 = num - 1;

  for( int i = 0; i < numM1; i++ )
  {
    // find non-zero diag
    int tempIdx = i;
    if( NEARZERO( m[i][i] ) )
    {
      for( int j = i + 1; j < num; j++ )
      {
        if( !( NEARZERO( m[j][i] ) ) )
        {
          tempIdx = j;
          break;
        }
      }
    }

    // swap line
    if( tempIdx != i )
    {
      swap( m[i], m[tempIdx] );
    }

    double* currRow = m[i];
    const double diagCoeff = currRow[i];

    if( NEARZERO( diagCoeff ) )
    {
      std::memset( x, 0, sizeof( *x ) * num );
      return;
    }

    // eliminate column
    for( int j = i + 1; j < num; j++ )
    {
      double* rowCoeff = m[j];
      const double coeffRatio = rowCoeff[i] / diagCoeff;

      for( int k = i + 1; k <= num; k++ )
      {
        rowCoeff[k] -= currRow[k] * coeffRatio;
      }
    }
  }

  if( NEARZERO( m[numM1][numM1] ) )
  {
    std::memset( x, 0, sizeof( *x ) * num );
    return;
  }

  double* currRow = m[numM1];
  x[numM1] = currRow[num] / currRow[numM1];

  for( int i = num - 2; i >= 0; i-- )
  {
    currRow = m[i];
    const double diagCoeff = currRow[i];

    if( NEARZERO( diagCoeff ) )
    {
      std::memset( x, 0, sizeof( *x ) * num );
      return;
    }

    double temp = 0;
    for( int j = i + 1; j < num; j++ )
    {
      temp += currRow[j] * x[j];
    }
    x[i] = ( currRow[num] - temp ) / diagCoeff;
  }
#undef NEARZERO
}
#else
void solveEqual( double dEqualCoeff[7][7], int iOrder, double *dAffinePara )
{
  for( int k = 0; k < iOrder; k++ )
  {
    dAffinePara[k] = 0.;
  }

  // row echelon
  for( int i = 1; i < iOrder; i++ )
  {
    // find column max
    double temp = fabs( dEqualCoeff[i][i - 1] );
    int tempIdx = i;
    for( int j = i + 1; j < iOrder + 1; j++ )
    {
      if( fabs( dEqualCoeff[j][i - 1] ) > temp )
      {
        temp = fabs( dEqualCoeff[j][i - 1] );
        tempIdx = j;
      }
    }

    // swap line
    if( tempIdx != i )
    {
      for( int j = 0; j < iOrder + 1; j++ )
      {
        dEqualCoeff[0][j] = dEqualCoeff[i][j];
        dEqualCoeff[i][j] = dEqualCoeff[tempIdx][j];
        dEqualCoeff[tempIdx][j] = dEqualCoeff[0][j];
      }
    }

    // elimination first column
    if( dEqualCoeff[i][i - 1] == 0. )
    {
      return;
    }
    for( int j = i + 1; j < iOrder + 1; j++ )
    {
      for( int k = i; k < iOrder + 1; k++ )
      {
        dEqualCoeff[j][k] = dEqualCoeff[j][k] - dEqualCoeff[i][k] * dEqualCoeff[j][i - 1] / dEqualCoeff[i][i - 1];
      }
    }
  }

  if( dEqualCoeff[iOrder][iOrder - 1] == 0. )
  {
    return;
  }
  dAffinePara[iOrder - 1] = dEqualCoeff[iOrder][iOrder] / dEqualCoeff[iOrder][iOrder - 1];
  for( int i = iOrder - 2; i >= 0; i-- )
  {
    if( dEqualCoeff[i + 1][i] == 0. )
    {
      for( int k = 0; k < iOrder; k++ )
      {
        dAffinePara[k] = 0.;
      }
      return;
    }
    double temp = 0;
    for( int j = i + 1; j < iOrder; j++ )
    {
      temp += dEqualCoeff[i + 1][j] * dAffinePara[j];
    }
    dAffinePara[i] = ( dEqualCoeff[i + 1][iOrder] - temp ) / dEqualCoeff[i + 1][i];
  }
}
#endif

void InterSearch::xCheckBestAffineMVP( PredictionUnit &pu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost )
{
  if ( affineAMVPInfo.numCand < 2 )
  {
    return;
  }
  int mvNum = pu.cu->affineType ? 3 : 2;
  m_pcRdCost->selectMotionLambda( );
  m_pcRdCost->setCostScale ( 0 );

  int iBestMVPIdx = riMVPIdx;

  // Get origin MV bits
  Mv tmpPredMv[3];
  int iOrgMvBits = xCalcAffineMVBits( pu, acMv, acMvPred );
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  int iBestMvBits = iOrgMvBits;
  for (int iMVPIdx = 0; iMVPIdx < affineAMVPInfo.numCand; iMVPIdx++)
  {
    if (iMVPIdx == riMVPIdx)
    {
      continue;
    }
    tmpPredMv[0] = affineAMVPInfo.mvCandLT[iMVPIdx];
    tmpPredMv[1] = affineAMVPInfo.mvCandRT[iMVPIdx];
    if ( mvNum == 3 )
    {
      tmpPredMv[2] = affineAMVPInfo.mvCandLB[iMVPIdx];
    }
    int iMvBits = xCalcAffineMVBits( pu, acMv, tmpPredMv );
    iMvBits += m_auiMVPIdxCost[iMVPIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = iMVPIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)  // if changed
  {
    acMvPred[0] = affineAMVPInfo.mvCandLT[iBestMVPIdx];
    acMvPred[1] = affineAMVPInfo.mvCandRT[iBestMVPIdx];
    acMvPred[2] = affineAMVPInfo.mvCandLB[iBestMVPIdx];
    riMVPIdx = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost = (ruiCost - m_pcRdCost->getCost( uiOrgBits )) + m_pcRdCost->getCost( ruiBits );
  }
}

void InterSearch::xAffineMotionEstimation( PredictionUnit& pu,
                                           PelUnitBuf&     origBuf,
                                           RefPicList      eRefPicList,
                                           Mv              acMvPred[3],
                                           int             iRefIdxPred,
                                           Mv              acMv[3],
                                           uint32_t&           ruiBits,
                                           Distortion&     ruiCost,
                                           int&            mvpIdx,
                                           const AffineAMVPInfo& aamvpi,
                                           bool            bBi)
{
  if( pu.cu->cs->sps->getUseBcw() && pu.cu->bcwIdx != BCW_DEFAULT && !bBi && xReadBufferedAffineUniMv(pu, eRefPicList, iRefIdxPred, acMvPred, acMv, ruiBits, ruiCost
      , mvpIdx, aamvpi
  ) )
  {
    return;
  }

  uint32_t dirBits = ruiBits - m_auiMVPIdxCost[mvpIdx][aamvpi.numCand];
  int bestMvpIdx   = mvpIdx;
  const int width  = pu.Y().width;
  const int height = pu.Y().height;

  const Picture* refPic = pu.cu->slice->getRefPic(eRefPicList, iRefIdxPred);

  // Set Origin YUV: pcYuv
  PelUnitBuf*   pBuf = &origBuf;
  double        fWeight       = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative( *pu.cu, pu ) );
  enum DFunc distFunc = (pu.cs->slice->getDisableSATDForRD()) ? DF_SAD : DF_HAD;
  m_iRefListIdx = eRefPicList;

  // if Bi, set to ( 2 * Org - ListX )
  if ( bBi )
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf( UnitAreaRelative( *pu.cu, pu ) );
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs()
                             ,getBcwWeight(pu.cu->bcwIdx, eRefPicList)
                             );
    pBuf = &origBufTmp;

    fWeight = xGetMEDistortionWeight( pu.cu->bcwIdx, eRefPicList );
  }

  // pred YUV
  PelUnitBuf  predBuf = m_tmpAffiStorage.getBuf( UnitAreaRelative(*pu.cu, pu) );

  // Set start Mv position, use input mv as started search mv
  Mv acMvTemp[3];
  ::memcpy( acMvTemp, acMv, sizeof(Mv)*3 );
  // Set delta mv
  // malloc buffer
  int iParaNum = pu.cu->affineType ? 7 : 5;
  int affineParaNum = iParaNum - 1;
  int mvNum = pu.cu->affineType ? 3 : 2;

  int64_t  i64EqualCoeff[7][7];
  Pel    *piError = m_tmpAffiError;
#if AFFINE_ENC_OPT // using Pel instead of int for better SIMD
  Pel    *pdDerivate[2];
#else
  int    *pdDerivate[2];
#endif
  pdDerivate[0] = m_tmpAffiDeri[0];
  pdDerivate[1] = m_tmpAffiDeri[1];

  Distortion uiCostBest = std::numeric_limits<Distortion>::max();
  uint32_t uiBitsBest = 0;

  // do motion compensation with origin mv
  if( m_pcEncCfg->getMCTSEncConstraint() )
  {
    Area curTileAreaRestricted = pu.cs->picture->mctsInfo.getTileAreaSubPelRestricted( pu );
    MCTSHelper::clipMvToArea( acMvTemp[0], pu.cu->Y(), curTileAreaRestricted, *pu.cs->sps );
    MCTSHelper::clipMvToArea( acMvTemp[1], pu.cu->Y(), curTileAreaRestricted, *pu.cs->sps );
    if( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      MCTSHelper::clipMvToArea( acMvTemp[2], pu.cu->Y(), curTileAreaRestricted, *pu.cs->sps );
    }
  }
  else
  {
    clipMv( acMvTemp[0], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    clipMv( acMvTemp[1], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    if( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      clipMv( acMvTemp[2], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
    }
  }
  acMvTemp[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  acMvTemp[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    acMvTemp[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
#if AFFINE_ENC_OPT
  int gStride = width;
#if JVET_Z0136_OOB
  xPredAffineBlk(COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cs->slice->clpRng(COMPONENT_Y), eRefPicList, false, SCALE_1X, true);
#else
  xPredAffineBlk(COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cs->slice->clpRng(COMPONENT_Y), false, SCALE_1X, true);
#endif
#else
#if JVET_Z0136_OOB
  xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cs->slice->clpRng(COMPONENT_Y), eRefPicList );
#else
  xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cs->slice->clpRng( COMPONENT_Y ) );
#endif
#endif

  // get error
  uiCostBest = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, distFunc);

  // get cost with mv
  m_pcRdCost->setCostScale(0);
  uiBitsBest = ruiBits;
  if ( pu.cu->imv == 2 && m_pcEncCfg->getUseAffineAmvrEncOpt() )
  {
    uiBitsBest  = dirBits + xDetermineBestMvp( pu, acMvTemp, mvpIdx, aamvpi );
    acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
    acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
    acMvPred[2] = aamvpi.mvCandLB[mvpIdx];
  }
  else
  {
    DTRACE( g_trace_ctx, D_COMMON, " (%d) xx uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest );
    uiBitsBest += xCalcAffineMVBits( pu, acMvTemp, acMvPred );
    DTRACE( g_trace_ctx, D_COMMON, " (%d) yy uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest );
  }
  uiCostBest = (Distortion)( floor( fWeight * (double)uiCostBest ) + (double)m_pcRdCost->getCost( uiBitsBest ) );

  DTRACE( g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest, uiCostBest );

  ::memcpy( acMv, acMvTemp, sizeof(Mv) * 3 );

  const int bufStride = pBuf->Y().stride;
  const int predBufStride = predBuf.Y().stride;
  Mv prevIterMv[7][3];
  int iIterTime;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iIterTime = bBi ? 3 : 4;
  }
  else
  {
    iIterTime = bBi ? 3 : 5;
  }

  if ( !pu.cu->cs->sps->getUseAffineType() )
  {
    iIterTime = bBi ? 5 : 7;
  }
  for ( int iter=0; iter<iIterTime; iter++ )    // iterate loop
  {
    memcpy( prevIterMv[iter], acMvTemp, sizeof( Mv ) * 3 );
    /*********************************************************************************
     *                         use gradient to update mv
     *********************************************************************************/
    // get Error Matrix
    Pel* pOrg  = pBuf->Y().buf;
    Pel* pPred = predBuf.Y().buf;
    Pel* error = piError;

    for ( int j=0; j< height; j++ )
    {
      for ( int i=0; i< width; i++ )
      {
        error[i] = pOrg[i] - pPred[i];
      }
      pOrg  += bufStride;
      pPred += predBufStride;
      error += width;
    }

#if AFFINE_ENC_OPT
    pdDerivate[0] = m_gradX0 + gStride + 1;
    pdDerivate[1] = m_gradY0 + gStride + 1;
#else
    // sobel x direction
    // -1 0 1
    // -2 0 2
    // -1 0 1
    pPred = predBuf.Y().buf;
    m_HorizontalSobelFilter( pPred, predBufStride, pdDerivate[0], width, width, height );

    // sobel y direction
    // -1 -2 -1
    //  0  0  0
    //  1  2  1
    m_VerticalSobelFilter( pPred, predBufStride, pdDerivate[1], width, width, height );
#endif

    // solve delta x and y
    for ( int row = 0; row < iParaNum; row++ )
    {
      memset( &i64EqualCoeff[row][0], 0, iParaNum * sizeof( int64_t ) );
    }

#if AFFINE_ENC_OPT
    // the "6" is the shift number in gradient (canculated in IF_INTERNAL_PREC precision), "-1" is for gradient normalization
    // the input parameter "shift" in is to compensate dI with regard to the gradient
    m_EqualCoeffComputer( piError, width, pdDerivate, gStride, i64EqualCoeff, width, height
      , (pu.cu->affineType == AFFINEMODEL_6PARAM)
      , 6 - 1 - std::max<int>(2, (IF_INTERNAL_PREC - pu.cs->slice->clpRng(COMPONENT_Y).bd))
    );
#else
    m_EqualCoeffComputer( piError, width, pdDerivate, width, i64EqualCoeff, width, height
      , (pu.cu->affineType == AFFINEMODEL_6PARAM)
    );
#endif

    double dAffinePara[6];
    double dDeltaMv[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
    Mv acDeltaMv[3];

#if AFFINE_ENC_OPT
    double pdEqualCoeff[6][7];

    for( int row = 0; row < affineParaNum; row++ )
    {
      double* dCoeff = pdEqualCoeff[row];
      int64_t* iCoeff = i64EqualCoeff[row + 1];

      for( int i = 0; i < iParaNum; i++ )
      {
        dCoeff[i] = ( double ) iCoeff[i];
      }
    }

    solveGaussElimination( pdEqualCoeff, dAffinePara, affineParaNum );
#else
    double pdEqualCoeff[7][7];
    for( int row = 0; row < iParaNum; row++ )
    {
      for( int i = 0; i < iParaNum; i++ )
      {
        pdEqualCoeff[row][i] = ( double ) i64EqualCoeff[row][i];
      }
    }

    solveEqual( pdEqualCoeff, affineParaNum, dAffinePara );
#endif

    // convert to delta mv
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = dAffinePara[3] * width + dAffinePara[2];
      dDeltaMv[4] = dAffinePara[4] * height + dAffinePara[0];
      dDeltaMv[5] = dAffinePara[5] * height + dAffinePara[2];
    }
    else
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = -dAffinePara[3] * width + dAffinePara[2];
    }

    const int normShiftTab[3] = { MV_PRECISION_QUARTER - MV_PRECISION_INT, MV_PRECISION_SIXTEENTH - MV_PRECISION_INT, MV_PRECISION_QUARTER - MV_PRECISION_INT };
    const int stepShiftTab[3] = { MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH, MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER };
    const int multiShift = 1 << normShiftTab[pu.cu->imv];
    const int mvShift = stepShiftTab[pu.cu->imv];
    acDeltaMv[0] = Mv( ( int ) ( dDeltaMv[0] * multiShift + SIGN( dDeltaMv[0] ) * 0.5 ) << mvShift, ( int ) ( dDeltaMv[2] * multiShift + SIGN( dDeltaMv[2] ) * 0.5 ) << mvShift );
    acDeltaMv[1] = Mv( ( int ) ( dDeltaMv[1] * multiShift + SIGN( dDeltaMv[1] ) * 0.5 ) << mvShift, ( int ) ( dDeltaMv[3] * multiShift + SIGN( dDeltaMv[3] ) * 0.5 ) << mvShift );
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      acDeltaMv[2] = Mv( ( int ) ( dDeltaMv[4] * multiShift + SIGN( dDeltaMv[4] ) * 0.5 ) << mvShift, ( int ) ( dDeltaMv[5] * multiShift + SIGN( dDeltaMv[5] ) * 0.5 ) << mvShift );
    }
    if ( !m_pcEncCfg->getUseAffineAmvrEncOpt() )
    {
      bool bAllZero = false;
      for ( int i = 0; i < mvNum; i++ )
      {
        Mv deltaMv = acDeltaMv[i];
        if ( pu.cu->imv == 2 )
        {
          deltaMv.roundToPrecision( MV_PRECISION_INTERNAL, MV_PRECISION_HALF );
        }
        if ( deltaMv.getHor() != 0 || deltaMv.getVer() != 0 )
        {
          bAllZero = false;
          break;
        }
        bAllZero = true;
      }

      if ( bAllZero )
        break;
    }
    // do motion compensation with updated mv
    for ( int i = 0; i < mvNum; i++ )
    {
      acMvTemp[i] += acDeltaMv[i];
      acMvTemp[i].hor = Clip3(MV_MIN, MV_MAX, acMvTemp[i].hor );
      acMvTemp[i].ver = Clip3(MV_MIN, MV_MAX, acMvTemp[i].ver );
      acMvTemp[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
        MCTSHelper::clipMvToArea( acMvTemp[i], pu.cu->Y(), pu.cs->picture->mctsInfo.getTileAreaSubPelRestricted( pu ), *pu.cs->sps );
      }
      else
      {
        clipMv( acMvTemp[i], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
      }
    }

    if ( m_pcEncCfg->getUseAffineAmvrEncOpt() )
    {
      bool identical = false;
      for ( int k = iter; k >= 0; k-- )
      {
        if ( acMvTemp[0] == prevIterMv[k][0] && acMvTemp[1] == prevIterMv[k][1] )
        {
          identical = pu.cu->affineType ? acMvTemp[2] == prevIterMv[k][2] : true;
          if ( identical )
          {
            break;
          }
        }
      }
      if ( identical )
      {
        break;
      }
    }

#if AFFINE_ENC_OPT
#if JVET_Z0136_OOB
    xPredAffineBlk(COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), eRefPicList, false, SCALE_1X, true);
#else
    xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, SCALE_1X, true );
#endif
#else
#if JVET_Z0136_OOB
    xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), eRefPicList  );
#else
    xPredAffineBlk( COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng( COMPONENT_Y ) );
#endif
#endif

    // get error
    Distortion uiCostTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, distFunc);
    DTRACE( g_trace_ctx, D_COMMON, " (%d) uiCostTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiCostTemp );

    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t uiBitsTemp = ruiBits;
    if ( pu.cu->imv == 2 && m_pcEncCfg->getUseAffineAmvrEncOpt() )
    {
      uiBitsTemp  = dirBits + xDetermineBestMvp( pu, acMvTemp, bestMvpIdx, aamvpi );
      acMvPred[0] = aamvpi.mvCandLT[bestMvpIdx];
      acMvPred[1] = aamvpi.mvCandRT[bestMvpIdx];
      acMvPred[2] = aamvpi.mvCandLB[bestMvpIdx];
    }
    else
    {
      uiBitsTemp += xCalcAffineMVBits( pu, acMvTemp, acMvPred );
    }
    uiCostTemp = (Distortion)( floor( fWeight * (double)uiCostTemp ) + (double)m_pcRdCost->getCost( uiBitsTemp ) );

    // store best cost and mv
    if ( uiCostTemp < uiCostBest )
    {
      uiCostBest = uiCostTemp;
      uiBitsBest = uiBitsTemp;
      memcpy( acMv, acMvTemp, sizeof(Mv) * 3 );
      mvpIdx = bestMvpIdx;
    }
  }

  auto checkCPMVRdCost = [&](Mv ctrlPtMv[3])
  {
#if JVET_Z0136_OOB
    xPredAffineBlk(COMPONENT_Y, pu, refPic, ctrlPtMv, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), eRefPicList);
#else
    xPredAffineBlk(COMPONENT_Y, pu, refPic, ctrlPtMv, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y));
#endif
    // get error
    Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, distFunc);
    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t bitsTemp = ruiBits;
    bitsTemp += xCalcAffineMVBits( pu, ctrlPtMv, acMvPred );
    costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));
    // store best cost and mv
    if (costTemp < uiCostBest)
    {
      uiCostBest = costTemp;
      uiBitsBest = bitsTemp;
      ::memcpy(acMv, ctrlPtMv, sizeof(Mv) * 3);
    }
  };

  const uint32_t mvShiftTable[3] = {MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL - MV_PRECISION_INTERNAL, MV_PRECISION_INTERNAL - MV_PRECISION_INT};
  const uint32_t mvShift = mvShiftTable[pu.cu->imv];
  if (uiCostBest <= AFFINE_ME_LIST_MVP_TH*m_hevcCost)
  {

    Mv mvPredTmp[3] = { acMvPred[0], acMvPred[1], acMvPred[2] };
    Mv mvME[3];
    ::memcpy(mvME, acMv, sizeof(Mv) * 3);
    Mv dMv = mvME[0] - mvPredTmp[0];

    for (int j = 0; j < mvNum; j++)
    {
      if ((!j && mvME[j] != mvPredTmp[j]) || (j && mvME[j] != (mvPredTmp[j] + dMv)))
      {
        ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
        acMvTemp[j] = mvPredTmp[j];

        if (j)
          acMvTemp[j] += dMv;

        checkCPMVRdCost(acMvTemp);
      }
    }

    //keep the rotation/zoom;
    if (mvME[0] != mvPredTmp[0])
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
      for (int i = 1; i < mvNum; i++)
      {
        acMvTemp[i] -= dMv;
      }
      acMvTemp[0] = mvPredTmp[0];

      checkCPMVRdCost(acMvTemp);
    }

    //keep the translation;
    if (pu.cu->affineType == AFFINEMODEL_6PARAM && mvME[1] != (mvPredTmp[1] + dMv) && mvME[2] != (mvPredTmp[2] + dMv))
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);

      acMvTemp[1] = mvPredTmp[1] + dMv;
      acMvTemp[2] = mvPredTmp[2] + dMv;

      checkCPMVRdCost(acMvTemp);
    }

    // 8 nearest neighbor search
    int testPos[8][2] = { { -1, 0 },{ 0, -1 },{ 0, 1 },{ 1, 0 },{ -1, -1 },{ -1, 1 },{ 1, 1 },{ 1, -1 } };
    const int maxSearchRound = (pu.cu->imv) ? 3 : ((m_pcEncCfg->getUseAffineAmvrEncOpt() && m_pcEncCfg->getIntraPeriod() == (uint32_t)-1) ? 2 : 3);

    for (int rnd = 0; rnd < maxSearchRound; rnd++)
    {
      bool modelChange = false;
      //search the model parameters with finear granularity;
      for (int j = 0; j < mvNum; j++)
      {
        bool loopChange = false;
        for (int iter = 0; iter < 2; iter++)
        {
          if (iter == 1 && !loopChange)
          {
            break;
          }
          Mv centerMv[3];
          memcpy(centerMv, acMv, sizeof(Mv) * 3);
          memcpy(acMvTemp, acMv, sizeof(Mv) * 3);

          for (int i = ((iter == 0) ? 0 : 4); i < ((iter == 0) ? 4 : 8); i++)
          {
            acMvTemp[j].set(centerMv[j].getHor() + (testPos[i][0] << mvShift), centerMv[j].getVer() + (testPos[i][1] << mvShift));
            clipMv( acMvTemp[j], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if JVET_Z0136_OOB
            xPredAffineBlk(COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y), eRefPicList);
#else
            xPredAffineBlk(COMPONENT_Y, pu, refPic, acMvTemp, predBuf, false, pu.cu->slice->clpRng(COMPONENT_Y));
#endif
            Distortion costTemp = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, distFunc);
            uint32_t bitsTemp = ruiBits;
            bitsTemp += xCalcAffineMVBits(pu, acMvTemp, acMvPred);
            costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));

            if (costTemp < uiCostBest)
            {
              uiCostBest = costTemp;
              uiBitsBest = bitsTemp;
              ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
              modelChange = true;
              loopChange = true;
            }
          }
        }
      }

      if (!modelChange)
      {
        break;
      }
    }
  }
  acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
  acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
  acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

  ruiBits = uiBitsBest;
  ruiCost = uiCostBest;
  DTRACE( g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_COMMON), uiBitsBest, uiCostBest );
}

void InterSearch::xEstimateAffineAMVP( PredictionUnit&  pu,
                                       AffineAMVPInfo&  affineAMVPInfo,
                                       PelUnitBuf&      origBuf,
                                       RefPicList       eRefPicList,
                                       int              iRefIdx,
                                       Mv               acMvPred[3],
                                       Distortion*      puiDistBiP )
{
  Mv         bestMvLT, bestMvRT, bestMvLB;
  int        iBestIdx = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();

  // Fill the MV Candidates
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  PU::fillAffineMvpCand( this, pu, eRefPicList, iRefIdx, affineAMVPInfo
#else
  PU::fillAffineMvpCand( pu, eRefPicList, iRefIdx, affineAMVPInfo 
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
                , this
#endif 
  );

  CHECK( affineAMVPInfo.numCand == 0, "Assertion failed." );

  PelUnitBuf predBuf = m_tmpStorageLCU.getBuf( UnitAreaRelative(*pu.cu, pu) );

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  for( int i = 0 ; i < affineAMVPInfo.numCand; i++ )
  {
    Mv mv[3] = { affineAMVPInfo.mvCandLT[i], affineAMVPInfo.mvCandRT[i], affineAMVPInfo.mvCandLB[i] };

    Distortion uiTmpCost = xGetAffineTemplateCost( pu, origBuf, predBuf, mv, i, AMVP_MAX_NUM_CANDS, eRefPicList, iRefIdx );

    if ( uiBestCost > uiTmpCost )
    {
      uiBestCost = uiTmpCost;
      bestMvLT = affineAMVPInfo.mvCandLT[i];
      bestMvRT = affineAMVPInfo.mvCandRT[i];
      bestMvLB = affineAMVPInfo.mvCandLB[i];
      iBestIdx  = i;
      *puiDistBiP = uiTmpCost;
    }
  }

  // Setting Best MVP
  acMvPred[0] = bestMvLT;
  acMvPred[1] = bestMvRT;
  acMvPred[2] = bestMvLB;

  pu.mvpIdx[eRefPicList] = iBestIdx;
  pu.mvpNum[eRefPicList] = affineAMVPInfo.numCand;

  DTRACE( g_trace_ctx, D_COMMON, "#estAffi=%d \n", affineAMVPInfo.numCand );
}

void InterSearch::xCopyAffineAMVPInfo (AffineAMVPInfo& src, AffineAMVPInfo& dst)
{
  dst.numCand = src.numCand;
  DTRACE( g_trace_ctx, D_COMMON, " (%d) #copyAffi=%d \n", DTRACE_GET_COUNTER( g_trace_ctx, D_COMMON ), src.numCand );
  ::memcpy( dst.mvCandLT, src.mvCandLT, sizeof(Mv)*src.numCand );
  ::memcpy( dst.mvCandRT, src.mvCandRT, sizeof(Mv)*src.numCand );
  ::memcpy( dst.mvCandLB, src.mvCandLB, sizeof(Mv)*src.numCand );
}


/**
* \brief Generate half-sample interpolated block
*
* \param pattern Reference picture ROI
* \param biPred    Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingH(CPelBuf* pattern, bool useAltHpelIf)
{
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  int intStride = width + 1;
  int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
#if IF_12TAP
  int filterSize = NTAPS_LUMA(0);
#else
  int filterSize = NTAPS_LUMA;
#endif 
  int halfFilterSize = (filterSize>>1);
  const Pel *srcPtr = pattern->buf - halfFilterSize*srcStride - 1;

  const ChromaFormat chFmt = m_currChromaFormat;

  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[0][0], intStride, width + 1, height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, 0, false, useAltHpelIf);
  if (!m_skipFracME)
  {
    m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, m_filteredBlockTmp[2][0], intStride, width + 1, height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng, 0, false, useAltHpelIf);
  }

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, 0, false, useAltHpelIf);
  if (m_skipFracME)
  {
    return;
  }

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, 0, false, useAltHpelIf);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, 0, false, useAltHpelIf);

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width + 1, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng, 0, false, useAltHpelIf);
}





/**
* \brief Generate quarter-sample interpolated blocks
*
* \param pattern    Reference picture ROI
* \param halfPelRef Half-pel mv
* \param biPred     Flag indicating whether block is for biprediction
*/
void InterSearch::xExtDIFUpSamplingQ( CPelBuf* pattern, Mv halfPelRef)
{
  const ClpRng& clpRng = m_lumaClpRng;
  int width      = pattern->width;
  int height     = pattern->height;
  int srcStride  = pattern->stride;

  Pel const* srcPtr;
  int intStride = width + 1;
  int dstStride = width + 1;
  Pel *intPtr;
  Pel *dstPtr;
#if IF_12TAP
  int filterSize = NTAPS_LUMA(0);
#else
  int filterSize = NTAPS_LUMA;
#endif

  int halfFilterSize = (filterSize>>1);

  int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize-1;

  const ChromaFormat chFmt = m_currChromaFormat;

  // Horizontal filter 1/4
  srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng);

  // Horizontal filter 3/4
  srcPtr = pattern->buf - halfFilterSize*srcStride - 1;
  intPtr = m_filteredBlockTmp[3][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
  m_if.filterHor(COMPONENT_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3 << MV_FRACTIONAL_BITS_DIFF, false, chFmt, clpRng);

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[1][1][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize-1) * intStride;
  dstPtr = m_filteredBlock[3][1][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][1][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][3][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1][0];
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3][0];
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[1][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[3][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0][0];
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0][0];
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[1][3][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[3][3][0];
  m_if.filterVer(COMPONENT_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false, true, chFmt, clpRng);
}





//! set wp tables
void InterSearch::setWpScalingDistParam( int iRefIdx, RefPicList eRefPicListCur, Slice *pcSlice )
{
  if ( iRefIdx<0 )
  {
    m_cDistParam.applyWeight = false;
    return;
  }

  WPScalingParam  *wp0 , *wp1;

  m_cDistParam.applyWeight = ( pcSlice->getSliceType()==P_SLICE && pcSlice->testWeightPred() ) || ( pcSlice->getSliceType()==B_SLICE && pcSlice->testWeightBiPred() ) ;

  if ( !m_cDistParam.applyWeight )
  {
    return;
  }

  int iRefIdx0 = ( eRefPicListCur == REF_PIC_LIST_0 ) ? iRefIdx : (-1);
  int iRefIdx1 = ( eRefPicListCur == REF_PIC_LIST_1 ) ? iRefIdx : (-1);

  getWpScaling( pcSlice, iRefIdx0, iRefIdx1, wp0 , wp1 );

  if ( iRefIdx0 < 0 )
  {
    wp0 = NULL;
  }
  if ( iRefIdx1 < 0 )
  {
    wp1 = NULL;
  }

  m_cDistParam.wpCur  = NULL;

  if ( eRefPicListCur == REF_PIC_LIST_0 )
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

void InterSearch::xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const ComponentID &compID)
{
  const UnitArea& currArea    = partitioner.currArea();
  const TransformUnit &currTU = *cs.getTU(isLuma(partitioner.chType) ? currArea.lumaPos() : currArea.chromaPos(), partitioner.chType);
  const CodingUnit &cu        = *currTU.cu;
  const unsigned currDepth    = partitioner.currTrDepth;

  const bool bSubdiv          = currDepth != currTU.depth;

  if (compID == MAX_NUM_TBLOCKS)  // we are not processing a channel, instead we always recurse and code the CBFs
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ) )
    {
      CHECK( !bSubdiv, "Not performing the implicit TU split" );
    }
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ) )
    {
      CHECK( !bSubdiv, "Not performing the implicit TU split - sbt" );
    }
    else
    {
      CHECK( bSubdiv, "transformsplit not supported" );
    }

    CHECK(CU::isIntra(cu), "Inter search provided with intra CU");

    if( cu.chromaFormat != CHROMA_400
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      && (!cu.isSepTree() || isChroma(partitioner.chType))
#else
      && (!CS::isDualITree(cs) || isChroma(partitioner.chType))
#endif
      )
    {
      {
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth );
          if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cb], currDepth );
        }
        {
          const bool  chroma_cbf = TU::getCbfAtDepth( currTU, COMPONENT_Cr, currDepth );
          if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
          m_CABACEstimator->cbf_comp( cs, chroma_cbf, currArea.blocks[COMPONENT_Cr], currDepth, TU::getCbfAtDepth( currTU, COMPONENT_Cb, currDepth ) );
        }
      }
    }

    if( !bSubdiv && !( cu.sbtInfo && currTU.noResidual )
      && !isChroma(partitioner.chType)
      )
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currArea.Y(), currDepth );
    }
  }

  if (!bSubdiv)
  {
    if (compID != MAX_NUM_TBLOCKS) // we have already coded the CBFs, so now we code coefficients
    {
      if( currArea.blocks[compID].valid() )
      {
#if JVET_AF0073_INTER_CCP_MERGE
        if ( compID == COMPONENT_Cr )
        {
          m_CABACEstimator->interCcpMerge(currTU);
        }
#endif
#if JVET_AE0059_INTER_CCCM
        if ( compID == COMPONENT_Cr )
        {
          m_CABACEstimator->interCccm(currTU);
        }
#endif
#if JVET_AL0181_ASBT  
        if (compID == COMPONENT_Y && TU::getCbf(currTU, compID))
        {
          m_CABACEstimator->interAsbt(currTU);
        }
#endif
        if( compID == COMPONENT_Cr )
        {
          const int cbfMask = ( TU::getCbf( currTU, COMPONENT_Cb ) ? 2 : 0) + ( TU::getCbf( currTU, COMPONENT_Cr ) ? 1 : 0 );
          m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
        }
        if( TU::getCbf( currTU, compID ) )
        {
          m_CABACEstimator->residual_coding( currTU, compID );
        }
      }
    }
  }
  else
  {
    if( compID == MAX_NUM_TBLOCKS || TU::getCbfAtDepth( currTU, compID, currDepth ) )
    {
      if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
        , false, false
#endif
      ) )
      {
        partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
      }
      else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs 
#if JVET_AI0087_BTCUS_RESTRICTION
        , false, false
#endif
      ) )
      {
        partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
      }
      else
        THROW( "Implicit TU split not available!" );

      do
      {
        xEncodeInterResidualQT( cs, partitioner, compID );
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
    }
  }
}

void InterSearch::calcMinDistSbt( CodingStructure &cs, const CodingUnit& cu, const uint8_t sbtAllowed )
{
  if( !sbtAllowed )
  {
    m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
    for( int comp = 0; comp < getNumberValidTBlocks( *cs.pcv ); comp++ )
    {
      const ComponentID compID = ComponentID( comp );
      CPelBuf pred = cs.getPredBuf( compID );
      CPelBuf org  = cs.getOrgBuf( compID );
      m_estMinDistSbt[NUMBER_SBT_MODE] += m_pcRdCost->getDistPart( org, pred, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }
    return;
  }

  //SBT fast algorithm 2.1 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
  //                         if this cost is larger than the best cost, no need to try a specific SBT mode
  int cuWidth  = cu.lwidth();
  int cuHeight = cu.lheight();
#if JVET_AJ0260_SBT_CORNER_MODE
  const int numPartX = cuWidth  >= 8 ? 4 : 1;
  const int numPartY = cuHeight >= 8 ? 4 : 1;

  std::vector<std::vector<Distortion>> dist( numPartY, std::vector<Distortion>( numPartX, 0 ));
#else
  int numPartX = cuWidth  >= 16 ? 4 : ( cuWidth  == 4 ? 1 : 2 );
  int numPartY = cuHeight >= 16 ? 4 : ( cuHeight == 4 ? 1 : 2 );
  Distortion dist[4][4];
  memset( dist, 0, sizeof( Distortion ) * 16 );
#endif

  for( uint32_t c = 0; c < getNumberValidTBlocks( *cs.pcv ); c++ )
  {
    const ComponentID compID   = ComponentID( c );
    const CompArea&   compArea = cu.blocks[compID];
    const CPelBuf orgPel  = cs.getOrgBuf( compArea );
    const CPelBuf predPel = cs.getPredBuf( compArea );
    int lengthX = compArea.width / numPartX;
    int lengthY = compArea.height / numPartY;
    int strideOrg  = orgPel.stride;
    int stridePred = predPel.stride;
    uint32_t   uiShift = DISTORTION_PRECISION_ADJUSTMENT( ( *cs.sps.getBitDepth( toChannelType( compID ) ) - 8 ) << 1 );
    Intermediate_Int iTemp;

    //calc distY of 16 sub parts
    for( int j = 0; j < numPartY; j++ )
    {
      for( int i = 0; i < numPartX; i++ )
      {
        int posX = i * lengthX;
        int posY = j * lengthY;
        const Pel* ptrOrg  = orgPel.bufAt( posX, posY );
        const Pel* ptrPred = predPel.bufAt( posX, posY );
        Distortion uiSum = 0;
        for( int n = 0; n < lengthY; n++ )
        {
          for( int m = 0; m < lengthX; m++ )
          {
            iTemp = ptrOrg[m] - ptrPred[m];
            uiSum += Distortion( ( iTemp * iTemp ) >> uiShift );
          }
          ptrOrg += strideOrg;
          ptrPred += stridePred;
        }
        if( isChroma( compID ) )
        {
          uiSum = (Distortion)( uiSum * m_pcRdCost->getChromaWeight() );
        }
        dist[j][i] += uiSum;
      }
    }
  }

  //SSE of a CU
  m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
  for( int j = 0; j < numPartY; j++ )
  {
    for( int i = 0; i < numPartX; i++ )
    {
      m_estMinDistSbt[NUMBER_SBT_MODE] += dist[j][i];
    }
  }
  //init per-mode dist
  for( int i = SBT_VER_H0; i < NUMBER_SBT_MODE; i++ )
  {
    m_estMinDistSbt[i] = std::numeric_limits<uint64_t>::max();
  }

  //SBT fast algorithm 1: not try SBT if the residual is too small to compensate bits for encoding residual info
  uint64_t minNonZeroResiFracBits = 12 << SCALE_BITS;
  if( m_pcRdCost->calcRdCost( 0, m_estMinDistSbt[NUMBER_SBT_MODE] ) < m_pcRdCost->calcRdCost( minNonZeroResiFracBits, 0 ) )
  {
    m_skipSbtAll = true;
    return;
  }

  //derive estimated minDist of SBT = zero-residual part distortion + non-zero residual part distortion / 16
  int shift = 5;
  Distortion distResiPart = 0, distNoResiPart = 0;

  if( CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) )
  {
    int offsetResiPart = 0;
    int offsetNoResiPart = numPartX / 2;
    distResiPart = distNoResiPart = 0;

    CHECKD( numPartX < 2, "numPartX should be 2 or more" );

    for( int j = 0; j < numPartY; j++ )
    {
      for( int i = 0; i < numPartX / 2; i++ )
      {
        distResiPart   += dist[j][i + offsetResiPart];
        distNoResiPart += dist[j][i + offsetNoResiPart];
      }
    }
    m_estMinDistSbt[SBT_VER_H0] = ( distResiPart >> shift ) + distNoResiPart;
    m_estMinDistSbt[SBT_VER_H1] = ( distNoResiPart >> shift ) + distResiPart;
  }

  if( CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed ) )
  {
    int offsetResiPart = 0;
    int offsetNoResiPart = numPartY / 2;

    CHECKD( numPartY < 2, "numPartY should be 2 or more" );
    
    distResiPart = distNoResiPart = 0;
    for( int j = 0; j < numPartY / 2; j++ )
    {
      for( int i = 0; i < numPartX; i++ )
      {
        distResiPart   += dist[j + offsetResiPart][i];
        distNoResiPart += dist[j + offsetNoResiPart][i];
      }
    }
    m_estMinDistSbt[SBT_HOR_H0] = ( distResiPart >> shift ) + distNoResiPart;
    m_estMinDistSbt[SBT_HOR_H1] = ( distNoResiPart >> shift ) + distResiPart;
  }

  if( CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) )
  {
    CHECKD( numPartX != 4, "numPartX should be 4");

    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q1] = 0;
    for( int j = 0; j < numPartY; j++ )
    {
      m_estMinDistSbt[SBT_VER_Q0] += dist[j][0] + ( ( dist[j][1] + dist[j][2] + dist[j][3] ) << shift );
      m_estMinDistSbt[SBT_VER_Q1] += dist[j][3] + ( ( dist[j][0] + dist[j][1] + dist[j][2] ) << shift );
    }
    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q0] >> shift;
    m_estMinDistSbt[SBT_VER_Q1] = m_estMinDistSbt[SBT_VER_Q1] >> shift;
  }

  if( CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed ) )
  {
    CHECKD( numPartY != 4, "numPartY should be 4" );

    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q1] = 0;
    for( int i = 0; i < numPartX; i++ )
    {
      m_estMinDistSbt[SBT_HOR_Q0] += dist[0][i] + ( ( dist[1][i] + dist[2][i] + dist[3][i] ) << shift );
      m_estMinDistSbt[SBT_HOR_Q1] += dist[3][i] + ( ( dist[0][i] + dist[1][i] + dist[2][i] ) << shift );
    }
    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q0] >> shift;
    m_estMinDistSbt[SBT_HOR_Q1] = m_estMinDistSbt[SBT_HOR_Q1] >> shift;
  }

#if JVET_AJ0260_SBT_CORNER_MODE
  if( CU::targetSbtAllowed( SBT_QUAD, sbtAllowed ) )
  {
    m_estMinDistSbt[ SBT_Q0 ] = m_estMinDistSbt[ SBT_Q1 ] = m_estMinDistSbt[ SBT_Q2 ] = m_estMinDistSbt[ SBT_Q3 ] = 0;

    for( int j = 0; j < numPartY / 2; j++ )
    {
      for( int i = 0; i < numPartX / 2; i++ )
      {
        m_estMinDistSbt[ SBT_Q0 ] += dist[ j ][ i ];
        m_estMinDistSbt[ SBT_Q1 ] += dist[ j ][ i + ( numPartX >> 1 ) ];
        m_estMinDistSbt[ SBT_Q2 ] += dist[ j + ( numPartY >> 1 ) ][ i ];
        m_estMinDistSbt[ SBT_Q3 ] += dist[ j + ( numPartY >> 1 ) ][ i + ( numPartX >> 1 ) ];
      }
    }
    m_estMinDistSbt[ SBT_Q0 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_Q0 ] ) + ( m_estMinDistSbt[ SBT_Q0 ] >> shift );
    m_estMinDistSbt[ SBT_Q1 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_Q1 ] ) + ( m_estMinDistSbt[ SBT_Q1 ] >> shift );
    m_estMinDistSbt[ SBT_Q2 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_Q2 ] ) + ( m_estMinDistSbt[ SBT_Q2 ] >> shift );
    m_estMinDistSbt[ SBT_Q3 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_Q3 ] ) + ( m_estMinDistSbt[ SBT_Q3 ] >> shift );
  }

  if( CU::targetSbtAllowed( SBT_QUARTER, sbtAllowed ) )
  {
    m_estMinDistSbt[ SBT_QT0 ] = m_estMinDistSbt[ SBT_QT1 ] = m_estMinDistSbt[ SBT_QT2 ] = m_estMinDistSbt[ SBT_QT3 ] = 0;

    for( int j = 0; j < numPartY / 4; j++ )
    {
      for( int i = 0; i < numPartX / 4; i++ )
      {
        m_estMinDistSbt[ SBT_QT0 ] += dist[ j ][ i ];
        m_estMinDistSbt[ SBT_QT1 ] += dist[ j ][ i + ( 3 * numPartX >> 2 ) ];
        m_estMinDistSbt[ SBT_QT2 ] += dist[ j + ( 3 * numPartY >> 2 ) ][ i ];
        m_estMinDistSbt[ SBT_QT3 ] += dist[ j + ( 3 * numPartY >> 2 ) ][ i + ( 3 * numPartX >> 2 ) ];
      }
    }
    m_estMinDistSbt[ SBT_QT0 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_QT0 ] ) + ( m_estMinDistSbt[ SBT_QT0 ] >> shift );
    m_estMinDistSbt[ SBT_QT1 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_QT1 ] ) + ( m_estMinDistSbt[ SBT_QT1 ] >> shift );
    m_estMinDistSbt[ SBT_QT2 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_QT2 ] ) + ( m_estMinDistSbt[ SBT_QT2 ] >> shift );
    m_estMinDistSbt[ SBT_QT3 ] = ( m_estMinDistSbt[ NUMBER_SBT_MODE ] - m_estMinDistSbt[ SBT_QT3 ] ) + ( m_estMinDistSbt[ SBT_QT3 ] >> shift );
  }
#endif

  //SBT fast algorithm 5: try N SBT modes with the lowest distortion
  Distortion temp[NUMBER_SBT_MODE];
  memcpy( temp, m_estMinDistSbt, sizeof( Distortion ) * NUMBER_SBT_MODE );
  memset( m_sbtRdoOrder, 255, NUMBER_SBT_MODE );
  int startIdx = 0, numRDO;
  numRDO = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  numRDO = std::min( ( numRDO << 1 ), SBT_NUM_RDO );
  for( int i = startIdx; i < startIdx + numRDO; i++ )
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();
    for( int n = SBT_VER_H0; n <= SBT_HOR_H1; n++ )
    {
      if( temp[n] < minDist )
      {
        minDist = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = std::numeric_limits<uint64_t>::max();
  }

  startIdx += numRDO;
#if JVET_AJ0260_SBT_CORNER_MODE
  numRDO = ( ( CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed ) ) << 1 ) + ( CU::targetSbtAllowed( SBT_QUAD, sbtAllowed ) << 2 );
  numRDO = std::min( numRDO, SBT_NUM_RDO );
#else
  numRDO = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  numRDO = std::min( ( numRDO << 1 ), SBT_NUM_RDO );
#endif

  for( int i = startIdx; i < startIdx + numRDO; i++ )
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();

#if JVET_AJ0260_SBT_CORNER_MODE
    for( int n = SBT_VER_Q0; n < NUMBER_SBT_MODE; n++ )
    {
      if( n >= SBT_QT0 && n <= SBT_QT3 )
      {
        continue;
      }
#else
    for( int n = SBT_VER_Q0; n <= SBT_HOR_Q1; n++ )
    {
#endif
      if( temp[n] < minDist )
      {
        minDist = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = std::numeric_limits<uint64_t>::max();
  }

#if JVET_AJ0260_SBT_CORNER_MODE
  startIdx += numRDO;
  numRDO = CU::targetSbtAllowed( SBT_QUARTER, sbtAllowed );
  numRDO = std::min( ( numRDO << 2 ), SBT_NUM_RDO );

  for( int i = startIdx; i < startIdx + numRDO; i++ )
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();
    for( int n = SBT_QT0; n <= SBT_QT3; n++ )
    {
      if( temp[ n ] < minDist )
      {
        minDist = temp[ n ];
        m_sbtRdoOrder[ i ] = n;
      }
    }
    temp[ m_sbtRdoOrder[ i ] ] = std::numeric_limits<uint64_t>::max();
  }
#endif
}

uint8_t InterSearch::skipSbtByRDCost( int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos, double bestCost, Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff )
{
  int sbtMode = CU::getSbtMode( sbtIdx, sbtPos );

  //SBT fast algorithm 2.2 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and coded part (assuming distorted can be reduced to 1/16);
  //                         if this cost is larger than the best cost, no need to try a specific SBT mode
  if( m_pcRdCost->calcRdCost( 11 << SCALE_BITS, m_estMinDistSbt[sbtMode] ) > bestCost )
  {
    return 0; //early skip type 0
  }

  if( costSbtOff != MAX_DOUBLE )
  {
    if( !rootCbfSbtOff )
    {
      //SBT fast algorithm 3: skip SBT when the residual is too small (estCost is more accurate than fast algorithm 1, counting PU mode bits)
      uint64_t minNonZeroResiFracBits = 10 << SCALE_BITS;
      Distortion distResiPart;
      if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_HOR_HALF )
      {
        distResiPart = (Distortion)( ( ( m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode] ) * 9 ) >> 4 );
      }
      else
      {
        distResiPart = (Distortion)( ( ( m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode] ) * 3 ) >> 3 );
      }

      double estCost = ( costSbtOff - m_pcRdCost->calcRdCost( 0 << SCALE_BITS, distSbtOff ) ) + m_pcRdCost->calcRdCost( minNonZeroResiFracBits, m_estMinDistSbt[sbtMode] + distResiPart );
      if( estCost > costSbtOff )
      {
        return 1;
      }
      if( estCost > bestCost )
      {
        return 2;
      }
    }
    else
    {
      //SBT fast algorithm 4: skip SBT when an estimated RD cost is larger than the bestCost
      double weight = sbtMode > SBT_HOR_H1 ? 0.4 : 0.6;
      double estCost = ( ( costSbtOff - m_pcRdCost->calcRdCost( 0 << SCALE_BITS, distSbtOff ) ) * weight ) + m_pcRdCost->calcRdCost( 0 << SCALE_BITS, m_estMinDistSbt[sbtMode] );
      if( estCost > bestCost )
      {
        return 3;
      }
    }
  }
  return MAX_UCHAR;
}
#if JVET_AA0133_INTER_MTS_OPT
bool InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist /*= NULL*/
  , const bool luma, const bool chroma
  , PelUnitBuf* orgResi
)
#else
void InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist /*= NULL*/
  , const bool luma, const bool chroma
  , PelUnitBuf* orgResi
)
#endif
{
  const UnitArea& currArea = partitioner.currArea();
  const SPS &sps           = *cs.sps;
  m_pcRdCost->setChromaFormat(sps.getChromaFormatIdc());

  const uint32_t numValidComp  = getNumberValidComponents( sps.getChromaFormatIdc() );
  const uint32_t numTBlocks    = getNumberValidTBlocks   ( *cs.pcv );
  const CodingUnit &cu = *cs.getCU(partitioner.chType);
  const unsigned currDepth = partitioner.currTrDepth;
  const bool colorTransFlag = cs.cus[0]->colorTransform;

  bool bCheckFull  = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  );
  if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs 
#if JVET_AI0087_BTCUS_RESTRICTION
    , false, false
#endif
  ) )
  {
    bCheckFull = false;
  }
  bool bCheckSplit = !bCheckFull;

  // get temporary data
  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }

  Distortion uiSingleDist         = 0;
  Distortion uiSingleDistComp [3] = { 0, 0, 0 };
  uint64_t   uiSingleFracBits[3] = { 0, 0, 0 };

  const TempCtx ctxStart  ( m_ctxCache, m_CABACEstimator->getCtx() );

  if (bCheckFull)
  {
    TransformUnit &tu = csFull->addTU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);
    tu.depth          = currDepth;
    for (int i = 0; i<MAX_NUM_TBLOCKS; i++) tu.mtsIdx[i] = MTS_DCT2_DCT2;
    tu.checkTuNoResidual( partitioner.currPartIdx() );
    Position tuPos = tu.Y();
    tuPos.relativeTo(cu.Y());
    const UnitArea relativeUnitArea(tu.chromaFormat, Area(tuPos, tu.Y().size()));

    const Slice           &slice = *cs.slice;
    if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && !(CS::isDualITree(cs) && slice.isIntra() && tu.cu->predMode == MODE_IBC))
    {
#if LMCS_CHROMA_CALC_CU
      const CompArea      &areaY = tu.cu->blocks[COMPONENT_Y];
#else
      const CompArea      &areaY = tu.blocks[COMPONENT_Y];
#endif
      int adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }

#if JVET_S0234_ACT_CRS_FIX
    PelUnitBuf colorTransResidual = m_colorTransResiBuf[1].getBuf(relativeUnitArea);
    if (colorTransFlag)
    {
      csFull->getResiBuf(currArea).copyFrom(cs.getOrgResiBuf(currArea));
      if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[COMPONENT_Cb].width*tu.blocks[COMPONENT_Cr].height > 4)
      {
        csFull->getResiBuf(currArea).bufs[1].scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
        csFull->getResiBuf(currArea).bufs[2].scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
      }
      csFull->getResiBuf(currArea).colorSpaceConvert(colorTransResidual, true, cu.cs->slice->clpRng(COMPONENT_Y));
    }
#endif
    double minCost            [MAX_NUM_TBLOCKS];
#if JVET_AF0073_INTER_CCP_MERGE
    double minCostInterCcpIdx[3][MAX_NUM_TBLOCKS];
    for (uint32_t i = 0; i < numTBlocks; i++)
    {
      minCostInterCcpIdx[0][i] = MAX_DOUBLE;
      minCostInterCcpIdx[1][i] = MAX_DOUBLE;
      minCostInterCcpIdx[2][i] = MAX_DOUBLE;
    }
#endif

    m_CABACEstimator->resetBits();

    memset(m_pTempPel, 0, sizeof(Pel) * tu.Y().area()); // not necessary needed for inside of recursion (only at the beginning)

    for (uint32_t i = 0; i < numTBlocks; i++)
    {
      minCost[i] = MAX_DOUBLE;
    }

    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv     = cs.pcv;
    saveCS.picture = cs.picture;
#if JVET_Z0118_GDR
    saveCS.m_pt = cs.m_pt;
#endif
    saveCS.area.repositionTo(currArea);
    saveCS.clearTUs();
    TransformUnit & bestTU = saveCS.addTU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);
#if JVET_AL0181_ASBT
    PelBuf  lumaResi = PelBuf(m_interAsbt[0], tu.blocks[COMPONENT_Y]);
    PelBuf  lumaPred = PelBuf(m_interAsbt[1], tu.blocks[COMPONENT_Y]);
    PelBuf  dwnSplResi = PelBuf(m_interAsbt[2], tu.blocks[COMPONENT_Y]);
    PelBuf  chromaPred[2] = { PelBuf(m_interAsbt[4], tu.blocks[COMPONENT_Cb]), \
                              PelBuf(m_interAsbt[5], tu.blocks[COMPONENT_Cr])};
    PelBuf  chromaResi[2] = { PelBuf(m_interAsbt[6], tu.blocks[COMPONENT_Cb]), \
                              PelBuf(m_interAsbt[7], tu.blocks[COMPONENT_Cr])};

    PelBuf  chromaDwnSplResi = PelBuf(m_interAsbt[8], tu.blocks[COMPONENT_Cr]);
    tu.initBlockRes(tu.blocksResidual, MAX_NUM_COMPONENT);
    bestTU.initBlockRes(bestTU.blocksResidual, MAX_NUM_COMPONENT);
    tu.initBlockRes(tu.blocksResForDbf, MAX_NUM_COMPONENT);
    bestTU.initBlockRes(bestTU.blocksResForDbf, MAX_NUM_COMPONENT);
#endif

#if JVET_AE0059_INTER_CCCM || JVET_AF0073_INTER_CCP_MERGE
    CodingStructure &saveCS2 = *m_pSaveCS[1];
    saveCS2.pcv              = cs.pcv;
    saveCS2.picture          = cs.picture;
    saveCS2.area.repositionTo(currArea);
    saveCS2.clearTUs();
    TransformUnit &interCccmTU = saveCS2.addTU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);
    PelBuf         interCccmResiBuf[3];      // temporary buffer to store transformed and quantized residual
    PelBuf         interCccmPredBuf[3];      // temporary buffers to store luma-to-chroma prediction
    PelBuf         interCccmOrgResiBuf[3];   // temporary buffer to store residual for interCccm predictions
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      interCccmResiBuf[i] = PelBuf(m_interCccmStorage[i], tu.blocks[ComponentID(i)]);
    }
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      interCccmPredBuf[i] = PelBuf(m_interCccmStorage[i + 3], tu.blocks[ComponentID(i)]);
    }
    for (int i = 0; i < MAX_NUM_COMPONENT; i++)
    {
      interCccmOrgResiBuf[i] = PelBuf(m_interCccmStorage[i + 6], tu.blocks[ComponentID(i)]);
    }
    PelBuf       lumaRecoBuf(m_interCccmStorage[9], tu.blocks[COMPONENT_Y]);
    PelBuf       lumaPredBuf(m_interCccmStorage[10], tu.blocks[COMPONENT_Y]);
    const bool   interCccmRdSearch = luma && chroma && !colorTransFlag && CU::interCccmSearchAllowed(*tu.cu);
    double       bestCost = MAX_DOUBLE;
    double       tempMinCost[3] = { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE };
    Distortion   tempUiSingleDistComp[3] = { 0, 0, 0 };
    uint64_t     tempUiSingleFracBits[3] = { 0, 0, 0 };
    bool         interCccmOk = false;
#if JVET_AF0073_INTER_CCP_MERGE
    PelBuf         interCcpMergePredBuf[2];      // temporary buffers to store luma-to-chroma prediction
    PelBuf         interCcpMergeOrgResiBuf[2];   // temporary buffer to store residual for interCcpMerge predictions
    for (int i = 1; i < MAX_NUM_COMPONENT; i++)
    {
      interCcpMergePredBuf[i-1] = PelBuf(m_interCcpMergeStorage[i-1], tu.blocks[ComponentID(i)]);
    }
    for (int i = 1; i < MAX_NUM_COMPONENT; i++)
    {
      interCcpMergeOrgResiBuf[i-1] = PelBuf(m_interCcpMergeStorage[i+1], tu.blocks[ComponentID(i)]);
    }
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    bool interCcpMergeRdSearch = luma && chroma && !colorTransFlag && CU::interCcpMergeSearchAllowed(*tu.cu);
    if (cu.slice->getSPS()->getUseInterCcpMergeZeroLumaCbf() && interCcpMergeRdSearch && m_pcEncCfg->getInterCcpMergeZeroLumaCbfFastMode())
    {
      if (tu.cu->blocks[COMPONENT_Cb].area() < 16 || tu.cu->blocks[COMPONENT_Cb].area() > 1024)
      {
        interCcpMergeRdSearch = false;
      }
    }
#else
    const bool   interCcpMergeRdSearch = luma && chroma && !colorTransFlag && CU::interCcpMergeSearchAllowed(*tu.cu);
#endif
    bool         interCcpMergeOk = false;
    bool         lumaRecoReady = false;
    bool skipInterCccm2 = false;

    for (int interCccm = 0; interCccm < 3; interCccm++)
    {
      if (interCccm == 2 && skipInterCccm2)
      {
        continue;
      }
      if ((!interCccmRdSearch) && interCccm == 1)
      {
        continue;
      }
      if ((!interCcpMergeRdSearch) && interCccm == 2)
      {
        continue;
      }
      tu.interCccm = (interCccm == 1) ? 1 : 0;
      tu.interCcpMerge = (interCccm == 2) ? 1 : 0;

      for (uint32_t i = ((tu.interCccm || tu.interCcpMerge) ? 1 : 0); i < numTBlocks; i++)
      {
        minCost[i] = MAX_DOUBLE;
      }
      if (interCccm == 1)
      {
        minCostInterCcpIdx[1][0] = minCostInterCcpIdx[0][0];
      }
      if (interCccm == 2)
      {
        minCostInterCcpIdx[2][0] = minCostInterCcpIdx[0][0];
      }
      for (uint32_t c = ((tu.interCccm || tu.interCcpMerge) ? 1 : 0); c < numTBlocks; c++)
#else
    for (int interCccm = 0; interCccm < (interCccmRdSearch ? 2 : 1); interCccm++)
    {
      tu.interCccm = interCccm;
      for (uint32_t i = (tu.interCccm ? 1 : 0); i < numTBlocks; i++)
      {
        minCost[i] = MAX_DOUBLE;
      }
      for (uint32_t c = (tu.interCccm ? 1 : 0); c < numTBlocks; c++)
#endif
#else
    for (uint32_t c = 0; c < numTBlocks; c++)
#endif
    {
      const ComponentID compID    = ComponentID(c);
      if( compID == COMPONENT_Y && !luma )
      {
        continue;
      }
      if( compID != COMPONENT_Y && !chroma )
      {
        continue;
      }
      const CompArea&   compArea  = tu.blocks[compID];
      const int channelBitDepth   = sps.getBitDepth(toChannelType(compID));

      if( !tu.blocks[compID].valid() )
      {
        continue;
      }

#if JVET_AE0059_INTER_CCCM
      if (tu.interCccm && compID == COMPONENT_Cr)
      {
        if (!interCccmOk) // determined on previous loop iteration (i.e., COMPONENT_Cb)
        {
          break;
        }
#if JVET_AF0073_INTER_CCP_MERGE
        if (tu.cu->cs->slice->getSPS()->getUseInterCcpMerge() && ((minCostInterCcpIdx[1][0] + minCostInterCcpIdx[1][1]) > bestCost))
        {
          if (m_pcEncCfg->getInterCcpMergeFastMode() == 1)
          {
            minCostInterCcpIdx[1][2] = minCost[2] = minCostInterCcpIdx[1][1];
          }
          break;
        }
#endif
      }
#endif

#if JVET_AF0073_INTER_CCP_MERGE
      if (tu.interCcpMerge && compID == COMPONENT_Cr)
      {
        if (!interCcpMergeOk) // determined on previous loop iteration (i.e., COMPONENT_Cb)
        {
          break;
        }
        if (tu.cu->cs->slice->getSPS()->getUseInterCcpMerge() && ((minCostInterCcpIdx[2][0] + minCostInterCcpIdx[2][1]) > bestCost))
        {
          if (m_pcEncCfg->getInterCcpMergeFastMode() == 1)
          {
            minCostInterCcpIdx[2][2] = minCost[2] = minCostInterCcpIdx[2][1];
          }
          break;
        }
      }
#endif

#if JVET_AL0181_ASBT
      bool earlyTermBreak = false;
#if JVET_AA0133_INTER_MTS_OPT
      bool skipRemainingMTS = false;
      bool skipMTSPass = false;
      int  countSkipMTSLoop = 0;
#endif
      bool asbtAllowed = false;
      bool asbtAllowedW = false;
      bool asbtAllowedH = false;
      if (isLuma(compID))
      {
        asbtAllowed = TU::isInterAsbt(tu) && !cu.mtsFlag && !cu.lfnstFlag && !tu.noResidual;
        asbtAllowedW = asbtAllowed && TU::isInterAsbtW(tu);
        asbtAllowedH = asbtAllowed && TU::isInterAsbtH(tu);
      }
      else
      {
        asbtAllowed = tu.asbtDecimH[COMPONENT_Y] || tu.asbtDecimW[COMPONENT_Y];
        int maxDecim = std::max(tu.asbtDecimH[COMPONENT_Y], tu.asbtDecimW[COMPONENT_Y]);
        tu.asbtDecimH[compID] = 0;
        tu.asbtDecimW[compID] = 0;

        if (asbtAllowed)
        {
          asbtAllowedW = ((compArea.size().width >> maxDecim) >= 2);
          asbtAllowedH = ((compArea.size().height >> maxDecim) >= 2);
        }
      }
      CompArea compAreaDwn = compArea;
      int nbH = 1, nbW = 1;
      if (asbtAllowedH && compID == COMPONENT_Y)
      {
        if (tu.blocks[0].height > MIN_ASBT_DECIM_4_SIZE)
        {
          nbH = 3;
        }
        else
        {
          nbH = 2;
        }
      }
      if (asbtAllowedW && compID == COMPONENT_Y)
      {
        if (tu.blocks[0].width > MIN_ASBT_DECIM_4_SIZE)
        {
          nbW = 3;
        }
        else
        {
          nbW = 2;
        }
      }
      int dir = 0;
      Distortion distTu = MAX_UINT64;

      int rdoAsbtorder[3] = { 0, 2, 1 };
      if (((asbtAllowedW || asbtAllowedH) ? std::max(nbW, nbH) : 1) == 2)
      {
        rdoAsbtorder[1] = 1;
      }
      bool isEstimated = false;
      for (int asbtindex = 0; (asbtindex < ((asbtAllowedW || asbtAllowedH) ? std::max(nbW, nbH) : 1)); asbtindex++)
      {
        if (asbtindex == 2 && isEstimated)
        {
          if ((!bestTU.cbf[0]) && (bestTU.asbtDecimH[COMPONENT_Y] || bestTU.asbtDecimW[COMPONENT_Y]))
          {
            continue;
          }
        }
        int asbtMode = rdoAsbtorder[asbtindex];
        if (compID == COMPONENT_Y)
        {
          tu.blocksResidual[COMPONENT_Y].size() = compArea.size();
          compAreaDwn.size() = compArea.size();
          tu.asbtDecimH[COMPONENT_Y] = 0;
          tu.asbtDecimW[COMPONENT_Y] = 0;
          tu.blocksResForDbf[COMPONENT_Y].size() = compArea.size();
          tu.blocksResForDbf[COMPONENT_Y].pos() = tu.blocks[COMPONENT_Y].pos();
        }

        if (compID == COMPONENT_Y && asbtMode)
        {
          tu.blocksResidual[COMPONENT_Y].size() = compArea.size();
          tu.blocksResForDbf[COMPONENT_Y].size() = compArea.size();

          if (nbW > asbtMode && nbH > asbtMode)
          {
            dir = 0;
            dir = asbtDownSamplingPred(tu, compID, lumaPred, lumaResi, dwnSplResi, asbtMode);
            if (dir == 0)
            {
              compAreaDwn.size().width = compArea.size().width >> asbtMode;
              compAreaDwn.size().height = compArea.size().height;
              tu.asbtDecimH[COMPONENT_Y] = 0;
              tu.asbtDecimW[COMPONENT_Y] = asbtMode;
              tu.blocksResForDbf[COMPONENT_Y].width = compArea.size().width >> asbtMode;
              tu.fillBlocksResForDbfPos(COMPONENT_Y);
            }
            else if (dir == 1)
            {
              compAreaDwn.size().width = compArea.size().width;
              compAreaDwn.size().height = compArea.size().height >> asbtMode;
              tu.asbtDecimH[COMPONENT_Y] = asbtMode;
              tu.asbtDecimW[COMPONENT_Y] = 0;
              tu.blocksResForDbf[COMPONENT_Y].height = compArea.size().height >> asbtMode;
              tu.fillBlocksResForDbfPos(COMPONENT_Y);
            }
            tu.blocksResidual[COMPONENT_Y].size() = compAreaDwn.size();
            tu.blocksResForDbf[COMPONENT_Y].size() = compAreaDwn.size();
          }
          else if (nbW > asbtMode)
          {
            dir = 0;
            compAreaDwn.size().width = compArea.size().width >> asbtMode;
            compAreaDwn.size().height = compArea.size().height;
            tu.blocksResidual[COMPONENT_Y].size() = compAreaDwn.size();
            asbtDownSampling(tu, compID, lumaPred, lumaResi, dwnSplResi);
            tu.asbtDecimH[COMPONENT_Y] = 0;
            tu.asbtDecimW[COMPONENT_Y] = asbtMode;
            tu.blocksResForDbf[COMPONENT_Y].size() = compAreaDwn.size();
            tu.fillBlocksResForDbfPos(COMPONENT_Y);
          }
          else if (nbH > asbtMode)
          {
            dir = 1;
            compAreaDwn.size().width = compArea.size().width;
            compAreaDwn.size().height = compArea.size().height >> asbtMode;
            tu.blocksResidual[COMPONENT_Y].size() = compAreaDwn.size();
            asbtDownSamplingH(tu, compID, lumaPred, lumaResi, dwnSplResi);
            tu.asbtDecimH[COMPONENT_Y] = asbtMode;
            tu.asbtDecimW[COMPONENT_Y] = 0;
            tu.blocksResForDbf[COMPONENT_Y].size() = compAreaDwn.size();
            tu.fillBlocksResForDbfPos(COMPONENT_Y);
          }
          if (tu.asbtDecimH[COMPONENT_Y] || tu.asbtDecimW[COMPONENT_Y])
          {
            Distortion distTuAsbt = getDistortionFullBlock(tu, dwnSplResi);
            if (distTu > 2 * distTuAsbt)
            {
              asbtAllowedH = 0;
              asbtAllowedW = 0;
              continue;
            }
            uint64_t minNonZeroResiFracBits = 12 << SCALE_BITS;
            if (m_pcRdCost->calcRdCost(0, distTuAsbt) < m_pcRdCost->calcRdCost(minNonZeroResiFracBits, 0))
            {
              continue;
            }
            isEstimated = true;
          }
        }
#endif
#if JVET_AA0133_INTER_MTS_OPT
      const bool mtsAllowed = CU::isMTSAllowed(*tu.cu, compID) && cu.mtsFlag;
#if JVET_AG0061_INTER_LFNST_NSPT
      const bool tsAllowed = TU::isTSAllowed(tu, compID) && ((isLuma(compID) && !cu.mtsFlag) || (isChroma(compID) && m_pcEncCfg->getUseChromaTS())) && !cu.lfnstFlag;
#else
      const bool tsAllowed = TU::isTSAllowed(tu, compID) && ((isLuma(compID) && !cu.mtsFlag) || (isChroma(compID) && m_pcEncCfg->getUseChromaTS()));
#endif
#else
      const bool tsAllowed = TU::isTSAllowed(tu, compID) && (isLuma(compID) || (isChroma(compID) && m_pcEncCfg->getUseChromaTS()));
      const bool mtsAllowed = CU::isMTSAllowed( *tu.cu, compID );
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_SBT_LFNST
      const bool lfnstAllowed = CU::isLfnstAllowed(*tu.cu, compID) && cu.lfnstFlag && !tu.noResidual;
#else
      const bool lfnstAllowed = CU::isLfnstAllowed(*tu.cu, compID) && cu.lfnstFlag;
#endif
#endif

#if JVET_AG0061_INTER_LFNST_NSPT
      uint8_t nNumTransformCands = 0;
#else 
      uint8_t nNumTransformCands = 1 + ( tsAllowed ? 1 : 0 ) + ( mtsAllowed ? 4 : 0 ); // DCT + TS + 4 MTS = 6 tests
#if JVET_AA0133_INTER_MTS_OPT
      if (cu.mtsFlag && compID == COMPONENT_Y)
      {
        nNumTransformCands = (mtsAllowed ? 4 : 0);
      }
#endif
#endif
      std::vector<TrMode> trModes;
#if TU_256
#if JVET_AI0050_SBT_LFNST
      if (tu.idx != cu.firstTU->idx && ((cu.sbtInfo && tu.noResidual) || !cu.sbtInfo))
#else
      if (tu.idx != cu.firstTU->idx)
#endif
      {
        trModes.push_back( TrMode( cu.firstTU->mtsIdx[compID], true ) );
        nNumTransformCands = 1;
      }
      else
      {
#endif
        if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
        {
          nNumTransformCands = 0;
        }
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_SBT_LFNST
        else if (!((cu.mtsFlag || (cu.lfnstFlag && !tu.noResidual)) && compID == COMPONENT_Y))
#else
        else if (!((cu.mtsFlag || cu.lfnstFlag) && compID == COMPONENT_Y))
#endif
#elif JVET_AA0133_INTER_MTS_OPT
        else if (!(cu.mtsFlag && compID == COMPONENT_Y))
#else
        else
#endif
        {
          trModes.push_back(TrMode(0, true)); //DCT2
          nNumTransformCands = 1;
        }
#if JVET_AA0133_INTER_MTS_OPT
        else
        {
          nNumTransformCands = 0;
        }
#endif
        // for a SBT-no-residual TU, the RDO process should be called once, in order to get the RD cost
        if (tsAllowed && !tu.noResidual)
        {
          trModes.push_back(TrMode(1, true));
          nNumTransformCands++;
        }

#if APPLY_SBT_SL_ON_MTS && !JVET_AG0061_INTER_LFNST_NSPT
        // skip MTS if DCT2 is the best
        if (mtsAllowed && (!tu.cu->slice->getSPS()->getUseSBT() || CU::getSbtIdx(m_histBestSbt) != SBT_OFF_DCT))
#else
        if (mtsAllowed)
#endif
        {
          for (int i = 2; i < 6; i++)
          {
#if APPLY_SBT_SL_ON_MTS
            // skip the non-best Mts mode
            if (!tu.cu->slice->getSPS()->getUseSBT() || (m_histBestMtsIdx == MAX_UCHAR || m_histBestMtsIdx == i))
            {
#endif
              trModes.push_back(TrMode(i, true));
              nNumTransformCands++;
#if APPLY_SBT_SL_ON_MTS
            }
#endif
          }
        }
#if JVET_AG0061_INTER_LFNST_NSPT
        if (lfnstAllowed)
        {
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
          int kerCandNum = ( cu.cs->sps->getUseLFNSTExt()
                        || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) ) ) ? 3 : 2;
          int numMode = cu.cs->sps->getUseInterMTSS() && tu.cu->geoFlag ? ( kerCandNum << 1 ) : kerCandNum;
#else
          int numMode = cu.cs->sps->getUseInterMTSS() && tu.cu->geoFlag ? 6 : 3;
#endif
          for (int i = NUM_TRAFO_MODES_MTS; i < NUM_TRAFO_MODES_MTS + numMode; i++)   // 3 or 6 lfnst
#else
#if AHG7_LN_TOOLOFF_CFG
          int kerCandNum = ( cu.cs->sps->getUseLFNSTExt()
                        || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) ) ) ? 3 : 2;
          for( int i = NUM_TRAFO_MODES_MTS; i < NUM_TRAFO_MODES_MTS + kerCandNum; i++ )   // 3 lfnst
#else
          for (int i = NUM_TRAFO_MODES_MTS; i < NUM_TRAFO_MODES_MTS + 3; i++)   // 3 lfnst
#endif
#endif
          {
            {
              trModes.push_back(TrMode(i, true));
              nNumTransformCands++;
            }
          }
        }
#endif
#if TU_256
      }
#endif

      if (colorTransFlag && (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless()))
      {
        m_pcTrQuant->lambdaAdjustColorTrans(true);
#if JVET_S0234_ACT_CRS_FIX
        if (isChroma(compID) && slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width*tu.blocks[compID].height > 4)
        {
          int cResScaleInv = tu.getChromaAdj();
          m_pcRdCost->lambdaAdjustColorTrans(true, compID, true, &cResScaleInv);
        }
        else
#endif
        m_pcRdCost->lambdaAdjustColorTrans(true, compID);
      }
#if JVET_AA0133_INTER_MTS_OPT && !JVET_AL0181_ASBT
      bool skipRemainingMTS = false;
      bool skipMTSPass = false;
      int  countSkipMTSLoop = 0;
#endif
#if JVET_AE0059_INTER_CCCM
      if (tu.interCccm && compID == COMPONENT_Cb)
      {
        if (!TU::getCbf(tu, COMPONENT_Y))
        {
#if JVET_AL0181_ASBT
          earlyTermBreak = true;
#endif
          break;
        }
#if JVET_AF0073_INTER_CCP_MERGE
        if (!lumaRecoReady)
        {
#endif
        lumaPredBuf.copyFrom(csFull->getPredBuf(tu.blocks[COMPONENT_Y]));
        if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
        {
          lumaPredBuf.rspSignal(m_pcReshape->getFwdLUT());
        }
#if JVET_AG0145_ADAPTIVE_CLIPPING
        ClpRng clpRng = tu.cs->slice->clpRng(COMPONENT_Y);
        if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
          clpRng.min = fwdLUT[tu.cu->cs->slice->getLumaPelMin()];
          clpRng.max = fwdLUT[tu.cu->cs->slice->getLumaPelMax()];
        }
        else
        {
          clpRng.min = tu.cu->cs->slice->getLumaPelMin();
          clpRng.max = tu.cu->cs->slice->getLumaPelMax();
        }
        lumaRecoBuf.reconstruct(lumaPredBuf, csFull->getResiBuf(tu.blocks[COMPONENT_Y]), clpRng);
#else
        lumaRecoBuf.reconstruct(lumaPredBuf, csFull->getResiBuf(tu.blocks[COMPONENT_Y]), cs.slice->clpRng(COMPONENT_Y));
#endif
        if (CU::isIBC(cu) && cu.rribcFlipType)
        {
          lumaRecoBuf.flipSignal(cu.rribcFlipType == 1);
        }
#if JVET_AF0073_INTER_CCP_MERGE
          lumaRecoReady = true;
        }
#endif

        PelBuf bufCb = csFull->getPredBuf( tu.blocks[COMPONENT_Cb] );
        PelBuf bufCr = csFull->getPredBuf( tu.blocks[COMPONENT_Cr] );

        const bool valid = deriveInterCccmPrediction( &tu, lumaPredBuf, lumaRecoBuf, bufCb, bufCr, interCccmPredBuf[COMPONENT_Cb], interCccmPredBuf[COMPONENT_Cr] );

        if( valid )
        {
          interCccmOk = true;
        }
        else
        {
#if JVET_AL0181_ASBT
          earlyTermBreak = true;
#endif
          break;
        }

        for (int y = 0; y < compArea.height; y++)
        {
          for (int x = 0; x < compArea.width; x++)
          {
            interCccmOrgResiBuf[COMPONENT_Cb].at(x, y) = csFull->getOrgBuf(tu.blocks[COMPONENT_Cb]).at(x, y) - interCccmPredBuf[COMPONENT_Cb].at(x, y);
            interCccmOrgResiBuf[COMPONENT_Cr].at(x, y) = csFull->getOrgBuf(tu.blocks[COMPONENT_Cr]).at(x, y) - interCccmPredBuf[COMPONENT_Cr].at(x, y);
          }
        }
      }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
      if (tu.interCcpMerge && compID == COMPONENT_Cb)
      {
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
        if ((!tu.cs->slice->getSPS()->getUseInterCcpMergeZeroLumaCbf() && !TU::getCbf(tu, COMPONENT_Y)) ||
           (tu.cs->slice->getSPS()->getUseInterCcpMergeZeroLumaCbf() && TU::getCbf(tu, COMPONENT_Y) && tu.cu->blocks[COMPONENT_Cb].area() > 1024 && !tu.cu->slice->getCheckLDB()))
#else
        if (!TU::getCbf(tu, COMPONENT_Y))
#endif
        {
#if JVET_AL0181_ASBT
          earlyTermBreak = true;
#endif
          break;
        }
        if (!lumaRecoReady)
        {
          lumaPredBuf.copyFrom(csFull->getPredBuf(tu.blocks[COMPONENT_Y]));
          if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
          {
            lumaPredBuf.rspSignal(m_pcReshape->getFwdLUT());
          }
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0 && JVET_AG0145_ADAPTIVE_CLIPPING
          ClpRng clpRng = tu.cs->slice->clpRng(COMPONENT_Y);
          if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
            clpRng.min = fwdLUT[tu.cu->cs->slice->getLumaPelMin()];
            clpRng.max = fwdLUT[tu.cu->cs->slice->getLumaPelMax()];
          }
          else
          {
            clpRng.min = tu.cu->cs->slice->getLumaPelMin();
            clpRng.max = tu.cu->cs->slice->getLumaPelMax();
          }
          lumaRecoBuf.reconstruct(lumaPredBuf, csFull->getResiBuf(tu.blocks[COMPONENT_Y]), clpRng);
#else
          lumaRecoBuf.reconstruct(lumaPredBuf, csFull->getResiBuf(tu.blocks[COMPONENT_Y]), cs.slice->clpRng(COMPONENT_Y));
#endif
          if (CU::isIBC(cu) && cu.rribcFlipType)
          {
            lumaRecoBuf.flipSignal(cu.rribcFlipType == 1);
          }
          lumaRecoReady = true;
        }

        PelBuf bufCb = csFull->getPredBuf( tu.blocks[COMPONENT_Cb] );
        PelBuf bufCr = csFull->getPredBuf( tu.blocks[COMPONENT_Cr] );

        if (m_isInterCcpModelReady == false)
        {
          m_pcIntraPred->xAddOnTheFlyCalcCCPCands4InterBlk(*tu.cu->firstPU, tu.blocks[COMPONENT_Cb], m_interCcpMergeList, m_validNum);
          m_isInterCcpModelReady = true;
        }
        const bool valid = deriveInterCcpMergePrediction(&tu, lumaRecoBuf, bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], m_interCcpMergeList, m_validNum);

        if( valid )
        {
          interCcpMergeOk = true;
        }
        else
        {
          break;
        }

        for (int y = 0; y < compArea.height; y++)
        {
          for (int x = 0; x < compArea.width; x++)
          {
            interCcpMergeOrgResiBuf[0].at(x, y) = csFull->getOrgBuf(tu.blocks[COMPONENT_Cb]).at(x, y) - interCcpMergePredBuf[0].at(x, y);
            interCcpMergeOrgResiBuf[1].at(x, y) = csFull->getOrgBuf(tu.blocks[COMPONENT_Cr]).at(x, y) - interCcpMergePredBuf[1].at(x, y);
          }
        }
      }
#endif
#if JVET_AL0181_ASBT
      if (isChroma(compID))
      {
        if (tu.asbtDecimH[COMPONENT_Y] || tu.asbtDecimW[COMPONENT_Y])
        {
          if (isChroma(compID) && tu.interCccm)
          {
            chromaPred[compID - 1].copyFrom(interCccmPredBuf[compID]);
            chromaResi[compID - 1].copyFrom(interCccmOrgResiBuf[compID]);
          }
          else if (isChroma(compID) && tu.interCcpMerge)
          {
            chromaPred[compID - 1].copyFrom(interCcpMergePredBuf[compID - 1]);
            chromaResi[compID - 1].copyFrom(interCcpMergeOrgResiBuf[compID - 1]);
          }
          else
          {
            chromaPred[compID - 1].copyFrom(csFull->getPredBuf(compArea));
            chromaResi[compID - 1].copyFrom(cs.getOrgResiBuf(compArea));
          }

          int maxDecim = std::max(tu.asbtDecimH[COMPONENT_Y], tu.asbtDecimW[COMPONENT_Y]);
          if (asbtAllowedH && asbtAllowedW)
          {
            tu.blocksResidual[compID].size() = compArea.size();
            tu.blocksResForDbf[compID].size() = compArea.size();

            dir = 0;
            dir = asbtDownSamplingPred(tu, compID, chromaPred[compID - 1], chromaResi[compID - 1], chromaDwnSplResi, maxDecim);

            if (dir == 0)
            {
              compAreaDwn.size().width = compArea.size().width >> maxDecim;
              compAreaDwn.size().height = compArea.size().height;
              tu.asbtDecimH[compID] = 0;
              tu.asbtDecimW[compID] = maxDecim;
            }
            else
            {
              compAreaDwn.size().width = compArea.size().width;
              compAreaDwn.size().height = compArea.size().height >> maxDecim;
              tu.asbtDecimH[compID] = maxDecim;
              tu.asbtDecimW[compID] = 0;
            }
            tu.blocksResidual[compID].size() = compAreaDwn.size();
            tu.blocksResForDbf[compID].size() = compAreaDwn.size();
            tu.fillBlocksResForDbfPos(compID);
          }
          else if (asbtAllowedW)
          {
            compAreaDwn.size().width = compArea.size().width >> maxDecim;
            compAreaDwn.size().height = compArea.size().height;
            tu.blocksResidual[compID].size() = compAreaDwn.size();
            asbtDownSampling(tu, compID, chromaPred[compID - 1], chromaResi[compID - 1], chromaDwnSplResi);
            tu.asbtDecimH[compID] = 0;
            tu.asbtDecimW[compID] = maxDecim;
            tu.blocksResForDbf[compID].size() = compAreaDwn.size();
            tu.fillBlocksResForDbfPos(compID);
          }
          else
          {
            compAreaDwn.size().width = compArea.size().width;
            compAreaDwn.size().height = compArea.size().height >> maxDecim;
            tu.blocksResidual[compID].size() = compAreaDwn.size();
            asbtDownSamplingH(tu, compID, chromaPred[compID - 1], chromaResi[compID - 1], chromaDwnSplResi);
            tu.asbtDecimH[compID] = maxDecim;
            tu.asbtDecimW[compID] = 0;
            tu.blocksResForDbf[compID].size() = compAreaDwn.size();
            tu.fillBlocksResForDbfPos(compID);
          }
        }
      }
#endif

      const int numTransformCandidates = nNumTransformCands;
      for( int transformMode = 0; transformMode < numTransformCandidates; transformMode++ )
      {
#if JVET_AL0181_ASBT
        const bool isFirstMode = (transformMode == 0 && asbtMode == 0);
#else
          const bool isFirstMode  = transformMode == 0;
#endif

          // copy the original residual into the residual buffer
#if JVET_S0234_ACT_CRS_FIX
          if (colorTransFlag)
          {
            csFull->getResiBuf(compArea).copyFrom(colorTransResidual.bufs[compID]);
          }
#if JVET_AL0181_ASBT
          else if (tu.asbtDecimH[compID] || tu.asbtDecimW[compID])
          {
            if (isLuma(compID))
            {
              csFull->getResiBuf(compAreaDwn).copyFrom(dwnSplResi.subBuf(0, 0, compAreaDwn.size().width, compAreaDwn.size().height));
            }
            else
            {
              csFull->getResiBuf(compAreaDwn).copyFrom(chromaDwnSplResi.subBuf(0, 0, compAreaDwn.size().width, compAreaDwn.size().height));
            }
          }
#endif
#if JVET_AE0059_INTER_CCCM
          else if (isChroma(compID) && tu.interCccm)
          {
            csFull->getResiBuf(compArea).copyFrom(interCccmOrgResiBuf[compID]);
          }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
          else if (isChroma(compID) && tu.interCcpMerge)
          {
            csFull->getResiBuf(compArea).copyFrom(interCcpMergeOrgResiBuf[compID-1]);
          }
#endif
          else
#endif
          csFull->getResiBuf(compArea).copyFrom(cs.getOrgResiBuf(compArea));

          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();

          {
            if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
            {
              if (bestTU.mtsIdx[compID] == MTS_SKIP && m_pcEncCfg->getUseTransformSkipFast())
              {
                continue;
              }
              if (!trModes[transformMode].second)
              {
                continue;
              }
            }
#if JVET_AG0061_INTER_LFNST_NSPT
            tu.mtsIdx[compID]   = trModes[transformMode].first < NUM_TRAFO_MODES_MTS ? trModes[transformMode].first : 0;
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
            int kerCandNum = ( cu.cs->sps->getUseLFNSTExt()
                          || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) ) ) ? 3 : 2;
            int factor = ( trModes[ transformMode ].first - NUM_TRAFO_MODES_MTS ) / kerCandNum;
            tu.lfnstIdx[ compID ] = trModes[ transformMode ].first < NUM_TRAFO_MODES_MTS ? 0 : trModes[ transformMode ].first - NUM_TRAFO_MODES_MTS - kerCandNum * factor + 1;
            tu.lfnstIntra[ compID ] = trModes[ transformMode ].first < ( NUM_TRAFO_MODES_MTS + kerCandNum ) ? 0 : ( trModes[ transformMode ].first < ( NUM_TRAFO_MODES_MTS + ( kerCandNum << 1 ) ) ? 1 : 2 );
#else
            int factor = (trModes[transformMode].first - NUM_TRAFO_MODES_MTS) / 3;
            tu.lfnstIdx[compID] = trModes[transformMode].first < NUM_TRAFO_MODES_MTS ? 0 : trModes[transformMode].first - NUM_TRAFO_MODES_MTS - 3 * factor + 1;
            tu.lfnstIntra[compID] = trModes[transformMode].first < (NUM_TRAFO_MODES_MTS + 3) ? 0 : (trModes[transformMode].first < (NUM_TRAFO_MODES_MTS + 6) ? 1 : 2);
#endif
#else
            tu.lfnstIdx[compID] = trModes[transformMode].first < NUM_TRAFO_MODES_MTS
                                    ? 0
                                    : trModes[transformMode].first - NUM_TRAFO_MODES_MTS + 1;
#endif
#if JVET_AI0050_SBT_LFNST
            if ((compID == COMPONENT_Y && !cu.sbtInfo) || (!tu.noResidual && compID == COMPONENT_Y && cu.sbtInfo))
#else
            if (compID == COMPONENT_Y)
#endif
            {
              tu.cu->lfnstIdx = tu.lfnstIdx[compID];
#if JVET_AI0050_INTER_MTSS
              tu.cu->lfnstIntra = tu.lfnstIntra[compID];
#endif
            }
#else
            tu.mtsIdx[compID] = trModes[transformMode].first;
#endif
          }
#if JVET_AG0061_INTER_LFNST_NSPT
          if (compID == COMPONENT_Y && (cu.mtsFlag || cu.lfnstFlag) && skipRemainingMTS)
          {
            break;
          }
#elif JVET_AA0133_INTER_MTS_OPT
          if (compID == COMPONENT_Y && cu.mtsFlag && skipRemainingMTS)
          {
            break;
          }
#endif
          QpParam cQP(tu, compID);  // note: uses tu.transformSkip[compID]

#if RDOQ_CHROMA_LAMBDA
          m_pcTrQuant->selectLambda(compID);
#endif
          if (slice.getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
          {
            double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)(tu.getChromaAdj());
            m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cRescale*cRescale));
          }
          if ( sps.getJointCbCrEnabledFlag() && isChroma( compID ) && ( tu.cu->cs->slice->getSliceQp() > 18 ) )
          {
            m_pcTrQuant->setLambda( 1.05 * m_pcTrQuant->getLambda() );
          }

          TCoeff     currAbsSum = 0;
          uint64_t   currCompFracBits = 0;
          Distortion currCompDist = 0;
          double     currCompCost = 0;
          uint64_t   nonCoeffFracBits = 0;
          Distortion nonCoeffDist = 0;
          double     nonCoeffCost = 0;

#if JVET_S0234_ACT_CRS_FIX 
          if (!colorTransFlag && slice.getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width*tu.blocks[compID].height > 4)
#else
          if (slice.getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width * tu.blocks[compID].height > 4)
#endif
          {
            PelBuf resiBuf = csFull->getResiBuf(compArea);
            resiBuf.scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(compID));
          }
          if( nNumTransformCands > 1 )
          {
            if( transformMode == 0 )
            {
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_INTER_MTSS
              m_pcTrQuant->transformNxN(tu, compID, cQP, &trModes, m_pcEncCfg->getMTSInterMaxCand() + 6);
#else
              m_pcTrQuant->transformNxN(tu, compID, cQP, &trModes, m_pcEncCfg->getMTSInterMaxCand() + 3);
#endif
              tu.mtsIdx[compID] = trModes[0].first < NUM_TRAFO_MODES_MTS ? trModes[0].first : 0;
#if JVET_AI0050_INTER_MTSS
#if AHG7_LN_TOOLOFF_CFG
              int kerCandNum = ( cu.cs->sps->getUseLFNSTExt()
                            || ( cu.cs->sps->getUseNSPT() && CU::isNSPTAllowed( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) ) ) ? 3 : 2;
              int factor = ( trModes[ 0 ].first - NUM_TRAFO_MODES_MTS ) / kerCandNum;
              tu.lfnstIdx[ compID ] = trModes[ 0 ].first < NUM_TRAFO_MODES_MTS ? 0 : trModes[ 0 ].first - NUM_TRAFO_MODES_MTS - kerCandNum * factor + 1;
              tu.lfnstIntra[ compID ] = trModes[ 0 ].first < ( NUM_TRAFO_MODES_MTS + kerCandNum ) ? 0 : ( trModes[ 0 ].first < ( NUM_TRAFO_MODES_MTS + ( kerCandNum << 1 ) ) ? 1 : 2 );
#else
              int factor = (trModes[0].first - NUM_TRAFO_MODES_MTS) / 3;
              tu.lfnstIdx[compID] = trModes[0].first < NUM_TRAFO_MODES_MTS ? 0 : trModes[0].first - NUM_TRAFO_MODES_MTS - 3 * factor + 1;
              tu.lfnstIntra[compID] = trModes[0].first < (NUM_TRAFO_MODES_MTS + 3) ? 0 : (trModes[0].first < (NUM_TRAFO_MODES_MTS + 6) ? 1 : 2);
#endif
#else
              tu.lfnstIdx[compID] =
                trModes[0].first < NUM_TRAFO_MODES_MTS ? 0 : trModes[0].first - NUM_TRAFO_MODES_MTS + 1;
#endif
#if JVET_AI0050_SBT_LFNST
              if ((compID == COMPONENT_Y && !tu.cu->sbtInfo) || (!tu.noResidual && compID == COMPONENT_Y && tu.cu->sbtInfo))
#else
              if (compID == COMPONENT_Y)
#endif
              {
                tu.cu->lfnstIdx = tu.lfnstIdx[compID];
#if JVET_AI0050_INTER_MTSS
                tu.cu->lfnstIntra = tu.lfnstIntra[compID];
#endif
              }
#else
              m_pcTrQuant->transformNxN(tu, compID, cQP, &trModes, m_pcEncCfg->getMTSInterMaxCand());
              tu.mtsIdx[compID] = trModes[0].first;
#endif
            }
            if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0))
            {
              m_pcTrQuant->transformNxN( tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx(), true );
            }
          }
          else
          {
            m_pcTrQuant->transformNxN( tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx() );
          }

          if (isFirstMode || (currAbsSum == 0))
          {
            const CPelBuf zeroBuf(m_pTempPel, compArea);
#if JVET_S0234_ACT_CRS_FIX
#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
            const CPelBuf orgResi = colorTransFlag ? colorTransResidual.bufs[compID] : ((isChroma(compID) && tu.interCccm) ? interCccmOrgResiBuf[compID] : ((isChroma(compID) && tu.interCcpMerge) ? interCcpMergeOrgResiBuf[compID-1] : csFull->getOrgResiBuf(compArea)));
#else
            const CPelBuf orgResi = colorTransFlag ? colorTransResidual.bufs[compID] : ((isChroma(compID) && tu.interCccm) ? interCccmOrgResiBuf[compID] : csFull->getOrgResiBuf(compArea));
#endif
#else
            const CPelBuf orgResi = colorTransFlag ? colorTransResidual.bufs[compID] : csFull->getOrgResiBuf(compArea);
#endif
#else
            const CPelBuf orgResi = csFull->getOrgResiBuf( compArea );
#endif

            {
              nonCoeffDist = m_pcRdCost->getDistPart( zeroBuf, orgResi, channelBitDepth, compID, DF_SSE ); // initialized with zero residual distortion
            }

            if( !tu.noResidual )
            {
            const bool prevCbf = ( compID == COMPONENT_Cr ? tu.cbf[COMPONENT_Cb] : false );
            m_CABACEstimator->cbf_comp( *csFull, false, compArea, currDepth, prevCbf );

#if JVET_AF0073_INTER_CCP_MERGE
            if ( compID == COMPONENT_Cr )
            {
              m_CABACEstimator->interCcpMerge(tu);
            }
#endif
#if JVET_AE0059_INTER_CCCM
            if ( compID == COMPONENT_Cr )
            {
              m_CABACEstimator->interCccm(tu);
            }
#endif
#if JVET_AL0181_ASBT
            if (compID == COMPONENT_Y && (tu.asbtDecimH[COMPONENT_Y] || tu.asbtDecimW[COMPONENT_Y]))
            {
              continue;
            }

            if (isChroma(compID) && (tu.asbtDecimH[compID] || tu.asbtDecimW[compID]) && (currAbsSum == 0))
            {
              tu.asbtDecimH[compID] = 0;
              tu.asbtDecimW[compID] = 0;
            }
#endif
            }

            nonCoeffFracBits = m_CABACEstimator->getEstFracBits();

#if JVET_AL0181_ASBT
            if ((asbtMode == 0) && (asbtAllowedH || asbtAllowedW) && (compID == COMPONENT_Y))
            {
              distTu = nonCoeffDist;
              uint64_t minNonZeroResiFracBits = 12 << SCALE_BITS;
              if (m_pcRdCost->calcRdCost(0, nonCoeffDist) < m_pcRdCost->calcRdCost(minNonZeroResiFracBits, 0))
              {
                asbtAllowedH = 0;
                asbtAllowedW = 0;
              }
              else
              {
                lumaPred.copyFrom(csFull->getPredBuf(tu.blocks[COMPONENT_Y]));
                initEnerPred(tu, csFull->getPredBuf(tu.blocks[COMPONENT_Y]));
                lumaResi.copyFrom(cs.getOrgResiBuf(compArea));
              }
            }
#endif
#if WCG_EXT
            if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
            {
              nonCoeffCost   = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, false);
            }
            else
#endif
              if (cs.slice->getSPS()->getUseColorTrans())
              {
                nonCoeffCost = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, false);
              }
              else
              {
                nonCoeffCost = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist);
              }
          }

#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
          if ((puiZeroDist != NULL) && isFirstMode && (!tu.interCccm && !tu.interCcpMerge))
#else
          if ((puiZeroDist != NULL) && isFirstMode && !tu.interCccm)
#endif
#else
          if ((puiZeroDist != NULL) && isFirstMode)
#endif
          {
            *puiZeroDist += nonCoeffDist; // initialized with zero residual distortion
          }
#if JVET_AG0061_INTER_LFNST_NSPT
          if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0&&cu.lfnstIdx == 0)
#else
          if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0)
#endif
          {
            currAbsSum = 0;
          }

#if JVET_Z0118_GDR
          cs.updateReconMotIPM(tu.blocks[compID], cs.getPredBuf(tu.blocks[compID]));
#else
          cs.picture->getRecoBuf(tu.blocks[compID]).copyFrom(cs.getPredBuf(tu.blocks[compID]));
#endif
          if (currAbsSum > 0) //if non-zero coefficients are present, a residual needs to be derived for further prediction
          {
            if (isFirstMode)
            {
              m_CABACEstimator->getCtx() = ctxStart;
              m_CABACEstimator->resetBits();
            }

            const bool prevCbf = ( compID == COMPONENT_Cr ? tu.cbf[COMPONENT_Cb] : false );
            m_CABACEstimator->cbf_comp( *csFull, true, compArea, currDepth, prevCbf );
#if JVET_AF0073_INTER_CCP_MERGE
            if ( compID == COMPONENT_Cr )
            {
              m_CABACEstimator->interCcpMerge(tu);
            }
#endif
#if JVET_AE0059_INTER_CCCM
            if ( compID == COMPONENT_Cr )
            {
              m_CABACEstimator->interCccm(tu);
            }
#endif
#if JVET_AL0181_ASBT 
            if (isLuma(compID) && (asbtAllowedW || asbtAllowedH))
            {
              m_CABACEstimator->interAsbt(tu);
            }
#endif
            if( compID == COMPONENT_Cr )
            {
              const int cbfMask = ( tu.cbf[COMPONENT_Cb] ? 2 : 0 ) + 1;
              m_CABACEstimator->joint_cb_cr( tu, cbfMask );
            }

#if JVET_AG0061_INTER_LFNST_NSPT
            bool bMtsLfnstVialateScanPos = false;
#endif
#if SIGN_PREDICTION
            if ( sps.getNumPredSigns() > 0)
            {
#if JVET_Y0141_SIGN_PRED_IMPROVE
              bool doSignPrediction = true;
#if JVET_AG0061_INTER_LFNST_NSPT
              if ((isLuma(compID) && tu.mtsIdx[COMPONENT_Y] > MTS_SKIP) || (isLuma(compID) && cu.lfnstIdx))
#else
              if (isLuma(compID) && tu.mtsIdx[COMPONENT_Y] > MTS_SKIP)
#endif
              {
                bool signHiding = slice.getSignDataHidingEnabledFlag();
                CoeffCodingContext  cctx(tu, COMPONENT_Y, signHiding);
                int scanPosLast = -1;
                TCoeff* coeff = tu.getCoeffs(compID).buf;
                for (int scanPos = cctx.maxNumCoeff() - 1; scanPos >= 0; scanPos--)
                {
                  unsigned blkPos = cctx.blockPos(scanPos);
                  if (coeff[blkPos])
                  {
                    scanPosLast = scanPos;
                    break;
                  }
                }
                if (scanPosLast < 1)
                {
                  doSignPrediction = false;
#if JVET_AG0061_INTER_LFNST_NSPT
                  bMtsLfnstVialateScanPos = true;
#endif
                }
              }
#if JVET_AL0181_ASBT
              if (isChroma(compID) && (tu.asbtDecimW[COMPONENT_Y] || tu.asbtDecimH[COMPONENT_Y]))
              {
                int maxDecim = (tu.asbtDecimW[COMPONENT_Y] > tu.asbtDecimH[COMPONENT_Y]) ? tu.asbtDecimW[COMPONENT_Y] : tu.asbtDecimH[COMPONENT_Y];
                if ((tu.blocks[compID].width >> maxDecim) >= 2 && (tu.blocks[compID].height >> maxDecim) >= 2)
                {
                  if ((tu.blocks[compID].width >> maxDecim) < 4 || (tu.blocks[compID].height >> maxDecim) < 4)
                  {
                    doSignPrediction = false;
                  }
                }
              }
#endif
              if (doSignPrediction)
              {
#endif
                bool reshapeChroma = slice.getPicHeader()->getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width*tu.blocks[compID].height > 4;
#if JVET_Y0065_GPM_INTRA
                if (isLuma(compID) && slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC(*tu.cu))
#else
                if (isLuma(compID) && slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !tu.cu->firstPU->ciipFlag && !CU::isIBC(*tu.cu))
#endif
                {
                  cs.getPredBuf(tu.blocks[compID]).rspSignal(m_pcReshape->getFwdLUT());
                }
#if JVET_AE0059_INTER_CCCM
                PelBuf tmpPredSignPred(m_interCccmStorage[11], compArea);
                if (tu.interCccm && isChroma(compID))
                {
                  tmpPredSignPred.copyFrom(tu.cs->getPredBuf(tu.blocks[compID]));
                  tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(interCccmPredBuf[compID]);
                }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
                PelBuf tmpPredSignPredCcpMerge(m_interCcpMergeStorage[4], compArea);
                if (tu.interCcpMerge && isChroma(compID))
                {
                  tmpPredSignPredCcpMerge.copyFrom(tu.cs->getPredBuf(tu.blocks[compID]));
                  tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(interCcpMergePredBuf[compID-1]);
                }
#endif
#if JVET_AL0181_ASBT
                if ((tu.asbtDecimW[compID] || tu.asbtDecimH[compID]))
                {
                  Pel predResiBorder[2 * SIGN_PRED_MAX_BS];
                  if (tu.asbtDecimW[compID] && !tu.asbtDecimH[compID])
                  {
                    asbtBorderResi(tu, tu.cs->picture->getRecoBuf(tu.blocks[compID]), tu.cs->getPredBuf(tu.blocks[compID]), compID,
                      predResiBorder, (tu.cs->sps->getBitDepth(toChannelType(compID)) - 1));

                  }
                  else if (!tu.asbtDecimW[compID] && tu.asbtDecimH[compID])
                  {
                    asbtBorderResiH(tu, tu.cs->picture->getRecoBuf(tu.blocks[compID]), tu.cs->getPredBuf(tu.blocks[compID]), compID,
                      predResiBorder, (tu.cs->sps->getBitDepth(toChannelType(compID)) - 1));
                  }
                  m_pcTrQuant->predCoeffSigns(tu, compID, reshapeChroma, predResiBorder);
                }
                else
                {
                  m_pcTrQuant->predCoeffSigns(tu, compID, reshapeChroma);
                }
#else
                m_pcTrQuant->predCoeffSigns(tu, compID, reshapeChroma);
#endif
#if JVET_AE0059_INTER_CCCM
                if (tu.interCccm && isChroma(compID))
                {
                  tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(tmpPredSignPred);
                }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
                if (tu.interCcpMerge && isChroma(compID))
                {
                  tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(tmpPredSignPredCcpMerge);
                }

#endif
#if JVET_Y0065_GPM_INTRA
                if (isLuma(compID) && slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC(*tu.cu))
#else
                if (isLuma(compID) && slice.getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !tu.cu->firstPU->ciipFlag && !CU::isIBC(*tu.cu))
#endif
                {
                  cs.getPredBuf(tu.blocks[COMPONENT_Y]).copyFrom(cs.picture->getRecoBuf(tu.blocks[COMPONENT_Y]));
                }
#if JVET_Y0141_SIGN_PRED_IMPROVE
              }
#endif
            }
#if JVET_AG0061_INTER_LFNST_NSPT
            else if (isLuma(compID) && tu.mtsIdx[COMPONENT_Y] != MTS_SKIP && cu.lfnstIdx != 0 )
            {
              CoeffCodingContext cctx(tu, COMPONENT_Y, false);
              int                scanPosLast = -1;
              TCoeff *           coeff       = tu.getCoeffs(compID).buf;
              for (int scanPos = cctx.maxNumCoeff() - 1; scanPos >= 0; scanPos--)
              {
                unsigned blkPos = cctx.blockPos(scanPos);
                if (coeff[blkPos])
                {
                  scanPosLast = scanPos;
                  break;
                }
              }
              if (scanPosLast < 1)
              {
                bMtsLfnstVialateScanPos = true;
              }
            }
#endif
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
            if (bMtsLfnstVialateScanPos)
            {
              currCompCost = MAX_DOUBLE;
            }
            else
            {
#endif
              CUCtx cuCtx;
              cuCtx.isDQPCoded         = true;
              cuCtx.isChromaQpAdjCoded = true;
              m_CABACEstimator->residual_coding(tu, compID, &cuCtx);

#if JVET_AG0061_INTER_LFNST_NSPT
              if (isLuma(compID) && CU::isInter(cu))
              {
                m_CABACEstimator->residual_lfnst_mode(cu, cuCtx);
              }
              if (isLuma(compID))
#endif
#if JVET_AL0181_ASBT 
              {
                if (!(tu.asbtDecimH[compID] || tu.asbtDecimW[compID]))
                {
                  m_CABACEstimator->mts_idx(cu, &cuCtx);
                }
              }
#else
                m_CABACEstimator->mts_idx(cu, &cuCtx);
#endif

              if (compID == COMPONENT_Y && tu.mtsIdx[compID] > MTS_SKIP && !cuCtx.mtsLastScanPos)
              {
                currCompCost = MAX_DOUBLE;
              }
              else
              {
                currCompFracBits = m_CABACEstimator->getEstFracBits();

                PelBuf resiBuf = csFull->getResiBuf(compArea);
#if JVET_S0234_ACT_CRS_FIX
#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
                CPelBuf orgResiBuf = colorTransFlag ? colorTransResidual.bufs[compID] : ((isChroma(compID) && tu.interCccm) ? interCccmOrgResiBuf[compID] : ((isChroma(compID) && tu.interCcpMerge) ? interCcpMergeOrgResiBuf[compID-1] : csFull->getOrgResiBuf(compArea)));
#else
                CPelBuf orgResiBuf = colorTransFlag ? colorTransResidual.bufs[compID] : ((isChroma(compID) && tu.interCccm) ? interCccmOrgResiBuf[compID] : csFull->getOrgResiBuf(compArea));
#endif
#else
                CPelBuf orgResiBuf = colorTransFlag ? colorTransResidual.bufs[compID] : csFull->getOrgResiBuf(compArea);
#endif
#else
                CPelBuf orgResiBuf = csFull->getOrgResiBuf(compArea);
#endif

                m_pcTrQuant->invTransformNxN(tu, compID, resiBuf, cQP);
#if JVET_AL0181_ASBT
                if (tu.asbtDecimW[compID] && !tu.asbtDecimH[compID])
                {
                  upsampleOneBlock(tu, resiBuf, compID);
                }
                if (!tu.asbtDecimW[compID] && tu.asbtDecimH[compID])
                {
                  upsampleOneBlockH(tu, resiBuf, compID);
                }
#endif
#if JVET_S0234_ACT_CRS_FIX
                if (!colorTransFlag && slice.getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width*tu.blocks[compID].height > 4)
#else
                if (slice.getLmcsEnabledFlag() && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[compID].width * tu.blocks[compID].height > 4)
#endif
                {
                  resiBuf.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
                }

#if JVET_V0094_BILATERAL_FILTER
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                bool applyBIF = cs.pps->getUseBIF() && isLuma(compID) && m_bilateralFilter->getApplyBIF(tu, compID);
#else
                bool isInter = (cu.predMode == MODE_INTER) ? true : false;
                // getCbf() is going to be 1 since currAbsSum > 0 here, according to the if-statement a couple of lines up.
                bool applyBIF = cs.pps->getUseBIF() && isLuma(compID) && tu.cu->qp > 17 && 128 > std::max(tu.lumaSize().width, tu.lumaSize().height) && (!isInter || 32 > std::min(tu.lumaSize().width, tu.lumaSize().height));
#endif
                if (applyBIF)
                {
                  CompArea tmpArea1(compID, tu.chromaFormat, Position(0, 0), Size(resiBuf.width, resiBuf.height));
                  PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
                  tmpRecLuma.copyFrom(resiBuf);

                  const CPelBuf    predBuf     = csFull->getPredBuf(compArea);
                  PelBuf           recIPredBuf = csFull->slice->getPic()->getRecoBuf(compArea);
                  std::vector<Pel> invLUT;
                  m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpRecLuma, predBuf, tmpRecLuma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, false, false, &invLUT );

                  currCompDist = m_pcRdCost->getDistPart(orgResiBuf, tmpRecLuma, channelBitDepth, compID, DF_SSE);
                }
                else
#endif
                {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
                  if (isChroma(compID))
                  {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                    bool applyChromaBIF = cs.pps->getUseChromaBIF() && m_bilateralFilter->getApplyBIF(tu, compID);
#else
                    bool applyChromaBIF = cs.pps->getUseChromaBIF() && isChroma(compID) && (tu.cu->qp > 17);
#endif
                    if (applyChromaBIF)
                    {
                      // chroma and bilateral
                      CompArea tmpArea1(compID, tu.chromaFormat, Position(0, 0), Size(resiBuf.width, resiBuf.height));
                      PelBuf   tmpRecChroma = m_tmpStorageLCU.getBuf(tmpArea1);
                      tmpRecChroma.copyFrom(resiBuf);

#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
                      const CPelBuf predBuf = tu.interCccm ? interCccmPredBuf[compID] : (tu.interCcpMerge ? interCcpMergePredBuf[compID-1] : csFull->getPredBuf(compArea));
#else
                      const CPelBuf predBuf = tu.interCccm ? interCccmPredBuf[compID] : csFull->getPredBuf(compArea);
#endif
#else
                      const CPelBuf predBuf = csFull->getPredBuf(compArea);
#endif
                      PelBuf recIPredBuf = csFull->slice->getPic()->getRecoBuf(compArea);
                      m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpRecChroma, predBuf, tmpRecChroma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, false );
                      currCompDist = m_pcRdCost->getDistPart(orgResiBuf, tmpRecChroma, channelBitDepth, compID, DF_SSE);
                    }
                    else
                    {   // chroma but not bilateral
                      currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DF_SSE);
                    }
                  }
                  else
                  {   // luma but not bilateral
                    currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DF_SSE);
                  }
#else
                currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DF_SSE);
#endif
                }

#if WCG_EXT
                currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist, false);
#else
              currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist);
#endif
              }
#if JVET_AG0061_INTER_LFNST_NSPT
            }
#endif
          }
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_SBT_LFNST
#if JVET_AL0181_ASBT
          else if ((cu.mtsFlag && compID == COMPONENT_Y) || (transformMode > 0) || (cu.lfnstFlag && !tu.noResidual && compID == COMPONENT_Y) || asbtMode)
#else
          else if ((cu.mtsFlag && compID == COMPONENT_Y) || (transformMode > 0) || (cu.lfnstFlag && !tu.noResidual && compID == COMPONENT_Y))
#endif
#else
          else if ((cu.mtsFlag && compID == COMPONENT_Y) || (transformMode > 0) || (cu.lfnstFlag && compID == COMPONENT_Y))
#endif
#elif JVET_AA0133_INTER_MTS_OPT
          else if ((cu.mtsFlag && compID == COMPONENT_Y) || (transformMode > 0))
#else
          else if (transformMode > 0)
#endif
          {
            currCompCost = MAX_DOUBLE;
          }
          else
          {
            currCompFracBits = nonCoeffFracBits;
            currCompDist     = nonCoeffDist;
            currCompCost     = nonCoeffCost;

            tu.cbf[compID] = 0;
#if JVET_AL0181_ASBT
            tu.asbtDecimH[compID] = 0;
            tu.asbtDecimW[compID] = 0;
#endif
          }
#if JVET_AA0133_INTER_MTS_OPT
#if JVET_AG0061_INTER_LFNST_NSPT
          if (compID == COMPONENT_Y && (cu.mtsFlag || cu.lfnstFlag) && currCompCost < MAX_DOUBLE)
#else
          if (compID == COMPONENT_Y && cu.mtsFlag && currCompCost < MAX_DOUBLE)
#endif
          {
            double globalMinCost = std::min(minCost[compID], m_bestDCT2PassLumaCost);
            double fac = std::max((1.0 + 1.0 / sqrt(tu.lumaSize().width * tu.lumaSize().height)), 1.06);
#if JVET_AG0061_INTER_LFNST_NSPT
            if (cu.lfnstFlag)
            {
              fac *= 1.1;
            }
#endif
            if (currCompCost > fac*globalMinCost)
            {
              skipRemainingMTS = true;
              if (countSkipMTSLoop == 0)
              {
                //skip MTS candidate condition fulfilled for first valid MTS candidate (count = 0), so skip chroma coding.
                skipMTSPass = true;
              }
            }
            countSkipMTSLoop++;
          }
 #endif
          // evaluate
#if TU_256
          if( isFirstMode || ( currCompCost < minCost[compID] ) || ( transformMode == 1 && currCompCost == minCost[compID] ) )
#else
          if( ( currCompCost < minCost[compID] ) || ( transformMode == 1 && currCompCost == minCost[compID] ) )
#endif
          {
            // copy component
            if (isFirstMode && ((nonCoeffCost < currCompCost) || (currAbsSum == 0))) // check for forced null
            {
              tu.getCoeffs( compID ).fill( 0 );
              csFull->getResiBuf( compArea ).fill( 0 );
              tu.cbf[compID]   = 0;

#if JVET_AL0181_ASBT
              tu.asbtDecimH[compID] = 0;
              tu.asbtDecimW[compID] = 0;
#endif
              currAbsSum       = 0;
              currCompFracBits = nonCoeffFracBits;
              currCompDist     = nonCoeffDist;
              currCompCost     = nonCoeffCost;
            }

            uiSingleDistComp[compID] = currCompDist;
            uiSingleFracBits[compID] = currCompFracBits;
            minCost[compID]          = currCompCost;

              bestTU.copyComponentFrom( tu, compID );
              saveCS.getResiBuf( compArea ).copyFrom( csFull->getResiBuf( compArea ) );
          }
          if( tu.noResidual )
          {
            CHECK( currCompFracBits > 0 || currAbsSum, "currCompFracBits > 0 when tu noResidual" );
          }
      }
#if JVET_AL0181_ASBT
      } 
      if (earlyTermBreak)
      {
        break;
      }
#endif

#if JVET_AF0073_INTER_CCP_MERGE
      minCostInterCcpIdx[interCccm][c] = minCost[c] / 3.0;
#endif
#if JVET_AA0133_INTER_MTS_OPT
      if (compID == 0)
      {
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_SBT_LFNST
        if (cu.mtsFlag || (cu.lfnstFlag && !bestTU.noResidual))
#else
        if (cu.mtsFlag || cu.lfnstFlag)
#endif
#else
        if (cu.mtsFlag)
#endif
        {
          if (minCost[compID] == MAX_DOUBLE || bestTU.cbf[0] == 0) //When checking only MTS cands, cbf can't be zero, or just only contain DC coefficients.
          {
            return false;
          }
          if (skipMTSPass) //Luma is not selecting any MTS (skipping), so skip chroma coding (Encoder speedup).
          {
            return false;
          }
        }
        else
        {
          if (!cu.sbtInfo)
          {
            m_bestDCT2PassLumaCost = minCost[compID];
          }
        }
      }
#endif
      // copy component
      tu.copyComponentFrom(bestTU, compID);
#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AI0050_SBT_LFNST
      if ((compID == COMPONENT_Y && !tu.cu->sbtInfo) || (!tu.noResidual && compID == COMPONENT_Y && tu.cu->sbtInfo))
#else
      if (compID == COMPONENT_Y)
#endif
      {
        tu.cu->lfnstIdx = tu.lfnstIdx[compID];
#if JVET_AI0050_INTER_MTSS
        tu.cu->lfnstIntra = tu.lfnstIntra[compID];
#endif
      }
#endif
      csFull->getResiBuf( compArea ).copyFrom( saveCS.getResiBuf( compArea ) );
      if (colorTransFlag && (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless()))
      {
        m_pcTrQuant->lambdaAdjustColorTrans(false);
        m_pcRdCost->lambdaAdjustColorTrans(false, compID);
      }
#if SIGN_PREDICTION
      if(cs.sps->getNumPredSigns() > 0 || (cs.pps->getUseBIF() && isLuma( compID )) || (cs.pps->getUseChromaBIF() && !isLuma( compID )))
      {
#if JVET_Z0118_GDR
#if JVET_Y0065_GPM_INTRA
        bool lmcsEnable = cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( compID ) && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC( *tu.cu );
#else
        bool lmcsEnable = cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( compID ) && !tu.cu->firstPU->ciipFlag && !CU::isIBC( *tu.cu );
#endif
#if JVET_AE0059_INTER_CCCM
        PelBuf tmpPredSignPred(m_interCccmStorage[11], compArea);
        if (tu.interCccm && isChroma(compID))
        {
          tmpPredSignPred.copyFrom(tu.cs->getPredBuf(tu.blocks[compID]));
          tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(interCccmPredBuf[compID]);
        }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
        PelBuf tmpPredSignPredCcpMerge(m_interCcpMergeStorage[4], compArea);
        if (tu.interCcpMerge && isChroma(compID))
        {
          tmpPredSignPredCcpMerge.copyFrom(tu.cs->getPredBuf(tu.blocks[compID]));
          tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(interCcpMergePredBuf[compID-1]);
        }
#endif
        cs.reconstructPicture(tu.blocks[compID], m_pcReshape->getFwdLUT(), csFull, lmcsEnable);
#if JVET_AE0059_INTER_CCCM
        if (tu.interCccm && isChroma(compID))
        {
          tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(tmpPredSignPred);
        }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
        if (tu.interCcpMerge && isChroma(compID))
        {
          tu.cs->getPredBuf(tu.blocks[compID]).copyFrom(tmpPredSignPredCcpMerge);
        }
#endif
#else
        PelBuf picRecoBuff = tu.cs->picture->getRecoBuf( tu.blocks[compID] );

#if JVET_Y0065_GPM_INTRA
        if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( compID ) && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC( *tu.cu ) )
#else
        if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( compID ) && !tu.cu->firstPU->ciipFlag && !CU::isIBC( *tu.cu ) )
#endif
        {
          picRecoBuff.rspSignal( cs.getPredBuf( tu.blocks[compID] ), m_pcReshape->getFwdLUT() );
          picRecoBuff.reconstruct( picRecoBuff, csFull->getResiBuf( tu.blocks[compID] ), tu.cu->cs->slice->clpRng( compID ) );
        }
        else
        {
          picRecoBuff.reconstruct( cs.getPredBuf( tu.blocks[compID] ), csFull->getResiBuf( tu.blocks[compID] ), tu.cu->cs->slice->clpRng( compID ) );
        }
      
#endif
      }
#endif
    } // component loop

#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
    if (interCccmRdSearch || interCcpMergeRdSearch)
#else
    if (interCccmRdSearch)
#endif
    {
      const double curCost = minCost[0]/3.0 + minCost[1]/3.0 + minCost[2]/3.0;
#if JVET_AF0073_INTER_CCP_MERGE
      if ((curCost < bestCost && (interCccmOk || interCcpMergeOk)) || interCccm == 0)
#else
      if ((curCost < bestCost && interCccmOk) || interCccm == 0)
#endif
      {
        bestCost = curCost;
#if JVET_AF0073_INTER_CCP_MERGE
        for (int c = ((tu.interCccm || tu.interCcpMerge) ? 1 : 0); c < numTBlocks; c++)
#else
        for (int c = 0; c < numTBlocks; c++)
#endif
        {
          const ComponentID compID = ComponentID(c);
          interCccmTU.copyComponentFrom(tu, compID);
          interCccmResiBuf[compID].copyFrom(csFull->getResiBuf(tu.blocks[compID]));
          tempMinCost[compID] = minCost[compID];
          tempUiSingleDistComp[compID] = uiSingleDistComp[compID];
          tempUiSingleFracBits[compID] = uiSingleFracBits[compID];
        }
      }
#if JVET_AF0073_INTER_CCP_MERGE
      if (interCccmRdSearch && interCcpMergeRdSearch && interCccm == 1
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
        && ( !tu.cs->slice->getSPS()->getUseInterCcpMergeZeroLumaCbf() || TU::getCbf(tu, COMPONENT_Y))
#endif
        )

      {
        const double reducedCurCost = curCost - (curCost / 2.0);
        if (reducedCurCost > bestCost)
        {
          skipInterCccm2 = true;
        }
      }
#endif
    }
    } // interCccm loop
#if JVET_AF0073_INTER_CCP_MERGE
    if (interCccmRdSearch || interCcpMergeRdSearch)
#else
    if (interCccmRdSearch)
#endif
    {
      for (int c = 0; c < numTBlocks; c++)
      {
        const ComponentID compID = ComponentID(c);
        tu.copyComponentFrom(interCccmTU, compID);
        bestTU.copyComponentFrom(interCccmTU, compID);
        csFull->getResiBuf(tu.blocks[compID]).copyFrom(interCccmResiBuf[compID]);
        saveCS.getResiBuf(tu.blocks[compID]).copyFrom(interCccmResiBuf[compID]);
        minCost[compID] = tempMinCost[compID];
        uiSingleDistComp[compID] = tempUiSingleDistComp[compID];
        uiSingleFracBits[compID] = tempUiSingleFracBits[compID];
#if SIGN_PREDICTION
#if JVET_AF0073_INTER_CCP_MERGE
        if (cs.sps->getNumPredSigns() > 0 && (interCccmOk || interCcpMergeOk) && (!tu.interCccm && !tu.interCcpMerge) && !isLuma(compID))
#else
        if (cs.sps->getNumPredSigns() > 0 && interCccmOk && !tu.interCccm && !isLuma(compID))
#endif
        {
          cs.reconstructPicture(tu.blocks[compID], m_pcReshape->getFwdLUT(), csFull, false);
        }
#endif
      }
    }
#endif
    if (colorTransFlag)
    {
      PelUnitBuf     orgResidual = orgResi->subBuf(relativeUnitArea);
      PelUnitBuf     invColorTransResidual = m_colorTransResiBuf[2].getBuf(relativeUnitArea);
      csFull->getResiBuf(currArea).colorSpaceConvert(invColorTransResidual, false, slice.clpRng(COMPONENT_Y));
#if JVET_S0234_ACT_CRS_FIX
      if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[COMPONENT_Cb].width*tu.blocks[COMPONENT_Cb].height > 4)
      {
        invColorTransResidual.bufs[1].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
        invColorTransResidual.bufs[2].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
      }
#endif

      for (uint32_t c = 0; c < numTBlocks; c++)
      {
        const ComponentID compID = (ComponentID)c;
        uiSingleDistComp[c] = m_pcRdCost->getDistPart(orgResidual.bufs[c], invColorTransResidual.bufs[c], sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
        minCost[c] = m_pcRdCost->calcRdCost(uiSingleFracBits[c], uiSingleDistComp[c]);
      }
    }

    if ( chroma && isChromaEnabled(tu.chromaFormat) && tu.blocks[COMPONENT_Cb].valid() )
    {
      const CompArea& cbArea = tu.blocks[COMPONENT_Cb];
      const CompArea& crArea = tu.blocks[COMPONENT_Cr];
      bool checkJointCbCr = (sps.getJointCbCrEnabledFlag()) && (!tu.noResidual) && (TU::getCbf(tu, COMPONENT_Cb) || TU::getCbf(tu, COMPONENT_Cr));
      bool checkDCTOnly = (TU::getCbf(tu, COMPONENT_Cb) && tu.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && !TU::getCbf(tu, COMPONENT_Cr)) ||
                          (TU::getCbf(tu, COMPONENT_Cr) && tu.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2 && !TU::getCbf(tu, COMPONENT_Cb)) ||
                          (TU::getCbf(tu, COMPONENT_Cb) && tu.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && TU::getCbf(tu, COMPONENT_Cr) && tu.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2);

      bool checkTSOnly = (TU::getCbf(tu, COMPONENT_Cb) && tu.mtsIdx[COMPONENT_Cb] == MTS_SKIP && !TU::getCbf(tu, COMPONENT_Cr)) ||
                         (TU::getCbf(tu, COMPONENT_Cr) && tu.mtsIdx[COMPONENT_Cr] == MTS_SKIP && !TU::getCbf(tu, COMPONENT_Cb)) ||
                         (TU::getCbf(tu, COMPONENT_Cb) && tu.mtsIdx[COMPONENT_Cb] == MTS_SKIP && TU::getCbf(tu, COMPONENT_Cr) && tu.mtsIdx[COMPONENT_Cr] == MTS_SKIP);
      const int channelBitDepth = sps.getBitDepth(toChannelType(COMPONENT_Cb));
      bool      reshape         = slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag()
                               && tu.blocks[COMPONENT_Cb].width * tu.blocks[COMPONENT_Cb].height > 4;
      double minCostCbCr = minCost[COMPONENT_Cb] + minCost[COMPONENT_Cr];
      if (colorTransFlag)
      {
        minCostCbCr += minCost[COMPONENT_Y];  // ACT should consider three-component cost
      }

      CompStorage      orgResiCb[4], orgResiCr[4];   // 0:std, 1-3:jointCbCr
#if JVET_AL0181_ASBT
      CompArea crAreaDown = crArea;
      CompArea cbAreaDown = cbArea;
#endif
      std::vector<int> jointCbfMasksToTest;
      if ( checkJointCbCr )
      {
        orgResiCb[0].create(cbArea);
        orgResiCr[0].create(crArea);
#if JVET_S0234_ACT_CRS_FIX
        if (colorTransFlag)
        {
          orgResiCb[0].copyFrom(colorTransResidual.bufs[1]);
          orgResiCr[0].copyFrom(colorTransResidual.bufs[2]);
        }
        else
        {
#endif
#if JVET_AL0181_ASBT 
          if ( tu.asbtDecimH[COMPONENT_Y] || tu.asbtDecimW[COMPONENT_Y] )
          {
            if ( tu.interCccm )
            {
              chromaPred[0].copyFrom(interCccmPredBuf[COMPONENT_Cb]);
              chromaResi[0].copyFrom(interCccmOrgResiBuf[COMPONENT_Cb]);
              chromaPred[1].copyFrom(interCccmPredBuf[COMPONENT_Cr]);
              chromaResi[1].copyFrom(interCccmOrgResiBuf[COMPONENT_Cr]);
            }
            else if ( tu.interCcpMerge )
            {
              chromaPred[0].copyFrom(interCcpMergePredBuf[0]);
              chromaResi[0].copyFrom(interCcpMergeOrgResiBuf[0]);
              chromaPred[1].copyFrom(interCcpMergePredBuf[1]);
              chromaResi[1].copyFrom(interCcpMergeOrgResiBuf[1]);
            }
            else
            {
              chromaPred[0].copyFrom(csFull->getPredBuf(tu.blocks[COMPONENT_Cb]));
              chromaResi[0].copyFrom(cs.getOrgResiBuf(cbArea));
              chromaPred[1].copyFrom(csFull->getPredBuf(tu.blocks[COMPONENT_Cr]));
              chromaResi[1].copyFrom(cs.getOrgResiBuf(crArea));
            }

            orgResiCb[0].copyFrom(chromaResi[COMPONENT_Cb-1]);
            orgResiCr[0].copyFrom(chromaResi[COMPONENT_Cr-1]);

            int maxDecim = std::max( tu.asbtDecimH[COMPONENT_Y], tu.asbtDecimW[COMPONENT_Y] );
            bool asbtAllowedW = (tu.blocks[COMPONENT_Cr].width >> maxDecim) >= 2;
            bool asbtAllowedH = (tu.blocks[COMPONENT_Cr].height >> maxDecim) >= 2;
            tu.blocksResidual[COMPONENT_Cr].size() = tu.blocks[COMPONENT_Cr].size();
            tu.blocksResidual[COMPONENT_Cb].size() = tu.blocks[COMPONENT_Cb].size();
            tu.blocksResForDbf[COMPONENT_Cr].size() = tu.blocks[COMPONENT_Cr].size();
            tu.blocksResForDbf[COMPONENT_Cb].size() = tu.blocks[COMPONENT_Cb].size();

            if ( asbtAllowedW && asbtAllowedH )
            {
              int dir = asbtDownSamplingPred ( tu, COMPONENT_Cb, chromaPred[COMPONENT_Cb-1], chromaResi[COMPONENT_Cb-1], orgResiCb[0], maxDecim );
              if( dir == 0 )
              {
                tu.blocksResidual[COMPONENT_Cr].width = tu.blocksResidual[COMPONENT_Cr].width >> maxDecim;
                tu.blocksResidual[COMPONENT_Cb].width  =tu.blocksResidual[COMPONENT_Cb].width >> maxDecim;

                tu.asbtDecimH[COMPONENT_Cr]=0;
                tu.asbtDecimW[COMPONENT_Cr]=maxDecim;
                tu.asbtDecimH[COMPONENT_Cb]=0;
                tu.asbtDecimW[COMPONENT_Cb]=maxDecim;

                crAreaDown.size().width = tu.blocksResidual[COMPONENT_Cr].width;
                cbAreaDown.size().width = tu.blocksResidual[COMPONENT_Cb].width;
                asbtDownSampling ( tu, COMPONENT_Cr, chromaPred[COMPONENT_Cr-1], chromaResi[COMPONENT_Cr-1], orgResiCr[0] );

                tu.blocksResForDbf[COMPONENT_Cr].width = crAreaDown.size().width;
                tu.blocksResForDbf[COMPONENT_Cb].width = crAreaDown.size().width;
                tu.fillBlocksResForDbfPos(COMPONENT_Cr);
                tu.fillBlocksResForDbfPos(COMPONENT_Cb);
              }
              else
              {
                tu.blocksResidual[COMPONENT_Cr].height = tu.blocksResidual[COMPONENT_Cr].height >> maxDecim;
                tu.blocksResidual[COMPONENT_Cb].height = tu.blocksResidual[COMPONENT_Cb].height >> maxDecim;

                tu.asbtDecimW[COMPONENT_Cr] = 0;
                tu.asbtDecimH[COMPONENT_Cr] = maxDecim;
                tu.asbtDecimW[COMPONENT_Cb] = 0;
                tu.asbtDecimH[COMPONENT_Cb] = maxDecim;

                crAreaDown.size().height = tu.blocksResidual[COMPONENT_Cr].height;
                cbAreaDown.size().height = tu.blocksResidual[COMPONENT_Cb].height;

                asbtDownSamplingH ( tu, COMPONENT_Cr, chromaPred[COMPONENT_Cr-1], chromaResi[COMPONENT_Cr-1], orgResiCr[0] );
                tu.blocksResForDbf[COMPONENT_Cr].height = crAreaDown.size().height;
                tu.blocksResForDbf[COMPONENT_Cb].height = crAreaDown.size().height;
                tu.fillBlocksResForDbfPos(COMPONENT_Cr);
                tu.fillBlocksResForDbfPos(COMPONENT_Cb);
              }
            }
            else if ( asbtAllowedW )
            {
              tu.blocksResidual[COMPONENT_Cr].width = tu.blocksResidual[COMPONENT_Cr].width >> maxDecim;
              tu.blocksResidual[COMPONENT_Cb].width = tu.blocksResidual[COMPONENT_Cb].width >> maxDecim;

              tu.asbtDecimH[COMPONENT_Cr] = 0;
              tu.asbtDecimW[COMPONENT_Cr] = maxDecim;
              tu.asbtDecimH[COMPONENT_Cb] = 0;
              tu.asbtDecimW[COMPONENT_Cb] = maxDecim;

              asbtDownSampling ( tu, COMPONENT_Cb, chromaPred[COMPONENT_Cb-1], chromaResi[COMPONENT_Cb-1], orgResiCb[0]);
              asbtDownSampling ( tu, COMPONENT_Cr, chromaPred[COMPONENT_Cr-1], chromaResi[COMPONENT_Cr-1], orgResiCr[0]);
              crAreaDown.size().width = tu.blocksResidual[COMPONENT_Cr].width;
              cbAreaDown.size().width = tu.blocksResidual[COMPONENT_Cb].width;
              tu.blocksResForDbf[COMPONENT_Cr].width = crAreaDown.size().width;
              tu.blocksResForDbf[COMPONENT_Cb].width = crAreaDown.size().width;
              tu.fillBlocksResForDbfPos(COMPONENT_Cr);
              tu.fillBlocksResForDbfPos(COMPONENT_Cb);
            }
            else
            {
              tu.blocksResidual[COMPONENT_Cr].height = tu.blocksResidual[COMPONENT_Cr].height >> maxDecim;
              tu.blocksResidual[COMPONENT_Cb].height = tu.blocksResidual[COMPONENT_Cb].height >> maxDecim;
              tu.asbtDecimW[COMPONENT_Cr] = 0;
              tu.asbtDecimH[COMPONENT_Cr] = maxDecim;
              tu.asbtDecimW[COMPONENT_Cb] = 0;
              tu.asbtDecimH[COMPONENT_Cb] = maxDecim;

              asbtDownSamplingH ( tu, COMPONENT_Cb, chromaPred[COMPONENT_Cb-1], chromaResi[COMPONENT_Cb-1], orgResiCb[0] );
              asbtDownSamplingH ( tu, COMPONENT_Cr, chromaPred[COMPONENT_Cr-1], chromaResi[COMPONENT_Cr-1], orgResiCr[0] );

              crAreaDown.size().height = tu.blocksResidual[COMPONENT_Cr].height;
              cbAreaDown.size().height = tu.blocksResidual[COMPONENT_Cb].height;
              tu.blocksResForDbf[COMPONENT_Cr].height = crAreaDown.size().height;
              tu.blocksResForDbf[COMPONENT_Cb].height = crAreaDown.size().height;
              tu.fillBlocksResForDbfPos(COMPONENT_Cr);
              tu.fillBlocksResForDbfPos(COMPONENT_Cb);
            }
            orgResiCb[0].resizeTo( tu.blocksResidual[COMPONENT_Cb] );
            orgResiCr[0].resizeTo( tu.blocksResidual[COMPONENT_Cr] );
          }
          else 
#endif 
#if JVET_AE0059_INTER_CCCM
          if (tu.interCccm)
          {
            orgResiCb[0].copyFrom(interCccmOrgResiBuf[COMPONENT_Cb]);
            orgResiCr[0].copyFrom(interCccmOrgResiBuf[COMPONENT_Cr]);
          }
#if JVET_AF0073_INTER_CCP_MERGE
          else if (tu.interCcpMerge)
          {
            orgResiCb[0].copyFrom(interCcpMergeOrgResiBuf[0]);
            orgResiCr[0].copyFrom(interCcpMergeOrgResiBuf[1]);
          }
#endif
          else
          {
            orgResiCb[0].copyFrom(cs.getOrgResiBuf(cbArea));
            orgResiCr[0].copyFrom(cs.getOrgResiBuf(crArea));
          }
#else
          orgResiCb[0].copyFrom(cs.getOrgResiBuf(cbArea));
          orgResiCr[0].copyFrom(cs.getOrgResiBuf(crArea));
#endif
#if JVET_S0234_ACT_CRS_FIX
        }
        if (!colorTransFlag && reshape)
#else
        if (reshape)
#endif
        {
          orgResiCb[0].scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
          orgResiCr[0].scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
        }
        jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(tu, orgResiCb, orgResiCr);
      }

      for (int cbfMask: jointCbfMasksToTest)
      {
        ComponentID codeCompId = (cbfMask >> 1 ? COMPONENT_Cb : COMPONENT_Cr);
        ComponentID otherCompId = (codeCompId == COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr);
        bool        tsAllowed = TU::isTSAllowed(tu, codeCompId) && (m_pcEncCfg->getUseChromaTS());
        uint8_t     numTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
        bool        cbfDCT2 = true;

        std::vector<TrMode> trModes;
        if (checkDCTOnly || checkTSOnly)
        {
          numTransformCands = 1;
        }

        if (!checkTSOnly)
        {
          trModes.push_back(TrMode(0, true)); // DCT2
        }
        if (tsAllowed && !checkDCTOnly)
        {
          trModes.push_back(TrMode(1, true));//TS
        }
        for (int modeId = 0; modeId < numTransformCands; modeId++)
        {
          if (modeId && !cbfDCT2)
          {
            continue;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        TCoeff     currAbsSum       = 0;
        uint64_t   currCompFracBits = 0;
        Distortion currCompDistCb   = 0;
        Distortion currCompDistCr   = 0;
        double     currCompCost     = 0;

        tu.jointCbCr = (uint8_t) cbfMask;
          // encoder bugfix: initialize mtsIdx for chroma under JointCbCrMode.
        tu.mtsIdx[codeCompId]  = trModes[modeId].first;
        tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
        int         codedCbfMask = 0;
        if (colorTransFlag && (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless()))
        {
          m_pcTrQuant->lambdaAdjustColorTrans(true);
          m_pcTrQuant->selectLambda(codeCompId);
        }
        else
        {
          m_pcTrQuant->selectLambda(codeCompId);
        }
        // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
        const int    absIct = abs( TU::getICTMode(tu) );
        const double lfact  = ( absIct == 1 || absIct == 3 ? 0.8 : 0.5 );
        m_pcTrQuant->setLambda( lfact * m_pcTrQuant->getLambda() );
        if ( checkJointCbCr && (tu.cu->cs->slice->getSliceQp() > 18))
        {
          m_pcTrQuant->setLambda( 1.05 * m_pcTrQuant->getLambda() );
        }

        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();

#if JVET_AL0181_ASBT
        PelBuf cbResi = csFull->getResiBuf(cbAreaDown);
        PelBuf crResi = csFull->getResiBuf(crAreaDown);
#else
        PelBuf cbResi = csFull->getResiBuf(cbArea);
        PelBuf crResi = csFull->getResiBuf(crArea);
#endif
        cbResi.copyFrom(orgResiCb[cbfMask]);
        crResi.copyFrom(orgResiCr[cbfMask]);

        if ( reshape )
        {
          double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)(tu.getChromaAdj());
          m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cRescale*cRescale));
        }

        Distortion currCompDistY = MAX_UINT64;
        QpParam qpCbCr(tu, codeCompId);

        tu.getCoeffs(otherCompId).fill(0);   // do we need that?
        TU::setCbfAtDepth(tu, otherCompId, tu.depth, false);

        PelBuf &codeResi   = (codeCompId == COMPONENT_Cr ? crResi : cbResi);
        TCoeff  compAbsSum = 0;
        if (numTransformCands > 1)
        {
          if (modeId == 0)
          {
            m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, &trModes, m_pcEncCfg->getMTSInterMaxCand());
            tu.mtsIdx[codeCompId] = trModes[modeId].first;
            tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
          }
          m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, compAbsSum, m_CABACEstimator->getCtx(), true);
        }
        else
        m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, compAbsSum, m_CABACEstimator->getCtx());
        if (compAbsSum > 0)
        {
          m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
          codedCbfMask += (codeCompId == COMPONENT_Cb ? 2 : 1);
        }
        else
        {
          codeResi.fill(0);
        }

        if (tu.jointCbCr == 3 && codedCbfMask == 2)
        {
          codedCbfMask = 3;
          TU::setCbfAtDepth(tu, COMPONENT_Cr, tu.depth, true);
        }
        if (codedCbfMask && tu.jointCbCr != codedCbfMask)
        {
          codedCbfMask = 0;
        }
        currAbsSum = codedCbfMask;

        if (!tu.mtsIdx[codeCompId])
        {
          cbfDCT2 = (currAbsSum > 0);
        }
        if (currAbsSum > 0)
        {
          m_CABACEstimator->cbf_comp(cs, codedCbfMask >> 1, cbArea, currDepth, false);
          m_CABACEstimator->cbf_comp(cs, codedCbfMask & 1, crArea, currDepth, codedCbfMask >> 1);
#if JVET_AF0073_INTER_CCP_MERGE
          m_CABACEstimator->interCcpMerge(tu);
#endif
#if JVET_AE0059_INTER_CCCM
          m_CABACEstimator->interCccm(tu);
#endif
          m_CABACEstimator->joint_cb_cr(tu, codedCbfMask);
          if (codedCbfMask >> 1)
            m_CABACEstimator->residual_coding(tu, COMPONENT_Cb);
          if (codedCbfMask & 1)
            m_CABACEstimator->residual_coding(tu, COMPONENT_Cr);
          currCompFracBits = m_CABACEstimator->getEstFracBits();

          m_pcTrQuant->invTransformICT(tu, cbResi, crResi);
#if JVET_AL0181_ASBT
          if (tu.asbtDecimW[codeCompId])
          {
            cbResi.resizeTo(cbArea);
            crResi.resizeTo(crArea);
            upsampleOneBlock(tu, cbResi, COMPONENT_Cb);
            upsampleOneBlock(tu, crResi, COMPONENT_Cr);
          }
          else if (tu.asbtDecimH[codeCompId])
          {
            cbResi.resizeTo(cbArea);
            crResi.resizeTo(crArea);
            upsampleOneBlockH(tu, cbResi, COMPONENT_Cb);
            upsampleOneBlockH(tu, crResi, COMPONENT_Cr);
          }
#endif
#if JVET_S0234_ACT_CRS_FIX
          if (!colorTransFlag && reshape)
#else
          if (reshape)
#endif
          {
            cbResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
            crResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
          }

          if (colorTransFlag)
          {
            PelUnitBuf     orgResidual = orgResi->subBuf(relativeUnitArea);
            PelUnitBuf     invColorTransResidual = m_colorTransResiBuf[2].getBuf(relativeUnitArea);
            csFull->getResiBuf(currArea).colorSpaceConvert(invColorTransResidual, false, slice.clpRng(COMPONENT_Y));
#if JVET_S0234_ACT_CRS_FIX
            if (reshape)
            {
              invColorTransResidual.bufs[1].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
              invColorTransResidual.bufs[2].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
            }
#endif

            currCompDistY = m_pcRdCost->getDistPart(orgResidual.bufs[COMPONENT_Y], invColorTransResidual.bufs[COMPONENT_Y], sps.getBitDepth(toChannelType(COMPONENT_Y)), COMPONENT_Y, DF_SSE);
            currCompDistCb = m_pcRdCost->getDistPart(orgResidual.bufs[COMPONENT_Cb], invColorTransResidual.bufs[COMPONENT_Cb], sps.getBitDepth(toChannelType(COMPONENT_Cb)), COMPONENT_Cb, DF_SSE);
            currCompDistCr = m_pcRdCost->getDistPart(orgResidual.bufs[COMPONENT_Cr], invColorTransResidual.bufs[COMPONENT_Cr], sps.getBitDepth(toChannelType(COMPONENT_Cr)), COMPONENT_Cr, DF_SSE);
            currCompCost = m_pcRdCost->calcRdCost(uiSingleFracBits[COMPONENT_Y] + currCompFracBits, currCompDistY + currCompDistCr + currCompDistCb, false);
          }
          else
          {
#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
            currCompDistCb = m_pcRdCost->getDistPart(tu.interCccm ? interCccmOrgResiBuf[COMPONENT_Cb] : (tu.interCcpMerge ? interCcpMergeOrgResiBuf[0] : csFull->getOrgResiBuf(cbArea)), cbResi, channelBitDepth, COMPONENT_Cb, DF_SSE);
            currCompDistCr = m_pcRdCost->getDistPart(tu.interCccm ? interCccmOrgResiBuf[COMPONENT_Cr] : (tu.interCcpMerge ? interCcpMergeOrgResiBuf[1] : csFull->getOrgResiBuf(crArea)), crResi, channelBitDepth, COMPONENT_Cr, DF_SSE);
#else
            currCompDistCb = m_pcRdCost->getDistPart(tu.interCccm ? interCccmOrgResiBuf[COMPONENT_Cb] : csFull->getOrgResiBuf(cbArea), cbResi, channelBitDepth, COMPONENT_Cb, DF_SSE);
            currCompDistCr = m_pcRdCost->getDistPart(tu.interCccm ? interCccmOrgResiBuf[COMPONENT_Cr] : csFull->getOrgResiBuf(crArea), crResi, channelBitDepth, COMPONENT_Cr, DF_SSE);
#endif
#else
            currCompDistCb = m_pcRdCost->getDistPart(csFull->getOrgResiBuf(cbArea), cbResi, channelBitDepth, COMPONENT_Cb, DF_SSE);
            currCompDistCr = m_pcRdCost->getDistPart(csFull->getOrgResiBuf(crArea), crResi, channelBitDepth, COMPONENT_Cr, DF_SSE);
#endif
#if WCG_EXT
          currCompCost   = m_pcRdCost->calcRdCost(currCompFracBits, currCompDistCr + currCompDistCb, false);
#else
          currCompCost   = m_pcRdCost->calcRdCost(currCompFracBits, currCompDistCr + currCompDistCb);
#endif
          }
        }
        else
          currCompCost = MAX_DOUBLE;

        // evaluate
        if( currCompCost < minCostCbCr )
        {
          uiSingleDistComp[COMPONENT_Cb] = currCompDistCb;
          uiSingleDistComp[COMPONENT_Cr] = currCompDistCr;
          if (colorTransFlag)
          {
            uiSingleDistComp[COMPONENT_Y] = currCompDistY;
          }
          minCostCbCr                    = currCompCost;
          {
            bestTU.copyComponentFrom(tu, COMPONENT_Cb);
            bestTU.copyComponentFrom(tu, COMPONENT_Cr);
            saveCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
            saveCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
          }
        }

        if (colorTransFlag && (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless()))
        {
          m_pcTrQuant->lambdaAdjustColorTrans(false);
        }
        }
      }
      // copy component
      tu.copyComponentFrom(bestTU, COMPONENT_Cb);
      tu.copyComponentFrom(bestTU, COMPONENT_Cr);
      csFull->getResiBuf(cbArea).copyFrom(saveCS.getResiBuf(cbArea));
      csFull->getResiBuf(crArea).copyFrom(saveCS.getResiBuf(crArea));

#if SIGN_PREDICTION
      if( tu.jointCbCr )
      {
        for( auto i = (int)COMPONENT_Cb; i <= (int)COMPONENT_Cr; ++i)
        {
          ComponentID comp = (ComponentID) i;
#if JVET_Z0118_GDR
#if JVET_Y0065_GPM_INTRA
          bool lmcsEnable = cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( comp ) && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC( *tu.cu );
#else         
          bool lmcsEnable = cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( comp ) && !tu.cu->firstPU->ciipFlag && !CU::isIBC( *tu.cu );
#endif
#if JVET_AE0059_INTER_CCCM
          PelBuf tmpPredSignPred(m_interCccmStorage[11], tu.blocks[comp]);
          if (tu.interCccm)
          {
            tmpPredSignPred.copyFrom(tu.cs->getPredBuf(tu.blocks[comp]));
            tu.cs->getPredBuf(tu.blocks[comp]).copyFrom(interCccmPredBuf[comp]);
          }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
          PelBuf tmpPredSignPredCcpMerge(m_interCcpMergeStorage[4], tu.blocks[comp]);
          if (tu.interCcpMerge)
          {
            tmpPredSignPredCcpMerge.copyFrom(tu.cs->getPredBuf(tu.blocks[comp]));
            tu.cs->getPredBuf(tu.blocks[comp]).copyFrom(interCcpMergePredBuf[comp-1]);
          }
#endif
          cs.reconstructPicture(tu.blocks[comp], m_pcReshape->getFwdLUT(), csFull, lmcsEnable);          
#if JVET_AE0059_INTER_CCCM
          if (tu.interCccm)
          {
            tu.cs->getPredBuf(tu.blocks[comp]).copyFrom(tmpPredSignPred);
          }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
          if (tu.interCcpMerge)
          {
            tu.cs->getPredBuf(tu.blocks[comp]).copyFrom(tmpPredSignPredCcpMerge);
          }
#endif
#else
          PelBuf picRecoBuff = tu.cs->picture->getRecoBuf( tu.blocks[comp] );

#if JVET_Y0065_GPM_INTRA
          if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( comp ) && !tu.cu->firstPU->ciipFlag && !tu.cu->firstPU->gpmIntraFlag && !CU::isIBC( *tu.cu ) )
#else
          if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( comp ) && !tu.cu->firstPU->ciipFlag && !CU::isIBC( *tu.cu ) )
#endif
          {
            picRecoBuff.rspSignal( cs.getPredBuf( tu.blocks[comp] ), m_pcReshape->getFwdLUT() );
            picRecoBuff.reconstruct( picRecoBuff, csFull->getResiBuf( tu.blocks[comp] ), tu.cu->cs->slice->clpRng( comp ) );
          }
          else
          {
            picRecoBuff.reconstruct( cs.getPredBuf( tu.blocks[comp] ), csFull->getResiBuf( tu.blocks[comp] ), tu.cu->cs->slice->clpRng( comp ) );
          }
#endif
        }

        if ( sps.getNumPredSigns() > 0)
        {
          bool bJccrWithCr = tu.jointCbCr && !(tu.jointCbCr >> 1);
          ComponentID jccrCompId = bJccrWithCr ? COMPONENT_Cr : COMPONENT_Cb;
          bool reshapeChroma = slice.getPicHeader()->getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[jccrCompId].width*tu.blocks[jccrCompId].height > 4;
#if JVET_AE0059_INTER_CCCM
          PelBuf tmpPredSignPred(m_interCccmStorage[11], tu.blocks[jccrCompId]);
          if (tu.interCccm)
          {
            tmpPredSignPred.copyFrom(tu.cs->getPredBuf(tu.blocks[jccrCompId]));
            tu.cs->getPredBuf(tu.blocks[jccrCompId]).copyFrom(interCccmPredBuf[jccrCompId]);
          }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
          PelBuf tmpPredSignPredCcpMerge(m_interCcpMergeStorage[4], tu.blocks[jccrCompId]);
          if (tu.interCcpMerge)
          {
            tmpPredSignPredCcpMerge.copyFrom(tu.cs->getPredBuf(tu.blocks[jccrCompId]));
            tu.cs->getPredBuf(tu.blocks[jccrCompId]).copyFrom(interCcpMergePredBuf[jccrCompId-1]);
          }
#endif
          m_pcTrQuant->predCoeffSigns(tu, COMPONENT_Cb, reshapeChroma);
#if JVET_AE0059_INTER_CCCM
          if (tu.interCccm)
          {
            tu.cs->getPredBuf(tu.blocks[jccrCompId]).copyFrom(tmpPredSignPred);
          }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
          if (tu.interCcpMerge)
          {
            tu.cs->getPredBuf(tu.blocks[jccrCompId]).copyFrom(tmpPredSignPredCcpMerge);
          }
#endif
        }
      }
#endif
    }

    m_CABACEstimator->getCtx() = ctxStart;
    m_CABACEstimator->resetBits();
    if( !tu.noResidual )
    {
      static const ComponentID cbf_getComp[MAX_NUM_COMPONENT] = { COMPONENT_Cb, COMPONENT_Cr, COMPONENT_Y };
      for( unsigned c = isChromaEnabled(tu.chromaFormat)?0 : 2; c < MAX_NUM_COMPONENT; c++)
    {
      const ComponentID compID = cbf_getComp[c];
      if (compID == COMPONENT_Y && !luma)
        continue;
      if (compID != COMPONENT_Y && !chroma)
        continue;
      if( tu.blocks[compID].valid() )
      {
        const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( tu, COMPONENT_Cb, currDepth ) : false );
        m_CABACEstimator->cbf_comp( *csFull, TU::getCbfAtDepth( tu, compID, currDepth ), tu.blocks[compID], currDepth, prevCbf );
      }
    }
    }

    for (uint32_t ch = 0; ch < numValidComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);
      if (compID == COMPONENT_Y && !luma)
        continue;
      if (compID != COMPONENT_Y && !chroma)
        continue;
      if (tu.blocks[compID].valid())
      {
#if JVET_AF0073_INTER_CCP_MERGE
        if ( compID == COMPONENT_Cr )
        {
          m_CABACEstimator->interCcpMerge(tu);
        }
#endif
#if JVET_AE0059_INTER_CCCM
        if ( compID == COMPONENT_Cr )
        {
          m_CABACEstimator->interCccm(tu);
        }
#endif
        if( compID == COMPONENT_Cr )
        {
          const int cbfMask = ( TU::getCbf( tu, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( tu, COMPONENT_Cr ) ? 1 : 0 );
          m_CABACEstimator->joint_cb_cr(tu, cbfMask);
        }
        if( TU::getCbf( tu, compID ) )
        {
#if JVET_AL0181_ASBT  
          if (compID == COMPONENT_Y && (TU::getCbfAtDepth(tu, COMPONENT_Cr, currDepth) == 0 && TU::getCbfAtDepth(tu, COMPONENT_Cb, currDepth) == 0))
          {
            m_CABACEstimator->interAsbt(tu);
          }
#endif
          m_CABACEstimator->residual_coding( tu, compID );
        }
        uiSingleDist += uiSingleDistComp[compID];
      }
    }
    if( tu.noResidual )
    {
      CHECK( m_CABACEstimator->getEstFracBits() > 0, "no residual TU's bits shall be 0" );
    }
#if JVET_S0234_ACT_CRS_FIX
    if (colorTransFlag)
    {
      PelUnitBuf resiBuf = csFull->getResiBuf(currArea);
      resiBuf.colorSpaceConvert(resiBuf, false, slice.clpRng(COMPONENT_Y));
      if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && tu.blocks[COMPONENT_Cb].width*tu.blocks[COMPONENT_Cb].height > 4)
      {
        resiBuf.bufs[1].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cb));
        resiBuf.bufs[2].scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
      }
    }
#endif

    csFull->fracBits += m_CABACEstimator->getEstFracBits();
    csFull->dist     += uiSingleDist;
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      csFull->cost    = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist, false);
    }
    else
#endif
    csFull->cost      = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
  } // check full

  // code sub-blocks
  if( bCheckSplit )
  {
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs 
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs
#if JVET_AI0087_BTCUS_RESTRICTION
      , false, false
#endif
    ) )
    {
      partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
    }
    else
      THROW( "Implicit TU split not available!" );

#if JVET_AI0050_SBT_LFNST
    bool isValid = false;
#endif
    do
    {
#if JVET_AI0050_SBT_LFNST
      if (cu.sbtInfo)
      {
        bool isSubValid = false;
        isSubValid = xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist, luma, chroma, orgResi);
        if (!csSplit->tus.back()->noResidual)
        {
          isValid = isSubValid;
        }
      }
      else
      {
        xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist
          , luma, chroma
          , orgResi
        );
      }
#else
      xEstimateInterResidualQT(*csSplit, partitioner, bCheckFull ? nullptr : puiZeroDist
        , luma, chroma
        , orgResi
      );
#endif

      csSplit->cost = m_pcRdCost->calcRdCost( csSplit->fracBits, csSplit->dist );
    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    unsigned        anyCbfSet   =   0;
    unsigned        compCbf[3]  = { 0, 0, 0 };

    if( !bCheckFull )
    {
      for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
      {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if ( currTU.chType != partitioner.chType )
        {
          continue;
        }
#endif
        for( unsigned ch = 0; ch < numTBlocks; ch++ )
        {
          compCbf[ ch ] |= ( TU::getCbfAtDepth( currTU, ComponentID(ch), currDepth + 1 ) ? 1 : 0 );
        }
      }

      {

        for( auto &currTU : csSplit->traverseTUs( currArea, partitioner.chType ) )
        {
          TU::setCbfAtDepth   ( currTU, COMPONENT_Y,  currDepth, compCbf[ COMPONENT_Y  ] );
          if( currArea.chromaFormat != CHROMA_400 )
          {
            TU::setCbfAtDepth ( currTU, COMPONENT_Cb, currDepth, compCbf[ COMPONENT_Cb ] );
            TU::setCbfAtDepth ( currTU, COMPONENT_Cr, currDepth, compCbf[ COMPONENT_Cr ] );
          }
        }

        anyCbfSet    = compCbf[ COMPONENT_Y  ];
        if( currArea.chromaFormat != CHROMA_400 )
        {
          anyCbfSet |= compCbf[ COMPONENT_Cb ];
          anyCbfSet |= compCbf[ COMPONENT_Cr ];
        }
      }

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT( *csSplit, partitioner, MAX_NUM_TBLOCKS );
      for (uint32_t ch = 0; ch < numValidComp; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        if (compID == COMPONENT_Y && !luma)
          continue;
        if (compID != COMPONENT_Y && !chroma)
          continue;
        xEncodeInterResidualQT( *csSplit, partitioner, ComponentID( ch ) );
      }

      csSplit->fracBits = m_CABACEstimator->getEstFracBits();
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      if( bCheckFull && anyCbfSet && csSplit->cost < csFull->cost )
      {
        cs.useSubStructure( *csSplit, partitioner.chType, currArea, false, false, false, true, true );
        cs.cost = csSplit->cost;
      }
    }


    if( csSplit && csFull )
    {
      csSplit->releaseIntermediateData();
      csFull ->releaseIntermediateData();
    }
#if JVET_AI0050_SBT_LFNST
    if (cu.sbtInfo)
    {
      return isValid;
    }
#endif
  }
#if JVET_AA0133_INTER_MTS_OPT
  return true;
#endif
}
#if JVET_AA0133_INTER_MTS_OPT
bool InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual
  , const bool luma, const bool chroma
)
#else
void InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual
  , const bool luma, const bool chroma
)
#endif
{
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

  CodingUnit &cu = *cs.getCU( partitioner.chType );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.predMode == MODE_INTER )
    CHECK( cu.isSepTree(), "CU with Inter mode must be in single tree" );
#endif
  const ChromaFormat format     = cs.area.chromaFormat;;
  const int  numValidComponents = getNumberValidComponents(format);
  const SPS &sps                = *cs.sps;

  bool colorTransAllowed = cs.slice->getSPS()->getUseColorTrans() && luma && chroma;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cs.slice->getSPS()->getUseColorTrans())
  {
    CHECK(cu.treeType != TREE_D || partitioner.treeType != TREE_D, "localtree should not be applied when adaptive color transform is enabled");
    CHECK(cu.modeType != MODE_TYPE_ALL || partitioner.modeType != MODE_TYPE_ALL, "localtree should not be applied when adaptive color transform is enabled");
  }
#endif
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    cu.interCcpMergeZeroRootCbfIdc = false;
    PelBuf interCcpMergePredBuf[2];
    for( int i = 1; i < MAX_NUM_COMPONENT; i++ )
    {
      interCcpMergePredBuf[i-1] = PelBuf( m_interCcpMergeStorage[i-1], cu.blocks[ComponentID(i)] ); // borrow the interCcpMergeStorage 
    }
#endif

  if( skipResidual ) //  No residual coding : SKIP mode
  {
    cu.skip    = true;
    cu.rootCbf = false;
    cu.colorTransform = false;
    CHECK( cu.sbtInfo != 0, "sbtInfo shall be 0 if CU has no residual" );
    cs.getResiBuf().fill(0);

#if JVET_Y0065_GPM_INTRA
    if( m_pcEncCfg->getLmcs() && ( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() ) && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC( cu ) )
#else
    if( m_pcEncCfg->getLmcs() && ( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() ) && !cu.firstPU->ciipFlag && !CU::isIBC( cu ) )
#endif
    {
      cs.getRecoBuf().Y().rspSignal( cs.getPredBuf().Y(), m_pcReshape->getFwdLUT() );
      cs.getRecoBuf().Cb().copyFrom( cs.getPredBuf().Cb() );
      cs.getRecoBuf().Cr().copyFrom( cs.getPredBuf().Cr() );
    }
    else
    {
      cs.getRecoBuf().copyFrom( cs.getPredBuf() );
#if JVET_AA0070_RRIBC
      if (CU::isIBC(cu) && cu.rribcFlipType)
      {
        cs.getRecoBuf().Y().flipSignal(cu.rribcFlipType == 1);
        cs.getRecoBuf().Cb().flipSignal(cu.rribcFlipType == 1);
        cs.getRecoBuf().Cr().flipSignal(cu.rribcFlipType == 1);
      }
#endif
    }
#if JVET_AG0145_ADAPTIVE_CLIPPING
    ClpRngs clpRngs = cu.slice->clpRngs();
    if (cs.picHeader->getLmcsEnabledFlag()) // LMCS enabled
    {
      std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
      clpRngs.comp[COMPONENT_Y].min = fwdLUT[cu.cs->slice->getLumaPelMin()];
      clpRngs.comp[COMPONENT_Y].max = fwdLUT[cu.cs->slice->getLumaPelMax()];
    }
    else
    {
      clpRngs.comp[COMPONENT_Y].min = cu.cs->slice->getLumaPelMin();
      clpRngs.comp[COMPONENT_Y].max = cu.cs->slice->getLumaPelMax();
    }
    cs.getRecoBuf().copyClip(cs.getRecoBuf(), clpRngs, luma && !chroma, !luma && chroma);
#endif

    // add empty TU(s)
    cs.addEmptyTUs( partitioner );
    Distortion distortion = 0;
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    Distortion distortionLuma = 0;
#endif
    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      if (compID == COMPONENT_Y && !luma)
        continue;
      if (compID != COMPONENT_Y && !chroma)
        continue;
      CPelBuf reco = cs.getRecoBuf (compID);
      CPelBuf org  = cs.getOrgBuf  (compID);
#if WCG_EXT
      if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
        m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
      {
        const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
        if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          const CompArea &areaY = cu.Y();
          CompArea      tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
          tmpRecLuma.rspSignal( reco, m_pcReshape->getInvLUT() );
          distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
      }
      else
#endif
      distortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
      if(compID == COMPONENT_Y)
      {
        distortionLuma = distortion;
      }
#endif
    }

#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    bool isInterCcpMergeRootCbfZeroAllowed = CU::interCcpMergeZeroRootCbfAllowed(cu);
    if(isInterCcpMergeRootCbfZeroAllowed && m_pcEncCfg->getInterCcpMergeZeroLumaCbfFastMode())
    {
      if(cu.blocks[COMPONENT_Cb].area() < 16 || cu.blocks[COMPONENT_Cb].area() > 1024)
      {
        if ((cu.skip && !cu.slice->getCheckLDB()) || !cu.skip)
        {
          isInterCcpMergeRootCbfZeroAllowed = false;
        }
      }
    }

    if (isInterCcpMergeRootCbfZeroAllowed)
    {
      PelBuf bufCb = cs.getPredBuf( cu.blocks[COMPONENT_Cb] );
      PelBuf bufCr = cs.getPredBuf( cu.blocks[COMPONENT_Cr] );
      if (m_isInterCcpModelReady == false)
      {
        m_pcIntraPred->xAddOnTheFlyCalcCCPCands4InterBlk(*cu.firstPU, cu.blocks[COMPONENT_Cb], m_interCcpMergeList, m_validNum);
        m_isInterCcpModelReady = true;
      }
      const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
      double bestCost = MAX_DOUBLE;
      int8_t bestWIdx = 0;
      for (int8_t wIdx = 0; wIdx <= MAX_CCP_MERGE_WEIGHT_IDX; wIdx++)
      {
        cu.interCcpMergeZeroRootCbfIdc = wIdx;
        if (wIdx == 0)
        {
          const bool valid = deriveInterCcpMergePrediction(cu.firstTU, cs.getRecoBuf(cu.blocks[COMPONENT_Y]), bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], m_interCcpMergeList, m_validNum);
          CHECK(!valid, "invalid inter ccp merge for rootCbf = 0");
        }
        else
        {
          m_pcIntraPred->combineCcpAndInter(*cu.firstPU, bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], true);
        }
#if JVET_AA0070_RRIBC
        if (CU::isIBC(cu) && cu.rribcFlipType)
        {
          interCcpMergePredBuf[0].flipSignal(cu.rribcFlipType == 1);
          interCcpMergePredBuf[1].flipSignal(cu.rribcFlipType == 1);
        }
#endif 
        Distortion distortionChromaTmp = 0;
        for (int comp = 1; comp < numValidComponents; comp++)
        {
          const ComponentID compID = ComponentID(comp);
          distortionChromaTmp += m_pcRdCost->getDistPart(cs.getOrgBuf(compID), interCcpMergePredBuf[comp - 1], sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
        }
        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();
        m_CABACEstimator->inter_ccp_merge_root_cbf_zero(cu);
        const uint64_t bits = m_CABACEstimator->getEstFracBits();
        double cost = m_pcRdCost->calcRdCost(bits, distortionChromaTmp);
        if (cost < bestCost)
        {
          bestCost = cost;
          bestWIdx = wIdx;
          distortion = distortionChromaTmp + distortionLuma;
          cs.getRecoBuf(cu.blocks[COMPONENT_Cb]).copyClip(interCcpMergePredBuf[0], cs.slice->clpRng(COMPONENT_Cb));
          cs.getRecoBuf(cu.blocks[COMPONENT_Cr]).copyClip(interCcpMergePredBuf[1], cs.slice->clpRng(COMPONENT_Cr));
#if NNVC_USE_PRED
          cs.getPredBufCustom(cu.blocks[COMPONENT_Cb]).copyFrom(cs.getRecoBuf(cu.blocks[COMPONENT_Cb]));
          cs.getPredBufCustom(cu.blocks[COMPONENT_Cr]).copyFrom(cs.getRecoBuf(cu.blocks[COMPONENT_Cr]));
#endif
        }
      }
      cu.interCcpMergeZeroRootCbfIdc = bestWIdx;
      cu.firstPU->idxNonLocalCCP = 0;
      cu.firstTU->curCand        = {};
      cu.firstTU->curCand.type   = CCP_TYPE_NONE;
      m_CABACEstimator->getCtx() = ctxStart;
      }
#endif
    m_CABACEstimator->resetBits();

    PredictionUnit &pu = *cs.getPU( partitioner.chType );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_CABACEstimator->cu_skip_flag  ( cu, partitioner );
#else
    m_CABACEstimator->cu_skip_flag  ( cu );
#endif
    m_CABACEstimator->merge_data(pu);
#if INTER_LIC
    m_CABACEstimator->cu_lic_flag(cu);
#endif

    cs.dist     = distortion;
    cs.fracBits = m_CABACEstimator->getEstFracBits();
    cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
#if JVET_AA0133_INTER_MTS_OPT
    return true;
#else
    return;
#endif
  }

  //  Residual coding.
  if( luma )
  {
    if( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() )
    {
#if JVET_Y0065_GPM_INTRA
      if( !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC( cu ) )
#else
      if( !cu.firstPU->ciipFlag && !CU::isIBC( cu ) )
#endif
      {
        cs.getResiBuf( COMPONENT_Y ).rspSignalAllAndSubtract( cs.getOrgBuf( COMPONENT_Y ), cs.getPredBuf( COMPONENT_Y ), m_pcReshape->getFwdLUT() );
      }
      else
      {
#if JVET_AA0070_RRIBC
        if (CU::isIBC(cu) && cu.rribcFlipType)
        {
          PelBuf   tmpPattern;
          CompArea tmpArea(COMPONENT_Y, cu.chromaFormat, Position(0, 0), Size(cs.getOrgBuf(COMPONENT_Y).width, cs.getOrgBuf(COMPONENT_Y).height));
          tmpPattern = m_tmpStorageLCU.getBuf(tmpArea);
          tmpPattern.copyFrom(cs.getOrgBuf(COMPONENT_Y));
          tmpPattern.flipSignal(cu.rribcFlipType == 1);
          cs.getResiBuf(COMPONENT_Y).rspSignalAndSubtract(tmpPattern, cs.getPredBuf(COMPONENT_Y), m_pcReshape->getFwdLUT());
        }
        else
#endif
        cs.getResiBuf( COMPONENT_Y ).rspSignalAndSubtract( cs.getOrgBuf( COMPONENT_Y ), cs.getPredBuf( COMPONENT_Y ), m_pcReshape->getFwdLUT() );
      }
    }
    else
    {
#if JVET_AA0070_RRIBC
      if (CU::isIBC(cu) && cu.rribcFlipType)
      {
        PelBuf   tmpPattern;
        CompArea tmpArea(COMPONENT_Y, cu.chromaFormat, Position(0, 0), Size(cs.getOrgBuf(COMPONENT_Y).width, cs.getOrgBuf(COMPONENT_Y).height));
        tmpPattern = m_tmpStorageLCU.getBuf(tmpArea);
        tmpPattern.copyFrom(cs.getOrgBuf(COMPONENT_Y));
        tmpPattern.flipSignal(cu.rribcFlipType == 1);
        cs.getResiBuf(COMPONENT_Y).subtract(tmpPattern, cs.getPredBuf(COMPONENT_Y));
      }
      else
#endif
      cs.getResiBuf( COMPONENT_Y ).subtract( cs.getOrgBuf( COMPONENT_Y ), cs.getPredBuf( COMPONENT_Y ) );
    }
  }

  if( chroma && isChromaEnabled( cs.pcv->chrFormat ) )
  {
#if JVET_AA0070_RRIBC
    if (CU::isIBC(cu) && cu.rribcFlipType)
    {
      PelBuf   tmpPattern;
      CompArea tmpArea1(COMPONENT_Cb, cu.chromaFormat, Position(0, 0), Size(cs.getOrgBuf(COMPONENT_Cb).width, cs.getOrgBuf(COMPONENT_Cb).height));
      tmpPattern = m_tmpStorageLCU.getBuf(tmpArea1);
      tmpPattern.copyFrom(cs.getOrgBuf(COMPONENT_Cb));
      tmpPattern.flipSignal(cu.rribcFlipType == 1);
      cs.getResiBuf(COMPONENT_Cb).subtract(tmpPattern, cs.getPredBuf(COMPONENT_Cb));

      CompArea tmpArea2(COMPONENT_Cr, cu.chromaFormat, Position(0, 0), Size(cs.getOrgBuf(COMPONENT_Cr).width, cs.getOrgBuf(COMPONENT_Cr).height));
      tmpPattern = m_tmpStorageLCU.getBuf(tmpArea2);
      tmpPattern.copyFrom(cs.getOrgBuf(COMPONENT_Cr));
      tmpPattern.flipSignal(cu.rribcFlipType == 1);
      cs.getResiBuf(COMPONENT_Cr).subtract(tmpPattern, cs.getPredBuf(COMPONENT_Cr));
    }
    else
    {
#endif
    cs.getResiBuf( COMPONENT_Cb ).subtract( cs.getOrgBuf( COMPONENT_Cb ), cs.getPredBuf( COMPONENT_Cb ) );
    cs.getResiBuf( COMPONENT_Cr ).subtract( cs.getOrgBuf( COMPONENT_Cr ), cs.getPredBuf( COMPONENT_Cr ) );
#if JVET_AA0070_RRIBC
    }
#endif
  }

  const UnitArea curUnitArea = partitioner.currArea();
  CodingStructure &saveCS = *m_pSaveCS[1];
  saveCS.pcv = cs.pcv;
  saveCS.picture = cs.picture;
#if JVET_Z0118_GDR
  saveCS.m_pt = cs.m_pt;
#endif
  saveCS.area.repositionTo(curUnitArea);
  saveCS.clearCUs();
  saveCS.clearPUs();
  saveCS.clearTUs();
  for (const auto &ppcu : cs.cus)
  {
    CodingUnit &pcu = saveCS.addCU(*ppcu, ppcu->chType);
    pcu = *ppcu;
  }
  for (const auto &ppu : cs.pus)
  {
    PredictionUnit &pu = saveCS.addPU(*ppu, ppu->chType);
    pu = *ppu;
  }

#if JVET_S0234_ACT_CRS_FIX
  PelUnitBuf orgResidual;
#else
  PelUnitBuf orgResidual, colorTransResidual;
#endif
  const UnitArea localUnitArea(cs.area.chromaFormat, Area(0, 0, cu.Y().width, cu.Y().height));
  orgResidual = m_colorTransResiBuf[0].getBuf(localUnitArea);
#if !JVET_S0234_ACT_CRS_FIX
  colorTransResidual = m_colorTransResiBuf[1].getBuf(localUnitArea);
#endif
  orgResidual.copyFrom(cs.getResiBuf());
#if !JVET_S0234_ACT_CRS_FIX
  if (colorTransAllowed)
  {
    cs.getResiBuf().colorSpaceConvert(colorTransResidual, true, cu.cs->slice->clpRng(COMPONENT_Y));
  }
#endif

  const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
  int           numAllowedColorSpace = (colorTransAllowed ? 2 : 1);
  Distortion    zeroDistortion = 0;

  double  bestCost = MAX_DOUBLE;
  bool    bestColorTrans = false;
  bool    bestRootCbf = false;
  uint8_t bestsbtInfo = 0;
  uint8_t orgSbtInfo = cu.sbtInfo;
  int     bestIter = 0;

  auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl);
  bool rootCbfFirstColorSpace = true;

  for (int iter = 0; iter < numAllowedColorSpace; iter++)
  {
    if (colorTransAllowed && !m_pcEncCfg->getRGBFormatFlag() && iter)
    {
      continue;
    }
    char colorSpaceOption = blkCache->getSelectColorSpaceOption(cu);
    if (colorTransAllowed)
    {
      if (colorSpaceOption)
      {
        CHECK(colorSpaceOption > 2 || colorSpaceOption < 0, "invalid color space selection option");
        if (colorSpaceOption == 1 && iter)
        {
          continue;
        }
        if (colorSpaceOption == 2 && !iter)
        {
          continue;
        }
      }
    }
    if (!colorSpaceOption)
    {
      if (iter && !rootCbfFirstColorSpace)
      {
        continue;
      }
      if (colorTransAllowed && cs.bestParent && cs.bestParent->tmpColorSpaceCost != MAX_DOUBLE)
      {
        if (cs.bestParent->firstColorSpaceSelected && iter)
        {
          continue;
        }
        if (m_pcEncCfg->getRGBFormatFlag())
        {
          if (!cs.bestParent->firstColorSpaceSelected && !iter)
          {
            continue;
          }
        }
      }
    }
    bool colorTransFlag = (colorTransAllowed && m_pcEncCfg->getRGBFormatFlag()) ? (1 - iter) : iter;
    cu.colorTransform = colorTransFlag;
    cu.sbtInfo = orgSbtInfo;

    m_CABACEstimator->resetBits();
    m_CABACEstimator->getCtx() = ctxStart;
    cs.clearTUs();
    cs.fracBits = 0;
    cs.dist = 0;
    cs.cost = 0;

  if (colorTransFlag)
  {
#if JVET_S0234_ACT_CRS_FIX
    cs.getOrgResiBuf().bufs[0].copyFrom(orgResidual.bufs[0]);
    cs.getOrgResiBuf().bufs[1].copyFrom(orgResidual.bufs[1]);
    cs.getOrgResiBuf().bufs[2].copyFrom(orgResidual.bufs[2]);
#else
    cs.getOrgResiBuf().bufs[0].copyFrom(colorTransResidual.bufs[0]);
    cs.getOrgResiBuf().bufs[1].copyFrom(colorTransResidual.bufs[1]);
    cs.getOrgResiBuf().bufs[2].copyFrom(colorTransResidual.bufs[2]);
#endif

    memset(m_pTempPel, 0, sizeof(Pel) * localUnitArea.blocks[0].area());
    zeroDistortion = 0;
    for (int compIdx = 0; compIdx < 3; compIdx++)
    {
      ComponentID componentID = (ComponentID)compIdx;
      const CPelBuf zeroBuf(m_pTempPel, localUnitArea.blocks[compIdx]);
      zeroDistortion += m_pcRdCost->getDistPart(zeroBuf, orgResidual.bufs[compIdx], sps.getBitDepth(toChannelType(componentID)), componentID, DF_SSE);
    }
    xEstimateInterResidualQT(cs, partitioner, NULL, luma, chroma, &orgResidual);
  }
  else
  {
    zeroDistortion = 0;
    if (luma)
    {
      cs.getOrgResiBuf().bufs[0].copyFrom(orgResidual.bufs[0]);
    }
    if (chroma && isChromaEnabled(cs.pcv->chrFormat))
    {
      cs.getOrgResiBuf().bufs[1].copyFrom(orgResidual.bufs[1]);
      cs.getOrgResiBuf().bufs[2].copyFrom(orgResidual.bufs[2]);
    }
#if JVET_AA0133_INTER_MTS_OPT
    bool isValidReturn = xEstimateInterResidualQT(cs, partitioner, &zeroDistortion, luma, chroma);
    if (cu.mtsFlag && !isValidReturn)
    {
      return false;
    }
#if JVET_AG0061_INTER_LFNST_NSPT
    if (cu.lfnstFlag && !isValidReturn)
    {
      return false;
    }
#endif
#else
    xEstimateInterResidualQT(cs, partitioner, &zeroDistortion, luma, chroma);
#endif
  }
  TransformUnit &firstTU = *cs.getTU( partitioner.chType );

  cu.rootCbf = false;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->rqt_root_cbf( cu );
  const uint64_t  zeroFracBits = m_CABACEstimator->getEstFracBits();
  double zeroCost;
  {
#if WCG_EXT
    if( m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
    {
      zeroCost = m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion, false );
    }
    else
#endif
    zeroCost = m_pcRdCost->calcRdCost( zeroFracBits, zeroDistortion );
  }

  const int  numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
  for (uint32_t i = 0; i < numValidTBlocks; i++)
  {
    cu.rootCbf |= TU::getCbfAtDepth(firstTU, ComponentID(i), 0);
  }

  // -------------------------------------------------------
  // If a block full of 0's is efficient, then just use 0's.
  // The costs at this point do not include header bits.

  if (zeroCost < cs.cost || !cu.rootCbf)
  {
    cs.cost = zeroCost;
    cu.colorTransform = false;
    cu.sbtInfo = 0;
    cu.rootCbf = false;

    cs.clearTUs();

    // add new "empty" TU(s) spanning the whole CU
    cs.addEmptyTUs( partitioner );
  }
  if (!iter)
  {
    rootCbfFirstColorSpace = cu.rootCbf;
  }
  if (cs.cost < bestCost)
  {
    bestIter = iter;
#if !JVET_S0234_ACT_CRS_FIX
    if (cu.rootCbf && cu.colorTransform)
    {
      cs.getResiBuf(curUnitArea).colorSpaceConvert(cs.getResiBuf(curUnitArea), false, cu.cs->slice->clpRng(COMPONENT_Y));
    }
#endif

    if (iter != (numAllowedColorSpace - 1))
    {
      bestCost = cs.cost;
      bestColorTrans = cu.colorTransform;
      bestRootCbf = cu.rootCbf;
      bestsbtInfo = cu.sbtInfo;

      saveCS.clearTUs();
      for (const auto &ptu : cs.tus)
      {
        TransformUnit &tu = saveCS.addTU(*ptu, ptu->chType);
        tu = *ptu;
      }
      saveCS.getResiBuf(curUnitArea).copyFrom(cs.getResiBuf(curUnitArea));
    }
  }
  }

  if (bestIter != (numAllowedColorSpace - 1))
  {
    cu.colorTransform = bestColorTrans;
    cu.rootCbf = bestRootCbf;
    cu.sbtInfo = bestsbtInfo;

    cs.clearTUs();
    for (const auto &ptu : saveCS.tus)
    {
      TransformUnit &tu = cs.addTU(*ptu, ptu->chType);
      tu = *ptu;
    }
    cs.getResiBuf(curUnitArea).copyFrom(saveCS.getResiBuf(curUnitArea));
  }

#if !JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  uint64_t finalFracBits = xGetSymbolFracBitsInter( cs, partitioner );
  // we've now encoded the CU, and so have a valid bit cost
#endif

  if (!cu.rootCbf)
  {
    if (luma)
    {
      cs.getResiBuf().bufs[0].fill(0); // Clear the residual image, if we didn't code it.
    }
    if (chroma && isChromaEnabled(cs.pcv->chrFormat))
    {
      cs.getResiBuf().bufs[1].fill(0); // Clear the residual image, if we didn't code it.
      cs.getResiBuf().bufs[2].fill(0); // Clear the residual image, if we didn't code it.
    }
  }

  if (luma)
  {
    if (cu.rootCbf && cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
#if JVET_Y0065_GPM_INTRA
      if( !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC( cu ) )
#else
      if( !cu.firstPU->ciipFlag && !CU::isIBC( cu ) )
#endif
      {
        const CompArea &areaY = cu.Y();
        CompArea      tmpArea( COMPONENT_Y, areaY.chromaFormat, Position( 0, 0 ), areaY.size() );
        PelBuf tmpPred = m_tmpStorageLCU.getBuf( tmpArea );
        tmpPred.rspSignal( cs.getPredBuf( COMPONENT_Y ), m_pcReshape->getFwdLUT() );

#if JVET_AG0145_ADAPTIVE_CLIPPING
        ClpRng clpRng = cs.slice->clpRng(COMPONENT_Y);
        std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
        clpRng.min = fwdLUT[cs.slice->getLumaPelMin()];
        clpRng.max = fwdLUT[cs.slice->getLumaPelMax()];
        cs.getRecoBuf(COMPONENT_Y).reconstruct(tmpPred, cs.getResiBuf(COMPONENT_Y), clpRng);
#else
        cs.getRecoBuf( COMPONENT_Y ).reconstruct( tmpPred, cs.getResiBuf( COMPONENT_Y ), cs.slice->clpRng( COMPONENT_Y ) );
#endif
      }
      else
      {
#if JVET_AG0145_ADAPTIVE_CLIPPING
        ClpRng clpRng = cs.slice->clpRng(COMPONENT_Y);
        if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
          clpRng.min = fwdLUT[cs.slice->getLumaPelMin()];
          clpRng.max = fwdLUT[cs.slice->getLumaPelMax()];
        }
        else
        {
          clpRng.min = cs.slice->getLumaPelMin();
          clpRng.max = cs.slice->getLumaPelMax();
        }
        cs.getRecoBuf(COMPONENT_Y)
          .reconstruct(cs.getPredBuf(COMPONENT_Y), cs.getResiBuf(COMPONENT_Y), clpRng);
#else
        cs.getRecoBuf(COMPONENT_Y)
          .reconstruct(cs.getPredBuf(COMPONENT_Y), cs.getResiBuf(COMPONENT_Y), cs.slice->clpRng(COMPONENT_Y));
#endif
#if JVET_AA0070_RRIBC
        if (CU::isIBC(cu) && cu.rribcFlipType)
        {
          cs.getRecoBuf(COMPONENT_Y).flipSignal(cu.rribcFlipType == 1);
        }
#endif
      }
    }
    else
    {
#if JVET_AG0145_ADAPTIVE_CLIPPING
      ClpRng clpRng = cs.slice->clpRng(COMPONENT_Y);
      if (cs.slice->getSPS()->getUseLmcs() && cs.slice->getLmcsEnabledFlag())
      {
        if (m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
        {
          clpRng.min = cs.slice->getLumaPelMin();
          clpRng.max = cs.slice->getLumaPelMax();
        }
        else
        {
          std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
          clpRng.min = fwdLUT[cs.slice->getLumaPelMin()];
          clpRng.max = fwdLUT[cs.slice ->getLumaPelMax()];
        }
      }
      else
      {
        clpRng.min = cs.slice->getLumaPelMin();
        clpRng.max = cs.slice->getLumaPelMax();
      }
      cs.getRecoBuf().bufs[0].reconstruct(cs.getPredBuf().bufs[0], cs.getResiBuf().bufs[0], clpRng);
#else
      cs.getRecoBuf().bufs[0].reconstruct(cs.getPredBuf().bufs[0], cs.getResiBuf().bufs[0], cs.slice->clpRngs().comp[0]);
#endif
#if JVET_Y0065_GPM_INTRA
      if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
#else
      if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !CU::isIBC(cu))
#endif
      {
        cs.getRecoBuf().bufs[0].rspSignal(m_pcReshape->getFwdLUT());
      }
#if JVET_AG0145_ADAPTIVE_CLIPPING
      ClpRng clpRngTmp = cs.slice->clpRng(COMPONENT_Y);
      if (cs.slice->getSPS()->getUseLmcs() && cs.slice->getLmcsEnabledFlag())
      {
        std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
        clpRngTmp.min = fwdLUT[cs.slice->getLumaPelMin()];
        clpRngTmp.max = fwdLUT[cs.slice->getLumaPelMax()];
      }
      else
      {
        clpRngTmp.min = cs.slice->getLumaPelMin();
        clpRngTmp.max = cs.slice->getLumaPelMax();
      }
      cs.getRecoBuf().bufs[0].copyClip(cs.getRecoBuf().bufs[0], clpRngTmp);
#endif
#if JVET_AA0070_RRIBC
      if (CU::isIBC(cu) && cu.rribcFlipType)
      {
        cs.getRecoBuf().bufs[0].flipSignal(cu.rribcFlipType == 1);
      }
#endif
    }
  }

#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
#if MULTI_HYP_PRED
  if (cu.cs->slice->getSPS()->getUseInterCcpMergeZeroLumaCbf() && cu.firstPU->mergeFlag && !cu.rootCbf && cu.firstPU->numMergedAddHyps == cu.firstPU->addHypData.size())
#else
  if (cu.firstPU->mergeFlag && !cu.rootCbf)
#endif
  {
    cu.skip = true;
  }
  bool isInterCcpMergeRootCbfZeroAllowed = CU::interCcpMergeZeroRootCbfAllowed(cu);
  if(isInterCcpMergeRootCbfZeroAllowed && m_pcEncCfg->getInterCcpMergeZeroLumaCbfFastMode())
  {
    if(cu.blocks[COMPONENT_Cb].area() < 16 || cu.blocks[COMPONENT_Cb].area() > 1024)
    {
      if ((cu.skip && !cu.slice->getCheckLDB()) || !cu.skip)
      {
        isInterCcpMergeRootCbfZeroAllowed = false;
      }
    }
  }

  if (chroma && isChromaEnabled(cs.pcv->chrFormat) && isInterCcpMergeRootCbfZeroAllowed)
  {
    PelBuf bufCb = cs.getPredBuf(cu.blocks[COMPONENT_Cb]);
    PelBuf bufCr = cs.getPredBuf(cu.blocks[COMPONENT_Cr]);
    if (m_isInterCcpModelReady == false)
    {
      m_pcIntraPred->xAddOnTheFlyCalcCCPCands4InterBlk(*cu.firstPU, cu.blocks[COMPONENT_Cb], m_interCcpMergeList, m_validNum);
      m_isInterCcpModelReady = true;
    }
    const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
    double bestCost = MAX_DOUBLE;
    int8_t bestWIdx = 0;
    for (int8_t wIdx = 0; wIdx <= MAX_CCP_MERGE_WEIGHT_IDX; wIdx++)
    {
      cu.interCcpMergeZeroRootCbfIdc = wIdx;
      if (wIdx == 0)
      {
        const bool valid = deriveInterCcpMergePrediction(cu.firstTU, cs.getRecoBuf(cu.blocks[COMPONENT_Y]), bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], m_interCcpMergeList, m_validNum);
        CHECK(!valid, "invalid inter ccp merge for rootCbf=0");
      }
      else
      {
        m_pcIntraPred->combineCcpAndInter(*cu.firstPU, bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], true);
      }
#if JVET_AA0070_RRIBC
      if (CU::isIBC(cu) && cu.rribcFlipType)
      {
        interCcpMergePredBuf[0].flipSignal(cu.rribcFlipType == 1);
        interCcpMergePredBuf[1].flipSignal(cu.rribcFlipType == 1);
      }
#endif
      // given zero root cbf, we should not have residuals here
      Distortion distChroma = 0;
      for (int comp = 1; comp < numValidComponents; comp++)
      {
        const ComponentID compID = ComponentID(comp);
        distChroma += m_pcRdCost->getDistPart(cs.getOrgBuf(compID), interCcpMergePredBuf[comp - 1], sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
      }
      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();
      m_CABACEstimator->inter_ccp_merge_root_cbf_zero(cu);
      const uint64_t bits = m_CABACEstimator->getEstFracBits();
      double cost = m_pcRdCost->calcRdCost(bits, distChroma);
      if (cost < bestCost)
      {
        bestCost = cost;
        bestWIdx = wIdx;
        cs.getRecoBuf(cu.blocks[COMPONENT_Cb]).copyClip(interCcpMergePredBuf[0], cs.slice->clpRng(COMPONENT_Cb));
        cs.getRecoBuf(cu.blocks[COMPONENT_Cr]).copyClip(interCcpMergePredBuf[1], cs.slice->clpRng(COMPONENT_Cr));
#if NNVC_USE_PRED
        cs.getPredBufCustom(cu.blocks[COMPONENT_Cb]).copyFrom(cs.getRecoBuf(cu.blocks[COMPONENT_Cb]));
        cs.getPredBufCustom(cu.blocks[COMPONENT_Cr]).copyFrom(cs.getRecoBuf(cu.blocks[COMPONENT_Cr]));
#endif
      }
    }
    cu.interCcpMergeZeroRootCbfIdc = bestWIdx;

    cu.firstPU->idxNonLocalCCP = 0;
    cu.firstTU->curCand = {};
    cu.firstTU->curCand.type = CCP_TYPE_NONE;
  }
  else
#endif
  if (chroma && isChromaEnabled(cs.pcv->chrFormat))
  {
#if JVET_AE0059_INTER_CCCM
#if JVET_AF0073_INTER_CCP_MERGE
    for (auto& tuTmp : cs.tus)
#else
    for (const auto& tuTmp : cs.tus)
#endif
    {
      if( tuTmp->interCccm && tuTmp->blocks[COMPONENT_Cb].valid() )
      {
        PelBuf lumaPredBuf( m_interCccmStorage[0], tuTmp->blocks[COMPONENT_Y] );
        lumaPredBuf.copyFrom( cs.getPredBuf( tuTmp->blocks[COMPONENT_Y] ) );

        if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC( cu ) )
        {
          lumaPredBuf.rspSignal( m_pcReshape->getFwdLUT() ); // so that matches with resibuf domain
        }

        PelBuf bufCb = cs.getPredBuf( tuTmp->blocks[COMPONENT_Cb] );
        PelBuf bufCr = cs.getPredBuf( tuTmp->blocks[COMPONENT_Cr] );
        PelBuf interCccmPredBuf[3];

        for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
        {
          interCccmPredBuf[i] = PelBuf( m_interCccmStorage[i + 3], tuTmp->blocks[ComponentID( i )] );
        }

        const bool valid = deriveInterCccmPrediction( tuTmp, lumaPredBuf, cs.getRecoBuf( tuTmp->blocks[COMPONENT_Y] ), bufCb, bufCr, interCccmPredBuf[COMPONENT_Cb], interCccmPredBuf[COMPONENT_Cr] );

        CHECK( !valid, "invalid inter cccm" );

#if NNVC_USE_PRED
        cs.getPredBufCustom(tuTmp->blocks[COMPONENT_Cb]).copyFrom(interCccmPredBuf[COMPONENT_Cb]);
        cs.getPredBufCustom(tuTmp->blocks[COMPONENT_Cr]).copyFrom(interCccmPredBuf[COMPONENT_Cr]);
#endif
        cs.getRecoBuf( tuTmp->blocks[COMPONENT_Cb] ).reconstruct( interCccmPredBuf[COMPONENT_Cb], cs.getResiBuf( tuTmp->blocks[COMPONENT_Cb] ), cs.slice->clpRngs().comp[COMPONENT_Cb] );
        cs.getRecoBuf( tuTmp->blocks[COMPONENT_Cr] ).reconstruct( interCccmPredBuf[COMPONENT_Cr], cs.getResiBuf( tuTmp->blocks[COMPONENT_Cr] ), cs.slice->clpRngs().comp[COMPONENT_Cr] );
      }
#if JVET_AF0073_INTER_CCP_MERGE
      else if( tuTmp->interCcpMerge && tuTmp->blocks[COMPONENT_Cb].valid() )
      {
        PelBuf bufCb = cs.getPredBuf( tuTmp->blocks[COMPONENT_Cb] );
        PelBuf bufCr = cs.getPredBuf( tuTmp->blocks[COMPONENT_Cr] );
#if !JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
        PelBuf interCcpMergePredBuf[2];

        for( int i = 1; i < MAX_NUM_COMPONENT; i++ )
        {
          interCcpMergePredBuf[i-1] = PelBuf( m_interCcpMergeStorage[i-1], tuTmp->blocks[ComponentID(i)] );
        }
#endif
        const bool valid = deriveInterCcpMergePrediction(tuTmp, cs.getRecoBuf( tuTmp->blocks[COMPONENT_Y] ), bufCb, bufCr, interCcpMergePredBuf[0], interCcpMergePredBuf[1], m_interCcpMergeList, m_validNum);

        CHECK( !valid, "invalid inter ccp merge" );

#if NNVC_USE_PRED
        cs.getPredBufCustom(tuTmp->blocks[COMPONENT_Cb]).copyFrom(interCcpMergePredBuf[0]);
        cs.getPredBufCustom(tuTmp->blocks[COMPONENT_Cr]).copyFrom(interCcpMergePredBuf[1]);
#endif
        cs.getRecoBuf( tuTmp->blocks[COMPONENT_Cb] ).reconstruct( interCcpMergePredBuf[0], cs.getResiBuf( tuTmp->blocks[COMPONENT_Cb] ), cs.slice->clpRngs().comp[COMPONENT_Cb] );
        cs.getRecoBuf( tuTmp->blocks[COMPONENT_Cr] ).reconstruct( interCcpMergePredBuf[1], cs.getResiBuf( tuTmp->blocks[COMPONENT_Cr] ), cs.slice->clpRngs().comp[COMPONENT_Cr] );
      }
#endif
      else
      {
        cs.getRecoBuf(tuTmp->blocks[COMPONENT_Cb]).reconstruct(cs.getPredBuf(tuTmp->blocks[COMPONENT_Cb]), cs.getResiBuf(tuTmp->blocks[COMPONENT_Cb]),cs.slice->clpRngs().comp[COMPONENT_Cb]);
        cs.getRecoBuf(tuTmp->blocks[COMPONENT_Cr]).reconstruct(cs.getPredBuf(tuTmp->blocks[COMPONENT_Cr]), cs.getResiBuf(tuTmp->blocks[COMPONENT_Cr]), cs.slice->clpRngs().comp[COMPONENT_Cr]);
      }
    }
#else
    cs.getRecoBuf().bufs[1].reconstruct(cs.getPredBuf().bufs[1], cs.getResiBuf().bufs[1], cs.slice->clpRngs().comp[1]);
    cs.getRecoBuf().bufs[2].reconstruct(cs.getPredBuf().bufs[2], cs.getResiBuf().bufs[2], cs.slice->clpRngs().comp[2]);
#endif
#if JVET_AA0070_RRIBC
    if (CU::isIBC(cu) && cu.rribcFlipType)
    {
      cs.getRecoBuf().bufs[1].flipSignal(cu.rribcFlipType == 1);
      cs.getRecoBuf().bufs[2].flipSignal(cu.rribcFlipType == 1);
    }
#endif
  }
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  uint64_t finalFracBits = xGetSymbolFracBitsInter( cs, partitioner );
  // we've now encoded the CU, and so have a valid bit cost
#endif

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)
  Distortion finalDistortion = 0;

  for (int comp = 0; comp < numValidComponents; comp++)
  {
    const ComponentID compID = ComponentID(comp);
    if (compID == COMPONENT_Y && !luma)
      continue;
    if (compID != COMPONENT_Y && !chroma)
      continue;
    CPelBuf reco = cs.getRecoBuf (compID);
    CPelBuf org  = cs.getOrgBuf  (compID);
#if JVET_V0094_BILATERAL_FILTER
    const CompArea &areaY = cu.Y();
    CompArea      tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
    PelBuf tmpRecLuma;
    if(isLuma(compID))
    {
      tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
      tmpRecLuma.copyFrom(reco);
      if(m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() ) && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
      {
        tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
      }
      
      if(cs.pps->getUseBIF() && isLuma(compID))
      {
        for (auto &currTU : CU::traverseTUs(cu))
        { 
#if JVET_AF0112_BIF_DYNAMIC_SCALING
          bool applyBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
          bool isInter = (cu.predMode == MODE_INTER) ? true : false;
          bool applyBIF = ((TU::getCbf(currTU, compID) || !isInter) && currTU.cu->qp > 17 && 128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height) && (!isInter || 32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
          Position tuPosInCu = currTU.lumaPos() - cu.lumaPos();
          if(applyBIF)
          {
            PelBuf tmpSubBuf = tmpRecLuma.subBuf(tuPosInCu, currTU.lumaSize());
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            
            // Only reshape surrounding samples if reshaping is on
            if( m_pcEncCfg->getLmcs() && cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() )
            {
              m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng( compID ), currTU, true, true, &m_pcReshape->getInvLUT() );
            }
            else
            {
              std::vector<Pel> invLUT;
              m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng( compID ), currTU, true, false, &invLUT );
            }
          }
          else
          {
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            recIPredBuf.copyFrom(reco.subBuf(tuPosInCu, currTU.lumaSize()));
          }
        }
      }
    }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    PelBuf tmpRecChroma;
    if(isChroma(compID))
    {
      const CompArea &area = cu.blocks[compID];
      CompArea      tmpArea2( compID, area.chromaFormat, Position(0, 0), area.size() );
      tmpRecChroma = m_tmpStorageLCU.getBuf(tmpArea2);
      tmpRecChroma.copyFrom(reco);

      if(cs.pps->getUseChromaBIF() && isChroma(compID))
      {
        for (auto &currTU : CU::traverseTUs(cu))
        {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
          bool applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
          bool isInter = (cu.predMode == MODE_INTER) ? true : false;
          bool applyChromaBIF = (TU::getCbf(currTU, compID) || isInter == false) && (cu.qp > 17);
#endif
          Position tuPosInCu = currTU.chromaPos() - cu.chromaPos();
          PelBuf tmpSubBuf = tmpRecChroma.subBuf(tuPosInCu, currTU.chromaSize());
          if (applyChromaBIF)
          {
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng(compID), currTU, true );
          }
          else
          {
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            recIPredBuf.copyFrom(tmpSubBuf);
          }
        }
      }
    }
#endif
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
      m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      if (compID == COMPONENT_Y )
      {
        //        if(!(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        //        {
        //          tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
        //        }
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      {
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecChroma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
#else
        finalDistortion += m_pcRdCost->getDistPart(org, reco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
#endif
    }
    else
#endif
    {
      if (compID == COMPONENT_Y )
      {
        finalDistortion += m_pcRdCost->getDistPart( org, tmpRecLuma, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
      }
      else
      {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        finalDistortion += m_pcRdCost->getDistPart( org, tmpRecChroma, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
#else
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
#endif
      }
    }
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    PelBuf tmpRecChroma;
    if(isChroma(compID))
    {
      const CompArea &area = cu.blocks[compID];
      CompArea      tmpArea2( compID, area.chromaFormat, Position(0, 0), area.size());
      tmpRecChroma = m_tmpStorageLCU.getBuf(tmpArea2);
      tmpRecChroma.copyFrom(reco);
      if(cs.pps->getUseChromaBIF() && isChroma(compID))
      {
        for (auto &currTU : CU::traverseTUs(cu))
        {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
          bool applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
          bool isInter = (cu.predMode == MODE_INTER) ? true : false;
          bool applyChromaBIF = (TU::getCbf(currTU, compID) || isInter == false) && (cu.qp > 17);
#endif
          if (applyChromaBIF)
          {
            Position tuPosInCu = currTU.chromaPos() - cu.chromaPos();
            PelBuf tmpSubBuf = tmpRecChroma.subBuf(tuPosInCu, currTU.chromaSize());
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng(compID), currTU, true );
          }
          else
          {
            CompArea compArea = currTU.blocks[compID];
            PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
            recIPredBuf.copyFrom(tmpSubBuf);
          }
        }
      }
    }
#endif
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()) )
      {
        const CompArea &areaY = cu.Y();
        CompArea      tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
        tmpRecLuma.rspSignal( reco, m_pcReshape->getInvLUT() );
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(isChroma(compID))
        {
          finalDistortion += m_pcRdCost->getDistPart(org, tmpRecChroma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
          finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
        }
#else
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
#endif
      }
    }
    else
#endif
    {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if(isChroma(compID))
      {
        finalDistortion += m_pcRdCost->getDistPart( org, tmpRecChroma, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
      }
      else
      {
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
      }
#else
      finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
#endif
    }
#endif
  }

  cs.dist     = finalDistortion;
  cs.fracBits = finalFracBits;
  cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
  if (cs.slice->getSPS()->getUseColorTrans())
  {
    if (cs.cost < cs.tmpColorSpaceCost)
    {
      cs.tmpColorSpaceCost = cs.cost;
      if (m_pcEncCfg->getRGBFormatFlag())
      {
        cs.firstColorSpaceSelected = cu.colorTransform || !cu.rootCbf;
      }
      else
      {
        cs.firstColorSpaceSelected = !cu.colorTransform || !cu.rootCbf;
      }
    }
  }

  CHECK(cs.tus.size() == 0, "No TUs present");
#if JVET_AA0133_INTER_MTS_OPT
  return true;
#endif
}

uint64_t InterSearch::xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner)
{
  uint64_t fracBits   = 0;
  CodingUnit &cu    = *cs.getCU( partitioner.chType );

  m_CABACEstimator->resetBits();

#if MULTI_HYP_PRED
  if (cu.firstPU->mergeFlag && !cu.rootCbf && cu.firstPU->numMergedAddHyps == cu.firstPU->addHypData.size())
#else
  if (cu.firstPU->mergeFlag && !cu.rootCbf)
#endif
  {
    cu.skip = true;
    CHECK(cu.colorTransform, "ACT should not be enabled for skip mode");
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_CABACEstimator->cu_skip_flag  ( cu, partitioner );
#else
    m_CABACEstimator->cu_skip_flag  ( cu );
#endif
    if (cu.firstPU->ciipFlag)
    {
      // CIIP shouldn't be skip, the upper level function will deal with it, i.e. setting the overall cost to MAX_DOUBLE
    }
    else
    {
      m_CABACEstimator->merge_data(*cu.firstPU);
    }
    fracBits   += m_CABACEstimator->getEstFracBits();
  }
  else
  {
    CHECK( cu.skip, "Skip flag has to be off at this point!" );

    if (cu.Y().valid())
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    m_CABACEstimator->cu_skip_flag( cu, partitioner );
    m_CABACEstimator->pred_mode   ( cu, partitioner );
#else
      m_CABACEstimator->cu_skip_flag( cu );
    m_CABACEstimator->pred_mode   ( cu );
#endif
    m_CABACEstimator->cu_pred_data( cu );
    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual ( cu, partitioner, cuCtx );
    fracBits       += m_CABACEstimator->getEstFracBits();
  }

  return fracBits;
}

double InterSearch::xGetMEDistortionWeight(uint8_t bcwIdx, RefPicList eRefPicList)
{
  if( bcwIdx != BCW_DEFAULT )
  {
    return fabs((double)getBcwWeight(bcwIdx, eRefPicList) / (double)g_bcwWeightBase);
  }
  else
  {
    return 0.5;
  }
}
bool InterSearch::xReadBufferedUniMv(PredictionUnit& pu, RefPicList eRefPicList, int32_t iRefIdx, Mv& pcMvPred, Mv& rcMv, uint32_t& ruiBits, Distortion& ruiCost)
{
  if (m_uniMotions.isReadMode((uint32_t)eRefPicList, (uint32_t)iRefIdx))
  {
    m_uniMotions.copyTo(rcMv, ruiCost, (uint32_t)eRefPicList, (uint32_t)iRefIdx);

    Mv pred = pcMvPred;
    pred.changeTransPrecInternal2Amvr(pu.cu->imv);
    m_pcRdCost->setPredictor(pred);
    m_pcRdCost->setCostScale(0);

    Mv mv = rcMv;
    mv.changeTransPrecInternal2Amvr(pu.cu->imv);
    uint32_t mvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);

    ruiBits += mvBits;
    ruiCost += m_pcRdCost->getCost(ruiBits);
    return true;
  }
  return false;
}

bool InterSearch::xReadBufferedAffineUniMv(PredictionUnit& pu, RefPicList eRefPicList, int32_t iRefIdx, Mv acMvPred[3], Mv acMv[3], uint32_t& ruiBits, Distortion& ruiCost
  , int& mvpIdx, const AffineAMVPInfo& aamvpi
)
{
  if (m_uniMotions.isReadModeAffine((uint32_t)eRefPicList, (uint32_t)iRefIdx, pu.cu->affineType))
  {
    m_uniMotions.copyAffineMvTo(acMv, ruiCost, (uint32_t)eRefPicList, (uint32_t)iRefIdx, pu.cu->affineType, mvpIdx);
    m_pcRdCost->setCostScale(0);
    acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
    acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
    acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

    uint32_t mvBits = 0;
    for (int verIdx = 0; verIdx<(pu.cu->affineType ? 3 : 2); verIdx++)
    {
      Mv pred = verIdx ? acMvPred[verIdx] + acMv[0] - acMvPred[0] : acMvPred[verIdx];
      pred.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      m_pcRdCost->setPredictor(pred);
      Mv mv = acMv[verIdx];
      mv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      mvBits += m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
    }
    ruiBits += mvBits;
    ruiCost += m_pcRdCost->getCost(ruiBits);
    return true;
  }
  return false;
}
void InterSearch::initWeightIdxBits()
{
  for (int n = 0; n < BCW_NUM; ++n)
  {
    m_estWeightIdxBits[n] = deriveWeightIdxBits(n);
  }
}

void InterSearch::xClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps )
{
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
  int horMax = ( pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1 ) << mvShift;
  int horMin = ( -( int ) sps.getMaxCUWidth()   - offset - ( int ) pos.x + 1 ) << mvShift;

  int verMax = ( pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1 ) << mvShift;
  int verMin = ( -( int ) sps.getMaxCUHeight()   - offset - ( int ) pos.y + 1 ) << mvShift;
  const SubPic &curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.getTreatedAsPicFlag() && m_clipMvInSubPic)
  {
    horMax = ((curSubPic.getSubPicRight() + 1)  + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.getMaxCUWidth()  - offset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << mvShift;

    verMax = ((curSubPic.getSubPicBottom() + 1) + offset -  (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.getMaxCUHeight() - offset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << mvShift;
  }
  if( pps.getWrapAroundEnabledFlag() )
  {
    int horMax = ( pps.getPicWidthInLumaSamples() + sps.getMaxCUWidth() - size.width + offset - (int)pos.x - 1 ) << mvShift;
    int horMin = ( -( int ) sps.getMaxCUWidth()                                      - offset - ( int ) pos.x + 1 ) << mvShift;
    rcMv.setHor( std::min( horMax, std::max( horMin, rcMv.getHor() ) ) );
    rcMv.setVer( std::min( verMax, std::max( verMin, rcMv.getVer() ) ) );
    return;
  }

  rcMv.setHor( std::min( horMax, std::max( horMin, rcMv.getHor() ) ) );
  rcMv.setVer( std::min( verMax, std::max( verMin, rcMv.getVer() ) ) );
}

uint32_t InterSearch::xDetermineBestMvp( PredictionUnit& pu, Mv acMvTemp[3], int& mvpIdx, const AffineAMVPInfo& aamvpi )
{
  bool mvpUpdated  = false;
  uint32_t minBits = std::numeric_limits<uint32_t>::max();
  for ( int i = 0; i < aamvpi.numCand; i++ )
  {
    Mv mvPred[3] = { aamvpi.mvCandLT[i], aamvpi.mvCandRT[i], aamvpi.mvCandLB[i] };
    uint32_t candBits = m_auiMVPIdxCost[i][aamvpi.numCand];
    candBits += xCalcAffineMVBits( pu, acMvTemp, mvPred );

    if ( candBits < minBits )
    {
      minBits    = candBits;
      mvpIdx     = i;
      mvpUpdated = true;
    }
  }
  CHECK( !mvpUpdated, "xDetermineBestMvp() error" );
  return minBits;
}

void InterSearch::symmvdCheckBestMvp(
  PredictionUnit& pu,
  PelUnitBuf& origBuf,
  Mv curMv,
  RefPicList curRefList,
  AMVPInfo amvpInfo[2][33],
  int32_t bcwIdx,
  Mv cMvPredSym[2],
  int32_t mvpIdxSym[2],
  Distortion& bestCost,
  bool skip
)
{
  RefPicList tarRefList = (RefPicList)(1 - curRefList);
  int32_t refIdxCur = pu.cu->slice->getSymRefIdx(curRefList);
  int32_t refIdxTar = pu.cu->slice->getSymRefIdx(tarRefList);

  MvField cCurMvField, cTarMvField;
  cCurMvField.setMvField(curMv, refIdxCur);
  AMVPInfo& amvpCur = amvpInfo[curRefList][refIdxCur];
  AMVPInfo& amvpTar = amvpInfo[tarRefList][refIdxTar];
#if TM_AMVP
  unsigned amvpNumCandCur = amvpCur.numCand;
  unsigned amvpNumCandTar = amvpTar.numCand;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (!pu.cs->sps->getUseTMAmvpMode())
#else
  if (!pu.cs->sps->getUseDMVDMode())
#endif
  {
    amvpNumCandCur = AMVP_MAX_NUM_CANDS;
    amvpNumCandTar = AMVP_MAX_NUM_CANDS;
  }
#endif
  m_pcRdCost->setCostScale(0);


  // get prediction of eCurRefPicList
  PelUnitBuf predBufA = m_tmpPredStorage[curRefList].getBuf(UnitAreaRelative(*pu.cu, pu));
  const Picture* picRefA = pu.cu->slice->getRefPic(curRefList, cCurMvField.refIdx);
  Mv mvA = cCurMvField.mv;
  clipMv( mvA, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if JVET_AD0213_LIC_IMP
  if ((mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 && !pu.cu->licFlag)
#else
  if ( (mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 )
#endif
  {
    Position offset = pu.blocks[COMPONENT_Y].pos().offset( mvA.getHor() >> 4, mvA.getVer() >> 4 );
    CPelBuf pelBufA = picRefA->getRecoBuf( CompArea( COMPONENT_Y, pu.chromaFormat, offset, pu.blocks[COMPONENT_Y].size() ), false );
    predBufA.bufs[0].buf = const_cast<Pel *>(pelBufA.buf);
    predBufA.bufs[0].stride = pelBufA.stride;
  }
  else
  {
    xPredInterBlk( COMPONENT_Y, pu, picRefA, mvA, predBufA, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false );
  }
  PelUnitBuf bufTmp = m_tmpStorageLCU.getBuf( UnitAreaRelative( *pu.cu, pu ) );
  bufTmp.copyFrom( origBuf );
  bufTmp.removeHighFreq( predBufA, m_pcEncCfg->getClipForBiPredMeEnabled(), pu.cu->slice->clpRngs(), getBcwWeight( pu.cu->bcwIdx, tarRefList ) );

  double fWeight = xGetMEDistortionWeight( pu.cu->bcwIdx, tarRefList );

  int32_t skipMvpIdx[2];
  skipMvpIdx[0] = skip ? mvpIdxSym[0] : -1;
  skipMvpIdx[1] = skip ? mvpIdxSym[1] : -1;

  for (int i = 0; i < amvpCur.numCand; i++)
  {
    for (int j = 0; j < amvpTar.numCand; j++)
    {
      if (skipMvpIdx[curRefList] == i && skipMvpIdx[tarRefList] == j)
        continue;

      cTarMvField.setMvField(curMv.getSymmvdMv(amvpCur.mvCand[i], amvpTar.mvCand[j]), refIdxTar);

      // get prediction of eTarRefPicList
      PelUnitBuf predBufB = m_tmpPredStorage[tarRefList].getBuf(UnitAreaRelative(*pu.cu, pu));
      const Picture* picRefB = pu.cu->slice->getRefPic(tarRefList, cTarMvField.refIdx);
      Mv mvB = cTarMvField.mv;
      clipMv( mvB, pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->sps, *pu.cs->pps );
#if JVET_AD0213_LIC_IMP
      if ((mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 && !pu.cu->licFlag)
#else
      if ( (mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 )
#endif
      {
        Position offset = pu.blocks[COMPONENT_Y].pos().offset( mvB.getHor() >> 4, mvB.getVer() >> 4 );
        CPelBuf pelBufB = picRefB->getRecoBuf( CompArea( COMPONENT_Y, pu.chromaFormat, offset, pu.blocks[COMPONENT_Y].size() ), false );
        predBufB.bufs[0].buf = const_cast<Pel *>(pelBufB.buf);
        predBufB.bufs[0].stride = pelBufB.stride;
      }
      else
      {
        xPredInterBlk( COMPONENT_Y, pu, picRefB, mvB, predBufB, false, pu.cu->slice->clpRng( COMPONENT_Y ), false, false );
      }
      // calc distortion
      DFunc distFunc = (!pu.cu->slice->getDisableSATDForRD()) ? DF_HAD : DF_SAD;
      Distortion cost = (Distortion)floor( fWeight * (double)m_pcRdCost->getDistPart( bufTmp.Y(), predBufB.Y(), pu.cs->sps->getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, distFunc ) );

      Mv pred = amvpCur.mvCand[i];
      pred.changeTransPrecInternal2Amvr(pu.cu->imv);
      m_pcRdCost->setPredictor(pred);
      Mv mv = curMv;
      mv.changeTransPrecInternal2Amvr(pu.cu->imv);
      uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
#if TM_AMVP
      bits += m_auiMVPIdxCost[i][amvpNumCandCur];
      bits += m_auiMVPIdxCost[j][amvpNumCandTar];
#else
      bits += m_auiMVPIdxCost[i][AMVP_MAX_NUM_CANDS];
      bits += m_auiMVPIdxCost[j][AMVP_MAX_NUM_CANDS];
#endif
      cost += m_pcRdCost->getCost(bits);
      if (cost < bestCost)
      {
        bestCost = cost;
        cMvPredSym[curRefList] = amvpCur.mvCand[i];
        cMvPredSym[tarRefList] = amvpTar.mvCand[j];
        mvpIdxSym[curRefList] = i;
        mvpIdxSym[tarRefList] = j;
      }
    }
  }
}

uint64_t InterSearch::xCalcPuMeBits(PredictionUnit& pu)
{
  CHECKD(!pu.mergeFlag, "");
  CHECKD(CU::isIBC(*pu.cu), "");

  m_CABACEstimator->resetBits();
  m_CABACEstimator->merge_flag(pu);
  if (pu.mergeFlag)
  {
    m_CABACEstimator->merge_data(pu);
#if MULTI_HYP_PRED
    m_CABACEstimator->mh_pred_data(pu);
#endif
  }
#if MULTI_HYP_PRED
  else if (pu.interDir == 3)
  {
    m_CABACEstimator->mh_pred_data(pu);
  }
#endif
  return m_CABACEstimator->getEstFracBits();
}

#if JVET_AD0213_LIC_IMP
void InterSearch::resetLicEncCtrlPara()
{
  updateMvNeeded = updateL1ZeroFlagMvNeeded = updateSMVDMvNeeded = isBDOFNotNeeded = false;
  amvpCand0 = amvpCand1 = amvpCand0Lic = amvpCand1Lic = nullptr;
  mvpIdx0 = mvpIdx1 = -1;
}

void InterSearch::setEncCtrlParaLicOff(CodingUnit& cu)
{
  CodingStructure& cs = *cu.cs;
  PredictionUnit&  pu = *cu.firstPU;
  isBDOFNotNeeded = (pu.interDir == 1 || pu.interDir == 2 || pu.cu->bcwIdx != BCW_DEFAULT || pu.cu->affine || pu.cu->smvdMode);
  amvpCand0 = &(m_tplAmvpInfo[cu.imv][REF_PIC_LIST_0][pu.refIdx[REF_PIC_LIST_0]]);
  amvpCand1 = &(m_tplAmvpInfo[cu.imv][REF_PIC_LIST_1][pu.refIdx[REF_PIC_LIST_1]]);
  amvpCand0Lic = &(m_tplAmvpInfoLIC[cu.imv][REF_PIC_LIST_0][pu.refIdx[REF_PIC_LIST_0]]);
  amvpCand1Lic = &(m_tplAmvpInfoLIC[cu.imv][REF_PIC_LIST_1][pu.refIdx[REF_PIC_LIST_1]]);
  mvpIdx0 = pu.mvpIdx[REF_PIC_LIST_0];
  mvpIdx1 = pu.mvpIdx[REF_PIC_LIST_1];

  if (!cu.affine)
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if ((pu.interDir & (1 << refList)) == 0)
      {
        continue;
      }
      RefPicList eRefList = (RefPicList)refList;

      AMVPInfo* curAmvpLicCand = ((eRefList == REF_PIC_LIST_0) ? amvpCand0Lic : amvpCand1Lic);
      if (curAmvpLicCand->numCand == 0)
      {
        PU::fillMvpCand(pu, eRefList, pu.refIdx[refList], *curAmvpLicCand, this);
      }

      AMVPInfo* curAmvpCand = (eRefList == REF_PIC_LIST_0) ? amvpCand0 : amvpCand1;
      if (curAmvpCand->numCand == 0)
      {
        pu.cu->licFlag = false;
        PU::fillMvpCand(pu, eRefList, pu.refIdx[refList], *curAmvpCand, this);
        pu.cu->licFlag = true;
      }
    }
    if (cs.picHeader->getMvdL1ZeroFlag() && pu.interDir == 3)
    {
      if (amvpCand1Lic->mvCand[mvpIdx1] != amvpCand1->mvCand[mvpIdx1])
      {
        updateL1ZeroFlagMvNeeded = true;
      }
    }
    if (pu.cu->smvdMode)
    {
      Mv mvd0 = pu.mv[REF_PIC_LIST_0] - amvpCand0->mvCand[mvpIdx0];
      Mv mvd1 = amvpCand1->mvCand[mvpIdx1] - pu.mv[REF_PIC_LIST_1];
      if (mvd0 != mvd1)
      {
        updateSMVDMvNeeded = true;
      }
    }
  }
  updateMvNeeded = updateL1ZeroFlagMvNeeded || updateSMVDMvNeeded;
#if JVET_AC0158_PIXEL_AFFINE_MC && !JVET_AD0213_LIC_IMP
  m_storeBeforeLIC = !updateMvNeeded && isBDOFNotNeeded && !pu.cu->affine;
#else
  m_storeBeforeLIC = !updateMvNeeded && isBDOFNotNeeded;
#endif
}

void InterSearch::setEncCtrlParaLicOn(CodingUnit& cu)
{
  PredictionUnit& pu = *cu.firstPU;

  amvpCand0 = &(m_tplAmvpInfo[cu.imv][REF_PIC_LIST_0][pu.refIdx[REF_PIC_LIST_0]]);
  amvpCand1 = &(m_tplAmvpInfo[cu.imv][REF_PIC_LIST_1][pu.refIdx[REF_PIC_LIST_1]]);
  amvpCand0Lic = &(m_tplAmvpInfoLIC[cu.imv][REF_PIC_LIST_0][pu.refIdx[REF_PIC_LIST_0]]);
  amvpCand1Lic = &(m_tplAmvpInfoLIC[cu.imv][REF_PIC_LIST_1][pu.refIdx[REF_PIC_LIST_1]]);
  mvpIdx0 = pu.mvpIdx[REF_PIC_LIST_0];
  mvpIdx1 = pu.mvpIdx[REF_PIC_LIST_1];

  if (!cu.affine)
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if ((pu.interDir & (1 << refList)) == 0)
      {
        continue;
      }
      RefPicList eRefList = (RefPicList)refList;

      AMVPInfo* curAmvpCand = (eRefList == REF_PIC_LIST_0) ? amvpCand0 : amvpCand1;
      if (curAmvpCand->numCand == 0)
      {
        PU::fillMvpCand(pu, eRefList, pu.refIdx[refList], *curAmvpCand, this);
      }

      AMVPInfo* curAmvpLicCand = ((eRefList == REF_PIC_LIST_0) ? amvpCand0Lic : amvpCand1Lic);
      if (curAmvpLicCand->numCand == 0)
      {
        pu.cu->licFlag = true;
        PU::fillMvpCand(pu, eRefList, pu.refIdx[refList], *curAmvpLicCand, this);
        pu.cu->licFlag = false;
      }
    }
  }
  m_predictionBeforeLIC = m_tmpStorageLCU.getBuf(UnitAreaRelative(*pu.cu, pu));
}

void InterSearch::checkEncLicOff(CodingUnit& cu, MergeCtx& mergeCtx)
{
  CodingStructure& cs = *cu.cs;
  PredictionUnit&  pu = *cu.firstPU;

  PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
  DistParam distParam;
  m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), predBuf.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
  Distortion distLicOn = distParam.distFunc(distParam);

  const double  sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
  uint64_t licOnFracBits = xCalcExpPuBits(pu);
  double   licOnCost = (double)distLicOn + (double)licOnFracBits * sqrtLambdaForFirstPassIntra;
  pu.cu->licFlag = false;
#if JVET_AC0158_PIXEL_AFFINE_MC
  if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
  {
    pu.cu->obmcFlag = false;
  }
  else
  {
    pu.cu->obmcFlag = true;
  }
#endif

  Mv orgMv1 = pu.mv[REF_PIC_LIST_1];
  Mv orgMvd0 = pu.mvd[REF_PIC_LIST_0];
  Mv orgMvd1 = pu.mvd[REF_PIC_LIST_1];

  if (!pu.cu->affine) // because affine has the same AMVPs when LIC is on and off, it is not needed to update mv and mvd
  {
    if (cs.picHeader->getMvdL1ZeroFlag() && pu.interDir == 3)
    {
      CHECK(pu.mvd[REF_PIC_LIST_1] != Mv(), "L1 mvd should be zero MV for MvdL1ZeroFlag");
      pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0->mvCand[mvpIdx0];
      pu.mv[REF_PIC_LIST_1] = amvpCand1->mvCand[mvpIdx1];
    }
    else if (pu.cu->smvdMode)
    {
      Mv mvd = pu.mv[REF_PIC_LIST_0] - amvpCand0->mvCand[mvpIdx0];
      pu.mv[REF_PIC_LIST_1] = amvpCand1->mvCand[mvpIdx1] - mvd;
      pu.mvd[REF_PIC_LIST_0] = mvd;
      pu.mvd[REF_PIC_LIST_1] = Mv() - mvd;
    }
    else
    {
      if (pu.interDir == 1)
      {
        pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0->mvCand[mvpIdx0];
      }
      else if (pu.interDir == 2)
      {
        pu.mvd[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1] - amvpCand1->mvCand[mvpIdx1];
      }
      else if (pu.interDir == 3)
      {
        pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0->mvCand[mvpIdx0];
        pu.mvd[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1] - amvpCand1->mvCand[mvpIdx1];
      }
    }
  }

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  if (PU::checkDoAffineBdofRefine(pu, this))
  {
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
    setDoAffineSubPuBdof(false);
  }
#if JVET_AC0158_PIXEL_AFFINE_MC && !JVET_AD0213_LIC_IMP
  if (updateMvNeeded || !isBDOFNotNeeded || pu.cu->affine)
#else
  if (updateMvNeeded || !isBDOFNotNeeded || pu.availableBdofRefinedMv == AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV)
#endif
  {
    motionCompensation(pu, m_predictionBeforeLIC, REF_PIC_LIST_X);
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
  }
#else
#if JVET_AC0158_PIXEL_AFFINE_MC && !JVET_AD0213_LIC_IMP
  if (updateMvNeeded || !isBDOFNotNeeded || pu.cu->affine)
#else
  if (updateMvNeeded || !isBDOFNotNeeded)
#endif
  {
    motionCompensation(pu, m_predictionBeforeLIC, REF_PIC_LIST_X);
  }
#endif
  m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), m_predictionBeforeLIC.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
  Distortion distLicOff = distParam.distFunc(distParam);
  m_CABACEstimator->getCtx() = ctxStart;
  uint64_t licOffFracBits = xCalcExpPuBits(pu);
  double licOffCost = (double)distLicOff + (double)licOffFracBits * sqrtLambdaForFirstPassIntra;
  if (licOnCost >= licOffCost)
  {
    pu.cu->licFlag = false;
#if JVET_AC0158_PIXEL_AFFINE_MC
    if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
    {
      pu.cu->obmcFlag = false;
    }
    else
    {
      pu.cu->obmcFlag = true;
    }
#endif
#if JVET_AD0213_LIC_IMP
    for (int list = 0; list < 2; list++)
    {
      for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
      {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        pu.cu->licScale[list][comp] = 32;
        pu.cu->licOffset[list][comp] = 0;
#else
        pu.cu->licScale[list][comp] = MAX_INT;
        pu.cu->licOffset[list][comp] = MAX_INT;
#endif
      }
    }
#endif
    PU::spanMotionInfo(pu, mergeCtx);
    predBuf.copyFrom(m_predictionBeforeLIC);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (getDoAffineSubPuBdof() == true)
    {
      PU::setAffineBdofRefinedMotion(pu, getBdofSubPuMvBuf());
      setDoAffineSubPuBdof(false);
    }
#endif
  }
  else
  {
    pu.cu->licFlag = true;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
    if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
    {
      pu.cu->obmcFlag = false;
    }
    else
    {
      pu.cu->obmcFlag = true;
    }
#else
    pu.cu->obmcFlag = false;
#endif
#endif
    pu.mv[REF_PIC_LIST_1] = orgMv1;
    pu.mvd[REF_PIC_LIST_0] = orgMvd0;
    pu.mvd[REF_PIC_LIST_1] = orgMvd1;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    setDoAffineSubPuBdof(false);
#endif
  }
  m_CABACEstimator->getCtx() = ctxStart;
}

void InterSearch::checkEncLicOn(CodingUnit& cu, MergeCtx& mergeCtx)
{
  CodingStructure& cs = *cu.cs;
  PredictionUnit&  pu = *cu.firstPU;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  const bool doAffineSubPuBdof = getDoAffineSubPuBdof();
  setDoAffineSubPuBdof(false);
#endif

  PelUnitBuf predBuf = pu.cs->getPredBuf(pu);
  DistParam distParam;
  m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), predBuf.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
  Distortion distLicOff = distParam.distFunc(distParam);

  const double  sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
  uint64_t licOffFracBits = xCalcExpPuBits(pu);
  double   licOffCost = (double)distLicOff + (double)licOffFracBits * sqrtLambdaForFirstPassIntra;
  pu.cu->licFlag = true;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
  if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
  {
    pu.cu->obmcFlag = false;
  }
  else
  {
    pu.cu->obmcFlag = true;
  }
#else
  pu.cu->obmcFlag = false;
#endif
#endif

  Mv orgMv1 = pu.mv[REF_PIC_LIST_1];
  Mv orgMvd0 = pu.mvd[REF_PIC_LIST_0];
  Mv orgMvd1 = pu.mvd[REF_PIC_LIST_1];

  if (!pu.cu->affine) // because affine has the same AMVPs when LIC is on and off, it is not needed to update mv and mvd
  {
    if (cs.picHeader->getMvdL1ZeroFlag() && pu.interDir == 3)
    {
      CHECK(pu.mvd[REF_PIC_LIST_1] != Mv(), "L1 mvd should be zero MV for MvdL1ZeroFlag");
      pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0Lic->mvCand[mvpIdx0];
      pu.mv[REF_PIC_LIST_1] = amvpCand1Lic->mvCand[mvpIdx1];
    }
    else if (pu.cu->smvdMode)
    {
      Mv mvd = pu.mv[REF_PIC_LIST_0] - amvpCand0Lic->mvCand[mvpIdx0];
      pu.mv[REF_PIC_LIST_1] = amvpCand1Lic->mvCand[mvpIdx1] - mvd;
      pu.mvd[REF_PIC_LIST_0] = mvd;
      pu.mvd[REF_PIC_LIST_1] = Mv() - mvd;
    }
    else
    {
      if (pu.interDir == 1)
      {
        pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0Lic->mvCand[mvpIdx0];
      }
      else if (pu.interDir == 2)
      {
        pu.mvd[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1] - amvpCand1Lic->mvCand[mvpIdx1];
      }
      else if (pu.interDir == 3)
      {
        pu.mvd[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0] - amvpCand0Lic->mvCand[mvpIdx0];
        pu.mvd[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1] - amvpCand1Lic->mvCand[mvpIdx1];
      }
    }
  }
  motionCompensation(pu, m_predictionBeforeLIC, REF_PIC_LIST_X);
#if JVET_AD0213_LIC_IMP
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    if (!pu.cu->licInheritPara)
#endif
  for (int list = 0; list < 2; list++)
  {
    if (pu.refIdx[list] >= 0)
    {
      for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
      {
        setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
      }
    }
  }
#endif
  m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), m_predictionBeforeLIC.Y(), cs.sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
  Distortion distLicOn = distParam.distFunc(distParam);
  m_CABACEstimator->getCtx() = ctxStart;
  uint64_t licOnFracBits = xCalcExpPuBits(pu);
  double licOnCost = (double)distLicOn + (double)licOnFracBits * sqrtLambdaForFirstPassIntra;

  if (licOnCost < licOffCost)
  {
    pu.cu->licFlag = true;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
    if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
    {
      pu.cu->obmcFlag = false;
    }
    else
    {
      pu.cu->obmcFlag = true;
    }
#else
    pu.cu->obmcFlag = false;
#endif
#endif
    PU::spanMotionInfo(pu, mergeCtx);
    predBuf.copyFrom(m_predictionBeforeLIC);
  }
  else
  {
    pu.cu->licFlag = false;
#if JVET_AC0158_PIXEL_AFFINE_MC
    if ((cu.lwidth() * cu.lheight() < 32) || cu.slice->getSPS()->getUseOBMC() == false)
    {
      pu.cu->obmcFlag = false;
    }
    else
    {
      pu.cu->obmcFlag = true;
    }
#endif
#if JVET_AD0213_LIC_IMP
    for (int list = 0; list < 2; list++)
    {
      for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
      {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
        pu.cu->licScale[list][comp] = 32;
        pu.cu->licOffset[list][comp] = 0;
#else
        pu.cu->licScale[list][comp] = MAX_INT;
        pu.cu->licOffset[list][comp] = MAX_INT;
#endif
      }
    }
#endif
    pu.mv[REF_PIC_LIST_1] = orgMv1;
    pu.mvd[REF_PIC_LIST_0] = orgMvd0;
    pu.mvd[REF_PIC_LIST_1] = orgMvd1;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (doAffineSubPuBdof == true)
    {
      PU::setAffineBdofRefinedMotion(pu, getBdofSubPuMvBuf());
    }
#endif
  }
  m_CABACEstimator->getCtx() = ctxStart;
}

uint64_t InterSearch::xCalcExpPuBits(PredictionUnit& pu)
{
  CHECKD(pu.mergeFlag, "");
  CHECKD(CU::isIBC(*pu.cu), "");

  m_CABACEstimator->resetBits();

#if JVET_X0083_BM_AMVP_MERGE_MODE
  m_CABACEstimator->amvpMerge_mode(pu);
  if (!(pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1]))
  {
#endif
    m_CABACEstimator->inter_pred_idc(pu);
    m_CABACEstimator->affine_flag(*pu.cu);
#if JVET_AG0098_AMVP_WITH_SBTMVP
    m_CABACEstimator->amvpSbTmvpFlag(pu);
    if (!pu.amvpSbTmvpFlag)
    {
#endif
    m_CABACEstimator->smvd_mode(pu);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if (pu.cs->sps->getUseMvdPred())
    {
#endif
      m_CABACEstimator->cu_bcw_flag(*pu.cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
    }
#endif
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  }
#endif
  if (pu.interDir != 2 /* PRED_L1 */)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    if (!pu.amvpMergeModeFlag[REF_PIC_LIST_0])
    {
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
      m_CABACEstimator->ref_idx(pu, REF_PIC_LIST_0, true);
#else
      m_CABACEstimator->ref_idx(pu, REF_PIC_LIST_0);
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      m_CABACEstimator->mvp_flag(pu, REF_PIC_LIST_0);
#endif
      if (pu.cu->affine)
      {
        Mv mvd = pu.mvdAffi[REF_PIC_LIST_0][0];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
        mvd = pu.mvdAffi[REF_PIC_LIST_0][1];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
        m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision

        if (pu.cu->affineType == AFFINEMODEL_6PARAM)
        {
          mvd = pu.mvdAffi[REF_PIC_LIST_0][2];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
          m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      else
      {
        Mv mvd = pu.mvd[REF_PIC_LIST_0];
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (pu.amvpMergeModeFlag[REF_PIC_LIST_1] == true && pu.mvpIdx[REF_PIC_LIST_0] < 2)
        {
          CHECK(mvd.hor != 0, "this is not possible");
          CHECK(mvd.ver != 0, "this is not possible");
        }
        else
        {
#endif
          mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
          m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
    }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    m_CABACEstimator->mvp_flag(pu, REF_PIC_LIST_0);
#endif
  }
  if (pu.interDir != 1 /* PRED_L0 */)
  {
    if (pu.cu->smvdMode != 1)
    {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!pu.amvpMergeModeFlag[REF_PIC_LIST_1])
      {
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
        m_CABACEstimator->ref_idx(pu, REF_PIC_LIST_1, true);
#else
        m_CABACEstimator->ref_idx(pu, REF_PIC_LIST_1);
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        m_CABACEstimator->mvp_flag(pu, REF_PIC_LIST_1);
#endif
        if (!pu.cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */)
        {
          if (pu.cu->affine)
          {
            Mv mvd = pu.mvdAffi[REF_PIC_LIST_1][0];
            mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
            m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
            mvd = pu.mvdAffi[REF_PIC_LIST_1][1];
            mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
            m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
            if (pu.cu->affineType == AFFINEMODEL_6PARAM)
            {
              mvd = pu.mvdAffi[REF_PIC_LIST_1][2];
              mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
              m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
            }
          }
          else
          {
            Mv mvd = pu.mvd[REF_PIC_LIST_1];
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
            if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] == true && pu.mvpIdx[REF_PIC_LIST_1] < 2)
            {
              CHECK(mvd.hor != 0, "this is not possible");
              CHECK(mvd.ver != 0, "this is not possible");
            }
            else
            {
#endif
              mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
              m_CABACEstimator->mvd_coding(mvd, 0); // already changed to signaling precision
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
            }
#endif
          }
        }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
    }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    else
    {
      CHECK(pu.refIdx[REF_PIC_LIST_1] != pu.cs->slice->getSymRefIdx(REF_PIC_LIST_1), "Wrong L1 reference index");
      m_CABACEstimator->mvp_flag(pu, REF_PIC_LIST_1);
    }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    m_CABACEstimator->mvp_flag(pu, REF_PIC_LIST_1);
#endif
  }

  m_CABACEstimator->imv_mode(*pu.cu);
  m_CABACEstimator->affine_amvr_mode(*pu.cu);
#if INTER_LIC
  m_CABACEstimator->cu_lic_flag(*pu.cu);
#endif
#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (!pu.cu->cs->sps->getUseMvdPred())
  {
#endif
    m_CABACEstimator->cu_bcw_flag(*pu.cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  }
#endif
#endif
#if ENABLE_OBMC 
  m_CABACEstimator->obmc_flag(*pu.cu);
#endif

  return m_CABACEstimator->getEstFracBits();
}
#endif

#if !JVET_Z0084_IBC_TM
bool InterSearch::searchBv(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize)
{
  const int ctuSizeLog2 = floorLog2(ctuSize);

  int refRightX = xPos + xBv + width - 1;
  int refBottomY = yPos + yBv + height - 1;

  int refLeftX = xPos + xBv;
  int refTopY = yPos + yBv;

  if ((xPos + xBv) < 0)
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

  if ((yPos + yBv) < 0)
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
  if ((xBv + width) > 0 && (yBv + height) > 0)
  {
    return false;
  }

#if !JVET_Z0153_IBC_EXT_REF
  // Don't search the above CTU row
  if (refTopY >> ctuSizeLog2 < yPos >> ctuSizeLog2)
    return false;
#endif

  // Don't search the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

#if JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
  {
    return false;
  }
#else
  unsigned curTileIdx = pu.cs->pps->getTileIdx(pu.lumaPos());
  unsigned refTileIdx = pu.cs->pps->getTileIdx(Position(refLeftX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refLeftX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refRightX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refRightX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }

#if JVET_Z0153_IBC_EXT_REF
#if JVET_AA0106_IBCBUF_CTU256
  if (256 == ctuSize)
  {
    if ((refTopY >> ctuSizeLog2) + 1 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 1 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 1 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }
  else
  {
    if ((refTopY >> ctuSizeLog2) + 2 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 2 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 2 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }
#else
  if ((refTopY >> ctuSizeLog2) + 2 < (yPos >> ctuSizeLog2))
  {
    return false;
  }
  if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
  {
    return false;
  }
  if (((refTopY >> ctuSizeLog2) + 2 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 2 < (xPos >> ctuSizeLog2)))
  {
    return false;
  }
#endif
#else
  // in the same CTU line
#if CTU_256
  int numLeftCTUs = ( 1 << ( ( MAX_CU_DEPTH - ctuSizeLog2 ) << 1 ) ) - ( ( ctuSizeLog2 < MAX_CU_DEPTH ) ? 1 : 0 );
#else
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);
#endif
  if ((refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2) && (refLeftX >> ctuSizeLog2 >= (xPos >> ctuSizeLog2) - numLeftCTUs))
  {

    // in the same CTU, or left CTU
    // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
#if CTU_256
    if( ( ( refLeftX >> ctuSizeLog2 ) == ( ( xPos >> ctuSizeLog2 ) - 1 ) ) && ( ctuSizeLog2 == MAX_CU_DEPTH ) )
#else
    if (((refLeftX >> ctuSizeLog2) == ((xPos >> ctuSizeLog2) - 1)) && (ctuSizeLog2 == 7))
#endif
    {
      // ref block's collocated block in current CTU
      const Position refPosCol = pu.Y().topLeft().offset(xBv + ctuSize, yBv);
      int offset64x = (refPosCol.x >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      int offset64y = (refPosCol.y >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      const Position refPosCol64x64 = {offset64x, offset64y};
      if (pu.cs->isDecomp(refPosCol64x64, toChannelType(COMPONENT_Y)))
        return false;
      if (refPosCol64x64 == pu.Y().topLeft())
        return false;
    }
  }
  else
    return false;
#endif
#endif

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
  const Position refPosLT = pu.Y().topLeft().offset(xBv, yBv);
  const Position refPosBR = pu.Y().bottomRight().offset(xBv, yBv);
  const ChannelType      chType = toChannelType(COMPONENT_Y);
  if (!pu.cs->isDecomp(refPosBR, chType))
    return false;
  if (!pu.cs->isDecomp(refPosLT, chType))
    return false;
  return true;
}
#endif

//! \}
