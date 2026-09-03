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

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_buffer.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#if K0149_BLOCK_STATISTICS
#include "CommonLib/ChromaFormat.h"
#include "CommonLib/dtrace_blockstatistics.h"
#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DecCu::DecCu()
{
  m_tmpStorageLCU = NULL;
}

DecCu::~DecCu()
{
  m_ciipBuffer.destroy();
}

void DecCu::init( TrQuant* pcTrQuant, IntraPrediction* pcIntra, InterPrediction* pcInter)
{
  m_pcTrQuant       = pcTrQuant;
  m_pcIntraPred     = pcIntra;
  m_pcInterPred     = pcInter;
  m_ciipBuffer.destroy();
  m_ciipBuffer.create(CHROMA_420, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)); // TODO: support other color format
}
void DecCu::initDecCuReshaper  (Reshape* pcReshape, ChromaFormat chromaFormatIDC)
{
  m_pcReshape = pcReshape;
  if (m_tmpStorageLCU == NULL)
  {
    m_tmpStorageLCU = new PelStorage;
    m_tmpStorageLCU->create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }

}
void DecCu::destoryDecCuReshaprBuf()
{
  if (m_tmpStorageLCU)
  {
    m_tmpStorageLCU->destroy();
    delete m_tmpStorageLCU;
    m_tmpStorageLCU = NULL;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void DecCu::decompressCtu( CodingStructure& cs, const UnitArea& ctuArea )
{

  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;

  if (cs.resetIBCBuffer)
  {
    m_pcInterPred->resetIBCBuffer(cs.pcv->chrFormat, cs.slice->getSPS()->getMaxCUHeight());
    cs.resetIBCBuffer = false;
  }
#if JVET_Z0153_IBC_EXT_REF && !JVET_AJ0172_IBC_ITMP_ALIGN_REF_AREA
  else
  {
    const int ctuSize = cs.slice->getSPS()->getMaxCUHeight();
    m_pcInterPred->resetVPDUforIBC(cs.pcv->chrFormat, ctuSize, ctuSize, ctuArea.Y().x, ctuArea.Y().y);
  }
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  m_pcIntraPred->resetCuData();
#endif

#if JVET_AE0159_FIBC || JVET_AE0059_INTER_CCCM || JVET_AE0078_IBC_LIC_EXTENSION || JVET_AF0073_INTER_CCP_MERGE
  m_pcInterPred->setIntraPrediction( m_pcIntraPred );
#endif
#if JVET_AH0200_INTRA_TMP_BV_REORDER
  m_pcIntraPred->setInterPrediction(m_pcInterPred);
#endif
#if JVET_Z0118_GDR
  // reset current IBC Buffer only when VB pass through
  if (cs.isGdrEnabled() && cs.isInGdrIntervalOrRecoveryPoc())
  {
#if JVET_Z0153_IBC_EXT_REF    
    m_pcInterPred->resetCurIBCBuffer(
      cs.pcv->chrFormat,
      ctuArea.Y(),
      cs.slice->getSPS()->getMaxCUHeight(),        
      1 << (cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1)
    );
#else
    int gdrEndX = cs.picHeader->getGdrEndX();

    if (ctuArea.lx() <= gdrEndX && gdrEndX < ctuArea.lx() + ctuArea.lwidth())
    {
      m_pcInterPred->resetCurIBCBuffer(
        cs.pcv->chrFormat,
        ctuArea.Y(),
        cs.slice->getSPS()->getMaxCUHeight(),
        1 << (cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1)
      );
    }
#endif
  }
#endif

  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );
    Position prevTmpPos;
    prevTmpPos.x = -1; prevTmpPos.y = -1;

    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
      m_pcInterPred->setDIMDForOBMC(false);
#if JVET_AK0076_EXTENDED_OBMC_IBC && !JVET_AK0212_GPM_OBMC_MODIFICATION
      m_pcInterPred->setIntraObmcPred(false);
#endif
      m_pcInterPred->setModeGetCheck(0, false);
      m_pcInterPred->setModeGetCheck(1, false);
      m_pcInterPred->setClearModeBuf(0);
      m_pcInterPred->setClearModeBuf(1);
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
      m_pcInterPred->setFillCurTplAboveARMC(false);
      m_pcInterPred->setFillCurTplLeftARMC(false);
#endif
#if JVET_AD0213_LIC_IMP
      m_pcInterPred->resetFillLicTpl();
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
      m_pcInterPred->m_neighbSccChecked = false;
      m_pcInterPred->m_intraObmcReload = false;
#endif

#if JVET_Z0118_GDR
      if (cs.isGdrEnabled())
      {
        Slice   *slice = currCU.slice;
        Picture *refPic;

        bool isInGdrInterval = slice->getPicHeader()->getInGdrInterval();
        bool isRecoveryPocPic = slice->getPicHeader()->getIsGdrRecoveryPocPic();
        bool isNorPicture = !(isInGdrInterval || isRecoveryPocPic) && slice->isInterB();

        if (isNorPicture)
        {
          currCU.cs->setReconBuf(PIC_RECONSTRUCTION_0);
          currCU.cs->picture->setCleanDirty(false);

          // 1.01 use only dirty reference picture
          for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
          {
            int n = slice->getNumRefIdx((RefPicList)rlist);
            for (int idx = 0; idx < n; idx++)
            {
              Picture *refPic = slice->getReferencePicture((RefPicList)rlist, idx);
              if (refPic)
              {
                // when cur picture is normal picture and ref picture is gdr/recovery picture
                // note: pic.slice.picHeader and pic.cs.picHeader could be different    
                // bool isGdrPic = refPic->cs->picHeader->getInGdrPeriod();
                bool isRefInGdrInterval = refPic->cs->picHeader->getInGdrInterval();
                bool isRefRecoveryPocPic = refPic->cs->picHeader->getIsGdrRecoveryPocPic();

                if (isRefInGdrInterval || isRefRecoveryPocPic)
                {
                  refPic->setCleanDirty(true);
                }
                else
                {
                  refPic->setCleanDirty(false);
                }
              }
            }
          }
        }

        // 1.02 use clean reference pictures for some CU when gdr starts
        if (isInGdrInterval || isRecoveryPocPic)
        {
          // 1.01 switch recon based on clean/dirty current area
          bool cleanDirtyFlag;

          bool isCuClean = currCU.Y().valid() ? cs.isClean(currCU.Y().topLeft(), CHANNEL_TYPE_LUMA) : cs.isClean(currCU.Cb().topLeft(), CHANNEL_TYPE_CHROMA);

          if (isCuClean)
          {
            cleanDirtyFlag = true;
          }
          else
          {
            cleanDirtyFlag = false;
          }

          currCU.cs->setReconBuf((cleanDirtyFlag) ? PIC_RECONSTRUCTION_1 : PIC_RECONSTRUCTION_0);
          currCU.cs->picture->setCleanDirty(cleanDirtyFlag);

          for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
          {
            int n = slice->getNumRefIdx((RefPicList)rlist);
            for (int idx = 0; idx < n; idx++)
            {
              refPic = slice->getReferencePicture((RefPicList)rlist, idx);
              if (refPic)
              {
                refPic->setCleanDirty(cleanDirtyFlag);
              }
            }
          }
        }
      }
#endif

#if !REMOVE_VPDU
      if(currCU.Y().valid())
      {
        const int vSize = cs.slice->getSPS()->getMaxCUHeight() > 64 ? 64 : cs.slice->getSPS()->getMaxCUHeight();
        if((currCU.Y().x % vSize) == 0 && (currCU.Y().y % vSize) == 0)
        {
          for(int x = currCU.Y().x; x < currCU.Y().x + currCU.Y().width; x += vSize)
          {
            for(int y = currCU.Y().y; y < currCU.Y().y + currCU.Y().height; y += vSize)
            {
              m_pcInterPred->resetVPDUforIBC(cs.pcv->chrFormat, cs.slice->getSPS()->getMaxCUHeight(), vSize, x + g_IBCBufferSize / cs.slice->getSPS()->getMaxCUHeight() / 2, y);
            }
          }
        }
      }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      bool changeProcIntraReg = false;
      if (currCU.isSST && currCU.separateTree && currCU.predMode == MODE_INTRA && currCU.slice->getProcessingSeparateTrees() && !currCU.slice->getProcessingIntraRegion())
      {
        currCU.slice->setProcessingIntraRegion(true);
        changeProcIntraReg = true;
      }
#endif

      if (currCU.predMode != MODE_INTRA && currCU.predMode != MODE_PLT && currCU.Y().valid())
      {
        xDeriveCUMV(currCU);
#if K0149_BLOCK_STATISTICS
        if(currCU.geoFlag)
        {
          storeGeoMergeCtx(m_geoMrgCtx);
        }
#endif
      }
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
      if (currCU.predMode == MODE_INTRA)
      {
        if ((chType == CHANNEL_TYPE_LUMA) && (currCU.dimd || currCU.sgpm || currCU.tmpFlag || currCU.timd || currCU.eipFlag))
        {
          if (currCU.sgpm)
          {
            m_pcIntraPred->m_itmpBvBasedMergeCandidates.clear();
            m_pcIntraPred->m_sgpmMvBasedMergeCandidates.clear();
            PU::getItmpMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_itmpBvBasedMergeCandidates, m_pcIntraPred->m_sgpmMvBasedMergeCandidates);
            const int maxToKeep = currCU.cs->sps->getItmpLicMode() ? MAX_TO_KEEP : MAX_TO_KEEP >> 1;
            if (m_pcIntraPred->m_sgpmMvBasedMergeCandidates.size() > maxToKeep)
            {
              m_pcIntraPred->m_sgpmMvBasedMergeCandidates.resize(maxToKeep);
            }
          }
#if JVET_AL0108_BVG_DIMD
#if JVET_AL0106_BV_EIP
          else if (currCU.tmpFlag || currCU.bvgDimdFlag || currCU.eipFlag)
#else
          else if (currCU.tmpFlag || currCU.bvgDimdFlag)
#endif

#else
          else if (currCU.tmpFlag)
#endif          
          {
            m_pcIntraPred->m_itmpBvBasedMergeCandidates.clear();
            m_pcIntraPred->itmpSize = PU::getItmpMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_itmpBvBasedMergeCandidates);
            
            if ((currCU.lwidth() <= cs.slice->getSPS()->getIntraTMPMaxSize() && currCU.lheight() <= cs.slice->getSPS()->getIntraTMPMaxSize()))
            {
              m_pcIntraPred->itmpSize = PU::sortInOut(&currCU, currCU.lwidth(), currCU.lheight(),
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                                         currCU.tempType,
#else
                                                         m_pcIntraPred->getRefTemplateType(currCU, currCU.blocks[COMPONENT_Y]),
#endif
                                                         m_pcIntraPred->m_itmpBvBasedMergeCandidates);
            }

            m_pcIntraPred->itmpSize1 = PU::getArBvMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_itmpBvBasedMergeCandidates, std::max(NUM_BV, static_cast<int>(m_pcIntraPred->m_itmpBvBasedMergeCandidates.size()) + NUM_TMP_ARBVP));
          }
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
          else if (currCU.timd)
          {
            m_pcIntraPred->m_timdBvBasedMergeCandidates.clear();
            PU::getItmpMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_timdBvBasedMergeCandidates);
            PU::getArBvMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_timdBvBasedMergeCandidates, std::max(NUM_BV, static_cast<int>(m_pcIntraPred->m_timdBvBasedMergeCandidates.size()) + NUM_TMP_ARBVP));
          }
#endif
          else if (currCU.dimd)
          {
            m_pcIntraPred->m_dimdBvBasedMergeCandidates.clear();
            PU::getItmpMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_dimdBvBasedMergeCandidates);
            const int maxToKeep = currCU.cs->sps->getItmpLicMode() ? MAX_TO_KEEP : MAX_TO_KEEP >> 1;
            if (m_pcIntraPred->m_dimdBvBasedMergeCandidates.size() > maxToKeep)
            {
              m_pcIntraPred->m_dimdBvBasedMergeCandidates.resize(maxToKeep);
            }
          }
        }
      }
#endif
      switch( currCU.predMode )
      {
      case MODE_INTER:
      case MODE_IBC:
#if JVET_Y0065_GPM_INTRA
#if ENABLE_DIMD && JVET_W0123_TIMD_FUSION
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if ((chType == CHANNEL_TYPE_LUMA) && (cs.slice->getSPS()->getUseDimd() || cs.slice->getSPS()->getUseTimd()) && currCU.geoFlag && currCU.firstPU->gpmIntraFlag)
#else
        if ((cs.slice->getSPS()->getUseDimd() || cs.slice->getSPS()->getUseTimd()) && currCU.geoFlag && currCU.firstPU->gpmIntraFlag)
#endif
#elif ENABLE_DIMD
        if (cs.slice->getSPS()->getUseDimd() && currCU.geoFlag && currCU.firstPU->gpmIntraFlag)
#elif JVET_W0123_TIMD_FUSION
        if (cs.slice->getSPS()->getUseTimd() && currCU.geoFlag && currCU.firstPU->gpmIntraFlag)
#endif
        {
#if JVET_AG0164_AFFINE_GPM
          if ((int)(currCU.firstPU->geoMergeIdx0)- GEO_MAX_ALL_INTER_UNI_CANDS > 0 || (int)(currCU.firstPU->geoMergeIdx1)- GEO_MAX_ALL_INTER_UNI_CANDS > 0) // dimd/timd
#else
          if ((int)(currCU.firstPU->geoMergeIdx0)-GEO_MAX_NUM_UNI_CANDS > 0 || (int)(currCU.firstPU->geoMergeIdx1)-GEO_MAX_NUM_UNI_CANDS > 0) // dimd/timd
#endif
          {
            const CompArea &area = currCU.Y();
#if ENABLE_DIMD
#if JVET_W0123_TIMD_FUSION
            if (cs.slice->getSPS()->getUseDimd())
#endif
            {
              IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
            }
#endif
#if JVET_W0123_TIMD_FUSION
#if ENABLE_DIMD
            if (cs.slice->getSPS()->getUseTimd())
#endif
            {
              currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
            }
#endif
          }
        }
#endif
#if JVET_AL0134_SGPM_INTER
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
        if (currCU.firstPU->sgpmInter && !currCU.firstPU->sgpmAffine)
#else
        if (currCU.firstPU->sgpmInter)
#endif
        {
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
          int mpmNum = 0;
          if (currCU.firstPU->sgpmIntra)
          {
            if (cs.slice->getSPS()->getUseDimd())
            {
              const CompArea& area = currCU.Y();
              IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
            }
            uint8_t intraNonMPM[NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES];
#if JVET_AK0059_MDIP
            currCU.isModeExcluded = false;
#endif
            mpmNum = PU::getIntraMPMs(*currCU.firstPU, m_pcIntraPred->m_intraMPM, intraNonMPM
#if JVET_AC0094_REF_SAMPLES_OPT
              , true
#endif
#if JVET_AK0061_PDP_MPM
              , true, true
#endif
              , m_pcIntraPred
            );
#if JVET_AK0059_MDIP
            currCU.isModeExcluded = true;
#endif
            mpmNum = std::min(mpmNum, SGPM_INTRA_NUM);
          }
#endif
          const CompArea                              &area = currCU.Y();
          static_vector<SgpmInterInfo, SGPM_INTER_NUM> sgpmInterInfoList;
          static_vector<double, SGPM_INTER_NUM>        sgpmInterCostList;
          int                                          sgpmInterIdx = currCU.firstPU->sgpmInterIdx;
          if (currCU.firstPU->sgpmInterTm)
          {
            currCU.firstPU->geoTmFlag0   = true;
            currCU.firstPU->geoTmFlag1   = true;
            m_pcIntraPred->deriveSgpmTmInterModeOrdered(m_geoMrgCtx, m_geoTmMrgCtx, currCU.cs->picture->getRecoBuf(area), area, currCU, sgpmInterInfoList, sgpmInterCostList, m_pcInterPred
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
              , sgpmInterIdx + 1
#endif
            );
          }
          else
          {
            currCU.firstPU->geoTmFlag0   = false;
            currCU.firstPU->geoTmFlag1   = false;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
            if (currCU.firstPU->sgpmIntra)
            {
              m_pcIntraPred->deriveSgpmInterIntraModeOrdered(m_geoMrgCtx, 
                currCU.cs->picture->getRecoBuf(area), area, currCU,
                sgpmInterInfoList, sgpmInterCostList, m_pcInterPred
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
                , sgpmInterIdx + 1
#endif 
                , m_pcIntraPred, m_pcIntraPred->m_intraMPM, mpmNum

              );
            }
            else
            {
#endif
            m_pcIntraPred->deriveSgpmInterModeOrdered(m_geoMrgCtx, currCU.cs->picture->getRecoBuf(area), area, currCU,
                                                      sgpmInterInfoList, sgpmInterCostList, m_pcInterPred
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
              , sgpmInterIdx + 1
#endif 
            );
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
            }
#endif
          }
          currCU.firstPU->geoSplitDir  = sgpmInterInfoList[sgpmInterIdx].sgpmSplitDir;
          currCU.firstPU->geoMergeIdx0 = sgpmInterInfoList[sgpmInterIdx].sgpmMode0;
          currCU.firstPU->geoMergeIdx1 = sgpmInterInfoList[sgpmInterIdx].sgpmMode1;
          currCU.firstPU->gpmIntraFlag = ((currCU.firstPU->geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS) || (currCU.firstPU->geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS));
        }
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
        if (currCU.firstPU->sgpmAffine)
        {
          const CompArea                              &area = currCU.Y();
          static_vector<SgpmInterInfo, SGPM_AFFINE_NUM> sgpmAffineInfoList;
          static_vector<double, SGPM_AFFINE_NUM>        sgpmAffineCostList;
          int                                           sgpmAffineIdx = currCU.firstPU->sgpmInterIdx;
          currCU.firstPU->geoTmFlag0   = false;
          currCU.firstPU->geoTmFlag1   = false;
          m_pcIntraPred->deriveSgpmAffineModeOrdered(m_geoMrgCtx, m_geoAffMrgCtx, currCU.cs->picture->getRecoBuf(area), area, currCU, sgpmAffineInfoList, sgpmAffineCostList, m_pcInterPred, sgpmAffineIdx + 1);
          currCU.firstPU->geoSplitDir  = sgpmAffineInfoList[sgpmAffineIdx].sgpmSplitDir;
          currCU.firstPU->geoMergeIdx0 = sgpmAffineInfoList[sgpmAffineIdx].sgpmMode0;
          currCU.firstPU->geoMergeIdx1 = sgpmAffineInfoList[sgpmAffineIdx].sgpmMode1;
          currCU.firstPU->gpmIntraFlag = false;
          currCU.firstPU->affineGPM[0] = sgpmAffineInfoList[sgpmAffineIdx].bAffine0;
          currCU.firstPU->affineGPM[1] = sgpmAffineInfoList[sgpmAffineIdx].bAffine1;
        }
#endif
        xReconInter( currCU );
#if JVET_AF0073_INTER_CCP_MERGE
        CU::saveModelsInHCCP(currCU);
        CU::saveProCcpInfo(currCU);
#endif
        break;
      case MODE_PLT:
      case MODE_INTRA:
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE || JVET_AL0106_BV_EIP || JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV)
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if (currCU.dimd || currCU.sgpm || (chType == CHANNEL_TYPE_LUMA && currCU.tmpFlag) || PU::isBvEip(*currCU.firstPU) || (currCU.timd && !currCU.timdMrg))
#else
#if JVET_AL0106_BV_EIP
        if (currCU.dimd || currCU.sgpm || (chType == CHANNEL_TYPE_LUMA && currCU.tmpFlag) || PU::isBvEip(*currCU.firstPU))
#else
        if (currCU.dimd || currCU.sgpm || (chType == CHANNEL_TYPE_LUMA && currCU.tmpFlag))
#endif
#endif
        {
          m_pcIntraPred->m_bvBasedMergeCandidates.clear();
#if JVET_AH0200_INTRA_TMP_BV_REORDER
          m_pcIntraPred->m_sgpmMvBasedMergeCandidates.clear();
#endif
          PU::getItmpMergeCandidate(*currCU.firstPU, m_pcIntraPred->m_bvBasedMergeCandidates
#if JVET_AH0200_INTRA_TMP_BV_REORDER
             , m_pcIntraPred->m_sgpmMvBasedMergeCandidates
#endif          
          );
        }
#endif
#endif
#if JVET_AM0074_INTRA_MERGE
#if JVET_AG0146_DIMD_ITMP_IBC
        if (currCU.dimd && !currCU.bvgDimdFlag)
        {
          const CompArea &area = currCU.Y();
          m_pcIntraPred->m_bvBasedMergeCostCandidates.clear();
          m_pcIntraPred->m_bvBasedMergeLocDepCandidates.clear();
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
          m_pcIntraPred->getBestNonAnglularMode(currCU.cs->picture->getRecoBuf(area), area, currCU, m_pcIntraPred->m_bvBasedMergeCostCandidates, m_pcIntraPred->m_bvBasedMergeLocDepCandidates);
#else
          m_pcIntraPred->getBestNonAnglularMode(currCU.cs->picture->getRecoBuf(area), area, currCU, m_pcIntraPred->m_bvBasedMergeCandidates, m_pcIntraPred->m_bvBasedMergeCostCandidates, m_pcIntraPred->m_bvBasedMergeLocDepCandidates);
#endif
        }
#endif
        if (currCU.intraMergeMode)
        {
          IntraMergeCandidate ddIntraFusionCand[MAX_NUM_INITIAL_INTRA_MERGE_CAND];
          int validCandSize = m_pcIntraPred->deriveIntraMergeCand(currCU, ddIntraFusionCand);
          const CompArea &area = currCU.Y();
          m_pcIntraPred->reorderIntraMergeCand(area, currCU, ddIntraFusionCand, validCandSize);
          int ddIntrafusionCandIdx = currCU.intraMergeMode - 1;

          currCU.dimdModeMerge = ddIntraFusionCand[ddIntrafusionCandIdx].dimdModeMerge;
          for (int i = 0; i < DIMD_FUSION_NUM - 2; i++)
          {
            currCU.dimdBlendModeMerge[i] = ddIntraFusionCand[ddIntrafusionCandIdx].dimdBlendModeMerge[i];
          }
          for (int m = 0; m < DIMD_FUSION_NUM - 1; m++)
          {
            currCU.dimdLocDepMerge[m] = ddIntraFusionCand[ddIntrafusionCandIdx].dimdLocDepMerge[m];
          }
          for (int m = 0; m < DIMD_FUSION_NUM; m++)
          {
            currCU.dimdRelWeightMerge[m] = ddIntraFusionCand[ddIntrafusionCandIdx].dimdRelWeightMerge[m];
          }
          currCU.isBvDimdExtAvail = false;
          for (int m = 0; m < DIMD_FUSION_NUM; m++)
          {
            currCU.isBvDimdExt[m] = ddIntraFusionCand[ddIntrafusionCandIdx].dimdBv[m];
            if (currCU.isBvDimdExt[m])
            {
              currCU.isBvDimdExtAvail = true;
              currCU.bvDimdExt[m] = Mv(ddIntraFusionCand[ddIntrafusionCandIdx].dimdBvX[m], ddIntraFusionCand[ddIntrafusionCandIdx].dimdBvY[m]);
            }
          }
        }
#endif
#if ENABLE_DIMD
#if JVET_AL0108_BVG_DIMD 
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if ((chType == CHANNEL_TYPE_LUMA) && currCU.dimd && !currCU.bvgDimdFlag)
#else
        if (currCU.dimd && !currCU.bvgDimdFlag)
#endif
#else
        if (currCU.dimd)
#endif
        {
          PredictionUnit *pu = currCU.firstPU;
          const CompArea &area = currCU.Y();
#if JVET_AM0074_INTRA_MERGE
          if (currCU.intraMergeMode > 0)
          {
            pu->intraDir[0] = currCU.dimdModeMerge;
          }
          else
          {
            IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
            pu->intraDir[0] = currCU.dimdMode;
          }
#else
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          pu->intraDir[0] = currCU.dimdMode;
#endif
#endif
#if JVET_AG0146_DIMD_ITMP_IBC
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV && !JVET_AM0074_INTRA_MERGE
          m_pcIntraPred->getBestNonAnglularMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#else
#if JVET_AG0146_DIMD_ITMP_IBC && !JVET_AM0074_INTRA_MERGE
          m_pcIntraPred->getBestNonAnglularMode(currCU.cs->picture->getRecoBuf(area), area, currCU, m_pcIntraPred->m_bvBasedMergeCandidates);
#endif
#endif
#if JVET_AH0076_OBIC
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
          if ((chType == CHANNEL_TYPE_LUMA) && currCU.obicFlag)
#else
          if (currCU.obicFlag)
#endif
          {
            m_pcIntraPred->deriveObicMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
            pu->intraDir[0] = currCU.obicMode[0];
          }
#endif
#endif
        }
#if JVET_AG0146_DIMD_ITMP_IBC
#if JVET_AL0108_BVG_DIMD
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        else if ((chType == CHANNEL_TYPE_LUMA) && currCU.bvgDimdFlag)
#else
        else if (currCU.bvgDimdFlag)
#endif
        {
          PredictionUnit *pu = currCU.firstPU;
          const CompArea &area = currCU.Y();
          m_pcIntraPred->m_bvListSorted.clear();
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          if (currCU.tempType != NO_TEMPLATE)
#else
          RefTemplateType tempType = m_pcIntraPred->getRefTemplateType(currCU, currCU.blocks[COMPONENT_Y]);
          if (tempType != NO_TEMPLATE)
#endif
          {
            m_pcIntraPred->getTargetTemplate(&currCU, currCU.lwidth(), currCU.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                             , currCU.tempType
#else
                                             , tempType
#endif
                                             );
            m_pcIntraPred->candidateSearchIntra(&currCU, currCU.lwidth(), currCU.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                                , currCU.tempType
#else
                                                , tempType
#endif
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
                                        , false
#endif
            );
          }
          m_pcIntraPred->deriveBvgDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          for(int i = 0; i < BVG_DIMD_INTRA_NUM; i++)
          {
            currCU.bvgDimdMode[i] = m_pcIntraPred->m_bvgDimdMode[i];
            currCU.bvgDimdFusionWeight[i] = m_pcIntraPred->m_bvgDimdFusionWeight[i];
          }
          pu->intraDir[0] = currCU.bvgDimdMode[0];
          currCU.bvDimdNum = m_pcIntraPred->m_bvDimdNum;
          for(int i = 0; i < currCU.bvDimdNum; i++)
          { 
            currCU.bvDimdWeight[i] = m_pcIntraPred->m_bvDimdWeight[i];
          }
          for(int i = 0; i < BVG_DIMD_BV_LIST_SIZE; i++)
          {
            currCU.bvDimdList[i] = i < currCU.bvDimdNum ? m_pcIntraPred->m_bvListSorted[i]: Mv(0, 0); 
          }
          pu->interDir = 1;
          pu->refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
          pu->mv[0] = Mv(currCU.bvDimdList[0].hor << MV_FRACTIONAL_BITS_INTERNAL, currCU.bvDimdList[0].ver << MV_FRACTIONAL_BITS_INTERNAL);
          pu->bv = Mv(currCU.bvDimdList[0].hor, currCU.bvDimdList[0].ver);
        }
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        else if ((chType == CHANNEL_TYPE_LUMA) && currCU.timd)
#else
        else if (currCU.timd)
#endif
        {
          PredictionUnit *pu = currCU.firstPU;
          const CompArea &area = currCU.Y();
#if JVET_AJ0061_TIMD_MERGE
          if (currCU.timdMrg)
          {
            m_pcIntraPred->deriveTimdMergeModes(currCU.cs->picture->getRecoBuf(area), area, currCU);
            CHECK(currCU.timdMrgList[currCU.timdMrg - 1][0] < 0, "Wrong timd-merge mode!");
            pu->intraDir[0] = currCU.timdMrgList[currCU.timdMrg - 1][0];
            currCU.timdMode = currCU.timdMrgList[currCU.timdMrg - 1][0]; // temporary
          }
          else
          {
#if SECONDARY_MPM
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif
#if JVET_AJ0146_TIMDSAD
          if (currCU.timdSad)
          {
            m_pcIntraPred->m_timdModeCostList.clear();          
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU
              , true, false, true, false, true);
#else 
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU
              , true, false, true );
#endif
            std::stable_sort(m_pcIntraPred->m_timdModeCostList.begin(),m_pcIntraPred->m_timdModeCostList.end());
            currCU.timdModeSad = m_pcIntraPred->deriveTimdModeSad(currCU.cs->picture->getRecoBuf(area), area, currCU);
          }
          else
          {
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU, true, false, false, false, true);
#else
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif
          }
          pu->intraDir[0] = currCU.timdSad ? currCU.timdModeSad : currCU.timdMode;
#else
          currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          pu->intraDir[0] = currCU.timdMode;
#endif
          }
#if !JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
          if (!currCU.timdMrg && !currCU.lfnstIdx)
          {
            m_pcTrQuant->getTrTypes(*currCU.firstTU, COMPONENT_Y, currCU.timdmTrType[NUM_TIMD_MERGE_MODES][0], currCU.timdmTrType[NUM_TIMD_MERGE_MODES][1]);
          }
#endif
#else
#if SECONDARY_MPM
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif
#if JVET_AJ0146_TIMDSAD
          if (currCU.timdSad)
          {
            m_pcIntraPred->m_timdModeCostList.clear();          
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU
              , true, false, true );
            std::stable_sort(m_pcIntraPred->m_timdModeCostList.begin(),m_pcIntraPred->m_timdModeCostList.end());
            currCU.timdModeSad = m_pcIntraPred->deriveTimdModeSad(currCU.cs->picture->getRecoBuf(area), area, currCU);
          }
          else
          {
            currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          }
          pu->intraDir[0] = currCU.timdSad ? currCU.timdModeSad : currCU.timdMode;
#else
          currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          pu->intraDir[0] = currCU.timdMode;
#endif
#endif
        }
#endif

#if JVET_AK0059_MDIP
        else if (currCU.mdip)
        {
          PredictionUnit *pu   = currCU.firstPU;
          const CompArea &area = currCU.Y();
          m_pcIntraPred->deriveMdipMode(currCU.cs->picture->getRecoBuf(area), area, currCU, false);
          pu->intraDir[0] = currCU.mdipMode;          
        }
#endif
#if JVET_AB0155_SGPM
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        else if ((chType == CHANNEL_TYPE_LUMA) && currCU.sgpm)
#else
        else if (currCU.sgpm)
#endif
        {
          PredictionUnit *pu   = currCU.firstPU;
          const CompArea &area = currCU.Y();
#if SECONDARY_MPM
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif
          static_vector<SgpmInfo, SGPM_NUM> sgpmInfoList;
          static_vector<double, SGPM_NUM>   sgpmCostList;
          int                         sgpmIdx = currCU.sgpmIdx;
#if JVET_AJ0112_REGRESSION_SGPM
          if (currCU.lwidth() * currCU.lheight() <= 1024 && currCU.cs->sps->getUseTimd() && !PU::isRegressionSgpm(*pu))
#else
          if (currCU.lwidth() * currCU.lheight() <= 1024 && currCU.cs->sps->getUseTimd() )
#endif
          {
            m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU, false, true);
          }

          m_pcIntraPred->deriveSgpmModeOrdered(currCU.cs->picture->getRecoBuf(area), area, currCU, sgpmInfoList, sgpmCostList);

          currCU.sgpmSplitDir = sgpmInfoList[sgpmIdx].sgpmSplitDir;
          currCU.sgpmMode0    = sgpmInfoList[sgpmIdx].sgpmMode0;
          currCU.sgpmMode1    = sgpmInfoList[sgpmIdx].sgpmMode1;

#if JVET_AG0152_SGPM_ITMP_IBC
          currCU.sgpmBv0 = sgpmInfoList[sgpmIdx].sgpmBv0;
          currCU.sgpmBv1 = sgpmInfoList[sgpmIdx].sgpmBv1;
#if JVET_AJ0112_REGRESSION_SGPM
          currCU.blendModel = sgpmInfoList[sgpmIdx].blendModel;
#endif
          pu->intraDir[0] = currCU.sgpmMode0 >= SGPM_BV_START_IDX ? 0 : currCU.sgpmMode0;
          pu->intraDir1[0] = currCU.sgpmMode1 >= SGPM_BV_START_IDX ? 0 : currCU.sgpmMode1;
#else
          pu->intraDir[0]  = currCU.sgpmMode0;
          pu->intraDir1[0] = currCU.sgpmMode1;
#endif
        }
#endif

#if JVET_AB0157_TMRL
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        else if ((chType == CHANNEL_TYPE_LUMA) && currCU.tmrlFlag)
#else
        else if (currCU.tmrlFlag)
#endif
        {
          PredictionUnit* pu = currCU.firstPU;
          const CompArea& area = currCU.Y();
#if SECONDARY_MPM
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif
          m_pcIntraPred->getTmrlList(currCU);
          pu->multiRefIdx = m_pcIntraPred->m_tmrlList[currCU.tmrlListIdx].multiRefIdx;
          pu->intraDir[0] = m_pcIntraPred->m_tmrlList[currCU.tmrlListIdx].intraDir;
        }
#endif
#if JVET_AK0061_PDP_MPM
        else if (currCU.plIdx) 
        {
          currCU.firstPU->intraDir[0] = PLANAR_IDX;
        }
#endif

        else if (currCU.firstPU->parseLumaMode)
        {
          const CompArea &area = currCU.Y();
#if JVET_AK0059_MDIP
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU, true);
#if !JVET_AK0061_PDP_MPM
          if(PU::allowMPMSorted(*currCU.firstPU))
          {
            for (int i = 0; i <= EXT_VDIA_IDX; i++)
            {
              g_intraModeCost[i] = MAX_UINT64;
            }
          }
#endif
          if(CU::allowMdip(currCU))
          {
            m_pcIntraPred->deriveMdipMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          }
#else
          IntraPrediction::deriveDimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
#endif          
        }

        //redo prediction dir derivation
        if (currCU.firstPU->parseLumaMode)
        {
#if SECONDARY_MPM
          uint8_t* mpmPred = m_pcIntraPred->m_intraMPM;  // mpm_idx / rem_intra_luma_pred_mode
          uint8_t* nonMpmPred = m_pcIntraPred->m_intraNonMPM;
#if JVET_AD0085_MPM_SORTING
#if JVET_AK0061_PDP_MPM
          bool enablePlanarSort = false;
          enablePlanarSort = PU::determinePDPTemp(*currCU.firstPU);
#if JVET_AL0125_IMPROVEMENT_ON_MPM
          bool planarDisable = false;
          if (currCU.slice->getPnnMode() && IntraPredictionNN::hasPnnPrediction(currCU))
          {
            planarDisable = true;
          }
          if (PU::allowMPMSorted(*currCU.firstPU)
              && !(currCU.firstPU->mpmFlag && currCU.firstPU->ipredIdx == 0 && !(enablePlanarSort || planarDisable)))
#else
          if (PU::allowMPMSorted(*currCU.firstPU)
              && !(currCU.firstPU->mpmFlag && currCU.firstPU->ipredIdx == 0 && !enablePlanarSort))
#endif
#else
          if (PU::allowMPMSorted(*currCU.firstPU) && !(currCU.firstPU->mpmFlag && currCU.firstPU->ipredIdx == 0))
#endif
          {
            PU::getIntraMPMs(*currCU.firstPU, mpmPred, nonMpmPred
#if JVET_AC0094_REF_SAMPLES_OPT
              , true
#if JVET_AK0061_PDP_MPM
              , true
              , true
#endif
#endif


              , m_pcIntraPred
            );
          }
          else
          {
#endif
          PU::getIntraMPMs( *currCU.firstPU, mpmPred, nonMpmPred
#if JVET_AC0094_REF_SAMPLES_OPT
                           , false
#endif
#if JVET_AK0061_PDP_MPM
            , true, false, m_pcIntraPred
#endif
          );
#if JVET_AD0085_MPM_SORTING
          }
#endif
#else
          unsigned int mpmPred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
          PU::getIntraMPMs(*currCU.firstPU, mpmPred);
#endif
          if (currCU.firstPU->mpmFlag)
          {
            currCU.firstPU->intraDir[0] = mpmPred[currCU.firstPU->ipredIdx];
          }
          else
          {
#if SECONDARY_MPM
            if (currCU.firstPU->secondMpmFlag)
            {
              currCU.firstPU->intraDir[0] = mpmPred[currCU.firstPU->ipredIdx];
            }
            else
            {
              currCU.firstPU->intraDir[0] = nonMpmPred[currCU.firstPU->ipredIdx];
            }
#else
            //postponed sorting of MPMs (only in remaining branch)
            std::sort(mpmPred, mpmPred + NUM_MOST_PROBABLE_MODES);
            unsigned ipredMode = currCU.firstPU->ipredIdx;

            for (uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
            {
              ipredMode += (ipredMode >= mpmPred[i]);
            }
            currCU.firstPU->intraDir[0] = ipredMode;
#endif
          }
        }
#if JVET_AH0136_CHROMA_REORDERING
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
        if (currCU.firstPU->parseChromaMode 
          && (!( CS::isDualITree(cs) || (currCU.isSST && currCU.separateTree) ) || !currCU.cs->sps->getUseChromaReordering() || !currCU.cs->slice->isIntra())
          )
#else
        if (currCU.firstPU->parseChromaMode && (!CS::isDualITree(cs) || !currCU.cs->sps->getUseChromaReordering()))
#endif
#else
        if (currCU.firstPU->parseChromaMode)
#endif
        {
          unsigned chromaCandModes[NUM_CHROMA_MODE];
          PU::getIntraChromaCandModes(*currCU.firstPU, chromaCandModes);

          CHECK(currCU.firstPU->candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
          CHECK(PU::isLMCMode(chromaCandModes[currCU.firstPU->candId]), "The intra dir cannot be LM_CHROMA for this path");
          CHECK(chromaCandModes[currCU.firstPU->candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
          CHECK(chromaCandModes[currCU.firstPU->candId] == DIMD_CHROMA_IDX, "The intra dir cannot be DIMD_CHROMA for this path");
#endif

          currCU.firstPU->intraDir[1] = chromaCandModes[currCU.firstPU->candId];
        }
#else
#if JVET_W0123_TIMD_FUSION || JVET_AJ0249_NEURAL_NETWORK_BASED
        if (currCU.timd)
        {
          PredictionUnit *pu = currCU.firstPU;
          const CompArea &area = currCU.Y();
          currCU.timdMode = m_pcIntraPred->deriveTimdMode(currCU.cs->picture->getRecoBuf(area), area, currCU);
          pu->intraDir[0] = currCU.timdMode;
        }

        //redo prediction dir derivation
        if (currCU.firstPU->parseLumaMode)
        {
#if SECONDARY_MPM
          uint8_t* mpmPred = currCU.firstPU->intraMPM;  // mpm_idx / rem_intra_luma_pred_mode
          uint8_t* nonMpmPred = currCU.firstPU->intraNonMPM;
          PU::getIntraMPMs( *currCU.firstPU, mpmPred, nonMpmPred );
#else
          unsigned int mpmPred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
          PU::getIntraMPMs(*currCU.firstPU, mpmPred);
#endif
          if (currCU.firstPU->mpmFlag)
          {
            currCU.firstPU->intraDir[0] = mpmPred[currCU.firstPU->ipredIdx];
          }
          else
          {
#if SECONDARY_MPM
            if (currCU.firstPU->secondMpmFlag)
            {
              currCU.firstPU->intraDir[0] = mpmPred[currCU.firstPU->ipredIdx];
            }
            else
            {
              currCU.firstPU->intraDir[0] = nonMpmPred[currCU.firstPU->ipredIdx];
            }
#else
            //postponed sorting of MPMs (only in remaining branch)
            std::sort(mpmPred, mpmPred + NUM_MOST_PROBABLE_MODES);
            unsigned ipredMode = currCU.firstPU->ipredIdx;

            for (uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
            {
              ipredMode += (ipredMode >= mpmPred[i]);
            }
            currCU.firstPU->intraDir[0] = ipredMode;
#endif
          }
        }
        if (currCU.firstPU->parseChromaMode)
        {
          unsigned chromaCandModes[NUM_CHROMA_MODE];
          PU::getIntraChromaCandModes(*currCU.firstPU, chromaCandModes);

          CHECK(currCU.firstPU->candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
          CHECK(PU::isLMCMode(chromaCandModes[currCU.firstPU->candId]), "The intra dir cannot be LM_CHROMA for this path");
          CHECK(chromaCandModes[currCU.firstPU->candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");

          currCU.firstPU->intraDir[1] = chromaCandModes[currCU.firstPU->candId];
        }
#endif
#endif
#if JVET_AK0217_INTRA_MTSS
        if (currCU.firstTU->mdirIdx[COMPONENT_Y])
        {
          const CompArea& area = currCU.Y();
          IntraPrediction::deriveDimdModeList(currCU.cs->picture->getRecoBuf(area), area, currCU, currCU.candModeListForTransformMtss, currCU.candCostListForTransformMtss);
        }
#endif 
        xReconIntraQT( currCU );
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if (!currCU.timdMrg && !currCU.lfnstIdx)
        {
          m_pcTrQuant->getTrTypes(*currCU.firstTU, COMPONENT_Y, currCU.timdmTrType[NUM_TIMD_MERGE_MODES][0], currCU.timdmTrType[NUM_TIMD_MERGE_MODES][1]);
        }
#endif
#if JVET_AF0079_STORING_INTRATMP
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if ((chType == CHANNEL_TYPE_LUMA) && currCU.tmpFlag)
#else
        if (currCU.tmpFlag)
#endif
        {
          bool isIbcSmallBlk = false;
          CU::saveMotionInHMVP(currCU, isIbcSmallBlk);
        }
#endif
#if JVET_AD0188_CCP_MERGE
        CU::saveModelsInHCCP(currCU);
#endif
#if JVET_AG0058_EIP
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        if (chType == CHANNEL_TYPE_LUMA)
        {
#endif
          CU::saveModelsInHEIP(currCU);
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
        }
#endif
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
        CU::saveCcInsideFilterFlagInCCP(currCU);
#endif
        break;
      default:
        THROW( "Invalid prediction mode" );
        break;
      }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      if (changeProcIntraReg )
      {
        currCU.slice->setProcessingIntraRegion(!currCU.slice->getProcessingIntraRegion());
      }
#endif


      m_pcInterPred->xFillIBCBuffer(currCU);
#if JVET_Z0118_GDR // decompressCtu
      cs.updateReconMotIPM( currCU ); // decompressCtu : need
#endif

      DTRACE_BLOCK_REC(cs.picture->getRecoBuf(currCU), currCU, currCU.predMode );
#if NNVC_USE_PRED && NN_LF_PRED_FIX
      cs.getPredBufCustom(currCU).copyFrom(cs.getPredBuf(currCU));
#endif
      if (CU::isInter(currCU))
      {
        DTRACE_MOT_FIELD(g_trace_ctx, *currCU.firstPU);
      }
    }
  }
#if K0149_BLOCK_STATISTICS
  getAndStoreBlockStatistics(cs, ctuArea);
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecCu::xIntraRecBlk( TransformUnit& tu, const ComponentID compID )
{
  if( !tu.blocks[ compID ].valid() )
  {
    return;
  }

        CodingStructure &cs = *tu.cs;
  const CompArea &area      = tu.blocks[compID];
#if SIGN_PREDICTION
  const bool  isJCCR = tu.jointCbCr && isChroma(compID);
  const CompArea &areaCr      = tu.blocks[isJCCR ? COMPONENT_Cr : compID];
  PelBuf piPredCr;
  if(isJCCR)
  {
    piPredCr = cs.getPredBuf( tu.blocks[COMPONENT_Cr] );
  }
#endif

  const ChannelType chType  = toChannelType( compID );

        PelBuf piPred       = cs.getPredBuf( area );

#if NNVC_USE_PRED
        PelBuf piPredCustom = cs.getPredBufCustom(area);
#endif

#if JVET_AB0061_ITMP_BV_FOR_IBC
  PredictionUnit &pu = *tu.cs->getPU(area.pos(), chType);
#else
  const PredictionUnit &pu  = *tu.cs->getPU( area.pos(), chType );
#endif

#if ENABLE_DIMD
#if JVET_AG0058_EIP
  if (pu.parseChromaMode && compID == COMPONENT_Cb && !CS::isDualITree(cs))
  {
    unsigned chromaCandModes[NUM_CHROMA_MODE];
    PU::getIntraChromaCandModes(pu, chromaCandModes);
    pu.intraDir[1] = chromaCandModes[pu.candId];
  }
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
#if JVET_AH0136_CHROMA_REORDERING
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
#if JVET_AJ0081_CHROMA_TMRL
  if (((!PU::isLMCMode(pu.intraDir[1]) && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma)
    && (CS::isDualITree(cs) || (pu.cu->isSST && pu.cu->separateTree)) && pu.cs->sps->getUseChromaReordering() && pu.cs->slice->isIntra()
    )
    || ((pu.intraDir[1] == DIMD_CHROMA_IDX || pu.ccpMergeFusionType == 1 || pu.chromaTmrlFlag) && compID == COMPONENT_Cb)
    )
#else
  if (((!PU::isLMCMode(pu.intraDir[1]) && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma)
    && (CS::isDualITree(cs) || (pu.cu->isSST && pu.cu->separateTree)) && pu.cs->sps->getUseChromaReordering() && pu.cs->slice->isIntra()
    )
    || ((pu.intraDir[1] == DIMD_CHROMA_IDX || pu.ccpMergeFusionType == 1) && compID == COMPONENT_Cb)
    )
#endif
#else
  if (((!PU::isLMCMode(pu.intraDir[1]) && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma) && CS::isDualITree(cs) && pu.cs->sps->getUseChromaReordering()) || ((pu.intraDir[1] == DIMD_CHROMA_IDX || pu.ccpMergeFusionType == 1) && compID == COMPONENT_Cb))
#endif
#else
  if ((pu.intraDir[1] == DIMD_CHROMA_IDX || pu.ccpMergeFusionType == 1) && compID == COMPONENT_Cb)
#endif
#else
  if (pu.intraDir[1] == DIMD_CHROMA_IDX && compID == COMPONENT_Cb)
#endif
  {
    CompArea areaCb = pu.Cb();
    CompArea areaCr = pu.Cr();
    CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, areaCb.lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, areaCb.size()));
    IntraPrediction::deriveDimdChromaMode(cs.picture->getRecoBuf(lumaArea), cs.picture->getRecoBuf(areaCb), cs.picture->getRecoBuf(areaCr), lumaArea, areaCb, areaCr, *pu.cu);
#if JVET_AC0094_REF_SAMPLES_OPT
    if (PU::getCoLocatedIntraLumaMode(pu) == (tu.cu)->dimdChromaMode)
    {
      if ((tu.cu)->dimdChromaMode == (tu.cu)->dimdChromaModeSecond)
      {
        (tu.cu)->dimdChromaMode = DC_IDX;
      }
      else
      {
        (tu.cu)->dimdChromaMode = (tu.cu)->dimdChromaModeSecond;
      }
    }
#endif
  }
#endif
#if JVET_AJ0081_CHROMA_TMRL
  if (pu.chromaTmrlFlag && compID == COMPONENT_Cb && CS::isDualITree(cs))
  {
    CompArea areaCb = pu.Cb();
    CompArea areaCr = pu.Cr();
    CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, areaCb.lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, areaCb.size()));
    m_pcIntraPred->getChromaTmrlList(cs.picture->getRecoBuf(lumaArea), cs.picture->getRecoBuf(areaCb), cs.picture->getRecoBuf(areaCr), lumaArea, areaCb, areaCr, *pu.cu, pu, m_pcInterPred);
    pu.chromaMrlIdx = m_pcIntraPred->m_chromaTmrlList[pu.chromaTmrlIdx].multiRefIdx;
    pu.intraDir[1] = m_pcIntraPred->m_chromaTmrlList[pu.chromaTmrlIdx].intraDir;
    CHECK(pu.intraDir[1] >= NUM_LUMA_MODE, "error intra mode")
  }
#endif
#if JVET_AH0136_CHROMA_REORDERING
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
#if JVET_AJ0081_CHROMA_TMRL
  if (
    (!PU::isLMCMode(pu.intraDir[1]) && !pu.chromaTmrlFlag && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma)
    && (CS::isDualITree(cs) || (pu.cu->isSST && pu.cu->separateTree)) && pu.cs->sps->getUseChromaReordering() && pu.cu->slice->isIntra()
    )
#else
  if (
    (!PU::isLMCMode(pu.intraDir[1]) && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma)
    && (CS::isDualITree(cs) || (pu.cu->isSST && pu.cu->separateTree)) && pu.cs->sps->getUseChromaReordering() && pu.cu->slice->isIntra()
    )
#endif
#else
  if ((!PU::isLMCMode(pu.intraDir[1]) && compID == COMPONENT_Cb && !pu.cu->bdpcmModeChroma) && CS::isDualITree(cs) && pu.cs->sps->getUseChromaReordering())
#endif
  {
    CompArea areaCb = pu.Cb();
    CompArea areaCr = pu.Cr();
    CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, areaCb.lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, areaCb.size()));
    m_pcIntraPred->deriveNonCcpChromaModes(cs.picture->getRecoBuf(lumaArea), cs.picture->getRecoBuf(areaCb), cs.picture->getRecoBuf(areaCr), lumaArea, areaCb, areaCr, *pu.cu, pu, m_pcInterPred);
    int chromaIdx = -1;
#if JVET_AC0071_DBV
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
    if (cs.slice->getSPS()->getUseDimd())
    {
      chromaIdx = pu.intraDir[1] == DBV_CHROMA_IDX ? 0 : (pu.intraDir[1] == DM_CHROMA_IDX ? 1 : (pu.intraDir[1] == DIMD_CHROMA_IDX ? 2 : pu.candId + 3));
    }
    else
    {
      chromaIdx = pu.intraDir[1] == DBV_CHROMA_IDX ? 0 : (pu.intraDir[1] == DM_CHROMA_IDX ? 1 : pu.candId + 2);
    }
#else
    chromaIdx = pu.intraDir[1] == DBV_CHROMA_IDX ? 0 : (pu.intraDir[1] == DM_CHROMA_IDX ? 1 : pu.candId + 2);
#endif
    if (!PU::hasChromaBvFlag(pu))
    {
      chromaIdx--;
    }
#else
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
    if (cs.slice->getSPS()->getUseDimd())
    {
      chromaIdx = pu.intraDir[1] == DM_CHROMA_IDX ? 0 : (pu.intraDir[1] == DIMD_CHROMA_IDX ? 1 : pu.candId + 2);
    }
    else
    {
      chromaIdx = pu.intraDir[1] == DM_CHROMA_IDX ? 0 : pu.candId + 1;
    }
#else
    chromaIdx = pu.intraDir[1] == DM_CHROMA_IDX ? 0 : pu.candId + 1;
#endif
#endif
    pu.intraDir[1] = pu.cu->chromaList[chromaIdx];
#if JVET_AC0071_DBV
    if (PU::isDbvMode(pu.intraDir[1]) && CS::isDualITree(cs))
    {
      pu.bv = pu.cu->bvs[pu.intraDir[1] - DBV_CHROMA_IDX];
      pu.mv[0] = pu.cu->mvs[pu.intraDir[1] - DBV_CHROMA_IDX];
      pu.cu->rribcFlipType = pu.cu->rribcTypes[pu.intraDir[1] - DBV_CHROMA_IDX];
    }
#endif
    CHECK(pu.intraDir[1] < 0, "wrong intraDir");
  }
#endif
#if JVET_AE0100_BVGCCCM
  if (pu.bvgCccmFlag && compID == COMPONENT_Cb)
  {
    bool validBv = false;
    PU::getBvgCccmCands(pu, validBv);
    CHECK(!validBv, "No valid BV found!");
  }
#endif
#if JVET_AC0071_DBV
  if (pu.intraDir[1] == DBV_CHROMA_IDX && compID == COMPONENT_Cb
#if JVET_AH0136_CHROMA_REORDERING
    && !pu.cs->sps->getUseChromaReordering()
#endif
    )
  {
    PU::deriveChromaBv(pu);
  }
#endif
  uint32_t uiChFinalMode = PU::getFinalIntraMode(pu, chType);
#else
#if JVET_AG0058_EIP
  if (pu.parseChromaMode && compID == COMPONENT_Cb && !CS::isDualITree(cs))
  {
    unsigned chromaCandModes[NUM_CHROMA_MODE];
    PU::getIntraChromaCandModes(pu, chromaCandModes);
    pu.intraDir[1] = chromaCandModes[pu.candId];
  }
#endif
  const uint32_t uiChFinalMode = PU::getFinalIntraMode(pu, chType);
#endif
  PelBuf pReco              = cs.getRecoBuf(area);



  //===== init availability pattern =====
  bool predRegDiffFromTB = CU::isPredRegDiffFromTB(*tu.cu, compID);
  bool firstTBInPredReg = CU::isFirstTBInPredReg(*tu.cu, compID, area);
  CompArea areaPredReg(COMPONENT_Y, tu.chromaFormat, area);
#if SIGN_PREDICTION
  if( !isJCCR || compID != COMPONENT_Cr )
  {
#endif
  if (tu.cu->ispMode && isLuma(compID))
  {
    if (predRegDiffFromTB)
    {
      if (firstTBInPredReg)
      {
        CU::adjustPredArea(areaPredReg);
        m_pcIntraPred->initIntraPatternChTypeISP(*tu.cu, areaPredReg, pReco);
      }
    }
    else
    {
      m_pcIntraPred->initIntraPatternChTypeISP(*tu.cu, area, pReco);
    }
  }
#if JVET_AA0057_CCCM
  else if ( isLuma(compID) || !pu.cccmFlag )
#else
  else
#endif
  {
#if JVET_AJ0249_NEURAL_NETWORK_BASED
    bool isUnused = isLuma(compID) && ((tu.cu)->tmpFlag || (tu.cu)->eipFlag);
    if (uiChFinalMode == PNN_IDX)
    {
      isUnused = !IntraPredictionNN::isUpsamplingNeeded(area);
    }
    if (!isUnused)
    {
#endif
    m_pcIntraPred->initIntraPatternChType(*tu.cu, area);
#if JVET_AJ0249_NEURAL_NETWORK_BASED
    }
#endif
  }

  //===== get prediction signal =====
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION	  
  if (compID != COMPONENT_Y && pu.decoderDerivedCcpMode)
  {
    if (compID == COMPONENT_Cb)
    {
      m_pcIntraPred->xGetLumaRecPixels(pu, area);
      pu.cccmFlag = 1;
      m_pcIntraPred->xGetLumaRecPixels(pu, area);
      std::vector<DecoderDerivedCcpCandidate> decoderDerivedCcpList;
      m_pcIntraPred->m_mmlmThreshold2 = m_pcIntraPred->xCccmCalcRefAver(pu, 2);
      m_pcIntraPred->decoderDerivedCcp(pu, decoderDerivedCcpList);

      pu.intraDir[1] = decoderDerivedCcpList[0].lmIndex;
      pu.cccmFlag = decoderDerivedCcpList[0].isCccm;
      pu.glCccmFlag = decoderDerivedCcpList[0].isGlcccm;
      pu.ccInsideFilter = decoderDerivedCcpList[0].isInsideFilter;
      pu.curCand = decoderDerivedCcpList[0].ddccpCand;
      PelBuf predCr = cs.getPredBuf(tu.blocks[COMPONENT_Cr]);
      if (pu.cccmFlag)
      {
        m_pcIntraPred->predIntraCCCM(pu, piPred, predCr, pu.intraDir[1]);
      }
      else
      {
        CclmModel modelsCb, modelsCr;
        PU::ccpParamsToCclmModel(COMPONENT_Cb, pu.curCand, modelsCb);
        PU::ccpParamsToCclmModel(COMPONENT_Cr, pu.curCand, modelsCr);
        m_pcIntraPred->predIntraChromaLM(compID, piPred, pu, area, pu.intraDir[1], false, &modelsCb);
        m_pcIntraPred->predIntraChromaLM(COMPONENT_Cr, predCr, pu, area, pu.intraDir[1], false, &modelsCr);
      }
      m_pcIntraPred->predDecoderDerivedIntraCCCMFusions(pu, piPred, predCr, decoderDerivedCcpList);
      pu.cccmFlag = 0;
      pu.glCccmFlag = 0;
      pu.ccInsideFilter = 0;
    }
  }
  else
#endif
#if JVET_AD0188_CCP_MERGE
  if (compID != COMPONENT_Y && pu.idxNonLocalCCP)
  {
    if (compID == COMPONENT_Cb)
    {
      PelBuf            predCr = cs.getPredBuf(tu.blocks[COMPONENT_Cr]);
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
      CCPModelCandidate candList[MAX_CCP_CAND_LIST_INIT_SIZE];
#else
      CCPModelCandidate candList[MAX_CCP_CAND_LIST_SIZE];
#endif

      int candIdx = pu.idxNonLocalCCP - 1;

#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
      static CCPModelCandidate cand[MAX_NUM_DERIVED_CCP_ADJUST_CAND];
      int candFlyNum = 0;
      int candNum = PU::getCCPModelCandidateList(pu, candList, false, 0, NULL, -1, &cand[0], candFlyNum);
      if (pu.ccpMergeAdjustFlag && candNum < MAX_CCP_CAND_LIST_INIT_SIZE)
      {
        m_pcIntraPred->deriveCcpMergeAdjustCands(pu, pu.blocks[1], &cand[0], candFlyNum);
        for (int i = 0; i < candFlyNum; i++)
        {
          candList[candNum] = cand[i];
          candNum++;
          if (candNum >= MAX_CCP_CAND_LIST_INIT_SIZE)
          {
            break;
          }
        }
      }
#else
      int candNum         = PU::getCCPModelCandidateList(pu, candList);
#endif
      bool hasFilteredCCCM   = false;
      bool hasFilteredCCLM   = false;
      bool hasFilteredNSCCCM = false;
      bool hasFilteredGLM[8] = { false, false, false, false, false, false, false, false};
#if JVET_AD0202_CCCM_MDF
      bool hasFilteredMFCCCM = false;
#endif

      for (int i = 0; i < candNum; i++)
      {
        if (candList[i].type & (CCP_TYPE_CCCM | CCP_TYPE_GLCCCM))
        {
          if (!hasFilteredCCCM)
          {
            m_pcIntraPred->xCccmCreateLumaRef(pu, area);
            hasFilteredCCCM = true;
          }
        }
#if JVET_AD0202_CCCM_MDF
        else if (candList[i].type & CCP_TYPE_MDFCCCM)
        {
          if (!hasFilteredMFCCCM)
          {
            pu.cccmMultiFilterIdx = candList[i].cccmMultiFilterIdx;
            if (!hasFilteredCCCM)
            {
              m_pcIntraPred->xCccmCreateLumaRef(pu, area, 0);
              hasFilteredCCCM = true;
            }
            m_pcIntraPred->xCccmCreateLumaRef(pu, area, 1);
            m_pcIntraPred->xCccmCreateLumaRef(pu, area, 2);
            m_pcIntraPred->xCccmCreateLumaRef(pu, area, 3);
            pu.cccmMultiFilterIdx = 0;
            hasFilteredMFCCCM = true;
          }
        }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AF0073_INTER_CCP_MERGE
        else if (candList[i].type & CCP_TYPE_NSCCCM
#if JVET_AF0073_INTER_CCP_MERGE
          || (candList[i].type & CCP_TYPE_INTER_CCCM)
#endif
          )
        {
          if (!hasFilteredNSCCCM)
          {
            pu.cccmNoSubFlag = 1;
            m_pcIntraPred->xCccmCreateLumaNoSubRef(pu, area);
            pu.cccmNoSubFlag = 0;
            hasFilteredNSCCCM  = true;
          }
        }
#endif
        else if (candList[i].type & CCP_TYPE_CCLM)
        {
          if (!hasFilteredCCLM)
          {
            m_pcIntraPred->xGetLumaRecPixels(pu, area);
            hasFilteredCCLM = true;
          }
        }
        else if (candList[i].type & (CCP_TYPE_GLM0123 | CCP_TYPE_GLM4567))
        {
          int filtertype = candList[i].glmIdc - 1;
          if (!hasFilteredGLM[filtertype])
          {
            pu.glmIdc.cr0 = pu.glmIdc.cb0 = filtertype + 1;
            m_pcIntraPred->xGetLumaRecPixels(pu, area);
            pu.glmIdc.cr0 = pu.glmIdc.cb0 = 0;
            hasFilteredGLM[filtertype] = true;
          }
        }
        else
        {
          THROW("Invalid type");
        }
      }
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
      if (pu.ccpMergeFusionFlag && pu.ccpMergeFusionType == 0 && !hasFilteredCCCM)
      {
        m_pcIntraPred->xCccmCreateLumaRef(pu, area, 0);
        hasFilteredCCCM = true;
      }
#endif
      CHECK(pu.idxNonLocalCCP < 1 || pu.idxNonLocalCCP > MAX_CCP_CAND_LIST_SIZE, " Invalid idxNonLocalCCP index");

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
      int fusionList[MAX_CCP_CAND_LIST_INIT_SIZE * 2] = { MAX_CCP_CAND_LIST_INIT_SIZE };
#else
      int fusionList[MAX_CCP_FUSION_NUM * 2] = { MAX_CCP_FUSION_NUM };
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
      if (pu.ccpMergeAdjustFlag)
      {
        int fusionList2[MAX_CCP_CAND_LIST_INIT_SIZE * 2] = { MAX_CCP_CAND_LIST_INIT_SIZE };
        m_pcIntraPred->reorderCCPCandidates(pu, candList, candNum, fusionList2);
        candNum = std::min(candNum, pu.cs->sps->getMaxNumCcpMergeAdjustCand());
        if (pu.ccpMergeAdjustFlag == 2)
        {
          pu.ccpMergeAdjustFlag = 1;
          m_pcIntraPred->adjustCCPCandidates(pu, candList, candNum, candIdx);
          pu.ccpMergeAdjustFlag = 2;
          m_pcIntraPred->adjustCCPCandidates(pu, candList, candNum, candIdx);
        }
        else
        {
          m_pcIntraPred->adjustCCPCandidates(pu, candList, candNum, candIdx);
        }
      }
      else
#endif
      m_pcIntraPred->reorderCCPCandidates(pu, candList, candNum, fusionList);
#else
      m_pcIntraPred->reorderCCPCandidates(pu, candList, candNum);
#endif
      pu.curCand = candList[candIdx];
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
      if (pu.ddNonLocalCCPFusion > 0)
      {
        const int idx = 2 * (pu.ddNonLocalCCPFusion - 1);
        m_pcIntraPred->predDecoderDerivedCcpMergeFusion(pu, piPred, predCr, candList[fusionList[idx]], candList[fusionList[idx + 1]]
#if JVET_AN0168_REGRESSION_CCP_FUSION
          , (pu.ddNonLocalCCPFusion - 1)
#endif
        );
      }
      else
#endif
      m_pcIntraPred->predCCPCandidate(pu, piPred, predCr);
    }
  }
  else
#endif
#if JVET_AA0057_CCCM
  if( compID != COMPONENT_Y && pu.cccmFlag )
  {
    // Create both Cb and Cr predictions when here for Cb
    if( compID == COMPONENT_Cb )
    {
#if JVET_AD0202_CCCM_MDF
      PredictionUnit& pu = *tu.cu->firstPU;
#else
      const PredictionUnit& pu = *tu.cu->firstPU;
#endif
      PelBuf predCr            = cs.getPredBuf( tu.blocks[COMPONENT_Cr] );
      
#if JVET_AD0202_CCCM_MDF
      if (pu.cccmMultiFilterIdx == 1)
      {
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 0);
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 1);
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 2);
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 3);
      }
      else if (pu.cccmMultiFilterIdx == 2)
      {
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 0);
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 1);
      }
      else if (pu.cccmMultiFilterIdx == 3)
      {
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 0);
        m_pcIntraPred->xGetLumaRecPixels(pu, area, 3);
      }
      else
      {
        m_pcIntraPred->xGetLumaRecPixels(pu, area);
      }
#else
      m_pcIntraPred->xGetLumaRecPixels( pu, area );
#endif
      m_pcIntraPred->predIntraCCCM( pu, piPred, predCr, uiChFinalMode );
    }
  }
  else
#endif
  if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
  {
#if JVET_AK0064_CCP_LFNST_NSPT
    if (compID == COMPONENT_Cb)
    {
#endif
#if JVET_AD0188_CCP_MERGE
      PredictionUnit& pu = *tu.cu->firstPU;
#else
      const PredictionUnit& pu = *tu.cu->firstPU;
#endif
      m_pcIntraPred->xGetLumaRecPixels( pu, area );
      m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
#if JVET_AK0064_CCP_LFNST_NSPT
      CompArea areaCr = pu.Cr();
      m_pcIntraPred->initIntraPatternChType(*tu.cu, areaCr);
      PelBuf predCr = cs.getPredBuf(tu.blocks[COMPONENT_Cr]);
      m_pcIntraPred->xGetLumaRecPixels(pu, areaCr);
      m_pcIntraPred->predIntraChromaLM(COMPONENT_Cr, predCr, pu, areaCr, uiChFinalMode);
    }
#endif
  }
  else
  {
#if JVET_V0130_INTRA_TMP
	  if (PU::isTmp(pu, chType))
	  {
		  int foundCandiNum;
#if JVET_W0069_TMP_BOUNDARY
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
      if ((tu.cu)->tempType != NO_TEMPLATE)
#else
		  RefTemplateType tempType = m_pcIntraPred->getRefTemplateType(*(tu.cu), tu.cu->blocks[COMPONENT_Y]);

      if( tempType != NO_TEMPLATE )
#endif
		  {
        m_pcIntraPred->getTargetTemplate(tu.cu, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                         , (tu.cu)->tempType
#else
                                         , tempType
#endif
                                         );

        m_pcIntraPred->candidateSearchIntra(tu.cu, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                            , (tu.cu)->tempType
#else
                                            , tempType
#endif
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
                                            , false
#endif
                                            );
#if JVET_AH0200_INTRA_TMP_BV_REORDER && !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        if(!tu.cu->tmpFlmFlag && !tu.cu->tmpFusionFlag)
        {
          if(pu.lwidth() * pu.lheight() > TMP_SKIP_REFINE_THRESHOLD)
          {
            tu.cu->tmpIsSubPel  = 0;
            tu.cu->tmpSubPelIdx = 0;
          } 
          else
          {
#if JVET_AI0129_INTRA_TMP_OVERLAPPING_REFINEMENT
            m_pcIntraPred->searchFracCandidate(tu.cu, m_pcIntraPred->getTargetPatch(), tempType);
#else
            m_pcIntraPred->searchFracCandidate(tu.cu, m_pcIntraPred->getTargetPatch(floorLog2(std::max(pu.lwidth(), pu.lheight())) - 2), tempType);
#endif
          }
        }
#endif		

#if JVET_AG0136_INTRA_TMP_LIC
        if (tu.cu->tmpLicFlag)
        {
          PelBuf bufDumb;
          if ((tu.cu)->tmpFusionFlag)
          {
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            const int tmpFusionIdx = getTmpFusionIdx((tu.cu)->tmpIdx);
            for (int i = 0; i < getTmpFusionNumber(tmpFusionIdx
#if JVET_AM0138_ENHANCED_TMP_MERGE_LIST_TIMD_BV
                                                   , false
#endif
                                                   ); i++)
#else
            IntraTMPFusionInfo infoIntermediate[TMP_GROUP_IDX << 1] = {
              { true, false, 0, TMP_FUSION_NUM },
              { true, false, TMP_FUSION_NUM, TMP_FUSION_NUM },
              { true, false, TMP_FUSION_NUM << 1, TMP_FUSION_NUM },
              { true, true, 0, TMP_FUSION_NUM },
              { true, true, TMP_FUSION_NUM, TMP_FUSION_NUM },
              { true, true, TMP_FUSION_NUM << 1, TMP_FUSION_NUM }
            };
            infoIntermediate[(tu.cu)->tmpIdx].tmpFusionNumber = m_pcIntraPred->xCalTMPFusionNumber(infoIntermediate[(tu.cu)->tmpIdx].tmpMaxNum, 0, true);
            for (int i = 0; i < infoIntermediate[(tu.cu)->tmpIdx].tmpFusionNumber; i++)
#endif
            {
              m_pcIntraPred->setBvMvFromMemory(*(tu.cu)
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                               , i + tmpFusionIdx
#else
                                               , i + infoIntermediate[(tu.cu)->tmpIdx].tmpFusionIdx
#endif
                                               , true);
              m_pcInterPred->LicItmp(pu, bufDumb
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                     , m_pcIntraPred->getRequiredTemplate(true, ((tu.cu)->slice)->getSPS()->getItmpLicExtension(), i + tmpFusionIdx)
#else
                                     , false
#endif
                                     );
              m_pcIntraPred->getMemLicParams((tu.cu)->ibcLicIdx,
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                             i + tmpFusionIdx
#else
                                             i + infoIntermediate[(tu.cu)->tmpIdx].tmpFusionIdx
#endif
                                             ) = m_pcInterPred->getArrayLicParams();
            }
          }
          else
          {
#if !JVET_AH0200_INTRA_TMP_BV_REORDER || JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            m_pcIntraPred->setBvMvFromMemory(*(tu.cu), (tu.cu)->tmpIdx, true);
            m_pcInterPred->LicItmp(pu, bufDumb
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                   , m_pcIntraPred->getRequiredTemplate(true, ((tu.cu)->slice)->getSPS()->getItmpLicExtension(), (tu.cu)->tmpIdx)
#else
                                   , false
#endif
                                   );
            m_pcIntraPred->getMemLicParams((tu.cu)->ibcLicIdx, (tu.cu)->tmpIdx) = m_pcInterPred->getArrayLicParams();
#endif
          }
        }
#endif
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
        if (!tu.cu->tmpFlmFlag && !tu.cu->tmpFusionFlag)
        {
          m_pcIntraPred->searchFracCandidate(tu.cu, m_pcIntraPred->getTargetPatch(), (tu.cu)->tempType, false);
        }
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
        if (tu.cu->tmpFlmFlag)
        {
          m_pcIntraPred->xCalTmpFlmParam(tu.cu, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                         , (tu.cu)->tempType
#else
                                         , tempType
#endif
                                         );
        }
        if (tu.cu->tmpFusionFlag)
        {
          m_pcIntraPred->xTMPBuildFusionCandidate(*tu.cu
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                                  , (tu.cu)->tempType
#else
                                                  , tempType
#endif
#if JVET_AG0136_INTRA_TMP_LIC
                                                  , (tu.cu)->tmpLicFlag
#endif
                                                  );
        }
#if !JVET_AH0200_INTRA_TMP_BV_REORDER
        if (pu.cu->tmpIsSubPel)
        {
          m_pcIntraPred->xPadForInterpolation(pu.cu);
        }
#endif
#endif
#if JVET_AB0061_ITMP_BV_FOR_IBC
        m_pcIntraPred->generateTMPrediction(piPred.buf, piPred.stride, foundCandiNum, pu
#if JVET_AG0136_INTRA_TMP_LIC
                                            , (tu.cu)->tmpLicFlag
                                            , !(tu.cu)->tmpFlmFlag
#endif
                                            );
#elif TMP_FAST_ENC
        m_pcIntraPred->generateTMPrediction( piPred.buf, piPred.stride, pu.Y(), foundCandiNum, pu.cu );
#else
        m_pcIntraPred->generateTMPrediction(piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum);
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AG0136_INTRA_TMP_LIC
        if ((tu.cu)->tmpLicFlag)
        {
          if (!(tu.cu)->tmpFusionFlag)
          {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            if ((tu.cu)->tmpIsSubPel)
            {
#endif
            PelBuf bufDumb;
            m_pcInterPred->LicItmp(pu, bufDumb
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                   , m_pcIntraPred->getRequiredTemplate(true, ((tu.cu)->slice)->getSPS()->getItmpLicExtension(), (tu.cu)->tmpIdx)
#else
                                   , false
#endif
                                   );
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
            }
            const auto& arrayLicParams = (tu.cu)->tmpIsSubPel ? m_pcInterPred->getArrayLicParams() : m_pcIntraPred->getMemLicParams((tu.cu)->ibcLicIdx, (tu.cu)->tmpIdx);
#else
            const auto& arrayLicParams = m_pcInterPred->getArrayLicParams();
#endif
#else
            const auto& arrayLicParams = m_pcIntraPred->getMemLicParams((tu.cu)->ibcLicIdx, (tu.cu)->tmpIdx);
#endif
            if ((tu.cu)->ibcLicIdx == IBC_LIC_IDX_M)
            {
              piPred.linearTransforms(arrayLicParams[1], arrayLicParams[0], arrayLicParams[2], arrayLicParams[4], arrayLicParams[3], arrayLicParams[5], arrayLicParams[6], true, (tu.cu)->cs->slice->clpRng(COMPONENT_Y));
#if JVET_AM0157_SGPM_BV_LIC
              pu.cu->ibcLicMean[0][0] = arrayLicParams[6];
              pu.cu->ibcLicScale[0][0][COMPONENT_Y] = arrayLicParams[1];
              pu.cu->ibcLicShift[0][0][COMPONENT_Y] = arrayLicParams[0];
              pu.cu->ibcLicOffset[0][0][COMPONENT_Y] = arrayLicParams[2];
              pu.cu->ibcLicScale[0][1][COMPONENT_Y] = arrayLicParams[4];
              pu.cu->ibcLicShift[0][1][COMPONENT_Y] = arrayLicParams[3];
              pu.cu->ibcLicOffset[0][1][COMPONENT_Y] = arrayLicParams[5];
#endif
            }
            else
            {
              piPred.linearTransform(arrayLicParams[1], arrayLicParams[0], arrayLicParams[2], true, (tu.cu)->cs->slice->clpRng(COMPONENT_Y));
#if JVET_AK0076_EXTENDED_OBMC_IBC
              pu.cu->licScale[0][COMPONENT_Y] = arrayLicParams[1];
              pu.cu->licOffset[0][COMPONENT_Y] = arrayLicParams[2];
#endif
            }
          }
        }
#endif
        m_pcIntraPred->xGenerateTmpFlmPred(piPred, pu.lwidth(), pu.lheight()
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                           , tempType
#endif
                                           , tu.cu);
        m_pcIntraPred->xTMPFusionApplyModel(piPred, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                            , (tu.cu)->tempType
#else
                                            , tempType
#endif
                                            , tu.cu
#if JVET_AG0136_INTRA_TMP_LIC
                                            , (tu.cu)->tmpLicFlag
#endif
                                            );
#endif
		  }
		  else
		  {
			  foundCandiNum = 1;
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
        m_pcIntraPred->generateTmDcPrediction(piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), 1 << (tu.cu->cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1), pu.cu);
#else
        m_pcIntraPred->generateTmDcPrediction(piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), 1 << (tu.cu->cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1));
#endif

#if JVET_AB0061_ITMP_BV_FOR_IBC
        pu.interDir               = 1;             // use list 0 for IBC mode
        pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;   // last idx in the list
        pu.mv->set(0, 0);
        pu.bv.set(0, 0);
#endif
		  }
#else
      m_pcIntraPred->getTargetTemplate(tu.cu, pu.lwidth(), pu.lheight());
      m_pcIntraPred->candidateSearchIntra(tu.cu, pu.lwidth(), pu.lheight());
#if TMP_FAST_ENC
      m_pcIntraPred->generateTMPrediction(piPred.buf, piPred.stride, pu.Y(), foundCandiNum, pu.cu);
#else
      m_pcIntraPred->generateTMPrediction(piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum);
#endif
#endif
#if JVET_AK0076_EXTENDED_OBMC_IBC
      PU::spanMotionInfo(pu);
      if (pu.cu->obmcFlag)
      {
        pu.cu->isobmcMC = true;
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
        m_pcInterPred->subBlockOBMC(pu, nullptr, m_pcIntraPred, true);
#else
        m_pcInterPred->subBlockOBMC(pu, nullptr, true);
#endif
        pu.cu->isobmcMC = false;
      }
#endif
      CHECK( foundCandiNum < 1, "Wrong candidate number");
	  }
	  else if (PU::isMIP(pu, chType))
#else
    if( PU::isMIP( pu, chType ) )
#endif
    {
      m_pcIntraPred->initIntraMip( pu, area );
#if JVET_AB0067_MIP_DIMD_LFNST
#if JVET_AH0076_OBIC
      m_pcIntraPred->predIntraMip( compID, piPred, pu, true);
#else
      m_pcIntraPred->predIntraMip( compID, piPred, pu, pu.cu->lfnstIdx > 0 ? true : false);
#endif
#else
      m_pcIntraPred->predIntraMip( compID, piPred, pu );
#endif
    }
#if JVET_AG0058_EIP
    else if (PU::isEIP(pu, chType))
    {
      const bool isEncoder = pu.cs->pcv->isEncoder;
      m_pcIntraPred->initEipParams(pu, COMPONENT_Y);
      if (!isEncoder)
      {
        if (pu.cu->eipMerge)
        {
          static_vector<EipModelCandidate, MAX_MERGE_EIP> eipMergeCandList;
          m_pcIntraPred->getNeiEipCands(pu, eipMergeCandList);
          m_pcIntraPred->reorderEipCands(pu, eipMergeCandList);
          pu.cu->eipModel = eipMergeCandList[pu.intraDir[CHANNEL_TYPE_LUMA]];
        }
#if JVET_AL0106_BV_EIP
        else if (pu.cu->bvEip)
        {
          static_vector<EipModelCandidate, MAX_BV_EIP> eipBvCandList;
#if !JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
          RefTemplateType tempType = m_pcIntraPred->getRefTemplateType(*(tu.cu), tu.cu->blocks[COMPONENT_Y]);
#endif
          m_pcIntraPred->getTargetTemplate(tu.cu, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                           , (tu.cu)->tempType
#else
                                           , tempType
#endif
                                           );
          m_pcIntraPred->candidateSearchIntra(tu.cu, pu.lwidth(), pu.lheight()
#if JVET_AM0229_INTRATMP_SUBMODES_DEPENDING
                                              , (tu.cu)->tempType
#else
                                              , tempType
#endif
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
            , false
#endif
          );

          m_pcIntraPred->initBvEipParams(pu);
          m_pcIntraPred->getBvEipCands(pu, eipBvCandList);
          m_pcIntraPred->setEipBvInfo(pu);
          pu.cu->eipModel = eipBvCandList[0];
        }
#endif
        else 
        {
          static_vector<EipModelCandidate, NUM_DERIVED_EIP> eipModelCandList;
          m_pcIntraPred->getCurEipCands(pu, eipModelCandList);
          pu.cu->eipModel = eipModelCandList[0];
        }
      }
      m_pcIntraPred->eipPred(pu, piPred);
      if (!isEncoder)
      {
#if JVET_AI0050_INTER_MTSS
        int secondDimdIntraDir = 0;
#endif
        pu.cu->eipModel.eipDimdMode = IntraPrediction::deriveIpmForTransform(piPred, *pu.cu
#if JVET_AI0050_INTER_MTSS
          , secondDimdIntraDir
#endif
        );
#if JVET_AI0050_INTER_MTSS
        pu.cu->dimdDerivedIntraDir2nd = secondDimdIntraDir;
#endif
#if JVET_AK0217_INTRA_MTSS || JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
        pu.cu->eipModel.eipDimdMode2nd = secondDimdIntraDir;
#endif
      }
    }
#endif
    else
    {
      if (predRegDiffFromTB)
      {
        if (firstTBInPredReg)
        {
          PelBuf piPredReg = cs.getPredBuf(areaPredReg);
          m_pcIntraPred->predIntraAng(compID, piPredReg, pu);
        }
      }
      else
#if JVET_AC0071_DBV
#if JVET_AH0136_CHROMA_REORDERING
        if (compID != COMPONENT_Y && PU::isDbvMode(uiChFinalMode))
#else
        if (compID != COMPONENT_Y && uiChFinalMode == DBV_CHROMA_IDX)
#endif
        {
          m_pcIntraPred->predIntraDbv(compID, piPred, pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                  , m_pcInterPred
#endif
        );
      }
      else
#endif
        m_pcIntraPred->predIntraAng(compID, piPred, pu);

#if JVET_Z0050_DIMD_CHROMA_FUSION
      if (compID != COMPONENT_Y && pu.isChromaFusion)
      {
        m_pcIntraPred->geneChromaFusionPred(compID, piPred, pu
#if JVET_AH0136_CHROMA_REORDERING
          , m_pcInterPred
#endif
        );
      }
#endif
#if JVET_AJ0112_REGRESSION_SGPM
      if (compID == COMPONENT_Y && pu.cu->sgpm && PU::isRegressionSgpm(pu))
      {
#if JVET_AI0050_INTER_MTSS
        int secondDimdIntraDir = 0;
#endif
#if JVET_AK0217_INTRA_MTSS || JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
        int firstDimdIntraDir =
#endif
        m_pcIntraPred->IntraPrediction::deriveIpmForTransform(piPred, *pu.cu
#if JVET_AI0050_INTER_MTSS
          , secondDimdIntraDir
#endif
        );
#if JVET_AK0217_INTRA_MTSS || JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
        pu.cu->dimdDerivedIntraDir    = firstDimdIntraDir;
        pu.cu->dimdDerivedIntraDir2nd = secondDimdIntraDir;
#endif
      }
#endif
    }
  }
#if SIGN_PREDICTION
#if JVET_AK0064_CCP_LFNST_NSPT
  if (isJCCR && compID == COMPONENT_Cb && !PU::isLMCMode(uiChFinalMode))
#if JVET_AA0057_CCCM
#if JVET_AD0188_CCP_MERGE
  if (isJCCR && compID == COMPONENT_Cb && !pu.cccmFlag && !pu.idxNonLocalCCP
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION	  
    && !pu.decoderDerivedCcpMode
#endif
    )   // Cr prediction was done already for CCCM
#else
  if(isJCCR && compID == COMPONENT_Cb && !pu.cccmFlag
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION	  
    && !pu.decoderDerivedCcpMode
#endif
    ) // Cr prediction was done already for CCCM
#endif
#else
  if(isJCCR && compID == COMPONENT_Cb
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION	  
    && !pu.decoderDerivedCcpMode
#endif
    )
#endif
#endif
  {
    m_pcIntraPred->initIntraPatternChType(*tu.cu, areaCr);
    if( PU::isLMCMode( uiChFinalMode ) )
    {
#if JVET_AD0188_CCP_MERGE
      PredictionUnit& pu = *tu.cu->firstPU;
#else
      const PredictionUnit& pu = *tu.cu->firstPU;
#endif
      m_pcIntraPred->xGetLumaRecPixels( pu, areaCr );
      m_pcIntraPred->predIntraChromaLM( COMPONENT_Cr, piPredCr, pu, areaCr, uiChFinalMode );
    }
    else
    {
      if( PU::isMIP( pu, chType ) )
      {
        m_pcIntraPred->initIntraMip( pu, area );
        m_pcIntraPred->predIntraMip( COMPONENT_Cr, piPredCr, pu );
      }
      else
      {
#if JVET_AC0071_DBV
#if JVET_AH0136_CHROMA_REORDERING
        if (PU::isDbvMode(uiChFinalMode))
#else
        if (uiChFinalMode == DBV_CHROMA_IDX)
#endif
        {
          m_pcIntraPred->predIntraDbv(COMPONENT_Cr, piPredCr, pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                    , m_pcInterPred
#endif
          );
        }
        else
#endif
        m_pcIntraPred->predIntraAng(COMPONENT_Cr, piPredCr, pu);

#if JVET_Z0050_DIMD_CHROMA_FUSION
        if (pu.isChromaFusion)
        {
          m_pcIntraPred->geneChromaFusionPred(COMPONENT_Cr, piPredCr, pu
#if JVET_AH0136_CHROMA_REORDERING
            , m_pcInterPred
#endif
          );
        }
#endif
      }
    }
  }
  }
#endif
#if JVET_AK0064_CCP_LFNST_NSPT
  if (compID != COMPONENT_Y)
  {
    if (compID == COMPONENT_Cb && PU::isLMCMode(uiChFinalMode) && tu.cu->lfnstIdx)
    {
      PelBuf predCr = cs.getPredBuf(tu.blocks[COMPONENT_Cr]);
      IntraPrediction::deriveChromaIpmForTransform(piPred, predCr, *pu.cu);
    }
  }
#endif
  const Slice           &slice = *cs.slice;
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
  if (flag && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (compID != COMPONENT_Y) && (tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr]))
  {
#if LMCS_CHROMA_CALC_CU
    const Area area = tu.cu->Y().valid() ? tu.cu->Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].size()));
#else
    const Area area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
#endif
    const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
    int adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
    tu.setChromaAdj(adj);
  }
#if SIGN_PREDICTION
  flag = flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
#endif
  //===== inverse transform =====
  PelBuf piResi = cs.getResiBuf( area );

  const QpParam cQP( tu, compID );
#if JVET_AK0217_INTRA_MTSS || JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
  if (chType == CHANNEL_TYPE_LUMA && !tu.cu->ispMode && !tu.cu->lfnstIdx && !tu.mtsIdx[0] && tu.cs->sps->getUseImplicitMTS())
  {
#if JVET_AK0217_INTRA_MTSS
    bool secondBucket = false;
    tu.intraDirStat = PU::getFinalIntraModeForTransform(secondBucket, tu, COMPONENT_Y);
#else
    tu.intraDirStat = PU::getFinalIntraModeForTransform(tu, COMPONENT_Y);
#endif
  }
#endif
#if SIGN_PREDICTION
  bool bJccrWithCr = tu.jointCbCr && !(tu.jointCbCr >> 1);
  bool bIsJccr     = tu.jointCbCr && isChroma(compID);
  ComponentID signPredCompID = bIsJccr ? (bJccrWithCr ? COMPONENT_Cr : COMPONENT_Cb): compID;
  bool reshapeChroma = flag && (TU::getCbf(tu, signPredCompID) || tu.jointCbCr) && isChroma(signPredCompID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag();
  m_pcTrQuant->predCoeffSigns(tu, compID, reshapeChroma);
#endif
  if( tu.jointCbCr && isChroma(compID) )
  {
    if( compID == COMPONENT_Cb )
    {
      PelBuf resiCr = cs.getResiBuf( tu.blocks[ COMPONENT_Cr ] );
      if( tu.jointCbCr >> 1 )
      {
        m_pcTrQuant->invTransformNxN( tu, COMPONENT_Cb, piResi, cQP );
      }
      else
      {
        const QpParam qpCr( tu, COMPONENT_Cr );
        m_pcTrQuant->invTransformNxN( tu, COMPONENT_Cr, resiCr, qpCr );
      }
      m_pcTrQuant->invTransformICT( tu, piResi, resiCr );
    }
  }
  else
  if( TU::getCbf( tu, compID ) )
  {
    m_pcTrQuant->invTransformNxN( tu, compID, piResi, cQP );
  }
  else
  {
    piResi.fill( 0 );
  }

  //===== reconstruction =====
#if !SIGN_PREDICTION
  flag = flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
#endif
  if (flag && (TU::getCbf(tu, compID) || tu.jointCbCr) && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
  }

  if( !tu.cu->ispMode || !isLuma( compID ) )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0071_DBV
#if JVET_AH0136_CHROMA_REORDERING
    if (!(PU::isDbvMode(uiChFinalMode) && compID == COMPONENT_Cb))
#else
    if (!(uiChFinalMode == DBV_CHROMA_IDX && compID == COMPONENT_Cb))
#endif
#endif
    cs.setDecomp( area );
  }
  else if( tu.cu->ispMode && isLuma( compID ) && CU::isISPFirst( *tu.cu, tu.blocks[compID], compID ) )
  {
    cs.setDecomp( tu.cu->blocks[compID] );
  }

#if REUSE_CU_RESULTS
  CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
  PelBuf tmpPred;
#endif
  if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
  {
#if REUSE_CU_RESULTS
    {
      tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
      tmpPred.copyFrom(piPred);
    }
#endif
  }
#if JVET_AG0145_ADAPTIVE_CLIPPING
  ClpRng clpRng = tu.cu->cs->slice->clpRng(compID);
  if (compID == COMPONENT_Y)
  {
    if (pu.cu->cs->sps->getUseLmcs() && pu.cu->cs->picHeader->getLmcsEnabledFlag())
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
  }
#if NNVC_USE_PRED
  piPredCustom.copyFrom( piPred );
#endif 
#if KEEP_PRED_AND_RESI_SIGNALS || NNVC_USE_PRED
  pReco.reconstruct(piPred, piResi, clpRng);
#else
  piPred.reconstruct(piPred, piResi, clpRng);
  pReco.copyFrom( piPred );
#if NN_LF_UNIFIED
  piPred.copyFrom(piPredCustom);
#endif
#endif
#else
#if KEEP_PRED_AND_RESI_SIGNALS || NNVC_USE_PRED
  pReco.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
#else
  piPred.reconstruct( piPred, piResi, tu.cu->cs->slice->clpRng( compID ) );
  pReco.copyFrom( piPred );
#if NN_LF_UNIFIED
  piPred.copyFrom(piPredCustom);
#endif
#endif
#endif

#if JVET_AC0071_DBV && JVET_AA0070_RRIBC
#if JVET_AH0136_CHROMA_REORDERING
  if (compID != COMPONENT_Y && PU::isDbvMode(uiChFinalMode) && tu.cu->rribcFlipType)
#else
  if (compID != COMPONENT_Y && uiChFinalMode == DBV_CHROMA_IDX && tu.cu->rribcFlipType)
#endif
  {
    pReco.flipSignal(tu.cu->rribcFlipType == 1);
  }
#endif
  if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
  {
#if REUSE_CU_RESULTS
    {
      piPred.copyFrom(tmpPred);
    }
#endif
  }
#if REUSE_CU_RESULTS || SIGN_PREDICTION
#if !SIGN_PREDICTION
  if( cs.pcv->isEncoder )
#endif
  {
#if JVET_Z0118_GDR
    cs.updateReconMotIPM(area, pReco);
#else
    cs.picture->getRecoBuf( area ).copyFrom( pReco );
#endif

    cs.picture->getPredBuf(area).copyFrom(piPred);
#if NNVC_USE_PRED
    if (cs.pcv->isEncoder)
    {
      piPredCustom.copyFrom(piPred);
    }
#endif 
  }
#endif
}

void DecCu::xIntraRecACTBlk(TransformUnit& tu)
{
  CodingStructure      &cs = *tu.cs;
  const PredictionUnit &pu = *tu.cs->getPU(tu.blocks[COMPONENT_Y], CHANNEL_TYPE_LUMA);
  const Slice          &slice = *cs.slice;

  CHECK(!tu.Y().valid() || !tu.Cb().valid() || !tu.Cr().valid(), "Invalid TU");
  CHECK(&pu != tu.cu->firstPU, "wrong PU fetch");
  CHECK(tu.cu->ispMode, "adaptive color transform cannot be applied to ISP");
  CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");

  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#if JVET_S0234_ACT_CRS_FIX
  if (flag && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
#else
  if (flag && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr]))
#endif
  {
#if LMCS_CHROMA_CALC_CU
    const Area      area = tu.cu->Y().valid() ? tu.cu->Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].size()));
#else
    const Area      area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
#endif
    const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
    int            adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
    tu.setChromaAdj(adj);
  }

  for (int i = 0; i < getNumberValidComponents(tu.chromaFormat); i++)
  {
    ComponentID          compID = (ComponentID)i;
    const CompArea       &area = tu.blocks[compID];
    const ChannelType    chType = toChannelType(compID);

    PelBuf piPred = cs.getPredBuf(area);
    m_pcIntraPred->initIntraPatternChType(*tu.cu, area);
#if JVET_V0130_INTRA_TMP && ! JVET_W0069_TMP_BOUNDARY
	if (PU::isTmp(pu, chType))
	{
		int foundCandiNum;
		const unsigned int uiStride = cs.picture->getRecoBuf(COMPONENT_Y).stride;
    m_pcIntraPred->getTargetTemplate(tu.cu, pu.lwidth(), pu.lheight());
    m_pcIntraPred->candidateSearchIntra(tu.cu, pu.lwidth(), pu.lheight());
    m_pcIntraPred->generateTMPrediction(piPred.buf, uiStride, pu.lwidth(), pu.lheight(), foundCandiNum);
	}
	else if (PU::isMIP(pu, chType))
#else
    if (PU::isMIP(pu, chType))
#endif
    {
      m_pcIntraPred->initIntraMip(pu, area);
#if JVET_AB0067_MIP_DIMD_LFNST
#if JVET_AH0076_OBIC
      m_pcIntraPred->predIntraMip(compID, piPred, pu, true);
#else
      m_pcIntraPred->predIntraMip(compID, piPred, pu, pu.cu->lfnstIdx > 0 ? true : false);
#endif
#else
      m_pcIntraPred->predIntraMip(compID, piPred, pu);
#endif
    }
    else
    {
      m_pcIntraPred->predIntraAng(compID, piPred, pu);
    }

    PelBuf piResi = cs.getResiBuf(area);

    QpParam cQP(tu, compID);

    if (tu.jointCbCr && isChroma(compID))
    {
      if (compID == COMPONENT_Cb)
      {
        PelBuf resiCr = cs.getResiBuf(tu.blocks[COMPONENT_Cr]);
        if (tu.jointCbCr >> 1)
        {
          m_pcTrQuant->invTransformNxN(tu, COMPONENT_Cb, piResi, cQP);
        }
        else
        {
          QpParam qpCr(tu, COMPONENT_Cr);

          m_pcTrQuant->invTransformNxN(tu, COMPONENT_Cr, resiCr, qpCr);
        }
        m_pcTrQuant->invTransformICT(tu, piResi, resiCr);
      }
    }
    else
    {
      if (TU::getCbf(tu, compID))
      {
        m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
      }
      else
      {
        piResi.fill(0);
      }
    }

#if !JVET_S0234_ACT_CRS_FIX
    flag = flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
    if (flag && (TU::getCbf(tu, compID) || tu.jointCbCr) && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
    {
      piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
    }
#endif

    cs.setDecomp(area);
  }

  cs.getResiBuf(tu).colorSpaceConvert(cs.getResiBuf(tu), false, tu.cu->cs->slice->clpRng(COMPONENT_Y));

  for (int i = 0; i < getNumberValidComponents(tu.chromaFormat); i++)
  {
    ComponentID          compID = (ComponentID)i;
    const CompArea       &area = tu.blocks[compID];

    PelBuf piPred = cs.getPredBuf(area);
    PelBuf piResi = cs.getResiBuf(area);
    PelBuf piReco = cs.getRecoBuf(area);
#if NNVC_USE_PRED
    PelBuf piPredCustom = cs.getPredBufCustom(area);
#endif

    PelBuf tmpPred;
    if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
    {
      CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
      tmpPred.copyFrom(piPred);
    }

#if JVET_S0234_ACT_CRS_FIX
    if (flag && isChroma(compID) && (tu.blocks[compID].width*tu.blocks[compID].height > 4) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
    {
      piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
    }
#endif
#if NNVC_USE_PRED
    piPredCustom.copyFrom(piPred);
#endif 
    piPred.reconstruct(piPred, piResi, tu.cu->cs->slice->clpRng(compID));
    piReco.copyFrom(piPred);
#if NN_LF_UNIFIED
    piPred.copyFrom(piPredCustom);
#endif

    if (slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || slice.isIntra()) && compID == COMPONENT_Y)
    {
      piPred.copyFrom(tmpPred);
    }

    if (cs.pcv->isEncoder)
    {
#if JVET_Z0118_GDR
      cs.updateReconMotIPM(area, piReco);
#else
      cs.picture->getRecoBuf(area).copyFrom(piReco);
#endif

      cs.picture->getPredBuf(area).copyFrom(piPred);
#if NNVC_USE_PRED 
      if (cs.pcv->isEncoder)
      {
        piPredCustom.copyFrom(piPred);
      }
#endif
    }
  }
}

void DecCu::xReconIntraQT( CodingUnit &cu )
{

  if (CU::isPLT(cu))
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs))
#else
    if (cu.isSepTree())
#endif
    {
      if (cu.chType == CHANNEL_TYPE_LUMA)
      {
        xReconPLT(cu, COMPONENT_Y, 1);
      }
      if (cu.chromaFormat != CHROMA_400 && (cu.chType == CHANNEL_TYPE_CHROMA))
      {
        xReconPLT(cu, COMPONENT_Cb, 2);
      }
    }
    else
    {
      if( cu.chromaFormat != CHROMA_400 )
      {
        xReconPLT(cu, COMPONENT_Y, 3);
      }
      else
      {
        xReconPLT(cu, COMPONENT_Y, 1);
      }
    }

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    if (cu.blocks[CHANNEL_TYPE_LUMA].valid())
    {
      PU::spanSCCInfo(*cu.firstPU);
    }
#endif
    return;
  }

  if (cu.colorTransform)
  {
    xIntraRecACTQT(cu);
  }
  else
  {
    const uint32_t numChType = ::getNumberValidChannels(cu.chromaFormat);

    for (uint32_t chType = CHANNEL_TYPE_LUMA; chType < numChType; chType++)
    {
      if (cu.blocks[chType].valid())
      {
        xIntraRecQT(cu, ChannelType(chType));
      }
    }
  }
#if JVET_W0123_TIMD_FUSION
#if JVET_AK0076_EXTENDED_OBMC_IBC
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid() && !cu.tmpFlag)
#else
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid())
#endif
  {
    PU::spanIpmInfoIntra(*cu.firstPU);
  }
#endif
#if JVET_AB0061_ITMP_BV_FOR_IBC
#if !JVET_AK0076_EXTENDED_OBMC_IBC
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid() && cu.tmpFlag)
  {
    PU::spanMotionInfo(*cu.firstPU);
  }
#endif
#endif
#if JVET_AL0108_BVG_DIMD
#if JVET_AL0106_BV_EIP
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid() && (PU::hasBvgBv(*cu.firstPU) || PU::isBvEip(*cu.firstPU)))
#else
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid() && PU::hasBvgBv(*cu.firstPU))
#endif
  {
    PU::spanMotionInfo(*cu.firstPU);
  }
#endif

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  if (cu.blocks[CHANNEL_TYPE_LUMA].valid() && !cu.tmpFlag)
  {
     PU::spanSCCInfo(*cu.firstPU);
  }
#endif
}

void DecCu::xReconPLT(CodingUnit &cu, ComponentID compBegin, uint32_t numComp)
{
  const SPS&       sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  PelBuf    curPLTIdx = tu.getcurPLTIdx(compBegin);

  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  //recon. pixels
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
      {
        const int  channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)compID));
        const CompArea &area = cu.blocks[compID];

        PelBuf       picReco   = cu.cs->getRecoBuf(area);
        PLTescapeBuf escapeValue = tu.getescapeValue((ComponentID)compID);
        if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
        {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
          TCoeff value;
#else
          Pel value;
#endif
          QpParam cQP(tu, (ComponentID)compID);
          int qp = cQP.Qp(true);
          int qpRem = qp % 6;
          int qpPer = qp / 6;
          if (compBegin != COMPONENT_Y || compID == COMPONENT_Y)
          {
            int invquantiserRightShift = IQUANT_SHIFT;
            int add = 1 << (invquantiserRightShift - 1);
            value = ((((escapeValue.at(x, y)*g_invQuantScales[0][qpRem]) << qpPer) + add) >> invquantiserRightShift);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            value = ClipBD<TCoeff>(value, channelBitDepth);
            picReco.at(x, y) = Pel(value);
#else
            value = Pel(ClipBD<int>(value, channelBitDepth));
            picReco.at(x, y) = value;
#endif
          }
          else if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            uint32_t posYC = y >> scaleY;
            uint32_t posXC = x >> scaleX;
            int invquantiserRightShift = IQUANT_SHIFT;
            int add = 1 << (invquantiserRightShift - 1);
            value = ((((escapeValue.at(posXC, posYC)*g_invQuantScales[0][qpRem]) << qpPer) + add) >> invquantiserRightShift);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            value = ClipBD<TCoeff>(value, channelBitDepth);
            picReco.at(posXC, posYC) = Pel(value);
#else
            value = Pel(ClipBD<int>(value, channelBitDepth));
            picReco.at(posXC, posYC) = value;
#endif

          }
        }
        else
        {
          uint32_t curIdx = curPLTIdx.at(x, y);
          if (compBegin != COMPONENT_Y || compID == COMPONENT_Y)
          {
            picReco.at(x, y) = cu.curPLT[compID][curIdx];
          }
          else if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            uint32_t posYC = y >> scaleY;
            uint32_t posXC = x >> scaleX;
            picReco.at(posXC, posYC) = cu.curPLT[compID][curIdx];
          }
        }
      }
    }
  }
  for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
  {
    const CompArea &area = cu.blocks[compID];
    PelBuf picReco = cu.cs->getRecoBuf(area);
#if JVET_Z0118_GDR
    cu.cs->updateReconMotIPM(area, picReco);
#else
    cu.cs->picture->getRecoBuf(area).copyFrom(picReco);
#endif
#if NNVC_USE_PRED
    cu.cs->getPredBuf(cu).bufs[compID].copyFrom(picReco);
#endif
    cu.cs->setDecomp(area);
  }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  cu.cs->getResiBuf(cu).bufs[0].fill(0);
#endif
#if NNVC_USE_PRED
  cu.cs->getPredBufCustom(cu).copyFrom(cu.cs->getPredBuf(cu));
#endif
}

/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
* \param pcRecoYuv pointer to reconstructed sample arrays
* \param pcPredYuv pointer to prediction sample arrays
* \param pcResiYuv pointer to residue sample arrays
* \param chType    texture channel type (luma/chroma)
* \param rTu       reference to transform data
*
\ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
*/

void
DecCu::xIntraRecQT(CodingUnit &cu, const ChannelType chType)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    if( isLuma( chType ) )
    {
      xIntraRecBlk( currTU, COMPONENT_Y );
    }
    else
    {
      const uint32_t numValidComp = getNumberValidComponents( cu.chromaFormat );

      for( uint32_t compID = COMPONENT_Cb; compID < numValidComp; compID++ )
      {
        xIntraRecBlk( currTU, ComponentID( compID ) );
      }
    }
  }
}

void DecCu::xIntraRecACTQT(CodingUnit &cu)
{
  for (auto &currTU : CU::traverseTUs(cu))
  {
    xIntraRecACTBlk(currTU);
  }
}

#if !REMOVE_PCM
/** Function for filling the PCM buffer of a CU using its reconstructed sample array
* \param pCU   pointer to current CU
* \param depth CU Depth
*/
void DecCu::xFillPCMBuffer(CodingUnit &cu)
{
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    for (const CompArea &area : currTU.blocks)
    {
      if( !area.valid() ) continue;;

      CPelBuf source      = cu.cs->getRecoBuf(area);
       PelBuf destination = currTU.getPcmbuf(area.compID);

      destination.copyFrom(source);
    }
  }
}
#endif

#include "CommonLib/dtrace_buffer.h"

void DecCu::xReconInter(CodingUnit &cu)
{
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  if ( cu.geoBlendFlag )
  {
    m_pcInterPred->motionCompensationGeoBlend( cu, m_geoMrgCtx
  #if JVET_AE0046_BI_GPM
      , m_mvBufBDMVR
      , m_mvBufBDOF4GPM
  #endif
#if JVET_AK0101_REGRESSION_GPM_INTRA || JVET_AK0212_GPM_OBMC_MODIFICATION
      , m_pcIntraPred
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
      , (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr
#endif
    );
  }
  else
#endif
  if( cu.geoFlag )
  {
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    m_pcInterPred->motionCompensationGeo( cu, m_geoMrgCtx 
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
                                        , m_geoTmMrgCtx
#endif
#if JVET_AG0164_AFFINE_GPM
                                        , m_geoAffMrgCtx
#if JVET_AJ0274_GPM_AFFINE_TM
                                        , m_geoAffTmMrgCtx
#endif
#endif
#if JVET_AE0046_BI_GPM
                                        , m_mvBufBDMVR
                                        , m_mvBufBDOF4GPM
#endif
#if JVET_Y0065_GPM_INTRA
                                        , m_pcIntraPred, (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
                                        , m_geoBvList
#endif
    );
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
#if JVET_AJ0107_GPM_SHAPE_ADAPT
    int whIdx = !cu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(cu.firstPU->lwidth()) - floorLog2(cu.firstPU->lheight()) + GEO_SQUARE_IDX);
    MergeCtx& m_geoTmMrgCtx0 = m_geoTmMrgCtx[g_geoTmShape[0][g_geoParams[g_gpmSplitDir[whIdx][cu.firstPU->geoSplitDir]][0]]];
    MergeCtx& m_geoTmMrgCtx1 = m_geoTmMrgCtx[g_geoTmShape[1][g_geoParams[g_gpmSplitDir[whIdx][cu.firstPU->geoSplitDir]][0]]];
#else
    MergeCtx& m_geoTmMrgCtx0 = m_geoTmMrgCtx[g_geoTmShape[0][g_geoParams[cu.firstPU->geoSplitDir][0]]];
    MergeCtx& m_geoTmMrgCtx1 = m_geoTmMrgCtx[g_geoTmShape[1][g_geoParams[cu.firstPU->geoSplitDir][0]]];
#endif
#endif
#if JVET_W0097_GPM_MMVD_TM
    PU::spanGeoMMVDMotionInfo
#else
	  PU::spanGeoMotionInfo
#endif
                             ( *cu.firstPU, m_geoMrgCtx
#if JVET_AG0164_AFFINE_GPM
                               , m_geoAffMrgCtx
#if JVET_AJ0274_GPM_AFFINE_TM
                               , m_geoAffTmMrgCtx
#endif
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
							                , m_geoTmMrgCtx0, m_geoTmMrgCtx1
#endif
							                , cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
                              , cu.firstPU->geoTmFlag0
#endif
							                , cu.firstPU->geoMMVDFlag0, cu.firstPU->geoMMVDIdx0
#if TM_MRG							 
							                , cu.firstPU->geoTmFlag1
#endif
							                , cu.firstPU->geoMMVDFlag1, cu.firstPU->geoMMVDIdx1
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
                              , cu.firstPU->geoBldIdx
#endif
                              , m_pcIntraPred->m_intraMPM
#if JVET_AI0082_GPM_WITH_INTER_IBC
                              , m_geoBvList
#endif
#if JVET_AE0046_BI_GPM
                               , cu.firstPU->gpmDmvrRefinePart0
                               , cu.firstPU->gpmDmvrRefinePart1
                               , cu.firstPU->gpmDmvrRefinePart0 ? m_mvBufBDOF4GPM[cu.firstPU->geoMergeIdx0] : nullptr
                               , cu.firstPU->gpmDmvrRefinePart1 ? m_mvBufBDOF4GPM[cu.firstPU->geoMergeIdx1] : nullptr
#endif
    );
#else
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
#if JVET_Y0065_GPM_INTRA
    m_pcInterPred->motionCompensationGeo( cu, m_geoMrgCtx, m_geoTmMrgCtx0, m_geoTmMrgCtx1, m_pcIntraPred, (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr );
#else
    m_pcInterPred->motionCompensationGeo(cu, m_geoMrgCtx, m_geoTmMrgCtx0, m_geoTmMrgCtx1);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    PU::spanGeoMMVDMotionInfo(*cu.firstPU, m_geoMrgCtx, m_geoTmMrgCtx0, m_geoTmMrgCtx1, cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1, cu.firstPU->geoTmFlag0, cu.firstPU->geoMMVDFlag0, cu.firstPU->geoMMVDIdx0, cu.firstPU->geoTmFlag1, cu.firstPU->geoMMVDFlag1, cu.firstPU->geoMMVDIdx1, cu.firstPU->geoBldIdx);
#else
    PU::spanGeoMMVDMotionInfo(*cu.firstPU, m_geoMrgCtx, m_geoTmMrgCtx0, m_geoTmMrgCtx1, cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1, cu.firstPU->geoTmFlag0, cu.firstPU->geoMMVDFlag0, cu.firstPU->geoMMVDIdx0, cu.firstPU->geoTmFlag1, cu.firstPU->geoMMVDFlag1, cu.firstPU->geoMMVDIdx1);
#endif
#else
#if JVET_Y0065_GPM_INTRA
#if JVET_AE0046_BI_GPM
    m_pcInterPred->motionCompensationGeo(cu, m_geoMrgCtx, m_mvBufBDMVR, m_mvBufBDOF4GPM, m_pcIntraPred, (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr);
#else
    m_pcInterPred->motionCompensationGeo( cu, m_geoMrgCtx, m_pcIntraPred, (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr );
#endif
#else
    m_pcInterPred->motionCompensationGeo(cu, m_geoMrgCtx);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    PU::spanGeoMMVDMotionInfo(*cu.firstPU, m_geoMrgCtx, cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1, cu.firstPU->geoMMVDFlag0, cu.firstPU->geoMMVDIdx0, cu.firstPU->geoMMVDFlag1, cu.firstPU->geoMMVDIdx1, cu.firstPU->geoBldIdx);
#else
    PU::spanGeoMMVDMotionInfo(*cu.firstPU, m_geoMrgCtx, cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1, cu.firstPU->geoMMVDFlag0, cu.firstPU->geoMMVDIdx0, cu.firstPU->geoMMVDFlag1, cu.firstPU->geoMMVDIdx1);
#endif
#endif
#else
#if JVET_Y0065_GPM_INTRA
    m_pcInterPred->motionCompensationGeo( cu, m_geoMrgCtx, m_pcIntraPred, (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) ? &m_pcReshape->getFwdLUT() : nullptr );
#else
    m_pcInterPred->motionCompensationGeo( cu, m_geoMrgCtx );
#endif
    PU::spanGeoMotionInfo( *cu.firstPU, m_geoMrgCtx, cu.firstPU->geoSplitDir, cu.firstPU->geoMergeIdx0, cu.firstPU->geoMergeIdx1 );
#endif
#endif
  }
#if JVET_AC0112_IBC_GPM
  else if (cu.firstPU->ibcGpmFlag)
  {
    m_pcInterPred->motionCompensationIbcGpm(cu, m_ibcMrgCtx, m_pcIntraPred);
  }
#endif
  else
  {
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
    if (cu.firstPU->ciipFlag
#if CIIP_PDPC
      && !cu.firstPU->ciipPDPC
#endif
      )
    {
      PredictionUnit *pu = cu.firstPU;
      const CompArea &area = cu.Y();
      if (cu.slice->getSPS()->getUseTimd() && (cu.lwidth() * cu.lheight() <= CIIP_MAX_SIZE) && cu.cs->slice->getSPS()->getUseCiipTimd())
      {
#if SECONDARY_MPM && ENABLE_DIMD
        IntraPrediction::deriveDimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
#endif
        cu.timdMode = m_pcIntraPred->deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
        pu->intraDir[0] = MAP131TO67(cu.timdMode);
      }
    }
#endif

#if JVET_AC0112_IBC_CIIP
    uint8_t savedIntraDir[2] = {cu.firstPU->intraDir[0], cu.firstPU->intraDir[1]};
    if (CU::isIBC(cu) && cu.firstPU->ibcCiipFlag)
    {
      uint8_t ibcCiipIntraList[IBC_CIIP_MAX_NUM_INTRA_CANDS] = {PLANAR_IDX, HOR_IDX};
      m_pcIntraPred->deriveDimdMode(cu.cs->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
      cu.timdMode = m_pcIntraPred->deriveTimdMode(cu.cs->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
      ibcCiipIntraList[0] = MAP131TO67(cu.timdMode);
      ibcCiipIntraList[1] = ibcCiipIntraList[0] == HOR_IDX ? PLANAR_IDX : HOR_IDX;
      if (cu.firstPU->mergeFlag && cu.firstPU->ibcCiipIntraIdx > 0)
      {
        const PredictionUnit *puBv = cu.cs->getPURestricted(cu.lumaPos().offset(cu.firstPU->bv.getHor(), cu.firstPU->bv.getVer()), *cu.firstPU, cu.chType);
#if JVET_AE0169_BIPREDICTIVE_IBC
        int intraDir = -1;
        if (puBv)
        {
          intraDir = puBv->getIpmInfo(cu.lumaPos().offset(cu.firstPU->bv.getHor(), cu.firstPU->bv.getVer()));
          intraDir = (intraDir == ibcCiipIntraList[0]) ? -1 : intraDir;
        }
        if (intraDir < 0 && cu.firstPU->interDir == 3)
        {
          Mv bvL1 = cu.firstPU->mv[1];
          bvL1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
          puBv = cu.cs->getPURestricted(cu.lumaPos().offset(bvL1.getHor(), bvL1.getVer()), *cu.firstPU, cu.chType);
          if (puBv)
          {
            intraDir = puBv->getIpmInfo(cu.lumaPos().offset(bvL1.getHor(), bvL1.getVer()));
            intraDir = (intraDir == ibcCiipIntraList[0]) ? -1 : intraDir;
          }
        }
        if (intraDir < 0)
        {
          intraDir = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
        }
        ibcCiipIntraList[1] = intraDir;
#else
        uint8_t intraDir = puBv ? puBv->getIpmInfo(cu.lumaPos().offset(cu.firstPU->bv.getHor(), cu.firstPU->bv.getVer())) : PLANAR_IDX;
        if (intraDir != ibcCiipIntraList[0])
        {
          ibcCiipIntraList[1] = intraDir;
        }
        else
        {
          ibcCiipIntraList[1] = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
        }
#endif
      }
      cu.firstPU->intraDir[0] = ibcCiipIntraList[cu.firstPU->ibcCiipIntraIdx];
      cu.firstPU->intraDir[1] = cu.firstPU->intraDir[0];
    }
#endif
    m_pcIntraPred->geneIntrainterPred(cu, m_ciipBuffer);
#if JVET_AC0112_IBC_CIIP
    if (CU::isIBC(cu) && cu.firstPU->ibcCiipFlag)
    {
      cu.firstPU->intraDir[0] = savedIntraDir[0];
      cu.firstPU->intraDir[1] = savedIntraDir[1];
    }
#endif

    // inter prediction
    CHECK(CU::isIBC(cu) && cu.firstPU->ciipFlag, "IBC and Ciip cannot be used together");
    CHECK(CU::isIBC(cu) && cu.affine, "IBC and Affine cannot be used together");
    CHECK(CU::isIBC(cu) && cu.geoFlag, "IBC and geo cannot be used together");
    CHECK(CU::isIBC(cu) && cu.firstPU->mmvdMergeFlag, "IBC and MMVD cannot be used together");
    const bool luma = cu.Y().valid();
    const bool chroma = isChromaEnabled(cu.chromaFormat) && cu.Cb().valid();
    if (luma && (chroma || !isChromaEnabled(cu.chromaFormat)))
    {
      m_pcInterPred->motionCompensation(cu);
    }
    else
    {
#if JVET_AE0169_BIPREDICTIVE_IBC
      m_pcInterPred->motionCompensation(cu, REF_PIC_LIST_X, luma, chroma);
#else
      m_pcInterPred->motionCompensation(cu, REF_PIC_LIST_0, luma, chroma);
#endif
    }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    cu.firstPU->availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    if (m_pcInterPred->getDoAffineSubPuBdof() == true)
    {
      PU::setAffineBdofRefinedMotion(*cu.firstPU, m_mvBufDecAffineBDOF);
      m_pcInterPred->setDoAffineSubPuBdof(false);
    }
#endif
#if JVET_AD0213_LIC_IMP
#if JVET_AK0076_EXTENDED_OBMC_IBC
    if (cu.licFlag || cu.ibcLicFlag)
#else
    if (cu.licFlag)
#endif
    {
      Slice* slice = cu.slice;
#if JVET_AK0076_EXTENDED_OBMC_IBC
      bool disCond = (!CU::isIBC(cu) && cu.firstPU->ciipFlag && !(((slice->getPOC() - slice->getRefPOC(REF_PIC_LIST_0, 0)) == 1) && slice->getCheckLDC()))
        || (CU::isIBC(cu) && (cu.firstPU->ibcCiipFlag || cu.firstPU->ibcGpmFlag || cu.ibcFilterFlag));
#else
      bool disCond = cu.firstPU->ciipFlag && !(((slice->getPOC() - slice->getRefPOC(REF_PIC_LIST_0, 0)) == 1) && slice->getCheckLDC());
#endif
      for (int list = 0; list < 2; list++)
      {
        if (cu.firstPU->refIdx[list] >= 0)
        {
#if JVET_AC0112_IBC_LIC && JVET_AK0076_EXTENDED_OBMC_IBC
          for (int comp = 0; comp < (chroma ? MAX_NUM_COMPONENT : 1); comp++)
#else
          for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
#endif
          {
            if (disCond)
            {
              cu.licScale[list][comp] = 32;
              cu.licOffset[list][comp] = 0;
            }
            else
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            if (!cu.licInheritPara)
#endif
            {
              m_pcInterPred->setLicParam(list, comp, cu.licScale[list][comp], cu.licOffset[list][comp]);
#if JVET_AM0157_SGPM_BV_LIC
              if (cu.ibcLicIdx == IBC_LIC_IDX_M)
              {
                m_pcInterPred->setMMLicParam(list, comp, cu.ibcLicScale[list][0][comp], cu.ibcLicShift[list][0][comp], cu.ibcLicOffset[list][0][comp], cu.ibcLicScale[list][1][comp], cu.ibcLicShift[list][1][comp], cu.ibcLicOffset[list][1][comp], cu.ibcLicMean[list][comp]);
              }
#endif
            }
          }
        }
      }
    }
#endif
#if JVET_AG0276_NLIC
    if (!cu.cs->pcv->isEncoder)
    {
      if (CU::isSecLicParaNeeded(cu))
      {
        UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, cu.lumaSize().width, cu.lumaSize().height));
        PelUnitBuf predBeforeMCAdjBuffer = m_pcInterPred->m_acPredBeforeLICBuffer[REF_PIC_LIST_0].getBuf(localUnitArea);

        if (CU::isAllowSecLicPara(cu))
        {
          if (CU::isPredRefined(cu))
          {
#if INTER_LIC
            bool orgLicFlag = cu.licFlag;
            cu.licFlag = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            bool orgLicInheritedFlag = cu.licInheritPara;
            cu.licInheritPara = false;
#endif
#endif
            bool orgCiipFlag = cu.firstPU->ciipFlag;
            cu.firstPU->ciipFlag = false;
            m_pcInterPred->xPredWoRefinement(*cu.firstPU, predBeforeMCAdjBuffer);
#if INTER_LIC
            cu.licFlag = orgLicFlag;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            cu.licInheritPara = orgLicInheritedFlag;
#endif
#endif
            cu.firstPU->ciipFlag = orgCiipFlag;
          }
          else
          {
            predBeforeMCAdjBuffer.copyFrom(cu.cs->getPredBuf(*cu.firstPU));
          }
        }
        else
        {
          if (cu.affine && cu.firstPU->mergeFlag && cu.cs->sps->getUseOBMC() && cu.obmcFlag && (cu.cs->sps->getUseAltLM() || cu.cs->sps->getUseAffAltLM()))
          {
            if (cu.obmcFlag && m_pcInterPred->isSCC(*cu.firstPU) && !CU::isTLCond(cu))
            {
              cu.obmcFlag = false;
            }
          }
        }
      }
    }
#endif
#if MULTI_PASS_DMVR
    if (cu.firstPU->bdmvrRefine)
    {
#if JVET_AG0276_LIC_BDOF_BDMVR
      if (cu.licFlag == true)
      {
        memset((void*)m_pcInterPred->getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
      }
#endif
      PU::spanMotionInfo(*cu.firstPU, MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        cu.firstPU->colIdx,
#endif
        m_mvBufBDMVR[0], m_mvBufBDMVR[1], m_pcInterPred->getBdofSubPuMvOffset());
    }
#endif
  }
  if (cu.Y().valid())
  {
#if JVET_AC0112_IBC_CIIP
    if (CU::isIBC(cu) && cu.firstPU->ibcCiipFlag)
    {
#if ENABLE_OBMC && JVET_AK0076_EXTENDED_OBMC_IBC
      cu.isobmcMC = true;
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
      m_pcInterPred->subBlockOBMC(*cu.firstPU, nullptr, m_pcIntraPred, !(isChromaEnabled(cu.chromaFormat) && cu.Cb().valid()));
#else
      m_pcInterPred->subBlockOBMC(*cu.firstPU, nullptr, !(isChromaEnabled(cu.chromaFormat) && cu.Cb().valid()));
#endif
      cu.isobmcMC = false;
#endif
      const UnitArea localUnitArea( cu.cs->area.chromaFormat, Area( 0, 0, cu.Y().width, cu.Y().height ) );
      m_pcIntraPred->geneWeightedPred( COMPONENT_Y, cu.cs->getPredBuf( *cu.firstPU ).Y(), *cu.firstPU, cu.cs->getPredBuf( *cu.firstPU ).Y(), m_ciipBuffer.getBuf( localUnitArea.Y() ) );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( isChromaEnabled( cu.chromaFormat ) && cu.Cb().valid())
#else
      if( isChromaEnabled( cu.chromaFormat ) && cu.Cb().valid() && cu.chromaSize().width > 2 )
#endif
      {
        m_pcIntraPred->geneWeightedPred( COMPONENT_Cb, cu.cs->getPredBuf( *cu.firstPU ).Cb(), *cu.firstPU, cu.cs->getPredBuf( *cu.firstPU ).Cb(), m_ciipBuffer.getBuf( localUnitArea.Cb() ) );
        m_pcIntraPred->geneWeightedPred( COMPONENT_Cr, cu.cs->getPredBuf( *cu.firstPU ).Cr(), *cu.firstPU, cu.cs->getPredBuf( *cu.firstPU ).Cr(), m_ciipBuffer.getBuf( localUnitArea.Cr() ) );
      }
    }
#endif
#if JVET_AF0079_STORING_INTRATMP
    bool isIbcSmallBlk = false;
#else
    bool isIbcSmallBlk = CU::isIBC(cu) && (cu.lwidth() * cu.lheight() <= 16);

#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
    if (cu.cs->sps->getUseIbcNonAdjCand())
    {
      isIbcSmallBlk = false;
    }
#endif
#endif
    CU::saveMotionInHMVP( cu, isIbcSmallBlk );
  }

  if (cu.firstPU->ciipFlag)
  {
    const UnitArea localUnitArea( cu.cs->area.chromaFormat, Area( 0, 0, cu.Y().width, cu.Y().height ) );
#if ENABLE_OBMC && JVET_X0090_CIIP_FIX
    cu.isobmcMC = true;
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
    m_pcInterPred->subBlockOBMC(*cu.firstPU, nullptr, m_pcIntraPred);
#else
    m_pcInterPred->subBlockOBMC(*cu.firstPU);
#endif
    cu.isobmcMC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
    if (cu.firstPU->ciipAffine)
    {
      if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        m_pcIntraPred->geneWeightedCIIPAffinePred<true>(COMPONENT_Y, cu.cs->getPredBuf(*cu.firstPU).Y(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Y(), m_ciipBuffer.getBuf(localUnitArea.Y()), m_pcReshape->getFwdLUT().data());
      }
      else
      {
        m_pcIntraPred->geneWeightedCIIPAffinePred<false>(COMPONENT_Y, cu.cs->getPredBuf(*cu.firstPU).Y(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Y(), m_ciipBuffer.getBuf(localUnitArea.Y()));
      }

#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (isChromaEnabled(cu.chromaFormat))
#else
      if (isChromaEnabled(cu.chromaFormat) && cu.chromaSize().width > 2)
#endif
      {
        m_pcIntraPred->geneWeightedCIIPAffinePred<false>(COMPONENT_Cb, cu.cs->getPredBuf(*cu.firstPU).Cb(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Cb(), m_ciipBuffer.getBuf(localUnitArea.Cb()));
        m_pcIntraPred->geneWeightedCIIPAffinePred<false>(COMPONENT_Cr, cu.cs->getPredBuf(*cu.firstPU).Cr(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Cr(), m_ciipBuffer.getBuf(localUnitArea.Cr()));
      }
    }
    else
    {
#endif
      if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        m_pcIntraPred->geneWeightedPred<true>(COMPONENT_Y, cu.cs->getPredBuf(*cu.firstPU).Y(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Y(), m_ciipBuffer.getBuf(localUnitArea.Y()), m_pcReshape->getFwdLUT().data());
      }
      else
      {
        m_pcIntraPred->geneWeightedPred<false>(COMPONENT_Y, cu.cs->getPredBuf(*cu.firstPU).Y(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Y(), m_ciipBuffer.getBuf(localUnitArea.Y()));
      }

#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (isChromaEnabled(cu.chromaFormat))
#else
      if (isChromaEnabled(cu.chromaFormat) && cu.chromaSize().width > 2)
#endif
      {
        m_pcIntraPred->geneWeightedPred<false>(COMPONENT_Cb, cu.cs->getPredBuf(*cu.firstPU).Cb(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Cb(), m_ciipBuffer.getBuf(localUnitArea.Cb()));
        m_pcIntraPred->geneWeightedPred<false>(COMPONENT_Cr, cu.cs->getPredBuf(*cu.firstPU).Cr(), *cu.firstPU, cu.cs->getPredBuf(*cu.firstPU).Cr(), m_ciipBuffer.getBuf(localUnitArea.Cr()));
      }
#if JVET_AG0135_AFFINE_CIIP
    }
#endif
  }
#if ENABLE_OBMC
  cu.isobmcMC = true;
#if JVET_X0090_CIIP_FIX
#if JVET_Y0065_GPM_INTRA
#if JVET_AK0076_EXTENDED_OBMC_IBC
  bool isIbcGpmIntra = cu.firstPU->ibcGpmFlag && (cu.firstPU->ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS || cu.firstPU->ibcGpmMergeIdx1 >= IBC_GPM_MAX_NUM_UNI_CANDS);
  if (!cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !cu.firstPU->ibcCiipFlag && !isIbcGpmIntra
#else
  if (!cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag
#endif
#if JVET_AG0164_AFFINE_GPM
    && (!cu.firstPU->affineGPM[0] && !cu.firstPU->affineGPM[1])
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
    && (!cu.firstPU->geoBlendIntraFlag)
#endif
#if JVET_AK0212_GPM_OBMC_MODIFICATION
    && (!(cu.geoFlag && cu.slice->getCheckUseSepOBMC()))
#endif
    )
#else
  if (!cu.firstPU->ciipFlag)
#endif
  {
#if JVET_AJ0161_OBMC_EXT_WITH_INTRA_PRED
#if JVET_AK0076_EXTENDED_OBMC_IBC
    m_pcInterPred->subBlockOBMC(*cu.firstPU, nullptr, m_pcIntraPred, !(isChromaEnabled(cu.chromaFormat) && cu.Cb().valid()));
#else
    m_pcInterPred->subBlockOBMC(*cu.firstPU, nullptr, m_pcIntraPred);
#endif
#else
    m_pcInterPred->subBlockOBMC(*cu.firstPU);
#endif
  }
#else
  m_pcInterPred->subBlockOBMC(*cu.firstPU);
#endif
  cu.isobmcMC = false;
#endif
  DTRACE    ( g_trace_ctx, D_TMP, "pred " );
  DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getPredBuf( cu ), &cu.Y() );

  // inter recon
  xDecodeInterTexture(cu);

  // clip for only non-zero cbf case
  CodingStructure &cs = *cu.cs;

#if NNVC_USE_PRED
  cs.getPredBufCustom(cu).copyFrom(cs.getPredBuf(cu));
#endif

#if !SIGN_PREDICTION
  if (cu.rootCbf)
  {
#if !JVET_S0234_ACT_CRS_FIX
    if (cu.colorTransform)
    {
      cs.getResiBuf(cu).colorSpaceConvert(cs.getResiBuf(cu), false, cu.cs->slice->clpRng(COMPONENT_Y));
    }
#endif
#if REUSE_CU_RESULTS
    const CompArea &area = cu.blocks[COMPONENT_Y];
    CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpPred;
#endif
#if NNVC_USE_PRED
    cs.getPredBufCustom(cu).copyFrom(cs.getPredBuf(cu));
#endif
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
#if REUSE_CU_RESULTS
      if (cs.pcv->isEncoder)
      {
        tmpPred = m_tmpStorageLCU->getBuf(tmpArea);
        tmpPred.copyFrom(cs.getPredBuf(cu).get(COMPONENT_Y));
      }
#endif
#if JVET_Y0065_GPM_INTRA
      if (!cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
#else
      if (!cu.firstPU->ciipFlag && !CU::isIBC(cu))
#endif
        cs.getPredBuf(cu).get(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
    }
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getRecoBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
#else
    cs.getResiBuf( cu ).reconstruct( cs.getPredBuf( cu ), cs.getResiBuf( cu ), cs.slice->clpRngs() );
    cs.getRecoBuf( cu ).copyFrom   (                      cs.getResiBuf( cu ) );
#endif
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
#if REUSE_CU_RESULTS
      if (cs.pcv->isEncoder)
      {
        cs.getPredBuf(cu).get(COMPONENT_Y).copyFrom(tmpPred);
      }
#endif
    }
  }
  else
  {
#if NNVC_USE_PRED
    cs.getPredBufCustom(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
#endif
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
#if JVET_Y0065_GPM_INTRA
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
#else
    if (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !CU::isIBC(cu))
#endif
    {
      cs.getRecoBuf(cu).get(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
    }
  }
#if JVET_AA0070_RRIBC
  if (CU::isIBC(cu) && cu.rribcFlipType)
  {
    cu.cs->getRecoBuf(cu).get(COMPONENT_Y).flipSignal(cu.rribcFlipType == 1);
    if (isChromaEnabled(cu.chromaFormat) && cu.Cb().valid())
    {
      cu.cs->getRecoBuf(cu).get(COMPONENT_Cb).flipSignal(cu.rribcFlipType == 1);
      cu.cs->getRecoBuf(cu).get(COMPONENT_Cr).flipSignal(cu.rribcFlipType == 1);
    }
  }
#endif
#endif

#if JVET_AG0276_NLIC
  if (!cu.cs->pcv->isEncoder)
  {
    if (cu.altLMFlag)
    {
      cu.secAltLMParaUnit = cu.altLMParaUnit;
    }
    else
    {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
      cu.secAltLMParaUnit.resetAltLinearModel();
#else
      cu.secAltLMParaUnit = cu.altLMParaUnit;
#endif
    }
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
    cu.altLMParaUnit.resetAltLinearModel();
#endif
    if (CU::isSecLicParaNeeded(cu) && CU::isAllowSecLicPara(cu))
    {
      UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, cu.lumaSize().width, cu.lumaSize().height));
      PelUnitBuf predBeforeMCAdjBuffer = m_pcInterPred->m_acPredBeforeLICBuffer[REF_PIC_LIST_0].getBuf(localUnitArea);
      PelUnitBuf recBuf = m_pcInterPred->m_acPredBeforeLICBuffer[REF_PIC_LIST_1].getBuf(localUnitArea);
      recBuf.copyFrom(cu.cs->getRecoBuf(*cu.firstPU));
      if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        recBuf.Y().rspSignal(m_pcInterPred->m_pcReshape->getInvLUT());
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
      m_pcInterPred->xDevSecLicPara<false>(cu, predBeforeMCAdjBuffer, recBuf);
#else
      m_pcInterPred->xDevSecLicPara(cu, predBeforeMCAdjBuffer, recBuf);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      m_pcInterPred->xDevSecLicPara<true>(cu, predBeforeMCAdjBuffer, recBuf);
      if ((cu.altLMBRParaUnit.scale[COMPONENT_Y] == cu.altLMParaUnit.scale[COMPONENT_Y]) && (cu.altLMBRParaUnit.offset[COMPONENT_Y] == cu.altLMParaUnit.offset[COMPONENT_Y]))
      {
        cu.altLMBRParaUnit.resetAltLinearModel();
      }
#endif
    }
  }
#endif

  DTRACE    ( g_trace_ctx, D_TMP, "reco " );
  DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getRecoBuf( cu ), &cu.Y() );

  cs.setDecomp(cu);
}

void DecCu::xDecodeInterTU( TransformUnit & currTU, const ComponentID compID )
{
  if( !currTU.blocks[compID].valid() ) return;

  const CompArea &area = currTU.blocks[compID];

  CodingStructure& cs = *currTU.cs;

  //===== inverse transform =====
  PelBuf resiBuf  = cs.getResiBuf(area);

  QpParam cQP(currTU, compID);

#if SIGN_PREDICTION
  bool bJccrWithCr = currTU.jointCbCr && !(currTU.jointCbCr >> 1);
  bool bIsJccr     = currTU.jointCbCr && isChroma(compID);
  ComponentID signPredCompID = bIsJccr ? (bJccrWithCr ? COMPONENT_Cr : COMPONENT_Cb): compID;

  bool reshapeChroma = currTU.cs->slice->getPicHeader()->getLmcsEnabledFlag() && isChroma(signPredCompID) && (TU::getCbf(currTU, signPredCompID) || currTU.jointCbCr) && currTU.cs->slice->getPicHeader()->getLmcsChromaResidualScaleFlag() && currTU.blocks[signPredCompID].width * currTU.blocks[signPredCompID].height > 4;
#if JVET_AL0181_ASBT
  if ((currTU.asbtDecimW[compID] || currTU.asbtDecimH[compID]) && TU::getCbf(currTU, compID))
  {
    // First Set the right Direction:
    if (!cs.pcv->isEncoder && (currTU.asbtDecimH[compID] || currTU.asbtDecimW[compID]) &&
      !(currTU.blocks[COMPONENT_Y].height <= 4) &&
      !(currTU.blocks[COMPONENT_Y].width <= 4) &&
      !((currTU.blocks[COMPONENT_Y].width >> currTU.asbtDecimH[COMPONENT_Y]) < 4) &&
      !((currTU.blocks[COMPONENT_Y].width >> currTU.asbtDecimW[COMPONENT_Y]) < 4) &&
      !((currTU.blocks[COMPONENT_Y].height >> currTU.asbtDecimH[COMPONENT_Y]) < 4) &&
      !((currTU.blocks[COMPONENT_Y].height >> currTU.asbtDecimW[COMPONENT_Y]) < 4)
      )
    {
      m_pcInterPred->asbtCoordPred(currTU, currTU.asbtDecimW[compID], compID);
      if (currTU.asbtDecimH[compID])
      {
        m_pcInterPred->invTransposeTransform(currTU, compID);
      }
    }
    else
    {
      if (currTU.asbtDecimW[compID] && !currTU.asbtDecimH[compID])
      {
        m_pcInterPred->asbtCoord(currTU, compID, 0);
      }
      else
      {
        m_pcInterPred->asbtCoord(currTU, compID, 1);
      }
    }
  }
#endif
#if JVET_Y0065_GPM_INTRA
  if (currTU.cs->slice->getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma(compID) && !currTU.cu->firstPU->ciipFlag && !currTU.cu->firstPU->gpmIntraFlag && !CU::isIBC(*currTU.cu))
#else
  if (currTU.cs->slice->getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma(compID) && !currTU.cu->firstPU->ciipFlag && !CU::isIBC(*currTU.cu))
#endif
  {
#if JVET_Z0118_GDR
    cs.updateReconMotIPM(currTU.blocks[COMPONENT_Y], cs.getPredBuf(currTU.blocks[COMPONENT_Y]));
#else
    cs.picture->getRecoBuf(currTU.blocks[COMPONENT_Y]).copyFrom(cs.getPredBuf(currTU.blocks[COMPONENT_Y]));
#endif
    cs.getPredBuf(currTU.blocks[COMPONENT_Y]).rspSignal(m_pcReshape->getFwdLUT());
  }
#if JVET_AL0181_ASBT
  if ((currTU.asbtDecimW[compID] || currTU.asbtDecimH[compID]) && TU::getCbf(currTU, compID))
  {
    Pel predResiBorder[2 * SIGN_PRED_MAX_BS];
    if (currTU.asbtDecimW[compID] && !currTU.asbtDecimH[compID])
    {
      m_pcInterPred->asbtBorderResi(currTU, currTU.cs->picture->getRecoBuf(currTU.blocks[compID]),
        currTU.cs->getPredBuf(currTU.blocks[compID]), compID,
        predResiBorder, (currTU.cs->sps->getBitDepth(toChannelType(compID)) - 1));
    }
    else if (!currTU.asbtDecimW[compID] && currTU.asbtDecimH[compID])
    {
      m_pcInterPred->asbtBorderResiH(currTU, currTU.cs->picture->getRecoBuf(currTU.blocks[compID]),
        currTU.cs->getPredBuf(currTU.blocks[compID]), compID,
        predResiBorder, (currTU.cs->sps->getBitDepth(toChannelType(compID)) - 1));
    }
    m_pcTrQuant->predCoeffSigns(currTU, compID, reshapeChroma, predResiBorder);
  }
  else
  {
    m_pcTrQuant->predCoeffSigns(currTU, compID, reshapeChroma);
  }
#else
  m_pcTrQuant->predCoeffSigns(currTU, compID, reshapeChroma);
#endif
#if JVET_Y0065_GPM_INTRA
  if (currTU.cs->slice->getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma(compID) && !currTU.cu->firstPU->ciipFlag && !currTU.cu->firstPU->gpmIntraFlag && !CU::isIBC(*currTU.cu))
#else
  if (currTU.cs->slice->getPicHeader()->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma(compID) && !currTU.cu->firstPU->ciipFlag && !CU::isIBC(*currTU.cu))
#endif
  {
    cs.getPredBuf(currTU.blocks[COMPONENT_Y]).copyFrom(cs.picture->getRecoBuf(currTU.blocks[COMPONENT_Y]));
  }
#endif

  if( currTU.jointCbCr && isChroma(compID) )
  {
    if( compID == COMPONENT_Cb )
    {
      PelBuf resiCr = cs.getResiBuf( currTU.blocks[ COMPONENT_Cr ] );
      if( currTU.jointCbCr >> 1 )
      {
        m_pcTrQuant->invTransformNxN( currTU, COMPONENT_Cb, resiBuf, cQP );
      }
      else
      {
        QpParam qpCr(currTU, COMPONENT_Cr);
        m_pcTrQuant->invTransformNxN( currTU, COMPONENT_Cr, resiCr, qpCr );
      }
      m_pcTrQuant->invTransformICT( currTU, resiBuf, resiCr );
#if JVET_AL0181_ASBT 
      if (currTU.asbtDecimW[COMPONENT_Cb])
      {
        m_pcInterPred->upsampleOneBlock(currTU, resiBuf, COMPONENT_Cb);
        m_pcInterPred->upsampleOneBlock(currTU, resiCr, COMPONENT_Cr);
      }

      if (currTU.asbtDecimH[COMPONENT_Cb])
      {
        m_pcInterPred->upsampleOneBlockH(currTU, resiBuf, COMPONENT_Cb);
        m_pcInterPred->upsampleOneBlockH(currTU, resiCr, COMPONENT_Cr);
      }
#endif
    }
  }
  else
  if( TU::getCbf( currTU, compID ) )
  {
#if JVET_AL0181_ASBT
    if ((currTU.asbtDecimW[compID] || currTU.asbtDecimH[compID]))
    {
      m_pcTrQuant->invTransformNxN(currTU, compID, resiBuf, cQP);
      if (currTU.asbtDecimW[compID] && (!currTU.asbtDecimH[compID]))
      {
        m_pcInterPred->upsampleOneBlock(currTU, resiBuf, compID);
      }
      else if ((!currTU.asbtDecimW[compID]) && currTU.asbtDecimH[compID])
      {
        m_pcInterPred->upsampleOneBlockH(currTU, resiBuf, compID);
      }
    }
    else
    {
      m_pcTrQuant->invTransformNxN(currTU, compID, resiBuf, cQP);
    }
#else
    m_pcTrQuant->invTransformNxN( currTU, compID, resiBuf, cQP );
#endif
  }
  else
  {
    resiBuf.fill( 0 );
  }

  //===== reconstruction =====
  const Slice           &slice = *cs.slice;
#if JVET_S0234_ACT_CRS_FIX
  if (!currTU.cu->colorTransform && slice.getLmcsEnabledFlag() && isChroma(compID) && (TU::getCbf(currTU, compID) || currTU.jointCbCr)
#else
  if (slice.getLmcsEnabledFlag() && isChroma(compID) && (TU::getCbf(currTU, compID) || currTU.jointCbCr)
#endif
   && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && currTU.blocks[compID].width * currTU.blocks[compID].height > 4)
  {
    resiBuf.scaleSignal(currTU.getChromaAdj(), 0, currTU.cu->cs->slice->clpRng(compID));
  }

#if SIGN_PREDICTION
  int firstComponent = compID;
  int lastComponent  = compID;

  if( currTU.jointCbCr && isChroma(compID) )
  {
    if( compID == COMPONENT_Cb )
    {
      firstComponent = MAX_INT;
    }
    else
    {
      firstComponent -= 1;
    }
  }
  for( auto i = firstComponent; i <= lastComponent; ++i)
  {
    ComponentID currCompID = (ComponentID) i;
    PelBuf compResiBuf = cs.getResiBuf(currTU.blocks[currCompID]);

#if JVET_Y0065_GPM_INTRA
    if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( currCompID ) && !currTU.cu->firstPU->ciipFlag && !currTU.cu->firstPU->gpmIntraFlag && !CU::isIBC( *currTU.cu ) )
#else
    if( cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && isLuma( currCompID ) && !currTU.cu->firstPU->ciipFlag && !CU::isIBC( *currTU.cu ) )
#endif
    {
      PelBuf picRecoBuff = currTU.cs->picture->getRecoBuf( currTU.blocks[currCompID] );
#if JVET_Z0118_GDR
      currTU.cs->rspSignalPicture(currTU.blocks[currCompID], m_pcReshape->getFwdLUT());
#else
      picRecoBuff.rspSignal( cs.getPredBuf( currTU.blocks[currCompID] ), m_pcReshape->getFwdLUT() );
#endif
#if JVET_AG0145_ADAPTIVE_CLIPPING
      ClpRng clpRng = currTU.cu->cs->slice->clpRng(currCompID);
      if (currCompID == COMPONENT_Y)
      {
        std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
        clpRng.min = fwdLUT[currTU.cu->cs->slice->getLumaPelMin()];
        clpRng.max = fwdLUT[currTU.cu->cs->slice->getLumaPelMax()];
      }
      currTU.cs->getRecoBuf( currTU.blocks[currCompID] ).reconstruct( picRecoBuff, compResiBuf, clpRng);
#else
      currTU.cs->getRecoBuf( currTU.blocks[currCompID] ).reconstruct( picRecoBuff, compResiBuf, currTU.cu->cs->slice->clpRng( currCompID ) );
#endif
    }
    else
    {
#if JVET_AG0145_ADAPTIVE_CLIPPING
      ClpRng clpRng = currTU.cu->cs->slice->clpRng(currCompID);
      if (currCompID == COMPONENT_Y)
      {
        if (cs.slice->getSPS()->getUseLmcs() && cs.picHeader->getLmcsEnabledFlag())
        {
          std::vector<Pel>& fwdLUT = m_pcReshape->getFwdLUT();
          clpRng.min = fwdLUT[currTU.cu->cs->slice->getLumaPelMin()];
          clpRng.max = fwdLUT[currTU.cu->cs->slice->getLumaPelMax()];
        }
        else
        {
          clpRng.min = currTU.cu->cs->slice->getLumaPelMin();
          clpRng.max = currTU.cu->cs->slice->getLumaPelMax();
        }
      }
      currTU.cs->getRecoBuf( currTU.blocks[currCompID] ).reconstruct( cs.getPredBuf( currTU.blocks[currCompID] ), compResiBuf, clpRng);
#else
      currTU.cs->getRecoBuf( currTU.blocks[currCompID] ).reconstruct( cs.getPredBuf( currTU.blocks[currCompID] ), compResiBuf, currTU.cu->cs->slice->clpRng( currCompID ) );
#endif
#if JVET_AA0070_RRIBC
      if (CU::isIBC(*currTU.cu) && currTU.cu->rribcFlipType)
      {
        currTU.cs->getRecoBuf(currTU.blocks[currCompID]).flipSignal(currTU.cu->rribcFlipType == 1);
      }
#endif
    }
  }
#endif
}

void DecCu::xDecodeInterTexture(CodingUnit &cu)
{
  if( !cu.rootCbf )
  {
#if SIGN_PREDICTION
    CodingStructure &cs = *cu.cs;
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    cs.getResiBuf(cu).bufs[0].fill(0);
#endif
#if NNVC_USE_PRED
    cs.getPredBufCustom(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
#endif
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->clpRngs());
#if JVET_Y0065_GPM_INTRA
    if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
#else
    if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !CU::isIBC(cu))
#endif
    {
      cs.getRecoBuf(cu).get(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
    }
#if JVET_AG0145_ADAPTIVE_CLIPPING
    ClpRng clpRng = cs.slice->clpRng(COMPONENT_Y);
    if (cs.slice->getSPS()->getUseLmcs() && cs.slice->getLmcsEnabledFlag())
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
    cs.getRecoBuf(cu).get(COMPONENT_Y).reconstruct(cs.getRecoBuf(cu).get(COMPONENT_Y), cs.getResiBuf(cu).get(COMPONENT_Y), clpRng);
#endif
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    if (CU::isIBC(cu) && cu.rribcFlipType)
    {
      cu.cs->getRecoBuf(cu).get(COMPONENT_Y).flipSignal(cu.rribcFlipType == 1);
    }
    if (cu.interCcpMergeZeroRootCbfIdc)
    {
      int validNum = 0;
      CCPModelCandidate interCcpMergeList[MAX_CCP_CAND_LIST_SIZE];
      m_pcIntraPred->xAddOnTheFlyCalcCCPCands4InterBlk(*cu.firstPU, cu.blocks[COMPONENT_Cb], interCcpMergeList, validNum);
      PelBuf bufCb = cs.getPredBuf( cu.blocks[COMPONENT_Cb] );
      PelBuf bufCr = cs.getPredBuf( cu.blocks[COMPONENT_Cr] );
      const bool valid = m_pcInterPred->deriveInterCcpMergePrediction(cu.firstTU, cs.getRecoBuf(cu.blocks[COMPONENT_Y]), bufCb, bufCr, bufCb, bufCr, interCcpMergeList, validNum);
      CHECK( !valid, "invalid inter ccp merge for rootCbf = 0" );
#if NNVC_USE_PRED
      cs.getPredBufCustom(cu).Cb().copyFrom(bufCb);
      cs.getPredBufCustom(cu).Cr().copyFrom(bufCr);
#endif
      cs.getRecoBuf(cu.blocks[COMPONENT_Cb]).copyClip(bufCb, cs.slice->clpRng(COMPONENT_Cb));
      cs.getRecoBuf(cu.blocks[COMPONENT_Cr]).copyClip(bufCr, cs.slice->clpRng(COMPONENT_Cr));
      cu.firstPU->idxNonLocalCCP = 0;
      cu.firstTU->curCand ={};
      cu.firstTU->curCand.type = CCP_TYPE_NONE;
    }
#endif
#if JVET_AA0070_RRIBC
    if (CU::isIBC(cu) && cu.rribcFlipType)
    {
#if !JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
      cu.cs->getRecoBuf(cu).get(COMPONENT_Y).flipSignal(cu.rribcFlipType == 1);
#endif
      if (isChromaEnabled(cu.chromaFormat) && cu.Cb().valid())
      {
        cu.cs->getRecoBuf(cu).get(COMPONENT_Cb).flipSignal(cu.rribcFlipType == 1);
        cu.cs->getRecoBuf(cu).get(COMPONENT_Cr).flipSignal(cu.rribcFlipType == 1);
      }
    }
#endif
#endif
    return;
  }

#if JVET_AG0061_INTER_LFNST_NSPT
  if (cu.lfnstIdx)
  {
#if JVET_AI0050_INTER_MTSS
    int secondDimdIntraDir = 0;
#endif
#if JVET_AI0050_SBT_LFNST
    if (cu.cs->sps->getUseSbtLFNST() && cu.sbtInfo)
    {
      Position pos(0, 0);
      Size size(0, 0);
      int sbtIdx = cu.getSbtIdx();
      int sbtPos = cu.getSbtPos();
      CU::getSBTPosAndSize(cu, pos, size, CU::getSbtMode(sbtIdx, sbtPos));
      cu.dimdDerivedIntraDir = m_pcIntraPred->deriveIpmForTransform(cu.cs->getPredBuf(*cu.firstPU).Y().subBuf(pos, size), cu
#if JVET_AI0050_INTER_MTSS
        , secondDimdIntraDir
#endif
#if JVET_AJ0203_DIMD_2X2_EDGE_OP
        , true
#endif 
      );
    }
    else
    {
#endif
      cu.dimdDerivedIntraDir = m_pcIntraPred->deriveIpmForTransform(cu.cs->getPredBuf(*cu.firstPU).Y(), cu
#if JVET_AI0050_INTER_MTSS
        , secondDimdIntraDir
#endif
#if JVET_AJ0203_DIMD_2X2_EDGE_OP
        , true
#endif 
      );
#if JVET_AI0050_SBT_LFNST
    }
#endif
#if JVET_AI0050_INTER_MTSS
    cu.dimdDerivedIntraDir2nd = secondDimdIntraDir;
#endif
  }
#endif

  const uint32_t uiNumVaildComp = getNumberValidComponents(cu.chromaFormat);

#if JVET_S0234_ACT_CRS_FIX
  if (cu.colorTransform)
  {
    CodingStructure  &cs = *cu.cs;
    const Slice &slice = *cs.slice;
    for (auto& currTU : CU::traverseTUs(cu))
    {
      for (uint32_t ch = 0; ch < uiNumVaildComp; ch++)
      {
        const ComponentID compID = ComponentID(ch);
        if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (compID == COMPONENT_Y))
        {
          const CompArea &areaY = currTU.blocks[COMPONENT_Y];
          int adj = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
          currTU.setChromaAdj(adj);
        }
        xDecodeInterTU(currTU, compID);
      }

      cs.getResiBuf(currTU).colorSpaceConvert(cs.getResiBuf(currTU), false, cu.cs->slice->clpRng(COMPONENT_Y));
      if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && currTU.blocks[COMPONENT_Cb].width * currTU.blocks[COMPONENT_Cb].height > 4)
      {
        cs.getResiBuf(currTU.blocks[COMPONENT_Cb]).scaleSignal(currTU.getChromaAdj(), 0, currTU.cu->cs->slice->clpRng(COMPONENT_Cb));
        cs.getResiBuf(currTU.blocks[COMPONENT_Cr]).scaleSignal(currTU.getChromaAdj(), 0, currTU.cu->cs->slice->clpRng(COMPONENT_Cr));
      }
    }
  }
  else
  {
#endif
#if SIGN_PREDICTION
  for ( auto& currTU : CU::traverseTUs( cu ) )
  {
    for(uint32_t ch = 0; ch < uiNumVaildComp; ch++)
    {
      const ComponentID compID = ComponentID(ch);
#else
  for (uint32_t ch = 0; ch < uiNumVaildComp; ch++)
  {
    const ComponentID compID = ComponentID(ch);

    for( auto& currTU : CU::traverseTUs( cu ) )
    {
#endif
      CodingStructure  &cs = *cu.cs;
      const Slice &slice = *cs.slice;
#if JVET_AE0059_INTER_CCCM
      if (currTU.interCccm && compID == COMPONENT_Cb && currTU.blocks[COMPONENT_Cb].valid())
      {
        PelStorage tmpInterCccmStorage;
        tmpInterCccmStorage.create(CHROMA_400,currTU.blocks[COMPONENT_Y]);
        tmpInterCccmStorage.getBuf(COMPONENT_Y).copyFrom(cs.getPredBuf(currTU.blocks[COMPONENT_Y]));
        if (cs.picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && !cu.firstPU->ciipFlag && !cu.firstPU->gpmIntraFlag && !CU::isIBC(cu))
        {
          tmpInterCccmStorage.getBuf(COMPONENT_Y).rspSignal(m_pcReshape->getFwdLUT());
        }

        PelBuf bufCb = cs.getPredBuf( currTU.blocks[COMPONENT_Cb] );
        PelBuf bufCr = cs.getPredBuf( currTU.blocks[COMPONENT_Cr] );

        const bool valid = m_pcInterPred->deriveInterCccmPrediction(&currTU, tmpInterCccmStorage.getBuf(COMPONENT_Y), cs.getRecoBuf(currTU.blocks[COMPONENT_Y]), bufCb, bufCr, bufCb, bufCr );

        CHECK( !valid, "invalid inter cccm" );

#if SIGN_PREDICTION
        cs.getRecoBuf(currTU.blocks[COMPONENT_Cb]).copyClip(cs.getPredBuf(currTU.blocks[COMPONENT_Cb]), cs.slice->clpRng(COMPONENT_Cb));
        cs.getRecoBuf(currTU.blocks[COMPONENT_Cr]).copyClip(cs.getPredBuf(currTU.blocks[COMPONENT_Cr]), cs.slice->clpRng(COMPONENT_Cr));
#endif
      }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
      if (currTU.interCcpMerge && compID == COMPONENT_Cb && currTU.blocks[COMPONENT_Cb].valid())
      {
        PelBuf bufCb = cs.getPredBuf( currTU.blocks[COMPONENT_Cb] );
        PelBuf bufCr = cs.getPredBuf( currTU.blocks[COMPONENT_Cr] );

        CCPModelCandidate interCcpMergeList[MAX_CCP_CAND_LIST_SIZE];
        int validNum = 0;
        m_pcIntraPred->xAddOnTheFlyCalcCCPCands4InterBlk(*currTU.cu->firstPU, currTU.blocks[COMPONENT_Cb], interCcpMergeList, validNum);
        const bool valid = m_pcInterPred->deriveInterCcpMergePrediction(&currTU, cs.getRecoBuf(currTU.blocks[COMPONENT_Y]), bufCb, bufCr, bufCb, bufCr, interCcpMergeList, validNum);

        CHECK( !valid, "invalid inter ccp merge" );

#if SIGN_PREDICTION
        cs.getRecoBuf(currTU.blocks[COMPONENT_Cb]).copyClip(cs.getPredBuf(currTU.blocks[COMPONENT_Cb]), cs.slice->clpRng(COMPONENT_Cb));
        cs.getRecoBuf(currTU.blocks[COMPONENT_Cr]).copyClip(cs.getPredBuf(currTU.blocks[COMPONENT_Cr]), cs.slice->clpRng(COMPONENT_Cr));
#endif
      }
#endif
      if (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (compID == COMPONENT_Y) && (currTU.cbf[COMPONENT_Cb] || currTU.cbf[COMPONENT_Cr]))
      {
#if LMCS_CHROMA_CALC_CU
        const CompArea &areaY = cu.blocks[COMPONENT_Y];
#else
        const CompArea &areaY = currTU.blocks[COMPONENT_Y];
#endif
        int adj = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
        currTU.setChromaAdj(adj);
    }
      xDecodeInterTU( currTU, compID );
#if SIGN_PREDICTION
      if(cs.pcv->isEncoder && cs.sps->getNumPredSigns() > 0)
      {
        PelBuf picRecoBuff = currTU.cs->picture->getRecoBuf( currTU.blocks[compID] );
        picRecoBuff.copyFrom(currTU.cs->getRecoBuf( currTU.blocks[compID] ));
      }
#endif
    }
  }
#if JVET_S0234_ACT_CRS_FIX
  }
#endif
}

void DecCu::xDeriveCUMV(CodingUnit &cu)
{
  for (auto &pu : CU::traversePUs(cu))
  {
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
    if (cu.cs->pcv->isEncoder && !cu.geoFlag && !(!pu.cu->affine && PU::checkBDMVRCondition(pu)) && pu.mergeType != MRG_TYPE_SUBPU_ATMVP
#if JVET_AG0098_AMVP_WITH_SBTMVP
      && !pu.amvpSbTmvpFlag
#endif
    )
    {
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
      if (PU::checkDoAffineBdofRefine(pu, m_pcInterPred))
      {
        pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
        m_pcInterPred->setDoAffineSubPuBdof(false);
        m_pcInterPred->setBdofSubPuMvBuf(m_mvBufDecAffineBDOF);
      }
#endif
      PU::spanMotionInfo(pu);
      continue;
    }
#endif
    MergeCtx mrgCtx;

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
    if( pu.cu->affine )
    {
      CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType{ STATS__TOOL_AFF, pu.Y().width, pu.Y().height } );
    }
#endif

    if( pu.mergeFlag )
    {
#if MULTI_HYP_PRED
#if REUSE_CU_RESULTS
      CHECK(!cu.cs->pcv->isEncoder && cu.skip && !pu.addHypData.empty(), "Dec: additional hypotheseis signalled in SKIP mode");
      CHECK(cu.skip && pu.addHypData.size() != pu.numMergedAddHyps, "additional hypotheseis signalled in SKIP mode");
      // save signalled add hyp data, to put it at the end after merging
      MultiHypVec addHypData = { std::begin(pu.addHypData) + pu.numMergedAddHyps, std::end(pu.addHypData) };
#else
      CHECK(cu.skip && !pu.addHypData.empty(), "additional hypotheseis signalled in SKIP mode");
      // save signalled add hyp data, to put it at the end after merging
      auto addHypData = std::move(pu.addHypData);
#endif
      pu.addHypData.clear();
      pu.numMergedAddHyps = 0;
#endif
      if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
      {
        CHECK(pu.ciipFlag == true, "invalid Ciip");
        if (pu.cs->sps->getSbTMVPEnabledFlag())
        {
          Size bufSize = g_miScaling.scale(pu.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
          for (int i = 0; i < SUB_TMVP_NUM; i++)
          {
            mrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
          }
#else
          mrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
        }
#if JVET_AL0160_SBSMVP
        if(pu.cs->sps->getSpatialMVPEnabledFlag())
        {
          Size bufSize = g_miScaling.scale(pu.lumaSize());
          for (int i = 0; i < NUM_SUB_SMVP; i++)
          {
            mrgCtx.subSpatialPuMvpMiBuf[i] = MotionBuf(m_subSpatialPuMiBuf[i], bufSize);
          }
        }
#endif
        int   fPosBaseIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                            !pu.cs->sps->getUseTMMMVD() ? 
                            pu.mmvdMergeIdx / VVC_MMVD_MAX_REFINE_NUM :
#endif
                            pu.mmvdMergeIdx / MMVD_MAX_REFINE_NUM;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
        PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 1, fPosBaseIdx + 1);
#else
        PU::getInterMergeCandidates(pu, mrgCtx, 1, fPosBaseIdx + 1);
#endif
        PU::getInterMMVDMergeCandidates(pu, mrgCtx,
          pu.mmvdMergeIdx
        );
#if JVET_AF0128_LIC_MERGE_TM
        if (pu.cs->sps->getUseAML()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          && pu.cs->sps->getTMToolsEnableFlag()
#endif
          )
        {
          m_pcInterPred->adjustMergeCandidatesLicFlag(pu, mrgCtx, fPosBaseIdx);
        }
#endif
#if JVET_AB0079_TM_BCW_MRG
        if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
          && pu.cs->sps->getTMToolsEnableFlag()
#endif
          )
        {
          m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, fPosBaseIdx);
        }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        uint32_t mmvdLUT[MMVD_ADD_NUM];
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
        uint16_t mmvdIdx = pu.mmvdMergeIdx;
#else
        uint8_t mmvdIdx = pu.mmvdMergeIdx;
#endif
        m_pcInterPred->sortInterMergeMMVDCandidates(pu, mrgCtx, mmvdLUT, mmvdIdx);
        mrgCtx.setMmvdMergeCandiInfo(pu, mmvdIdx, mmvdLUT[mmvdIdx]);
#else
        mrgCtx.setMmvdMergeCandiInfo(pu, pu.mmvdMergeIdx);
#endif

        PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
          , pu.colIdx
#endif       
        );
      }
      else
      {
      {
        if( pu.cu->geoFlag )
        {     
#if JVET_AJ0107_GPM_SHAPE_ADAPT
          int whIdx = !cu.cs->slice->getSPS()->getUseGeoShapeAdapt() ? GEO_SQUARE_IDX : Clip3(0, GEO_NUM_CU_SHAPES-1, floorLog2(cu.firstPU->lwidth()) - floorLog2(cu.firstPU->lheight()) + GEO_SQUARE_IDX);
#endif
#if JVET_AG0164_AFFINE_GPM
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
          if (pu.affineGPM[0] || pu.affineGPM[1] || pu.sgpmAffine)
#else
          if (pu.affineGPM[0] || pu.affineGPM[1])
#endif
          {
            PU::getGeoAffMergeCandidates(pu, m_geoAffMrgCtx, m_pcInterPred);
            if (pu.cs->sps->getUseAML() && (m_geoAffMrgCtx.numValidMergeCand > 1)
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              && pu.cs->sps->getTMToolsEnableFlag()
#endif
              )
            {
              m_geoAffMrgCtx.maxNumMergeCand = m_geoAffMrgCtx.numValidMergeCand;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
              if (!pu.sgpmAffine)              
              {
                bool affineStored = pu.cu->affine;
                int  affTypeStored = pu.cu->affineType;
                int  bcwStored = pu.cu->bcwIdx;
#if INTER_LIC
                bool licStored = pu.cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                bool obmcStored = pu.cu->obmcFlag;
#endif
                m_pcInterPred->adjustAffineMergeCandidates(pu, m_geoAffMrgCtx);
                pu.cu->affine = affineStored;
                pu.cu->affineType = affTypeStored;
                pu.cu->bcwIdx = bcwStored;
#if INTER_LIC
                pu.cu->licFlag = licStored;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                pu.cu->obmcFlag = obmcStored;
#endif
#else
              PredictionUnit puSaved = pu;
              CodingUnit     cuSaved = *pu.cu;
              puSaved.cu = &cuSaved;
              m_pcInterPred->adjustAffineMergeCandidates(puSaved, m_geoAffMrgCtx);
#endif

             m_geoAffMrgCtx.numValidMergeCand = std::min(m_geoAffMrgCtx.numValidMergeCand, (int)pu.cs->sps->getMaxNumGpmAffCand());
             m_geoAffMrgCtx.maxNumMergeCand = m_geoAffMrgCtx.numValidMergeCand;
#if JVET_AJ0274_GPM_AFFINE_TM
            m_geoAffTmMrgCtx = m_geoAffMrgCtx;
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA 
              }
#endif
            }
          }
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
          if (!pu.affineGPM[0] || !pu.affineGPM[1] || pu.sgpmAffine)
#else
          if (!pu.affineGPM[0] || !pu.affineGPM[1])
#endif
          {
#endif
#if JVET_AE0046_BI_GPM
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            PU::getGeoMergeCandidates(pu, m_geoMrgCtx, m_pcInterPred, nullptr, true);
#else
            PU::getGeoMergeCandidates(pu, m_geoMrgCtx, nullptr, true);
#endif
#else
            PU::getGeoMergeCandidates(pu, m_geoMrgCtx);
#endif

#if JVET_AG0164_AFFINE_GPM
          }
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
          if (pu.cu->geoBlendFlag)
          {
            return;
          }
#endif
#if JVET_AL0134_SGPM_INTER
          if (pu.sgpmInterTm)
          {
            int numInter = std::min(m_geoMrgCtx.numValidMergeCand, GEO_MAX_NUM_UNI_CANDS);

            int  pocMrg[GEO_MAX_NUM_UNI_CANDS];
            Mv   mrgMv[GEO_MAX_NUM_UNI_CANDS];
            bool mrgDuplicated[GEO_MAX_NUM_UNI_CANDS];

            for (int mergeCand = 0; mergeCand < numInter; mergeCand++)
            {
              int mrgList   = m_geoMrgCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
              int mrgRefIdx = m_geoMrgCtx.mvFieldNeighbours[(mergeCand << 1) + mrgList].refIdx;

              pocMrg[mergeCand]        = pu.cs->slice->getRefPic((RefPicList) mrgList, mrgRefIdx)->getPOC();
              mrgMv[mergeCand]         = m_geoMrgCtx.mvFieldNeighbours[(mergeCand << 1) + mrgList].mv;
              mrgDuplicated[mergeCand] = false;
              if (mergeCand)
              {
                for (int i = 0; i < mergeCand; i++)
                {
                  if (pocMrg[mergeCand] == pocMrg[i] && mrgMv[mergeCand] == mrgMv[i])
                  {
                    mrgDuplicated[mergeCand] = true;
                    break;
                  }
                }
              }
            }
            for (int i = GEO_TM_SHAPE_AL; i < GEO_NUM_TM_MV_CAND; i++)
            {
              m_geoTmMrgCtx[i].numValidMergeCand = m_geoMrgCtx.numValidMergeCand;
              for (int idx = 0; idx < m_geoTmMrgCtx[i].numValidMergeCand; idx++)
              {
                if (mrgDuplicated[idx])
                {
                  continue;
                }
                m_geoTmMrgCtx[i].bcwIdx[idx]       = BCW_DEFAULT;
                m_geoTmMrgCtx[i].useAltHpelIf[idx] = false;
#if JVET_AG0276_NLIC
                m_geoTmMrgCtx[i].altLMFlag[idx] = false;
                m_geoTmMrgCtx[i].altLMParaNeighbours[idx].resetAltLinearModel();
#endif
#if INTER_LIC
                m_geoTmMrgCtx[i].licFlags[idx] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                m_geoTmMrgCtx[i].copyLICParamFromCtx(idx, m_geoMrgCtx, idx);
                m_geoTmMrgCtx[i].licFlags[idx] = m_geoTmMrgCtx[i].licInheritPara[idx];
#else
                m_geoTmMrgCtx[i].setDefaultLICParamToCtx(idx);
#endif
#endif
#endif
                m_geoTmMrgCtx[i].interDirNeighbours[idx]          = m_geoMrgCtx.interDirNeighbours[idx];
                m_geoTmMrgCtx[i].mvFieldNeighbours[(idx << 1)].mv = m_geoMrgCtx.mvFieldNeighbours[(idx << 1)].mv;
                m_geoTmMrgCtx[i].mvFieldNeighbours[(idx << 1) + 1].mv = m_geoMrgCtx.mvFieldNeighbours[(idx << 1) + 1].mv;
                m_geoTmMrgCtx[i].mvFieldNeighbours[(idx << 1)].refIdx = m_geoMrgCtx.mvFieldNeighbours[(idx << 1)].refIdx;
                m_geoTmMrgCtx[i].mvFieldNeighbours[(idx << 1) + 1].refIdx = m_geoMrgCtx.mvFieldNeighbours[(idx << 1) + 1].refIdx;
              }
            }

            bool origTmMrgFlag = pu.tmMergeFlag;
            pu.tmMergeFlag     = true;

            for (uint8_t tmType = GEO_TM_SHAPE_AL; tmType < GEO_NUM_TM_MV_CAND; tmType++)
            {
              pu.geoTmType = tmType;
              for (uint8_t mrgIdx = 0; mrgIdx < m_geoMrgCtx.numValidMergeCand; mrgIdx++)
              {
                if (mrgDuplicated[mrgIdx])
                {
                  continue;
                }

                m_geoTmMrgCtx[tmType].setMergeInfo(pu, mrgIdx);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                m_pcInterPred->deriveTMMv(pu, NULL, mrgIdx);
#else
                m_pcInterPred->deriveTMMv(pu);
#endif
#if JVET_AE0046_BI_GPM
                m_geoTmMrgCtx[tmType].interDirNeighbours[mrgIdx] = pu.interDir;
                m_geoTmMrgCtx[tmType].bcwIdx[mrgIdx] = (pu.interDir != 3) ? BCW_DEFAULT : m_geoTmMrgCtx[tmType].bcwIdx[mrgIdx];
                m_geoTmMrgCtx[tmType].mvFieldNeighbours[(mrgIdx << 1)].refIdx     = pu.refIdx[0];
                m_geoTmMrgCtx[tmType].mvFieldNeighbours[(mrgIdx << 1) + 1].refIdx = pu.refIdx[1];
#endif
                m_geoTmMrgCtx[tmType].mvFieldNeighbours[(mrgIdx << 1)].mv     = pu.mv[0];
                m_geoTmMrgCtx[tmType].mvFieldNeighbours[(mrgIdx << 1) + 1].mv = pu.mv[1];
              }
            }
            pu.tmMergeFlag = origTmMrgFlag;

            return;
          }
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
          if (pu.geoTmFlag0)
          {
#if JVET_AJ0274_GPM_AFFINE_TM
            if (pu.affineGPM[0])
            {
              for (int i = 0; i < 3; i++)
              {
                m_mvBufBDMVR[0][i].setZero();
                m_mvBufBDMVR[1][i].setZero();
              }
              int uiAffMergeCand = pu.geoMergeIdx0 - m_geoAffTmMrgCtx.m_indexOffset;
              m_geoAffTmMrgCtx.setAffMergeInfo(pu, pu.geoMergeIdx0);
              m_pcInterPred->setFillCurTplAboveARMC(false);
              m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
              pu.bdmvrRefine = false;
              m_pcInterPred->processTM4Affine(pu, m_geoAffTmMrgCtx, 0, false
#if JVET_AH0119_SUBBLOCK_TM
                                              , pu.cs->slice->getCheckLDB() ? true : false
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                                              , uiAffMergeCand
#endif
                                              );
              pu.cu->affine = false;
              m_pcInterPred->setFillCurTplAboveARMC(false);
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR[0][0];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR[0][1];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR[0][2];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR[1][0];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR[1][1];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR[1][2];
            }
            else
            {
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
            MergeCtx& m_geoTmMrgCtx0 = m_geoTmMrgCtx[GEO_TM_SHAPE_AL];
#endif
            m_geoTmMrgCtx0.numValidMergeCand = m_geoMrgCtx.numValidMergeCand;
            m_geoTmMrgCtx0.bcwIdx[pu.geoMergeIdx0] = BCW_DEFAULT;
            m_geoTmMrgCtx0.useAltHpelIf[pu.geoMergeIdx0] = false;
#if JVET_AG0276_NLIC
            m_geoTmMrgCtx0.altLMFlag[pu.geoMergeIdx0] = m_geoMrgCtx.altLMFlag[pu.geoMergeIdx0];
            m_geoTmMrgCtx0.altLMParaNeighbours[pu.geoMergeIdx0] = m_geoMrgCtx.altLMParaNeighbours[pu.geoMergeIdx0];
#endif
#if INTER_LIC
            m_geoTmMrgCtx0.licFlags[pu.geoMergeIdx0] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            m_geoTmMrgCtx0.copyLICParamFromCtx(pu.geoMergeIdx0, m_geoMrgCtx, pu.geoMergeIdx0);
            m_geoTmMrgCtx0.licFlags[pu.geoMergeIdx0] = m_geoTmMrgCtx0.licInheritPara[pu.geoMergeIdx0];
#else
            m_geoTmMrgCtx0.setDefaultLICParamToCtx(pu.geoMergeIdx0);
#endif
#endif
#endif
#if MULTI_HYP_PRED
            m_geoTmMrgCtx0.addHypNeighbours[pu.geoMergeIdx0] = m_geoMrgCtx.addHypNeighbours[pu.geoMergeIdx0];
#endif
            m_geoTmMrgCtx0.interDirNeighbours[pu.geoMergeIdx0] = m_geoMrgCtx.interDirNeighbours[pu.geoMergeIdx0];
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].mv = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].mv;
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].mv = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].mv;
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].refIdx = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].refIdx;
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].refIdx = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].refIdx;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_AG0276_NLIC
            m_geoTmMrgCtx[GEO_TM_SHAPE_A] = m_geoTmMrgCtx0;
#else
            memcpy(&m_geoTmMrgCtx[GEO_TM_SHAPE_A], &m_geoTmMrgCtx0, sizeof(m_geoTmMrgCtx0));
#endif
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
            for (uint8_t tmType = GEO_TM_SHAPE_AL; tmType < GEO_NUM_TM_MV_CAND; ++tmType)
            {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
              if (tmType == GEO_TM_SHAPE_L || (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode() && tmType != g_geoTmShape[0][g_geoParams[g_gpmSplitDir[whIdx][pu.geoSplitDir]][0]]))
#else
              if (tmType == GEO_TM_SHAPE_L || (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode() && tmType != g_geoTmShape[0][g_geoParams[pu.geoSplitDir][0]]))
#endif
              {
                continue;
              }
              m_geoTmMrgCtx[tmType].setMergeInfo(pu, pu.geoMergeIdx0);
              pu.geoTmType = tmType;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              m_pcInterPred->deriveTMMv(pu, NULL, pu.geoMergeIdx0);
#else
              m_pcInterPred->deriveTMMv(pu);
#endif
#if JVET_AE0046_BI_GPM
              m_geoTmMrgCtx[tmType].interDirNeighbours[pu.geoMergeIdx0] = pu.interDir;
              m_geoTmMrgCtx[tmType].bcwIdx[pu.geoMergeIdx0] = (pu.interDir != 3) ? BCW_DEFAULT : m_geoMrgCtx.bcwIdx[pu.geoMergeIdx0];
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].refIdx = pu.refIdx[0];
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].refIdx = pu.refIdx[1];
#endif
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx0 << 1)    ].mv.set(pu.mv[0].getHor(), pu.mv[0].getVer());
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].mv.set(pu.mv[1].getHor(), pu.mv[1].getVer());
            }
#else
            m_geoTmMrgCtx0.setMergeInfo(pu, pu.geoMergeIdx0);
            pu.geoTmType = g_geoTmShape[0][g_geoParams[pu.geoSplitDir][0]];
            m_pcInterPred->deriveTMMv(pu);
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1)].mv.set(pu.mv[0].getHor(), pu.mv[0].getVer());
            m_geoTmMrgCtx0.mvFieldNeighbours[(pu.geoMergeIdx0 << 1) + 1].mv.set(pu.mv[1].getHor(), pu.mv[1].getVer());
#endif
#if JVET_AJ0274_GPM_AFFINE_TM
            }
#endif
          }
          if (pu.geoTmFlag1)
          {
#if JVET_AJ0274_GPM_AFFINE_TM
            if (pu.affineGPM[1])
            {
              for (int i = 0; i < 3; i++)
              {
                m_mvBufBDMVR[0][i].setZero();
                m_mvBufBDMVR[1][i].setZero();
              }
              int uiAffMergeCand = pu.geoMergeIdx1 - m_geoAffTmMrgCtx.m_indexOffset;
              m_geoAffTmMrgCtx.setAffMergeInfo(pu, pu.geoMergeIdx1);
              m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
              pu.bdmvrRefine = false;
              m_pcInterPred->setFillCurTplAboveARMC(false);
              m_pcInterPred->processTM4Affine(pu, m_geoAffTmMrgCtx, 0, false
#if JVET_AH0119_SUBBLOCK_TM
                                             , pu.cs->slice->getCheckLDB() ? true : false
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                                              , uiAffMergeCand
#endif
                                              );
              pu.cu->affine = false;
              m_pcInterPred->setFillCurTplAboveARMC(false);
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR[0][0];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR[0][1];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR[0][2];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR[1][0];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR[1][1];
              m_geoAffTmMrgCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR[1][2];
            }
            else
            {
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
            MergeCtx& m_geoTmMrgCtx1 = m_geoTmMrgCtx[GEO_TM_SHAPE_AL];
#endif
            m_geoTmMrgCtx1.numValidMergeCand = m_geoMrgCtx.numValidMergeCand;
            m_geoTmMrgCtx1.bcwIdx[pu.geoMergeIdx1] = BCW_DEFAULT;
            m_geoTmMrgCtx1.useAltHpelIf[pu.geoMergeIdx1] = false;
#if JVET_AG0276_NLIC
            m_geoTmMrgCtx1.altLMFlag[pu.geoMergeIdx1] = m_geoMrgCtx.altLMFlag[pu.geoMergeIdx1];
            m_geoTmMrgCtx1.altLMParaNeighbours[pu.geoMergeIdx1] = m_geoMrgCtx.altLMParaNeighbours[pu.geoMergeIdx1];
#endif
#if INTER_LIC
            m_geoTmMrgCtx1.licFlags[pu.geoMergeIdx1] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
            m_geoTmMrgCtx1.copyLICParamFromCtx(pu.geoMergeIdx1, m_geoMrgCtx, pu.geoMergeIdx1);
            m_geoTmMrgCtx1.licFlags[pu.geoMergeIdx1] = m_geoTmMrgCtx1.licInheritPara[pu.geoMergeIdx1];
#else
            m_geoTmMrgCtx1.setDefaultLICParamToCtx(pu.geoMergeIdx1);
#endif
#endif
#endif
#if MULTI_HYP_PRED
            m_geoTmMrgCtx1.addHypNeighbours[pu.geoMergeIdx1] = m_geoMrgCtx.addHypNeighbours[pu.geoMergeIdx1];
#endif
            m_geoTmMrgCtx1.interDirNeighbours[pu.geoMergeIdx1] = m_geoMrgCtx.interDirNeighbours[pu.geoMergeIdx1];
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].mv = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].mv;
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].mv = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].mv;
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].refIdx = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].refIdx;
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].refIdx = m_geoMrgCtx.mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].refIdx;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_AG0276_NLIC
            m_geoTmMrgCtx[GEO_TM_SHAPE_L] = m_geoTmMrgCtx1;
#else
            memcpy(&m_geoTmMrgCtx[GEO_TM_SHAPE_L], &m_geoTmMrgCtx1, sizeof(m_geoTmMrgCtx1));
#endif
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
            for (uint8_t tmType = GEO_TM_SHAPE_AL; tmType < GEO_NUM_TM_MV_CAND; ++tmType)
            {
#if JVET_AJ0107_GPM_SHAPE_ADAPT
              if (tmType == GEO_TM_SHAPE_A || (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode() && tmType != g_geoTmShape[1][g_geoParams[g_gpmSplitDir[whIdx][pu.geoSplitDir]][0]]))
#else
              if (tmType == GEO_TM_SHAPE_A || (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode() && tmType != g_geoTmShape[1][g_geoParams[pu.geoSplitDir][0]]))
#endif
              {
                continue;
              }
              m_geoTmMrgCtx[tmType].setMergeInfo(pu, pu.geoMergeIdx1);
              pu.geoTmType = tmType;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              m_pcInterPred->deriveTMMv(pu, NULL, pu.geoMergeIdx1);
#else
              m_pcInterPred->deriveTMMv(pu);
#endif
#if JVET_AE0046_BI_GPM
              m_geoTmMrgCtx[tmType].interDirNeighbours[pu.geoMergeIdx1] = pu.interDir;
              m_geoTmMrgCtx[tmType].bcwIdx[pu.geoMergeIdx1] = (pu.interDir != 3) ? BCW_DEFAULT : m_geoMrgCtx.bcwIdx[pu.geoMergeIdx1];
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].refIdx = pu.refIdx[0];
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].refIdx = pu.refIdx[1];
#endif
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx1 << 1)    ].mv.set(pu.mv[0].getHor(), pu.mv[0].getVer());
              m_geoTmMrgCtx[tmType].mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].mv.set(pu.mv[1].getHor(), pu.mv[1].getVer());
            }
#else
            m_geoTmMrgCtx1.setMergeInfo(pu, pu.geoMergeIdx1);
            pu.geoTmType = g_geoTmShape[1][g_geoParams[pu.geoSplitDir][0]];
            m_pcInterPred->deriveTMMv(pu);
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1)].mv.set(pu.mv[0].getHor(), pu.mv[0].getVer());
            m_geoTmMrgCtx1.mvFieldNeighbours[(pu.geoMergeIdx1 << 1) + 1].mv.set(pu.mv[1].getHor(), pu.mv[1].getVer());
#endif
          }
#if JVET_AJ0274_GPM_AFFINE_TM
          }
#endif
#endif
        }
        else
        {
        if( pu.cu->affine )
        {
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
          MergeCtx mrgCtxAll[2];
          for (int i = 0; i < 2; i++)
          {
            mrgCtxAll[i].numValidMergeCand = 0;
          }
          if (pu.cs->picHeader->getEnableTMVPFlag())
          {
            if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              && pu.cs->sps->getTMToolsEnableFlag()
#endif
              )
            {
              MergeCtx namvpMergeCandCtx[2];
              int nWidth = pu.lumaSize().width;
              int nHeight = pu.lumaSize().height;
              bool tplAvail = m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
              int poc0 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag()), pu.cu->slice->getColRefIdx())->getPOC();
              int poc1 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag2nd()), pu.cu->slice->getColRefIdx2nd())->getPOC();
              for (int col = 0; col < ((pu.cu->slice->getCheckLDC() || (poc0 == poc1)) ? 1 : 2); col++)
              {
                PU::getNonAdjacentMergeCandSubTMVP(pu, namvpMergeCandCtx[col], col);
                if (col == 0 && tplAvail)
                {
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroupSubTMVP(pu, namvpMergeCandCtx[col], 9);
                }
                else
                {
                  namvpMergeCandCtx[col].numValidMergeCand = std::min(9, namvpMergeCandCtx[col].numValidMergeCand);
                }
                PU::getInterMergeCandidatesSubTMVP(pu, mrgCtxAll[col], col, -1, &namvpMergeCandCtx[col]);
                if (col == 0 && tplAvail)
                {
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroupSubTMVP(pu, mrgCtxAll[col], pu.cs->sps->getMaxNumMergeCand());
                }
              }
            }
          }
#endif
          AffineMergeCtx affineMergeCtx;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
          RefineTmvpSbTmvpParams*    decSbTmvpParams    = new RefineTmvpSbTmvpParams   [SUB_TMVP_NUM];
          RefineTmvpSbTmvpSubParams* decSbTmvpSubParams = new RefineTmvpSbTmvpSubParams[SUB_TMVP_NUM * (pu.lumaSize().area() >> 4)];
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
          AffineMergeCtx affineRMVFCtx;
          AffineMergeCtx affineRMVFOriCtx;
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
          AffineMergeCtx affineTMVPCtx;
#endif
#if JVET_AG0276_NLIC
          AltLMAffineMergeCtx altAffineRMVFCtx;
          altAffineRMVFCtx.init();
#if JVET_AG0276_LIC_FLAG_SIGNALING
          AltLMAffineMergeCtx altBRAffineRMVFCtx;
          altBRAffineRMVFCtx.init();
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
          AffineMergeCtx affineRMVFCtxOppositeLic;
#endif
          if (pu.cs->sps->getSbTMVPEnabledFlag())
          {
            Size bufSize = g_miScaling.scale(pu.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
            for (int i = 0; i < SUB_TMVP_NUM; i++)
            {
              mrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
            }
#else
            mrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
            affineMergeCtx.mrgCtx = &mrgCtx;
          }
#if JVET_AL0160_SBSMVP
          if(pu.cs->sps->getSpatialMVPEnabledFlag())
          {
            Size bufSize = g_miScaling.scale(pu.lumaSize());
            for (int i = 0; i < NUM_SUB_SMVP; i++)
            {
              mrgCtx.subSpatialPuMvpMiBuf[i] = MotionBuf(m_subSpatialPuMiBuf[i], bufSize);
            }
            affineMergeCtx.mrgCtx = &mrgCtx;
          }
#endif
#if AFFINE_MMVD
#if JVET_W0090_ARMC_TM
          int affMrgIdx = pu.cs->sps->getUseAML() && (((pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE + 1)*ADAPTIVE_AFFINE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumAffineMergeCand()) || (pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE * ADAPTIVE_AFFINE_SUB_GROUP_SIZE + ADAPTIVE_AFFINE_SUB_GROUP_SIZE - 1 : pu.mergeIdx;
          if (pu.afMmvdFlag)
          {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
            affMrgIdx =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_ENHANCED_MMVD_EXTENSION
              !pu.cs->sps->getUseTMMMVD() ?
              ECM3_AF_MMVD_BASE_NUM :
#endif
              AF_MMVD_BASE_NUM;
#else
            affMrgIdx = pu.afMmvdBaseIdx;
#endif
          }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.cu->imv = IMV_OFF;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
          uint16_t addNumRMVF = 0;
          PU::getRMVFAffineCand(pu, affineRMVFCtx, affineRMVFOriCtx, m_pcInterPred, addNumRMVF
#if JVET_AG0276_NLIC
            , altAffineRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
            , altBRAffineRMVFCtx
#endif
#endif
          );
          if (pu.affBMMergeFlag)
          {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            PU::getBMAffineMergeCand(m_pcInterPred, pu, affineMergeCtx, affineRMVFOriCtx, -1);
#else
            PU::getBMAffineMergeCand(pu, affineMergeCtx, affineRMVFOriCtx, -1);
#endif
          }
          else
          {
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
            AffineMergeCtx affineTMVPCtx;
            AffineMergeCtx affineTMVPCtxtmp;
            int nWidth = pu.lumaSize().width;
            int nHeight = pu.lumaSize().height;
            bool tplAvail = m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
            bool isTmvpOptUsed = tplAvail && pu.cs->sps->getUseTemporalAffineOpt() && (nWidth * nHeight > 64) && !pu.cu->slice->getCheckLDC() ;

            if (isTmvpOptUsed && pu.cs->picHeader->getEnableTMVPFlag())
            {
              PU::getAffineTMVPMergeCand(pu, affineTMVPCtx);
              
              if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                && pu.cs->sps->getTMToolsEnableFlag()
#endif
                )
              {
                PredictionUnit pu2 = pu;
#if JVET_AG0164_AFFINE_GPM
                bool affineStored = pu.cu->affine;
                int  affTypeStored = pu.cu->affineType;
                int  bcwStored = pu.cu->bcwIdx;
#if INTER_LIC
                bool licStored = pu.cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                bool obmcStored = pu.cu->obmcFlag;
#endif
#endif
                m_pcInterPred->adjustAffineMergeCandidatesOneGroup(pu2, affineTMVPCtx, affineTMVPCtx.numValidMergeCand, affineTMVPCtxtmp);
#if JVET_AG0164_AFFINE_GPM
                pu.cu->affine = affineStored;
                pu.cu->affineType = affTypeStored;
                pu.cu->bcwIdx = bcwStored;
#if INTER_LIC
                pu.cu->licFlag = licStored;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                pu.cu->obmcFlag = obmcStored;
#endif
#endif
              }
            }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            PU::getAffineMergeCand(m_pcInterPred, decSbTmvpParams, decSbTmvpSubParams, pu, affineMergeCtx
#else
            PU::getAffineMergeCand(pu, affineMergeCtx
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                               , mrgCtxAll
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
                               , m_pcInterPred
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
              , affineRMVFCtx
              , affineRMVFOriCtx
              , addNumRMVF
#endif
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
              , isTmvpOptUsed ? &affineTMVPCtx : NULL
#endif
              , affMrgIdx
              , pu.afMmvdFlag
#if JVET_Z0139_NA_AFF
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
#if JVET_AG0135_AFFINE_CIIP
              , (cu.slice->getTLayer() < 4 && PU::checkAffineTMCondition(pu)) ? false : (pu.mergeIdx == 0 && !pu.affBMMergeFlag)
#else
              , (pu.mergeIdx == 0 && !pu.affBMMergeFlag)
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
                && !pu.affineOppositeLic
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              , (pu.affineOppositeLic || pu.afMmvdFlag)
#endif
#endif
#else
              , pu.mergeIdx == 0
#endif
#endif
            );
#if JVET_AG0276_LIC_FLAG_SIGNALING
            if (pu.affineOppositeLic && !pu.afMmvdFlag)
            {
              int cnt = 0;
              affineRMVFCtxOppositeLic = affineMergeCtx;
              affineMergeCtx.numValidMergeCand = 0;
              for (int i = 0; i < affineRMVFCtxOppositeLic.numValidMergeCand; i++)
              {
#if !JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                if (affineRMVFCtxOppositeLic.mergeType[i] == MRG_TYPE_DEFAULT_N)
                {
#endif
                  for (int mvNum = 0; mvNum < 3; mvNum++)
                  {
                    affineMergeCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum] = affineRMVFCtxOppositeLic.mvFieldNeighbours[(i << 1) + 0][mvNum];
                    affineMergeCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum] = affineRMVFCtxOppositeLic.mvFieldNeighbours[(i << 1) + 1][mvNum];
                  }
                  affineMergeCtx.interDirNeighbours[cnt] = affineRMVFCtxOppositeLic.interDirNeighbours[i];
                  affineMergeCtx.affineType[cnt] = affineRMVFCtxOppositeLic.affineType[i];
                  affineMergeCtx.mergeType[cnt] = affineRMVFCtxOppositeLic.mergeType[i];
                  affineMergeCtx.bcwIdx[cnt] = affineRMVFCtxOppositeLic.bcwIdx[i];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  affineMergeCtx.colIdx[cnt] = affineRMVFCtxOppositeLic.colIdx[i];
#endif
#if JVET_AG0276_NLIC
                  affineMergeCtx.altLMFlag[cnt] = affineRMVFCtxOppositeLic.altLMFlag[i];
                  affineMergeCtx.altLMParaNeighbours[cnt] = affineRMVFCtxOppositeLic.altLMParaNeighbours[i];
#endif
#if INTER_LIC                                                   
                  affineMergeCtx.licFlags[cnt] = !affineRMVFCtxOppositeLic.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                  affineMergeCtx.licInheritPara[cnt] = false;
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                  affineMergeCtx.obmcFlags[cnt] = affineRMVFCtxOppositeLic.obmcFlags[i];
#endif
                  affineMergeCtx.candCost[cnt] = affineRMVFCtxOppositeLic.candCost[i];
                  cnt++;
                  affineMergeCtx.numValidMergeCand++;
#if !JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                }
                else
                {
                  continue;
                }
#endif
              }
              affineMergeCtx.numAffCandToTestEnc = std::min(affineMergeCtx.numValidMergeCand, affineRMVFCtxOppositeLic.numAffCandToTestEnc);
              affineMergeCtx.maxNumMergeCand = affineMergeCtx.numValidMergeCand;
            }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
          }
#endif
#else
          PU::getAffineMergeCand(pu, affineMergeCtx, (pu.afMmvdFlag ? pu.afMmvdBaseIdx : pu.mergeIdx), pu.afMmvdFlag);
#endif

          if (pu.afMmvdFlag)
          {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            pu.mergeIdx = pu.afMmvdBaseIdx;
#else
            pu.mergeIdx = PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtx, pu.afMmvdBaseIdx);
#endif
            CHECK(pu.mergeIdx >= pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand(), "Affine MMVD mode doesn't have a valid base candidate!");
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
            pu.interDir = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
            pu.cu->affineType = affineMergeCtx.affineType[pu.mergeIdx];
            pu.cu->bcwIdx = affineMergeCtx.bcwIdx[pu.mergeIdx];
#if JVET_AG0276_NLIC
            pu.cu->altLMFlag = affineMergeCtx.altLMFlag[pu.mergeFlag];
            pu.cu->altLMParaUnit = affineMergeCtx.altLMParaNeighbours[pu.mergeFlag];
#endif
#if INTER_LIC
            pu.cu->licFlag = affineMergeCtx.licFlags[pu.mergeIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG && JVET_AG0164_AFFINE_GPM
            affineMergeCtx.setLICParamToPu(pu, pu.mergeIdx, pu.cu->licFlag);
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            pu.cu->obmcFlag = affineMergeCtx.obmcFlags[pu.mergeIdx];
#endif
#if ENABLE_INTER_TEMPLATE_MATCHING && JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            pu.colIdx = affineMergeCtx.colIdx[pu.mergeIdx];
#endif
            pu.mergeType = affineMergeCtx.mergeType[pu.mergeIdx];
            {
              for (int i = 0; i < 2; ++i)
              {
                if (pu.cs->slice->getNumRefIdx(RefPicList(i)) > 0)
                {
                  MvField* mvField = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i];
                  pu.mvpIdx[i] = 0;
                  pu.mvpNum[i] = 0;
                  pu.mvd[i] = Mv();
                  pu.refIdx[i] = mvField[0].refIdx;
                  pu.mvAffi[i][0] = mvField[0].mv;
                  pu.mvAffi[i][1] = mvField[1].mv;
                  pu.mvAffi[i][2] = mvField[2].mv;
                }
              }
            }
#if JVET_AG0276_LIC_BDOF_BDMVR
            if (PU::checkBDMVRCondition4Aff(pu))
#else
            if (PU::checkBDMVRCondition(pu))
#endif
            {
              if (PU::checkBDMVR4Affine(pu))
              {
                m_pcInterPred->processBDMVR4Affine(pu
#if JVET_AH0119_SUBBLOCK_TM
                  , affineMergeCtx, false
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                  , pu.mergeIdx
#endif
                );
              }
              for (int i = 0; i < 2; ++i)
              {
                affineMergeCtx.affineType[pu.mergeIdx] = (EAffineModel)pu.cu->affineType;
                affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i][0].mv = pu.mvAffi[i][0];
                affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i][1].mv = pu.mvAffi[i][1];
                affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i][2].mv = pu.mvAffi[i][2];
              }
            }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
            if(pu.cs->sps->getUseTMMMVD())
            {
#endif
            uint32_t affMmvdLUT[AF_MMVD_NUM];
            uint32_t tempVal = pu.afMmvdMergeIdx;
            m_pcInterPred->sortAffineMergeCandidates(pu, affineMergeCtx, affMmvdLUT, tempVal );
            pu.afMmvdMergeIdx = tempVal;
            int uiMergeCandTemp = affMmvdLUT[pu.afMmvdMergeIdx];
            int baseIdx = (int)uiMergeCandTemp / AF_MMVD_MAX_REFINE_NUM;
            int stepIdx = (int)uiMergeCandTemp - baseIdx * AF_MMVD_MAX_REFINE_NUM;
            int dirIdx  = stepIdx % AF_MMVD_OFFSET_DIR;
            stepIdx = stepIdx / AF_MMVD_OFFSET_DIR;
            pu.afMmvdDir      = (uint8_t)dirIdx;
            pu.afMmvdStep     = (uint8_t)stepIdx;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
            }
#endif
#endif
            PU::getAfMmvdMvf(pu, affineMergeCtx, affineMergeCtx.mvFieldNeighbours + (pu.mergeIdx << 1), pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
          }
#if JVET_W0090_ARMC_TM
          else
          {
#if JVET_AG0276_NLIC
            if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              && pu.cs->sps->getTMToolsEnableFlag()
#endif
               )
            {
              if (pu.affBMMergeFlag)
              {
                m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, pu.mergeIdx);
              }
              else
              {
                if (affineMergeCtx.numValidMergeCand > 1)
                {
                  if (pu.cs->sps->getUseAffAltLM() && !CU::isTLCond(*pu.cu))
                  {
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  if (!pu.affineOppositeLic)
                  {
#endif
                    AltLMAffineMergeCtx altLMAffMrgCtx;
                    PU::getAltLMAffineMergeCand(pu, altLMAffMrgCtx);
                    m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, altLMAffMrgCtx, altAffineRMVFCtx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  }
                  else
                  {
                    AltLMAffineMergeCtx altLMBRAffMrgCtx;
                    PU::getAltLMBRAffineMergeCand(pu, altLMBRAffMrgCtx);
                    m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, altLMBRAffMrgCtx, altBRAffineRMVFCtx);
                  }
#endif
                  }
                  else
                  {
                    m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, pu.mergeIdx);
                  }
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
                  if (pu.affineOppositeLic)
                  {
                    affineMergeCtx.numValidMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineOppositeLicMergeCand();
                    affineMergeCtx.maxNumMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineOppositeLicMergeCand();
                  }
                  else
                  {
#endif
                    affineMergeCtx.numValidMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
                    affineMergeCtx.maxNumMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
#if JVET_AK0095_ENHANCED_AFFINE_CANDIDATE
                  }
                  int nWidth = pu.lumaSize().width;
                  int nHeight = pu.lumaSize().height;
                  bool tplAvail = m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
                  bool isConstrutedUsed = tplAvail && pu.cs->sps->getUseSyntheticAffine() && (pu.cu->slice->getTLayer() < 5) && !pu.cu->slice->getCheckLDB();
                  if (isConstrutedUsed)
                  {
                    AffineMergeCtx affineTmpMergeCtx;
                    for (int i = 0; i < AFFINE_MRG_MAX_NUM_CANDS_ALL; i++)
                    {
                      for (int mvNum = 0; mvNum < 3; mvNum++)
                      {
                        affineTmpMergeCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
                        affineTmpMergeCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
                      }
                      affineTmpMergeCtx.interDirNeighbours[i] = 0;
                      affineTmpMergeCtx.affineType[i] = AFFINEMODEL_4PARAM;
                      affineTmpMergeCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
                      affineTmpMergeCtx.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
                      affineTmpMergeCtx.altLMFlag[i] = false;
                      affineTmpMergeCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
                      affineTmpMergeCtx.licFlags[i] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                      affineTmpMergeCtx.setDefaultLICParamToCtx(i);
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                      affineTmpMergeCtx.obmcFlags[i] = true;
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
                      affineTmpMergeCtx.candCost[i] = MAX_UINT64;
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                      affineTmpMergeCtx.colIdx[i] = 0;
#endif
                    }
                    affineTmpMergeCtx.numValidMergeCand = 0;
                    for (int i = 0; i < affineMergeCtx.numValidMergeCand; i++)
                    {
                      if (affineMergeCtx.mergeType[i] == MRG_TYPE_SUBPU_ATMVP || affineMergeCtx.altLMFlag[i] || affineMergeCtx.licInheritPara[i])
                      {
                        continue;
                      }
                      for (int mvNum = 0; mvNum < 3; mvNum++)
                      {
                        affineTmpMergeCtx.mvFieldNeighbours[(affineTmpMergeCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(affineMergeCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].mv, affineMergeCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].refIdx);
                        affineTmpMergeCtx.mvFieldNeighbours[(affineTmpMergeCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(affineMergeCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].mv, affineMergeCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].refIdx);
                      }
                      affineTmpMergeCtx.interDirNeighbours[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.interDirNeighbours[i];
                      affineTmpMergeCtx.affineType[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.affineType[i];
                      affineTmpMergeCtx.mergeType[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.mergeType[i];
                      affineTmpMergeCtx.bcwIdx[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.bcwIdx[i];
#if JVET_AG0276_NLIC
                      affineTmpMergeCtx.altLMFlag[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.altLMFlag[i];
#endif
#if INTER_LIC
                      affineTmpMergeCtx.licFlags[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                      affineTmpMergeCtx.obmcFlags[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.obmcFlags[i];
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                      affineTmpMergeCtx.colIdx[affineTmpMergeCtx.numValidMergeCand] = affineMergeCtx.colIdx[i];
#endif
                      affineTmpMergeCtx.numValidMergeCand++;
                    }
                    const int PAIR_CONSTRUCT_NUM = 12;
                    const int preDefinedPairs[PAIR_CONSTRUCT_NUM][2] = { {0, 1}, {1, 0}, {0, 2}, {2, 0}, {1, 2}, {2, 1}, {0, 3}, {3, 0}, {1, 3}, {3, 1}, {2, 3}, {3, 2} };
                    int counter = 0;
                    for (int i = 0; i < PAIR_CONSTRUCT_NUM; i++)
                    {
                      if (affineTmpMergeCtx.numValidMergeCand > preDefinedPairs[i][0] && affineTmpMergeCtx.numValidMergeCand > preDefinedPairs[i][1] &&
                        ((affineTmpMergeCtx.interDirNeighbours[preDefinedPairs[i][0]] & 1) && (affineTmpMergeCtx.interDirNeighbours[preDefinedPairs[i][1]] & 2)))
                      {
                        for (int mvNum = 0; mvNum < 3; mvNum++)
                        {
                          affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][0] << 1) + 0][mvNum].mv,
                            affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][0] << 1) + 0][mvNum].refIdx);
                          affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][1] << 1) + 1][mvNum].mv,
                            affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][1] << 1) + 1][mvNum].refIdx);
                        }
                        affineMergeCtx.mergeType[affineMergeCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
                        affineMergeCtx.interDirNeighbours[affineMergeCtx.numValidMergeCand] = 3;
                        CHECK(((affineTmpMergeCtx.interDirNeighbours[preDefinedPairs[i][0]] & 1) + (affineTmpMergeCtx.interDirNeighbours[preDefinedPairs[i][1]] & 2)) != 3, "Wrong interDir");
                        affineMergeCtx.affineType[affineMergeCtx.numValidMergeCand] = (affineTmpMergeCtx.affineType[preDefinedPairs[i][0]] == AFFINEMODEL_4PARAM && affineTmpMergeCtx.affineType[preDefinedPairs[i][1]] == AFFINEMODEL_4PARAM) ?
                          AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;
                        if ((affineTmpMergeCtx.affineType[preDefinedPairs[i][0]] == AFFINEMODEL_4PARAM && affineTmpMergeCtx.affineType[preDefinedPairs[i][1]] == AFFINEMODEL_6PARAM) ||
                          (affineTmpMergeCtx.affineType[preDefinedPairs[i][0]] == AFFINEMODEL_6PARAM && affineTmpMergeCtx.affineType[preDefinedPairs[i][1]] == AFFINEMODEL_4PARAM))
                        {
                          Mv mvLT, mvRT, mvLB;
                          int curW = pu.Y().width;
                          int curH = pu.Y().height;
                          if (affineTmpMergeCtx.affineType[preDefinedPairs[i][0]] == AFFINEMODEL_4PARAM && affineTmpMergeCtx.affineType[preDefinedPairs[i][1]] == AFFINEMODEL_6PARAM)
                          {
                            mvLT = affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 0][0].mv;
                            mvRT = affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 0][1].mv;
                          }
                          else
                          {
                            mvLT = affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 1][0].mv;
                            mvRT = affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 1][1].mv;
                          }
                          int shift = MAX_CU_DEPTH;
                          int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
                          iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(curW));
                          iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(curH));
                          iDMvVerX = -iDMvHorY;
                          iDMvVerY = iDMvHorX;
                          int iMvScaleHor = mvLT.getHor() << shift;
                          int iMvScaleVer = mvLT.getVer() << shift;
                          int horTmp, verTmp;
                          // v2
                          horTmp = iMvScaleHor + iDMvHorX * curW + iDMvVerX * curH;
                          verTmp = iMvScaleVer + iDMvHorY * curW + iDMvVerY * curH;
                          roundAffineMv(horTmp, verTmp, shift);
                          mvLB.hor = horTmp;
                          mvLB.ver = verTmp;
                          mvLB.clipToStorageBitDepth();
                          if (affineTmpMergeCtx.affineType[preDefinedPairs[i][0]] == AFFINEMODEL_4PARAM && affineTmpMergeCtx.affineType[preDefinedPairs[i][1]] == AFFINEMODEL_6PARAM)
                          {
                            affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 0][2].setMvField(mvLB, affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][0] << 1) + 0][2].refIdx);
                          }
                          else
                          {
                            affineMergeCtx.mvFieldNeighbours[(affineMergeCtx.numValidMergeCand << 1) + 1][2].setMvField(mvLB, affineTmpMergeCtx.mvFieldNeighbours[(preDefinedPairs[i][1] << 1) + 1][2].refIdx);
                          }
                        }
#if INTER_LIC
                        affineMergeCtx.licFlags[affineMergeCtx.numValidMergeCand] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG && JVET_AG0164_AFFINE_GPM
                        affineMergeCtx.setDefaultLICParamToCtx(affineMergeCtx.numValidMergeCand);
#endif
#endif
                        affineMergeCtx.bcwIdx[affineMergeCtx.numValidMergeCand] = BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
                        affineMergeCtx.obmcFlags[affineMergeCtx.numValidMergeCand] = true;
#endif
                        affineMergeCtx.m_isConstructed[affineMergeCtx.numValidMergeCand] = true;
                        if (PU::checkLastAffineMergeCandRedundancy(pu, affineMergeCtx))
                        {
                          counter++;
                          affineMergeCtx.numValidMergeCand++;
                          if (counter >= CONSTRUCTED_CANDIDATE_MAX_NUM)
                          {
                            break;
                          }
                        }
                      }
                    }
                    affineMergeCtx.maxNumMergeCand = affineMergeCtx.numValidMergeCand;
                    m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, pu.mergeIdx, affineMergeCtx.numValidMergeCand);
                    affineMergeCtx.numValidMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
                    affineMergeCtx.maxNumMergeCand = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
                  }
#endif
                }
              }
            }
#else
            if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                && pu.cs->sps->getTMToolsEnableFlag()
#endif
              )
#if JVET_Z0139_NA_AFF
            if (affineMergeCtx.numValidMergeCand > 1)
#endif
            {
              m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, pu.mergeIdx);
            }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            if (affineMergeCtx.mergeType[pu.mergeIdx] == MRG_TYPE_SUBPU_ATMVP && affineMergeCtx.colIdx[pu.mergeIdx] >= 0
                && (pu.cu->slice->getCheckLDC() == true || pu.cu->slice->getSPS()->getRefineTmvpCfgIdx() < 2))
            {
              PU::refineTmvpGetInterMergeSubPuMvpCand(m_pcInterPred, pu, *affineMergeCtx.mrgCtx,
                  decSbTmvpParams, decSbTmvpSubParams, affineMergeCtx.colIdx[pu.mergeIdx]);
            }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
            if (pu.affBMMergeFlag)
            {
              affineMergeCtx.numValidMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
              affineMergeCtx.maxNumMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
            }
#endif
          }
#endif
#else
#if JVET_W0090_ARMC_TM
          int affMrgIdx = pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
            && (((pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE + 1)*ADAPTIVE_AFFINE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumAffineMergeCand()) || (pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_AFFINE_SUB_GROUP_SIZE * ADAPTIVE_AFFINE_SUB_GROUP_SIZE + ADAPTIVE_AFFINE_SUB_GROUP_SIZE - 1 : pu.mergeIdx;
#if !JVET_Z0139_NA_AFF
          PU::getAffineMergeCand(pu, affineMergeCtx, affMrgIdx);
#else
          PU::getAffineMergeCand(pu, affineMergeCtx, 
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
            m_pcInterPred,
#endif
            affMrgIdx, pu.mergeIdx == 0);
          if (affineMergeCtx.numValidMergeCand > 1)
#endif
          if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
             && pu.cs->sps->getTMToolsEnableFlag()
#endif
            )
          {
            m_pcInterPred->adjustAffineMergeCandidates(pu, affineMergeCtx, pu.mergeIdx);
          }
#else
          PU::getAffineMergeCand( pu, affineMergeCtx, pu.mergeIdx );
#endif
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
          int validNum = 0;
          bool isAdditional = false;
          int mergeIdx = pu.mergeIdx;
          if (pu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
            && PU::checkAffineTMCondition(pu)
             )
          {
#if JVET_AG0276_LIC_FLAG_SIGNALING
            for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < (pu.affineOppositeLic ? int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) : affineMergeCtx.numValidMergeCand); uiAffMergeCand++)
#else
            for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
#endif
            {
              if (affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv == Mv(0, 0) &&
                affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv == Mv(0, 0) &&
                affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv == Mv(0, 0)
                && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
              {
                validNum = uiAffMergeCand;
                break;
              }
#if JVET_AG0276_LIC_FLAG_SIGNALING
              if (uiAffMergeCand == (pu.affineOppositeLic ? (int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) - 1) : pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand() - 1))
              {
                validNum = pu.affineOppositeLic ? int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) : pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
              }
#else
              if (uiAffMergeCand == pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand() - 1)
              {
                validNum = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
              }
#endif
            }
            if (!pu.afMmvdFlag)
            {
              if (pu.mergeIdx > validNum - 1)
              {
                isAdditional = true;
                int value = pu.mergeIdx - validNum;
                for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < validNum; uiAffMergeCand++)
                {
#if JVET_AG0276_NLIC
                  if (!affineMergeCtx.altLMFlag[uiAffMergeCand] && affineMergeCtx.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#else
                  if (affineMergeCtx.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#endif
                  {
                    value--;
                    if (value == -1)
                    {
                      pu.mergeIdx = uiAffMergeCand;
                      break;
                    }
                  }
                }
                if (pu.mergeIdx == mergeIdx)
                {
                  isAdditional = false;
                }
              }
            }
          }
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
          delete [] decSbTmvpParams;
          delete [] decSbTmvpSubParams;
#endif
          pu.interDir = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
          pu.cu->affineType = affineMergeCtx.affineType[pu.mergeIdx];
          pu.cu->bcwIdx = affineMergeCtx.bcwIdx[pu.mergeIdx];
#if JVET_AG0276_NLIC
          pu.cu->altLMFlag = affineMergeCtx.altLMFlag[pu.mergeIdx];
          pu.cu->altLMParaUnit = affineMergeCtx.altLMParaNeighbours[pu.mergeIdx];
#endif
#if INTER_LIC
          pu.cu->licFlag = affineMergeCtx.licFlags[pu.mergeIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG && JVET_AG0164_AFFINE_GPM
          affineMergeCtx.setLICParamToPu(pu, pu.mergeIdx, pu.cu->licFlag);
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.colIdx = affineMergeCtx.colIdx[pu.mergeIdx];
#endif
          pu.mergeType = affineMergeCtx.mergeType[pu.mergeIdx];
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          pu.cu->obmcFlag = (addHypData.size() > 0 && pu.mergeType != MRG_TYPE_SUBPU_ATMVP) ? false : affineMergeCtx.obmcFlags[pu.mergeIdx];
#endif
          if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
          {
            pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + 1][0].refIdx;
          }
          else
          {
            for (int i = 0; i < 2; ++i)
            {
              if (pu.cs->slice->getNumRefIdx(RefPicList(i)) > 0)
              {
                MvField* mvField = affineMergeCtx.mvFieldNeighbours[(pu.mergeIdx << 1) + i];
                pu.mvpIdx[i] = 0;
                pu.mvpNum[i] = 0;
                pu.mvd[i] = Mv();
                pu.refIdx[i] = mvField[0].refIdx;
                pu.mvAffi[i][0] = mvField[0].mv;
                pu.mvAffi[i][1] = mvField[1].mv;
                pu.mvAffi[i][2] = mvField[2].mv;
              }
            }
          }
#if JVET_AB0112_AFFINE_DMVR
#if JVET_AG0135_AFFINE_CIIP
#if JVET_AG0276_LIC_BDOF_BDMVR
          if (!pu.afMmvdFlag&&pu.mergeType != MRG_TYPE_SUBPU_ATMVP && (pu.ciipAffine ? PU::checkBDMVRConditionCIIPAffine(pu) : PU::checkBDMVRCondition4Aff(pu)))
#else
          if (!pu.afMmvdFlag&&pu.mergeType != MRG_TYPE_SUBPU_ATMVP && (pu.ciipAffine ? PU::checkBDMVRConditionCIIPAffine(pu) : PU::checkBDMVRCondition(pu)))
#endif
#else
#if JVET_AG0276_LIC_BDOF_BDMVR
          if (!pu.afMmvdFlag&&pu.mergeType != MRG_TYPE_SUBPU_ATMVP && PU::checkBDMVRCondition4Aff(pu))
#else
          if (!pu.afMmvdFlag&&pu.mergeType != MRG_TYPE_SUBPU_ATMVP && PU::checkBDMVRCondition(pu))
#endif
#endif
          {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
            m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
#endif
            pu.bdmvrRefine = false;
            if (!affineMergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu))
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
              || pu.affBMMergeFlag
#endif
              )
            {
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
              Mv refinedMvL0[2][3];
              Mv refinedMvL1[2][3];
              EAffineModel affTypeL0;
              EAffineModel affTypeL1;
              if (pu.affBMMergeFlag && PU::checkBDMVR4Affine(pu))
              {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                m_pcInterPred->processBDMVR4AdaptiveAffine(pu, refinedMvL0, refinedMvL1, affTypeL0, affTypeL1, pu.mergeIdx);
#else
                m_pcInterPred->processBDMVR4AdaptiveAffine(pu, refinedMvL0, refinedMvL1, affTypeL0, affTypeL1);
#endif
              }
              else
              {
#endif
#if JVET_AH0119_SUBBLOCK_TM
                for (int i = 0; i < 3; i++)
                {
                  m_mvBufBDMVR[0][i].setZero();
                  m_mvBufBDMVR[1][i].setZero();
                }
                m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
                if (PU::checkBDMVR4Affine(pu))
#endif

                m_pcInterPred->processBDMVR4Affine(pu
#if JVET_AH0119_SUBBLOCK_TM
                  , affineMergeCtx
                  , !pu.affineOppositeLic
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                  , pu.mergeIdx
#endif
                );
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
              }
#endif
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
              pu.mvAffi[0][0] += m_mvBufBDMVR[0][0];
              pu.mvAffi[0][1] += m_mvBufBDMVR[0][0];
              pu.mvAffi[0][2] += m_mvBufBDMVR[0][0];
              pu.mvAffi[1][0] += m_mvBufBDMVR[1][0];
              pu.mvAffi[1][1] += m_mvBufBDMVR[1][0];
              pu.mvAffi[1][2] += m_mvBufBDMVR[1][0];
#endif
            }
          }
#endif
#if JVET_AH0119_SUBBLOCK_TM
          else if (pu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif          
#if JVET_AG0276_NLIC
            && (pu.cs->sps->getUseAffAltLMTM() || !pu.cu->altLMFlag)
#endif
            && !pu.afMmvdFlag&&pu.mergeType != MRG_TYPE_SUBPU_ATMVP && pu.interDir == 3 && PU::checkBDMVR4Affine(pu) && !pu.affBMMergeFlag &&
#if JVET_AG0276_NLIC
            ((!pu.cu->altLMFlag && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), false)) ||
            (pu.cu->altLMFlag && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), true)))
#else
            !affineMergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu))
#endif
            && !pu.affineOppositeLic
            )
          {
            for (int i = 0; i < 3; i++)
            {
              m_mvBufBDMVR[0][i].setZero();
              m_mvBufBDMVR[1][i].setZero();
            }
            m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
            pu.bdmvrRefine = false;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
            m_pcInterPred->processTM4Affine(pu, affineMergeCtx, -1, false, true, pu.mergeIdx);//bi
#else
            m_pcInterPred->processTM4Affine(pu, affineMergeCtx, -1, false);//bi
#endif
            pu.mvAffi[0][0] += m_mvBufBDMVR[0][0];
            pu.mvAffi[0][1] += m_mvBufBDMVR[0][1];
            pu.mvAffi[0][2] += m_mvBufBDMVR[0][2];
            pu.mvAffi[1][0] += m_mvBufBDMVR[1][0];
            pu.mvAffi[1][1] += m_mvBufBDMVR[1][1];
            pu.mvAffi[1][2] += m_mvBufBDMVR[1][2];
          }
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
          if (pu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
#if JVET_AG0276_NLIC
            && (pu.cs->sps->getUseAffAltLMTM() || !pu.cu->altLMFlag)
#endif
            && !pu.afMmvdFlag && pu.mergeType != MRG_TYPE_SUBPU_ATMVP && pu.interDir != 3 && !((pu.mergeIdx > validNum - 1) && !isAdditional) && PU::checkAffineTMCondition(pu)
             )
          {
            for (int i = 0; i < 3; i++)
            {
              m_mvBufBDMVR[0][i].setZero();
              m_mvBufBDMVR[1][i].setZero();
            }
            m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
            pu.bdmvrRefine = false;

#if JVET_AG0276_NLIC
            if ((!pu.cu->altLMFlag && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), false)) ||
                (pu.cu->altLMFlag && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), true)))
#else
            if (!affineMergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
#endif
            {
              m_pcInterPred->processTM4Affine(pu, affineMergeCtx, isAdditional ? 0 : -1, false
#if JVET_AH0119_SUBBLOCK_TM              
                , !pu.affineOppositeLic
#endif
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                , pu.mergeIdx
#endif
              );//uni
#if JVET_AH0119_SUBBLOCK_TM
              pu.mvAffi[0][0] += m_mvBufBDMVR[0][0];
              pu.mvAffi[0][1] += m_mvBufBDMVR[0][1];
              pu.mvAffi[0][2] += m_mvBufBDMVR[0][2];
              pu.mvAffi[1][0] += m_mvBufBDMVR[1][0];
              pu.mvAffi[1][1] += m_mvBufBDMVR[1][1];
              pu.mvAffi[1][2] += m_mvBufBDMVR[1][2];
#else
              pu.mvAffi[0][0] += m_mvBufBDMVR[0][0];
              pu.mvAffi[0][1] += m_mvBufBDMVR[0][0];
              pu.mvAffi[0][2] += m_mvBufBDMVR[0][0];
              pu.mvAffi[1][0] += m_mvBufBDMVR[1][0];
              pu.mvAffi[1][1] += m_mvBufBDMVR[1][0];
              pu.mvAffi[1][2] += m_mvBufBDMVR[1][0];
#endif
            }
          }
#endif
#if JVET_AH0119_SUBBLOCK_TM
          if (pu.cs->sps->getUseSbTmvpTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
            &&  PU::checkAffineTMCondition(pu)
            &&  pu.mergeType== MRG_TYPE_SUBPU_ATMVP)
          {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
            m_pcInterPred->processTM4SbTmvp(pu, affineMergeCtx, -1, false, pu.mergeIdx);
#else
            m_pcInterPred->processTM4SbTmvp(pu, affineMergeCtx, -1, false);
#endif
          }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
          if (PU::checkDoAffineBdofRefine(pu, m_pcInterPred))
          {
            pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
            m_pcInterPred->setDoAffineSubPuBdof(false);
            m_pcInterPred->setBdofSubPuMvBuf(m_mvBufDecAffineBDOF);
            if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
            {
              int bioSubPuIdx = 0;
              const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
              for (int yy = 0; yy < pu.lumaSize().height; yy += 4)
              {
                for (int xx = 0; xx < pu.lumaSize().width; xx += 4)
                {
                  m_mvBufDecAffineBDOF[bioSubPuIdx].setZero();
                  bioSubPuIdx++;
                }
                bioSubPuIdx += bioSubPuStrideIncr;
              }
            }
          }
#endif
          PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , pu.colIdx
#endif
          );
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
          if (pu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
            && PU::checkAffineTMCondition(pu))
          {
            pu.mergeIdx = mergeIdx;
          }
#endif
        }
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
        else if (pu.ciipFlag && pu.tmMergeFlag)
        {
          int storeMrgIdx = pu.mergeIdx;
#if JVET_AG0135_AFFINE_CIIP
          pu.tmMergeFlag = true;
#else
          pu.tmMergeFlag = false;
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
          PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, CIIP_TM_MRG_MAX_NUM_CANDS - 1);
#else
          PU::getInterMergeCandidates(pu, mrgCtx, 0, CIIP_TM_MRG_MAX_NUM_CANDS - 1);
#endif
          mrgCtx.numValidMergeCand = int(pu.cs->sps->getMaxNumCiipTMMergeCand());
          pu.tmMergeFlag = true;
          for (uint32_t uiMergeCand = 0; uiMergeCand < CIIP_TM_MRG_MAX_NUM_CANDS; uiMergeCand++)
          {
            mrgCtx.setMergeInfo(pu, uiMergeCand);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
            m_pcInterPred->deriveTMMv(pu, NULL, uiMergeCand);
#else
            m_pcInterPred->deriveTMMv(pu);
#endif
            // Store refined motion back to ciipTmMrgCtx
            mrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
            mrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // Bcw may change, because bi may be reduced to uni by deriveTMMv(pu)
            mrgCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField(pu.mv[0], pu.refIdx[0]);
            mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(pu.mv[1], pu.refIdx[1]);
            if (pu.interDir == 1)
            {
              mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(Mv(), NOT_VALID);
            }
            if (pu.interDir == 2)
            {
              mrgCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField(Mv(), NOT_VALID);
            }
          }
#if JVET_W0090_ARMC_TM
          if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
             && pu.cs->sps->getTMToolsEnableFlag()
#endif
            )
          {
             m_pcInterPred->adjustInterMergeCandidates(pu, mrgCtx, CIIP_TM_MRG_MAX_NUM_CANDS - 1);
          }
#endif

          mrgCtx.setMergeInfo(pu, storeMrgIdx);
          pu.bdmvrRefine = false;
          PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , pu.colIdx
#endif
          );
        }
#endif
        else
        {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
#if TM_MRG
          bool applyBDMVR4TM[TM_MRG_MAX_NUM_INIT_CANDS] = { false };
          bool tmMergeRefinedMotion = false;
#endif
          bool admvrRefinedMotion = false;
#endif
          if (CU::isIBC(*pu.cu))
          {
#if JVET_AA0061_IBC_MBVD
            if (pu.ibcMbvdMergeFlag)
            {
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
              const int mbvdsPerBase = pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch() ? IBC_MBVD_SIZE_ENC : IBC_MBVD_MAX_REFINE_NUM;
#else
              const int mbvdsPerBase = IBC_MBVD_MAX_REFINE_NUM;
#endif
              int fPosIBCBaseIdx = pu.ibcMbvdMergeIdx / mbvdsPerBase;
#if JVET_AE0169_BIPREDICTIVE_IBC
              if (pu.interDir == 3)
              {
                fPosIBCBaseIdx = std::max(fPosIBCBaseIdx, (pu.ibcMergeIdx1 >= IBC_MRG_MAX_NUM_CANDS) ? (pu.ibcMergeIdx1-IBC_MRG_MAX_NUM_CANDS) / mbvdsPerBase : pu.ibcMergeIdx1);
              }
#endif
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
              if (pu.cs->sps->getUseAML())
#endif
              {
#if JVET_AE0169_BIPREDICTIVE_IBC
                uint8_t interDir = pu.interDir;
#endif
#if JVET_Z0075_IBC_HMVP_ENLARGE
                PU::getIBCMergeCandidates(pu, mrgCtx);
#if JVET_AC0112_IBC_LIC && !JVET_AA0070_RRIBC
                pu.ibcMbvdMergeFlag = false;
#endif
                m_pcInterPred->adjustIBCMergeCandidates(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#if JVET_AC0112_IBC_LIC && !JVET_AA0070_RRIBC
                pu.ibcMbvdMergeFlag = true;
#endif
#else
                PU::getIBCMergeCandidates(pu, mrgCtx, (((fPosIBCBaseIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE + 1) * ADAPTIVE_IBC_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumIBCMergeCand()) || (fPosIBCBaseIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE) == 0) ? fPosIBCBaseIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE * ADAPTIVE_IBC_SUB_GROUP_SIZE + ADAPTIVE_IBC_SUB_GROUP_SIZE - 1 : fPosIBCBaseIdx);
                m_pcInterPred->adjustIBCMergeCandidates(pu, mrgCtx, fPosIBCBaseIdx);
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
                pu.interDir = interDir;
#endif
              }
              else
              {
#endif
                PU::getIBCMergeCandidates(pu, mrgCtx, fPosIBCBaseIdx);
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_W0090_ARMC_TM
              }
#endif

              PU::getIbcMbvdMergeCandidates(pu, mrgCtx, fPosIBCBaseIdx + 1);

#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
              {
#endif
                uint32_t ibcMbvdLUT[IBC_MBVD_NUM];
                uint32_t ibcMbvdValidNum[IBC_MBVD_BASE_NUM] = { 0 };
                int      ibcMbvdIdx= pu.ibcMbvdMergeIdx;
#if JVET_AE0169_BIPREDICTIVE_IBC
                int ibcMbvdIdx1 = pu.interDir == 1 ? -1 : pu.ibcMergeIdx1;
                int mask = 1<<(ibcMbvdIdx/mbvdsPerBase);
                if (ibcMbvdIdx1 >= IBC_MRG_MAX_NUM_CANDS)
                {
                  mask |= 1<<((ibcMbvdIdx1 - IBC_MRG_MAX_NUM_CANDS)/mbvdsPerBase);
                }
                m_pcInterPred->sortIbcMergeMbvdCandidates(pu, mrgCtx, ibcMbvdLUT, ibcMbvdValidNum, mask);
                int ibcMbvdIdx1LUT = (ibcMbvdIdx1 < IBC_MRG_MAX_NUM_CANDS) ? -1 : ibcMbvdLUT[ibcMbvdIdx1-IBC_MRG_MAX_NUM_CANDS];
                bool mbvdCandMisAlign = mrgCtx.setIbcMbvdMergeCandiInfo(pu, ibcMbvdIdx, ibcMbvdLUT[ibcMbvdIdx], ibcMbvdIdx1, ibcMbvdIdx1LUT);
#else
                m_pcInterPred->sortIbcMergeMbvdCandidates(pu, mrgCtx, ibcMbvdLUT, ibcMbvdValidNum, ibcMbvdIdx);
                bool mbvdCandMisAlign = mrgCtx.setIbcMbvdMergeCandiInfo(pu, ibcMbvdIdx, ibcMbvdLUT[ibcMbvdIdx]);
#endif
                CHECK(mbvdCandMisAlign, "MBVD candidate is invalid");
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              }
#endif
            }
            else
            {
#endif
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
              if (pu.cs->sps->getUseAML())
#endif
              {
#if JVET_Z0075_IBC_HMVP_ENLARGE
                auto mrgCandIdx = pu.mergeIdx;
                PU::getIBCMergeCandidates(pu, mrgCtx);

#if JVET_AK0076_EXTENDED_OBMC_IBC
                bool tmpGpm = pu.ibcGpmFlag;
                pu.ibcGpmFlag = false;
#endif
                m_pcInterPred->adjustIBCMergeCandidates(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);

#if JVET_AK0076_EXTENDED_OBMC_IBC
                pu.ibcGpmFlag = tmpGpm;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
                if (pu.ibcMergeIdx1 != MAX_INT)
                {
                  pu.interDir = 3;
                }
#endif
#if JVET_AC0112_IBC_GPM && JVET_AA0070_RRIBC
#if JVET_AE0169_BIPREDICTIVE_IBC
                if (pu.ibcGpmFlag || pu.interDir == 3)
#else
                if (pu.ibcGpmFlag)
#endif
                {
                  m_pcInterPred->adjustIbcMergeRribcCand(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
                }
#endif
                pu.mergeIdx = mrgCandIdx;
#else
                PU::getIBCMergeCandidates(pu, mrgCtx, (((pu.mergeIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE + 1) * ADAPTIVE_IBC_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumIBCMergeCand()) || (pu.mergeIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_IBC_SUB_GROUP_SIZE * ADAPTIVE_IBC_SUB_GROUP_SIZE + ADAPTIVE_IBC_SUB_GROUP_SIZE - 1 : pu.mergeIdx);
                m_pcInterPred->adjustIBCMergeCandidates(pu, mrgCtx, pu.mergeIdx);
#endif
              }
              else
#endif
#if JVET_AC0112_IBC_GPM
              {
                auto mrgCandIdx = pu.mergeIdx;
                if (pu.ibcGpmFlag)
                {
                  PU::getIBCMergeCandidates(pu, mrgCtx);
                }
                else
                {
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
                PU::getIBCMergeCandidates(pu, mrgCtx, pu.interDir == 3 ? pu.ibcMergeIdx1 : pu.mergeIdx);
#else
                PU::getIBCMergeCandidates(pu, mrgCtx, pu.mergeIdx);
#endif
#if JVET_AC0112_IBC_GPM
                }
                if (pu.ibcGpmFlag)
                {
                  m_pcInterPred->adjustIbcMergeRribcCand(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
                }
                pu.mergeIdx = mrgCandIdx;
              }
              m_ibcMrgCtx = mrgCtx;
#endif
#if JVET_AA0061_IBC_MBVD
            }
#endif
          }
          else
#if JVET_X0049_ADAPT_DMVR
            if (pu.bmMergeFlag)
            {
              auto mergeIdx = pu.bmDir == 2 ? pu.mergeIdx - BM_MRG_MAX_NUM_CANDS : pu.mergeIdx;
#if JVET_AI0187_TMVP_FOR_CMVP
              MergeCtx tmpBMMergeCtx;
              for (uint32_t ui = 0; ui < NUM_MERGE_CANDS; ++ui)
              {
                tmpBMMergeCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
                tmpBMMergeCtx.altLMFlag[ui] = false;
                tmpBMMergeCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
                tmpBMMergeCtx.licFlags[ui] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                tmpBMMergeCtx.setDefaultLICParamToCtx(ui);
#endif
#endif
                tmpBMMergeCtx.interDirNeighbours[ui] = 0;
                tmpBMMergeCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
                tmpBMMergeCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
                tmpBMMergeCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
                tmpBMMergeCtx.addHypNeighbours[ui].clear();
#endif
                tmpBMMergeCtx.candCost[ui] = MAX_UINT64;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                tmpBMMergeCtx.refineTmvpParams.refineTmvpRefList[(ui << 1)] = REF_PIC_LIST_X;
                tmpBMMergeCtx.refineTmvpParams.refineTmvpRefList[(ui << 1) + 1] = REF_PIC_LIST_X;
#endif
              }
              tmpBMMergeCtx.numValidMergeCand = 0;
#endif
#if JVET_W0090_ARMC_TM
              if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                 && pu.cs->sps->getTMToolsEnableFlag()
#endif
                )
              {
                uint8_t bmDir = pu.bmDir;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
                int nWidth = pu.lumaSize().width;
                int nHeight = pu.lumaSize().height;
                bool tplAvail = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && !JVET_AA0093_REFINED_MOTION_FOR_ARMC
                                pu.cs->sps->getUseTmvpNmvpReordering() &&
#endif
                                m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                if (pu.cs->sps->getUseTmvpNmvpReordering())
                {
#endif
                MergeCtx tmvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                if ( tplAvail)
                {
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
                  PU::getTmvpBMCand(pu, tmvpMergeCandCtx, tmpBMMergeCtx);
#else
                PU::getTmvpBMCand(pu, tmvpMergeCandCtx);
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                }
#endif
                pu.bmDir = 0;
#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                if (tplAvail)
                {
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpMergeCandCtx, 1, pu.mergeIdx);
                }
                else
                {
                  tmvpMergeCandCtx.numValidMergeCand = std::min(1, tmvpMergeCandCtx.numValidMergeCand);
                }
#endif
                MergeCtx namvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                if ( tplAvail)
                {
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
                  PU::getNonAdjacentBMCand(pu, namvpMergeCandCtx, tmpBMMergeCtx);
#else
                PU::getNonAdjacentBMCand(pu, namvpMergeCandCtx);
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                }
#endif
                pu.bmDir = 0;
#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                if (tplAvail)
                {
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, namvpMergeCandCtx, 3, pu.mergeIdx);
                }
                else
                {
                  namvpMergeCandCtx.numValidMergeCand = std::min(3, namvpMergeCandCtx.numValidMergeCand);
                }
#else
                if (!tplAvail)
                {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                  PU::getInterBMCandidates(m_pcInterPred, pu, mrgCtx
#else
                  PU::getInterBMCandidates(pu, mrgCtx
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
                    , tmpBMMergeCtx
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
                    , -1
                    , NULL
                    , NULL
#endif
                  );
              }
                else
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                  PU::getInterBMCandidates(m_pcInterPred, pu, mrgCtx, tmpBMMergeCtx, -1, &tmvpMergeCandCtx, &namvpMergeCandCtx);
#else
                  PU::getInterBMCandidates(pu, mrgCtx, tmpBMMergeCtx, -1, &tmvpMergeCandCtx, &namvpMergeCandCtx);
#endif
#else
                PU::getInterBMCandidates(pu, mrgCtx, -1, &tmvpMergeCandCtx, &namvpMergeCandCtx);
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                }
#endif
#endif
#if !JVET_Y0134_TMVP_NAMVP_CAND_REORDERING || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
                else
                {
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AI0187_TMVP_FOR_CMVP
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                  PU::getInterBMCandidates(m_pcInterPred, pu, mrgCtx, tmpBMMergeCtx, -1);
#else
                  PU::getInterBMCandidates(pu, mrgCtx, tmpBMMergeCtx, -1);
#endif
#else
                PU::getInterBMCandidates(pu, mrgCtx, -1);
#endif
#else
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                PU::getInterBMCandidates(pu, mrgCtx, pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag() && (((mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumBMMergeCand()) || (mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : mergeIdx);

#else
                PU::getInterBMCandidates(pu, mrgCtx, pu.cs->sps->getUseAML() && (((mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumBMMergeCand()) || (mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : mergeIdx);
#endif
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
                }
#endif
#endif
                pu.bmDir = 0;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                if (pu.cs->sps->getUseTmvpNmvpReordering())
                {
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
                admvrRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 1);
                admvrRefinedMotion &= tplAvail;
#endif
                if (tplAvail)
                {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
                  m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, pu.cs->sps->getMaxNumBMMergeCand());
                  if (mrgCtx.numCandToTestEnc > mrgCtx.numValidMergeCand)
                  {
                    mrgCtx.numCandToTestEnc = mrgCtx.numValidMergeCand;
                  }
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                  int refineTmvpTargetRefIdx = TMVP_REFINE_PRE_SET_TARGET_REF_IDX, refineTmvpL0Idx = 0, refineTmvpL1Idx = 1;
                  Mv   refineTmvpColMv, refineTmvpOrgMv[2];
                  for (int refineTmvpIdx = 0; refineTmvpIdx < mrgCtx.numValidMergeCand; refineTmvpIdx++)
                  {
                    refineTmvpL0Idx = refineTmvpIdx << 1;
                    refineTmvpL1Idx = refineTmvpL0Idx + 1;
                    refineTmvpOrgMv[0] = mrgCtx.mvFieldNeighbours[refineTmvpL0Idx].mv;
                    refineTmvpOrgMv[1] = mrgCtx.mvFieldNeighbours[refineTmvpL1Idx].mv;
                    if (mrgCtx.refineTmvpParams.refineTmvpRefList[refineTmvpL0Idx] == REF_PIC_LIST_0)
                    {
                      refineTmvpTargetRefIdx = mrgCtx.refineTmvpParams.refineTmvpTargetRefIdx[refineTmvpL0Idx];
                      Position refineTmvpPosition(mrgCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL0Idx][0], mrgCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL0Idx][1]);
                      m_pcInterPred->refineTmvpCoverSearchForRefinedMv(false/*refineTmvpIsSbTmvp*/, pu, REF_PIC_LIST_0, refineTmvpPosition, refineTmvpColMv,
                          mrgCtx.refineTmvpParams.refineTmvpRefIdx[refineTmvpL0Idx], mrgCtx.refineTmvpParams.refineTmvpSbFlag[refineTmvpL0Idx],
                          mrgCtx.refineTmvpParams.refineTmvpCol[refineTmvpL0Idx], &refineTmvpTargetRefIdx, 0/*sbTmvpType*/,
                          mrgCtx.refineTmvpParams.refineTmvpColRefPicList[refineTmvpL0Idx], mrgCtx.refineTmvpParams.refineTmvpColRefPOC[refineTmvpL0Idx],
                          *(mrgCtx.refineTmvpParams.refineTmvpMiPtr[refineTmvpL0Idx]));
                      mrgCtx.mvFieldNeighbours[refineTmvpL0Idx].refIdx = refineTmvpTargetRefIdx;
                      mrgCtx.mvFieldNeighbours[refineTmvpL0Idx].mv     = refineTmvpColMv;
                    }
                    if (mrgCtx.refineTmvpParams.refineTmvpRefList[refineTmvpL1Idx] == REF_PIC_LIST_1)
                    {
                      refineTmvpTargetRefIdx = mrgCtx.refineTmvpParams.refineTmvpTargetRefIdx[refineTmvpL1Idx];
                      Position refineTmvpPosition(mrgCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL1Idx][0], mrgCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL1Idx][1]);
                      m_pcInterPred->refineTmvpCoverSearchForRefinedMv(false/*refineTmvpIsSbTmvp*/, pu, REF_PIC_LIST_1, refineTmvpPosition, refineTmvpColMv,
                          mrgCtx.refineTmvpParams.refineTmvpRefIdx[refineTmvpL1Idx], mrgCtx.refineTmvpParams.refineTmvpSbFlag[refineTmvpL1Idx],
                          mrgCtx.refineTmvpParams.refineTmvpCol[refineTmvpL1Idx], &refineTmvpTargetRefIdx, 0/*sbTmvpType*/,
                          mrgCtx.refineTmvpParams.refineTmvpColRefPicList[refineTmvpL1Idx], mrgCtx.refineTmvpParams.refineTmvpColRefPOC[refineTmvpL1Idx],
                          *(mrgCtx.refineTmvpParams.refineTmvpMiPtr[refineTmvpL1Idx]));
                      mrgCtx.mvFieldNeighbours[refineTmvpL1Idx].refIdx = refineTmvpTargetRefIdx;
                      mrgCtx.mvFieldNeighbours[refineTmvpL1Idx].mv     = refineTmvpColMv;
                    }
                    if (mrgCtx.xCheckSimilarMotion( refineTmvpIdx, 1 ))
                    {
                      mrgCtx.mvFieldNeighbours[refineTmvpL0Idx].mv = refineTmvpOrgMv[0];
                      mrgCtx.mvFieldNeighbours[refineTmvpL1Idx].mv = refineTmvpOrgMv[1];
                    }
                  }
#endif
                  if (admvrRefinedMotion)
                  {
                    bool subPuRefineList[BM_MRG_MAX_NUM_INIT_CANDS][2] = { { false, } };
                    bool subPuRefineListTmp[BM_MRG_MAX_NUM_INIT_CANDS][2] = { { false, } };
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
                    uint16_t orgMergeIdx = pu.mergeIdx;
#else
                    uint8_t orgMergeIdx = pu.mergeIdx;
#endif
                    pu.bmDir = 0;
                    mrgCtx.setMergeInfo( pu, 0 );

                    pu.bmDir = bmDir;
                    pu.bdmvrRefine = true;
                    for( uint32_t candIdx = 0; candIdx < mrgCtx.numValidMergeCand; candIdx++ )
                    {
                      pu.cu->imv = mrgCtx.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
                      pu.cu->bcwIdx = mrgCtx.bcwIdx[candIdx];
#if JVET_AD0213_LIC_IMP
                      pu.cu->licFlag = mrgCtx.licFlags[candIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                      mrgCtx.setLICParamToPu(pu, candIdx, mrgCtx.licInheritPara[candIdx]);
#endif
#endif
                      pu.mv[0] = mrgCtx.mvFieldNeighbours[candIdx << 1].mv;
                      pu.mv[1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].mv;
                      pu.refIdx[0] = mrgCtx.mvFieldNeighbours[candIdx << 1].refIdx;
                      pu.refIdx[1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;

                      Mv   finalMvDir[2];
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                      applyBDMVR4BM[candIdx] = m_pcInterPred->processBDMVRPU2Dir(pu, subPuRefineList[candIdx], finalMvDir, candIdx);
#else
                      applyBDMVR4BM[candIdx] = m_pcInterPred->processBDMVRPU2Dir(pu, subPuRefineList[candIdx], finalMvDir);
#endif
                      subPuRefineListTmp[candIdx][0] = subPuRefineList[candIdx][0];
                      subPuRefineListTmp[candIdx][1] = subPuRefineList[candIdx][1];
                      mrgCtx.mvFieldNeighbours[(candIdx << 1) + bmDir - 1].mv = finalMvDir[bmDir - 1];
                    }
                    pu.bmDir = 0;
                    m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, bmDir == 2 ? applyBDMVR4BM : NULL, NULL, NULL, mergeIdx + 1, subPuRefineList, subPuRefineListTmp, mergeIdx);
#if JVET_AB0079_TM_BCW_MRG
                    m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, mergeIdx);
#endif
                    pu.bmDir = bmDir;
                    pu.mergeIdx = orgMergeIdx;
                    mrgCtx.setMergeInfo( pu, pu.mergeIdx);
                    m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                    m_pcInterPred->processBDMVRSubPU(pu, subPuRefineList[mergeIdx][pu.bmDir - 1], pu.mergeIdx);
#else
                    m_pcInterPred->processBDMVRSubPU(pu, subPuRefineList[mergeIdx][pu.bmDir - 1]);
#endif
                  }
#if JVET_AB0079_TM_BCW_MRG
                  else
                  {
                    auto orgMergeIdx = pu.mergeIdx;
                    pu.bmDir = 0;
                    m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, mergeIdx);
                    pu.bmDir = bmDir;
                    pu.mergeIdx = orgMergeIdx;
                  }
#endif
#if !JVET_AA0093_REFINED_MOTION_FOR_ARMC
                  else
#endif
#endif
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
#if !JVET_AA0093_REFINED_MOTION_FOR_ARMC
                  m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, pu.cs->sps->getMaxNumBMMergeCand());
#endif
#else
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, mergeIdx + 1, mergeIdx);
#endif
                }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                }
#endif
#endif
#if !JVET_Y0134_TMVP_NAMVP_CAND_REORDERING || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
                else
                {
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
                admvrRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 1);
                admvrRefinedMotion &= tplAvail;
                if (admvrRefinedMotion)
                {
                  if (mrgCtx.numCandToTestEnc > mrgCtx.numValidMergeCand)
                  {
                    mrgCtx.numCandToTestEnc = mrgCtx.numValidMergeCand;
                  }
                  if (admvrRefinedMotion)
                  {
                    bool subPuRefineList[BM_MRG_MAX_NUM_INIT_CANDS][2] = { { false, } };
                    bool subPuRefineListTmp[BM_MRG_MAX_NUM_INIT_CANDS][2] = { { false, } };
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
                    uint16_t orgMergeIdx = pu.mergeIdx;
#else
                    uint8_t orgMergeIdx = pu.mergeIdx;
#endif
                    pu.bmDir = 0;
                    mrgCtx.setMergeInfo( pu, 0 );

                    pu.bmDir = bmDir;
                    pu.bdmvrRefine = true;
                    for( uint32_t candIdx = 0; candIdx < mrgCtx.numValidMergeCand; candIdx++ )
                    {
                      pu.cu->imv = mrgCtx.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
                      pu.cu->bcwIdx = mrgCtx.bcwIdx[candIdx];
#if JVET_AD0213_LIC_IMP
                      pu.cu->licFlag = mrgCtx.licFlags[candIdx];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                      mrgCtx.setLICParamToPu(pu, candIdx, mrgCtx.licInheritPara[candIdx]);
#endif
#endif
                      pu.mv[0] = mrgCtx.mvFieldNeighbours[candIdx << 1].mv;
                      pu.mv[1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].mv;
                      pu.refIdx[0] = mrgCtx.mvFieldNeighbours[candIdx << 1].refIdx;
                      pu.refIdx[1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;

                      Mv   finalMvDir[2];
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                      applyBDMVR4BM[candIdx] = m_pcInterPred->processBDMVRPU2Dir(pu, subPuRefineList[candIdx], finalMvDir, candIdx);
#else
                      applyBDMVR4BM[candIdx] = m_pcInterPred->processBDMVRPU2Dir(pu, subPuRefineList[candIdx], finalMvDir);
#endif
                      subPuRefineListTmp[candIdx][0] = subPuRefineList[candIdx][0];
                      subPuRefineListTmp[candIdx][1] = subPuRefineList[candIdx][1];
                      mrgCtx.mvFieldNeighbours[(candIdx << 1) + bmDir - 1].mv = finalMvDir[bmDir - 1];
                    }
                    pu.bmDir = 0;
                    m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, bmDir == 2 ? applyBDMVR4BM : NULL, NULL, NULL, mergeIdx + 1, subPuRefineList, subPuRefineListTmp, mergeIdx);
#if JVET_AB0079_TM_BCW_MRG
                    m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, mergeIdx);
#endif
                    pu.bmDir = bmDir;
                    pu.mergeIdx = orgMergeIdx;
                    mrgCtx.setMergeInfo( pu, pu.mergeIdx);
                    m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                    m_pcInterPred->processBDMVRSubPU(pu, subPuRefineList[mergeIdx][pu.bmDir - 1], pu.mergeIdx);
#else
                    m_pcInterPred->processBDMVRSubPU(pu, subPuRefineList[mergeIdx][pu.bmDir - 1]);
#endif
                  }
                }
                else
#endif
#if JVET_AB0079_TM_BCW_MRG
                {
#endif
                m_pcInterPred->adjustInterMergeCandidates(pu, mrgCtx, mergeIdx);
#if JVET_AB0079_TM_BCW_MRG
                  auto orgMergeIdx = pu.mergeIdx;
                  pu.bmDir = 0;
                  m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, mergeIdx);
                  pu.bmDir = bmDir;
                  pu.mergeIdx = orgMergeIdx;
                }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
                }
#endif
#endif
                pu.bmDir = bmDir;
              }
              else
#endif
#if JVET_AI0187_TMVP_FOR_CMVP
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::getInterBMCandidates(m_pcInterPred, pu, mrgCtx, tmpBMMergeCtx, mergeIdx);
#else
              PU::getInterBMCandidates(pu, mrgCtx, tmpBMMergeCtx, mergeIdx);
#endif
#else
              PU::getInterBMCandidates(pu, mrgCtx, mergeIdx);
#endif
            }
            else
#endif
#if JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag())
#else
            if (pu.cs->sps->getUseAML())
#endif
            {
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
              if (pu.cs->sps->getUseTmvpNmvpReordering())
              {
#endif
              int nWidth = pu.lumaSize().width;
              int nHeight = pu.lumaSize().height;
              bool tplAvail = m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);

              MergeCtx tmvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              if (tplAvail)
              {
#endif
              PU::getTmvpMergeCand(pu, tmvpMergeCandCtx);
#if JVET_AI0187_TMVP_FOR_CMVP
              bool isLD = pu.cs->slice->getCheckLDC();
              m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpMergeCandCtx, pu.tmMergeFlag ? (isLD ? TM_ARMC_NUM_LD : TM_ARMC_NUM) : (isLD ? REGULAR_ARMC_NUM_LD : REGULAR_ARMC_NUM), -1, true);
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              int refineTmvpTargetRefIdx = TMVP_REFINE_PRE_SET_TARGET_REF_IDX, refineTmvpL0Idx = 0, refineTmvpL1Idx = 1;
              Mv   refineTmvpColMv, refineTmvpOrgMv[2];
              for (int refineTmvpIdx = 0; refineTmvpIdx < tmvpMergeCandCtx.numValidMergeCand; refineTmvpIdx++)
              {
                refineTmvpL0Idx = refineTmvpIdx << 1;
                refineTmvpL1Idx = refineTmvpL0Idx + 1;
                refineTmvpOrgMv[0] = tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL0Idx].mv;
                refineTmvpOrgMv[1] = tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL1Idx].mv;
                if (tmvpMergeCandCtx.refineTmvpParams.refineTmvpRefList[refineTmvpL0Idx] == REF_PIC_LIST_0)
                {
                  refineTmvpTargetRefIdx = tmvpMergeCandCtx.refineTmvpParams.refineTmvpTargetRefIdx[refineTmvpL0Idx];
                  Position refineTmvpPosition(tmvpMergeCandCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL0Idx][0], tmvpMergeCandCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL0Idx][1]);
                  m_pcInterPred->refineTmvpCoverSearchForRefinedMv(false/*refineTmvpIsSbTmvp*/, pu, REF_PIC_LIST_0, refineTmvpPosition, refineTmvpColMv,
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpRefIdx[refineTmvpL0Idx], tmvpMergeCandCtx.refineTmvpParams.refineTmvpSbFlag[refineTmvpL0Idx],
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpCol[refineTmvpL0Idx], &refineTmvpTargetRefIdx, 0/*sbTmvpType*/,
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpColRefPicList[refineTmvpL0Idx], tmvpMergeCandCtx.refineTmvpParams.refineTmvpColRefPOC[refineTmvpL0Idx],
                      *(tmvpMergeCandCtx.refineTmvpParams.refineTmvpMiPtr[refineTmvpL0Idx]));
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL0Idx].refIdx = refineTmvpTargetRefIdx;
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL0Idx].mv     = refineTmvpColMv;
                }
                if (tmvpMergeCandCtx.refineTmvpParams.refineTmvpRefList[refineTmvpL1Idx] == REF_PIC_LIST_1)
                {
                  refineTmvpTargetRefIdx = tmvpMergeCandCtx.refineTmvpParams.refineTmvpTargetRefIdx[refineTmvpL1Idx];
                  Position refineTmvpPosition(tmvpMergeCandCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL1Idx][0], tmvpMergeCandCtx.refineTmvpParams.refineTmvpPosition[refineTmvpL1Idx][1]);
                  m_pcInterPred->refineTmvpCoverSearchForRefinedMv(false/*refineTmvpIsSbTmvp*/, pu, REF_PIC_LIST_1, refineTmvpPosition, refineTmvpColMv,
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpRefIdx[refineTmvpL1Idx], tmvpMergeCandCtx.refineTmvpParams.refineTmvpSbFlag[refineTmvpL1Idx],
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpCol[refineTmvpL1Idx], &refineTmvpTargetRefIdx, 0/*sbTmvpType*/,
                      tmvpMergeCandCtx.refineTmvpParams.refineTmvpColRefPicList[refineTmvpL1Idx], tmvpMergeCandCtx.refineTmvpParams.refineTmvpColRefPOC[refineTmvpL1Idx],
                      *(tmvpMergeCandCtx.refineTmvpParams.refineTmvpMiPtr[refineTmvpL1Idx]));
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL1Idx].refIdx = refineTmvpTargetRefIdx;
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL1Idx].mv     = refineTmvpColMv;
                }
                if (tmvpMergeCandCtx.xCheckSimilarMotion( refineTmvpIdx, 1 ))
                {
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL0Idx].mv = refineTmvpOrgMv[0];
                  tmvpMergeCandCtx.mvFieldNeighbours[refineTmvpL1Idx].mv = refineTmvpOrgMv[1];
                }
              }
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              }
#endif
#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              if (tplAvail)
              {
                m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpMergeCandCtx, 1, pu.mergeIdx);
              }
              else
              {
                tmvpMergeCandCtx.numValidMergeCand = std::min(1, tmvpMergeCandCtx.numValidMergeCand);
              }
#endif
              MergeCtx namvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              if (tplAvail)
              {
#endif
              PU::getNonAdjacentMergeCand(pu, namvpMergeCandCtx);
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              }
#endif
#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
              if (tplAvail)
              {
                m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, namvpMergeCandCtx, 9, pu.mergeIdx);
              }
              else
              {
                namvpMergeCandCtx.numValidMergeCand = std::min(9, namvpMergeCandCtx.numValidMergeCand);
              }
#else
              if (!tplAvail)
              {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, -1);
#else
                PU::getInterMergeCandidates(pu, mrgCtx, 0, -1);
#endif
                mrgCtx.numValidMergeCand =
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                           pu.cs->sps->getUseTMMrgMode() &&
#endif
                                           pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() :
#endif
                                           pu.cs->sps->getMaxNumMergeCand();
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if ((pu.mergeOppositeLic || pu.tmMergeFlagOppositeLic) && !pu.bmMergeFlag)
                {
                  for (int i = 0; i < mrgCtx.numValidMergeCand; i++)
                  {
                    mrgCtx.licFlags[i] = !mrgCtx.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    mrgCtx.licInheritPara[i] = false;
#endif
                  }
                }
#endif
              }
              else
#endif
              {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, -1, &tmvpMergeCandCtx, &namvpMergeCandCtx);
#else
              PU::getInterMergeCandidates(pu, mrgCtx, 0, -1, &tmvpMergeCandCtx, &namvpMergeCandCtx);
#endif
              }
#if JVET_AF0128_LIC_MERGE_TM
              m_pcInterPred->adjustMergeCandidatesLicFlag(pu, mrgCtx, pu.mergeIdx);
#endif

#if TM_MRG && JVET_AA0093_REFINED_MOTION_FOR_ARMC
              tmMergeRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 2);
              tmMergeRefinedMotion &= tplAvail;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
              tmMergeRefinedMotion &= pu.cs->sps->getUseTMMrgMode();
#endif
#endif
              if (tplAvail)
              {
#if TM_MRG && JVET_AA0093_REFINED_MOTION_FOR_ARMC
                if (pu.tmMergeFlag && tmMergeRefinedMotion)
                {
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  if (pu.tmMergeFlagOppositeLic)
                  {
                    for (int i = 0; i < mrgCtx.numValidMergeCand; i++)
                    {
                      mrgCtx.licFlags[i] = !mrgCtx.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                      mrgCtx.licInheritPara[i] = false;
#endif
                    }
                  }
#endif
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
#if JVET_AN0236_GENERATED_MERGE_CANDIDATES
                  m_pcInterPred->generateMergeCandidates(pu, mrgCtx, pu.cs->sps->getMaxNumTMMergeCand());
#endif
                  m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, pu.cs->sps->getMaxNumTMMergeCand());
#else
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, mrgCtx.numValidMergeCand, pu.mergeIdx);
#endif
                  int tmpPuMrgIdx = pu.mergeIdx;
                  pu.reduceTplSize = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  if (pu.tmMergeFlagOppositeLic)
                  {
                    if (mrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMOppositeLicMergeCand())
                    {
                      mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMOppositeLicMergeCand();
                    }
                  }
                  else
                  {
                    if (mrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMMergeCand())
                    {
                      mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
                    }
                  }
#else
                  if (mrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMMergeCand())
                  {
                    mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
                  }
#endif
                  if (mrgCtx.numCandToTestEnc > mrgCtx.numValidMergeCand)
                  {
                    mrgCtx.numCandToTestEnc = mrgCtx.numValidMergeCand;
                  }

                  for (uint32_t ui = mrgCtx.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
                  {
                    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
                    mrgCtx.altLMFlag[ui] = false;
                    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
                    mrgCtx.licFlags[ui] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    mrgCtx.setDefaultLICParamToCtx(ui);
#endif
#endif
                    mrgCtx.interDirNeighbours[ui] = 0;
                    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
                    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
                    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
                    mrgCtx.addHypNeighbours[ui].clear();
#endif
                    mrgCtx.candCost[ui] = MAX_UINT64;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                    mrgCtx.refineTmvpParams.refineTmvpRefList[(ui << 1)] = REF_PIC_LIST_X;
                    mrgCtx.refineTmvpParams.refineTmvpRefList[(ui << 1) + 1] = REF_PIC_LIST_X;
#endif
                  }

                  Distortion tempCost[1];
                  for( uint32_t uiMergeCand = 0; uiMergeCand < mrgCtx.numValidMergeCand; uiMergeCand++ )
                  {
                    mrgCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
                    applyBDMVR4TM[uiMergeCand] = PU::checkBDMVRCondition(pu);
                    if (applyBDMVR4TM[uiMergeCand])
                    {
                      m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);
                      pu.bdmvrRefine = true;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                      applyBDMVR4TM[uiMergeCand] = m_pcInterPred->processBDMVR( pu, 1, tempCost, uiMergeCand);
#else
                      applyBDMVR4TM[uiMergeCand] = m_pcInterPred->processBDMVR( pu, 1, tempCost );
#endif
                    }
                    else
                    {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                      m_pcInterPred->deriveTMMv(pu, tempCost, uiMergeCand);
#else
                      m_pcInterPred->deriveTMMv(pu, tempCost);
#endif
                    }
#else
                    m_pcInterPred->deriveTMMv( pu );
#endif

                    mrgCtx.candCost[uiMergeCand] = tempCost[0];
                    mrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
                    mrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand].mv = pu.mv[0];
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand].refIdx = pu.refIdx[0];
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv = pu.mv[1];
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].refIdx = pu.refIdx[1];
                    if( pu.interDir == 1 )
                    {
                      mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv.setZero();
                      mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].refIdx = NOT_VALID;
                    }
                    if( pu.interDir == 2 )
                    {
                      mrgCtx.mvFieldNeighbours[2 * uiMergeCand].mv.setZero();
                      mrgCtx.mvFieldNeighbours[2 * uiMergeCand].refIdx = NOT_VALID;
                    }
                  }
                  pu.reduceTplSize = false;
                  m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, applyBDMVR4TM, NULL, NULL, mrgCtx.numValidMergeCand);
#if JVET_AB0079_TM_BCW_MRG
                  m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, tmpPuMrgIdx);
#endif
                  pu.mergeIdx = tmpPuMrgIdx;
                }
                else
#endif
#if JVET_AB0079_TM_BCW_MRG
                {
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if (pu.mergeOppositeLic || pu.tmMergeFlagOppositeLic)
                {
                  for (int i = 0; i < mrgCtx.numValidMergeCand; i++)
                  {
                    mrgCtx.licFlags[i] = !mrgCtx.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    mrgCtx.licInheritPara[i] = false;
#endif
                  }
                }
#endif
#if JVET_AG0276_NLIC || JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                if (pu.tmMergeFlag || pu.ciipFlag)
                {
#endif
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
#if JVET_AN0236_GENERATED_MERGE_CANDIDATES			

                  m_pcInterPred->generateMergeCandidates(pu, mrgCtx,
#if TM_MRG
                    pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() :
#endif
                    pu.cs->sps->getMaxNumMergeCand());

#endif
                m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, 
#if TM_MRG
                                                     pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : 
#endif
                                                     pu.cs->sps->getMaxNumMergeCand());
#else
                m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, pu.mergeIdx + 1, pu.mergeIdx);
#endif
#if JVET_AG0276_NLIC
                }
                else
                {
                  if (pu.cs->sps->getUseAltLM() && !CU::isTLCond(*pu.cu))
                  {
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  if (!pu.mergeOppositeLic)
                  {
#endif
                    AltLMMergeCtx altLMMrgCtx;
                    PU::getAltMergeCandidates(pu, altLMMrgCtx);
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    AltLMMergeCtx mrgCtxInherited;
                    PU::getAltMergeCandidates(pu, mrgCtxInherited, true);
#endif
                    m_pcInterPred->adjustMergeCandidates(pu, mrgCtx
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                                       , &altLMMrgCtx
                                                       , &mrgCtxInherited
#else
                                                       , altLMMrgCtx
#endif
                                                       , pu.cs->sps->getMaxNumMergeCand());
#if JVET_AG0276_LIC_FLAG_SIGNALING
                  }
                  else
                  {
                    AltLMMergeCtx altLMBRMrgCtx;
                    PU::getAltBRMergeCandidates(pu, altLMBRMrgCtx);
                    m_pcInterPred->adjustMergeCandidates(pu, mrgCtx
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                                                       , &altLMBRMrgCtx
                                                       , nullptr
#else
                                                       , altLMBRMrgCtx
#endif
                                                       , pu.cs->sps->getMaxNumMergeCand());
                  }
#endif
                  }
                  else
                  {
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
#if JVET_AG0276_LIC_FLAG_SIGNALING
                    if (!PU::isOppositeLIC(pu))
#else
                    if(true)
#endif
                    {
                      AltLMMergeCtx mrgCtxInherited;
                      PU::getAltMergeCandidates(pu, mrgCtxInherited, true);
                      m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, nullptr, &mrgCtxInherited,
#if TM_MRG
                                                           pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : 
#endif
                                                           pu.cs->sps->getMaxNumMergeCand());
                    }
                    else
                    {
#endif
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
#if JVET_AN0236_GENERATED_MERGE_CANDIDATES
                      m_pcInterPred->generateMergeCandidates(pu, mrgCtx,
#if TM_MRG 
                        pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() :
#endif
                        pu.cs->sps->getMaxNumMergeCand());
#endif
                    m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, 
#if TM_MRG
                                                         pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : 
#endif
                                                         pu.cs->sps->getMaxNumMergeCand());
#else
                    m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, pu.mergeIdx + 1, pu.mergeIdx);
#endif
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    }
#endif
                  }
                }
#elif JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                }
                else
                {
                  AltLMMergeCtx mrgCtxInherited;
                  PU::getAltMergeCandidates(pu, mrgCtxInherited, true);
                  m_pcInterPred->adjustMergeCandidates(pu, mrgCtx, nullptr, &mrgCtxInherited, pu.cs->sps->getMaxNumMergeCand());
                }
#endif
#if JVET_AB0079_TM_BCW_MRG
                m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, pu.mergeIdx);
                }
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
                if (mrgCtx.numCandToTestEnc > mrgCtx.numValidMergeCand)
                {
                  mrgCtx.numCandToTestEnc = mrgCtx.numValidMergeCand;
                }
#endif
              }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
              }
#endif
#endif
#if !JVET_Y0134_TMVP_NAMVP_CAND_REORDERING || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
              else
              {
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag() && (((pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumMergeCand()) || (pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : pu.mergeIdx);
#else
              PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag() && (((pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumMergeCand()) || (pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : pu.mergeIdx);
#endif
#else
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, pu.cs->sps->getUseAML() && (((pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumMergeCand()) || (pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : pu.mergeIdx);
#else
              PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.cs->sps->getUseAML() && (((pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE + 1)*ADAPTIVE_SUB_GROUP_SIZE < pu.cs->sps->getMaxNumMergeCand()) || (pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE) == 0) ? pu.mergeIdx / ADAPTIVE_SUB_GROUP_SIZE * ADAPTIVE_SUB_GROUP_SIZE + ADAPTIVE_SUB_GROUP_SIZE - 1 : pu.mergeIdx);
#endif
#endif
#if JVET_AF0128_LIC_MERGE_TM
              m_pcInterPred->adjustMergeCandidatesLicFlag(pu, mrgCtx, pu.mergeIdx);
#endif
#if TM_MRG && JVET_AA0093_REFINED_MOTION_FOR_ARMC
              tmMergeRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 2);
              int nWidth = pu.lumaSize().width;
              int nHeight = pu.lumaSize().height;
              bool tplAvail = m_pcInterPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
              tmMergeRefinedMotion &= tplAvail;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
              tmMergeRefinedMotion &= pu.cs->sps->getUseTMMrgMode();
#endif
              if (pu.tmMergeFlag && tmMergeRefinedMotion)
              {
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if (pu.tmMergeFlagOppositeLic)
                {
                  for (int i = 0; i < mrgCtx.numValidMergeCand; i++)
                  {
                    mrgCtx.licFlags[i] = !mrgCtx.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    mrgCtx.licInheritPara[i] = false;
#endif
                  }
                }
#endif
                int tmpPuMrgIdx = pu.mergeIdx;
                pu.reduceTplSize = true;
                if (mrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMMergeCand())
                {
                  mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
                }

                if (mrgCtx.numCandToTestEnc > mrgCtx.numValidMergeCand)
                {
                  mrgCtx.numCandToTestEnc = mrgCtx.numValidMergeCand;
                }

                for (uint32_t ui = mrgCtx.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
                {
                  mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
                  mrgCtx.altLMFlag[ui] = false;
                  mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
                  mrgCtx.licFlags[ui] = false;
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                  mrgCtx.setDefaultLICParamToCtx(ui);
#endif
#endif
                  mrgCtx.interDirNeighbours[ui] = 0;
                  mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
                  mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
                  mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
                  mrgCtx.addHypNeighbours[ui].clear();
#endif
                  mrgCtx.candCost[ui] = MAX_UINT64;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
                  mrgCtx.refineTmvpParams.refineTmvpRefList[(ui << 1)] = REF_PIC_LIST_X;
                  mrgCtx.refineTmvpParams.refineTmvpRefList[(ui << 1) + 1] = REF_PIC_LIST_X;
#endif
                }

                Distortion tempCost[1];
                for( uint32_t uiMergeCand = 0; uiMergeCand < mrgCtx.numValidMergeCand; uiMergeCand++ )
                {
                  mrgCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
                  applyBDMVR4TM[uiMergeCand] = PU::checkBDMVRCondition(pu);
                  if (applyBDMVR4TM[uiMergeCand])
                  {
                    m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);
                    pu.bdmvrRefine = true;
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                    applyBDMVR4TM[uiMergeCand] = m_pcInterPred->processBDMVR( pu, 1, tempCost, uiMergeCand);
#else
                    applyBDMVR4TM[uiMergeCand] = m_pcInterPred->processBDMVR( pu, 1, tempCost );
#endif
                  }
                  else
                  {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
                    m_pcInterPred->deriveTMMv(pu, tempCost, uiMergeCand);
#else
                    m_pcInterPred->deriveTMMv(pu, tempCost);
#endif
                  }
#else
                  m_pcInterPred->deriveTMMv( pu );
#endif

                  mrgCtx.candCost[uiMergeCand] = tempCost[0];
                  mrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
                  mrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;
                  mrgCtx.mvFieldNeighbours[2 * uiMergeCand].mv = pu.mv[0];
                  mrgCtx.mvFieldNeighbours[2 * uiMergeCand].refIdx = pu.refIdx[0];
                  mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv = pu.mv[1];
                  mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].refIdx = pu.refIdx[1];
                  if( pu.interDir == 1 )
                  {
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv.setZero();
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].refIdx = NOT_VALID;
                  }
                  if( pu.interDir == 2 )
                  {
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand].mv.setZero();
                    mrgCtx.mvFieldNeighbours[2 * uiMergeCand].refIdx = NOT_VALID;
                  }
                }
                pu.reduceTplSize = false;
                m_pcInterPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, applyBDMVR4TM, NULL, NULL, mrgCtx.numValidMergeCand);
#if JVET_AB0079_TM_BCW_MRG
                m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, tmpPuMrgIdx);
#endif
                pu.mergeIdx = tmpPuMrgIdx;
              }
              else
#endif
#if JVET_AB0079_TM_BCW_MRG
              {
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if (pu.mergeOppositeLic)
                {
                  for (int i = 0; i < mrgCtx.numValidMergeCand; i++)
                  {
                    mrgCtx.licFlags[i] = !mrgCtx.licFlags[i];
#if JVET_AH0314_LIC_INHERITANCE_FOR_MRG
                    mrgCtx.licInheritPara[i] = false;
#endif
                  }
                }
#endif
              m_pcInterPred->adjustInterMergeCandidates(pu, mrgCtx, pu.mergeIdx);
#if JVET_AB0079_TM_BCW_MRG
              m_pcInterPred->adjustMergeCandidatesBcwIdx(pu, mrgCtx, pu.mergeIdx);
              }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
              }
#endif
#endif
            }
            else
            {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::getInterMergeCandidates(m_pcInterPred, pu, mrgCtx, 0, pu.mergeIdx);
#else
              PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.mergeIdx);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
              if (pu.mergeOppositeLic)
              {
                mrgCtx.licFlags[pu.mergeIdx] = !mrgCtx.licFlags[pu.mergeIdx];
              }
#endif
            }
#else
            PU::getInterMergeCandidates(pu, mrgCtx, 0, pu.mergeIdx);
#endif
#if JVET_AA0061_IBC_MBVD
          if (!pu.ibcMbvdMergeFlag)
          {
#endif
          mrgCtx.setMergeInfo( pu, pu.mergeIdx );
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (CU::isIBC(*pu.cu) && pu.ibcMergeIdx1 != MAX_INT)
          {
            mrgCtx.setIbcL1Info(pu, pu.ibcMergeIdx1);
          }
#endif
#if JVET_AA0061_IBC_MBVD
          }
#endif
#if TM_MRG && JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (pu.tmMergeFlag && tmMergeRefinedMotion)
          {
            pu.bdmvrRefine = applyBDMVR4TM[pu.mergeIdx];
            if (pu.bdmvrRefine)
            {
              m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              pu.bdmvrRefine = m_pcInterPred->processBDMVR( pu, 0, NULL, pu.mergeIdx);
#else
              pu.bdmvrRefine = m_pcInterPred->processBDMVR( pu );
#endif
            }
            else
            {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              m_pcInterPred->deriveTMMv(pu, NULL, pu.mergeIdx);
#else
              m_pcInterPred->deriveTMMv(pu);
#endif
            }
          }
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (pu.bmMergeFlag)
          {
            pu.bdmvrRefine = true;
          }
#endif
#if (TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)) && !MULTI_PASS_DMVR
          if (pu.tmMergeFlag)
          {
            m_pcInterPred->deriveTMMv(pu);
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
            if (CU::isIBC(*pu.cu))
            {
              pu.bv = pu.mv[0];
              pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
            }
#endif
          }
#endif
#if MULTI_PASS_DMVR
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
#if TM_MRG
          if (PU::checkBDMVRCondition(pu) && (!pu.tmMergeFlag || (pu.tmMergeFlag && !tmMergeRefinedMotion)) && (!pu.bmMergeFlag || (pu.bmMergeFlag && !admvrRefinedMotion)))
#else
          if (PU::checkBDMVRCondition(pu) && (!pu.bmMergeFlag || (pu.bmMergeFlag && !admvrRefinedMotion)))
#endif
#else
          if (PU::checkBDMVRCondition(pu))
#endif
          {
            m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
            pu.bdmvrRefine = true;

            CHECK(mrgCtx.numValidMergeCand <= 0, "this is not possible");

#if JVET_X0049_ADAPT_DMVR
#if TM_MRG
            if (!pu.tmMergeFlag && !pu.bmMergeFlag && mrgCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
#else
            if (!pu.bmMergeFlag && mrgCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
#endif
#else
#if TM_MRG
            if( !pu.tmMergeFlag && mrgCtx.xCheckSimilarMotion( pu.mergeIdx, PU::getBDMVRMvdThreshold( pu ) ) )
#else
            if( mrgCtx.xCheckSimilarMotion( pu.mergeIdx, PU::getBDMVRMvdThreshold( pu ) ) )
#endif
#endif
            {
              // span motion to subPU
              for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
              {
                m_mvBufBDMVR[0][subPuIdx] = pu.mv[0];
                m_mvBufBDMVR[1][subPuIdx] = pu.mv[1];
              }
            }
            else
            {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              pu.bdmvrRefine = m_pcInterPred->processBDMVR( pu, 0, NULL, pu.mergeIdx);
#else
              pu.bdmvrRefine = m_pcInterPred->processBDMVR( pu );
#endif
            }
          }
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
          else
          {
#if TM_MRG && JVET_AA0093_REFINED_MOTION_FOR_ARMC
            if (pu.tmMergeFlag && !tmMergeRefinedMotion)
#else
            if (pu.tmMergeFlag)
#endif
            {
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
              m_pcInterPred->deriveTMMv(pu, NULL, CU::isIBC(*pu.cu)? -1 : pu.mergeIdx);
#else
              m_pcInterPred->deriveTMMv(pu);
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
              if (CU::isIBC(*pu.cu))
              {
                pu.bv = pu.mv[0];
#if JVET_AE0169_BIPREDICTIVE_IBC
                if (pu.interDir == 3)
                {
                  pu.mv[0] = pu.mv[1];
                  m_pcInterPred->deriveTMMv(pu);
                  pu.mv[1] = pu.mv[0];
                  pu.mv[0] = pu.bv;
                }
#endif
                pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
              }
#endif
            }
          }
#endif
#endif
#if MULTI_HYP_PRED
          CHECK(pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE && !pu.addHypData.empty(), "There are add hyps in small block")
#endif
#if MULTI_PASS_DMVR
          if( !pu.bdmvrRefine )
          {
            PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              , pu.colIdx
#endif
            );
          }
#else
          PU::spanMotionInfo( pu, mrgCtx );
#endif
        }
        }
      }
      }
#if MULTI_HYP_PRED
      // put saved additional hypotheseis to the end
#if JVET_Z0118_GDR
      size_t n = addHypData.capacity();
      size_t s = pu.addHypData.size();
      size_t e = addHypData.end() - addHypData.begin();

      if ((s + e) < n)
      {
        pu.addHypData.insert(pu.addHypData.end(), std::make_move_iterator(addHypData.begin()), std::make_move_iterator(addHypData.end()));
      }
#else
      pu.addHypData.insert(pu.addHypData.end(), std::make_move_iterator(addHypData.begin()), std::make_move_iterator(addHypData.end()));
#endif
#endif
    }
    else
    {
#if JVET_Z0054_BLK_REF_PIC_REORDER
#if REUSE_CU_RESULTS
      if (!cu.cs->pcv->isEncoder)
#endif
      {
#if TM_AMVP
        m_pcInterPred->clearTplAmvpBuffer();
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        m_pcInterPred->clearAmvpTmvpBuffer();
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
        m_pcInterPred->clearAffineAmvpBuffer();
#endif
        if (pu.cu->affine)
        {
          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            if (pu.interDir & (1 << uiRefListIdx))
            {
              pu.mvdAffi[uiRefListIdx][0].changeAffinePrecAmvr2Internal(pu.cu->imv);
              pu.mvdAffi[uiRefListIdx][1].changeAffinePrecAmvr2Internal(pu.cu->imv);
              if (cu.affineType == AFFINEMODEL_6PARAM)
              {
                pu.mvdAffi[uiRefListIdx][2].changeAffinePrecAmvr2Internal(pu.cu->imv);
              }
            }
          }
        }
#if JVET_AE0169_BIPREDICTIVE_IBC
        else if (CU::isIBC(*pu.cu))
#else
        else if (CU::isIBC(*pu.cu) && pu.interDir == 1)
#endif
        {
          pu.mvd[REF_PIC_LIST_0].changeIbcPrecAmvr2Internal(pu.cu->imv);
        }
        else
        {
          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            if (pu.interDir & (1 << uiRefListIdx))
            {
#if JVET_AG0098_AMVP_WITH_SBTMVP 
              if (pu.amvpSbTmvpFlag)
              {
                if (pu.amvpSbTmvpMvdIdx >= 0)
                {
                  int tempDir = pu.amvpSbTmvpMvdIdx % AMVP_SBTMVP_NUM_DIR;
                  int tempOffset = pu.amvpSbTmvpMvdIdx / AMVP_SBTMVP_NUM_DIR;
                  int mvdDirOffset = ((tempOffset % 2) == 0) ? 0 : AMVP_SBTMVP_NUM_DIR;
                  if (pu.cu->imv)
                  {
                    mvdDirOffset = ((tempOffset % 2) == 1) ? 0 : AMVP_SBTMVP_NUM_DIR;
                    tempOffset += AMVP_SBTMVP_NUM_OFFSET;
                  }
                  int dirX = g_amvpSbTmvp_mvd_dir[0][tempDir + mvdDirOffset];
                  int dirY = g_amvpSbTmvp_mvd_dir[1][tempDir + mvdDirOffset];
                  int step = g_amvpSbTmvp_mvd_offset[tempOffset];
                  pu.mvd[uiRefListIdx].set(dirX * step, dirY * step);
                  pu.mvd[uiRefListIdx].changeTransPrecAmvr2Internal(IMV_FPEL);
                }
              }
              else
              {
                pu.mvd[uiRefListIdx].changeTransPrecAmvr2Internal(pu.cu->imv);
              }
#else
              pu.mvd[uiRefListIdx].changeTransPrecAmvr2Internal(pu.cu->imv);
#endif
            }
          }
        }
        if (PU::useRefPairList(pu))
        {
          m_pcInterPred->setBiRefIdx(pu);
        }
        if (PU::useRefCombList(pu))
        {
          m_pcInterPred->setUniRefListAndIdx(pu);
        }
      }
#endif

#if !JVET_Z0054_BLK_REF_PIC_REORDER
#if REUSE_CU_RESULTS
      if ( cu.imv && !pu.cu->affine && !cu.cs->pcv->isEncoder 
#if JVET_AC0104_IBC_BVD_PREDICTION
        && !CU::isIBC(cu)
#endif
         )
#else
        if (cu.imv && !pu.cu->affine)
#endif
        {
          PU::applyImv(pu, mrgCtx, m_pcInterPred);
        }
        else
#endif
      {
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if REUSE_CU_RESULTS
#if JVET_AE0169_BIPREDICTIVE_IBC
        if (!cu.cs->pcv->isEncoder && !CU::isIBC(cu) && (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]))
#else
        if (!cu.cs->pcv->isEncoder && (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]))
#endif
#else
        if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
#endif
        {
          CHECK(pu.interDir != 3, "this is not possible");
          const RefPicList refListMerge = pu.amvpMergeModeFlag[REF_PIC_LIST_0] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
          const RefPicList refListAmvp = RefPicList(1 - refListMerge);
          int orgRefIdxAMVP = pu.refIdx[refListAmvp];
          int orgInterDir = pu.interDir;
          int orgMvpIdxL0 = pu.mvpIdx[REF_PIC_LIST_0];
          int orgMvpIdxL1 = pu.mvpIdx[REF_PIC_LIST_1];
          Mv orgMvd0 = pu.mvd[0];
          Mv orgMvd1 = pu.mvd[1];
          // this part is to derive the merge info
#if JVET_AD0213_LIC_IMP
          m_pcInterPred->getAmvpMergeModeMergeList(pu, m_mvFieldAmListDec, m_licAmListDec, orgRefIdxAMVP);
          const int mvFieldMergeIdx = orgRefIdxAMVP * AMVP_MAX_NUM_CANDS_MEM + (pu.amvpMergeModeFlag[REF_PIC_LIST_0] ? orgMvpIdxL1 : orgMvpIdxL0);
          const int mvFieldAmvpIdx = MAX_NUM_AMVP_CANDS_MAX_REF + mvFieldMergeIdx;
          CHECK(m_licAmListDec[mvFieldMergeIdx] != m_licAmListDec[mvFieldAmvpIdx], "inconsistent LIC value");
          pu.cu->licFlag = m_licAmListDec[mvFieldMergeIdx];
#else
          m_pcInterPred->getAmvpMergeModeMergeList(pu, m_mvFieldAmListDec, orgRefIdxAMVP);
#endif
          // if there was set PU merge info, restore the AMVP information
          pu.mvpIdx[REF_PIC_LIST_0] = orgMvpIdxL0;
          pu.mvpIdx[REF_PIC_LIST_1] = orgMvpIdxL1;
          pu.interDir = orgInterDir;
          pu.mergeFlag = false;
          pu.mvd[0] = orgMvd0;
          pu.mvd[1] = orgMvd1;
          pu.refIdx[refListAmvp] = orgRefIdxAMVP;
        }
#endif
        if( pu.cu->affine )
        {
          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ( pu.cs->slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir & ( 1 << uiRefListIdx ) ) )
            {
              AffineAMVPInfo affineAMVPInfo;
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
              PU::fillAffineMvpCand( m_pcInterPred, pu, eRefList, pu.refIdx[eRefList], affineAMVPInfo
#else
              PU::fillAffineMvpCand( pu, eRefList, pu.refIdx[eRefList], affineAMVPInfo 
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
                , m_pcInterPred
#endif  
              );
              const unsigned mvpIdx = pu.mvpIdx[eRefList];

              pu.mvpNum[eRefList] = affineAMVPInfo.numCand;

              //    Mv mv[3];
              CHECK( pu.refIdx[eRefList] < 0, "Unexpected negative refIdx." );
#if JVET_Z0054_BLK_REF_PIC_REORDER
              if (!pu.cs->sps->getUseARL())
              {
#else
              if (!cu.cs->pcv->isEncoder)
              {
                pu.mvdAffi[eRefList][0].changeAffinePrecAmvr2Internal(pu.cu->imv);
                pu.mvdAffi[eRefList][1].changeAffinePrecAmvr2Internal(pu.cu->imv);
                if (cu.affineType == AFFINEMODEL_6PARAM)
                {
                  pu.mvdAffi[eRefList][2].changeAffinePrecAmvr2Internal(pu.cu->imv);
                }
              }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
              if (pu.isMvdPredApplicable())
              {
                Mv absMvd[3];
                absMvd[0] = Mv(pu.mvdAffi[eRefList][0].getAbsMv());
                absMvd[1] = Mv(pu.mvdAffi[eRefList][1].getAbsMv());
                absMvd[2] = (cu.affineType == AFFINEMODEL_6PARAM) ? Mv(pu.mvdAffi[eRefList][2].getAbsMv()) : Mv(0, 0);
                if ((absMvd[0] != Mv(0, 0) || absMvd[1] != Mv(0, 0) || absMvd[2] != Mv(0, 0)) && pu.isMvdPredApplicable())
                {
#if JVET_AD0140_MVD_PREDICTION 
                  std::vector<Mv> cMvdDerived[3];
                  m_pcInterPred->deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx],
                    absMvd, pu, eRefList, pu.refIdx[eRefList], cMvdDerived[0], cMvdDerived[1], cMvdDerived[2]);
                  CHECK(pu.mvsdIdx[eRefList] >= cMvdDerived[0].size(), "");
                  std::vector<Mv> cMvd = { Mv(0, 0), Mv(0, 0), Mv(0, 0) };
                  m_pcInterPred->deriveMVDFromMVSDIdxAffineSI(pu, eRefList, cMvdDerived[0], cMvdDerived[1], cMvdDerived[2], cMvd);
                  m_pcInterPred->applyOffsetsAffineMvd(pu, eRefList, cMvd, cMvdDerived);
                  for (int i = 0; i < 3; ++i)
                  {
                    pu.mvdAffi[eRefList][i] = cMvd[i];
                  }
#else
                  std::vector<Mv> cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3;
#if JVET_Z0054_BLK_REF_PIC_REORDER
                  m_pcInterPred->deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx],
                    absMvd, pu, eRefList, pu.refIdx[eRefList], cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#else
                  m_pcInterPred->deriveMvdSignAffine(affineAMVPInfo.mvCandLT[mvpIdx], affineAMVPInfo.mvCandRT[mvpIdx], affineAMVPInfo.mvCandLB[mvpIdx],
                    absMvd[0], absMvd[1], absMvd[2], pu, eRefList, pu.refIdx[eRefList], cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#endif
                  CHECK(pu.mvsdIdx[eRefList] >= cMvdDerivedVec.size(), "");
                  m_pcInterPred->deriveMVDFromMVSDIdxAffine(pu, eRefList, cMvdDerivedVec, cMvdDerivedVec2, cMvdDerivedVec3);
#endif
                }
              }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
              }
#endif
              Mv mvLT = affineAMVPInfo.mvCandLT[mvpIdx] + pu.mvdAffi[eRefList][0];
              Mv mvRT = affineAMVPInfo.mvCandRT[mvpIdx] + pu.mvdAffi[eRefList][1];
              mvRT += pu.mvdAffi[eRefList][0];

              Mv mvLB;
              if ( cu.affineType == AFFINEMODEL_6PARAM )
              {
                mvLB = affineAMVPInfo.mvCandLB[mvpIdx] + pu.mvdAffi[eRefList][2];
                mvLB += pu.mvdAffi[eRefList][0];
              }

              pu.mvAffi[eRefList][0] = mvLT;
              pu.mvAffi[eRefList][1] = mvRT;
              pu.mvAffi[eRefList][2] = mvLB;
            }
          }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
          if (PU::checkDoAffineBdofRefine(pu, m_pcInterPred))
          {
            pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
            m_pcInterPred->setDoAffineSubPuBdof(false);
            m_pcInterPred->setBdofSubPuMvBuf(m_mvBufDecAffineBDOF);
          }
#endif
        }
#if JVET_AE0169_BIPREDICTIVE_IBC
        else if (CU::isIBC(*pu.cu))
#else
        else if (CU::isIBC(*pu.cu) && pu.interDir == 1)
#endif
        {
          AMVPInfo amvpInfo;
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AE0169_BIPREDICTIVE_IBC
          PU::fillIBCMvpCand(pu, amvpInfo, mrgCtx, m_pcInterPred);
#else
          PU::fillIBCMvpCand(pu, amvpInfo, m_pcInterPred);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          if (pu.isBvpClusterApplicable())
          {
#if JVET_AA0070_RRIBC
            if (pu.cu->rribcFlipType == 0)
#endif
            {
              if (pu.cu->bvZeroCompDir == 1)
              {
                for (int i = 0; i < 2; i++)
                {
                  amvpInfo.mvCand[i] = Mv(i == 0 ? std::max(-(int) pu.lwidth(), -pu.Y().x) : -pu.Y().x, 0);
                  amvpInfo.mvCand[i].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
                }
              }
              else if (pu.cu->bvZeroCompDir == 2)
              {
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
                for (int i = 0; i < 2; i++)
                {
                  amvpInfo.mvCand[i] = Mv(0, i == 0 ? std::max(-(int) pu.lheight(), rrTop) : rrTop);
                  amvpInfo.mvCand[i].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
                }
              }
            }
          }
#endif
#else
          PU::fillIBCMvpCand(pu, amvpInfo);
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
          {
            uint8_t imv = pu.cu->imv;
            pu.cu->imv = 0;
            pu.mergeFlag = false;
            PU::getIBCMergeCandidates(pu, mrgCtx);
            uint16_t mergeIdx = pu.mergeIdx;
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_W0090_ARMC_TM
            {
              if (pu.cs->sps->getUseAML())
              {
#if JVET_Z0075_IBC_HMVP_ENLARGE
                m_pcInterPred->adjustIBCMergeCandidates(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#else
                pcInter->adjustIBCMergeCandidates(pu, mergeCtx);
#endif
              }
              m_pcInterPred->adjustIbcMergeRribcCand(pu, mrgCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
              pu.cu->rribcFlipType = 0;
              pu.cu->ibcLicFlag = 0;
#if JVET_AE0159_FIBC
              pu.cu->ibcFilterFlag = false;
#endif
            }
#endif
            pu.cu->imv = imv;
            pu.mergeFlag = false;
            pu.mergeIdx = mergeIdx;
          }
#endif
          pu.mvpNum[REF_PIC_LIST_0] = amvpInfo.numCand;

          Mv mvd = pu.mvd[REF_PIC_LIST_0];
#if !JVET_Z0054_BLK_REF_PIC_REORDER
#if REUSE_CU_RESULTS
          if (!cu.cs->pcv->isEncoder)
#endif
          {
            mvd.changeIbcPrecAmvr2Internal(pu.cu->imv);
          }
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
          if (pu.isBvdPredApplicable() && mvd.isMvdPredApplicable())
          {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
            Mv cMvPred = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]];
            cMvPred.regulateMv(pu.getBvType());
#else
#if JVET_AA0070_RRIBC
            auto cMvPred = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]];
            if (pu.cu->rribcFlipType == 1)
            {
              cMvPred.setVer(0);
            }
            else if (pu.cu->rribcFlipType == 2)
            {
              cMvPred.setHor(0);
            }
#else
            const auto cMvPred = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]];
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            pu.bvdSuffixInfo.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
            pu.bvdSuffixInfo.initPrefixes(mvd, pu.cu->imv, true);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            static std::vector<Mv> cMvdDerivedVec;
            cMvdDerivedVec.resize(0);
#else
            std::vector<Mv> cMvdDerivedVec;
#endif
            m_pcInterPred->deriveBvdSignIBC(cMvPred, mvd, pu, cMvdDerivedVec, pu.cu->imv);
            CHECK(pu.mvsdIdx[REF_PIC_LIST_0] >= cMvdDerivedVec.size(), "pu.mvsdIdx[REF_PIC_LIST_0] out of range");
            int mvsdIdx = pu.mvsdIdx[REF_PIC_LIST_0];
            mvd = m_pcInterPred->deriveMVDFromMVSDIdxTransIBC(mvsdIdx, cMvdDerivedVec, pu.bvdSuffixInfo);
            m_pcInterPred->applyOffsets(mvd, cMvdDerivedVec, pu.bvdSuffixInfo, pu.cu->imv);
            CHECK(mvd == Mv(0, 0), " zero MVD!");
          }
#endif
          if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
          {
            CHECK( pu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
          }
          pu.mv[REF_PIC_LIST_0] = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]] + mvd;
          pu.mv[REF_PIC_LIST_0].mvCliptoStorageBitDepth();
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV || JVET_AA0070_RRIBC)
          pu.mv[REF_PIC_LIST_0].regulateMv(pu.getBvType());
#else
#if JVET_AA0070_RRIBC
          if (pu.cu->rribcFlipType == 1)
          {
            pu.mv[REF_PIC_LIST_0].setVer(0);
          }
          else if (pu.cu->rribcFlipType == 2)
          {
            pu.mv[REF_PIC_LIST_0].setHor(0);
          }
#endif
#endif
#if JVET_Z0160_IBC_ZERO_PADDING
          pu.bv = pu.mv[0];
          pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
          {
            mrgCtx.setIbcL1Info(pu, pu.mergeIdx);
          }
#endif
        }
        else
        {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
          Mv cMvpL0;
#endif
          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            RefPicList eRefList = RefPicList( uiRefListIdx );
            if ((pu.cs->slice->getNumRefIdx(eRefList) > 0 || (eRefList == REF_PIC_LIST_0 && CU::isIBC(*pu.cu))) && (pu.interDir & (1 << uiRefListIdx)))
            {
              AMVPInfo amvpInfo;
#if JVET_X0083_BM_AMVP_MERGE_MODE
              if (pu.amvpMergeModeFlag[eRefList] == true)
              {
#if REUSE_CU_RESULTS
                if (!cu.cs->pcv->isEncoder)
                {
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
                const int mvFieldMergeIdx = pu.refIdx[1 - eRefList] * AMVP_MAX_NUM_CANDS_MEM + pu.mvpIdx[1 - eRefList];
#else
                const int mvFieldMergeIdx = pu.refIdx[1 - eRefList] * AMVP_MAX_NUM_CANDS + pu.mvpIdx[1 - eRefList];
#endif
                pu.mv[eRefList] = m_mvFieldAmListDec[mvFieldMergeIdx].mv;
                pu.refIdx[eRefList] = m_mvFieldAmListDec[mvFieldMergeIdx].refIdx;
#if REUSE_CU_RESULTS
                }
#endif
              }
              else
              {

                if (pu.amvpMergeModeFlag[1 - eRefList] == true)
                {
#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
                  amvpInfo.numCand = PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) ? 1 : 2;
#else
                  amvpInfo.numCand = 1;
#endif
#else
                  amvpInfo.numCand = AMVP_MAX_NUM_CANDS;
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
                  amvpInfo.numCand += 1;
#endif
#if REUSE_CU_RESULTS
                  if (cu.cs->pcv->isEncoder)
                  {
                    amvpInfo.mvCand[pu.mvpIdx[eRefList]] = pu.mv[eRefList] - pu.mvd[eRefList];
                  }
                  else
#endif
                  {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
                    const int mvFieldAmvpIdx = MAX_NUM_AMVP_CANDS_MAX_REF + pu.refIdx[eRefList] * AMVP_MAX_NUM_CANDS_MEM + pu.mvpIdx[eRefList];
#else
                    const int mvFieldAmvpIdx = MAX_NUM_AMVP_CANDS_MAX_REF + pu.refIdx[eRefList] * AMVP_MAX_NUM_CANDS + pu.mvpIdx[eRefList];
#endif
                    amvpInfo.mvCand[pu.mvpIdx[eRefList]] = m_mvFieldAmListDec[mvFieldAmvpIdx].mv;
                  }
                }
                else
#endif
              PU::fillMvpCand(pu, eRefList, pu.refIdx[eRefList], amvpInfo
#if TM_AMVP
                            , m_pcInterPred
#endif
              );
              pu.mvpNum [eRefList] = amvpInfo.numCand;
#if JVET_Z0054_BLK_REF_PIC_REORDER
#if JVET_X0083_BM_AMVP_MERGE_MODE
              if( (!PU::useRefCombList(pu) && !PU::useRefPairList(pu)) || (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]))
#else
              if(!PU::useRefCombList(pu) && !PU::useRefPairList(pu))
#endif
              {
#else
              if (!cu.cs->pcv->isEncoder)
              {
                pu.mvd[eRefList].changeTransPrecAmvr2Internal(pu.cu->imv);
              }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
#if JVET_AG0098_AMVP_WITH_SBTMVP
              if (pu.isMvdPredApplicable() && pu.mvd[eRefList].isMvdPredApplicable() && !pu.amvpSbTmvpFlag)
#else
              if (pu.isMvdPredApplicable() && pu.mvd[eRefList].isMvdPredApplicable())
#endif
              {
                if (pu.cu->smvdMode)
                {
                  if (eRefList == REF_PIC_LIST_0)
                  {
                    cMvpL0 = amvpInfo.mvCand[pu.mvpIdx[eRefList]];
                  }
                  else
                  {
                    std::vector<Mv> cMvdDerivedVec;
#if JVET_AD0140_MVD_PREDICTION
                    Mv cMvdKnownAtDecoder = Mv(pu.mvd[REF_PIC_LIST_0].getHor(), pu.mvd[REF_PIC_LIST_0].getVer());
#else
                    Mv cMvdKnownAtDecoder = Mv(pu.mvd[eRefList].getAbsHor(), pu.mvd[eRefList].getAbsVer());
#endif
                    m_pcInterPred->deriveMvdSignSMVD(cMvpL0, amvpInfo.mvCand[pu.mvpIdx[1]], cMvdKnownAtDecoder, pu, cMvdDerivedVec); //pass the absolute Mvd value as MVd may be negative while CU reuse at encoder.
#if JVET_AD0140_MVD_PREDICTION
                    if (!cMvdDerivedVec.empty())
                    {
#endif
                    CHECK(pu.mvsdIdx[REF_PIC_LIST_0] >= cMvdDerivedVec.size(), "");
#if JVET_AD0140_MVD_PREDICTION
                    }
#endif
                    int mvsdIdx = pu.mvsdIdx[REF_PIC_LIST_0];

#if JVET_AD0140_MVD_PREDICTION
                    Mv cMvd = cMvdKnownAtDecoder;
                    if (!cMvdDerivedVec.empty())
                    {
                      cMvd = m_pcInterPred->deriveMVDFromMVSDIdxTransSI(mvsdIdx, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0]);
                      m_pcInterPred->applyOffsetsMvd(cMvd, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0], pu.cu->imv);
                    }
#else
                    Mv cMvd = m_pcInterPred->deriveMVDFromMVSDIdxTrans(mvsdIdx, cMvdDerivedVec);
#endif
                    CHECK(cMvd == Mv(0, 0), " zero MVD for SMVD!");
                    pu.mvd[REF_PIC_LIST_0] = cMvd;
                    pu.mv[REF_PIC_LIST_0] = cMvpL0 + pu.mvd[REF_PIC_LIST_0];
                    pu.mv[REF_PIC_LIST_0].mvCliptoStorageBitDepth();
                    pu.mvd[REF_PIC_LIST_1].set(-pu.mvd[REF_PIC_LIST_0].hor, -pu.mvd[REF_PIC_LIST_0].ver);
                  }
                }
                else
                {
                  std::vector<Mv> cMvdDerivedVec;
#if JVET_AD0140_MVD_PREDICTION
                  Mv cMvdKnownAtDecoder = Mv(pu.mvd[eRefList].getHor(), pu.mvd[eRefList].getVer());
#else
                  Mv cMvdKnownAtDecoder = Mv(pu.mvd[eRefList].getAbsHor(), pu.mvd[eRefList].getAbsVer());
#endif
                  m_pcInterPred->deriveMvdSign(amvpInfo.mvCand[pu.mvpIdx[eRefList]], cMvdKnownAtDecoder, pu, eRefList, pu.refIdx[eRefList], cMvdDerivedVec); //pass the absolute Mvd value as MVd may be negative while CU reuse at encoder.
                  CHECK(pu.mvsdIdx[eRefList] >= cMvdDerivedVec.size(), "");
                  int mvsdIdx = pu.mvsdIdx[eRefList];
#if JVET_AD0140_MVD_PREDICTION
                  Mv cMvd = cMvdKnownAtDecoder;
                  if (!cMvdDerivedVec.empty())
                  {
                    cMvd = m_pcInterPred->deriveMVDFromMVSDIdxTransSI(mvsdIdx, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[eRefList][0]);
                    m_pcInterPred->applyOffsetsMvd(cMvd, cMvdDerivedVec, pu.mvdSuffixInfo.mvBins[eRefList][0], pu.cu->imv);
                  }
#else
                  Mv cMvd = m_pcInterPred->deriveMVDFromMVSDIdxTrans(mvsdIdx, cMvdDerivedVec);
#endif
                  CHECK(cMvd == Mv(0, 0), " zero MVD!");
                  pu.mvd[eRefList] = cMvd;
                }
              }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
              }
#endif
              pu.mv[eRefList] = amvpInfo.mvCand[pu.mvpIdx[eRefList]] + pu.mvd[eRefList];
              pu.mv[eRefList].mvCliptoStorageBitDepth();
#if JVET_X0083_BM_AMVP_MERGE_MODE
              }
#endif
            }
          }
#if JVET_AG0098_AMVP_WITH_SBTMVP
          if (pu.amvpSbTmvpFlag)
          {
            if (mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX].area() == 0 || !mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX].buf)
            {
              Size bufSize = g_miScaling.scale(pu.lumaSize());
              mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX] = MotionBuf(m_subPuMiBuf[AMVP_SBTMVP_BUFFER_IDX], bufSize);
            }
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            PU::getAmvpSbTmvp(m_pcInterPred, pu, mrgCtx, pu.interDir == 1 ? pu.mv[0] : pu.mv[1]);
#else
            PU::getAmvpSbTmvp(pu, mrgCtx, pu.interDir == 1 ? pu.mv[0] : pu.mv[1]);
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
            if (PU::checkDoAffineBdofRefine(pu, m_pcInterPred))
            {
              pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
              m_pcInterPred->setDoAffineSubPuBdof(false);
              m_pcInterPred->setBdofSubPuMvBuf(m_mvBufDecAffineBDOF);
              int bioSubPuIdx = 0;
              const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
              for (int yy = 0; yy < pu.lumaSize().height; yy += 4)
              {
                for (int xx = 0; xx < pu.lumaSize().width; xx += 4)
                {
                  m_mvBufDecAffineBDOF[bioSubPuIdx].setZero();
                  bioSubPuIdx++;
                }
                bioSubPuIdx += bioSubPuStrideIncr;
              }
            }
#endif
          }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
          if ((pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1]) && PU::checkBDMVRCondition(pu))
          {
            m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
            pu.bdmvrRefine = true;
            // span motion to subPU
            for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
            {
              m_mvBufBDMVR[0][subPuIdx] = pu.mv[0];
              m_mvBufBDMVR[1][subPuIdx] = pu.mv[1];
            }
          }
#endif
        }
#if JVET_X0083_BM_AMVP_MERGE_MODE
        if (!pu.bdmvrRefine)
#endif
          PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , pu.colIdx
#endif
          );
      }
    }
    if( !cu.geoFlag )
    {
      if( g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint( pu, true ) )
      {
        printf( "DECODER: pu motion vector across tile boundaries (%d,%d,%d,%d)\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight() );
      }
    }
    if (CU::isIBC(cu))
    {
      const int cuPelX = pu.Y().x;
      const int cuPelY = pu.Y().y;
      int roiWidth = pu.lwidth();
      int roiHeight = pu.lheight();
      const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
      int xPred = pu.mv[0].getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
      int yPred = pu.mv[0].getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
#if JVET_Z0118_GDR
      if (cu.cs->isGdrEnabled())
      {
        if (cu.cs->isClean(cu)) 
        {
          CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred), "invalid block vector for IBC detected.");
        }
      }
      else 
      {
        CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred), "invalid block vector for IBC detected.");
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3)
      {
        xPred = pu.mv[1].getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
        yPred = pu.mv[1].getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
        if (cu.cs->isGdrEnabled())
        {
          if (cu.cs->isClean(cu)) 
          {
            CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred), "invalid block vector for IBC detected.");
          }
        }
        else 
        {
          CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred), "invalid block vector for IBC detected.");
        }
      }
#endif
#else
      CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred), "invalid block vector for IBC detected.");
#endif
    }
#if MULTI_HYP_PRED
    {
      bool derivedMrgList = false;
#if REUSE_CU_RESULTS
      if (!cu.cs->pcv->isEncoder)
#endif
      for (int i = pu.numMergedAddHyps; i < int(pu.addHypData.size()); ++i)
      {
        auto &mhData = pu.addHypData[i];
#if INTER_LIC
        mhData.licFlag = pu.cu->licFlag;
#endif
        mhData.imv = pu.cu->imv;
        if (mhData.isMrg)
        {
          if (!derivedMrgList)
          {
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
            PU::getGeoMergeCandidates(pu, m_geoMrgCtx, m_pcInterPred);
#else
            PU::getGeoMergeCandidates(pu, m_geoMrgCtx);
#endif
            derivedMrgList = true;
          }
          int refList = m_geoMrgCtx.interDirNeighbours[mhData.mrgIdx] - 1; CHECK(refList != 0 && refList != 1, "");
          mhData.refIdx = m_geoMrgCtx.mvFieldNeighbours[(mhData.mrgIdx << 1) + refList].refIdx;
          mhData.mv = m_geoMrgCtx.mvFieldNeighbours[(mhData.mrgIdx << 1) + refList].mv;
          mhData.refList = refList;
          mhData.imv = m_geoMrgCtx.useAltHpelIf[mhData.mrgIdx] ? IMV_HPEL : 0;
          continue;
        }
        if (pu.cu->affine)
        {
          mhData.mvd.changeAffinePrecAmvr2Internal(pu.cu->imv);
        }
        else
        {
          mhData.mvd.changeTransPrecAmvr2Internal(pu.cu->imv);
        }
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
        const Mv mvp = PU::getMultiHypMVP(m_pcInterPred, pu, mhData);
#else
        const Mv mvp = PU::getMultiHypMVP(pu, mhData);
#endif
        mhData.mv = mvp + mhData.mvd;
        mhData.mv.mvCliptoStorageBitDepth();
      }
    }
#endif
  }
}
//! \}
