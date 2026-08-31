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

#include "EncLoopFilterCccm.h"

#if JVET_AL0153_ALF_CCCM
void EncLoopFilterCccm::lfCccmRDO(CodingStructure& cs, const PelUnitBuf recSAO
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  , PelUnitBuf recYuv
#endif
  , CtxCache* ctxCache, CABACEncoder* cabacEncoder, Slice* pcSlice)
{
  m_ctxCache = ctxCache;
  m_CABACEstimator = cabacEncoder->getCABACEstimator(pcSlice->getSPS());
  m_CABACEstimator->initCtxModels(*pcSlice);
  m_CABACEstimator->resetBits();

  lfCccmAllocateArraysEncoder(cs.pcv->maxCUWidth, cs.pcv->maxCUHeight);
  lfCccmCreatePelStorage(cs);

  cs.slice->lfCccmClearControlInformation();

  PelStorage localPelStorage;
  localPelStorage.create(CHROMA_ONLY_420, Area(0, 0, cs.picture->lwidth(), cs.picture->lheight()));
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  PelStorage localPelStorageImp[MAX_NUM_FACTOR_LF_CCCM];
  PelUnitBuf* localPelUnitBufImpPtr[MAX_NUM_FACTOR_LF_CCCM] = { nullptr };
  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    localPelStorageImp[factorIdx].create(CHROMA_ONLY_420, Area(0, 0, cs.picture->lwidth(), cs.picture->lheight()));
    localPelUnitBufImpPtr[factorIdx] = &localPelStorageImp[factorIdx];
  }
#endif
  const PreCalcValues pcv = *cs.pcv;
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  PelStorage recYuvInheritStorage;
#endif
  const bool doFrameLevelInherit = !cs.slice->isIntra() && cs.slice->lfCccmGetReferencePicture();

  // collect distortions
  RdCost rdcost;
  struct lfCccmEncCandidate
  {
    int ctuRsAddr;
    int uselfCccm;
    int windowSize;
    int modelType;
    int ctuMerge;
    int frameLevelInherit;
    uint64_t distCb;
    uint64_t distCr;
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    uint64_t distImpCb[MAX_NUM_FACTOR_LF_CCCM];
    uint64_t distImpCr[MAX_NUM_FACTOR_LF_CCCM];
#else
    CPelBuf bufCb;
    CPelBuf bufCr;
#endif
    bool isEqualCand(const lfCccmEncCandidate& test)
    {
      if (test.uselfCccm != uselfCccm)
      {
        return false;
      }
      if (test.windowSize != windowSize)
      {
        return false;
      }
      if (test.modelType != modelType)
      {
        return false;
      }
      if (test.ctuRsAddr != ctuRsAddr)
      {
        return false;
      }
      return true;
    };
  };

  const TempCtx ctxStartlfCccm1(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
  auto resetCtx1 = [&]()
  {
    m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm1);
  };

  cs.slice->setLfCccmEnabledFlag(true);

  const double lambda = (cs.slice->getLambdas()[COMPONENT_Cb] + cs.slice->getLambdas()[COMPONENT_Cr]) / 4.0;

  double frameLevelInheritCost = MAX_DOUBLE;
  if (doFrameLevelInherit)
  {
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->lfCccm(cs, 0);
    frameLevelInheritCost = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
    cs.slice->m_lfCccmFrameLevelInherit = 0;
    resetCtx1();
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    recYuvInheritStorage.create(CHROMA_ONLY_420, Area(0, 0, cs.picture->lwidth(), cs.picture->lheight()));
    recYuvInheritStorage.copyFrom(recYuv, false, true);
#endif
  }
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  std::vector<std::vector<lfCccmEncCandidate>> rdoCandidates;
  rdoCandidates.resize(cs.picture->m_ctuNums);
#else
  double totalCost = 0;
  double totalCostOff = 0;
#endif
  for (int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
  {

    bool ctuProcessedEnc = false;

    const int xc = ctuRsAddr % pcv.widthInCtus;
    const int yc = ctuRsAddr / pcv.widthInCtus;
    const UnitArea ctuArea = clipArea(UnitArea(pcv.chrFormat, Area(xc << pcv.maxCUWidthLog2, yc << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth)), *cs.slice->getPic());
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    UnitArea bufArea = ctuArea;
    bufArea.repositionTo(UnitArea(pcv.chrFormat, Area(0, 0, bufArea.lwidth(), bufArea.lheight())));
#endif
    CPelBuf origCb = cs.picture->getTrueOrigBuf(ctuArea.Cb());
    CPelBuf origCr = cs.picture->getTrueOrigBuf(ctuArea.Cr());

    CPelBuf saoCb = recSAO.subBuf(ctuArea).Cb();
    CPelBuf saoCr = recSAO.subBuf(ctuArea).Cr();
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    std::vector<lfCccmEncCandidate> rdoCandidates;
#endif
    // initial distortion
    lfCccmEncCandidate curCand;
    curCand.ctuRsAddr = ctuRsAddr;
    curCand.uselfCccm = 0;
    curCand.windowSize = 0;
    curCand.modelType = 0;
    curCand.ctuMerge = 0;
    curCand.frameLevelInherit = 0;
    curCand.distCb = rdcost.getDistPart(origCb, saoCb, cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
    curCand.distCr = rdcost.getDistPart(origCr, saoCr, cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
    {
      curCand.distImpCb[factorIdx] = curCand.distCb;
      curCand.distImpCr[factorIdx] = curCand.distCr;
    }
    rdoCandidates.at(ctuRsAddr).push_back(curCand);
#else
    curCand.bufCb = recSAO.subBuf(ctuArea).Cb();
    curCand.bufCr = recSAO.subBuf(ctuArea).Cr();
    rdoCandidates.push_back(curCand);
#endif
    // regular candidates (full set)
    cs.slice->lfCccmClearControlInformation(ctuRsAddr);
    for (int windowSizeIndx = 0; windowSizeIndx < m_lfCccmMaxNumWindows; windowSizeIndx++)
    {
      for (int modelTypeIndx = 0; modelTypeIndx < m_lfCccmMaxNumModels; modelTypeIndx++)
      {
        cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = 1;
        cs.slice->m_lfCccmWindowSizeIndex[ctuRsAddr] = windowSizeIndx;
        cs.slice->m_lfCccmModelType[ctuRsAddr] = modelTypeIndx;

        localPelStorage.subBuf(ctuArea).Cb().copyFrom(saoCb);
        localPelStorage.subBuf(ctuArea).Cr().copyFrom(saoCr);
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
        for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
        {
          localPelStorageImp[factorIdx].subBuf(ctuArea).Cb().copyFrom(saoCb);
          localPelStorageImp[factorIdx].subBuf(ctuArea).Cr().copyFrom(saoCr);
        }
        lfCccmCtuProcess(cs, recSAO, ctuRsAddr, localPelStorage, 5, localPelUnitBufImpPtr, true, &ctuProcessedEnc);
#else
        lfCccmCtuProcess(cs, recSAO, ctuRsAddr, localPelStorage.Cb(), localPelStorage.Cr(), &ctuProcessedEnc); // very awkward use of localPelStorage
        m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cb().copyFrom(localPelStorage.subBuf(ctuArea).Cb()); // awkward
        m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cr().copyFrom(localPelStorage.subBuf(ctuArea).Cr());
#endif

        lfCccmEncCandidate curCand;
        curCand.ctuRsAddr = ctuRsAddr;
        curCand.uselfCccm = 1;
        curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
        curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
        curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
        curCand.frameLevelInherit = 0;
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
        curCand.distCb = rdcost.getDistPart(origCb, localPelStorage.subBuf(ctuArea).Cb(), cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
        curCand.distCr = rdcost.getDistPart(origCr, localPelStorage.subBuf(ctuArea).Cr(), cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
        for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
        {
          curCand.distImpCb[factorIdx] = rdcost.getDistPart(origCb, localPelStorageImp[factorIdx].subBuf(ctuArea).Cb(), cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
          curCand.distImpCr[factorIdx] = rdcost.getDistPart(origCr, localPelStorageImp[factorIdx].subBuf(ctuArea).Cr(), cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
        }
        rdoCandidates.at(ctuRsAddr).push_back(curCand);
#else
        curCand.bufCb = m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cb();
        curCand.bufCr = m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cr();
        curCand.distCb = rdcost.getDistPart(origCb, curCand.bufCb, cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
        curCand.distCr = rdcost.getDistPart(origCr, curCand.bufCr, cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Y, DF_SSE);
        rdoCandidates.push_back(curCand);
#endif
      }
    }
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  }
  cs.slice->lfCccmClearControlInformation();
  const TempCtx ctxStartlfCccm3(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
  auto resetCtx3 = [&]()
  {
    m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm3);
  };

  double totalCostBest = 0.0;
  // base mode
  std::vector<int8_t> lfCccmEnabled;
  std::vector<int8_t> lfCccmWindowSizeIndex;
  std::vector<int8_t> lfCccmModelType;
  std::vector<int8_t> lfCccmCTUMerge;
  lfCccmEnabled.resize(cs.picture->m_ctuNums, 0);
  lfCccmWindowSizeIndex.resize(cs.picture->m_ctuNums, 0);
  lfCccmModelType.resize(cs.picture->m_ctuNums, 0);
  lfCccmCTUMerge.resize(cs.picture->m_ctuNums, 0);
  double totalCostBaseMode = 0.0;
  double totalCostOff = 0.0;
  double frameLevelInheritCostBaseMode = frameLevelInheritCost;
  for (int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
  {
    const int numCandRegular = int(rdoCandidates.at(ctuRsAddr).size());
    int delCandsNum = 0;
    // frame-level inherit
    if (doFrameLevelInherit)
    {
      cs.slice->m_lfCccmFrameLevelInherit = 1;
      lfCccmSetFrameLevelInheritedParameters(cs, ctuRsAddr);
      cs.slice->m_lfCccmFrameLevelInherit = 0;

      lfCccmEncCandidate curCand;
      curCand.ctuRsAddr = ctuRsAddr;
      curCand.uselfCccm = cs.slice->m_lfCccmEnabled.at(ctuRsAddr);
      curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
      curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
      curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
      curCand.frameLevelInherit = 1;

      for (int i = 0;i < numCandRegular;i++)
      {
        if (curCand.isEqualCand(rdoCandidates.at(ctuRsAddr).at(i)))
        {
          curCand.distCb = rdoCandidates.at(ctuRsAddr).at(i).distCb;
          curCand.distCr = rdoCandidates.at(ctuRsAddr).at(i).distCr;
          for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
          {
            curCand.distImpCb[factorIdx] = rdoCandidates.at(ctuRsAddr).at(i).distImpCb[factorIdx];
            curCand.distImpCr[factorIdx] = rdoCandidates.at(ctuRsAddr).at(i).distImpCr[factorIdx];
          }
          rdoCandidates.at(ctuRsAddr).push_back(curCand);
          break;
        }
      }
    }
    // merge candidates distortion
    const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
    delCandsNum += nCand;
    for (int cand = 0; cand < nCand; cand++)
    {
      cs.slice->m_lfCccmCTUMerge[ctuRsAddr] = 1 + 2 * cand;
      cs.slice->lfCccmMerge(ctuRsAddr);

      lfCccmEncCandidate curCand;

      curCand.ctuRsAddr = ctuRsAddr;
      curCand.uselfCccm = 1;
      curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
      curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
      curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
      curCand.frameLevelInherit = 0;

      for (int i = 0;i < numCandRegular;i++)
      {
        if (curCand.isEqualCand(rdoCandidates.at(ctuRsAddr).at(i)))
        {
          curCand.distCb = rdoCandidates.at(ctuRsAddr).at(i).distCb;
          curCand.distCr = rdoCandidates.at(ctuRsAddr).at(i).distCr;
          rdoCandidates.at(ctuRsAddr).push_back(curCand);
          break;
        }
      }
    }

    const TempCtx ctxStartlfCccm2(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
    auto resetCtx2 = [&]()
    {
      m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm2);
    };
    double bestCost = MAX_DOUBLE;
    lfCccmEncCandidate bestCand;
    bestCand.ctuRsAddr = ctuRsAddr;
    bestCand.uselfCccm = 0;
    bestCand.windowSize = 0;
    bestCand.modelType = 0;
    bestCand.ctuMerge = 0;

    for (auto curCand : rdoCandidates.at(ctuRsAddr))
    {
      cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = curCand.windowSize;
      cs.slice->m_lfCccmModelType.at(ctuRsAddr) = curCand.modelType;
      cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = curCand.uselfCccm;
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = curCand.ctuMerge;

      const double curDist = (curCand.distCb + curCand.distCr) / 2.0;

      if (curCand.frameLevelInherit == 1)
      {
        frameLevelInheritCostBaseMode += curDist;
        continue;
      }

      resetCtx2();
      m_CABACEstimator->resetBits();
      m_CABACEstimator->lfCccm(cs, ctuRsAddr);
      const double curCost = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits() + curDist;

      if (!curCand.uselfCccm && !curCand.frameLevelInherit)
      {
        totalCostOff += curDist;
      }

      if (curCost < bestCost)
      {
        bestCost = curCost;
        bestCand = curCand;
      }
    }
    // recover list
    for (int delIdx = 0; delIdx < delCandsNum; delIdx++)
    {
      rdoCandidates.at(ctuRsAddr).pop_back();
    }
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = bestCand.windowSize;
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) = bestCand.modelType;
    cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = bestCand.uselfCccm;
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = bestCand.ctuMerge;

    resetCtx2();
    m_CABACEstimator->lfCccm(cs, ctuRsAddr);
    totalCostBaseMode += bestCost;
  }

  // inherit base mode
  int frameLevelInheritBaseMode = 0;
  if (doFrameLevelInherit && (frameLevelInheritCostBaseMode < totalCostBaseMode))
  {
    frameLevelInheritBaseMode = 1;
    totalCostBaseMode = frameLevelInheritCostBaseMode;
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    lfCccmSetFrameLevelInheritedParameters(cs);
    cs.slice->m_lfCccmFrameLevelInherit = 0;
  }

  lfCccmEnabled.swap(cs.slice->m_lfCccmEnabled);
  lfCccmWindowSizeIndex.swap(cs.slice->m_lfCccmWindowSizeIndex);
  lfCccmModelType.swap(cs.slice->m_lfCccmModelType);
  lfCccmCTUMerge.swap(cs.slice->m_lfCccmCTUMerge);

  // imp mode
  std::vector<int8_t> lfCccmEnabledImp;
  std::vector<int8_t> lfCccmWindowSizeIndexImp;
  std::vector<int8_t> lfCccmModelTypeImp;
  std::vector<int8_t> lfCccmCTUMergeImp;
  lfCccmEnabledImp.resize(cs.picture->m_ctuNums, 0);
  lfCccmWindowSizeIndexImp.resize(cs.picture->m_ctuNums, 0);
  lfCccmModelTypeImp.resize(cs.picture->m_ctuNums, 0);
  lfCccmCTUMergeImp.resize(cs.picture->m_ctuNums, 0);

  double totalCostImpModeBest = MAX_DOUBLE;
  int bestFactorIdx = 0;
  double frameLevelInheritCostImpMode[MAX_NUM_FACTOR_LF_CCCM] = { frameLevelInheritCost, frameLevelInheritCost, frameLevelInheritCost, frameLevelInheritCost };

  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    double totalCostImpMode = 0.0;
    cs.slice->lfCccmClearControlInformation();
    resetCtx3();
    for (int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
    {
      const int numCandRegular = int(rdoCandidates.at(ctuRsAddr).size());
      int delCandsNum = 0;
      // merge candidates distortion
      const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
      delCandsNum += nCand;
      for (int cand = 0; cand < nCand; cand++)
      {
        cs.slice->m_lfCccmCTUMerge[ctuRsAddr] = 1 + 2 * cand;
        cs.slice->lfCccmMerge(ctuRsAddr);

        lfCccmEncCandidate curCand;

        curCand.ctuRsAddr = ctuRsAddr;
        curCand.uselfCccm = 1;
        curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
        curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
        curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
        curCand.frameLevelInherit = 0;

        for (int i = 0;i < numCandRegular;i++)
        {
          if (curCand.isEqualCand(rdoCandidates.at(ctuRsAddr).at(i)))
          {
            curCand.distImpCb[factorIdx] = rdoCandidates.at(ctuRsAddr).at(i).distImpCb[factorIdx];
            curCand.distImpCr[factorIdx] = rdoCandidates.at(ctuRsAddr).at(i).distImpCr[factorIdx];
            rdoCandidates.at(ctuRsAddr).push_back(curCand);
            break;
          }
        }
      }

      const TempCtx ctxStartlfCccm2(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
      auto resetCtx2 = [&]()
      {
        m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm2);
      };
      double bestCost = MAX_DOUBLE;
      lfCccmEncCandidate bestCand;
      bestCand.ctuRsAddr = ctuRsAddr;
      bestCand.uselfCccm = 0;
      bestCand.windowSize = 0;
      bestCand.modelType = 0;
      bestCand.ctuMerge = 0;

      for (auto curCand : rdoCandidates.at(ctuRsAddr))
      {
        cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = curCand.windowSize;
        cs.slice->m_lfCccmModelType.at(ctuRsAddr) = curCand.modelType;
        cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = curCand.uselfCccm;
        cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = curCand.ctuMerge;

        const double curDist = (curCand.distImpCb[factorIdx] + curCand.distImpCr[factorIdx]) / 2.0;

        if (curCand.frameLevelInherit == 1)
        {
          frameLevelInheritCostImpMode[factorIdx] += curDist;
          continue;
        }

        resetCtx2();
        m_CABACEstimator->resetBits();
        m_CABACEstimator->lfCccm(cs, ctuRsAddr);
        const double curCost = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits() + curDist;

        if (curCost < bestCost)
        {
          bestCost = curCost;
          bestCand = curCand;
        }
      }
      // recover list
      for (int delIdx = 0; delIdx < delCandsNum; delIdx++)
      {
        rdoCandidates.at(ctuRsAddr).pop_back();
      }
      cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = bestCand.windowSize;
      cs.slice->m_lfCccmModelType.at(ctuRsAddr) = bestCand.modelType;
      cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = bestCand.uselfCccm;
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = bestCand.ctuMerge;

      resetCtx2();
      m_CABACEstimator->lfCccm(cs, ctuRsAddr);
      totalCostImpMode += bestCost;
    }
    if (totalCostImpMode < totalCostImpModeBest)
    {
      totalCostImpModeBest = totalCostImpMode;
      bestFactorIdx = factorIdx;
      lfCccmEnabledImp.swap(cs.slice->m_lfCccmEnabled);
      lfCccmWindowSizeIndexImp.swap(cs.slice->m_lfCccmWindowSizeIndex);
      lfCccmModelTypeImp.swap(cs.slice->m_lfCccmModelType);
      lfCccmCTUMergeImp.swap(cs.slice->m_lfCccmCTUMerge);
    }
  }

  // inherit imp mode
  int bestInheritFactorIdx = 0;
  double frameLevelInheritCostImpModeBest = MAX_DOUBLE;
  if (doFrameLevelInherit)
  {
    for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
    {
      if (frameLevelInheritCostImpMode[factorIdx] < frameLevelInheritCostImpModeBest)
      {
        bestInheritFactorIdx = factorIdx;
        frameLevelInheritCostImpModeBest = frameLevelInheritCostImpMode[factorIdx];
      }
    }
  }
  int frameLevelInheritImpMode = 0;
  if (doFrameLevelInherit && (frameLevelInheritCostImpModeBest < totalCostImpModeBest))
  {
    frameLevelInheritImpMode = 1;
    bestFactorIdx = bestInheritFactorIdx;
    totalCostImpModeBest = frameLevelInheritCostImpModeBest;
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    lfCccmSetFrameLevelInheritedParameters(cs);
    cs.slice->m_lfCccmFrameLevelInherit = 0;
    lfCccmEnabledImp.swap(cs.slice->m_lfCccmEnabled);
    lfCccmWindowSizeIndexImp.swap(cs.slice->m_lfCccmWindowSizeIndex);
    lfCccmModelTypeImp.swap(cs.slice->m_lfCccmModelType);
    lfCccmCTUMergeImp.swap(cs.slice->m_lfCccmCTUMerge);
  }
  totalCostImpModeBest += 2 * lambda;

  if (totalCostImpModeBest < totalCostBaseMode)
  {
    totalCostBest = totalCostImpModeBest;
    cs.slice->setLfCccmImpEnabledFlag(true);
    cs.slice->setLfCccmImpFactorIdx(bestFactorIdx);
    cs.slice->m_lfCccmEnabled.swap(lfCccmEnabledImp);
    cs.slice->m_lfCccmWindowSizeIndex.swap(lfCccmWindowSizeIndexImp);
    cs.slice->m_lfCccmModelType.swap(lfCccmModelTypeImp);
    cs.slice->m_lfCccmCTUMerge.swap(lfCccmCTUMergeImp);
    cs.slice->m_lfCccmFrameLevelInherit = frameLevelInheritImpMode;
  }
  else
  {
    totalCostBest = totalCostBaseMode;
    cs.slice->setLfCccmImpEnabledFlag(false);
    cs.slice->setLfCccmImpFactorIdx(0);
    cs.slice->m_lfCccmEnabled.swap(lfCccmEnabled);
    cs.slice->m_lfCccmWindowSizeIndex.swap(lfCccmWindowSizeIndex);
    cs.slice->m_lfCccmModelType.swap(lfCccmModelType);
    cs.slice->m_lfCccmCTUMerge.swap(lfCccmCTUMerge);
    cs.slice->m_lfCccmFrameLevelInherit = frameLevelInheritBaseMode;
  }

  if (totalCostOff < totalCostBest)
  {
    cs.slice->lfCccmClearControlInformation();
    cs.slice->setLfCccmEnabledFlag(false);
    cs.slice->setLfCccmImpEnabledFlag(false);
    cs.slice->setLfCccmImpFactorIdx(0);
  }
  resetCtx1();
  lfCccmDestroyPelStorage();
  lfCccmDeallocateArraysEncoder();
  localPelStorage.destroy();
  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    localPelStorageImp[factorIdx].destroy();
    localPelUnitBufImpPtr[factorIdx] = nullptr;
  }
  lfCccmEnabled.clear();
  lfCccmWindowSizeIndex.clear();
  lfCccmModelType.clear();
  lfCccmCTUMerge.clear();
  lfCccmEnabledImp.clear();
  lfCccmWindowSizeIndexImp.clear();
  lfCccmModelTypeImp.clear();
  lfCccmCTUMergeImp.clear();
#else
    const int numCandRegular = int(rdoCandidates.size());
    // merge candidates distortion
    const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
    for (int cand = 0; cand < nCand; cand++)
    {
      cs.slice->m_lfCccmCTUMerge[ctuRsAddr] = 1 + 2 * cand;
      cs.slice->lfCccmMerge(ctuRsAddr);

      lfCccmEncCandidate curCand;

      curCand.ctuRsAddr = ctuRsAddr;
      curCand.uselfCccm = 1;
      curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
      curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
      curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
      curCand.frameLevelInherit = 0;

      for (int i = 0;i < numCandRegular;i++)
      {
        if (curCand.isEqualCand(rdoCandidates.at(i)))
        {
          curCand.distCb = rdoCandidates.at(i).distCb;
          curCand.distCr = rdoCandidates.at(i).distCr;
          curCand.bufCb = rdoCandidates.at(i).bufCb;
          curCand.bufCr = rdoCandidates.at(i).bufCr;
          rdoCandidates.push_back(curCand);
          break;
        }
      }
    }
    // frame-level inherit
    if (doFrameLevelInherit)
    {
      cs.slice->m_lfCccmFrameLevelInherit = 1;
      lfCccmSetFrameLevelInheritedParameters(cs, ctuRsAddr);
      cs.slice->m_lfCccmFrameLevelInherit = 0;

      lfCccmEncCandidate curCand;
      curCand.ctuRsAddr = ctuRsAddr;
      curCand.uselfCccm = cs.slice->m_lfCccmEnabled.at(ctuRsAddr);
      curCand.windowSize = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
      curCand.modelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
      curCand.ctuMerge = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
      curCand.frameLevelInherit = 1;

      for (int i = 0;i < numCandRegular;i++)
      {
        if (curCand.isEqualCand(rdoCandidates.at(i)))
        {
          curCand.distCb = rdoCandidates.at(i).distCb;
          curCand.distCr = rdoCandidates.at(i).distCr;
          curCand.bufCb = rdoCandidates.at(i).bufCb;
          curCand.bufCr = rdoCandidates.at(i).bufCr;
          rdoCandidates.push_back(curCand);
          break;
        }
      }
    }
    const TempCtx ctxStartlfCccm2(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
    auto resetCtx2 = [&]()
    {
      m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm2);
    };

    double bestCost = MAX_DOUBLE;
    lfCccmEncCandidate bestCand;

    for (auto curCand : rdoCandidates)
    {
      cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = curCand.windowSize;
      cs.slice->m_lfCccmModelType.at(ctuRsAddr) = curCand.modelType;
      cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = curCand.uselfCccm;
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = curCand.ctuMerge;

      const double curDist = (curCand.distCb + curCand.distCr) / 2.0;

      if (curCand.frameLevelInherit == 1)
      {
        frameLevelInheritCost += curDist;
        recYuvInheritStorage.subBuf(ctuArea).Cb().copyFrom(curCand.bufCb);
        recYuvInheritStorage.subBuf(ctuArea).Cr().copyFrom(curCand.bufCr);
        continue;
      }

      resetCtx2();
      m_CABACEstimator->resetBits();
      m_CABACEstimator->lfCccm(cs, ctuRsAddr);
      const double curCost = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits() + curDist;

      if (!curCand.uselfCccm && !curCand.frameLevelInherit)
      {
        totalCostOff += curDist;
      }

      if (curCost < bestCost)
      {
        bestCost = curCost;
        bestCand = curCand;
      }
    }
    recYuv.subBuf(ctuArea).Cb().copyFrom(bestCand.bufCb);
    recYuv.subBuf(ctuArea).Cr().copyFrom(bestCand.bufCr);
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = bestCand.windowSize;
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) = bestCand.modelType;
    cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = bestCand.uselfCccm;
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = bestCand.ctuMerge;

    resetCtx2();
    m_CABACEstimator->lfCccm(cs, ctuRsAddr);
    totalCost += bestCost;
  }
  if (doFrameLevelInherit && (frameLevelInheritCost < totalCost))
  {
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    lfCccmSetFrameLevelInheritedParameters(cs);
    recYuv.copyFrom(recYuvInheritStorage, false, true);
    totalCost = frameLevelInheritCost;
  }
  if (totalCostOff < totalCost)
  {
    cs.slice->lfCccmClearControlInformation();
    cs.slice->setLfCccmEnabledFlag(false);
    recYuv.copyFrom(cs.getRecoBuf(), false, true);
    totalCost = totalCostOff;
  }
  resetCtx1();
  lfCccmDeallocateArraysEncoder();
#endif
}
#endif
