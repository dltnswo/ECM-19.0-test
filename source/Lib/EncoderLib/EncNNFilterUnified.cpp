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
#if JVET_AL0228_NNLF_INTERFACES

#include "UnitTools.h"
#include "CodingStructure.h"
#include "CABACWriter.h"
#include "EncNNFilterUnified.h"
#include <sadl/model.h>
#if JVET_AK0093_NON_NORMATIVE_TDO
#include <queue>
#endif
using namespace std;


static uint64_t xCalcSSD(const CPelUnitBuf& refBuf, const CPelUnitBuf& cmpBuf)
{
  uint64_t ssd = 0;
  
  for (int comp = 0; comp < MAX_NUM_COMPONENT; ++comp)
  {
    const ComponentID c = ComponentID(comp);
    int width = refBuf.get(c).width;
    int height = refBuf.get(c).height;
    int orgStride = refBuf.get(c).stride;
    int cmpStride = cmpBuf.get(c).stride;
    const Pel* pOrg = refBuf.get(c).buf;
    const Pel* pCmp = cmpBuf.get(c).buf;
    
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        int temp = pOrg[x] - pCmp[x];
        ssd += (temp * temp);
      }
      pOrg += orgStride;
      pCmp += cmpStride;
    }
  }
  return ssd;
}

#if NNVC_JVET_AG0196_CABAC_RETRAIN
void EncNNFilterUnified::initCabac(CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice& slice)
#else
void EncNNFilterUnified::initCabac( CABACEncoder* cabacEncoder, CtxCache* ctxCache, const Slice &slice )
#endif
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( slice.getSPS() );
  m_CtxCache       = ctxCache;
  m_CABACEstimator->initCtxModels( slice );
  m_CABACEstimator->resetBits();
  for(int c = 0; c < MAX_NUM_COMPONENT; ++c)
  {
    m_lambda[c]= slice.getLambdas()[c];
  }
}

void EncNNFilterUnified::setNnlfInferGranularity(const Picture& pic, Slice& pcSlice)
{
  
  NnlfUnifiedInferGranularity inferGranularity = NNLF_UNIFIED_INFER_GRANULARITY_BASE;
  if (pcSlice.getSliceType() == I_SLICE) 
  {
    pcSlice.setNnlfUnifiedInferGranularity(NNLF_UNIFIED_INFER_GRANULARITY_LARGE);
    return;
  }
  if (pcSlice.getSliceQp() < 23) 
  {
    inferGranularity = NNLF_UNIFIED_INFER_GRANULARITY_BASE;
  }
  else if (pcSlice.getSliceQp() < 29)
  {
    inferGranularity   = NNLF_UNIFIED_INFER_GRANULARITY_BASE;
  }
  else
  {
    inferGranularity = pic.getPicWidthInLumaSamples() <= 832 ? NNLF_UNIFIED_INFER_GRANULARITY_BASE : NNLF_UNIFIED_INFER_GRANULARITY_LARGE;
  }

#if NNLF_ALF_POS_INTERFACE
  inferGranularity = NNLF_UNIFIED_INFER_GRANULARITY_LARGE;
#endif  
  pcSlice.setNnlfUnifiedInferGranularity(inferGranularity);
}

#if JVET_AK0093_NON_NORMATIVE_TDO
bool EncNNFilterUnified::getNnlfTDO()
{
  return useNnlfTDO;
}
void EncNNFilterUnified::setNnlfTDO(bool a)
{
  useNnlfTDO = a;
}
std::vector<int> EncNNFilterUnified::getNnlfTDOParam()
{
  return paramTDO;
}
void EncNNFilterUnified::setNnlfTDOParam(const std::vector<int>& v)
{
  paramTDO = v;
}


void EncNNFilterUnified::subPartitionCounter(Picture& pic, const UnitArea& inferArea, uint32_t& numPartition, uint8_t* checkMap)
{
  PelBuf blkBSMap = pic.getBsMapBuf().subBuf(inferArea).get(COMPONENT_Y);
  int width = blkBSMap.width;
  int height = blkBSMap.height;

  std::queue<std::pair<int, int>> quwh;
  uint32_t blkCounter = 0;

  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      if (checkMap[(h * width) + w] != 0) {
        continue;
      }
      Pel bs = blkBSMap.at(w, h);
      if (bs != 0) {
        continue;
      }

      quwh.push({ w,h });
      checkMap[h * width + w] = 1;

      while (!quwh.empty())
      {
        std::pair<int, int>& wh = quwh.front();

        int pw = wh.first;
        int ph = wh.second;

        if (ph - 1 > -1 && checkMap[(ph - 1) * width + pw] == 0 && blkBSMap.at(pw, ph - 1) == 0)
        {
          quwh.push({ pw,ph - 1 });
          checkMap[(ph - 1) * width + pw] = 1;
        }

        if (ph + 1 < height && checkMap[(ph + 1) * width + pw] == 0 && blkBSMap.at(pw, ph + 1) == 0)
        {
          quwh.push({ pw,ph + 1 });
          checkMap[(ph + 1) * width + pw] = 1;
        }

        if (pw - 1 > -1 && checkMap[(ph * width) + pw - 1] == 0 && blkBSMap.at(pw - 1, ph) == 0)
        {
          quwh.push({ pw - 1,ph });
          checkMap[ph * width + pw - 1] = 1;
        }

        if (pw + 1 < width && checkMap[(ph * width) + pw + 1] == 0 && blkBSMap.at(pw + 1, ph) == 0)
        {
          quwh.push({ pw + 1,ph });
          checkMap[ph * width + pw + 1] = 1;
        }
        quwh.pop();
      }
      blkCounter++;
    }
  }
  numPartition = blkCounter;
}

void EncNNFilterUnified::partitionCounter(Picture& pic, std::vector<uint32_t>& numPartitionCounter)
{
  FilterParameters& picprms = getPicprms();
  CodingStructure& cs = *pic.cs;
  const PreCalcValues& pcv = *cs.pcv;

  int cpt = 0;
  uint8_t* checkMap = new uint8_t[picprms.block_size * picprms.block_size];
  for (int y = 0; y < picprms.numBlocksHeight; y++)
  {
    for (int x = 0; x < picprms.numBlocksWidth; x++, cpt++)
    {

      int xPos = x * picprms.block_size;
      int yPos = y * picprms.block_size;

      int width = (xPos + picprms.block_size > (int)pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.block_size;
      int height = (yPos + picprms.block_size > (int)pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.block_size;
      uint32_t blkCounter = 0;
      const UnitArea partArea(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      ::memset(checkMap, 0, sizeof(uint8_t) * picprms.block_size * picprms.block_size);
      subPartitionCounter(pic, partArea, blkCounter, checkMap);
      numPartitionCounter[cpt] = blkCounter;
    }
  }
  delete[] checkMap;
}


double EncNNFilterUnified::getLambdaBLKSkip(int sliceQp)
{

  double decTimeLambda = 0.;

  if (sliceQp < 24) {
    decTimeLambda = paramTDO[0];
  }
  else if (23 < sliceQp && sliceQp < 27) {
    decTimeLambda = paramTDO[1];
  }
  else if (26 < sliceQp && sliceQp < 32) {
    decTimeLambda = paramTDO[2];
  }
  else if (31 < sliceQp && sliceQp < 37) {
    decTimeLambda = paramTDO[3];
  }
  else if (36 < sliceQp && sliceQp < 42) {
    decTimeLambda = paramTDO[4];
  }
  else {
    decTimeLambda = paramTDO[5];
  }

  return decTimeLambda;
}

double EncNNFilterUnified::getSignalingCost(Picture& pic, const std::vector<int>& bsFilteringBlocks)
{
  // calculate RD cost
  const TempCtx  ctxStart(m_CtxCache, SubCtx(Ctx::nnlfUnifiedParams, m_CABACEstimator->getCtx()));
  m_CABACEstimator->getCtx() = SubCtx(Ctx::nnlfUnifiedParams, ctxStart);    // ?
  m_CABACEstimator->resetBits();
  m_CABACEstimator->computeCostNnlfUnifiedParameters(*pic.cs, bsFilteringBlocks);
  double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return rate;
}
#endif
#if JVET_AM0231_NNLF
void EncNNFilterUnified::setAICfg()
{
  cfgAI = true;
}
#endif

double EncNNFilterUnified::getSignalingCost(Picture &pic)
{
  // calculate RD cost
  const TempCtx  ctxStart ( m_CtxCache, SubCtx( Ctx::nnlfUnifiedParams, m_CABACEstimator->getCtx() ) );
  m_CABACEstimator->getCtx() = SubCtx( Ctx::nnlfUnifiedParams, ctxStart );
  m_CABACEstimator->resetBits();
  m_CABACEstimator->writeNnlfUnifiedParameters(*pic.cs);
  double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return rate;
}

void EncNNFilterUnified::chooseParameters(Picture &pic)
{
  // find the best scale and qp offset

#if NNLF_ALF_POS_INTERFACE
  const CodingStructure &cs         = *pic.cs;
  const PreCalcValues& pcv          = *cs.pcv;
  
  FilterParameters& picprms = getPicprms();

  for (int prmId = 0; prmId < picprms.prmNum; prmId++ )
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), prmId);
    filter(pic, false);
    for (int scaleId = 1; scaleId < 3; scaleId++) 
    {
      getScaledBuf(scaleId, prmId).copyFrom(getScaledBuf(0, prmId));
    }
  }
#if JVET_AK0093_NON_NORMATIVE_TDO
  if (useNnlfTDO && cs.slice->getSliceType() != I_SLICE)
  {
    bsPartitionCount.resize(picprms.numBlocksHeight * picprms.numBlocksWidth);
    fill(bsPartitionCount.begin(), bsPartitionCount.begin(), 0);
    partitionCounter(pic, bsPartitionCount);
  }
#endif
  
  double minCost = MAX_DOUBLE;
  std::vector<int> bestPrmId;

  // find best parameters without or with scaling
  for (int scaleId = -1; scaleId < 0; scaleId++) 
  {
    parameterSearch(pic, minCost, bestPrmId, scaleId);
  }
  picprms.prmId = bestPrmId;

  if (picprms.sprm.mode < picprms.prmNum)
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), picprms.sprm.mode);
  }

  int cpt = 0;
  for (int y = 0; y < picprms.numBlocksHeight; ++y)
  {
    for (int x = 0; x < picprms.numBlocksWidth; ++x, ++cpt)
    {
      int xPos   = x * picprms.blockSize;
      int yPos   = y * picprms.blockSize;
      int width  = (xPos + picprms.blockSize > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.blockSize;
      int height = (yPos + picprms.blockSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.blockSize;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      
      if( picprms.prmId[cpt] == -1 )
      {
        continue;
      }

      if (picprms.sprm.scaleFlag != -1)
      {
        int scaleIdx = picprms.sprm.scaleFlag;
        pic.getRecoBuf(block).copyFrom(getScaledBuf(scaleIdx, picprms.prmId[cpt], block));
      }
      else
      {
        pic.getRecoBuf(block).copyFrom(getFilteredBuf(picprms.prmId[cpt], block));
      }
    }
  }

#else

  // find the best scale and qp offset
  const CodingStructure &cs         = *pic.cs;
  const PreCalcValues& pcv          = *cs.pcv;
  
  FilterParameters& picprms = getPicprms();

#if JVET_AH0096_CONTENT_ADAPTIVE_LOP
  bool applyMultiplier = cs.slice->getSliceType() != I_SLICE;
#endif
  for (int prmId = 0; prmId < picprms.prmNum; prmId++ )
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), prmId);
#if JVET_AH0096_CONTENT_ADAPTIVE_LOP
    filter(pic, applyMultiplier, false);
#else
    filter(pic, false);
#endif
    for (int scaleId = 1; scaleId < 3; scaleId++) 
    {
      getScaledBuf(scaleId, prmId).copyFrom(getScaledBuf(0, prmId));
    }
  }
#if JVET_AK0093_NON_NORMATIVE_TDO
  if (useNnlfTDO && cs.slice->getSliceType() != I_SLICE)
  {
    bsPartitionCount.resize(picprms.numBlocksHeight * picprms.numBlocksWidth);
    fill(bsPartitionCount.begin(), bsPartitionCount.begin(), 0);
    partitionCounter(pic, bsPartitionCount);
  }
#endif
  
  double minCost = MAX_DOUBLE;
  std::vector<int> bestPrmId;

  // find best parameters without or with scaling
  for (int scaleId = -1; scaleId < 3; scaleId++)
  {
    scalePicture(pic, scaleId);
    parameterSearch(pic, minCost, bestPrmId, scaleId);
  }
  picprms.prmId = bestPrmId;

  if (picprms.sprm.mode < picprms.prmNum)
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), picprms.sprm.mode);
  }
  
  int cpt = 0;
  for (int y = 0; y < picprms.numBlocksHeight; ++y)
  {
    for (int x = 0; x < picprms.numBlocksWidth; ++x, ++cpt)
    {
      int xPos   = x * picprms.blockSize;
      int yPos   = y * picprms.blockSize;
      int width  = (xPos + picprms.blockSize > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.blockSize;
      int height = (yPos + picprms.blockSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.blockSize;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      
      if( picprms.prmId[cpt] == -1 )
      {
        continue;
      }

      if (picprms.sprm.scaleFlag != -1)
      {
        int scaleIdx = picprms.sprm.scaleFlag;
        pic.getRecoBuf(block).copyFrom(getScaledBuf(scaleIdx, picprms.prmId[cpt], block));
      }
      else
      {
        pic.getRecoBuf(block).copyFrom(getFilteredBuf(picprms.prmId[cpt], block));
      }
    }
  }


#endif

}

void EncNNFilterUnified::parameterSearch(Picture &pic, double &minCost, std::vector<int> &bestPrmId, int scaleId)
{
#if NNLF_ALF_POS_INTERFACE || NNLF_JVET_AK0183_RDO
  CodingStructure&  cs            = *pic.cs;
  const PreCalcValues& pcv        = *cs.pcv;
  
  FilterParameters& picprms = getPicprms();
  const int prmNum = picprms.prmNum;
  
  std::vector<uint64_t> dist(prmNum + 2, 0LL);
  uint64_t* pDist = &dist[1]; // pDist[-1]: slice off distortion, pDist[picprms.prmNum]: block level adaptation distortion, pDist[i]: slice on distortion

#if NNLF_JVET_AK0183_RDO
  std::vector<double> ssdRec(picprms.numBlocksHeight * picprms.numBlocksWidth, 0LL);
  std::map<int, double> euclideanDist;
  double costRec = 0;
#endif

#if JVET_AK0093_NON_NORMATIVE_TDO
  std::vector<int> bsPartitionOn;                       // bsPartitionOn[prmId] -> prmId = 0, 1, ..., prmNum (no prmId = -1 : LOP off)
  std::vector<uint64_t> nonLOPDist;                     // lopDist[cpt]
  std::vector<std::vector<int64_t>> LOPDist(prmNum + 1);  // lopDist[prmId][cpt] // block level selection also check
  std::vector<uint32_t> numBSPartition;
  bool useTDO = useNnlfTDO && cs.slice->getSliceType() != I_SLICE;
#endif
  
  int cpt = 0;
  for (int y = 0; y < picprms.numBlocksHeight; ++y)
  {
    for (int x = 0; x < picprms.numBlocksWidth; ++x, ++cpt)
    {
      int xPos   = x * picprms.blockSize;
      int yPos   = y * picprms.blockSize;
      int width  = (xPos + picprms.blockSize > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.blockSize;
      int height = (yPos + picprms.blockSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.blockSize;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      
      uint64_t distBest = MAX_UINT64;
      int prmIdBest = -1;
      for (int prmId = -1; prmId < prmNum; prmId ++)
      {
        const CPelUnitBuf tempBuf = prmId == -1 ? pic.getRecoBuf(block) : (scaleId == -1 ? getFilteredBuf(prmId, block) : getScaledBuf(scaleId, prmId, block));
        uint64_t distTemp = xCalcSSD(pic.getOrigBuf(block), tempBuf);
#if NNLF_JVET_AK0183_RDO
        if (prmId == -1) 
        {
          ssdRec[cpt] = (double) distTemp;
        }
#endif
#if JVET_AK0093_NON_NORMATIVE_TDO
        if (useTDO)
        {
          if (prmId == -1)
          {
            nonLOPDist.push_back(distTemp);
          }
          else
          {
            lopDist[prmId].push_back(nonlopDist[cpt] - distTemp); // if lopDist[prmId][cpt] has negative sign, it has gain when LOP is NOT activated
          }
        }
#endif
        if (distTemp < distBest)
        {
          distBest = distTemp;
          prmIdBest = prmId;
        }
        pDist[prmId] += distTemp;
      }
      pDist[prmNum] += distBest;
      picprms.prmId[cpt] = prmIdBest;
#if NNLF_JVET_AK0183_RDO
      if ((double)distBest - ssdRec[cpt] < 0)
      { 
        euclideanDist[cpt] = ssdRec[cpt] - (double)distBest;
      }
#endif
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO)
      {
        lopDist[prmNum].push_back(nonlopDist[cpt] - distBest);
      }
#endif
    }
  }

#if JVET_AK0093_NON_NORMATIVE_TDO
  if (useTDO)
  {
    //====================================================
    // TDO LOP get best BS partition value for block skip
    //====================================================

    int sliceQp = cs.slice->getSliceQp();
    int blkNum = (int)bsPartitionCount.size();
    double decTimeLambda;

    decTimeLambda = getLambdaBLKSkip(sliceQp);

    bsFilteringBlocks.resize(prmNum + 1);
    bsFilteringBlocks[prmNum] = picprms.prmId;
    for (int prmId = 0; prmId < prmNum; prmId++)
    {
      fill(bsFilteringBlocks[prmId].begin(), bsFilteringBlocks[prmId].begin(), prmId);
    }

    bsFilteringOn.resize(prmNum + 1);
    fill(bsFilteringOn.begin(), bsFilteringOn.begin(), 0);

    for (int prmId = 0; prmId < prmNum + 1; prmId++)
    {
      int bestPartThreshold = 0;
      uint64_t bestDist = MAX_UINT64;
      double bestCost = MAX_DOUBLE;
      int isBSParitionOn = 0;
      std::vector<int> tempBSFilteringBlocks(blkNum);

      for (int partIdx = 0; partIdx < blkNum; partIdx++)
      {
        CHECK(bsPartitionCount[partIdx] < 1, "number of BS partition should be over zero");

        if (prmId != prmNum)
        {
          std::fill(tempBSFilteringBlocks.begin(), tempBSFilteringBlocks.end(), prmId);
        }
        else
        {
          tempBSFilteringBlocks = picprms.prmId;
        }
        // <numPart> == 1 -> all blocks would be disabled
        uint32_t numPart = bsPartitionCount[partIdx];   // non-normative condition

        for (uint32_t tempPartThreshold = numPart; tempPartThreshold < numPart + 2; tempPartThreshold++)
        {
          if (prmId != prmNum)
          {
            std::fill(tempBSFilteringBlocks.begin(), tempBSFilteringBlocks.end(), prmId);
          }
          else
          {
            tempBSFilteringBlocks = picprms.prmId;
          }

          uint64_t totDist = pDist[prmId];
          int lopDisableCounter = 0;

          for (int cpt = 0; cpt < blkNum; cpt++)
          {
            //=====================
            // LOP deactivate
            //=====================
            if (tempPartThreshold > bsPartitionCount[cpt])
            {
              totDist += lopDist[prmId][cpt];
              lopDisableCounter++;
              tempBSFilteringBlocks[cpt] = -1;
            }
          }

          double cost = (double)totDist + decTimeLambda * (blkNum - lopDisableCounter);
          if (cost < bestCost)
          {
            bestCost = cost;
            bestPartThreshold = tempPartThreshold;
            bestDist = totDist;
            isBSParitionOn = lopDisableCounter == 0 ? 0 : 1;
            bsFilteringBlocks[prmId] = tempBSFilteringBlocks;
            bsFilteringOn[prmId] = isBSParitionOn;
          }
        }
      }
      pDist[prmId] = bestDist;
      numBSPartition.push_back(bestPartThreshold);   // numBSPartition[prmid] (<prmid>: 0,1, ..., <prmNum> )
      bsPartitionOn.push_back(isBSParitionOn);       // bsPartitionOn[prmid] (<prmid>: 0,1, ..., <prmNum> )
    }
  }
#endif

#if JVET_AK0093_NON_NORMATIVE_TDO
  double rateAdpt = 0.;
  double rateScale = MAX_NUM_COMPONENT * (log2ResidueScale + 1); //rate of signalling scales
  if (!useTDO)
  {
    int tempMode = picprms.sprm.mode;
    picprms.sprm.mode = prmNum;
    rateAdpt = getSignalingCost(pic); // rate of adaptation
    picprms.sprm.mode = tempMode;
  }
#else
  int tempMode = picprms.sprm.mode;
  picprms.sprm.mode = prmNum;
  double rateAdpt = getSignalingCost(pic); // rate of adaptation
  double rateScale = MAX_NUM_COMPONENT * (log2ResidueScale + 1); //rate of signalling scales
  picprms.sprm.mode = tempMode;
#endif
  
  // compare R-D cost

  for (int mode = -1; mode < prmNum + 1; mode ++)
  {
#if JVET_AK0093_NON_NORMATIVE_TDO
    if (useTDO)
    {
      rateAdpt = 0.;
      if (mode != -1 && (mode == prmNum || bsPartitionOn[mode]))
      {
        rateAdpt = getSignalingCost(pic, bsFilteringBlocks[mode]);
      }
    }
#endif
    double cost;
    if( mode == -1 )
    {
      cost = ( double ) pDist[mode];
#if NNLF_JVET_AK0183_RDO
      costRec = cost;
#endif
    }
    else if (mode < prmNum)
#if JVET_AF0085_RESIDUAL_ADJ
    {
      int i = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO && bsPartitionOn[mode])
      {
        for (int mode = 0; mode < prmNum; mode++)
        {
          rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
          rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
          rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
        }
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * ((double)(scaleId == 0 ? (rateScale + rateOffset) : (rateOffset)) + rateAdpt);
      }
      else
      {
        rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? (rateScale + rateOffset) : (rateOffset));
      }
#else
      rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
      rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
      rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
      cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? (rateScale + rateOffset) : (rateOffset));
#endif
    }
#else
#if JVET_AK0093_NON_NORMATIVE_TDO
    {
      if (useTDO && bsPartitionOn[mode])
      {
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (rateAdpt + (scaleId == 0 ? prmNum * rateScale : 0));
      }
      else
      {
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? rateScale : 0);
      }
    }
#else
      cost = (double) pDist[mode] + m_lambda[COMPONENT_Y] * ( scaleId == 0 ? rateScale : 0);
#endif
#endif
    else
#if JVET_AF0085_RESIDUAL_ADJ
    {
      int i = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
      for (int mode = 0; mode < prmNum; mode++)
      {
        rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
      }
      cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (rateAdpt + (scaleId == 0 ? (prmNum * rateScale + rateOffset) : (rateOffset)));
    }
#else
      cost = (double) pDist[mode] + m_lambda[COMPONENT_Y] * ( rateAdpt + (scaleId == 0 ? prmNum * rateScale : 0));
#endif

    if (cost < minCost)
   
    {
      minCost = cost;
      picprms.sprm.mode = mode;
      picprms.sprm.scaleFlag = scaleId;
      bestPrmId = picprms.prmId;
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO && mode != -1 && bsPartitionOn[picprms.sprm.mode])
      {
        bestPrmId = bsFilteringBlocks[picprms.sprm.mode];
        picprms.sprm.mode = prmNum;
      }
#endif
    }
  }

#if NNLF_JVET_AK0183_RDO
#if JVET_AM0231_NNLF
  if (minCost > ((cs.slice->getSliceType() == I_SLICE) ? (cfgAI ? I_SLICE_RDO_FACTOR_AI_CFG : I_SLICE_RDO_FACTOR) : NON_I_SLICE_RDO_FACTOR) * costRec)
#else
  if (minCost > ((cs.slice->getSliceType() == I_SLICE) ? 0.97 : 0.99) * costRec)
#endif
  {
    minCost = costRec;
    picprms.sprm.mode = -1;
    picprms.sprm.scaleFlag = -1;
    bestPrmId = picprms.prmId;
    return;
  }
  if( picprms.sprm.mode == 2 && ( scaleId == -1 || scaleId == 2 ) )
  {
    std::vector<pair<int, double>> vtEuclideanDist;
    double                         ratio = 0.01;
    int                            offNum = 0;
    for (auto it = euclideanDist.begin(); it != euclideanDist.end(); it++) 
    {
      vtEuclideanDist.push_back(make_pair(it->first, it->second));
    }
    sort(vtEuclideanDist.begin(), vtEuclideanDist.end(), [](const pair<int, double> &x, const pair<int, double> &y) -> int { return x.second < y.second; });

    for (auto it = vtEuclideanDist.begin(); it != vtEuclideanDist.end(); it++)
    {
      if ((double)(it->second / ssdRec[it->first]) > ratio)
      {
        break;
      }
      offNum++;
    }

    for (int i = 0; i < offNum; i++) 
    {
      bestPrmId[vtEuclideanDist[i].first] = -1;
    }
  }
#endif

#else

  CodingStructure&  cs            = *pic.cs;
  const PreCalcValues& pcv        = *cs.pcv;
  
  FilterParameters& picprms = getPicprms();
  const int prmNum = picprms.prmNum;
  
  std::vector<uint64_t> dist(prmNum + 2, 0LL);
  uint64_t* pDist = &dist[1]; // pDist[-1]: slice off distortion, pDist[picprms.prmNum]: block level adaptation distortion, pDist[i]: slice on distortion
  
#if JVET_AK0093_NON_NORMATIVE_TDO
  std::vector<int> bsPartitionOn;                       // bsPartitionOn[prmId] -> prmId = 0, 1, ..., prmNum (no prmId = -1 : LOP off)
  std::vector<uint64_t> nonLOPDist;                     // lopDist[cpt]
  std::vector<std::vector<int64_t>> LOPDist(prmNum + 1);  // lopDist[prmId][cpt] // block level selection also check
  std::vector<uint32_t> numBSPartition;
  bool useTDO = useNnlfTDO && cs.slice->getSliceType() != I_SLICE;
#endif

  int cpt = 0;
  for (int y = 0; y < picprms.numBlocksHeight; ++y)
  {
    for (int x = 0; x < picprms.numBlocksWidth; ++x, ++cpt)
    {
      int xPos   = x * picprms.blockSize;
      int yPos   = y * picprms.blockSize;
      int width  = (xPos + picprms.blockSize > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.blockSize;
      int height = (yPos + picprms.blockSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.blockSize;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      
      uint64_t distBest = MAX_UINT64;
      int prmIdBest = -1;
      
      for (int prmId = -1; prmId < prmNum; prmId ++)
      {
        const CPelUnitBuf tempBuf = prmId == -1 ? pic.getRecoBuf(block) : (scaleId == -1 ? getFilteredBuf(prmId, block) : getScaledBuf(scaleId, prmId, block));
        uint64_t distTemp = xCalcSSD(pic.getOrigBuf(block), tempBuf);
#if JVET_AF0193_DECODER_OPTIMIZATION
        if (prmId == -1) 
        {
          ssdRec[cpt] = (double) distTemp;
        }
#endif
#if JVET_AK0093_NON_NORMATIVE_TDO
        if (useTDO)
        {
          if (prmId == -1)
          {
            nonLOPDist.push_back(distTemp);
          }
          else
          {
            lopDist[prmId].push_back(nonlopDist[cpt] - distTemp); // if lopDist[prmId][cpt] has negative sign, it has gain when LOP is NOT activated
          }
        }
#endif
        if (distTemp < distBest)
        {
          distBest = distTemp;
          prmIdBest = prmId;
        }
        pDist[prmId] += distTemp;
      }
      pDist[prmNum] += distBest;
      picprms.prmId[cpt] = prmIdBest;
#if JVET_AF0193_DECODER_OPTIMIZATION
      if ((double)distBest - ssdRec[cpt] < 0)
      { 
        euclideanDist[cpt] = ssdRec[cpt] - (double)distBest;
      }
#endif
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO)
      {
        lopDist[prmNum].push_back(nonlopDist[cpt] - distBest);
      }
#endif
    }
  }
  
#if JVET_AK0093_NON_NORMATIVE_TDO
  if (useTDO)
  {
    //====================================================
    // TDO LOP get best BS partition value for block skip
    //====================================================

    int sliceQp = cs.slice->getSliceQp();
    int blkNum = (int)bsPartitionCount.size();
    double decTimeLambda;

    decTimeLambda = getLambdaBLKSkip(sliceQp);

    bsFilteringBlocks.resize(prmNum + 1);
    bsFilteringBlocks[prmNum] = picprms.prmId;
    for (int prmId = 0; prmId < prmNum; prmId++)
    {
      fill(bsFilteringBlocks[prmId].begin(), bsFilteringBlocks[prmId].begin(), prmId);
    }

    bsFilteringOn.resize(prmNum + 1);
    fill(bsFilteringOn.begin(), bsFilteringOn.begin(), 0);

    for (int prmId = 0; prmId < prmNum + 1; prmId++)
    {
      int bestPartThreshold = 0;
      uint64_t bestDist = MAX_UINT64;
      double bestCost = MAX_DOUBLE;
      int isBSParitionOn = 0;
      std::vector<int> tempBSFilteringBlocks(blkNum);

      for (int partIdx = 0; partIdx < blkNum; partIdx++)
      {
        CHECK(bsPartitionCount[partIdx] < 1, "number of BS partition should be over zero");

        if (prmId != prmNum)
        {
          std::fill(tempBSFilteringBlocks.begin(), tempBSFilteringBlocks.end(), prmId);
        }
        else
        {
          tempBSFilteringBlocks = picprms.prmId;
        }
        // <numPart> == 1 -> all blocks would be disabled
        uint32_t numPart = bsPartitionCount[partIdx];   // non-normative condition

        for (uint32_t tempPartThreshold = numPart; tempPartThreshold < numPart + 2; tempPartThreshold++)
        {
          if (prmId != prmNum)
          {
            std::fill(tempBSFilteringBlocks.begin(), tempBSFilteringBlocks.end(), prmId);
          }
          else
          {
            tempBSFilteringBlocks = picprms.prmId;
          }

          uint64_t totDist = pDist[prmId];
          int lopDisableCounter = 0;

          for (int cpt = 0; cpt < blkNum; cpt++)
          {
            //=====================
            // LOP deactivate
            //=====================
            if (tempPartThreshold > bsPartitionCount[cpt])
            {
              totDist += lopDist[prmId][cpt];
              lopDisableCounter++;
              tempBSFilteringBlocks[cpt] = -1;
            }
          }

          double cost = (double)totDist + decTimeLambda * (blkNum - lopDisableCounter);
          if (cost < bestCost)
          {
            bestCost = cost;
            bestPartThreshold = tempPartThreshold;
            bestDist = totDist;
            isBSParitionOn = lopDisableCounter == 0 ? 0 : 1;
            bsFilteringBlocks[prmId] = tempBSFilteringBlocks;
            bsFilteringOn[prmId] = isBSParitionOn;
          }
        }
      }
      pDist[prmId] = bestDist;
      numBSPartition.push_back(bestPartThreshold);   // numBSPartition[prmid] (<prmid>: 0,1, ..., <prmNum> )
      bsPartitionOn.push_back(isBSParitionOn);       // bsPartitionOn[prmid] (<prmid>: 0,1, ..., <prmNum> )
    }
  }
#endif

#if JVET_AK0093_NON_NORMATIVE_TDO
  double rateAdpt = 0.;
  double rateScale = MAX_NUM_COMPONENT * (log2ResidueScale + 1); //rate of signalling scales
  if (!useTDO)
  {
    int tempMode = picprms.sprm.mode;
    picprms.sprm.mode = prmNum;
    rateAdpt = getSignalingCost(pic); // rate of adaptation
    picprms.sprm.mode = tempMode;
  }
#else
  int tempMode = picprms.sprm.mode;
  picprms.sprm.mode = prmNum;
  double rateAdpt = getSignalingCost(pic); // rate of adaptation
  double rateScale = MAX_NUM_COMPONENT * (log2ResidueScale + 1); //rate of signalling scales
  picprms.sprm.mode = tempMode;
#endif
  
  // compare R-D cost
  for (int mode = -1; mode < prmNum + 1; mode ++)
  {
#if JVET_AK0093_NON_NORMATIVE_TDO
    if (useTDO)
    {
      rateAdpt = 0.;
      if (mode != -1 && (mode == prmNum || bsPartitionOn[mode]))
      {
        rateAdpt = getSignalingCost(pic, bsFilteringBlocks[mode]);
      }
    }
#endif
    double cost;
    if (mode == -1)
      cost = (double) pDist[mode];
    else if (mode < prmNum)
#if JVET_AF0085_RESIDUAL_ADJ
    {
      int i = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO && bsPartitionOn[mode])
      {
        for (int mode = 0; mode < prmNum; mode++)
        {
          rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
          rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
          rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
        }
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * ((double)(scaleId == 0 ? (rateScale + rateOffset) : (rateOffset)) + rateAdpt);
      }
      else
      {
        rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? (rateScale + rateOffset) : (rateOffset));
      }
#else
      rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
      rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
      rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
      cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? (rateScale + rateOffset) : (rateOffset));
#endif
    }
#else
#if JVET_AK0093_NON_NORMATIVE_TDO
    {
      if (useTDO && bsPartitionOn[mode])
      {
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (rateAdpt + (scaleId == 0 ? prmNum * rateScale : 0));
      }
      else
      {
        cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (scaleId == 0 ? rateScale : 0);
      }
    }
#else
      cost = (double) pDist[mode] + m_lambda[COMPONENT_Y] * ( scaleId == 0 ? rateScale : 0);
#endif
#endif
    else
#if JVET_AF0085_RESIDUAL_ADJ
    {
      int i = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
      for (int mode = 0; mode < prmNum; mode++)
      {
        rateOffset += (picprms.sprm.offset[COMPONENT_Y][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cb][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[COMPONENT_Cr][mode][i] == 0) ? 1 : 2;
      }
      cost = (double)pDist[mode] + m_lambda[COMPONENT_Y] * (rateAdpt + (scaleId == 0 ? (prmNum * rateScale + rateOffset) : (rateOffset)));
    }
#else
      cost = (double) pDist[mode] + m_lambda[COMPONENT_Y] * ( rateAdpt + (scaleId == 0 ? prmNum * rateScale : 0));
#endif

    if (cost < minCost)
    {
      minCost = cost;
      picprms.sprm.mode = mode;
      picprms.sprm.scaleFlag = scaleId;
      bestPrmId = picprms.prmId;
#if JVET_AK0093_NON_NORMATIVE_TDO
      if (useTDO && mode != -1 && bsPartitionOn[picprms.sprm.mode])
      {
        bestPrmId = bsFilteringBlocks[picprms.sprm.mode];
        picprms.sprm.mode = prmNum;
      }
#endif
    }
  }
#if JVET_AF0193_DECODER_OPTIMIZATION
  if (getNnlfDecOpt() && picprms.sprm.mode == 2 && (scaleId == -1 || scaleId == 2)) // This optimzation is used only for block-level mode 
  {
    std::vector<pair<int, double>> vtEuclideanDist;
    double                         ratio = 0.01;
    int                            offNum = 0;
    for (auto it = euclideanDist.begin(); it != euclideanDist.end(); it++) 
    {
      vtEuclideanDist.push_back(make_pair(it->first, it->second));
    }
    sort(vtEuclideanDist.begin(), vtEuclideanDist.end(), [](const pair<int, double> &x, const pair<int, double> &y) -> int { return x.second < y.second; });

    for (auto it = vtEuclideanDist.begin(); it != vtEuclideanDist.end(); it++)
    {
      if ((double)(it->second / ssdRec[it->first]) > ratio)
      {
        break;
      }
      offNum++;
    }

    for (int i = 0; i < offNum; i++) 
    {
      bestPrmId[vtEuclideanDist[i].first] = -1;
    }
  }
#endif


#endif

}

void EncNNFilterUnified::scaleFactorDerivation(Picture &pic, FilterParameters &prms, int prmId
#if JVET_AF0085_RESIDUAL_ADJ
  , int scaleId
#endif
)
{
  CodingStructure &cs      = *pic.cs;
  Slice *          pcSlice = cs.slice;

  const CPelUnitBuf recoBuf = pic.getRecoBuf();
  const CPelUnitBuf origBuf = pic.getOrigBuf();
  const CPelUnitBuf filteredBuf = getScaledBuf(0, prmId);
#if JVET_AF0085_RESIDUAL_ADJ
  const CPelUnitBuf recoBeforeDbfBuf = pic.getRecBeforeDbfBuf();
  const CPelUnitBuf filteredNoScaledBuf = getFilteredBuf(prmId);
#endif

  const int inputBitDepth    = pcSlice->clpRng(COMPONENT_Y).bd;   // internal bitdepth
  const int shift            = log2OutputScale - inputBitDepth;
  const float stablizingFactor = (0.1f * (1 << shift));

  int area = recoBuf.get(COMPONENT_Y).width * recoBuf.get(COMPONENT_Y).height;
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++) 
  {
    ComponentID compID = ComponentID(compIdx);
    const CPelBuf orgBuf = origBuf.get(compID);
    const CPelBuf recBuf = recoBuf.get(compID);
    const CPelBuf cnnBuf = filteredBuf.get(compID);

    int height = recBuf.height;
    int width  = recBuf.width;

#if JVET_AF0085_RESIDUAL_ADJ
    const CPelBuf recBeforeDbfBuf = recoBeforeDbfBuf.get(compID);
    const CPelBuf cnnNoScaledBuf = filteredNoScaledBuf.get(compID);

    if (scaleId == -1)
    {
      uint64_t costMin = MAX_UINT64;
      int bestRoaOffset = 0;

      for (int roaOffset = 0; roaOffset <= 2; roaOffset++)
      {
        uint64_t ssd = 0LL;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            // positive-, negative+
            int curP = 0;
            if( ( cnnBuf.at( x, y ) - ( recBeforeDbfBuf.at( x, y ) << shift ) ) >= ( roaOffset << shift ) )
            {
              curP = cnnNoScaledBuf.at( x, y ) - roaOffset;
            }
            else if( ( cnnBuf.at( x, y ) - ( recBeforeDbfBuf.at( x, y ) << shift ) ) <= ( -roaOffset << shift ) )
            {
              curP = cnnNoScaledBuf.at( x, y ) + roaOffset;
            }
            else
            {
              curP = cnnNoScaledBuf.at( x, y );
            }

            Pel clipP = Pel(Clip3<int>(0, (1 << inputBitDepth) - 1, curP));
            ssd += (orgBuf.at(x, y) - clipP) * (orgBuf.at(x, y) - clipP);
          }
        }

        if (ssd < costMin)
        {
          costMin = ssd; bestRoaOffset = roaOffset;
        }
      }
      prms.sprm.offset[compID][prmId][3] = bestRoaOffset;

      continue;
    }
#endif

    double selfMulti  = 0.;
    double crossMulti = 0.;
    double sumOrgResi = 0.;
    double sumCnnResi = 0.;

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        int orgResi = (orgBuf.at(x, y) - recBuf.at(x, y)) << shift;
        int cnnResi = cnnBuf.at(x, y) - (recBuf.at(x, y) << shift);
        selfMulti += cnnResi * cnnResi;
        crossMulti += cnnResi * orgResi;
        sumOrgResi += orgResi;
        sumCnnResi += cnnResi;
      }
    }

    int Up  = int(nnResidueScaleDerivationUpBound * (1 << log2ResidueScale));
    int Bot = int(nnResidueScaleDerivationLowBound * (1 << log2ResidueScale));

    int areaComp = compID == 0 ? area : (area / 4);

    int scale = int(((areaComp * crossMulti - sumOrgResi * sumCnnResi + areaComp * areaComp * stablizingFactor)
           / (areaComp * selfMulti - sumCnnResi * sumCnnResi + areaComp * areaComp * stablizingFactor))
           * (1 << log2ResidueScale) + 0.5);

    scale = Clip3(Bot, Up, scale);
    prms.sprm.scale[compID][prmId] = scale;

#if JVET_AF0085_RESIDUAL_ADJ
    const int shift2 = shift + log2ResidueScale;
    const int offset = (1 << shift2) / 2;

    for (int i = 0; i < 3; i++)
    {
      int scale = i == 0 ? prms.sprm.scale[compID][prmId] : scaleCandidates[i];
      uint64_t costMin = MAX_UINT64;
      int bestRoaOffset = 0;

      for (int roaOffset = 0; roaOffset <= 2; roaOffset++)
      {
        uint64_t ssd = 0LL;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            // positive-, negative+
            int curP = 0;
            if( ( cnnBuf.at( x, y ) - ( recBeforeDbfBuf.at( x, y ) << shift ) ) >= ( roaOffset << shift ) )
            {
              curP = ( ( ( int ) recBuf.at( x, y ) << shift2 ) + ( cnnBuf.at( x, y ) - ( recBuf.at( x, y ) << shift ) - ( roaOffset << shift ) ) * scale + offset ) >> shift2;
            }
            else if( ( cnnBuf.at( x, y ) - ( recBeforeDbfBuf.at( x, y ) << shift ) ) <= ( -roaOffset << shift ) )
            {
              curP = ( ( ( int ) recBuf.at( x, y ) << shift2 ) + ( cnnBuf.at( x, y ) - ( recBuf.at( x, y ) << shift ) + ( roaOffset << shift ) ) * scale + offset ) >> shift2;
            }
            else
            {
              curP = ( ( ( int ) recBuf.at( x, y ) << shift2 ) + ( cnnBuf.at( x, y ) - ( recBuf.at( x, y ) << shift ) ) * scale + offset ) >> shift2;
            }

            Pel clipP = Pel(Clip3<int>(0, (1 << inputBitDepth) - 1, curP));
            ssd += (orgBuf.at(x, y) - clipP) * (orgBuf.at(x, y) - clipP);
          }
        }

        if (ssd < costMin)
        {
          costMin = ssd; bestRoaOffset = roaOffset;
        }
      }
      prms.sprm.offset[compID][prmId][i] = bestRoaOffset;
    }
#endif
  }
}

void EncNNFilterUnified::scalePicture(Picture &pic, int scaleId)
{
#if JVET_AF0085_RESIDUAL_ADJ
  if (scaleId > 2)
#else
  if (scaleId < 0 || scaleId > 2)
#endif
    return;

  CodingStructure&  cs            = *pic.cs;
  const PreCalcValues& pcv        = *cs.pcv;
  FilterParameters &prms = getPicprms();
  const int  numValidComponents = getNumberValidComponents( cs.area.chromaFormat );

  for (int prmId = 0; prmId < prms.prmNum; prmId++)
  {
#if JVET_AF0085_RESIDUAL_ADJ
    if (scaleId == 0 || scaleId == -1)
    {
      scaleFactorDerivation(pic, prms, prmId, scaleId);
    }
#else
    if (scaleId == 0)
    {
      scaleFactorDerivation(pic, prms, prmId);
    }
#endif
    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const ComponentID c = ComponentID(comp);
      
      for (int y = 0; y < prms.numBlocksHeight; ++y)
      {
        for (int x = 0; x < prms.numBlocksWidth; ++x)
        {
          int xPos   = x * prms.blockSize;
          int yPos   = y * prms.blockSize;
          int width  = (xPos + prms.blockSize > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : prms.blockSize;
          int height = (yPos + prms.blockSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : prms.blockSize;
          const UnitArea inferAreaNoExt(cs.area.chromaFormat, Area(xPos, yPos, width, height));
          
#if JVET_AF0085_RESIDUAL_ADJ
          if (scaleId == -1)
          {
            PelUnitBuf scaledBuf = getScaledBuf(0, prmId, inferAreaNoExt);
            PelUnitBuf noScaledBuf = getFilteredBuf(prmId, inferAreaNoExt);
            scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), noScaledBuf.get(c), 0, prms.sprm.offset[c][prmId][3]);
          }
          else
          {
            PelUnitBuf scaledBuf = getScaledBuf(scaleId, prmId, inferAreaNoExt);
            scaleId == 0
              ? scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c), prms.sprm.scale[c][prmId], prms.sprm.offset[c][prmId][scaleId])
              : scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c), scaleCandidates[scaleId], prms.sprm.offset[c][prmId][scaleId]);
          }
 #else
          PelUnitBuf scaledBuf = getScaledBuf(scaleId, prmId, inferAreaNoExt);
          scaleId == 0
                ? scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c), prms.sprm.scale[c][prmId])
                : scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c), scaleCandidates[scaleId]);
#endif
        }
      }
    }
  }


}

#endif
