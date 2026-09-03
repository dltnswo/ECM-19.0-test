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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include <vector>

//! \ingroup DecoderLib
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

DecSlice::DecSlice()
{
}

DecSlice::~DecSlice()
{
}

#if JVET_AG0117_CABAC_SPATIAL_TUNING
void DecSlice::create(int width, int iMaxCUWidth)
{
  int numBinBuffers = width / iMaxCUWidth + 1;
  
  for ( int i = 0; i < numBinBuffers; i++ )
  {
    m_binVectors.push_back( BinStoreVector() );
    m_binVectors[i].reserve( CABAC_SPATIAL_MAX_BINS );
  }
}
#else
void DecSlice::create()
{
}
#endif

void DecSlice::destroy()
{
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  m_binVectors.clear();
#endif
}

void DecSlice::init( CABACDecoder* cabacDecoder, DecCu* pcCuDecoder )
{
  m_CABACDecoder    = cabacDecoder;
  m_pcCuDecoder     = pcCuDecoder;
}

void DecSlice::decompressSlice( Slice* slice, InputBitstream* bitstream, int debugCTU )
{
  //-- For time output for each slice
  slice->startProcessingTimer();

  const SPS*     sps          = slice->getSPS();
  Picture*       pic          = slice->getPic();
  CABACReader&   cabacReader  = *m_CABACDecoder->getCABACReader( 0 );
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  cabacReader.m_CABACDataStore->updateBufferState( slice );
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  clearAmvpSbTmvpStatArea(slice);
#endif

  if ( slice->getSliceID() == 0 )
  {
    pic->getRecoBuf().bufs[COMPONENT_Y].fill( 0, MAX_ALF_PADDING_SIZE );
    pic->getRecoBuf().bufs[COMPONENT_Cb].fill( 0, MAX_ALF_PADDING_SIZE );
    pic->getRecoBuf().bufs[COMPONENT_Cr].fill( 0, MAX_ALF_PADDING_SIZE );
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( slice->isIntra() )
  {
    slice->setSeparateTreeEnabled( slice->getSPS()->getUseDualITree() );
  }
  else
  {
    slice->setSeparateTreeEnabled( slice->getSPS()->getInterSliceSeparateTreeEnabled() );

    if ( !slice->isIntra())
    {
      int picWidth = sps->getMaxPicWidthInLumaSamples();
      int picHeight = sps->getMaxPicHeightInLumaSamples();
      if ( (picWidth * picHeight) <= (1280*720) && ( slice->getTLayer()>0 ) )
      {
        slice->setSeparateTreeEnabled(false);
      }
      else if (slice->getTLayer()>=ID_SEP_TREE_TID_OFF)
      {
        slice->setSeparateTreeEnabled(false);
      }
    }
  }
#endif


  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.slice            = slice;
  cs.sps              = sps;
  cs.pps              = slice->getPPS();
  memcpy(cs.alfApss, slice->getAlfAPSs(), sizeof(cs.alfApss));

#if JVET_AK0065_TALF
  memcpy(cs.talfApss, slice->getTAlfAPSs(), sizeof(cs.talfApss));
#endif
  cs.lmcsAps          = slice->getPicHeader()->getLmcsAPS();
  cs.scalinglistAps   = slice->getPicHeader()->getScalingListAPS();

  cs.pcv              = slice->getPPS()->pcv;
  cs.chromaQpAdj      = 0;

  cs.picture->resizeSAO(cs.pcv->sizeInCtus, 0);
  cs.resetPrevPLT(cs.prevPLT);

  if (slice->getFirstCtuRsAddrInSlice() == 0)
  {
    cs.picture->resizeAlfCtuEnableFlag( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtbFilterIndex(cs.pcv->sizeInCtus);
    cs.picture->resizeAlfCtuAlternative( cs.pcv->sizeInCtus );
  }

#if JVET_AL0153_ALF_CCCM
  slice->lfCccmClearControlInformation();
#endif
  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;

  // init each couple {EntropyDecoder, Substream}
  // Table of extracted substreams.
  std::vector<InputBitstream*> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx] = bitstream->extractSubstream( idx+1 < numSubstreams ? ( slice->getSubstreamSize(idx) << 3 ) : bitstream->getNumBitsLeft() );
  }

  const unsigned  widthInCtus             = cs.pcv->widthInCtus;
  const bool     wavefrontsEnabled           = cs.sps->getEntropyCodingSyncEnabledFlag();
  const bool     entryPointPresent           = cs.sps->getEntryPointsPresentFlag();

  cabacReader.initBitstream( ppcSubstreams[0] );
  cabacReader.initCtxModels( *slice );

  // Quantization parameter
    pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
  CHECK( pic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->getPOC() );


#if JVET_S0258_SUBPIC_CONSTRAINTS
  if( slice->getSliceType() != I_SLICE && slice->getRefPic( REF_PIC_LIST_0, 0 )->subPictures.size() > 1 )
#else
  if (slice->getSliceType() != I_SLICE && slice->getRefPic(REF_PIC_LIST_0, 0)->numSubpics > 1)
#endif
  {
    clipMv = clipMvInSubpic;
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    clipMv2 = clipMvInSubpic2;
#endif
  }
  else
  {
    clipMv = clipMvInPic;
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
    clipMv2 = clipMvInPic2;
#endif
  }

#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
  if (slice->getPicHeader()->getEnableTMVPFlag())
  {
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    for (int colFrameIdx = 0; colFrameIdx < (slice->isInterB() ? 2 : 1); colFrameIdx++)
    {
      const Picture* const pColPic = slice->getRefPic(RefPicList(colFrameIdx == 0 ? 1 - slice->getColFromL0Flag() : 1 - slice->getColFromL0Flag2nd()), colFrameIdx == 0 ? slice->getColRefIdx() : slice->getColRefIdx2nd());
#else
    const Picture* const pColPic = slice->getRefPic(RefPicList(slice->isInterB() ? 1 - slice->getColFromL0Flag() : 0), slice->getColRefIdx());
#endif 
    if (pColPic)
    {
      const int currPOC = slice->getPOC();
      const int colPOC = pColPic->getPOC();

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      slice->resizeImBuf(pColPic->numSlices, colFrameIdx);
#else
      slice->resizeImBuf(pColPic->numSlices);
#endif
      Slice *pColSlice = nullptr;
      for (int sliceIdx = 0; sliceIdx < pColPic->numSlices; sliceIdx++)
      {
        pColSlice = pColPic->slices[sliceIdx];
        if (pColSlice->isIntra())
        {
          continue;
        }

        for (int colRefPicListIdx = 0; colRefPicListIdx < (pColSlice->isInterB() ? 2 : 1); colRefPicListIdx++)
        {
          for (int colRefIdx = 0; colRefIdx < pColSlice->getNumRefIdx(RefPicList(colRefPicListIdx)); colRefIdx++)
          {
#if JVET_AI0183_MVP_EXTENSION
            const bool bIsColRefLongTerm = pColSlice->getIsUsedAsLongTerm(RefPicList(colRefPicListIdx), colRefIdx);
            const int colRefPOC = pColSlice->getRefPOC(RefPicList(colRefPicListIdx), colRefIdx);

            for (int curRefPicListIdx = 0; curRefPicListIdx < (slice->isInterB() ? 2 : 1); curRefPicListIdx++)
            {
              double bestDistScale = MAX_DOUBLE;
              int targetRefIdx = -1;
              for (int curRefIdx = 0; curRefIdx < slice->getNumRefIdx(RefPicList(curRefPicListIdx)); curRefIdx++)
              {
                const int currRefPOC = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->getPOC();
                const bool bIsCurrRefLongTerm = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->longTerm;
                if (bIsCurrRefLongTerm != bIsColRefLongTerm)
                {
                  continue;
                }
                if (bIsCurrRefLongTerm)
                {
                  targetRefIdx = curRefIdx;
                  bestDistScale = 1;
                  break;
                }
                else if (colPOC - colRefPOC == currPOC - currRefPOC)
                {
                  targetRefIdx = curRefIdx;
                  bestDistScale = 1;
                  break;
                }
                else
                {
                  if (abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0)) < bestDistScale)
                  {
                    bestDistScale = abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0));
                    targetRefIdx = curRefIdx;
                  }
                }
              }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx, targetRefIdx, colFrameIdx);
#else
              slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx, targetRefIdx);
#endif
              if (slice->getCheckLDC() == true)
              {
                continue;
              }
              int targetRefIdx1st = targetRefIdx;
              double bestOverScale = 0;
              double scale         = 0;
              int    curPOCMax, curPOCMin;
              int    colPOCMax, colPOCMin;
              int    bestTargetRefIdx = -1;
              bestDistScale = MAX_DOUBLE;
              targetRefIdx  = -1;
              for (int curRefIdx = 0; curRefIdx < slice->getNumRefIdx(RefPicList(curRefPicListIdx)); curRefIdx++)
              {
                if (curRefIdx == targetRefIdx1st)
                {
                  continue;
                }
                const int  currRefPOC         = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->getPOC();
                const bool bIsCurrRefLongTerm = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->longTerm;
                if (bIsCurrRefLongTerm != bIsColRefLongTerm)
                {
                  continue;
                }
                if (bIsCurrRefLongTerm)
                {
                  targetRefIdx  = curRefIdx;
                  bestDistScale = 1;
                  bestTargetRefIdx = -1;
                  break;
                }
                else if (colRefPOC == currRefPOC)
                {
                  targetRefIdx  = curRefIdx;
                  bestDistScale = 1;
                  bestTargetRefIdx = -1;
                  break;
                }
                else
                {
                  curPOCMax = std::max(currPOC, currRefPOC);
                  curPOCMin = std::min(currPOC, currRefPOC);
                  colPOCMax = std::max(colPOC, colRefPOC);
                  colPOCMin = std::min(colPOC, colRefPOC);
                  scale = std::max(0, std::min(curPOCMax, colPOCMax) - std::max(curPOCMin, colPOCMin));
                  scale = scale * scale / (abs(currPOC - currRefPOC) * abs(colPOC - colRefPOC));
                  if (scale > bestOverScale)
                  {
                    bestOverScale    = scale;
                    bestTargetRefIdx = curRefIdx;
                  }
                  if (abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0)) < bestDistScale)
                  {
                    bestDistScale = abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0));
                    targetRefIdx  = curRefIdx;
                  }
                }
              }   // curRefIdx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              if (bestTargetRefIdx != -1)
              {
                targetRefIdx = bestTargetRefIdx;
              }
              if (targetRefIdx == -1)
              {
                targetRefIdx = 0;
              }
              slice->setImRefIdx2nd(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx,
                                 targetRefIdx, colFrameIdx  );
#else
              slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx,
                                 targetRefIdx);
#endif
                bestOverScale = 0;
                scale         = 0;
                bestTargetRefIdx = -1;
                bestDistScale = MAX_DOUBLE;
                targetRefIdx  = -1;
                for (int curRefIdx = 0; curRefIdx < slice->getNumRefIdx(RefPicList(curRefPicListIdx)); curRefIdx++)
                {
                  const int  currRefPOC         = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->getPOC();
                  const bool bIsCurrRefLongTerm = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->longTerm;
                  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
                  {
                    continue;
                  }
                  if (bIsCurrRefLongTerm)
                  {
                    targetRefIdx  = curRefIdx;
                    bestDistScale = 1;
                    bestTargetRefIdx = -1;
                    break;
                  }
                  else if (colRefPOC == currRefPOC)
                  {
                    targetRefIdx  = curRefIdx;
                    bestDistScale = 1;
                    bestTargetRefIdx = -1;
                    break;
                  }
                  else
                  {
                    curPOCMax = std::max(currPOC, currRefPOC);
                    curPOCMin = std::min(currPOC, currRefPOC);
                    colPOCMax = std::max(colPOC, colRefPOC);
                    colPOCMin = std::min(colPOC, colRefPOC);
                    scale = std::max(0, std::min(curPOCMax, colPOCMax) - std::max(curPOCMin, colPOCMin));
                    scale = scale * scale / (abs(currPOC - currRefPOC) * abs(colPOC - colRefPOC));
                    if (scale > bestOverScale)
                    {
                      bestOverScale    = scale;
                      bestTargetRefIdx = curRefIdx;
                    }
                    if (abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0)) < bestDistScale)
                    {
                      bestDistScale = abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0));
                      targetRefIdx  = curRefIdx;
                    }
                  }
                }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                if (bestTargetRefIdx != -1)
                {
                  targetRefIdx = bestTargetRefIdx;
                }
                if (targetRefIdx == -1)
                {
                  targetRefIdx = 0;
                }
                slice->setImRefIdx3rd(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx,
                                      targetRefIdx, colFrameIdx);
#else
                slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx,
                                   targetRefIdx);
#endif
            }
#else
            const bool bIsColRefLongTerm = pColSlice->getIsUsedAsLongTerm(RefPicList(colRefPicListIdx), colRefIdx);
            const int colRefPOC = pColSlice->getRefPOC(RefPicList(colRefPicListIdx), colRefIdx);

            for (int curRefPicListIdx = 0; curRefPicListIdx < (slice->isInterB() ? 2 : 1); curRefPicListIdx++)
            {
              double bestDistScale = MAX_DOUBLE;
              int targetRefIdx = -1;
              for (int curRefIdx = 0; curRefIdx < slice->getNumRefIdx(RefPicList(curRefPicListIdx)); curRefIdx++)
              {
                const int currRefPOC = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->getPOC();
                const bool bIsCurrRefLongTerm = slice->getRefPic(RefPicList(curRefPicListIdx), curRefIdx)->longTerm;
                if (bIsCurrRefLongTerm != bIsColRefLongTerm)
                {
                  continue;
                }
                if (bIsCurrRefLongTerm)
                {
                  targetRefIdx = curRefIdx;
                  bestDistScale = 1;
                  break;
                }
                else if (colPOC - colRefPOC == currPOC - currRefPOC)
                {
                  targetRefIdx = curRefIdx;
                  bestDistScale = 1;
                  break;
                }
                else
                {
                  if (abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0)) < bestDistScale)
                  {
                    bestDistScale = abs(1.0 - (abs(currPOC - currRefPOC) * 1.0 / abs(colPOC - colRefPOC) * 1.0));
                    targetRefIdx = curRefIdx;
                  }
                }
              } // curRefIdx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx, targetRefIdx, colFrameIdx);
#else
              slice->setImRefIdx(sliceIdx, RefPicList(colRefPicListIdx), RefPicList(curRefPicListIdx), colRefIdx, targetRefIdx);
#endif
            } // curRefPicListIdx
#endif
          }
        }
      }
    }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (!slice->isIntra())
  {
    int minPoc = abs(slice->getRefPic(RefPicList(1 - slice->getColFromL0Flag()), slice->getColRefIdx())->getPOC() - slice->getPOC());
    if (slice->isInterB() && slice->getCheckLDC())
    {
      int min2ndPoc = abs(slice->getRefPic(RefPicList(1 - slice->getColFromL0Flag2nd()), slice->getColRefIdx2nd())->getPOC() - slice->getPOC());
      minPoc = std::min(minPoc, min2ndPoc);
    }
    if (minPoc > 4)
    {
      slice->setAmvpSbTmvpEnabledFlag(false);
      slice->setAmvpSbTmvpAmvrEnabledFlag(false);
    }
    else
    {
      slice->setAmvpSbTmvpEnabledFlag(true);
      g_picAmvpSbTmvpEnabledArea = 0;
      uint32_t prevEnabledArea;
      bool isExist = loadAmvpSbTmvpStatArea(slice->getTLayer(), prevEnabledArea);
      if (isExist)
      {
        int ratio = int(prevEnabledArea * 100.0 / (slice->getPic()->getPicWidthInLumaSamples() * slice->getPic()->getPicHeightInLumaSamples()));
        if (ratio < 4)
        {
          slice->setAmvpSbTmvpNumOffset(1);
        }
        else if (ratio < 7)
        {
          slice->setAmvpSbTmvpNumOffset(2);
        }
        else
        {
          slice->setAmvpSbTmvpNumOffset(3);
        }
      }
      else
      {
        slice->setAmvpSbTmvpNumOffset(2);
      }
      if (slice->isInterB() && slice->getCheckLDC())
      {
        if (slice->getRefPic(RefPicList(1 - slice->getColFromL0Flag()), slice->getColRefIdx())->getPOC() == slice->getRefPic(RefPicList(1 - slice->getColFromL0Flag2nd()), slice->getColRefIdx2nd())->getPOC())
        {
          slice->setAmvpSbTmvpNumColPic(1);
        }
        else
        {
          slice->setAmvpSbTmvpNumColPic(2);
        }
      }
      else
      {
        slice->setAmvpSbTmvpNumColPic(1);
      }
      slice->setAmvpSbTmvpAmvrEnabledFlag(slice->getPic()->getPicWidthInLumaSamples() * slice->getPic()->getPicHeightInLumaSamples() < 3840*2160 ? false : true);
    }
  }
  else
  {
    slice->setAmvpSbTmvpEnabledFlag(false);
  }
#endif
  }
#if JVET_AG0098_AMVP_WITH_SBTMVP
  else
  {
    slice->setAmvpSbTmvpEnabledFlag(false);
  }
#endif
#endif

#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  if (!slice->isIntra())
  {
    int picH = slice->getPic()->getPicHeightInLumaSamples();
    slice->setExtAmvpLevel(picH >= 2160 ? 3 : (picH >= 1080 ? 2 : (picH >= 720 ? 1 : 0)));
  }
#endif

  // for every CTU in the slice segment...

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  static Ctx storedCtx;
#endif
#if JVET_AD0188_CCP_MERGE
  {
#if JVET_Z0118_GDR
    cs.ccpLut.lutCCP0.resize(0);
    cs.ccpLut.lutCCP1.resize(0);
#else
    cs.ccpLut.lutCCP.resize(0);
#endif
  }
#endif
#if JVET_AG0058_EIP
  {
#if JVET_Z0118_GDR
    cs.eipLut.lutEip0.resize(0);
    cs.eipLut.lutEip1.resize(0);
#else
    cs.eipLut.lutEip.resize(0);
#endif
  }
#endif
  unsigned subStrmId = 0;
  for( unsigned ctuIdx = 0; ctuIdx < slice->getNumCtuInSlice(); ctuIdx++ )
  {
    const unsigned  ctuRsAddr       = slice->getCtuAddrInSlice(ctuIdx);
    const unsigned  ctuXPosInCtus   = ctuRsAddr % widthInCtus;
    const unsigned  ctuYPosInCtus   = ctuRsAddr / widthInCtus;
    const unsigned  tileColIdx      = slice->getPPS()->ctuToTileCol( ctuXPosInCtus );
    const unsigned  tileRowIdx      = slice->getPPS()->ctuToTileRow( ctuYPosInCtus );
    const unsigned  tileXPosInCtus  = slice->getPPS()->getTileColumnBd( tileColIdx );
    const unsigned  tileYPosInCtus  = slice->getPPS()->getTileRowBd( tileRowIdx );
    const unsigned  tileColWidth    = slice->getPPS()->getTileColumnWidth( tileColIdx );
    const unsigned  tileRowHeight   = slice->getPPS()->getTileRowHeight( tileRowIdx );
    const unsigned  tileIdx         = slice->getPPS()->getTileIdx( ctuXPosInCtus, ctuYPosInCtus);
    const unsigned  maxCUSize             = sps->getMaxCUWidth();
    Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );
    const SubPic &curSubPic = slice->getPPS()->getSubPicFromPos(pos);
#if JVET_AL0153_ALF_CCCM
    slice->lfCccmClearControlInformation(ctuRsAddr);
#endif
    // padding/restore at slice level
    if (slice->getPPS()->getNumSubPics()>=2 && curSubPic.getTreatedAsPicFlag() && ctuIdx==0)
    {
      int subPicX      = (int)curSubPic.getSubPicLeft();
      int subPicY      = (int)curSubPic.getSubPicTop();
      int subPicWidth  = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();
      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
      {
        int n = slice->getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);

#if JVET_S0258_SUBPIC_CONSTRAINTS
          if( !refPic->getSubPicSaved() && refPic->subPictures.size() > 1 )
#else
          if (!refPic->getSubPicSaved() && refPic->numSubpics > 1)
#endif
          {
            refPic->saveSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->extendSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->setSubPicSaved(true);
          }
        }
      }
    }

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId] );

    // set up CABAC contexts' state for this CTU
    if( ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
    {
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
        cs.resetPrevPLT(cs.prevPLT);
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }
    else if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      // Synchronize cabac probabilities with top CTU if it's available and at the start of a line.
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
        cs.resetPrevPLT(cs.prevPLT);
      }
      if( cs.getCURestricted( pos.offset(0, -1), pos, slice->getIndependentSliceIdx(), tileIdx, CH_L ) )
      {
        // Top is available, so use it.
        cabacReader.getCtx() = m_entropyCodingSyncContextState;
        cs.setPrevPLT(m_palettePredictorSyncState);
      }
      pic->m_prevQP[0] = pic->m_prevQP[1] = slice->getSliceQp();
    }

    bool updateBcwCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuIdx == 0;
    if(updateBcwCodingOrder)
    {
      resetBcwCodingOrder(true, cs);
    }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if ((cs.slice->getSliceType() != I_SLICE || cs.slice->getUseIBC()) && ctuXPosInCtus == tileXPosInCtus)
#else
    if ((cs.slice->getSliceType() != I_SLICE || cs.sps->getIBCFlag()) && ctuXPosInCtus == tileXPosInCtus)
#endif
    {
#if JVET_Z0118_GDR
      cs.motionLut.lut0.resize(0);      
      cs.motionLut.lutIbc0.resize(0);

      if (cs.isGdrEnabled())
      {
        cs.motionLut.lut1.resize(0);
        cs.motionLut.lutIbc1.resize(0);
      }
#else
      cs.motionLut.lut.resize(0);
      cs.motionLut.lutIbc.resize(0);
#endif

#if JVET_Z0139_HIST_AFF   
      for (int i = 0; i < 2 * MAX_NUM_AFFHMVP_ENTRIES_ONELIST; i++)
      {
#if JVET_Z0118_GDR
        cs.motionLut.lutAff0[i].resize(0);
        if (cs.isGdrEnabled())
        {
          cs.motionLut.lutAff1[i].resize(0);
        }
#else
        cs.motionLut.lutAff[i].resize(0);
#endif
      }
#if JVET_Z0118_GDR
      cs.motionLut.lutAffInherit0.resize(0);
      if (cs.isGdrEnabled())
      {
        cs.motionLut.lutAffInherit1.resize(0);
      }
#else
      cs.motionLut.lutAffInherit.resize(0);
#endif
#endif
#if !JVET_Z0153_IBC_EXT_REF
      cs.resetIBCBuffer = true;
#endif
    }
#if JVET_AD0188_CCP_MERGE
    if (ctuXPosInCtus == tileXPosInCtus)
    {
#if JVET_Z0118_GDR
      cs.ccpLut.lutCCP0.resize(0);
      cs.ccpLut.lutCCP1.resize(0);
#else
      cs.ccpLut.lutCCP.resize(0);
#endif
    }
#endif
#if JVET_AG0058_EIP
    if (ctuXPosInCtus == tileXPosInCtus)
    {
#if JVET_Z0118_GDR
      cs.eipLut.lutEip0.resize(0);
      cs.eipLut.lutEip1.resize(0);
#else
      cs.eipLut.lutEip.resize(0);
#endif
    }
#endif
    if( !cs.slice->isIntra() )
    {
      pic->mctsInfo.init( &cs, getCtuAddr( ctuArea.lumaPos(), *( cs.pcv ) ) );
    }

    if( ctuRsAddr == debugCTU )
    {
      break;
    }
#if JVET_V0094_BILATERAL_FILTER
    if (ctuRsAddr == 0)
    {
      if (cs.pps->getUseBIF())
      {
        cabacReader.bif(COMPONENT_Y, cs);
      }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if (cs.pps->getUseChromaBIF())
      {
        cabacReader.bif(COMPONENT_Cb, cs);
        cabacReader.bif(COMPONENT_Cr, cs);
      }
#endif
    }
#endif

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Update the CABAC states based on CTU above
    if ( ctuYPosInCtus )
    {
      cabacReader.updateCtxs( getBinVector(ctuXPosInCtus) );
    }

    // Clear the bin counters and prepare for collecting new data for this CTU
    cabacReader.setBinBuffer( getBinVector(ctuXPosInCtus) );
#endif

    cabacReader.coding_tree_unit( cs, ctuArea, pic->m_prevQP, ctuRsAddr );

#if JVET_AG0117_CABAC_SPATIAL_TUNING
    // Done with data collection for this CTU
    cabacReader.setBinBuffer( nullptr );
#endif

    m_pcCuDecoder->decompressCtu( cs, ctuArea );

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
    // store CABAC context to be used in next frames
    if( storeContexts( slice, ctuXPosInCtus, ctuYPosInCtus ) )
    {
      storedCtx = cabacReader.getCtx();
    }
#endif

    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = cabacReader.getCtx();
      cs.storePrevPLT(m_palettePredictorSyncState);
    }


    if( ctuIdx == slice->getNumCtuInSlice()-1 )
    {
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes( false );
#endif
    }
    else if( ( ctuXPosInCtus + 1 == tileXPosInCtus + tileColWidth ) &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + tileRowHeight || wavefrontsEnabled ) )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
      if( entryPointPresent )
      {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
        cabacReader.remaining_bytes( true );
#endif
        subStrmId++;
      }
    }
    if (slice->getPPS()->getNumSubPics() >= 2 && curSubPic.getTreatedAsPicFlag() && ctuIdx == (slice->getNumCtuInSlice() - 1))
    // for last Ctu in the slice
    {
      int subPicX = (int)curSubPic.getSubPicLeft();
      int subPicY = (int)curSubPic.getSubPicTop();
      int subPicWidth = (int)curSubPic.getSubPicWidthInLumaSample();
      int subPicHeight = (int)curSubPic.getSubPicHeightInLumaSample();
      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
      {
        int n = slice->getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);
          if (refPic->getSubPicSaved())
          {
            refPic->restoreSubPicBorder(refPic->getPOC(), subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->setSubPicSaved(false);
          }
        }
      }
    }
  }

#if JVET_AC0089_NNVC_USE_BPM_INFO
  pic->dumpPicBpmInfo();
#if JVET_AJ0124_QP_BLOCK
  pic->dumpQpBlock();
#endif
#endif

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  // store CABAC context to be used in next frames when the last CTU in a picture is processed
  if( slice->getPPS()->pcv->sizeInCtus - 1 == slice->getCtuAddrInSlice( slice->getNumCtuInSlice() - 1 ) )
  {
    cabacReader.m_CABACDataStore->storeCtxStates( slice, storedCtx );
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (!slice->isIntra())
  {
    storeAmvpSbTmvpStatArea(slice->getTLayer(), g_picAmvpSbTmvpEnabledArea);
  }
#endif

  // deallocate all created substreams, including internal buffers.
  for( auto substr: ppcSubstreams )
  {
    delete substr;
  }
  slice->stopProcessingTimer();
}

//! \}
