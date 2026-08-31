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

/** \file     SampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "SampleAdaptiveOffset.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup CommonLib
//! \{

SAOOffset::SAOOffset()
{
  reset();
}

SAOOffset::~SAOOffset()
{

}

void SAOOffset::reset()
{
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(int)* MAX_NUM_SAO_CLASSES);

  return *this;
}


SAOBlkParam::SAOBlkParam()
{
  reset();
}

SAOBlkParam::~SAOBlkParam()
{

}

void SAOBlkParam::reset()
{
  for(int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)
{
  for(int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;
}




SampleAdaptiveOffset::SampleAdaptiveOffset()
{
  m_numberOfComponents = 0;
#if JVET_W0066_CCSAO
  m_ccSaoControl[0] = m_ccSaoControl[1] = m_ccSaoControl[2] = nullptr;
#endif
}


SampleAdaptiveOffset::~SampleAdaptiveOffset()
{
  destroy();

  m_signLineBuf1.clear();
  m_signLineBuf2.clear();
}

void SampleAdaptiveOffset::create( int picWidth, int picHeight, ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight, uint32_t maxCUDepth, uint32_t lumaBitShift, uint32_t chromaBitShift )
{
  //temporary picture buffer
  UnitArea picArea(format, Area(0, 0, picWidth, picHeight));

  m_tempBuf.destroy();
  m_tempBuf.create( picArea );

  //bit-depth related
  for(int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
  m_numberOfComponents = getNumberValidComponents(format);

#if JVET_W0066_CCSAO
#if !RPR_ENABLE
  if( m_created )
  {
    return;
  }
#endif
  m_created = true;

  m_ccSaoBuf.destroy();
  m_ccSaoBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_CCSAO_FILTER_LENGTH >> 1, 0, false);

  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;

  m_numCTUsInWidth = ( m_picWidth / m_maxCUWidth ) + ( ( m_picWidth % m_maxCUWidth ) ? 1 : 0 );
  m_numCTUsInHeight = ( m_picHeight / m_maxCUHeight ) + ( ( m_picHeight % m_maxCUHeight ) ? 1 : 0 );
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;
#if JVET_AL0142_CCSAO_REUSE_CTU
  CHECK(m_numCTUsInPic > MAX_CCSAO_CTU_NUM, "CCSAO CTU out of range");
#endif

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    if( m_ccSaoControl[compIdx] ) { delete[] m_ccSaoControl[compIdx]; m_ccSaoControl[compIdx] = nullptr; }
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    if(m_ccSaoControl[compIdx] == nullptr) { m_ccSaoControl[compIdx] = new uint8_t[m_numCTUsInPic]; }
    ::memset(m_ccSaoControl[compIdx], 0, sizeof(uint8_t) * m_numCTUsInPic);
  }
#endif
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter.create();
#endif
}

void SampleAdaptiveOffset::destroy()
{
  m_tempBuf.destroy();

#if JVET_W0066_CCSAO
#if !RPR_ENABLE
  if( !m_created )
  {
    return;
  }
#endif
  m_created = false;

  m_ccSaoBuf.destroy();

  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    if (m_ccSaoControl[compIdx]) { delete [] m_ccSaoControl[compIdx]; m_ccSaoControl[compIdx] = nullptr; }
  }
#endif
}

void SampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, int typeIdc, int typeAuxInfo, int* dstOffsets, int* srcOffsets)
{
  int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO
  {
    for(int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    CHECK(dstOffsets[SAO_CLASS_EO_PLAIN] != 0, "EO offset is not '0'"); //keep EO plain offset as zero
  }

}

int SampleAdaptiveOffset::getMergeList(CodingStructure& cs, int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const PreCalcValues& pcv = *cs.pcv;

  int ctuX = ctuRsAddr % pcv.widthInCtus;
  int ctuY = ctuRsAddr / pcv.widthInCtus;
  const CodingUnit& cu = *cs.getCU(Position(ctuX*pcv.maxCUWidth, ctuY*pcv.maxCUHeight), CH_L);
  int mergedCTUPos;
  int numValidMergeCandidates = 0;

  for(int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      {
        if(ctuY > 0)
        {
          mergedCTUPos = ctuRsAddr- pcv.widthInCtus;
          if(cs.getCURestricted(Position(ctuX*pcv.maxCUWidth, (ctuY-1)*pcv.maxCUHeight), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    case SAO_MERGE_LEFT:
      {
        if(ctuX > 0)
        {
          mergedCTUPos = ctuRsAddr- 1;
          if(cs.getCURestricted(Position((ctuX-1)*pcv.maxCUWidth, ctuY*pcv.maxCUHeight), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    default:
      {
        THROW("not a supported merge type");
      }
    }

    mergeList[mergeType]=mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


void SampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const int numberOfComponents = m_numberOfComponents;
  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        CHECK(mergeTarget == NULL, "Merge target does not exist");

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        THROW("Not a supported mode");
      }
    }
  }
}

void SampleAdaptiveOffset::xReconstructBlkSAOParams(CodingStructure& cs, SAOBlkParam* saoBlkParams)
{
  for(uint32_t compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;
  }

  const uint32_t numberOfComponents = getNumberValidComponents(cs.pcv->chrFormat);

  for(int ctuRsAddr=0; ctuRsAddr< cs.pcv->sizeInCtus; ctuRsAddr++)
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(cs, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);

    for(uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if(saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}


void SampleAdaptiveOffset::offsetBlock(const int channelBitDepth, const ClpRng& clpRng, int typeIdx, int* offset
                                          , const Pel* srcBlk, Pel* resBlk, int srcStride, int resStride,  int width, int height
                                          , bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail
                                          , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
  )
{
  int x,y, startX, startY, endX, endY, edgeType;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;

  const Pel* srcLine = srcBlk;
        Pel* resLine = resBlk;

  switch(typeIdx)
  {
  case SAO_TYPE_EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight = (int8_t)sgn(srcLine[x] - srcLine[x+1]);
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            signLeft = -signRight;
            continue;
          }
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine  += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[0];

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      const Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
      }

      const Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=0; x< width; x++)
        {
          signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signUpLine[x] = -signDown;
            continue;
          }
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;

          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_135:
    {
      offset += 2;
      int8_t *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = &m_signLineBuf1[0];
      signDownLine= &m_signLineBuf2[0];

      startX = isLeftAvail ? 0 : 1 ;
      endX   = isRightAvail ? width : (width-1);

      //prepare 2nd line's upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];

        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine  += srcStride;
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=startX; x<endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x+ 1]);
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signDownLine[x + 1] = -signDown;
            continue;
          }
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

          signDownLine[x+1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng);

      }
    }
    break;
  case SAO_TYPE_EO_45:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[1];

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
      }


      //first line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signUpLine[x - 1] = -signDown;
            continue;
          }
          edgeType =  signDown + signUpLine[x];
          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

      }
    }
    break;
  case SAO_TYPE_BO:
    {
      const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = ClipPel<int>(srcLine[x] + offset[srcLine[x] >> shiftBits], clpRng );
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      THROW("Not a supported SAO types\n");
    }
  }
}

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER || JVET_W0066_CCSAO
void SampleAdaptiveOffset::offsetBlockNoClip(const int channelBitDepth, const ClpRng& clpRng, int typeIdx, int* offset
                                             , const Pel* srcBlk, Pel* resBlk, int srcStride, int resStride,  int width, int height
                                             , bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                             , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
)
{
  int x,y, startX, startY, endX, endY, edgeType;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;
  
  const Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;
  
  switch(typeIdx)
  {
    case SAO_TYPE_EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      for (y=0; y< height; y++)
      {
        signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX-1]);
        for (x=startX; x< endX; x++)
        {
          signRight = (int8_t)sgn(srcLine[x] - srcLine[x+1]);
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            signLeft = -signRight;
            continue;
          }
#endif
          edgeType =  signRight + signLeft;
          signLeft  = -signRight;
          
          resLine[x] = srcLine[x] + offset[edgeType];
        }
        srcLine  += srcStride;
        resLine += resStride;
      }
      
    }
      break;
    case SAO_TYPE_EO_90:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[0];
      
      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }
      
      const Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
      }
      
      const Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;
        
        for (x=0; x< width; x++)
        {
          signDown  = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signUpLine[x] = -signDown;
            continue;
          }
#endif
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;
          
          resLine[x] = srcLine[x] + offset[edgeType];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
      
    }
      break;
    case SAO_TYPE_EO_135:
    {
      offset += 2;
      int8_t *signUpLine, *signDownLine, *signTmpLine;
      
      signUpLine  = &m_signLineBuf1[0];
      signDownLine= &m_signLineBuf2[0];
      
      startX = isLeftAvail ? 0 : 1 ;
      endX   = isRightAvail ? width : (width-1);
      
      //prepare 2nd line's upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX; x< endX+1; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }
      
      //1st line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
#endif
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];
        
        resLine[x] = srcLine[x] + offset[edgeType];
      }
      srcLine  += srcStride;
      resLine  += resStride;
      
      
      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;
        
        for (x=startX; x<endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x+ 1]);
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signDownLine[x + 1] = -signDown;
            continue;
          }
#endif
          edgeType =  signDown + signUpLine[x];
          resLine[x] = srcLine[x] + offset[edgeType];
          
          signDownLine[x+1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX-1]);
        
        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;
        
        srcLine += srcStride;
        resLine += resStride;
      }
      
      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
#endif
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = srcLine[x] + offset[edgeType];
        
      }
    }
      break;
    case SAO_TYPE_EO_45:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[1];
      
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);
      
      //prepare 2nd line upper sign
      const Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x+1]);
      }
      
      
      //first line
      const Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
#endif
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
        resLine[x] = srcLine[x] + offset[edgeType];
      }
      srcLine += srcStride;
      resLine += resStride;
      
      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;
        
        for(x= startX; x< endX; x++)
        {
          signDown =  (int8_t)sgn(srcLine[x] - srcLineBelow[x-1]);
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            signUpLine[x - 1] = -signDown;
            continue;
          }
#endif
          edgeType =  signDown + signUpLine[x];
          resLine[x] = srcLine[x] + offset[edgeType];
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (int8_t)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }
      
      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
#endif
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
        resLine[x] = srcLine[x] + offset[edgeType];
        
      }
    }
      break;
    case SAO_TYPE_BO:
    {
      const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = srcLine[x] + offset[srcLine[x] >> shiftBits];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
      break;
    default:
    {
      THROW("Not a supported SAO types\n");
    }
  }
}
#endif

void SampleAdaptiveOffset::offsetCTU( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents( area.chromaFormat );
  bool bAllOff=true;
  for( uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;

  //block boundary availability
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

  const size_t lineBufferSize = area.Y().width + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { -1,-1,-1 };
  int verVirBndryPos[] = { -1,-1,-1 };
  int horVirBndryPosComp[] = { -1,-1,-1 };
  int verVirBndryPosComp[] = { -1,-1,-1 };
  bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader );
  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);
    const CompArea& compArea = area.block(compID);
    SAOOffset& ctbOffset     = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      int  srcStride    = src.get(compID).stride;
      const Pel* srcBlk = src.get(compID).bufAt(compArea);
      int  resStride    = res.get(compID).stride;
      Pel* resBlk       = res.get(compID).bufAt(compArea);
      for (int i = 0; i < numHorVirBndry; i++)
      {
        horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
      }
      for (int i = 0; i < numVerVirBndry; i++)
      {
        verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
      }

      offsetBlock( cs.sps->getBitDepth(toChannelType(compID)),
                   cs.slice->clpRng(compID),
                   ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
                  );
    }
  } //compIdx
}

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER || JVET_W0066_CCSAO
void SampleAdaptiveOffset::offsetCTUnoClip( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents( area.chromaFormat );
  bool bAllOff=true;
  for( uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }
  
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;
  
  //block boundary availability
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);
  
  const size_t lineBufferSize = area.Y().width + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }
  
#if !JVET_W0066_CCSAO || JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { -1,-1,-1 };
  int verVirBndryPos[] = { -1,-1,-1 };
  int horVirBndryPosComp[] = { -1,-1,-1 };
  int verVirBndryPosComp[] = { -1,-1,-1 };
  bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader);
#endif
  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);
    const CompArea& compArea = area.block(compID);
    SAOOffset& ctbOffset     = saoblkParam[compIdx];
    
    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      int  srcStride    = src.get(compID).stride;
      const Pel* srcBlk = src.get(compID).bufAt(compArea);
      int  resStride    = res.get(compID).stride;
      Pel* resBlk       = res.get(compID).bufAt(compArea);
#if !JVET_W0066_CCSAO || JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      for (int i = 0; i < numHorVirBndry; i++)
      {
        horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
      }
      for (int i = 0; i < numVerVirBndry; i++)
      {
        verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
      }
#endif
#if JVET_W0066_CCSAO
      // Do not clip the final output for both luma and chroma. Clipping is done jontly for SAO/BIF/CCSAO.
      
	    offsetBlockNoClip(cs.sps->getBitDepth(toChannelType(compID)),
		    cs.slice->clpRng(compID),
		    ctbOffset.typeIdc, ctbOffset.offset
		    , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
		    , isLeftAvail, isRightAvail
		    , isAboveAvail, isBelowAvail
		    , isAboveLeftAvail, isAboveRightAvail
		    , isBelowLeftAvail, isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
		    , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
	    );
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if(isLuma(compID) || isChroma(compID))
#else
      if(compID == COMPONENT_Y)
#endif
      {
        // If it is luma we should not clip, since we will clip
        // after BIF has been added.

        offsetBlockNoClip( cs.sps->getBitDepth(toChannelType(compID)),
                    cs.slice->clpRng(compID),
                    ctbOffset.typeIdc, ctbOffset.offset
                    , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
                    , isLeftAvail, isRightAvail
                    , isAboveAvail, isBelowAvail
                    , isAboveLeftAvail, isAboveRightAvail
                    , isBelowLeftAvail, isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                    , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
                    );
        }
      else
      {
        // If it is chroma we should clip as normal, since
        // chroma is not bilaterally filtered.
        offsetBlock( cs.sps->getBitDepth(toChannelType(compID)),
                    cs.slice->clpRng(compID),
                    ctbOffset.typeIdc, ctbOffset.offset
                    , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height
                    , isLeftAvail, isRightAvail
                    , isAboveAvail, isBelowAvail
                    , isAboveLeftAvail, isAboveRightAvail
                    , isBelowLeftAvail, isBelowRightAvail
                    , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
                    );

      }
      
#endif
    }
  } //compIdx
}
#endif

void SampleAdaptiveOffset::SAOProcess( CodingStructure& cs, SAOBlkParam* saoBlkParams )
{
  CHECK(!saoBlkParams, "No parameters present");

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  // In code without BIF, SAOProcess would not be run if 'SAO=0'.
  // However, in the BIF-enabled code, we still might go here if 'SAO=0' and 'BIF=1'.
  // Hence we must check getSAOEnabledFlag() for some of the function calls.
  if( cs.sps->getSAOEnabledFlag() )
  {
    xReconstructBlkSAOParams(cs, saoBlkParams);
  }
#else
  xReconstructBlkSAOParams(cs, saoBlkParams);
#endif

  const uint32_t numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);
  bool bAllDisabled = true;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  // If 'SAO=0' we would not normally get here. However, now we might get
  // here if 'SAO=0' and 'BIF=1'. Hence we should only run this if
  // getSAOEnabledFlag() is true. Note that if getSAOEnabledFlag() is false,
  // we will not run the code and bAllDisabled will stay true, which will
  // give the correct behavior.
  if( cs.sps->getSAOEnabledFlag() )
  {
#endif
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_picSAOEnabled[compIdx])
    {
      bAllDisabled = false;
    }
  }
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  }
#endif
  if (bAllDisabled)
  {
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    if(!cs.pps->getUseBIF() && !cs.pps->getUseChromaBIF())
#else
    // Even if we are not doing SAO we might still need to do BIF
    // so we cannot return even if SAO is never used.
    if(!cs.pps->getUseBIF())
#endif
    {
      // However, if we are not doing BIF it is safe to return.
      return;
    }
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    if(!cs.pps->getUseChromaBIF())
    {
      return;
    }
#else
    return;
#endif
#endif
  }

  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf rec = cs.getRecoBuf();
  m_tempBuf.copyFrom( rec );

  int ctuRsAddr = 0;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

#if JVET_W0066_CCSAO
      // Always do non-clipped version for SAO/BIF, the clipping is done jointly after CCSAO is also applied
      if( !bAllDisabled )
      {
        offsetCTUnoClip( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs );
      }
#if JVET_V0094_BILATERAL_FILTER
      if (cs.pps->getUseBIF())
      {
        BifParams& bifParams = cs.picture->getBifParam( COMPONENT_Y );

        // And now we traverse the CTU to do BIF
        for (auto& currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
        {
          for (auto& currTU : CU::traverseTUs(currCU))
          {
            bool applyBIF = bifParams.ctuOn[ctuRsAddr];
            if (applyBIF)
            {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
              applyBIF = m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              applyBIF = ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
            }
            if (applyBIF)
            {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
              int  numHorVirBndry = 0, numVerVirBndry = 0;
              int  horVirBndryPos[]               = { 0, 0, 0 };
              int  verVirBndryPos[]               = { 0, 0, 0 };
              bool isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop, clipBottom,
                clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif
              m_bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
              );
            }
          }
        }
      }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if(cs.pps->getUseChromaBIF())
      {
        bool isDualTree = CS::isDualITree(cs);
        ChannelType chType = isDualTree ? CH_C : CH_L;
        bool applyChromaBIF = false;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
        const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, cs.pcv->chrFormat );
#endif

        // And now we traverse the CTU to do BIF
        for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
        {
          bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
          if(!chromaValid)
          {
            continue;
          }
          for (auto &currTU : CU::traverseTUs(currCU))
          {
            for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              applyChromaBIF = false;
              ComponentID compID = ComponentID( compIdx );
              BifParams& chromaBifParams = cs.picture->getBifParam( compID );
              bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
              applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
              bool tuValid = false;
              bool tuCBF = false;
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              if(!isDualTree)
              {
                tuValid = currTU.blocks[compIdx].valid();
                tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                if(tuValid)
                {
                  tuCBF = TU::getCbf(currTU, compID);
                }
                applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
              }
              else
              {
                tuCBF = TU::getCbf(currTU, compID);
                applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
              }
#endif

              if(applyChromaBIF)
              {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int       numHorVirBndry = 0, numVerVirBndry = 0;
                int       horVirBndryPos[]               = { 0, 0, 0 };
                int       verVirBndryPos[]               = { 0, 0, 0 };
                CompArea &myArea                         = currTU.block(compID);
                int       yPos                           = myArea.y << chromaScaleY;
                int       xPos                           = myArea.x << chromaScaleX;
                bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                  clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                m_bilateralFilter.bilateralFilterDiamond5x5( compID, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(compID), currTU, true
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                );
              }
            }
          }
        }
      }
#endif
#else
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
      if(cs.pps->getUseBIF() || cs.pps->getUseChromaBIF())
#else
      if(cs.pps->getUseBIF())
#endif
      {
        // We are using BIF, so we run SAO without clipping
        // However, if 'SAO=0', bAllDisabled=true and we should not run offsetCTUnoClip.
        if( !bAllDisabled )
        {
          offsetCTUnoClip( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs );
        }
        
        // We don't need to clip if SAO was not performed on luma.
        SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
        SAOOffset& myCtbOffset     = mySAOblkParam[0];
        BifParams& bifParams = cs.picture->getBifParam(COMPONENT_Y);
        
        bool clipLumaIfNoBilat = false;
        if( !bAllDisabled && myCtbOffset.modeIdc != SAO_MODE_OFF )
        {
          clipLumaIfNoBilat = true;
        }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
        SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];

        bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

        if(!bAllDisabled && myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
        {
          clipChromaIfNoBilat[COMPONENT_Cb] = true;
        }
        if(!bAllDisabled && myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
        {
          clipChromaIfNoBilat[COMPONENT_Cr] = true;
        }
        if(cs.pps->getUseBIF())
        {
#endif
        
        // And now we traverse the CTU to do BIF
        for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
        {
          for (auto &currTU : CU::traverseTUs(currCU))
          {
            
            bool applyBIF = bifParams.ctuOn[ctuRsAddr];
            if (applyBIF)
            {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
              applyBIF = m_bilateralFilter.getApplyBIF(currTU, COMPONENT_Y);
#else
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              applyBIF = ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
            }
            if (applyBIF)
            {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
              int  numHorVirBndry = 0, numVerVirBndry = 0;
              int  horVirBndryPos[]               = { 0, 0, 0 };
              int  verVirBndryPos[]               = { 0, 0, 0 };
              bool isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                cs, currTU.Y().x, currTU.Y().y, currTU.lumaSize().width, currTU.lumaSize().height, clipTop, clipBottom,
                clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
#endif

              m_bilateralFilter.bilateralFilterDiamond5x5( COMPONENT_Y, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(COMPONENT_Y), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry
                  , clipTop, clipBottom, clipLeft, clipRight
#endif
              );
            }
            else
            {
              // We don't need to clip if SAO was not performed on luma.
              if( clipLumaIfNoBilat )
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, m_tempBuf, rec, cs.slice->clpRng( COMPONENT_Y ), currTU );
              }
            }
          }
        }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        } // BIF LUMA is disabled
        else
        {
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
          {
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              if(clipLumaIfNoBilat)
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Y), currTU);
              }
            }
          }
        }
        if(cs.pps->getUseChromaBIF())
        {          
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          bool applyChromaBIF = false;
          // And now we traverse the CTU to do BIF
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
              {
                ComponentID compID = ComponentID( compIdx );
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                bool tuValid = false;
                bool tuCBF = false;
                bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                applyChromaBIF = false;
                if(!isDualTree)
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if(tuValid)
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                  }
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                }
                else
                {
                  tuCBF = TU::getCbf(currTU, compID);
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                }
#endif
                if(applyChromaBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int       numHorVirBndry = 0, numVerVirBndry = 0;
                  int       horVirBndryPos[]               = { 0, 0, 0 };
                  int       verVirBndryPos[]               = { 0, 0, 0 };
                  CompArea &myArea                         = currTU.block(compID);
                  const int chromaScaleX                   = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                  const int chromaScaleY                   = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                  int       yPos                           = myArea.y << chromaScaleY;
                  int       xPos                           = myArea.x << chromaScaleX;
                  bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                    clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                  m_bilateralFilter.bilateralFilterDiamond5x5( compID, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
                else
                {
                  bool useClip = clipChromaIfNoBilat[compID];
                  if(useClip && currTU.blocks[compIdx].valid())
                  {
                    m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, m_tempBuf, rec, cs.slice->clpRng(compID), currTU );
                  }
                }
              }
            }
          }
        }// BIF chroma is disabled
        else
        {
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;

          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              if( clipChromaIfNoBilat[COMPONENT_Cb] && currTU.blocks[COMPONENT_Cb].valid())
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cb, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Cb), currTU );
              }
              if( clipChromaIfNoBilat[COMPONENT_Cr] && currTU.blocks[COMPONENT_Cr].valid())
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cr, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Cr), currTU );
              }
            }
          }
        }
#endif
      }
      else
      {
        // BIF is not used, use old SAO code
        offsetCTU( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
      }
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if( !bAllDisabled )
        {
          offsetCTUnoClip( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
        }

        SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
        SAOOffset& myCtbOffset     = mySAOblkParam[0];

        bool clipLumaIfNoBilat = false;
        if(!bAllDisabled && myCtbOffset.modeIdc != SAO_MODE_OFF)
        {
          clipLumaIfNoBilat = true;
        }

        SAOOffset& myCtbOffsetCb     = mySAOblkParam[1];
        SAOOffset& myCtbOffsetCr     = mySAOblkParam[2];

        bool clipChromaIfNoBilat[MAX_NUM_COMPONENT] = { false };

        if(!bAllDisabled && myCtbOffsetCb.modeIdc != SAO_MODE_OFF)
        {
          clipChromaIfNoBilat[COMPONENT_Cb] = true;
        }
        if(!bAllDisabled && myCtbOffsetCr.modeIdc != SAO_MODE_OFF)
        {
          clipChromaIfNoBilat[COMPONENT_Cr] = true;
        }

        for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, CH_L), CH_L))
        {
          for (auto &currTU : CU::traverseTUs(currCU))
          {
            if(clipLumaIfNoBilat)
            {
              m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Y, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Y), currTU );
            }
          }
        }

        if(cs.pps->getUseChromaBIF())
        {
          bool tuValid = false;
          bool tuCBF = false;
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          bool applyChromaBIF = false;
          // And now we traverse the CTU to do BIF
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
              for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
              {
                ComponentID compID = ComponentID( compIdx );
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                bool ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                applyChromaBIF = false;
                if(!isDualTree)
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if(tuValid)
                  {
                    tuCBF = TU::getCbf(currTU, compID);
                  }
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)) && (tuValid));
                }
                else
                {
                  tuCBF = TU::getCbf(currTU, compID);
                  applyChromaBIF = (ctuEnableChromaBIF && ((tuCBF || isInter == false) && (currTU.cu->qp > 17)));
                }
#endif
                if(applyChromaBIF)
                {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int       numHorVirBndry = 0, numVerVirBndry = 0;
                  int       horVirBndryPos[]               = { 0, 0, 0 };
                  int       verVirBndryPos[]               = { 0, 0, 0 };
                  CompArea &myArea                         = currTU.block(compID);
                  const int chromaScaleX                   = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                  const int chromaScaleY                   = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                  int       yPos                           = myArea.y << chromaScaleY;
                  int       xPos                           = myArea.x << chromaScaleX;
                  bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                    clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

#endif
                  m_bilateralFilter.bilateralFilterDiamond5x5( compID, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                  , isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight
#endif
                  );
                }
                else
                {
                  bool useClip = clipChromaIfNoBilat[compID];
                  if(useClip && currTU.blocks[compIdx].valid())
                  {
                    m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, m_tempBuf, rec, cs.slice->clpRng(compID), currTU );
                  }
                }
              }
            }
          }
        }// BIF chroma off
        else
        {
          bool isDualTree = CS::isDualITree(cs);
          ChannelType chType = isDualTree ? CH_C : CH_L;
          for (auto &currCU : cs.traverseCUs(CS::getArea(cs, area, chType), chType))
          {
            bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
            if(!chromaValid)
            {
              continue;
            }
            for (auto &currTU : CU::traverseTUs(currCU))
            {
              if( clipChromaIfNoBilat[COMPONENT_Cb] && currTU.blocks[COMPONENT_Cb].valid())
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cb, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Cb), currTU );
              }
              if( clipChromaIfNoBilat[COMPONENT_Cr] && currTU.blocks[COMPONENT_Cr].valid())
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks( COMPONENT_Cr, m_tempBuf, rec, cs.slice->clpRng(COMPONENT_Cr), currTU );
              }
            }
          }
        }
#else
      offsetCTU( area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
#endif
#endif
#endif
      ctuRsAddr++;
    }
  }

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->getPOC())));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "SAO" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );

}

#if JVET_W0066_CCSAO
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
std::vector<CcSaoPrvParam> g_ccSaoPrvParam[MAX_NUM_COMPONENT];
#endif

void SampleAdaptiveOffset::CCSAOProcess(CodingStructure& cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);
  bool bAllDisabled = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      bAllDisabled = false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }
  
  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf dstYuv = cs.getRecoBuf();
  PelUnitBuf srcYuv = m_ccSaoBuf.getBuf( cs.area );
  srcYuv.extendBorderPel( MAX_CCSAO_FILTER_LENGTH >> 1 );

  applyCcSao(cs, pcv, srcYuv, dstYuv);
}
void SampleAdaptiveOffset::applyCcSao(CodingStructure &cs, const PreCalcValues& pcv, const CPelUnitBuf& srcYuv, PelUnitBuf& dstYuv)
{
  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth  > pcv.lumaWidth ) ? (pcv.lumaWidth  - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      offsetCTUCcSaoNoClip(cs, area, srcYuv, dstYuv, ctuRsAddr);
      ctuRsAddr++;
    }
  }
}

void SampleAdaptiveOffset::jointClipSaoBifCcSao(CodingStructure& cs)
{
#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (!cs.sps->getSAOEnabledFlag() && !cs.pps->getUseBIF() && !cs.pps->getUseChromaBIF() && !cs.sps->getCCSAOEnabledFlag())
#else
  if (!cs.sps->getSAOEnabledFlag() && !cs.pps->getUseBIF() && !cs.sps->getCCSAOEnabledFlag())
#endif
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if (!cs.sps->getSAOEnabledFlag() && !cs.sps->getCCSAOEnabledFlag() && !cs.pps->getUseChromaBIF())
#else
  if (!cs.sps->getSAOEnabledFlag() && !cs.sps->getCCSAOEnabledFlag())
#endif
#endif
  {
    return;
  }

  const PreCalcValues& pcv = *cs.pcv;
  PelUnitBuf dstYuv = cs.getRecoBuf();

  // Iterate all CTUs and check if any of the filters is on for a given component
  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);

      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        ComponentID compID = ComponentID( compIdx );
        bool saoOn = false;
        bool ccsaoOn = false;
        if (cs.sps->getSAOEnabledFlag())
        {
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset& myCtbOffset = mySAOblkParam[compIdx];
          saoOn = myCtbOffset.modeIdc != SAO_MODE_OFF;
        }
        if (cs.sps->getCCSAOEnabledFlag())
        {
          const int setIdc = m_ccSaoControl[compIdx][ctuRsAddr];
          ccsaoOn = m_ccSaoComParam.enabled[compIdx] && setIdc != 0;
        }
        if (ccsaoOn || saoOn)
        {
          // We definitely need to clip if either SAO or CCSAO is on for the given component of the CTU                  
          clipCTU(cs, dstYuv, area, compID );
        }
#if JVET_V0094_BILATERAL_FILTER
        else
        {
          // When BIF is on, the luma component might need to be clipped
          if (cs.pps->getUseBIF() && isLuma( compID ) )
          {
            BifParams& bifParams = cs.picture->getBifParam( compID );

            // And now we traverse the CTU to do clipping
            for( auto& currCU : cs.traverseCUs( CS::getArea( cs, area, CH_L ), CH_L ) )
            {
              for( auto& currTU : CU::traverseTUs( currCU ) )
              {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                bool isInter = ( currCU.predMode == MODE_INTER ) ? true : false;
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && ((TU::getCbf(currTU, compID) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
                if (applyBIF)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, m_tempBuf, dstYuv, cs.slice->clpRng( compID ), currTU );
                }
              }
            }
          }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          if( cs.pps->getUseChromaBIF() && isChroma( compID ) )
          {
            bool ctuEnableChromaBIF = false;
            bool isDualTree = CS::isDualITree( cs );
            ChannelType chType = isDualTree ? CH_C : CH_L;
            bool applyChromaBIF = false;
            for( auto &currCU : cs.traverseCUs( CS::getArea( cs, area, chType ), chType ) )
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if( !chromaValid )
              {
                continue;
              }
              for( auto &currTU : CU::traverseTUs( currCU ) )
              {
                //Cb or Cr
                applyChromaBIF = false;
                BifParams& chromaBifParams = cs.picture->getBifParam( compID );
                ctuEnableChromaBIF = chromaBifParams.ctuOn[ctuRsAddr];

#if JVET_AF0112_BIF_DYNAMIC_SCALING
                applyChromaBIF = ctuEnableChromaBIF && m_bilateralFilter.getApplyBIF(currTU, compID);
#else
                bool tuValid = false;
                bool tuCBF = false;
                bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
                if( !isDualTree )
                {
                  tuValid = currTU.blocks[compIdx].valid();
                  tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
                  if( tuValid )
                  {
                    tuCBF = TU::getCbf( currTU, compID );
                  }
                  applyChromaBIF = ( ctuEnableChromaBIF && ( ( tuCBF || isInter == false ) && ( currTU.cu->qp > 17 ) ) && ( tuValid ) );
                }
                else
                {
                  tuCBF = TU::getCbf( currTU, compID );
                  applyChromaBIF = ( ctuEnableChromaBIF && ( ( tuCBF || isInter == false ) && ( currTU.cu->qp > 17 ) ) );
                }
#endif
                if( applyChromaBIF )
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks( compID, m_tempBuf, dstYuv, cs.slice->clpRng( compID ), currTU );
                }
              }
            }
          }
#endif
        }
#endif
      }
      ctuRsAddr++;
    }
  }
}

void SampleAdaptiveOffset::clipCTU(CodingStructure& cs, PelUnitBuf& dstYuv, const UnitArea& area, const ComponentID compID)
{
  const CompArea &compArea = area.block(compID);
  const uint32_t height = compArea.height;
  const uint32_t width = compArea.width;
  Pel *dst = dstYuv.get(compID).bufAt(area.block(compID));
  int dstStride = dstYuv.get(compID).stride;
  
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      // new result = old result (which is SAO-treated already) + clipping
      dst[x] = ClipPel<int>(dst[x], cs.slice->clpRng(compID));
    }
    dst += dstStride;
  }
}

void SampleAdaptiveOffset::offsetCTUCcSaoNoClip(CodingStructure& cs, const UnitArea& area, const CPelUnitBuf& srcYuv, PelUnitBuf& dstYuv, const int ctuRsAddr)
{
  const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);
  bool bAllOff = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      bAllOff = false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { -1,-1,-1 };
  int verVirBndryPos[] = { -1,-1,-1 };
  int horVirBndryPosComp[] = { -1,-1,-1 };
  int verVirBndryPosComp[] = { -1,-1,-1 };
  bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader);
#endif

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      const int setIdc = m_ccSaoControl[compIdx][ctuRsAddr];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
      int setType = m_ccSaoComParam.setType[compIdx][setIdc - 1];
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
      if (setType == CCSAO_SET_TYPE_EDGE)
      {
#else
      if (setType)
      {
        /* Edge offset over here */
#endif
        if (setIdc != 0)
        {
          const ComponentID compID     = ComponentID(compIdx);
          const CompArea   &compArea   = area.block(compID);
          const int         srcStrideY = srcYuv.get(COMPONENT_Y ).stride;
          const int         srcStrideU = srcYuv.get(COMPONENT_Cb).stride;
          const int         srcStrideV = srcYuv.get(COMPONENT_Cr).stride;
          const Pel        *srcBlkY    = srcYuv.get(COMPONENT_Y ).bufAt(area.block(COMPONENT_Y ));
          const Pel        *srcBlkU    = srcYuv.get(COMPONENT_Cb).bufAt(area.block(COMPONENT_Cb));
          const Pel        *srcBlkV    = srcYuv.get(COMPONENT_Cr).bufAt(area.block(COMPONENT_Cr));
          const int         dstStride  = dstYuv.get(compID).stride;
          Pel              *dstBlk     = dstYuv.get(compID).bufAt(compArea);

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const uint16_t edgeCmp  = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMPONENT_Cb];
          const uint16_t edgeIdc  = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cr];
          const uint16_t edgeDir  = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMPONENT_Y ];
          const uint16_t edgeThr  = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cb];
          const uint16_t bandIdc  = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Y ];
#else
          const uint16_t candPosY = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMPONENT_Y];  /* Edge Type */
          const uint16_t bandNumY = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Y];  /* Num Bands */
          const uint16_t bandNumU = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cb]; /* treshold */
          const uint16_t bandNumV = bandNumU;
#endif
          const short   *offset   = m_ccSaoComParam.offset[compIdx][setIdc - 1];
      
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          for (int i = 0; i < numHorVirBndry; i++)
          {
            horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
          }
          for (int i = 0; i < numVerVirBndry; i++)
          {
            verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
          }
#endif

          offsetBlockCcSaoNoClipEdge(compID, cs.sps->getChromaFormatIdc(), cs.sps->getBitDepth(toChannelType(compID)), cs.slice->clpRng(compID)
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                                   , edgeCmp
                                   , edgeDir, bandIdc, edgeThr, edgeIdc
#else
                                   , candPosY, bandNumY, bandNumU, bandNumV
#endif
                                   , offset
                                   , srcBlkY, srcBlkU, srcBlkV, dstBlk, srcStrideY, srcStrideU, srcStrideV, dstStride
                                   , compArea.width, compArea.height
                                   , isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                   , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
          );
        }
      }
      else
      {
#endif
        if (setIdc != 0)
        {
          const ComponentID compID     = ComponentID(compIdx);
          const CompArea   &compArea   = area.block(compID);
          const int         srcStrideY = srcYuv.get(COMPONENT_Y ).stride;
          const int         srcStrideU = srcYuv.get(COMPONENT_Cb).stride;
          const int         srcStrideV = srcYuv.get(COMPONENT_Cr).stride;
          const Pel        *srcBlkY    = srcYuv.get(COMPONENT_Y ).bufAt(area.block(COMPONENT_Y ));
          const Pel        *srcBlkU    = srcYuv.get(COMPONENT_Cb).bufAt(area.block(COMPONENT_Cb));
          const Pel        *srcBlkV    = srcYuv.get(COMPONENT_Cr).bufAt(area.block(COMPONENT_Cr));
          const int         dstStride  = dstYuv.get(compID).stride;
          Pel              *dstBlk     = dstYuv.get(compID).bufAt(compArea);

          const uint16_t candPosY = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMPONENT_Y ];
          const uint16_t bandNumY = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Y ];
          const uint16_t bandNumU = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cb];
          const uint16_t bandNumV = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cr];
          const short   *offset   = m_ccSaoComParam.offset [compIdx][setIdc - 1];

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          for (int i = 0; i < numHorVirBndry; i++)
          {
            horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
          }
          for (int i = 0; i < numVerVirBndry; i++)
          {
            verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
          }
#endif
      
          offsetBlockCcSaoNoClip(compID, cs.sps->getChromaFormatIdc(), cs.sps->getBitDepth(toChannelType(compID)), cs.slice->clpRng(compID)
                               , candPosY, bandNumY, bandNumU, bandNumV
                               , offset
                               , srcBlkY, srcBlkU, srcBlkV, dstBlk, srcStrideY, srcStrideU, srcStrideV, dstStride
                               , compArea.width, compArea.height
                               , isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                               , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
                                );
        }
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
      }
#endif
    }
  }
}

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void SampleAdaptiveOffset::offsetCTUCcSao(CodingStructure& cs, const UnitArea& area, const CPelUnitBuf& srcYuv, PelUnitBuf& dstYuv, const int ctuRsAddr)
{
  const uint32_t numberOfComponents = getNumberValidComponents( area.chromaFormat );
  bool bAllOff = true;
  for( uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      bAllOff = false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail;
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { -1,-1,-1 };
  int verVirBndryPos[] = { -1,-1,-1 };
  int horVirBndryPosComp[] = { -1,-1,-1 };
  int verVirBndryPosComp[] = { -1,-1,-1 };
  bool isCtuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(area.Y().x, area.Y().y, area.Y().width, area.Y().height, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, cs.picHeader);
#endif

  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if(m_ccSaoComParam.enabled[compIdx])
    {
      const int setIdc = m_ccSaoControl[compIdx][ctuRsAddr];

      if (setIdc != 0)
      {
        const ComponentID compID     = ComponentID(compIdx);
        const CompArea   &compArea   = area.block(compID);
        const int         srcStrideY = srcYuv.get(COMPONENT_Y ).stride;
        const int         srcStrideU = srcYuv.get(COMPONENT_Cb).stride;
        const int         srcStrideV = srcYuv.get(COMPONENT_Cr).stride;
        const int         dstStride  = dstYuv.get(compID      ).stride;
        const Pel        *srcBlkY    = srcYuv.get(COMPONENT_Y ).bufAt(area.block(COMPONENT_Y ));
        const Pel        *srcBlkU    = srcYuv.get(COMPONENT_Cb).bufAt(area.block(COMPONENT_Cb));
        const Pel        *srcBlkV    = srcYuv.get(COMPONENT_Cr).bufAt(area.block(COMPONENT_Cr));
              Pel        *dstBlk     = dstYuv.get(compID      ).bufAt(compArea);
              
        const uint16_t    candPosY   = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMPONENT_Y ];
        const uint16_t    bandNumY   = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Y ];
        const uint16_t    bandNumU   = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cb];
        const uint16_t    bandNumV   = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMPONENT_Cr];
        const short      *offset     = m_ccSaoComParam.offset [compIdx][setIdc - 1];
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        for (int i = 0; i < numHorVirBndry; i++)
        {
          horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
        }
        for (int i = 0; i < numVerVirBndry; i++)
        {
          verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
        }
#endif

        offsetBlockCcSao( compID, cs.sps->getChromaFormatIdc(), cs.sps->getBitDepth(toChannelType(compID)), cs.slice->clpRng(compID)
                        , candPosY, bandNumY, bandNumU, bandNumV
                        , offset
                        , srcBlkY, srcBlkU, srcBlkV, dstBlk
                        , srcStrideY, srcStrideU, srcStrideV, dstStride
                        , compArea.width, compArea.height
                        , isLeftAvail, isRightAvail
                        , isAboveAvail, isBelowAvail
                        , isAboveLeftAvail, isAboveRightAvail
                        , isBelowLeftAvail, isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                        , isCtuCrossedByVirtualBoundaries, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry
#endif
                        );
      }
    }
  }
}
#endif

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
int SampleAdaptiveOffset::getCcSaoClassNum(const int compIdx, const int setIdx, const CcSaoComParam& ccSaoParam)
{
  int classNum = 0;

  if (ccSaoParam.setType[compIdx][setIdx] == CCSAO_SET_TYPE_EDGE)
  {
    int bandIdc = ccSaoParam.bandNum[compIdx][setIdx][COMPONENT_Y ], bandNum = g_ccSaoBandTab[bandIdc][1];
    int edgeIdc = ccSaoParam.bandNum[compIdx][setIdx][COMPONENT_Cr], edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
    classNum = bandNum * edgeNum;
  }
  else
  {
    classNum = ccSaoParam.bandNum[compIdx][setIdx][COMPONENT_Y ]
             * ccSaoParam.bandNum[compIdx][setIdx][COMPONENT_Cb]
             * ccSaoParam.bandNum[compIdx][setIdx][COMPONENT_Cr];
  }
  
  return classNum;
}
#endif

#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
int calcDiffRange(Pel a, Pel b, int th)
{
  int diff      = a - b;
  int value     = 0;
  int thred     = g_ccSaoQuanValue[th];
  int neg_thred = (-1) * thred;
  if (diff < 0)
  {
    if (diff < neg_thred)
    {
      value = 0;
    }
    else
    {
      value = 1;
    }      
  }
  else
  {
    if (diff < thred)
    {
      value = 2;
    }
    else
    {
      value = 3;
    }
  }
  return value;
}
#endif

void SampleAdaptiveOffset::offsetBlockCcSaoNoClipEdge(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth, const ClpRng &clpRng
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
                                                    , const uint16_t edgeCmp
                                                    , const uint16_t edgeDir, const uint16_t bandIdc, const uint16_t edgeThr, const uint16_t edgeIdc
#else
                                                    , const uint16_t candPosY, const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
#endif
                                                    , const short *offset
                                                    , const Pel *srcY, const Pel *srcU, const Pel *srcV, Pel *dst
                                                    , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int dstStride
                                                    , const int width, const int height
                                                    , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                                    , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                                                     )
{
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  const int edgePosXA  = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
  const int edgePosXB  = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];
  const int bandCmp    = g_ccSaoBandTab [bandIdc][0];
  const int bandNum    = g_ccSaoBandTab [bandIdc][1];
#if JVET_AJ0237_INTERNAL_12BIT
  const int bdShift = std::max(0, bitDepth - 10);
  const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr] << bdShift;
#else
  const int edgeThrVal = g_ccSaoEdgeThr [edgeIdc][edgeThr];
#endif
  const int edgeNum    = g_ccSaoEdgeNum [edgeIdc][0];
  const int edgeNumUni = g_ccSaoEdgeNum [edgeIdc][1];
  const int srcStrideE = edgeCmp == COMPONENT_Y ? srcStrideY : edgeCmp == COMPONENT_Cb ? srcStrideU : srcStrideV;
#else
  const int candPosYXA = g_ccSaoEdgeTypeX[candPosY][0];
  const int candPosYYA = g_ccSaoEdgeTypeY[candPosY][0];
  const int candPosYXB = g_ccSaoEdgeTypeX[candPosY][1];
  const int candPosYYB = g_ccSaoEdgeTypeY[candPosY][1];
  int       signa, signb, band;
  int       th = bandNumU - 1;
#endif

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleYM1 = 1 - chromaScaleY;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int x, y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  switch (compID)
  {
  case COMPONENT_Y:
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    switch (edgeDir)
#else
    switch (candPosY)
#endif
    {
    case SAO_TYPE_EO_0:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);
      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + x;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          signa = calcDiffRange(*colY, *colA, th);
          signb = calcDiffRange(*colY, *colB, th);

          signa = signa * 4 + signb;
          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int bandc          = bandNumY - 4;
            band               = (*colU * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            int bandc          = bandNumY - 6;
            band               = (*colV * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
    }
    break;
    case SAO_TYPE_EO_90:
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : (height - 1);
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + x;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          signa = calcDiffRange(*colY, *colA, th);
          signb = calcDiffRange(*colY, *colB, th);

          signa = signa * 4 + signb;
          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int bandc          = bandNumY - 4;
            band               = (*colU * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            int bandc          = bandNumY - 6;
            band               = (*colV * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
    }
    break;
    case SAO_TYPE_EO_135:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // 1st line
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail ? endX : 1;
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + x;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + x;
        const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        signa = calcDiffRange(*colY, *colA, th);
        signb = calcDiffRange(*colY, *colB, th);

        signa = signa * 4 + signb;
        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int bandc          = bandNumY - 4;
          band               = (*colU * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          int bandc          = bandNumY - 6;
          band               = (*colV * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
      srcY += srcStrideY;
      srcU += srcStrideU * chromaScaleYM1;
      srcV += srcStrideV * chromaScaleYM1;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + x;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          signa = calcDiffRange(*colY, *colA, th);
          signb = calcDiffRange(*colY, *colB, th);

          signa = signa * 4 + signb;
          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int bandc          = bandNumY - 4;
            band               = (*colU * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            int bandc          = bandNumY - 6;
            band               = (*colV * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + x;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + x;
        const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        signa = calcDiffRange(*colY, *colA, th);
        signb = calcDiffRange(*colY, *colB, th);

        signa = signa * 4 + signb;
        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int bandc          = bandNumY - 4;
          band               = (*colU * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          int bandc          = bandNumY - 6;
          band               = (*colV * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
    }
    break;
    case SAO_TYPE_EO_45:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + x;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + x;
        const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        signa = calcDiffRange(*colY, *colA, th);
        signb = calcDiffRange(*colY, *colB, th);

        signa = signa * 4 + signb;
        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int bandc          = bandNumY - 4;
          band               = (*colU * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          int bandc          = bandNumY - 6;
          band               = (*colV * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
      srcY += srcStrideY;
      srcU += srcStrideU * chromaScaleYM1;
      srcV += srcStrideV * chromaScaleYM1;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + x;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + x;
          const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          signa = calcDiffRange(*colY, *colA, th);
          signb = calcDiffRange(*colY, *colB, th);

          signa = signa * 4 + signb;
          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int bandc          = bandNumY - 4;
            band               = (*colU * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            int bandc          = bandNumY - 6;
            band               = (*colV * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + x;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + x;
        const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        signa = calcDiffRange(*colY, *colA, th);
        signb = calcDiffRange(*colY, *colB, th);

        signa = signa * 4 + signb;
        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int bandc          = bandNumY - 4;
          band               = (*colU * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          int bandc          = bandNumY - 6;
          band               = (*colV * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
    }
    break;
    }
    break;
  }

  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    switch (edgeDir)
#else
    switch (candPosY)
#endif
    {
    case SAO_TYPE_EO_0:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);
      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
          signa           = calcDiffRange(*colY, *colA, th);
          signb           = calcDiffRange(*colY, *colB, th);
          signa           = signa * 4 + signb;

          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int        bandc   = bandNumY - 4;
            const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            band               = (*colC * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            const Pel *colCT   = (compID == COMPONENT_Cb)
                                   ? srcV + x
                                   : srcU + x; /* also use the remaining third component for bandIdx calc.*/
            int        bandc   = bandNumY - 6;
            band               = (*colCT * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
    }
    break;
    case SAO_TYPE_EO_90:
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : (height - 1);
      if (!isAboveAvail)
      {
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
          signa           = calcDiffRange(*colY, *colA, th);
          signb           = calcDiffRange(*colY, *colB, th);
          signa           = signa * 4 + signb;

          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int        bandc   = bandNumY - 4;
            const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            band               = (*colC * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            const Pel *colCT   = (compID == COMPONENT_Cb)
                                   ? srcV + x
                                   : srcU + x; /* also use the remaining third component for bandIdx calc.*/
            int        bandc   = bandNumY - 6;
            band               = (*colCT * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
    }
    break;
    case SAO_TYPE_EO_135:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // 1st line
      firstLineStartX = isAboveLeftAvail ? 0 : 1;
      firstLineEndX   = isAboveAvail ? endX : 1;
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
        signa           = calcDiffRange(*colY, *colA, th);
        signb           = calcDiffRange(*colY, *colB, th);
        signa           = signa * 4 + signb;

        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int        bandc   = bandNumY - 4;
          const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          band               = (*colC * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          const Pel *colCT   = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remaining third component for bandIdx calc.*/
          int        bandc   = bandNumY - 6;
          band               = (*colCT * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
          signa           = calcDiffRange(*colY, *colA, th);
          signb           = calcDiffRange(*colY, *colB, th);
          signa           = signa * 4 + signb;

          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int        bandc   = bandNumY - 4;
            const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            band               = (*colC * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            const Pel *colCT   = (compID == COMPONENT_Cb)
                                   ? srcV + x
                                   : srcU + x; /* also use the remaining third component for bandIdx calc.*/
            int        bandc   = bandNumY - 6;
            band               = (*colCT * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
        signa           = calcDiffRange(*colY, *colA, th);
        signb           = calcDiffRange(*colY, *colB, th);
        signa           = signa * 4 + signb;

        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int        bandc   = bandNumY - 4;
          const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          band               = (*colC * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          const Pel *colCT   = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remaining third component for bandIdx calc.*/
          int        bandc   = bandNumY - 6;
          band               = (*colCT * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
    }
    break;
    case SAO_TYPE_EO_45:
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
        signa           = calcDiffRange(*colY, *colA, th);
        signb           = calcDiffRange(*colY, *colB, th);
        signa           = signa * 4 + signb;

        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int        bandc   = bandNumY - 4;
          const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          band               = (*colC * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          const Pel *colCT   = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remaining third component for bandIdx calc.*/
          int        bandc   = bandNumY - 6;
          band               = (*colCT * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;
          const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
          const Pel *colE = col[edgeCmp];
          const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
          const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

          const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
          const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
          const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          
          const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
#else
          const Pel *colY = srcY + (x << chromaScaleX);
          const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
          const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
          signa           = calcDiffRange(*colY, *colA, th);
          signb           = calcDiffRange(*colY, *colB, th);
          signa           = signa * 4 + signb;

          if (bandNumY <= 4)
          {
            band               = (*colY * bandNumY) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else if (bandNumY > 4 && bandNumY <= 6)
          {
            int        bandc   = bandNumY - 4;
            const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
            band               = (*colC * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
          else
          {
            const Pel *colCT   = (compID == COMPONENT_Cb)
                                   ? srcV + x
                                   : srcU + x; /* also use the remaining third component for bandIdx calc.*/
            int        bandc   = bandNumY - 6;
            band               = (*colCT * bandc) >> bitDepth;
            band               = band * CCSAO_EDGE_NUM + signa;
            const int classIdx = band;
            dst[x]             = dst[x] + offset[classIdx];
          }
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
#else
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
        signa           = calcDiffRange(*colY, *colA, th);
        signb           = calcDiffRange(*colY, *colB, th);
        signa           = signa * 4 + signb;

        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int        bandc   = bandNumY - 4;
          const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          band               = (*colC * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          const Pel *colCT   = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remaining third component for bandIdx calc.*/
          int        bandc   = bandNumY - 6;
          band               = (*colCT * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }
    }
    break;
    }
    break;
  }
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }

#else
  switch (compID)
  {
  case COMPONENT_Y:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + x;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
        dst[x] = dst[x] + offset[classIdx];
#else
        const Pel *colY = srcY + x;
        const Pel *colA = srcY + x + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + x + srcStrideY * candPosYYB + candPosYXB;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        signa = calcDiffRange(*colY, *colA, th);
        signb = calcDiffRange(*colY, *colB, th);

        signa = signa * 4 + signb;
        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int bandc          = bandNumY - 4;
          band               = (*colU * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          int bandc          = bandNumY - 6;
          band               = (*colV * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }

      srcY += srcStrideY;
      srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
      srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
      dst += dstStride;
    }
  }
  break;
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;
        const Pel *col[MAX_NUM_COMPONENT] = { colY, colU, colV };
        const Pel *colE = col[edgeCmp];
        const Pel *colA = colE + srcStrideE * edgePosYA + edgePosXA;
        const Pel *colB = colE + srcStrideE * edgePosYB + edgePosXB;

        const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
        const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
        
        const int classIdx = bandIdx * edgeNum + edgeIdx;
        dst[x] = dst[x] + offset[classIdx];
#else
        const Pel *colY = srcY + (x << chromaScaleX);
        const Pel *colA = srcY + (x << chromaScaleX) + srcStrideY * candPosYYA + candPosYXA;
        const Pel *colB = srcY + (x << chromaScaleX) + srcStrideY * candPosYYB + candPosYXB;
        signa           = calcDiffRange(*colY, *colA, th);
        signb           = calcDiffRange(*colY, *colB, th);
        signa           = signa * 4 + signb;

        if (bandNumY <= 4)
        {
          band               = (*colY * bandNumY) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else if (bandNumY > 4 && bandNumY <= 6)
        {
          int        bandc   = bandNumY - 4;
          const Pel *colC    = (compID == COMPONENT_Cb) ? srcU + x : srcV + x;
          band               = (*colC * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
        else
        {
          const Pel *colCT   = (compID == COMPONENT_Cb)
                                 ? srcV + x
                                 : srcU + x; /* also use the remaining third component for bandIdx calc.*/
          int        bandc   = bandNumY - 6;
          band               = (*colCT * bandc) >> bitDepth;
          band               = band * CCSAO_EDGE_NUM + signa;
          const int classIdx = band;
          dst[x]             = dst[x] + offset[classIdx];
        }
#endif
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#endif
}
#endif
void SampleAdaptiveOffset::offsetBlockCcSaoNoClip(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth, const ClpRng& clpRng
                                                , const uint16_t candPosY
                                                , const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
                                                , const short* offset
                                                , const Pel* srcY, const Pel* srcU, const Pel* srcV, Pel* dst
                                                , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int dstStride
                                                , const int width, const int height
                                                , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                                , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                                                 )
{
  const int candPosYX = g_ccSaoCandPosX[COMPONENT_Y][candPosY];
  const int candPosYY = g_ccSaoCandPosY[COMPONENT_Y][candPosY];

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleYM1 = 1 - chromaScaleY;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int x, y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;

  switch (compID)
  {
  case COMPONENT_Y:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
            dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
            dst[x] = dst[x] + offset[classIdx];
#endif
          }
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          dst += dstStride;
        }
      }
      break;
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }

        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }

      srcY += srcStrideY;
      srcU += srcStrideU * chromaScaleYM1;
      srcV += srcStrideV * chromaScaleYM1;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }

        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;

      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }
      break;
    }
    }
    break;
  }
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + x;
            const Pel *colV = srcV + x;

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
            dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
            dst[x] = dst[x] + offset[classIdx];
#endif
          }
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;
        }
      }
      break;
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }

        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }

        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;

      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
          dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
          dst[x] = dst[x] + offset[classIdx];
#endif
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
#if JVET_AJ0237_INTERNAL_12BIT
        dst[x] = dst[x] + (offset[classIdx] << m_offsetStepLog2[compID]);
#else
        dst[x] = dst[x] + offset[classIdx];
#endif
      }
    }
    break;
    }
    break;
  }
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }

#else
  switch (compID)
  {
  case COMPONENT_Y:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY   = (*colY * bandNumY) >> bitDepth;
        const int bandU   = (*colU * bandNumU) >> bitDepth;
        const int bandV   = (*colV * bandNumV) >> bitDepth;
        const int bandIdx = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
        dst[x] = dst[x] + offset[classIdx];
      }

      srcY += srcStrideY;
      srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
      srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
      dst += dstStride;
    }
  }
  break;
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {

        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY   = (*colY * bandNumY) >> bitDepth;
        const int bandU   = (*colU * bandNumU) >> bitDepth;
        const int bandV   = (*colV * bandNumV) >> bitDepth;
        const int bandIdx = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
        dst[x] = dst[x] + offset[classIdx];
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#endif
}

#if !JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
void SampleAdaptiveOffset::offsetBlockCcSao(const ComponentID compID, const ChromaFormat chromaFormat, const int bitDepth, const ClpRng& clpRng
                                          , const uint16_t candPosY
                                          , const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV
                                          , const short* offset
                                          , const Pel* srcY, const Pel* srcU, const Pel* srcV, Pel* dst
                                          , const int srcStrideY, const int srcStrideU, const int srcStrideV, const int dstStride
                                          , const int width, const int height
                                          , bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                          , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
#endif
                                           )
{
  const int candPosYX = g_ccSaoCandPosX[COMPONENT_Y][candPosY];
  const int candPosYY = g_ccSaoCandPosY[COMPONENT_Y][candPosY];

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormat );
  const int chromaScaleYM1 = 1 - chromaScaleY;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int x, y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  switch (compID)
  {
  case COMPONENT_Y:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + (x >> chromaScaleX);
            const Pel *colV = srcV + (x >> chromaScaleX);

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);            
          }
          srcY += srcStrideY;
          srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
          srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
          dst += dstStride;
        }
      }
      break;
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }

        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);        
      }

      srcY += srcStrideY;
      srcU += srcStrideU * chromaScaleYM1;
      srcV += srcStrideV * chromaScaleYM1;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }

        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;

      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);        
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + (x >> chromaScaleX);
          const Pel *colV = srcV + (x >> chromaScaleX);

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY;
        srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
        srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);        
      }
      break;
    }
    }
    break;
  }
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    switch (candPosY)
    {
    case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
              // information
      /* total 9 cases will come up here
      for (-1,-1) just use the top and middle lines and check for vb */
      {
        startX = isLeftAvail ? 0 : 1;
        endX   = width;
        // 1st line
        firstLineStartX = isAboveLeftAvail ? 0 : 1;
        firstLineEndX   = isAboveAvail ? endX : 1;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;

        // middle lines
        for (y = 1; y < height; y++)
        {
          for (x = startX; x < endX; x++)
          {
            if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
            {
              continue;
            }
            const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
            const Pel *colU = srcU + x;
            const Pel *colV = srcV + x;

            const int bandY    = (*colY * bandNumY) >> bitDepth;
            const int bandU    = (*colU * bandNumU) >> bitDepth;
            const int bandV    = (*colV * bandNumV) >> bitDepth;
            const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
            const int classIdx = bandIdx;

            dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);            
          }
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;
        }
      }
      break;
    case 1: /*(0, -1)  top sample */
    {
      startY = isAboveAvail ? 0 : 1;
      endY   = height;
      if (!isAboveAvail)
      {
        srcY += srcStrideY;
        srcU += srcStrideU * chromaScaleYM1;
        srcV += srcStrideV * chromaScaleYM1;
        dst += dstStride;
      }
      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }

        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 2: /*(0, -1)  top right sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;
      // first line
      firstLineStartX = isAboveAvail ? startX : (width - 1);
      firstLineEndX   = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, 0, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);        
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;

      // middle lines
      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }

        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 3: /*(-1, 0)  left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 4: /*(0, 0)  current sample */
    {       /* when current sample is choosen there is no more dependency on neighbor samples*/

      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 5: /*(1, 0)  right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, 0, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 6: /*(-1, 1)  below left sample */
    {
      startX = isLeftAvail ? 0 : 1;
      endX   = width;

      for (y = 1; y < height; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;

      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);        
      }
      break;
    }
    case 7: /*(0, 1)  below sample */
    {
      startY = 0;
      endY   = isBelowAvail ? height : height - 1;

      for (y = startY; y < endY; y++)
      {
        for (x = 0; x < width; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, 0, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }
      break;
    }
    case 8: /*(1, 1)  below right sample */
    {
      startX = 0;
      endX   = isRightAvail ? width : (width - 1);

      for (y = 0; y < height - 1; y++)
      {
        for (x = startX; x < endX; x++)
        {
          if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, y, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
          {
            continue;
          }
          const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
          const Pel *colU = srcU + x;
          const Pel *colV = srcV + x;

          const int bandY    = (*colY * bandNumY) >> bitDepth;
          const int bandU    = (*colU * bandNumU) >> bitDepth;
          const int bandV    = (*colV * bandNumV) >> bitDepth;
          const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
          const int classIdx = bandIdx;

          dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);          
        }
        srcY += srcStrideY << chromaScaleY;
        srcU += srcStrideU;
        srcV += srcStrideV;
        dst += dstStride;
      }

      // last line
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        if (isCtuCrossedByVirtualBoundaries && isProcessDisabled(x, height - 1, numVerVirBndry, numHorVirBndry, verVirBndryPos, horVirBndryPos))
        {
          continue;
        }
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY    = (*colY * bandNumY) >> bitDepth;
        const int bandU    = (*colU * bandNumU) >> bitDepth;
        const int bandV    = (*colV * bandNumV) >> bitDepth;
        const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
       
      }
    }
    break;
    }
    break;
  }
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }

#else
  switch (compID)
  {
  case COMPONENT_Y:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + (x >> chromaScaleX);
        const Pel *colV = srcV + (x >> chromaScaleX);

        const int bandY = (*colY * bandNumY) >> bitDepth;
        const int bandU = (*colU * bandNumU) >> bitDepth;
        const int bandV = (*colV * bandNumV) >> bitDepth;
        const int bandIdx = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
      }

      srcY += srcStrideY;
      srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
      srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
      dst += dstStride;
    }
  }
  break;
  case COMPONENT_Cb:
  case COMPONENT_Cr:
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
        const Pel *colU = srcU + x;
        const Pel *colV = srcV + x;

        const int bandY = (*colY * bandNumY) >> bitDepth;
        const int bandU = (*colU * bandNumU) >> bitDepth;
        const int bandV = (*colV * bandNumV) >> bitDepth;
        const int bandIdx = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
        const int classIdx = bandIdx;

        dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
      }

      srcY += srcStrideY << chromaScaleY;
      srcU += srcStrideU;
      srcV += srcStrideV;
      dst += dstStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported CCSAO compID\n");
  }
  }
#endif  
}
#endif
#endif

void SampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position &pos,
  bool& isLeftAvail,
  bool& isRightAvail,
  bool& isAboveAvail,
  bool& isBelowAvail,
  bool& isAboveLeftAvail,
  bool& isAboveRightAvail,
  bool& isBelowLeftAvail,
  bool& isBelowRightAvail
  ) const
{
  const int width = cs.pcv->maxCUWidth;
  const int height = cs.pcv->maxCUHeight;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-width, 0), CH_L);
  const CodingUnit* cuRight = cs.getCU(pos.offset(width, 0), CH_L);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -height), CH_L);
  const CodingUnit* cuBelow = cs.getCU(pos.offset(0, height), CH_L);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-width, -height), CH_L);
  const CodingUnit* cuAboveRight = cs.getCU(pos.offset(width, -height), CH_L);
  const CodingUnit* cuBelowLeft = cs.getCU(pos.offset(-width, height), CH_L);
  const CodingUnit* cuBelowRight = cs.getCU(pos.offset(width, height), CH_L);

  // check cross slice flags
  const bool isLoopFilterAcrossSlicePPS = cs.pps->getLoopFilterAcrossSlicesEnabledFlag();
  if (!isLoopFilterAcrossSlicePPS)
  {
    isLeftAvail       = (cuLeft == NULL)       ? false : CU::isSameSlice(*cuCurr, *cuLeft);
    isAboveAvail      = (cuAbove == NULL)      ? false : CU::isSameSlice(*cuCurr, *cuAbove);
    isRightAvail      = (cuRight == NULL)      ? false : CU::isSameSlice(*cuCurr, *cuRight);
    isBelowAvail      = (cuBelow == NULL)      ? false : CU::isSameSlice(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (cuAboveLeft == NULL)  ? false : CU::isSameSlice(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (cuAboveRight == NULL) ? false : CU::isSameSlice(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (cuBelowLeft == NULL)  ? false : CU::isSameSlice(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (cuBelowRight == NULL) ? false : CU::isSameSlice(*cuCurr, *cuBelowRight);
  }
  else
  {
    isLeftAvail       = (cuLeft != NULL);
    isAboveAvail      = (cuAbove != NULL);
    isRightAvail      = (cuRight != NULL);
    isBelowAvail      = (cuBelow != NULL);
    isAboveLeftAvail  = (cuAboveLeft != NULL);
    isAboveRightAvail = (cuAboveRight != NULL);
    isBelowLeftAvail  = (cuBelowLeft != NULL);
    isBelowRightAvail = (cuBelowRight != NULL);
  }

  // check cross tile flags
  const bool isLoopFilterAcrossTilePPS = cs.pps->getLoopFilterAcrossTilesEnabledFlag();
  if (!isLoopFilterAcrossTilePPS)
  {
    isLeftAvail       = (!isLeftAvail)       ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail      = (!isAboveAvail)      ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isRightAvail      = (!isRightAvail)      ? false : CU::isSameTile(*cuCurr, *cuRight);
    isBelowAvail      = (!isBelowAvail)      ? false : CU::isSameTile(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (!isAboveLeftAvail)  ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (!isAboveRightAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (!isBelowLeftAvail)  ? false : CU::isSameTile(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (!isBelowRightAvail) ? false : CU::isSameTile(*cuCurr, *cuBelowRight);
  }

  // check cross subpic flags
  const SubPic& curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
  if (!curSubPic.getloopFilterAcrossEnabledFlag())
  {
    isLeftAvail       = (!isLeftAvail)       ? false : CU::isSameSubPic(*cuCurr, *cuLeft);
    isAboveAvail      = (!isAboveAvail)      ? false : CU::isSameSubPic(*cuCurr, *cuAbove);
    isRightAvail      = (!isRightAvail)      ? false : CU::isSameSubPic(*cuCurr, *cuRight);
    isBelowAvail      = (!isBelowAvail)      ? false : CU::isSameSubPic(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (!isAboveLeftAvail)  ? false : CU::isSameSubPic(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (!isAboveRightAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (!isBelowLeftAvail)  ? false : CU::isSameSubPic(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (!isBelowRightAvail) ? false : CU::isSameSubPic(*cuCurr, *cuBelowRight);
  }
}

bool SampleAdaptiveOffset::isCrossedByVirtualBoundaries(const int xPos, const int yPos, const int width, const int height, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], const PicHeader* picHeader )
{
  numHorVirBndry = 0; numVerVirBndry = 0;
  if( picHeader->getVirtualBoundariesPresentFlag() )
  {
    for (int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++)
    {
     if (yPos <= picHeader->getVirtualBoundariesPosY(i) && picHeader->getVirtualBoundariesPosY(i) <= yPos + height)
      {
        horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY(i);
      }
    }
    for (int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++)
    {
      if (xPos <= picHeader->getVirtualBoundariesPosX(i) && picHeader->getVirtualBoundariesPosX(i) <= xPos + width)
      {
        verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX(i);
      }
    }
  }
  return numHorVirBndry > 0 || numVerVirBndry > 0;
}
//! \}
