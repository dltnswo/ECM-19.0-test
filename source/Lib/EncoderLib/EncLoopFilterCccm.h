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

#include "CommonLib/LoopFilterCccm.h"

#if JVET_AL0153_ALF_CCCM
#ifndef __ENCLOOPFILTERCCCM__
#define __ENCLOOPFILTERCCCM__

#include "CABACWriter.h"
#include "EncCfg.h"

class EncLoopFilterCccm : public LoopFilterCccm
{
public:
  void lfCccmRDO(CodingStructure& cs, const PelUnitBuf recSAO
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    , PelUnitBuf recYuv
#endif
    , CtxCache* ctxCache, CABACEncoder* cabacEncoder, Slice* pcSlice);
protected:
  CtxCache *m_ctxCache;
  CABACWriter* m_CABACEstimator;
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  std::vector<std::vector<PelStorage>> m_lfCccmOutputsEncoder;
#endif
  void lfCccmAllocateArraysEncoder(const int maxCUWidth, const int maxCUHeight)
  {
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    m_lfCccmOutputsEncoder.resize(m_lfCccmMaxNumModels);
    for(int i=0;i<m_lfCccmMaxNumModels;i++)
    {
      m_lfCccmOutputsEncoder.at(i).resize(m_lfCccmMaxNumWindows);
    }
    const Area ctuArea(0, 0, maxCUWidth, maxCUHeight);
    for(int modelTypeIndx = 0; modelTypeIndx < m_lfCccmMaxNumModels; modelTypeIndx++)
    {
      for(int windowSizeIndx = 0; windowSizeIndx < m_lfCccmMaxNumWindows; windowSizeIndx++)
      {
        if(m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).bufs.empty())
        {
          m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).create(CHROMA_ONLY_420, ctuArea);
        }
      }
    }
    const int chromaArea = m_lfCccmOutputsEncoder.at(0).at(0).Cb().area();
#else
    const int chromaArea = (maxCUWidth >> 1) * (maxCUHeight >> 1);
#endif
    m_lfCccmXCorr.resize(chromaArea);
    m_lfCccmAutoCorr.resize(chromaArea);
    for(int i=0;i<chromaArea;i++)
    {
      m_lfCccmXCorr.at(i).resize(2*m_lfCccmArrayStride);
      m_lfCccmAutoCorr.at(i).resize(m_lfCccmArrayStride);
      for(int j=0;j<m_lfCccmArrayStride;j++)
      {
        m_lfCccmAutoCorr.at(i).at(j).resize(m_lfCccmArrayStride);
      }
    }
  }
  void lfCccmDeallocateArraysEncoder()
  {
    m_lfCccmXCorr.clear();
    m_lfCccmAutoCorr.clear();
#if !JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    m_lfCccmOutputsEncoder.clear();
#endif
  }
};
#endif
#endif
