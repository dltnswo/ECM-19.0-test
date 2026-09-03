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

#include "LoopFilterCccm.h"

#if JVET_AL0153_ALF_CCCM

#include "CodingStructure.h"
#include "Picture.h"
#include "IntraPrediction.h"
#include <array>
#include <cmath>
#include <numeric>

void LoopFilterCccm::lfCccmSetWindows( const int xi, const int yi )
{
  m_lfCccmWindowArea = clipArea(Area(Position(xi,yi), m_lfCccmWindowSize), m_lfCccmPartitionArea);
  m_lfCccmRegressionWindow = m_lfCccmWindowArea;
  m_lfCccmRegressionWindow.x = std::max( m_lfCccmPartitionArea.x, m_lfCccmRegressionWindow.x-1 );
  m_lfCccmRegressionWindow.y = std::max( m_lfCccmPartitionArea.y, m_lfCccmRegressionWindow.y-1 );
  m_lfCccmRegressionWindow.width += 2;
  m_lfCccmRegressionWindow.height += 2;
  m_lfCccmRegressionWindow = clipArea(m_lfCccmRegressionWindow, m_lfCccmPartitionArea);
#if JVET_AN0098_ALF_CCCM_REGULARIZATION
  m_lfCccmDoRegularization = m_lfCccmRegressionWindow.area() > 4096 ? false : true;
#endif
}

#if JVET_AN0098_ALF_CCCM_REGULARIZATION
void LoopFilterCccm::setLfCccmBadPointThreshold()
{
  m_lfCccmBadBlockThreshold = m_lfCccmBadPointThreshold[m_lfCccmDoRegularization ? m_lfCccmMultiModel + 1 : 0];
};
#endif
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
void LoopFilterCccm::lfCccmSetMultiModelParametersSR()
{
  const Pel* src = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmRegressionWindow.x, m_lfCccmRegressionWindow.y);
  const int strideY = m_lfCccmLumaSaoDownsampled.stride;
  const int starty = m_lfCccmRegressionWindow.y;
  const int startx = m_lfCccmRegressionWindow.x;
  const int endy = m_lfCccmRegressionWindow.bottomRight().getY() + 1;
  const int endx = m_lfCccmRegressionWindow.bottomRight().getX() + 1;
  int numSamples = 0;
  m_lfCccmThresholdSR = 0;
  for (int y = starty; y < endy; y++)
  {
    for (int x = startx, xx = 0; x < endx; x++, xx++)
    {
      if (m_lfCccmMultiModel == 1 && src[xx] >= m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModel == 2 && src[xx] < m_lfCccmThreshold)
      {
        continue;
      }
      m_lfCccmThresholdSR += src[xx];
      numSamples++;
    }
    src += strideY;
  }
  m_lfCccmThresholdSR = numSamples == 0 ? 0 : Pel((m_lfCccmThresholdSR + (numSamples >> 1)) / numSamples);
}
bool LoopFilterCccm::lfCccmUpdateStatus(int numSamples)
{
  m_lfCccmModelTypeTmp = m_lfCccmModelType;
  lfCccmResetArrays();
  if (numSamples >= 7)
  {
    m_lfCccmModelType = 0;
    m_lfCccmModelSize = 7;
  }
  else if (numSamples >= 3)
  {
    m_lfCccmModelType = 2;
    m_lfCccmModelSize = 3;
  }
  else
  {
    return false;
  }
  if (m_lfCccmIsEncoder)
  {
    m_lfCccmMmap = m_lfCccmModelMap[m_lfCccmModelType];
    for (int i = 0; i < 11; i++)
    {
      m_lfCccmMmap2[i] = m_lfCccmMmap[i] + m_lfCccmArrayStride;
    }
  }
  m_lfCccmModelCb = CccmModel(m_lfCccmModelSize, m_lfCccmChromaBitDepth);
  m_lfCccmModelCr = CccmModel(m_lfCccmModelSize, m_lfCccmChromaBitDepth);
  m_lfCccmSamples[m_lfCccmModelSize - 1] = m_lfCccmLumaOffset;
  return true;
}
void LoopFilterCccm::lfCccmRecoverStatus()
{
  m_lfCccmModelType = m_lfCccmModelTypeTmp;
  switch (m_lfCccmModelType)
  {
  case 0:
    m_lfCccmModelSize = CCCM_NUM_PARAMS;
    break;
  case 1:
    m_lfCccmModelSize = 2;
    break;
  case 2:
    m_lfCccmModelSize = 3;
    break;
  case 3:
    m_lfCccmModelSize = 6;
    break;
  case 4:
    m_lfCccmModelSize = 5;
    break;
  case 5:
    m_lfCccmModelSize = 5;
    break;
  case 6:
    m_lfCccmModelSize = 11;
    break;
  case 7:
    m_lfCccmModelSize = 10;
    break;
  default:
    THROW("modeltype not supported");
    break;
  }
  if (m_lfCccmIsEncoder)
  {
    m_lfCccmMmap = m_lfCccmModelMap[m_lfCccmModelType];
    for (int i = 0; i < 11; i++)
    {
      m_lfCccmMmap2[i] = m_lfCccmMmap[i] + m_lfCccmArrayStride;
    }
  }
  m_lfCccmModelCb = CccmModel(m_lfCccmModelSize, m_lfCccmChromaBitDepth);
  m_lfCccmModelCr = CccmModel(m_lfCccmModelSize, m_lfCccmChromaBitDepth);
  m_lfCccmSamples[m_lfCccmModelSize - 1] = m_lfCccmLumaOffset;
}
#endif
void LoopFilterCccm::lfCccmSumStatisticsEncoder(int &numSamples, const Pel *src00, const Pel *srcCb, const Pel *srcCr, const int strideY0, const int strideCb, const int strideCr)
{
  m_lfCccmChromaOffsetCb = 0;
  m_lfCccmChromaOffsetCr = 0;
  const int starty = m_lfCccmRegressionWindow.y;
  const int startyy = m_lfCccmPartitionArea.width*(starty-m_lfCccmPartitionArea.topLeft().y);
  const int startx = m_lfCccmRegressionWindow.x;
  const int startxxx = (startx-m_lfCccmPartitionArea.topLeft().x);
  const int endy = m_lfCccmRegressionWindow.bottomRight().getY()+1;
  const int endx = m_lfCccmRegressionWindow.bottomRight().getX()+1;
  const int pstride = m_lfCccmPartitionArea.width;
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  const Pel* src0 = src00;
#endif
  for (int y = starty, yy = startyy; y < endy; y++, yy += pstride)
  {
    for (int x = startx, xx = 0, xxx = startxxx; x < endx; x++, xx++, xxx++)
    {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      if (m_lfCccmMultiModel == 1 && src0[xx] >= m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModel == 2 && src0[xx] < m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 1 && src0[xx] >= m_lfCccmThresholdSR)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 2 && src0[xx] < m_lfCccmThresholdSR)
      {
        continue;
      }
#else
      if(m_lfCccmMultiModel == 1 && src00[xx] >= m_lfCccmThreshold)
      {
        continue;
      }
      if(m_lfCccmMultiModel == 2 && src00[xx] < m_lfCccmThreshold)
      {
        continue;
      }
#endif
      m_lfCccmChromaOffsetCb += srcCb[xx];
      m_lfCccmChromaOffsetCr += srcCr[xx];
      const int linIdx2 = xxx + yy;
      int32_t *gxlocal = m_lfCccmXCorr[linIdx2].data();
      for( int coli0 = 0; coli0 < m_lfCccmModelSize; coli0++ )
      {
        m_lfCccmATCb[coli0] += gxlocal[m_lfCccmMmap[coli0]];
        m_lfCccmATCr[coli0] += gxlocal[m_lfCccmMmap2[coli0]];
        int32_t *galocal = m_lfCccmAutoCorr[linIdx2][m_lfCccmMmap[coli0]].data();
        for( int coli1 = coli0; coli1 < m_lfCccmModelSize; coli1++ )
        {
          m_lfCccmATA[coli0][coli1] += galocal[m_lfCccmMmap[coli1]];
        }
      }
      numSamples++;
    }
    srcCb += strideCb;
    srcCr += strideCr;
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
    src0 += strideY0;
#else
    src00 += strideY0;
#endif
  }
  if(numSamples < m_lfCccmModelSize)
  {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
    m_lfCccmSampleLessThanModelSizeFlag = true;
    bool isUpdated = lfCccmUpdateStatus(numSamples);
    if (!isUpdated)
    {
      return;
    }
    for (int y = starty, yy = startyy; y < endy; y++, yy += pstride)
    {
      for (int x = startx, xx = 0, xxx = startxxx; x < endx; x++, xx++, xxx++)
      {
        if (m_lfCccmMultiModel == 1 && src00[xx] >= m_lfCccmThreshold)
        {
          continue;
        }
        if (m_lfCccmMultiModel == 2 && src00[xx] < m_lfCccmThreshold)
        {
          continue;
        }
        if (m_lfCccmMultiModelSR == 1 && src00[xx] >= m_lfCccmThresholdSR)
        {
          continue;
        }
        if (m_lfCccmMultiModelSR == 2 && src00[xx] < m_lfCccmThresholdSR)
        {
          continue;
        }
        const int linIdx2 = xxx + yy;
        int32_t* gxlocal = m_lfCccmXCorr[linIdx2].data();
        for (int coli0 = 0; coli0 < m_lfCccmModelSize; coli0++)
        {
          m_lfCccmATCb[coli0] += gxlocal[m_lfCccmMmap[coli0]];
          m_lfCccmATCr[coli0] += gxlocal[m_lfCccmMmap2[coli0]];
          int32_t* galocal = m_lfCccmAutoCorr[linIdx2][m_lfCccmMmap[coli0]].data();
          for (int coli1 = coli0; coli1 < m_lfCccmModelSize; coli1++)
          {
            m_lfCccmATA[coli0][coli1] += galocal[m_lfCccmMmap[coli1]];
          }
        }
      }
      src00 += strideY0;
    }
#else
    m_lfCccmBadWindow = true;
    return;
#endif
  }
  m_lfCccmChromaOffsetCb = (m_lfCccmChromaOffsetCb+numSamples/2)/numSamples;
  m_lfCccmChromaOffsetCr = (m_lfCccmChromaOffsetCr+numSamples/2)/numSamples;
}
void LoopFilterCccm::lfCccmSumStatisticsDecoder(int &numSamples, const Pel* src0, const Pel *src1, const Pel *src2, const Pel *srcCb, const Pel *srcCr, const int stride, const int strideCb, const int strideCr)
{
  m_lfCccmChromaOffsetCb = 0;
  m_lfCccmChromaOffsetCr = 0;

  const int starty = m_lfCccmRegressionWindow.y;
  const int startx = m_lfCccmRegressionWindow.x;
  const int endy = m_lfCccmRegressionWindow.bottomRight().getY()+1;
  const int endx = m_lfCccmRegressionWindow.bottomRight().getX()+1;
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  const Pel* src00 = src0;
  const Pel* src01 = src1;
  const Pel* src02 = src2;
  for (int y = starty; y < endy; y++, srcCb += strideCb, srcCr += strideCr, src00 += stride, src01 += stride, src02 += stride)
  {
#else
  const int nx = endx - startx;
  for (int y = starty; y < endy; y++, srcCb += strideCb, srcCr += strideCr, src0 += stride, src1 += stride, src2 += stride)
  {
    if(m_lfCccmMultiModel == 0)
    {
      std::memcpy(m_lfCccmYCb + numSamples, srcCb, nx*sizeof(Pel));
      std::memcpy(m_lfCccmYCr + numSamples, srcCr, nx*sizeof(Pel));
      lfCccmCollectData2(src0, src1, src2, numSamples, nx);
      numSamples += nx;
      continue;
    }
#endif
    for (int x = startx, xx = 0; x < endx; x++, xx++)
    {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      if (m_lfCccmMultiModel == 1 && src00[xx] >= m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModel == 2 && src00[xx] < m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 1 && src00[xx] >= m_lfCccmThresholdSR)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 2 && src00[xx] < m_lfCccmThresholdSR)
      {
        continue;
      }
      lfCccmCollectData(&src00[xx], &src01[xx], &src02[xx], numSamples);
#else
      if(m_lfCccmMultiModel == 1 && src0[xx] >= m_lfCccmThreshold)
      {
        continue;
      }
      if(m_lfCccmMultiModel == 2 && src0[xx] < m_lfCccmThreshold)
      {
        continue;
      }
      lfCccmCollectData(&src0[xx], &src1[xx], &src2[xx], numSamples);
#endif
      m_lfCccmChromaOffsetCb += srcCb[xx];
      m_lfCccmChromaOffsetCr += srcCr[xx];

      m_lfCccmYCb[numSamples] = srcCb[xx];
      m_lfCccmYCr[numSamples] = srcCr[xx];
      numSamples++;
    }
  }
#if !JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  if(m_lfCccmMultiModel == 0)
  {
    m_lfCccmChromaOffsetCb = std::reduce(m_lfCccmYCb, m_lfCccmYCb + numSamples, Pel{0});
    m_lfCccmChromaOffsetCr = std::reduce(m_lfCccmYCr, m_lfCccmYCr + numSamples, Pel{0});
    std::fill(m_lfCccmA[m_lfCccmModelSize-1], m_lfCccmA[m_lfCccmModelSize-1] + numSamples, m_lfCccmLumaOffset);
    int nonlinearIdx = -1;
    switch( m_lfCccmModelType )
    {
      case 0:
        nonlinearIdx = 5;
        break;
      case 2:
        nonlinearIdx = 1;
        break;
      case 4:
        nonlinearIdx = 3;
        break;
      case 5:
        nonlinearIdx = 3;
        break;
      case 6:
        nonlinearIdx = 9;
        break;
    }
    if( nonlinearIdx > -1 )
    {
      for(int i=0; i < numSamples; i++)
      {
        m_lfCccmA[nonlinearIdx][i] = m_lfCccmModelCb.nonlinear(m_lfCccmA[0][i]);
      }
    }
  }
#endif
  if(numSamples < m_lfCccmModelSize)
  {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
    m_lfCccmSampleLessThanModelSizeFlag = true;
    bool isUpdated = lfCccmUpdateStatus(numSamples);
    if (!isUpdated)
    {
      return;
    }
    numSamples = 0;
    for (int y = starty; y < endy; y++, src0 += stride, src1 += stride, src2 += stride)
    {
      for (int x = startx, xx = 0; x < endx; x++, xx++)
      {
        if (m_lfCccmMultiModel == 1 && src0[xx] >= m_lfCccmThreshold)
        {
          continue;
        }
        if (m_lfCccmMultiModel == 2 && src0[xx] < m_lfCccmThreshold)
        {
          continue;
        }
        if (m_lfCccmMultiModelSR == 1 && src0[xx] >= m_lfCccmThresholdSR)
        {
          continue;
        }
        if (m_lfCccmMultiModelSR == 2 && src0[xx] < m_lfCccmThresholdSR)
        {
          continue;
        }
        lfCccmCollectData(&src0[xx], &src1[xx], &src2[xx], numSamples);
        numSamples++;
      }
    }
#else
    m_lfCccmBadWindow = true;
    return;
#endif
  }

  m_lfCccmChromaOffsetCb = (m_lfCccmChromaOffsetCb+numSamples/2)/numSamples;
  m_lfCccmChromaOffsetCr = (m_lfCccmChromaOffsetCr+numSamples/2)/numSamples;

  constexpr int log2BatchSize = 10;
  constexpr int batchSize = (1 << log2BatchSize);
  const int samplesInBatches = (numSamples >> log2BatchSize) << log2BatchSize;
  for (int coli0 = 0; coli0 < m_lfCccmModelSize; coli0++)
  {
    for (int coli1 = coli0; coli1 < m_lfCccmModelSize; coli1++)
    {
      for (int offset = 0; offset < samplesInBatches; offset += batchSize)
      {
        m_lfCccmATA[coli0][coli1] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli0][offset], &m_lfCccmA[coli1][offset], batchSize);
      }
      m_lfCccmATA[coli0][coli1] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli0][samplesInBatches], &m_lfCccmA[coli1][samplesInBatches], numSamples - samplesInBatches);
    }
  }
  for (int coli = 0; coli < m_lfCccmModelSize; coli++)
  {
    for (int offset = 0; offset < samplesInBatches; offset += batchSize)
    {
      m_lfCccmATCb[coli] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli][offset], &m_lfCccmYCb[offset], batchSize);
      m_lfCccmATCr[coli] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli][offset], &m_lfCccmYCr[offset], batchSize);
    }
    m_lfCccmATCb[coli] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli][samplesInBatches], &m_lfCccmYCb[samplesInBatches], numSamples - samplesInBatches);
    m_lfCccmATCr[coli] += m_lfCccmIntraPred->m_calcAeipGroupSum(&m_lfCccmA[coli][samplesInBatches], &m_lfCccmYCr[samplesInBatches], numSamples - samplesInBatches);
  }
}
void LoopFilterCccm::lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  const int impMode
#endif
)
{
#if !JVET_AM0063_ALF_CCCM_UPDATED_MULTI_MODELS_STRATEGY
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  if ((m_lfCccmBadWindow && m_lfCccmIsFR) || (m_lfCccmBadWindowSR && !m_lfCccmIsFR))
#else
  if (m_lfCccmBadWindow)
#endif
  {
    return;
  }
#endif
  lfCccmResetArrays();

  int numSamples = 0;

  const Pel *src00 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmRegressionWindow.x,m_lfCccmRegressionWindow.y);
  const Pel *srcCb = m_lfCccmSaoCb.bufAt(m_lfCccmRegressionWindow.x,m_lfCccmRegressionWindow.y);
  const Pel *srcCr = m_lfCccmSaoCr.bufAt(m_lfCccmRegressionWindow.x,m_lfCccmRegressionWindow.y);

  const int strideY0 = m_lfCccmLumaSaoDownsampled.stride;
  const int strideCb = m_lfCccmSaoCb.stride;
  const int strideCr = m_lfCccmSaoCr.stride;

#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  m_lfCccmSampleLessThanModelSizeFlag = false;
#endif

  if(m_lfCccmIsEncoder)
  {
    lfCccmSumStatisticsEncoder(numSamples, src00, srcCb, srcCr, strideY0, strideCb, strideCr);
  }
  else
  {
    const Pel *src1 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmRegressionWindow.x,m_lfCccmRegressionWindow.y-1);
    const Pel *src2 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmRegressionWindow.x,m_lfCccmRegressionWindow.y+1);
    lfCccmSumStatisticsDecoder(numSamples, src00, src1, src2, srcCb, srcCr, strideY0, strideCb, strideCr);
  }

  if(numSamples < m_lfCccmModelSize)
  {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
    if (m_lfCccmIsFR)
    {
      m_lfCccmBadWindow = true;
    }
    else
    {
      m_lfCccmBadWindowSR = true;
    }
#else
    m_lfCccmBadWindow = true;
#endif
    return;
  }

#if JVET_AN0098_ALF_CCCM_REGULARIZATION
  m_lfCccmSolver.solve3(m_lfCccmATA, m_lfCccmATCb, m_lfCccmATCr, numSamples, m_lfCccmChromaOffsetCb, m_lfCccmChromaOffsetCr, m_lfCccmModelCb, m_lfCccmModelCr, true, m_lfCccmHasNonLinearTerm, m_lfCccmDoRegularization);
#else
  m_lfCccmSolver.solve3(m_lfCccmATA, m_lfCccmATCb, m_lfCccmATCr, numSamples, m_lfCccmChromaOffsetCb, m_lfCccmChromaOffsetCr, m_lfCccmModelCb, m_lfCccmModelCr, true);
#endif

  if(!m_lfCccmModelCb.valid() || !m_lfCccmModelCr.valid())
  {
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
    if (m_lfCccmSampleLessThanModelSizeFlag)
    {
      lfCccmRecoverStatus();
    }
    if (m_lfCccmIsFR)
    {
      m_lfCccmBadWindow = true;
    }
    else
    {
      m_lfCccmBadWindowSR = true;
    }
#else
    m_lfCccmBadWindow = true;
#endif
    return;
  }

  Pel *uCb = m_lfCccmUpdateCb.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y);
  Pel *uCr = m_lfCccmUpdateCr.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y);

  const Pel *recCb = m_lfCccmSaoCb.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y);
  const Pel *recCr = m_lfCccmSaoCr.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y);

  const Pel *src0 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y);
  const Pel *src1 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y-1);
  const Pel *src2 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmWindowArea.x,m_lfCccmWindowArea.y+1);

  const int strideY = m_lfCccmLumaSaoDownsampled.stride;
  const int strideUCb = m_lfCccmUpdateCb.stride;
  const int strideUCr = m_lfCccmUpdateCr.stride;

#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  Pel* uImpCb[4] = { nullptr };
  Pel* uImpCr[4] = { nullptr };
  int strideUImpCb[4] = { 0 };
  int strideUImpCr[4] = { 0 };
  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    uImpCb[factorIdx] = m_lfCccmUpdateImpCb[factorIdx].bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
    strideUImpCb[factorIdx] = m_lfCccmUpdateImpCb[factorIdx].stride;
    uImpCr[factorIdx] = m_lfCccmUpdateImpCr[factorIdx].bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
    strideUImpCr[factorIdx] = m_lfCccmUpdateImpCr[factorIdx].stride;
  }
#endif

#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  bool skipLoop = false;
  for (int y = 0; (y < m_lfCccmWindowArea.height) && !skipLoop; y++)
  {
    for (int x = 0; (x < m_lfCccmWindowArea.width) && !skipLoop; x++)
#else
  for (int y = 0; (y < m_lfCccmWindowArea.height) && !m_lfCccmBadWindow; y++)
  {
    for (int x = 0; (x < m_lfCccmWindowArea.width) && !m_lfCccmBadWindow; x++)
#endif
    {
      const Pel curCb = recCb[x];
      const Pel curCr = recCr[x];

      if(m_lfCccmMultiModel == 1 && src0[x] >= m_lfCccmThreshold)
      {
        continue;
      }
      if(m_lfCccmMultiModel == 2 && src0[x] < m_lfCccmThreshold)
      {
        continue;
      }
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      if (m_lfCccmMultiModelSR == 1 && src0[x] >= m_lfCccmThresholdSR)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 2 && src0[x] < m_lfCccmThresholdSR)
      {
        continue;
      }
#endif
      Pel outCb  = 0;
      Pel outCr  = 0;

      lfCccmConvolveModel(&src0[x], &src1[x], &src2[x], outCb, outCr, m_lfCccmModelCb, m_lfCccmModelCr);
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      if (m_lfCccmIsFR)
      {
        m_lfCccmBadWindow |= abs(curCb - outCb) > m_lfCccmBadBlockThreshold;
        m_lfCccmBadWindow |= abs(curCr - outCr) > m_lfCccmBadBlockThreshold;
        skipLoop = m_lfCccmBadWindow;
      }
      else
      {
        m_lfCccmBadWindowSR |= abs(curCb - outCb) > m_lfCccmBadBlockThreshold;
        m_lfCccmBadWindowSR |= abs(curCr - outCr) > m_lfCccmBadBlockThreshold;
        skipLoop = m_lfCccmBadWindowSR;
      }
#else
      m_lfCccmBadWindow |= abs(curCb-outCb) > m_lfCccmBadBlockThreshold;
      m_lfCccmBadWindow |= abs(curCr-outCr) > m_lfCccmBadBlockThreshold;
#endif
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
      switch (impMode)
      {
      case 0:
        uCb[x] = (curCb + outCb + 1) >> 1;
        uCr[x] = (curCr + outCr + 1) >> 1;
        break;
      case 1:
        uImpCb[0][x] = (curCb + (outCb << 1) + outCb + 2) >> 2;
        uImpCr[0][x] = (curCr + (outCr << 1) + outCr + 2) >> 2;
        break;
      case 2:
        uImpCb[1][x] = ((curCb << 1) + curCb + (outCb << 2) + outCb + 4) >> 3;
        uImpCr[1][x] = ((curCr << 1) + curCr + (outCr << 2) + outCr + 4) >> 3;
        break;
      case 3:
        uImpCb[2][x] = ((curCb << 2) + curCb + (outCb << 1) + outCb + 4) >> 3;
        uImpCr[2][x] = ((curCr << 2) + curCr + (outCr << 1) + outCr + 4) >> 3;
        break;
      case 4:
        uImpCb[3][x] = ((curCb << 1) + curCb + outCb + 2) >> 2;
        uImpCr[3][x] = ((curCr << 1) + curCr + outCr + 2) >> 2;
        break;
      case 5:
        uCb[x] = (curCb + outCb + 1) >> 1;
        uCr[x] = (curCr + outCr + 1) >> 1;
        uImpCb[0][x] = (curCb + (outCb << 1) + outCb + 2) >> 2;
        uImpCr[0][x] = (curCr + (outCr << 1) + outCr + 2) >> 2;
        uImpCb[1][x] = ((curCb << 1) + curCb + (outCb << 2) + outCb + 4) >> 3;
        uImpCr[1][x] = ((curCr << 1) + curCr + (outCr << 2) + outCr + 4) >> 3;
        uImpCb[2][x] = ((curCb << 2) + curCb + (outCb << 1) + outCb + 4) >> 3;
        uImpCr[2][x] = ((curCr << 2) + curCr + (outCr << 1) + outCr + 4) >> 3;
        uImpCb[3][x] = ((curCb << 1) + curCb + outCb + 2) >> 2;
        uImpCr[3][x] = ((curCr << 1) + curCr + outCr + 2) >> 2;
        break;
      default:
        THROW("The factor idx is error.");
        break;
      }
#else
      uCb[x] = (curCb + outCb + 1) >> 1;
      uCr[x] = (curCr + outCr + 1) >> 1;
#endif
    }
    uCb += strideUCb;
    uCr += strideUCr;
    recCb += strideCb;
    recCr += strideCr;
    src0 += strideY;
    src1 += strideY;
    src2 += strideY;
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
    {
      uImpCb[factorIdx] += strideUImpCb[factorIdx];
      uImpCr[factorIdx] += strideUImpCr[factorIdx];
    }
#endif
  }
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  if (m_lfCccmSampleLessThanModelSizeFlag)
  {
    lfCccmRecoverStatus();
  }
#endif
}
void LoopFilterCccm::lfCccmWindowProcess(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  const int impMode
#endif
)
{
  if( m_lfCccmIsEncoder )
  {
    m_lfCccmMmap = m_lfCccmModelMap[m_lfCccmModelType];
    for(int i = 0; i < 11; i++)
    {
      m_lfCccmMmap2[i] = m_lfCccmMmap[i] + m_lfCccmArrayStride;
    }
  }
  m_lfCccmSamples[m_lfCccmModelSize-1] = m_lfCccmLumaOffset;
  const int starty = m_lfCccmPartitionArea.y;
  const int startx = m_lfCccmPartitionArea.x;
  const int endy = m_lfCccmPartitionArea.bottomRight().getY()+1;
  const int endx = m_lfCccmPartitionArea.bottomRight().getX()+1;
  const int ystep = m_lfCccmWindowSize.height;
  const int xstep = m_lfCccmWindowSize.width;
  for (int yi = starty; yi < endy; yi += ystep)
  {
    for (int xi = startx; xi < endx; xi += xstep)
    {
      lfCccmSetWindows(xi,yi);
      if( m_lfCccmRegressionWindow.area() < m_lfCccmModelSize )
      {
        continue;
      }
#if !JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      m_lfCccmMultiModel = 0;
#if JVET_AN0098_ALF_CCCM_REGULARIZATION
      setLfCccmBadPointThreshold();
#endif
      m_lfCccmBadWindow = false;
      lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
        impMode
#endif
      );
      if( m_lfCccmBadWindow )
      {
#endif
        m_lfCccmBadWindow = false;
        lfCccmSetMultiModelParameters();
        m_lfCccmMultiModel = 1;
#if JVET_AN0098_ALF_CCCM_REGULARIZATION
        setLfCccmBadPointThreshold();
#endif
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        m_lfCccmIsFR = true;
        m_lfCccmMultiModelSR = -1;
#endif
        lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
          impMode
#endif
        );
#if JVET_AM0063_ALF_CCCM_UPDATED_MULTI_MODELS_STRATEGY
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        if (!m_lfCccmSkipFlag && !m_lfCccmSampleLessThanModelSizeFlag && m_lfCccmBadWindow)
        {
          m_lfCccmIsFR = false;
          lfCccmSetMultiModelParametersSR();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 1;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          lfCccmDealWithBadWindow();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 2;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          lfCccmDealWithBadWindow();
        }
        else if (m_lfCccmBadWindow)
        {
          lfCccmDealWithBadWindow();
        }
#else
        lfCccmDealWithBadWindow();
#endif
        m_lfCccmBadWindow = false;
#else
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        if (!m_lfCccmSampleLessThanModelSizeFlag && !m_lfCccmSkipFlag && m_lfCccmBadWindow)
        {
          m_lfCccmIsFR = false;
          lfCccmSetMultiModelParametersSR();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 1;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          m_lfCccmMultiModelSR = 2;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          m_lfCccmBadWindow = m_lfCccmBadWindowSR;
        }
        bool skipFlag = m_lfCccmBadWindow || m_lfCccmSkipFlag;
#endif
#endif
        m_lfCccmMultiModel = 2;
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        m_lfCccmIsFR = true;
        m_lfCccmMultiModelSR = -1;
#endif
        lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
          impMode
#endif
        );
#if JVET_AM0063_ALF_CCCM_UPDATED_MULTI_MODELS_STRATEGY
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        if (!m_lfCccmSkipFlag && !m_lfCccmSampleLessThanModelSizeFlag && m_lfCccmBadWindow)
        {
          m_lfCccmIsFR = false;
          lfCccmSetMultiModelParametersSR();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 1;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          lfCccmDealWithBadWindow();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 2;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          lfCccmDealWithBadWindow();
        }
        else if (m_lfCccmBadWindow)
        {
          lfCccmDealWithBadWindow();
        }
#else
        lfCccmDealWithBadWindow();
#endif
#else
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
        if (!skipFlag && !m_lfCccmSampleLessThanModelSizeFlag && m_lfCccmBadWindow)
        {
          m_lfCccmIsFR = false;
          lfCccmSetMultiModelParametersSR();
          m_lfCccmBadWindowSR = false;
          m_lfCccmMultiModelSR = 1;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          m_lfCccmMultiModelSR = 2;
          lfCccmFiltersConvolution(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
            impMode
#endif
          );
          m_lfCccmBadWindow = m_lfCccmBadWindowSR;
        }
#endif
#endif
#if !JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      }
#endif
#if !JVET_AM0063_ALF_CCCM_UPDATED_MULTI_MODELS_STRATEGY
      if (m_lfCccmBadWindow)
      {
        m_lfCccmUpdateCb.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea).copyFrom(m_lfCccmSaoCb.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea));
        m_lfCccmUpdateCr.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea).copyFrom(m_lfCccmSaoCr.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea));
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
        if (m_lfCccmIsEncoder)
        {
          for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
          {
            m_lfCccmUpdateImpCb[factorIdx].subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea).copyFrom(m_lfCccmSaoCb.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea));
            m_lfCccmUpdateImpCr[factorIdx].subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea).copyFrom(m_lfCccmSaoCr.subBuf(m_lfCccmWindowArea, m_lfCccmWindowArea));
          }
        }
#endif 
      }
#endif
    }
  }
}
#if JVET_AM0063_ALF_CCCM_UPDATED_MULTI_MODELS_STRATEGY
void LoopFilterCccm::lfCccmDealWithBadWindow()
{
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  if (!m_lfCccmIsFR && !m_lfCccmBadWindowSR)
#else
  if (!m_lfCccmBadWindow)
#endif
  {
    return;
  }

  const Pel* src0 = m_lfCccmLumaSaoDownsampled.bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
  const int strideY = m_lfCccmLumaSaoDownsampled.stride;

  const Pel* recCb = m_lfCccmSaoCb.bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
  const Pel* recCr = m_lfCccmSaoCr.bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
  const int strideCb = m_lfCccmSaoCb.stride;
  const int strideCr = m_lfCccmSaoCr.stride;

  Pel* uCb = m_lfCccmUpdateCb.bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
  Pel* uCr = m_lfCccmUpdateCr.bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
  const int strideUCb = m_lfCccmUpdateCb.stride;
  const int strideUCr = m_lfCccmUpdateCr.stride;
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  Pel* uCbImp[MAX_NUM_FACTOR_LF_CCCM] = { nullptr };
  int strideUCbImp[MAX_NUM_FACTOR_LF_CCCM] = { 0 };
  Pel* uCrImp[MAX_NUM_FACTOR_LF_CCCM] = { nullptr };
  int strideUCrImp[MAX_NUM_FACTOR_LF_CCCM] = { 0 };
  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    uCbImp[factorIdx] = m_lfCccmUpdateImpCb[factorIdx].bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
    strideUCbImp[factorIdx] = m_lfCccmUpdateImpCb[factorIdx].stride;
    uCrImp[factorIdx] = m_lfCccmUpdateImpCr[factorIdx].bufAt(m_lfCccmWindowArea.x, m_lfCccmWindowArea.y);
    strideUCrImp[factorIdx] = m_lfCccmUpdateImpCr[factorIdx].stride;
  }
#endif
  for (int y = 0; y < m_lfCccmWindowArea.height; y++)
  {
    for (int x = 0; x < m_lfCccmWindowArea.width; x++)
    {
      if (m_lfCccmMultiModel == 1 && src0[x] >= m_lfCccmThreshold)
      {
        continue;
      }
      if (m_lfCccmMultiModel == 2 && src0[x] < m_lfCccmThreshold)
      {
        continue;
      }
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
      if (m_lfCccmMultiModelSR == 1 && src0[x] >= m_lfCccmThresholdSR)
      {
        continue;
      }
      if (m_lfCccmMultiModelSR == 2 && src0[x] < m_lfCccmThresholdSR)
      {
        continue;
      }
#endif
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
      uCb[x] = uCbImp[0][x] = uCbImp[1][x] = uCbImp[2][x] = uCbImp[3][x] = recCb[x];
      uCr[x] = uCrImp[0][x] = uCrImp[1][x] = uCrImp[2][x] = uCrImp[3][x] = recCr[x];
#else
      uCb[x] = recCb[x];
      uCr[x] = recCr[x];
#endif
    }
    src0 += strideY;
    recCb += strideCb;
    recCr += strideCr;
    uCb += strideUCb;
    uCr += strideUCr;
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
    {
      uCbImp[factorIdx] += strideUCbImp[factorIdx];
      uCrImp[factorIdx] += strideUCrImp[factorIdx];
    }
#endif
  }
}
#endif
void LoopFilterCccm::lfCccmCalcStats(const PelBuf &saoY, const CPelBuf &saoCb, const CPelBuf& saoCr, CccmModel &cccmModelCb, const Area partitionArea, std::vector<std::vector<int32_t>> &xCorr, std::vector<std::vector<std::vector<int32_t>>> &aCorr, Pel *samples)
{
  auto model6 = [&](const Pel* src0, const Pel *src1, const Pel *src2)
  {
    samples[0] = src0[0]; // C
    samples[1] = src1[0]; // N
    samples[2] = src2[0]; // S
    samples[3] = src0[-1]; // W
    samples[4] = src0[1]; // E
    samples[5] = src1[-1]; // NW
    samples[6] = src1[1]; // NE
    samples[7] = src2[-1]; // SW
    samples[8] = src2[1]; // SE
    samples[9] = cccmModelCb.nonlinear(src0[0]);
  };
  samples[10] = cccmModelCb.bias();
  const Pel *src0 = saoY.bufAt(partitionArea.x,partitionArea.y);
  const Pel *src1 = saoY.bufAt(partitionArea.x,partitionArea.y-1);
  const Pel *src2 = saoY.bufAt(partitionArea.x,partitionArea.y+1);
  const int stride = saoY.stride;
  const Pel *srcCb = saoCb.bufAt(partitionArea.x,partitionArea.y);
  const Pel *srcCr = saoCr.bufAt(partitionArea.x,partitionArea.y);
  const int strideCb = saoCb.stride;
  const int strideCr = saoCr.stride;
  const int endy = partitionArea.height;
  const int endx = partitionArea.width;
  const int pstride = partitionArea.width;
  for (int y = 0, yy = 0; y < endy; y++, yy += pstride)
  {
    for (int x = 0; x < endx; x++)
    {
      model6(&src0[x],&src1[x],&src2[x]);
      const int linIdx = x + yy;
      for( int coli0 = 0; coli0 < 11; coli0++ )
      {
        xCorr[linIdx][coli0] = samples[coli0] * srcCb[x];
        xCorr[linIdx][coli0+m_lfCccmArrayStride] = samples[coli0] * srcCr[x];
        for( int coli1 = coli0; coli1 < 11; coli1++ )
        {
          aCorr[linIdx][coli0][coli1] = samples[coli0] * samples[coli1];
        }
      }
    }
    src0 += stride;
    src1 += stride;
    src2 += stride;
    srcCb += strideCb;
    srcCr += strideCr;
  }
}
void LoopFilterCccm::lfCccmFillDownsampledLumaBuffers(const CodingStructure &cs)
{
  Area downsampleArea = m_lfCccmPartitionArea;
  downsampleArea.x -= 1;
  downsampleArea.y -= 1;
  downsampleArea.width += 2;
  downsampleArea.height += 2;

  downsampleArea = clipArea(downsampleArea,Area(0,0,cs.picture->Cb().width,cs.picture->Cb().height));

  const bool clipBoundary[4] =
  {
    downsampleArea.x < 0,
    downsampleArea.y < 0,
    downsampleArea.bottomRight().x == cs.picture->chromaSize().width-1,
    downsampleArea.bottomRight().y == cs.picture->chromaSize().height-1
  };

  if(clipBoundary[0])
  {
    downsampleArea.x = 0;
    downsampleArea.width -= 1;
  }
  if(clipBoundary[1])
  {
    downsampleArea.y = 0;
    downsampleArea.height -= 1;
  }


  PelBuf lumaSaoCtu = m_lfCccmLumaSaoDownsampled.subBuf(downsampleArea,downsampleArea);
  Pel *dstSao = lumaSaoCtu.bufAt(0,0); // out
  const int dstStrideSao = lumaSaoCtu.stride;

  const int starty = downsampleArea.y;
  const int startx = downsampleArea.x;
  const int endy = downsampleArea.bottomRight().y+1;
  const int endx = downsampleArea.bottomRight().x+1;

#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  Pel (LoopFilterCccm::*lfCccmDownsample)(const Pel*, const int, const int, const int) const =
    ( !cs.sps->getCclmVerCollocatedChromaFlag() && !cs.sps->getCclmHorCollocatedChromaFlag() ) ? &LoopFilterCccm::lfCccmDownsampleType1 : &LoopFilterCccm::lfCccmDownsampleType0;
#endif
  const Pel *bufSao = m_lfCccmSaoY.buf;
  const int srcStrideSao = m_lfCccmSaoY.stride;

  for (int y = starty; y < endy; y++)
  {
    for (int x = startx, xx = 0; x < endx; x++, xx++)
    {
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
      dstSao[xx] = (this->*lfCccmDownsample)(bufSao, srcStrideSao, x, y);
#else
      dstSao[xx] = lfCccmDownsample(bufSao, srcStrideSao, x, y);
#endif
    }
    dstSao += dstStrideSao;
  }

  auto addPadding = [](const bool *clipBoundary, PelBuf &buf)
  {
    if(clipBoundary[0])
    {
      for(int y = 0; y < buf.height; y++)
      {
        buf.at(-1,y) = buf.at(0,y);
      }
    }
    if(clipBoundary[1])
    {
      for(int x = 0; x < buf.width; x++)
      {
        buf.at(x,-1) = buf.at(x,0);
      }
    }
    if(clipBoundary[2])
    {
      for(int y = 0; y < buf.height; y++)
      {
        buf.at(buf.width,y) = buf.at(buf.width-1,y);
      }
    }
    if(clipBoundary[3])
    {
      for(int x = 0; x < buf.width; x++)
      {
        buf.at(x,buf.height) = buf.at(x,buf.height-1);
      }
    }
    if(clipBoundary[0] && clipBoundary[1]) // top-left
    {
      buf.at(-1,-1) = buf.at(0,0);
    }
    if(clipBoundary[0] && clipBoundary[3]) // bottom-left
    {
      buf.at(-1,buf.height) = buf.at(0,buf.height-1);
    }
    if(clipBoundary[2] && clipBoundary[1]) // top-right
    {
      buf.at(buf.width,-1) = buf.at(buf.width-1,0);
    }
    if(clipBoundary[2] && clipBoundary[3]) // bottom-right
    {
      buf.at(buf.width,buf.height) = buf.at(buf.width-1,buf.height-1);
    }
  };

  addPadding(clipBoundary, lumaSaoCtu);
}
void LoopFilterCccm::lfCccmSetFrameLevelInheritedParameters(CodingStructure &cs, const int ctuRsAddr0)
{
  const Slice &slice = *cs.slice;
  if(slice.isIntra())
  {
    return;
  }
  if(!cs.slice->m_lfCccmFrameLevelInherit)
  {
    return;
  }

  const Picture *refPic = cs.slice->lfCccmGetReferencePicture();

  if(ctuRsAddr0 >= 0)
  {
    cs.slice->m_lfCccmEnabled.at(ctuRsAddr0) = refPic ? refPic->cs->slice->m_lfCccmEnabled.at(ctuRsAddr0) : 0;
    cs.slice->m_lfCccmModelType.at(ctuRsAddr0) = refPic ? refPic->cs->slice->m_lfCccmModelType.at(ctuRsAddr0) : 0;
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr0) = refPic? refPic->cs->slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr0) : 0;
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr0) = 0;
    return;
  }
  for(int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
  {
    cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = refPic ? refPic->cs->slice->m_lfCccmEnabled.at(ctuRsAddr) : 0;
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) = refPic ? refPic->cs->slice->m_lfCccmModelType.at(ctuRsAddr): 0;
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = refPic? refPic->cs->slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) : 0;
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = 0;
  }
}
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
void LoopFilterCccm::lfCccmCtuProcess(const CodingStructure& cs, const PelUnitBuf& recSao, const int ctuRsAddr, PelUnitBuf& update, const int impMode, PelUnitBuf* updateImp[MAX_NUM_FACTOR_LF_CCCM], const bool isRDO, bool* ctuProcessedEnc)
#else
void LoopFilterCccm::lfCccmCtuProcess(const CodingStructure &cs, const PelUnitBuf &recSao, const int ctuRsAddr, PelBuf &updateCb, PelBuf &updateCr, bool *ctuProcessedEnc )
#endif
{
  if(!cs.slice->m_lfCccmEnabled.at(ctuRsAddr))
  {
    return;
  }

  m_lfCccmCalcStats = lfCccmCalcStats;

  m_lfCccmWindowSize = m_lfCccmWindowSizes.at(cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr));
  m_lfCccmModelType = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  m_lfCccmSkipFlag = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) > 4 ? true : false;
#endif
  m_lfCccmSaoY = recSao.get(COMPONENT_Y);
  m_lfCccmSaoCb = recSao.get(COMPONENT_Cb);
  m_lfCccmSaoCr = recSao.get(COMPONENT_Cr);

  m_lfCccmClpRngCb = cs.slice->clpRng(COMPONENT_Cb);
  m_lfCccmClpRngCr = cs.slice->clpRng(COMPONENT_Cr);
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  m_lfCccmUpdateCb = update.get(COMPONENT_Cb);
  m_lfCccmUpdateCr = update.get(COMPONENT_Cr);
  for (int factorIdx = 0; factorIdx < MAX_NUM_FACTOR_LF_CCCM; factorIdx++)
  {
    m_lfCccmUpdateImpCb[factorIdx] = updateImp[factorIdx]->get(COMPONENT_Cb);
    m_lfCccmUpdateImpCr[factorIdx] = updateImp[factorIdx]->get(COMPONENT_Cr);
  }
#else
  m_lfCccmUpdateCb = updateCb;
  m_lfCccmUpdateCr = updateCr;
#endif
  switch( m_lfCccmModelType )
  {
    case 0:
      m_lfCccmModelSize = CCCM_NUM_PARAMS;
      break;
    case 1:
      m_lfCccmModelSize = 2;
      break;
    case 2:
      m_lfCccmModelSize = 3;
      break;
    case 3:
      m_lfCccmModelSize = 6;
      break;
    case 4:
      m_lfCccmModelSize = 5;
      break;
    case 5:
      m_lfCccmModelSize = 5;
      break;
    case 6:
      m_lfCccmModelSize = 11;
      break;
    case 7:
      m_lfCccmModelSize = 10;
      break;
    default:
      THROW("modeltype not supported");
      break;
  }
#if JVET_AN0101_ALF_CCCM_REMOVE_SINGLE_MODEL
  m_lfCccmChromaBitDepth = cs.sps->getBitDepth(CH_C);
#endif
  m_lfCccmModelCb = CccmModel(m_lfCccmModelSize, cs.sps->getBitDepth(CH_C));
  m_lfCccmModelCr = CccmModel(m_lfCccmModelSize, cs.sps->getBitDepth(CH_C));

  const PreCalcValues &pcv = *cs.pcv;
  const int xc = ctuRsAddr % pcv.widthInCtus;
  const int yc = ctuRsAddr / pcv.widthInCtus;
  m_lfCccmPartitionArea = clipArea(UnitArea(pcv.chrFormat, Area(xc << pcv.maxCUWidthLog2, yc << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth)), *cs.slice->getPic()).Cb();

#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
  m_lfCccmIsEncoder = cs.pcv->isEncoder && isRDO;
#else
  m_lfCccmIsEncoder = cs.pcv->isEncoder;
#endif
#if JVET_AN0098_ALF_CCCM_REGULARIZATION
  m_lfCccmHasNonLinearTerm = (m_lfCccmModelType != 1 && m_lfCccmModelType != 3 && m_lfCccmModelType != 7) ? true : false;
#endif
  if( m_lfCccmIsEncoder )
  {
    if(ctuProcessedEnc && !*ctuProcessedEnc)
    {
      lfCccmInitBd(*cs.sps);
      lfCccmFillDownsampledLumaBuffers(cs);
      m_lfCccmCalcStats(m_lfCccmLumaSaoDownsampled, m_lfCccmSaoCb, m_lfCccmSaoCr, m_lfCccmModelCb, m_lfCccmPartitionArea, m_lfCccmXCorr, m_lfCccmAutoCorr, m_lfCccmSamples);
      *ctuProcessedEnc = true;
    }
  }
  else
  {
    lfCccmInitBd(*cs.sps);
    lfCccmFillDownsampledLumaBuffers(cs);
  }
  lfCccmWindowProcess(
#if JVET_AM0063_ALF_CCCM_ADAPTIVE_FACTOR
    impMode
#endif
  );
}
#endif
