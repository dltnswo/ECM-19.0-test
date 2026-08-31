/*/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2024, ITU/ISO/IEC
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

#include "CommonDef.h"

#if JVET_AJ0249_NEURAL_NETWORK_BASED
#include "IntraPredictionNN.h"
#include <IntraNnip/model_scalar.h>
#include <IntraNnip/model_sse42.h>
#include <IntraNnip/model_avx2.h>
#include "IntraPredictionNnModels.h"

extern int8_t g_aucLog2[MAX_CU_SIZE + 1];
#include "Unit.h"
#include "CodingStructure.h"

namespace
{
#ifdef TARGET_SIMD_X86
  static X86_VEXT vext = X86_VEXT::SCALAR;
#endif
  using tModelCoeffs = unsigned int;
  static const int g_inputQuantizer = 7;

  struct ModelParam
  {
    WidthHeight modelTargetSize;
    WidthHeight contextMaxLength;
    WidthHeight contextNbLines;
    tModelCoeffs *modelCoeffs;
    int modelCoeffsSizeInBytes;
  };

  struct PnnModel
  {
    ModelParam param;
    std::unique_ptr<sadl_scalar::Model<int16_t>> targetModel_scalar;
    std::unique_ptr<sadl_sse42::Model<int16_t>> targetModel_sse42;
    std::unique_ptr<sadl_avx2::Model<int16_t>> targetModel_avx2;
    std::vector<sadl_scalar::Tensor<int16_t>> inputTensor_scalar;
    std::vector<sadl_sse42::Tensor<int16_t>> inputTensor_sse42;
    std::vector<sadl_avx2::Tensor<int16_t>> inputTensor_avx2;
    sadl_scalar::layers::Layer<int16_t> *outputLayer_scalar = nullptr;
    sadl_sse42::layers::Layer<int16_t> *outputLayer_sse42 = nullptr;
    sadl_avx2::layers::Layer<int16_t> *outputLayer_avx2 = nullptr;
    int16_t* getInputData() 
    {
#ifdef TARGET_SIMD_X86
      if (::vext >= X86_VEXT::AVX2)
      {
        return inputTensor_avx2[0].data();
      }
      else if (::vext >= X86_VEXT::SSE42)
      {
        return inputTensor_sse42[0].data();
      }
      else
#endif
      {
        return inputTensor_scalar[0].data();
      }
    }
    int16_t* getOutputData() 
    {
#ifdef TARGET_SIMD_X86
      if (::vext >= X86_VEXT::AVX2)
      {
        return outputLayer_avx2->output().data();
      }
      else if (::vext >= X86_VEXT::SSE42)
      {
        return outputLayer_sse42->output().data();
      }
      else
#endif
      {
        return outputLayer_scalar->output().data();
      }
    }
    void apply()
    {
#ifdef TARGET_SIMD_X86
      if (::vext >= X86_VEXT::AVX2)
      {
        sadl_avx2::infer(*targetModel_avx2, inputTensor_avx2);
      }
      else if (::vext >= X86_VEXT::SSE42)
      {
        sadl_sse42::infer(*targetModel_sse42, inputTensor_sse42);
      }
      else
#endif
      {
        sadl_scalar::infer(*targetModel_scalar, inputTensor_scalar);
      }
    }
    int getoutputQuantizer() 
    { 
#ifdef TARGET_SIMD_X86
      if (::vext >= X86_VEXT::AVX2)
      {
        return outputLayer_avx2->output().quantizer;
      }
      else if (::vext >= X86_VEXT::SSE42)
      {
        return outputLayer_sse42->output().quantizer;
      }
      else
#endif
      {
        return outputLayer_scalar->output().quantizer;
      }
    }
    PnnModel(const ModelParam& cfg_param) { param = cfg_param; }
    void readGraphInitModel(const tModelCoeffs* coeffs, const size_t modelSize)
    {
      std::string        input{ (char*)coeffs, modelSize };
      std::istringstream file(input, std::ios::binary);

#ifdef TARGET_SIMD_X86
      if (::vext >= X86_VEXT::AVX2)
      {
        inputTensor_avx2 = { sadl_avx2::Tensor<int16_t>(sadl_avx2::Dimensions{
            { 1, static_cast<int>(param.contextNbLines.first * (param.contextNbLines.second + param.contextMaxLength.first) + param.contextNbLines.second * param.contextMaxLength.second) } }) };
        targetModel_avx2 = sadl_avx2::load(file, inputTensor_avx2);
        for (auto& tensorInput : inputTensor_avx2)
        {
          tensorInput.quantizer = g_inputQuantizer;
        }
        outputLayer_avx2 = targetModel_avx2->getOutputLayer().matMul.get();
      }
      else if (::vext >= X86_VEXT::SSE42)
      {
        inputTensor_sse42 = { sadl_sse42::Tensor<int16_t>(sadl_sse42::Dimensions{
            { 1, static_cast<int>(param.contextNbLines.first * (param.contextNbLines.second + param.contextMaxLength.first) + param.contextNbLines.second * param.contextMaxLength.second) } }) };
        targetModel_sse42 = sadl_sse42::load(file, inputTensor_sse42);
        for (auto& tensorInput : inputTensor_sse42)
        {
          tensorInput.quantizer = g_inputQuantizer;
        }
        outputLayer_sse42 = targetModel_sse42->getOutputLayer().matMul.get();
      }
      else
#endif
      {
        inputTensor_scalar = { sadl_scalar::Tensor<int16_t>(sadl_scalar::Dimensions{
        { 1, static_cast<int>(param.contextNbLines.first * (param.contextNbLines.second + param.contextMaxLength.first) + param.contextNbLines.second * param.contextMaxLength.second) } }) };
        targetModel_scalar = sadl_scalar::load(file, inputTensor_scalar);
        for (auto& tensorInput : inputTensor_scalar)
        {
          tensorInput.quantizer = g_inputQuantizer;
        }
        outputLayer_scalar = targetModel_scalar->getOutputLayer().matMul.get();
      }
    }
  };

  static const std::vector<ModelParam> g_pnnModelParams = {
    { std::make_pair(4, 4), std::make_pair(8, 8), std::make_pair(4, 4), (tModelCoeffs*)model_4_4, sizeof(model_4_4) },
    { std::make_pair(4, 8), std::make_pair(8, 16), std::make_pair(4, 4), (tModelCoeffs*)model_4_8, sizeof(model_4_8) },
    { std::make_pair(4, 16), std::make_pair(8, 32), std::make_pair(4, 4), (tModelCoeffs*)model_4_16, sizeof(model_4_16) },
    { std::make_pair(8, 8), std::make_pair(16, 16), std::make_pair(8, 8), (tModelCoeffs*)model_8_8, sizeof(model_8_8) },
    { std::make_pair(8, 16), std::make_pair(16, 32), std::make_pair(8, 8), (tModelCoeffs*)model_8_16, sizeof(model_8_16) },
    { std::make_pair(16, 16), std::make_pair(32, 32), std::make_pair(8, 8), (tModelCoeffs*)model_16_16, sizeof(model_16_16) }
  };

  int fillMapHeightWidthTargetModel(std::map<WidthHeight, int16_t> &mapHeightWidthTargetModelIndex, std::vector<PnnModel> &pnnModels)
  {
#ifdef TARGET_SIMD_X86
    ::vext = read_x86_extension_flags();
#endif
    for (auto &param: g_pnnModelParams)
    {
      if (mapHeightWidthTargetModelIndex.find(param.modelTargetSize) != mapHeightWidthTargetModelIndex.end())
      {
        return -1;
      }
      mapHeightWidthTargetModelIndex.insert(std::make_pair(param.modelTargetSize, static_cast<int16_t>(pnnModels.size())));
      pnnModels.push_back(PnnModel(param));
      pnnModels.back().readGraphInitModel(param.modelCoeffs, param.modelCoeffsSizeInBytes);
    }
    return 0;
  }

  void invertPreprocessingContext(const Pel *ptrSrc, Pel *ptrDst, const int heightImage, const int widthImage,
                                  const int strideSrc, const int strideDst, const int nbBitshiftsScaling,
                                  const int meanContext, const int bitdepth)
  {
    const int rounding = 1 << (nbBitshiftsScaling - 1);
    const int maximum  = (1 << (bitdepth + nbBitshiftsScaling)) - 1;
    for (int i = 0; i < heightImage; i++)
    {
      for (int j = 0; j < widthImage; j++)
      {
        const int value = (ptrSrc[j] + (meanContext << nbBitshiftsScaling) + rounding);
        ptrDst[j] = static_cast<Pel>(std::max(0, std::min(maximum, value)) >> nbBitshiftsScaling);
      }
      ptrSrc += strideSrc;
      ptrDst += strideDst;
    }
  }
  
  Pel downsample2d(const Pel *ptrTopLeftRectangle, const int factorDownsamplingVertical, const int factorDownsamplingHorizontal, const int stridePicture)
  {
    int accumulation = 0;
    const int shiftUpsampling = g_aucLog2[factorDownsamplingVertical] + g_aucLog2[factorDownsamplingHorizontal];
    const int rounding = 1 << (shiftUpsampling - 1);
    for (int i = 0; i < factorDownsamplingVertical; i++)
    {
      for (int j = 0; j < factorDownsamplingHorizontal; j++)
      {
        accumulation += ptrTopLeftRectangle[j];
      }
      ptrTopLeftRectangle += stridePicture;
    }
    const Pel value = static_cast<Pel>((accumulation + rounding) >> shiftUpsampling);
    return value;
  }

  struct TransformationsContextPrediction
  {
    WidthHeight pairHeightWidthAfterTr;
    WidthHeight maxNbLines;
    WidthHeight maxCtxLength;
    int         factorDownsamplingVertical; 
    int         factorDownsamplingHorizontal; 
    bool        isTransposed; 
    TransformationsContextPrediction() = default;

    TransformationsContextPrediction(const int blockWidth, const int blockHeight)
    {
      const int minDim = std::min(blockWidth, blockHeight);
      const int maxDim = std::max(blockWidth, blockHeight);
      if (minDim == 8 && maxDim == 32)
      {
        pairHeightWidthAfterTr     = std::pair<unsigned int, unsigned int>(8, 16);
        factorDownsamplingHorizontal = blockWidth > blockHeight ? 2 : 1;
        factorDownsamplingVertical   = blockWidth > blockHeight ? 1 : 2;
        isTransposed                  = blockWidth < blockHeight;
      }
      else if (minDim == 4 && maxDim == 32)
      {
        pairHeightWidthAfterTr     = std::pair<unsigned int, unsigned int>(4, 16);
        factorDownsamplingHorizontal = blockWidth > blockHeight ? 2 : 1;
        factorDownsamplingVertical   = blockWidth > blockHeight ? 1 : 2;
        isTransposed                  = blockWidth < blockHeight;
      }
      else if ((blockHeight == blockWidth && blockHeight > 16) || (minDim == 16 && maxDim == 32))
      {
        pairHeightWidthAfterTr     = std::pair<unsigned int, unsigned int>(16, 16);
        factorDownsamplingHorizontal = blockWidth / 16;
        factorDownsamplingVertical = blockHeight / 16;
        isTransposed = false;
      }
      else
      {
        pairHeightWidthAfterTr     = std::pair<unsigned int, unsigned int>(minDim, maxDim);
        factorDownsamplingHorizontal = 1;
        factorDownsamplingVertical   = 1;
        isTransposed                  = blockHeight > blockWidth;
      }
      maxNbLines = IntraPredictionNN::getNbLines(blockWidth, blockHeight);
      maxNbLines.first  = maxNbLines.first / factorDownsamplingVertical;
      maxNbLines.second = maxNbLines.second / factorDownsamplingHorizontal;
      maxCtxLength = IntraPredictionNN::getPnnCtxLength(blockWidth, blockHeight);
      maxCtxLength.first  = maxCtxLength.first / factorDownsamplingHorizontal;
      maxCtxLength.second = maxCtxLength.second / factorDownsamplingVertical;
      if (isTransposed)
      {
        std::swap(maxNbLines.first, maxNbLines.second);
        std::swap(maxCtxLength.first, maxCtxLength.second);
      }
    }
  };
  template<bool downSampling>
  void extractContextPortions(const Pel *const piRoiOrigin, Pel *const piPortion, Pel *const piPortionLeft, 
                              const WidthHeight &ctxLength, const WidthHeight &maxCtxLength,
                              const WidthHeight &ctxNbLines, const WidthHeight &maxCtxNbLines,
                              const int factorDownsamplingVertical, const int factorDownsamplingHorizontal,
                              const int iPicStride, const Pel maskValue, const bool isTransposed, Pel &meanContext)
  {   
    const int roiOriginStride = iPicStride * factorDownsamplingVertical;
    int sumContext = 0;
    if (isTransposed)
    {
      const int topBorder = maxCtxNbLines.second - ctxNbLines.second;
      const int leftBorder = maxCtxNbLines.first - ctxNbLines.first;
      const Pel *piRoiOriginTemp = piRoiOrigin - ctxNbLines.second * factorDownsamplingVertical * iPicStride - ctxNbLines.first * factorDownsamplingHorizontal;
      for (int i = 0; i < topBorder; i++)
      {
        Pel *piPortionTemp = piPortion + i * maxCtxNbLines.first;
        for (int j = 0; j < maxCtxNbLines.first; j++)
        {
          piPortionTemp[j] = maskValue;
        }
      }
      for (int i = 0; i < ctxNbLines.second + ctxLength.first; i++)
      {
        Pel *piPortionTemp = piPortion + (i + topBorder) * maxCtxNbLines.first;
        for (int j = 0; j < leftBorder; j++)
        {
          piPortionTemp[j] = maskValue;
        }
        piPortionTemp += leftBorder;
        for (int j = 0; j < ctxNbLines.first; j++)
        {
          if (downSampling)
          {
            const Pel valueDownsampling = downsample2d(piRoiOriginTemp + j * factorDownsamplingHorizontal, factorDownsamplingVertical, factorDownsamplingHorizontal, iPicStride);
            piPortionTemp[j] = static_cast<Pel>(valueDownsampling);
            sumContext += valueDownsampling;
          }
          else
          {
            piPortionTemp[j] = piRoiOriginTemp[j];
            sumContext += piRoiOriginTemp[j];
          }
        }
        piRoiOriginTemp += roiOriginStride;
      }
      piRoiOriginTemp = piRoiOrigin - ctxNbLines.second * factorDownsamplingVertical * iPicStride;
      for (int j = 0; j < topBorder; j++)
      {
        Pel *piPortionTemp = piPortionLeft + j;
        for (int k = 0; k < ctxLength.second; k++)
        {
          piPortionTemp[k * maxCtxNbLines.second] = maskValue;
        }
      }
      for (int j = 0; j < ctxNbLines.second; j++)
      {
        Pel *piPortionTemp = piPortionLeft + topBorder + j;
        for (int k = 0; k < ctxLength.second; k++)
        {
          if (downSampling)
          {
            const Pel valueDownsampling = downsample2d(piRoiOriginTemp + k * factorDownsamplingHorizontal, factorDownsamplingVertical, factorDownsamplingHorizontal, iPicStride);
            piPortionTemp[k * maxCtxNbLines.second] = static_cast<Pel>(valueDownsampling);
            sumContext += valueDownsampling;
          }
          else
          {
            piPortionTemp[k * maxCtxNbLines.second] = piRoiOriginTemp[k];
            sumContext += piRoiOriginTemp[k];
          }
        }
        piRoiOriginTemp += roiOriginStride;
      }
    }
    else
    {
      const int topBorder = maxCtxNbLines.first - ctxNbLines.first;
      const int leftBorder = maxCtxNbLines.second - ctxNbLines.second;
      const Pel *piRoiOriginTemp = piRoiOrigin - ctxNbLines.first * factorDownsamplingVertical * iPicStride - ctxNbLines.second * factorDownsamplingHorizontal;
      for (int i = 0; i < topBorder; i++)
      {
        Pel *piPortionTemp = piPortion + i;
        for (int j = 0; j < maxCtxNbLines.second + ctxLength.first; j++)
        {
          piPortionTemp[j * maxCtxNbLines.first] = maskValue;
        }
      }
      for (int i = 0; i < ctxNbLines.first; i++)
      {
        Pel *piPortionTemp = piPortion + topBorder + i;
        for (int j = 0; j < leftBorder; j++)
        {
          piPortionTemp[j * maxCtxNbLines.first] = maskValue;
        }
        piPortionTemp = piPortion + leftBorder * maxCtxNbLines.first + topBorder + i;
        for (int j = 0; j < ctxNbLines.second + ctxLength.first; j++)
        {
          if (downSampling)
          {
            const Pel valueDownsampling = downsample2d(piRoiOriginTemp + j * factorDownsamplingHorizontal, factorDownsamplingVertical, factorDownsamplingHorizontal, iPicStride);
            piPortionTemp[j * maxCtxNbLines.first] = static_cast<Pel>(valueDownsampling);
            sumContext += valueDownsampling;
          }
          else
          {
            piPortionTemp[j * maxCtxNbLines.first] = piRoiOriginTemp[j];
            sumContext += piRoiOriginTemp[j];
          }
        }
        piRoiOriginTemp += roiOriginStride;
      }
      for (int j = 0; j < ctxLength.second; j++)
      {
        Pel *piPortionTemp = piPortionLeft + j * maxCtxNbLines.second;
        for (int k = 0; k < leftBorder; k++)
        {
          piPortionTemp[k] = maskValue;
        }
        piPortionTemp += leftBorder;
        for (int k = 0; k < ctxNbLines.second; k++)
        {
          if (downSampling)
          {
            const Pel valueDownsampling = downsample2d(piRoiOriginTemp + k * factorDownsamplingHorizontal, factorDownsamplingVertical, factorDownsamplingHorizontal, iPicStride);
            piPortionTemp[k] = static_cast<Pel>(valueDownsampling);
            sumContext += valueDownsampling;
          }
          else
          {
            piPortionTemp[k] = piRoiOriginTemp[k];
            sumContext += piRoiOriginTemp[k];
          }
        }
        piRoiOriginTemp += roiOriginStride;
      }
    }
    const int counterInSumContext = ctxNbLines.first * (ctxNbLines.second + ctxLength.first) + ctxNbLines.second * ctxLength.second;
    meanContext = counterInSumContext > 0 ? (sumContext + (counterInSumContext >> 1)) / counterInSumContext : 0;
    Pel *piPortionRem = piPortion + (maxCtxNbLines.second + ctxLength.first) * maxCtxNbLines.first;
    for (int i = 0; i < (maxCtxLength.first - ctxLength.first) * maxCtxNbLines.first; i++)
    {
      piPortionRem[i] = maskValue;
    }
    piPortionRem = piPortionLeft + ctxLength.second * maxCtxNbLines.second;
    for (int i = 0; i < (maxCtxLength.second - ctxLength.second) * maxCtxNbLines.second; i++)
    {
      piPortionRem[i] = maskValue;
    }
  }

  void zeroMeanContextPortion(Pel* piPortion, const int portionSize, const int nbBitshiftsScaling, const Pel meanContext)
  {
    for (int i = 0; i < portionSize; i++)
    {
      piPortion[i] = (piPortion[i] - meanContext) << nbBitshiftsScaling;
    }
  }

  void extractZeroMeanContextPortions(const Pel *const piRoiOrigin, const WidthHeight &aboveLeftAvailableContext, 
                                      const WidthHeight &aboveLeftAvailableNbLines, Pel *const piPortion,
                                      const TransformationsContextPrediction &transfos, const int iPicStride,
                                      const int nbBitshiftsScaling, const Pel maskValue, Pel &meanContext,
                                      WidthHeight &ctxLength)
  {
    ctxLength = WidthHeight(aboveLeftAvailableContext.first / transfos.factorDownsamplingHorizontal, aboveLeftAvailableContext.second / transfos.factorDownsamplingVertical);
    WidthHeight ctxNbLines = WidthHeight(aboveLeftAvailableNbLines.first / transfos.factorDownsamplingVertical, aboveLeftAvailableNbLines.second / transfos.factorDownsamplingHorizontal);
    if (transfos.isTransposed)
    {
      std::swap(ctxLength.first, ctxLength.second);
      std::swap(ctxNbLines.first, ctxNbLines.second);
    }
    Pel *const piPortionLeft = piPortion + transfos.maxNbLines.first * (transfos.maxNbLines.second + transfos.maxCtxLength.first);
    if (transfos.factorDownsamplingVertical > 1 || transfos.factorDownsamplingHorizontal > 1)
    {
      extractContextPortions<true>(piRoiOrigin, piPortion, piPortionLeft, ctxLength, transfos.maxCtxLength, ctxNbLines, transfos.maxNbLines, transfos.factorDownsamplingVertical, transfos.factorDownsamplingHorizontal, iPicStride, maskValue, transfos.isTransposed, meanContext);
    }
    else
    {
      extractContextPortions<false>(piRoiOrigin, piPortion, piPortionLeft, ctxLength, transfos.maxCtxLength, ctxNbLines, transfos.maxNbLines, transfos.factorDownsamplingVertical, transfos.factorDownsamplingHorizontal, iPicStride, maskValue, transfos.isTransposed, meanContext);
    }
    zeroMeanContextPortion(piPortion, (transfos.maxNbLines.second + ctxLength.first) * transfos.maxNbLines.first, nbBitshiftsScaling, meanContext);
    zeroMeanContextPortion(piPortionLeft, ctxLength.second * transfos.maxNbLines.second, nbBitshiftsScaling, meanContext);
  }

  void upsample1d(const Pel *ptrSrc, Pel *ptrDst, const Pel *ptrBoundary, const int sizeDimSrcUpsampled,
                  const int sizeDimSrcOrth, const int stepSrcUpsampled, const int stepSrcOrth,
                  const int stepDstUpsampled, const int stepDstOrth, const int stepBoundary,
                  const int factorUpsampling)
  {
    if (factorUpsampling == 1)
    {
      for (int i = 0; i < sizeDimSrcOrth; i++)
      {
        const Pel *ptrAfter      = ptrSrc;
        Pel       *ptrCurrentDst = ptrDst;
        for (int j = 0; j < sizeDimSrcUpsampled; j++)
        {
          *ptrCurrentDst = *ptrAfter;
          ptrCurrentDst += stepDstUpsampled;
          ptrAfter += stepSrcUpsampled;
        }
        ptrSrc += stepSrcOrth;
        ptrDst += stepDstOrth;
      }
    }
    else if (factorUpsampling == 2)
    {
      for (int i = 0; i < sizeDimSrcOrth; i++)
      {
        Pel *ptrCurrentDst = ptrDst;
        Pel center = *ptrSrc;
        Pel left = ptrBoundary == nullptr ? center : *ptrBoundary;
        const Pel *ptrCurrentSrc = ptrSrc + stepSrcUpsampled;
        for (int j = 0; j < sizeDimSrcUpsampled; j++)
        {
          const Pel right    = *ptrCurrentSrc;
          *ptrCurrentDst = (left + 3 * center + 2) >> 2;
          ptrCurrentDst += stepDstUpsampled;
          *ptrCurrentDst = (right + 3 * center + 2) >> 2;
          ptrCurrentDst += stepDstUpsampled;
          left = center;
          center = right;
          if (j < sizeDimSrcUpsampled - 2)
          {
            ptrCurrentSrc += stepSrcUpsampled;
          }
        }
        ptrSrc += stepSrcOrth;
        ptrDst += stepDstOrth;
        if (ptrBoundary != nullptr)
        {
          ptrBoundary += stepBoundary;
        }
      }
    }
    else
    {
      const int shiftUpsampling1 = 1 + g_aucLog2[factorUpsampling];
      const int shiftUpsampling = g_aucLog2[factorUpsampling];
      const int rounding        = 1 << (shiftUpsampling - 1);
      for (int i = 0; i < sizeDimSrcOrth; i++)
      {
        Pel *ptrCurrentDst = ptrDst;
        Pel center         = *ptrSrc;
        Pel           left = ptrBoundary == nullptr ? center : *ptrBoundary;
        const Pel *ptrCurrentSrc = ptrSrc + stepSrcUpsampled;
        for (int j = 0; j < sizeDimSrcUpsampled; j++)
        {
          int valueLeft = ((left + center) << shiftUpsampling) + (center - left);
          for (int k = 0; k < factorUpsampling; k += 2)
          {
            *ptrCurrentDst = (valueLeft + rounding) >> shiftUpsampling1;
            valueLeft += (center - left) << 1;
            ptrCurrentDst += stepDstUpsampled;
          }
          const Pel right    = *ptrCurrentSrc;
          int valueRight = (center << shiftUpsampling1) + (right - center);
          for (int k = 0; k < factorUpsampling; k += 2)
          {
            *ptrCurrentDst = (valueRight + rounding) >> shiftUpsampling1;
            valueRight += (right - center) << 1;
            ptrCurrentDst += stepDstUpsampled;
          }
          left = center;
          center = right;
          if (j < sizeDimSrcUpsampled - 2)
          {
            ptrCurrentSrc += stepSrcUpsampled;
          }
        }
        ptrSrc += stepSrcOrth;
        ptrDst += stepDstOrth;
        if (ptrBoundary != nullptr)
        {
          ptrBoundary += stepBoundary;
        }
      }
    }
  }

  void upsample2d(const Pel *const ptrSrc, Pel *const ptrDst, const Pel *const ptrBoundaryLeft,
                  const Pel *const ptrBoundaryAbove, const int heightTarget, const int widthTarget,
                  const int strideSrc, const int strideDst, const int strideBoundaryLeft,
                  const int factorUpsamplingVertical, const int factorUpsamplingHorizontal,
                  const bool isTransposed)
  {
    const int heightTargetSrc = heightTarget / factorUpsamplingVertical;
    const int widthTargetSrc = widthTarget / factorUpsamplingHorizontal;
    int stepSrcUpsampled = isTransposed ? strideSrc : 1;
    int stepSrcOrth = isTransposed ? 1 : strideSrc;
    Pel *const ptrDstShifted = ptrDst + strideDst * (factorUpsamplingVertical - 1);
    const Pel *ptrSrcShifted = nullptr;
    int sizeDimSrcUpsampled = 0;
    int sizeDimSrcOrth = 0;
    if (factorUpsamplingHorizontal > 1)
    {
      const Pel* const ptrBoundaryLeftShifted = ptrBoundaryLeft == nullptr ? nullptr : ptrBoundaryLeft + strideBoundaryLeft * (factorUpsamplingVertical - 1);
      upsample1d(ptrSrc, ptrDstShifted, ptrBoundaryLeftShifted, widthTargetSrc, heightTargetSrc, stepSrcUpsampled, stepSrcOrth, 1, strideDst * factorUpsamplingVertical, strideBoundaryLeft * factorUpsamplingVertical, factorUpsamplingHorizontal);
      ptrSrcShifted        = ptrDstShifted;
      sizeDimSrcUpsampled = heightTargetSrc;
      sizeDimSrcOrth      = widthTarget;
      stepSrcUpsampled     = strideDst * factorUpsamplingVertical;
      stepSrcOrth          = 1;
    }
    else
    {
      ptrSrcShifted        = ptrSrc;
      sizeDimSrcUpsampled = heightTargetSrc;
      sizeDimSrcOrth      = widthTargetSrc;
      stepSrcUpsampled     = isTransposed ? 1 : strideSrc;
      stepSrcOrth          = isTransposed ? strideSrc : 1;
    }
    upsample1d(ptrSrcShifted, ptrDst, ptrBoundaryAbove, sizeDimSrcUpsampled, sizeDimSrcOrth, stepSrcUpsampled, stepSrcOrth, strideDst, 1, 1, factorUpsamplingVertical);
  }
}  // namespace

struct IntraPredictionNN::InternalData
{
  std::map<WidthHeight, int16_t> m_mapHeightWidthTargetModelIndex;
  std::vector<PnnModel> m_pnnModels;

  int currentModelIndex;
  CompArea m_cachedCollectedArea;
  Area m_cachedPredictedArea;

  int16_t m_meanContext;
  WidthHeight m_ctxLength;
  TransformationsContextPrediction m_transfos;
  Pel m_intermediate[64 * 64];
  Pel m_predicted[MAX_CU_SIZE * MAX_CU_SIZE];
  int m_inputBitdepth;
};

IntraPredictionNN::IntraPredictionNN()
{
  m_predModel = new InternalData();
  m_predModel->m_meanContext = 0;
}

IntraPredictionNN::~IntraPredictionNN()
{
  delete m_predModel;
}

void IntraPredictionNN::init(const int bitdepth)
{
  if (m_predModel->m_mapHeightWidthTargetModelIndex.empty())
  {
    const int error_code = fillMapHeightWidthTargetModel(m_predModel->m_mapHeightWidthTargetModelIndex, m_predModel->m_pnnModels);
    CHECK(error_code < 0, "Error in `fillMapHeightWidthTargetModel`.");
    m_predModel->m_inputBitdepth = bitdepth;
    const int nbBitshiftsScaling = g_inputQuantizer - bitdepth + 8;
    CHECK(nbBitshiftsScaling < 0, "`g_inputQuantizer` is strictly smaller than `bitdepth` - 8.");
  }
}

void IntraPredictionNN::resetCuData()
{
  m_predModel->m_cachedCollectedArea = CompArea();
  m_aboveLeftAvailableContext = WidthHeight(0, 0);
  m_predModel->m_cachedPredictedArea = Area();
  m_predModel->currentModelIndex     = -1;
  for (auto it{m_argmaxs.begin()}; it != m_argmaxs.end(); it++)
  {
    std::fill(it->begin(), it->end(), MAX_INT);
  }
}

void IntraPredictionNN::predictPnn(PelBuf& pDst, const CPelBuf& srcBuf, const int inputBitDepth, const CompArea& area, bool& isPredictionRun)
{
  PelBuf predictedBuf(m_predModel->m_predicted, pDst.width, pDst.height);
  if (m_predModel->m_cachedPredictedArea != area)
  {
    CHECK(area.compID != COMPONENT_Y, "area.compID is not luma");
    CHECK(inputBitDepth != m_predModel->m_inputBitdepth, "`inputBitDepth` is not equal to `m_predModel->m_inputBitdepth`.");
    CHECK(!m_enable, "`enable_` is false.");
    if (!m_enable)
    {
      return;
    }
    const WidthHeight pairHeightWidthBeforeTr(pDst.height, pDst.width);
    PnnModel &pnnModel = m_predModel->m_pnnModels[m_predModel->currentModelIndex];
    pnnModel.apply();

    const int16_t* ptrBeforeUpsampling = pnnModel.getOutputData();
    int nbBitshiftsScaling = pnnModel.getoutputQuantizer() - m_predModel->m_inputBitdepth + 8;
    const int heightTargetSrc = (int) pDst.height / m_predModel->m_transfos.factorDownsamplingVertical;
    const int widthTargetSrc = (int) pDst.width / m_predModel->m_transfos.factorDownsamplingHorizontal;
    const int heightTargetTr = m_predModel->m_transfos.isTransposed ? widthTargetSrc : heightTargetSrc;
    const int widthTargetTr = m_predModel->m_transfos.isTransposed ? heightTargetSrc : widthTargetSrc;
    invertPreprocessingContext(ptrBeforeUpsampling, m_predModel->m_intermediate, heightTargetTr, widthTargetTr, m_predModel->m_transfos.pairHeightWidthAfterTr.second, m_predModel->m_transfos.pairHeightWidthAfterTr.second, nbBitshiftsScaling, m_predModel->m_meanContext, m_predModel->m_inputBitdepth);
    const bool leftBoundaryAvailable = (m_predModel->m_transfos.isTransposed ? m_aboveLeftAvailableNbLines.first : m_aboveLeftAvailableNbLines.second) > 0;
    const bool topBoundaryAvailable = (m_predModel->m_transfos.isTransposed ? m_aboveLeftAvailableNbLines.second : m_aboveLeftAvailableNbLines.first) > 0;
    upsample2d(m_predModel->m_intermediate, m_predModel->m_predicted, leftBoundaryAvailable?  srcBuf.buf + srcBuf.stride + 1: nullptr, topBoundaryAvailable? srcBuf.buf + 1: nullptr, predictedBuf.height, predictedBuf.width, m_predModel->m_transfos.pairHeightWidthAfterTr.second, predictedBuf.stride, 1, m_predModel->m_transfos.factorDownsamplingVertical, m_predModel->m_transfos.factorDownsamplingHorizontal, m_predModel->m_transfos.isTransposed);
    m_predModel->m_cachedPredictedArea = area;
    isPredictionRun = true;
  }
  else
  {
    isPredictionRun = false;
  }
#if !JVET_AK0118_BF_FOR_INTRA_PRED
  pDst.copyFrom(predictedBuf);
#endif
}
#if JVET_AK0118_BF_FOR_INTRA_PRED
Pel* IntraPredictionNN::getNnIntraPredPtr ()
{
  return m_predModel->m_predicted;
}
#endif

bool IntraPredictionNN::decideShapeHandledPnn(const uint32_t height, const uint32_t width)
{
  const int minDim = std::min(height, width);
  const int maxDim = std::max(height, width);
  if (minDim < 4 || maxDim > 64)
  {
    return false;
  }
  WidthHeight blkShape = WidthHeight(minDim, maxDim);
  if (blkShape.first == blkShape.second && blkShape.first > 16)
  {
    blkShape.first = blkShape.second = 16;
  }
  else  if (blkShape.second == 32 && (blkShape.first == 4 || blkShape.first == 8 || blkShape.first == 16))
  {
    blkShape.second = 16;
  }
  if (std::find_if(g_pnnModelParams.begin(), g_pnnModelParams.end(), [blkShape](const ModelParam &a) { return a.modelTargetSize == blkShape; }) != g_pnnModelParams.end())
  {
    return true;
  }
  return false;
}

int IntraPredictionNN::getNbLinesIndividualDimension(const unsigned int dimension)
{
  return dimension > 8 ? dimension >> 1 : dimension;
}

void IntraPredictionNN::xPredIntraPnnContext(const CompArea& area, const CodingUnit& cu, const int unitHeight, const int unitWidth)
{
  const ChannelType chType = toChannelType(area.compID);
  const Position posLT = area;
  const CodingStructure& cs = *cu.cs;
  const WidthHeight pnnContextLength = IntraPredictionNN::getPnnCtxLength(area.width, area.height); 
  int availablePixelsAbove = 0, availablePixelsLeft = 0;
  for (int dx = area.width - unitWidth; dx < pnnContextLength.first; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);
    if (!cs.isDecomp(refPos, chType) || cs.getCURestricted(refPos, cu, chType) == nullptr)
    {
      break;
    }
    availablePixelsAbove = dx + unitWidth;
  }
  for (int dy = area.height - unitHeight; dy < pnnContextLength.second; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);
    if (!cs.isDecomp(refPos, chType) || cs.getCURestricted(refPos, cu, chType) == nullptr)
    {
      break;
    }
    availablePixelsLeft = dy + unitHeight;
  }
  const WidthHeight maxNbLines = IntraPredictionNN::getNbLines(area.width, area.height);
  int nbLinesAbove = 0, nbLinesLeft = 0;
  if (availablePixelsAbove > 0)
  {
    const int maxNbLinesAbove = maxNbLines.first;
    nbLinesAbove = unitHeight;
    for (int dy = maxNbLinesAbove; dy > unitHeight ; dy -= unitHeight)
    {
      const Position refPos = posLT.offset(0, -dy);
      if (cs.isDecomp(refPos, chType) && cs.getCURestricted(refPos, cu, chType) != nullptr)
      {
        nbLinesAbove = dy;
        break;
      }
    }
  }
  if (availablePixelsLeft > 0)
  {
    const int maxNbLinesLeft = maxNbLines.second;
    nbLinesLeft = unitWidth;
    for (int dx = maxNbLinesLeft; dx > unitWidth ; dx -= unitWidth)
    {
      const Position refPos = posLT.offset(-dx, 0);
      if (cs.isDecomp(refPos, chType) && cs.getCURestricted(refPos, cu, chType) != nullptr)
      {
        nbLinesLeft = dx;
        break;
      }
    }
  }
  setAvailableContext(availablePixelsAbove, availablePixelsLeft, nbLinesAbove, nbLinesLeft);
}

WidthHeight IntraPredictionNN::getNbLines(const int blockWidth, const int blockHeight)
{
  WidthHeight pairNbLines;
  if (blockWidth == 4 && blockHeight == 32)
  {
    pairNbLines = WidthHeight(8, 4);
  }
  else if (blockWidth == 32 && blockHeight == 4)
  {
    pairNbLines = WidthHeight(4, 8);
  }
  else
  if ((blockWidth <= 8 || blockHeight <= 8) && blockWidth * blockHeight < 256)
  {
    pairNbLines.first = pairNbLines.second = std::min(blockWidth, blockHeight);
  }
  else
  {
    pairNbLines.first = getNbLinesIndividualDimension(blockHeight);
    pairNbLines.second = getNbLinesIndividualDimension(blockWidth);
  }
  return pairNbLines;
}

WidthHeight IntraPredictionNN::getPnnCtxLength(const int blockWidth, const int blockHeight)
{
  return WidthHeight(blockWidth << 1, blockHeight << 1);
}

bool IntraPredictionNN::hasPnnPrediction(const CodingUnit& cu)
{
  const CompArea& area = cu.block(COMPONENT_Y);
  bool isContextAvailable = false;
  if (decideShapeHandledPnn(area.height, area.width))
  {
    const CodingStructure& cs = *cu.cs;
    const Position posLT = area;
    const WidthHeight pairNbLines = getNbLines(area.width, area.height);
    const int nbLinesAbove = static_cast<int>(pairNbLines.first);
    const int nbLinesLeft = static_cast<int>(pairNbLines.second);
    const Position refPos = posLT.offset(-nbLinesLeft, -nbLinesAbove);
    isContextAvailable = cs.getCURestricted(refPos, cu, CHANNEL_TYPE_LUMA) != nullptr;
  }
  return isContextAvailable;
}
  
bool IntraPredictionNN::isUpsamplingNeeded(const CompArea& area)
{
  const auto transfos = TransformationsContextPrediction(area.width, area.height);
  return transfos.factorDownsamplingVertical != 1 || transfos.factorDownsamplingHorizontal != 1;
}
  
bool IntraPredictionNN::isContextCollectionNeeded(const CompArea& area)
{
  return m_predModel->m_cachedCollectedArea != area && m_predModel->m_cachedPredictedArea != area;
}

void IntraPredictionNN::collectContextWidthHeightMasksInPixels(const CompArea& area, const int16_t* const piRoiOrigin, const int iPicStride, const int inputBitDepth, const CodingUnit& cu, const ComponentID compID)
{
  if (isContextCollectionNeeded(area))
  {
    const int tuHeight = static_cast<int>(area.height);
    const int tuWidth = static_cast<int>(area.width);
    if (compID != COMPONENT_Cr)
    {
      const SPS& sps = *(cu.cs)->sps;
      const PreCalcValues& pcv = *(cu.cs)->pcv;
      const bool noShift = pcv.noChroma2x2 && tuWidth == 4;
      const int unitWidth = tuWidth <= 2 && cu.ispMode && isLuma(compID) ? tuWidth : pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(compID, sps.getChromaFormatIdc()));
      const int unitHeight = tuHeight <= 2 && cu.ispMode && isLuma(compID) ? tuHeight : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(compID, sps.getChromaFormatIdc()));
      xPredIntraPnnContext(area, cu, unitHeight, unitWidth);
    }
    CHECK(inputBitDepth != m_predModel->m_inputBitdepth, "`inputBitDepth` is not equal to `m_predModel->m_inputBitdepth`.");
    CHECK(!m_enable, "`enable_` is false.");
    m_predModel->m_transfos = TransformationsContextPrediction(tuWidth, tuHeight);
    m_predModel->currentModelIndex = m_predModel->m_mapHeightWidthTargetModelIndex[m_predModel->m_transfos.pairHeightWidthAfterTr];
    const int nbBitshiftsScaling = g_inputQuantizer - m_predModel->m_inputBitdepth + 8 ;
    const int16_t maskValue = 0;

    extractZeroMeanContextPortions(piRoiOrigin, m_aboveLeftAvailableContext, m_aboveLeftAvailableNbLines, m_predModel->m_pnnModels[m_predModel->currentModelIndex].getInputData(), m_predModel->m_transfos, iPicStride, nbBitshiftsScaling, maskValue, m_predModel->m_meanContext, m_predModel->m_ctxLength);

    m_predModel->m_cachedCollectedArea = area;
  }
}
#endif


