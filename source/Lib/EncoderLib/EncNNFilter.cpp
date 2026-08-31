/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
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

 /** \file     EncNNFilter.cpp
  \brief    cnn loop filter on the encoder side class
  */
#if JVET_AL0228_NNLF_INTERFACES

#include "EncNNFilter.h"
#include <sadl/model.h>

#if JVET_AB0068_AC0328_NNLF_RDO || NN_HOP_RDO
void EncNNFilter::createEnc(const int picWidth, const int picHeight, const int maxCuWidth, const int maxCuHeight, const ChromaFormat format, const int nnlfSet1NumParams)
{
  if (m_tempBuf.getOrigin(0) == NULL)
  {
    m_tempBuf.create(format, Area(0, 0, picWidth, picHeight));
  }
  if (m_tempRecBuf[0].getOrigin(0) == NULL)
  {
    m_tempRecBuf[0].create(format, Area(0, 0, maxCuWidth, maxCuHeight));
  }
  if (!m_tempPredBuf[0].getOrigin(0))
  {
    m_tempPredBuf[0].create(format, Area(0, 0, maxCuWidth, maxCuHeight));
  }
  if (!m_tempSplitBuf[0].getOrigin(0))
  {
    m_tempSplitBuf[0].create(format, Area(0, 0, maxCuWidth, maxCuHeight));
  }
}
void EncNNFilter::destroyEnc()
{
  m_tempBuf.destroy();
  m_tempRecBuf[0].destroy();
  m_tempPredBuf[0].destroy();
  m_tempSplitBuf[0].destroy();
}

#if JVET_AB0068_AC0328_NNLF_RDO || NN_HOP_RDO
void EncNNFilter::initEnc(const int numModels, const std::string& interLuma, const std::string& intraLuma)
{
  m_NumModelsValid = numModels;
#if JVET_AB0068_AC0328_NNLF_RDO
  m_interLumaRdNNFilter1 = interLuma;
  m_intraLumaRdNNFilter1 = intraLuma;
  m_interLumaRdNNFilter0 = interLuma;
  m_intraLumaRdNNFilter0 = intraLuma;
#endif
#if NN_HOP_RDO
  m_rdoNnlfHopModelName = interLuma;
#endif
}
#endif

template<typename T>
struct ModelData
{
  sadl::Model<T> model;
  vector<sadl::Tensor<T>> inputs;
  int hor,ver;
  bool luma,inter;
};

template<typename T>
static std::vector<ModelData<T>> initSpace()
{
  std::vector<ModelData<T>> v;
  v.reserve(5);
  return v;
}

#if NN_FIXED_POINT_IMPLEMENTATION
static std::vector<ModelData<int16_t>> modelsRd = initSpace<int16_t>();
#else
static std::vector<ModelData<float>> modelsRd = initSpace<float>();
#endif

template<typename T>
static ModelData<T> &getModelRd(int ver, int hor, bool luma, bool inter, const std::string modelName) {
  ModelData<T> *ptr = nullptr;
  for(auto &m: modelsRd)
  {
    if (m.luma == luma && m.inter == inter)
    {
      ptr = &m;
      break;
    }
  }
  if (ptr == nullptr)
  {
    if (modelsRd.size() == modelsRd.capacity())
    {
      THROW( "RDO increase cache" );
      exit(-1);
    }
    modelsRd.resize(modelsRd.size()+1);
    ptr = &modelsRd.back();
    ModelData<T> &m = *ptr;
    ifstream file(modelName, ios::binary);

    if (!file)
    {
      std::cerr << "[ERROR] unable to open NNFilter RDO " << modelName << std::endl;
      exit(-1);
    }
    m.model.load(file);
    m.luma = luma;
    m.inter = inter;
    m.ver = 0;
    m.hor = 0;
  }
  ModelData<T> &m = *ptr;
  if (m.ver != ver || m.hor != hor)
  {
    m.inputs = m.model.getInputsTemplate();
    if (luma)
    {
      for(auto &t: m.inputs)
      {
        sadl::Dimensions dims(std::initializer_list<int>({1, ver, hor, 1}));
        t.resize(dims);
      }
    }
    else if (inter) // inter chroma
    {
      int inputId = 0;
      for(auto &t: m.inputs)
      {
        if (t.dims()[3] == 1 && inputId != 3) // luma
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver, hor, 1}));
          t.resize(dims);
        }
        else if (t.dims()[3] == 1 && inputId == 3) // qp
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver/2, hor/2, 1}));
          t.resize(dims);
        }
        else
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver/2, hor/2, 2}));
          t.resize(dims);
        }
        inputId ++;
      }
    }
    else // intra chroma
    {
      int inputId = 0;
      for(auto &t: m.inputs)
      {
        if (t.dims()[3] == 1 && inputId != 4 ) // luma
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver, hor, 1}));
          t.resize(dims);
        }
        else if (t.dims()[3] == 1 && inputId == 4) // QP
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver/2, hor/2, 1}));
          t.resize(dims);
        }
        else
        {
          sadl::Dimensions dims(std::initializer_list<int>({1, ver/2, hor/2, 2}));
          t.resize(dims);
        }
        inputId ++;
      }
    }
    if (!m.model.init(m.inputs))
    {
      THROW( "RDO issue during initialization" );
      exit(-1);
    }
    m.ver = ver;
    m.hor = hor;
  }
  return m;
}

template<typename T>
void EncNNFilter::prepareInputsLumaRd (Picture* pic, UnitArea inferArea, vector<sadl::Tensor<T>> &inputs, int sliceQp, int baseQp, bool inter)
{
  double inputScale = 1024;
  bool nnlfSet0Flag = pic->cs->sps->getNnlfSet0EnabledFlag();
#if NN_FIXED_POINT_IMPLEMENTATION
  int shiftInput = nnlfSet0Flag ? 11 : NN_INPUT_PRECISION;
#else
  int shiftInput = 0;
#endif

  CompArea tempLumaArea = inferArea.Y();
  tempLumaArea.repositionTo(Position(0, 0));

  PelBuf bufRec = m_tempRecBuf[0].getBuf(tempLumaArea);
  PelBuf bufPred = m_tempPredBuf[0].getBuf(tempLumaArea);
  PelBuf bufPartition = m_tempSplitBuf[0].getBuf(tempLumaArea);

  sadl::Tensor<T>* inputRec, *inputPred, *inputSliceQp, *inputBaseQp, *inputPartition, *inputQp;

#if NN_HOP_RDO
  bool nnlfUnifiedFlag = pic->cs->sps->getNnlfUnifiedEnabledFlag();
  sadl::Tensor<T>* inputGlobalQp, *inputLocalQp;
  if (nnlfUnifiedFlag)
  {
    inputRec = &inputs[0];
    inputPred = &inputs[1];
    inputGlobalQp = &inputs[2];
    inputLocalQp = &inputs[3];
  }
  else
  {
#endif

    if (inter)
    {
      inputRec = &inputs[0];
      inputPred = &inputs[1];
      if (nnlfSet0Flag)
      {
        inputSliceQp = &inputs[2];
        inputBaseQp = &inputs[3];
      }
      else
      {
        inputQp = &inputs[2];
      }
    }
    else
    {
      inputRec = &inputs[0];
      inputPred = &inputs[1];
      if (nnlfSet0Flag)
      {
        inputBaseQp = &inputs[2];
      }
      else
      {
        inputPartition = &inputs[2];
        inputQp = &inputs[3];
      }
    }
#if NN_HOP_RDO
  }
#endif
  
  int hor = inferArea.lwidth();
  int ver = inferArea.lheight();
  
  for (int yy = 0; yy < ver; yy++)
  {
    for (int xx = 0; xx < hor; xx++)
    {
      (*inputRec)(0, yy, xx, 0) = (T)(bufRec.at(xx, yy) / inputScale * (1 << shiftInput));
      (*inputPred)(0, yy, xx, 0) = (T)(bufPred.at(xx, yy) / inputScale * (1 << shiftInput));

#if NN_HOP_RDO
      if (nnlfUnifiedFlag)
      {
        (*inputGlobalQp)(0, yy, xx, 0) = (T)(baseQp  / inputScale * (1 << shiftInput));
        (*inputLocalQp) (0, yy, xx, 0) = (T)(sliceQp / inputScale * (1 << shiftInput));
      }
      else
      {
#endif
        if (nnlfSet0Flag)
        {
          if (inter)
          {
            (*inputSliceQp)(0, yy, xx, 0) = (T)(sliceQp / inputScale * (1 << shiftInput));
          }
          (*inputBaseQp)(0, yy, xx, 0) = (T)(baseQp / inputScale * (1 << shiftInput));
        }
        else
        {
          (*inputQp)(0, yy, xx, 0) = (T)(baseQp / 64.0 * (1 << shiftInput));
          if (!inter)
          {
            (*inputPartition)(0, yy, xx, 0) = (T)(bufPartition.at(xx, yy) / inputScale * (1 << shiftInput));
          }
        }
#if NN_HOP_RDO
      }
#endif
    }
  }
}

template<typename T>
void EncNNFilter::extractOutputsLumaRd (Picture* pic, sadl::Model<T> &m, PelStorage& tempBuf, UnitArea inferArea)
{
#if NN_FIXED_POINT_IMPLEMENTATION
  int log2InputScale = 10;
  int log2OutputScale = 10;
  int shiftInput = pic->cs->sps->getNnlfSet0EnabledFlag() ? 11 - log2InputScale : NN_OUTPUT_PRECISION - log2OutputScale;
  int shiftOutput = pic->cs->sps->getNnlfSet0EnabledFlag() ? 11 - log2InputScale : NN_OUTPUT_PRECISION - log2OutputScale;
  int offset    = (1 << shiftOutput) / 2;
#else
  double inputScale = 1024;
  double outputScale = 1024;
#endif
  auto output = m.result(0);
  PelBuf bufDst = tempBuf.getBuf(inferArea).get(COMPONENT_Y);

  int hor = inferArea.lwidth();
  int ver = inferArea.lheight();
  
  CompArea tempLumaArea = inferArea.Y();
  tempLumaArea.repositionTo(Position(0, 0));

  PelBuf bufRec = m_tempRecBuf[0].getBuf(tempLumaArea);
  
  for (int c = 0; c < 4; c++) // output includes 4 sub images
  {
    for (int y = 0; y < ver >> 1; y++)
    {
      for (int x = 0; x < hor >> 1; x++)
      {
        int yy = (y << 1) + c / 2;
        int xx = (x << 1) + c % 2;
#if NN_FIXED_POINT_IMPLEMENTATION
        int out = ( output(0, y, x, c) + (bufRec.at(xx, yy) << shiftInput) + offset) >> shiftOutput;
#else
        int out = ( output(0, y, x, c) + bufRec.at(xx, yy) / inputScale ) * outputScale + 0.5;
#endif
        bufDst.at(xx, yy) = Pel(Clip3<int>( 0, 1023, out));
      }
    }
  }
}

template<typename T> 
void EncNNFilter::cnnFilterLumaBlockRd(Picture* pic, UnitArea inferArea, bool inter)
{
  // get model
  string modelName("");

#if JVET_AB0068_AC0328_NNLF_RDO
  bool nnlfSet0Flag = pic->cs->sps->getNnlfSet0EnabledFlag();
  string m_interLumaRd, m_intraLumaRd;
  m_interLumaRd = nnlfSet0Flag ? m_interLumaRdNNFilter0 : m_interLumaRdNNFilter1;
  m_intraLumaRd = nnlfSet0Flag ? m_intraLumaRdNNFilter0 : m_intraLumaRdNNFilter1;
  modelName = inter ? m_interLumaRd : m_intraLumaRd;
#endif
  
#if NN_HOP_RDO
  bool nnlfUnifiedFlag = pic->cs->sps->getNnlfUnifiedEnabledFlag();
  modelName = nnlfUnifiedFlag? m_rdoNnlfHopModelName : modelName;
#endif

  ModelData<T> &m = getModelRd<T>(inferArea.lheight(), inferArea.lwidth(), true, inter, modelName);
  sadl::Model<T> &model = m.model;
  
  // get inputs
  vector<sadl::Tensor<T>> &inputs = m.inputs;

  int baseQp = pic->slices[0]->getPPS()->getPicInitQPMinus26() + 26;
  int sliceQp = pic->slices[0]->getSliceQp();
  int paramIdx = 0;
  int delta = inter ? paramIdx * 5 : paramIdx * 2;
  if ( pic->slices[0]->getTLayer() >= 4 && paramIdx >= 2 )
  {
    delta = 5 - delta;
  }
  int qp = inter ? baseQp - delta : sliceQp - delta;
  
#if NN_HOP_RDO
  if (nnlfUnifiedFlag)
  {
    prepareInputsLumaRd<T>(pic, inferArea, inputs, sliceQp, baseQp, inter);
  }
  else
  {
#endif
#if JVET_AB0068_AC0328_NNLF_RDO
    if (nnlfSet0Flag)
    {
      prepareInputsLumaRd<T>(pic, inferArea, inputs, sliceQp, baseQp, inter);
    }
    else
    {
      prepareInputsLumaRd<T>(pic, inferArea, inputs, sliceQp, qp, inter);
    }
#endif 
#if NN_HOP_RDO
  }
#endif
  
  // inference
  if (!model.apply(inputs))
  {
    THROW( "RDO issue during luma model inference" );
    exit(-1);
  }
  
  // get outputs
  extractOutputsLumaRd<T>(pic, model, m_tempBuf, inferArea);
}

void EncNNFilter::cnnFilterLumaBlockRd_ext(Picture* pic, UnitArea inferArea, bool inter)
{
#if NN_FIXED_POINT_IMPLEMENTATION
  cnnFilterLumaBlockRd<int16_t>(pic, inferArea, inter);
#else
  cnnFilterLumaBlockRd<float>(pic, inferArea, inter);
#endif
}

#endif

#endif
