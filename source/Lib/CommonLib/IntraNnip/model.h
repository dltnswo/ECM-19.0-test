/* The copyright in this software is being made available under the BSD
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

#ifndef SADL_MODEL
#define SADL_MODEL

#include <memory>
#include <vector>
#include <cstdint>
#include <iostream>
#include <string>
#include "layer.h"
#include "tensor.h"
#include "layer_matmul.h"
#include "layer_prelu.h"

namespace sadl_vext
{
// input Tensor<T> dims: depth, nb rows, nb col

template<typename T> class Model
{
public:
  struct LayerData
  {
    std::unique_ptr<layers::MatMul<T>> matMul;
    std::vector<Tensor<T> *>           matMulInputs;
    std::unique_ptr<layers::PReLU<T>>  pReLU;
    std::vector<Tensor<T> *>           pReLUInputs;
  };
private:
  std::vector<std::unique_ptr<layers::Layer<T>>> m_consts;
  std::vector<LayerData>                     m_data;
  int32_t                                    m_numInputs      = 0;
  std::vector<typename layers::Layer<T>::Id> m_idsInput, m_idsOutput;
  bool                                       m_initDone = false;
  bool                                       m_sparsityComputed = false;
  std::string                                m_info;

public:
  bool             load(std::istream &in);
  bool             init(std::vector<Tensor<T>> &in);
  bool             apply(std::vector<Tensor<T>> &in);   // change input for optiz
  const Tensor<T> &result(int idxOut = 0) const { return getLayer(m_idsOutput[idxOut]).layer->output(); }

  // aditionnal info
  const std::vector<typename layers::Layer<T>::Id> &getIdsOutput() const { return m_idsOutput; }
  size_t                                            nbOutputs() const { return m_idsOutput.size(); }
  LayerData&                                        getOutputLayer() { return m_data.back();}
  const std::string &                               info() const { return m_info; }
};

template<typename T> std::unique_ptr<layers::Layer<T>> createLayer(int32_t id, layers::OperationType::Type op)
{
  switch (op)
  {
  case layers::OperationType::Const:
    return std::unique_ptr<layers::Layer<T>>(new layers::Const<T>{ id, op });
    break;
  case layers::OperationType::Placeholder:
    return std::unique_ptr<layers::Layer<T>>(new layers::Placeholder<T>{ id, op });
    break;
  case layers::OperationType::MatMul:
    return std::unique_ptr<layers::Layer<T>>(new layers::MatMul<T>{ id, op });
    break;
  case layers::OperationType::LeakyRelu:
    return std::unique_ptr<layers::Layer<T>>(new layers::PReLU<T>{ id, op });
    break;
  default:
    std::cerr << "[ERROR] unknown layer " << op << std::endl;
    exit(-1);
    break;   // no default on purpose
  }
  std::cerr << "[ERROR] unknown layer " << op << std::endl;
  exit(-1);
}

template<typename T> bool Model<T>::load(std::istream &file)
{
  if (!file)
  {
    std::cerr << "[ERROR] Pb reading model" << std::endl;
    return false;
  }
  // reset model
  m_info.clear();

  // clear current object
  m_consts.clear();
  m_data.clear();
  m_numInputs = 0;
  m_idsInput.clear();
  m_idsOutput.clear();
  m_initDone = false;
  char magic[9];
  file.read(magic, 8);
  magic[8] = '\0';
  std::string magic_s = magic;

  int32_t x = 0;
  file.read( ( char* ) &x, sizeof( int32_t ) );
  if( std::is_same<T, int16_t>::value && x != layers::TensorInternalType::Int16 )
  {
    std::cerr << "[ERROR] wrong model type and Model<T> " << std::endl;
    return false;
  }

  int16_t y = 0;
  file.read( ( char* ) &y, sizeof( int16_t ) );
  if( y > 0 )
  {
    std::vector<char> v;
    v.resize( y + 1 );
    v[ y ] = 0;
    file.read( v.data(), sizeof( char ) * y );
    m_info = std::string{ v.data() };
  }

  int32_t numlayers = 0;
  file.read((char *) &numlayers, sizeof(int32_t));
  if ( numlayers <= 0 || numlayers > 8192)
  {
    std::cerr << "[ERROR] Pb reading model: nb layers " << numlayers << std::endl;
    return false;
  }

  int32_t nb;
  file.read( ( char* ) &nb, sizeof( int32_t ) );
  m_idsInput.resize( nb );
  file.read( ( char* ) m_idsInput.data(), sizeof( int32_t ) * nb );
  file.read( ( char* ) &nb, sizeof( int32_t ) );
  m_idsOutput.resize( nb );
  file.read( ( char* ) m_idsOutput.data(), sizeof( int32_t ) * nb );
  file.read((char*)&m_sparsityComputed, sizeof(m_sparsityComputed));

  for (int k = 0; k < numlayers; ++k)
  {
    typename layers::Layer<T>::Id id = 0;
    file.read((char *) &id, sizeof(int32_t));
    int32_t op = 0;
    file.read((char *) &op, sizeof(int32_t));
    if (!(op > 0 && op < layers::OperationType::OperationTypeCount))
    {
      std::cerr << "[ERROR] Pb reading model: layer op " << op << std::endl;
      return false;
    }
    if (op == layers::OperationType::Reshape)
    {
      continue;
    }
    if (op == layers::OperationType::MatMul)
    {
      m_data.push_back(LayerData());
      m_data.back().matMul = std::unique_ptr<layers::MatMul<T>>(new layers::MatMul<T>{ id, layers::OperationType::MatMul });
      m_data.back().matMulInputs.resize(2);
      m_data.back().matMulInputs[1] = &m_consts.back()->output();
      if (!m_data.back().matMul->load(file))
      {
        m_data.clear();
        return false;
      }
    }
    else if (op == layers::OperationType::PReLU || op == layers::OperationType::LeakyRelu)
    {
      m_data.back().pReLU = std::unique_ptr<layers::PReLU<T>>(new layers::PReLU<T>{ id, layers::OperationType::PReLU });
      m_data.back().pReLUInputs.resize(2);
      m_data.back().pReLUInputs[1] = &m_consts.back()->output();
      if (!m_data.back().pReLU->load(file))
      {
        m_data.clear();
        return false;
      }
    }
    else if (op == layers::OperationType::Const)
    {
      m_consts.push_back(createLayer<T>(id, (layers::OperationType::Type)op));
      if (!m_consts.back()->load(file))
      {
        m_data.clear();
        return false;
      }
#if __SSE4_2__ || __AVX2__
      if (m_consts.back()->output().isSparse())
      {
        m_consts.back()->output().checkSimdAlignment();
      }
#endif

    }
    else
    {
      auto tmpLayer = createLayer<T>(id, (layers::OperationType::Type)op);
      if (!tmpLayer->load(file))
      {
        m_data.clear();
        return false;
      }
    }
    }

  if (m_data.empty())
  {
    std::cerr << "[ERROR] Pb reading model: no layer" << std::endl;
    return false;
  }

  return true;
}

template<typename T> bool Model<T>::init(std::vector<Tensor<T>>& in)
{
  if (m_data.empty())
  {
    std::cerr << "[ERROR] Empty model" << std::endl;
    return false;
  }
  m_numInputs = (int)in.size();
  if (m_numInputs != (int)m_idsInput.size())
  {
    std::cerr << "[ERROR] inconsistent input dimension" << std::endl;
    return false;
  }
  bool ok = true;

  m_data[0].matMulInputs[0] = &in[0];
  ok &= m_data[0].matMul->initMatMul(m_data[0].matMulInputs);
  m_data[0].pReLUInputs[0] = &m_data[0].matMul->output();
  ok &= m_data[0].pReLU->initPreLU(m_data[0].pReLUInputs);

  m_data[1].matMulInputs[0] = &m_data[0].pReLU->output();
  ok &= m_data[1].matMul->initMatMul(m_data[1].matMulInputs);
  m_data[1].pReLUInputs[0] = &m_data[1].matMul->output();
  ok &= m_data[1].pReLU->initPreLU(m_data[1].pReLUInputs);

  m_data[2].matMulInputs[0] = &m_data[1].pReLU->output();
  ok &= m_data[2].matMul->initMatMul(m_data[2].matMulInputs);
  if (m_data.size() > 3)
  {
    m_data[2].pReLUInputs[0] = &m_data[2].matMul->output();
    ok &= m_data[2].pReLU->initPreLU(m_data[2].pReLUInputs);

    m_data[3].matMulInputs[0] = &m_data[2].pReLU->output();
    ok &= m_data[3].matMul->initMatMul(m_data[3].matMulInputs);
  }

  if( !ok )
  {
    return false;
  }
  m_initDone = true;
  return true;
}

template<typename T> bool Model<T>::apply(std::vector<Tensor<T>> &in)
{
  assert(!m_data.empty());
  assert((int) in.size() == m_numInputs);
  bool ok = true;

  ok &= m_data[0].matMul->applyMatMul(m_data[0].matMulInputs);
  ok &= m_data[0].pReLU->applyPreLU(m_data[0].pReLUInputs);

  ok &= m_data[1].matMul->applyMatMul(m_data[1].matMulInputs);
  ok &= m_data[1].pReLU->applyPreLU(m_data[1].pReLUInputs);
  ok &= m_data[2].matMul->applyMatMul(m_data[2].matMulInputs);
  if (m_data.size() > 3)
  {
    ok &= m_data[2].pReLU->applyPreLU(m_data[2].pReLUInputs);
    ok &= m_data[3].matMul->applyMatMul(m_data[3].matMulInputs);
  }
  return ok;
}

}   // namespace sadl_vext
#endif
