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
#ifndef SADL_LAYER
#define SADL_LAYER
#include <vector>
#include <cstdint>
#include <iostream>
#include "tensor.h"

namespace sadl_vext
{
template<typename> class Model;   // fwd
namespace layers
{
// should be similar to python dumper:
struct OperationType
{
  enum Type
  {
    Const              = 1,    // important to have const first
    Placeholder        = 2,
    MatMul             = 6,
    Reshape            = 7,
    LeakyRelu          = 14,
    PReLU              = 21,
    OperationTypeCount = 30
  };
};

struct TensorInternalType
{
  enum Type
  {
    Int16 = 2,
  };
};

template<typename T> class Layer
{
public:
  using Id         = int32_t;
  using value_type = T;

  Layer(Id iid, OperationType::Type iop) : m_id(iid), m_op(iop) {}
  virtual ~Layer() = default;

  bool                      load(std::istream &file);
  Tensor<T> &               output();
protected:
  bool                      loadPrefix(std::istream &file);
  virtual bool              loadInternal(std::istream &file) = 0;
  Tensor<T>                 m_out;
  const Id                  m_id;
  const OperationType::Type m_op;
  std::vector<Id>           m_inputsId;
  bool                      m_initDone = false;
  template<typename> friend class sadl_vext::Model;
};

template<typename T> bool Layer<T>::load(std::istream &file) { return loadPrefix(file) && loadInternal(file); }

template<typename T> sadl_vext::Tensor<T> &Layer<T>::output() { return m_out; }

template<typename T> bool Layer<T>::loadPrefix(std::istream &file)
{
  m_initDone = false;
  int32_t length = 0;
  file.read((char *) &length, sizeof(int32_t));
  constexpr int maxLength = 2048;
  assert(length > 0 && length + 1 < maxLength);   // max name size
  char s[maxLength];
  file.read(s, length);
  s[length]  = '\0';

  file.read((char *) &length, sizeof(int32_t));
  assert(length >= 0 && length < 8);
  m_inputsId.resize(length);
  for (auto &x: m_inputsId)
  {
    file.read((char *) &x, sizeof(int32_t));
  }
  return static_cast<bool>(file);
}

template<typename T> class Placeholder : public Layer<T>
{
public:
  using Layer<T>::Layer;
  using Layer<T>::m_initDone;

  Dimensions   dims() const { return m_dims; }

protected:
  virtual bool loadInternal(std::istream &file) override;
  int          m_q = -1000;   // will override user input
  Dimensions   m_dims;        // can be use as a hint by user
};

template<typename T> bool Placeholder<T>::loadInternal(std::istream &file)
{
  int32_t x = 0;
  file.read((char *) &x, sizeof(x));
  if (x <= 0 || x > Dimensions::maxDim)
  {
    std::cerr << "[ERROR] invalid nb of dimensions: " << x << std::endl;
    return false;
  }
  m_dims.resize(x);
  file.read((char *) m_dims.begin(), sizeof(int) * x);
  // HACK
  if (m_dims.size() == 1)
  {
    x = m_dims[0];
    m_dims.resize(2);
    m_dims[0] = 1;
    m_dims[1] = x;
  }
  // END HACK
  file.read((char *) &m_q, sizeof(m_q));
  m_initDone = true;
  return true;
}

template<typename T> class Const : public Layer<T>
{
public:
  using Layer<T>::Layer;
  using Layer<T>::m_out;   // to avoid this->
  using Layer<T>::m_initDone;

protected:
  virtual bool              loadInternal(std::istream &file) override;
  template<typename U> void readTensor(std::istream &file, Tensor<T> &out, const int32_t sizeSparse, const int32_t packedSparsitySize);
};

template<typename T> template<typename U> void Const<T>::readTensor(std::istream &file, Tensor<T> &out, const int32_t sizeSparse, const int32_t packedSparsitySize)
{
  std::vector<T> &dataSparse = out.getDataSparse();
  std::vector<uint16_t> &indices = out.getIndices();
  std::vector<uint16_t> &nbNonzerosCol = out.getNbNonzerosCol();
  if (sizeSparse > 0)
  {
    file.read((char*)nbNonzerosCol.data(), sizeof(uint16_t) * nbNonzerosCol.size());
    file.read((char*)indices.data(), sizeof(uint16_t) * indices.size());
  }
  T *dstData = (sizeSparse > 0) ? dataSparse.data() : out.data();
  size_t sizeData = (sizeSparse > 0) ? dataSparse.size(): out.size();
  if (std::is_same<T, U>::value)
  {
    file.read((char *) dstData, sizeof(T) * sizeData);
  }
  else
  {
    std::vector<U> data(sizeData);
    file.read((char *) data.data(), sizeof(U) * sizeData);
    for( int k = 0; k < ( int ) data.size(); ++k )
    {
      dstData[ k ] = static_cast< T >( data[ k ] );
    }
  }
}

template<typename T> bool Const<T>::loadInternal(std::istream &file)
{
  // load values
  int32_t x = 0;
  file.read((char *) &x, sizeof(x));
  if (x <= 0 || x > Dimensions::maxDim)
  {
    std::cerr << "[ERROR] invalid nb of dimensions: " << x << std::endl;
    return false;
  }
  Dimensions d;
  d.resize(x);
  for (int k = 0; k < d.size(); ++k)
  {
    file.read((char *) &x, sizeof(x));
    d[k] = x;
  }

  if (d.numElements() >= Tensor<T>::kMaxSize)
  {
    std::cerr << "[ERROR] tensor too large? " << d.numElements() << std::endl;
    return false;
  }
  uint8_t isSparse = 0;
  int32_t sizeSparse = 0;
  int16_t packedSparsitySize = 1;
  file.read((char*)&isSparse, sizeof(uint8_t));
  if (isSparse != 0)
  {
    file.read((char*)&sizeSparse, sizeof(int32_t));
    file.read((char *)&packedSparsitySize, sizeof(packedSparsitySize));
  }
  bool transposed = false;
  file.read((char*)&transposed, sizeof(transposed));
  m_out.setTransposed(transposed);
  m_out.resize(d, sizeSparse, packedSparsitySize);

  file.read((char *) &x, sizeof(x));

  // cannot check internal type because tensor also used by reshape etc.
  switch (x)
  {
  case TensorInternalType::Int16:
    // assert((std::is_same<T, int16_t>::value));
    file.read((char *) &m_out.quantizer, sizeof(m_out.quantizer));
    readTensor<int16_t>(file, m_out, sizeSparse, packedSparsitySize);
    break;
  default:
    std::cerr << "[ERROR] unknown internal type " << x << std::endl;
    return false;
  }

  m_initDone = true;
  return true;
}

}   // namespace layers

}   // namespace sadl_vext
#endif
