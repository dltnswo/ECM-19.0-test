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
#ifndef SADL_TENSOR
#define SADL_TENSOR
#include <algorithm>
#include <cstdlib>
#if _WIN32 || __USE_ISOC11
#include <malloc.h>
#else
#include <malloc/malloc.h>
#endif
#include <numeric>
#include <vector>
#include <limits>
#include <utility>

namespace sadl_vext
{
  struct Dimensions
  {
    static constexpr int maxDim = 6;
    using iterator = int*;
    using const_iterator = const int*;

    Dimensions() = default;
    Dimensions( std::initializer_list<int> L )
    {
      assert( ( int ) L.size() <= maxDim );
      m_s = ( int ) L.size();
      std::copy( L.begin(), L.end(), m_v );
    }

    void resize( int s )
    {
      assert( s <= maxDim );
      m_s = s;
    }
    int     size() const { return m_s; }
    int64_t numElements() const
    {
#if __GNUC__ && !__clang__
#pragma GCC diagnostic push                             // from gcc 12... to remove later 
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
      return std::accumulate( m_v, m_v + m_s, ( int64_t ) 1, []( int64_t a, int64_t b ) { return a * b; } );
#if __GNUC__ && !__clang__
#pragma GCC diagnostic pop
#endif
    }
    int            operator[]( int k ) const { return m_v[ k ]; }
    int& operator[]( int k ) { return m_v[ k ]; }
    iterator       begin() { return m_v; }
    iterator       end() { return m_v + m_s; }
    const_iterator begin() const { return m_v; }
    const_iterator end() const { return m_v + m_s; }
    bool           operator==( const Dimensions& d ) const { return d.m_s == m_s && std::equal( m_v, m_v + m_s, d.m_v ); }
    int            back() const { return m_v[ m_s - 1 ]; }

  private:
    int m_v[ maxDim ] = {};
    int m_s = 0;
  };

// tensor between layers: depth height width (or width height?)
template<typename T, std::size_t Alignment> struct aligned_allocator
{
  using pointer         = T *;
  using const_pointer   = const T *;
  using reference       = T &;
  using const_reference = const T &;
  using value_type      = T;
  using size_type       = std::size_t;
  using difference_type = std::ptrdiff_t;

  pointer       address(reference r) const { return &r; }
  const_pointer address(const_reference s) const { return &s; }
  size_type     maxSize() const { return (static_cast<std::size_t>(0) - static_cast<std::size_t>(1)) / sizeof(T); }
  template<typename U> struct rebind
  {
    typedef aligned_allocator<U, Alignment> other;
  };

  bool operator!=(const aligned_allocator &other) const { return !(*this == other); }
  void construct(pointer p, const_reference t) const
  {
    void *const pv = static_cast<void *>(p);
    new (pv) T(t);
  }
  void destroy(T *const p) const { p->~T(); }
  bool operator==(const aligned_allocator & /*other*/) const { return true; }

  aligned_allocator()                          = default;
  aligned_allocator(const aligned_allocator &) = default;
  ~aligned_allocator()                         = default;
  aligned_allocator &operator=(const aligned_allocator &) = delete;

  template<typename U> aligned_allocator(const aligned_allocator<U, Alignment> &) {}

  pointer allocate(const std::size_t n) const
  {
    if (n == 0)
      return nullptr;
    size_t s = ((n * sizeof(T) + Alignment - 1) / Alignment) * Alignment;

#if _WIN32
#if __MINGW32__
    void *const pv = __mingw_aligned_malloc(s, Alignment);
#else
    void *const pv = _aligned_malloc(s, Alignment);
#endif
#else
#if __USE_ISOC11
    void *const pv = aligned_alloc(Alignment, s);
#else
    void *pv = nullptr;
    if (posix_memalign(&pv, Alignment, s))
    {
      throw std::bad_alloc();
    }
#endif
#endif

    if (!pv)
      throw std::bad_alloc();
    return static_cast<T *>(pv);
  }

#ifdef _WIN32
  void deallocate(T *const p, const std::size_t n) const { _aligned_free(p); }
#else
  void deallocate(T *const p, const std::size_t /*n*/) const { free(p); }
#endif

  template<typename U> pointer allocate(const std::size_t n, const U * /* const hint */) const { return allocate(n); }
};

template<typename T> struct ComputationType
{
};

// predecl for friendness
template<typename T> class Tensor;
template<typename T> void swap(Tensor<T> &t0, Tensor<T> &t1);
template<typename T> void swapData(Tensor<T> &t0, Tensor<T> &t1);

template<typename T> class Tensor
{
public:
  using value_type     = T;
  using Data           = std::vector<value_type, aligned_allocator<value_type, 64>>;
  using iterator       = typename Data::iterator;
  using const_iterator = typename Data::const_iterator;
  using index = uint16_t;

  Tensor() = default;
  explicit Tensor(Dimensions d);

  void resize(Dimensions d, int32_t sizeSparse = 0, int32_t packedSparsitySize = 1);

  // linear access
  value_type &operator[](int i);
  value_type  operator[](int i) const;

  // tensor access
  value_type &operator()(int i);
  value_type  operator()(int i) const;

  value_type &operator()(int i, int j);
  value_type  operator()(int i, int j) const;

  value_type &operator()(int i, int j, int k);

  bool in(int i) const;
  bool in(int i, int j) const;

  const Dimensions &dims() const;
  int64_t           size() const;

  const value_type *data() const { return m_data.data(); }
  value_type *      data() { return m_data.data(); }

  iterator begin()
  {
    assert(!isSparse());
    return m_data.begin();
  }
  const_iterator begin() const { return m_data.begin(); }
  iterator       end()
  {
    assert(!isSparse());
    return m_data.end();
  }
  const_iterator end() const { return m_data.end(); }

  int                      quantizer   = 0;   // for int
  static constexpr int64_t kMaxSize    = 32LL * 1024 * 1024 * 1024;

  Data &getData() { return m_data; }
  void                           setTransposed(const bool val) { m_transposed = val; }
  bool                           getTransposed() const { return m_transposed; }
  std::vector<value_type> &getDataSparse() { return m_dataSparse; }
  const std::vector<value_type> &getDataSparse() const { return m_dataSparse; }
  const std::vector<index> &     getIndices() const { return m_indices; }
  std::vector<index> &           getIndices() { return m_indices; }
  const std::vector<uint16_t> &  getNbNonzerosCol() const { return m_numNonzeroCol; }
  std::vector<uint16_t> &        getNbNonzerosCol() { return m_numNonzeroCol; }
  bool                           isSparse() const { return !m_dataSparse.empty(); }
  bool                           isSparsePack8() const { return !m_dataSparse.empty() && m_packedSparsitySize == 8; }
  bool                           isSparsePack16() const { return !m_dataSparse.empty() && m_packedSparsitySize == 16; }
  int16_t                        getPackedSparsitySize() const { return m_packedSparsitySize; }
#if __SSE4_2__ || __AVX2__
  void                           checkSimdAlignment();
#endif
private:
  Dimensions m_dims;
  Data       m_data;
  bool                    m_transposed = false;
  int16_t                 m_packedSparsitySize = 1;
  std::vector<value_type> m_dataSparse;
  std::vector<index>      m_indices;
  std::vector<uint16_t>   m_numNonzeroCol;

  friend void swap<>(Tensor<T> &t0, Tensor<T> &t1);
  friend void swapData<>(Tensor<T> &t0, Tensor<T> &t1);
};

// Sparse Matmul
#if __SSE4_2__ || __AVX2__
template<typename T> void Tensor<T>::checkSimdAlignment()
{
  assert(dims()[0] < (1 << 16) && dims()[0] < (1 << 16));

#if __AVX2__
  const int pad = 16;
  const int padShift = 4;
#else
  const int pad = 8;
  const int padShift = 3;
#endif

  uint64_t totalWeights = 0;
  bool alreadyAligned = true;
  uint64_t totalnbAligned = 0;

  for (auto nunNonzero : getNbNonzerosCol())
  {
    totalWeights += nunNonzero;
    if ((nunNonzero % pad) != 0)
    {
      alreadyAligned = false;
    }
    totalnbAligned += ((nunNonzero + (pad - 1)) >> padShift) << padShift;
  }
  if (alreadyAligned)
  {
    std::cout << "already aligned number " << std::endl;
  }
  else if (totalnbAligned == m_dataSparse.size())
  {
    std::cout << "data seems already padded " << std::endl;
  }
  else if (totalWeights != m_dataSparse.size())
  {
    std::cout << "WARNING: unknown padding have already been processed" << std::endl;
  }
  else
  {
    std::vector<T> tmpDataSparse;
    std::vector<index>      tmpIndices;

    T* bptr = getDataSparse().data();
    const auto *   idx  = getIndices().data();
    for (auto nunNonzero : getNbNonzerosCol())
    {
      if (nunNonzero > 0)
      {
        tmpDataSparse.insert(tmpDataSparse.end(), bptr, bptr + nunNonzero);
        tmpIndices.insert(tmpIndices.end(), idx, idx + (nunNonzero + m_packedSparsitySize - 1) / m_packedSparsitySize);

        int nbPad = (((nunNonzero + (pad - 1)) >> padShift) << padShift) - nunNonzero;
        tmpDataSparse.insert(tmpDataSparse.end(), nbPad, 0);
        int nbPadIdx = ((((nunNonzero + (pad - 1)) >> padShift) << padShift) - nunNonzero) / m_packedSparsitySize;
        tmpIndices.insert(tmpIndices.end(), nbPadIdx, tmpIndices.back());
        bptr += nunNonzero;
        idx += (nunNonzero + m_packedSparsitySize - 1) / m_packedSparsitySize;
      }
    }      
    std::swap(tmpDataSparse, m_dataSparse);
    std::swap(tmpIndices, m_indices);
  }
}
#endif

template<> struct ComputationType<int16_t>
{
  using type                = int32_t;
  static constexpr type max = std::numeric_limits<int16_t>::max();
  static void           quantize(type &z, int q) { z >>= q; }
  static void           shiftLeft(type &z, int q) { z <<= q; }
  static void           quantize(int16_t &z, int q) { z >>= q; }
  static void           shiftLeft(int16_t &z, int q) { z <<= q; }
};

template<typename T> void swap(Tensor<T> &t0, Tensor<T> &t1)
{
  assert(!t0.isSparse() && !t1.isSparse());
  std::swap(t0.m_dims, t1.m_dims);
  std::swap(t0.m_data, t1.m_data);
  std::swap(t0.quantizer, t1.quantizer);
}

template<typename T> void swapData(Tensor<T> &t0, Tensor<T> &t1)
{
  assert(!t0.isSparse() && !t1.isSparse());
  assert(t0.size() == t1.size());
  std::swap(t0.m_data, t1.m_data);
  std::swap(t0.quantizer, t1.quantizer);
}

template<typename T> Tensor<T>::Tensor(Dimensions d)
{
  assert(!isSparse());
  resize(d);
}

template<typename T> const Dimensions &Tensor<T>::dims() const { return m_dims; }

template<typename T> int64_t Tensor<T>::size() const { return m_data.size(); }

template<typename T> void Tensor<T>::resize(Dimensions d, int32_t sizeSparse, int32_t packedSparsitySize)
{
  m_dims     = d;
  int64_t m = m_dims.numElements();
  assert(m < kMaxSize);
  m_dataSparse.clear();
  if (sizeSparse > 0)
  {
    m_data.clear();
    m_dataSparse.resize(sizeSparse);
    m_indices.resize(sizeSparse / packedSparsitySize);
    m_packedSparsitySize = packedSparsitySize;
    m_numNonzeroCol.resize(d[1]);
  }
  else
  {
    m_data.resize(m);
  }
}

// TODO: variadic template to define all accesors
template<typename T> T &Tensor<T>::operator[](int i)
{
  assert(!isSparse());
  return m_data[i];
}

template<typename T> T &Tensor<T>::operator()(int i)
{
  assert(!isSparse());
  assert(m_dims.size() == 1);
  assert(i < m_dims[0] && i >= 0);

  return m_data[i];
}

template<typename T> bool Tensor<T>::in(int i) const { return m_dims.size() == 1 && i < m_dims[0] && i >= 0; }

template<typename T> T Tensor<T>::operator[](int i) const { return m_data[i]; }

template<typename T> T Tensor<T>::operator()(int i) const
{
  assert(m_dims.size() == 1);
  assert(i < m_dims[0] && i >= 0);

  return m_data[i];
}

template<typename T> T &Tensor<T>::operator()(int i, int j)
{
  assert(!isSparse());
  assert(m_dims.size() == 2);
  assert(i < m_dims[0] && i >= 0);
  assert(j < m_dims[1] && j >= 0);

  return m_data[(int64_t) m_dims[1] * i + j];
}

template<typename T> T Tensor<T>::operator()(int i, int j) const
{
  assert(m_dims.size() == 2);
  assert(i < m_dims[0] && i >= 0);
  assert(j < m_dims[1] && j >= 0);

  return m_data[(int64_t) m_dims[1] * i + j];
}

template<typename T> bool Tensor<T>::in(int i, int j) const { return m_dims.size() == 2 && i < m_dims[0] && i >= 0 && j < m_dims[1] && j >= 0; }

template<typename T> T &Tensor<T>::operator()(int i, int j, int k)
{
  assert(!isSparse());
  assert(m_dims.size() == 3);
  assert(i < m_dims[0] && i >= 0);
  assert(j < m_dims[1] && j >= 0);
  assert(k < m_dims[2] && k >= 0);

  return m_data[(int64_t) m_dims[2] * (m_dims[1] * i + j) + k];
}

}   // namespace sadl_vext

#endif
