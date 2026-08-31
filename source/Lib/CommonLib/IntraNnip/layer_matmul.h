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
#ifndef SADL_LAYER_MATMUL
#define SADL_LAYER_MATMUL
#include "layer.h"
#if __AVX2__ || __SSE4_2__
#include <immintrin.h>
#endif
#if __AVX2__
#include <immintrin.h>
#endif

#define SATURATE(X) \
  X = (X > ComputationType<T>::max) ? ComputationType<T>::max : (X < -ComputationType<T>::max ? -ComputationType<T>::max : X)

#if __AVX2__
static inline typename sadl_vext::ComputationType<int16_t>::type hsum_epi32_avx(__m128i x)
{
  __m128i hi64 = _mm_unpackhi_epi64(x, x);   // 3-operand non-destructive AVX lets us save a
  // byte without needing a movdqa
  __m128i sum64 = _mm_add_epi32(hi64, x);
  __m128i hi32  = _mm_shuffle_epi32(sum64, _MM_SHUFFLE(2, 3, 0, 1));   // Swap the low two elements
  __m128i sum32 = _mm_add_epi32(sum64, hi32);
  return _mm_cvtsi128_si32(sum32);   // movd
}

static inline typename sadl_vext::ComputationType<int16_t>::type sum32_int16(__m256i x)
{
  __m128i sum128 = _mm_add_epi32(_mm256_castsi256_si128(x), _mm256_extracti128_si256(x, 1));
  return hsum_epi32_avx(sum128);
}
#endif
#if __SSE4_2__ || USE_SSE42
static inline typename sadl_vext::ComputationType<int16_t>::type sum32_int16(__m128i s)
{
  __m128i hi64  = _mm_unpackhi_epi64(s, s);   // 3-operand non-destructive AVX lets us save a byte without needing a movdqa
  __m128i sum64 = _mm_add_epi32(hi64, s);
  __m128i hi32  = _mm_shuffle_epi32(sum64, _MM_SHUFFLE(2, 3, 0, 1));   // Swap the low two elements
  __m128i sum32 = _mm_add_epi32(sum64, hi32);

  typename sadl_vext::ComputationType<int16_t>::type z = _mm_cvtsi128_si32(sum32);
  return z;
}
#endif

namespace sadl_vext
{
namespace layers
{
template<typename T> class MatMul : public Layer<T>
{
public:
  using Layer<T>::Layer;
  using Layer<T>::m_out;   // to avoid this->
  using Layer<T>::m_initDone;

  bool initMatMul(const std::vector<Tensor<T> *> &in);
  bool applyMatMul(std::vector<Tensor<T>*>& in);

protected:
  virtual bool          loadInternal(std::istream &file) override;
  template<int NN> bool applyDim2(std::vector<Tensor<T> *> &in);

#if __AVX2__
  bool applyDim2Simd8(std::vector<Tensor<T> *> &in) { return applyDim2<8>(in); }
  bool applyDim2Simd16(std::vector<Tensor<T> *> &in) { return applyDim2Simd8(in); }
#endif

  bool applySparseMatMul(std::vector<Tensor<T> *> &in);
  bool applySparsePackMatMul(std::vector<Tensor<T> *> &in);
#if __AVX2__
  bool applySparseMatMulSimd16(std::vector<Tensor<T> *> &in);
  bool applySparsePack8MatMulSimd16(std::vector<Tensor<T> *> &in);
  bool applySparsePack16MatmulSimd16(std::vector<Tensor<T> *> &in);
#endif
#if __SSE4_2__ || USE_SSE42
  bool applySparseMatMulSimd8(std::vector<Tensor<T> *> &in);
  bool applySparsePack8MatMulSimd8(std::vector<Tensor<T> *> &in);
#endif

  int m_q = 0;
};

template<typename T> bool MatMul<T>::applyMatMul(std::vector<Tensor<T> *> &in)
{
  assert(in.size() == 2);
#if __AVX2__
#define MULT8_DIM2 applyDim2Simd8
#define MULT16_DIM2 applyDim2Simd16
#else
#define MULT8_DIM2 applyDim2<8>
#define MULT16_DIM2 applyDim2<16>
#endif

#if __AVX2__
#define SPARSE_MATMULT applySparseMatMulSimd16
#define SPARSE_MATMULT_PACK8 applySparsePack8MatMulSimd16
#define SPARSE_MATMULT_PACK16 applySparsePack16MatmulSimd16
#elif __SSE4_2__ || USE_SSE42
#define SPARSE_MATMULT applySparseMatMulSimd8
#define SPARSE_MATMULT_PACK8 applySparsePack8MatMulSimd8
#define SPARSE_MATMULT_PACK16 applySparsePackMatMul
#else
#define SPARSE_MATMULT applySparseMatMul
#define SPARSE_MATMULT_PACK8 applySparsePackMatMul
#define SPARSE_MATMULT_PACK16 applySparsePackMatMul
#endif

  const Tensor<T> &A{ *in[0] };
  m_out.quantizer = A.quantizer - m_q;
  assert(m_out.quantizer >= 0);
  assert(in[1]->quantizer + m_q >= 0);
  assert(A.dims().size() == 2);
  // cases:
  // A: always a tensor
  // B: tensor or const
  // 1- A [x] B[x] || A [x,y] B[y,z] || A [x,y,z] B[x,z,t]
  // 2- A [1,x] B[x] || A [1,x,y] B[y,z] || A [1,x,y,z] B[x,z,t]
  const int H{ A.dims().back() };   // to be changed if SIMD for more than dim1 and dim2

  if( in[ 1 ]->isSparsePack16() )
  {
    return SPARSE_MATMULT_PACK16( in );
  }
  else if( in[ 1 ]->isSparsePack8() )
  {
    return SPARSE_MATMULT_PACK8( in );
  }
  else if( in[ 1 ]->isSparse() )
  {
    return SPARSE_MATMULT( in );
  }
  else if( H % 16 == 0 )
  {
    return MULT16_DIM2( in );
  }
  else if( H % 8 == 0 )
  {
    return MULT8_DIM2( in );
  }
  else
  {
    return applyDim2<1>( in );
  }
}

template<typename T> template<int NN> bool MatMul<T>::applyDim2(std::vector<Tensor<T> *> &in)
{
  const Tensor<T> &A{ *in[0] };
  const Tensor<T> &B{ *in[1] };
  const int        shift{ in[1]->quantizer + m_q };
  const int        last = A.dims().size() - 1;
  const int        N{ A.dims()[last - 1] };
  const int        H{ (A.dims()[last] / NN) * NN };
  const int        R{ B.dims().back() };
  constexpr int idx_start{ 0 };
  const int     idx_end{ H };
  if (A.dims().size() == 2)
  {
    for (int b = 0; b < N; ++b)
    {
      const T *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
      for (int t = 0; t < R; ++t)
      {
        typename ComputationType<T>::type x = 0;
        const T *bptr = B.data() + t * H;   // T * i + t  (i, t); => B[t*H+i] if transposed
        {
          for (int i = idx_start; i < idx_end; ++i)
          {
            x += (typename ComputationType<T>::type) aptr[i] * bptr[i];   // A(b,i)*B(i, t);
          }
        }
        ComputationType<T>::quantize(x, shift);
        SATURATE(x);
        m_out(b, t) = (T) x;
      }
    }
  }
  else
  {
    for (int b = 0; b < N; ++b)
    {
      const T *aptr = A.data() + H * b;   // A(0,b,i)  => A[H*b]
      for (int t = 0; t < R; ++t)
      {
        typename ComputationType<T>::type x = 0;
        const T *bptr = B.data() + t * H;   // T * i + t  (i, t); => B[t*H+i] if transposed
        {
          for (int i = idx_start; i < idx_end; ++i)
          {
            x += (typename ComputationType<T>::type) aptr[i] * bptr[i];   // A(0,b,i)*B(i, t);
          }
        }
        ComputationType<T>::quantize(x, shift);
        SATURATE(x);
        m_out(0, b, t) = (T) x;
      }
    }
  }
  return true;
}

template<typename T> bool MatMul<T>::applySparseMatMul(std::vector<Tensor<T> *> &in)
{
  const Tensor<T> &A{ *in[0] };
  const Tensor<T> &B{ *in[1] };
  const int        shift{ in[1]->quantizer + m_q };
  const int        last = A.dims().size() - 1;
  const int        N{ A.dims()[last - 1] };
  const int        H{ A.dims()[last] };

  uint32_t offsetData = 0;
  uint16_t i           = 0;

  if (A.dims().size() == 2)
  {
    for (int b = 0; b < N; ++b)
    {
      const T *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]

      for (const auto &nunNonzero: B.getNbNonzerosCol())
      {
        typename ComputationType<T>::type x = 0;

        for (auto k = 0; k < nunNonzero; ++k, ++offsetData)
        {
          uint16_t j = B.getIndices()[offsetData];
          x += (typename ComputationType<T>::type) aptr[j] * B.getDataSparse()[offsetData];
        }

        ComputationType<T>::quantize(x, shift);
        SATURATE(x);
        m_out(b, i) = (T) x;
        i++;
      }
    }
  }
  else
  {
    for (int b = 0; b < N; ++b)
    {
      const T *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]

      for (const auto &nunNonzero: B.getNbNonzerosCol())
      {
        typename ComputationType<T>::type x = 0;

        for (uint16_t k = 0; k < nunNonzero; ++k, ++offsetData)
        {
          uint16_t j = B.getIndices()[offsetData];
          x += (typename ComputationType<T>::type) aptr[j] * B.getDataSparse()[offsetData];
        }

        ComputationType<T>::quantize(x, shift);
        SATURATE(x);
        m_out(0, b, i) = (T) x;
        i++;
      }
    }
  }

  return true;
}
template<typename T> bool MatMul<T>::applySparsePackMatMul(std::vector<Tensor<T> *> &in)
{
  const Tensor<T> &A{ *in[0] };
  const Tensor<T> &B{ *in[1] };
  const int        shift{ in[1]->quantizer + m_q };
  const int        last = A.dims().size() - 1;
  const int        N{ A.dims()[last - 1] };
  const int        H{ A.dims()[last] };

  uint32_t offsetData = 0;
  uint16_t i = 0;
#if __AVX2__
  const int pad = 16;
  const int padShift = 4;
#elif __SSE4_2__ || USE_SSE42
  const int pad = 8;
  const int padShift = 3;
#endif

  int packedSparsitySize = B.getPackedSparsitySize();
  const auto* idx = B.getIndices().data();
  for (int b = 0; b < N; ++b)
  {
    const T* aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    for (const auto& nunNonzero : B.getNbNonzerosCol())
    {
      typename ComputationType<T>::type x = 0;

      for (auto k = 0; k < nunNonzero / packedSparsitySize; k++, offsetData += packedSparsitySize)
      {
        uint16_t j = idx[k];
        for (auto p = 0; p < packedSparsitySize; ++p)
        {
          x += (typename ComputationType<T>::type) aptr[j + p] * B.getDataSparse()[offsetData + p];
        }
      }

      ComputationType<T>::quantize(x, shift);
      SATURATE(x);
      m_out(b, i) = (T)x;
      i++;
      idx += nunNonzero / packedSparsitySize;
#if (__SSE4_2__ || USE_SSE42 || __AVX2__)
      if (std::is_same<T, int16_t>::value)
      {
        int aligning = (((nunNonzero + (pad - 1)) >> padShift) << padShift) - nunNonzero;
        idx += aligning / packedSparsitySize;
        offsetData += aligning;
      }
#endif
    }
  }

  return true;
}

#if __SSE4_2__ || USE_SSE42
template<> inline bool MatMul<int16_t>::applySparseMatMulSimd8(std::vector<Tensor<int16_t> *> &in)
{
  using T = int16_t;

  const Tensor<int16_t> &A{ *in[0] };
  const Tensor<int16_t> &B{ *in[1] };
  const int              shift{ in[1]->quantizer + m_q };
  const int              last = A.dims().size() - 1;
  const int              N{ A.dims()[last - 1] };
  const int              H{ A.dims()[last] };

  int t = 0;

  for (int b = 0; b < N; ++b)
  {
    const int16_t *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    const int16_t *bptr = B.getDataSparse().data();
    const auto *   idx  = B.getIndices().data();

    for (const auto &nunNonzero: B.getNbNonzerosCol())
    {
      __m128i s = _mm_setzero_si128();

      for (int j = 0; j < nunNonzero; j += 8)
      {
        int16_t eA[8];

        for (int k = 0; k < 8; ++k)
        {
          eA[k] = *(aptr + *idx);
          idx++;
        }

        __m128i a  = _mm_loadu_si128((__m128i *) eA);
        __m128i b  = _mm_loadu_si128((const __m128i *) bptr);
        __m128i ab = _mm_madd_epi16(a, b);   // res in si32

        s = _mm_add_epi32(s, ab);

        bptr += 8;
      }

      __m128i hi64  = _mm_unpackhi_epi64(s, s);   // 3-operand non-destructive AVX lets us save a byte without needing a movdqa
      __m128i sum64 = _mm_add_epi32(hi64, s);
      __m128i hi32  = _mm_shuffle_epi32(sum64, _MM_SHUFFLE(2, 3, 0, 1));   // Swap the low two elements
      __m128i sum32 = _mm_add_epi32(sum64, hi32);

      typename ComputationType<int16_t>::type z = _mm_cvtsi128_si32(sum32);

      ComputationType<int16_t>::quantize(z, shift);
      SATURATE(z);
      m_out[t] = z;

      t++;
    }
  }
  return true;
}
template<> inline bool MatMul<int16_t>::applySparsePack8MatMulSimd8(std::vector<Tensor<int16_t> *> &in)
{
  using T = int16_t;

  const Tensor<int16_t> &A{ *in[0] };
  const Tensor<int16_t> &B{ *in[1] };
  const int              shift{ in[1]->quantizer + m_q };
  const int              last = A.dims().size() - 1;
  const int              N{ A.dims()[last - 1] };
  const int              H{ A.dims()[last] };

  int t = 0;

  for (int b = 0; b < N; ++b)
  {
    const int16_t *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    const int16_t *bptr = B.getDataSparse().data();
    const auto *   idx  = B.getIndices().data();

    for (auto nunNonzero: B.getNbNonzerosCol())
    {
      __m128i s = _mm_setzero_si128();

      for (int j = 0; j < nunNonzero; j += 8)
      {
        __m128i a  = _mm_loadu_si128((const __m128i *) (aptr + *idx));
        __m128i b  = _mm_loadu_si128((const __m128i *) bptr);
        __m128i ab = _mm_madd_epi16(a, b);

        s = _mm_add_epi32(s, ab);

        bptr += 8;
        idx ++;
      }

      typename ComputationType<int16_t>::type z = sum32_int16(s);
      ComputationType<int16_t>::quantize(z, shift);
      SATURATE(z);
      m_out[t] = z;

      t++;
    }
  }

  return true;
}
#endif

#if __AVX2__
template<> inline bool MatMul<int16_t>::applySparseMatMulSimd16(std::vector<Tensor<int16_t> *> &in)
{
  using T = int16_t;

  const Tensor<int16_t> &A{ *in[0] };
  const Tensor<int16_t> &B{ *in[1] };
  const int              shift{ in[1]->quantizer + m_q };
  const int              last = A.dims().size() - 1;
  const int              N{ A.dims()[last - 1] };
  const int              H{ A.dims()[last] };


  int t = 0;

  for (int b = 0; b < N; ++b)
  {
    const int16_t *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    const int16_t *bptr = B.getDataSparse().data();
    const auto *   idx  = B.getIndices().data();

    for (auto nunNonzero: B.getNbNonzerosCol())
    {
      __m256i s = _mm256_setzero_si256();

      for (int j = 0; j < nunNonzero; j += 16)
      {
        int16_t eA[16];

        for (int k = 0; k < 16; ++k)
        {
          eA[k] = *(aptr + *idx);
          idx++;
        }

        __m256i a  = _mm256_loadu_si256((__m256i *) eA);
        __m256i b  = _mm256_loadu_si256((const __m256i *) bptr);
        __m256i ab = _mm256_madd_epi16(a, b);

        s = _mm256_add_epi32(s, ab);

        bptr += 16;
      }

      typename ComputationType<int16_t>::type z = sum32_int16(s);
      ComputationType<int16_t>::quantize(z, shift);
      SATURATE(z);
      m_out[t] = z;

      t++;
    }
  }

  return true;
}

template<> inline bool MatMul<int16_t>::applySparsePack8MatMulSimd16(std::vector<Tensor<int16_t> *> &in)
{
  using T = int16_t;

  const Tensor<int16_t> &A{ *in[0] };
  const Tensor<int16_t> &B{ *in[1] };
  const int              shift{ in[1]->quantizer + m_q };
  const int              last = A.dims().size() - 1;
  const int              N{ A.dims()[last - 1] };
  const int              H{ A.dims()[last] };

  int t = 0;

  for (int b = 0; b < N; ++b)
  {
    const int16_t *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    const int16_t *bptr = B.getDataSparse().data();
    const auto *   idx  = B.getIndices().data();

    for (auto nunNonzero: B.getNbNonzerosCol())
    {
      __m256i s = _mm256_setzero_si256();

      for (int j = 0; j < nunNonzero; j += 16)
      {
#ifdef _mm256_loadu2_m128i
        __m256i a  = _mm256_loadu2_m128i((const __m128i *) (aptr + *(idx + 1)), (const __m128i *) (aptr + *idx));
#else
        __m256i a = _mm256_set_m128i(_mm_loadu_si128((const __m128i*) (aptr + *(idx + 1))), _mm_loadu_si128((const __m128i*) (aptr + *idx)));
#endif
        __m256i b  = _mm256_loadu_si256((const __m256i *) bptr);
        __m256i ab = _mm256_madd_epi16(a, b);

        s = _mm256_add_epi32(s, ab);

        bptr += 16;
        idx += 2;
      }

      typename ComputationType<int16_t>::type z = sum32_int16(s);
      ComputationType<int16_t>::quantize(z, shift);
      SATURATE(z);
      m_out[t] = z;

      t++;
    }
  }

  return true;
}

template<> inline bool MatMul<int16_t>::applySparsePack16MatmulSimd16(std::vector<Tensor<int16_t> *> &in)
{
  using T = int16_t;

  const Tensor<int16_t> &A{ *in[0] };
  const Tensor<int16_t> &B{ *in[1] };
  const int              shift{ in[1]->quantizer + m_q };
  const int              last = A.dims().size() - 1;
  const int              N{ A.dims()[last - 1] };
  const int              H{ A.dims()[last] };

  int t = 0;

  for (int b = 0; b < N; ++b)
  {
    const int16_t *aptr = A.data() + H * b;   // A(b,i)   => A[H*b]
    const int16_t *bptr = B.getDataSparse().data();
    const auto    *idx  = B.getIndices().data();

    for (auto nunNonzero: B.getNbNonzerosCol())
    {
      __m256i s = _mm256_setzero_si256();

      for (int j = 0; j < nunNonzero; j += 16)
      {
        __m256i a  = _mm256_loadu_si256((__m256i *) (aptr + *idx));
        __m256i b  = _mm256_loadu_si256((const __m256i *) bptr);
        __m256i ab = _mm256_madd_epi16(a, b);

        s = _mm256_add_epi32(s, ab);

        bptr += 16;
        idx ++;
      }

      typename ComputationType<int16_t>::type z = sum32_int16(s);
      ComputationType<int16_t>::quantize(z, shift);
      SATURATE(z);
      m_out[t] = z;

      t++;
    }
  }

  return true;
}
#endif

template<typename T> bool MatMul<T>::initMatMul(const std::vector<Tensor<T> *> &in)
{
  if (in.size() != 2)
  {
    return false;
  }
  // cases:
  // A: always a tensor
  // B: const (because assumed transposed)
  // 1- A [x,y] B[y,z] || A [x,y,z] B[x,z,t] || A [1,x,y,z] B[1,x,z,t]
  // 2- A [1,x,y] B[y,z] || A [1,x,y,z] B[x,z,t]
  if (in[1]->dims().size() < 2 || in[1]->dims().size() > 4)
  {
    return false;
  }
  if (in[0]->dims().size() != in[1]->dims().size() && !(in[0]->dims().size() - 1 == in[1]->dims().size() && in[0]->dims()[0] == 1))
  {
    return false;
  }
  Dimensions dim  = in[0]->dims();
  const int  last = in[0]->dims().size() - 1;

  if (in[0]->dims().size() - 1 == in[1]->dims().size())
  {
    for (int k = 1; k < last - 1; ++k)
    {
      if (in[0]->dims()[k] != in[1]->dims()[k - 1])
      {
        return false;
      }
    }
    if (in[0]->dims()[last] != in[1]->dims()[last - 2])
    {
      return false;
    }
  }
  else
  {
    // Excluding the last two dimensions, the dimension
    // of index i in the first input Tensor<T> must be equal
    // to the dimension of index i in the second input
    // Tensor<T>.
    for (int k = 0; k < last - 1; ++k)
    {
      if (in[0]->dims()[k] != in[1]->dims()[k])
      {
        return false;
      }
    }
    if (in[0]->dims()[last] != in[1]->dims()[last - 1])
    {
      return false;
    }
  }
  dim[last] = in[1]->dims().back();
  m_out.resize(dim);
  m_initDone = true;
  return true;
}

template<typename T> bool MatMul<T>::loadInternal(std::istream &file)
{
  file.read((char *) &m_q, sizeof(m_q));
  return true;
}

}   // namespace layers
}   // namespace sadl_vext
#endif
