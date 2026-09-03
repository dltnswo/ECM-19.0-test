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
#ifndef SADL_LAYER_PRELU
#define SADL_LAYER_PRELU
#include "layer.h"

namespace sadl_vext
{
namespace layers
{
template<typename T> class PReLU : public Layer<T>
{
public:
  using Layer<T>::Layer;
  using Layer<T>::m_out;   // to avoid this->
  using Layer<T>::m_initDone;

  bool initPreLU(const std::vector<Tensor<T> *> &in);
  bool applyPreLU(std::vector<Tensor<T> *> &in);

protected:
  virtual bool                   loadInternal(std::istream &file) override;
  template<bool multialpha> bool applyScalar(std::vector<Tensor<T> *> &in);
#if __AVX2__
  template<bool multialpha> bool applySimd256(std::vector<Tensor<T> *> &in);
#endif
};

template<typename T> bool PReLU<T>::applyPreLU(std::vector<Tensor<T> *> &in)
{
  assert(in.size() == 2);
  assert(in[0]->dims() == m_out.dims());

#if __AVX2__
  if (std::is_same<T, int16_t>::value && in[0]->size() % 16 == 0)
  {
    if (in[1]->size() == 1)
    {
      return applySimd256<false>(in);
    }
    else if (in[1]->size() % 16 == 0)
    {
      return applySimd256<true>(in);
    }
  }
#endif
  if (in[1]->size() == 1)
  {
    return applyScalar<false>(in);
  }
  else
  {
    return applyScalar<true>(in);
  }
}

template<typename T> template<bool multialpha> bool PReLU<T>::applyScalar(std::vector<Tensor<T> *> &in)   // without simd
{
  const Tensor<T> &A = *in[1];
  swap(*in[0], m_out);
  const int alpha_q = A.quantizer;
  if (multialpha)
  {
    switch( in[ 0 ]->dims().size() )
    {
    case 2:
    {
      const int in_N{ in[ 0 ]->dims()[ 0 ] };
      const int in_C{ in[ 0 ]->dims()[ 1 ] };

      // keep same qunatiz as input
      const int alpha_q = A.quantizer;
      for( int n_nb = 0; n_nb < in_N; n_nb++ )
      {
        for( int c_nb = 0; c_nb < in_C; c_nb++ )
        {
          // A.dims()[0] == 1, means all channels share the same alpha parameter
          const typename ComputationType<T>::type alpha = A( A.dims()[ 0 ] == 1 ? 0 : c_nb );
          if( m_out( n_nb, c_nb ) < 0 )
          {
            typename ComputationType<T>::type z = m_out( n_nb, c_nb ) * alpha;
            ComputationType<T>::quantize( z, alpha_q );
            m_out( n_nb, c_nb ) = static_cast< T >( z );
          }
        }
      }
      break;
    }
    default:
      return false;
    }
  }
  else
  {
    const typename ComputationType<T>::type alpha = A[0];
    for (auto &x: m_out)
    {
      if (x < 0)
      {
        typename ComputationType<T>::type z = x * alpha;
        ComputationType<T>::quantize(z, alpha_q);
        x = static_cast<T>(z);
      }
    }
  }
  return true;
}

#if __AVX2__
template<> template<bool multialpha> inline bool PReLU<int16_t>::applySimd256(std::vector<Tensor<int16_t> *> &in)
{
  Tensor<int16_t> &A = *in[1];
  swap(*in[0], m_out);
  int16_t *const                        data_ptr  = m_out.data();
  [[maybe_unused]] const int16_t *const alpha_ptr = A.data();
  const int                             alpha_q   = A.quantizer;

  __m256i       alpha = _mm256_set1_epi16(A[0]);
  const __m256i mask  = _mm256_set1_epi32(65535);
  const __m256i max   = _mm256_set1_epi32(32767);
  const __m256i min   = _mm256_set1_epi32(-32768);
  const __m256i zeros = _mm256_setzero_si256();
  const auto     N     = m_out.size();
  for (int64_t iter = 0; iter < N; iter += 16)
  {
    int16_t *aptr = data_ptr + iter;
    auto     a    = _mm256_load_si256((__m256i *) aptr);   // load
    if (multialpha)
    {
      alpha = _mm256_load_si256((__m256i *) (alpha_ptr + (iter % A.size())));
    }

    // prepare branches
    auto max0 = _mm256_max_epi16(a, zeros);
    auto min0 = _mm256_min_epi16(a, zeros);
    // branch neg
    // mul
    auto lo = _mm256_mullo_epi16(min0, alpha);   // min(a,0)*alpha lo part
    auto hi = _mm256_mulhi_epi16(min0, alpha);   // min(a,0)*alpha hi part
    // repack32
    auto lo32 = _mm256_unpacklo_epi16(lo, hi);
    auto hi32 = _mm256_unpackhi_epi16(lo, hi);
    auto y0   = _mm256_permute2x128_si256(lo32, hi32, _MM_SHUFFLE(0, 2, 0, 0));
    auto y1   = _mm256_permute2x128_si256(lo32, hi32, _MM_SHUFFLE(0, 3, 0, 1));
    // shift
    auto y0s = _mm256_srai_epi32(y0, alpha_q);
    auto y1s = _mm256_srai_epi32(y1, alpha_q);
    // clip
    auto y0c  = _mm256_max_epi32(y0s, min);
    auto y1c  = _mm256_max_epi32(y1s, min);
    auto y0c2 = _mm256_min_epi32(y0c, max);
    auto y1c2 = _mm256_min_epi32(y1c, max);
    // mask 16bits
    auto y0p = _mm256_and_si256(y0c2, mask);
    auto y1p = _mm256_and_si256(y1c2, mask);
    // repack
    auto z  = _mm256_packus_epi32(y0p, y1p);
    auto z2 = _mm256_permute4x64_epi64(z, _MM_SHUFFLE(3, 1, 2, 0));
    // merge 2 branches
    auto r = _mm256_add_epi16(max0, z2);
    _mm256_store_si256((__m256i *) aptr, r);
  }
  return true;
}

template<typename T> template<bool multialpha> bool PReLU<T>::applySimd256(std::vector<Tensor<T> *> &in)   //
{
  std::cerr << "[ERROR] simd type not supported: " << std::endl;
  exit(-1);
}
#endif

template<typename T> bool PReLU<T>::initPreLU(const std::vector<Tensor<T> *> &in)
{
  if( in.size() != 2 )
  {
    return false;
  }

  if( in[ 0 ]->dims().size() != 2 && in[ 0 ]->dims().size() != 4 )
  {
    return false;
  }

  m_out.resize(in[0]->dims());
  m_initDone = true;
  return true;
}

template<typename T> bool PReLU<T>::loadInternal(std::istream &) { return true; }

}   // namespace layers
}   // namespace sadl_vext
#endif
