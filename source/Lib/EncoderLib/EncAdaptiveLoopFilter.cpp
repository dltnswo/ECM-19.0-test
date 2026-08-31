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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#if JVET_AK0123_ALF_COEFF_RESTRICTION
#include <inttypes.h>
#include <unordered_set>
#endif

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx( Ctx::Alf, c)
std::vector<double> EncAdaptiveLoopFilter::m_lumaLevelToWeightPLUT;

#include <algorithm>

#if MAX_NUM_CC_ALF_FILTERS>1
struct FilterIdxCount
{
  uint64_t count;
  uint8_t filterIdx;
};

bool compareCounts(FilterIdxCount a, FilterIdxCount b) { return a.count > b.count; }
#endif
#if JVET_AK0065_TALF
struct ParamApsIdAndCount
{

  TAlfFilterParam param;
  int apsId;
  int count;
};

bool compareCountsTALF(ParamApsIdAndCount a, ParamApsIdAndCount b) { return a.count > b.count; }
#endif

#if JVET_AK0123_ALF_COEFF_RESTRICTION

/*
* Presudo-random number generator based on XoShiRo.
*/
class RandomGen
{
private:
  uint64_t state[4];

public:
  RandomGen()
  {
    {
      state[0] = 3780342784249958894ull;
      state[1] = 6804200696383934958ull;
      state[2] = 7225962928618213870ull;
      state[3] = 7542320113583526382ull;
    }
  }

  uint64_t cyclicShiftLeft(const uint64_t x, int k)
  {
    return (x << k) | (x >> (64 - k));
  }

  uint64_t nextRand()
  {
    uint64_t result = cyclicShiftLeft(state[0] + state[3], 23) + state[0];
    uint64_t tempState = state[1] << 17;
    state[2] ^= state[0];
    state[3] ^= state[1];
    state[1] ^= state[2];
    state[0] ^= state[3];
    state[2] ^= tempState;
    state[3] = cyclicShiftLeft(state[3], 45);
    return result;
  }

  bool nextBool(double trueProbability)
  {
    uint64_t coin = nextRand() >> 32;
    uint64_t bound = uint64_t(trueProbability * (1ull << 32));
    return coin < bound;
  }
};

#endif

void AlfCovariance::getClipMax(const AlfFilterShape& alfShape, int *clip_max) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    clip_max[k] = 0;

    bool inc = true;
    while( inc && clip_max[k]+1 < numBins && y(clip_max[k]+1,k) == y(clip_max[k],k) )
    {
      for( int l = 0; inc && l < numCoeff; ++l )
      {
        if( E(clip_max[k],0,k,l) != E(clip_max[k]+1,0,k,l) )
        {
          inc = false;
        }
      }
      if( inc )
      {
        ++clip_max[k];
      }
    }
  }
  clip_max[numCoeff-1] = 0;
}

void AlfCovariance::reduceClipCost(const AlfFilterShape& alfShape, int *clip) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    bool dec = true;
    while( dec && clip[k] > 0 && y(clip[k]-1,k) == y(clip[k],k) )
    {
      for( int l = 0; dec && l < numCoeff; ++l )
      {
        if( E(clip[k],clip[l],k,l) != E(clip[k]-1,clip[l],k,l) )
        {
          dec = false;
        }
      }
      if( dec )
      {
        --clip[k];
      }
    }
  }
}

#if JVET_AD0222_ALF_RESI_CLASS
#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
double AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, Ty f, bool optimize_clip, bool enableLessClip, bool encAlfResidualCheck) const
#else
double AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, Ty f, bool optimize_clip, bool enableLessClip) const
#endif
#else
double AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip, bool enableLessClip) const
#endif
#else
double AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, double *f, bool optimize_clip) const
#endif
{
  const int size = alfShape.numCoeff;
  int clip_max[MAX_NUM_ALF_LUMA_COEFF];
#if JVET_AF0177_ALF_COV_FLOAT
  Ty fBest;
  int copySize = sizeof( float ) * size;
#else
  double fBest[MAX_NUM_ALF_LUMA_COEFF];
  int copySize = sizeof( double ) * size;
#endif
  double err_best, err_last;

  TE kE;
  Ty ky;

  if( optimize_clip )
  {
    // Start by looking for min clipping that has no impact => max_clipping
    getClipMax(alfShape, clip_max);
    for (int k=0; k<size; ++k)
    {
      clip[k] = std::max(clip_max[k], clip[k]);
      clip[k] = std::min(clip[k], numBins-1);
    }
  }

  setEyFromClip( clip, kE, ky, size );

  gnsSolveByChol( kE, ky, f, size 
#if JVET_AN0086_RESIDUAL_CHECK
  , encAlfResidualCheck
#endif
  );
  err_best = calculateError( clip, f, size );
  memcpy( fBest, f, copySize );
  int step = optimize_clip ? (numBins+1)/2 : 0;

#if JVET_AD0222_ALF_RESI_CLASS
  int iter = 0;
  while( enableLessClip ? step > 0 && iter < 32 : step > 0 )
  {
    iter++;
#else
  while( step > 0 )
  {
#endif
    double err_min = err_best;
    int idx_min = -1;
    int inc_min = 0;

    for( int k = 0; k < size-1; ++k )
    {
      if( clip[k] - step >= clip_max[k] )
      {
        clip[k] -= step;
        ky[k] = y(clip[k],k);
        // Upper triangular
        for( int l = 0; l < k; l++ )
        {
          kE[l][k] = E(clip[l],clip[k],l,k);
        }
        for( int l = k; l < size; l++ )
        {
          kE[k][l] = E(clip[k],clip[l],k,l);
        }

        gnsSolveByChol( kE, ky, f, size 
#if JVET_AN0086_RESIDUAL_CHECK
        , encAlfResidualCheck
#endif
        );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = -step;
          memcpy(fBest, f, copySize);
        }
        clip[k] += step;
      }
      if( clip[k] + step < numBins )
      {
        clip[k] += step;
        ky[k] = y(clip[k],k);
        // Upper triangular
        for (int l = 0; l < k; l++)
        {
          kE[l][k] = E(clip[l], clip[k], l, k);
        }
        for (int l = k; l < size; l++)
        {
          kE[k][l] = E(clip[k], clip[l], k, l);
        }

        gnsSolveByChol( kE, ky, f, size 
#if JVET_AN0086_RESIDUAL_CHECK
        , encAlfResidualCheck
#endif
        );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = step;
          memcpy(fBest, f, copySize);
        }
        clip[k] -= step;

      }
      ky[k] = y(clip[k],k);
      // Upper triangular
      for (int l = 0; l < k; l++)
      {
        kE[l][k] = E(clip[l], clip[k], l, k);
      }
      for (int l = k; l < size; l++)
      {
        kE[k][l] = E(clip[k], clip[l], k, l);
      }
    }

    if( idx_min >= 0 )
    {
      err_best = err_min;
      clip[idx_min] += inc_min;
      ky[idx_min] = y(clip[idx_min],idx_min);
      // Upper triangular
      for (int l = 0; l < idx_min; l++)
      {
        kE[l][idx_min] = E(clip[l],clip[idx_min],l,idx_min);
      }
      for (int l = idx_min; l < size; l++)
      {
        kE[idx_min][l] = E(clip[idx_min],clip[l],idx_min,l);
      }
    }
    else
    {
      --step;
    }
  }

  if (optimize_clip)
  {
    // test all max
    for( int k = 0; k < size-1; ++k )
    {
      clip_max[k] = 0;
    }
    TE kE_max;
    Ty ky_max;
    setEyFromClip( clip_max, kE_max, ky_max, size );

    gnsSolveByChol( kE_max, ky_max, f, size 
#if JVET_AN0086_RESIDUAL_CHECK
    , encAlfResidualCheck
#endif
    );
    err_last = calculateError( clip_max, f, size );
    if( err_last < err_best )
    {
      err_best = err_last;
      for (int k=0; k<size; ++k)
      {
        clip[k] = clip_max[k];
      }
    }
    else
    {
      // update clip to reduce coding cost
      reduceClipCost(alfShape, clip);
      memcpy( f, fBest, copySize );
    }
  }

  return err_best;
}

#if ALF_PRECISION_VARIETY
void AlfCovariance::calcInitErrorForCoeffs(double *cAc, double *cA, double *bc,  const int *clip, const int *coeff, const int numCoeff, const int bitDepth, const int scaleIdx) const
{
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  double factor = (double)(1 << (bitDepth - 1 + AdaptiveLoopFilter::m_SCALE_SHIFT));
#else
  double factor = (double)(1 << (bitDepth - 1));
#endif

  *cAc = 0;
  *bc = 0;

  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    double sum = 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    for (ptrdiff_t j = 0; j < numCoeff; j++)
    {
      sum += E(clip[i], clip[j], i, j) * coeff[j] * AdaptiveLoopFilter::m_SCALE_FACTOR[scaleIdx];
    }
    (*cAc) += sum * coeff[i] * AdaptiveLoopFilter::m_SCALE_FACTOR[scaleIdx];
    cA[i] = 2 * sum;
    *bc += 2 * coeff[i] * AdaptiveLoopFilter::m_SCALE_FACTOR[scaleIdx] * y(clip[i], i);
#else
    for (ptrdiff_t j = 0; j < numCoeff; j++)
    {
      sum += E(clip[i],clip[j],i,j) * coeff[j];
    }
    (*cAc) += sum * coeff[i];
    cA[i] = 2*sum;
    *bc += 2*coeff[i] * y(clip[i],i);
#endif
  }

  *cAc /= factor * factor;
  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    cA[i] /= factor;
  }

  *bc /= factor;
}
#if JVET_AK0065_TALF
void AlfCovariance::calcInitErrorForTAlfCoeffs(double *cAc, double *cA, double *bc,  const int *clip, const int *coeff, const int numCoeff, const int bitDepth) const
{
  double factor = (double)(1 << (bitDepth - 1));

  *cAc = 0;
  *bc = 0;

  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    double sum = 0;
    for (ptrdiff_t j = 0; j < numCoeff; j++)
    {
      sum += E(clip[i],clip[j],i,j) * coeff[j];
    }
    (*cAc) += sum * coeff[i];
    cA[i] = 2*sum;
    *bc += 2*coeff[i] * y(clip[i],i);
  }

  *cAc /= factor * factor;
  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    cA[i] /= factor;
  }

  *bc /= factor;
}
#endif
void AlfCovariance::updateErrorForCoeffsDelta(double *cAc, double *cA, double *bc,  const int *clip, const int *coeff, const int numCoeff, const int bitDepth, double cDelta, int modInd  ) const
{
  int i;

  *cAc = (*cAc) + cDelta * cA[modInd] + cDelta * cDelta * E( clip[modInd], clip[modInd], modInd, modInd);
  for (i = 0; i < numCoeff; i++) {
    cA[i] += 2 * cDelta * E( clip[modInd], clip[i], modInd, i);
  }
  (*bc) = (*bc) + 2 * y(clip[modInd], modInd) * cDelta;
}
double AlfCovariance::calcErrorForCoeffsDelta(double cAc, double *cA, double bc,  const int *clip, const int *coeff, const int numCoeff, const int bitDepth, double cDelta, int modInd ) const
{
  double error;
  error = cAc - bc;
  error += cDelta * cA[modInd];
  error += cDelta * cDelta * E(clip[modInd],clip[modInd],modInd,modInd);
  error -= 2 * y(clip[modInd],modInd) * cDelta;

  return(error);
}
#endif

double AlfCovariance::calcErrorForCoeffs( const int *clip, const int *coeff, const int numCoeff, const int bitDepth, const int scaleIdx) const
{
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  double factor = (double)(1 << (bitDepth - 1 + AdaptiveLoopFilter::m_SCALE_SHIFT));
#else
  double factor = 1 << ( bitDepth - 1 );
#endif
  double error = 0;

  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    double sum = 0;
    for (ptrdiff_t j = 0; j < i; j++)
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E(clip[i],clip[j],i,j) * coeff[j];
    }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    error += ((E(clip[i], clip[i], i, i) * coeff[i] + sum * 2) * AdaptiveLoopFilter::m_SCALE_FACTOR[scaleIdx] / factor - 2 * y(clip[i], i)) * coeff[i] * AdaptiveLoopFilter::m_SCALE_FACTOR[scaleIdx];
#else
    error += ( ( E(clip[i],clip[i],i,i) * coeff[i] + sum * 2 ) / factor - 2 * y(clip[i],i) ) * coeff[i];
#endif
  }

  return error / factor;
}

double AlfCovariance::calcErrorForCcAlfCoeffs(const int16_t *coeff, const int numCoeff, const int bitDepth) const
{
  double factor = 1 << (bitDepth - 1);
  double error = 0;

  for (int i = 0; i < numCoeff; i++)   // diagonal
  {
    double sum = 0;
    for (int j = i + 1; j < numCoeff; j++)
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E(0,0,i,j) * coeff[j];
    }
    error += ((E(0,0,i,i) * coeff[i] + sum * 2) / factor - 2 * y(0,i)) * coeff[i];
  }

  return error / factor;
}

#if JVET_AK0065_TALF
double AlfCovariance::calcErrorForTAlfCoeffs( const int *clip, const int *coeff, const int numCoeff, const int bitDepth) const
{
  double factor = 1 << ( bitDepth - 1 );
  double error = 0;

  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    double sum = 0;
    for (ptrdiff_t j = 0; j < i; j++)
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E(clip[i],clip[j],i,j) * coeff[j];
    }
    error += ( ( E(clip[i],clip[i],i,i) * coeff[i] + sum * 2 ) / factor - 2 * y(clip[i],i) ) * coeff[i];
  }

  return error / factor;
}
#endif

#if JVET_AF0177_ALF_COV_FLOAT
double AlfCovariance::calculateError( const int *clip, const Ty coeff, const int numCoeff ) const
#else
double AlfCovariance::calculateError( const int *clip, const double *coeff, const int numCoeff ) const
#endif
{
  double sum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    sum += coeff[i] * y(clip[i],i);
  }

  return pixAcc - sum;
}

#if JVET_AN0086_RESIDUAL_CHECK
double AlfCovariance::calculateError( const int *clip, bool encAlfResidualCheck ) const
#else
double AlfCovariance::calculateError( const int* clip ) const
#endif
{
  Ty c;

  return optimizeFilter( clip, c, numCoeff 
#if JVET_AN0086_RESIDUAL_CHECK
  , encAlfResidualCheck
#endif
  );
}
//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#if JVET_AF0177_ALF_COV_FLOAT
#define REG              (0.0001f)
#define REG_SQR          (0.0000001f)
#else
#define REG              0.0001
#define REG_SQR          0.0000001
#endif

//Find filter coeff related
int AlfCovariance::gnsCholeskyDec( TE inpMatr, TE outMatr, int numEq ) const
{
  for( int i = 0; i < numEq; i++ )
  {
#if JVET_AF0177_ALF_COV_FLOAT
    float* inputM = inpMatr[i];
    float* outputM = outMatr[i];
    float scale = inputM[i];
#else
    double* inputM  = inpMatr[i];
    double* outputM = outMatr[i];
    double scale    = inputM[i];
#endif

#if JVET_AK0123_ALF_COEFF_RESTRICTION
    for( int k = 0; k < i; k++ )
#else
    for( int k = i - 1; k >= 0; k-- )
#endif
    {
      scale -= outMatr[k][i] * outMatr[k][i];
    }

    if( scale <= REG_SQR ) // inpMatr is singular
    {
      return 0;
    }

    outputM[i] = sqrt( scale );
#if JVET_AF0177_ALF_COV_FLOAT
    float tmp = 1 / outputM[i];
#else
    double tmp = 1 / outputM[i];
#endif

    for( int j = i + 1; j < numEq; j++ )
    {
      scale = inputM[j];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      for( int k = 0; k < i; k++ )
#else
      for( int k = i - 1; k >= 0; k-- )
#endif
      {
        scale -= outMatr[k][j] * outMatr[k][i];
      }

      outputM[j] = scale * tmp; // Upper triangular
      //outMatr[j][i] = 0.0;      // Lower triangular part
    }
  }

  return 1; // Signal that Cholesky factorization is successfully performed
}

#if JVET_AF0177_ALF_COV_FLOAT
void AlfCovariance::gnsTransposeBacksubstitution( TE U, Ty rhs, Ty x, int order ) const
#else
void AlfCovariance::gnsTransposeBacksubstitution( TE U, double* rhs, double* x, int order ) const
#endif
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for( int i = 1; i < order; i++ )
  {         /* For the rows 1..order-1           */

#if JVET_AF0177_ALF_COV_FLOAT
    float sum = 0; //Holds backsubstitution from already handled rows
#else
    double sum = 0; //Holds backsubstitution from already handled rows
#endif

    for( int j = 0; j < i; j++ ) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = ( rhs[i] - sum ) / U[i][i];       /* i'th component of solution vect.  */
  }
}

#if JVET_AF0177_ALF_COV_FLOAT
void AlfCovariance::gnsBacksubstitution( TE R, Ty z, int size, Ty A ) const
#else
void AlfCovariance::gnsBacksubstitution( TE R, double* z, int size, double* A ) const
#endif
{
  size--;
  A[size] = z[size] / R[size][size];

  for( int i = size - 1; i >= 0; i-- )
  {
#if JVET_AF0177_ALF_COV_FLOAT
    float sum = 0;
#else
    double sum = 0;
#endif

    for( int j = i + 1; j <= size; j++ )
    {
      sum += R[i][j] * A[j];
    }

    A[i] = ( z[i] - sum ) / R[i][i];
  }
}

#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
int AlfCovariance::gnsSolveByChol( const int *clip, Ty x, int numEq, bool encAlfResidualCheck ) const
#else
int AlfCovariance::gnsSolveByChol( const int *clip, Ty x, int numEq ) const
#endif
#else
int AlfCovariance::gnsSolveByChol( const int *clip, double *x, int numEq ) const
#endif
{
  TE LHS;
  Ty rhs;

  setEyFromClip( clip, LHS, rhs, numEq );
  return gnsSolveByChol( LHS, rhs, x, numEq 
#if JVET_AN0086_RESIDUAL_CHECK
  , encAlfResidualCheck
#endif
  );
}

#if JVET_AF0177_ALF_COV_FLOAT
#if JVET_AN0086_RESIDUAL_CHECK
int AlfCovariance::gnsSolveByChol( TE LHS, Ty rhs, Ty x, int numEq, bool encAlfResidualCheck ) const
#else
int AlfCovariance::gnsSolveByChol( TE LHS, Ty rhs, Ty x, int numEq ) const
#endif
#else
int AlfCovariance::gnsSolveByChol(TE LHS, double* rhs, double* x, int numEq) const
#endif
{
  Ty aux;     /* Auxiliary vector */
  TE U;    /* Upper triangular Cholesky factor of LHS */

  int res = 1;  // Signal that Cholesky factorization is successfully performed

  /* The equation to be solved is LHSx = rhs */

  /* Compute upper triangular U such that U'*U = LHS */
#if ENABLE_SIMD_OPT_ALF_CHOLESKY
  if (AdaptiveLoopFilter::m_fastCholeskyDec(LHS, U, numEq)) /* If Cholesky decomposition has been successful */
#else
  if( gnsCholeskyDec( LHS, U, numEq ) ) /* If Cholesky decomposition has been successful */
#endif
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution( U, aux, numEq, x );

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for( int i = 0; i < numEq; i++ )
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
#if ENABLE_SIMD_OPT_ALF_CHOLESKY
    res = AdaptiveLoopFilter::m_fastCholeskyDec(LHS, U, numEq);
#else
    res = gnsCholeskyDec( LHS, U, numEq );
#endif

    if( !res )
    {
#if JVET_AF0177_ALF_COV_FLOAT
      std::memset( x, 0, sizeof( float )*numEq );
#else
      std::memset( x, 0, sizeof( double )*numEq );
#endif
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* Solve U*x = aux for x */
    gnsBacksubstitution( U, aux, numEq, x );
  }
#if JVET_AN0086_RESIDUAL_CHECK
  if (encAlfResidualCheck)
  {
    Ty productAx;
    Ty residualVector;

    std::memset(productAx, 0, sizeof(float) * numEq);

    for (int i = 0; i < numEq; i++)
    {
      for (int j = i; j < numEq; j++)
      {
        if (i == j)
        {
          productAx[i] += LHS[i][i] * x[i];
        }
        else
        {
          productAx[i] += LHS[i][j] * x[j];
          productAx[j] += LHS[i][j] * x[i]; // A[j][i] = A[i][j]
        }
      }
    }

    // residualVector = rhs - productAx
    for (int i = 0; i < numEq; i++)
    {
      residualVector[i] = rhs[i] - productAx[i];
      if (std::abs(residualVector[i]) > RESIDUAL_CHECK_THR)
      {
        std::memset(x, 0, sizeof(float) * numEq);
      }
    }
  }
#endif
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////

#if JVET_AK0065_TALF
EncAdaptiveLoopFilter::EncAdaptiveLoopFilter( int& apsIdStart, int& apsIdStart2 )
#else
EncAdaptiveLoopFilter::EncAdaptiveLoopFilter( int& apsIdStart )
#endif
  : m_CABACEstimator( nullptr )
  , m_apsIdStart( apsIdStart )
#if JVET_AK0065_TALF
  , m_apsIdStart2( apsIdStart2 )
#endif
{
  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_alfCovariance[i] = nullptr;
  }
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_alfCovarianceFrame[i] = nullptr;
  }
  m_filterCoeffSet = nullptr;
  m_filterClippSet = nullptr;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  m_filterScaleIdx = nullptr;
#endif
  m_diffFilterCoeff = nullptr;

  m_alfWSSD = 0;

  m_alfCovarianceCcAlf = nullptr;
  m_alfCovarianceFrameCcAlf = nullptr;

#if JVET_AK0065_TALF
  for(int shapeIdx = 0; shapeIdx < MAX_TALF_FILTER_SHAPE; shapeIdx++)
  {
    for(int tAlfMode = 0; tAlfMode < NUM_TALF_MODE; tAlfMode++)
    {
      m_alfCovarianceTALF[shapeIdx][tAlfMode] = nullptr;
    }
  }
#endif
}

void EncAdaptiveLoopFilter::create( const EncCfg* encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE], bool createEncData )
{
  if( !createEncData )
  {
#if FIXFILTER_CFG
    AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth, encCfg->getUseAlfFixedFilter() );
#else
    AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth );
#endif
    return;
  }

  CHECK( encCfg == nullptr, "encCfg must not be null" );
  m_encCfg = encCfg;

#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  if( encCfg->getIntraPeriod() == 1 )   // all intra
  {
#if FIXFILTER_CFG
    if( encCfg->getUseAlfFixedFilter() )
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = false;
#endif
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = true;
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = false;
#if FIXFILTER_CFG
    }
    else
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = true;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = false;
    }
#endif
#if JVET_AD0222_ALF_RESI_CLASS
    m_enableLessClip = false;
#endif
  }
  else if( encCfg->getIntraPeriod() > 1 )   // random access
  {
#if FIXFILTER_CFG
    if( encCfg->getUseAlfFixedFilter() )
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = false;
#endif
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = false;
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = true;
#if FIXFILTER_CFG
    }
    else
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = true;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = false;
    }
#endif
#if JVET_AD0222_ALF_RESI_CLASS
    m_enableLessClip = true;
#endif
  }
  else if( encCfg->getIntraPeriod() == -1 )   // low delay
  {
#if FIXFILTER_CFG
    if( encCfg->getUseAlfFixedFilter() )
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = false;
#endif
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = false;
    m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = true;
#if FIXFILTER_CFG
    }
    else
    {
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_DB_RESI] = true;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = false;
      m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] = false;
    }
#endif
#if JVET_AD0222_ALF_RESI_CLASS
    m_enableLessClip = true;
#endif
  }
#if FIXFILTER_CFG
  if( encCfg->getUseAlfFixedFilter() )
  {
    m_filterTypeTest[CHANNEL_TYPE_CHROMA][ALF_FILTER_9_NO_FIX] = false;
    m_filterTypeTest[CHANNEL_TYPE_CHROMA][ALF_FILTER_9] = true;
  }
  else
  {
    m_filterTypeTest[CHANNEL_TYPE_CHROMA][ALF_FILTER_9_NO_FIX] = true;
    m_filterTypeTest[CHANNEL_TYPE_CHROMA][ALF_FILTER_9] = false;
  }
#endif
#endif
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  m_isLowDelayConfig = encCfg->getIntraPeriod() == -1 ? true : false;
  m_chromaFactor = m_isLowDelayConfig ? ( m_picWidth > 1280 && m_picHeight > 720 ? 0.60 : 0.40 ) : 1.00;
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  m_reuseCondition = m_isLowDelayConfig ? (m_picWidth > 1280 && m_picHeight > 720) : true;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION 
  m_alfPrecisonFlag = m_encCfg->getUseAlfPrecision();
#endif
  const int numBinsLuma = m_encCfg->getUseNonLinearAlfLuma() ? MaxAlfNumClippingValues : 1;
  const int numBinsChroma = m_encCfg->getUseNonLinearAlfChroma() ? MaxAlfNumClippingValues : 1;

  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    ChannelType chType = (ChannelType)channelIdx;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
#if ALF_IMPROVEMENT
      m_alfCovarianceFrame[chType][i] = nullptr;
      if (m_filterTypeTest[chType][m_filterShapes[chType][i].filterType] == false)
      {
        continue;
      }
#endif
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];

      for( int k = 0; k < numClasses; k++ )
      {
        m_alfCovarianceFrame[chType][i][k].create( m_filterShapes[chType][i].numCoeff, isLuma(chType) ? numBinsLuma : numBinsChroma);
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    m_ctuEnableFlagTmp2[compIdx] = new uint8_t[m_numCTUsInPic];
#if !ALF_IMPROVEMENT
    if( isLuma( ComponentID(compIdx) ) )
    {
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }
    else
#endif
    {
      m_ctuAlternativeTmp[compIdx] = new uint8_t[m_numCTUsInPic];
      std::fill_n( m_ctuAlternativeTmp[compIdx], m_numCTUsInPic, 0 );
    }
    ChannelType chType = toChannelType( ComponentID( compIdx ) );
#if !JVET_X0071_ALF_BAND_CLASSIFIER
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
    int numClassifier = compIdx ? 1 : ALF_NUM_CLASSIFIER;
    m_alfCovariance[compIdx] = new AlfCovariance****[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = nullptr;
      if( m_filterTypeTest[chType][m_filterShapes[chType][i].filterType] == false )
      {
        continue;
      }
      m_alfCovariance[compIdx][i] = new AlfCovariance***[m_numCTUsInPic];
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      int numFixedFilterSets = numFixedFilters( m_filterShapes[chType][i].filterType );
#else
      int numFixedFilterSets = ( m_filterShapes[chType][i].filterType == ALF_FILTER_EXT || m_filterShapes[chType][i].filterType == ALF_FILTER_9_EXT ) ? 2 : 1;
#endif
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance**[numFixedFilterSets];
        for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSets; fixedFilterSetIdx++ )
        {
          m_alfCovariance[compIdx][i][j][fixedFilterSetIdx] = new AlfCovariance*[numClassifier];
          for( int l = 0; l < numClassifier; l++ )
          {
            int numClasses = compIdx ? 1 : ALF_NUM_CLASSES_CLASSIFIER[l];
            m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][l] = new AlfCovariance[numClasses];
            for( int k = 0; k < numClasses; k++ )
            {
              m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][l][k].create(m_filterShapes[chType][i].numCoeff, isLuma(chType) ? numBinsLuma : numBinsChroma);
            }
          }
        }                           
      }
    }
#else
    m_alfCovariance[compIdx] = new AlfCovariance***[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = nullptr;
      if( m_filterTypeTest[chType][m_filterShapes[chType][i].filterType] == false )
      {
        continue;
      }
      m_alfCovariance[compIdx][i] = new AlfCovariance**[m_numCTUsInPic];
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      int numFixedFilterSets = numFixedFilters( m_filterShapes[chType][i].filterType );
#else
      int numFixedFilterSets = ( m_filterShapes[chType][i].filterType == ALF_FILTER_EXT || m_filterShapes[chType][i].filterType == ALF_FILTER_9_EXT ) ? 2 : 1;
#endif
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance*[numFixedFilterSets];
        for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSets; fixedFilterSetIdx++)
        {
          m_alfCovariance[compIdx][i][j][fixedFilterSetIdx] = new AlfCovariance[numClasses];
          for (int k = 0; k < numClasses; k++)
          {
            m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][k].create(m_filterShapes[chType][i].numCoeff);
          }
        }             
      }
    }
#endif
#else
    m_alfCovariance[compIdx] = new AlfCovariance**[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numCTUsInPic];
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovariance[compIdx][i][j][k].create( m_filterShapes[chType][i].numCoeff );
        }
      }
    }
#endif
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    if (m_filterTypeTest[CHANNEL_TYPE_LUMA][m_filterShapes[CHANNEL_TYPE_LUMA][i].filterType] == false)
    {
      continue;
    }
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMPONENT_Y][i].numCoeff, numBinsLuma);
    }
  }
#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
  const int filterCoeffSize = std::max(MAX_NUM_ALF_ALTERNATIVES_LUMA, m_alfPrecisionVariety);
#else
  const int filterCoeffSize = MAX_NUM_ALF_ALTERNATIVES_LUMA;
#endif
  m_filterCoeffSet = new int* [std::max(filterCoeffSize * MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
  m_filterClippSet = new int* [std::max(filterCoeffSize * MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  m_filterScaleIdx = new char[std::max(filterCoeffSize * MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
#endif
#else
  m_filterCoeffSet = new int*[std::max(MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
  m_filterClippSet = new int*[std::max(MAX_NUM_ALF_CLASSES, MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
#endif
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];
  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
#if ALF_IMPROVEMENT
    for (int j = 0; j < filterCoeffSize; j++)
    {
      m_filterCoeffSet[i * filterCoeffSize + j] = new int[MAX_NUM_ALF_LUMA_COEFF];
      m_filterClippSet[i * filterCoeffSize + j] = new int[MAX_NUM_ALF_LUMA_COEFF];
    }
#else
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
#endif
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }
#if !ALF_IMPROVEMENT
  for( int i = 0; i < NUM_FIXED_FILTER_SETS; i++ )
  {
    m_ctbDistortionFixedFilter[i] = new double[m_numCTUsInPic];
  }
#else
  for (int i = 0; i < NUM_FIXED_FILTER_SETS; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      m_ctbDistortionFixedFilter[i][j] = new double[m_numCTUsInPic];
    }
  }
#endif
  for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
  {
#if ALF_IMPROVEMENT
    for( int j = 0; j< MAX_NUM_ALF_ALTERNATIVES_LUMA; j++ )
    {
      for( int k = 0; k < 2; k++ )
      {
        m_distCtbApsLuma[i][j][k] = new double[m_numCTUsInPic];
      }
    }
#else
    m_distCtbApsLuma[i] = new double[m_numCTUsInPic];
#endif
  }
#if ALF_IMPROVEMENT
  for( int i = 0; i < MAX_NUM_ALF_ALTERNATIVES_LUMA; i++ )
  {
    for (int k = 0; k < 2; k++)
    {
      m_distCtbLumaNewFilt[i][k] = new double[m_numCTUsInPic];
    }
  }
#else
  m_distCtbLumaNewFilt = new double[m_numCTUsInPic];
#endif

  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    m_ctbDistortionUnfilter[comp] = new double[m_numCTUsInPic];
  }
  m_alfCtbFilterSetIndexTmp.resize(m_numCTUsInPic);
#if !ALF_IMPROVEMENT
  memset(m_clipDefaultEnc, 0, sizeof(m_clipDefaultEnc));
#endif
  m_apsIdCcAlfStart[0] = (int) MAX_NUM_APS;
  m_apsIdCcAlfStart[1] = (int) MAX_NUM_APS;
  m_alfCovarianceCcAlf = new AlfCovariance * [m_filterShapesCcAlf.size()];
  m_alfCovarianceFrameCcAlf = new AlfCovariance[m_filterShapesCcAlf.size()];
  for (int i = 0; i != m_filterShapesCcAlf.size(); i++)
  {
    m_alfCovarianceFrameCcAlf[i].create(m_filterShapesCcAlf[i].numCoeff, 1);
    m_alfCovarianceCcAlf[i] = new AlfCovariance[m_numCTUsInPic];
    for (int k = 0; k < m_numCTUsInPic; k++)
    {
      m_alfCovarianceCcAlf[i][k].create(m_filterShapesCcAlf[i].numCoeff, 1);
    }
  }
  m_trainingCovControl   = new uint8_t[m_numCTUsInPic];
  for ( int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++ )
  {
    m_trainingDistortion[i] = new uint64_t[m_numCTUsInPic];
  }
  m_filterControl         = new uint8_t[m_numCTUsInPic];
  m_bestFilterControl     = new uint8_t[m_numCTUsInPic];
  uint32_t area           = (picWidth >> getComponentScaleX(COMPONENT_Cb,chromaFormatIDC))*(picHeight >> getComponentScaleY(COMPONENT_Cb,chromaFormatIDC));
  m_bufOrigin             = ( Pel* ) xMalloc( Pel, area );
  m_buf                   = new PelBuf( m_bufOrigin, picWidth >> getComponentScaleX(COMPONENT_Cb,chromaFormatIDC), picWidth >> getComponentScaleX(COMPONENT_Cb,chromaFormatIDC), picHeight >> getComponentScaleY(COMPONENT_Cb,chromaFormatIDC) );
  m_lumaSwingGreaterThanThresholdCount = new uint64_t[m_numCTUsInPic];
  m_chromaSampleCountNearMidPoint = new uint64_t[m_numCTUsInPic];
#if JVET_AK0065_TALF
  for (int j = 0; j < ALF_CTB_MAX_NUM_APS; j++)
  {
    for ( int i = 0; i < MAX_NUM_TALF_FILTERS; i++ )
    {
      m_talfDistortion[j][i] = new uint64_t[m_numCTUsInPic];
    }
  }
  for(int shapeIdx = 0; shapeIdx < MAX_TALF_FILTER_SHAPE; shapeIdx++)
  {
    for(int tAlfMode = 0; tAlfMode < NUM_TALF_MODE; tAlfMode++)
    {
      const int numCoeff = NUM_TALF_COEFF + 1;
      m_alfCovarianceFrameTALF[shapeIdx][tAlfMode].create(numCoeff, numBinsLuma);
      m_alfCovarianceTALF[shapeIdx][tAlfMode] = new AlfCovariance[m_numCTUsInPic];
      for (int j = 0; j < m_numCTUsInPic; j++)
      {
        m_alfCovarianceTALF[shapeIdx][tAlfMode][j].create(numCoeff, numBinsLuma);
      }
    }
  }
  m_sseCost = new RdCost;
  for(int i = 0; i < 2; i++)
  {
    m_tempSaveTALF[i].destroy();
    m_tempSaveTALF[i].create(chromaFormatIDC, Area(0, 0, picWidth, picHeight));
  }
#endif
}

void EncAdaptiveLoopFilter::destroy( bool destroyEncData )
{
  if (!m_created)
  {
    return;
  }

  // !m_created guarantees common data, encoder data is guaranteed by create/destroy before/after ALFProcess
  if (!destroyEncData)
  {
    AdaptiveLoopFilter::destroy();
    return;
  }

  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    if( m_alfCovarianceFrame[channelIdx] )
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for (int i = 0; i != m_filterShapes[chType].size(); i++)
      {
#if ALF_IMPROVEMENT
        if (m_alfCovarianceFrame[channelIdx][i] == nullptr)
        {
          continue;
        }
#endif
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }

  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    if( m_ctuEnableFlagTmp[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }

    if( m_ctuEnableFlagTmp2[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp2[compIdx];
      m_ctuEnableFlagTmp2[compIdx] = nullptr;
    }

    if( m_ctuAlternativeTmp[compIdx] )
    {
      delete[] m_ctuAlternativeTmp[compIdx];
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }

    if( m_alfCovariance[compIdx] )
    {
      ChannelType chType = toChannelType( ComponentID( compIdx ) );
#if !JVET_X0071_ALF_BAND_CLASSIFIER
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;
#endif

      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
#if ALF_IMPROVEMENT
        if (m_alfCovariance[compIdx][i] == nullptr)
        {
          continue;
        }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        int numFixedFilterSet = numFixedFilters( m_filterShapes[chType][i].filterType );
#else
        int numFixedFilterSet = ( m_filterShapes[chType][i].filterType == ALF_FILTER_9_EXT || m_filterShapes[chType][i].filterType == ALF_FILTER_EXT ) ? 2 : 1;
#endif
#endif
        for( int j = 0; j < m_numCTUsInPic; j++ )
        {
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
          int numClassifier = compIdx ? 1 : ALF_NUM_CLASSIFIER;
          for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
          {
            for( int l = 0; l < numClassifier; l++ )
            {
              int numClasses = compIdx ? 1 : ALF_NUM_CLASSES_CLASSIFIER[l];
              for( int k = 0; k < numClasses; k++ )
              {
                m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][l][k].destroy();
              }
              delete[] m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][l];
              m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][l] = nullptr;
            }
            delete[] m_alfCovariance[compIdx][i][j][fixedFilterSetIdx];
            m_alfCovariance[compIdx][i][j][fixedFilterSetIdx] = nullptr;
          }
#else
          for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
          {
            for( int k = 0; k < numClasses; k++ )
            {
              m_alfCovariance[compIdx][i][j][fixedFilterSetIdx][k].destroy();
            }
            delete[] m_alfCovariance[compIdx][i][j][fixedFilterSetIdx];
            m_alfCovariance[compIdx][i][j][fixedFilterSetIdx] = nullptr;
          }
#endif
#else
          for( int k = 0; k < numClasses; k++ )
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
#endif
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if( m_filterCoeffSet )
  {
#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
    const int filterCoeffSize = std::max(MAX_NUM_ALF_ALTERNATIVES_LUMA, m_alfPrecisionVariety);
#else
    const int filterCoeffSize = MAX_NUM_ALF_ALTERNATIVES_LUMA;
#endif
    for( int i = 0; i < MAX_NUM_ALF_CLASSES * filterCoeffSize; i++ )
#else
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
#endif
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if( m_filterClippSet )
  {
#if ALF_IMPROVEMENT
    for ( int i = 0; i < MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_ALTERNATIVES_LUMA; i++ )
#else
    for ( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
#endif
    {
      delete[] m_filterClippSet[i];
      m_filterClippSet[i] = nullptr;
    }
    delete[] m_filterClippSet;
    m_filterClippSet = nullptr;
  }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  if (m_filterScaleIdx)
  {
    delete[] m_filterScaleIdx;
    m_filterScaleIdx = nullptr;
  }
#endif
  if( m_diffFilterCoeff )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }
#if ALF_IMPROVEMENT
  for( int i = 0; i < NUM_FIXED_FILTER_SETS; i++ )
  {
    for( int j = 0; j < 2; j++ )
    {
      if( m_ctbDistortionFixedFilter[i][j] )
      {
        delete[] m_ctbDistortionFixedFilter[i][j];
        m_ctbDistortionFixedFilter[i][j] = nullptr;
      }
    }
  }
#else
  for( int i = 0; i < NUM_FIXED_FILTER_SETS; i++ )
  {
    if( m_ctbDistortionFixedFilter[i] )
    {
      delete[] m_ctbDistortionFixedFilter[i];
      m_ctbDistortionFixedFilter[i] = nullptr;
    }
  }
#endif
  for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
  {
#if ALF_IMPROVEMENT
    for( int j = 0; j< MAX_NUM_ALF_ALTERNATIVES_LUMA; j++ )
    {
      for( int k = 0; k < 2; k++)
      {
        if( m_distCtbApsLuma[i][j][k] )
        {
          delete[] m_distCtbApsLuma[i][j][k];
          m_distCtbApsLuma[i][j][k] = nullptr;
        }
      }
    }
#else
    if( m_distCtbApsLuma[i] )
    {
      delete[] m_distCtbApsLuma[i];
      m_distCtbApsLuma[i] = nullptr;
    }
#endif
  }
#if ALF_IMPROVEMENT
  for( int i = 0; i < MAX_NUM_ALF_ALTERNATIVES_LUMA; i++ )
  {
    for( int k = 0; k < 2; k++)
    {
      if( m_distCtbLumaNewFilt[i][k] )
      {
        delete[] m_distCtbLumaNewFilt[i][k];
        m_distCtbLumaNewFilt[i][k] = nullptr;
      }
    }
  }
#else
  if( m_distCtbLumaNewFilt )
  {
    delete[] m_distCtbLumaNewFilt;
    m_distCtbLumaNewFilt = nullptr;
  }
#endif

  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    delete[] m_ctbDistortionUnfilter[comp];
    m_ctbDistortionUnfilter[comp] = nullptr;
  }

  if (m_alfCovarianceFrameCcAlf)
  {
    for (int i = 0; i != m_filterShapesCcAlf.size(); i++)
    {
      m_alfCovarianceFrameCcAlf[i].destroy();
    }
    delete[] m_alfCovarianceFrameCcAlf;
    m_alfCovarianceFrameCcAlf = nullptr;
  }
  if (m_alfCovarianceCcAlf)
  {
    for (int i = 0; i != m_filterShapesCcAlf.size(); i++)
    {
      for (int k = 0; k < m_numCTUsInPic; k++)
      {
        m_alfCovarianceCcAlf[i][k].destroy();
      }
      delete[] m_alfCovarianceCcAlf[i];
    }
    delete[] m_alfCovarianceCcAlf;
    m_alfCovarianceCcAlf = nullptr;
  }

  if (m_trainingCovControl)
  {
    delete[] m_trainingCovControl;
    m_trainingCovControl = nullptr;
  }

  for ( int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++ )
  {
    if (m_trainingDistortion[i])
    {
      delete[] m_trainingDistortion[i];
      m_trainingDistortion[i] = nullptr;
    }
  }

  if (m_filterControl)
  {
    delete[] m_filterControl;
    m_filterControl = nullptr;
  }

  if (m_bestFilterControl)
  {
    delete[] m_bestFilterControl;
    m_bestFilterControl = nullptr;
  }

  if (m_bufOrigin)
  {
    xFree(m_bufOrigin);
    m_bufOrigin = nullptr;
  }

  if (m_buf)
  {
    delete m_buf;
    m_buf = nullptr;
  }

  if (m_lumaSwingGreaterThanThresholdCount)
  {
    delete[] m_lumaSwingGreaterThanThresholdCount;
    m_lumaSwingGreaterThanThresholdCount = nullptr;
  }
  if (m_chromaSampleCountNearMidPoint)
  {
    delete[] m_chromaSampleCountNearMidPoint;
    m_chromaSampleCountNearMidPoint = nullptr;
  }
#if JVET_AK0065_TALF
  for (int j = 0; j < ALF_CTB_MAX_NUM_APS; j++)
  {
    for ( int i = 0; i < MAX_NUM_TALF_FILTERS; i++ )
    {
      if ( m_talfDistortion[j][i] )
      {
        delete[] m_talfDistortion[j][i];
        m_talfDistortion[j][i] = nullptr;
      }
    }
  }
  for(int shapeIdx = 0; shapeIdx < MAX_TALF_FILTER_SHAPE; shapeIdx++)
  {
    for(int tAlfMode = 0; tAlfMode < NUM_TALF_MODE; tAlfMode++)
    {
      m_alfCovarianceFrameTALF[shapeIdx][tAlfMode].destroy();
      if (m_alfCovarianceTALF[shapeIdx][tAlfMode])
      {
        for (int k = 0; k < m_numCTUsInPic; k++)
        {
          m_alfCovarianceTALF[shapeIdx][tAlfMode][k].destroy();
        }
        delete[] m_alfCovarianceTALF[shapeIdx][tAlfMode];
        m_alfCovarianceTALF[shapeIdx][tAlfMode] = nullptr;
      }
    }
  }
  if(m_sseCost)
  {
    m_sseCost = nullptr;
  }
  for(int i = 0; i < 2; i++)
  {
    m_tempSaveTALF[i].destroy();
  }
#endif
}

void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice
, ParameterSetMap<APS>* apsMap )
{
  m_apsMap = apsMap;
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_ctxCache = ctxCache;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}

void EncAdaptiveLoopFilter::xSetupCcAlfAPS( CodingStructure &cs )
{
  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMPONENT_Cb - 1])
  {
    int  ccAlfCbApsId = cs.slice->getTileGroupCcAlfCbApsId();
    APS* aps = m_apsMap->getPS((cs.slice->getTileGroupCcAlfCbApsId() << NUM_APS_TYPE_LEN) + ALF_APS);
    if (aps == NULL)
    {
      aps = m_apsMap->allocatePS((ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      aps->setTemporalId(cs.slice->getTLayer());
    }
    aps->getCcAlfAPSParam().ccAlfFilterEnabled[COMPONENT_Cb - 1] = 1;
#if JVET_AH0057_CCALF_COEFF_PRECISION
    aps->getCcAlfAPSParam().ccAlfCoeffPrec[COMPONENT_Cb - 1] = m_ccAlfFilterParam.ccAlfCoeffPrec[COMPONENT_Cb - 1];
#endif
    aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cb - 1] = m_ccAlfFilterParam.ccAlfFilterCount[COMPONENT_Cb - 1];
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMPONENT_Cb - 1][filterIdx];
      memcpy(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdx],
             m_ccAlfFilterParam.ccAlfCoeff[COMPONENT_Cb - 1][filterIdx], sizeof(short) * MAX_NUM_CC_ALF_CHROMA_COEFF);
    }
    aps->setAPSId(ccAlfCbApsId);
    aps->setAPSType(ALF_APS);
    if (m_reuseApsId[COMPONENT_Cb - 1] < 0)
    {
      aps->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cb - 1] = 1;
      m_apsMap->setChangedFlag((ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS, true);
      aps->setTemporalId(cs.slice->getTLayer());
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      if( (!cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) && !cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) ) ||cs.slice->getTileGroupApsIdChroma() != ccAlfCbApsId)
      {
        aps->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = 0;
      }
#endif
    }
    cs.slice->setTileGroupCcAlfCbEnabledFlag(true);
  }
  else
  {
    cs.slice->setTileGroupCcAlfCbEnabledFlag(false);
  }
  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMPONENT_Cr - 1])
  {
    int  ccAlfCrApsId = cs.slice->getTileGroupCcAlfCrApsId();
    APS* aps = m_apsMap->getPS((cs.slice->getTileGroupCcAlfCrApsId() << NUM_APS_TYPE_LEN) + ALF_APS);
    if (aps == NULL)
    {
      aps = m_apsMap->allocatePS((ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      aps->setTemporalId(cs.slice->getTLayer());
    }
    aps->getCcAlfAPSParam().ccAlfFilterEnabled[COMPONENT_Cr - 1] = 1;
#if JVET_AH0057_CCALF_COEFF_PRECISION
    aps->getCcAlfAPSParam().ccAlfCoeffPrec[COMPONENT_Cr - 1] = m_ccAlfFilterParam.ccAlfCoeffPrec[COMPONENT_Cr - 1];
#endif
    aps->getCcAlfAPSParam().ccAlfFilterCount[COMPONENT_Cr - 1] = m_ccAlfFilterParam.ccAlfFilterCount[COMPONENT_Cr - 1];
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      aps->getCcAlfAPSParam().ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMPONENT_Cr - 1][filterIdx];
      memcpy(aps->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdx],
             m_ccAlfFilterParam.ccAlfCoeff[COMPONENT_Cr - 1][filterIdx], sizeof(short) * MAX_NUM_CC_ALF_CHROMA_COEFF);
    }
    aps->setAPSId(ccAlfCrApsId);
    if (m_reuseApsId[COMPONENT_Cr - 1] < 0)
    {
      aps->getCcAlfAPSParam().newCcAlfFilter[COMPONENT_Cr - 1] = 1;
      m_apsMap->setChangedFlag((ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS, true);
      aps->setTemporalId(cs.slice->getTLayer());
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      if( (!cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) && !cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) ) || cs.slice->getTileGroupApsIdChroma() != ccAlfCrApsId)
      {
        aps->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = 0;
      }
#endif
    }
    aps->setAPSType(ALF_APS);
    cs.slice->setTileGroupCcAlfCrEnabledFlag(true);
  }
  else
  {
    cs.slice->setTileGroupCcAlfCrEnabledFlag(false);
  }
}

void EncAdaptiveLoopFilter::ALFProcess( CodingStructure& cs, const double *lambdas
#if ENABLE_QPA
                                        , const double lambdaChromaWeight
#endif
                                        , Picture* pcPic, uint32_t numSliceSegments
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                                        , const int intraPeriod
#endif
)
{
  int layerIdx = cs.vps == nullptr ? 0 : cs.vps->getGeneralLayerIdx( cs.slice->getPic()->layerId );

  // IRAP AU is assumed
  if( !layerIdx && ( cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA() ) )
  {
    memset( cs.slice->getAlfAPSs(), 0, sizeof( *cs.slice->getAlfAPSs() )*ALF_CTB_MAX_NUM_APS );
    m_apsIdStart = ALF_CTB_MAX_NUM_APS;

    m_apsMap->clear();
    for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
    {
      APS* alfAPS = m_apsMap->getPS( ( i << NUM_APS_TYPE_LEN ) + ALF_APS );
      m_apsMap->clearChangedFlag( ( i << NUM_APS_TYPE_LEN ) + ALF_APS );
      if( alfAPS )
      {
        alfAPS->getAlfAPSParam().reset();
        alfAPS->getCcAlfAPSParam().reset();
        alfAPS = nullptr;
      }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      g_pictureControlInfo[i].reset();
#endif
    }
#if JVET_AK0065_TALF
    memset(cs.slice->getTAlfAPSs(), 0, sizeof(*cs.slice->getTAlfAPSs()) * ALF_CTB_MAX_NUM_APS);
    m_apsIdStart2 = ALF_CTB_MAX_NUM_APS;
    for(int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* talfAPS = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + TALF_APS);
      m_apsMap->clearChangedFlag((i << NUM_APS_TYPE_LEN) + TALF_APS);
      if(talfAPS)
      {
        talfAPS->getTAlfAPSParam().reset();
        talfAPS = nullptr;
      }
    }
#endif
  }

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  resetAlfScalePrev( *cs.slice );
#endif

#if ALF_IMPROVEMENT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  cs.slice->setTileGroupAlfFixedFilterSetIdx(COMPONENT_Y, -1);
  cs.slice->setTileGroupAlfFixedFilterSetIdx(COMPONENT_Cb, -1);
  cs.slice->setTileGroupAlfFixedFilterSetIdx(COMPONENT_Cr, -1);
#else
  cs.slice->setTileGroupAlfFixedFilterSetIdx(-1);
#endif
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
  cs.slice->setTileGroupAlfReuseFlag(COMPONENT_Y, false);
  cs.slice->setTileGroupAlfReuseFlag(COMPONENT_Cb, false);
  cs.slice->setTileGroupAlfReuseFlag(COMPONENT_Cr, false);
  cs.slice->setTileGroupCcalfReuseFlag(COMPONENT_Cb, false);
  cs.slice->setTileGroupCcalfReuseFlag(COMPONENT_Cr, false);
#endif
  AlfParam alfParam;
  alfParam.reset();
  const TempCtx  ctxStart( m_ctxCache, AlfCtx( m_CABACEstimator->getCtx() ) );

  const TempCtx ctxStartCcAlf( m_ctxCache, SubCtx( Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );
#if JVET_AK0065_TALF
  const TempCtx ctxStartTAlf(m_ctxCache, SubCtx( Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );
#endif

  // set available filter shapes
  alfParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
    m_ctuAlternative[compIdx] = cs.picture->getAlfCtuAlternativeData( compIdx );
  }


  // reset ALF parameters
  alfParam.reset();
  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT( m_inputBitDepth[CHANNEL_TYPE_LUMA] );
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT( m_inputBitDepth[CHANNEL_TYPE_CHROMA] );
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double( 1 << shiftLuma );
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double( 1 << shiftChroma );
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double( 1 << shiftChroma );
#if JVET_AI0058_ALF_RELAXED_RDO_LUMA
  if (pcPic->temporalId > 0)
  {
    double lambdaModifierLuma = 0.75;
    m_lambda[COMPONENT_Y] *= lambdaModifierLuma;
  }
#endif

#if JVET_AL0153_ALF_CCCM
#if ALF_SAO_TRUE_ORG
  PelUnitBuf orgYuv = cs.slice->getLfCccmEnabledFlag() ? *m_newOrgBuf : cs.getTrueOrgBuf();
#else
  PelUnitBuf orgYuv = cs.slice->getLfCccmEnabledFlag() ? *m_newOrgBuf : cs.getOrgBuf();
#endif
#else
#if ALF_SAO_TRUE_ORG
  PelUnitBuf orgYuv = cs.getTrueOrgBuf();
#else
  PelUnitBuf orgYuv = cs.getOrgBuf();
#endif
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  memset( m_ctuPadFlag, 0, sizeof( uint8_t ) * m_numCTUsInPic );
#endif

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  m_tempBuf3.copyFrom( cs.getRecoBuf(), false, true );  // chroma only
  PelUnitBuf recYuvSao = m_tempBuf3.getBuf( cs.area );

  cs.slice->setAlfScaleChroma( 1, 0 );
  cs.slice->setAlfScaleChroma( 2, 0 );
#endif

  m_tempBuf.copyFrom( cs.getRecoBuf() );
  PelUnitBuf recYuv = m_tempBuf.getBuf( cs.area );
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_LONGER_FILTER 
  mirroredPaddingForAlf(cs, recYuv, MAX_FILTER_LENGTH_FIXED >> 1, true, true);
#else
  recYuv.extendBorderPel( MAX_FILTER_LENGTH_FIXED >> 1 );
#endif
#else
#if JVET_AA0095_ALF_LONGER_FILTER 
  mirroredPaddingForAlf(cs, recYuv, MAX_ALF_FILTER_LENGTH >> 1, true, true);
#else
  recYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );
#endif
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  m_isFixedFilterPaddedPerCtu = cs.picHeader->getVirtualBoundariesPresentFlag() || cs.slice->getCuQpDeltaSubdiv();
#endif

#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  PelUnitBuf recYuvBeforeDb = m_tempBufBeforeDb.getBuf( cs.area );
  mirroredPaddingForAlf(cs, recYuvBeforeDb, NUM_DB_PAD, true, true);
#else
  PelUnitBuf recYuvBeforeDb = m_tempBufBeforeDb.getBuf( UnitArea( CHROMA_400, cs.area.blocks[COMPONENT_Y] ) );
  mirroredPaddingForAlf(cs, recYuvBeforeDb, NUM_DB_PAD, true, false);
#endif
  const CPelBuf& recLumaBeforeDb = recYuvBeforeDb.get( COMPONENT_Y );
#else
  m_tempBufBeforeDb.getBuf( UnitArea( CHROMA_400, cs.area.blocks[COMPONENT_Y] ) ).extendBorderPel( NUM_DB_PAD );
#endif
#endif
#if NNLF_ALF_POS_INTERFACE
  if (cs.slice[0].getNnlfUnifiedParameters().mode != -1)
  {
    PelUnitBuf recYuvNNVC = m_tempBufAfterNNVC.getBuf(cs.area);
    mirroredPaddingForAlf(cs, recYuvNNVC, NUM_DB_PAD, true, true);
  }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  PelUnitBuf resiYuv = m_tempBufResi.getBuf( UnitArea( CHROMA_400, cs.area.blocks[COMPONENT_Y] ) );
  mirroredPaddingForAlf(cs, resiYuv, MAX_FILTER_LENGTH_FIXED >> 1, true, false);
#else
  PelUnitBuf resiYuv = m_tempBufResi.getBuf(cs.area);
  mirroredPaddingForAlf(cs, resiYuv, MAX_FILTER_LENGTH_FIXED, true, false);
#endif
  const CPelBuf &resiLuma = resiYuv.get(COMPONENT_Y);
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  PelUnitBuf recYuvCodingInfo = m_tempBufCodingInfo.getBuf( cs.area );
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  m_tempBufSAO.copyFrom(cs.getRecoBuf());
  PelUnitBuf recYuvSAO = m_tempBufSAO.getBuf(cs.area);
  mirroredPaddingForAlf(cs, recYuvSAO, MAX_FILTER_LENGTH_FIXED >> 1, false, true);
#endif
  // derive classification
  const CPelBuf& recLuma = recYuv.get( COMPONENT_Y );
  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  setAlfScaleMode( cs.sps->getAlfScaleMode() );
  cs.slice->resetAlfScale();
#endif

#if FIXFILTER_CFG
  bool useFixedFilter = m_encCfg->getUseAlfFixedFilter();
#endif

  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
      int rasterSliceAlfPad = 0;
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            const int wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
            const int hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
            PelUnitBuf buf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            buf.copyFrom( recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, buf, MAX_ALF_PADDING_SIZE, true, true);
#else
            buf.extendBorderPel( MAX_ALF_PADDING_SIZE );
#endif
            buf = buf.subBuf( UnitArea( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION || JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            PelUnitBuf bufCodingInfo = m_tempBufCodingInfo2.subBuf( UnitArea( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
            bufCodingInfo.copyFrom( recYuvCodingInfo.subBuf( UnitArea( CHROMA_400, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              bufCodingInfo.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              bufCodingInfo.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
            mirroredPaddingForAlf(cs, bufCodingInfo, MAX_ALF_PADDING_SIZE, true, false);
            bufCodingInfo = bufCodingInfo.subBuf( UnitArea( CHROMA_400, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
            PelUnitBuf bufResi = m_tempBufResi2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
            bufResi.copyFrom(resiYuv.subBuf(
              UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
                                                  yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
            // pad top-left unavailable samples for raster slice
            if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
            {
              bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
            }

            // pad bottom-right unavailable samples for raster slice
            if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
            {
              bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
            }
#if JVET_AA0095_ALF_LONGER_FILTER
            mirroredPaddingForAlf(cs, bufResi, MAX_ALF_PADDING_SIZE, true, false);
#else
            bufResi.extendBorderPel(MAX_ALF_PADDING_SIZE);
#endif
            bufResi = bufResi.subBuf(UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            //RecBeforeDb
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf(UnitArea(m_chromaFormat, Area(0, 0, wBuf, hBuf)));
            bufDb.copyFrom(m_tempBufBeforeDb.subBuf(UnitArea(m_chromaFormat, Area(xStart - (clipL ? 0 : NUM_DB_PAD), yStart - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf))));
#else
            PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf(UnitArea(CHROMA_400, Area(0, 0, wBuf, hBuf)));
            bufDb.copyFrom(m_tempBufBeforeDb.subBuf(UnitArea(CHROMA_400, Area(xStart - (clipL ? 0 : NUM_DB_PAD), yStart - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf))));
#endif
            // pad top-left unavailable samples for raster slice
            if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
            {
              bufDb.padBorderPel(NUM_DB_PAD, 1);
            }
            // pad bottom-right unavailable samples for raster slice
            if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
            {
              bufDb.padBorderPel(NUM_DB_PAD, 2);
            }
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            mirroredPaddingForAlf(cs, bufDb, NUM_DB_PAD, true, true);
            bufDb = bufDb.subBuf(UnitArea(m_chromaFormat, Area(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h)));
#else
            mirroredPaddingForAlf(cs, bufDb, NUM_DB_PAD, true, false);
            bufDb = bufDb.subBuf(UnitArea(CHROMA_400, Area(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h)));
#endif
#endif
            const Area blkSrc( 0, 0, w, h );
            const Area blkDst( xStart, yStart, w, h );
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if FIXFILTER_CFG
            if(useFixedFilter)
            {
#endif
            int scaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, m_chromaFormat);
            int scaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, m_chromaFormat);
            const Area blkSrcChroma(0, 0, w >> scaleX, h >> scaleY);
            const Area blkDstChroma( xStart >> scaleX, yStart >> scaleY, w >> scaleX, h >> scaleY );
            deriveFixedFilterChroma( m_classifier, buf, bufDb, blkDstChroma, blkSrcChroma, cs, -1, MAX_NUM_COMPONENT );
#if FIXFILTER_CFG
            }
#endif
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
            calcAlfLumaCodingInfoBlk(cs, m_classifierCodingInfo[0], blkDst, blkSrc, buf.get(COMPONENT_Y), 2, 2, m_inputBitDepth[CHANNEL_TYPE_LUMA], bufResi.get(COMPONENT_Y), m_laplacian[0], bufCodingInfo.get(COMPONENT_Y) );
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
            deriveClassification( m_classifier, buf.get( COMPONENT_Y ),
#if FIXFILTER_CFG
                  useFixedFilter,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                                 m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] == true, bufResi.get(COMPONENT_Y),
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
              bufDb.get( COMPONENT_Y ), 0,
#endif
              blkDst, blkSrc, cs, -1, ALF_NUM_CLASSIFIER );
#else
            deriveClassification( m_classifier, buf.get( COMPONENT_Y ), blkDst, blkSrc 
#if ALF_IMPROVEMENT
              , cs, -1
#endif
            );
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            deriveGaussResults( bufDb.get( COMPONENT_Y ), blkDst, blkSrc, cs, 0, 0 );
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            deriveLaplacianResults( buf.get( COMPONENT_Y ), blkDst, blkSrc, cs, 0, 0, bufCodingInfo.get( COMPONENT_Y ) );
#endif
            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        Area blk( xPos, yPos, width, height );
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if FIXFILTER_CFG
        if(useFixedFilter)
        {
#endif
        int scaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, m_chromaFormat);
        int scaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, m_chromaFormat);
        Area blkChroma(xPos >> scaleX, yPos >> scaleY, width >> scaleX, height >> scaleY);
        deriveFixedFilterChroma( m_classifier, recYuv, recYuvBeforeDb, blkChroma, blkChroma, cs, -1, MAX_NUM_COMPONENT );
#if FIXFILTER_CFG
        }
#endif
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
        calcAlfLumaCodingInfoBlk(cs, m_classifierCodingInfo[0], blk, blk, recYuv.get(COMPONENT_Y), 2, 2, m_inputBitDepth[CHANNEL_TYPE_LUMA], resiYuv.get(COMPONENT_Y), m_laplacian[0], recYuvCodingInfo.get(COMPONENT_Y) );
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
        deriveClassification( m_classifier, recLuma,
#if FIXFILTER_CFG
                          useFixedFilter,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                             m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI] == true, resiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          m_tempBufBeforeDb.get( COMPONENT_Y ), 0,
#endif
          blk, blk , cs, -1, ALF_NUM_CLASSIFIER );
#else
        deriveClassification( m_classifier, recLuma, blk, blk 
#if ALF_IMPROVEMENT
          , cs, -1
#endif
        );
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        deriveGaussResults( recLumaBeforeDb, blk, blk, cs, 0, 0 );
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        deriveLaplacianResults( recLuma, blk, blk, cs, 0, 0, recYuvCodingInfo.get( COMPONENT_Y ) );
#endif
      }
    }
  }
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if FIXFILTER_CFG
  if(useFixedFilter)
  {
#endif
  for(int fixedFilterSetIdx = 0; fixedFilterSetIdx < NUM_FIXED_FILTER_SETS; fixedFilterSetIdx++)
  {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
    paddingFixedFilterResultsPic(m_fixFilterResult, fixedFilterSetIdx, COMPONENT_Y);
#else
    paddingFixedFilterResultsPic(m_fixFilterResult, fixedFilterSetIdx);
#endif
  }
#if FIXFILTER_CFG
  }
#endif
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
  {
    paddingGaussResultsPic(m_gaussPic, gaussIdx);
  }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++ )
  {
    paddingLaplacianResultsPic(m_laplacianPic, laplacianIdx);
  }
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if FIXFILTER_CFG
  if(useFixedFilter)
  {
#endif
  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
      int rasterSliceAlfPad = 0;
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            const int wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
            const int hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );

            //RecBeforeDb
            PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf( UnitArea( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
            bufDb.copyFrom( m_tempBufBeforeDb.subBuf( UnitArea( CHROMA_400, Area( xStart - ( clipL ? 0 : NUM_DB_PAD ), yStart - (clipT ? 0 : NUM_DB_PAD ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              bufDb.padBorderPel( NUM_DB_PAD, 1 );
            }
            // pad bottom-right unavailable samples for raster slice
            if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              bufDb.padBorderPel( NUM_DB_PAD, 2 );
            }
            mirroredPaddingForAlf( cs, bufDb, NUM_DB_PAD, true, false );
            bufDb = bufDb.subBuf( UnitArea( CHROMA_400, Area( clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h ) ) );
            const Area blkSrc( 0, 0, w, h );
            const Area blkDst( xStart, yStart, w, h );
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            paddingFixedFilterResultsCtu( m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, 0, blkDst, 0 );
            paddingFixedFilterResultsCtu( m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, 1, blkDst, 0 );
#else
            paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, 0, blkDst, 0 );
            paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, 1, blkDst, 0 );
#endif
            deriveFixedFilterResults( m_classifier, recLuma, bufDb.get( COMPONENT_Y ), blkDst, blkSrc, cs, 1, -1 );

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        Area blk(xPos, yPos, width, height);
        deriveFixedFilterResults( m_classifier, recLuma, m_tempBufBeforeDb.get(COMPONENT_Y), blk, blk, cs, 1, -1 );
      }
    }
  }

  for( int fixedFilterSetIdx = NUM_FIXED_FILTER_SETS; fixedFilterSetIdx < NUM_FIXED_FILTER_SETS * EXT_LENGTH; fixedFilterSetIdx++ )
  {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
    paddingFixedFilterResultsPic( m_fixFilterResult, fixedFilterSetIdx, COMPONENT_Y );
#else
    paddingFixedFilterResultsPic( m_fixFilterResult, fixedFilterSetIdx );
#endif
  }
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < 2; fixedFilterSetIdx++ )
  {
    paddingFixedFilterResultsPic( m_fixFilterResult, fixedFilterSetIdx, COMPONENT_Cb );
    paddingFixedFilterResultsPic( m_fixFilterResult, fixedFilterSetIdx, COMPONENT_Cr );
  }
#endif
#if FIXFILTER_CFG
  }
#endif
#endif

  // get CTB stats for filtering
  if( m_alfWSSD )
  {
    deriveStatsForFiltering<true>( orgYuv, recYuv, cs );
  }
  else
  {
    deriveStatsForFiltering<false>( orgYuv, recYuv, cs );
  }

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    cs.slice->getPic()->getAlfCtbFilterIndex()[ctbIdx] = NUM_FIXED_FILTER_SETS;
  }
  // consider using new filter (only)
  alfParam.newFilterFlag[CHANNEL_TYPE_LUMA] = true;
  alfParam.newFilterFlag[CHANNEL_TYPE_CHROMA] = true;
  cs.slice->setTileGroupNumAps(1); // Only new filter for RD cost optimization
  // derive filter (luma)
  alfEncoder( cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA
#if ENABLE_QPA
            , lambdaChromaWeight
#endif
            );

  // derive filter (chroma)
  if (isChromaEnabled(cs.pcv->chrFormat))
  {
    alfEncoder( cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA
#if ENABLE_QPA
              , lambdaChromaWeight
#endif
              );
  }

  // let alfEncoderCtb decide now
  alfParam.newFilterFlag[CHANNEL_TYPE_LUMA] = false;
  alfParam.newFilterFlag[CHANNEL_TYPE_CHROMA] = false;
  cs.slice->setTileGroupNumAps(0);
  m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
  alfEncoderCtb(cs, alfParam
#if ENABLE_QPA
    , lambdaChromaWeight
#endif
  );

  for (int s = 0; s < numSliceSegments; s++)
  {
    if (pcPic->slices[s]->isLossless())
    {
      for (uint32_t ctuIdx = 0; ctuIdx < pcPic->slices[s]->getNumCtuInSlice(); ctuIdx++)
      {
        uint32_t ctuRsAddr = pcPic->slices[s]->getCtuAddrInSlice(ctuIdx);
        m_ctuEnableFlag[COMPONENT_Y][ctuRsAddr] = 0;
        m_ctuEnableFlag[COMPONENT_Cb][ctuRsAddr] = 0;
        m_ctuEnableFlag[COMPONENT_Cr][ctuRsAddr] = 0;
      }
    }
  }

  alfReconstructor(cs, recYuv);

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  if ( cs.sps->getAlfScaleMode() )
  {
    alfCorrection( cs, orgYuv, recYuv, true );
    alfCorrection( cs, orgYuv, recYuv );
  }
#endif

  // Do not transmit CC ALF if it is unchanged
  if (cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    for (int32_t lumaAlfApsId : cs.slice->getTileGroupApsIdLuma())
    {
      APS* aps = (lumaAlfApsId >= 0) ? m_apsMap->getPS((lumaAlfApsId << NUM_APS_TYPE_LEN) + ALF_APS) : nullptr;
      if (aps && m_apsMap->getChangedFlag((lumaAlfApsId << NUM_APS_TYPE_LEN) + ALF_APS))
      {
        aps->getCcAlfAPSParam().newCcAlfFilter[0] = false;
          aps->getCcAlfAPSParam().newCcAlfFilter[1] = false;
      }
    }
  }
  int chromaAlfApsId = ( cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) || cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) ) ? cs.slice->getTileGroupApsIdChroma() : -1;
  APS* aps = (chromaAlfApsId >= 0) ? m_apsMap->getPS((chromaAlfApsId << NUM_APS_TYPE_LEN) + ALF_APS) : nullptr;
  if (aps && m_apsMap->getChangedFlag((chromaAlfApsId << NUM_APS_TYPE_LEN) + ALF_APS))
  {
    aps->getCcAlfAPSParam().newCcAlfFilter[0] = false;
    aps->getCcAlfAPSParam().newCcAlfFilter[1] = false;
  }

  if (!cs.slice->getSPS()->getCCALFEnabledFlag())
  {
#if JVET_AI0084_ALF_RESIDUALS_SCALING
    if ( cs.slice->getUseAlfScale() && cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y) )
    {
      storeAlfScalePrev( cs, *cs.slice );
    }
    alfCorrectionChroma( cs, recYuvSao );
#endif
    return;
  }

  m_tempBuf.get(COMPONENT_Cb).copyFrom(cs.getRecoBuf().get(COMPONENT_Cb));
  m_tempBuf.get(COMPONENT_Cr).copyFrom(cs.getRecoBuf().get(COMPONENT_Cr));
  recYuv = m_tempBuf.getBuf(cs.area);
#if JVET_AA0095_ALF_LONGER_FILTER 
  mirroredPaddingForAlf(cs, recYuv, MAX_ALF_FILTER_LENGTH >> 1, false, true);
#else
  recYuv.extendBorderPel(MAX_ALF_FILTER_LENGTH >> 1);
#endif

  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag, ctxStartCcAlf);
  deriveCcAlfFilter(cs, COMPONENT_Cb, orgYuv, recYuv, cs.getRecoBuf());
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag, ctxStartCcAlf);
  deriveCcAlfFilter(cs, COMPONENT_Cr, orgYuv, recYuv, cs.getRecoBuf());

  xSetupCcAlfAPS(cs);

  for (int compIdx = 1; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
  {
    ComponentID compID     = ComponentID(compIdx);
    if (m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
    {
#if JVET_AH0057_CCALF_COEFF_PRECISION
      applyCcAlfFilter(cs, compID, cs.getRecoBuf().get(compID), recYuv, m_ccAlfFilterControl[compIdx - 1], m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1], -1, m_ccAlfFilterParam.ccAlfCoeffPrec[compIdx - 1]);
#else
      applyCcAlfFilter(cs, compID, cs.getRecoBuf().get(compID), recYuv, m_ccAlfFilterControl[compIdx - 1], m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1], -1);
#endif
    }
  }

#if JVET_AI0084_ALF_RESIDUALS_SCALING
  if ( cs.slice->getUseAlfScale() && cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y) )
  {
    storeAlfScalePrev(cs, *cs.slice);
  }
  alfCorrectionChroma( cs, recYuvSao );
#endif

#if JVET_AK0065_TALF
  for(int i = 0; i < m_numCTUsInPic; i++)
  {
    m_tAlfCtbControl[i].reset();
  }
  TAlfControl talfControl;
  cs.slice->setTileGroupTAlfControl(talfControl);
  if( cs.sps->getUseTAlf() && !cs.slice->isIntra() && !cs.sps->getRprEnabledFlag())
  {
    getRefPics(cs);
    const ComponentID compId = COMPONENT_Y;
    PelUnitBuf bestRec = m_tempSaveTALF[0].getBuf( cs.area );
    PelUnitBuf recTALF = m_tempSaveTALF[1].getBuf( cs.area );
    bestRec.get(compId).copyFrom(cs.getRecoBuf(compId));
    deriveStatsForTAlfFilter(cs, compId, orgYuv, recYuv);
    double bestCost = getRecoDist(orgYuv, bestRec, compId, cs);
    std::vector<CandTALF> talfCandList;
    CandTALF bestCand;
    m_CABACEstimator->getCtx() = SubCtx(Ctx::TAlfFilterControlFlag, ctxStartTAlf);
    deriveTAlfFilter(cs, compId, talfCandList);
    for (CandTALF cand: talfCandList)
    {
      recTALF.get(compId).copyFrom(cs.getRecoBuf(compId));
      applyTAlfFilter(cs, compId, recTALF, recYuv, cand.params, cand.talfCtbParam.data(), cand.talfControl);
      double cost = getRecoDist(orgYuv, recTALF, compId, cs) + cand.rate * m_lambda[compId];
      if (cost < bestCost)
      {
        bestCost = cost;
        bestCand = cand;
        bestRec.get(compId).copyFrom(recTALF.get(compId));
      }
    }
    setTAlfAPS( cs, bestCand );
    if(bestCand.talfControl.enabledFlag)
    {
      cs.getRecoBuf(compId).copyFrom(bestRec.get(compId));
    }
  }
#endif
}

double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                                       const double chromaWeight,
#endif
                                                       const int numClasses, const int numCoeff, double& distUnfilter
#if ALF_IMPROVEMENT
                                                       , int fixedFilterSetIdx
#endif
  , bool* flagChanged)
{
  if (flagChanged)
  {
    *flagChanged = false;
  }
  TempCtx        ctxTempStart( m_ctxCache );
  TempCtx        ctxTempBest( m_ctxCache );
  TempCtx        ctxTempAltStart( m_ctxCache );
  TempCtx        ctxTempAltBest( m_ctxCache );
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
#if ALF_IMPROVEMENT
  const int numAlts = isLuma(channel) ? m_alfParamTemp.numAlternativesLuma : m_alfParamTemp.numAlternativesChroma;
#else
  const int numAlts = isLuma( channel ) ? 1 : m_alfParamTemp.numAlternativesChroma;
#endif
  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfParamTemp, channel, true);
#if ENABLE_QPA
  CHECK ((chromaWeight > 0.0) && (cs.slice->getFirstCtuRsAddrInSlice() != 0), "incompatible start CTU address, must be 0");
#endif

  reconstructCoeff(m_alfParamTemp, channel, true, isLuma(channel));
#if ALF_IMPROVEMENT
  for( int altIdx = 0; altIdx < (isLuma(channel) ? MAX_NUM_ALF_ALTERNATIVES_LUMA : MAX_NUM_ALF_ALTERNATIVES_CHROMA); altIdx++ )
#else
  for( int altIdx = 0; altIdx < (isLuma(channel) ? 1 : MAX_NUM_ALF_ALTERNATIVES_CHROMA); altIdx++)
#endif
  {
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
#if ALF_IMPROVEMENT
        m_filterCoeffSet[altIdx*numClasses + classIdx][i] = isLuma(channel) ? m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[altIdx][i];
        m_filterClippSet[altIdx*numClasses + classIdx][i] = isLuma(channel) ? m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[altIdx][i];
#else
        m_filterCoeffSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[altIdx][i];
        m_filterClippSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[altIdx][i];
#endif
      }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      m_filterScaleIdx[altIdx * numClasses + classIdx] = isLuma(channel) ? m_scaleIdxFinal[altIdx][classIdx] : m_chromaScaleIdxFinal[altIdx][0];
#endif
    }
  }

  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
      int bestBeforeAltIdx = m_ctuEnableFlag[compID][ctuIdx] ? m_ctuAlternative[compID][ctuIdx] : -1;
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight) : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif

      double distUnfilterCtu = m_ctbDistortionUnfilter[compID][ctuIdx];

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;

      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfParamTemp );

      if( isLuma( channel ) )
      {
        // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
        CHECKD( cs.slice->getPic()->getAlfCtbFilterIndex()[ctuIdx] != NUM_FIXED_FILTER_SETS, "Wrong getAlfCtbFilterIndex");
        CHECKD( cs.slice->getTileGroupNumAps() != 1, "Wrong getTileGroupNumAps");

        m_CABACEstimator->codeAlfCtuFilterIndex( cs, ctuIdx, m_alfParamTemp.enabledFlag[COMPONENT_Y] );
      }
      double costOn = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );

      if( isLuma( channel ) )
      {
#if ALF_IMPROVEMENT
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx(ctxTempBest);
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
          {
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          }
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compID][ctuIdx] = altIdx;
          m_CABACEstimator->codeAlfCtuAlternative(cs, ctuIdx, compID, &m_alfParamTemp, numAlts);
          double r_altCost = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#if JVET_X0071_ALF_BAND_CLASSIFIER
          int classifierIdx = m_classifierFinal[altIdx];
#if ALF_PRECISION_VARIETY
          double altDist = getFilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx][fixedFilterSetIdx][classifierIdx], ALF_NUM_CLASSES_CLASSIFIER[classifierIdx], m_alfParamTemp.numLumaFilters[altIdx] - 1, numCoeff, altIdx, m_coeffBitsFinal[altIdx]);
#else
          double altDist = getFilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx][fixedFilterSetIdx][classifierIdx], ALF_NUM_CLASSES_CLASSIFIER[classifierIdx], m_alfParamTemp.numLumaFilters[altIdx] - 1, numCoeff, altIdx);
#endif
#else
          double altDist = getFilteredDistortion(m_alfCovariance[compID][iShapeIdx][ctuIdx][fixedFilterSetIdx], numClasses, m_alfParamTemp.numLumaFilters[altIdx] - 1, numCoeff, altIdx);
#endif
          double altCost = altDist + r_altCost;

          if (altCost < bestAltCost)
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
            ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
          }
        }
        m_ctuAlternative[compID][ctuIdx] = bestAltIdx;
        costOn += bestAltCost;
#else
        costOn += getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfParamTemp.numLumaFilters - 1, numCoeff );
#endif
      }
      else
      {
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx( ctxTempBest );
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
          {
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          }
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compID][ctuIdx] = altIdx;
          m_CABACEstimator->codeAlfCtuAlternative( cs, ctuIdx, compID, &m_alfParamTemp );
          double r_altCost = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

          double altDist = 0.;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          const int scaleIdx = m_filterScaleIdx[altIdx];
#else
          const int scaleIdx = 0;
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS_CHROMA, scaleIdx);
#else
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][0][0][0].calcErrorForCoeffs( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS_CHROMA, scaleIdx);
#else
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][0][0][0].calcErrorForCoeffs( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif
#endif
#else
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][0][0].calcErrorForCoeffs(  m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
          altDist += m_alfCovariance[compID][iShapeIdx][ctuIdx][0].calcErrorForCoeffs(  m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif

          double altCost = altDist + r_altCost;
          if( altCost < bestAltCost )
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
          }
        }
        m_ctuAlternative[compID][ctuIdx] = bestAltIdx;
        costOn += bestAltCost;
      }

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfParamTemp);
      double costOff = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

      if( costOn < costOff )
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
        m_ctuEnableFlag[compID][ctuIdx] = 1;

      }
      else
      {
        cost += costOff;
        m_ctuEnableFlag[compID][ctuIdx] = 0;
        distUnfilter += distUnfilterCtu;
      }
      int bestAfterAltIdx = m_ctuEnableFlag[compID][ctuIdx] ? m_ctuAlternative[compID][ctuIdx] : -1;
      if (bestBeforeAltIdx != bestAfterAltIdx)
      {
        if (flagChanged)
        {
          *flagChanged = true;
        }
      }
    }
  }

  if( isChroma( channel ) )
  {
    setEnableFlag(m_alfParamTemp, channel, m_ctuEnableFlag);
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfParam& alfParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                                      , const double lambdaChromaWeight // = 0.0
#endif
                                      )
{
  const TempCtx  ctxStart( m_ctxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_ctxCache );

  double costMin = MAX_DOUBLE;
  std::vector<AlfFilterShape>& alfFilterShape = alfParam.filterShapes[channel];
  m_bitsNewFilter[channel] = 0;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  int bestShapeIdx = -1, bestFixedFilterSetIdx = -1;
  for (int withSA = 0; withSA < 2; withSA++)
  {
    if (withSA && (bestShapeIdx < 0 || bestFixedFilterSetIdx < 0))
    {
      // ALF is disabled
      break;
    }
    int iShapeIdxFrom = 0, iShapeIdxTo = (int)alfFilterShape.size();
    if (withSA)
    {
      iShapeIdxFrom = bestShapeIdx, iShapeIdxTo = bestShapeIdx + 1;
    }
    for (int iShapeIdx = iShapeIdxFrom; iShapeIdx < iShapeIdxTo; iShapeIdx++)
#else
    for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
#endif
    {
#if ALF_IMPROVEMENT
      if (m_filterTypeTest[channel][alfFilterShape[iShapeIdx].filterType] == false)
      {
        continue;
      }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      int numFixedFilterSet = numFixedFilters( alfFilterShape[iShapeIdx].filterType );
#if FIXFILTER_CFG
      if(m_encCfg->getUseAlfFixedFilter() == false)
      {
        numFixedFilterSet = 1;
      }
#endif
#else
      int numFixedFilterSet = ( alfFilterShape[iShapeIdx].filterType == ALF_FILTER_EXT || alfFilterShape[iShapeIdx].filterType == ALF_FILTER_9_EXT ) ? 2 : 1;
#endif
#endif
      m_alfParamTemp = alfParam;
      //1. get unfiltered distortion
      if( isChroma( channel ) )
      {
        m_alfParamTemp.numAlternativesChroma = 1;
      }
#if ALF_IMPROVEMENT
      else
      {
        m_alfParamTemp.numAlternativesLuma = 1;
      }
#endif

      double cost = isLuma( channel ) ? m_unFiltDistCompnent[COMPONENT_Y] : m_unFiltDistCompnent[COMPONENT_Cb] + m_unFiltDistCompnent[COMPONENT_Cr];
      cost /= 1.001; // slight preference for unfiltered choice

      if( cost < costMin )
      {
        costMin = cost;
        setEnableFlag( alfParam, channel, false );

        // no CABAC signalling
        ctxBest = AlfCtx( ctxStart );
        setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
        if( isChroma( channel ) )
        {
          setCtuAlternativeChroma( m_ctuAlternativeTmp, 0 );
        }
#if ALF_IMPROVEMENT
        else
        {
          std::fill_n( m_ctuAlternativeTmp[COMPONENT_Y], m_numCTUsInPic, 0 );
        }
#endif
      }
#if ALF_IMPROVEMENT
      const int nonLinearFlagMax = 1;
#else
      const int nonLinearFlagMax =
        ( isLuma( channel ) ? m_encCfg->getUseNonLinearAlfLuma() : m_encCfg->getUseNonLinearAlfChroma()) // For Chroma non linear flag is check for each alternative filter
        ? 2 : 1;
#endif
      for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
      {
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int fixedFilterSetIdxFrom = 0, fixedFilterSetIdxTo = numFixedFilterSet;
        if (withSA)
        {
          fixedFilterSetIdxFrom = bestFixedFilterSetIdx, fixedFilterSetIdxTo = bestFixedFilterSetIdx + 1;
        }
        for (int fixedFilterSetIdx = fixedFilterSetIdxFrom; fixedFilterSetIdx < fixedFilterSetIdxTo; fixedFilterSetIdx++)
#else
        for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
#endif
        {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          int numAlternativesFrom = 1, numAlternativesTo = (isLuma(channel) ? std::max(1, std::min(m_numCTUsInPic, MAX_NUM_ALF_ALTERNATIVES_LUMA)) : getMaxNumAlternativesChroma());
          if (withSA)
          {
            numAlternativesFrom = (isLuma(channel) ? alfParam.numAlternativesLuma : alfParam.numAlternativesChroma), numAlternativesTo = numAlternativesFrom;
          }
          for (int numAlternatives = numAlternativesFrom; numAlternatives <= numAlternativesTo; numAlternatives++)
#else
          for( int numAlternatives = 1; numAlternatives <= (isLuma(channel) ? std::max(1, std::min(m_numCTUsInPic, MAX_NUM_ALF_ALTERNATIVES_LUMA)) : getMaxNumAlternativesChroma()); numAlternatives++ )
#endif
#else
          for( int numAlternatives = isLuma( channel ) ? 1 : getMaxNumAlternativesChroma(); numAlternatives > 0; numAlternatives-- )
#endif
          {
            if( isChroma( channel ) )
            {
              m_alfParamTemp.numAlternativesChroma = numAlternatives;
            }
#if ALF_IMPROVEMENT
            else
            {
              m_alfParamTemp.numAlternativesLuma = numAlternatives;
            }
            m_alfParamTemp.filterType[channel] = alfFilterShape[iShapeIdx].filterType;
            bool update = false;
#else
            m_alfParamTemp.nonLinearFlag[channel] = nonLinearFlag;
#endif
#if !ALF_RD_COST_BUG_FIX
            m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
#endif

            //2. all CTUs are on
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            if (withSA)
            {
              // restore the best parameters
              copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, channel);
              if (isChroma(channel))
              {
                copyCtuAlternativeChroma(m_ctuAlternative, m_ctuAlternativeTmp);
              }
              else
              {
#if ALF_IMPROVEMENT
                std::copy_n(m_ctuAlternativeTmp[COMPONENT_Y], m_numCTUsInPic, m_ctuAlternative[COMPONENT_Y]);
#endif
              }
            }
            else
#endif
            {
              setEnableFlag( m_alfParamTemp, channel, true );
              setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );

              // all alternatives are on
              if( isChroma( channel ) )
              {
                initCtuAlternativeChroma( m_ctuAlternative );
              }
#if ALF_IMPROVEMENT
              else
              {
                initCtuAlternativeLuma( m_ctuAlternative );
              }
            }
#endif

#if ALF_RD_COST_BUG_FIX
            //3. CTU decision
            double distUnfilter = 0;
            const int iterNum = isLuma(channel) ? 5 : (2 + m_alfParamTemp.numAlternativesChroma);
            bool flagChanged = true;
            int bestIter = -1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            int maxIterNoChange = withSA ? 2 : 1;
#else
            int maxIterNoChange = 1;
#endif
            double bestIterCost = MAX_DOUBLE;
            for (int iter = 0; iter < iterNum && flagChanged; iter++)
            {
              // unfiltered distortion is added due to some CTBs may not use filter
              // no need to reset CABAC here, since uiCoeffBits is not affected
              m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
              cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits
#if ALF_IMPROVEMENT
                , fixedFilterSetIdx
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                , withSA
#else
                , false
#endif
              );

              m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
              cost = m_lambda[channel] * uiCoeffBits;
              cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel,
#if ENABLE_QPA
                lambdaChromaWeight,
#endif
                numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter
#if ALF_IMPROVEMENT
                , fixedFilterSetIdx
#endif
                , &flagChanged);
              if (cost < costMin)
              {
#if ALF_IMPROVEMENT
                update = true;
#endif
                costMin = cost;
                ctxBest = AlfCtx(m_CABACEstimator->getCtx());
                copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
                if (isChroma(channel))
                {
                  copyCtuAlternativeChroma(m_ctuAlternativeTmp, m_ctuAlternative);
                }
                else
                {
#if ALF_IMPROVEMENT
                  std::copy_n(m_ctuAlternative[COMPONENT_Y], m_numCTUsInPic, m_ctuAlternativeTmp[COMPONENT_Y]);
#endif
                }
                m_bitsNewFilter[channel] = uiCoeffBits;
                copyAlfParam(alfParam, m_alfParamTemp, channel);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                bestShapeIdx = iShapeIdx;
#if ALF_IMPROVEMENT
                bestFixedFilterSetIdx = fixedFilterSetIdx;
#endif
#endif
              }
              if (cost < bestIterCost)
              {
                bestIter = iter;
                bestIterCost = cost;
              }
              else if (iter >= bestIter + maxIterNoChange)
              {
                // High probability that we have converged or we are diverging
                break;
              }
            }//for iter
#else
            // RD-cost is not properly compared in this branch:
            // the results of getFilterCoeffAndCost and deriveCtbAlfEnableFlags are different due to the floating-point implementation
            cost = getFilterCoeffAndCost( cs, 0, channel, true, iShapeIdx, uiCoeffBits
#if ALF_IMPROVEMENT
              , fixedFilterSetIdx
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              , withSA
#else
              , false
#endif
            );
            if( cost < costMin )
            {
#if ALF_IMPROVEMENT
              update = true;
#endif
              costMin = cost;
              m_bitsNewFilter[channel] = uiCoeffBits;
              copyAlfParam( alfParam, m_alfParamTemp, channel );
              ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
              setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );

              if( isChroma( channel ) )
              {
                copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
              }
#if ALF_IMPROVEMENT
              else
              {
                std::copy_n( m_ctuAlternative[COMPONENT_Y], m_numCTUsInPic, m_ctuAlternativeTmp[COMPONENT_Y] );
              }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              bestShapeIdx = iShapeIdx;
#if ALF_IMPROVEMENT
              bestFixedFilterSetIdx = fixedFilterSetIdx;
#endif
#endif
            }

            //3. CTU decision
            double distUnfilter = 0;
#if ALF_IMPROVEMENT
            double prevItCost = cost;
#else
            double prevItCost = MAX_DOUBLE;
#endif
            const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * (2 + m_alfParamTemp.numAlternativesChroma - 1) + 1);

            for( int iter = 0; iter < iterNum; iter++ )
            {
              if ((iter & 0x01) == 0)
              {
                m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
                cost = m_lambda[channel] * uiCoeffBits;
                cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel,
#if ENABLE_QPA
                                                lambdaChromaWeight,
#endif
                                                numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter
#if ALF_IMPROVEMENT
                                                , fixedFilterSetIdx
#endif
                                                , nullptr);
                if (cost < costMin)
                {
#if ALF_IMPROVEMENT
                  update = true;
#endif
                  costMin = cost;
                  ctxBest = AlfCtx(m_CABACEstimator->getCtx());
                  copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
                  if( isChroma( channel ) )
                  {
                    copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
                  }
                  else
                  {
#if ALF_IMPROVEMENT
                    std::copy_n( m_ctuAlternative[COMPONENT_Y], m_numCTUsInPic, m_ctuAlternativeTmp[COMPONENT_Y] );
#endif
                  }
                  m_bitsNewFilter[channel] = uiCoeffBits;
                  copyAlfParam(alfParam, m_alfParamTemp, channel);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                  bestShapeIdx = iShapeIdx;
#if ALF_IMPROVEMENT
                  bestFixedFilterSetIdx = fixedFilterSetIdx;
#endif
#endif
                }
                else if( cost >= prevItCost )
                {
                  // High probability that we have converged or we are diverging
                  break;
                }
                prevItCost = cost;
              }
              else
              {
                // unfiltered distortion is added due to some CTBs may not use filter
                // no need to reset CABAC here, since uiCoeffBits is not affected
                /*cost = */getFilterCoeffAndCost( cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits 
#if ALF_IMPROVEMENT
                  , fixedFilterSetIdx
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                  , withSA
#else
                  , false
#endif
                );
              }
            }//for iter
#endif
#if ALF_IMPROVEMENT
            if (update == false && numAlternatives > 1)
            {
              break;
            }
#endif
          }// for number of alternatives and reset ctu params and filters
#if ALF_IMPROVEMENT
        }//for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++)
#endif
      }// for nonLineaFlag
    }//for shapeIdx
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  }//for withSA
#endif

  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
  if( isChroma( channel ) )
  {
    copyCtuAlternativeChroma( m_ctuAlternative, m_ctuAlternativeTmp );
  }
  else
  {
#if ALF_IMPROVEMENT
    std::copy_n( m_ctuAlternativeTmp[COMPONENT_Y], m_numCTUsInPic, m_ctuAlternative[COMPONENT_Y] );
#endif

  }
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );
}

void EncAdaptiveLoopFilter::copyAlfParam( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    std::copy_n( reinterpret_cast< const char * >( &alfParamSrc ), sizeof( AlfParam ), reinterpret_cast< char * >( &alfParamDst ) );
  }
  else
  {
    alfParamDst.enabledFlag[COMPONENT_Cb] = alfParamSrc.enabledFlag[COMPONENT_Cb];
    alfParamDst.enabledFlag[COMPONENT_Cr] = alfParamSrc.enabledFlag[COMPONENT_Cr];
    alfParamDst.numAlternativesChroma = alfParamSrc.numAlternativesChroma;
#if ALF_IMPROVEMENT
    alfParamDst.filterType[CHANNEL_TYPE_CHROMA] = alfParamSrc.filterType[CHANNEL_TYPE_CHROMA];
    memcpy( alfParamDst.nonLinearFlag[CHANNEL_TYPE_CHROMA], alfParamSrc.nonLinearFlag[CHANNEL_TYPE_CHROMA], sizeof( alfParamDst.nonLinearFlag[CHANNEL_TYPE_CHROMA] ) );
#else
    alfParamDst.nonLinearFlag[CHANNEL_TYPE_CHROMA] = alfParamSrc.nonLinearFlag[CHANNEL_TYPE_CHROMA];
#endif
    memcpy( alfParamDst.chromaCoeff, alfParamSrc.chromaCoeff, sizeof( alfParamDst.chromaCoeff ) );
    memcpy( alfParamDst.chromaClipp, alfParamSrc.chromaClipp, sizeof( alfParamDst.chromaClipp ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    memcpy(alfParamDst.chromaScaleIdx, alfParamSrc.chromaScaleIdx, sizeof(alfParamDst.chromaScaleIdx));
#endif
  }
}

double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, 
#if ALF_IMPROVEMENT
  int fixedFilterSetIdx,
#endif
  bool tryImproveBySA, bool onlyFilterCost )
{
  double dist = distUnfilter;
  uiCoeffBits = 0;
  AlfFilterShape& alfFilterShape = m_alfParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
#if ALF_IMPROVEMENT
    for (int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesLuma; ++altIdx)
    {
#if JVET_X0071_ALF_BAND_CLASSIFIER
      AlfParam bestSliceParam;
      double bestCost = MAX_DOUBLE;
      double bestDist = MAX_DOUBLE;
      int bestCoeffBits = 0;
      for( int classifierIdx = 0; classifierIdx < ALF_NUM_CLASSIFIER; classifierIdx++ )
      {
#endif
#if JVET_AD0222_ALF_RESI_CLASS
        if( classifierIdx == ALF_NUM_CLASSIFIER - 1 && cs.slice->isIntra() )
        {
          continue;
        }
#endif
      //collect stat based on CTU decision
      if ( bReCollectStat )
      {
#if JVET_X0071_ALF_BAND_CLASSIFIER
        getFrameStats( channel, iShapeIdx, altIdx, fixedFilterSetIdx, classifierIdx );
#else
        getFrameStats( channel, iShapeIdx, altIdx 
#if ALF_IMPROVEMENT
          , fixedFilterSetIdx
#endif
        );
#endif
      }

      CHECKD(alfFilterShape.numCoeff != m_alfCovarianceFrame[channel][iShapeIdx][0].numCoeff, "Wrong numCoeff");

#if !JVET_X0071_ALF_BAND_CLASSIFIER
      AlfParam bestSliceParam;
      double bestCost = MAX_DOUBLE;
      double bestDist = MAX_DOUBLE;
      int bestCoeffBits = 0;
#endif
      const int nonLinearFlagMax = m_encCfg->getUseNonLinearAlfLuma() ? 2 : 1;
      for (int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++)
      {
#if JVET_X0071_ALF_BAND_CLASSIFIER
        m_alfParamTemp.lumaClassifierIdx[altIdx] = classifierIdx;
#endif
        m_alfParamTemp.nonLinearFlag[channel][altIdx] = nonLinearFlag;
#if ALF_PRECISION_VARIETY
        for( int i = 0; i < m_alfPrecisionVariety; i++ )
        {
          std::fill_n(m_alfClipMerged[iShapeIdx][i][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel][altIdx] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
        }
#else
        std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel][altIdx] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
#endif

        // Reset Merge Tmp Cov
        m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset();
        m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset();
        int curCoeffBits;
#if JVET_X0071_ALF_BAND_CLASSIFIER
        double curDist = mergeFiltersAndCost(m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], curCoeffBits, altIdx, classifierIdx, m_alfParamTemp.numLumaFilters[altIdx], tryImproveBySA);
#else
        double curDist = mergeFiltersAndCost(m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], curCoeffBits, altIdx, tryImproveBySA);
#endif
        double cost = curDist + m_lambda[channel] * curCoeffBits;
        if (cost < bestCost)
        {
          bestCost = cost;
          bestDist = curDist;
          bestCoeffBits = curCoeffBits;
          bestSliceParam = m_alfParamTemp;
        }
      }//for (int nonLinearFlag)
#if JVET_X0071_ALF_BAND_CLASSIFIER
      }//for(int classifierIdx)
#endif
      uiCoeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    } //for (int altIdx
    uiCoeffBits += lengthUvlc(m_alfParamTemp.numAlternativesLuma - 1);
    uiCoeffBits += m_alfParamTemp.numAlternativesLuma; // non-linear flags   
#else
    //collect stat based on CTU decision
    if( bReCollectStat )
    {
      getFrameStats( channel, iShapeIdx, 0 );
    }
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset(AlfNumClippingValues[channel]);
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset(AlfNumClippingValues[channel]);
    //distortion
    dist += mergeFiltersAndCost( m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], uiCoeffBits, tryImproveBySA );
#endif
  }
  else
  {
    //distortion
    CHECKD( m_alfParamTemp.numAlternativesChroma < 1, "Wrong number of m_alfParamTemp.numAlternativesChroma" );

    for( int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesChroma; ++altIdx )
    {
      //collect stat based on CTU decision
      if ( bReCollectStat )
      {
#if JVET_X0071_ALF_BAND_CLASSIFIER
        getFrameStats( channel, iShapeIdx, altIdx, fixedFilterSetIdx, 0 );
#else
        getFrameStats( channel, iShapeIdx, altIdx 
#if ALF_IMPROVEMENT
          , fixedFilterSetIdx
#endif
        );
#endif
      }

      CHECKD(alfFilterShape.numCoeff != m_alfCovarianceFrame[channel][iShapeIdx][0].numCoeff, "Wrong numCoeff");

      AlfParam bestSliceParam;
      double bestCost = MAX_DOUBLE;
      double bestDist = MAX_DOUBLE;
      int bestCoeffBits = 0;
      const int nonLinearFlagMax = m_encCfg->getUseNonLinearAlfChroma() ? 2 : 1;

      for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
      {
#if ALF_IMPROVEMENT
        m_alfParamTemp.nonLinearFlag[channel][altIdx] = nonLinearFlag;
#else
        int currentNonLinearFlag = m_alfParamTemp.nonLinearFlag[channel] ? 1 : 0;
        if (nonLinearFlag != currentNonLinearFlag)
        {
          continue;
        }
#endif
        std::fill_n( m_filterClippSet[altIdx], MAX_NUM_ALF_CHROMA_COEFF, nonLinearFlag ? AlfNumClippingValues[CHANNEL_TYPE_CHROMA] / 2 : 0 );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        char& scaleIdxRef = m_filterScaleIdx[altIdx];
#else
        char scaleIdxRef;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
        double dist = m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS_CHROMA, nonLinearFlag, false, 1, m_lambda[channel], scaleIdxRef, tryImproveBySA);
#else
        double dist = m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS, nonLinearFlag, false, 1, m_lambda[channel], scaleIdxRef, tryImproveBySA);
#endif
        for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
        {
          m_alfParamTemp.chromaCoeff[altIdx][i] = m_filterCoeffSet[altIdx][i];
          m_alfParamTemp.chromaClipp[altIdx][i] = m_filterClippSet[altIdx][i];
        }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        m_alfParamTemp.chromaScaleIdx[altIdx][0] = m_filterScaleIdx[altIdx];
#endif
        int coeffBits = getChromaCoeffRate( m_alfParamTemp, altIdx );
        double cost = dist + m_lambda[channel] * coeffBits;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
        if (tryImproveBySA)
        {
          short prevChromaCoeff[MAX_NUM_ALF_CHROMA_COEFF];
          Pel prevChromaClipp[MAX_NUM_ALF_CHROMA_COEFF];
          std::copy_n(m_alfParamTemp.chromaCoeff[altIdx], MAX_NUM_ALF_CHROMA_COEFF, prevChromaCoeff);
          std::copy_n(m_alfParamTemp.chromaClipp[altIdx], MAX_NUM_ALF_CHROMA_COEFF, prevChromaClipp);
          char prevChromaScaleIdx = m_filterScaleIdx[altIdx];

          double distSA = m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + tryImproveCoeffQuant(m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_alfCovarianceFrame[channel][iShapeIdx][0], alfFilterShape, m_NUM_BITS_CHROMA, nonLinearFlag, false, 1, m_lambda[channel], m_filterScaleIdx[altIdx]);
          for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
          {
            m_alfParamTemp.chromaCoeff[altIdx][i] = m_filterCoeffSet[altIdx][i];
            m_alfParamTemp.chromaClipp[altIdx][i] = m_filterClippSet[altIdx][i];
          }
          m_alfParamTemp.chromaScaleIdx[altIdx][0] = m_filterScaleIdx[altIdx];
          int coeffBitsSA = getChromaCoeffRate(m_alfParamTemp, altIdx);
          double costSA = distSA + m_lambda[channel] * coeffBitsSA;

          if (costSA < cost)
          {
            cost = costSA;
            dist = distSA;
            coeffBits = coeffBitsSA;
          }
          else
          {
            std::copy_n(prevChromaCoeff, MAX_NUM_ALF_CHROMA_COEFF, m_alfParamTemp.chromaCoeff[altIdx]);
            std::copy_n(prevChromaClipp, MAX_NUM_ALF_CHROMA_COEFF, m_alfParamTemp.chromaClipp[altIdx]);
            m_alfParamTemp.chromaScaleIdx[altIdx][0] = prevChromaScaleIdx;
          }
        }
#endif

        if( cost < bestCost )
        {
          bestCost = cost;
          bestDist = dist;
          bestCoeffBits = coeffBits;
          bestSliceParam = m_alfParamTemp;
        }
      }
      uiCoeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    }
    uiCoeffBits += lengthUvlc( m_alfParamTemp.numAlternativesChroma-1 );
#if ALF_IMPROVEMENT
    uiCoeffBits += m_alfParamTemp.numAlternativesChroma; // non-linear flags
#else
    uiCoeffBits++;
#endif
  }
  if (onlyFilterCost)
  {
    return dist + m_lambda[channel] * uiCoeffBits;
  }
  double rate = uiCoeffBits;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfParamTemp);
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    if( isLuma( channel ) )
    {
      // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
      CHECK( cs.slice->getPic()->getAlfCtbFilterIndex()[ctuIdx] != NUM_FIXED_FILTER_SETS, "Wrong getAlfCtbFilterIndex");
      CHECK( cs.slice->getTileGroupNumAps() != 1, "Wrong getTileGroupNumAps");

      m_CABACEstimator->codeAlfCtuFilterIndex( cs, ctuIdx, m_alfParamTemp.enabledFlag[COMPONENT_Y] );
#if ALF_IMPROVEMENT
      m_CABACEstimator->codeAlfCtuAlternative(cs, ctuIdx, COMPONENT_Y, &m_alfParamTemp, m_alfParamTemp.numAlternativesLuma);
#endif
    }
  }
  m_CABACEstimator->codeAlfCtuAlternatives( cs, channel, &m_alfParamTemp );
  rate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getChromaCoeffRate( AlfParam& alfParam, int altIdx )
{
  int iBits = 0;
#if ALF_IMPROVEMENT
  AlfFilterShape alfShape = m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][alfParam.filterType[CHANNEL_TYPE_CHROMA]]];
#else
  AlfFilterShape alfShape(5);
#endif
  // Filter coefficients
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    HuffmanForALF huffman(false, m_NUM_BITS_CHROMA, 1, orderIdx);
    huffman.init();
    for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
    {
      iBits += lengthHuffman(alfParam.chromaCoeff[altIdx][coeffIdx], huffman);
    }
  }
#else
  for( int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++ )
  {
    int minBits = MAX_INT;
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    for( int k = 0; k < 4; k++ )
    {
      int curBits = 0;
      for( int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++ )
      {
        curBits += EncAdaptiveLoopFilter::lengthGolomb( alfParam.chromaCoeff[altIdx][coeffIdx], k + ( orderIdx == 0 ? alfShape.offset0 : ALF_ORDER ) );
      }
      if (curBits < minBits)
      {
        minBits = curBits;
      }
    }
    iBits += minBits + 2;
  }
#endif
#else
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
    iBits += lengthUvlc( abs( alfParam.chromaCoeff[ altIdx ][ i ] ) );  // alf_coeff_chroma[altIdx][i]
    if( ( alfParam.chromaCoeff[ altIdx ][ i ] ) != 0 )
      iBits += 1;
  }
#endif
#if ALF_IMPROVEMENT
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx] )
#else
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_CHROMA] )
#endif
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if( !abs( alfParam.chromaCoeff[altIdx][i] ) )
      {
        alfParam.chromaClipp[altIdx][i] = 0;
      }
    }
    iBits += ((alfShape.numCoeff - 1) << 1);
  }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  iBits += AdaptiveLoopFilter::m_SCALE_BITS_NUM;
#endif
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel )
{
  double dist = 0;
  if( isLuma( channel ) )
  {
    dist = getUnfilteredDistortion( cov, MAX_NUM_ALF_CLASSES );
  }
  else
  {
    dist = getUnfilteredDistortion( cov, 1 );
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, const int numClasses )
{
  double dist = 0;
  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

double EncAdaptiveLoopFilter::getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff 
#if ALF_IMPROVEMENT
  , int altIdx
#endif
#if ALF_PRECISION_VARIETY
  , int coeffBits
#endif
)
{
  double dist = 0;

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
#if ALF_IMPROVEMENT
    const int scaleIdx = m_filterScaleIdx[altIdx * numClasses + classIdx];
#else
    const int scaleIdx = m_filterScaleIdx[classIdx];
#endif
#else
    const int scaleIdx = 0;
#endif
#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[altIdx * numClasses + classIdx], m_filterCoeffSet[altIdx * numClasses + classIdx], numCoeff, coeffBits, scaleIdx);
#else
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[altIdx * numClasses + classIdx], m_filterCoeffSet[altIdx * numClasses + classIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[classIdx], m_filterCoeffSet[classIdx], numCoeff, m_NUM_BITS, scaleIdx);
#endif
  }

  return dist;
}

#if JVET_X0071_ALF_BAND_CLASSIFIER
#if ALF_PRECISION_VARIETY
double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits, int altIdx, int classifierIdx, int numFiltersLinear
#else
double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits, int altIdx, int classifierIdx, int numFiltersLinear 
#endif
  , bool tryImproveBySA
)
#else
double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits 
#if ALF_IMPROVEMENT
  , int altIdx
#endif
  , bool tryImproveBySA
)
#endif
{
  bool tryImproveScale = tryImproveBySA;
  int numFiltersBest = 0;
#if JVET_X0071_ALF_BAND_CLASSIFIER
  int numFilters = ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];
#else
  int numFilters = MAX_NUM_ALF_CLASSES;
#endif
  bool   codedVarBins[MAX_NUM_ALF_CLASSES];
#if ALF_PRECISION_VARIETY
  double errorForce0CoeffTab[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][2];
#else
  double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];
#endif
  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int coeffBits, coeffBitsForce0;
#if ALF_RD_COST_BUG_FIX
  int nonFilterBits;
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  int maxNumFilters = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] ? std::min( numFiltersLinear + 3, numFilters ) : numFilters;
  int bestCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  int bestClipp[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char bestScaleIdx[MAX_NUM_ALF_CLASSES];
#endif
  double bestDist = 0;
  int bestBits = 0;
  bool   bestCodedVarBins[MAX_NUM_ALF_CLASSES] = { false };
  bool   bestAlfLumaCoeffDeltaFlag = false;
  static int mergedPair[MAX_NUM_ALF_CLASSES][2] = { { 0 } };
#if ALF_PRECISION_VARIETY
  int mergedCoeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF] = { { { 0 } } };
  char mergedScaleIdx[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES] = { { 0 } };
  double mergedErr[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES] = { { 0 } };
#else
  int mergedCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF] = { { 0 } };
  char mergedScaleIdx[MAX_NUM_ALF_CLASSES] = { 0 };
  double mergedErr[MAX_NUM_ALF_CLASSES] = { 0 };
#endif
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] == false )
  {
    memset( mergedPair, 0, sizeof( mergedPair ) );
#if ALF_PRECISION_VARIETY
    mergeClasses( alfShape, covFrame, covMerged, clipMerged[0], numFilters, m_filterIndices, altIdx, mergedPair );
#else
    mergeClasses( alfShape, covFrame, covMerged, clipMerged, numFilters, m_filterIndices, altIdx, mergedPair );
#endif
  }
  else
  {
    for (int i = 0; i <= MAX_NUM_ALF_CLASSES; i++)
    {
      covMerged[i].numBins = AlfNumClippingValues[COMPONENT_Y];
    }
    for (int i = 0; i < numFilters; i++)
    {
      for (int j = 0; j < numFilters; j++)
      {
#if ALF_PRECISION_VARIETY
        for (int k = 0; k < m_alfPrecisionVariety; k++)
        {
          std::fill_n(clipMerged[k][i][j], MAX_NUM_ALF_LUMA_COEFF, 2);
        }
#else
        std::fill_n(clipMerged[i][j], MAX_NUM_ALF_LUMA_COEFF, 2);
#endif
      }
    }
  }
  numFilters = maxNumFilters;
#else
  mergeClasses( alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices 
#if ALF_IMPROVEMENT
    , altIdx
#endif
  );
#endif
#if ALF_PRECISION_VARIETY
  char bestCoeffBits = 8;
  char minCoeffBits = 8, maxCoeffBits = 8;
  char bestCoeffMantissa = 1;
  char minCoeffMantissa = 1, maxCoeffMantissa = 1;
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  if( m_encCfg->getUseAlfPrecision() == true )
  {
    minCoeffBits = 6, maxCoeffBits = 9;
  }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  minCoeffMantissa = 1, maxCoeffMantissa = 2;
#endif
  numFilters = maxNumFilters;
  while (numFilters >= 1)
  {
    alfParam.numLumaFilters[altIdx] = numFilters;
#if ALF_PRECISION_VARIETY
#if JVET_X0071_ALF_BAND_CLASSIFIER
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], classifierIdx, numFilters == maxNumFilters ? true : false, mergedPair, mergedCoeff, mergedScaleIdx, mergedErr, tryImproveScale);
#else
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam
#if ALF_IMPROVEMENT
      , m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx]
#endif
      , tryImproveScale
    );
#endif
    for (char coeffMantissa = minCoeffMantissa; coeffMantissa <= maxCoeffMantissa; coeffMantissa++)
    {
      if (!tryImproveScale)
      {
        if (coeffMantissa != maxCoeffMantissa)
        {
          continue;
        }
      }
      for( char coeffBitDepth = minCoeffBits; coeffBitDepth <= maxCoeffBits; coeffBitDepth++ )
      {
        int bitIdx = (coeffMantissa - minCoeffMantissa) * m_numBitPrecision + (coeffBitDepth - minCoeffBits);
        dist = 0.0;
        for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
        {
          dist += errorForce0CoeffTab[bitIdx][filtIdx][1];
        }
#else
#if JVET_X0071_ALF_BAND_CLASSIFIER
        dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], classifierIdx, numFilters == maxNumFilters ? true : false, mergedPair, mergedCoeff, mergedScaleIdx, mergedErr, tryImproveScale);
#else
        dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam
#if ALF_IMPROVEMENT
          , m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx]
#endif
          , tryImproveScale
        );
#endif
#endif
        // filter coeffs are stored in m_filterCoeffSet
#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
        distForce0 = getDistForce0(alfShape, numFilters, errorForce0CoeffTab[bitIdx], codedVarBins, altIdx, bitIdx, true, coeffBitDepth, coeffMantissa);
        coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], bitIdx, true, coeffBitDepth, coeffMantissa);
        coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins, altIdx, bitIdx, true, coeffBitDepth, coeffMantissa);
#else
        distForce0 = getDistForce0(alfShape, numFilters, errorForce0CoeffTab, codedVarBins, altIdx);
        coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx]);
        coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins, altIdx);
#endif
#if ALF_RD_COST_BUG_FIX
#if JVET_X0071_ALF_BAND_CLASSIFIER
        nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx, classifierIdx);
#else
        nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx);
#endif
#endif
#else
        distForce0 = getDistForce0(alfShape, numFilters, errorForce0CoeffTab, codedVarBins);
        coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters);
        coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins);
#if ALF_RD_COST_BUG_FIX
        nonFilterBits = getNonFilterCoeffRate(alfParam);
#endif
#endif
#if ALF_RD_COST_BUG_FIX
        coeffBits += nonFilterBits;
        coeffBitsForce0 += nonFilterBits;
#endif
        cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
        cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

#if JVET_X0071_ALF_BAND_CLASSIFIER
        bool cost0better = false;
#endif
        if (cost0 < cost)
        {
          cost = cost0;
#if JVET_X0071_ALF_BAND_CLASSIFIER
          cost0better = true;
#endif
        }

        if (cost <= costMin)
        {
#if ALF_PRECISION_VARIETY
          bestCoeffMantissa = coeffMantissa;
          bestCoeffBits = coeffBitDepth;
#endif
          costMin = cost;
          numFiltersBest = numFilters;
#if JVET_X0071_ALF_BAND_CLASSIFIER
          memcpy(bestCodedVarBins, codedVarBins, sizeof(codedVarBins));
          for (int varInd = 0; varInd < numFilters; varInd++)
          {
            if (cost0better && (!bestCodedVarBins[varInd]))
            {
              memset(bestCoeff[varInd], 0, sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
              memset(bestClipp[varInd], 0, sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              bestScaleIdx[varInd] = 0;
#endif
            }
            else
            {
#if ALF_PRECISION_VARIETY
              memcpy(bestCoeff[varInd], m_filterCoeffSet[bitIdx * MAX_NUM_ALF_CLASSES + varInd], sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
              memcpy(bestClipp[varInd], m_filterClippSet[bitIdx * MAX_NUM_ALF_CLASSES + varInd], sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              bestScaleIdx[varInd] = m_filterScaleIdx[bitIdx * MAX_NUM_ALF_CLASSES + varInd];
#endif
#else
              memcpy(bestCoeff[varInd], m_filterCoeffSet[varInd], sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
              memcpy(bestClipp[varInd], m_filterClippSet[varInd], sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              bestScaleIdx[varInd] = m_filterScaleIdx[varInd];
#endif
#endif
            }
          }
          if (cost0better)
          {
            bestDist = distForce0;
            bestBits = coeffBitsForce0;
            bestAlfLumaCoeffDeltaFlag = 1;
          }
          else
          {
            bestDist = dist;
            bestBits = coeffBits;
            bestAlfLumaCoeffDeltaFlag = 0;
          }
#endif
        }
#if ALF_PRECISION_VARIETY
      }
    }
#endif
    numFilters--;
  }

#if JVET_AK0123_ALF_COEFF_RESTRICTION
  if (tryImproveBySA)
  {
    for (int filtIdx = 0; filtIdx < numFiltersBest; filtIdx++)
    {
      memcpy(m_filterCoeffSet[filtIdx], bestCoeff[filtIdx], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
      memcpy(m_filterClippSet[filtIdx], bestClipp[filtIdx], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
      m_filterScaleIdx[filtIdx] = bestScaleIdx[filtIdx];
    }
#if JVET_X0071_ALF_BAND_CLASSIFIER
    dist = tryImproveFilterCoeffs(covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], classifierIdx, bestCoeffBits, bestCoeffMantissa, m_lambda[COMPONENT_Y]);
#else
    dist = tryImproveFilterCoeffs(covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest,
#if ALF_IMPROVEMENT
      , m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx]
#endif
    );
#endif
    // filter coeffs are stored in m_filterCoeffSet
#if ALF_IMPROVEMENT
    coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], 0, true, bestCoeffBits, bestCoeffMantissa);
#if ALF_RD_COST_BUG_FIX
#if JVET_X0071_ALF_BAND_CLASSIFIER
    nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx, classifierIdx);
#else
    nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx);
#endif
#endif
#else
    coeffBits = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest);
#if ALF_RD_COST_BUG_FIX
    nonFilterBits = getNonFilterCoeffRate(alfParam);
#endif
#endif
#if ALF_RD_COST_BUG_FIX
    coeffBits += nonFilterBits;
#endif
    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  
    if (cost < costMin)
    {
      costMin = cost;
#if JVET_X0071_ALF_BAND_CLASSIFIER
      for (int varInd = 0; varInd < numFiltersBest; varInd++)
      {
        memcpy(bestCoeff[varInd], m_filterCoeffSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        memcpy(bestClipp[varInd], m_filterClippSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        bestScaleIdx[varInd] = m_filterScaleIdx[varInd];
      }
      bestDist = dist;
      bestBits = coeffBits;
      bestAlfLumaCoeffDeltaFlag = 0;
#endif
    }
  }
#endif

#if ALF_PRECISION_VARIETY
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  alfParam.coeffMantissa[altIdx] = bestCoeffMantissa;
#else
  (void)bestCoeffMantissa; // compilation
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  alfParam.coeffBits[altIdx] = bestCoeffBits;
#else
  (void)bestCoeffBits; // compilation
#endif
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  double distReturn = bestDist;
  uiCoeffBits = bestBits;
  alfParam.alfLumaCoeffDeltaFlag = bestAlfLumaCoeffDeltaFlag;
  memcpy( alfParam.alfLumaCoeffFlag, bestCodedVarBins, sizeof( bestCodedVarBins ) );
  alfParam.numLumaFilters[altIdx] = numFiltersBest;
#else
#if ALF_IMPROVEMENT
  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfParam, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx], tryImproveScale);
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins, altIdx );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins, altIdx );
#else
  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfParam, tryImproveScale);
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );
#endif

  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;
#if ALF_IMPROVEMENT
  alfParam.numLumaFilters[altIdx] = numFiltersBest;
#else
  alfParam.numLumaFilters = numFiltersBest;
#endif
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfParam.alfLumaCoeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
  }
  else
  {
    distReturn = distForce0;
    alfParam.alfLumaCoeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfParam.alfLumaCoeffFlag, codedVarBins, sizeof( codedVarBins ) );

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
        memset( m_filterClippSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }
#endif
#if ALF_IMPROVEMENT
  for (int ind = 0; ind < alfParam.numLumaFilters[altIdx]; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff; i++)
    {
#if JVET_X0071_ALF_BAND_CLASSIFIER
      alfParam.lumaCoeff[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = bestCoeff[ind][i];
      alfParam.lumaClipp[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = bestClipp[ind][i];
#else
      alfParam.lumaCoeff[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      alfParam.lumaClipp[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
#endif
    }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    alfParam.lumaScaleIdx[altIdx][ind] = bestScaleIdx[ind];
#endif
  }
  memcpy( alfParam.filterCoeffDeltaIdx[altIdx], m_filterIndices[numFiltersBest - 1], sizeof(short) * MAX_NUM_ALF_CLASSES );
#if !ALF_RD_COST_BUG_FIX
#if JVET_X0071_ALF_BAND_CLASSIFIER
  uiCoeffBits += getNonFilterCoeffRate(alfParam, altIdx, classifierIdx);
#else
  uiCoeffBits += getNonFilterCoeffRate(alfParam, altIdx);
#endif
#endif
#else
  for( int ind = 0; ind < alfParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
      alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      alfParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
    }
  }

  memcpy( alfParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
#if !ALF_RD_COST_BUG_FIX
  uiCoeffBits += getNonFilterCoeffRate( alfParam );
#endif
#endif
  return distReturn;
}

#if JVET_X0071_ALF_BAND_CLASSIFIER
int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfParam& alfParam, int altIdx, int classifierIdx )
#else
int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfParam& alfParam 
#if ALF_IMPROVEMENT
  , int altIdx
#endif
)
#endif
{
#if ALF_IMPROVEMENT
  CHECK( alfParam.numLumaFilters[altIdx] < 1, "Wrong number of alfParam.numLumaFilters[altIdx]" );
#else
  CHECK( alfParam.numLumaFilters < 1, "Wrong number of alfParam.numLumaFilters[altIdx]" );
#endif

  int len = 0   // alf_coefficients_delta_flag
#if ALF_IMPROVEMENT
    + lengthUvlc(alfParam.numLumaFilters[altIdx] - 1);  // alf_luma_num_filters_signalled_minus1   ue(v)
  if (alfParam.numLumaFilters[altIdx] > 1)
  {
    const int coeffLength = ceilLog2(alfParam.numLumaFilters[altIdx]);
#else
          + 2                                          // slice_alf_chroma_idc                     u(2)
          + lengthUvlc (alfParam.numLumaFilters - 1);  // alf_luma_num_filters_signalled_minus1   ue(v)

  if( alfParam.numLumaFilters > 1 )
  {
    const int coeffLength = ceilLog2(alfParam.numLumaFilters);
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
    for( int i = 0; i < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; i++ )
#else
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
#endif
    {
      len += coeffLength;                              // alf_luma_coeff_delta_idx   u(v)
    }
  }
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AD0222_ALF_RESI_CLASS
  if( classifierIdx == 1 )
  {
    len++;
  }
  else
  {
    len += 2;
  }
#else
  len++;
#endif
#endif
#if ALF_RD_COST_BUG_FIX
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  len += 2; // alf_luma_bits
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  len += 1; // alf_luma_mantissa
#endif
  return len;
}


int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins 
#if ALF_IMPROVEMENT
  , int altIdx
#endif
#if ALF_PRECISION_VARIETY
  , char bitIdx, bool isLuma, char bitNum, char mantissa
#endif
)
{
  int len = 0;
  // Filter coefficients
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    HuffmanForALF huffman(isLuma, bitNum, mantissa, orderIdx);
    huffman.init();
    for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
    {
#if ALF_PRECISION_VARIETY
      int realFiltIdx = filtIdx + bitIdx * MAX_NUM_ALF_CLASSES;
#endif
      for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
      {
        if (codedVarBins[filtIdx])
        {
#if ALF_PRECISION_VARIETY
          len += lengthHuffman(pDiffQFilterCoeffIntPP[realFiltIdx][coeffIdx], huffman);
#else
          len += lengthHuffman(pDiffQFilterCoeffIntPP[filtIdx][coeffIdx], huffman);
#endif
        }
        else
        {
          len += lengthHuffman(0, huffman);
        }
      }
    }
  }
#else
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int minBits = MAX_INT;
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    for (int k = 0; k < 4; k++)
    {
      int curBits = 0;
      for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
      {
#if ALF_PRECISION_VARIETY
        int realFiltIdx = filtIdx + bitIdx * MAX_NUM_ALF_CLASSES;
#endif
        for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
        {
          if (codedVarBins[filtIdx])
          {
#if ALF_PRECISION_VARIETY
            curBits += lengthGolomb(pDiffQFilterCoeffIntPP[realFiltIdx][coeffIdx], k + (orderIdx == 0 ? alfShape.offset0 : ALF_ORDER));
#else
            curBits += lengthGolomb(pDiffQFilterCoeffIntPP[filtIdx][coeffIdx], k + (orderIdx == 0 ? alfShape.offset0 : ALF_ORDER));
#endif
          }
          else
          {
            curBits += lengthGolomb(0, k + (orderIdx == 0 ? alfShape.offset0 : ALF_ORDER));
          }
        }
      }
      if (curBits < minBits)
      {
        minBits = curBits;
      }
    }
    len += minBits + 2;
  }
#endif
#else
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        len += lengthUvlc( abs( pDiffQFilterCoeffIntPP[ ind ][ i ] ) ); // alf_coeff_luma_delta[i][j]
        if( ( abs( pDiffQFilterCoeffIntPP[ ind ][ i ] ) != 0 ) )
          len += 1;
      }
    }
    else
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        len += lengthUvlc( 0 ); // alf_coeff_luma_delta[i][j]
      }
    }
  }
#endif
#if ALF_IMPROVEMENT
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] )
#else
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
  {
#if ALF_PRECISION_VARIETY
    for (int ind = 0 + bitIdx * MAX_NUM_ALF_CLASSES; ind < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; ++ind)
#else
    for (int ind = 0; ind < numFilters; ++ind)
#endif
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(pDiffQFilterCoeffIntPP[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
        len += 2;
      }
    }
  }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  len += numFilters * AdaptiveLoopFilter::m_SCALE_BITS_NUM;
#endif

  return len;
}

#if ALF_IMPROVEMENT
#if ALF_PRECISION_VARIETY
void EncAdaptiveLoopFilter::deriveCoeffQuantMultipleBitDepths(int *filterClipp, const AlfCovariance& cov, const AlfFilterShape& shape, const bool optimizeClip, int filtIdx, double errorTabForce0Coeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][2], double lambda, bool tryImproveScale)
{
  const int numCoeff = shape.numCoeff;
  double cAc, bc, coeffDelta;
  static double cA[MAX_NUM_ALF_LUMA_COEFF];
#if JVET_AF0177_ALF_COV_FLOAT
  float     filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#else
  double    filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#endif

#if JVET_AD0222_ALF_RESI_CLASS
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip, m_enableLessClip 
#if JVET_AN0086_RESIDUAL_CHECK
  , m_encCfg->getAlfResidualCheckEnabled()
#endif
  );
#else
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
#endif

  int minBits = 8;
  int numBitCand = 1;
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  if( m_encCfg->getUseAlfPrecision() == true )
  {
    numBitCand = 4;
    minBits = 6;
  }
#endif
  int minMantissa = 1;
  int numMantissaCand = 1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  numMantissaCand = 2;
#endif

  for (int mantissa = minMantissa; mantissa < minMantissa + numMantissaCand; mantissa++)
  {
    if (!tryImproveScale)
    {
      if (mantissa != minMantissa + numMantissaCand - 1)
      {
        continue;
      }
    }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    int baseFiltIdxStore = filtIdx + ((mantissa - minMantissa) * m_numBitPrecision + numBitCand - 1) * MAX_NUM_ALF_CLASSES; // should be tried the first
#endif
    for (int coeffBitIdx = numBitCand - 1; coeffBitIdx >= 0; coeffBitIdx--)
    {
      int bitDepth = coeffBitIdx + minBits;
      int bitIdx = (mantissa - minMantissa) * m_numBitPrecision + coeffBitIdx;
      const int factor = 1 << (bitDepth - 1);
      int max_value = factor - 1;
      int min_value = -factor + 1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      ALFCoeffRestriction coeffRestriction(bitDepth, mantissa);
      coeffRestriction.init();
      max_value = coeffRestriction.getParam().maxValue;
      min_value = coeffRestriction.getParam().minValue;
#endif
      int filtIdxStore = filtIdx + bitIdx * MAX_NUM_ALF_CLASSES;

#if JVET_AK0123_ALF_COEFF_RESTRICTION
      char scaleIdxFrom = 0, scaleIdxTo = 1;
      if (tryImproveScale)
      {
        if (filtIdxStore == baseFiltIdxStore)
        {
          scaleIdxFrom = 0, scaleIdxTo = (1 << AdaptiveLoopFilter::m_SCALE_BITS_NUM);
        }
        else
        {
          scaleIdxFrom = m_filterScaleIdx[baseFiltIdxStore], scaleIdxTo = m_filterScaleIdx[baseFiltIdxStore] + 1;
        }
      }
      char bestScaleIdx = 0;
      int bestFilterCoeffQuant[MAX_NUM_ALF_LUMA_COEFF];
      double bestError = std::numeric_limits<double>::max(), errRef = bestError;
      double bestRdCost = std::numeric_limits<double>::max();
      for (char scaleIdx = scaleIdxFrom; scaleIdx < scaleIdxTo; scaleIdx++)
      {
        double scaleFactor = (double)AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdx] / (1 << AdaptiveLoopFilter::m_SCALE_SHIFT);
        double scaleFactorInv = (double)(1 << AdaptiveLoopFilter::m_SCALE_SHIFT) / AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdx];
#if JVET_AF0177_ALF_COV_FLOAT
        float     filterCoeffScaled[MAX_NUM_ALF_LUMA_COEFF];
        for (int i = 0; i < numCoeff; i++)
        {
          filterCoeffScaled[i] = float(filterCoeff[i] * scaleFactorInv);
        }
#else
        double    filterCoeffScaled[MAX_NUM_ALF_LUMA_COEFF];
        for (int i = 0; i < numCoeff; i++)
        {
          filterCoeffScaled[i] = filterCoeff[i] * scaleFactorInv;
        }
#endif
        
        roundFiltCoeff(m_filterCoeffSet[filtIdxStore], filterCoeffScaled, numCoeff, factor);
#else
        char scaleIdx = 0;
        roundFiltCoeff(m_filterCoeffSet[filtIdxStore], filterCoeff, numCoeff, factor);
#endif
        for ( int i = 0; i < numCoeff - 1; i++ )
        {
          m_filterCoeffSet[filtIdxStore][i] = std::min( max_value, std::max( min_value, m_filterCoeffSet[filtIdxStore][i] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          m_filterCoeffSet[filtIdxStore][i] = coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[m_filterCoeffSet[filtIdxStore][i] - min_value]];
#endif
        }
        m_filterCoeffSet[filtIdxStore][numCoeff - 1] = 0;

        int modified=1;
        double errRef = cov.calcErrorForCoeffs(filterClipp, m_filterCoeffSet[filtIdxStore], numCoeff, bitDepth, scaleIdx);
        cov.calcInitErrorForCoeffs(&cAc, cA, &bc, filterClipp, m_filterCoeffSet[filtIdxStore], numCoeff, bitDepth, scaleIdx);

        while( modified )
        {
          modified=0;
          for( int sign: {1, -1} )
          {
            double errMin = MAX_DOUBLE;
            int minInd = -1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            coeffDelta = (double)-sign / (double)factor * scaleFactor;
#else
            coeffDelta = (double)-sign / (double)factor;
#endif

            for( int k = 0; k < numCoeff-1; k++ )
            {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              int oldVal = m_filterCoeffSet[filtIdxStore][k];
              int newIdx = coeffRestriction.getParam().coeffToIdx[oldVal - min_value] - sign;
              if (newIdx < 0 || newIdx >= (int)coeffRestriction.getParam().idxToCoeff.size())
              {
                continue;
              }
              m_filterCoeffSet[filtIdxStore][k] = coeffRestriction.getParam().idxToCoeff[newIdx];
              coeffDelta = (double)(m_filterCoeffSet[filtIdxStore][k] - oldVal) / (double)factor * scaleFactor;
#else
              if(m_filterCoeffSet[filtIdxStore][k] - sign > max_value || m_filterCoeffSet[filtIdxStore][k] - sign < min_value )
              {
                continue;
              }
              m_filterCoeffSet[filtIdxStore][k] -= sign;
#endif
              double error = cov.calcErrorForCoeffsDelta(cAc, cA, bc, filterClipp, m_filterCoeffSet[filtIdxStore], numCoeff, bitDepth, coeffDelta, k);
              if( error < errMin )
              {
                errMin = error;
                minInd = k;
              }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              m_filterCoeffSet[filtIdxStore][k] = oldVal;
#else
              m_filterCoeffSet[filtIdxStore][k] += sign;
#endif
            }
            if( errMin < errRef )
            {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              int oldVal = m_filterCoeffSet[filtIdxStore][minInd];
              m_filterCoeffSet[filtIdxStore][minInd] = coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[m_filterCoeffSet[filtIdxStore][minInd] - min_value] - sign];
              coeffDelta = (double)(m_filterCoeffSet[filtIdxStore][minInd] - oldVal) / (double)factor * scaleFactor;
#else
              m_filterCoeffSet[filtIdxStore][minInd] -= sign;
#endif
              modified++;
              errRef = errMin;
              cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, m_filterCoeffSet[filtIdxStore], numCoeff, bitDepth, coeffDelta, minInd);
            }
          }
        }
#if ALF_RD_COST_BUG_FIX
        errRef = cov.calcErrorForCoeffs(filterClipp, m_filterCoeffSet[filtIdxStore], numCoeff, bitDepth, scaleIdx); // recalculate for computational stability
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        int bits = lengthFilterCoeffsOneFilter(shape, m_filterCoeffSet[filtIdxStore], true, bitDepth, mantissa);
        double rdCost = errRef + lambda * bits;
        if (rdCost < bestRdCost)
        {
          bestRdCost = rdCost;
          bestError = errRef;
          bestScaleIdx = scaleIdx;
          std::copy_n(m_filterCoeffSet[filtIdxStore], MAX_NUM_ALF_LUMA_COEFF, bestFilterCoeffQuant);
        }
      }
      errRef = bestError;
      m_filterScaleIdx[filtIdxStore] = bestScaleIdx;
      std::copy_n(bestFilterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, m_filterCoeffSet[filtIdxStore]);
#endif

      errorTabForce0Coeff[bitIdx][filtIdx][1] = errRef + cov.pixAcc;
    }
  }
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode(AlfFilterShape& alfShape, int** filterSet, int** filterCoeffDiff, const int numFilters, bool nonLinearFlag, char bitIdx, bool isLuma, char bitNum, char mantissa)
{
  return (nonLinearFlag ? getCostFilterClipp(alfShape, filterSet, numFilters, bitIdx) : 0) + getCostFilterCoeff(alfShape, filterSet, numFilters, bitIdx, isLuma, bitNum, mantissa);
}
#else
int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, bool nonLinearFlag )
{
  return (nonLinearFlag ? getCostFilterClipp(alfShape, filterSet, numFilters) : 0) + getCostFilterCoeff(alfShape, filterSet, numFilters);
}
#endif
#else
int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters )
{
  return (m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? getCostFilterClipp(alfShape, filterSet, numFilters) : 0) + getCostFilterCoeff(alfShape, filterSet, numFilters);
}
#endif

#if ALF_PRECISION_VARIETY
int EncAdaptiveLoopFilter::getCostFilterCoeff(AlfFilterShape& alfShape, int** pDiffQFilterCoeffIntPP, const int numFilters, char bitIdx, bool isLuma, char bitNum, char mantissa)
{
  return lengthFilterCoeffs(alfShape, numFilters, pDiffQFilterCoeffIntPP, bitIdx, isLuma, bitNum, mantissa);  // alf_coeff_luma_delta[i][j];
}
#else
int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  return lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP );  // alf_coeff_luma_delta[i][j];
}
#endif

int EncAdaptiveLoopFilter::getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters
#if ALF_PRECISION_VARIETY
, char bitIdx
#endif
)
{
#if ALF_PRECISION_VARIETY
  for (int filterIdx = 0 + bitIdx * MAX_NUM_ALF_CLASSES; filterIdx < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; ++filterIdx)
#else
  for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
#endif
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if (!abs(pDiffQFilterCoeffIntPP[filterIdx][i]))
      {
        m_filterClippSet[filterIdx][i] = 0;
      }
    }
  }
  return (numFilters * (alfShape.numCoeff - 1)) << 1;
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs(AlfFilterShape& alfShape, const int numFilters, int** FilterCoeff
#if ALF_PRECISION_VARIETY
  , char bitIdx, bool isLuma, char bitNum, char mantissa
#endif
)
{
  int bitCnt = 0;

#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    HuffmanForALF huffman(isLuma, bitNum, mantissa, orderIdx);
    huffman.init();
    for (int filtIdx = 0 + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx++)
    {
      for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
      {
        bitCnt += lengthHuffman(FilterCoeff[filtIdx][coeffIdx], huffman);
      }
    }
  }
#else
  for( int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++ )
  {
    int minBits = MAX_INT;
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    for( int k = 0; k < 4; k++ )
    {
      int curBits = 0;
#if ALF_PRECISION_VARIETY
      for( int filtIdx = 0 + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx++ )
#else
      for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
#endif
      {
        for( int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++ )
        {
          curBits += lengthGolomb( FilterCoeff[filtIdx][coeffIdx], k + (orderIdx == 0 ? alfShape.offset0 : ALF_ORDER ) );
        }
      }
      if (curBits < minBits)
      {
        minBits = curBits;
      }
    }
    bitCnt += minBits + 2;
  }
#endif
#else
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitCnt += lengthUvlc( abs( FilterCoeff[ ind ][ i ] ) );
      if( abs( FilterCoeff[ ind ][ i ] ) != 0 )
        bitCnt += 1;
    }
  }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  bitCnt += numFilters * AdaptiveLoopFilter::m_SCALE_BITS_NUM;
#endif
  return bitCnt;
}

#if JVET_AK0123_ALF_COEFF_RESTRICTION
int EncAdaptiveLoopFilter::lengthFilterCoeffsOneFilter(const AlfFilterShape& alfShape, int* FilterCoeff, bool isLuma, char bitNum, char mantissa
)
{
  int bitCnt = 0;

#if ALF_IMPROVEMENT
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    HuffmanForALF huffman(isLuma, bitNum, mantissa, orderIdx);
    huffman.init();
    for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
    {
      bitCnt += lengthHuffman(FilterCoeff[coeffIdx], huffman);
    }
  }
#else
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
    bitCnt += lengthUvlc(abs(FilterCoeff[ind][i]));
    if (abs(FilterCoeff[ind][i]) != 0)
      bitCnt += 1;
  }
#endif
  bitCnt += AdaptiveLoopFilter::m_SCALE_BITS_NUM;
  return bitCnt;
}
#endif

double EncAdaptiveLoopFilter::getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins 
#if ALF_IMPROVEMENT
  , int altIdx
#endif
#if ALF_PRECISION_VARIETY
  , char bitIdx, bool isLuma, char bitNum, char mantissa
#endif
)
{
  int bitsVarBin[MAX_NUM_ALF_CLASSES];

#if ALF_IMPROVEMENT
#if !JVET_AK0123_ALF_COEFF_RESTRICTION
  int bestK[2] = { 0, 0 };
  for( int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int minBits = MAX_INT;
    int startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int endIdx = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    for( int k = 0; k < 4; k++ )
    {
      int curBits = 0;
#if ALF_PRECISION_VARIETY
      for( int filtIdx = 0 + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; filtIdx++ )
#else
      for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
#endif
      {
        for( int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++ )
        {
          curBits += lengthGolomb( m_filterCoeffSet[filtIdx][coeffIdx], k + ( orderIdx == 0 ? alfShape.offset0 : ALF_ORDER ) );
        }
      }
      if( curBits < minBits )
      {
        bestK[orderIdx] = k;
        minBits = curBits;
      }
    }
  }
  bestK[0] += alfShape.offset0;
  bestK[1] += ALF_ORDER;
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  HuffmanForALF huffman(isLuma, bitNum, mantissa, 0);
  huffman.init();
#endif
  for( int ind = 0; ind < numFilters; ++ind )
  {
#if ALF_PRECISION_VARIETY
    int filtIdx = ind + bitIdx * MAX_NUM_ALF_CLASSES;
#endif
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      if (i == 0)
      {
        huffman.setGroup(0);
      }
      else if (i == alfShape.indexSecOrder)
      {
        huffman.setGroup(1);
      }
      bitsVarBin[ind] += lengthHuffman(m_filterCoeffSet[filtIdx][i], huffman);
#else
      if (i < alfShape.indexSecOrder)
      {
#if ALF_PRECISION_VARIETY
        bitsVarBin[ind] += lengthGolomb( m_filterCoeffSet[filtIdx][i], bestK[0] );
#else
        bitsVarBin[ind] += lengthGolomb( m_filterCoeffSet[ind][i], bestK[0] );
#endif
      }
      else
      {
#if ALF_PRECISION_VARIETY
        bitsVarBin[ind] += lengthGolomb(m_filterCoeffSet[filtIdx][i], bestK[1]);
#else
        bitsVarBin[ind] += lengthGolomb(m_filterCoeffSet[ind][i], bestK[1]);
#endif
      }
#endif
#else
      bitsVarBin[ ind ] += lengthUvlc( abs( m_filterCoeffSet[ ind ][ i ] ) );
      if( abs( m_filterCoeffSet[ ind ][ i ] ) != 0 )
        bitsVarBin[ ind ] += 1;
#endif
    }
  }

  int zeroBitsVarBin = 0;
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    if (i == 0)
    {
      huffman.setGroup(0);
    }
    else if (i == alfShape.indexSecOrder)
    {
      huffman.setGroup(1);
    }
    zeroBitsVarBin += lengthHuffman(0, huffman);
#else
    if (i < alfShape.indexSecOrder)
    {
      zeroBitsVarBin += lengthGolomb(0, bestK[0]);
    }
    else
    {
      zeroBitsVarBin += lengthGolomb(0, bestK[1]);
    }
#endif
#else
    zeroBitsVarBin += lengthUvlc( 0 );
#endif
  }
#if ALF_IMPROVEMENT
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] )
#else
  if( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
  {
#if ALF_PRECISION_VARIETY
    for (int ind = 0 + bitIdx * MAX_NUM_ALF_CLASSES; ind < numFilters + bitIdx * MAX_NUM_ALF_CLASSES; ++ind)
#else
    for (int ind = 0; ind < numFilters; ++ind)
#endif
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(m_filterCoeffSet[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
      }
    }
  }

  double distForce0 = getDistCoeffForce0( codedVarBins, errorTabForce0Coeff, bitsVarBin, zeroBitsVarBin, numFilters);

  return distForce0;
}
double EncAdaptiveLoopFilter::getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, int zeroBitsVarBin, const int numFilters)
{
  double distForce0 = 0;
  std::memset( codedVarBins, 0, sizeof( *codedVarBins ) * MAX_NUM_ALF_CLASSES );

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    double costDiff = (errorForce0CoeffTab[filtIdx][0] + m_lambda[COMPONENT_Y] * zeroBitsVarBin) - (errorForce0CoeffTab[filtIdx][1] + m_lambda[COMPONENT_Y] * bitsVarBin[filtIdx]);
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc( int uiCode )
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return ( uiLength >> 1 ) + ( ( uiLength + 1 ) >> 1 );
}

#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
int EncAdaptiveLoopFilter::lengthHuffman(int coeffVal, HuffmanForALF& huffman)
{
  uint32_t symbol; int length;
  huffman.encodeCoeff(coeffVal, symbol, length);
  return length;
}
#endif
int EncAdaptiveLoopFilter::lengthGolomb(int coeffVal, int k, bool signed_coeff)
{
  int numBins = 0;
  unsigned int symbol = abs(coeffVal);
  while (symbol >= (unsigned int)(1 << k))
  {
    numBins++;
    symbol -= 1 << k;
    k++;
  }
  numBins += (k + 1);
  if (signed_coeff && coeffVal != 0)
  {
    numBins++;
  }
  return numBins;
}
#endif

#if JVET_X0071_ALF_BAND_CLASSIFIER
#if ALF_PRECISION_VARIETY
double EncAdaptiveLoopFilter::deriveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam, bool nonLinear, int classifierIdx, bool isMaxNum, int mergedPair[MAX_NUM_ALF_CLASSES][2], int mergedCoeff[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], char mergedScaleIdx[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES], double mergedErr[m_alfPrecisionVariety][MAX_NUM_ALF_CLASSES], bool tryImproveScale)
#else
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam, bool nonLinear, int classifierIdx, bool isMaxNum, int mergedPair[MAX_NUM_ALF_CLASSES][2], int mergedCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], char mergedScaleIdx[MAX_NUM_ALF_CLASSES], double mergedErr[MAX_NUM_ALF_CLASSES], bool tryImproveScale)
#endif
#else
double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam 
#if ALF_IMPROVEMENT
  , bool nonLinear
#endif
  , bool tryImproveScale
)
#endif
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
#if JVET_X0071_ALF_BAND_CLASSIFIER
  int changedClass = -1;
  if( !isMaxNum )
  {
#if ALF_PRECISION_VARIETY
    for(int i = 0;i < m_alfPrecisionVariety; i++)
    {
      memcpy(clipMerged[i][numFilters - 1], clipMerged[i][numFilters], sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    }
#else
    memcpy( clipMerged[numFilters - 1], clipMerged[numFilters], sizeof( int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF] ) );
#endif
  }
#endif
  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    bool found_clip = false;
#if JVET_X0071_ALF_BAND_CLASSIFIER
    bool changedFilter = isMaxNum;
    for( int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++ )
#else
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
#endif
    {
      if( filterIndices[classIdx] == filtIdx )
      {
        tmpCov += cov[classIdx];
        if( !found_clip )
        {
          found_clip = true; // clip should be at the adress of shortest one
#if JVET_X0071_ALF_BAND_CLASSIFIER
          if( changedFilter == false && mergedPair[numFilters][0] == classIdx )
          {
            changedFilter = true;
            if( nonLinear )
            {
#if ALF_PRECISION_VARIETY
              for( int i = 0; i < m_alfPrecisionVariety; i++ )
              {
                std::fill_n(clipMerged[i][numFilters - 1][classIdx], MAX_NUM_ALF_LUMA_COEFF, 2);
              }
#else
              std::fill_n(clipMerged[numFilters - 1][classIdx], MAX_NUM_ALF_LUMA_COEFF, 2);
#endif
            }
          }
          changedClass = classIdx;
#if ALF_PRECISION_VARIETY
          for(int i = 0;i < m_alfPrecisionVariety; i++)
          {        
            memcpy( m_filterCoeffSet[filtIdx + i * MAX_NUM_ALF_CLASSES], mergedCoeff[i][classIdx], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            m_filterScaleIdx[filtIdx + i * MAX_NUM_ALF_CLASSES] = mergedScaleIdx[i][classIdx];
#endif
            errorTabForce0Coeff[i][filtIdx][1] = mergedErr[i][classIdx];
          }
#else
          memcpy( m_filterCoeffSet[filtIdx], mergedCoeff[classIdx], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          m_filterScaleIdx[filtIdx] = mergedScaleIdx[classIdx];
#endif
          errorTabForce0Coeff[filtIdx][1] = mergedErr[classIdx];
#endif
#endif
#if ALF_PRECISION_VARIETY
          for( int i = 0; i < m_alfPrecisionVariety; i++ )
          {
            memcpy(m_filterClippSet[filtIdx + i * MAX_NUM_ALF_CLASSES], clipMerged[i][numFilters - 1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
          }
#else
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters - 1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
#endif
        }
      }
    }

    // Find coeffcients
    CHECK(alfShape.numCoeff != tmpCov.numCoeff, "The number of coefficients in AlfFilterShape does not match AlfCovariance");
#if !ALF_IMPROVEMENT || !JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    char& scaleIdxRef = m_filterScaleIdx[filtIdx];
#else
    char* scaleIdxRef;
#endif
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
    if( changedFilter )
    {
#if ALF_PRECISION_VARIETY
      deriveCoeffQuantMultipleBitDepths( m_filterClippSet[filtIdx], tmpCov, alfShape, nonLinear, filtIdx, errorTabForce0Coeff, m_lambda[CHANNEL_TYPE_LUMA], tryImproveScale);
      for( int i = 1; i < m_alfPrecisionVariety; i++ )
      {
        memcpy( m_filterClippSet[filtIdx + i * MAX_NUM_ALF_CLASSES], m_filterClippSet[filtIdx] , sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
      }
#else
      errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, nonLinear, m_lambda[CHANNEL_TYPE_LUMA], nullptr, tryImproveScale);
#endif
      if( nonLinear )
      {
#if ALF_PRECISION_VARIETY
        for( int i = 0; i < m_alfPrecisionVariety; i++ )
        {
          memcpy( clipMerged[i][numFilters - 1][changedClass], m_filterClippSet[filtIdx + i * MAX_NUM_ALF_CLASSES], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
        }
#else
        memcpy( clipMerged[numFilters - 1][changedClass], m_filterClippSet[filtIdx], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
#endif
      }
#if ALF_PRECISION_VARIETY
      for( int i = 0; i < m_alfPrecisionVariety; i++ )
      {
        memcpy( mergedCoeff[i][changedClass], m_filterCoeffSet[filtIdx + i * MAX_NUM_ALF_CLASSES], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        mergedScaleIdx[i][changedClass] = m_filterScaleIdx[filtIdx + i * MAX_NUM_ALF_CLASSES];
#endif
        mergedErr[i][changedClass] = errorTabForce0Coeff[i][filtIdx][1];
      }
#else
      memcpy( mergedCoeff[changedClass], m_filterCoeffSet[filtIdx], sizeof( int[MAX_NUM_ALF_LUMA_COEFF] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      mergedScaleIdx[changedClass] = m_filterScaleIdx[filtIdx];
#endif
      mergedErr[changedClass] = errorTabForce0Coeff[filtIdx][1];
#endif
    }
#else
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, nonLinear, m_lambda[CHANNEL_TYPE_LUMA], scaleIdxRef, tryImproveScale);
#endif
#else
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, false, m_lambda[CHANNEL_TYPE_LUMA], scaleIdxRef, tryImproveScale);
#endif
#if ALF_PRECISION_VARIETY
    for( int i = 0; i < m_alfPrecisionVariety; i++ )
    {
      errorTabForce0Coeff[i][filtIdx][0] = tmpCov.pixAcc;
    }
    error += errorTabForce0Coeff[0][filtIdx][1];
#else
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
#endif
  }
  return error;
}

#if JVET_AK0123_ALF_COEFF_RESTRICTION
#if JVET_X0071_ALF_BAND_CLASSIFIER
double EncAdaptiveLoopFilter::tryImproveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, bool nonLinear, int classifierIdx, int bitsNum, int mantissa, double lambda)
#else
double EncAdaptiveLoopFilter::tryImproveFilterCoeffs(AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters,
#if ALF_IMPROVEMENT
  , bool nonLinear
#endif
  , double lambda
)
#endif
{
  double error = 0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    tmpCov.reset();
#if JVET_X0071_ALF_BAND_CLASSIFIER
    for (int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++)
#else
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
#endif
    {
      if (filterIndices[classIdx] == filtIdx)
      {
        tmpCov += cov[classIdx];
      }
    }

    // Find coeffcients
    CHECK(alfShape.numCoeff != tmpCov.numCoeff, "The number of coefficients in AlfFilterShape does not match AlfCovariance");
    double curError = 0;
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
    curError = tmpCov.pixAcc + tryImproveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, bitsNum, nonLinear, true, mantissa, lambda, m_filterScaleIdx[filtIdx]);
#else
    curError = tmpCov.pixAcc + tryImproveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, nonLinear, true, 1, lambda, m_filterScaleIdx[filtIdx]);
#endif
#else
    curError = tmpCov.pixAcc + tryImproveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, false, true, 1, lambda, m_filterScaleIdx[filtIdx]);
#endif
    error += curError;
  }
  return error;
}
#endif

double EncAdaptiveLoopFilter::deriveCoeffQuant(int* filterClipp, int* filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip, const bool isLuma, const int mantissa, double lambda, char& scaleIdxRef, bool tryImproveScale)
{
  const int factor = 1 << (bitDepth - 1);
  int max_value = factor - 1;
  int min_value = -factor + 1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  ALFCoeffRestriction coeffRestriction(bitDepth, mantissa);
  coeffRestriction.init();
  max_value = coeffRestriction.getParam().maxValue;
  min_value = coeffRestriction.getParam().minValue;
#endif

  const int numCoeff = shape.numCoeff;
#if JVET_AF0177_ALF_COV_FLOAT
  float     filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#else
  double    filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if ALF_PRECISION_VARIETY
  double cAc, bc, coeffDelta;
  static double cA[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if JVET_AD0222_ALF_RESI_CLASS
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip, m_enableLessClip 
#if JVET_AN0086_RESIDUAL_CHECK
  , m_encCfg->getAlfResidualCheckEnabled()
#endif
  );
#else
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
#endif

#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char scaleIdxFrom = 0, scaleIdxTo = 1;
  if (tryImproveScale)
  {
    scaleIdxFrom = 0, scaleIdxTo = (1 << AdaptiveLoopFilter::m_SCALE_BITS_NUM);
  }
  char bestScaleIdx = 0;
  int bestFilterCoeffQuant[MAX_NUM_ALF_LUMA_COEFF];
  double bestError = std::numeric_limits<double>::max(), errRef = bestError;
  double bestRdCost = std::numeric_limits<double>::max();
  for (scaleIdxRef = scaleIdxFrom; scaleIdxRef < scaleIdxTo; scaleIdxRef++)
  {
    double scaleFactor = (double)AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxRef] / (1 << AdaptiveLoopFilter::m_SCALE_SHIFT);
    double scaleFactorInv = (double)(1 << AdaptiveLoopFilter::m_SCALE_SHIFT) / AdaptiveLoopFilter::m_SCALE_FACTOR[(int)scaleIdxRef];
#if JVET_AF0177_ALF_COV_FLOAT
    float     filterCoeffScaled[MAX_NUM_ALF_LUMA_COEFF];
    for (int i = 0; i < numCoeff; i++)
    {
      filterCoeffScaled[i] = float(filterCoeff[i] * scaleFactorInv);
    }
#else
    double    filterCoeffScaled[MAX_NUM_ALF_LUMA_COEFF];
    for (int i = 0; i < numCoeff; i++)
    {
      filterCoeffScaled[i] = filterCoeff[i] * scaleFactorInv;
    }
#endif
    
    roundFiltCoeff(filterCoeffQuant, filterCoeffScaled, numCoeff, factor);
#else
    roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );
#endif

    for ( int i = 0; i < numCoeff - 1; i++ )
    {
      filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      filterCoeffQuant[i] = coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[filterCoeffQuant[i] - min_value]];
#endif
    }
    filterCoeffQuant[numCoeff - 1] = 0;

    int modified=1;

    double errRef=cov.calcErrorForCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth, scaleIdxRef);
#if ALF_PRECISION_VARIETY
    cov.calcInitErrorForCoeffs(&cAc, cA, &bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth, scaleIdxRef);
#endif
    while( modified )
    {
      modified=0;
      for( int sign: {1, -1} )
      {
        double errMin = MAX_DOUBLE;
        int minInd = -1;
#if ALF_PRECISION_VARIETY
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        coeffDelta = (double)-sign / (double)factor * scaleFactor;
#else
        coeffDelta = (double)-sign / (double)factor;
#endif
#endif
        for( int k = 0; k < numCoeff-1; k++ )
        {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          int oldVal = filterCoeffQuant[k];
          int newIdx = coeffRestriction.getParam().coeffToIdx[oldVal - min_value] - sign;
          if (newIdx < 0 || newIdx >= (int)coeffRestriction.getParam().idxToCoeff.size())
          {
            continue;
          }
          filterCoeffQuant[k] = coeffRestriction.getParam().idxToCoeff[newIdx];
          coeffDelta = (double)(filterCoeffQuant[k] - oldVal) / (double)factor * scaleFactor;
#else
          if( filterCoeffQuant[k] - sign > max_value || filterCoeffQuant[k] - sign < min_value )
          {
            continue;
          }
          filterCoeffQuant[k] -= sign;
#endif
#if ALF_PRECISION_VARIETY
          double error = cov.calcErrorForCoeffsDelta(cAc, cA, bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth, coeffDelta, k);
#else
          double error = cov.calcErrorForCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth, scaleIdxRef );
#endif
          if( error < errMin )
          {
            errMin = error;
            minInd = k;
          }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          filterCoeffQuant[k] = oldVal;
#else
          filterCoeffQuant[k] += sign;
#endif
        }
        if( errMin < errRef )
        {
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          int oldVal = filterCoeffQuant[minInd];
          filterCoeffQuant[minInd] = coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[filterCoeffQuant[minInd] - min_value] - sign];
          coeffDelta = (double)(filterCoeffQuant[minInd] - oldVal) / (double)factor * scaleFactor;
#else
          filterCoeffQuant[minInd] -= sign;
#endif
          modified++;
          errRef = errMin;
#if ALF_PRECISION_VARIETY
          cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth, coeffDelta, minInd);
#endif
        }
      }
    }
#if ALF_RD_COST_BUG_FIX
#if ALF_PRECISION_VARIETY
    errRef = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, bitDepth, scaleIdxRef); // recalculate for computational stability
#endif
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    int bits = lengthFilterCoeffsOneFilter(shape, filterCoeffQuant, isLuma, bitDepth, mantissa);
    double rdCost = errRef + lambda * bits;
    if (rdCost < bestRdCost)
    {
      bestRdCost = rdCost;
      bestError = errRef;
      bestScaleIdx = scaleIdxRef;
      std::copy_n(filterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, bestFilterCoeffQuant);
    }
  }
  errRef = bestError;
  scaleIdxRef = bestScaleIdx;
  std::copy_n(bestFilterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, filterCoeffQuant);
#endif
  return errRef;
}

#if JVET_AK0065_TALF
double EncAdaptiveLoopFilter::deriveCoeffForTAlfQuant(int* filterClipp, int* filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip)
{
  const int factor = 1 << (bitDepth - 1);
  int max_value = factor - 1;
  int min_value = -factor + 1;

  const int numCoeff = shape.numCoeff;
#if JVET_AF0177_ALF_COV_FLOAT
  float     filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#else
  double    filterCoeff[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if ALF_PRECISION_VARIETY
  double cAc, bc, coeffDelta;
  static double cA[MAX_NUM_ALF_LUMA_COEFF];
#endif
#if JVET_AD0222_ALF_RESI_CLASS
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip, m_enableLessClip 
#if JVET_AN0086_RESIDUAL_CHECK
  , m_encCfg->getAlfResidualCheckEnabled()
#endif
  );
#else
  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
#endif

  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );

  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
  }
  filterCoeffQuant[numCoeff - 1] = 0;

  int modified=1;

  double errRef=cov.calcErrorForTAlfCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth);
#if ALF_PRECISION_VARIETY
  cov.calcInitErrorForTAlfCoeffs(&cAc, cA, &bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth);
#endif
  while( modified )
  {
    modified=0;
    for( int sign: {1, -1} )
    {
      double errMin = MAX_DOUBLE;
      int minInd = -1;
#if ALF_PRECISION_VARIETY
      coeffDelta = (double)-sign / (double)factor;
#endif
      for( int k = 0; k < numCoeff-1; k++ )
      {
        if( filterCoeffQuant[k] - sign > max_value || filterCoeffQuant[k] - sign < min_value )
        {
          continue;
        }
        filterCoeffQuant[k] -= sign;
#if ALF_PRECISION_VARIETY
        double error = cov.calcErrorForCoeffsDelta(cAc, cA, bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth, coeffDelta, k);
#else
        double error = cov.calcErrorForTAlfCoeffs( filterClipp, filterCoeffQuant, numCoeff, bitDepth );
#endif
        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
        filterCoeffQuant[k] += sign;
      }
      if( errMin < errRef )
      {
        filterCoeffQuant[minInd] -= sign;
        modified++;
        errRef = errMin;
#if ALF_PRECISION_VARIETY
        cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, filterCoeffQuant, numCoeff, bitDepth, coeffDelta, minInd);
#endif
      }
    }
  }
#if ALF_RD_COST_BUG_FIX
#if ALF_PRECISION_VARIETY
  errRef = cov.calcErrorForTAlfCoeffs(filterClipp, filterCoeffQuant, numCoeff, bitDepth); // recalculate for computational stability
#endif
#endif
  return errRef;
}
#endif

#if JVET_AF0177_ALF_COV_FLOAT
void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, float *filterCoeff, const int numCoeff, const int factor )
#else
void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor )
#endif
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

#if JVET_AK0123_ALF_COEFF_RESTRICTION
double EncAdaptiveLoopFilter::tryImproveCoeffQuant(int* filterClipp, int* filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip, const bool isLuma, const int mantissa, double lambda, char& scaleIdxRef)
{
  const int numCoeff = shape.numCoeff;
  double errRef = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, bitDepth, scaleIdxRef);
  int bitsRef = lengthFilterCoeffsOneFilter(shape, filterCoeffQuant, isLuma, bitDepth, mantissa);
  double costRef = errRef + lambda * bitsRef;

  const int factor = 1 << (bitDepth - 1);
  ALFCoeffRestriction coeffRestriction(bitDepth, mantissa);
  coeffRestriction.init();
  const int minValue = coeffRestriction.getParam().minValue;

  double cAc, bc, coeffDelta = 0;
  static double cA[MAX_NUM_ALF_LUMA_COEFF];

  int initFilterCoeff[MAX_NUM_ALF_LUMA_COEFF];
  std::copy_n(filterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, initFilterCoeff);

  struct StartPoint {
    std::vector<int> coeff;
    char scaleIdx;
    double error;
    int bits;
    double rdCost;
  };
  std::vector<StartPoint> startQueue;
  std::vector<StartPoint> startPool;

  char scaleIdxFrom = 0, scaleIdxTo = (1 << AdaptiveLoopFilter::m_SCALE_BITS_NUM);
  for (char curScaleIdx = scaleIdxFrom; curScaleIdx < scaleIdxTo; curScaleIdx++)
  {
    startQueue.push_back(StartPoint());
    StartPoint& sp = startQueue.back();

    sp.coeff.assign(initFilterCoeff, initFilterCoeff + numCoeff);
    sp.scaleIdx = curScaleIdx;

    double scaleFactor = (double)AdaptiveLoopFilter::m_SCALE_FACTOR[(int)curScaleIdx] / (1 << AdaptiveLoopFilter::m_SCALE_SHIFT);
    sp.error = cov.calcErrorForCoeffs(filterClipp, sp.coeff.data(), numCoeff, bitDepth, curScaleIdx);
    cov.calcInitErrorForCoeffs(&cAc, cA, &bc, filterClipp, sp.coeff.data(), numCoeff, bitDepth, curScaleIdx);
    bool modified = true;
    while (modified)
    {
      modified = 0;
      for (int sign : {1, -1})
      {
        double errMin = MAX_DOUBLE;
        int minInd = -1;
        coeffDelta = (double)-sign / (double)factor * scaleFactor;
        for (int k = 0; k < numCoeff - 1; k++)
        {
          int oldVal = sp.coeff[k];
          int newIdx = coeffRestriction.getParam().coeffToIdx[oldVal - minValue] - sign;
          if (newIdx < 0 || newIdx >= (int)coeffRestriction.getParam().idxToCoeff.size())
          {
            continue;
          }
          sp.coeff[k] = coeffRestriction.getParam().idxToCoeff[newIdx];
          coeffDelta = (double)(sp.coeff[k] - oldVal) / (double)factor * scaleFactor;
          double error = cov.calcErrorForCoeffsDelta(cAc, cA, bc, filterClipp, sp.coeff.data(), numCoeff, bitDepth, coeffDelta, k);
          if (error < errMin)
          {
            errMin = error;
            minInd = k;
          }
          sp.coeff[k] = oldVal;
        }
        if (errMin < sp.error)
        {
          int oldVal = sp.coeff[minInd];
          sp.coeff[minInd] = coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[sp.coeff[minInd] - minValue] - sign];
          coeffDelta = (double)(sp.coeff[minInd] - oldVal) / (double)factor * scaleFactor;
          modified = true;
          sp.error = errMin;
          cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, sp.coeff.data(), numCoeff, bitDepth, coeffDelta, minInd);
        }
      }
    }
    sp.error = cov.calcErrorForCoeffs(filterClipp, sp.coeff.data(), numCoeff, bitDepth, sp.scaleIdx);
    sp.bits = lambda > 0 ? lengthFilterCoeffsOneFilter(shape, sp.coeff.data(), isLuma, bitDepth, mantissa) : 0;
    sp.rdCost = sp.error + lambda * sp.bits;
  }

  // settings
  int startPoolSize = ALF_SA_SOLUTION_POOL_SIZE_MAX;
  int tmpFilterCoef[MAX_NUM_ALF_LUMA_COEFF], curFilterCoef[MAX_NUM_ALF_LUMA_COEFF];
  int posIdx[MAX_NUM_ALF_LUMA_COEFF];
  for (int i = 0; i < numCoeff - 1; i++)
  {
    posIdx[i] = i;
  }
  RandomGen gen;

  for (int saRun = 0; saRun < ALF_SA_RUNS_COUNT; saRun++)
  {
    if (startQueue.empty())
    {
      std::sort(startPool.begin(), startPool.end(), [](const StartPoint& p1, const StartPoint& p2)
        {
          return p1.rdCost < p2.rdCost;
        }
      );
      startQueue.assign(startPool.rbegin(), startPool.rend());
      startPoolSize--;
      startPoolSize = std::max(startPoolSize, ALF_SA_SOLUTION_POOL_SIZE_MIN);
      if (startPoolSize < (int)startPool.size())
      {
        startPool.resize(startPoolSize);
      }
    }
    std::copy_n(startQueue.back().coeff.data(), numCoeff, curFilterCoef);
    std::copy_n(startQueue.back().coeff.data(), numCoeff, tmpFilterCoef);
    char curScaleIdx = startQueue.back().scaleIdx;
    double scaleFactor = (double)AdaptiveLoopFilter::m_SCALE_FACTOR[(int)curScaleIdx] / (1 << AdaptiveLoopFilter::m_SCALE_SHIFT);
    double curError = startQueue.back().error;
    int curBits = startQueue.back().bits;
    startQueue.pop_back();
    cov.calcInitErrorForCoeffs(&cAc, cA, &bc, filterClipp, curFilterCoef, numCoeff, bitDepth, curScaleIdx);
    int iteration = 0, nonImprovingIterationNum = 0;
    while (nonImprovingIterationNum < numCoeff * ALF_SA_NON_IMPROVES_PER_PARAMETER_TO_STOP)
    {
      // move
      double error = curError;
      int posChanged = 0;
      std::copy_n(curFilterCoef, numCoeff, tmpFilterCoef);
      int curChangeCnt = ALF_SA_CHANGES_PER_ITERATION - (ALF_SA_CHANGES_PER_ITERATION > 1 ? (gen.nextRand() % 2) : 0);
      for (int j = 0; j < curChangeCnt; j++)
      {
        int idx = gen.nextRand() % (numCoeff - 1 - j);
        std::swap(posIdx[j], posIdx[j + idx]);
        int i = posIdx[j];
        int move = (gen.nextRand() % 2);
        move = move * 2 - 1;
        int newIdx = coeffRestriction.getParam().coeffToIdx[curFilterCoef[i] - minValue] - move;
        newIdx = std::max(newIdx, 0);
        newIdx = std::min(newIdx, (int)coeffRestriction.getParam().idxToCoeff.size() - 1);
        tmpFilterCoef[i] = coeffRestriction.getParam().idxToCoeff[newIdx];
        if (tmpFilterCoef[i] != curFilterCoef[i])
        {
          posChanged++;
          coeffDelta = (double)(tmpFilterCoef[i] - curFilterCoef[i]) / (double)factor * scaleFactor;
          error = cov.calcErrorForCoeffsDelta(cAc, cA, bc, filterClipp, tmpFilterCoef, numCoeff, bitDepth, coeffDelta, i);
          cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, tmpFilterCoef, numCoeff, bitDepth, coeffDelta, i);
        }
      }
      if (posChanged == 0)
      {
        continue;
      }

      // decision
      double prob = 0;
      int bits = 0;
      if (lambda > 0)
      {
        bits = lengthFilterCoeffsOneFilter(shape, tmpFilterCoef, isLuma, bitDepth, mantissa);
        double curLambda = 2.0 / (1.0 + pow(iteration + 1, -0.5)) - 1;
        curLambda *= lambda;
        prob = (curError - error + curLambda * (curBits - bits));
      }
      else
      {
        prob = (curError - error);
      }
      prob = prob * pow(iteration + 1, 2);
      prob = exp(-prob);
      prob = 1.0 / (1.0 + prob);

      if (gen.nextBool(prob))
      {
        if (prob > 0.9)
        {
          nonImprovingIterationNum = 0;
        }
        curError = error;
        curBits = bits;
        std::copy_n(tmpFilterCoef, numCoeff, curFilterCoef);
      }
      else
      {
        for (int i = 0; i < numCoeff - 1; i++)
        {
          if (tmpFilterCoef[i] != curFilterCoef[i])
          {
            coeffDelta = (double)(-tmpFilterCoef[i] + curFilterCoef[i]) / (double)factor * scaleFactor;
            tmpFilterCoef[i] = curFilterCoef[i];
            cov.updateErrorForCoeffsDelta(&cAc, cA, &bc, filterClipp, tmpFilterCoef, numCoeff, bitDepth, coeffDelta, i);
          }
        }
      }
      iteration++;
      nonImprovingIterationNum++;
    }

    curError = cov.calcErrorForCoeffs(filterClipp, tmpFilterCoef, numCoeff, bitDepth, curScaleIdx); // recalculate for computational stability

    double curCost = curError + lambda * curBits;
    bool isNew = true;
    for (int i = 0; i < (int)startPool.size(); i++)
    {
      if (curScaleIdx == startPool[i].scaleIdx && std::equal(curFilterCoef, curFilterCoef + numCoeff, startPool[i].coeff.data()))
      {
        isNew = false;
        break;
      }
    }
    if (isNew)
    {
      int maxIdx = 0;
      if ((int)startPool.size() < startPoolSize)
      {
        startPool.emplace_back(StartPoint());
        startPool.back().rdCost = MAX_DOUBLE;
        maxIdx = (int)startPool.size() - 1;
      }
      else
      {
        for (int i = 1; i < startPoolSize; i++)
        {
          if (startPool[i].rdCost > startPool[maxIdx].rdCost)
          {
            maxIdx = i;
          }
        }
      }
      if (curCost < startPool[maxIdx].rdCost)
      {
        startPool[maxIdx].coeff.assign(curFilterCoef, curFilterCoef + numCoeff);
        startPool[maxIdx].scaleIdx = curScaleIdx;
        startPool[maxIdx].error = curError;
        startPool[maxIdx].bits = curBits;
        startPool[maxIdx].rdCost = curCost;
      }
    }
    if (curCost < costRef)
    {
      errRef = curError;
      bitsRef = curBits;
      costRef = curCost;
      std::copy_n(curFilterCoef, numCoeff, filterCoeffQuant);
      scaleIdxRef = curScaleIdx;
    }
  }

  return errRef;
}
#endif

#if JVET_AF0177_ALF_COV_FLOAT
void EncAdaptiveLoopFilter::roundFiltCoeffCCALF(int16_t *filterCoeffQuant, float *filterCoeff, const int numCoeff, const int factor)
#else
void EncAdaptiveLoopFilter::roundFiltCoeffCCALF(int16_t *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor)
#endif
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
    double best_err = MAX_DOUBLE;
#else
    double best_err = 128.0*128.0;
#endif
    int best_index = 0;
    for(int k = 0; k < CCALF_CANDS_COEFF_NR; k++)
    {
      double err = (filterCoeff[i] * sign * factor - CCALF_SMALL_TAB[k]);
      err = err*err;
      if(err < best_err)
      {
        best_err = err;
        best_index = k;
      }
    }
    filterCoeffQuant[i] = CCALF_SMALL_TAB[best_index] * sign;
  }
}
#if JVET_X0071_ALF_BAND_CLASSIFIER
void EncAdaptiveLoopFilter::mergeClasses( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] , const int altIdx, int mergedPair[MAX_NUM_ALF_CLASSES][2] )
#else
void EncAdaptiveLoopFilter::mergeClasses( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] 
#if ALF_IMPROVEMENT
  , const int altIdx
#endif
)
#endif
{
  int     tmpClip[MAX_NUM_ALF_LUMA_COEFF];
  int     bestMergeClip[MAX_NUM_ALF_LUMA_COEFF];
  double  err[MAX_NUM_ALF_CLASSES];
  double  bestMergeErr = std::numeric_limits<double>::max();
  bool    availableClass[MAX_NUM_ALF_CLASSES];
  uint8_t indexList[MAX_NUM_ALF_CLASSES];
  uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset( filterIndices, 0, sizeof( short ) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES );

  for( int i = 0; i < numClasses; i++ )
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
#if ALF_IMPROVEMENT
    covMerged[i].numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#else
    covMerged[i].numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#endif
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
#if ALF_IMPROVEMENT
  tmpCov.numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#else
  tmpCov.numBins = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[COMPONENT_Y] : 1;
#endif

#if ALF_IMPROVEMENT
  for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
  {
    for (int j = 0; j < MAX_NUM_ALF_CLASSES; j++)
    {
      classChanged[i][j] = true;
      memset(clipHistory[i][j], 0, sizeof(int)*MAX_NUM_ALF_LUMA_COEFF);
    }
  }
  memset(errorHistory, 0, sizeof(double)*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES);
#endif

  // init Clip
  for( int i = 0; i < numClasses; i++ )
  {
#if ALF_IMPROVEMENT
    std::fill_n(clipMerged[numRemaining - 1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    if ( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA][altIdx] )
#else
    std::fill_n(clipMerged[numRemaining-1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? AlfNumClippingValues[CHANNEL_TYPE_LUMA] / 2 : 0);
    if ( m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] )
#endif
    {
#if JVET_AD0222_ALF_RESI_CLASS
      err[i] = covMerged[i].optimizeFilterClip( alfShape, clipMerged[numRemaining - 1][i], m_enableLessClip 
#if JVET_AN0086_RESIDUAL_CHECK
      , m_encCfg->getAlfResidualCheckEnabled()
#endif
      );
#else
      err[i] = covMerged[i].optimizeFilterClip( alfShape, clipMerged[numRemaining-1][i] );
#endif
    }
    else
    {
      err[i] = covMerged[i].calculateError( clipMerged[numRemaining-1][i] 
#if JVET_AN0086_RESIDUAL_CHECK
      , m_encCfg->getAlfResidualCheckEnabled()
#endif
      );
    }
  }

  while( numRemaining >= 2 )
  {
    double errorMin = std::numeric_limits<double>::max();
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for( int i = 0; i < numClasses - 1; i++ )
    {
      if( availableClass[i] )
      {
        for( int j = i + 1; j < numClasses; j++ )
        {
          if( availableClass[j] )
          {
            double error1 = err[i];
            double error2 = err[j];

            tmpCov.add( covMerged[i], covMerged[j] );
            for( int l = 0; l < MAX_NUM_ALF_LUMA_COEFF; ++l )
            {
              tmpClip[l] = (clipMerged[numRemaining-1][i][l] + clipMerged[numRemaining-1][j][l] + 1 ) >> 1;
            }
#if ALF_IMPROVEMENT
            double errorMerged = 0;
            if (classChanged[i][j])
            {
              errorMerged = tmpCov.calculateError(tmpClip
#if JVET_AN0086_RESIDUAL_CHECK
                , m_encCfg->getAlfResidualCheckEnabled()
#endif
              );
              classChanged[i][j] = false;
              errorHistory[i][j] = errorMerged;
              memcpy(clipHistory[i][j], tmpClip, sizeof(tmpClip));
            }
            else
            {
              errorMerged = errorHistory[i][j];
              memcpy(tmpClip, clipHistory[i][j], sizeof(tmpClip));
            }
#else
            double errorMerged = m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] ? tmpCov.optimizeFilterClip(alfShape, tmpClip) : tmpCov.calculateError(tmpClip);
#endif

            double error = errorMerged - error1 - error2;

            if( error < errorMin )
            {
              bestMergeErr = errorMerged;
              memcpy(bestMergeClip, tmpClip, sizeof(bestMergeClip));
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    memcpy(clipMerged[numRemaining-2], clipMerged[numRemaining-1], sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    memcpy(clipMerged[numRemaining-2][bestToMergeIdx1], bestMergeClip, sizeof(bestMergeClip));
    err[bestToMergeIdx1] = bestMergeErr;
    availableClass[bestToMergeIdx2] = false;
#if ALF_IMPROVEMENT
    for (int i = 0; i < numClasses; i++)
    {
      classChanged[bestToMergeIdx1][i] = classChanged[i][bestToMergeIdx1] = true;
    }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
    mergedPair[numRemaining - 1][0] = bestToMergeIdx1;
    mergedPair[numRemaining - 1][1] = bestToMergeIdx2;
    if( numRemaining == 2 )
    {
      int ind = 0;
      for (int i = 0; i < numClasses; i++)
      {
        if( availableClass[i] )
        {
          mergedPair[numRemaining - 2][ind] = i;
          ind++;
        }
      }
    }
#endif
    for( int i = 0; i < numClasses; i++ )
    {
      if( indexList[i] == bestToMergeIdx2 )
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if( numRemaining <= numClasses )
    {
      std::memcpy( indexListTemp, indexList, sizeof( uint8_t ) * numClasses );

      bool exist = false;
      int ind = 0;

      for( int j = 0; j < numClasses; j++ )
      {
        exist = false;
        for( int i = 0; i < numClasses; i++ )
        {
          if( indexListTemp[i] == j )
          {
            exist = true;
            break;
          }
        }

        if( exist )
        {
          for( int i = 0; i < numClasses; i++ )
          {
            if( indexListTemp[i] == j )
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}
#if JVET_X0071_ALF_BAND_CLASSIFIER
void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx, int altIdx, int fixedFilterSetIdx, int classifierIdx )
#else
void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx, int altIdx 
#if ALF_IMPROVEMENT
  , int fixedFilterSetIdx
#endif
)
#endif
{
#if JVET_X0071_ALF_BAND_CLASSIFIER
  int numClasses = isLuma(channel) ? ALF_NUM_CLASSES_CLASSIFIER[classifierIdx] : 1;
#else
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
#endif

  for (int i = 0; i < numClasses; i++)
  {
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
  }
  if (isLuma(channel))
  {
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], m_ctuAlternative[COMPONENT_Y], numClasses, altIdx, fixedFilterSetIdx, classifierIdx );
#else
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], m_ctuAlternative[COMPONENT_Y], numClasses, altIdx, fixedFilterSetIdx );
#endif
#else
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], nullptr, numClasses, altIdx );
#endif
  }
  else
  {
#if JVET_X0071_ALF_BAND_CLASSIFIER
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], m_ctuAlternative[COMPONENT_Cb], numClasses, altIdx, fixedFilterSetIdx, classifierIdx );
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], m_ctuAlternative[COMPONENT_Cr], numClasses, altIdx, fixedFilterSetIdx, classifierIdx );
#else
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], m_ctuAlternative[COMPONENT_Cb], numClasses, altIdx 
#if ALF_IMPROVEMENT
      , fixedFilterSetIdx
#endif
    );
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], m_ctuAlternative[COMPONENT_Cr], numClasses, altIdx 
#if ALF_IMPROVEMENT
      , fixedFilterSetIdx
#endif
    );
#endif
  }
}

#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance**** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx, int fixedFilterSetIdx, int classifierIdx )
#else
void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance*** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx, int fixedFilterSetIdx )
#endif
#else
void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx )
#endif
{
#if !ALF_IMPROVEMENT
  const ChannelType channel = (!ctbAltIdx ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA);
#endif
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    if( ctbEnableFlags[ctuIdx]  )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
#if ALF_IMPROVEMENT
        if (altIdx == ctbAltIdx[ctuIdx])
        {
#if JVET_X0071_ALF_BAND_CLASSIFIER
          frameCov[classIdx] += ctbCov[ctuIdx][fixedFilterSetIdx][classifierIdx][classIdx];
#else
          frameCov[classIdx] += ctbCov[ctuIdx][fixedFilterSetIdx][classIdx];
#endif
        }
#else
        if( isLuma( channel ) || altIdx == ctbAltIdx[ctuIdx] )
        {
          frameCov[classIdx] += ctbCov[ctuIdx][classIdx];
        }
#endif
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv, CodingStructure& cs )
{
  int ctuRsAddr = 0;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );

  // init CTU stats buffers
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
#if JVET_X0071_ALF_BAND_CLASSIFIER
    for( int classifierIdx = 0; classifierIdx < (isLuma(compID) ? ALF_NUM_CLASSIFIER : 1); classifierIdx++ )
    {
#if JVET_AD0222_ALF_RESI_CLASS
      if( isLuma( compID ) && classifierIdx == ALF_NUM_CLASSIFIER - 1 && cs.slice->isIntra() )
      {
        continue;
      }
#endif
      const int numClasses = isLuma(compID) ? ALF_NUM_CLASSES_CLASSIFIER[classifierIdx] : 1;
#else
    const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;
#endif

    for( int shape = 0; shape != m_filterShapes[toChannelType( compID )].size(); shape++ )
    {
#if ALF_IMPROVEMENT
      if (m_alfCovariance[compIdx][shape] == nullptr
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        || m_alfCovariance[compIdx] == nullptr
#endif
        )
      {
        continue;
      }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      if( m_filterTypeTest[toChannelType(compID)][m_filterShapes[toChannelType(compID)][shape].filterType] == false )
      {
        continue;
      }
      int numFixedFilterSet = numFixedFilters( m_filterShapes[toChannelType(compID)][shape].filterType );
#if FIXFILTER_CFG
      if( m_encCfg->getUseAlfFixedFilter() == false )
      {
        numFixedFilterSet = 1;
      }
#endif
#else
      int numFixedFilterSet = ( m_filterShapes[toChannelType(compID)][shape].filterType == ALF_FILTER_EXT || m_filterShapes[toChannelType(compID)][shape].filterType == ALF_FILTER_9_EXT ) ? 2 : 1;
#endif
#endif
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
        {
#if ALF_IMPROVEMENT
          for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
          {
#if JVET_X0071_ALF_BAND_CLASSIFIER
            m_alfCovariance[compIdx][shape][ctuIdx][fixedFilterSetIdx][classifierIdx][classIdx].reset();
#else
            m_alfCovariance[compIdx][shape][ctuIdx][fixedFilterSetIdx][classIdx].reset(AlfNumClippingValues[toChannelType(compID)]);
#endif
          }
#else
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset(AlfNumClippingValues[toChannelType( compID )]);
#endif
        }
      }
    }
#if JVET_X0071_ALF_BAND_CLASSIFIER
    }
#endif
  }

  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels( m_chromaFormat );
  for( int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++ )
  {
    const ChannelType channelID = ChannelType( channelIdx );
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
    {
#if ALF_IMPROVEMENT
      if (m_alfCovarianceFrame[channelIdx][shape] == nullptr)
      {
        continue;
      }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      if( m_filterTypeTest[channelIdx][m_filterShapes[channelIdx][shape].filterType] == false )
      {
        continue;
      }
#endif
#endif
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset(AlfNumClippingValues[channelID]);
      }
    }
  }

  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };

#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  PelUnitBuf recYuvBeforeDb = m_tempBufBeforeDb.getBuf( cs.area );
#else
  PelUnitBuf recYuvBeforeDb = m_tempBufBeforeDb.getBuf( UnitArea ( CHROMA_400, cs.area.blocks[COMPONENT_Y] ) );
#endif
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  PelUnitBuf resiYuv = m_tempBufResi.getBuf(UnitArea(CHROMA_400, cs.area.blocks[COMPONENT_Y]));
#endif

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
      int rasterSliceAlfPad = 0;

      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf recBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            recBuf.copyFrom( recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, recBuf, MAX_ALF_PADDING_SIZE, true, true);
#else
            recBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
#endif
            recBuf = recBuf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

            const UnitArea area( m_chromaFormat, Area( 0, 0, w, h ) );
            const UnitArea areaDst( m_chromaFormat, Area( xStart, yStart, w, h ) );
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if ( m_isFixedFilterPaddedPerCtu )
            {
              for(int fixedFilterSetIdx = 0; fixedFilterSetIdx < NUM_FIXED_FILTER_SETS; fixedFilterSetIdx++ )
              {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
                paddingFixedFilterResultsCtu(m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area(xStart, yStart, w, h), 0);
                paddingFixedFilterResultsCtu(m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area(xStart, yStart, w, h), 1);
#else
                paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area(xStart, yStart, w, h), 0);
                paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area(xStart, yStart, w, h), 1);
#endif
#else
                paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area(xStart, yStart, w, h));
#endif
              }
            }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            if( m_isFixedFilterPaddedPerCtu )
            {
              for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
              {
                paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, Area(xStart, yStart, w, h));
              }
            }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
            if( m_isFixedFilterPaddedPerCtu )
            {
              for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++)
              {
                paddingLaplacianResultsCtu(m_laplacianPic, m_laplacianCtu, laplacianIdx, Area(xStart, yStart, w, h));
              }
            }
#endif
            for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
            {
              const ComponentID compID = ComponentID( compIdx );
              const CompArea& compArea = area.block( compID );

              int  recStride = recBuf.get( compID ).stride;
              Pel* rec = recBuf.get( compID ).bufAt( compArea );

              int  orgStride = orgYuv.get(compID).stride;
              Pel* org = orgYuv.get(compID).bufAt(xStart >> ::getComponentScaleX(compID, m_chromaFormat), yStart >> ::getComponentScaleY(compID, m_chromaFormat));
              ChannelType chType = toChannelType( compID );
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              const int numClasses = isLuma(compID) ? MAX_NUM_ALF_CLASSES : 1;
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
              PelUnitBuf recBufDb = m_tempBufBeforeDb2.subBuf( UnitArea( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
              recBufDb.copyFrom( recYuvBeforeDb.subBuf( UnitArea( CHROMA_400, Area( xStart - (clipL ? 0 : NUM_DB_PAD ), yStart - ( clipT ? 0 : NUM_DB_PAD ), wBuf, hBuf ) ) ) );
              // pad top-left unavailable samples for raster slice
              if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
              {
                recBufDb.padBorderPel( NUM_DB_PAD, 1 );
              }

              // pad bottom-right unavailable samples for raster slice
              if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
              {
                recBufDb.padBorderPel( NUM_DB_PAD, 2 );
              }
              recBufDb.extendBorderPel( NUM_DB_PAD );
              recBufDb = recBufDb.subBuf( UnitArea ( CHROMA_400, Area( clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h ) ) );
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
              PelUnitBuf resiBuf = m_tempBufResi2.subBuf(UnitArea(CHROMA_400, Area(0, 0, wBuf, hBuf)));
              resiBuf.copyFrom(
                resiYuv.subBuf(UnitArea(CHROMA_400, Area(xStart - (clipL ? 0 : NUM_RESI_PAD),
                                                         yStart - (clipT ? 0 : NUM_RESI_PAD), wBuf, hBuf))));
              // pad top-left unavailable samples for raster slice
              if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
              {
                resiBuf.padBorderPel(NUM_RESI_PAD, 1);
              }

              // pad bottom-right unavailable samples for raster slice
              if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
              {
                resiBuf.padBorderPel(NUM_RESI_PAD, 2);
              }
              resiBuf.extendBorderPel(NUM_RESI_PAD);
              resiBuf =
                resiBuf.subBuf(UnitArea(CHROMA_400, Area(clipL ? 0 : NUM_RESI_PAD, clipT ? 0 : NUM_RESI_PAD, w, h)));
#endif

              for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
              {
              const CompArea& compAreaDst = areaDst.block( compID );

#if ALF_IMPROVEMENT
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                if (m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false)
                {
                  continue;
                }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if FIXFILTER_CFG
                for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < ( m_encCfg->getUseAlfFixedFilter() ? numFixedFilters(m_filterShapes[chType][shape].filterType) : 1); fixedFilterSetIdx++)
#else
              for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilters( m_filterShapes[chType][shape].filterType ); fixedFilterSetIdx++)
#endif
#else
              for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < ((m_filterShapes[chType][shape].filterType == ALF_FILTER_EXT || m_filterShapes[chType][shape].filterType == ALF_FILTER_9_EXT) ? 2 : 1); fixedFilterSetIdx++)
#endif
              {
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AD0222_ALF_RESI_CLASS
                int classifierIdx = 0;
                getBlkStats<alfWSSD, false>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, nullptr, nullptr, isLuma(compID) ? recBufDb.get(compID).bufAt(compArea) : nullptr, isLuma(compID) ? recBufDb.get(compID).stride : 0, isLuma(compID) ? resiBuf.get(compID).bufAt(compAreaDst) : nullptr, isLuma(compID) ? resiBuf.get(compID).stride : 0, compAreaDst, compArea, chType, fixedFilterSetIdx, classifierIdx);
                if (isLuma(compID))
                {
                  classifierIdx = 1;
                  if (cs.slice->isIntra())
                  {
                    getBlkStats<alfWSSD, false>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, nullptr, nullptr, isLuma(compID) ? recBufDb.get(compID).bufAt(compArea) : nullptr, isLuma(compID) ? recBufDb.get(compID).stride : 0, isLuma(compID) ? resiBuf.get(compID).bufAt(compAreaDst) : nullptr, isLuma(compID) ? resiBuf.get(compID).stride : 0, compAreaDst, compArea, chType, fixedFilterSetIdx, classifierIdx);
                  }
                  else
                  {
                    getBlkStats<alfWSSD, true>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, m_classifier[2], m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][2], isLuma(compID) ? recBufDb.get(compID).bufAt(compArea) : nullptr, isLuma(compID) ? recBufDb.get(compID).stride : 0, isLuma(compID) ? resiBuf.get(compID).bufAt(compAreaDst) : nullptr,
                      isLuma(compID) ? resiBuf.get(compID).stride : 0, compAreaDst, compArea, chType, fixedFilterSetIdx, classifierIdx);
                  }
                }
#else
                for (int classifierIdx = 0; classifierIdx < (compIdx ? 1 : ALF_NUM_CLASSIFIER); classifierIdx++)
                {
                  getBlkStats<alfWSSD>( m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                    isLuma( compID ) ? recBufDb.get( compID ).bufAt( compArea ) : nullptr, isLuma( compID ) ? recBufDb.get( compID ).stride : 0,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                    isLuma(compID) ? resiBuf.get(compID).bufAt(compArea) : nullptr,
                    isLuma(compID) ? resiBuf.get(compID).stride : 0,
#endif
                    compAreaDst, compArea, chType, fixedFilterSetIdx, classifierIdx );
                }
#endif
#else
                getBlkStats<alfWSSD>( m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                  recBufDb.get(compID).bufAt(compArea), recBufDb.get(compID).stride,
#endif
                  compAreaDst, compArea, chType, fixedFilterSetIdx);
#endif
              }
#else
              getBlkStats( m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compAreaDst, compArea, chType, ((compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight), (compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos );
#endif
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              for (int classIdx = 0; classIdx < numClasses; classIdx++)
              {
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                if(isLuma(compID))
                {
                  continue;
                }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
                m_alfCovarianceFrame[chType][shape][isLuma(compID) ? classIdx : 0] +=
                  m_alfCovariance[compIdx][shape][ctuRsAddr][0][0][classIdx];
#else
                m_alfCovarianceFrame[chType][shape][isLuma(compID) ? classIdx : 0] +=
                  m_alfCovariance[compIdx][shape][ctuRsAddr][0][classIdx];
#endif
#else
                m_alfCovarianceFrame[chType][shape][isLuma(compID) ? classIdx : 0] +=
                  m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
              }
#endif
              }
            }

            xStart = xEnd;
          }

          yStart = yEnd;
        }
#if !JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
        for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
        {
          const ComponentID compID = ComponentID( compIdx );
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if(isLuma(compID))
          {
            continue;
          }
#endif
          ChannelType chType = toChannelType( compID );


          for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
          {
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if( m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false )
            {
              continue;
            }
#endif
            const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

            for( int classIdx = 0; classIdx < numClasses; classIdx++ )
            {
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
              m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][0][classIdx];
#else
              m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][classIdx];
#endif
#else
              m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
            }
          }
        }
#endif
      }
      else
      {
        const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        if ( m_isFixedFilterPaddedPerCtu )
        {
          for(int fixedFilterSetIdx = 0; fixedFilterSetIdx < NUM_FIXED_FILTER_SETS; fixedFilterSetIdx++ )
          {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            paddingFixedFilterResultsCtu( m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area( xPos, yPos, width, height ), 0 );
            paddingFixedFilterResultsCtu( m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area( xPos, yPos, width, height ), 1 );
#else
            paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area( xPos, yPos, width, height ), 0 );
            paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area( xPos, yPos, width, height ), 1 );
#endif
#else
            paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, Area( xPos, yPos, width, height ));
#endif
          }
        }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
        if( m_isFixedFilterPaddedPerCtu )
        {
          for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
          {
            paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, Area( xPos, yPos, width, height ));
          }
        }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
        if( m_isFixedFilterPaddedPerCtu )
        {
          for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++)
          {
            paddingLaplacianResultsCtu(m_laplacianPic, m_laplacianCtu, laplacianIdx, Area( xPos, yPos, width, height ));
          }
        }
#endif
        for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
        {
          const ComponentID compID = ComponentID( compIdx );
          const CompArea &  compArea = area.block( compID );

          int  recStride = recYuv.get( compID ).stride;
          Pel *rec = recYuv.get( compID ).bufAt( compArea );

#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
          int  recDbStride = isLuma( compID ) ? recYuvBeforeDb.get( compID ).stride : 0;
          Pel *recDb = isLuma( compID ) ? recYuvBeforeDb.get( compID ).bufAt( compArea ) : nullptr;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
          int  resiStride = isLuma(compID) ? resiYuv.get(compID).stride : 0;
          Pel *resi       = isLuma(compID) ? resiYuv.get(compID).bufAt(compArea) : nullptr;
#endif
          int  orgStride = orgYuv.get( compID ).stride;
          Pel *org = orgYuv.get( compID ).bufAt( compArea );

          ChannelType chType = toChannelType( compID );
          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
          {
#if ALF_IMPROVEMENT
            if( m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false )
            {
              continue;
            }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if FIXFILTER_CFG
            for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < ( m_encCfg->getUseAlfFixedFilter() ? numFixedFilters(m_filterShapes[chType][shape].filterType) : 1); fixedFilterSetIdx++)
#else
            for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilters(m_filterShapes[chType][shape].filterType); fixedFilterSetIdx++)
#endif
#else
            for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < ((m_filterShapes[chType][shape].filterType == ALF_FILTER_EXT || m_filterShapes[chType][shape].filterType == ALF_FILTER_9_EXT) ? 2 : 1); fixedFilterSetIdx++ )
#endif

            {
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AD0222_ALF_RESI_CLASS
              int classifierIdx = 0;
              getBlkStats<alfWSSD, false>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, nullptr, nullptr, recDb, recDbStride, resi, resiStride, compArea, compArea, chType, fixedFilterSetIdx, classifierIdx);
              if (isLuma(compID))
              {
                classifierIdx = 1;
                if (cs.slice->isIntra())
                {
                  getBlkStats<alfWSSD, false>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, nullptr, nullptr, recDb, recDbStride, resi, resiStride, compArea, compArea, chType, fixedFilterSetIdx, classifierIdx);
                }
                else
                {
                  getBlkStats<alfWSSD, true>(m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride, m_classifier[2], m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][2], recDb, recDbStride, resi, resiStride, compArea, compArea, chType, fixedFilterSetIdx, classifierIdx);
                }
              }
#else
              for( int classifierIdx = 0; classifierIdx < (compIdx ? 1 : ALF_NUM_CLASSIFIER); classifierIdx++ )
              {
                getBlkStats<alfWSSD>( m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx][classifierIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier[classifierIdx], org, orgStride, rec, recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                  recDb, recDbStride,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                  resi, resiStride,
#endif
                  compArea, compArea, chType, fixedFilterSetIdx, classifierIdx );
              }
#endif
#else
              getBlkStats<alfWSSD>( m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetIdx], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                  recDb, recDbStride,
#endif
                  compArea, compArea, chType, fixedFilterSetIdx );
#endif
            }
#else
            getBlkStats( m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea, compArea, chType, ( ( compIdx == 0 ) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight ), ( compIdx == 0 ) ? m_alfVBLumaPos : m_alfVBChmaPos );
#endif        

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if(isLuma(compID))
            {
              continue;
            }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
            m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][0][classIdx];
#else
            m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][classIdx];
#endif
#else
            m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
#endif
          }
          }
        }
      }
      ctuRsAddr++;
    }
  }

  if (m_alfWSSD)
  {
    initDistortion<true>(
#if ALF_IMPROVEMENT
      cs
#endif
      );
  }
  else
  {
    initDistortion<false>(
#if ALF_IMPROVEMENT
      cs
#endif
      );
  }
}

#if ALF_IMPROVEMENT
#if JVET_AD0222_ALF_RESI_CLASS
template<bool m_alfWSSD, bool reuse>
#else
template<bool m_alfWSSD>
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, const Pel* org, const int orgStride, const Pel* rec, const int recStride,
#if JVET_AD0222_ALF_RESI_CLASS
  AlfClassifier** classifierNext, AlfCovariance* alfCovarianceNext,
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const Pel *resi, const int resiStride,
#endif
  const CompArea& areaDst, const CompArea& area, const ChannelType channel, int fixedFilterSetIdx, int classifierIdx )
#else
void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, const Pel* org, const int orgStride, const Pel* rec, const int recStride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
    const CompArea& areaDst, const CompArea& area, const ChannelType channel, int fixedFilterSetIdx )
#endif
#else
void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos )
#endif
{
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];
#else
  int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int numCoeff = shape.numCoeff;
  if( shape.filterType == ALF_FILTER_9_EXT_DB || shape.filterType == ALF_FILTER_13_EXT_DB )
  {
    numCoeff = shape.numCoeff + 1;
  }
#endif

  const int numBins = alfCovariance[0].numBins;
  int transposeIdx = 0;
  int classIdx = 0;
#if JVET_AD0222_ALF_RESI_CLASS
  int classIdxNext = 0;
#if JVET_AF0177_ALF_COV_FLOAT
  float curY, curP;
#else
  double curY, curP;
#endif
#endif

  for( int i = 0; i < area.height; i++ )
  {
#if !ALF_IMPROVEMENT
    int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
#endif
    for( int j = 0; j < area.width; j++ )
    {
#if ALF_IMPROVEMENT
      if( classifier && (classifier[areaDst.y + i][areaDst.x + j]>>2) == m_ALF_UNUSED_CLASSIDX && (classifier[areaDst.y + i][areaDst.x + j] & 0x3) == m_ALF_UNUSED_TRANSPOSIDX )
#else
      if( classifier && classifier[areaDst.y + i][areaDst.x + j].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[areaDst.y + i][areaDst.x + j].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
#endif
      {
        continue;
      }
      std::memset( ELocal, 0, sizeof( ELocal ) );
      if( classifier )
      {
        AlfClassifier& cl = classifier[areaDst.y + i][areaDst.x + j];
#if ALF_IMPROVEMENT
        transposeIdx = cl & 0x3;
        classIdx = cl >> 2;
#else
        transposeIdx = cl.transposeIdx;
        classIdx = cl.classIdx;
#endif
      }
#if JVET_AD0222_ALF_RESI_CLASS
      if( classifierNext )
      {
        classIdxNext = classifierNext[areaDst.y + i][areaDst.x + j] >> 2;
        CHECK(!alfCovariance[classIdx].sameSizeAs(alfCovarianceNext[classIdxNext]), "Covariance size mismatch");
      }
#endif
#if JVET_AF0177_ALF_COV_FLOAT
      float weight = 1.0;
#else
      double weight = 1.0;
#endif
      if( m_alfWSSD )
      {
#if JVET_AF0177_ALF_COV_FLOAT
        weight = (float)m_lumaLevelToWeightPLUT[org[j]];
#else
        weight = m_lumaLevelToWeightPLUT[org[j]];
#endif
      }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      Intermediate_Int yLocal = org[j] - rec[j];
#else
      int yLocal = org[j] - rec[j];
#endif

#if ALF_IMPROVEMENT
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      Position posInCtu = Position( j, i);
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
      calcCovariance( ELocal, rec + j, recStride, recBeforeDb + j, recBeforeDbStride, resi + j, resiStride, shape, transposeIdx, channel, ( shape.filterType >= ALF_FILTER_9_EXT && isLuma( channel ) ) ? m_fixFilterResult[COMPONENT_Y] : ( shape.filterType == ALF_FILTER_9 && isChroma( channel ) ? m_fixFilterResult[area.compID] : nullptr ), shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResiResult : nullptr, Position(areaDst.x + j, areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx, posInCtu );
#else
      calcCovariance(ELocal, rec + j, recStride, recBeforeDb + j, recBeforeDbStride, resi + j, resiStride, shape,
                     transposeIdx, channel, shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResult : nullptr,
                     shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResiResult : nullptr,
                     Position(areaDst.x + j, areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx,
                     posInCtu);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
calcCovariance( ELocal, rec + j, recStride, recBeforeDb + j, recBeforeDbStride, shape, transposeIdx, channel, shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResult : nullptr, Position(areaDst.x + j ,areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx, posInCtu );
#else
      calcCovariance( ELocal, rec + j, recStride, recBeforeDb + j, recBeforeDbStride, shape, transposeIdx, channel, shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResult : nullptr, Position(areaDst.x + j ,areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx );
#endif
#endif
#elif JVET_AA0095_ALF_LONGER_FILTER
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResult : nullptr, Position(areaDst.x + j, areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx, posInCtu);
#else
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, shape.filterType >= ALF_FILTER_9_EXT ? m_fixFilterResult : nullptr, Position(areaDst.x + j, areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, ( shape.filterType == ALF_FILTER_9_EXT || shape.filterType == ALF_FILTER_EXT ) ? m_fixFilterResult : nullptr, Position(areaDst.x + j ,areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx, posInCtu );
#else
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, ( shape.filterType == ALF_FILTER_9_EXT || shape.filterType == ALF_FILTER_EXT ) ? m_fixFilterResult : nullptr, Position(areaDst.x + j ,areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetIdx );
#endif
#endif
#else
      calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance );
#endif

      for (ptrdiff_t b0 = 0; b0 < numBins; b0++)
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for (ptrdiff_t k = 0; k < numCoeff; k++)
#else
      for (ptrdiff_t k = 0; k < shape.numCoeff; k++)
#endif
        {
          for (ptrdiff_t b1 = 0; b1 <= b0; b1++)
          {
          const ptrdiff_t maxl = b0 == b1 ? k + 1 : shape.numCoeff;
          for (ptrdiff_t l = 0; l < maxl; l++)
          {
            if (m_alfWSSD)
            {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
              const ptrdiff_t oe   = alfCovariance[classIdx].getOffsetEfast(b0, b1, k, l);
#if JVET_AF0177_ALF_COV_FLOAT
              const float     curE = weight * (ELocal[k][b0] * (float)ELocal[l][b1]);
#else
              const double    curE = weight * (ELocal[k][b0] * (double) ELocal[l][b1]);
#endif
              alfCovariance[classIdx].data[oe] += curE;
              if (reuse)
              {
                  alfCovarianceNext[classIdxNext].data[oe] += curE;
              }
#else
                alfCovariance[classIdx].E(b0,b1,k,l) += weight * (ELocal[k][b0] * (double)ELocal[l][b1]);
#endif
#else
                alfCovariance[classIdx].E(b0,b1,k,l) += weight * (double)(ELocal[k][b0] * ELocal[l][b1]);
#endif
              }
              else
              {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
              const ptrdiff_t oe   = alfCovariance[classIdx].getOffsetEfast(b0, b1, k, l);
#if JVET_AF0177_ALF_COV_FLOAT
              const float     curE = ELocal[k][b0] * (float)ELocal[l][b1];
#else
              const double    curE = ELocal[k][b0] * (double) ELocal[l][b1];
#endif
              alfCovariance[classIdx].data[oe] += curE;
              if (reuse)
              {
                  alfCovarianceNext[classIdxNext].data[oe] += curE;
              }
#else
                alfCovariance[classIdx].E(b0,b1,k,l) += ELocal[k][b0] * (double)ELocal[l][b1];
#endif
#else
                alfCovariance[classIdx].E(b0,b1,k,l) += ELocal[k][b0] * ELocal[l][b1];
#endif
              }
          }
          }
        }
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for (ptrdiff_t k = 0; k < numCoeff; k++)
#else
        for (ptrdiff_t k = 0; k < shape.numCoeff; k++)
#endif
        {
          ptrdiff_t b = b0;
          if (m_alfWSSD)
          {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
            const ptrdiff_t oy = alfCovariance[classIdx].getOffsetY(b, k);
            if (reuse)
            {
#if JVET_AF0177_ALF_COV_FLOAT
              curY = weight * (ELocal[k][b] * (float)yLocal);
#else
              curY = weight * (ELocal[k][b] * (double)yLocal);
#endif
              alfCovariance[classIdx].data[oy] += curY;
              alfCovarianceNext[classIdxNext].data[oy] += curY;
            }
            else
            {
#if JVET_AF0177_ALF_COV_FLOAT
              alfCovariance[classIdx].data[oy] += weight * (ELocal[k][b] * (float)yLocal);
#else
              alfCovariance[classIdx].data[oy] += weight * (ELocal[k][b] * (double) yLocal);
#endif
            }
#else
            alfCovariance[classIdx].y[b][k] += weight * (ELocal[k][b] * (double)yLocal);
#endif
#else
            alfCovariance[classIdx].y[b][k] += weight * (double)(ELocal[k][b] * yLocal);
#endif
          }
          else
          {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
            const ptrdiff_t oy = alfCovariance[classIdx].getOffsetY(b, k);
            if( reuse )
            {
#if JVET_AF0177_ALF_COV_FLOAT
              curY = ELocal[k][b] * (float)yLocal;
#else
              curY = ELocal[k][b] * (double)yLocal;
#endif
              alfCovariance[classIdx].data[oy] += curY;
              alfCovarianceNext[classIdxNext].data[oy] += curY;
            }
            else
            {
#if JVET_AF0177_ALF_COV_FLOAT
              alfCovariance[classIdx].data[oy] += ELocal[k][b] * (float)yLocal;
#else
              alfCovariance[classIdx].data[oy] += ELocal[k][b] * (double) yLocal;
#endif
            }           
#else
            alfCovariance[classIdx].y[b][k] += ELocal[k][b] * (double)yLocal;
#endif
#else
            alfCovariance[classIdx].y[b][k] += ELocal[k][b] * yLocal;
#endif
          }
        }
      }
      if (m_alfWSSD)
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
        if( reuse )
        {
#if JVET_AF0177_ALF_COV_FLOAT
          curP = weight * (yLocal * (float)yLocal);
#else
         curP = weight * (yLocal * (double)yLocal);
#endif
         alfCovariance[classIdx].pixAcc += curP;
         alfCovarianceNext[classIdxNext].pixAcc += curP;
        }
        else
        {
#if JVET_AF0177_ALF_COV_FLOAT
          alfCovariance[classIdx].pixAcc += weight * (yLocal * (float)yLocal);
#else
          alfCovariance[classIdx].pixAcc += weight * (yLocal * (double)yLocal);
#endif
        }
#else
        alfCovariance[classIdx].pixAcc += weight * (yLocal * (double)yLocal);
#endif
#else
        alfCovariance[classIdx].pixAcc += weight * (double)(yLocal * yLocal);
#endif
      }
      else
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AD0222_ALF_RESI_CLASS
        if( reuse )
        {
#if JVET_AF0177_ALF_COV_FLOAT
          curP = yLocal * (float)yLocal;
#else
          curP = yLocal * (double)yLocal;
#endif
          alfCovariance[classIdx].pixAcc += curP;
          alfCovarianceNext[classIdxNext].pixAcc += curP;
        }
        else
        {
#if JVET_AF0177_ALF_COV_FLOAT
          alfCovariance[classIdx].pixAcc += yLocal * (float)yLocal;
#else
          alfCovariance[classIdx].pixAcc += yLocal * (double)yLocal;
#endif
        }
#else
        alfCovariance[classIdx].pixAcc += yLocal * (double)yLocal;
#endif
#else
        alfCovariance[classIdx].pixAcc += yLocal * yLocal;
#endif
      }
    }
    org += orgStride;
    rec += recStride;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    recBeforeDb += recBeforeDbStride;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    resi += resiStride;
#endif
  }
}

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if ALF_IMPROVEMENT
void EncAdaptiveLoopFilter::calcCovariance( Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const Pel *resi, const int resiStride,
#endif
  const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, Pel ***fixedFitlerResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFitlerResiResults,
#endif
  Position posDst, Position pos, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Position posInCtu
#endif
  )
#else
void EncAdaptiveLoopFilter::calcCovariance( Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance )
#endif
#else
#if ALF_IMPROVEMENT
void EncAdaptiveLoopFilter::calcCovariance( int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
     const Pel* recBeforeDb, const int recBeforeDbStride,
#endif
     const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, Pel *fixedFitlerResults, int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
     , Position posInCtu
#endif
     )
#else
void EncAdaptiveLoopFilter::calcCovariance( int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance )
#endif
#endif
{
#if !ALF_IMPROVEMENT
  int clipTopRow = -4;
  int clipBotRow = 4;
  if (vbDistance >= -3 && vbDistance < 0)
  {
    clipBotRow = -vbDistance - 1;
    clipTopRow = -clipBotRow; // symmetric
  }
  else if (vbDistance >= 0 && vbDistance < 3)
  {
    clipTopRow = -vbDistance;
    clipBotRow = -clipTopRow; // symmetric
  }
#endif
  const int *filterPattern = shape.pattern.data();
  const int halfFilterLength = shape.filterLength >> 1;
  const Pel* clip = m_alfClippingValues[channel];
  const int numBins = AlfNumClippingValues[channel];

  int k = 0;

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel curr = rec[0];
#else
  const short curr = rec[0];
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  int padSizeLaplacian = ALF_PADDING_SIZE_LAPLACIAN_RESULTS;
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  if( shape.filterType == ALF_FILTER_13_EXT || shape.filterType == ALF_FILTER_13_EXT_DB 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
      || shape.filterType == ALF_FILTER_13_EXT_DB_RESI
      || shape.filterType == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#if FIXFILTER_CFG
      || shape.filterType == ALF_FILTER_13_DB_RESI
      || shape.filterType == ALF_FILTER_13_DB_RESI_DIRECT
#endif
#endif
    )
#else
  if( shape.filterType == ALF_FILTER_13_EXT )
#endif
  {
#if JVET_AD0222_ALF_LONG_FIXFILTER
    const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;

    pImg0 = rec;
    pImg1 = pImg0 + stride;
    pImg2 = pImg0 - stride;
    pImg3 = pImg1 + stride;
    pImg4 = pImg2 - stride;
    pImg5 = pImg3 + stride;
    pImg6 = pImg4 - stride;
    pImg7 = pImg5 + stride;
    pImg8 = pImg6 - stride;

    if (transposeIdx == 1)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[6][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg3[+0], pImg4[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[0][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else if (transposeIdx == 2)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg3[+0], pImg4[-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else if (transposeIdx == 3)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[6][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg3[+0], pImg4[-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[0][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg3[+0], pImg4[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    k = 10;
#else
    const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10;
    const Pel *pImg11, *pImg12;

    pImg0 = rec;
    pImg1 = pImg0 + stride;
    pImg2 = pImg0 - stride;
    pImg3 = pImg1 + stride;
    pImg4 = pImg2 - stride;
    pImg5 = pImg3 + stride;
    pImg6 = pImg4 - stride;
    pImg7 = pImg5 + stride;
    pImg8 = pImg6 - stride;
    pImg9 = pImg7 + stride;
    pImg10 = pImg8 - stride;
    pImg11 = pImg9 + stride;
    pImg12 = pImg10 - stride;

    if(transposeIdx == 1)
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[14][b] += clipALF(clip[b], curr, pImg11[+0], pImg12[-0]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg9[+0], pImg10[-0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);

        ELocal[4][b] += clipALF(clip[b], curr, pImg3[+2],  pImg4[-2]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg3[+1],  pImg4[-1]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg3[+0],  pImg4[-0]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg3[-1],  pImg4[+1]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg3[-2],  pImg4[+2]);

        ELocal[5][b]  += clipALF(clip[b], curr, pImg1[+2], pImg2[-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg1[-2], pImg2[+2]);

        ELocal[0][b] += clipALF(clip[b], curr, pImg0[+6], pImg0[-6]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg0[+5], pImg0[-5]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else if(transposeIdx == 2)
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg11[+0], pImg12[-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg9[+0], pImg10[-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);

        ELocal[8][b] += clipALF(clip[b], curr, pImg3[+2],  pImg4[-2]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg3[+1],  pImg4[-1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg3[+0],  pImg4[-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg3[-1],  pImg4[+1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg3[-2],  pImg4[+2]);

        ELocal[13][b]  += clipALF(clip[b], curr, pImg1[+2], pImg2[-2]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg1[-2], pImg2[+2]);

        ELocal[14][b] += clipALF(clip[b], curr, pImg0[+6], pImg0[-6]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg0[+5], pImg0[-5]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else if(transposeIdx == 3)
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[14][b] += clipALF(clip[b], curr, pImg11[+0], pImg12[-0]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg9[+0], pImg10[-0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);

        ELocal[8][b] += clipALF(clip[b], curr, pImg3[+2],  pImg4[-2]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg3[+1],  pImg4[-1]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg3[+0],  pImg4[-0]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg3[-1],  pImg4[+1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg3[-2],  pImg4[+2]);

        ELocal[7][b]  += clipALF(clip[b], curr, pImg1[+2], pImg2[-2]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg1[-2], pImg2[+2]);

        ELocal[0][b] += clipALF(clip[b], curr, pImg0[+6], pImg0[-6]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg0[+5], pImg0[-5]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    else
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg11[+0], pImg12[-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg9[+0], pImg10[-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg7[+0], pImg8[-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg5[+0], pImg6[-0]);

        ELocal[4][b] += clipALF(clip[b], curr, pImg3[+2],  pImg4[-2]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg3[+1],  pImg4[-1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg3[+0],  pImg4[-0]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg3[-1],  pImg4[+1]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg3[-2],  pImg4[+2]);

        ELocal[9][b]  += clipALF(clip[b], curr, pImg1[+2], pImg2[-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg1[+1], pImg2[-1]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg1[+0], pImg2[-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg1[-1], pImg2[+1]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg1[-2], pImg2[+2]);

        ELocal[14][b] += clipALF(clip[b], curr, pImg0[+6], pImg0[-6]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg0[+5], pImg0[-5]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg0[+4], pImg0[-4]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg0[+3], pImg0[-3]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg0[+2], pImg0[-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg0[+1], pImg0[-1]);
      }
    }
    k = 20;
#endif
  }
  else
  {
#endif
  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
#if ALF_IMPROVEMENT
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;
#else
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
#endif
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
#if ALF_IMPROVEMENT
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
#else
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
#endif
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
#if ALF_IMPROVEMENT
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i * stride], rec[-i * stride]);
#else
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
#endif
      }
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
#if ALF_IMPROVEMENT
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;
#else
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
#endif

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
      {
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
#if ALF_IMPROVEMENT
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
#else
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
#endif
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
#if ALF_IMPROVEMENT
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i * stride], rec[-i * stride]);
#else
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
#endif
      }
    }
  }
#if JVET_AA0095_ALF_LONGER_FILTER
  } //data collection for new shape
#endif
#if ALF_IMPROVEMENT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
  Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
  Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

  for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++ )
  {
    if( m_isFixedFilterPaddedPerCtu )
    {
      pImg0Gauss[gaussIdx] = &m_gaussCtu[gaussIdx][posInCtu.y + padSizeGauss + 0][posInCtu.x + padSizeGauss];
      pImg1Gauss[gaussIdx] = &m_gaussCtu[gaussIdx][posInCtu.y + padSizeGauss + 1][posInCtu.x + padSizeGauss];
      pImg2Gauss[gaussIdx] = &m_gaussCtu[gaussIdx][posInCtu.y + padSizeGauss - 1][posInCtu.x + padSizeGauss];
      pImg3Gauss[gaussIdx] = &m_gaussCtu[gaussIdx][posInCtu.y + padSizeGauss + 2][posInCtu.x + padSizeGauss];
      pImg4Gauss[gaussIdx] = &m_gaussCtu[gaussIdx][posInCtu.y + padSizeGauss - 2][posInCtu.x + padSizeGauss];

    }
    else
    {
      pImg0Gauss[gaussIdx] = &m_gaussPic[gaussIdx][posDst.y + padSizeGauss + 0][posDst.x + padSizeGauss];
      pImg1Gauss[gaussIdx] = &m_gaussPic[gaussIdx][posDst.y + padSizeGauss + 1][posDst.x + padSizeGauss];
      pImg2Gauss[gaussIdx] = &m_gaussPic[gaussIdx][posDst.y + padSizeGauss - 1][posDst.x + padSizeGauss];
      pImg3Gauss[gaussIdx] = &m_gaussPic[gaussIdx][posDst.y + padSizeGauss + 2][posDst.x + padSizeGauss];
      pImg4Gauss[gaussIdx] = &m_gaussPic[gaussIdx][posDst.y + padSizeGauss - 2][posDst.x + padSizeGauss];

    }
  }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
  Pel *pImg0Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
  Pel *pImg1Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg2Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];
  Pel *pImg3Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE], *pImg4Laplacian[NUM_LAPLACIAN_FILTERED_SOURCE];

  for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++ )
  {
    if( m_isFixedFilterPaddedPerCtu )
    {
      pImg0Laplacian[laplacianIdx] = &m_laplacianCtu[laplacianIdx][posInCtu.y + padSizeLaplacian + 0][posInCtu.x + padSizeLaplacian];
      pImg1Laplacian[laplacianIdx] = &m_laplacianCtu[laplacianIdx][posInCtu.y + padSizeLaplacian + 1][posInCtu.x + padSizeLaplacian];
      pImg2Laplacian[laplacianIdx] = &m_laplacianCtu[laplacianIdx][posInCtu.y + padSizeLaplacian - 1][posInCtu.x + padSizeLaplacian];
      pImg3Laplacian[laplacianIdx] = &m_laplacianCtu[laplacianIdx][posInCtu.y + padSizeLaplacian + 2][posInCtu.x + padSizeLaplacian];
      pImg4Laplacian[laplacianIdx] = &m_laplacianCtu[laplacianIdx][posInCtu.y + padSizeLaplacian - 2][posInCtu.x + padSizeLaplacian];
    }
    else
    {
      pImg0Laplacian[laplacianIdx] = &m_laplacianPic[laplacianIdx][posDst.y + padSizeLaplacian + 0][posDst.x + padSizeLaplacian];
      pImg1Laplacian[laplacianIdx] = &m_laplacianPic[laplacianIdx][posDst.y + padSizeLaplacian + 1][posDst.x + padSizeLaplacian];
      pImg2Laplacian[laplacianIdx] = &m_laplacianPic[laplacianIdx][posDst.y + padSizeLaplacian - 1][posDst.x + padSizeLaplacian];
      pImg3Laplacian[laplacianIdx] = &m_laplacianPic[laplacianIdx][posDst.y + padSizeLaplacian + 2][posDst.x + padSizeLaplacian];
      pImg4Laplacian[laplacianIdx] = &m_laplacianPic[laplacianIdx][posDst.y + padSizeLaplacian - 2][posDst.x + padSizeLaplacian];
    }
  }
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if FIXFILTER_CFG
  if (shape.filterType >= ALF_FILTER_13_EXT && shape.filterType != ALF_FILTER_9_EXT_DB && shape.filterType != ALF_FILTER_13_DB_RESI && shape.filterType != ALF_FILTER_13_DB_RESI_DIRECT && shape.filterType != ALF_FILTER_9_NO_FIX)
#else
  if (shape.filterType >= ALF_FILTER_13_EXT && shape.filterType != ALF_FILTER_9_EXT_DB)
#endif
  {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    int filterSetIdx = 2 + fixedFilterSetIdx;
#else
    int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
    Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased, *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;

    if (m_isFixedFilterPaddedPerCtu)
    {
      pImg0FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 0][posInCtu.x + padSize];
      pImg1FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 1][posInCtu.x + padSize];
      pImg2FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 1][posInCtu.x + padSize];
      pImg3FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 2][posInCtu.x + padSize];
      pImg4FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 2][posInCtu.x + padSize];
      pImg5FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 3][posInCtu.x + padSize];
      pImg6FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 3][posInCtu.x + padSize];
      pImg7FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 4][posInCtu.x + padSize];
      pImg8FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 4][posInCtu.x + padSize];
      pImg9FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 5][posInCtu.x + padSize];
      pImg10FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 5][posInCtu.x + padSize];
      pImg11FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 6][posInCtu.x + padSize];
      pImg12FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 6][posInCtu.x + padSize];
    }
    else
    {
      pImg0FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 0][pos.x + padSize];
      pImg1FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 1][pos.x + padSize];
      pImg2FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 1][pos.x + padSize];
      pImg3FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 2][pos.x + padSize];
      pImg4FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 2][pos.x + padSize];
      pImg5FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 3][pos.x + padSize];
      pImg6FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 3][pos.x + padSize];
      pImg7FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 4][pos.x + padSize];
      pImg8FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 4][pos.x + padSize];
      pImg9FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 5][pos.x + padSize];
      pImg10FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 5][pos.x + padSize];
      pImg11FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize + 6][pos.x + padSize];
      pImg12FixedBased = &fixedFitlerResults[filterSetIdx][pos.y + padSize - 6][pos.x + padSize];
    }
    if (transposeIdx == 1)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[22][b] += clipALF(clip[b], curr, pImg12FixedBased[+0], pImg11FixedBased[-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg10FixedBased[+0], pImg9FixedBased[-0]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg8FixedBased[+0], pImg7FixedBased[-0]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg6FixedBased[+0], pImg5FixedBased[-0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg4FixedBased[-1], pImg3FixedBased[+1]);
        ELocal[26][b] += clipALF(clip[b], curr, pImg4FixedBased[-0], pImg3FixedBased[+0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg4FixedBased[+1], pImg3FixedBased[-1]);
        ELocal[14][b] += clipALF(clip[b], curr, pImg2FixedBased[-2], pImg1FixedBased[+2]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]);
        ELocal[27][b] += clipALF(clip[b], curr, pImg2FixedBased[-0], pImg1FixedBased[+0]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg2FixedBased[+2], pImg1FixedBased[-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg0FixedBased[+6], pImg0FixedBased[-6]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg0FixedBased[+5], pImg0FixedBased[-5]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg0FixedBased[+4], pImg0FixedBased[-4]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg0FixedBased[+3], pImg0FixedBased[-3]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else if (transposeIdx == 2)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[10][b] += clipALF(clip[b], curr, pImg12FixedBased[+0], pImg11FixedBased[-0]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg10FixedBased[+0], pImg9FixedBased[-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg8FixedBased[+0], pImg7FixedBased[-0]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg6FixedBased[+0], pImg5FixedBased[-0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg4FixedBased[-1], pImg3FixedBased[+1]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg4FixedBased[-0], pImg3FixedBased[+0]);
        ELocal[14][b] += clipALF(clip[b], curr, pImg4FixedBased[+1], pImg3FixedBased[-1]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg2FixedBased[-2], pImg1FixedBased[+2]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg2FixedBased[-0], pImg1FixedBased[+0]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg2FixedBased[+2], pImg1FixedBased[-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg0FixedBased[+6], pImg0FixedBased[-6]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg0FixedBased[+5], pImg0FixedBased[-5]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg0FixedBased[+4], pImg0FixedBased[-4]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg0FixedBased[+3], pImg0FixedBased[-3]);
        ELocal[26][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[27][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else if (transposeIdx == 3)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[22][b] += clipALF(clip[b], curr, pImg12FixedBased[+0], pImg11FixedBased[-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg10FixedBased[+0], pImg9FixedBased[-0]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg8FixedBased[+0], pImg7FixedBased[-0]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg6FixedBased[+0], pImg5FixedBased[-0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg4FixedBased[-1], pImg3FixedBased[+1]);
        ELocal[26][b] += clipALF(clip[b], curr, pImg4FixedBased[-0], pImg3FixedBased[+0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg4FixedBased[+1], pImg3FixedBased[-1]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg2FixedBased[-2], pImg1FixedBased[+2]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]);
        ELocal[27][b] += clipALF(clip[b], curr, pImg2FixedBased[-0], pImg1FixedBased[+0]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]);
        ELocal[14][b] += clipALF(clip[b], curr, pImg2FixedBased[+2], pImg1FixedBased[-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImg0FixedBased[+6], pImg0FixedBased[-6]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg0FixedBased[+5], pImg0FixedBased[-5]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg0FixedBased[+4], pImg0FixedBased[-4]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg0FixedBased[+3], pImg0FixedBased[-3]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[10][b] += clipALF(clip[b], curr, pImg12FixedBased[+0], pImg11FixedBased[-0]);
        ELocal[11][b] += clipALF(clip[b], curr, pImg10FixedBased[+0], pImg9FixedBased[-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImg8FixedBased[+0], pImg7FixedBased[-0]);
        ELocal[13][b] += clipALF(clip[b], curr, pImg6FixedBased[+0], pImg5FixedBased[-0]);
        ELocal[14][b] += clipALF(clip[b], curr, pImg4FixedBased[-1], pImg3FixedBased[+1]);
        ELocal[15][b] += clipALF(clip[b], curr, pImg4FixedBased[-0], pImg3FixedBased[+0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImg4FixedBased[+1], pImg3FixedBased[-1]);
        ELocal[17][b] += clipALF(clip[b], curr, pImg2FixedBased[-2], pImg1FixedBased[+2]);
        ELocal[18][b] += clipALF(clip[b], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImg2FixedBased[-0], pImg1FixedBased[+0]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg2FixedBased[+2], pImg1FixedBased[-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg0FixedBased[+6], pImg0FixedBased[-6]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg0FixedBased[+5], pImg0FixedBased[-5]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg0FixedBased[+4], pImg0FixedBased[-4]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg0FixedBased[+3], pImg0FixedBased[-3]);
        ELocal[26][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[27][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
  }
  else
#endif
  if( shape.filterType >= ALF_FILTER_9_EXT && shape.filterType != ALF_FILTER_EXT
#if FIXFILTER_CFG
    && shape.filterType != ALF_FILTER_13_DB_RESI && shape.filterType != ALF_FILTER_13_DB_RESI_DIRECT && shape.filterType != ALF_FILTER_9_NO_FIX
#endif
    )
  {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    int filterSetIdx = 2 + fixedFilterSetIdx;
#else
    int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
    Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;

    if( m_isFixedFilterPaddedPerCtu )
    {
      pImg0FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 0][posInCtu.x + padSize];
      pImg1FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 1][posInCtu.x + padSize];
      pImg2FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 1][posInCtu.x + padSize];
      pImg3FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize + 2][posInCtu.x + padSize];
      pImg4FixedBased = &m_fixedFilterResultPerCtu[filterSetIdx][posInCtu.y + padSize - 2][posInCtu.x + padSize];
    }
    else
    {
      pImg0FixedBased = &fixedFitlerResults[filterSetIdx][posDst.y + padSize + 0][posDst.x + padSize];
      pImg1FixedBased = &fixedFitlerResults[filterSetIdx][posDst.y + padSize + 1][posDst.x + padSize];
      pImg2FixedBased = &fixedFitlerResults[filterSetIdx][posDst.y + padSize - 1][posDst.x + padSize];
      pImg3FixedBased = &fixedFitlerResults[filterSetIdx][posDst.y + padSize + 2][posDst.x + padSize];
      pImg4FixedBased = &fixedFitlerResults[filterSetIdx][posDst.y + padSize - 2][posDst.x + padSize];
    }

    if( transposeIdx == 1 )
    {
      for(int b = 0; b < numBins; b++)
      {
        ELocal[24][b] += clipALF(clip[b], curr, pImg3FixedBased[+0], pImg4FixedBased[+0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else if (transposeIdx == 2 )
    {
      for(int b = 0; b < numBins; b++)
      {
        ELocal[20][b] += clipALF(clip[b], curr, pImg3FixedBased[+0], pImg4FixedBased[+0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else if (transposeIdx == 3 )
    {
      for(int b = 0; b < numBins; b++)
      {
        ELocal[24][b] += clipALF(clip[b], curr, pImg3FixedBased[+0], pImg4FixedBased[+0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]);
        ELocal[20][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
    else
    {
      for(int b = 0; b < numBins; b++)
      {
        ELocal[20][b] += clipALF(clip[b], curr, pImg3FixedBased[+0], pImg4FixedBased[+0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]);
        ELocal[22][b] += clipALF(clip[b], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]);
        ELocal[24][b] += clipALF(clip[b], curr, pImg0FixedBased[+2], pImg0FixedBased[-2]);
        ELocal[25][b] += clipALF(clip[b], curr, pImg0FixedBased[+1], pImg0FixedBased[-1]);
      }
    }
  }
#endif
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  if( shape.filterType == ALF_FILTER_13_EXT )
  {
    for ( int b = 0; b < numBins; b++ )
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[30][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[31][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);
      ELocal[32][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[33][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[34][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[29][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[26][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[27][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[28][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);
      ELocal[30][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[31][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[32][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#endif
    }
    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
  else
#endif
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER && !JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  if( shape.filterType == ALF_FILTER_9_EXT || shape.filterType == ALF_FILTER_13_EXT )
#else
  if( shape.filterType == ALF_FILTER_9_EXT )
#endif
  {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    for ( int b = 0; b < numBins; b++ )
    {
      ELocal[26][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[27][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
    }
#else
    for( int i = filterPattern[k]; i < filterPattern[k] + EXT_LENGTH; i++ )
    {
      for ( int b = 0; b < numBins; b++ )
      {
        ELocal[i][b] += clipALF(clip[b], curr, fixedFitlerResults[(i - filterPattern[k]) * EXT_LENGTH + fixedFilterSetIdx][posDst.y][posDst.x]);
      }
    }
#endif
    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  else if (shape.filterType == ALF_FILTER_13_EXT_DB)
  {
    const Pel* pRecDbTmp0 = recBeforeDb;
    const Pel* pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel* pRecDbTmp2 = recBeforeDb - recBeforeDbStride;
    for (int b = 0; b < numBins; b++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[30][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[31][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[34][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[35][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[36][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[37][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[29][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[30][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[31][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[26][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[27][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[28][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[30][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[31][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[32][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[33][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][pos.y + padSize][pos.x + padSize]);
      ELocal[34][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[35][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#endif
    }
    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER && !JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  else if( shape.filterType == ALF_FILTER_9_EXT_DB || shape.filterType == ALF_FILTER_13_EXT_DB )
#else
  else if( shape.filterType == ALF_FILTER_9_EXT_DB )
#endif
  {
    const Pel* pRecDbTmp0 = recBeforeDb;
    const Pel* pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel* pRecDbTmp2 = recBeforeDb - recBeforeDbStride;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    for ( int b = 0; b < numBins; b++ )
    {
      ELocal[26][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[27][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[28][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[29][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[30][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
    }
#else
    for (int b = 0; b < numBins; b++)
    {
      ELocal[filterPattern[k]][b]     += clipALF(clip[b], curr, pRecDbTmp1[+0]) + clipALF(clip[b], curr, pRecDbTmp2[+0]);
      ELocal[filterPattern[k] + 1][b] += clipALF(clip[b], curr, pRecDbTmp0[-1]) + clipALF(clip[b], curr, pRecDbTmp0[+1]);
      ELocal[filterPattern[k] + 2][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
    }
    for (int i = filterPattern[k] + NUM_DB; i < filterPattern[k] + NUM_DB + EXT_LENGTH; i++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[i][b] += clipALF(clip[b], curr, fixedFitlerResults[(i - filterPattern[k] - NUM_DB) * EXT_LENGTH + fixedFilterSetIdx][posDst.y][posDst.x]);
      }
    }
#endif
    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#endif
#if FIXFILTER_CFG
  else if (shape.filterType == ALF_FILTER_13_DB_RESI_DIRECT)
  {
    const Pel *pRecDbTmp0 = recBeforeDb;
    const Pel *pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel *pRecDbTmp2 = recBeforeDb - recBeforeDbStride;

    const Pel *pResiTmp0 = resi;

    for (int b = 0; b < numBins; b++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[11][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[12][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[13][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[14][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[15][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[16][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[17][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[18][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);
      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[11][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[12][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[13][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[14][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[15][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[16][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[17][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[18][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#else
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#endif
    }
    for (int b = 0; b < numBins; b++)
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
  else if (shape.filterType == ALF_FILTER_13_DB_RESI)
  {
    const Pel *pRecDbTmp0 = recBeforeDb;
    const Pel *pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel *pRecDbTmp2 = recBeforeDb - recBeforeDbStride;

    const Pel *pResiTmp0 = resi;

    for (int b = 0; b < numBins; b++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[14][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[14][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#else
      ELocal[10][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[11][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[12][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[13][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#endif
    }
    for (int b = 0; b < numBins; b++)
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  else if (shape.filterType == ALF_FILTER_13_EXT_DB_RESI_DIRECT)
  {
    const Pel *pRecDbTmp0 = recBeforeDb;
    const Pel *pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel *pRecDbTmp2 = recBeforeDb - recBeforeDbStride;

    const Pel *pResiTmp0 = resi;

    for (int b = 0; b < numBins; b++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[30][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[31][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[34][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[35][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[36][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[37][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[38][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[29][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);
      ELocal[30][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[31][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[33][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[26][b] += clipALF(clip[b], curr, pImg3Gauss[0][+0], pImg4Gauss[0][-0]);
      ELocal[27][b] += clipALF(clip[b], curr, pImg1Gauss[0][+0], pImg2Gauss[0][-0]);
      ELocal[28][b] += clipALF(clip[b], curr, pImg0Gauss[0][+2], pImg0Gauss[0][-2]);
      ELocal[29][b] += clipALF(clip[b], curr, pImg0Gauss[0][+1], pImg0Gauss[0][-1]);

      ELocal[30][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[31][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[32][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[33][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[34][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[35][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[36][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#else
      ELocal[26][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[27][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[28][b] +=
        clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[29][b] +=
        clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[30][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[31][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#endif
    }
    for (int b = 0; b < numBins; b++)
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
  else if (shape.filterType == ALF_FILTER_13_EXT_DB_RESI)
  {
    const Pel *pRecDbTmp0 = recBeforeDb;
    const Pel *pRecDbTmp1 = recBeforeDb + recBeforeDbStride;
    const Pel *pRecDbTmp2 = recBeforeDb - recBeforeDbStride;

    const Pel *pResiTmp0 = resi;

    for (int b = 0; b < numBins; b++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      ELocal[28][b] += clipALF(clip[b],    0, pImg3Laplacian[0][+0], pImg4Laplacian[0][-0]);
      ELocal[29][b] += clipALF(clip[b],    0, pImg1Laplacian[0][+0], pImg2Laplacian[0][-0]);
      ELocal[30][b] += clipALF(clip[b],    0, pImg0Laplacian[0][+2], pImg0Laplacian[0][-2]);
      ELocal[31][b] += clipALF(clip[b],    0, pImg0Laplacian[0][+1], pImg0Laplacian[0][-1]);
      
      ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[34][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[35][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[36][b] += clipALF(clip[b],    0, fixedFitlerResiResults[0 + 1 - fixedFilterSetIdx][pos.y][pos.x]);
      ELocal[37][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[38][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[39][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
      ELocal[40][b] += clipALF(clip[b],    0, pImg0Laplacian[0][+0] );
#else
      ELocal[28][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[29][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[30][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[31][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[32][b] += clipALF(clip[b],    0, fixedFitlerResiResults[0 + 1 - fixedFilterSetIdx][pos.y][pos.x]);
      ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[34][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[35][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      ELocal[28][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[29][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[30][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[31][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[32][b] += clipALF(clip[b], 0, fixedFitlerResiResults[0 + 1 - fixedFilterSetIdx][pos.y][pos.x]);
      ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[34][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      ELocal[26][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[27][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[28][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[29][b] += clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[30][b] += clipALF(clip[b],    0, fixedFitlerResiResults[0 + 1 - fixedFilterSetIdx][pos.y][pos.x]);
      ELocal[31][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[32][b] += clipALF(clip[b],    0, pResiTmp0[+0]);
      ELocal[33][b] += clipALF(clip[b], curr, pImg0Gauss[0][+0] );
#else
      ELocal[26][b] += clipALF(clip[b], curr, pRecDbTmp2[+0], pRecDbTmp1[+0]);
      ELocal[27][b] += clipALF(clip[b], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]);

      ELocal[28][b] +=
        clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[29][b] +=
        clipALF(clip[b], curr, fixedFitlerResults[2 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      ELocal[30][b] += clipALF(clip[b], 0, fixedFitlerResiResults[0 + 1 - fixedFilterSetIdx][pos.y][pos.x]);
      ELocal[31][b] += clipALF(clip[b], curr, pRecDbTmp0[+0]);
      ELocal[32][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
#endif
    }
    for (int b = 0; b < numBins; b++)
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#endif 
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  else if( shape.filterType == ALF_FILTER_9 )
  {
    if( transposeIdx == 0 || transposeIdx == 2 )
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[20][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize-2][posDst.x + padSize], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize + 2][posDst.x + padSize]);
        ELocal[21][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize - 1][posDst.x + padSize], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize + 1][posDst.x + padSize]);
        ELocal[22][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize - 2], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize +2]);
        ELocal[23][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize-1], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize+1]);
        ELocal[24][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      }
    }
    else
    {
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[22][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize - 2][posDst.x + padSize], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize + 2][posDst.x + padSize]);
        ELocal[23][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize - 1][posDst.x + padSize], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize + 1][posDst.x + padSize]);
        ELocal[20][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize - 2], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize + 2]);
        ELocal[21][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize - 1], fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize + 1]);
        ELocal[24][b] += clipALF(clip[b], curr, fixedFitlerResults[0 + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
      }
    }
    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
#endif
  else if( shape.filterType == ALF_FILTER_EXT )
  {
    for( int i = 0; i < EXT_LENGTH; i++ )
    {
      for( int b = 0; b < numBins; b++ )
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        ELocal[i][b] += clipALF(clip[b], curr, fixedFitlerResults[i * EXT_LENGTH + fixedFilterSetIdx][posDst.y + padSize][posDst.x + padSize]);
#else
        ELocal[i][b] += clipALF(clip[b], curr, fixedFitlerResults[i * EXT_LENGTH + fixedFilterSetIdx][posDst.y][posDst.x]);
#endif
      }
    }

    for( int b = 0; b < numBins; b++ )
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
  else
#endif
  for( int b = 0; b < numBins; b++ )
  {
    ELocal[filterPattern[k]][b] += curr;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, bool val )
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    alfSlicePara.enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfSlicePara.enabledFlag[COMPONENT_Cb] = alfSlicePara.enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
{
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  for( int compId = compIDFirst; compId <= compIDLast; compId++ )
  {
    alfSlicePara.enabledFlag[compId] = false;
    for( int i = 0; i < m_numCTUsInPic; i++ )
    {
      if( ctuFlags[compId][i] )
      {
        alfSlicePara.enabledFlag[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memcpy( ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof( uint8_t ) * m_numCTUsInPic );
    memcpy( ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val )
{
  if( isLuma( channel ) )
  {
    memset( ctuFlags[COMPONENT_Y], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memset( ctuFlags[COMPONENT_Cb], val, sizeof( uint8_t ) * m_numCTUsInPic );
    memset( ctuFlags[COMPONENT_Cr], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

std::vector<int> EncAdaptiveLoopFilter::getAvaiApsIdsLuma(CodingStructure& cs, int &newApsId)
{
  APS** apss = cs.slice->getAlfAPSs();
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  int apsIdChecked = 0, curApsId = m_apsIdStart;
  if (curApsId < ALF_CTB_MAX_NUM_APS)
  {
    while (apsIdChecked < ALF_CTB_MAX_NUM_APS && !cs.slice->isIntra() && result.size() < ALF_CTB_MAX_NUM_APS && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
    {
      APS* curAPS = cs.slice->getAlfAPSs()[curApsId];

      if( curAPS && curAPS->getLayerId() == cs.slice->getPic()->layerId && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] )
      {
        result.push_back(curApsId);
      }
      apsIdChecked++;
      curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
    }
  }
  cs.slice->setTileGroupNumAps((int)result.size());
  cs.slice->setAlfAPSs(result);
  newApsId = m_apsIdStart - 1;
  if (newApsId < 0)
  {
    newApsId = ALF_CTB_MAX_NUM_APS - 1;
  }
  CHECK(newApsId >= ALF_CTB_MAX_NUM_APS, "Wrong APS index assignment in getAvaiApsIdsLuma");
  return result;
}

template<bool alfWSSD>
void  EncAdaptiveLoopFilter::initDistortion(
#if ALF_IMPROVEMENT
  CodingStructure& cs
#endif
)
{
  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
  {
    m_unFiltDistCompnent[comp] = 0.0;

    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
#if ALF_IMPROVEMENT
      for (int shapeIdx = 0; shapeIdx < m_filterShapes[toChannelType((ComponentID)comp)].size(); shapeIdx++)
      {
        if (m_filterTypeTest[toChannelType((ComponentID)comp)][m_filterShapes[toChannelType((ComponentID)comp)][shapeIdx].filterType])
        {
#if JVET_X0071_ALF_BAND_CLASSIFIER
          m_ctbDistortionUnfilter[comp][ctbIdx] = getUnfilteredDistortion(m_alfCovariance[comp][shapeIdx][ctbIdx][0][0], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
#else
          m_ctbDistortionUnfilter[comp][ctbIdx] = getUnfilteredDistortion(m_alfCovariance[comp][shapeIdx][ctbIdx][0], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
#endif
          m_unFiltDistCompnent[comp] += m_ctbDistortionUnfilter[comp][ctbIdx];
          break;
        }
      }
#else
      m_ctbDistortionUnfilter[comp][ctbIdx] = getUnfilteredDistortion(m_alfCovariance[comp][0][ctbIdx], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
      m_unFiltDistCompnent[comp] += m_ctbDistortionUnfilter[comp][ctbIdx];
#endif
    }
  }

#if ALF_IMPROVEMENT
#if FIXFILTER_CFG
  if( m_encCfg->getUseAlfFixedFilter() )
  {
#endif
#if ALF_SAO_TRUE_ORG
  PelUnitBuf orgYuv = cs.getTrueOrgBuf();
#else
  PelUnitBuf orgYuv = cs.getOrgBuf();
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
  int orgStride = orgYuv.get(COMPONENT_Y).stride;
  int ctbIdx = 0;
  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      const int width = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
      const int height = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;
      const Area blk(xPos, yPos, width, height);
      for (int classifierIdx = 0; classifierIdx < NUM_FIXED_FILTER_SETS; classifierIdx++)
      {
        for (int filterSetIdx = 0; filterSetIdx < 2; filterSetIdx++)
        {
          m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] = 0;
          Pel *org = orgYuv.get(COMPONENT_Y).bufAt(blk);
          int fixedFilter = classifierIdx * 2 + filterSetIdx;
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              if (alfWSSD)
              {
                double weight = m_lumaLevelToWeightPLUT[org[x]];
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += weight * (org[x] - m_fixFilterResult[COMPONENT_Y][fixedFilter][y + yPos + padSize][x + xPos + padSize]) *  (org[x] - m_fixFilterResult[COMPONENT_Y][fixedFilter][y + yPos + padSize][x + xPos + padSize]);
#else
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += weight * (org[x] - m_fixFilterResult[fixedFilter][y + yPos + padSize][x + xPos + padSize]) *  (org[x] - m_fixFilterResult[fixedFilter][y + yPos + padSize][x + xPos + padSize]);
#endif
#else
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += weight * (org[x] - m_fixFilterResult[fixedFilter][y + yPos][x + xPos]) *  (org[x] - m_fixFilterResult[fixedFilter][y + yPos][x + xPos]);
#endif
              }
              else
              {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += (org[x] - m_fixFilterResult[COMPONENT_Y][fixedFilter][y + yPos + padSize][x + xPos + padSize]) *  (org[x] - m_fixFilterResult[COMPONENT_Y][fixedFilter][y + yPos + padSize][x + xPos + padSize]);
#else
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += (org[x] - m_fixFilterResult[fixedFilter][y + yPos + padSize][x + xPos + padSize]) *  (org[x] - m_fixFilterResult[fixedFilter][y + yPos + padSize][x + xPos + padSize]);
#endif
#else
                m_ctbDistortionFixedFilter[classifierIdx][filterSetIdx][ctbIdx] += (org[x] - m_fixFilterResult[fixedFilter][y + yPos][x + xPos]) *  (org[x] - m_fixFilterResult[fixedFilter][y + yPos][x + xPos]);
#endif
              }
            }
            org += orgStride;
          }
        }
      }
      ctbIdx++;
    }
  }
#if FIXFILTER_CFG
  }
#endif
#else
  for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
  {
    for( int filterSetIdx = 0; filterSetIdx < NUM_FIXED_FILTER_SETS; filterSetIdx++ )    
    {
      m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx] = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
      for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
      {
        int filterIdx = m_classToFilterMapping[filterSetIdx][classIdx];
        m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs( m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS );
      }
    }
  }  
#endif
}

void  EncAdaptiveLoopFilter::initDistortionCcalf(int comp)
{
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    m_ctbDistortionUnfilter[comp][ctbIdx] = m_alfCovarianceCcAlf[0][ctbIdx].pixAcc;
  }
}

void  EncAdaptiveLoopFilter::getDistNewFilter( AlfParam& alfParam )
{
  reconstructCoeff( alfParam, CHANNEL_TYPE_LUMA, true, true );
#if ALF_IMPROVEMENT
  AlfFilterType filterTypeCtb = alfParam.filterType[CHANNEL_TYPE_LUMA];
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int numFixedFilterSet = numFixedFilters( filterTypeCtb );
#else
  int numFixedFilterSet = (filterTypeCtb == ALF_FILTER_EXT || filterTypeCtb == ALF_FILTER_9_EXT) ? 2 : 1;
#endif
  for( int altIdx = 0; altIdx < alfParam.numAlternativesLuma; altIdx++ )
  {
#if JVET_X0071_ALF_BAND_CLASSIFIER
    int classifierIdx = m_classifierFinal[altIdx];
    for( int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++ )
#else
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
#endif
    {
      for( int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; coeff++ )
      {
        m_filterTmp[coeff] = m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        m_clipTmp[coeff]   = m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      }
      for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
      {
        for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
        {
          if (classIdx == 0)
          {
            m_distCtbLumaNewFilt[altIdx][fixedFilterSetIdx][ctbIdx] = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
          }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          const char scaleIdx = m_scaleIdxFinal[altIdx][classIdx];
#else
          const char scaleIdx = 0;
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          m_distCtbLumaNewFilt[altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_coeffBitsFinal[altIdx], scaleIdx);
#else
          m_distCtbLumaNewFilt[altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs( m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
          m_distCtbLumaNewFilt[altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
        }
      }
    }
  }
#else
  for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
  {
    for( int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; coeff++ )
    {
      m_filterTmp[coeff] = m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      m_clipTmp[coeff] = m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
    }
    for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
    {
      if( classIdx == 0 )
      {
        m_distCtbLumaNewFilt[ctbIdx] = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
      }
      m_distCtbLumaNewFilt[ctbIdx] += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
    }
  }
#endif
}

void  EncAdaptiveLoopFilter::getDistApsFilter( CodingStructure& cs, std::vector<int> apsIds )
{
  APS** aps = cs.slice->getAlfAPSs();
  AlfParam alfParamTmp;
  APS* curAPS;

  for( int apsIdx = 0; apsIdx < apsIds.size(); apsIdx++ )
  {
    int apsId = apsIds[apsIdx];
    curAPS = aps[apsId];
    alfParamTmp = curAPS->getAlfAPSParam();
    reconstructCoeff( alfParamTmp, CHANNEL_TYPE_LUMA, true, true );
#if ALF_IMPROVEMENT
    m_numLumaAltAps[apsIdx] = alfParamTmp.numAlternativesLuma;
    AlfFilterType filterTypeCtb = alfParamTmp.filterType[CHANNEL_TYPE_LUMA];
    m_filterTypeApsLuma[apsIdx] = filterTypeCtb;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AA0095_ALF_LONGER_FILTER || JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    int numFixedFilterSet = numFixedFilters( filterTypeCtb );
#else
    int numFixedFilterSet = ( filterTypeCtb == ALF_FILTER_EXT || filterTypeCtb == ALF_FILTER_9_EXT ) ? 2 : 1;
#endif
    for( int altIdx = 0; altIdx < alfParamTmp.numAlternativesLuma; altIdx++ )
    {
#if JVET_X0071_ALF_BAND_CLASSIFIER
      int classifierIdx = m_classifierFinal[altIdx];
      m_classifierIdxApsLuma[apsIdx][altIdx] = classifierIdx;
      for( int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++ )
#else
      for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
#endif
      {
        for ( int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; coeff++ )
        {
          m_filterTmp[coeff] = m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
          m_clipTmp[coeff]   = m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        }
        for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
        {
          for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < numFixedFilterSet; fixedFilterSetIdx++ )
          {
            if( classIdx == 0 )
            {
              m_distCtbApsLuma[apsIdx][altIdx][fixedFilterSetIdx][ctbIdx] = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
            }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
            char scaleIdx = m_scaleIdxFinal[altIdx][classIdx];
#else
            const char scaleIdx = 0;
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
            m_distCtbApsLuma[apsIdx][altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_coeffBitsFinal[altIdx], scaleIdx);
#else
            m_distCtbApsLuma[apsIdx][altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
            m_distCtbApsLuma[apsIdx][altIdx][fixedFilterSetIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]][ctbIdx][fixedFilterSetIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeCtb]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
          }          
        }
      }
    }
#else
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      for( int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; coeff++ )
      {
        m_filterTmp[coeff] = m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        m_clipTmp[coeff] = m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      }
      for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
      {
        if( classIdx == 0 )
        {
          m_distCtbApsLuma[apsIdx][ctbIdx] = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
        }
        m_distCtbApsLuma[apsIdx][ctbIdx] += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS);
      }
    }
#endif
  }
}

void  EncAdaptiveLoopFilter::alfEncoderCtb(CodingStructure& cs, AlfParam& alfParamNewFilters
#if ENABLE_QPA
  , const double lambdaChromaWeight
#endif
)
{
  TempCtx        ctxStart(m_ctxCache, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx        ctxBest(m_ctxCache);
  TempCtx        ctxTempStart(m_ctxCache);
  TempCtx        ctxTempBest(m_ctxCache);
  TempCtx        ctxTempAltStart(m_ctxCache);
  TempCtx        ctxTempAltBest(m_ctxCache);
  AlfParam  alfParamNewFiltersBest = alfParamNewFilters;
  bool     hasNewFilters[2] = { alfParamNewFilters.enabledFlag[COMPONENT_Y] , alfParamNewFilters.enabledFlag[COMPONENT_Cb] || alfParamNewFilters.enabledFlag[COMPONENT_Cr] };
  m_alfParamTemp = alfParamNewFilters;
  APS**          apss = cs.slice->getAlfAPSs();
  short*     alfCtbFilterSetIndex = cs.picture->getAlfCtbFilterIndex();

  //luma
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 1);
#if ALF_IMPROVEMENT
  std::fill_n(m_ctuAlternative[COMPONENT_Y], m_numCTUsInPic, 0);
  if (m_alfParamTemp.numAlternativesLuma < 1)
  {
    m_alfParamTemp.numAlternativesLuma = 1;
  }
#endif
  double costOff = m_unFiltDistCompnent[COMPONENT_Y];
  setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 0);
  int newApsId;
  std::vector<int> apsIds = getAvaiApsIdsLuma(cs, newApsId);
  std::vector<int> bestApsIds;
  double costMin = MAX_DOUBLE;

  getDistApsFilter( cs, apsIds );
  int numLoops = hasNewFilters[CHANNEL_TYPE_LUMA] ? 2 : 1;
#if ALF_IMPROVEMENT
  AlfFilterType filterTypeNewFilter = ALF_NUM_OF_FILTER_TYPES;
  int numAlfLumaNew = alfParamNewFilters.numAlternativesLuma;
  int bestFixedFilterSetIdx = -1;
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  bool filterWasRederived = false;
  for (int withSA = 0; withSA < 2; withSA++)
  {
    if (withSA && !(alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] && filterWasRederived))
    {
      break;
    }
#endif
#if ALF_IMPROVEMENT
#if JVET_AK0123_ALF_COEFF_RESTRICTION
#if FIXFILTER_CFG
    int fixedFilterSetIdxFrom = 0, fixedFilterSetIdxTo = m_encCfg->getUseAlfFixedFilter() ? 2 : 1;
#else
    int fixedFilterSetIdxFrom = 0, fixedFilterSetIdxTo = 2;
#endif
    if (withSA)
    {
      fixedFilterSetIdxFrom = bestFixedFilterSetIdx, fixedFilterSetIdxTo = bestFixedFilterSetIdx + 1;
    }
    for (int fixedFilterSetIdx = fixedFilterSetIdxFrom; fixedFilterSetIdx < fixedFilterSetIdxTo; fixedFilterSetIdx++)
#else
#if FIXFILTER_CFG
    for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < (m_encCfg->getUseAlfFixedFilter() ? 2 : 1); fixedFilterSetIdx++)
#else
    for (int fixedFilterSetIdx = 0; fixedFilterSetIdx < 2; fixedFilterSetIdx++)
#endif
#endif
    {
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
      int useNewFilterFrom = 0, useNewFilterTo = numLoops;
      if (withSA)
      {
        useNewFilterFrom = 1, useNewFilterTo = 2;
      }
      for (int useNewFilter = useNewFilterFrom; useNewFilter < useNewFilterTo; useNewFilter++)
#else
      for (int useNewFilter = 0; useNewFilter < numLoops; useNewFilter++)
#endif
      {
        int bitsNewFilter = 0;
        if (useNewFilter == 1)
        {
          if (!hasNewFilters[CHANNEL_TYPE_LUMA])
          {
            continue;
          }
          else
          {
            bitsNewFilter = m_bitsNewFilter[CHANNEL_TYPE_LUMA];
            getDistNewFilter(alfParamNewFilters);
#if ALF_IMPROVEMENT
            filterTypeNewFilter = alfParamNewFilters.filterType[CHANNEL_TYPE_LUMA];
            numAlfLumaNew = alfParamNewFilters.numAlternativesLuma;
#endif
          }
        }
        int numIter = useNewFilter ? 2 : 1;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
        std::unordered_set<int> triedBlocksUsingNewFilter;
        int numTemporalApsFrom = 0, numTemporalApsTo = (int)apsIds.size();
        if (withSA)
        {
          numTemporalApsFrom = (int)bestApsIds.size() - 1, numTemporalApsTo = (int)bestApsIds.size() - 1;
        }
        for (int numTemporalAps = numTemporalApsFrom; numTemporalAps <= numTemporalApsTo; numTemporalAps++)
#else
        for (int numTemporalAps = 0; numTemporalAps <= apsIds.size(); numTemporalAps++)
#endif
        {
          if (numTemporalAps + useNewFilter >= ALF_CTB_MAX_NUM_APS)
          {
            continue;
          }
          cs.slice->setTileGroupNumAps(numTemporalAps + useNewFilter);
          int numFilterSet = NUM_FIXED_FILTER_SETS + numTemporalAps + useNewFilter;
          if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back()) //last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
          {
            continue;
          }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          int iterFrom = 0, iterTo = numIter;
          if (withSA)
          {
            iterFrom = 1, iterTo = 2;
          }
          for (int iter = iterFrom; iter < iterTo; iter++)
#else
          for (int iter = 0; iter < numIter; iter++)
#endif
          {
            m_alfParamTemp = alfParamNewFilters;
            m_alfParamTemp.enabledFlag[CHANNEL_TYPE_LUMA] = true;
            double curCost = 3 * m_lambda[CHANNEL_TYPE_LUMA];
            if (iter > 0)  //re-derive new filter-set
            {
              double dDistOrgNewFilter = 0;
              int blocksUsingNewFilter = 0;
              for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
              {
                if (m_ctuEnableFlag[COMPONENT_Y][ctbIdx] && alfCtbFilterSetIndex[ctbIdx] != NUM_FIXED_FILTER_SETS)
                {
                  m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
                }
                else if (m_ctuEnableFlag[COMPONENT_Y][ctbIdx] && alfCtbFilterSetIndex[ctbIdx] == NUM_FIXED_FILTER_SETS)
                {
                  blocksUsingNewFilter++;
#if ALF_IMPROVEMENT
                  int altIdx = m_ctuAlternative[COMPONENT_Y][ctbIdx];
#endif
                  dDistOrgNewFilter += m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];
#if JVET_X0071_ALF_BAND_CLASSIFIER
                  int classifierIdx = m_classifierFinal[altIdx];
                  for (int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++)
#else
                  for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
#endif
                  {
#if ALF_IMPROVEMENT
                    short* pCoeff = m_coeffFinal[altIdx];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
                    Pel* pClipp = m_clippFinal[altIdx];
#else
                    short* pClipp = m_clippFinal[altIdx];
#endif
#else
                    short* pCoeff = m_coeffFinal;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
                    Pel* pClipp = m_clippFinal;
#else
                    short* pClipp = m_clippFinal;
#endif
#endif
                    for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                    {
                      m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                      m_clipTmp[i] = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                    }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                    const char scaleIdx = m_scaleIdxFinal[altIdx][classIdx];
#else
                    const char scaleIdx = 0;
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
                    dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]].numCoeff, m_coeffBitsFinal[altIdx], scaleIdx);
#else
                    dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]][ctbIdx][fixedFilterSetIdx][classifierIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
                    dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]][ctbIdx][fixedFilterSetIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_LUMA][m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][filterTypeNewFilter]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
                    dDistOrgNewFilter += m_alfCovariance[COMPONENT_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, m_NUM_BITS, scaleIdx);
#endif
                  }
                }
              }
              if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                && triedBlocksUsingNewFilter.count(blocksUsingNewFilter) == 0 /* re-derived filter this number of blocks was bad */
#endif
                )
              {
#if ALF_IMPROVEMENT
                int bitsNewFilterTempLuma = 0, bitsTemp = 0;
                double err = 0.0;
                double costNewFilter = MAX_DOUBLE;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                int shapeIdxFrom = 0, shapeIdxTo = (int)m_filterShapes[CHANNEL_TYPE_LUMA].size();
                if (withSA)
                {
                  for (int shapeIdx = 0; shapeIdx < m_filterShapes[CHANNEL_TYPE_LUMA].size(); shapeIdx++)
                  {
                    if (m_filterShapes[CHANNEL_TYPE_LUMA][shapeIdx].filterType == alfParamNewFiltersBest.filterType[CHANNEL_TYPE_LUMA])
                    {
                      shapeIdxFrom = shapeIdx, shapeIdxTo = shapeIdx + 1;
                      break;
                    }
                  }
                }
                for (int shapeIdx = shapeIdxFrom; shapeIdx < shapeIdxTo; shapeIdx++)
#else
                for (int shapeIdx = 0; shapeIdx < m_filterShapes[CHANNEL_TYPE_LUMA].size(); shapeIdx++)
#endif
                {
                  if( m_filterTypeTest[CHANNEL_TYPE_LUMA][m_filterShapes[CHANNEL_TYPE_LUMA][shapeIdx].filterType] == false )
                  {
                    continue;
                  }
                  m_alfParamTemp.filterType[CHANNEL_TYPE_LUMA] = m_filterShapes[CHANNEL_TYPE_LUMA][shapeIdx].filterType;
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                  err = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, shapeIdx, bitsTemp, fixedFilterSetIdx, withSA, true);
#else
                  err = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, shapeIdx, bitsTemp, fixedFilterSetIdx, false, true);
#endif
                  if (err < costNewFilter)
                  {
                    costNewFilter = err;
                    bitsNewFilterTempLuma = bitsTemp;
                    m_alfParamTempNL = m_alfParamTemp;
                  }
                }
                err = costNewFilter;
                m_alfParamTemp = m_alfParamTempNL;
#else
                int bitNL[2] = { 0, 0 };
                double errNL[2] = { 0.0, 0.0 };
                m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 1;
                if (m_encCfg->getUseNonLinearAlfLuma())
                {
                  errNL[1] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[1], true, true);
                  m_alfParamTempNL = m_alfParamTemp;
                }
                else
                {
                  errNL[1] = MAX_DOUBLE;
                }
                m_alfParamTemp.nonLinearFlag[CHANNEL_TYPE_LUMA] = 0;
                errNL[0] = getFilterCoeffAndCost(cs, 0, CHANNEL_TYPE_LUMA, true, 0, bitNL[0], true, true);

                int bitsNewFilterTempLuma = bitNL[0];
                double err = errNL[0];
                if (errNL[1] < errNL[0])
                {
                  err = errNL[1];
                  bitsNewFilterTempLuma = bitNL[1];
                  m_alfParamTemp = m_alfParamTempNL;
                }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                triedBlocksUsingNewFilter.insert(blocksUsingNewFilter);
#endif
                if (dDistOrgNewFilter + m_lambda[CHANNEL_TYPE_LUMA] * m_bitsNewFilter[CHANNEL_TYPE_LUMA] < err) //re-derived filter is not good, skip
                {
                  continue;
                }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                filterWasRederived = true;
#endif

                getDistNewFilter(m_alfParamTemp);
                bitsNewFilter = bitsNewFilterTempLuma;
#if ALF_IMPROVEMENT
                filterTypeNewFilter = m_alfParamTemp.filterType[CHANNEL_TYPE_LUMA];
                numAlfLumaNew = m_alfParamTemp.numAlternativesLuma;
#endif
              }
              else //no blocks using new filter, skip
              {
                continue;
              }
            }

            m_CABACEstimator->getCtx() = AlfCtx(ctxStart);

            for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
            {
              double distUnfilterCtb = m_ctbDistortionUnfilter[COMPONENT_Y][ctbIdx];

              //ctb on
              m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 1;
              double         costOn = MAX_DOUBLE;
              ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
              int iBestFilterSetIdx = 0;
#if ALF_IMPROVEMENT
              int iBestAltIdx = 0;
#endif

              for (int filterSetIdx = 0; filterSetIdx < numFilterSet; filterSetIdx++)  // to select best APS index / fixed filter index
              {
#if ALF_IMPROVEMENT
#if FIXFILTER_CFG
                if(m_encCfg->getUseAlfFixedFilter() == false && filterSetIdx < NUM_FIXED_FILTER_SETS)
                {
                  continue;
                }
#endif
                int iterAltLuma = 1;
                if (filterSetIdx >= NUM_FIXED_FILTER_SETS)
                {
                  if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                  {
                    iterAltLuma = numAlfLumaNew;
                  }
                  else if (useNewFilter)
                  {
                    iterAltLuma = m_numLumaAltAps[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
                  }
                  else
                  {
                    iterAltLuma = m_numLumaAltAps[filterSetIdx - NUM_FIXED_FILTER_SETS];
                  }
                }
                for (int altIdx = 0; altIdx < iterAltLuma; altIdx++)     // to select best filter set index inside one APS
                {
#endif

                  //rate
                  m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
                  m_CABACEstimator->resetBits();
                  m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfParamTemp);
                  alfCtbFilterSetIndex[ctbIdx] = filterSetIdx;
                  m_CABACEstimator->codeAlfCtuFilterIndex( cs, ctbIdx, m_alfParamTemp.enabledFlag[COMPONENT_Y] );
#if ALF_IMPROVEMENT
                  m_ctuAlternative[COMPONENT_Y][ctbIdx] = altIdx;
                  if (filterSetIdx >= NUM_FIXED_FILTER_SETS)
                  {
                    m_CABACEstimator->codeAlfCtuAlternative(cs, ctbIdx, COMPONENT_Y, &m_alfParamTemp, iterAltLuma);
                  }
#endif
                  double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

                  //distortion
                  double dist;
#if ALF_IMPROVEMENT
                  if (filterSetIdx < NUM_FIXED_FILTER_SETS)
                  {
                    dist = m_ctbDistortionFixedFilter[filterSetIdx][fixedFilterSetIdx][ctbIdx];
                  }
                  else if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                  {
                    dist = m_distCtbLumaNewFilt[altIdx][fixedFilterSetIdx][ctbIdx];
                  }
                  else
                  {
                    dist = m_distCtbApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS - useNewFilter][altIdx][fixedFilterSetIdx][ctbIdx];
                  }
#else
                  if (filterSetIdx < NUM_FIXED_FILTER_SETS)
                  {
                    dist = m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx];
                  }
                  else if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                  {
                    dist = m_distCtbLumaNewFilt[ctbIdx];
                  }
                  else
                  {
                    dist = m_distCtbApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS - useNewFilter][ctbIdx];
                  }
#endif

                  //cost
                  double costOnTmp = dist + m_lambda[COMPONENT_Y] * rateOn;
                  if (costOnTmp < costOn)
                  {
                    ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
                    costOn = costOnTmp;
                    iBestFilterSetIdx = filterSetIdx;
#if ALF_IMPROVEMENT
                    iBestAltIdx = altIdx;
#endif
                  }
#if ALF_IMPROVEMENT
                }
#endif
              }

              //ctb off
              m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
              //rate
              m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
              m_CABACEstimator->resetBits();
              m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMPONENT_Y, &m_alfParamTemp);
              //cost
              double costOff = distUnfilterCtb + m_lambda[COMPONENT_Y] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

              if (costOn < costOff)
              {
                m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
                m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 1;
                alfCtbFilterSetIndex[ctbIdx] = iBestFilterSetIdx;
#if ALF_IMPROVEMENT
                m_ctuAlternative[COMPONENT_Y][ctbIdx] = iBestAltIdx;
#endif
                curCost += costOn;
              }
              else
              {
                m_ctuEnableFlag[COMPONENT_Y][ctbIdx] = 0;
                curCost += costOff;
              }
            } //for(ctbIdx)

#if ALF_IMPROVEMENT
            int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTER_SETS) + 1;
#else
            int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTER_SETS);
#endif
            curCost += tmpBits * m_lambda[COMPONENT_Y];
            if (curCost < costMin)
            {
              costMin = curCost;
              bestApsIds.resize(numFilterSet - NUM_FIXED_FILTER_SETS);
              for (int i = 0; i < bestApsIds.size(); i++)
              {
                if (i == 0 && useNewFilter)
                {
                  bestApsIds[i] = newApsId;
                }
                else
                {
                  bestApsIds[i] = apsIds[i - useNewFilter];
                }
              }
#if ALF_IMPROVEMENT
              bestFixedFilterSetIdx = fixedFilterSetIdx;
#endif
              alfParamNewFiltersBest = m_alfParamTemp;
              ctxBest = AlfCtx(m_CABACEstimator->getCtx());
              copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_LUMA);
              for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
              {
                m_alfCtbFilterSetIndexTmp[ctuIdx] = alfCtbFilterSetIndex[ctuIdx];
#if ALF_IMPROVEMENT
                m_ctuAlternativeTmp[COMPONENT_Y][ctuIdx] = m_ctuAlternative[COMPONENT_Y][ctuIdx];
#endif
              }
              alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] = useNewFilter;
            }
          }//for (int iter = 0; iter < numIter; iter++)
        }// for (int numTemporalAps = 0; numTemporalAps < apsIds.size(); numTemporalAps++)
      }//for (int useNewFilter = 0; useNewFilter <= 1; useNewFilter++)
#if ALF_IMPROVEMENT
    }//for(fixedFilterSetIdx)
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  }//for(withSA)
#endif

  cs.slice->setTileGroupCcAlfCbApsId(newApsId);
  cs.slice->setTileGroupCcAlfCrApsId(newApsId);
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  m_isLumaSignalNewAps = false;
#endif

  if (costOff <= costMin)
  {
    cs.slice->resetTileGroupAlfEnabledFlag();
    cs.slice->setTileGroupNumAps(0);
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_LUMA, 0);
    setCtuEnableFlag(m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 0);
    return;
  }
  else
  {
#if ALF_IMPROVEMENT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
    cs.slice->setTileGroupAlfFixedFilterSetIdx( COMPONENT_Y, bestFixedFilterSetIdx );
#else
    cs.slice->setTileGroupAlfFixedFilterSetIdx( bestFixedFilterSetIdx );
#endif
#endif
    cs.slice->setTileGroupAlfEnabledFlag(COMPONENT_Y, true);
    cs.slice->setTileGroupNumAps((int)bestApsIds.size());
    cs.slice->setAlfAPSs(bestApsIds);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_LUMA);
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      alfCtbFilterSetIndex[ctuIdx] = m_alfCtbFilterSetIndexTmp[ctuIdx];
#if ALF_IMPROVEMENT
      m_ctuAlternative[COMPONENT_Y][ctuIdx] = m_ctuAlternativeTmp[COMPONENT_Y][ctuIdx];
#endif
    }
    if (alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA])
    {
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
      m_isLumaSignalNewAps = true;
#endif
      APS* newAPS = m_apsMap->getPS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->setAPSId(newApsId);
        newAPS->setAPSType(ALF_APS);
      }
      newAPS->setAlfAPSParam(alfParamNewFiltersBest);
      newAPS->setTemporalId( cs.slice->getTLayer() );
      newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = false;
      m_apsMap->setChangedFlag((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsIdStart = newApsId;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      g_pictureControlInfo[newApsId].reset();
#endif
    }

    std::vector<int> apsIds = cs.slice->getTileGroupApsIdLuma();
    for (int i = 0; i < (int)cs.slice->getTileGroupNumAps(); i++)
    {
      apss[apsIds[i]] = m_apsMap->getPS((apsIds[i] << NUM_APS_TYPE_LEN) + ALF_APS);
    }
  }

  //chroma
  if( isChromaEnabled( cs.pcv->chrFormat ) )
  {
    m_alfParamTemp = alfParamNewFiltersBest;
    if( m_alfParamTemp.numAlternativesChroma < 1 )
    {
      m_alfParamTemp.numAlternativesChroma = 1;
    }
    setCtuAlternativeChroma( m_ctuAlternative, 0 );
    setCtuEnableFlag( m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 1 );
    costOff = m_unFiltDistCompnent[COMPONENT_Cb] + m_unFiltDistCompnent[COMPONENT_Cr];
    costMin = MAX_DOUBLE;
    m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
    ctxStart = AlfCtx( m_CABACEstimator->getCtx() );
    int newApsIdChroma = -1;
    if( alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] && ( alfParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfParamNewFiltersBest.enabledFlag[COMPONENT_Cr] ) )
    {
      newApsIdChroma = newApsId;
    }
    else if( alfParamNewFiltersBest.enabledFlag[COMPONENT_Cb] || alfParamNewFiltersBest.enabledFlag[COMPONENT_Cr] )
    {
      int curId = m_apsIdStart;
      while( newApsIdChroma < 0 )
      {
        curId--;
        if( curId < 0 )
        {
          curId = ALF_CTB_MAX_NUM_APS - 1;
        }
        if( std::find( bestApsIds.begin(), bestApsIds.end(), curId ) == bestApsIds.end() )
        {
          newApsIdChroma = curId;
        }
      }
    }

    for( int curApsId = 0; curApsId < ALF_CTB_MAX_NUM_APS; curApsId++ )
    {
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      bool bestReuseFlag[MAX_NUM_COMPONENT] = { false };
#endif
      if( ( cs.slice->getPendingRasInit() || cs.slice->isIDRorBLA() || cs.slice->isIntra() )
          && curApsId != newApsIdChroma )
      {
        continue;
      }
      APS *curAPS = m_apsMap->getPS( ( curApsId << NUM_APS_TYPE_LEN ) + ALF_APS );

      if( curAPS && curAPS->getLayerId() != cs.slice->getPic()->layerId )
      {
        continue;
      }
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if FIXFILTER_CFG
      for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < (m_encCfg->getUseAlfFixedFilter()  ? 2 : 1); fixedFilterSetIdx++)
#else
      for( int fixedFilterSetIdx = 0; fixedFilterSetIdx < 2; fixedFilterSetIdx++ )
#endif
      {
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
        bool checkReuse = !cs.slice->isIntra();
#endif        
        double curCost = m_lambda[CHANNEL_TYPE_CHROMA] * 3;
        if( curApsId == newApsIdChroma )
        {
          m_alfParamTemp = alfParamNewFilters;
          curCost += m_lambda[CHANNEL_TYPE_CHROMA] * m_bitsNewFilter[CHANNEL_TYPE_CHROMA];
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          checkReuse = false;
#endif
        }
        else if( curAPS && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] )
        {
          m_alfParamTemp = curAPS->getAlfAPSParam();
        }
        else
        {
          continue;
        }
        reconstructCoeff( m_alfParamTemp, CHANNEL_TYPE_CHROMA, true, true );
#if ALF_IMPROVEMENT
        AlfFilterType filterTypeChroma = m_alfParamTemp.filterType[CHANNEL_TYPE_CHROMA];
#endif
        m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
        for( int compId = 1; compId < MAX_NUM_COMPONENT; compId++ )
        {
          m_alfParamTemp.enabledFlag[compId] = true;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          bool checkReuseComponent = checkReuse && g_pictureControlInfo[curApsId].alfTemporalLayer[compId] <= cs.slice->getTLayer();
          double costReuse[2] = { MAX_DOUBLE, MAX_DOUBLE };
          for( int reuse = checkReuseComponent ? 1 : 0; reuse >= 0; reuse-- )
          {
            costReuse[reuse] = 0;
#endif
          for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
          {
            double distUnfilterCtu = m_ctbDistortionUnfilter[compId][ctbIdx];
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
            if (reuse)
            {
              costReuse[reuse] += distUnfilterCtu;
              if (g_pictureControlInfo[curApsId].bufAlfFlag[compId][ctbIdx])
              {
                int altIdx = g_pictureControlInfo[curApsId].bufAlfFilter[compId][ctbIdx];
                for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
                {
                  m_filterTmp[i] = m_chromaCoeffFinal[altIdx][i];
                  m_clipTmp[i] = m_chromaClippFinal[altIdx][i];
                }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                const char scaleIdx = m_chromaScaleIdxFinal[altIdx][0];
#else
                const char scaleIdx = 0;
#endif
                costReuse[reuse] += m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS_CHROMA, scaleIdx);
              }
              continue;
            }
#endif
            //cost on
            m_ctuEnableFlag[compId][ctbIdx] = 1;
            ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
            //rate
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
            m_CABACEstimator->resetBits();
            //ctb flag
            m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctbIdx, compId, &m_alfParamTemp );
            double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#if ENABLE_QPA
            const double ctuLambda =
              lambdaChromaWeight > 0.0 ? cs.picture->m_uEnerHpCtu[ctbIdx] / lambdaChromaWeight : m_lambda[compId];
#else
            const double ctuLambda = m_lambda[compId];
#endif
            double dist = MAX_DOUBLE;
            int    numAlts = m_alfParamTemp.numAlternativesChroma;
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
            double bestAltRate = 0;
            double bestAltCost = MAX_DOUBLE;
            int    bestAltIdx = -1;
            ctxTempAltStart = AlfCtx( ctxTempBest );
            for( int altIdx = 0; altIdx < numAlts; ++altIdx )
            {
              if( altIdx )
              {
                m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
              }
              m_CABACEstimator->resetBits();
              m_ctuAlternative[compId][ctbIdx] = altIdx;
              m_CABACEstimator->codeAlfCtuAlternative( cs, ctbIdx, compId, &m_alfParamTemp );
              double altRate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
              double r_altCost = ctuLambda * altRate;

              // distortion
              for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
              {
                m_filterTmp[i] = m_chromaCoeffFinal[altIdx][i];
                m_clipTmp[i] = m_chromaClippFinal[altIdx][i];
              }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
              const char scaleIdx = m_chromaScaleIdxFinal[altIdx][0];
#else
              const char scaleIdx = 0;
#endif
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
              double altDist = m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS_CHROMA, scaleIdx);
#else
              double altDist = m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
              double altDist = m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][0][0][0].calcErrorForCoeffs( m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS_CHROMA, scaleIdx);
#else
              double altDist = m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][0][0][0].calcErrorForCoeffs( m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#endif
#else
              double altDist = m_alfCovariance[compId][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]][ctbIdx][0][0].calcErrorForCoeffs( m_clipTmp, m_filterTmp, m_filterShapes[CHANNEL_TYPE_CHROMA][m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][filterTypeChroma]].numCoeff, m_NUM_BITS, scaleIdx);
#endif
#else            
              double altDist = m_alfCovariance[compId][0][ctbIdx][0].calcErrorForCoeffs( m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, m_NUM_BITS, scaleIdx);
#endif
              double altCost = altDist + r_altCost;
              if( altCost < bestAltCost )
              {
                bestAltCost = altCost;
                bestAltIdx = altIdx;
                bestAltRate = altRate;
                ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
                dist = altDist;
              }
            }
            m_ctuAlternative[compId][ctbIdx] = bestAltIdx;
            rateOn += bestAltRate;
            dist += distUnfilterCtu;
            // cost            
            double costOn = dist + ctuLambda * rateOn;

            // cost off
            m_ctuEnableFlag[compId][ctbIdx] = 0;
            // rate
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
            m_CABACEstimator->resetBits();
            m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctbIdx, compId, &m_alfParamTemp );
            // cost
            double costOff = distUnfilterCtu + m_lambda[compId] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
            if( costOn < costOff )
            {
              m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
              m_ctuEnableFlag[compId][ctbIdx] = 1;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
              costReuse[reuse] += costOn;
#else
              curCost += costOn;
#endif
            }
            else
            {
              m_ctuEnableFlag[compId][ctbIdx] = 0;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
              costReuse[reuse] += costOff;
#else
              curCost += costOff;
#endif
            }
          }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          } //for( int reuse)
          int bestReuse = 0;
          if( (costReuse[1] < costReuse[0]) && ( (cs.picture->poc % 8) != 0 || m_reuseCondition) )
          {
            bestReuse = 1;
            for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
            {
              m_ctuEnableFlag[compId][ctbIdx] = g_pictureControlInfo[curApsId].bufAlfFlag[compId][ctbIdx];
              m_ctuAlternative[compId][ctbIdx] = g_pictureControlInfo[curApsId].bufAlfFilter[compId][ctbIdx];
            }
          }
          curCost += costReuse[bestReuse];
          bestReuseFlag[compId] = bestReuse;
#endif
        }
        // chroma idc
        setEnableFlag( m_alfParamTemp, CHANNEL_TYPE_CHROMA, m_ctuEnableFlag );

        if( curCost < costMin )
        {
          costMin = curCost;
          cs.slice->setTileGroupApsIdChroma( curApsId );
          cs.slice->setTileGroupAlfEnabledFlag( COMPONENT_Cb, m_alfParamTemp.enabledFlag[COMPONENT_Cb] );
          cs.slice->setTileGroupAlfEnabledFlag( COMPONENT_Cr, m_alfParamTemp.enabledFlag[COMPONENT_Cr] );
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          cs.slice->setTileGroupAlfFixedFilterSetIdx( COMPONENT_Cb, fixedFilterSetIdx );
          cs.slice->setTileGroupAlfFixedFilterSetIdx( COMPONENT_Cr, fixedFilterSetIdx );
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          cs.slice->setTileGroupAlfReuseFlag(COMPONENT_Cb, bestReuseFlag[COMPONENT_Cb]);
          cs.slice->setTileGroupAlfReuseFlag(COMPONENT_Cr, bestReuseFlag[COMPONENT_Cr]);
#endif
          copyCtuEnableFlag( m_ctuEnableFlagTmp, m_ctuEnableFlag, CHANNEL_TYPE_CHROMA );
          copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
        }
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
      }
#endif
    }

    if( newApsIdChroma >= 0 )
    {
      cs.slice->setTileGroupCcAlfCbApsId( newApsIdChroma );
      cs.slice->setTileGroupCcAlfCrApsId( newApsIdChroma );
    }
    if( costOff < costMin )
    {
      cs.slice->setTileGroupAlfEnabledFlag( COMPONENT_Cb, false );
      cs.slice->setTileGroupAlfEnabledFlag( COMPONENT_Cr, false );
      setCtuEnableFlag( m_ctuEnableFlag, CHANNEL_TYPE_CHROMA, 0 );
    }
    else
    {
      copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, CHANNEL_TYPE_CHROMA );
      copyCtuAlternativeChroma( m_ctuAlternative, m_ctuAlternativeTmp );
      if( cs.slice->getTileGroupApsIdChroma() == newApsIdChroma )  //new filter
      {
        APS* newAPS = m_apsMap->getPS( ( newApsIdChroma << NUM_APS_TYPE_LEN ) + ALF_APS );
        if( newAPS == NULL )
        {
          newAPS = m_apsMap->allocatePS( ( newApsIdChroma << NUM_APS_TYPE_LEN ) + ALF_APS );
          newAPS->setAPSType( ALF_APS );
          newAPS->setAPSId( newApsIdChroma );
          newAPS->getAlfAPSParam().reset();
        }
        newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_CHROMA] = true;
        if( !alfParamNewFiltersBest.newFilterFlag[CHANNEL_TYPE_LUMA] )
        {
          newAPS->getAlfAPSParam().newFilterFlag[CHANNEL_TYPE_LUMA] = false;
        }
        newAPS->getAlfAPSParam().numAlternativesChroma = alfParamNewFilters.numAlternativesChroma;
#if ALF_IMPROVEMENT
        for( int altIdx = 0; altIdx < MAX_NUM_ALF_ALTERNATIVES_CHROMA; ++altIdx )
        {
          newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx] = alfParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA][altIdx];
        }
        newAPS->getAlfAPSParam().filterType[CHANNEL_TYPE_CHROMA] = alfParamNewFilters.filterType[CHANNEL_TYPE_CHROMA];
#else
        newAPS->getAlfAPSParam().nonLinearFlag[CHANNEL_TYPE_CHROMA] = alfParamNewFilters.nonLinearFlag[CHANNEL_TYPE_CHROMA];
#endif
        newAPS->setTemporalId( cs.slice->getTLayer() );
        for( int altIdx = 0; altIdx < MAX_NUM_ALF_ALTERNATIVES_CHROMA; ++altIdx )
        {
          for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
          {
            newAPS->getAlfAPSParam().chromaCoeff[altIdx][i] = alfParamNewFilters.chromaCoeff[altIdx][i];
            newAPS->getAlfAPSParam().chromaClipp[altIdx][i] = alfParamNewFilters.chromaClipp[altIdx][i];
          }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          newAPS->getAlfAPSParam().chromaScaleIdx[altIdx][0] = alfParamNewFilters.chromaScaleIdx[altIdx][0];
#endif
        }
        m_apsMap->setChangedFlag( ( newApsIdChroma << NUM_APS_TYPE_LEN ) + ALF_APS );
        m_apsIdStart = newApsIdChroma;
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
        g_pictureControlInfo[newApsIdChroma].reset();
#endif
      }
      apss[cs.slice->getTileGroupApsIdChroma()] = m_apsMap->getPS( ( cs.slice->getTileGroupApsIdChroma() << NUM_APS_TYPE_LEN ) + ALF_APS );
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      auto apsIdxChroma = cs.slice->getTileGroupApsIdChroma();
      {
        for (auto comp = 1; comp < 3; ++comp)
        {
          ComponentID compID = (ComponentID)comp;
          if (cs.slice->getTileGroupAlfEnabledFlag(compID) && (!cs.slice->getTileGroupAlfReuseFlag(compID)))
          {
            memcpy(g_pictureControlInfo[apsIdxChroma].bufAlfFlag[compID].data(), m_ctuEnableFlag[compID], sizeof(uint8_t)* m_numCTUsInPic);
            memcpy(g_pictureControlInfo[apsIdxChroma].bufAlfFilter[compID].data(), m_ctuAlternative[compID], sizeof(uint8_t)* m_numCTUsInPic);
            g_pictureControlInfo[apsIdxChroma].alfTemporalLayer[compID] = cs.slice->getTLayer();
          }
        }
      }
#endif
    }
  }
}

#if JVET_AI0084_ALF_RESIDUALS_SCALING

int getEstVlcRate( const SPS* sps, ScaleAlf& curScaleAlfParam, int nbCorrMax )
{
  int rate = 0;

  if ( !curScaleAlfParam.initDone ) 
  {
    // this aps is not used in this slice
    if ( sps->getAlfScalePrevEnabled() )
    {
      rate++;
    }
    else
    {
      rate++;  // groupShift = 0
      rate++;  // curScaleAlfParam.groupIdxCorr[0] = 0
    }
    return rate;
  }

  if ( sps->getAlfScalePrevEnabled() )
  {
    bool  usePrev = curScaleAlfParam.usePrev;
    rate++;
    if ( usePrev )
    {
      return rate;
    }
  }

  rate += curScaleAlfParam.groupShift + 1;  //  READ_UVLC(uiCode, "alf_scale_group_shift");

  int nbGroup = 1 << curScaleAlfParam.groupShift;

  for (int g = 0; g < nbGroup; g++)
  {
    rate += 1;  // READ_FLAG(uiCode, "alf_scale_group_idx_corr");

    if ( curScaleAlfParam.groupIdxCorr[g] && nbCorrMax > 2 ) 
    {
      rate += ceilLog2(nbCorrMax - 1);
    }
  }

  return rate;
}

int getEstVlcRate( Slice& slice )
{
  int rate = 0;
  const int nbCorrMax = nbCorrAlfScale[ slice.getSPS()->getAlfScaleMode() ];
  bool  bCodedUseAlfScale = false;

  const std::vector<int32_t>& tabTileGroupApsIdLuma = slice.getTileGroupApsIdLuma();
  for (int i = 0; i < slice.getTileGroupNumAps(); i++)
  {
    if (i >= tabTileGroupApsIdLuma.size()) continue;

    const int apsIdx = tabTileGroupApsIdLuma[i];
    const AlfParam& alfParam = slice.getAlfAPSs()[apsIdx]->getAlfAPSParam();
    const int numAlts = alfParam.numAlternativesLuma;

    for (int j = 0; j < numAlts; j++)
    {
      ScaleAlf& curScaleAlfParam = slice.getAlfScale(i, j);

      if ( !bCodedUseAlfScale ) 
      {
        rate++;
        bCodedUseAlfScale = true;
      }

      if ( !slice.getUseAlfScale() ) 
      {
        continue;
      }

      curScaleAlfParam.apsIdx = apsIdx;
      rate += getEstVlcRate( slice.getSPS(), curScaleAlfParam, nbCorrMax );
    }
  }

  return rate;
}

void EncAdaptiveLoopFilter::alfCorrection( CodingStructure& cs, const PelUnitBuf& origBuf, const PelUnitBuf& recExtBuf, bool mode )
{
  const bool  bModeAnalysis = mode;

  if ( !cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y) || !cs.sps->getAlfScaleMode() )
  {
    cs.slice->setUseAlfScale(false);
    return;
  }

  PelUnitBuf& recBuf = cs.getRecoBufRef();
  const PreCalcValues& pcv = *cs.pcv;
  short* alfCtuFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();

  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };

  const ClpRng& clpRng = m_clpRngs.comp[COMPONENT_Y];

  const int nbCorrMax = m_nbCorr;

  if ( bModeAnalysis )
  {
    cs.slice->resetAlfScale();
  }

  const int adjustOffCorr = (1 << (shiftCorr + shiftPrecis - 1));

  const int offsetN = adjustOffCorr;
  const int offsetP = offsetN - 1;

  int numAlfLumaEnabled[ALF_CTB_MAX_NUM_APS];
  memset( numAlfLumaEnabled, 0, sizeof(int) * ALF_CTB_MAX_NUM_APS );

  int ctuIdx = 0;

  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      bool ctuEnableFlag = m_ctuEnableFlag[COMPONENT_Y][ctuIdx];
      for ( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
      {
        ctuEnableFlag |= m_ctuEnableFlag[compIdx][ctuIdx] > 0;
      }

      int rasterSliceAlfPad = 0;
      bool  nok = ctuEnableFlag && isCrossedByVirtualBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad);
      CHECK( nok, "alfCorrection() with isCrossedByVirtualBoundaries() not implemented.");

      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      if ( m_ctuEnableFlag[COMPONENT_Y][ctuIdx] )
      {
        Area blk(xPos, yPos, width, height);
        short filterSetIndex = alfCtuFilterIndex[ctuIdx];

        if ( filterSetIndex < NUM_FIXED_FILTER_SETS )
        {
          // do not consider fixed filters
        }
        else
        {
          if ( bModeAnalysis ) 
          {
            numAlfLumaEnabled[filterSetIndex - NUM_FIXED_FILTER_SETS]++;
          }

          const int alt_num = m_ctuAlternative[COMPONENT_Y][ctuIdx];
          int classifierIdx = m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
          ScaleAlf& curScaleAlfParam        = cs.slice->getAlfScale( filterSetIndex - NUM_FIXED_FILTER_SETS, alt_num );
          ScaleAlfEnc& curScaleAlfEncParam  = getAlfScaleEnc( filterSetIndex - NUM_FIXED_FILTER_SETS, alt_num );

          if ( bModeAnalysis && !curScaleAlfParam.initDone ) 
          {
            curScaleAlfEncParam.reset();

#if JVET_AJ0237_INTERNAL_12BIT
            curScaleAlfParam.init(filterSetIndex, alt_num, classifierIdx, cs.slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
#else
            curScaleAlfParam.init( filterSetIndex, alt_num, classifierIdx );
#endif
            curScaleAlfParam.setMinMax( cs.slice->getLumaPelMin(), cs.slice->getLumaPelMax() );

            const int apsIdx = cs.slice->getTileGroupApsIdLuma()[filterSetIndex - NUM_FIXED_FILTER_SETS];
            curScaleAlfParam.setApsIdx( apsIdx );
          }

          CHECK( curScaleAlfParam.classifierIdx != classifierIdx || curScaleAlfParam.filterSetIndex != filterSetIndex || curScaleAlfParam.alt_num != alt_num, "alfCorrection() failed.");

          char coeffBits = m_coeffBitsApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AJ0237_INTERNAL_12BIT
          const Pel currBase = 1 << (curScaleAlfParam.bitDepth - 1);
#else
          const Pel currBase = 512; // 10-bits
#endif

          if ( !bModeAnalysis )
          {
            CHECK( !curScaleAlfParam.initMinMaxDone, "initMinMaxDone should be true");

            alfAddCorrect( m_classifier[classifierIdx], recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeffBits, curScaleAlfParam.idxCorr.data() );
          }
          else
          {
            const PelBuf& orgLuma = origBuf.get(COMPONENT_Y); // orig
            Pel* org = orgLuma.buf + blk.y * orgLuma.stride + blk.x;
            const PelBuf& recLuma = recExtBuf.get(COMPONENT_Y); // before alf correction
            Pel* rec = recLuma.buf + blk.y * recLuma.stride + blk.x;
            const PelBuf& dstLuma = recBuf.get(COMPONENT_Y); // after alf correction
            Pel* dst = dstLuma.buf + blk.y * dstLuma.stride + blk.x;

            for (int i = 0; i < blk.height; i++)
            {
              for (int j = 0; j < blk.width; j++)
              {
                int idxClass = m_classifier[classifierIdx][blk.y + i][blk.x + j] >> 2;

                Pel corr = dst[j] - currBase;
                for ( int idxCorr = 0; idxCorr < (bModeAnalysis ? nbCorrMax : 1); idxCorr++ )
                {
                  int sCorr = bModeAnalysis ? getScaleCorrInt( idxCorr ) : getScaleCorrInt( curScaleAlfParam.idxCorr[idxClass] );

                  Pel dstS = rec[j] + ((corr * sCorr + (corr > 0 ? offsetP : offsetN)) >> (shiftCorr + shiftPrecis));  // re-scaled alf correction

                  dstS = ClipPel( dstS, clpRng );

                  int diff = (dstS - org[j]);

                  curScaleAlfEncParam.addMse( idxClass, idxCorr, diff );

                } //for idxCorr
              } // j

            org += orgLuma.stride;
            rec += recLuma.stride;
            dst += dstLuma.stride;
            } // i

          }

        }
      } // m_ctuEnableFlag[COMPONENT_Y][ctuIdx]

      ctuIdx++;
    } // xPos
  } // yPos


  if ( !bModeAnalysis ) 
  {
    return;
  }

  int64_t   sumBestMseDiff = 0;

  cs.slice->setUseAlfScale(false);

  double rateOff = (double)getEstVlcRate(*cs.slice);

  double estCostAllOff = m_lambda[COMPONENT_Y] * rateOff;

  cs.slice->setUseAlfScale(true);


  const int maxGroupShift = ScaleAlf::maxGroupShift;

  const int maxCptAps = ALF_CTB_MAX_NUM_APS * MAX_NUM_ALF_ALTERNATIVES_LUMA;
  ScaleAlf* gScaleAlf[maxCptAps];
  for ( int i = 0; i < maxCptAps; i++ ) 
  {
    gScaleAlf[i] = nullptr;
  }
  int cptAps = 0;

  for (int f = 0; f < cs.slice->getTileGroupNumAps(); f++)
  {
    int apsIdx = cs.slice->getTileGroupApsIdLuma()[f];

    APS* curAps = cs.slice->getAlfAPSs()[apsIdx];
    if ( !curAps )
    {
      continue;
    }
    const AlfParam& alfParam = curAps->getAlfAPSParam();
    const int numAlts = alfParam.numAlternativesLuma;
    for (int a = 0; a < numAlts; a++)
    {
      ScaleAlf& curScaleAlfParam = cs.slice->getAlfScale(f, a);
      ScaleAlfEnc& curScaleAlfEncParam = getAlfScaleEnc(f, a);
      curScaleAlfParam.usePrev = false;

      if ( curScaleAlfParam.initDone )  // => it means that this APS-alt is actually used in this frame
      {
        gScaleAlf[cptAps] = cs.slice->getAlfScalePtr(f, a);

        int kMin = curScaleAlfParam.idxClassMin;
        int kMax = curScaleAlfParam.idxClassMax;

        // init best idxCorr as s=0 :
        std::vector<int>  bestGroupIdxCorr;
        bestGroupIdxCorr.resize(MAX_NUM_ALF_CLASSES, 0);
        int64_t   bestGroupMse = 0;

        uint64_t  sumBestMse0 = 0;
        for (int k = kMin; k <= kMax; k++) 
        {
          sumBestMse0 += curScaleAlfEncParam.mse[k][0];
        }

        // init best cost with 1 group and s=0 :
        curScaleAlfParam.setGroupSize(0);
        curScaleAlfParam.groupIdxCorr.resize(1, 0);
        curScaleAlfParam.fillIdxCorr();

        double estCostBestApsAlt = m_lambda[COMPONENT_Y] * (double)getEstVlcRate( cs.sps, curScaleAlfParam, nbCorrMax );

        ScaleAlf  tmpScaleAlfParam = curScaleAlfParam;

        double estCostCurApsAlt = 0;

        for (int groupShift = 0; groupShift <= maxGroupShift; groupShift++)  // group the scaling values with consecutive classes
        {
          bool bCurGroupShiftIsBest = false;

          tmpScaleAlfParam = curScaleAlfParam;
          tmpScaleAlfParam.setGroupSize(groupShift);
          int groupSize = tmpScaleAlfParam.groupSize;
          int nbGroup = tmpScaleAlfParam.groupNum;

          if (groupSize == 0)
          {
            continue;
          }

          bestGroupIdxCorr.resize(MAX_NUM_ALF_CLASSES, 0);
          memset(bestGroupIdxCorr.data(), 0, sizeof(int) * MAX_NUM_ALF_CLASSES);

          double estCostBestCurGroupShiftApsAlt = MAX_DOUBLE;

          double    sumBestCost = MAX_DOUBLE;
          uint64_t  sumBestMse = 0;
          sumBestMse0 = 0;
          for (int g = 0; g < nbGroup; g++)
          {
            tmpScaleAlfParam.groupIdxCorr = bestGroupIdxCorr;

            int kMinG = kMin + g * groupSize;
            int kMaxG = (g == nbGroup - 1) ? (kMax + 1) : (kMin + (g + 1) * groupSize);

            int bestIdxCorr = 0;

            uint64_t  bestMse0 = 0;
            for (int k = kMinG; k < kMaxG; k++) 
            {
              bestMse0 += curScaleAlfEncParam.mse[k][0];
            }

            uint64_t  bestMse = bestMse0;
            for (int s = 0; s < nbCorrMax; s++)
            {
              tmpScaleAlfParam.groupIdxCorr[g] = s;

              uint64_t  curMse = 0;
              for (int k = kMinG; k < kMaxG; k++)
              {
                curMse += curScaleAlfEncParam.mse[k][s];
              }

              int64_t mseDiff = (int64_t)bestMse0 - (int64_t)curMse;
              int64_t mseSumDiff = (int64_t)sumBestMse - (int64_t)sumBestMse0 - mseDiff;

              double rate = (double)getEstVlcRate( cs.sps, tmpScaleAlfParam, nbCorrMax );

              double rateEstCur = m_lambda[COMPONENT_Y] * rate + (double)mseSumDiff;

              estCostCurApsAlt = rateEstCur;

              if ( estCostCurApsAlt < sumBestCost ) 
              { // best for this groupShift
                sumBestCost = estCostCurApsAlt;
                bestMse = curMse;
                bestIdxCorr = s;
              }

              if ( estCostCurApsAlt < estCostBestApsAlt ) 
              { // overall the best
                estCostBestApsAlt = estCostCurApsAlt;
                bestIdxCorr = s;
                bestMse = curMse;
                bCurGroupShiftIsBest = true;
                curScaleAlfParam = tmpScaleAlfParam;
              }

              if ( estCostCurApsAlt < estCostBestCurGroupShiftApsAlt ) 
              {
                estCostBestCurGroupShiftApsAlt = estCostCurApsAlt;
              }

            } // s
            sumBestMse += bestMse;
            sumBestMse0 += bestMse0;

            bestGroupIdxCorr[g] = bestIdxCorr;

          } // g

          if (bCurGroupShiftIsBest) 
          {
            bestGroupMse = sumBestMse - sumBestMse0;
          }

          curScaleAlfParam.fillIdxCorr();

        } // groupShift++


        if ( cs.sps->getAlfScalePrevEnabled() )
        {
          tmpScaleAlfParam.usePrev = true;
          tmpScaleAlfParam.idxCorr = getAlfScalePrev( apsIdx, a );

          int64_t  mseSumDiff = 0;
          for (int k = 0; k < MAX_NUM_ALF_CLASSES; k++)
          {
            const int s = tmpScaleAlfParam.idxCorr[k];
            mseSumDiff += curScaleAlfEncParam.mse[k][s] - curScaleAlfEncParam.mse[k][0];
          }

          double rate = (double)getEstVlcRate( cs.sps, tmpScaleAlfParam, nbCorrMax);

          estCostCurApsAlt = m_lambda[COMPONENT_Y] * rate + (double)mseSumDiff;

          if ( estCostCurApsAlt < estCostBestApsAlt ) 
          { // overall the best
            estCostBestApsAlt = estCostCurApsAlt;

            curScaleAlfParam = tmpScaleAlfParam;

            bestGroupMse = mseSumDiff;
          }
        }

        sumBestMseDiff += bestGroupMse;

        if ( !curScaleAlfParam.usePrev )
        {
          curScaleAlfParam.fillIdxCorr();
        }

        cptAps++;

      }
      else
      {
        if ( cs.sps->getAlfScalePrevEnabled() )
        {
          curScaleAlfParam.usePrev = true;  // force lowest coding cost
          curScaleAlfParam.idxCorr = getAlfScalePrev( apsIdx, a );
        }
      }

    } // a - numAlts
  } // f - cs.slice->getTileGroupNumAps()


  double rate = (double)getEstVlcRate( *cs.slice );

  double cost = m_lambda[COMPONENT_Y] * rate + double(sumBestMseDiff);

  if ( cost >= estCostAllOff )
  {
    cs.slice->setUseAlfScale( false );
    for (int i = 0; i < cptAps; i++) 
    {
      memset( gScaleAlf[i]->groupIdxCorr.data(), 0, sizeof(int) * gScaleAlf[i]->groupIdxCorr.size() );
      memset( gScaleAlf[i]->idxCorr.data(), 0, sizeof(int) * gScaleAlf[i]->idxCorr.size() );
    }
  }

}
#endif

#if JVET_AI0084_ALF_RESIDUALS_SCALING
void EncAdaptiveLoopFilter::alfCorrectionChroma( CodingStructure& cs, PelUnitBuf& tmpYuvRecSAO )
{
  if ( !cs.sps->getAlfScaleMode() )
  {
    return;
  }
  PelUnitBuf recYuvTmp = cs.getRecoBuf();

#if JVET_AL0153_ALF_CCCM
#if ALF_SAO_TRUE_ORG
  PelUnitBuf orgYuvTmp = cs.slice->getLfCccmEnabledFlag() ? *m_newOrgBuf : cs.getTrueOrgBuf();
#else
  PelUnitBuf orgYuvTmp = cs.slice->getLfCccmEnabledFlag() ? *m_newOrgBuf : cs.getOrgBuf();
#endif
#else
#if ALF_SAO_TRUE_ORG
  PelUnitBuf orgYuvTmp = cs.getTrueOrgBuf();
#else
  PelUnitBuf orgYuvTmp = cs.getOrgBuf();
#endif
#endif

  int ccAlfUsedFlag[3] = { false, false, false };
  if ( cs.slice->getSPS()->getCCALFEnabledFlag() )
  {
    ccAlfUsedFlag[1] = cs.slice->getTileGroupCcAlfCbEnabledFlag() ? 1 : 0;
    ccAlfUsedFlag[2] = cs.slice->getTileGroupCcAlfCrEnabledFlag() ? 1 : 0;
  }

  int alfUsedFlag[3];
  alfUsedFlag[0] = cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y)  ? 1 : 0;
  alfUsedFlag[1] = cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) ? 1 : ccAlfUsedFlag[1];
  alfUsedFlag[2] = cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr) ? 1 : ccAlfUsedFlag[2];

  const PreCalcValues& pcv = *cs.pcv;

  int offset = (1 << (shiftCorrChroma - 1));

  const int offsetN = offset;
  const int offsetP = offsetN - 1;

  int shiftCorr = shiftCorrChroma;

  int sBest[2] = { -1, -1 };

  for (int comp = 1; comp < MAX_NUM_COMPONENT; comp++)
  {
    cs.slice->setAlfScaleChroma( comp, 0 );

    std::vector<int64_t> ssdROA;
    ssdROA.resize( m_nbCorrChroma, 0 );

    if ( alfUsedFlag[comp] )
    {
      ComponentID compID  = ComponentID(comp);
      PelBuf& saoPelBuf   = tmpYuvRecSAO.get(compID);
      PelBuf& recPelBuf   = recYuvTmp.get(compID);
      PelBuf& orgPelBuf   = orgYuvTmp.get(compID);

      const ClpRng& clpRng = m_clpRngs.comp[comp];

      int ctuIdx = 0;
      for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
      {
        for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
        {
          const int chromaScaleX = getComponentScaleX(compID, cs.getRecoBuf().chromaFormat);
          const int chromaScaleY = getComponentScaleY(compID, cs.getRecoBuf().chromaFormat);

          const int width   = ((xPos + pcv.maxCUWidth  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUWidth)  >> chromaScaleX;
          const int height  = ((yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight) >> chromaScaleY;

          bool ctuEnableFlag_alf = cs.picture->getAlfCtuEnableFlag(compID)[ctuIdx];

          bool     skipFiltering_ccalf = false;

          if ( !ctuEnableFlag_alf && skipFiltering_ccalf )
          {
            ctuIdx++;
            continue;
          }

          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width, height );

          Pel* org = orgPelBuf.buf + blk.y * orgPelBuf.stride + blk.x;
          Pel* sao = saoPelBuf.buf + blk.y * saoPelBuf.stride + blk.x;
          Pel* rec = recPelBuf.buf + blk.y * recPelBuf.stride + blk.x;

          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              int diffOrg   = rec[x] - org[x];
              int diffOrg2  = diffOrg * diffOrg;

              int corr = rec[x] - sao[x];

              for (int s = 1; s < m_nbCorrChroma; s++)
              {
                int sCorr = m_scaleCorrChroma[s];

                Pel dstS = sao[x] + ((corr * sCorr + (corr > 0 ? offsetP : offsetN)) >> shiftCorr);  // re-scaled alf correction

                dstS = ClipPel( dstS, clpRng );

                int diffScale = dstS - org[x];

                ssdROA[s] += int64_t( diffScale * diffScale - diffOrg2 );
              } // s

            } // x
            org += orgPelBuf.stride;
            rec += recPelBuf.stride;
            sao += saoPelBuf.stride;
          }  // y

          ctuIdx++;
        } // xPos
      } // yPos

      int64_t ssdBest     = 0;

      double estCostBest = m_lambda[comp] * 1 + (double)ssdBest;

      for ( int s = 1; s < m_nbCorrChroma; s++ )
      {
        int rate = 1;
        rate += s ? ceilLog2(m_nbCorrChroma - 1) : 0;

        double  estCost = m_lambda[comp] * rate + (double)ssdROA[s];

        if ( estCost < estCostBest ) 
        {
          estCostBest   = estCost;
          sBest[comp-1] = s;
          ssdBest     = ssdROA[s];
          cs.slice->setAlfScaleChroma( comp, s );
        }
      }

    } // alfUsedFlag[comp]
  } // comp


  if ( sBest[0] > 0 || sBest[1] > 0 )
  {
    alfAddCorrectChroma( cs, tmpYuvRecSAO );
  }

}
#endif

void EncAdaptiveLoopFilter::alfReconstructor(CodingStructure& cs, const PelUnitBuf& recExtBuf)
{
  if (!cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    return;
  }
  reconstructCoeffAPSs(cs, true, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) || cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr), false);
  short* alfCtuFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  PelUnitBuf& recBuf = cs.getRecoBufRef();
  const PreCalcValues& pcv = *cs.pcv;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if ALF_IMPROVEMENT
  PelUnitBuf tmpYuvBeforeDb = m_tempBufBeforeDb.getBuf( cs.area );
#else
  PelUnitBuf tmpYuvBeforeDb = m_tempBufBeforeDb.getBuf(UnitArea(CHROMA_400, Area(cs.area.blocks[COMPONENT_Y])));
#endif
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  PelUnitBuf tmpYuvResi = m_tempBufResi.getBuf(UnitArea(CHROMA_400, Area(cs.area.blocks[COMPONENT_Y])));
#endif
#if ALF_IMPROVEMENT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
  int fixedFilterSetIdx = cs.slice->getTileGroupAlfFixedFilterSetIdx( COMPONENT_Y );
  int fixedFilterSetIdxChroma[2] = { cs.slice->getTileGroupAlfFixedFilterSetIdx( COMPONENT_Cb ), cs.slice->getTileGroupAlfFixedFilterSetIdx( COMPONENT_Cr ) };
#else
  int fixedFilterSetIdx = cs.slice->getTileGroupAlfFixedFilterSetIdx();
#endif
#endif

  int ctuIdx = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };
  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      bool ctuEnableFlag = m_ctuEnableFlag[COMPONENT_Y][ctuIdx];
      for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ctuEnableFlag |= m_ctuEnableFlag[compIdx][ctuIdx] > 0;
      }
      int rasterSliceAlfPad = 0;
      if ( ctuEnableFlag && isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for (int i = 0; i <= numHorVirBndry; i++)
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = (i == 0 && clipTop) || (i > 0) || (yStart == 0);
          const bool clipB = (i == numHorVirBndry && clipBottom) || (i < numHorVirBndry ) || (yEnd == pcv.lumaHeight);
          int xStart = xPos;
          for (int j = 0; j <= numVerVirBndry; j++)
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = (j == 0 && clipLeft) || (j > 0) || (xStart == 0);
            const bool clipR = (j == numVerVirBndry && clipRight) || (j < numVerVirBndry ) || (xEnd == pcv.lumaWidth);
            const int wBuf = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
            const int hBuf = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
            PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
            buf.copyFrom(recExtBuf.subBuf(UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
            // pad top-left unavailable samples for raster slice
            if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, buf, MAX_ALF_PADDING_SIZE, true, true);
#else
            buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
#endif
            buf = buf.subBuf(UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));


            if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
            {
              const Area blkSrc(0, 0, w, h);
              const Area blkDst(xStart, yStart, w, h);
              short filterSetIndex = alfCtuFilterIndex[ctuIdx];
              short *coeff;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
              Pel *clip;
#else
              short *clip;
#endif
#if ALF_IMPROVEMENT
              if (filterSetIndex < NUM_FIXED_FILTER_SETS)
              {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
                copyFixedFilterResults(recBuf, blkDst, COMPONENT_Y, m_fixFilterResult[COMPONENT_Y], fixedFilterSetIdx, filterSetIndex);
#else
                copyFixedFilterResults(recBuf, blkDst, COMPONENT_Y, m_fixFilterResult, fixedFilterSetIdx, filterSetIndex);
#endif
              }
              else
              {
                const int alt_num = m_ctuAlternative[COMPONENT_Y][ctuIdx];
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                char* scaleIdx = m_scaleIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#endif
                AlfFilterType filterTypeCtb = m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                if( m_isFixedFilterPaddedPerCtu )
                {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
                  paddingFixedFilterResultsCtu(m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst, 1);
#else
                  paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst, 1);
#endif
#else
                  paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst);
#endif
                }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                if( m_isFixedFilterPaddedPerCtu )
                {
                  for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
                  {
                    paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, blkDst);
                  }
                }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                if( m_isFixedFilterPaddedPerCtu )
                {
                  for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++)
                  {
                    paddingLaplacianResultsCtu(m_laplacianPic, m_laplacianCtu, laplacianIdx, blkDst);
                  }
                }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf( UnitArea ( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
#if JVET_AA0095_ALF_LONGER_FILTER
                if( filterTypeCtb == ALF_FILTER_9_EXT_DB || filterTypeCtb == ALF_FILTER_13_EXT_DB 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                    || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI
                    || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#endif
                  )
#else
                if( filterTypeCtb == ALF_FILTER_9_EXT_DB )
#endif
                {
                  bufDb.copyFrom( m_tempBufBeforeDb.subBuf( UnitArea( CHROMA_400, Area( xStart - (clipL ? 0 : NUM_DB_PAD), yStart - ( clipT ? 0 : NUM_DB_PAD ), wBuf, hBuf))));
                  // pad top-left unavailable samples for raster slice
                  if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
                  {
                    bufDb.padBorderPel( NUM_DB_PAD, 1 );
                  }
                  // pad bottom-right unavailable samples for raster slice
                  if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
                  {
                    bufDb.padBorderPel( NUM_DB_PAD, 2 );
                  }
                  bufDb.extendBorderPel( NUM_DB_PAD );
                  bufDb = bufDb.subBuf( UnitArea( CHROMA_400, Area( clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h ) ) );
                }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                PelUnitBuf bufResi = m_tempBufResi2.subBuf(UnitArea(CHROMA_400, Area(0, 0, wBuf, hBuf)));
                if (filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI_DIRECT)
                {
                  bufResi.copyFrom(
                    m_tempBufResi.subBuf(UnitArea(CHROMA_400, Area(xStart - (clipL ? 0 : NUM_RESI_PAD),
                                                                   yStart - (clipT ? 0 : NUM_RESI_PAD), wBuf, hBuf))));
                  // pad top-left unavailable samples for raster slice
                  if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
                  {
                    bufResi.padBorderPel(NUM_RESI_PAD, 1);
                  }
                  // pad bottom-right unavailable samples for raster slice
                  if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
                  {
                    bufResi.padBorderPel(NUM_RESI_PAD, 2);
                  }
                  bufResi.extendBorderPel(NUM_RESI_PAD);
                  bufResi = bufResi.subBuf(
                    UnitArea(CHROMA_400, Area(clipL ? 0 : NUM_RESI_PAD, clipT ? 0 : NUM_RESI_PAD, w, h)));
                }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
                int classifierIdx = m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
                char coeffBits = m_coeffBitsApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
                if ( cs.sps->getAlfScaleMode() )
                {
                  fixedFilterSetIdx = - fixedFilterSetIdx - 1;  // indicate correction only is saved in buf 
                }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, coeffBits, scaleIdx);
#else
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, coeffBits);
#endif
#else
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, coeffBits);
#endif
#else
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu);
#else
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, coeffBits);
#else
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
                alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[classifierIdx], recBuf, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier[classifierIdx], recBuf, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[classifierIdx], recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier[classifierIdx], recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recBuf, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier, recBuf, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
                if ( cs.sps->getAlfScaleMode() )
                {
                  fixedFilterSetIdx = - fixedFilterSetIdx - 1;  // restore value of 'fixedFilterSetIdx'
                }
#endif
              }
#else
              if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
              {
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
              }
              else
              {
                coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
                clip = m_clipDefault;
              }
              m_filter7x7Blk( m_classifier, recBuf, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
#endif
            }

            for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
            {
              ComponentID compID = ComponentID(compIdx);
              const int chromaScaleX = getComponentScaleX(compID, recBuf.chromaFormat);
              const int chromaScaleY = getComponentScaleY(compID, recBuf.chromaFormat);
              if (m_ctuEnableFlag[compIdx][ctuIdx])
              {
                const Area blkSrc(0, 0, w >> chromaScaleX, h >> chromaScaleY);
                const Area blkDst(xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY);
                const int alt_num = m_ctuAlternative[compID][ctuIdx];
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
#if JVET_AK0123_ALF_COEFF_RESTRICTION
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compID], nullptr, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, m_NUM_BITS_CHROMA, m_chromaScaleIdxFinal[alt_num]);
#else
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compID], nullptr, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, m_NUM_BITS_CHROMA);
#endif
#else
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compID], nullptr, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_NUM_BITS_CHROMA);
#endif
#else
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compID], nullptr, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu);
#else
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compID], nullptr, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu, m_NUM_BITS_CHROMA);
#else
                alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
                alfFiltering( m_classifier[0], recBuf, tmpYuvBeforeDb, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                             tmpYuvResi,
#endif
                  buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                             nullptr,
#endif
                  -1, nullptr, false );
#endif
#else
                alfFiltering( m_classifier[0], recBuf, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[0], recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier[0], recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recBuf, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier, recBuf, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#endif
#else
                m_filter5x5Blk( m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_alfVBChmaCTUHeight, m_alfVBChmaPos );
#endif
              }
            }

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {

      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
      {
        Area blk(xPos, yPos, width, height);
        short filterSetIndex = alfCtuFilterIndex[ctuIdx];
        short *coeff;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
          Pel *clip;
#else        
        short *clip;
#endif
#if ALF_IMPROVEMENT
        if( filterSetIndex < NUM_FIXED_FILTER_SETS )
        {
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
          copyFixedFilterResults(recBuf, blk, COMPONENT_Y, m_fixFilterResult[COMPONENT_Y], fixedFilterSetIdx, filterSetIndex);
#else
          copyFixedFilterResults(recBuf, blk, COMPONENT_Y, m_fixFilterResult, fixedFilterSetIdx, filterSetIndex);
#endif
        }
        else
        {
          const int alt_num = m_ctuAlternative[COMPONENT_Y][ctuIdx];
          coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
          clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          char* scaleIdx = m_scaleIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#endif
          AlfFilterType filterTypeCtb = m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          if( m_isFixedFilterPaddedPerCtu )
          {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
            paddingFixedFilterResultsCtu(m_fixFilterResult[COMPONENT_Y], m_fixedFilterResultPerCtu, fixedFilterSetIdx, blk, 1);
#else
            paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blk, 1);
#endif
#else
            paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blk);
#endif
          }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          if( m_isFixedFilterPaddedPerCtu )
          {
            for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
            {
              paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, blk);
            }
          }
#endif
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          if( m_isFixedFilterPaddedPerCtu )
          {
            for(int laplacianIdx = 0; laplacianIdx < NUM_LAPLACIAN_FILTERED_SOURCE; laplacianIdx++)
            {
              paddingLaplacianResultsCtu(m_laplacianPic, m_laplacianCtu, laplacianIdx, blk);
            }
          }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
          int classifierIdx = m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          char coeffBits = m_coeffBitsApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
          if ( cs.sps->getAlfScaleMode() )
          {
            fixedFilterSetIdx = - fixedFilterSetIdx - 1;  // indicate correction only is saved in recExtBuf 
          }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, coeffBits, scaleIdx);
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, coeffBits);
#endif
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, coeffBits);
#endif
#else
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu);
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult[COMPONENT_Y], m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, coeffBits);
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                       tmpYuvResi,
#endif
            recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                       m_fixFilterResiResult,
#endif
            fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#endif
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering(m_classifier[classifierIdx], recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#else
          alfFiltering(m_classifier[classifierIdx], recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx);
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering(m_classifier, recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#else
          alfFiltering(m_classifier, recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
          alfFiltering(m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx);
#endif
#endif
#endif
#if JVET_AI0084_ALF_RESIDUALS_SCALING
          if ( cs.sps->getAlfScaleMode() )
          {
            fixedFilterSetIdx = - fixedFilterSetIdx - 1;  // restore value of 'fixedFilterSetIdx'
          }
#endif
        }
#else
        if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
        {
          coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
          clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
        }
        else
        {
          coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
          clip = m_clipDefault;
        }
        m_filter7x7Blk( m_classifier, recBuf, recExtBuf, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
#endif
      }

      const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, recBuf.chromaFormat );
      const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, recBuf.chromaFormat );

      for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
      {
        ComponentID compID = ComponentID(compIdx);
        if (m_ctuEnableFlag[compIdx][ctuIdx])
        {
          Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          const int altNum = m_ctuAlternative[compIdx][ctuIdx];
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
#if JVET_AK0123_ALF_COEFF_RESTRICTION
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compIdx], m_fixFilterResiResult, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, m_NUM_BITS_CHROMA, m_chromaScaleIdxFinal[altNum]);
#else
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compIdx], m_fixFilterResiResult, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu, m_NUM_BITS_CHROMA); 
#endif
#else
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compIdx], m_fixFilterResiResult, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_NUM_BITS_CHROMA);
#endif
#else
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compIdx], m_fixFilterResiResult, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu, m_laplacianPic, m_laplacianCtu);
#else
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, m_fixFilterResult[compIdx], m_fixFilterResiResult, fixedFilterSetIdxChroma[compIdx - 1], m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu, m_NUM_BITS_CHROMA);
#else
          alfFiltering(m_classifier[0], recBuf, tmpYuvBeforeDb, tmpYuvResi, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu);
#endif
#endif
#else
          alfFiltering( m_classifier[0], recBuf, tmpYuvBeforeDb, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                       tmpYuvResi,
#endif
            recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                       nullptr,
#endif
            -1, nullptr, false );
#endif
#else
          alfFiltering( m_classifier[0], recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering( m_classifier[0], recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
          alfFiltering( m_classifier[0], recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering( m_classifier, recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
          alfFiltering( m_classifier, recBuf, tmpYuvBeforeDb, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          alfFiltering( m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
          alfFiltering( m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#endif
#else
          m_filter5x5Blk( m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altNum], m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], cs, m_alfVBChmaCTUHeight, m_alfVBChmaPos );
#endif
        }
      }
      }
      ctuIdx++;
    }
  }

}

void EncAdaptiveLoopFilter::copyCtuAlternativeChroma( uint8_t* ctuAltsDst[MAX_NUM_COMPONENT], uint8_t* ctuAltsSrc[MAX_NUM_COMPONENT] )
{
  std::copy_n( ctuAltsSrc[COMPONENT_Cb], m_numCTUsInPic, ctuAltsDst[COMPONENT_Cb] );
  std::copy_n( ctuAltsSrc[COMPONENT_Cr], m_numCTUsInPic, ctuAltsDst[COMPONENT_Cr] );
}

void EncAdaptiveLoopFilter::setCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT], uint8_t val )
{
  std::fill_n( ctuAlts[COMPONENT_Cb], m_numCTUsInPic, val );
  std::fill_n( ctuAlts[COMPONENT_Cr], m_numCTUsInPic, val );
}

#if ALF_IMPROVEMENT
void EncAdaptiveLoopFilter::initCtuAlternativeLuma(uint8_t* ctuAlts[MAX_NUM_COMPONENT])
{
  uint8_t altIdx = 0;
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx)
  {
    ctuAlts[COMPONENT_Y][ctuIdx] = altIdx;
    if ((ctuIdx + 1) * m_alfParamTemp.numAlternativesLuma >= (altIdx + 1)*m_numCTUsInPic)
      ++altIdx;
  }
}
#endif

void EncAdaptiveLoopFilter::initCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMPONENT] )
{
  uint8_t altIdx = 0;
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMPONENT_Cb][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
  altIdx = 0;
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMPONENT_Cr][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
}

int EncAdaptiveLoopFilter::getMaxNumAlternativesChroma( )
{
  return std::min<int>( m_numCTUsInPic * 2, m_encCfg->getMaxNumAlfAlternativesChroma() );
}

int EncAdaptiveLoopFilter::getCoeffRateCcAlf(short chromaCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], bool filterEnabled[MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, ComponentID compID)
{
  int bits = 0;

  if ( filterCount > 0 )
  {
    bits += lengthUvlc(filterCount - 1);
    int signaledFilterCount = 0;
    for ( int filterIdx=0; filterIdx<MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      if (filterEnabled[filterIdx])
      {
        AlfFilterShape alfShape(size_CC_ALF);
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          bits += CCALF_BITS_PER_COEFF_LEVEL + (chromaCoeff[filterIdx][i] == 0 ? 0 : 1);
        }

        signaledFilterCount++;
      }
    }
    CHECK(signaledFilterCount != filterCount, "Number of filter signaled not same as indicated");
  }

  return bits;
}

#if JVET_AH0057_CCALF_COEFF_PRECISION
void EncAdaptiveLoopFilter::deriveCcAlfFilterCoeff( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx, int coeffPrec )
#else
void EncAdaptiveLoopFilter::deriveCcAlfFilterCoeff( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx )
#endif
{
  int forward_tab[CCALF_CANDS_COEFF_NR * 2 - 1] = {0};
  for (int i = 0; i < CCALF_CANDS_COEFF_NR; i++)
  {
    forward_tab[CCALF_CANDS_COEFF_NR - 1 + i] = CCALF_SMALL_TAB[i];
    forward_tab[CCALF_CANDS_COEFF_NR - 1 - i] = (-1) * CCALF_SMALL_TAB[i];
  }

#if JVET_X0071_LONGER_CCALF && !JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && !JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS && !JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AF0177_ALF_COV_FLOAT
  using TE = float[MAX_NUM_CC_ALF_CHROMA_COEFF][MAX_NUM_CC_ALF_CHROMA_COEFF];
  using Ty = float[MAX_NUM_CC_ALF_CHROMA_COEFF];
#else
  using TE = double[MAX_NUM_CC_ALF_CHROMA_COEFF][MAX_NUM_CC_ALF_CHROMA_COEFF];
  using Ty = double[MAX_NUM_CC_ALF_CHROMA_COEFF];
#endif
#else
#if JVET_AF0177_ALF_COV_FLOAT
  using TE = float[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = float[MAX_NUM_ALF_LUMA_COEFF];
#else
  using TE = double[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = double[MAX_NUM_ALF_LUMA_COEFF];
#endif
#endif
#if JVET_AF0177_ALF_COV_FLOAT
  Ty filterCoeffDbl;
#else
  double filterCoeffDbl[MAX_NUM_CC_ALF_CHROMA_COEFF];
#endif
  int16_t filterCoeffInt[MAX_NUM_CC_ALF_CHROMA_COEFF];

  std::fill_n(filterCoeffInt, MAX_NUM_CC_ALF_CHROMA_COEFF, 0);

  TE        kE;
  Ty        ky;
  const int size = m_filterShapesCcAlf[0].numCoeff - 1;

  for (int k = 0; k < size; k++)
  {
    ky[k] = m_alfCovarianceFrameCcAlf[0].y(0,k);
    // Upper triangular
    for (int l = k; l < size; l++)
    {
      kE[k][l] = m_alfCovarianceFrameCcAlf[0].E(0,0,k,l);
    }
  }

  m_alfCovarianceFrameCcAlf[0].gnsSolveByChol(kE, ky, filterCoeffDbl, size
#if JVET_AN0086_RESIDUAL_CHECK
  , m_encCfg->getAlfResidualCheckEnabled()
#endif
  );
#if JVET_AH0057_CCALF_COEFF_PRECISION
  roundFiltCoeffCCALF(filterCoeffInt, filterCoeffDbl, size, (1 << coeffPrec));
#else
  roundFiltCoeffCCALF(filterCoeffInt, filterCoeffDbl, size, (1 << m_scaleBits));
#endif

  for (int k = 0; k < size; k++)
  {
    CHECK( filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE), "this is not possible: filterCoeffInt[k] <  -(1 << CCALF_DYNAMIC_RANGE)");
    CHECK( filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE), "this is not possible: filterCoeffInt[k] >  (1 << CCALF_DYNAMIC_RANGE)");
  }

  // Refine quanitzation
  int modified       = 1;
#if JVET_AH0057_CCALF_COEFF_PRECISION
  double errRef      = m_alfCovarianceFrameCcAlf[0].calcErrorForCcAlfCoeffs(filterCoeffInt, size, (coeffPrec + 1));
#else
  double errRef      = m_alfCovarianceFrameCcAlf[0].calcErrorForCcAlfCoeffs(filterCoeffInt, size, (m_scaleBits+1));
#endif

  while (modified)
  {
    modified = 0;
    for (int delta : { 1, -1 })
    {
      double errMin = MAX_DOUBLE;
      int    idxMin = -1;
      int minIndex = -1;

      for (int k = 0; k < size; k++)
      {
        int org_idx = -1;
        for (int i = 0; i < CCALF_CANDS_COEFF_NR * 2 - 1; i++)
        {
          if (forward_tab[i] == filterCoeffInt[k])
          {
            org_idx = i;
            break;
          }
        }
        CHECK( org_idx < 0, "this is wrong, does not find coeff from forward_tab");
        if ( (org_idx - delta < 0) || (org_idx - delta >= CCALF_CANDS_COEFF_NR * 2 - 1) )
          continue;

        filterCoeffInt[k] = forward_tab[org_idx - delta];
#if JVET_AH0057_CCALF_COEFF_PRECISION
        double error = m_alfCovarianceFrameCcAlf[0].calcErrorForCcAlfCoeffs(filterCoeffInt, size, (coeffPrec + 1));
#else
        double error = m_alfCovarianceFrameCcAlf[0].calcErrorForCcAlfCoeffs(filterCoeffInt, size, (m_scaleBits+1));
#endif
        if( error < errMin )
        {
          errMin = error;
          idxMin = k;
          minIndex = org_idx;
        }
        filterCoeffInt[k] = forward_tab[org_idx];
      }
      if (errMin < errRef)
      {
        minIndex -= delta;
        CHECK( minIndex < 0, "this is wrong, index - delta < 0");
        CHECK( minIndex >= CCALF_CANDS_COEFF_NR * 2 - 1, "this is wrong, index - delta >= CCALF_CANDS_COEFF_NR * 2 - 1");
        filterCoeffInt[idxMin] = forward_tab[minIndex];
        modified++;
        errRef = errMin;
      }
    }
  }

  for (int k = 0; k < (size + 1); k++)
  {
    CHECK((filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE)) || (filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE)), "Exceeded valid range for CC ALF coefficient");
    filterCoeff[filterIdx][k] = filterCoeffInt[k];
  }
}

void EncAdaptiveLoopFilter::determineControlIdcValues(CodingStructure &cs, const ComponentID compID, const PelBuf *buf,
                                                      const int ctuWidthC, const int ctuHeightC, const int picWidthC,
                                                      const int picHeightC, double **unfilteredDistortion,
                                                      uint64_t *trainingDistortion[MAX_NUM_CC_ALF_FILTERS],
                                                      uint64_t *lumaSwingGreaterThanThresholdCount,
                                                      uint64_t *chromaSampleCountNearMidPoint,
                                                      bool reuseTemporalFilterCoeff, uint8_t *trainingCovControl,
                                                      uint8_t *filterControl, uint64_t &curTotalDistortion,
                                                      double &curTotalRate, bool filterEnabled[MAX_NUM_CC_ALF_FILTERS],
                                                      uint8_t  mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1],
                                                      uint8_t &ccAlfFilterCount)
{
  bool curFilterEnabled[MAX_NUM_CC_ALF_FILTERS];
  std::fill_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, false);

#if MAX_NUM_CC_ALF_FILTERS>1
  FilterIdxCount filterIdxCount[MAX_NUM_CC_ALF_FILTERS];
  for (int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++)
  {
    filterIdxCount[i].count     = 0;
    filterIdxCount[i].filterIdx = i;
  }

  double prevRate = curTotalRate;
#endif

  TempCtx ctxInitial(m_ctxCache);
  TempCtx ctxBest(m_ctxCache);
  TempCtx ctxStart(m_ctxCache);
  ctxInitial = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  double chromaFactor = ( m_isLowDelayConfig && m_isLumaSignalNewAps && (getAvailableCcAlfApsIds(cs, compID).size() < 4)) ? m_chromaFactor : 1.00;
#endif

  int ctuIdx = 0;
  for (int yCtu = 0; yCtu < buf->height; yCtu += ctuHeightC)
  {
    for (int xCtu = 0; xCtu < buf->width; xCtu += ctuWidthC)
    {
      uint64_t ssd;
      double   rate;
      double   cost;

      uint64_t bestSSD       = MAX_UINT64;
      double   bestRate      = MAX_DOUBLE;
      double   bestCost      = MAX_DOUBLE;
      uint8_t  bestFilterIdc = 0;
      uint8_t  bestFilterIdx = 0;
      const uint32_t thresholdS = std::min<int>(buf->height - yCtu, ctuHeightC) << getComponentScaleY(COMPONENT_Cb, m_chromaFormat);
      const uint32_t numberOfChromaSamples = std::min<int>(buf->height - yCtu, ctuHeightC) * std::min<int>(buf->width - xCtu, ctuWidthC);
      const uint32_t thresholdC = (numberOfChromaSamples >> 2);

      m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag,ctxBest);
      ctxStart                   = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        uint8_t filterIdc = mapFilterIdxToFilterIdc[filterIdx];
        if (filterIdx < MAX_NUM_CC_ALF_FILTERS && !filterEnabled[filterIdx])
        {
          continue;
        }

        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          ssd = (uint64_t)unfilteredDistortion[compID][ctuIdx];   // restore saved distortion computation
        }
        else
        {
          ssd = trainingDistortion[filterIdx][ctuIdx];
        }
        m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag,ctxStart);
        m_CABACEstimator->resetBits();
        const Position lumaPos = Position({ xCtu << getComponentScaleX(compID, cs.pcv->chrFormat),
          yCtu << getComponentScaleY(compID, cs.pcv->chrFormat) });
        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdc, cs, compID, ctuIdx, filterControl, lumaPos,
                                                    ccAlfFilterCount);
        rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
        cost = rate * m_lambda[compID] * chromaFactor + ssd;
#else
        cost = rate * m_lambda[compID] + ssd;
#endif

        bool limitationExceeded = false;
        if (m_limitCcAlf && filterIdx < MAX_NUM_CC_ALF_FILTERS)
        {
          limitationExceeded = limitationExceeded || (lumaSwingGreaterThanThresholdCount[ctuIdx] >= thresholdS);
          limitationExceeded = limitationExceeded || (chromaSampleCountNearMidPoint[ctuIdx] >= thresholdC);
        }
        if (cost < bestCost && !limitationExceeded)
        {
          bestCost      = cost;
          bestRate      = rate;
          bestSSD       = ssd;
          bestFilterIdc = filterIdc;
          bestFilterIdx = filterIdx;

          ctxBest = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

          trainingCovControl[ctuIdx] = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
          filterControl[ctuIdx]      = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
        }
      }
      if (bestFilterIdc != 0)
      {
        curFilterEnabled[bestFilterIdx] = true;
#if MAX_NUM_CC_ALF_FILTERS>1
        filterIdxCount[bestFilterIdx].count++;
#endif
      }
      curTotalRate += bestRate;
      curTotalDistortion += bestSSD;
      ctuIdx++;
    }
  }

#if MAX_NUM_CC_ALF_FILTERS>1
  if (!reuseTemporalFilterCoeff)
  {
    std::copy_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, filterEnabled);

    std::stable_sort(filterIdxCount, filterIdxCount + MAX_NUM_CC_ALF_FILTERS, compareCounts);

    int filterIdc = 1;
    ccAlfFilterCount = 0;
    for ( FilterIdxCount &s : filterIdxCount )
    {
      const int filterIdx = s.filterIdx;
      if (filterEnabled[filterIdx])
      {
        mapFilterIdxToFilterIdc[filterIdx] = filterIdc;
        filterIdc++;
        ccAlfFilterCount++;
      }
    }

    curTotalRate = prevRate;
    m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag,ctxInitial);
    m_CABACEstimator->resetBits();
    int ctuIdx = 0;
    for (int y = 0; y < buf->height; y += ctuHeightC)
    {
      for (int x = 0; x < buf->width; x += ctuWidthC)
      {
        const int filterIdxPlus1 = filterControl[ctuIdx];

        const Position lumaPos = Position(
                                          { x << getComponentScaleX(compID, cs.pcv->chrFormat), y << getComponentScaleY(compID, cs.pcv->chrFormat) });

        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdxPlus1 == 0 ? 0
                                                    : mapFilterIdxToFilterIdc[filterIdxPlus1 - 1],
                                                    cs, compID, ctuIdx, filterControl, lumaPos, ccAlfFilterCount);

        ctuIdx++;
      }
    }
    curTotalRate += FRAC_BITS_SCALE*m_CABACEstimator->getEstFracBits();
  }
#endif

  // restore for next iteration
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag,ctxInitial);
}

std::vector<int> EncAdaptiveLoopFilter::getAvailableCcAlfApsIds(CodingStructure& cs, ComponentID compID)
{
  APS** apss = cs.slice->getAlfAPSs();
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  int apsIdChecked = 0, curApsId = m_apsIdStart;
  if (curApsId < ALF_CTB_MAX_NUM_APS)
  {
    while (apsIdChecked < ALF_CTB_MAX_NUM_APS && !cs.slice->isIntra() && result.size() < ALF_CTB_MAX_NUM_APS && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
    {
      APS* curAPS = cs.slice->getAlfAPSs()[curApsId];
      if (curAPS && curAPS->getLayerId() == cs.slice->getPic()->layerId
          && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getCcAlfAPSParam().newCcAlfFilter[compID - 1])
      {
        result.push_back(curApsId);
      }
      apsIdChecked++;
      curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
    }
  }
  return result;
}

void EncAdaptiveLoopFilter::getFrameStatsCcalf(ComponentID compIdx, int filterIdc)
{
        int ctuRsAddr = 0;

  // init Frame stats buffers
  for (int shape = 0; shape != m_filterShapesCcAlf.size(); shape++)
  {
    m_alfCovarianceFrameCcAlf[shape].reset();
  }

  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      if (m_trainingCovControl[ctuRsAddr] == filterIdc)
      {
        for (int shape = 0; shape != m_filterShapesCcAlf.size(); shape++)
        {
          m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::deriveCcAlfFilter( CodingStructure& cs, ComponentID compID, const PelUnitBuf& orgYuv, const PelUnitBuf& tempDecYuvBuf, const PelUnitBuf& dstYuv )
{
  if (!cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y))
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  m_limitCcAlf = m_encCfg->getBaseQP() >= m_encCfg->getCCALFQpThreshold();
  if (m_limitCcAlf && cs.slice->getSliceQp() <= m_encCfg->getBaseQP() + 1)
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  if (m_alfWSSD)
  {
    deriveStatsForCcAlfFiltering<true>(orgYuv, tempDecYuvBuf, compID, cs);
  }
  else
  {
    deriveStatsForCcAlfFiltering<false>(orgYuv, tempDecYuvBuf, compID, cs);
  }
  initDistortionCcalf(compID);
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  double chromaFactor = ( m_isLowDelayConfig && m_isLumaSignalNewAps && (getAvailableCcAlfApsIds(cs, compID).size() < 4)) ? m_chromaFactor : 1.00 ;
#endif

  uint8_t bestMapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS+1];
  const int scaleX               = getComponentScaleX(compID, cs.pcv->chrFormat);
  const int scaleY               = getComponentScaleY(compID, cs.pcv->chrFormat);
  const int ctuWidthC            = cs.pcv->maxCUWidth >> scaleX;
  const int ctuHeightC           = cs.pcv->maxCUHeight >> scaleY;
  const int picWidthC            = cs.pcv->lumaWidth >> scaleX;
  const int picHeightC           = cs.pcv->lumaHeight >> scaleY;
  const int maxTrainingIterCount = 15;

  if (m_limitCcAlf)
  {
    countLumaSwingGreaterThanThreshold(dstYuv.get(COMPONENT_Y).bufAt(0, 0), dstYuv.get(COMPONENT_Y).stride, dstYuv.get(COMPONENT_Y).height, dstYuv.get(COMPONENT_Y).width, cs.pcv->maxCUWidthLog2, cs.pcv->maxCUHeightLog2, m_lumaSwingGreaterThanThresholdCount, m_numCTUsInWidth);
  }
  if (m_limitCcAlf)
  {
    countChromaSampleValueNearMidPoint(dstYuv.get(compID).bufAt(0, 0), dstYuv.get(compID).stride, dstYuv.get(compID).height, dstYuv.get(compID).width, cs.pcv->maxCUWidthLog2 - scaleX, cs.pcv->maxCUHeightLog2 - scaleY, m_chromaSampleCountNearMidPoint, m_numCTUsInWidth);
  }

  for ( int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
  {
    if ( filterIdx < MAX_NUM_CC_ALF_FILTERS)
    {
      memset( m_bestFilterCoeffSet[filterIdx], 0, sizeof(m_bestFilterCoeffSet[filterIdx]) );
      bestMapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
    }
    else
    {
      bestMapFilterIdxToFilterIdc[filterIdx] = 0;
    }
  }
  memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  int ccalfReuseApsId      = -1;
  m_reuseApsId[compID - 1] = -1;
  m_bestFilterCount = 0;

  const TempCtx ctxStartCcAlfFilterControlFlag  ( m_ctxCache, SubCtx( Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );

  // compute cost of not filtering
  uint64_t unfilteredDistortion = 0;
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    unfilteredDistortion += (uint64_t)m_alfCovarianceCcAlf[0][ctbIdx].pixAcc;
  }

  double bestUnfilteredTotalCost = 1 * m_lambda[compID] + unfilteredDistortion;   // 1 bit is for gating flag

  bool             ccAlfFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
#if JVET_AH0057_CCALF_COEFF_PRECISION
  int              ccAlfCoeffPrec = MIN_CCALF_PREC;
  int              bestCoeffPrec = MIN_CCALF_PREC;
#endif
  short            ccAlfFilterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  uint8_t          ccAlfFilterCount             = MAX_NUM_CC_ALF_FILTERS;
  double bestFilteredTotalCost        = MAX_DOUBLE;
  bool   bestreuseTemporalFilterCoeff = false;
  std::vector<int> apsIds             = getAvailableCcAlfApsIds(cs, compID);

#if JVET_X0071_LONGER_CCALF
  for (int testFilterIdx = 0; testFilterIdx < (apsIds.size() + MAX_NUM_CC_ALF_FILTERS); testFilterIdx++)
#else
  for (int testFilterIdx = 0; testFilterIdx < ( apsIds.size() + 1 ); testFilterIdx++ )
#endif
  {
    bool referencingExistingAps   = (testFilterIdx < apsIds.size()) ? true : false;
    int maxNumberOfFiltersBeingTested = MAX_NUM_CC_ALF_FILTERS - (testFilterIdx - static_cast<int>(apsIds.size()));

    if (maxNumberOfFiltersBeingTested < 0)
    {
      maxNumberOfFiltersBeingTested = 1;
    }

	{
      // Instead of rewriting the control buffer for every training iteration just keep a mapping from filterIdx to filterIdc
      uint8_t mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1];
      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          mapFilterIdxToFilterIdc[filterIdx] = 0;
        }
        else
        {
          mapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
        }
      }

      // initialize filters
#if JVET_AH0057_CCALF_COEFF_PRECISION
      ccAlfCoeffPrec = MIN_CCALF_PREC;
#endif
      for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
      {
        ccAlfFilterIdxEnabled[filterIdx] = false;
        memset(ccAlfFilterCoeff[filterIdx], 0, sizeof(ccAlfFilterCoeff[filterIdx]));
      }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
      bool checkReuse = false;
#endif
      if ( referencingExistingAps )
      {
        maxNumberOfFiltersBeingTested = m_apsMap->getPS((apsIds[testFilterIdx] << NUM_APS_TYPE_LEN) + ALF_APS)->getCcAlfAPSParam().ccAlfFilterCount[compID - 1];
        ccAlfFilterCount = maxNumberOfFiltersBeingTested;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          ccAlfFilterIdxEnabled[filterIdx] = true;
          memcpy(ccAlfFilterCoeff[filterIdx], m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx],sizeof(ccAlfFilterCoeff[filterIdx]));
        }
        memcpy( ccAlfFilterCoeff, m_apsMap->getPS((apsIds[testFilterIdx] << NUM_APS_TYPE_LEN) + ALF_APS)->getCcAlfAPSParam().ccAlfCoeff[compID - 1], sizeof(ccAlfFilterCoeff) );
#if JVET_AH0057_CCALF_COEFF_PRECISION
        ccAlfCoeffPrec = m_apsMap->getPS((apsIds[testFilterIdx] << NUM_APS_TYPE_LEN) + ALF_APS)->getCcAlfAPSParam().ccAlfCoeffPrec[compID - 1];
#endif
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
        if( !cs.slice->isIntra() && g_pictureControlInfo[apsIds[testFilterIdx]].ccalfTemporalLayer[compID] <= cs.slice->getTLayer() )
        {
          checkReuse = true;
        }
#endif
      }
      else
      {
        for (int i = 0; i < maxNumberOfFiltersBeingTested; i++)
        {
          ccAlfFilterIdxEnabled[i] = true;
        }
        ccAlfFilterCount = maxNumberOfFiltersBeingTested;
      }

      // initialize
      int controlIdx = 0;
      const int columnSize = ( m_buf->width / maxNumberOfFiltersBeingTested);
      for (int y = 0; y < m_buf->height; y += ctuHeightC)
      {
        for (int x = 0; x < m_buf->width; x += ctuWidthC)
        {
          m_trainingCovControl[controlIdx] = ( x / columnSize ) + 1;
          controlIdx++;
        }
      }

#if JVET_AH0057_CCALF_COEFF_PRECISION
      const int coeffPrecNum = ( m_encCfg->getUseCCALFPrecision() && ( !referencingExistingAps ) )  ? MAX_NUM_CCALF_PREC : 1;
      for (int coeffPrecTest = MIN_CCALF_PREC; coeffPrecTest < coeffPrecNum + MIN_CCALF_PREC; coeffPrecTest++)
      {
#endif
      // compute cost of filtering
      int    trainingIterCount = 0;
      bool   keepTraining      = true;
      bool   improvement       = false;
      double prevTotalCost     = MAX_DOUBLE;
#if JVET_AH0057_CCALF_COEFF_PRECISION
      int coeffPrec = referencingExistingAps ? ccAlfCoeffPrec : coeffPrecTest;
#endif
      while (keepTraining)
      {
        improvement = false;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          if (ccAlfFilterIdxEnabled[filterIdx])
          {
            if (!referencingExistingAps)
            {
              getFrameStatsCcalf(compID, (filterIdx + 1));
#if JVET_AH0057_CCALF_COEFF_PRECISION
              deriveCcAlfFilterCoeff(compID, dstYuv, tempDecYuvBuf, ccAlfFilterCoeff, filterIdx, coeffPrec);
#else
              deriveCcAlfFilterCoeff(compID, dstYuv, tempDecYuvBuf, ccAlfFilterCoeff, filterIdx);
#endif
            }
            const int numCoeff  = m_filterShapesCcAlf[0].numCoeff - 1;
            int log2BlockWidth  = cs.pcv->maxCUWidthLog2 - scaleX;
            int log2BlockHeight = cs.pcv->maxCUHeightLog2 - scaleY;
            for (int y = 0; y < m_buf->height; y += (1 << log2BlockHeight))
            {
              for (int x = 0; x < m_buf->width; x += (1 << log2BlockWidth))
              {
                int ctuIdx = (y >> log2BlockHeight) * m_numCTUsInWidth + (x >> log2BlockWidth);
#if JVET_AH0057_CCALF_COEFF_PRECISION
                m_trainingDistortion[filterIdx][ctuIdx] = int(m_ctbDistortionUnfilter[compID][ctuIdx] + m_alfCovarianceCcAlf[0][ctuIdx].calcErrorForCcAlfCoeffs(ccAlfFilterCoeff[filterIdx], numCoeff, coeffPrec + 1));
#else
                m_trainingDistortion[filterIdx][ctuIdx] = int(m_ctbDistortionUnfilter[compID][ctuIdx] + m_alfCovarianceCcAlf[0][ctuIdx].calcErrorForCcAlfCoeffs(ccAlfFilterCoeff[filterIdx], numCoeff, m_scaleBits + 1));
#endif
              }
            }
          }
        }

        m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag,ctxStartCcAlfFilterControlFlag);

        uint64_t curTotalDistortion = 0;
        double curTotalRate = 0;
        determineControlIdcValues(cs, compID, m_buf, ctuWidthC, ctuHeightC, picWidthC, picHeightC,
                                  m_ctbDistortionUnfilter, m_trainingDistortion,
                                  m_lumaSwingGreaterThanThresholdCount,
                                  m_chromaSampleCountNearMidPoint,
                                  (referencingExistingAps == true),
                                  m_trainingCovControl, m_filterControl, curTotalDistortion, curTotalRate,
                                  ccAlfFilterIdxEnabled, mapFilterIdxToFilterIdc, ccAlfFilterCount);

        // compute coefficient coding bit cost
        if (ccAlfFilterCount > 0)
        {
          if (referencingExistingAps)
          {
            curTotalRate += 1 + 3; // +1 for enable flag, +3 APS ID in slice header
          }
          else
          {
#if JVET_AH0057_CCALF_COEFF_PRECISION
            curTotalRate += getCoeffRateCcAlf(ccAlfFilterCoeff, ccAlfFilterIdxEnabled, ccAlfFilterCount, compID) + 1
              + 9 + 2;
#else
            curTotalRate += getCoeffRateCcAlf(ccAlfFilterCoeff, ccAlfFilterIdxEnabled, ccAlfFilterCount, compID) + 1
            + 9;   // +1 for the enable flag, +9 3-bit for APS ID in slice header, 5-bit for APS ID in APS, a 1-bit
            // new filter flags (ignore shared cost such as other new-filter flags/NALU header/RBSP
            // terminating bit/byte alignment bits)
#endif
          }
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
          double curTotalCost = curTotalRate * m_lambda[compID] * chromaFactor + curTotalDistortion;
#else
          double curTotalCost = curTotalRate * m_lambda[compID] + curTotalDistortion;
#endif
          if (curTotalCost < prevTotalCost)
          {
            prevTotalCost = curTotalCost;
            improvement = true;
          }

          if (curTotalCost < bestFilteredTotalCost)
          {
            bestFilteredTotalCost = curTotalCost;
#if JVET_AH0057_CCALF_COEFF_PRECISION
            bestCoeffPrec = coeffPrec;
#endif
            memcpy(m_bestFilterIdxEnabled, ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
            memcpy(m_bestFilterCoeffSet, ccAlfFilterCoeff, sizeof(ccAlfFilterCoeff));
            memcpy(m_bestFilterControl, m_filterControl, sizeof(uint8_t) * m_numCTUsInPic);
            m_bestFilterCount = ccAlfFilterCount;
            ccalfReuseApsId = referencingExistingAps ? apsIds[testFilterIdx] : -1;
            memcpy(bestMapFilterIdxToFilterIdc, mapFilterIdxToFilterIdc, sizeof(mapFilterIdxToFilterIdc));
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
            cs.slice->setTileGroupCcalfReuseFlag(compID, false);
#endif
          }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
          if (checkReuse)
          {
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
            double reuseCost = 4 * m_lambda[compID] * chromaFactor;
#else
            double reuseCost = 4 * m_lambda[compID];
#endif
            for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
            {
              int filterControl = g_pictureControlInfo[apsIds[testFilterIdx]].bufCcalfFilter[compID][ctuIdx];
              m_filterControl[ctuIdx] = filterControl;
              if (filterControl == 0)
              {
                reuseCost += m_ctbDistortionUnfilter[compID][ctuIdx];
              }
              else
              {
                reuseCost += m_trainingDistortion[filterControl - 1][ctuIdx];
              }
            }
            if (reuseCost < prevTotalCost)
            {
              prevTotalCost = reuseCost;
              improvement = true;
            }
            if (reuseCost < bestFilteredTotalCost && (cs.picture->poc % 8 != 0 || m_reuseCondition))
            {
              bestFilteredTotalCost = reuseCost;
#if JVET_AH0057_CCALF_COEFF_PRECISION
              bestCoeffPrec = coeffPrec;
#endif
              memcpy(m_bestFilterIdxEnabled, ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
              memcpy(m_bestFilterCoeffSet, ccAlfFilterCoeff, sizeof(ccAlfFilterCoeff));
              memcpy(m_bestFilterControl, m_filterControl, sizeof(uint8_t) * m_numCTUsInPic);
              m_bestFilterCount = ccAlfFilterCount;
              ccalfReuseApsId = apsIds[testFilterIdx];
              memcpy(bestMapFilterIdxToFilterIdc, mapFilterIdxToFilterIdc, sizeof(mapFilterIdxToFilterIdc));
              cs.slice->setTileGroupCcalfReuseFlag(compID, true);
            }
          }
#endif
        }

        trainingIterCount++;
        if (!improvement || trainingIterCount > maxTrainingIterCount || referencingExistingAps)
        {
          keepTraining = false;
        }
      }
#if JVET_AH0057_CCALF_COEFF_PRECISION
      }
#endif
    }
  }

  if (bestUnfilteredTotalCost < bestFilteredTotalCost)
  {
    memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  }

  // save best coeff and control
  bool atleastOneBlockUndergoesFitlering = false;
  for (int controlIdx = 0; m_bestFilterCount > 0 && controlIdx < m_numCTUsInPic; controlIdx++)
  {
    if (m_bestFilterControl[controlIdx])
    {
      atleastOneBlockUndergoesFitlering = true;
      break;
    }
  }
  m_ccAlfFilterParam.numberValidComponents          = getNumberValidComponents(m_chromaFormat);
  m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = atleastOneBlockUndergoesFitlering;
  if (atleastOneBlockUndergoesFitlering)
  {
    // update the filter control indicators
    if (bestreuseTemporalFilterCoeff!=1)
    {
      short storedBestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
      for (int filterIdx=0; filterIdx<MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        memcpy(storedBestFilterCoeffSet[filterIdx], m_bestFilterCoeffSet[filterIdx], sizeof(m_bestFilterCoeffSet[filterIdx]));
      }
      memcpy(m_filterControl, m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);

      int filterCount = 0;
      for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
      {
        uint8_t curFilterIdc = bestMapFilterIdxToFilterIdc[filterIdx];
        if (m_bestFilterIdxEnabled[filterIdx])
        {
          for (int controlIdx = 0; controlIdx < m_numCTUsInPic; controlIdx++)
          {
            if (m_filterControl[controlIdx] == (filterIdx+1) )
            {
              m_bestFilterControl[controlIdx] = curFilterIdc;
            }
          }
          memcpy( m_bestFilterCoeffSet[curFilterIdc-1], storedBestFilterCoeffSet[filterIdx], sizeof(storedBestFilterCoeffSet[filterIdx]) );
          filterCount++;
        }
        m_bestFilterIdxEnabled[filterIdx] = ( filterIdx < m_bestFilterCount ) ? true : false;
      }
      CHECK( filterCount != m_bestFilterCount, "Number of filters enabled did not match the filter count");
    }

    m_ccAlfFilterParam.ccAlfFilterCount[compID - 1] = m_bestFilterCount;
#if JVET_AH0057_CCALF_COEFF_PRECISION
    m_ccAlfFilterParam.ccAlfCoeffPrec[compID - 1] = bestCoeffPrec;
#endif
    // cleanup before copying
    memset(m_ccAlfFilterControl[compID - 1], 0, sizeof(uint8_t) * m_numCTUsInPic);
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      memset(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], 0, sizeof(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx]));
    }
    memset(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1], false, sizeof(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1]));
    for ( int filterIdx = 0; filterIdx < m_bestFilterCount; filterIdx++ )
    {
      m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1][filterIdx] = m_bestFilterIdxEnabled[filterIdx];
      memcpy(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], m_bestFilterCoeffSet[filterIdx], sizeof(m_bestFilterCoeffSet[filterIdx]));
    }
    memcpy(m_ccAlfFilterControl[compID - 1], m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);
    if ( ccalfReuseApsId >= 0 )
    {
      m_reuseApsId[compID - 1] = ccalfReuseApsId;
      if (compID == COMPONENT_Cb)
      {
        cs.slice->setTileGroupCcAlfCbApsId(ccalfReuseApsId);
      }
      else
      {
        cs.slice->setTileGroupCcAlfCrApsId(ccalfReuseApsId);
      }
    }
#if JVET_AM0209_CHROMA_ALF_CCALF_REUSE_CTU
    if ((!cs.slice->getTileGroupCcalfReuseFlag(compID)))
    {
      int apsID = compID == COMPONENT_Cb ? cs.slice->getTileGroupCcAlfCbApsId() : cs.slice->getTileGroupCcAlfCrApsId();  
      {
        memcpy(g_pictureControlInfo[apsID].bufCcalfFilter[compID].data(), m_ccAlfFilterControl[compID - 1], sizeof(uint8_t)* m_numCTUsInPic);
        g_pictureControlInfo[apsID].ccalfTemporalLayer[compID] = cs.slice->getTLayer();
      }
    }
#endif
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilter::deriveStatsForCcAlfFiltering( const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const int compIdx, CodingStructure &cs )
{
  // init CTU stats buffers
  for( int shape = 0; shape != m_filterShapesCcAlf.size(); shape++ )
  {
    for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
    {
      m_alfCovarianceCcAlf[shape][ctuIdx].reset();
    }
  }

  // init Frame stats buffers
  for( int shape = 0; shape != m_filterShapesCcAlf.size(); shape++ )
  {
    m_alfCovarianceFrameCcAlf[shape].reset();
  }
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  PelUnitBuf resiYuv = m_tempBufResi.getBuf(UnitArea(CHROMA_400, Area(cs.area.blocks[COMPONENT_Y])));
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  PelUnitBuf recYuvSAO = m_tempBufSAO.getBuf(UnitArea(CHROMA_420, Area(cs.area.blocks[COMPONENT_Y])));
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
  PelUnitBuf recBeforeDbf = m_tempBufBeforeDb.getBuf( cs.area );
#endif
  int                  ctuRsAddr = 0;
  const PreCalcValues &pcv = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int                  numHorVirBndry = 0, numVerVirBndry = 0;
  int                  horVirBndryPos[] = { 0, 0, 0 };
  int                  verVirBndryPos[] = { 0, 0, 0 };

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
      int       rasterSliceAlfPad = 0;
      if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                        numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos,
                                        rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int  yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int  h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
          int        xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int  xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int  w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            const int  wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
            const int  hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
            PelUnitBuf recBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            recBuf.copyFrom( recYuv.subBuf(
              UnitArea( cs.area.chromaFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ),
                                                    yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, recBuf, MAX_ALF_PADDING_SIZE, true, true);
#else
            recBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
#endif
            recBuf = recBuf.subBuf( UnitArea(
              cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
            PelUnitBuf resiBuf = m_tempBufResi2.subBuf( UnitArea( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
            resiBuf.copyFrom( resiYuv.subBuf( UnitArea( CHROMA_400, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              resiBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }
            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              resiBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
            mirroredPaddingForAlf(cs, resiBuf, MAX_ALF_PADDING_SIZE, true, false);
            resiBuf = resiBuf.subBuf( UnitArea( CHROMA_400, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
            PelUnitBuf recBufSAO = m_tempBufSAO2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
            recBufSAO.copyFrom(recYuvSAO.subBuf(UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
            // pad top-left unavailable samples for raster slice
            if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
            {
              recBufSAO.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
            }
            // pad bottom-right unavailable samples for raster slice
            if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
            {
              recBufSAO.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, recBufSAO, MAX_ALF_PADDING_SIZE, false, true);
#else
            recBufSAO.extendBorderPel(MAX_ALF_PADDING_SIZE);
#endif
            recBufSAO = recBufSAO.subBuf(UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
            PelUnitBuf bufDbf = m_tempBufBeforeDb2.subBuf( UnitArea( cs.pcv->chrFormat, Area( 0, 0, wBuf, hBuf ) ) );
            bufDbf.copyFrom( recBeforeDbf.subBuf( UnitArea( cs.pcv->chrFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              bufDbf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }
            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              bufDbf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
            mirroredPaddingForAlf(cs, bufDbf, MAX_ALF_PADDING_SIZE, true, true);
            bufDbf = bufDbf.subBuf( UnitArea( cs.pcv->chrFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );
#endif

            const UnitArea area( m_chromaFormat, Area( 0, 0, w, h ) );
            const UnitArea areaDst( m_chromaFormat, Area( xStart, yStart, w, h ) );

            const ComponentID compID = ComponentID( compIdx );

            for( int shape = 0; shape != m_filterShapesCcAlf.size(); shape++ )
            {
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
              getBlkStatsCcAlf<alfWSSD>( m_alfCovarianceCcAlf[shape][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recBuf, areaDst, area, compID, yPos, resiBuf 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
                , recBufSAO
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
                , bufDbf
#endif
                );
#else
              getBlkStatsCcAlf<alfWSSD>(m_alfCovarianceCcAlf[shape][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recBuf, areaDst, area, compID, yPos
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
                , recBufSAO
#endif
                );
#endif
              m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
            }

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );

        const ComponentID compID = ComponentID( compIdx );

        for( int shape = 0; shape != m_filterShapesCcAlf.size(); shape++ )
        {
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
          getBlkStatsCcAlf<alfWSSD>( m_alfCovarianceCcAlf[shape][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recYuv, area, area, compID, yPos, resiYuv 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
            , recYuvSAO
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
            , recBeforeDbf
#endif
            );
#else
          getBlkStatsCcAlf<alfWSSD>(m_alfCovarianceCcAlf[0][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recYuv, area, area, compID, yPos
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
            , recYuvSAO
#endif
            );
#endif
          m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
      ctuRsAddr++;
    }
  }
}

template<bool m_alfWSSD>
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
void EncAdaptiveLoopFilter::getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape, const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area, const ComponentID compID, const int yPos, PelUnitBuf &resiYuv 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , PelUnitBuf& recYuvSAO
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
  , PelUnitBuf& recBeforeDbf
#endif
)
#else
void EncAdaptiveLoopFilter::getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape, const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv, const UnitArea &areaDst, const UnitArea &area, const ComponentID compID, const int yPos
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , PelUnitBuf& recYuvSAO
#endif
)
#endif
{
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
  const CompArea &compArea           = areaDst.block(compID);
  int  recStride[MAX_NUM_COMPONENT];
  const Pel* rec[MAX_NUM_COMPONENT];
  for ( int cIdx = 0; cIdx < numberOfComponents; cIdx++ )
  {
    recStride[cIdx] = recYuv.get(ComponentID(cIdx)).stride;
    rec[cIdx] = recYuv.get(ComponentID(cIdx)).bufAt(isLuma(ComponentID(cIdx)) ? area.lumaPos() : area.chromaPos());
  }

  int        orgStride = orgYuv.get(compID).stride;
  const Pel *org       = orgYuv.get(compID).bufAt(compArea);
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  int resiStride = resiYuv.get(COMPONENT_Y).stride;
  Pel* resiPtr   = resiYuv.get(COMPONENT_Y).bufAt( area.lumaPos());
  int scaleX = getComponentScaleX( compID, m_chromaFormat );
  int scaleY = getComponentScaleY( compID, m_chromaFormat );
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  int  recSAOStride = recYuvSAO.get(compID).stride;
  Pel* recSAOPtr    = recYuvSAO.get(compID).bufAt(area.chromaPos());
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
  const int recDbfCbStride = recBeforeDbf.get(COMPONENT_Cb).stride;
  const int recDbfCrStride = recBeforeDbf.get(COMPONENT_Cr).stride;
  Pel*  recDbfCbPtr = recBeforeDbf.get(COMPONENT_Cb).bufAt( area.chromaPos() );
  Pel*  recDbfCrPtr = recBeforeDbf.get(COMPONENT_Cr).bufAt( area.chromaPos() );

  const int crossChromaStride = compID == COMPONENT_Cb ? recDbfCrStride : recDbfCbStride;
  Pel* crossChromaPtr = compID == COMPONENT_Cb ? recDbfCrPtr : recDbfCbPtr;
#endif
  const int  numBins   = 1;

#if !ALF_IMPROVEMENT
  int vbCTUHeight = m_alfVBLumaCTUHeight;
  int vbPos       = m_alfVBLumaPos;
  if ((yPos + m_maxCUHeight) >= m_picHeight)
  {
    vbPos = m_picHeight;
  }
#endif

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1];
#else
  int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1];
#endif

  for (int i = 0; i < compArea.height; i++)
  {
#if !ALF_IMPROVEMENT
    int vbDistance = ((i << getComponentScaleY(compID, m_chromaFormat)) % vbCTUHeight) - vbPos;
    const bool skipThisRow = getComponentScaleY(compID, m_chromaFormat) == 0 && (vbDistance == 0 || vbDistance == 1);
    for (int j = 0; j < compArea.width && (!skipThisRow); j++)
#else
    for (int j = 0; j < compArea.width; j++)
#endif
    {
      std::memset(ELocal, 0, sizeof(ELocal));
#if JVET_AF0177_ALF_COV_FLOAT
      float weight = 1.0;
      if (m_alfWSSD)
      {
        weight = (float)m_lumaLevelToWeightPLUT[org[j]];
      }
#else
      double weight = 1.0;
      if (m_alfWSSD)
      {
        weight = m_lumaLevelToWeightPLUT[org[j]];
      }
#endif

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      Intermediate_Int yLocal = org[j] - rec[compID][j];
#else
      int yLocal = org[j] - rec[compID][j];
#endif

#if ALF_IMPROVEMENT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
      calcCovarianceCcAlf( ELocal, rec[COMPONENT_Y] + ( j << scaleX ), recStride[COMPONENT_Y], shape, resiPtr + (j << scaleX), resiStride 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
        , recSAOPtr + j, recSAOStride
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
        , crossChromaPtr + j, crossChromaStride
#endif
      );
#else
      calcCovarianceCcAlf( ELocal, rec[COMPONENT_Y] + ( j << getComponentScaleX( compID, m_chromaFormat ) ), recStride[COMPONENT_Y], shape 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
        , recSAOPtr + j, recSAOStride
#endif
      );
#endif
#else
      calcCovarianceCcAlf( ELocal, rec[COMPONENT_Y] + ( j << getComponentScaleX(compID, m_chromaFormat)), recStride[COMPONENT_Y], shape, vbDistance );
#endif

      for( int k = 0; k < (shape.numCoeff - 1); k++ )
      {
        for( int l = 0; l <= k; l++ )
        {
          for( int b0 = 0; b0 < numBins; b0++ )
          {
            for (int b1 = 0; b1 <= b0; b1++)
            {
              if (m_alfWSSD)
              {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
                alfCovariance.E(b0, b1, k, l) += weight * (ELocal[k][b0] * (float)ELocal[l][b1]);
#else
                alfCovariance.E(b0,b1,k,l) += weight * (ELocal[k][b0] * (double)ELocal[l][b1]);
#endif
#else
                alfCovariance.E(b0,b1,k,l) += weight * (double) (ELocal[k][b0] * ELocal[l][b1]);
#endif
              }
              else
              {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
                alfCovariance.E(b0, b1, k, l) += ELocal[k][b0] * (float)ELocal[l][b1];
#else
                alfCovariance.E(b0,b1,k,l) += ELocal[k][b0] * (double)ELocal[l][b1];
#endif
#else
                alfCovariance.E(b0,b1,k,l) += ELocal[k][b0] * ELocal[l][b1];
#endif
              }
            }
          }
        }
        for (int b = 0; b < numBins; b++)
        {
          if (m_alfWSSD)
          {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
            alfCovariance.y(b,k) += weight * (ELocal[k][b] * (float)yLocal);
#else
            alfCovariance.y(b,k) += weight * (ELocal[k][b] * (double)yLocal);
#endif
#else
            alfCovariance.y[b][k] += weight * (double) (ELocal[k][b] * yLocal);
#endif
          }
          else
          {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
            alfCovariance.y(b,k) += ELocal[k][b] * (float)yLocal;
#else
            alfCovariance.y(b,k) += ELocal[k][b] * (double)yLocal;
#endif
#else
            alfCovariance.y[b][k] += ELocal[k][b] * yLocal;
#endif
          }
        }
      }
      if (m_alfWSSD)
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
        alfCovariance.pixAcc += weight * (yLocal * (float)yLocal);
#else
        alfCovariance.pixAcc += weight * (yLocal * (double)yLocal);
#endif
#else
        alfCovariance.pixAcc += weight * (double) (yLocal * yLocal);
#endif
      }
      else
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if JVET_AF0177_ALF_COV_FLOAT
        alfCovariance.pixAcc += yLocal * (float)yLocal;
#else
        alfCovariance.pixAcc += yLocal * (double)yLocal;
#endif
#else
        alfCovariance.pixAcc += yLocal * yLocal;
#endif
      }
    }
    org += orgStride;
    for (int srcCIdx = 0; srcCIdx < numberOfComponents; srcCIdx++)
    {
      ComponentID srcCompID = ComponentID(srcCIdx);
      if (toChannelType(srcCompID) == toChannelType(compID))
      {
        rec[srcCIdx] += recStride[srcCIdx];
      }
      else
      {
        if (isLuma(compID))
        {
          rec[srcCIdx] += (recStride[srcCIdx] >> getComponentScaleY(srcCompID, m_chromaFormat));
        }
        else
        {
          rec[srcCIdx] += (recStride[srcCIdx] << getComponentScaleY(compID, m_chromaFormat));
        }
      }
    }
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
    resiPtr += resiStride << scaleY;
#endif
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    recSAOPtr += recSAOStride;
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    crossChromaPtr += crossChromaStride;
#endif
  }
}
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if ALF_IMPROVEMENT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
void EncAdaptiveLoopFilter::calcCovarianceCcAlf( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape, Pel* resiPtr, int resiStride 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , Pel* recSAO, int saoStride
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
  , Pel* crossChromaPtr, const int crossChromaStride
#endif
)
#else
void EncAdaptiveLoopFilter::calcCovarianceCcAlf( Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , Pel* recSAO, int saoStride
#endif
)
#endif
#else
void EncAdaptiveLoopFilter::calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape, int vbDistance)
#endif
#else
#if ALF_IMPROVEMENT
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
void EncAdaptiveLoopFilter::calcCovarianceCcAlf( int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape, Pel* resiPtr, int resiStride 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , Pel* recSAO, int saoStride
#endif
)
#else
void EncAdaptiveLoopFilter::calcCovarianceCcAlf( int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape 
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
  , Pel* recSAO, int saoStride
#endif
)
#endif
#else
void EncAdaptiveLoopFilter::calcCovarianceCcAlf(int ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const int stride, const AlfFilterShape& shape, int vbDistance)
#endif
#endif
{
  CHECK(shape.filterType != CC_ALF, "Bad CC ALF shape");
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  bool isHighRes = m_picWidth > 1280 && m_picHeight > 720 ? true : false;
  Pel  clipValue = m_alfClippingValues[CHANNEL_TYPE_LUMA][1];
#endif

  const Pel *recYM1 = rec - 1 * stride;
  const Pel *recY0  = rec;
  const Pel *recYP1 = rec + 1 * stride;
  const Pel *recYP2 = rec + 2 * stride;
#if JVET_X0071_LONGER_CCALF
  const Pel *recYM2 = rec - 2 * stride;
  const Pel *recYM3 = rec - 3 * stride;
  const Pel *recYP3 = rec + 3 * stride;
  const Pel *recYM4 = rec - 4 * stride;
  const Pel *recYP4 = rec + 4 * stride;
#endif

#if !ALF_IMPROVEMENT
  if (vbDistance == -2 || vbDistance == +1)
  {
    recYP2 = recYP1;
  }
  else if (vbDistance == -1 || vbDistance == 0)
  {
    recYM1 = recY0;
    recYP2 = recYP1 = recY0;
  }
#endif

  for (int b = 0; b < 1; b++)
  {
    const Pel centerValue = recY0[+0];
#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    const Pel centerValueSAO = recSAO[+0];
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    const Pel centerValueCrossChroma = crossChromaPtr[+0];
#endif

#if JVET_X0071_LONGER_CCALF
#if !JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
    ELocal[0][b] += recYM4[+0] - centerValue;
    ELocal[1][b] += recYM3[+0] - centerValue;
    ELocal[2][b] += recYM2[+0] - centerValue;
    ELocal[3][b] += recYM1[+0] - centerValue;

    ELocal[4][b] += recY0[-4] - centerValue;
    ELocal[5][b] += recY0[-3] - centerValue;
    ELocal[6][b] += recY0[-2] - centerValue;
    ELocal[7][b] += recY0[-1] - centerValue;
    ELocal[8][b] += recY0[+1] - centerValue;
    ELocal[9][b] += recY0[+2] - centerValue;
    ELocal[10][b] += recY0[+3] - centerValue;
    ELocal[11][b] += recY0[+4] - centerValue;

    ELocal[12][b] += recYP1[-4] - centerValue;
    ELocal[13][b] += recYP1[-3] - centerValue;
    ELocal[14][b] += recYP1[-2] - centerValue;
    ELocal[15][b] += recYP1[-1] - centerValue;
    ELocal[16][b] += recYP1[+0] - centerValue;
    ELocal[17][b] += recYP1[+1] - centerValue;
    ELocal[18][b] += recYP1[+2] - centerValue;
    ELocal[19][b] += recYP1[+3] - centerValue;
    ELocal[20][b] += recYP1[+4] - centerValue;

    ELocal[21][b] += recYP2[+0] - centerValue;
    ELocal[22][b] += recYP3[+0] - centerValue;
    ELocal[23][b] += recYP4[+0] - centerValue;

#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    ELocal[24][b] += (recSAO[-1 * saoStride + 0] - centerValueSAO);
    ELocal[25][b] += (recSAO[+0 * saoStride - 1] - centerValueSAO);
    ELocal[26][b] += (recSAO[+0 * saoStride + 1] - centerValueSAO);
    ELocal[27][b] += (recSAO[+1 * saoStride + 0] - centerValueSAO);
#endif
#endif
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
    ELocal[0][b] += recYM4[+0] - centerValue + recYP4[+0] - centerValue;
    ELocal[1][b] += recYM3[+0] - centerValue;
    ELocal[2][b] += recYM2[+0] - centerValue;
    ELocal[3][b] += recYM1[+0] - centerValue;

    ELocal[4][b] += recY0[-4] - centerValue;
    ELocal[5][b] += recY0[-3] - centerValue;
    ELocal[6][b] += recY0[-2] - centerValue;
    ELocal[7][b] += recY0[-1] - centerValue;
    ELocal[8][b] += recY0[+1] - centerValue;
    ELocal[9][b] += recY0[+2] - centerValue;
    ELocal[10][b] += recY0[+3] - centerValue;
    ELocal[11][b] += recY0[+4] - centerValue;

    ELocal[12][b] += recYP1[-4] - centerValue + recYP1[+4] - centerValue;
    ELocal[13][b] += recYP1[-3] - centerValue;
    ELocal[14][b] += recYP1[-2] - centerValue;
    ELocal[15][b] += recYP1[-1] - centerValue;
    ELocal[16][b] += recYP1[+0] - centerValue;
    ELocal[17][b] += recYP1[+1] - centerValue;
    ELocal[18][b] += recYP1[+2] - centerValue;
    ELocal[19][b] += recYP1[+3] - centerValue;

    ELocal[20][b] += recYP2[+0] - centerValue;
    ELocal[21][b] += recYP3[+0] - centerValue;

    ELocal[22][b] += (isHighRes ) ? clipALF(clipValue, 0, resiPtr[-1 * resiStride + 0]) : 0;
    ELocal[23][b] += (isHighRes ) ? clipALF(clipValue, 0, resiPtr[+0 * resiStride - 1]) : 0;
    ELocal[24][b] +=                clipALF(clipValue, 0, resiPtr[+0 * resiStride + 0])    ;
    ELocal[25][b] += (isHighRes ) ? clipALF(clipValue, 0, resiPtr[+0 * resiStride + 1]) : 0;
    ELocal[26][b] += (isHighRes ) ? clipALF(clipValue, 0, resiPtr[+1 * resiStride + 0]) : 0;

#if JVET_AI0166_CCALF_CHROMA_SAO_INPUT
    ELocal[27][b] += (recSAO[-1 * saoStride + 0] - centerValueSAO);
    ELocal[28][b] += (recSAO[+0 * saoStride - 1] - centerValueSAO);
    ELocal[29][b] += (recSAO[+0 * saoStride + 1] - centerValueSAO);
    ELocal[30][b] += (recSAO[+1 * saoStride + 0] - centerValueSAO);
#endif
#if JVET_AL0135_CCALF_CROSS_CHROMA_INPUT
    ELocal[31][b] += (crossChromaPtr[-1 * crossChromaStride + 0] + crossChromaPtr[+1 * crossChromaStride + 0] - centerValueCrossChroma * 2 );
    ELocal[32][b] += (crossChromaPtr[+0 * crossChromaStride - 1] + crossChromaPtr[+0 * crossChromaStride + 1] - centerValueCrossChroma * 2 );
#endif
#endif
#else
    ELocal[0][b] += recYM1[+0] - centerValue;
    ELocal[1][b] += recY0[-1] - centerValue;
    ELocal[2][b] += recY0[+1] - centerValue;
    ELocal[3][b] += recYP1[-1] - centerValue;
    ELocal[4][b] += recYP1[+0] - centerValue;
    ELocal[5][b] += recYP1[+1] - centerValue;
    ELocal[6][b] += recYP2[+0] - centerValue;

#endif

  }
}

void EncAdaptiveLoopFilter::countLumaSwingGreaterThanThreshold(const Pel* luma, int lumaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* lumaSwingGreaterThanThresholdCount, int lumaCountStride)
{
  const int lumaBitDepth = m_inputBitDepth[CH_L];
  const int threshold = (1 << ( m_inputBitDepth[CH_L] - 2 )) - 1;
#if JVET_X0071_LONGER_CCALF
  int xSupport[] = { 0,  0,  0,  0,  -4, -3,  -2, -1, 0, +1, +2, +3, +4,    -4,  -3,   -2,  -1, 0,   +1,  +2,  +3, +4,  0,  0, 0 };
  int ySupport[] = { -4, -3, -2, -1,   0,  0,   0,  0, 0,  0,  0,  0, 0,    +1,  +1,   +1,  +1, +1,  +1,  +1,  +1, +1, +2, +3, +4 };
#else
  // 3x4 Diamond
  int xSupport[] = {  0, -1, 0, 1, -1, 0, 1, 0 };
  int ySupport[] = { -1,  0, 0, 0,  1, 1, 1, 2 };
#endif
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
  int numSpatialTap = JVET_X0071_LONGER_CCALF ? 25 : 8;
#endif

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
#if JVET_X0071_LONGER_CCALF
          if ((y + yOff) >= (height - 4) || (x + xOff) >= (width - 4) || (y + yOff) < 4 || (x + xOff) < 4) // only consider samples that are fully supported by picture
#else
          if ((y + yOff) >= (height - 2) || (x + xOff) >= (width - 1) || (y + yOff) < 1 || (x + xOff) < 1) // only consider samples that are fully supported by picture
#endif
          {
            continue;
          }

          int minVal = ((1 << lumaBitDepth) - 1);
          int maxVal = 0;
#if JVET_X0071_LONGER_CCALF
#if JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF
          for (int i = 0; i < numSpatialTap; i++)
#else
          for (int i = 0; i < MAX_NUM_CC_ALF_CHROMA_COEFF; i++)
#endif
#else
          for (int i = 0; i < 8; i++)
#endif
          {
            Pel p = luma[(yOff + ySupport[i]) * lumaStride + x + xOff + xSupport[i]];

            if ( p < minVal )
            {
              minVal = p;
            }
            if ( p > maxVal )
            {
              maxVal = p;
            }
          }

          if ((maxVal - minVal) > threshold)
          {
            lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    luma += (lumaStride << log2BlockHeight);
  }
}

void EncAdaptiveLoopFilter::countChromaSampleValueNearMidPoint(const Pel* chroma, int chromaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* chromaSampleCountNearMidPoint, int chromaSampleCountNearMidPointStride)
{
  const int midPoint  = (1 << m_inputBitDepth[CH_C]) >> 1;
#if JVET_AJ0237_INTERNAL_12BIT
  const int threshold = 16 << std::max(0, m_inputBitDepth[CH_C] - 10);
#else
  const int threshold = 16;
#endif

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      chromaSampleCountNearMidPoint[(y >> log2BlockHeight)* chromaSampleCountNearMidPointStride + (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
          if ((y + yOff) >= height || (x + xOff) >= width)
          {
            continue;
          }

          int distanceToMidPoint = abs(chroma[yOff * chromaStride + x + xOff] - midPoint);
          if (distanceToMidPoint < threshold)
          {
            chromaSampleCountNearMidPoint[(y >> log2BlockHeight)* chromaSampleCountNearMidPointStride + (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    chroma += (chromaStride << log2BlockHeight);
  }
}

#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
bool EncAdaptiveLoopFilter::calcOffsetRefinementOnOff(CodingStructure& cs, PelUnitBuf& src0, PelUnitBuf& src1, PelUnitBuf& src2, int& refineIdx )
{
  const PreCalcValues& pcv = *cs.pcv;

  PelUnitBuf org = cs.getTrueOrgBuf();

  double dist0 = 0.0, dist1 = 0.0, dist2 = 0.0;

  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      Area blk(xPos, yPos, width, height);

      const int src0Stride = src0.get(COMPONENT_Y).stride;
      const int src1Stride = src1.get(COMPONENT_Y).stride;
      const int src2Stride = src2.get(COMPONENT_Y).stride;

      const int orgStride = org.get(COMPONENT_Y).stride;
      Pel* src0Ptr = src0.get(COMPONENT_Y).buf + blk.y * src0Stride + blk.x;
      Pel* src1Ptr = src1.get(COMPONENT_Y).buf + blk.y * src1Stride + blk.x;
      Pel* src2Ptr = src2.get(COMPONENT_Y).buf + blk.y * src2Stride + blk.x;

      Pel* orgPtr = org.get(COMPONENT_Y).buf + blk.y * orgStride + blk.x;

      for(int y = 0; y < blk.height; y++)
      {
        for(int x = 0; x < blk.width; x++)
        {
          int diff0 = orgPtr[x] - src0Ptr[x];
          int diff1 = orgPtr[x] - src1Ptr[x];
          int diff2 = orgPtr[x] - src2Ptr[x];

          dist0 += diff0 * diff0;
          dist1 += diff1 * diff1;
          dist2 += diff2 * diff2;
        }
        src0Ptr += src0Stride;
        src1Ptr += src1Stride;
        src2Ptr += src2Stride;
        orgPtr += orgStride;
      }
    }
  }

  //src0 = disable, src1 = enable index 0, src2 = enable index 1

  double lambda = cs.slice->getLambdas()[COMPONENT_Y];
  dist0 += 1 * lambda;
  dist1 += 2 * lambda;
  dist2 += 2 * lambda;
  refineIdx = dist1 <= dist2 ? 0 : 1;
  double distMinEnable = refineIdx == 0 ? dist1 : dist2;
  bool enableOffsetRefinement = distMinEnable < dist0 ? true : false;

  return enableOffsetRefinement;
}
#endif
#if JVET_AK0065_TALF
std::vector<std::vector<int>> getCombMap(int k, std::vector<int>& apsIds)
{
  int n = (int)apsIds.size();
  std::string bitmask(k, 1);
  bitmask.resize(n, 0);
  std::vector<std::vector<int>> combMap;

  do
  {
    std::vector<int> comb;
    for (int i = 0; i < n; ++i)
    {
      if (bitmask[i])
      {
        comb.push_back(apsIds[i]);
      }
    }
    combMap.push_back(comb);
  }
  while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  return combMap;
}

void EncAdaptiveLoopFilter::determineCtuFlag(CodingStructure &cs, const ComponentID compId, double **unfilteredDistortion
  , uint64_t *trainingDistortion[MAX_NUM_TALF_FILTERS], std::vector<TAlfCtbParam>& filterControl
  , uint64_t &curTotalDist, double &curTotalRate, const int filterCount, uint8_t* trainingControl)
{
  TempCtx ctxInitial(m_ctxCache);
  TempCtx ctxBest(m_ctxCache);
  TempCtx ctxStart(m_ctxCache);
  ctxInitial = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
  ctxBest = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());

  int ctbIdx = 0;
  for (int yCtu = 0; yCtu < m_picHeight; yCtu += m_maxCUHeight)
  {
    for (int xCtu = 0; xCtu < m_picWidth; xCtu += m_maxCUWidth)
    {
      uint64_t ctbDist      = MAX_UINT64;
      double   ctbRate      = MAX_DOUBLE;
      double   ctbCost      = MAX_DOUBLE;
      m_CABACEstimator->getCtx() = ctxBest;
      ctxStart                   = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
      for (int filterIdc = 0; filterIdc <= filterCount; filterIdc++)
      {
        TAlfCtbParam curControl;
        curControl.enabledFlag = (filterIdc != 0);
        curControl.filterIdx = curControl.enabledFlag ? (filterIdc - 1) : 0;
        uint64_t dist = curControl.enabledFlag ? uint64_t(trainingDistortion[curControl.filterIdx][ctbIdx])
            : uint64_t(unfilteredDistortion[compId][ctbIdx]);
        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();
        const Position lumaPos( xCtu, yCtu );
        m_CABACEstimator->codeTAlfFilterControlIdc(curControl, cs, compId, ctbIdx, filterControl.data(), lumaPos, filterCount, 1, true);
        double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        double cost = rate * m_lambda[COMPONENT_Y] + dist;

        if ( cost < ctbCost )
        {
          ctbCost = cost;
          ctbRate = rate;
          ctbDist = dist;
          ctxBest = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
          filterControl[ctbIdx]   = curControl;
          trainingControl[ctbIdx] = filterIdc;
        }
      }
      curTotalDist += ctbDist;
      curTotalRate += ctbRate;
      ctbIdx++;
    } // ctu x address
  } // ctu y address

  m_CABACEstimator->getCtx() = ctxInitial;
}

bool EncAdaptiveLoopFilter::determineCtuFlagReuse(CodingStructure &cs, const ComponentID compID, double **unfilteredDistortion
  , uint64_t *trainingDistortion[ALF_CTB_MAX_NUM_APS][MAX_NUM_TALF_FILTERS], TAlfCtbParam *filterControl
  , std::vector<int>& apsIds, std::vector<TAlfFilterParam>& params, uint64_t &curTotalDistortion, double &curTotalRate)
{
  const int ctuWidth              = cs.pcv->maxCUWidth;
  const int ctuHeight             = cs.pcv->maxCUHeight;
  const int picWidth              = cs.pcv->lumaWidth;
  const int picHeight             = cs.pcv->lumaHeight;
  TempCtx ctxInitial(m_ctxCache);
  TempCtx ctxBest(m_ctxCache);
  TempCtx ctxStart(m_ctxCache);
  ctxInitial = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
  const int numSets = int(params.size());
  // estimate on/off on each CTB
  if (numSets > 1)
  {
    std::vector<ParamApsIdAndCount> paramApsIdAndCount;
    for(int setId = 0; setId < numSets; setId++)
    {
      ParamApsIdAndCount p;
      p.param = params[setId];
      p.apsId = apsIds[setId];
      p.count = 0;
      paramApsIdAndCount.push_back(p);
    }
    for (uint32_t ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      double   bestCost      = MAX_DOUBLE;
      uint32_t bestSetId     = MAX_UINT;
      for(uint32_t setId = 0; setId < numSets; setId++)
      {
        const int filterCount = params[setId].filterCount;
        const int apsId = apsIds[setId];
        for (int filterIdc =  1; filterIdc <= filterCount; filterIdc++)
        {
          int filterIdx = filterIdc - 1;
          double cost = std::min(filterIdc, filterCount - 1) * m_lambda[COMPONENT_Y]
            + double(trainingDistortion[apsId][filterIdx][ctbIdx]);
          if ( cost < bestCost )
          {
            bestCost = cost;
            bestSetId = setId;
          }
        }
      }
      paramApsIdAndCount[bestSetId].count++;
    }
    std::stable_sort(paramApsIdAndCount.begin(), paramApsIdAndCount.end(), compareCountsTALF);
    params.clear();
    apsIds.clear();
    for(auto p: paramApsIdAndCount)
    {
      if (p.count > 0)
      {
        params.push_back(p.param);
        apsIds.push_back(p.apsId);
      }
    }
  }

  // early termination because the combination is tested.
  if (int(params.size()) != numSets)
  {
    return false;
  }

  bool anyCtbOn = false;
  int ctbIdx = 0;
  for (int yCtu = 0; yCtu < picHeight; yCtu += ctuHeight)
  {
    for (int xCtu = 0; xCtu < picWidth; xCtu += ctuWidth)
    {
      double   bestRate = MAX_DOUBLE;
      uint64_t bestDist = MAX_UINT64;
      double   bestCost = MAX_DOUBLE;
      m_CABACEstimator->getCtx() = ctxBest;
      ctxStart                   = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
      for (int setId = 0; setId < numSets; setId++)
      {
        const TAlfFilterParam param = params[setId];
        const int       filterCount = param.filterCount;
        for (int filterIdc = 0; filterIdc <= filterCount; filterIdc++)
        {
          if (setId > 0 && filterIdc == 0)
          {
            continue;
          }
          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();
          const Position lumaPos( xCtu, yCtu );
          TAlfCtbParam curControl;
          curControl.enabledFlag = (filterIdc != 0);
          curControl.filterIdx   = curControl.enabledFlag ? (filterIdc - 1) : 0;
          curControl.setIdx      = setId;
          m_CABACEstimator->codeTAlfFilterControlIdc(curControl, cs, compID, ctbIdx, filterControl, lumaPos, filterCount, numSets, false);
          double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          uint64_t dist = curControl.enabledFlag ? uint64_t(trainingDistortion[apsIds[setId]][curControl.filterIdx][ctbIdx]) : uint64_t(unfilteredDistortion[compID][ctbIdx]);
          double cost = rate * m_lambda[COMPONENT_Y] + dist;
          if ( cost < bestCost )
          {
            bestCost = cost;
            bestDist = dist;
            bestRate = rate;
            ctxBest = SubCtx(Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx());
            filterControl[ctbIdx] = curControl;
          }
        }
      }
      anyCtbOn |= filterControl[ctbIdx].enabledFlag;
      curTotalDistortion += bestDist;
      curTotalRate       += bestRate;
      ctbIdx++;
    }
  }
  m_CABACEstimator->getCtx() = ctxInitial;

  return anyCtbOn;
}

CandTALF EncAdaptiveLoopFilter::deriveTAlfNewFilter(CodingStructure &cs, ComponentID compId, const int mode, const int shapeIdx)
{
  const TempCtx ctxStartTAlfFilterControlFlag ( m_ctxCache, SubCtx( Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );
  const int nonLinearFlagMax = m_encCfg->getUseNonLinearAlfLuma() ? 2 : 1;

  const int numCoeff         = NUM_TALF_COEFF + 1;
  AlfFilterShape alfFilterShape(size_TALF - numCoeff);
  int newApsId = -1;
  std::vector<int> apsIds = getAvailableTAlfApsIds(cs, newApsId);

  double   globalMinCost = MAX_DOUBLE;
  CandTALF bestTalfCand;

  TAlfFilterParam talfParam;
  talfParam.newFlag  = true;
  talfParam.shapeIdx = shapeIdx;
  std::vector<TAlfCtbParam> filtControl;
  filtControl.resize(m_numCTUsInPic);
  std::vector<TAlfCtbParam> tmpBestControl;
  tmpBestControl.resize(m_numCTUsInPic);
  std::vector<TAlfCtbParam> bestControl;
  bestControl.resize(m_numCTUsInPic);
  for (int filterCount = 1; filterCount <= MAX_NUM_TALF_FILTERS; filterCount++)
  {
    const int filterIdx = filterCount - 1;
    double bestCostCurFilter = MAX_DOUBLE;
    for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
    {
      for(int shift = TALF_SCALE_BIT; shift < TALF_SCALE_BIT + 4; shift++)
      {
        int filterCoeff  [MAX_NUM_ALF_LUMA_COEFF] = { 0 };
        int filterClipIdx[MAX_NUM_ALF_LUMA_COEFF] = { 0 };
        // initialize training state.
        const int maxTrainingIters = 15;
        double prevIterCost = MAX_DOUBLE;
        for(int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          m_trainingCovControl[ctbIdx] = tmpBestControl[ctbIdx].enabledFlag ? (MAX_NUM_TALF_FILTERS + 1) : filterCount;
        }
        for(int iter = 0; iter < maxTrainingIters; iter++)
        {
          AlfCovariance* c  = m_alfCovarianceTALF[shapeIdx][mode];
          AlfCovariance& cf = m_alfCovarianceFrameTALF[shapeIdx][mode];
          getFrameStatsTAlf(filterCount, c, cf);
          deriveCoeffTAlf(compId, alfFilterShape, cf, filterCoeff, filterClipIdx, nonLinearFlag, shift + 1);
          for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
          {
            m_talfDistortion[0][filterIdx][ctbIdx] =
              uint64_t(c[ctbIdx].pixAcc + c[ctbIdx].calcErrorForTAlfCoeffs(filterClipIdx, filterCoeff, numCoeff, shift + 1));
          }
          m_CABACEstimator->getCtx() = ctxStartTAlfFilterControlFlag;
          uint64_t curTotalDistortion = 0;
          double   curTotalRate       = 0;
          double   curTotalCost       = 0;
          // rate for ctbs and distortions
          determineCtuFlag(cs, compId, m_ctbDistortionUnfilter, m_talfDistortion[0], filtControl
            , curTotalDistortion, curTotalRate, filterCount, m_trainingCovControl);

          // rate for new filter coefficients
          curTotalRate += getSetTAlfRate(filterCount, talfParam.coeff, filterCoeff, talfParam.clipFlag, nonLinearFlag);
          // rate for slice level controls
          curTotalRate += 1             /* sh/ph enabledFlag */
                        + (isBiTAlf(mode) ? 2 : 3) /* bits for mode */
                        + 1             /* bit for newFilter Flag */
                        + 3;            /* sh/ph APS ID */
          curTotalRate += 5             /* APS ID set for APS */
                        + 3;            /* APS Type */

          curTotalCost = curTotalRate * m_lambda[compId] + curTotalDistortion;
          // update current filterIdx best.
          if (curTotalCost < bestCostCurFilter)
          {
            bestCostCurFilter             = curTotalCost;
            talfParam.filterCount         = filterCount;
            talfParam.clipFlag[filterIdx] = nonLinearFlag;
            talfParam.shift   [filterIdx] = shift;
            memcpy(talfParam.coeff[filterIdx], filterCoeff, sizeof(filterCoeff));
            memcpy(talfParam.clipIdx[filterIdx], filterClipIdx, sizeof(filterClipIdx));
            bestControl = filtControl;
          }
          // update overall best
          if (curTotalCost < globalMinCost)
          {
            CandTALF cand;
            cand.rate                    = curTotalRate;
            cand.talfControl.enabledFlag = true;
            cand.talfControl.newFilters  = true;
            cand.talfControl.mode        = mode;
            cand.talfControl.apsIds.push_back(newApsId);
            cand.talfCtbParam            = bestControl;
            cand.params.push_back(talfParam);
            globalMinCost = curTotalCost;
            bestTalfCand = cand;
          }
          if (curTotalCost < prevIterCost)
          {
            prevIterCost = curTotalCost;
          }
          else
          {
            break;
          }
        }
      }
    }
    // update training control and ctbFlags.
    tmpBestControl = bestControl;
    int numCtbOff = 0;
    for(int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      numCtbOff += tmpBestControl[ctbIdx].enabledFlag ? 0 : 1;
    }
    if (numCtbOff == 0)
    {
      break;
    }
    // update distortion for current best filterIdx
    AlfCovariance* c  = m_alfCovarianceTALF[shapeIdx][mode];
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_talfDistortion[0][filterIdx][ctbIdx] = uint64_t(c[ctbIdx].pixAcc + c[ctbIdx].calcErrorForTAlfCoeffs(
        talfParam.clipIdx[filterIdx], talfParam.coeff[filterIdx], numCoeff, talfParam.shift[filterIdx] + 1));
    }
  }
  filtControl.clear();
  tmpBestControl.clear();
  bestControl.clear();

  m_CABACEstimator->getCtx() = ctxStartTAlfFilterControlFlag;

  bool validNewFilter = bestTalfCand.talfControl.enabledFlag;
  if (validNewFilter)
  {
    validNewFilter = false;
    int sizeCtbControlFlags = int(bestTalfCand.talfCtbParam.size());
    CHECK(sizeCtbControlFlags != m_numCTUsInPic, "sizeCtbControlFlags != m_numCTUsInPic");
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      validNewFilter |= bestTalfCand.talfCtbParam[ctbIdx].enabledFlag;
    }
  }
  bestTalfCand.talfControl.enabledFlag = validNewFilter;

  return bestTalfCand;
}

void EncAdaptiveLoopFilter::deriveTAlfReuseFilter(CodingStructure &cs, ComponentID compId
  , const int mode, static_vector<CandTALF, NUM_TALF_REUSE_CANDS>& reuseCandList)
{
  const int numCoeff = NUM_TALF_COEFF + 1;
  AlfFilterShape alfFilterShape(size_TALF - numCoeff);
  const TempCtx ctxStartTAlfFilterControlFlag  ( m_ctxCache, SubCtx( Ctx::TAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );
  int newApsId = -1;
  std::vector<int> apsIds = getAvailableTAlfApsIds(cs, newApsId);
  std::vector<TAlfCtbParam> filtCtbControl;
  filtCtbControl.resize(m_numCTUsInPic);
  static_vector<double, NUM_TALF_REUSE_CANDS> reuseCostList;

  if ( apsIds.size() > 0 )
  {
    for(int apsId : apsIds)
    {
      auto param = m_apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + TALF_APS)->getTAlfAPSParam();
      for (int filterIdx = 0; filterIdx < param.filterCount; filterIdx++)
      {
        int shapeIdx = param.shapeIdx;
        AlfCovariance* c = m_alfCovarianceTALF[shapeIdx][mode];
        for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
        {
          m_talfDistortion[apsId][filterIdx][ctuIdx] = uint64_t(c[ctuIdx].pixAcc + c[ctuIdx].calcErrorForTAlfCoeffs(
            param.clipIdx[filterIdx], param.coeff[filterIdx], numCoeff, param.shift[filterIdx] + 1));
        }
      }
    }
    for ( int k = 1; k <= apsIds.size(); k++ )
    {
      std::vector<std::vector<int>> combMap = getCombMap(k, apsIds);
      for( int combId = 0; combId < combMap.size(); combId++ )
      {
        // set the combination, distortion
        std::vector<int> curApsIds = combMap[combId];
        std::vector<TAlfFilterParam> params;
        for( int apsId : curApsIds )
        {
          APS* aps = m_apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + TALF_APS);
          CHECK(!aps, "empty aps, error");
          auto param = aps->getTAlfAPSParam();
          params.push_back(param);
        }

        uint64_t curTotalDistortion = 0;
        double   curTotalRate       = 0;
        double   curTotalCost       = 0;
        m_CABACEstimator->getCtx() = ctxStartTAlfFilterControlFlag;
        if(determineCtuFlagReuse(cs, compId, m_ctbDistortionUnfilter, m_talfDistortion, filtCtbControl.data()
          , curApsIds, params, curTotalDistortion, curTotalRate))
        {
          curTotalRate += 1                                  /*sh/ph enabledFlag*/
                        + (isBiTAlf(mode) ? 2 : 3)
                        + 1                                  /*newFilters flag*/
                        + lengthUvlc(int(params.size()) - 1) /*numApsMinus1*/
                        + int(params.size()) * 3;            /*apsIds*/
          curTotalCost = curTotalDistortion + m_lambda[compId] * curTotalRate;
          CandTALF cand;
          cand.rate   = curTotalRate;
          cand.talfCtbParam = filtCtbControl;
          cand.params = params;
          cand.talfControl.enabledFlag = true;
          cand.talfControl.newFilters  = false;
          cand.talfControl.apsIds      = curApsIds;
          cand.talfControl.mode        = mode;
          updateCandList(cand, curTotalCost, reuseCandList, reuseCostList, NUM_TALF_REUSE_CANDS);
        }
      }
    }
  }

  filtCtbControl.clear();
  m_CABACEstimator->getCtx() = ctxStartTAlfFilterControlFlag;
}

void EncAdaptiveLoopFilter::deriveTAlfFilter(CodingStructure &cs, ComponentID compId, std::vector<CandTALF>& talfCandList)
{
  bool modeOff[NUM_TALF_MODE] = { false, false, false, false, false, false };
  for(int mode = 0; mode < NUM_TALF_MODE; mode++)
  {
    AlfCovariance& covarFrame0 = m_alfCovarianceFrameTALF[0][mode];
    modeOff[mode] = (covarFrame0.pixAcc == 0) ? true : false;
  }

  for(int mode = 0; mode < NUM_TALF_MODE; mode++)
  {
    if (modeOff[mode])
    {
      continue;
    }
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_ctbDistortionUnfilter[compId][ctbIdx] = double(m_alfCovarianceTALF[0][mode][ctbIdx].pixAcc);
    }
    for(int shapeIdx = 0; shapeIdx < MAX_TALF_FILTER_SHAPE; shapeIdx++)
    {
      CandTALF newCand = deriveTAlfNewFilter(cs, compId, mode, shapeIdx);
      if(newCand.talfControl.enabledFlag)
      {
        talfCandList.push_back(newCand);
      }
    }
    static_vector<CandTALF, NUM_TALF_REUSE_CANDS> reuseCandList;
    deriveTAlfReuseFilter(cs, compId, mode, reuseCandList);
    for (size_t candIdx = 0; candIdx < reuseCandList.size(); candIdx++)
    {
      talfCandList.push_back(reuseCandList[candIdx]);
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForTAlfFilter(CodingStructure &cs, ComponentID compId, const PelUnitBuf &orgYuv, const PelUnitBuf &recBeforeALF)
{
  for(int shapeIdx = 0; shapeIdx < MAX_TALF_FILTER_SHAPE; shapeIdx++)
  {
    for (int mode = 0; mode < NUM_TALF_MODE; mode++)
    {
      AlfCovariance* covar = m_alfCovarianceTALF[shapeIdx][mode];
      AlfCovariance& covarFrame = m_alfCovarianceFrameTALF[shapeIdx][mode];
      covarFrame.reset(m_encCfg->getUseNonLinearAlfLuma() ? MaxAlfNumClippingValues : 1);
      int ctuRsAddr = 0;
      for( ; ctuRsAddr < m_numCTUsInPic; ctuRsAddr++ )
      {
        covar[ctuRsAddr].reset(m_encCfg->getUseNonLinearAlfLuma() ? MaxAlfNumClippingValues : 1);
      }
      ctuRsAddr = 0;
      for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
      {
        for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
        {
          UnitArea ctu(cs.pcv->chrFormat, Area(xPos, yPos, m_maxCUWidth, m_maxCUHeight));
          UnitArea clippedCtu = clipArea(ctu, *cs.slice->getPic());
          const CPelBuf org = orgYuv.get(compId);
          const CPelBuf rec = recBeforeALF.get(compId);
          getBlkStatsTAlf(covar[ctuRsAddr], org, rec, cs, clippedCtu, compId, shapeIdx, mode);
          covarFrame += covar[ctuRsAddr];
          ctuRsAddr++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStatsTAlf(AlfCovariance &alfCovariance, const CPelBuf &orgYuv, const CPelBuf &recBeforeALF, CodingStructure &cs
    , const UnitArea& clippedCtu, const ComponentID compId, const int shapeIdx, const int mode)
{
  const CPelBuf recBuf = cs.getRecoBuf(compId);
  const int numBins = alfCovariance.numBins;
  const int isBiFilter = isBiTAlf(mode);
  const int numCoeff = NUM_TALF_COEFF + 1;
  MvField mvField[2];
  Pel inputBatch[4][NUM_TALF_COEFF + 1][TALF_SBB_SIZE][TALF_SBB_SIZE];
  Pel clipMax[4][MAX_NUM_ALF_LUMA_COEFF];
  Pel clipMin[4][MAX_NUM_ALF_LUMA_COEFF];
  for(int clipIdx = 0; clipIdx < MaxAlfNumClippingValues; clipIdx++)
  {
    std::fill_n(clipMax[clipIdx], MAX_NUM_ALF_LUMA_COEFF,  m_alfClippingValues[toChannelType(compId)][clipIdx]);
    std::fill_n(clipMin[clipIdx], MAX_NUM_ALF_LUMA_COEFF, -m_alfClippingValues[toChannelType(compId)][clipIdx]);
  }
 
  int numSamples = 0;
  for(int y = clippedCtu.blocks[compId].y; y < (clippedCtu.blocks[compId].y + clippedCtu.blocks[compId].height); y += TALF_SBB_SIZE)
  {
    for (int x = clippedCtu.blocks[compId].x; x < (clippedCtu.blocks[compId].x + clippedCtu.blocks[compId].width); x += TALF_SBB_SIZE)
    {
      const Position sbbPos(x, y);
      if (getMotionOffset(cs, sbbPos, mvField, mode, shapeIdx))
      {
        m_setTAlfInput[isBiFilter](inputBatch, cs, compId, recBeforeALF, clipMax, clipMin, sbbPos, shapeIdx
          , m_picWidth, m_picHeight, mode, m_refCombs, mvField, MaxAlfNumClippingValues);
        for (int ySbb = 0; ySbb < TALF_SBB_SIZE; ySbb++)
        {
          for (int xSbb = 0; xSbb < TALF_SBB_SIZE; xSbb++)
          {
            const Position pos(sbbPos.offset(xSbb, ySbb));
            Intermediate_Int yLocal = orgYuv.at(pos) - recBuf.at(pos);
            alfCovariance.pixAcc += yLocal * (float)yLocal;
            for(int clipIdx = 0; clipIdx < MaxAlfNumClippingValues; clipIdx++)
            {
              for(int idx = 0; idx < numCoeff; idx++)
              {
                m_ELocalStorage[clipIdx][idx][numSamples] = inputBatch[clipIdx][idx][ySbb][xSbb];
              }
            }
            m_yLocalStorage[numSamples++] = yLocal;
          }
        }
      }
      
    }
  }

  if(numSamples == 0)
  {
    return;
  }

  for (ptrdiff_t b0 = 0; b0 < numBins; b0++)
  {
    for (ptrdiff_t k = 0; k < numCoeff; k++)
    {
      for (ptrdiff_t b1 = 0; b1 <= b0; b1++)
      {
        const ptrdiff_t maxl = b0 == b1 ? k + 1 : numCoeff;
        for (ptrdiff_t l = 0; l < maxl; l++)
        {
          const ptrdiff_t oe = alfCovariance.getOffsetEfast(b0, b1, k, l);
          float curE = 0;
          int batchNum = numSamples >> 4;
          int idx = 0;
          for(; idx < (batchNum << 4); idx += 16)
          {
            curE += (float)m_groupSumTAlf(&m_ELocalStorage[b0][k][idx], &m_ELocalStorage[b1][l][idx]);
          }
          for(; idx < numSamples; idx++)
          {
            curE += m_ELocalStorage[b0][k][idx] * (float)m_ELocalStorage[b1][l][idx];
          }
          alfCovariance.data[oe] += curE;
        }
      }
    }
    for (ptrdiff_t k = 0; k < numCoeff; k++)
    {
      ptrdiff_t b = b0;
      const ptrdiff_t oy = alfCovariance.getOffsetY(b, k);
      float curY = 0;
      int batchNum = numSamples >> 4;
      int idx = 0;
      for(; idx < (batchNum << 4); idx += 16)
      {
        curY += (float)m_groupSumTAlf(&m_ELocalStorage[b][k][idx], &m_yLocalStorage[idx]);
      }
      for(; idx < numSamples; idx++)
      {
        curY += m_ELocalStorage[b][k][idx] * (float)m_yLocalStorage[idx];
      }
      alfCovariance.data[oy] += curY;
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStatsTAlf(const int filterIdc, AlfCovariance* covar, AlfCovariance& covarFrame)
{
  covarFrame.reset(m_encCfg->getUseNonLinearAlfLuma() ? MaxAlfNumClippingValues : 1);
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    if (m_trainingCovControl[ctuIdx] == filterIdc)
    {
      covarFrame += covar[ctuIdx];
    }
  }
}

void EncAdaptiveLoopFilter::deriveCoeffTAlf(const ComponentID compId, const AlfFilterShape& alfFilterShape, const AlfCovariance& covarFrame
    , int filterCoeffQuant[MAX_NUM_ALF_LUMA_COEFF], int filterClipIdx[MAX_NUM_ALF_LUMA_COEFF]
    , const int nonLinearFlag, const int bitDepth)
{
  const int numCoeff = alfFilterShape.numCoeff;
  std::fill_n( filterClipIdx, numCoeff, nonLinearFlag ? AlfNumClippingValues[toChannelType(compId)] / 2 : 0 );
  deriveCoeffForTAlfQuant(filterClipIdx, filterCoeffQuant, covarFrame, alfFilterShape, bitDepth, nonLinearFlag);
}

double EncAdaptiveLoopFilter::getRecoDist(const PelUnitBuf &orgYuv, PelUnitBuf &recYuv, const ComponentID compId, const CodingStructure& cs)
{
  const int bd = cs.sps->getBitDepth(toChannelType(compId));
  Distortion dist = m_sseCost->getDistPart(orgYuv.get(compId), recYuv.get(compId), bd, compId, DF_SSE);

  return double(dist);
}

int EncAdaptiveLoopFilter::getOneTAlfRate(int nonLinearFlag, int coeff[MAX_NUM_ALF_LUMA_COEFF])
{
  const int numCoeff = NUM_TALF_COEFF;
  int bestCurBits = MAX_INT;
  for(int kCur = 0; kCur <= 1; kCur++)
  {
    int curBits = 0;
    curBits += 1; // 1-bit for alternative order-k
    curBits += ceilLog2(MAX_TALF_FILTER_SHAPE); // 1-bit shape index
    for (int cIdx = 0; cIdx < numCoeff; cIdx++)
    {
      curBits += lengthGolomb(coeff[cIdx], kCur);
      if (nonLinearFlag && coeff[cIdx])
      {
        curBits += 2;
      }
    }
    if(curBits < bestCurBits)
    {
      bestCurBits = curBits;
    }
  }

  return bestCurBits;
}

int EncAdaptiveLoopFilter::getSetTAlfRate(uint8_t filterCount, int bestCoeff[MAX_NUM_TALF_FILTERS][MAX_NUM_ALF_LUMA_COEFF]
    , int coeff[MAX_NUM_ALF_LUMA_COEFF], int bestNonLinearFlag[MAX_NUM_TALF_FILTERS], int nonLinearFlag)
{
  int bits = 0;
  if ( filterCount > 0 )
  {
    int filterCountMinus1 = filterCount - 1;
    bits += lengthUvlc(filterCountMinus1);
    bits += ceilLog2(MAX_TALF_FILTER_SHAPE); // 1-bit shape index
    const int numCoeff = NUM_TALF_COEFF;
    for ( int fIdx = 0; fIdx <= filterCountMinus1; fIdx++ )
    {
      int bestCurBits = MAX_INT;
      int* c = (fIdx == filterCountMinus1) ? coeff : bestCoeff[fIdx];
      int nl = (fIdx == filterCountMinus1) ? nonLinearFlag : bestNonLinearFlag[fIdx];
      for(int kCur = 0; kCur <= 1; kCur++)
      {
        int curBits = 0;
        curBits += 1; // 1-bit for alternative order-k
        curBits += 2; // 2-bits for coefficient precision
        curBits += 1; // non-linear clip

        // bits for coefficients.
        for (int cIdx = 0; cIdx < numCoeff; cIdx++)
        {
          curBits += lengthGolomb(c[cIdx], kCur);
          if (nl && c[cIdx])
          {
            curBits += 2;
          }
        }
        if(curBits < bestCurBits)
        {
          bestCurBits = curBits;
        }
      }
      bits += bestCurBits;
    }
  }

  return bits;
}

void EncAdaptiveLoopFilter::setTAlfAPS( CodingStructure &cs, CandTALF& cand )
{
  auto talfSliceControl = cand.talfControl;
  cs.slice->setTileGroupTAlfControl(talfSliceControl);
  if(!talfSliceControl.enabledFlag)
  {
    return;
  }
  if (talfSliceControl.newFilters)
  {
    CHECK(talfSliceControl.apsIds.size() < 1, "talfSliceControl.apsIds.size() < 1");
    int numAPS = int(talfSliceControl.apsIds.size());
    int newApsId = talfSliceControl.apsIds[numAPS - 1];
    APS* newAPS = m_apsMap->getPS((newApsId << NUM_APS_TYPE_LEN) + TALF_APS);
    if (newAPS == NULL)
    {
      newAPS = m_apsMap->allocatePS((newApsId << NUM_APS_TYPE_LEN) + TALF_APS);
      newAPS->setAPSId(newApsId);
      newAPS->setAPSType(TALF_APS);
    }
    CHECK(cand.params.size() != numAPS, "cand.params.size() != numAPS");
    CHECK(!cand.params[numAPS - 1].newFlag, "!cand.params[numAPS - 1].newFlag");
    newAPS->setTAlfAPSParam(cand.params[numAPS - 1]);
    newAPS->setTemporalId( cs.slice->getTLayer() );
    m_apsMap->setChangedFlag((newApsId << NUM_APS_TYPE_LEN) + TALF_APS);
    m_apsIdStart2 = newApsId; // --included
  }

  for(auto apsId : talfSliceControl.apsIds)
  {
    cs.slice->getTAlfAPSs()[apsId] = m_apsMap->getPS((apsId << NUM_APS_TYPE_LEN) + TALF_APS);
  }
  auto ctbControl = cand.talfCtbParam;
  for (int i = 0; i < m_numCTUsInPic; i++)
  {
    m_tAlfCtbControl[i] = ctbControl[i];
  }
}

std::vector<int> EncAdaptiveLoopFilter::getAvailableTAlfApsIds(CodingStructure& cs, int& newApsId)
{
  APS** apss2 = cs.slice->getTAlfAPSs();
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    apss2[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + TALF_APS);
  }

  std::vector<int> result;
  for(int curApsId = 0; curApsId < ALF_CTB_MAX_NUM_APS; curApsId++)
  {
    if (!cs.slice->isIntra() && !cs.slice->getPendingRasInit() && !cs.slice->isIDRorBLA())
    {
      APS* curAPS = cs.slice->getTAlfAPSs()[curApsId];
      if (curAPS && curAPS->getLayerId() == cs.slice->getPic()->layerId
          && curAPS->getTemporalId() <= cs.slice->getTLayer() && curAPS->getTAlfAPSParam().newFlag)
      {
        result.push_back(curApsId);
      }
    }
  }

  newApsId = m_apsIdStart2 - 1;
  if (newApsId < 0)
  {
    newApsId = ALF_CTB_MAX_NUM_APS - 1;
  }
  CHECK(newApsId >= ALF_CTB_MAX_NUM_APS, "Wrong APS index assignment in getAvailableTAlfAps");

  return result;
}
#endif
