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

/** \file     AlfParameters.h
    \brief    Define types for storing ALF parameters
*/

#ifndef __ALFPARAMETERS__
#define __ALFPARAMETERS__

#include <vector>
#include "CommonDef.h"

//! \ingroup AlfParameters
//! \{

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
enum AlfFixedFilterType
{
  ALF_FIXED_FILTER_13_DB_9,
  ALF_FIXED_FILTER_9_DB_9
};
#endif

enum AlfFilterType
{
  ALF_FILTER_5,
  ALF_FILTER_7,
  CC_ALF,

#if ALF_IMPROVEMENT
  ALF_FILTER_9,
  ALF_FILTER_9_EXT,
  ALF_FILTER_EXT,
#if JVET_AA0095_ALF_LONGER_FILTER
  ALF_FILTER_13_EXT,
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  ALF_FILTER_9_EXT_DB,
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
  ALF_FILTER_13_EXT_DB,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  ALF_FILTER_13_EXT_DB_RESI_DIRECT,
  ALF_FILTER_13_EXT_DB_RESI,
#if FIXFILTER_CFG
  ALF_FILTER_13_DB_RESI_DIRECT,
  ALF_FILTER_13_DB_RESI,
#endif
#endif
#if FIXFILTER_CFG
  ALF_FILTER_9_NO_FIX,
#endif
#endif
  ALF_NUM_OF_FILTER_TYPES
};


static const int size_CC_ALF = -1;
#if ALF_IMPROVEMENT
static const int size_ALF_FILTER_9_EXT = -2;
static const int size_ALF_FILTER_EXT = -3;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
static const int size_ALF_FILTER_13_EXT = -4;
static const int size_ALF_FILTER_9_EXT_DB = -5;
static const int size_ALF_FILTER_13_EXT_DB = -6;
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
static const int size_ALF_FILTER_13_EXT_DB_RESI_DIRECT = -7;
static const int size_ALF_FILTER_13_EXT_DB_RESI        = -8;
#if FIXFILTER_CFG
static const int size_ALF_FILTER_13_DB_RESI_DIRECT = -9;
static const int size_ALF_FILTER_13_DB_RESI        = -10;
#endif
#endif
#if FIXFILTER_CFG
static const int size_ALF_FILTER_9_NO_FIX          = -11;
#endif
#if JVET_AK0065_TALF
static const int size_TALF = -100;
#endif
const int alfTypeToSize[ALF_NUM_OF_FILTER_TYPES] = { 5, 7, size_CC_ALF, 9, size_ALF_FILTER_9_EXT, size_ALF_FILTER_EXT, size_ALF_FILTER_13_EXT, size_ALF_FILTER_9_EXT_DB, size_ALF_FILTER_13_EXT_DB 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                                                     ,size_ALF_FILTER_13_EXT_DB_RESI_DIRECT, size_ALF_FILTER_13_EXT_DB_RESI
#if FIXFILTER_CFG
  ,size_ALF_FILTER_13_DB_RESI_DIRECT, size_ALF_FILTER_13_DB_RESI
#endif
#endif
#if FIXFILTER_CFG
  , size_ALF_FILTER_9_NO_FIX
#endif
};
#elif JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
static const int size_ALF_FILTER_9_EXT_DB = -4;
const int alfTypeToSize[ALF_NUM_OF_FILTER_TYPES] = { 5, 7, size_CC_ALF, 9, size_ALF_FILTER_9_EXT, size_ALF_FILTER_EXT, size_ALF_FILTER_9_EXT_DB };
#elif JVET_AA0095_ALF_LONGER_FILTER
static const int size_ALF_FILTER_13_EXT = -4;
const int alfTypeToSize[ALF_NUM_OF_FILTER_TYPES] = { 5, 7, size_CC_ALF, 9, size_ALF_FILTER_9_EXT, size_ALF_FILTER_EXT, size_ALF_FILTER_13_EXT };
#elif JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
const int alfTypeToSize[ALF_NUM_OF_FILTER_TYPES] = { 5, 7, size_CC_ALF, 9, size_ALF_FILTER_9_EXT, size_ALF_FILTER_EXT };
#endif
#endif

struct AlfFilterShape
{
  AlfFilterShape( int size )
    : filterLength( size ),
    numCoeff( size * size / 4 + 1 ),
    filterSize( size * size / 2 + 1 )
#if ALF_IMPROVEMENT
    , numOrder( 0 )
    , indexSecOrder( 0 )
    , offset0( 0 )
#endif
  {
    if( size == 5 )
    {
      pattern = {
                 0,
             1,  2,  3,
         4,  5,  6,  5,  4,
             3,  2,  1,
                 0
      };

      weights = {
                 2,
              2, 2, 2,
           2, 2, 1, 1
      };
      filterType = ALF_FILTER_5;
#if ALF_IMPROVEMENT
      numOrder = 1;
      indexSecOrder = numCoeff - 1;
      offset0 = 0;
#endif
    }
    else if( size == 7 )
    {
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 11, 10, 9,
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };

      weights = {
                    2,
                2,  2,  2,
            2,  2,  2,  2,  2,
        2,  2,  2,  1,  1
      };
      filterType = ALF_FILTER_7;
#if ALF_IMPROVEMENT
      numOrder = 1;
      indexSecOrder = numCoeff - 1;
      offset0 = 0;
#endif
    }
#if ALF_IMPROVEMENT
    else if( size == 9 )
    {
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 19, 18, 17, 16,
        15, 14, 13, 12, 11, 10,  9, 
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };

      weights = {
                    2,
                2,  2,  2,
            2,  2,  2,  2,  2,
        2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  1,  1
      };
      filterType = ALF_FILTER_9;
#if ALF_IMPROVEMENT
#if JVET_AG0157_ALF_CHROMA_FIXED_FILTER
      numCoeff = MAX_NUM_ALF_CHROMA_COEFF;
      numOrder = 2;
      indexSecOrder = numCoeff - 2;
#else
      numOrder = 1;
      indexSecOrder = numCoeff - 1;
#endif
      offset0 = 0;
#endif
    }
    else if( size == size_ALF_FILTER_EXT )
    {
      size = filterLength = 0;
      numCoeff = filterSize = EXT_LENGTH;
      filterType = ALF_FILTER_EXT;
#if ALF_IMPROVEMENT
      numOrder = 1;
      indexSecOrder = numCoeff - 1;
      offset0 = ALF_ORDER;
#endif
    }
#if FIXFILTER_CFG
    else if( size == size_ALF_FILTER_9_NO_FIX )
    {
      filterLength = 9;
      filterSize = 9 * 9 / 2 + 1 ;
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 19, 18, 17, 16,
        15, 14, 13, 12, 11, 10,  9, 
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };

      weights = {
                    2,
                2,  2,  2,
            2,  2,  2,  2,  2,
        2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  1,  1
      };
      filterType = ALF_FILTER_9_NO_FIX;
      numCoeff = 21;
      numOrder = 1;
      indexSecOrder = 20;
      offset0 = 0;
    }
#endif
    else if( size == size_ALF_FILTER_9_EXT )
    {
      size = 9;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      numCoeff = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1;
      filterSize = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1;
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
      numCoeff = 21 + EXT_LENGTH;
      filterSize = 21 + EXT_LENGTH;
#else
      numCoeff = MAX_NUM_ALF_LUMA_COEFF;
      filterSize = MAX_NUM_ALF_LUMA_COEFF;
#endif
#endif
      filterLength = 9;
      filterType = ALF_FILTER_9_EXT;
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 19, 18, 17, 16,
        15, 14, 13, 12, 11, 10,  9,
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };
#if ALF_IMPROVEMENT
      numOrder = 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      indexSecOrder = 20 + NUM_FIXED_BASED_COEFF - 1;
#else
      indexSecOrder = 20;
#endif
      offset0 = 0;
#endif
    }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    else if( size == size_ALF_FILTER_9_EXT_DB )
    {
      size = 9;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      numCoeff = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
      filterSize = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
#else
      numCoeff = 21 + EXT_LENGTH + NUM_DB;
      filterSize = 21 + EXT_LENGTH + NUM_DB;
#endif
      filterLength = 9;
      filterType = ALF_FILTER_9_EXT_DB;
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 19, 18, 17, 16,
        15, 14, 13, 12, 11, 10,  9,
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };
      numOrder = 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      indexSecOrder = 19 + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
#else
      indexSecOrder = 19 + NUM_DB;
#endif
      offset0 = 0;
    }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
    else if( size == size_ALF_FILTER_13_EXT )
    {
      size = 13;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 11 + EXT_LENGTH + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_FIXED_BASED_COEFF_NEW - 1;
      filterSize = 11 + EXT_LENGTH + NUM_FIXED_BASED_COEFF_NEW - 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF;
#else
      numCoeff = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1;
      filterSize = 21 + EXT_LENGTH + NUM_FIXED_BASED_COEFF - 1;
#endif
#else
      numCoeff = 21 + EXT_LENGTH;
      filterSize = 21 + EXT_LENGTH;
#endif
      filterLength = 13;
      filterType = ALF_FILTER_13_EXT;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder = 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 10 + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 10 + NUM_FIXED_BASED_COEFF_NEW - 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 20 + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#else
      indexSecOrder = 20 + NUM_FIXED_BASED_COEFF - 1;
#endif
#else
      indexSecOrder = 20;
#endif
      offset0 = 0;
    }
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
    else if( size == size_ALF_FILTER_13_EXT_DB )
    {
      size = 13;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1;
      filterSize = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF;
#else
      numCoeff = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
      filterSize = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
#endif
#else
      numCoeff = 21 + EXT_LENGTH + NUM_DB;
      filterSize = 21 + EXT_LENGTH + NUM_DB;
#endif
      filterLength = 13;
      filterType = ALF_FILTER_13_EXT_DB;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder = 2;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 9 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 9 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 19 + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#else
      indexSecOrder = 19 + NUM_DB + NUM_FIXED_BASED_COEFF - 1;
#endif
#else
      indexSecOrder = 19 + NUM_DB;
#endif
      offset0 = 0;
    }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    else if (size == size_ALF_FILTER_13_EXT_DB_RESI_DIRECT)
    {
      size          = 13;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI;
      filterSize = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
#else
      numCoeff      = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI;
      filterSize    = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI;
#endif
      filterLength  = 13;
      filterType    = ALF_FILTER_13_EXT_DB_RESI_DIRECT;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder      = 2;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 18 + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#else
      indexSecOrder = 18 + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI;
#endif
      offset0       = 0;
    }
    else if (size == size_ALF_FILTER_13_EXT_DB_RESI)
    {
      size          = 13;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      numCoeff   = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1 + 1 + NUM_LAPLACIAN_FILTERED_COEFF;
      filterSize = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1 + 1 + NUM_LAPLACIAN_FILTERED_COEFF;
#else
      numCoeff      = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1 + 1;
      filterSize    = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1 + 1;
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff      = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1;
      filterSize    = 11 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff      = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + 1 + 1;
      filterSize    = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + 1 + 1;
#else
      numCoeff      = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + 1;
      filterSize    = 21 + EXT_LENGTH + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI + 1;
#endif
      filterLength  = 13;
      filterType    = ALF_FILTER_13_EXT_DB_RESI;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder      = 2;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
#if JVET_AK0091_LAPLACIAN_INFO_IN_ALF
      indexSecOrder = 8 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI + NUM_LAPLACIAN_FILTERED_COEFF - NUM_LAPLACIAN_FILTERED_SOURCE;
#else
      indexSecOrder = 8 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI;
#endif
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_FIXED_BASED_COEFF_NEW - 1 + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 18 + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI;
#else
      indexSecOrder = 18 + NUM_DB + NUM_FIXED_BASED_COEFF - 1 + NUM_RESI;
#endif
      offset0       = 0;
    }
#if FIXFILTER_CFG
    else if (size == size_ALF_FILTER_13_DB_RESI_DIRECT)
    {
      size          = 13;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 11 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 11 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff = 11 + NUM_DB + NUM_RESI;
      filterSize = 11 + NUM_DB + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff = 21 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
      filterSize = 21 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF;
#else
      numCoeff      = 21 + NUM_DB;
      filterSize    = 21 + NUM_DB;
#endif
      filterLength  = 13;
      filterType    = ALF_FILTER_13_DB_RESI_DIRECT;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder      = 2;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 18 + NUM_DB + NUM_RESI + NUM_GAUSS_FILTERED_COEFF - NUM_GAUSS_FILTERED_SOURCE;
#else
      indexSecOrder = 18 + NUM_DB + NUM_RESI;
#endif
      offset0       = 0;
    }
    else if (size == size_ALF_FILTER_13_DB_RESI)
    {
      size          = 13;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff      = 11 + NUM_DB + NUM_RESI + 1;
      filterSize    = 11 + NUM_DB + NUM_RESI + 1;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      numCoeff      = 11 + NUM_DB + NUM_RESI + 1;
      filterSize    = 11 + NUM_DB + NUM_RESI + 1;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      numCoeff      = 21 + NUM_DB + NUM_RESI + 1 + 1;
      filterSize    = 21 + NUM_DB + NUM_RESI + 1 + 1;
#else
      numCoeff      = 21 + NUM_DB + NUM_RESI + 1;
      filterSize    = 21 + NUM_DB + NUM_RESI + 1;
#endif
      filterLength  = 13;
      filterType    = ALF_FILTER_13_DB_RESI;
      pattern = {
                            0,
                            1,
                            2,  
                            3,
                    4,  5,  6,  7,  8,
                    9, 10, 11, 12, 13, 
   14, 15, 16, 17, 18, 19, 20, 19, 18, 17, 16, 15, 14, 
                   13, 12, 11, 10,  9,
                    8,  7,  6,  5,  4,
                            3,  
                            2,  
                            1,
                            0
      };
      numOrder      = 2;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_RESI;
#elif JVET_AD0222_ALF_LONG_FIXFILTER
      indexSecOrder = 8 + NUM_DB + NUM_RESI;
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
      indexSecOrder = 18 + NUM_DB + NUM_RESI;
#else
      indexSecOrder = 18 + NUM_DB + NUM_RESI;
#endif
      offset0       = 0;
    }
#endif
#endif
#endif
    else if( size == size_CC_ALF )
    {
      size = 4;
#if JVET_X0071_LONGER_CCALF
      filterLength = MAX_NUM_CC_ALF_CHROMA_COEFF;
      numCoeff = MAX_NUM_CC_ALF_CHROMA_COEFF;
      filterSize = MAX_NUM_CC_ALF_CHROMA_COEFF;
#else
      filterLength = 8;
      numCoeff = 8;
      filterSize = 8;
#endif
      filterType   = CC_ALF;
    }
#if JVET_AK0065_TALF
    else if (size <= size_TALF)
    {
      numCoeff = abs(size - size_TALF);
    }
#endif
    else
    {
      filterType = ALF_NUM_OF_FILTER_TYPES;
      CHECK( 0, "Wrong ALF filter shape" );
    }
  }

  AlfFilterType filterType;
  int filterLength;
  int numCoeff;      //TO DO: check whether we need both numCoeff and filterSize
  int filterSize;
  std::vector<int> pattern;
  std::vector<int> weights;
#if ALF_IMPROVEMENT
  int numOrder;
  int indexSecOrder;
  int offset0;
#endif
};

#if JVET_AI0084_ALF_RESIDUALS_SCALING
static const int shiftPrecis = 3;

static const int shiftCorr    = 3;
static const int oneCorr      = (1 << shiftCorr);
static const int offCorr      = (1 << (shiftCorr - 1));

static const int shiftCorrChroma  = 3;
static const int oneCorrChroma    = (1 << shiftCorrChroma);
static const int offCorrChroma    = (1 << (shiftCorrChroma - 1));

static const int nbCorrAlfScale[4] = { 0, 5, 5, 9 };

static const int nbCorrChromaAlfScale[4] = { 5, 5, 5, 5 };


struct ScaleAlfEnc 
{
  static const int nbCorr = 12;

  std::vector<int>  num;
  uint64_t  mse[MAX_NUM_ALF_CLASSES][nbCorr];
  double bestCost[MAX_NUM_ALF_CLASSES];

  void reset() 
  {
    num.resize( MAX_NUM_ALF_CLASSES, 0 );
    memset( num.data(), 0, sizeof(int) * num.size() );
    memset( mse, 0, sizeof(mse) );
  }

  void addMse( const int idxClass, const int idxCorr, const int s )
  {
    mse[idxClass][idxCorr] += s * s;
    if ( idxCorr == 0 )
    {
      num[idxClass]++;
    }
  }

};

struct ScaleAlf 
{
  static const int maxGroupShift = 4;

  bool  initDone = false;
  bool  initMinMaxDone = false;
  int filterSetIndex;
  int classifierIdx;
  int alt_num;
  std::vector<int>  idxCorr;
  int idxClassMin, idxClassMax;
  // group :
  std::vector<int>  groupIdxCorr;
  int groupShift;
  int groupSize;
  int groupNum;
  bool usePrev;

  int apsIdx;
#if JVET_AJ0237_INTERNAL_12BIT
  int bitDepth;
#endif

  void reset() 
  {
    idxCorr.resize(MAX_NUM_ALF_CLASSES, 0);
    groupIdxCorr.resize(MAX_NUM_ALF_CLASSES, 0);

    std::fill( idxCorr.begin(), idxCorr.end(), 0 );
    idxClassMin = 0;
    idxClassMax = MAX_NUM_ALF_CLASSES - 1;

    std::fill( groupIdxCorr.begin(), groupIdxCorr.end(), 0 );
    groupShift = groupNum = 0;
    groupSize = MAX_NUM_ALF_CLASSES;

    apsIdx = -1;

    usePrev = false;

    initDone = false;
  }

  void setMinMax( const Pel lumaMin = 0, const Pel lumaMax = 1024, const bool bCheckClassifier = true ) 
  {
    const int c = classifierIdx;
#if !JVET_AJ0237_INTERNAL_12BIT
    const int bitDepth = 10;
#endif
    idxClassMin = (!bCheckClassifier || c == 1) ? ((lumaMin * ALF_NUM_CLASSES_CLASSIFIER[c]) >> bitDepth) : 0 ;
    idxClassMax = (!bCheckClassifier || c == 1) ? ((lumaMax * ALF_NUM_CLASSES_CLASSIFIER[c]) >> bitDepth) : (ALF_NUM_CLASSES_CLASSIFIER[c] - 1) ;

    initMinMaxDone = true;
  }

#if JVET_AJ0237_INTERNAL_12BIT
  void init(const int f, const int a, const int c, const int bDepth)
#else
  void init( const int f, const int a, const int c )
#endif
  {
    filterSetIndex  = f;
    alt_num         = a;
    classifierIdx   = c;

    reset();

    idxClassMin = 0 ;
    idxClassMax = ALF_NUM_CLASSES_CLASSIFIER[c] - 1 ;
#if JVET_AJ0237_INTERNAL_12BIT
    bitDepth = bDepth;
#endif
    initDone = true;
  }

  int setGroupSize( const int _groupShift ) 
  {
    groupShift  = _groupShift;
    groupNum    = 1 << groupShift;

    const int kMin = idxClassMin;
    const int kMax = idxClassMax;
    groupSize = (kMax - kMin + 1) >> groupShift;
    return groupSize;
  }

  void  fillIdxCorr() 
  {
    if ( usePrev )
    {
      return;
    }

    const int kMin = idxClassMin;
    const int kMax = idxClassMax;

    CHECK( groupSize != ((kMax - kMin + 1) >> groupShift), "fillIdxCorr() check groupSize failed.");

    memset( idxCorr.data(), 0, sizeof(int) * idxCorr.size() );
    for ( int g = 0; g < groupNum; g++ )
    {
      int kMinG = kMin + g * groupSize;
      int kMaxG = (g == groupNum - 1) ? (kMax + 1) : (kMin + (g + 1) * groupSize);
      for ( int k = kMinG; k < kMaxG; k++ ) 
      {
        idxCorr[k] = groupIdxCorr[g];
      }
    }

  }

  void setApsIdx( const int _apsIdx ) 
  {
    apsIdx = _apsIdx;
  }

  void copyFrom( ScaleAlf& other ) 
  {
    *this = other;
  }

};
#endif

struct AlfParam
{
  bool                         enabledFlag[MAX_NUM_COMPONENT];                          // alf_slice_enable_flag, alf_chroma_idc
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
  char                         lumaClassifierIdx[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  char                         coeffBits[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char                         coeffMantissa[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#endif
  AlfFilterType                filterType[MAX_NUM_CHANNEL_TYPE];
  bool                         nonLinearFlag[MAX_NUM_CHANNEL_TYPE][32]; // alf_[luma/chroma]_clip_flag
  int                          numAlternativesLuma;
  short                        lumaCoeff[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          lumaClipp[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
#else  
  short                        lumaClipp[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
#endif  
  int                          numLumaFilters[MAX_NUM_ALF_ALTERNATIVES_LUMA];                                          // number_of_filters_minus1 + 1
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]// number_of_filters_minus1 + 1
#else
  bool                         nonLinearFlag[MAX_NUM_CHANNEL_TYPE];                     // alf_[luma/chroma]_clip_flag
  short                        lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          lumaClipp[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
#else
  short                        lumaClipp[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_clipp_luma_[i][j]
#endif
  int                          numLumaFilters;                                          // number_of_filters_minus1 + 1
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]
#endif
  int                          numAlternativesChroma;                                                  // alf_chroma_num_alts_minus_one + 1
  short                        chromaCoeff[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF]; // alf_coeff_chroma[i]
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          chromaClipp[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF]; // alf_clipp_chroma[i]
#else
  short                        chromaClipp[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF]; // alf_clipp_chroma[i]
#endif
  bool                         alfLumaCoeffFlag[MAX_NUM_ALF_CLASSES];                   // alf_luma_coeff_flag[i]
  bool                         alfLumaCoeffDeltaFlag;                                   // alf_luma_coeff_delta_flag
  std::vector<AlfFilterShape>* filterShapes;
  bool                         newFilterFlag[MAX_NUM_CHANNEL_TYPE];
#if JVET_AK0123_ALF_COEFF_RESTRICTION
  char                         lumaScaleIdx[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES];
  char                         chromaScaleIdx[MAX_NUM_ALF_ALTERNATIVES_CHROMA][1];
#endif

  AlfParam()
  {
    reset();
  }

  void reset()
  {
    std::memset( enabledFlag, false, sizeof( enabledFlag ) );
    std::memset( nonLinearFlag, false, sizeof( nonLinearFlag ) );
#if JVET_X0071_ALF_BAND_CLASSIFIER
    std::memset( lumaClassifierIdx, 0, sizeof( lumaClassifierIdx ) );
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    std::memset( coeffBits, 0, sizeof( coeffBits ) );
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    std::memset(coeffMantissa, 0, sizeof(coeffMantissa));
#endif
    std::memset( lumaCoeff, 0, sizeof( lumaCoeff ) );
    std::memset( lumaClipp, 0, sizeof( lumaClipp ) );
    numAlternativesChroma = 1;
    std::memset( chromaCoeff, 0, sizeof( chromaCoeff ) );
    std::memset( chromaClipp, 0, sizeof( chromaClipp ) );
    std::memset( filterCoeffDeltaIdx, 0, sizeof( filterCoeffDeltaIdx ) );
    std::memset( alfLumaCoeffFlag, true, sizeof( alfLumaCoeffFlag ) );
#if ALF_IMPROVEMENT
    std::memset(filterType, false, sizeof(filterType));
    numAlternativesLuma = 1;
    for (int i = 0; i < MAX_NUM_ALF_ALTERNATIVES_LUMA; i++)
    {
      numLumaFilters[i] = 1;
    }
#else
    numLumaFilters = 1;
#endif
    alfLumaCoeffDeltaFlag = false;
    memset(newFilterFlag, 0, sizeof(newFilterFlag));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    std::memset(lumaScaleIdx, 0, sizeof(lumaScaleIdx));
    std::memset(chromaScaleIdx, 0, sizeof(chromaScaleIdx));
#endif
  }

  const AlfParam& operator = ( const AlfParam& src )
  {
    std::memcpy( enabledFlag, src.enabledFlag, sizeof( enabledFlag ) );
    std::memcpy( nonLinearFlag, src.nonLinearFlag, sizeof( nonLinearFlag ) );
#if JVET_X0071_ALF_BAND_CLASSIFIER
    std::memcpy( lumaClassifierIdx, src.lumaClassifierIdx, sizeof( lumaClassifierIdx ) );
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    std::memcpy( coeffBits, src.coeffBits, sizeof( coeffBits ) );
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    std::memcpy(coeffMantissa, src.coeffMantissa, sizeof(coeffMantissa));
#endif
    std::memcpy( lumaCoeff, src.lumaCoeff, sizeof( lumaCoeff ) );
    std::memcpy( lumaClipp, src.lumaClipp, sizeof( lumaClipp ) );
    numAlternativesChroma = src.numAlternativesChroma;
    std::memcpy( chromaCoeff, src.chromaCoeff, sizeof( chromaCoeff ) );
    std::memcpy( chromaClipp, src.chromaClipp, sizeof( chromaClipp ) );
    std::memcpy( filterCoeffDeltaIdx, src.filterCoeffDeltaIdx, sizeof( filterCoeffDeltaIdx ) );
    std::memcpy( alfLumaCoeffFlag, src.alfLumaCoeffFlag, sizeof( alfLumaCoeffFlag ) );
#if ALF_IMPROVEMENT
    std::memcpy(filterType, src.filterType, sizeof(filterType));
    numAlternativesLuma = src.numAlternativesLuma;
    std::memcpy(numLumaFilters, src.numLumaFilters, sizeof(numLumaFilters));
#else
    numLumaFilters = src.numLumaFilters;
#endif
    alfLumaCoeffDeltaFlag = src.alfLumaCoeffDeltaFlag;
    filterShapes = src.filterShapes;
    std::memcpy(newFilterFlag, src.newFilterFlag, sizeof(newFilterFlag));
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    std::memcpy(lumaScaleIdx, src.lumaScaleIdx, sizeof(lumaScaleIdx));
    std::memcpy(chromaScaleIdx, src.chromaScaleIdx, sizeof(chromaScaleIdx));
#endif
    return *this;
  }

  bool operator==( const AlfParam& other )
  {
    if( memcmp( enabledFlag, other.enabledFlag, sizeof( enabledFlag ) ) )
    {
      return false;
    }
    if( memcmp( nonLinearFlag, other.nonLinearFlag, sizeof( nonLinearFlag ) ) )
    {
      return false;
    }
#if ALF_IMPROVEMENT
    if( memcmp( filterType, other.filterType, sizeof( filterType ) ) )
    {
      return false;
    }
#if JVET_X0071_ALF_BAND_CLASSIFIER
    if( memcmp( lumaClassifierIdx, other.lumaClassifierIdx, sizeof( lumaClassifierIdx ) ) )
    {
      return false;
    }
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
    if( memcmp( coeffBits, other.coeffBits, sizeof( coeffBits ) ) )
    {
      return false;
    }
#endif
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    if (memcmp(coeffMantissa, other.coeffMantissa, sizeof(coeffMantissa)))
    {
      return false;
    }
#endif
#endif
    if( memcmp( lumaCoeff, other.lumaCoeff, sizeof( lumaCoeff ) ) )
    {
      return false;
    }
    if( memcmp( lumaClipp, other.lumaClipp, sizeof( lumaClipp ) ) )
    {
      return false;
    }
    if( memcmp( chromaCoeff, other.chromaCoeff, sizeof( chromaCoeff ) ) )
    {
      return false;
    }
    if( memcmp( chromaClipp, other.chromaClipp, sizeof( chromaClipp ) ) )
    {
      return false;
    }
    if( memcmp( filterCoeffDeltaIdx, other.filterCoeffDeltaIdx, sizeof( filterCoeffDeltaIdx ) ) )
    {
      return false;
    }
    if( memcmp( alfLumaCoeffFlag, other.alfLumaCoeffFlag, sizeof( alfLumaCoeffFlag ) ) )
    {
      return false;
    }
    if( memcmp( newFilterFlag, other.newFilterFlag, sizeof( newFilterFlag ) ) )
    {
      return false;
    }
    if( numAlternativesChroma != other.numAlternativesChroma )
    {
      return false;
    }
#if ALF_IMPROVEMENT
    if (numAlternativesLuma != other.numAlternativesLuma)
    {
      return false;
    }
    if (memcmp(numLumaFilters, other.numLumaFilters, sizeof(numLumaFilters)))
    {
      return false;
    }
#else
    if( numLumaFilters != other.numLumaFilters )
    {
      return false;
    }
#endif
    if( alfLumaCoeffDeltaFlag != other.alfLumaCoeffDeltaFlag )
    {
      return false;
    }
#if JVET_AK0123_ALF_COEFF_RESTRICTION
    if (memcmp(lumaScaleIdx, other.lumaScaleIdx, sizeof(lumaScaleIdx)))
    {
      return false;
    }
    if (memcmp(chromaScaleIdx, other.chromaScaleIdx, sizeof(chromaScaleIdx)))
    {
      return false;
    }
#endif

    return true;
  }

  bool operator!=( const AlfParam& other )
  {
    return !( *this == other );
  }

};

struct CcAlfFilterParam
{
  bool    ccAlfFilterEnabled[2];
  bool    ccAlfFilterIdxEnabled[2][MAX_NUM_CC_ALF_FILTERS];
  uint8_t ccAlfFilterCount[2];
#if JVET_AH0057_CCALF_COEFF_PRECISION
  int     ccAlfCoeffPrec[2];
#endif
  short   ccAlfCoeff[2][MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  int     newCcAlfFilter[2];
  int     numberValidComponents;
  CcAlfFilterParam()
  {
    reset();
  }
  void reset()
  {
    std::memset( ccAlfFilterEnabled, false, sizeof( ccAlfFilterEnabled ) );
    std::memset( ccAlfFilterIdxEnabled, false, sizeof( ccAlfFilterIdxEnabled ) );
    std::memset( ccAlfCoeff, 0, sizeof( ccAlfCoeff ) );
#if JVET_AH0057_CCALF_COEFF_PRECISION
    ccAlfCoeffPrec[0] = ccAlfCoeffPrec[1] = 0;
#endif
    ccAlfFilterCount[0] = ccAlfFilterCount[1] = MAX_NUM_CC_ALF_FILTERS;
    numberValidComponents = 3;
    newCcAlfFilter[0] = newCcAlfFilter[1] = 0;
  }
  const CcAlfFilterParam& operator = ( const CcAlfFilterParam& src )
  {
    std::memcpy( ccAlfFilterEnabled, src.ccAlfFilterEnabled, sizeof( ccAlfFilterEnabled ) );
    std::memcpy( ccAlfFilterIdxEnabled, src.ccAlfFilterIdxEnabled, sizeof( ccAlfFilterIdxEnabled ) );
    std::memcpy( ccAlfCoeff, src.ccAlfCoeff, sizeof( ccAlfCoeff ) );
    ccAlfFilterCount[0] = src.ccAlfFilterCount[0];
    ccAlfFilterCount[1] = src.ccAlfFilterCount[1];
    numberValidComponents = src.numberValidComponents;
    newCcAlfFilter[0] = src.newCcAlfFilter[0];
    newCcAlfFilter[1] = src.newCcAlfFilter[1];
#if JVET_AH0057_CCALF_COEFF_PRECISION
    ccAlfCoeffPrec[0] = src.ccAlfCoeffPrec[0];
    ccAlfCoeffPrec[1] = src.ccAlfCoeffPrec[1];
#endif
    return *this;
  }
};

#if JVET_W0066_CCSAO
struct CcSaoComParam
{
  bool     enabled   [MAX_NUM_COMPONENT];
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  bool     extChroma [MAX_NUM_COMPONENT];
#if JVET_AL0142_CCSAO_REUSE_CTU
  int      reusePrv  [MAX_NUM_COMPONENT];
#else
  bool     reusePrv  [MAX_NUM_COMPONENT];
#endif
  int      reusePrvId[MAX_NUM_COMPONENT];
#endif
  uint8_t  setNum    [MAX_NUM_COMPONENT];
  bool     setEnabled[MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM];
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
  bool     setType   [MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM];
#endif
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
  uint16_t candPos   [MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];  // BO 0: candPosY, EO 0: edgeDir, 1: edgeCmp, 2: dummy
#else
  uint16_t candPos   [MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM][MAX_NUM_LUMA_COMP];
#endif
  uint16_t bandNum   [MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];
  short    offset    [MAX_NUM_COMPONENT][MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
  CcSaoComParam()
  {
    reset();
  }
  void reset()
  {
    std::memset( enabled,    false, sizeof( enabled    ) );
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    std::memset( extChroma,  false, sizeof( extChroma  ) );
    std::memset( reusePrv,   false, sizeof( reusePrv   ) );
    std::memset( reusePrvId,     0, sizeof( reusePrvId ) );
#endif
    std::memset( setNum,         0, sizeof( setNum     ) );
    std::memset( setEnabled, false, sizeof( setEnabled ) );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    std::memset( setType,        0, sizeof( setType    ) );
#endif
    std::memset( candPos,        0, sizeof( candPos    ) );
    std::memset( bandNum,        0, sizeof( bandNum    ) );
    std::memset( offset,         0, sizeof( offset     ) );
  }
  void reset(ComponentID compID)
  {
    enabled   [compID] = false;
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    extChroma [compID] = false;
    reusePrv  [compID] = false;
    reusePrvId[compID] = 0;
#endif
    setNum    [compID] = 0;
    std::memset( setEnabled[compID], false, sizeof( setEnabled[compID]) );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    std::memset( setType   [compID],     0, sizeof( setType   [compID]) );
#endif
    std::memset( candPos   [compID],     0, sizeof( candPos   [compID]) );
    std::memset( bandNum   [compID],     0, sizeof( bandNum   [compID]) );
    std::memset( offset    [compID],     0, sizeof( offset    [compID]) );
  }
  const CcSaoComParam& operator = ( const CcSaoComParam& src )
  {
    std::memcpy( enabled,    src.enabled,    sizeof( enabled    ) );
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
    std::memcpy( extChroma,  src.extChroma,  sizeof( extChroma  ) );
    std::memcpy( reusePrv,   src.reusePrv,   sizeof( reusePrv   ) );
    std::memcpy( reusePrvId, src.reusePrvId, sizeof( reusePrvId ) );
#endif
    std::memcpy( setNum,     src.setNum,     sizeof( setNum     ) );
    std::memcpy( setEnabled, src.setEnabled, sizeof( setEnabled ) );
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
    std::memcpy( setType,    src.setType,    sizeof( setType    ) );
#endif
    std::memcpy( candPos,    src.candPos,    sizeof( candPos    ) );
    std::memcpy( bandNum,    src.bandNum,    sizeof( bandNum    ) );
    std::memcpy( offset,     src.offset,     sizeof( offset     ) );
    return *this;
  }
};

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
struct CcSaoPrvParam
{
  bool     enabled;
  bool     extChroma;
#if JVET_AL0142_CCSAO_REUSE_CTU
  int      reusePrv;
#else
  bool     reusePrv;
#endif
  int      reusePrvId;
  int      temporalId;
  uint8_t  setNum;
  bool     setEnabled[MAX_CCSAO_SET_NUM];
  bool     setType   [MAX_CCSAO_SET_NUM];
  uint16_t candPos   [MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];
  uint16_t bandNum   [MAX_CCSAO_SET_NUM][MAX_NUM_COMPONENT];  
  short    offset    [MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
#if JVET_AL0142_CCSAO_REUSE_CTU
  uint8_t  ccSaoControl[MAX_CCSAO_CTU_NUM];
#endif
  CcSaoPrvParam()
  {
    reset();
  }
  void reset()
  {
    enabled    = false;
    extChroma  = false;
    reusePrv   = false;
    reusePrvId = 0;
    temporalId = 0;
    setNum = 0;
    std::memset( setEnabled, false, sizeof( setEnabled ) );
    std::memset( setType,        0, sizeof( setType    ) );
    std::memset( candPos,        0, sizeof( candPos    ) );
    std::memset( bandNum,        0, sizeof( bandNum    ) );
    std::memset( offset,         0, sizeof( offset     ) );
#if JVET_AL0142_CCSAO_REUSE_CTU
    std::memset( ccSaoControl,   0, sizeof( ccSaoControl ) );
#endif
  }
  const CcSaoPrvParam& operator = ( const CcSaoPrvParam& src )
  {
    enabled    = src.enabled;
    extChroma  = src.extChroma;
    reusePrv   = src.reusePrv;
    reusePrvId = src.reusePrvId;
    temporalId = src.temporalId;
    setNum     = src.setNum;
    std::memcpy( setEnabled, src.setEnabled, sizeof( setEnabled ) );
    std::memcpy( setType,    src.setType,    sizeof( setType    ) );
    std::memcpy( candPos,    src.candPos,    sizeof( candPos    ) );
    std::memcpy( bandNum,    src.bandNum,    sizeof( bandNum    ) );
    std::memcpy( offset,     src.offset,     sizeof( offset     ) );
#if JVET_AL0142_CCSAO_REUSE_CTU
    std::memcpy( ccSaoControl, src.ccSaoControl, sizeof( ccSaoControl ) );
#endif
    return *this;
  }
};
#endif
#endif

#if JVET_AK0065_TALF
const Position templateShape0[NUM_TALF_COEFF] = { 
  Position( 0, 0), Position( 1, 0), Position( 0, 1), Position( 1, 1), Position(-1, 1), Position( 2, 0), Position( 0, 2),
  Position(-2, 1), Position( 2, 1), Position(-1, 2), Position( 1, 2), Position( 3, 0), Position( 0, 3)
};
const Position templateShape1[NUM_TALF_COEFF] = { 
  Position( 0, 0), Position( 1, 0), Position( 0, 1), Position( 1, 1), Position(-1, 1), Position( 2, 0), Position( 0, 2),
  Position( 3, 0), Position( 0, 3), Position( 4, 0), Position( 0, 4), Position( 5, 0), Position( 0, 5)
};
#endif
//! \}

#endif  // end of #ifndef  __ALFPARAMETERS__
