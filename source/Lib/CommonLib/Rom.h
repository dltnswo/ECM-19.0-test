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

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#ifndef __ROM__
#define __ROM__

#include "CommonDef.h"
#include "Common.h"

#include <stdio.h>
#include <iostream>
#if JVET_AH0209_PDP
#include <fstream>
#include <unordered_map>
#endif

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================
void         initROM();
void         destroyROM();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================
#if JVET_AK0061_PDP_MPM
extern const bool g_sBlkPdpMode[NUM_LUMA_MODE];
extern const bool g_bBlkPdpMode[NUM_LUMA_MODE];
#endif
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
extern const int g_modeGroupExt[(PDP_NUM_MODES - 3) * 2 + 3];
#endif
#if JVET_AH0209_PDP
extern const int g_sizeData[PDP_NUM_SIZES][11];
extern std::unordered_map<int, int> g_size;

extern int16_t*** g_pdpFilters[PDP_NUM_GROUPS][PDP_NUM_SIZES];
extern int g_validSize[PDP_NUM_SIZES];
void createPdpFilters();
void destroyPdpFilters();
#if JVET_AI0208_PDP_MIP
extern const int16_t g_filterDataPdpMip[];
extern int16_t*** g_pdpFiltersMip[PDP_NUM_GROUPS][PDP_NUM_SIZES];
extern int g_validSizeMip[PDP_NUM_SIZES];
void createMipFilters();
void destroyMipFilters();
#endif
#endif

#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_W0119_LFNST_EXTENSION || EXTENDED_LFNST
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
#if JVET_AJ0237_INTERNAL_12BIT
extern       int16_t* g_resiBorderTemplateLFNST[NUM_NSPT_BLOCK_TYPES][6][6][210];
#else
extern       int8_t * g_resiBorderTemplateLFNST[NUM_NSPT_BLOCK_TYPES][6][6][210];
#endif
#else
#if JVET_AJ0237_INTERNAL_12BIT
extern       int16_t* g_resiBorderTemplateLFNST[6][6][210];
#else
extern       int8_t * g_resiBorderTemplateLFNST[6][6][210];
#endif
#endif
#else
extern       int8_t * g_resiBorderTemplateLFNST[6][6][16];
#endif
#if JVET_AJ0237_INTERNAL_12BIT
extern       int16_t* g_resiBorderTemplate[6][6][NUM_TRANS_TYPE * NUM_TRANS_TYPE];
#else
extern       int8_t * g_resiBorderTemplate[6][6][NUM_TRANS_TYPE*NUM_TRANS_TYPE];
#endif
#else
extern const int8_t * g_resiBorderTemplate[6][6][NUM_TRANS_TYPE*NUM_TRANS_TYPE];
#endif
#endif

// flexible conversion from relative to absolute index
struct ScanElement
{
  uint32_t idx;
  uint16_t x;
  uint16_t y;
};

extern       uint32_t   g_log2SbbSize[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2];
extern ScanElement
  *g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#if JVET_W0119_LFNST_EXTENSION
extern       ScanElement   g_coefTopLeftDiagScan8x8  [ MAX_CU_DEPTH + 1 ][  64 ];
extern       ScanElement   g_coefTopLeftDiagScan16x16[ MAX_CU_DEPTH + 1 ][ 256 ];
#else
extern       ScanElement   g_coefTopLeftDiagScan8x8[ MAX_CU_SIZE / 2 + 1 ][ 64 ];
#endif

extern const int g_quantScales   [2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_invQuantScales[2/*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)
#if TU_256
static const int g_numTransformMatrixSizes = 8;
#else
static const int g_numTransformMatrixSizes = 6;
#endif
#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
static const int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = { 14, 6 };
#else
static const int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };
#endif

#if LMS_LINEAR_MODEL || TRANSFORM_SIMD_OPT
extern int8_t          g_aucLog2[MAX_CU_SIZE + 1];
#endif
#if LMS_LINEAR_MODEL
extern int8_t          g_aucNextLog2[MAX_CU_SIZE + 1];
extern int8_t          g_aucPrevLog2[MAX_CU_SIZE + 1];
#endif
#if MMLM && !LMS_LINEAR_MODEL
extern int8_t          g_aucPrevLog2[MAX_CU_SIZE + 1];
#endif
// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

#if TU_256
extern const uint32_t   g_uiGroupIdx[];
#else
extern const uint32_t   g_uiGroupIdx[ MAX_TB_SIZEY ];
#endif
extern const uint32_t   g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
extern const uint32_t g_auiGoRiceParsCoeffGTN[GTN_MAXSUM];
#endif
extern const uint32_t   g_auiGoRiceParsCoeff     [ 32 ];
inline uint32_t g_auiGoRicePosCoeff0(int st, uint32_t ricePar)
{
#if TCQ_8STATES
	return (1 + (st & 1)) << ricePar;
#else
  return (st < 2 ? 1 : 2) << ricePar;
#endif
}

#if TCQ_8STATES
extern const uint64_t g_stateTransTab[3];
#endif
// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const uint8_t  g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];
extern const uint8_t  g_aucIntraModeNumFast_UseMPM   [MAX_CU_DEPTH];
extern const uint8_t  g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const uint8_t  g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];


// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================


extern const TMatrixCoeff g_trCoreDCT2P2  [TRANSFORM_NUMBER_OF_DIRECTIONS][  2][  2];
extern const TMatrixCoeff g_trCoreDCT2P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT2P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT2P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT2P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
extern const TMatrixCoeff g_trCoreDCT2P64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];
#if TU_256
extern const TMatrixCoeff g_trCoreDCT2P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];
#endif

extern const TMatrixCoeff g_trCoreDCT8P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDCT8P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDCT8P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDCT8P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
#if TU_256
extern const TMatrixCoeff g_trCoreDCT8P64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];
extern const TMatrixCoeff g_trCoreDCT8P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];
#endif

extern const TMatrixCoeff g_trCoreDST7P4  [TRANSFORM_NUMBER_OF_DIRECTIONS][  4][  4];
extern const TMatrixCoeff g_trCoreDST7P8  [TRANSFORM_NUMBER_OF_DIRECTIONS][  8][  8];
extern const TMatrixCoeff g_trCoreDST7P16 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 16][ 16];
extern const TMatrixCoeff g_trCoreDST7P32 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 32][ 32];
#if TU_256
extern const TMatrixCoeff g_trCoreDST7P64 [TRANSFORM_NUMBER_OF_DIRECTIONS][ 64][ 64];
extern const TMatrixCoeff g_trCoreDST7P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];

extern TMatrixCoeff g_trCoreDCT2P256[256][256];
extern TMatrixCoeff g_trCoreDCT8P256[256][256];
extern TMatrixCoeff g_trCoreDST7P256[256][256];
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
extern int g_bdofWeight[1600];
#endif
#if JVET_W0103_INTRA_MTS
extern TMatrixCoeff g_aiTr2[NUM_TRANS_TYPE][2][2];
extern TMatrixCoeff g_aiTr4[NUM_TRANS_TYPE][4][4];
extern TMatrixCoeff g_aiTr8[NUM_TRANS_TYPE][8][8];
extern TMatrixCoeff g_aiTr16[NUM_TRANS_TYPE][16][16];
extern TMatrixCoeff g_aiTr32[NUM_TRANS_TYPE][32][32];
extern TMatrixCoeff g_aiTr64[NUM_TRANS_TYPE][64][64];
extern TMatrixCoeff g_aiTr128[NUM_TRANS_TYPE][128][128];
extern TMatrixCoeff g_aiTr256[NUM_TRANS_TYPE][256][256];

extern const uint8_t g_aucIpmToTrSet[16][36];

#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
extern const uint8_t g_aucIpmToTrSetModDimd[3][12][67];
extern const uint8_t g_aucIpmToTrSetModTimd[3][10][67];
extern const uint8_t g_aucIpmToTrSetModMip[3][12][67];
extern const uint8_t g_aucIpmToTrSetModTmp[3][12][67];
extern const uint8_t g_aucIpmToTrSetModSgpm[3][11][67];
extern const uint8_t g_aucIpmToTrSetModEip[3][10][67];
extern const uint8_t g_aucIpmToTrSetModPnn[3][11][67];
#endif

#if JVET_AJ0257_IMPLICIT_MTS_LUT
extern const uint8_t g_aucImplicitToTrSet[16][35];
extern const uint8_t g_aucImplicitTrIdxToTr[36][2];
#endif

#if JVET_Y0142_ADAPT_INTRA_MTS
extern const uint8_t g_aucTrSet[80][6];
#else
extern const uint8_t g_aucTrSet[80][4];
#endif
extern const int8_t  g_aiIdLut[3][3];
extern const uint8_t g_aucTrIdxToTr[25][2];
#endif
#if JVET_W0119_LFNST_EXTENSION
extern const     int8_t   g_lfnst16x16[ 35 ][ 3 ][ L16H ][ L16W ];
extern const     int8_t   g_lfnst8x8  [ 35 ][ 3 ][  L8H ][  L8W ];
extern const     int8_t   g_lfnst4x4  [ 35 ][ 3 ][   16 ][   16 ];
#if AHG7_LN_TOOLOFF_CFG
extern const     int8_t   g_vvcLfnst8x8[ 4 ][ 2 ][ 16 ][ 48 ];
extern const     int8_t   g_vvcLfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];
#endif
#else
#if EXTENDED_LFNST
extern const     int8_t   g_lfnst8x8[ 35 ][ 3 ][ 64 ][ 64 ];
extern const     int8_t   g_lfnst4x4[ 35 ][ 3 ][ 16 ][ 16 ];
#else
extern const     int8_t   g_lfnst8x8[ 4 ][ 2 ][ 16 ][ 48 ];
extern const     int8_t   g_lfnst4x4[ 4 ][ 2 ][ 16 ][ 16 ];
#endif
#endif

#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
extern const     uint8_t  g_lfnstLut[NUM_LFNST_INTRA_MODES];
#if AHG7_LN_TOOLOFF_CFG
extern const     uint8_t  g_vvcLfnstLut[ NUM_LFNST_INTRA_MODES ];
#endif
#else
extern const     uint8_t  g_lfnstLut[ NUM_INTRA_MODE + NUM_EXT_LUMA_MODE - 1 ];
#endif

#if JVET_AC0130_NSPT
extern const     uint8_t  g_nsptLut[ 97 ];                      // 97= NUM_LUMA_MODE + NUM_EXT_LUMA_MODE - 1

#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
extern const     int8_t   g_nspt4x4[ NUM_NSPT_CLUSTERS_4x4 ][ 16 ][ 16 ];
extern const     int8_t   g_nspt4x8[ NUM_NSPT_CLUSTERS_4x8 ][ 20 ][ 32 ];
extern const     int8_t   g_nspt8x4[ NUM_NSPT_CLUSTERS_8x4 ][ 20 ][ 32 ];
extern const     int8_t   g_nspt8x8[ NUM_NSPT_CLUSTERS_8x8 ][ 32 ][ 64 ];
extern const     int8_t   g_nspt4x16[ NUM_NSPT_CLUSTERS_4x16 ][ 24 ][ 64 ];
extern const     int8_t   g_nspt16x4[ NUM_NSPT_CLUSTERS_16x4 ][ 24 ][ 64 ];
extern const     int8_t   g_nspt8x16[ NUM_NSPT_CLUSTERS_8x16 ][ 40 ][ 128 ];
extern const     int8_t   g_nspt16x8[ NUM_NSPT_CLUSTERS_16x8 ][ 40 ][ 128 ];
#if JVET_AE0086_LARGE_NSPT
extern const     int8_t   g_nspt4x32[ NUM_NSPT_CLUSTERS_4x32 ][ 20 ][ 128 ];
extern const     int8_t   g_nspt32x4[ NUM_NSPT_CLUSTERS_32x4 ][ 20 ][ 128 ];
extern const     int8_t   g_nspt8x32[ NUM_NSPT_CLUSTERS_8x32 ][ 24 ][ 256 ];
extern const     int8_t   g_nspt32x8[ NUM_NSPT_CLUSTERS_32x8 ][ 24 ][ 256 ];
#endif
extern const     uint8_t  g_nsptIdx4x4[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx4x8[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx8x4[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx8x8[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx4x16[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx16x4[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx8x16[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx16x8[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
#if JVET_AE0086_LARGE_NSPT
extern const     uint8_t  g_nsptIdx4x32[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx32x4[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx8x32[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
extern const     uint8_t  g_nsptIdx32x8[ 35 ][ NUM_NSPT_BLOCK_TYPES ][ 3 ];
#endif
#else
extern const     int8_t   g_nspt4x4[ 35 ][ 3 ][ 16 ][ 16 ];
extern const     int8_t   g_nspt4x8[ 35 ][ 3 ][ 20 ][ 32 ];
extern const     int8_t   g_nspt8x4[ 35 ][ 3 ][ 20 ][ 32 ];
extern const     int8_t   g_nspt8x8[ 35 ][ 3 ][ 32 ][ 64 ];
extern const     int8_t   g_nspt4x16[ 35 ][ 3 ][ 24 ][ 64 ];
extern const     int8_t   g_nspt16x4[ 35 ][ 3 ][ 24 ][ 64 ];
extern const     int8_t   g_nspt8x16[ 35 ][ 3 ][ 40 ][ 128 ];
extern const     int8_t   g_nspt16x8[ 35 ][ 3 ][ 40 ][ 128 ];
#if JVET_AE0086_LARGE_NSPT
extern const     int8_t   g_nspt4x32[ 35 ][ 3 ][ 20 ][ 128 ];
extern const     int8_t   g_nspt32x4[ 35 ][ 3 ][ 20 ][ 128 ];
extern const     int8_t   g_nspt8x32[ 35 ][ 3 ][ 24 ][ 256 ];
extern const     int8_t   g_nspt32x8[ 35 ][ 3 ][ 24 ][ 256 ];
#endif
#endif

#endif
#if JVET_AH0209_PDP
extern int16_t g_weights4x4[11][16][36];
extern int16_t g_weightsShort4x4[8][16][20];
extern int16_t g_weights4x8[11][32][52];
extern int16_t g_weightsShort4x8[8][32][28];
extern int16_t g_weights8x4[11][32][52];
extern int16_t g_weightsShort8x4[8][32][28];
extern int16_t g_weights8x8[11][64][68];
extern int16_t g_weightsShort8x8[8][64][36];
extern int16_t g_weights4x16[11][64][84];
extern int16_t g_weightsShort4x16[8][64][44];
extern int16_t g_weights16x4[11][64][84];
extern int16_t g_weightsShort16x4[8][64][44];
extern int16_t g_weights8x16[11][128][100];
extern int16_t g_weightsShort8x16[8][128][52];
extern int16_t g_weights16x8[11][128][100];
extern int16_t g_weightsShort16x8[8][128][52];
extern int16_t g_weights16x16[11][256][132];
extern int16_t g_weightsShort16x16[8][256][68];

extern int16_t g_weights16x32[7][256][97];
extern int16_t g_weightsShort16x32[4][256][49];
extern int16_t g_weights32x16[7][256][97];
extern int16_t g_weightsShort32x16[4][256][49];
extern int16_t g_weights32x32[7][256][129];
extern int16_t g_weightsShort32x32[4][256][65];
#endif
// ====================================================================================================================
// Misc.
// ====================================================================================================================
extern SizeIndexInfo* gp_sizeIdxInfo;

extern const int       g_ictModes[2][4];

inline bool is34( const SizeType& size )
{
  return ( size & ( ( int64_t ) 1 << ( floorLog2(size) - 1 ) ) );
}

inline bool is58( const SizeType& size )
{
  return ( size & ( ( int64_t ) 1 << ( floorLog2(size) - 2 ) ) );
}

inline bool isNonLog2BlockSize( const Size& size )
{
  return ( ( 1 << floorLog2(size.width) ) != size.width ) || ( ( 1 << floorLog2(size.height) ) != size.height );
}

inline bool isNonLog2Size( const SizeType& size )
{
  return ( ( 1 << floorLog2(size) ) != size );
}

extern UnitScale     g_miScaling; // scaling object for motion scaling

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
#include "dtrace.h"
extern CDTrace* g_trace_ctx;
#endif

const char* nalUnitTypeToString(NalUnitType type);

extern const char *MatrixType   [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const int g_quantTSDefault4x4   [4*4];
extern const int g_quantIntraDefault8x8[8*8];
extern const int g_quantInterDefault8x8[8*8];

extern const uint32_t g_scalingListSize [SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListId[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern MsgLevel g_verbosity;
#if MMLM
extern int g_aiLMDivTableLow[];
extern int g_aiLMDivTableHigh[];
#endif

extern const int8_t g_bcwLog2WeightBase;
extern const int8_t g_bcwWeightBase;
extern const int8_t g_bcwWeights[BCW_NUM];
extern const int8_t g_bcwSearchOrder[BCW_NUM];
extern       int8_t g_bcwCodingOrder[BCW_NUM];
extern       int8_t g_bcwParsingOrder[BCW_NUM];
#if JVET_AB0079_TM_BCW_MRG
extern const int8_t g_bcwMrgWeights[BCW_MRG_NUM];
extern       int8_t g_bcwMrgParsingOrder[BCW_MRG_NUM];
#endif

class CodingStructure;
int8_t getBcwWeight(uint8_t bcwIdx, uint8_t uhRefFrmList);
void resetBcwCodingOrder(bool bRunDecoding, const CodingStructure &cs);
uint32_t deriveWeightIdxBits(uint8_t bcwIdx);

constexpr uint8_t g_tbMax[257] = { 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8 };

#if SIGN_PREDICTION
#if !JVET_W0103_INTRA_MTS
extern const int8_t    g_initRomSignPred[];
#endif
extern const int32_t   g_aucIdxTrCombo[5][5][5][NUM_INTRA_MODE];
extern const int32_t   g_aucNumTrCombo[5][5];
#endif

//! \}


extern bool g_mctsDecCheckEnabled;

class  Mv;
#if CTU_256
extern Mv   g_reusedUniMVs        [MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2][33];
extern bool g_isReusedUniMVsFilled[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];
#else
extern Mv   g_reusedUniMVs[32][32][8][8][2][33];
extern bool g_isReusedUniMVsFilled[32][32][8][8];
#endif
#if INTER_LIC
extern Mv   g_reusedUniMVsLIC        [MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2][33];
extern bool g_isReusedUniMVsFilledLIC[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];
#endif

extern uint16_t g_paletteQuant[57];
extern uint8_t g_paletteRunTopLut[5];
extern uint8_t g_paletteRunLeftLut[5];

#if CTU_256
const int g_IBCBufferSize = 256 * 128 * 4;
#else
const int g_IBCBufferSize = 256 * 128;
#endif

void initGeoTemplate();
extern int16_t** g_geoParams;
#if JVET_AB0155_SGPM
extern int16_t *g_geoWeights[TOTAL_GEO_BLENDING_NUM][GEO_NUM_PRESTORED_MASK];
extern int      g_bld2Width[TOTAL_GEO_BLENDING_NUM];
#elif JVET_AA0058_GPM_ADAPTIVE_BLENDING
extern int16_t*  g_geoWeights   [GEO_BLENDING_NUM][GEO_NUM_PRESTORED_MASK];
extern int       g_bld2Width          [GEO_BLENDING_NUM];
#else
extern int16_t*  g_geoWeights   [GEO_NUM_PRESTORED_MASK];
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
extern Pel*      g_geoWeightsTpl[GEO_NUM_PRESTORED_MASK];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
extern Pel*      g_geoEncSadMask[GEO_NUM_PRESTORED_MASK];
#else
extern int16_t*  g_geoEncSadMask[GEO_NUM_PRESTORED_MASK];
#endif
#if JVET_AJ0107_GPM_SHAPE_ADAPT
extern int16_t   g_weightOffset       [GEO_TOTAL_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
#else
extern int16_t   g_weightOffset       [GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
#endif
extern int8_t    g_angle2mask         [GEO_NUM_ANGLES];
extern int8_t    g_dis[GEO_NUM_ANGLES];
extern int8_t    g_angle2mirror[GEO_NUM_ANGLES];


#if JVET_AB0155_SGPM
#if JVET_AJ0107_GPM_SHAPE_ADAPT
extern int16_t   g_weightOffsetEx[GEO_TOTAL_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE_EX][GEO_NUM_CU_SIZE_EX][2];
#else
extern int16_t   g_weightOffsetEx[GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE_EX][GEO_NUM_CU_SIZE_EX][2];
extern int8_t    g_sgpmSplitDir[GEO_NUM_PARTITION_MODE];
#endif
#endif
#if JVET_AJ0107_GPM_SHAPE_ADAPT
extern int8_t    g_gpmSplitDir[GEO_NUM_CU_SHAPES][GEO_NUM_PARTITION_MODE];

extern int8_t    g_sgpmSplitDir[SGPM_TOTAL_NUM_PARTITIONS];
extern int8_t    g_ibcGpmSplitDir[IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM + IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM];
extern int8_t    g_ibcGpmSplitDirFirstSetRank[IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM + IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM];
#endif
#if JVET_Y0065_GPM_INTRA
extern int8_t    g_geoAngle2IntraAng  [GEO_NUM_ANGLES];
#endif
#if (JVET_W0097_GPM_MMVD_TM && TM_MRG) || JVET_Y0065_GPM_INTRA
extern uint8_t   g_geoTmShape[2][GEO_NUM_ANGLES];
#endif
#if MULTI_HYP_PRED
extern const int g_addHypWeight[MULTI_HYP_PRED_NUM_WEIGHTS];
#endif
#if JVET_W0066_CCSAO
extern const int8_t g_ccSaoCandPosX[MAX_NUM_LUMA_COMP][MAX_CCSAO_CAND_POS_Y];
extern const int8_t g_ccSaoCandPosY[MAX_NUM_LUMA_COMP][MAX_CCSAO_CAND_POS_Y];
#endif
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
extern const int8_t g_ccSaoEdgePosX[MAX_CCSAO_EDGE_DIR][2];
extern const int8_t g_ccSaoEdgePosY[MAX_CCSAO_EDGE_DIR][2];
extern const short  g_ccSaoEdgeThr [MAX_CCSAO_EDGE_IDC][MAX_CCSAO_EDGE_THR];
extern const int8_t g_ccSaoEdgeNum [MAX_CCSAO_EDGE_IDC][2];
extern const int8_t g_ccSaoBandTab [MAX_CCSAO_BAND_IDC][2];
#else
extern const int8_t g_ccSaoEdgeTypeX[CCSAO_EDGE_TYPE][2];
extern const int8_t g_ccSaoEdgeTypeY[CCSAO_EDGE_TYPE][2];
extern const short  g_ccSaoQuanValue[CCSAO_QUAN_NUM];
#endif
#endif
#if JVET_V0130_INTRA_TMP
extern unsigned int g_uiDepth2Width[5];
#endif
#if JVET_X0149_TIMD_DIMD_LUT
extern int g_gradDivTable[16];
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
extern int g_rmvfMultApproxTbl[3 << sizeof(int64_t)];
#endif
#if JVET_AA0126_GLM
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
extern const int8_t g_glmPattern[2][NUM_GLM_PATTERN][6];
#else
extern const int8_t g_glmPattern[NUM_GLM_PATTERN][6];
#endif
#endif
#if JVET_AC0112_IBC_GPM
#if !JVET_AJ0107_GPM_SHAPE_ADAPT
extern const int8_t g_ibcGpmFirstSetSplitDirToIdx[GEO_NUM_PARTITION_MODE];
extern const int8_t g_ibcGpmFirstSetSplitDir[IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM];
extern const int8_t g_ibcGpmSecondSetSplitDir[GEO_NUM_PARTITION_MODE];
#endif
#endif
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
extern int g_ibcMbvdCandOffsets[IBC_MBVD_AD_STEP_NUM];
extern int g_ibcMbvdStepCandIdxList[IBC_MBVD_AD_NUM >> IBC_MBVD_LOG2_START_STEP];
extern int g_ibcMbvdNeiOffsets[IBC_MBVD_NEI_NUM * 2];
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
extern const int8_t g_amvpSbTmvp_mvd_dir[2][8];
extern const int8_t g_amvpSbTmvp_mvd_offset[6];
extern uint32_t g_picAmvpSbTmvpEnabledArea;
#endif
#if JVET_AG0058_EIP
#if JVET_AH0086_EIP_BIAS_AND_CLIP
extern const Position g_eipFilter[NUM_EIP_SHAPE][EIP_FILTER_TAP - 1];
#else
extern const Position g_eipFilter[NUM_EIP_SHAPE][EIP_FILTER_TAP];
#endif
extern const EIPInfo  g_eipInfoLut[4][4][9];
#if JVET_AJ0082_MM_EIP
extern const EIPInfo  g_mmEipInfoLut[4][4][9];
#endif
#endif
#if JVET_AG0276_LIC_SLOPE_ADJUST
extern const int g_licSlopeDeltaSet[LIC_SLOPE_MAX_NUM_DELTA + 1];
#endif
#if JVET_AK0059_MDIP
extern uint64_t g_intraModeCost[EXT_VDIA_IDX + 1];
#endif
#if JVET_AJ0061_TIMD_MERGE
extern uint64_t g_timdMrgCost[EXT_VDIA_IDX + 1];
extern const std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> g_timdMergeOffsetXTable;
extern const std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> g_timdMergeOffsetYTable;
#endif
#endif  //__TCOMROM__

