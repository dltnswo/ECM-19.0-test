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

 /** \file     Rom.cpp
     \brief    global variables & functions
 */

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

#if JVET_AH0209_PDP
#include "RomMIPFIlters.h"
#endif
#if JVET_AI0208_PDP_MIP
const int16_t g_filterDataPdpMip[] = {
#include "RomMIPFIlters2.h"
};
#endif

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================
#if JVET_AH0209_PDP
#if JVET_AK0061_PDP_MPM
const bool g_sBlkPdpMode[NUM_LUMA_MODE] = 
{
  1,1,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1,0,1,0,1,0,
  1,0,1 
};
const bool g_bBlkPdpMode[NUM_LUMA_MODE] = 
{
  1,1,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1,0,0,0,1,0,
  0,0,1 
};
#endif 
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
const int g_modeGroupExt[(PDP_NUM_MODES - 3) * 2 + 3] = { 0,1,2,-1,-1,-1,4,-1,-1,-1,6,-1,-1,-1,8,-1,-1,-1,10,-1,-1,-1,12,-1,-1,-1,14,-1,-1,-1,16,-1,-1,-1,18,-1,-1,-1,20,-1,-1,-1,22,-1,-1,-1,24,-1,-1,-1,26,-1,-1,-1,28,-1,-1,-1,30,-1,-1,-1,32,
  -1,-1,-1,34,-1,-1,-1,36,-1,-1,-1,38,-1,-1,-1,40,-1,-1,-1,42,-1,-1,-1,44,-1,-1,-1,46,-1,-1,-1,48,-1,-1,-1,50,-1,-1,-1,52,-1,-1,-1,54,-1,-1,-1,56,-1,-1,-1,58,-1,-1,-1,60,-1,-1,-1,62,-1,-1,-1,64,-1,-1,-1,66 };
#endif
const int g_sizeData[ PDP_NUM_SIZES ][ 11 ] =
{
  { 16,     4,    4,   4,  4,  2,  2,    36,  0, 0,     20 },
  { 32,     4,	  8,	 4,	 8,	 2,	 2,	   52,	0, 0,     28 },
  { 32,	    8,	  4,	 8,	 4,	 2,	 2,	   52,	0, 0,     28 },
  { 64,	    8,	  8,	 8,	 8,	 2,	 2,	   68,	0, 0,     36 },
  { 64,	    4,	 16,	 4,	16,	 2,	 2,	   84,	0, 0,     44 },
  { 64,	   16,	  4,	16,	 4,	 2,	 2,	   84,	0, 0,     44 },
  { 128,    4,   32,	 4,	32,	 2,	 2,	  148,	0, 0,     76 },
  { 128,   32,	  4,	32,	 4,	 2,	 2,	  148,	0, 0,     76 },
  { 128,    8,	 16,	 8,	16,	 2,	 2,	  100,	0, 0,     52 },
  { 128,   16,	  8,	16,	 8,	 2,	 2,	  100,	0, 0,     52 },
  { 256,   16,	 16,	16,	16,	 2,	 2,	  132,	0, 0,     68 },
  { 256,    8,   32,	 8,	32,	 2,	 2,	  164,	0, 0,     84 },
  { 256,   32,	  8,	32,	 8,	 2,	 2,	  164,	0, 0,     84 },
  { 512,   16,	 32,	16,	32,	 1,	 1,	   97,	0, 0,     49 },
  { 512,   32,	 16,	32,	16,	 1,	 1,	   97,	0, 0,     49 },
  { 1024,  32,   32,  32, 32,  1,	 1,	  129,	0, 0,     65 },
  { 1024,  64,	 64,	32,	32,	 2,	 2,	  516,	1, 1,    260 }, //dont use anything more than 64x64 at this point
  { 1024, 128,	128,	32,	32,	 2,	 2,	 1028,	2, 2,    516 }
};

std::unordered_map<int, int> g_size
{
  { 1028, 0 },
  { 1032, 1 },
  { 2052, 2 },
  { 2056, 3 },
  { 1040, 4 },
  { 4100, 5 },
  { 1056, 6 },
  { 8196, 7 },
  { 2064, 8 },
  { 4104, 9 },
  { 4112,10 },
  { 2080,11 },
  { 8200,12 },
  { 4128,13 },
  { 8208,14 },
  { 8224,15 },
  { 16448,16 },
  { 32896,17 }
};

const int g_modeGroupSym[ PDP_NUM_MODES ] = { 0, 1, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34 }; //BR
int16_t*** g_pdpFilters[ PDP_NUM_GROUPS ][ PDP_NUM_SIZES ] = { {nullptr} };
int g_validSize[ PDP_NUM_SIZES ] = { 0 };
#if JVET_AI0208_PDP_MIP
int16_t*** g_pdpFiltersMip[ PDP_NUM_GROUPS ][ PDP_NUM_SIZES ] = { {nullptr} };
int g_validSizeMip[ PDP_NUM_SIZES ] = { 0 };

void createMipFilters()
{
  auto createBuf = []( int16_t***& f, int w, int h, int len, int aligned = 1, int logRowInterleave = 0 )
  {
    if( !f )
    {
      const int lenAlign = aligned > 1 ? ( len + aligned - 1 ) / aligned * aligned : len;

      auto memSize = lenAlign * h * w;

      int16_t* pMem = new int16_t[ memSize ];
      ::memset( pMem, 0, sizeof( int16_t ) * memSize );

      const int buffW = w >> logRowInterleave;

      f = new int16_t * *[ h ];
      for( int i = 0; i < h; ++i )
      {
        f[ i ] = new int16_t * [ buffW ];
        for( int j = 0; j < buffW; ++j )
        {
          f[ i ][ j ] = pMem;
          pMem += lenAlign << logRowInterleave;
        }
      }
    }
  };

  auto transposeMatrixLine = []( int16_t* src, int16_t* dst, int w, int h, int ts )
  {
    const int stride1 = ts + w * 2;
    const int stride2 = ts + h * 2;

    AreaBuf<int16_t> srcCorner( src, stride1, ts, ts );
    AreaBuf<int16_t> srcTop( src + ts, stride1, w * 2, ts );
    AreaBuf<int16_t> srcLeft( src + ts * stride1, ts, ts, h * 2 );
    AreaBuf<int16_t> dstCorner( dst, stride2, ts, ts );
    AreaBuf<int16_t> dstTop( dst + ts, stride2, h * 2, ts );
    AreaBuf<int16_t> dstLeft( dst + ts * stride2, ts, ts, w * 2 );

    dstCorner.copyTranspose( srcCorner );
    dstTop.copyTranspose( srcLeft );
    dstLeft.copyTranspose( srcTop );
  };

  auto interleaveMatrixLines = []( AreaBuf<int16_t>& src, AreaBuf<int16_t>& dst, int sublen )
  {
    const int ratio = src.height / dst.height;

    for( auto i = 0; i < dst.height; ++i )
    {
      for( auto j = 0; j < dst.width; j += ratio * sublen )
      {
        int16_t* pSubDst = &dst.at( j, i );
        AreaBuf<int16_t> subDst( pSubDst, sublen, ratio );
        const int srcSubLen = std::min( sublen, int( src.width - j / ratio ) );
        AreaBuf<int16_t> subSrc = src.subBuf( Position( j / ratio, i * ratio ), Size( srcSubLen, ratio ) );

        subDst.copyFromFill( subSrc, sublen, ratio, 0 );
      }

    }
  };

  static int16_t readBuffer[ 266240 ];
  static int16_t readBufferT[ 266240 ];
  const int16_t* pData = g_filterDataPdpMip;

  while( pData < g_filterDataPdpMip + sizeof( g_filterDataPdpMip ) / sizeof( int16_t ) )
  {
    int tuWidth, tuHeight, groupID;
    tuWidth = *pData++;
    tuHeight = *pData++;
    groupID = *pData++;

    const int sizeID = g_size[ ( tuWidth << 8 ) + tuHeight ];
    g_validSizeMip[ sizeID ] = 1;
    g_validSizeMip[ g_size[ ( tuHeight << 8 ) + tuWidth ] ] = 1;
    const int dsWidth = g_sizeData[ sizeID ][ 3 ], dsHeight = g_sizeData[ sizeID ][ 4 ];

    bool shortLen = false;
    const int len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];
    int16_t***& f = g_pdpFiltersMip[ groupID ][ sizeID ];

    createBuf( f, dsWidth, dsHeight, len, 8, 2 );

    int16_t***& fSym = g_pdpFiltersMip[ groupID + 16 ][ g_size[ ( tuHeight << 8 ) + tuWidth ] ];

    createBuf( fSym, dsHeight, dsWidth, len, 8, 2 );

    memcpy( readBuffer, pData, sizeof( int16_t ) * len * dsWidth * dsHeight );

    pData += len * dsWidth * dsHeight;
    AreaBuf<int16_t> srcBuf( readBuffer, len, dsWidth * dsHeight );
    AreaBuf<int16_t> dstBuf( f[ 0 ][ 0 ], ( len + 7 ) / 8 * 8 * 4, dsWidth * dsHeight / 4 );

    interleaveMatrixLines( srcBuf, dstBuf, 8 );

    const int tsX = g_sizeData[ sizeID ][ 5 ];
    const int tsY = g_sizeData[ sizeID ][ 6 ];
    CHECK( tsX != tsY, "Symmetry not true" );

    for( auto i = 0; i < dsHeight; ++i )
    {
      for( auto j = 0; j < dsWidth; ++j )
      {
        transposeMatrixLine( readBuffer + ( i * dsWidth + j ) * len, readBufferT + ( j * dsHeight + i ) * len, dsWidth, dsHeight, tsX );
      }
    }

    AreaBuf<int16_t> srcBufT( readBufferT, len, dsWidth * dsHeight );
    AreaBuf<int16_t> dstBufT( fSym[ 0 ][ 0 ], ( len + 7 ) / 8 * 8 * 4, dsWidth * dsHeight / 4 );
    interleaveMatrixLines( srcBufT, dstBufT, 8 );
  }
}
#endif
void createPdpFilters()
{
  for( int sizeID = 0; sizeID < 16; sizeID++ )
  {
    if( sizeID == 6 || sizeID == 7 || sizeID == 11 || sizeID == 12 )
    {
      continue;
    }

    const int dsWidth = g_sizeData[ sizeID ][ 3 ], dsHeight = g_sizeData[ sizeID ][ 4 ];
    g_validSize[ sizeID ] = 1;

    for( int groupID = 0; groupID < 35; groupID++ )
    {
      if( groupID > 1 && groupID % 2 == 1 )
      {
        continue;
      }
      if( sizeID > 12 && groupID > 1 && groupID % 4 != 2 )
      {
        continue;
      }

      const bool shortLen = ( groupID < PDP_SHORT_TH[ 0 ] ) || ( groupID >= PDP_SHORT_TH[ 1 ] && groupID <= PDP_SHORT_TH[ 2 ] );
      const int len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];
      int16_t***& f = g_pdpFilters[ groupID ][ sizeID ];
      int16_t***& fSym = g_pdpFilters[ g_modeGroupSym[ groupID ] ][ g_size[ ( dsHeight << 8 ) + dsWidth ] ];

      if( !f )
      {
        f = new int16_t * *[ dsHeight ];
        for( int h = 0; h < dsHeight; ++h )
        {
          f[ h ] = new int16_t * [ dsWidth ];
          for( int w = 0; w < dsWidth / 4; w += 1 )
          {
            auto len4 = ( ( ( len + 7 ) / 8 ) * 8 ) * 4;
            f[ h ][ w ] = new int16_t[ len4 ];
            ::memset( f[ h ][ w ], 0, sizeof( int16_t ) * len4 );
          }
        }
      }

      if( !fSym && groupID > 1 && groupID < 34 )
      {
        fSym = new int16_t * *[ dsWidth ];
        for( int h = 0; h < dsWidth; ++h )
        {
          fSym[ h ] = new int16_t * [ dsHeight ];
          for( int w = 0; w < dsHeight / 4; w += 1 )
          {
            auto len4 = ( ( ( len + 7 ) / 8 ) * 8 ) * 4;
            fSym[ h ][ w ] = new int16_t[ len4 ];
            ::memset( fSym[ h ][ w ], 0, sizeof( int16_t ) * len4 );
          }
        }
      }
    }
  }

  int val = -1;

  for( int sizeID = 0; sizeID < 11; sizeID++ )
  {
    if( sizeID == 6 || sizeID == 7 )
    {
      continue;
    }

    const int dsWidth = g_sizeData[ sizeID ][ 3 ], dsHeight = g_sizeData[ sizeID ][ 4 ];
    bool shortLen = false;
    int len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];

    for( int groupID = 0; groupID < 19; groupID++ )
    {
      if( groupID > 1 && groupID % 2 == 1 )
      {
        continue;
      }

      int16_t***& f = g_pdpFilters[ groupID ][ sizeID ];
      int16_t***& fSym = g_pdpFilters[ g_modeGroupSym[ groupID ] ][ g_size[ ( dsHeight << 8 ) + dsWidth ] ];

      const int dsMode = ( groupID > 2 ) ? ( ( groupID - 2 ) >> 1 ) + 2 : groupID;

      for( int y = 0; y < dsHeight; y++ )
      {
        for( int x = 0; x < dsWidth; x++ )
        {
          for( int i = 0; i < len; i++ )
          {
            if( dsWidth == 4 && dsHeight == 4 )
            {
              val = g_weights4x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 4 && dsHeight == 8 )
            {
              val = g_weights4x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 4 )
            {
              val = g_weights8x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 8 )
            {
              val = g_weights8x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 4 && dsHeight == 16 )
            {
              val = g_weights4x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 4 )
            {
              val = g_weights16x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 16 )
            {
              val = g_weights8x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 8 )
            {
              val = g_weights16x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 16 )
            {
              val = g_weights16x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else
            {
              CHECK( true, "not supported case" );
            }

            auto ii = i - ( i % 8 );
            auto ij = i % 8;
            f[ y ][ x >> 2 ][ ( ii * 4 + 8 * ( x % 4 ) ) + ij ] = val;

            if( fSym && ( groupID > 1 && groupID < 34 ) )
            {
              const int templateX = g_sizeData[ sizeID ][ 5 ];
              const int templateY = g_sizeData[ sizeID ][ 6 ];
              const int stride1 = dsWidth * ( shortLen ? 1 : 2 ) + templateX;
              const int stride2 = templateX;
              const int stride1Sym = dsHeight * ( shortLen ? 1 : 2 ) + templateY;
              const int stride2Sym = templateY;
              const int lenZone1 = stride1 * templateY;
              const int x1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) % stride2 ) : ( i % stride1 );
              const int y1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) / stride2 ) + templateY : ( i / stride1 );
              const int iSym = y1 + ( x1 > templateX ? ( templateX * stride1Sym + ( x1 - templateX ) * stride2Sym ) : x1 * stride1Sym );
              const int iSymInterleave = ( iSym / 8 ) * 4 * 8 + ( y % 4 ) * 8 + ( iSym % 8 );

              fSym[ x ][ y >> 2 ][ iSymInterleave ] = val;
            }
          }
        }
      }
    }

    shortLen = true;
    len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];

    for( int groupID2 = 20; groupID2 < 35; groupID2++ )
    {
      if( groupID2 > 1 && groupID2 % 2 == 1 )
      {
        continue;
      }

      int16_t***& f = g_pdpFilters[ groupID2 ][ sizeID ];
      int16_t***& fSym = g_pdpFilters[ g_modeGroupSym[ groupID2 ] ][ g_size[ ( dsHeight << 8 ) + dsWidth ] ];

      const int dsMode = ( ( groupID2 - 20 ) >> 1 );

      for( int y = 0; y < dsHeight; y++ )
      {
        for( int x = 0; x < dsWidth; x++ )
        {
          for( int i = 0; i < len; i++ )
          {
            if( dsWidth == 4 && dsHeight == 4 )
            {
              val = g_weightsShort4x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 4 && dsHeight == 8 )
            {
              val = g_weightsShort4x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 4 )
            {
              val = g_weightsShort8x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 8 )
            {
              val = g_weightsShort8x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 4 && dsHeight == 16 )
            {
              val = g_weightsShort4x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 4 )
            {
              val = g_weightsShort16x4[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 8 && dsHeight == 16 )
            {
              val = g_weightsShort8x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 8 )
            {
              val = g_weightsShort16x8[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else if( dsWidth == 16 && dsHeight == 16 )
            {
              val = g_weightsShort16x16[ dsMode ][ y * dsWidth + x ][ i ];
            }
            else
            {
              CHECK( true, "not supported case" );
            }

            auto ii = i - ( i % 8 );
            auto ij = i % 8;
            f[ y ][ x >> 2 ][ ( ii * 4 + 8 * ( x % 4 ) ) + ij ] = val;

            if( fSym && ( groupID2 > 1 && groupID2 < 34 ) )
            {
              const int templateX = g_sizeData[ sizeID ][ 5 ];
              const int templateY = g_sizeData[ sizeID ][ 6 ];
              const int stride1 = dsWidth * ( shortLen ? 1 : 2 ) + templateX;
              const int stride2 = templateX;
              const int stride1Sym = dsHeight * ( shortLen ? 1 : 2 ) + templateY;
              const int stride2Sym = templateY;
              const int lenZone1 = stride1 * templateY;
              const int x1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) % stride2 ) : ( i % stride1 );
              const int y1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) / stride2 ) + templateY : ( i / stride1 );
              const int iSym = y1 + ( x1 > templateX ? ( templateX * stride1Sym + ( x1 - templateX ) * stride2Sym ) : x1 * stride1Sym );
              const int iSymInterleave = ( iSym / 8 ) * 4 * 8 + ( y % 4 ) * 8 + ( iSym % 8 );

              fSym[ x ][ y >> 2 ][ iSymInterleave ] = val;
            }
          }
        }
      }
    }
  }
  /////////////////////////Big blocks filter value loading///////////////////////////////////////////////////////////////
  val = -1;
  for( int sizeID = 13; sizeID < 16; sizeID++ )
  {
    const int dsWidth = g_sizeData[ sizeID ][ 3 ], dsHeight = g_sizeData[ sizeID ][ 4 ];
    const int sampFacHor = dsWidth / 16;
    const int sampFacVer = dsHeight / 16;
    int dx, dy;
    const int dsWidth2 = dsWidth / sampFacHor; //modified width at downsampled domain..needed for picking up correct values using raster scan order.
    bool shortLen = false;
    int len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];

    for( int groupID = 0; groupID < 19; groupID++ )
    {
      if( groupID > 1 && groupID % 4 != 2 )
      {
        continue;
      }

      int16_t***& f = g_pdpFilters[ groupID ][ sizeID ];
      int16_t***& fSym = g_pdpFilters[ g_modeGroupSym[ groupID ] ][ g_size[ ( dsHeight << 8 ) + dsWidth ] ];
      const int dsMode = groupID > 2 ? ( ( groupID - 2 ) >> 2 ) + 2 : groupID;

      for( int y = 0; y < dsHeight; y++ )
      {
        for( int x = 0; x < dsWidth; x++ )
        {
          if( ( x % sampFacHor != ( sampFacHor - 1 ) ) || ( y % sampFacVer != ( sampFacVer - 1 ) ) )
          {
            continue;
          }
          dx = x / sampFacHor;
          dy = y / sampFacVer;

          for( int i = 0; i < len; i++ )
          {
            if( dsWidth == 16 && dsHeight == 32 )
            {
              val = g_weights16x32[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else if( dsWidth == 32 && dsHeight == 16 )
            {
              val = g_weights32x16[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else if( dsWidth == 32 && dsHeight == 32 )
            {
              val = g_weights32x32[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else
            {
              CHECK( true, "not supported case" );
            }

            f[ y ][ x >> 2 ][ ( ( i / 8 ) * 32 + 8 * ( x % 4 ) ) + ( i % 8 ) ] = val;

            if( fSym && ( groupID > 1 && groupID < 34 ) )
            {
              const int templateX = g_sizeData[ sizeID ][ 5 ];
              const int templateY = g_sizeData[ sizeID ][ 6 ];
              const int stride1 = dsWidth * ( shortLen ? 1 : 2 ) + templateX;
              const int stride2 = templateX;
              const int stride1Sym = dsHeight * ( shortLen ? 1 : 2 ) + templateY;
              const int stride2Sym = templateY;
              const int lenZone1 = stride1 * templateY;
              const int x1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) % stride2 ) : ( i % stride1 );
              const int y1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) / stride2 ) + templateY : ( i / stride1 );
              const int iSym = y1 + ( x1 > templateX ? ( templateX * stride1Sym + ( x1 - templateX ) * stride2Sym ) : x1 * stride1Sym );
              const int iSymInterleave = ( iSym / 8 ) * 4 * 8 + ( y % 4 ) * 8 + ( iSym % 8 );

              fSym[ x ][ y >> 2 ][ iSymInterleave ] = val;
            }
          }
        }
      }
    }

    shortLen = true;
    len = shortLen ? g_sizeData[ sizeID ][ 10 ] : g_sizeData[ sizeID ][ 7 ];

    for( int groupID2 = 20; groupID2 < 35; groupID2++ )
    {
      if( groupID2 > 1 && groupID2 % 4 != 2 )
      {
        continue;
      }

      int16_t***& f = g_pdpFilters[ groupID2 ][ sizeID ];
      int16_t***& fSym = g_pdpFilters[ g_modeGroupSym[ groupID2 ] ][ g_size[ ( dsHeight << 8 ) + dsWidth ] ];
      const int dsMode = ( ( groupID2 - 22 ) >> 2 );

      for( int y = 0; y < dsHeight; y++ )
      {
        for( int x = 0; x < dsWidth; x++ )
        {
          if( ( x % sampFacHor != ( sampFacHor - 1 ) ) || ( y % sampFacVer != ( sampFacVer - 1 ) ) )
          {
            continue;
          }

          dx = x / sampFacHor;
          dy = y / sampFacVer;

          for( int i = 0; i < len; i++ )
          {
            if( dsWidth == 16 && dsHeight == 32 )
            {
              val = g_weightsShort16x32[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else if( dsWidth == 32 && dsHeight == 16 )
            {
              val = g_weightsShort32x16[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else if( dsWidth == 32 && dsHeight == 32 )
            {
              val = g_weightsShort32x32[ dsMode ][ dy * dsWidth2 + dx ][ i ];
            }
            else
            {
              CHECK( true, "not supported case" );
            }

            f[ y ][ x >> 2 ][ ( ( i / 8 ) * 32 + 8 * ( x % 4 ) ) + ( i % 8 ) ] = val;

            if( fSym && ( groupID2 > 1 && groupID2 < 34 ) )
            {
              const int templateX = g_sizeData[ sizeID ][ 5 ];
              const int templateY = g_sizeData[ sizeID ][ 6 ];
              const int stride1 = dsWidth * ( shortLen ? 1 : 2 ) + templateX;
              const int stride2 = templateX;
              const int stride1Sym = dsHeight * ( shortLen ? 1 : 2 ) + templateY;
              const int stride2Sym = templateY;
              const int lenZone1 = stride1 * templateY;
              const int x1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) % stride2 ) : ( i % stride1 );
              const int y1 = ( i >= lenZone1 ) ? ( ( i - lenZone1 ) / stride2 ) + templateY : ( i / stride1 );
              const int iSym = y1 + ( x1 > templateX ? ( templateX * stride1Sym + ( x1 - templateX ) * stride2Sym ) : x1 * stride1Sym );
              const int iSymInterleave = ( iSym / 8 ) * 4 * 8 + ( y % 4 ) * 8 + ( iSym % 8 );

              fSym[ x ][ y >> 2 ][ iSymInterleave ] = val;
            }
          }
        }
      }
    }
  }
}
#if JVET_AI0208_PDP_MIP
void destroyMipFilters()
{
  for( int size = 0; size < PDP_NUM_SIZES; ++size )
  {
    //int currHeight = g_sizeData[size][4];
    for( int group = 0; group < PDP_NUM_GROUPS; ++group )
    {
      if( g_pdpFiltersMip[ group ][ size ] )
      {
        if( g_pdpFiltersMip[ group ][ size ][ 0 ][ 0 ] )
        {
          delete[] g_pdpFiltersMip[ group ][ size ][ 0 ][ 0 ];
          g_pdpFiltersMip[ group ][ size ][ 0 ][ 0 ] = nullptr;

          const int currHeight = g_sizeData[ size ][ 4 ];

          for( int i = 0; i < currHeight; i++ )
          {
            if( g_pdpFiltersMip[ group ][ size ][ i ] )
            {
              delete[] g_pdpFiltersMip[ group ][ size ][ i ];
              g_pdpFiltersMip[ group ][ size ][ i ] = nullptr;
            }
          }
        }

        if( g_pdpFiltersMip[ group ][ size ] )
        {
          delete[] g_pdpFiltersMip[ group ][ size ];
          g_pdpFiltersMip[ group ][ size ] = nullptr;
        }
      }
    }
  }
}
#endif
void destroyPdpFilters()
{
  for( int size = 0; size < PDP_NUM_SIZES; ++size )
  {
    const int currWidth = g_sizeData[ size ][ 3 ], currHeight = g_sizeData[ size ][ 4 ];

    for( int group = 0; group < PDP_NUM_GROUPS; ++group )
    {
      if( g_pdpFilters[ group ][ size ] )
      {
        for( int h = 0; h < currHeight; ++h )
        {

          for( int w = 0; w < currWidth / 4; ++w )
          {
            delete[] g_pdpFilters[ group ][ size ][ h ][ w ];
            g_pdpFilters[ group ][ size ][ h ][ w ] = nullptr;
          }

          delete[] g_pdpFilters[ group ][ size ][ h ];
          g_pdpFilters[ group ][ size ][ h ] = nullptr;
        }

        delete[] g_pdpFilters[ group ][ size ];
        g_pdpFilters[ group ][ size ] = nullptr;
      }
    }
  }
}
#endif

#if JVET_AL0215_NSPT_SET_FOR_INTRANN
#include "RomNSPT_al0215.h"
#elif JVET_AJ0175_NSPT_FOR_NONREG_MODES
#include "RomNSPT_aj0175.h"
#elif JVET_AC0130_NSPT
#include "RomNSPT_ac0130.h"
#endif

#if ENABLE_TRACING
CDTrace* g_trace_ctx = NULL;
#endif
bool g_mctsDecCheckEnabled = false;

//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_W0119_LFNST_EXTENSION || EXTENDED_LFNST
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
#if JVET_AJ0237_INTERNAL_12BIT
int16_t* g_resiBorderTemplateLFNST[ NUM_NSPT_BLOCK_TYPES ][ 6 ][ 6 ][ 210 ];
#else
int8_t* g_resiBorderTemplateLFNST[ NUM_NSPT_BLOCK_TYPES ][ 6 ][ 6 ][ 210 ];
#endif
#else
#if JVET_AJ0237_INTERNAL_12BIT
int16_t* g_resiBorderTemplateLFNST[ 6 ][ 6 ][ 210 ];
#else
int8_t* g_resiBorderTemplateLFNST[ 6 ][ 6 ][ 210 ];
#endif
#endif
#else
int8_t* g_resiBorderTemplateLFNST[ 6 ][ 6 ][ 16 ];
#endif
#if JVET_AJ0237_INTERNAL_12BIT
int16_t* g_resiBorderTemplate[ 6 ][ 6 ][ NUM_TRANS_TYPE * NUM_TRANS_TYPE ];
#else
int8_t* g_resiBorderTemplate[ 6 ][ 6 ][ NUM_TRANS_TYPE * NUM_TRANS_TYPE ];
#endif
#else
const int8_t* g_resiBorderTemplate[ 6 ][ 6 ][ NUM_TRANS_TYPE * NUM_TRANS_TYPE ];
#endif
// g_initRomSignPred: Format for each [W][H][Idx] in [0-5][0-5][0-8], 0 => empty, 1 => (W+H-1)*16 template coefficients
#if !JVET_W0103_INTRA_MTS
const int8_t g_initRomSignPred[] = {
 1,8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, -8, -8, 8, 5, 5, 5, 5, -10, 10, -4, -10, -4, 5, 10, 10, 10, 10, -13, -6, 6, 13, 6, -6, -13, -10, -4, 5, 10,
 -10, -10, 10, -6, -3, 3, 6, -13, 13, -6, 8, -8, -8, 8, 8, 8, 8, 10, -10, -10, 10, 5, -4, -10, 8, -8, -8, 8, -8, -8, 8, 5, -4, -4, 5, -10, 10, -4, -4, 10, -10, 5, 5, 5,
 5, -6, 13, -13, 6, 3, -3, -6, -4, 10, -10, 5, -4, -4, 5, -3, 6, -6, 3, -6, 6, -3, 0,0,0,1,5, 9, 12, 14, 12, 9, 5, 4, 8, 11, 12, 0, -12, -12, 3, 6, 8, 9, -12, -5, 14, 2,
 3, 4, 5, -12, 14, -9, -12, -12, 0, 12, 11, 8, 4, -11, -11, 0, 11, 0, -11, -11, -8, -8, 0, 8, -11, -4, 12, -4, -4, 0, 4, -11, 12, -8, 14, -5, -12, 9, 8, 6, 3, 12, -4,
 -11, 8, 0, -8, -8, 9, -3, -8, 6, -8, -3, 9, 5, -2, -4, 3, -8, 9, -6, -9, 14, -12, 5, 4, 3, 2, -8, 12, -11, 4, 0, -4, -4, -6, 9, -8, 3, -4, -2, 5, -3, 5, -4, 2, -4, 5,
 -3, 1,14, 12, 9, 5, 4, 3, 2, 12, 11, 8, 4, 0, -4, -4, 9, 8, 6, 3, -4, -2, 5, 5, 4, 3, 2, -4, 5, -3, -12, 0, 12, 12, 11, 8, 4, -11, 0, 11, 11, 0, -11, -11, -8, 0, 8, 8,
 -11, -4, 12, -4, 0, 4, 4, -11, 12, -8, 9, -12, -5, 14, 12, 9, 5, 8, -11, -4, 12, 0, -12, -12, 6, -8, -3, 9, -12, -5, 14, 3, -4, -2, 5, -12, 14, -9, -5, 12, -14, 9, 8, 6,
 3, -4, 11, -12, 8, 0, -8, -8, -3, 8, -9, 6, -8, -3, 9, -2, 4, -5, 3, -8, 9, -6, 0,1,2, 3, 4, 5, 9, 12, 14, 4, 8, 11, 12, 12, 0, -12, 5, 9, 12, 14, -5, -12, 9, 3, 6, 8,
 9, -14, 12, -5, -4, -4, 0, 4, 8, 11, 12, -11, -11, 0, 11, 11, 0, -11, -12, -12, 0, 12, -4, -11, 8, -8, -8, 0, 8, -12, 11, -4, 5, -2, -4, 3, 6, 8, 9, 12, -4, -11, 8, 8,
 0, -8, 14, -5, -12, 9, -3, -8, 6, 9, -3, -8, 6, -9, 8, -3, -3, 5, -4, 2, 3, 4, 5, -8, 12, -11, 4, 4, 0, -4, -9, 14, -12, 5, -2, -4, 3, -6, 9, -8, 3, -5, 4, -2, 1,5, 4,
 3, 2, 3, 4, 5, 12, 11, 8, 4, 4, 0, -4, 14, 12, 9, 5, -2, -4, 3, 9, 8, 6, 3, -5, 4, -2, -4, 0, 4, 4, 8, 11, 12, -11, 0, 11, 11, 11, 0, -11, -12, 0, 12, 12, -4, -11, 8,
 -8, 0, 8, 8, -12, 11, -4, 3, -4, -2, 5, 9, 12, 14, 8, -11, -4, 12, 12, 0, -12, 9, -12, -5, 14, -5, -12, 9, 6, -8, -3, 9, -14, 12, -5, -2, 4, -5, 3, 6, 8, 9, -4, 11, -12,
 8, 8, 0, -8, -5, 12, -14, 9, -3, -8, 6, -3, 8, -9, 6, -9, 8, -3, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, 8, 8, 8, 8,
 -8, -8, 8, 5, 5, 5, 5, 5, 5, 5, 5, -10, 10, -4, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, -14, -12, -8, -3, 3, 8, 12, 14, 6, -6, -14, -11, -9, -6, -2, 2, 6, 9, 11, -11,
 -11, 11, -6, -5, -4, -1, 1, 4, 5, 6, -14, 14, -6, 10, 5, -4, -10, -10, -4, 5, 10, 10, 10, 10, 13, 6, -6, -13, -13, -6, 6, 13, 6, -6, -13, 10, 5, -4, -10, -10, -4, 5, 10,
 -10, -10, 10, 6, 3, -3, -6, -6, -3, 3, 6, -13, 13, -6, -9, 2, 11, 6, -6, -11, -2, 9, 9, 9, 9, -12, 3, 14, 8, -8, -14, -3, 12, 5, -5, -12, -9, 2, 11, 6, -6, -11, -2, 9,
 -9, -9, 9, -5, 1, 6, 4, -4, -6, -1, 5, -12, 12, -5, 0,0,0,1,3, 5, 8, 10, 12, 13, 14, 14, 12, 9, 5, 2, 5, 7, 9, 10, 11, 12, 12, 0, -12, -12, 2, 3, 5, 6, 8, 8, 9, 9, -12,
 -5, 14, 1, 2, 3, 3, 4, 4, 5, 5, -12, 14, -9, -8, -13, -14, -12, -5, 3, 10, 14, 12, 9, 5, -7, -11, -12, -10, -5, 2, 9, 12, 0, -12, -12, -5, -8, -9, -8, -3, 2, 6, 9, -12,
 -5, 14, -3, -4, -5, -4, -2, 1, 3, 5, -12, 14, -9, 12, 14, 5, -8, -14, -10, 3, 13, 11, 8, 4, 10, 12, 5, -7, -12, -9, 2, 11, 0, -11, -11, 8, 9, 3, -5, -9, -6, 2, 8, -11,
 -4, 13, 4, 5, 2, -3, -5, -3, 1, 4, -11, 13, -8, -14, -8, 10, 13, -3, -14, -5, 12, 10, 8, 4, -12, -7, 9, 11, -2, -12, -5, 10, 0, -10, -10, -9, -5, 6, 8, -2, -9, -3, 8,
 -10, -4, 12, -5, -3, 3, 4, -1, -5, -2, 4, -10, 12, -8, 1,14, 14, 13, 12, 10, 8, 5, 3, 2, 2, 1, 12, 12, 11, 10, 9, 7, 5, 2, 0, -2, -2, 9, 9, 8, 8, 6, 5, 3, 2, -2, -1, 3,
 5, 5, 4, 4, 3, 3, 2, 1, -2, 3, -2, -14, -10, -3, 5, 12, 14, 13, 8, 7, 5, 3, -12, -9, -2, 5, 10, 12, 11, 7, 0, -7, -7, -9, -6, -2, 3, 8, 9, 8, 5, -7, -3, 8, -5, -3, -1,
 2, 4, 5, 4, 3, -7, 8, -5, 13, 3, -10, -14, -8, 5, 14, 12, 10, 8, 4, 11, 2, -9, -12, -7, 5, 12, 10, 0, -10, -10, 8, 2, -6, -9, -5, 3, 9, 8, -10, -4, 12, 4, 1, -3, -5, -3,
 2, 5, 4, -10, 12, -8, -12, 5, 14, 3, -13, -10, 8, 14, 12, 9, 5, -10, 5, 12, 2, -11, -9, 7, 12, 0, -12, -12, -8, 3, 9, 2, -8, -6, 5, 9, -12, -5, 14, -4, 2, 5, 1, -4, -3,
 3, 5, -12, 14, -9, 0,1,1, 2, 3, 3, 4, 4, 5, 5, 9, 12, 14, 2, 5, 7, 9, 10, 11, 12, 12, 12, 0, -12, 3, 5, 8, 10, 12, 13, 14, 14, -5, -12, 9, 2, 3, 5, 6, 8, 8, 9, 9, -14,
 12, -5, -3, -4, -5, -4, -2, 1, 3, 5, 9, 12, 14, -7, -11, -12, -10, -5, 2, 9, 12, 12, 0, -12, -8, -13, -14, -12, -5, 3, 10, 14, -5, -12, 9, -5, -8, -9, -8, -3, 2, 6, 9,
 -14, 12, -5, 4, 5, 2, -3, -5, -3, 1, 4, 8, 11, 13, 10, 12, 5, -7, -12, -9, 2, 11, 11, 0, -11, 12, 14, 5, -8, -14, -10, 3, 13, -4, -11, 8, 8, 9, 3, -5, -9, -6, 2, 8, -13,
 11, -4, -5, -3, 3, 4, -1, -5, -2, 4, 8, 10, 12, -12, -7, 9, 11, -2, -12, -5, 10, 10, 0, -10, -14, -8, 10, 13, -3, -14, -5, 12, -4, -10, 8, -9, -5, 6, 8, -2, -9, -3, 8,
 -12, 10, -4, 1,5, 5, 4, 4, 3, 3, 2, 1, 2, 2, 3, 12, 12, 11, 10, 9, 7, 5, 2, 2, 0, -2, 14, 14, 13, 12, 10, 8, 5, 3, -1, -2, 2, 9, 9, 8, 8, 6, 5, 3, 2, -3, 2, -1, -5, -3,
 -1, 2, 4, 5, 4, 3, 5, 7, 8, -12, -9, -2, 5, 10, 12, 11, 7, 7, 0, -7, -14, -10, -3, 5, 12, 14, 13, 8, -3, -7, 5, -9, -6, -2, 3, 8, 9, 8, 5, -8, 7, -3, 4, 1, -3, -5, -3,
 2, 5, 4, 8, 10, 12, 11, 2, -9, -12, -7, 5, 12, 10, 10, 0, -10, 13, 3, -10, -14, -8, 5, 14, 12, -4, -10, 8, 8, 2, -6, -9, -5, 3, 9, 8, -12, 10, -4, -4, 2, 5, 1, -4, -3,
 3, 5, 9, 12, 14, -10, 5, 12, 2, -11, -9, 7, 12, 12, 0, -12, -12, 5, 14, 3, -13, -10, 8, 14, -5, -12, 9, -8, 3, 9, 2, -8, -6, 5, 9, -14, 12, -5, 1,8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, -8, -8, 8,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -10, 10, -4, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, -15, -14, -13, -11, -9, -7, -4,
 -1, 1, 4, 7, 9, 11, 13, 14, 15, 6, -6, -15, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, -11, -11, 11, -6, -6, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6,
 6, 6, -15, 15, -6, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 14, 12, 8, 3, -3, -8, -12, -14, -14, -12, -8, -3, 3, 8, 12, 14, 6, -6, -14,
 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, -11, -11, 11, 6, 5, 4, 1, -1, -4, -5, -6, -6, -5, -4, -1, 1, 4, 5, 6, -14, 14, -6, -11, -7, -1, 5, 10, 11, 9,
 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, -14, -9, -1, 7, 13, 15, 11, 4, -4, -11, -15, -13, -7, 1, 9, 14, 6, -6, -14, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11,
 -10, -5, 1, 7, 11, -11, -11, 11, -6, -4, -1, 3, 6, 6, 5, 2, -2, -5, -6, -6, -3, 1, 4, 6, -14, 14, -6, 0,0,0,1,1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14,
 13, 9, 5, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 0, -13, -13, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, -13, -5, 14, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4,
 4, 5, 5, 5, 5, 5, -13, 14, -9, -4, -8, -11, -13, -14, -14, -13, -11, -8, -4, 0, 4, 8, 11, 13, 14, 13, 9, 5, -4, -7, -10, -12, -13, -13, -12, -10, -7, -4, 0, 4, 7, 10,
 12, 13, 0, -13, -13, -3, -5, -7, -9, -9, -9, -9, -7, -5, -3, 0, 3, 5, 7, 9, 9, -13, -5, 14, -1, -3, -4, -5, -5, -5, -5, -4, -3, -1, 0, 1, 3, 4, 5, 5, -13, 14, -9, 7, 12,
 14, 14, 10, 4, -3, -9, -13, -14, -13, -8, -1, 5, 11, 14, 13, 9, 5, 6, 11, 13, 12, 9, 4, -2, -8, -12, -13, -11, -7, -1, 5, 10, 13, 0, -13, -13, 4, 8, 9, 9, 7, 3, -2, -6,
 -9, -9, -8, -5, -1, 4, 7, 9, -13, -5, 14, 2, 4, 5, 5, 4, 1, -1, -3, -5, -5, -4, -3, 0, 2, 4, 5, -13, 14, -9, -9, -14, -13, -7, 3, 11, 14, 12, 4, -5, -13, -14, -10, -1,
 8, 14, 12, 9, 5, -8, -13, -12, -6, 2, 10, 13, 11, 4, -5, -11, -13, -9, -1, 7, 12, 0, -12, -12, -6, -9, -9, -4, 2, 7, 9, 8, 3, -4, -8, -9, -7, -1, 5, 9, -12, -5, 14, -3,
 -5, -5, -2, 1, 4, 5, 4, 1, -2, -4, -5, -4, 0, 3, 5, -12, 14, -9, 1,14, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 1, 0, 13, 13, 13, 12, 12, 11, 11, 10, 9,
 8, 7, 6, 5, 4, 2, 1, 0, -1, -1, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, -1, 0, 1, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, 1, -1, -14, -13, -11, -8,
 -4, 0, 4, 8, 11, 13, 14, 14, 13, 11, 8, 4, 4, 3, 1, -13, -12, -10, -7, -4, 0, 4, 7, 10, 12, 13, 13, 12, 10, 7, 4, 0, -4, -4, -9, -9, -7, -5, -3, 0, 3, 5, 7, 9, 9, 9, 9,
 7, 5, 3, -4, -1, 4, -5, -5, -4, -3, -1, 0, 1, 3, 4, 5, 5, 5, 5, 4, 3, 1, -4, 4, -3, 14, 11, 5, -1, -8, -13, -14, -13, -9, -3, 4, 10, 14, 14, 12, 7, 6, 4, 2, 13, 10, 5,
 -1, -7, -11, -13, -12, -8, -2, 4, 9, 12, 13, 11, 6, 0, -6, -6, 9, 7, 4, -1, -5, -8, -9, -9, -6, -2, 3, 7, 9, 9, 8, 4, -6, -2, 7, 5, 4, 2, 0, -3, -4, -5, -5, -3, -1, 1,
 4, 5, 5, 4, 2, -6, 7, -4, -14, -8, 1, 10, 14, 13, 5, -4, -12, -14, -11, -3, 7, 13, 14, 9, 8, 6, 3, -12, -7, 1, 9, 13, 11, 5, -4, -11, -13, -10, -2, 6, 12, 13, 8, 0, -8,
 -8, -9, -5, 1, 7, 9, 8, 4, -3, -8, -9, -7, -2, 4, 9, 9, 6, -8, -3, 9, -5, -3, 0, 4, 5, 4, 2, -1, -4, -5, -4, -1, 2, 5, 5, 3, -8, 9, -6, 0,1,0, 1, 1, 2, 2, 3, 3, 4, 4, 4,
 4, 5, 5, 5, 5, 5, 9, 13, 14, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 0, -13, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, -5, -13, 9, 1,
 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, -14, 13, -5, -1, -3, -4, -5, -5, -5, -5, -4, -3, -1, 0, 1, 3, 4, 5, 5, 9, 13, 14, -4, -7, -10, -12, -13, -13, -12, -10, -7,
 -4, 0, 4, 7, 10, 12, 13, 13, 0, -13, -4, -8, -11, -13, -14, -14, -13, -11, -8, -4, 0, 4, 8, 11, 13, 14, -5, -13, 9, -3, -5, -7, -9, -9, -9, -9, -7, -5, -3, 0, 3, 5, 7,
 9, 9, -14, 13, -5, 2, 4, 5, 5, 4, 1, -1, -3, -5, -5, -4, -3, 0, 2, 4, 5, 9, 13, 14, 6, 11, 13, 12, 9, 4, -2, -8, -12, -13, -11, -7, -1, 5, 10, 13, 13, 0, -13, 7, 12, 14,
 14, 10, 4, -3, -9, -13, -14, -13, -8, -1, 5, 11, 14, -5, -13, 9, 4, 8, 9, 9, 7, 3, -2, -6, -9, -9, -8, -5, -1, 4, 7, 9, -14, 13, -5, -3, -5, -5, -2, 1, 4, 5, 4, 1, -2,
 -4, -5, -4, 0, 3, 5, 9, 12, 14, -8, -13, -12, -6, 2, 10, 13, 11, 4, -5, -11, -13, -9, -1, 7, 12, 12, 0, -12, -9, -14, -13, -7, 3, 11, 14, 12, 4, -5, -13, -14, -10, -1,
 8, 14, -5, -12, 9, -6, -9, -9, -4, 2, 7, 9, 8, 3, -4, -8, -9, -7, -1, 5, 9, -14, 12, -5, 1,5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 1, 1, 1, 13, 13, 13, 12, 12,
 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 0, -1, 14, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 0, -1, 1, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, -1, 1,
 0, -5, -5, -4, -3, -1, 0, 1, 3, 4, 5, 5, 5, 5, 4, 3, 1, 3, 4, 4, -13, -12, -10, -7, -4, 0, 4, 7, 10, 12, 13, 13, 12, 10, 7, 4, 4, 0, -4, -14, -13, -11, -8, -4, 0, 4, 8,
 11, 13, 14, 14, 13, 11, 8, 4, -1, -4, 3, -9, -9, -7, -5, -3, 0, 3, 5, 7, 9, 9, 9, 9, 7, 5, 3, -4, 4, -1, 5, 4, 2, 0, -3, -4, -5, -5, -3, -1, 1, 4, 5, 5, 4, 2, 4, 6, 7,
 13, 10, 5, -1, -7, -11, -13, -12, -8, -2, 4, 9, 12, 13, 11, 6, 6, 0, -6, 14, 11, 5, -1, -8, -13, -14, -13, -9, -3, 4, 10, 14, 14, 12, 7, -2, -6, 4, 9, 7, 4, -1, -5, -8,
 -9, -9, -6, -2, 3, 7, 9, 9, 8, 4, -7, 6, -2, -5, -3, 0, 4, 5, 4, 2, -1, -4, -5, -4, -1, 2, 5, 5, 3, 6, 8, 9, -12, -7, 1, 9, 13, 11, 5, -4, -11, -13, -10, -2, 6, 12, 13,
 8, 8, 0, -8, -14, -8, 1, 10, 14, 13, 5, -4, -12, -14, -11, -3, 7, 13, 14, 9, -3, -8, 6, -9, -5, 1, 7, 9, 8, 4, -3, -8, -9, -7, -2, 4, 9, 9, 6, -9, 8, -3, 1,8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, -8, -8,
 8, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -10, 10, -4, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4,
 -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 6, -6, -15, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11,
 11, -11, -11, 11, -6, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, -15, 15, -6, 11, 11, 10, 9, 7, 5, 3, 1,
 -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14,
 -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, 6, -6, -15, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5,
 -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, -11, -11, 11, 6, 6, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -6, -6, -6, -6, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 6, 6, -15, 15,
 -6, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5, -8, -10, -11, -11, -11, -9, -7, -4, 0, 3, 6, 8, 10, 11, 11, 11, 11, -15, -13, -11, -7, -4, 1, 5,
 9, 12, 14, 15, 14, 13, 10, 6, 2, -2, -6, -10, -13, -14, -15, -14, -12, -9, -5, -1, 4, 7, 11, 13, 15, 6, -6, -15, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5,
 2, -2, -5, -8, -10, -11, -11, -11, -9, -7, -4, 0, 3, 6, 8, 10, 11, -11, -11, 11, -6, -6, -5, -3, -2, 0, 2, 4, 5, 6, 6, 6, 5, 4, 3, 1, -1, -3, -4, -5, -6, -6, -6, -5, -4,
 -2, 0, 2, 3, 5, 6, 6, -15, 15, -6, 0,0,0,1,1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 13, 10, 5,
 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 0, -13, -13, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6,
 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, -13, -5, 15, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -13,
 15, -10, -2, -4, -6, -8, -10, -11, -13, -13, -14, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -3, -1, 1, 3, 5, 7, 9, 10, 12, 13, 14, 14, 15, 13, 10, 5, -2, -4, -5,
 -7, -9, -10, -11, -12, -12, -13, -13, -13, -12, -12, -11, -10, -8, -7, -5, -3, -1, 1, 2, 4, 6, 8, 9, 10, 11, 12, 13, 13, 0, -13, -13, -1, -3, -4, -5, -6, -7, -8, -9, -9,
 -10, -10, -9, -9, -9, -8, -7, -6, -5, -4, -2, -1, 0, 2, 3, 5, 6, 7, 8, 8, 9, 9, 10, -13, -5, 15, -1, -1, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3,
 -2, -1, -1, 0, 1, 2, 2, 3, 4, 4, 4, 5, 5, 5, -13, 15, -10, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3,
 7, 10, 12, 14, 15, 13, 10, 5, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3, 0, 3, 6, 9, 11, 12, 13, 0, -13, -13, 2,
 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, -13, -5, 15, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1, 0, -1, -2,
 -3, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, -13, 15, -10, -5, -9, -13, -14, -15, -13, -10, -6, -1, 3, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11, -13,
 -15, -14, -12, -9, -4, 1, 6, 10, 13, 14, 13, 9, 5, -4, -8, -11, -13, -13, -12, -9, -5, -1, 3, 7, 10, 12, 13, 12, 10, 7, 2, -2, -6, -10, -12, -13, -12, -11, -8, -4, 1, 5,
 9, 11, 13, 0, -13, -13, -3, -6, -8, -9, -10, -9, -7, -4, -1, 2, 5, 8, 9, 10, 9, 7, 5, 2, -1, -5, -7, -9, -10, -9, -8, -6, -3, 0, 4, 6, 8, 9, -13, -5, 14, -2, -3, -4, -5,
 -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -4, -3, -1, 0, 2, 3, 4, 5, -13, 14, -9, 1,15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12,
 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, 1, 0, 0, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 2,
 2, 1, 1, 0, -1, -1, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, 0, 1, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4,
 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, -1, 1, 0, -15, -14, -14, -13, -12, -10, -9, -7, -5, -3, -1, 1, 3, 6, 8, 9, 11, 12, 13, 14, 14, 15, 15, 14, 13,
 13, 11, 10, 8, 6, 4, 2, 2, 1, 1, -13, -13, -12, -11, -10, -9, -8, -6, -4, -2, -1, 1, 3, 5, 7, 8, 10, 11, 12, 12, 13, 13, 13, 12, 12, 11, 10, 9, 7, 5, 4, 2, 0, -2, -2,
 -10, -9, -9, -8, -8, -7, -6, -5, -3, -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 4, 3, 1, -2, -1, 2, -5, -5, -5, -4, -4, -4, -3, -2, -2, -1, 0, 1, 1,
 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, -2, 2, -1, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14,
 15, 15, 14, 12, 10, 7, 3, 3, 2, 1, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3, 0, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 0, -3, -3,
 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, -3, -1, 3, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5,
 -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1, -3, 3, -2, -14, -13, -10, -6, -1, 4, 9, 12, 14, 15, 13, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -3,
 1, 6, 10, 13, 15, 14, 13, 9, 5, 4, 3, 2, -13, -11, -9, -5, -1, 4, 8, 11, 12, 13, 12, 10, 6, 2, -2, -7, -10, -12, -13, -12, -10, -7, -3, 1, 5, 9, 12, 13, 13, 11, 8, 4, 0,
 -4, -4, -9, -8, -6, -4, 0, 3, 6, 8, 9, 10, 9, 7, 5, 1, -2, -5, -7, -9, -10, -9, -8, -5, -2, 1, 4, 7, 9, 10, 9, 8, 6, 3, -4, -2, 5, -5, -4, -3, -2, 0, 1, 3, 4, 5, 5, 5,
 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2, 4, 5, 5, 5, 4, 3, 2, -4, 5, -3, 1,4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 7, 9, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0, -9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -4, -9, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, -10, 9, -4, -5, -5, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 10, 13, 15, -13,
 -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 0, -13, -15, -15, -14, -14, -13, -13, -12,
 -11, -10, -9, -8, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, -5, -13, 10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -5, -4, -3, -2, -1,
 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, -15, 13, -5, 5, 5, 5, 4, 3, 2, 1, 1, -1, -1, -2, -3, -4, -5, -5, -5, -5, -5, -5, -4, -3, -2, -1, -1, 1, 1, 2, 3, 4,
 5, 5, 5, 10, 13, 15, 13, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -13, -13, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 13, 13, 0, -13, 15, 14,
 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14, -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, -5, -13, 10, 10, 9, 9, 8, 6, 5, 3, 1, -1, -3, -5,
 -6, -8, -9, -9, -10, -10, -9, -9, -8, -6, -5, -3, -1, 1, 3, 5, 6, 8, 9, 9, 10, -15, 13, -5, -5, -5, -4, -3, -1, 0, 2, 3, 4, 5, 5, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -5,
 -5, -4, -3, -2, 0, 1, 3, 4, 5, 5, 10, 13, 15, -13, -12, -10, -7, -3, 1, 4, 8, 11, 12, 13, 13, 11, 9, 5, 2, -2, -5, -9, -11, -13, -13, -12, -11, -8, -4, -1, 3, 7, 10, 12,
 13, 13, 0, -13, -15, -13, -11, -8, -4, 1, 5, 9, 12, 14, 15, 14, 13, 10, 6, 2, -2, -6, -10, -13, -14, -15, -14, -12, -9, -5, -1, 4, 8, 11, 13, 15, -5, -13, 10, -10, -9,
 -7, -5, -2, 0, 3, 6, 8, 9, 10, 9, 8, 7, 4, 1, -1, -4, -7, -8, -9, -10, -9, -8, -6, -3, 0, 2, 5, 7, 9, 10, -15, 13, -5, 1,0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4,
 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 13, 15, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13,
 13, 13, 0, -13, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, -5, -13, 10, 0, 1, 1, 2, 2, 3, 3, 4, 4,
 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, -15, 13, -5, -1, -1, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -1, -1,
 0, 1, 2, 2, 3, 4, 4, 4, 5, 5, 5, 10, 13, 15, -2, -4, -5, -7, -9, -10, -11, -12, -12, -13, -13, -13, -12, -12, -11, -10, -8, -7, -5, -3, -1, 1, 2, 4, 6, 8, 9, 10, 11, 12,
 13, 13, 13, 0, -13, -2, -4, -6, -8, -10, -11, -13, -13, -14, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -3, -1, 1, 3, 5, 7, 9, 10, 12, 13, 14, 14, 15, -5, -13, 10,
 -1, -3, -4, -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -9, -8, -7, -6, -5, -4, -2, -1, 0, 2, 3, 5, 6, 7, 8, 8, 9, 9, 10, -15, 13, -5, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1,
 0, -1, -2, -3, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 10, 13, 15, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12,
 -11, -9, -6, -3, 0, 3, 6, 9, 11, 12, 13, 13, 0, -13, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10,
 12, 14, 15, -5, -13, 10, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, -15, 13, -5, -2, -3, -4, -5,
 -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -4, -3, -1, 0, 2, 3, 4, 5, 9, 13, 14, -4, -8, -11, -13, -13, -12, -9, -5, -1, 3, 7, 10, 12, 13,
 12, 10, 7, 2, -2, -6, -10, -12, -13, -12, -11, -8, -4, 1, 5, 9, 11, 13, 13, 0, -13, -5, -9, -13, -14, -15, -13, -10, -6, -1, 3, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11,
 -13, -15, -14, -12, -9, -4, 1, 6, 10, 13, 14, -5, -13, 9, -3, -6, -8, -9, -10, -9, -7, -4, -1, 2, 5, 8, 9, 10, 9, 7, 5, 2, -1, -5, -7, -9, -10, -9, -8, -6, -3, 0, 4, 6,
 8, 9, -14, 13, -5, 1,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 1, 1, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11,
 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, -1, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6,
 6, 5, 4, 3, 3, 2, 1, 1, 0, -1, 0, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, 1, 0, -5, -5, -5, -4, -4, -4,
 -3, -2, -2, -1, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, 1, 2, 2, -13, -13, -12, -11, -10, -9, -8, -6, -4, -2, -1, 1, 3, 5, 7, 8, 10, 11, 12,
 12, 13, 13, 13, 12, 12, 11, 10, 9, 7, 5, 4, 2, 2, 0, -2, -15, -14, -14, -13, -12, -10, -9, -7, -5, -3, -1, 1, 3, 6, 8, 9, 11, 12, 13, 14, 14, 15, 15, 14, 13, 13, 11, 10,
 8, 6, 4, 2, -1, -2, 1, -10, -9, -9, -8, -8, -7, -6, -5, -3, -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 4, 3, 1, -2, 2, -1, 5, 5, 4, 3, 2, 1, 0, -1,
 -2, -3, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1, 2, 3, 3, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3,
 0, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 3, 0, -3, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14,
 12, 10, 7, 3, -1, -3, 2, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, -3, 3, -1, -5, -4, -3, -2, 0,
 1, 3, 4, 5, 5, 5, 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2, 4, 5, 5, 5, 4, 3, 2, 3, 4, 5, -13, -11, -9, -5, -1, 4, 8, 11, 12, 13, 12, 10, 6, 2, -2, -7, -10,
 -12, -13, -12, -10, -7, -3, 1, 5, 9, 12, 13, 13, 11, 8, 4, 4, 0, -4, -14, -13, -10, -6, -1, 4, 9, 12, 14, 15, 13, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -3, 1,
 6, 10, 13, 15, 14, 13, 9, 5, -2, -4, 3, -9, -8, -6, -4, 0, 3, 6, 8, 9, 10, 9, 7, 5, 1, -2, -5, -7, -9, -10, -9, -8, -5, -2, 1, 4, 7, 9, 10, 9, 8, 6, 3, -5, 4, -2, 1,8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, -8, -8, 8, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, -10, 10, -4, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0,
 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13,
 -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 13,
 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 6, -6, -15, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3,
 -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, -11, -11, 11, -6, -6, -6, -6, -6, -6,
 -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, -15, 15, -6, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11,
 -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 15, 15, 14, 14, 13, 13, 12, 11,
 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -7, -6, -5, -4, -2,
 -1, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 6, -6, -15, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9,
 -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, -11, -11, 11, 6,
 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1,
 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, -15, 15, -6, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11, 11, 11, 11, 10, 9, 8, 7,
 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11, 11, 11, 11, -15, -14, -14,
 -13, -12, -10, -8, -7, -5, -2, 0, 2, 4, 6, 8, 10, 11, 12, 13, 14, 15, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, -15, -14,
 -13, -12, -11, -10, -8, -6, -4, -2, 0, 2, 5, 7, 8, 10, 12, 13, 14, 14, 15, 6, -6, -15, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11,
 11, 11, 11, 10, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11,
 -11, -11, 11, -6, -6, -6, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 3, 4, 5, 5, 6, 6, 6, 6, 6, 6, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -5, -6, -6, -6, -6, -6,
 -6, -6, -5, -5, -4, -3, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 6, 6, 6, -15, 15, -6, 0,0,0,0,1,15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13,
 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5,
 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6,
 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -15, -15, -15, -14, -14, -14, -13, -13,
 -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12,
 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 1, 0, -13, -13, -13, -13, -12, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7,
 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -1, -10, -10, -10, -9, -9, -9, -9, -8,
 -8, -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5,
 4, 3, 3, 2, 1, 1, -1, 0, 1, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, -1, 1, -1, 15, 15, 14, 13, 13, 12, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13,
 -14, -14, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -7, -6, -4, -2, 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2,
 2, 1, 1, 13, 13, 12, 12, 11, 10, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -12, -13, -13, -13, -13, -13, -12, -12, -11, -10, -9, -8, -6, -5, -3, -2, 0,
 1, 3, 4, 6, 7, 9, 10, 11, 11, 12, 13, 13, 13, 13, 13, 12, 12, 11, 10, 9, 8, 6, 5, 3, 2, 0, -2, -2, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 0, -1, -2, -3, -4, -5, -6, -7,
 -8, -8, -9, -9, -10, -10, -10, -9, -9, -9, -8, -7, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 10, 9, 9, 9, 8, 7, 6, 6, 5, 3, 2, 1, -2, -1, 2, 5,
 5, 5, 5, 4, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5,
 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, -2, 2, -1, -15, -14, -13, -12, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -2,
 -5, -7, -9, -11, -12, -14, -14, -15, -15, -14, -13, -12, -10, -9, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15, 14, 14, 13, 11, 9, 7, 5, 2, 2, 2, 1, -13, -13, -12,
 -11, -9, -8, -6, -4, -2, 1, 3, 5, 7, 9, 10, 11, 12, 13, 13, 13, 12, 11, 10, 9, 7, 5, 2, 0, -2, -4, -6, -8, -10, -11, -12, -13, -13, -13, -13, -12, -11, -9, -8, -5, -3,
 -1, 1, 3, 5, 7, 9, 10, 12, 12, 13, 13, 13, 12, 11, 10, 8, 6, 4, 2, 0, -2, -2, -10, -9, -9, -8, -7, -6, -4, -3, -1, 0, 2, 4, 5, 6, 8, 8, 9, 10, 10, 10, 9, 8, 7, 6, 5, 3,
 2, 0, -1, -3, -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -4, -3, -1, 1, 2, 4, 5, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 3, 2, -2, -1, 2, -5, -5, -5, -4, -4,
 -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 4, 4, 5, 5,
 5, 5, 5, 5, 4, 4, 3, 2, 2, 1, -2, 2, -2, 0,0,1,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11,
 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15,
 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4,
 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5,
 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1,
 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, -13, -13, -13, -13, -12, -12, -12, -11,
 -11, -10, -9, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10,
 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 1, 0, -1, -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, 1, -10, -10, -10, -9, -9, -9, -9, -8,
 -8, -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5,
 4, 3, 3, 2, 1, 1, -1, 1, 0, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -2, -2, -1, -1, 0,
 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 2, 2, 13, 13, 12, 12, 11, 10, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11,
 -12, -13, -13, -13, -13, -13, -12, -12, -11, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 4, 6, 7, 9, 10, 11, 11, 12, 13, 13, 13, 13, 13, 12, 12, 11, 10, 9, 8, 6, 5, 3, 2, 2,
 0, -2, 15, 15, 14, 13, 13, 12, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -7, -6, -4, -2, 0,
 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, -1, -2, 1, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 0, -1, -2, -3, -4, -5, -6, -7,
 -8, -8, -9, -9, -10, -10, -10, -9, -9, -9, -8, -7, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 10, 9, 9, 9, 8, 7, 6, 6, 5, 3, 2, 1, -2, 2, -1,
 -5, -5, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -1, -1, 0,
 1, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 2, 1, 2, 2, 2, -13, -13, -12, -11, -9, -8, -6, -4, -2, 1, 3, 5, 7, 9, 10, 11, 12, 13, 13, 13, 12, 11, 10, 9, 7, 5, 2, 0, -2,
 -4, -6, -8, -10, -11, -12, -13, -13, -13, -13, -12, -11, -9, -8, -5, -3, -1, 1, 3, 5, 7, 9, 10, 12, 12, 13, 13, 13, 12, 11, 10, 8, 6, 4, 2, 2, 0, -2, -15, -14, -13, -12,
 -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -2, -5, -7, -9, -11, -12, -14, -14, -15, -15, -14, -13, -12, -10, -9, -6, -4,
 -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15, 14, 14, 13, 11, 9, 7, 5, 2, -1, -2, 2, -10, -9, -9, -8, -7, -6, -4, -3, -1, 0, 2, 4, 5, 6, 8, 8, 9, 10, 10, 10, 9, 8, 7, 6, 5, 3,
 2, 0, -1, -3, -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -4, -3, -1, 1, 2, 4, 5, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 3, 2, -2, 2, -1, 1,8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, -8, -8, 8, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -10, 10,
 -4, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7,
 -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5,
 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -10,
 -10, -10, -9, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5,
 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15,
 15, 15, 15, 6, -6, -15, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8,
 -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3,
 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, -11, -11, 11, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4,
 -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, -15, 15, -6, 11, 11, 11, 11, 11, 11,
 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9,
 -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3,
 -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 15, 15, 15, 14, 14, 14,
 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12,
 -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8,
 -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 6, -6, -15,
 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7,
 -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6,
 -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, -11, -11, 11,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5,
 -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1,
 -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, -15, 15, -6, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9,
 -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6,
 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8,
 -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, -15, -15, -14, -14, -14, -13, -13, -12, -12,
 -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11,
 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 0, 0, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13,
 -12, -12, -11, -10, -9, -8, -7, -6, -6, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 6, -6, -15, -11, -11, -11, -11,
 -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10,
 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10,
 -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, -11, -11, 11, -6, -6, -6, -6, -6,
 -6, -6, -5, -5, -5, -5, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4,
 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -3,
 -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, -15, 15, -6, 0,0,0,0,1,15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4,
 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -15, -15, -15, -15,
 -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1,
 -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, -13, -13, -13,
 -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -3, -2, -2, -1,
 -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, -10, -10, -10, -10, -10,
 -10, -9, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2,
 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8,
 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -4,
 -4, -4, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11,
 -12, -12, -13, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -13, -12, -11, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, -1,
 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5,
 4, 4, 3, 2, 1, 1, 1, 0, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 5, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9,
 -10, -10, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2,
 -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 5, 5,
 4, 3, 2, 2, 1, 0, -1, -1, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8,
 -8, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4,
 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 3, 3, 2, 2, 1, 1, -1, 0, 1, 5, 5, 5, 5, 5, 5, 5, 5,
 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
 -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, -1, 1, -1, -15, -15, -14, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0,
 1, 3, 4, 5, 6, 7, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11,
 -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -10, -9, -8, -6, -5, -4, -3, -2, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13,
 14, 14, 14, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 1, 0, -13, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -3, -2,
 -1, 0, 1, 2, 3, 4, 5, 7, 8, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -9,
 -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11,
 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -1, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2,
 -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -6, -7, -8, -8,
 -8, -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10,
 10, 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, -1, 0, 1, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
 -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, -1, 1, -1,
 0,0,1,5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8,
 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -2,
 -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -13, -13, -13, -13, -13,
 -13, -13, -13, -12, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0,
 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, -15, -15, -15, -15, -15, -15, -14,
 -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1,
 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14,
 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, -10, -10, -10, -10, -10, -10, -9,
 -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3,
 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8,
 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1,
 -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2,
 -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1,
 1, 0, 1, 1, 1, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 5, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10,
 -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0,
 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 5, 5, 4, 3, 2,
 2, 1, 1, 0, -1, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11,
 -12, -12, -13, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -13, -12, -11, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, -1,
 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5,
 4, 4, 3, 2, 1, 0, -1, 1, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8,
 -8, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4,
 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 3, 3, 2, 2, 1, 1, -1, 1, 0, -5, -5, -5, -5, -5, -5,
 -4, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1,
 -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4,
 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 1, 1, 1, -13, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0,
 1, 2, 3, 4, 5, 7, 8, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10,
 -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 12, 12,
 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, -1, -15, -15, -14, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0,
 1, 3, 4, 5, 6, 7, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11,
 -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -10, -9, -8, -6, -5, -4, -3, -2, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13,
 14, 14, 14, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, 1, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1,
 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -6, -7, -8, -8, -8,
 -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10,
 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, -1, 1, 0, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, 10, 10, 10, 10, 5, -4, -10, -10, -4,
 5, 10, 9, 9, 9, 9, -2, -11, -6, 6, 11, 2, -9, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, -14, -6, 6, 14, 12, 8, 3, -3, -8, -12, -14, -13, -6, 6, 13, 6, -6, -13, -13,
 -6, 6, 13, -12, -5, 5, 12, -3, -14, -8, 8, 14, 3, -12, 8, -8, -8, 8, 8, 8, 8, 8, 8, 8, 8, 11, -11, -11, 11, 9, 6, 2, -2, -6, -9, -11, 10, -10, -10, 10, 5, -4, -10, -10,
 -4, 5, 10, 9, -9, -9, 9, -2, -11, -6, 6, 11, 2, -9, -4, 10, -10, 5, 5, 5, 5, 5, 5, 5, 5, -6, 14, -14, 6, 5, 4, 1, -1, -4, -5, -6, -6, 13, -13, 6, 3, -3, -6, -6, -3, 3,
 6, -5, 12, -12, 5, -1, -6, -4, 4, 6, 1, -5, 0,0,0,1,5, 9, 12, 14, 14, 13, 12, 10, 8, 5, 3, 5, 9, 12, 14, 10, 3, -5, -12, -14, -13, -8, 4, 8, 11, 13, 3, -10, -14, -8, 5,
 14, 12, 4, 8, 10, 12, -5, -14, -3, 13, 10, -8, -14, -12, -12, 0, 12, 12, 11, 10, 9, 7, 5, 2, -12, -12, 0, 12, 9, 2, -5, -10, -12, -11, -7, -11, -11, 0, 11, 2, -9, -12,
 -7, 5, 12, 10, -10, -10, 0, 10, -5, -12, -2, 11, 9, -7, -12, 14, -5, -12, 9, 9, 8, 8, 6, 5, 3, 2, 14, -5, -12, 9, 6, 2, -3, -8, -9, -8, -5, 13, -4, -11, 8, 2, -6, -9,
 -5, 3, 9, 8, 12, -4, -10, 8, -3, -9, -2, 8, 6, -5, -9, -9, 14, -12, 5, 5, 4, 4, 3, 3, 2, 1, -9, 14, -12, 5, 3, 1, -2, -4, -5, -4, -3, -8, 13, -11, 4, 1, -3, -5, -3, 2,
 5, 4, -8, 12, -10, 4, -2, -5, -1, 4, 3, -3, -5, 1,14, 12, 9, 5, 5, 4, 4, 3, 3, 2, 1, 14, 12, 9, 5, 3, 1, -2, -4, -5, -4, -3, 13, 11, 8, 4, 1, -3, -5, -3, 2, 5, 4, 12,
 10, 8, 4, -2, -5, -1, 4, 3, -3, -5, -12, 0, 12, 12, 12, 11, 10, 9, 7, 5, 2, -12, 0, 12, 12, 9, 2, -5, -10, -12, -11, -7, -11, 0, 11, 11, 2, -9, -12, -7, 5, 12, 10, -10,
 0, 10, 10, -5, -12, -2, 11, 9, -7, -12, 9, -12, -5, 14, 14, 13, 12, 10, 8, 5, 3, 9, -12, -5, 14, 10, 3, -5, -12, -14, -13, -8, 8, -11, -4, 13, 3, -10, -14, -8, 5, 14,
 12, 8, -10, -4, 12, -5, -14, -3, 13, 10, -8, -14, -5, 12, -14, 9, 9, 8, 8, 6, 5, 3, 2, -5, 12, -14, 9, 6, 2, -3, -8, -9, -8, -5, -4, 11, -13, 8, 2, -6, -9, -5, 3, 9, 8,
 -4, 10, -12, 8, -3, -9, -2, 8, 6, -5, -9, 0,1,1, 2, 2, 3, 5, 8, 10, 12, 13, 14, 14, 3, 5, 7, 8, 13, 14, 12, 5, -3, -10, -14, 4, 8, 10, 12, 14, 5, -8, -14, -10, 3, 13, 5,
 9, 12, 14, 8, -10, -13, 3, 14, 5, -12, -2, -2, 0, 2, 5, 7, 9, 10, 11, 12, 12, -7, -7, 0, 7, 11, 12, 10, 5, -2, -9, -12, -10, -10, 0, 10, 12, 5, -7, -12, -9, 2, 11, -12,
 -12, 0, 12, 7, -9, -11, 2, 12, 5, -10, 3, -1, -2, 2, 3, 5, 6, 8, 8, 9, 9, 8, -3, -7, 5, 8, 9, 8, 3, -2, -6, -9, 12, -4, -10, 8, 9, 3, -5, -9, -6, 2, 8, 14, -5, -12, 9,
 5, -6, -8, 2, 9, 3, -8, -2, 3, -2, 1, 2, 3, 3, 4, 4, 5, 5, -5, 8, -7, 3, 4, 5, 4, 2, -1, -3, -5, -8, 12, -10, 4, 5, 2, -3, -5, -3, 1, 4, -9, 14, -12, 5, 3, -3, -4, 1, 5,
 2, -4, 1,3, 2, 2, 1, 2, 3, 3, 4, 4, 5, 5, 8, 7, 5, 3, 4, 5, 4, 2, -1, -3, -5, 12, 10, 8, 4, 5, 2, -3, -5, -3, 1, 4, 14, 12, 9, 5, 3, -3, -4, 1, 5, 2, -4, -2, 0, 2, 2, 5,
 7, 9, 10, 11, 12, 12, -7, 0, 7, 7, 11, 12, 10, 5, -2, -9, -12, -10, 0, 10, 10, 12, 5, -7, -12, -9, 2, 11, -12, 0, 12, 12, 7, -9, -11, 2, 12, 5, -10, 2, -2, -1, 3, 5, 8,
 10, 12, 13, 14, 14, 5, -7, -3, 8, 13, 14, 12, 5, -3, -10, -14, 8, -10, -4, 12, 14, 5, -8, -14, -10, 3, 13, 9, -12, -5, 14, 8, -10, -13, 3, 14, 5, -12, -1, 2, -3, 2, 3,
 5, 6, 8, 8, 9, 9, -3, 7, -8, 5, 8, 9, 8, 3, -2, -6, -9, -4, 10, -12, 8, 9, 3, -5, -9, -6, 2, 8, -5, 12, -14, 9, 5, -6, -8, 2, 9, 3, -8, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, -10, -4, 5, 10, 9, 9, 9, 9, 9, 9, 9, 9, -2, -11, -6,
 6, 11, 2, -9, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, -15, -13, -9, -3, 3, 9, 13, 15, 13, 9, 3, -3, -9, -13, -15, -14, -12, -8, -3, 3, 8, 12, 14, 6,
 -6, -14, -14, -6, 6, 14, -13, -11, -7, -3, 3, 7, 11, 13, -3, -15, -9, 9, 15, 3, -13, 10, 5, -4, -10, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 14, 6, -6, -14, -14, -6,
 6, 14, 12, 8, 3, -3, -8, -12, -14, 13, 6, -6, -13, -13, -6, 6, 13, 6, -6, -13, -13, -6, 6, 13, 12, 5, -5, -12, -12, -5, 5, 12, -3, -14, -8, 8, 14, 3, -12, -9, 2, 11, 6,
 -6, -11, -2, 9, 9, 9, 9, 9, 9, 9, 9, -13, 3, 15, 9, -9, -15, -3, 13, 11, 7, 3, -3, -7, -11, -13, -12, 3, 14, 8, -8, -14, -3, 12, 5, -5, -12, -12, -5, 5, 12, -11, 3, 13,
 7, -7, -13, -3, 11, -3, -13, -7, 7, 13, 3, -11, 0,0,0,1,3, 5, 8, 10, 12, 13, 14, 14, 14, 13, 12, 10, 8, 5, 3, 3, 5, 8, 10, 12, 13, 14, 14, 10, 3, -5, -12, -14, -13, -8,
 3, 5, 7, 9, 11, 12, 13, 13, 3, -10, -14, -8, 5, 14, 12, 2, 4, 6, 8, 10, 11, 12, 12, -5, -14, -3, 13, 10, -8, -14, -8, -13, -14, -12, -5, 3, 10, 14, 14, 13, 12, 10, 8, 5,
 3, -8, -13, -14, -12, -5, 3, 10, 14, 10, 3, -5, -12, -14, -13, -8, -7, -12, -13, -11, -5, 3, 9, 13, 3, -10, -14, -8, 5, 14, 12, -6, -11, -12, -10, -4, 2, 8, 12, -5, -14,
 -3, 13, 10, -8, -14, 12, 14, 5, -8, -14, -10, 3, 13, 13, 12, 11, 9, 7, 5, 3, 12, 14, 5, -8, -14, -10, 3, 13, 9, 3, -5, -11, -13, -12, -7, 11, 13, 5, -7, -13, -9, 3, 12,
 3, -9, -13, -7, 5, 13, 11, 10, 12, 4, -6, -12, -8, 2, 11, -5, -13, -3, 12, 9, -7, -13, -14, -8, 10, 13, -3, -14, -5, 12, 12, 11, 10, 8, 6, 4, 2, -14, -8, 10, 13, -3,
 -14, -5, 12, 8, 2, -4, -10, -12, -11, -6, -13, -7, 9, 12, -3, -13, -5, 11, 2, -8, -12, -6, 4, 12, 10, -12, -6, 8, 11, -2, -12, -4, 10, -4, -12, -2, 11, 8, -6, -12, 1,14,
 14, 13, 12, 10, 8, 5, 3, 3, 3, 2, 2, 2, 1, 1, 14, 14, 13, 12, 10, 8, 5, 3, 2, 1, -1, -2, -3, -3, -2, 13, 13, 12, 11, 9, 7, 5, 3, 1, -2, -3, -2, 1, 3, 2, 12, 12, 11, 10,
 8, 6, 4, 2, -1, -3, -1, 3, 2, -2, -3, -14, -10, -3, 5, 12, 14, 13, 8, 8, 7, 6, 5, 4, 3, 2, -14, -10, -3, 5, 12, 14, 13, 8, 5, 2, -3, -6, -8, -7, -4, -13, -9, -3, 5, 11,
 13, 12, 7, 2, -5, -8, -4, 3, 8, 6, -12, -8, -2, 4, 10, 12, 11, 6, -3, -8, -2, 7, 5, -4, -8, 13, 3, -10, -14, -8, 5, 14, 12, 12, 11, 10, 8, 6, 4, 2, 13, 3, -10, -14, -8,
 5, 14, 12, 8, 2, -4, -10, -12, -11, -6, 12, 3, -9, -13, -7, 5, 13, 11, 2, -8, -12, -6, 4, 12, 10, 11, 2, -8, -12, -6, 4, 12, 10, -4, -12, -2, 11, 8, -6, -12, -12, 5, 14,
 3, -13, -10, 8, 14, 14, 13, 12, 10, 8, 5, 3, -12, 5, 14, 3, -13, -10, 8, 14, 10, 3, -5, -12, -14, -13, -8, -11, 5, 13, 3, -12, -9, 7, 13, 3, -10, -14, -8, 5, 14, 12,
 -10, 4, 12, 2, -11, -8, 6, 12, -5, -14, -3, 13, 10, -8, -14, 0,1,1, 1, 2, 2, 2, 3, 3, 3, 5, 8, 10, 12, 13, 14, 14, 2, 3, 4, 5, 6, 7, 8, 8, 13, 14, 12, 5, -3, -10, -14,
 2, 4, 6, 8, 10, 11, 12, 12, 14, 5, -8, -14, -10, 3, 13, 3, 5, 8, 10, 12, 13, 14, 14, 8, -10, -13, 3, 14, 5, -12, -2, -3, -3, -2, -1, 1, 2, 3, 5, 8, 10, 12, 13, 14, 14,
 -4, -7, -8, -6, -3, 2, 5, 8, 13, 14, 12, 5, -3, -10, -14, -6, -11, -12, -10, -4, 2, 8, 12, 14, 5, -8, -14, -10, 3, 13, -8, -13, -14, -12, -5, 3, 10, 14, 8, -10, -13, 3,
 14, 5, -12, 2, 3, 1, -2, -3, -2, 1, 3, 5, 7, 9, 11, 12, 13, 13, 6, 8, 3, -4, -8, -5, 2, 7, 12, 13, 11, 5, -3, -9, -13, 10, 12, 4, -6, -12, -8, 2, 11, 13, 5, -7, -13, -9,
 3, 12, 12, 14, 5, -8, -14, -10, 3, 13, 7, -9, -12, 3, 13, 5, -11, -3, -2, 2, 3, -1, -3, -1, 2, 4, 6, 8, 10, 11, 12, 12, -8, -4, 5, 7, -2, -8, -3, 6, 11, 12, 10, 4, -2,
 -8, -12, -12, -6, 8, 11, -2, -12, -4, 10, 12, 4, -6, -12, -8, 2, 11, -14, -8, 10, 13, -3, -14, -5, 12, 6, -8, -11, 2, 12, 4, -10, 1,3, 3, 3, 2, 2, 2, 1, 1, 1, 2, 2, 2,
 3, 3, 3, 8, 8, 7, 6, 5, 4, 3, 2, 3, 3, 2, 1, -1, -2, -3, 12, 12, 11, 10, 8, 6, 4, 2, 3, 1, -2, -3, -2, 1, 3, 14, 14, 13, 12, 10, 8, 5, 3, 2, -2, -3, 1, 3, 1, -2, -3, -2,
 -1, 1, 2, 3, 3, 2, 3, 4, 5, 6, 7, 8, 8, -8, -5, -2, 3, 6, 8, 7, 4, 7, 8, 6, 3, -2, -5, -8, -12, -8, -2, 4, 10, 12, 11, 6, 8, 3, -4, -8, -5, 2, 7, -14, -10, -3, 5, 12,
 14, 13, 8, 4, -5, -7, 2, 8, 3, -6, 3, 1, -2, -3, -2, 1, 3, 2, 4, 6, 8, 10, 11, 12, 12, 7, 2, -5, -8, -4, 3, 8, 6, 11, 12, 10, 4, -2, -8, -12, 11, 2, -8, -12, -6, 4, 12,
 10, 12, 4, -6, -12, -8, 2, 11, 13, 3, -10, -14, -8, 5, 14, 12, 6, -8, -11, 2, 12, 4, -10, -2, 1, 3, 1, -3, -2, 2, 3, 5, 8, 10, 12, 13, 14, 14, -6, 3, 8, 2, -7, -5, 4, 8,
 13, 14, 12, 5, -3, -10, -14, -10, 4, 12, 2, -11, -8, 6, 12, 14, 5, -8, -14, -10, 3, 13, -12, 5, 14, 3, -13, -10, 8, 14, 8, -10, -13, 3, 14, 5, -12, 1,8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, -10, -4, 5, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, -2, -11, -6, 6, 11, 2, -9, -11, -11, -10, -9, -7, -5, -3,
 -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, 13, 9, 3, -3, -9, -13, -16, -15, -14, -13,
 -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, 6, -6, -15, -15, -6, 6, 15, -13, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 13, -3, -16, -9, 9, 16, 3, -13,
 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 15, 13, 9, 3, -3, -9, -13, -15, -15, -13, -9, -3, 3, 9, 13, 15, 13, 9, 3, -3, -9,
 -13, -15, 14, 12, 8, 3, -3, -8, -12, -14, -14, -12, -8, -3, 3, 8, 12, 14, 6, -6, -14, -14, -6, 6, 14, 13, 11, 7, 3, -3, -7, -11, -13, -13, -11, -7, -3, 3, 7, 11, 13, -3,
 -15, -9, 9, 15, 3, -13, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, 11, 11, 11, 11, -15, -10, -2, 7, 14, 16, 12, 4, -4, -12, -16, -14, -7,
 2, 10, 15, 13, 8, 3, -3, -8, -13, -15, -14, -9, -1, 7, 13, 15, 11, 4, -4, -11, -15, -13, -7, 1, 9, 14, 6, -6, -14, -14, -6, 6, 14, -13, -8, -1, 6, 12, 13, 10, 4, -4,
 -10, -13, -12, -6, 1, 8, 13, -3, -15, -8, 8, 15, 3, -13, 0,0,0,1,1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 13, 12, 10, 8, 6, 3, 1, 3, 4, 5, 7, 8, 9,
 10, 11, 12, 13, 13, 14, 14, 15, 15, 10, 3, -5, -12, -15, -13, -8, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 13, 13, 3, -10, -15, -8, 6, 15, 12, 1, 2, 3, 5, 6, 7,
 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, -5, -15, -3, 13, 10, -8, -15, -4, -8, -11, -14, -15, -15, -14, -11, -8, -4, 0, 4, 8, 11, 14, 15, 15, 13, 12, 10, 8, 6, 3, -4, -8,
 -11, -13, -15, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13, 15, 10, 3, -5, -12, -15, -13, -8, -4, -7, -10, -12, -13, -13, -12, -10, -7, -4, 0, 4, 7, 10, 12, 13, 3, -10, -15,
 -8, 6, 15, 12, -3, -7, -9, -11, -12, -12, -11, -9, -7, -3, 0, 3, 7, 9, 11, 12, -5, -15, -3, 13, 10, -8, -15, 7, 12, 15, 14, 10, 4, -3, -9, -14, -15, -13, -8, -1, 6, 11,
 15, 14, 13, 12, 10, 8, 5, 3, 7, 12, 15, 14, 10, 4, -3, -9, -13, -15, -13, -8, -1, 5, 11, 14, 10, 3, -5, -12, -15, -13, -8, 6, 11, 13, 13, 9, 4, -3, -8, -12, -13, -12,
 -7, -1, 5, 10, 13, 3, -10, -15, -8, 5, 14, 12, 6, 10, 12, 12, 9, 3, -2, -8, -11, -12, -11, -7, -1, 5, 9, 12, -5, -15, -3, 13, 10, -8, -14, -9, -15, -14, -7, 3, 11, 15,
 12, 4, -6, -13, -15, -10, -1, 8, 14, 14, 13, 12, 10, 8, 5, 3, -9, -14, -13, -7, 3, 11, 15, 12, 4, -5, -13, -15, -10, -1, 8, 14, 10, 3, -5, -12, -14, -13, -8, -8, -13,
 -12, -6, 3, 10, 13, 11, 4, -5, -12, -13, -9, -1, 7, 13, 3, -10, -14, -8, 5, 14, 12, -8, -12, -11, -6, 2, 9, 12, 10, 3, -5, -11, -12, -9, -1, 7, 12, -5, -14, -3, 13, 10,
 -8, -14, 1,15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 0, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 0, 0, -1, -1, -1,
 -1, 13, 13, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 1, 0, -1, -1, -1, 1, 1, 1, 12, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 3, 2, 1, 0, -1, 0, 1, 1, -1, -1, -15,
 -14, -11, -8, -4, 0, 4, 8, 11, 14, 15, 15, 14, 11, 8, 4, 4, 4, 3, 3, 2, 2, 1, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13, 15, 15, 13, 11, 8, 4, 3, 1, -2, -3, -4, -4, -2,
 -13, -12, -10, -7, -4, 0, 4, 7, 10, 12, 13, 13, 12, 10, 7, 4, 1, -3, -4, -2, 2, 4, 3, -12, -11, -9, -7, -3, 0, 3, 7, 9, 11, 12, 12, 11, 9, 7, 3, -2, -4, -1, 4, 3, -2,
 -4, 15, 11, 6, -1, -8, -13, -15, -14, -9, -3, 4, 10, 14, 15, 12, 7, 7, 6, 6, 5, 4, 3, 1, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4, 10, 14, 15, 12, 7, 5, 1, -2, -6,
 -7, -6, -4, 13, 10, 5, -1, -7, -12, -13, -12, -8, -3, 4, 9, 13, 13, 11, 6, 1, -5, -7, -4, 3, 7, 6, 12, 9, 5, -1, -7, -11, -12, -11, -8, -2, 3, 9, 12, 12, 10, 6, -2, -7,
 -1, 6, 5, -4, -7, -14, -8, 1, 10, 15, 13, 6, -4, -12, -15, -11, -3, 7, 14, 15, 9, 9, 8, 8, 6, 5, 3, 2, -14, -8, 1, 10, 15, 13, 5, -4, -12, -15, -11, -3, 7, 13, 14, 9, 6,
 2, -3, -8, -9, -8, -5, -13, -7, 1, 9, 13, 12, 5, -4, -11, -13, -10, -3, 6, 12, 13, 8, 2, -6, -9, -5, 3, 9, 8, -12, -7, 1, 9, 12, 11, 5, -3, -10, -12, -9, -2, 6, 11, 12,
 8, -3, -9, -2, 8, 6, -5, -9, 0,1,0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 6, 8, 10, 12, 13, 15, 15, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 8, 13, 15, 12, 6,
 -3, -10, -15, 1, 2, 3, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, 15, 6, -8, -15, -10, 3, 13, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 8, -10, -13, 3,
 15, 6, -12, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 6, 8, 10, 12, 13, 15, 15, -2, -4, -6, -7, -8, -8, -7, -6, -4, -2, 0, 2, 4, 6, 7, 8, 13, 15, 12, 6,
 -3, -10, -15, -3, -7, -9, -11, -12, -12, -11, -9, -7, -3, 0, 3, 7, 9, 11, 12, 15, 6, -8, -15, -10, 3, 13, -4, -8, -11, -13, -15, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13,
 15, 8, -10, -13, 3, 15, 6, -12, 1, 2, 3, 3, 2, 1, -1, -2, -3, -3, -3, -2, 0, 1, 2, 3, 5, 8, 10, 12, 13, 14, 15, 4, 7, 8, 8, 6, 2, -2, -5, -7, -8, -7, -4, -1, 3, 6, 8,
 13, 15, 12, 5, -3, -10, -14, 6, 10, 12, 12, 9, 3, -2, -8, -11, -12, -11, -7, -1, 5, 9, 12, 14, 5, -8, -15, -10, 3, 13, 7, 12, 15, 14, 10, 4, -3, -9, -13, -15, -13, -8,
 -1, 5, 11, 14, 8, -10, -13, 3, 15, 5, -12, -2, -3, -3, -1, 1, 2, 3, 2, 1, -1, -3, -3, -2, 0, 2, 3, 5, 8, 10, 12, 13, 14, 14, -5, -8, -7, -4, 2, 6, 8, 7, 2, -3, -7, -8,
 -6, -1, 4, 8, 13, 14, 12, 5, -3, -10, -14, -8, -12, -11, -6, 2, 9, 12, 10, 3, -5, -11, -12, -9, -1, 7, 12, 14, 5, -8, -14, -10, 3, 13, -9, -14, -13, -7, 3, 11, 15, 12,
 4, -5, -13, -15, -10, -1, 8, 14, 8, -10, -13, 3, 14, 5, -12, 1,3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 4, 4,
 3, 2, 2, 1, 1, 1, 1, 1, 0, -1, -1, 12, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 3, 2, 1, 1, 1, -1, -1, -1, 0, 1, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3,
 1, 1, -1, -1, 0, 1, 1, -1, -3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 3, 3, 2, 2, 1, 2, 2, 3, 3, 4, 4, 4, -8, -7, -6, -4, -2, 0, 2, 4, 6, 7, 8, 8, 7, 6, 4, 2, 4, 4, 3, 2, -1,
 -3, -4, -12, -11, -9, -7, -3, 0, 3, 7, 9, 11, 12, 12, 11, 9, 7, 3, 4, 2, -2, -4, -3, 1, 4, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13, 15, 15, 13, 11, 8, 4, 2, -3, -4, 1, 4,
 2, -3, 3, 2, 1, 0, -2, -3, -3, -3, -2, -1, 1, 2, 3, 3, 2, 1, 3, 4, 5, 6, 6, 7, 7, 8, 6, 3, -1, -4, -7, -8, -7, -5, -2, 2, 6, 8, 8, 7, 4, 6, 7, 6, 3, -1, -5, -7, 12, 9,
 5, -1, -7, -11, -12, -11, -8, -2, 3, 9, 12, 12, 10, 6, 7, 3, -4, -7, -5, 1, 6, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4, 10, 14, 15, 12, 7, 4, -5, -6, 1, 7, 3, -6,
 -3, -2, 0, 2, 3, 3, 1, -1, -2, -3, -2, -1, 1, 3, 3, 2, 3, 5, 6, 8, 8, 9, 9, -8, -4, 1, 6, 8, 7, 3, -2, -7, -8, -6, -2, 4, 7, 8, 5, 8, 9, 8, 3, -2, -6, -9, -12, -7, 1, 9,
 12, 11, 5, -3, -10, -12, -9, -2, 6, 11, 12, 8, 9, 3, -5, -9, -6, 2, 8, -14, -8, 1, 10, 15, 13, 5, -4, -12, -15, -11, -3, 7, 13, 14, 9, 5, -6, -8, 2, 9, 3, -8, 1,8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, -10, -4, 5, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, -2, -11, -6, 6, 11, 2, -9, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 13, 9, 3, -3, -9, -13,
 -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 6, -6, -15, -15, -6, 6, 15, -13,
 -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, -3, -16, -9, 9, 16, 3, -13, 11, 11, 10, 9, 7,
 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 15, 14, 12, 10, 7, 4, 2, -2,
 -4, -7, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, 13, 9, 3, -3, -9, -13, -16, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7,
 -9, -11, -13, -14, -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, 6, -6, -15, -15, -6, 6, 15, 13, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -12,
 -13, -13, -13, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 13, -3, -16, -9, 9, 16, 3, -13, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5,
 -8, -10, -11, -11, -11, -9, -7, -4, 0, 3, 6, 8, 10, 11, 11, 11, 11, 11, 11, 11, 11, -16, -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15,
 -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 13, 9, 3, -3, -9, -13, -16, -15, -13, -11, -7, -4, 1, 5, 9, 12, 14, 15, 14, 13, 10, 6, 2, -2, -6, -10, -13, -14, -15, -14,
 -12, -9, -5, -1, 4, 7, 11, 13, 15, 6, -6, -15, -15, -6, 6, 15, -13, -12, -10, -7, -3, 1, 5, 8, 11, 12, 13, 13, 11, 9, 6, 2, -2, -6, -9, -11, -13, -13, -12, -11, -8, -5,
 -1, 3, 7, 10, 12, 13, -3, -16, -9, 9, 16, 3, -13, 0,0,0,1,1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15,
 15, 15, 14, 12, 11, 8, 6, 3, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 11, 3, -6, -12, -15, -14,
 -8, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 3, -11, -15, -8, 6, 15, 12, 1, 1, 2, 2, 3, 4, 4, 5,
 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, -6, -15, -3, 14, 11, -8, -15, -2, -4, -6, -8, -10, -11, -13, -14, -14, -15, -15,
 -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 15, 14, 12, 11, 8, 6, 3, -2, -4, -6, -8, -10, -11, -13, -14, -14, -15, -15, -15, -14,
 -13, -12, -11, -9, -8, -6, -3, -1, 1, 3, 5, 7, 9, 10, 12, 13, 14, 14, 15, 11, 3, -6, -12, -15, -14, -8, -2, -4, -6, -8, -9, -10, -12, -12, -13, -14, -14, -13, -13, -12,
 -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 13, 13, 14, 3, -11, -15, -8, 6, 15, 12, -2, -4, -5, -7, -8, -9, -11, -11, -12, -12, -12, -12, -12, -11, -10, -9,
 -8, -6, -5, -3, -1, 1, 2, 4, 6, 7, 9, 10, 11, 12, 12, 12, -6, -15, -3, 14, 11, -8, -15, 4, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 4, 0, -4, -7, -10, -12, -14, -15, -15,
 -14, -12, -10, -7, -4, 0, 4, 7, 10, 12, 14, 15, 15, 14, 12, 10, 8, 6, 3, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10,
 -7, -3, 0, 3, 7, 10, 12, 14, 15, 10, 3, -6, -12, -15, -14, -8, 3, 6, 9, 11, 13, 14, 14, 13, 11, 9, 6, 3, 0, -3, -6, -9, -11, -13, -14, -14, -13, -11, -9, -6, -3, 0, 3,
 6, 9, 11, 13, 14, 3, -10, -15, -8, 6, 15, 12, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12,
 -6, -15, -3, 14, 10, -8, -15, -5, -9, -13, -15, -15, -13, -11, -6, -2, 4, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11, -14, -15, -14, -12, -9, -4, 1, 6, 10, 13, 15, 15, 13,
 12, 10, 8, 6, 3, -5, -9, -13, -14, -15, -13, -10, -6, -1, 3, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11, -14, -15, -14, -12, -9, -4, 1, 6, 10, 13, 15, 10, 3, -5, -12, -15,
 -13, -8, -5, -9, -12, -13, -14, -12, -10, -6, -1, 3, 8, 11, 13, 14, 13, 10, 7, 3, -2, -6, -10, -12, -14, -13, -11, -8, -4, 1, 5, 9, 12, 13, 3, -10, -15, -8, 6, 15, 12,
 -4, -8, -11, -12, -12, -11, -9, -5, -1, 3, 7, 10, 12, 12, 12, 9, 6, 2, -2, -6, -9, -11, -12, -12, -10, -7, -4, 1, 5, 8, 11, 12, -5, -15, -3, 13, 10, -8, -15, 1,15, 15,
 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13,
 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, 0, 0, 0, -1, -1, -1, 0, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 10, 10, 10, 9,
 9, 8, 8, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, 0, -1, 0, 0, 1, 1, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2,
 1, 1, 0, -1, 0, 1, 0, 0, -1, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, 14, 14, 13, 11, 10, 8, 6, 4, 2, 2, 2, 2, 2, 1,
 1, 0, -15, -14, -14, -13, -12, -10, -9, -7, -5, -3, -1, 1, 3, 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, 14, 14, 13, 11, 10, 8, 6, 4, 2, 2, 0, -1, -2, -2, -2, -1, -14, -13,
 -13, -12, -11, -10, -8, -6, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 12, 13, 13, 14, 14, 13, 12, 12, 10, 9, 8, 6, 4, 2, 0, -2, -2, -1, 1, 2, 2, -12, -12, -12, -11, -10, -9,
 -7, -6, -4, -2, -1, 1, 3, 5, 6, 8, 9, 10, 11, 12, 12, 12, 12, 12, 11, 11, 9, 8, 7, 5, 4, 2, -1, -2, 0, 2, 2, -1, -2, 15, 14, 12, 10, 7, 4, 0, -4, -7, -10, -12, -14, -15,
 -15, -14, -12, -10, -7, -4, 0, 4, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 4, 3, 3, 3, 2, 2, 1, 1, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10,
 -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 2, 1, -1, -3, -4, -3, -2, 14, 13, 11, 9, 6, 3, 0, -3, -6, -9, -11, -13, -14, -14, -13, -11, -9, -6, -3, 0, 3, 6,
 9, 11, 13, 14, 14, 13, 11, 9, 6, 3, 1, -2, -4, -2, 1, 3, 3, 12, 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 12, 12,
 10, 8, 6, 3, -1, -4, -1, 3, 2, -2, -3, -15, -13, -10, -6, -1, 4, 9, 12, 14, 15, 14, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -4, 2, 6, 11, 13, 15, 15, 13, 9, 5, 5,
 5, 4, 4, 3, 2, 1, -15, -13, -10, -6, -1, 4, 9, 12, 14, 15, 14, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -3, 1, 6, 10, 13, 15, 14, 13, 9, 5, 4, 1, -2, -4, -5, -5,
 -3, -13, -12, -9, -5, -1, 4, 8, 11, 13, 14, 12, 10, 6, 2, -3, -7, -10, -13, -14, -13, -11, -8, -3, 1, 6, 10, 12, 14, 13, 12, 9, 5, 1, -4, -5, -3, 2, 5, 4, -12, -11, -8,
 -5, -1, 4, 7, 10, 12, 12, 11, 9, 6, 2, -2, -6, -9, -12, -12, -12, -10, -7, -3, 1, 5, 9, 11, 12, 12, 11, 8, 4, -2, -5, -1, 5, 4, -3, -5, 1,2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 6, 8, 9, 10, 11, 11, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
 6, 6, 6, 6, 6, 6, 10, 11, 9, 4, -2, -7, -11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 11, 4, -6, -11, -7, 2, 10,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 6, -7, -10, 2, 11, 4, -9, -3, -3, -3, -3,
 -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 6, 8, 11, 12, 14, 15, 15, -8, -8, -8, -8, -7, -7, -7, -6, -5, -5, -4, -3,
 -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 7, 8, 8, 8, 8, 14, 15, 12, 6, -3, -11, -15, -12, -12, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2,
 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 12, 12, 15, 6, -8, -15, -11, 3, 14, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8,
 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 8, -11, -14, 3, 15, 6, -12, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2,
 3, 3, 3, 6, 8, 11, 12, 14, 15, 15, 8, 8, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -8, -8, -8, -8, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7, 8, 8, 14, 15, 12, 6, -3,
 -11, -15, 12, 12, 11, 10, 8, 6, 3, 1, -1, -3, -6, -8, -10, -11, -12, -12, -12, -12, -11, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 11, 12, 12, 15, 6, -8, -15, -11, 3, 14, 15,
 14, 13, 12, 9, 7, 4, 1, -1, -4, -7, -9, -12, -13, -14, -15, -15, -14, -13, -12, -9, -7, -4, -1, 1, 4, 7, 9, 12, 13, 14, 15, 8, -11, -14, 3, 15, 6, -12, -3, -3, -2, -2,
 -1, 0, 1, 2, 2, 3, 3, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 6, 8, 11, 12, 14, 15, 15, -8, -7, -6, -4, -2, 0, 3, 5, 7, 8, 8, 8, 7, 5, 3,
 1, -1, -3, -5, -7, -8, -8, -8, -7, -5, -3, 0, 2, 4, 6, 7, 8, 14, 15, 12, 6, -3, -11, -15, -12, -11, -9, -6, -3, 1, 4, 7, 10, 12, 12, 12, 11, 8, 5, 2, -2, -5, -8, -11,
 -12, -12, -12, -10, -7, -4, -1, 3, 6, 9, 11, 12, 15, 6, -8, -15, -11, 3, 14, -15, -14, -11, -8, -4, 1, 5, 9, 12, 14, 15, 15, 13, 10, 6, 2, -2, -6, -10, -13, -15, -15,
 -14, -12, -9, -5, -1, 4, 8, 11, 14, 15, 8, -11, -14, 3, 15, 6, -12, 1,0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 6,
 8, 11, 12, 14, 15, 15, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 14, 15, 12, 6, -3, -11, -15, 1, 1, 2, 2, 3, 4, 4,
 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 15, 6, -8, -15, -11, 3, 14, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10,
 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 8, -11, -14, 3, 15, 6, -12, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -1,
 -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 6, 8, 11, 12, 14, 15, 15, -1, -2, -3, -4, -5, -6, -7, -7, -8, -8, -8, -8, -8, -7, -7, -6, -5, -4, -3, -2, -1, 0, 2, 3, 4, 5, 6,
 6, 7, 8, 8, 8, 14, 15, 12, 6, -3, -11, -15, -2, -4, -5, -7, -8, -9, -11, -11, -12, -12, -12, -12, -12, -11, -10, -9, -8, -6, -5, -3, -1, 1, 2, 4, 6, 7, 9, 10, 11, 12,
 12, 12, 15, 6, -8, -15, -11, 3, 14, -2, -4, -6, -8, -10, -11, -13, -14, -14, -15, -15, -15, -14, -13, -12, -11, -9, -8, -6, -3, -1, 1, 3, 5, 7, 9, 10, 12, 13, 14, 14,
 15, 8, -11, -14, 3, 15, 6, -12, 1, 1, 2, 2, 3, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 6, 8, 10, 12, 14, 15, 15, 2,
 4, 5, 7, 8, 8, 8, 8, 7, 5, 4, 2, 0, -2, -4, -5, -7, -8, -8, -8, -8, -7, -5, -4, -2, 0, 2, 4, 5, 7, 8, 8, 14, 15, 12, 6, -3, -10, -15, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8,
 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 15, 6, -8, -15, -10, 3, 14, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7,
 -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 8, -10, -14, 3, 15, 6, -12, -1, -2, -3, -3, -3, -3, -2, -1, 0, 1, 2, 2, 3, 3, 3, 2, 2, 1, 0, -1,
 -2, -3, -3, -3, -2, -2, -1, 0, 1, 2, 3, 3, 6, 8, 10, 12, 13, 15, 15, -3, -5, -7, -8, -8, -7, -6, -3, -1, 2, 4, 6, 8, 8, 8, 6, 4, 2, -1, -4, -6, -7, -8, -8, -7, -5, -2,
 0, 3, 5, 7, 8, 13, 15, 12, 6, -3, -10, -15, -4, -8, -11, -12, -12, -11, -9, -5, -1, 3, 7, 10, 12, 12, 12, 9, 6, 2, -2, -6, -9, -11, -12, -12, -10, -7, -4, 1, 5, 8, 11,
 12, 15, 6, -8, -15, -10, 3, 13, -5, -9, -13, -14, -15, -13, -10, -6, -1, 3, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11, -14, -15, -14, -12, -9, -4, 1, 6, 10, 13, 15, 8,
 -10, -13, 3, 15, 6, -12, 1,3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 8, 8, 8, 8, 8, 8, 8, 8,
 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 1, 1, 1, 0, 0, 0, -1, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7,
 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, -1, 0, 0, 1, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1,
 1, 0, 0, -1, 0, 1, 0, -1, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 1, 1, 2, 2, 2, 2, 2, -8, -8, -8, -7,
 -6, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 8, 8, 8, 8, 7, 7, 6, 5, 4, 3, 2, 1, 2, 2, 2, 1, 0, -2, -2, -12, -12, -12, -11, -10, -9, -7, -6, -4, -2, -1, 1, 3,
 5, 6, 8, 9, 10, 11, 12, 12, 12, 12, 12, 11, 11, 9, 8, 7, 5, 4, 2, 2, 1, -1, -2, -2, 0, 2, -15, -14, -14, -13, -12, -10, -9, -7, -5, -3, -1, 1, 3, 6, 8, 9, 11, 12, 13,
 14, 15, 15, 15, 14, 14, 13, 11, 10, 8, 6, 4, 2, 1, -2, -2, 0, 2, 1, -2, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 3, 3,
 2, 2, 1, 1, 1, 2, 2, 3, 3, 3, 4, 8, 8, 7, 5, 4, 2, 0, -2, -4, -5, -7, -8, -8, -8, -8, -7, -5, -4, -2, 0, 2, 4, 5, 7, 8, 8, 8, 8, 7, 5, 4, 2, 3, 4, 3, 1, -1, -2, -3, 12,
 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 3, 3, 1, -2, -4, -2, 1, 3, 15, 14, 12, 10, 7, 3, 0,
 -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 2, -2, -3, 1, 4, 1, -3, -3, -3, -2, -1, 0, 1, 2, 2, 3, 3, 3, 2, 1,
 0, -1, -2, -2, -3, -3, -3, -2, -2, -1, 0, 1, 2, 3, 3, 3, 3, 2, 1, 2, 3, 4, 4, 5, 5, 5, -8, -7, -5, -3, 0, 2, 5, 7, 8, 8, 7, 6, 4, 1, -2, -4, -6, -8, -8, -8, -6, -4, -2,
 1, 3, 6, 7, 8, 8, 7, 5, 3, 5, 5, 4, 2, -1, -4, -5, -12, -11, -8, -5, -1, 4, 7, 10, 12, 12, 11, 9, 6, 2, -2, -6, -9, -12, -12, -12, -10, -7, -3, 1, 5, 9, 11, 12, 12, 11,
 8, 4, 5, 2, -3, -5, -4, 1, 5, -15, -13, -10, -6, -1, 4, 9, 12, 14, 15, 14, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -3, 1, 6, 10, 13, 15, 14, 13, 9, 5, 3, -4, -5,
 1, 5, 2, -4, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9,
 -11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, -10, -4, 5, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, -2, -11, -6, 6, 11, 2,
 -9, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5,
 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13,
 -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14,
 15, 15, 15, 15, 16, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7,
 -6, -5, -5, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 6, -6, -15, -15, -6,
 6, 15, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4,
 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, -3, -16, -9, 9, 16, 3, -13, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3,
 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8,
 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 13, 9, 3, -3, -9,
 -13, -16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -14, -14, -13, -13,
 -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 6, -6, -15, -15, -6, 6, 15, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7,
 6, 5, 3, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 3,
 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, -3, -16, -9, 9, 16, 3, -13, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11, 11, 11, 11,
 10, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 6, 8, 10, 12, 13, 14, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 3, 1, -1, -3, -6, -8, -10, -11, -13,
 -14, -15, -16, -16, -16, -15, -14, -13, -12, -10, -8, -6, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 13, 9, 3, -3, -9, -13, -16, -15, -14, -14, -13, -12, -10, -8,
 -7, -5, -2, 0, 2, 4, 6, 8, 10, 11, 12, 13, 14, 15, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, -15, -14, -13, -12, -11, -10,
 -8, -6, -4, -2, 0, 2, 5, 7, 8, 10, 12, 13, 14, 14, 15, 6, -6, -15, -15, -6, 6, 15, -13, -13, -12, -12, -10, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7, 9, 10, 11, 12, 13, 13, 13,
 13, 13, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -10, -11, -12, -13, -13, -13, -13, -13, -12, -11, -10, -9, -7, -5, -4, -2, 0, 2, 4, 6, 8, 9, 10, 12, 12, 13, 13,
 -3, -16, -9, 9, 16, 3, -13, 0,0,0,0,1,15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11,
 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14,
 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9,
 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0,
 0, 0, 0, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 14, 14,
 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11,
 -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9,
 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 0, -1, -1, -1, -1, -14, -14, -14, -13, -13, -13, -12, -12, -11, -11, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7,
 8, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -1, -1, 0, 1, 1, -12, -12, -12,
 -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 4, 3, 2, 1, 0, -1, 0, 1, 1, -1, -1, 15, 15, 14, 14, 13, 12, 11, 9, 8, 6, 4, 3, 1, -1, -3, -5, -6, -8, -10, -11, -12,
 -13, -14, -15, -15, -15, -15, -15, -14, -14, -13, -12, -10, -9, -7, -6, -4, -2, 0, 2, 3, 5, 7, 8, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 14, 13, 13, 11, 10, 9, 7, 5, 4,
 2, 2, 2, 2, 1, 1, 1, 0, 15, 15, 14, 14, 13, 12, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -15, -15, -14, -13, -13, -11, -10, -9,
 -7, -6, -4, -2, 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, 1, 0, -1, -2, -2, -2, -1, 14, 14, 13, 12, 12, 11, 10, 8, 7,
 5, 4, 2, 1, -1, -3, -4, -6, -7, -9, -10, -11, -12, -13, -13, -14, -14, -14, -13, -13, -12, -12, -11, -9, -8, -7, -5, -4, -2, 0, 1, 3, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14,
 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 3, 2, 0, -1, -2, -1, 1, 2, 2, 12, 12, 12, 11, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -12, -12, -12,
 -12, -12, -12, -12, -11, -11, -10, -9, -7, -6, -5, -3, -2, 0, 1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, -1, -2, 0, 2, 1, -1,
 -2, -15, -15, -14, -13, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 5, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14,
 -12, -11, -9, -6, -4, -2, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11, 10, 7, 5, 3, 2, 2, 2, 2, 1, 1, 0, -15, -14, -14, -12, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10,
 12, 13, 14, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -14, -13, -12, -10, -9, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15,
 15, 14, 13, 11, 9, 7, 5, 2, 2, 0, -1, -2, -3, -2, -1, -14, -13, -12, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 14, 14, 13, 12, 11, 9, 7, 5, 3, 0, -2, -4,
 -6, -8, -10, -12, -13, -13, -14, -14, -13, -12, -11, -10, -8, -6, -4, -1, 1, 3, 5, 8, 9, 11, 12, 13, 14, 14, 13, 13, 12, 10, 9, 7, 5, 2, 0, -2, -3, -1, 1, 2, 2, -12,
 -12, -11, -10, -9, -7, -6, -4, -2, 1, 3, 5, 7, 8, 10, 11, 12, 12, 12, 12, 12, 11, 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -12, -12, -12, -12, -11, -10, -9, -7,
 -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 12, 12, 12, 12, 12, 11, 9, 8, 6, 4, 2, -1, -3, 0, 2, 2, -1, -2, 0,0,1,3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8,
 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14,
 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0,
 0, 0, 0, 0, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 1, 1,
 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, -1, -1, -12, -12, -12, -12, -12,
 -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 4, 3, 2, 1, 1, 0, -1, -1, -1, 0, 1, -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, -1, -1, 0, 1,
 0, -1, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2,
 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 8, 8, 8, 7, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -7, -8, -8, -8, -8,
 -8, -8, -7, -7, -6, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 2, 2, 2, 1, 0, -1, -2, 12, 12, 12, 11, 11, 10,
 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -12, -12, -12, -12, -12, -12, -12, -11, -11, -10, -9, -7, -6, -5, -3, -2, 0, 1, 3, 4, 6, 7, 8, 9, 10, 11, 12,
 12, 12, 12, 12, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 2, 1, -1, -2, -1, 0, 2, 15, 15, 14, 14, 13, 12, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13, -14,
 -14, -15, -15, -15, -15, -14, -13, -13, -11, -10, -9, -7, -6, -4, -2, 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, 1, -1,
 -2, 0, 2, 1, -2, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2,
 -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2, 1, 1, 0, 1, 1, 2, 2, 2, 2, 3, -8, -8, -7, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 7, 6, 5, 4,
 3, 2, 0, -1, -3, -4, -5, -6, -7, -7, -8, -8, -8, -8, -7, -7, -6, -5, -3, -2, -1, 1, 2, 3, 4, 6, 6, 7, 8, 8, 8, 8, 8, 7, 6, 5, 4, 3, 1, 2, 3, 2, 1, 0, -2, -2, -12, -12,
 -11, -10, -9, -7, -6, -4, -2, 1, 3, 5, 7, 8, 10, 11, 12, 12, 12, 12, 12, 11, 10, 8, 6, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -12, -12, -12, -12, -11, -10, -9, -7, -5,
 -3, -1, 1, 3, 5, 7, 9, 10, 11, 12, 12, 12, 12, 12, 11, 9, 8, 6, 4, 2, 2, 1, -1, -3, -2, 0, 2, -15, -14, -14, -12, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15,
 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -14, -13, -12, -10, -9, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11,
 9, 7, 5, 2, 1, -2, -2, 0, 3, 1, -2, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, -4, -10, -10, -4, 5,
 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, -2, -11, -6, 6, 11, 2, -9, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10,
 -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1,
 -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -15, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5,
 -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12,
 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -15, -15, -15, -15, -15, -15,
 -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -9, -8, -8,
 -8, -7, -7, -7, -6, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9,
 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 6, -6, -15, -15, -6,
 6, 15, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10,
 -9, -9, -9, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4,
 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 13, -3, -16, -9, 9, 16, 3, -13, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2,
 -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10,
 -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1,
 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15,
 -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10,
 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10,
 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14,
 -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0,
 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 6, -6, -15, -15, -6, 6, 15, 13, 13, 13, 13, 13, 13, 13,
 12, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -4, -5, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11,
 -11, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6,
 -5, -5, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, -3, -16, -9, 9, 16, 3,
 -13, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, -16, -16, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14,
 14, 15, 15, 15, 15, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14,
 -14, -15, -15, -15, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 10, 11, 12,
 13, 13, 14, 14, 15, 15, 15, 16, 16, 13, 9, 3, -3, -9, -13, -16, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2,
 3, 4, 6, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 0, 0, -2, -3, -4, -5, -6, -7,
 -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -8, -7, -6, -6, -4, -3, -2, -1, 0, 1,
 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 6, -6, -15, -15, -6, 6, 15, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -7,
 -6, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3,
 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -7,
 -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, -3, -16, -9, 9, 16, 3, -13, 0,0,0,0,1,15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6,
 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4,
 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10,
 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2,
 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5,
 -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1,
 0, 0, 0, 0, 0, 0, 0, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6,
 -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14,
 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3,
 2, 2, 1, 0, 0, 0, 0, 0, -1, 0, 0, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -7,
 -7, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13,
 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 4,
 3, 2, 2, 2, 1, 0, 0, 0, -1, 0, 0, 0, 0, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7,
 -7, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 4,
 3, 3, 2, 2, 1, 1, 0, 0, -1, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7,
 -8, -9, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -9, -8, -8,
 -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12,
 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -2,
 -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11,
 -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14,
 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 4, 4, 3, 2, 1, 1, 0, 0, -1, -1, -1, -1, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4,
 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -14, -14, -13, -13, -13, -12, -12,
 -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14,
 14, 14, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -1, -1, 0, 1, 1, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 9, 8, 8, 7,
 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12,
 -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 0, -1, 0, 1, 1, -1, -1, -15, -15, -15, -14, -14, -14, -13, -12, -11,
 -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1,
 -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0,
 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 0, -15, -15, -15, -14,
 -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 3, 4, 5, 6, 7, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10,
 9, 8, 7, 6, 4, 3, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -9, -8,
 -6, -5, -4, -3, -2, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 0, 0, -1, -1,
 -1, -1, -14, -14, -13, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -5, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 13,
 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -14, -14, -13, -13, -12, -12,
 -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 5,
 4, 2, 1, 0, -1, -1, -1, 1, 1, 1, -12, -12, -12, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -12, -12, -12, -12, -12, -12,
 -12, -12, -11, -11, -10, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 9, 9,
 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, 0, 1, 1, -1, -1, 0,0,1,3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 6,
 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6,
 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3,
 -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -4,
 -4, -4, -4, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, -12,
 -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2,
 -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, -1, 0, 0, 0, -15,
 -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3,
 -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, 0,
 1, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3,
 -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5,
 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7,
 -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7,
 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, -1,
 -1, -2, -3, -4, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -10, -10,
 -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, -1, -1, -1, 0, 1, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3,
 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13,
 -13, -12, -12, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15,
 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 4, 4, 3, 2, 1, 1, -1, -1, 0, 1, 0, -1, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1,
 -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2,
 -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, -8, -8, -8, -8, -8, -7, -7, -6, -6, -6, -5, -4, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7,
 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7,
 -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 1, 1, 1,
 1, 0, -1, -1, -12, -12, -12, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11,
 -10, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2,
 1, 1, 1, -1, -1, -1, 0, 1, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 3, 4, 5, 6, 7, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15,
 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, -15, -15,
 -14, -14, -14, -13, -12, -12, -11, -10, -9, -8, -6, -5, -4, -3, -2, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12,
 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, -1, -1, 0, 1, 1, -1, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7,
 -9, -10, -11, -11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11, -10,
 -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, -15, -6, 6, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14, -15, -14, -6, 6, 14, 12, 8,
 3, -3, -8, -12, -14, -14, -12, -8, -3, 3, 8, 12, 14, -14, -6, 6, 14, 9, 1, -7, -13, -15, -11, -4, 4, 11, 15, 13, 7, -1, -9, -14, 8, -8, -8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 11, -11, -11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11, -11, -11, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11,
 11, -11, -11, 11, 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11, -4, 10, -10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -6, 15, -15, 6, 6, 6, 5, 4, 3,
 2, 1, -1, -2, -3, -4, -5, -6, -6, -6, -6, 14, -14, 6, 5, 4, 1, -1, -4, -5, -6, -6, -5, -4, -1, 1, 4, 5, 6, -6, 14, -14, 6, 4, 1, -3, -6, -6, -5, -2, 2, 5, 6, 6, 3, -1,
 -4, -6, 0,0,0,1,5, 9, 13, 14, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 5, 9, 13, 14, 13, 11, 8, 4, 0, -4, -8, -11, -13, -14, -14, -13, -11, -8, -4, 5, 9, 13,
 14, 11, 5, -1, -8, -13, -14, -13, -9, -3, 4, 10, 14, 14, 12, 7, 5, 9, 12, 14, 8, -1, -10, -14, -13, -5, 4, 12, 14, 11, 3, -7, -13, -14, -9, -13, -13, 0, 13, 13, 13, 12,
 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, -13, -13, 0, 13, 12, 10, 7, 4, 0, -4, -7, -10, -12, -13, -13, -12, -10, -7, -4, -13, -13, 0, 13, 10, 5, -1, -7, -11, -13, -12,
 -8, -2, 4, 9, 12, 13, 11, 6, -12, -12, 0, 12, 7, -1, -9, -13, -11, -5, 4, 11, 13, 10, 2, -6, -12, -13, -8, 14, -5, -13, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1,
 14, -5, -13, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, -9, -9, -7, -5, -3, 14, -5, -13, 9, 7, 4, -1, -5, -8, -9, -9, -6, -2, 3, 7, 9, 9, 8, 4, 14, -5, -12, 9, 5, -1, -7, -9,
 -8, -4, 3, 8, 9, 7, 2, -4, -9, -9, -6, -9, 14, -13, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, -9, 14, -13, 5, 5, 4, 3, 1, 0, -1, -3, -4, -5, -5, -5, -5, -4, -3,
 -1, -9, 14, -13, 5, 4, 2, 0, -3, -4, -5, -5, -3, -1, 1, 4, 5, 5, 4, 2, -9, 14, -12, 5, 3, 0, -4, -5, -4, -2, 1, 4, 5, 4, 1, -2, -5, -5, -3, 1,14, 13, 9, 5, 5, 5, 5, 5,
 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 14, 13, 9, 5, 5, 4, 3, 1, 0, -1, -3, -4, -5, -5, -5, -5, -4, -3, -1, 14, 13, 9, 5, 4, 2, 0, -3, -4, -5, -5, -3, -1, 1, 4, 5, 5, 4, 2,
 14, 12, 9, 5, 3, 0, -4, -5, -4, -2, 1, 4, 5, 4, 1, -2, -5, -5, -3, -13, 0, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, -13, 0, 13, 13, 12, 10, 7, 4, 0,
 -4, -7, -10, -12, -13, -13, -12, -10, -7, -4, -13, 0, 13, 13, 10, 5, -1, -7, -11, -13, -12, -8, -2, 4, 9, 12, 13, 11, 6, -12, 0, 12, 12, 7, -1, -9, -13, -11, -5, 4, 11,
 13, 10, 2, -6, -12, -13, -8, 9, -13, -5, 14, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 9, -13, -5, 14, 13, 11, 8, 4, 0, -4, -8, -11, -13, -14, -14, -13, -11,
 -8, -4, 9, -13, -5, 14, 11, 5, -1, -8, -13, -14, -13, -9, -3, 4, 10, 14, 14, 12, 7, 9, -12, -5, 14, 8, -1, -10, -14, -13, -5, 4, 12, 14, 11, 3, -7, -13, -14, -9, -5, 13,
 -14, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, -5, 13, -14, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, -9, -9, -7, -5, -3, -5, 13, -14, 9, 7, 4, -1, -5, -8, -9, -9, -6,
 -2, 3, 7, 9, 9, 8, 4, -5, 12, -14, 9, 5, -1, -7, -9, -8, -4, 3, 8, 9, 7, 2, -4, -9, -9, -6, 0,1,0, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, 1, 3,
 4, 4, 8, 11, 13, 14, 14, 13, 11, 8, 4, 0, -4, -8, -11, -13, -14, 2, 4, 6, 7, 12, 14, 14, 10, 4, -3, -9, -13, -14, -13, -8, -1, 5, 11, 14, 3, 6, 8, 9, 14, 13, 7, -3, -11,
 -14, -12, -4, 5, 13, 14, 10, 1, -8, -14, -1, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, -4, -4, 0, 4, 7, 10, 12, 13, 13, 12, 10, 7, 4, 0, -4, -7,
 -10, -12, -13, -6, -6, 0, 6, 11, 13, 12, 9, 4, -2, -8, -12, -13, -11, -7, -1, 5, 10, 13, -8, -8, 0, 8, 13, 12, 6, -2, -10, -13, -11, -4, 5, 11, 13, 9, 1, -7, -12, 1, 0,
 -1, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, 4, -1, -4, 3, 5, 7, 9, 9, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, 7, -2, -6, 4, 8, 9, 9, 7, 3, -2, -6, -9, -9, -8, -5,
 -1, 4, 7, 9, 9, -3, -8, 6, 9, 9, 4, -2, -7, -9, -8, -3, 4, 8, 9, 7, 1, -5, -9, -1, 1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, -3, 4, -4, 1, 3, 4, 5, 5, 5, 5,
 4, 3, 1, 0, -1, -3, -4, -5, -5, -4, 7, -6, 2, 4, 5, 5, 4, 1, -1, -3, -5, -5, -4, -3, 0, 2, 4, 5, -6, 9, -8, 3, 5, 5, 2, -1, -4, -5, -4, -1, 2, 4, 5, 4, 0, -3, -5, 1,1,
 1, 1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 4, 4, 3, 1, 3, 4, 5, 5, 5, 5, 4, 3, 1, 0, -1, -3, -4, -5, -5, 7, 6, 4, 2, 4, 5, 5, 4, 1, -1, -3, -5, -5, -4, -3, 0,
 2, 4, 5, 9, 8, 6, 3, 5, 5, 2, -1, -4, -5, -4, -1, 2, 4, 5, 4, 0, -3, -5, -1, 0, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, -4, 0, 4, 4, 7, 10, 12, 13,
 13, 12, 10, 7, 4, 0, -4, -7, -10, -12, -13, -6, 0, 6, 6, 11, 13, 12, 9, 4, -2, -8, -12, -13, -11, -7, -1, 5, 10, 13, -8, 0, 8, 8, 13, 12, 6, -2, -10, -13, -11, -4, 5,
 11, 13, 9, 1, -7, -12, 1, -1, 0, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 14, 3, -4, -1, 4, 8, 11, 13, 14, 14, 13, 11, 8, 4, 0, -4, -8, -11, -13, -14, 4, -6,
 -2, 7, 12, 14, 14, 10, 4, -3, -9, -13, -14, -13, -8, -1, 5, 11, 14, 6, -8, -3, 9, 14, 13, 7, -3, -11, -14, -12, -4, 5, 13, 14, 10, 1, -8, -14, 0, 1, -1, 1, 2, 3, 4, 4,
 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, -1, 4, -4, 3, 5, 7, 9, 9, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, -2, 6, -7, 4, 8, 9, 9, 7, 3, -2, -6, -9, -9, -8, -5, -1, 4, 7, 9, -3, 8,
 -9, 6, 9, 9, 4, -2, -7, -9, -8, -3, 4, 8, 9, 7, 1, -5, -9, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10,
 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -13, -9, -3, 3,
 9, 13, 16, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -16, -15, -13, -9, -3, 3, 9, 13, 15, 13, 9, 3, -3, -9, -13, -15, -15, -13, -9, -3, 3, 9, 13, 15, -15,
 -13, -8, -3, 3, 8, 13, 15, 10, 2, -7, -14, -16, -12, -4, 4, 12, 16, 14, 7, -2, -10, -15, 10, 5, -4, -10, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 15, 6, -6, -15, -15, -6, 6, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14, -15, 14, 6, -6, -14, -14, -6, 6, 14, 12, 8, 3, -3, -8, -12, -14, -14,
 -12, -8, -3, 3, 8, 12, 14, 14, 6, -6, -14, -14, -6, 6, 14, 9, 1, -7, -13, -15, -11, -4, 4, 11, 15, 13, 7, -1, -9, -14, -9, 2, 11, 6, -6, -11, -2, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, -13, 3, 16, 9, -9, -16, -3, 13, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -13, -13, 3, 15, 9, -9, -15, -3, 13, 11, 7, 3, -3, -7,
 -11, -13, -13, -11, -7, -3, 3, 7, 11, 13, -13, 3, 15, 8, -8, -15, -3, 13, 8, 1, -6, -12, -13, -10, -4, 4, 10, 13, 12, 6, -1, -8, -13, 0,0,0,1,3, 6, 8, 10, 12, 13, 15,
 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 4, 3, 1, 3, 6, 8, 10, 12, 13, 15, 15, 14, 11, 8, 4, 0, -4, -8, -11, -14, -15, -15, -14, -11, -8, -4, 3, 5, 8, 10, 12, 13,
 14, 15, 11, 6, -1, -8, -13, -15, -14, -9, -3, 4, 10, 14, 15, 12, 7, 3, 5, 8, 10, 12, 13, 14, 14, 8, -1, -10, -15, -13, -6, 4, 12, 15, 11, 3, -7, -14, -15, -9, -8, -13,
 -15, -12, -5, 3, 10, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, -8, -13, -15, -12, -5, 3, 10, 15, 13, 11, 8, 4, 0, -4, -8, -11, -13, -15, -15, -13, -11,
 -8, -4, -8, -13, -15, -12, -5, 3, 10, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4, 10, 14, 15, 12, 7, -8, -13, -14, -12, -5, 3, 10, 14, 8, -1, -10, -15, -13, -5, 4, 12,
 15, 11, 3, -7, -13, -14, -9, 12, 15, 6, -8, -15, -10, 3, 13, 13, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 1, 12, 15, 6, -8, -15, -10, 3, 13, 12, 10, 7, 4, 0, -4, -7,
 -10, -12, -13, -13, -12, -10, -7, -4, 12, 14, 5, -8, -15, -10, 3, 13, 10, 5, -1, -7, -12, -13, -12, -8, -3, 4, 9, 13, 13, 11, 6, 12, 14, 5, -8, -14, -10, 3, 13, 7, -1,
 -9, -13, -12, -5, 4, 11, 13, 10, 3, -6, -12, -13, -8, -15, -8, 10, 13, -3, -15, -5, 12, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 3, 2, 1, -15, -8, 10, 13, -3, -15, -5,
 12, 11, 9, 7, 3, 0, -3, -7, -9, -11, -12, -12, -11, -9, -7, -3, -14, -8, 10, 13, -3, -15, -5, 12, 9, 5, -1, -7, -11, -12, -11, -8, -2, 3, 9, 12, 12, 10, 6, -14, -8, 10,
 13, -3, -14, -5, 12, 7, -1, -9, -12, -11, -5, 3, 10, 12, 9, 2, -6, -11, -12, -8, 1,15, 15, 13, 12, 10, 8, 6, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 15, 15, 13,
 12, 10, 8, 6, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, 15, 14, 13, 12, 10, 8, 5, 3, 2, 1, 0, -2, -3, -3, -3, -2, -1, 1, 2, 3, 3, 2, 1, 14, 14, 13, 12,
 10, 8, 5, 3, 2, 0, -2, -3, -3, -1, 1, 2, 3, 2, 1, -1, -3, -3, -2, -15, -10, -3, 6, 12, 15, 13, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, -15, -10, -3, 6, 12, 15,
 13, 8, 7, 6, 4, 2, 0, -2, -4, -6, -7, -8, -8, -7, -6, -4, -2, -14, -10, -3, 5, 12, 15, 13, 8, 6, 3, -1, -4, -7, -8, -7, -5, -2, 2, 6, 8, 8, 7, 4, -14, -10, -3, 5, 12,
 14, 13, 8, 4, -1, -6, -8, -7, -3, 2, 7, 8, 6, 2, -4, -7, -8, -5, 13, 3, -10, -15, -8, 6, 15, 12, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 3, 2, 1, 13, 3, -10, -15, -8,
 6, 15, 12, 11, 9, 7, 3, 0, -3, -7, -9, -11, -12, -12, -11, -9, -7, -3, 13, 3, -10, -15, -8, 5, 14, 12, 9, 5, -1, -7, -11, -12, -11, -8, -2, 3, 9, 12, 12, 10, 6, 13, 3,
 -10, -14, -8, 5, 14, 12, 7, -1, -9, -12, -11, -5, 3, 10, 12, 9, 2, -6, -11, -12, -8, -12, 6, 15, 3, -13, -10, 8, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1,
 -12, 6, 15, 3, -13, -10, 8, 15, 13, 11, 8, 4, 0, -4, -8, -11, -13, -15, -15, -13, -11, -8, -4, -12, 5, 15, 3, -13, -10, 8, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4,
 10, 14, 15, 12, 7, -12, 5, 14, 3, -13, -10, 8, 14, 8, -1, -10, -15, -13, -5, 4, 12, 15, 11, 3, -7, -13, -14, -9, 0,1,0, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 9, 10, 11,
 12, 13, 14, 14, 15, 15, 15, 1, 2, 2, 3, 3, 4, 4, 4, 8, 11, 14, 15, 15, 14, 11, 8, 4, 0, -4, -8, -11, -14, -15, 1, 3, 4, 5, 6, 6, 7, 7, 12, 15, 14, 10, 4, -3, -9, -14,
 -15, -13, -8, -1, 6, 11, 15, 2, 3, 5, 6, 8, 8, 9, 9, 15, 14, 7, -3, -11, -15, -12, -4, 6, 13, 15, 10, 1, -8, -14, -1, -1, -1, -1, 0, 0, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11,
 12, 13, 13, 14, 14, 15, 15, -2, -4, -4, -3, -2, 1, 3, 4, 8, 11, 13, 15, 15, 13, 11, 8, 4, 0, -4, -8, -11, -13, -15, -4, -6, -7, -6, -2, 1, 5, 7, 12, 15, 14, 10, 4, -3,
 -9, -13, -15, -13, -8, -1, 5, 11, 14, -5, -8, -9, -8, -3, 2, 6, 9, 14, 13, 7, -3, -11, -15, -12, -4, 5, 13, 15, 10, 1, -8, -14, 1, 1, 1, -1, -1, -1, 0, 1, 3, 4, 5, 6, 7,
 8, 9, 10, 11, 12, 12, 13, 13, 13, 13, 3, 4, 2, -2, -4, -3, 1, 4, 7, 10, 12, 13, 13, 12, 10, 7, 4, 0, -4, -7, -10, -12, -13, 6, 7, 3, -4, -7, -5, 1, 6, 11, 13, 13, 9, 4,
 -3, -8, -12, -13, -12, -7, -1, 5, 10, 13, 8, 9, 3, -5, -9, -6, 2, 8, 13, 12, 6, -3, -10, -13, -11, -4, 5, 12, 13, 9, 1, -7, -13, -1, -1, 1, 1, 0, -1, 0, 1, 2, 3, 5, 6,
 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, -4, -2, 3, 4, -1, -4, -2, 3, 7, 9, 11, 12, 12, 11, 9, 7, 3, 0, -3, -7, -9, -11, -12, -7, -4, 5, 6, -1, -7, -2, 6, 10, 12, 12, 9,
 3, -2, -8, -11, -12, -11, -7, -1, 5, 9, 12, -9, -5, 6, 8, -2, -9, -3, 8, 12, 11, 6, -2, -9, -12, -10, -3, 5, 11, 12, 9, 1, -7, -12, 1,1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 3, 3, 2, 2, 1, 2, 2, 3, 3, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, 7, 7, 6, 6, 5, 4, 3, 1, 2, 3, 3, 2, 1, -1, -2, -3, -3, -3, -2,
 0, 1, 2, 3, 9, 9, 8, 8, 6, 5, 3, 2, 3, 3, 1, -1, -2, -3, -2, -1, 1, 3, 3, 2, 0, -2, -3, -1, -1, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 7, 7, 8, 8, 8, 8, -4, -3,
 -1, 2, 3, 4, 4, 2, 4, 6, 7, 8, 8, 7, 6, 4, 2, 0, -2, -4, -6, -7, -8, -7, -5, -1, 3, 6, 7, 6, 4, 7, 8, 8, 6, 2, -2, -5, -7, -8, -7, -4, -1, 3, 6, 8, -9, -6, -2, 3, 8, 9,
 8, 5, 8, 7, 4, -2, -6, -8, -7, -2, 3, 7, 8, 6, 1, -4, -8, 1, 0, -1, -1, -1, 1, 1, 1, 2, 3, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, 4, 1, -3, -4, -2, 2, 4, 3, 7, 9,
 11, 12, 12, 11, 9, 7, 3, 0, -3, -7, -9, -11, -12, 6, 1, -5, -7, -4, 3, 7, 6, 10, 12, 12, 9, 3, -2, -8, -11, -12, -11, -7, -1, 5, 9, 12, 8, 2, -6, -9, -5, 3, 9, 8, 12,
 11, 6, -2, -9, -12, -10, -3, 5, 11, 12, 9, 1, -7, -12, -1, 1, 1, 0, -1, -1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, -3, 2, 4, 1, -4, -3, 2, 4, 8, 11,
 13, 15, 15, 13, 11, 8, 4, 0, -4, -8, -11, -13, -15, -6, 3, 7, 1, -6, -5, 4, 7, 12, 15, 14, 10, 4, -3, -9, -13, -15, -13, -8, -1, 5, 11, 14, -8, 3, 9, 2, -8, -6, 5, 9,
 14, 13, 7, -3, -11, -15, -12, -4, 5, 13, 15, 10, 1, -8, -14, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5,
 -1, -7, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -14, -12, -10, -8, -4,
 -2, 2, 4, 8, 10, 12, 14, 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, 13, 9,
 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8,
 -2, -10, -15, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 13, 9, 3, -3, -9, -13, -16,
 -16, -13, -9, -3, 3, 9, 13, 16, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -16, 15, 13, 9, 3, -3, -9, -13, -15, -15, -13, -9, -3, 3, 9, 13, 15, 13, 9, 3,
 -3, -9, -13, -15, -15, -13, -9, -3, 3, 9, 13, 15, 15, 13, 8, 3, -3, -8, -13, -15, -15, -13, -8, -3, 3, 8, 13, 15, 10, 2, -7, -14, -16, -12, -4, 4, 12, 16, 14, 7, -2,
 -10, -15, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -15, -10, -2, 8, 14, 16, 12, 4, -4,
 -12, -16, -14, -8, 2, 10, 15, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -15, -15, -10, -2, 7, 14, 16, 12, 4, -4, -12, -16, -14, -7, 2, 10, 15, 13, 8, 3,
 -3, -8, -13, -15, -15, -13, -8, -3, 3, 8, 13, 15, -15, -10, -2, 7, 14, 15, 12, 4, -4, -12, -15, -14, -7, 2, 10, 15, 10, 2, -7, -14, -15, -12, -4, 4, 12, 15, 14, 7, -2,
 -10, -15, 0,0,0,1,1, 3, 4, 6, 7, 8, 9, 11, 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 14, 13, 13, 12, 11, 9, 8, 7, 6, 4, 3, 1, 1, 3, 4, 6, 7, 8, 9, 11, 12, 13, 13, 14,
 15, 15, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 1, 3, 4, 6, 7, 8, 9, 11, 12, 12, 13, 14, 14, 15, 15, 15, 12, 6, -1, -8, -13, -15, -14, -9,
 -3, 4, 11, 15, 15, 13, 7, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 8, -1, -11, -15, -13, -6, 4, 13, 15, 12, 3, -7, -14, -15, -9, -4, -8, -12, -14, -15,
 -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 15, 15, 14, 13, 13, 12, 11, 9, 8, 7, 6, 4, 3, 1, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 14,
 12, 8, 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 12, 6, -1, -8, -13, -15, -14, -9, -3, 4,
 11, 15, 15, 13, 7, -4, -8, -11, -13, -15, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13, 15, 8, -1, -11, -15, -13, -6, 4, 13, 15, 12, 3, -7, -14, -15, -9, 7, 13, 15, 15, 11, 4,
 -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 15, 15, 14, 14, 13, 12, 12, 11, 9, 8, 7, 6, 4, 3, 1, 7, 13, 15, 15, 11, 4, -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 14, 12, 8,
 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 7, 12, 15, 14, 11, 4, -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 12, 6, -1, -8, -13, -15, -14, -9, -3, 4, 11, 14, 15,
 12, 7, 7, 12, 15, 14, 10, 4, -3, -9, -13, -15, -13, -8, -1, 5, 11, 14, 8, -1, -11, -15, -13, -6, 4, 12, 15, 12, 3, -7, -14, -15, -9, -9, -15, -14, -7, 3, 12, 15, 13, 4,
 -6, -13, -15, -11, -1, 8, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, -9, -15, -14, -7, 3, 12, 15, 13, 4, -6, -13, -15, -11, -1, 8, 15, 13, 11, 8, 4, 0, -4,
 -8, -11, -13, -15, -15, -13, -11, -8, -4, -9, -15, -14, -7, 3, 12, 15, 12, 4, -6, -13, -15, -11, -1, 8, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4, 10, 14, 15, 12, 7,
 -9, -14, -13, -7, 3, 11, 15, 12, 4, -5, -13, -15, -10, -1, 8, 14, 8, -1, -10, -15, -13, -5, 4, 12, 15, 11, 3, -7, -13, -14, -9, 1,15, 15, 15, 15, 14, 13, 13, 12, 11, 9,
 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 15, 15, 15, 15, 14, 13, 13, 12, 11, 9, 8, 7, 6, 4, 3, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1,
 0, 15, 15, 15, 14, 14, 13, 12, 12, 11, 9, 8, 7, 6, 4, 3, 1, 1, 1, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1,
 0, -1, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, -1, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, -15, -14,
 -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -4, -4, -3, -2, -1, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4,
 3, 2, 0, -2, -4, -4, -4, -3, -1, 1, 3, 4, 4, 4, 2, -15, -13, -11, -8, -4, 0, 4, 8, 11, 13, 15, 15, 13, 11, 8, 4, 2, 0, -3, -4, -4, -2, 1, 4, 4, 3, 1, -2, -4, -4, -3, 15,
 12, 6, -1, -8, -13, -15, -14, -9, -3, 4, 11, 15, 15, 13, 7, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 15, 12, 6, -1, -8, -13, -15, -14, -9, -3, 4, 11, 15, 15, 13, 7,
 6, 5, 4, 2, 0, -2, -4, -5, -6, -7, -7, -6, -5, -4, -2, 15, 12, 6, -1, -8, -13, -15, -14, -9, -3, 4, 11, 14, 15, 12, 7, 5, 3, -1, -4, -6, -7, -6, -4, -1, 2, 5, 7, 7, 6,
 3, 14, 11, 5, -1, -8, -13, -15, -13, -9, -3, 4, 10, 14, 15, 12, 7, 4, -1, -5, -7, -6, -3, 2, 6, 7, 5, 1, -3, -6, -7, -4, -15, -8, 1, 11, 15, 13, 6, -4, -13, -15, -12,
 -3, 7, 14, 15, 9, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, -15, -8, 1, 11, 15, 13, 6, -4, -13, -15, -12, -3, 7, 14, 15, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, -9, -9,
 -7, -5, -3, -14, -8, 1, 11, 15, 13, 6, -4, -12, -15, -12, -3, 7, 14, 15, 9, 7, 4, -1, -5, -8, -9, -9, -6, -2, 3, 7, 9, 9, 8, 4, -14, -8, 1, 10, 15, 13, 5, -4, -12, -15,
 -11, -3, 7, 13, 14, 9, 5, -1, -7, -9, -8, -4, 3, 8, 9, 7, 2, -4, -9, -9, -6, 0,1,0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 9, 11, 12, 13, 13, 14,
 15, 15, 15, 15, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7,
 7, 13, 15, 15, 11, 4, -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, 15, 14, 7, -3, -12, -15, -13, -4, 6, 13, 15, 11, 1, -8,
 -15, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 3, 4, 6, 7, 8, 9, 11, 12, 13, 13, 14, 15, 15, 15, 15, -1, -2, -3, -4, -4, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4,
 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -2, -4, -5, -6, -7, -7, -6, -5, -4, -2, 0, 2, 4, 5, 6, 7, 13, 15, 15, 11, 4, -3, -9, -14, -15, -13, -8, -1,
 6, 12, 15, -3, -5, -7, -9, -9, -9, -9, -7, -5, -3, 0, 3, 5, 7, 9, 9, 15, 14, 7, -3, -12, -15, -13, -4, 6, 13, 15, 11, 1, -8, -15, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1,
 -1, 0, 1, 1, 1, 3, 4, 6, 7, 8, 9, 11, 12, 12, 13, 14, 14, 15, 15, 15, 2, 4, 4, 4, 3, 1, -1, -3, -4, -4, -4, -2, 0, 2, 3, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8,
 -12, -14, -15, 3, 6, 7, 7, 5, 2, -1, -4, -6, -7, -6, -4, -1, 3, 5, 7, 12, 15, 14, 11, 4, -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 4, 8, 9, 9, 7, 3, -2, -6, -9, -9, -8,
 -5, -1, 4, 7, 9, 15, 14, 7, -3, -12, -15, -12, -4, 6, 13, 15, 11, 1, -8, -14, -1, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, -1, 0, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13,
 13, 14, 14, 15, 15, -3, -4, -4, -2, 1, 3, 4, 4, 1, -2, -4, -4, -3, 0, 2, 4, 8, 11, 13, 15, 15, 13, 11, 8, 4, 0, -4, -8, -11, -13, -15, -4, -7, -6, -3, 1, 5, 7, 6, 2, -3,
 -6, -7, -5, -1, 4, 7, 12, 15, 14, 10, 4, -3, -9, -13, -15, -13, -8, -1, 5, 11, 14, -6, -9, -9, -4, 2, 7, 9, 8, 3, -4, -8, -9, -7, -1, 5, 9, 14, 13, 7, -3, -11, -15, -12,
 -4, 5, 13, 15, 10, 1, -8, -14, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1,
 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, 7, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 1, 1, 1, 9, 9, 9, 9, 9,
 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 1, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4,
 4, 4, 4, 4, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 4, 4, 3, 2, 1, 2, 3, 4, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -7, -6, -5, -4, -2, 0, 2, 4, 5, 6, 7, 7, 6, 5, 4, 2,
 4, 4, 4, 3, 1, -1, -3, -4, -4, -4, -2, 0, 2, 3, 4, -9, -9, -7, -5, -3, 0, 3, 5, 7, 9, 9, 9, 9, 7, 5, 3, 4, 4, 2, -1, -3, -4, -4, -1, 2, 4, 4, 3, 0, -2, -4, 1, 1, 1, 0,
 -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 4, 3, 2, 0, -2, -4, -4, -4, -3, -1, 1, 3, 4, 4, 4, 2, 4, 5, 6, 7, 7, 6, 5, 4, 2, 0,
 -2, -4, -5, -6, -7, 7, 5, 3, -1, -4, -6, -7, -6, -4, -1, 2, 5, 7, 7, 6, 3, 6, 7, 7, 5, 2, -1, -4, -6, -7, -6, -4, -1, 3, 5, 7, 9, 7, 4, -1, -5, -8, -9, -9, -6, -2, 3, 7,
 9, 9, 8, 4, 7, 6, 3, -1, -5, -7, -6, -2, 3, 6, 7, 5, 1, -4, -7, -1, -1, 0, 1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 9, -4, -2,
 0, 3, 4, 4, 2, -1, -4, -4, -3, -1, 2, 4, 4, 3, 5, 7, 9, 9, 9, 9, 7, 5, 3, 0, -3, -5, -7, -9, -9, -7, -4, 1, 5, 7, 6, 3, -2, -6, -7, -5, -1, 3, 6, 7, 4, 8, 9, 9, 7, 3,
 -2, -6, -9, -9, -8, -5, -1, 4, 7, 9, -9, -5, 1, 7, 9, 8, 4, -3, -8, -9, -7, -2, 4, 9, 9, 6, 9, 9, 4, -2, -7, -9, -8, -3, 4, 8, 9, 7, 1, -5, -9, 1,8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11,
 -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 15, 14, 12, 10, 8, 4,
 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16,
 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13,
 14, 14, 15, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7,
 -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16,
 -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, 16, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10,
 -12, -14, -15, -16, -16, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 15, 14, 12, 10, 7,
 4, 2, -2, -4, -7, -10, -12, -14, -15, -15, -15, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10,
 -15, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5, -8, -10, -11, -11, -11, -9, -7, -4, 0, 3, 6, 8, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, -16, -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 15, 14, 12, 10, 8,
 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14,
 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15, -14, -11, -8, -4, 1, 5, 9, 12, 14, 15, 15, 13, 10, 6, 2, -2, -6, -10, -13, -15, -15, -14, -12, -9,
 -5, -1, 4, 8, 11, 14, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 0,0,0,1,1, 2, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13,
 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13,
 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12,
 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11,
 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 8, -1, -11, -15, -14, -6, 4, 13, 15, 12, 3, -7, -14, -15, -10, -2, -4, -7, -9, -10, -12, -13, -14, -15, -15,
 -15, -15, -15, -14, -13, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, -2, -4, -7, -9, -10, -12,
 -13, -14, -15, -15, -15, -15, -15, -14, -13, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8,
 -4, -2, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -14, -13, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 12, 6, -1, -8, -14, -15, -14,
 -10, -3, 4, 11, 15, 15, 13, 7, -2, -4, -6, -8, -10, -11, -13, -14, -14, -15, -15, -15, -14, -13, -12, -11, -9, -8, -6, -3, -1, 1, 3, 5, 7, 9, 10, 12, 13, 14, 14, 15, 8,
 -1, -11, -15, -14, -6, 4, 13, 15, 12, 3, -7, -14, -15, -10, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4,
 7, 10, 13, 14, 15, 15, 15, 15, 14, 13, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10,
 -7, -4, 0, 4, 7, 10, 13, 14, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14,
 -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 12, 6, -1, -8, -13, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3,
 -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 8, -1, -11, -15, -13, -6, 4, 13, 15, 12, 3, -7, -14, -15, -10, -5, -10, -13, -15, -15, -14,
 -11, -7, -2, 4, 9, 12, 15, 15, 14, 12, 8, 3, -2, -7, -11, -14, -15, -15, -13, -9, -4, 1, 6, 10, 13, 15, 15, 15, 15, 14, 13, 13, 12, 11, 9, 8, 7, 6, 4, 3, 1, -5, -10,
 -13, -15, -15, -14, -11, -7, -2, 4, 9, 12, 15, 15, 14, 12, 8, 3, -2, -7, -11, -14, -15, -15, -13, -9, -4, 1, 6, 10, 13, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -15,
 -14, -12, -8, -4, -5, -10, -13, -15, -15, -14, -11, -6, -2, 4, 8, 12, 14, 15, 14, 12, 8, 3, -2, -7, -11, -14, -15, -15, -13, -9, -4, 1, 6, 10, 13, 15, 12, 6, -1, -8,
 -13, -15, -14, -9, -3, 4, 11, 15, 15, 13, 7, -5, -9, -13, -14, -15, -13, -10, -6, -1, 3, 8, 12, 14, 15, 14, 11, 8, 3, -2, -7, -11, -14, -15, -14, -12, -9, -4, 1, 6, 10,
 13, 15, 8, -1, -11, -15, -13, -6, 4, 13, 15, 12, 3, -7, -14, -15, -9, 1,15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5,
 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3,
 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2,
 2, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 0, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1,
 0, 0, 0, -1, -1, 0, 0, 1, 1, 1, 0, 0, -1, -1, 0, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 11, 13, 14, 15, 15, 15, 15, 15, 14, 13, 12, 10, 9, 7,
 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 11, 13, 14, 15, 15, 15, 15, 15, 14, 13, 12, 10, 9,
 7, 4, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -1, -1, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 11, 13, 14, 14, 15, 15, 15, 15, 14, 13,
 12, 10, 8, 6, 4, 2, 2, 1, 0, -1, -2, -2, -2, -1, 0, 1, 2, 2, 2, 2, 1, -15, -14, -14, -13, -12, -10, -9, -7, -5, -3, -1, 1, 3, 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, 14,
 14, 13, 11, 10, 8, 6, 4, 2, 1, 0, -2, -2, -2, -1, 1, 2, 2, 2, 0, -1, -2, -2, -1, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4,
 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0,
 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 3, 3, 2, 1, 0, -1, -2, -3, -3, -4, -4, -3, -3, -2, -1, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13,
 -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 3, 1, 0, -2, -3, -4, -3, -2, -1, 1, 3, 3, 4, 3, 2, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15,
 -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 2, 0, -3, -4, -3, -1, 1, 3, 4, 3, 1, -2, -3, -4, -2, -15, -13, -10, -6, -1, 4, 9, 13, 15, 15, 14,
 11, 7, 2, -3, -8, -12, -14, -15, -15, -12, -9, -4, 2, 7, 11, 14, 15, 15, 13, 10, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, -15, -13, -10, -6, -1, 4, 9, 13, 15, 15,
 14, 11, 7, 2, -3, -8, -12, -14, -15, -15, -12, -9, -4, 2, 7, 11, 14, 15, 15, 13, 10, 5, 5, 4, 3, 1, 0, -1, -3, -4, -5, -5, -5, -5, -4, -3, -1, -15, -13, -10, -6, -1, 4,
 9, 13, 15, 15, 14, 11, 7, 2, -3, -8, -12, -14, -15, -14, -12, -8, -4, 2, 6, 11, 14, 15, 15, 13, 10, 5, 4, 2, 0, -3, -5, -5, -5, -3, -1, 1, 4, 5, 5, 4, 2, -15, -13, -10,
 -6, -1, 4, 9, 12, 14, 15, 14, 11, 7, 2, -3, -8, -11, -14, -15, -14, -12, -8, -3, 1, 6, 10, 13, 15, 14, 13, 9, 5, 3, 0, -4, -5, -5, -2, 1, 4, 5, 4, 1, -2, -5, -5, -3, 1,
1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 10, 11, 11, 11, 11, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 6, 9, 10, 11, 11, 10, 9, 6, 3, 0, -3, -6, -8, -10, -11, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 9, 11, 11, 8, 3, -2, -7, -10, -11, -10, -6, -1, 4, 9, 11, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 11, 10, 5, -2, -8, -11, -9, -3, 4, 10, 11, 8, 1, -6, -11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2,
 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -7, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3,
 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -5, -4, -3, -2, -1, 0, 0, 1, 2,
 3, 4, 5, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 4, 4, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4, -3,
 -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, 7, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -7, -7, -7, -6, -5,
 -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 10, 9, 9, 8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -9, -9, -10, -10, -9,
 -9, -8, -6, -5, -3, -1, 1, 3, 5, 6, 8, 9, 9, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1,
 -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, -4, -4, -3, -2, -1, 0, 2, 3, 4, 4, 4, 4, 4, 3, 2, 1, -1, -2, -3,
 -4, -4, -4, -4, -4, -3, -2, 0, 1, 2, 3, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -7, -6, -5, -4, -2, 0, 2, 4, 6, 7, 7, 7, 6, 5, 3, 1, -1, -3, -5,
 -6, -7, -7, -7, -6, -4, -2, 0, 2, 4, 5, 6, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, -10, -9, -7, -5, -2, 0, 3, 6, 8, 9, 10, 9, 8, 7, 4, 1, -1,
 -4, -7, -8, -9, -10, -9, -8, -6, -3, 0, 2, 5, 7, 9, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 1,0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7,
 7, 7, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10,
 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, -1, -1, -2, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4,
 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -1, -2, -3, -4, -5, -5, -6, -6, -7, -7, -7, -7, -7, -6, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4,
 5, 6, 6, 7, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, -1, -3, -4, -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -9, -8, -7, -6, -5, -4, -2, -1, 0,
 2, 3, 5, 6, 7, 8, 8, 9, 9, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, 0, 0, 0, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 13, 14, 15, 15, 15, 15, 1, 2, 3, 4, 4, 4, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -4, -4, -4, -4, -3, -2, -1,
 0, 1, 2, 3, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, 2, 3, 5, 6, 7, 7, 7, 7, 6, 5, 3, 2, 0, -2, -3, -5, -6, -7, -7, -7, -7, -6, -5, -3, -2, 0,
 2, 3, 5, 6, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -13, -8, -1, 6, 12, 15, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5,
 -2, 0, 2, 5, 6, 8, 9, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 13, 15, 11, 1, -8, -15, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, 0, 0, 1, 1, 1, 1, 3, 4, 6, 7, 8, 9, 11, 12, 13, 13, 14, 15, 15, 15, 15, -1, -3, -4, -4, -4, -4, -3, -2, 0, 1, 2, 4, 4, 4, 4, 3, 2, 1, -1, -2, -3, -4, -4, -4,
 -4, -3, -1, 0, 2, 3, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -2, -4, -6, -7, -7, -6, -5, -3, -1, 2, 4, 6, 7, 7, 7, 5, 4, 1, -1, -3, -5, -6, -7,
 -7, -6, -4, -2, 0, 3, 5, 6, 7, 13, 15, 15, 11, 4, -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, -3, -6, -8, -9, -10, -9, -7, -4, -1, 2, 5, 8, 9, 10, 9, 7, 5, 2, -1, -5, -7,
 -9, -10, -9, -8, -6, -3, 0, 4, 6, 8, 9, 15, 14, 7, -3, -12, -15, -13, -4, 6, 13, 15, 11, 1, -8, -15, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0,
 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0,
 -1, -1, -1, 0, 0, 0, 1, 1, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 1, 1, 0, 0, -1, -1, -1, 0, 0, 1, 1, 0, 0,
 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, -4, -4, -4,
 -4, -4, -3, -3, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -7, -7, -7, -6, -6, -5,
 -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 6, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 1, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2, -1, 0, 1, 2, 2, -10, -9, -9, -8, -8, -7, -6, -5, -3,
 -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 4, 3, 1, 2, 2, 1, 0, -2, -2, -2, -1, 1, 2, 2, 2, 0, -1, -2, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -4, -4, -4, -4, -3,
 -2, -1, 0, 1, 2, 3, 4, 4, 4, 4, 4, 4, 3, 2, 1, 2, 3, 3, 4, 4, 3, 3, 2, 1, 0, -1, -2, -3, -3, -4, 7, 7, 6, 5, 3, 2, 0, -2, -3, -5, -6, -7, -7, -7, -7, -6, -5, -3, -2, 0,
 2, 3, 5, 6, 7, 7, 7, 7, 6, 5, 3, 2, 3, 4, 3, 3, 1, -1, -2, -3, -4, -3, -2, 0, 1, 3, 4, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5,
 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 4, 3, 2, -1, -3, -4, -3, -1, 1, 3, 4, 3, 0, -2, -3, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1,
 1, 1, 1, 1, 1, 1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, -4, -4, -3, -2, 0, 1, 3, 4, 4, 4, 4, 3, 2, 1, -1, -2, -3, -4, -4, -4, -4, -2, -1, 0, 2, 3, 4, 4, 4, 4,
 3, 1, 3, 4, 5, 5, 5, 5, 4, 3, 1, 0, -1, -3, -4, -5, -5, -7, -6, -5, -3, 0, 2, 4, 6, 7, 7, 6, 5, 3, 1, -1, -4, -5, -7, -7, -7, -6, -4, -2, 1, 3, 5, 6, 7, 7, 6, 4, 2, 4,
 5, 5, 4, 1, -1, -3, -5, -5, -5, -3, 0, 2, 4, 5, -9, -8, -6, -4, 0, 3, 6, 8, 9, 10, 9, 7, 5, 1, -2, -5, -7, -9, -10, -9, -8, -5, -2, 1, 4, 7, 9, 10, 9, 8, 6, 3, 5, 5, 2,
 -1, -4, -5, -4, -1, 2, 5, 5, 4, 0, -3, -5, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 7, 1, -5, -10,
 -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3, -3, -2,
 -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1,
 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15,
 -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4,
 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15, -15,
 -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -7, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7,
 8, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 11, 11, 11, 11, 10, 10,
 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0,
 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2,
 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9,
 11, 12, 13, 14, 14, 15, 15, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4,
 -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13,
 14, 14, 15, 15, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -8, -9,
 -10, -11, -12, -13, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 14, 15,
 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11, 11, 11, 11, 10, 9,
 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1,
 -4, -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8,
 -10, -12, -14, -15, -16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 6, 8, 10, 12, 13, 14, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 3, 1, -1, -3, -6, -8,
 -10, -11, -13, -14, -15, -16, -16, -16, -15, -14, -13, -12, -10, -8, -6, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3,
 9, 13, 16, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, 0, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 15, 15, 15, 14, 12, 11, 10, 7, 6, 3, 1, -1, -3, -6, -7, -10, -11, -12,
 -14, -15, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -4, -2, 0, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 0,
0,0,0,1,15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 9, 8,
 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12,
 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5,
 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -9, -8, -7, -5,
 -4, -3, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -9, -8, -7, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1,
 -1, -1, 0, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 14,
 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, -15, -15, -15, -14, -14,
 -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14,
 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 0, -1, -1, -1, 0, 0, 1, 1, 1, 0, -1, -1, -1, -1, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 4, 3, 1, -1, -3, -5, -7,
 -8, -10, -11, -12, -13, -14, -15, -15, -15, -15, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 3, 5, 7, 9, 10, 11, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 13,
 12, 10, 9, 7, 6, 4, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 4, 3, 1, -1, -3, -5, -7, -8, -10, -11, -12, -13, -14, -15, -15,
 -15, -15, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 3, 5, 7, 9, 10, 11, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 13, 12, 10, 9, 7, 6, 4, 2, 2, 1, 1, 1, 0,
 -1, -1, -1, -2, -2, -2, -2, -1, -1, -1, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 4, 3, 1, -1, -3, -5, -6, -8, -10, -11, -12, -13, -14, -15, -15, -15, -15, -15, -14, -14,
 -13, -12, -11, -9, -7, -6, -4, -2, 0, 2, 3, 5, 7, 8, 10, 11, 12, 13, 14, 15, 15, 15, 15, 15, 14, 14, 13, 12, 10, 9, 7, 5, 4, 2, 1, 1, 0, -1, -2, -2, -2, -1, 0, 1, 1, 2,
 2, 2, 1, 15, 15, 14, 14, 13, 12, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -15, -15, -14, -13, -13, -11, -10, -9, -7, -6, -4, -2,
 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, 1, 0, -1, -2, -2, -1, 1, 2, 2, 1, 0, -1, -2, -2, -1, -15, -15, -14, -13,
 -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 14, 15, 15, 15, 15, 15, 13, 12, 10, 8, 6, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -4,
 -2, 1, 4, 6, 9, 11, 12, 14, 15, 15, 15, 15, 14, 13, 12, 10, 8, 5, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, -15, -15, -14, -13, -11, -9, -7, -4, -2, 1, 3, 6, 8,
 10, 12, 14, 15, 15, 15, 15, 15, 13, 12, 10, 8, 6, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 12, 14, 15, 15,
 15, 15, 14, 13, 12, 10, 8, 5, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -2, -2, -1, -1, -15, -15, -14, -13, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15,
 14, 13, 12, 10, 8, 5, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -12, -11, -9, -6, -4, -2, 1, 4, 6, 8, 11, 12, 14, 15, 15, 15, 15, 14, 13, 12, 10, 7,
 5, 3, 2, 1, 0, -1, -2, -3, -2, -2, 0, 1, 2, 2, 3, 2, 1, -15, -14, -14, -12, -11, -9, -7, -4, -2, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -2,
 -5, -7, -9, -11, -13, -14, -15, -15, -15, -14, -13, -12, -10, -9, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 11, 9, 7, 5, 2, 1, 0, -2, -3, -2, -1, 1, 2,
 3, 2, 0, -1, -2, -3, -2, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4,
 -8, -12, -14, -15, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5,
 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 15, 14, 7, -3, -12, -15, -13, -4,
 6, 14, 15, 11, 1, -8, -15, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 0, -1, -1, -1, -2, -2, -2, -2,
 -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3,
 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -1, -1, -2, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7,
 -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 13, 15, 15,
 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, -1, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9,
 -8, -8, -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15,
 11, 1, -8, -15, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 8, 12,
 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7,
 -7, -7, -7, -7, -7, -6, -6, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 4, 4, 5, 6, 6, 6, 7, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 1, 2, 3,
 5, 6, 6, 7, 8, 9, 9, 9, 10, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -7, -8, -9, -9, -9, -10, -10, -10, -9, -9, -8, -8, -7, -6, -5, -4, -3,
 -2, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 3, 4,
 6, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, -1, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2,
 1, 1, 0, -1, -2, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 0, -4, -8, -12, -14, -15, -1,
 -2, -3, -4, -5, -6, -7, -7, -7, -7, -7, -6, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 6, 7, 7, 7, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -4, -5, -5, -6, -7, -7, -7, -7, -7,
 -6, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 6, 7, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, -2, -3, -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -8,
 -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -7, -8, -9, -10, -10, -10, -9, -8, -8, -6, -5, -4, -2, 0, 1, 3, 4, 6, 7,
 8, 9, 9, 10, 15, 14, 7, -3, -12, -15, -13, -4, 6, 14, 15, 11, 1, -8, -15, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4,
 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2,
 -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 0, 0, 0, -1, -1, -1, -1, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7,
 -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2,
 1, 1, 1, 1, 1, 0, -1, -1, -1, 0, 0, 1, 1, 1, 0, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 4, 4, 3, 3, 3,
 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 3, 3, 3, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 0, -1, -1, -1, -2, -2, 7, 7, 7, 6, 6, 6, 5, 4, 4, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -6, -6, -7, -7, -7, -7,
 -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 2, 2, 2, 1, 1, 0, -1, -2, -2, -2, -1, 0, 1, 1,
 2, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 0, -1, -2, -3, -4, -5, -6, -7, -8, -8, -9, -9, -10, -10, -10, -9, -9, -9, -8, -7, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6,
 7, 8, 8, 9, 9, 10, 10, 10, 9, 9, 9, 8, 7, 6, 6, 5, 3, 2, 1, 2, 2, 1, 0, -1, -2, -2, -1, 1, 2, 2, 1, 0, -1, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, -4, -4, -4, -4, -3, -3, -2, -1, -1, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1, 0, -1, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4,
 -4, -4, -3, -3, -2, -1, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 1, 1, 1, 2, 2, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -7, -7, -6, -6, -5, -4, -3, -2, -1, 0,
 2, 3, 4, 5, 6, 6, 7, 7, 7, 7, 7, 6, 5, 5, 4, 3, 1, 0, -1, -2, -3, -4, -5, -6, -6, -7, -7, -7, -7, -6, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 6, 7, 7, 7, 7, 7, 6, 5,
 4, 3, 2, 1, 2, 3, 2, 2, 1, 0, -2, -2, -3, -2, -1, 0, 1, 2, 3, -10, -9, -9, -8, -7, -6, -4, -3, -1, 0, 2, 4, 5, 6, 8, 8, 9, 10, 10, 10, 9, 8, 7, 6, 5, 3, 2, 0, -1, -3,
 -5, -6, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -4, -3, -1, 1, 2, 4, 5, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 6, 5, 3, 2, 3, 2, 1, 0, -2, -3, -2, -1, 1, 2, 3, 2, 0, -1,
 -2, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 7, 1, -5, -10, -11, -9, -3, 3, 9, 11, 10, 5, -1, -7, -11, -11, -11, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5,
 -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8,
 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11,
 -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4,
 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4,
 -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 13,
 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15,
 -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -11, -11, -11, -11, -10,
 -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6,
 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2,
 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8,
 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13,
 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15,
 -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4,
 -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4,
 -8, -10, -12, -14, -15, -16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4,
 -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14,
 -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13,
 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12,
 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -13, -14, -14, -14,
 -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3,
 -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16,
 14, 8, -2, -10, -15, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1,
 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4,
 -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4,
 -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -16, -15, -15,
 -15, -14, -14, -13, -13, -12, -11, -10, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 15,
 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16,
 -15, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 13,
 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, -15, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6,
 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8,
 -9, -10, -11, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2,
 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 15, 15, 10, 2, -8, -14, -16, -12, -4, 4, 12, 16, 14, 8, -2, -10, -15, 0,0,0,0,1,15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6,
 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6,
 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6,
 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5,
 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -15, -15, -15, -15, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2,
 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5,
 -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0,
 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -10,
 -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13,
 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7,
 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12,
 -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9,
 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12,
 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12,
 12, 11, 10, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15,
 -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13,
 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13,
 -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1,
 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4,
 3, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3,
 -4, -4, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -13, -12, -11,
 -11, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 0, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 11,
 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13,
 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 4, 4, 3, 2, 1, 1, 0, -1, -1, -1, 0, 0, 1, 1, 1, 0, 0, -1, -1,
 -1, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -13,
 -13, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6,
 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10,
 11, 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14,
 -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -13, -13, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 12, 13, 14, 14, 15, 15, 15, 15,
 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, -15, -15, -15, -15, -14, -14, -13, -12, -11, -11,
 -10, -8, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1,
 -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2,
 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 7, 6, 5, 4, 3, 1, 1, 1, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1,
 1, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 3, 4, 5, 6, 7, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14,
 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12,
 -12, -11, -10, -9, -8, -6, -5, -4, -3, -2, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4,
 2, 1, 1, 0, -1, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, -1, 0,0,1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3,
 -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2,
 -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1,
 -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1,
 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9,
 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2,
 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3,
 -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3,
 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0,
 -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -6, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4,
 -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3,
 2, 2, 1, 1, 0, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3,
 -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -8, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3,
 -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 3, 3, 2,
 2, 1, 1, 1, 1, 0, 0, -1, -1, -1, 0, 0, 1, 1, 1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2,
 -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1,
 -1, -7, -7, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 4, 4,
 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -5, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1,
 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 1, 1, 1, -10, -10, -9, -9,
 -9, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 1,
 0, 0, -1, -2, -3, -4, -4, -5, -6, -6, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 1, 2, 3, 4,
 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 1, 0, -1, -1, -1, 0, 1, 1, 1, 1, 0, -1, -1, 1,8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4,
 -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3,
 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11, -10, -8, -5, -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -10, -4, 5,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, -15, -6, 6, 15, 15, 14, 14, 13, 13, 12,
 11, 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -6, 6, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13,
 -14, -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, -15, -6, 6, 15, 13, 11, 7, 4, -1, -5, -9, -12, -14, -15, -14, -13, -10, -6, -2, 2, 6, 10, 13,
 14, 15, 14, 12, 9, 5, 1, -4, -7, -11, -13, -15, 8, -8, -8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, -11, -11,
 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, -11, -11, 11, 11, 10, 9, 7, 5, 3, 1, -1,
 -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, -11, -11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11, -10, -8, -5,
 -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -4, 10, -10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, -6, 15, -15, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -6, -6, -6, 15, -15, 6, 6, 6, 5, 4, 3, 2, 1, -1,
 -2, -3, -4, -5, -6, -6, -6, -6, -6, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 6, 6, -6, 15, -15, 6, 6, 5, 3, 2, 0, -2, -4, -5, -6, -6, -6, -5, -4, -3, -1, 1, 3, 4, 5, 6,
 6, 6, 5, 4, 2, 0, -2, -3, -5, -6, -6, 0,1,11, 9, 7, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 15, 13, 10, 5, 5, 5,
 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, 15, 13, 10, 5, 5, 5, 4, 3, 2, 1, 1, -1, -1, -2, -3, -4, -5, -5, -5,
 -5, -5, -5, -4, -3, -2, -1, -1, 1, 1, 2, 3, 4, 5, 5, 5, 15, 13, 10, 5, 5, 4, 3, 1, 0, -2, -3, -4, -5, -5, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 5, 5, 4, 3, 2, 0, -1, -3,
 -4, -5, -5, -9, 0, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, -13, 0, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8,
 7, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, 0, 13, 13, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -13,
 -13, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 13, -13, 0, 13, 13, 12, 10, 7, 3, -1, -4, -8, -11, -12, -13, -13, -11, -9, -5, -2, 2, 5, 9, 11, 13, 13, 12,
 11, 8, 4, 1, -3, -7, -10, -12, -13, 7, -9, -4, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 10, -13, -5, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, 10, -13, -5, 15,
 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14, -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13, 14, 15, 10, -13, -5, 15, 13, 11, 8, 4, -1, -5, -9,
 -12, -14, -15, -14, -13, -10, -6, -2, 2, 6, 10, 13, 14, 15, 14, 12, 9, 5, 1, -4, -8, -11, -13, -15, -4, 9, -10, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, -5, 13, -15, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10,
 -5, 13, -15, 10, 9, 9, 8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -9, -9, -10, -10, -9, -9, -8, -6, -5, -3, -1, 1, 3, 5, 6, 8, 9, 9, 10, -5, 13, -15, 10, 9, 7, 5, 2, 0, -3, -6,
 -8, -9, -10, -9, -8, -7, -4, -1, 1, 4, 7, 8, 9, 10, 9, 8, 6, 3, 0, -2, -5, -7, -9, -10, 0,1,5, 10, 13, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 11, 11,
 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, 5, 10, 13, 15, 14, 14, 13, 12, 10, 9, 7, 5, 3, 1, -1, -3, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -14, -13, -13,
 -11, -10, -8, -6, -4, -2, 5, 10, 13, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 5,
 9, 13, 14, 13, 10, 6, 1, -4, -9, -12, -14, -15, -13, -11, -7, -2, 3, 8, 11, 14, 15, 14, 12, 8, 3, -1, -6, -10, -13, -15, -14, -13, -9, -5, -13, -13, 0, 13, 13, 13, 13,
 13, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, -13, -13, 0, 13, 13, 12, 11, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -7, -8,
 -10, -11, -12, -12, -13, -13, -13, -12, -12, -11, -10, -9, -7, -5, -4, -2, -13, -13, 0, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3, 0,
 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, -13, -13, 0, 13, 11, 9, 5, 1, -4, -8, -11, -12, -13, -12, -10, -6, -2, 2, 7, 10, 12, 13, 12, 10, 7, 3, -1, -5, -9, -12, -13,
 -13, -11, -8, -4, 15, -5, -13, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 15, -5, -13, 10, 9, 9, 8, 8, 7, 6, 5,
 3, 2, 0, -1, -2, -4, -5, -6, -7, -8, -9, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -4, -3, -1, 15, -5, -13, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8,
 -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 14, -5, -13, 9, 8, 6, 4, 0, -3, -6, -8, -9, -10, -9, -7, -5, -1, 2, 5, 7, 9, 10, 9, 8, 5, 2, -1, -4, -7, -9, -10,
 -9, -8, -6, -3, -10, 15, -13, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, -10, 15, -13, 5, 5, 5, 4, 4, 4, 3, 2, 2, 1,
 0, -1, -1, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -1, -1, -10, 15, -13, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -5, -5, -4, -3, -2, -1,
 0, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1, -9, 14, -13, 5, 4, 3, 2, 0, -1, -3, -4, -5, -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -4, -3, -2, 1,
15, 13, 10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 15, 13, 10, 5, 5, 5, 4, 4, 4, 3, 2, 2, 1, 0, -1, -1, -2, -3,
 -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -1, -1, 15, 13, 10, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5,
 5, 5, 4, 3, 2, 1, 14, 13, 9, 5, 4, 3, 2, 0, -1, -3, -4, -5, -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -4, -3, -2, -13, 0, 13, 13, 13, 13,
 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, -13, 0, 13, 13, 13, 12, 11, 10, 9, 8, 6, 4, 2, 1, -1, -3, -5, -7, -8,
 -10, -11, -12, -12, -13, -13, -13, -12, -12, -11, -10, -9, -7, -5, -4, -2, -13, 0, 13, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3, 0,
 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, -13, 0, 13, 13, 11, 9, 5, 1, -4, -8, -11, -12, -13, -12, -10, -6, -2, 2, 7, 10, 12, 13, 12, 10, 7, 3, -1, -5, -9, -12, -13,
 -13, -11, -8, -4, 10, -13, -5, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, 10, -13, -5, 15, 14, 14,
 13, 12, 10, 9, 7, 5, 3, 1, -1, -3, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -14, -13, -13, -11, -10, -8, -6, -4, -2, 10, -13, -5, 15, 14, 12, 10, 7, 3, 0, -3, -7,
 -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 9, -13, -5, 14, 13, 10, 6, 1, -4, -9, -12, -14, -15, -13, -11, -7, -2, 3,
 8, 11, 14, 15, 14, 12, 8, 3, -1, -6, -10, -13, -15, -14, -13, -9, -5, -5, 13, -15, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2,
 2, 1, 1, 0, -5, 13, -15, 10, 9, 9, 8, 8, 7, 6, 5, 3, 2, 0, -1, -2, -4, -5, -6, -7, -8, -9, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -4, -3, -1, -5, 13, -15, 10, 9, 8,
 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, -5, 13, -14, 9, 8, 6, 4, 0, -3, -6, -8, -9, -10, -9, -7, -5, -1,
 2, 5, 7, 9, 10, 9, 8, 5, 2, -1, -4, -7, -9, -10, -9, -8, -6, -3, 0,1,0, 0, 1, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14,
 14, 14, 14, 15, 15, 15, 1, 1, 2, 2, 4, 6, 8, 10, 11, 13, 13, 14, 15, 15, 14, 14, 13, 12, 11, 9, 8, 6, 3, 1, -1, -3, -5, -7, -9, -10, -12, -13, -14, -14, -15, 1, 2, 3, 3,
 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 2, 3, 4, 5, 9, 13, 14, 15, 13, 10, 6, 1, -3,
 -8, -12, -14, -15, -14, -11, -8, -3, 2, 7, 11, 13, 15, 14, 12, 9, 4, -1, -6, -10, -13, -14, -1, -1, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11,
 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, -2, -2, 0, 2, 4, 5, 7, 9, 10, 11, 12, 12, 13, 13, 13, 12, 12, 11, 10, 8, 7, 5, 3, 1, -1, -2, -4, -6, -8, -9, -10, -11,
 -12, -13, -13, -3, -3, 0, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12, -11, -9, -6, -3, 0, 3, 6, 9, 11, 12, 13, -4, -4, 0, 4, 8, 11,
 13, 13, 12, 9, 5, 1, -3, -7, -10, -12, -13, -12, -10, -7, -2, 2, 6, 10, 12, 13, 12, 11, 8, 4, -1, -5, -9, -11, -13, 1, 0, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6,
 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 2, -1, -2, 1, 3, 4, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -5, -6, -7, -8, -8, -9,
 -9, -10, 3, -1, -3, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 5, -2, -4, 3, 6, 8, 9, 10, 9, 7, 4,
 1, -2, -5, -8, -9, -10, -9, -7, -5, -2, 1, 5, 7, 9, 10, 9, 8, 6, 3, 0, -4, -6, -8, -9, 0, 1, -1, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, -1, 2, -2, 1, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, -1, -2, -2, -3, -4, -4, -4, -5, -5, -5, -2, 3, -3, 1, 2, 3, 4, 5, 5,
 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, -3, 5, -4, 2, 3, 4, 5, 5, 5, 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2,
 4, 5, 5, 5, 4, 3, 1, 0, -2, -3, -4, -5, 1,1, 1, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 1, 1, 1, 2, 3,
 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, -1, -2, -2, -3, -4, -4, -4, -5, -5, -5, 3, 3, 2, 1, 2, 3, 4, 5, 5, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -5,
 -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 5, 5, 4, 3, 2, 3, 4, 5, 5, 5, 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2, 4, 5, 5, 5, 4, 3, 1, 0, -2, -3, -4, -5, -1, 0, 1,
 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, -2, 0, 2, 2, 4, 5, 7, 9, 10, 11, 12, 12, 13, 13, 13, 12,
 12, 11, 10, 8, 7, 5, 3, 1, -1, -2, -4, -6, -8, -9, -10, -11, -12, -13, -13, -3, 0, 3, 3, 6, 9, 11, 12, 13, 13, 12, 11, 9, 6, 3, 0, -3, -6, -9, -11, -12, -13, -13, -12,
 -11, -9, -6, -3, 0, 3, 6, 9, 11, 12, 13, -4, 0, 4, 4, 8, 11, 13, 13, 12, 9, 5, 1, -3, -7, -10, -12, -13, -12, -10, -7, -2, 2, 6, 10, 12, 13, 12, 11, 8, 4, -1, -5, -9,
 -11, -13, 0, -1, 0, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 1, -2, -1, 2, 4, 6, 8, 10, 11, 13,
 13, 14, 15, 15, 14, 14, 13, 12, 11, 9, 8, 6, 3, 1, -1, -3, -5, -7, -9, -10, -12, -13, -14, -14, -15, 2, -3, -1, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7,
 -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 3, -4, -2, 5, 9, 13, 14, 15, 13, 10, 6, 1, -3, -8, -12, -14, -15, -14, -11, -8, -3, 2, 7, 11,
 13, 15, 14, 12, 9, 4, -1, -6, -10, -13, -14, 0, 1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, -1, 2, -2, 1,
 3, 4, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -5, -6, -7, -8, -8, -9, -9, -10, -1, 3, -3, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6,
 -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, -2, 4, -5, 3, 6, 8, 9, 10, 9, 7, 4, 1, -2, -5, -8, -9, -10, -9, -7, -5, -2, 1, 5, 7, 9, 10, 9, 8, 6, 3, 0,
 -4, -6, -8, -9, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5,
 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11,
 -11, -11, -10, -8, -5, -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -13, -9, -3, 3, 9, 13, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4,
 -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -16, -16, -15, -14,
 -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, -16, -13, -9, -3, 3, 9, 13, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15, 16,
 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 10, 5, -4, -10, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 15, 6, -6, -15, -15, -6, 6, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -13, -13,
 -14, -14, -15, -15, 15, 6, -6, -15, -15, -6, 6, 15, 14, 13, 11, 9, 7, 4, 1, -1, -4, -7, -9, -11, -13, -14, -15, -15, -14, -13, -11, -9, -7, -4, -1, 1, 4, 7, 9, 11, 13,
 14, 15, 15, 6, -6, -15, -15, -6, 6, 15, 13, 11, 7, 4, -1, -5, -9, -12, -14, -15, -14, -13, -10, -6, -2, 2, 6, 10, 13, 14, 15, 14, 12, 9, 5, 1, -4, -7, -11, -13, -15, -9,
 2, 11, 6, -6, -11, -2, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 12, 12,
 11, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, 3, 16, 9, -9, -16, -3, 13, 13, 12, 10, 8, 6, 4, 1, -1,
 -4, -6, -8, -10, -12, -13, -13, -13, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 13, -13, 3, 16, 9, -9, -16, -3, 13, 12, 10, 7, 3, -1, -5, -8, -11, -12, -13,
 -13, -11, -9, -6, -2, 2, 6, 9, 11, 13, 13, 12, 11, 8, 5, 1, -3, -7, -10, -12, -13, 0,1,11, 11, 10, 9, 8, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3,
 -3, -3, 15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 15, 15, 14, 12, 11, 8, 6,
 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -11, -7, -2, 4, 9, 11, 10, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, -15, -11, -3, 6, 12, 15, 14, 8, 8, 8, 8, 7, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5,
 -5, -6, -7, -7, -7, -8, -8, -8, -8, -15, -11, -3, 6, 12, 15, 14, 8, 8, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -8, -8, -8, -8, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6,
 7, 8, 8, -15, -11, -3, 6, 12, 15, 14, 8, 7, 6, 4, 2, 0, -3, -5, -7, -8, -8, -8, -7, -5, -3, -1, 1, 3, 5, 7, 8, 8, 8, 7, 5, 3, 0, -2, -4, -6, -7, -8, 10, 2, -7, -11, -6,
 4, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 3, -11, -15, -8, 6, 15, 12, 12, 12, 12, 11, 11, 10, 9, 8, 7,
 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -12, -12, 14, 3, -11, -15, -8, 6, 15, 12, 12, 11, 10, 8, 6, 3, 1, -1, -3, -6, -8, -10,
 -11, -12, -12, -12, -12, -11, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 11, 12, 12, 14, 3, -11, -15, -8, 6, 15, 12, 11, 9, 6, 3, -1, -4, -7, -10, -12, -12, -12, -11, -8, -5,
 -2, 2, 5, 8, 11, 12, 12, 12, 10, 7, 4, 1, -3, -6, -9, -11, -12, -9, 4, 11, 2, -10, -7, 6, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -12, 6, 15, 3, -14, -11, 8, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -8, -9, -10,
 -11, -12, -13, -14, -14, -15, -15, -15, -12, 6, 15, 3, -14, -11, 8, 15, 14, 13, 12, 9, 7, 4, 1, -1, -4, -7, -9, -12, -13, -14, -15, -15, -14, -13, -12, -9, -7, -4, -1,
 1, 4, 7, 9, 12, 13, 14, 15, -12, 6, 15, 3, -14, -11, 8, 15, 14, 11, 8, 4, -1, -5, -9, -12, -14, -15, -15, -13, -10, -6, -2, 2, 6, 10, 13, 15, 15, 14, 12, 9, 5, 1, -4,
 -8, -11, -14, -15, 0,1,3, 6, 8, 11, 12, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 3, 6,
 8, 11, 12, 14, 15, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -14, -14, -13, -11, -10, -8, -6, -4, -2, 3, 6, 8, 10,
 12, 14, 15, 15, 14, 12, 10, 7, 4, 0, -4, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -4, 0, 4, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 4, 3, 6, 8, 10, 12, 13, 15, 15,
 13, 10, 6, 1, -4, -9, -12, -14, -15, -14, -11, -7, -2, 3, 8, 11, 14, 15, 14, 12, 8, 4, -2, -6, -11, -13, -15, -15, -13, -9, -5, -8, -14, -15, -12, -6, 3, 11, 15, 15, 15,
 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, -8, -14, -15, -12, -6, 3, 11, 15, 14, 14, 13, 12, 10, 9, 7, 5, 3,
 1, -1, -3, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -14, -14, -13, -11, -10, -8, -6, -4, -2, -8, -14, -15, -12, -6, 3, 10, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10,
 -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, -8, -13, -15, -12, -5, 3, 10, 15, 13, 10, 6, 1, -4, -9, -12, -14, -15, -14,
 -11, -7, -2, 3, 8, 11, 14, 15, 14, 12, 8, 3, -1, -6, -10, -13, -15, -14, -13, -9, -5, 12, 15, 6, -8, -15, -11, 3, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11,
 10, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 12, 15, 6, -8, -15, -11, 3, 14, 13, 13, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -12, -13, -13,
 -14, -14, -13, -12, -12, -10, -9, -8, -6, -4, -2, 12, 15, 6, -8, -15, -10, 3, 14, 13, 11, 9, 6, 3, 0, -3, -6, -9, -11, -13, -14, -14, -13, -11, -9, -6, -3, 0, 3, 6, 9,
 11, 13, 14, 14, 13, 11, 9, 6, 3, 12, 15, 6, -8, -15, -10, 3, 13, 12, 9, 5, 1, -4, -8, -11, -13, -14, -12, -10, -6, -2, 3, 7, 10, 13, 14, 13, 11, 8, 3, -1, -6, -10, -12,
 -14, -13, -12, -9, -5, -15, -8, 11, 14, -3, -15, -6, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, -15,
 -8, 11, 14, -3, -15, -6, 12, 12, 12, 11, 10, 9, 7, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -10, -11, -12, -12, -12, -12, -12, -11, -11, -9, -8, -7, -5, -4, -2, -15, -8, 10,
 14, -3, -15, -6, 12, 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 3, -15, -8, 10, 13, -3, -15, -5,
 12, 11, 8, 5, 1, -4, -7, -10, -12, -12, -11, -9, -6, -2, 2, 6, 9, 12, 12, 12, 10, 7, 3, -1, -5, -9, -11, -12, -12, -11, -8, -4, 1,15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -2,
 -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -1, -1, 0, 15, 15, 14, 12, 10, 8, 6, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3,
 3, 3, 3, 2, 2, 1, 1, 15, 15, 13, 12, 10, 8, 6, 3, 3, 2, 1, 0, -1, -2, -2, -3, -3, -3, -2, -1, 0, 1, 2, 2, 3, 3, 3, 2, 2, 1, 0, -1, -2, -3, -3, -3, -3, -2, -1, -15, -11,
 -3, 6, 12, 15, 14, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, -15, -11, -3, 6, 12, 15, 14, 8, 8, 8, 7, 6, 6, 5, 4,
 3, 2, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -8, -8, -8, -8, -7, -7, -6, -5, -4, -3, -2, -1, -15, -10, -3, 6, 12, 15, 14, 8, 8, 7, 5, 4, 2, 0, -2, -4, -5, -7, -8, -8,
 -8, -8, -7, -5, -4, -2, 0, 2, 4, 5, 7, 8, 8, 8, 8, 7, 5, 4, 2, -15, -10, -3, 6, 12, 15, 13, 8, 7, 5, 3, 0, -2, -5, -7, -8, -8, -7, -6, -4, -1, 2, 4, 6, 8, 8, 8, 6, 4, 2,
 -1, -3, -6, -7, -8, -8, -7, -5, -3, 14, 3, -11, -15, -8, 6, 15, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2,
 1, 1, 14, 3, -11, -15, -8, 6, 15, 12, 12, 12, 11, 10, 9, 7, 6, 4, 2, 1, -1, -3, -5, -6, -8, -9, -10, -11, -12, -12, -12, -12, -12, -11, -11, -9, -8, -7, -5, -4, -2, 14,
 3, -10, -15, -8, 6, 15, 12, 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 3, 13, 3, -10, -15, -8,
 6, 15, 12, 11, 8, 5, 1, -4, -7, -10, -12, -12, -11, -9, -6, -2, 2, 6, 9, 12, 12, 12, 10, 7, 3, -1, -5, -9, -11, -12, -12, -11, -8, -4, -12, 6, 15, 3, -14, -11, 8, 15,
 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 1, -12, 6, 15, 3, -14, -11, 8, 15, 14, 14, 13, 12, 10, 9, 7,
 5, 3, 1, -1, -3, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -14, -14, -13, -11, -10, -8, -6, -4, -2, -12, 6, 15, 3, -14, -10, 8, 15, 14, 12, 10, 7, 3, 0, -3, -7,
 -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, -12, 6, 15, 3, -13, -10, 8, 15, 13, 10, 6, 1, -4, -9, -12, -14, -15, -14,
 -11, -7, -2, 3, 8, 11, 14, 15, 14, 12, 8, 3, -1, -6, -10, -13, -15, -14, -13, -9, -5, 0,1,0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 11, 11, 11,
 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 0, 1, 1, 2, 2, 2, 2, 2, 4, 6, 8, 10, 11, 13, 14, 14, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 4, 2, -1, -3, -5, -7,
 -9, -11, -12, -13, -14, -15, -15, 1, 1, 2, 2, 3, 3, 3, 4, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 4, 0, -4, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -4, 0, 4, 7,
 10, 12, 14, 15, 1, 2, 3, 4, 4, 5, 5, 5, 9, 13, 15, 15, 13, 11, 6, 2, -4, -8, -12, -14, -15, -14, -11, -8, -3, 2, 7, 11, 14, 15, 14, 12, 9, 4, -1, -6, -10, -13, -15, 0,
 -1, -1, -1, 0, 0, 0, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, -1, -2, -2, -2, -1, 0, 2, 2, 4, 6,
 8, 10, 11, 13, 14, 14, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 3, 1, -1, -3, -5, -7, -9, -10, -12, -13, -14, -14, -15, -2, -3, -4, -3, -1, 1, 2, 3, 7, 10, 12, 14, 15, 15,
 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, -3, -5, -5, -4, -2, 1, 4, 5, 9, 13, 14, 15, 13, 10, 6, 1, -3, -8,
 -12, -14, -15, -14, -11, -8, -3, 2, 7, 11, 14, 15, 14, 12, 9, 4, -1, -6, -10, -13, -15, 1, 1, 0, 0, -1, 0, 0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 10,
 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 2, 2, 1, -1, -2, -2, 0, 2, 4, 6, 8, 9, 10, 12, 12, 13, 14, 14, 13, 13, 12, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -6,
 -8, -10, -11, -12, -13, -13, -14, 3, 3, 1, -2, -4, -2, 1, 3, 6, 9, 11, 13, 14, 14, 13, 11, 9, 6, 3, 0, -3, -6, -9, -11, -13, -14, -14, -13, -11, -9, -6, -3, 0, 3, 6, 9,
 11, 13, 14, 4, 5, 2, -3, -5, -4, 1, 5, 9, 12, 13, 14, 12, 10, 6, 1, -3, -8, -11, -13, -14, -13, -10, -7, -3, 2, 6, 10, 12, 14, 13, 11, 8, 4, -1, -5, -9, -12, -13, -1, 0,
 0, 1, 0, -1, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, -2, -1, 2, 2, 0, -2, -1, 2, 4, 5, 7, 8, 9,
 11, 11, 12, 12, 12, 12, 12, 11, 10, 9, 8, 6, 5, 3, 1, -1, -2, -4, -6, -7, -9, -10, -11, -12, -12, -12, -3, -2, 2, 3, -1, -4, -1, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6,
 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, -5, -3, 4, 5, -1, -5, -2, 4, 8, 11, 12, 12, 11, 9, 5, 1, -3, -7, -10, -12, -12, -12,
 -9, -6, -2, 2, 6, 9, 11, 12, 12, 10, 7, 4, -1, -5, -8, -11, -12, 1,1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, 4, 3, 3, 3, 2, 2, 1, 1, 1,
 2, 2, 3, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 5, 5, 5, 4, 4, 3, 2, 1, 2, 3, 3, 3, 3, 2, 1, 0, -1, -2, -2, -3, -3,
 -3, -2, -2, -1, 0, 1, 2, 3, 3, 3, 2, 2, 1, 0, -1, -2, -3, -3, -1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8,
 8, 8, 8, 8, -2, -2, 0, 1, 2, 2, 2, 1, 2, 3, 4, 5, 6, 7, 7, 8, 8, 8, 8, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -6, -7, -8, -8, -8, -3, -2, -1, 1, 3, 4, 3, 2,
 4, 5, 7, 8, 8, 8, 8, 7, 5, 4, 2, 0, -2, -4, -5, -7, -8, -8, -8, -8, -7, -5, -4, -2, 0, 2, 4, 5, 7, 8, 8, -5, -4, -1, 2, 4, 5, 5, 3, 5, 7, 8, 8, 7, 6, 3, 1, -2, -4, -6,
 -8, -8, -8, -6, -4, -2, 1, 4, 6, 7, 8, 8, 7, 5, 2, 0, -3, -5, -7, -8, 1, 0, 0, -1, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 11, 11,
 12, 12, 12, 12, 12, 12, 12, 12, 2, 0, -2, -2, -1, 1, 2, 2, 4, 5, 7, 8, 9, 11, 11, 12, 12, 12, 12, 12, 11, 10, 9, 8, 6, 5, 3, 1, -1, -2, -4, -6, -7, -9, -10, -11, -12,
 -12, -12, 3, 1, -2, -4, -2, 1, 3, 3, 6, 8, 10, 12, 12, 12, 12, 10, 8, 6, 3, 0, -3, -6, -8, -10, -12, -12, -12, -12, -10, -8, -6, -3, 0, 3, 6, 8, 10, 12, 12, 5, 1, -4,
 -5, -3, 2, 5, 4, 8, 11, 12, 12, 11, 9, 5, 1, -3, -7, -10, -12, -12, -12, -9, -6, -2, 2, 6, 9, 11, 12, 12, 10, 7, 4, -1, -5, -8, -11, -12, -1, 0, 1, 0, -1, 0, 0, 1, 1, 2,
 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, -2, 1, 2, 0, -2, -2, 1, 2, 4, 6, 8, 10, 11, 13, 14, 14, 15, 15,
 15, 14, 13, 12, 11, 9, 8, 6, 3, 1, -1, -3, -5, -7, -9, -10, -12, -13, -14, -14, -15, -3, 1, 4, 1, -3, -2, 2, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10,
 -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, -4, 2, 5, 1, -5, -4, 3, 5, 9, 13, 14, 15, 13, 10, 6, 1, -3, -8, -12, -14, -15, -14, -11, -8, -3, 2,
 7, 11, 14, 15, 14, 12, 9, 4, -1, -6, -10, -13, -15, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7,
 -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11,
 -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11, -10,
 -8, -5, -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 15, 15, 14,
 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14,
 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -15, -14, -12, -10, -8, -4,
 -2, 2, 4, 8, 10, 12, 14, 15, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 11, 9, 6,
 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14,
 -14, -15, -15, -16, -16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -16, -16, -15, -14,
 -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14,
 -11, -7, -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -15, -10, -2, 8, 14, 16, 12, 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 14,
 14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -15, -10, -2, 8, 14, 16, 12, 4, -4, -12, -16, -14, -8,
 2, 10, 15, 15, 14, 12, 10, 7, 4, 2, -2, -4, -7, -10, -12, -14, -15, -15, -15, -15, -14, -12, -10, -7, -4, -2, 2, 4, 7, 10, 12, 14, 15, 15, -15, -10, -2, 8, 14, 16, 12,
 4, -4, -12, -16, -14, -8, 2, 10, 15, 14, 11, 8, 4, -1, -5, -9, -12, -14, -15, -15, -13, -10, -6, -2, 2, 6, 10, 13, 15, 15, 14, 12, 9, 5, 1, -4, -8, -11, -14, -15, 0,1,
11, 11, 11, 11, 10, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 15, 15, 15, 15, 14, 14,
 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 15, 15, 15, 15, 14, 14, 13,
 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 15, 15, 15, 15, 14, 14, 13, 12,
 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -11, -10, -8, -6, -3, 0, 3, 6, 9, 10,
 11, 11, 10, 9, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12,
 8, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12,
 8, 4, 4, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 4, 4, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12,
 8, 4, 4, 3, 2, 1, 0, -2, -3, -4, -4, -4, -4, -4, -3, -2, -1, 1, 2, 3, 4, 4, 4, 4, 4, 3, 2, 0, -1, -2, -3, -4, -4, 11, 9, 4, -1, -6, -10, -11, -10, -7, -2, 3, 8, 11, 11,
 9, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 7,
 7, 6, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -7, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7,
 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -7, -7, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 7, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13,
 7, 6, 5, 4, 2, 0, -2, -4, -6, -7, -7, -7, -6, -5, -3, -1, 1, 3, 5, 6, 7, 7, 7, 6, 4, 2, 0, -2, -4, -5, -6, -7, -11, -6, 1, 8, 11, 10, 4, -3, -9, -11, -8, -2, 5, 10, 11,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 10, 9,
 9, 9, 8, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10,
 9, 9, 8, 6, 5, 3, 1, -1, -3, -5, -6, -8, -9, -9, -10, -10, -9, -9, -8, -6, -5, -3, -1, 1, 3, 5, 6, 8, 9, 9, 10, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14,
 15, 10, 9, 7, 5, 2, 0, -3, -6, -8, -9, -10, -9, -8, -7, -4, -1, 1, 4, 7, 8, 9, 10, 9, 8, 6, 3, 0, -2, -5, -7, -9, -10, 0,1,1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 14, 15,
 15, 15, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11, -13, -14, -15, -15, -15, -15, -15, -14, -13, -12, -10, -9, -7, -4, -2, 1, 3, 4, 6, 7, 8, 10, 11,
 12, 13, 13, 14, 15, 15, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 1, 3, 4, 6,
 7, 8, 9, 11, 12, 13, 13, 14, 15, 15, 15, 15, 13, 10, 6, 1, -4, -9, -13, -15, -15, -14, -11, -7, -2, 3, 8, 12, 14, 15, 15, 12, 9, 4, -2, -7, -11, -14, -15, -15, -13, -10,
 -5, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4,
 3, 2, 2, 1, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11, -13, -14, -15, -15, -15, -15,
 -15, -14, -13, -12, -10, -9, -7, -4, -2, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14,
 -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, -4, -8, -12, -14, -15, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 13, 10, 6, 1, -4, -9, -13, -15, -15,
 -14, -11, -7, -2, 3, 8, 12, 14, 15, 15, 12, 9, 4, -2, -7, -11, -14, -15, -15, -13, -10, -5, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 2, 2, 1, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15, -14, -8, -1, 6, 12, 15, 15,
 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11, -13, -14, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -4, -2, 7, 13, 15, 15, 11, 4, -3, -10, -14, -15,
 -13, -8, -1, 6, 12, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 7, 13, 15, 15, 11, 4,
 -3, -9, -14, -15, -13, -8, -1, 6, 12, 15, 13, 10, 6, 1, -4, -9, -13, -15, -15, -14, -11, -7, -2, 3, 8, 12, 14, 15, 14, 12, 8, 4, -2, -6, -11, -14, -15, -15, -13, -10,
 -5, -10, -15, -14, -7, 3, 12, 15, 13, 4, -6, -14, -15, -11, -1, 8, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3,
 3, 2, 1, 1, -10, -15, -14, -7, 3, 12, 15, 13, 4, -6, -14, -15, -11, -1, 8, 15, 14, 14, 13, 12, 10, 9, 7, 5, 3, 1, -1, -3, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15,
 -14, -14, -13, -11, -10, -8, -6, -4, -2, -10, -15, -14, -7, 3, 12, 15, 13, 4, -6, -13, -15, -11, -1, 8, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14,
 -12, -10, -7, -3, 0, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, -9, -15, -14, -7, 3, 12, 15, 13, 4, -6, -13, -15, -11, -1, 8, 15, 13, 10, 6, 1, -4, -9, -12, -14, -15,
 -14, -11, -7, -2, 3, 8, 11, 14, 15, 14, 12, 8, 3, -1, -6, -10, -13, -15, -14, -13, -9, -5, 1,15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 15, 15, 15, 15, 14, 13, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 15, 15, 15, 15, 14, 13, 13, 12, 11, 9, 8, 7, 6, 4, 3, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0,
 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3,
 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4,
 -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, -1, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -4, -4, -4,
 -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 4, 4, 4, 4, 3, 2, 1, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 3, 2, 0, -1, -3, -4, -4, -4, -4, -3, -2, -1, 1,
 2, 3, 4, 4, 4, 4, 2, 1, 0, -2, -3, -4, -4, -4, -4, -3, -1, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5,
 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5,
 -6, -6, -7, -7, -7, -7, -7, -6, -6, -5, -5, -4, -3, -2, -1, 15, 12, 6, -1, -8, -13, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 6, 5, 3, 2, 0, -2, -3, -5, -6, -7, -7,
 -7, -7, -6, -5, -3, -2, 0, 2, 3, 5, 6, 7, 7, 7, 7, 6, 5, 3, 2, 15, 12, 6, -1, -8, -13, -15, -14, -9, -3, 4, 11, 15, 15, 13, 7, 6, 5, 3, 0, -2, -4, -6, -7, -7, -6, -5,
 -3, -1, 1, 4, 5, 7, 7, 7, 6, 4, 2, -1, -3, -5, -6, -7, -7, -6, -4, -2, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8,
 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 9, 9, 8, 8, 7, 6, 5, 3, 2, 0, -1, -2, -4,
 -5, -6, -7, -8, -9, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -4, -3, -1, -15, -8, 1, 11, 15, 13, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6,
 -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, -15, -8, 1, 11, 15, 13, 6, -4, -13, -15, -12, -3, 7, 14, 15, 9, 8, 6, 4, 0, -3, -6, -8,
 -9, -10, -9, -7, -5, -1, 2, 5, 7, 9, 10, 9, 8, 5, 2, -1, -4, -7, -9, -10, -9, -8, -6, -3, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7,
 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 7, 9, 10, 12, 13, 14, 15, 15, 15,
 15, 15, 14, 13, 11, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10,
 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 10, 13, 15, 15, 14, 11, 7, 2,
 -4, -9, -12, -15, -15, -14, -12, -8, -3, 2, 7, 11, 14, 15, 15, 13, 9, 4, -1, -6, -10, -13, -15, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5,
 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, -1, -1, -2, -2, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 4, 7, 9, 10, 12,
 13, 14, 15, 15, 15, 15, 15, 14, 13, 11, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, -1, -2, -3, -3, -4, -4, -3, -3, -2, -1, 0, 1, 2, 3, 3, 4, 7,
 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, -1, -3, -4, -5, -5, -5, -5, -4, -3, -1, 0, 1,
 3, 4, 5, 5, 10, 13, 15, 15, 14, 11, 7, 2, -4, -9, -12, -15, -15, -14, -12, -8, -3, 2, 7, 11, 14, 15, 15, 13, 9, 4, -1, -6, -10, -13, -15, 0, 1, 1, 1, 0, 0, 0, 0, -1, -1,
 -1, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 1, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2,
 -1, 0, 1, 2, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 14, 13, 11, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, 2, 3, 4, 3, 3, 1, -1, -2, -3,
 -4, -3, -2, 0, 1, 3, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 2, 4, 5, 5, 4, 1,
 -1, -3, -5, -5, -5, -3, 0, 2, 4, 5, 10, 13, 15, 15, 14, 11, 6, 2, -4, -8, -12, -14, -15, -14, -12, -8, -3, 2, 7, 11, 14, 15, 15, 13, 9, 4, -1, -6, -10, -13, -15, 0, -1,
 -1, 0, 0, 1, 1, 1, 0, 0, -1, -1, 0, 0, 0, 1, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, -1, -2, -2,
 -1, 0, 2, 2, 2, 1, -1, -2, -2, -2, 0, 1, 2, 4, 6, 8, 10, 11, 13, 14, 14, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 3, 1, -1, -3, -5, -7, -9, -10, -12, -13, -14, -14, -15, -2,
 -4, -3, -2, 1, 3, 4, 3, 1, -1, -3, -4, -3, 0, 2, 3, 7, 10, 12, 14, 15, 15, 14, 12, 10, 7, 3, 0, -3, -7, -10, -12, -14, -15, -15, -14, -12, -10, -7, -3, 0, 3, 7, 10, 12,
 14, 15, -3, -5, -5, -2, 1, 4, 5, 4, 1, -2, -5, -5, -4, 0, 3, 5, 9, 13, 14, 15, 13, 10, 6, 1, -3, -8, -12, -14, -15, -14, -11, -8, -3, 2, 7, 11, 14, 15, 14, 12, 9, 4, -1,
 -6, -10, -13, -15, 1,1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 4, 4, 4, 3, 3, 3, 3, 3, 3,
 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1,
 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1,
 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4, -3, -3, -2, -1, 0, 1, 2, 3, 3, 4, 4, 3, 3, 2, 1, 2, 3, 4, 4, 4, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3,
 -4, -4, -4, -4, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 4, -5, -5, -4, -3, -1, 0, 1, 3, 4, 5, 5, 5, 5, 4, 3, 1, 3, 4, 4, 4, 4, 3, 2, 0, -1, -2, -4, -4, -4, -4, -3, -2, -1,
 1, 2, 3, 4, 4, 4, 4, 3, 1, 0, -2, -3, -4, -4, 1, 1, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7,
 7, 7, 7, 7, 7, 7, 7, 2, 2, 1, 0, -1, -2, -2, -2, -1, 0, 1, 2, 2, 2, 2, 1, 2, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 6, 6, 5, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -6, -7,
 -7, -7, 4, 3, 1, 0, -2, -3, -4, -3, -2, -1, 1, 3, 3, 4, 3, 2, 3, 5, 6, 7, 7, 7, 7, 6, 5, 3, 2, 0, -2, -3, -5, -6, -7, -7, -7, -7, -6, -5, -3, -2, 0, 2, 3, 5, 6, 7, 7, 5,
 4, 2, 0, -3, -5, -5, -5, -3, -1, 1, 4, 5, 5, 4, 2, 4, 6, 7, 7, 6, 5, 3, 1, -2, -4, -6, -7, -7, -7, -5, -4, -1, 1, 3, 5, 6, 7, 7, 6, 4, 2, 0, -3, -5, -6, -7, -1, 0, 0, 0,
 1, 1, 0, 0, -1, -1, -1, 0, 0, 1, 1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, -2, -1, 0, 2, 2, 2, 1, -1, -2, -2,
 -2, 0, 1, 2, 2, 1, 3, 4, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -5, -6, -7, -8, -8, -9, -9, -10, -3, -2, 0, 3, 4, 3, 1, -1, -3, -4, -3, -1,
 2, 3, 4, 2, 5, 6, 8, 9, 10, 10, 9, 8, 6, 5, 2, 0, -2, -5, -6, -8, -9, -10, -10, -9, -8, -6, -5, -2, 0, 2, 5, 6, 8, 9, 10, -5, -3, 0, 4, 5, 5, 2, -1, -4, -5, -4, -1, 2,
 5, 5, 3, 6, 8, 9, 10, 9, 7, 4, 1, -2, -5, -8, -9, -10, -9, -7, -5, -2, 1, 5, 7, 9, 10, 9, 8, 6, 3, 0, -4, -6, -8, -9, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5,
 -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11, -10, -8, -5, -2, 2, 5, 8, 10, 11, 11,
 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8,
 -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12,
 -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 15,
 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -16, -15, -15, -14, -14, -13, -12, -11,
 -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15,
 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10,
 -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9,
 -11, -12, -13, -14, -14, -15, -15, -16, -16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14,
 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2,
 -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7,
 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5, -8, -10, -11, -11, -11, -9, -7, -4, 0, 3, 6,
 8, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -14, -12, -8, -4, 1, 5, 9,
 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5,
 -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1,
 4, 8, 12, 14, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -14, -12, -8, -4,
 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7,
 -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 0,0,0,1,1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15,
 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9,
 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 15, 15, 14, 13, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -12, -13, -14, -15, -15, -16, -16,
 -15, -14, -14, -12, -11, -9, -7, -5, -2, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 15, 13, 11,
 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11,
 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 11, 6, 1, -5, -9, -13, -15, -16, -14, -12, -7, -2, 3, 8, 12, 15, 16, 15, 13, 9, 4, -2, -7, -11, -14, -16,
 -15, -14, -10, -5, -2, -5, -7, -9, -11, -12, -14, -14, -15, -16, -16, -15, -15, -14, -13, -12, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 13, 14, 15, 15, 16, 16, 16, 15,
 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, -2, -5, -7, -9, -11, -12, -14, -14, -15, -16, -16, -15, -15, -14, -13,
 -12, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 13, 14, 15, 15, 16, 15, 15, 14, 13, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -12, -13, -14, -15, -15, -16, -16, -15, -14,
 -14, -12, -11, -9, -7, -5, -2, -2, -5, -7, -9, -10, -12, -13, -14, -15, -15, -16, -15, -15, -14, -13, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 13, 14, 15, 15, 16,
 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, -2, -4, -7, -9, -10, -12, -13, -14, -15, -15,
 -15, -15, -15, -14, -13, -11, -10, -8, -6, -4, -2, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 14, 11, 6, 1, -5, -9, -13, -15, -16, -14, -12, -7, -2, 3, 8, 12, 15, 16, 15,
 13, 9, 4, -2, -7, -11, -14, -16, -15, -14, -10, -5, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11,
 13, 15, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 0,
 -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 15, 15, 14, 13, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11, -13, -14, -15, -15, -16,
 -15, -15, -14, -13, -12, -10, -9, -7, -5, -2, 4, 7, 10, 13, 15, 15, 15, 15, 13, 10, 7, 4, 0, -4, -7, -10, -13, -15, -15, -15, -15, -13, -10, -7, -4, 0, 4, 7, 10, 13, 15,
 15, 15, 13, 10, 7, 4, 0, -4, -7, -10, -13, -15, -15, -15, -15, -13, -10, -7, -4, 0, 4, 7, 10, 13, 15, 15, 15, 15, 13, 10, 7, 4, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7,
 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 14, 10, 6, 1, -5, -9, -13, -15, -16, -14, -11, -7, -2, 3, 8, 12, 15, 16, 15, 13,
 9, 4, -2, -7, -11, -14, -15, -15, -13, -10, -5, -5, -10, -14, -15, -16, -14, -11, -7, -2, 4, 9, 13, 15, 16, 15, 12, 8, 3, -2, -7, -12, -14, -16, -15, -13, -9, -5, 1, 6,
 11, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, -5, -10, -14, -15, -16, -14, -11, -7, -2, 4,
 9, 13, 15, 16, 15, 12, 8, 3, -2, -7, -12, -14, -16, -15, -13, -9, -5, 1, 6, 11, 14, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11, -13, -14, -15, -15,
 -15, -15, -15, -14, -13, -12, -10, -9, -7, -4, -2, -5, -10, -13, -15, -15, -14, -11, -7, -2, 4, 9, 13, 15, 16, 15, 12, 8, 3, -2, -7, -11, -14, -16, -15, -13, -9, -5, 1,
 6, 10, 14, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, -5, -10, -13, -15, -15, -14,
 -11, -7, -2, 4, 9, 12, 15, 15, 14, 12, 8, 3, -2, -7, -11, -14, -15, -15, -13, -9, -4, 1, 6, 10, 13, 15, 13, 10, 6, 1, -4, -9, -13, -15, -15, -14, -11, -7, -2, 3, 8, 12,
 14, 15, 15, 12, 9, 4, -2, -7, -11, -14, -15, -15, -13, -10, -5, 1,16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4,
 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12,
 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 16, 16, 15,
 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0,
 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16,
 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -16, -15, -15, -14, -13, -11, -9, -7,
 -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16, 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -1, -1, -1, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 11, 13, 14, 15, 15, 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 2, 2, 2, 2, 1,
 1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 11, 13,
 14, 15, 15, 15, 15, 15, 14, 13, 12, 10, 9, 7, 4, 2, 2, 2, 1, 0, -1, -1, -2, -2, -2, -2, -2, -1, 0, 0, 1, 2, 2, 2, 2, 2, 1, 1, 0, -1, -2, -2, -2, -2, -2, -1, -1, 16, 15,
 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7,
 4, 4, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 15, 15, 13, 10, 7, 4, 0, -4, -7, -10, -13, -15, -15,
 -15, -15, -13, -10, -7, -4, 0, 4, 7, 10, 13, 15, 15, 15, 15, 13, 10, 7, 4, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -4, -4, -3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 4, 4, 3,
 3, 2, 2, 1, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15, -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 3, 2, 1, 0, -1, -2, -3, -4,
 -4, -3, -3, -2, -1, 1, 2, 3, 3, 4, 3, 3, 2, 1, 0, -2, -3, -3, -4, -4, -3, -2, -1, -15, -14, -11, -6, -1, 5, 9, 13, 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13,
 -9, -4, 2, 7, 11, 14, 16, 15, 14, 10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, -15, -14, -11, -6, -1, 5, 9, 13,
 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 16, 15, 14, 10, 5, 5, 5, 5, 4, 4, 3, 2, 2, 1, 0, -1, -1, -2, -3, -3, -4, -4, -5, -5, -5, -5,
 -5, -5, -5, -5, -4, -4, -3, -2, -2, -1, -15, -14, -10, -6, -1, 5, 9, 13, 15, 16, 14, 11, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 15, 15, 13, 10, 5,
 5, 4, 4, 2, 1, 0, -1, -2, -4, -4, -5, -5, -5, -5, -4, -4, -2, -1, 0, 1, 2, 4, 4, 5, 5, 5, 5, 4, 4, 2, 1, -15, -13, -10, -6, -1, 4, 9, 13, 15, 15, 14, 11, 7, 2, -3, -8,
 -12, -14, -15, -15, -12, -9, -4, 2, 7, 11, 14, 15, 15, 13, 10, 5, 5, 4, 2, 0, -2, -3, -4, -5, -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -5,
 -3, -2, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13,
 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 7, 9, 11, 12, 14,
 14, 15, 16, 16, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 0, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 14, 15, 16, 14, 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 12, 14, 16,
 15, 13, 9, 5, -1, -6, -11, -14, -15, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7,
 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0,
 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 5, 7, 9, 11, 12, 14, 14, 15, 16, 16, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, -1, -1, -2,
 -2, -2, -3, -3, -3, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13,
 -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, -1, -2, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -1, -1, 0, 1, 2, 2, 3, 4, 4,
 5, 5, 5, 5, 10, 14, 15, 16, 14, 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 12, 14, 16, 15, 13, 9, 5, -1, -6, -11, -14, -15, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15,
 15, 15, 16, 16, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, 5, 7, 9, 10, 12, 13, 14, 15, 15, 16, 15, 15,
 14, 13, 11, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, 1, 2, 2, 3, 3, 4, 4, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -4, -4, -3, -3, -2, -2, -1, 0,
 1, 2, 2, 3, 3, 4, 7, 10, 13, 15, 15, 15, 15, 13, 10, 7, 4, 0, -4, -7, -10, -13, -15, -15, -15, -15, -13, -10, -7, -4, 0, 4, 7, 10, 13, 15, 15, 1, 2, 4, 4, 5, 5, 5, 5, 4,
 4, 2, 1, 0, -1, -2, -4, -4, -5, -5, -5, -5, -4, -4, -2, -1, 0, 1, 2, 4, 4, 5, 5, 10, 13, 15, 15, 14, 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 11, 14, 16,
 15, 13, 9, 5, -1, -6, -10, -14, -15, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7,
 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, -1, -1, -2, -2, -2, -2, -2, -1, 0, 1, 1, 2, 2, 2, 2, 2, 1, 0, 0, -1, -2, -2, -2, -2,
 -2, -1, -1, 0, 1, 2, 2, 2, 4, 7, 9, 10, 12, 13, 14, 15, 15, 15, 15, 15, 14, 13, 11, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, -1, -2, -3, -4, -4,
 -3, -3, -2, 0, 1, 2, 3, 3, 4, 3, 3, 2, 1, -1, -2, -3, -3, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 7, 10, 13, 14, 15, 15, 14, 13, 10, 7, 4, 0, -4, -7, -10, -13, -14, -15, -15,
 -14, -13, -10, -7, -4, 0, 4, 7, 10, 13, 14, 15, -2, -3, -5, -5, -5, -5, -4, -2, -1, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -2, -4, -5, -5, -5, -4, -3, -2, 0, 2, 4, 5, 5, 10, 13,
 15, 15, 14, 11, 7, 2, -4, -9, -12, -15, -15, -14, -12, -8, -3, 2, 7, 11, 14, 15, 15, 13, 9, 4, -1, -6, -10, -13, -15, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 4,
 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0,
 0, 0, 0, 0, 0, 1, 1, 1, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -4, -4, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2,
 2, 3, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, -5, -5, -5,
 -5, -4, -4, -3, -2, -2, -1, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 2, 1, 1, 2, 2, 2, 2, 2, 1, 0, -1, -1, -2, -2, -2, -2, -2, -1, 0, 0, 1, 2, 2, 2,
 2, 2, 1, 1, 0, -1, -2, -2, -2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1,
 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -3, -4, -4, 4, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -4, -4, -3, -3, -2, -2, -1,
 0, 1, 2, 2, 3, 3, 4, 4, 3, 3, 2, 2, 1, 2, 2, 3, 3, 4, 4, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -4, -4, -3, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 4, 5, 5, 4, 4, 2, 1, 0, -1,
 -2, -4, -4, -5, -5, -5, -5, -4, -4, -2, -1, 0, 1, 2, 4, 4, 5, 5, 5, 5, 4, 4, 2, 1, 2, 3, 4, 4, 3, 3, 2, 0, -1, -2, -3, -3, -4, -3, -3, -2, -1, 1, 2, 3, 3, 4, 4, 3, 2, 1,
 0, -1, -2, -3, -4, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -2, -2, -2, -1, 0, 1, 1, 2, 2, 2, 2, 2, 1, 0, 0, -1, -2, -2, -2, -2, -2, -1, -1, 0, 1, 2, 2, 2, 2, 2, 1, 1, 2, 2, 3, 4, 4,
 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 3, 3, 2, 1, -1, -2, -3, -3, -4, -3, -3, -2, -1,
 0, 2, 3, 3, 4, 4, 3, 2, 1, 2, 4, 4, 5, 5, 5, 5, 4, 4, 2, 1, 0, -1, -2, -4, -4, -5, -5, -5, -5, -4, -4, -2, -1, 0, 1, 2, 4, 4, 5, 5, -5, -5, -4, -2, 0, 2, 3, 4, 5, 5, 5,
 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2, 4, 5, 5, 5, 5, 3, 2, 3, 5, 5, 5, 5, 4, 2, 1, -1, -3, -4, -5, -5, -5, -4, -3, -1, 1, 2, 4, 5, 5, 5, 4, 3, 2, 0, -2, -4,
 -5, -5, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5,
 -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11,
 -10, -8, -5, -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6,
 -6, -5, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14,
 -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14,
 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6, -7, -8, -10, -11, -12, -13, -14, -15, -15, -16, -16, -16, -16,
 -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7,
 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12,
 -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5,
 -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 12, 8, 4, -1, -6, -10, -13,
 -15, -16, -16, -14, -11, -7, -2, 2, 7, 11, 14, 16, 16, 15, 13, 10, 6, 1, -4, -8, -12, -15, -16, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5,
 -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8,
 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4,
 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16,
 -16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13,
 -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15,
 -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 14, 12, 8, 4, -1,
 -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7,
 9, 10, 10, 11, 11, 11, 11, 11, 10, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9,
 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -15, -14, -12, -11,
 -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14,
 -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14,
 -14, -15, -15, -16, -16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4, -6, -8,
 -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12,
 -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16,
 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15,
 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 0,0,0,0,1,16, 16, 16, 16, 16, 16, 16,
 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5,
 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3,
 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14,
 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5,
 -4, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7,
 -6, -5, -4, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4,
 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -16, -16, -15, -15, -15, -14, -14, -13, -13, -12, -11,
 -10, -10, -9, -8, -7, -6, -5, -3, -2, -1, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10,
 10, 9, 8, 7, 6, 5, 3, 2, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, -15, -15, -15, -15, -15, -14, -14,
 -13, -13, -12, -11, -10, -9, -9, -8, -7, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14,
 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 16, 16, 15, 14,
 14, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7, 9, 10,
 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 16,
 16, 15, 14, 14, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5,
 7, 9, 10, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 6, 4, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -1, -1, -1, -1, -1, 0, 16, 15, 15, 14, 13, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -14, -15, -15, -16, -16, -15, -15, -14, -13, -12, -11, -9,
 -8, -6, -4, -2, 0, 2, 3, 5, 7, 9, 10, 11, 13, 14, 15, 15, 15, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7, 6, 4, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -2, -2, -2, -1, -1,
 0, 0, 0, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 0, 15, 15, 15, 14, 13, 12, 11, 9, 8, 6, 4, 3, 1, -1, -3, -5, -7, -8, -10, -11, -12, -13, -14, -15, -15, -15, -15, -15, -15, -14,
 -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 3, 5, 7, 9, 10, 11, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 13, 12, 10, 9, 7, 6, 4, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -2, -1,
 -1, 0, 0, 1, 1, 2, 2, 2, 2, 1, 0, 0, -1, -1, -2, -2, -2, -2, -1, -1, -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 4, 6, 8, 11, 12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 8,
 6, 3, 0, -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 16, 16, 15, 15, 14, 12, 10, 8, 5, 3, 3, 3, 3, 3,
 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 4, 6, 8, 11, 12, 14, 15, 16, 16, 16, 15, 14,
 12, 10, 8, 6, 3, 0, -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 16, 16, 15, 15, 14, 12, 10, 8, 5, 3,
 3, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 3, 6, 8, 10,
 12, 14, 15, 15, 16, 15, 15, 14, 12, 10, 8, 6, 3, 0, -2, -5, -7, -10, -11, -13, -14, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 15,
 16, 15, 15, 13, 12, 10, 8, 5, 3, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 3, 3, 2, 2, 2, 1, 1, -15, -15, -14, -13, -11, -9,
 -7, -4, -2, 1, 3, 6, 8, 10, 12, 14, 15, 15, 15, 15, 15, 13, 12, 10, 8, 6, 3, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4,
 6, 9, 11, 12, 14, 15, 15, 15, 15, 14, 13, 12, 10, 8, 5, 3, 2, 2, 1, 0, -1, -2, -2, -3, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 2, 2, 1, 1, 0, -1, -2, -2, -3, -3, -2, -2, -1,
 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 7,
 9, 11, 12, 14, 14, 15, 16, 16, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 7, 11, 13, 15, 16, 16, 15,
 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 10, 14, 15, 16, 14, 11, 7, 2, -4, -9, -13, -15,
 -16, -15, -12, -8, -3, 2, 7, 12, 14, 16, 15, 13, 9, 5, -1, -6, -11, -14, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11,
 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 7, 9, 11, 12, 14, 14, 15, 16, 16, 15,
 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4,
 -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 7, 11, 13, 15, 16, 16,
 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 10,
 14, 15, 16, 14, 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 12, 14, 16, 15, 13, 9, 5, -1, -6, -11, -14, -15, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4,
 5, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 5, 7, 9, 11, 12, 14, 14,
 15, 16, 16, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1,
 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 7, 11, 13, 15, 16, 16,
 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 1, 1, 2, 2, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2,
 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 10, 14, 15, 16, 14,
 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 12, 14, 16, 15, 13, 9, 5, -1, -6, -11, -14, -15, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6,
 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1,
 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 5, 7, 9, 11, 12, 14, 14,
 15, 16, 16, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, -1, -3, -5, -7, -9, -11, -13, -14, -15, -15, -16, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -3, -3, -3, -2, -1, -1, 0,
 0, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 3, 4, 4, 7, 11, 13, 15,
 16, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, -1, -2, -3, -3, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3,
 -2, -1, 0, 1, 1, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -1, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 10,
 14, 15, 16, 14, 11, 7, 2, -4, -9, -13, -15, -16, -15, -12, -8, -3, 2, 7, 12, 14, 16, 15, 13, 9, 5, -1, -6, -11, -14, -15, 1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4,
 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, -5,
 -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2,
 -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, 4, 4, 4, 3, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4,
 -4, -4, -4, -3, -3, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0,
 -1, -1, -2, -2, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5,
 -4, -4, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1, 0, 0, -1, -2, -2, -2, -2, -1, -1, 0,
 0, 1, 1, 2, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3,
 3, 3, 3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -2, -2, -3, -3, -4, -4,
 -3, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3,
 3, 3, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 2, 2, 2, 3, 3, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 3, -5, -5, -5, -4, -4, -3,
 -2, -2, -1, 0, 1, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 3, 3, 2, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 4, 4, 5, 5, 5,
 5, 5, 5, 5, 4, 3, 3, 2, 1, 2, 2, 3, 3, 2, 2, 1, 0, -1, -1, -2, -2, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 3, 2, 2, 1, 0, -1, -2, -2, -3, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2,
 -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5,
 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 10, 8, 6, 3, 0, -4, -7, -9, -11, -11, -11, -10, -8, -5, -2, 2, 5, 8, 10, 11, 11, 11, 9, 7, 4, 1, -3, -6, -8, -10, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6,
 -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7,
 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15,
 -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4,
 -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13,
 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6,
 -7, -8, -10, -11, -12, -13, -14, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14,
 -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2,
 -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14,
 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8,
 -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13,
 -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1,
 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15,
 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 12, 8, 4, -1, -6, -10, -13, -15, -16, -16, -14, -11, -7, -2, 2, 7, 11, 14, 16, 16, 15, 13, 10, 6, 1, -4, -8,
 -12, -15, -16, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6,
 -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8,
 -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14,
 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14,
 -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6,
 -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12,
 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10,
 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15,
 -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0,
 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15,
 -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5,
 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16,
 -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7,
 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16, -15, -14, -11, -7, -2, 2, 7, 11, 14, 15,
 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8,
 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9,
 -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9,
 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16,
 -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16,
 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16,
 -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16,
 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -15, -14,
 -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14,
 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15,
 -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 15, 14, 12, 10, 8,
 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11,
 -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8,
 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12,
 -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 14, 12, 8, 4, -1, -5, -9, -13, -15, -16,
 -15, -14, -11, -7, -2, 2, 7, 11, 14, 15, 16, 15, 13, 9, 5, 1, -4, -8, -12, -14, -16, 0,0,0,0,1,16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12,
 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4,
 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13,
 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6,
 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9,
 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11,
 11, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2,
 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15,
 -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4,
 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9,
 -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14,
 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7,
 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -15, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2,
 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15,
 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10,
 -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13,
 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9,
 9, 8, 8, 7, 7, 6, 6, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 16, 16, 16, 15, 15, 15,
 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15,
 -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9,
 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0,
 -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13,
 -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16,
 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, 0, 0, 0, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -10, -11, -12,
 -12, -13, -13, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -3, -2,
 -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 10, 9, 8, 7,
 6, 6, 5, 4, 3, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12,
 11, 10, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15,
 -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14,
 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0,
 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10,
 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 12, 12, 11, 10, 8, 7, 6, 5, 4, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16,
 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -16,
 -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14,
 13, 12, 12, 11, 10, 8, 7, 6, 5, 4, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12,
 -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -16, -16, -15, -15, -15, -14, -13, -13, -12, -11, -10,
 -9, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 10, 10, 8, 7, 6, 5, 3, 2, 1, -1, -2,
 -3, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4,
 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11,
 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15,
 -15, -15, -15, -15, -15, -15, -15, -15, -14, -13, -13, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12, 12, 13, 14, 14, 15, 15, 15, 15, 15,
 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0,
 0,0,1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2,
 -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4,
 -4, -4, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0,
 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2,
 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3,
 -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3,
 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
 -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1,
 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3,
 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1,
 -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4,
 -4, -4, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 1, 1, 1, 1, 1, 1,
 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10,
 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10,
 -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7,
 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10, -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5,
 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -10, -11, -11, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, -15, -6, 6, 15,
 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -10,
 -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -6, 6, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5,
 -6, -7, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13,
 13, 14, 14, 15, 15, -15, -6, 6, 15, 14, 14, 13, 12, 10, 8, 7, 5, 2, 0, -2, -4, -6, -8, -10, -11, -12, -13, -14, -15, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3, -1, 1,
 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 15, 14, 13, 12, 11, 10, 8, 6, 4, 2, 0, -2, -5, -7, -8, -10, -12, -13, -14, -14, -15, 8, -8, -8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, -11, -11,
 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7,
 -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, -11, -11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5,
 -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11,
 11, 11, -11, -11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10, -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9,
 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -10, -11, -11, -4, 10, -10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -6, 15, -15, 6, 6, 6, 6, 6, 6, 6, 6,
 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -6, -6, -6, -6,
 -6, -6, -6, -6, -6, -6, -6, -6, 15, -15, 6, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6,
 -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, -6, 15, -15, 6, 6, 6, 6, 5, 4, 4, 3, 2, 1, 0, -1, -2, -3, -3, -4, -5, -5,
 -6, -6, -6, -6, -6, -6, -6, -5, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 6, 6, 6, 6, 6, 6, 5, 5, 4, 3, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -6, -6, -6, 0,0,0,0,
1,15, 13, 10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 15, 13, 10, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 15, 13, 10, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1,
 -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1,
 15, 13, 10, 5, 5, 5, 4, 4, 3, 2, 1, 1, 0, -1, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 1, 0,
 -1, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -2, -1, -13, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11,
 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, -13, 0, 13, 13, 13, 13, 13, 12,
 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13,
 -12, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, -13, 0, 13, 13, 13, 12, 12, 11, 10, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11,
 -12, -13, -13, -13, -13, -13, -12, -12, -11, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 4, 6, 7, 9, 10, 11, 11, 12, 13, 13, 13, 13, 13, 12, 12, 11, 10, 9, 8, 6, 5, 3, 2, -13,
 0, 13, 13, 13, 12, 11, 9, 8, 6, 4, 2, -1, -3, -5, -7, -9, -10, -11, -12, -13, -13, -13, -12, -11, -10, -9, -7, -5, -2, 0, 2, 4, 6, 8, 10, 11, 12, 13, 13, 13, 13, 12, 11,
 9, 8, 5, 3, 1, -1, -3, -5, -7, -9, -10, -12, -12, -13, -13, -13, -12, -11, -10, -8, -6, -4, -2, 10, -13, -5, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14,
 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0,
 10, -13, -5, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14,
 -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 10, -13, -5, 15, 15, 14, 13, 13, 12, 10, 9, 8, 6, 4,
 2, 1, -1, -3, -5, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -7, -6, -4, -2, 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15,
 15, 14, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, 10, -13, -5, 15, 14, 13, 12, 11, 9, 7, 4, 2, -1, -3, -6, -8, -10, -12, -13, -14, -15, -15, -15, -14, -13, -11, -10, -8, -5,
 -3, 0, 2, 5, 7, 9, 11, 12, 14, 14, 15, 15, 14, 13, 12, 10, 9, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -14, -14, -13, -11, -9, -7, -5, -2, -5, 13, -15, 10,
 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2,
 2, 2, 2, 1, 1, 1, 1, 0, 0, -5, 13, -15, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9,
 -9, -9, -9, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, -5, 13, -15, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 0,
 -1, -2, -3, -4, -5, -6, -7, -8, -8, -9, -9, -10, -10, -10, -9, -9, -9, -8, -7, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 10, 9, 9, 9, 8, 7, 6,
 6, 5, 3, 2, 1, -5, 13, -15, 10, 9, 9, 8, 7, 6, 4, 3, 1, 0, -2, -4, -5, -6, -8, -8, -9, -10, -10, -10, -9, -8, -7, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 8, 9, 9, 10, 10, 9,
 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -3, -2, 0,1,0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7,
 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15,
 0, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2,
 -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, 1, 1, 2, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 14, 14, 13, 12, 11,
 10, 8, 7, 5, 3, 1, 0, -2, -4, -6, -7, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -5, -3, -1, 1, 2, 4, 6, 8, 9, 10, 12, 13, 13,
 14, 15, 15, 1, 2, 2, 2, 5, 7, 9, 11, 13, 14, 14, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -9, -10, -12, -13, -14, -15, -15, -14, -14, -12, -11, -9, -7, -5, -2, 0,
 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -12, -13, -14, -15, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5,
 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
 -1, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2,
 -3, -4, -5, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -12, -12, -13, -13, -13, -13, -2, -2, 0, 2, 3, 5, 6, 8, 9, 10, 11, 12, 12, 13, 13, 13, 13, 13, 12, 11, 11, 10, 9,
 7, 6, 4, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -11, -12, -12, -13, -13, -13, -13, -13, -12, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 10, 11, 12, 12,
 13, 13, -2, -2, 0, 2, 4, 6, 8, 10, 11, 12, 13, 13, 13, 12, 12, 10, 9, 7, 5, 3, 1, -1, -3, -5, -8, -9, -11, -12, -13, -13, -13, -13, -12, -11, -10, -8, -6, -4, -2, 0, 2,
 5, 7, 9, 10, 11, 12, 13, 13, 13, 12, 11, 10, 9, 7, 5, 3, 1, -2, -4, -6, -8, -9, -11, -12, -13, -13, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5,
 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 1, 0, -1, 1, 1, 2, 3, 3, 4,
 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7, -7, -8,
 -8, -9, -9, -9, -9, -10, -10, -10, 2, -1, -2, 1, 2, 3, 5, 6, 6, 7, 8, 9, 9, 9, 10, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -7, -8, -9, -9,
 -9, -10, -10, -10, -9, -9, -8, -8, -7, -6, -5, -4, -3, -2, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, 2, -1, -2, 2, 3, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 5, 4, 2,
 1, -1, -3, -4, -6, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -3, -1, 0, 2, 3, 5, 6, 7, 8, 9, 10, 10, 10, 9, 8, 8, 6, 5, 4, 2, 0, -1, -3, -4, -6, -7, -8, -9, -9,
 -10, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -1, 1, -1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1,
 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -1, 2, -2, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2,
 1, 1, 0, -1, -1, -2, -2, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5, -2, 2, -2, 1, 2, 2, 3,
 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -2, -1, 0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1,
 0, -1, -1, -2, -3, -4, -4, -5, -5, -5, 1,0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4,
 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, 2, 2, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5,
 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4,
 5, 5, 5, 5, 2, 2, 2, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -2, -1, 0, 1, 2, 3, 3, 4, 4,
 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -5, -5, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9,
 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, -1, 0, 1, 1, 2, 3, 4, 5, 5, 6, 7,
 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -9,
 -10, -11, -11, -12, -12, -12, -13, -13, -13, -13, -2, 0, 2, 2, 3, 5, 6, 8, 9, 10, 11, 12, 12, 13, 13, 13, 13, 13, 12, 11, 11, 10, 9, 7, 6, 4, 3, 1, 0, -2, -3, -5, -6,
 -8, -9, -10, -11, -12, -12, -13, -13, -13, -13, -13, -12, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 10, 11, 12, 12, 13, 13, -2, 0, 2, 2, 4, 6, 8, 10,
 11, 12, 13, 13, 13, 12, 12, 10, 9, 7, 5, 3, 1, -1, -3, -5, -8, -9, -11, -12, -13, -13, -13, -13, -12, -11, -10, -8, -6, -4, -2, 0, 2, 5, 7, 9, 10, 11, 12, 13, 13, 13,
 12, 11, 10, 9, 7, 5, 3, 1, -2, -4, -6, -8, -9, -11, -12, -13, -13, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10,
 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 1, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8,
 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10,
 -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, 1, -2, -1, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 5, 3, 1, 0, -2, -4,
 -6, -7, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -5, -3, -1, 1, 2, 4, 6, 8, 9, 10, 12, 13, 13, 14, 15, 15, 2, -2, -1, 2, 5,
 7, 9, 11, 13, 14, 14, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -9, -10, -12, -13, -14, -15, -15, -14, -14, -12, -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 11, 13, 14,
 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -12, -13, -14, -15, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6,
 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 0, 1, -1, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7,
 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9,
 -9, -10, -10, -10, -1, 2, -2, 1, 2, 3, 5, 6, 6, 7, 8, 9, 9, 9, 10, 10, 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -7, -8, -9, -9, -9, -10, -10, -10,
 -9, -9, -8, -8, -7, -6, -5, -4, -3, -2, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9, 10, 10, -1, 2, -2, 2, 3, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 5, 4, 2, 1, -1, -3, -4, -6,
 -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -3, -1, 0, 2, 3, 5, 6, 7, 8, 9, 10, 10, 10, 9, 8, 8, 6, 5, 4, 2, 0, -1, -3, -4, -6, -7, -8, -9, -9, -10, 1,8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0,
 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7,
 -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9,
 -10, -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9,
 -10, -10, -11, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -13, -9, -3, 3, 9, 13, 16, 16,
 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10,
 -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1,
 -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9,
 11, 12, 13, 14, 14, 15, 15, 16, 16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -6, -8, -10, -12, -13, -14, -15, -16, -16, -16, -15, -14,
 -13, -11, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 14, 13, 12, 10, 8, 6, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, 10, 5, -4,
 -10, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 15, 6, -6, -15, -15, -6, 6, 15, 15, 15, 15, 14, 14, 14, 14, 13,
 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12,
 -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, 15, 6, -6, -15, -15, -6, 6, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -7, -9,
 -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -7, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 14,
 15, 15, 15, 6, -6, -15, -15, -6, 6, 15, 14, 14, 13, 12, 10, 8, 7, 5, 2, 0, -2, -4, -6, -8, -10, -11, -12, -13, -14, -15, -15, -15, -14, -13, -12, -11, -9, -7, -5, -3,
 -1, 1, 3, 5, 7, 9, 11, 12, 13, 14, 15, 15, 15, 14, 13, 12, 11, 10, 8, 6, 4, 2, 0, -2, -5, -7, -8, -10, -12, -13, -14, -14, -15, -9, 2, 11, 6, -6, -11, -2, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3,
 -4, -4, -5, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 12,
 12, 11, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6,
 -5, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, -13, 3, 16, 9, -9, -16, -3, 13, 13, 12, 12, 10, 9, 8, 6, 4, 2, 0, -2, -4, -5, -7, -9, -10, -11,
 -12, -13, -13, -13, -13, -13, -12, -11, -10, -8, -6, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 13, 13, 13, 13, 13, 12, 11, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6, -8, -9, -10,
 -12, -12, -13, -13, 0,0,0,0,1,15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 15,
 15, 14, 12, 11, 8, 6, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1,
 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 15, 15, 14, 12, 11, 8, 6, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -3, -3,
 -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, -15, -11, -3, 6,
 12, 15, 14, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3,
 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, -15, -11, -3, 6, 12, 15, 14, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5,
 -5, -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, -15, -11, -3, 6, 12, 15, 14, 8, 8, 8,
 7, 7, 6, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -7, -8, -8, -8, -8, -8, -8, -7, -7, -6, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 8,
 8, 8, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, -15, -11, -3, 6, 12, 15, 14, 8, 8, 7, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -8, -8, -8, -7, -6, -5, -4, -3, -2, 0,
 1, 3, 4, 5, 6, 7, 7, 8, 8, 8, 8, 7, 7, 6, 5, 3, 2, 1, -1, -2, -3, -4, -6, -6, -7, -8, -8, -8, -8, -8, -7, -6, -5, -4, -3, -1, 14, 3, -11, -15, -8, 6, 15, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4,
 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 14, 3, -11, -15, -8, 6, 15, 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6,
 -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -4, -3, -2, -1, 14, 3,
 -11, -15, -8, 6, 15, 12, 12, 12, 11, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -12, -12, -12, -12, -12, -12, -12, -11, -11, -10, -9, -7, -6, -5,
 -3, -2, 0, 1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 14, 3, -11, -15, -8, 6, 15, 12, 12, 11, 10, 9, 7, 6, 4, 2, -1, -3, -5,
 -7, -8, -10, -11, -12, -12, -12, -12, -12, -11, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 9, 11, 12, 12, 12, 12, 12, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -12,
 -12, -12, -12, -12, -11, -9, -8, -6, -4, -2, -12, 6, 15, 3, -14, -11, 8, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13,
 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, -12, 6, 15, 3, -14, -11, 8, 15, 15,
 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, -12, 6, 15, 3, -14, -11, 8, 15, 15, 14, 14, 13, 12, 10, 9, 8, 6, 4, 2, 1,
 -1, -3, -5, -6, -8, -9, -11, -12, -13, -14, -14, -15, -15, -15, -15, -14, -13, -13, -11, -10, -9, -7, -6, -4, -2, 0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 14, 15, 15, 15,
 15, 14, 13, 12, 11, 10, 9, 7, 5, 4, 2, -12, 6, 15, 3, -14, -11, 8, 15, 14, 14, 12, 11, 9, 7, 4, 2, -1, -3, -6, -8, -10, -12, -13, -14, -15, -15, -15, -14, -13, -11, -10,
 -8, -5, -3, 0, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 9, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0,1,0,
 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13,
 13, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -13, -14, -14, -15,
 -15, -15, -15, 0, 1, 1, 1, 2, 2, 2, 2, 4, 5, 7, 9, 10, 11, 13, 13, 14, 15, 15, 15, 15, 15, 14, 13, 12, 11, 10, 8, 7, 5, 3, 2, 0, -2, -4, -6, -7, -9, -10, -12, -13, -14,
 -14, -15, -15, -15, -15, -15, -14, -13, -12, -11, -10, -8, -6, -5, -3, -1, 1, 3, 4, 6, 8, 9, 11, 12, 13, 14, 14, 15, 15, 0, 1, 1, 2, 2, 2, 2, 3, 5, 7, 10, 11, 13, 14,
 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, -2, -4, -6, -9, -11, -12, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13,
 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -13, -14, -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10,
 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, -1, -1, -1, -1, 0, 0, 1, 1,
 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5,
 -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, -1, -2, -2, -2, -1, 0, 1, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15, 15, 15, 15, 14, 14, 13, 12,
 11, 10, 8, 7, 5, 3, 1, 0, -2, -4, -6, -7, -9, -10, -11, -13, -13, -14, -15, -15, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -5, -3, -1, 1, 2, 4, 6, 8, 9, 10, 12, 13,
 14, 14, 15, 15, -1, -2, -3, -2, -1, 0, 2, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -9, -10, -12, -13, -14, -15, -15, -15, -14, -13,
 -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -12, -14, -14, -15, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2,
 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13,
 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 1, 1, 0, -1, -1, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12,
 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -13, -13, -13, -14, -14, -14, 2, 2, 1, -1, -2, -1, 0, 2, 3,
 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 3, 1, 0, -2, -4, -5, -7, -8, -9, -11, -12, -12, -13, -13, -14, -14, -14, -13, -13, -12, -11,
 -10, -9, -7, -6, -4, -3, -1, 1, 2, 4, 5, 7, 8, 10, 11, 12, 12, 13, 14, 14, 2, 2, 1, -1, -3, -2, 0, 2, 5, 7, 9, 10, 12, 13, 13, 14, 14, 13, 12, 11, 9, 8, 5, 3, 1, -1, -4,
 -6, -8, -10, -11, -12, -13, -14, -14, -13, -13, -12, -10, -8, -6, -4, -2, 0, 3, 5, 7, 9, 11, 12, 13, 14, 14, 14, 13, 12, 11, 9, 7, 5, 3, 1, -2, -4, -6, -8, -10, -11,
 -12, -13, -14, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11,
 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, -1, -1, 1, 1, 0, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12,
 -12, -12, -12, -12, -2, -1, 1, 2, 0, -2, -1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 1, 0, -2, -3, -5, -6, -7, -9, -10, -11,
 -11, -12, -12, -12, -12, -12, -12, -12, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 11, 12, 12, 12, -2, -1, 2, 2, 0, -3, -1, 2, 4, 6, 8, 9, 11,
 12, 12, 12, 12, 12, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -12, -12, -12, -12, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 11, 12, 12, 12, 12, 12, 11,
 10, 8, 7, 5, 3, 1, -2, -4, -6, -7, -9, -10, -11, -12, -12, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3,
 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2,
 -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 1, 1, 2, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3,
 -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, -1, -1, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1,
 -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -8, -2, -1, 0, 1, 2, 2, 2, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 7, 7, 6, 5, 4, 4,
 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -6, -7, -7, -8, -8, -8, -8, -8, -8, -7, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 6, 7, 7, 8, 8, 8, -2, -2, 0, 1, 2,
 3, 2, 1, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 7, 6, 6, 4, 3, 2, 1, -1, -2, -3, -5, -6, -7, -7, -8, -8, -8, -8, -7, -7, -6, -5, -4, -3, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8,
 7, 6, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -7, -8, -8, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8,
 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 1, 0, -1, -1, -1, 0, 1, 1, 2, 3, 4, 4,
 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -5, -6, -7, -8,
 -8, -9, -10, -10, -11, -11, -12, -12, -12, -12, -12, -12, 2, 0, -1, -2, -1, 1, 2, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 1,
 0, -2, -3, -5, -6, -7, -9, -10, -11, -11, -12, -12, -12, -12, -12, -12, -12, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 11, 12, 12, 12, 2, 0, -2,
 -3, -1, 1, 2, 2, 4, 6, 8, 9, 11, 12, 12, 12, 12, 12, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -12, -12, -12, -12, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 6,
 8, 10, 11, 12, 12, 12, 12, 12, 11, 10, 8, 7, 5, 3, 1, -2, -4, -6, -7, -9, -10, -11, -12, -12, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6,
 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, -1, 0, 1, 0, -1, -1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4,
 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, -2, 1, 2, 0, -2, -1, 1, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14,
 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 5, 3, 1, 0, -2, -4, -6, -7, -9, -10, -11, -13, -13, -14, -15, -15, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -5, -3,
 -1, 1, 2, 4, 6, 8, 9, 10, 12, 13, 14, 14, 15, 15, -2, 1, 3, 0, -2, -2, 1, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -9, -10, -12, -13,
 -14, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -12, -14, -14, -15, 1,8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10,
 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10,
 -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5,
 -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10, -10, -11, -11, -11, -11, -11, -10, -9,
 -8, -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -10, -11, -11, -11, -11, -10, -9, -7, -5,
 -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10,
 12, 14, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8,
 -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16,
 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12,
 -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 15, 15, 14,
 12, 11, 9, 7, 5, 3, 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15,
 15, 14, 12, 10, 8, 7, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12,
 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15,
 -15, -15, -15, -16, -16, -16, -16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5,
 -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14,
 14, 15, 15, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -6, -8, -10, -12, -13, -14, -15, -16, -16,
 -16, -15, -14, -13, -11, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 14, 13, 12, 10, 8, 6, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15,
 -16, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -15, -10, -2, 8,
 14, 16, 12, 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 7, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1,
 -2, -3, -3, -4, -5, -6, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -10, -2, 8, 14, 16, 12,
 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 2, 1, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -15,
 -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, -15, -10, -2, 8, 14, 16, 12, 4, -4, -12,
 -16, -14, -8, 2, 10, 15, 15, 14, 13, 12, 11, 9, 7, 5, 3, 0, -2, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -15, -14, -12, -11, -10, -7, -6, -3, -1, 1, 3, 6, 7,
 10, 11, 12, 14, 15, 15, 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 2, 0, -3, -5, -7, -9, -11, -12, -13, -14, -15, -15, 0,0,0,0,1,15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7,
 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 15, 15, 15, 15, 14, 14,
 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 6, 4, 3, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, 0, 0, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
 -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, -15, -14, -12, -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3,
 -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, -15, -14, -12,
 -8, -4, 0, 4, 8, 12, 14, 15, 15, 14, 12, 8, 4, 4, 4, 4, 3, 3, 2, 1, 1, 0, -1, -2, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, 0, 1, 1, 2, 3, 3, 4, 4, 4,
 4, 4, 4, 4, 4, 3, 3, 2, 1, 0, 0, -1, -2, -2, -3, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -1, -1, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1,
 1, 1, 1, 1, 0, 0, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3,
 -3, -4, -4, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -2, -1, -1, 15, 12, 6, -1, -8, -14,
 -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 7, 6, 6, 6, 5, 4, 4, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -6, -6, -6, -7, -7, -7, -7, -7, -7, -6, -6, -5, -5, -4, -3, -3,
 -2, -1, 0, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 15, 12, 6, -1, -8, -14, -15, -14, -10, -3, 4, 11, 15, 15, 13, 7, 7, 6, 6, 5, 4, 3,
 2, 1, 0, -2, -3, -4, -5, -6, -6, -7, -7, -7, -7, -7, -6, -5, -5, -4, -2, -1, 0, 1, 2, 3, 4, 5, 6, 6, 7, 7, 7, 7, 6, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -6, -7, -7,
 -7, -7, -7, -6, -5, -4, -3, -2, -1, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8,
 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15,
 -12, -3, 7, 14, 15, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10,
 -10, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -3, -3, -2, -1, -1, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 10, 9,
 9, 8, 8, 7, 6, 5, 4, 3, 2, 0, -1, -2, -3, -4, -5, -6, -7, -8, -8, -9, -9, -10, -10, -10, -9, -9, -9, -8, -7, -7, -6, -5, -4, -3, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 9,
 10, 10, 10, 9, 9, 9, 8, 7, 6, 6, 5, 3, 2, 1, -15, -8, 1, 11, 15, 14, 6, -4, -13, -15, -12, -3, 7, 14, 15, 10, 9, 9, 8, 7, 6, 4, 3, 1, 0, -2, -4, -5, -6, -8, -8, -9, -10,
 -10, -10, -9, -8, -7, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -9, -10, -10, -9, -9, -8, -7, -6, -5, -3,
 -2, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12,
 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4,
 6, 7, 8, 9, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -7, -8,
 -9, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 4, 6, 7, 9, 10, 12, 13, 14, 15, 15, 15, 15, 15, 15,
 14, 14, 13, 11, 10, 9, 7, 5, 3, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -15, -15, -15, -14, -13, -12, -11, -10, -8, -7, -5, -3, -1, 1, 3, 4, 6, 8,
 9, 11, 12, 13, 14, 15, 15, 15, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13,
 -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 6, 8, 10, 12, 13, 15, 15, 15, 15, 15, 14, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -13, -14, -15, -15, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12,
 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 3, 4,
 6, 7, 8, 9, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -7, -8,
 -9, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -1, -1, -1, -2, -2, -2, -2, -1, -1, -1, 0, 1, 1, 1, 2, 2, 4, 6, 7, 9, 10, 12, 13, 14, 15, 15, 15, 15,
 15, 15, 14, 14, 13, 11, 10, 9, 7, 5, 3, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -15, -15, -15, -14, -13, -12, -11, -10, -8, -7, -5, -3, -1, 1, 3, 4,
 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, -1, -1, -2, -2, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 1, -2, -4, -7,
 -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 6, 8, 10, 12, 13, 15, 15, 15, 15, 15, 14, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -13, -14,
 -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12,
 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1,
 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5,
 -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, 1, 2, 2, 2, 1, 1, 0, -1, -2, -2, -2, -1, 0, 1, 1, 2, 4, 5, 7, 9, 10, 12, 13, 14, 14, 15, 15,
 15, 15, 15, 14, 13, 12, 11, 10, 8, 7, 5, 3, 2, 0, -2, -4, -6, -7, -9, -11, -12, -13, -14, -14, -15, -15, -15, -15, -15, -14, -13, -12, -11, -10, -8, -6, -5, -3, -1, 1,
 3, 4, 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, 1, 2, 3, 2, 2, 1, 0, -2, -2, -3, -2, -1, 0, 1, 2, 3, 5, 7, 10, 12, 13, 14, 15, 15, 15, 15, 14, 12, 11, 8, 6, 4, 1, -2, -4, -6,
 -9, -11, -12, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -13, -14,
 -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11,
 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, -1, -1, -1, -1, 0, 1, 1, 1, 0, 0, -1, -1, -1, 0, 1,
 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4,
 -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -14, -15, -15, -15, -1, -2, -2, -1, 0, 1, 2, 2, 1, -1, -2, -2, -1, 0, 1, 2, 4, 5, 7, 9, 10, 11, 12, 13, 14,
 15, 15, 15, 15, 14, 14, 13, 12, 11, 10, 8, 7, 5, 3, 1, 0, -2, -4, -6, -7, -9, -10, -11, -13, -13, -14, -15, -15, -15, -15, -14, -14, -13, -12, -11, -9, -8, -6, -5, -3,
 -1, 1, 2, 4, 6, 8, 9, 10, 12, 13, 14, 14, 15, 15, -2, -3, -2, -1, 0, 2, 3, 2, 1, -1, -2, -3, -2, 0, 1, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1,
 -1, -4, -6, -9, -10, -12, -13, -14, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9,
 -11, -12, -14, -14, -15, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3,
 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4,
 -4, -4, -4, -2, -2, -1, -1, -1, 0, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3,
 -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 2, 2, 1, 1, 1, 2,
 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 2, 2, 1, 0, 0, -1, -2, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -2, -1, -1, 0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 4, 4, 4, 3, 3, 2, 2, 1,
 0, -1, -1, -2, -3, -3, -4, -4, -4, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4,
 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2,
 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6,
 -6, -6, -6, -7, -7, -7, -7, -7, 2, 1, 1, 0, -1, -2, -2, -2, -1, 0, 1, 1, 2, 2, 2, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 7, 7, 7, 7, 7, 6, 6, 5, 5, 4, 3, 2, 2, 1, 0, -1, -2,
 -3, -3, -4, -5, -5, -6, -6, -7, -7, -7, -7, -7, -7, -6, -6, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 4, 4, 5, 6, 6, 6, 7, 7, 7, 3, 2, 1, 0, -1, -2, -3, -2, -2, 0, 1,
 2, 2, 3, 2, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 6, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -6, -7, -7, -7, -7, -6, -6, -5, -4, -3, -2, -1, 0, 1, 3, 4, 5, 5, 6, 7, 7, 7,
 7, 7, 6, 6, 5, 4, 3, 2, 0, -1, -2, -3, -4, -5, -6, -6, -7, -7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5,
 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, -1, -1, 0, 1, 1, 1, 0, 0, -1,
 -1, -1, 0, 1, 1, 1, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2,
 -3, -3, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -2, -1, 0, 1, 2, 2, 1, -1, -2, -2, -1, 0, 1, 2, 2, 1, 2, 3, 5, 6, 6, 7, 8, 9, 9, 9, 10, 10,
 10, 9, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -7, -8, -9, -9, -9, -10, -10, -10, -9, -9, -8, -8, -7, -6, -5, -4, -3, -2, -1, 0, 2, 3, 4, 5, 6, 7, 8, 8,
 9, 9, 10, 10, -2, -1, 0, 2, 3, 2, 1, -1, -2, -3, -2, 0, 1, 2, 3, 2, 3, 5, 6, 7, 8, 9, 9, 10, 10, 9, 9, 8, 7, 5, 4, 2, 1, -1, -3, -4, -6, -7, -8, -9, -9, -10, -10, -9,
 -9, -8, -7, -6, -5, -3, -1, 0, 2, 3, 5, 6, 7, 8, 9, 10, 10, 10, 9, 8, 8, 6, 5, 4, 2, 0, -1, -3, -4, -6, -7, -8, -9, -9, -10, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3,
 -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10,
 -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10,
 -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5, -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10,
 -10, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -15, -15, -14, -13, -12, -11, -10, -8, -7, -6, -4, -2, -1, 1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 14, 15,
 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8,
 -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4,
 -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14,
 -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, -16, -16, -15,
 -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -7, -8,
 -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 8, 7, 4, 2, 0, -3, -5,
 -7, -9, -11, -12, -14, -15, -15, -16, 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10,
 -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3,
 -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4,
 -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7,
 -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14,
 15, 15, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 15, 15, 14, 12, 11,
 9, 7, 5, 3, 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14,
 12, 10, 8, 7, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5, -8, -10, -11, -11, -11, -9, -7,
 -4, 0, 3, 6, 8, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15, -12, -8, -4, 1, 6, 10, 13, 15, 16, 16, 14, 11, 7,
 2, -2, -7, -11, -14, -16, -16, -15, -13, -10, -6, -1, 4, 8, 12, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4,
 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16,
 -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5,
 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7,
 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, -16, -14, -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14,
 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15,
 16, 16, 16, 15, 15, 14, 12, 10, 8, 7, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, 0,0,0,0,1,16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12,
 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5,
 5, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3,
 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16, 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
 0, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16, 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1,
 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16, 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 2, 2, 1,
 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 1, 1, 1, 1, 0, -16, -15, -15, -14, -13, -11, -9, -7, -5, -3, -1, 2, 4, 6, 8, 10, 12, 13, 14, 15, 15, 16, 16, 15, 14, 14, 12, 11, 9, 7, 5, 2, 2, 2, 2, 2, 1, 1, 1,
 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -1, -1, -1, 0, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
 0, 0, 0, 0, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3,
 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -2, -2,
 -2, -2, -2, -1, -1, -1, -1, 0, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 4, 4, 3,
 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4,
 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 16, 15, 13, 11, 7, 4, 0, -4, -7, -11, -13, -15, -16, -16, -15, -13, -11, -7, -4, 0, 4, 7, 11, 13, 15, 16, 16, 15, 13, 11, 7, 4, 4, 3,
 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -3, -3,
 -3, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, -15, -14, -11, -6, -1, 5, 9, 13, 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 16, 15, 14,
 10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1,
 1, 1, 1, 1, 1, 1, 0, 0, 0, -15, -14, -11, -6, -1, 5, 9, 13, 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 16, 15, 14, 10, 5, 5, 5, 5, 5,
 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4,
 -4, -4, -4, -3, -3, -3, -2, -2, -2, -1, -1, 0, -15, -14, -11, -6, -1, 5, 9, 13, 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 16, 15, 14,
 10, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -3, -3, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 4, 4,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 2, 2, 1, 1, -15, -14, -11, -6, -1, 5, 9, 13, 15, 16, 14, 12, 7, 2, -3, -8, -12, -15, -16, -15, -13, -9, -4, 2, 7, 11, 14, 16, 15,
 14, 10, 5, 5, 5, 4, 4, 3, 2, 2, 1, 0, -1, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 1, 0, -1,
 -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -5, -4, -3, -3, -2, -1, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9,
 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10,
 -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 6, 7, 9,
 11, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -11,
 -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 14, 14, 15, 16, 16, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16, -16, -15, -15, -13, -12, -10, -7, -5, -2, 0, 3, 6, 8, 10, 12, 14,
 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14,
 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1,
 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0,
 -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
 -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 4, 6, 7, 9, 11, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13,
 -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 14, 14, 15, 16, 16, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3,
 -3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 2, 3, 3, 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15,
 -16, -16, -15, -15, -13, -12, -10, -7, -5, -2, 0, 3, 6, 8, 10, 12, 14, 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
 10, 10, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 15,
 15, 15, 14, 14, 13, 13, 12, 11, 10, 10, 9, 8, 7, 6, 5, 3, 2, 1, 0, -1, -2, -3, -5, -6, -7, -8, -9, -10, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -16, -16, 0, 1,
 1, 2, 2, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 4, 6, 7, 9, 10, 12, 13, 14, 15, 15, 16, 16, 15, 15, 15, 14, 13, 11,
 10, 9, 7, 5, 3, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -16, -16, -15, -15, -14, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 13,
 14, 15, 15, 16, 1, 1, 2, 2, 2, 3, 3, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 3, 5, 8, 10, 12, 13, 15, 15, 16, 15, 15, 14, 13,
 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16, -16, -15, -14, -13, -11, -10, -7, -5, -2, 0, 3, 6, 8, 10, 12, 14, 15, 15, 16, 15, 15, 14, 12, 10, 8, 6, 3, 1,
 -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4,
 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 3, 4, 6, 7, 8, 9, 9, 10, 11, 12,
 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -7, -8, -9, -9, -10, -11, -12, -13,
 -13, -14, -14, -15, -15, -15, -15, -15, -1, -1, -2, -2, -2, -2, -1, -1, 0, 0, 1, 2, 2, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 4, 6, 7, 9, 10,
 12, 13, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 11, 10, 9, 7, 5, 3, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -15, -15, -15, -15, -14, -13, -12, -11, -10,
 -8, -7, -5, -3, -1, 1, 3, 4, 6, 8, 9, 11, 12, 13, 14, 15, 15, 15, -1, -2, -2, -3, -3, -2, -2, -1, 0, 1, 1, 2, 2, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -3, -2, -2, -1, 0, 1,
 2, 2, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 3, 6, 8, 10,
 12, 13, 15, 15, 15, 15, 15, 14, 12, 10, 8, 6, 3, 1, -2, -4, -7, -9, -11, -13, -14, -15, -15, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, -3, -3,
 -2, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2,
 -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1,
 -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 2, 2, 1,
 1, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2,
 -1, -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 3, 2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2, -3, -3, -2, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 3, 3, 2, 2, 2, 1, 1, 1, 2, 2, 3,
 3, 3, 4, 4, 4, 4, 3, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0,
 -1, -2, -2, -3, -3, -3, -4, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -1, -1, -1, 0, 0, 0,
 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 2, 1, 1,
 0, 0, -1, -1, -2, -2, -2, -2, -1, 0, 0, 1, 1, 2, 2, 2, 2, 1, 1, 1, 2, 2, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -4,
 -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 5, 5, -3, -2, -2, -1, 0, 1, 2, 2, 3, 3, 2, 2, 1, 0, 0, -1, -2,
 -2, -3, -2, -2, -1, -1, 0, 1, 2, 2, 3, 3, 2, 2, 1, 2, 3, 3, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 0, -1, -1, -2, -3, -4, -4, -5, -5, -5, -5, -5, -5, -4, -4, -3, -2, -2,
 -1, 0, 1, 2, 3, 3, 4, 5, 5, 5, 5, 5, 5, 5, 4, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -5, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0,
 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10, -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5, -4,
 -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10,
 -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10,
 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -15, -15, -15, -15,
 -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13,
 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2,
 -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15,
 -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12,
 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6, -7, -8, -10, -11, -12, -13, -14, -15, -15,
 -16, -16, -16, -16, -16, -16, -15, -15, -14, -13, -12, -11, -10, -8, -7, -6, -4, -2, -1, 1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 14, 15, 15, 16, 16, 16, -16, -16, -16, -16,
 -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10,
 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 15, 14, 13, 11, 9, 7, 5, 3, 0, -2, -4, -7, -9, -10, -12, -14, -15, -15, -16, -16, -16, -15,
 -14, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 9, 7, 4, 2, 0, -3, -5, -7, -9, -11, -13, -14, -15, -16, -16, 11, 11,
 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5,
 -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 15,
 15, 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6, -7, -8, -10, -11, -12, -13, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -13, -12, -11, -10, -8,
 -7, -6, -4, -2, -1, 1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6,
 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16,
 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12,
 -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8,
 -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15,
 15, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14,
 -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -7, -8, -10, -12,
 -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 8, 7, 4, 2, 0, -3, -5, -7, -9,
 -11, -12, -14, -15, -15, -16, -11, -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11, 11, 11, 11, 10, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7,
 -8, -9, -10, -11, -11, -11, -11, -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, -16, -16, -15, -14, -13, -11, -9, -7, -5, -3, 0, 2, 4, 7, 9, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10,
 -12, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -9, -7, -4, -2, 0, 3, 5, 7, 9, 11, 13, 14, 15, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12,
 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15,
 -15, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4,
 -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7,
 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5,
 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1,
 -1, -4, -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3,
 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 8, 7,
 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, 0,0,0,0,1,16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13,
 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16,
 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6,
 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13,
 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16,
 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6,
 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2,
 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16,
 -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16,
 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, -16, -16, -16, -15, -15, -15, -14,
 -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14,
 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6,
 -5, -4, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2,
 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 16, 16, 15, 14, 14, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -15,
 -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7, 9, 10, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16,
 15, 14, 14, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7,
 9, 10, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 16, 16, 15, 14, 14, 12, 11, 10, 8,
 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7, 9, 10, 12, 13, 14, 15, 15,
 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7, 6, 4, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
 -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 16, 16, 15, 14, 14, 12, 11, 10, 8, 6, 5, 3, 1, -1, -3, -5, -7, -8, -10, -11,
 -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -12, -11, -9, -8, -6, -4, -2, 0, 2, 4, 5, 7, 9, 10, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 11, 9, 7,
 6, 4, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1,
 -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 4, 6, 8, 11, 12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 8, 6, 3, 0,
 -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 16, 16, 15, 15, 14, 12, 10, 8, 5, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
 -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 4, 6, 8, 11, 12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 8, 6, 3, 0, -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14,
 -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 16, 16, 15, 15, 14, 12, 10, 8, 5, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, -16, -15, -14, -13,
 -11, -9, -7, -5, -2, 1, 4, 6, 8, 11, 12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 8, 6, 3, 0, -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4,
 -2, 1, 4, 6, 9, 11, 13, 14, 15, 16, 16, 15, 15, 14, 12, 10, 8, 5, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -2,
 -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, -16, -15, -14, -13, -11, -9, -7, -5, -2, 1, 4, 6, 8, 11,
 12, 14, 15, 16, 16, 16, 15, 14, 12, 10, 8, 6, 3, 0, -2, -5, -7, -10, -12, -13, -15, -15, -16, -16, -15, -14, -13, -11, -9, -7, -4, -2, 1, 4, 6, 9, 11, 13, 14, 15, 16,
 16, 15, 15, 14, 12, 10, 8, 5, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2,
 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7,
 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2,
 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 6, 7, 9, 11, 12, 13, 14, 15, 15, 16, 16,
 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5,
 6, 8, 10, 11, 12, 14, 14, 15, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16, -16,
 -15, -15, -13, -12, -10, -7, -5, -2, 0, 3, 6, 8, 10, 12, 14, 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15,
 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14,
 -15, -15, -15, -16, -16, -16, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 6, 7, 9, 11, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7,
 5, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 14, 14, 15, 16,
 16, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16,
 -16, -15, -15, -13, -12, -10, -7, -5, -2, 0, 3, 6, 8, 10, 12, 14, 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15,
 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15,
 -15, -16, -16, -16, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 4, 6, 7, 9, 11, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6,
 -8, -9, -11, -12, -13, -14, -15, -15, -16, -16, -16, -15, -15, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 14, 14, 15, 16, 16, 0, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1,
 1, 2, 2, 2, 2, 2, 3, 3, 3, 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16, -16, -15, -15, -13, -12, -10, -7, -5,
 -2, 0, 3, 6, 8, 10, 12, 14, 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5,
 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16,
 16, 16, 16, 16, 16, 16, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15,
 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0, -1, -1, -1,
 -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1,
 -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 4, 6, 7, 9, 11, 12, 13, 14, 15, 15, 16, 16, 16, 15, 15, 14, 13, 12, 10, 9, 7, 5, 4, 2, 0, -2, -4, -6, -8, -9, -11, -12, -13, -14,
 -15, -15, -16, -16, -16, -15, -15, -14, -13, -11, -10, -8, -7, -5, -3, -1, 1, 3, 5, 6, 8, 10, 11, 12, 14, 14, 15, 16, 16, 0, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -2,
 -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -2, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2,
 2, 3, 3, 5, 8, 10, 12, 14, 15, 15, 16, 16, 15, 14, 13, 11, 9, 6, 4, 1, -2, -4, -7, -9, -11, -13, -14, -15, -16, -16, -15, -15, -13, -12, -10, -7, -5, -2, 0, 3, 6, 8, 10,
 12, 14, 15, 16, 16, 16, 15, 14, 12, 11, 8, 6, 4, 1, -2, -5, -7, -9, -11, -13, -14, -15, -16, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3, -3, -3, -3, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -3,
 -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2,
 -2, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, -1,
 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2,
 -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2,
 -2, -2, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1,
 -2, -2, -2, -2, -3, -3, -3, -3, -2, -2, -2, -2, -1, -1, 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -2, -3, -3, 1,8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0,
 -1, -1, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 8,
 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2,
 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 9, 8, 7, 5, 4, 2, 0, -1, -3, -5, -6, -7, -9, -10, -10, -11, -11, -11, -11, -11, -10, -9, -8, -7, -5,
 -4, -2, -1, 1, 3, 4, 6, 7, 8, 9, 10, 11, 11, 11, 11, 11, 10, 10, 9, 7, 6, 5, 3, 1, 0, -2, -3, -5, -6, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5,
 -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8,
 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13,
 -13, -12, -12, -12, -12, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1,
 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15,
 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2,
 -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16,
 -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9,
 -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9,
 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15,
 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6, -7, -8, -10, -11, -12, -13, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -13, -12, -11, -10, -8, -7,
 -6, -4, -2, -1, 1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 14, 15, 15, 16, 16, 16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14,
 -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3,
 -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13,
 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 14, 13, 11, 9, 7, 5, 3, 0, -2, -4, -7, -9, -10, -12, -14, -15, -15,
 -16, -16, -16, -15, -14, -13, -12, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 12, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 9, 7, 4, 2, 0, -3, -5, -7, -9, -11, -13, -14,
 -15, -16, -16, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6,
 -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8,
 -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8,
 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16,
 -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3,
 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11,
 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15,
 -15, -16, -16, -16, -16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5,
 -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14,
 -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13,
 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16,
 -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14,
 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13,
 -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7,
 -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 15, 15, 14, 12, 11, 9,
 7, 5, 3, 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4, 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12,
 10, 8, 7, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3,
 3, 4, 5, 6, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7,
 -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5,
 6, 6, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -15, -15,
 -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15,
 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16,
 -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16,
 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11,
 -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2,
 -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3,
 -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5,
 -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8,
 -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15,
 15, 16, 16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15,
 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13,
 14, 14, 15, 15, 15, 16, 16, 16, 15, 15, 14, 12, 11, 9, 7, 5, 3, 0, -2, -4, -7, -8, -10, -12, -14, -15, -15, -16, -16, -16, -15, -14, -13, -11, -10, -8, -6, -4, -1, 1, 4,
 6, 8, 10, 11, 13, 14, 15, 16, 16, 16, 15, 15, 14, 12, 10, 8, 7, 4, 2, 0, -3, -5, -7, -9, -11, -12, -14, -15, -15, -16, 0,0,0,0,1,16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13,
 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5,
 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12,
 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4,
 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11,
 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2,
 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11,
 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1,
 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8,
 -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 14, 15,
 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6,
 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10,
 -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13,
 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9,
 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -14, -13, -13, -13, -12,
 -12, -12, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11,
 11, 12, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 12, 12,
 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14,
 -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 7, 8,
 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14,
 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12,
 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16,
 -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 13,
 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15,
 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15,
 -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9,
 -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -10, -10, -9, -8, -7,
 -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12,
 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0,
 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2,
 1, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14,
 -14, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -4, -4, -2, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16,
 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -16, -16, -15, -15, -15, -14,
 -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 12, 12, 11, 10, 8,
 7, 6, 5, 4, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7,
 -6, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15,
 15, 14, 13, 12, 12, 11, 10, 8, 7, 6, 5, 4, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14,
 -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7,
 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9,
 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 12, 12, 11, 10, 8, 7, 6, 5, 4, 2, 1, -1, -2, -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14,
 -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4, 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15,
 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10,
 -9, -8, -6, -5, -4, -2, -1, 0, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 12, 12, 11, 10, 8, 7, 6, 5, 4, 2, 1, -1, -2,
 -3, -5, -6, -7, -8, -9, -11, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, 0, 1, 2, 4,
 5, 6, 7, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,
 0, 0,0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, -3, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
 -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3,
 -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2,
 -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2,
 -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2,
 2, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2,
 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9,
 -9, -9, -10, -10, -10, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10,
 -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2,
 -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9,
 9, 8, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9,
 -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7,
 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -10, -4, 5, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, -15, -6, 6, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5,
 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -9, -10, -10,
 -10, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15,
 -6, 6, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8,
 -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -13, -13, -12, -12, -12,
 -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14,
 14, 14, 15, 15, 15, -15, -6, 6, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -6, -6, -7, -8, -9, -10, -11, -12, -12,
 -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10,
 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 6, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11,
 -12, -12, -13, -13, -14, -14, -14, -15, -15, 8, -8, -8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0,
 -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10,
 -10, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9,
 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1,
 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, -11, -11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 6, 6,
 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7,
 -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3,
 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -4, 10, -10, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, -6,
 15, -15, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2,
 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -5, -5, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, 15, -15, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5,
 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6,
 -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3,
 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, -6, 15, -15, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2,
 -3, -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3,
 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -5, -5, -5, -5, -6,
 -6, -6, -6, -6, -6, -6, 0,0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7,
 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0,
 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4,
 -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, 0, 1,
 1, 1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 6, 5,
 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -13, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13,
 -13, -13, -12, -12, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15,
 15, 0, 1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, 0, -2, -3, -4, -5, -6, -8,
 -9, -10, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10,
 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14,
 -14, -14, -15, -15, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7,
 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4,
 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12,
 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -7,
 -8, -8, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -1, -1, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 8,
 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4,
 -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8, -7,
 -7, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, -1, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -12, -13, -13,
 -13, -13, -13, -13, -13, -12, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13,
 12, 12, 11, 11, 10, 9, 8, 8, 7, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -13, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2,
 -3, -3, -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, 1, 0, -1, 1, 1, 2,
 2, 3, 3, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3,
 -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4,
 -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 1, 0, -1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10,
 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8,
 -8, -7, -6, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -2,
 -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2,
 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3,
 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -1, 1, -1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4,
 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
 -5, -5, -5, -5, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, -1, 1,
 -1, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4,
 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3,
 -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -5, -5, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4,
 4, 5, 5, 5, 5, 5, 5, 5, 5, 1, 1, 1, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2,
 -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -2, -2, -1, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5,
 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8,
 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -10, -11,
 -11, -11, -11, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -1, 0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -11,
 -11, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 3,
 3, 4, 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, -1, 0, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12,
 12, 12, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -13, -12, -12, -11, -11,
 -10, -9, -9, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 9, 8, 8, 7, 5, 4, 3, 2, 1,
 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -13, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4,
 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14,
 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4,
 4, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14,
 -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, 1, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14,
 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -13, -13, -13, -14, -14,
 -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -13, -12, -12, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8,
 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 14, 15, 15, 15, 1, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 14, 14, 14, 13, 12, 12,
 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, 0, -2, -3, -4, -5, -6, -8, -9, -10, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -11, -10,
 -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 0, -1,
 -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -14, -15, -15, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 0, 0, 0, 0, 1, 1,
 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9,
 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -7,
 -7, -7, -7, -8, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, 0, 1, -1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9,
 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -8, -8, -9,
 -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6,
 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 0, 1, -1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 1,
 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -6, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4,
 5, 6, 6, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10,
 -10, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1,
 -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10,
 -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9,
 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1,
 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9,
 8, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9,
 -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7,
 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -13, -9, -3,
 3, 9, 13, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9,
 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9,
 -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -16, -16, -16, -16, -16, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1,
 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15,
 -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10,
 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 10, 9, 8, 7, 6, 4, 3,
 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9,
 -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7,
 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -16, -16, 10, 5, -4, -10, -10, -4, 5, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 15, 6, -6, -15,
 -15, -6, 6, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 9, 8, 8,
 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -6, -7, -7, -7, -8, -8, -8,
 -9, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -14, -14, -14, -14, -15,
 -15, -15, -15, -15, -15, 15, 6, -6, -15, -15, -6, 6, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 2, 1, 0, 0,
 -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14,
 -14, -14, -14, -13, -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10,
 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 6, -6, -15, -15, -6, 6, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
 -1, -2, -3, -4, -6, -6, -7, -8, -9, -10, -11, -12, -12, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7,
 -6, -5, -4, -3, -2, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 6, 4, 3, 2,
 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -9, 2, 11, 6, -6, -11, -2, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
 9, 9, 9, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9,
 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -6, -6,
 -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13,
 -13, -13, -13, -13, -13, -13, -13, -13, -13, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4,
 3, 2, 2, 1, 0, 0, -1, -2, -2, -3, -4, -4, -5, -5, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13,
 -13, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -7, -6, -5, -5, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 6, 7, 7,
 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, -13, 3, 16, 9, -9, -16, -3, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4,
 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -13, -12, -12, -12, -11, -11, -10, -9, -8, -8,
 -7, -6, -5, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 7, 7, 6, 5, 4,
 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -6, -7, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -13, -13, -13, 0,0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11,
 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,
 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12,
 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10,
 -11, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, 0, 0, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10,
 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5,
 -6, -7, -8, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -9,
 -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 0, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 6,
 7, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -12, -13,
 -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -14, -15, -15, -15, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9,
 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, -1, 0, 0, 0, 0, 0, 1, 2, 2, 3, 3,
 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14,
 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6,
 -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -1, -1, -1, -1, 0, 0, 1,
 1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4,
 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13,
 -13, -12, -12, -11, -11, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, -1,
 -1, -1, -1, 0, 0, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, 0, -2, -3, -4, -5,
 -6, -8, -9, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 3, 4, 6, 7, 8,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13,
 -14, -14, -15, -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7,
 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 0,
 0, -1, 0, 0, 0, 1, 2, 2, 2, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14,
 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3,
 -4, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, 1,
 1, 0, -1, -1, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 10, 9, 9, 8,
 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -14, -14, -14, -13,
 -13, -13, -12, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 13, 13, 13,
 13, 14, 14, 14, 1, 1, 1, -1, -1, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
 -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2,
 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -5, -7, -8, -9, -9, -10, -11,
 -12, -12, -13, -13, -13, -14, -14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6,
 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 0, 0, 0,
 0, 0, -1, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3,
 -4, -4, -5, -5, -5, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -1, -1, 1,
 1, 0, -1, 0, 1, 2, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5,
 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12,
 -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 12,
 12, -1, -1, 1, 1, 0, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3,
 -4, -5, -6, -7, -8, -9, -10, -10, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7,
 8, 8, 9, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12,
 -12, -12, -12, -12, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3,
 -3, -3, -3, -3, -3, -3, -3, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2,
 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3,
 -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1,
 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3,
 -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1,
 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7,
 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0,
 1, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7,
 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5,
 -5, -5, -6, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7,
 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7,
 -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5,
 5, 5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, -1, -1, 0, 1, 1, 1, 1, 1, 1, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2,
 1, 1, 0, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7, -7, -6, -6, -5, -5, -4, -4, -3, -2, -2, -1, 0, 0, 1, 2, 2, 3, 4,
 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 3, 3, 2, 1, 1, 0, -1, -1, -2, -3, -3, -4, -4, -5, -6, -6, -6, -7, -7, -8, -8, -8, -8, -8, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7,
 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 0, 0, 0, -1, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4,
 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 6, 6, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3, -3, -4, -4, -5, -5, -5, -6, -6, -7, -7,
 -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, 1, 0, -1, -1, -1, 0, 1, 1, 2, 2, 3, 4, 4, 5,
 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3,
 -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -10, -10, -9, -9, -8,
 -8, -7, -6, -6, -5, -4, -4, -3, -2, -1, -1, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 1, 0, -1, -1, -1, 1, 1, 1, 2, 3, 4,
 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -10, -11,
 -11, -12, -12, -12, -12, -12, -12, -12, -12, -12, -11, -11, -10, -10, -9, -8, -7, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 12, 12, 12,
 12, 12, 12, 12, 12, 12, 11, 11, 10, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -11, -12, -12, -12, -12, -12, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9,
 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 2, 3, 3, 4, 4,
 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14,
 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7,
 -8, -8, -9, -9, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -1, 0, 1, 0, -1, -1, 1, 1, 2, 3,
 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6, 5, 4, 3, 2, 1,
 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13,
 -12, -12, -11, -11, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, -1, 1,
 1, 0, -1, -1, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 3, 2, 1, 0, -2, -3, -4, -5, -6,
 -8, -9, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -5, -4, -3, -2, 0, 1, 2, 3, 4, 6, 7, 8, 9,
 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14,
 -14, -15, -15, -15, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5,
 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7,
 -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1,
 -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11,
 -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10,
 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 2,
 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -5,
 -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0,
 -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -15,
 -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12,
 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5,
 -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15,
 -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15,
 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13,
 -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7,
 -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, -16, -15, -14, -12,
 -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12,
 -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10,
 -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 11, 9, 6, 2, -2, -6, -9, -11, -11, -9, -6, -2, 2, 6, 9, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9,
 -3, 3, 9, 13, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 10, 9,
 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9,
 -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -16, -16, -16, -16, -16, -16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9,
 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15,
 -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2,
 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 13, 9, 3, -3, -9, -13, -16, -16, -13, -9, -3, 3, 9, 13, 16,
 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -15, -16,
 -16, -16, -16, -15, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15,
 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -10, -11, -12, -13, -13, -14, -14, -15, -15,
 -15, -16, -16, -11, -7, -1, 5, 10, 11, 9, 3, -3, -9, -11, -10, -5, 1, 7, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -15, -10, -2, 8, 14, 16, 12, 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4,
 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11,
 -11, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -10, -2,
 8, 14, 16, 12, 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, 0, -1,
 -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -10, -10, -11, -11, -12, -12, -13, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -15, -14, -14, -14, -13, -13, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11,
 11, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, -15, -10, -2, 8, 14, 16, 12, 4, -4, -12, -16, -14, -8, 2, 10, 15, 15, 15, 15, 15, 14, 14, 13, 12, 12, 11,
 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13,
 -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13,
 13, 12, 11, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, -15, 0,0,0,0,0,0,1,0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0,
 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15,
 -15, -15, -15, -15, -15, -15, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14,
 -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4,
 5, 6, 7, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14,
 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 12, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -13, -13, -14, -15, -15, -15, -15, -15,
 -15, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14,
 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10,
 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4,
 -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, 0,
 -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14,
 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15,
 -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 10,
 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15,
 15, 15, 15, 15, 14, 14, 13, 12, 12, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -13, -13, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15,
 -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 13, 12, 11, 10,
 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10,
 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3,
 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 14, 14, 14, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7,
 -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, 0, 1, 1, 1, 1, 0, 0, -1,
 -1, -1, -1, -1, 0, 0, 1, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11,
 11, 10, 9, 8, 7, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -8, -9, -10, -11, -11, -12, -13, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -13, -13, -12, -11, -11, -10, -9, -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14,
 14, 14, 15, 15, 15, 15, 15, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 1, 1, 1, 3, 4, 5, 6, 7, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14,
 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12,
 -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3,
 2, 0, -1, -2, -4, -5, -6, -7, -8, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2,
 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11,
 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7,
 7, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13,
 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9,
 -9, -10, -10, -11, -11, -11, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -1, -1, -1, 0, 0, 1, 1, 1, 0, 0, -1, -1, -1,
 0, 1, 1, 2, 3, 4, 4, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 13, 13, 13, 12, 12, 11, 10, 10, 9, 8, 7, 6, 6,
 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7, -7, -8, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14,
 -14, -13, -13, -12, -12, -11, -11, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, -2, -1, 0, 1, 2, 3, 4, 5, 5, 6, 7, 8, 9, 9, 10, 11, 11, 12, 13, 13, 13, 14, 14, 14, 15, 15,
 15, 15, -1, -1, -1, -1, 0, 1, 1, 1, 0, -1, -1, -1, -1, 0, 1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9,
 8, 7, 6, 5, 3, 2, 1, 0, -2, -3, -4, -5, -6, -8, -9, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7,
 -5, -4, -3, -2, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 7, 6, 5, 4, 3, 1, 0, -1, -2, -4, -5,
 -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -15, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -3,
 -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 2, 2,
 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2,
 -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0,
 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -2, -2, -2, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3,
 -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2,
 -2, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6,
 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 1, 0, 0, 0, 0, 0, -1, 0, 0,
 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -4, -4,
 -4, -4, -4, -5, -5, -5, -5, -5, -5, -6, -6, -6, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, 1, 1, 0, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 2,
 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 3, 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -2, -3,
 -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -6, -5, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1,
 -1, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 1, 1, 1, 0, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6,
 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -5, -6, -6, -6, -7, -7, -7, -7, -7, -7, -7, -7, -7, -6,
 -6, -6, -5, -5, -5, -4, -4, -3, -3, -2, -1, -1, 0, 0, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 5, 5, 5, 4, 4, 3, 2, 2, 1, 1, 0, -1, -1,
 -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -7, -7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
 10, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10,
 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -2, -2,
 -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -7, -7, -7, -7, -8, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -1, -1, 0, 1,
 1, 1, 0, 0, -1, -1, -1, 0, 0, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 6, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 6, 6, 5,
 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9,
 -8, -8, -8, -7, -7, -6, -6, -5, -5, -4, -4, -3, -3, -2, -2, -1, 0, 0, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, -1, -1, 0, 1, 1, 1,
 1, 0, -1, -1, -1, 0, 1, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 1, 0, -1, -2, -3, -3, -4, -5,
 -6, -6, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -6, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 6, 7, 8, 8, 8, 9, 9,
 9, 10, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -5, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, 1,8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10,
 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3,
 -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5,
 -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9,
 -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9,
 8, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9,
 -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7,
 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6,
 -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -15, -15, -14, -13, -12, -11, -10, -8, -7, -6, -4, -2, -1, 1, 2, 4, 6, 7, 8,
 10, 11, 12, 13, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11,
 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7,
 -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15,
 -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14,
 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8,
 -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13,
 -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15,
 15, 15, 15, 16, 16, 16, 16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 15,
 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16,
 -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16,
 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16,
 11, 11, 10, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -10, -11, -11, -11, -11, -10, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8,
 -10, -12, -14, -15, -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14,
 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2,
 -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14,
 -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15,
 -16, -16, -15, -14, -12, -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4,
 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16,
 -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8,
 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 15, 14, 12, 10, 8, 4, 2, -2, -4, -8, -10, -12, -14, -15, -16, -16, -15, -14, -12,
 -10, -8, -4, -2, 2, 4, 8, 10, 12, 14, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12,
 -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10,
 -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -11, -10, -8, -6, -3, 1, 4, 7, 9, 11, 11, 11, 10, 8, 5, 2, -2, -5, -8, -10, -11, -11, -11, -9, -7, -4, 0, 3,
 6, 8, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, -16, -15, -12, -8, -4, 1, 6, 10, 13, 15, 16, 16, 14, 11, 7, 2, -2, -7, -11, -14, -16, -16, -15, -13, -10, -6, -1, 4, 8, 12, 15, 16, 16, 16, 16, 16, 16,
 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5,
 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11,
 -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -14,
 -12, -8, -4, 1, 5, 9, 13, 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13,
 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15,
 -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4,
 -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, -16, -14, -12, -8, -4, 1, 5, 9, 13,
 15, 16, 15, 14, 11, 7, 2, -2, -7, -11, -14, -15, -16, -15, -13, -9, -5, -1, 4, 8, 12, 14, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0,
 -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7,
 -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4,
 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0,0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8,
 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12,
 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11,
 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12,
 -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9,
 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15,
 -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15,
 15, 15, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16,
 16, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16,
 -15, -15, -14, -14, -13, -12, -11, -11, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13,
 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8,
 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14,
 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0,
 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12,
 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11, 11,
 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12, -13,
 -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0,
 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11,
 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16,
 -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14,
 15, 15, 15, 15, 16, 16, 16, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11,
 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15,
 -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -11, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16, 16,
 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6,
 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13,
 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7, 8,
 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14,
 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9,
 -10, -10, -11, -11, -11, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
 -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 15, 15, 15, 15,
 15, 14, 14, 13, 13, 12, 11, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -15, -15, -15,
 -15, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9,
 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 3,
 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 15, 15, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12,
 -13, -14, -14, -15, -15, -15, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 10, 10, 11, 12, 13, 14,
 14, 15, 15, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -13, -13, -14, -15, -15, -15, -16,
 -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4,
 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12,
 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4,
 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14,
 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8,
 -8, -9, -9, -10, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, 0, -1, -1, -1, -1, -1, -1, 0, 0,
 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 2, 3, 4, 5, 6, 6, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 11, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -3, -4, -5, -6, -7, -8, -9, -9, -10, -11, -12, -12, -13, -13, -14, -14,
 -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -11, -10, -9, -9, -8, -7, -6, -5, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6,
 7, 7, 8, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0,
 1, 1, 1, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 13, 12, 12, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -5, -7, -8, -9,
 -10, -11, -12, -13, -13, -14, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11,
 12, 13, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -7, -9, -10, -11, -12, -12, -13, -14, -14, -15,
 -15, -15, -15, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, -1, -1, -1, -1,
 -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0,
 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
 0, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4, 1, 1, 1,
 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4,
 -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3,
 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4,
 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3,
 -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2,
 -2, -2, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4,
 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
 5, 5, 5, 5, 5, 5, 5, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3,
 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2,
 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -4, -4, -4, -4, -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3,
 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3, -3, -3,
 -4, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -3, -3, -3, -2, -2, -2, -2, -1, -1, -1, 0, 0, 0, 1, 1, 1,
 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
 1, 1, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, -1, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -5,
 -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, 0, 0, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 3,
 3, 3, 2, 2, 1, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -3, -4, -4, -4, -5, -5, -5, -5, -5, -5, -5, 1,8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10,
 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3, -3,
 -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -10, -11, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9,
 -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4,
 -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7, -8, -8,
 -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6, 7, 7,
 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9, -9,
 -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -9, -9, -9, -8, -8, -7, -7, -6, -6, -5, -5, -5, -4, -3, -3, -2, -2,
 -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -15, -15, -15, -15,
 -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13,
 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12,
 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6,
 -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15,
 -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8,
 -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16,
 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12,
 -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -10,
 -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16,
 -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6,
 7, 7, 8, 8, 9, 10, 10, 11, 11, 12, 12, 13, 14, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1,
 -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -10, -9, -8, -7, -6,
 -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2,
 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, 11, 11, 11, 11, 10, 10, 9, 8, 8, 7, 6, 5, 4, 3, 2, 1, 0, -2, -3,
 -4, -5, -6, -7, -8, -8, -9, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -8, -8, -7, -6, -5, -4, -3, -2, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 10, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 16, 16, 16, 15, 15, 14, 13, 12, 11, 10, 8, 7, 6, 4, 2, 1, -1, -2, -4, -6, -7, -8, -10, -11, -12, -13, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15,
 -15, -14, -13, -12, -11, -10, -8, -7, -6, -4, -2, -1, 1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15,
 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0,
 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13,
 -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7,
 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5,
 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2,
 -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15,
 -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12,
 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 9, 8, 7, 5, 4, 2, 1, -1, -2, -4, -5, -7, -8, -9, -11, -12, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -9, -8, -7, -5, -4, -2, -1, 1, 2, 4, 5, 7, 8, 9, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 15, 15, 15,
 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16,
 -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16,
 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -11,
 -11, -10, -10, -9, -8, -6, -5, -3, -2, 0, 1, 3, 5, 6, 7, 9, 10, 10, 11, 11, 11, 11, 11, 10, 9, 8, 7, 6, 4, 3, 1, -1, -2, -4, -5, -7, -8, -9, -10, -11, -11, -11, -11,
 -11, -10, -10, -9, -7, -6, -5, -3, -1, 0, 2, 4, 5, 7, 8, 9, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -15, -14, -13, -11, -9, -7, -5, -3, 0, 2, 4, 7, 9, 10, 12, 14, 15, 15, 16, 16,
 16, 15, 14, 13, 12, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -12, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -9, -7, -4, -2, 0, 3, 5, 7, 9, 11, 13, 14, 15, 16,
 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8,
 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10,
 -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16,
 -16, -16, -16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3, 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -11,
 -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7, -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12,
 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15,
 -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2,
 -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, -16, -15, -15, -14, -12, -11, -9, -7, -5, -3,
 0, 2, 4, 7, 8, 10, 12, 14, 15, 15, 16, 16, 16, 15, 14, 13, 11, 10, 8, 6, 4, 1, -1, -4, -6, -8, -10, -11, -13, -14, -15, -16, -16, -16, -15, -15, -14, -12, -10, -8, -7,
 -4, -2, 0, 3, 5, 7, 9, 11, 12, 14, 15, 15, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12,
 -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8,
 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10,
 -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0,0,0,0,0,0,1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5,
 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12,
 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14,
 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8,
 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14,
 -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16,
 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13,
 -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0,
 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16,
 16, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16,
 -15, -15, -14, -14, -13, -12, -11, -11, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13,
 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2,
 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11,
 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1,
 -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9,
 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14,
 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10,
 -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 8, 7, 6, 5,
 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14,
 -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16,
 16, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9,
 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -11, -9, -8, -7, -6,
 -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6,
 -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5,
 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13,
 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16,
 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14,
 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7,
 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14,
 -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15,
 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 12, 12, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11,
 -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4,
 -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9,
 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14,
 -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -11, -11, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16,
 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5, 4, 3, 2, 0, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10,
 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15,
 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2,
 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15,
 15, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 0, 0, -1, -1, -2, -2, -3, -4, -4, -5, -5, -6,
 -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, -12, -13, -13, -13, -14, -14, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, 0, 0, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13,
 12, 12, 11, 10, 9, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -2, -4, -4, -5, -6, -7, -8, -9, -10, -10, -11, -12, -12, -13, -14, -14, -14, -15, -15, -15, -16, -16, -16, -16,
 -16, -16, -16, -15, -15, -15, -14, -14, -14, -13, -12, -12, -11, -11, -10, -9, -8, -7, -6, -5, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 9, 10, 11, 11, 12, 13,
 13, 14, 14, 15, 15, 15, 15, 16, 16, 16, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 15, 15,
 15, 14, 13, 13, 12, 11, 10, 9, 7, 6, 5, 4, 2, 1, 0, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13,
 -12, -11, -11, -9, -8, -7, -6, -5, -3, -2, -1, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 12, 13, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 5,
 4, 3, 2, 0, -1, -2, -4, -5, -6, -8, -9, -10, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, 1,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1,
 -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
 -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -3, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1,
 -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3,
 -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
 3, 3, 3, 3, 3, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0,
 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2,
 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -3, -3, -3, -3, 1,8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10,
 10, 10, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -1, -2, -2, -2, -3,
 -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, -5, -6, -6, -6, -6, -7, -7, -7, -7, -7, -8, -8, -8, -8, -8, -9, -9, -9, -9, -9, -9, -10, -10, -10, -10, -10, -10, -10, -10, -11,
 -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3,
 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11,
 -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8,
 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 2, 2, 1, 0, -1, -2, -3, -3, -4, -5, -6, -6, -7,
 -8, -8, -9, -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -9, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 5, 5, 6,
 7, 7, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6, 5, 4, 3, 3, 2, 1, 0, -1, -1, -2, -3, -4, -5, -5, -6, -7, -7, -8, -9,
 -9, -10, -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -10, -9, -9,
 -9, -9, -9, -9, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -6, -6, -6, -6, -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1,
 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13,
 -13, -13, -12, -12, -12, -12, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1,
 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15,
 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12,
 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5,
 -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15,
 -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14,
 -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6, -5, -5, -4, -4, -4, -3, -3, -2,
 -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14,
 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 7,
 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14, -14, -15, -15, -15, -15, -16, -16, -16, -16,
 -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -9, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4,
 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15,
 -15, -15, -15, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -12, -12, -12, -12, -11, -11, -11, -11, -10, -10, -10, -9, -9, -9, -8, -8, -8, -7, -7, -7, -6, -6, -6,
 -5, -5, -4, -4, -4, -3, -3, -2, -2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12,
 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8,
 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12,
 -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11,
 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -16, -16, -16, -16, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10,
 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0, 0, -1, -1, -2, -2, -3, -3, -4, -5, -5, -6, -6, -6, -7, -7, -8, -8, -9, -9, -9, -9, -10, -10, -10,
 -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, -7, -7, -6, -6, -6, -5, -5, -4, -3, -3, -2, -2,
 -1, -1, 0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14,
 14, 13, 13, 12, 12, 11, 10, 10, 9, 9, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -9, -9, -10, -10, -11, -12, -12, -13, -13, -14, -14,
 -14, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -14, -14, -14, -13, -13, -12, -12, -11, -10, -10, -9, -9, -8, -7, -7, -6,
 -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 9, 9, 10, 10, 11, 12, 12, 13, 13, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4,
 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11,
 -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, 16, 16, 16,
 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11,
 -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10,
 -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16,
 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9,
 -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12,
 -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15,
 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7, 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7,
 -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13,
 -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15,
 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14,
 -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -11, -12,
 -13, -14, -14, -15, -15, -15, -16, -16, -16, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -7, -7, -6, -5, -5, -4, -3, -2, -1, -1, 0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8,
 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 9, 9, 8, 7, 7, 6, 5, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -9,
 -10, -10, -11, -11, -11, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -9, -9, -8, -8, -7, -6, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 2, 3, 4, 5, 6, 6, 7, 8, 8, 9, 9,
 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
 11, 11, 11, 11, 11, 11, 11, 11, 11, -16, -16, -16, -16, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11,
 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12,
 -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9,
 10, 11, 12, 12, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12,
 12, 11, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6,
 -6, -6, -7, -7, -7, -8, -8, -8, -9, -9, -9, -10, -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, -13, -13, -13, -13, -14, -14, -14, -14, -14, -14, -15, -15, -15, -15,
 -15, -15, -15, -15, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7, -6, -4, -3, -2, -1, 0, 1,
 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5,
 -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -2,
 -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 14, 14, 13, 13, 12, 12, 12, 11, 10, 10, 9, 8, 8, 7, 7,
 6, 5, 4, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -4, -5, -6, -7, -7, -8, -8, -9, -10, -10, -11, -12, -12, -12, -13, -13, -14, -14, -15, -15, -15, -15, -15, -16, -16, -16, -16,
 -16, -16, -16, -16, -15, -15, -15, -15, -15, -14, -14, -13, -13, -12, -12, -12, -11, -10, -10, -9, -8, -8, -7, -7, -6, -5, -4, -4, -3, -2, -1, 0, 0, 1, 2, 3, 4, 4, 5, 6,
 7, 7, 8, 8, 9, 10, 10, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 15, 15, 16, 16, 16, 16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -11, -11, -10, -9, -8, -7,
 -6, -4, -3, -2, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3,
 2, 1, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -13, -14, -14, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -15, -14, -14, -13, -12, -12, -11, -10,
 -9, -8, -7, -6, -5, -4, -2, -1, 0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 11, 11, 10, 9, 8, 7,
 6, 4, 3, 2, 1, 0, -1, -2, -4, -5, -6, -7, -8, -9, -10, -11, -12, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, -16, -16, -16, -15, -15, -14, -14, -13, -13, -12, -11,
 -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 16, 16, 16, 16, 15, 15, 15, 14, 14, 13, 12, 12, 11, 10,
 9, 8, 7, 6, 5, 4, 2, 1, 0, -1, -2, -3, -4, -6, -7, -8, -9, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -16, -16, -16, 0,0,0,0,0,0,0,0,
};
#endif
#endif

const char* nalUnitTypeToString( NalUnitType type )
{
  switch( type )
  {
  case NAL_UNIT_CODED_SLICE_TRAIL:      return "TRAIL";
  case NAL_UNIT_CODED_SLICE_STSA:       return "STSA";
  case NAL_UNIT_CODED_SLICE_RADL:       return "RADL";
  case NAL_UNIT_CODED_SLICE_RASL:       return "RASL";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_GDR:        return "GDR";
  case NAL_UNIT_DCI:                    return "DCI";
  case NAL_UNIT_VPS:                    return "VPS";
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_PREFIX_APS:             return "Prefix APS";
  case NAL_UNIT_SUFFIX_APS:             return "Suffix APS";
  case NAL_UNIT_PH:                     return "PH";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  case NAL_UNIT_FD:                     return "FD";
  default:                              return "UNK";

  }
}

class ScanGenerator
{
private:
  uint32_t m_line, m_column;
  const uint32_t m_blockWidth, m_blockHeight;
  const uint32_t m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator( uint32_t blockWidth, uint32_t blockHeight, uint32_t stride, CoeffScanType scanType )
    : m_line( 0 ), m_column( 0 ), m_blockWidth( blockWidth ), m_blockHeight( blockHeight ), m_stride( stride ), m_scanType( scanType )
  {
  }

  uint32_t GetCurrentX() const { return m_column; }
  uint32_t GetCurrentY() const { return m_line; }

  uint32_t GetNextIndex( uint32_t blockOffsetX, uint32_t blockOffsetY )
  {
    const uint32_t rtn = ( ( m_line + blockOffsetY ) * m_stride ) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch( m_scanType )
    {
      //------------------------------------------------

    case SCAN_DIAG:

      if( ( m_column == m_blockWidth - 1 ) || ( m_line == 0 ) ) //if we reach the end of a rank, go diagonally down to the next one
      {
        m_line += m_column + 1;
        m_column = 0;

        if( m_line >= m_blockHeight ) //if that takes us outside the block, adjust so that we are back on the bottom row
        {
          m_column += m_line - ( m_blockHeight - 1 );
          m_line = m_blockHeight - 1;
        }
      }
      else
      {
        m_column++;
        m_line--;
      }
      break;

    case SCAN_TRAV_HOR:
      if( m_line % 2 == 0 )
      {
        if( m_column == ( m_blockWidth - 1 ) )
        {
          m_line++;
          m_column = m_blockWidth - 1;
        }
        else
        {
          m_column++;
        }
      }
      else
      {
        if( m_column == 0 )
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column--;
        }
      }
      break;

    case SCAN_TRAV_VER:
      if( m_column % 2 == 0 )
      {
        if( m_line == ( m_blockHeight - 1 ) )
        {
          m_column++;
          m_line = m_blockHeight - 1;
        }
        else
        {
          m_line++;
        }
      }
      else
      {
        if( m_line == 0 )
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line--;
        }
      }
      break;
      //------------------------------------------------

    default:

      THROW( "ERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex" );
      break;
    }

    return rtn;
  }
};
const int8_t g_bcwLog2WeightBase = 3;
const int8_t g_bcwWeightBase = ( 1 << g_bcwLog2WeightBase );
#if JVET_AB0079_TM_BCW_MRG
const int8_t g_bcwWeights[ BCW_NUM ] = { 1, 3, 4, 5, 7 };
const int8_t g_bcwMrgWeights[ BCW_MRG_NUM ] = { 1, 3, 4, 5, 7, 2, 6 };
int8_t g_bcwMrgParsingOrder[ BCW_MRG_NUM ];
#else
const int8_t g_bcwWeights[ BCW_NUM ] = { -2, 3, 4, 5, 10 };
#endif
const int8_t g_bcwSearchOrder[ BCW_NUM ] = { BCW_DEFAULT, BCW_DEFAULT - 2, BCW_DEFAULT + 2, BCW_DEFAULT - 1, BCW_DEFAULT + 1 };
int8_t g_bcwCodingOrder[ BCW_NUM ];
int8_t g_bcwParsingOrder[ BCW_NUM ];

int8_t getBcwWeight( uint8_t bcwIdx, uint8_t uhRefFrmList )
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
#if JVET_AB0079_TM_BCW_MRG
  return ( uhRefFrmList == REF_PIC_LIST_0 ? g_bcwWeightBase - g_bcwMrgWeights[ bcwIdx ] : g_bcwMrgWeights[ bcwIdx ] );
#else
  return ( uhRefFrmList == REF_PIC_LIST_0 ? g_bcwWeightBase - g_bcwWeights[ bcwIdx ] : g_bcwWeights[ bcwIdx ] );
#endif
}

void resetBcwCodingOrder( bool bRunDecoding, const CodingStructure& cs )
{
  // Form parsing order: { BCW_DEFAULT, BCW_DEFAULT+1, BCW_DEFAULT-1, BCW_DEFAULT+2, BCW_DEFAULT-2, ... }
  g_bcwParsingOrder[ 0 ] = BCW_DEFAULT;
  for( int i = 1; i <= ( BCW_NUM >> 1 ); ++i )
  {
    g_bcwParsingOrder[ 2 * i - 1 ] = BCW_DEFAULT + ( int8_t ) i;
    g_bcwParsingOrder[ 2 * i ] = BCW_DEFAULT - ( int8_t ) i;
  }
#if JVET_AB0079_TM_BCW_MRG
  for( int i = 0; i < BCW_NUM; i++ )
  {
    g_bcwMrgParsingOrder[ i ] = g_bcwParsingOrder[ i ];
  }
  for( int i = BCW_NUM; i < BCW_MRG_NUM; i++ )
  {
    g_bcwMrgParsingOrder[ i ] = i;
  }
#endif
  // Form encoding order
  if( !bRunDecoding )
  {
    for( int i = 0; i < BCW_NUM; ++i )
    {
      g_bcwCodingOrder[ ( uint32_t ) g_bcwParsingOrder[ i ] ] = i;
    }
  }
}

uint32_t deriveWeightIdxBits( uint8_t bcwIdx ) // Note: align this with TEncSbac::codeBcwIdx and TDecSbac::parseBcwIdx
{
  uint32_t numBits = 1;
  uint8_t  bcwCodingIdx = ( uint8_t ) g_bcwCodingOrder[ bcwIdx ];

  if( BCW_NUM > 2 && bcwCodingIdx != 0 )
  {
    uint32_t prefixNumBits = BCW_NUM - 2;
    uint32_t step = 1;
    uint8_t  prefixSymbol = bcwCodingIdx;

    // Truncated unary code
    uint8_t idx = 1;
    for( int ui = 0; ui < prefixNumBits; ++ui )
    {
      if( prefixSymbol == idx )
      {
        ++numBits;
        break;
      }
      else
      {
        ++numBits;
        idx += step;
      }
    }
  }
  return numBits;
}

uint32_t g_log2SbbSize[ MAX_CU_DEPTH + 1 ][ MAX_CU_DEPTH + 1 ][ 2 ] =
//===== luma/chroma =====
{
#if TU_256
  { { 0,0 },{ 0,1 },{ 0,2 },{ 0,3 },{ 0,4 },{ 0,4 },{ 0,4 },{ 0,4 },{ 0,4 } },
  { { 1,0 },{ 1,1 },{ 1,1 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 } },
  { { 2,0 },{ 1,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 3,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } }
#else
  { { 0,0 },{ 0,1 },{ 0,2 },{ 0,3 },{ 0,4 },{ 0,4 },{ 0,4 },{ 0,4 } },
  { { 1,0 },{ 1,1 },{ 1,1 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 },{ 1,3 } },
  { { 2,0 },{ 1,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 3,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } },
  { { 4,0 },{ 3,1 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 },{ 2,2 } }
#endif
};

#if TU_256
TMatrixCoeff g_trCoreDCT2P256[ 256 ][ 256 ];
TMatrixCoeff g_trCoreDCT8P256[ 256 ][ 256 ];
TMatrixCoeff g_trCoreDST7P256[ 256 ][ 256 ];
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
int g_bdofWeight[ 1600 ];
#endif
#if JVET_W0103_INTRA_MTS
TMatrixCoeff g_aiTr2[ NUM_TRANS_TYPE ][ 2 ][ 2 ];
TMatrixCoeff g_aiTr4[ NUM_TRANS_TYPE ][ 4 ][ 4 ];
TMatrixCoeff g_aiTr8[ NUM_TRANS_TYPE ][ 8 ][ 8 ];
TMatrixCoeff g_aiTr16[ NUM_TRANS_TYPE ][ 16 ][ 16 ];
TMatrixCoeff g_aiTr32[ NUM_TRANS_TYPE ][ 32 ][ 32 ];
TMatrixCoeff g_aiTr64[ NUM_TRANS_TYPE ][ 64 ][ 64 ];
TMatrixCoeff g_aiTr128[ NUM_TRANS_TYPE ][ 128 ][ 128 ];
TMatrixCoeff g_aiTr256[ NUM_TRANS_TYPE ][ 256 ][ 256 ];

#if JVET_AK0187_IMPLICIT_MTS_LUT_EXTENSION
const uint8_t g_aucIpmToTrSetModDimd[3][12][67] = {
{
{26, 0, 15, 15, 14, 14, 15, 14, 15, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 20, 20, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 15, 15, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 20, 14, 20, 14, 14, 20, 20},
{12, 0, 14, 14, 14, 15, 14, 20, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 20, 15, 14, 15, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 20, 14, 20, 20, 14, 14, 14, 20, 20, 20, 20, 14, 20, 14, 20, 15, 14, 20, 14, 14, 14, 14, 15, 14, 20, 21},
{2, 5, 14, 20, 20, 21, 20, 3, 20, 20, 20, 21, 2, 2, 15, 14, 20, 21, 20, 18, 20, 21, 21, 15, 20, 14, 14, 20, 20, 14, 20, 20, 12, 14, 20, 20, 3, 20, 14, 2, 20, 2, 14, 2, 20, 4, 18, 3, 22, 22, 20, 3, 20, 20, 2, 21, 20, 14, 21, 15, 20, 20, 20, 14, 22, 18, 20},
{4, 12, 34, 22, 2, 21, 3, 2, 3, 2, 20, 20, 2, 21, 5, 23, 21, 0, 3, 2, 0, 20, 21, 3, 3, 15, 20, 2, 3, 2, 15, 3, 21, 12, 4, 8, 20, 20, 27, 22, 34, 21, 2, 22, 2, 18, 22, 4, 4, 4, 2, 4, 22, 20, 3, 20, 2, 21, 8, 4, 20, 4, 20, 13, 2, 20, 21},
{34, 0, 0, 14, 14, 15, 15, 15, 12, 14, 14, 12, 14, 14, 15, 14, 12, 0, 15, 12, 12, 0, 15, 14, 15, 14, 15, 21, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 21, 20, 14, 20, 14, 20, 0, 2, 2, 20, 0, 2, 14, 20, 14, 14, 2, 14, 14, 2, 20, 20, 20, 14, 14, 0},
{2, 21, 12, 20, 20, 12, 20, 14, 3, 21, 21, 15, 15, 3, 3, 3, 15, 2, 3, 2, 21, 20, 21, 20, 20, 20, 14, 14, 20, 20, 14, 21, 20, 15, 2, 20, 20, 16, 20, 20, 21, 14, 21, 2, 2, 2, 20, 20, 20, 21, 20, 21, 20, 20, 20, 21, 20, 20, 20, 14, 2, 20, 20, 12, 20, 20, 32},
{15, 18, 14, 20, 20, 15, 2, 20, 21, 2, 21, 18, 0, 2, 3, 3, 0, 3, 3, 2, 21, 21, 3, 20, 20, 2, 14, 2, 2, 20, 20, 20, 15, 2, 18, 3, 2, 14, 27, 2, 22, 2, 14, 20, 20, 20, 20, 20, 2, 3, 21, 4, 2, 2, 18, 20, 20, 18, 26, 20, 21, 20, 12, 14, 2, 20, 2},
{26, 9, 14, 20, 3, 15, 18, 14, 0, 21, 15, 20, 21, 21, 21, 3, 12, 0, 18, 3, 21, 12, 18, 15, 20, 14, 14, 20, 3, 20, 15, 20, 12, 21, 0, 21, 2, 15, 20, 15, 18, 15, 14, 14, 15, 20, 3, 2, 21, 18, 3, 0, 2, 18, 21, 21, 21, 15, 20, 21, 0, 14, 3, 20, 18, 15, 14},
{12, 8, 0, 4, 21, 12, 0, 3, 15, 2, 0, 21, 21, 20, 21, 21, 12, 0, 3, 0, 15, 18, 12, 21, 20, 0, 0, 12, 20, 2, 0, 2, 12, 21, 0, 2, 2, 2, 3, 2, 20, 20, 20, 2, 2, 3, 2, 20, 20, 2, 2, 18, 0, 2, 3, 2, 2, 14, 21, 2, 12, 14, 20, 14, 2, 3, 18},
{28, 14, 21, 12, 12, 0, 2, 12, 18, 0, 18, 12, 12, 0, 15, 3, 0, 21, 0, 18, 12, 3, 0, 12, 12, 0, 2, 0, 0, 18, 0, 0, 12, 0, 0, 0, 2, 0, 0, 3, 0, 0, 12, 0, 2, 2, 0, 18, 2, 3, 0, 21, 0, 18, 20, 0, 2, 2, 3, 0, 3, 2, 12, 0, 2, 2, 21},
{21, 21, 0, 14, 12, 14, 0, 12, 12, 12, 12, 12, 12, 12, 0, 18, 18, 18, 0, 18, 15, 12, 12, 12, 14, 2, 14, 14, 14, 14, 14, 15, 14, 14, 18, 14, 14, 0, 18, 12, 18, 12, 12, 14, 14, 14, 18, 14, 18, 3, 0, 18, 18, 22, 3, 3, 15, 15, 14, 2, 12, 2, 14, 4, 14, 33, 20},
{2, 12, 0, 1, 12, 0, 3, 12, 24, 12, 12, 12, 14, 12, 12, 0, 18, 0, 12, 15, 21, 0, 14, 14, 2, 0, 14, 20, 18, 14, 0, 0, 2, 35, 0, 0, 14, 14, 0, 0, 18, 12, 34, 21, 15, 0, 16, 20, 20, 18, 18, 0, 3, 20, 28, 14, 2, 8, 22, 14, 4, 12, 0, 0, 2, 0, 15}
},
{
{12, 0, 21, 14, 14, 14, 15, 23, 23, 14, 14, 14, 14, 23, 14, 21, 23, 20, 14, 15, 23, 20, 14, 14, 14, 15, 15, 21, 14, 14, 15, 14, 15, 20, 14, 15, 20, 14, 20, 14, 14, 21, 20, 20, 14, 14, 14, 15, 33, 20, 14, 15, 33, 21, 14, 33, 14, 14, 14, 14, 33, 33, 20, 14, 14, 14, 21},
{0, 0, 21, 14, 5, 14, 22, 23, 18, 14, 14, 14, 20, 20, 21, 14, 14, 15, 20, 2, 14, 18, 14, 15, 12, 14, 14, 21, 21, 15, 21, 14, 17, 21, 21, 14, 2, 14, 2, 2, 15, 3, 2, 4, 20, 15, 20, 20, 14, 2, 14, 20, 21, 12, 2, 15, 15, 14, 14, 22, 14, 14, 5, 14, 2, 14, 14},
{0, 3, 4, 3, 14, 2, 9, 3, 3, 21, 15, 2, 12, 23, 2, 20, 22, 18, 18, 18, 21, 23, 17, 15, 15, 20, 15, 16, 21, 22, 12, 5, 2, 20, 23, 30, 3, 3, 20, 1, 16, 12, 2, 4, 2, 3, 2, 9, 20, 29, 21, 3, 4, 4, 20, 3, 2, 4, 33, 26, 15, 22, 1, 18, 15, 22, 32},
{0, 23, 32, 9, 26, 12, 34, 34, 34, 0, 32, 10, 26, 18, 20, 0, 32, 0, 20, 14, 19, 3, 18, 2, 35, 11, 5, 5, 3, 4, 0, 27, 27, 31, 16, 27, 9, 4, 3, 4, 8, 3, 3, 4, 4, 23, 3, 18, 14, 32, 20, 28, 32, 10, 2, 2, 3, 18, 14, 9, 9, 20, 28, 4, 33, 18, 33},
{0, 0, 3, 0, 0, 20, 2, 12, 15, 12, 14, 2, 2, 0, 0, 15, 14, 0, 21, 12, 15, 12, 18, 14, 15, 21, 14, 14, 21, 14, 15, 14, 14, 3, 15, 18, 14, 14, 20, 14, 21, 14, 14, 21, 20, 14, 3, 2, 20, 2, 21, 0, 14, 20, 0, 0, 12, 12, 14, 2, 20, 2, 12, 15, 0, 0, 18},
{0, 18, 12, 16, 15, 14, 17, 12, 2, 15, 14, 3, 14, 3, 3, 12, 14, 3, 2, 21, 14, 14, 14, 21, 14, 20, 26, 20, 9, 21, 2, 20, 3, 23, 2, 18, 2, 16, 20, 21, 2, 14, 0, 3, 3, 2, 20, 4, 20, 2, 20, 4, 4, 12, 20, 21, 22, 23, 2, 15, 20, 14, 32, 22, 21, 21, 22},
{0, 20, 12, 11, 29, 26, 6, 23, 20, 4, 14, 18, 20, 3, 1, 0, 3, 22, 3, 34, 4, 12, 0, 20, 2, 3, 16, 6, 1, 32, 27, 24, 15, 21, 18, 30, 2, 9, 12, 20, 18, 16, 2, 13, 9, 14, 3, 23, 2, 20, 0, 0, 26, 30, 20, 21, 15, 14, 20, 2, 15, 2, 18, 8, 20, 34, 16},
{0, 24, 12, 18, 2, 14, 14, 15, 15, 20, 25, 0, 21, 20, 16, 20, 14, 18, 18, 27, 31, 21, 15, 21, 20, 14, 12, 18, 0, 12, 8, 32, 14, 2, 21, 12, 14, 17, 13, 2, 0, 3, 2, 14, 15, 21, 20, 21, 11, 22, 3, 3, 14, 15, 26, 15, 21, 0, 10, 15, 20, 20, 14, 14, 12, 3, 2},
{0, 14, 27, 23, 6, 0, 34, 21, 0, 12, 14, 20, 20, 18, 18, 21, 1, 12, 3, 23, 0, 24, 1, 22, 2, 14, 14, 3, 0, 2, 20, 0, 12, 18, 3, 7, 20, 15, 6, 0, 4, 3, 22, 18, 21, 3, 15, 20, 3, 6, 21, 2, 2, 26, 16, 20, 20, 26, 2, 21, 2, 14, 24, 2, 20, 4, 20},
{0, 3, 21, 15, 12, 12, 14, 12, 2, 12, 15, 0, 0, 20, 15, 20, 0, 18, 0, 3, 12, 3, 14, 0, 0, 19, 20, 14, 0, 0, 14, 18, 2, 14, 21, 14, 12, 3, 14, 0, 0, 14, 15, 9, 0, 0, 14, 18, 2, 18, 0, 3, 0, 15, 20, 15, 0, 0, 20, 2, 12, 2, 14, 2, 2, 20, 21},
{0, 6, 0, 14, 12, 26, 0, 14, 15, 12, 2, 12, 14, 12, 14, 0, 12, 3, 0, 0, 12, 14, 12, 12, 2, 14, 2, 14, 15, 12, 14, 0, 0, 15, 18, 14, 14, 14, 10, 12, 12, 14, 14, 12, 14, 2, 0, 2, 0, 6, 0, 9, 0, 0, 15, 2, 2, 0, 0, 14, 13, 8, 0, 2, 14, 28, 0},
{0, 3, 0, 0, 15, 0, 14, 12, 23, 14, 24, 16, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 14, 14, 0, 0, 14, 21, 24, 24, 0, 14, 0, 0, 0, 12, 3, 20, 15, 21, 21, 12, 35, 12, 12, 0, 0, 0, 21, 2, 0, 0, 0, 14, 20, 26, 0, 34, 0, 18, 0, 0, 0, 2, 0, 18}
},
{
{26, 0, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 20, 14, 21, 20, 15, 14, 20, 15, 15, 14, 20, 15, 14, 15, 14, 14, 14, 14, 14, 20, 20, 15, 14, 14, 14, 14, 14, 20, 14, 20, 15, 14, 20, 20, 15, 14, 20, 15, 21, 14, 15, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14},
{0, 0, 14, 14, 20, 15, 14, 12, 20, 15, 15, 14, 14, 15, 20, 14, 12, 21, 2, 21, 18, 23, 18, 20, 15, 14, 15, 15, 17, 14, 14, 14, 14, 20, 15, 14, 14, 15, 15, 14, 14, 20, 14, 14, 14, 2, 20, 20, 14, 2, 15, 21, 20, 2, 14, 20, 18, 14, 20, 15, 14, 15, 14, 14, 14, 14, 14},
{0, 20, 22, 21, 2, 2, 23, 20, 2, 2, 20, 14, 15, 22, 21, 23, 2, 21, 18, 21, 3, 12, 18, 27, 18, 15, 28, 20, 22, 24, 20, 2, 2, 22, 3, 3, 17, 20, 20, 4, 21, 27, 18, 2, 20, 0, 0, 3, 3, 20, 21, 20, 20, 23, 21, 20, 15, 5, 17, 22, 12, 21, 20, 2, 2, 17, 15},
{0, 22, 3, 2, 21, 18, 32, 20, 2, 0, 22, 21, 0, 0, 5, 22, 14, 3, 2, 21, 18, 4, 26, 20, 22, 15, 15, 0, 32, 2, 15, 20, 25, 20, 12, 5, 20, 1, 30, 20, 3, 22, 21, 26, 15, 20, 4, 23, 5, 23, 23, 21, 2, 18, 2, 1, 3, 11, 16, 2, 4, 2, 21, 21, 21, 27, 12},
{0, 0, 15, 12, 15, 14, 3, 15, 20, 0, 2, 0, 14, 12, 3, 20, 3, 0, 0, 0, 0, 3, 20, 12, 15, 12, 20, 18, 12, 0, 0, 14, 14, 0, 20, 0, 14, 14, 0, 0, 2, 3, 15, 2, 20, 2, 15, 18, 0, 0, 0, 0, 18, 15, 18, 2, 14, 0, 12, 0, 15, 20, 18, 14, 20, 2, 20},
{0, 23, 20, 14, 14, 21, 5, 20, 3, 20, 14, 15, 21, 20, 21, 21, 2, 20, 21, 3, 21, 12, 15, 3, 21, 12, 20, 20, 0, 2, 3, 15, 20, 15, 21, 15, 20, 21, 14, 21, 21, 14, 3, 20, 20, 20, 21, 21, 15, 20, 21, 15, 2, 14, 14, 14, 20, 3, 21, 18, 21, 20, 20, 2, 20, 2, 21},
{0, 20, 2, 14, 15, 20, 20, 21, 30, 2, 20, 14, 18, 2, 12, 2, 14, 22, 2, 3, 2, 33, 32, 21, 15, 3, 8, 21, 0, 3, 35, 10, 21, 22, 3, 35, 8, 27, 18, 20, 21, 15, 21, 3, 11, 2, 2, 31, 2, 10, 3, 3, 1, 15, 0, 2, 3, 3, 2, 2, 13, 2, 18, 2, 2, 2, 3},
{0, 0, 21, 28, 3, 20, 12, 20, 2, 20, 15, 0, 21, 18, 15, 15, 18, 20, 21, 21, 4, 0, 0, 2, 21, 2, 20, 2, 17, 20, 20, 2, 0, 26, 0, 16, 0, 12, 15, 15, 32, 12, 15, 12, 21, 12, 0, 0, 24, 21, 21, 15, 3, 20, 20, 3, 21, 0, 20, 15, 12, 15, 2, 15, 18, 28, 21},
{0, 3, 3, 26, 2, 21, 14, 17, 18, 2, 21, 18, 21, 0, 15, 2, 21, 21, 3, 21, 20, 12, 18, 3, 2, 34, 12, 23, 21, 3, 15, 24, 12, 0, 18, 21, 14, 12, 20, 20, 20, 2, 34, 14, 20, 0, 18, 4, 21, 15, 0, 21, 2, 3, 5, 19, 15, 14, 12, 17, 14, 12, 27, 31, 0, 14, 3},
{0, 18, 3, 20, 21, 0, 12, 0, 14, 31, 14, 21, 0, 0, 21, 22, 8, 15, 0, 3, 18, 30, 0, 0, 12, 0, 3, 0, 18, 2, 12, 2, 14, 0, 18, 0, 14, 12, 2, 12, 3, 0, 18, 0, 2, 0, 0, 5, 3, 18, 0, 20, 13, 27, 21, 0, 0, 21, 14, 11, 14, 0, 2, 0, 21, 15, 18},
{0, 21, 30, 12, 14, 12, 12, 0, 15, 12, 0, 2, 20, 18, 2, 27, 25, 14, 0, 14, 27, 24, 0, 12, 14, 14, 12, 14, 14, 14, 14, 0, 12, 26, 15, 14, 12, 4, 14, 12, 14, 21, 14, 14, 2, 12, 16, 21, 7, 21, 0, 24, 18, 28, 15, 18, 14, 17, 3, 14, 18, 14, 8, 12, 12, 26, 12},
{0, 0, 0, 20, 14, 0, 18, 12, 18, 12, 12, 2, 3, 15, 0, 12, 9, 0, 0, 4, 18, 18, 0, 17, 14, 0, 0, 15, 0, 16, 0, 0, 14, 0, 12, 0, 12, 27, 0, 14, 0, 0, 26, 0, 14, 0, 26, 3, 18, 20, 0, 0, 0, 28, 0, 20, 26, 0, 28, 0, 0, 0, 0, 0, 12, 0, 0}
}
};
const uint8_t g_aucIpmToTrSetModTimd[3][10][67] = {
{
{12, 20, 12, 15, 14, 15, 15, 14, 20, 20, 14, 14, 14, 14, 15, 20, 20, 15, 15, 15, 14, 15, 12, 15, 15, 15, 14, 15, 15, 20, 15, 14, 14, 14, 21, 14, 14, 14, 20, 15, 20, 20, 14, 20, 20, 20, 2, 20, 14, 20, 20, 20, 15, 15, 20, 14, 14, 14, 14, 15, 15, 14, 20, 20, 14, 20, 2},
{3, 20, 14, 20, 15, 2, 15, 2, 20, 21, 20, 3, 15, 15, 15, 15, 15, 21, 21, 21, 20, 14, 18, 14, 14, 14, 14, 14, 14, 14, 20, 14, 20, 20, 21, 21, 21, 20, 20, 20, 20, 21, 2, 20, 21, 20, 20, 2, 2, 2, 20, 20, 20, 20, 20, 20, 20, 20, 20, 2, 3, 20, 20, 21, 20, 20, 21},
{0, 3, 12, 21, 18, 2, 20, 18, 21, 18, 0, 3, 18, 3, 21, 21, 15, 18, 21, 18, 21, 18, 21, 2, 20, 21, 20, 20, 20, 20, 20, 3, 2, 20, 2, 20, 20, 21, 3, 21, 3, 20, 20, 20, 2, 2, 20, 21, 0, 2, 20, 20, 20, 3, 21, 2, 20, 2, 20, 20, 20, 2, 3, 20, 3, 3, 2},
{0, 0, 21, 21, 2, 2, 21, 2, 3, 3, 3, 2, 21, 2, 0, 2, 3, 15, 18, 21, 2, 0, 0, 21, 21, 3, 2, 21, 2, 21, 21, 21, 21, 23, 4, 21, 20, 26, 2, 22, 3, 2, 18, 18, 21, 3, 2, 2, 20, 21, 2, 22, 2, 3, 2, 20, 20, 22, 2, 15, 2, 21, 20, 20, 15, 3, 18},
{0, 0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 12, 0, 12, 12, 0, 0, 0, 0, 0, 0, 0, 12, 12, 15, 0, 15, 12, 3, 15, 15, 15, 21, 15, 20, 20, 21, 20, 20, 20, 18, 2, 20, 0, 20, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2, 2, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 2},
{0, 21, 3, 21, 21, 3, 21, 21, 21, 3, 3, 0, 3, 21, 21, 15, 15, 15, 21, 21, 21, 15, 3, 3, 21, 21, 21, 21, 20, 3, 21, 20, 20, 21, 20, 20, 20, 20, 20, 20, 20, 21, 20, 2, 20, 21, 21, 3, 3, 20, 21, 21, 21, 20, 3, 3, 20, 2, 18, 20, 20, 21, 3, 21, 2, 2, 21},
{0, 0, 0, 3, 18, 2, 3, 0, 20, 3, 3, 3, 18, 3, 0, 3, 18, 3, 15, 0, 18, 0, 0, 3, 3, 18, 21, 3, 20, 20, 20, 2, 20, 20, 2, 18, 3, 21, 0, 21, 2, 2, 3, 18, 20, 2, 3, 21, 3, 3, 3, 2, 3, 3, 3, 18, 21, 2, 3, 20, 20, 2, 21, 20, 20, 20, 0},
{0, 21, 12, 3, 3, 15, 21, 3, 0, 3, 18, 18, 21, 18, 21, 0, 12, 21, 21, 21, 21, 18, 21, 21, 21, 3, 18, 15, 21, 21, 15, 12, 21, 2, 21, 12, 21, 2, 20, 21, 21, 20, 3, 18, 21, 21, 21, 3, 21, 21, 21, 21, 2, 0, 21, 3, 21, 3, 3, 18, 0, 18, 21, 20, 18, 18, 2},
{0, 21, 3, 3, 0, 21, 21, 3, 3, 3, 3, 3, 3, 21, 0, 3, 21, 21, 3, 18, 21, 18, 3, 15, 18, 18, 21, 21, 21, 3, 14, 20, 14, 20, 20, 2, 2, 14, 21, 20, 20, 2, 2, 2, 2, 21, 3, 3, 4, 3, 20, 20, 0, 0, 20, 2, 0, 0, 20, 2, 18, 0, 0, 0, 3, 2, 3},
{0, 3, 0, 0, 3, 0, 0, 0, 0, 3, 0, 3, 3, 12, 0, 3, 21, 12, 12, 18, 18, 21, 0, 18, 0, 0, 12, 0, 0, 0, 0, 3, 0, 0, 14, 0, 0, 18, 0, 0, 0, 0, 2, 0, 0, 3, 0, 21, 3, 3, 2, 2, 21, 18, 0, 2, 18, 18, 0, 18, 0, 0, 0, 0, 18, 0, 0}
},
{
{15, 20, 18, 18, 20, 14, 15, 20, 20, 14, 14, 15, 12, 12, 15, 15, 15, 20, 15, 20, 20, 18, 20, 18, 17, 20, 18, 14, 17, 15, 3, 14, 14, 20, 20, 15, 14, 14, 18, 20, 32, 14, 3, 15, 32, 3, 15, 3, 15, 15, 20, 15, 20, 20, 20, 2, 2, 20, 14, 14, 15, 15, 20, 14, 15, 3, 3},
{15, 21, 21, 18, 3, 0, 23, 3, 14, 14, 15, 15, 15, 15, 14, 15, 15, 21, 20, 15, 15, 15, 20, 17, 17, 3, 21, 14, 18, 15, 20, 23, 12, 14, 20, 20, 2, 3, 0, 2, 20, 20, 3, 3, 2, 2, 2, 3, 14, 3, 21, 2, 20, 20, 20, 21, 2, 2, 20, 14, 12, 15, 2, 14, 14, 20, 21},
{21, 20, 18, 0, 18, 2, 26, 21, 2, 2, 5, 18, 0, 2, 15, 15, 15, 27, 21, 3, 5, 14, 2, 20, 3, 19, 16, 20, 33, 26, 12, 20, 18, 15, 4, 18, 12, 32, 5, 20, 33, 4, 34, 3, 8, 0, 21, 21, 22, 2, 20, 2, 20, 21, 21, 20, 21, 2, 20, 3, 33, 21, 21, 29, 0, 19, 14},
{2, 2, 34, 13, 5, 21, 2, 23, 28, 15, 3, 4, 0, 21, 21, 18, 18, 20, 21, 5, 5, 2, 23, 24, 3, 10, 18, 17, 9, 3, 3, 8, 6, 20, 4, 21, 34, 2, 18, 26, 10, 22, 2, 3, 21, 2, 21, 0, 14, 20, 3, 34, 22, 21, 2, 21, 20, 2, 3, 21, 18, 9, 9, 14, 16, 28, 31},
{0, 0, 14, 0, 15, 5, 0, 0, 0, 0, 0, 0, 0, 12, 0, 0, 0, 12, 15, 12, 0, 2, 0, 0, 12, 2, 2, 2, 20, 0, 12, 5, 2, 15, 12, 20, 12, 30, 2, 0, 15, 12, 12, 12, 2, 0, 0, 12, 0, 2, 20, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 30, 20, 0, 14},
{3, 18, 21, 12, 15, 18, 12, 24, 20, 3, 21, 3, 0, 3, 12, 15, 21, 15, 15, 3, 25, 4, 4, 3, 18, 9, 0, 20, 14, 21, 3, 14, 15, 0, 0, 4, 12, 30, 18, 12, 14, 2, 0, 3, 20, 3, 14, 3, 32, 2, 21, 20, 21, 20, 21, 21, 2, 20, 20, 21, 2, 26, 2, 14, 18, 14, 21},
{0, 3, 18, 22, 0, 24, 27, 0, 27, 2, 2, 18, 3, 12, 3, 18, 12, 20, 18, 3, 21, 5, 29, 20, 33, 11, 23, 13, 21, 18, 2, 21, 3, 4, 3, 11, 3, 18, 18, 33, 3, 21, 3, 6, 20, 14, 0, 26, 32, 4, 2, 20, 2, 3, 21, 3, 3, 2, 21, 33, 4, 22, 21, 2, 2, 1, 16},
{21, 21, 26, 21, 24, 20, 14, 2, 3, 12, 18, 12, 21, 12, 12, 15, 21, 18, 18, 18, 18, 27, 3, 3, 21, 32, 0, 15, 3, 21, 2, 15, 18, 17, 12, 32, 3, 20, 12, 21, 18, 20, 0, 17, 21, 18, 18, 22, 3, 3, 3, 3, 21, 20, 2, 2, 21, 2, 3, 2, 18, 12, 14, 15, 4, 21, 16},
{0, 21, 2, 18, 14, 21, 15, 12, 12, 0, 3, 3, 21, 15, 3, 0, 15, 20, 18, 21, 13, 27, 2, 3, 0, 22, 0, 2, 0, 3, 21, 15, 19, 2, 4, 20, 3, 15, 5, 21, 3, 0, 12, 0, 21, 2, 18, 2, 21, 20, 2, 3, 21, 2, 20, 4, 18, 21, 2, 14, 14, 0, 21, 16, 21, 4, 3},
{3, 18, 18, 0, 14, 0, 3, 18, 12, 12, 12, 0, 4, 21, 21, 3, 0, 0, 18, 15, 3, 20, 18, 12, 12, 25, 2, 0, 18, 14, 12, 2, 18, 14, 2, 14, 3, 12, 2, 14, 3, 0, 12, 10, 2, 2, 3, 15, 18, 20, 3, 0, 0, 18, 21, 21, 24, 0, 2, 2, 2, 3, 18, 0, 14, 0, 3}
},
{
{14, 20, 15, 14, 20, 15, 20, 14, 14, 20, 15, 14, 20, 20, 15, 14, 15, 15, 20, 20, 14, 15, 15, 20, 14, 20, 15, 14, 14, 20, 15, 14, 14, 14, 21, 14, 14, 14, 20, 15, 14, 14, 20, 15, 14, 15, 20, 20, 14, 15, 15, 20, 20, 14, 20, 15, 15, 14, 20, 15, 14, 14, 15, 20, 15, 14, 20},
{20, 20, 20, 14, 20, 20, 2, 2, 20, 20, 15, 14, 15, 14, 15, 15, 14, 20, 21, 21, 15, 20, 14, 15, 15, 15, 15, 14, 15, 15, 20, 21, 15, 21, 21, 20, 21, 20, 21, 14, 21, 20, 21, 21, 21, 2, 20, 2, 2, 21, 21, 20, 20, 20, 20, 20, 14, 20, 20, 20, 20, 15, 2, 15, 20, 15, 15},
{3, 2, 20, 14, 20, 2, 2, 22, 20, 3, 18, 2, 23, 3, 3, 21, 21, 21, 21, 18, 18, 12, 21, 14, 3, 15, 20, 20, 21, 21, 20, 21, 21, 20, 3, 21, 20, 20, 3, 21, 2, 2, 2, 20, 21, 20, 3, 20, 20, 3, 3, 21, 21, 21, 21, 21, 2, 2, 20, 21, 15, 3, 2, 4, 2, 3, 20},
{3, 3, 2, 21, 20, 14, 2, 2, 2, 3, 2, 18, 2, 2, 21, 2, 3, 18, 21, 20, 20, 2, 2, 14, 2, 21, 2, 20, 4, 21, 2, 0, 33, 20, 4, 4, 3, 2, 2, 2, 4, 20, 2, 3, 0, 21, 3, 2, 3, 3, 4, 2, 2, 2, 2, 18, 2, 17, 21, 3, 3, 18, 20, 14, 22, 3, 3},
{0, 0, 2, 0, 2, 12, 0, 0, 0, 12, 12, 0, 12, 15, 15, 15, 12, 0, 0, 0, 0, 0, 12, 12, 15, 14, 14, 18, 21, 21, 20, 14, 14, 15, 15, 20, 14, 14, 15, 21, 21, 3, 14, 14, 20, 2, 2, 0, 0, 0, 0, 0, 2, 20, 20, 20, 2, 0, 2, 2, 0, 0, 0, 2, 12, 0, 12},
{3, 3, 20, 3, 20, 3, 20, 2, 21, 18, 21, 20, 3, 3, 15, 21, 15, 15, 21, 15, 15, 12, 21, 21, 21, 15, 20, 20, 21, 20, 21, 20, 20, 20, 2, 20, 21, 20, 20, 2, 20, 3, 2, 2, 2, 21, 20, 3, 3, 20, 3, 20, 20, 20, 21, 20, 3, 4, 20, 20, 2, 3, 2, 20, 21, 3, 21},
{3, 3, 2, 2, 3, 20, 2, 3, 2, 0, 3, 2, 12, 3, 3, 2, 3, 15, 3, 15, 3, 3, 3, 3, 3, 2, 2, 0, 3, 18, 3, 2, 3, 2, 3, 2, 18, 20, 20, 2, 4, 3, 2, 2, 2, 20, 3, 21, 3, 3, 3, 21, 3, 22, 2, 0, 21, 5, 5, 14, 20, 20, 18, 14, 0, 20, 2},
{21, 0, 21, 3, 20, 21, 15, 18, 18, 21, 18, 3, 21, 21, 15, 15, 15, 21, 21, 18, 21, 12, 15, 18, 18, 21, 15, 15, 14, 15, 15, 21, 15, 15, 14, 20, 20, 21, 20, 20, 14, 20, 20, 21, 3, 3, 20, 2, 21, 3, 21, 21, 20, 20, 20, 21, 21, 18, 3, 21, 3, 3, 20, 21, 15, 18, 21},
{3, 21, 3, 21, 2, 21, 2, 2, 3, 3, 18, 3, 21, 3, 21, 3, 21, 15, 3, 21, 21, 3, 18, 0, 21, 20, 20, 21, 21, 21, 3, 18, 20, 21, 2, 21, 20, 21, 2, 2, 4, 3, 0, 2, 21, 3, 2, 21, 20, 3, 2, 2, 2, 21, 0, 21, 2, 20, 2, 2, 3, 0, 0, 2, 3, 2, 20},
{0, 0, 12, 3, 0, 3, 0, 4, 18, 15, 21, 3, 0, 21, 18, 3, 3, 0, 18, 3, 3, 18, 0, 0, 0, 15, 18, 18, 0, 18, 14, 18, 0, 0, 18, 0, 0, 3, 14, 3, 0, 3, 3, 20, 0, 0, 0, 3, 18, 18, 3, 0, 18, 18, 3, 21, 0, 18, 21, 20, 3, 24, 0, 18, 0, 18, 2}
}
};
const uint8_t g_aucIpmToTrSetModMip[3][12][67] = {
{
{0, 0, 18, 2, 20, 20, 20, 12, 20, 0, 23, 20, 18, 18, 23, 3, 18, 23, 18, 0, 0, 23, 18, 18, 23, 18, 21, 21, 20, 14, 20, 20, 21, 15, 15, 20, 21, 15, 15, 14, 15, 21, 21, 3, 33, 3, 3, 33, 0, 0, 3, 33, 3, 18, 33, 3, 3, 15, 33, 0, 15, 2, 15, 15, 15, 12, 3},
{18, 0, 21, 2, 21, 20, 14, 21, 21, 18, 3, 21, 23, 23, 23, 23, 21, 20, 20, 23, 23, 18, 20, 23, 15, 15, 15, 20, 21, 15, 20, 21, 21, 15, 21, 15, 21, 21, 21, 21, 15, 2, 21, 15, 15, 21, 21, 21, 20, 3, 21, 2, 17, 21, 3, 3, 21, 15, 20, 21, 21, 20, 21, 14, 15, 15, 0},
{32, 0, 21, 15, 29, 20, 3, 15, 4, 4, 3, 2, 23, 21, 0, 14, 14, 21, 15, 15, 21, 3, 17, 21, 14, 21, 23, 21, 21, 2, 2, 5, 18, 22, 3, 20, 2, 0, 12, 2, 14, 0, 3, 20, 15, 14, 18, 5, 18, 20, 12, 23, 16, 32, 4, 2, 12, 3, 20, 2, 2, 3, 5, 18, 2, 20, 15},
{24, 0, 31, 31, 5, 1, 0, 23, 4, 4, 0, 2, 2, 12, 28, 21, 13, 18, 1, 0, 21, 15, 15, 21, 19, 23, 14, 23, 12, 1, 3, 18, 28, 14, 14, 35, 3, 5, 21, 18, 21, 26, 14, 14, 3, 15, 23, 12, 21, 30, 3, 15, 30, 20, 15, 0, 17, 3, 21, 3, 20, 5, 30, 34, 22, 3, 17},
{0, 0, 2, 21, 20, 20, 15, 14, 21, 15, 21, 21, 20, 21, 21, 18, 21, 15, 18, 15, 15, 15, 15, 15, 15, 12, 21, 21, 21, 15, 21, 21, 21, 21, 15, 21, 21, 21, 21, 20, 21, 21, 21, 2, 20, 20, 20, 20, 20, 20, 3, 20, 21, 3, 21, 21, 15, 21, 21, 20, 21, 14, 20, 15, 15, 21, 12},
{0, 0, 20, 20, 20, 21, 20, 15, 21, 21, 21, 20, 21, 21, 21, 15, 21, 20, 21, 21, 15, 21, 21, 20, 21, 21, 21, 20, 21, 18, 20, 15, 21, 14, 21, 18, 21, 3, 20, 2, 21, 21, 21, 21, 21, 18, 2, 2, 3, 2, 20, 20, 3, 20, 21, 20, 12, 21, 21, 21, 20, 21, 21, 3, 14, 21, 26},
{0, 0, 18, 3, 18, 4, 15, 15, 21, 21, 18, 0, 20, 23, 14, 21, 21, 3, 0, 3, 20, 12, 23, 18, 13, 3, 21, 3, 26, 33, 21, 5, 15, 32, 3, 3, 18, 5, 21, 2, 2, 12, 20, 32, 18, 3, 23, 21, 3, 0, 2, 21, 23, 3, 3, 3, 3, 21, 23, 21, 5, 21, 20, 3, 2, 23, 3},
{0, 0, 15, 20, 14, 20, 14, 15, 14, 14, 14, 14, 14, 14, 14, 12, 12, 15, 12, 12, 21, 15, 14, 14, 20, 2, 12, 14, 21, 15, 20, 14, 2, 15, 3, 20, 12, 14, 15, 20, 21, 14, 2, 12, 15, 14, 14, 20, 21, 2, 2, 20, 2, 2, 14, 14, 14, 14, 14, 14, 14, 20, 14, 15, 14, 15, 20},
{0, 0, 21, 14, 14, 14, 20, 14, 21, 14, 14, 14, 14, 14, 15, 14, 14, 20, 21, 21, 14, 14, 14, 14, 20, 20, 14, 14, 26, 15, 14, 14, 14, 14, 3, 14, 14, 18, 3, 3, 3, 20, 3, 21, 14, 2, 2, 0, 18, 20, 0, 23, 18, 0, 21, 3, 14, 20, 3, 2, 21, 20, 2, 20, 14, 12, 8},
{0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 12, 14, 12, 0, 12, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 2, 0, 2, 14, 2, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14},
{18, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 12, 0, 0, 2, 2, 0, 2, 2, 2, 2, 2, 2, 2, 0, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
{22, 0, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 2, 2, 0, 2, 2, 0, 0, 2, 2, 2, 3, 3, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}
},
{
{0, 0, 0, 5, 23, 18, 18, 3, 18, 18, 0, 3, 18, 18, 23, 12, 5, 12, 12, 18, 18, 2, 35, 0, 23, 17, 23, 33, 18, 20, 15, 23, 15, 15, 32, 20, 20, 33, 20, 15, 3, 23, 33, 32, 33, 0, 35, 12, 3, 3, 2, 2, 30, 2, 33, 3, 3, 18, 0, 3, 3, 18, 3, 3, 33, 30, 0},
{0, 0, 3, 15, 6, 19, 18, 21, 3, 18, 12, 18, 23, 18, 18, 12, 3, 20, 20, 17, 21, 18, 18, 18, 21, 21, 21, 20, 21, 21, 21, 14, 12, 20, 3, 22, 17, 20, 20, 21, 17, 2, 18, 2, 3, 18, 18, 21, 20, 15, 18, 8, 35, 19, 5, 0, 0, 18, 21, 20, 3, 0, 15, 17, 13, 32, 20},
{0, 0, 29, 16, 9, 17, 0, 12, 0, 35, 21, 21, 18, 12, 33, 15, 31, 18, 20, 15, 10, 20, 2, 15, 22, 21, 18, 20, 25, 29, 0, 10, 2, 32, 14, 19, 15, 0, 1, 25, 0, 0, 2, 9, 23, 12, 18, 20, 0, 13, 10, 20, 5, 8, 21, 1, 20, 18, 26, 14, 10, 4, 21, 11, 3, 2, 12},
{0, 0, 19, 8, 15, 35, 28, 15, 34, 23, 24, 21, 19, 21, 15, 12, 12, 2, 35, 4, 13, 0, 20, 5, 23, 20, 24, 21, 23, 32, 24, 14, 22, 5, 23, 11, 20, 18, 4, 7, 5, 32, 23, 21, 18, 17, 29, 29, 24, 0, 22, 31, 17, 18, 4, 23, 20, 9, 2, 23, 22, 20, 13, 19, 11, 19, 18},
{0, 0, 32, 32, 35, 22, 8, 12, 12, 30, 0, 18, 3, 21, 15, 15, 2, 0, 20, 23, 18, 23, 15, 15, 0, 14, 12, 23, 0, 17, 15, 18, 21, 21, 18, 21, 21, 3, 20, 32, 0, 33, 2, 14, 0, 20, 20, 33, 3, 33, 15, 0, 12, 20, 20, 21, 18, 3, 0, 5, 2, 2, 13, 27, 35, 17, 17},
{0, 0, 20, 18, 32, 3, 15, 21, 0, 0, 3, 18, 21, 5, 20, 20, 23, 12, 18, 12, 14, 21, 20, 21, 14, 0, 28, 21, 3, 0, 20, 15, 15, 30, 21, 15, 0, 15, 21, 32, 6, 12, 20, 0, 32, 14, 23, 33, 0, 2, 2, 3, 2, 21, 2, 14, 21, 3, 21, 3, 23, 7, 3, 23, 3, 20, 1},
{0, 0, 0, 3, 16, 1, 1, 19, 18, 18, 14, 30, 12, 14, 17, 23, 2, 24, 0, 31, 0, 9, 17, 3, 23, 2, 29, 1, 3, 3, 20, 26, 25, 5, 15, 29, 22, 34, 25, 18, 2, 25, 15, 32, 22, 26, 32, 23, 1, 21, 0, 29, 6, 10, 18, 9, 21, 3, 5, 19, 22, 8, 6, 8, 19, 13, 0},
{0, 0, 21, 14, 14, 19, 5, 20, 21, 27, 21, 21, 21, 14, 15, 20, 14, 14, 15, 17, 21, 22, 14, 14, 14, 14, 20, 14, 21, 14, 14, 33, 32, 12, 32, 2, 17, 23, 14, 14, 21, 14, 15, 14, 14, 14, 14, 27, 21, 32, 20, 14, 14, 15, 20, 14, 21, 21, 21, 22, 21, 15, 30, 9, 14, 14, 21},
{0, 0, 15, 26, 14, 12, 24, 17, 25, 15, 17, 14, 14, 20, 14, 23, 14, 26, 20, 29, 14, 3, 29, 2, 14, 19, 20, 24, 3, 21, 5, 14, 24, 3, 3, 15, 20, 3, 15, 24, 26, 32, 0, 15, 34, 32, 21, 34, 0, 33, 18, 20, 2, 30, 14, 3, 15, 2, 2, 3, 21, 32, 8, 3, 20, 0, 14},
{0, 0, 20, 14, 14, 18, 12, 12, 18, 14, 14, 15, 14, 14, 14, 2, 14, 14, 18, 12, 2, 14, 14, 14, 14, 0, 14, 14, 14, 3, 14, 14, 14, 14, 14, 14, 14, 14, 14, 18, 14, 14, 14, 0, 14, 14, 14, 14, 12, 2, 3, 14, 14, 12, 14, 14, 14, 20, 14, 14, 3, 2, 2, 3, 14, 14, 15},
{0, 0, 2, 2, 8, 2, 2, 2, 2, 2, 0, 2, 2, 0, 2, 2, 32, 28, 12, 14, 3, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 3, 15, 2, 12, 2, 2, 2, 2, 2, 2, 2, 2, 2, 20, 2, 4, 14, 0, 2, 2, 2, 2, 2, 2, 2, 2, 26, 2, 2, 14, 2, 2, 2},
{0, 0, 0, 0, 18, 2, 28, 14, 2, 2, 4, 2, 0, 3, 2, 9, 2, 0, 15, 0, 2, 4, 2, 0, 2, 2, 15, 3, 0, 3, 0, 0, 3, 2, 2, 2, 0, 14, 2, 0, 0, 2, 3, 2, 2, 15, 14, 2, 2, 33, 3, 0, 28, 2, 2, 2, 2, 2, 14, 2, 14, 0, 0, 2, 35, 4, 3}
},
{
{0, 0, 20, 33, 3, 3, 3, 21, 33, 20, 0, 18, 30, 33, 17, 0, 21, 2, 3, 3, 23, 30, 18, 12, 3, 33, 12, 14, 12, 33, 33, 15, 30, 3, 0, 18, 5, 20, 23, 23, 2, 14, 2, 23, 18, 2, 3, 5, 33, 18, 18, 12, 21, 0, 32, 23, 5, 3, 0, 15, 23, 21, 18, 18, 18, 23, 15},
{0, 0, 18, 21, 17, 20, 15, 14, 18, 23, 21, 5, 21, 20, 20, 35, 15, 33, 18, 15, 15, 13, 2, 20, 17, 5, 14, 22, 20, 15, 0, 23, 5, 15, 35, 2, 2, 20, 2, 3, 21, 20, 3, 20, 3, 5, 14, 20, 26, 5, 21, 15, 33, 20, 20, 18, 24, 15, 21, 20, 5, 18, 3, 23, 2, 23, 21},
{0, 0, 21, 35, 21, 21, 3, 29, 19, 32, 18, 2, 3, 31, 23, 20, 21, 13, 9, 14, 11, 10, 18, 0, 22, 3, 14, 21, 21, 21, 1, 18, 3, 20, 21, 21, 18, 31, 2, 17, 17, 19, 20, 25, 2, 0, 19, 7, 22, 24, 0, 15, 33, 5, 18, 19, 0, 18, 7, 17, 21, 2, 0, 0, 1, 21, 19},
{0, 0, 5, 19, 5, 0, 23, 9, 8, 22, 1, 18, 14, 3, 21, 34, 24, 24, 17, 0, 8, 19, 23, 5, 1, 30, 17, 25, 2, 19, 15, 2, 14, 21, 35, 23, 18, 20, 35, 17, 7, 0, 9, 17, 23, 27, 31, 11, 7, 0, 33, 0, 22, 32, 0, 1, 21, 14, 35, 5, 3, 0, 21, 3, 0, 1, 3},
{0, 0, 18, 15, 21, 14, 2, 3, 17, 18, 2, 18, 0, 21, 23, 0, 21, 20, 1, 17, 32, 13, 15, 21, 20, 15, 14, 0, 32, 21, 15, 3, 12, 3, 2, 18, 2, 18, 20, 21, 17, 0, 14, 20, 15, 21, 20, 8, 17, 32, 6, 15, 21, 0, 33, 21, 0, 3, 12, 3, 32, 18, 12, 14, 21, 20, 3},
{0, 0, 0, 21, 20, 23, 21, 0, 14, 20, 21, 21, 15, 21, 21, 7, 17, 15, 30, 21, 17, 20, 26, 20, 15, 17, 20, 12, 15, 21, 25, 15, 21, 21, 23, 32, 23, 21, 2, 21, 21, 21, 18, 20, 21, 3, 14, 20, 35, 0, 21, 15, 20, 33, 2, 20, 14, 15, 32, 20, 5, 6, 2, 21, 2, 21, 21},
{0, 0, 3, 21, 0, 20, 3, 2, 2, 21, 21, 17, 18, 17, 15, 20, 29, 16, 24, 0, 0, 28, 13, 23, 0, 20, 14, 6, 28, 15, 3, 26, 4, 0, 5, 29, 21, 21, 15, 12, 17, 19, 14, 12, 27, 21, 24, 26, 20, 10, 11, 11, 23, 23, 18, 11, 1, 21, 19, 18, 21, 21, 33, 27, 0, 0, 21},
{0, 0, 21, 14, 20, 20, 22, 26, 21, 22, 15, 15, 14, 14, 14, 14, 14, 15, 12, 20, 16, 19, 2, 14, 15, 27, 20, 14, 27, 14, 14, 14, 21, 14, 27, 14, 21, 14, 14, 14, 22, 14, 15, 22, 20, 14, 12, 9, 26, 15, 2, 20, 14, 14, 14, 14, 14, 20, 20, 27, 21, 16, 27, 15, 15, 14, 21},
{0, 0, 14, 2, 20, 15, 2, 14, 15, 14, 3, 16, 18, 15, 14, 0, 17, 6, 18, 14, 16, 34, 2, 14, 2, 14, 2, 2, 21, 2, 3, 15, 14, 17, 3, 3, 15, 14, 14, 15, 2, 35, 14, 8, 2, 14, 28, 26, 5, 0, 21, 14, 5, 14, 2, 14, 14, 20, 14, 4, 14, 15, 32, 2, 14, 20, 34},
{0, 0, 14, 14, 14, 14, 14, 14, 14, 2, 14, 14, 14, 14, 26, 20, 14, 14, 3, 18, 18, 14, 14, 14, 14, 14, 14, 14, 14, 0, 2, 14, 14, 12, 12, 2, 14, 14, 12, 0, 14, 14, 14, 14, 14, 14, 14, 14, 3, 3, 18, 14, 14, 15, 16, 14, 14, 14, 14, 12, 14, 14, 14, 14, 14, 14, 14},
{0, 0, 2, 2, 2, 2, 2, 2, 0, 2, 2, 12, 2, 2, 2, 2, 14, 2, 3, 14, 2, 14, 2, 14, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 2, 4, 2, 2, 0, 0, 2, 2, 2, 2, 2, 32, 2, 2, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 2, 2, 2, 2, 2, 2},
{0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 8, 2, 2, 3, 0, 14, 15, 2, 0, 2, 2, 2, 2, 2, 2, 3, 20, 22, 20, 14, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 15, 8, 0, 2, 22, 2, 2, 2, 2, 2, 2, 2, 0, 28, 2, 2, 2, 2}
}
};
const uint8_t g_aucIpmToTrSetModTmp[3][12][67] = {
{
{21, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21},
{21, 0, 21, 21, 3, 21, 21, 21, 21, 21, 3, 3, 3, 3, 3, 21, 3, 3, 3, 3, 3, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 3, 3, 18, 3, 21, 21, 21, 3, 3, 21, 21, 21, 21, 21, 21, 21, 3, 21, 3},
{5, 0, 21, 3, 21, 21, 21, 18, 18, 3, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 18, 3, 0, 21, 21, 21, 0, 21, 21, 21, 21, 18, 21, 21, 18, 18, 21, 18, 21, 3, 21, 18, 21, 21, 18, 18, 18, 21, 18, 21, 21, 21, 21, 21, 21},
{18, 0, 0, 21, 3, 21, 3, 18, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 3, 21, 21, 18, 21, 21, 21, 3, 3, 21, 21, 18, 3, 21, 3, 0, 3, 18, 21, 18, 3, 21, 21, 21, 23, 21, 18, 21, 21, 18, 3, 21, 21, 3, 21, 18, 0, 3, 3, 21, 0, 18, 3, 15, 21, 3, 21, 21, 21},
{30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 21, 3, 0, 18, 21, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{21, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 3, 18, 18, 3, 21, 21, 21, 21, 18, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21},
{18, 0, 3, 3, 3, 18, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 18, 3, 3, 18, 3, 18, 3, 3, 21, 3, 21, 3, 15, 18, 21, 3, 18, 3, 3, 3, 3, 18, 18, 3, 3, 3, 3, 18, 18, 18, 3, 18, 3, 3, 21, 0, 3, 21, 3, 3, 3},
{18, 0, 21, 21, 3, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 3, 21, 3, 21, 21, 21, 18, 21, 18, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 18, 21, 21},
{21, 0, 21, 0, 21, 3, 3, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 3, 21, 3, 3, 3, 21, 21, 18, 21, 21, 21, 0, 21, 0, 21, 21, 3, 21, 3, 3, 21, 21, 21, 18, 3, 0, 18, 21, 21, 21, 21, 21, 21, 21, 3, 3, 21, 21, 21, 15, 0, 18, 18, 27, 21, 3, 3, 21},
{21, 0, 18, 0, 2, 0, 0, 0, 21, 21, 21, 18, 21, 3, 3, 0, 21, 0, 21, 3, 0, 20, 0, 21, 21, 21, 3, 21, 0, 21, 0, 21, 18, 18, 3, 3, 3, 21, 0, 21, 0, 21, 18, 21, 21, 21, 0, 15, 0, 18, 21, 0, 21, 0, 18, 18, 21, 3, 21, 21, 21, 0, 0, 0, 12, 0, 3},
{0, 0, 23, 15, 22, 21, 21, 4, 18, 3, 21, 0, 21, 18, 12, 18, 18, 15, 3, 0, 21, 21, 21, 21, 4, 0, 21, 21, 0, 20, 18, 2, 15, 22, 21, 1, 15, 21, 14, 9, 22, 21, 21, 15, 14, 27, 6, 24, 18, 18, 21, 21, 16, 0, 12, 12, 3, 4, 21, 35, 12, 21, 15, 3, 24, 3, 12}
},
{
{0, 0, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 21, 18, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 3, 21, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3},
{0, 0, 3, 3, 18, 21, 5, 0, 21, 3, 3, 3, 21, 3, 3, 3, 3, 23, 21, 21, 3, 3, 21, 3, 0, 3, 21, 3, 3, 21, 21, 21, 3, 0, 21, 3, 0, 23, 18, 0, 21, 21, 21, 3, 3, 0, 0, 21, 3, 21, 21, 15, 21, 3, 3, 21, 21, 3, 21, 21, 21, 3, 17, 21, 3, 3, 3},
{0, 0, 21, 3, 5, 18, 21, 14, 21, 21, 18, 21, 21, 21, 21, 20, 18, 2, 21, 5, 21, 5, 18, 21, 3, 23, 3, 18, 18, 15, 21, 18, 21, 0, 3, 20, 18, 21, 21, 21, 21, 0, 21, 5, 21, 21, 21, 21, 21, 23, 21, 0, 18, 15, 23, 18, 18, 12, 21, 21, 5, 21, 3, 12, 18, 21, 5},
{0, 0, 0, 23, 2, 15, 18, 0, 0, 15, 23, 3, 21, 21, 18, 3, 21, 23, 0, 21, 23, 9, 0, 2, 3, 15, 21, 21, 20, 8, 18, 14, 11, 21, 0, 30, 21, 0, 23, 21, 3, 15, 3, 3, 21, 18, 3, 23, 21, 21, 3, 23, 2, 21, 21, 21, 21, 18, 0, 18, 20, 9, 23, 0, 3, 0, 21},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 21, 21, 12, 21, 18, 21, 18, 21, 3, 21, 21, 21, 21, 21, 21, 18, 21, 3, 18, 21, 21, 21, 21, 0, 21, 18, 3, 20, 21, 3, 3, 15, 21, 18, 0, 21, 0, 18, 21, 18, 21, 21, 3, 21, 21, 0, 21, 21, 21, 21, 0, 3, 18, 3, 21, 21, 3, 20, 3, 21, 3, 20, 21, 3, 3},
{0, 0, 18, 23, 20, 15, 19, 33, 3, 3, 21, 12, 3, 18, 19, 30, 4, 5, 3, 20, 23, 0, 21, 21, 18, 3, 17, 18, 18, 21, 1, 18, 18, 18, 0, 0, 12, 10, 3, 4, 3, 18, 26, 21, 32, 3, 0, 18, 3, 25, 0, 3, 3, 27, 21, 3, 3, 3, 3, 21, 0, 3, 28, 12, 19, 5, 18},
{0, 0, 3, 21, 20, 21, 21, 20, 21, 21, 21, 21, 21, 21, 18, 18, 21, 21, 21, 18, 18, 12, 18, 18, 18, 3, 21, 20, 18, 3, 15, 21, 15, 3, 3, 18, 20, 21, 20, 18, 3, 15, 21, 18, 3, 3, 3, 2, 3, 3, 21, 21, 21, 3, 3, 21, 21, 21, 21, 21, 21, 15, 21, 21, 15, 21, 18},
{0, 0, 21, 17, 18, 21, 18, 18, 21, 21, 21, 20, 18, 3, 3, 20, 18, 0, 21, 0, 18, 4, 4, 21, 3, 21, 0, 22, 18, 18, 3, 21, 3, 2, 3, 3, 0, 21, 21, 15, 21, 21, 21, 21, 21, 21, 21, 3, 33, 21, 21, 33, 18, 24, 0, 21, 2, 21, 18, 21, 21, 32, 16, 18, 27, 0, 0},
{0, 0, 21, 30, 30, 15, 13, 20, 18, 27, 24, 21, 3, 2, 21, 3, 20, 2, 21, 19, 16, 9, 18, 21, 27, 21, 18, 18, 0, 12, 18, 3, 32, 4, 3, 24, 17, 18, 3, 2, 0, 3, 3, 21, 22, 21, 3, 19, 26, 9, 21, 12, 15, 18, 21, 12, 18, 21, 4, 22, 3, 15, 8, 20, 5, 5, 21},
{0, 0, 0, 0, 0, 0, 0, 0, 3, 14, 3, 27, 9, 22, 18, 0, 0, 0, 3, 0, 0, 0, 28, 0, 26, 0, 33, 22, 15, 0, 2, 22, 0, 0, 0, 35, 19, 22, 2, 0, 10, 25, 12, 34, 22, 20, 26, 0, 16, 0, 3, 0, 0, 0, 0, 32, 18, 0, 0, 0, 12, 0, 0, 0, 0, 0, 3}
},
{
{0, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 21, 3, 21, 21, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 0, 21, 21, 18, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21},
{0, 0, 3, 3, 3, 3, 3, 3, 3, 21, 3, 3, 3, 21, 3, 0, 3, 20, 21, 0, 20, 2, 21, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 21, 15, 21, 21, 3, 3, 3, 21, 3, 21, 21, 3, 15, 23, 21, 3, 15, 0, 18, 0, 3, 21, 18, 21, 3, 21, 3, 3, 21, 3, 3, 3},
{0, 0, 21, 21, 3, 3, 21, 3, 21, 21, 3, 0, 3, 21, 21, 15, 21, 3, 0, 23, 23, 21, 18, 21, 18, 21, 21, 23, 21, 3, 21, 3, 21, 21, 3, 5, 21, 3, 21, 3, 21, 0, 0, 21, 21, 18, 21, 0, 21, 21, 21, 20, 21, 21, 18, 21, 21, 3, 18, 21, 18, 18, 21, 21, 18, 18, 21},
{0, 0, 0, 12, 21, 20, 0, 21, 21, 3, 3, 18, 5, 2, 5, 12, 19, 5, 3, 23, 29, 21, 20, 0, 18, 23, 12, 23, 3, 18, 0, 20, 21, 0, 21, 32, 3, 15, 0, 3, 3, 12, 0, 0, 21, 24, 5, 5, 0, 3, 3, 11, 5, 21, 3, 5, 30, 18, 0, 4, 3, 15, 21, 0, 0, 0, 18},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 21, 18, 21, 21, 21, 21, 21, 21, 3, 21, 21, 3, 21, 18, 2, 3, 21, 21, 21, 2, 3, 21, 18, 3, 21, 3, 21, 3, 18, 3, 21, 18, 21, 3, 0, 3, 18, 21, 3, 21, 0, 21, 21, 3, 21, 15, 12, 18, 18, 0, 18, 21, 21, 18, 21, 21, 21, 21, 21, 18, 3, 21, 21, 21, 21},
{0, 0, 3, 3, 21, 20, 18, 21, 3, 3, 3, 3, 5, 21, 18, 15, 0, 24, 3, 20, 0, 33, 21, 21, 18, 21, 20, 18, 24, 17, 3, 3, 5, 2, 3, 26, 9, 23, 20, 35, 0, 21, 23, 33, 12, 0, 33, 3, 0, 3, 18, 19, 21, 18, 18, 2, 21, 0, 3, 3, 21, 21, 18, 21, 20, 3, 3},
{0, 0, 21, 21, 21, 21, 21, 21, 18, 21, 21, 18, 21, 3, 15, 21, 3, 21, 21, 18, 15, 21, 21, 18, 21, 21, 21, 9, 12, 3, 15, 21, 12, 0, 21, 0, 2, 21, 20, 18, 2, 19, 21, 21, 21, 3, 21, 21, 20, 3, 21, 21, 18, 21, 20, 18, 21, 3, 21, 21, 3, 21, 21, 21, 21, 21, 21},
{0, 0, 3, 21, 0, 15, 0, 21, 3, 18, 18, 20, 18, 3, 18, 33, 20, 18, 21, 14, 0, 7, 18, 21, 21, 3, 3, 20, 3, 0, 32, 21, 21, 21, 21, 18, 18, 21, 17, 3, 21, 3, 22, 21, 21, 21, 21, 18, 0, 21, 21, 3, 4, 21, 21, 3, 21, 12, 3, 3, 18, 9, 18, 3, 21, 33, 3},
{0, 0, 0, 18, 21, 21, 0, 21, 0, 21, 18, 18, 21, 21, 18, 0, 12, 3, 0, 2, 2, 23, 0, 3, 0, 18, 2, 18, 18, 21, 21, 21, 27, 4, 3, 24, 22, 21, 21, 21, 3, 3, 12, 3, 0, 18, 0, 33, 12, 12, 0, 18, 2, 0, 3, 21, 21, 3, 3, 21, 0, 21, 0, 21, 21, 3, 0},
{0, 0, 0, 21, 27, 21, 17, 0, 12, 3, 35, 16, 12, 28, 20, 0, 0, 0, 0, 26, 18, 0, 35, 21, 14, 2, 18, 0, 33, 0, 20, 3, 9, 21, 3, 0, 32, 15, 15, 20, 18, 0, 0, 0, 16, 22, 7, 8, 18, 0, 0, 0, 0, 0, 12, 21, 20, 20, 7, 9, 18, 0, 7, 24, 3, 3, 3}
}
};
const uint8_t g_aucIpmToTrSetModSgpm[3][11][67] = {
{
{18, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 3, 18, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 3, 18, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21},
{21, 0, 21, 21, 3, 21, 21, 21, 21, 0, 3, 3, 3, 21, 3, 21, 21, 3, 3, 3, 0, 3, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 21, 0, 0, 0, 0, 21, 3, 21, 21, 0, 3, 21, 21, 21, 21, 21, 21, 18, 3, 18},
{23, 0, 21, 21, 3, 21, 3, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 18, 3, 21, 21, 18, 21, 18, 21, 18, 18, 21, 21, 21, 21, 21, 21, 18, 21, 18, 21, 18, 18, 21, 3, 3, 21, 21, 21, 21, 21},
{0, 0, 21, 3, 21, 0, 21, 23, 21, 21, 21, 21, 21, 21, 21, 18, 21, 3, 21, 3, 21, 0, 21, 21, 3, 0, 21, 15, 21, 3, 3, 21, 3, 3, 3, 21, 21, 3, 18, 12, 3, 21, 21, 21, 21, 18, 21, 0, 3, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 3, 12, 21, 18, 21, 21, 21, 3},
{3, 0, 0, 3, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 0, 0, 0, 21, 18, 0, 0, 3, 0, 18, 21, 21, 21, 18, 21, 21, 21, 3, 21, 21, 21, 3, 0, 18, 0, 0, 3, 21, 0, 0, 0, 0, 18, 18, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 18, 0},
{0, 0, 3, 21, 3, 3, 21, 21, 3, 18, 21, 21, 21, 21, 21, 21, 21, 3, 3, 21, 21, 3, 21, 21, 21, 3, 21, 3, 21, 18, 21, 18, 3, 18, 21, 3, 18, 21, 21, 21, 21, 3, 21, 18, 21, 21, 21, 0, 18, 21, 21, 21, 21, 21, 21, 18, 18, 21, 21, 3, 21, 21, 21, 21, 21, 18, 12},
{3, 0, 18, 21, 3, 21, 18, 3, 3, 3, 3, 3, 3, 3, 18, 18, 3, 3, 3, 3, 21, 3, 3, 3, 21, 3, 3, 18, 3, 0, 3, 3, 18, 12, 3, 15, 21, 21, 18, 3, 18, 0, 18, 18, 18, 3, 3, 18, 3, 18, 21, 3, 3, 3, 3, 18, 18, 21, 21, 18, 3, 3, 3, 20, 21, 18, 3},
{21, 0, 3, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 0, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 18},
{18, 0, 3, 18, 21, 3, 21, 0, 18, 21, 21, 21, 21, 3, 3, 0, 21, 3, 21, 3, 21, 3, 18, 3, 3, 21, 21, 21, 21, 21, 21, 21, 3, 21, 3, 18, 3, 21, 21, 21, 21, 21, 3, 15, 21, 18, 21, 12, 21, 21, 21, 21, 21, 21, 3, 21, 21, 21, 21, 18, 21, 18, 21, 3, 21, 21, 24},
{21, 0, 12, 0, 3, 0, 20, 0, 2, 21, 2, 12, 20, 21, 3, 3, 21, 0, 0, 2, 21, 0, 3, 0, 21, 0, 3, 0, 3, 0, 0, 18, 0, 3, 24, 18, 0, 3, 0, 0, 18, 0, 18, 0, 21, 0, 18, 0, 21, 12, 0, 0, 21, 18, 18, 21, 15, 2, 12, 21, 12, 0, 15, 0, 18, 0, 2},
{3, 0, 0, 0, 0, 21, 21, 23, 18, 21, 21, 18, 15, 0, 20, 18, 2, 15, 0, 3, 3, 21, 18, 0, 0, 0, 21, 27, 18, 22, 20, 6, 35, 23, 13, 18, 30, 21, 15, 28, 20, 3, 20, 3, 20, 0, 20, 22, 18, 0, 18, 12, 34, 14, 26, 18, 4, 16, 24, 13, 32, 13, 18, 15, 22, 25, 12}
},
{
{0, 0, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 3, 3, 21, 18, 3, 21, 21, 3, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 18, 21, 21, 18, 3, 21, 18, 18, 18, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21},
{0, 0, 21, 3, 3, 12, 21, 21, 21, 0, 3, 3, 3, 21, 21, 3, 0, 21, 21, 3, 3, 3, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 3, 3, 21, 21, 21, 3, 21, 3, 18, 0, 21, 12, 0, 18, 21, 0, 0, 0, 3, 3, 21, 21, 21, 21, 21, 15, 21, 21, 0, 21},
{0, 0, 2, 21, 20, 5, 21, 21, 18, 21, 23, 21, 18, 18, 21, 18, 21, 21, 3, 21, 18, 23, 3, 3, 12, 21, 21, 18, 21, 21, 18, 21, 18, 21, 3, 18, 18, 18, 3, 3, 0, 0, 3, 21, 21, 21, 18, 21, 21, 18, 21, 21, 18, 21, 0, 18, 18, 3, 21, 2, 15, 18, 21, 15, 21, 3, 3},
{0, 0, 18, 21, 20, 18, 21, 2, 3, 17, 21, 18, 3, 0, 3, 21, 23, 23, 18, 18, 20, 21, 0, 18, 21, 0, 21, 5, 0, 33, 3, 18, 14, 15, 18, 18, 20, 18, 3, 18, 3, 18, 18, 3, 18, 21, 23, 21, 0, 21, 21, 18, 21, 21, 21, 3, 0, 3, 21, 4, 21, 18, 3, 18, 21, 9, 21},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 3, 0, 0, 0, 3, 0, 18, 0, 0, 0, 18, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 21, 18, 21, 3, 3, 3, 21, 3, 21, 21, 21, 3, 21, 3, 21, 21, 21, 3, 21, 0, 18, 3, 21, 21, 21, 21, 0, 21, 3, 21, 21, 21, 18, 3, 21, 21, 21, 3, 3, 18, 3, 3, 0, 18, 18, 21, 0, 18, 18, 18, 21, 21, 21, 21, 21, 0, 21, 18, 21, 21, 3, 21, 18, 21, 21},
{0, 0, 3, 3, 18, 21, 21, 5, 3, 13, 3, 3, 21, 3, 0, 21, 3, 20, 3, 3, 3, 18, 20, 3, 18, 3, 19, 3, 3, 33, 0, 3, 21, 27, 0, 29, 18, 18, 24, 3, 21, 0, 18, 18, 15, 21, 0, 3, 0, 3, 18, 0, 18, 18, 18, 3, 5, 3, 3, 28, 21, 3, 21, 21, 29, 0, 3},
{0, 0, 3, 3, 21, 21, 21, 21, 3, 18, 0, 2, 20, 18, 3, 2, 21, 21, 21, 21, 21, 20, 18, 21, 21, 21, 18, 18, 21, 21, 3, 18, 20, 18, 21, 3, 15, 3, 18, 21, 21, 3, 3, 21, 21, 21, 3, 15, 21, 21, 21, 21, 21, 12, 18, 3, 15, 12, 0, 3, 18, 21, 21, 21, 21, 18, 18},
{0, 0, 12, 0, 3, 12, 15, 15, 15, 0, 18, 18, 15, 3, 21, 18, 21, 21, 21, 0, 2, 18, 3, 21, 3, 3, 2, 3, 21, 15, 18, 21, 3, 21, 0, 0, 3, 0, 34, 18, 3, 35, 21, 3, 3, 15, 21, 3, 21, 18, 21, 3, 18, 18, 6, 3, 3, 34, 3, 3, 3, 18, 21, 0, 9, 27, 2},
{0, 0, 15, 20, 27, 21, 0, 2, 21, 12, 21, 1, 0, 3, 20, 0, 20, 4, 0, 2, 20, 20, 0, 2, 18, 20, 4, 18, 1, 20, 5, 21, 0, 21, 18, 21, 0, 21, 30, 15, 6, 3, 24, 15, 3, 12, 0, 15, 15, 12, 0, 24, 15, 0, 15, 18, 0, 6, 21, 2, 21, 12, 0, 21, 22, 15, 20},
{0, 0, 0, 0, 0, 14, 12, 0, 20, 0, 3, 0, 0, 17, 18, 1, 0, 0, 3, 0, 3, 19, 34, 20, 18, 20, 27, 11, 33, 0, 18, 28, 7, 35, 0, 23, 4, 3, 20, 0, 6, 0, 22, 0, 0, 34, 12, 9, 20, 0, 21, 0, 0, 27, 2, 24, 0, 0, 35, 23, 4, 34, 0, 0, 0, 4, 3}
},
{
{0, 0, 30, 3, 21, 21, 21, 21, 3, 3, 21, 21, 21, 21, 3, 18, 21, 21, 3, 21, 21, 3, 21, 21, 21, 21, 21, 3, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 18, 21, 21, 21, 21, 21, 18, 21, 21, 18, 21, 21, 3, 18, 21, 21, 21, 21, 18, 18, 21, 21, 21, 21, 18, 5},
{0, 0, 3, 21, 21, 3, 3, 3, 3, 0, 3, 3, 3, 3, 21, 3, 3, 3, 3, 3, 3, 21, 21, 3, 21, 21, 3, 3, 3, 21, 3, 21, 21, 3, 21, 3, 21, 21, 21, 21, 21, 21, 0, 3, 21, 0, 21, 18, 21, 18, 21, 21, 3, 18, 0, 3, 3, 21, 3, 3, 0, 3, 21, 3, 5, 0, 21},
{0, 0, 21, 21, 21, 18, 18, 0, 3, 20, 0, 21, 18, 21, 18, 21, 3, 18, 18, 20, 21, 21, 18, 20, 5, 18, 23, 21, 5, 15, 18, 21, 20, 0, 21, 5, 18, 21, 20, 21, 18, 0, 21, 21, 21, 3, 23, 18, 21, 21, 18, 3, 21, 3, 18, 3, 0, 18, 21, 23, 21, 21, 21, 3, 2, 21, 0},
{0, 0, 0, 0, 17, 12, 15, 23, 5, 18, 18, 23, 21, 18, 18, 3, 21, 18, 18, 0, 23, 5, 3, 5, 23, 21, 3, 21, 3, 2, 2, 18, 21, 17, 3, 2, 14, 3, 21, 17, 0, 12, 18, 3, 12, 0, 3, 15, 21, 18, 3, 15, 21, 18, 3, 0, 33, 0, 0, 33, 0, 6, 0, 1, 29, 6, 3},
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 18, 0, 0, 18, 0, 0, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{0, 0, 21, 2, 21, 21, 0, 0, 21, 21, 21, 18, 21, 3, 3, 21, 18, 18, 18, 18, 18, 18, 0, 21, 21, 0, 21, 3, 18, 18, 3, 21, 3, 3, 21, 21, 21, 3, 21, 21, 3, 3, 12, 2, 0, 21, 3, 21, 18, 3, 21, 18, 18, 3, 21, 18, 21, 21, 3, 21, 21, 20, 18, 3, 21, 20, 21},
{0, 0, 3, 3, 3, 21, 18, 33, 3, 23, 20, 3, 18, 3, 21, 15, 0, 3, 3, 22, 20, 3, 3, 3, 3, 6, 4, 3, 18, 35, 3, 27, 26, 2, 0, 12, 18, 22, 30, 18, 3, 18, 21, 23, 3, 20, 15, 18, 21, 3, 21, 3, 12, 3, 3, 21, 18, 3, 0, 0, 21, 3, 18, 2, 2, 0, 18},
{0, 0, 21, 3, 21, 3, 15, 20, 21, 2, 21, 3, 21, 21, 21, 0, 21, 3, 21, 18, 21, 2, 18, 21, 18, 0, 21, 0, 0, 3, 18, 21, 21, 21, 21, 21, 21, 21, 3, 18, 0, 0, 21, 0, 3, 21, 3, 12, 21, 3, 21, 18, 21, 0, 21, 21, 21, 18, 21, 12, 21, 15, 20, 18, 21, 18, 21},
{0, 0, 3, 3, 18, 18, 3, 18, 21, 0, 14, 18, 3, 21, 3, 18, 3, 0, 21, 0, 22, 21, 3, 20, 21, 18, 21, 18, 21, 0, 14, 21, 3, 18, 21, 18, 21, 3, 21, 3, 12, 21, 18, 18, 15, 0, 12, 24, 21, 21, 21, 18, 0, 12, 3, 2, 18, 21, 24, 21, 15, 18, 18, 21, 21, 21, 20},
{0, 0, 18, 24, 19, 3, 18, 34, 18, 30, 2, 21, 20, 20, 20, 3, 22, 0, 0, 18, 20, 27, 0, 18, 21, 2, 0, 19, 18, 0, 21, 21, 22, 9, 18, 19, 27, 21, 21, 0, 3, 9, 0, 12, 21, 3, 0, 22, 15, 3, 0, 0, 27, 18, 15, 15, 15, 21, 12, 5, 3, 29, 3, 18, 9, 4, 3},
{0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 25, 25, 19, 16, 0, 27, 0, 0, 18, 0, 22, 5, 0, 21, 34, 27, 0, 31, 27, 0, 21, 21, 15, 34, 21, 31, 1, 4, 28, 0, 21, 0, 20, 9, 3, 18, 0, 0, 0, 0, 21, 0, 0, 0, 23, 2, 0, 8, 20, 0, 24, 0, 0, 0, 0, 0, 24}
}
};
const uint8_t g_aucIpmToTrSetModEip[3][10][67] = {
{
{21, 0, 15, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20},
{3, 0, 21, 21, 15, 14, 21, 15, 21, 15, 21, 15, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 15, 21, 20, 21, 21, 20, 21, 21, 21, 20, 21, 21, 15, 21, 20, 15, 14, 21, 21, 21, 21, 15, 15, 15, 15, 21, 14, 20, 20},
{5, 0, 15, 21, 14, 14, 20, 20, 20, 20, 15, 21, 14, 15, 15, 20, 21, 20, 21, 21, 21, 20, 15, 20, 15, 20, 20, 14, 20, 20, 20, 20, 21, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 20, 15, 20, 21, 20, 20, 21, 20, 21, 21, 21, 20, 2, 20, 20, 21, 21, 14, 20, 2, 14, 21, 21, 15},
{3, 0, 22, 28, 21, 14, 14, 20, 20, 22, 15, 21, 18, 21, 21, 15, 17, 15, 21, 14, 15, 14, 21, 15, 21, 21, 20, 21, 21, 20, 20, 20, 14, 14, 20, 21, 15, 20, 14, 3, 2, 14, 2, 3, 21, 21, 20, 27, 18, 14, 22, 15, 21, 15, 21, 20, 21, 27, 32, 0, 2, 3, 15, 16, 30, 10, 15},
{0, 0, 0, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 12, 14, 12, 0, 0, 0, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 0, 0, 0, 2, 14, 2, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 0},
{21, 0, 21, 20, 21, 14, 21, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 21, 14, 15, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 20, 20, 20, 14, 14, 14, 20, 20, 14, 14, 20, 20, 20, 20, 20, 20, 20, 20, 14, 20, 20, 21, 20, 14, 14, 21, 14, 20, 14, 21, 14, 14, 12, 21},
{18, 0, 14, 14, 14, 3, 14, 14, 15, 14, 14, 20, 20, 20, 14, 14, 14, 14, 15, 15, 14, 14, 14, 14, 14, 20, 20, 20, 14, 20, 20, 20, 14, 20, 14, 14, 14, 20, 20, 20, 20, 20, 14, 14, 14, 14, 22, 20, 26, 20, 20, 14, 14, 21, 14, 14, 14, 14, 2, 2, 20, 14, 22, 20, 14, 20, 30},
{0, 0, 15, 14, 14, 16, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 26, 14, 14, 20},
{3, 0, 15, 14, 8, 14, 14, 20, 20, 14, 14, 14, 14, 14, 14, 15, 14, 20, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 2, 20, 15, 20, 14, 14, 14, 20, 27, 15, 14, 14, 14, 2, 16, 14, 3},
{21, 0, 26, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 16}
},
{
{0, 0, 20, 21, 15, 15, 21, 33, 23, 33, 21, 23, 21, 15, 15, 20, 15, 21, 20, 20, 15, 20, 20, 20, 21, 15, 15, 20, 21, 21, 21, 15, 20, 21, 21, 21, 15, 20, 21, 21, 21, 15, 20, 20, 21, 15, 15, 15, 20, 15, 15, 21, 20, 15, 20, 20, 21, 33, 21, 23, 33, 23, 21, 20, 20, 21, 15},
{0, 0, 15, 14, 23, 15, 33, 14, 14, 15, 20, 20, 15, 15, 21, 14, 20, 14, 32, 20, 20, 20, 21, 14, 21, 15, 21, 15, 23, 14, 21, 21, 21, 20, 15, 21, 15, 3, 14, 20, 20, 21, 21, 21, 14, 14, 15, 21, 21, 20, 14, 15, 15, 21, 20, 20, 20, 15, 21, 21, 15, 14, 20, 15, 33, 21, 20},
{0, 0, 3, 14, 23, 14, 21, 20, 20, 14, 20, 28, 20, 21, 21, 20, 14, 2, 21, 2, 15, 15, 21, 21, 15, 15, 14, 23, 14, 14, 14, 21, 14, 21, 20, 4, 14, 20, 21, 20, 21, 12, 21, 14, 14, 20, 15, 14, 26, 23, 21, 16, 20, 18, 21, 20, 20, 3, 20, 27, 15, 16, 17, 3, 32, 20, 23},
{0, 0, 4, 12, 21, 17, 27, 10, 21, 22, 2, 14, 18, 15, 16, 24, 22, 14, 21, 16, 3, 3, 21, 20, 14, 32, 21, 21, 20, 16, 16, 34, 2, 33, 14, 14, 14, 12, 2, 4, 20, 29, 20, 21, 15, 16, 32, 21, 20, 20, 21, 2, 26, 4, 14, 20, 27, 20, 22, 32, 27, 13, 29, 8, 18, 0, 3},
{0, 0, 0, 17, 2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 2, 0, 14, 14, 0, 17, 20, 12, 0, 0, 12, 14, 0, 12, 14, 14, 2, 14, 14, 14, 0, 14, 14, 14, 12, 14, 14, 2, 0, 14, 2, 0, 0, 2, 15, 32, 0, 14, 14, 0, 12, 0, 0, 0, 0, 12, 0, 0, 0, 0, 12, 32, 0},
{0, 0, 21, 15, 2, 14, 20, 15, 21, 21, 14, 14, 14, 14, 14, 14, 18, 14, 21, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 27, 14, 20, 20, 14, 14, 20, 15, 14, 14, 21, 26, 14, 14, 20, 14, 20, 26, 21, 20, 14, 20, 21, 14, 20, 26, 20, 14, 20, 14, 14, 16, 28, 26, 18},
{0, 0, 2, 15, 35, 26, 14, 33, 28, 26, 20, 15, 26, 14, 14, 15, 15, 2, 21, 15, 20, 15, 14, 14, 14, 26, 14, 15, 20, 20, 21, 3, 32, 26, 14, 32, 14, 22, 14, 14, 20, 27, 32, 14, 20, 27, 14, 19, 12, 3, 3, 20, 32, 15, 20, 21, 15, 21, 26, 20, 34, 11, 20, 14, 27, 4, 33},
{0, 0, 20, 14, 15, 14, 17, 14, 14, 28, 14, 14, 15, 14, 14, 14, 14, 14, 21, 26, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 20, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 16, 21, 14, 14, 14, 14, 14, 20, 14, 14, 28, 14, 14, 32, 14, 20, 14, 15},
{0, 0, 16, 14, 12, 22, 14, 0, 21, 14, 17, 14, 14, 2, 14, 21, 14, 4, 21, 16, 12, 14, 14, 14, 26, 14, 14, 14, 14, 14, 14, 14, 14, 2, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 16, 20, 14, 0, 14, 26, 20, 32, 26, 14, 20, 14, 20, 26, 14, 14, 14, 11, 8, 18, 2, 22, 21},
{0, 0, 20, 26, 14, 18, 25, 26, 14, 14, 14, 14, 14, 14, 14, 16, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 26, 14, 14, 14, 14, 14, 14, 14, 16, 10, 3, 14, 16, 15}
},
{
{0, 0, 15, 21, 20, 21, 21, 15, 23, 21, 20, 21, 15, 21, 21, 23, 21, 15, 20, 20, 21, 20, 20, 21, 15, 21, 21, 21, 21, 20, 21, 21, 15, 15, 21, 20, 20, 21, 21, 15, 21, 21, 21, 21, 20, 21, 15, 15, 21, 15, 15, 20, 21, 33, 21, 21, 20, 21, 15, 21, 33, 20, 21, 21, 15, 21, 20},
{0, 0, 21, 14, 21, 15, 21, 21, 15, 21, 15, 15, 15, 21, 21, 21, 15, 20, 21, 20, 15, 32, 14, 14, 20, 20, 15, 14, 15, 21, 15, 15, 14, 20, 20, 15, 14, 15, 21, 21, 14, 15, 20, 15, 14, 21, 21, 14, 32, 21, 20, 15, 20, 15, 15, 14, 15, 20, 20, 14, 20, 15, 21, 15, 20, 15, 20},
{0, 0, 20, 21, 15, 14, 14, 14, 23, 20, 20, 16, 20, 14, 3, 17, 28, 2, 21, 18, 17, 13, 14, 26, 14, 20, 20, 15, 20, 15, 27, 15, 14, 33, 15, 20, 14, 15, 21, 15, 2, 12, 14, 18, 2, 14, 5, 21, 2, 12, 12, 15, 20, 23, 14, 15, 12, 18, 21, 21, 20, 3, 3, 21, 20, 21, 14},
{0, 0, 3, 14, 21, 18, 2, 14, 15, 23, 14, 17, 23, 15, 21, 20, 18, 28, 16, 22, 22, 4, 12, 21, 3, 4, 21, 23, 35, 22, 20, 27, 20, 4, 2, 21, 20, 12, 21, 21, 20, 0, 27, 32, 14, 16, 18, 4, 35, 22, 3, 21, 35, 35, 16, 28, 27, 20, 23, 21, 2, 2, 31, 21, 21, 15, 2},
{0, 0, 0, 14, 12, 0, 0, 0, 0, 2, 0, 0, 14, 14, 14, 0, 14, 14, 0, 14, 20, 20, 14, 14, 20, 14, 2, 0, 14, 14, 0, 0, 14, 2, 0, 12, 14, 0, 0, 14, 14, 0, 12, 14, 15, 14, 14, 15, 15, 14, 0, 14, 14, 0, 14, 14, 14, 0, 0, 12, 0, 0, 0, 0, 2, 14, 0},
{0, 0, 3, 14, 14, 14, 26, 14, 20, 16, 15, 26, 14, 15, 14, 3, 22, 21, 21, 14, 20, 15, 14, 14, 14, 21, 27, 14, 14, 14, 14, 20, 20, 14, 20, 14, 3, 20, 14, 14, 14, 14, 20, 20, 14, 14, 2, 20, 26, 21, 21, 12, 3, 27, 2, 20, 26, 14, 14, 27, 2, 20, 14, 14, 14, 20, 21},
{0, 0, 21, 15, 2, 20, 20, 14, 21, 14, 14, 15, 15, 2, 34, 33, 8, 16, 21, 16, 21, 2, 26, 10, 2, 14, 15, 20, 26, 28, 14, 20, 14, 14, 14, 20, 28, 9, 2, 20, 12, 21, 28, 26, 2, 14, 20, 22, 4, 20, 15, 17, 18, 26, 32, 27, 20, 2, 14, 2, 21, 24, 5, 20, 5, 14, 20},
{0, 0, 21, 20, 16, 14, 20, 14, 15, 14, 14, 15, 14, 14, 21, 14, 16, 26, 21, 15, 14, 14, 14, 14, 14, 26, 14, 14, 14, 12, 16, 15, 14, 26, 20, 16, 14, 20, 26, 2, 14, 14, 14, 16, 14, 14, 14, 14, 14, 20, 21, 16, 26, 14, 21, 14, 14, 20, 14, 14, 20, 14, 15, 14, 26, 15, 21},
{0, 0, 15, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 19, 20, 21, 20, 24, 12, 21, 14, 14, 14, 14, 14, 26, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 15, 16, 16, 14, 20, 26, 14, 16, 26, 15, 2, 28, 18, 26, 14, 14, 14, 2, 32, 2, 15, 26, 14, 14, 14, 26, 15},
{0, 0, 14, 14, 14, 14, 14, 14, 16, 14, 14, 14, 14, 14, 14, 14, 14, 16, 20, 35, 28, 14, 14, 14, 14, 24, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 4, 14, 14, 14, 14, 28, 35, 15, 26, 14, 14, 14, 14, 14, 14, 14, 14, 26, 14, 14, 14, 14, 14, 14}
}
};
const uint8_t g_aucIpmToTrSetModPnn[3][11][67] = {
{
{26, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14},
{16, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14},
{21, 0, 15, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 21, 20, 20, 20, 14, 20, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 20, 14, 20, 20, 14, 14, 14, 20, 20, 14, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 14, 20, 20, 20, 14, 20, 20, 14, 14, 14, 14, 14, 20},
{16, 0, 21, 14, 20, 14, 20, 14, 21, 14, 3, 14, 21, 21, 21, 15, 14, 15, 21, 21, 21, 14, 21, 21, 21, 20, 20, 14, 21, 20, 20, 21, 14, 20, 21, 20, 15, 20, 14, 20, 21, 20, 20, 14, 14, 14, 20, 14, 16, 22, 14, 20, 14, 20, 21, 14, 14, 14, 20, 20, 21, 21, 20, 20, 16, 20, 21},
{26, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14},
{26, 0, 15, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 15, 21, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 20, 14, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 21},
{28, 0, 20, 14, 14, 14, 26, 14, 15, 14, 20, 20, 14, 14, 14, 14, 14, 14, 21, 15, 15, 14, 14, 14, 20, 14, 20, 14, 20, 14, 14, 20, 14, 14, 20, 20, 14, 14, 20, 20, 20, 20, 26, 14, 20, 20, 26, 14, 20, 2, 20, 20, 20, 14, 20, 14, 14, 20, 2, 20, 14, 14, 14, 14, 20, 14, 2},
{21, 0, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 21, 14, 14, 14, 15, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 21, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 20, 14, 14, 14, 21, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15},
{28, 0, 15, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 21, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 20, 14, 14, 14, 21, 14, 14, 14, 14, 14, 14, 14, 20, 14, 20, 20, 15, 14, 14, 14, 20, 26, 14, 20, 14, 14, 20, 14, 15, 14, 14, 14, 20, 14, 14, 14, 15},
{14, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 15, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 14, 20, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14},
{28, 0, 0, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 14, 2, 2, 14, 14, 14, 2, 2, 2, 14, 2, 2, 2, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 2, 14, 14, 14, 14, 2, 2, 14, 14, 14, 2, 14, 0, 26, 14, 26, 14, 14, 2, 14, 14, 14, 20, 14, 14, 12, 26, 14}
},
{
{0, 0, 20, 20, 15, 15, 20, 20, 20, 20, 14, 14, 20, 15, 14, 20, 14, 15, 20, 14, 15, 14, 20, 20, 20, 20, 14, 20, 14, 20, 15, 15, 14, 14, 20, 14, 14, 20, 20, 15, 14, 15, 14, 15, 15, 15, 15, 14, 20, 14, 15, 20, 14, 15, 14, 20, 15, 14, 14, 15, 15, 15, 15, 20, 20, 15, 15},
{0, 0, 15, 14, 21, 20, 21, 20, 21, 15, 20, 20, 14, 14, 21, 14, 14, 20, 20, 14, 20, 20, 20, 15, 14, 14, 14, 15, 15, 14, 20, 14, 14, 14, 14, 15, 15, 20, 14, 14, 15, 14, 15, 14, 14, 14, 14, 15, 14, 21, 15, 20, 14, 14, 14, 14, 15, 14, 20, 15, 20, 15, 14, 21, 15, 15, 14},
{0, 0, 21, 14, 14, 21, 15, 20, 15, 21, 21, 20, 20, 20, 15, 15, 15, 20, 21, 14, 20, 20, 21, 20, 14, 21, 20, 20, 21, 15, 20, 20, 15, 14, 21, 15, 20, 20, 21, 14, 20, 20, 21, 20, 20, 20, 21, 20, 14, 20, 21, 14, 20, 20, 14, 20, 20, 20, 20, 21, 20, 20, 20, 21, 20, 21, 20},
{0, 0, 3, 20, 21, 2, 5, 20, 21, 21, 27, 20, 21, 21, 20, 20, 2, 14, 21, 15, 15, 21, 15, 21, 21, 14, 21, 21, 21, 20, 20, 21, 27, 16, 21, 20, 21, 21, 20, 20, 20, 16, 14, 15, 21, 20, 3, 22, 14, 15, 21, 21, 20, 14, 16, 16, 15, 20, 3, 21, 14, 21, 12, 0, 14, 20, 21},
{0, 0, 14, 20, 14, 15, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 12, 14, 20, 20, 14, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 20, 14, 20, 14, 15, 20, 14, 14, 14, 14, 14, 14, 14, 14, 14, 20, 14, 15, 15, 14, 2, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 20, 14, 15, 14},
{0, 0, 21, 20, 14, 15, 21, 14, 20, 14, 21, 14, 21, 21, 14, 20, 14, 14, 21, 15, 15, 20, 15, 20, 15, 20, 20, 20, 21, 14, 14, 20, 14, 20, 21, 20, 20, 14, 20, 14, 20, 14, 20, 20, 20, 20, 20, 21, 15, 18, 21, 14, 20, 14, 20, 20, 14, 20, 21, 21, 21, 20, 20, 20, 14, 21, 21},
{0, 0, 3, 14, 4, 20, 20, 15, 3, 27, 20, 20, 14, 15, 14, 21, 12, 20, 3, 27, 14, 3, 20, 20, 15, 15, 21, 26, 3, 20, 14, 2, 2, 15, 3, 2, 20, 20, 26, 14, 20, 14, 3, 14, 14, 21, 14, 20, 14, 15, 3, 18, 33, 14, 20, 15, 22, 21, 14, 14, 20, 16, 15, 3, 21, 35, 3},
{0, 0, 21, 23, 20, 21, 20, 2, 21, 14, 14, 14, 14, 14, 14, 20, 14, 14, 21, 14, 15, 14, 20, 14, 14, 15, 15, 14, 20, 15, 14, 14, 14, 14, 21, 14, 14, 14, 14, 20, 15, 14, 20, 20, 14, 14, 15, 14, 20, 14, 21, 14, 14, 15, 14, 14, 14, 14, 14, 14, 21, 12, 15, 21, 15, 33, 21},
{0, 0, 21, 15, 32, 14, 13, 15, 21, 21, 22, 15, 21, 14, 27, 26, 12, 15, 21, 16, 14, 15, 16, 20, 20, 2, 14, 15, 21, 20, 15, 14, 20, 26, 21, 15, 14, 21, 20, 32, 20, 20, 14, 15, 14, 20, 14, 2, 2, 17, 21, 22, 0, 14, 2, 14, 20, 20, 20, 15, 15, 4, 14, 18, 14, 35, 20},
{0, 0, 21, 20, 20, 14, 16, 26, 14, 14, 14, 12, 20, 14, 14, 14, 2, 14, 0, 0, 15, 16, 14, 14, 15, 14, 14, 2, 15, 14, 14, 14, 14, 12, 21, 2, 14, 14, 14, 14, 20, 12, 14, 14, 20, 14, 14, 26, 20, 0, 0, 14, 12, 14, 14, 14, 15, 2, 14, 14, 14, 16, 26, 14, 15, 15, 21},
{0, 0, 0, 4, 0, 14, 0, 0, 2, 14, 0, 2, 27, 2, 14, 0, 0, 0, 14, 0, 0, 28, 14, 2, 14, 0, 14, 16, 14, 0, 3, 14, 0, 0, 14, 0, 12, 16, 27, 0, 14, 3, 12, 26, 3, 2, 14, 0, 0, 0, 14, 0, 14, 0, 0, 2, 14, 0, 0, 28, 14, 0, 26, 15, 0, 0, 14}
},
{
{0, 0, 14, 20, 14, 14, 15, 20, 20, 15, 15, 20, 14, 20, 20, 20, 20, 14, 14, 20, 15, 20, 14, 14, 15, 14, 20, 20, 20, 20, 20, 15, 14, 15, 15, 20, 14, 20, 15, 15, 15, 15, 15, 14, 20, 14, 14, 15, 20, 15, 14, 14, 15, 15, 15, 15, 14, 15, 20, 20, 15, 15, 20, 14, 14, 15, 14},
{0, 0, 14, 14, 14, 14, 14, 21, 15, 14, 14, 15, 14, 15, 15, 15, 14, 15, 20, 15, 14, 15, 14, 15, 20, 14, 14, 14, 14, 15, 15, 14, 14, 14, 15, 21, 14, 14, 21, 15, 14, 20, 15, 15, 14, 14, 14, 21, 20, 15, 15, 15, 14, 14, 15, 14, 14, 14, 20, 14, 20, 21, 14, 14, 14, 14, 14},
{0, 0, 21, 14, 21, 20, 20, 20, 14, 20, 20, 20, 14, 14, 21, 14, 14, 20, 20, 15, 14, 14, 20, 15, 20, 20, 21, 20, 21, 20, 21, 20, 14, 21, 21, 21, 20, 21, 20, 14, 20, 21, 20, 20, 20, 20, 14, 27, 14, 20, 21, 14, 20, 20, 20, 20, 14, 20, 21, 21, 20, 20, 20, 21, 21, 14, 21},
{0, 0, 3, 21, 20, 20, 2, 21, 21, 14, 21, 3, 2, 20, 22, 14, 21, 13, 15, 12, 25, 20, 20, 27, 21, 21, 21, 21, 21, 3, 21, 20, 28, 21, 21, 22, 21, 18, 20, 20, 20, 20, 20, 22, 20, 20, 20, 21, 22, 0, 3, 22, 20, 18, 16, 20, 20, 15, 21, 20, 21, 20, 16, 20, 20, 21, 21},
{0, 0, 15, 14, 14, 14, 15, 14, 14, 14, 14, 14, 14, 20, 15, 14, 20, 20, 15, 12, 14, 12, 14, 14, 14, 14, 20, 14, 15, 14, 15, 14, 20, 14, 0, 14, 15, 14, 20, 14, 20, 14, 15, 14, 14, 14, 14, 2, 14, 2, 20, 15, 15, 14, 20, 15, 14, 14, 14, 14, 14, 14, 20, 14, 14, 14, 20},
{0, 0, 21, 14, 20, 14, 14, 21, 21, 15, 20, 14, 15, 20, 20, 14, 14, 14, 21, 18, 15, 20, 20, 20, 20, 14, 15, 20, 20, 20, 20, 20, 20, 21, 21, 14, 20, 20, 20, 20, 20, 15, 21, 20, 20, 20, 20, 20, 14, 15, 21, 20, 14, 15, 14, 14, 20, 14, 20, 20, 20, 14, 14, 15, 14, 20, 21},
{0, 0, 3, 16, 4, 14, 14, 20, 21, 14, 14, 20, 14, 27, 20, 21, 14, 16, 21, 26, 14, 24, 20, 20, 27, 14, 14, 20, 3, 26, 14, 20, 20, 16, 3, 20, 20, 20, 15, 3, 15, 14, 21, 20, 14, 15, 20, 14, 20, 14, 3, 2, 14, 3, 20, 20, 20, 15, 20, 15, 15, 29, 21, 14, 20, 26, 3},
{0, 0, 21, 14, 14, 14, 14, 14, 20, 20, 14, 20, 14, 15, 14, 20, 21, 14, 21, 15, 15, 18, 15, 14, 15, 14, 14, 14, 20, 21, 14, 20, 14, 21, 21, 21, 14, 15, 14, 21, 15, 14, 14, 14, 20, 14, 20, 3, 20, 20, 21, 14, 21, 15, 14, 20, 14, 15, 14, 15, 15, 14, 14, 14, 14, 14, 21},
{0, 0, 15, 14, 14, 14, 14, 27, 21, 4, 2, 14, 15, 20, 27, 15, 28, 14, 21, 14, 15, 3, 20, 21, 27, 21, 20, 14, 21, 14, 14, 20, 14, 14, 20, 20, 26, 20, 20, 15, 15, 3, 14, 14, 26, 20, 14, 16, 9, 3, 21, 32, 12, 18, 4, 15, 20, 14, 21, 27, 26, 20, 14, 20, 14, 14, 21},
{0, 0, 15, 14, 14, 14, 14, 14, 15, 14, 14, 14, 14, 14, 20, 22, 26, 1, 0, 16, 24, 2, 15, 14, 14, 14, 20, 14, 14, 14, 14, 14, 14, 14, 2, 14, 14, 14, 14, 14, 14, 14, 15, 14, 14, 14, 20, 12, 4, 26, 0, 6, 16, 27, 15, 14, 14, 14, 14, 14, 20, 14, 14, 14, 14, 14, 20},
{0, 0, 0, 14, 2, 14, 26, 0, 14, 0, 26, 32, 2, 2, 16, 0, 0, 0, 14, 0, 0, 14, 14, 14, 16, 0, 14, 0, 2, 0, 5, 15, 0, 0, 2, 0, 0, 2, 12, 0, 2, 0, 22, 0, 26, 16, 30, 0, 0, 0, 20, 0, 0, 0, 0, 14, 14, 14, 26, 2, 20, 0, 16, 2, 0, 14, 14}
}
};
#endif

const uint8_t g_aucIpmToTrSet[ 16 ][ 36 ] =
{
  //0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 MIP
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4 }, //4x4
  { 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9 }, //4x8
  {10,10,11,11,11,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,14 }, //4x16
  {15,15,16,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,18,18,18,18,18,19 }, //4x32
  {20,20,21,21,21,21,21,21,21,21,21,21,21,22,22,22,22,22,22,22,22,22,22,22,23,23,23,23,23,23,23,23,23,23,23,24 }, //8x4
  {25,25,26,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,27,27,27,28,28,28,28,28,28,28,28,28,28,28,29 }, //8x8
  {30,30,31,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,33,33,33,34 }, //8x16
  {35,35,36,36,36,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,38,38,39 }, //8x32
  {40,40,41,41,41,41,41,41,41,41,41,41,41,42,42,42,42,42,42,42,42,42,42,42,43,43,43,43,43,43,43,43,43,43,43,44 }, //16x4
  {45,45,46,46,46,46,46,46,46,46,46,46,46,47,47,47,47,47,47,47,47,47,47,47,48,48,48,48,48,48,48,48,48,48,48,49 }, //16x8
  {50,50,51,51,51,51,51,51,51,51,51,51,51,52,52,52,52,52,52,52,52,52,52,52,53,53,53,53,53,53,53,53,53,53,53,54 }, //16x16
  {55,55,56,56,56,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,58,58,58,58,59 }, //16x32
  {60,60,61,61,61,61,61,61,61,61,61,61,61,62,62,62,62,62,62,62,62,62,62,62,63,63,63,63,63,63,63,63,63,63,63,64 }, //32x4
  {65,65,66,66,66,66,66,66,66,66,66,66,66,67,67,67,67,67,67,67,67,67,67,67,68,68,68,68,68,68,68,68,68,68,68,69 }, //32x8
  {70,70,71,71,71,71,71,71,71,71,71,71,71,72,72,72,72,72,72,72,72,72,72,72,73,73,73,73,73,73,73,73,73,73,73,74 }, //32x16
  {75,75,76,76,76,76,76,76,76,76,76,76,76,77,77,77,77,77,77,77,77,77,77,77,78,78,78,78,78,78,78,78,78,78,78,79 }, //32x32
};

#if JVET_AJ0257_IMPLICIT_MTS_LUT
const uint8_t g_aucImplicitToTrSet[ 16 ][ 35 ] =
{
  { 14,15,14,14,14,14,14,14,14,14,20,20,14,20,14,20,20,20,20,20,20,20,20,20,20,20,14,20,20,20,20,14,14,14,14},
  { 14,15,14,14,14,14,14,14,14,14,14,14,14,20,14,20,20,20,20,20,21,21,14,20,15,14,14,20,14,14,14,14,14,14,14},
  { 14,15,20,15,14,15,14,15,15,15,15,15,15,15,15,15,15,21,15,21,21,21,21,21,15,21,15,15,21,15,14,15,14,21,15},
  { 21,15,21,15,15,21,15,18,12,15,15,15,21,18,21,21,18,21,21,18,12,18,18,12,18,21,18,18,18,21,21,15,15,18,15},
  { 14,14,14,14,14,20,20, 2,14,14,14,14,14,14,14,14,14,14,14,20,15,20,14,14,14,14,15,14,14,14,14,14,14,14,14},
  { 14,14,14,15,14,14,14,14,14,14,14,15,15,15,15,15,15,21,20,21,21,20,15,15,14,15,14,14,14,14,14,14,14,14,14},
  { 14,21,14,15,14,15,14,15,15,15,14,15,14,15,15,21,15,21,21,21,21,21,21,21,21,15,15,21,15,21,14,15,14,20,14},
  { 20,21,15,15,15,15,15,15,21,15,21,15,15,15,21,21,18,21,18,18,18,15,21,21,21,21,21,15,21,15,15,15,21,21,18},
  { 14,20,15,15,15,15,15,15,21,15,21,15,14,14,15,15,14,20,20,18,18,18,14,14,14,14,14,14,15,21,15,14,20,21,14},
  { 14,21,14,12,14,15,14,15,14,15,14,15,14,14,15,15,21,20,21,21,21,20,15,20,14,14,14,20,14,20,14,20,14,14,14},
  { 14,15,14,12,14,15,14,15,14,21,14,21,15,21,14,21,14,21,21,18,21,21,15,21,15,21,14,21,14,14,14,21,14,21,14},
  { 14,12,14,15,14,15,14,15,15,15,15,15,18,21,21,18,21,18,21,18,18,15,15,21, 3,12,21, 0,12, 3,18,12,14, 3,18},
  { 12, 2,20,21,21, 3,20,20,20,21,20, 3,20, 3,21, 2,21,15,21,21,21,20,21,21,21,21,20, 2,14, 2,20,21, 3,21,20},
  { 12,21, 3,20,20,21,14,20,20,21,20, 2,21,21, 3,21, 3,21,21, 0,21,18, 3, 3,21,20,18,20,21,20, 2,14, 2,15,20},
  { 12,21,20,20, 2, 3,20,21, 3, 0,18,21,15,15,21,21, 3,21, 0, 0, 3,21,15, 3, 3, 0,14, 0, 0, 3, 3, 3,21,18,20},
  { 12,18,14,18, 0,15,14, 0, 0,21,15,18,21,18,12, 3,21, 0, 3, 0, 0,18, 0, 3,12, 0, 0, 0, 3, 0,12, 0, 3, 3,18},
};

const uint8_t g_aucImplicitTrIdxToTr[ 36 ][ 2 ] =
{
    { DCT2, DCT2 }, { DCT2, DCT8 },{ DCT2, DST7 },{ DCT2, DCT5 },{ DCT2, DST4 }, { DCT2, DST1 },
    { DCT8, DCT2 }, { DCT8, DCT8 },{ DCT8, DST7 },{ DCT8, DCT5 },{ DCT8, DST4 }, { DCT8, DST1 },
    { DST7, DCT2 }, { DST7, DCT8 },{ DST7, DST7 },{ DST7, DCT5 },{ DST7, DST4 }, { DST7, DST1 },
    { DCT5, DCT2 }, { DCT5, DCT8 },{ DCT5, DST7 },{ DCT5, DCT5 },{ DCT5, DST4 }, { DCT5, DST1 },
    { DST4, DCT2 }, { DST4, DCT8 },{ DST4, DST7 },{ DST4, DCT5 },{ DST4, DST4 }, { DST4, DST1 },
    { DST1, DCT2 }, { DST1, DCT8 },{ DST1, DST7 },{ DST1, DCT5 },{ DST1, DST4 }, { DST1, DST1 },
};
#endif

const int8_t g_aiIdLut[ 3 ][ 3 ] =
{
  { 8, 6, 4 },{ 8, 8, 6 },{ 4, 2, -1 }
};
const uint8_t g_aucTrIdxToTr[ 25 ][ 2 ] =
{
    { DCT8, DCT8 },{ DCT8, DST7 },{ DCT8, DCT5 },{ DCT8, DST4 }, {DCT8, DST1},
    { DST7, DCT8 },{ DST7, DST7 },{ DST7, DCT5 },{ DST7, DST4 }, {DST7, DST1},
    { DCT5, DCT8 },{ DCT5, DST7 },{ DCT5, DCT5 },{ DCT5, DST4 }, {DCT5, DST1},
    { DST4, DCT8 },{ DST4, DST7 },{ DST4, DCT5 },{ DST4, DST4 }, {DST4, DST1},
    { DST1, DCT8 },{ DST1, DST7 },{ DST1, DCT5 },{ DST1, DST4 }, {DST1, DST1},
};
#if JVET_Y0142_ADAPT_INTRA_MTS
const uint8_t g_aucTrSet[ 80 ][ 6 ] =
{
  //T0:0,  1,  2,  3,  4,  5,
  { 18, 24, 17, 23, 8, 12},
  //T1:0,  1,  2,  3,  4,  5,
  { 18, 3, 7, 22,  0, 16},
  //T2:0,  1,  2,  3,  4,  5,
  { 18, 2, 17, 22, 3, 23},
  //T3:0,  1,  2,  3,  4,  5,
  { 18, 3, 15, 17, 12, 23},
  //T4:0,  1,  2,  3,  4,  5,
  { 18, 12, 3, 19, 10, 13},
  //T5:0,  1,  2,  3,  4,  5,
  { 18, 12, 19, 23, 13, 24},
  //T6:0,  1,  2,  3,  4,  5,
  { 18, 12, 17, 2, 3, 23},
  //T7:0,  1,  2,  3,  4,  5,
  { 18, 2, 17, 22, 12, 23},
  //T8:0,  1,  2,  3,  4,  5,
  { 18, 2, 11, 17, 22, 23},
  //T9:0,  1,  2,  3,  4,  5,
  { 18, 12, 19, 23, 3, 10},
  //T10:0,  1,  2,  3,  4, 5,
  { 16, 12, 13, 24, 7, 8},
  //T11:0,  1,  2,  3,  4,  5,
  { 16, 2, 11, 23, 12, 18},
  //T12:0,  1,  2,  3,  4,  5,
  { 13, 17, 2, 22, 12, 18},
  //T13:0,  1,  2,  3,  4,  5,
  { 17, 11, 2, 21, 12, 18},
  //T14:0,  1,  2,  3,  4,  5,
  { 16, 13, 19, 22, 3, 10},
  //T15:0,  1,  2,  3,  4,  5,
  { 18, 12, 13, 7, 14, 22},
  //T16:0,  1,  2,  3,  4,  5,
  { 16, 12, 11, 1, 18, 22},
  //T17:0,  1,  2,  3,  4,  5,
  { 17, 13,  3, 22, 12, 18},
  //T18:0,  1,  2,  3,  4,  5,
  {  6, 12,  1, 22, 13, 17},
  //T19:0,  1,  2,  3,  4,  5,
  { 16, 12, 13, 15, 2, 23},
  //T20:0,  1,  2,  3,  4,  5,
  { 18, 24, 23, 19, 12, 17},
  //T21:0,  1,  2,  3,  4,  5,
  { 18, 24, 2,  17,  0, 23},
  //T22:0,  1,  2,  3,  4,  5,
  { 17, 3,  4, 22, 2, 13},
  //T23:0,  1,  2,  3,  4,  5,
  { 18, 12, 19, 23, 3, 15},
  //T24:0,  1,  2,  3,  4,  5,
  { 18, 12, 19, 23, 3, 10},
  //T25:0,  1,  2,  3,  4,  5,
  {  6, 12, 18, 24, 13, 19},
  //T26:0,  1,  2,  3,  4,  5,
  {  6, 12,  2, 21, 13, 18},
  //T27:0,  1,  2,  3,  4,  5,
  { 17, 11,  1, 22, 2, 18},
  //T28:0,  1,  2,  3,  4,  5,
  { 16, 17, 3,  11, 12, 23},
  //T29:0,  1,  2,  3,  4,  5,
  {  8, 12, 19, 23, 11, 24},
  //T30:0,  1,  2,  3,  4,  5,
  { 16, 13, 7, 23, 12, 19},
  //T31:0,  1,  2,  3,  4,  5,
  {  6, 12, 1, 11, 18, 22},
  //T32:0,  1,  2,  3,  4,  5,
  { 17, 11, 1, 21, 12, 18},
  //T33:0,  1,  2,  3,  4,  5,
  {  6, 11, 17, 21, 12, 18},
  //T34:0,  1,  2,  3,  4,  5,
  {  8, 11, 14, 17, 12, 22},
  //T35:0,  1,  2,  3,  4,  5,
  {  6, 12, 11, 21, 14, 16},
  //T36:0,  1,  2,  3,  4,  5,
  {  6, 12, 11,  1, 17, 21},
  //T37:0,  1,  2,  3,  4,  5,
  {  6, 12, 11,  2, 17, 21},
  //T38:0,  1,  2,  3,  4,  5,
  {  6, 11, 21,  1, 12, 17},
  //T39:0,  1,  2,  3,  4,  5,
  {  16, 12, 11, 7, 1, 5},
  //T40:0,  1,  2,  3,  4,  5,
  {  8, 12, 19, 24, 11, 17},
  //T41:0,  1,  2,  3,  4,  5,
  { 18, 13, 1,  22, 2, 24},
  //T42:0,  1,  2,  3,  4,  5,
  {  6, 2, 17, 21, 19, 22},
  //T43:0,  1,  2,  3,  4,  5,
  { 16, 12, 11, 19, 8, 15},
  //T44:0,  1,  2,  3,  4,  5,
  {  8, 12, 17, 24, 13, 15},
  //T45:0,  1,  2,  3,  4,  5,
  {  6, 12, 19, 21, 17, 18},
  //T46:0,  1,  2,  3,  4,  5,
  {  6, 12, 13, 21, 2, 18},
  //T47:0,  1,  2,  3,  4,  5,
  { 16, 2,  17, 21, 1, 11},
  //T48:0,  1,  2,  3,  4,  5,
  {  6, 17, 19, 23, 12, 16},
  //T49:0,  1,  2,  3,  4,  5,
  {  6, 12, 14, 17, 8, 22},
  //T50:0,  1,  2,  3,  4,  5,
  {  6,  7, 11, 21, 9, 12},
  //T51:0,  1,  2,  3,  4,  5,
  { 16, 12, 11,  1, 7, 21},
  //T52:0,  1,  2,  3,  4,  5,
  {  6, 12, 11, 1, 17, 21},
  //T53:0,  1,  2,  3,  4,  5,
  {  6, 12, 11, 21, 1, 16},
  //T54:0,  1,  2,  3,  4,  5,
  {  8,  7,  9, 11, 12, 21},
  //T55:0,  1,  2,  3,  4,  5,
  {  6, 12, 7,  11, 14, 21},
  //T56:0,  1,  2,  3,  4,  5,
  {  6, 12, 7,  11, 1, 21},
  //T57:0,  1,  2,  3,  4,  5,
  { 16, 12, 11, 1, 2, 21},
  //T58:0,  1,  2,  3,  4,  5,
  {  6, 11, 17, 21, 1, 12},
  //T59:0,  1,  2,  3,  4,  5,
  {  6, 12, 7,  11, 9, 21},
  //T60:0,  1,  2,  3,  4,  5,
  { 18, 12, 14, 21, 6, 21},
  //T61:0,  1,  2,  3,  4,  5,
  { 16, 11, 1, 22,  2, 17},
  //T62:0,  1,  2,  3,  4,  5,
  { 16, 11, 1, 22,  2, 17},
  //T63:0,  1,  2,  3,  4,  5,
  { 16, 13, 15, 7, 14, 19},
  //T64:0,  1,  2,  3,  4,  5,
  {  8, 12, 1, 19, 16, 23},
  //T65:0,  1,  2,  3,  4,  5,
  {  6, 12, 7,  9, 13, 21},
  //T66:0,  1,  2,  3,  4,  5,
  {  6, 12, 13, 2,  7, 18},
  //T67:0,  1,  2,  3,  4,  5,
  { 16, 12, 1, 21, 11, 17},
  //T68:0,  1,  2,  3,  4,  5,
  { 16, 11, 7, 19, 12, 15},
  //T69:0,  1,  2,  3,  4,  5,
  {  8, 12, 7, 11, 14, 21},
  //T70:0,  1,  2,  3,  4,  5,
  {  6, 12, 7, 11, 8, 9},
  //T71:0,  1,  2,  3,  4,  5,
  { 6, 12,  7, 11, 2, 21},
  //T72:0,  1,  2,  3,  4,  5,
  {  6, 12, 1, 11, 21, 22},
  //T73:0,  1,  2,  3,  4,  5,
  {  6,  7, 11, 16, 9, 12},
  //T74:0,  1,  2,  3,  4,  5,
  { 6,  12,  7, 11, 9, 21},
  //T75:0,  1,  2,  3,  4,  5,
  { 6, 12,  7, 11, 13, 17},
  //T76:0,  1,  2,  3,  4,  5,
  {  6, 12, 11, 21, 2, 7},
  //T77:0,  1,  2,  3,  4,  5,
  {  6, 12, 1,  11, 2, 7},
  //T78:0,  1,  2,  3,  4,  5,
  { 6,  12, 7, 11, 16, 21},
  //T79:0,  1,  2,  3,  4,  5,
  { 6,  12, 7, 11, 9, 16},
};
#else
const uint8_t g_aucTrSet[ 80 ][ 4 ] =
{
  //T0:0,  1,  2,  3,
  { 17, 18, 23, 24},
  //T1:0,  1,  2,  3,
  {  3,  7, 18, 22},
  //T2:0,  1,  2,  3, 
  {  2, 17, 18, 22},
  //T3:0,  1,  2,  3,  
  {  3, 15, 17, 18},
  //T4:0,  1,  2,  3, 
  {  3, 12, 18, 19},
  //T5:0,  1,  2,  3, 
  { 12, 18, 19, 23},
  //T6:0,  1,  2,  3,
  {  2, 12, 17, 18},
  //T7:0,  1,  2,  3, 
  {  2, 17, 18, 22},
  //T8:0,  1,  2,  3,  
  {  2, 11, 17, 18},
  //T9:0,  1,  2,  3, 
  { 12, 18, 19, 23},
  //T10:0,  1, 2,   3, 
  { 12, 13, 16, 24},
  //T11:0,  1,  2,  3, 
  {  2, 11, 16, 23},
  //T12:0,  1,  2,  3, 
  {  2, 13, 17, 22},
  //T13:0,  1,  2,  3,
  {  2, 11, 17, 21},
  //T14:0,  1,  2,  3, 
  { 13, 16, 19, 22},
  //T15:0,  1,  2,  3,
  {  7, 12, 13, 18},
  //T16:0,  1,  2,  3, 
  {  1, 11, 12, 16},
  //T17:0,  1,  2,  3,
  {  3, 13, 17, 22},
  //T18:0,  1,  2,  3,  
  {  1,  6, 12, 22},
  //T19:0,  1,  2,  3,  
  { 12, 13, 15, 16},
  //T20:0,  1,  2,  3,  
  { 18, 19, 23, 24},
  //T21:0,  1,  2,  3, 
  {  2, 17, 18, 24},
  //T22:0,  1,  2,  3,
  {  3,  4, 17, 22},
  //T23:0,  1,  2,  3, 
  { 12, 18, 19, 23},
  //T24:0,  1,  2,  3,
  { 12, 18, 19, 23},
  //T25:0,  1,  2,  3,  
  {  6, 12, 18, 24},
  //T26:0,  1,  2,  3, 
  {  2,  6, 12, 21},
  //T27:0,  1,  2,  3, 
  {  1, 11, 17, 22},
  //T28:0,  1,  2,  3, 
  {  3, 11, 16, 17},
  //T29:0,  1,  2,  3, 
  {  8, 12, 19, 23},
  //T30:0,  1,  2,  3, 
  {  7, 13, 16, 23},
  //T31:0,  1,  2,  3, 
  {  1,  6, 11, 12},
  //T32:0,  1,  2,  3,
  {  1, 11, 17, 21},
  //T33:0,  1,  2,  3,
  {  6, 11, 17, 21},
  //T34:0,  1,  2,  3,
  {  8, 11, 14, 17},
  //T35:0,  1,  2,  3, 
  {  6, 11, 12, 21},
  //T36:0,  1,  2,  3, 
  {  1,  6, 11, 12},
  //T37:0,  1,  2,  3, 
  {  2,  6, 11, 12},
  //T38:0,  1,  2,  3,
  {  1,  6, 11, 21},
  //T39:0,  1,  2,  3,
  {  7, 11, 12, 16},
  //T40:0,  1,  2,  3,
  {  8, 12, 19, 24},
  //T41:0,  1,  2,  3,
  {  1, 13, 18, 22},
  //T42:0,  1,  2,  3,
  {  2,  6, 17, 21},
  //T43:0,  1,  2,  3,
  { 11, 12, 16, 19},
  //T44:0,  1,  2,  3,
  {  8, 12, 17, 24},
  //T45:0,  1,  2,  3,
  {  6, 12, 19, 21},
  //T46:0,  1,  2,  3,
  {  6, 12, 13, 21},
  //T47:0,  1,  2,  3,
  {  2, 16, 17, 21},
  //T48:0,  1,  2,  3,
  {  6, 17, 19, 23},
  //T49:0,  1,  2,  3,
  {  6, 12, 14, 17},
  //T50:0,  1,  2,  3,
  {  6,  7, 11, 21},
  //T51:0,  1,  2,  3,  
  {  1, 11, 12, 16},
  //T52:0,  1,  2,  3,
  {  1,  6, 11, 12},
  //T53:0,  1,  2,  3,
  {  6, 11, 12, 21},
  //T54:0,  1,  2,  3,
  {  7,  8,  9, 11},
  //T55:0,  1,  2,  3,
  {  6,  7, 11, 12},
  //T56:0,  1,  2,  3,
  {  6,  7, 11, 12},
  //T57:0,  1,  2,  3,
  {  1, 11, 12, 16},
  //T58:0,  1,  2,  3,
  {  6, 11, 17, 21},
  //T59:0,  1,  2,  3,
  {  6,  7, 11, 12},
  //T60:0,  1,  2,  3,
  { 12, 14, 18, 21},
  //T61:0,  1,  2,  3, 
  {  1, 11, 16, 22},
  //T62:0,  1,  2,  3,
  {  1, 11, 16, 22},
  //T63:0,  1,  2,  3, 
  {  7, 13, 15, 16},
  //T64:0,  1,  2,  3, 
  {  1,  8, 12, 19},
  //T65:0,  1,  2,  3,
  {  6,  7,  9, 12},
  //T66:0,  1,  2,  3,  
  {  2,  6, 12, 13},
  //T67:0,  1,  2,  3,  
  {  1, 12, 16, 21},
  //T68:0,  1,  2,  3, 
  {  7, 11, 16, 19},
  //T69:0,  1,  2,  3, 
  {  7,  8, 11, 12},
  //T70:0,  1,  2,  3,  
  {  6,  7, 11, 12},
  //T71:0,  1,  2,  3,  
  {  6,  7, 11, 12},
  //T72:0,  1,  2,  3,  
  {  1,  6, 11, 12},
  //T73:0,  1,  2,  3, 
  {  6,  7, 11, 16},
  //T74:0,  1,  2,  3,  
  {  6,  7, 11, 12},
  //T75:0,  1,  2,  3,  
  {  6,  7, 11, 12},
  //T76:0,  1,  2,  3, 
  {  6, 11, 12, 21},
  //T77:0,  1,  2,  3,
  {  1,  6, 11, 12},
  //T78:0,  1,  2,  3, 
  {  6,  7, 11, 12},
  //T79:0,  1,  2,  3, 
  {  6,  7, 11, 12},
};
#endif
#endif

#if JVET_AG0058_EIP
#if JVET_AH0086_EIP_BIAS_AND_CLIP
// Note: Positions here are identical to the definition in the #else branch, just omitting the last position from each array
const Position g_eipFilter[ NUM_EIP_SHAPE ][ EIP_FILTER_TAP - 1 ] =
{
#if JVET_AL0205_EIP_FILTER_SHAPES
  { Position(-1, 0), Position(-2, 0), Position(-1, -1), Position(0, -1), Position(-2, -1), Position(0, -2), Position(-1, -2), Position(0, -3), Position(-3, 0), Position(-2, -2), Position(-2, 1), Position(1, -2), Position(1, -3), Position(-3, 1) },
  { Position(-1,  0), Position(0, -1), Position(-1, -1), Position(0, -2), Position(-1, -2), Position(0, -3), Position(-1, -3), Position(0, -4), Position(-1, -4), Position(0, -5), Position(0, -6), Position(0, -7), Position(1, -3), Position(1, -2) },
  { Position(0, -1), Position(-1,  0), Position(-1, -1), Position(-2,  0), Position(-2, -1), Position(-3,  0), Position(-3, -1), Position(-4,  0), Position(-4, -1), Position(-5,  0), Position(-6,  0), Position(-7, 0), Position(-2,  1), Position(-3, 1) },
#else
  { Position( -1,  0 ), Position( -2,  0 ), Position( -3,  0 ), Position( 0, -1 ), Position( -1, -1 ), Position( -2, -1 ), Position( -3, -1 ), Position( 0, -2 ), Position( -1, -2 ), Position( -2, -2 ), Position( -3, -2 ), Position( 0, -3 ), Position( -1, -3 ), Position( -2, -3 ) },
  { Position( -1,  0 ), Position( 0, -1 ), Position( -1, -1 ), Position( 0, -2 ), Position( -1, -2 ), Position( 0, -3 ), Position( -1, -3 ), Position( 0, -4 ), Position( -1, -4 ), Position( 0, -5 ), Position( -1, -5 ), Position( 0, -6 ), Position( -1, -6 ), Position( 0, -7 ) },
  { Position( 0, -1 ), Position( -1,  0 ), Position( -1, -1 ), Position( -2,  0 ), Position( -2, -1 ), Position( -3,  0 ), Position( -3, -1 ), Position( -4,  0 ), Position( -4, -1 ), Position( -5,  0 ), Position( -5, -1 ), Position( -6,  0 ), Position( -6, -1 ), Position( -7,  0 ) },
#endif
};
#else
const Position g_eipFilter[ NUM_EIP_SHAPE ][ EIP_FILTER_TAP ] =
{
  { Position( -1,  0 ), Position( -2,  0 ), Position( -3,  0 ), Position( 0, -1 ), Position( -1, -1 ), Position( -2, -1 ), Position( -3, -1 ), Position( 0, -2 ), Position( -1, -2 ), Position( -2, -2 ), Position( -3, -2 ), Position( 0, -3 ), Position( -1, -3 ), Position( -2, -3 ), Position( -3, -3 ) },
  { Position( -1,  0 ), Position( 0, -1 ), Position( -1, -1 ), Position( 0, -2 ), Position( -1, -2 ), Position( 0, -3 ), Position( -1, -3 ), Position( 0, -4 ), Position( -1, -4 ), Position( 0, -5 ), Position( -1, -5 ), Position( 0, -6 ), Position( -1, -6 ), Position( 0, -7 ), Position( -1, -7 ) },
  { Position( 0, -1 ), Position( -1,  0 ), Position( -1, -1 ), Position( -2,  0 ), Position( -2, -1 ), Position( -3,  0 ), Position( -3, -1 ), Position( -4,  0 ), Position( -4, -1 ), Position( -5,  0 ), Position( -5, -1 ), Position( -6,  0 ), Position( -6, -1 ), Position( -7,  0 ), Position( -7, -1 ) },
};
#endif
#if JVET_AJ0082_MM_EIP
const EIPInfo g_eipInfoLut[ 4 ][ 4 ][ 9 ] =
{
  {
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x4, 0modes
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x8, 0modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },// 4x16, 1modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },// 4x32, 1modes
  },
  {
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x4, 0modes
#if JVET_AL0205_EIP_FILTER_SHAPES
    { EIPInfo( EIP_AL_L, EIP_FILTER_S), EIPInfo(EIP_AL_A, EIP_FILTER_S), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x8, 1modes
#else
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x8, 1modes
#endif
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x16, 1modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x32, 5modes
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 16x4, 1modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()}, // 16x8, 1modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()}, // 16x16, 9modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()}, // 16x32, 9modes
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
  },
};
const EIPInfo g_mmEipInfoLut[ 4 ][ 4 ][ 9 ] =
{
  {
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
  },
  {
    { EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x4, 0modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() },
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A, EIP_FILTER_S ), EIPInfo( EIP_AL_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A, EIP_FILTER_S ), EIPInfo( EIP_AL_L, EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo()},
  },
};
#else
const EIPInfo g_eipInfoLut[ 4 ][ 4 ][ 9 ] =
{
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x4, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x8, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x16, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 4x32, 2modes
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x4, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x8, 5modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x16, 5modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 8x32, 5modes
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 16x4, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 16x8, 5modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo( EIP_AL_L,   EIP_FILTER_H ), EIPInfo( EIP_AL_A,   EIP_FILTER_V ) }, // 16x16, 9modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo( EIP_AL_L,   EIP_FILTER_H ), EIPInfo( EIP_AL_A,   EIP_FILTER_V ) }, // 16x32, 9modes
  },
  {
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 32x4, 3modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo(), EIPInfo(), EIPInfo(), EIPInfo() }, // 32x8, 5modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo( EIP_AL_A,   EIP_FILTER_V ), EIPInfo( EIP_AL_L,   EIP_FILTER_H ) }, // 32x16, 9modes
    { EIPInfo( EIP_AL_A_L, EIP_FILTER_S ), EIPInfo( EIP_AL_A_L, EIP_FILTER_V ), EIPInfo( EIP_AL_A_L, EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_S ), EIPInfo( EIP_AL_A,   EIP_FILTER_H ), EIPInfo( EIP_AL_L,   EIP_FILTER_V ), EIPInfo( EIP_AL_L,   EIP_FILTER_H ), EIPInfo( EIP_AL_A,   EIP_FILTER_V ) }, // 32x32, 9modes
  },
};
#endif
#endif

// initialize ROM variables
void initROM()
{
#if LMS_LINEAR_MODEL || TRANSFORM_SIMD_OPT
  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset( g_aucLog2, 0, sizeof( g_aucLog2 ) );
  int c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
#if LMS_LINEAR_MODEL
    g_aucNextLog2[ i ] = i <= 1 ? 0 : c + 1;
#endif

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

#if LMS_LINEAR_MODEL
    g_aucPrevLog2[ i ] = c;
#endif
    g_aucLog2[ i ] = c;
  }
#elif MMLM
  int c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[ i ] = c;
  }
#elif TU_256
  int c = 0;
#endif

  gp_sizeIdxInfo = new SizeIndexInfoLog2();
  gp_sizeIdxInfo->init( MAX_CU_SIZE );


  SizeIndexInfoLog2 sizeInfo;
  sizeInfo.init( MAX_CU_SIZE );

  // initialize scan orders
  for( uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++ )
  {
    for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
    {
      const uint32_t blockWidth = sizeInfo.sizeFrom( blockWidthIdx );
      const uint32_t blockHeight = sizeInfo.sizeFrom( blockHeightIdx );
      const uint32_t totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for( uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++ )
      {
        const CoeffScanType scanType = CoeffScanType( scanTypeIndex );
        ScanElement* scan = nullptr;

        if( blockWidthIdx < sizeInfo.numWidths() && blockHeightIdx < sizeInfo.numHeights() )
        {
          scan = new ScanElement[ totalValues ];
        }

        g_scanOrder[ SCAN_UNGROUPED ][ scanType ][ blockWidthIdx ][ blockHeightIdx ] = scan;

        if( scan == nullptr )
        {
          continue;
        }

        ScanGenerator fullBlockScan( blockWidth, blockHeight, blockWidth, scanType );

        for( uint32_t scanPosition = 0; scanPosition < totalValues; scanPosition++ )
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY = rasterPos / blockWidth;
          const int posX = rasterPos - ( posY * blockWidth );

          scan[ scanPosition ].idx = rasterPos;
          scan[ scanPosition ].x = posX;
          scan[ scanPosition ].y = posY;
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
      const uint32_t* log2Sbb = g_log2SbbSize[ floorLog2( blockWidth ) ][ floorLog2( blockHeight ) ];
      const uint32_t  log2CGWidth = log2Sbb[ 0 ];
      const uint32_t  log2CGHeight = log2Sbb[ 1 ];

      const uint32_t  groupWidth = 1 << log2CGWidth;
      const uint32_t  groupHeight = 1 << log2CGHeight;
      const uint32_t  widthInGroups = std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockWidth ) >> log2CGWidth;
      const uint32_t  heightInGroups = std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockHeight ) >> log2CGHeight;

      const uint32_t  groupSize = groupWidth * groupHeight;
      const uint32_t  totalGroups = widthInGroups * heightInGroups;

      for( uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++ )
      {
        const CoeffScanType scanType = CoeffScanType( scanTypeIndex );

        ScanElement* scan = new ScanElement[ totalValues ];

        g_scanOrder[ SCAN_GROUPED_4x4 ][ scanType ][ blockWidthIdx ][ blockHeightIdx ] = scan;

        if( blockWidth > JVET_C0024_ZERO_OUT_TH || blockHeight > JVET_C0024_ZERO_OUT_TH )
        {
          for( uint32_t i = 0; i < totalValues; i++ )
          {
            scan[ i ].idx = totalValues - 1;
            scan[ i ].x = blockWidth - 1;
            scan[ i ].y = blockHeight - 1;
          }
        }

        ScanGenerator fullBlockScan( widthInGroups, heightInGroups, groupWidth, scanType );

        for( uint32_t groupIndex = 0; groupIndex < totalGroups; groupIndex++ )
        {
          const uint32_t groupPositionY = fullBlockScan.GetCurrentY();
          const uint32_t groupPositionX = fullBlockScan.GetCurrentX();
          const uint32_t groupOffsetX = groupPositionX * groupWidth;
          const uint32_t groupOffsetY = groupPositionY * groupHeight;
          const uint32_t groupOffsetScan = groupIndex * groupSize;

          ScanGenerator groupScan( groupWidth, groupHeight, blockWidth, scanType );

          for( uint32_t scanPosition = 0; scanPosition < groupSize; scanPosition++ )
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY = rasterPos / blockWidth;
            const int posX = rasterPos - ( posY * blockWidth );

            scan[ groupOffsetScan + scanPosition ].idx = rasterPos;
            scan[ groupOffsetScan + scanPosition ].x = posX;
            scan[ groupOffsetScan + scanPosition ].y = posY;
          }

          fullBlockScan.GetNextIndex( 0, 0 );
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }

  // initialize CoefTopLeftDiagScan8x8 for LFNST
  for( uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++ )
  {
    const uint32_t blockWidth = sizeInfo.sizeFrom( blockWidthIdx );

    const static uint8_t g_auiXYDiagScan8x8[ 64 ][ 2 ] =
    {
      { 0, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 1 }, { 2, 0 }, { 0, 3 }, { 1, 2 },
      { 2, 1 }, { 3, 0 }, { 1, 3 }, { 2, 2 }, { 3, 1 }, { 2, 3 }, { 3, 2 }, { 3, 3 },
      { 0, 4 }, { 0, 5 }, { 1, 4 }, { 0, 6 }, { 1, 5 }, { 2, 4 }, { 0, 7 }, { 1, 6 },
      { 2, 5 }, { 3, 4 }, { 1, 7 }, { 2, 6 }, { 3, 5 }, { 2, 7 }, { 3, 6 }, { 3, 7 },
      { 4, 0 }, { 4, 1 }, { 5, 0 }, { 4, 2 }, { 5, 1 }, { 6, 0 }, { 4, 3 }, { 5, 2 },
      { 6, 1 }, { 7, 0 }, { 5, 3 }, { 6, 2 }, { 7, 1 }, { 6, 3 }, { 7, 2 }, { 7, 3 },
      { 4, 4 }, { 4, 5 }, { 5, 4 }, { 4, 6 }, { 5, 5 }, { 6, 4 }, { 4, 7 }, { 5, 6 },
      { 6, 5 }, { 7, 4 }, { 5, 7 }, { 6, 6 }, { 7, 5 }, { 6, 7 }, { 7, 6 }, { 7, 7 }
    };
    for( int i = 0; i < 64; i++ )
    {
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].idx = g_auiXYDiagScan8x8[ i ][ 0 ] + g_auiXYDiagScan8x8[ i ][ 1 ] * blockWidth;
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].x = g_auiXYDiagScan8x8[ i ][ 0 ];
      g_coefTopLeftDiagScan8x8[ blockWidthIdx ][ i ].y = g_auiXYDiagScan8x8[ i ][ 1 ];
    }
#if JVET_W0119_LFNST_EXTENSION
    for( int blky = 0; blky <= 1; blky++ )
    {
      for( int blkx = 0; blkx <= 1; blkx++ )
      {
        for( int i = 0; i < 64; i++ )
        {
          int i4 = blky * 128 + blkx * 64 + i;
          g_coefTopLeftDiagScan16x16[ blockWidthIdx ][ i4 ].idx = g_auiXYDiagScan8x8[ i ][ 0 ] + g_auiXYDiagScan8x8[ i ][ 1 ] * blockWidth + blkx * 8 + blky * 8 * blockWidth;
          g_coefTopLeftDiagScan16x16[ blockWidthIdx ][ i4 ].x = g_auiXYDiagScan8x8[ i ][ 0 ];
          g_coefTopLeftDiagScan16x16[ blockWidthIdx ][ i4 ].y = g_auiXYDiagScan8x8[ i ][ 1 ];
        }
      }
    }
#endif
  }

  initGeoTemplate();

  ::memset( g_isReusedUniMVsFilled, 0, sizeof( g_isReusedUniMVsFilled ) );
#if INTER_LIC
  ::memset( g_isReusedUniMVsFilledLIC, 0, sizeof( g_isReusedUniMVsFilledLIC ) );
#endif

  for( int qp = 0; qp < 57; qp++ )
  {
    int qpRem = ( qp + 12 ) % 6;
    int qpPer = ( qp + 12 ) / 6;
    int quantiserScale = g_quantScales[ 0 ][ qpRem ];
    int quantiserRightShift = QUANT_SHIFT + qpPer;
    double threshQP = ( ( double ) ( 1 << quantiserRightShift ) ) / quantiserScale;
    g_paletteQuant[ qp ] = ( int ) ( threshQP * 0.16 + 0.5 );
  }

#if SIGN_PREDICTION && !JVET_W0103_INTRA_MTS
  memset( &g_resiBorderTemplate[ 0 ][ 0 ][ 0 ], 0, sizeof( g_resiBorderTemplate ) );
  const int8_t* p_data = g_initRomSignPred;
  for( int log2Width = 0; log2Width < 6; ++log2Width )
  {
    int width = 4 << log2Width;
    for( int log2Height = 0; log2Height < 6; ++log2Height )
    {
      int height = 4 << log2Height;
      int length = width + height - 1;
      for( int idx = 0; idx < 9; ++idx )
      {
        if( *p_data != 0 && *p_data != 1 )
        {
          int j = 0;
          j += 1;
        }
        if( *p_data++ != 0 )
        {
          g_resiBorderTemplate[ log2Width ][ log2Height ][ idx ] = p_data;
          p_data += length * 16;
        }
      }
    }
  }
#endif
#if JVET_AG0067_DMVR_EXTENSIONS
  int bdofWidth[] = { 8,  8,  8,   12,  12,  12,  20,  20,   20 };
  int bdofHeight[] = { 8, 12, 20,    8,  12,  20,   8,  12,   20 };
  int weightOffset[] = { 0, 64, 160, 320, 416, 560, 800, 960, 1200 };
  for( int i = 0; i < 9; i++ )
  {
    int offset = weightOffset[ i ];
    int width = bdofWidth[ i ];
    int height = bdofHeight[ i ];
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        g_bdofWeight[ y * width + x + offset ] = ( x >= ( width / 2 ) ? width - x : x + 1 ) * ( y >= ( height / 2 ) ? height - y : y + 1 );
      }
    }
  }
#if JVET_AI0046_HIGH_PRECISION_BDOF_SAMPLE
  int weight8x8[] = {
    4,     5,     6,     7,     8,     8,     8,     8,     7,     6,     5,     4,
    5,     6,     8,     9,    10,    11,    11,    10,     9,     8,     6,     5,
    6,     8,    10,    11,    13,    13,    13,    13,    11,    10,     8,     6,
    7,     9,    11,    13,    15,    15,    15,    15,    13,    11,     9,     7,
    8,    10,    13,    15,    16,    17,    17,    16,    15,    13,    10,     8,
    8,    11,    13,    15,    17,    18,    18,    17,    15,    13,    11,     8,
    8,    11,    13,    15,    17,    18,    18,    17,    15,    13,    11,     8,
    8,    10,    13,    15,    16,    17,    17,    16,    15,    13,    10,     8,
    7,     9,    11,    13,    15,    15,    15,    15,    13,    11,     9,     7,
    6,     8,    10,    11,    13,    13,    13,    13,    11,    10,     8,     6,
    5,     6,     8,     9,    10,    11,    11,    10,     9,     8,     6,     5,
    4,     5,     6,     7,     8,     8,     8,     8,     7,     6,     5,     4 };
  for( int i = 0; i < 144; i++ )
  {
    g_bdofWeight[ 416 + i ] = weight8x8[ i ];
  }
#endif
#endif

#if JVET_Y0141_SIGN_PRED_IMPROVE
  memset( &g_resiBorderTemplate[ 0 ][ 0 ][ 0 ], 0, sizeof( g_resiBorderTemplate ) );
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
  memset( &g_resiBorderTemplateLFNST[ 0 ][ 0 ][ 0 ][ 0 ], 0, sizeof( g_resiBorderTemplateLFNST ) );
#else
  memset( &g_resiBorderTemplateLFNST[ 0 ][ 0 ][ 0 ], 0, sizeof( g_resiBorderTemplateLFNST ) );
#endif
#endif
#if TU_256
  c = 256;
  const double s = sqrt( ( double ) c ) * ( 64 << COM16_C806_TRANS_PREC );
  const double PI = 3.14159265358979323846;

  for( int k = 0; k < c; k++ )
  {
    for( int n = 0; n < c; n++ )
    {
      double w0, v;

      // DCT-II
      w0 = ( k == 0 ) ? sqrt( 0.5 ) : 1.0;
      v = cos( PI * ( n + 0.5 ) * k / c ) * w0 * sqrt( 2.0 / c );
      g_trCoreDCT2P256[ k ][ n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

      // DCT-VIII
      v = cos( PI * ( k + 0.5 ) * ( n + 0.5 ) / ( c + 0.5 ) ) * sqrt( 2.0 / ( c + 0.5 ) );
      g_trCoreDCT8P256[ k ][ n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

      // DST-VII
      v = sin( PI * ( k + 0.5 ) * ( n + 1 ) / ( c + 0.5 ) ) * sqrt( 2.0 / ( c + 0.5 ) );
      g_trCoreDST7P256[ k ][ n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );
    }
  }
#endif
#if JVET_W0103_INTRA_MTS
#if LMS_LINEAR_MODEL || TRANSFORM_SIMD_OPT || TU_256 
  c = 2;
#else
  int c = 2;
  const double PI = 3.14159265358979323846;
#endif

  for( int i = 0; i < 8; i++ )
  {
    const double s = sqrt( ( double ) c ) * ( 64 << COM16_C806_TRANS_PREC );
    TMatrixCoeff* iT = NULL;

    switch( i )
    {
    case 0: iT = g_aiTr2[ 0 ][ 0 ]; break;
    case 1: iT = g_aiTr4[ 0 ][ 0 ]; break;
    case 2: iT = g_aiTr8[ 0 ][ 0 ]; break;
    case 3: iT = g_aiTr16[ 0 ][ 0 ]; break;
    case 4: iT = g_aiTr32[ 0 ][ 0 ]; break;
    case 5: iT = g_aiTr64[ 0 ][ 0 ]; break;
    case 6: iT = g_aiTr128[ 0 ][ 0 ]; break;
    case 7: iT = g_aiTr256[ 0 ][ 0 ]; break;
    case 8: exit( 0 ); break;
    }

    for( int k = 0; k < c; k++ )
    {
      for( int n = 0; n < c; n++ )
      {
        double w0, w1, v;

        // DCT-II
        w0 = k == 0 ? sqrt( 0.5 ) : 1;
        v = cos( PI * ( n + 0.5 ) * k / c ) * w0 * sqrt( 2.0 / c );
        iT[ DCT2 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // DCT-V
        w0 = ( k == 0 ) ? sqrt( 0.5 ) : 1.0;
        w1 = ( n == 0 ) ? sqrt( 0.5 ) : 1.0;
        v = cos( PI * n * k / ( c - 0.5 ) ) * w0 * w1 * sqrt( 2.0 / ( c - 0.5 ) );
        iT[ DCT5 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // DCT-VIII
        v = cos( PI * ( k + 0.5 ) * ( n + 0.5 ) / ( c + 0.5 ) ) * sqrt( 2.0 / ( c + 0.5 ) );
        iT[ DCT8 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // DST-I
        v = sin( PI * ( n + 1 ) * ( k + 1 ) / ( c + 1 ) ) * sqrt( 2.0 / ( c + 1 ) );
        iT[ DST1 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // DST-VII
        v = sin( PI * ( k + 0.5 ) * ( n + 1 ) / ( c + 0.5 ) ) * sqrt( 2.0 / ( c + 0.5 ) );
        iT[ DST7 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // DST4
        v = sin( PI * ( k + 0.5 ) * ( n + 0.5 ) / c ) * sqrt( 2.0 / c );
        iT[ DST4 * c * c + k * c + n ] = ( short ) ( s * v + ( v > 0 ? 0.5 : -0.5 ) );

        // ID
        iT[ IDTR * c * c + k * c + n ] = ( k == n ) ? ( short ) ( s + ( s > 0 ? 0.5 : -0.5 ) ) : 0;
      }
    }
    c <<= 1;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
#if JVET_AA0133_INTER_MTS_OPT
  TMatrixCoeff KLT4[ 2 ][ 4 ][ 4 ] = {
  {
  { -79,  -88,  -46,  -17},
  {  74,  -10,  -86,  -58},
  { -59,   69,    4,  -91},
  { -35,   62,  -82,   67},
  },
  {
  {   3,   19,   71,  105},
  {  27,   90,   64,  -60},
  { -91,  -45,   69,  -36},
  {  86,  -77,   51,  -23},
  },
  };
  TMatrixCoeff KLT8[ 2 ][ 8 ][ 8 ] = {
  {
  {  55,   81,   92,   87,   68,   43,   25,   15},
  { -84,  -87,  -28,   46,   82,   72,   49,   31},
  {  75,   32,  -63,  -79,   -2,   71,   84,   63},
  {  80,  -15,  -88,   15,   88,   11,  -68,  -75},
  {  65,  -52,  -37,   83,  -13,  -92,   20,   92},
  {  63,  -83,   33,   39,  -82,   53,   53,  -83},
  { -45,   76,  -75,   53,  -12,  -46,   96,  -71},
  {  22,  -44,   61,  -75,   83,  -85,   75,  -38},
  },
  {
  {   8,   16,   31,   53,   74,   88,   93,   83},
  {  23,   43,   70,   90,   71,    7,  -64,  -91},
  { -53,  -83,  -83,  -19,   67,   77,   -6,  -72},
  { -73,  -79,   -4,   86,   31,  -82,  -36,   70},
  { -93,  -33,   89,   22,  -83,   27,   62,  -55},
  { -88,   45,   58,  -82,   35,   41,  -86,   52},
  { -77,   95,  -43,  -13,   54,  -77,   75,  -37},
  { -42,   75,  -83,   82,  -75,   62,  -43,   19},
  },
  };
  TMatrixCoeff KLT16[ 2 ][ 16 ][ 16 ] = {
  {
  {  42,   57,   71,   82,   90,   94,   93,   88,   79,   64,   50,   38,   28,   20,   15,   11},
  { -72,  -91,  -95,  -83,  -55,  -16,   26,   61,   80,   83,   75,   63,   50,   39,   31,   23},
  {  71,   78,   56,   12,  -39,  -77,  -87,  -65,  -20,   31,   68,   84,   84,   75,   63,   49},
  {  87,   77,   22,  -47,  -89,  -76,  -15,   57,   88,   67,   17,  -31,  -62,  -75,  -74,  -61},
  {  80,   51,  -24,  -78,  -62,   12,   77,   73,    3,  -73,  -90,  -49,   12,   62,   88,   84},
  {  89,   29,  -68,  -85,   -1,   84,   64,  -37,  -87,  -31,   53,   84,   48,  -16,  -68,  -82},
  {  80,   -1,  -84,  -36,   68,   62,  -44,  -82,   10,   90,   41,  -58,  -89,  -35,   51,   93},
  {  83,  -31,  -91,   24,   91,  -21,  -90,   20,   85,  -15,  -83,  -15,   73,   67,  -19,  -82},
  {  66,  -50,  -57,   69,   34,  -82,   -5,   88,  -25,  -87,   41,   88,  -29,  -96,  -10,   87},
  {  67,  -72,  -27,   91,  -35,  -67,   83,    7,  -85,   52,   56,  -78,  -37,   82,   38,  -74},
  {  57,  -80,   16,   62,  -77,   17,   59,  -83,   27,   63,  -89,    5,   93,  -54,  -71,   76},
  {  55,  -92,   57,   17,  -77,   82,  -32,  -38,   81,  -68,    4,   72,  -81,   -3,   91,  -66},
  {  43,  -82,   77,  -37,  -19,   68,  -87,   73,  -28,  -32,   79,  -81,   25,   58, -100,   58},
  { -28,   60,  -71,   64,  -37,    0,   43,  -79,   98,  -96,   71,  -21,  -40,   84,  -88,   43},
  {  30,  -67,   92, -106,  108, -101,   88,  -71,   55,  -40,   25,  -11,    0,    7,   -8,    4},
  {   6,  -13,   17,  -19,   17,  -10,    0,   16,  -36,   63,  -91,  115, -125,  115,  -84,   36},
  },
  {
  {   6,   10,   15,   22,   30,   39,   51,   63,   73,   81,   86,   90,   91,   89,   83,   74},
  { -19,  -28,  -39,  -50,  -63,  -75,  -84,  -87,  -77,  -53,  -20,   18,   53,   80,   95,   93},
  {  43,   60,   75,   85,   86,   73,   41,   -5,  -49,  -80,  -86,  -64,  -21,   28,   68,   81},
  {  58,   75,   78,   66,   36,  -12,  -61,  -88,  -69,   -9,   57,   92,   75,   15,  -54,  -89},
  {  83,   92,   66,   14,  -48,  -89,  -75,   -7,   64,   83,   34,  -42,  -81,  -50,   26,   78},
  {  83,   72,   17,  -49,  -84,  -52,   32,   88,   47,  -51,  -89,  -25,   70,   85,   -2,  -83},
  {  92,   54,  -35,  -90,  -54,   45,   90,   14,  -78,  -54,   49,   80,  -13,  -88,  -24,   73},
  {  83,   23,  -68,  -74,   22,   87,   12,  -87,  -25,   87,   36,  -83,  -44,   80,   50,  -71},
  {  87,  -10,  -95,  -22,   91,   34,  -89,  -23,   89,    1,  -85,   21,   78,  -43,  -62,   58},
  {  76,  -37,  -80,   42,   74,  -61,  -47,   86,   -5,  -86,   60,   48,  -89,   10,   82,  -58},
  {  79,  -71,  -52,   94,   -3,  -86,   68,   24,  -85,   57,   26,  -81,   53,   29,  -81,   48},
  {  68,  -90,    7,   78,  -78,    5,   66,  -84,   42,   34,  -84,   71,   -6,  -64,   89,  -45},
  {  59,  -98,   60,   19,  -79,   84,  -41,  -21,   70,  -88,   68,  -15,  -43,   80,  -78,   36},
  {  44,  -87,   86,  -47,   -9,   61,  -91,   97,  -80,   43,    3,  -44,   70,  -75,   59,  -25},
  {   5,   -8,    6,    2,  -13,   28,  -43,   59,  -76,   92, -105,  110, -103,   86,  -59,   23},
  {  36,  -81,  111, -122,  114,  -94,   67,  -41,   18,    0,  -14,   22,  -25,   22,  -16,    7},
  },
  };

  for( int i = 0; i < 4; i++ )
  {
    for( int j = 0; j < 4; j++ )
    {
      g_aiTr4[ KLT0 ][ i ][ j ] = KLT4[ 0 ][ i ][ j ];
      g_aiTr4[ KLT1 ][ i ][ j ] = KLT4[ 1 ][ i ][ j ];
    }
  }
  for( int i = 0; i < 8; i++ )
  {
    for( int j = 0; j < 8; j++ )
    {
      g_aiTr8[ KLT0 ][ i ][ j ] = KLT8[ 0 ][ i ][ j ];
      g_aiTr8[ KLT1 ][ i ][ j ] = KLT8[ 1 ][ i ][ j ];
    }
  }
  for( int i = 0; i < 16; i++ )
  {
    for( int j = 0; j < 16; j++ )
    {
      g_aiTr16[ KLT0 ][ i ][ j ] = KLT16[ 0 ][ i ][ j ];
      g_aiTr16[ KLT1 ][ i ][ j ] = KLT16[ 1 ][ i ][ j ];
    }
  }
#endif
  ////////////////////////////////////////////////////////////////////////////////////////////////
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  g_rmvfMultApproxTbl[ 0 ] = 0;
  for( int k = 1; k < ( 3 << sizeof( int64_t ) ); k++ )
  {
    g_rmvfMultApproxTbl[ k ] = ( ( 1 << 15 ) + ( k >> 1 ) ) / k;
  }
#endif
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  const int bvpStep = IBC_MBVD_AD_MAX_REFINE_NUM;
  const int curInterval = 1 << IBC_MBVD_LOG2_START_STEP;
  const int incInterval = curInterval * IBC_MBVD_OFFSET_DIR;
  const int startStepOffset = ( curInterval - 1 ) * IBC_MBVD_OFFSET_DIR;
  int searchCandIdx = 0;
  for( int bvpIdx = 0; bvpIdx < IBC_MBVD_BASE_NUM; bvpIdx++ )
  {
    int startBvdIdx = bvpIdx * bvpStep;
    int endBvdIdx = startBvdIdx + bvpStep;
    for( int bvdIdx = startBvdIdx + startStepOffset; bvdIdx < endBvdIdx; bvdIdx += incInterval )
    {
      for( int dir = 0; dir < IBC_MBVD_OFFSET_DIR; dir++ )
      {
        g_ibcMbvdStepCandIdxList[ searchCandIdx++ ] = bvdIdx + dir;
      }
    }
  }
  for( int bvdIdx = 0; bvdIdx < IBC_MBVD_AD_STEP_NUM; bvdIdx++ )
  {
    g_ibcMbvdCandOffsets[ bvdIdx ] = ( ( bvdIdx + 1 ) << MV_FRACTIONAL_BITS_INTERNAL );
  }
  int neiShift = 0;
  for( int neiIdx = 0; neiIdx < IBC_MBVD_NEI_NUM * 2; neiIdx += 2 )
  {
    neiShift += IBC_MBVD_OFFSET_DIR;
    g_ibcMbvdNeiOffsets[ neiIdx ] = -neiShift;
    g_ibcMbvdNeiOffsets[ neiIdx + 1 ] = +neiShift;
  }
#endif
}

void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

  for( uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++ )
  {
    for( uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++ )
    {
      for( uint32_t blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++ )
      {
        for( uint32_t blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++ )
        {
          delete[] g_scanOrder[ groupTypeIndex ][ scanOrderIndex ][ blockWidthIdx ][ blockHeightIdx ];
          g_scanOrder[ groupTypeIndex ][ scanOrderIndex ][ blockWidthIdx ][ blockHeightIdx ] = nullptr;
        }
      }
    }
  }

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;

#if JVET_AJ0107_GPM_SHAPE_ADAPT
  for( int modeIdx = 0; modeIdx < GEO_TOTAL_NUM_PARTITION_MODE; modeIdx++ )
#else
  for( int modeIdx = 0; modeIdx < GEO_NUM_PARTITION_MODE; modeIdx++ )
#endif
  {
    delete[] g_geoParams[ modeIdx ];
    g_geoParams[ modeIdx ] = nullptr;
  }
  delete[] g_geoParams;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AB0155_SGPM
  for( int bldIdx = 0; bldIdx < TOTAL_GEO_BLENDING_NUM; bldIdx++ )
#else
  for( int bldIdx = 0; bldIdx < GEO_BLENDING_NUM; bldIdx++ )
#endif
  {
    for( int i = 0; i < GEO_NUM_PRESTORED_MASK; i++ )
    {
      delete[] g_geoWeights[ bldIdx ][ i ];
      g_geoWeights[ bldIdx ][ i ] = nullptr;
    }
  }

  for( int i = 0; i < GEO_NUM_PRESTORED_MASK; i++ )
  {
    delete[] g_geoEncSadMask[ i ];
    g_geoEncSadMask[ i ] = nullptr;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    delete[] g_geoWeightsTpl[ i ];
    g_geoWeightsTpl[ i ] = nullptr;
#endif
  }
#else
  for( int i = 0; i < GEO_NUM_PRESTORED_MASK; i++ )
  {
    delete[] g_geoWeights[ i ];
    delete[] g_geoEncSadMask[ i ];
    g_geoWeights[ i ] = nullptr;
    g_geoEncSadMask[ i ] = nullptr;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    delete[] g_geoWeightsTpl[ i ];
    g_geoWeightsTpl[ i ] = nullptr;
#endif
  }
#endif
#if JVET_Y0141_SIGN_PRED_IMPROVE
  for( int log2Width = 0; log2Width < 6; log2Width++ )
  {
    for( int log2Height = 0; log2Height < 6; log2Height++ )
    {
      for( int idx = 0; idx < NUM_TRANS_TYPE * NUM_TRANS_TYPE; idx++ )
      {
        if( g_resiBorderTemplate[ log2Width ][ log2Height ][ idx ] )
        {
          xFree( g_resiBorderTemplate[ log2Width ][ log2Height ][ idx ] );
          g_resiBorderTemplate[ log2Width ][ log2Height ][ idx ] = nullptr;
        }
      }
#if JVET_W0119_LFNST_EXTENSION || EXTENDED_LFNST
      for( int idx = 0; idx < 210; idx++ )
#else
      for( int idx = 0; idx < 16; idx++ )
#endif
      {
#if JVET_AJ0175_NSPT_FOR_NONREG_MODES
        for( int t = 0; t < NUM_NSPT_BLOCK_TYPES; t++ )
        {
          if( g_resiBorderTemplateLFNST[ t ][ log2Width ][ log2Height ][ idx ] )
          {
            xFree( g_resiBorderTemplateLFNST[ t ][ log2Width ][ log2Height ][ idx ] );
            g_resiBorderTemplateLFNST[ t ][ log2Width ][ log2Height ][ idx ] = nullptr;
          }
        }
#else
        if( g_resiBorderTemplateLFNST[ log2Width ][ log2Height ][ idx ] )
        {
          xFree( g_resiBorderTemplateLFNST[ log2Width ][ log2Height ][ idx ] );
          g_resiBorderTemplateLFNST[ log2Width ][ log2Height ][ idx ] = nullptr;
        }
#endif
      }
    }
  }
#endif
}

#if MMLM
int g_aiLMDivTableLow[] = {
  0, 0, 21845, 0, 13107, 43690, 18724, 0, 50972, 39321, 53620, 21845, 15123, 9362, 4369, 0, 3855, 58254, 17246, 52428, 49932, 59578, 25644, 43690, 28835, 40329, 16990, 37449, 56496, 34952, 4228, 0, 61564, 34695, 29959, 29127, 15941, 41391, 26886, 26214, 28771, 24966, 6096, 29789, 23301, 45590, 25098, 21845, 30761, 47185, 1285, 20164, 34622, 41263, 36938, 18724, 49439, 61016, 51095, 17476, 23635, 2114, 16644, 0, 16131, 63550, 9781, 50115, 52238, 14979, 2769, 14563, 49376, 40738, 53302, 20695, 7660, 13443, 37330, 13107, 5663, 14385, 38689, 12483, 771, 3048, 18832, 47662, 23563, 11650, 11522, 22795, 45100, 12549, 55878, 43690, 41213, 48148, 64212, 23592, 57100, 33410, 17815, 10082, 9986, 17311, 31849, 53399, 16233, 51237, 27159, 9362, 63216, 57487, 57557, 63276, 8962, 25547, 47362, 8738, 40621, 11817, 53281, 33825, 18874, 8322, 2064, 0, 2032, 8065, 18009, 31775, 49275, 4890, 29612, 57825, 23918, 58887, 31589, 7489, 52056, 34152, 19248, 7281, 63728, 57456, 53944, 53137, 54979, 59419, 868, 10347, 22273, 36598, 53274, 6721, 27967, 51433, 11540, 39321, 3663, 35599, 4020, 39960, 12312, 52112, 28255, 6241, 51575, 33153, 16479, 1524, 53792, 42184, 32206, 23831, 17031, 11781, 8054, 5825, 5069, 5761, 7878, 11397, 16295, 22550, 30139, 39042, 49238, 60707, 7891, 21845, 37012, 53374, 5377, 24074, 43912, 64874, 21406, 44564, 3260, 28550, 54882, 16705, 45075, 8907, 39258, 5041, 37314, 4993, 39135, 8655, 44613, 15924, 53648, 26699, 604, 40884, 16458, 58386, 35585, 13579, 57895, 37449, 17767, 64376, 46192, 28743, 12019, 61546, 46244, 31638, 17720, 4481, 57448, 45541, 34288, 23681, 13710, 4369, 61185, 53078, 45578, 38676, 32366, 26640, 21491, 16912, 12896, 9437, 6527, 4161, 2331, 1032, 257, 0, 255, 1016, 2277, 4032, 6277, 9004, 12210, 15887, 20031, 24637, 29699, 35213, 41173, 47574, 54411, 61680, 3840, 11959, 20494, 29443, 38801, 48562, 58724, 3744, 14693, 26028, 37746, 49844, 62316, 9624, 22834, 36408, 50342, 64632, 13737, 28728, 44063, 59740, 10219, 26568, 43249, 60257, 12055, 29709, 47682, 434, 19033, 37941, 57155, 11136, 30953, 51067, 5938, 26637, 47624, 3360, 24916, 46751, 3328, 25716, 48376, 5770, 28967, 52428, 10616, 34599, 58840, 17799, 42547, 2010, 27256, 52748, 12947, 38924, 65140, 26056, 52743, 14127, 41277, 3120, 30726, 58555, 21072, 49344, 12300, 41007, 4394, 33530, 62876, 26896, 56659, 21092, 51264, 16103, 46678, 11915, 42886, 8515, 39875, 5890, 37632, 4027, 36145, 2912, 35400, 2534, 35385, 2880, 36089, 3939, 37500, 5698, 39605, 8147, 42395, 11275, 45857, 15069, 49982, 19521, 54758, 24619, 60175, 30353, 688, 36713, 7357, 43690, 14639, 51274, 22522, 59455, 30999, 2688, 40059, 12037, 49693, 21956, 59894, 32437, 5117, 43471, 16425, 55050, 28273, 1630, 40655, 14275, 53561, 27441, 1449, 41120, 15382, 55305, 29818, 4453, 44748, 19629, 60166, 35288, 10529, 51425, 26902, 2496, 43742, 19567, 61042, 37095, 13261, 55074, 31463, 7962, 50106, 26824, 3649, 46117, 23157, 302, 43088, 20442, 63436, 40997, 18660, 61961, 39826, 17792, 61393, 39557, 17819, 61715, 40171, 18724, 62908, 41651, 20489, 64956, 43980, 23096, 2304, 47139, 26529, 6009, 51115, 30773, 10519, 55890, 35811, 15819, 61448, 41628, 21892, 2240, 48208, 28724, 9322, 55538, 36301, 17144, 63604, 44608, 25692, 6855, 53632, 34952, 16349, 63360, 44911, 26539, 8242, 55557, 37410, 19338, 1340, 48951, 31099, 13320, 61149, 43513, 25949, 8456, 56569, 39216, 21932, 4718, 53109, 36031, 19022, 2080, 50741, 33933, 17191, 516, 49441, 32896, 16416, 0,
};
int g_aiLMDivTableHigh[] = {
  65536, 32768, 21845, 16384, 13107, 10922, 9362, 8192, 7281, 6553, 5957, 5461, 5041, 4681, 4369, 4096, 3855, 3640, 3449, 3276, 3120, 2978, 2849, 2730, 2621, 2520, 2427, 2340, 2259, 2184, 2114, 2048, 1985, 1927, 1872, 1820, 1771, 1724, 1680, 1638, 1598, 1560, 1524, 1489, 1456, 1424, 1394, 1365, 1337, 1310, 1285, 1260, 1236, 1213, 1191, 1170, 1149, 1129, 1110, 1092, 1074, 1057, 1040, 1024, 1008, 992, 978, 963, 949, 936, 923, 910, 897, 885, 873, 862, 851, 840, 829, 819, 809, 799, 789, 780, 771, 762, 753, 744, 736, 728, 720, 712, 704, 697, 689, 682, 675, 668, 661, 655, 648, 642, 636, 630, 624, 618, 612, 606, 601, 595, 590, 585, 579, 574, 569, 564, 560, 555, 550, 546, 541, 537, 532, 528, 524, 520, 516, 512, 508, 504, 500, 496, 492, 489, 485, 481, 478, 474, 471, 468, 464, 461, 458, 455, 451, 448, 445, 442, 439, 436, 434, 431, 428, 425, 422, 420, 417, 414, 412, 409, 407, 404, 402, 399, 397, 394, 392, 390, 387, 385, 383, 381, 378, 376, 374, 372, 370, 368, 366, 364, 362, 360, 358, 356, 354, 352, 350, 348, 346, 344, 343, 341, 339, 337, 336, 334, 332, 330, 329, 327, 326, 324, 322, 321, 319, 318, 316, 315, 313, 312, 310, 309, 307, 306, 304, 303, 302, 300, 299, 297, 296, 295, 293, 292, 291, 289, 288, 287, 286, 284, 283, 282, 281, 280, 278, 277, 276, 275, 274, 273, 271, 270, 269, 268, 267, 266, 265, 264, 263, 262, 261, 260, 259, 258, 257, 256, 255, 254, 253, 252, 251, 250, 249, 248, 247, 246, 245, 244, 243, 242, 241, 240, 240, 239, 238, 237, 236, 235, 234, 234, 233, 232, 231, 230, 229, 229, 228, 227, 226, 225, 225, 224, 223, 222, 222, 221, 220, 219, 219, 218, 217, 217, 216, 215, 214, 214, 213, 212, 212, 211, 210, 210, 209, 208, 208, 207, 206, 206, 205, 204, 204, 203, 202, 202, 201, 201, 200, 199, 199, 198, 197, 197, 196, 196, 195, 195, 194, 193, 193, 192, 192, 191, 191, 190, 189, 189, 188, 188, 187, 187, 186, 186, 185, 185, 184, 184, 183, 183, 182, 182, 181, 181, 180, 180, 179, 179, 178, 178, 177, 177, 176, 176, 175, 175, 174, 174, 173, 173, 172, 172, 172, 171, 171, 170, 170, 169, 169, 168, 168, 168, 167, 167, 166, 166, 165, 165, 165, 164, 164, 163, 163, 163, 162, 162, 161, 161, 161, 160, 160, 159, 159, 159, 158, 158, 157, 157, 157, 156, 156, 156, 155, 155, 154, 154, 154, 153, 153, 153, 152, 152, 152, 151, 151, 151, 150, 150, 149, 149, 149, 148, 148, 148, 147, 147, 147, 146, 146, 146, 145, 145, 145, 144, 144, 144, 144, 143, 143, 143, 142, 142, 142, 141, 141, 141, 140, 140, 140, 140, 139, 139, 139, 138, 138, 138, 137, 137, 137, 137, 136, 136, 136, 135, 135, 135, 135, 134, 134, 134, 134, 133, 133, 133, 132, 132, 132, 132, 131, 131, 131, 131, 130, 130, 130, 130, 129, 129, 129, 129, 128, 128, 128, 128,
};
#endif

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const int g_quantScales[ 2 ][ SCALING_LIST_REM_NUM ] = // can be represented as a 9 element table
{
    { 26214,23302,20560,18396,16384,14564 },
    { 18396,16384,14564,13107,11651,10280 } // Note: last 3 values of second row == half of the first 3 values of the first row
};

const int g_invQuantScales[ 2 ][ SCALING_LIST_REM_NUM ] = // can be represented as a 9 element table
{
  { 40,45,51,57,64,72 },
  { 57,64,72,80,90,102 } // Note: last 3 values of second row == double of the first 3 values of the first row
};

//--------------------------------------------------------------------------------------------------
//structures
//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------
// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const uint8_t g_aucIntraModeNumFast_UseMPM_2D[ 7 - MIN_CU_LOG2 + 1 ][ 7 - MIN_CU_LOG2 + 1 ] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const uint8_t g_aucIntraModeNumFast_UseMPM[ MAX_CU_DEPTH ] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const uint8_t g_aucIntraModeNumFast_NotUseMPM[ MAX_CU_DEPTH ] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const uint8_t g_chroma422IntraAngleMappingTable[ NUM_INTRA_MODE ] =
//                                    *                                H                              *                                D      *   *   *   *       *   *   *                   *        V       *                   *   *   *      *   *   *   *
//0, 1,  2,  3,  4,  5,  6,  7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 61, 62, 63, 64, 65, 66, 2, 3,  5,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 31, 33, 34, 35, 36, 37, 38, 39, 40, 41, 41, 42, 43, 43, 44, 44, 45, 45, 46, 47, 48, 48, 49, 49, 50, 51, 51, 52, 52, 53, 54, 55, 55, 56, 56, 57, 57, 58, 59, 59, 60, DM_CHROMA_IDX };


#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
int g_ibcMbvdCandOffsets[ IBC_MBVD_AD_STEP_NUM ];
int g_ibcMbvdStepCandIdxList[ IBC_MBVD_AD_NUM >> IBC_MBVD_LOG2_START_STEP ];
int g_ibcMbvdNeiOffsets[ IBC_MBVD_NEI_NUM * 2 ];
#endif

// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo* gp_sizeIdxInfo = NULL;

const int                 g_ictModes[ 2 ][ 4 ] = { { 0, 3, 1, 2 }, { 0, -3, -1, -2 } };

#if LMS_LINEAR_MODEL || TRANSFORM_SIMD_OPT
int8_t                    g_aucLog2[ MAX_CU_SIZE + 1 ];
#endif
#if LMS_LINEAR_MODEL
int8_t                    g_aucNextLog2[ MAX_CU_SIZE + 1 ];
int8_t                    g_aucPrevLog2[ MAX_CU_SIZE + 1 ];
#endif

#if MMLM && !LMS_LINEAR_MODEL
int8_t                    g_aucPrevLog2[ MAX_CU_SIZE + 1 ];
#endif

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
ScanElement* g_scanOrder[ SCAN_NUMBER_OF_GROUP_TYPES ][ SCAN_NUMBER_OF_TYPES ][ MAX_CU_SIZE / 2 + 1 ][ MAX_CU_SIZE / 2 + 1 ];
#if JVET_W0119_LFNST_EXTENSION
ScanElement  g_coefTopLeftDiagScan8x8[ MAX_CU_DEPTH + 1 ][ 64 ];
ScanElement  g_coefTopLeftDiagScan16x16[ MAX_CU_DEPTH + 1 ][ 256 ];
#else
ScanElement  g_coefTopLeftDiagScan8x8[ MAX_CU_SIZE / 2 + 1 ][ 64 ];
#endif

#if TU_256
// starting position of the coefficient in a group
// 2^suffix_length * ( 2 + (prefix & 1) )
const uint32_t g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96,128,192 };
#else
const uint32_t g_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
#endif

// suffix_length = prefix/2 - 1
// lastPos = 2^suffix_length * ( 2 + (prefix & 1) ) + suffix
// length of the prefix code
#if TU_256
const uint32_t g_uiGroupIdx[] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13
,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14
,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15
};
#else
const uint32_t g_uiGroupIdx[ MAX_TB_SIZEY ] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11 };
#endif
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const uint32_t g_auiGoRiceParsCoeffGTN[ GTN_MAXSUM ] =
{
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 3, 3, 3, 3, 3, 3, 3
};
#endif
const uint32_t g_auiGoRiceParsCoeff[ 32 ] =
{
  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3
};

#if TCQ_8STATES
const uint64_t g_stateTransTab[ 3 ] = { UINT64_C( 0 ), UINT64_C( 587280912 ), UINT64_C( 6274468666787591456 ) };
// the values represent the following state transition tables (curr : next[parity=0]  next[parity=1])
//
//    (1) no dep quant:  0 : 0 0
//
//    (2) 4 states:      0 : 0 1
//                       1 : 2 3
//                       2 : 1 0
//                       3 : 3 2
//
//    (3) 8 states:      0 : 0 2
//                       1 : 5 7
//                       2 : 1 3
//                       3 : 6 4
//                       4 : 2 0
//                       5 : 4 6
//                       6 : 3 1
//                       7 : 7 5
#endif

const char* MatrixType[ SCALING_LIST_SIZE_NUM ][ SCALING_LIST_NUM ] =
{
  {
    "INTRA1X1_LUMA",
    "INTRA1X1_CHROMAU",
    "INTRA1X1_CHROMAV",
    "INTER1X1_LUMA",
    "INTER1X1_CHROMAU",
    "INTER1X1_CHROMAV"
  },
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
    "INTRA32X32_LUMA",
    "INTRA32X32_CHROMAU",
    "INTRA32X32_CHROMAV",
    "INTER32X32_LUMA",
    "INTER32X32_CHROMAU",
    "INTER32X32_CHROMAV"
  },
  {
    "INTRA64X64_LUMA",
    "INTRA64X64_CHROMAU",
    "INTRA64X64_CHROMAV",
    "INTER64X64_LUMA",
    "INTER64X64_CHROMAU",
    "INTER64X64_CHROMAV"
  },
  {
  },
};

const char* MatrixType_DC[ SCALING_LIST_SIZE_NUM ][ SCALING_LIST_NUM ] =
{
  {  //1x1
  },
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC",
    "INTRA32X32_CHROMAV_DC",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC",
    "INTER32X32_CHROMAV_DC"
  },
  {
    "INTRA64X64_LUMA_DC",
    "INTRA64X64_CHROMAU_DC",
    "INTRA64X64_CHROMAV_DC",
    "INTER64X64_LUMA_DC",
    "INTER64X64_CHROMAU_DC",
    "INTER64X64_CHROMAV_DC"
  },
  {
  },
};

const int g_quantTSDefault4x4[ 4 * 4 ] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const int g_quantIntraDefault8x8[ 8 * 8 ] =
{
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16
};

const int g_quantInterDefault8x8[ 8 * 8 ] =
{
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16
};

const uint32_t g_scalingListSize[ SCALING_LIST_SIZE_NUM ] = { 1, 4, 16, 64, 256, 1024, 4096, 16384 };
const uint32_t g_scalingListSizeX[ SCALING_LIST_SIZE_NUM ] = { 1, 2,  4,  8,  16,   32,   64,   128 };

const uint32_t g_scalingListId[ SCALING_LIST_SIZE_NUM ][ SCALING_LIST_NUM ] =
{
  {  0,  0,  0,  0,  0,  0},  // SCALING_LIST_1x1
  {  0,  0,  0,  0,  0,  1},  // SCALING_LIST_2x2
  {  2,  3,  4,  5,  6,  7},  // SCALING_LIST_4x4
  {  8,  9, 10, 11, 12, 13},  // SCALING_LIST_8x8
  { 14, 15, 16, 17, 18, 19},  // SCALING_LIST_16x16
  { 20, 21, 22, 23, 24, 25},  // SCALING_LIST_32x32
  { 26, 21, 22, 27, 24, 25},  // SCALING_LIST_64x64
  {  0,  0,  0,  0,  0,  0},  // SCALING_LIST_128x128
};

#if CTU_256
Mv   g_reusedUniMVs[ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_DEPTH + 1 ][ MAX_CU_DEPTH + 1 ][ 2 ][ 33 ];
bool g_isReusedUniMVsFilled[ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_DEPTH + 1 ][ MAX_CU_DEPTH + 1 ];
#else
Mv   g_reusedUniMVs[ 32 ][ 32 ][ 8 ][ 8 ][ 2 ][ 33 ];
bool g_isReusedUniMVsFilled[ 32 ][ 32 ][ 8 ][ 8 ];
#endif
#if INTER_LIC
Mv   g_reusedUniMVsLIC[ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_DEPTH + 1 ][ MAX_CU_DEPTH + 1 ][ 2 ][ 33 ];
bool g_isReusedUniMVsFilledLIC[ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_SIZE >> MIN_CU_LOG2 ][ MAX_CU_DEPTH + 1 ][ MAX_CU_DEPTH + 1 ];
#endif

uint16_t g_paletteQuant[ 57 ];
uint8_t g_paletteRunTopLut[ 5 ] = { 0, 1, 1, 2, 2 };
uint8_t g_paletteRunLeftLut[ 5 ] = { 0, 1, 2, 3, 4 };

void initGeoTemplate()
{
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  g_geoParams = new int16_t * [ GEO_TOTAL_NUM_PARTITION_MODE ];
#else
  g_geoParams = new int16_t * [ GEO_NUM_PARTITION_MODE ];
#endif
  int modeIdx = 0;
  for( int angleIdx = 0; angleIdx < GEO_NUM_ANGLES; angleIdx++ )
  {
    for( int distanceIdx = 0; distanceIdx < GEO_NUM_DISTANCES; distanceIdx++ )
    {
      if( ( distanceIdx == 0 && angleIdx >= 16 )
#if !JVET_AJ0107_GPM_SHAPE_ADAPT
        || ( ( distanceIdx == 2 || distanceIdx == 0 ) && ( g_angle2mask[ angleIdx ] == 0 || g_angle2mask[ angleIdx ] == 5 ) )
#endif
        || g_angle2mask[ angleIdx ] == -1 )
      {
        continue;
      }
      g_geoParams[ modeIdx ] = new int16_t[ 2 ];
      g_geoParams[ modeIdx ][ 0 ] = ( int16_t ) angleIdx;
      g_geoParams[ modeIdx ][ 1 ] = ( int16_t ) distanceIdx;
      modeIdx++;
    }
  }
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING || JVET_AB0155_SGPM
  // initialization of blending weights
  for( int angleIdx = 0; angleIdx < ( GEO_NUM_ANGLES >> 2 ) + 1; angleIdx++ )
  {
    if( g_angle2mask[ angleIdx ] == -1 )
    {
      continue;
    }
#if JVET_AB0155_SGPM
    for( int bldIdx = 0; bldIdx < TOTAL_GEO_BLENDING_NUM; bldIdx++ )
#else
    for( int bldIdx = 0; bldIdx < GEO_BLENDING_NUM; bldIdx++ )
#endif
    {
      g_geoWeights[ bldIdx ][ g_angle2mask[ angleIdx ] ] = new int16_t[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];

      int distanceX = angleIdx;
      int distanceY = ( distanceX + ( GEO_NUM_ANGLES >> 2 ) ) % GEO_NUM_ANGLES;
      int16_t rho = ( g_dis[ distanceX ] << ( GEO_MAX_CU_LOG2 + 1 ) ) + ( g_dis[ distanceY ] << ( GEO_MAX_CU_LOG2 + 1 ) );
      static const int16_t maskOffset = ( 2 * GEO_MAX_CU_SIZE - GEO_WEIGHT_MASK_SIZE ) >> 1;
      int index = 0;

      for( int y = 0; y < GEO_WEIGHT_MASK_SIZE; y++ )
      {
        int16_t lookUpY = ( ( ( y + maskOffset ) << 1 ) + 1 ) * g_dis[ distanceY ];
        for( int x = 0; x < GEO_WEIGHT_MASK_SIZE; x++, index++ )
        {
          int16_t sxi = ( ( x + maskOffset ) << 1 ) + 1;
          int16_t weightIdx = sxi * g_dis[ distanceX ] + lookUpY - rho;
          if( g_bld2Width[ bldIdx ] > 1 )
          {
            int weightLinearIdx = 8 * g_bld2Width[ bldIdx ] + weightIdx;
            g_geoWeights[ bldIdx ][ g_angle2mask[ angleIdx ] ][ index ] = Clip3( 0, 32, ( weightLinearIdx + ( g_bld2Width[ bldIdx ] >> 2 ) ) >> floorLog2( g_bld2Width[ bldIdx ] >> 1 ) );
          }
          else
          {
            int weightLinearIdx = 8 + weightIdx;
            g_geoWeights[ bldIdx ][ g_angle2mask[ angleIdx ] ][ index ] = Clip3( 0, 32, weightLinearIdx << 1 );
          }
        }
      }
    }
  }

  // initialization of mask weights
  for( int angleIdx = 0; angleIdx < ( GEO_NUM_ANGLES >> 2 ) + 1; angleIdx++ )
  {
    if( g_angle2mask[ angleIdx ] == -1 )
    {
      continue;
    }
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
    g_geoWeightsTpl[ g_angle2mask[ angleIdx ] ] = new Pel[ GEO_WEIGHT_MASK_SIZE_EXT * GEO_WEIGHT_MASK_SIZE_EXT ];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    g_geoEncSadMask[ g_angle2mask[ angleIdx ] ] = new Pel[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];
#else
    g_geoEncSadMask[ g_angle2mask[ angleIdx ] ] = new int16_t[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];
#endif

    int distanceX = angleIdx;
    int distanceY = ( distanceX + ( GEO_NUM_ANGLES >> 2 ) ) % GEO_NUM_ANGLES;
    int16_t rho = ( g_dis[ distanceX ] << ( GEO_MAX_CU_LOG2 + 1 ) ) + ( g_dis[ distanceY ] << ( GEO_MAX_CU_LOG2 + 1 ) );
    static const int16_t maskOffset = ( 2 * GEO_MAX_CU_SIZE - GEO_WEIGHT_MASK_SIZE ) >> 1;
    int index = 0;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
    int indexGeoWeight = 0;
    for( int y = -GEO_TM_ADDED_WEIGHT_MASK_SIZE; y < GEO_WEIGHT_MASK_SIZE + GEO_TM_ADDED_WEIGHT_MASK_SIZE; y++ )
    {
      int16_t lookUpY = ( ( ( y + maskOffset ) << 1 ) + 1 ) * g_dis[ distanceY ];
      for( int x = -GEO_TM_ADDED_WEIGHT_MASK_SIZE; x < GEO_WEIGHT_MASK_SIZE + GEO_TM_ADDED_WEIGHT_MASK_SIZE; x++ )
      {
        int16_t sx_i = ( ( x + maskOffset ) << 1 ) + 1;
        int16_t weightIdx = sx_i * g_dis[ distanceX ] + lookUpY - rho;
        g_geoWeightsTpl[ g_angle2mask[ angleIdx ] ][ index++ ] = weightIdx > 0 ? 1 : 0;
        if( x >= 0 && x < GEO_WEIGHT_MASK_SIZE && y >= 0 && y < GEO_WEIGHT_MASK_SIZE )
        {
          g_geoEncSadMask[ g_angle2mask[ angleIdx ] ][ indexGeoWeight ] = weightIdx > 0 ? 1 : 0;
          ++indexGeoWeight;
        }
      }
    }
#else
    for( int y = 0; y < GEO_WEIGHT_MASK_SIZE; y++ )
    {
      int16_t lookUpY = ( ( ( y + maskOffset ) << 1 ) + 1 ) * g_dis[ distanceY ];
      for( int x = 0; x < GEO_WEIGHT_MASK_SIZE; x++, index++ )
      {
        int16_t sx_i = ( ( x + maskOffset ) << 1 ) + 1;
        int16_t weightIdx = sx_i * g_dis[ distanceX ] + lookUpY - rho;
        g_geoEncSadMask[ g_angle2mask[ angleIdx ] ][ index ] = weightIdx > 0 ? 1 : 0;
      }
    }
#endif
  }
#else
  for( int angleIdx = 0; angleIdx < ( GEO_NUM_ANGLES >> 2 ) + 1; angleIdx++ )
  {
    if( g_angle2mask[ angleIdx ] == -1 )
    {
      continue;
    }
    g_geoWeights[ g_angle2mask[ angleIdx ] ] = new int16_t[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
    g_geoWeightsTpl[ g_angle2mask[ angleIdx ] ] = new Pel[ GEO_WEIGHT_MASK_SIZE_EXT * GEO_WEIGHT_MASK_SIZE_EXT ];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    g_geoEncSadMask[ g_angle2mask[ angleIdx ] ] = new Pel[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];
#else
    g_geoEncSadMask[ g_angle2mask[ angleIdx ] ] = new int16_t[ GEO_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE ];
#endif

    int distanceX = angleIdx;
    int distanceY = ( distanceX + ( GEO_NUM_ANGLES >> 2 ) ) % GEO_NUM_ANGLES;
    int16_t rho = ( g_dis[ distanceX ] << ( GEO_MAX_CU_LOG2 + 1 ) ) + ( g_dis[ distanceY ] << ( GEO_MAX_CU_LOG2 + 1 ) );
    static const int16_t maskOffset = ( 2 * GEO_MAX_CU_SIZE - GEO_WEIGHT_MASK_SIZE ) >> 1;
    int index = 0;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
    int indexGeoWeight = 0;
    for( int y = -GEO_TM_ADDED_WEIGHT_MASK_SIZE; y < GEO_WEIGHT_MASK_SIZE + GEO_TM_ADDED_WEIGHT_MASK_SIZE; y++ )
    {
      int16_t lookUpY = ( ( ( y + maskOffset ) << 1 ) + 1 ) * g_dis[ distanceY ];
      for( int x = -GEO_TM_ADDED_WEIGHT_MASK_SIZE; x < GEO_WEIGHT_MASK_SIZE + GEO_TM_ADDED_WEIGHT_MASK_SIZE; x++ )
      {
        int16_t sx_i = ( ( x + maskOffset ) << 1 ) + 1;
        int16_t weightIdx = sx_i * g_dis[ distanceX ] + lookUpY - rho;
        int weightLinearIdx = 32 + weightIdx;
        g_geoWeightsTpl[ g_angle2mask[ angleIdx ] ][ index++ ] = weightIdx > 0 ? 1 : 0;
        if( x >= 0 && x < GEO_WEIGHT_MASK_SIZE && y >= 0 && y < GEO_WEIGHT_MASK_SIZE )
        {
          g_geoWeights[ g_angle2mask[ angleIdx ] ][ indexGeoWeight ] = Clip3( 0, 8, ( weightLinearIdx + 4 ) >> 3 );
          g_geoEncSadMask[ g_angle2mask[ angleIdx ] ][ indexGeoWeight ] = weightIdx > 0 ? 1 : 0;
          ++indexGeoWeight;
        }
      }
    }
#else
    for( int y = 0; y < GEO_WEIGHT_MASK_SIZE; y++ )
    {
      int16_t lookUpY = ( ( ( y + maskOffset ) << 1 ) + 1 ) * g_dis[ distanceY ];
      for( int x = 0; x < GEO_WEIGHT_MASK_SIZE; x++, index++ )
      {
        int16_t sx_i = ( ( x + maskOffset ) << 1 ) + 1;
        int16_t weightIdx = sx_i * g_dis[ distanceX ] + lookUpY - rho;
        int weightLinearIdx = 32 + weightIdx;
        g_geoWeights[ g_angle2mask[ angleIdx ] ][ index ] = Clip3( 0, 8, ( weightLinearIdx + 4 ) >> 3 );
        g_geoEncSadMask[ g_angle2mask[ angleIdx ] ][ index ] = weightIdx > 0 ? 1 : 0;
      }
    }
#endif
  }
#endif

  for( int hIdx = 0; hIdx < GEO_NUM_CU_SIZE; hIdx++ )
  {
    int16_t height = 1 << ( hIdx + GEO_MIN_CU_LOG2 );
    for( int wIdx = 0; wIdx < GEO_NUM_CU_SIZE; wIdx++ )
    {
      int16_t width = 1 << ( wIdx + GEO_MIN_CU_LOG2 );
#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for( int splitDir = 0; splitDir < GEO_TOTAL_NUM_PARTITION_MODE; splitDir++ )
#else
      for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++ )
#endif
      {
        int16_t angle = g_geoParams[ splitDir ][ 0 ];
        int16_t distance = g_geoParams[ splitDir ][ 1 ];
        int16_t offsetX = ( GEO_WEIGHT_MASK_SIZE - width ) >> 1;
        int16_t offsetY = ( GEO_WEIGHT_MASK_SIZE - height ) >> 1;
        if( distance > 0 )
        {
          if( angle % 16 == 8 || ( angle % 16 != 0 && height >= width ) )
          {
            offsetY += angle < 16 ? ( ( distance * ( int32_t ) height ) >> 3 ) : -( ( distance * ( int32_t ) height ) >> 3 );
          }
          else
          {
            offsetX += angle < 16 ? ( ( distance * ( int32_t ) width ) >> 3 ) : -( ( distance * ( int32_t ) width ) >> 3 );
          }
        }
        g_weightOffset[ splitDir ][ hIdx ][ wIdx ][ 0 ] = offsetX;
        g_weightOffset[ splitDir ][ hIdx ][ wIdx ][ 1 ] = offsetY;
      }
    }
  }
#if JVET_AB0155_SGPM
  for( int hIdx = 0; hIdx < GEO_NUM_CU_SIZE_EX; hIdx++ )
  {
    int16_t height = 1 << ( hIdx + GEO_MIN_CU_LOG2_EX );
    for( int wIdx = 0; wIdx < GEO_NUM_CU_SIZE_EX; wIdx++ )
    {
      int16_t width = 1 << ( wIdx + GEO_MIN_CU_LOG2_EX );
#if JVET_AJ0107_GPM_SHAPE_ADAPT
      for( int splitDir = 0; splitDir < GEO_TOTAL_NUM_PARTITION_MODE; splitDir++ )
#else
      for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++ )
#endif
      {
        int16_t angle = g_geoParams[ splitDir ][ 0 ];
        int16_t distance = g_geoParams[ splitDir ][ 1 ];
        int16_t offsetX = ( GEO_WEIGHT_MASK_SIZE - width ) >> 1;
        int16_t offsetY = ( GEO_WEIGHT_MASK_SIZE - height ) >> 1;
        if( distance > 0 )
        {
          if( angle % 16 == 8 || ( angle % 16 != 0 && height >= width ) )
          {
            offsetY += angle < 16 ? ( ( distance * ( int32_t ) height ) >> 3 ) : -( ( distance * ( int32_t ) height ) >> 3 );
          }
          else
          {
            offsetX += angle < 16 ? ( ( distance * ( int32_t ) width ) >> 3 ) : -( ( distance * ( int32_t ) width ) >> 3 );
          }
        }
        g_weightOffsetEx[ splitDir ][ hIdx ][ wIdx ][ 0 ] = offsetX;
        g_weightOffsetEx[ splitDir ][ hIdx ][ wIdx ][ 1 ] = offsetY;
      }
    }
  }
#endif

}

int16_t** g_geoParams;
#if JVET_AB0155_SGPM
int16_t* g_geoWeights[ TOTAL_GEO_BLENDING_NUM ][ GEO_NUM_PRESTORED_MASK ];
int      g_bld2Width[ TOTAL_GEO_BLENDING_NUM ] = { 1, 2, 4, 8, 16, 32 };
#elif JVET_AA0058_GPM_ADAPTIVE_BLENDING
int16_t* g_geoWeights[ GEO_BLENDING_NUM ][ GEO_NUM_PRESTORED_MASK ];
int       g_bld2Width[ GEO_BLENDING_NUM ] = { 1, 2, 4, 8, 16 };
#else
int16_t* g_geoWeights[ GEO_NUM_PRESTORED_MASK ];
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING || JVET_AB0155_SGPM
Pel* g_geoWeightsTpl[ GEO_NUM_PRESTORED_MASK ];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
Pel* g_geoEncSadMask[ GEO_NUM_PRESTORED_MASK ];
#else
int16_t* g_geoEncSadMask[ GEO_NUM_PRESTORED_MASK ];
#endif
#if JVET_AB0155_SGPM
#if JVET_AJ0107_GPM_SHAPE_ADAPT
int16_t g_weightOffsetEx[ GEO_TOTAL_NUM_PARTITION_MODE ][ GEO_NUM_CU_SIZE_EX ][ GEO_NUM_CU_SIZE_EX ][ 2 ];
#else
int16_t g_weightOffsetEx[ GEO_NUM_PARTITION_MODE ][ GEO_NUM_CU_SIZE_EX ][ GEO_NUM_CU_SIZE_EX ][ 2 ];
#endif
#if !JVET_AJ0107_GPM_SHAPE_ADAPT
int8_t g_sgpmSplitDir[ GEO_NUM_PARTITION_MODE ] = {
1,1,0,0,0,0,1,0,
1,0,1,0,1,0,1,0,
1,0,1,1,1,0,1,0,
1,0,1,0,1,0,1,0,
0,0,0,0,1,1,0,0,
0,0,1,0,0,1,0,0,
1,0,1,1,0,1,0,0,
1,0,0,1,0,0,0,0
};
#endif
#endif

#if JVET_AJ0107_GPM_SHAPE_ADAPT
// Shape-dependent indices for 'regular GPM'
int8_t g_gpmSplitDir[ GEO_NUM_CU_SHAPES ][ GEO_NUM_PARTITION_MODE ] = {
{1, 3, 64, 66, 33, 35, 88, 90, 4, 7, 69, 60, 63, 111, 8, 11, 72, 56, 59, 108, 12, 15, 75, 52, 55, 105, 5, 67, 61, 109, 9, 70, 57, 106, 13, 73, 53, 103, 16, 17, 19, 76, 78, 48, 49, 51, 100, 102, 6, 68, 62, 110, 10, 71, 58, 107, 14, 74, 54, 104, 18, 77, 50, 101},
{1, 3, 64, 66, 33, 35, 88, 90, 8, 11, 72, 56, 59, 108, 12, 15, 75, 52, 55, 105, 4, 7, 69, 60, 63, 111, 9, 70, 57, 106, 13, 73, 53, 103, 5, 67, 61, 109, 20, 21, 23, 79, 81, 44, 45, 47, 97, 99, 10, 71, 58, 107, 14, 74, 54, 104, 6, 68, 62, 110, 22, 80, 46, 98},
{1, 3, 64, 66, 33, 35, 88, 90, 12, 15, 75, 52, 55, 105, 16, 19, 78, 48, 51, 102, 8, 11, 72, 56, 59, 108, 13, 73, 53, 103, 17, 76, 49, 100, 9, 70, 57, 106, 4, 5, 7, 67, 69, 60, 61, 63, 109, 111, 14, 74, 54, 104, 18, 77, 50, 101, 10, 71, 58, 107, 6, 68, 62, 110},
{1, 3, 64, 66, 33, 35, 88, 90, 16, 19, 78, 48, 51, 102, 20, 23, 81, 44, 47, 99, 12, 15, 75, 52, 55, 105, 17, 76, 49, 100, 21, 79, 45, 97, 13, 73, 53, 103, 8, 9, 11, 70, 72, 56, 57, 59, 106, 108, 18, 77, 50, 101, 22, 80, 46, 98, 14, 74, 54, 104, 10, 71, 58, 107},
{1, 3, 64, 66, 33, 35, 88, 90, 20, 23, 81, 44, 47, 99, 24, 27, 84, 40, 43, 96, 16, 19, 78, 48, 51, 102, 21, 79, 45, 97, 25, 82, 41, 94, 17, 76, 49, 100, 12, 13, 15, 73, 75, 52, 53, 55, 103, 105, 22, 80, 46, 98, 26, 83, 42, 95, 18, 77, 50, 101, 14, 74, 54, 104},
{1, 3, 64, 66, 33, 35, 88, 90, 24, 27, 84, 40, 43, 96, 28, 31, 87, 36, 39, 93, 20, 23, 81, 44, 47, 99, 25, 82, 41, 94, 29, 85, 37, 91, 21, 79, 45, 97, 16, 17, 19, 76, 78, 48, 49, 51, 100, 102, 26, 83, 42, 95, 30, 86, 38, 92, 22, 80, 46, 98, 18, 77, 50, 101},
{1, 3, 64, 66, 33, 35, 88, 90, 28, 31, 87, 36, 39, 93, 20, 23, 81, 44, 47, 99, 24, 27, 84, 40, 43, 96, 29, 85, 37, 91, 21, 79, 45, 97, 25, 82, 41, 94, 16, 17, 19, 76, 78, 48, 49, 51, 100, 102, 30, 86, 38, 92, 14, 74, 54, 104, 26, 83, 42, 95, 18, 77, 50, 101} };
// Fixed-shape indices for IBC-GPM and SGPM
int8_t g_sgpmSplitDir[ SGPM_TOTAL_NUM_PARTITIONS ] = { 1, 3, 12, 14, 16, 18, 20, 22, 33, 35, 44, 46, 48, 50, 52, 54, 64, 66, 74, 77, 80, 88, 90, 98, 101, 104 };
int8_t g_ibcGpmSplitDir[ IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM + IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM ] = { 1, 3, 8, 9, 11, 12, 13, 15, 16, 17, 19, 20, 21, 23, 33, 35, 44, 45, 47, 48, 49, 51, 52, 53, 55, 56, 57, 59, 64, 66, 70, 72, 73, 75, 76, 78, 79, 81, 88, 90, 97, 99, 100, 102, 103, 105, 106, 108 };
int8_t g_ibcGpmSplitDirFirstSetRank[ IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM + IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM ] = { 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

#if JVET_AJ0107_GPM_SHAPE_ADAPT
int16_t   g_weightOffset[ GEO_TOTAL_NUM_PARTITION_MODE ][ GEO_NUM_CU_SIZE ][ GEO_NUM_CU_SIZE ][ 2 ];
int8_t    g_angle2mask[ GEO_NUM_ANGLES ] = { 0,  1, 2, 3, 4, 5,  6, 7, 8, 7,  6, 5, 4, 3, 2,  1, 0,  1, 2, 3, 4, 5,  6, 7, 8, 7,  6, 5, 4, 3, 2,  1 };
#else
int16_t   g_weightOffset[ GEO_NUM_PARTITION_MODE ][ GEO_NUM_CU_SIZE ][ GEO_NUM_CU_SIZE ][ 2 ];
int8_t    g_angle2mask[ GEO_NUM_ANGLES ] = { 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1, 0, -1, 1, 2, 3, 4, -1, -1, 5, -1, -1, 4, 3, 2, 1, -1 };
#endif
int8_t    g_dis[ GEO_NUM_ANGLES ] = { 8, 8, 8, 8, 4, 4, 2, 1, 0, -1, -2, -4, -4, -8, -8, -8, -8, -8, -8, -8, -4, -4, -2, -1, 0, 1, 2, 4, 4, 8, 8, 8 };
int8_t    g_angle2mirror[ GEO_NUM_ANGLES ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 2 };
#if JVET_Y0065_GPM_INTRA
#if JVET_AJ0107_GPM_SHAPE_ADAPT
int8_t    g_geoAngle2IntraAng[ GEO_NUM_ANGLES ] = { 50,46, 44, 41, 34, 27,23, 21, 18, 15,13, 9, 66, 59, 56,54, 50,46, 44, 41, 34, 27,23, 21, 18, 15,13, 9, 66, 59, 56,54 };
#else
int8_t    g_geoAngle2IntraAng[ GEO_NUM_ANGLES ] = { 50, 0, 44, 41, 34, 27, 0, 0, 18, 0, 0, 9, 66, 59, 56, 0, 50, 0, 44, 41, 34, 27, 0, 0, 18, 0, 0, 9, 66, 59, 56, 0 };
#endif
#endif


#if MULTI_HYP_PRED
const int g_addHypWeight[ MULTI_HYP_PRED_NUM_WEIGHTS ] = { 2, -1 };
static_assert( g_bcwLog2WeightBase == MULTI_HYP_PRED_WEIGHT_BITS, "number of bits for gbi and multi-hyp weights do not match" );
#endif
#if (JVET_W0097_GPM_MMVD_TM && TM_MRG) || JVET_Y0065_GPM_INTRA
#if JVET_AJ0107_GPM_SHAPE_ADAPT
uint8_t g_geoTmShape[ 2 ][ GEO_NUM_ANGLES ] = {
                                          { GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,
                                            GEO_TM_SHAPE_A,  GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,
                                            GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,
                                            GEO_TM_SHAPE_A,  GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A, },
                                          { GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, } };
#else
uint8_t g_geoTmShape[ 2 ][ GEO_NUM_ANGLES ] = {
                                          { GEO_TM_SHAPE_A,  0,               GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,
                                            GEO_TM_SHAPE_A,  GEO_TM_SHAPE_AL, 0,               0,
                                            GEO_TM_SHAPE_AL, 0,               0,               GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  0,
                                            GEO_TM_SHAPE_A,  0,               GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,
                                            GEO_TM_SHAPE_A,  GEO_TM_SHAPE_AL, 0,               0,
                                            GEO_TM_SHAPE_AL, 0,               0,               GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_A,  GEO_TM_SHAPE_A,  0, },
                                          { GEO_TM_SHAPE_AL, 0,               GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  0,               0,
                                            GEO_TM_SHAPE_L,  0,               0,               GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, 0,
                                            GEO_TM_SHAPE_AL, 0,               GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL,
                                            GEO_TM_SHAPE_L,  GEO_TM_SHAPE_L,  0,               0,
                                            GEO_TM_SHAPE_L,  0,               0,               GEO_TM_SHAPE_L,
                                            GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, GEO_TM_SHAPE_AL, 0, } };
#endif
#endif
#if JVET_W0066_CCSAO
const int8_t g_ccSaoCandPosX[ MAX_NUM_LUMA_COMP ][ MAX_CCSAO_CAND_POS_Y ] = { {-1,  0,  1, -1,  0,  1, -1,  0,  1} };
const int8_t g_ccSaoCandPosY[ MAX_NUM_LUMA_COMP ][ MAX_CCSAO_CAND_POS_Y ] = { {-1, -1, -1,  0,  0,  0,  1,  1,  1} };
#endif
#if JVET_Y0106_CCSAO_EDGE_CLASSIFIER
#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
const int8_t g_ccSaoEdgePosX[ MAX_CCSAO_EDGE_DIR ][ 2 ] = { { -1,  1 }, {  0,  0 }, { -1,  1 }, {  1, -1 } };
const int8_t g_ccSaoEdgePosY[ MAX_CCSAO_EDGE_DIR ][ 2 ] = { {  0,  0 }, { -1,  1 }, { -1,  1 }, { -1,  1 } };
const short  g_ccSaoEdgeThr[ MAX_CCSAO_EDGE_IDC ][ MAX_CCSAO_EDGE_THR ]
= { {   2,   4,   6,   8,  10,  14,  18,  22,  30,  38,  54,  70,  86, 118, 150, 182},
    {  -4,   4,  -8,   8, -14,  14, -22,  22, -38,  38, -70,  70,-118, 118,-182, 182} };
const int8_t g_ccSaoEdgeNum[ MAX_CCSAO_EDGE_IDC ][ 2 ] = { {16, 4}, { 4, 2} };
const int8_t g_ccSaoBandTab[ MAX_CCSAO_BAND_IDC ][ 2 ] = { {COMPONENT_Y, 1}, {COMPONENT_Y, 2}, {COMPONENT_Y, 3}, {COMPONENT_Y, 4}, {COMPONENT_Y, 5}, {COMPONENT_Y, 6}, {COMPONENT_Y,  7}, {COMPONENT_Y,  8},
                                                        {COMPONENT_Y, 9}, {COMPONENT_Y,10}, {COMPONENT_Y,11}, {COMPONENT_Y,12}, {COMPONENT_Y,13}, {COMPONENT_Y,14}, {COMPONENT_Cb, 2}, {COMPONENT_Cr, 2} };
#else
const int8_t g_ccSaoEdgeTypeX[ CCSAO_EDGE_TYPE ][ 2 ] = { { -1, 1 }, { 0, 0 }, { -1, 1 }, { 1, -1 } };
const int8_t g_ccSaoEdgeTypeY[ CCSAO_EDGE_TYPE ][ 2 ] = { { 0, 0 }, { -1, 1 }, { -1, 1 }, { -1, 1 } };
const short  g_ccSaoQuanValue[ CCSAO_QUAN_NUM ] = { 2, 4, 6, 8, 10, 14, 18, 22, 30, 38, 54, 70, 86, 118, 150, 182 };
#endif
#endif
#if JVET_AC0130_NSPT
const uint8_t g_nsptLut[ 97 ] =
{  // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  48  49  50  51  52  53  54  55  56  57  58  59  60  61  62  63  64  65  66  67  68  69  70  71  72  73  74  75  76  77  78  79  80  81  82  83  84  85  86  87  88  89  90  91  92  93  94  95  96
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2
};
#endif
#if JVET_V0130_INTRA_TMP
unsigned int g_uiDepth2Width[ 5 ] = { 4, 8, 16, 32, 64 };
#endif
#if JVET_X0149_TIMD_DIMD_LUT
int g_gradDivTable[ 16 ] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
int g_rmvfMultApproxTbl[ 3 << sizeof( int64_t ) ];
#endif
#if JVET_AA0126_GLM
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
const int8_t g_glmPattern[2][ NUM_GLM_PATTERN ][ 6 ] =
{
  {
#else
const int8_t g_glmPattern[ NUM_GLM_PATTERN ][ 6 ] =
{
#endif
  {  1,  0, -1,  1,  0, -1, }, {  1,  2,  1, -1, -2, -1, }, {  2,  1, -1,  1, -1, -2, }, { -1,  1,  2, -2, -1,  1, },
#if !JVET_AA0057_CCCM
  {  0,  2, -2,  0,  1, -1, }, {  1,  1,  1, -1, -1, -1, }, {  1,  1, -1,  1, -1, -1, }, { -1,  1,  1, -1, -1,  1, },
  {  0,  1, -1,  0,  1, -1, }, {  0,  1,  1,  0, -1, -1, }, {  1,  1,  0,  0, -1, -1, }, {  0,  1,  1, -1, -1,  0, },
  {  1, -1,  0,  1, -1,  0, }, {  1,  1,  0, -1, -1,  0, }, {  1,  2,  0,  0, -2, -1, }, {  0,  2,  1, -1, -2,  0, },
#endif
#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  },
  {
    {  0,  1, -1,  0,  1, -1, }, {  0,  1,  1,  0, -1, -1, }, {  0,  1, 0,   0,  0, -1, }, {  0,  0,  1,  0, -1,  0, },
  },
#endif
};
#endif
#if JVET_AC0112_IBC_GPM
#if !JVET_AJ0107_GPM_SHAPE_ADAPT
const int8_t g_ibcGpmFirstSetSplitDirToIdx[ GEO_NUM_PARTITION_MODE ] = {
0,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,2,3,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,4,5,0,0,
0,0,0,0,0,0,0,0,
0,0,6,7,0,0,0,0,
0,0,0,0,0,0,0,0
};
const int8_t g_ibcGpmFirstSetSplitDir[ IBC_GPM_MAX_SPLIT_DIR_FIRST_SET_NUM ] = { 0, 1, 18, 19, 36, 37, 50, 51 };
const int8_t g_ibcGpmSecondSetSplitDir[ GEO_NUM_PARTITION_MODE ] = {
0,0,1,1,0,1,1,1,
0,1,1,1,0,1,1,1,
0,1,0,0,1,1,0,1,
1,1,0,1,1,1,0,1,
1,1,0,1,0,0,1,0,
1,1,0,1,1,0,1,1,
0,1,0,0,1,0,1,1,
0,1,1,0,1,1,0,1
};
#endif
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
const int8_t g_amvpSbTmvp_mvd_dir[ 2 ][ 8 ] = { { 1, -1,  0,  0,  1, -1,  1, -1 },
                                            { 0,  0,  1, -1,  1, -1, -1,  1 } };
const int8_t g_amvpSbTmvp_mvd_offset[ 6 ] = { 4, 8, 12, 16, 24, 32 };
uint32_t g_picAmvpSbTmvpEnabledArea = 0;
#endif

#if JVET_AG0276_LIC_SLOPE_ADJUST
const int g_licSlopeDeltaSet[ LIC_SLOPE_MAX_NUM_DELTA + 1 ] = { 0, 1, -1 };
#endif
#if JVET_AK0059_MDIP
uint64_t g_intraModeCost[EXT_VDIA_IDX + 1];
#endif
#if JVET_AJ0061_TIMD_MERGE
uint64_t g_timdMrgCost[ EXT_VDIA_IDX + 1 ];
static constexpr std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> timdMergeOffsetXTable()
{
  std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> tab{ {{0}} };
  for( size_t widthLog2MinusMinCU = 0; widthLog2MinusMinCU < MAX_CU_DEPTH - MIN_CU_LOG2 + 1; widthLog2MinusMinCU++ )
  {
    size_t i = 0;
    const PosType width = 1 << ( widthLog2MinusMinCU + MIN_CU_LOG2 );
    int       offsetX = 0;
    int offsetX2 = width >> 1;
    const int numNACandidate[ 7 ] = { 11, 13, 10, 2, 2, 2, 2 };
    const int idxMap[ 7 ][ 15 ] = { {0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 12}, {0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14}, {0, 1, 3, 4, 5, 6, 7, 8, 11, 12}, {0, 1}, {0, 1}, {0, 1}, {0, 1} };
    for( int iDistanceIndex = 0; iDistanceIndex < 7; iDistanceIndex++ )
    {
      const int iNADistanceHor = width * ( iDistanceIndex + 1 );
      const int offsetX0 = -iNADistanceHor - 1;
      const int offsetX1 = width + iNADistanceHor - 1;
      for( int iNASPIdx = 0; iNASPIdx < numNACandidate[ iDistanceIndex ]; iNASPIdx++ )
      {
        switch( idxMap[ iDistanceIndex ][ iNASPIdx ] )
        {                                                                             // Angle CCW from (1,0), approx.
        case 0:  offsetX = offsetX2;                                        break;  //  90
        case 1:  offsetX = offsetX0;                                        break;  // 180
        case 2:  offsetX = offsetX0;                                        break;  // 135
        case 3:  offsetX = offsetX0;                                        break;  // 157.5
        case 4:  offsetX = ( offsetX0 + offsetX2 ) >> 1;                      break;  // 112.5
        case 5:  offsetX = offsetX0;                                        break;  // 146.25
        case 6:  offsetX = ( ( offsetX2 + ( offsetX0 + offsetX2 ) ) >> 1 ) >> 1;  break;  // 123.75
        case 7:  offsetX = offsetX1;                                        break;  //   0
        case 8:  offsetX = -1;                                              break;  // 270
        case 9:  offsetX = offsetX1;                                        break;  //  45
        case 10: offsetX = offsetX0;                                        break;  // 215
        case 11: offsetX = ( offsetX2 + offsetX1 ) >> 1;                      break;  //  67.5
        case 12: offsetX = offsetX0;                                        break;  // 197.5
        case 13: offsetX = offsetX1;                                        break;  //  22.5
        case 14: offsetX = ( offsetX0 + offsetX2 ) >> 1;                      break;  // 242.5
        default: printf( "error!" ); exit( 0 );                                 break;
        }
        tab[ widthLog2MinusMinCU ][ i++ ] = offsetX;
      }
    }
  }
  return tab;
}
const std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> g_timdMergeOffsetXTable = timdMergeOffsetXTable();

static constexpr std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> timdMergeOffsetYTable()
{
  std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> tab{ {{0}} };
  for( size_t heightLog2MinusMinCU = 0; heightLog2MinusMinCU < MAX_CU_DEPTH - MIN_CU_LOG2 + 1; heightLog2MinusMinCU++ )
  {
    size_t i = 0;
    const PosType height = 1 << ( heightLog2MinusMinCU + MIN_CU_LOG2 );
    int       offsetY = 0;
    int offsetY2 = height >> 1;
    const int numNACandidate[ 7 ] = { 11, 13, 10, 2, 2, 2, 2 };
    const int idxMap[ 7 ][ 15 ] = { {0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 12}, {0, 1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14}, {0, 1, 3, 4, 5, 6, 7, 8, 11, 12}, {0, 1}, {0, 1}, {0, 1}, {0, 1} };
    for( int iDistanceIndex = 0; iDistanceIndex < 7; iDistanceIndex++ )
    {
      const int iNADistanceVer = height * ( iDistanceIndex + 1 );
      const int offsetY0 = height + iNADistanceVer - 1;
      const int offsetY1 = -iNADistanceVer - 1;
      for( int iNASPIdx = 0; iNASPIdx < numNACandidate[ iDistanceIndex ]; iNASPIdx++ )
      {
        switch( idxMap[ iDistanceIndex ][ iNASPIdx ] )
        {                                                                             // Angle CCW from (1,0), approx.
        case 0:  offsetY = offsetY1;                                        break;  // 90
        case 1:  offsetY = offsetY2;                                        break;  // 180
        case 2:  offsetY = offsetY1;                                        break;  // 135
        case 3:  offsetY = ( offsetY1 + offsetY2 ) >> 1;                      break;  // 157.5
        case 4:  offsetY = offsetY1;                                        break;  // 112.5
        case 5:  offsetY = ( ( offsetY2 + ( offsetY1 + offsetY2 ) ) >> 1 ) >> 1;  break;  // 146.25
        case 6:  offsetY = offsetY1;                                        break;  // 123.75
        case 7:  offsetY = -1;                                              break;  //   0
        case 8:  offsetY = offsetY0;                                        break;  // 270
        case 9:  offsetY = offsetY1;                                        break;  //  45
        case 10: offsetY = offsetY0;                                        break;  // 215
        case 11: offsetY = offsetY1;                                        break;  //  67.5
        case 12: offsetY = ( offsetY2 + offsetY0 ) >> 1;                      break;  // 197.5
        case 13: offsetY = ( offsetY1 + offsetY2 ) >> 1;                      break;  //  22.5
        case 14: offsetY = offsetY0;                                        break;  // 242.5
        default: printf( "error!" ); exit( 0 );                                 break;
        }
        tab[ heightLog2MinusMinCU ][ i++ ] = offsetY;
      }
    }
  }
  return tab;
}
const std::array<std::array<PosType, TIMD_MERGE_MAX_NONADJACENT>, MAX_CU_DEPTH - MIN_CU_LOG2 + 1> g_timdMergeOffsetYTable = timdMergeOffsetYTable();
#endif

//! \}
