/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
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

/** \file     NNInference.h
    \brief    neural network-based inference class (header)
*/
#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Picture.h"
#include "Reshape.h"
#if NN_COMMON_API
#include <sadl/tensor.h>
//! \ingroup CommonLib
//! \{

struct InputData
{
  NNInputType nnInputType;
  int index;
  double scale;
  int shift;
  bool luma;
  bool chroma;
};

namespace sadl
{
  template<typename T> class Model;
}

class NNInference
{
public:
  template<typename T>
  static void fillInputFromBuf (const Picture* pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf, bool luma, bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;
    
    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr =  buf.get(COMPONENT_Cr);
    }
    
    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          if (luma && !chroma)
          {
            input(0, yy, xx, 0) = bufY.at(xx, yy) << shift;
          }
          else if (!luma && chroma)
          {
            input(0, yy, xx, 0) = bufCb.at(xx, yy) << shift;
            input(0, yy, xx, 1) = bufCr.at(xx, yy) << shift;
          }
          else if (luma && chroma)
          {
            input(0, yy, xx, 0) = bufY.at(xx, yy) << shift;
            input(0, yy, xx, 1) = bufCb.at(xx >> 1, yy >> 1) << shift;
            input(0, yy, xx, 2) = bufCr.at(xx >> 1, yy >> 1) << shift;
          }
        }
      }
    }
    else
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          if (luma && !chroma)
          {
            input(0, yy, xx, 0) = bufY.at(xx, yy) / scale;
          }
          else if (!luma && chroma)
          {
            input(0, yy, xx, 0) = bufCb.at(xx, yy) / scale;
            input(0, yy, xx, 1) = bufCr.at(xx, yy) / scale;
          }
          else if (luma && chroma)
          {
            input(0, yy, xx, 0) = bufY.at(xx, yy) / scale;
            input(0, yy, xx, 1) = bufCb.at(xx >> 1, yy >> 1) / scale;
            input(0, yy, xx, 2) = bufCr.at(xx >> 1, yy >> 1) / scale;
          }
        }
      }
    }
  }
#if JVET_AC0177_FLIP_INPUT
  static void geometricTransform (int ver, int hor, int& y, int& x, bool flip)
  {
    x = flip ? hor - 1 - x : x;
  }
  template<typename T>
  static void fillInputFromBuf (const Picture* pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf, bool luma, bool chroma, double scale, int shift, bool flip)
  {
    CPelBuf bufY, bufCb, bufCr;
    
    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr =  buf.get(COMPONENT_Cr);
    }
    
    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int y = 0; y < ver; y++)
      {
        for (int x = 0; x < hor; x++)
        {
          int yT = y;
          int xT = x;
          geometricTransform(ver, hor, yT, xT, flip);
          if (luma && !chroma)
          {
            input(0, yT, xT, 0) = bufY.at(x, y) << shift;
          }
          else if (!luma && chroma)
          {
            input(0, yT, xT, 0) = bufCb.at(x, y) << shift;
            input(0, yT, xT, 1) = bufCr.at(x, y) << shift;
          }
          else if (luma && chroma)
          {
            input(0, yT, xT, 0) = bufY.at(x, y) << shift;
            input(0, yT, xT, 1) = bufCb.at(x >> 1, y >> 1) << shift;
            input(0, yT, xT, 2) = bufCr.at(x >> 1, y >> 1) << shift;
          }
        }
      }
    }
    else
    {
      for (int y = 0; y < ver; y++)
      {
        for (int x = 0; x < hor; x++)
        {
          int yT = y;
          int xT = x;
          geometricTransform(ver, hor, yT, xT, flip);
          if (luma && !chroma)
          {
            input(0, yT, xT, 0) = bufY.at(x, y) / scale;
          }
          else if (!luma && chroma)
          {
            input(0, yT, xT, 0) = bufCb.at(x, y) / scale;
            input(0, yT, xT, 1) = bufCr.at(x, y) / scale;
          }
          else if (luma && chroma)
          {
            input(0, yT, xT, 0) = bufY.at(x, y) / scale;
            input(0, yT, xT, 1) = bufCb.at(x >> 1, y >> 1) / scale;
            input(0, yT, xT, 2) = bufCr.at(x >> 1, y >> 1) / scale;
          }
        }
      }
    }
  }
#endif
  template<typename T>
  static void fillInputFromConstant (const Picture* pic, UnitArea inferArea, sadl::Tensor<T> &input, int c, bool luma, double scale, int shift)
  {
    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          input(0, yy, xx, 0) = c << shift;
        }
      }
    }
    else
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          input(0, yy, xx, 0) = c / scale;
        }
      }
    }
  }

  template<typename T>
  static void fillInputFromConstantChroma( const Picture* pic, UnitArea inferArea, sadl::Tensor<T>& input, int c, bool luma, double scale, int shift )
  {
    int hor, ver;
    if( luma )
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr( !std::is_same_v<T, float> )
    {
      for( int yy = 0; yy < ver; yy++ )
      {
        for( int xx = 0; xx < hor; xx++ )
        {
          input( 0, yy, xx, 1 ) = c << shift;
          input( 0, yy, xx, 2 ) = c << shift;
        }
      }
    }
    else
    {
      for( int yy = 0; yy < ver; yy++ )
      {
        for( int xx = 0; xx < hor; xx++ )
        {
          input( 0, yy, xx, 1 ) = c / scale;
          input( 0, yy, xx, 2 ) = c / scale;
        }
      }
    }
  }
  template<typename T>
  static void fillInputFromConstantChromaTrans( const Picture* pic, UnitArea inferArea, sadl::Tensor<T>& input, int c, bool luma, double scale, int shift )
  {
    int hor, ver;
    if( luma )
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr( !std::is_same_v<T, float> )
    {
      for( int yy = 0; yy < ver; yy++ )
      {
        for( int xx = 0; xx < hor; xx++ )
        {
          input( 0, yy, xx, 4 ) = c << shift;
          input( 0, yy, xx, 5 ) = c << shift;
        }
      }
    }
    else
    {
      for( int yy = 0; yy < ver; yy++ )
      {
        for( int xx = 0; xx < hor; xx++ )
        {
          input( 0, yy, xx, 4 ) = c / scale;
          input( 0, yy, xx, 5 ) = c / scale;
        }
      }
    }
  }

#if JVET_AC0089_COMBINE_INTRA_INTER
  template<typename T>
  static void fillInputFromBufIpb(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf, bool luma,
                                  bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;

    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          if (luma && !chroma)
          {
            input(0, yy, xx, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (!luma && chroma)
          {
            input(0, yy, xx, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yy, xx, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (luma && chroma)
          {
            input(0, yy, xx, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yy, xx, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yy, xx, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
        }
      }
    }
    else
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          if (luma && !chroma)
          {
            input(0, yy, xx, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (!luma && chroma)
          {
            input(0, yy, xx, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yy, xx, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (luma && chroma)
          {
            input(0, yy, xx, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yy, xx, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yy, xx, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
        }
      }
    }
  }
#if JVET_AC0177_FLIP_INPUT
  template<typename T>
  static void fillInputFromBufIpb(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf, bool luma,
                                  bool chroma, double scale, int shift, bool flip)
  {
    CPelBuf bufY, bufCb, bufCr;

    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          int yT = yy;
          int xT = xx;
          geometricTransform(ver, hor, yT, xT, flip);
          if (luma && !chroma)
          {
            input(0, yT, xT, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (!luma && chroma)
          {
            input(0, yT, xT, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yT, xT, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (luma && chroma)
          {
            input(0, yT, xT, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yT, xT, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yT, xT, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
        }
      }
    }
    else
    {
      for (int yy = 0; yy < ver; yy++)
      {
        for (int xx = 0; xx < hor; xx++)
        {
          int yT = yy;
          int xT = xx;
          geometricTransform(ver, hor, yT, xT, flip);
          if (luma && !chroma)
          {
            input(0, yT, xT, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (!luma && chroma)
          {
            input(0, yT, xT, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yT, xT, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (luma && chroma)
          {
            input(0, yT, xT, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yT, xT, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yT, xT, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
        }
      }
    }
  }
#endif
#endif
#if JVET_AH0080_TRANS_INPUT
  template<typename T>
  static void fillInputFromConstantTrans(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, int c, bool luma,
                                       double scale, int shift)
  {
    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }

    constexpr int height = 2;   // dct size
    constexpr int width  = 2;

    // DCT not needed for constant
    if constexpr (std::is_same_v<T, float>)
    {
      for (int hh = 0; hh < ver; hh += height)
      {
        for (int ww = 0; ww < hor; ww += width)
        {
          input(0, hh / height, ww / width, 0) = c / scale;   // DC value = c
        }
      }
    }
    else
    {
      for (int hh = 0; hh < ver; hh += height)
      {
        for (int ww = 0; ww < hor; ww += width)
        {
          input(0, hh / height, ww / width, 0) = c << shift;
        }
      }
    }
  }

#if JVET_AJ0124_QP_BLOCK
  template<typename T>
  static void fillInputFromBufTransSkipDCT(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf,
                                  bool luma, bool chroma, double scale, int shift)
  {
    CHECK( chroma, "Not tested"); // not tested

    int hor, ver;
    if( luma )
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    CPelBuf bufY, bufCb, bufCr;
    if( luma )
    {
      bufY = buf.get( COMPONENT_Y );
    }
    if( chroma )
    {
      bufCb = buf.get( COMPONENT_Cb );
      bufCr = buf.get( COMPONENT_Cr );
    }

    constexpr int height = 2;   // dct size
    constexpr int width = 2;

    // DCT not needed for constant
    if constexpr( std::is_same_v<T, float> )
    {
      for( int y = 0; y < ver; y += height )
      {
        for( int x = 0; x < hor; x += width )
        {

          if( luma )
          {
            input( 0, y / height, x / width, 0 ) = bufY.at( x, y ) / scale;
          }
          if( chroma )
          {
            input( 0, y / height, x / width, 1 ) = bufCb.at( x / 2, y / 2 ) / scale;
            input( 0, y / height, x / width, 2 ) = bufCr.at( x / 2, y / 2 ) / scale;
          }
        }
      }
    }
    else
    {
      for( int y = 0; y < ver; y += height )
      {
        for( int x = 0; x < hor; x += width )
        {
          if( luma )
          {
            input( 0, y / height, x / width, 0 ) = bufY.at( x, y ) << shift;
          }
          if( chroma )
          {
            input( 0, y / height, x / width, 1 ) = bufCb.at( x / 2, y / 2 ) << shift;
            input( 0, y / height, x / width, 2 ) = bufCr.at( x / 2, y / 2 ) << shift;
          }
        }
      }
    }
  }
#endif

  template<typename T>
  static void fillInputFromBufTrans(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf,
                                  bool luma, bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;
    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }

    constexpr int height                    = 2;   // dct size
    constexpr int width                     = 2;
    TCoeff        block[width * height]     = {};
    TCoeff        blockCb[width * height]   = {};
    TCoeff        blockCr[width * height]   = {};
    TCoeff        tempCoeff[width * height] = {};
    TCoeff        tmp[width * height]       = {};

    const int    shiftTransToInput    = shift - 2;
    const int    shiftTransToInputUV = shift;
    const double scale1            = scale * 4;
    const double scale1_uv         = scale;
    int offset = 0;
    if( shiftTransToInput < 0 )
    {
      offset = 1 << ( -shiftTransToInput - 1 );
    }

    for (int hh = 0; hh < ver; hh += height)
    {
      for (int ww = 0; ww < hor; ww += width)
      {
        // each DCT
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            if (luma)
            {
              block[(y * width) + x] = bufY.at(ww + x, hh + y);   // 10-bit
            }
            if (chroma)
            {
              blockCb[(y * width) + x] = bufCb.at((ww + x) >> 1, (hh + y) >> 1);
              blockCr[(y * width) + x] = bufCr.at((ww + x) >> 1, (hh + y) >> 1);
            }
          }
        }

        // only needed for luma
        tmp[0]       = block[0] + block[1];
        tmp[1]       = block[2] + block[3];
        tmp[2]       = block[0] - block[1];
        tmp[3]       = block[2] - block[3];
        tempCoeff[0] = tmp[0] + tmp[1];
        tempCoeff[1] = tmp[2] + tmp[3];
        tempCoeff[2] = tmp[0] - tmp[1];
        tempCoeff[3] = tmp[2] - tmp[3];

        if constexpr (std::is_same_v<T, float>)
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              int yT = hh / height;
              int xT = ww / width;
              if (luma && !chroma)
              {
                input(0, yT, xT, (y * width) + x) = tempCoeff[(y * width) + x] / scale1;
              }
              else if (!luma && chroma)
              {
                input(0, yT, xT, (y / 2 * width) + x / 2) = blockCb[(y / 2 * width) + x / 2] / scale1_uv;
                input(0, yT, xT, width * height / 4 + (y / 2 * width) + x / 2) = blockCr[(y / 2 * width) + x / 2] / scale1_uv;
              }
              else if (luma && chroma)
              {
                input(0, yT, xT, (y * width) + x) = tempCoeff[(y * width) + x] / scale1;
                input(0, yT, xT, width * height + (y / 2 * width) + x / 2) = blockCb[(y / 2 * width) + x / 2] / scale1_uv;
                input(0, yT, xT, width * height * 5 / 4 + (y / 2 * width) + x / 2) = blockCr[(y / 2 * width) + x / 2] / scale1_uv;
              }
            }
          }
        }
        else
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              int yT = hh / height;
              int xT = ww / width;
              if (luma && !chroma)
              {
                if( shiftTransToInput >= 0 )
                {
                  input( 0, yT, xT, ( y * width ) + x ) = ( tempCoeff[( y * width ) + x] ) << shiftTransToInput;
                }
                else
                {
                  input( 0, yT, xT, ( y * width ) + x ) = ( tempCoeff[( y * width ) + x] + offset ) >> ( -shiftTransToInput );
                }
              }
              else if (!luma && chroma)
              {
                input(0, yT, xT, (y / 2 * width) + x / 2) = (blockCb[(y / 2 * width) + x / 2]) << shiftTransToInputUV;
                input(0, yT, xT, width * height / 4 + (y / 2 * width) + x / 2) = (blockCr[(y / 2 * width) + x / 2]) << shiftTransToInputUV;
              }
              else if (luma && chroma)
              {
                if (shiftTransToInput >= 0)
                {
                  input(0, yT, xT, (y * width) + x) = (tempCoeff[(y * width) + x]) << shiftTransToInput;
                }
                else
                {
                  input(0, yT, xT, (y * width) + x) = (tempCoeff[(y * width) + x] + offset) >> (-shiftTransToInput);
                }
                input(0, yT, xT, width * height + (y / 2 * width) + x / 2) = (blockCb[(y / 2 * width) + x / 2]) << shiftTransToInputUV;
                input(0, yT, xT, width * height * 5 / 4 + (y / 2 * width) + x / 2) = (blockCr[(y / 2 * width) + x / 2]) << shiftTransToInputUV;
              }
            }
          }
        }
      }
    }
  }

#if JVET_AJ0066
  template<typename T>
  static void fillInputFromBufIpb141(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf,
                                   bool luma, bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;

    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0, yOut = 0; yy < ver; yy += 2, yOut++)
      {
        for (int xx = 0, xOut = 0; xx < hor; xx += 2, xOut++)
        {
          if (luma && !chroma)
          {
            input(0, yOut, xOut, 0) = bufY.at(xx, yy) << shift;
          }
          else if (!luma && chroma)
          {
            input(0, yOut, xOut, 0) = bufCb.at(xOut, yOut) << shift;
            input(0, yOut, xOut, 1) = bufCr.at(xOut, yOut) << shift;
          }
          else if (luma && chroma)
          {
            input(0, yOut, xOut, 0) = bufY.at(xx, yy) << (shift);
            input(0, yOut, xOut, 1) = bufCb.at(xOut, yOut) << (shift );
            input(0, yOut, xOut, 2) = bufCr.at(xOut, yOut) << (shift);
          }
        }
      }
    }
    else
    {
      for (int yy = 0, yOut = 0; yy < ver; yy += 2, yOut++)
      {
        for (int xx = 0, xOut = 0; xx < hor; xx += 2, xOut++)
        {
          if (luma && !chroma)
          {
            input(0, yOut, xOut, 0) = bufY.at(xx, yy) / scale;
          }
          else if (!luma && chroma)
          {
            input(0, yOut, xOut, 0) = bufCb.at(xOut, yOut) / scale;
            input(0, yOut, xOut, 1) = bufCr.at(xOut, yOut) / scale;
          }
          else if (luma && chroma)
          {
            input(0, yOut, xOut, 0) = bufY.at(xx, yy) / scale;
            input(0, yOut, xOut, 1) = bufCb.at(xOut, yOut) / scale;
            input(0, yOut, xOut, 2) = bufCr.at(xOut, yOut) / scale;
          }
        }
      }
    }

  }

  template<typename T>
  static void fillInputFromBufIpb2(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf,
                                   bool luma, bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;
    // int     yOut;
    // int     xOut;
    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }
    if constexpr (!std::is_same_v<T, float>)
    {
      for (int yy = 0, yOut = 0; yy < ver; yy += 2, yOut++)
      {
        for (int xx = 0, xOut = 0; xx < hor; xx += 2, xOut++)
        {
          if (luma && !chroma)
          {
            input(0, yOut, xOut, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (!luma && chroma)
          {
            input(0, yOut, xOut, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yOut, xOut, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
          else if (luma && chroma)
          {
            input(0, yOut, xOut, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yOut, xOut, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
            input(0, yOut, xOut, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) << (shift - 1);
          }
        }
      }
    }
    else
    {
      for (int yy = 0, yOut = 0; yy < ver; yy += 2, yOut++)
      {
        for (int xx = 0, xOut = 0; xx < hor; xx += 2, xOut++)
        {
          if (luma && !chroma)
          {
            input(0, yOut, xOut, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (!luma && chroma)
          {
            input(0, yOut, xOut, 0) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yOut, xOut, 1) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
          else if (luma && chroma)
          {
            input(0, yOut, xOut, 0) = ((bufY.at(xx, yy) >> 1) + ((bufY.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yOut, xOut, 1) = ((bufCb.at(xx, yy) >> 1) + ((bufCb.at(xx, yy) >> 1) == 0) - 1) / 2.0;
            input(0, yOut, xOut, 2) = ((bufCr.at(xx, yy) >> 1) + ((bufCr.at(xx, yy) >> 1) == 0) - 1) / 2.0;
          }
        }
      }
    }
  }
#endif

  template<typename T>
  static void fillInputFromBufIpbTran(const Picture *pic, UnitArea inferArea, sadl::Tensor<T> &input, CPelUnitBuf buf,
                                     bool luma, bool chroma, double scale, int shift)
  {
    CPelBuf bufY, bufCb, bufCr;
    if (luma)
    {
      bufY = buf.get(COMPONENT_Y);
    }
    if (chroma)
    {
      bufCb = buf.get(COMPONENT_Cb);
      bufCr = buf.get(COMPONENT_Cr);
    }

    int hor, ver;
    if (luma)
    {
      hor = inferArea.lwidth();
      ver = inferArea.lheight();
    }
    else
    {
      hor = inferArea.lwidth() >> 1;
      ver = inferArea.lheight() >> 1;
    }

    constexpr int height                    = 2;   // dct size
    constexpr int width                     = 2;
    TCoeff        block[width * height]     = {};
    TCoeff        tempCoeff[width * height] = {};
    TCoeff        tmp[width * height]       = {};
    const int    shiftTransToInput = shift - 3; //-2 from transform, -1 from ipb;
    const double scale1         = scale * 8;
    int offset = 0;
    if( shiftTransToInput < 0 )
    {
      offset = 1 << ( -shiftTransToInput - 1 );
    }

    for (int hh = 0; hh < ver; hh += height)
    {
      for (int ww = 0; ww < hor; ww += width)
      {
        // each DCT block
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            block[(y * width) + x] = ((bufY.at(ww + x, hh + y) >> 1) + ((bufY.at(ww + x, hh + y) >> 1) == 0) - 1);   // ipb=[0,1,2]
          }
        }
        tmp[0]       = block[0] + block[1];
        tmp[1]       = block[2] + block[3];
        tmp[2]       = block[0] - block[1];
        tmp[3]       = block[2] - block[3];
        tempCoeff[0] = tmp[0] + tmp[1];
        tempCoeff[1] = tmp[2] + tmp[3];
        tempCoeff[2] = tmp[0] - tmp[1];
        tempCoeff[3] = tmp[2] - tmp[3];

        if constexpr (std::is_same_v<T, float>)
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              input(0, hh / height, ww / width, (y * width) + x) = tempCoeff[(y * width) + x] / scale1;
            }
          }
        }
        else
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              if( shiftTransToInput >= 0 )
              {
                input( 0, hh / height, ww / width, ( y * width ) + x ) = tempCoeff[( y * width ) + x] << shiftTransToInput;
              }
              else
              {
                input( 0, hh / height, ww / width, ( y * width ) + x ) = ( tempCoeff[( y * width ) + x] + offset ) >> ( -shiftTransToInput );
              }
            }
          }
        }
      }
    }
  }
#endif

  template<typename T>
  static void prepareInputs (const Picture* pic, UnitArea inferArea, std::vector<sadl::Tensor<T>> &inputs, int globalQp, int localQp, int sliceType, const std::vector<InputData> &listInputData)
  {
    for (auto inputData : listInputData)
    {
      switch (inputData.nnInputType)
      {
#if NNVC_USE_REC_BEFORE_DBF
      case NN_INPUT_REC:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->getRecBeforeDbfBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
#if NNVC_USE_PRED
      case NN_INPUT_PRED:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->getPredBufCustom(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
#if NNVC_USE_BS
      case NN_INPUT_BS:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->getBsMapBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
#if NNVC_USE_QP
      case NN_INPUT_GLOBAL_QP:
        fillInputFromConstant<T>(pic, inferArea, inputs[inputData.index], globalQp, inputData.luma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_LOCAL_QP:
          fillInputFromConstant<T>(pic, inferArea, inputs[inputData.index], localQp, inputData.luma, inputData.scale, inputData.shift);
        break;
#endif
#if NNVC_USE_PRED
      case NN_INPUT_REF_LIST_0:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_REF_LIST_1:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->slices[0]->getRefPic(REF_PIC_LIST_1, 0)->getRecoBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
#if JVET_AC0089_COMBINE_INTRA_INTER
#if JVET_AC0089_NNVC_USE_BPM_INFO
      case NN_INPUT_IPB:
        fillInputFromBufIpb<T>(pic, inferArea, inputs[inputData.index], pic->getBlockPredModeBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
#endif
#if JVET_AJ0124_QP_BLOCK
      case NN_INPUT_LOCAL_QP_BLOCK:
        fillInputFromBuf<T>(pic, inferArea, inputs[inputData.index], pic->getBlockQpBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
      default:
        THROW("invalid input data");
        break;
      }
    }
  }


#if JVET_AH0080_TRANS_INPUT
   template<typename T>
  static void prepareTransInputs (const Picture* pic, UnitArea inferArea, std::vector<sadl::Tensor<T>> &inputs, int globalQp, int localQp, int sliceType, const std::vector<InputData> &listInputData)
  {
    for (auto inputData : listInputData)
    {
      switch (inputData.nnInputType)
      {
      case NN_INPUT_REC:
        fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->getRecBeforeDbfBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_PRED:
           fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->getPredBufCustom(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;

#if NNVC_USE_PARTITION_AS_CU_AVERAGE
      case NN_INPUT_PARTITION:
        fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->getCuAverageBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
      case NN_INPUT_BS:
#if JVET_AJ0066
        fillInputFromBufIpb141<T>(pic, inferArea, inputs[inputData.index], pic->getBsMapBuf(inferArea),inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
#else
        fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->getBsMapBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
#endif
        break;
      case NN_INPUT_GLOBAL_QP:
        fillInputFromConstantTrans<T>(pic, inferArea, inputs[inputData.index], globalQp, inputData.luma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_LOCAL_QP:
        fillInputFromConstantTrans<T>(pic, inferArea, inputs[inputData.index], localQp, inputData.luma, inputData.scale, inputData.shift);
        break;
#if JVET_AJ0124_QP_BLOCK
      case NN_INPUT_LOCAL_QP_BLOCK:
         fillInputFromBufTransSkipDCT<T>(pic, inferArea, inputs[inputData.index], pic->getBlockQpBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#endif
      case NN_INPUT_SLICE_TYPE:
        fillInputFromConstantTrans<T>(pic, inferArea, inputs[inputData.index], sliceType, inputData.luma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_REF_LIST_0:
        fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->slices[0]->getRefPic(REF_PIC_LIST_0, 0)->getRecoBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
      case NN_INPUT_REF_LIST_1:
        fillInputFromBufTrans<T>(pic, inferArea, inputs[inputData.index], pic->slices[0]->getRefPic(REF_PIC_LIST_1, 0)->getRecoBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
        break;
#if JVET_AC0089_COMBINE_INTRA_INTER
      case NN_INPUT_IPB:
#if (JVET_AJ0066)
        fillInputFromBufIpb2<T>(pic, inferArea, inputs[inputData.index], pic->getBlockPredModeBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
#else
        fillInputFromBufIpbTran<T>(pic, inferArea, inputs[inputData.index], pic->getBlockPredModeBuf(inferArea), inputData.luma, inputData.chroma, inputData.scale, inputData.shift);
#endif
        break;
#endif
#if NN_HOP_UNIFIED_FORCE_USE
      case NN_INPUT_ZERO:
        fillInputFromConstantTrans<T>(pic, inferArea, inputs[inputData.index], 0, inputData.luma, inputData.scale, inputData.shift);
        break;
#endif
      default:
        THROW("invalid input data");
        break;
      }
    }
  }
#endif
  template<typename T>
  static void infer(sadl::Model<T> &model, std::vector<sadl::Tensor<T>> &inputs);
};

template<>
void NNInference::infer(sadl::Model<int16_t> &model, std::vector<sadl::Tensor<int16_t>> &inputs);
template<>
void NNInference::infer(sadl::Model<float> &model, std::vector<sadl::Tensor<float>> &inputs);
//! \}

#endif

