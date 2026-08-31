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
#if JVET_AL0228_NNLF_INTERFACES

#include "NNFilterUnified.h"
#include "NNInference.h"
#include <sadl/model.h>
#include <fstream>
using namespace std;

// some constants
static constexpr int log2InputQpScale  = 6;
static constexpr int log2InputIbpScale = 0;
static constexpr int defaultInputSize  = 128;   // block size
static constexpr int defaultBlockExt   = 8;

struct Input
{
  enum
  {
    Rec = 0,
    Pred,
    BS,
    QPbase,
#if JVET_AJ0124_QP_BLOCK
      QPBlock,
#else
    QPSlice,
#endif
    IPB,
    nbInputs
  };
};
void NNFilterUnified::init(const std::string &filename, int picWidth, int picHeight, ChromaFormat format, int prmNum)
{

 if (!m_model)
  {
    ifstream file(filename, ios::binary);
    if (!file)
    {
      THROW( "Unable to open NNFilter model" );
      exit(-1);
    }

    m_model.reset(new sadl::Model<TypeSadlLFUnified>());
    if (!m_model->load(file))
    {
      THROW( "Issue in loading model NNFilter" " << filename << endl" );
      exit(-1);
    }
  }

  // prepare inputs
  m_inputs.resize(Input::nbInputs);
  resizeInputs(defaultInputSize + defaultBlockExt * 2, defaultInputSize + defaultBlockExt * 2);

  if (m_filtered.size() > 0 || m_scaled[0].size() > 0)
  {
    return;
  }

  m_filtered.resize(prmNum);
  for (int i = 0; i < prmNum; i++)
  {
    m_filtered[i].create(format, Area(0, 0, picWidth, picHeight));
  }
  for (int j = 0; j < 3; j++)
  {
    m_scaled[j].resize(prmNum);
    for (int i = 0; i < prmNum; i++)
    {
      m_scaled[j][i].create(format, Area(0, 0, picWidth, picHeight));
    }
  }
}
void NNFilterUnified::destroy()
{
  for (int i = 0; i < (int)m_filtered.size(); i++)
  {
    m_filtered[i].destroy();
  }
  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < (int)m_scaled[j].size(); i++)
    {
      m_scaled[j][i].destroy();
    }
  }

}

// default is square block + extension
void NNFilterUnified::resizeInputs(int width, int height)
{
  const int sizeW = width;
  const int sizeH = height;

  if (sizeH == m_blocksize[0] && sizeW == m_blocksize[1])
  {
    return;
  }
  m_blocksize[0] = sizeH;
  m_blocksize[1] = sizeW;

  // note: later QP inputs can be optimized to avoid some duplicate computation with a 1x1 input

#if JVET_AH0080_TRANS_INPUT 
  if (m_nnlfTransInput)
  {
    constexpr int dctSizeW = 2;
    constexpr int dctSizeH = 2;
    constexpr int dctArea   = dctSizeW * dctSizeH;

    CHECK( sizeH % dctSizeH, "Wrong horizontal size");
    CHECK( sizeW % dctSizeW, "Wrong vertical size" );

#if NNLF_INPUT_Y_ONLY
    m_inputs[Input::Rec].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea }));
    m_inputs[Input::Pred].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea }));
#if JVET_AJ0066
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#else
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea }));
#endif
    m_inputs[Input::QPbase].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#if JVET_AJ0124_QP_BLOCK
    m_inputs[Input::QPBlock].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#else
    m_inputs[Input::QPSlice].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#endif
#if (JVET_AJ0066) 
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#else
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 * dctArea }));
#endif
#else
    m_inputs[Input::Rec].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea + dctArea / 2 }));
    m_inputs[Input::Pred].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea + dctArea / 2 }));
#if JVET_AJ0066
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 3 }));
#else
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctArea + dctArea / 2 }));
#endif
    m_inputs[Input::QPbase].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#if JVET_AJ0124_QP_BLOCK
    m_inputs[Input::QPBlock].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#else
    m_inputs[Input::QPSlice].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#endif
#if (JVET_AJ0066) 
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
#else
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 * dctArea }));
#endif
#endif
  }
  else
  {
#endif
    // note: later QP inputs can be optimized to avoid some duplicate computation with a 1x1 input
    m_inputs[Input::Rec].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::Pred].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::QPbase].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
#if JVET_AJ0124_QP_BLOCK
    m_inputs[Input::QPBlock].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
#else
    m_inputs[Input::QPSlice].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
#endif
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
#if JVET_AH0080_TRANS_INPUT
  }
#endif

  if (!m_model->init(m_inputs))
  {
    THROW( "Issue init model NNFilterUnified" );
    exit(-1);
  }

  CHECK( numInputs != m_inputs.size(), "Wrong number of inputs");

  for( int i = 0; i < numInputs; ++i )
  {
    m_inputQuantizer[i] = m_model->getInputsTemplate()[i].quantizer;
  }
}

void roundToOutputBitdepth(const PelUnitBuf &src, PelUnitBuf &dst, const ClpRngs &clpRngs)
{
  for (int c = 0; c < MAX_NUM_COMPONENT; c++)
  {
    const int shift  = NNFilterUnified::log2OutputScale - clpRngs.comp[ComponentID(c)].bd;
    const int offset = 1 << (shift - 1);

    CHECK( !shift, "Wrong shift" );

    const PelBuf &bufSrc = src.get(ComponentID(c));
    PelBuf &      bufDst = dst.get(ComponentID(c));
    const int     width  = bufSrc.width;
    const int     height = bufSrc.height;
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        bufDst.at(x, y) = Pel(Clip3<int>(0, 1023, (bufSrc.at(x, y) + offset) >> shift));
      }
    }
  }
}

template<typename T>
static void extractOutputs(const Picture &pic, sadl::Model<T> &m, PelUnitBuf &bufDst, UnitArea inferArea, int extLeft,
                           int extRight, int extTop, int extBottom)
{
  const int log2InputBitdepth = pic.cs->slice->clpRng(COMPONENT_Y).bd;   // internal bitdepth
  auto      output            = m.result(0);
  const int qQutputSadl     = output.quantizer;
  const int shiftInput        = NNFilterUnified::log2OutputScale - log2InputBitdepth;
  const int shiftOutput       = NNFilterUnified::log2OutputScale - qQutputSadl;
  assert(shiftInput >= 0);
  assert(shiftOutput >= 0);

  const int width   = bufDst.Y().width;
  const int height  = bufDst.Y().height;
  PelBuf &  bufDstY = bufDst.get(COMPONENT_Y);
  CPelBuf   bufRecY = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Y);

  for (int c = 0; c < 4; ++c)   // unshuffle on C++ side
  {
    for (int y = 0; y < height / 2; ++y)
    {
      for (int x = 0; x < width / 2; ++x)
      {
        int yy = (y * 2) + c / 2;
        int xx = (x * 2) + c % 2;
        if (xx < extLeft || yy < extTop || xx >= width - extRight || yy >= height - extBottom)
        {
          continue;
        }
        int out;
        if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
        {
          out = round((output(0, y, x, c) * (1 << shiftOutput) + (float) bufRecY.at(xx, yy) * (1 << shiftInput)));
        }
        else
        {
          out = ((output(0, y, x, c) << shiftOutput) + (bufRecY.at(xx, yy) << shiftInput));
        }

        bufDstY.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, out));
      }
    }
  }

  PelBuf &bufDstCb = bufDst.get(COMPONENT_Cb);
  PelBuf &bufDstCr = bufDst.get(COMPONENT_Cr);
  CPelBuf bufRecCb = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cb);
  CPelBuf bufRecCr = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cr);

  for (int y = 0; y < height / 2; ++y)
  {
    for (int x = 0; x < width / 2; ++x)
    {
      if (x < extLeft / 2 || y < extTop / 2 || x >= width / 2 - extRight / 2 || y >= height / 2 - extBottom / 2)
      {
        continue;
      }

      int outCb;
      int outCr;
      if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
      {
        outCb = round(output(0, y, x, 4) * (1 << shiftOutput) + (float) bufRecCb.at(x, y) * (1 << shiftInput));
        outCr = round(output(0, y, x, 5) * (1 << shiftOutput) + (float) bufRecCr.at(x, y) * (1 << shiftInput));
      }
      else
      {
        outCb = ((output(0, y, x, 4) << shiftOutput) + (bufRecCb.at(x, y) << shiftInput));
        outCr = ((output(0, y, x, 5) << shiftOutput) + (bufRecCr.at(x, y) << shiftInput));
      }

      bufDstCb.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCb));
      bufDstCr.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCr));
    }
  }
}
#if JVET_AH0080_TRANS_INPUT
template<typename T>
static void extractOutputsDCT(const Picture &pic, sadl::Model<T> &m, PelUnitBuf &bufDst, UnitArea inferArea,
                              int extLeft, int extRight, int extTop, int extBottom)
{
  const int log2InputBitdepth = pic.cs->slice->clpRng(COMPONENT_Y).bd;   // internal bitdepth
  auto      output            = m.result(0);
  const int qQutputSadl     = output.quantizer;
  const int shiftInput        = NNFilterUnified::log2OutputScale - log2InputBitdepth;
  const int shiftOutput       = NNFilterUnified::log2OutputScale - qQutputSadl;

  CHECK( !shiftInput, "Wrong shift input");
  CHECK( !shiftOutput, "Wrong shift output" );

  const int width   = bufDst.Y().width;
  const int height  = bufDst.Y().height;
  PelBuf   &bufDstY = bufDst.get(COMPONENT_Y);
  CPelBuf   bufRecY = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Y);

  constexpr int dctSizeW                        = 2;
  constexpr int dctSizeH                        = 2;
  TCoeff        block[dctSizeW * dctSizeH]     = {};
  TCoeff        tempCoeff[dctSizeW * dctSizeH] = {};
  TCoeff        tmp[dctSizeW * dctSizeH]       = {};
#if JVET_AJ0054_EARLY_CROPPING
  block[0] = 0;
  block[1] = 0;
  block[2] = 0;
  block[3] = 0;
  const bool earlyCropping=(inferArea.Y().width/2>output.dims()[2]);
  const int border=extLeft/dctSizeH;
#endif
  for (int y = 0; y < height / dctSizeH; y++)
  {
    for( int x = 0; x < width / dctSizeW; x++ )
    {
      int c1 = x % 2 + ( y % 2 ) * 2;   // take every 4th channel before pixel shuffle
#if JVET_AJ0054_EARLY_CROPPING
      if( earlyCropping )
      {
        if( y < border || y >= ( ( height / dctSizeH ) - border ) || x < border || x >= ( ( width / dctSizeW ) - border ) )
        {
          continue;
        }
      }
#endif      
      if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
      {
        for( int cc = c1; cc < dctSizeW * dctSizeH * 4; cc += 4 )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeff[cc / 4] = output( 0, ( y - border ) / 2, ( x - border ) / 2, cc ) * ( 1 << shiftOutput );
          }
          else
#endif
          {
            tempCoeff[cc / 4] = output( 0, y / 2, x / 2, cc ) * ( 1 << shiftOutput );
          }
        }
      }
      else
      {
        for( int cc = c1; cc < dctSizeW * dctSizeH * 4; cc += 4 )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeff[cc / 4] = output( 0, ( y - border ) / 2, ( x - border ) / 2, cc ) << shiftOutput;
          }
          else
#endif
          {
            tempCoeff[cc / 4] = output( 0, y / 2, x / 2, cc ) << shiftOutput;
          }
        }
      }
      tmp[0]   = tempCoeff[0] + tempCoeff[1];
      tmp[1]   = tempCoeff[2] + tempCoeff[3];
      tmp[2]   = tempCoeff[0] - tempCoeff[1];
      tmp[3]   = tempCoeff[2] - tempCoeff[3];
      block[0] = tmp[0] + tmp[1];
      block[1] = tmp[2] + tmp[3];
      block[2] = tmp[0] - tmp[1];
      block[3] = tmp[2] - tmp[3];

      for( int y1 = 0; y1 < dctSizeH; y1++ )
      {
        for( int x1 = 0; x1 < dctSizeW; x1++ )
        {
          int xx = x * dctSizeW + x1;
          int yy = y * dctSizeH + y1;

          if( xx < extLeft || yy < extTop || xx >= width - extRight || yy >= height - extBottom )
          {
            continue;
          }
          int out;
          if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
          {
            out = round( ( block[( y1 * dctSizeW ) + x1] + ( float ) bufRecY.at( xx, yy ) * ( 1 << shiftInput ) ) );
          }
          else
          {
            out = ( ( block[( y1 * dctSizeW ) + x1] ) + ( bufRecY.at( xx, yy ) << shiftInput ) );
          }

          bufDstY.at( xx, yy ) = Pel( Clip3<int>( 0, ( 1 << NNFilterUnified::log2OutputScale ) - 1, out ) );
        }
      }
    }
  }
#if NNLF_ALF_POS_INTERFACE
  PelBuf &bufDstCb    = bufDst.get(COMPONENT_Cb);
  PelBuf &bufDstCr    = bufDst.get(COMPONENT_Cr);
  CPelBuf bufRecCb    = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cb);
  CPelBuf bufRecCr    = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cr);

  TCoeff  tempCoeffCb[dctSizeW * dctSizeH] = {};
  TCoeff  tempCoeffCr[dctSizeW * dctSizeH] = {};

  for( int y = 0; y < height / 2 / dctSizeH; y++ )
  {
    for( int x = 0; x < width / 2 / dctSizeW; x++ )
    {
#if JVET_AJ0054_EARLY_CROPPING
      if( earlyCropping )
      {
        if( y < 2 || y >= ( ( height / 2 / dctSizeH ) - 2 ) || x < 2 || x >= ( ( width / 2 / dctSizeW ) - 2 ) )
        {
          continue;
        }
      }
#endif      
      if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
      {
        for( int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++ )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc ) * ( 1 << shiftOutput );
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc + dctSizeW * dctSizeH ) * ( 1 << shiftOutput );
          }
          else
#endif
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc ) * ( 1 << shiftOutput );
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc + dctSizeW * dctSizeH ) * ( 1 << shiftOutput );
          }
        }
      }
      else
      {
        for( int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++ )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc ) << shiftOutput;
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc + dctSizeW * dctSizeH ) << shiftOutput;
          }
          else
#endif
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc ) << shiftOutput;
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc + dctSizeW * dctSizeH ) << shiftOutput;
          }
        }
      }

      for( int y1 = 0; y1 < dctSizeH; y1++ )
      {
        for( int x1 = 0; x1 < dctSizeW; x1++ )
        {
          int xx = x * dctSizeW + x1;
          int yy = y * dctSizeH + y1;
          if( xx < extLeft / 2 || yy < extTop / 2 || xx >= width / 2 - extRight / 2 || yy >= height / 2 - extBottom / 2 )
          {
            continue;
          }
          int outCb;
          int outCr;

          if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
          {
            outCb = round( ( 0 + ( float ) bufRecCb.at( xx, yy ) * ( 1 << shiftInput ) ) );
            outCr = round( ( 0 + ( float ) bufRecCr.at( xx, yy ) * ( 1 << shiftInput ) ) );
          }
          else
          {
            outCb = ( ( 0 ) + ( bufRecCb.at( xx, yy ) << shiftInput ) );
            outCr = ( ( 0 ) + ( bufRecCr.at( xx, yy ) << shiftInput ) );
          }
          bufDstCb.at( xx, yy ) = Pel( Clip3<int>( 0, ( 1 << NNFilterUnified::log2OutputScale ) - 1, outCb ) );
          bufDstCr.at( xx, yy ) = Pel( Clip3<int>( 0, ( 1 << NNFilterUnified::log2OutputScale ) - 1, outCr ) );
        }
      }
    }
  }


#else
  PelBuf &bufDstCb    = bufDst.get(COMPONENT_Cb);
  PelBuf &bufDstCr    = bufDst.get(COMPONENT_Cr);
  CPelBuf bufRecCb    = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cb);
  CPelBuf bufRecCr    = pic.getRecBeforeDbfBuf(inferArea).get(COMPONENT_Cr);
  TCoeff  blockCb[dctSizeW * dctSizeH]     = {};
  TCoeff  blockCr[dctSizeW * dctSizeH]     = {};
  TCoeff  tempCoeffCb[dctSizeW * dctSizeH] = {};
  TCoeff  tempCoeffCr[dctSizeW * dctSizeH] = {};
  TCoeff  tmpCb[dctSizeW * dctSizeH]       = {};
  TCoeff  tmpCr[dctSizeW * dctSizeH]       = {};

  for( int y = 0; y < height / 2 / dctSizeH; y++ )
  {
    for( int x = 0; x < width / 2 / dctSizeW; x++ )
    {
#if JVET_AJ0054_EARLY_CROPPING
      if( earlyCropping )
      {
        if( y < 2 || y >= ( ( height / 2 / dctSizeH ) - 2 ) || x < 2 || x >= ( ( width / 2 / dctSizeW ) - 2 ) )
        {
          continue;
        }
      }
#endif      
      if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
      {
        for( int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++ )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc ) * ( 1 << shiftOutput );
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc + dctSizeW * dctSizeH ) * ( 1 << shiftOutput );
          }
          else
#endif
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc ) * ( 1 << shiftOutput );
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc + dctSizeW * dctSizeH ) * ( 1 << shiftOutput );
          }
        }
      }
      else
      {
        for( int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++ )
        {
#if JVET_AJ0054_EARLY_CROPPING
          if( earlyCropping )
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc ) << shiftOutput;
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y - 2, x - 2, cc + dctSizeW * dctSizeH ) << shiftOutput;
          }
          else
#endif
          {
            tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc ) << shiftOutput;
            tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output( 0, y, x, cc + dctSizeW * dctSizeH ) << shiftOutput;
          }
        }
      }
      tmpCb[0] = tempCoeffCb[0] + tempCoeffCb[1];
      tmpCb[1] = tempCoeffCb[2] + tempCoeffCb[3];
      tmpCb[2] = tempCoeffCb[0] - tempCoeffCb[1];
      tmpCb[3] = tempCoeffCb[2] - tempCoeffCb[3];
      blockCb[0] = tmpCb[0] + tmpCb[1];
      blockCb[1] = tmpCb[2] + tmpCb[3];
      blockCb[2] = tmpCb[0] - tmpCb[1];
      blockCb[3] = tmpCb[2] - tmpCb[3];
      tmpCr[0] = tempCoeffCr[0] + tempCoeffCr[1];
      tmpCr[1] = tempCoeffCr[2] + tempCoeffCr[3];
      tmpCr[2] = tempCoeffCr[0] - tempCoeffCr[1];
      tmpCr[3] = tempCoeffCr[2] - tempCoeffCr[3];
      blockCr[0] = tmpCr[0] + tmpCr[1];
      blockCr[1] = tmpCr[2] + tmpCr[3];
      blockCr[2] = tmpCr[0] - tmpCr[1];
      blockCr[3] = tmpCr[2] - tmpCr[3];

      for( int y1 = 0; y1 < dctSizeH; y1++ )
      {
        for( int x1 = 0; x1 < dctSizeW; x1++ )
        {
          int xx = x * dctSizeW + x1;
          int yy = y * dctSizeH + y1;
          if( xx < extLeft / 2 || yy < extTop / 2 || xx >= width / 2 - extRight / 2 || yy >= height / 2 - extBottom / 2 )
          {
            continue;
          }
          int outCb;
          int outCr;
          if constexpr( std::is_same<TypeSadlLFUnified, float>::value )
          {
            outCb = round( ( blockCb[( y1 * dctSizeW ) + x1] + ( float ) bufRecCb.at( xx, yy ) * ( 1 << shiftInput ) ) );
            outCr = round( ( blockCr[( y1 * dctSizeW ) + x1] + ( float ) bufRecCr.at( xx, yy ) * ( 1 << shiftInput ) ) );
          }
          else
          {
            outCb = ( ( blockCb[( y1 * dctSizeW ) + x1] ) + ( bufRecCb.at( xx, yy ) << shiftInput ) );
            outCr = ( ( blockCr[( y1 * dctSizeW ) + x1] ) + ( bufRecCr.at( xx, yy ) << shiftInput ) );
          }
          bufDstCb.at( xx, yy ) = Pel( Clip3<int>( 0, ( 1 << NNFilterUnified::log2OutputScale ) - 1, outCb ) );
          bufDstCr.at( xx, yy ) = Pel( Clip3<int>( 0, ( 1 << NNFilterUnified::log2OutputScale ) - 1, outCr ) );
        }
      }
    }
  }
#endif
}
#endif

void NNFilterUnified::filterBlock(Picture &pic, UnitArea inferArea, UnitArea outArea, int extLeft, int extRight,int extTop, int extBottom, int prmId)
{
  // get model
  auto &model    = *m_model;
  bool  inter    = pic.slices[0]->getSliceType() != I_SLICE;
  int   qpOffset = (inter ? prmId * 5 : prmId * 2) * (pic.slices[0]->getTLayer() >= 4 ? 1 : -1);
  int    seqQp   = pic.slices[0]->getPPS()->getPicInitQPMinus26() + 26 + qpOffset;
  int   sliceQp  = pic.slices[0]->getSliceQp();
  resizeInputs(inferArea.Y().width, inferArea.Y().height);

  const int    log2InputBitdepth = pic.cs->slice->clpRng(COMPONENT_Y).bd;   // internal bitdepth
  const double inputScalePred    = (1 << log2InputBitdepth);
  const double inputScaleQp      = (1 << log2InputQpScale);
  const double inputScaleIpb     = (1 << log2InputIbpScale);

  std::vector<InputData> listInputData;
#if NNLF_INPUT_Y_ONLY 
  listInputData.push_back({ NN_INPUT_REC, 0, inputScalePred, m_inputQuantizer[0] - log2InputBitdepth, true, false });
  listInputData.push_back({ NN_INPUT_PRED, 1, inputScalePred, m_inputQuantizer[1] - log2InputBitdepth, true, false });
  listInputData.push_back({ NN_INPUT_BS, 2, inputScalePred, m_inputQuantizer[2] - log2InputBitdepth, true, false });
#else
  listInputData.push_back({ NN_INPUT_REC, 0, inputScalePred, m_inputQuantizer[0] - log2InputBitdepth, true, true });
#if NNVC_USE_PRED
  listInputData.push_back({ NN_INPUT_PRED, 1, inputScalePred, m_inputQuantizer[1] - log2InputBitdepth, true, true });
#endif
  listInputData.push_back({ NN_INPUT_BS, 2, inputScalePred, m_inputQuantizer[2] - log2InputBitdepth, true, true });
#endif
  listInputData.push_back({ NN_INPUT_GLOBAL_QP, 3, inputScaleQp, m_inputQuantizer[3] - log2InputQpScale, true, false });
#if JVET_AJ0124_QP_BLOCK
  listInputData.push_back({ NN_INPUT_LOCAL_QP_BLOCK, 4, inputScaleQp, m_inputQuantizer[4] - log2InputQpScale, true, false });
#else
  listInputData.push_back({ NN_INPUT_LOCAL_QP, 4, inputScaleQp, m_inputQuantizer[4] - log2InputQpScale, true, false });
#endif
  listInputData.push_back({ NN_INPUT_IPB, 5, inputScaleIpb, m_inputQuantizer[5] - log2InputIbpScale, true, false });
 #if JVET_AH0080_TRANS_INPUT
  if (m_nnlfTransInput)
  {
    NNInference::prepareTransInputs<TypeSadlLFUnified>(&pic, inferArea, m_inputs, seqQp, sliceQp, -1 /* sliceType */, listInputData);
  }
  else
#endif
  NNInference::prepareInputs<TypeSadlLFUnified>(&pic, inferArea, m_inputs, seqQp, sliceQp, -1 /* sliceType */, listInputData);

  NNInference::infer<TypeSadlLFUnified>(model, m_inputs);

  UnitArea inferAreaNoExt(inferArea.chromaFormat, Area(inferArea.lx() + extLeft, inferArea.ly() + extTop, inferArea.lwidth() - extLeft - extRight, inferArea.lheight() - extTop - extBottom));
  UnitArea InferAreaExt(inferArea.chromaFormat, Area(-extLeft, -extTop, inferArea.lwidth(), inferArea.lheight()));
  PelUnitBuf bufDst = m_scaled[0][prmId].getBuf(inferAreaNoExt).subBuf(InferAreaExt);
 
#if JVET_AH0080_TRANS_INPUT
  if (m_nnlfTransInput)
  {
    extractOutputsDCT(pic, model, bufDst, inferArea, extLeft, extRight, extTop, extBottom);
  }
  else
#endif
   extractOutputs(pic, model, bufDst, inferArea, extLeft, extRight, extTop, extBottom);
}
  
void NNFilterUnified::filter(Picture &pic, const bool isDec)
{
  const CodingStructure &cs  = *pic.cs;
  const PreCalcValues &  pcv = *cs.pcv;
  int cpt = 0;
  for (int y = 0; y < m_picprm->numBlocksHeight; ++y)
  {
    for (int x = 0; x < m_picprm->numBlocksWidth; ++x, ++cpt)
    {
      int prmId = m_picprm->prmId[cpt];

      if (prmId == -1)
      {
        continue;
      }

      int xPos  = x * m_picprm->blockSize;
      int yPos  = y * m_picprm->blockSize;
      int width = (xPos + m_picprm->blockSize > (int) pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : m_picprm->blockSize;
      int height = (yPos + m_picprm->blockSize > (int) pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : m_picprm->blockSize;

#if JVET_AF0043_AF0205_PADDING
      int extLeft = m_picprm->extension;
      int extRight = m_picprm->extension;
      int extTop = m_picprm->extension;
      int extBottom = m_picprm->extension;
#else
      int extLeft   = xPos > 0 ? m_picprm->extension : 0;
      int extRight  = (xPos + width + m_picprm->extension > (int) pcv.lumaWidth) ? (pcv.lumaWidth - xPos - width)
                                                                                 : m_picprm->extension;
      int extTop    = yPos > 0 ? m_picprm->extension : 0;
      int extBottom = (yPos + height + m_picprm->extension > (int) pcv.lumaHeight) ? (pcv.lumaHeight - yPos - height)
                                                                                   : m_picprm->extension;
#endif

      int            extXPos   = xPos - extLeft;
      int            extYPos   = yPos - extTop;
      int            extWidth  = width + extLeft + extRight;
      int            extHeight = height + extTop + extBottom;
      const UnitArea inferArea(cs.area.chromaFormat, Area(extXPos, extYPos, extWidth, extHeight));

      const UnitArea outArea(cs.area.chromaFormat, Area(xPos, yPos, width, height));

#if JVET_AJ0166_BlOCK_SIZE_INV
      if( ( m_picprm->filterMode == ( int ) NNLFUnifiedID::HOP5 ) && ( m_picprm->blockSize > 128 ) )
      {
        int xPos_org = x * m_picprm->blockSize;
        int yPos_org = y * m_picprm->blockSize;
        int xPos, yPos, width, height, extXPos, extYPos, extWidth, extHeight;
        for( int i = 0; i < 2; i++ )
        {
          for( int j = 0; j < 2; j++ )
          {
            xPos = xPos_org + i * ( m_picprm->blockSize >> 1 );
            yPos = yPos_org + j * ( m_picprm->blockSize >> 1 );
            if( ( xPos < pcv.lumaWidth ) && ( yPos < pcv.lumaHeight ) )
            {
              width = ( xPos + ( m_picprm->blockSize >> 1 ) > ( int ) pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : m_picprm->blockSize >> 1;
              height = ( yPos + ( m_picprm->blockSize >> 1 ) > ( int ) pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : m_picprm->blockSize >> 1;
              extXPos = xPos - extLeft;
              extYPos = yPos - extTop;
              extWidth = width + extLeft + extRight;
              extHeight = height + extTop + extBottom;
              const UnitArea inferArea( cs.area.chromaFormat, Area( extXPos, extYPos, extWidth, extHeight ) );
              const UnitArea outArea( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
              filterBlock( pic, inferArea, outArea, extLeft, extRight, extTop, extBottom, prmId );
            }
          }
        }
      }
      else
#endif      
        filterBlock(pic, inferArea, outArea, extLeft, extRight, extTop, extBottom, prmId);

      const UnitArea inferAreaNoExt(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      PelUnitBuf     filteredBuf = getFilteredBuf(prmId, inferAreaNoExt);
      PelUnitBuf     scaledBuf   = getScaledBuf(0, prmId, inferAreaNoExt);
      PelUnitBuf     recBuf      = pic.getRecoBuf(inferAreaNoExt);

      roundToOutputBitdepth(scaledBuf, filteredBuf, cs.slice->clpRngs());

      if (!isDec)
      {
        continue;
      }
      if (m_picprm->sprm.scaleFlag != -1)
      {
        int scaleIdx = m_picprm->sprm.scaleFlag;
        for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
        {
          ComponentID compID = ComponentID(compIdx);
          scaleIdx == 0 ? scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), recBuf.get(compID),
                                             m_picprm->sprm.scale[compID][prmId]
#if JVET_AF0085_RESIDUAL_ADJ
            , m_picprm->sprm.offset[compID][prmId][scaleIdx]
#endif
          )
                        : scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), recBuf.get(compID),
                                             scaleCandidates[scaleIdx]
#if JVET_AF0085_RESIDUAL_ADJ
                          , m_picprm->sprm.offset[compID][prmId][scaleIdx]
#endif
                        );
        }
      }
      else
      {
#if JVET_AF0085_RESIDUAL_ADJ
        for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
        {
          ComponentID compID = ComponentID(compIdx);
          scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), filteredBuf.get(compID), 0, m_picprm->sprm.offset[compID][prmId][3]);
        }
#endif
        recBuf.copyFrom(filteredBuf);
      }
    }
  }
}

void NNFilterUnified::scaleResidualBlock(Picture &pic, ComponentID compID, UnitArea inferAreaNoExt, CPelBuf src, PelBuf tgt,
                                     int scale
#if JVET_AF0085_RESIDUAL_ADJ
  , int roaOffset
#endif
) const
{
  const CodingStructure &cs            = *pic.cs;
  const Slice &          slice         = *cs.slice;
  const int              inputBitdepth = slice.clpRng(COMPONENT_Y).bd;   // internal bitdepth
  const int              shift         = log2OutputScale - inputBitdepth;
  const int              shift2        = shift + log2ResidueScale;
  const int              offset        = (1 << shift2) / 2;
  CPelBuf                rec           = pic.getRecoBuf(inferAreaNoExt).get(compID);
  int                    width         = inferAreaNoExt.lwidth();
  int                    height        = inferAreaNoExt.lheight();
#if JVET_AF0085_RESIDUAL_ADJ
  CPelBuf                recBeforeDbf = pic.getRecBeforeDbfBuf(inferAreaNoExt).get(compID);
#endif

  if (compID)
  {
    width  = width / 2;
    height = height / 2;
  }

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
#if JVET_AF0085_RESIDUAL_ADJ
      if (scale > 0)
      {
        // positive-, negative+
        int v = 0;
        if( ( src.at( x, y ) - ( recBeforeDbf.at( x, y ) << shift ) ) >= ( roaOffset << shift ) )
        {
          v = ( ( ( int ) rec.at( x, y ) << shift2 ) + ( src.at( x, y ) - ( rec.at( x, y ) << shift ) - ( roaOffset << shift ) ) * scale + offset ) >> shift2;
        }
        else if( ( src.at( x, y ) - ( recBeforeDbf.at( x, y ) << shift ) ) <= ( -roaOffset << shift ) )
        {
          v = ( ( ( int ) rec.at( x, y ) << shift2 ) + ( src.at( x, y ) - ( rec.at( x, y ) << shift ) + ( roaOffset << shift ) ) * scale + offset ) >> shift2;
        }
        else
        {
          v = ( ( ( int ) rec.at( x, y ) << shift2 ) + ( src.at( x, y ) - ( rec.at( x, y ) << shift ) ) * scale + offset ) >> shift2;
        }

        tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, v));
      }
      else
      {
        if( ( src.at( x, y ) - ( recBeforeDbf.at( x, y ) << shift ) ) >= ( roaOffset << shift ) )
        {
          tgt.at( x, y ) = Pel( Clip3<int>( 0, ( 1 << inputBitdepth ) - 1, tgt.at( x, y ) - roaOffset ) );
        }
        else if( ( src.at( x, y ) - ( recBeforeDbf.at( x, y ) << shift ) ) <= ( -roaOffset << shift ) )
        {
          tgt.at( x, y ) = Pel( Clip3<int>( 0, ( 1 << inputBitdepth ) - 1, tgt.at( x, y ) + roaOffset ) );
        }
        else
        {
          tgt.at( x, y ) = Pel( Clip3<int>( 0, ( 1 << inputBitdepth ) - 1, tgt.at( x, y ) ) );
        }
      }
#else
      int v = (((int) rec.at(x, y) << shift2) + (src.at(x, y) - (rec.at(x, y) << shift)) * scale + offset) >> shift2;
      tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, v));
#endif
    }
  }
}
#endif
