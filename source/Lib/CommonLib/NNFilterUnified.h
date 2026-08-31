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

#pragma once

#include <memory>
#include "CommonDef.h"
#include "Unit.h"
#if NN_LF_UNIFIED
// fwd
struct Picture;
struct PelStorage;
namespace sadl
{
  template<typename T> class Model;
  template<typename T> class Tensor;
}   // namespace sadl



class NNFilterUnified
{
public:
  static constexpr int log2ResidueScale = 8;
  static constexpr int log2OutputScale  = 10 + 4;
  static constexpr float nnResidueScaleDerivationUpBound = 1.25f;
  static constexpr float nnResidueScaleDerivationLowBound = 0.0625f;
  static constexpr int maxScale = (1<<log2ResidueScale);
  static constexpr int numInputs = 6;

  // parameters signaled at slice level
  struct SliceParameters
  {
    int mode; // -1: off
    int scaleFlag;

    int scale[MAX_NUM_COMPONENT][MAX_PRM_NUM + 1];
#if JVET_AF0085_RESIDUAL_ADJ
    int offset[MAX_NUM_COMPONENT][MAX_PRM_NUM + 1][4];
#endif


    SliceParameters()
    {
      reset();
    }
    void reset()
    {
      mode = -1;
      scaleFlag = -1;
      std::memset( scale, 0, sizeof( scale ) );
#if JVET_AF0085_RESIDUAL_ADJ
      std::memset(offset, 0, sizeof(offset));
#endif
      
    }
  };
  
  // parameters used to filter the picture
  struct FilterParameters
  {
    int              blockSize;
    int              extension;
    int              numBlocksWidth;
    int              numBlocksHeight;
    int              prmNum;
    std::vector<int> prmId; // -1: off
    SliceParameters  sprm;
    bool             temporal;
#if JVET_AJ0166_BlOCK_SIZE_INV
    int              filterMode;
#endif
  };

  static constexpr int scaleCandidates[3] = { 0, (maxScale + (maxScale << 1)) >> 2, maxScale >> 1 };
  static constexpr int blockSizes[]={128,256};

  void init(const std::string &filename, int picWidth, int picHeight, ChromaFormat format, int prmNum);
  void destroy();

  // filter the whole picture with prms
  void filter(Picture &pic, const bool isDec = true);

  // just filter the block, output on log2OutputScale bits
  void filterBlock(Picture &pic, UnitArea inferArea, UnitArea outArea, int extLeft, int extRight, int extTop,int extBottom, int prmId);

  // put scaled res+rec in tgt from the filtered src
  void scaleResidualBlock(Picture &pic, ComponentID compID, UnitArea inferArea, CPelBuf src, PelBuf tgt,
                          int scale
#if JVET_AF0085_RESIDUAL_ADJ
    , int roaOffset
#endif
  ) const;
  
  PelUnitBuf  getFilteredBuf (const int prmId)                              { return m_filtered[prmId];              }
  CPelUnitBuf getFilteredBuf (const int prmId)                        const { return m_filtered[prmId];              }
  PelUnitBuf  getFilteredBuf (const int prmId, const UnitArea unit)         { return m_filtered[prmId].getBuf(unit); }
  CPelUnitBuf getFilteredBuf (const int prmId, const UnitArea unit)   const { return m_filtered[prmId].getBuf(unit); }
  
  PelUnitBuf  getScaledBuf   (const int scaleId, const int prmId)                              { return m_scaled[scaleId][prmId];       }
  CPelUnitBuf getScaledBuf   (const int scaleId, const int prmId)                        const { return m_scaled[scaleId][prmId];                }
  PelUnitBuf  getScaledBuf   (const int scaleId, const int prmId, const UnitArea unit)         { return m_scaled[scaleId][prmId].getBuf(unit);   }
  CPelUnitBuf getScaledBuf   (const int scaleId, const int prmId, const UnitArea unit)   const { return m_scaled[scaleId][prmId].getBuf(unit);   }
  
  FilterParameters& getPicprms ()                                                { return *m_picprm;                      }
  void              setPicprms (FilterParameters* picprm)                        { m_picprm = picprm;                     }
  SliceParameters&  getSliceprms ()                                              { return m_picprm->sprm;                 }
  void              setSliceprms (SliceParameters sprm)                          { m_picprm->sprm = sprm;                 }
#if JVET_AH0080_TRANS_INPUT
  void              nnlfTransInput(bool b) { m_nnlfTransInput = b; }
#endif
private:
  int  m_blocksize[2];   // current inputs size
  int  m_inputQuantizer[numInputs] = {};
  void resizeInputs(int width, int height);
  std::unique_ptr<sadl::Model<TypeSadlLFUnified>> m_model;
  std::vector<sadl::Tensor<TypeSadlLFUnified>>    m_inputs;
  std::vector<PelStorage>                m_filtered; // filtered results of each parameter

  std::vector<PelStorage>                m_scaled[3];   // residue scaling results
 
  
  FilterParameters                       *m_picprm; // filtering parameters
#if JVET_AH0080_TRANS_INPUT
  bool                                   m_nnlfTransInput = true;
#endif
};

#endif