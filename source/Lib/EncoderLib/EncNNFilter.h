/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2018, ITU/ISO/IEC
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

/** \file     EncNNFilterSet0.h
 \brief    estimation part of cnn loop filter class (header)
 */

/** \file     EncNNFilter.h
 \brief    cnn loop filter on the encoder side class (header)
 */

#ifndef __ENCNNFILTER__
#define __ENCNNFILTER__

#include "CommonLib/CommonDef.h"

#if JVET_AB0068_AC0328_NNLF_RDO || NN_HOP_RDO

#include "Picture.h"
#include <fstream>
using namespace std;

namespace sadl
{
  template<typename T> class Model;
  template<typename T> class Tensor;
}


class EncNNFilter
{
public:
  int                      m_NumModelsValid;
#if NN_HOP_RDO
  std::string              m_rdoNnlfHopModelName;
#endif
#if JVET_AB0068_AC0328_NNLF_RDO
  std::string              m_interLumaRdNNFilter0;
  std::string              m_intraLumaRdNNFilter0;
  std::string              m_interLumaRdNNFilter1;
  std::string              m_intraLumaRdNNFilter1;
#endif

  PelStorage               m_tempBuf;
  PelStorage               m_tempRecBuf  [MAX_NUM_COMPONENT];
  PelStorage               m_tempPredBuf [MAX_NUM_COMPONENT];
  PelStorage               m_tempSplitBuf[MAX_NUM_COMPONENT];
  void                     createEnc(const int picWidth, const int picHeight, const int maxCuWidth, const int maxCuHeight, const ChromaFormat format, const int nnlfSet1NumParams);
  void                     destroyEnc();
  void                     initEnc(const int numModels, const std::string& interLuma, const std::string& intraLuma);
  
private:  
  template<typename T> 
  void prepareInputsLumaRd(Picture* pic, UnitArea inferArea, std::vector<sadl::Tensor<T>>& inputs, int sliceQp, int baseQp, bool inter);
  template<typename T> 
  void extractOutputsLumaRd(Picture* pic, sadl::Model<T>& m, PelStorage& tempBuf, UnitArea inferArea);
  template<typename T>
  void cnnFilterLumaBlockRd(Picture* pic, UnitArea inferArea, bool inter);

public:
  void cnnFilterLumaBlockRd_ext(Picture* pic, UnitArea inferArea, bool inter);
};
#endif
#endif
