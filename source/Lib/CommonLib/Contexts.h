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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTS__
#define __CONTEXTS__

#include "CommonDef.h"
#include "Slice.h"

#include <vector>

static constexpr int     PROB_BITS   = 15;   // Nominal number of bits to represent probabilities
#if EC_HIGH_PRECISION
static constexpr int     PROB_BITS_0 = 15;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 15;   // Number of bits to represent 2nd estimate
#else
static constexpr int     PROB_BITS_0 = 10;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 14;   // Number of bits to represent 2nd estimate
#endif
static constexpr int     MASK_0      = ~(~0u << PROB_BITS_0) << (PROB_BITS - PROB_BITS_0);
static constexpr int     MASK_1      = ~(~0u << PROB_BITS_1) << (PROB_BITS - PROB_BITS_1);
static constexpr uint8_t DWS         = 8;   // 0x47 Default window sizes
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
static constexpr uint8_t DWE         = 18;  // default weights
static constexpr uint8_t DWO         = 119; // default window offsets
#endif

struct BinFracBits
{
  uint32_t intBits[2];
};


enum BPMType
{
  BPM_Undefined = 0,
  BPM_Std,
  BPM_NUM
};

#if JVET_AG0117_CABAC_SPATIAL_TUNING
struct BinStoreElem
{
  uint8_t  bins;
  uint8_t  count;
  unsigned ctxId;
  
  BinStoreElem(unsigned b, unsigned c) : bins(b), count(1), ctxId(c) {}

  unsigned bin(int i) const
  {
    return (bins >> i) & 1;
  }
  
  void addBin(unsigned bin)
  {
    bins |= bin << count++;
  }
};

typedef std::vector<BinStoreElem> BinStoreVector;

class BinBuffer
{
public:
  BinBuffer() : m_active(false), m_maxSize(0), m_numCtx(0), m_maxBinsPerCtx(0), m_buffer(nullptr) {}
  ~BinBuffer() {}

  void init(int size, int numCtx, int maxBinsPerCtx)
  {
    m_active           = false;
    m_maxSize          = size;
    m_numCtx           = numCtx;
    m_maxBinsPerCtx    = maxBinsPerCtx;
    m_indOfCtxInBuffer = std::vector<int16_t>(numCtx, -1);

    if ( m_maxBinsPerCtx > 8 )
    {
      fprintf(stdout, "Chance type of BinStoreElem/bins to support over 8 bins per context\n");
      exit(0);
    }
  }

  void addBin( unsigned bin, unsigned ctxId )
  {
    if( m_active )
    {
      int bufferInd = m_indOfCtxInBuffer[ctxId];
      
      if( bufferInd >= 0 ) // Already active context type
      {
        BinStoreElem& store = (*m_buffer)[bufferInd];

        if( store.count < m_maxBinsPerCtx ) // Room to add more bins
        {
          store.addBin( bin );
        }
      }
      else if ( m_buffer->size() < m_maxSize ) // Room to add more context types
      {
        m_indOfCtxInBuffer[ctxId] = static_cast<int16_t>(m_buffer->size());
        m_buffer->push_back( BinStoreElem( bin, ctxId ) );
      }
    }
  }

  void clear()           { m_buffer->clear(); std::fill(m_indOfCtxInBuffer.begin(), m_indOfCtxInBuffer.end(), -1); }
  void setActive(bool b) { m_active = b; }
  
  const BinStoreVector* getBinStoreVector() const { return m_buffer; }
  void                  setBinStoreVector(BinStoreVector *bb) { m_buffer = bb; if (bb) clear(); }

private:
  bool                 m_active;
  std::size_t          m_maxSize;
  int                  m_numCtx;
  int                  m_maxBinsPerCtx;
  std::vector<int16_t> m_indOfCtxInBuffer; // Only used in active CTU for convenience (8 bit enough)
  BinStoreVector*      m_buffer;
};

class BinBufferer
{
public:
  BinBufferer(): m_binBuffer () {}

  void             setBinBufferActive ( bool b )             { m_binBuffer.setActive(b); }
  void             setBinBuffer       ( BinStoreVector *bb ) { m_binBuffer.setBinStoreVector(bb); }
  const BinStoreVector* getBinBuffer  ( )             const  { return m_binBuffer.getBinStoreVector(); }
  void             initBufferer(int size, int numCtx, int maxBinsPerCtx) { m_binBuffer.init( size, numCtx, maxBinsPerCtx); }
  virtual void     updateCtxs         ( BinStoreVector *bb ) = 0;

protected:
  BinBuffer m_binBuffer;
};
#endif

class ProbModelTables
{
protected:
#if EC_HIGH_PRECISION
	static const BinFracBits m_binFracBits[512];
#else
  static const BinFracBits m_binFracBits[256];
#endif
  static const uint8_t      m_RenormTable_32  [ 32];          // Std         MP   MPI
};

class BinProbModelBase : public ProbModelTables
{
public:
  BinProbModelBase () {}
  ~BinProbModelBase() {}
  static uint32_t estFracBitsEP ()                    { return  (       1 << SCALE_BITS ); }
  static uint32_t estFracBitsEP ( unsigned numBins )  { return  ( numBins << SCALE_BITS ); }
};

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
const uint8_t weightedAdaptRate[5] = { 10, 12, 16, 20, 22 };
#endif

class BinProbModel_Std : public BinProbModelBase
{
public:
  BinProbModel_Std()
  {
    uint16_t half = 1 << (PROB_BITS - 1);
    m_state[0]    = half;
    m_state[1]    = half;
    m_rate        = DWS;
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
    m_weight        = DWE;
    m_stateUsed[0]  = half;
    m_stateUsed[1]  = half;
    m_rateOffset[0] = DWO;
    m_rateOffset[1] = DWO;
#endif

  }
  ~BinProbModel_Std ()                {}
public:
  void            init              ( int qp, int initId );
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  void update(unsigned bin)
  {
    int rate0 = m_rate >> 4;
    int rate1 = m_rate & 15;

    auto ws = m_rateOffset[bin];

    int rateUsed0 = std::max( 2, rate0 + (ws >> 4) - ADJUSTMENT_RANGE );
    int rateUsed1 = std::max( 2, rate1 + (ws & 15) - ADJUSTMENT_RANGE );

    m_stateUsed[0] = m_state[0] - ((m_state[0] >> rateUsed0) & MASK_0);
    m_stateUsed[1] = m_state[1] - ((m_state[1] >> rateUsed1) & MASK_1);

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    m_state[1] -= (m_state[1] >> rate1) & MASK_1;

    if (bin)
    {
      m_stateUsed[0] += (0x7FFFU >> rateUsed0) & MASK_0;
      m_stateUsed[1] += (0x7FFFU >> rateUsed1) & MASK_1;

      m_state[0] += (0x7fffu >> rate0) & MASK_0;
      m_state[1] += (0x7fffu >> rate1) & MASK_1;
    }
  }
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  void updateShortWin(unsigned bin)
  {
    int rate0     = m_rate >> 4;
    auto ws       = m_rateOffset[bin];
    int rateUsed0 = std::max( 2, rate0 + (ws >> 4) - ADJUSTMENT_RANGE );

    m_stateUsed[0] = m_state[0] - ((m_state[0] >> rateUsed0) & MASK_0);
    m_state[0]    -= (m_state[0] >> rate0) & MASK_0;

    if (bin)
    {
      m_stateUsed[0] += (0x7FFFU >> rateUsed0) & MASK_0;
      m_state[0]     += (0x7fffu >> rate0) & MASK_0;
    }
  }
#endif
#else
  void update(unsigned bin)
  {
    int rate0 = m_rate >> 4;
    int rate1 = m_rate & 15;

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    m_state[1] -= (m_state[1] >> rate1) & MASK_1;
    if (bin)
    {
      m_state[0] += (0x7fffu >> rate0) & MASK_0;
      m_state[1] += (0x7fffu >> rate1) & MASK_1;
    }
  }
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  void updateShortWin(unsigned bin)
  {
    int rate0 = m_rate >> 4;

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    
    if (bin)
    {
      m_state[0] += (0x7fffu >> rate0) & MASK_0;
    }
  }
#endif
#endif
  void setLog2WindowSize(uint8_t log2WindowSize)
  {
    int rate0 = 2 + ((log2WindowSize >> 2) & 3);
    int rate1 = 3 + rate0 + (log2WindowSize & 3);
    m_rate    = 16 * rate0 + rate1;
    CHECK(rate1 > 9, "Second window size is too large!");
  }
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  void     setAdaptRateWeight( uint8_t weight ) { m_weight = weight; }
  uint8_t  getAdaptRateWeight() const           { return m_weight; }
  void     setWinSizes( uint8_t rate )          { m_rate = rate; }
  uint8_t  getWinSizes() const                  { return m_rate; }
  void     setAdaptRateOffset(uint8_t rateOffset, bool bin ) { m_rateOffset[bin] = rateOffset;}
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  uint8_t  getAdaptRateOffset( bool bin ) const { return m_rateOffset[bin]; }
#endif
#endif

  void estFracBitsUpdate(unsigned bin, uint64_t &b)
  {
    b += estFracBits(bin);
    update(bin);
  }

  uint32_t        estFracBits(unsigned bin) const { return getFracBitsArray().intBits[bin]; }
  static uint32_t estFracBitsTrm(unsigned bin) { return (bin ? 0x3bfbb : 0x0010c); }
#if EC_HIGH_PRECISION
	BinFracBits     getFracBitsArray() const { return m_binFracBits[state_est()]; }
#else
  BinFracBits     getFracBitsArray() const { return m_binFracBits[state()]; }
#endif
public:
#if EC_HIGH_PRECISION
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  uint16_t        state_est() const
  {
    uint8_t  wIdx0 = (m_weight >> 3);
    uint8_t  wIdx1 = (m_weight & 0x07);
    uint32_t w0 = weightedAdaptRate[wIdx0];
    uint32_t w1 = weightedAdaptRate[wIdx1];
    uint32_t pd;
    if( (w0 + w1) <= 32 )
    {
      pd = (uint32_t( m_stateUsed[0] ) * w0 + uint32_t( m_stateUsed[1] ) * w1) >> 11;
    }
    else
    {
      pd = (uint32_t( m_stateUsed[0] ) * w0 + uint32_t( m_stateUsed[1] ) * w1) >> 12;
    }
    return uint16_t( pd );
  }
  uint16_t       state() const
  {
    uint8_t  wIdx0 = (m_weight >> 3);
    uint8_t  wIdx1 = (m_weight & 0x07);
    uint32_t w0 = weightedAdaptRate[wIdx0];
    uint32_t w1 = weightedAdaptRate[wIdx1];
    uint32_t pd;
    if( (w0 + w1) <= 32 )
    {
      pd = (uint32_t( m_stateUsed[0] ) * w0 + uint32_t( m_stateUsed[1] ) * w1) >> 5;
    }
    else
    {
      pd = (uint32_t( m_stateUsed[0] ) * w0 + uint32_t( m_stateUsed[1] ) * w1) >> 6;
    }
    return uint16_t( pd );
  }
#else
  uint16_t state_est() const { return (m_state[0] + m_state[1]) >> 7; }
  uint16_t state() const { return (m_state[0] + m_state[1]) >> 1; }
#endif

	uint8_t mps() const { return state() >> 14; }
	uint8_t getLPS(unsigned range) const
	{
		uint16_t q = state();
    if( q & 0x4000 )
    {
      q = q ^ 0x7fff;
    }
		return ((range * (q >> 6)) >> 9) + 1;
	}
#else
  uint8_t state() const { return (m_state[0] + m_state[1]) >> 8; }
  uint8_t mps() const { return state() >> 7; }
  uint8_t getLPS(unsigned range) const
  {
    uint16_t q = state();
    if (q & 0x80)
      q = q ^ 0xff;
    return ((q >> 2) * (range >> 5) >> 1) + 4;
  }
#endif
  static uint8_t  getRenormBitsLPS  ( unsigned LPS )                    { return    m_RenormTable_32  [LPS>>3]; }
  static uint8_t  getRenormBitsRange( unsigned range )                  { return    1; }
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  std::pair<uint16_t, uint16_t> getState() const { return std::pair<uint16_t, uint16_t>(m_state[0], m_state[1]); }

  void setState( std::pair<uint16_t, uint16_t> pState )
  {
    m_state[0] = pState.first;
    m_state[1] = pState.second;
    m_stateUsed[0] = m_state[0];
    m_stateUsed[1] = m_state[1];
  }
#else
  uint16_t getState() const { return m_state[0] + m_state[1]; }
  void     setState(uint16_t pState)
  {
    m_state[0] = (pState >> 1) & MASK_0;
    m_state[1] = (pState >> 1) & MASK_1;
  }
#endif
public:
  uint64_t estFracExcessBits(const BinProbModel_Std &r) const
  {
#if EC_HIGH_PRECISION
		int n = 2 * state_est() + 1;
		return ((1024 - n) * r.estFracBits(0) + n * r.estFracBits(1) + 512) >> 10;
#else
    int n = 2 * state() + 1;
    return ((512 - n) * r.estFracBits(0) + n * r.estFracBits(1) + 256) >> 9;
#endif
  }

private:
  uint16_t m_state[2];
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT 
  uint16_t m_stateUsed[2];
  uint8_t  m_rateOffset[2];
  uint8_t  m_weight;
#endif
  uint8_t  m_rate;
};



class CtxSet
{
public:
  CtxSet( uint16_t offset, uint16_t size ) : Offset( offset ), Size( size ) {}
  CtxSet( const CtxSet& ctxSet ) : Offset( ctxSet.Offset ), Size( ctxSet.Size ) {}
  CtxSet( std::initializer_list<CtxSet> ctxSets );
public:
  uint16_t  operator()  ()  const
  {
    return Offset;
  }
  uint16_t  operator()  ( uint16_t inc )  const
  {
    CHECKD( inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "]." );
    return Offset + inc;
  }
  bool operator== ( const CtxSet& ctxSet ) const
  {
    return ( Offset == ctxSet.Offset && Size == ctxSet.Size );
  }
  bool operator!= ( const CtxSet& ctxSet ) const
  {
    return ( Offset != ctxSet.Offset || Size != ctxSet.Size );
  }
public:
  uint16_t  Offset;
  uint16_t  Size;
};



class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet   SplitFlag;
  static const CtxSet   SplitQtFlag;
  static const CtxSet   SplitHvFlag;
  static const CtxSet   Split12Flag;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  static const CtxSet   ModeConsFlag;
#endif
  static const CtxSet   SkipFlag;
  static const CtxSet   MergeFlag;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  static const CtxSet   MergeFlagOppositeLic;
  static const CtxSet   TmMergeFlagOppositeLic;
  static const CtxSet   AffineFlagOppositeLic;
#endif
  static const CtxSet   RegularMergeFlag;
  static const CtxSet   MergeIdx;
#if JVET_AG0164_AFFINE_GPM
  static const CtxSet   GpmMergeIdx;
  static const CtxSet   GpmAffMergeIdx;
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  static const CtxSet   TmMergeIdx;
#endif
#if JVET_X0049_ADAPT_DMVR
  static const CtxSet   BMMergeFlag;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  static const CtxSet   affBMFlag;
#endif
#if JVET_AA0070_RRIBC
  static const CtxSet   rribcFlipType;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  static const CtxSet bvOneZeroComp;
#endif
#if JVET_Y0065_GPM_INTRA
  static const CtxSet   GPMIntraFlag;
#endif
#if JVET_AI0082_GPM_WITH_INTER_IBC
  static const CtxSet   GpmInterIbcFlag;
  static const CtxSet   GpmInterIbcIdx;
#endif
  static const CtxSet   PredMode;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  static const CtxSet   SeparateTree;
#endif
  static const CtxSet   MultiRefLineIdx;
  static const CtxSet   IntraLumaMpmFlag;
#if SECONDARY_MPM
  static const CtxSet   IntraLumaSecondMpmFlag;
#if JVET_AD0085_MPM_SORTING
  static const CtxSet   IntraLumaSecondMpmIdx;
#endif
#endif
  static const CtxSet   IntraLumaPlanarFlag;
#if JVET_AL0125_IMPROVEMENT_ON_MPM
  static const CtxSet   IntraLumaMPMfistIdxFlag;
#endif
#if SECONDARY_MPM
  static const CtxSet   IntraLumaMPMIdx;
#endif
  static const CtxSet   CclmModeFlag;
  static const CtxSet   CclmModeIdx;
  static const CtxSet   IntraChromaPredMode;
#if JVET_AD0188_CCP_MERGE
  static const CtxSet   nonLocalCCP;
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  static const CtxSet   ccpMergeAdjust;
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  static const CtxSet   decoderDerivedCCP;
  static const CtxSet   ddNonLocalCCP;
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
  static const CtxSet   DimdChromaMode;
#endif
  static const CtxSet   ChromaFusionMode;
#if JVET_AC0119_LM_CHROMA_FUSION
  static const CtxSet   ChromaFusionType;
  static const CtxSet   ChromaFusionCclm;
#endif
#endif
#if JVET_AC0071_DBV
  static const CtxSet DbvChromaMode;
#endif
  static const CtxSet   MipFlag;
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  static const CtxSet PnnLuminanceFlag;
#endif
#if JVET_AK0097_LAST_POS_SIGNALING
  static const CtxSet lastXSecondaryPrefix;
  static const CtxSet lastYSecondaryPrefix;
  static const CtxSet lastXSuffix[3];
  static const CtxSet lastYSuffix[3];
#endif
#if JVET_V0130_INTRA_TMP
  static const CtxSet   TmpFlag;
#if JVET_AD0086_ENHANCED_INTRA_TMP
  static const CtxSet   TmpIdx;
  static const CtxSet   TmpFusion;
#if JVET_AG0136_INTRA_TMP_LIC
  static const CtxSet   TmpLic;
  static const CtxSet   ItmpLicIndex;
#endif
#endif  
#endif
#if MMLM
  static const CtxSet   MMLMFlag;
#endif
  static const CtxSet   DeltaQP;
  static const CtxSet   InterDir;
  static const CtxSet   RefPic;
#if JVET_Z0054_BLK_REF_PIC_REORDER
  static const CtxSet   RefPicLC;
#endif
  static const CtxSet   MmvdFlag;
  static const CtxSet   MmvdMergeIdx;
  static const CtxSet   MmvdStepMvpIdx;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  static const CtxSet   MmvdStepMvpIdxECM3;
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  static const CtxSet   GeoBlendFlag;
#if JVET_AJ0274_REGRESSION_GPM_TM
  static const CtxSet   GeoBlendTMFlag;
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  static const CtxSet   GeoBlendIntraFlag;
#endif
#endif
#if JVET_W0097_GPM_MMVD_TM
  static const CtxSet   GeoMmvdFlag;
  static const CtxSet   GeoMmvdStepMvpIdx;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  static const CtxSet   GeoBldFlag;
#endif
#if JVET_AL0134_SGPM_INTER
  static const CtxSet   SgpmInterFlag;
  static const CtxSet   SgpmInterTmFlag;
#if JVET_AN0093_JRGPM_WITH_AFFINE_AND_INTRA
  static const CtxSet   SgpmIntraFlag;
  static const CtxSet   SgpmAffineFlag;
#endif 
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  static const CtxSet   GeoSubModeIdx;
#endif
  static const CtxSet   SubblockMergeFlag;
  static const CtxSet   AffineFlag;
  static const CtxSet   AffineType;
  static const CtxSet   AffMergeIdx;
#if AFFINE_MMVD
  static const CtxSet   AfMmvdFlag;
  static const CtxSet   AfMmvdIdx;
  static const CtxSet   AfMmvdOffsetStep;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  static const CtxSet   AfMmvdOffsetStepECM3;
#endif
#endif
  #if JVET_AA0061_IBC_MBVD
  static const CtxSet   IbcMbvdFlag;
  static const CtxSet   IbcMbvdMergeIdx;
  static const CtxSet   IbcMbvdStepMvpIdx;
#endif
#if JVET_AC0112_IBC_CIIP
  static const CtxSet   IbcCiipFlag;
  static const CtxSet   IbcCiipIntraIdx;
#endif
#if JVET_AC0112_IBC_GPM
  static const CtxSet   IbcGpmFlag;
  static const CtxSet   IbcGpmIntraFlag;
  static const CtxSet   IbcGpmSplitDirSetFlag;
  static const CtxSet   IbcGpmBldIdx;
#endif
#if JVET_AC0112_IBC_LIC
  static const CtxSet   IbcLicFlag;
#if JVET_AE0078_IBC_LIC_EXTENSION
  static const CtxSet   IbcLicIndex;
#endif
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  static const CtxSet   TMMergeFlag;
#endif
#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
  static const CtxSet   CiipTMMergeFlag;
#endif
#endif
  static const CtxSet   Mvd;
#if JVET_Z0131_IBC_BVD_BINARIZATION
  static const CtxSet   Bvd;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdx;
#endif
#if JVET_AD0140_MVD_PREDICTION && JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdxIBC;
#endif

#if JVET_AD0140_MVD_PREDICTION
  static const CtxSet   MvsdIdxMVDMSB;
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdxBVDMSB;
#endif

#if MULTI_HYP_PRED
  static const CtxSet   MultiHypothesisFlag;
  static const CtxSet   MHRefPic;
  static const CtxSet   MHWeight;
#endif
  static const CtxSet   BDPCMMode;
  static const CtxSet   QtRootCbf;
  static const CtxSet   ACTFlag;
  static const CtxSet   QtCbf           [3];    // [ channel ]
  static const CtxSet   SigCoeffGroup   [2];    // [ ChannelType ]
  static const CtxSet   LastX           [2];    // [ ChannelType ]
  static const CtxSet   LastY           [2];    // [ ChannelType ]
#if JVET_AE0102_LFNST_CTX
  static const CtxSet   SigFlagL[6];    // [ ChannelType + State ]
  static const CtxSet   ParFlagL[2];    // [ ChannelType ]
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  static const CtxSet   GtxFlagL[8];    // [ ChannelType + x ]
#else
  static const CtxSet   GtxFlagL[4];    // [ ChannelType + x ]
#endif
#endif
  static const CtxSet   SigFlag         [6];    // [ ChannelType + State ]
  static const CtxSet   ParFlag         [2];    // [ ChannelType ]
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  static const CtxSet   GtxFlag[8];    // [ ChannelType + x ]
#else
  static const CtxSet   GtxFlag         [4];    // [ ChannelType + x ]
#endif
  static const CtxSet   TsSigCoeffGroup;
  static const CtxSet   TsSigFlag;
  static const CtxSet   TsParFlag;
  static const CtxSet   TsGtxFlag;
  static const CtxSet   TsLrg1Flag;
  static const CtxSet   TsResidualSign;
#if JVET_AG0143_INTER_INTRA
  static const CtxSet   SigCoeffGroupCtxSetSwitch[2];   // [ ChannelType ]
  static const CtxSet   SigFlagCtxSetSwitch[6];         // [ ChannelType + State ]
  static const CtxSet   ParFlagCtxSetSwitch[2];         // [ ChannelType ]
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
  static const CtxSet   GtxFlagCtxSetSwitch[8];    // [ ChannelType + x ]
#else
  static const CtxSet   GtxFlagCtxSetSwitch[4];    // [ ChannelType + x ]
#endif
  static const CtxSet   LastXCtxSetSwitch[2];           // [ ChannelType ]
  static const CtxSet   LastYCtxSetSwitch[2];           // [ ChannelType ]
  static const CtxSet   TsSigCoeffGroupCtxSetSwitch;
  static const CtxSet   TsSigFlagCtxSetSwitch;
  static const CtxSet   TsParFlagCtxSetSwitch;
  static const CtxSet   TsGtxFlagCtxSetSwitch;
  static const CtxSet   TsLrg1FlagCtxSetSwitch;
  static const CtxSet   TsResidualSignCtxSetSwitch;
#endif
  static const CtxSet   MVPIdx;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  static const CtxSet   amFlagState;
#endif
  static const CtxSet   SaoMergeFlag;
  static const CtxSet   SaoTypeIdx;
#if JVET_V0094_BILATERAL_FILTER
  static const CtxSet   BifCtrlFlags[];
#endif
#if JVET_AG0164_AFFINE_GPM
  static const CtxSet   AffineGPMFlag;
#endif
#if JVET_W0066_CCSAO
  static const CtxSet   CcSaoControlIdc;
#endif
  static const CtxSet   TransformSkipFlag;
  static const CtxSet   MTSIdx;
  static const CtxSet   LFNSTIdx;
#if AHG7_LN_TOOLOFF_CFG
  static const CtxSet   VvcLFNSTIdx;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  static const CtxSet   InterLFNSTIdx;
#endif
#if JVET_AI0050_INTER_MTSS
  static const CtxSet   InterLFNSTIntraIdx;
#endif
  static const CtxSet   PLTFlag;
  static const CtxSet   RotationFlag;
  static const CtxSet   RunTypeFlag;
  static const CtxSet   IdxRunModel;
  static const CtxSet   CopyRunModel;
  static const CtxSet   SbtFlag;
  static const CtxSet   SbtQuadFlag;
  static const CtxSet   SbtHorFlag;
  static const CtxSet   SbtPosFlag;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
  static const CtxSet   ImvFlag;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  static const CtxSet   ImvFlagIBC;
#endif
#if JVET_AM0074_INTRA_MERGE
  static const CtxSet   GeneralIntraDerivationFlag;
  static const CtxSet   IntraMergeModeFlag;
#endif
#if ENABLE_DIMD
  static const CtxSet   DimdFlag;
#endif
#if JVET_AL0108_BVG_DIMD
  static const CtxSet   RegularDimdFlag;
#endif
#if JVET_AH0076_OBIC
  static const CtxSet   obicFlag;
#endif
#if JVET_W0123_TIMD_FUSION
  static const CtxSet   TimdFlag;
#if JVET_AJ0061_TIMD_MERGE
  static const CtxSet   TimdMrgFlag;
#endif
#if JVET_AJ0146_TIMDSAD
  static const CtxSet   TimdFlagSad;
#endif
#endif
#if JVET_AK0059_MDIP
  static const CtxSet   MdipFlag;
#endif
#if JVET_AB0155_SGPM
  static const CtxSet   SgpmFlag;
#endif
#if ENABLE_OBMC
  static const CtxSet   ObmcFlag;
#endif 
  static const CtxSet   BcwIdx;
  static const CtxSet   ctbAlfFlag;
  static const CtxSet   ctbAlfAlternative;
  static const CtxSet   AlfUseTemporalFilt;
  static const CtxSet   CcAlfFilterControlFlag;
  static const CtxSet   CiipFlag;
#if JVET_AG0135_AFFINE_CIIP
  static const CtxSet   CiipAffineFlag;
#endif
  static const CtxSet   SmvdFlag;
#if JVET_AG0098_AMVP_WITH_SBTMVP
  static const CtxSet   amvpSbTmvpFlag;
  static const CtxSet   amvpSbTmvpMvdIdx;
#endif
  static const CtxSet   IBCFlag;
#if JVET_AE0169_BIPREDICTIVE_IBC
  static const CtxSet   BiPredIbcFlag;
#endif
  static const CtxSet   ISPMode;
  static const CtxSet   JointCbCrFlag;
#if INTER_LIC
  static const CtxSet   LICFlag;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  static const CtxSet   LicDelta;
#endif
#endif
#if SIGN_PREDICTION
  static const CtxSet   signPred[2];
#endif
#if JVET_Z0050_CCLM_SLOPE
  static const CtxSet   CclmDeltaFlags;
#endif
#if JVET_AA0126_GLM
  static const CtxSet   GlmFlags;
#endif
#if JVET_AA0057_CCCM
  static const CtxSet   CccmFlag;
#endif
#if JVET_AE0100_BVGCCCM
  static const CtxSet   BvgCccmFlag;
#endif
#if JVET_AD0202_CCCM_MDF
  static const CtxSet   CccmMpfFlag;
#endif
#if JVET_AD0120_LBCCP
  static const CtxSet   CcInsideFilterFlag;
#endif
#if JVET_AB0157_TMRL
  static const CtxSet   TmrlDerive;
#if JVET_AJ0081_CHROMA_TMRL
  static const CtxSet   ChromaTmrlFlag;
#endif
#endif
#if JVET_AE0059_INTER_CCCM
  static const CtxSet   InterCccmFlag;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  static const CtxSet   InterCcpMergeFlag;
#endif
#if JVET_AG0058_EIP
  static const CtxSet   EipFlag;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  static const CtxSet   CCPMergeFusionFlag;
  static const CtxSet   CCPMergeFusionType;
#endif


#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  static const CtxSet   InterCcpMergeZeroRootCbfIdc;
#endif
#if JVET_AK0065_TALF
  static const CtxSet   TAlfFilterControlFlag;
#endif
#if JVET_AL0153_ALF_CCCM
  static const CtxSet   LfCccmFlag;
#endif
#if JVET_AL0181_ASBT
  static const CtxSet   AsbtEnabledFlag;
  static const CtxSet   AsbtSecondDecim;
#endif
#if NN_LF_UNIFIED
  static const CtxSet   nnlfUnifiedParams;
#endif
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet   Sao;
  static const CtxSet   Alf;
  static const CtxSet   Palette;
  static const CtxSet   Split;

public:
  static const std::vector<uint8_t>&  getInitTable( unsigned initId );
private:
  static std::vector<std::vector<uint8_t> > sm_InitTables;
  static CtxSet addCtxSet( std::initializer_list<std::initializer_list<uint8_t> > initSet2d);
};



class FracBitsAccess
{
public:
  virtual BinFracBits getFracBitsArray( unsigned ctxId ) const = 0;
};



template <class BinProbModel>
class CtxStore : public FracBitsAccess
{
public:
  CtxStore();
  CtxStore( bool dummy );
  CtxStore( const CtxStore<BinProbModel>& ctxStore );
public:
  void copyFrom(const CtxStore<BinProbModel>& src)
  {
    checkInit();
    std::copy_n(reinterpret_cast<const char*>(src.m_ctx), sizeof(BinProbModel) * ContextSetCfg::NumberOfContexts,
      reinterpret_cast<char*>(m_ctx));
  }
  void copyFrom(const CtxStore<BinProbModel>& src, const CtxSet& ctxSet)
  {
    checkInit();
    std::copy_n(reinterpret_cast<const char*>(src.m_ctx + ctxSet.Offset), sizeof(BinProbModel) * ctxSet.Size,
      reinterpret_cast<char*>(m_ctx + ctxSet.Offset));
  }
  void init(int qp, int initId);
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  void loadWinSizes( const std::vector<uint8_t>&   windows );
  void saveWinSizes( std::vector<uint8_t>&         windows ) const;
  void loadWeights( const std::vector <uint8_t>&  weights );
  void saveWeights( std::vector<uint8_t>&         weights )  const;
  void loadPStates( const std::vector<std::pair<uint16_t, uint16_t>>&  probStates );
  void savePStates( std::vector<std::pair<uint16_t, uint16_t>>&        probStates )  const;
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  void loadRateOffsets( const std::vector<uint8_t>& rateOffsets0, const std::vector<uint8_t>& rateOffsets1 );
  void saveRateOffsets( std::vector<uint8_t>& rateOffsets0, std::vector<uint8_t>& rateOffsets1 ) const;
#endif
#else
  void loadPStates( const std::vector<uint16_t>&  probStates );
  void savePStates( std::vector<uint16_t>&        probStates )  const;
#endif

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  void updateCtxs(BinStoreVector *binStoreElemVector)
  {
    BinStoreVector& vector = *binStoreElemVector;
    
    int numCtxTypes = int( vector.size() );
    
    for ( int ctxType = 0; ctxType < numCtxTypes; ctxType++ )
    {
      const BinStoreElem& store = vector[ ctxType ];

      for ( int i = 0; i < store.count; i++ )
      {
        m_ctxBuffer[ store.ctxId ].updateShortWin( store.bin(i) );
      }
    }
  }
#endif

  const BinProbModel& operator[]      ( unsigned  ctxId  )  const { return m_ctx[ctxId]; }
  BinProbModel&       operator[]      ( unsigned  ctxId  )        { return m_ctx[ctxId]; }
  uint32_t            estFracBits     ( unsigned  bin,
                                        unsigned  ctxId  )  const { return m_ctx[ctxId].estFracBits(bin); }

  BinFracBits         getFracBitsArray( unsigned  ctxId  )  const { return m_ctx[ctxId].getFracBitsArray(); }

private:
  inline void checkInit() { if( m_ctx ) return; m_ctxBuffer.resize( ContextSetCfg::NumberOfContexts ); m_ctx = m_ctxBuffer.data(); }
private:
  std::vector<BinProbModel> m_ctxBuffer;
  BinProbModel*             m_ctx;
};



class Ctx;
class SubCtx
{
  friend class Ctx;
public:
  SubCtx( const CtxSet& ctxSet, const Ctx& ctx ) : m_ctxSet( ctxSet          ), m_ctx( ctx          ) {}
  SubCtx( const SubCtx& subCtx )                 : m_ctxSet( subCtx.m_ctxSet ), m_ctx( subCtx.m_ctx ) {}
  const SubCtx& operator= ( const SubCtx& ) = delete;
private:
  const CtxSet  m_ctxSet;
  const Ctx&    m_ctx;
};



class Ctx : public ContextSetCfg
{
public:
  Ctx();
  Ctx( const BinProbModel_Std*    dummy );
  Ctx( const Ctx&                 ctx   );

public:
  const Ctx& operator= ( const Ctx& ctx )
  {
    m_BPMType = ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std  .copyFrom( ctx.m_ctxStore_Std   );  break;
    default:        break;
    }
    ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
    return *this;
  }

  SubCtx operator= ( SubCtx&& subCtx )
  {
    m_BPMType = subCtx.m_ctx.m_BPMType;
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std  .copyFrom( subCtx.m_ctx.m_ctxStore_Std,   subCtx.m_ctxSet );  break;
    default:        break;
    }
    return std::move(subCtx);
  }

  void  init(int qp, int initId)
  {
    switch (m_BPMType)
    {
    case BPM_Std:   m_ctxStore_Std  .init( qp, initId );  break;
    default:        break;
    }
    for (std::size_t k = 0; k < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS; k++)
    {
      m_GRAdaptStats[k] = 0;
    }
  }

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  void  loadWeights( const std::vector< uint8_t>& weights )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.loadWeights( weights );  break;
    default:        break;
    }
  }

  void  saveWeights( std::vector<uint8_t>& weights ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.saveWeights( weights );  break;
    default:        break;
    }
  }

  void  loadWinSizes( const std::vector<uint8_t>& windows )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.loadWinSizes( windows );  break;
    default:        break;
    }
  }

  void  saveWinSizes( std::vector<uint8_t>& windows ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.saveWinSizes( windows );  break;
    default:        break;
    }
  }

  void  loadPStates( const std::vector<std::pair<uint16_t, uint16_t>>& probStates )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.loadPStates( probStates );  break;
    default:        break;
    }
  }

  void  savePStates( std::vector<std::pair<uint16_t, uint16_t>>& probStates ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.savePStates( probStates );  break;
    default:        break;
    }
  }

#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  void  loadRateOffsets( const std::vector<uint8_t>& rateOffsets0, const std::vector<uint8_t>& rateOffsets1 )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.loadRateOffsets( rateOffsets0, rateOffsets1 );  break;
    default:        break;
    }
  }

  void  saveRateOffsets( std::vector<uint8_t>& rateOffsets0, std::vector<uint8_t>& rateOffsets1 ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std.saveRateOffsets( rateOffsets0, rateOffsets1 );  break;
    default:        break;
    }
}
#endif

#else
  void  loadPStates( const std::vector<uint16_t>& probStates )
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std  .loadPStates( probStates );  break;
    default:        break;
    }
  }

  void  savePStates( std::vector<uint16_t>& probStates ) const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   m_ctxStore_Std  .savePStates( probStates );  break;
    default:        break;
    }
  }
#endif

  void  initCtxAndWinSize( unsigned ctxId, const Ctx& ctx, const uint8_t winSize )
  {
    switch( m_BPMType )
    {
    case BPM_Std:
      m_ctxStore_Std  [ctxId] = ctx.m_ctxStore_Std  [ctxId];
      m_ctxStore_Std  [ctxId] . setLog2WindowSize   (winSize);
      break;
    default:
      break;
    }
  }

  const unsigned&     getGRAdaptStats ( unsigned      id )      const { return m_GRAdaptStats[id]; }
  unsigned&           getGRAdaptStats ( unsigned      id )            { return m_GRAdaptStats[id]; }

public:
  unsigned            getBPMType      ()                        const { return m_BPMType; }
  const Ctx&          getCtx          ()                        const { return *this; }
  Ctx&                getCtx          ()                              { return *this; }

  explicit operator   const CtxStore<BinProbModel_Std>  &()     const { return m_ctxStore_Std; }
  explicit operator         CtxStore<BinProbModel_Std>  &()           { return m_ctxStore_Std; }

  const FracBitsAccess&   getFracBitsAcess()  const
  {
    switch( m_BPMType )
    {
    case BPM_Std:   return m_ctxStore_Std;
    default:        THROW("BPMType out of range");
    }
  }
#if JVET_AG0196_CABAC_RETRAIN
  // direct access to a model prm
  int getRate(int ctxidx) const
  {
    return m_ctxStore_Std[ctxidx].getWinSizes();
  }
#endif
private:
  BPMType                       m_BPMType;
  CtxStore<BinProbModel_Std>    m_ctxStore_Std;
protected:
  unsigned                      m_GRAdaptStats[RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS];
#if ENABLE_SPLIT_PARALLELISM

public:
  int64_t cacheId;
  bool    cacheUsed;
#endif
};



typedef dynamic_cache<Ctx> CtxCache;

class TempCtx
{
  TempCtx( const TempCtx& ) = delete;
  const TempCtx& operator=( const TempCtx& ) = delete;
public:
  TempCtx ( CtxCache* cache )                     : m_ctx( *cache->get() ), m_cache( cache ) {}
  TempCtx ( CtxCache* cache, const Ctx& ctx    )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = ctx; }
  TempCtx ( CtxCache* cache, SubCtx&&   subCtx )  : m_ctx( *cache->get() ), m_cache( cache ) { m_ctx = std::forward<SubCtx>(subCtx); }
  ~TempCtx()                                      { m_cache->cache( &m_ctx ); }
  void operator=( const Ctx& ctx )          { m_ctx = ctx ; }
  void operator=( SubCtx&&   subCtx )       { m_ctx = std::forward<SubCtx>( subCtx ); }
  operator const Ctx& ()           const          { return m_ctx; }
  operator       Ctx& ()                          { return m_ctx; }
private:
  Ctx&      m_ctx;
  CtxCache* m_cache;
};

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
class CtxStateBuf
{
public:
  CtxStateBuf() : m_valid( false ) {}
  ~CtxStateBuf() {}
  inline void reset() { m_valid = false; }
  inline bool getIfValid( Ctx &ctx ) const
  {
    if( m_valid )
    {
      ctx.loadPStates( m_states );
      ctx.loadWeights( m_weights );
      ctx.loadWinSizes( m_rate );
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
      ctx.loadRateOffsets( m_rateOffset[0], m_rateOffset[1] );
#endif
      return true;
    }
    return false;
  }
  inline void store( const Ctx &ctx )
  {
    ctx.savePStates( m_states );
    ctx.saveWeights( m_weights );
    ctx.saveWinSizes( m_rate );
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
    ctx.saveRateOffsets( m_rateOffset[0], m_rateOffset[1] );
#endif
    m_valid = true;
  }

private:
  std::vector<std::pair<uint16_t, uint16_t>> m_states;
  bool                 m_valid;
  std::vector<uint8_t> m_weights;
  std::vector<uint8_t> m_rate;
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
  std::vector<uint8_t> m_rateOffset[2];
#endif
};

class CtxStateArray
{
public:
  CtxStateArray() {}
  ~CtxStateArray() {}

  inline void resetAll()
  {
    for( std::size_t k = 0; k < m_data.size(); k++ )
    {
      m_data[k].reset();
    }
  }

  inline void resize( std::size_t reqSize )
  {
    if( m_data.size() < reqSize )
    {
      m_data.resize( reqSize );
    }
  }

  inline bool getIfValid( Ctx &ctx ) const
  {
    if( m_data.size() )
    {
      return m_data[0].getIfValid( ctx );
    }
    return false;
  }

#if JVET_AG0196_CABAC_RETRAIN
  inline void store( const Ctx &ctx, const SliceType sliceType )
#else
  inline void store( const Ctx &ctx )
#endif
  {
    if( !m_data.size() )
    {
      resize( 1 );
    }
    m_data[0].store( ctx );

#if JVET_AG0196_CABAC_RETRAIN
    m_sliceType = sliceType;
#endif
  }

  inline size_t size() const
  {    
    return  m_data.size();
  }

#if JVET_AG0196_CABAC_RETRAIN
  SliceType getSliceType() const { return m_sliceType; }
#endif

private:
  std::vector<CtxStateBuf> m_data;
#if JVET_AG0196_CABAC_RETRAIN
  SliceType m_sliceType;
#endif
};

class CtxStateStore
{
public:
  CtxStateStore() { static_assert((B_SLICE < NUMBER_OF_SLICE_TYPES - 1) && (P_SLICE < NUMBER_OF_SLICE_TYPES - 1), "index out of bound"); }
  ~CtxStateStore() {}

  void storeCtx( const Slice* slice, const Ctx& ctx )
  {
#if JVET_AH0176_LOW_DELAY_B_CTX
    SliceType s = slice->getSliceType();

    if (s != I_SLICE)
    {
      int t = (s == P_SLICE ? 1 : 0);
#else
    SliceType t = slice->getSliceType();

    if( t != I_SLICE )
    {
#endif
      CtxStateArray* ctxStateArray = nullptr;
      std::pair<int, int> entry( slice->getTLayer(), slice->getSliceQp() );

      if( m_stateBuf[t].find( entry ) != m_stateBuf[t].end() || m_stateBuf[t].size() < TEMP_CABAC_BUFFER_SIZE )
      {
        ctxStateArray = &m_stateBuf[t][entry];
      }
      else
      {
        if( m_stateBuf[t].size() == TEMP_CABAC_BUFFER_SIZE )
        {         
          m_stateBuf[t].erase( m_stateBuf[t].begin() );
        }

        ctxStateArray = &m_stateBuf[t][entry];

        CHECK( m_stateBuf[t].size() > TEMP_CABAC_BUFFER_SIZE, "Wrong buffer size" );
      }

#if JVET_AG0196_CABAC_RETRAIN
      ctxStateArray->store( ctx, slice->getCabacInitSliceType() );
#else
      ctxStateArray->store( ctx );
#endif
    }
  }

#if JVET_AG0196_CABAC_RETRAIN
  bool loadCtx( Slice* slice, Ctx& ctx )
#else
  bool loadCtx( const Slice* slice, Ctx& ctx )
#endif
  {
#if JVET_AH0176_LOW_DELAY_B_CTX
    SliceType s = slice->getSliceType();

    if (s != I_SLICE)
    {
      int t = (s == P_SLICE ? 1 : 0);
#else
    SliceType t = slice->getSliceType();

    if( t != I_SLICE )
    {
#endif
      std::pair<int, int> entry( slice->getTLayer(), slice->getSliceQp() );

      if( m_stateBuf[t].find( entry ) != m_stateBuf[t].end() )
      {
        const CtxStateArray& ctxStateArray = m_stateBuf[t][entry];
#if JVET_AG0196_CABAC_RETRAIN
        slice->setCabacInitSliceType( m_stateBuf[t][entry].getSliceType() );
#endif
        return ctxStateArray.getIfValid( ctx );
      }      
    }
    return false;
  }

  void clearValid()
  {
    m_stateBuf[0].clear();
    m_stateBuf[1].clear();
  }

private:
  std::map<std::pair<int, int>, CtxStateArray> m_stateBuf[2];
};

class CABACDataStore
{
public:

#if JVET_AG0196_CABAC_RETRAIN
  bool loadCtxStates( Slice* slice, Ctx& ctx ) { return m_ctxStateStore.loadCtx( slice, ctx ); }
#else
  bool loadCtxStates( const Slice* slice, Ctx& ctx ) { return m_ctxStateStore.loadCtx( slice, ctx ); }
#endif
  void storeCtxStates( const Slice* slice, const Ctx& ctx ) { m_ctxStateStore.storeCtx( slice, ctx ); }

  void updateBufferState( const Slice* slice )
  {
#if JVET_Z0118_GDR && JVET_AD0206_CABAC_INIT_AT_GDR
    if( slice->getPendingRasInit() || slice->isInterGDR() )
#else
    if( slice->getPendingRasInit() )
#endif
    {
      m_ctxStateStore.clearValid();
    }
  }
private:
  CtxStateStore       m_ctxStateStore;
};
#endif

#endif
