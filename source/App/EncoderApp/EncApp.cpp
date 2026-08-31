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

/** \file     EncApp.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <iomanip>

#include "EncApp.h"
#include "EncoderLib/AnnexBwrite.h"
#include "EncoderLib/EncLibCommon.h"

using namespace std;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncApp::EncApp( fstream& bitStream, EncLibCommon* encLibCommon )
  : m_cEncLib( encLibCommon )
  , m_bitstream( bitStream )
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif
  m_numEncoded = 0;
  m_flush = false;
}

EncApp::~EncApp()
{
}

void EncApp::xInitLibCfg()
{
  VPS& vps = *m_cEncLib.getVPS();
  vps.m_targetOlsIdx = m_targetOlsIdx;

  vps.setMaxLayers( m_maxLayers );

  if (vps.getMaxLayers() > 1)
  {
    vps.setVPSId(1);  //JVET_P0205 vps_video_parameter_set_id shall be greater than 0 for multi-layer coding
  }
  else
  {
    vps.setVPSId(0);
    vps.setEachLayerIsAnOlsFlag(1); // If vps_max_layers_minus1 is equal to 0,
                                    // the value of each_layer_is_an_ols_flag is inferred to be equal to 1.
                                    // Otherwise, when vps_all_independent_layers_flag is equal to 0,
                                    // the value of each_layer_is_an_ols_flag is inferred to be equal to 0.
  }
  vps.setMaxSubLayers(m_maxSublayers);
  if (vps.getMaxLayers() > 1 && vps.getMaxSubLayers() > 1)
  {
    vps.setAllLayersSameNumSublayersFlag(m_allLayersSameNumSublayersFlag);
  }
  if (vps.getMaxLayers() > 1)
  {
    vps.setAllIndependentLayersFlag(m_allIndependentLayersFlag);
    if (!vps.getAllIndependentLayersFlag())
    {
      vps.setEachLayerIsAnOlsFlag(0);
      for (int i = 0; i < m_maxTempLayer; i++)
      {
        vps.setPredDirection(i, 0);
      }
      for (int i = 0; i < m_predDirectionArray.size(); i++)
      {
        if (m_predDirectionArray[i] != ' ')
        {
          vps.setPredDirection(i >> 1, int(m_predDirectionArray[i] - 48));
        }
      }
    }
  }

  for (int i = 0; i < vps.getMaxLayers(); i++)
  {
    vps.setGeneralLayerIdx( m_layerId[i], i );
    vps.setLayerId(i, m_layerId[i]);

    if (i > 0 && !vps.getAllIndependentLayersFlag())
    {
      vps.setIndependentLayerFlag( i, m_numRefLayers[i] ? false : true );

      if (!vps.getIndependentLayerFlag(i))
      {
        for (int j = 0, k = 0; j < i; j++)
        {
          if (m_refLayerIdxStr[i].find(to_string(j)) != std::string::npos)
          {
            vps.setDirectRefLayerFlag(i, j, true);
            vps.setInterLayerRefIdc( i, j, k );
            vps.setDirectRefLayerIdx(i, k++, j);
          }
          else
          {
            vps.setDirectRefLayerFlag(i, j, false);
          }
        }
      }
    }
  }


  if (vps.getMaxLayers() > 1)
  {
    if (vps.getAllIndependentLayersFlag())
    {
      vps.setEachLayerIsAnOlsFlag(m_eachLayerIsAnOlsFlag);
      if (vps.getEachLayerIsAnOlsFlag() == 0)
      {
        vps.setOlsModeIdc(2); // When vps_all_independent_layers_flag is equal to 1 and each_layer_is_an_ols_flag is equal to 0, the value of ols_mode_idc is inferred to be equal to 2
      }
    }
    if (!vps.getEachLayerIsAnOlsFlag())
    {
      if (!vps.getAllIndependentLayersFlag())
      {
        vps.setOlsModeIdc(m_olsModeIdc);
      }
      if (vps.getOlsModeIdc() == 2)
      {
        vps.setNumOutputLayerSets(m_numOutputLayerSets);
        for (int i = 1; i < vps.getNumOutputLayerSets(); i++)
        {
          for (int j = 0; j < vps.getMaxLayers(); j++)
          {
            if (m_olsOutputLayerStr[i].find(to_string(j)) != std::string::npos)
            {
              vps.setOlsOutputLayerFlag(i, j, 1);
            }
            else
            {
              vps.setOlsOutputLayerFlag(i, j, 0);
            }
          }
        }
      }
    }
  }
  CHECK( m_numPtlsInVps == 0, "There has to be at least one PTL structure in the VPS." );
  vps.setNumPtls                                                 ( m_numPtlsInVps );
  vps.setPtPresentFlag                                           (0, 1);
  for (int i = 0; i < vps.getNumPtls(); i++)
  {
    if( i > 0 )
      vps.setPtPresentFlag                                         (i, 0);
    vps.setPtlMaxTemporalId                                      (i, vps.getMaxSubLayers() - 1);
  }
  for (int i = 0; i < vps.getNumOutputLayerSets(); i++)
  {
    vps.setOlsPtlIdx                                             (i, m_olsPtlIdx[i]);
  }
  std::vector<ProfileTierLevel> ptls;
  ptls.resize(vps.getNumPtls());
  // PTL0 shall be the same as the one signalled in the SPS
  ptls[0].setLevelIdc                                            ( m_level );
  ptls[0].setProfileIdc                                          ( m_profile);
  ptls[0].setTierFlag                                            ( m_levelTier );
#if JVET_S0138_GCI_PTL
  ptls[0].setFrameOnlyConstraintFlag                             ( m_frameOnlyConstraintFlag);
  ptls[0].setMultiLayerEnabledFlag                               ( m_multiLayerEnabledFlag);
#if JVET_S_PROFILES
  CHECK((m_profile == Profile::MAIN_10 || m_profile == Profile::MAIN_10_444
         || m_profile == Profile::MAIN_10_STILL_PICTURE || m_profile == Profile::MAIN_10_444_STILL_PICTURE)
          && m_multiLayerEnabledFlag,
        "ptl_multilayer_enabled_flag shall be equal to 0 for non-multilayer profiles");
#else
  CHECK( (m_profile == Profile::MAIN_10 || m_profile == Profile::MAIN_444_10) && m_multiLayerEnabledFlag, "ptl_multilayer_enabled_flag shall be equal to 0 for Main 10 and Main 10 4:4:4 profiles");
#endif
#endif
  ptls[0].setNumSubProfile                                       ( m_numSubProfile );
  for (int i = 0; i < m_numSubProfile; i++)
  {
    ptls[0].setSubProfileIdc                                   (i, m_subProfile[i]);
  }
  for(int i = 1; i < vps.getNumPtls(); i++)
  {
    ptls[i].setLevelIdc                                          (m_levelPtl[i]);
  }
  vps.setProfileTierLevel(ptls);
  vps.setVPSExtensionFlag                                        ( false );
  m_cEncLib.setProfile                                           ( m_profile);
  m_cEncLib.setLevel                                             ( m_levelTier, m_level);
#if JVET_S0138_GCI_PTL
  m_cEncLib.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cEncLib.setMultiLayerEnabledFlag                             ( m_multiLayerEnabledFlag || m_maxLayers > 1);
#endif
  m_cEncLib.setNumSubProfile                                     ( m_numSubProfile );
  for (int i = 0; i < m_numSubProfile; i++)
  {
    m_cEncLib.setSubProfile(i, m_subProfile[i]);
  }

  m_cEncLib.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cEncLib.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cEncLib.setPrintHexPsnr(m_printHexPsnr);
  m_cEncLib.setPrintSequenceMSE                                  ( m_printSequenceMSE);
#if MSSIM_UNIFORM_METRICS_LOG
  m_cEncLib.setPrintMSSSIM                                       ( m_printMSSSIM );
#endif
  m_cEncLib.setCabacZeroWordPaddingEnabled                       ( m_cabacZeroWordPaddingEnabled );

  m_cEncLib.setFrameRate                                         ( m_iFrameRate );
  m_cEncLib.setFrameSkip                                         ( m_FrameSkip );
  m_cEncLib.setTemporalSubsampleRatio                            ( m_temporalSubsampleRatio );
#if JVET_AA0146_WRAP_AROUND_FIX
  m_cEncLib.setSourceWidth                                       ( m_sourceWidth );
  m_cEncLib.setSourceHeight                                      ( m_sourceHeight );
#else
  m_cEncLib.setSourceWidth                                       ( m_iSourceWidth );
  m_cEncLib.setSourceHeight                                      ( m_iSourceHeight );
#endif
  m_cEncLib.setConformanceWindow                                 ( m_confWinLeft / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinRight / SPS::getWinUnitX( m_InputChromaFormatIDC ), m_confWinTop / SPS::getWinUnitY( m_InputChromaFormatIDC ), m_confWinBottom / SPS::getWinUnitY( m_InputChromaFormatIDC ) );
  m_cEncLib.setScalingRatio                                      ( m_scalingRatioHor, m_scalingRatioVer );
#if JVET_Q0114_ASPECT5_GCI_FLAG
  m_cEncLib.setRprEnabled                                        (m_rprEnabledFlag);
#endif
  m_cEncLib.setResChangeInClvsEnabled                            ( m_resChangeInClvsEnabled );
  m_cEncLib.setSwitchPocPeriod                                   ( m_switchPocPeriod );
  m_cEncLib.setUpscaledOutput                                    ( m_upscaledOutput );
#if JVET_AC0096
  m_cEncLib.setRprFunctionalityTestingEnabledFlag                (m_rprFunctionalityTestingEnabledFlag);
  m_cEncLib.setRprSwitchingSegmentSize                           (m_rprSwitchingSegmentSize);
  m_cEncLib.setRprPopulatePPSatIntraFlag                         (m_rprPopulatePPSatIntraFlag);
  m_cEncLib.setScalingRatio2                                     (m_scalingRatioHor2, m_scalingRatioVer2);
  m_cEncLib.setScalingRatio3                                     (m_scalingRatioHor3, m_scalingRatioVer3);
#endif
#if JVET_AG0116
  m_cEncLib.setGOPBasedRPREnabledFlag                            (m_gopBasedRPREnabledFlag);
  m_cEncLib.setGOPBasedRPRQPThreshold                            (m_gopBasedRPRQPThreshold);
  m_cEncLib.setPsnrThresholdRPR                                  (m_psnrThresholdRPR, m_psnrThresholdRPR2, m_psnrThresholdRPR3);
  m_cEncLib.setQpOffsetRPR                                       (m_qpOffsetRPR, m_qpOffsetRPR2, m_qpOffsetRPR3);
  m_cEncLib.setQpOffsetChromaRPR                                 (m_qpOffsetChromaRPR, m_qpOffsetChromaRPR2, m_qpOffsetChromaRPR3);
#endif
#if JVET_AB0082
  m_cEncLib.setUpscaleFilerForDisplay                            (m_upscaleFilterForDisplay);
#endif
  m_cEncLib.setFramesToBeEncoded                                 ( m_framesToBeEncoded );
  m_cEncLib.setValidFrames                                       ( m_firstValidFrame, m_lastValidFrame );
  m_cEncLib.setAvoidIntraInDepLayer                              ( m_avoidIntraInDepLayer );

  //====== SPS constraint flags =======
#if JVET_S0179_CONDITIONAL_SIGNAL_GCI
  m_cEncLib.setGciPresentFlag                                    ( m_gciPresentFlag );
#endif
  if (m_cEncLib.getGciPresentFlag())
  {
    m_cEncLib.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
    m_cEncLib.setNonProjectedConstraintFlag(m_nonProjectedConstraintFlag);
    m_cEncLib.setOneTilePerPicConstraintFlag(m_oneTilePerPicConstraintFlag);
    m_cEncLib.setPicHeaderInSliceHeaderConstraintFlag(m_picHeaderInSliceHeaderConstraintFlag);
    m_cEncLib.setOneSlicePerPicConstraintFlag(m_oneSlicePerPicConstraintFlag);
#if JVET_S0113_S0195_GCI
    m_cEncLib.setNoIdrRplConstraintFlag(m_noIdrRplConstraintFlag);
    CHECK(m_noIdrRplConstraintFlag&& m_idrRefParamList, "IDR RPL shall be deactivated when gci_no_idr_rpl_constraint_flag equal to 1");

    m_cEncLib.setNoRectSliceConstraintFlag(m_noRectSliceConstraintFlag);
    CHECK(m_noRectSliceConstraintFlag && !m_rasterSliceFlag, "Rectangular slice shall be deactivated when gci_no_rectangular_slice_constraint_flag equal to 1");

    m_cEncLib.setOneSlicePerSubpicConstraintFlag(m_oneSlicePerSubpicConstraintFlag);
    CHECK(m_oneSlicePerSubpicConstraintFlag && !m_singleSlicePerSubPicFlag, "Each picture shall consist of one and only one rectangular slice when gci_one_slice_per_subpic_constraint_flag equal to 1");

    m_cEncLib.setNoSubpicInfoConstraintFlag(m_noSubpicInfoConstraintFlag);
    CHECK(m_noSubpicInfoConstraintFlag&& m_subPicInfoPresentFlag, "Subpicture information shall not present when gci_no_subpic_info_constraint_flag equal to 1");
#else
    m_cEncLib.setOneSubpicPerPicConstraintFlag(m_oneSubpicPerPicConstraintFlag);
#endif
#if !JVET_S0138_GCI_PTL
    m_cEncLib.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);
#endif
    m_cEncLib.setOnePictureOnlyConstraintFlag(m_onePictureOnlyConstraintFlag);
    m_cEncLib.setIntraOnlyConstraintFlag(m_intraOnlyConstraintFlag);
    m_cEncLib.setNoIdrConstraintFlag(m_noIdrConstraintFlag);
    m_cEncLib.setNoGdrConstraintFlag(m_noGdrConstraintFlag);
#if !JVET_S0138_GCI_PTL
    m_cEncLib.setSingleLayerConstraintFlag(m_singleLayerConstraintFlag);
#endif
    m_cEncLib.setAllLayersIndependentConstraintFlag(m_allLayersIndependentConstraintFlag);
    m_cEncLib.setNoQpDeltaConstraintFlag(m_noQpDeltaConstraintFlag);

    m_cEncLib.setNoTrailConstraintFlag(m_noTrailConstraintFlag);
    CHECK(m_noTrailConstraintFlag && m_iIntraPeriod != 1, "TRAIL shall be deactivated when m_noTrailConstraintFlag is equal to 1");

    m_cEncLib.setNoStsaConstraintFlag(m_noStsaConstraintFlag);
    CHECK(m_noStsaConstraintFlag && (m_iIntraPeriod != 1 || xHasNonZeroTemporalID()), "STSA shall be deactivated when m_noStsaConstraintFlag is equal to 1");

    m_cEncLib.setNoRaslConstraintFlag(m_noRaslConstraintFlag);
    CHECK(m_noRaslConstraintFlag && (m_iIntraPeriod != 1 || xHasLeadingPicture()), "RASL shall be deactivated when m_noRaslConstraintFlag is equal to 1");

    m_cEncLib.setNoRadlConstraintFlag(m_noRadlConstraintFlag);
    CHECK(m_noRadlConstraintFlag && (m_iIntraPeriod != 1 || xHasLeadingPicture()), "RADL shall be deactivated when m_noRadlConstraintFlag is equal to 1");

    m_cEncLib.setNoCraConstraintFlag(m_noCraConstraintFlag);
    CHECK(m_noCraConstraintFlag && (m_iDecodingRefreshType == 1), "CRA shall be deactivated when m_noCraConstraintFlag is equal to 1");

#if JVET_Q0114_ASPECT5_GCI_FLAG
    m_cEncLib.setNoRprConstraintFlag(m_noRprConstraintFlag);
    CHECK(m_noRprConstraintFlag && m_rprEnabledFlag, "Reference picture resampling shall be deactivated when m_noRprConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoResChangeInClvsConstraintFlag(m_noResChangeInClvsConstraintFlag);
    CHECK(m_noResChangeInClvsConstraintFlag && m_resChangeInClvsEnabled, "Resolution change in CLVS shall be deactivated when m_noResChangeInClvsConstraintFlag is equal to 1");

    m_cEncLib.setMaxBitDepthConstraintIdc(m_maxBitDepthConstraintIdc);
    CHECK(m_internalBitDepth[CHANNEL_TYPE_LUMA] > m_maxBitDepthConstraintIdc, "Internal bit depth shall be less than or equal to m_maxBitDepthConstraintIdc");

    m_cEncLib.setMaxChromaFormatConstraintIdc(m_maxChromaFormatConstraintIdc);
    CHECK(m_chromaFormatIDC > m_maxChromaFormatConstraintIdc, "Chroma format Idc shall be less than or equal to m_maxBitDepthConstraintIdc");

#if JVET_S0058_GCI
    m_cEncLib.setNoMttConstraintFlag(m_noMttConstraintFlag);
    CHECK(m_noMttConstraintFlag && (m_uiMaxMTTHierarchyDepth || m_uiMaxMTTHierarchyDepthI || m_uiMaxMTTHierarchyDepthIChroma), "Mtt shall be deactivated when m_bNoMttConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoQtbttDualTreeIntraConstraintFlag(m_noQtbttDualTreeIntraConstraintFlag);
    CHECK(m_noQtbttDualTreeIntraConstraintFlag && m_dualTree, "Dual tree shall be deactivated when m_bNoQtbttDualTreeIntraConstraintFlag is equal to 1");

#if JVET_S0066_GCI
    m_cEncLib.setMaxLog2CtuSizeConstraintIdc(m_maxLog2CtuSizeConstraintIdc);
    CHECK( m_uiCTUSize > (1<<(m_maxLog2CtuSizeConstraintIdc)), "CTUSize shall be less than or equal to 1 << m_maxLog2CtuSize");

#endif
    m_cEncLib.setNoPartitionConstraintsOverrideConstraintFlag(m_noPartitionConstraintsOverrideConstraintFlag);
    CHECK(m_noPartitionConstraintsOverrideConstraintFlag && m_SplitConsOverrideEnabledFlag, "Partition override shall be deactivated when m_noPartitionConstraintsOverrideConstraintFlag is equal to 1");

    m_cEncLib.setNoSaoConstraintFlag(m_noSaoConstraintFlag);
    CHECK(m_noSaoConstraintFlag && m_bUseSAO, "SAO shall be deactivated when m_bNoSaoConstraintFlag is equal to 1");

#if JVET_W0066_CCSAO
    m_cEncLib.setNoCCSaoConstraintFlag(m_noCCSaoConstraintFlag);
    CHECK(m_noCCSaoConstraintFlag && m_CCSAO, "CCSAO shall be deactivated when m_noCCSaoConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoAlfConstraintFlag(m_noAlfConstraintFlag);
    CHECK(m_noAlfConstraintFlag && m_alf, "ALF shall be deactivated when m_bNoAlfConstraintFlag is equal to 1");

    m_cEncLib.setNoCCAlfConstraintFlag(m_noCCAlfConstraintFlag);
    CHECK(m_noCCAlfConstraintFlag && m_ccalf, "CCALF shall be deactivated when m_noCCAlfConstraintFlag is equal to 1");

#if JVET_S0058_GCI
    m_cEncLib.setNoWeightedPredictionConstraintFlag(m_noWeightedPredictionConstraintFlag);
    CHECK(m_noWeightedPredictionConstraintFlag && (m_useWeightedPred || m_useWeightedBiPred), "Weighted Prediction shall be deactivated when m_bNoWeightedPredictionConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoRefWraparoundConstraintFlag(m_noRefWraparoundConstraintFlag);
    CHECK(m_noRefWraparoundConstraintFlag && m_wrapAround, "Wrap around shall be deactivated when m_bNoRefWraparoundConstraintFlag is equal to 1");

    m_cEncLib.setNoTemporalMvpConstraintFlag(m_noTemporalMvpConstraintFlag);
    CHECK(m_noTemporalMvpConstraintFlag && m_TMVPModeId, "Temporal MVP shall be deactivated when m_bNoTemporalMvpConstraintFlag is equal to 1");

    m_cEncLib.setNoSbtmvpConstraintFlag(m_noSbtmvpConstraintFlag);
    CHECK(m_noSbtmvpConstraintFlag && m_sbTmvpEnableFlag,
          "SbTMVP shall be deactivated when m_bNoSbtmvpConstraintFlag is equal to 1");

    m_cEncLib.setNoAmvrConstraintFlag(m_noAmvrConstraintFlag);
    CHECK(m_noAmvrConstraintFlag && (m_ImvMode != IMV_OFF || m_AffineAmvr), "AMVR shall be deactivated when m_bNoAmvrConstraintFlag is equal to 1");

    m_cEncLib.setNoBdofConstraintFlag(m_noBdofConstraintFlag);
    CHECK(m_noBdofConstraintFlag && m_BIO, "BIO shall be deactivated when m_bNoBdofConstraintFlag is equal to 1");

    m_cEncLib.setNoDmvrConstraintFlag(m_noDmvrConstraintFlag);
    CHECK(m_noDmvrConstraintFlag && m_DMVR, "DMVR shall be deactivated when m_noDmvrConstraintFlag is equal to 1");

    m_cEncLib.setNoCclmConstraintFlag(m_noCclmConstraintFlag);
    CHECK(m_noCclmConstraintFlag && m_LMChroma, "CCLM shall be deactivated when m_bNoCclmConstraintFlag is equal to 1");

    m_cEncLib.setNoMtsConstraintFlag(m_noMtsConstraintFlag);
    CHECK(m_noMtsConstraintFlag && (m_MTS || m_MTSImplicit), "MTS shall be deactivated when m_bNoMtsConstraintFlag is equal to 1");

    m_cEncLib.setNoSbtConstraintFlag(m_noSbtConstraintFlag);
    CHECK(m_noSbtConstraintFlag && m_SBT, "SBT shall be deactivated when mm_noSbtConstraintFlag_nonPackedConstraintFlag is equal to 1");

    m_cEncLib.setNoAffineMotionConstraintFlag(m_noAffineMotionConstraintFlag);
    CHECK(m_noAffineMotionConstraintFlag && m_Affine, "Affine shall be deactivated when m_bNoAffineMotionConstraintFlag is equal to 1");

    m_cEncLib.setNoBcwConstraintFlag(m_noBcwConstraintFlag);
    CHECK(m_noBcwConstraintFlag && m_bcw, "BCW shall be deactivated when m_bNoBcwConstraintFlag is equal to 1");

    m_cEncLib.setNoIbcConstraintFlag(m_noIbcConstraintFlag);
    CHECK(m_noIbcConstraintFlag && m_IBCMode, "IBC shall be deactivated when m_noIbcConstraintFlag is equal to 1");

    m_cEncLib.setNoCiipConstraintFlag(m_noCiipConstraintFlag);
    CHECK(m_noCiipConstraintFlag && m_ciip, "CIIP shall be deactivated when m_bNoCiipConstraintFlag is equal to 1");

    m_cEncLib.setNoGeoConstraintFlag(m_noGeoConstraintFlag);
    CHECK(m_noGeoConstraintFlag && m_Geo, "GEO shall be deactivated when m_noGeoConstraintFlag is equal to 1");
#if ENABLE_DIMD
    m_cEncLib.setNoDimdConstraintFlag(m_noDimdConstraintFlag);
    CHECK(m_noDimdConstraintFlag && m_dimd, "DIMD shall be deactivated when m_noDimdConstraintFlag is equal to 1");
#endif
#if JVET_W0123_TIMD_FUSION
    m_cEncLib.setNoTimdConstraintFlag(m_noTimdConstraintFlag);
    CHECK(m_noTimdConstraintFlag && m_timd, "TIMD shall be deactivated when m_noTimdConstraintFlag is equal to 1");
#endif
#if JVET_AB0155_SGPM
    m_cEncLib.setNoSgpmConstraintFlag(m_noSgpmConstraintFlag);
    CHECK(m_noSgpmConstraintFlag && m_sgpm, "SGPM shall be deactivated when m_noSgpmConstraintFlag is equal to 1");
#endif
#if JVET_AD0082_TMRL_CONFIG
    m_cEncLib.setNoTmrlConstraintFlag(m_noTmrlConstraintFlag);
    CHECK(m_noTmrlConstraintFlag && m_tmrl, "TMRL shall be deactivated when m_noTmrlConstraintFlag is equal to 1");
#endif
#if JVET_AG0058_EIP
    m_cEncLib.setNoEipConstraintFlag(m_noEipConstraintFlag);
    CHECK(m_noEipConstraintFlag && m_eip, "EIP shall be deactivated when m_noTmrlConstraintFlag is equal to 1");
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
    m_cEncLib.setNoIntraPredBfConstraintFlag(m_noIntraPredBfConstraintFlag);
    CHECK(m_noIntraPredBfConstraintFlag && m_intraPredBf, "Intra Pred Bf shall be deactivated when m_noIntraPredBfConstraintFlag is equal to 1");
#endif
#if ENABLE_OBMC
    m_cEncLib.setNoObmcConstraintFlag(m_noObmcConstraintFlag);
    CHECK(m_noObmcConstraintFlag && m_OBMC, "OBMC shall be deactivated when m_noObmcConstraintFlag is equal to 1");
#endif
    m_cEncLib.setNoLadfConstraintFlag(m_noLadfConstraintFlag);
    CHECK(m_noLadfConstraintFlag && m_LadfEnabed, "LADF shall be deactivated when m_bNoLadfConstraintFlag is equal to 1");

    m_cEncLib.setNoTransformSkipConstraintFlag(m_noTransformSkipConstraintFlag);
    CHECK(m_noTransformSkipConstraintFlag && m_useTransformSkip, "Transform skip shall be deactivated when m_noTransformSkipConstraintFlag is equal to 1");

#if JVET_S0066_GCI
    m_cEncLib.setNoLumaTransformSize64ConstraintFlag(m_noLumaTransformSize64ConstraintFlag);
    CHECK(m_noLumaTransformSize64ConstraintFlag && m_log2MaxTbSize > 5, "Max transform size shall be less than 64 when m_noLumaTransformSize64ConstraintFlag is equal to 1");

#endif
    m_cEncLib.setNoBDPCMConstraintFlag(m_noBDPCMConstraintFlag);
    CHECK(m_noBDPCMConstraintFlag && m_useBDPCM, "BDPCM shall be deactivated when m_noBDPCMConstraintFlag is equal to 1");

    m_cEncLib.setNoJointCbCrConstraintFlag(m_noJointCbCrConstraintFlag);
    CHECK(m_noJointCbCrConstraintFlag && m_JointCbCrMode, "JCCR shall be deactivated when m_noJointCbCrConstraintFlag is equal to 1");

    m_cEncLib.setNoDepQuantConstraintFlag(m_noDepQuantConstraintFlag);
#if TCQ_8STATES
    CHECK( m_noDepQuantConstraintFlag && m_depQuantEnabledIdc, "DQ shall be deactivated when m_bNoDepQuantConstraintFlag is equal to 1" );
#else
    CHECK(m_noDepQuantConstraintFlag && m_depQuantEnabledFlag, "DQ shall be deactivated when m_bNoDepQuantConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoSignDataHidingConstraintFlag(m_noSignDataHidingConstraintFlag);
    CHECK(m_noSignDataHidingConstraintFlag && m_signDataHidingEnabledFlag, "SDH shall be deactivated when m_bNoSignDataHidingConstraintFlag is equal to 1");

    m_cEncLib.setNoApsConstraintFlag(m_noApsConstraintFlag);
    CHECK(m_noApsConstraintFlag && (m_lmcsEnabled || (m_useScalingListId != SCALING_LIST_OFF)), "LMCS and explict scaling list shall be deactivated when m_noApsConstraintFlag is equal to 1");

    m_cEncLib.setNoMrlConstraintFlag(m_noMrlConstraintFlag);
    CHECK(m_noMrlConstraintFlag && m_MRL, "MRL shall be deactivated when m_noMrlConstraintFlag is equal to 1");

    m_cEncLib.setNoIspConstraintFlag(m_noIspConstraintFlag);
    CHECK(m_noIspConstraintFlag && m_ISP, "ISP shall be deactivated when m_noIspConstraintFlag is equal to 1");

    m_cEncLib.setNoMipConstraintFlag(m_noMipConstraintFlag);
    CHECK(m_noMipConstraintFlag && m_MIP, "MIP shall be deactivated when m_noMipConstraintFlag is equal to 1");

    m_cEncLib.setNoLfnstConstraintFlag(m_noLfnstConstraintFlag);
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
    bool spsLfnstEnabled = ( m_intraLFNSTISlice || m_intraLFNSTPBSlice );
    spsLfnstEnabled |= m_interLFNST;
    CHECK( m_noLfnstConstraintFlag && spsLfnstEnabled, "LFNST shall be deactivated when m_noLfnstConstraintFlag is equal to 1" );
#else
    CHECK(m_noLfnstConstraintFlag && m_LFNST, "LFNST shall be deactivated when m_noLfnstConstraintFlag is equal to 1");
#endif

    m_cEncLib.setNoMmvdConstraintFlag(m_noMmvdConstraintFlag);
    CHECK(m_noMmvdConstraintFlag && m_MMVD, "MMVD shall be deactivated when m_noMmvdConstraintFlag is equal to 1");

    m_cEncLib.setNoSmvdConstraintFlag(m_noSmvdConstraintFlag);
    CHECK(m_noSmvdConstraintFlag && m_SMVD, "SMVD shall be deactivated when m_noSmvdConstraintFlag is equal to 1");

    m_cEncLib.setNoProfConstraintFlag(m_noProfConstraintFlag);
    CHECK(m_noProfConstraintFlag && m_PROF, "PROF shall be deactivated when m_noProfConstraintFlag is equal to 1");

    m_cEncLib.setNoPaletteConstraintFlag(m_noPaletteConstraintFlag);
    CHECK(m_noPaletteConstraintFlag && m_PLTMode, "Palette shall be deactivated when m_noPaletteConstraintFlag is equal to 1");

    m_cEncLib.setNoActConstraintFlag(m_noActConstraintFlag);
    CHECK(m_noActConstraintFlag && m_useColorTrans, "ACT shall be deactivated when m_noActConstraintFlag is equal to 1");

    m_cEncLib.setNoLmcsConstraintFlag(m_noLmcsConstraintFlag);
    CHECK(m_noLmcsConstraintFlag && m_lmcsEnabled, "LMCS shall be deactivated when m_noLmcsConstraintFlag is equal to 1");

#if JVET_S0050_GCI
    m_cEncLib.setNoExplicitScaleListConstraintFlag(m_noExplicitScaleListConstraintFlag);
    CHECK(m_noExplicitScaleListConstraintFlag && m_useScalingListId != SCALING_LIST_OFF, "Explicit scaling list shall be deactivated when m_noExplicitScaleListConstraintFlag is equal to 1");

    m_cEncLib.setNoVirtualBoundaryConstraintFlag(m_noVirtualBoundaryConstraintFlag);
    CHECK(m_noVirtualBoundaryConstraintFlag && m_virtualBoundariesEnabledFlag, "Virtuall boundaries shall be deactivated when m_noVirtualBoundaryConstraintFlag is equal to 1");
#endif
#if JVET_R0341_GCI
    m_cEncLib.setNoChromaQpOffsetConstraintFlag(m_noChromaQpOffsetConstraintFlag);
    CHECK(m_noChromaQpOffsetConstraintFlag && m_cuChromaQpOffsetSubdiv, "Chroma Qp offset shall be 0 when m_noChromaQpOffsetConstraintFlag is equal to 1");
#endif
  }
  else
  {
    m_cEncLib.setNonPackedConstraintFlag(false);
    m_cEncLib.setNonProjectedConstraintFlag(false);
#if !JVET_S0138_GCI_PTL
    m_cEncLib.setSingleLayerConstraintFlag(false);
#endif
    m_cEncLib.setAllLayersIndependentConstraintFlag(false);
    m_cEncLib.setNoResChangeInClvsConstraintFlag(false);
    m_cEncLib.setOneTilePerPicConstraintFlag(false);
    m_cEncLib.setPicHeaderInSliceHeaderConstraintFlag(false);
    m_cEncLib.setOneSlicePerPicConstraintFlag(false);
#if JVET_S0113_S0195_GCI
    m_cEncLib.setNoIdrRplConstraintFlag(false);
    m_cEncLib.setNoRectSliceConstraintFlag(false);
    m_cEncLib.setOneSlicePerSubpicConstraintFlag(false);
    m_cEncLib.setNoSubpicInfoConstraintFlag(false);
#else
    m_cEncLib.setOneSubpicPerPicConstraintFlag(false);
#endif
#if !JVET_S0138_GCI_PTL
    m_cEncLib.setFrameOnlyConstraintFlag(false);
#endif
    m_cEncLib.setOnePictureOnlyConstraintFlag(false);
    m_cEncLib.setIntraOnlyConstraintFlag(false);
    m_cEncLib.setMaxBitDepthConstraintIdc(16);
    m_cEncLib.setMaxChromaFormatConstraintIdc(3);
#if JVET_S0058_GCI
    m_cEncLib.setNoMttConstraintFlag(false);
#endif
    m_cEncLib.setNoQtbttDualTreeIntraConstraintFlag(false);
    m_cEncLib.setNoPartitionConstraintsOverrideConstraintFlag(false);
    m_cEncLib.setNoSaoConstraintFlag(false);
#if JVET_W0066_CCSAO
    m_cEncLib.setNoCCSaoConstraintFlag(false);
#endif
    m_cEncLib.setNoAlfConstraintFlag(false);
    m_cEncLib.setNoCCAlfConstraintFlag(false);
#if JVET_S0058_GCI
    m_cEncLib.setNoWeightedPredictionConstraintFlag(false);
#endif
    m_cEncLib.setNoRefWraparoundConstraintFlag(false);
    m_cEncLib.setNoTemporalMvpConstraintFlag(false);
    m_cEncLib.setNoSbtmvpConstraintFlag(false);
    m_cEncLib.setNoAmvrConstraintFlag(false);
    m_cEncLib.setNoBdofConstraintFlag(false);
    m_cEncLib.setNoDmvrConstraintFlag(false);
    m_cEncLib.setNoCclmConstraintFlag(false);
    m_cEncLib.setNoMtsConstraintFlag(false);
    m_cEncLib.setNoSbtConstraintFlag(false);
    m_cEncLib.setNoAffineMotionConstraintFlag(false);
    m_cEncLib.setNoBcwConstraintFlag(false);
    m_cEncLib.setNoIbcConstraintFlag(false);
    m_cEncLib.setNoCiipConstraintFlag(false);
    m_cEncLib.setNoGeoConstraintFlag(false);
#if ENABLE_DIMD
    m_cEncLib.setNoDimdConstraintFlag(false);
#endif
#if JVET_W0123_TIMD_FUSION
    m_cEncLib.setNoTimdConstraintFlag(false);
#endif
#if JVET_AB0155_SGPM
    m_cEncLib.setNoSgpmConstraintFlag(false);
#endif
#if JVET_AD0082_TMRL_CONFIG
    m_cEncLib.setNoTmrlConstraintFlag(false);
#endif
#if JVET_AG0058_EIP
    m_cEncLib.setNoEipConstraintFlag(false);
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
    m_cEncLib.setNoIntraPredBfConstraintFlag(false);
#endif
#if ENABLE_OBMC
    m_cEncLib.setNoObmcConstraintFlag(false);
#endif
    m_cEncLib.setNoLadfConstraintFlag(false);
    m_cEncLib.setNoTransformSkipConstraintFlag(false);
    m_cEncLib.setNoBDPCMConstraintFlag(false);
    m_cEncLib.setNoJointCbCrConstraintFlag(false);
    m_cEncLib.setNoQpDeltaConstraintFlag(false);
    m_cEncLib.setNoDepQuantConstraintFlag(false);
    m_cEncLib.setNoSignDataHidingConstraintFlag(false);
    m_cEncLib.setNoTrailConstraintFlag(false);
    m_cEncLib.setNoStsaConstraintFlag(false);
    m_cEncLib.setNoRaslConstraintFlag(false);
    m_cEncLib.setNoRadlConstraintFlag(false);
    m_cEncLib.setNoIdrConstraintFlag(false);
    m_cEncLib.setNoCraConstraintFlag(false);
    m_cEncLib.setNoGdrConstraintFlag(false);
    m_cEncLib.setNoApsConstraintFlag(false);
    m_cEncLib.setNoMrlConstraintFlag(false);
    m_cEncLib.setNoIspConstraintFlag(false);
    m_cEncLib.setNoMipConstraintFlag(false);
    m_cEncLib.setNoLfnstConstraintFlag(false);
    m_cEncLib.setNoMmvdConstraintFlag(false);
    m_cEncLib.setNoSmvdConstraintFlag(false);
    m_cEncLib.setNoProfConstraintFlag(false);
    m_cEncLib.setNoPaletteConstraintFlag(false);
    m_cEncLib.setNoActConstraintFlag(false);
    m_cEncLib.setNoLmcsConstraintFlag(false);
#if JVET_R0341_GCI
    m_cEncLib.setNoChromaQpOffsetConstraintFlag(false);
#endif
  }

  //====== Coding Structure ========
  m_cEncLib.setIntraPeriod                                       ( m_iIntraPeriod );
#if JVET_Z0118_GDR
  m_cEncLib.setGdrEnabled                                        ( m_gdrEnabled );
  m_cEncLib.setGdrPeriod                                         ( m_gdrPeriod );
  m_cEncLib.setGdrPocStart                                       ( m_gdrPocStart );
  m_cEncLib.setGdrInterval                                       ( m_gdrInterval);
  m_cEncLib.setGdrNoHash                                         ( m_gdrNoHash );
#endif
  m_cEncLib.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cEncLib.setGOPSize                                           ( m_iGOPSize );
  m_cEncLib.setDrapPeriod                                        ( m_drapPeriod );
  m_cEncLib.setReWriteParamSets                                  ( m_rewriteParamSets );
  m_cEncLib.setRPLList0                                          ( m_RPLList0);
  m_cEncLib.setRPLList1                                          ( m_RPLList1);
  m_cEncLib.setIDRRefParamListPresent                            ( m_idrRefParamList );
  m_cEncLib.setGopList                                           ( m_GOPList );
  
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  m_cEncLib.setIsRA                                              ( m_isRA );
  m_cEncLib.setNumQPOffset                                       ( m_numQPOffset );
  m_cEncLib.setQPOffsetList                                      ( m_qpOffsetList );
#endif

  for(int i = 0; i < MAX_TLAYER; i++)
  {
    m_cEncLib.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cEncLib.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( uint32_t uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cEncLib.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cEncLib.setIntraLambdaModifier                               ( m_adIntraLambdaModifier );
  m_cEncLib.setIntraQpFactor                                     ( m_dIntraQpFactor );

  m_cEncLib.setBaseQP                                            ( m_iQP );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cEncLib.setIntraQPOffset                                     ( m_intraQPOffset );
  m_cEncLib.setLambdaFromQPEnable                                ( m_lambdaFromQPEnable );
#endif
  m_cEncLib.setChromaQpMappingTableParams                         (m_chromaQpMappingTableParams);

#if JVET_AA0146_WRAP_AROUND_FIX
  m_cEncLib.setSourcePadding                                     ( m_sourcePadding );
#else
  m_cEncLib.setPad                                               ( m_aiPad );
#endif

  m_cEncLib.setAccessUnitDelimiter                               ( m_AccessUnitDelimiter );
  m_cEncLib.setEnablePictureHeaderInSliceHeader                  ( m_enablePictureHeaderInSliceHeader );

  m_cEncLib.setMaxTempLayer                                      ( m_maxTempLayer );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
#if JVET_AB0171_ASYMMETRIC_DB_FOR_GDR
  m_cEncLib.setAsymmetricILF(m_asymmetricILF);
#endif
  m_cEncLib.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cEncLib.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cEncLib.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cEncLib.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_cEncLib.setLoopFilterCbBetaOffset                            ( m_loopFilterCbBetaOffsetDiv2  );
  m_cEncLib.setLoopFilterCbTcOffset                              ( m_loopFilterCbTcOffsetDiv2    );
  m_cEncLib.setLoopFilterCrBetaOffset                            ( m_loopFilterCrBetaOffsetDiv2  );
  m_cEncLib.setLoopFilterCrTcOffset                              ( m_loopFilterCrTcOffsetDiv2    );
#if W0038_DB_OPT
  m_cEncLib.setDeblockingFilterMetric                            ( m_deblockingFilterMetric );
#else
  m_cEncLib.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );
#endif

  //====== Motion search ========
  m_cEncLib.setDisableIntraPUsInInterSlices                      ( m_bDisableIntraPUsInInterSlices );
  m_cEncLib.setMotionEstimationSearchMethod                      ( m_motionEstimationSearchMethod  );
  m_cEncLib.setSearchRange                                       ( m_iSearchRange );
  m_cEncLib.setBipredSearchRange                                 ( m_bipredSearchRange );
  m_cEncLib.setClipForBiPredMeEnabled                            ( m_bClipForBiPredMeEnabled );
  m_cEncLib.setFastMEAssumingSmootherMVEnabled                   ( m_bFastMEAssumingSmootherMVEnabled );
  m_cEncLib.setMinSearchWindow                                   ( m_minSearchWindow );
  m_cEncLib.setRestrictMESampling                                ( m_bRestrictMESampling );

  //====== Quality control ========
  m_cEncLib.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cEncLib.setCuQpDeltaSubdiv                                   ( m_cuQpDeltaSubdiv );
  m_cEncLib.setCuChromaQpOffsetSubdiv                            ( m_cuChromaQpOffsetSubdiv );
  m_cEncLib.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cEncLib.setChromaCrQpOffset                                  ( m_crQpOffset  );
  m_cEncLib.setChromaCbQpOffsetDualTree                          ( m_cbQpOffsetDualTree );
  m_cEncLib.setChromaCrQpOffsetDualTree                          ( m_crQpOffsetDualTree );
  m_cEncLib.setChromaCbCrQpOffset                                ( m_cbCrQpOffset         );
  m_cEncLib.setChromaCbCrQpOffsetDualTree                        ( m_cbCrQpOffsetDualTree );
#if ER_CHROMA_QP_WCG_PPS
  m_cEncLib.setWCGChromaQpControl                                ( m_wcgChromaQpControl );
#endif
#if W0038_CQP_ADJ
  m_cEncLib.setSliceChromaOffsetQpIntraOrPeriodic                ( m_sliceChromaQpOffsetPeriodicity, m_sliceChromaQpOffsetIntraOrPeriodic );
#endif
  m_cEncLib.setChromaFormatIdc                                   ( m_chromaFormatIDC  );
  m_cEncLib.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_cEncLib.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
#if ENABLE_QPA
  m_cEncLib.setUsePerceptQPA                                     ( m_bUsePerceptQPA && !m_bUseAdaptiveQP );
  m_cEncLib.setUseWPSNR                                          ( m_bUseWPSNR );
#endif
  m_cEncLib.setExtendedPrecisionProcessingFlag                   ( m_extendedPrecisionProcessingFlag );
  m_cEncLib.setHighPrecisionOffsetsEnabledFlag                   ( m_highPrecisionOffsetsEnabledFlag );

  m_cEncLib.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
#if SHARP_LUMA_DELTA_QP
  m_cEncLib.setLumaLevelToDeltaQPControls                        ( m_lumaLevelToDeltaQPMapping );
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cEncLib.setDeltaQpRD( (m_costMode==COST_LOSSLESS_CODING) ? 0 : m_uiDeltaQpRD );
#else
  m_cEncLib.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
#endif
  m_cEncLib.setFastDeltaQp                                       ( m_bFastDeltaQP  );
  m_cEncLib.setUseASR                                            ( m_bUseASR      );
  m_cEncLib.setUseHADME                                          ( m_bUseHADME    );
  m_cEncLib.setdQPs                                              ( m_aidQP        );
  m_cEncLib.setUseRDOQ                                           ( m_useRDOQ     );
  m_cEncLib.setUseRDOQTS                                         ( m_useRDOQTS   );
#if T0196_SELECTIVE_RDOQ
  m_cEncLib.setUseSelectiveRDOQ                                  ( m_useSelectiveRDOQ );
#endif
  m_cEncLib.setRDpenalty                                         ( m_rdPenalty );
  m_cEncLib.setCTUSize                                           ( m_uiCTUSize );
  m_cEncLib.setSubPicInfoPresentFlag                             ( m_subPicInfoPresentFlag );
  if(m_subPicInfoPresentFlag)
  {
    m_cEncLib.setNumSubPics                                      ( m_numSubPics );
#if JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT
    m_cEncLib.setSubPicSameSizeFlag                              ( m_subPicSameSizeFlag );
#endif
    m_cEncLib.setSubPicCtuTopLeftX                               ( m_subPicCtuTopLeftX );
    m_cEncLib.setSubPicCtuTopLeftY                               ( m_subPicCtuTopLeftY );
    m_cEncLib.setSubPicWidth                                     ( m_subPicWidth );
    m_cEncLib.setSubPicHeight                                    ( m_subPicHeight );
    m_cEncLib.setSubPicTreatedAsPicFlag                          ( m_subPicTreatedAsPicFlag );
    m_cEncLib.setLoopFilterAcrossSubpicEnabledFlag               ( m_loopFilterAcrossSubpicEnabledFlag );
    m_cEncLib.setSubPicIdMappingInSpsFlag                        ( m_subPicIdMappingInSpsFlag );
    m_cEncLib.setSubPicIdLen                                     ( m_subPicIdLen );
    m_cEncLib.setSubPicIdMappingExplicitlySignalledFlag          ( m_subPicIdMappingExplicitlySignalledFlag );
    if (m_subPicIdMappingExplicitlySignalledFlag)
    {
      m_cEncLib.setSubPicId                                      ( m_subPicId );
    }
  }
  else
  {
    m_cEncLib.setNumSubPics                                      ( 1 );
    m_cEncLib.setSubPicIdMappingExplicitlySignalledFlag          ( false );
  }

  m_cEncLib.setUseSplitConsOverride                              ( m_SplitConsOverrideEnabledFlag );
  m_cEncLib.setMinQTSizes                                        ( m_uiMinQT );
  m_cEncLib.setMaxMTTHierarchyDepth                              ( m_uiMaxMTTHierarchyDepth, m_uiMaxMTTHierarchyDepthI, m_uiMaxMTTHierarchyDepthIChroma );
  m_cEncLib.setMaxBTSizes                                        ( m_uiMaxBT );
  m_cEncLib.setMaxTTSizes                                        ( m_uiMaxTT );
#if JVET_Y0152_TT_ENC_SPEEDUP
  m_cEncLib.setFastTTskip                                        ( m_ttFastSkip );
  m_cEncLib.setFastTTskipThr                                     ( m_ttFastSkipThr );
#endif
  m_cEncLib.setDualITree                                         ( m_dualTree );
#if SIGN_PREDICTION
  m_cEncLib.setNumPredSigns                                      ( m_numPredSign );
#if JVET_Y0141_SIGN_PRED_IMPROVE
  m_cEncLib.setLog2SignPredArea                                  (m_log2SignPredArea);
#endif
#endif
#if AHG7_MTS_TOOLOFF_CFG
  m_cEncLib.setMTSExt                                            (m_MTSExt);
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  m_cEncLib.setIntraLFNSTISlice                                  ( m_intraLFNSTISlice );
  m_cEncLib.setIntraLFNSTPBSlice                                 ( m_intraLFNSTPBSlice );
  m_cEncLib.setInterLFNST                                        ( m_interLFNST );
#else
  m_cEncLib.setLFNST                                             ( m_LFNST );
#endif
  m_cEncLib.setUseFastLFNST                                      ( m_useFastLFNST );
#if JVET_AI0050_INTER_MTSS
  m_cEncLib.setUseInterMTSS                                      ( m_useInterMTSS );
#endif
#if JVET_AI0050_SBT_LFNST
  m_cEncLib.setUseSBTLFNST                                       ( m_useSbtLFNST );
#endif
#if AHG7_LN_TOOLOFF_CFG
  m_cEncLib.setNSPT                                              ( m_NSPT );
  m_cEncLib.setLFNSTExt                                          ( m_LFNSTExt );
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  m_cEncLib.setUseFastInterLFNST                                 ( m_useFastInterLFNST );
#endif
  m_cEncLib.setSbTmvpEnabledFlag                                 ( m_sbTmvpEnableFlag );
  m_cEncLib.setAffine                                            ( m_Affine );
  m_cEncLib.setAffineType                                        ( m_AffineType );
#if JVET_AI0185_ADAPTIVE_COST_IN_MERGE_MODE
  m_cEncLib.setUseAltCost                                        ( m_useAltCost );
#endif
#if JVET_AJ0126_INTER_AMVP_ENHANCEMENT
  m_cEncLib.setUseExtAmvp                                        ( m_useExtAmvp );
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  m_cEncLib.setUseAffineTM                                       ( m_useAffineTM );
#if JVET_AG0276_NLIC
  m_cEncLib.setUseAffAltLMTM                                     ( m_useAffAltLMTM );
#endif
#if JVET_AH0119_SUBBLOCK_TM
  m_cEncLib.setUseSbTmvpTM                                       ( m_useSbTmvpTM );
#endif
#endif
#if AFFINE_MMVD
  m_cEncLib.setAffineMmvdMode                                    ( m_AffineMmvdMode );
#endif
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM || MULTI_PASS_DMVR
  m_cEncLib.setUseDMVDMode                                       ( m_DMVDMode );
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  m_cEncLib.setTMToolsEnableFlag                                 ( m_tmToolsEnableFlag );
#if TM_AMVP
  m_cEncLib.setUseTMAmvpMode                                     ( m_tmAmvpMode );
#endif
#if TM_MRG
  m_cEncLib.setUseTMMrgMode                                      ( m_tmMrgMode );
#endif
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
  m_cEncLib.setUseGPMTMMode                                      ( m_tmGPMMode );
#endif
#if JVET_Z0061_TM_OBMC && ENABLE_OBMC
  m_cEncLib.setUseOBMCTMMode                                     ( m_tmOBMCMode );
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  m_cEncLib.setUseCIIPTMMode                                     ( m_tmCIIPMode );
#endif
#if JVET_AG0135_AFFINE_CIIP
  m_cEncLib.setUseCIIPAffine                                     ( m_useCiipAffine );
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  m_cEncLib.setUseTmvpNmvpReordering                             ( m_useTmvpNmvpReorder );
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  m_cEncLib.setUseTMMMVD                                         ( m_useTMMMVD );
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  m_cEncLib.setUseAltGPMSplitModeCode                            ( m_altGPMSplitModeCode );
#endif
#if JVET_AL0134_SGPM_INTER
  m_cEncLib.setUseSgpmInter                                      ( m_useSgpmInter );
#endif
  m_cEncLib.setPROF                                              ( m_PROF );
  m_cEncLib.setBIO                                               (m_BIO);
#if JVET_W0090_ARMC_TM
  m_cEncLib.setAML                                               ( m_AML );
#if JVET_AG0276_NLIC
  m_cEncLib.setAltLM                                             ( m_altLM );
  m_cEncLib.setAffAltLM                                          ( m_affAltLM );
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  m_cEncLib.setMergeOppositeLic                                  ( m_mergeOppositeLic );
  m_cEncLib.setMergeTMOppositeLic                                ( m_mergeTMOppositeLic );
  m_cEncLib.setMergeAffOppositeLic                               ( m_mergeAffOppositeLic );
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  m_cEncLib.setArmcRefinedMotion                                 ( m_iQP < 25 ? false : m_armcRefinedMotion );
#endif
  m_cEncLib.setUseLMChroma                                       ( m_LMChroma );
  m_cEncLib.setHorCollocatedChromaFlag                           ( m_horCollocatedChromaFlag );
  m_cEncLib.setVerCollocatedChromaFlag                           ( m_verCollocatedChromaFlag );
  m_cEncLib.setIntraMTS                                          ( m_MTS & 1 );
  m_cEncLib.setInterMTS                                          ( ( m_MTS >> 1 ) & 1 );
  m_cEncLib.setMTSIntraMaxCand                                   ( m_MTSIntraMaxCand );
  m_cEncLib.setMTSInterMaxCand                                   ( m_MTSInterMaxCand );
  m_cEncLib.setImplicitMTS                                       ( m_MTSImplicit );
  m_cEncLib.setUseSBT                                            ( m_SBT );
  m_cEncLib.setSBTFast64WidthTh                                  ( m_SBTFast64WidthTh );
  m_cEncLib.setUseCompositeRef                                   ( m_compositeRefEnabled );
  m_cEncLib.setUseSMVD                                           ( m_SMVD );
  m_cEncLib.setUseBcw                                            ( m_bcw );
  m_cEncLib.setUseBcwFast                                        ( m_BcwFast );
#if ENABLE_OBMC
  m_cEncLib.setUseOBMC                                           ( m_OBMC );
#endif
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  m_cEncLib.setUseLadf                                           ( m_LadfEnabed );
  if ( m_LadfEnabed )
  {
    m_cEncLib.setLadfNumIntervals                                ( m_LadfNumIntervals);
    for ( int k = 0; k < m_LadfNumIntervals; k++ )
    {
      m_cEncLib.setLadfQpOffset( m_LadfQpOffset[k], k );
      m_cEncLib.setLadfIntervalLowerBound(m_LadfIntervalLowerBound[k], k);
    }
  }
#endif
#if JVET_AC0096
  if (m_rprFunctionalityTestingEnabledFlag)
  {
    for (int k = 0; k < m_rprSwitchingListSize; k++)
    {
      m_cEncLib.setRprSwitchingResolutionOrderList(m_rprSwitchingResolutionOrderList[k], k);
      m_cEncLib.setRprSwitchingQPOffsetOrderList(m_rprSwitchingQPOffsetOrderList[k], k);
    }
    m_cEncLib.setRprSwitchingListSize(m_rprSwitchingListSize);
  }
#endif
#if JVET_AA0133_INTER_MTS_OPT
  m_cEncLib.setInterMTSMaxSize(m_interMTSMaxSize);
#endif
#if AHG7_MTS_TOOLOFF_CFG
  m_cEncLib.setIntraMTSMaxSize(m_intraMTSMaxSize);
#endif
#if ENABLE_DIMD
  m_cEncLib.setUseDimd                                           ( m_dimd );
#endif
#if JVET_AL0108_BVG_DIMD
  m_cEncLib.setUseBvgDimd                                        ( m_bvgDimd );
#endif
#if JVET_W0123_TIMD_FUSION
  m_cEncLib.setUseTimd                                           ( m_timd );
#if JVET_AJ0061_TIMD_MERGE
  m_cEncLib.setUseTimdMrg                                        ( m_timdMrg );
  if (!m_timd)
  {
    m_cEncLib.setUseTimdMrg                                        ( false );
  }
#endif
#endif
#if JVET_AB0155_SGPM
  m_cEncLib.setUseSgpm                                           ( m_sgpm );
#if JVET_AC0189_SGPM_NO_BLENDING
  m_cEncLib.setUseSgpmNoBlend                                    ( m_sgpmNoBlend );
#endif
#endif
#if JVET_AD0082_TMRL_CONFIG
  m_cEncLib.setUseTmrl                                           ( m_tmrl );
#endif
#if JVET_AN0095_QUANTIZATION_CENTER_SHIFT
  m_cEncLib.setUseQcs                                            ( m_qcs );
#endif

#if JVET_AG0058_EIP
  m_cEncLib.setUseEip                                            ( m_eip );
#endif
#if JVET_AK0118_BF_FOR_INTRA_PRED
  m_cEncLib.setUseIntraPredBf                                    ( m_intraPredBf );
#endif
#if JVET_AD0085_MPM_SORTING
  m_cEncLib.setUseMpmSorting                                     ( m_mpmSorting );
#endif
#if JVET_AK0059_MDIP
  m_cEncLib.setUseMdip                                           ( m_mdip ); 
#endif
#if JVET_AH0136_CHROMA_REORDERING
  m_cEncLib.setUseChromaReordering                               (m_chromaReordering);
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  m_cEncLib.setUseCccm                                           ( m_cccm );
#endif
#if JVET_AD0188_CCP_MERGE
  m_cEncLib.setUseCcpMerge                                       ( m_ccpMerge );
#endif
#if JVET_AL0126_CCP_MERGE_WITH_ADJUST
  m_cEncLib.setMaxNumCcpMergeAdjustCand                          ( m_maxNumCcpMergeAdjustCand );
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  m_cEncLib.setUseDdCcpFusion                                    ( m_cccm ? m_ddCcpFusion : false );
#endif
#if ENABLE_OBMC
  m_cEncLib.setUseObmc                                           ( m_OBMC );
#endif
  m_cEncLib.setUseCiip                                           ( m_ciip );
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  m_cEncLib.setUseCiipTimd                                       (m_ciipTimd);
#endif
  m_cEncLib.setUseGeo                                            ( m_Geo );
#if JVET_AI0082_GPM_WITH_INTER_IBC
  m_cEncLib.setUseGeoInterIbc                                    ( m_Geo ? m_geoInterIbc : false );
#endif
#if JVET_AJ0107_GPM_SHAPE_ADAPT
  m_cEncLib.setUseGeoShapeAdapt                                  ( m_Geo ? m_geoShapeAdapt : false);
#endif
#if JVET_AK0101_REGRESSION_GPM_INTRA
  m_cEncLib.setUseGeoBlendIntra                                  ( m_Geo ? m_geoBlendIntra : false);
#endif
#if JVET_AN0203_RGPM_NO_TM
  m_cEncLib.setUseGeoBlendInter                                  ( m_Geo ? m_geoBlendInter : false);
#endif
  m_cEncLib.setUseHashME                                         ( m_HashME );

  m_cEncLib.setAllowDisFracMMVD                                  ( m_allowDisFracMMVD );
  m_cEncLib.setUseAffineAmvr                                     ( m_AffineAmvr );
  m_cEncLib.setUseAffineAmvrEncOpt                               ( m_AffineAmvrEncOpt );
  m_cEncLib.setUseAffineAmvp                                     ( m_AffineAmvp );
  m_cEncLib.setDMVR                                              ( m_DMVR );
  m_cEncLib.setMMVD                                              ( m_MMVD );
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  m_cEncLib.setAffineParaRefinement                              (m_affineParaRefinement);
#endif
  m_cEncLib.setMmvdDisNum                                        (m_MmvdDisNum);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
  m_cEncLib.setUseMvdPred                                        (m_mvdPred);
#endif
#if JVET_AC0104_IBC_BVD_PREDICTION
  m_cEncLib.setUseBvdPred                                        (m_bvdPred);
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  m_cEncLib.setUseBvpCluster                                     (m_bvpCluster);
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  m_cEncLib.setUseARL                                            (m_useARL);
#endif
  m_cEncLib.setRGBFormatFlag(m_rgbFormat);
  m_cEncLib.setUseColorTrans(m_useColorTrans);
  m_cEncLib.setPLTMode                                           ( m_PLTMode );
  m_cEncLib.setJointCbCr                                         ( m_JointCbCrMode );
  m_cEncLib.setIBCMode                                           ( m_IBCMode );
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_cEncLib.setIBCFracMode                                       ( m_IBCFracMode );
#endif
  m_cEncLib.setIBCLocalSearchRangeX                              ( m_IBCLocalSearchRangeX );
  m_cEncLib.setIBCLocalSearchRangeY                              ( m_IBCLocalSearchRangeY );
  m_cEncLib.setIBCHashSearch                                     ( m_IBCHashSearch );
  m_cEncLib.setIBCHashSearchMaxCand                              ( m_IBCHashSearchMaxCand );
  m_cEncLib.setIBCHashSearchRange4SmallBlk                       ( m_IBCHashSearchRange4SmallBlk );
  m_cEncLib.setIBCFastMethod                                     ( m_IBCFastMethod );
#if JVET_AF0057
  if (m_dmvrEncSelect && (m_iQP >= m_dmvrEncSelectBaseQpTh))
  {
    m_cEncLib.setDMVREncMvSelection                              (true);
  }
  else
  {
    m_cEncLib.setDMVREncMvSelection                              (false);
  }
  m_cEncLib.setDMVREncMvSelectDisableHighestTemporalLayer        (m_dmvrEncSelectDisableHighestTemporalLayer);
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  m_cEncLib.setTMnoninterToolsEnableFlag                         ( m_tmNoninterToolsEnableFlag );
#endif
#if JVET_AA0061_IBC_MBVD
  m_cEncLib.setIbcMbvd                                           ( m_ibcMbvd );
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  m_cEncLib.setIbcMbvdAdSearch                                   ( m_ibcMbvdAdSearch );
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  m_cEncLib.setIbcCiip                                           ( m_ibcCiip );
#endif
#if JVET_AC0112_IBC_GPM
  m_cEncLib.setIbcGpm                                            ( m_ibcGpm );
#endif
#if JVET_AC0112_IBC_LIC
  m_cEncLib.setIbcLic                                            ( m_ibcLic );
#endif
#if JVET_AE0159_FIBC
  m_cEncLib.setIbcFilter                                         ( m_ibcFilter );
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  m_cEncLib.setIbcBiPred                                         ( m_ibcBiPred );
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  m_cEncLib.setIbcNonAdjCand                                     ( m_ibcNonAdjCand);
#endif
#if JVET_AG0136_INTRA_TMP_LIC
  m_cEncLib.setItmpLicExtension                                  ( m_itmpLicExtension );
  m_cEncLib.setItmpLicMode                                       ( m_itmpLicMode );
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_cEncLib.setRRIbc(m_rribc);
  m_cEncLib.setTMIbc(m_tmibc);
  m_cEncLib.setIbcMerge                                          ( m_ibcMerge );
#endif
#if JVET_AJ0057_HL_INTRA_METHOD_CONTROL
  m_cEncLib.setIntraToolControlMode                              ( m_intraToolControlMode );
#endif
  m_cEncLib.setUseWrapAround                                     ( m_wrapAround );
  m_cEncLib.setWrapAroundOffset                                  ( m_wrapAroundOffset );
#if JVET_V0130_INTRA_TMP
  m_cEncLib.setUseIntraTMP                                       ( m_intraTMP );
  m_cEncLib.setIntraTMPMaxSize                                   ( m_intraTmpMaxSize );
#if JVET_AB0130_ITMP_SAMPLING
  m_cEncLib.setUseFastIntraTMP(m_fastIntraTMP);
#endif
#endif
#if JVET_AC0071_DBV
  m_cEncLib.setUseIntraDBV(m_intraDBV);
#endif
#if JVET_AE0100_BVGCCCM
  m_cEncLib.setUseBvgCccm(m_bvgCccm);
#endif
#if JVET_V0094_BILATERAL_FILTER
  m_cEncLib.setUseBIF                                            ( m_BIF );
  m_cEncLib.setBIFStrength                                       ( m_BIFStrength );
  m_cEncLib.setBIFQPOffset                                       ( m_BIFQPOffset );
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  m_cEncLib.setUseChromaBIF                                      ( m_chromaBIF );
  m_cEncLib.setChromaBIFStrength                                 ( m_chromaBIFStrength );
  m_cEncLib.setChromaBIFQPOffset                                 ( m_chromaBIFQPOffset );
#endif

#if JVET_AE0059_INTER_CCCM
  m_cEncLib.setUseInterCccm                                      ( m_interCccm );
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  m_cEncLib.setUseInterCcpMerge                                  ( m_interCcpMerge );
  m_cEncLib.setInterCcpMergeFastMode                             ( m_interCcpMergeFastMode );
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  m_cEncLib.setUseInterCcpMergeZeroLumaCbf                       ( m_interCcpMergeZeroLumaCbf );
  m_cEncLib.setInterCcpMergeZeroLumaCbfFastMode                  ( m_interCcpMergeZeroLumaCbfFastMode );
#endif
#endif
#if JVET_AH0209_PDP
  m_cEncLib.setUsePDP                                           ( m_pdp );
#endif
#if JVET_AI0183_MVP_EXTENSION
  m_cEncLib.setConfigScaledMvExtTmvp                             ( m_scaledMvExtTmvp );
  m_cEncLib.setConfigScaledMvExtBiTmvp                           ( m_scaledMvExtBiTmvp );
#endif
#if JVET_AJ0158_SUBBLOCK_INTER_EXTENSION
  m_cEncLib.setConfigSbTmvpMvExt                                 ( m_sbTmvpMvExt );
#endif
#if JVET_AL0157_MERGE_CANDIDATE_EXTENSION
  m_cEncLib.setConfigBoundaryMvpExt                              ( m_boundaryMvpExt );
#endif
#if JVET_AL0214_MV_REFINEMENT_FOR_TMVP
  m_cEncLib.setRefineTmvpCfgIdx                                  ( m_refineTmvpCfgIdx );
#endif

  // ADD_NEW_TOOL : (encoder app) add setting of tool enabling flags and associated parameters here
#if JVET_AN0086_RESIDUAL_CHECK
  m_cEncLib.setAlfResidualCheckEnabled                           ( m_encAlfResidualCheck );
#endif
  m_cEncLib.setVirtualBoundariesEnabledFlag                      ( m_virtualBoundariesEnabledFlag );
  if( m_cEncLib.getVirtualBoundariesEnabledFlag() )
  {
    m_cEncLib.setVirtualBoundariesPresentFlag                      ( m_virtualBoundariesPresentFlag );
    m_cEncLib.setNumVerVirtualBoundaries                           ( m_numVerVirtualBoundaries );
    m_cEncLib.setNumHorVirtualBoundaries                           ( m_numHorVirtualBoundaries );
    for( unsigned i = 0; i < m_numVerVirtualBoundaries; i++ )
    {
      m_cEncLib.setVirtualBoundariesPosX                           ( m_virtualBoundariesPosX[ i ], i );
    }
    for( unsigned i = 0; i < m_numHorVirtualBoundaries; i++ )
    {
      m_cEncLib.setVirtualBoundariesPosY                           ( m_virtualBoundariesPosY[ i ], i );
    }
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  m_cEncLib.setInterSliceSeparateTreeEnabled                     ( m_interSliceSeparateTreeEnabled );
#endif

  m_cEncLib.setMaxCUWidth                                        ( m_uiCTUSize );
  m_cEncLib.setMaxCUHeight                                       ( m_uiCTUSize );
  m_cEncLib.setLog2MinCodingBlockSize                            ( m_log2MinCuSize );
  m_cEncLib.setLog2MaxTbSize                                     ( m_log2MaxTbSize );
  m_cEncLib.setUseEncDbOpt(m_encDbOpt);
  m_cEncLib.setUseFastLCTU                                       ( m_useFastLCTU );
  m_cEncLib.setFastInterSearchMode                               ( m_fastInterSearchMode );
  m_cEncLib.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cEncLib.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cEncLib.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cEncLib.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cEncLib.setUseFastMerge                                      ( m_useFastMrg );
#if MERGE_ENC_OPT
  m_cEncLib.setNumFullRDMerge                                    ( m_numFullRDMrg );
#endif
  m_cEncLib.setUsePbIntraFast                                    ( m_usePbIntraFast );
  m_cEncLib.setUseAMaxBT                                         ( m_useAMaxBT );
  m_cEncLib.setUseE0023FastEnc                                   ( m_e0023FastEnc );
  m_cEncLib.setUseContentBasedFastQtbt                           ( m_contentBasedFastQtbt );
  m_cEncLib.setUseNonLinearAlfLuma                               ( m_useNonLinearAlfLuma );
  m_cEncLib.setUseNonLinearAlfChroma                             ( m_useNonLinearAlfChroma );
  m_cEncLib.setMaxNumAlfAlternativesChroma                       ( m_maxNumAlfAlternativesChroma );
  m_cEncLib.setUseMRL                                            ( m_MRL );
  m_cEncLib.setUseMIP                                            ( m_MIP );
  m_cEncLib.setUseFastMIP                                        ( m_useFastMIP );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  m_cEncLib.setFastLocalDualTreeMode                             ( m_fastLocalDualTreeMode );
#endif
  m_cEncLib.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cEncLib.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cEncLib.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_cEncLib.setUseChromaTS                                       ( m_useChromaTS && m_useTransformSkip);
  m_cEncLib.setUseBDPCM                                          ( m_useBDPCM );
  m_cEncLib.setTransformSkipRotationEnabledFlag                  ( m_transformSkipRotationEnabledFlag );
  m_cEncLib.setTransformSkipContextEnabledFlag                   ( m_transformSkipContextEnabledFlag   );
  m_cEncLib.setPersistentRiceAdaptationEnabledFlag               ( m_persistentRiceAdaptationEnabledFlag );
  m_cEncLib.setCabacBypassAlignmentEnabledFlag                   ( m_cabacBypassAlignmentEnabledFlag );
  m_cEncLib.setLog2MaxTransformSkipBlockSize                     ( m_log2MaxTransformSkipBlockSize  );
  m_cEncLib.setFastUDIUseMPMEnabled                              ( m_bFastUDIUseMPMEnabled );
  m_cEncLib.setFastMEForGenBLowDelayEnabled                      ( m_bFastMEForGenBLowDelayEnabled );
  m_cEncLib.setUseBLambdaForNonKeyLowDelayPictures               ( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_cEncLib.setUseISP                                            ( m_ISP );
  m_cEncLib.setUseFastISP                                        ( m_useFastISP );
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  m_cEncLib.setTempCabacInitMode                                 ( m_tempCabacInitMode );
#endif
  #if JVET_AJ0226_MTT_SKIP 
  m_cEncLib.setUseMttSkip                                        (m_useMttSkip);
#endif
  // set internal bit-depth and constants
  for (uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
  {
    m_cEncLib.setBitDepth((ChannelType)channelType, m_internalBitDepth[channelType]);
    m_cEncLib.setInputBitDepth((ChannelType)channelType, m_inputBitDepth[channelType]);
  }

  m_cEncLib.setMaxNumMergeCand                                   ( m_maxNumMergeCand );
#if JVET_AG0276_LIC_FLAG_SIGNALING
  m_cEncLib.setMaxNumOppositeLicMergeCand                        ( m_maxNumOppositeLicMergeCand );
  m_cEncLib.setMaxNumAffineOppositeLicMergeCand                  ( m_maxNumAffineOppositeLicMergeCand);
#endif
#if JVET_X0049_ADAPT_DMVR
  m_cEncLib.setMaxNumBMMergeCand                                 ( m_maxNumBMMergeCand );
#endif
  m_cEncLib.setMaxNumAffineMergeCand                             ( m_maxNumAffineMergeCand );
  m_cEncLib.setMaxNumGeoCand                                     ( m_maxNumGeoCand );
#if JVET_AG0164_AFFINE_GPM
  m_cEncLib.setMaxNumGpmAffCand                                  ( m_maxNumGpmAffCand );
#if JVET_AJ0274_GPM_AFFINE_TM
  m_cEncLib.setMaxNumGpmAffTmCand                                ( m_maxNumGpmAffTmCand );
#endif
#endif
  m_cEncLib.setMaxNumIBCMergeCand                                ( m_maxNumIBCMergeCand );
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  m_cEncLib.setMaxNumMHPCand                                     ( m_maxNumMHPCand );
#endif

  //====== Weighted Prediction ========
  m_cEncLib.setUseWP                                             ( m_useWeightedPred     );
  m_cEncLib.setWPBiPred                                          ( m_useWeightedBiPred   );

  //====== Parallel Merge Estimation ========
  m_cEncLib.setLog2ParallelMergeLevelMinus2(m_log2ParallelMergeLevel - 2);
  m_cEncLib.setMixedLossyLossless(m_mixedLossyLossless);
  m_cEncLib.setSliceLosslessArray(m_sliceLosslessArray);

  //====== Tiles and Slices ========
  m_cEncLib.setNoPicPartitionFlag( !m_picPartitionFlag );
  if( m_picPartitionFlag )
  {
    m_cEncLib.setTileColWidths( m_tileColumnWidth );
    m_cEncLib.setTileRowHeights( m_tileRowHeight );
    m_cEncLib.setRectSliceFlag( !m_rasterSliceFlag );
    m_cEncLib.setNumSlicesInPic( m_numSlicesInPic );
    m_cEncLib.setTileIdxDeltaPresentFlag( m_tileIdxDeltaPresentFlag );
    m_cEncLib.setRectSlices( m_rectSlices );
    m_cEncLib.setRasterSliceSizes( m_rasterSliceSize );
    m_cEncLib.setLFCrossTileBoundaryFlag( !m_disableLFCrossTileBoundaryFlag );
    m_cEncLib.setLFCrossSliceBoundaryFlag( !m_disableLFCrossSliceBoundaryFlag );
  }
  else
  {
    m_cEncLib.setRectSliceFlag( true );
    m_cEncLib.setNumSlicesInPic( 1 );
    m_cEncLib.setTileIdxDeltaPresentFlag( 0 );
    m_cEncLib.setLFCrossTileBoundaryFlag( true );
    m_cEncLib.setLFCrossSliceBoundaryFlag( true );
  }

  //====== Sub-picture and Slices ========
  m_cEncLib.setSingleSlicePerSubPicFlagFlag                      ( m_singleSlicePerSubPicFlag );
  m_cEncLib.setUseSAO                                            ( m_bUseSAO );
#if JVET_W0066_CCSAO
  m_cEncLib.setUseCCSAO                                          ( m_CCSAO );
#endif
#if JVET_AG0158_ALF_LUMA_COEFF_PRECISION
  m_cEncLib.setUseAlfPrecision                                   ( m_alfPrecision );
#endif
#if JVET_AH0057_CCALF_COEFF_PRECISION
  m_cEncLib.setUseCCALFPrecision                                 ( m_ccalfPrecision );
#endif
#if JVET_AJ0188_CODING_INFO_CLASSIFICATION
  m_cEncLib.setAlfLumaFixedFilterAdjust                          ( m_alfLumaFixedFilterAdjust );
#endif
#if JVET_AK0121_LOOPFILTER_OFFSET_REFINEMENT
  m_cEncLib.setInloopOffsetRefineFlag                            ( m_inloopOffsetRefineFlag );
  m_cEncLib.setInloopOffsetRefineFunc                            ( m_inloopOffsetRefineFunc );
#endif
  m_cEncLib.setTestSAODisableAtPictureLevel                      ( m_bTestSAODisableAtPictureLevel );
  m_cEncLib.setSaoEncodingRate                                   ( m_saoEncodingRate );
  m_cEncLib.setSaoEncodingRateChroma                             ( m_saoEncodingRateChroma );
  m_cEncLib.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cEncLib.setSaoCtuBoundary                                    ( m_saoCtuBoundary);

  m_cEncLib.setSaoGreedyMergeEnc                                 ( m_saoGreedyMergeEnc);
  m_cEncLib.setIntraSmoothingDisabledFlag                        (!m_enableIntraReferenceSmoothing );

#if JVET_AK0085_TM_BOUNDARY_PADDING
  m_cEncLib.setTMBP                                               ( m_templateMatchingBoundaryPrediction );
#endif
  m_cEncLib.setDecodedPictureHashSEIType                         ( m_decodedPictureHashSEIType );
#if JVET_R0294_SUBPIC_HASH
  m_cEncLib.setSubpicDecodedPictureHashType                      ( m_subpicDecodedPictureHashType );
#endif
  m_cEncLib.setDependentRAPIndicationSEIEnabled                  ( m_drapPeriod > 0 );
  m_cEncLib.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_cEncLib.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_cEncLib.setFrameFieldInfoSEIEnabled                          ( m_frameFieldInfoSEIEnabled );
   m_cEncLib.setBpDeltasGOPStructure                             ( m_bpDeltasGOPStructure );
  m_cEncLib.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_cEncLib.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_cEncLib.setHrdParametersPresentFlag                          ( m_hrdParametersPresentFlag );
  m_cEncLib.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_cEncLib.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_cEncLib.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_cEncLib.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_cEncLib.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_cEncLib.setParameterSetsInclusionIndicationSEIEnabled        (m_parameterSetsInclusionIndicationSEIEnabled);
  m_cEncLib.setSelfContainedClvsFlag                             (m_selfContainedClvsFlag);
  m_cEncLib.setErpSEIEnabled                                     ( m_erpSEIEnabled );
  m_cEncLib.setErpSEICancelFlag                                  ( m_erpSEICancelFlag );
  m_cEncLib.setErpSEIPersistenceFlag                             ( m_erpSEIPersistenceFlag );
  m_cEncLib.setErpSEIGuardBandFlag                               ( m_erpSEIGuardBandFlag );
  m_cEncLib.setErpSEIGuardBandType                               ( m_erpSEIGuardBandType );
  m_cEncLib.setErpSEILeftGuardBandWidth                          ( m_erpSEILeftGuardBandWidth );
  m_cEncLib.setErpSEIRightGuardBandWidth                         ( m_erpSEIRightGuardBandWidth );
  m_cEncLib.setSphereRotationSEIEnabled                          ( m_sphereRotationSEIEnabled );
  m_cEncLib.setSphereRotationSEICancelFlag                       ( m_sphereRotationSEICancelFlag );
  m_cEncLib.setSphereRotationSEIPersistenceFlag                  ( m_sphereRotationSEIPersistenceFlag );
  m_cEncLib.setSphereRotationSEIYaw                              ( m_sphereRotationSEIYaw );
  m_cEncLib.setSphereRotationSEIPitch                            ( m_sphereRotationSEIPitch );
  m_cEncLib.setSphereRotationSEIRoll                             ( m_sphereRotationSEIRoll );
  m_cEncLib.setOmniViewportSEIEnabled                            ( m_omniViewportSEIEnabled );
  m_cEncLib.setOmniViewportSEIId                                 ( m_omniViewportSEIId );
  m_cEncLib.setOmniViewportSEICancelFlag                         ( m_omniViewportSEICancelFlag );
  m_cEncLib.setOmniViewportSEIPersistenceFlag                    ( m_omniViewportSEIPersistenceFlag );
  m_cEncLib.setOmniViewportSEICntMinus1                          ( m_omniViewportSEICntMinus1 );
  m_cEncLib.setOmniViewportSEIAzimuthCentre                      ( m_omniViewportSEIAzimuthCentre );
  m_cEncLib.setOmniViewportSEIElevationCentre                    ( m_omniViewportSEIElevationCentre );
  m_cEncLib.setOmniViewportSEITiltCentre                         ( m_omniViewportSEITiltCentre );
  m_cEncLib.setOmniViewportSEIHorRange                           ( m_omniViewportSEIHorRange );
  m_cEncLib.setOmniViewportSEIVerRange                           ( m_omniViewportSEIVerRange );
  m_cEncLib.setRwpSEIEnabled                                     (m_rwpSEIEnabled);
  m_cEncLib.setRwpSEIRwpCancelFlag                               (m_rwpSEIRwpCancelFlag);
  m_cEncLib.setRwpSEIRwpPersistenceFlag                          (m_rwpSEIRwpPersistenceFlag);
  m_cEncLib.setRwpSEIConstituentPictureMatchingFlag              (m_rwpSEIConstituentPictureMatchingFlag);
  m_cEncLib.setRwpSEINumPackedRegions                            (m_rwpSEINumPackedRegions);
  m_cEncLib.setRwpSEIProjPictureWidth                            (m_rwpSEIProjPictureWidth);
  m_cEncLib.setRwpSEIProjPictureHeight                           (m_rwpSEIProjPictureHeight);
  m_cEncLib.setRwpSEIPackedPictureWidth                          (m_rwpSEIPackedPictureWidth);
  m_cEncLib.setRwpSEIPackedPictureHeight                         (m_rwpSEIPackedPictureHeight);
  m_cEncLib.setRwpSEIRwpTransformType                            (m_rwpSEIRwpTransformType);
  m_cEncLib.setRwpSEIRwpGuardBandFlag                            (m_rwpSEIRwpGuardBandFlag);
  m_cEncLib.setRwpSEIProjRegionWidth                             (m_rwpSEIProjRegionWidth);
  m_cEncLib.setRwpSEIProjRegionHeight                            (m_rwpSEIProjRegionHeight);
  m_cEncLib.setRwpSEIRwpSEIProjRegionTop                         (m_rwpSEIRwpSEIProjRegionTop);
  m_cEncLib.setRwpSEIProjRegionLeft                              (m_rwpSEIProjRegionLeft);
  m_cEncLib.setRwpSEIPackedRegionWidth                           (m_rwpSEIPackedRegionWidth);
  m_cEncLib.setRwpSEIPackedRegionHeight                          (m_rwpSEIPackedRegionHeight);
  m_cEncLib.setRwpSEIPackedRegionTop                             (m_rwpSEIPackedRegionTop);
  m_cEncLib.setRwpSEIPackedRegionLeft                            (m_rwpSEIPackedRegionLeft);
  m_cEncLib.setRwpSEIRwpLeftGuardBandWidth                       (m_rwpSEIRwpLeftGuardBandWidth);
  m_cEncLib.setRwpSEIRwpRightGuardBandWidth                      (m_rwpSEIRwpRightGuardBandWidth);
  m_cEncLib.setRwpSEIRwpTopGuardBandHeight                       (m_rwpSEIRwpTopGuardBandHeight);
  m_cEncLib.setRwpSEIRwpBottomGuardBandHeight                    (m_rwpSEIRwpBottomGuardBandHeight);
  m_cEncLib.setRwpSEIRwpGuardBandNotUsedForPredFlag              (m_rwpSEIRwpGuardBandNotUsedForPredFlag);
  m_cEncLib.setRwpSEIRwpGuardBandType                            (m_rwpSEIRwpGuardBandType);
  m_cEncLib.setGcmpSEIEnabled                                    ( m_gcmpSEIEnabled );
  m_cEncLib.setGcmpSEICancelFlag                                 ( m_gcmpSEICancelFlag );
  m_cEncLib.setGcmpSEIPersistenceFlag                            ( m_gcmpSEIPersistenceFlag );
  m_cEncLib.setGcmpSEIPackingType                                ( (uint8_t)m_gcmpSEIPackingType );
  m_cEncLib.setGcmpSEIMappingFunctionType                        ( (uint8_t)m_gcmpSEIMappingFunctionType );
  m_cEncLib.setGcmpSEIFaceIndex                                  ( m_gcmpSEIFaceIndex );
  m_cEncLib.setGcmpSEIFaceRotation                               ( m_gcmpSEIFaceRotation );
  m_cEncLib.setGcmpSEIFunctionCoeffU                             ( m_gcmpSEIFunctionCoeffU );
  m_cEncLib.setGcmpSEIFunctionUAffectedByVFlag                   ( m_gcmpSEIFunctionUAffectedByVFlag );
  m_cEncLib.setGcmpSEIFunctionCoeffV                             ( m_gcmpSEIFunctionCoeffV );
  m_cEncLib.setGcmpSEIFunctionVAffectedByUFlag                   ( m_gcmpSEIFunctionVAffectedByUFlag );
  m_cEncLib.setGcmpSEIGuardBandFlag                              ( m_gcmpSEIGuardBandFlag );
  m_cEncLib.setGcmpSEIGuardBandType                              ( m_gcmpSEIGuardBandType );
  m_cEncLib.setGcmpSEIGuardBandBoundaryExteriorFlag              ( m_gcmpSEIGuardBandBoundaryExteriorFlag );
  m_cEncLib.setGcmpSEIGuardBandSamplesMinus1                     ( (uint8_t)m_gcmpSEIGuardBandSamplesMinus1 );
  m_cEncLib.setSubpicureLevelInfoSEICfg                          (m_cfgSubpictureLevelInfoSEI);
  m_cEncLib.setSampleAspectRatioInfoSEIEnabled                   (m_sampleAspectRatioInfoSEIEnabled);
  m_cEncLib.setSariCancelFlag                                    (m_sariCancelFlag);
  m_cEncLib.setSariPersistenceFlag                               (m_sariPersistenceFlag);
  m_cEncLib.setSariAspectRatioIdc                                (m_sariAspectRatioIdc);
  m_cEncLib.setSariSarWidth                                      (m_sariSarWidth);
  m_cEncLib.setSariSarHeight                                     (m_sariSarHeight);
  m_cEncLib.setMCTSEncConstraint                                 ( m_MCTSEncConstraint);
  m_cEncLib.setMasteringDisplaySEI                               ( m_masteringDisplay );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  m_cEncLib.setSEIAlternativeTransferCharacteristicsSEIEnable    ( m_preferredTransferCharacteristics>=0     );
  m_cEncLib.setSEIPreferredTransferCharacteristics               ( uint8_t(m_preferredTransferCharacteristics) );
#endif
  // film grain charcteristics
  m_cEncLib.setFilmGrainCharactersticsSEIEnabled                 (m_fgcSEIEnabled);
  m_cEncLib.setFilmGrainCharactersticsSEICancelFlag              (m_fgcSEICancelFlag);
  m_cEncLib.setFilmGrainCharactersticsSEIPersistenceFlag         (m_fgcSEIPersistenceFlag);
  m_cEncLib.setFilmGrainCharactersticsSEIModelID                 ((uint8_t)m_fgcSEIModelID);
  m_cEncLib.setFilmGrainCharactersticsSEISepColourDescPresent    (m_fgcSEISepColourDescPresentFlag);
  m_cEncLib.setFilmGrainCharactersticsSEIBlendingModeID          ((uint8_t)m_fgcSEIBlendingModeID);
  m_cEncLib.setFilmGrainCharactersticsSEILog2ScaleFactor         ((uint8_t)m_fgcSEILog2ScaleFactor);
  for (int i = 0; i < MAX_NUM_COMPONENT; i++) {
    m_cEncLib.setFGCSEICompModelPresent                          (m_fgcSEICompModelPresent[i], i);
  }
  // content light level
  m_cEncLib.setCLLSEIEnabled                                     (m_cllSEIEnabled);
  m_cEncLib.setCLLSEIMaxContentLightLevel                        ((uint16_t)m_cllSEIMaxContentLevel);
  m_cEncLib.setCLLSEIMaxPicAvgLightLevel                         ((uint16_t)m_cllSEIMaxPicAvgLevel);
  // ambient viewing enviornment
  m_cEncLib.setAmbientViewingEnvironmentSEIEnabled               (m_aveSEIEnabled);
  m_cEncLib.setAmbientViewingEnvironmentSEIIlluminance           (m_aveSEIAmbientIlluminance);
  m_cEncLib.setAmbientViewingEnvironmentSEIAmbientLightX         ((uint16_t)m_aveSEIAmbientLightX);
  m_cEncLib.setAmbientViewingEnvironmentSEIAmbientLightY         ((uint16_t)m_aveSEIAmbientLightY);
  // content colour volume SEI
  m_cEncLib.setCcvSEIEnabled                                     (m_ccvSEIEnabled);
  m_cEncLib.setCcvSEICancelFlag                                  (m_ccvSEICancelFlag);
  m_cEncLib.setCcvSEIPersistenceFlag                             (m_ccvSEIPersistenceFlag);
  m_cEncLib.setCcvSEIEnabled                                     (m_ccvSEIEnabled);
  m_cEncLib.setCcvSEICancelFlag                                  (m_ccvSEICancelFlag);
  m_cEncLib.setCcvSEIPersistenceFlag                             (m_ccvSEIPersistenceFlag);
  m_cEncLib.setCcvSEIPrimariesPresentFlag                        (m_ccvSEIPrimariesPresentFlag);
  m_cEncLib.setCcvSEIMinLuminanceValuePresentFlag                (m_ccvSEIMinLuminanceValuePresentFlag);
  m_cEncLib.setCcvSEIMaxLuminanceValuePresentFlag                (m_ccvSEIMaxLuminanceValuePresentFlag);
  m_cEncLib.setCcvSEIAvgLuminanceValuePresentFlag                (m_ccvSEIAvgLuminanceValuePresentFlag);
  for(int i = 0; i < MAX_NUM_COMPONENT; i++) {
    m_cEncLib.setCcvSEIPrimariesX                                (m_ccvSEIPrimariesX[i], i);
    m_cEncLib.setCcvSEIPrimariesY                                (m_ccvSEIPrimariesY[i], i);
  }
  m_cEncLib.setCcvSEIMinLuminanceValue                           (m_ccvSEIMinLuminanceValue);
  m_cEncLib.setCcvSEIMaxLuminanceValue                           (m_ccvSEIMaxLuminanceValue);
  m_cEncLib.setCcvSEIAvgLuminanceValue                           (m_ccvSEIAvgLuminanceValue);
  m_cEncLib.setEntropyCodingSyncEnabledFlag                      ( m_entropyCodingSyncEnabledFlag );
  m_cEncLib.setEntryPointPresentFlag                             ( m_entryPointPresentFlag );
  m_cEncLib.setTMVPModeId                                        ( m_TMVPModeId );
  m_cEncLib.setSliceLevelRpl                                     ( m_sliceLevelRpl  );
  m_cEncLib.setSliceLevelDblk                                    ( m_sliceLevelDblk );
  m_cEncLib.setSliceLevelSao                                     ( m_sliceLevelSao  );
  m_cEncLib.setSliceLevelWp                                      ( m_sliceLevelWp );
  m_cEncLib.setSliceLevelDeltaQp                                 ( m_sliceLevelDeltaQp );
  m_cEncLib.setSliceLevelAlf                                     ( m_sliceLevelAlf  );
#if JVET_AH0135_TEMPORAL_PARTITIONING
  m_cEncLib.setEnableMaxMttIncrease                              ( m_enableMaxMttIncrease );
#endif
#if MULTI_HYP_PRED
  m_cEncLib.setNumMHPCandsToTest(m_numMHPCandsToTest);
  m_cEncLib.setMaxNumAddHyps(m_maxNumAddHyps);
  m_cEncLib.setMaxNumAddHypWeights(m_numAddHypWeights);
  m_cEncLib.setMaxNumAddHypRefFrames(m_maxNumAddHypRefFrames);
  m_cEncLib.setAddHypTries(m_addHypTries);
#endif
  m_cEncLib.setUseScalingListId                                  ( m_useScalingListId  );
  m_cEncLib.setScalingListFileName                               ( m_scalingListFileName );
  m_cEncLib.setDisableScalingMatrixForLfnstBlks                  ( m_disableScalingMatrixForLfnstBlks);
  if ( m_cEncLib.getUseColorTrans() && m_cEncLib.getUseScalingListId() )
  {
    m_cEncLib.setDisableScalingMatrixForAlternativeColourSpace(m_disableScalingMatrixForAlternativeColourSpace);
    if( m_cEncLib.getDisableScalingMatrixForAlternativeColourSpace() )
    {
      m_cEncLib.setScalingMatrixDesignatedColourSpace( m_scalingMatrixDesignatedColourSpace );
    }
  }

#if TCQ_8STATES
  m_cEncLib.setDepQuantEnabledIdc                                ( m_depQuantEnabledIdc );
#else
  m_cEncLib.setDepQuantEnabledFlag                               ( m_depQuantEnabledFlag);
#endif
  m_cEncLib.setSignDataHidingEnabledFlag                         ( m_signDataHidingEnabledFlag);
  m_cEncLib.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_cEncLib.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_cEncLib.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_cEncLib.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_cEncLib.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_cEncLib.setInitialQP                                         ( m_RCInitialQP );
  m_cEncLib.setForceIntraQP                                      ( m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
  m_cEncLib.setCpbSaturationEnabled                              ( m_RCCpbSaturationEnabled );
  m_cEncLib.setCpbSize                                           ( m_RCCpbSize );
  m_cEncLib.setInitialCpbFullness                                ( m_RCInitialCpbFullness );
#endif
  m_cEncLib.setCostMode                                          ( m_costMode );
  m_cEncLib.setTSRCdisableLL                                     ( m_TSRCdisableLL );
  m_cEncLib.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_cEncLib.setDCIEnabled                                        ( m_DCIEnabled );
  m_cEncLib.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_cEncLib.setSamePicTimingInAllOLS                             (m_samePicTimingInAllOLS);
  m_cEncLib.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_cEncLib.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_cEncLib.setSarWidth                                          ( m_sarWidth );
  m_cEncLib.setSarHeight                                         ( m_sarHeight );
  m_cEncLib.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_cEncLib.setColourPrimaries                                   ( m_colourPrimaries );
  m_cEncLib.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_cEncLib.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_cEncLib.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cEncLib.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cEncLib.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_cEncLib.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_cEncLib.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_cEncLib.setChromaSampleLocType                               ( m_chromaSampleLocType );
  m_cEncLib.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_cEncLib.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_cEncLib.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_cEncLib.setEfficientFieldIRAPEnabled                         ( m_bEfficientFieldIRAPEnabled );
  m_cEncLib.setHarmonizeGopFirstFieldCoupleEnabled               ( m_bHarmonizeGopFirstFieldCoupleEnabled );
  m_cEncLib.setSummaryOutFilename                                ( m_summaryOutFilename );
  m_cEncLib.setSummaryPicFilenameBase                            ( m_summaryPicFilenameBase );
  m_cEncLib.setSummaryVerboseness                                ( m_summaryVerboseness );
  m_cEncLib.setIMV                                               ( m_ImvMode );
  m_cEncLib.setIMV4PelFast                                       ( m_Imv4PelFast );
  m_cEncLib.setDecodeBitstream                                   ( 0, m_decodeBitstreams[0] );
  m_cEncLib.setDecodeBitstream                                   ( 1, m_decodeBitstreams[1] );
  m_cEncLib.setSwitchPOC                                         ( m_switchPOC );
  m_cEncLib.setSwitchDQP                                         ( m_switchDQP );
  m_cEncLib.setFastForwardToPOC                                  ( m_fastForwardToPOC );
  m_cEncLib.setForceDecodeBitstream1                             ( m_forceDecodeBitstream1 );
  m_cEncLib.setStopAfterFFtoPOC                                  ( m_stopAfterFFtoPOC );
  m_cEncLib.setBs2ModPOCAndType                                  ( m_bs2ModPOCAndType );
  m_cEncLib.setDebugCTU                                          ( m_debugCTU );
#if ENABLE_SPLIT_PARALLELISM
  m_cEncLib.setNumSplitThreads                                   ( m_numSplitThreads );
  m_cEncLib.setForceSingleSplitThread                            ( m_forceSplitSequential );
#endif

#if NN_COMMON_SPS
  m_cEncLib.setUseNnlf                                           (m_nnlf);
  m_cEncLib.setUseNnlfId                                         (m_nnlfId);
#endif
  
#if NN_LF_UNIFIED
  m_cEncLib.setUseNnlfUnified                                        (m_nnlfUnified);
  m_cEncLib.setNnlfUnifiedBlockSize                                     (m_nnlfBlockSize);
  m_cEncLib.setNnlfUnifiedInfSizeExt                                    (m_nnlfInfSizeExt);
  m_cEncLib.setNnlfUnifiedMaxNumPrms                                    (m_nnlfMaxNumPrms);
  m_cEncLib.setNnlfUnifiedModelName(m_nnlfModelName);
#if JVET_AK0093_NON_NORMATIVE_TDO
  m_cEncLib.setUseNnlfTDO(m_nnlfTDO);
  if (m_nnlfTDO)
  {
    m_cEncLib.setTDOParam(m_nnlfTDOParam);
  }
#endif
#endif
#if JVET_AH0080_TRANS_INPUT
  m_cEncLib.setUseNnlfTransInput(m_nnlfTransInput);
#endif 
  m_cEncLib.setUseALF                                            ( m_alf );
#if JVET_AL0153_ALF_CCCM
  m_cEncLib.setUseLfCccm                                         ( m_lfCccm );
#endif
#if FIXFILTER_CFG
  m_cEncLib.setUseAlfFixedFilter                                 ( m_alfFixedFilter );
#endif
  m_cEncLib.setUseCCALF                                          ( m_ccalf );
  m_cEncLib.setCCALFQpThreshold                                  ( m_ccalfQpThreshold );
  m_cEncLib.setLmcs                                              ( m_lmcsEnabled );
  m_cEncLib.setReshapeSignalType                                 ( m_reshapeSignalType );
  m_cEncLib.setReshapeIntraCMD                                   ( m_intraCMD );
  m_cEncLib.setReshapeCW                                         ( m_reshapeCW );
  m_cEncLib.setReshapeCSoffset                                   ( m_CSoffset );
#if JVET_AG0116
  m_cEncLib.setGOPBasedRPRQPThreshold                            (m_gopBasedRPRQPThreshold);
#endif
#if INTER_LIC
  m_cEncLib.setUseLIC                                            ( m_lic );
  m_cEncLib.setFastPicLevelLIC                                   ( m_lic ? m_fastPicLevelLIC : false );
#if JVET_AD0213_LIC_IMP
  m_cEncLib.setFastLic(0x00);
  m_cEncLib.setFastLicAffine(false);
  m_cEncLib.setFastLicBcw(false);
  int picSize = m_cEncLib.getSourceWidth() * m_cEncLib.getSourceHeight();
  if (m_cEncLib.getIntraPeriod() > 1)
  {
    if (picSize >= (1280 * 720))
    {
      m_cEncLib.setFastLic(0x01);
    }
    else if (picSize >= (832 * 480))
    {
      m_cEncLib.setFastLic(0x02);
      if (m_cEncLib.getTMToolsEnableFlag())
      {
        m_cEncLib.setFastLicBcw(true);
      }
    }
    else if (picSize >= (416 * 240))
    {
      m_cEncLib.setFastLic(0x03);
    }
    if (picSize < (3840 * 2160))
    {
      m_cEncLib.setFastLicAffine(true);
    }
    else
    {
      if (m_cEncLib.getTMToolsEnableFlag())
      {
        m_cEncLib.setFastLicBcw(true);
      }
    }
  }
  else if (m_cEncLib.getIntraPeriod() < 0)
  {
    if (picSize >= (1920 * 1080))
    {
      m_cEncLib.setFastLic(0x01);
    }
    else if (picSize >= (1280 * 720))
    {
      m_cEncLib.setFastLic(0x02);
    }
    else if (picSize >= (416 * 240))
    {
      m_cEncLib.setFastLic(0x03);
    }
    if (!m_cEncLib.getIBCMode())
    {
      m_cEncLib.setFastLic(m_cEncLib.getFastLic() + 0x04);
    }
    if (picSize >= (832 * 480) && picSize <= (1280 * 720))
    {
      m_cEncLib.setFastLicAffine(true);
    }
  }
#if JVET_AG0276_LIC_SLOPE_ADJUST
  m_cEncLib.setUseLicSlopeAdjust                                 ( m_lic ? m_licSlopeAdjust : false );
#endif
#endif
#endif
#if DUMP_BEFORE_INLOOP
  m_cEncLib.setDumpBeforeInloop                                  ( m_dumpBeforeInloop );
#endif
#if CONVERT_NUM_TU_SPLITS_TO_CFG
  m_cEncLib.setMaxNumTUs                                         ( m_maxNumTUs );
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  for (int i=0; i<hdrtoolslib::NB_REF_WHITE; i++)
  {
    m_cEncLib.setWhitePointDeltaE                                (i, m_whitePointDeltaE[i] );
  }
  m_cEncLib.setMaxSampleValue                                    (m_maxSampleValue);
  m_cEncLib.setSampleRange                                       (m_sampleRange);
  m_cEncLib.setColorPrimaries                                    (m_colorPrimaries);
  m_cEncLib.setEnableTFunctionLUT                                (m_enableTFunctionLUT);
  for (int i=0; i<2; i++)
  {
    m_cEncLib.setChromaLocation                                    (i, m_chromaLocation);
    m_cEncLib.setChromaUPFilter                                    (m_chromaUPFilter);
  }
  m_cEncLib.setCropOffsetLeft                                    (m_cropOffsetLeft);
  m_cEncLib.setCropOffsetTop                                     (m_cropOffsetTop);
  m_cEncLib.setCropOffsetRight                                   (m_cropOffsetRight);
  m_cEncLib.setCropOffsetBottom                                  (m_cropOffsetBottom);
  m_cEncLib.setCalculateHdrMetrics                               (m_calculateHdrMetrics);
#endif
  m_cEncLib.setGopBasedTemporalFilterEnabled(m_gopBasedTemporalFilterEnabled);
#if JVET_Y0240_BIM
  m_cEncLib.setBIM                                               (m_bimEnabled);
  if (m_cEncLib.getBIM())
  {
      std::map<int, int*> adaptQPmap;
      m_cEncLib.setAdaptQPmap(adaptQPmap);
  }
#endif
  m_cEncLib.setNumRefLayers                                       ( m_numRefLayers );

  m_cEncLib.setVPSParameters(m_cfgVPSParameters);

#if JVET_X0144_MAX_MTT_DEPTH_TID
  m_cEncLib.setMaxMTTHierarchyDepthByTid                          ( m_maxMTTHierarchyDepthByTid );
#endif

#if JVET_AB0171_ASYMMETRIC_DB_FOR_GDR
  m_cEncLib.setAsymmetricILF(m_asymmetricILF);
#endif
#if JVET_AJ0249_NEURAL_NETWORK_BASED
  m_cEncLib.setNnipMode(m_intraNN);
#endif
}

void EncApp::xCreateLib( std::list<PelUnitBuf*>& recBufList, const int layerId )
{
  // Video I/O
  m_cVideoIOYuvInputFile.open( m_inputFileName,     false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );  // read  mode
#if EXTENSION_360_VIDEO
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC);
#else
#if JVET_AA0146_WRAP_AROUND_FIX
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_sourceHeight;
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_sourceWidth - m_sourcePadding[0], sourceHeight - m_sourcePadding[1], m_InputChromaFormatIDC);
#else
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
  m_cVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], sourceHeight - m_aiPad[1], m_InputChromaFormatIDC);
#endif
#endif
  if (!m_reconFileName.empty())
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    if (m_packedYUVMode && ((m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12)
        || ((m_sourceWidth & (1 + (m_outputBitDepth[CH_L] & 3))) != 0)))
#else
    if (m_packedYUVMode && ((m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12)
        || ((m_iSourceWidth & (1 + (m_outputBitDepth[CH_L] & 3))) != 0)))
#endif
    {
      EXIT ("Invalid output bit-depth or image width for packed YUV output, aborting\n");
    }
#if JVET_AA0146_WRAP_AROUND_FIX
    if (m_packedYUVMode && (m_chromaFormatIDC != CHROMA_400) && ((m_outputBitDepth[CH_C] != 10 && m_outputBitDepth[CH_C] != 12)
        || (((m_sourceWidth / SPS::getWinUnitX (m_chromaFormatIDC)) & (1 + (m_outputBitDepth[CH_C] & 3))) != 0)))
#else
    if (m_packedYUVMode && (m_chromaFormatIDC != CHROMA_400) && ((m_outputBitDepth[CH_C] != 10 && m_outputBitDepth[CH_C] != 12)
        || (((m_iSourceWidth / SPS::getWinUnitX (m_chromaFormatIDC)) & (1 + (m_outputBitDepth[CH_C] & 3))) != 0)))
#endif
    {
      EXIT ("Invalid chroma output bit-depth or image width for packed YUV output, aborting\n");
    }

    std::string reconFileName = m_reconFileName;
    if( m_reconFileName.compare( "/dev/null" ) &&  (m_maxLayers > 1) )
    {
      size_t pos = reconFileName.find_last_of('.');
      if (pos != string::npos)
      {
        reconFileName.insert( pos, std::to_string( layerId ) );
      }
      else
      {
        reconFileName.append( std::to_string( layerId ) );
      }
    }
#if Y4M_SUPPORT
    if (isY4mFileExt(reconFileName))
    {
#if JVET_AA0146_WRAP_AROUND_FIX
      m_cVideoIOYuvReconFile.setOutputY4mInfo(m_sourceWidth - m_confWinLeft - m_confWinRight, m_sourceHeight - m_confWinTop - m_confWinBottom,
        m_iFrameRate, 1, m_internalBitDepth[0], m_chromaFormatIDC);
#else
      m_cVideoIOYuvReconFile.setOutputY4mInfo(m_iSourceWidth - m_confWinLeft - m_confWinRight, m_iSourceHeight - m_confWinTop - m_confWinBottom, 
        m_iFrameRate, 1, m_internalBitDepth[0], m_chromaFormatIDC);
#endif
    }
#endif
    m_cVideoIOYuvReconFile.open( reconFileName, true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth );  // write mode
  }

  // create the encoder
  m_cEncLib.create( layerId );
#if DUMP_BEFORE_INLOOP
  m_cEncLib.m_reconFileName = m_reconFileName;
#endif

  // create the output buffer
  for( int i = 0; i < (m_iGOPSize + 1 + (m_isField ? 1 : 0)); i++ )
  {
    recBufList.push_back( new PelUnitBuf );
  }
}

void EncApp::xDestroyLib()
{
  // Video I/O
  m_cVideoIOYuvInputFile.close();
  m_cVideoIOYuvReconFile.close();

  // Neo Decoder
  m_cEncLib.destroy();
}

void EncApp::xInitLib(bool isFieldCoding)
{
  m_cEncLib.init(isFieldCoding, this );
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncApp::createLib( const int layerIdx )
{
#if JVET_AA0146_WRAP_AROUND_FIX
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_sourceHeight;
  UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_sourceWidth, sourceHeight ) );
#else
  const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
  UnitArea unitArea( m_chromaFormatIDC, Area( 0, 0, m_iSourceWidth, sourceHeight ) );
#endif

  m_orgPic = new PelStorage;
  m_trueOrgPic = new PelStorage;
  m_orgPic->create( unitArea );
  m_trueOrgPic->create( unitArea );

#if JVET_AG0116
  if (m_resChangeInClvsEnabled && m_gopBasedRPREnabledFlag)
  {
      UnitArea unitAreaRPR10(m_chromaFormatIDC, Area(0, 0, m_sourceWidth, sourceHeight));
      UnitArea unitAreaRPR20(m_chromaFormatIDC, Area(0, 0, m_sourceWidth / 2, sourceHeight / 2));
      m_rprPic[0] = new PelStorage;
      m_rprPic[0]->create(unitAreaRPR10);
      m_rprPic[1] = new PelStorage;
      m_rprPic[1]->create(unitAreaRPR20);
  }
#endif
  if( !m_bitstream.is_open() )
  {
    m_bitstream.open( m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
    if( !m_bitstream )
    {
      EXIT( "Failed to open bitstream file " << m_bitstreamFileName.c_str() << " for writing\n" );
    }
  }

  // initialize internal class & member variables and VPS
  xInitLibCfg();
  const int layerId = m_cEncLib.getVPS() == nullptr ? 0 : m_cEncLib.getVPS()->getLayerId( layerIdx );
  xCreateLib( m_recBufList, layerId );
  xInitLib( m_isField );

  printChromaFormat();

#if EXTENSION_360_VIDEO
  m_ext360 = new TExt360AppEncTop( *this, m_cEncLib.getGOPEncoder()->getExt360Data(), *( m_cEncLib.getGOPEncoder() ), *m_orgPic );
#endif

#if JVET_Y0240_BIM
  if( m_gopBasedTemporalFilterEnabled || m_bimEnabled )
#else
  if( m_gopBasedTemporalFilterEnabled )
#endif
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    m_cEncLib.getTemporalFilter().init(m_FrameSkip, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth, m_sourceWidth,
                           sourceHeight, m_sourcePadding, m_bClipInputVideoToRec709Range, m_inputFileName,
                           m_chromaFormatIDC, m_inputColourSpaceConvert, m_iQP, m_gopBasedTemporalFilterStrengths,
                           m_gopBasedTemporalFilterPastRefs, m_gopBasedTemporalFilterFutureRefs, m_firstValidFrame,
                           m_lastValidFrame
#if JVET_Y0240_BIM
                           , m_gopBasedTemporalFilterEnabled, m_cEncLib.getAdaptQPmap(), m_cEncLib.getBIM(), m_uiCTUSize
#endif
                           );
#else
    m_cEncLib.getTemporalFilter().init(m_FrameSkip, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth, m_iSourceWidth,
                           sourceHeight, m_aiPad, m_bClipInputVideoToRec709Range, m_inputFileName,
                           m_chromaFormatIDC, m_inputColourSpaceConvert, m_iQP, m_gopBasedTemporalFilterStrengths,
                           m_gopBasedTemporalFilterPastRefs, m_gopBasedTemporalFilterFutureRefs, m_firstValidFrame,
                           m_lastValidFrame
#if JVET_Y0240_BIM
                           , m_gopBasedTemporalFilterEnabled, m_cEncLib.getAdaptQPmap(), m_cEncLib.getBIM(), m_uiCTUSize
#endif
                           );
#endif
  }
}

void EncApp::destroyLib()
{
  printf( "\nLayerId %2d", m_cEncLib.getLayerId() );

  m_cEncLib.printSummary( m_isField );

  // delete used buffers in encoder class
  m_cEncLib.deletePicBuffer();

  for( auto &p : m_recBufList )
  {
    delete p;
  }
  m_recBufList.clear();

  xDestroyLib();

  if( m_bitstream.is_open() )
  {
    m_bitstream.close();
  }

  m_orgPic->destroy();
  m_trueOrgPic->destroy();
  delete m_trueOrgPic;
  delete m_orgPic;
#if JVET_AG0116
  if (m_resChangeInClvsEnabled && m_gopBasedRPREnabledFlag)
  {
    for (int i = 0; i < 2; i++)
    {
      m_rprPic[i]->destroy();
      delete m_rprPic[i];
  }
}
#endif
#if JVET_Y0240_BIM
  if ( m_bimEnabled )
  {
    auto map = m_cEncLib.getAdaptQPmap();
    for (auto it = map->begin(); it != map->end(); ++it)
    {
      int *p = it->second;
      delete p;
    }
  }
#endif
#if EXTENSION_360_VIDEO
  delete m_ext360;
#endif

  printRateSummary();
}

bool EncApp::encodePrep( bool& eos )
{
  // main encoder loop
  const InputColourSpaceConversion ipCSC = m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC = ( !m_snrInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

#if JVET_AN0090_ADAPTIVE_SUBSAMPLING_FILTER_SELECTION
  if (m_cEncLib.getHorCollocatedChromaFlag() < 0)
  {
    bool read = m_cVideoIOYuvInputFile.readFirstPOC( *m_orgPic, *m_trueOrgPic, ipCSC, m_sourcePadding, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
    if (read && !m_cVideoIOYuvInputFile.isEof())
    {
      m_cEncLib.deriveHorCollocatedChroma(m_orgPic);
    }
    else
    {
      m_cEncLib.setHorCollocatedChromaFlag(1);
    }
  }
#endif
  // read input YUV file
#if EXTENSION_360_VIDEO
  if( m_ext360->isEnabled() )
  {
    m_ext360->read( m_cVideoIOYuvInputFile, *m_orgPic, *m_trueOrgPic, ipCSC );
  }
  else
  {
#if JVET_AA0146_WRAP_AROUND_FIX
    m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_sourcePadding, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#else
    m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#endif
  }
#else
#if JVET_AA0146_WRAP_AROUND_FIX
  m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_sourcePadding, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#else
  m_cVideoIOYuvInputFile.read( *m_orgPic, *m_trueOrgPic, ipCSC, m_aiPad, m_InputChromaFormatIDC, m_bClipInputVideoToRec709Range );
#endif
#endif

  // increase number of received frames
  m_iFrameRcvd++;

  eos = ( m_isField && ( m_iFrameRcvd == ( m_framesToBeEncoded >> 1 ) ) ) || ( !m_isField && ( m_iFrameRcvd == m_framesToBeEncoded ) );

  // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
  if( m_cVideoIOYuvInputFile.isEof() )
  {
    m_flush = true;
    eos = true;
    m_iFrameRcvd--;
    m_cEncLib.setFramesToBeEncoded( m_iFrameRcvd );
  }

  bool keepDoing = false;

  // call encoding function for one frame
  if( m_isField )
  {
    keepDoing = m_cEncLib.encodePrep( eos, m_flush ? 0 : m_orgPic, snrCSC, m_recBufList, m_numEncoded, m_isTopFieldFirst );
  }
  else
  {
    keepDoing = m_cEncLib.encodePrep(eos, m_flush ? 0 : m_orgPic, snrCSC, m_recBufList, m_numEncoded
#if JVET_AG0116
      , m_rprPic
#endif
    );
  }

  return keepDoing;
}

bool EncApp::encode()
{
  const InputColourSpaceConversion snrCSC = ( !m_snrInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  bool keepDoing = false;

  // call encoding function for one frame
  if( m_isField )
  {
    keepDoing = m_cEncLib.encode( snrCSC, m_recBufList, m_numEncoded, m_isTopFieldFirst );
  }
  else
  {
    keepDoing = m_cEncLib.encode( snrCSC, m_recBufList, m_numEncoded );
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cEncLib.getMetricTime();
#endif

  // output when the entire GOP was proccessed
  if( !keepDoing )
  {
    // write bistream to file if necessary
    if( m_numEncoded > 0 )
    {
      xWriteOutput( m_numEncoded, m_recBufList );
    }
    // temporally skip frames
    if( m_temporalSubsampleRatio > 1 )
    {
#if EXTENSION_360_VIDEO
      m_cVideoIOYuvInputFile.skipFrames( m_temporalSubsampleRatio - 1, m_inputFileWidth, m_inputFileHeight, m_InputChromaFormatIDC );
#else
#if JVET_AA0146_WRAP_AROUND_FIX
    const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_sourceHeight;
    m_cVideoIOYuvInputFile.skipFrames( m_temporalSubsampleRatio - 1, m_sourceWidth - m_sourcePadding[0], sourceHeight - m_sourcePadding[1], m_InputChromaFormatIDC );
#else
    const int sourceHeight = m_isField ? m_iSourceHeightOrg : m_iSourceHeight;
    m_cVideoIOYuvInputFile.skipFrames( m_temporalSubsampleRatio - 1, m_iSourceWidth - m_aiPad[0], sourceHeight - m_aiPad[1], m_InputChromaFormatIDC );
#endif
#endif
    }
  }

  return keepDoing;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param iNumEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */
void EncApp::xWriteOutput( int iNumEncoded, std::list<PelUnitBuf*>& recBufList )
{
  const InputColourSpaceConversion ipCSC = (!m_outputInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  std::list<PelUnitBuf*>::iterator iterPicYuvRec = recBufList.end();
  int i;

  for ( i = 0; i < iNumEncoded; i++ )
  {
    --iterPicYuvRec;
  }

  if (m_isField)
  {
    //Reinterlace fields
    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      const PelUnitBuf*  pcPicYuvRecTop     = *(iterPicYuvRec++);
      const PelUnitBuf*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (!m_reconFileName.empty())
      {
        m_cVideoIOYuvReconFile.write( *pcPicYuvRecTop, *pcPicYuvRecBottom,
                                      ipCSC,
                                      false, // TODO: m_packedYUVMode,
                                      m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
      }
    }
  }
  else
  {
    for ( i = 0; i < iNumEncoded; i++ )
    {
      const PelUnitBuf* pcPicYuvRec = *(iterPicYuvRec++);
      if (!m_reconFileName.empty())
      {
#if JVET_AC0096
        const SPS& sps = *m_cEncLib.getSPS(0);
        int ppsID = 0;
#if JVET_AG0116
        if (m_rprFunctionalityTestingEnabledFlag || m_gopBasedRPREnabledFlag)
#else
        if (m_rprFunctionalityTestingEnabledFlag)
#endif
        {
          const PPS& pps1 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR);
          const PPS& pps2 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR2);
          const PPS& pps3 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR3);
          if (pps1.getPicWidthInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).width && pps1.getPicHeightInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR;
          }
          else if (pps2.getPicWidthInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).width && pps2.getPicHeightInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR2;
          }
          else if (pps3.getPicWidthInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).width && pps3.getPicHeightInLumaSamples() == pcPicYuvRec->get(COMPONENT_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR3;
          }
          else
          {
            ppsID = 0;
          }
        }
        else
        {
          ppsID = (sps.getMaxPicWidthInLumaSamples() != pcPicYuvRec->get(COMPONENT_Y).width || sps.getMaxPicHeightInLumaSamples() != pcPicYuvRec->get(COMPONENT_Y).height) ? ENC_PPS_ID_RPR : 0;
        }
        const PPS& pps = *m_cEncLib.getPPS(ppsID);
#endif
        if( m_cEncLib.isResChangeInClvsEnabled() && m_cEncLib.getUpscaledOutput() )
        {
#if !JVET_AC0096
          const SPS& sps = *m_cEncLib.getSPS( 0 );
          const PPS& pps = *m_cEncLib.getPPS( ( sps.getMaxPicWidthInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).width || sps.getMaxPicHeightInLumaSamples() != pcPicYuvRec->get( COMPONENT_Y ).height ) ? ENC_PPS_ID_RPR : 0 );
#endif
#if JVET_AB0082
          m_cVideoIOYuvReconFile.writeUpscaledPicture(sps, pps, *pcPicYuvRec, ipCSC, m_packedYUVMode, m_cEncLib.getUpscaledOutput(), NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range, m_upscaleFilterForDisplay);
#else
          m_cVideoIOYuvReconFile.writeUpscaledPicture( sps, pps, *pcPicYuvRec, ipCSC, m_packedYUVMode, m_cEncLib.getUpscaledOutput(), NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#endif
        }
        else
        {
#if JVET_AC0096
          Window confWindowPPS = pps.getConformanceWindow();
          m_cVideoIOYuvReconFile.write(
            pcPicYuvRec->get(COMPONENT_Y).width, pcPicYuvRec->get(COMPONENT_Y).height, *pcPicYuvRec, ipCSC,
            m_packedYUVMode, confWindowPPS.getWindowLeftOffset() * SPS::getWinUnitX(m_cEncLib.getChromaFormatIdc()),
            confWindowPPS.getWindowRightOffset() * SPS::getWinUnitX(m_cEncLib.getChromaFormatIdc()),
            confWindowPPS.getWindowTopOffset() * SPS::getWinUnitY(m_cEncLib.getChromaFormatIdc()),
            confWindowPPS.getWindowBottomOffset() * SPS::getWinUnitY(m_cEncLib.getChromaFormatIdc()), NUM_CHROMA_FORMAT,
            m_bClipOutputVideoToRec709Range);

#else
          m_cVideoIOYuvReconFile.write( pcPicYuvRec->get( COMPONENT_Y ).width, pcPicYuvRec->get( COMPONENT_Y ).height, *pcPicYuvRec, ipCSC, m_packedYUVMode,
            m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_bClipOutputVideoToRec709Range );
#endif
        }
      }
    }
  }
}


void EncApp::outputAU( const AccessUnit& au )
{
#if JVET_R0294_SUBPIC_HASH
  const vector<uint32_t>& stats = writeAnnexBAccessUnit(m_bitstream, au);
#else
  const vector<uint32_t>& stats = writeAnnexB(m_bitstream, au);
#endif
  rateStatsAccum(au, stats);
  m_bitstream.flush();
}


/**
 *
 */
void EncApp::rateStatsAccum(const AccessUnit& au, const std::vector<uint32_t>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<uint32_t>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
    case NAL_UNIT_DCI:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
    case NAL_UNIT_PH:
    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_SUFFIX_APS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void EncApp::printRateSummary()
{
  double time = (double) m_iFrameRcvd / m_iFrameRate * m_temporalSubsampleRatio;
  msg( DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if (m_summaryVerboseness > 0)
  {
    msg(DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
  }
}

void EncApp::printChromaFormat()
{
  if( g_verbosity >= DETAILS )
  {
    std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
    switch (m_InputChromaFormatIDC)
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << std::endl;

    std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
    switch (m_cEncLib.getChromaFormatIdc())
    {
    case CHROMA_400:  std::cout << "  4:0:0"; break;
    case CHROMA_420:  std::cout << "  4:2:0"; break;
    case CHROMA_422:  std::cout << "  4:2:2"; break;
    case CHROMA_444:  std::cout << "  4:4:4"; break;
    default:
      THROW( "invalid chroma fomat");
    }
    std::cout << "\n" << std::endl;
  }
}

//! \}
