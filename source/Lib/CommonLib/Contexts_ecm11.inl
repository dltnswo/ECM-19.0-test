#pragma once
#include "CommonDef.h"

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
#if JVET_AF0133_RETRAINING_ISLICE_CTX
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
  ({
     {  11,  20,  44,   4,   6,  30,  20,  15,  31, },
     {  11,  20,  52,  12,  21,  30,  13,  15,  31, },
     {  27,  36,  38,  20,  29,  30,  28,  23,  31, },
     {   5,   9,   4,   9,  13,  12,   9,   9,  12, },
     {  12,  12,   4,  13,  13,  12,   9,   9,  12, },
     {  12,   9,  13,   9,   9,  12,   5,   5,   9, },
     {   4,  11,  11,  11,   4,  11,   4,   4,  11, },
     {  11,  18,  11,  18,  11,   4,  11,   4,   4, },
     {  18,   4,  32,  18,   4,   4,  11,   4,  11, },
     { 131, 134, 142, 133, 238, 238, 110, 116, 137, },
     { 140, 237, 101, 202, 236, 227, 121, 125,  84, },
     });

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
  ({
     {  19,  14,  23,  11,  12,  13, },
     {   5,   7,  23,  11,  12,  13, },
     {  11,   5,  14,  18,  27,  22, },
     {   6,  12,  12,  12,   8,   8, },
     {   5,   8,   9,  12,  12,   8, },
     {   0,   6,   6,  12,  12,  13, },
     {   4,   4,  11,  11,   4,  18, },
     {   4,   4,   4,   4,  11,   4, },
     {   4,  18,  18,  18,  18,  32, },
     { 176, 107, 117, 124, 182, 148, },
     { 138, 238, 235, 238, 235, 181, },
     });

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
  ({
     {  43,  35,  44,  42,  37, },
     {  36,  35,  44,  34,  45, },
     {  43,  50,  29,  27,  52, },
     {  10,   9,   9,   8,   5, },
     {  10,   9,  13,   9,   6, },
     {   9,   9,  10,   5,   5, },
     {   4,  11,   4,  11,  11, },
     {  11,   4,  11,  11,   4, },
     {  18,  18,  18,  11,  11, },
     { 120, 119, 148, 211, 125, },
     { 133, 137, 157, 117, 119, },
     });

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
  ({
     {  36,  37,  21,  22, },
     {  36,  37,  36,  22, },
     {  44,  45,  44,  45, },
     {  12,  13,  12,  13, },
     {  12,  13,  12,  13, },
     {  12,  13,   8,  13, },
     {  11,  11,  11,   4, },
     {   4,   4,   4,   4, },
     {  18,  11,   4,  11, },
     { 237, 139, 186, 120, },
     { 102,  92, 110, 107, },
     });

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
  ({
    {   25, 20 },
    {   25, 12 },
    {   35, 35 },
    {   1,   0 },
    {   1,   0 },
    {   1,   0 }
    { DWE, DWE },
    { DWE, DWE },
    { DWE, DWE },
    { DWO, DWO },
    { DWO, DWO }
  });
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
  ({
     {  56,  59,  60, },
     {  57,  59,  60, },
     {   0,  41,  29, },
     {   6,   6,  10, },
     {   6,   6,   9, },
     {   8,   8,  10, },
     {  18,  18,  18, },
     {  18,  11,  11, },
     {  25,  18,  11, },
     { 110, 117, 119, },
     { 123, 142, 228, },
     });

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
  ({
     {  14, },
     {   7, },
     {   5, },
     {   6, },
     {   6, },
     {   5, },
     {  18, },
     {  11, },
     {  11, },
     { 133, },
     { 105, },
     });

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  23,  14, },
     {  31,   6, },
     { CNU, CNU, },
     {   6,   5, },
     {   6,   6, },
     { DWS, DWS, },
     {   4,  18, },
     {   4,  18, },
     { DWE, DWE, },
     { 131, 110, },
     { 131, 117, },
     });

#if JVET_AG0164_AFFINE_GPM
const CtxSet ContextSetCfg::AffineGPMFlag = ContextSetCfg::addCtxSet
  ({
     {  19,   6,   7, },
     {  19,   5,   6, },
     { CNU, CNU, CNU, },
     {   5,   1,   0, },
     {   5,   1,   1, },
     { DWS, DWS, DWS, },
     {  11,  11,   4, },
     {  11,  11,  18, },
     { DWE, DWE, DWE, },
     { 116, 119, 148, },
     { 117, 117, 116, },
	 });

const CtxSet ContextSetCfg::GpmMergeIdx = ContextSetCfg::addCtxSet
   ({
     {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
     {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
     {  33,  42,  43,  27,  18, CNU, CNU, CNU, CNU, CNU, },
     {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
     {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
     {   6,   7,   6,  13,  12, DWS, DWS, DWS, DWS, DWS, },
     {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
     { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
     { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
	 });

const CtxSet ContextSetCfg::GpmAffMergeIdx = ContextSetCfg::addCtxSet
   ({
     {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
     {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
     {  33,  42,  43,  27,  18, CNU, CNU, CNU, CNU, CNU, },
     {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
     {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
     {   6,   7,   6,  13,  12, DWS, DWS, DWS, DWS, DWS, },
     {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
     { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
     { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
	 });
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
const CtxSet ContextSetCfg::GeoBlendFlag = ContextSetCfg::addCtxSet
   ({
     {  CNU, },
     {  CNU, },
     {  CNU, },
     {  DWS, },
     {  DWS, },
     {  DWS, },
     {  DWE, },
     {  DWE, },
     {  DWE, },
     {  DWO, },
     {  DWO, },
	 });
#endif

#if JVET_AG0135_AFFINE_CIIP 
const CtxSet ContextSetCfg::CiipAffineFlag = ContextSetCfg::addCtxSet
  ({
     {  12,   6,  15, },
     {  19,   5,   6, },
     { CNU, CNU, CNU, },
     {   5,   1,   0, },
     {   5,   1,   8, },
     { DWS, DWS, DWS, },
     {  11,  11,   4, },
     {   4,   4,  11, },
     {  18,  18,  18, },
     { 108, 122, 133, },
     { 116, 117, 116, },
	 });
#endif

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
      {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
      {  34,  35,  43,  35,  19, CNU, CNU, CNU, CNU, CNU, },
      {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
      {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
      {   5,   5,   8,  12,  12, DWS, DWS, DWS, DWS, DWS, },
      {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
      {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
      {  11,   4,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
      { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
      { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
#else
    {  26 },
      {   5 },
      {  33 },
      {   5 },
      {   6 },
      {   6 },
      {  18 },
      {  18 },
      {  18 },
      { 119 },
      { 117 },
#endif
  });

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    {  20,  43,  42, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
      {  21,  20,  42, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
      {  36,  36,  36,  36,  43, CNU, CNU, CNU, CNU, CNU, },
      {   4,   4,   4, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
      {   4,   5,   5, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
      {   4,   8,  12,  10,   9, DWS, DWS, DWS, DWS, DWS, },
      {  18,  18,  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
      {  18,  18,  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
      {   4,  11,  18,   4,   4, DWE, DWE, DWE, DWE, DWE, },
      { 135, 118, 125, 119, 119, 119, 119, 119, 119, 119, },
      { 116, 121, 119, 119, 119, 119, 119, 119, 119, 119, },
#else
    {  20 },
      {  21 },
      { CNU },
      {   4 },
      {   4 },
      { DWS },
      {  18 },
      {  18 },
      { DWE },
      { 135 },
      { 116 },
#endif
  });
#endif

#if JVET_Y0065_GPM_INTRA
const CtxSet ContextSetCfg::GPMIntraFlag = ContextSetCfg::addCtxSet
  ({
     {  19, },
     {  26, },
     { CNU, },
     {   6, },
     {   6, },
     { DWS, },
     {  18, },
     {  18, },
     { DWE, },
     { 196, },
     { 121, },
     });
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    {  18,  18,  18,  18, },
      {  18,  18,  18,  18, },
      { CNU, CNU, CNU, CNU, },
      {   6,   6,   6,   6, },
      {   5,   5,   5,   5, },
      { DWS, DWS, DWS, DWS, },
      {  18,  18,  18,  18, },
      {  18,  18,  18,  18, },
      { DWE, DWE, DWE, DWE, },
      { 117, 117, 117, 117, },
      { 117, 117, 117, 117, },
#else
    {  18, },
      {  18, },
      { CNU, },
      {   6, },
      {   5, },
      { DWS, },
      {  18, },
      {  18, },
      { DWE, },
      { 117, },
      { 117, },
#endif
  });

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    {  58,  58,  58,  58, },
      {  43,  43,  43,  43, },
      { CNU, CNU, CNU, CNU, },
      {   9,   9,   9,   9, },
      {  10,  10,  10,  10, },
      { DWS, DWS, DWS, DWS, },
      {  18,  18,  18,  18, },
      {  11,  11,  11,  11, },
      { DWE, DWE, DWE, DWE, },
      { 116, 116, 116, 116, },
      { 118, 118, 118, 118, },
#else
    {  58, },
      {  43, },
      { CNU, },
      {   9, },
      {  10, },
      { DWS, },
      {  18, },
      {  11, },
      { DWE, },
      { 116, },
      { 118, },
#endif
  });

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    {  43,  36,  36,  28,  35, },
      {  28,  21,  36,  28,  43, },
      { CNU, CNU, CNU, CNU, CNU, },
      {   5,   4,   4,   4,   4, },
      {   5,   5,   5,   5,   4, },
      { DWS, DWS, DWS, DWS, DWS, },
      {  18,  11,  11,  11,  11, },
      {  18,  18,  18,  18,  11, },
      { DWE, DWE, DWE, DWE, DWE, },
      { 142, 157, 157, 142, 155, },
      { 116, 103, 104, 108, 117, },
#else
    {  59 },
      {  60 },
      {  35 },
      {   0 },
      {   0 },
      {   0 },
      { DWE },
      { DWE },
      { DWE },
      { DWO },
      { DWO },
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
  ({
     {  59 },
     {  60 },
     { CNU },
     {   0 },
     {   0 },
     { DWS },
     { DWE },
     { DWE },
     { DWE },
     { DWO },
     { DWO },
     });
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  33, },
     {  26, },
     { CNU, },
     {   3, },
     {   2, },
     { DWS, },
     {   4, },
     {   4, },
     { DWE, },
     { 109, },
     { 115, },
     });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
     {  59, },
     {  60, },
     { CNU, },
     {   1, },
     {   1, },
     { DWS, },
     {  11, },
     {  11, },
     { DWE, },
     { 126, },
     { 122, },
     });
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
const CtxSet ContextSetCfg::GeoBldFlag = ContextSetCfg::addCtxSet
  ({
     { 59,  59,  59,  59 },
     { 60,  60,  60,  60 },
     { CNU, CNU, CNU, CNU },
     { 1,   1,   1,   1 },
     { 1,   1,   1,   1 },
     { DWS, DWS, DWS, DWS },
     { 11,  11,  11,  11 },
     { 11,  11,  11,  11 },
     { DWE, DWE, DWE, DWE },
     { 126, 126, 126, 126 },
     { 122, 122, 122, 122 },
     });
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
  ({
    {  33,  28,  36,  36,  29, },
    {  20,  21,  29,  29,  29, },
    { CNU, CNU, CNU, CNU, CNU, },
    {   4,   5,   4,   4,   8, },
    {   5,   5,   5,   4,   8, },
    { DWS, DWS, DWS, DWS, DWS, },
    { DWE, DWE, DWE, DWE, DWE, },
    { DWE, DWE, DWE, DWE, DWE, },
    { DWE, DWE, DWE, DWE, DWE, },
    { DWO, DWO, DWO, DWO, DWO, },
    { DWO, DWO, DWO, DWO, DWO, }
  });
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  11, },
     {   4, },
     { CNU, },
     {   5, },
     {   5, },
     { DWS, },
     {  18, },
     {  25, },
     { DWE, },
     { 119, },
     { 132, },
     });

const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    { CNU, CNU,CNU,CNU},
      { CNU, CNU,CNU,CNU},
      { CNU, CNU,CNU,CNU},
      { DWS, DWS,DWS,DWS},
      { DWS, DWS,DWS,DWS},
      { DWS, DWS,DWS,DWS},
      { DWE, DWE,DWE,DWE},
      { DWE, DWE,DWE,DWE},
      { DWE, DWE,DWE,DWE},
      { 119, 119,119,119},
      { 119, 119,119,119},
#else
    { CNU, },
      { CNU, },
      { CNU, },
      { DWS, },
      { DWS, },
      { DWS, },
      { DWE, },
      { DWE, },
      { DWE, },
      { 119, },
      { 119, },
#endif
  });

const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    {  21,  29,  29,  29,  44, CNU},
      {  13,  21,  36,  29,  44, CNU},
      { CNU, CNU, CNU, CNU, CNU, CNU},
      {   5,   5,   5,   4,   5, DWS},
      {   5,   5,   4,   5,   9, DWS},
      { DWS, DWS, DWS, DWS, DWS, DWS},
      {  18,  18,  18,  11,  11, DWE},
      {  18,  18,  11,  18,  25, DWE},
      { DWE, DWE, DWE, DWE, DWE, DWE},
      { 126, 141, 187, 219, 238, DWO},
      { 117, 102, 100, 100,  87, DWO},
#else
    {  21,  29,  29,  29,  44, },
      {  13,  21,  36,  29,  44, },
      { CNU, CNU, CNU, CNU, CNU, },
      {   5,   5,   5,   4,   5, },
      {   5,   5,   4,   5,   9, },
      { DWS, DWS, DWS, DWS, DWS, },
      {  18,  18,  18,  11,  11, },
      {  18,  18,  11,  18,  25, },
      { DWE, DWE, DWE, DWE, DWE, },
      { 126, 141, 187, 219, 238, },
      { 117, 102, 100, 100,  87, },
#endif
#else
    {  21 },
      {  13 },
      { CNU },
      {   5 },
      {   5 },
      { DWS },
      {  18 },
      {  18 },
      { DWE },
      { 126 },
      { 117 },
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
  ({
     {  21 },
     {  13 },
     { CNU },
     {   5 },
     {   5 },
     { DWS },
     {  18 },
     {  18 },
     { DWE },
     { 126 },
     { 117 },
     });
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AE0169_BIPREDICTIVE_IBC
    {  18,  35, },
      {  18,  35, },
      {  36,  44, },
      {   6,   6, },
      {   5,   5, },
      {   4,   8, },
      {  18,  18, },
      {  18,  18, },
      {   4,  11, },
      { 117, 119, },
      { 117, 119, },
#else
    {  18, },
      {  18, },
      { CNU, },
      {   6, },
      {   5, },
      { DWS, },
      {  18, },
      {  18, },
      { DWE, },
      { 117, },
      { 117, },
#endif
  });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
  ({
     {  58, },
     {  43, },
     {  43, },
     {   9, },
     {  10, },
     {   9, },
     {  18, },
     {  11, },
     {  25, },
     { 116, },
     { 118, },
     });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
     {  43,  36,  36,  28,  35, },
     {  28,  21,  36,  28,  43, },
     {  35,  43,  43, CNU, CNU, },
     {   5,   4,   4,   4,   4, },
     {   5,   5,   5,   5,   4, },
     {   8,   9,  13, DWS, DWS, },
     {  18,  11,  11,  11,  11, },
     {  18,  18,  18,  18,  11, },
     {   4,  11,  18, DWE, DWE, },
     { 142, 157, 157, 142, 155, },
     { 116, 103, 104, 108, 117, },
     });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    {  25,  33, },
      {  33,  25, },
      { CNU,  33, },
      {   5,   5, },
      {   5,   5, },
      { DWS,   5, },
      {  18,  18, },
      {  18,  18, },
      { DWE,  18, },
      { 119, 119, },
      { 125, 125, },
#else
    {  25, },
      {  33, },
      { CNU, },
      {   5, },
      {   5, },
      { DWS, },
      {  18, },
      {  18, },
      { DWE, },
      { 119, },
      { 125, },
#endif
  });
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  18, },
     {  26, },
     { CNU, },
     {   5, },
     {   5, },
     { DWS, },
     {  18, },
     {  11, },
     { DWE, },
     { 141, },
     { 118, },
     });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
  ({
     {  25,  21, },
     {  40,  28, },
     { CNU, CNU, },
     {   6,   2, },
     {   6,   2, },
     { DWS, DWS, },
     {  18,  18, },
     {  18,  18, },
     { DWE, DWE, },
     { 106, 119, },
     { 117, 118, },
     });

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
    {  18,  51,  51,  51,  43,  25,  59, },
      {  25,  50,  50,  43,  50,  25,  58, },
      { CNU, CNU, CNU, CNU, CNU,  25,  60, },
      {   6,   4,   5,   5,   6,   6,   4, },
      {   6,   4,   5,   4,   5,   6,   4, },
      { DWS, DWS, DWS, DWS, DWS,   6,   8, },
      {  18,  11,  11,  18,  18,  11,  11, },
      {  11,  11,  18,   4,  11,  11,  18, },
      { DWE, DWE, DWE, DWE, DWE,  18,  32, },
      { 123, 115, 109, 101, 100, 124, 117, },
      { 146, 150, 138, 139, 168, 121, 148, },
#else
    {  18,  51,  51,  51,  43 },
      {  25,  50,  50,  43,  50 },
      {  25,  59,  52,  44,  59 },
      {   6,   4,   5,   5,   6 },
      {   6,   4,   5,   4,   5 },
      {   6,   4,   8,   4,   4 },
      {  18,  11,  11,  18,  18 },
      {  11,  11,  18,   4,  11 },
      {  18,   4,  18,   4,   4 },
      { 123, 115, 109, 101, 100 },
      { 146, 150, 138, 139, 168 },
#endif
#else
#if JVET_W0123_TIMD_FUSION
    {  18,  51,  51,  51 },
      {  25,  50,  50,  43 },
      {  25,  59,  52,  44 },
      {   6,   4,   5,   5 },
      {   6,   4,   5,   4 },
      {   6,   4,   8,   4 },
      {  18,  11,  11,  18 },
      {  11,  11,  18,   4 },
      {  18,   4,  18,   4 },
      { 123, 115, 109, 101 },
      { 146, 150, 138, 139 },
#else
    {  18,  51 },
      {  25,  50 },
      {  25,  59 },
      {   6,   4 },
      {   6,   4 },
      {   6,   4 },
      {  18,  11 },
      {  11,  11 },
      {  18,   4 },
      { 123, 115 },
      { 146, 150 },
#endif
#endif
  });

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
  ({
     {  44, },
     {  29, },
     {  22, },
     {   6, },
     {   6, },
     {   6, },
     {  11, },
     {  11, },
     {  18, },
     { 117, },
     { 119, },
     });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
  ({
     {  36, },
     {  36, },
     {  29, },
     {   6, },
     {  10, },
     {   6, },
     {   4, },
     {  11, },
     {  11, },
     { 119, },
     { 126, },
     });

const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
  ({
     {  28,  44,  28, },
     {  21,  37,  28, },
     {  27, CNU,  42, },
     {   1,   2,   6, },
     {   6,   2,   6, },
     {   2, DWS,   6, },
     {  11,  11,  18, },
     {  11,   4,  11, },
     {   4, DWE,  11, },
     { 121, 126, 119, },
     { 119, 109, 121, },
     });

#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU, },
     { CNU, CNU, CNU, CNU, CNU, },
     {  51,  35,  35, CNU, CNU, },
     { DWS, DWS, DWS, DWS, DWS, },
     { DWS, DWS, DWS, DWS, DWS, },
     {   9,   9,   9, DWS, DWS, },
     { DWE, DWE, DWE, DWE, DWE, },
     { DWE, DWE, DWE, DWE, DWE, },
     {  18,  11,  11, DWE, DWE, },
     { 119, 119, 119, 119, 119, },
     { 119, 119, 119, 119, 119, },
     });
#endif
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AC0105_DIRECTIONAL_PLANAR
    {   6,   6,  CNU,  CNU, },
      {   6,  28,  CNU,  CNU, },
      {  23,  21,   27,   28, },
      {   1,   2,  DWS,  DWS, },
      {   1,   2,  DWS,  DWS, },
      {   1,   6,    8,    8, },
      {  18,  18,  DWE,  DWE, },
      {  18,  18,  DWE,  DWE, },
      {   4,  25,   18,   18, },
      { 125, 116,  DWO,  DWO, },
      { 116, 117,  DWO,  DWO, },
#else
    {   6,   6, },
      {   6,  28, },
      {  23,  21, },
      {   1,   2, },
      {   1,   2, },
      {   1,   6, },
      {  18,  18, },
      {  18,  18, },
      {  11,  25, },
      { 125, 116, },
      { 116, 117, },
#endif
  });

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
  ({
     {  26, },
     {  26, },
     {  44, },
     {   1, },
     {   5, },
     {   5, },
     {  18, },
     {  25, },
     {  25, },
     { 117, },
     { 147, },
     });

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
  ({
     {  26, },
     {  34, },
     {  20, },
     {   5, },
     {   5, },
     {   5, },
     {  11, },
     {  11, },
     {  11, },
     { 116, },
     { 131, },
     });

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  25, },
     {  35, },
     {   5, },
     {   5, },
     {   5, },
     {  18, },
     {  11, },
     {  18, },
     { 116, },
     { 118, },
     });

#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdChromaMode = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  27, },
     { DWS, },
     { DWS, },
     {   5, },
     { DWE, },
     { DWE, },
     {  11, },
     { DWO, },
     { DWO, },
     });
#endif

const CtxSet ContextSetCfg::ChromaFusionMode = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  27, },
     { DWS, },
     { DWS, },
     {   4, },
     { DWE, },
     { DWE, },
     {   4, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AC0071_DBV
const CtxSet ContextSetCfg::DbvChromaMode = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  28, },
     { DWS, },
     { DWS, },
     {   1, },
     { DWE, },
     { DWE, },
     {  11, },
     { DWO, },
     { DWO, },
     });
#endif

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
  ({
     {  48,  49,  42,  33, },
     {  48,  49,  50,  33, },
     {  48,  41,  42,  25, },
     {   9,  10,   8,   6, },
     {   9,   9,   8,   6, },
     {  10,  10,   8,   6, },
     {  18,  18,  18,  18, },
     {  11,  11,  18,  11, },
     {  18,   4,   4,   4, },
     { 104, 118, 118, 108, },
     { 117, 118, 120, 124, },
     });

#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AD0086_ENHANCED_INTRA_TMP
    {  33,  42,   7,  33, CNU, CNU, CNU, },
      {   0,  25,  57,   0, CNU, CNU, CNU, },
      {  25,  26,  28,  25,  13,  20,  49, },
      {   1,   4,   6,   1, DWS, DWS, DWS, },
      {   8,   8,   0,  12, DWS, DWS, DWS, },
      {   6,   5,   1,   2,   5,   4,   4, },
      {  11,  32,  25,  18, DWE, DWE, DWE, },
      {   4,  25,  32,  11, DWE, DWE, DWE, },
      {  18,  18,  11,  18,  25,   4,  18, },
      {  99, 101, 133, 115, DWO, DWO, DWO, },
      { 147, 161, 114, 131, DWO, DWO, DWO, },
#else
    {  33,  42,   7,  33, },
      {   0,  25,  57,   0, },
      {  40,  19,  21,  33, },
      {   1,   4,   6,   1, },
      {   8,   8,   0,  12, },
      {   6,   5,   0,   2, },
      {  11,  32,  25,  18, },
      {   4,  25,  32,  11, },
      {  25,  32,   4,  25, },
      {  99, 101, 133, 115, },
      { 147, 161, 114, 131, },
#endif
  });
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
const CtxSet ContextSetCfg::TmpIdx = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, },
     { CNU, CNU, CNU, },
     {  20,  28,  28, },
     { DWS, DWS, DWS, },
     { DWS, DWS, DWS, },
     {   5,   8,   9, },
     { DWE, DWE, DWE, },
     { DWE, DWE, DWE, },
     {   4,  11,  11, },
     { DWO, DWO, DWO, },
     { DWO, DWO, DWO, },
     });

const CtxSet ContextSetCfg::TmpFusion = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU, CNU, },
     { CNU, CNU, CNU, CNU, CNU, CNU, },
     {  42,  43,  36,  19,  20,  42, },
     { DWS, DWS, DWS, DWS, DWS, DWS, },
     { DWS, DWS, DWS, DWS, DWS, DWS, },
     {   4,   4,   4,   5,   4,   8, },
     { DWE, DWE, DWE, DWE, DWE, DWE, },
     { DWE, DWE, DWE, DWE, DWE, DWE, },
     {  11,   4,   4,  18,   4,  18, },
     { DWO, DWO, DWO, DWO, DWO, DWO, },
     { DWO, DWO, DWO, DWO, DWO, DWO, },
     });
#endif

#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
  ({
     {  46, },
     {  46, },
     {  35, },
     {   5, },
     {   5, },
     {   5, },
     {   4, },
     {  18, },
     {  11, },
     { 109, },
     { 133, },
     });
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
  ({
     { 35, 35, },
     { 35, 35, },
     { 35, 35, },
     { 8,  8, },
     { 8,  8, },
     { 8,  8, },
     { DWE, DWE, },
     { DWE, DWE, },
     { DWE, DWE, },
     { 119, 119, },
     { 119, 119, },
     });

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
  ({
#if CTU_256
    {   7,  13,  12,   4,  18,   3,  10,   0, },
      {   7,   6,   5,   4,  11,  18,  10,  48, },
      { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
      {   0,   0,   1,   1,   5,   5,   6,   1, },
      {   1,   1,   1,   1,   5,   9,   6,   1, },
      { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
      {  18,  11,  11,   4,  18,  18,  18,  32, },
      {  32,  18,  11,   4,  11,  18,   4,  18, },
      { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
      { 116, 116, 117, 117, 117, 103,  67, 228, },
      { 116, 148, 148, 124, 148, 148, 119, 116, },
#else
    {   7,  13,  12,   4,  18,   3,  10 },
      {   7,   6,   5,   4,  11,  18,  10 },
      { CNU, CNU, CNU, CNU, CNU, CNU, CNU },
      {   0,   0,   1,   1,   5,   5,   6 },
      {   1,   1,   1,   1,   5,   9,   6 },
      { DWS, DWS, DWS, DWS, DWS, DWS, DWS },
      {  18,  11,  11,   4,  18,  18,  18 },
      {  32,  18,  11,   4,  11,  18,   4 },
      { DWE, DWE, DWE, DWE, DWE, DWE, DWE },
      { 116, 116, 117, 117, 117, 103,  67 },
      { 116, 148, 148, 124, 148, 148, 119 },
#endif
  });

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
  ({
     {  20,  28, },
     {   5,  29, },
     { CNU, CNU, },
     {   1,   1, },
     {   1,   5, },
     { DWS, DWS, },
     {  11,   4, },
     {  18,  18, },
     { DWE, DWE, },
     { 122, 125, },
     { 124, 103, },
     });

#if JVET_Z0054_BLK_REF_PIC_REORDER
const CtxSet ContextSetCfg::RefPicLC = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU },
     { CNU, CNU, CNU },
     { CNU, CNU, CNU },
     { DWS, DWS, DWS },
     { DWS, DWS, DWS },
     { DWS, DWS, DWS },
     { DWE, DWE, DWE },
     { DWE, DWE, DWE },
     { DWE, DWE, DWE },
     { DWO, DWO, DWO },
     { DWO, DWO, DWO },
     });
#endif

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  43,  30, },
     {  40,  34,  36, },
     { CNU, CNU, CNU, },
     {   6,   5,   5, },
     {   6,   5,   5, },
     { DWS, DWS, DWS, },
     {  18,  18,  18, },
     {  11,  18,  18, },
     { DWE, DWE, DWE, },
     { 121, 117, 118, },
     { 126, 133, 148, },
     });

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  48,  50,  50,  50, },
     {  56,  50,  43,  28, },
     { CNU, CNU, CNU, CNU, },
     {   5,   5,   9,   5, },
     {   5,   5,   5,   5, },
     { DWS, DWS, DWS, DWS, },
     {  18,  18,  25,  18, },
     {  18,  18,  11,  18, },
     { DWE, DWE, DWE, DWE, },
     { 126, 126, 181, 126, },
     { 117, 117, 110, 116, },
     });
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
const CtxSet ContextSetCfg::affBMFlag = ContextSetCfg::addCtxSet
  ({
     {  48,  50, },
     {  56,  50, },
     { CNU, CNU, },
     {   5,   5, },
     {   5,   5, },
     { DWS, DWS, },
     {  18,  18, },
     {  18,  18, },
     { DWE, DWE, },
     { 126, 126, },
     { 117, 117, },
     });
#endif
#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
  ({
     {  48,  50,  50,  50, },
     {  56,  50,  43,  28, },
     {  39,  39,  39, CNU, },
     {   5,   5,   9,   5, },
     {   5,   5,   5,   5, },
     {   0,   0,   0, DWS, },
     {  18,  18,  25,  18, },
     {  18,  18,  11,  18, },
     {  32,  32,  25, DWE, },
     { 126, 126, 181, 126, },
     { 117, 117, 110, 116, },
     });
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
const CtxSet ContextSetCfg::bvOneZeroComp = ContextSetCfg::addCtxSet
  ({
     {  48,  50,  50,  50, },
     {  56,  50,  43,  28, },
     {  41,  42,  27,  27, },
     {   5,   5,   9,   5, },
     {   5,   5,   5,   5, },
     {   5,   7,  12,  10, },
     {  18,  18,  25,  18, },
     {  18,  18,  11,  18, },
     {  18,   4,   4,  18, },
     { 126, 126, 181, 126, },
     { 117, 117, 110, 116, },
     });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
  ({
     {  19,   6,   7, },
     {  19,   5,   6, },
     { CNU, CNU, CNU, },
     {   5,   1,   0, },
     {   5,   1,   1, },
     { DWS, DWS, DWS, },
     {  11,  11,   4, },
     {  11,  11,  18, },
     { DWE, DWE, DWE, },
     { 116, 119, 148, },
     { 117, 117, 116, },
     });

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
  ({
     {  19, },
     {  34, },
     { CNU, },
     {   1, },
     {   1, },
     { DWS, },
     {   4, },
     {   4, },
     { DWE, },
     { 102, },
     { 211, },
     });

#if JVET_AA0128_AFFINE_MERGE_CTX_INC
const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
  ({
     {   4, CNU, CNU, },
     {   4, CNU, CNU, },
     { CNU, CNU, CNU, },
     {   1, DWS, DWS, },
     {   1, DWS, DWS, },
     { DWS, DWS, DWS, },
     {  18, DWE, DWE, },
     {  25, DWE, DWE, },
     { DWE, DWE, DWE, },
     { 118, DWO, DWO, },
     { 148, DWO, DWO, },
     });
#else
const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
  ({
     {   4, },
     {   4, },
     { CNU, },
     {   1, },
     {   1, },
     { DWS, },
     {  18, },
     {  25, },
     { DWE, },
     { 118, },
     { 148, },
     });
#endif

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
  ({
     {  12, },
     {   4, },
     { CNU, },
     {   5, },
     {   5, },
     { DWS, },
     {  11, },
     {   4, },
     { DWE, },
     { 132, },
     { 132, },
     });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
  ({
     {   4, },
     {   5, },
     { CNU, },
     {   0, },
     {   0, },
     { DWS, },
     {   4, },
     {  11, },
     { DWE, },
     { 164, },
     { 118, },
     });

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
  ({
     {  44,  36, },
     {  44,  43, },
     { CNU, CNU, },
     {   6,   3, },
     {   6,   3, },
     { DWS, DWS, },
     {   4,  11, },
     {   4,  11, },
     { DWE, DWE, },
     { 141, 116, },
     { 126,  68, },
     });

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
  ({
    {  53,  38,  38,  29,  20,  34,  27,  45,  37,  43,  34,  48, },
    {  38,  38,  38,  29,  28,  42,  27,  45,  44,  28,  42,  33, },
    {  38,  38,  38,  30,  29,  43,  21,  52,  29,  29,  28,  27, },
    {   1,  12,   8,   4,   2,   5,   3,   4,   0,   0,   5,   4, },
    {   6,  10,   9,   6,   7,   7,   5,   5,   4,   1,   2,   3, },
    {   6,  10,  10,   7,   7,   2,   1,   5,   5,   1,   3,   1, },
    { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
    { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
    {  32,  25,  25,  18,  18,  11,  11,  25,  25,  18,  18,  18, },
    { DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
    { DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, }
  });
#endif

#if JVET_AD0140_MVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxMVDMSB = ContextSetCfg::addCtxSet
  ({
     {  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41},
     {  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41},
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU},
     {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12},
     {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12},
     { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS},
     {   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4},
     {   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4},
     { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE},
     { 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94},
     { 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101},
     });
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxBVDMSB = ContextSetCfg::addCtxSet
  ({
     {  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41, 34,   41,  49,  41, },
     {  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41, 34,   41,  34,  41, },
     {  42,  41,  34,  41,  42,  34, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12, 13,   13,  12,  12, },
     {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12, 13,   13,  12,  12, },
     {  13,   7,   8,  13,  13,  10, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
     {   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4, },
     {   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4, },
     {  11,   4,   4,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
     { 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, },
     { 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, },
     });
#endif


#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdx = ContextSetCfg::addCtxSet
  ({
     {  34,  41,  49,  41, },
     {  34,  41,  34,  41, },
     { CNU, CNU, CNU, CNU, },
     {  13,  13,  12,  12, },
     {  13,  13,  12,  12, },
     { DWS, DWS, DWS, DWS, },
     {   4,   4,   4,   4, },
     {   4,   4,  18,   4, },
     { DWE, DWE, DWE, DWE, },
     { 222,  83, 181,  94, },
     { 100, 116, 100, 101, },
     });
#endif


#if JVET_AD0140_MVD_PREDICTION && JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxIBC = ContextSetCfg::addCtxSet
  ({
     {  34,  41, },
     {  34,  41, },
     {  42,  41, },
     {  13,  13, },
     {  13,  13, },
     {  13,  10, },
     {   4,   4, },
     {   4,   4, },
     {  25,   4, },
     { 222,  83, },
     { 100, 116, },
     });
#endif

#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
  ({
     {  17,  42,  51, },
     {  10,  42,  51, },
     { CNU, CNU, CNU, },
     {   2,   6,   5, },
     {   2,   5,   1, },
     { DWS, DWS, DWS, },
     {  11,  11,  18, },
     {  11,  11,   4, },
     { DWE, DWE, DWE, },
     { 116, 139, 118, },
     { 132, 109, 131, },
     });
#endif

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
  ({
     {  20,  43, },
     {  28,  19, },
     { CNU, CNU, },
     {   1,   1, },
     {   1,   0, },
     { DWS, DWS, },
     {  11,  11, },
     {  11,  11, },
     { DWE, DWE, },
     { 132, 148, },
     { 118, 117, },
     });

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
  ({
     {  35, CNU, },
     {  35, CNU, },
     { CNU, CNU, },
     {   1, DWS, },
     {   1, DWS, },
     { DWS, DWS, },
     {  11, DWE, },
     {  11, DWE, },
     { DWE, DWE, },
     { 126, 119, },
     { 126, 119, },
     });

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
  ({
     {  33,  14,   0,   4, },
     {   0,  21,   0,  12, },
     {  11,  50,   0,  11, },
     {   1,   1,   4,   3, },
     {   1,   4,   0,   8, },
     {   1,   4,   4,   3, },
     {  18,  25,  32,  32, },
     {   4,  32,  32,  32, },
     {  18,  32,  11,  11, },
     { 118, 133,  83, 156, },
     { 120, 132, 212, 147, },
     });

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
  ({
     {   5, },
     {   5, },
     {   6, },
     {   1, },
     {   1, },
     {   1, },
     {   4, },
     {   4, },
     {  11, },
     { 118, },
     { 102, },
     });

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
  ({
     { 46, },
     { 46, },
     { 52, },
     { 1, },
     { 1, },
     { 1, },
     { DWE, },
     { DWE, },
     { DWE, },
     { 119, },
     { 119, },
     });

const CtxSet ContextSetCfg::QtCbf[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  23,  30,  13,  14, },
       {  31,   6,  13,   7, },
       {   7,  19,  12,   7, },
       {   6,   1,   9,   9, },
       {   6,   0,   5,   9, },
       {   5,   1,   5,   5, },
       {  18,  18,  18,  25, },
       {  18,  18,   4,  11, },
       {  18,  18,   4,   4, },
       { 117, 118, 100, 147, },
       { 116, 116, 179, 139, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,  38, },
       {  25,   5, },
       {  12,  14, },
       {   6,   2, },
       {   6,   0, },
       {   5,   3, },
       {  18,  32, },
       {  25,  32, },
       {  18,  25, },
       { 106, 117, },
       { 131, 132, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  17,  36,  22, },
      {  17,  36,  53, },
      {  26,  28,  45, },
      {   3,   2,  13, },
      {   2,   2,   2, },
      {   1,   2,   1, },
      {  18,  18,   4, },
      {  18,  18,  25, },
      {  11,  18,  18, },
      { 116, 118,  91, },
      { 121, 126, 130, },
      }),
    };

const CtxSet ContextSetCfg::SigCoeffGroup[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  25,  38, },
       {  25,  45, },
       {  18,  31, },
       {   8,   5, },
       {   8,   5, },
       {   5,   5, },
       {  25,  18, },
       {  25,  18, },
       {  18,  18, },
       { 116, 124, },
       { 116, 119, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  17,  45, },
      {  25,  45, },
      {  25,  15, },
      {   5,   8, },
      {   9,  13, },
      {   5,   9, },
      {  18,  11, },
      {  11,  25, },
      {  11,  18, },
      {  93, 151, },
      { 116, 121, },
      }),
    };
#if JVET_AE0102_LFNST_CTX
const CtxSet ContextSetCfg::SigFlagL[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  17,  41,  42,  29,   9,  42,  28,  37,  33,  44,  51,  45, },
       {  17,  41,  42,  29,  25,  42,  43,  37,  33,  44,  51,  30, },
       {  18,  26,  27,  13,  26,  27,  21,  22,  35,  52,  37,  38, },
       {  13,   9,   9,  10,   9,  10,   9,  10,   8,   9,   9,   9, },
       {  13,   9,   9,  10,   9,  10,  10,  10,   8,   9,   9,   9, },
       {  12,   9,   9,  10,   8,  10,   9,  13,   9,   9,   9,  10, },
       {  11,  11,  18,  18,  11,  18,  18,  11,  11,  18,  18,  11, },
       {  11,  11,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
       {  32,   4,  11,   4,  11,   4,   4,  11,  18,  11,  11,   4, },
       {  90, 119, 117, 121, 123, 119, 117, 126, 126, 132, 118, 132, },
       { 120, 133, 118, 125, 117, 126, 119, 120, 117, 117, 132, 108, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,  42,  43,  29,  41,  60,  60,  38, },
       {  18,  27,  28,  29,  34,  45,  45,  38, },
       {  18,  34,  20,  29,  35,  53,  53,  38, },
       {   8,  13,   9,  13,   5,   5,   8,  10, },
       {  13,  13,  13,  13,   5,   5,   8,   9, },
       {  12,  12,   9,  13,   8,   5,   9,   9, },
       {   4,  25,  11,  11,  18,  18,  18,  18, },
       {  18,  18,  18,  11,  11,  11,  18,  18, },
       {  25,  11,   4,  18,  18,   4,   4,  11, },
       { 119, 108, 119, 103, 125, 133, 132, 118, },
       { 109, 117, 126, 124, 119, 110, 126, 137, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  26,  45,  53,  46,  19,  54,  61,  39,  34,  39,  39,  39, },
       {  26,  38,  38,  46,  34,  54,  54,  39,  13,  39,  39,  39, },
       {  27,  30,  53,  54,  28,  39,  39,  39,  29,  39,  39,  39, },
       {   9,  13,  12,   8,   8,   8,   8,   4,   0,   0,   0,   0, },
       {   9,  13,  12,   8,   8,   8,   8,   5,   0,   0,   0,   0, },
       {   9,  12,  12,  12,  12,   8,   8,   4,   0,   0,   0,   0, },
       {  11,  11,  18,  18,  18,  25,  25,  18,   4,  32,  32,  32, },
       {  11,  11,  32,  11,  18,  25,  25,  25,  32,  32,  32,  32, },
       {  18,  11,  32,  18,  32,  32,  32,  18,  25,  32,  32,  32, },
       { 118, 238, 151, 158, 148, 182, 166, 142, 131, 168, 238, 138, },
       { 126, 236, 126,  91, 117, 131, 131,  98, 228, 116, 116, 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  41,  45,  38,  31,   4,  39,  39,  39, },
       {  34,  38,  53,  54,  44,  39,  39,  39, },
       {  20,  46,  38,  39,  53,  39,  39,  39, },
       {   8,  12,  12,   8,   4,   0,   0,   0, },
       {   8,  12,  12,   8,   4,   0,   0,   0, },
       {   8,  12,  13,   8,   0,   0,   0,   0, },
       {  18,  18,  25,  25,  25,  32,  32,  32, },
       {  11,  11,  25,  25,  25,  32,  32,  32, },
       {  25,  18,  25,  25,  32,  32,  32,  32, },
       { 132, 233, 221, 197, 116, 197, 214, 116, },
       { 117, 155,  98, 102, 149,  98, 116,  99, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  26,  54,  39,  39,  34,  39,  39,  39,   0,  39,  39,  39, },
       {  19,  39,  54,  39,  19,  39,  39,  39,  48,  39,  39,  39, },
       {  26,  39,  39,  39,  35,  39,  39,  39,   0,  39,  39,  39, },
       {   8,   8,   8,  12,   8,   4,   4,   8,   4,   0,   0,   0, },
       {   8,   8,   8,  12,   8,   4,   4,   8,   0,   0,   0,   0, },
       {  10,   8,  12,  12,  13,   0,   4,   4,   0,   0,   0,   0, },
       {  18,  25,  25,  32,  18,  18,  25,  32,  32,  32,  32,  32, },
       {  18,  25,  18,  32,  18,  18,  25,  32,  25,  32,  32,  32, },
       {  25,  25,  32,  32,  25,  25,  32,  32,  25,  32,  32,  32, },
       { 119, 190, 190, 171, 132, 155, 142, 139,  82, 117, 238, 229, },
       { 119, 115, 108,  92, 117,  99,  99,  82, 134, 116, 116, 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  26,  38,  54,  39,  26,  39,  39,  39, },
      {  34,  38,  62,  39,  26,  39,  39,  39, },
      {  12,  39,  39,  39,  50,  39,  39,  39, },
      {   8,  12,   8,   8,   0,   0,   0,   0, },
      {   8,   8,   8,   8,   0,   0,   0,   0, },
      {   8,   8,   8,   8,   0,   0,   0,   0, },
      {  25,  32,  25,  25,  18,  32,  32,  32, },
      {  18,  18,  25,  25,  18,  32,  32,  32, },
      {  18,  18,  25,  25,  25,  32,  32,  32, },
      { 227, 190, 168, 158, 115, 185, 229, 196, },
      { 117, 182, 118, 116, 134, 114, 114,  99, },
      }),
    };

const CtxSet ContextSetCfg::ParFlagL[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  33,  40,  33,  26,  34,  42,  25,  33,  34,  34,  27,  25,  34,  42,  42,  35,  33,  27,  35,  42,  35, },
       {  33,  25,  33,  26,  34,  42,  25,  33,  34,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  35,  35, },
       {  25,   0,  17,  17,   2,  58,  25,  18,  11,  34,  27,  33,  19,  19,  27,  35,  34,  42,  20,  43,  20, },
       {   8,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,   9,  13,  13,  12,  13, },
       {   9,   9,   9,  12,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
       {   8,  13,  10,  12,   0,   0,  10,  12,  12,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13, },
       {   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4,  11,  18,  11,  11,   4,  11,  18,  11,  11,   4, },
       {  11,  11,   4,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4, },
       {   4,  32,  25,   4,  32,   4,   4,   4,  11,   4,   4,   4,   4,  11,   4,   4,   4,   4,   4,  11,   4, },
       {  78,  90,  91, 102, 126, 126,  89, 119, 103, 158, 119, 171, 119, 126, 125, 123, 106, 135, 120, 119, 135, },
       { 133, 147, 117, 118, 126, 134, 120, 126, 140, 120, 140, 147, 126, 121, 133, 126, 147, 118, 126, 212, 119, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  33,  25,  26,  19,  42,  27,  33,  42,  35,  35,  35, },
      {  33,  25,  26,  19,  34,  27,  33,  42,  35,  35,  35, },
      {  33,  25,  26,  34,  19,  27,  26,  50,  35,  35,  35, },
      {   9,  13,  13,  13,  13,  13,   9,   9,  13,  13,  13, },
      {   8,  13,  13,  12,  13,  13,  13,  13,  13,  13,  13, },
      {  13,  13,  12,  13,  12,  13,  12,  13,  13,  13,  13, },
      {  11,  11,  18,  18,  11,   4,   4,   4,  18,  18,   4, },
      {   4,  11,  18,  11,  11,   4,  11,  18,  11,  18,   4, },
      {  32,   4,   4,   4,   4,   4,   4,  11,   4,   4,   4, },
      {  69, 123, 115, 100, 101, 119, 121, 117, 121, 117, 236, },
      { 198, 118, 120, 132, 137, 124, 116, 133, 138, 135, 104, },
      }),
    };

const CtxSet ContextSetCfg::GtxFlagL[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  25,   0,  40,  25,  33,  26,   0,  17,  25,  33,  19,   9,  25,  26,  34,  20,  25,  18,  19,  20,  37, },
       {   1,   0,  17,  17,  25,  26,   0,   9,  25,  33,  34,   9,  25,  33,  34,  20,  25,  33,  19,  27,  29, },
       {   1,   0,   0,   0,  40,  56,   1,  17,  17,  25,   3,  17,  25,  18,  26,  12,  33,  19,  20,  28,  14, },
       {   1,   9,   9,   6,   6,   5,   9,   9,  10,   9,   6,   9,   9,   9,   5,   5,   6,   8,   5,   9,   9, },
       {   2,   8,   5,   9,   9,   5,  12,  12,   9,  13,   2,   9,   9,   9,   9,   5,   6,   5,   9,   9,   9, },
       {   8,   0,   0,  13,   0,   2,  12,  12,  12,  12,   6,   9,  10,   9,  10,   9,   6,   9,  10,  10,   9, },
       {  11,  11,  18,   4,   4,  18,   4,   4,  11,   4,  18,  11,  11,  18,   4,   4,  11,  18,  11,  18,  11, },
       {   4,   4,   4,   4,   4,  18,  11,   4,   4,  11,   4,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
       {   4,  32,   4,   4,  11,   4,  25,  18,  11,  18,   4,   4,   4,   4,  11,  11,   4,  18,  18,  18,   4, },
       {  99,  69,  83,  90,  78, 147,  68,  84, 124, 103, 116,  88, 105, 125, 117, 117, 105, 120, 121, 124, 125, },
       { 134, 163, 118, 117, 116, 117, 148, 119, 117, 131, 118, 117, 131, 117, 117, 115, 126, 116, 116, 116, 118, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,   9,  25,  33,  26,  12,  17,  33,  34,  28,  45, },
       {   1,   1,  25,  18,  11,  12,  17,  33,  19,  20,  22, },
       {   0,   1,  25,  18,  26,  19,  18,  19,  20,  28,  14, },
       {   1,   6,   9,   5,   5,   2,   6,   9,   8,   5,   8, },
       {   5,   8,   9,   5,   5,   2,   6,   9,   5,   5,   8, },
       {  10,  12,  13,   4,  10,   5,   5,   5,   9,   9,   8, },
       {  18,   4,   4,   4,   4,   4,   4,   4,  11,   4,  11, },
       {   4,   4,  11,  11,   4,   4,   4,   4,  11,  11,  11, },
       {   4,  25,  32,   4,  25,   4,   4,   4,  25,  18,   4, },
       {  99,  69,  76, 101, 100, 117,  99, 100, 102, 102, 116, },
       { 118, 117, 118, 119, 148, 116, 117, 119, 134, 141, 120, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   9,  17,  26,  27,  35,  21,  25,  34,  35,  36,  37,  33,  35,  36,  29,  30,  34,  36,  37,  45,  38, },
       {   1,  17,  26,  34,  35,  44,  25,  34,  35,  36,  37,  33,  20,  36,  29,  37,  34,  28,  37,  37,  38, },
       {  25,   0,  25,  18,  49,  58,  25,  11,  27,  20,  21,  26,  20,  21,  21,  22,  28,  29,  45,  30,  23, },
       {   9,   8,   6,   9,  10,   9,   9,  10,  13,  13,  10,   9,  10,   9,  10,  10,   6,   9,  10,  13,  13, },
       {   9,   9,   6,   9,  10,  10,   9,  10,  13,  13,  10,  10,  10,  10,  10,   9,   9,   9,  10,   9,  13, },
       {   9,  12,  12,  12,   6,   0,  13,  13,  13,  13,  12,  13,  13,   9,  10,  13,   9,  10,  10,  10,  13, },
       {  11,   4,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  11,  18,  18,  11, },
       {  11,  11,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  18,  11,  11,  11, },
       {  18,   4,   4,  32,  32,  18,  11,  11,  11,  11,   4,  18,  11,   4,   4,   4,  11,  11,  11,   4,   4, },
       {  88,  99, 107, 117, 117, 117,  92, 122, 117, 116, 110, 116, 123, 118, 122, 117, 116, 124, 126, 125, 124, },
       { 227, 117, 116, 100, 104, 117, 117, 126, 120, 156, 135, 118, 116, 124, 120, 102, 117, 116, 118, 116,  90, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {   9,  25,  27,  36,  13,  37,  42,  37,  45,  38,  46, },
      {   9,  25,  35,  28,  21,  22,  35,  37,  30,  30,  23, },
      {  40,  33,  27,  28,  13,  29,  36,  37,  45,  30,  46, },
      {   9,   9,   9,   9,  13,  10,   5,   6,   5,   9,   9, },
      {   9,  13,  13,  13,  13,  10,   5,   9,   5,   5,   9, },
      {  13,  13,  12,   8,   9,  10,   8,  12,   8,   8,  13, },
      {  11,  11,  11,   4,  11,  11,  11,   4,   4,  18,  11, },
      {   4,  18,  18,  18,  18,  11,  11,  18,  11,  11,  11, },
      {  32,  18,  11,   4,  11,  11,   4,  25,   4,   4,  18, },
      {  74,  99, 109, 115, 115, 117, 117, 100, 164, 110, 118, },
      { 117, 115, 119, 138, 167, 121, 118, 221, 105, 164, 105, },
      }),
    };
#endif

const CtxSet ContextSetCfg::SigFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  17,  41,  42,  29,   9,  42,  28,  37,  33,  44,  51,  45, },
       {  17,  41,  42,  29,  25,  42,  43,  37,  33,  44,  51,  30, },
       {  25,  19,  20,  14,  18,  28,  29,  30,  19,  45,  30,  38, },
       {  13,   9,   9,  10,   9,  10,   9,  10,   8,   9,   9,   9, },
       {  13,   9,   9,  10,   9,  10,  10,  10,   8,   9,   9,   9, },
       {  13,   9,   9,  10,  10,   9,   9,  13,   9,   9,  10,  10, },
       {  11,  11,  18,  18,  11,  18,  18,  11,  11,  18,  18,  11, },
       {  11,  11,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
       {  18,  11,  18,  11,  18,  11,  11,  11,  18,  18,  18,  11, },
       {  90, 119, 117, 121, 123, 119, 117, 126, 126, 132, 118, 132, },
       { 120, 133, 118, 125, 117, 126, 119, 120, 117, 117, 132, 108, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,  42,  43,  29,  41,  60,  60,  38, },
       {  18,  27,  28,  29,  34,  45,  45,  38, },
       {  25,  27,  28,  37,  34,  53,  53,  46, },
       {   8,  13,   9,  13,   5,   5,   8,  10, },
       {  13,  13,  13,  13,   5,   5,   8,   9, },
       {  12,  12,   9,   9,   5,   5,   8,   8, },
       {   4,  25,  11,  11,  18,  18,  18,  18, },
       {  18,  18,  18,  11,  11,  11,  18,  18, },
       {  18,  18,  11,   4,   4,   4,   4,   4, },
       { 119, 108, 119, 103, 125, 133, 132, 118, },
       { 109, 117, 126, 124, 119, 110, 126, 137, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  26,  45,  53,  46,  19,  54,  61,  39,  34,  39,  39,  39, },
       {  26,  38,  38,  46,  34,  54,  54,  39,  13,  39,  39,  39, },
       {  26,  38,  46,  54,  19,  39,  39,  39,  28,  39,  39,  39, },
       {   9,  13,  12,   8,   8,   8,   8,   4,   0,   0,   0,   0, },
       {   9,  13,  12,   8,   8,   8,   8,   5,   0,   0,   0,   0, },
       {   9,  13,  12,   8,   8,   8,   4,   4,   4,   0,   0,   0, },
       {  11,  11,  18,  18,  18,  25,  25,  18,   4,  32,  32,  32, },
       {  11,  11,  32,  11,  18,  25,  25,  25,  32,  32,  32,  32, },
       {  11,  11,  25,   4,  18,  32,  18,  32,  32,  32,  32,  32, },
       { 118, 238, 151, 158, 148, 182, 166, 142, 131, 168, 238, 138, },
       { 126, 236, 126,  91, 117, 131, 131,  98, 228, 116, 116, 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  41,  45,  38,  31,   4,  39,  39,  39, },
       {  34,  38,  53,  54,  44,  39,  39,  39, },
       {  26,  46,  38,  39,  44,  39,  39,  39, },
       {   8,  12,  12,   8,   4,   0,   0,   0, },
       {   8,  12,  12,   8,   4,   0,   0,   0, },
       {   8,  12,  12,   8,   0,   0,   0,   0, },
       {  18,  18,  25,  25,  25,  32,  32,  32, },
       {  11,  11,  25,  25,  25,  32,  32,  32, },
       {  18,  11,  18,  25,  32,  32,  32,  32, },
       { 132, 233, 221, 197, 116, 197, 214, 116, },
       { 117, 155,  98, 102, 149,  98, 116,  99, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  26,  54,  39,  39,  34,  39,  39,  39,   0,  39,  39,  39, },
       {  19,  39,  54,  39,  19,  39,  39,  39,  48,  39,  39,  39, },
       {  18,  39,  39,  39,  19,  39,  39,  39,   0,  39,  39,  39, },
       {   8,   8,   8,  12,   8,   4,   4,   8,   4,   0,   0,   0, },
       {   8,   8,   8,  12,   8,   4,   4,   8,   0,   0,   0,   0, },
       {   8,   8,  12,  12,   8,   0,   4,   4,   0,   0,   0,   0, },
       {  18,  25,  25,  32,  18,  18,  25,  32,  32,  32,  32,  32, },
       {  18,  25,  18,  32,  18,  18,  25,  32,  25,  32,  32,  32, },
       {  18,  25,  32,  25,  18,  32,  32,  32,  32,  32,  32,  32, },
       { 119, 190, 190, 171, 132, 155, 142, 139,  82, 117, 238, 229, },
       { 119, 115, 108,  92, 117,  99,  99,  82, 134, 116, 116, 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  26,  38,  54,  39,  26,  39,  39,  39, },
      {  34,  38,  62,  39,  26,  39,  39,  39, },
      {  11,  39,  39,  39,  18,  39,  39,  39, },
      {   8,  12,   8,   8,   0,   0,   0,   0, },
      {   8,   8,   8,   8,   0,   0,   0,   0, },
      {   8,   8,   8,   8,   4,   0,   0,   0, },
      {  25,  32,  25,  25,  18,  32,  32,  32, },
      {  18,  18,  25,  25,  18,  32,  32,  32, },
      {  18,  25,  25,  25,  32,  32,  32,  32, },
      { 227, 190, 168, 158, 115, 185, 229, 196, },
      { 117, 182, 118, 116, 134, 114, 114,  99, },
      }),
    };

const CtxSet ContextSetCfg::ParFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  33,  40,  33,  26,  34,  42,  25,  33,  34,  34,  27,  25,  34,  42,  42,  35,  33,  27,  35,  42,  35, },
       {  33,  25,  33,  26,  34,  42,  25,  33,  34,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  35,  35, },
       {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
       {   8,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,   9,  13,  13,  12,  13, },
       {   9,   9,   9,  12,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
       {   8,   9,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13, },
       {   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4,  11,  18,  11,  11,   4,  11,  18,  11,  11,   4, },
       {  11,  11,   4,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4, },
       {  11,   4,   4,   4,   4,   4,   4,   4,  11,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,  11,   4, },
       {  78,  90,  91, 102, 126, 126,  89, 119, 103, 158, 119, 171, 119, 126, 125, 123, 106, 135, 120, 119, 135, },
       { 133, 147, 117, 118, 126, 134, 120, 126, 140, 120, 140, 147, 126, 121, 133, 126, 147, 118, 126, 212, 119, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  33,  25,  26,  19,  42,  27,  33,  42,  35,  35,  35, },
      {  33,  25,  26,  19,  34,  27,  33,  42,  35,  35,  35, },
      {  33,  25,  26,  34,  19,  27,  26,  50,  35,  35,  35, },
      {   9,  13,  13,  13,  13,  13,   9,   9,  13,  13,  13, },
      {   8,  13,  13,  12,  13,  13,  13,  13,  13,  13,  13, },
      {  13,  12,  12,  12,  12,  13,  13,  13,  13,  12,  13, },
      {  11,  11,  18,  18,  11,   4,   4,   4,  18,  18,   4, },
      {   4,  11,  18,  11,  11,   4,  11,  18,  11,  18,   4, },
      {  18,   4,  11,  11,  11,   4,  11,  11,   4,   4,   4, },
      {  69, 123, 115, 100, 101, 119, 121, 117, 121, 117, 236, },
      { 198, 118, 120, 132, 137, 124, 116, 133, 138, 135, 104, },
      }),
    };

const CtxSet ContextSetCfg::GtxFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  25,   0,  40,  25,  33,  26,   0,  17,  25,  33,  19,   9,  25,  26,  34,  20,  25,  18,  19,  20,  37, },
       {   1,   0,  17,  17,  25,  26,   0,   9,  25,  33,  34,   9,  25,  33,  34,  20,  25,  33,  19,  27,  29, },
       {  25,   1,  40,  25,  33,  11,   9,  25,  25,  18,  12,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
       {   1,   9,   9,   6,   6,   5,   9,   9,  10,   9,   6,   9,   9,   9,   5,   5,   6,   8,   5,   9,   9, },
       {   2,   8,   5,   9,   9,   5,  12,  12,   9,  13,   2,   9,   9,   9,   9,   5,   6,   5,   9,   9,   9, },
       {   4,  12,   6,   9,   9,   5,  12,   9,   9,   9,   5,  13,  10,  10,  10,  10,   6,   9,  10,  10,  10, },
       {  11,  11,  18,   4,   4,  18,   4,   4,  11,   4,  18,  11,  11,  18,   4,   4,  11,  18,  11,  18,  11, },
       {   4,   4,   4,   4,   4,  18,  11,   4,   4,  11,   4,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
       {  11,  25,   4,  11,   4,  18,  18,  11,   4,  11,  11,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
       {  99,  69,  83,  90,  78, 147,  68,  84, 124, 103, 116,  88, 105, 125, 117, 117, 105, 120, 121, 124, 125, },
       { 134, 163, 118, 117, 116, 117, 148, 119, 117, 131, 118, 117, 131, 117, 117, 115, 126, 116, 116, 116, 118, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,   9,  25,  33,  26,  12,  17,  33,  34,  28,  45, },
       {   1,   1,  25,  18,  11,  12,  17,  33,  19,  20,  22, },
       {  40,   1,  25,  18,  26,  12,  25,  26,  27,  36,  37, },
       {   1,   6,   9,   5,   5,   2,   6,   9,   8,   5,   8, },
       {   5,   8,   9,   5,   5,   2,   6,   9,   5,   5,   8, },
       {   4,  12,  12,   8,   8,   2,   8,   5,  10,   4,   9, },
       {  18,   4,   4,   4,   4,   4,   4,   4,  11,   4,  11, },
       {   4,   4,  11,  11,   4,   4,   4,   4,  11,  11,  11, },
       {   4,  25,  25,  18,  18,   4,   4,   4,  25,   4,   4, },
       {  99,  69,  76, 101, 100, 117,  99, 100, 102, 102, 116, },
       { 118, 117, 118, 119, 148, 116, 117, 119, 134, 141, 120, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   9,  17,  26,  27,  35,  21,  25,  34,  35,  36,  37,  33,  35,  36,  29,  30,  34,  36,  37,  45,  38, },
       {   1,  17,  26,  34,  35,  44,  25,  34,  35,  36,  37,  33,  20,  36,  29,  37,  34,  28,  37,  37,  38, },
       {  25,  25,  19,  27,  20,  29,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  28,  29,  45,  30,  23, },
       {   9,   8,   6,   9,  10,   9,   9,  10,  13,  13,  10,   9,  10,   9,  10,  10,   6,   9,  10,  13,  13, },
       {   9,   9,   6,   9,  10,  10,   9,  10,  13,  13,  10,  10,  10,  10,  10,   9,   9,   9,  10,   9,  13, },
       {   9,   9,   6,  10,  10,  10,  13,  10,  13,  13,  13,   9,  10,  10,  10,  13,  10,  10,  10,  10,  13, },
       {  11,   4,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  11,  18,  18,  11, },
       {  11,  11,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  18,  11,  11,  11, },
       {  18,  18,   4,   4,   4,  18,  18,  11,  11,  11,  18,  11,  11,  11,   4,  11,  18,  18,  11,  11,  11, },
       {  88,  99, 107, 117, 117, 117,  92, 122, 117, 116, 110, 116, 123, 118, 122, 117, 116, 124, 126, 125, 124, },
       { 227, 117, 116, 100, 104, 117, 117, 126, 120, 156, 135, 118, 116, 124, 120, 102, 117, 116, 118, 116,  90, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {   9,  25,  27,  36,  13,  37,  42,  37,  45,  38,  46, },
      {   9,  25,  35,  28,  21,  22,  35,  37,  30,  30,  23, },
      {  40,  33,  35,  28,  13,  37,  43,  37,  45,  38,  46, },
      {   9,   9,   9,   9,  13,  10,   5,   6,   5,   9,   9, },
      {   9,  13,  13,  13,  13,  10,   5,   9,   5,   5,   9, },
      {  12,  12,   9,  12,   9,   9,   8,   8,   8,  13,  13, },
      {  11,  11,  11,   4,  11,  11,  11,   4,   4,  18,  11, },
      {   4,  18,  18,  18,  18,  11,  11,  18,  11,  11,  11, },
      {  18,  11,  11,  25,  11,  11,   4,  11,   4,  25,  18, },
      {  74,  99, 109, 115, 115, 117, 117, 100, 164, 110, 118, },
      { 117, 115, 119, 138, 167, 121, 118, 221, 105, 164, 105, },
      }),
    };

const CtxSet ContextSetCfg::LastX[] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {  21,   6,  12,  14,   7,   4,  14,   7,   6,   4,  14,   7,  14,   6,   5,  14,   7,  14,  14,  22,  20,  21,  14,  29,   5, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {   6,  13,  12,   6,   6,  12,  14,   6,   5,  12,  29,  14,   6,   6,   6,  14,  14,   6,   6,  14,  53,  14,   6,  22,  54, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {  21,   5,   4,   6,   6,   4,   6,   6,  14,   4,  14,  14,  22,   6,  11,  14,  15,  15,   7,   6,  11,   7,  37,   6,   6, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {   5,   5,   5,   6,   5,   1,   6,   5,   2,   1,   6,   1,   1,   1,   1,   2,   2,   1,   1,   1,   0,   1,   4,   4,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {   5,   5,   5,   6,   5,   1,   6,   5,   1,   0,   6,   1,   1,   1,   0,   1,   1,   1,   0,   0,   0,   1,   4,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {   9,   5,   5,   6,   6,   4,   6,   5,   1,   1,   6,   1,   1,   1,   0,   1,   2,   1,   0,   1,   0,   3,   4,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  25,  11,  11,  18,  25,  32,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  32,  11,  11,  18,  18,  32,  32,  18,  18,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        {  25,  11,  11,  18,  25,  18,  18,  25,  18,  25,  25,  18,  25,  32,  32,  18,  25,  18,  32,  32,  32,  25,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148, 116, 165, 237, 212, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
        { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228, 116, 132, 228, 228, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
      {  21,   6,  12,  14,   7,   4,  14,   7,   6,   4,  14,   7,  14,   6,   5,  14,   7,  14,  14,  22 },
        {   6,  13,  12,   6,   6,  12,  14,   6,   5,  12,  29,  14,   6,   6,   6,  14,  14,   6,   6,  14 },
        {  21,   5,   4,   6,   6,   4,   6,  14,  14,   4,  14,   7,  30,  14,   4,  22,  38,  15,  22,   6 },
        {   5,   5,   5,   6,   5,   1,   6,   5,   2,   1,   6,   1,   1,   1,   1,   2,   2,   1,   1,   1 },
        {   5,   5,   5,   6,   5,   1,   6,   5,   1,   0,   6,   1,   1,   1,   0,   1,   1,   1,   0,   0 },
        {   9,   5,   5,   6,   5,   4,   5,   5,   1,   1,   5,   1,   1,   0,   0,   1,   1,   1,   0,   0 },
        {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  25,  11,  11,  18,  25,  32 },
        {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  32,  11,  11,  18,  18,  32 },
        {  25,  18,  18,  18,  25,  25,  18,  25,  18,  32,  25,  18,  25,  18,  32,  18,  18,  25,  32,  32 },
        { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148 },
        { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228 },
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
      {   4,  12,   3, },
      {  19,  26,  25, },
      {  20,   4,   3, },
      {   1,   4,   1, },
      {   6,   5,   5, },
      {   6,   4,   1, },
      {  11,  18,   4, },
      {  18,  18,  18, },
      {  18,  18,   4, },
      { 117, 118, 117, },
      { 132, 118, 117, },
      }),
    };

const CtxSet ContextSetCfg::LastY[] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {   5,   5,  20,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  20,  21,   7,   6,   5,  28,  43,  21,   6,  21,  57, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {   5,   5,   4,   6,   6,   4,  14,  14,   5,   4,  14,   7,  13,   5,  14,  21,   7,  13,  13,  14,  22,  14,   6,   6,  30, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {  13,   5,   4,   6,   6,   4,  14,  14,   5,  11,  14,   7,   6,   5,  26,  37,  38,  22,  14,  12,  26,  43,  37,   5,   6, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
        {   6,   5,   5,   6,   5,   5,   6,   6,   5,   1,   2,   6,   1,   1,   0,   2,   2,   1,   0,   0,   0,   1,   8,   4,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {   5,   5,   5,   6,   6,   1,   6,   6,   5,   1,   6,   5,   1,   0,   0,   2,   2,   1,   0,   0,   0,   1,   8,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {   9,   5,   8,   6,   5,   4,   6,   6,   5,   0,   6,   6,   1,   1,   0,   1,   1,   2,   2,   2,   0,   1,   4,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
        {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,   4,  18,  11,  18,  25,  11,   4,  11,  11,  25,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        {   4,  11,  11,  18,  18,   4,  18,  18,  18,  18,  18,  18,  11,  18,  32,  18,   4,  18,  18,  32,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        {  18,  11,  11,  18,  18,  11,  18,  18,  25,  18,  25,  25,  18,  25,  32,  18,  18,  25,  32,  32,  32,  32,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
        { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196, 116, 196, 151, 232, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
        { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228, 116, 117, 230, 213, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
      {   5,   5,  20,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  20,  21,   7,   6,   5,  28 },
        {   5,   5,   4,   6,   6,   4,  14,  14,   5,   4,  14,   7,  13,   5,  14,  21,   7,  13,  13,  14 },
        {  13,   5,   4,  14,   6,  11,  14,  14,   5,  11,  14,   7,   6,   5,   3,  22,  38,  22,  14,   5 },
        {   6,   5,   5,   6,   5,   5,   6,   6,   5,   1,   2,   6,   1,   1,   0,   2,   2,   1,   0,   0 },
        {   5,   5,   5,   6,   6,   1,   6,   6,   5,   1,   6,   5,   1,   0,   0,   2,   2,   1,   0,   0 },
        {   9,   5,   8,   6,   5,   4,   6,   5,   4,   0,   6,   6,   1,   4,   0,   1,   1,   1,   1,   0 },
        {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,   4,  18,  11,  18,  25,  11,   4,  11,  11,  25 },
        {   4,  11,  11,  18,  18,   4,  18,  18,  18,  18,  18,  18,  11,  18,  32,  18,   4,  18,  18,  32 },
        {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  25,  25,  18,  32,  32,  18,  18,  25,  32,  32 },
        { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196 },
        { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228 },
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
      {  26,  20,  34, },
      {  26,  11,  25, },
      {  20,   4,  11, },
      {   2,   5,   0, },
      {   2,   6,   2, },
      {   7,   6,   5, },
      {  18,  32,  11, },
      {  11,  18,   4, },
      {  11,  18,   4, },
      { 117, 118, 117, },
      { 131, 148, 116, },
      }),
    };

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
  ({
     {  34, },
     {  34, },
     {  34, },
     {   6, },
     {   9, },
     {  13, },
     {   4, },
     {   4, },
     {  25, },
     { 122, },
     { 100, },
     });

#if JVET_X0083_BM_AMVP_MERGE_MODE
const CtxSet ContextSetCfg::amFlagState = ContextSetCfg::addCtxSet
  ({
     {  48, },
     {  41, },
     { CNU, },
     {   5, },
     {   5, },
     { DWS, },
     {  18, },
     {  11, },
     { DWE, },
     { 118, },
     { 118, },
     });
#endif

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  43, },
     {  28, },
     { CNU, },
     {   1, },
     {   4, },
     { DWS, },
     {  11, },
     {  11, },
     { DWE, },
     { 126, },
     { 102, },
     });

#if JVET_AG0098_AMVP_WITH_SBTMVP
const CtxSet ContextSetCfg::amvpSbTmvpFlag = ContextSetCfg::addCtxSet
  ({
     {  33,  33 },
     {  25,  49 },
     {  35,  35 },
     {   2,   1 },
     {   2,   1 },
     {   8,   8 },
     {  18,   4 },
     {  11,   4 },
     {  18,  18 },
     { 116, 134 },
     { 115, 117 },
     });

const CtxSet ContextSetCfg::amvpSbTmvpMvdIdx = ContextSetCfg::addCtxSet
  ({
     {   4,   4,  59 },
     {  58,  19,  33 },
     {  35,  35,  35 },
     {   3,   2,   3 },
     {   2,   3,   0 },
     {   8,   8,   8 },
     {   4,  11,  11 },
     {  11,   4,  11 },
     {  18,  18,  18 },
     { 151, 251, 251 },
     { 115, 115, 115 },
     });
#endif

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  10, },
     {  61, },
     {  53, },
     {   1, },
     {   1, },
     {   0, },
     {  32, },
     {  18, },
     {   4, },
     { 116, },
     { 116, },
     });

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
  ({
     {  10, },
     {   5, },
     {   4, },
     {   1, },
     {   8, },
     {   2, },
     {  18, },
     {  32, },
     {  32, },
     { 232, },
     { 197, },
     });

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  38, },
       {  38, },
       {  31, },
       {   6, },
       {   3, },
       {   4, },
       {   4, },
       {   4, },
       {   4, },
       { 115, },
       { 102, },
       }),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    ContextSetCfg::addCtxSet
    ({
       {  53, },
       {  37, },
       {  22, },
       {   2, },
       {   3, },
       {   0, },
       {  25, },
       {  18, },
       {   4, },
       { 116, },
       { 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  37, },
       {  30, },
       {  22, },
       {   3, },
       {   3, },
       {   0, },
       {  25, },
       {  18, },
       {   4, },
       { 116, },
       { 117, },
       })
#endif
  };
#endif

#if JVET_W0066_CCSAO
const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet
  ({
     {  11,  29,  38,  18,  29,  38,  18,  36,  45, },
     {   5,  30,  38,  19,  29,  45,  19,  21,  37, },
     {  43,  46,  46,  27,  38,  46,  35,  38,  46, },
     {   8,   0,   4,   5,   4,   4,   5,   4,   8, },
     {   5,   0,   4,   4,   4,   4,   4,   4,   4, },
     {   0,   0,   0,   0,   3,   4,   0,   0,   0, },
     {  18,   4,  11,   4,  18,  11,  11,  11,  32, },
     {  32,   4,  18,  18,  18,  18,  18,  18,  18, },
     {  32,   4,   4,  32,  25,  18,  32,  18,   4, },
     {  99, 165, 203,  99, 134, 237,  99, 148, 238, },
     { 198,  98,  99, 151,  98,  99, 141, 129,  99, },
     });
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
  ({
#if INTRA_TRANS_ENC_OPT
    {  51, CNU,  43,  42, },
      {  36, CNU,  36,  35, },
      { CNU,  43,  50,  42, },
      {  10, DWS,   5,  13, },
      {  10, DWS,   6,  13, },
      { DWS,  10,   5,  10, },
      {  11, DWE,  11,  11, },
      {  11, DWE,  11,  11, },
      { DWE,   4,   4,   4, },
      { 119, 126, 116, 116, },
      { 117, 117, 116, 125, },
#elif EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
    { 58, 37, 42, 35 },
      { 43, 45, 42, 35 },
      { 28, 43, 42, 27 },
      {  9,  9,  9, 10 },
      {  9,  9,  6, 13 },
      {  9, 10,  9, 10 },
      {DWE, DWE, DWE, DWE},
      {DWE, DWE, DWE, DWE},
      {DWE, DWE, DWE, DWE},
      {DWO, DWO, DWO, DWO},
      {DWO, DWO, DWO, DWO},
#else
    { 58, 37, 42 },
      { 43, 45, 42 },
      { 28, 43, 42 },
      {  9,  9,  9 },
      {  9,  9,  6 },
      {  9, 10,  9 },
      {DWE, DWE, DWE},
      {DWE, DWE, DWE},
      {DWE, DWE, DWE},
      {DWO, DWO, DWO},
      {DWO, DWO, DWO},
#endif
  });

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
  ({
     { 17, },
     { 0, },
     { 25, },
     { 1, },
     { 1, },
     { 2, },
     { DWE, },
     { DWE, },
     { 25, },
     { 119, },
     { 119, },
     });

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
  ({
     { 35, },
     { 42, },
     { 42, },
     { 5, },
     { 5, },
     { 7, },
     { DWE, },
     { DWE, },
     { 25, },
     { 119, },
     { 119, },
     });

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
  ({
     { 50, },
     { 59, },
     { 42, },
     { 9, },
     { 9, },
     { 9, },
     { DWE, },
     { DWE, },
     { 11, },
     { 119, },
     { 119, },
     });

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
  ({
     {  58,  45,  45,  30,  38, },
     {  51,  30,  30,  38,  23, },
     {  58,  37,  45,  30,  46, },
     {   9,   6,   9,  10,   5, },
     {   9,   6,   9,  10,   5, },
     {   9,   6,   5,  10,   5, },
     { DWE, DWE, DWE, DWE, DWE, },
     { DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,   4,  11,  18, },
     { 119, 119, 119, 119, 119, },
     { 119, 119, 119, 119, 119, },
     });

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
  ({
     {  45,  38,  46, },
     {  38,  53,  46, },
     {  46,  38,  54, },
     {   0,   9,   5, },
     {   0,   9,   5, },
     {   0,   6,   8, },
     { DWE, DWE, DWE, },
     { DWE, DWE, DWE, },
     {  32,   4,  18, },
     { 119, 119, 119, },
     { 119, 119, 119, },
     });

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
  ({
     {  25,   9, },
     {  25,   1, },
     {  25,   1, },
     {   2,   1, },
     {   2,   5, },
     {   5,   5, },
     {  18,  11, },
     {  18,  18, },
     {  25,  11, },
     { 119, 102, },
     { 147, 117, },
     });

const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
     {  43,  46,  46, CNU, },
     {  36,  46,  46, CNU, },
     {  19,  38,  38, CNU, },
     {   8,   9,   9, DWS, },
     {  12,   8,   8, DWS, },
     {   9,   9,   8, DWS, },
     {  11,  11,  18, DWE, },
     {  18,  11,  18, DWE, },
     {  18,  18,   4, DWE, },
     { 104, 135, 133, DWO, },
     { 135,  99,  83, DWO, },
     });

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
  ({
#if JVET_W0123_TIMD_FUSION
    {  26,  43,  33, },
      {  33,  43,  33, },
      {  33,  43,  33, },
      {   5,   2,   5, },
      {   6,   3,   8, },
      {   9,   1,   9, },
      {   4,  18,   4, },
      {  11,  11,  11, },
      {  18,  11,  18, },
      { 107, 119,  86, },
      { 116, 120, 133, },
#else
    {  26,  43 },
      {  33,  43 },
      {  33,  43 },
      {   5,   2 },
      {   6,   3 },
      {   9,   1 },
      {   4,  18 },
      {  11,  11 },
      {  18,  11 },
      { 107, 119 },
      { 116, 120 },
#endif
  });

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
  ({
     {  40,  49, },
     {  48,  49, },
     { CNU, CNU, },
     {   2,   6, },
     {   2,   6, },
     { DWS, DWS, },
     {  11,  18, },
     {  11,  11, },
     { DWE, DWE, },
     { 118, 110, },
     { 126, 126, },
     });

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
  ({
     {  42, },
     {  42, },
     { CNU, },
     {  10, },
     {  10, },
     { DWS, },
     {  11, },
     {  11, },
     { DWE, },
     { 100, },
     { 133, },
     });

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
  ({
     {  20,  43,  20, },
     {  20,  58,  19, },
     { CNU, CNU, CNU, },
     {   5,   5,   2, },
     {   6,   5,   5, },
     { DWS, DWS, DWS, },
     {   4,  11,  11, },
     {  11,  11,  11, },
     { DWE, DWE, DWE, },
     { 117, 122, 119, },
     { 124, 117, 117, },
     });

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
  ({
     {  28, },
     {  28, },
     { CNU, },
     {  13, },
     {  13, },
     { DWS, },
     {  11, },
     {  11, },
     { DWE, },
     { 102, },
     { 110, },
     });

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
  ({
     { 35, },
     { 35, },
     { 35, },
     { 8, },
     { 8, },
     { 8, },
     { DWE, },
     { DWE, },
     { DWE, },
     { DWO, },
     { DWO, },
     });

#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
  ({
     {  40, CNU, CNU, },
     {  48, CNU, CNU, },
     {  25, CNU, CNU, },
     {   6, DWS, DWS, },
     {   6, DWS, DWS, },
     {   3, DWS, DWS, },
     {  11, DWE, DWE, },
     {  11, DWE, DWE, },
     {   4, DWE, DWE, },
     { 107, DWO, DWO, },
     { 131, DWO, DWO, },
     });
#endif

#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
  ({
     {  41,  34,  42, },
     {  34,  34,  34, },
     {  57,  50,  58, },
     {   6,   6,   6, },
     {   7,   7,   5, },
     {   6,   6,   2, },
     {  11,  11,  18, },
     {   4,   4,   4, },
     {   4,   4,   4, },
     { 124, 126, 126, },
     { 126, 124, 117, },
     });
#endif

#if JVET_AB0155_SGPM
const CtxSet ContextSetCfg::SgpmFlag = ContextSetCfg::addCtxSet
  ({
     {  41,  34,  42, },
     {  34,  34,  34, },
     {  41,  49,  42, },
     {   6,   6,   6, },
     {   7,   7,   5, },
     {   6,   6,   7, },
     {  11,  11,  18, },
     {   4,   4,   4, },
     {   4,   4,   4, },
     { 124, 126, 126, },
     { 126, 124, 117, },
     });
#endif
#if ENABLE_OBMC
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    {  39,  53, },
      {  39,  53, },
      { CNU, CNU, },
      {   1,   7, },
      {   1,   4, },
      { DWS, DWS, },
      {  32,  25, },
      {  32,   4, },
      { DWE, DWE, },
      { 115, 102, },
      {  98,  68, },
#else
    {  39, },
      {  39, },
      { CNU, },
      {   1, },
      {   1, },
      { DWS, },
      {  32, },
      {  32, },
      { DWE, },
      { 115, },
      {  98, },
#endif
  });
#endif

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
  ({
     { 35, },
     { 35, },
     { 35, },
     { 8, },
     { 8, },
     { 8, },
     { DWE, },
     { DWE, },
     { DWE, },
     { DWO, },
     { DWO, },
     });

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
  ({
     {  59,  33,  50,  59,  53, },
     {  59,  33,  50,  59,  60, },
     { CNU, CNU, CNU, CNU, CNU, },
     {   1,   5,   1,   0,   4, },
     {   1,   5,   1,   0,   5, },
     { DWS, DWS, DWS, DWS, DWS, },
     {  11,  18,  11,  32,   4, },
     {  11,  11,  11,  32,  11, },
     { DWE, DWE, DWE, DWE, DWE, },
     { 126,  92, 126, 116, 118, },
     { 117, 147, 117, 116, 119, },
     });

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
  ({
     {  59,  33,  50,  59,  53, },
     {  59,  33,  50,  59,  60, },
     {  51,  40, CNU, CNU, CNU, },
     {   1,   5,   1,   0,   4, },
     {   1,   5,   1,   0,   5, },
     {   4,   8, DWS, DWS, DWS, },
     {  11,  18,  11,  32,   4, },
     {  11,  11,  11,  32,  11, },
     {  32,  32, DWE, DWE, DWE, },
     { 126,  92, 126, 116, 118, },
     { 117, 147, 117, 116, 119, },
     });
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
  ({
     {  11,  23,  46,  18,  46,  54,  18,  46,  54, },
     {   6,  15,  31,  12,  46,  54,   5,  46,  54, },
     {  39,  39,  39,  39,  39,  39,  54,  39,  39, },
     {   8,   4,   5,  12,   0,   2,  12,   0,   5, },
     {   4,   4,   4,   4,   4,   4,   4,   0,   4, },
     {   0,   3,   1,   0,   1,   4,   0,   5,   4, },
     {  11,  25,  25,  32,  11,   4,  32,  11,  11, },
     {  18,  25,  18,  25,  32,  18,  32,  18,  18, },
     {  32,  32,  18,  25,  25,  25,  32,  11,  25, },
     {  85, 133, 135, 115, 212, 139,  85, 164, 136, },
     { 153, 237,  64, 167, 132,  64, 165, 227,  64, },
     });

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
  ({
#if ALF_IMPROVEMENT
    {  27,  19,  35, },
      {  20,  27,  27, },
      {  26,  34,  34, },
      {   1,   1,   1, },
      {   1,   1,   1, },
      {   0,   0,   0, },
      {  32,  32,  32, },
      {  25,  32,  32, },
      {  32,  25,  25, },
      { 231, 230, 229, },
      { 116, 116, 116, },
#else
    {  27,  19 },
      {  20,  27 },
      {  19,  27 },
      {   1,   1 },
      {   1,   1 },
      {   0,   0 },
      {  32,  32 },
      {  25,  32 },
      {  32,  18 },
      { 231, 230 },
      { 116, 116 },
#endif
  });

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
  ({
     {  39, },
     {  39, },
     {  39, },
     {   1, },
     {   1, },
     {   8, },
     {  11, },
     {   4, },
     {  18, },
     {  93, },
     {  67, },
     });

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
  ({
     {  33,  44,  46,  18,  44,  46, },
     {   3,  37,  46,   3,  45,  46, },
     {  18,  29,  46,  10,  52,  46, },
     {   4,   1,   5,   4,   2,   8, },
     {   1,   1,   1,   4,   2,   4, },
     {   4,   3,   4,   4,   3,   4, },
     {   4,  18,   4,  11,  25,  18, },
     {   4,  18,   4,  11,  25,  11, },
     {  25,  18,  32,  18,  25,  25, },
     {  99, 117, 122, 100, 130, 135, },
     { 142, 130,  99, 135, 130,  99, },
     });

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
  ({
#if CIIP_PDPC
    {  56,  43, },
      {  57,  43, },
      { CNU, CNU, },
      {   1,   2, },
      {   2,   3, },
      { DWS, DWS, },
      {  11,  11, },
      {  11,  11, },
      { DWE, DWE, },
      { 126, 126, },
      { 117, 126, },
#else
    {  56 },
      {  57 },
      { CNU },
      {   1 },
      {   2 },
      { DWS },
      {  11 },
      {  11 },
      { DWE },
      { 126 },
      { 117 },
#endif
  });

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  43,  45, },
     {   0,  19,  36, },
     {   0,  18,  27, },
     {   5,   5,   6, },
     {   4,   7,   7, },
     {   5,   5,   8, },
     {  25,  18,  32, },
     {  18,  11,  18, },
     {  11,  18,  25, },
     { 117, 124, 119, },
     { 147, 117, 146, },
     });

#if JVET_AE0169_BIPREDICTIVE_IBC
const CtxSet ContextSetCfg::BiPredIbcFlag = ContextSetCfg::addCtxSet
  ({
     {   35, 35, },
     {   35, 35, },
     {  20,  18, },
     {    8,  8, },
     {    8,  8, },
     {   5,   5, },
     {   18, 18, },
     {   18, 18, },
     {  18,  11, },
     {  119,119, },
     {  119,119, },
     });
#endif

#if JVET_AC0112_IBC_CIIP
const CtxSet ContextSetCfg::IbcCiipFlag = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, },
     {  CNU, CNU, CNU, },
     {  33,   0,  CNU, },
     {  DWS, DWS, DWS, },
     {  DWS, DWS, DWS, },
     {   8,   8,  DWS, },
     {  DWE, DWE, DWE, },
     {  DWE, DWE, DWE, },
     {  25,   4,  DWE, },
     {  DWO, DWO, DWO, },
     {  DWO, DWO, DWO, },
     });

const CtxSet ContextSetCfg::IbcCiipIntraIdx = ContextSetCfg::addCtxSet
  ({
     {  CNU, },
     {  CNU, },
     {   35, },
     {  DWS, },
     {  DWS, },
     {   12, },
     {  DWE, },
     {  DWE, },
     {   18, },
     {  DWO, },
     {  DWO, },
     });
#endif

#if JVET_AC0112_IBC_GPM
const CtxSet ContextSetCfg::IbcGpmFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  41, },
     { DWS, },
     { DWS, },
     {   5, },
     { DWE, },
     { DWE, },
     {  18, },
     { DWO, },
     { DWO, },
     });

#if JVET_AE0169_GPM_IBC_IBC
const CtxSet ContextSetCfg::IbcGpmIntraFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU},
     { CNU, CNU},
     {  50,  43, },
     { DWS, DWS},
     { DWS, DWS},
     {   7,   0, },
     { DWE, DWE},
     { DWE, DWE},
     {  18,  11, },
     { DWO, DWO},
     { DWO, DWO},
     });
#else
const CtxSet ContextSetCfg::IbcGpmIntraFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     { DWE, },
     { DWE, },
     { DWE, },
     { DWO, },
     { DWO, },
     });
#endif

const CtxSet ContextSetCfg::IbcGpmSplitDirSetFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  21, },
     { DWS, },
     { DWS, },
     {   4, },
     { DWE, },
     { DWE, },
     {  11, },
     { DWO, },
     { DWO, },
     });

const CtxSet ContextSetCfg::IbcGpmBldIdx = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU },
     {  29,  44,  27,  36, },
     { DWS, DWS, DWS, DWS },
     { DWS, DWS, DWS, DWS },
     {   4,   5,   5,  10, },
     { DWE, DWE, DWE, DWE },
     { DWE, DWE, DWE, DWE },
     {  25,  11,   4,  11, },
     { DWO, DWO, DWO, DWO },
     { DWO, DWO, DWO, DWO },
     });
#endif

#if JVET_AC0112_IBC_LIC
const CtxSet ContextSetCfg::IbcLicFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AE0159_FIBC
    { CNU, CNU, CNU, CNU, },
      { CNU, CNU, CNU, CNU, },
      {  19,  25,  33,  26, },
      { DWS, DWS, DWS, DWS, },
      { DWS, DWS, DWS, DWS, },
      {   1,   6,   1,   3, },
      { DWE, DWE, DWE, DWE, },
      { DWE, DWE, DWE, DWE, },
      {  32,  32,  18,  25, },
      { DWO, DWO, DWO, DWO, },
      { DWO, DWO, DWO, DWO, },
#else
    { CNU, },
      { CNU, },
      { CNU, },
      { DWS, },
      { DWS, },
      { DWS, },
      { DWE, },
      { DWE, },
      { DWE, },
      { DWO, },
      { DWO, },
#endif
  });
#if JVET_AE0078_IBC_LIC_EXTENSION
const CtxSet ContextSetCfg::IbcLicIndex = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, },
     { CNU, CNU, },
     {  49,  41, },
     { DWS, DWS, },
     { DWS, DWS, },
     {   4,   4, },
     { DWE, DWE, },
     { DWE, DWE, },
     {   4,  25, },
     { DWO, DWO, },
     { DWO, DWO, },
     });
#endif
#endif

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
  ({
     {  34,  28,  52, },
     {  27,  36,  52, },
     {  20,  29,  58, },
     {   1,   0,   1, },
     {   1,   0,   2, },
     {   1,   1,   2, },
     {  18,  18,  25, },
     {  18,  18,  25, },
     {  18,  18,  32, },
     { 117, 132, 117, },
     { 120, 117, 117, },
     });

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
  ({
     {  18,  20,  37, },
     {  18,  27,  29, },
     {  18,  20,  38, },
     {   6,   9,   5, },
     {   7,   9,   5, },
     {   6,   9,   9, },
     {  11,  18,  18, },
     {  18,   4,   4, },
     {  18,  25,  25, },
     { 117, 125, 133, },
     { 227, 228, 123, },
     });

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  35,  37, },
     {  40,  35,  44, },
     {  25,  28,  38, },
     {  13,  13,   5, },
     {  13,  13,   6, },
     {  13,  13,   9, },
     {   4,  18,  18, },
     {   4,  11,  11, },
     {   4,  18,  18, },
     {  90, 189, 110, },
     { 104, 238, 119, },
     });

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
  ({
     {   3, },
     {  10, },
     {  11, },
     {   5, },
     {   3, },
     {   6, },
     {  18, },
     {   4, },
     {  11, },
     { 116, },
     { 110, },
     });

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
  ({
     { CNU,  10,   3,   3,   4, },
     { CNU,  17,  10,   3,   3, },
     { CNU,  10,   3,   3,   4, },
     { DWS,   1,   2,   1,   3, },
     { DWS,   3,   2,   1,   0, },
     { DWS,   1,   2,   1,   1, },
     { DWE,  18,  25,  25,  32, },
     { DWE,  11,   4,   4,  11, },
     { DWE,  11,  18,  18,  11, },
     { DWO, 132, 226, 227, 227, },
     { DWO, 118, 118, 118, 116, },
     });

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
  ({
     {  11,  11,   4,  14, },
     {  25,  11,   4,  21, },
     {  11,   5,   5,   6, },
     {   1,   1,   1,   2, },
     {   6,   3,   2,   2, },
     {   5,   2,   2,   7, },
     {  11,  18,  18,  18, },
     {  11,  18,  11,  11, },
     {  18,  18,  18,  18, },
     { 116, 117, 118, 145, },
     { 118, 117, 116, 102, },
     });

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
  ({
     {   5,  10,  61,  28,  33,  38, },
     {  12,  10,  53,  27,  25,  46, },
     {   5,   2,  46,  28,  25,  46, },
     {   1,   5,   5,   7,   5,   8, },
     {   5,   6,   5,   2,   7,   7, },
     {   1,   5,   5,   5,   9,   8, },
     {  11,  18,  18,  18,  25,  32, },
     {  11,  18,  18,  18,  11,   4, },
     {  18,  25,  18,  18,  32,  25, },
     { 118, 164, 117, 226, 227, 132, },
     { 117, 117, 116, 115, 118, 195, },
     });

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  34,  34,  34,  26, },
       {  34,  34,  19,  26, },
       {  34,  34,  34,  26, },
       {  13,  10,  10,  10, },
       {  13,  13,  13,  10, },
       {  13,  10,  10,   7, },
       {  11,   4,   4,  11, },
       {   4,   4,  11,  11, },
       {   4,   4,   4,   4, },
       { 110, 102, 104, 100, },
       { 110, 110, 116, 116, },
       }),
    ContextSetCfg::addCtxSet
    ({
      {  34,  34,  34,  26, },
      {  41,  41,  26,  41, },
      {  34,  34, CNU, CNU, },
      {  13,  12,  10,   9, },
      {  13,  13,  10,   9, },
      {  10,  10, DWS, DWS, },
      {  11,  11,  11,  11, },
      {   4,   4,   4,   4, },
      {   4,   4, DWE, DWE, },
      { 100,  91,  99,  98, },
      { 116, 110, 117, 125, },
      }),
    };
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  35,  42,  27,  36,  50, },
     {  DWS, DWS, DWS, DWS, DWS, },
     {  DWS, DWS, DWS, DWS, DWS, },
     {   4,   9,   4,  13,  12, },
     {  DWE, DWE, DWE, DWE, DWE, },
     {  DWE, DWE, DWE, DWE, DWE, },
     {  18,  25,  32,  25,  18, },
     {  DWO, DWO, DWO, DWO, DWO, },
     {  DWO, DWO, DWO, DWO, DWO, },
     });
#endif

#if JVET_AA0126_GLM
const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  18,  21,  44,  35,  35, },
     {  DWS, DWS, DWS, DWS, DWS, },
     {  DWS, DWS, DWS, DWS, DWS, },
     {   4,   4,   0,   0,   0, },
     {  DWE, DWE, DWE, DWE, DWE, },
     {  DWE, DWE, DWE, DWE, DWE, },
     {  18,  18,   4,  25,  11, },
     {  DWO, DWO, DWO, DWO, DWO, },
     {  DWO, DWO, DWO, DWO, DWO, },
     });
#endif

#if JVET_AA0057_CCCM
const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AC0147_CCCM_NO_SUBSAMPLING && JVET_AC0054_GLCCCM
    { CNU, CNU, CNU, },
      { CNU, CNU, CNU, },
      {  21,  42,  27, },
      { DWS, DWS, DWS, },
      { DWS, DWS, DWS, },
      {   4,   4,   4, },
      { DWE, DWE, DWE, },
      { DWE, DWE, DWE, },
      {   4,  11,   4, },
      { DWO, DWO, DWO, },
      { DWO, DWO, DWO, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
    { CNU, CNU, },
      { CNU, CNU, },
      { CNU, CNU, },
      { DWS, DWS, },
      { DWS, DWS, },
      { DWS, DWS, },
      { DWE, DWE, },
      { DWE, DWE, },
      { DWE, DWE, },
      { DWO, DWO, },
      { DWO, DWO, },
#else
    { CNU, },
      { CNU, },
      { CNU, },
      { DWS, },
      { DWS, },
      { DWS, },
      { DWE, },
      { DWE, },
      { DWE, },
      { DWO, },
      { DWO, },
#endif
  });

#if JVET_AD0202_CCCM_MDF
const CtxSet ContextSetCfg::CccmMpfFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, },
     { CNU, CNU, CNU, },
     {  35,  29,  35, },
     { DWS, DWS, DWS, },
     { DWS, DWS, DWS, },
     {   8,   4,   4, },
     { DWE, DWE, DWE, },
     { DWE, DWE, DWE, },
     {  25,  11,  18, },
     { DWO, DWO, DWO, },
     { DWO, DWO, DWO, },
     });
#endif
#if JVET_AE0100_BVGCCCM
const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  41, },
     { DWS, },
     { DWS, },
     {   1, },
     { DWE, },
     { DWE, },
     {  11, },
     { DWO, },
     { DWO, },
     });
#endif
#endif

#if JVET_AD0120_LBCCP
const CtxSet ContextSetCfg::CcInsideFilterFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  29, },
     { DWS, },
     { DWS, },
     {   4, },
     { DWE, },
     { DWE, },
     {  32, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  35, },
     { DWS, },
     { DWS, },
     {   5, },
     { DWE, },
     { DWE, },
     {   4, },
     { DWO, },
     { DWO, },
     });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  58, },
     { DWS, },
     { DWS, },
     {   4, },
     { DWE, },
     { DWE, },
     {   4, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     {  41,  50,  43,  43,  35,  35,  50, CNU, },
     {   8,   8,   9,   9,   9,   8,   9, DWS, },
     {   8,   8,   9,   9,   9,   8,   9, DWS, },
     {   5,   8,   9,   9,   9,   8,   9, DWS, },
     {  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
     {  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
     {  11,  18,  18,  18,  18,  18,  18, DWE, },
     { 119, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
     { 119, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
     });
#endif

#if JVET_AD0188_CCP_MERGE
const CtxSet ContextSetCfg::nonLocalCCP = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     {  50, },
     { DWS, },
     { DWS, },
     {   4, },
     { DWE, },
     { DWE, },
     {  11, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
const CtxSet ContextSetCfg::decoderDerivedCCP = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, }, 
  { DWE, },
  { DWE, },
  { DWE, }, 
  { DWO, },
  { DWO, },
	});

const CtxSet ContextSetCfg::ddNonLocalCCP = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { 50, },
  { DWS, },
  { DWS, },
  { 4, },
  { DWE, },
  { DWE, },
  { 11, },
  { DWO, },
  { DWO, },
	});
#endif

#if JVET_AE0059_INTER_CCCM
const CtxSet ContextSetCfg::InterCccmFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     { DWE, },
     { DWE, },
     { DWE, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AF0073_INTER_CCP_MERGE
const CtxSet ContextSetCfg::InterCcpMergeFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     { DWE, },
     { DWE, },
     { DWE, },
     { DWO, },
     { DWO, },
     });
#endif

#if JVET_AG0058_EIP
const CtxSet ContextSetCfg::EipFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, },
     { CNU, CNU, },
     { CNU, CNU, },
     { DWS, DWS, },
     { DWS, DWS, },
     { DWS, DWS, },
     { DWE, DWE, },
     { DWE, DWE, },
     { DWE, DWE, },
     { DWO, DWO, },
     { DWO, DWO, },
     });
#endif

#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
const CtxSet ContextSetCfg::CCPMergeFusionFlag = ContextSetCfg::addCtxSet
({
   { CNU },
   { CNU },
   { CNU },
   { DWS },
   { DWS },
   { DWS },
   { DWE },
   { DWE },
   { DWE },
   { DWO },
   { DWO },
    });
const CtxSet ContextSetCfg::CCPMergeFusionType = ContextSetCfg::addCtxSet
({
     { CNU },
     { CNU },
     { CNU },
     { DWS },
     { DWS },
     { DWS },
     { DWE },
     { DWE },
     { DWE },
     { DWO },
     { DWO },
    });
#endif
#endif
#endif

