#pragma once
#include "CommonDef.h"

// VTM
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
  ({
     {  18,  27,  15,  18,  28,  45,  26,   7,  23, },
     {  11,  35,  53,  12,   6,  30,  13,  15,  31, },
     {  19,  28,  38,  27,  29,  38,  20,  30,  31, },
     {  12,  13,   8,   8,  13,  12,   5,   9,   9, },
     });

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
  ({
     {  26,  36,  38,  18,  34,  21, },
     {  20,  14,  23,  18,  19,   6, },
     {  27,   6,  15,  25,  19,  37, },
     {   0,   8,   8,  12,  12,   8, },
     });

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
  ({
     {  43,  42,  37,  42,  44, },
     {  43,  35,  37,  34,  52, },
     {  43,  42,  29,  27,  44, },
     {   9,   8,   9,   8,   5, },
     });

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
  ({
     {  28,  29,  28,  29, },
     {  43,  37,  21,  22, },
     {  36,  45,  36,  45, },
     {  12,  13,  12,  13, },
     });

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  20, },
     {  25,  12, },
     { CNU, CNU, },
     {   1,   0, },
     });
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
  ({
     {  57,  60,  46, },
     {  57,  59,  45, },
     {   0,  26,  28, },
     {   5,   4,   8, },
     });

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
  ({
     {   6, },
     {  21, },
     {  26, },
     {   4, },
     });

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  46,  15, },
     {  38,   7, },
     { CNU, CNU, },
     {   5,   5, },
     });

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    { 33, 28, 36, 36, 29, 35, 35, 35, 35, 35 },
      { 20, 21, 29, 29, 29, 35, 35, 35, 35, 35 },
      { 34, 58, 28, 35, 25, 35, 35, 35, 35, 35 },
      { 4,  5,  5,  4,  8,  4,  4,  4,  4,  4 }
#else
    {  18, },
      {  20, },
      {  34, },
      {   4, },
#endif
  });

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    { 19, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
      { 13, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
      { 34, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
      { 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 }
#else
    {  18, },
      {  20, },
      {  34, },
      {   4, },
#endif
  });
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  26, },
     { CNU, },
     {   4, },
     });

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
  ({
     {  43, },
     {  43, },
     { CNU, },
     {  10, },
     });

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
     {  59, },
     {  60, },
     { CNU, },
     {   0, },
     });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
  ({
     {  59, },
     {  60, },
     { CNU, },
     {   0, },
     });
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  26, },
     { CNU, },
     {   4, },
     });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
     {  59, },
     {  60, },
     { CNU, },
     {   0, },
     });
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
  ({
    { 33, 28, 36, 36, 29, },
    { 20, 21, 29, 29, 29, },
    { 34, 58, 28, 35, 25, },
    {  4,  5,  5,  4,  8, }
  });
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
  ({
    { 18 },
    { 11 },
    { 35 },
    { 4 }
  });
const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
  ({
    { 43 },
    { 43 },
    { 35 },
    { 10 }
  });
const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
  ({
    { 51 },
    { 60 },
    { 35 },
    { 0 }
  });
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
  ({
    { 51 },
    { 60 },
    { 35 },
    {  0 }
  });
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  26, },
     { CNU, },
     {   4, },
     });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
  ({
     {  43, },
     {  43, },
     { CNU, },
     {  10, },
     });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
     {  59, },
     {  60, },
     { CNU, },
     {   0, },
     });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    {  25,  33 },
      {  26,  25 },
      { CNU, CNU },
      {   4,   5 }
#else
    {  25, },
      {  26, },
      { CNU, },
      {   4, },
#endif
  });
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  26, },
     { CNU, },
     {   4, },
     });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
  ({
     {  40,  35, },
     {  40,  35, },
     { CNU, CNU, },
     {   5,   1, },
     });

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
  ({
     { CNU },
     { CNU },
     { CNU },
     { DWS },
     });
#endif

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
    { 25, 59, 59, 59, 59, 25, 59},
      { 25, 58, 58, 58, 58, 25, 58},
      { 25, 60, 60, 60, 60, 25, 60},
      { 5,  8,  8,  8,  8,  5,  8 },
#else
    { 25, 59, 59, 59, 59},
      { 25, 58, 58, 58, 58},
      { 25, 60, 60, 60, 60},
      { 5,  8,  8,  8,  8 },
#endif
#else
#if JVET_W0123_TIMD_FUSION
    {  25,  59,  25,  59, },
      {  25,  58,  25,  58, },
      {  25,  60,  25,  60, },
      {   5,   8,  5,   8, },
#else
    {  25,  59, },
      {  25,  58, },
      {  25,  60, },
      {   5,   8, },
#endif
#endif
  });

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
  ({
     {  44, },
     {  36, },
     {  45, },
     {   6, },
     });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
  ({
    { 36 },
    { 36 },
    { 44 },
    { 7 }
  });
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
  ({
     {  13,   6, },
     {  12,  20, },
     {  13,  28, },
     {   1,   5, },
     });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
  ({
    { 20, 21, 20 },
    { 5, 28, 13 },
    { 20, 44, 35 },
    { 2,  2,  6 }
  });
#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU, CNU },
     { DWS, DWS, DWS, DWS, DWS },
     });
#endif
#endif

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
  ({
     {  26, },
     {  34, },
     {  59, },
     {   4, },
     });

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
  ({
     {  27, },
     {  27, },
     {  27, },
     {   9, },
     });

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
  ({
     {  25, },
     {  25, },
     {  34, },
     {   5, },
     });

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
  ({
     {  56,  57,  50,  26, },
     {  41,  57,  58,  26, },
     {  33,  49,  50,  25, },
     {   9,  10,   9,   6, },
     });
#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
  ({
     {  CNU,  CNU,  CNU,  CNU, },
     {  CNU,  CNU,  CNU,  CNU, },
     {  CNU,  CNU,  CNU,  CNU, },
     {  DWS,  DWS,  DWS,  DWS, },
     });
#endif


#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
  ({
    { 46 },
    { 46 },
    { 53 },
    { 8 }
  });
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, },
     { CNU, CNU, },
     { CNU, CNU, },
     { DWS, DWS, },
     });

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
  ({
#if CTU_256
    { 7,  6,  5, 12, 11,  3, 10, 40 },
      { 7, 21,  5, 12,  4, 18, 18, 48 },
      { 35, 35, 35, 35, 35, 35, 35, 35 },
      { 0,  0,  0,  1,  4,  5,  5,  0 }
#else
    {  14,  13,   5,   4,   3,   3,  40, },
      {   7,   6,   5,  12,   4,   4,  40, },
      { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
      {   0,   0,   1,   4,   4,   4,   0, },
#endif
  });

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
  ({
     {   5,  35, },
     {  20,  35, },
     { CNU, CNU, },
     {   0,   4, },
     });

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  58,  45, },
     {  48,  57,  44, },
     { CNU, CNU, CNU, },
     {   4,   4,   4, },
     });

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
  ({
     { 25, CNU, CNU, CNU },
     { 26, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU },
     { 4, 4, 4, 4 },
     });
#endif

#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
  ({
     { 25, CNU, CNU, CNU },
     { 26, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU },
     { 4, 4, 4, 4 },
     });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
  ({
     {  19,  13,   6, },
     {  12,  13,  14, },
     { CNU, CNU, CNU, },
     {   4,   0,   0, },
     });

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
  ({
     {  35, },
     {  35, },
     { CNU, },
     {   4, },
     });

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
  ({
     {   4, },
     {   5, },
     { CNU, },
     {   0, },
     });

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
  ({
    { 27 },
    { 34 },
    { 35 },
    { 8 }
  });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
  ({
     {   5, },
     {   4, },
     { CNU, },
     {   1, },
     });

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
  ({
     {  51,  36, },
     {  44,  43, },
     {  14,  45, },
     {   9,   5, },
     });

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
  ({
    { 53, 38, 38, 29, 20, 34, 27, 45, 37, 43, 34, 48 },
    { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
    { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
    {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 }
  });
#endif

#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
  ({
     { 3, 26,CNU,},
     { 3, 26,CNU,},
     { 35, 35,CNU,},
     { 1,   4,CNU,},
     });

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
  ({
    { 28, 50 },
    { 28, 27 },
    { 35, 35 },
    { 1,  0 }
  });

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
  ({
    { 50, 35 },
    { 12, 35 },
    { 35, 35 },
    { 0,  4 }
  });
#endif

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
  ({
     {  19,  21,   0,  28, },
     {  40,  36,   0,  13, },
     {  19,  35,   1,  27, },
     {   1,   4,   1,   0, },
     });

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
  ({
     {  12, },
     {   5, },
     {   6, },
     {   4, },
     });

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
  ({
     {  46, },
     {  46, },
     {  52, },
     {   1, },
     });

const CtxSet ContextSetCfg::QtCbf[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  15,   6,   5,  14, },
       {  23,   5,  20,   7, },
       {  15,  12,   5,   7, },
       {   5,   1,   8,   9, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,  37, },
       {  25,  28, },
       {  12,  21, },
       {   5,   0, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   9,  36,  45, },
       {  25,  29,  45, },
       {  33,  28,  36, },
       {   2,   1,   0, },
       })
  };

const CtxSet ContextSetCfg::SigCoeffGroup[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  25,  45, },
       {  25,  30, },
       {  18,  31, },
       {   8,   5, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,  14, },
       {  25,  45, },
       {  25,  15, },
       {   5,   8, },
       })
  };

const CtxSet ContextSetCfg::SigFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  17,  41,  49,  36,   1,  49,  50,  37,  48,  51,  58,  45, },
       {  17,  41,  42,  29,  25,  49,  43,  37,  33,  58,  51,  30, },
       {  25,  19,  28,  14,  25,  20,  29,  30,  19,  37,  30,  38, },
       {  12,   9,   9,  10,   9,   9,   9,  10,   8,   8,   8,  10, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   9,  49,  50,  36,  48,  59,  59,  38, },
       {  17,  34,  35,  21,  41,  59,  60,  38, },
       {  25,  27,  28,  37,  34,  53,  53,  46, },
       {  12,  12,   9,  13,   4,   5,   8,   9, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  26,  45,  53,  46,  49,  54,  61,  39,  35,  39,  39,  39, },
       {  19,  38,  38,  46,  34,  54,  54,  39,   6,  39,  39,  39, },
       {  11,  38,  46,  54,  27,  39,  39,  39,  44,  39,  39,  39, },
       {   9,  13,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  34,  45,  38,  31,  58,  39,  39,  39, },
       {  35,  45,  53,  54,  44,  39,  39,  39, },
       {  19,  46,  38,  39,  52,  39,  39,  39, },
       {   8,  12,  12,   8,   4,   0,   0,   0, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  19,  54,  39,  39,  50,  39,  39,  39,   0,  39,  39,  39, },
       {  19,  39,  54,  39,  19,  39,  39,  39,  56,  39,  39,  39, },
       {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
       {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  34,  38,  54,  39,  41,  39,  39,  39, },
       {  34,  38,  62,  39,  26,  39,  39,  39, },
       {  11,  39,  39,  39,  19,  39,  39,  39, },
       {   8,   8,   8,   8,   4,   0,   0,   0, },
       })
  };

const CtxSet ContextSetCfg::ParFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  33,  40,  25,  41,  26,  42,  25,  33,  26,  34,  27,  25,  41,  42,  42,  35,  33,  27,  35,  42,  43, },
       {  18,  17,  33,  18,  26,  42,  25,  33,  26,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  20,  20, },
       {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
       {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  33,  25,  26,  34,  19,  27,  33,  42,  43,  35,  43, },
       {  25,  25,  26,  11,  19,  27,  33,  42,  35,  35,  43, },
       {  33,  25,  26,  42,  19,  27,  26,  50,  35,  20,  43, },
       {   8,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13, },
       })
  };

const CtxSet ContextSetCfg::GtxFlag[] =
  {
    ContextSetCfg::addCtxSet
    ({
       {  25,   0,   0,  17,  25,  26,   0,   9,  25,  33,  19,   0,  25,  33,  26,  20,  25,  33,  27,  35,  22, },
       {  17,   0,   1,  17,  25,  18,   0,   9,  25,  33,  34,   9,  25,  18,  26,  20,  25,  18,  19,  27,  29, },
       {  25,   1,  40,  25,  33,  11,  17,  25,  25,  18,   4,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
       {   1,   5,   9,   9,   9,   6,   5,   9,  10,  10,   9,   9,   9,   9,   9,   9,   6,   8,   9,   9,  10, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {  25,   1,  25,  33,  26,  12,  25,  33,  27,  28,  37, },
       {  17,   9,  25,  10,  18,   4,  17,  33,  19,  20,  29, },
       {  40,   9,  25,  18,  26,  35,  25,  26,  35,  28,  37, },
       {   1,   5,   8,   8,   9,   6,   6,   9,   8,   8,   9, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   0,   0,  33,  34,  35,  21,  25,  34,  35,  28,  29,  40,  42,  43,  29,  30,  49,  36,  37,  45,  38, },
       {   0,  17,  26,  19,  35,  21,  25,  34,  20,  28,  29,  33,  27,  28,  29,  22,  34,  28,  44,  37,  38, },
       {  25,  25,  11,  27,  20,  21,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  36,  29,  45,  30,  23, },
       {   9,   5,  10,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  13,   8,   9,  10,  10,  13, },
       }),
    ContextSetCfg::addCtxSet
    ({
       {   0,  40,  34,  43,  36,  37,  57,  52,  45,  38,  46, },
       {   0,  25,  19,  20,  13,  14,  57,  44,  30,  30,  23, },
       {  40,  33,  27,  28,  21,  37,  36,  37,  45,  38,  46, },
       {   8,   8,   9,  12,  12,  10,   5,   9,   9,   9,  13, },
       })
  };

const CtxSet ContextSetCfg::LastX[] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37, 43, 35,  6, 15, 53, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
        {  6, 13, 12,  6,  6, 12, 14,  6, 13, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6, 37,  6,  6, 15, 31, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 },
        { 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6, 19,  7, 37,  7, 13, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42 },
        {  5,  5,  4,  5,  4,  4,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
      {   6,   6,  12,  14,   6,   4,  14,   7,   6,   4,  29,   7,   6,   6,  12,  28,   7,  13,  13,  35, }, // 20 contexts
        {   6,  13,  12,   6,   6,  12,  14,  14,  13,  12,  29,   7,   6,  13,  36,  28,  14,  13,   5,  26, },
        {  13,   5,   4,  21,  14,   4,   6,  14,  21,  11,  14,   7,  14,   5,  11,  21,  30,  22,  13,  42, },
        {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   4,   1,   0,   0,   0,   0,   1,   0,   0,   0, },
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
       {  19,   5,   4, },
       {  12,   4,  18, },
       {  12,   4,   3, },
       {   5,   4,   4, },
       })
  };

const CtxSet ContextSetCfg::LastY[] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57, 51, 42, 13, 20,  5, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41 },
        {  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37, 37, 28, 13,  6, 15, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
        { 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20, 19, 51, 15,  6,  4, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
        {  8,  5,  8,  5,  5,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  5,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
      {   5,   5,  20,  13,  13,  19,  21,   6,  12,  12,  14,  14,   5,   4,  12,  13,   7,  13,  12,  41, }, // 20 contexts
        {   5,   5,  12,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  13,  21,  14,  20,  12,  34, },
        {  13,   5,   4,   6,  13,  11,  14,   6,   5,   3,  14,  22,   6,   4,   3,   6,  22,  29,  20,  34, },
        {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   4,   1,   0,   0,   1,   4,   0,   0,   0, },
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
       {  11,   5,  27, },
       {  11,   4,  18, },
       {  12,   4,   3, },
       {   6,   5,   5, },
       })
  };

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
  ({
     {  34, },
     {  34, },
     {  42, },
     {  12, },
     });

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
  ({
     {  28, },
     {  28, },
     { CNU, },
     {   5, },
     });

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
  ({
     {   2, },
     {  60, },
     {  60, },
     {   0, },
     });

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
  ({
     {   2, },
     {   5, },
     {  13, },
     {   4, },
     });

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
  {
    ContextSetCfg::addCtxSet
    ( {
       { 39, },
       { 39, },
       { 39, },
       { DWS, },
       } ),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    ContextSetCfg::addCtxSet
    ( {
       { 39, },
       { 39, },
       { 39, },
       { DWS, },
       } ),
    ContextSetCfg::addCtxSet
    ( {
       { 39, },
       { 39, },
       { 39, },
       { DWS, },
       } )
#endif
  }
#endif

#if JVET_W0066_CCSAO
const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
     { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
     });
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
  ({
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
    { 58, 37, 42, 35 },
      { 43, 45, 42, 35 },
      { 28, 43, 42, 27 },
      {  9, 10,  9, 10 }
#else
    {  52,  37,  27, },
      {  37,  45,  27, },
      {  28,  52,  42, },
      {   9,   9,  10, },
#endif
  });

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
  ({
     {  17, },
     {   0, },
     {  25, },
     {   1, },
     });

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
  ({
     {  35, },
     {  42, },
     {  42, },
     {   5, },
     });

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
  ({
     {  50, },
     {  59, },
     {  42, },
     {   9, },
     });

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
  ({
     {  58,  45,  45,  30,  38, },
     {  51,  30,  30,  38,  23, },
     {  50,  37,  45,  30,  46, },
     {   9,   6,   9,  10,   5, },
     });

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
  ({
     {  45,  38,  46, },
     {  38,  53,  46, },
     {  45,  38,  46, },
     {   0,   9,   5, },
     });

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  17, },
     {  25,   9, },
     {  25,   9, },
     {   1,   1, },
     });
#if JVET_W0103_INTRA_MTS
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
     { 45, 35, 20, 45, },
     { 38, 35, 35, 38, },
     { 37, 28, 28, 37, },
     { 8,   9,  9, 8,  },
     });
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
     {  45,  25,  27,   0, },
     {  45,  40,  27,   0, },
     {  29,   0,  28,   0, },
     {   8,   0,   9,   0, },
     });
#endif
const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
  ({
#if JVET_W0123_TIMD_FUSION
    {  33,  43,  33, },
      {  33,  36,  33, },
      {  33,  43,  33, },
      {   9,   2,   9, },
#else
    {  33,  43, },
      {  33,  36, },
      {  33,  43, },
      {   9,   2, },
#endif
  });

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
  ({
     {  41,  57, },
     {  56,  57, },
     { CNU, CNU, },
     {   1,   5, },
     });

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
  ({
     {  42, },
     {  42, },
     { CNU, },
     {  10, },
     });

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
  ({
     {  35,  51,  27, },
     {  20,  43,  12, },
     { CNU, CNU, CNU, },
     {   8,   4,   1, },
     });

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
  ({
     {  28, },
     {  28, },
     { CNU, },
     {  13, },
     });

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     });
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
  ({
    { 48, 56, 56 },
    { 41, 49, 49 },
    { 33, 49, 49 },
    {  5,  1,  1 }
  });
#endif

#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
  ({
    { 48, 56, 56 },
    { 41, 49, 49 },
    { 33, 49, 49 },
    {  5,  1,  1 }
  });
#endif

#if ENABLE_OBMC
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
  ({
    { 62 },
    { 39 },
    { 35 },
    { 4 }
  });
#endif
const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     });

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
  ({
     {  59,  26,  50,  60,  38, },
     {  59,  48,  58,  60,  60, },
     { CNU,  34, CNU, CNU, CNU, },
     {   0,   5,   0,   0,   4, },
     });

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
  ({
     {  59,  26,  50,  60,  38, },
     {  59,  48,  58,  60,  60, },
     {  59,  26,  50,  60,  38, },
     {   0,   5,   0,   0,   4, },
     });
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
  ({
     {  33,  52,  46,  25,  61,  54,  25,  61,  54, },
     {  13,  23,  46,   4,  61,  54,  19,  46,  54, },
     {  62,  39,  39,  54,  39,  39,  31,  39,  39, },
     {   0,   0,   0,   4,   0,   0,   1,   0,   0, },
     });

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
  ({
#if ALF_IMPROVEMENT
    {  4, 19, 19 },
      { 35, 35, 20 },
      { 26, 19, 19 },
      {  0,  0,  0 }
#else
    {  11,  26, },
      {  20,  12, },
      {  11,  11, },
      {   0,   0, },
#endif
  });

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
  ({
     {  46, },
     {  46, },
     {  46, },
     {   0, },
     });

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  35,  38,  25,  28,  38, },
     {  18,  21,  38,  18,  21,  38, },
     {  18,  30,  31,  18,  30,  31, },
     {   4,   1,   4,   4,   1,   4, },
     });

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
  ({
#if CIIP_PDPC
    { 50, 21 },
      { 50, 36 },
      { 35, 35 },
      {  1,  2 }
#else
    {  57, },
      {  57, },
      { CNU, },
      {   1, },
#endif
  });

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
  ({
     {   0,  43,  45, },
     {   0,  57,  44, },
     {  17,  42,  36, },
     {   1,   5,   8, },
     });

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
  ({
     {  42,  43,  52, },
     {  27,  36,  45, },
     {  12,  21,  35, },
     {   1,   1,   0, },
     });

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
  ({
     {  18,  35,  45, },
     {  18,  12,  29, },
     {  18,  20,  38, },
     {   5,   8,   8, },
     });

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
  ({
     {  25,  50,  37, },
     {  40,  35,  44, },
     {  25,  28,  38, },
     {  13,  13,   8, },
     });

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
  ({
     {  11, },
     {   3, },
     {  11, },
     {   6, },
     });

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
  ({
     { CNU,   3,   4,   4,   5, },
     { CNU,   2,  10,   3,   3, },
     { CNU,  10,   3,   3,   3, },
     { DWS,   1,   1,   1,   1, },
     });

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
  ({
     {  19,  11,   4,   6, },
     {  18,  11,   4,  28, },
     {  11,   5,   5,  14, },
     {   4,   2,   1,   6, },
     });

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
  ({
     {  35,  25,  46,  28,  33,  38, },
     {   5,  10,  53,  43,  25,  46, },
     {  12,  17,  46,  28,  25,  46, },
     {   1,   4,   4,   5,   8,   8, },
     });

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[2] =
  {
    ContextSetCfg::addCtxSet
    ( {
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      {  9,  9,  9,  9 }
    } ),

    ContextSetCfg::addCtxSet
    ( {
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      { 49, 49, 49, 49 },
      {  9,  9,  9,  9 }
    } )
  };
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  DWS, DWS, DWS, DWS, DWS, },
     });
#endif

#if JVET_AA0126_GLM
const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  DWS, DWS, DWS, DWS, DWS, },
     });
#endif

#if JVET_AA0057_CCCM
const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_AC0147_CCCM_NO_SUBSAMPLING && JVET_AC0054_GLCCCM
    { CNU, CNU, CNU, },
      { CNU, CNU, CNU, },
      { CNU, CNU, CNU, },
      { DWS, DWS, DWS, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
    { CNU, CNU, },
      { CNU, CNU, },
      { CNU, CNU, },
      { DWS, DWS, },
#else
    { CNU, },
      { CNU, },
      { CNU, },
      { DWS, },
#endif
  });
#if JVET_AE0100_BVGCCCM
const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     });
#endif
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     });
#endif
#if JVET_AG0058_EIP
const CtxSet ContextSetCfg::EipFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, },
     { CNU, CNU, },
     { CNU, CNU, },
     { DWS, DWS, },
     });
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
const CtxSet ContextSetCfg::CCPMergeFusionFlag = ContextSetCfg::addCtxSet
({
   { CNU },
   { CNU },
   { CNU },
   { DWS },
    });
const CtxSet ContextSetCfg::CCPMergeFusionType = ContextSetCfg::addCtxSet
({
   { CNU },
   { CNU },
   { CNU },
   { DWS },
    });
#endif