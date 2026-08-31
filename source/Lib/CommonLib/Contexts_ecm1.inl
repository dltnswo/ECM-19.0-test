#pragma once
#include "CommonDef.h"

#if SLICE_TYPE_WIN_SIZE
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
  ({
    { 18, 20, 37, 11, 28, 45, 27, 22, 23 },
    { 11, 35, 45, 12, 21, 30, 35, 15, 31 },
    { 27, 36, 38, 35, 29, 38, 28, 38, 31 },
    { 9,  9,  4,  8, 12, 12,  9,  9,  8 },
    { 12, 12,  4, 12, 13, 12,  9,  9, 13 },
    { 12, 13,  9,  9, 13, 13,  6,  9, 13 }
  });

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
  ({
    { 19, 36, 38, 18, 34,  6 },
    { 27, 14, 23, 11, 12,  6 },
    { 19, 13,  7, 33, 27, 22 },
    { 3, 10, 13, 12,  8,  8 },
    { 4,  8, 12, 12, 12,  8 },
    { 0,  5,  7, 12, 13, 12 }
  });

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
  ({
    { 36, 35, 37, 42, 37 },
    { 36, 35, 37, 34, 52 },
    { 43, 50, 29, 27, 52 },
    { 9,  8,  9,  8,  5 },
    { 9,  9, 12,  8,  6 },
    { 9,  9,  9,  8,  5 }
  });

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
  ({
    { 28, 29, 28, 29 },
    { 43, 37, 28, 22 },
    { 36, 45, 36, 45 },
    { 12, 12, 12, 13 },
    { 12, 13, 12, 13 },
    { 12, 13, 12, 13 }
  });
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
  ({
    { 25, 20 },
    { 25, 12 },
    { 35, 35 },
    { 1,  0 },
    { 1,  0 },
    { 1,  0 }
  });
#endif
const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
  ({
    { 50, 60, 53 },
    { 57, 59, 60 },
    { 32, 34, 36 },
    { 5,  4,  8 },
    { 5,  5,  9 },
    { 5, 10,  9 }
  });

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
  ({
    { 6 },
    { 14 },
    { 26 },
    { 4 },
    { 5 },
    { 4 }
  });

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
  ({
    { 31, 15 },
    { 31, 14 },
    { 35, 35 },
    { 9,  4 },
    { 9,  5 },
    { 5,  5 }
  });

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    { 33, 28, 36, 36, 29, 35, 35, 35, 35, 35 },
      { 20, 21, 29, 29, 29, 35, 35, 35, 35, 35 },
      { 34, 43, 36, 35, 25, 35, 35, 35, 35, 35 },
      { 4,  5,  4,  4,  8,  4,  4,  4,  4,  4 },
      { 5,  5,  5,  4,  8,  4,  4,  4,  4,  4 },
      { 5,  4, 10, 13, 13,  4,  4,  4,  4,  4 }
#else
    { 33 },
      { 20 },
      { 34 },
      { 4  },
      { 5  },
    { 5  }
#endif
  });

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
  ({
#if NON_ADJACENT_MRG_CAND
    { 19, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
      { 13, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
      { 34, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
      { 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 }
#else
    { 19 },
      { 13 },
      { 34 },
      { 4  },
      { 4  },
    { 4  }
#endif
  });
#endif

#if JVET_Y0065_GPM_INTRA
const CtxSet ContextSetCfg::GPMIntraFlag = ContextSetCfg::addCtxSet
  ({
    {  35, },
    {  35, },
    {  35, },
    {   4, },
    {   4, },
    {   4, }
  });
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
  ({
    { 25 },
    { 33 },
    { 35 },
    { 5 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
  ({
    { 58 },
    { 43 },
    { 35 },
    { 9 },
    { 10 },
    { 10 }
  });

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    { 35, 35, 35, 35, 35},
      { 35, 35, 35, 35, 35},
      { 35, 35, 35, 35, 35},
      { 4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4 }
#else
    { 59 },
      { 60 },
      { 35 },
      { 0 },
      { 0 },
    { 0 }
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
  ({
    { 59 },
    { 60 },
    { 35 },
    { 0 },
    { 0 },
    { 0 }
  });
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
  ({
    { 25 },
    { 33 },
    { 35 },
    { 5 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
    { 59 },
    { 60 },
    { 35 },
    { 0 },
    { 0 },
    { 0 }
  });
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
const CtxSet ContextSetCfg::GeoBldFlag = ContextSetCfg::addCtxSet
  ({
    { 59 },
    { 60 },
    { 35 },
    { 0 },
    { 0 },
    { 0 }
  });
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
  ({
    { 33, 28, 36, 36, 29, },
    { 20, 21, 29, 29, 29, },
    { 34, 43, 36, 35, 25, },
    {  4,  5,  4,  4,  8, },
    {  5,  5,  5,  4,  8, },
    {  5,  4, 10, 13, 13, }
  });
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
  ({
    { 18 },
    { 11 },
    { 35 },
    { 4 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
  ({
    { 43 },
    { 43 },
    { 35 },
    { 10 },
    { 10 },
    { 10 }
  });

const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    { 35, 35, 35, 35, 35 },
      { 35, 35, 35, 35, 35 },
      { 35, 35, 35, 35, 35 },
      { 4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4 },
      { 4,  4,  4,  4,  4 }
#else
    { 51 },
      { 60 },
      { 35 },
      { 0 },
      { 0 },
    { 0 }
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
  ({
    { 51 },
    { 60 },
    { 35 },
    {  0 },
    {  0 },
    {  0 }
  });
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
  ({
    { 25 },
    { 33 },
    { 35 },
    { 5 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
  ({
    { 58 },
    { 43 },
    { 35 },
    { 9 },
    { 10 },
    { 10 }
  });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
  ({
    { 35, 35, 35, 35, 35},
    { 35, 35, 35, 35, 35},
    { 35, 35, 35, 35, 35},
    { 4,  4,  4,  4,  4 },
    { 4,  4,  4,  4,  4 },
    { 4,  4,  4,  4,  4 }
  });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
  ({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    { 25, 33 },
      { 26, 25 },
      { 35, 35 },
      {  4,  5 },
      {  4,  5 },
      {  4,  4 }
#else
    { 25 },
      { 26 },
      { 35 },
      { 4 },
      { 4 },
    { 4 }
#endif
  });
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
  ({
    { 25 },
    { 26 },
    { 35 },
    { 4 },
    { 4 },
    { 4 }
  });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
  ({
    { 40, 35 },
    { 40, 35 },
    { 35, 35 },
    { 5,  1 },
    { 6,  2 },
    { 5,  1 }
  });

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
  ({
    { CNU },
    { CNU },
    { CNU },
    { DWS },
    { DWS },
    { DWS }
  });
#endif

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
  ({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
    { 25, 59, 59, 59, 59, 25, 59},
      { 25, 58, 58, 58, 58, 25, 58},
      { 25, 60, 60, 60, 60, 25, 60},
      { 6,  5,  5,  5,  5,  6,  5 },
      { 6,  5,  5,  5,  5,  6,  5 },
      { 6,  8,  8,  8,  8,  6,  8 }
#else
    { 25, 59, 59, 59, 59},
      { 25, 58, 58, 58, 58},
      { 25, 60, 60, 60, 60},
      { 6,  5,  5,  5,  5 },
      { 6,  5,  5,  5,  5 },
      { 6,  8,  8,  8,  8 }
#endif
#else
#if JVET_W0123_TIMD_FUSION
    { 25, 59, 25, 59 },
      { 25, 58, 25, 58 },
      { 25, 60, 25, 60 },
      { 6,  5,  6,  5 },
      { 6,  5,  6,  5 },
      { 6,  8,  6,  8 }
#else
    { 25, 59 },
      { 25, 58 },
      { 25, 60 },
      { 6,  5 },
      { 6,  5 },
    { 6,  8 }
#endif
#endif
  });

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
  ({
    { 37 },
    { 29 },
    { 37 },
    { 6 },
    { 6 },
    { 6 }
  });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
  ({
    { 36 },
    { 36 },
    { 44 },
    { 10 },
    { 10 },
    { 7 }
  });
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
  ({
    { 6, 14 },
    { 13, 21 },
    { 14, 29 },
    { 1,  2 },
    { 0,  2 },
    { 1,  5 }
  });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
  ({
    { 20, 21, 13 },
    { 5, 28, 13 },
    { 20, 44, 35 },
    { 1,  2,  5 },
    { 4,  1,  6 },
    { 2,  2,  6 }
  });
#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU, CNU },
     { CNU, CNU, CNU, CNU, CNU },
     { DWS, DWS, DWS, DWS, DWS },
     { DWS, DWS, DWS, DWS, DWS },
     { DWS, DWS, DWS, DWS, DWS },
     });
#endif
#endif

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
  ({
    { 26 },
    { 41 },
    { 59 },
    { 1 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
  ({
    { 34 },
    { 34 },
    { 12 },
    { 8 },
    { 8 },
    { 9 }
  });

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
  ({
    { 25 },
    { 25 },
    { 34 },
    { 5 },
    { 5 },
    { 5 }
  });

#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdChromaMode = ContextSetCfg::addCtxSet
  ( {
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     } );
#endif

const CtxSet ContextSetCfg::ChromaFusionMode = ContextSetCfg::addCtxSet
  ( {
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     } );
#endif

#if JVET_AC0071_DBV
const CtxSet ContextSetCfg::DbvChromaMode = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
     { DWS, },
     });
#endif

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
  ({
    { 56, 57, 50, 33 },
    { 41, 57, 58, 26 },
    { 33, 49, 50, 25 },
    { 9,  9,  9,  6 },
    { 9,  9,  8,  6 },
    { 10, 10,  9,  6 }
  });
#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
  ({
     {  CNU,  CNU,  CNU,  CNU, },
     {  CNU,  CNU,  CNU,  CNU, },
     {  CNU,  CNU,  CNU,  CNU, },
     {  DWS,  DWS,  DWS,  DWS, },
     {  DWS,  DWS,  DWS,  DWS, },
     {  DWS,  DWS,  DWS,  DWS, },
     });
#endif

#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
  ({
    { 46 },
    { 46 },
    { 53 },
    { 8 },
    { 4 },
    { 8 }
  });
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
  ({
    { 35, 35 },
    { 35, 35 },
    { 35, 35 },
    { 8,  8 },
    { 8,  8 },
    { 8,  8 }
  });

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
  ({
#if CTU_256
    { 7,  6,  5, 12, 11,  3, 10, 40 },
      { 7, 21,  5, 12,  4, 18, 18, 48 },
      { 35, 35, 35, 35, 35, 35, 35, 35 },
      { 0,  0,  0,  1,  4,  4,  6,  0 },
      { 0,  0,  0,  1,  4,  8,  5,  0 },
      { 0,  0,  0,  1,  4,  4,  4,  0 }
#else
    {  6,  5, 12, 11,  3, 10, 40 },
      { 21,  5, 12,  4, 18, 18, 48 },
      { 35, 35, 35, 35, 35, 35, 35 },
      {  0,  0,  1,  4,  4,  6,  0 },
      {  0,  0,  1,  4,  8,  5,  0 },
    {  0,  0,  1,  4,  4,  4,  0 }
#endif
  });

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
  ({
    { 12, 20 },
    { 27, 35 },
    { 35, 35 },
    { 0,  4 },
    { 0,  4 },
    { 0,  4 }
  });

#if JVET_Z0054_BLK_REF_PIC_REORDER
const CtxSet ContextSetCfg::RefPicLC = ContextSetCfg::addCtxSet
  ({
    { CNU, CNU, CNU },
    { CNU, CNU, CNU },
    { CNU, CNU, CNU },
    { 0, 2, 4 },
    { 0, 2, 4 },
    { 0, 2, 4 }
  });
#endif

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
  ({
    { 25, 58, 52 },
    { 48, 57, 44 },
    { 35, 35, 35 },
    { 5,  4,  4 },
    { 5,  4,  4 },
    { 4,  4,  4 }
  });

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
  ({
    { 25, CNU, CNU, CNU },
    { 26, CNU, CNU, CNU },
    { 35, CNU, CNU, CNU },
    { 4, 4, 4, 4 },
    { 4, 4, 4, 4 },
    { 4, 4, 4, 4 }
  });
#endif

#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
  ({
    { 25, CNU, CNU, CNU },
    { 26, CNU, CNU, CNU },
    { 35, CNU, CNU, CNU },
    { 4, 4, 4, 4 },
    { 4, 4, 4, 4 },
    { 4, 4, 4, 4 }
  });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
  ({
    { 34, 27,  6 },
    { 26, 12,  6 },
    { 35, 35, 35 },
    { 4,  0,  0 },
    { 5,  1,  0 },
    { 4,  0,  0 }
  });

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
  ({
    { 35 },
    { 42 },
    { 35 },
    { 4 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
  ({
    { 4 },
    { 12 },
    { 35 },
    { 0 },
    { 0 },
    { 0 }
  });

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
  ({
    { 27 },
    { 34 },
    { 35 },
    { 5 },
    { 8 },
    { 8 }
  });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
  ({
    { 12 },
    { 4 },
    { 35 },
    { 0 },
    { 0 },
    { 1 }
  });

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
  ({
    { 51, 43 },
    { 51, 43 },
    { 21, 30 },
    { 9,  5 },
    { 9,  6 },
    { 9, 10 }
  });

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
  ({
    { 53, 38, 38, 29, 20, 34, 27, 45, 37, 43, 34, 48 },
    { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
    { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
    {  1, 12,  8,  4,  2,  5,  3,  4,  0,  0,  5,  4 },
    {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 },
    {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 }
  });
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdx = ContextSetCfg::addCtxSet
  ({
    { 34, 34, 34, 34,},
    { 34, 34, 34, 34,},
    { 34, 34, 34, 34,},
    { 13, 13, 13, 13,},
    { 13, 13, 13, 13,},
    { 13, 13, 13, 13,}
  });
#endif
#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
  ({
    { 3, 26, CNU, },
    { 3, 26, CNU, },
    { 35, 35, CNU, },
    { 1,  4, DWS, },
    { 1,  4, DWS, },
    { 4,  4, DWS, }
  });

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
  ({
    { 28, 50 },
    { 28, 27 },
    { 35, 35 },
    { 1,  0 },
    { 1,  0 },
    { 4,  4 }
  });

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
  ({
    { 50, 35 },
    { 12, 35 },
    { 35, 35 },
    { 0,  4 },
    { 0,  4 },
    { 4,  4 }
  });
#endif

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
  ({
    { 12, 21, 32, 36 },
    { 40, 21, 32,  5 },
    { 11, 50, 32, 19 },
    { 0,  0,  1,  9 },
    { 0,  0,  1,  0 },
    { 1,  4,  8,  3 }
  });

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
  ({
    { 5 },
    { 5 },
    { 13 },
    { 4 },
    { 4 },
    { 4 }
  });

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
  ({
    { 46 },
    { 46 },
    { 52 },
    { 1 },
    { 1 },
    { 1 }
  });

const CtxSet ContextSetCfg::QtCbf[3] =
  {
    ContextSetCfg::addCtxSet
    ({
      { 23, 14,  5, 14 },
      { 23,  5, 20,  7 },
      { 15, 12,  5,  7 },
      { 6,  0,  8,  8 },
      { 6,  0,  8, 10 },
      { 5,  1,  8,  9 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 25, 14 },
      { 25,  6 },
      { 12,  6 },
      { 5,  0 },
      { 5,  3 },
      { 5,  0 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 9, 44, 37 },
      { 25, 29, 37 },
      { 26, 13, 52 },
      { 2,  1,  8 },
      { 2,  1,  8 },
      { 2,  2,  0 }
    }),
    };

const CtxSet ContextSetCfg::SigCoeffGroup[2] =
  {
    ContextSetCfg::addCtxSet
    ({
      { 25, 45 },
      { 25, 45 },
      { 11, 31 },
      { 8,  5 },
      { 8,  5 },
      { 5,  5 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 25, 30 },
      { 25, 45 },
      { 25, 15 },
      { 5,  9 },
      { 9, 12 },
      { 5,  9 }
    }),
    };

const CtxSet ContextSetCfg::SigFlag[6] =
  {
    ContextSetCfg::addCtxSet
    ({
      { 17, 41, 49, 36,  1, 49, 50, 37, 48, 51, 58, 45 },
      { 17, 41, 42, 29, 25, 49, 43, 37, 33, 51, 51, 30 },
      { 25, 19, 28, 14, 18, 28, 29, 30, 19, 45, 30, 38 },
      { 13,  9,  9,  9,  9,  9,  8, 10,  8,  8,  8,  9 },
      { 13,  9,  9,  9,  9,  9,  9, 10,  8,  8,  8,  9 },
      { 13,  9,  9, 10, 10, 10, 10, 13,  9,  9,  9, 10 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 25, 34, 35, 29, 56, 52, 52, 38 },
      { 25, 34, 35, 29, 34, 37, 52, 38 },
      { 25, 27, 28, 37, 42, 53, 53, 46 },
      { 12, 12,  9, 13,  4,  5,  8,  9 },
      { 13, 13, 13, 13,  4,  5,  8,  8 },
      { 12, 12, 10, 13,  5,  5,  9,  9 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 26, 45, 53, 46, 34, 54, 61, 39, 27, 39, 39, 39 },
      { 19, 38, 38, 46, 34, 54, 54, 39,  6, 39, 39, 39 },
      { 11, 38, 46, 54, 27, 39, 39, 39, 36, 39, 39, 39 },
      { 9, 13, 12,  8,  8,  8,  8,  5,  4,  0,  0,  0 },
      { 9, 13,  8,  8,  8,  8,  8,  5,  0,  0,  0,  0 },
      { 9, 13,  8,  8,  8,  8,  8,  5,  4,  0,  0,  0 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 34, 45, 38, 31, 27, 39, 39, 39 },
      { 42, 45, 53, 54, 44, 39, 39, 39 },
      { 19, 46, 38, 39, 52, 39, 39, 39 },
      { 8, 12, 12,  8,  4,  4,  0,  0 },
      { 8, 12, 12,  8,  4,  4,  0,  0 },
      { 8, 12, 12,  8,  0,  0,  0,  1 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 19, 54, 39, 39, 42, 39, 39, 39, 32, 39, 39, 39 },
      { 19, 39, 54, 39, 19, 39, 39, 39, 56, 39, 39, 39 },
      { 18, 39, 39, 39, 27, 39, 39, 39, 32, 39, 39, 39 },
      { 8,  8,  8,  8,  8,  4,  4,  8,  5,  0,  0,  0 },
      { 8,  8,  8,  8,  8,  4,  4,  8,  0,  0,  0,  0 },
      { 8,  8,  8,  8,  8,  0,  4,  4,  5,  0,  0,  0 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 34, 38, 54, 39, 34, 39, 39, 39 },
      { 34, 38, 62, 39, 34, 39, 39, 39 },
      { 11, 39, 39, 39, 34, 39, 39, 39 },
      { 8,  8,  8,  8,  0,  1,  1,  2 },
      { 8,  8,  8,  8,  0,  1,  0,  0 },
      { 8,  8,  8,  8,  4,  0,  2,  4 }
    }),
    };

const CtxSet ContextSetCfg::ParFlag[2] =
  {
    ContextSetCfg::addCtxSet
    ({
      { 33, 40, 33, 26, 34, 42, 25, 33, 34, 34, 27, 25, 34, 42, 42, 35, 33, 27, 35, 42, 35 },
      { 33, 25, 33, 26, 34, 42, 25, 33, 34, 42, 27, 25, 34, 42, 42, 35, 26, 27, 42, 35, 35 },
      { 33, 25, 18, 26, 34, 27, 25, 26, 19, 42, 35, 33, 19, 27, 35, 35, 34, 42, 20, 43, 20 },
      { 12,  9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13,  9, 13, 13, 12, 13 },
      { 13,  9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13, 10, 13, 13, 12, 13 },
      { 9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 33, 25, 34, 34, 34, 27, 33, 42, 35, 35, 35 },
      { 33, 25, 26, 11, 34, 27, 33, 42, 35, 35, 35 },
      { 33, 25, 26, 42, 19, 27, 26, 50, 35, 35, 35 },
      { 12, 13, 12, 12, 13, 13, 13, 13, 12, 13, 13 },
      { 13, 13, 12, 12, 13, 13, 13, 12, 12, 12, 13 },
      { 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13 }
    }),
    };

const CtxSet ContextSetCfg::GtxFlag[4] =
  {
    ContextSetCfg::addCtxSet
    ({
      { 25, 32, 40, 25, 33, 34, 32, 17, 25, 33, 27,  9, 25, 41, 34, 20, 25, 33, 34, 35, 37 },
      { 40, 32, 40, 25, 33, 26, 32,  9, 25, 33, 34,  9, 25, 33, 34, 35, 25, 33, 34, 42, 29 },
      { 25,  1, 40, 25, 33, 19, 17, 25, 25, 18, 12, 17, 33, 26, 19, 13, 33, 19, 20, 28, 22 },
      { 1,  9,  9, 10, 13,  5,  9,  9, 10, 13,  6,  9,  9,  9,  9,  9,  6,  8,  5,  9, 10 },
      { 0,  9,  5,  9, 13,  5, 12, 12, 12, 13,  5, 12,  9,  8,  9,  8,  6,  8,  8,  8,  9 },
      { 4,  9, 10, 10, 10,  5,  9, 12, 10, 10,  6, 13, 10, 10, 10, 10,  7,  9, 10, 10, 10 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 25,  1, 25, 33, 34, 12, 25, 33, 34,  5, 37 },
      { 40,  9, 25,  3, 11,  4, 17, 33, 34, 27, 14 },
      { 40,  1, 25, 18, 34, 27, 25, 34, 35, 36, 37 },
      { 1,  9, 12,  8,  8,  5,  6, 12,  8,  8,  8 },
      { 4,  8,  8,  4,  5,  5,  9, 12,  4,  6,  8 },
      { 2,  9,  9,  8,  8,  6, 10,  9,  8,  8,  9 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 32, 32, 41, 42, 35, 21, 25, 34, 35, 36, 37, 40, 42, 36, 29, 30, 34, 36, 37, 45, 38 },
      { 32, 17, 26, 34, 35, 44, 25, 34, 35, 36, 37, 33, 27, 36, 29, 37, 34, 28, 37, 37, 38 },
      { 25, 25, 11, 27, 20, 29, 33, 12, 28, 21, 22, 34, 28, 29, 29, 30, 36, 29, 45, 30, 23 },
      { 9,  8, 10, 13, 13,  9,  9, 10, 13, 13, 13,  9,  9, 10, 10, 13,  5,  9, 10, 13, 13 },
      { 9,  9,  9, 12, 13,  9,  9, 10, 12, 13, 13,  9,  9, 10, 10, 12,  8,  8, 10,  9, 13 },
      { 9,  9, 10, 13, 13,  9, 13, 10, 13, 13, 13, 10, 10, 10, 10, 13,  9, 10, 10, 10, 13 }
    }),
    ContextSetCfg::addCtxSet
    ({
      { 32, 40, 27, 36, 36, 37, 57, 37, 37, 30, 38 },
      { 1, 25, 27, 20, 13, 14, 42, 37, 37, 37, 23 },
      { 40, 41, 35, 36, 21, 37, 36, 37, 45, 38, 46 },
      { 9,  9, 10, 12, 12, 10,  5,  9,  8,  9, 12 },
      { 9, 12, 12, 12, 12, 10,  5,  8,  5,  6, 12 },
      { 9, 12,  9, 12, 12, 10, 10, 12,  9, 13, 13 }
    }),
    };

const CtxSet ContextSetCfg::LastX[2] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37, 43, 35,  6, 15, 53, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35 }, // 36 contexts
        {  6, 13, 12,  6,  6, 12, 14,  6,  5, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6, 37,  6,  6, 15, 31, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 },
        { 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6, 19,  7, 37,  7, 13, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42 },
        {  5,  4,  4,  5,  4,  1,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
        {  5,  4,  4,  5,  4,  0,  5,  4,  0,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
        {  8,  5,  5,  6,  4,  4,  5,  4,  1,  0,  4,  1,  0,  0,  0,  1,  1,  1,  0,  0,  0,  1,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
      {  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37 }, // 20 contexts
        {  6, 13, 12,  6,  6, 12, 14,  6,  5, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6 },
        { 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6 },
        {  5,  4,  4,  5,  4,  1,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0 },
        {  5,  4,  4,  5,  4,  0,  5,  4,  0,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0 },
      {  8,  5,  5,  6,  4,  4,  5,  4,  1,  0,  4,  1,  0,  0,  0,  1,  1,  1,  0,  0 }
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
      { 26, 12, 11 },
      { 19, 34, 33 },
      { 12,  4,  3 },
      { 4,  4,  4 },
      { 5,  4,  4 },
      { 6,  4,  4 }
    }),
    };

const CtxSet ContextSetCfg::LastY[2] =
  {
    ContextSetCfg::addCtxSet
    ({
#if TU_256
      {  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57, 51, 42, 13, 20,  5, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41 }, // 36 contexts
        {  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37, 37, 28, 13,  6, 15, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
        { 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20, 19, 51,  7, 14,  4, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
        {  5,  4,  4,  5,  4,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  4,  1,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
        {  8,  5,  4,  5,  5,  4,  6,  5,  4,  0,  5,  4,  1,  0,  0,  1,  5,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
        {  9,  5,  8,  6,  5,  4,  6,  5,  4,  0,  5,  5,  1,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
      {  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57 }, // 20 contexts
        {  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37 },
        { 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20 },
        {  5,  4,  4,  5,  4,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  4,  1,  0,  0 },
        {  8,  5,  4,  5,  5,  4,  6,  5,  4,  0,  5,  4,  1,  0,  0,  1,  5,  0,  0,  0 },
      {  9,  5,  8,  6,  5,  4,  6,  5,  4,  0,  5,  5,  1,  0,  0,  1,  1,  0,  0,  0 }
#endif
    }),
    ContextSetCfg::addCtxSet
    ({
      { 26, 20, 34 },
      { 26, 19, 18 },
      { 20,  4,  3 },
      { 2,  4,  4 },
      { 5,  5,  5 },
      { 6,  5,  5 }
    }),
    };

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
  ({
    { 41 },
    { 41 },
    { 42 },
    { 8 },
    { 8 },
    { 13 }
  });

#if JVET_X0083_BM_AMVP_MERGE_MODE
const CtxSet ContextSetCfg::amFlagState = ContextSetCfg::addCtxSet
  ({
     {   34 },
     {   34 },
     {  CNU },
     {    4 },
     {    4 },
     {  DWS },
     });
#endif

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
  ({
    { 28 },
    { 13 },
    { 35 },
    { 4 },
    { 4 },
    { 5 }
  });

#if JVET_AG0098_AMVP_WITH_SBTMVP
const CtxSet ContextSetCfg::amvpSbTmvpFlag = ContextSetCfg::addCtxSet
  ({
    {  33,  33 },
    {  25,  49 },
    {  35,  35 },
    {   2,   1 },
    {   2,   1 },
    {   8,   8 }
  });

const CtxSet ContextSetCfg::amvpSbTmvpMvdIdx = ContextSetCfg::addCtxSet
  ({
    {   4,   4,  59 },
    {  58,  19,  33 },
    {  35,  35,  35 },
    {   3,   2,   3 },
    {   2,   3,   0 },
    {   8,   8,   8 }
  });
#endif

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
  ({
    { 2 },
    { 60 },
    { 59 },
    { 0 },
    { 0 },
    { 1 }
  });

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
  ({
    { 10 },
    { 13 },
    { 6 },
    { 0 },
    { 4 },
    { 4 }
  });

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
  {
    ContextSetCfg::addCtxSet
    ( {
      { 39 },
      { 39 },
      { 39 },
      { DWS },
      { DWS },
      { DWS }
    } ),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    ContextSetCfg::addCtxSet
    ( {
      { 39 },
      { 39 },
      { 39 },
      { DWS },
      { DWS },
      { DWS }
    } ),
    ContextSetCfg::addCtxSet
    ( {
      { 39 },
      { 39 },
      { 39 },
      { DWS },
      { DWS },
      { DWS }
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
     { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
     { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
     });
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
  ({
#if INTRA_TRANS_ENC_OPT
    { 51, CNU,  50,  35, },
      { 36, CNU,  43,  35, },
      { CNU,  51,  43,  42, },
      { 10, DWS,   8,  13, },
      { 10, DWS,   6,  13, },
      { DWS,  10,   9,  10, }
#elif EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
    { 58, 37, 42, 35 },
      { 43, 45, 42, 35 },
      { 28, 43, 42, 27 },
      {  9,  9,  9, 10 },
      {  9,  9,  6, 13 },
      {  9, 10,  9, 10 }
#else
    { 58, 37, 42 },
      { 43, 45, 42 },
      { 28, 43, 42 },
      {  9,  9,  9 },
      {  9,  9,  6 },
    {  9, 10,  9 }
#endif
  });

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
  ({
    { 17 },
    { 0 },
    { 25 },
    { 1 },
    { 1 },
    { 1 }
  });

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
  ({
    { 35 },
    { 42 },
    { 42 },
    { 5 },
    { 5 },
    { 5 }
  });

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
  ({
    { 50 },
    { 59 },
    { 42 },
    { 9 },
    { 9 },
    { 9 }
  });

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
  ({
    { 58, 45, 45, 30, 38 },
    { 51, 30, 30, 38, 23 },
    { 50, 37, 45, 30, 46 },
    { 9,  6,  9, 10,  5 },
    { 9,  6,  9, 10,  5 },
    { 9,  6,  9, 10,  5 }
  });

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
  ({
    { 45, 38, 46 },
    { 38, 53, 46 },
    { 45, 38, 46 },
    { 0,  9,  5 },
    { 0,  9,  5 },
    { 0,  9,  5 }
  });

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
  ({
    { 25, 17 },
    { 25,  1 },
    { 25,  9 },
    { 0,  1 },
    { 1,  5 },
    { 2,  5 }
  });

#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
    { 43, 38, 46, 38 },
    { 36, 38, 46, 38 },
    { 35, 38, 38, 38 },
    {  8,  9,  9,  8 },
    { 12,  8,  8,  8 },
    {  9,  9,  9,  9 }
  });
#elif INTRA_TRANS_ENC_OPT
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
    { 38,  42,  27,  45, },
    { 31,  27,  27,  38, },
    { 45,  28,  28,  37, },
    { 8,   8,   9,   8, },
    { 8,   8,   9,   8, },
    { 9,   9,  10,   8, }
  });
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
    { 45, 35, 20, 45, },
    { 38, 35, 35, 38, },
    { 37, 28, 28, 37, },
    { 8,  10, 10, 8,  },
    { 8,  10, 10, 8,  },
    { 9,  10, 10, 8,  }
  });
#endif
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
  ({
    { 37, 25, 34, 40 },
    { 45, 40, 42, 32 },
    { 36, 32, 28, 32 },
    { 9,  0,  4,  0 },
    { 9,  0, 10,  8 },
    { 9,  1,  9,  0 }
  });
#endif

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
  ({
#if JVET_W0123_TIMD_FUSION
    { 33, 43, 33 },
      { 33, 43, 33 },
      { 33, 43, 33 },
      { 9,  2,  9 },
      { 9,  3,  9 },
      { 9,  2,  9 }
#else
    { 33, 43 },
      { 33, 43 },
      { 33, 43 },
      { 9,  2 },
      { 9,  3 },
    { 9,  2 }
#endif
  });

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
  ({
    { 33, 57 },
    { 56, 57 },
    { 35, 35 },
    { 1,  5 },
    { 1,  5 },
    { 1,  5 }
  });

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
  ({
    { 42 },
    { 42 },
    { 35 },
    { 10 },
    { 10 },
    { 10 }
  });

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
  ({
    { 35, 51, 27 },
    { 20, 58, 34 },
    { 35, 35, 35 },
    { 8,  5,  2 },
    { 5,  4,  4 },
    { 8,  4,  1 }
  });

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
  ({
    { 35 },
    { 28 },
    { 35 },
    { 12 },
    { 13 },
    { 13 }
  });

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
  ({
    { 35 },
    { 35 },
    { 35 },
    { 8 },
    { 8 },
    { 8 }
  });

#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
  ({
#if INTRA_TRANS_ENC_OPT
    { 48, CNU, CNU, },
      { 48, CNU, CNU, },
      { 33, CNU, CNU, },
      { 6, DWS, DWS, },
      { 6, DWS, DWS, },
      { 3, DWS, DWS, }
#else
    { 48, 56, 56 },
      { 41, 49, 49 },
      { 33, 49, 49 },
      { 5,  1,  1 },
      { 5,  1,  1 },
    { 2,  1,  1 }
#endif
  });
#endif


#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
  ({
#if INTRA_TRANS_ENC_OPT
    { 41,  49,  49, },
      { 34,  34,  34, },
      { 34,  42,  50, },
      { 6,   6,   5, },
      { 7,   6,   5, },
      { 6,   5,   2, }
#else
    { 48, 56, 56 },
      { 41, 49, 49 },
      { 33, 49, 49 },
      { 5,  1,  1 },
      { 5,  1,  1 },
    { 2,  1,  1 }
#endif
  });

#endif
#if ENABLE_OBMC
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
  ({
    { 62 },
    { 39 },
    { 35 },
    { 0 },
    { 8 },
    { 0 }
  });
#endif

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
  ({
    { 35 },
    { 35 },
    { 35 },
    { 8 },
    { 8 },
    { 8 }
  });

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
  ({
    { 59, 26, 50, 60, 38 },
    { 59, 48, 58, 60, 60 },
    { 35, 34, 35, 35, 35 },
    { 1,  4,  1,  0,  4 },
    { 0,  5,  1,  0,  4 },
    { 0, 10,  0,  0,  4 }
  });

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
  ({
     { 59, 26, 50, 60, 38 },
     { 59, 48, 58, 60, 60 },
     { 59, 26, 50, 60, 38 },
     { 1,  4,  1,  0,  4 },
     { 0,  5,  1,  0,  4 },
     { 1,  4,  1,  0,  4 },
     });
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
  ({
    { 18, 37, 46, 25, 53, 54, 25, 46, 54 },
    { 5, 23, 46,  4, 53, 46, 34, 46, 46 },
    { 45, 39, 39, 22, 31, 39, 37, 31, 39 },
    { 5,  0,  0, 10,  0,  3,  7,  0,  6 },
    { 0,  0,  0,  4,  0,  0,  0,  0,  0 },
    { 3,  3, 13,  0,  1, 10,  4,  1, 10 }
  });

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
  ({
#if ALF_IMPROVEMENT
    {  4, 19, 19 },
      { 35, 35, 20 },
      { 26, 19, 19 },
      { 0,  0,  0 },
      { 0,  0,  0 },
      { 0,  0,  0 }
#else
    { 19, 19 },
      { 35, 20 },
      { 19, 19 },
      {  0,  0 },
      {  0,  0 },
    {  0,  0 }
#endif
  });

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
  ({
    { 46 },
    { 53 },
    { 31 },
    { 0 },
    { 0 },
    { 1 }
  });

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
  ({
    { 25, 20, 38, 25, 20, 38 },
    { 18,  6, 38, 18,  6, 38 },
    { 34, 38, 31, 34, 46, 31 },
    { 5,  1,  4,  4,  3,  4 },
    { 4,  0,  0,  4,  0,  0 },
    { 0,  3,  8,  0,  0,  8 }
  });

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
  ({
#if CIIP_PDPC
    { 57, 21 },
      { 50, 36 },
      { 35, 35 },
      { 0,  1 },
      { 1,  2 },
      { 1,  1 }
#else
    { 57 },
      { 50 },
      { 35 },
      {  0 },
      {  1 },
    {  1 }
#endif
  });

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
  ({
    { 32, 50, 37 },
    { 32, 42, 29 },
    { 40, 27, 36 },
    { 5,  4,  4 },
    { 5,  5,  4 },
    { 5,  6,  8 }
  });

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
  ({
    { 42, 43, 45 },
    { 27, 36, 45 },
    { 20, 29, 58 },
    { 1,  0,  0 },
    { 1,  0,  0 },
    { 1,  1,  0 }
  });

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
  ({
    { 18, 35, 37 },
    { 18, 12, 37 },
    { 18, 20, 38 },
    { 5,  8,  4 },
    { 6,  9,  6 },
    { 5,  8,  9 }
  });

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
  ({
    { 25, 35, 37 },
    { 40, 35, 44 },
    { 25, 28, 38 },
    { 13, 12,  5 },
    { 13, 12,  5 },
    { 13, 13, 10 }
  });

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
  ({
    { 11 },
    { 18 },
    { 11 },
    { 5 },
    { 6 },
    { 10 }
  });

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
  ({
    { 35,  3,  4,  4,  5 },
    { 35,  2, 18, 34, 11 },
    { 35, 10,  3,  3,  3 },
    { 8,  1,  0,  0,  0 },
    { 8,  2,  1,  0,  0 },
    { 8,  1,  1,  1,  1 }
  });

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
  ({
    { 34, 11,  4, 14 },
    { 18, 11,  4, 21 },
    { 19,  5, 13, 14 },
    { 1,  1,  1,  2 },
    { 4,  2,  1,  0 },
    { 5,  2,  2,  7 }
  });

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
  ({
    { 28, 25, 46, 28, 33, 38 },
    { 5, 10, 53, 50, 25, 46 },
    { 20, 17, 46, 28, 25, 46 },
    { 1,  4,  4,  6,  4,  8 },
    { 5,  5,  4,  0,  8,  9 },
    { 1,  4,  5,  9,  8,  8 }
  });

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[2] =
  {
#if JVET_Y0141_SIGN_PRED_IMPROVE
    ContextSetCfg::addCtxSet
    ( {
      { 34,  34,  19,  26, },
      { 34,  34,  34,  26, },
      { 34,  34,  34,  26, },
      { 13,  13,  13,  10, },
      { 13,  13,  13,  10, },
      { 13,  10,  10,  10, }
    } ),
    ContextSetCfg::addCtxSet
    ( {
      { 34,  34,  19,  26, },
      { 34,  41,  34,  41, },
      { 34,  34, CNU, CNU, },
      { 13,  13,   9,   8, },
      { 13,  13,  10,  10, },
      { 13,  13, DWS, DWS, }
    } )
#else
    ContextSetCfg::addCtxSet
    ( {
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      {  9,  9,  9,  9 },
      {  9,  9,  9,  9 },
      { 10, 10, 10, 10 }
    } ),

    ContextSetCfg::addCtxSet
    ( {
      { 34, 34, 34, 34 },
      { 34, 34, 34, 34 },
      { 49, 49, 49, 49 },
      {  9,  9,  9,  9 },
      {  9,  9,  9,  9 },
      {  9,  9,  9,  9 }
    } )
#endif
  };
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
  ({
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  CNU, CNU, CNU, CNU, CNU, },
     {  DWS, DWS, DWS, DWS, DWS, },
     {  DWS, DWS, DWS, DWS, DWS, },
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
     {  DWS, DWS, DWS, DWS, DWS, },
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
      { DWS, DWS, DWS, },
      { DWS, DWS, DWS, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
    { CNU, CNU, },
      { CNU, CNU, },
      { CNU, CNU, },
      { DWS, DWS, },
      { DWS, DWS, },
      { DWS, DWS, },
#else
    { CNU, },
      { CNU, },
      { CNU, },
      { DWS, },
      { DWS, },
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
     { DWS, },
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
     { DWS, },
     { DWS, },
     });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
  ({
     { CNU, },
     { CNU, },
     { CNU, },
     { DWS, },
     { DWS, },
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
     { DWS, DWS, },
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
   { DWS },
   { DWS },
    });
const CtxSet ContextSetCfg::CCPMergeFusionType = ContextSetCfg::addCtxSet
({
   { CNU },
   { CNU },
   { CNU },
   { DWS },
   { DWS },
   { DWS },
    });
#endif
#endif