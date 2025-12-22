/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_Unified_args.c
 *
 * Code generation for function 'stateDeriv_withGrav_LiamSet_Unified_args'
 *
 */

/* Include files */
#include "stateDeriv_withGrav_LiamSet_Unified_args.h"
#include "assertCompatibleDims.h"
#include "calculateAdaptiveControl_Unified.h"
#include "div.h"
#include "dot.h"
#include "mldivide.h"
#include "norm.h"
#include "rotm2quat.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_emxutil.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"
#include "sum.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    252,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    237,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    236,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    223,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    220,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    219,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    218,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    162,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    144,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    143,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    142,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo l_emlrtRSI = {
    139,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo m_emlrtRSI = {
    137,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo n_emlrtRSI = {
    127,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo o_emlrtRSI = {
    121,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo p_emlrtRSI = {
    88,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    84,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    83,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    56,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    55,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    54,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    52,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    51,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    50,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRSInfo kb_emlrtRSI = {
    34,               /* lineNo */
    "rdivide_helper", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\rdivide_"
    "helper.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    53,    /* lineNo */
    "div", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\div.m" /* pathName
                                                                          */
};

static emlrtRSInfo xe_emlrtRSI = {
    41,    /* lineNo */
    "cat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtRSInfo ye_emlrtRSI = {
    65,         /* lineNo */
    "cat_impl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                          */
};

static emlrtECInfo emlrtECI = {
    -1,                                         /* nDims */
    260,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    260,                                        /* lineNo */
    67,                                         /* colNo */
    "ds",                                       /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtECInfo b_emlrtECI = {
    -1,                                         /* nDims */
    253,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    253,                                        /* lineNo */
    20,                                         /* colNo */
    "ds",                                       /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    229,                                        /* lineNo */
    49,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    184,                                        /* lineNo */
    29,                                         /* colNo */
    "nodes_vel",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    183,                                        /* lineNo */
    29,                                         /* colNo */
    "nodes_pos",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    122,                                        /* lineNo */
    44,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    120,                                        /* lineNo */
    28,                                         /* colNo */
    "nodes_pos",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtRTEInfo emlrtRTEI = {
    119,                                        /* lineNo */
    13,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo h_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    115,                                        /* lineNo */
    29,                                         /* colNo */
    "activeMask",                               /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    114,                                        /* lineNo */
    28,                                         /* colNo */
    "Tvecs",                                    /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    113,                                        /* lineNo */
    29,                                         /* colNo */
    "E_vecs",                                   /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    112,                                        /* lineNo */
    28,                                         /* colNo */
    "VR_vecs",                                  /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    111,                                        /* lineNo */
    24,                                         /* colNo */
    "L_mags",                                   /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtECInfo c_emlrtECI = {
    -1,                                         /* nDims */
    107,                                        /* lineNo */
    9,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo m_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    107,                                        /* lineNo */
    26,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    107,                                        /* lineNo */
    24,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtECInfo d_emlrtECI = {
    2,                                          /* nDims */
    107,                                        /* lineNo */
    35,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo o_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    107,                                        /* lineNo */
    52,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    107,                                        /* lineNo */
    50,                                         /* colNo */
    "Force_nodes",                              /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    105,                                        /* lineNo */
    58,                                         /* colNo */
    "Tvecs",                                    /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtECInfo e_emlrtECI = {
    2,                                          /* nDims */
    102,                                        /* lineNo */
    19,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo f_emlrtECI = {
    2,                                          /* nDims */
    98,                                         /* lineNo */
    13,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo g_emlrtECI = {
    2,                                          /* nDims */
    95,                                         /* lineNo */
    18,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo h_emlrtECI = {
    2,                                          /* nDims */
    90,                                         /* lineNo */
    13,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo i_emlrtECI = {
    2,                                          /* nDims */
    88,                                         /* lineNo */
    19,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo j_emlrtECI = {
    2,                                          /* nDims */
    85,                                         /* lineNo */
    15,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo k_emlrtECI = {
    2,                                          /* nDims */
    82,                                         /* lineNo */
    14,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo r_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    75,                                         /* lineNo */
    39,                                         /* colNo */
    "nodes_vel",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    75,                                         /* lineNo */
    37,                                         /* colNo */
    "nodes_vel",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    29,                                         /* lineNo */
    21,                                         /* colNo */
    "nodes_vel",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    74,                                         /* lineNo */
    39,                                         /* colNo */
    "nodes_pos",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    74,                                         /* lineNo */
    37,                                         /* colNo */
    "nodes_pos",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo w_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    28,                                         /* lineNo */
    21,                                         /* colNo */
    "nodes_pos",                                /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtDCInfo emlrtDCI = {
    64,                                         /* lineNo */
    28,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = {
    36,                                         /* lineNo */
    19,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    26,                                         /* lineNo */
    13,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtECInfo l_emlrtECI = {
    -1,                                         /* nDims */
    14,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtBCInfo x_emlrtBCI = {
    1,                                          /* iFirst */
    95,                                         /* iLast */
    14,                                         /* lineNo */
    12,                                         /* colNo */
    "s",                                        /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = {
    14,                                         /* lineNo */
    12,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtBCInfo y_emlrtBCI = {
    1,                                          /* iFirst */
    95,                                         /* iLast */
    14,                                         /* lineNo */
    40,                                         /* colNo */
    "s",                                        /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtDCInfo d_emlrtDCI = {
    14,                                         /* lineNo */
    40,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtRTEInfo c_emlrtRTEI = {
    225,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtDCInfo e_emlrtDCI = {
    23,                                         /* lineNo */
    26,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = {
    23,                                         /* lineNo */
    26,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    4                                  /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    24,                                         /* lineNo */
    26,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    1                                  /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    97,                                         /* lineNo */
    11,                                         /* colNo */
    "Tmags",                                    /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = {
    -1,                                         /* iFirst */
    -1,                                         /* iLast */
    131,                                        /* lineNo */
    5,                                          /* colNo */
    "ds",                                       /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = {
    1,                                          /* iFirst */
    95,                                         /* iLast */
    29,                                         /* lineNo */
    28,                                         /* colNo */
    "s",                                        /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = {
    1,                                          /* iFirst */
    95,                                         /* iLast */
    28,                                         /* lineNo */
    28,                                         /* colNo */
    "s",                                        /* aName */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m", /* pName */
    0                                  /* checkKind */
};

static emlrtRTEInfo i_emlrtRTEI = {
    23,                                         /* lineNo */
    17,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    24,                                         /* lineNo */
    17,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo k_emlrtRTEI = {
    36,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo l_emlrtRTEI = {
    74,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo m_emlrtRTEI = {
    75,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo n_emlrtRTEI = {
    82,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo o_emlrtRTEI =
    {
        31,            /* lineNo */
        30,            /* colNo */
        "unsafeSxfun", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\unsafeSxfun.m" /* pName */
};

static emlrtRTEInfo q_emlrtRTEI = {
    84,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo r_emlrtRTEI = {
    85,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo s_emlrtRTEI = {
    87,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo t_emlrtRTEI = {
    88,                                         /* lineNo */
    19,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo u_emlrtRTEI = {
    90,                                         /* lineNo */
    13,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo v_emlrtRTEI = {
    90,                                         /* lineNo */
    33,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo w_emlrtRTEI = {
    90,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo x_emlrtRTEI = {
    93,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo y_emlrtRTEI = {
    94,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = {
    95,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = {
    98,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = {
    102,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo db_emlrtRTEI = {
    107,                                        /* lineNo */
    35,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = {
    232,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo fb_emlrtRTEI = {
    252,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = {
    260,                                        /* lineNo */
    52,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo hb_emlrtRTEI = {
    262,                                        /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo ib_emlrtRTEI = {
    23,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo jb_emlrtRTEI = {
    24,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = {
    64,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo lb_emlrtRTEI = {
    83,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo mb_emlrtRTEI = {
    88,                                         /* lineNo */
    5,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = {
    1,                                          /* lineNo */
    11,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo ob_emlrtRTEI = {
    107,                                        /* lineNo */
    9,                                          /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo rb_emlrtRTEI = {
    95,                                         /* lineNo */
    18,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRTEInfo sb_emlrtRTEI = {
    85,                                         /* lineNo */
    15,                                         /* colNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pName */
};

static emlrtRSInfo bf_emlrtRSI = {
    95,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo cf_emlrtRSI = {
    90,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo df_emlrtRSI = {
    85,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo ef_emlrtRSI = {
    82,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo ff_emlrtRSI = {
    54,    /* lineNo */
    "div", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\div.m" /* pathName
                                                                          */
};

static emlrtRSInfo gf_emlrtRSI = {
    98,                                         /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo hf_emlrtRSI = {
    102,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

static emlrtRSInfo if_emlrtRSI = {
    107,                                        /* lineNo */
    "stateDeriv_withGrav_LiamSet_Unified_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_Unified_args.m" /* pathName */
};

/* Function Declarations */
static void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
                  const emxArray_boolean_T *in2);

static void binary_expand_op(const emlrtStack *sp, emxArray_real_T *in1,
                             const emxArray_real_T *in2, int32_T in3,
                             const emxArray_real_T *in4, int32_T in5,
                             int32_T in6);

static void binary_expand_op_1(const emlrtStack *sp, emxArray_real_T *in1,
                               const struct0_T *in2,
                               const emxArray_real_T *in3);

static void binary_expand_op_2(const emlrtStack *sp, emxArray_real_T *in1,
                               const emlrtRSInfo in2,
                               const emxArray_real_T *in3,
                               const emxArray_real_T *in4);

static void d_stateDeriv_withGrav_LiamSet_U(const real_T x[3],
                                            real_T varargout_1[3]);

static void minus(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2);

static void plus(const emlrtStack *sp, emxArray_real_T *in1,
                 const emxArray_real_T *in2);

static void times(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2, const emxArray_real_T *in3);

/* Function Definitions */
static void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
                  const emxArray_boolean_T *in2)
{
  emxArray_boolean_T *b_in1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  const boolean_T *in2_data;
  boolean_T *b_in1_data;
  boolean_T *in1_data;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &b_in1, &rb_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 1;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_in1, i, &rb_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = (in1_data[i * stride_0_1] && in2_data[i * stride_1_1]);
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, i, &rb_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_boolean_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void binary_expand_op(const emlrtStack *sp, emxArray_real_T *in1,
                             const emxArray_real_T *in2, int32_T in3,
                             const emxArray_real_T *in4, int32_T in5,
                             int32_T in6)
{
  const real_T *in2_data;
  const real_T *in4_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in4_data = in4->data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 3;
  emxEnsureCapacity_real_T(sp, in1, i, &db_emlrtRTEI);
  i = in6 - in5;
  if (i == 1) {
    loop_ub = in3;
  } else {
    loop_ub = i;
  }
  i1 = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i1, &db_emlrtRTEI);
  in1_data = in1->data;
  stride_0_1 = (in3 != 1);
  stride_1_1 = (i != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in2_data[3 * aux_0_1]);
    i1 = in5 + aux_1_1;
    r1 = _mm_loadu_pd(&in4_data[3 * i1]);
    _mm_storeu_pd(&in1_data[3 * i], _mm_add_pd(r, r1));
    in1_data[3 * i + 2] = in2_data[3 * aux_0_1 + 2] + in4_data[3 * i1 + 2];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

static void binary_expand_op_1(const emlrtStack *sp, emxArray_real_T *in1,
                               const struct0_T *in2, const emxArray_real_T *in3)
{
  const real_T *in3_data;
  real_T *in1_data;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_1_1;
  in3_data = in3->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 3;
  emxEnsureCapacity_real_T(sp, in1, i, &cb_emlrtRTEI);
  if (in3->size[1] == 1) {
    loop_ub = (int32_T)in2->N_mt_nodes;
  } else {
    loop_ub = in3->size[1];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i, &cb_emlrtRTEI);
  in1_data = in1->data;
  stride_1_1 = (in3->size[1] != 1);
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    r = _mm_loadu_pd(&in3_data[3 * aux_1_1]);
    _mm_storeu_pd(&in1_data[3 * i], _mm_sub_pd(_mm_set1_pd(0.0), r));
    in1_data[3 * i + 2] = 0.0 - in3_data[3 * aux_1_1 + 2];
    aux_1_1 += stride_1_1;
  }
}

static void binary_expand_op_2(const emlrtStack *sp, emxArray_real_T *in1,
                               const emlrtRSInfo in2,
                               const emxArray_real_T *in3,
                               const emxArray_real_T *in4)
{
  emlrtStack st;
  emxArray_real_T *b_in3;
  const real_T *in3_data;
  const real_T *in4_data;
  real_T *b_in3_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  st.prev = sp;
  st.tls = sp->tls;
  in4_data = in4->data;
  in3_data = in3->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in3, 2, &t_emlrtRTEI);
  i = b_in3->size[0] * b_in3->size[1];
  b_in3->size[0] = 3;
  if (in4->size[1] == 1) {
    loop_ub = in3->size[1];
  } else {
    loop_ub = in4->size[1];
  }
  b_in3->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in3, i, &t_emlrtRTEI);
  b_in3_data = b_in3->data;
  stride_0_1 = (in3->size[1] != 1);
  stride_1_1 = (in4->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in3_data[3 * aux_0_1]);
    r1 = _mm_loadu_pd(&in4_data[3 * aux_1_1]);
    _mm_storeu_pd(&b_in3_data[3 * i], _mm_mul_pd(r, r1));
    b_in3_data[3 * i + 2] =
        in3_data[3 * aux_0_1 + 2] * in4_data[3 * aux_1_1 + 2];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  st.site = (emlrtRSInfo *)&in2;
  sum(&st, b_in3, in1);
  emxFree_real_T(sp, &b_in3);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void d_stateDeriv_withGrav_LiamSet_U(const real_T x[3],
                                            real_T varargout_1[3])
{
  __m128d r;
  real_T dv[2];
  _mm_storeu_pd(&dv[0], _mm_div_pd(_mm_loadu_pd(&x[0]), _mm_set1_pd(0.001)));
  dv[0] = muDoubleScalarMin(muDoubleScalarMax(dv[0], -1.0), 1.0);
  dv[1] = muDoubleScalarMin(muDoubleScalarMax(dv[1], -1.0), 1.0);
  r = _mm_loadu_pd(&dv[0]);
  _mm_storeu_pd(&varargout_1[0], r);
  varargout_1[2] =
      muDoubleScalarMin(muDoubleScalarMax(x[2] / 0.001, -1.0), 1.0);
}

static void minus(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2)
{
  emxArray_real_T *b_in2;
  const real_T *in2_data;
  real_T *b_in2_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in2, 2, &sb_emlrtRTEI);
  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 3;
  if (in1->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in1->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in2, i, &sb_emlrtRTEI);
  b_in2_data = b_in2->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&in2_data[3 * aux_0_1]);
    r1 = _mm_loadu_pd(&in1_data[3 * aux_1_1]);
    _mm_storeu_pd(&b_in2_data[3 * i], _mm_sub_pd(r, r1));
    b_in2_data[3 * i + 2] =
        in2_data[3 * aux_0_1 + 2] - in1_data[3 * aux_1_1 + 2];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 3;
  emxEnsureCapacity_real_T(sp, in1, i, &sb_emlrtRTEI);
  loop_ub = b_in2->size[1];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in2->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &sb_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < loop_ub; i++) {
    in1_data[3 * i] = b_in2_data[3 * i];
    stride_0_1 = 3 * i + 1;
    in1_data[stride_0_1] = b_in2_data[stride_0_1];
    stride_0_1 = 3 * i + 2;
    in1_data[stride_0_1] = b_in2_data[stride_0_1];
  }
  emxFree_real_T(sp, &b_in2);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void plus(const emlrtStack *sp, emxArray_real_T *in1,
                 const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &u_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 1;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &u_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = in1_data[i * stride_0_1] + in2_data[i * stride_1_1];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i, &u_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void times(const emlrtStack *sp, emxArray_real_T *in1,
                  const emxArray_real_T *in2, const emxArray_real_T *in3)
{
  const real_T *in2_data;
  const real_T *in3_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in3_data = in3->data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 3;
  emxEnsureCapacity_real_T(sp, in1, i, &bb_emlrtRTEI);
  if (in3->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in3->size[1];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, in1, i, &bb_emlrtRTEI);
  in1_data = in1->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    r = _mm_loadu_pd(&in2_data[3 * aux_0_1]);
    _mm_storeu_pd(&in1_data[3 * i],
                  _mm_mul_pd(r, _mm_set1_pd(in3_data[aux_1_1])));
    in1_data[3 * i + 2] = in3_data[aux_1_1] * in2_data[3 * aux_0_1 + 2];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

void stateDeriv_withGrav_LiamSet_Unified_args(const emlrtStack *sp, real_T t,
                                              real_T s[95],
                                              const struct0_T *args,
                                              emxArray_real_T *ds)
{
  static const int8_T c_a[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const int8_T iv[3] = {0, 0, -1};
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r4;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T *maskLength;
  emxArray_boolean_T *maskTension;
  emxArray_real_T *Force_nodes;
  emxArray_real_T *L_mags;
  emxArray_real_T *P_prev;
  emxArray_real_T *Tmags;
  emxArray_real_T *Tvecs;
  emxArray_real_T *V_prev;
  emxArray_real_T *b_P_prev;
  emxArray_real_T *b_ds;
  emxArray_real_T *deltas;
  emxArray_real_T *nodes_pos;
  emxArray_real_T *nodes_vel;
  emxArray_real_T *r3;
  emxArray_real_T *temp_node_states;
  emxArray_real_T *vr_dots;
  real_T tmp_data[95];
  real_T Qmat_desCh[12];
  real_T Tvec_st[12];
  real_T b_tmp[12];
  real_T q_eCh_tmp[12];
  real_T rotMat_C_A_I[9];
  real_T rotMat_D_A_I[9];
  real_T y_tmp[9];
  real_T quar_CDot[4];
  real_T quar_TDot[4];
  real_T quat_desCh[4];
  real_T accChaser[3];
  real_T b[3];
  real_T b_s[3];
  real_T distAttPt_to_C[3];
  real_T i_Loc[3];
  real_T k_Loc[3];
  real_T slidingVec_tmp[3];
  real_T tau_Att_Ch_tmp[3];
  real_T Qmat_desCh_tmp;
  real_T a;
  real_T accTarget_idx_0;
  real_T appTorqueTarget_idx_0;
  real_T appTorqueTarget_idx_1;
  real_T appTorqueTarget_idx_2;
  real_T b_a;
  real_T b_rotMat_C_A_I_tmp;
  real_T c_rotMat_C_A_I_tmp;
  real_T coeffT;
  real_T d;
  real_T d1;
  real_T d_rotMat_C_A_I_tmp;
  real_T e_rotMat_C_A_I_tmp;
  real_T idx;
  real_T m_node;
  real_T rotMat_C_A_I_tmp;
  real_T targetSideLengthY;
  real_T *L_mags_data;
  real_T *P_prev_data;
  real_T *Tmags_data;
  real_T *V_prev_data;
  real_T *deltas_data;
  real_T *ds_data;
  real_T *nodes_pos_data;
  real_T *nodes_vel_data;
  real_T *vr_dots_data;
  int32_T input_sizes[2];
  int32_T Force_nodes_tmp;
  int32_T b_i;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  int8_T input_sizes_idx_0;
  int8_T sizes_idx_0;
  boolean_T empty_non_axis_sizes;
  boolean_T *maskLength_data;
  boolean_T *maskTension_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  /*     %% Achira Boonrath */
  targetSideLengthY = args->targetSideLengthY;
  /* Extract States */
  r = _mm_loadu_pd(&s[0]);
  r1 = _mm_set1_pd(args->ODEscale);
  _mm_storeu_pd(&s[0], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[13]);
  _mm_storeu_pd(&s[13], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[2]);
  _mm_storeu_pd(&s[2], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[15]);
  _mm_storeu_pd(&s[15], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[4]);
  _mm_storeu_pd(&s[4], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[17]);
  _mm_storeu_pd(&s[17], _mm_mul_pd(r, r1));
  d = 6.0 * args->N_mt_nodes;
  if (d + 26.0 < 27.0) {
    i = 0;
    i1 = 0;
    i2 = 0;
    b_i = 0;
  } else {
    i = 26;
    d1 = (int32_T)muDoubleScalarFloor(d + 26.0);
    if (d + 26.0 != d1) {
      emlrtIntegerCheckR2012b(d + 26.0, &d_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)(d + 26.0) < 1) || ((int32_T)(d + 26.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(d + 26.0), 1, 95, &y_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i1 = (int32_T)(d + 26.0);
    i2 = 26;
    if (d + 26.0 != d1) {
      emlrtIntegerCheckR2012b(d + 26.0, &c_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)(d + 26.0) < 1) || ((int32_T)(d + 26.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(d + 26.0), 1, 95, &x_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_i = (int32_T)(d + 26.0);
  }
  loop_ub = i1 - i;
  scalarLB = (loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i1 = 0; i1 <= vectorUB; i1 += 2) {
    r = _mm_loadu_pd(&s[i + i1]);
    _mm_storeu_pd(&tmp_data[i1], _mm_mul_pd(r, r1));
  }
  for (i1 = scalarLB; i1 < loop_ub; i1++) {
    tmp_data[i1] = s[i + i1] * args->ODEscale;
  }
  b_loop_ub = b_i - i2;
  if (b_loop_ub != loop_ub) {
    emlrtSubAssignSizeCheck1dR2017a(b_loop_ub, loop_ub, &l_emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  for (i = 0; i < b_loop_ub; i++) {
    s[i2 + i] = tmp_data[i];
  }
  /*  Extract Nodes */
  emxInit_real_T(sp, &nodes_pos, 2, &ib_emlrtRTEI);
  i = nodes_pos->size[0] * nodes_pos->size[1];
  nodes_pos->size[0] = 3;
  emxEnsureCapacity_real_T(sp, nodes_pos, i, &i_emlrtRTEI);
  if (!(args->N_mt_nodes >= 0.0)) {
    emlrtNonNegativeCheckR2012b(args->N_mt_nodes, &f_emlrtDCI,
                                (emlrtConstCTX)sp);
  }
  i = (int32_T)muDoubleScalarFloor(args->N_mt_nodes);
  if (args->N_mt_nodes != i) {
    emlrtIntegerCheckR2012b(args->N_mt_nodes, &e_emlrtDCI, (emlrtConstCTX)sp);
  }
  i1 = nodes_pos->size[0] * nodes_pos->size[1];
  nodes_pos->size[1] = (int32_T)args->N_mt_nodes;
  emxEnsureCapacity_real_T(sp, nodes_pos, i1, &i_emlrtRTEI);
  nodes_pos_data = nodes_pos->data;
  emxInit_real_T(sp, &nodes_vel, 2, &jb_emlrtRTEI);
  i1 = nodes_vel->size[0] * nodes_vel->size[1];
  nodes_vel->size[0] = 3;
  emxEnsureCapacity_real_T(sp, nodes_vel, i1, &j_emlrtRTEI);
  if (args->N_mt_nodes != i) {
    emlrtIntegerCheckR2012b(args->N_mt_nodes, &g_emlrtDCI, (emlrtConstCTX)sp);
  }
  Force_nodes_tmp = (int32_T)args->N_mt_nodes;
  i1 = nodes_vel->size[0] * nodes_vel->size[1];
  nodes_vel->size[1] = Force_nodes_tmp;
  emxEnsureCapacity_real_T(sp, nodes_vel, i1, &j_emlrtRTEI);
  nodes_vel_data = nodes_vel->data;
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, args->N_mt_nodes, mxDOUBLE_CLASS,
                                (int32_T)args->N_mt_nodes, &b_emlrtRTEI,
                                (emlrtConstCTX)sp);
  for (b_i = 0; b_i < Force_nodes_tmp; b_i++) {
    idx = (((real_T)b_i + 1.0) - 1.0) * 6.0 + 27.0;
    if ((int32_T)((uint32_T)b_i + 1U) > nodes_pos->size[1]) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                    nodes_pos->size[1], &w_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (((int32_T)idx < 1) || ((int32_T)idx > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)idx, 1, 95, &db_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    nodes_pos_data[3 * b_i] = s[(int32_T)idx - 1];
    if (((int32_T)(idx + 1.0) < 1) || ((int32_T)(idx + 1.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(idx + 1.0), 1, 95, &db_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i1 = 3 * b_i + 1;
    nodes_pos_data[i1] = s[(int32_T)(idx + 1.0) - 1];
    if (((int32_T)(idx + 2.0) < 1) || ((int32_T)(idx + 2.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(idx + 2.0), 1, 95, &db_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    i2 = 3 * b_i + 2;
    nodes_pos_data[i2] = s[(int32_T)(idx + 2.0) - 1];
    if ((int32_T)((uint32_T)b_i + 1U) > nodes_vel->size[1]) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                    nodes_vel->size[1], &t_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (((int32_T)(idx + 3.0) < 1) || ((int32_T)(idx + 3.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)(idx + 3.0), 1, 95, &cb_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    nodes_vel_data[3 * b_i] = s[(int32_T)(idx + 3.0) - 1];
    if (((int32_T)((idx + 3.0) + 1.0) < 1) ||
        ((int32_T)((idx + 3.0) + 1.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((idx + 3.0) + 1.0), 1, 95,
                                    &cb_emlrtBCI, (emlrtConstCTX)sp);
    }
    nodes_vel_data[i1] = s[(int32_T)((idx + 3.0) + 1.0) - 1];
    if (((int32_T)((idx + 3.0) + 2.0) < 1) ||
        ((int32_T)((idx + 3.0) + 2.0) > 95)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((idx + 3.0) + 2.0), 1, 95,
                                    &cb_emlrtBCI, (emlrtConstCTX)sp);
    }
    nodes_vel_data[i2] = s[(int32_T)((idx + 3.0) + 2.0) - 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  /*  MRAC States indices */
  /*  Total states = 26 + 6*N + 7 */
  if (((d + 27.0) + 9.0) - 1.0 !=
      (int32_T)muDoubleScalarFloor(((d + 27.0) + 9.0) - 1.0)) {
    emlrtIntegerCheckR2012b(((d + 27.0) + 9.0) - 1.0, &b_emlrtDCI,
                            (emlrtConstCTX)sp);
  }
  emxInit_real_T(sp, &b_ds, 2, &nb_emlrtRTEI);
  i1 = b_ds->size[0] * b_ds->size[1];
  b_ds->size[0] = 1;
  b_loop_ub = (int32_T)(((d + 27.0) + 9.0) - 1.0);
  b_ds->size[1] = (int32_T)(((d + 27.0) + 9.0) - 1.0);
  emxEnsureCapacity_real_T(sp, b_ds, i1, &k_emlrtRTEI);
  ds_data = b_ds->data;
  for (i1 = 0; i1 < b_loop_ub; i1++) {
    ds_data[i1] = 0.0;
  }
  /* From Field 2022 Thesis */
  b_tmp[0] = -s[7];
  b_tmp[4] = -s[8];
  b_tmp[8] = -s[9];
  b_tmp[1] = s[6];
  b_tmp[5] = -s[9];
  b_tmp[9] = s[8];
  b_tmp[2] = s[9];
  b_tmp[6] = s[6];
  b_tmp[10] = -s[7];
  b_tmp[3] = -s[8];
  b_tmp[7] = s[7];
  b_tmp[11] = s[6];
  idx = 0.5 * -s[20];
  Qmat_desCh[0] = idx;
  Qmat_desCh_tmp = 0.5 * -s[21];
  Qmat_desCh[4] = Qmat_desCh_tmp;
  coeffT = 0.5 * -s[22];
  Qmat_desCh[8] = coeffT;
  Qmat_desCh[1] = 0.5 * s[19];
  Qmat_desCh[5] = coeffT;
  Qmat_desCh[9] = 0.5 * s[21];
  Qmat_desCh[2] = 0.5 * s[22];
  Qmat_desCh[6] = 0.5 * s[19];
  Qmat_desCh[10] = idx;
  Qmat_desCh[3] = Qmat_desCh_tmp;
  Qmat_desCh[7] = 0.5 * s[20];
  Qmat_desCh[11] = 0.5 * s[19];
  d1 = s[23];
  rotMat_C_A_I_tmp = s[24];
  b_rotMat_C_A_I_tmp = s[25];
  c_rotMat_C_A_I_tmp = s[10];
  d_rotMat_C_A_I_tmp = s[11];
  e_rotMat_C_A_I_tmp = s[12];
  for (i1 = 0; i1 <= 2; i1 += 2) {
    r = _mm_loadu_pd(&Qmat_desCh[i1]);
    r = _mm_mul_pd(r, _mm_set1_pd(d1));
    r2 = _mm_loadu_pd(&Qmat_desCh[i1 + 4]);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    r2 = _mm_loadu_pd(&Qmat_desCh[i1 + 8]);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(b_rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    _mm_storeu_pd(&quar_TDot[i1], r);
    r = _mm_loadu_pd(&b_tmp[i1]);
    r = _mm_mul_pd(_mm_set1_pd(0.5), r);
    r = _mm_mul_pd(r, _mm_set1_pd(c_rotMat_C_A_I_tmp));
    r2 = _mm_loadu_pd(&b_tmp[i1 + 4]);
    r2 = _mm_mul_pd(_mm_set1_pd(0.5), r2);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(d_rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    r2 = _mm_loadu_pd(&b_tmp[i1 + 8]);
    r2 = _mm_mul_pd(_mm_set1_pd(0.5), r2);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(e_rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    _mm_storeu_pd(&quar_CDot[i1], r);
  }
  /*     %% Rotation Matris */
  /* From Field 2022 Thesis */
  st.site = &x_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &x_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &x_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &x_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &w_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &w_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &w_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &w_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &v_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &v_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &v_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &v_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  idx = s[6] * s[6];
  Qmat_desCh_tmp = s[7] * s[7];
  coeffT = s[8] * s[8];
  b_rotMat_C_A_I_tmp = s[9] * s[9];
  rotMat_C_A_I[0] = ((idx + Qmat_desCh_tmp) - coeffT) - b_rotMat_C_A_I_tmp;
  c_rotMat_C_A_I_tmp = s[7] * s[8];
  d_rotMat_C_A_I_tmp = s[6] * s[9];
  rotMat_C_A_I[1] = 2.0 * (c_rotMat_C_A_I_tmp - d_rotMat_C_A_I_tmp);
  e_rotMat_C_A_I_tmp = s[7] * s[9];
  rotMat_C_A_I_tmp = s[6] * s[8];
  rotMat_C_A_I[2] = 2.0 * (e_rotMat_C_A_I_tmp + rotMat_C_A_I_tmp);
  rotMat_C_A_I[3] = 2.0 * (c_rotMat_C_A_I_tmp + d_rotMat_C_A_I_tmp);
  idx -= Qmat_desCh_tmp;
  rotMat_C_A_I[4] = (idx + coeffT) - b_rotMat_C_A_I_tmp;
  Qmat_desCh_tmp = s[8] * s[9];
  c_rotMat_C_A_I_tmp = s[6] * s[7];
  rotMat_C_A_I[5] = 2.0 * (Qmat_desCh_tmp - c_rotMat_C_A_I_tmp);
  rotMat_C_A_I[6] = 2.0 * (e_rotMat_C_A_I_tmp - rotMat_C_A_I_tmp);
  rotMat_C_A_I[7] = 2.0 * (Qmat_desCh_tmp + c_rotMat_C_A_I_tmp);
  rotMat_C_A_I[8] = (idx - coeffT) + b_rotMat_C_A_I_tmp;
  st.site = &u_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &u_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &u_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &u_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &t_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &t_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &t_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &t_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &s_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &s_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &s_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &s_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  idx = s[19] * s[19];
  Qmat_desCh_tmp = s[20] * s[20];
  coeffT = s[21] * s[21];
  b_rotMat_C_A_I_tmp = s[22] * s[22];
  rotMat_D_A_I[0] = ((idx + Qmat_desCh_tmp) - coeffT) - b_rotMat_C_A_I_tmp;
  c_rotMat_C_A_I_tmp = s[20] * s[21];
  d_rotMat_C_A_I_tmp = s[19] * s[22];
  rotMat_D_A_I[1] = 2.0 * (c_rotMat_C_A_I_tmp - d_rotMat_C_A_I_tmp);
  e_rotMat_C_A_I_tmp = s[20] * s[22];
  rotMat_C_A_I_tmp = s[19] * s[21];
  rotMat_D_A_I[2] = 2.0 * (e_rotMat_C_A_I_tmp + rotMat_C_A_I_tmp);
  rotMat_D_A_I[3] = 2.0 * (c_rotMat_C_A_I_tmp + d_rotMat_C_A_I_tmp);
  idx -= Qmat_desCh_tmp;
  rotMat_D_A_I[4] = (idx + coeffT) - b_rotMat_C_A_I_tmp;
  Qmat_desCh_tmp = s[21] * s[22];
  c_rotMat_C_A_I_tmp = s[19] * s[20];
  rotMat_D_A_I[5] = 2.0 * (Qmat_desCh_tmp - c_rotMat_C_A_I_tmp);
  rotMat_D_A_I[6] = 2.0 * (e_rotMat_C_A_I_tmp - rotMat_C_A_I_tmp);
  rotMat_D_A_I[7] = 2.0 * (Qmat_desCh_tmp + c_rotMat_C_A_I_tmp);
  rotMat_D_A_I[8] = (idx - coeffT) + b_rotMat_C_A_I_tmp;
  /*     %% N2L Chaser & MT Nodes */
  idx = 0.5 * args->chaserSideLength;
  for (b_i = 0; b_i < 3; b_i++) {
    distAttPt_to_C[b_i] = idx * (real_T)iv[b_i];
    y_tmp[3 * b_i] = rotMat_C_A_I[b_i];
    y_tmp[3 * b_i + 1] = rotMat_C_A_I[b_i + 3];
    y_tmp[3 * b_i + 2] = rotMat_C_A_I[b_i + 6];
  }
  /*  Initialize Forces */
  if (Force_nodes_tmp != i) {
    emlrtIntegerCheckR2012b(args->N_mt_nodes, &emlrtDCI, (emlrtConstCTX)sp);
  }
  appTorqueTarget_idx_0 = 0.0;
  appTorqueTarget_idx_1 = 0.0;
  appTorqueTarget_idx_2 = 0.0;
  /*  For logic use */
  /*  Vectorized Loop over N segments */
  /*  Seg 1: Chaser -> Node 1 */
  /*  Seg k: Node k-1 -> Node k */
  /*  Prep Pos/Vel Matrices for Predecessor (Prev) and Current (Curr) points */
  /*  Prev: [pos_Att, Node 1, Node 2 ... Node N-1] */
  if (nodes_pos->size[1] - 1 < 1) {
    loop_ub = 0;
  } else {
    if (nodes_pos->size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, nodes_pos->size[1], &v_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if ((nodes_pos->size[1] - 1 < 1) ||
        (nodes_pos->size[1] - 1 > nodes_pos->size[1])) {
      emlrtDynamicBoundsCheckR2012b(nodes_pos->size[1] - 1, 1,
                                    nodes_pos->size[1], &u_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = nodes_pos->size[1] - 1;
  }
  emxInit_real_T(sp, &P_prev, 2, &l_emlrtRTEI);
  i = P_prev->size[0] * P_prev->size[1];
  P_prev->size[0] = 3;
  P_prev->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(sp, P_prev, i, &l_emlrtRTEI);
  P_prev_data = P_prev->data;
  d1 = distAttPt_to_C[0];
  rotMat_C_A_I_tmp = distAttPt_to_C[1];
  b_rotMat_C_A_I_tmp = distAttPt_to_C[2];
  for (i = 0; i < 3; i++) {
    P_prev_data[i] = s[i] + ((y_tmp[i] * d1 + y_tmp[i + 3] * rotMat_C_A_I_tmp) +
                             y_tmp[i + 6] * b_rotMat_C_A_I_tmp);
  }
  for (i = 0; i < loop_ub; i++) {
    i1 = 3 * (i + 1);
    P_prev_data[i1] = nodes_pos_data[3 * i];
    P_prev_data[i1 + 1] = nodes_pos_data[3 * i + 1];
    P_prev_data[i1 + 2] = nodes_pos_data[3 * i + 2];
  }
  if (nodes_vel->size[1] - 1 < 1) {
    loop_ub = 0;
  } else {
    if (nodes_vel->size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, nodes_vel->size[1], &s_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if ((nodes_vel->size[1] - 1 < 1) ||
        (nodes_vel->size[1] - 1 > nodes_vel->size[1])) {
      emlrtDynamicBoundsCheckR2012b(nodes_vel->size[1] - 1, 1,
                                    nodes_vel->size[1], &r_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    loop_ub = nodes_vel->size[1] - 1;
  }
  emxInit_real_T(sp, &V_prev, 2, &m_emlrtRTEI);
  i = V_prev->size[0] * V_prev->size[1];
  V_prev->size[0] = 3;
  V_prev->size[1] = loop_ub + 1;
  emxEnsureCapacity_real_T(sp, V_prev, i, &m_emlrtRTEI);
  V_prev_data = V_prev->data;
  d1 = distAttPt_to_C[2] * s[11] - distAttPt_to_C[1] * s[12];
  rotMat_C_A_I_tmp = distAttPt_to_C[0] * s[12] - distAttPt_to_C[2] * s[10];
  b_rotMat_C_A_I_tmp = distAttPt_to_C[1] * s[10] - distAttPt_to_C[0] * s[11];
  for (i = 0; i < 3; i++) {
    V_prev_data[i] =
        s[i + 3] + ((y_tmp[i] * d1 + y_tmp[i + 3] * rotMat_C_A_I_tmp) +
                    y_tmp[i + 6] * b_rotMat_C_A_I_tmp);
  }
  for (i = 0; i < loop_ub; i++) {
    i1 = 3 * (i + 1);
    V_prev_data[i1] = nodes_vel_data[3 * i];
    V_prev_data[i1 + 1] = nodes_vel_data[3 * i + 1];
    V_prev_data[i1 + 2] = nodes_vel_data[3 * i + 2];
  }
  /*  Curr: [Node 1, Node 2 ... Node N] */
  /*  Vectorized Force Calc */
  if ((nodes_pos->size[1] != P_prev->size[1]) &&
      ((nodes_pos->size[1] != 1) && (P_prev->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(nodes_pos->size[1], P_prev->size[1],
                                &k_emlrtECI, (emlrtConstCTX)sp);
  }
  if (nodes_pos->size[1] == P_prev->size[1]) {
    loop_ub = 3 * nodes_pos->size[1];
    i = P_prev->size[0] * P_prev->size[1];
    P_prev->size[0] = 3;
    P_prev->size[1] = nodes_pos->size[1];
    emxEnsureCapacity_real_T(sp, P_prev, i, &n_emlrtRTEI);
    P_prev_data = P_prev->data;
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&nodes_pos_data[i]);
      r2 = _mm_loadu_pd(&P_prev_data[i]);
      _mm_storeu_pd(&P_prev_data[i], _mm_sub_pd(r, r2));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      P_prev_data[i] = nodes_pos_data[i] - P_prev_data[i];
    }
  } else {
    st.site = &ef_emlrtRSI;
    minus(&st, P_prev, nodes_pos);
    P_prev_data = P_prev->data;
  }
  emxInit_real_T(sp, &b_P_prev, 2, &p_emlrtRTEI);
  i = b_P_prev->size[0] * b_P_prev->size[1];
  b_P_prev->size[0] = 3;
  loop_ub = P_prev->size[1];
  b_P_prev->size[1] = P_prev->size[1];
  emxEnsureCapacity_real_T(sp, b_P_prev, i, &o_emlrtRTEI);
  vr_dots_data = b_P_prev->data;
  b_loop_ub = 3 * P_prev->size[1];
  for (i = 0; i < b_loop_ub; i++) {
    idx = P_prev_data[i];
    vr_dots_data[i] = idx * idx;
  }
  emxInit_real_T(sp, &L_mags, 2, &lb_emlrtRTEI);
  st.site = &r_emlrtRSI;
  sum(&st, b_P_prev, L_mags);
  st.site = &r_emlrtRSI;
  b_sqrt(&st, L_mags);
  L_mags_data = L_mags->data;
  st.site = &q_emlrtRSI;
  b_st.site = &kb_emlrtRSI;
  c_st.site = &lb_emlrtRSI;
  assertCompatibleDims(&c_st, P_prev, L_mags);
  if (P_prev->size[1] == L_mags->size[1]) {
    i = b_P_prev->size[0] * b_P_prev->size[1];
    b_P_prev->size[0] = 3;
    b_P_prev->size[1] = P_prev->size[1];
    emxEnsureCapacity_real_T(&b_st, b_P_prev, i, &p_emlrtRTEI);
    vr_dots_data = b_P_prev->data;
    for (i = 0; i < loop_ub; i++) {
      r = _mm_loadu_pd(&P_prev_data[3 * i]);
      _mm_storeu_pd(&vr_dots_data[3 * i],
                    _mm_div_pd(r, _mm_set1_pd(L_mags_data[i])));
      i1 = 3 * i + 2;
      vr_dots_data[i1] = P_prev_data[i1] / L_mags_data[i];
    }
    i = P_prev->size[0] * P_prev->size[1];
    P_prev->size[0] = 3;
    P_prev->size[1] = b_P_prev->size[1];
    emxEnsureCapacity_real_T(&b_st, P_prev, i, &q_emlrtRTEI);
    P_prev_data = P_prev->data;
    b_loop_ub = 3 * b_P_prev->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      P_prev_data[i] = vr_dots_data[i];
    }
  } else {
    c_st.site = &ff_emlrtRSI;
    rdivide(&c_st, P_prev, L_mags);
    P_prev_data = P_prev->data;
  }
  if ((nodes_vel->size[1] != V_prev->size[1]) &&
      ((nodes_vel->size[1] != 1) && (V_prev->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(nodes_vel->size[1], V_prev->size[1],
                                &j_emlrtECI, (emlrtConstCTX)sp);
  }
  if (nodes_vel->size[1] == V_prev->size[1]) {
    loop_ub = 3 * nodes_vel->size[1];
    i = V_prev->size[0] * V_prev->size[1];
    V_prev->size[0] = 3;
    V_prev->size[1] = nodes_vel->size[1];
    emxEnsureCapacity_real_T(sp, V_prev, i, &r_emlrtRTEI);
    V_prev_data = V_prev->data;
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&nodes_vel_data[i]);
      r2 = _mm_loadu_pd(&V_prev_data[i]);
      _mm_storeu_pd(&V_prev_data[i], _mm_sub_pd(r, r2));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      V_prev_data[i] = nodes_vel_data[i] - V_prev_data[i];
    }
  } else {
    st.site = &df_emlrtRSI;
    minus(&st, V_prev, nodes_vel);
    V_prev_data = V_prev->data;
  }
  emxInit_real_T(sp, &deltas, 2, &s_emlrtRTEI);
  i = deltas->size[0] * deltas->size[1];
  deltas->size[0] = 1;
  loop_ub = L_mags->size[1];
  deltas->size[1] = L_mags->size[1];
  emxEnsureCapacity_real_T(sp, deltas, i, &s_emlrtRTEI);
  deltas_data = deltas->data;
  idx = args->l0vec[0];
  scalarLB = (L_mags->size[1] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&L_mags_data[i]);
    _mm_storeu_pd(&deltas_data[i], _mm_sub_pd(r, _mm_set1_pd(idx)));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    deltas_data[i] = L_mags_data[i] - idx;
  }
  if ((V_prev->size[1] != P_prev->size[1]) &&
      ((V_prev->size[1] != 1) && (P_prev->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(V_prev->size[1], P_prev->size[1], &i_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  emxInit_real_T(sp, &vr_dots, 2, &mb_emlrtRTEI);
  if (V_prev->size[1] == P_prev->size[1]) {
    i = b_P_prev->size[0] * b_P_prev->size[1];
    b_P_prev->size[0] = 3;
    b_P_prev->size[1] = V_prev->size[1];
    emxEnsureCapacity_real_T(sp, b_P_prev, i, &t_emlrtRTEI);
    vr_dots_data = b_P_prev->data;
    b_loop_ub = 3 * V_prev->size[1];
    scalarLB = (b_loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&V_prev_data[i]);
      r2 = _mm_loadu_pd(&P_prev_data[i]);
      _mm_storeu_pd(&vr_dots_data[i], _mm_mul_pd(r, r2));
    }
    for (i = scalarLB; i < b_loop_ub; i++) {
      vr_dots_data[i] = V_prev_data[i] * P_prev_data[i];
    }
    st.site = &p_emlrtRSI;
    sum(&st, b_P_prev, vr_dots);
  } else {
    st.site = &p_emlrtRSI;
    binary_expand_op_2(&st, vr_dots, p_emlrtRSI, V_prev, P_prev);
  }
  emxFree_real_T(sp, &b_P_prev);
  emxInit_real_T(sp, &Tmags, 2, &w_emlrtRTEI);
  i = Tmags->size[0] * Tmags->size[1];
  Tmags->size[0] = 1;
  loop_ub = deltas->size[1];
  Tmags->size[1] = deltas->size[1];
  emxEnsureCapacity_real_T(sp, Tmags, i, &u_emlrtRTEI);
  Tmags_data = Tmags->data;
  idx = args->Kvec[0];
  scalarLB = (deltas->size[1] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&deltas_data[i]);
    _mm_storeu_pd(&Tmags_data[i], _mm_mul_pd(_mm_set1_pd(idx), r));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    Tmags_data[i] = idx * deltas_data[i];
  }
  i = vr_dots->size[0] * vr_dots->size[1];
  vr_dots->size[0] = 1;
  emxEnsureCapacity_real_T(sp, vr_dots, i, &v_emlrtRTEI);
  vr_dots_data = vr_dots->data;
  idx = args->cVec[0];
  b_loop_ub = vr_dots->size[1] - 1;
  scalarLB = (vr_dots->size[1] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&vr_dots_data[i]);
    _mm_storeu_pd(&vr_dots_data[i], _mm_mul_pd(_mm_set1_pd(idx), r));
  }
  for (i = scalarLB; i <= b_loop_ub; i++) {
    vr_dots_data[i] *= idx;
  }
  if ((Tmags->size[1] != vr_dots->size[1]) &&
      ((Tmags->size[1] != 1) && (vr_dots->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(Tmags->size[1], vr_dots->size[1], &h_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (Tmags->size[1] == vr_dots->size[1]) {
    b_loop_ub = Tmags->size[1] - 1;
    i = Tmags->size[0] * Tmags->size[1];
    Tmags->size[0] = 1;
    emxEnsureCapacity_real_T(sp, Tmags, i, &w_emlrtRTEI);
    Tmags_data = Tmags->data;
    scalarLB = (Tmags->size[1] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&Tmags_data[i]);
      r2 = _mm_loadu_pd(&vr_dots_data[i]);
      _mm_storeu_pd(&Tmags_data[i], _mm_add_pd(r, r2));
    }
    for (i = scalarLB; i <= b_loop_ub; i++) {
      Tmags_data[i] += vr_dots_data[i];
    }
  } else {
    st.site = &cf_emlrtRSI;
    plus(&st, Tmags, vr_dots);
    Tmags_data = Tmags->data;
  }
  emxFree_real_T(sp, &vr_dots);
  /*  Masks */
  emxInit_boolean_T(sp, &maskLength, &x_emlrtRTEI);
  i = maskLength->size[0] * maskLength->size[1];
  maskLength->size[0] = 1;
  maskLength->size[1] = deltas->size[1];
  emxEnsureCapacity_boolean_T(sp, maskLength, i, &x_emlrtRTEI);
  maskLength_data = maskLength->data;
  for (i = 0; i < loop_ub; i++) {
    maskLength_data[i] = (deltas_data[i] > 0.0);
  }
  emxInit_boolean_T(sp, &maskTension, &y_emlrtRTEI);
  i = maskTension->size[0] * maskTension->size[1];
  maskTension->size[0] = 1;
  loop_ub = Tmags->size[1];
  maskTension->size[1] = Tmags->size[1];
  emxEnsureCapacity_boolean_T(sp, maskTension, i, &y_emlrtRTEI);
  maskTension_data = maskTension->data;
  for (i = 0; i < loop_ub; i++) {
    maskTension_data[i] = (Tmags_data[i] > 0.0);
  }
  if ((deltas->size[1] != Tmags->size[1]) &&
      ((deltas->size[1] != 1) && (Tmags->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(deltas->size[1], Tmags->size[1], &g_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  emxFree_real_T(sp, &deltas);
  if (maskLength->size[1] == maskTension->size[1]) {
    loop_ub = maskLength->size[1] - 1;
    i = maskLength->size[0] * maskLength->size[1];
    maskLength->size[0] = 1;
    emxEnsureCapacity_boolean_T(sp, maskLength, i, &ab_emlrtRTEI);
    maskLength_data = maskLength->data;
    for (i = 0; i <= loop_ub; i++) {
      maskLength_data[i] = (maskLength_data[i] && maskTension_data[i]);
    }
  } else {
    st.site = &bf_emlrtRSI;
    b_and(&st, maskLength, maskTension);
    maskLength_data = maskLength->data;
  }
  emxFree_boolean_T(sp, &maskTension);
  b_loop_ub = maskLength->size[1];
  for (b_i = 0; b_i < b_loop_ub; b_i++) {
    if (!maskLength_data[b_i]) {
      if (b_i > Tmags->size[1] - 1) {
        emlrtDynamicBoundsCheckR2012b(b_i, 0, Tmags->size[1] - 1, &ab_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      Tmags_data[b_i] = 0.0;
    }
  }
  loop_ub = P_prev->size[1];
  if ((P_prev->size[1] != Tmags->size[1]) &&
      ((P_prev->size[1] != 1) && (Tmags->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(P_prev->size[1], Tmags->size[1], &f_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  emxInit_real_T(sp, &Tvecs, 2, &bb_emlrtRTEI);
  if (P_prev->size[1] == Tmags->size[1]) {
    i = Tvecs->size[0] * Tvecs->size[1];
    Tvecs->size[0] = 3;
    Tvecs->size[1] = P_prev->size[1];
    emxEnsureCapacity_real_T(sp, Tvecs, i, &bb_emlrtRTEI);
    deltas_data = Tvecs->data;
    for (i = 0; i < loop_ub; i++) {
      r = _mm_loadu_pd(&P_prev_data[3 * i]);
      _mm_storeu_pd(&deltas_data[3 * i],
                    _mm_mul_pd(r, _mm_set1_pd(Tmags_data[i])));
      i1 = 3 * i + 2;
      deltas_data[i1] = Tmags_data[i] * P_prev_data[i1];
    }
  } else {
    st.site = &gf_emlrtRSI;
    times(&st, Tvecs, P_prev, Tmags);
    deltas_data = Tvecs->data;
  }
  emxFree_real_T(sp, &Tmags);
  /*  3xN */
  /*  Apply Forces */
  /*  Force on 'curr' (Node k) gets -Tvec (Pulls back to Prev) */
  if ((Force_nodes_tmp != Tvecs->size[1]) &&
      ((args->N_mt_nodes != 1.0) && (Tvecs->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b((int32_T)args->N_mt_nodes, Tvecs->size[1],
                                &e_emlrtECI, (emlrtConstCTX)sp);
  }
  emxInit_real_T(sp, &Force_nodes, 2, &kb_emlrtRTEI);
  if (Force_nodes_tmp == Tvecs->size[1]) {
    i = Force_nodes->size[0] * Force_nodes->size[1];
    Force_nodes->size[0] = 3;
    Force_nodes->size[1] = Force_nodes_tmp;
    emxEnsureCapacity_real_T(sp, Force_nodes, i, &cb_emlrtRTEI);
    Tmags_data = Force_nodes->data;
    loop_ub = 3 * Force_nodes_tmp;
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&deltas_data[i]);
      _mm_storeu_pd(&Tmags_data[i], _mm_sub_pd(_mm_set1_pd(0.0), r));
    }
    for (i = scalarLB; i < loop_ub; i++) {
      Tmags_data[i] = 0.0 - deltas_data[i];
    }
  } else {
    st.site = &hf_emlrtRSI;
    binary_expand_op_1(&st, Force_nodes, args, Tvecs);
    Tmags_data = Force_nodes->data;
  }
  /*  Force on 'prev' (Node k-1 or Chaser) gets +Tvec (Pulls fwd to Curr) */
  if (Tvecs->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, Tvecs->size[1], &q_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (args->N_mt_nodes > 1.0) {
    if (Force_nodes->size[1] - 1 < 1) {
      loop_ub = 0;
    } else {
      if (Force_nodes->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, Force_nodes->size[1], &p_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if ((Force_nodes->size[1] - 1 < 1) ||
          (Force_nodes->size[1] - 1 > Force_nodes->size[1])) {
        emlrtDynamicBoundsCheckR2012b(Force_nodes->size[1] - 1, 1,
                                      Force_nodes->size[1], &o_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      loop_ub = Force_nodes->size[1] - 1;
    }
    if (Tvecs->size[1] < 2) {
      i = 0;
      i1 = 0;
    } else {
      i = 1;
      i1 = Tvecs->size[1];
    }
    i2 = i1 - i;
    if ((loop_ub != i2) && ((loop_ub != 1) && (i2 != 1))) {
      emlrtDimSizeImpxCheckR2021b(loop_ub, i2, &d_emlrtECI, (emlrtConstCTX)sp);
    }
    if (Force_nodes->size[1] - 1 < 1) {
      b_loop_ub = 0;
    } else {
      if (Force_nodes->size[1] < 1) {
        emlrtDynamicBoundsCheckR2012b(1, 1, Force_nodes->size[1], &n_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      if ((Force_nodes->size[1] - 1 < 1) ||
          (Force_nodes->size[1] - 1 > Force_nodes->size[1])) {
        emlrtDynamicBoundsCheckR2012b(Force_nodes->size[1] - 1, 1,
                                      Force_nodes->size[1], &m_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      b_loop_ub = Force_nodes->size[1] - 1;
    }
    emxInit_real_T(sp, &r3, 2, &ob_emlrtRTEI);
    if (loop_ub == i2) {
      i1 = r3->size[0] * r3->size[1];
      r3->size[0] = 3;
      r3->size[1] = loop_ub;
      emxEnsureCapacity_real_T(sp, r3, i1, &db_emlrtRTEI);
      vr_dots_data = r3->data;
      for (i1 = 0; i1 < loop_ub; i1++) {
        r = _mm_loadu_pd(&Tmags_data[3 * i1]);
        i2 = i + i1;
        r2 = _mm_loadu_pd(&deltas_data[3 * i2]);
        _mm_storeu_pd(&vr_dots_data[3 * i1], _mm_add_pd(r, r2));
        b_i = 3 * i1 + 2;
        vr_dots_data[b_i] = Tmags_data[b_i] + deltas_data[3 * i2 + 2];
      }
    } else {
      st.site = &if_emlrtRSI;
      binary_expand_op(&st, r3, Force_nodes, loop_ub, Tvecs, i, i1);
      vr_dots_data = r3->data;
    }
    input_sizes[0] = 3;
    input_sizes[1] = b_loop_ub;
    emlrtSubAssignSizeCheckR2012b(&input_sizes[0], 2, &r3->size[0], 2,
                                  &c_emlrtECI, (emlrtCTX)sp);
    for (i = 0; i < b_loop_ub; i++) {
      Tmags_data[3 * i] = vr_dots_data[3 * i];
      i1 = 3 * i + 1;
      Tmags_data[i1] = vr_dots_data[i1];
      i1 = 3 * i + 2;
      Tmags_data[i1] = vr_dots_data[i1];
    }
    emxFree_real_T(sp, &r3);
  }
  /*  Capture Segment 1 info for Control logic */
  if (L_mags->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, L_mags->size[1], &l_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (V_prev->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, V_prev->size[1], &k_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (P_prev->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, P_prev->size[1], &j_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (Tvecs->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, Tvecs->size[1], &i_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if (maskLength->size[1] < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, maskLength->size[1], &h_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  /*  Gravity on Nodes */
  m_node = args->massPoint1 / args->N_mt_nodes;
  /*  Distribute mass */
  emlrtForLoopVectorCheckR2021a(1.0, 1.0, args->N_mt_nodes, mxDOUBLE_CLASS,
                                (int32_T)args->N_mt_nodes, &emlrtRTEI,
                                (emlrtConstCTX)sp);
  for (b_i = 0; b_i < Force_nodes_tmp; b_i++) {
    if (b_i + 1 > nodes_pos->size[1]) {
      emlrtDynamicBoundsCheckR2012b(b_i + 1, 1, nodes_pos->size[1], &g_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    a = -args->mu * m_node;
    st.site = &o_emlrtRSI;
    b_st.site = &y_emlrtRSI;
    c_rotMat_C_A_I_tmp =
        muDoubleScalarPower(b_norm(&nodes_pos_data[3 * b_i]), 3.0);
    if ((int32_T)((uint32_T)b_i + 1U) > Force_nodes->size[1]) {
      emlrtDynamicBoundsCheckR2012b((int32_T)((uint32_T)b_i + 1U), 1,
                                    Force_nodes->size[1], &f_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    r = _mm_loadu_pd(&nodes_pos_data[3 * b_i]);
    r2 = _mm_loadu_pd(&Tmags_data[3 * b_i]);
    _mm_storeu_pd(&b_s[0],
                  _mm_add_pd(r2, _mm_div_pd(_mm_mul_pd(_mm_set1_pd(a), r),
                                            _mm_set1_pd(c_rotMat_C_A_I_tmp))));
    b_loop_ub = 3 * b_i + 2;
    b_s[2] = Tmags_data[b_loop_ub] +
             a * nodes_pos_data[b_loop_ub] / c_rotMat_C_A_I_tmp;
    Tmags_data[3 * b_i] = b_s[0];
    Tmags_data[3 * b_i + 1] = b_s[1];
    Tmags_data[b_loop_ub] = b_s[2];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  real_T MRAC_2_updateParams[11];
  real_T MRAC_1_updateParams[7];
  /*     %% Adaptive Control */
  st.site = &n_emlrtRSI;
  c_calculateAdaptiveControl_Unif(
      &st, t, s, args, d + 27.0, L_mags_data[0], &V_prev_data[0],
      &P_prev_data[0], maskLength_data[0], rotMat_C_A_I, y_tmp, k_Loc,
      MRAC_1_updateParams, MRAC_2_updateParams);
  emxFree_boolean_T(sp, &maskLength);
  emxFree_real_T(sp, &V_prev);
  emxFree_real_T(sp, &P_prev);
  /*  Update MRAC states */
  for (i = 0; i < 9; i++) {
    i1 = (int32_T)((d + 27.0) + (real_T)i);
    if ((i1 < 1) || (i1 > (int32_T)(((d + 27.0) + 9.0) - 1.0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, (int32_T)(((d + 27.0) + 9.0) - 1.0),
                                    &bb_emlrtBCI, (emlrtConstCTX)sp);
    }
    ds_data[i1 - 1] = y_tmp[i];
  }
  __m128d r5;
  /*     %% with gravity; */
  Qmat_desCh_tmp = b_norm(&s[0]);
  st.site = &m_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &m_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  coeffT = 0.001623945 * args->mu *
           (4.0678884E+13 / muDoubleScalarPower(Qmat_desCh_tmp, 4.0));
  a = -args->mu * args->chaserM;
  st.site = &l_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  c_rotMat_C_A_I_tmp = muDoubleScalarPower(Qmat_desCh_tmp, 3.0);
  st.site = &k_emlrtRSI;
  d_rotMat_C_A_I_tmp = s[2] / Qmat_desCh_tmp;
  b_st.site = &y_emlrtRSI;
  st.site = &j_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &i_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  b_a = args->J2on * args->chaserM * coeffT;
  idx = 5.0 * (d_rotMat_C_A_I_tmp * d_rotMat_C_A_I_tmp);
  accChaser[0] = b_a * ((idx - 1.0) * (s[0] / Qmat_desCh_tmp));
  accChaser[1] = b_a * ((idx - 1.0) * (s[1] / Qmat_desCh_tmp));
  accChaser[2] = b_a * ((idx - 3.0) * d_rotMat_C_A_I_tmp);
  /*     %% Sliding Att Control */
  /*  Vortex */
  /*      k_Loc = -evec_mt; */
  d1 = b_norm(&s[3]);
  r = _mm_loadu_pd(&deltas_data[0]);
  r2 = _mm_loadu_pd(&k_Loc[0]);
  r4 = _mm_loadu_pd(&s[0]);
  r5 = _mm_loadu_pd(&accChaser[0]);
  _mm_storeu_pd(
      &accChaser[0],
      _mm_div_pd(
          _mm_add_pd(_mm_add_pd(_mm_add_pd(r, r2),
                                _mm_div_pd(_mm_mul_pd(_mm_set1_pd(a), r4),
                                           _mm_set1_pd(c_rotMat_C_A_I_tmp))),
                     r5),
          _mm_set1_pd(args->chaserM)));
  r = _mm_loadu_pd(&s[3]);
  _mm_storeu_pd(&k_Loc[0],
                _mm_mul_pd(_mm_div_pd(r, _mm_set1_pd(d1)), _mm_set1_pd(-1.0)));
  _mm_storeu_pd(&b[0], _mm_div_pd(r4, _mm_set1_pd(Qmat_desCh_tmp)));
  accChaser[2] =
      (((deltas_data[2] + k_Loc[2]) + a * s[2] / c_rotMat_C_A_I_tmp) +
       accChaser[2]) /
      args->chaserM;
  k_Loc[2] = -(s[5] / d1);
  i_Loc[0] = k_Loc[1] * d_rotMat_C_A_I_tmp - b[1] * k_Loc[2];
  i_Loc[1] = b[0] * k_Loc[2] - k_Loc[0] * d_rotMat_C_A_I_tmp;
  i_Loc[2] = k_Loc[0] * b[1] - b[0] * k_Loc[1];
  y_tmp[3] = k_Loc[1] * i_Loc[2] - i_Loc[1] * k_Loc[2];
  y_tmp[4] = i_Loc[0] * k_Loc[2] - k_Loc[0] * i_Loc[2];
  y_tmp[5] = k_Loc[0] * i_Loc[1] - i_Loc[0] * k_Loc[1];
  y_tmp[0] = i_Loc[0];
  y_tmp[6] = k_Loc[0];
  y_tmp[1] = i_Loc[1];
  y_tmp[7] = k_Loc[1];
  y_tmp[2] = i_Loc[2];
  y_tmp[8] = k_Loc[2];
  st.site = &h_emlrtRSI;
  rotm2quat(&st, y_tmp, quat_desCh);
  Qmat_desCh[0] = -quat_desCh[1];
  Qmat_desCh[4] = -quat_desCh[2];
  Qmat_desCh[8] = -quat_desCh[3];
  Qmat_desCh[1] = quat_desCh[0];
  Qmat_desCh[5] = -quat_desCh[3];
  Qmat_desCh[9] = quat_desCh[2];
  Qmat_desCh[2] = quat_desCh[3];
  Qmat_desCh[6] = quat_desCh[0];
  Qmat_desCh[10] = -quat_desCh[1];
  Qmat_desCh[3] = -quat_desCh[2];
  Qmat_desCh[7] = quat_desCh[1];
  Qmat_desCh[11] = quat_desCh[0];
  idx = 0.0;
  for (i = 0; i < 4; i++) {
    q_eCh_tmp[3 * i] = Qmat_desCh[i];
    q_eCh_tmp[3 * i + 1] = Qmat_desCh[i + 4];
    q_eCh_tmp[3 * i + 2] = Qmat_desCh[i + 8];
    idx += s[i + 6] * quat_desCh[i];
  }
  idx = muDoubleScalarSign(idx);
  a = 16.0 * idx;
  r = _mm_loadu_pd(&s[10]);
  r = _mm_mul_pd(r, _mm_set1_pd(0.0));
  _mm_storeu_pd(&slidingVec_tmp[0], r);
  r = _mm_loadu_pd(&args->chaserI[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(s[10]));
  r2 = _mm_loadu_pd(&args->chaserI[3]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(s[11]));
  r = _mm_add_pd(r, r2);
  r2 = _mm_loadu_pd(&args->chaserI[6]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(s[12]));
  r = _mm_add_pd(r, r2);
  _mm_storeu_pd(&b[0], r);
  slidingVec_tmp[2] = s[12] * 0.0;
  b[2] = (args->chaserI[2] * s[10] + args->chaserI[5] * s[11]) +
         args->chaserI[8] * s[12];
  tau_Att_Ch_tmp[0] = b[2] * s[11] - b[1] * s[12];
  tau_Att_Ch_tmp[1] = b[0] * s[12] - b[2] * s[10];
  tau_Att_Ch_tmp[2] = b[1] * s[10] - b[0] * s[11];
  b_a = 8.0 * idx;
  /*  with SMC */
  /*  Tvec_mt_seg1 (from Chaser to Node 1) acts at distAttPt */
  /*     %% MODIFY ST ATTACHMENT PT HERE
   * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Connects to Node N */
  if ((args->N_mt_nodes < 1.0) || (Force_nodes_tmp > nodes_pos->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)args->N_mt_nodes, 1,
                                  nodes_pos->size[1], &e_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  if ((args->N_mt_nodes < 1.0) || (Force_nodes_tmp > nodes_vel->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)args->N_mt_nodes, 1,
                                  nodes_vel->size[1], &d_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  memset(&Tvec_st[0], 0, 12U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    y_tmp[3 * i] = rotMat_D_A_I[i];
    y_tmp[3 * i + 1] = rotMat_D_A_I[i + 3];
    y_tmp[3 * i + 2] = rotMat_D_A_I[i + 6];
  }
  for (b_i = 0; b_i < 4; b_i++) {
    if (b_i + 1 == 1) {
      d1 = -0.5 * targetSideLengthY;
      rotMat_C_A_I_tmp = -5.1;
    } else if (b_i + 1 == 2) {
      d1 = -0.5 * targetSideLengthY;
      rotMat_C_A_I_tmp = 5.1;
    } else if (b_i + 1 == 3) {
      d1 = 0.5 * targetSideLengthY;
      rotMat_C_A_I_tmp = -5.1;
    } else {
      d1 = 0.5 * targetSideLengthY;
      rotMat_C_A_I_tmp = 5.1;
    }
    /*  For Tension in links */
    for (i = 0; i < 3; i++) {
      k_Loc[i] = (nodes_pos_data[i + 3 * (Force_nodes_tmp - 1)] - s[i + 13]) -
                 ((y_tmp[i] * d1 + y_tmp[i + 3] * 0.0) +
                  y_tmp[i + 6] * rotMat_C_A_I_tmp);
    }
    /*  Node N to Debris AttPt */
    idx = b_norm(k_Loc);
    r = _mm_loadu_pd(&k_Loc[0]);
    _mm_storeu_pd(&k_Loc[0], _mm_div_pd(r, _mm_set1_pd(idx)));
    k_Loc[2] /= idx;
    /* Compute Tension */
    b_rotMat_C_A_I_tmp = args->l0vec[b_i + 1];
    if (idx > b_rotMat_C_A_I_tmp) {
      c_rotMat_C_A_I_tmp = rotMat_C_A_I_tmp * s[24] - 0.0 * s[25];
      d_rotMat_C_A_I_tmp = d1 * s[25] - rotMat_C_A_I_tmp * s[23];
      e_rotMat_C_A_I_tmp = s[23] * 0.0 - d1 * s[24];
      for (i = 0; i < 3; i++) {
        b_s[i] = (nodes_vel_data[i + 3 * ((int32_T)args->N_mt_nodes - 1)] -
                  s[i + 16]) -
                 ((y_tmp[i] * c_rotMat_C_A_I_tmp +
                   y_tmp[i + 3] * d_rotMat_C_A_I_tmp) +
                  y_tmp[i + 6] * e_rotMat_C_A_I_tmp);
      }
      idx = args->Kvec[b_i + 1] * (idx - b_rotMat_C_A_I_tmp) +
            args->cVec[b_i + 1] * dot(b_s, k_Loc);
      if (idx > 0.0) {
        r = _mm_loadu_pd(&k_Loc[0]);
        _mm_storeu_pd(&Tvec_st[3 * b_i], _mm_mul_pd(_mm_set1_pd(idx), r));
        Tvec_st[3 * b_i + 2] = idx * k_Loc[2];
      }
    }
    b_rotMat_C_A_I_tmp = Tvec_st[3 * b_i];
    c_rotMat_C_A_I_tmp = Tvec_st[3 * b_i + 1];
    d_rotMat_C_A_I_tmp = Tvec_st[3 * b_i + 2];
    r = _mm_loadu_pd(&rotMat_D_A_I[0]);
    r = _mm_mul_pd(r, _mm_set1_pd(b_rotMat_C_A_I_tmp));
    r2 = _mm_loadu_pd(&rotMat_D_A_I[3]);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(c_rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    r2 = _mm_loadu_pd(&rotMat_D_A_I[6]);
    r2 = _mm_mul_pd(r2, _mm_set1_pd(d_rotMat_C_A_I_tmp));
    r = _mm_add_pd(r, r2);
    _mm_storeu_pd(&b[0], r);
    b[2] = (rotMat_D_A_I[2] * b_rotMat_C_A_I_tmp +
            rotMat_D_A_I[5] * c_rotMat_C_A_I_tmp) +
           rotMat_D_A_I[8] * d_rotMat_C_A_I_tmp;
    appTorqueTarget_idx_0 += 0.0 * b[2] - b[1] * rotMat_C_A_I_tmp;
    appTorqueTarget_idx_1 += b[0] * rotMat_C_A_I_tmp - d1 * b[2];
    appTorqueTarget_idx_2 += d1 * b[1] - b[0] * 0.0;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  emxFree_real_T(sp, &nodes_pos);
  idx = b_norm(&s[13]);
  st.site = &g_emlrtRSI;
  d_rotMat_C_A_I_tmp = s[15] / idx;
  b_st.site = &y_emlrtRSI;
  st.site = &f_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  st.site = &e_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  Qmat_desCh_tmp = args->J2on * args->targetM * coeffT;
  coeffT = -args->mu * args->targetM;
  st.site = &d_emlrtRSI;
  b_st.site = &y_emlrtRSI;
  c_rotMat_C_A_I_tmp = muDoubleScalarPower(idx, 3.0);
  b_sum(Tvec_st, k_Loc);
  b_rotMat_C_A_I_tmp = 5.0 * (d_rotMat_C_A_I_tmp * d_rotMat_C_A_I_tmp);
  accTarget_idx_0 =
      ((k_Loc[0] + coeffT * s[13] / c_rotMat_C_A_I_tmp) +
       Qmat_desCh_tmp * ((b_rotMat_C_A_I_tmp - 1.0) * (s[13] / idx))) /
      args->targetM;
  targetSideLengthY =
      ((k_Loc[1] + coeffT * s[14] / c_rotMat_C_A_I_tmp) +
       Qmat_desCh_tmp * ((b_rotMat_C_A_I_tmp - 1.0) * (s[14] / idx))) /
      args->targetM;
  coeffT =
      ((k_Loc[2] + coeffT * s[15] / c_rotMat_C_A_I_tmp) +
       Qmat_desCh_tmp * ((b_rotMat_C_A_I_tmp - 3.0) * d_rotMat_C_A_I_tmp)) /
      args->targetM;
  /*     %% N2L Connection Point (Node N) */
  /*  Force on Node N needs ST forces. */
  if ((args->N_mt_nodes < 1.0) || (Force_nodes_tmp > Force_nodes->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)args->N_mt_nodes, 1,
                                  Force_nodes->size[1], &c_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  i = 3 * ((int32_T)args->N_mt_nodes - 1);
  r = _mm_loadu_pd(&Tmags_data[i]);
  r2 = _mm_loadu_pd(&k_Loc[0]);
  r = _mm_sub_pd(r, r2);
  _mm_storeu_pd(&k_Loc[0], r);
  _mm_storeu_pd(&Tmags_data[i], r);
  Tmags_data[i + 2] -= k_Loc[2];
  /*  Calculate Accelerations for all Nodes */
  loop_ub = 3 * Force_nodes->size[1];
  i = Force_nodes->size[0] * Force_nodes->size[1];
  Force_nodes->size[0] = 3;
  emxEnsureCapacity_real_T(sp, Force_nodes, i, &eb_emlrtRTEI);
  Tmags_data = Force_nodes->data;
  scalarLB = (loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&Tmags_data[i]);
    _mm_storeu_pd(&Tmags_data[i], _mm_div_pd(r, _mm_set1_pd(m_node)));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    Tmags_data[i] /= m_node;
  }
  /*     %% */
  for (i = 0; i < 3; i++) {
    b[i] = (rotMat_C_A_I[i] * deltas_data[0] +
            rotMat_C_A_I[i + 3] * deltas_data[1]) +
           rotMat_C_A_I[i + 6] * deltas_data[2];
    b_s[i] = (s[i + 10] - slidingVec_tmp[i]) +
             a * (((q_eCh_tmp[i] * s[6] + q_eCh_tmp[i + 3] * s[7]) +
                   q_eCh_tmp[i + 6] * s[8]) +
                  q_eCh_tmp[i + 9] * s[9]);
  }
  emxFree_real_T(sp, &Tvecs);
  d_stateDeriv_withGrav_LiamSet_U(b_s, k_Loc);
  for (i = 0; i < 3; i++) {
    d1 = 0.0;
    rotMat_C_A_I_tmp = 0.0;
    b_rotMat_C_A_I_tmp = 0.0;
    c_rotMat_C_A_I_tmp = q_eCh_tmp[i];
    d_rotMat_C_A_I_tmp = q_eCh_tmp[i + 3];
    e_rotMat_C_A_I_tmp = q_eCh_tmp[i + 6];
    idx = q_eCh_tmp[i + 9];
    i1 = i << 2;
    for (i2 = 0; i2 < 3; i2++) {
      b_i = i2 << 2;
      Qmat_desCh_tmp = s[i2 + 10];
      b_rotMat_C_A_I_tmp += (((b_tmp[i1] * Qmat_desCh[b_i] +
                               b_tmp[i1 + 1] * Qmat_desCh[b_i + 1]) +
                              b_tmp[i1 + 2] * Qmat_desCh[b_i + 2]) +
                             b_tmp[i1 + 3] * Qmat_desCh[b_i + 3]) *
                            Qmat_desCh_tmp;
      d1 += (((c_rotMat_C_A_I_tmp * b_tmp[b_i] +
               d_rotMat_C_A_I_tmp * b_tmp[b_i + 1]) +
              e_rotMat_C_A_I_tmp * b_tmp[b_i + 2]) +
             idx * b_tmp[b_i + 3]) *
            Qmat_desCh_tmp;
      rotMat_C_A_I_tmp += (real_T)c_a[i + 3 * i2] * k_Loc[i2];
    }
    i_Loc[i] = (b_a * (b_rotMat_C_A_I_tmp * 0.0 - d1) + slidingVec_tmp[i]) -
               rotMat_C_A_I_tmp;
  }
  b_s[0] = distAttPt_to_C[1] * b[2] - b[1] * distAttPt_to_C[2];
  b_s[1] = b[0] * distAttPt_to_C[2] - distAttPt_to_C[0] * b[2];
  b_s[2] = distAttPt_to_C[0] * b[1] - b[0] * distAttPt_to_C[1];
  d1 = i_Loc[0];
  rotMat_C_A_I_tmp = i_Loc[1];
  b_rotMat_C_A_I_tmp = i_Loc[2];
  r = _mm_loadu_pd(&args->chaserI[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(d1));
  r2 = _mm_loadu_pd(&args->chaserI[3]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(rotMat_C_A_I_tmp));
  r = _mm_add_pd(r, r2);
  r2 = _mm_loadu_pd(&args->chaserI[6]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(b_rotMat_C_A_I_tmp));
  r = _mm_add_pd(r, r2);
  r2 = _mm_loadu_pd(&tau_Att_Ch_tmp[0]);
  r = _mm_add_pd(r2, r);
  r4 = _mm_loadu_pd(&b_s[0]);
  r = _mm_add_pd(r4, r);
  r = _mm_sub_pd(r, r2);
  _mm_storeu_pd(&b_s[0], r);
  c_rotMat_C_A_I_tmp = tau_Att_Ch_tmp[2];
  b_s[2] = (b_s[2] +
            (c_rotMat_C_A_I_tmp +
             ((args->chaserI[2] * d1 + args->chaserI[5] * rotMat_C_A_I_tmp) +
              args->chaserI[8] * b_rotMat_C_A_I_tmp))) -
           c_rotMat_C_A_I_tmp;
  st.site = &c_emlrtRSI;
  mldivide(&st, args->chaserI, b_s, i_Loc);
  d1 = s[23];
  rotMat_C_A_I_tmp = s[24];
  b_rotMat_C_A_I_tmp = s[25];
  r = _mm_loadu_pd(&args->targetI[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(d1));
  r2 = _mm_loadu_pd(&args->targetI[3]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(rotMat_C_A_I_tmp));
  r = _mm_add_pd(r, r2);
  r2 = _mm_loadu_pd(&args->targetI[6]);
  r2 = _mm_mul_pd(r2, _mm_set1_pd(b_rotMat_C_A_I_tmp));
  r = _mm_add_pd(r, r2);
  _mm_storeu_pd(&b[0], r);
  b[2] = (args->targetI[2] * d1 + args->targetI[5] * rotMat_C_A_I_tmp) +
         args->targetI[8] * b_rotMat_C_A_I_tmp;
  b_s[0] = appTorqueTarget_idx_0 - (b[2] * s[24] - b[1] * s[25]);
  b_s[1] = appTorqueTarget_idx_1 - (b[0] * s[25] - b[2] * s[23]);
  b_s[2] = appTorqueTarget_idx_2 - (b[1] * s[23] - b[0] * s[24]);
  st.site = &b_emlrtRSI;
  mldivide(&st, args->targetI, b_s, k_Loc);
  /* chaser state derivatives */
  /* linear vel */
  ds_data[0] = s[3];
  ds_data[3] = accChaser[0];
  ds_data[1] = s[4];
  ds_data[4] = accChaser[1];
  ds_data[2] = s[5];
  ds_data[5] = accChaser[2];
  /* acc vel */
  ds_data[6] = quar_CDot[0];
  ds_data[7] = quar_CDot[1];
  ds_data[8] = quar_CDot[2];
  ds_data[9] = quar_CDot[3];
  /* ang vel */
  /* ang acc */
  /* target state derivatives */
  /* linear vel */
  ds_data[10] = i_Loc[0];
  ds_data[13] = s[16];
  ds_data[16] = accTarget_idx_0;
  ds_data[11] = i_Loc[1];
  ds_data[14] = s[17];
  ds_data[17] = targetSideLengthY;
  ds_data[12] = i_Loc[2];
  ds_data[15] = s[18];
  ds_data[18] = coeffT;
  /* acc vel */
  ds_data[19] = quar_TDot[0];
  ds_data[20] = quar_TDot[1];
  ds_data[21] = quar_TDot[2];
  ds_data[22] = quar_TDot[3];
  /* ang vel */
  ds_data[23] = k_Loc[0];
  ds_data[24] = k_Loc[1];
  ds_data[25] = k_Loc[2];
  /* ang acc */
  /* nodes state derivatives */
  st.site = &emlrtRSI;
  b_st.site = &xe_emlrtRSI;
  if (nodes_vel->size[1] != 0) {
    b_loop_ub = nodes_vel->size[1];
  } else if (Force_nodes->size[1] != 0) {
    b_loop_ub = Force_nodes->size[1];
  } else {
    b_loop_ub = 0;
  }
  c_st.site = &ye_emlrtRSI;
  if ((nodes_vel->size[1] != b_loop_ub) && (nodes_vel->size[1] != 0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if ((Force_nodes->size[1] != b_loop_ub) && (Force_nodes->size[1] != 0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  empty_non_axis_sizes = (b_loop_ub == 0);
  if (empty_non_axis_sizes || (nodes_vel->size[1] != 0)) {
    input_sizes_idx_0 = 3;
  } else {
    input_sizes_idx_0 = 0;
  }
  if (empty_non_axis_sizes || (Force_nodes->size[1] != 0)) {
    sizes_idx_0 = 3;
  } else {
    sizes_idx_0 = 0;
  }
  b_i = sizes_idx_0;
  emxInit_real_T(&b_st, &temp_node_states, 2, &fb_emlrtRTEI);
  i = temp_node_states->size[0] * temp_node_states->size[1];
  temp_node_states->size[0] = input_sizes_idx_0 + sizes_idx_0;
  temp_node_states->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(&b_st, temp_node_states, i, &fb_emlrtRTEI);
  vr_dots_data = temp_node_states->data;
  for (i = 0; i < b_loop_ub; i++) {
    loop_ub = input_sizes_idx_0;
    for (i1 = 0; i1 < loop_ub; i1++) {
      vr_dots_data[i1 + temp_node_states->size[0] * i] =
          nodes_vel_data[i1 + input_sizes_idx_0 * i];
    }
    for (i1 = 0; i1 < b_i; i1++) {
      vr_dots_data[(i1 + input_sizes_idx_0) + temp_node_states->size[0] * i] =
          Tmags_data[i1 + sizes_idx_0 * i];
    }
  }
  emxFree_real_T(&b_st, &Force_nodes);
  emxFree_real_T(&b_st, &nodes_vel);
  /*  6 x N */
  if (((int32_T)((d + 27.0) - 1.0) < 1) ||
      ((int32_T)((d + 27.0) - 1.0) > b_ds->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)((d + 27.0) - 1.0), 1, b_ds->size[1],
                                  &b_emlrtBCI, (emlrtConstCTX)sp);
  }
  i = temp_node_states->size[0] * temp_node_states->size[1];
  if ((int32_T)((d + 27.0) - 1.0) - 26 != i) {
    emlrtSubAssignSizeCheck1dR2017a((int32_T)((d + 27.0) - 1.0) - 26, i,
                                    &b_emlrtECI, (emlrtConstCTX)sp);
  }
  loop_ub = (int32_T)((d + 27.0) - 1.0) - 26;
  for (i = 0; i < loop_ub; i++) {
    ds_data[i + 26] = vr_dots_data[i];
  }
  emxFree_real_T(sp, &temp_node_states);
  /*  Scale back */
  r = _mm_loadu_pd(&ds_data[0]);
  _mm_storeu_pd(&ds_data[0], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds_data[13]);
  _mm_storeu_pd(&ds_data[13], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds_data[2]);
  _mm_storeu_pd(&ds_data[2], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds_data[15]);
  _mm_storeu_pd(&ds_data[15], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds_data[4]);
  _mm_storeu_pd(&ds_data[4], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds_data[17]);
  _mm_storeu_pd(&ds_data[17], _mm_div_pd(r, r1));
  /*  ds(27:32) = ds(27:32) / args.ODEscale; */
  /*  Scale N nodes */
  loop_ub = b_ds->size[1];
  if (((int32_T)((d + 27.0) - 1.0) < 1) ||
      ((int32_T)((d + 27.0) - 1.0) > b_ds->size[1])) {
    emlrtDynamicBoundsCheckR2012b((int32_T)((d + 27.0) - 1.0), 1, b_ds->size[1],
                                  &emlrtBCI, (emlrtConstCTX)sp);
  }
  b_loop_ub = (int32_T)((d + 27.0) - 1.0) - 27;
  i = L_mags->size[0] * L_mags->size[1];
  L_mags->size[0] = 1;
  L_mags->size[1] = (int32_T)((d + 27.0) - 1.0) - 26;
  emxEnsureCapacity_real_T(sp, L_mags, i, &gb_emlrtRTEI);
  L_mags_data = L_mags->data;
  scalarLB = (((int32_T)((d + 27.0) - 1.0) - 26) / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&ds_data[i + 26]);
    _mm_storeu_pd(&L_mags_data[i], _mm_div_pd(r, r1));
  }
  for (i = scalarLB; i <= b_loop_ub; i++) {
    L_mags_data[i] = ds_data[i + 26] / args->ODEscale;
  }
  b_loop_ub = L_mags->size[1];
  if ((int32_T)((d + 27.0) - 1.0) - 26 != L_mags->size[1]) {
    emlrtSubAssignSizeCheck1dR2017a((int32_T)((d + 27.0) - 1.0) - 26,
                                    L_mags->size[1], &emlrtECI,
                                    (emlrtConstCTX)sp);
  }
  for (i = 0; i < b_loop_ub; i++) {
    ds_data[i + 26] = L_mags_data[i];
  }
  emxFree_real_T(sp, &L_mags);
  /*  ds(mrac_idx:mrac_idx+6) = ds(mrac_idx:mrac_idx+6) / args.ODEscale; */
  i = ds->size[0];
  ds->size[0] = b_ds->size[1];
  emxEnsureCapacity_real_T(sp, ds, i, &hb_emlrtRTEI);
  vr_dots_data = ds->data;
  for (i = 0; i < loop_ub; i++) {
    vr_dots_data[i] = ds_data[i];
  }
  emxFree_real_T(sp, &b_ds);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (stateDeriv_withGrav_LiamSet_Unified_args.c) */
