/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_Unified_args_initialize.c
 *
 * Code generation for function
 * 'stateDeriv_withGrav_LiamSet_Unified_args_initialize'
 *
 */

/* Include files */
#include "stateDeriv_withGrav_LiamSet_Unified_args_initialize.h"
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_mex.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"

/* Function Declarations */
static void stateDeriv_withGrav_LiamSet_Unified_args_once(void);

/* Function Definitions */
static void stateDeriv_withGrav_LiamSet_Unified_args_once(void)
{
  mex_InitInfAndNan();
}

void stateDeriv_withGrav_LiamSet_Unified_args_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    stateDeriv_withGrav_LiamSet_Unified_args_once();
  }
}

/* End of code generation
 * (stateDeriv_withGrav_LiamSet_Unified_args_initialize.c) */
