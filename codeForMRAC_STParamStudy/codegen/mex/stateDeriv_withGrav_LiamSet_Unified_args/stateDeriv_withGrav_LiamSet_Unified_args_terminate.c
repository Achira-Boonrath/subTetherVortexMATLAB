/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_Unified_args_terminate.c
 *
 * Code generation for function
 * 'stateDeriv_withGrav_LiamSet_Unified_args_terminate'
 *
 */

/* Include files */
#include "stateDeriv_withGrav_LiamSet_Unified_args_terminate.h"
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_mex.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void stateDeriv_withGrav_LiamSet_Unified_args_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void stateDeriv_withGrav_LiamSet_Unified_args_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (stateDeriv_withGrav_LiamSet_Unified_args_terminate.c)
 */
