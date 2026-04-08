/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_terminate.c
 *
 * Code generation for function
 * 'stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_terminate'
 *
 */

/* Include files */
#include "stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_terminate.h"
#include "_coder_stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_mex.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_data.h"
#include "omp.h"

/* Function Declarations */
static void emlrtExitTimeCleanupDtorFcn(const void *r);

/* Function Definitions */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_atexit(void)
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(
      &stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_nestLockGlobal);
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    emlrtPushHeapReferenceStackR2021a(&st, false, NULL,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      NULL, NULL, NULL);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(
        &stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(
        &stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

void stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation
 * (stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_terminate.c) */
