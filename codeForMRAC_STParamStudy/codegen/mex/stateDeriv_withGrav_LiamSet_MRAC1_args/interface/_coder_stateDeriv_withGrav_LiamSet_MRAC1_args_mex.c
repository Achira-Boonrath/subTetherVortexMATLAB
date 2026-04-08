/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_stateDeriv_withGrav_LiamSet_MRAC1_args_mex.c
 *
 * Code generation for function
 * '_coder_stateDeriv_withGrav_LiamSet_MRAC1_args_mex'
 *
 */

/* Include files */
#include "_coder_stateDeriv_withGrav_LiamSet_MRAC1_args_mex.h"
#include "_coder_stateDeriv_withGrav_LiamSet_MRAC1_args_api.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_data.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_initialize.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_terminate.h"
#include "omp.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static jmp_buf emlrtJBEnviron;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&stateDeriv_withGrav_LiamSet_MRAC1_args_atexit);
  /* Initialize the memory manager. */
  omp_init_lock(&emlrtLockGlobal);
  omp_init_nest_lock(&stateDeriv_withGrav_LiamSet_MRAC1_args_nestLockGlobal);
  stateDeriv_withGrav_LiamSet_MRAC1_args_initialize();
  st.tls = emlrtRootTLSGlobal;
  emlrtSetJmpBuf(&st, &emlrtJBEnviron);
  if (setjmp(emlrtJBEnviron) == 0) {
    stateDeriv_withGrav_LiamSet_MRAC1_args_mexFunction(nlhs, plhs, nrhs, prhs);
    stateDeriv_withGrav_LiamSet_MRAC1_args_terminate();
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(
        &stateDeriv_withGrav_LiamSet_MRAC1_args_nestLockGlobal);
  } else {
    omp_destroy_lock(&emlrtLockGlobal);
    omp_destroy_nest_lock(
        &stateDeriv_withGrav_LiamSet_MRAC1_args_nestLockGlobal);
    emlrtReportParallelRunTimeError(&st);
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal,
                           &emlrtLockerFunction, omp_get_num_procs(), NULL,
                           "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void stateDeriv_withGrav_LiamSet_MRAC1_args_mexFunction(int32_T nlhs,
                                                        mxArray *plhs[1],
                                                        int32_T nrhs,
                                                        const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        38, "stateDeriv_withGrav_LiamSet_MRAC1_args");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 38,
                        "stateDeriv_withGrav_LiamSet_MRAC1_args");
  }
  /* Call the function. */
  c_stateDeriv_withGrav_LiamSet_M(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation (_coder_stateDeriv_withGrav_LiamSet_MRAC1_args_mex.c)
 */
