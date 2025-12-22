/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_stateDeriv_withGrav_LiamSet_Unified_args_mex.c
 *
 * Code generation for function
 * '_coder_stateDeriv_withGrav_LiamSet_Unified_args_mex'
 *
 */

/* Include files */
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_mex.h"
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_api.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_initialize.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_terminate.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&stateDeriv_withGrav_LiamSet_Unified_args_atexit);
  stateDeriv_withGrav_LiamSet_Unified_args_initialize();
  stateDeriv_withGrav_LiamSet_Unified_args_mexFunction(nlhs, plhs, nrhs, prhs);
  stateDeriv_withGrav_LiamSet_Unified_args_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void stateDeriv_withGrav_LiamSet_Unified_args_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[3])
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
                        40, "stateDeriv_withGrav_LiamSet_Unified_args");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 40,
                        "stateDeriv_withGrav_LiamSet_Unified_args");
  }
  /* Call the function. */
  c_stateDeriv_withGrav_LiamSet_U(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/* End of code generation
 * (_coder_stateDeriv_withGrav_LiamSet_Unified_args_mex.c) */
