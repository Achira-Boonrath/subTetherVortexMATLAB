/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * assertCompatibleDims.c
 *
 * Code generation for function 'assertCompatibleDims'
 *
 */

/* Include files */
#include "assertCompatibleDims.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"

/* Variable Definitions */
static emlrtRTEInfo d_emlrtRTEI = {
    13,                     /* lineNo */
    27,                     /* colNo */
    "assertCompatibleDims", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\assertCompatibleDims.m" /* pName */
};

/* Function Definitions */
void assertCompatibleDims(const emlrtStack *sp, const emxArray_real_T *x,
                          const emxArray_real_T *y)
{
  if ((x->size[1] != 1) && (y->size[1] != 1) && (x->size[1] != y->size[1])) {
    emlrtErrorWithMessageIdR2018a(sp, &d_emlrtRTEI,
                                  "MATLAB:sizeDimensionsMustMatch",
                                  "MATLAB:sizeDimensionsMustMatch", 0);
  }
}

/* End of code generation (assertCompatibleDims.c) */
