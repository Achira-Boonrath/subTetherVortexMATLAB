/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sumMatrixIncludeNaN.c
 *
 * Code generation for function 'sumMatrixIncludeNaN'
 *
 */

/* Include files */
#include "sumMatrixIncludeNaN.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"

/* Function Definitions */
real_T sumColumnB(const emxArray_real_T *x, int32_T col)
{
  const real_T *x_data;
  int32_T i0;
  x_data = x->data;
  i0 = (col - 1) * 3;
  return (x_data[i0] + x_data[i0 + 1]) + x_data[i0 + 2];
}

/* End of code generation (sumMatrixIncludeNaN.c) */
