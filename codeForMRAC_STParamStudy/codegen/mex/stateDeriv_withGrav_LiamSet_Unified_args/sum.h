/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.h
 *
 * Code generation for function 'sum'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_sum(const real_T x[12], real_T y[3]);

void sum(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y);

/* End of code generation (sum.h) */
