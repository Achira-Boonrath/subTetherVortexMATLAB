/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_AdaptiveLinear_args.h
 *
 * Code generation for function
 * 'stateDeriv_withGrav_LiamSet_AdaptiveLinear_args'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_types.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
emlrtCTX emlrtGetRootTLSGlobal(void);

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData);

void stateDeriv_withGrav_LiamSet_AdaptiveLinear_args(const emlrtStack *sp,
                                                     real_T t, real_T s[39],
                                                     const struct0_T *args,
                                                     real_T ds[39]);

/* End of code generation (stateDeriv_withGrav_LiamSet_AdaptiveLinear_args.h) */
