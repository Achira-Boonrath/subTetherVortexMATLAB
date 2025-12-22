/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * calculateAdaptiveControl_Unified.h
 *
 * Code generation for function 'calculateAdaptiveControl_Unified'
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
void c_calculateAdaptiveControl_Unif(
    const emlrtStack *sp, real_T t, const real_T s[95], const struct0_T *args,
    real_T mrac_idx, real_T l_mt_seg1, const real_T VR_mt_seg1[3],
    const real_T evec_mt_seg1[3], boolean_T maskMTreal,
    const real_T rotMat_C_A_I[9], real_T ds_mrac[9], real_T Fthrust[3],
    real_T MRAC_1_updateParams[7], real_T MRAC_2_updateParams[11]);

/* End of code generation (calculateAdaptiveControl_Unified.h) */
