/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rotm2quat.h
 *
 * Code generation for function 'rotm2quat'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void rotm2quat(const emlrtStack *sp, const real_T R[9], real_T quat[4]);

/* End of code generation (rotm2quat.h) */
