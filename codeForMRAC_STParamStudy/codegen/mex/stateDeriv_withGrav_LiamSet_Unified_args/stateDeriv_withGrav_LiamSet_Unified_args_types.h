/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_Unified_args_types.h
 *
 * Code generation for function 'stateDeriv_withGrav_LiamSet_Unified_args'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  real_T N_mt_nodes;
  real_T L0;
  real_T kA;
  real_T gamma;
  real_T sigmaMRAC_h;
  real_T sigmaMRAC_a1;
  real_T sigmaMRAC_a2;
  real_T Gamma_x[4];
  real_T Gamma_r;
  real_T Gamma_theta[4];
  real_T P[4];
  real_T B_linear[2];
  real_T sigmaMRACLin;
  real_T massPoint1;
  real_T targetM;
  real_T targetI[9];
  real_T chaserM;
  real_T chaserI[9];
  real_T chaserSideLength;
  real_T targetSideLengthX;
  real_T targetSideLengthY;
  real_T targetSideLengthZ;
  real_T Kvec[5];
  real_T l0vec[5];
  real_T cVec[5];
  real_T mu;
  real_T FT;
  real_T x1_m0;
  real_T tOnChaser_fromt0;
  real_T slopeTime;
  real_T desElong;
  real_T ThrustSaturation;
  real_T Kp;
  real_T Kd;
  real_T J2on;
  real_T MRAC_v;
  real_T ODEscale;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

/* End of code generation (stateDeriv_withGrav_LiamSet_Unified_args_types.h) */
