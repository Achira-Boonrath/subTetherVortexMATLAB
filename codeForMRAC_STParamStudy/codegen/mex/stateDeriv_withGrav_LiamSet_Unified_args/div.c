/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * div.c
 *
 * Code generation for function 'div'
 *
 */

/* Include files */
#include "div.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_emxutil.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"
#include <emmintrin.h>

/* Function Definitions */
void rdivide(const emlrtStack *sp, emxArray_real_T *in1,
             const emxArray_real_T *in2)
{
  emxArray_real_T *b_in1;
  const real_T *in2_data;
  real_T *b_in1_data;
  real_T *in1_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &b_in1, 2, &p_emlrtRTEI);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = 3;
  if (in2->size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2->size[1];
  }
  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, b_in1, i, &p_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    __m128d r;
    r = _mm_loadu_pd(&in1_data[3 * aux_0_1]);
    _mm_storeu_pd(&b_in1_data[3 * i],
                  _mm_div_pd(r, _mm_set1_pd(in2_data[aux_1_1])));
    b_in1_data[3 * i + 2] = in1_data[3 * aux_0_1 + 2] / in2_data[aux_1_1];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 3;
  emxEnsureCapacity_real_T(sp, in1, i, &p_emlrtRTEI);
  loop_ub = b_in1->size[1];
  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(sp, in1, i, &p_emlrtRTEI);
  in1_data = in1->data;
  for (i = 0; i < loop_ub; i++) {
    in1_data[3 * i] = b_in1_data[3 * i];
    stride_0_1 = 3 * i + 1;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
    stride_0_1 = 3 * i + 2;
    in1_data[stride_0_1] = b_in1_data[stride_0_1];
  }
  emxFree_real_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (div.c) */
