/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xrot.c
 *
 * Code generation for function 'xrot'
 *
 */

/* Include files */
#include "xrot.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"

/* Variable Definitions */
static emlrtRSInfo me_emlrtRSI =
    {
        32,     /* lineNo */
        "xrot", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
        "blas\\xrot.m" /* pathName */
};

static emlrtRSInfo ne_emlrtRSI = {
    24,     /* lineNo */
    "xrot", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xrot.m" /* pathName */
};

/* Function Definitions */
void xrot(const emlrtStack *sp, int32_T n, real_T x[16], int32_T ix0,
          int32_T iy0, real_T c, real_T s)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &me_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &ne_emlrtRSI;
  if (n > 2147483646) {
    c_st.site = &hb_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  i = (uint8_T)n;
  for (k = 0; k < i; k++) {
    real_T b_temp_tmp;
    real_T c_temp_tmp;
    int32_T temp_tmp;
    int32_T temp_tmp_tmp;
    temp_tmp = k << 2;
    temp_tmp_tmp = (iy0 + temp_tmp) - 1;
    b_temp_tmp = x[temp_tmp_tmp];
    temp_tmp = (ix0 + temp_tmp) - 1;
    c_temp_tmp = x[temp_tmp];
    x[temp_tmp_tmp] = c * b_temp_tmp - s * c_temp_tmp;
    x[temp_tmp] = c * c_temp_tmp + s * b_temp_tmp;
  }
}

/* End of code generation (xrot.c) */
