/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_emxutil.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"
#include "sumMatrixIncludeNaN.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo bb_emlrtRSI = {
    20,    /* lineNo */
    "sum", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\sum.m" /* pathName
                                                                        */
};

static emlrtRSInfo cb_emlrtRSI = {
    99,        /* lineNo */
    "sumprod", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumpro"
    "d.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI = {
    86,                      /* lineNo */
    "combineVectorElements", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\combin"
    "eVectorElements.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    107,                /* lineNo */
    "blockedSummation", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\blocke"
    "dSummation.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    22,                    /* lineNo */
    "sumMatrixIncludeNaN", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    41,                 /* lineNo */
    "sumMatrixColumns", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pathName */
};

static emlrtRTEInfo pb_emlrtRTEI = {
    35,                    /* lineNo */
    20,                    /* colNo */
    "sumMatrixIncludeNaN", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\datafun\\private\\sumMat"
    "rixIncludeNaN.m" /* pName */
};

/* Function Definitions */
void b_sum(const real_T x[12], real_T y[3])
{
  __m128d r;
  y[0] = x[0];
  y[1] = x[1];
  y[2] = x[2];
  r = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&y[0], _mm_add_pd(r, _mm_loadu_pd(&x[3])));
  y[2] += x[5];
  r = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&y[0], _mm_add_pd(r, _mm_loadu_pd(&x[6])));
  y[2] += x[8];
  r = _mm_loadu_pd(&y[0]);
  _mm_storeu_pd(&y[0], _mm_add_pd(r, _mm_loadu_pd(&x[9])));
  y[2] += x[11];
}

void sum(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  real_T *y_data;
  int32_T col;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  b_st.site = &cb_emlrtRSI;
  c_st.site = &db_emlrtRSI;
  if (x->size[1] == 0) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    int32_T i;
    d_st.site = &eb_emlrtRSI;
    e_st.site = &fb_emlrtRSI;
    col = y->size[0] * y->size[1];
    y->size[0] = 1;
    i = x->size[1];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(&e_st, y, col, &pb_emlrtRTEI);
    y_data = y->data;
    f_st.site = &gb_emlrtRSI;
    if (x->size[1] > 2147483646) {
      g_st.site = &hb_emlrtRSI;
      check_forloop_overflow_error(&g_st);
    }
    for (col = 0; col < i; col++) {
      y_data[col] = sumColumnB(x, col + 1);
    }
  }
}

/* End of code generation (sum.c) */
