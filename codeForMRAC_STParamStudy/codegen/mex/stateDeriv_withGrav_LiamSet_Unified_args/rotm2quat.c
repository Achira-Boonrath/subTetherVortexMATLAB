/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rotm2quat.c
 *
 * Code generation for function 'rotm2quat'
 *
 */

/* Include files */
#include "rotm2quat.h"
#include "anyNonFinite.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "warning.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo sb_emlrtRSI =
    {
        32,          /* lineNo */
        "rotm2quat", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutils\\rotm2qua"
        "t.m" /* pathName */
};

static emlrtRSInfo tb_emlrtRSI = {
    57,          /* lineNo */
    "rotm2quat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2quat.m" /* pathName */
};

static emlrtRSInfo ub_emlrtRSI = {
    81,    /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo vb_emlrtRSI = {
    125,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo wb_emlrtRSI = {
    133,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo xb_emlrtRSI = {
    141,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo cc_emlrtRSI = {
    24,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    40,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    10,        /* lineNo */
    "xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI = {
    61,              /* lineNo */
    "ceval_xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo dd_emlrtRSI = {
    12,                         /* lineNo */
    "eigSkewHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigSkew"
    "HermitianStandard.m" /* pathName */
};

static emlrtRSInfo ed_emlrtRSI = {
    22,                             /* lineNo */
    "eigRealSkewSymmetricStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigReal"
    "SkewSymmetricStandard.m" /* pathName */
};

static emlrtRSInfo oe_emlrtRSI = {
    24,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo pe_emlrtRSI = {
    45,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo qe_emlrtRSI = {
    159,           /* lineNo */
    "ceval_xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

static emlrtRSInfo re_emlrtRSI = {
    40,      /* lineNo */
    "xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

/* Function Definitions */
void rotm2quat(const emlrtStack *sp, const real_T R[9], real_T quat[4])
{
  static const char_T b_fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                     '_', 'd', 'g', 'e', 'e', 'v', 'x'};
  static const char_T fname[13] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 's', 'y', 'e', 'v'};
  ptrdiff_t ihi_t;
  ptrdiff_t ilo_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  creal_T V[16];
  creal_T D[4];
  real_T K[16];
  real_T vright[16];
  real_T scale[4];
  real_T wimag[4];
  real_T wreal[4];
  real_T K12;
  real_T K13;
  real_T K14;
  real_T K23;
  real_T K24;
  real_T K34;
  int32_T i;
  int32_T im_tmp;
  int32_T j;
  int32_T sgn;
  boolean_T exitg2;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &sb_emlrtRSI;
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
  K12 = R[1] + R[3];
  K13 = R[2] + R[6];
  K14 = R[5] - R[7];
  K23 = R[5] + R[7];
  K24 = R[6] - R[2];
  K34 = R[1] - R[3];
  K[0] = ((R[0] - R[4]) - R[8]) / 3.0;
  K[4] = K12 / 3.0;
  K[8] = K13 / 3.0;
  K[12] = K14 / 3.0;
  K[1] = K12 / 3.0;
  K[5] = ((R[4] - R[0]) - R[8]) / 3.0;
  K[9] = K23 / 3.0;
  K[13] = K24 / 3.0;
  K[2] = K13 / 3.0;
  K[6] = K23 / 3.0;
  K[10] = ((R[8] - R[0]) - R[4]) / 3.0;
  K[14] = K34 / 3.0;
  K[3] = K14 / 3.0;
  K[7] = K24 / 3.0;
  K[11] = K34 / 3.0;
  K[15] = ((R[0] + R[4]) + R[8]) / 3.0;
  b_st.site = &tb_emlrtRSI;
  c_st.site = &ub_emlrtRSI;
  if (anyNonFinite(K)) {
    for (i = 0; i < 16; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
    D[0].re = rtNaN;
    D[1].re = rtNaN;
    D[2].re = rtNaN;
    D[3].re = rtNaN;
  } else {
    int32_T exitg1;
    boolean_T p;
    p = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 4)) {
      i = 0;
      do {
        exitg1 = 0;
        if (i <= j) {
          if (!(K[i + (j << 2)] == K[j + (i << 2)])) {
            p = false;
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (p) {
      ptrdiff_t info_t;
      c_st.site = &vb_emlrtRSI;
      d_st.site = &cc_emlrtRSI;
      e_st.site = &ec_emlrtRSI;
      ilo_t = (ptrdiff_t)4;
      info_t = LAPACKE_dsyev(102, 'V', 'L', ilo_t, &K[0], ilo_t, &scale[0]);
      f_st.site = &fc_emlrtRSI;
      if ((int32_T)info_t < 0) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&f_st, &e_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&f_st, &f_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &fname[0], 12, (int32_T)info_t);
        }
      }
      D[0].re = scale[0];
      D[1].re = scale[1];
      D[2].re = scale[2];
      D[3].re = scale[3];
      for (i = 0; i < 16; i++) {
        V[i].re = K[i];
        V[i].im = 0.0;
      }
      if ((int32_T)info_t != 0) {
        d_st.site = &dc_emlrtRSI;
        warning(&d_st);
      }
    } else {
      p = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 4)) {
        i = 0;
        do {
          exitg1 = 0;
          if (i <= j) {
            if (!(K[i + (j << 2)] == -K[j + (i << 2)])) {
              p = false;
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (p) {
        real_T T[16];
        c_st.site = &wb_emlrtRSI;
        d_st.site = &dd_emlrtRSI;
        e_st.site = &ed_emlrtRSI;
        schur(&e_st, K, vright, T);
        i = 1;
        do {
          exitg1 = 0;
          if (i <= 4) {
            boolean_T guard1;
            guard1 = false;
            if (i != 4) {
              K12 = T[i + ((i - 1) << 2)];
              if (K12 != 0.0) {
                K12 = muDoubleScalarAbs(K12);
                D[i - 1].re = 0.0;
                D[i - 1].im = K12;
                D[i].re = 0.0;
                D[i].im = -K12;
                i += 2;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
            if (guard1) {
              D[i - 1].re = 0.0;
              D[i - 1].im = 0.0;
              i++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        for (i = 0; i < 16; i++) {
          V[i].re = vright[i];
          V[i].im = 0.0;
        }
        j = 1;
        do {
          exitg1 = 0;
          if (j <= 4) {
            if (j != 4) {
              i = (j - 1) << 2;
              K12 = T[j + i];
              if (K12 != 0.0) {
                if (K12 < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                K12 = V[i].re;
                im_tmp = j << 2;
                K13 = (real_T)sgn * V[im_tmp].re;
                if (K13 == 0.0) {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i].re = 0.0;
                  V[i].im = K13 / 1.4142135623730951;
                } else {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = K13 / 1.4142135623730951;
                }
                V[im_tmp].re = V[i].re;
                V[im_tmp].im = -V[i].im;
                K12 = V[i + 1].re;
                K13 = (real_T)sgn * V[im_tmp + 1].re;
                if (K13 == 0.0) {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 1].re = 0.0;
                  V[i + 1].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = K13 / 1.4142135623730951;
                }
                V[im_tmp + 1].re = V[i + 1].re;
                V[im_tmp + 1].im = -V[i + 1].im;
                K12 = V[i + 2].re;
                K13 = (real_T)sgn * V[im_tmp + 2].re;
                if (K13 == 0.0) {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 2].re = 0.0;
                  V[i + 2].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = K13 / 1.4142135623730951;
                }
                V[im_tmp + 2].re = V[i + 2].re;
                V[im_tmp + 2].im = -V[i + 2].im;
                K12 = V[i + 3].re;
                K13 = (real_T)sgn * V[im_tmp + 3].re;
                if (K13 == 0.0) {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 3].re = 0.0;
                  V[i + 3].im = K13 / 1.4142135623730951;
                } else {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = K13 / 1.4142135623730951;
                }
                V[im_tmp + 3].re = V[i + 3].re;
                V[im_tmp + 3].im = -V[i + 3].im;
                j += 2;
              } else {
                j++;
              }
            } else {
              j++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        ptrdiff_t info_t;
        c_st.site = &xb_emlrtRSI;
        d_st.site = &oe_emlrtRSI;
        e_st.site = &re_emlrtRSI;
        info_t = LAPACKE_dgeevx(102, 'B', 'N', 'V', 'N', (ptrdiff_t)4, &K[0],
                                (ptrdiff_t)4, &wreal[0], &wimag[0], &K12,
                                (ptrdiff_t)1, &vright[0], (ptrdiff_t)4, &ilo_t,
                                &ihi_t, &scale[0], &K13, &K14, &K23);
        f_st.site = &qe_emlrtRSI;
        if ((int32_T)info_t < 0) {
          if ((int32_T)info_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&f_st, &e_emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(
                &f_st, &f_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
                "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &b_fname[0], 12,
                (int32_T)info_t);
          }
        }
        D[0].re = wreal[0];
        D[1].re = wreal[1];
        D[2].re = wreal[2];
        D[3].re = wreal[3];
        for (i = 0; i < 16; i++) {
          V[i].re = vright[i];
          V[i].im = 0.0;
        }
        for (i = 0; i < 3; i++) {
          if ((wimag[i] > 0.0) && (wimag[i + 1] < 0.0)) {
            sgn = i << 2;
            im_tmp = (i + 1) << 2;
            K12 = V[im_tmp].re;
            V[sgn].im = K12;
            V[im_tmp].re = V[sgn].re;
            V[im_tmp].im = -K12;
            K12 = V[im_tmp + 1].re;
            V[sgn + 1].im = K12;
            V[im_tmp + 1].re = V[sgn + 1].re;
            V[im_tmp + 1].im = -K12;
            K12 = V[im_tmp + 2].re;
            V[sgn + 2].im = K12;
            V[im_tmp + 2].re = V[sgn + 2].re;
            V[im_tmp + 2].im = -K12;
            K12 = V[im_tmp + 3].re;
            V[sgn + 3].im = K12;
            V[im_tmp + 3].re = V[sgn + 3].re;
            V[im_tmp + 3].im = -K12;
          }
        }
        if ((int32_T)info_t != 0) {
          d_st.site = &pe_emlrtRSI;
          warning(&d_st);
        }
      }
    }
  }
  scale[0] = D[0].re;
  scale[1] = D[1].re;
  scale[2] = D[2].re;
  scale[3] = D[3].re;
  if (!muDoubleScalarIsNaN(D[0].re)) {
    sgn = 1;
  } else {
    sgn = 0;
    j = 2;
    exitg2 = false;
    while ((!exitg2) && (j < 5)) {
      if (!muDoubleScalarIsNaN(scale[j - 1])) {
        sgn = j;
        exitg2 = true;
      } else {
        j++;
      }
    }
  }
  if (sgn == 0) {
    im_tmp = 0;
  } else {
    K13 = scale[sgn - 1];
    im_tmp = sgn - 1;
    i = sgn + 1;
    for (j = i; j < 5; j++) {
      K12 = scale[j - 1];
      if (K13 < K12) {
        K13 = K12;
        im_tmp = j - 1;
      }
    }
  }
  sgn = im_tmp << 2;
  quat[0] = V[sgn + 3].re;
  quat[1] = V[sgn].re;
  quat[2] = V[sgn + 1].re;
  quat[3] = V[sgn + 2].re;
  if (quat[0] < 0.0) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&quat[0]);
    r1 = _mm_set1_pd(-1.0);
    _mm_storeu_pd(&quat[0], _mm_mul_pd(r, r1));
    r = _mm_loadu_pd(&quat[2]);
    _mm_storeu_pd(&quat[2], _mm_mul_pd(r, r1));
  }
}

/* End of code generation (rotm2quat.c) */
