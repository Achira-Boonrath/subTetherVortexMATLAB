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
#include "rt_nonfinite.h"
#include "schur.h"
#include "stateDeriv_withGrav_LiamSet_AdaptiveLinear_args_data.h"
#include "warning.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>

/* Variable Definitions */
static emlrtRSInfo t_emlrtRSI =
    {
        34,          /* lineNo */
        "rotm2quat", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\shared\\robotics\\robotutils\\rotm2qua"
        "t.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    57,          /* lineNo */
    "rotm2quat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\shared\\robotics\\robotutilsint\\+"
    "robotics\\+internal\\rotm2quat.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    81,    /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo w_emlrtRSI = {
    125,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo x_emlrtRSI = {
    133,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo y_emlrtRSI = {
    141,   /* lineNo */
    "eig", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\eig.m" /* pathName
                                                                       */
};

static emlrtRSInfo db_emlrtRSI = {
    24,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo eb_emlrtRSI = {
    40,                     /* lineNo */
    "eigHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigHerm"
    "itianStandard.m" /* pathName */
};

static emlrtRSInfo fb_emlrtRSI = {
    10,        /* lineNo */
    "xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI = {
    62,              /* lineNo */
    "ceval_xsyheev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xsyheev.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI = {
    12,                         /* lineNo */
    "eigSkewHermitianStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigSkew"
    "HermitianStandard.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI = {
    22,                             /* lineNo */
    "eigRealSkewSymmetricStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigReal"
    "SkewSymmetricStandard.m" /* pathName */
};

static emlrtRSInfo td_emlrtRSI = {
    24,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    45,            /* lineNo */
    "eigStandard", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\private\\eigStan"
    "dard.m" /* pathName */
};

static emlrtRSInfo vd_emlrtRSI = {
    159,           /* lineNo */
    "ceval_xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

static emlrtRSInfo wd_emlrtRSI = {
    40,      /* lineNo */
    "xgeev", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgeev.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI = {
    45,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    48,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\infocheck.m" /* pName */
};

/* Function Definitions */
void rotm2quat(const emlrtStack *sp, const real_T R[9], real_T quat[4])
{
  static const char_T b_fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                     '_', 'd', 'g', 'e', 'e', 'v', 'x'};
  static const char_T fname[13] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 's', 'y', 'e', 'v'};
  ptrdiff_t ihi_t;
  ptrdiff_t n_t;
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
  real_T abnrm;
  real_T lambda;
  real_T rconde;
  real_T rcondv;
  int32_T i;
  int32_T idx;
  int32_T k;
  int32_T sgn;
  boolean_T exitg2;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
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
  st.site = &t_emlrtRSI;
  K[0] = ((R[0] - R[4]) - R[8]) / 3.0;
  lambda = (R[1] + R[3]) / 3.0;
  K[4] = lambda;
  abnrm = (R[2] + R[6]) / 3.0;
  K[8] = abnrm;
  rcondv = (R[5] - R[7]) / 3.0;
  K[12] = rcondv;
  K[1] = lambda;
  K[5] = ((R[4] - R[0]) - R[8]) / 3.0;
  lambda = (R[5] + R[7]) / 3.0;
  K[9] = lambda;
  rconde = (R[6] - R[2]) / 3.0;
  K[13] = rconde;
  K[2] = abnrm;
  K[6] = lambda;
  K[10] = ((R[8] - R[0]) - R[4]) / 3.0;
  lambda = (R[1] - R[3]) / 3.0;
  K[14] = lambda;
  K[3] = rcondv;
  K[7] = rconde;
  K[11] = lambda;
  K[15] = ((R[0] + R[4]) + R[8]) / 3.0;
  b_st.site = &u_emlrtRSI;
  c_st.site = &v_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  e_st.site = &bb_emlrtRSI;
  p = true;
  for (k = 0; k < 16; k++) {
    if (p) {
      lambda = K[k];
      if (muDoubleScalarIsInf(lambda) || muDoubleScalarIsNaN(lambda)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (k = 0; k < 16; k++) {
      V[k].re = rtNaN;
      V[k].im = 0.0;
    }
    D[0].re = rtNaN;
    D[1].re = rtNaN;
    D[2].re = rtNaN;
    D[3].re = rtNaN;
  } else {
    int32_T exitg1;
    p = true;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 4)) {
      idx = 0;
      do {
        exitg1 = 0;
        if (idx <= i) {
          if (!(K[idx + (i << 2)] == K[i + (idx << 2)])) {
            p = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        } else {
          i++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (p) {
      c_st.site = &w_emlrtRSI;
      d_st.site = &db_emlrtRSI;
      e_st.site = &fb_emlrtRSI;
      n_t = (ptrdiff_t)4;
      n_t = LAPACKE_dsyev(102, 'V', 'L', n_t, &K[0], n_t, &scale[0]);
      f_st.site = &gb_emlrtRSI;
      if ((int32_T)n_t < 0) {
        if ((int32_T)n_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&f_st, &emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&f_st, &b_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 13, &fname[0], 12, (int32_T)n_t);
        }
      }
      D[0].re = scale[0];
      D[1].re = scale[1];
      D[2].re = scale[2];
      D[3].re = scale[3];
      for (k = 0; k < 16; k++) {
        V[k].re = K[k];
        V[k].im = 0.0;
      }
      if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&c_st))) {
        d_st.site = &eb_emlrtRSI;
        warning(&d_st);
      }
    } else {
      p = true;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 4)) {
        idx = 0;
        do {
          exitg1 = 0;
          if (idx <= i) {
            if (!(K[idx + (i << 2)] == -K[i + (idx << 2)])) {
              p = false;
              exitg1 = 1;
            } else {
              idx++;
            }
          } else {
            i++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (p) {
        real_T T[16];
        c_st.site = &x_emlrtRSI;
        d_st.site = &hc_emlrtRSI;
        e_st.site = &ic_emlrtRSI;
        schur(&e_st, K, vright, T);
        i = 1;
        do {
          exitg1 = 0;
          if (i <= 4) {
            boolean_T guard1;
            guard1 = false;
            if (i != 4) {
              lambda = T[i + ((i - 1) << 2)];
              if (lambda != 0.0) {
                lambda = muDoubleScalarAbs(lambda);
                D[i - 1].re = 0.0;
                D[i - 1].im = lambda;
                D[i].re = 0.0;
                D[i].im = -lambda;
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
        for (k = 0; k < 16; k++) {
          V[k].re = vright[k];
          V[k].im = 0.0;
        }
        i = 1;
        do {
          exitg1 = 0;
          if (i <= 4) {
            if (i != 4) {
              idx = (i - 1) << 2;
              lambda = T[i + idx];
              if (lambda != 0.0) {
                int32_T b_i;
                if (lambda < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                lambda = V[idx].re;
                b_i = i << 2;
                abnrm = (real_T)sgn * V[b_i].re;
                if (abnrm == 0.0) {
                  V[idx].re = lambda / 1.4142135623730951;
                  V[idx].im = 0.0;
                } else if (lambda == 0.0) {
                  V[idx].re = 0.0;
                  V[idx].im = abnrm / 1.4142135623730951;
                } else {
                  V[idx].re = lambda / 1.4142135623730951;
                  V[idx].im = abnrm / 1.4142135623730951;
                }
                V[b_i].re = V[idx].re;
                V[b_i].im = -V[idx].im;
                lambda = V[idx + 1].re;
                abnrm = (real_T)sgn * V[b_i + 1].re;
                if (abnrm == 0.0) {
                  V[idx + 1].re = lambda / 1.4142135623730951;
                  V[idx + 1].im = 0.0;
                } else if (lambda == 0.0) {
                  V[idx + 1].re = 0.0;
                  V[idx + 1].im = abnrm / 1.4142135623730951;
                } else {
                  V[idx + 1].re = lambda / 1.4142135623730951;
                  V[idx + 1].im = abnrm / 1.4142135623730951;
                }
                V[b_i + 1].re = V[idx + 1].re;
                V[b_i + 1].im = -V[idx + 1].im;
                lambda = V[idx + 2].re;
                abnrm = (real_T)sgn * V[b_i + 2].re;
                if (abnrm == 0.0) {
                  V[idx + 2].re = lambda / 1.4142135623730951;
                  V[idx + 2].im = 0.0;
                } else if (lambda == 0.0) {
                  V[idx + 2].re = 0.0;
                  V[idx + 2].im = abnrm / 1.4142135623730951;
                } else {
                  V[idx + 2].re = lambda / 1.4142135623730951;
                  V[idx + 2].im = abnrm / 1.4142135623730951;
                }
                V[b_i + 2].re = V[idx + 2].re;
                V[b_i + 2].im = -V[idx + 2].im;
                lambda = V[idx + 3].re;
                abnrm = (real_T)sgn * V[b_i + 3].re;
                if (abnrm == 0.0) {
                  V[idx + 3].re = lambda / 1.4142135623730951;
                  V[idx + 3].im = 0.0;
                } else if (lambda == 0.0) {
                  V[idx + 3].re = 0.0;
                  V[idx + 3].im = abnrm / 1.4142135623730951;
                } else {
                  V[idx + 3].re = lambda / 1.4142135623730951;
                  V[idx + 3].im = abnrm / 1.4142135623730951;
                }
                V[b_i + 3].re = V[idx + 3].re;
                V[b_i + 3].im = -V[idx + 3].im;
                i += 2;
              } else {
                i++;
              }
            } else {
              i++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        c_st.site = &y_emlrtRSI;
        d_st.site = &td_emlrtRSI;
        e_st.site = &wd_emlrtRSI;
        n_t = LAPACKE_dgeevx(102, 'B', 'N', 'V', 'N', (ptrdiff_t)4, &K[0],
                             (ptrdiff_t)4, &wreal[0], &wimag[0], &lambda,
                             (ptrdiff_t)1, &vright[0], (ptrdiff_t)4, &n_t,
                             &ihi_t, &scale[0], &abnrm, &rconde, &rcondv);
        f_st.site = &vd_emlrtRSI;
        if ((int32_T)n_t < 0) {
          if ((int32_T)n_t == -1010) {
            emlrtErrorWithMessageIdR2018a(&f_st, &emlrtRTEI, "MATLAB:nomem",
                                          "MATLAB:nomem", 0);
          } else {
            emlrtErrorWithMessageIdR2018a(
                &f_st, &b_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
                "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, &b_fname[0], 12,
                (int32_T)n_t);
          }
        }
        D[0].re = wreal[0];
        D[1].re = wreal[1];
        D[2].re = wreal[2];
        D[3].re = wreal[3];
        for (k = 0; k < 16; k++) {
          V[k].re = vright[k];
          V[k].im = 0.0;
        }
        for (k = 0; k < 3; k++) {
          if ((wimag[k] > 0.0) && (wimag[k + 1] < 0.0)) {
            i = k << 2;
            idx = (k + 1) << 2;
            lambda = V[idx].re;
            V[i].im = lambda;
            V[idx].re = V[i].re;
            V[idx].im = -lambda;
            lambda = V[idx + 1].re;
            V[i + 1].im = lambda;
            V[idx + 1].re = V[i + 1].re;
            V[idx + 1].im = -lambda;
            lambda = V[idx + 2].re;
            V[i + 2].im = lambda;
            V[idx + 2].re = V[i + 2].re;
            V[idx + 2].im = -lambda;
            lambda = V[idx + 3].re;
            V[i + 3].im = lambda;
            V[idx + 3].re = V[i + 3].re;
            V[idx + 3].im = -lambda;
          }
        }
        if (((int32_T)n_t != 0) && (!emlrtSetWarningFlag(&c_st))) {
          d_st.site = &ud_emlrtRSI;
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
    idx = 1;
  } else {
    idx = 0;
    i = 2;
    exitg2 = false;
    while ((!exitg2) && (i < 5)) {
      if (!muDoubleScalarIsNaN(scale[i - 1])) {
        idx = i;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  if (idx == 0) {
    sgn = 0;
  } else {
    lambda = scale[idx - 1];
    sgn = idx - 1;
    i = idx + 1;
    for (k = i; k < 5; k++) {
      abnrm = scale[k - 1];
      if (lambda < abnrm) {
        lambda = abnrm;
        sgn = k - 1;
      }
    }
  }
  i = sgn << 2;
  quat[0] = V[i + 3].re;
  quat[1] = V[i].re;
  quat[2] = V[i + 1].re;
  quat[3] = V[i + 2].re;
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
