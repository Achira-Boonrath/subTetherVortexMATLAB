/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * schur.c
 *
 * Code generation for function 'schur'
 *
 */

/* Include files */
#include "schur.h"
#include "anyNonFinite.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "warning.h"
#include "xhseqr.h"
#include "xnrm2.h"
#include "xscal.h"
#include "xzlarf.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo gc_emlrtRSI = {
    20,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI = {
    41,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI = {
    53,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI = {
    68,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI = {
    71,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI = {
    81,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo fd_emlrtRSI = {
    35,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo gd_emlrtRSI = {
    66,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo hd_emlrtRSI = {
    69,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo id_emlrtRSI = {
    70,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo jd_emlrtRSI = {
    83,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo kd_emlrtRSI = {
    18,       /* lineNo */
    "xgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgehrd.m" /* pathName */
};

static emlrtRSInfo ld_emlrtRSI = {
    46,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo md_emlrtRSI = {
    50,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo nd_emlrtRSI = {
    58,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo od_emlrtRSI = {
    84,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo pd_emlrtRSI = {
    91,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo qd_emlrtRSI = {
    58,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

static emlrtRSInfo rd_emlrtRSI = {
    72,                /* lineNo */
    "ceval_xungorghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xungorghr.m" /* pathName */
};

static emlrtRSInfo sd_emlrtRSI = {
    11,          /* lineNo */
    "xungorghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xungorghr.m" /* pathName */
};

/* Function Definitions */
void schur(const emlrtStack *sp, const real_T A[16], real_T V[16], real_T T[16])
{
  static const char_T fname[14] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'o', 'r', 'g', 'h', 'r'};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  real_T work[4];
  real_T tau[3];
  int32_T b_i;
  int32_T ia;
  int32_T j;
  int32_T k;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &fd_emlrtRSI;
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
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  if (anyNonFinite(A)) {
    int32_T i;
    for (j = 0; j < 16; j++) {
      V[j] = rtNaN;
    }
    i = 2;
    for (j = 0; j < 3; j++) {
      if (i <= 4) {
        memset(&V[(j * 4 + i) + -1], 0, (uint32_T)(-i + 5) * sizeof(real_T));
      }
      i++;
    }
    for (j = 0; j < 16; j++) {
      T[j] = rtNaN;
    }
  } else {
    ptrdiff_t info_t;
    int32_T i;
    boolean_T p;
    st.site = &gd_emlrtRSI;
    memcpy(&T[0], &A[0], 16U * sizeof(real_T));
    b_st.site = &kd_emlrtRSI;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      __m128d r;
      real_T alpha1_tmp;
      real_T xnorm;
      int32_T alpha1_tmp_tmp;
      int32_T in;
      int32_T knt;
      int32_T lastc;
      int32_T lastv;
      int32_T scalarLB;
      i = b_i << 2;
      in = (b_i + 1) << 2;
      alpha1_tmp_tmp = (b_i + i) + 1;
      alpha1_tmp = T[alpha1_tmp_tmp];
      knt = b_i + 3;
      i += muIntScalarMin_sint32(knt, 4);
      c_st.site = &ld_emlrtRSI;
      tau[b_i] = 0.0;
      d_st.site = &gc_emlrtRSI;
      xnorm = xnrm2(&d_st, 2 - b_i, T, i);
      if (xnorm != 0.0) {
        xnorm = muDoubleScalarHypot(alpha1_tmp, xnorm);
        if (alpha1_tmp >= 0.0) {
          xnorm = -xnorm;
        }
        if (muDoubleScalarAbs(xnorm) < 1.0020841800044864E-292) {
          knt = 0;
          j = (i - b_i) + 1;
          do {
            knt++;
            d_st.site = &hc_emlrtRSI;
            e_st.site = &oc_emlrtRSI;
            f_st.site = &pc_emlrtRSI;
            scalarLB = ((((j - i) + 1) / 2) << 1) + i;
            vectorUB = scalarLB - 2;
            for (k = i; k <= vectorUB; k += 2) {
              r = _mm_loadu_pd(&T[k - 1]);
              _mm_storeu_pd(&T[k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (k = scalarLB; k <= j; k++) {
              T[k - 1] *= 9.9792015476736E+291;
            }
            xnorm *= 9.9792015476736E+291;
            alpha1_tmp *= 9.9792015476736E+291;
          } while ((muDoubleScalarAbs(xnorm) < 1.0020841800044864E-292) &&
                   (knt < 20));
          d_st.site = &ic_emlrtRSI;
          xnorm = xnrm2(&d_st, 2 - b_i, T, i);
          xnorm = muDoubleScalarHypot(alpha1_tmp, xnorm);
          if (alpha1_tmp >= 0.0) {
            xnorm = -xnorm;
          }
          tau[b_i] = (xnorm - alpha1_tmp) / xnorm;
          d_st.site = &jc_emlrtRSI;
          xscal(&d_st, 2 - b_i, 1.0 / (alpha1_tmp - xnorm), T, i);
          d_st.site = &kc_emlrtRSI;
          for (k = 0; k < knt; k++) {
            xnorm *= 1.0020841800044864E-292;
          }
          alpha1_tmp = xnorm;
        } else {
          tau[b_i] = (xnorm - alpha1_tmp) / xnorm;
          d_st.site = &lc_emlrtRSI;
          xscal(&d_st, 2 - b_i, 1.0 / (alpha1_tmp - xnorm), T, i);
          alpha1_tmp = xnorm;
        }
      }
      T[alpha1_tmp_tmp] = 1.0;
      scalarLB = in + 1;
      c_st.site = &md_emlrtRSI;
      if (tau[b_i] != 0.0) {
        boolean_T exitg2;
        lastv = 2 - b_i;
        i = (alpha1_tmp_tmp - b_i) + 2;
        while ((lastv + 1 > 0) && (T[i] == 0.0)) {
          lastv--;
          i--;
        }
        lastc = 4;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int32_T exitg1;
          knt = in + lastc;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia <= knt + (lastv << 2)) {
              if (T[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 4;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        d_st.site = &od_emlrtRSI;
        e_st.site = &uc_emlrtRSI;
        if (lastc != 0) {
          f_st.site = &wc_emlrtRSI;
          memset(&work[0], 0, (uint32_T)lastc * sizeof(real_T));
          i = alpha1_tmp_tmp;
          j = (in + (lastv << 2)) + 1;
          for (vectorUB = scalarLB; vectorUB <= j; vectorUB += 4) {
            knt = vectorUB + lastc;
            f_st.site = &qd_emlrtRSI;
            for (ia = vectorUB; ia < knt; ia++) {
              k = ia - vectorUB;
              work[k] += T[ia - 1] * T[i];
            }
            i++;
          }
        }
        d_st.site = &pd_emlrtRSI;
        e_st.site = &xc_emlrtRSI;
        f_st.site = &yc_emlrtRSI;
        g_st.site = &ad_emlrtRSI;
        if (!(-tau[b_i] == 0.0)) {
          i = in;
          h_st.site = &bd_emlrtRSI;
          for (j = 0; j <= lastv; j++) {
            xnorm = T[alpha1_tmp_tmp + j];
            if (xnorm != 0.0) {
              xnorm *= -tau[b_i];
              knt = i + 1;
              k = lastc + i;
              h_st.site = &cd_emlrtRSI;
              if ((i + 1 <= k) && (k > 2147483646)) {
                i_st.site = &hb_emlrtRSI;
                check_forloop_overflow_error(&i_st);
              }
              scalarLB = ((((k - i) / 2) << 1) + i) + 1;
              vectorUB = scalarLB - 2;
              for (ia = knt; ia <= vectorUB; ia += 2) {
                __m128d r1;
                r = _mm_loadu_pd(&work[(ia - i) - 1]);
                r1 = _mm_loadu_pd(&T[ia - 1]);
                _mm_storeu_pd(
                    &T[ia - 1],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xnorm))));
              }
              for (ia = scalarLB; ia <= k; ia++) {
                T[ia - 1] += work[(ia - i) - 1] * xnorm;
              }
            }
            i += 4;
          }
        }
      }
      c_st.site = &nd_emlrtRSI;
      xzlarf(&c_st, 3 - b_i, 3 - b_i, alpha1_tmp_tmp + 1, tau[b_i], T,
             (b_i + in) + 2, work);
      T[alpha1_tmp_tmp] = alpha1_tmp;
    }
    st.site = &hd_emlrtRSI;
    memcpy(&V[0], &T[0], 16U * sizeof(real_T));
    b_st.site = &sd_emlrtRSI;
    info_t = LAPACKE_dorghr(102, (ptrdiff_t)4, (ptrdiff_t)1, (ptrdiff_t)4,
                            &V[0], (ptrdiff_t)4, &tau[0]);
    c_st.site = &rd_emlrtRSI;
    if ((int32_T)info_t != 0) {
      boolean_T b_p;
      p = true;
      b_p = false;
      if ((int32_T)info_t == -5) {
        b_p = true;
      } else if ((int32_T)info_t == -7) {
        b_p = true;
      }
      if (!b_p) {
        if ((int32_T)info_t == -1010) {
          emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI, "MATLAB:nomem",
                                        "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&c_st, &f_emlrtRTEI,
                                        "Coder:toolbox:LAPACKCallErrorInfo",
                                        "Coder:toolbox:LAPACKCallErrorInfo", 5,
                                        4, 14, &fname[0], 12, (int32_T)info_t);
        }
      }
    } else {
      p = false;
    }
    if (p) {
      for (j = 0; j < 16; j++) {
        V[j] = rtNaN;
      }
    }
    st.site = &id_emlrtRSI;
    i = xhseqr(&st, T, V);
    if (i != 0) {
      st.site = &jd_emlrtRSI;
      b_warning(&st);
    }
  }
}

/* End of code generation (schur.c) */
