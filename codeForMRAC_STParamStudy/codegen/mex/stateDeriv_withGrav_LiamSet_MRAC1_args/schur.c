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
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_data.h"
#include "warning.h"
#include "xhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo fb_emlrtRSI =
    {
        121,                  /* lineNo */
        "flatVectorAllOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\vAllOrAny.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI = {
    20,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    41,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo mb_emlrtRSI = {
    53,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo nb_emlrtRSI = {
    68,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo ob_emlrtRSI = {
    71,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo pb_emlrtRSI = {
    81,        /* lineNo */
    "xzlarfg", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarfg.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI = {
    31,      /* lineNo */
    "xscal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xscal."
    "m" /* pathName */
};

static emlrtRSInfo tb_emlrtRSI = {
    18,      /* lineNo */
    "xscal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xscal.m" /* pathName */
};

static emlrtRSInfo ub_emlrtRSI = {
    34,        /* lineNo */
    "xzungqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzungqr.m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI = {
    41,        /* lineNo */
    "xzungqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzungqr.m" /* pathName */
};

static emlrtRSInfo wb_emlrtRSI = {
    46,        /* lineNo */
    "xzungqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzungqr.m" /* pathName */
};

static emlrtRSInfo mc_emlrtRSI = {
    35,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo nc_emlrtRSI = {
    66,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo oc_emlrtRSI = {
    69,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo pc_emlrtRSI = {
    70,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo qc_emlrtRSI = {
    83,      /* lineNo */
    "schur", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\lib\\matlab\\matfun\\schur.m" /* pathName
                                                                         */
};

static emlrtRSInfo rc_emlrtRSI = {
    18,       /* lineNo */
    "xgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgehrd.m" /* pathName */
};

static emlrtRSInfo sc_emlrtRSI = {
    46,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo tc_emlrtRSI = {
    50,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo uc_emlrtRSI = {
    58,        /* lineNo */
    "xzgehrd", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgehrd.m" /* pathName */
};

static emlrtRSInfo vc_emlrtRSI = {
    84,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo wc_emlrtRSI = {
    91,       /* lineNo */
    "xzlarf", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzlarf.m" /* pathName */
};

static emlrtRSInfo xc_emlrtRSI = {
    58,      /* lineNo */
    "xgemv", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgemv.m" /* pathName */
};

static emlrtRSInfo yc_emlrtRSI = {
    14,          /* lineNo */
    "xungorghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xungorghr.m" /* pathName */
};

static emlrtRSInfo ad_emlrtRSI = {
    15,        /* lineNo */
    "xzunghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzunghr.m" /* pathName */
};

static emlrtRSInfo bd_emlrtRSI = {
    53,        /* lineNo */
    "xzunghr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzunghr.m" /* pathName */
};

/* Function Definitions */
void schur(const emlrtStack *sp, const real_T A[16], real_T V[16], real_T T[16])
{
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
  real_T xnorm;
  int32_T b_i;
  int32_T i;
  int32_T k;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  st.site = &mc_emlrtRSI;
  b_st.site = &db_emlrtRSI;
  c_st.site = &eb_emlrtRSI;
  p = true;
  d_st.site = &fb_emlrtRSI;
  for (k = 0; k < 16; k++) {
    if (p) {
      xnorm = A[k];
      if (muDoubleScalarIsInf(xnorm) || muDoubleScalarIsNaN(xnorm)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    int32_T istart;
    for (k = 0; k < 16; k++) {
      V[k] = rtNaN;
    }
    istart = 2;
    for (k = 0; k < 3; k++) {
      if (istart <= 4) {
        memset(&V[(k * 4 + istart) + -1], 0,
               (uint32_T)(-istart + 5) * sizeof(real_T));
      }
      istart++;
    }
    for (k = 0; k < 16; k++) {
      T[k] = rtNaN;
    }
  } else {
    __m128d r;
    real_T tau[3];
    int32_T c;
    int32_T ia;
    int32_T iaii;
    int32_T istart;
    int32_T knt;
    int32_T work_tmp;
    st.site = &nc_emlrtRSI;
    memcpy(&T[0], &A[0], 16U * sizeof(real_T));
    b_st.site = &rc_emlrtRSI;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (i = 0; i < 3; i++) {
      real_T alpha1;
      int32_T alpha1_tmp;
      int32_T in;
      int32_T lastc;
      int32_T lastv;
      istart = i << 2;
      in = (i + 1) << 2;
      alpha1_tmp = (i + istart) + 1;
      alpha1 = T[alpha1_tmp];
      c = i + 3;
      iaii = muIntScalarMin_sint32(c, 4) + istart;
      c_st.site = &sc_emlrtRSI;
      tau[i] = 0.0;
      d_st.site = &kb_emlrtRSI;
      xnorm = xnrm2(&d_st, 2 - i, T, iaii);
      if (xnorm != 0.0) {
        real_T beta1;
        beta1 = muDoubleScalarHypot(alpha1, xnorm);
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        if (muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          c = (iaii - i) + 1;
          ia = ((((c - iaii) + 1) / 2) << 1) + iaii;
          istart = ia - 2;
          do {
            knt++;
            d_st.site = &lb_emlrtRSI;
            e_st.site = &sb_emlrtRSI;
            f_st.site = &tb_emlrtRSI;
            for (k = iaii; k <= istart; k += 2) {
              r = _mm_loadu_pd(&T[k - 1]);
              _mm_storeu_pd(&T[k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (k = ia; k <= c; k++) {
              T[k - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            alpha1 *= 9.9792015476736E+291;
          } while ((muDoubleScalarAbs(beta1) < 1.0020841800044864E-292) &&
                   (knt < 20));
          d_st.site = &mb_emlrtRSI;
          xnorm = xnrm2(&d_st, 2 - i, T, iaii);
          beta1 = muDoubleScalarHypot(alpha1, xnorm);
          if (alpha1 >= 0.0) {
            beta1 = -beta1;
          }
          tau[i] = (beta1 - alpha1) / beta1;
          xnorm = 1.0 / (alpha1 - beta1);
          d_st.site = &nb_emlrtRSI;
          e_st.site = &sb_emlrtRSI;
          f_st.site = &tb_emlrtRSI;
          istart = ia - 2;
          for (k = iaii; k <= istart; k += 2) {
            r = _mm_loadu_pd(&T[k - 1]);
            _mm_storeu_pd(&T[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
          }
          for (k = ia; k <= c; k++) {
            T[k - 1] *= xnorm;
          }
          d_st.site = &ob_emlrtRSI;
          for (k = 0; k < knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }
          alpha1 = beta1;
        } else {
          tau[i] = (beta1 - alpha1) / beta1;
          xnorm = 1.0 / (alpha1 - beta1);
          d_st.site = &pb_emlrtRSI;
          e_st.site = &sb_emlrtRSI;
          istart = (iaii - i) + 1;
          f_st.site = &tb_emlrtRSI;
          c = ((((istart - iaii) + 1) / 2) << 1) + iaii;
          ia = c - 2;
          for (k = iaii; k <= ia; k += 2) {
            r = _mm_loadu_pd(&T[k - 1]);
            _mm_storeu_pd(&T[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
          }
          for (k = c; k <= istart; k++) {
            T[k - 1] *= xnorm;
          }
          alpha1 = beta1;
        }
      }
      T[alpha1_tmp] = 1.0;
      iaii = in + 1;
      c_st.site = &tc_emlrtRSI;
      if (tau[i] != 0.0) {
        boolean_T exitg2;
        lastv = 2 - i;
        istart = (alpha1_tmp - i) + 2;
        while ((lastv + 1 > 0) && (T[istart] == 0.0)) {
          lastv--;
          istart--;
        }
        lastc = 4;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int32_T exitg1;
          istart = in + lastc;
          c = istart;
          do {
            exitg1 = 0;
            if (c <= istart + (lastv << 2)) {
              if (T[c - 1] != 0.0) {
                exitg1 = 1;
              } else {
                c += 4;
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
        d_st.site = &vc_emlrtRSI;
        e_st.site = &cc_emlrtRSI;
        if (lastc != 0) {
          f_st.site = &ec_emlrtRSI;
          memset(&work[0], 0, (uint32_T)lastc * sizeof(real_T));
          istart = alpha1_tmp;
          c = (in + (lastv << 2)) + 1;
          for (k = iaii; k <= c; k += 4) {
            ia = k + lastc;
            f_st.site = &xc_emlrtRSI;
            for (b_i = k; b_i < ia; b_i++) {
              work_tmp = b_i - k;
              work[work_tmp] += T[b_i - 1] * T[istart];
            }
            istart++;
          }
        }
        d_st.site = &wc_emlrtRSI;
        e_st.site = &fc_emlrtRSI;
        f_st.site = &gc_emlrtRSI;
        g_st.site = &hc_emlrtRSI;
        if (!(-tau[i] == 0.0)) {
          istart = in;
          h_st.site = &ic_emlrtRSI;
          for (k = 0; k <= lastv; k++) {
            xnorm = T[alpha1_tmp + k];
            if (xnorm != 0.0) {
              xnorm *= -tau[i];
              c = istart + 1;
              ia = lastc + istart;
              h_st.site = &jc_emlrtRSI;
              if ((istart + 1 <= ia) && (ia > 2147483646)) {
                i_st.site = &v_emlrtRSI;
                check_forloop_overflow_error(&i_st);
              }
              iaii = ((((ia - istart) / 2) << 1) + istart) + 1;
              knt = iaii - 2;
              for (b_i = c; b_i <= knt; b_i += 2) {
                __m128d r1;
                r = _mm_loadu_pd(&work[(b_i - istart) - 1]);
                r1 = _mm_loadu_pd(&T[b_i - 1]);
                _mm_storeu_pd(
                    &T[b_i - 1],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xnorm))));
              }
              for (b_i = iaii; b_i <= ia; b_i++) {
                T[b_i - 1] += work[(b_i - istart) - 1] * xnorm;
              }
            }
            istart += 4;
          }
        }
      }
      c_st.site = &uc_emlrtRSI;
      xzlarf(&c_st, 3 - i, 3 - i, alpha1_tmp + 1, tau[i], T, (i + in) + 2,
             work);
      T[alpha1_tmp] = alpha1;
    }
    st.site = &oc_emlrtRSI;
    memcpy(&V[0], &T[0], 16U * sizeof(real_T));
    b_st.site = &yc_emlrtRSI;
    for (b_i = 2; b_i >= 0; b_i--) {
      ia = (b_i + 1) << 2;
      c_st.site = &ad_emlrtRSI;
      for (k = 0; k <= b_i; k++) {
        V[ia + k] = 0.0;
      }
      istart = b_i + 3;
      for (k = istart; k < 5; k++) {
        c = ia + k;
        V[c - 1] = V[c - 5];
      }
    }
    V[1] = 0.0;
    V[2] = 0.0;
    V[3] = 0.0;
    V[0] = 1.0;
    c_st.site = &bd_emlrtRSI;
    istart = 2;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (b_i = 2; b_i >= 0; b_i--) {
      iaii = (b_i + (b_i << 2)) + 5;
      if (b_i + 1 < 3) {
        V[iaii] = 1.0;
        d_st.site = &ub_emlrtRSI;
        xzlarf(&d_st, 3 - b_i, 2 - b_i, iaii + 1, tau[istart], V, iaii + 5,
               work);
        c = iaii + 2;
        d_st.site = &vb_emlrtRSI;
        e_st.site = &sb_emlrtRSI;
        ia = (iaii - b_i) + 3;
        f_st.site = &tb_emlrtRSI;
        knt = (((((ia - iaii) - 1) / 2) << 1) + iaii) + 2;
        work_tmp = knt - 2;
        for (k = c; k <= work_tmp; k += 2) {
          r = _mm_loadu_pd(&V[k - 1]);
          _mm_storeu_pd(&V[k - 1], _mm_mul_pd(_mm_set1_pd(-tau[istart]), r));
        }
        for (k = knt; k <= ia; k++) {
          V[k - 1] *= -tau[istart];
        }
      }
      V[iaii] = 1.0 - tau[istart];
      d_st.site = &wb_emlrtRSI;
      for (k = 0; k < b_i; k++) {
        V[(iaii - k) - 1] = 0.0;
      }
      istart = b_i - 1;
    }
    st.site = &pc_emlrtRSI;
    istart = xhseqr(&st, T, V);
    if ((istart != 0) && (!emlrtSetWarningFlag((emlrtCTX)sp))) {
      st.site = &qc_emlrtRSI;
      b_warning(&st);
    }
  }
}

/* End of code generation (schur.c) */
