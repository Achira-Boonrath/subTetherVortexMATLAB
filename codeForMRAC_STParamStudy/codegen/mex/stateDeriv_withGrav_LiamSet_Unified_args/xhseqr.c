/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xhseqr.c
 *
 * Code generation for function 'xhseqr'
 *
 */

/* Include files */
#include "xhseqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xrot.h"
#include "xzlarfg.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo td_emlrtRSI = {
    21,       /* lineNo */
    "xhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xhseqr.m" /* pathName */
};

static emlrtRSInfo ud_emlrtRSI = {
    16,        /* lineNo */
    "xdhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdhseqr.m" /* pathName */
};

static emlrtRSInfo de_emlrtRSI = {
    342,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

/* Function Definitions */
int32_T xhseqr(const emlrtStack *sp, real_T h[16], real_T z[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v[3];
  real_T aa;
  real_T d;
  real_T h12;
  real_T h21;
  real_T h22;
  real_T rt2r;
  real_T s;
  real_T tst;
  int32_T b_i;
  int32_T b_k;
  int32_T c_k;
  int32_T i;
  int32_T info;
  int32_T k;
  int32_T kdefl;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &td_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &ud_emlrtRSI;
  info = 0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  kdefl = 0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int32_T i1;
    int32_T i2;
    int32_T i3;
    int32_T its;
    int32_T l;
    int32_T nr;
    int32_T tst_tmp_tmp;
    boolean_T converged;
    boolean_T exitg2;
    l = 1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      boolean_T exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        i1 = k + ((k - 1) << 2);
        d = muDoubleScalarAbs(h[i1]);
        if (d <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          tst_tmp_tmp = k + (k << 2);
          h21 = muDoubleScalarAbs(h[tst_tmp_tmp]);
          tst = muDoubleScalarAbs(h[i1 - 1]) + h21;
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = muDoubleScalarAbs(h[(k + ((k - 2) << 2)) - 1]);
            }
            if (k + 2 <= 4) {
              tst += muDoubleScalarAbs(h[tst_tmp_tmp + 1]);
            }
          }
          if (d <= 2.2204460492503131E-16 * tst) {
            h12 = muDoubleScalarAbs(h[tst_tmp_tmp - 1]);
            tst = muDoubleScalarAbs(h[i1 - 1] - h[tst_tmp_tmp]);
            aa = muDoubleScalarMax(h21, tst);
            tst = muDoubleScalarMin(h21, tst);
            s = aa + tst;
            if (muDoubleScalarMin(d, h12) * (muDoubleScalarMax(d, h12) / s) <=
                muDoubleScalarMax(4.0083367200179456E-292,
                                  2.2204460492503131E-16 * (tst * (aa / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      l = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }
      if (k + 1 >= i) {
        converged = true;
        exitg2 = true;
      } else {
        __m128d r;
        real_T rt1r;
        int32_T m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = muDoubleScalarAbs(h[i + ((i - 1) << 2)]) +
              muDoubleScalarAbs(h[(i + ((i - 2) << 2)) - 1]);
          tst = 0.75 * s + h[i + (i << 2)];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = tst;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          tst_tmp_tmp = k + (k << 2);
          s = muDoubleScalarAbs(h[tst_tmp_tmp + 1]) +
              muDoubleScalarAbs(h[(k + ((k + 1) << 2)) + 2]);
          tst = 0.75 * s + h[tst_tmp_tmp];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = tst;
        } else {
          tst_tmp_tmp = i + ((i - 1) << 2);
          tst = h[tst_tmp_tmp - 1];
          h21 = h[tst_tmp_tmp];
          tst_tmp_tmp = i + (i << 2);
          h12 = h[tst_tmp_tmp - 1];
          h22 = h[tst_tmp_tmp];
        }
        s = ((muDoubleScalarAbs(tst) + muDoubleScalarAbs(h12)) +
             muDoubleScalarAbs(h21)) +
            muDoubleScalarAbs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          tst = 0.0;
          rt2r = 0.0;
          h21 = 0.0;
        } else {
          tst /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          aa = (tst + h22) / 2.0;
          tst = (tst - aa) * (h22 - aa) - h12 * h21;
          h21 = muDoubleScalarSqrt(muDoubleScalarAbs(tst));
          if (tst >= 0.0) {
            rt1r = aa * s;
            rt2r = rt1r;
            tst = h21 * s;
            h21 = -tst;
          } else {
            rt1r = aa + h21;
            rt2r = aa - h21;
            if (muDoubleScalarAbs(rt1r - h22) <=
                muDoubleScalarAbs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            tst = 0.0;
            h21 = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          tst_tmp_tmp = m + ((m - 1) << 2);
          h12 = h[tst_tmp_tmp - 1];
          aa = h12 - rt2r;
          s = (muDoubleScalarAbs(aa) + muDoubleScalarAbs(h21)) +
              muDoubleScalarAbs(h[tst_tmp_tmp]);
          h22 = h[tst_tmp_tmp] / s;
          nr = m + (m << 2);
          v[0] = (h22 * h[nr - 1] + aa * (aa / s)) - tst * (h21 / s);
          v[1] = h22 * (((h12 + h[nr]) - rt1r) - rt2r);
          v[2] = h22 * h[nr + 1];
          s = (muDoubleScalarAbs(v[0]) + muDoubleScalarAbs(v[1])) +
              muDoubleScalarAbs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
          if ((m == k + 1) ||
              (muDoubleScalarAbs(h[m - 1]) *
                   (muDoubleScalarAbs(v[1]) + muDoubleScalarAbs(v[2])) <=
               2.2204460492503131E-16 * muDoubleScalarAbs(v[0]) *
                   ((muDoubleScalarAbs(h[0]) +
                     muDoubleScalarAbs(h[tst_tmp_tmp - 1])) +
                    muDoubleScalarAbs(h[nr])))) {
            exitg3 = true;
          } else {
            m--;
          }
        }
        for (c_k = m; c_k <= i; c_k++) {
          i1 = (i - c_k) + 2;
          nr = muIntScalarMin_sint32(3, i1);
          if (c_k > m) {
            tst_tmp_tmp = (((c_k - 2) << 2) + c_k) - 1;
            for (b_k = 0; b_k < nr; b_k++) {
              v[b_k] = h[tst_tmp_tmp + b_k];
            }
          }
          tst = v[0];
          aa = xzlarfg(nr, &tst, v);
          if (c_k > m) {
            i1 = c_k + ((c_k - 2) << 2);
            h[i1 - 1] = tst;
            h[i1] = 0.0;
            if (c_k < i) {
              h[c_k + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[c_k - 1] *= 1.0 - aa;
          }
          d = v[1];
          tst = aa * v[1];
          if (nr == 3) {
            __m128d r1;
            __m128d r2;
            __m128d r3;
            h22 = v[2];
            h12 = aa * v[2];
            for (b_i = c_k; b_i < 5; b_i++) {
              i1 = c_k + ((b_i - 1) << 2);
              rt2r = h[i1 - 1];
              rt1r = h[i1];
              s = h[i1 + 1];
              h21 = (rt2r + d * rt1r) + h22 * s;
              rt2r -= h21 * aa;
              h[i1 - 1] = rt2r;
              rt1r -= h21 * tst;
              h[i1] = rt1r;
              s -= h21 * h12;
              h[i1 + 1] = s;
            }
            i1 = c_k + 3;
            b_i = i + 1;
            i1 = muIntScalarMin_sint32(i1, b_i);
            tst_tmp_tmp = (i1 / 2) << 1;
            nr = tst_tmp_tmp - 2;
            for (b_i = 0; b_i <= nr; b_i += 2) {
              i2 = b_i + (c_k << 2);
              r = _mm_loadu_pd(&h[i2]);
              i3 = b_i + ((c_k + 1) << 2);
              r1 = _mm_loadu_pd(&h[i3]);
              b_k = b_i + ((c_k - 1) << 2);
              r2 = _mm_loadu_pd(&h[b_k]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(d), r)),
                              _mm_mul_pd(_mm_set1_pd(h22), r1));
              _mm_storeu_pd(&h[b_k],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(aa))));
              _mm_storeu_pd(&h[i2],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(tst))));
              _mm_storeu_pd(&h[i3],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(h12))));
            }
            for (b_i = tst_tmp_tmp; b_i < i1; b_i++) {
              i2 = b_i + ((c_k - 1) << 2);
              rt2r = h[i2];
              i3 = b_i + (c_k << 2);
              rt1r = h[i3];
              b_k = b_i + ((c_k + 1) << 2);
              s = h[b_k];
              h21 = (rt2r + d * rt1r) + h22 * s;
              rt2r -= h21 * aa;
              h[i2] = rt2r;
              rt1r -= h21 * tst;
              h[i3] = rt1r;
              s -= h21 * h12;
              h[b_k] = s;
            }
            __m128d r4;
            __m128d r5;
            __m128d r6;
            __m128d r7;
            __m128d r8;
            i1 = c_k << 2;
            r = _mm_loadu_pd(&z[i1]);
            i2 = (c_k + 1) << 2;
            r1 = _mm_loadu_pd(&z[i2]);
            i3 = (c_k - 1) << 2;
            r2 = _mm_loadu_pd(&z[i3]);
            r4 = _mm_set1_pd(v[1]);
            r5 = _mm_set1_pd(v[2]);
            r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r4, r)),
                            _mm_mul_pd(r5, r1));
            r6 = _mm_set1_pd(aa);
            _mm_storeu_pd(&z[i3], _mm_sub_pd(r2, _mm_mul_pd(r3, r6)));
            r7 = _mm_set1_pd(tst);
            _mm_storeu_pd(&z[i1], _mm_sub_pd(r, _mm_mul_pd(r3, r7)));
            r8 = _mm_set1_pd(h12);
            _mm_storeu_pd(&z[i2], _mm_sub_pd(r1, _mm_mul_pd(r3, r8)));
            r = _mm_loadu_pd(&z[i1 + 2]);
            r1 = _mm_loadu_pd(&z[i2 + 2]);
            r2 = _mm_loadu_pd(&z[i3 + 2]);
            r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r4, r)),
                            _mm_mul_pd(r5, r1));
            _mm_storeu_pd(&z[i3 + 2], _mm_sub_pd(r2, _mm_mul_pd(r3, r6)));
            _mm_storeu_pd(&z[i1 + 2], _mm_sub_pd(r, _mm_mul_pd(r3, r7)));
            _mm_storeu_pd(&z[i2 + 2], _mm_sub_pd(r1, _mm_mul_pd(r3, r8)));
          } else if (nr == 2) {
            __m128d r1;
            __m128d r2;
            for (b_i = c_k; b_i < 5; b_i++) {
              i1 = c_k + ((b_i - 1) << 2);
              h22 = h[i1 - 1];
              rt2r = h[i1];
              h21 = h22 + d * rt2r;
              h22 -= h21 * aa;
              h[i1 - 1] = h22;
              rt2r -= h21 * tst;
              h[i1] = rt2r;
            }
            tst_tmp_tmp = ((i + 1) / 2) << 1;
            nr = tst_tmp_tmp - 2;
            for (b_i = 0; b_i <= nr; b_i += 2) {
              i1 = b_i + (c_k << 2);
              r = _mm_loadu_pd(&h[i1]);
              i2 = b_i + ((c_k - 1) << 2);
              r1 = _mm_loadu_pd(&h[i2]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(d), r));
              _mm_storeu_pd(&h[i2],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(aa))));
              _mm_storeu_pd(&h[i1],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(tst))));
            }
            for (b_i = tst_tmp_tmp; b_i <= i; b_i++) {
              i1 = b_i + ((c_k - 1) << 2);
              h22 = h[i1];
              i2 = b_i + (c_k << 2);
              rt2r = h[i2];
              h21 = h22 + d * rt2r;
              h22 -= h21 * aa;
              h[i1] = h22;
              rt2r -= h21 * tst;
              h[i2] = rt2r;
            }
            __m128d r3;
            __m128d r4;
            __m128d r5;
            i1 = c_k << 2;
            r = _mm_loadu_pd(&z[i1]);
            i2 = (c_k - 1) << 2;
            r1 = _mm_loadu_pd(&z[i2]);
            r3 = _mm_set1_pd(v[1]);
            r2 = _mm_add_pd(r1, _mm_mul_pd(r3, r));
            r4 = _mm_set1_pd(aa);
            _mm_storeu_pd(&z[i2], _mm_sub_pd(r1, _mm_mul_pd(r2, r4)));
            r5 = _mm_set1_pd(tst);
            _mm_storeu_pd(&z[i1], _mm_sub_pd(r, _mm_mul_pd(r2, r5)));
            r = _mm_loadu_pd(&z[i1 + 2]);
            r1 = _mm_loadu_pd(&z[i2 + 2]);
            r2 = _mm_add_pd(r1, _mm_mul_pd(r3, r));
            _mm_storeu_pd(&z[i2 + 2], _mm_sub_pd(r1, _mm_mul_pd(r2, r4)));
            _mm_storeu_pd(&z[i1 + 2], _mm_sub_pd(r, _mm_mul_pd(r2, r5)));
          }
        }
        its++;
      }
    }
    if (!converged) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((l != i + 1) && (l == i)) {
        i1 = i << 2;
        i2 = i + i1;
        d = h[i2 - 1];
        i3 = (i - 1) << 2;
        b_k = i + i3;
        h22 = h[b_k];
        rt2r = h[i2];
        xdlanv2(&h[b_k - 1], &d, &h22, &rt2r, &s, &tst, &h21, &h12, &aa);
        h[i2 - 1] = d;
        h[b_k] = h22;
        h[i2] = rt2r;
        if (i + 1 < 4) {
          i2 = ((i + 1) << 2) + i;
          c_st.site = &de_emlrtRSI;
          xrot(&c_st, 3 - i, h, i2, i2 + 1, h12, aa);
        }
        if (i - 1 >= 1) {
          i2 = (uint8_T)(i - 1);
          for (k = 0; k < i2; k++) {
            tst_tmp_tmp = i1 + k;
            tst = h[tst_tmp_tmp];
            nr = i3 + k;
            h21 = h[nr];
            h[tst_tmp_tmp] = h12 * tst - aa * h21;
            h[nr] = h12 * h21 + aa * tst;
          }
        }
        tst = h12 * z[i3] + aa * z[i1];
        z[i1] = h12 * z[i1] - aa * z[i3];
        z[i3] = tst;
        tst = z[i1 + 1];
        h21 = z[i3 + 1];
        z[i1 + 1] = h12 * tst - aa * h21;
        z[i3 + 1] = h12 * h21 + aa * tst;
        tst = z[i1 + 2];
        h21 = z[i3 + 2];
        z[i1 + 2] = h12 * tst - aa * h21;
        z[i3 + 2] = h12 * h21 + aa * tst;
        tst = z[i1 + 3];
        h21 = z[i3 + 3];
        z[i1 + 3] = h12 * tst - aa * h21;
        z[i3 + 3] = h12 * h21 + aa * tst;
      }
      kdefl = 0;
      i = l - 2;
    }
  }
  for (b_i = 0; b_i < 2; b_i++) {
    for (i = b_i + 3; i < 5; i++) {
      h[(i + (b_i << 2)) - 1] = 0.0;
    }
  }
  return info;
}

/* End of code generation (xhseqr.c) */
