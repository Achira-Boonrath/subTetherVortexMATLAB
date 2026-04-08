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
#include "xnrm2.h"
#include "xrot.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo cd_emlrtRSI = {
    21,       /* lineNo */
    "xhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xhseqr.m" /* pathName */
};

static emlrtRSInfo dd_emlrtRSI = {
    16,        /* lineNo */
    "xdhseqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdhseqr.m" /* pathName */
};

static emlrtRSInfo ld_emlrtRSI = {
    342,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

static emlrtRSInfo md_emlrtRSI = {
    345,       /* lineNo */
    "xdlahqr", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2025b\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xdlahqr.m" /* pathName */
};

/* Function Definitions */
int32_T xhseqr(const emlrtStack *sp, real_T h[16], real_T z[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T v[3];
  real_T h12;
  real_T h21;
  real_T h21s;
  real_T h22;
  real_T rt1r;
  real_T rt2r;
  real_T t3;
  real_T temp;
  int32_T b_i;
  int32_T i;
  int32_T info;
  int32_T j;
  int32_T kdefl;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &cd_emlrtRSI;
  b_st.site = &dd_emlrtRSI;
  info = 0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  kdefl = 0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int32_T its;
    int32_T knt;
    int32_T l;
    int32_T s_tmp;
    int32_T scalarLB;
    int32_T v_tmp;
    boolean_T converged;
    boolean_T exitg2;
    l = 1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      real_T s;
      int32_T k;
      boolean_T exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        s_tmp = k + ((k - 1) << 2);
        h21s = muDoubleScalarAbs(h[s_tmp]);
        if (h21s <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          v_tmp = k + (k << 2);
          h12 = muDoubleScalarAbs(h[v_tmp]);
          temp = muDoubleScalarAbs(h[s_tmp - 1]) + h12;
          if (temp == 0.0) {
            if (k - 1 >= 1) {
              temp = muDoubleScalarAbs(h[(k + ((k - 2) << 2)) - 1]);
            }
            if (k + 2 <= 4) {
              temp += muDoubleScalarAbs(h[v_tmp + 1]);
            }
          }
          if (h21s <= 2.2204460492503131E-16 * temp) {
            h21 = muDoubleScalarAbs(h[v_tmp - 1]);
            temp = muDoubleScalarAbs(h[s_tmp - 1] - h[v_tmp]);
            t3 = muDoubleScalarMax(h12, temp);
            temp = muDoubleScalarMin(h12, temp);
            s = t3 + temp;
            if (muDoubleScalarMin(h21s, h21) *
                    (muDoubleScalarMax(h21s, h21) / s) <=
                muDoubleScalarMax(4.0083367200179456E-292,
                                  2.2204460492503131E-16 * (temp * (t3 / s)))) {
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
        int32_T m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = muDoubleScalarAbs(h[i + ((i - 1) << 2)]) +
              muDoubleScalarAbs(h[(i + ((i - 2) << 2)) - 1]);
          temp = 0.75 * s + h[i + (i << 2)];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = temp;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          s_tmp = k + (k << 2);
          s = muDoubleScalarAbs(h[s_tmp + 1]) +
              muDoubleScalarAbs(h[(k + ((k + 1) << 2)) + 2]);
          temp = 0.75 * s + h[s_tmp];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = temp;
        } else {
          v_tmp = i + ((i - 1) << 2);
          temp = h[v_tmp - 1];
          h21 = h[v_tmp];
          s_tmp = i + (i << 2);
          h12 = h[s_tmp - 1];
          h22 = h[s_tmp];
        }
        s = ((muDoubleScalarAbs(temp) + muDoubleScalarAbs(h12)) +
             muDoubleScalarAbs(h21)) +
            muDoubleScalarAbs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          temp = 0.0;
          rt2r = 0.0;
          h12 = 0.0;
        } else {
          temp /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          t3 = (temp + h22) / 2.0;
          temp = (temp - t3) * (h22 - t3) - h12 * h21;
          h12 = muDoubleScalarSqrt(muDoubleScalarAbs(temp));
          if (temp >= 0.0) {
            rt1r = t3 * s;
            rt2r = rt1r;
            temp = h12 * s;
            h12 = -temp;
          } else {
            rt1r = t3 + h12;
            rt2r = t3 - h12;
            if (muDoubleScalarAbs(rt1r - h22) <=
                muDoubleScalarAbs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            temp = 0.0;
            h12 = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          s_tmp = m + ((m - 1) << 2);
          h21 = h[s_tmp - 1];
          t3 = h21 - rt2r;
          s = (muDoubleScalarAbs(t3) + muDoubleScalarAbs(h12)) +
              muDoubleScalarAbs(h[s_tmp]);
          h21s = h[s_tmp] / s;
          v_tmp = m + (m << 2);
          v[0] = (h21s * h[v_tmp - 1] + t3 * (t3 / s)) - temp * (h12 / s);
          v[1] = h21s * (((h21 + h[v_tmp]) - rt1r) - rt2r);
          v[2] = h21s * h[v_tmp + 1];
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
                     muDoubleScalarAbs(h[s_tmp - 1])) +
                    muDoubleScalarAbs(h[v_tmp])))) {
            exitg3 = true;
          } else {
            m--;
          }
        }
        for (b_i = m; b_i <= i; b_i++) {
          int32_T nr;
          knt = (i - b_i) + 2;
          nr = muIntScalarMin_sint32(3, knt);
          if (b_i > m) {
            knt = (((b_i - 2) << 2) + b_i) - 1;
            for (j = 0; j < nr; j++) {
              v[j] = h[knt + j];
            }
          }
          h12 = v[0];
          s = 0.0;
          if (nr > 0) {
            temp = b_xnrm2(nr - 1, v);
            if (temp != 0.0) {
              h21 = muDoubleScalarHypot(v[0], temp);
              if (v[0] >= 0.0) {
                h21 = -h21;
              }
              if (muDoubleScalarAbs(h21) < 1.0020841800044864E-292) {
                knt = 0;
                v_tmp = (((nr - 1) / 2) << 1) + 2;
                s_tmp = v_tmp - 2;
                do {
                  knt++;
                  for (j = 2; j <= s_tmp; j += 2) {
                    r = _mm_loadu_pd(&v[1]);
                    _mm_storeu_pd(
                        &v[1],
                        _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
                  }
                  for (j = v_tmp; j <= nr; j++) {
                    v[j - 1] *= 9.9792015476736E+291;
                  }
                  h21 *= 9.9792015476736E+291;
                  h12 *= 9.9792015476736E+291;
                } while ((muDoubleScalarAbs(h21) < 1.0020841800044864E-292) &&
                         (knt < 20));
                h21 = muDoubleScalarHypot(h12, b_xnrm2(nr - 1, v));
                if (h12 >= 0.0) {
                  h21 = -h21;
                }
                s = (h21 - h12) / h21;
                temp = 1.0 / (h12 - h21);
                s_tmp = v_tmp - 2;
                for (j = 2; j <= s_tmp; j += 2) {
                  r = _mm_loadu_pd(&v[1]);
                  _mm_storeu_pd(&v[1], _mm_mul_pd(_mm_set1_pd(temp), r));
                }
                for (j = v_tmp; j <= nr; j++) {
                  v[j - 1] *= temp;
                }
                for (j = 0; j < knt; j++) {
                  h21 *= 1.0020841800044864E-292;
                }
                h12 = h21;
              } else {
                s = (h21 - v[0]) / h21;
                temp = 1.0 / (v[0] - h21);
                scalarLB = (((nr - 1) / 2) << 1) + 2;
                s_tmp = scalarLB - 2;
                for (j = 2; j <= s_tmp; j += 2) {
                  r = _mm_loadu_pd(&v[1]);
                  _mm_storeu_pd(&v[1], _mm_mul_pd(_mm_set1_pd(temp), r));
                }
                for (j = scalarLB; j <= nr; j++) {
                  v[j - 1] *= temp;
                }
                h12 = h21;
              }
            }
          }
          if (b_i > m) {
            knt = b_i + ((b_i - 2) << 2);
            h[knt - 1] = h12;
            h[knt] = 0.0;
            if (b_i < i) {
              h[b_i + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[b_i - 1] *= 1.0 - s;
          }
          h22 = v[1];
          rt2r = s * v[1];
          if (nr == 3) {
            __m128d r1;
            __m128d r2;
            __m128d r3;
            int32_T b_scalarLB;
            h21s = v[2];
            t3 = s * v[2];
            for (j = b_i; j < 5; j++) {
              knt = b_i + ((j - 1) << 2);
              temp = h[knt - 1];
              h12 = h[knt];
              h21 = h[knt + 1];
              rt1r = (temp + h22 * h12) + h21s * h21;
              temp -= rt1r * s;
              h[knt - 1] = temp;
              h12 -= rt1r * rt2r;
              h[knt] = h12;
              h21 -= rt1r * t3;
              h[knt + 1] = h21;
            }
            knt = b_i + 3;
            scalarLB = i + 1;
            nr = muIntScalarMin_sint32(knt, scalarLB);
            b_scalarLB = (nr / 2) << 1;
            s_tmp = b_scalarLB - 2;
            for (j = 0; j <= s_tmp; j += 2) {
              v_tmp = j + (b_i << 2);
              r = _mm_loadu_pd(&h[v_tmp]);
              knt = j + ((b_i + 1) << 2);
              r1 = _mm_loadu_pd(&h[knt]);
              scalarLB = j + ((b_i - 1) << 2);
              r2 = _mm_loadu_pd(&h[scalarLB]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(h22), r)),
                              _mm_mul_pd(_mm_set1_pd(h21s), r1));
              _mm_storeu_pd(&h[scalarLB],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[v_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt2r))));
              _mm_storeu_pd(&h[knt],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(t3))));
            }
            for (j = b_scalarLB; j < nr; j++) {
              s_tmp = j + ((b_i - 1) << 2);
              temp = h[s_tmp];
              v_tmp = j + (b_i << 2);
              h12 = h[v_tmp];
              knt = j + ((b_i + 1) << 2);
              h21 = h[knt];
              rt1r = (temp + h22 * h12) + h21s * h21;
              temp -= rt1r * s;
              h[s_tmp] = temp;
              h12 -= rt1r * rt2r;
              h[v_tmp] = h12;
              h21 -= rt1r * t3;
              h[knt] = h21;
            }
            __m128d r4;
            __m128d r5;
            __m128d r6;
            __m128d r7;
            __m128d r8;
            scalarLB = b_i << 2;
            r = _mm_loadu_pd(&z[scalarLB]);
            s_tmp = (b_i + 1) << 2;
            r1 = _mm_loadu_pd(&z[s_tmp]);
            v_tmp = (b_i - 1) << 2;
            r2 = _mm_loadu_pd(&z[v_tmp]);
            r3 = _mm_set1_pd(v[1]);
            r4 = _mm_set1_pd(v[2]);
            r5 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r3, r)),
                            _mm_mul_pd(r4, r1));
            r6 = _mm_set1_pd(s);
            _mm_storeu_pd(&z[v_tmp], _mm_sub_pd(r2, _mm_mul_pd(r5, r6)));
            r7 = _mm_set1_pd(rt2r);
            _mm_storeu_pd(&z[scalarLB], _mm_sub_pd(r, _mm_mul_pd(r5, r7)));
            r8 = _mm_set1_pd(t3);
            _mm_storeu_pd(&z[s_tmp], _mm_sub_pd(r1, _mm_mul_pd(r5, r8)));
            r = _mm_loadu_pd(&z[scalarLB + 2]);
            r1 = _mm_loadu_pd(&z[s_tmp + 2]);
            r2 = _mm_loadu_pd(&z[v_tmp + 2]);
            r5 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r3, r)),
                            _mm_mul_pd(r4, r1));
            _mm_storeu_pd(&z[v_tmp + 2], _mm_sub_pd(r2, _mm_mul_pd(r5, r6)));
            _mm_storeu_pd(&z[scalarLB + 2], _mm_sub_pd(r, _mm_mul_pd(r5, r7)));
            _mm_storeu_pd(&z[s_tmp + 2], _mm_sub_pd(r1, _mm_mul_pd(r5, r8)));
          } else if (nr == 2) {
            __m128d r1;
            __m128d r2;
            for (j = b_i; j < 5; j++) {
              knt = b_i + ((j - 1) << 2);
              temp = h[knt - 1];
              h12 = h[knt];
              rt1r = temp + h22 * h12;
              temp -= rt1r * s;
              h[knt - 1] = temp;
              h12 -= rt1r * rt2r;
              h[knt] = h12;
            }
            nr = ((i + 1) / 2) << 1;
            s_tmp = nr - 2;
            for (j = 0; j <= s_tmp; j += 2) {
              v_tmp = j + (b_i << 2);
              r = _mm_loadu_pd(&h[v_tmp]);
              knt = j + ((b_i - 1) << 2);
              r1 = _mm_loadu_pd(&h[knt]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(h22), r));
              _mm_storeu_pd(&h[knt],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(s))));
              _mm_storeu_pd(&h[v_tmp],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt2r))));
            }
            for (j = nr; j <= i; j++) {
              scalarLB = j + ((b_i - 1) << 2);
              temp = h[scalarLB];
              s_tmp = j + (b_i << 2);
              h12 = h[s_tmp];
              rt1r = temp + h22 * h12;
              temp -= rt1r * s;
              h[scalarLB] = temp;
              h12 -= rt1r * rt2r;
              h[s_tmp] = h12;
            }
            __m128d r3;
            __m128d r4;
            __m128d r5;
            knt = b_i << 2;
            r = _mm_loadu_pd(&z[knt]);
            scalarLB = (b_i - 1) << 2;
            r1 = _mm_loadu_pd(&z[scalarLB]);
            r2 = _mm_set1_pd(v[1]);
            r3 = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            r4 = _mm_set1_pd(s);
            _mm_storeu_pd(&z[scalarLB], _mm_sub_pd(r1, _mm_mul_pd(r3, r4)));
            r5 = _mm_set1_pd(rt2r);
            _mm_storeu_pd(&z[knt], _mm_sub_pd(r, _mm_mul_pd(r3, r5)));
            r = _mm_loadu_pd(&z[knt + 2]);
            r1 = _mm_loadu_pd(&z[scalarLB + 2]);
            r3 = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&z[scalarLB + 2], _mm_sub_pd(r1, _mm_mul_pd(r3, r4)));
            _mm_storeu_pd(&z[knt + 2], _mm_sub_pd(r, _mm_mul_pd(r3, r5)));
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
        knt = i << 2;
        s_tmp = i + knt;
        temp = h[s_tmp - 1];
        scalarLB = (i - 1) << 2;
        v_tmp = i + scalarLB;
        h12 = h[v_tmp];
        h21 = h[s_tmp];
        xdlanv2(&h[v_tmp - 1], &temp, &h12, &h21, &t3, &h21s, &h22, &rt2r,
                &rt1r);
        h[s_tmp - 1] = temp;
        h[v_tmp] = h12;
        h[s_tmp] = h21;
        if (i + 1 < 4) {
          s_tmp = ((i + 1) << 2) + i;
          c_st.site = &ld_emlrtRSI;
          xrot(&c_st, 3 - i, h, s_tmp, s_tmp + 1, rt2r, rt1r);
        }
        c_st.site = &md_emlrtRSI;
        b_xrot(&c_st, i - 1, h, scalarLB + 1, knt + 1, rt2r, rt1r);
        temp = rt2r * z[scalarLB] + rt1r * z[knt];
        z[knt] = rt2r * z[knt] - rt1r * z[scalarLB];
        z[scalarLB] = temp;
        temp = z[knt + 1];
        h12 = z[scalarLB + 1];
        z[knt + 1] = rt2r * temp - rt1r * h12;
        z[scalarLB + 1] = rt2r * h12 + rt1r * temp;
        temp = z[knt + 2];
        h12 = z[scalarLB + 2];
        z[knt + 2] = rt2r * temp - rt1r * h12;
        z[scalarLB + 2] = rt2r * h12 + rt1r * temp;
        temp = z[knt + 3];
        h12 = z[scalarLB + 3];
        z[knt + 3] = rt2r * temp - rt1r * h12;
        z[scalarLB + 3] = rt2r * h12 + rt1r * temp;
      }
      kdefl = 0;
      i = l - 2;
    }
  }
  for (j = 0; j < 2; j++) {
    for (b_i = j + 3; b_i < 5; b_i++) {
      h[(b_i + (j << 2)) - 1] = 0.0;
    }
  }
  return info;
}

/* End of code generation (xhseqr.c) */
