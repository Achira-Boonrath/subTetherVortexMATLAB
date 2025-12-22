/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dot.c
 *
 * Code generation for function 'dot'
 *
 */

/* Include files */
#include "dot.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>

/* Function Definitions */
real_T dot(const real_T a[3], const real_T b[3])
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  n_t = (ptrdiff_t)3;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  return ddot(&n_t, (real_T *)&a[0], &incx_t, (real_T *)&b[0], &incy_t);
}

/* End of code generation (dot.c) */
