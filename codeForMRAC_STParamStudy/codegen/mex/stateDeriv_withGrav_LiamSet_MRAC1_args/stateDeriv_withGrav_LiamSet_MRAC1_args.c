/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateDeriv_withGrav_LiamSet_MRAC1_args.c
 *
 * Code generation for function 'stateDeriv_withGrav_LiamSet_MRAC1_args'
 *
 */

/* Include files */
#include "stateDeriv_withGrav_LiamSet_MRAC1_args.h"
#include "mldivide.h"
#include "norm.h"
#include "rotm2quat.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_data.h"
#include "stateDeriv_withGrav_LiamSet_MRAC1_args_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include "omp.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    239,                                      /* lineNo */
    "stateDeriv_withGrav_LiamSet_MRAC1_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_MRAC1_args.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    238,                                      /* lineNo */
    "stateDeriv_withGrav_LiamSet_MRAC1_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_MRAC1_args.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    169,                                      /* lineNo */
    "stateDeriv_withGrav_LiamSet_MRAC1_args", /* fcnName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
    "withGrav_LiamSet_MRAC1_args.m" /* pathName */
};

/* Function Definitions */
emlrtCTX emlrtGetRootTLSGlobal(void)
{
  return emlrtRootTLSGlobal;
}

void emlrtLockerFunction(EmlrtLockeeFunction aLockee, emlrtConstCTX aTLS,
                         void *aData)
{
  omp_set_lock(&emlrtLockGlobal);
  emlrtCallLockeeFunction(aLockee, aTLS, aData);
  omp_unset_lock(&emlrtLockGlobal);
}

void stateDeriv_withGrav_LiamSet_MRAC1_args(const emlrtStack *sp, real_T t,
                                            real_T s[39], const struct0_T *args,
                                            real_T ds[39])
{
  static const int8_T b_a[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const int8_T iv[3] = {0, 0, -1};
  __m128d r;
  __m128d r1;
  __m128d r2;
  __m128d r3;
  __m128d r5;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack st;
  real_T Qmat_desCh[12];
  real_T Tvec_st[12];
  real_T b[12];
  real_T q_eCh_tmp[12];
  real_T rotMat_C_A_I[9];
  real_T rotMat_D_A_I[9];
  real_T y_tmp[9];
  real_T quar_CDot[4];
  real_T quar_TDot[4];
  real_T quat_desCh[4];
  real_T Tvec_mt[3];
  real_T VR_mt[3];
  real_T accChaser[3];
  real_T accPoint1[3];
  real_T appTorqueTarget[3];
  real_T b_b[3];
  real_T b_distAttPt_to_D[3];
  real_T distAttPt_to_C[3];
  real_T distAttPt_to_D[3];
  real_T l_mt_vec[3];
  real_T slidingVec_tmp[3];
  real_T tau_Att_Ch_tmp[3];
  real_T dv[2];
  real_T Qmat_desCh_tmp;
  real_T Tmag_mt;
  real_T a;
  real_T b_rotMat_C_A_I_tmp;
  real_T l_mt;
  real_T rotMat_C_A_I_tmp;
  real_T targetSideLengthY;
  real_T x1;
  real_T x1dot;
  real_T x_rDdot;
  int32_T b_i;
  int32_T i;
  int32_T maskMTreal;
  boolean_T maskFT;
  st.prev = sp;
  st.tls = sp->tls;
  /*  Achira Boonrath, MAE 566 Project */
  targetSideLengthY = args->targetSideLengthY;
  /* Extract States */
  r = _mm_loadu_pd(&s[0]);
  r1 = _mm_set1_pd(args->ODEscale);
  _mm_storeu_pd(&s[0], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[13]);
  _mm_storeu_pd(&s[13], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[26]);
  _mm_storeu_pd(&s[26], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[2]);
  _mm_storeu_pd(&s[2], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[15]);
  _mm_storeu_pd(&s[15], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[28]);
  _mm_storeu_pd(&s[28], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[4]);
  _mm_storeu_pd(&s[4], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[17]);
  _mm_storeu_pd(&s[17], _mm_mul_pd(r, r1));
  r = _mm_loadu_pd(&s[30]);
  _mm_storeu_pd(&s[30], _mm_mul_pd(r, r1));
  /* From Field 2022 Thesis */
  b[0] = -s[7];
  b[4] = -s[8];
  b[8] = -s[9];
  b[1] = s[6];
  b[5] = -s[9];
  b[9] = s[8];
  b[2] = s[9];
  b[6] = s[6];
  b[10] = -s[7];
  b[3] = -s[8];
  b[7] = s[7];
  b[11] = s[6];
  memset(&quar_CDot[0], 0, sizeof(real_T) << 2);
  Tmag_mt = 0.5 * -s[20];
  Qmat_desCh[0] = Tmag_mt;
  Qmat_desCh_tmp = 0.5 * -s[21];
  Qmat_desCh[4] = Qmat_desCh_tmp;
  x_rDdot = 0.5 * -s[22];
  Qmat_desCh[8] = x_rDdot;
  l_mt = 0.5 * s[19];
  Qmat_desCh[1] = l_mt;
  Qmat_desCh[5] = x_rDdot;
  Qmat_desCh[9] = 0.5 * s[21];
  Qmat_desCh[2] = 0.5 * s[22];
  Qmat_desCh[6] = l_mt;
  Qmat_desCh[10] = Tmag_mt;
  Qmat_desCh[3] = Qmat_desCh_tmp;
  Qmat_desCh[7] = 0.5 * s[20];
  Qmat_desCh[11] = l_mt;
  memset(&quar_TDot[0], 0, sizeof(real_T) << 2);
  r2 = _mm_set1_pd(0.5);
  for (i = 0; i < 3; i++) {
    __m128d r4;
    __m128d r6;
    maskMTreal = i << 2;
    r = _mm_loadu_pd(&b[maskMTreal]);
    r3 = _mm_loadu_pd(&quar_CDot[0]);
    r4 = _mm_set1_pd(s[i + 10]);
    _mm_storeu_pd(&quar_CDot[0],
                  _mm_add_pd(r3, _mm_mul_pd(_mm_mul_pd(r2, r), r4)));
    r = _mm_loadu_pd(&Qmat_desCh[maskMTreal]);
    r5 = _mm_loadu_pd(&quar_TDot[0]);
    r6 = _mm_set1_pd(s[i + 23]);
    _mm_storeu_pd(&quar_TDot[0], _mm_add_pd(r5, _mm_mul_pd(r, r6)));
    r = _mm_loadu_pd(&b[maskMTreal + 2]);
    r3 = _mm_loadu_pd(&quar_CDot[2]);
    _mm_storeu_pd(&quar_CDot[2],
                  _mm_add_pd(r3, _mm_mul_pd(_mm_mul_pd(r2, r), r4)));
    r = _mm_loadu_pd(&Qmat_desCh[maskMTreal + 2]);
    r5 = _mm_loadu_pd(&quar_TDot[2]);
    _mm_storeu_pd(&quar_TDot[2], _mm_add_pd(r5, _mm_mul_pd(r, r6)));
  }
  /*     %% Rotation Matris */
  /* From Field 2022 Thesis */
  Tmag_mt = s[6] * s[6];
  Qmat_desCh_tmp = s[7] * s[7];
  x1 = s[8] * s[8];
  x1dot = s[9] * s[9];
  rotMat_C_A_I[0] = ((Tmag_mt + Qmat_desCh_tmp) - x1) - x1dot;
  x_rDdot = s[7] * s[8];
  l_mt = s[6] * s[9];
  rotMat_C_A_I[1] = 2.0 * (x_rDdot - l_mt);
  rotMat_C_A_I_tmp = s[7] * s[9];
  b_rotMat_C_A_I_tmp = s[6] * s[8];
  rotMat_C_A_I[2] = 2.0 * (rotMat_C_A_I_tmp + b_rotMat_C_A_I_tmp);
  rotMat_C_A_I[3] = 2.0 * (x_rDdot + l_mt);
  Tmag_mt -= Qmat_desCh_tmp;
  rotMat_C_A_I[4] = (Tmag_mt + x1) - x1dot;
  Qmat_desCh_tmp = s[8] * s[9];
  x_rDdot = s[6] * s[7];
  rotMat_C_A_I[5] = 2.0 * (Qmat_desCh_tmp - x_rDdot);
  rotMat_C_A_I[6] = 2.0 * (rotMat_C_A_I_tmp - b_rotMat_C_A_I_tmp);
  rotMat_C_A_I[7] = 2.0 * (Qmat_desCh_tmp + x_rDdot);
  rotMat_C_A_I[8] = (Tmag_mt - x1) + x1dot;
  Tmag_mt = s[19] * s[19];
  Qmat_desCh_tmp = s[20] * s[20];
  x1 = s[21] * s[21];
  x1dot = s[22] * s[22];
  rotMat_D_A_I[0] = ((Tmag_mt + Qmat_desCh_tmp) - x1) - x1dot;
  x_rDdot = s[20] * s[21];
  l_mt = s[19] * s[22];
  rotMat_D_A_I[1] = 2.0 * (x_rDdot - l_mt);
  rotMat_C_A_I_tmp = s[20] * s[22];
  b_rotMat_C_A_I_tmp = s[19] * s[21];
  rotMat_D_A_I[2] = 2.0 * (rotMat_C_A_I_tmp + b_rotMat_C_A_I_tmp);
  rotMat_D_A_I[3] = 2.0 * (x_rDdot + l_mt);
  Tmag_mt -= Qmat_desCh_tmp;
  rotMat_D_A_I[4] = (Tmag_mt + x1) - x1dot;
  Qmat_desCh_tmp = s[21] * s[22];
  x_rDdot = s[19] * s[20];
  rotMat_D_A_I[5] = 2.0 * (Qmat_desCh_tmp - x_rDdot);
  rotMat_D_A_I[6] = 2.0 * (rotMat_C_A_I_tmp - b_rotMat_C_A_I_tmp);
  rotMat_D_A_I[7] = 2.0 * (Qmat_desCh_tmp + x_rDdot);
  rotMat_D_A_I[8] = (Tmag_mt - x1) + x1dot;
  /*  N2L Chaser */
  /*  For Tension in links */
  Tmag_mt = 0.5 * args->chaserSideLength;
  for (i = 0; i < 3; i++) {
    distAttPt_to_C[i] = Tmag_mt * (real_T)iv[i];
    y_tmp[3 * i] = rotMat_C_A_I[i];
    y_tmp[3 * i + 1] = rotMat_C_A_I[i + 3];
    y_tmp[3 * i + 2] = rotMat_C_A_I[i + 6];
  }
  Tmag_mt = distAttPt_to_C[0];
  Qmat_desCh_tmp = distAttPt_to_C[1];
  x_rDdot = distAttPt_to_C[2];
  r = _mm_loadu_pd(&y_tmp[0]);
  r3 = _mm_mul_pd(r, _mm_set1_pd(Tmag_mt));
  r = _mm_loadu_pd(&y_tmp[3]);
  r = _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp));
  r3 = _mm_add_pd(r3, r);
  r = _mm_loadu_pd(&y_tmp[6]);
  r = _mm_mul_pd(r, _mm_set1_pd(x_rDdot));
  r = _mm_add_pd(r3, r);
  r3 = _mm_loadu_pd(&s[0]);
  r = _mm_add_pd(r3, r);
  r3 = _mm_loadu_pd(&s[26]);
  r = _mm_sub_pd(r3, r);
  _mm_storeu_pd(&l_mt_vec[0], r);
  l_mt_vec[2] =
      s[28] - (s[2] + ((y_tmp[2] * Tmag_mt + y_tmp[5] * Qmat_desCh_tmp) +
                       y_tmp[8] * x_rDdot));
  l_mt = b_norm(l_mt_vec);
  /* Compute Tension */
  Tmag_mt = distAttPt_to_C[2] * s[11] - distAttPt_to_C[1] * s[12];
  Qmat_desCh_tmp = distAttPt_to_C[0] * s[12] - distAttPt_to_C[2] * s[10];
  x_rDdot = distAttPt_to_C[1] * s[10] - distAttPt_to_C[0] * s[11];
  r = _mm_loadu_pd(&l_mt_vec[0]);
  r = _mm_div_pd(r, _mm_set1_pd(l_mt));
  _mm_storeu_pd(&l_mt_vec[0], r);
  r = _mm_loadu_pd(&y_tmp[0]);
  r3 = _mm_mul_pd(r, _mm_set1_pd(Tmag_mt));
  r = _mm_loadu_pd(&y_tmp[3]);
  r = _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp));
  r3 = _mm_add_pd(r3, r);
  r = _mm_loadu_pd(&y_tmp[6]);
  r = _mm_mul_pd(r, _mm_set1_pd(x_rDdot));
  r = _mm_add_pd(r3, r);
  r3 = _mm_loadu_pd(&s[3]);
  r = _mm_add_pd(r3, r);
  r3 = _mm_loadu_pd(&s[29]);
  r = _mm_sub_pd(r3, r);
  _mm_storeu_pd(&VR_mt[0], r);
  _mm_storeu_pd(&appTorqueTarget[0], _mm_set1_pd(0.0));
  _mm_storeu_pd(&Tvec_mt[0], _mm_set1_pd(0.0));
  l_mt_vec[2] /= l_mt;
  VR_mt[2] = s[31] - (s[5] + ((y_tmp[2] * Tmag_mt + y_tmp[5] * Qmat_desCh_tmp) +
                              y_tmp[8] * x_rDdot));
  appTorqueTarget[2] = 0.0;
  Tvec_mt[2] = 0.0;
  maskMTreal = 0;
  if (l_mt > args->l0vec[0]) {
    n_t = (ptrdiff_t)3;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    Tmag_mt = ddot(&n_t, &VR_mt[0], &incx_t, &l_mt_vec[0], &incy_t);
    Tmag_mt = args->Kvec[0] * (l_mt - args->l0vec[0]) + args->cVec[0] * Tmag_mt;
    if (Tmag_mt > 0.0) {
      r = _mm_loadu_pd(&l_mt_vec[0]);
      _mm_storeu_pd(&Tvec_mt[0], _mm_mul_pd(_mm_set1_pd(Tmag_mt), r));
      Tvec_mt[2] = Tmag_mt * l_mt_vec[2];
      maskMTreal = 1;
    }
  }
  /*  slopeTime = 50.0; */
  /*  if (t < (slopeTime)) */
  /*      ThrustConstantFinal = FT * (t / (slopeTime)); */
  /*  else */
  /*      ThrustConstantFinal = FT; */
  /*  end */
  /*   */
  /*  Fthrust = -ThrustConstantFinal*(velChaser/norm(velChaser)); */
  /*     %% Adaptive Control  */
  x1 = l_mt - args->l0vec[0];
  n_t = (ptrdiff_t)3;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  x1dot = ddot(&n_t, &VR_mt[0], &incx_t, &l_mt_vec[0], &incy_t);
  ds[32] = s[33];
  Qmat_desCh_tmp = x1dot - s[33];
  l_mt = Qmat_desCh_tmp + args->L0 * (x1 - s[32]);
  /*  ========================== */
  /*  Desired elongation scaling */
  /*  ========================== */
  /*  Reference model dynamics */
  /*  ========================== */
  if (s[32] > 0.0) {
    Tmag_mt = s[32];
  } else {
    Tmag_mt = 0.0;
  }
  Tmag_mt = args->cVec[0] / args->chaserM * s[33] +
            args->Kvec[0] / args->chaserM * Tmag_mt;
  if (!(Tmag_mt > 0.0)) {
    Tmag_mt = 0.0;
  }
  ds[33] =
      -Tmag_mt +
      muDoubleScalarMin(
          muDoubleScalarMax(
              args->Kp * ((args->x1_m0 +
                           muDoubleScalarMin(
                               muDoubleScalarMax((t - args->tOnChaser_fromt0) /
                                                     args->slopeTime,
                                                 0.0),
                               1.0) *
                               (args->desElong - args->x1_m0)) -
                          s[32]) +
                  args->Kd * -s[33],
              -args->ThrustSaturation / args->chaserM),
          args->ThrustSaturation / args->chaserM);
  /*  linear ref model */
  x_rDdot = ds[33] - args->L0 * Qmat_desCh_tmp;
  Tmag_mt =
      (s[34] * x_rDdot + (real_T)maskMTreal * (x1 * s[35] + x1dot * s[36])) -
      args->kA * l_mt;
  /*  ========================== */
  /*  Thrust along +cZ direction */
  /*  ========================== */
  rotMat_C_A_I_tmp =
      muDoubleScalarMin(muDoubleScalarMax(Tmag_mt, -args->ThrustSaturation),
                        args->ThrustSaturation);
  /*  ds(35) = -1*gamma*sliding*x_rDdot*maskMTreal; */
  /*  ds(36) = -1*gamma*sliding*x1*maskMTreal; */
  /*  ds(37) = -1*gamma*sliding*x1dot*maskMTreal; */
  maskFT =
      (Tmag_mt * Tmag_mt < args->ThrustSaturation * args->ThrustSaturation);
  Qmat_desCh_tmp = -args->gamma * l_mt;
  Tmag_mt = l_mt * l_mt;
  ds[34] = (real_T)maskFT *
           (Qmat_desCh_tmp * x_rDdot - args->sigmaMRAC_h * Tmag_mt * s[34]);
  ds[35] = (real_T)maskFT * (Qmat_desCh_tmp * x1 * (real_T)maskMTreal -
                             args->sigmaMRAC_a1 * Tmag_mt * s[35]);
  ds[36] = (real_T)maskFT * (Qmat_desCh_tmp * x1dot * (real_T)maskMTreal -
                             args->sigmaMRAC_a2 * Tmag_mt * s[36]);
  ds[37] = 0.0;
  ds[38] = 0.0;
  /*     %% PID */
  /*      errorInt = s(33); */
  /*      Kp = 6000; Kd = 2000; Ki = 1000; desElong = 0.1; */
  /*      elongError = desElong - (l_mt - l0vec(1)); */
  /*      ds(33) = elongError; */
  /*   */
  /*      elongErrorDot = -dot(VR_mt,evec_mt); */
  /*      F_PD = Kp*elongError + Kd*elongErrorDot + Ki*errorInt; */
  /*      Fthrust = -F_PD*(velChaser/(norm(velChaser))); */
  /*     %% with gravity; */
  l_mt = b_norm(&s[0]);
  x1 = -args->mu * args->chaserM;
  x1dot = muDoubleScalarPower(l_mt, 3.0);
  x_rDdot = s[2] / l_mt;
  Tmag_mt = args->J2on * args->chaserM *
            (0.001623945 * args->mu *
             (4.0678884E+13 / muDoubleScalarPower(l_mt, 4.0)));
  memset(&accChaser[0], 0, 3U * sizeof(real_T));
  r = _mm_loadu_pd(&y_tmp[0]);
  r3 = _mm_loadu_pd(&accChaser[0]);
  r5 = _mm_set1_pd(0.0);
  _mm_storeu_pd(&accChaser[0], _mm_add_pd(r3, _mm_mul_pd(r, r5)));
  accChaser[2] += y_tmp[2] * 0.0;
  r = _mm_loadu_pd(&y_tmp[3]);
  r3 = _mm_loadu_pd(&accChaser[0]);
  _mm_storeu_pd(&accChaser[0], _mm_add_pd(r3, _mm_mul_pd(r, r5)));
  accChaser[2] += y_tmp[5] * 0.0;
  r = _mm_loadu_pd(&y_tmp[6]);
  r3 = _mm_loadu_pd(&accChaser[0]);
  _mm_storeu_pd(&accChaser[0], _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(1.0))));
  accChaser[2] += y_tmp[8];
  Qmat_desCh_tmp = 5.0 * (x_rDdot * x_rDdot);
  VR_mt[0] = Tmag_mt * ((Qmat_desCh_tmp - 1.0) * (s[0] / l_mt));
  VR_mt[1] = Tmag_mt * ((Qmat_desCh_tmp - 1.0) * (s[1] / l_mt));
  VR_mt[2] = Tmag_mt * ((Qmat_desCh_tmp - 3.0) * x_rDdot);
  /*     %% Sliding Att Control */
  /*  Vortex */
  /*      k_Loc = -evec_mt; */
  Tmag_mt = b_norm(&s[3]);
  r = _mm_loadu_pd(&accChaser[0]);
  r3 = _mm_loadu_pd(&Tvec_mt[0]);
  r2 = _mm_loadu_pd(&s[0]);
  r5 = _mm_loadu_pd(&VR_mt[0]);
  _mm_storeu_pd(
      &accChaser[0],
      _mm_div_pd(
          _mm_add_pd(
              _mm_add_pd(
                  _mm_add_pd(r3, _mm_mul_pd(_mm_set1_pd(rotMat_C_A_I_tmp), r)),
                  _mm_div_pd(_mm_mul_pd(_mm_set1_pd(x1), r2),
                             _mm_set1_pd(x1dot))),
              r5),
          _mm_set1_pd(args->chaserM)));
  r = _mm_loadu_pd(&s[3]);
  _mm_storeu_pd(&l_mt_vec[0], _mm_mul_pd(_mm_div_pd(r, _mm_set1_pd(Tmag_mt)),
                                         _mm_set1_pd(-1.0)));
  _mm_storeu_pd(&b_b[0], _mm_div_pd(r2, _mm_set1_pd(l_mt)));
  accChaser[2] =
      (((Tvec_mt[2] + rotMat_C_A_I_tmp * accChaser[2]) + x1 * s[2] / x1dot) +
       VR_mt[2]) /
      args->chaserM;
  l_mt_vec[2] = -(s[5] / Tmag_mt);
  VR_mt[0] = l_mt_vec[1] * x_rDdot - b_b[1] * l_mt_vec[2];
  VR_mt[1] = b_b[0] * l_mt_vec[2] - l_mt_vec[0] * x_rDdot;
  VR_mt[2] = l_mt_vec[0] * b_b[1] - b_b[0] * l_mt_vec[1];
  y_tmp[3] = l_mt_vec[1] * VR_mt[2] - VR_mt[1] * l_mt_vec[2];
  y_tmp[4] = VR_mt[0] * l_mt_vec[2] - l_mt_vec[0] * VR_mt[2];
  y_tmp[5] = l_mt_vec[0] * VR_mt[1] - VR_mt[0] * l_mt_vec[1];
  y_tmp[0] = VR_mt[0];
  y_tmp[6] = l_mt_vec[0];
  y_tmp[1] = VR_mt[1];
  y_tmp[7] = l_mt_vec[1];
  y_tmp[2] = VR_mt[2];
  y_tmp[8] = l_mt_vec[2];
  st.site = &e_emlrtRSI;
  rotm2quat(&st, y_tmp, quat_desCh);
  Qmat_desCh[0] = -quat_desCh[1];
  Qmat_desCh[4] = -quat_desCh[2];
  Qmat_desCh[8] = -quat_desCh[3];
  Qmat_desCh[1] = quat_desCh[0];
  Qmat_desCh[5] = -quat_desCh[3];
  Qmat_desCh[9] = quat_desCh[2];
  Qmat_desCh[2] = quat_desCh[3];
  Qmat_desCh[6] = quat_desCh[0];
  Qmat_desCh[10] = -quat_desCh[1];
  Qmat_desCh[3] = -quat_desCh[2];
  Qmat_desCh[7] = quat_desCh[1];
  Qmat_desCh[11] = quat_desCh[0];
  Tmag_mt = 0.0;
  for (i = 0; i < 4; i++) {
    q_eCh_tmp[3 * i] = Qmat_desCh[i];
    q_eCh_tmp[3 * i + 1] = Qmat_desCh[i + 4];
    q_eCh_tmp[3 * i + 2] = Qmat_desCh[i + 8];
    Tmag_mt += s[i + 6] * quat_desCh[i];
  }
  Tmag_mt = muDoubleScalarSign(Tmag_mt);
  Qmat_desCh_tmp = 16.0 * Tmag_mt;
  memset(&b_b[0], 0, 3U * sizeof(real_T));
  slidingVec_tmp[0] = s[10] * 0.0;
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->chaserI[0]),
                                         _mm_set1_pd(s[10]))));
  b_b[2] += args->chaserI[2] * s[10];
  slidingVec_tmp[1] = s[11] * 0.0;
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->chaserI[3]),
                                         _mm_set1_pd(s[11]))));
  b_b[2] += args->chaserI[5] * s[11];
  slidingVec_tmp[2] = s[12] * 0.0;
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->chaserI[6]),
                                         _mm_set1_pd(s[12]))));
  b_b[2] += args->chaserI[8] * s[12];
  tau_Att_Ch_tmp[0] = b_b[2] * s[11] - b_b[1] * s[12];
  tau_Att_Ch_tmp[1] = b_b[0] * s[12] - b_b[2] * s[10];
  tau_Att_Ch_tmp[2] = b_b[1] * s[10] - b_b[0] * s[11];
  a = 8.0 * Tmag_mt;
  memset(&l_mt_vec[0], 0, 3U * sizeof(real_T));
  r = _mm_loadu_pd(&q_eCh_tmp[0]);
  r3 = _mm_loadu_pd(&l_mt_vec[0]);
  _mm_storeu_pd(&l_mt_vec[0], _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(s[6]))));
  l_mt_vec[2] += q_eCh_tmp[2] * s[6];
  r = _mm_loadu_pd(&q_eCh_tmp[3]);
  r3 = _mm_loadu_pd(&l_mt_vec[0]);
  _mm_storeu_pd(&l_mt_vec[0], _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(s[7]))));
  l_mt_vec[2] += q_eCh_tmp[5] * s[7];
  r = _mm_loadu_pd(&q_eCh_tmp[6]);
  r3 = _mm_loadu_pd(&l_mt_vec[0]);
  _mm_storeu_pd(&l_mt_vec[0], _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(s[8]))));
  l_mt_vec[2] += q_eCh_tmp[8] * s[8];
  r = _mm_loadu_pd(&q_eCh_tmp[9]);
  r3 = _mm_loadu_pd(&l_mt_vec[0]);
  _mm_storeu_pd(&l_mt_vec[0], _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(s[9]))));
  l_mt_vec[2] += s[9] * q_eCh_tmp[11];
  r = _mm_loadu_pd(&s[10]);
  r3 = _mm_loadu_pd(&slidingVec_tmp[0]);
  r5 = _mm_loadu_pd(&l_mt_vec[0]);
  r = _mm_div_pd(_mm_add_pd(_mm_sub_pd(r, r3),
                            _mm_mul_pd(_mm_set1_pd(Qmat_desCh_tmp), r5)),
                 _mm_set1_pd(0.001));
  _mm_storeu_pd(&l_mt_vec[0], r);
  _mm_storeu_pd(&dv[0], r);
  dv[0] = muDoubleScalarMin(muDoubleScalarMax(dv[0], -1.0), 1.0);
  dv[1] = muDoubleScalarMin(muDoubleScalarMax(dv[1], -1.0), 1.0);
  r = _mm_loadu_pd(&dv[0]);
  _mm_storeu_pd(&b_b[0], r);
  b_b[2] = muDoubleScalarMin(
      muDoubleScalarMax(
          ((s[12] - slidingVec_tmp[2]) + Qmat_desCh_tmp * l_mt_vec[2]) / 0.001,
          -1.0),
      1.0);
  /*  with SMC */
  /*     %% MODIFY ST ATTACHMENT PT HERE
   * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  memset(&Tvec_st[0], 0, 12U * sizeof(real_T));
  for (i = 0; i < 3; i++) {
    y_tmp[3 * i] = rotMat_D_A_I[i];
    y_tmp[3 * i + 1] = rotMat_D_A_I[i + 3];
    y_tmp[3 * i + 2] = rotMat_D_A_I[i + 6];
  }
  rotMat_C_A_I_tmp = s[24];
  b_rotMat_C_A_I_tmp = s[25];
  x1 = s[23];
  for (i = 0; i < 4; i++) {
    /*  if i == 1 */
    /*      distAttPt_to_D = ([-0.5*targetSideLengthY, 0.0,  -5.1 ]'); */
    /*  elseif i == 2 */
    /*      distAttPt_to_D = ([-0.5*targetSideLengthY,0.0,  5.1 ]'); */
    /*  elseif i == 3 */
    /*      distAttPt_to_D = ([0.5*targetSideLengthY, 0.0, -5.1 ]'); */
    /*  elseif i == 4 */
    /*      distAttPt_to_D = ([0.5*targetSideLengthY,0.0,  5.1 ]'); */
    /*  end  */
    if (i == 0) {
      b_distAttPt_to_D[0] = -0.5 * targetSideLengthY;
      b_distAttPt_to_D[2] = args->targetSideLengthZ[0];
    } else if (i + 1 == 2) {
      b_distAttPt_to_D[0] = -0.5 * targetSideLengthY;
      b_distAttPt_to_D[2] = args->targetSideLengthZ[1];
    } else if (i + 1 == 3) {
      b_distAttPt_to_D[0] = 0.5 * targetSideLengthY;
      b_distAttPt_to_D[2] = args->targetSideLengthZ[2];
    } else {
      b_distAttPt_to_D[0] = 0.5 * targetSideLengthY;
      b_distAttPt_to_D[2] = args->targetSideLengthZ[3];
    }
    /*  For Tension in links */
    Tmag_mt = b_distAttPt_to_D[0];
    Qmat_desCh_tmp = b_distAttPt_to_D[2];
    r = _mm_loadu_pd(&s[26]);
    r3 = _mm_loadu_pd(&s[13]);
    r5 = _mm_sub_pd(r, r3);
    r = _mm_loadu_pd(&y_tmp[0]);
    r3 = _mm_mul_pd(r, _mm_set1_pd(Tmag_mt));
    r = _mm_loadu_pd(&y_tmp[3]);
    r = _mm_mul_pd(r, _mm_set1_pd(0.0));
    r3 = _mm_add_pd(r3, r);
    r = _mm_loadu_pd(&y_tmp[6]);
    r = _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp));
    r = _mm_add_pd(r3, r);
    r = _mm_sub_pd(r5, r);
    _mm_storeu_pd(&l_mt_vec[0], r);
    l_mt_vec[2] = (s[28] - s[15]) - ((y_tmp[2] * Tmag_mt + y_tmp[5] * 0.0) +
                                     y_tmp[8] * Qmat_desCh_tmp);
    x_rDdot = b_norm(l_mt_vec);
    Tmag_mt = b_distAttPt_to_D[2] * rotMat_C_A_I_tmp - 0.0 * b_rotMat_C_A_I_tmp;
    Qmat_desCh_tmp =
        b_distAttPt_to_D[0] * b_rotMat_C_A_I_tmp - b_distAttPt_to_D[2] * x1;
    l_mt = x1 * 0.0 - b_distAttPt_to_D[0] * rotMat_C_A_I_tmp;
    r = _mm_loadu_pd(&l_mt_vec[0]);
    r = _mm_div_pd(r, _mm_set1_pd(x_rDdot));
    _mm_storeu_pd(&l_mt_vec[0], r);
    r = _mm_loadu_pd(&s[29]);
    r3 = _mm_loadu_pd(&s[16]);
    r5 = _mm_sub_pd(r, r3);
    r = _mm_loadu_pd(&y_tmp[0]);
    r3 = _mm_mul_pd(r, _mm_set1_pd(Tmag_mt));
    r = _mm_loadu_pd(&y_tmp[3]);
    r = _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp));
    r3 = _mm_add_pd(r3, r);
    r = _mm_loadu_pd(&y_tmp[6]);
    r = _mm_mul_pd(r, _mm_set1_pd(l_mt));
    r = _mm_add_pd(r3, r);
    r = _mm_sub_pd(r5, r);
    _mm_storeu_pd(&VR_mt[0], r);
    l_mt_vec[2] /= x_rDdot;
    VR_mt[2] =
        (s[31] - s[18]) -
        ((y_tmp[2] * Tmag_mt + y_tmp[5] * Qmat_desCh_tmp) + y_tmp[8] * l_mt);
    /* Compute Tension */
    Tmag_mt = args->l0vec[i + 1];
    if (x_rDdot > Tmag_mt) {
      n_t = (ptrdiff_t)3;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      Qmat_desCh_tmp = ddot(&n_t, &VR_mt[0], &incx_t, &l_mt_vec[0], &incy_t);
      Tmag_mt = args->Kvec[i + 1] * (x_rDdot - Tmag_mt) +
                args->cVec[i + 1] * Qmat_desCh_tmp;
      if (Tmag_mt > 0.0) {
        r = _mm_loadu_pd(&l_mt_vec[0]);
        _mm_storeu_pd(&Tvec_st[3 * i], _mm_mul_pd(_mm_set1_pd(Tmag_mt), r));
        Tvec_st[3 * i + 2] = Tmag_mt * l_mt_vec[2];
      }
    }
    memset(&l_mt_vec[0], 0, 3U * sizeof(real_T));
    Tmag_mt = Tvec_st[3 * i];
    r = _mm_loadu_pd(&rotMat_D_A_I[0]);
    r3 = _mm_loadu_pd(&l_mt_vec[0]);
    _mm_storeu_pd(&l_mt_vec[0],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Tmag_mt))));
    l_mt_vec[2] += rotMat_D_A_I[2] * Tmag_mt;
    Tmag_mt = Tvec_st[3 * i + 1];
    r = _mm_loadu_pd(&rotMat_D_A_I[3]);
    r3 = _mm_loadu_pd(&l_mt_vec[0]);
    _mm_storeu_pd(&l_mt_vec[0],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Tmag_mt))));
    l_mt_vec[2] += rotMat_D_A_I[5] * Tmag_mt;
    Tmag_mt = Tvec_st[3 * i + 2];
    r = _mm_loadu_pd(&rotMat_D_A_I[6]);
    r3 = _mm_loadu_pd(&l_mt_vec[0]);
    _mm_storeu_pd(&l_mt_vec[0],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Tmag_mt))));
    l_mt_vec[2] += rotMat_D_A_I[8] * Tmag_mt;
    Tmag_mt = appTorqueTarget[0];
    Qmat_desCh_tmp = appTorqueTarget[1];
    x_rDdot = appTorqueTarget[2];
    appTorqueTarget[0] =
        Tmag_mt + (0.0 * l_mt_vec[2] - l_mt_vec[1] * b_distAttPt_to_D[2]);
    appTorqueTarget[1] = Qmat_desCh_tmp + (l_mt_vec[0] * b_distAttPt_to_D[2] -
                                           b_distAttPt_to_D[0] * l_mt_vec[2]);
    appTorqueTarget[2] =
        x_rDdot + (b_distAttPt_to_D[0] * l_mt_vec[1] - l_mt_vec[0] * 0.0);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  Tmag_mt = -args->mu * args->targetM;
  Qmat_desCh_tmp = muDoubleScalarPower(b_norm(&s[13]), 3.0);
  accPoint1[0] = Tvec_st[0];
  accPoint1[1] = Tvec_st[1];
  accPoint1[2] = Tvec_st[2];
  r = _mm_loadu_pd(&accPoint1[0]);
  r3 = _mm_loadu_pd(&Tvec_st[3]);
  _mm_storeu_pd(&accPoint1[0], _mm_add_pd(r, r3));
  accPoint1[2] += Tvec_st[5];
  r = _mm_loadu_pd(&accPoint1[0]);
  r3 = _mm_loadu_pd(&Tvec_st[6]);
  _mm_storeu_pd(&accPoint1[0], _mm_add_pd(r, r3));
  accPoint1[2] += Tvec_st[8];
  r = _mm_loadu_pd(&accPoint1[0]);
  r3 = _mm_loadu_pd(&Tvec_st[9]);
  _mm_storeu_pd(&accPoint1[0], _mm_add_pd(r, r3));
  accPoint1[2] += Tvec_st[11];
  r = _mm_loadu_pd(&s[13]);
  r3 = _mm_loadu_pd(&accPoint1[0]);
  _mm_storeu_pd(
      &b_distAttPt_to_D[0],
      _mm_div_pd(_mm_add_pd(r3, _mm_div_pd(_mm_mul_pd(_mm_set1_pd(Tmag_mt), r),
                                           _mm_set1_pd(Qmat_desCh_tmp))),
                 _mm_set1_pd(args->targetM)));
  b_distAttPt_to_D[2] =
      (accPoint1[2] + Tmag_mt * s[15] / Qmat_desCh_tmp) / args->targetM;
  /*  N2L Connection Point */
  Tmag_mt = -args->mu * args->massPoint1;
  x_rDdot = muDoubleScalarPower(b_norm(&s[26]), 3.0);
  /*  */
  memset(&l_mt_vec[0], 0, 3U * sizeof(real_T));
  memset(&distAttPt_to_D[0], 0, 3U * sizeof(real_T));
  memset(&y_tmp[0], 0, 9U * sizeof(real_T));
  for (b_i = 0; b_i < 3; b_i++) {
    int32_T y_tmp_tmp;
    accPoint1[b_i] =
        (-(accPoint1[b_i] + Tvec_mt[b_i]) + Tmag_mt * s[b_i + 26] / x_rDdot) /
        args->massPoint1;
    maskMTreal = b_i << 2;
    for (i = 0; i < 3; i++) {
      l_mt_vec[i] += rotMat_C_A_I[i + 3 * b_i] * Tvec_mt[b_i];
      y_tmp_tmp = i << 2;
      distAttPt_to_D[i] += (((b[y_tmp_tmp] * Qmat_desCh[maskMTreal] +
                              b[y_tmp_tmp + 1] * Qmat_desCh[maskMTreal + 1]) +
                             b[y_tmp_tmp + 2] * Qmat_desCh[maskMTreal + 2]) +
                            b[y_tmp_tmp + 3] * Qmat_desCh[maskMTreal + 3]) *
                           s[b_i + 10];
    }
    maskMTreal = b_i << 2;
    Qmat_desCh_tmp = b[maskMTreal];
    r = _mm_loadu_pd(&q_eCh_tmp[0]);
    r3 = _mm_loadu_pd(&y_tmp[3 * b_i]);
    _mm_storeu_pd(&y_tmp[3 * b_i],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp))));
    y_tmp_tmp = 3 * b_i + 2;
    y_tmp[y_tmp_tmp] += q_eCh_tmp[2] * Qmat_desCh_tmp;
    Qmat_desCh_tmp = b[maskMTreal + 1];
    r = _mm_loadu_pd(&q_eCh_tmp[3]);
    r3 = _mm_loadu_pd(&y_tmp[3 * b_i]);
    _mm_storeu_pd(&y_tmp[3 * b_i],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp))));
    y_tmp[y_tmp_tmp] += q_eCh_tmp[5] * Qmat_desCh_tmp;
    Qmat_desCh_tmp = b[maskMTreal + 2];
    r = _mm_loadu_pd(&q_eCh_tmp[6]);
    r3 = _mm_loadu_pd(&y_tmp[3 * b_i]);
    _mm_storeu_pd(&y_tmp[3 * b_i],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp))));
    y_tmp[y_tmp_tmp] += q_eCh_tmp[8] * Qmat_desCh_tmp;
    Qmat_desCh_tmp = b[maskMTreal + 3];
    r = _mm_loadu_pd(&q_eCh_tmp[9]);
    r3 = _mm_loadu_pd(&y_tmp[3 * b_i]);
    _mm_storeu_pd(&y_tmp[3 * b_i],
                  _mm_add_pd(r3, _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp))));
    y_tmp[y_tmp_tmp] += q_eCh_tmp[11] * Qmat_desCh_tmp;
  }
  Tmag_mt = s[10];
  Qmat_desCh_tmp = s[11];
  x_rDdot = s[12];
  l_mt = b_b[0];
  x1 = b_b[1];
  x1dot = b_b[2];
  for (i = 0; i < 3; i++) {
    VR_mt[i] = (a * (distAttPt_to_D[i] * 0.0 -
                     ((y_tmp[i] * Tmag_mt + y_tmp[i + 3] * Qmat_desCh_tmp) +
                      y_tmp[i + 6] * x_rDdot)) +
                slidingVec_tmp[i]) -
               (((real_T)b_a[i] * l_mt + (real_T)b_a[i + 3] * x1) +
                (real_T)b_a[i + 6] * x1dot);
  }
  Tvec_mt[0] =
      distAttPt_to_C[1] * l_mt_vec[2] - l_mt_vec[1] * distAttPt_to_C[2];
  Tvec_mt[1] =
      l_mt_vec[0] * distAttPt_to_C[2] - distAttPt_to_C[0] * l_mt_vec[2];
  Tvec_mt[2] =
      distAttPt_to_C[0] * l_mt_vec[1] - l_mt_vec[0] * distAttPt_to_C[1];
  Tmag_mt = VR_mt[0];
  Qmat_desCh_tmp = VR_mt[1];
  x_rDdot = VR_mt[2];
  r = _mm_loadu_pd(&args->chaserI[0]);
  r3 = _mm_mul_pd(r, _mm_set1_pd(Tmag_mt));
  r = _mm_loadu_pd(&args->chaserI[3]);
  r = _mm_mul_pd(r, _mm_set1_pd(Qmat_desCh_tmp));
  r3 = _mm_add_pd(r3, r);
  r = _mm_loadu_pd(&args->chaserI[6]);
  r = _mm_mul_pd(r, _mm_set1_pd(x_rDdot));
  r = _mm_add_pd(r3, r);
  r5 = _mm_loadu_pd(&tau_Att_Ch_tmp[0]);
  r = _mm_add_pd(r5, r);
  r3 = _mm_loadu_pd(&Tvec_mt[0]);
  r = _mm_add_pd(r3, r);
  r = _mm_sub_pd(r, r5);
  _mm_storeu_pd(&Tvec_mt[0], r);
  l_mt = tau_Att_Ch_tmp[2];
  Tvec_mt[2] = (Tvec_mt[2] + (l_mt + ((args->chaserI[2] * Tmag_mt +
                                       args->chaserI[5] * Qmat_desCh_tmp) +
                                      args->chaserI[8] * x_rDdot))) -
               l_mt;
  st.site = &b_emlrtRSI;
  mldivide(&st, args->chaserI, Tvec_mt, VR_mt);
  memset(&b_b[0], 0, 3U * sizeof(real_T));
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->targetI[0]),
                                         _mm_set1_pd(s[23]))));
  b_b[2] += args->targetI[2] * s[23];
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->targetI[3]),
                                         _mm_set1_pd(s[24]))));
  b_b[2] += args->targetI[5] * s[24];
  r = _mm_loadu_pd(&b_b[0]);
  _mm_storeu_pd(&b_b[0],
                _mm_add_pd(r, _mm_mul_pd(_mm_loadu_pd(&args->targetI[6]),
                                         _mm_set1_pd(s[25]))));
  b_b[2] += args->targetI[8] * s[25];
  distAttPt_to_D[0] = appTorqueTarget[0] - (b_b[2] * s[24] - b_b[1] * s[25]);
  distAttPt_to_D[1] = appTorqueTarget[1] - (b_b[0] * s[25] - b_b[2] * s[23]);
  distAttPt_to_D[2] = appTorqueTarget[2] - (b_b[1] * s[23] - b_b[0] * s[24]);
  st.site = &emlrtRSI;
  mldivide(&st, args->targetI, distAttPt_to_D, l_mt_vec);
  /* chaser state derivatives */
  /* linear vel */
  ds[0] = s[3];
  ds[3] = accChaser[0];
  ds[1] = s[4];
  ds[4] = accChaser[1];
  ds[2] = s[5];
  ds[5] = accChaser[2];
  /* acc vel */
  ds[6] = quar_CDot[0];
  ds[7] = quar_CDot[1];
  ds[8] = quar_CDot[2];
  ds[9] = quar_CDot[3];
  /* ang vel */
  /* ang acc */
  /* target state derivatives */
  /* linear vel */
  ds[10] = VR_mt[0];
  ds[13] = s[16];
  ds[16] = b_distAttPt_to_D[0];
  ds[11] = VR_mt[1];
  ds[14] = s[17];
  ds[17] = b_distAttPt_to_D[1];
  ds[12] = VR_mt[2];
  ds[15] = s[18];
  ds[18] = b_distAttPt_to_D[2];
  /* acc vel */
  ds[19] = quar_TDot[0];
  ds[20] = quar_TDot[1];
  ds[21] = quar_TDot[2];
  ds[22] = quar_TDot[3];
  /* ang vel */
  /* ang acc */
  /* connection state derivatives */
  /* linear vel */
  ds[23] = l_mt_vec[0];
  ds[26] = s[29];
  ds[29] = accPoint1[0];
  ds[24] = l_mt_vec[1];
  ds[27] = s[30];
  ds[30] = accPoint1[1];
  ds[25] = l_mt_vec[2];
  ds[28] = s[31];
  ds[31] = accPoint1[2];
  /* acc vel */
  r = _mm_loadu_pd(&ds[0]);
  _mm_storeu_pd(&ds[0], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[13]);
  _mm_storeu_pd(&ds[13], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[26]);
  _mm_storeu_pd(&ds[26], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[2]);
  _mm_storeu_pd(&ds[2], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[15]);
  _mm_storeu_pd(&ds[15], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[28]);
  _mm_storeu_pd(&ds[28], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[4]);
  _mm_storeu_pd(&ds[4], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[17]);
  _mm_storeu_pd(&ds[17], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&ds[30]);
  _mm_storeu_pd(&ds[30], _mm_div_pd(r, r1));
}

/* End of code generation (stateDeriv_withGrav_LiamSet_MRAC1_args.c) */
