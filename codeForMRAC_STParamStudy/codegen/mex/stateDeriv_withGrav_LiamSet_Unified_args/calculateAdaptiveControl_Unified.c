/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * calculateAdaptiveControl_Unified.c
 *
 * Code generation for function 'calculateAdaptiveControl_Unified'
 *
 */

/* Include files */
#include "calculateAdaptiveControl_Unified.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo eb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    134,                                /* lineNo */
    16,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    133,                                /* lineNo */
    16,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    132,                                /* lineNo */
    14,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    131,                                /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    54,                                 /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    53,                                 /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo kb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    52,                                 /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo lb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    51,                                 /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo mb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    50,                                 /* lineNo */
    17,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

static emlrtBCInfo nb_emlrtBCI = {
    1,                                  /* iFirst */
    95,                                 /* iLast */
    135,                                /* lineNo */
    19,                                 /* colNo */
    "s",                                /* aName */
    "calculateAdaptiveControl_Unified", /* fName */
    "D:\\CM "
    "Labs\\subTetherVortexMATLAB\\codeForMRAC_"
    "STParamStudy\\calculateAdaptiveControl_Unified.m", /* pName */
    0                                                   /* checkKind */
};

/* Function Definitions */
void c_calculateAdaptiveControl_Unif(
    const emlrtStack *sp, real_T t, const real_T s[95], const struct0_T *args,
    real_T mrac_idx, real_T l_mt_seg1, const real_T VR_mt_seg1[3],
    const real_T evec_mt_seg1[3], boolean_T maskMTreal,
    const real_T rotMat_C_A_I[9], real_T ds_mrac[9], real_T Fthrust[3],
    real_T MRAC_1_updateParams[7], real_T MRAC_2_updateParams[11])
{
  __m128d r;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T ds_mrac_1[9];
  real_T ds_mrac_2[9];
  real_T b_args[4];
  real_T Fthrust_1[3];
  real_T Fthrust_1_tmp[3];
  real_T Phi_idx_0;
  real_T Theta_hat_idx_0;
  real_T Theta_hat_idx_1;
  real_T b_delta_tmp;
  real_T b_ds_mrac_1_tmp;
  real_T b_s;
  real_T b_sliding_tmp;
  real_T c_delta_tmp;
  real_T c_ds_mrac_1_tmp;
  real_T clipped_delta;
  real_T d;
  real_T delta;
  real_T delta_tmp;
  real_T ds_mrac_1_tmp;
  real_T maskFT_tmp;
  real_T sliding;
  real_T sliding_tmp;
  real_T x1_tmp;
  real_T x1dot;
  real_T x_rDdot;
  real_T x_vec_idx_0;
  int32_T i;
  boolean_T maskFT;
  /*  calculateAdaptiveControl_Unified */
  /*  */
  /*  Computes the adaptive control law and state derivatives for the Model */
  /*  Reference Adaptive Control (MRAC) system applied to the tethered system.
   */
  /*  This function supports two versions of MRAC logic, selected via
   * args.MRAC_v. */
  /*  */
  /*  Inputs: */
  /*    t            - Current simulation time (scalar) */
  /*    s            - State vector (linear array) */
  /*    args         - Structure containing system parameters and control gains:
   */
  /*                   * args.MRAC_v: Control version flag */
  /*                   * args.chaserM: Mass of the chaser */
  /*                   * args.cVec, args.Kvec: Damping and stiffness arrays */
  /*                   * args.l0vec: Unstretched tether lengths */
  /*                   * args.L0, args.kA, args.gamma: MRAC-1 specific gains */
  /*                   * args.Gamma_x, args.Gamma_r, args.Gamma_theta: Adaptive
   * Linear update gains */
  /*                   * args.P, args.B_linear: Lyapunov equation parameters */
  /*                   * args.ThrustSaturation: Maximum allowable thrust */
  /*                   * ... and other desElong/slopeTime parameters. */
  /*    mrac_idx     - Index for the start of MRAC states in vector 's' */
  /*    l_mt_seg1    - Current length of the first tether segment (scalar) */
  /*    VR_mt_seg1   - Relative velocity vector of the first tether segment
   * (3x1) */
  /*    evec_mt_seg1 - Unit vector along the first tether segment (3x1) */
  /*    maskMTreal   - Boolean flag indicating if the tether segment is taut */
  /*    rotMat_C_A_I - Rotation matrix from Inertial to Chaser Attachment frame
   * (3x3) */
  /*  */
  /*  Outputs: */
  /*    ds_mrac      - Derivative of the MRAC states (1x9 vector). */
  /*                   Contains derivatives for reference model, adaptive gains,
   * and tracked variables. */
  /*    Fthrust      - Calculated thrust force vector (3x1) in the Inertial
   * frame. */
  /*  */
  /*     %% Extract Parameters needed locally */
  /*  Initialize output state derivative vector (size 9 to match allocations) */
  memset(&ds_mrac[0], 0, 9U * sizeof(real_T));
  Fthrust[0] = 0.0;
  Fthrust[1] = 0.0;
  Fthrust[2] = 0.0;
  /*  ==================================================================================================================================================
   */
  /*  MRAC Version 1: */
  /*  ==================================================================================================================================================
   */
  /*  --- Extract current Adaptive States --- */
  if (((int32_T)mrac_idx < 1) || ((int32_T)mrac_idx > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)mrac_idx, 1, 95, &mb_emlrtBCI,
                                  (emlrtConstCTX)sp);
  }
  /*  Reference model state (position/elongation) */
  if (((int32_T)(mrac_idx + 1.0) < 1) || ((int32_T)(mrac_idx + 1.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 1.0), 1, 95,
                                  &lb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Reference model state (rate) */
  if (((int32_T)(mrac_idx + 2.0) < 1) || ((int32_T)(mrac_idx + 2.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 2.0), 1, 95,
                                  &kb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive parameter estimate: h_hat */
  if (((int32_T)(mrac_idx + 3.0) < 1) || ((int32_T)(mrac_idx + 3.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 3.0), 1, 95,
                                  &jb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive parameter estimate: a_hat_1 */
  if (((int32_T)(mrac_idx + 4.0) < 1) || ((int32_T)(mrac_idx + 4.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 4.0), 1, 95,
                                  &ib_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive parameter estimate: a_hat_2 */
  /*  --- Extract Gains for MRAC 1 --- */
  /*  Sliding surface gain / Decay rate */
  /*  Control gain */
  /*  Adaptation rate */
  /*  Sigma-modification term for h */
  /*  Sigma-modification term for a1 */
  /*  Sigma-modification term for a2 */
  /*  --- Compute System States for Segment 1 --- */
  /*  x1: Elongation error (Current length - Unstretched length) */
  x1_tmp = l_mt_seg1 - args->l0vec[0];
  /*  x1dot: Rate of change of elongation (projected relative velocity) */
  n_t = (ptrdiff_t)3;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  x1dot = ddot(&n_t, (real_T *)&VR_mt_seg1[0], &incx_t,
               (real_T *)&evec_mt_seg1[0], &incy_t);
  /*  1. Derivative of Reference Model Position */
  ds_mrac_1_tmp = s[(int32_T)(mrac_idx + 1.0) - 1];
  ds_mrac_1[0] = ds_mrac_1_tmp;
  /*  --- Desired Trajectory Generation --- */
  /*  Ramps up desired elongation from x1_m0 to desElong over slopeTime */
  /*  Sliding surface variable 's' (not to be confused with state vector s) */
  b_s = s[(int32_T)mrac_idx - 1];
  sliding_tmp = x1dot - ds_mrac_1_tmp;
  b_sliding_tmp = x1_tmp - b_s;
  sliding = sliding_tmp + args->L0 * b_sliding_tmp;
  /*  Compute time-varying desired elongation */
  /*  --- Reference Model Dynamics --- */
  /*  The reference model is a mass-spring-damper system driven by PD control to
   * track desElonVary. */
  /*  It includes a saturation term to represent physical thrust limits. */
  if (b_s > 0.0) {
    Phi_idx_0 = b_s;
  } else {
    Phi_idx_0 = 0.0;
  }
  Phi_idx_0 = args->cVec[0] / args->chaserM * ds_mrac_1_tmp +
              args->Kvec[0] / args->chaserM * Phi_idx_0;
  b_ds_mrac_1_tmp =
      args->x1_m0 +
      muDoubleScalarMin(
          muDoubleScalarMax((t - args->tOnChaser_fromt0) / args->slopeTime,
                            0.0),
          1.0) *
          (args->desElong - args->x1_m0);
  if (!(Phi_idx_0 > 0.0)) {
    Phi_idx_0 = 0.0;
  }
  c_ds_mrac_1_tmp =
      -Phi_idx_0 +
      muDoubleScalarMin(
          muDoubleScalarMax(args->Kp * (b_ds_mrac_1_tmp - b_s) +
                                args->Kd * -ds_mrac_1_tmp,
                            -args->ThrustSaturation / args->chaserM),
          args->ThrustSaturation / args->chaserM);
  ds_mrac_1[1] = c_ds_mrac_1_tmp;
  /*  --- Control Law Calculation --- */
  x_rDdot = c_ds_mrac_1_tmp - args->L0 * sliding_tmp;
  delta_tmp = s[(int32_T)(mrac_idx + 2.0) - 1];
  b_delta_tmp = s[(int32_T)(mrac_idx + 3.0) - 1];
  c_delta_tmp = s[(int32_T)(mrac_idx + 4.0) - 1];
  x_vec_idx_0 = x1_tmp * b_delta_tmp + x1dot * c_delta_tmp;
  delta = (delta_tmp * x_rDdot + (real_T)maskMTreal * x_vec_idx_0) -
          args->kA * sliding;
  /*  --- Thrust Calculation --- */
  /*  Saturate the calculated control effort */
  clipped_delta =
      muDoubleScalarMin(muDoubleScalarMax(delta, -args->ThrustSaturation),
                        args->ThrustSaturation);
  /*  Apply thrust in the Chaser's body frame (+Z direction), rotated to
   * Inertial frame */
  for (i = 0; i < 3; i++) {
    int32_T aoffset;
    aoffset = i * 3;
    b_s = (rotMat_C_A_I[aoffset] * 0.0 + rotMat_C_A_I[aoffset + 1] * 0.0) +
          rotMat_C_A_I[aoffset + 2];
    Fthrust_1_tmp[i] = b_s;
    Fthrust_1[i] = clipped_delta * b_s;
  }
  /*  --- Adaptive Laws Update --- */
  /*  Only update adaptive parameters if not saturated to prevent wind-up */
  b_s = muDoubleScalarAbs(delta);
  maskFT_tmp = args->ThrustSaturation * args->ThrustSaturation;
  maskFT = ((b_s + 0.1) * (b_s + 0.1) < maskFT_tmp);
  /*  Update laws with Sigma-modification for robustness */
  Phi_idx_0 = -args->gamma * sliding;
  sliding *= sliding;
  b_s = Phi_idx_0 * x_rDdot;
  ds_mrac_1[2] =
      (real_T)maskFT * (b_s - args->sigmaMRAC_h * sliding * delta_tmp);
  d = Phi_idx_0 * x1_tmp * (real_T)maskMTreal;
  ds_mrac_1[3] =
      (real_T)maskFT * (d - args->sigmaMRAC_a1 * sliding * b_delta_tmp);
  Phi_idx_0 = Phi_idx_0 * x1dot * (real_T)maskMTreal;
  ds_mrac_1[4] =
      (real_T)maskFT * (Phi_idx_0 - args->sigmaMRAC_a2 * sliding * c_delta_tmp);
  MRAC_1_updateParams[0] = maskFT;
  MRAC_1_updateParams[1] = b_s;
  MRAC_1_updateParams[2] = -args->sigmaMRAC_h * sliding * delta_tmp;
  MRAC_1_updateParams[3] = d;
  MRAC_1_updateParams[4] = -args->sigmaMRAC_a1 * sliding * b_delta_tmp;
  MRAC_1_updateParams[5] = Phi_idx_0;
  MRAC_1_updateParams[6] = -args->sigmaMRAC_a2 * sliding * c_delta_tmp;
  /*  Unused states for this version (fillers) */
  ds_mrac_1[5] = 0.0;
  ds_mrac_1[6] = 0.0;
  /*  Record tracked variables for debugging/plotting */
  ds_mrac_1[7] = x1dot;
  ds_mrac_1[8] = clipped_delta;
  /*  ==================================================================================================================================================
   */
  /*  MRAC Version 2: */
  /*  ==================================================================================================================================================
   */
  /*  --- Extract States --- */
  /*  Ref model pos */
  if (((int32_T)(mrac_idx + 1.0) < 1) || ((int32_T)(mrac_idx + 1.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 1.0), 1, 95,
                                  &hb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Ref model rate */
  if (((int32_T)(mrac_idx + 2.0) < 1) || ((int32_T)(mrac_idx + 2.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 2.0), 1, 95,
                                  &gb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive gain: K_r_hat (Command feedforward) */
  if (((int32_T)(mrac_idx + 3.0) < 1) || ((int32_T)(mrac_idx + 3.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 3.0), 1, 95,
                                  &fb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive gain: K_x_hat component 1 */
  if (((int32_T)(mrac_idx + 4.0) < 1) || ((int32_T)(mrac_idx + 4.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 4.0), 1, 95,
                                  &eb_emlrtBCI, (emlrtConstCTX)sp);
  }
  /*  Adaptive gain: K_x_hat component 2 */
  if (((int32_T)(mrac_idx + 5.0) < 1) || ((int32_T)(mrac_idx + 5.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)(mrac_idx + 5.0), 1, 95,
                                  &nb_emlrtBCI, (emlrtConstCTX)sp);
  }
  Theta_hat_idx_0 = s[(int32_T)(mrac_idx + 5.0) - 1];
  if (((int32_T)((mrac_idx + 5.0) + 1.0) < 1) ||
      ((int32_T)((mrac_idx + 5.0) + 1.0) > 95)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)((mrac_idx + 5.0) + 1.0), 1, 95,
                                  &nb_emlrtBCI, (emlrtConstCTX)sp);
  }
  Theta_hat_idx_1 = s[(int32_T)((mrac_idx + 5.0) + 1.0) - 1];
  /*  Adaptive gain: Theta_hat ( */
  /*  Reconstruct gain vectors */
  /*  --- System States (Segment 1) --- */
  /*  1. Derivative of Reference Model Position */
  ds_mrac_2[0] = ds_mrac_1_tmp;
  /*  --- Desired Trajectory --- */
  /*  Extract Adaptive Linear specific gains */
  /*  Solution to Lyapunov equation */
  /*  Compute desired elongation */
  /*  --- Reference Model Dynamics --- */
  ds_mrac_2[1] = c_ds_mrac_1_tmp;
  /*  --- Error and Regressor Definitions --- */
  /*  Tracking error vector e = x - x_m */
  /*  Phi is the regressor vector. It zeroes out terms when the tether is slack
   * (maskMTreal=0) */
  Phi_idx_0 = x1_tmp * ((real_T)maskMTreal - 1.0);
  b_s = x1dot * ((real_T)maskMTreal - 1.0);
  /*  State vector x */
  /*  --- Control Input Calculation --- */
  /*  u = K_x'x + K_r*r - Theta'*Phi */
  delta = (x_vec_idx_0 + delta_tmp * b_ds_mrac_1_tmp) -
          (Theta_hat_idx_0 * Phi_idx_0 + Theta_hat_idx_1 * b_s);
  /*  --- Adaptive Laws Update --- */
  /*  Updates based on Lyapunov stability analysis (e'PB term drives adaptation)
   */
  /*  Includes sigma-modification for robustness. */
  x_rDdot = (b_sliding_tmp * args->P[0] + sliding_tmp * args->P[1]) *
                args->B_linear[0] +
            (b_sliding_tmp * args->P[2] + sliding_tmp * args->P[3]) *
                args->B_linear[1];
  /*  Scalar term appearing in all updates */
  d = muDoubleScalarAbs(x_rDdot);
  r = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&b_args[0], _mm_mul_pd(_mm_loadu_pd(&args->Gamma_x[0]), r));
  _mm_storeu_pd(&b_args[2], _mm_mul_pd(_mm_loadu_pd(&args->Gamma_x[2]), r));
  sliding_tmp = (b_args[0] * x1_tmp + b_args[2] * x1dot) * x_rDdot;
  c_ds_mrac_1_tmp = (b_args[1] * x1_tmp + b_args[3] * x1dot) * x_rDdot;
  ds_mrac_1_tmp = args->sigmaMRACLin * d;
  _mm_storeu_pd(&b_args[0], _mm_mul_pd(_mm_loadu_pd(&args->Gamma_theta[0]), r));
  _mm_storeu_pd(&b_args[2], _mm_mul_pd(_mm_loadu_pd(&args->Gamma_theta[2]), r));
  x_vec_idx_0 = (b_args[0] * Phi_idx_0 + b_args[2] * b_s) * x_rDdot;
  sliding = (b_args[1] * Phi_idx_0 + b_args[3] * b_s) * x_rDdot;
  /*  --- Thrust Calculation --- */
  clipped_delta =
      muDoubleScalarMin(muDoubleScalarMax(delta, -args->ThrustSaturation),
                        args->ThrustSaturation);
  /*  Mask updates if saturated */
  b_s = muDoubleScalarAbs(delta);
  maskFT = ((b_s + 0.1) * (b_s + 0.1) < maskFT_tmp);
  Phi_idx_0 = -args->sigmaMRACLin * d;
  MRAC_2_updateParams[0] = maskFT;
  b_s = -args->Gamma_r * b_ds_mrac_1_tmp * x_rDdot;
  MRAC_2_updateParams[5] = b_s;
  MRAC_2_updateParams[6] =
      Phi_idx_0 * args->Gamma_r * b_ds_mrac_1_tmp * delta_tmp;
  /*  Store derivatives */
  ds_mrac_2[2] =
      (b_s - ds_mrac_1_tmp * args->Gamma_r * b_ds_mrac_1_tmp * delta_tmp) *
      (real_T)maskFT;
  /*  d(hhat)/dt */
  /*  d(ahat_1)/dt, d(ahat_2)/dt */
  MRAC_2_updateParams[1] = sliding_tmp;
  MRAC_2_updateParams[7] = x_vec_idx_0;
  MRAC_2_updateParams[3] = Phi_idx_0 * args->Gamma_x[0] * b_delta_tmp +
                           Phi_idx_0 * args->Gamma_x[2] * c_delta_tmp;
  MRAC_2_updateParams[9] = Phi_idx_0 * args->Gamma_theta[0] * Theta_hat_idx_0 +
                           Phi_idx_0 * args->Gamma_theta[2] * Theta_hat_idx_1;
  ds_mrac_2[3] =
      (sliding_tmp - (ds_mrac_1_tmp * args->Gamma_x[0] * b_delta_tmp +
                      ds_mrac_1_tmp * args->Gamma_x[2] * c_delta_tmp)) *
      (real_T)maskFT;
  ds_mrac_2[5] =
      (x_vec_idx_0 - (ds_mrac_1_tmp * args->Gamma_theta[0] * Theta_hat_idx_0 +
                      ds_mrac_1_tmp * args->Gamma_theta[2] * Theta_hat_idx_1)) *
      (real_T)maskFT;
  MRAC_2_updateParams[2] = c_ds_mrac_1_tmp;
  MRAC_2_updateParams[8] = sliding;
  MRAC_2_updateParams[4] = Phi_idx_0 * args->Gamma_x[1] * b_delta_tmp +
                           Phi_idx_0 * args->Gamma_x[3] * c_delta_tmp;
  MRAC_2_updateParams[10] = Phi_idx_0 * args->Gamma_theta[1] * Theta_hat_idx_0 +
                            Phi_idx_0 * args->Gamma_theta[3] * Theta_hat_idx_1;
  ds_mrac_2[4] =
      (c_ds_mrac_1_tmp - (ds_mrac_1_tmp * args->Gamma_x[1] * b_delta_tmp +
                          ds_mrac_1_tmp * args->Gamma_x[3] * c_delta_tmp)) *
      (real_T)maskFT;
  ds_mrac_2[6] =
      (sliding - (ds_mrac_1_tmp * args->Gamma_theta[1] * Theta_hat_idx_0 +
                  ds_mrac_1_tmp * args->Gamma_theta[3] * Theta_hat_idx_1)) *
      (real_T)maskFT;
  /*  d(Theta_hat)/dt */
  ds_mrac_2[7] = x1dot;
  ds_mrac_2[8] = clipped_delta;
  /*  Tracked vars */
  if (args->MRAC_v == 1.0) {
    Fthrust[0] = Fthrust_1[0];
    Fthrust[1] = Fthrust_1[1];
    Fthrust[2] = Fthrust_1[2];
    memcpy(&ds_mrac[0], &ds_mrac_1[0], 9U * sizeof(real_T));
  } else if (args->MRAC_v == 2.0) {
    r = _mm_loadu_pd(&Fthrust_1_tmp[0]);
    _mm_storeu_pd(&Fthrust[0], _mm_mul_pd(_mm_set1_pd(clipped_delta), r));
    Fthrust[2] = clipped_delta * Fthrust_1_tmp[2];
    memcpy(&ds_mrac[0], &ds_mrac_2[0], 9U * sizeof(real_T));
  }
}

/* End of code generation (calculateAdaptiveControl_Unified.c) */
