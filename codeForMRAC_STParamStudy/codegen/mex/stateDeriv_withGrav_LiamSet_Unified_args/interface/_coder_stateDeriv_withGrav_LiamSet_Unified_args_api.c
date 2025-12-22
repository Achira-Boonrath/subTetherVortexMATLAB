/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_stateDeriv_withGrav_LiamSet_Unified_args_api.c
 *
 * Code generation for function
 * '_coder_stateDeriv_withGrav_LiamSet_Unified_args_api'
 *
 */

/* Include files */
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_api.h"
#include "rt_nonfinite.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_data.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_emxutil.h"
#include "stateDeriv_withGrav_LiamSet_Unified_args_types.h"

/* Variable Definitions */
static emlrtRTEInfo qb_emlrtRTEI = {
    1,                                                     /* lineNo */
    1,                                                     /* colNo */
    "_coder_stateDeriv_withGrav_LiamSet_Unified_args_api", /* fName */
    ""                                                     /* pName */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[95];

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[95];

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(emxArray_real_T *u);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T *y);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4]);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[2]);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9]);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[5]);

static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[95];

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4]);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[2]);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9]);

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[5]);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[95]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[95];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[95]
{
  real_T(*y)[95];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(emxArray_real_T *u)
{
  static const int32_T i = 0;
  const mxArray *m;
  const mxArray *y;
  real_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 1);
  u->canFreeData = false;
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct0_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[37] = {"N_mt_nodes",
                                         "L0",
                                         "kA",
                                         "gamma",
                                         "sigmaMRAC_h",
                                         "sigmaMRAC_a1",
                                         "sigmaMRAC_a2",
                                         "Gamma_x",
                                         "Gamma_r",
                                         "Gamma_theta",
                                         "P",
                                         "B_linear",
                                         "sigmaMRACLin",
                                         "massPoint1",
                                         "targetM",
                                         "targetI",
                                         "chaserM",
                                         "chaserI",
                                         "chaserSideLength",
                                         "targetSideLengthX",
                                         "targetSideLengthY",
                                         "targetSideLengthZ",
                                         "Kvec",
                                         "l0vec",
                                         "cVec",
                                         "mu",
                                         "FT",
                                         "x1_m0",
                                         "tOnChaser_fromt0",
                                         "slopeTime",
                                         "desElong",
                                         "ThrustSaturation",
                                         "Kp",
                                         "Kd",
                                         "J2on",
                                         "MRAC_v",
                                         "ODEscale"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 37,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "N_mt_nodes";
  y->N_mt_nodes = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "N_mt_nodes")),
      &thisId);
  thisId.fIdentifier = "L0";
  y->L0 = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "L0")),
      &thisId);
  thisId.fIdentifier = "kA";
  y->kA = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 2, "kA")),
      &thisId);
  thisId.fIdentifier = "gamma";
  y->gamma = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 3, "gamma")),
      &thisId);
  thisId.fIdentifier = "sigmaMRAC_h";
  y->sigmaMRAC_h =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        4, "sigmaMRAC_h")),
                         &thisId);
  thisId.fIdentifier = "sigmaMRAC_a1";
  y->sigmaMRAC_a1 =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        5, "sigmaMRAC_a1")),
                         &thisId);
  thisId.fIdentifier = "sigmaMRAC_a2";
  y->sigmaMRAC_a2 =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        6, "sigmaMRAC_a2")),
                         &thisId);
  thisId.fIdentifier = "Gamma_x";
  g_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 7, "Gamma_x")),
      &thisId, y->Gamma_x);
  thisId.fIdentifier = "Gamma_r";
  y->Gamma_r = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 8, "Gamma_r")),
      &thisId);
  thisId.fIdentifier = "Gamma_theta";
  g_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 9,
                                                    "Gamma_theta")),
                     &thisId, y->Gamma_theta);
  thisId.fIdentifier = "P";
  g_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 10, "P")),
      &thisId, y->P);
  thisId.fIdentifier = "B_linear";
  h_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 11, "B_linear")),
      &thisId, y->B_linear);
  thisId.fIdentifier = "sigmaMRACLin";
  y->sigmaMRACLin =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        12, "sigmaMRACLin")),
                         &thisId);
  thisId.fIdentifier = "massPoint1";
  y->massPoint1 =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        13, "massPoint1")),
                         &thisId);
  thisId.fIdentifier = "targetM";
  y->targetM = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 14, "targetM")),
      &thisId);
  thisId.fIdentifier = "targetI";
  i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 15, "targetI")),
      &thisId, y->targetI);
  thisId.fIdentifier = "chaserM";
  y->chaserM = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 16, "chaserM")),
      &thisId);
  thisId.fIdentifier = "chaserI";
  i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 17, "chaserI")),
      &thisId, y->chaserI);
  thisId.fIdentifier = "chaserSideLength";
  y->chaserSideLength =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 18, "chaserSideLength")),
                         &thisId);
  thisId.fIdentifier = "targetSideLengthX";
  y->targetSideLengthX =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 19, "targetSideLengthX")),
                         &thisId);
  thisId.fIdentifier = "targetSideLengthY";
  y->targetSideLengthY =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 20, "targetSideLengthY")),
                         &thisId);
  thisId.fIdentifier = "targetSideLengthZ";
  y->targetSideLengthZ =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 21, "targetSideLengthZ")),
                         &thisId);
  thisId.fIdentifier = "Kvec";
  j_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 22, "Kvec")),
      &thisId, y->Kvec);
  thisId.fIdentifier = "l0vec";
  j_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 23, "l0vec")),
      &thisId, y->l0vec);
  thisId.fIdentifier = "cVec";
  j_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 24, "cVec")),
      &thisId, y->cVec);
  thisId.fIdentifier = "mu";
  y->mu = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 25, "mu")),
      &thisId);
  thisId.fIdentifier = "FT";
  y->FT = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 26, "FT")),
      &thisId);
  thisId.fIdentifier = "x1_m0";
  y->x1_m0 = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 27, "x1_m0")),
      &thisId);
  thisId.fIdentifier = "tOnChaser_fromt0";
  y->tOnChaser_fromt0 =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 28, "tOnChaser_fromt0")),
                         &thisId);
  thisId.fIdentifier = "slopeTime";
  y->slopeTime = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 29, "slopeTime")),
      &thisId);
  thisId.fIdentifier = "desElong";
  y->desElong = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 30, "desElong")),
      &thisId);
  thisId.fIdentifier = "ThrustSaturation";
  y->ThrustSaturation =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 31, "ThrustSaturation")),
                         &thisId);
  thisId.fIdentifier = "Kp";
  y->Kp = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 32, "Kp")),
      &thisId);
  thisId.fIdentifier = "Kd";
  y->Kd = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 33, "Kd")),
      &thisId);
  thisId.fIdentifier = "J2on";
  y->J2on = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 34, "J2on")),
      &thisId);
  thisId.fIdentifier = "MRAC_v";
  y->MRAC_v = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 35, "MRAC_v")),
      &thisId);
  thisId.fIdentifier = "ODEscale";
  y->ODEscale = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 36, "ODEscale")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4])
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[2])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[5])
{
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[95]
{
  static const int32_T dims = 95;
  real_T(*ret)[95];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[95])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims[2] = {2, 2};
  real_T(*r)[4];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[2])
{
  static const int32_T dims = 2;
  real_T(*r)[2];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[2])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  emlrtDestroyArray(&src);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2] = {3, 3};
  real_T(*r)[9];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[9])emlrtMxGetData(src);
  for (i = 0; i < 9; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[5])
{
  static const int32_T dims[2] = {1, 5};
  real_T(*r)[5];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[5])emlrtMxGetData(src);
  for (i = 0; i < 5; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

void c_stateDeriv_withGrav_LiamSet_U(const mxArray *const prhs[3],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_real_T *ds;
  const mxArray *prhs_copy_idx_1;
  struct0_T args;
  real_T(*s)[95];
  real_T t;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1], 1, false, -1);
  /* Marshall function inputs */
  t = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "t");
  s = c_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "s");
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "args", &args);
  /* Invoke the target function */
  emxInit_real_T(&st, &ds, 1, &qb_emlrtRTEI);
  stateDeriv_withGrav_LiamSet_Unified_args(&st, t, *s, &args, ds);
  /* Marshall function outputs */
  ds->canFreeData = false;
  *plhs = emlrt_marshallOut(ds);
  emxFree_real_T(&st, &ds);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation
 * (_coder_stateDeriv_withGrav_LiamSet_Unified_args_api.c) */
