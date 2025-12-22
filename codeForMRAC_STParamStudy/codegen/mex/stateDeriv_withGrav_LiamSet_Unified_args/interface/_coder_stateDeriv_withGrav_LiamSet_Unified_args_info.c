/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_stateDeriv_withGrav_LiamSet_Unified_args_info.c
 *
 * Code generation for function 'stateDeriv_withGrav_LiamSet_Unified_args'
 *
 */

/* Include files */
#include "_coder_stateDeriv_withGrav_LiamSet_Unified_args_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789cc553414ec240141d0c1237282bd79cc08daedcd522c6d80ad2e2426bcad00e74b4ed"
      "3433d30a0b136fe009f40c1ec1a587f030164a4b219994a411ffe6f7"
      "e565e6bdfffe14542ed50a00601f24f5594b7a7d811b8bbe03566b9daf087a5abba0ba72"
      "2ee5df16dd223e47139e001f7a283b69130ffbd0e7fa3440802246dc",
      "08d97366845da4630f6979703d435e3b47656046cdbe6507594f5ae801eab0a543370fb2"
      "3cbe05f35637cc632cc8a3b1c6df9f3fb44e0d596d2a70c80c160e75"
      "c41d446f098d6351255d91ce0c8bd8a84da8da936453d3bb90424fe3a13d3518871cb510"
      "c591f98cb9734161642a38661137fb3e1e61649b908ed99197cdf55a",
      "72ae66c15c296f41d70addd89e64c380e308c9f1a22971535fa99f41493f35a19f84619c"
      "86165feef5aba49e29d45be54bedb528bcdc3e0702bf071bce23fa6f"
      "eb606fde43f633a7b6a5f7f2fe41b6a997d67fe94d04f76dfa1e0f057a8d35fe1807779d"
      "a07772a3b425e7f1ca97a69dbedb5efae816e814f90002fcd7f7ff02",
      "f68195cf", ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1736U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 3);
  emlrtSetField(
      xEntryPoints, 0, "QualifiedName",
      emlrtMxCreateString("stateDeriv_withGrav_LiamSet_Unified_args"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(3.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "ResolvedFilePath",
      emlrtMxCreateString(
          "D:\\CM "
          "Labs\\subTetherVortexMATLAB\\codeForMRAC_STParamStudy\\stateDeriv_"
          "withGrav_LiamSet_Unified_args.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739973.53996527777));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2712019 (R2024b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("oim0jzKZg9PIcewpY4eVNH"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation
 * (_coder_stateDeriv_withGrav_LiamSet_Unified_args_info.c) */
