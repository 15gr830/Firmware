/*
 * File: _coder_posEKF_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 17-May-2015 11:18:31
 */

/* Include files */
#include "_coder_posEKF_api.h"

/* Function Declarations */
static unsigned char (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *zFlag, const char *identifier))[3];
static unsigned char (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3];
static double c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier);
static double d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z, const
  char *identifier))[9];
static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *r_ptam,
  const char *identifier))[3];
static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static const mxArray *emlrt_marshallOut(const float u[9]);
static const mxArray *b_emlrt_marshallOut(const float u[81]);
static const mxArray *c_emlrt_marshallOut(const float u[4]);
static unsigned char (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId))[3];
static double j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static double (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static double (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void posEKF_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void posEKF_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void posEKF_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  posEKF_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[9]
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void posEKF_api(const mxArray *prhs[9], const mxArray *plhs[3])
{
  float (*xa_apo)[9];
  float (*Pa_apo)[81];
  float (*debugOutput)[4];
  unsigned char (*zFlag)[3];
  double dt;
  double (*z)[9];
  double q_acc;
  double q_speed;
  double q_pos;
  double r_acc;
  double (*r_ptam)[3];
  double r_got;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  xa_apo = (float (*)[9])mxMalloc(sizeof(float [9]));
  Pa_apo = (float (*)[81])mxMalloc(sizeof(float [81]));
  debugOutput = (float (*)[4])mxMalloc(sizeof(float [4]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[7] = emlrtProtectR2012b(prhs[7], 7, false, -1);

  /* Marshall function inputs */
  zFlag = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "zFlag");
  dt = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "dt");
  z = e_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "z");
  q_acc = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "q_acc");
  q_speed = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "q_speed");
  q_pos = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "q_pos");
  r_acc = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "r_acc");
  r_ptam = g_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "r_ptam");
  r_got = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "r_got");

  /* Invoke the target function */
  posEKF(*zFlag, dt, *z, q_acc, q_speed, q_pos, r_acc, *r_ptam, r_got, *xa_apo, *
         Pa_apo, *debugOutput);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*xa_apo);
  plhs[1] = b_emlrt_marshallOut(*Pa_apo);
  plhs[2] = c_emlrt_marshallOut(*debugOutput);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *zFlag
 *                const char *identifier
 * Return Type  : unsigned char (*)[3]
 */
static unsigned char (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *zFlag, const char *identifier))[3]
{
  unsigned char (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(zFlag), &thisId);
  emlrtDestroyArray(&zFlag);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : unsigned char (*)[3]
 */
  static unsigned char (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *
  u, const emlrtMsgIdentifier *parentId))[3]
{
  unsigned char (*y)[3];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dt
 *                const char *identifier
 * Return Type  : double
 */
static double c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier)
{
  double y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double
 */
static double d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  double y;
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *z
 *                const char *identifier
 * Return Type  : double (*)[9]
 */
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z, const
  char *identifier))[9]
{
  double (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(z), &thisId);
  emlrtDestroyArray(&z);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[9]
 */
  static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  double (*y)[9];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *r_ptam
 *                const char *identifier
 * Return Type  : double (*)[3]
 */
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *r_ptam,
  const char *identifier))[3]
{
  double (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(r_ptam), &thisId);
  emlrtDestroyArray(&r_ptam);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[3]
 */
  static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  double (*y)[3];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const float u[9])
{
  const mxArray *y;
  static const int iv0[1] = { 0 };

  const mxArray *m0;
  static const int iv1[1] = { 9 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const float u[81]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const float u[81])
{
  const mxArray *y;
  static const int iv2[2] = { 0, 0 };

  const mxArray *m1;
  static const int iv3[2] = { 9, 9 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const float u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const float u[4])
{
  const mxArray *y;
  static const int iv4[1] = { 0 };

  const mxArray *m2;
  static const int iv5[1] = { 4 };

  y = NULL;
  m2 = emlrtCreateNumericArray(1, iv4, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv5, 1);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : unsigned char (*)[3]
 */
static unsigned char (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId))[3]
{
  unsigned char (*ret)[3];
  int iv6[1];
  iv6[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 1U, iv6);
  ret = (unsigned char (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double
 */
  static double j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  double ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, 0);
  ret = *(double *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[9]
 */
static double (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  double (*ret)[9];
  int iv7[1];
  iv7[0] = 9;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv7);
  ret = (double (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[3]
 */
  static double (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  double (*ret)[3];
  int iv8[1];
  iv8[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, iv8);
  ret = (double (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_posEKF_api.c
 *
 * [EOF]
 */
