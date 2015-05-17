/*
 * _coder_AttitudeEKF2grav_mex.c
 *
 * Code generation for function 'AttitudeEKF2grav'
 *
 */

/* Include files */
#include "mex.h"
#include "_coder_AttitudeEKF2grav_api.h"

/* Function Declarations */
static void AttitudeEKF2grav_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/* Variable Definitions */
emlrtContext emlrtContextGlobal = { true, false, EMLRT_VERSION_INFO, NULL, "AttitudeEKF2grav", NULL, false, {2045744189U,2170104910U,2743257031U,4284093946U}, NULL };
void *emlrtRootTLSGlobal = NULL;

/* Function Definitions */
static void AttitudeEKF2grav_mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *outputs[6];
  const mxArray *inputs[13];
  int n = 0;
  int nOutputs = (nlhs < 1 ? 1 : nlhs);
  int nInputs = nrhs;
  emlrtStack st = { NULL, NULL, NULL };
  /* Module initialization. */
  AttitudeEKF2grav_initialize(&emlrtContextGlobal);
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 13) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, mxINT32_CLASS, 13, mxCHAR_CLASS, 16, "AttitudeEKF2grav");
  } else if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, mxCHAR_CLASS, 16, "AttitudeEKF2grav");
  }
  /* Temporary copy for mex inputs. */
  for (n = 0; n < nInputs; ++n) {
    inputs[n] = prhs[n];
  }
  /* Call the function. */
  AttitudeEKF2grav_api(inputs, outputs);
  /* Copy over outputs to the caller. */
  for (n = 0; n < nOutputs; ++n) {
    plhs[n] = emlrtReturnArrayR2009a(outputs[n]);
  }
  /* Module finalization. */
  AttitudeEKF2grav_terminate();
}

void AttitudeEKF2grav_atexit_wrapper(void)
{
   AttitudeEKF2grav_atexit();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  /* Initialize the memory manager. */
  mexAtExit(AttitudeEKF2grav_atexit_wrapper);
  /* Dispatch the entry-point. */
  AttitudeEKF2grav_mexFunction(nlhs, plhs, nrhs, prhs);
}
/* End of code generation (_coder_AttitudeEKF2grav_mex.c) */
