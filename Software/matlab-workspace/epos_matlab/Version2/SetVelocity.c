/* This a library for communication with Maxon Motors EPOS2 motor controllers
 * using MATLAB.
 *
 * Copyright, Eugenio Yime Rodr�guez, 2015
 *  
 */

#include "mex.h"
#include "Definitions.h"

#ifdef _LINUX_
#include "Win2Linux.h"
#endif

void
mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    DWORD  ErrCode  = 0;
    BOOL   Fault = FALSE;
    HANDLE mHandle;
    WORD   NodeID;
    long   lHandle;
    long   Velocity;
    char   ErrorInfo[255];
    
    /* Examine input (right-hand-side) arguments. */
    if (nrhs != 3) {
        mexPrintf("Error: this function should be use with three input arguments\n");
        return;
    }
    /* Check first input */
    if (mxGetM(prhs[0]) != 1 || mxGetM(prhs[0]) != 1 ) {
       mexPrintf("Error: this function requires three input scalar\n");
       return;
    }
    /* Check second input */
    if (mxGetM(prhs[1]) != 1 || mxGetM(prhs[1]) != 1 ) {
       mexPrintf("Error: this function requires three input scalar\n");
       return;
    }
    /* Check third input */
    if (mxGetM(prhs[2]) != 1 || mxGetM(prhs[2]) != 1 ) {
       mexPrintf("Error: this function requires three input scalar\n");
       return;
    }

    /* Examine output (left-hand-side) arguments. */
    if (nlhs > 1) {
        mexPrintf("Error: this function should be use with only one output argument\n");
        return;
    }
    
    /* create output matrix */
    plhs[0] = mxCreateDoubleScalar(0.0);

    /* first input */
    lHandle = (long) *mxGetPr(prhs[0]);
    mHandle = LongToHandle(lHandle);
    /* second input */
    NodeID = (WORD) *mxGetPr(prhs[1]);
    /* third input */
    Velocity = (long) (mxGetScalar(prhs[2]));
    
    /* Calling function */
    if (!VCS_SetVelocityMust(mHandle, NodeID, Velocity, &ErrCode)) {
        VCS_GetErrorInfo(ErrCode, ErrorInfo, 255);
        mexPrintf("Error: %s \n", ErrorInfo);
        *mxGetPr(plhs[0]) = 1;
    }
}
