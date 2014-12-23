// Simple interface to simulink mainly to provide easy HIL testing using the turning table
// 20110718, SH
//
// Should be compiled from Matlab using script make.m
//
//

# include "rhdlink.h"

# define S_FUNCTION_NAME rhdlink
# define S_FUNCTION_LEVEL 2

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
# include "simstruc.h"

// S-function parameters
#define NUM_PARAMS (3)
#define TS_PARAM (ssGetSFcnParam(s, 0))
#define HOSTNAME_PARAM (ssGetSFcnParam(s, 1))
#define PORT_PARAM (ssGetSFcnParam(s, 2))

// Macros to access the S-function parameter values
#define SAMPLE_TIME (mxGetPr(TS_PARAM)[0])

// *====================*
// * S-function methods *
// *====================*

static void mdlInitializeSizes(SimStruct *s)
{
  int i;
  rhd_data *rd = new rhd_data();
  
  ssSetNumSFcnParams(s, 3);
  
  /*
   * If the the number of expected input parameters is not equal
   * to the number of parameters entered in the dialog box return.
   * Simulink will generate an error indicating that there is a
   * parameter mismatch.
   */
  if(ssGetNumSFcnParams(s) != ssGetSFcnParamsCount(s)) 
    return;
    
  ssSetNumContStates(s, 0);
  ssSetNumDiscStates(s, 0);
  
  // If no input port is found
  if(!ssSetNumInputPorts(s, rd->input_num)) 
    return;
  
  for(i = 0; i < rd->input_num; i++)
  {
    ssSetInputPortWidth(s, i, rd->input_sig[i].size);
    ssSetInputPortRequiredContiguous(s, i, true);
    ssSetInputPortDirectFeedThrough(s, i, 1);
  }
  
  if(!ssSetNumOutputPorts(s, rd->output_num))
    return;
  
  for(i = 0; i < rd->output_num; i++)
    ssSetOutputPortWidth(s, i, rd->output_sig[i].size);
  
  ssSetNumSampleTimes(s, 1);
  ssSetNumRWork(s, 0);
  ssSetNumIWork(s, 0);
  ssSetNumPWork(s, 1);
  ssSetNumModes(s, 0);
  ssSetNumNonsampledZCs(s, 0);
  ssSetOptions(s, 0);
}

static void mdlInitializeSampleTimes(SimStruct *s)
{
  ssSetSampleTime(s, 0, SAMPLE_TIME);
  ssSetOffsetTime(s, 0, 0.0);
}

# define MDL_START
static void mdlStart(SimStruct *s)
{
  char hostname[NAMELENGTH];
  char status[NAMELENGTH];
  unsigned int port = 24902;

  // The rhdsock pointer points to one of the persistent variables
  ssGetPWork(s)[0] = (void *) new rhd_data;
	
  // Read the host name parameter
  mxGetString(HOSTNAME_PARAM, hostname, NAMELENGTH);

  // Read the port number
  port = (unsigned int)(mxGetPr(PORT_PARAM)[0]);
	
  // Connect to rhd
  if(rhdConnect('w', hostname, port) > 0)
  {
    rhdSync();
  } 
  else
  {
    sprintf(status, "Could not connect to rhd at %s !", hostname);
    ssSetErrorStatus(s, status);
    return;
  }
}

static void mdlOutputs(SimStruct *s, int_T tid)
{
  int i, j;
  rhd_data *rd;
  real_T *indata, *outdata; // = (const real_T *) ssGetInputPortSignal(s, 0);
  void **PWork = ssGetPWork(s);
  
  rd = (rhd_data *) PWork[0];
  
  for(i = 0; i < rd->input_num; i++)
  {
    indata = (real_T *) ssGetInputPortSignal(s, i);
    for(j = 0; j < rd->input_sig[i].size; j++)
      writeValueNamed((char *) rd->input_sig[i].name, j, (double) indata[j]);
  }

  rhdSync();
  
  for(i = 0; i < rd->output_num; i++)
  {
    outdata = (real_T *) ssGetOutputPortRealSignal(s, i);
    for(j = 0; j < rd->output_sig[i].size; j++)
    {
      outdata[j] = readValueNamed((char *) rd->output_sig[i].name, j);
    }
  }
}

static void mdlTerminate(SimStruct *s)
{
  int i, j;
  rhd_data *rd;
  void **PWork = ssGetPWork(s);
  
  rd = (rhd_data *) PWork[0];
  
  // Nice shutdown:
  for(i = 0; i < rd->input_num; i++)
  {
    for(j = 0; j < rd->input_sig[i].size; j++)
      writeValueNamed((char *) rd->input_sig[i].name, j, 0);
  }

  rhdSync();
  
  rhdDisconnect();
  
  free(PWork[0]);
}

// Required S-function trailer
# ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
# include "simulink.c"      /* MEX-file interface mechanism */
# else
# include "cg_sfun.h"       /* Code generation registration function */
# endif
