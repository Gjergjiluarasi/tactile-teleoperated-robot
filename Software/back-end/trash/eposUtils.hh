#ifndef EPOSUTILS_HH
#define EPOSUTILS_HH

// System libraries
#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

// Custom headers
#include "Definitions.h"

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

typedef void* HANDLE;
typedef int BOOL;


void  LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(std::string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   Demo(unsigned int* p_pErrorCode);
int   PrepareDemo(unsigned int* p_pErrorCode);
int   PrintAvailableInterfaces();
int	  PrintAvailablePorts(char* p_pInterfaceNameSel);
int	  PrintAvailableProtocols();
int   PrintDeviceVersion();

#endif