#ifndef EPOS_MOTOR_CONTROL_UTILS_H
#define EPOS_MOTOR_CONTROL_UTILS_H

// ------------- PLEASE CHOOSE ONLY ONE OF THE LIBRARIES BELOW! -------------  

#define USE_ETHERCAT_ECRT_LIB 1
#define USE_USB_EPOS_CMD_LIB 0

/****************************************************************************/
/****************************************************************************/

#if USE_ETHERCAT_ECRT_LIB

/****************************************************************************/

// System libraries
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */

#include <stdlib.h>
#include <string>

/****************************************************************************/

// Custom headers
#include "ecrt.h"

/****************************************************************************/

// Preprocessor defines

// Default parameters
#define SDO_ACCESS          1
#define PERIOD_NS           (1000000) /** Task period in ns. */
#define MAX_SAFE_STACK      (8 * 1024) /* The maximum stack size which is guranteed safe to access without faulting */
#define NSEC_PER_SEC        (1000000000) /* Nano seconds per second */
#define FREQUENCY           (NSEC_PER_SEC / PERIOD_NS) /* Frequency */
#define DEFAULT_EPOS_POSITION 0
#define DEFAULT_EPOS_VELOCITY 0
#define DEFAULT_EPOS_CURRENT 0

// Slave alias and position in the bus
#define Epos0Pos  0, 0
#define Epos1Pos  0, 1
#define Epos2Pos  0, 2
#define Epos3Pos  0, 3

// Vendor ID and product code
#define EPOS4 0x000000fb, 0x61500000

// RxPDO
#define CONTROL_WORD 0x6040, 0x00
#define TARGET_POSITION 0x607A, 0x00
#define PROFILE_ACCELERATION 0x6083, 0x00
#define PROFILE_DECELERATION 0x6084, 0x00
#define PROFILE_VELOCITY 0x6081, 0x00
#define MODES_OF_OPEATION 0x6060, 0x00
#define DIGITAL_OUTPUT 0x60FE, 0x01

// TxPDO
#define STATUS_WORD 0x6041, 0x00
#define POSITION_ACTUAL_VALUE 0x6064, 0x00
#define VELOCITY_ACTUAL_VALUE 0x606C, 0x00
#define FOLLOWING_ERROR_ACTUAL_VALUE 0x60F4, 0x00
#define MODES_OF_OPERATION_DISPLAY 0x6061, 0x00
#define DIGITAL_INPUT 0x60FD, 0x00

/****************************************************************************/

// Global variables

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_epos0 = NULL;
static ec_slave_config_state_t sc_epos0_state = {};
static ec_slave_config_t *sc_epos1 = NULL;
static ec_slave_config_state_t sc_epos1_state = {};
static ec_slave_config_t *sc_epos2 = NULL;
static ec_slave_config_state_t sc_epos2_state = {};
// static ec_slave_config_t *sc_epos3 = NULL;
// static ec_slave_config_state_t sc_epos3_state = {};

// struct timespec wakeup_time;

// process data
static uint8_t *domain1_pd = NULL;

// Epos0
// offsets for RxPDO entries
static unsigned int off_epos0_CONTROL_WORD;
static unsigned int off_epos0_TARGET_POSITION;
static unsigned int off_epos0_PROFILE_ACCELERATION;
static unsigned int off_epos0_PROFILE_DECELERATION;
static unsigned int off_epos0_PROFILE_VELOCITY;
static unsigned int off_epos0_MODES_OF_OPEATION;
static unsigned int off_epos0_DIGITAL_OUTPUT;

// offsets for TxPDO entries
static unsigned int off_epos0_STATUS_WORD;
static unsigned int off_epos0_POSITION_ACTUAL_VALUE;
static unsigned int off_epos0_VELOCITY_ACTUAL_VALUE;
static unsigned int off_epos0_FOLLOWING_ERROR_ACTUAL_VALUE;
static unsigned int off_epos0_MODES_OF_OPERATION_DISPLAY;
static unsigned int off_epos0_DIGITAL_INPUT;

// Epos1
// offsets for RxPDO entries
static unsigned int off_epos1_CONTROL_WORD;
static unsigned int off_epos1_TARGET_POSITION;
static unsigned int off_epos1_PROFILE_ACCELERATION;
static unsigned int off_epos1_PROFILE_DECELERATION;
static unsigned int off_epos1_PROFILE_VELOCITY;
static unsigned int off_epos1_MODES_OF_OPEATION;
static unsigned int off_epos1_DIGITAL_OUTPUT;

// offsets for TxPDO entries
static unsigned int off_epos1_STATUS_WORD;
static unsigned int off_epos1_POSITION_ACTUAL_VALUE;
static unsigned int off_epos1_VELOCITY_ACTUAL_VALUE;
static unsigned int off_epos1_FOLLOWING_ERROR_ACTUAL_VALUE;
static unsigned int off_epos1_MODES_OF_OPERATION_DISPLAY;
static unsigned int off_epos1_DIGITAL_INPUT;

// Epos2
// offsets for RxPDO entries
static unsigned int off_epos2_CONTROL_WORD;
static unsigned int off_epos2_TARGET_POSITION;
static unsigned int off_epos2_PROFILE_ACCELERATION;
static unsigned int off_epos2_PROFILE_DECELERATION;
static unsigned int off_epos2_PROFILE_VELOCITY;
static unsigned int off_epos2_MODES_OF_OPEATION;
static unsigned int off_epos2_DIGITAL_OUTPUT;

// offsets for TxPDO entries
static unsigned int off_epos2_STATUS_WORD;
static unsigned int off_epos2_POSITION_ACTUAL_VALUE;
static unsigned int off_epos2_VELOCITY_ACTUAL_VALUE;
static unsigned int off_epos2_FOLLOWING_ERROR_ACTUAL_VALUE;
static unsigned int off_epos2_MODES_OF_OPERATION_DISPLAY;
static unsigned int off_epos2_DIGITAL_INPUT;

// // Epos3
// // offsets for RxPDO entries
// static unsigned int off_epos3_CONTROL_WORD;
// static unsigned int off_epos3_TARGET_POSITION;
// static unsigned int off_epos3_PROFILE_ACCELERATION;
// static unsigned int off_epos3_PROFILE_DECELERATION;
// static unsigned int off_epos3_PROFILE_VELOCITY;
// static unsigned int off_epos3_MODES_OF_OPEATION;
// static unsigned int off_epos3_DIGITAL_OUTPUT;

// // offsets for TxPDO entries
// static unsigned int off_epos3_STATUS_WORD;
// static unsigned int off_epos3_POSITION_ACTUAL_VALUE;
// static unsigned int off_epos3_VELOCITY_ACTUAL_VALUE;
// static unsigned int off_epos3_FOLLOWING_ERROR_ACTUAL_VALUE;
// static unsigned int off_epos3_MODES_OF_OPERATION_DISPLAY;
// static unsigned int off_epos3_DIGITAL_INPUT;

// Domain 1 PDOs
const static ec_pdo_entry_reg_t domain1_regs[] = {
    {Epos0Pos,  EPOS4, CONTROL_WORD, &off_epos0_CONTROL_WORD},
    {Epos0Pos,  EPOS4, TARGET_POSITION, &off_epos0_TARGET_POSITION},
    {Epos0Pos,  EPOS4, PROFILE_ACCELERATION, &off_epos0_PROFILE_ACCELERATION},
    {Epos0Pos,  EPOS4, PROFILE_DECELERATION, &off_epos0_PROFILE_DECELERATION},
    {Epos0Pos,  EPOS4, PROFILE_VELOCITY, &off_epos0_PROFILE_VELOCITY},
    {Epos0Pos,  EPOS4, MODES_OF_OPEATION, &off_epos0_MODES_OF_OPEATION},
    {Epos0Pos,  EPOS4, DIGITAL_OUTPUT, &off_epos0_DIGITAL_OUTPUT},
    {Epos0Pos,  EPOS4, STATUS_WORD, &off_epos0_STATUS_WORD},
    {Epos0Pos,  EPOS4, POSITION_ACTUAL_VALUE, &off_epos0_POSITION_ACTUAL_VALUE},
    {Epos0Pos,  EPOS4, VELOCITY_ACTUAL_VALUE, &off_epos0_VELOCITY_ACTUAL_VALUE},
    {Epos0Pos,  EPOS4, FOLLOWING_ERROR_ACTUAL_VALUE, &off_epos0_FOLLOWING_ERROR_ACTUAL_VALUE},
    {Epos0Pos,  EPOS4, MODES_OF_OPERATION_DISPLAY, &off_epos0_MODES_OF_OPERATION_DISPLAY},
    {Epos0Pos,  EPOS4, DIGITAL_INPUT, &off_epos0_DIGITAL_INPUT},
    {Epos1Pos,  EPOS4, CONTROL_WORD, &off_epos1_CONTROL_WORD},
    {Epos1Pos,  EPOS4, TARGET_POSITION, &off_epos1_TARGET_POSITION},
    {Epos1Pos,  EPOS4, PROFILE_ACCELERATION, &off_epos1_PROFILE_ACCELERATION},
    {Epos1Pos,  EPOS4, PROFILE_DECELERATION, &off_epos1_PROFILE_DECELERATION},
    {Epos1Pos,  EPOS4, PROFILE_VELOCITY, &off_epos1_PROFILE_VELOCITY},
    {Epos1Pos,  EPOS4, MODES_OF_OPEATION, &off_epos1_MODES_OF_OPEATION},
    {Epos1Pos,  EPOS4, DIGITAL_OUTPUT, &off_epos1_DIGITAL_OUTPUT},
    {Epos1Pos,  EPOS4, STATUS_WORD, &off_epos1_STATUS_WORD},
    {Epos1Pos,  EPOS4, POSITION_ACTUAL_VALUE, &off_epos1_POSITION_ACTUAL_VALUE},
    {Epos1Pos,  EPOS4, VELOCITY_ACTUAL_VALUE, &off_epos1_VELOCITY_ACTUAL_VALUE},
    {Epos1Pos,  EPOS4, FOLLOWING_ERROR_ACTUAL_VALUE, &off_epos1_FOLLOWING_ERROR_ACTUAL_VALUE},
    {Epos1Pos,  EPOS4, MODES_OF_OPERATION_DISPLAY, &off_epos1_MODES_OF_OPERATION_DISPLAY},
    {Epos1Pos,  EPOS4, DIGITAL_INPUT, &off_epos1_DIGITAL_INPUT},
    {Epos2Pos,  EPOS4, CONTROL_WORD, &off_epos2_CONTROL_WORD},
    {Epos2Pos,  EPOS4, TARGET_POSITION, &off_epos2_TARGET_POSITION},
    {Epos2Pos,  EPOS4, PROFILE_ACCELERATION, &off_epos2_PROFILE_ACCELERATION},
    {Epos2Pos,  EPOS4, PROFILE_DECELERATION, &off_epos2_PROFILE_DECELERATION},
    {Epos2Pos,  EPOS4, PROFILE_VELOCITY, &off_epos2_PROFILE_VELOCITY},
    {Epos2Pos,  EPOS4, MODES_OF_OPEATION, &off_epos2_MODES_OF_OPEATION},
    {Epos2Pos,  EPOS4, DIGITAL_OUTPUT, &off_epos2_DIGITAL_OUTPUT},
    {Epos2Pos,  EPOS4, STATUS_WORD, &off_epos2_STATUS_WORD},
    {Epos2Pos,  EPOS4, POSITION_ACTUAL_VALUE, &off_epos2_POSITION_ACTUAL_VALUE},
    {Epos2Pos,  EPOS4, VELOCITY_ACTUAL_VALUE, &off_epos2_VELOCITY_ACTUAL_VALUE},
    {Epos2Pos,  EPOS4, FOLLOWING_ERROR_ACTUAL_VALUE, &off_epos2_FOLLOWING_ERROR_ACTUAL_VALUE},
    {Epos2Pos,  EPOS4, MODES_OF_OPERATION_DISPLAY, &off_epos2_MODES_OF_OPERATION_DISPLAY},
    {Epos2Pos,  EPOS4, DIGITAL_INPUT, &off_epos2_DIGITAL_INPUT},    
    // {Epos3Pos,  EPOS4, CONTROL_WORD, &off_epos3_CONTROL_WORD},
    // {Epos3Pos,  EPOS4, TARGET_POSITION, &off_epos3_TARGET_POSITION},
    // {Epos3Pos,  EPOS4, PROFILE_ACCELERATION, &off_epos3_PROFILE_ACCELERATION},
    // {Epos3Pos,  EPOS4, PROFILE_DECELERATION, &off_epos3_PROFILE_DECELERATION},
    // {Epos3Pos,  EPOS4, PROFILE_VELOCITY, &off_epos3_PROFILE_VELOCITY},
    // {Epos3Pos,  EPOS4, MODES_OF_OPEATION, &off_epos3_MODES_OF_OPEATION},
    // {Epos3Pos,  EPOS4, DIGITAL_OUTPUT, &off_epos3_DIGITAL_OUTPUT},
    // {Epos3Pos,  EPOS4, STATUS_WORD, &off_epos3_STATUS_WORD},
    // {Epos3Pos,  EPOS4, POSITION_ACTUAL_VALUE, &off_epos3_POSITION_ACTUAL_VALUE},
    // {Epos3Pos,  EPOS4, VELOCITY_ACTUAL_VALUE, &off_epos3_VELOCITY_ACTUAL_VALUE},
    // {Epos3Pos,  EPOS4, FOLLOWING_ERROR_ACTUAL_VALUE, &off_epos3_FOLLOWING_ERROR_ACTUAL_VALUE},
    // {Epos3Pos,  EPOS4, MODES_OF_OPERATION_DISPLAY, &off_epos3_MODES_OF_OPERATION_DISPLAY},
    // {Epos3Pos,  EPOS4, DIGITAL_INPUT, &off_epos3_DIGITAL_INPUT},
    {}
};

// EPOS 0 --------------------------

// PDO entries
static ec_pdo_entry_info_t epos0_pdo_entries[] = {
    {CONTROL_WORD, 16}, // Control word - RXPDO1
    {TARGET_POSITION, 32}, // Target position - RXPDO2
    {PROFILE_ACCELERATION, 32}, // Profile acceleration - RXPDO3
    {PROFILE_DECELERATION, 32}, // Profile deceleration - RXPDO4
    {PROFILE_VELOCITY, 32}, // Profile velocity - RXPDO5
    {MODES_OF_OPEATION, 8}, // Modes of operation - RXPDO6
    {DIGITAL_OUTPUT, 32}, // Digital output - RXPDO7
    {STATUS_WORD, 16},  // Status word - TXPDO1
    {POSITION_ACTUAL_VALUE, 32}, // Position actual value - RXPDO2
    {VELOCITY_ACTUAL_VALUE, 32}, // Velocity actual value - RXPDO3
    {FOLLOWING_ERROR_ACTUAL_VALUE, 32}, // Following error actual value - RXPDO4
    {MODES_OF_OPERATION_DISPLAY, 8}, // Modes of operation display - RXPDO5
    {DIGITAL_INPUT, 32}, // Digital input - RXPDO6
};

// PDOs
static ec_pdo_info_t epos0_pdos[] = {
    {0x1600, 7, epos0_pdo_entries},
    {0x1a00, 6, epos0_pdo_entries+7},
};

// Sync managers
static ec_sync_info_t epos0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, epos0_pdos+0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, epos0_pdos+1, EC_WD_DISABLE},
    {0xff}
};

// EPOS 1 --------------------------

// PDO entries
static ec_pdo_entry_info_t epos1_pdo_entries[] = {
    {CONTROL_WORD, 16}, // Control word - RXPDO1
    {TARGET_POSITION, 32}, // Target position - RXPDO2
    {PROFILE_ACCELERATION, 32}, // Profile acceleration - RXPDO3
    {PROFILE_DECELERATION, 32}, // Profile deceleration - RXPDO4
    {PROFILE_VELOCITY, 32}, // Profile velocity - RXPDO5
    {MODES_OF_OPEATION, 8}, // Modes of operation - RXPDO6
    {DIGITAL_OUTPUT, 32}, // Digital output - RXPDO7
    {STATUS_WORD, 16},  // Status word - TXPDO1
    {POSITION_ACTUAL_VALUE, 32}, // Position actual value - RXPDO2
    {VELOCITY_ACTUAL_VALUE, 32}, // Velocity actual value - RXPDO3
    {FOLLOWING_ERROR_ACTUAL_VALUE, 32}, // Following error actual value - RXPDO4
    {MODES_OF_OPERATION_DISPLAY, 8}, // Modes of operation display - RXPDO5
    {DIGITAL_INPUT, 32}, // Digital input - RXPDO6
};

// PDOs
static ec_pdo_info_t epos1_pdos[] = {
    {0x1600, 7, epos1_pdo_entries},
    {0x1a00, 6, epos1_pdo_entries+7},
};

// Sync managers
static ec_sync_info_t epos1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, epos1_pdos+0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, epos1_pdos+1, EC_WD_DISABLE},
    {0xff}
};

// EPOS 2 --------------------------

// PDO entries
static ec_pdo_entry_info_t epos2_pdo_entries[] = {
    {CONTROL_WORD, 16}, // Control word - RXPDO1
    {TARGET_POSITION, 32}, // Target position - RXPDO2
    {PROFILE_ACCELERATION, 32}, // Profile acceleration - RXPDO3
    {PROFILE_DECELERATION, 32}, // Profile deceleration - RXPDO4
    {PROFILE_VELOCITY, 32}, // Profile velocity - RXPDO5
    {MODES_OF_OPEATION, 8}, // Modes of operation - RXPDO6
    {DIGITAL_OUTPUT, 32}, // Digital output - RXPDO7
    {STATUS_WORD, 16},  // Status word - TXPDO1
    {POSITION_ACTUAL_VALUE, 32}, // Position actual value - RXPDO2
    {VELOCITY_ACTUAL_VALUE, 32}, // Velocity actual value - RXPDO3
    {FOLLOWING_ERROR_ACTUAL_VALUE, 32}, // Following error actual value - RXPDO4
    {MODES_OF_OPERATION_DISPLAY, 8}, // Modes of operation display - RXPDO5
    {DIGITAL_INPUT, 32}, // Digital input - RXPDO6
};

// PDOs
static ec_pdo_info_t epos2_pdos[] = {
    {0x1600, 7, epos2_pdo_entries},
    {0x1a00, 6, epos2_pdo_entries+7},
};

// Sync managers
static ec_sync_info_t epos2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, epos2_pdos+0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, epos2_pdos+1, EC_WD_DISABLE},
    {0xff}
};

// // EPOS 3 --------------------------

// // PDO entries
// static ec_pdo_entry_info_t epos3_pdo_entries[] = {
//     {CONTROL_WORD, 16}, // Control word - RXPDO1
//     {TARGET_POSITION, 32}, // Target position - RXPDO2
//     {PROFILE_ACCELERATION, 32}, // Profile acceleration - RXPDO3
//     {PROFILE_DECELERATION, 32}, // Profile deceleration - RXPDO4
//     {PROFILE_VELOCITY, 32}, // Profile velocity - RXPDO5
//     {MODES_OF_OPEATION, 8}, // Modes of operation - RXPDO6
//     {DIGITAL_OUTPUT, 32}, // Digital output - RXPDO7
//     {STATUS_WORD, 16},  // Status word - TXPDO1
//     {POSITION_ACTUAL_VALUE, 32}, // Position actual value - RXPDO2
//     {VELOCITY_ACTUAL_VALUE, 32}, // Velocity actual value - RXPDO3
//     {FOLLOWING_ERROR_ACTUAL_VALUE, 32}, // Following error actual value - RXPDO4
//     {MODES_OF_OPERATION_DISPLAY, 8}, // Modes of operation display - RXPDO5
//     {DIGITAL_INPUT, 32}, // Digital input - RXPDO6
// };

// // PDOs
// static ec_pdo_info_t epos3_pdos[] = {
//     {0x1600, 7, epos3_pdo_entries},
//     {0x1a00, 6, epos3_pdo_entries+7},
// };

// // Sync managers
// static ec_sync_info_t epos3_syncs[] = {
//     {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
//     {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
//     {2, EC_DIR_OUTPUT, 1, epos3_pdos+0, EC_WD_ENABLE},
//     {3, EC_DIR_INPUT, 1, epos3_pdos+1, EC_WD_DISABLE},
//     {0xff}
// };

/****************************************************************************/

// Emumerations
typedef enum{
    SUCCESS_INIT_M,
    ERROR_INIT_M
}initMotorsResult;

typedef enum{
    SUCCESS_SET_C,
    ERROR_SET_C
}setCommandResult;

typedef enum{
    RUNNING,
    ERROR_INIT_MOTORS,
    ERROR_SET_COMMAND
}runProgramResult;

/****************************************************************************/

// Structs
typedef struct MotorControllerParameters{
    long position;
    long velocity;
    long current;
}MotorControllerParameters;

/****************************************************************************/

// Functions 
void check_domain1_state(void);

void check_master_state(void);

void check_slave_config_states(void);

#if SDO_ACCESS
// void read_sdo(void);
// void write_sdo(void);
#endif

void clearInitialErrors(void);

void stack_prefault(void);

setCommandResult setProfilePosition(struct timespec *wakeup_time, unsigned int *counter);

initMotorsResult initMotors(void);

runProgramResult runProgram(void);

#endif

/****************************************************************************/
/****************************************************************************/

#if USE_USB_EPOS_CMD_LIB

// System libraries
#include <iostream>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>

// Custom headers
#include "Definitions.h"

// Default paramters
#define DEFAULT_EPOS_NODE_ID 1
#define DEFAULT_EPOS_DEVICE_NAME "EPOS4"
#define DEFAULT_EPOS_PROTOCOL_STACK_NAME "MAXON SERIAL V2"
#define DEFAULT_EPOS_INTERFACE_NAME "USB"
#define DEFAULT_EPOS_PORT_NAME "USB0"
#define DEFAULT_EPOS_BAUDRATE 1000000
#define DEFAULT_EPOS_DEVICE_HANDLE NULL
#define DEFAULT_EPOS_POSITION 0
#define DEFAULT_EPOS_VELOCITY 0
#define DEFAULT_EPOS_CURRENT 0

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

typedef struct MotorControllerParameters{
    long nodeID;
    std::string deviceName;
    std::string protocolStackName;
    std::string interfaceName;
    std::string portName;
    int baudrate;
    HANDLE p_deviceHandle;
    long position;
    long velocity;
    long current;
}MotorControllerParameters;

MotorControllerParameters mParams;

// Functions
void initEposMotorController();
void initEposMotorController(MotorControllerParameters& a_mParams);
void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
void LogInfo(std::string message);
int Init();
HANDLE OpenDevice(unsigned int* p_pErrorCode);
int PrepareDevice(unsigned int* p_pErrorCode);
int CloseDevice(unsigned int* p_pErrorCode);
int setPosition(unsigned int & p_rlErrorCode, long a_position);
long getPosition();
int setVelocity(unsigned int & p_rlErrorCode, long a_velocity);
long getVelocity();
int setCurrent(long a_current);
long getCurrent();

#endif

/****************************************************************************/
/****************************************************************************/

#endif