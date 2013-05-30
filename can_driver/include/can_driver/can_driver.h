#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <libgen.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#ifdef __WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <process.h>
#include <setupapi.h>
#else
#include <pthread.h>
#include <unistd.h>
#include <dirent.h>
#endif

#ifdef __WIN32
#define MUTEX HANDLE
#else
#define MUTEX pthread_mutex_t
#endif

#include <can_driver/bdc-comm.h>
#include "can_driver/can_proto.h"
#include "can_driver/cmdline.h"
#include "can_driver/os.h"
#include "can_driver/uart_handler.h"
#include "can_driver/board_stat.h"
#include "can_driver/fast-comm.h"

//*****************************************************************************
//
// The maximum number of CAN IDs that can be on the network.
//
//*****************************************************************************
#define MAX_CAN_ID        64

//*****************************************************************************
//
// The current UART state and its global variable.
//
//*****************************************************************************
#define UART_STATE_IDLE         0
#define UART_STATE_LENGTH       1
#define UART_STATE_DATA         2
#define UART_STATE_ESCAPE       3

//*****************************************************************************
//
// This is used to range check values from the console. It should be set to the
// last valid PSTAT message byte ID (in can_proto.h).
//
//*****************************************************************************
#define PSTATUS_MAX_ID  LM_PSTAT_CANERR_B1

//*****************************************************************************
//
// Define the scale factors for the position and voltage values
//
//*****************************************************************************
#define POS_SCALE_FACTOR 65536
#define MAX_VOUT_SCALE_FACTOR 3072


#endif
