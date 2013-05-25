#ifndef CART_SENSORS_H
#define CART_SENSORS_H

#include "ros/ros.h"
#include "cart_sensors/Encoder.h"
#include <libgen.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <signal.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

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

#include "cart_sensors/os.h"

#endif
