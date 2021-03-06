
#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include "ros/ros.h"
#include "drive_train/CartDrive.h"
#include "sensor_msgs/Joy.h"
#include "cart_sensors/EngageAuto.h"
#include "cart_sensors/Encoder.h"
#include "map_lin_moves.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61

#define MAX_VOUT_PERC 1.0 

#define POT_MAX_TURNS 10
#define POTENTIOMETER_REF 1
#define POS_START_REF 5.0
#define POS_LEFT_STOP 8.0
#define POS_RIGHT_STOP 2.0

#define MAX_THROT_POS 100.0
#define MIN_THROT_POS 70.0

int readTable(char *fileName, int *numSteps, int *numCols, double **goalMap);

#endif
