
#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include "ros/ros.h"
#include "drive_train/CartDrive.h"
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

#define posScaleFactor 65536
#define maxVScaleFactor 3072

#define potentiometerTurns 10
#define POTENTIOMETER_REF 1
#define posStartRef 5

#endif
