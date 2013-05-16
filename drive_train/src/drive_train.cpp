/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "drive_train.h"
#include <can_driver/can_driver.h>

std::string g_canCommPort;
std::string g_throttleCommPort;
extern MUTEX mMutex;
int steerID;
int brakeID;
double currentSteerPos;
double currentThrottlePos;
double steerStartPos;
double steerLeftStop;
double steerRightStop;
double steerPVal;
double steerIVal;
double steerDVal;
FILE *throttleDrive;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void driveTrainCallBack(const drive_train::CartDriveConstPtr& msg)
{
  //add the steering to the current direction
  currentSteerPos += msg->steering;

  //check limits
  if (currentSteerPos > steerLeftStop)
     {
     currentSteerPos = steerLeftStop;
      }

  if (currentSteerPos < steerRightStop)
     {
     currentSteerPos = steerRightStop;
     }

  
  //enable the Positon mode
  int32_t inpValue;
  inpValue = (int32_t) ((double) POS_SCALE_FACTOR *currentSteerPos );
  fastCmdPosSet(steerID, inpValue);

  //add the steering to the current direction
  currentThrottlePos += msg->throttle;

  //check limits
  if (currentThrottlePos > MAX_THROT_POS)
     {
     currentThrottlePos = MAX_THROT_POS;
      }

  if (currentThrottlePos < MIN_THROT_POS)
     {
     currentThrottlePos = MIN_THROT_POS;
     }

  if (throttleDrive != NULL)
  {
  fprintf(throttleDrive,"%f\n",currentThrottlePos);
} 

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_INFO("Steering: [%f]", msg->steering);
  ROS_INFO("CurrentPos: [%f]", currentSteerPos);
  ROS_INFO("Throttle: [%f]", msg->throttle);
  ROS_INFO("Brake: [%f]", msg->brake);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "drive_train");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /* Get the parameters needed */
  n.param<std::string>("can_comm_port",g_canCommPort,"/dev/ttyUSB0");
  n.param<std::string>("throttle_comm_port",g_throttleCommPort,"/dev/talos/servo1/position");
  n.param("can_steer_id",steerID,1);
  n.param("can_brake_id",brakeID,2);
  n.param("steer_start_pos",steerStartPos,POS_START_REF);
  n.param("steer_left_stop",steerLeftStop,POS_LEFT_STOP);
  n.param("steer_right_stop",steerRightStop,POS_RIGHT_STOP);
  n.param("steer_P_val",steerPVal,150.0);
  n.param("steer_I_val",steerIVal,0.01);
  n.param("steer_D_val",steerDVal,0.01);

  //
  // Open the COM port.
  //
  if (OpenUART((char *) g_canCommPort.c_str(), 115200)) {
     printf("Failed to configure Host UART\n");
     return (-1);
  }
  g_bConnected = true;

  //
  // Initialize the mutex that restricts access to the COM port.
  //
  MutexInit(&mMutex);

  //
  // Create the heart beat thread.
  //
  OSThreadCreate(HeartbeatThread);

  //
  //initialize the Jaguar motor controller for the steering
  //
  fastConfigTurns(steerID,POT_MAX_TURNS);

  //Configure the max output voltage
  int32_t outputVoltage;
  outputVoltage = (int32_t) ((double) MAX_VOUT_SCALE_FACTOR * (double) MAX_VOUT_PERC);
  fastConfigMaxV(steerID,outputVoltage);

  //set the reference
  fastCmdPosRef(steerID,POTENTIOMETER_REF);

  //set the values or P I and D
  int32_t inpValue;
  inpValue = (int32_t) ((double) POS_SCALE_FACTOR * steerIVal);
  fastCmdPosI(steerID, inpValue);
  inpValue = (int32_t) ((double) POS_SCALE_FACTOR * steerDVal);
  fastCmdPosD(steerID, inpValue);
  inpValue = (int32_t) ((double) POS_SCALE_FACTOR * steerPVal);
  fastCmdPosP(steerID, inpValue);

  //enable the Positon mode
  inpValue = (int32_t) ((double) POS_SCALE_FACTOR * steerStartPos);
  currentSteerPos = steerStartPos;
  fastCmdPosEnable(steerID, inpValue);

  //
  // Open the throttle port
  //
  throttleDrive = fopen(g_throttleCommPort.c_str(),"w");
  if (throttleDrive != NULL)
  {
  setbuf(throttleDrive,NULL);
  currentThrottlePos = MIN_THROT_POS;
  fprintf(throttleDrive,"%f\n",currentThrottlePos);
  }
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("command_cart", 1, driveTrainCallBack);

   
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

