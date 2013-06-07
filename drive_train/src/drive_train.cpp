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

double throttleStartPos;
double throttleStopPos;
double maxThrottlePos;

bool brakeState = true;
double currentBrakePos;
double brakeFullStop;
double brakeOff;
double brakePVal;
double brakeIVal;
double brakeDVal;

//setup the actual values
double actualSteerPos;
double actualBrakePos;
double actualThrottlePos;

//setup globals
ros::Publisher baseState;

// create global maps
mapLinearMoves steerMap;
mapLinearMoves brakeMap;
mapLinearMoves throttleMap;
mapLinearMoves maxThrottleMap;


//*******************************************
//
//auto callback function
//
//*******************************************
void driveTrainAutoCallBack(const drive_train::CartDriveConstPtr& msg)
{

  //get steering to the current direction
  currentSteerPos = msg->steering;

  //set position
  fastCmdPosSetNoAck(steerID, currentSteerPos);

  //set the throttle
  currentThrottlePos = msg->throttle;
  maxThrottlePos = msg->maxThrottle;

  //check limits
  if (currentThrottlePos > maxThrottlePos)
  {
     currentThrottlePos = maxThrottlePos;
  }

  if (msg->fullBrakeEnable == true )
  {
     currentThrottlePos = throttleStartPos;
  }

  if (throttleDrive != NULL)
  {
     fprintf(throttleDrive,"%lf\n",currentThrottlePos);
  } 
  //get the brake current position
  currentBrakePos = msg->brake;

  //check buttons
  if (msg->fullBrakeEnable == true)  
  {
     currentBrakePos = brakeFullStop;
  }
  
  //set position
  fastCmdPosSetNoAck(brakeID, currentBrakePos);

}

//*******************************************
//
//joystick callback function
////
//*******************************************
void driveTrainJoyCallBack(const sensor_msgs::JoyConstPtr& msg)
{

  //add the steering to the current direction
  currentSteerPos = steerMap.getValue(msg->axes[0]);

  //set position4yy
  fastCmdPosSetNoAck(steerID, currentSteerPos);

  //add the steering to the current direction
  currentBrakePos = brakeMap.getValue(msg->axes[1]);;

  //check buttons
  if (msg->buttons[1] == 1 ||  brakeState == true)
  {
     currentBrakePos = brakeFullStop;
     brakeState = true;
  }
  
  if (msg->buttons[2] == 1)
  {
      currentBrakePos = brakeOff;
      brakeState = false;
  }

  //set position4yy
  fastCmdPosSetNoAck(brakeID, currentBrakePos);

  //add the steering to the current direction
  currentThrottlePos = throttleMap.getValue(msg->axes[1]);
  maxThrottlePos = maxThrottleMap.getValue(msg->axes[2]);

  //check limits
  if (currentThrottlePos > maxThrottlePos)
  {
     currentThrottlePos = maxThrottlePos;
  }

  if (brakeState == true)
  {
     currentThrottlePos = throttleStartPos;
  }

  if (throttleDrive != NULL)
  {
     fprintf(throttleDrive,"%lf\n",currentThrottlePos);
  } 

}

void driveTrainState(const ros::TimerEvent&)
{
  actualSteerPos = fastCmdPosGet(steerID);
  if (actualSteerPos < 0)
  {
    actualSteerPos = currentSteerPos;
  }

  actualBrakePos = fastCmdPosGet(brakeID);
  if (actualBrakePos < 0)
  {
    actualBrakePos = currentBrakePos;
  }

  if (throttleDrive != NULL)
  {
     //read back the value to get the actual position
     fscanf(throttleDrive, "%lf", &actualThrottlePos);
  } 

   //create message
   drive_train::CartDrive currBasePos;
   currBasePos.steering = actualSteerPos;
   currBasePos.brake = actualBrakePos;
   currBasePos.throttle = actualThrottlePos;
   currBasePos.fullBrakeEnable = brakeState;
   currBasePos.maxThrottle = maxThrottlePos;
   currBasePos.header.stamp = ros::Time().now();
   
   //publish the message
   baseState.publish(currBasePos);
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
  ros::NodeHandle np_param("~");

  /* Get the parameters needed */
  np_param.param<std::string>("can_comm_port",g_canCommPort,"/dev/ttyUSB0");
  np_param.param<std::string>("throttle_comm_port",g_throttleCommPort,"/dev/talos/servo1/position");
  np_param.param("steer_P_val",steerPVal,150.0);
  np_param.param("steer_I_val",steerIVal,0.01);
  np_param.param("steer_D_val",steerDVal,0.01);
  np_param.param("brake_P_val",brakePVal,150.0);
  np_param.param("brake_I_val",brakeIVal,0.01);
  np_param.param("brake_D_val",brakeDVal,0.01);
  np_param.param("can_steer_id",steerID,1);
  np_param.param("can_brake_id",brakeID,2);

  n.param("steer_start_pos",steerStartPos,POS_START_REF);
  n.param("steer_left_stop",steerLeftStop,POS_LEFT_STOP);
  n.param("steer_right_stop",steerRightStop,POS_RIGHT_STOP);

  n.param("brake_full_stop",brakeFullStop,0.0);
  n.param("brake_off",brakeOff,5.0);
  n.param("throttle_start",throttleStartPos,MIN_THROT_POS);
  n.param("throttle_stop",throttleStopPos,MAX_THROT_POS);
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
  fastConfigMaxV(steerID,(double) MAX_VOUT_PERC);

  //set the reference
  fastCmdPosRef(steerID,POTENTIOMETER_REF);

  //set the values or P I and D
  fastCmdPosI(steerID, steerIVal);
  fastCmdPosD(steerID, steerDVal);
  fastCmdPosP(steerID, steerPVal);

  //enable the Positon mode
  currentSteerPos = steerStartPos;
  fastCmdPosEnable(steerID, steerStartPos);

  // set the initial position
  fastCmdPosSetNoAck(steerID, currentSteerPos);
  actualSteerPos = fastCmdPosGet(steerID);

  //
  //initialize the Jaguar motor controller for brake
  //
  fastConfigTurns(brakeID,POT_MAX_TURNS);

  //Configure the max output voltage
  fastConfigMaxV(brakeID, (double) MAX_VOUT_PERC);

  //set the reference
  fastCmdPosRef(brakeID,POTENTIOMETER_REF);

  //set the values or P I and D
  fastCmdPosI(brakeID, brakeIVal);
  fastCmdPosD(brakeID, brakeDVal);
  fastCmdPosP(brakeID, brakePVal);

  //enable the Positon mode
  currentBrakePos = brakeFullStop;
  fastCmdPosEnable(brakeID, brakeFullStop);

  // set the initial position
  fastCmdPosSetNoAck(brakeID, currentBrakePos);
  actualBrakePos = fastCmdPosGet(brakeID);
  brakeState = true;
  //
  // Open the throttle port
  //
  throttleDrive = fopen(g_throttleCommPort.c_str(),"r+");
  if (throttleDrive != NULL)
  {
     setbuf(throttleDrive,NULL);
     currentThrottlePos = throttleStartPos;
     fprintf(throttleDrive,"%lf\n",currentThrottlePos);

     //read back the value to get the actual pOosition
     fscanf(throttleDrive, "%lf", &actualThrottlePos);
  }
  
  //setup the mappings for the joystick
  steerMap.initMap(steerRightStop,steerLeftStop,-1,1,steerStartPos);
  brakeMap.initMap(brakeFullStop,brakeOff,-1, 0);
  throttleMap.initMap(throttleStartPos,throttleStopPos,0,1);
  maxThrottleMap.initMap(throttleStartPos,throttleStopPos,-1,1);

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
  ros::Subscriber sub = n.subscribe("command_cart", 1, driveTrainAutoCallBack);
  //ros::Subscriber subJoy = n.subscribe("joy", 1, driveTrainJoyCallBack);

  // setup the publisher for the state of the cart
  baseState = n.advertise<drive_train::CartDrive>("cart_state",10);

  //setup the timer to fire out the publisher
  ros::Timer pubTimer = n.createTimer(ros::Duration(.02), driveTrainState);
   
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

