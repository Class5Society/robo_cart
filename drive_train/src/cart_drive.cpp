#include <drive_train.h>


     mapLinearMoves steerMap;
     mapLinearMoves brakeMap;
     mapLinearMoves throttleMap;
     mapLinearMoves maxThrottleMap;
     ros::Publisher commandCart;

     //control values
     bool autoEnable;
     bool startAutoRun;

     //travel inputs
     double steerStartPos;
     double steerLeftStop;
     double steerRightStop;
     double throttleStartPos;
     double throttleStopPos;
     double maxThrottlePos;
     double brakeFullStop;
     double brakeOff;
 
     //outputs to command 
     double currentSteerPos;
     double currentThrottlePos;
     double currentBrakePos;
     bool fullBrakeEnable;


void cartJoyCallBack(const sensor_msgs::JoyConstPtr& msg)
{
//*******************************************
//
//joystick callback function
//
//*******************************************

  if (autoEnable == false)
  {
     //generate the steering 
     currentSteerPos = steerMap.getValue(msg->axes[0]);

     //generate the brake 
     currentBrakePos = brakeMap.getValue(msg->axes[1]);

     //check buttons
     if (msg->buttons[1] == 1 ||  fullBrakeEnable)
     {
        currentBrakePos = brakeFullStop;
        fullBrakeEnable = true;
     }
  
     if (msg->buttons[2] == 1)
     {
        currentBrakePos = brakeOff;
        fullBrakeEnable = false;
     }

     //generate the throttle
     currentThrottlePos = throttleMap.getValue(msg->axes[1]);
     maxThrottlePos = maxThrottleMap.getValue(msg->axes[2]);

     //check limits
     if (currentThrottlePos > maxThrottlePos)
     {
        currentThrottlePos = maxThrottlePos;
     }

     if (fullBrakeEnable)
     {
        currentThrottlePos = throttleStartPos;
     }
  }
  else 
  {
     if (msg->buttons[1] == 1)
     {
        startAutoRun = true;
     }

     if (msg->buttons[2] == 1)
     {
        startAutoRun = false;
     }
  }
}

void autoButtonCallBack(const cart_sensors::EngageAutoConstPtr& msg)
{
    if (msg->autoDriveOn == true)
    {
       autoEnable = true;
    }
    else
    {
       autoEnable = false;
    }
}

void autoCartDriveCallBack(const ros::TimerEvent&)
{
   //create a message
   drive_train::CartDrive commandMsg;
   commandMsg.steering = currentSteerPos;
   commandMsg.brake = currentBrakePos;
   commandMsg.throttle = currentThrottlePos;
   commandMsg.maxThrottle = maxThrottlePos;
   commandMsg.fullBrakeEnable = fullBrakeEnable;

   //publish message
   commandCart.publish(commandMsg);
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
  ros::init(argc, argv, "cart_drive");
  
  ros::NodeHandle nh;

  //get a parameter node
  ros::NodeHandle n_param;

  //Get the parameters
  n_param.param("steer_start_pos",steerStartPos,POS_START_REF);
  n_param.param("steer_left_stop",steerLeftStop,POS_LEFT_STOP);
  n_param.param("steer_right_stop",steerRightStop,POS_RIGHT_STOP);

  n_param.param("brake_full_stop",brakeFullStop,0.0);
  n_param.param("brake_off",brakeOff,5.0);
  n_param.param("throttle_start",throttleStartPos,MIN_THROT_POS);
  n_param.param("throttle_stop",throttleStopPos,MAX_THROT_POS);

  //setup the mappings for the joystick
  steerMap.initMap(steerRightStop,steerLeftStop,-1,1,steerStartPos);
  brakeMap.initMap(brakeFullStop,brakeOff,-1, 0);
  throttleMap.initMap(throttleStartPos,throttleStopPos,0,1);
  maxThrottleMap.initMap(throttleStartPos,throttleStopPos,-1,1);

  //initialize the value
  currentSteerPos = steerStartPos;
  currentBrakePos = brakeFullStop;
  currentThrottlePos = throttleStartPos;
  fullBrakeEnable = true;
  autoEnable = false;
  startAutoRun = false;

  //create the joystick subscriber
  ros::Subscriber subJoy = nh.subscribe("joy", 1, cartJoyCallBack);

  //create the button subscriber
  ros::Subscriber subButton = nh.subscribe("auto_cart", 1, autoButtonCallBack);

  // setup the publisher for commanding of the cart
  commandCart= nh.advertise<drive_train::CartDrive>("command_cart",1);

  //setup the timer to fire out the command to the cart 
  ros::Timer pubTimer = nh.createTimer(ros::Duration(.05), autoCartDriveCallBack);

  ros::spin();

  return 0;
}


