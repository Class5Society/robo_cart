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
     double startCartPos;
     uint32_t currGoal = 0;
     double *currGoalPtr;
 
     //outputs to command 
     double currentSteerPos;
     double currentThrottlePos;
     double currentBrakePos;
     bool fullBrakeEnable;
     bool firstTimeAuto;
     

//#define NUMPOINTS 6
//#define numGoals 8
#define STEERPOS 0
#define BRAKEPOS 1
#define THROTTLEPOS 2
#define MAXTHROTTLEPOS 3
#define BRAKEENABLEPOS 4
#define DISTGOALPOS 5

std::string goalMapFile;
double *goalMap;
int numGoals;
int numCols;
                                      // steer brake throttle maxThrottle brakeEnable distGoal 
//double goalMap[numGoals][NUMPOINTS] = { 
//                                        {0, 0, 1, 0, 0, 20}, //straight
//                                        {1, 0, 1, 0, 0, 10}, //turn
//                                        {0, 0, 1, 0, 0, 20}, //straight
//                                        {1, 0, 1, 0, 0, 10}, //turn
//                                        {0, 0, 1, 0, 0, 20}, //straight
//                                        {1, 0, 1, 0, 0, 10}, //turn
 //                                       {0, 0, 1, 0, 0, 20}, //straight
//                                        {0, 0, 0, 0, 1, 0}, //stop
//                                     };

void cartAutoDriveCallBack(const cart_sensors::EncoderConstPtr& msg)
{
//*******************************************
//
//auto callback function
//
//*******************************************

  if (autoEnable == true && startAutoRun == true)
  {
     uint32_t currGoalLoc;

     //get the offset
     if (firstTimeAuto == true)
     {
        startCartPos = msg->distance;
        currGoal = 0;
        currGoalPtr = goalMap;
        firstTimeAuto = false;
     }
     
     
     //check the goal
     double goalPos = msg->distance - startCartPos;
     if (firstTimeAuto != true && goalPos >= currGoalPtr[DISTGOALPOS] && currGoal < numGoals-1)
     {
        currGoal++;
        currGoalPtr += numCols;
        startCartPos = msg->distance;
     }
     

     //generate the steering 
     currentSteerPos = steerMap.getValue(currGoalPtr[STEERPOS]);

     //generate the brake 
     currentBrakePos = brakeMap.getValue(currGoalPtr[BRAKEPOS]);

     //check buttons
     if (currGoalPtr[BRAKEENABLEPOS] == 1 ||  fullBrakeEnable)
     {
        currentBrakePos = brakeFullStop;
        fullBrakeEnable = true;
     }
  
     if (currGoalPtr[BRAKEENABLEPOS] == 0 )
     {
        currentBrakePos = brakeOff;
        fullBrakeEnable = false;
     }

     //generate the throttle
     currentThrottlePos = throttleMap.getValue(currGoalPtr[THROTTLEPOS]);
     maxThrottlePos = maxThrottleMap.getValue(currGoalPtr[MAXTHROTTLEPOS]);

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
  else if (autoEnable == true && startAutoRun == false)
  {
     currentBrakePos = brakeFullStop;
     fullBrakeEnable = true;
     currentThrottlePos = throttleStartPos;
  }
}

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
        firstTimeAuto = true;
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
  ros::NodeHandle np_param("~");

  //Get the parameters
  n_param.param("steer_start_pos",steerStartPos,POS_START_REF);
  n_param.param("steer_left_stop",steerLeftStop,POS_LEFT_STOP);
  n_param.param("steer_right_stop",steerRightStop,POS_RIGHT_STOP);

  n_param.param("brake_full_stop",brakeFullStop,0.0);
  n_param.param("brake_off",brakeOff,5.0);
  n_param.param("throttle_start",throttleStartPos,MIN_THROT_POS);
  n_param.param("throttle_stop",throttleStopPos,MAX_THROT_POS);

  np_param.param<std::string>("goal_map_file",goalMapFile,"goalMap.txt");

  //read in the goal map
  int retVal = readTable((char *) goalMapFile.c_str(),&numGoals,&numCols, &goalMap);
 
  //setup the mappings for the joystick
  steerMap.initMap(steerRightStop,steerLeftStop,-1,1,steerStartPos);
  brakeMap.initMap(brakeFullStop,brakeOff,-1, 0);
  throttleMap.initMap(throttleStartPos,throttleStopPos,0,1);
  maxThrottleMap.initMap(throttleStartPos,throttleStopPos,-1,1);

  //initialize the value
  currentSteerPos = steerStartPos;
  currentBrakePos = brakeFullStop;
  currentThrottlePos = throttleStartPos;
  maxThrottlePos = throttleStartPos;
  fullBrakeEnable = true;
  autoEnable = false;
  startAutoRun = false;
  firstTimeAuto = false;

  //create the joystick subscriber
  ros::Subscriber subJoy = nh.subscribe("joy", 1, cartJoyCallBack);

  //create the button subscriber
  ros::Subscriber subButton = nh.subscribe("auto_cart", 1, autoButtonCallBack);

  //create the encoder subscriber
  ros::Subscriber subEncoder = nh.subscribe("cart_distance", 1, cartAutoDriveCallBack);

  // setup the publisher for commanding of the cart
  commandCart= nh.advertise<drive_train::CartDrive>("command_cart",1);

  //setup the timer to fire out the command to the cart 
  ros::Timer pubTimer = nh.createTimer(ros::Duration(.01), autoCartDriveCallBack);

  ros::spin();

  return 0;
}


