#include "cart_sensors/cart_sensors.h"

/* code to read the button */
#define POLL_TIMEOUT 500  /*milliseconds*/
#define MAX_BUF 1

std::string buttonPort;
uint8_t autoOn = 0;
MUTEX buttonMutex;

//Setup globals
ros::Publisher autoPub;

void *pollButton(void *pvData)
{
   int status;
   struct pollfd fdset[1];
   int timeOut;
   int pollReturn;
   int gpioPort;
   char buf;
   int value = 0;
   int errorRet = -1;
   int closeRet = 0;
 
   // set the edge and directon
   status = gpio_set_dir(buttonPort, "in");

   //check stuff
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   status = gpio_set_edge(buttonPort, "falling");
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   //open the port
   gpioPort = gpio_fd_open(buttonPort);

   if (gpioPort < 0)
   {
     pthread_exit((void *) &errorRet);
   } 

   while (ros::ok())
   { 
      //clear out the memory
      memset((void*) fdset, 0, sizeof(fdset));

      //set the event to watch
      fdset[0].fd = gpioPort;
      fdset[0].events = POLLPRI;

      //wait for event
      pollReturn = poll(fdset, 1, timeOut);

      if (pollReturn < 0)
      {
         pthread_exit((void *) &errorRet);
      }
      if (fdset[0].revents & POLLPRI)
      {

        //read in the data
        read(fdset[0].fd, &buf, 1);
       
        // lock the mutex
        MutexLock(&buttonMutex);
  
        // swap the state
        if (buf != '1')
        {
          if (autoOn == 0)
          {
             autoOn = 1;
          }
          else
          {
             autoOn = 0;
          }
        }

        //Unlock the mutex
        MutexUnlock(&buttonMutex);
      }
      
   }
   
   //close the port
   gpio_fd_close(gpioPort);
   pthread_exit((void *) &closeRet);
     
}

void AutoEngage(const ros::TimerEvent&)
{
  uint8_t currButton; 

  //read the current button
  MutexLock(&buttonMutex);
  currButton = autoOn;
  MutexUnlock(&buttonMutex);

  //create a message
  cart_sensors::EngageAuto autoMsg;
  autoMsg.autoDriveOn = currButton;
  autoMsg.header.stamp = ros::Time().now();
  
  //Publish the message
  autoPub.publish(autoMsg); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "engage_auto");
  ros::NodeHandle n("~");

  n.param<std::string>("button_port",buttonPort,"/dev/talos_direct/button1");

  //
  // Initialize the mutex that restricts access to the COM port.
  //
  MutexInit(&buttonMutex);

  //Initialize the engage
  autoOn = 0;

  // start the thread
  OSThreadCreate(pollButton);

  //set the publisher
  autoPub = n.advertise<cart_sensors::EngageAuto>("auto_cart",1);

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(.1), AutoEngage);

  ros::spin();

  //return 
  return 0;
}



