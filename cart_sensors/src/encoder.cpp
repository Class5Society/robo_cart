#include "cart_sensors/cart_sensors.h"

/* code to read the encoder */
#define POLL_TIMEOUT 1  /*milliseconds*/
#define MAX_BUF 1
#define PULSE_DIST 3.14
#define NUM_PULSE 12
#define INCH_PER_FOOT 12.0

std::string encoderPort;
uint64_t numCounts;
MUTEX encoderMutex;

//Setup globals
ros::Publisher distancePub;

void *pollEncoder(void *pvData)
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
   status = gpio_set_dir(encoderPort, "in");

   //check stuff
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   status = gpio_set_edge(encoderPort, "rising");
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   //open the port
   gpioPort = gpio_fd_open(encoderPort);

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
        MutexLock(&encoderMutex);
  
        // increment the counter
        numCounts++;

        //Unlock the mutex
        MutexUnlock(&encoderMutex);
      }
      
   }
   
   //close the port
   gpio_fd_close(gpioPort);
   pthread_exit((void *) &closeRet);
     
}


void CartDistance(const ros::TimerEvent&)
{
  uint64_t currCounts;
  double distTrav;

  //read the current counts
  MutexLock(&encoderMutex);
  currCounts = numCounts;
  MutexUnlock(&encoderMutex);

  distTrav = (currCounts * PULSE_DIST)/INCH_PER_FOOT;
  
  //create a message
  cart_sensors::Encoder distanceMsg;
  distanceMsg.distance = distTrav;
  distanceMsg.numTurns = currCounts/NUM_PULSE;
  distanceMsg.header.stamp = ros::Time().now();
  
  //Publish the message
  distancePub.publish(distanceMsg); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoder");
  ros::NodeHandle n;
  ros::NodeHandle n_param("~");

  n_param.param<std::string>("encoder_port",encoderPort,"/dev/talos_direct/digio_11");

  //
  // Initialize the mutex that restricts access to the COM port.
  //
  MutexInit(&encoderMutex);

  //Initialize the counter
  numCounts = 0;

  // start the thread
  OSThreadCreate(pollEncoder);

  //set the publisher
  distancePub = n.advertise<cart_sensors::Encoder>("cart_distance",1);

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(.01), CartDistance);

  ros::spin();

  //return 
  return 0;
}



