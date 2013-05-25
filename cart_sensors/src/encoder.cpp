#include "cart_sensors/cart_sensors.h"

/* code to read the encoder */
#define POLL_TIMEOUT 100  /*milliseconds*/
#define MAX_BUF 1
std::string encoderPort;
uint64_t numCounts;
MUTEX encoderMutex;

/****************************************************************
 * gpio_set_edge
 ****************************************************************/
int gpio_set_edge(std::string edge)
{
	int fd;
	std::string buf = encoderPort + "/edge";

	fd = open((char *) buf.c_str(), O_WRONLY);
	if (fd < 0) {
		return -1;
	}
 
	write(fd, (char *) edge.c_str(), edge.length() + 1); 
	close(fd);
	return 0;
}
/****************************************************************
 * gpio_set_dir
 ****************************************************************/
int gpio_set_dir(std::string dirStr)
{
	int fd;
	std::string buf = encoderPort + "/direction";
 
	fd = open((char *) buf.c_str(), O_WRONLY);
	if (fd < 0) {
		return -1;
	}
 
	write(fd, (char *) dirStr.c_str(), dirStr.length() + 1); 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open()
{
	int fd, len;

	std::string buf = encoderPort + "/value";
 
	fd = open((char *) buf.c_str(), O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		return -1;
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
	return close(fd);
}

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
   status = gpio_set_dir("in");

   //check stuff
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   status = gpio_set_edge("rising");
   if (status < 0)
   {
     pthread_exit((void *) &errorRet);
   }

   //open the port
   gpioPort = gpio_fd_open();

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
  ROS_INFO("Callback 1 triggered NumCounts %llu",numCounts);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  n.param<std::string>("encoder_port",encoderPort,"/dev/talos_direct/digio_11");

  //
  // Initialize the mutex that restricts access to the COM port.
  //
  MutexInit(&encoderMutex);

  //Initialize the counter
  numCounts = 0;

  // start the thread
  OSThreadCreate(pollEncoder);

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(.01), CartDistance);

  ros::spin();

  //return 
  return 0;
}



