#include <drive_train.h>

class TeleopCart
{
public:
  TeleopCart();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double steering, throttle, brake;
  double steer_incr, throt_incr, brake_incr;
  ros::Publisher cart_pub_;
  
};

TeleopCart::TeleopCart():
  steering(0),
  throttle(0),
  brake(0),
  steer_incr(1),
  throt_incr(1),
  brake_incr(1)
{
  nh_.param("steering_increment", steer_incr,steer_incr);
  nh_.param("throttle_increment", throt_incr,throt_incr);
  nh_.param("brake_increment", brake_incr,brake_incr);

  cart_pub_ = nh_.advertise<drive_train::CartDrive>("command_cart", 1);

  //get parameters
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_cart");
  TeleopCart teleop_cart;

  signal(SIGINT,quit);

  teleop_cart.keyLoop();
  
  return(0);
}


void TeleopCart::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the cart.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    steering=throttle=brake=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        steering = 1;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        steering = -1;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        throttle = 1;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        throttle = -1;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("STOP");
        brake = -1;
        dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("ADVANCE");
        brake = 1;
        dirty = true;
        break;
    }
   
    drive_train::CartDrive drvCart;
    drvCart.steering = steering * steer_incr;
    drvCart.throttle = throttle * throt_incr;
    drvCart.brake = brake * brake_incr;

    if(dirty ==true)
    {
      ROS_INFO("Brake %f",drvCart.brake);
      cart_pub_.publish(drvCart);    
      dirty=false;
    }
  }


  return;
}




