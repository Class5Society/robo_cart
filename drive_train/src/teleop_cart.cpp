#include <drive_train.h>

class TeleopCart
{
public:
  TeleopCart();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  float steering, throttle, brake;
  float steer_scale, throt_scale, brake_scale;
  ros::Publisher cart_pub_;
  
};

TeleopCart::TeleopCart():
  steering(5),
  throttle(0),
  brake(1),
  steer_scale(10);
  throt_scale(2);
  brake_scale(30);
{
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  cart_pub_ = nh_.advertise<drive_train::CartDrive>("command_cart", 1);
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
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        dirty = true;
        break;
    }
   

    drive_train::CartDrive drvCart;
    vel.angular = a_scale_*angular_;
    vel.linear = l_scale_*linear_;
    if(dirty ==true)
    {
      cart_pub_.publish(drvCart);    
      dirty=false;
    }
  }


  return;
}




