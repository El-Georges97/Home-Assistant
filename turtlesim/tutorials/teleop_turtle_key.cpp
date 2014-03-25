# include <rclcpp/rclcpp.hpp>
# include <turtlesim/Pose.h>
# include "turtlesim/dds_impl/Pose_convert.h"
# include <geometry_msgs/Twist.h>
//# include "geometry_msgs/dds_impl/Twist_convert.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

std::shared_ptr<rclcpp::node::Node> g_nh;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  double linear_, angular_, l_scale_, a_scale_;
  //std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub_;
  std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  g_nh = rclcpp::create_node("teleop_turtle");

  //twist_pub_ = g_nh->create_publisher<geometry_msgs::Twist>("turtle1_cmd_vel", 1);
  twist_pub_ = g_nh->create_publisher<turtlesim::Pose>("turtle1_cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  g_nh->shutdown("caught a signal");
  exit(0);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
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
    //ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        //ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        //ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        //ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        //ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
    }
   

    //geometry_msgs::Twist twist;
    //twist.angular.z = a_scale_*angular_;
    //twist.linear.x = l_scale_*linear_;
    turtlesim::Pose twist;
    twist.angular_velocity = a_scale_*angular_;
    twist.linear_velocity = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      g_nh->spin_once();
      dirty=false;
    }
  }


  return;
}



