#include <rclcpp/rclcpp.hpp>
# include <turtlesim/Pose.h>
# include "turtlesim/dds_impl/Pose_convert.h"
# include <geometry_msgs/Twist.h>
//# include "geometry_msgs/dds_impl/Twist_convert.h"
#include <std_srvs/Empty.h>

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;

enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;

#define PI 3.141592

void poseCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.1 && fabsf(g_pose->y - g_goal.y) < 0.1 && fabsf(g_pose->theta - g_goal.theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

void printGoal()
{
  //ROS_INFO("New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

/*
void commandTurtle(std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub->publish(twist);
}
*/
void commandTurtle(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub, float linear, float angular)
{
  turtlesim::Pose twist;
  twist.linear_velocity = linear;
  twist.angular_velocity = angular;
  twist_pub->publish(twist);
}

//void stopForward(std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub)
void stopForward(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub)
{
  if (hasStopped())
  {
    //ROS_INFO("Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;
    g_goal.y = g_pose->y;
    g_goal.theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}

//void stopTurn(std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub)
void stopTurn(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub)
{
  if (hasStopped())
  {
    //ROS_INFO("Reached goal");
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }
  else
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


//void forward(std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub)
void forward(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 1.0, 0.0);
  }
}

//void turn(std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub)
void turn(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  }
  else
  {
    commandTurtle(twist_pub, 0.0, 0.4);
  }
}

//void timerCallback(const ros::TimerEvent&, std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub)
void timerCallback(std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub)
{
  if (!g_pose)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = cos(g_pose->theta) * 2 + g_pose->x;
    g_goal.y = sin(g_pose->theta) * 2 + g_pose->y;
    g_goal.theta = g_pose->theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(twist_pub);
  }
  else if (g_state == STOP_FORWARD)
  {
    stopForward(twist_pub);
  }
  else if (g_state == TURN)
  {
    turn(twist_pub);
  }
  else if (g_state == STOP_TURN)
  {
    stopTurn(twist_pub);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::node::Node> nh = rclcpp::create_node("draw_square");
  std::shared_ptr<rclcpp::subscription::Subscription<turtlesim::Pose> > pose_sub = nh->create_subscription<turtlesim::Pose>("turtle1_pose", 1, poseCallback);
  //std::shared_ptr<rclcpp::publisher::Publisher<geometry_msgs::Twist> > twist_pub = nh->create_publisher<geometry_msgs::Twist>("turtle1_cmd_vel", 1);
  std::shared_ptr<rclcpp::publisher::Publisher<turtlesim::Pose> > twist_pub = nh->create_publisher<turtlesim::Pose>("turtle1_cmd_vel", 1);
  //ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");
  //ros::Timer timer = nh.createTimer(ros::Duration(0.016), boost::bind(timerCallback, _1, twist_pub));

  //std_srvs::Empty empty;
  //reset.call(empty);

  //nh->spin();
  while(nh->is_running())
  {
    nh->spin_once();
    timerCallback(twist_pub);
  }
}
