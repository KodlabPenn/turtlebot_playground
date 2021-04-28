#include "turtlebot_playground/controller.h"
#include <ros/ros.h>

Controller::~Controller(){}

Controller::Controller(/* args */)
{
  //State subscriber setup
  std::string sub_robot_topic;
  if (n.getParam("/sub_robot_topic", sub_robot_topic)) {
    odometry_subscriber_ = n.subscribe(sub_robot_topic, 1, &Controller::UpdateWrapper, this);
  }

  //Twist publisher setup
  std::string pub_twist_topic;
  if (n.getParam("/pub_twist_topic", pub_twist_topic)) {
    twist_publisher_ = n.advertise<geometry_msgs::Twist>(pub_twist_topic, 1);
  }
}

void Controller::UpdateWrapper(const nav_msgs::Odometry::ConstPtr &msg)
{

  double roll, pitch, yaw;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  model_state_[0] = msg->pose.pose.position.x;
  model_state_[1] = msg->pose.pose.position.y;
  model_state_[2] = yaw;

  this->Update();
}

Eigen::Vector3d Controller::GetModelState()
{
  return model_state_;
}

void Controller::SetVelocity(Eigen::Vector2d &velocity)
{
  twist_msg_.linear.x = velocity[0];
  twist_msg_.angular.z = velocity[1];
  twist_publisher_.publish(twist_msg_);
}

double Controller::GetTime() { 
  return ros::Time::now().toSec();
}