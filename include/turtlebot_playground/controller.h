#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifndef __CATKIN_WS_1_SRC_TURTLEBOT_PLAYGROUND_MASTER_INCLUDE_TURTLEBOT_PLAYGROUND_CONTROLLER_H_
#define __CATKIN_WS_1_SRC_TURTLEBOT_PLAYGROUND_MASTER_INCLUDE_TURTLEBOT_PLAYGROUND_CONTROLLER_H_

class Controller
{

public:
  /**
   * Base constructor where the publisher/subscribers are setup
   * */
  Controller();

  virtual ~Controller();

  /**
   * Returns the current pose of the robot x, y, yaw
   * */
  Eigen::Vector3d GetModelState();

  /**
   * Takes in a 2d vector of heading velocity and yaw rate and applies the twist to the turtle bot
   * */
  void SetVelocity(Eigen::Vector2d &velocity);

  /**
   * return time in s
   * */
  double GetTime();

  /**
   * Wraps update function, parses odometry messages for model state x,y,yaw and calls Update() function **/
  void UpdateWrapper(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief This function is where the controller update is implemented
   * Note that Update is a pure virtual function - this makes the controller
   * class an abstract class Any child(typically called Behavior) is REQUIRED to
   * implement the Update function to inherit from the base Controller class
   * @param msg GazeboRos message containing complete robot state
   */
  virtual void Update() = 0;

private:
  // Node handle
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber odometry_subscriber_;
  Eigen::Vector3d model_state_;

  // Publisher and publisher message
  ros::Publisher twist_publisher_;
  geometry_msgs::Twist twist_msg_;
};

#endif // __CATKIN_WS_1_SRC_TURTLEBOT_PLAYGROUND_MASTER_INCLUDE_TURTLEBOT_PLAYGROUND_CONTROLLER_H_