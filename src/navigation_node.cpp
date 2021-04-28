#include <ros/ros.h>
#include "turtlebot_playground/behavior.h"


int main(int argc, char **argv){
  //Initialize ROS Node
  ros::init(argc, argv, "navigation_node");

  //Instantiate Leg state machine
  Behavior myBehavior;

  // Process Ros node and callbacks
  while(ros::ok()) { 

    ros::spinOnce(); 

  }
  
  return 0;
}