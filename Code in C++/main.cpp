#include "obstacle_avoidance/obstacle_avoidance.h"

int main(int argc, char* argv[]) {

  // Initialize the ROS node
  ros::init(argc, argv, "obstacle_avoidance");
  
  
  // Create object for the class obstacle avoidance 
  
  ObstacleAvoidance obstacleAvoidance;

  
  
  // Start the robot
  obstacleAvoidance.startBot();
  return 0;
}