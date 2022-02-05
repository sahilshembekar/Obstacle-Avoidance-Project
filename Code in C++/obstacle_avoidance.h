#ifndef INCLUDE_OBSTACLE_AVOIDANCE_H_
#define INCLUDE_OBSTACLE_AVOIDANCE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class ObstacleAvoidance {
private:
	// Define the NodeHandle to communications with the ROS system
	
  ros::NodeHandle nh;
	
  
  // Define a publisher object with topic name and buffer size of messages and make sure the listener is subscribed to the same topic 
	ros::Publisher publishVelocity;


	// Define a subscriber object with topic name and buffer size of messages and make sure you have subscribed to the same topic
	ros::Subscriber subscibeSensor;
	
  // Initialize linear velocity with 0.2 m/s
  const float linearVelocity = 0.2;
  // Initialize angular velocity with 30 degrees/s
  const float anguarVelocity = 0.52;
  // Initalize safe distance/threshold as 1.2m
  const float distanceThreshold = 1.2;
  // Initialize publishing rate
  const int rate = 2;
	// bool variable if obstacle was detected
	bool obstacleDetected;
  // variables for previous velocities
  float prevLinearVelocity, prevAnguarVelocity;
	// twist object to publish velocities
	geometry_msgs::Twist velocities;

public:

	// Constructor for the class
	ObstacleAvoidance();

	// Destructor for the class
	~ObstacleAvoidance();
	
	// To Check if obstacle is within safe distance
  // bool return obstacle found or not
  bool checkObstacle();

  // Starts running the robot
  void startBot();

  // Callback function for subscriber
  // param is msg data from LaserScan node
  void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  // Resets velocities of the robot
  void resetBot();

  //Checks change in velocites of the bot
  // bool return velocities changed or not
  bool checkVelocityChanged();

  // To detect obstacle detected
  // bool return obstacle detected or not
  bool getObstacleDetected() const {
    return obstacleDetected;
  }

  // To set obstacle detected
  // param is obstacle detected status
  void setObstacleDetected(bool obstacle) {
    obstacleDetected = obstacle;
  }
};

#endif // INCLUDE_OBSTACLE_AVOIDANCE_H_
