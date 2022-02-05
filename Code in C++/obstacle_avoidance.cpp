#include "obstacle_avoidance/obstacle_avoidance.h"

//Constructor
ObstacleAvoidance::ObstacleAvoidance() {
  
  ROS_INFO_STREAM("Setting up the robot for obstacle avoidance....");
  
  // Initialize previous velocities with the current value of velocities
  
  prevLinearVelocity = linearVelocity;
  prevAnguarVelocity = anguarVelocity;
  
  // Initialize obstacle detected value as false to begin with
  obstacleDetected = false;

  // Publish the velocitiy values to the robot on the navigation topic
  publishVelocity = nh.advertise<geometry_msgs::Twist>\
               ("/cmd_vel_mux/input/navi", 1000);
  
  // Subscribe to the data of laser sensor on the scan topic
  subscibeSensor = nh.subscribe<sensor_msgs::LaserScan>("/scan", 500, \
              &ObstacleAvoidance::sensorCallback, this);
  ROS_INFO_STREAM("Set up complete");
}

//Destructor
ObstacleAvoidance::~ObstacleAvoidance() {
  // Reset the robot back
  resetBot();
}

bool ObstacleAvoidance::checkObstacle() {
  // Check if  we have a obstacle ahead
  //print if true else false
  if (getObstacleDetected()) {
    ROS_WARN_STREAM("Obstacle ahead!");
    return true;
  }

  return false;
}

void ObstacleAvoidance::startBot() {

  // Setting the publishing rate
  ros::Rate loop_rate(rate);
  while (ros::ok()) {
    if (checkObstacle()) {
      
      // Turning the robot to avoid obstacles
      velocities.linear.x = 0.0;
      velocities.angular.z = anguarVelocity;
      
      // Check the change in velocities 
      checkVelocityChanged();
    } else {
        
        // Start moving the robot once the obstacle is avoided
        velocities.angular.z = 0.0;
        velocities.linear.x = linearVelocity;
        
        // Check the change in velocities
        checkVelocityChanged();
    }

    // Publishing the velocities
    publishVelocity.publish(velocities);
   
    ros::spinOnce();
    // To maintain loop rate
    loop_rate.sleep();
  }
}

void ObstacleAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& \
                                       sensorData) {
  
  // Read sensor data to get obstacle distances w.r.t. robot
  for (const float &range : sensorData->ranges) {
    if (range < distanceThreshold) {
      // if less than threshould make it true
      setObstacleDetected(true);
      return;
    }
  }
  // else false
  setObstacleDetected(false);
}

void ObstacleAvoidance::resetBot() {
  ROS_INFO_STREAM("Resetting the robot configuration...");
  
  // Reset linear velocities
  velocities.linear.x = 0.0;
  velocities.linear.y = 0.0;
  velocities.linear.z = 0.0;

  // Reset angular velocities of the both robot
  velocities.angular.x = 0.0;
  velocities.angular.y = 0.0;
  velocities.angular.z = 0.0;
  
  // Publish the reset velocities
  publishVelocity.publish(velocities);
  ROS_INFO_STREAM("Reset complete");
}

bool ObstacleAvoidance::checkVelocityChanged() {
  // Linear and angular change simultaneously hence Check if they have changed
  if (velocities.linear.x != prevLinearVelocity and \
      velocities.angular.z != prevAnguarVelocity) {
    ROS_DEBUG_STREAM("Velocity of the robot have changed");
    
    // Update previous velocities value
    velocities.linear.x = prevLinearVelocity;
    velocities.angular.z = prevAnguarVelocity;
    return true;
  }

  return false;
}