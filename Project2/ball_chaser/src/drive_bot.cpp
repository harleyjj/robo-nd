#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

class DriveToTarget
{
public:
  DriveToTarget()
  {
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
  ros::ServiceServer get_service()
  {
    ros::ServiceServer service = n_.advertiseService("/ball_chaser/command_robot", &DriveToTarget::handle_drive_request, this);
    return service;
  }

  // This callback function executes whenever a comand_robot service is requested
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
  {

    ROS_INFO("DriveToTargetRequest received - l_x:%1.2f, a_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [linear_x, angular_z]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Wheel velocities set - l_x: " + std::to_string(req.linear_x) + " , a_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

  

private:
  ros::NodeHandle n_; 
  ros::Publisher motor_command_publisher_;

};//End of class DriveToTarget

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "drive_bot");

    //Create an object of class DriveToTarget that will take care of everything
    DriveToTarget SAPObject;

    ros::ServiceServer service = SAPObject.get_service();

    ROS_INFO("Ready to send drive commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
