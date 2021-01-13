#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{
public:
  ProcessImage()
  {
    // Define a client service capable of requesting services from command_robot
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub1_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
  } 

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img)
  {

    int white_pixel = 255;
    int pixel_location = -1;
    float lin_x = 0.0;
    float ang_z = 0.0;
    
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < (img.height * img.step) - 2; i++) {
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            pixel_location = i;
            break;
        }
    }

    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, set lin_x or ang_z variables to move
    if (pixel_location > -1) {
	int pixel_step = pixel_location % img.step;
	// we ignore the imprescision in divisibility by 3, since we are also using the first pixel encountered - also imprecise
	int image_segment = img.step / 3;
	if (pixel_step < image_segment) 
	    ang_z = 0.5;
	else if (pixel_step < 2 * image_segment)
	    lin_x = 0.5;
	else
	    ang_z = -0.5;	
    }
    // Pass lin_x and ang_z values to drive_robot_ (default is 0,0 which should stop robot if no white pixel detected)
    drive_robot_(lin_x, ang_z);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub1_;
  ros::ServiceClient client_;

  // This function calls the command_robot service to drive the robot in the specified direction
  void drive_robot_(float lin_x, float ang_z)
  {
    ROS_INFO_STREAM("Moving the robot to requested location");

    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client_.call(srv))
	ROS_ERROR("Failed to call service drive_bot");
  }

};//End of class ProcessImage

int main(int argc, char **argv)
{ 
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    //Create an object of class ProcessImage that will take care of everything
    ProcessImage PIObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
