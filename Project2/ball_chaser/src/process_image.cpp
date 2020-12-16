#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
bool found_white = false;
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
	ROS_INFO_STREAM("Driving the robot");
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	
	//Call the command_robot service and pass the linear and angular values
	if (!client.call(srv))
		ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int white_pos;
	ROS_INFO("In process_image_callback function");
	// Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
		ROS_INFO("Data image - %d", (int)img.data[i]);
        if (img.data[i] == white_pixel) {
            ROS_INFO("Found white");
			white_pos = i % img.step;
            if(white_pos < (img.step* (1/3))){
				drive_robot(0,0.4);
				break;
			}
			else if(white_pos >= img.step* (1/3) && white_pos <= (img.step *(2/3))){
				drive_robot(0.4,0);
				break;
			}
			else{
				drive_robot(0,-0.4);
				break;
			}
        }
		else{
			drive_robot(0,0);
		}
	}			
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}