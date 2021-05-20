#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<std_msgs/Bool.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  // Create the pubisher at_goal for the subscriber in add_markers
  ros::Publisher pub = n.advertise<std_msgs::Bool>("at_goal", 10);
  std_msgs::Bool moved;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot has reached the object");
    moved.data = true;
	pub.publish(moved); //Let the subscriber know the robot has reached the goal
  }
  else
    ROS_INFO("The robot failed to move to the pick up location for some reason");

  //Sleep to simulate the pick up
  ros::Duration(5.0).sleep();
  pub.publish(moved);
  ROS_INFO("Robot has picked up the object. Robot will move to the drop off location.");
  //Move to the drop off location
  goal.target_pose.pose.position.x= -2.0;
  goal.target_pose.pose.position.y= -2.0;
  goal.target_pose.pose.orientation.w= -1.0;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot moved to the drop off location");
	pub.publish(moved);
  }
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  ros::Duration(5.0).sleep();
  ROS_INFO("Robot has successfully dropped off the package");

  return 0;
}
