#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

double pick_up_zone[]={1.0,1.0,0};
double drop_off_zone[]={-2.0,-2.0,0};
bool reached_pick_up = false;
bool picked_up = false;
bool reach_drop_off = false;
bool value;

void robotMoving(const std_msgs::Bool::ConstPtr& msg){
  value = msg->data;
  if(!reached_pick_up && value ==true){
    ROS_INFO("Robot reached the pick up zone");
	reached_pick_up = true;
  }
  else if(!picked_up && value == true){
	ROS_INFO("Object was picked up");
	picked_up = true;
  }
  else if(reached_pick_up && value == true){
	ROS_INFO("Robot has reached the drop off zone");
	reach_drop_off = true;
  }
  else{
	ROS_INFO("Unexpected message received");
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub =n.subscribe("at_goal", 100, robotMoving);
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "marker";
    marker.id = 0;
    marker.type = shape;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
	if(!reached_pick_up){
		marker.pose.position.x = pick_up_zone[0];
		marker.pose.position.y = pick_up_zone[1];
		marker.pose.position.z = pick_up_zone[2];
		marker.action = visualization_msgs::Marker::ADD;
		marker_pub.publish(marker);
	}
	else if(reached_pick_up&& !picked_up && !reach_drop_off){
		marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(marker);
	}
	else if(picked_up && reached_pick_up &&!reach_drop_off){
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = drop_off_zone[0];
		marker.pose.position.y = drop_off_zone[1];
		marker.pose.position.z = drop_off_zone[2];
		marker_pub.publish(marker);
	}
	else if(picked_up && reached_pick_up && reach_drop_off){
		marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(marker);
	}
	else{
		ROS_INFO("Some unexpected state has occurred");
	}
    ros::spinOnce();
    r.sleep();
   
  }
}