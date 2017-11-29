#include "ros/ros.h"
#include "sensor_msgs/Range.h"

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "fakesensor");
  ros::NodeHandle n;
  ros::Publisher dist_pub = n.advertise<sensor_msgs::Range>("fakesensor",10);
  ros::Rate loop_rate(10);
  
  while(ros::ok())
  {
	sensor_msgs::Range msg;
	
	msg.header.frame_id = "fakesensor";
    msg.radiation_type = msg.ULTRASOUND;
	msg.field_of_view = 0.5;  // fake
	msg.min_range=0.02;
	msg.max_range=0.50;
	msg.range = 0.3;
	msg.header.stamp=ros::Time::now();
	dist_pub.publish(msg);
	if(msg.range==-1) ROS_INFO("distance = out of range");
    else ROS_INFO("distance = %fm", msg.range);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
