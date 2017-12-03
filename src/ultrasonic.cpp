#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#define trig_pin 8  //header 3
#define echo_pin 9  //header 5

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ultrasonic");
    ros::NodeHandle n;
    ros::Publisher dist_pub = n.advertise<sensor_msgs::Range>("ultrasonic_distance",10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        sensor_msgs::Range msg;

        msg.header.frame_id = "ultrasonic";
        msg.radiation_type = msg.ULTRASOUND;
        msg.field_of_view = 0.5;
        msg.min_range=0.02; //2cm
        msg.max_range=0.50; //50cm
        msg.range = 0.315;
        msg.header.stamp=ros::Time::now();
        dist_pub.publish(msg);
        if(msg.range==-1) ROS_INFO("distance = out of range");
        else ROS_INFO("distance = %fm", msg.range);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}