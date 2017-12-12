#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <stdio.h>
#include <wiringPi.h>

#define trig_pin 8  //header 3
#define echo_pin 9  //header 5

float read(){
	int time=0;
	float dist=0;
	digitalWrite(trig_pin,LOW);
	delayMicroseconds(2);
	digitalWrite(trig_pin,HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_pin,LOW);
	
	while((digitalRead(echo_pin)==0)&&time<60000){
		time++;
		delayMicroseconds(1);
	}
	time=0;
	while((digitalRead(echo_pin)==1)&&time<60000){
		time++;
		delayMicroseconds(1);
	}
	dist = (time*0.1657);
	if(dist>5000 || dist==0) return -1;
	
	return dist/1000;
}
int main(int argc, char **argv)
{

    setenv("WIRINGPI_GPIOMEM","1",1);
    wiringPiSetup();
    pinMode(trig_pin,OUTPUT);
    pinMode(echo_pin,INPUT);
  
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
        msg.max_range=4.5; //450cm
        msg.range = read();
        msg.header.stamp=ros::Time::now();
        dist_pub.publish(msg);
        if(msg.range==-1) ROS_INFO("distance = out of range");
        else ROS_INFO("distance = %fm", msg.range);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}