#include "ros/ros.h"
#include "pi/led.h"

bool led;
bool setLed(pi::led::Request &req, pi::led::Response &res){
    res.led_id=req.led_id;
    res.color=req.color;
    res.brightness=req.brightness;

    ROS_INFO("req : id=%d, color=%d, brightness=%d",req.led_id,req.color,req.brightness);
    ROS_INFO("res : id=%d, color=%d, brightness=%d",res.led_id,res.color,res.brightness);
    
    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "led");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("led", setLed);

    ROS_INFO("ready led service server");
    
    ros::spin();

    return 0;
}