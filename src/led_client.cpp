#include "ros/ros.h"
#include "pi/led.h" //service

int main(int argc, char **argv)
{
    
    ros::ServiceClient sc;
    pi::led srv;

    ros::init(argc, argv, "led_client");
    
    ros::NodeHandle scn;

	srv.request.led_id=atoi(0);
    srv.request.color=atoi(255);
    srv.request.brightness=atoi(255);
	sc.call(srv);
	/*
    sc=scn.serviceClient<pi::led>("led");
    if(argc==4){
    srv.request.led_id=atoi(argv[1]);
    srv.request.color=atoi(argv[2]);
    srv.request.brightness=atoi(argv[3]);
	}
    if(sc.call(srv))
    {
        ROS_INFO("req : id=%d, color=%d, brightness=%d",srv.request.led_id,srv.request.color,srv.request.brightness);
        ROS_INFO("res : id=%d, color=%d, brightness=%d",srv.response.led_id,srv.response.color,srv.response.brightness);
    }
    else{
        ROS_ERROR("Failed to call service");
    }
*/
  return 0;
}
