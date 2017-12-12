#include "ros/ros.h"
#include "pi/led.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

ros::ServiceClient sc;
pi::led srv;

int dist_sonar=0;
int dist_lrf=0;
void sonar_callback(const sensor_msgs::Range::ConstPtr& msg) //20~4500
{
	//4500
    
  ROS_INFO("distance = %fm", msg->range); 
  dist_sonar=(int)msg->range;
  
}
void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//led 4~7  4*3
  ROS_INFO("distance = %fm", msg->range_min); 
   dist_lrf=(int)msg->range_min;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_to_led");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ultrasonic_distance", 1000, sonar_callback);
  ros::NodeHandle n2;
  ros::Subscriber sub2 = n2.subscribe("scan", 1000, lrf_callback);
  
  ros::NodeHandle scn;

  sc=scn.serviceClient<pi::led>("led");

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        for(int i=0; i<8;i++){
            srv.request.led_id=i;
            srv.request.color=0xFF;
            if(i<4) srv.request.brightness=(i<dist_sonar)? 0xFF:0;
            else    srv.request.brightness=(i<dist_lrf)? 0xFF:0;
            if((srv.request.led_id!=srv.response.led_id)||(srv.request.color!=srv.response.color)||(srv.request.brightness!=srv.response.brightness)){
                if(sc.call(srv))
                {
                    ROS_INFO("req : id=%u, color=%ld, brightness=%u",srv.request.led_id,srv.request.color,srv.request.brightness);
                    ROS_INFO("res : id=%u, color=%ld, brightness=%u",srv.response.led_id,srv.response.color,srv.response.brightness);
                }
                else{
                    ROS_INFO("Failed to call service");
                    ROS_ERROR("Failed to call service");
                }
            } 
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
  return 0;
}
