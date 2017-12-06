#include "ros/ros.h"
#include "pi/led.h"
#include "sensor_msgs/Range.h"


ros::ServiceClient sc;
pi::led srv;
void callback(const sensor_msgs::Range::ConstPtr& msg)
{
  long int id=1;
  long int color=255;
  long int brightness=255;
  
  ROS_INFO("distance = %fm", msg->range); 
//  int num=((msg->range)*20;
  for(int i=0;i<8;i++){
    if(i==8)  break;
    id=i;
    srv.request.led_id=id;
    srv.request.color=color;
    srv.request.brightness=brightness;
    if((srv.request.led_id!=srv.response.led_id)||(srv.request.color!=srv.response.color)||(srv.request.brightness!=srv.response.brightness)){
        if(sc.call(srv))
        {
            ROS_INFO("req : id=%d, color=%d, brightness=%d",srv.request.led_id,srv.request.color,srv.request.brightness);
            ROS_INFO("res : id=%d, color=%d, brightness=%d",srv.response.led_id,srv.response.color,srv.response.brightness);
        }
        else{
            ROS_ERROR("Failed to call service");
        }
    }		  
  }	  
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_to_led");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ultrasonic_distance", 1000, callback);
  
   ros::NodeHandle scn;

  sc=scn.serviceClient<pi::led>("led");
  
  ros::spin();

  return 0;
}
