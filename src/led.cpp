#include "ros/ros.h"
#include "pi/led.h" //service
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h" //message for publication

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "ws2812b.h"

visualization_msgs::MarkerArray markerArray;
    
bool setLed(pi::led::Request &req, pi::led::Response &res){
    res.led_id=req.led_id;
    res.color=req.color;
    res.brightness=req.brightness;
    
    ws2812b_set_static_led(req.led_id,req.color,req.brightness);
    ROS_INFO("req : id=%d, color=%d, brightness=%d",req.led_id,req.color,req.brightness);
    ROS_INFO("res : id=%d, color=%d, brightness=%d",res.led_id,res.color,res.brightness);
    visualization_msgs::Marker marker;

    marker.header.frame_id = "led_link";
    marker.header.stamp = ros::Time();
    marker.ns = "led_namespace";
    marker.id = req.led_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = 0.0015;
    marker.pose.position.y = 0.0315-(0.009*req.led_id);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005*(req.brightness/255);
    marker.scale.y = 0.005*(req.brightness/255);
    marker.scale.z = 0.005*(req.brightness/255);
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = ((req.color>>16)&0xFF);
    marker.color.g = ((req.color>>8)&0xFF);
    marker.color.b = (req.color&0xFF);
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    markerArray.markers[req.led_id]=marker;
    return true;
}
int main(int argc, char **argv)
{
    if(ws2812b_setup()<0){
        printf("ws2812b error");
        return 1;
    }   
    else printf("ws2812b setup ok \n");

    ros::init(argc, argv, "led");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("led", setLed);

    ROS_INFO("ready led service server");

    ros::NodeHandle nh_pub;
    ros::Publisher vis_pub = nh_pub.advertise<visualization_msgs::MarkerArray>( "visualization_markers", 10 );
    ros::Rate loop_rate(10);
    
    //initialize led
    markerArray.markers.resize(8);
    for(int i =0;i<8; i++){
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = "led_link";
        marker.header.stamp = ros::Time();
        marker.ns = "led_namespace";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0015;
        marker.pose.position.y = 0.0315-(0.009*i);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        markerArray.markers[i]=marker;
    }    
       
    while(ros::ok())
    {
        vis_pub.publish( markerArray );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}