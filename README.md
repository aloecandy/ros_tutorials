# Making an ultrasonic sensor light

> This tutorial covers how to make nodes communicating with other nodes.</br>
> Prerequisites: Raspberry PI3(ROS installed), ultrasonic sensor(us-100), led(NS-LED-2)

---

## Overview

Let's make a sensor light which can automatically turn on when something is close to the sensor.

We will make following nodes:

1. A sensing node: This node will sense the distance and publish the distance data. (Publisher)
1. A computing node: This node will subscribe the published distance data and decide the led status and then send a request to control node. (Subscriber, Service client)
1. A controlling node: This node will control the led when other nodes send a request. (Service server)

![](2017-12-01-16-06-13.png)
![](2017-12-01-16-40-22.png)

---

## Creating a package

In your catkin sourcespace, use the catkin_create_pkg script to create a new package called 'pi' which depends on roscpp, message_generation,message_runtime, sensor_msgs:

```no
catkin_create_pkg pi roscpp message_generation message_runtime sensor_msgs
```

* meesage_generation and message_runtime are for the communication.
* sensor_msgs is the data type which we will send and receive.

---

## the sensing node

~/catkin_ws/src/pi/src/ultrasonic.cpp:

```cpp
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <stdio.h>
#include <wiringPi.h>

#define trig_pin 8  //header 3
#define echo_pin 9  //header 5

int read(){
    int time=0;
    int dist=0;
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
    return dist;
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
    msg.max_range=0.50; //50cm
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
```

---

## the controlling node

~/catkin_ws/src/pi/src/led.cpp:

```cpp
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "pi_gpio/SrvLed.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "ws2812b.h"

bool led;
bool setLed(pi_gpio::SrvLed::Request &req, pi_gpio::SrvLed::Response &res){
    res.n=req.n;
    led=res.n;
    for(int i=0;i<7;i++){
        ws2812b_set_static_led(i,0xFFFFFF,req.n ? 255:0);
    }
    ROS_INFO("req=%d, res=%d",req.n,req.n);
    //ROS_INFO("Position=%d, Color=%d, Brightness=%d",req.position,req.color,req.brightness);
    return true;
}

int main(int argc, char **argv)
{
    led=false;
    int ret = 0 , i = 0;

    //ws2812b setup
    if((ret = ws2812b_setup())<0)
        ;//pabort("set up");
    else
        printf("ws2812b setup ok \n");

    for(int i=0;i<8;i++){
        ws2812b_set_static_led(i,0xFFFFFF,0);
    }
    ros::init(argc, argv, "led_service");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("led_srv", setLed);

    ros::NodeHandle nh_pub;
    ros::Publisher dist_pub = nh_pub.advertise<sensor_msgs::Range>("led",10);
    ros::Rate loop_rate(10);

    ROS_INFO("ready led srv server");
    while(ros::ok()){
        sensor_msgs::Range msg;

        msg.header.frame_id = "led";
        msg.radiation_type = msg.ULTRASOUND;
        msg.field_of_view = 66000;  // fake
        msg.min_range=0.002;
        msg.max_range=0.005;
        if(led){
            msg.range = 0.005;
        }
        else{
            msg.range=0;
        }
        msg.header.stamp=ros::Time::now();
        dist_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ws2812b_close();
    return 0;
}
```

~/catkin_ws/src/pi/srv/led.srv: 

```no
int64 led_id
int64 color
int64 brightness
---
int64 led_id
int64 color
int64 brightness
```

---

## the computing node

~/catkin_ws/src/pi/src/distance_to_led.cpp:

```cpp
#include "ros/ros.h"
#include "pi/led.h"
#include "sensor_msgs/Range.h"


ros::ServiceClient sc;
pi::led srv;
void callback(const sensor_msgs::Range::ConstPtr& msg)
{
    ROS_INFO("distance = %dmm", msg->range); 
    if((msg->range)<0.5){
        srv.request.led_id=0;
        srv.request.color=0xFF003F;
        srv.request.brightness=0xFF;
    }
    else{
        srv.request.led_id=0;
        srv.request.color=0x00FF3F;
        srv.request.brightness=0xFF;
    }
    if((srv.request.led_id!=srv.response.led_id)|(srv.request.color!=srv.response.color)|(srv.request.brightness!=srv.response.brightness)){
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
```

---

## package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>pi</name>
  <version>0.0.0</version>
  <description>The pi package</description>

  <maintainer email="aloe@todo.todo">aloe</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>message_runtime</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>sensor_msgs</build_depend>

  <build_export_depend>message_generation</build_export_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>

  <exec_depend>message_generation</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <export>

  </export>
</package>
```

---

## CMakeLists.txt

```CMake
cmake_minimum_required(VERSION 2.8.3)
project(pi)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  sensor_msgs
)

add_service_files(FILES led.srv)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES pi
  CATKIN_DEPENDS message_generation message_runtime roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(led src/led.cpp)
add_dependencies(led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(led ${catkin_LIBRARIES})

add_executable(ultrasonic src/ultrasonic.cpp)
add_dependencies(ultrasonic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ultrasonic ${catkin_LIBRARIES})

add_executable(distance_to_led src/distance_to_led.cpp)
add_dependencies(distance_to_led ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_to_led ${catkin_LIBRARIES})
```