/**
 * \file
 * \brief
 * \author
 * \version 0.1
 * \date
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"

// Include here the ".h" files corresponding to the topic type you use.
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>

// You may have a number of globals here.
//...
ros::Publisher pubMarker, pubPos, pubSpeed;
visualization_msgs::Marker marker;
geometry_msgs::Point position, target;
std_msgs::Float64 targetSpeed;
bool positionInfoReceived = false, speedInfoReceived = false;
double xInit, yInit;

// Callback functions...
void positionCallback(geometry_msgs::Point posData) {
    // Setting the value of the target point
    target.x = posData.x;
    target.y = posData.y;
    target.z = 0;
    positionInfoReceived = true;
}

void speedCallback(std_msgs::Float64 spData)  {
  targetSpeed.data = spData.data;

  speedInfoReceived = true;
}

void initializeMarker(){
    // Fetch node name. Markers will be blue if the word "blue" is in the name, red otherwise.
    std::string nodeName ;
    nodeName = ros::this_node::getName();
    // Create a marker for this node. Only timestamp and position will be later updated.
    marker.header.frame_id = "/map";
    marker.ns = nodeName;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

// Function to publish a marke at a given (x,y) position.
void publishMarkerAt( geometry_msgs::Point markerPos) {
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = markerPos.x ;
    marker.pose.position.y = markerPos.y ;
    marker.lifetime = ros::Duration();
    pubMarker.publish(marker);
}

//Function to publish the master's position and speed
void publishPosAndSpeed(geometry_msgs::Point dPos, std_msgs::Float64 dSpeed) {
    position.x = dPos.x;
    position.y = dPos.y;
    position.z = dPos.z;

    pubPos.publish( position );
    pubSpeed.publish( dSpeed );
}

int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "dog");

    // Define your node handles
    ros::NodeHandle nh_loc("~") ;
    ros::NodeHandle nh_glob ;

    // Read the node parameters if any

    nh_loc.param("init_x", xInit, 0.0);
    nh_loc.param("init_y", yInit, 0.0);

    // Declare your node's subscriptions and service clients
    // ...
    ros::Subscriber masterPosSub = nh_glob.subscribe<geometry_msgs::Point>("pos", 1, positionCallback);
    ros::Subscriber masterSpeedSub = nh_glob.subscribe<std_msgs::Float64>("speed", 1, speedCallback);
    // Declare you publishers and service servers
    // ...
    pubMarker = nh_glob.advertise<visualization_msgs::Marker>("/visualization_marker",1) ;
    pubPos = nh_loc.advertise<geometry_msgs::Point>("pos",1);
    pubSpeed = nh_loc.advertise<std_msgs::Float64>("speed", 1);

    geometry_msgs::Point dogPos;
    std_msgs::Float64 dogSpeed;
    double distNorm = 0, Kp = 0.01;

    dogPos.x = xInit;
    dogPos.y = yInit;
    dogPos.z = 0;

    dogSpeed.data = 0.0;

    initializeMarker() ;
    publishMarkerAt( dogPos ) ;

    ros::Rate rate(50);   // Or other rate.
    ros::Time currentTime, prevTime = ros::Time::now() ;

    while (ros::ok()){
        ros::spinOnce();

        // Your node's code goes here.
        currentTime = ros::Time::now();
        //get the elapsed time
        ros::Duration delT = currentTime - prevTime;
        prevTime = currentTime;

        if( positionInfoReceived && speedInfoReceived) {
          //Calculate the norm of the distance between the two points : (x^2 + y^2)^0.5
          distNorm = sqrt(pow((target.x - dogPos.x) , 2) + pow((target.y - dogPos.y), 2));


          //Increase speed slowly
          while (dogSpeed.data <= targetSpeed.data) {
            dogSpeed.data += Kp;
            ROS_INFO("Master speed: %.2f Dog speed: %.2f", targetSpeed.data, dogSpeed.data);
          }

          //Update position based on formula
          dogPos.x += ((target.x - dogPos.x)/distNorm) * dogSpeed.data * delT.toSec();
          dogPos.y += ((target.y - dogPos.y)/distNorm) * dogSpeed.data * delT.toSec();

          //publish the marker at the dog's position
          publishMarkerAt(dogPos);
          pubPos.publish(dogPos);
          pubSpeed.publish(dogSpeed);
        }
        rate.sleep();
    }
}
