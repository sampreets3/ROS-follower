#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>

#include <algorithm>
#include <stdlib.h>

//ROS
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std ;

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	return ch;
}

int main (int argc, char** argv)
{
	int key;
	ros::Publisher pub_key;

	//Connect to ROS
	ros::init(argc, argv, "capture_key_node");
	ROS_INFO("Node capture_key_node connected to roscore");

	ros::NodeHandle nh_;//ROS Handler

	pub_key = nh_.advertise<std_msgs::Int16>("/key_typed", 1);

	ros::Rate rate(100);
	while (ros::ok()){
		ros::spinOnce();
		key=kbhit();
		std_msgs::Int16 key_typed ;

		if( key > 0 ){
			key_typed.data=key ;
			pub_key.publish(key_typed);
		}

		rate.sleep();
	}


	ROS_INFO("ROS-Node Terminated\n");
}
