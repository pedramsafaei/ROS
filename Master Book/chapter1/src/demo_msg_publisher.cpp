#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "mastering_ros_demo_pkg/demo_msg.h"
#include <iostream>
#include <sstream>


using namespace std;

int main(int argc, char **argv)

{
	ros::init(argc, argv,"demo_msg_publisher");
	ros::NodeHandle node_obj;
	ros::Publisher number_publisher = node_obj.advertise<mastering_ros_demo_pkg::demo_msg>("/demo_msg_topic",10);
	ros::Rate loop_rate(10);
	int number_count = 0;
	while (ros::ok())
	{
		mastering_ros_demo_pkg::demo_msg msg;
	       std::stringstream ss;
	       ss << "hello world ";
	       msg.greeting = ss.str();

		msg.number = number_count;
		ROS_INFO("%d",msg.number);
		ROS_INFO("%s",msg.greeting.c_str());
		number_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++number_count;
	}
	return 0;
}


