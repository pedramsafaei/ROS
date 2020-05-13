#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"demo_topic_publisher");
	ros::NodeHandle node_obj;
	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",10);
	ros::Rate loop_rate(10);
	int number_count = 0;
	while (ros::ok())
	{
		std_msgs::Int32 msg;
		msg.data = number_count;
		ROS_INFO("%d",msg.data);
		number_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++number_count;
	}
	return 0;
}


