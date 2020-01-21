#include "ros/ros.h"
#include "std_msgs/Stringh"

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "cost_sharer");
	ros::NodeHandle nh;

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	return 0;
}

