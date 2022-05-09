#include "ipt_ros_streamer.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ipt_streamer_node");

	ipt::IPT_Streamer Streamer;

	Streamer.Loop();
}
