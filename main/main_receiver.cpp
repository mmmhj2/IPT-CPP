#include "ipt_ros_pose_receiver.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ipt_pose_receiver");
	ipt::IPT_PoseReceiver pr;

	pr.Loop();
}
