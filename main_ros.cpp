#include <ros/ros.h>
#include "ipt_ros_interface.h"

int main(int argc, char * argv[])
{

	IPT::IPT_ROSInterface* pInterface = IPT::IPT_ROSInterface::GetInstance(argc, argv);

	return 0;
}
