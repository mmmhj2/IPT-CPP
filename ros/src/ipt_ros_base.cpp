#include "ipt_ros_base.h"

using namespace ipt;

IPT_ROSBase::IPT_ROSBase()
{
	nh.reset(new ros::NodeHandle());
	pnh.reset(new ros::NodeHandle("~"));
	rate.reset(new ros::Rate(120));
}

IPT_ROSBase::~IPT_ROSBase()
{
}

void IPT_ROSBase::WaitAndSpin()
{
	ros::spinOnce();
	this->rate->sleep();
}

ros::NodeHandle* IPT_ROSBase::GetPrivateNH() const
{
	return this->pnh.get();
}
