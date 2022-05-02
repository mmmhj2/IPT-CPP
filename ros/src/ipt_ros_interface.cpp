#include "ipt_ros_interface.h"

#include <functional>

using namespace IPT;

IPT_ROSInterface* IPT_ROSInterface::instance = nullptr;

IPT_ROSInterface::IPT_ROSInterface(int argc, char* argv[], const std::string& nname)
{
	ros::init(argc, argv, nname);

	// NodeHandle must be constructed after ros::init
	// and therefore it's not desirable to use delegating constructors
	nh.reset(new ros::NodeHandle());
	pnh.reset(new ros::NodeHandle("~"));
	pRate.reset(new ros::Rate(20));

	poseSeq = 0;
	//poseFrameName = "map";

	this->ReadParameters();
	this->ConstructNodes();
}

void IPT_ROSInterface::ReadParameters()
{
	pnh->param<std::string>("OrientationSubscriberNodeName", this->orientationSubscriberNodeName, "mavros/local_position/pose");
	pnh->param<std::string>("PosePublisherNodeName", this->posePublisherNodeName, "ipt_node/pose");
	pnh->param<std::string>("PoseFrameName", this->poseFrameName, "map");
}

void IPT_ROSInterface::ConstructNodes()
{
	// Use std::bind + std::mem_fn to call a member function statically
	std::function<void(const geometry_msgs::PoseStampedConstPtr&)> BoundCallback 
		= std::bind(std::mem_fn(&IPT_ROSInterface::OrientationCallback), this, std::placeholders::_1);
	this->orientationSubscriber
		= nh->subscribe<geometry_msgs::PoseStamped>(this->orientationSubscriberNodeName, 10, BoundCallback);
	this->posePublisher
		= nh->advertise<geometry_msgs::PoseStamped>(this->posePublisherNodeName, 10);
}

void IPT_ROSInterface::OrientationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	this->CurrentPose = *msg;
}

IPT_ROSInterface::~IPT_ROSInterface()
{
}

IPT_ROSInterface*
IPT_ROSInterface::GetInstance(int argc, char* argv[], const std::string & nname)
{
	if (IPT_ROSInterface::instance == nullptr)
		IPT_ROSInterface::instance = new IPT_ROSInterface(argc, argv, nname);
	return IPT_ROSInterface::instance;
}

void IPT_ROSInterface::PublishPose(geometry_msgs::PoseStamped& pose)
{
	pose.header.seq = ++(this->poseSeq);
	pose.header.frame_id = this->poseFrameName;
	this->posePublisher.publish(pose);
}

void IPT_ROSInterface::GetEstimatedPose(geometry_msgs::PoseStamped& pose) const
{
	pose = this->CurrentPose;
}

void IPT_ROSInterface::WaitAndSpin()
{
	pRate->sleep();
	ros::spin();
}

ros::NodeHandle* IPT::IPT_ROSInterface::GetPrivateNH()
{
	return pnh.get();
}
