#include "ipt_ros_interface.h"

#include <functional>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace ipt;

IPT_ROSInterface* IPT_ROSInterface::instance = nullptr;

IPT_ROSInterface::IPT_ROSInterface(/*int argc, char* argv[], const std::string& nname*/)
{

	poseSeq = 0;
	//poseFrameName = "map";

	this->ReadParameters();
	this->ConstructNodes();
}

void IPT_ROSInterface::ReadParameters()
{
	pnh->param<std::string>("OrientationSubscriberNodeName", this->orientationSubscriberNodeName, "mavros/local_position/pose");
	pnh->param<std::string>("ImuSubscriberNodeName", this->imuSubscriberNodeName, "mavros/imu/data");
	pnh->param<std::string>("PosePublisherNodeName", this->posePublisherNodeName, "ipt_node/pose");
	pnh->param<std::string>("PoseFrameName", this->poseFrameName, "map");
	pnh->param<double>("Frequency", this->frequency, 50.0);
}

void IPT_ROSInterface::ConstructNodes()
{
	// Use std::bind + std::mem_fn to call a member function statically
	std::function<void(const geometry_msgs::PoseStampedConstPtr&)> BoundCallbackPose
		= std::bind(std::mem_fn(&IPT_ROSInterface::OrientationCallback), this, std::placeholders::_1);
	std::function<void(const sensor_msgs::ImuConstPtr&)> BoundCallbackImu
		= std::bind(std::mem_fn(&IPT_ROSInterface::ImuCallback), this, std::placeholders::_1);
	this->orientationSubscriber
		= nh->subscribe<geometry_msgs::PoseStamped>(this->orientationSubscriberNodeName, 10, BoundCallbackPose);
	this->quatSubscriber
		= nh->subscribe<sensor_msgs::Imu>(this->imuSubscriberNodeName, 10, BoundCallbackImu);
	this->posePublisher
		= nh->advertise<geometry_msgs::PoseStamped>(this->posePublisherNodeName, 10);
	this->rawPosePublisher
		= nh->advertise<geometry_msgs::PoseStamped>(this->posePublisherNodeName + "_raw", 10);
	this->uncalibratedPosePublisher
		= nh->advertise<geometry_msgs::PoseStamped>(this->posePublisherNodeName + "_uncalib", 10);
	this->covPosePublisher
		= nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(this->posePublisherNodeName + "_cov", 10);
}

void IPT_ROSInterface::OrientationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	this->CurrentPose = *msg;
}

void IPT_ROSInterface::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	this->ImuHeader = msg->header;
	this->ImuQuat = msg->orientation;
}

IPT_ROSInterface::~IPT_ROSInterface()
{
}

IPT_ROSInterface*
IPT_ROSInterface::GetInstance(/*int argc, char* argv[], const std::string& nname*/)
{
	if (IPT_ROSInterface::instance == nullptr)
		IPT_ROSInterface::instance = new IPT_ROSInterface(/*argc, argv, nname */ );
	return IPT_ROSInterface::instance;
}

void IPT_ROSInterface::PublishPose(geometry_msgs::PoseStamped& pose)
{
	pose.header.seq = ++(this->poseSeq);
	pose.header.frame_id = this->poseFrameName;
	this->posePublisher.publish(pose);

	geometry_msgs::PoseWithCovarianceStamped pose_cov;
	pose_cov.header = pose.header;
	pose_cov.pose.pose = pose.pose;
	//pose_cov.pose.covariance = IPT_ROSInterface::CovarianceArray.data();
	for (size_t i = 0; i < 36; i++)
		pose_cov.pose.covariance[i] = IPT_ROSInterface::CovarianceArray[i];

	this->covPosePublisher.publish(pose_cov);
}

void IPT_ROSInterface::PublishPose(geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& poseRaw, geometry_msgs::PoseStamped & poseUncalib)
{
	this->PublishPose(pose);
	poseRaw.header.seq = (this->poseSeq);
	poseRaw.header.frame_id = this->poseFrameName;
	poseUncalib.header.seq = (this->poseSeq);
	poseUncalib.header.frame_id = this->poseFrameName;
	this->rawPosePublisher.publish(poseRaw);
	this->uncalibratedPosePublisher.publish(poseUncalib);

	geometry_msgs::PoseWithCovarianceStamped pose_cov;
	pose_cov.header = pose.header;
	pose_cov.pose.pose = pose.pose;
	//pose_cov.pose.covariance = IPT_ROSInterface::CovarianceArray.data();
	for (size_t i = 0; i < 36; i++)
		pose_cov.pose.covariance[i] = IPT_ROSInterface::CovarianceArray[i];

	this->covPosePublisher.publish(pose_cov);
}

void IPT_ROSInterface::GetEstimatedPose(geometry_msgs::Quaternion& quat) const
{
	quat = this->ImuQuat;
}

void IPT_ROSInterface::GetEstimatedPose(geometry_msgs::Quaternion& quat, std_msgs::Header& header) const
{
	header = this->ImuHeader;
	quat = this->ImuQuat;
}

void IPT_ROSInterface::GetFilteredPose(geometry_msgs::PoseStamped& pose) const
{
	pose = this->CurrentPose;
}
/*
void IPT_ROSInterface::WaitAndSpin()
{
	pRate->sleep();
	ros::spinOnce();
}

ros::NodeHandle* ipt::IPT_ROSInterface::GetPrivateNH()
{
	return pnh.get();
}
*/
