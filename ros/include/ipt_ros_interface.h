
#ifndef IPT_ROS_INTERFACE_INCLUDED
#define IPT_ROS_INTERFACE_INCLUDED

#include "ipt_ros_base.h"

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

namespace ipt
{
	/*
	* IPT_ROSInterface : A naive singleton used for ROS operations
	*/
	class IPT_ROSInterface : public IPT_ROSBase
	{
	private:

		constexpr static std::array<double, 36> CovarianceArray = 
			{0.1, 0, 0, 0, 0, 0, 
			0, 0.1, 0, 0, 0, 0,
			0, 0, 0.1, 0, 0, 0,
			0, 0, 0, 0.1, 0, 0,
			0, 0, 0, 0, 0.1, 0,
			0, 0, 0, 0, 0, 0.1};

		std::string orientationSubscriberNodeName;
		std::string imuSubscriberNodeName;
		std::string posePublisherNodeName;
		std::string poseFrameName;
		uint32_t poseSeq;
		double frequency;

		geometry_msgs::PoseStamped CurrentPose;

		std_msgs::Header ImuHeader;
		geometry_msgs::Quaternion ImuQuat;

		ros::Publisher posePublisher, rawPosePublisher, uncalibratedPosePublisher, covPosePublisher;
		ros::Subscriber orientationSubscriber, quatSubscriber;

		// ros::Rate has no default constructor, so we use a pointer
		//std::unique_ptr<ros::Rate> pRate;
		//std::unique_ptr<ros::NodeHandle> nh, pnh;

		void OrientationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

		void ReadParameters();
		void ConstructNodes();

	protected:
		IPT_ROSInterface(/*int argc, char* argv[], const std::string& node_name*/);

		static IPT_ROSInterface* instance;
	public:
		virtual ~IPT_ROSInterface();

		// Delete copy operator
		IPT_ROSInterface& operator = (const IPT_ROSInterface&) = delete;
		// Delete copy constructor
		IPT_ROSInterface(const IPT_ROSInterface&) = delete;

		static IPT_ROSInterface* GetInstance(/*int argc = 0, char* argv[] = nullptr, const std::string& node_name = "ipt_node"*/);

		// Modify the header of the pose and then publish it
		void PublishPose(geometry_msgs::PoseStamped& pose);
		// Modify the header and publish the pose with raw data
		void PublishPose(geometry_msgs::PoseStamped& pose, 
			geometry_msgs::PoseStamped& raw, 
			geometry_msgs::PoseStamped& uncalibrated);
		void GetEstimatedPose(geometry_msgs::Quaternion& quat) const;
		void GetEstimatedPose(geometry_msgs::Quaternion& quat, std_msgs::Header & header) const;

		// Aquire filtered pose from MAVROS
		void GetFilteredPose(geometry_msgs::PoseStamped& pose) const;
		
		//void WaitAndSpin();
		//ros::NodeHandle* GetPrivateNH();
	};
}

#endif // IPT_ROS_INTERFACE_INCLUDED
