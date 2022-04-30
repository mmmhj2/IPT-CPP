
#ifndef IPT_ROS_INTERFACE_INCLUDED
#define IPT_ROS_INTERFACE_INCLUDED

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace IPT
{
	/*
	* IPT_ROSInterface : A naive singleton used for ROS operations
	*/
	class IPT_ROSInterface
	{
	private:
		std::string orientationSubscriberNodeName;
		std::string posePublisherNodeName;

		geometry_msgs::PoseStamped CurrentPose;

		ros::Publisher posePublisher;
		ros::Subscriber orientationSubscriber;

		void OrientationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

		void ReadParameters();
		void ConstructNodes();

	protected:
		IPT_ROSInterface(int argc, char* argv[], const std::string& node_name);

		static IPT_ROSInterface* instance;
		std::unique_ptr<ros::NodeHandle> nh, pnh;
	public:
		virtual ~IPT_ROSInterface();

		// Delete copy operator
		IPT_ROSInterface& operator = (const IPT_ROSInterface&) = delete;
		// Delete copy constructor
		IPT_ROSInterface(const IPT_ROSInterface&) = delete;

		static IPT_ROSInterface* GetInstance(int argc = 0, char * argv[] = nullptr, const std::string & node_name = "ipt_node");

		void PublishPose(const geometry_msgs::PoseStamped& pose);
		void GetEstimatedPose(geometry_msgs::PoseStamped& pose) const;
	};
}

#endif // IPT_ROS_INTERFACE_INCLUDED
