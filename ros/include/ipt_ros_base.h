#ifndef IPT_ROS_BASE_H
#define IPT_ROS_BASE_H

#include <memory>
#include <ros/ros.h>

namespace ipt
{
	class IPT_ROSBase
	{
	protected:
		std::unique_ptr <ros::NodeHandle> nh, pnh;
		std::unique_ptr <ros::Rate> rate;
	public:
		IPT_ROSBase();

		// A pure virtual deconstructor ensures the class is abstract
		virtual ~IPT_ROSBase() = 0;

		// Delete copy semantics due to unique_ptr
		IPT_ROSBase(const IPT_ROSBase&) = delete;
		void operator = (const IPT_ROSBase&) = delete;

		void WaitAndSpin();
		ros::NodeHandle* GetPrivateNH() const;
	};
}

#endif
