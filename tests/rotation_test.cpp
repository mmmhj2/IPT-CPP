/*
* File : rotation_test.cpp
* Author : Li
* Date : 2022/05/02
* Description : Unit test for conversion functions
*/

#include "tools.h"
#include <random>

using namespace cv;

int main(int argc, char** argv)
{
	double roll, pitch, yaw;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> dist(-3.14159, 3.14159);

	roll = dist(generator);
	pitch = dist(generator);
	yaw = dist(generator);

	cv::Vec3d euler = { roll, pitch, yaw };

	auto quat = ipt::euler_2_quaternion(euler);
	auto rotMat = ipt::quaternion_2_rotation(quat);
	
	std::cout << "Euler angle = " << std::endl << euler << std::endl ;
	std::cout << "Quaterions = " << std::endl << quat << std::endl;
	std::cout << "Rotation Matrix = " << std::endl << rotMat << std::endl;
	std::cout << "Recalculated quaterions = " << std::endl << ipt::rotation_2_quaternion(rotMat) << std::endl;
	std::cout << "Recalculated euler using matrix = " << std::endl << ipt::rotation_2_euler(rotMat) << std::endl;
}
