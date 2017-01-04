/*
 * EtalotnageCI.cpp
 *
 *  Created on: 2 déc. 2016
 *      Author: amaury
 */


#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "serial_amaury/CI_msg.h"

using std::vector;

int countAcc=0;
int countGyr=0;
int countMag=0;

vector<float> moyAcc, moyGyr, moyMag;


void AccCallback(const serial_amaury::CI_msg::ConstPtr& msg)
{
	if(countAcc==0){
		moyAcc.push_back(msg->x);
		moyAcc.push_back(msg->y);
		moyAcc.push_back(msg->z);
		countAcc++;
	}
	else{
		moyAcc[0]=(moyAcc[0]*countAcc + msg->x)/(countAcc+1);
		moyAcc[1]=(moyAcc[1]*countAcc + msg->y)/(countAcc+1);
		moyAcc[2]=(moyAcc[2]*countAcc + msg->z)/(countAcc+1);
		countAcc++;
	}
	
	ROS_INFO("Moyenne acc : x=%f ; y=%f ; z=%f",moyAcc[0],moyAcc[1],moyAcc[2]);
}


void GyrCallback(const serial_amaury::CI_msg::ConstPtr& msg)
{

	if(countGyr==0){
		moyGyr.push_back(msg->x);
		moyGyr.push_back(msg->y);
		moyGyr.push_back(msg->z);
		countGyr++;
	}
	else{
		moyGyr[0]=(moyGyr[0]*countGyr + msg->x)/(countGyr+1);
		moyGyr[1]=(moyGyr[1]*countGyr + msg->y)/(countGyr+1);
		moyGyr[2]=(moyGyr[2]*countGyr + msg->z)/(countGyr+1);
		countGyr++;
	}
	//ROS_INFO("Moyenne gyr : x=%f ; y=%f ; z=%f",moyGyr[0],moyGyr[1],moyGyr[2]);
}

void MagCallback(const serial_amaury::CI_msg::ConstPtr& msg)
{
	
	if(countMag==0){
		moyMag.push_back(msg->x);
		moyMag.push_back(msg->y);
		moyMag.push_back(msg->z);
		countMag++;
	}
	else{
		moyMag[0]=(moyMag[0]*countMag + msg->x)/(countMag+1);
		moyMag[1]=(moyMag[1]*countMag + msg->y)/(countMag+1);
		moyMag[2]=(moyMag[2]*countMag + msg->z)/(countMag+1);
		countMag++;
	}
	//ROS_INFO("Moyenne mag : x=%f ; y=%f ; z=%f",moyMag[0],moyMag[1],moyMag[2]);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "Etallonage");
  
  ros::NodeHandle n;
  
  //ros::Rate loop_rate(2);
  
	ros::Subscriber sub_acc = n.subscribe("CIAcc", 1000, AccCallback);
	ros::Subscriber sub_gyr = n.subscribe("CIGyr", 1000, GyrCallback);
	ros::Subscriber sub_mag = n.subscribe("CIMag", 1000, MagCallback);

  ros::spin();

  return 0;

}




