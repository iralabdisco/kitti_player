#include "bulletUtil.h"

void printBullet(tf::Vector3 vector, const char* msg) {
	std::cout << msg<<": ["  << vector.x() <<"," << vector.y() << "," <<vector.z() << "]"<<std::endl;
}

void printBullet(tf::Transform transform, const char* msg) {
	std::cout << msg << std::endl;
	printBullet(transform.getOrigin(), "position");
	printBullet(transform.getBasis().getColumn(0),"orientation");
}

std::string bulletString(tf::Vector3 vector){
	std::stringstream ss;
	ss.str("");
    ss << "["  << vector.x() <<"," << vector.y() << "," <<vector.z() << "]";
	return ss.str();
}

std::string bulletString(tf::Transform transform){
	std::stringstream ss;
	ss.str("");
	ss << "[" << bulletString(transform.getOrigin()) << ", ";
	ss << bulletString(transform.getBasis().getColumn(0)) << "]";
	return ss.str();
}

