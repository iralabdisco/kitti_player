#include "bulletUtil.h"

void printBullet(btVector3 vector, const char* msg) {
	std::cout << msg<<": ["  << vector.x() <<"," << vector.y() << "," <<vector.z() << "]"<<std::endl;
}

void printBullet(btTransform transform, const char* msg) {
	std::cout << msg << std::endl;
	printBullet(transform.getOrigin(), "position");
	printBullet(transform.getBasis().getColumn(0),"orientation");
}

std::string bulletString(btVector3 vector){
	std::stringstream ss;
	ss.str("");
    ss << "["  << vector.x() <<"," << vector.y() << "," <<vector.z() << "]";
	return ss.str();
}

std::string bulletString(btTransform transform){
	std::stringstream ss;
	ss.str("");
	ss << "[" << bulletString(transform.getOrigin()) << ", ";
	ss << bulletString(transform.getBasis().getColumn(0)) << "]";
	return ss.str();
}

