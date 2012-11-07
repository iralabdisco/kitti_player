#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <fstream>
#include <iostream>
#include <sstream>
std::string path;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher map_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_in", 10, true);

  ros::Rate loop_rate(1);

  path = argv[0];
  std::string::size_type pos = path.find_last_of( "\\/" );
  path = path.substr( 0, pos);

  std::string infile = path+"/../test/kitty/velodyne_points/data/0000000000.bin";

  std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
  if(!input.good()){
    std::cerr << "Could not read file: " << infile << std::endl;
    exit(EXIT_FAILURE);
  }
  input.seekg(0, std::ios::beg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
  points->header.frame_id = "/map";

  int i;
  for (i=0; input.good() && !input.eof(); i++) {
    pcl::PointXYZI point;
    input.read((char *) &point.x, 3*sizeof(float));
    input.read((char *) &point.intensity, sizeof(float));
    points->push_back(point);
  }
  input.close();

  while (ros::ok())
  {
    points->header.stamp = ros::Time();
    map_pub.publish(points);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}