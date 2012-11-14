#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <fstream>
#include <iostream>
#include <sstream>

#include "LinearMath/btTransform.h"
#include <tf/transform_broadcaster.h>
#include <boost/tokenizer.hpp>

#include <dynamic_reconfigure/server.h>
#include <kitti_player/kitti_playerConfig.h>
#include <boost/algorithm/string.hpp>

std::string path;
std::string sequence_path;
std::string pose_path;
kitti_player::kitti_playerConfig myConfig;
int frame_count = 0;
std::string sequence;

btTransform pose;
std::ifstream* poseFile = NULL;

std::string laser_frame;
std::string camera_ref_frame;
std::string camera_ref_zero_frame;
std::string robot_frame;
std::string odom_frame;

tf::Transform readTransform;

void callback(kitti_player::kitti_playerConfig &config, uint32_t level)
{

  if (config.sequence != myConfig.sequence || config.start == false)
  {
    frame_count = 0;
    delete poseFile;
    poseFile = NULL;
    
  }
    
  myConfig = config;

  if (config.publish)
  {
    config.publish = false;
  }
}

void publish_pose()
{
  if (!poseFile)
  {
  std::cout << "new" << pose_path << std::endl;
    poseFile = new std::ifstream(pose_path.c_str());
  
  }
  if( !poseFile->good()){
    std::cerr << "Could not read file: " << *poseFile << std::endl;
  }
  else
  {
    std::string line;
    getline(*poseFile, line);

    std::cout << line << std::endl;

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of("\t "));
    std::vector<double> values;

    for ( std::vector<std::string>::iterator it=strs.begin() ; it < strs.end(); it++ )
    {
      double numb;
      std::istringstream ( *it ) >> numb;
      values.push_back(numb);
      std::cout << numb <<std::endl;
    }



    readTransform.setOrigin( tf::Vector3(values[3], values[7], values[11]) );
    
    readTransform.setBasis( btMatrix3x3(values[0], values[1], values[2],values[4], values[5], values[6],values[8], values[9], values[10]) );
    // transform.setIdentity();
    

  }
  
}

void publish_transforms()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  br.sendTransform(tf::StampedTransform(readTransform, ros::Time::now(), camera_ref_zero_frame, camera_ref_frame));
  
  transform.setIdentity();
  transform.setOrigin(btVector3(0.76,0,1.73));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_frame, laser_frame));

  transform.setIdentity();
  transform.setOrigin(btVector3(1.03,0,1.65));
  btQuaternion rotation;

  btQuaternion rot1;
  btQuaternion rot2;

  rot1.setEuler(M_PI/2,0,0);
  rot2.setEuler(0,-M_PI/2,0);
  // rotation = rot1;
  rotation = rot2*rot1;
  
  transform.setRotation(rotation.normalize());
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, camera_ref_zero_frame));
  br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), camera_ref_frame, robot_frame));
}

void publish_velodyne(ros::Publisher &pub, std::string infile)
{
  std::fstream input(infile.c_str(), std::ios::in | std::ios::binary);
  if(!input.good()){
    std::cerr << "Could not read file: " << infile << std::endl;
  }
  else
  {
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);
    points->header.frame_id = laser_frame;

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
      pcl::PointXYZI point;
      input.read((char *) &point.x, 3*sizeof(float));
      input.read((char *) &point.intensity, sizeof(float));
      points->push_back(point);
    }
    input.close();
    points->header.stamp = ros::Time::now();
    pub.publish(points);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kitti_player");

  dynamic_reconfigure::Server<kitti_player::kitti_playerConfig> srv;
  dynamic_reconfigure::Server<kitti_player::kitti_playerConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::NodeHandle n;

  ros::Publisher map_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_in", 10, true);

  ros::Rate loop_rate(1);

  path = argv[0];
  std::string::size_type pos = path.find_last_of( "\\/" );
  path = path.substr( 0, pos);

  n.param<std::string>("camera_ref_frame",camera_ref_frame,"camera_ref");
  n.param<std::string>("camera_ref_zero_frame",camera_ref_zero_frame,"camera_ref_zero");
  n.param<std::string>("laser_frame",laser_frame,"laser");
  n.param<std::string>("robot_frame",robot_frame,"base_link");
  n.param<std::string>("odom_frame",odom_frame,"odom");


  // sequence_path = path+"/../dataset/sequences/01/velodyne/0000000000.bin";
  // pose_path = path+"/../dataset/poses/01.txt";

  readTransform.setIdentity();

  while (ros::ok())
  {
    if (myConfig.start && (myConfig.continuous || myConfig.publish))
    {

      std::stringstream ss;
      ss << std::setfill('0') << std::setw(2) << myConfig.sequence; 
     
      ss >> sequence;
      pose_path = path+"/../dataset/poses/"+sequence+".txt";
              
      ss.clear();
      // the number is converted to string with the help of stringstream
      ss << std::setfill('0') << std::setw(6) << frame_count; 
      std::string frame;
      ss >> frame;
      sequence_path = path+"/../dataset/sequences/"+sequence+"/velodyne/"+frame+".bin";
      std::cout << sequence_path << std::endl;

      std::cout << pose_path << std::endl;
      ++frame_count;
      publish_velodyne(map_pub, sequence_path);
      publish_pose();
      if (myConfig.publish)
      {
        myConfig.publish = false;
      }
    }
    publish_transforms();

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}