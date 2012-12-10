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
#include "Random.h"
#include "randomUtil.h"
#include "bulletUtil.h"
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
std::string gt_camera_ref_frame;
std::string camera_ref_zero_frame;
std::string robot_frame;
std::string gt_robot_frame;
std::string odom_frame;

tf::Transform readTransform;
ros::Rate* loop_rate;

btTransform oldPose;
btTransform last_uncertain_pose;

void callback(kitti_player::kitti_playerConfig &config, uint32_t level)
{

  if (config.sequence != myConfig.sequence || config.start == false)
  {
    oldPose.setIdentity();
    last_uncertain_pose.setIdentity();
    frame_count = 0;
    delete poseFile;
    poseFile = NULL;
    
  }
  if (config.loop_rate != myConfig.loop_rate)
  {
    delete loop_rate;
    loop_rate = new ros::Rate(config.loop_rate);
  }
    
  myConfig = config;

  if (config.publish)
  {
    config.publish = false;
  }

}
/** 
 * \brief Reads the transform from the dataset
*/
void read_pose()
{
  if (!poseFile)
  {
  // std::cout << "new" << pose_path << std::endl;
    poseFile = new std::ifstream(pose_path.c_str());
  
  }
  if( !poseFile->good()){
    std::cerr << "Could not read file: " << *poseFile << std::endl;
  }
  else
  {
    std::string line;
    getline(*poseFile, line);

    // std::cout << line << std::endl;

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of("\t "));
    std::vector<double> values;

    for ( std::vector<std::string>::iterator it=strs.begin() ; it < strs.end(); it++ )
    {
      double numb;
      std::istringstream ( *it ) >> numb;
      values.push_back(numb);
      // std::cout << numb <<std::endl;
    }

    readTransform.setOrigin( tf::Vector3(values[3], values[7], values[11]) );
    readTransform.setBasis( btMatrix3x3(values[0], values[1], values[2],values[4], values[5], values[6],values[8], values[9], values[10]) );
  }
}

void publish_uncertain_pose()
{
  if (myConfig.generateUncertain)
  {
    static tf::TransformBroadcaster br;
    // publish the groundtruth

    btTransform odometryDelta;
    odometryDelta.mult(oldPose.inverse(),readTransform);
    // odometryDelta.getRotation().normalize();
    if (!odometryDelta.getOrigin().isZero())
    {
      if (myConfig.alphaPose!=0)
      {
        odometryDelta.getOrigin() += odometryDelta.getOrigin()*btVector3( RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaPose),
                                                RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaPose),
                                                RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaPose));
      }
      if (myConfig.alphaOrientation!=0)
      {
        double roll, pitch, yaw;
        odometryDelta.getBasis().getRPY(roll, pitch, yaw);
        roll += roll * RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaOrientation);
        pitch += pitch * RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaOrientation);
        yaw += yaw * RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaOrientation);
        odometryDelta.getBasis().setRPY(roll, pitch, yaw);
        // odometryDelta.getRotation().normalize();
      }

      last_uncertain_pose *= odometryDelta;
      // last_uncertain_pose.getRotation().normalize();
    }

    br.sendTransform(tf::StampedTransform(last_uncertain_pose, ros::Time::now(), camera_ref_zero_frame, camera_ref_frame));

    oldPose = readTransform;
    printBullet(last_uncertain_pose, "last unc pos");
    printBullet(odometryDelta, "delta");
  }
}

/** 
 * \brief Publish the groundtruth of the pose
*/
void publish_pose_groundtruth()
{
  if (myConfig.generateGroundtruth)
  {
    static tf::TransformBroadcaster br;
    // publish the groundtruth
    br.sendTransform(tf::StampedTransform(readTransform, ros::Time::now(), camera_ref_zero_frame, gt_camera_ref_frame));
    printBullet(readTransform, "read transf: gt");

  }
}
/** 
 * \brief Publish the reference frames of the vehicle and the odom frames
*/
void publish_static_transforms()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
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
  br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), gt_camera_ref_frame, gt_robot_frame));
}

/** 
 * \brief Publish the velodyne data
*/
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
    points->header.frame_id = robot_frame; // Temp fix for easier interaction
    // points->header.frame_id = laser_frame;

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

  ros::NodeHandle n;
  loop_rate = new ros::Rate(1);
  Random random;
  RandGlobal::setRandomInstance(random);
  last_uncertain_pose.setIdentity();
  oldPose.setIdentity();

  dynamic_reconfigure::Server<kitti_player::kitti_playerConfig> srv;
  dynamic_reconfigure::Server<kitti_player::kitti_playerConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);


  ros::Publisher map_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_in", 10, true);

  path = argv[0];
  std::string::size_type pos = path.find_last_of( "\\/" );
  path = path.substr( 0, pos);

  n.param<std::string>("camera_ref_frame",camera_ref_frame,"/camera_ref");
  n.param<std::string>("gt_camera_ref_frame",gt_camera_ref_frame,"/gt_camera_ref");
  n.param<std::string>("camera_ref_zero_frame",camera_ref_zero_frame,"/camera_ref_zero");
  n.param<std::string>("laser_frame",laser_frame,"/laser");
  n.param<std::string>("robot_frame",robot_frame,"/base_link");
  n.param<std::string>("gt_robot_frame",gt_robot_frame,"/gt_base_link");
  n.param<std::string>("odom_frame",odom_frame,"/odom");


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
      // std::cout << sequence_path << std::endl;

      // std::cout << pose_path << std::endl;
      ++frame_count;
      publish_velodyne(map_pub, sequence_path);
      read_pose();
      if (myConfig.publish)
      {
        myConfig.publish = false;
      }
    }
    publish_static_transforms();
    publish_pose_groundtruth();
    publish_uncertain_pose();

    ros::spinOnce();

    loop_rate->sleep();
  }


  return 0;
}