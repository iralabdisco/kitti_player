#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <fstream>
#include <iostream>
#include <sstream>

#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>
#include <boost/tokenizer.hpp>

#include <dynamic_reconfigure/server.h>
#include <kitti_player/kitti_playerConfig.h>
#include <boost/algorithm/string.hpp>
#include "Random.h"
#include "randomUtil.h"
#include "bulletUtil.h"

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace tf;

string path;
string sequence_path;
string pose_path;
kitti_player::kitti_playerConfig myConfig;
int frame_count = 0;
string sequence;

tf::Transform pose;
ifstream* poseFile = NULL;

string laser_frame;
string gt_laser_frame;
string camera_ref_frame;
string gt_camera_ref_frame;
string camera_ref_zero_frame;
string robot_frame;
string gt_robot_frame;
string odom_frame;

tf::Transform readTransform;
ros::Rate* loop_rate;

tf::Transform oldPose;
tf::Transform last_uncertain_pose;
bool shoudlExit;
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
  // cout << "new" << pose_path << endl;
    poseFile = new ifstream(pose_path.c_str());
  
  }
  if( !poseFile->good()){
    if (myConfig.exitIfNotFound)
    {
      shoudlExit = true;
    }
    cerr << "Could not read file: " << *poseFile << endl;
  }
  else
  {
    string line;
    getline(*poseFile, line);

    // cout << line << endl;

    vector<string> strs;
    boost::split(strs, line, boost::is_any_of("\t "));
    vector<double> values;

    for ( vector<string>::iterator it=strs.begin() ; it < strs.end(); it++ )
    {
      double numb;
      istringstream ( *it ) >> numb;
      values.push_back(numb);
      // cout << numb <<endl;
    }

    readTransform.setOrigin( tf::Vector3(values[3], values[7], values[11]) );
    readTransform.setBasis( tf::Matrix3x3(values[0], values[1], values[2],values[4], values[5], values[6],values[8], values[9], values[10]) );
  }
}

void publish_uncertain_pose()
{
  if (myConfig.generateUncertain)
  {
    static tf::TransformBroadcaster br;
    // publish the groundtruth

    tf::Transform odometryDelta;
    odometryDelta.mult(oldPose.inverse(),readTransform);
    // odometryDelta.getRotation().normalize();
    if (!odometryDelta.getOrigin().isZero())
    {
      if (myConfig.alphaPose!=0)
      {
        odometryDelta.getOrigin() += odometryDelta.getOrigin()*tf::Vector3( RandGlobal::getRandomInstance().sampleNormalDistribution(myConfig.alphaPose),
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
    // printBullet(last_uncertain_pose, "last unc pos");
    // printBullet(odometryDelta, "delta");
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
    // printBullet(readTransform, "read transf: gt");

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
  transform.setOrigin(tf::Vector3(0.76,0,1.73));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_frame, laser_frame));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), gt_robot_frame, gt_laser_frame));


  transform.setIdentity();
  transform.setOrigin(tf::Vector3(1.03,0,1.65));

  tf::Quaternion rotation;
  tf::Quaternion rot1;
  tf::Quaternion rot2;

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
void publish_velodyne(ros::Publisher &pub, string infile)
{
  fstream input(infile.c_str(), ios::in | ios::binary);
  if(!input.good()){
    cerr << "Could not read file: " << infile << endl;
  }
  else
  {
    input.seekg(0, ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    //workaround for the PCL headers... http://wiki.ros.org/hydro/Migration#PCL
    sensor_msgs::PointCloud2 pc2;
    pc2.header.frame_id=laser_frame;

    //old line
    //points->header.frame_id = laser_frame;

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
      pcl::PointXYZI point;
      input.read((char *) &point.x, 3*sizeof(float));
      input.read((char *) &point.intensity, sizeof(float));
      points->push_back(point);
    }
    input.close();

    //workaround for the "old line" below...
    pc2.header.stamp=ros::Time::now();
    points->header = pcl_conversions::toPCL(pc2.header);

    // old line
    //points->header.stamp = ros::Time::now();

    pub.publish(points);
  }
}


int main(int argc, char **argv)
{
  shoudlExit = false;
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

  path = argv[1];

  string::size_type pos = path.find_last_of( "\\/" );
  path = path.substr( 0, pos);

  n.param<string>("camera_ref_frame",camera_ref_frame,"/camera_ref");
  n.param<string>("gt_camera_ref_frame",gt_camera_ref_frame,"/gt_camera_ref");
  n.param<string>("camera_ref_zero_frame",camera_ref_zero_frame,"/camera_ref_zero");
  n.param<string>("laser_frame",laser_frame,"/laser_frame");
  n.param<string>("gt_laser_frame",gt_laser_frame,"/gt_laser_frame");
  n.param<string>("robot_frame",robot_frame,"/robot_frame");
  n.param<string>("gt_robot_frame",gt_robot_frame,"/gt_robot_frame");
  n.param<string>("odom_frame",odom_frame,"/odom");


  // sequence_path = path+"/../dataset/sequences/01/velodyne/0000000000.bin";
  // pose_path = path+"/../dataset/poses/01.txt";

  readTransform.setIdentity();

  while (ros::ok() && !shoudlExit)
  {
    if (myConfig.start && (myConfig.continuous || myConfig.publish))
    {

      stringstream ss;
      ss << setfill('0') << setw(2) << myConfig.sequence;
     
      ss >> sequence;
      pose_path = path+"/dataset/poses/"+sequence+".txt";

      ss.clear();
      // the number is converted to string with the help of stringstream
      ss << setfill('0') << setw(6) << frame_count;
      string frame;
      ss >> frame;
      sequence_path = path+"/dataset/sequences/"+sequence+"/velodyne/"+frame+".bin";

      cout << sequence_path << endl;
      cout << pose_path << endl;

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
