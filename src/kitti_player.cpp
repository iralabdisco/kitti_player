// ###############################################################################################
// ###############################################################################################
// ###############################################################################################

/*
 * This node aims to play the kitti data into ROS.
 * This repository is a catkin upgrade from the original work by Francesco Sacchi
 * see http://irawiki.disco.unimib.it/irawiki/index.php/Kitti_Player for the original version.
 *
 * Porting to ROS-Hydro and other minor mods: Augusto Luis Ballardini ballardini@disco.unimib.it
 *
 * https://github.com/iralabdisco/kitti_player
 *
 */

// COMPILE-TIME CONFIGURATIONS //

//#define PRINT_GT_ROBOT_FRAME 0  // if activated, you'll see the current tranform+rotation of the
// gt_robot_frame w.r.t. odom_frame

//#define PRINT_CURRENT_INPUT 1   // if activated, put on screen the current file readed

// COMPILE-TIME CONFIGURATIONS //


// ###############################################################################################
// ###############################################################################################
// ###############################################################################################

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <fstream>
#include <iostream>
#include <sstream>

#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/tokenizer.hpp>

#include <dynamic_reconfigure/server.h>
#include <kitti_player/kitti_playerConfig.h>
#include <boost/algorithm/string.hpp>
#include "Random.h"
#include "randomUtil.h"
#include "bulletUtil.h"

#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h" //for initialpose

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace tf;

namespace po = boost::program_options;

struct kitti_player_options
{
    string  path;
    float   frequency;      // publisher frequency. 1 > Kitti default 10Hz
    bool    all_data;       // publish everything
    bool    velodyne;       // publish velodyne point clouds /as PCL
    bool    gps;            // publish GPS sensor_msgs/NavSatFix    message
    bool    imu;            // publish IMU sensor_msgs/Imu Message  message
    bool    grayscale;      // publish
    bool    color;          // publish

};


string path;
string sequence_path;
string pose_path;
kitti_player::kitti_playerConfig myConfig;
int frame_count = 0;

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

tf::Transform oldPose;
tf::Transform last_uncertain_pose;

bool    exitIfNotFound;
double  alphaOrientation;
double  alphaPose;
bool    continuous;
bool    generateGroundtruth;
bool    generateUncertain;
bool    publish;
string  sequence;
bool    player_start;
ros::Rate* loop_rate;


void callback(kitti_player::kitti_playerConfig &config, uint32_t level)
{

    if (config.sequence != atoi(sequence.c_str()) || config.start == false)
    {
        oldPose.setIdentity();
        last_uncertain_pose.setIdentity();
        frame_count = 0;
        delete poseFile;
        poseFile = NULL;

    }
    if (config.loop_rate != loop_rate->cycleTime().toSec())
    {
        delete loop_rate;
        loop_rate = new ros::Rate(config.loop_rate);
    }

    exitIfNotFound      = config.exitIfNotFound;
    alphaOrientation    = config.alphaOrientation;
    alphaPose           = config.alphaPose;
    continuous          = config.continuous;
    generateGroundtruth = config.generateGroundtruth;
    generateUncertain   = config.generateUncertain;
    sequence            = boost::lexical_cast<string>(config.sequence);
    player_start        = config.start;

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

    if( !poseFile->good())
    {
        if (exitIfNotFound)
        {
            exitIfNotFound = true;
        }
        cerr << "Could not read file: " << *poseFile << endl;
    }
    else
    {
        string line;
        getline(*poseFile, line);

        cout << line << " " << *poseFile << endl;

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
    if (generateUncertain)
    {
        static tf::TransformBroadcaster br;
        // publish the groundtruth

        tf::Transform odometryDelta;
        odometryDelta.mult(oldPose.inverse(),readTransform);
        // odometryDelta.getRotation().normalize();
        if (!odometryDelta.getOrigin().isZero())
        {
            if (alphaPose!=0)
            {
                odometryDelta.getOrigin() += odometryDelta.getOrigin()*tf::Vector3( RandGlobal::getRandomInstance().sampleNormalDistribution(alphaPose),
                                                                                    RandGlobal::getRandomInstance().sampleNormalDistribution(alphaPose),
                                                                                    RandGlobal::getRandomInstance().sampleNormalDistribution(alphaPose));
            }
            if (alphaOrientation!=0)
            {
                double roll, pitch, yaw;
                odometryDelta.getBasis().getRPY(roll, pitch, yaw);
                roll += roll * RandGlobal::getRandomInstance().sampleNormalDistribution(alphaOrientation);
                pitch += pitch * RandGlobal::getRandomInstance().sampleNormalDistribution(alphaOrientation);
                yaw += yaw * RandGlobal::getRandomInstance().sampleNormalDistribution(alphaOrientation);
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
    if (generateGroundtruth)
    {
        static tf::TransformBroadcaster br;
        // publish the groundtruth
        br.sendTransform(tf::StampedTransform(readTransform, ros::Time::now(), camera_ref_zero_frame, gt_camera_ref_frame));
        // printBullet(readTransform, "read transf: gt");

        // print (in 1 line) the gt_camera_ref_frame  w.r.t. starting point (camera_ref_zero_frame)
        //    printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",   readTransform.getOrigin().getZ(),
        //                                               readTransform.getOrigin().getY(),
        //                                               readTransform.getOrigin().getX(),
        //                                               readTransform.getRotation().getW(),
        //                                               readTransform.getRotation().getX(),
        //                                               readTransform.getRotation().getY(),
        //                                               readTransform.getRotation().getZ());
    }
}
/** 
 * \brief Publish the reference frames of the vehicle and the odom frames
*/
void publish_static_transforms()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform,offset;

    transform.setIdentity();
    transform.setOrigin(tf::Vector3(0.76,0,1.73));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_frame, laser_frame));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), gt_robot_frame, gt_laser_frame));

    transform.setIdentity();
    transform.setOrigin(tf::Vector3(1.03,0,1.65));

    //  offset.setIdentity();
    //  offset.setOrigin( tf::Vector3(10,10,0) );
    //  transform.mult(offset,transform);

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
        cout << infile << endl;
        input.seekg(0, ios::beg);

        pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

        int i;
        for (i=0; input.good() && !input.eof(); i++) {
            pcl::PointXYZI point;
            input.read((char *) &point.x, 3*sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();

        //workaround for the PCL headers... http://wiki.ros.org/hydro/Migration#PCL
        sensor_msgs::PointCloud2 pc2;

        //uncertain frame link
        if (generateUncertain)
        {
            pc2.header.frame_id=laser_frame;
            pc2.header.stamp=ros::Time::now();
            points->header = pcl_conversions::toPCL(pc2.header);
            pub.publish(points);    //publish the PCL as
        }

        //ground truth frame link
        if (generateGroundtruth)
        {
            pc2.header.frame_id=gt_laser_frame;
            pc2.header.stamp=ros::Time::now();
            points->header = pcl_conversions::toPCL(pc2.header);
            pub.publish(points);
        }
    }
}



int main(int argc, char **argv)
{
    kitti_player_options options;

    DIR *dir;
    struct dirent *ent;
    std::string frames_dir;
    std::vector<std::string> images;


    // Declare the supported options.
    po::options_description desc("Allowed options",150);
    desc.add_options()
        ("help"                                                                        , "help message")
        ("directory,d", po::value<string>()->required()                                , "path to the kitti dataset Directory")
        ("frequency,f", po::value<float>(&options.frequency)->default_value(1.0)       , "set replay Frequency")
        ("all      ,a", po::value<bool> (&options.all_data)->default_value(true)       , "replay All data")
        ("velodyne ,v", po::value<bool> (&options.velodyne)->default_value(false)      , "replay Velodyne data")
        ("gps      ,g", po::value<bool> (&options.gps)->default_value(false)           , "replay Gps data")
        ("imu      ,i", po::value<bool> (&options.imu)->default_value(false)           , "replay Imu data")
        ("grayscale,G", po::value<bool> (&options.grayscale)->default_value(false)     , "replay Stereo Grayscale images")
        ("color    ,C", po::value<bool> (&options.color)->default_value(false)         , "replay Stereo Color images")
    ;

    po::variables_map vm;
    try
    {
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        po::notify(vm);
    }
    catch(...)
    {
        cout << "kitti_player, a player for KITTI raw datasets" << endl;
        cout << "Datasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php" << endl << endl;

        cout << "kitti_player needs a directory tree like the following:";
        cout << "└── 2011_09_26_drive_0001_sync" << endl;
        cout << "    ├── image_00              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_01              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_03              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    └── velodyne_points       " << endl;
        cout << "        └── data              " << endl << endl;

        cerr << desc << endl;
        return -1;
    }

    if (vm.count("help")) {
        cout << "Kitti_player, a player for KITTI raw datasets" << endl;
        cout << "Datasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php" << endl << endl;

        cout << "kitti_player needs a directory tree like the following:";
        cout << "└── 2011_09_26_drive_0001_sync" << endl;
        cout << "    ├── image_00              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_01              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── image_03              " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── data              " << endl;
        cout << "    └── velodyne_points       " << endl;
        cout << "        └── data              " << endl << endl;

        cout << desc << endl;
        return 1;
    }

    if ((dir = opendir(vm["directory"].as<string>().c_str())) != NULL)
    {
        while((ent = readdir(dir)) != NULL)
        {
            std::string filename(ent->d_name);
            if(filename.rfind(".png") != std::string::npos)
                images.push_back(frames_dir + filename);
        }
        closedir (dir);

        if(images.size() == 0){
            ROS_ERROR("Incorrect directory tree, use --help for details");
            return -1;
        }
    } else
    {
        ROS_ERROR("Couldn't open the directory\t<%s>", vm["directory"].as<string>().c_str());
        return 1;
    }





    return -1;

    exitIfNotFound = false;
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

    ros::Publisher map_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_in", 1, true);
    ros::Publisher initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    path = argv[1];

    string::size_type pos = path.find_last_of( "\\/" );
    path = path.substr( 0, pos);

    n.param<string>("camera_ref_zero_frame",camera_ref_zero_frame,"/camera_ref_zero_frame");
    n.param<string>("odom_frame",odom_frame,"/odom"); //odom

    n.param<string>("gt_camera_ref_frame",gt_camera_ref_frame,"/gt_camera_ref");
    n.param<string>("gt_laser_frame",gt_laser_frame,"/laser_frame");
    n.param<string>("gt_robot_frame",gt_robot_frame,"/cart_frame");

    n.param<string>("camera_ref_frame",camera_ref_frame,"/BIASED_camera_ref");
    n.param<string>("laser_frame",laser_frame,"/BIASED_laser_frame");
    n.param<string>("robot_frame",robot_frame,"/BIASED_robot_frame");

    readTransform.setIdentity();

    while (ros::ok() && !exitIfNotFound)
    {
        stringstream ss;
        ss << setfill('0') << setw(2) << sequence;

        ss >> sequence;
        pose_path = path+"/dataset/poses/"+sequence+".txt";

        ss.clear();
        // the number is converted to string with the help of stringstream
        ss << setfill('0') << setw(10) << frame_count;
        string frame;
        ss >> frame;
        sequence_path = path+"/dataset/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/"+frame+".bin";

        #if defined (PRINT_CURRENT_INPUT)
        {
            cout << sequence_path << endl;
            cout << pose_path << endl;
        }
        #endif

        publish_velodyne(map_pub, sequence_path);

        if (player_start && (continuous || publish))
        {
            read_pose();
            if (publish)
            {
                publish = false;
            }
            ++frame_count;
        }

        // FOR GT-PRINT PURPOSES ** BEGIN **
        #if defined (PRINT_GT_ROBOT_FRAME)
        {
            static tf::TransformListener listener;
            tf::StampedTransform stamped_transform;
            if(listener.waitForTransform (odom_frame,gt_robot_frame,ros::Time(0),ros::Duration(0.1)))
                listener.lookupTransform(odom_frame,gt_robot_frame,ros::Time(0),stamped_transform);

            printf("%s\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",sequence_path.c_str(),
                   stamped_transform.getOrigin().getX(),
                   stamped_transform.getOrigin().getY(),
                   stamped_transform.getOrigin().getZ(),
                   stamped_transform.getRotation().getW(),
                   stamped_transform.getRotation().getX(),
                   stamped_transform.getRotation().getY(),
                   stamped_transform.getRotation().getZ());
        }
        #endif
        // FOR GT-PRINT PURPOSES ** END **


        publish_static_transforms();
        publish_pose_groundtruth();
        publish_uncertain_pose();

        ros::spinOnce();

        loop_rate->sleep();
    }


    return 0;
}
