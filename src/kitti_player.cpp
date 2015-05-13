// redmine usage: This commit refs #388 @2h

// ###############################################################################################
// ###############################################################################################
// ###############################################################################################

/*
 * This node aims to play the kitti data into ROS.
 * This repository is a catkin upgrade from the original work by Francesco Sacchi
 * see http://irawiki.disco.unimib.it/irawiki/index.php/Kitti_Player for the original versionode.
 *
 * Porting to ROS-Hydro and other minor mods: Augusto Luis Ballardini ballardini@disco.unimib.it
 *
 * https://github.com/iralabdisco/kitti_player
 *
 * WARNING: this package is using some C++11
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

#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <boost/tokenizer.hpp>

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
    bool    viewer;

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


int getCalibration(string dir_root, string camera_name, double* K,std::vector<double> & D,double *R,double* P){
/*
 *   from: http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip
 *   calib_cam_to_cam.txt: Camera-to-camera calibration
 *   --------------------------------------------------
 *
 *     - S_xx: 1x2 size of image xx before rectification
 *     - K_xx: 3x3 calibration matrix of camera xx before rectification
 *     - D_xx: 1x5 distortion vector of camera xx before rectification
 *     - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
 *     - T_xx: 3x1 translation vector of camera xx (extrinsic)
 *     - S_rect_xx: 1x2 size of image xx after rectification
 *     - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
 *     - P_rect_xx: 3x4 projection matrix after rectification
*/

    //    double K[9];         // Calibration Matrix
    //    double D[5];         // Distortion Coefficients
    //    double R[9];         // Rectification Matrix
    //    double P[12];        // Projection Matrix Rectified (u,v,w) = P * R * (x,y,z,q)


    string calib_cam_to_cam=dir_root+"calib_cam_to_cam.txt";
    ifstream file_c2c(calib_cam_to_cam.c_str());
    if (!file_c2c.is_open())
        return false;

    ROS_INFO_STREAM("Reading camera" << camera_name << " calibration from " << calib_cam_to_cam);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep{" "};

    vector< string > vec;
    string line="";
    char index=0;
    tokenizer::iterator token_iterator;

    while (getline(file_c2c,line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line,sep);

        // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("K_")+camera_name+string(":"))).c_str())==0) //Calibration Matrix
        {
            index=0; //should be 9 at the end
            ROS_DEBUG_STREAM("K_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
//                std::cout << *token_iterator << '\n';
                K[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("D_")+camera_name+string(":"))).c_str())==0) //Distortion Coefficients
        {
            index=0; //should be 5 at the end
            ROS_DEBUG_STREAM("D_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
//                std::cout << *token_iterator << '\n';
                D[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("R_")+camera_name+string(":"))).c_str())==0) //Rectification Matrix
        {
            index=0; //should be 12 at the end
            ROS_DEBUG_STREAM("R_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
//                std::cout << *token_iterator << '\n';
                R[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator=tok.begin();
        if (strcmp((*token_iterator).c_str(),((string)(string("P_rect_")+camera_name+string(":"))).c_str())==0) //Projection Matrix Rectified
        {
            index=0; //should be 12 at the end
            ROS_DEBUG_STREAM("P_rect_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
//                std::cout << *token_iterator << '\n';
                P[index++]=boost::lexical_cast<double>(*token_iterator);
            }
        }

    }

    return true;
}

int main(int argc, char **argv)
{
    kitti_player_options options;
    po::variables_map vm;

    po::options_description desc("Kitti_player, a player for KITTI raw datasets\nDatasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php\n\nAllowed options:",150);
    desc.add_options()
        ("help,h"                                                                                   , "help message")
        ("directory,d", po::value<string>(&options.path)->required()                                , "path to the kitti dataset Directory")
        ("frequency,f", po::value<float>(&options.frequency)->default_value(1.0)                    , "set replay Frequency")
        ("all      ,a", po::value<bool> (&options.all_data) ->implicit_value(1) ->default_value(0)  , "replay All data")
        ("velodyne ,v", po::value<bool> (&options.velodyne) ->implicit_value(1) ->default_value(0)  , "replay Velodyne data")
        ("gps      ,g", po::value<bool> (&options.gps)      ->implicit_value(1) ->default_value(0)  , "replay Gps data")
        ("imu      ,i", po::value<bool> (&options.imu)      ->implicit_value(1) ->default_value(0)  , "replay Imu data")
        ("grayscale,G", po::value<bool> (&options.grayscale)->implicit_value(1) ->default_value(0)  , "replay Stereo Grayscale images")
        ("color    ,C", po::value<bool> (&options.color)    ->implicit_value(1) ->default_value(0)  , "replay Stereo Color images")
        ("viewer     ", po::value<bool> (&options.viewer)   ->implicit_value(1) ->default_value(0)  , "enable image viewer")
    ;

    try
    {
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        po::notify(vm);

        vector<string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);

        if (to_pass_further.size()>0)
        {
            ROS_WARN_STREAM("Unknown Options Detected, shutting down node\n");
            cerr << desc << endl;
            return -1;

        }
    }
    catch(...)
    {
        cerr << desc << endl;

        cout << "kitti_player needs a directory tree like the following:" << endl;
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

        return -1;
    }

    ros::init(argc, argv, "kitti_player");
    ros::NodeHandle node;

    DIR *dir;
    struct dirent *ent;
    std::string frames_dir;
    std::vector<std::string> dataset_entries;
    unsigned long total_entries = 0;        //number of elements to be played
    unsigned long entries_played  = 0;      //number of elements played until now
    unsigned int len = 0;                   //counting elements support variable
    string dir_root             ;
    string dir_image00          ;string full_filename_image00;
    string dir_image01          ;string full_filename_image01;
    string dir_image02          ;string full_filename_image02;
    string dir_image03          ;string full_filename_image03;
    string dir_oxts             ;
    string dir_velodyne_points  ;
    cv::Mat cv_image00;
    cv::Mat cv_image01;
    cv::Mat cv_image02;
    cv::Mat cv_image03;
    std::string encoding;
    ros::Rate loop_rate(options.frequency);

    image_transport::ImageTransport it(node);
    image_transport::CameraPublisher pub00 = it.advertiseCamera("image00", 1);
    image_transport::CameraPublisher pub01 = it.advertiseCamera("image01", 1);
    image_transport::CameraPublisher pub02 = it.advertiseCamera("image02", 1);
    image_transport::CameraPublisher pub03 = it.advertiseCamera("image03", 1);

    sensor_msgs::Image ros_msg00;
    sensor_msgs::Image ros_msg01;
    sensor_msgs::Image ros_msg02;
    sensor_msgs::Image ros_msg03;

//    sensor_msgs::CameraInfo ros_cameraInfoMsg;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera00;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera01;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera02;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera03;

    cv_bridge::CvImage cv_bridge_img;


    if (vm.count("help")) {
        cout << desc << endl;

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
        cout << "        └── data              " << endl << endl; //todo add calib_cam_to_cam.txt

        return 1;
    }

    if (!(options.all_data || options.color || options.gps || options.grayscale || options.imu || options.velodyne))
    {
        ROS_WARN_STREAM("Job finished without playing the dataset. No 'publishing' parameters provided");
        return 0;
    }

    dir_root             = options.path;
    dir_image00          = options.path;
    dir_image01          = options.path;
    dir_image02          = options.path;
    dir_image03          = options.path;
    dir_oxts             = options.path;
    dir_velodyne_points  = options.path;
    (*(options.path.end()-1) != '/' ? dir_root            = options.path+"/"                :       dir_root=options.path);
    (*(options.path.end()-1) != '/' ? dir_image00         = options.path+"/image_00/data/"  :       dir_image00=options.path+"image_00/data/");
    (*(options.path.end()-1) != '/' ? dir_image01         = options.path+"/image_01/data/"  :       dir_image01=options.path+"image_01/data/");
    (*(options.path.end()-1) != '/' ? dir_image02         = options.path+"/image_02/data/"  :       dir_image02=options.path+"image_02/data/");
    (*(options.path.end()-1) != '/' ? dir_image03         = options.path+"/image_03/data/"  :       dir_image03=options.path+"image_03/data/");
    (*(options.path.end()-1) != '/' ? dir_oxts            = options.path+"/oxts/data/"      :       dir_oxts=options.path+"oxts/data/");
    (*(options.path.end()-1) != '/' ? dir_velodyne_points = options.path+"/velodyne_points/data/" : dir_velodyne_points=options.path+"velodyne_points/data/");

    // Check all the directories
    if (
            (options.all_data   && (   (opendir(dir_image00.c_str())            == NULL) ||
                                       (opendir(dir_image01.c_str())            == NULL) ||
                                       (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_image03.c_str())            == NULL) ||
                                       (opendir(dir_oxts.c_str())               == NULL) ||
                                       (opendir(dir_velodyne_points.c_str())    == NULL)))
            ||
            (options.color      && (   (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_image03.c_str())            == NULL)))
            ||
            (options.grayscale  && (   (opendir(dir_image00.c_str())            == NULL) ||
                                       (opendir(dir_image01.c_str())            == NULL)))
            ||
            (options.imu        && (   (opendir(dir_oxts.c_str())               == NULL)))
            ||
            (options.gps        && (   (opendir(dir_oxts.c_str())               == NULL)))
            ||
            (options.velodyne   && (   (opendir(dir_velodyne_points.c_str())    == NULL)))

        )
    {
        ROS_ERROR("Incorrect tree directory , use --help for details");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM ("Checking directories...");
        ROS_INFO_STREAM (options.path << "\t[OK]");
    }

    // publish color images

    //count elements in the folder

    if (options.all_data)
    {
        dir = opendir(dir_image02.c_str());
        while(ent = readdir(dir))
        {
            //skip . & ..
            len = strlen (ent->d_name);
            //skip . & ..
            if (len>2)
                total_entries++;
        }
        closedir (dir);
    }
    else
    {
        bool done=false;
        if (!done && options.color)
        {
            total_entries=0;
            dir = opendir(dir_image02.c_str());
            while(ent = readdir(dir))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len>2)
                    total_entries++;
            }            closedir (dir);
            done=true;
        }
        if (!done && options.grayscale)
        {
            total_entries=0;
            dir = opendir(dir_image00.c_str());
            while(ent = readdir(dir))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len>2)
                    total_entries++;
            }
            closedir (dir);
            done=true;
        }
        if (!done && options.gps)
        {
            total_entries=0;
            dir = opendir(dir_oxts.c_str());
            while(ent = readdir(dir))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len>2)
                    total_entries++;
            }
            closedir (dir);
            done=true;
        }
        if (!done && options.imu)
        {
            total_entries=0;
            dir = opendir(dir_oxts.c_str());
            while(ent = readdir(dir))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len>2)
                    total_entries++;
            }
            closedir (dir);
            done=true;
        }
        if (!done && options.velodyne)
        {
            total_entries=0;
            dir = opendir(dir_oxts.c_str());            
            while(ent = readdir(dir))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len>2)
                    total_entries++;
            }
            closedir (dir);
            done=true;
        }
    }


    if(options.viewer)
    {
        ROS_INFO_STREAM("Opening CV viewer(s)");
        if(options.color || options.all_data)
        {
            ROS_DEBUG_STREAM("color||all " << options.color << " " << options.all_data);
            cv::namedWindow("CameraSimulator Color Viewer",CV_WINDOW_AUTOSIZE);
            full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % 0 ) + ".png";
            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        if(options.grayscale|| options.all_data)
        {
            ROS_DEBUG_STREAM("grayscale||all " << options.grayscale << " " << options.all_data);
            cv::namedWindow("CameraSimulator Greyscale Viewer",CV_WINDOW_AUTOSIZE);
            full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % 0 ) + ".png";
            cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        ROS_INFO_STREAM("Opening CV viewer(s)... OK");
    }

    // CAMERA INFO SECTION: read one for all

    ros_cameraInfoMsg_camera00.header.seq = 0;
    ros_cameraInfoMsg_camera00.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera00.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera00.height = 0;
    ros_cameraInfoMsg_camera00.width  = 0;
    ros_cameraInfoMsg_camera00.D.resize(5);

    ros_cameraInfoMsg_camera01.header.seq = 0;
    ros_cameraInfoMsg_camera01.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera01.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera01.height = 0;
    ros_cameraInfoMsg_camera01.width  = 0;
    ros_cameraInfoMsg_camera01.D.resize(5);

    ros_cameraInfoMsg_camera02.header.seq = 0;
    ros_cameraInfoMsg_camera02.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera02.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera02.height = 0;
    ros_cameraInfoMsg_camera02.width  = 0;
    ros_cameraInfoMsg_camera02.D.resize(5);

    ros_cameraInfoMsg_camera03.header.seq = 0;
    ros_cameraInfoMsg_camera03.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera03.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera03.height = 0;
    ros_cameraInfoMsg_camera03.width  = 0;
    ros_cameraInfoMsg_camera03.D.resize(5);

    if(options.color || options.all_data)
    {
        if(
           !getCalibration(dir_root,"02",ros_cameraInfoMsg_camera02.K.data(),ros_cameraInfoMsg_camera02.D,ros_cameraInfoMsg_camera02.R.data(),ros_cameraInfoMsg_camera02.P.data()) &&
           !getCalibration(dir_root,"03",ros_cameraInfoMsg_camera03.K.data(),ros_cameraInfoMsg_camera03.D,ros_cameraInfoMsg_camera03.R.data(),ros_cameraInfoMsg_camera03.P.data())
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA02/CAMERA03 calibration");
            node.shutdown();
        }
        //Assume same height/width for the camera pair
        full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % 0 ) + ".png";
        cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera03.height = ros_cameraInfoMsg_camera02.height = cv_image02.rows;
        ros_cameraInfoMsg_camera03.width  = ros_cameraInfoMsg_camera02.width  = cv_image02.cols;
    }

    if(options.grayscale || options.all_data)
    {
        if(
           !getCalibration(dir_root,"00",ros_cameraInfoMsg_camera00.K.data(),ros_cameraInfoMsg_camera00.D,ros_cameraInfoMsg_camera00.R.data(),ros_cameraInfoMsg_camera00.P.data()) &&
           !getCalibration(dir_root,"01",ros_cameraInfoMsg_camera01.K.data(),ros_cameraInfoMsg_camera01.D,ros_cameraInfoMsg_camera01.R.data(),ros_cameraInfoMsg_camera01.P.data())
          )
        {
            ROS_ERROR_STREAM("Error reading CAMERA00/CAMERA01 calibration");
            node.shutdown();
        }
        //Assume same height/width for the camera pair
        full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % 0 ) + ".png";
        cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
        cv::waitKey(5);
        ros_cameraInfoMsg_camera01.height = ros_cameraInfoMsg_camera00.height = cv_image00.rows;
        ros_cameraInfoMsg_camera01.width  = ros_cameraInfoMsg_camera00.width  = cv_image00.cols;
    }


    do
    {
        if(options.color || options.all_data)
        {
            full_filename_image02 = dir_image02 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            full_filename_image03 = dir_image03 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            ROS_DEBUG_STREAM ( full_filename_image02 << endl << full_filename_image03 << endl << endl);

            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
            cv_image03 = cv::imread(full_filename_image03, CV_LOAD_IMAGE_UNCHANGED);

            if ( (cv_image02.data == NULL) || (cv_image03.data == NULL) ){
                ROS_ERROR_STREAM("Error reading color images (02 & 03)");
                ROS_ERROR_STREAM(full_filename_image02 << endl << full_filename_image03);
                node.shutdown();
                return -1;
            }

            if(options.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Color Viewer",cv_image02);
                //give some time to draw images
                cv::waitKey(5);
            }

            cv_bridge_img.header.stamp = ros::Time::now();
            cv_bridge_img.encoding = sensor_msgs::image_encodings::BGR8; //sensor_msgs::image_encodings::MONO8
            cv_bridge_img.header.frame_id = ros::this_node::getName();
            cv_bridge_img.header.seq = entries_played;

            cv_bridge_img.image = cv_image02;
            cv_bridge_img.toImageMsg(ros_msg02);

            cv_bridge_img.image = cv_image03;
            cv_bridge_img.toImageMsg(ros_msg03);

            pub02.publish(ros_msg02,ros_cameraInfoMsg_camera02);
            pub03.publish(ros_msg03,ros_cameraInfoMsg_camera03);

        }

        if(options.grayscale || options.all_data)
        {
            full_filename_image00 = dir_image00 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            full_filename_image01 = dir_image01 + boost::str(boost::format("%010d") % entries_played ) + ".png";
            ROS_DEBUG_STREAM ( full_filename_image00 << endl << full_filename_image01 << endl << endl);

            cv_image00 = cv::imread(full_filename_image00, CV_LOAD_IMAGE_UNCHANGED);
            cv_image01 = cv::imread(full_filename_image01, CV_LOAD_IMAGE_UNCHANGED);

            if ( (cv_image00.data == NULL) || (cv_image01.data == NULL) ){
                ROS_ERROR_STREAM("Error reading color images (00 & 01)");
                ROS_ERROR_STREAM(full_filename_image00 << endl << full_filename_image01);
                node.shutdown();
                return -1;
            }

            if(options.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Greyscale Viewer",cv_image00);
                //give some time to draw images
                cv::waitKey(5);
            }

            cv_bridge_img.header.stamp = ros::Time::now();
            cv_bridge_img.encoding = sensor_msgs::image_encodings::MONO8;
            cv_bridge_img.header.frame_id = ros::this_node::getName();
            cv_bridge_img.header.seq = entries_played;

            cv_bridge_img.image = cv_image00;
            cv_bridge_img.toImageMsg(ros_msg00);

            cv_bridge_img.image = cv_image01;
            cv_bridge_img.toImageMsg(ros_msg01);

            pub00.publish(ros_msg00,ros_cameraInfoMsg_camera00);
            pub01.publish(ros_msg01,ros_cameraInfoMsg_camera01);

        }

//        if(options.grayscale || options.all_data)
//        {
//        publish_velodyne(map_pub, sequence_path);
//        }



        entries_played++;
        loop_rate.sleep();
    }while(entries_played<=total_entries-1 && ros::ok());







    // TODO
//    if(viewer)
//        cv::destroyWindow("CameraSimulator Viewer");


    ROS_INFO_STREAM("Done!");
    node.shutdown();

    return -1;














































    exitIfNotFound = false;

    Random random;
    RandGlobal::setRandomInstance(random);
    last_uncertain_pose.setIdentity();
    oldPose.setIdentity();

    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig> srv;
    dynamic_reconfigure::Server<kitti_player::kitti_playerConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    srv.setCallback(f);

    ros::Publisher map_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloud_in", 1, true);
    ros::Publisher initialpose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    path = argv[1];

    string::size_type pos = path.find_last_of( "\\/" );
    path = path.substr( 0, pos);

    node.param<string>("camera_ref_zero_frame",camera_ref_zero_frame,"/camera_ref_zero_frame");
    node.param<string>("odom_frame",odom_frame,"/odom"); //odom

    node.param<string>("gt_camera_ref_frame",gt_camera_ref_frame,"/gt_camera_ref");
    node.param<string>("gt_laser_frame",gt_laser_frame,"/laser_frame");
    node.param<string>("gt_robot_frame",gt_robot_frame,"/cart_frame");

    node.param<string>("camera_ref_frame",camera_ref_frame,"/BIASED_camera_ref");
    node.param<string>("laser_frame",laser_frame,"/BIASED_laser_frame");
    node.param<string>("robot_frame",robot_frame,"/BIASED_robot_frame");

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


    }


    return 0;
}
