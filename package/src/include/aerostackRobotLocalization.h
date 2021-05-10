#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <sstream>

// ROS
#include "ros/ros.h"

//Drone module
#include <Eigen/Dense>


//Main class -> Algorithm developed!


//Messages in
//ROS
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

//OpenCV if needed
//#include <opencv2/opencv.hpp>

// low pass filter
#include "control/LowPassFilter.h"
#include "xmlfilereader.h"
#include "control/filtered_derivative_wcb.h"

//Messages out
//Keypoints: Messages out
#include <droneMsgsROS/dronePitchRollCmd.h>
#include <droneMsgsROS/droneDAltitudeCmd.h>
#include <droneMsgsROS/droneDYawCmd.h>
#include <droneMsgsROS/droneAltitude.h>
#include <droneMsgsROS/vector2Stamped.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/droneSpeeds.h>

//ros time synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//bebop msgs
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"

//Ground Robots: Messages out
//#include <droneMsgsROS/targetInImage.h>
//#include <droneMsgsROS/vectorTargetsInImageStamped.h>
#include "xmlfilereader.h"
#include <gazebo_msgs/ModelStates.h>

//#define slamdunk_has_angle
const float slamdunk_angle = 17*(M_PI/180);


/////////////////////////////////////////
// Class DroneKeypointsGridDetectorROSModule
//
//   Description
//
/////////////////////////////////////////
class AerostackRobotLocalizationModule : public DroneModule
{	
protected:



    //Config files path names
protected:
    //std::string camCalibParamFile;
    //std::string gridIntersectionsParamFile;


    //Variables
public:
    sensor_msgs::Imu Imu_data;
    nav_msgs::Odometry controllerOdometryData;
    droneMsgsROS::dronePitchRollCmd PitchRollCmd;
    droneMsgsROS::droneDAltitudeCmd DaltitudeCmd;
    droneMsgsROS::droneDYawCmd      DyawCmd;
    nav_msgs::Odometry altitudeData;
    nav_msgs::Odometry altitudeGTData;
    nav_msgs::Odometry groundSpeedsData;
    //nav_msgs::Odometry rotationAngleData;
    sensor_msgs::Imu rotationAngleData;
    nav_msgs::Odometry PnPPoseData;
    nav_msgs::Odometry HectorSlamPoseData;
    nav_msgs::Odometry SlamDunkPoseData;
    nav_msgs::Odometry VOSemanticSLAMPoseData;
    nav_msgs::Odometry LaserScanPoseData;
    nav_msgs::Odometry LaserScanWindowPoseData;
    nav_msgs::Odometry bebop_vel_alti_data;
    nav_msgs::Odometry bebop_odom_data;
    sensor_msgs::Imu bebop_imu_data;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PoseWithCovarianceStamped ResetPose;
    tf::TransformBroadcaster broadcastTransform;
    droneMsgsROS::vector2Stamped speeds;
    droneMsgsROS::dronePose EstimatedPose;
    droneMsgsROS::droneSpeeds EstimatedSpeeds;
    bool receivedPitchRoll, receivedDaltitude, receivedDyaw;
    tf::TransformListener listener;
    geometry_msgs::Vector3Stamped vin, vout;
    sensor_msgs::NavSatFix gps_data;
    tf::StampedTransform hector_transform;
    double yaw_ground_truth;
    bool bebop_first_yaw_measurement_;
    double bebop_first_yaw_, bebop_first_pitch_, bebop_first_roll_;


public:
    std::string configFile;
    double init_position_x,init_position_y, init_position_z;
    double init_pitch, init_roll, init_yaw;
    double optical_flow_co_x, optical_flow_co_y;
    double altitude_co_z, altitude_co_dz;
    double pitch_cmd_co, roll_cmd_co, dyaw_cmd_co, daltitude_cmd_co;
    double pitch_rad, roll_rad, yaw_rad;

protected:
    CVG_BlockDiagram::FilteredDerivativeWCB filtered_derivative_wcb_x, filtered_derivative_wcb_y, filtered_derivative_wcb_z;
    CVG_BlockDiagram::FilteredDerivativeWCB filtered_derivative_wcb_lx, filtered_derivative_wcb_ly;
    CVG_BlockDiagram::FilteredDerivativeWCB filtered_derivative_wcb_lwx, filtered_derivative_wcb_lwy;
    CVG_BlockDiagram::FilteredDerivativeWCB filtered_derivative_vos_x, filtered_derivative_vos_y, filtered_derivative_vos_z;
    //subscribers
protected:

    //Subscriber
    ros::Subscriber droneImuSub;
    ros::Subscriber droneRotationAnglesSub;
    ros::Subscriber droneCommandPitchRollSub;
    ros::Subscriber droneCommandDaltitudeSub;
    ros::Subscriber droneCommandDyawSub;
    ros::Subscriber droneAltitudeSub;
    ros::Subscriber droneSpeedsSub;
    ros::Subscriber droneOdomFilteredSub;
    ros::Subscriber dronePnPPoseSub;
    ros::Subscriber droneGPSDataSub;
    ros::Subscriber droneHectorSlamdPoseSub;
    ros::Subscriber mavrosLocalSpeedsSubsriber;
    ros::Subscriber droneLaserScanPoseSub;
    ros::Subscriber droneLaserScanWindowPoseSub;
    ros::Subscriber droneSlamDunkPoseSub;
    ros::Subscriber droneVOSematicSLAMSub;
   
public:
    //Callback function
    void droneImuCallback(const sensor_msgs::Imu& msg);
    void droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg);
    void droneCommandPitchRollCallback(const droneMsgsROS::dronePitchRollCmd& msg);
    void droneCommandDaltitudeCallback(const droneMsgsROS::droneDAltitudeCmd& msg);
    void droneCommandDyawCallback(const droneMsgsROS::droneDYawCmd& msg);
    void droneAltitudeCallback(const droneMsgsROS::droneAltitude& msg);
    void droneSpeedsCallback(const droneMsgsROS::vector2Stamped& msg);
    void droneOdometryFilteredCallback(const nav_msgs::Odometry& msg);
    void dronePnPPoseCallback(const geometry_msgs::Pose& msg);
    void droneGPSDataCallback(const sensor_msgs::NavSatFix& msg);
    void droneHectorSlamPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void localSpeedsCallbackGazebo(const gazebo_msgs::ModelStatesConstPtr &msg);
    void droneLaserScanPoseCallback(const nav_msgs::Odometry& msg);
    void droneLaserScanWindowPoseCallback(const nav_msgs::Odometry& msg);
    void droneSlamDunkPoseCallback(const geometry_msgs::PoseStamped& msg);
    void droneVOSemanticSLAMPoseCallback(const geometry_msgs::PoseStamped& msg);

    //publishers
protected:
    //Publisher
    ros::Publisher droneImuPub;
    ros::Publisher droneRotationAnglesPub;
    ros::Publisher droneOdomPub;
    ros::Publisher droneAltitudePub;
    ros::Publisher droneAltitudeGTPub;
    ros::Publisher droneGroundSpeedsPub;
    ros::Publisher droneSetModePub;
    ros::Publisher droneEstimatedPosePub;
    ros::Publisher droneEstimatedSpeedsPub;
    ros::Publisher dronePnPPosePub;
    ros::Publisher droneGPSdataPub;
    ros::Publisher droneHectorSlamDataPub;
    ros::Publisher droneLaserScanPosePub;
    ros::Publisher droneLaserScanWindowPosePub;
    ros::Publisher droneSlamDunkPosePub;
    ros::Publisher droneVOSematicSLAMPub;
    ros::Publisher droneBebopIMUPub;
    ros::Publisher dronebebopVelandAltiPub;
    ros::Publisher droneBebopOdomDataPub;
    ros::Publisher self_localization_pose_pub;
    ros::Publisher self_localization_twist_pub;


    //Publisher functions
public:
    void PublishOdomData();
    void PublishRotationAnglesData();
    void PublishAltitudeData();
    void PublishAltitudeGTData();
    void PublishSpeedsData();
    void PublishImuData();
    void PublishEstimatedData();
    void PublishPnPData();
    void PublishGPSData();
    void PublishHectorSlamData();
    void PublishLaserScanPoseData();
    void PublishLaserScanWindowPoseData();
    void PublishSlamdunkData();
    void PublishVOSemanticSLAMPoseData();
    void PublishBebopIMUData();
    void PublishBebopOdomData();

public:
    AerostackRobotLocalizationModule();
    ~AerostackRobotLocalizationModule();

public:
    void open(ros::NodeHandle & nIn);
    void close();

public:
    bool init(std::string configFile);
    bool readConfigs(std::string configFile);
    void readParameters();

    //Reset
protected:
    bool resetValues();

    //Start
protected:
    bool startVal();

    //Stop
protected:
    bool stopVal();

    //Run
public:
    bool run();


};
