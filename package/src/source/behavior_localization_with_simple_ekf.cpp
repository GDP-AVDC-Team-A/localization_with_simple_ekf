/*!********************************************************************************
 * \brief     localization_with_simple_ekf implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "../include/behavior_localization_with_simple_ekf.h"
#include <pluginlib/class_list_macros.h>

namespace localization_with_simple_ekf{
BehaviorLocalizationWithSimpleEkf::BehaviorLocalizationWithSimpleEkf() : BehaviorExecutionController() {
  setName("localization_with_simple_ekf");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorLocalizationWithSimpleEkf::~BehaviorLocalizationWithSimpleEkf() {}

void BehaviorLocalizationWithSimpleEkf::onConfigure(){
  //Init
  n = getNodeHandle();
  //Node
    //read the parameters from the launch file
    readParameters();
    //Init
    if(!iniciar(configFile))
    {
        std::cout<<"Error init"<<std::endl;
        return;
    }
    return;
}

bool BehaviorLocalizationWithSimpleEkf::checkSituation(){
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  rsp.situation_occurs = true;
  return rsp.situation_occurs;
}

void BehaviorLocalizationWithSimpleEkf::checkGoal(){
}

void BehaviorLocalizationWithSimpleEkf::onExecute(){
}

void BehaviorLocalizationWithSimpleEkf::checkProgress() {
}

void BehaviorLocalizationWithSimpleEkf::onActivate(){
    //// TOPICS
    //Subscribers
    droneImuSub                 = n.subscribe("imu", 1, &BehaviorLocalizationWithSimpleEkf::droneImuCallback, this);
    droneCommandPitchRollSub    = n.subscribe("command/pitch_roll",1,&BehaviorLocalizationWithSimpleEkf::droneCommandPitchRollCallback, this);
    droneCommandDaltitudeSub    = n.subscribe("command/dAltitude",1,&BehaviorLocalizationWithSimpleEkf::droneCommandDaltitudeCallback, this);
    droneCommandDyawSub         = n.subscribe("command/dYaw",1, &BehaviorLocalizationWithSimpleEkf::droneCommandDyawCallback, this);
    droneAltitudeSub            = n.subscribe("altitude",1,&BehaviorLocalizationWithSimpleEkf::droneAltitudeCallback, this);
    droneSpeedsSub              = n.subscribe("ground_speed",1,&BehaviorLocalizationWithSimpleEkf::droneSpeedsCallback, this);
    droneOdomFilteredSub        = n.subscribe("odometry/filtered",1,&BehaviorLocalizationWithSimpleEkf::droneOdometryFilteredCallback, this);
    droneRotationAnglesSub      = n.subscribe("rotation_angles",1,&BehaviorLocalizationWithSimpleEkf::droneRotationAnglesCallback, this);
    dronePnPPoseSub             = n.subscribe("vb_estimated_pose/rpnp_pose",1,&BehaviorLocalizationWithSimpleEkf::dronePnPPoseCallback, this);
    droneGPSDataSub             = n.subscribe("mavros/global_position/raw/fix",1,&BehaviorLocalizationWithSimpleEkf::droneGPSDataCallback, this);
    droneHectorSlamdPoseSub     = n.subscribe("poseupdate",1,&BehaviorLocalizationWithSimpleEkf::droneHectorSlamPoseCallback, this);
    mavrosLocalSpeedsSubsriber  = n.subscribe("/gazebo/model_states",1, &BehaviorLocalizationWithSimpleEkf::localSpeedsCallbackGazebo, this);
    droneLaserScanPoseSub       = n.subscribe("lb_estimated_pose/door_pose", 1, &BehaviorLocalizationWithSimpleEkf::droneLaserScanPoseCallback, this);
    droneLaserScanWindowPoseSub = n.subscribe("lb_estimated_pose/window_pose", 1, &BehaviorLocalizationWithSimpleEkf::droneLaserScanWindowPoseCallback, this);
    droneSlamDunkPoseSub        = n.subscribe("/pose", 1, &BehaviorLocalizationWithSimpleEkf::droneSlamDunkPoseCallback, this);
    droneVOSematicSLAMSub       = n.subscribe("/final_pose",1, &BehaviorLocalizationWithSimpleEkf::droneVOSemanticSLAMPoseCallback, this);



    //Publishers
    droneImuPub             = n.advertise<sensor_msgs::Imu>("imu/data", 1, true);
    droneRotationAnglesPub  = n.advertise<sensor_msgs::Imu>("rotation_angles/data",1,true);
    droneOdomPub            = n.advertise<nav_msgs::Odometry>("controller/odom/data",1,true);
    droneAltitudePub        = n.advertise<nav_msgs::Odometry>("altitude/data",1,true);
    droneAltitudeGTPub        = n.advertise<nav_msgs::Odometry>("altitude_gt/data",1,true);
    droneGroundSpeedsPub    = n.advertise<nav_msgs::Odometry>("optical_flow/data",1,true);
    dronePnPPosePub         = n.advertise<nav_msgs::Odometry>("PnP_pose/data",1, true);
    droneGPSdataPub         = n.advertise<sensor_msgs::NavSatFix>("gps/fix_data",1, true);
    droneHectorSlamDataPub  = n.advertise<nav_msgs::Odometry>("hector_slam/data",1,true);
    droneLaserScanPosePub   = n.advertise<nav_msgs::Odometry>("laser_scan_pose/data",1,true);
    droneLaserScanWindowPosePub   = n.advertise<nav_msgs::Odometry>("laser_scan_window_pose/data",1,true);
    droneSetModePub         = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose",1,true);
    droneEstimatedPosePub   = n.advertise<droneMsgsROS::dronePose>("EstimatedPose_droneGMR_wrt_GFF",1,true);
    droneEstimatedSpeedsPub = n.advertise<droneMsgsROS::droneSpeeds>("EstimatedSpeed_droneGMR_wrt_GFF",1,true);
    droneSlamDunkPosePub    = n.advertise<nav_msgs::Odometry>("slamdunk_pose/data", 1, true);
    droneVOSematicSLAMPub   = n.advertise<nav_msgs::Odometry>("VO_Semantic_SLAM/Pose/data",1, true);
    droneBebopIMUPub        = n.advertise<sensor_msgs::Imu>("bebop_imu/data", 1, true);
    dronebebopVelandAltiPub = n.advertise<nav_msgs::Odometry>("bebop_vel_alti/data",1, true);
    droneBebopOdomDataPub   = n.advertise<nav_msgs::Odometry>("bebop_odom/data",1,true);

    self_localization_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/drone4/self_localization/pose",1,true);
    self_localization_twist_pub = n.advertise<geometry_msgs::TwistStamped>("/drone4/self_localization/speed",1,true);

    //autostart module!
    moduleStarted=true;

    //End
  //ros::spin();
}

void BehaviorLocalizationWithSimpleEkf::onDeactivate(){
    droneImuSub.shutdown();
    droneCommandPitchRollSub.shutdown();
    droneCommandDaltitudeSub.shutdown();
    droneCommandDyawSub.shutdown();
    droneAltitudeSub.shutdown();
    droneSpeedsSub.shutdown();
    droneOdomFilteredSub.shutdown();
    droneRotationAnglesSub.shutdown();
    dronePnPPoseSub.shutdown();
    droneGPSDataSub.shutdown();
    droneHectorSlamdPoseSub.shutdown();
    mavrosLocalSpeedsSubsriber.shutdown();
    droneLaserScanPoseSub.shutdown();
    droneLaserScanWindowPoseSub.shutdown();
    droneSlamDunkPoseSub.shutdown();
    droneVOSematicSLAMSub.shutdown();
    droneImuPub.shutdown();
    droneRotationAnglesPub.shutdown();
    droneOdomPub.shutdown();
    droneAltitudePub.shutdown();
    droneAltitudeGTPub.shutdown();
    droneGroundSpeedsPub.shutdown();
    dronePnPPosePub.shutdown();
    droneGPSdataPub.shutdown();
    droneHectorSlamDataPub.shutdown();
    droneLaserScanPosePub.shutdown();
    droneLaserScanWindowPosePub.shutdown();
    droneSetModePub.shutdown();
    droneEstimatedPosePub.shutdown();
    droneEstimatedSpeedsPub.shutdown();
    droneSlamDunkPosePub.shutdown();
    droneVOSematicSLAMPub.shutdown();
    droneBebopIMUPub.shutdown();
    dronebebopVelandAltiPub.shutdown();
    droneBebopOdomDataPub.shutdown();
    self_localization_pose_pub.shutdown();
    self_localization_twist_pub.shutdown();
}

void BehaviorLocalizationWithSimpleEkf::checkProcesses() { 
 
}

void BehaviorLocalizationWithSimpleEkf::readParameters()
{
    //Config file
    ros::param::get("~config_file", configFile);
    if ( configFile.length() == 0)
    {
        configFile="robot_localization.xml";
    }
}

bool BehaviorLocalizationWithSimpleEkf::resetValues()
{
    //Reseting the ekf

    ResetPose.header.stamp = ros::Time::now();
    ResetPose.header.frame_id = "speeds_odom";
    ResetPose.pose.pose.position.x    = init_position_x;
    ResetPose.pose.pose.position.y    = init_position_y;
    ResetPose.pose.pose.position.z    = init_position_z;

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(init_roll,init_pitch,init_yaw);

    ResetPose.pose.pose.orientation.x = quaternion.getX();
    ResetPose.pose.pose.orientation.y = quaternion.getY();
    ResetPose.pose.pose.orientation.z = quaternion.getZ();
    ResetPose.pose.pose.orientation.w = quaternion.getW();

    for (size_t ind = 0; ind < 36; ind+=7)
    {
        ResetPose.pose.covariance[ind] = 1e-6;
    }

    droneSetModePub.publish(ResetPose);
    return true;
}


bool BehaviorLocalizationWithSimpleEkf::startVal()
{
    //Reseting the ekf

    ResetPose.header.stamp = ros::Time::now();
    ResetPose.header.frame_id = "speeds_odom";
    ResetPose.pose.pose.position.x    = init_position_x;
    ResetPose.pose.pose.position.y    = init_position_y;
    ResetPose.pose.pose.position.z    = init_position_z;

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(init_roll,init_pitch,init_yaw);

    ResetPose.pose.pose.orientation.x = quaternion.getX();
    ResetPose.pose.pose.orientation.y = quaternion.getY();
    ResetPose.pose.pose.orientation.z = quaternion.getZ();
    ResetPose.pose.pose.orientation.w = quaternion.getW();

    for (size_t ind = 0; ind < 36; ind+=7)
    {
        ResetPose.pose.covariance[ind] = 1e-6;
    }

    droneSetModePub.publish(ResetPose);
    return true;
    //End
}

void BehaviorLocalizationWithSimpleEkf::droneImuCallback(const sensor_msgs::Imu &msg)
{
    //Publishing IMU data to the EKF
    Imu_data.header.stamp = msg.header.stamp;
    Imu_data.header.frame_id = "fcu";

    Imu_data.orientation            = msg.orientation;
    Imu_data.orientation_covariance = msg.orientation_covariance;
    Imu_data.orientation_covariance[0] = 0.01;
    Imu_data.orientation_covariance[1] = 0.0;
    Imu_data.orientation_covariance[2] = 0.0;
    Imu_data.orientation_covariance[3] = 0.0;
    Imu_data.orientation_covariance[4] = 0.01;
    Imu_data.orientation_covariance[5] = 0.0;
    Imu_data.orientation_covariance[6] = 0.0;
    Imu_data.orientation_covariance[7] = 0.0;
    Imu_data.orientation_covariance[8]=  0.01;
    
    //converting from NED frame to ENU frame for the robot_localization
    Imu_data.angular_velocity.x = +1*msg.angular_velocity.x;
    Imu_data.angular_velocity.y = -1*msg.angular_velocity.y;
    Imu_data.angular_velocity.z = -1*msg.angular_velocity.z;
    Imu_data.angular_velocity_covariance = msg.angular_velocity_covariance;
    
    //converting from NED frame to ENU frame for the robot_localization
    Imu_data.linear_acceleration.x = +1*msg.linear_acceleration.x;
    Imu_data.linear_acceleration.y = -1*msg.linear_acceleration.y;
    Imu_data.linear_acceleration.z = -1*msg.linear_acceleration.z;

    PublishImuData();
    return;
}

//This data is Published only if the drone does not publish IMU data
void BehaviorLocalizationWithSimpleEkf::droneRotationAnglesCallback(const geometry_msgs::Vector3Stamped &msg)
{
    rotationAngleData.header.stamp    = ros::Time::now();
    rotationAngleData.header.frame_id = "fcu";
    //rotationAngleData.child_frame_id  = "fcu";

    //convert from degrees to radians
    roll_rad      =  (msg.vector.x) * (M_PI/180);
    pitch_rad     = -(msg.vector.y) * (M_PI/180);
    yaw_rad       = -(msg.vector.z) * (M_PI/180);

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll_rad, pitch_rad, yaw_rad);

    rotationAngleData.orientation.x = quaternion.getX();
    rotationAngleData.orientation.y = quaternion.getY();
    rotationAngleData.orientation.z = quaternion.getZ();
    rotationAngleData.orientation.w = quaternion.getW();

    rotationAngleData.orientation_covariance[0] = 0.01;
    rotationAngleData.orientation_covariance[4] = 0.01;
    rotationAngleData.orientation_covariance[8] = 0.01;


    //    rotationAngleData.pose.pose.orientation.x = quaternion.getX();
    //    rotationAngleData.pose.pose.orientation.y = quaternion.getY();
    //    rotationAngleData.pose.pose.orientation.z = quaternion.getZ();
    //    rotationAngleData.pose.pose.orientation.w = quaternion.getW();
    //    rotationAngleData.pose.covariance[21] = 0.33;
    //    rotationAngleData.pose.covariance[28] = 0.33;
    //    rotationAngleData.pose.covariance[35] = 0.33;

    PublishRotationAnglesData();
}

void BehaviorLocalizationWithSimpleEkf::droneCommandPitchRollCallback(const droneMsgsROS::dronePitchRollCmd &msg)
{
    PitchRollCmd.pitchCmd = msg.pitchCmd;
    PitchRollCmd.rollCmd  = msg.rollCmd;
    receivedPitchRoll = true;
    return;
}

void BehaviorLocalizationWithSimpleEkf::droneCommandDaltitudeCallback(const droneMsgsROS::droneDAltitudeCmd &msg)
{
    DaltitudeCmd.dAltitudeCmd = msg.dAltitudeCmd;
    receivedDaltitude = true;
    return;
}

void BehaviorLocalizationWithSimpleEkf::droneCommandDyawCallback(const droneMsgsROS::droneDYawCmd &msg)
{
    DyawCmd.dYawCmd = msg.dYawCmd;
    receivedDyaw = true;

    PublishOdomData();
    return;
}

void BehaviorLocalizationWithSimpleEkf::droneAltitudeCallback(const droneMsgsROS::droneAltitude &msg)
{
    //Publishing Alitude data to the EKF
    altitudeData.header.stamp    = msg.header.stamp;
    altitudeData.header.frame_id = "speeds_odom";
    altitudeData.child_frame_id  = "fcu";

    altitudeData.pose.pose.position.z = -msg.altitude;
    altitudeData.twist.twist.linear.z = -msg.altitude_speed;
    altitudeData.pose.covariance[14]  = altitude_co_z;
    altitudeData.twist.covariance[14] = altitude_co_dz;


    PublishAltitudeData();
    return;
}

void BehaviorLocalizationWithSimpleEkf::droneSpeedsCallback(const droneMsgsROS::vector2Stamped &msg)
{
    //Publishing optical flow data to the EKF
    groundSpeedsData.header.stamp = ros::Time::now();
    groundSpeedsData.header.frame_id = "speeds_odom";
    groundSpeedsData.child_frame_id  = "fcu";

    groundSpeedsData.twist.twist.linear.x  =  msg.vector.x;
    groundSpeedsData.twist.twist.linear.y  = -msg.vector.y;
    groundSpeedsData.twist.covariance[0]   = optical_flow_co_x;
    groundSpeedsData.twist.covariance[7]   = optical_flow_co_y;

    PublishSpeedsData();
    return;
}

void BehaviorLocalizationWithSimpleEkf::dronePnPPoseCallback(const geometry_msgs::Pose &msg)
{
    PnPPoseData.header.stamp = ros::Time::now();
    PnPPoseData.child_frame_id  = "fcu";
    PnPPoseData.header.frame_id = "speeds_odom";

    PnPPoseData.pose.pose.position.x = msg.position.x;
    PnPPoseData.pose.pose.position.y = msg.position.y;
    PnPPoseData.pose.pose.position.z = msg.position.z;

    //std::cout << EstimatedPose.x << std::endl;
    if(EstimatedPose.x >= 6.0 && EstimatedPose.x <= 10.0){
        PnPPoseData.pose.covariance[0]  = 1.0;
        PnPPoseData.pose.covariance[7]  = 1.0;
        PnPPoseData.pose.covariance[14] = 1.0;
    }
    else{
        PnPPoseData.pose.covariance[0]  = 10000.0;
        PnPPoseData.pose.covariance[7]  = 10000.0;
        PnPPoseData.pose.covariance[14] = 10000.0;
    }

    PublishPnPData();

}

void BehaviorLocalizationWithSimpleEkf::droneGPSDataCallback(const sensor_msgs::NavSatFix &msg)
{
    gps_data.header.stamp             = msg.header.stamp;
    gps_data.header.frame_id          = "gps";
    gps_data.latitude                 = msg.latitude;
    gps_data.longitude                = msg.longitude;
    gps_data.altitude                 = msg.altitude;
    gps_data.position_covariance[0]   = 100;
    gps_data.position_covariance[4]   = 100;
    gps_data.position_covariance[8]   = 100;
    gps_data.position_covariance_type = 1;

    PublishGPSData();
}


void BehaviorLocalizationWithSimpleEkf::localSpeedsCallbackGazebo(const gazebo_msgs::ModelStatesConstPtr &msg){

    /* Calculating Roll, Pitch, Yaw */
    tf::Quaternion q(msg->pose[2].orientation.x, msg->pose[2].orientation.y, msg->pose[2].orientation.z, msg->pose[2].orientation.w);
    tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double y, p, r;
    m.getEulerYPR(y, p, r);

    yaw_ground_truth = y;

    // Publish ground truth altitude data
    altitudeGTData.header.stamp    = ros::Time::now();
    altitudeGTData.header.frame_id = "speeds_odom";
    altitudeGTData.child_frame_id  = "fcu";

    altitudeGTData.pose.pose.position.z = msg->pose[2].position.z;
    altitudeGTData.twist.twist.linear.z = msg->twist[2].linear.z;
    altitudeGTData.pose.covariance[14]  = altitude_co_z;
    altitudeGTData.twist.covariance[14] = altitude_co_dz;

    PublishAltitudeGTData();

}

void BehaviorLocalizationWithSimpleEkf::droneHectorSlamPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    HectorSlamPoseData.child_frame_id  = "fcu";
    HectorSlamPoseData.header.frame_id = "speeds_odom";
    HectorSlamPoseData.header.stamp    = msg.header.stamp;

    //     geometry_msgs::PoseStamped PoseIn;
    //     geometry_msgs::PoseStamped PoseOut;

    //     PoseIn.header.stamp    = msg.header.stamp;
    //     PoseIn.header.frame_id = "/map";
    //     PoseIn.pose            = msg.pose;

    //     try{
    //     listener.transformPose("/odom",ros::Time(0) ,PoseIn, "/map",PoseOut);
    //     std::cout << "PoseIn" <<  PoseIn << std::endl;
    //     std::cout << "PoseOut" << PoseOut << std::endl;
    //     }
    //     catch (tf::TransformException ex)
    //     {
    //         ROS_ERROR("%s",ex.what());
    //         ros::Duration(1.0).sleep();
    //     }

    //     geometry_msgs::PoseStamped PoseDiff;
    //     PoseDiff.pose.position.x = PoseIn.pose.position.x - PoseOut.pose.position.x;
    //     PoseDiff.pose.position.y = PoseIn.pose.position.y - PoseOut.pose.position.y;
    //     std::cout << "PoseDiff " << PoseDiff << std::endl;


    ros::Time current_timestamp = ros::Time::now();

    double x_raw_t = msg.pose.pose.position.x;
    double y_raw_t = msg.pose.pose.position.y;


    time_t tv_sec; suseconds_t tv_usec;
    {
        tv_sec  = current_timestamp.sec;
        tv_usec = current_timestamp.nsec / 1000.0;
        filtered_derivative_wcb_x.setInput( x_raw_t, tv_sec, tv_usec);
        filtered_derivative_wcb_y.setInput( y_raw_t, tv_sec, tv_usec);
    }

    double x_t, dx_t;
    double y_t, dy_t;
    filtered_derivative_wcb_x.getOutput( x_t,  dx_t);
    filtered_derivative_wcb_y.getOutput( y_t,  dy_t);


    //     std::cout << "x_raw_t" << x_raw_t << std::endl;
    //     std::cout << "x_t" << x_t << std::endl;
    //     std::cout << "dx_t " << dx_t << std::endl;
    //     std::cout << "dy_t " << dy_t << std::endl;

    HectorSlamPoseData.pose.pose.position.x = msg.pose.pose.position.x ;  /*PoseIn.pose.position.x */;
    HectorSlamPoseData.pose.pose.position.y = msg.pose.pose.position.y ;  /*PoseIn.pose.position.y*/;
    HectorSlamPoseData.pose.pose.orientation = msg.pose.pose.orientation;

    // Converting to Body
    /* Calculating Roll, Pitch, Yaw */
    //     tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    //     tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double y, p, r;
    //     m.getEulerYPR(y, p, r);
    y = EstimatedPose.yaw;
    p = EstimatedPose.pitch;
    r = EstimatedPose.roll;

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    GlobalFrame(0) = (+1)*dx_t;
    GlobalFrame(1) = (+1)*dy_t;
    GlobalFrame(2) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    BodyFrame = RotationMat*GlobalFrame;


    HectorSlamPoseData.twist.twist.linear.x  = (+1) * BodyFrame(0);
    HectorSlamPoseData.twist.twist.linear.y  = (+1) * BodyFrame(1);

    //     HectorSlamPoseData.twist.twist.linear.x  = dx_t;
    //     HectorSlamPoseData.twist.twist.linear.y  = dy_t;


    //     geometry_msgs::PoseStamped PoseDiff;
    //     PoseDiff.pose.position.x = PoseIn.pose.position.x - PoseOut.pose.position.x;
    //     PoseDiff.pose.position.y = PoseIn.pose.position.y - PoseOut.pose.position.y;
    //     std::cout << "PoseDiff " << PoseDiff << std::endl;

    //      if(HectorSlamPoseData.pose.pose.position.x  > -20 && HectorSlamPoseData.pose.pose.position.x < 20 &&
    //        HectorSlamPoseData.pose.pose.position.y  > -20 && HectorSlamPoseData.pose.pose.position.y < 20)
    //     {
    //HectorSlamPoseData.pose.covariance[0]   = 0.0099;
    //HectorSlamPoseData.pose.covariance[7]   = 0.0099;
    //HectorSlamPoseData.pose.covariance[28]  = 0.0099;
    //HectorSlamPoseData.pose.covariance[35]  = 0.0099;
    //HectorSlamPoseData.twist.covariance[0]  = 0.0099;
    //HectorSlamPoseData.twist.covariance[7]  = 0.0099;

    HectorSlamPoseData.pose.covariance[0]   = 0.9;
    HectorSlamPoseData.pose.covariance[7]   = 0.9;
    HectorSlamPoseData.pose.covariance[28]  = 0.9;
    HectorSlamPoseData.pose.covariance[35]  = 0.9;
    HectorSlamPoseData.twist.covariance[0]  = 0.9;
    HectorSlamPoseData.twist.covariance[7]  = 0.9;
    //     }
    //     else
    //     {
    //     HectorSlamPoseData.pose.covariance[0]   = 10000.0;
    //     HectorSlamPoseData.pose.covariance[7]   = 10000.0;
    //     HectorSlamPoseData.pose.covariance[28]  = 10000.0;
    //     HectorSlamPoseData.pose.covariance[35]  = 10000.0;
    //     HectorSlamPoseData.twist.covariance[0]  = 10000.0;
    //     HectorSlamPoseData.twist.covariance[7]  = 10000.0;
    //     }

    PublishHectorSlamData();

}

void BehaviorLocalizationWithSimpleEkf::droneSlamDunkPoseCallback(const geometry_msgs::PoseStamped &msg)
{

    ros::Time current_timestamp;

    SlamDunkPoseData.header.stamp = ros::Time::now();
    SlamDunkPoseData.child_frame_id = "fcu";
    SlamDunkPoseData.header.frame_id = "speeds_odom";

    SlamDunkPoseData.pose.pose.position.x = msg.pose.position.x+init_position_x;
    SlamDunkPoseData.pose.pose.position.y = msg.pose.position.y+init_position_y;
    SlamDunkPoseData.pose.pose.position.z = msg.pose.position.z;
    SlamDunkPoseData.pose.pose.orientation.x = msg.pose.orientation.x;
    SlamDunkPoseData.pose.pose.orientation.y = msg.pose.orientation.y;
    SlamDunkPoseData.pose.pose.orientation.z = msg.pose.orientation.z;
    SlamDunkPoseData.pose.pose.orientation.w = msg.pose.orientation.w;


#ifdef slamdunk_has_angle
    /* Calculating Roll, Pitch, Yaw */
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double yaw, pitch, roll;
    m.getRPY(roll, pitch, yaw);

    //subtracting the pitch offset
    pitch = pitch - slamdunk_angle;

    //converting back to quaternions
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll,pitch,yaw);
    SlamDunkPoseData.pose.pose.orientation.x = quaternion.getX();
    SlamDunkPoseData.pose.pose.orientation.y = quaternion.getY();
    SlamDunkPoseData.pose.pose.orientation.z = quaternion.getZ();
    SlamDunkPoseData.pose.pose.orientation.w = quaternion.getW();

#endif

    current_timestamp = ros::Time::now();

    double x_raw_t = msg.pose.position.x;
    double y_raw_t = msg.pose.position.y;
    double z_raw_t = msg.pose.position.z;

    time_t tv_sec; suseconds_t tv_usec;
    {
        tv_sec  = current_timestamp.sec;
        tv_usec = current_timestamp.nsec / 1000.0;
        filtered_derivative_wcb_x.setInput( x_raw_t, tv_sec, tv_usec);
        filtered_derivative_wcb_y.setInput( y_raw_t, tv_sec, tv_usec);
        filtered_derivative_wcb_z.setInput( z_raw_t, tv_sec, tv_usec);
    }

    double x_t, dx_t;
    double y_t, dy_t;
    double z_t, dz_t;
    filtered_derivative_wcb_x.getOutput( x_t,  dx_t);
    filtered_derivative_wcb_y.getOutput( y_t,  dy_t);
    filtered_derivative_wcb_z.getOutput( z_t,  dz_t);

    double y, p, r;
    //     m.getEulerYPR(y, p, r);
    y = EstimatedPose.yaw;
    p = EstimatedPose.pitch;
    r = EstimatedPose.roll;

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    GlobalFrame(0) = (+1)*dx_t;
    GlobalFrame(1) = (+1)*dy_t;
    GlobalFrame(2) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    BodyFrame = RotationMat*GlobalFrame;


    SlamDunkPoseData.twist.twist.linear.x  = (+1) * BodyFrame(0);
    SlamDunkPoseData.twist.twist.linear.y  = (+1) * BodyFrame(1);
    SlamDunkPoseData.twist.twist.linear.z  = dz_t;

    SlamDunkPoseData.pose.covariance[0]   = 0.9;
    SlamDunkPoseData.pose.covariance[7]   = 0.9;
    //SlamDunkPoseData.pose.covariance[21]  = 0.9;
    //SlamDunkPoseData.pose.covariance[28]  = 0.9;
    //SlamDunkPoseData.pose.covariance[35]  = 0.9;
    //    SlamDunkPoseData.twist.covariance[0]  = 0.09;
    //    SlamDunkPoseData.twist.covariance[7]  = 0.09;

    PublishSlamdunkData();
}

void BehaviorLocalizationWithSimpleEkf::droneVOSemanticSLAMPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    VOSemanticSLAMPoseData.child_frame_id = "fcu";
    VOSemanticSLAMPoseData.header.frame_id = "speeds_odom";
    VOSemanticSLAMPoseData.header.stamp = ros::Time::now();

    VOSemanticSLAMPoseData.pose.pose.position.x = msg.pose.position.x;
    VOSemanticSLAMPoseData.pose.pose.position.y = msg.pose.position.y;
    VOSemanticSLAMPoseData.pose.pose.position.z = msg.pose.position.z;

    VOSemanticSLAMPoseData.pose.pose.orientation.x = msg.pose.orientation.x;
    VOSemanticSLAMPoseData.pose.pose.orientation.y = msg.pose.orientation.y;
    VOSemanticSLAMPoseData.pose.pose.orientation.z = msg.pose.orientation.z;
    VOSemanticSLAMPoseData.pose.pose.orientation.w = msg.pose.orientation.w;

    ros::Time current_timestamp = ros::Time::now();
    double x_raw_t = msg.pose.position.x;
    double y_raw_t = msg.pose.position.y;
    double z_raw_t = msg.pose.position.z;

    time_t tv_sec; suseconds_t tv_usec;
    {
        tv_sec  = current_timestamp.sec;
        tv_usec = current_timestamp.nsec / 1000.0;
        filtered_derivative_vos_x.setInput( x_raw_t, tv_sec, tv_usec);
        filtered_derivative_vos_y.setInput( y_raw_t, tv_sec, tv_usec);
        filtered_derivative_vos_z.setInput( z_raw_t, tv_sec, tv_usec);
    }

    double x_t, dx_t;
    double y_t, dy_t;
    double z_t, dz_t;
    filtered_derivative_vos_x.getOutput( x_t,  dx_t);
    filtered_derivative_vos_y.getOutput( y_t,  dy_t);
    filtered_derivative_vos_z.getOutput( z_t,  dz_t);

    double y, p, r;
    //     m.getEulerYPR(y, p, r);
    y = EstimatedPose.yaw;
    p = EstimatedPose.pitch;
    r = EstimatedPose.roll;

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    GlobalFrame(0) = (+1)*dx_t;
    GlobalFrame(1) = (+1)*dy_t;
    GlobalFrame(2) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    BodyFrame = RotationMat*GlobalFrame;


    VOSemanticSLAMPoseData.twist.twist.linear.x  = (+1) * BodyFrame(0);
    VOSemanticSLAMPoseData.twist.twist.linear.y  = (+1) * BodyFrame(1);
    VOSemanticSLAMPoseData.twist.twist.linear.z  = dz_t;


    VOSemanticSLAMPoseData.pose.covariance[0] = 0.9;
    VOSemanticSLAMPoseData.pose.covariance[7] = 0.9;
    VOSemanticSLAMPoseData.pose.covariance[21]  = 0.9;
    VOSemanticSLAMPoseData.pose.covariance[28]  = 0.9;
    VOSemanticSLAMPoseData.pose.covariance[35]  = 0.9;
    VOSemanticSLAMPoseData.twist.covariance[0]  = 0.09;
    VOSemanticSLAMPoseData.twist.covariance[7]  = 0.09;
    VOSemanticSLAMPoseData.twist.covariance[35] = 0.09;

    PublishVOSemanticSLAMPoseData();

}

void BehaviorLocalizationWithSimpleEkf::droneLaserScanPoseCallback(const nav_msgs::Odometry &msg)
{
    LaserScanPoseData.child_frame_id  = "fcu";
    LaserScanPoseData.header.frame_id = "speeds_odom";
    LaserScanPoseData.header.stamp    =  ros::Time::now();

    ros::Time current_timestamp = ros::Time::now();

    double x_raw_t = msg.pose.pose.position.x;
    double y_raw_t = msg.pose.pose.position.y;

    time_t tv_sec; suseconds_t tv_usec;
    {
        tv_sec  = current_timestamp.sec;
        tv_usec = current_timestamp.nsec / 1000.0;
        filtered_derivative_wcb_lx.setInput( x_raw_t, tv_sec, tv_usec);
        filtered_derivative_wcb_ly.setInput( y_raw_t, tv_sec, tv_usec);
    }

    double x_t, dx_t;
    double y_t, dy_t;
    filtered_derivative_wcb_lx.getOutput( x_t,  dx_t);
    filtered_derivative_wcb_ly.getOutput( y_t,  dy_t);


    LaserScanPoseData.pose.pose.position.x  = msg.pose.pose.position.x ;  /*PoseIn.pose.position.x */;
    LaserScanPoseData.pose.pose.position.y  = msg.pose.pose.position.y ;  /*PoseIn.pose.position.y*/;
    LaserScanPoseData.pose.pose.orientation = msg.pose.pose.orientation;


    //convert quaternion to euler angels
    double y, p, r;
    //     m.getEulerYPR(y, p, r);
    y = EstimatedPose.yaw;
    p = EstimatedPose.pitch;
    r = EstimatedPose.roll;

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    GlobalFrame(0) = (+1)*dx_t;
    GlobalFrame(1) = (+1)*dy_t;
    GlobalFrame(2) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    BodyFrame = RotationMat*GlobalFrame;

    LaserScanPoseData.twist.twist.linear.x  = (+1) * BodyFrame(0);
    LaserScanPoseData.twist.twist.linear.y  = (+1) * BodyFrame(1);

    LaserScanPoseData.pose.covariance[0]   = 5.0;
    LaserScanPoseData.pose.covariance[7]   = 5.0;
    LaserScanPoseData.pose.covariance[28]  = 5.0;
    LaserScanPoseData.pose.covariance[35]  = 5.0;
    LaserScanPoseData.twist.covariance[0]  = 5.0;
    LaserScanPoseData.twist.covariance[7]  = 5.0;

    PublishLaserScanPoseData();

}

void BehaviorLocalizationWithSimpleEkf::droneLaserScanWindowPoseCallback(const nav_msgs::Odometry &msg)
{
    LaserScanWindowPoseData.child_frame_id  = "fcu";
    LaserScanWindowPoseData.header.frame_id = "speeds_odom";
    LaserScanWindowPoseData.header.stamp    =  ros::Time::now();

    ros::Time current_timestamp = ros::Time::now();

    double x_raw_t = msg.pose.pose.position.x;
    double y_raw_t = msg.pose.pose.position.y;

    time_t tv_sec; suseconds_t tv_usec;
    {
        tv_sec  = current_timestamp.sec;
        tv_usec = current_timestamp.nsec / 1000.0;
        filtered_derivative_wcb_lwx.setInput( x_raw_t, tv_sec, tv_usec);
        filtered_derivative_wcb_lwy.setInput( y_raw_t, tv_sec, tv_usec);
    }

    double x_t, dx_t;
    double y_t, dy_t;
    filtered_derivative_wcb_lwx.getOutput( x_t,  dx_t);
    filtered_derivative_wcb_lwy.getOutput( y_t,  dy_t);


    LaserScanWindowPoseData.pose.pose.position.x  = msg.pose.pose.position.x ;  /*PoseIn.pose.position.x */;
    LaserScanWindowPoseData.pose.pose.position.y  = msg.pose.pose.position.y ;  /*PoseIn.pose.position.y*/;
    LaserScanWindowPoseData.pose.pose.orientation = msg.pose.pose.orientation;


    //convert quaternion to euler angels
    double y, p, r;
    //     m.getEulerYPR(y, p, r);
    y = EstimatedPose.yaw;
    p = EstimatedPose.pitch;
    r = EstimatedPose.roll;

    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    GlobalFrame(0) = (+1)*dx_t;
    GlobalFrame(1) = (+1)*dy_t;
    GlobalFrame(2) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    BodyFrame = RotationMat*GlobalFrame;

    LaserScanWindowPoseData.twist.twist.linear.x  = (+1) * BodyFrame(0);
    LaserScanWindowPoseData.twist.twist.linear.y  = (+1) * BodyFrame(1);

    LaserScanWindowPoseData.pose.covariance[0]   = 1;
    LaserScanWindowPoseData.pose.covariance[7]   = 1;
    LaserScanWindowPoseData.pose.covariance[28]  = 1;
    LaserScanWindowPoseData.pose.covariance[35]  = 1;
    LaserScanWindowPoseData.twist.covariance[0]  = 1;
    LaserScanWindowPoseData.twist.covariance[7]  = 1;

    PublishLaserScanWindowPoseData();

}

void BehaviorLocalizationWithSimpleEkf::droneOdometryFilteredCallback(const nav_msgs::Odometry &msg)
{
    //-----------------------------------------------------------------------------------------------------------
    // Estimated pose from the EKF
    //-----------------------------------------------------------------------------------------------------------
    static geometry_msgs::PoseStamped estimated_pose;
    static geometry_msgs::TwistStamped estimated_twist;
    
    
    estimated_pose.header.frame_id = msg.header.frame_id;
    estimated_pose.header.stamp = ros::Time::now();
    estimated_pose.pose = msg.pose.pose;

    estimated_twist.header.frame_id = msg.header.frame_id;
    estimated_twist.header.stamp = ros::Time::now();
    estimated_twist.twist = msg.twist.twist;



    //-------------------------------------------------------------------------------------------------------------
    // Estimated Speeds from the EKF
    //-------------------------------------------------------------------------------------------------------------

    //transformVector funtion requires the frame_id as well as the current time
    vin.header.stamp = ros::Time::now();
    vin.header.frame_id = "/base_link_ekf";
    vin.vector.x = msg.twist.twist.linear.x;
    vin.vector.y = msg.twist.twist.linear.y;
    vin.vector.z = msg.twist.twist.linear.z;

    //Using tf to convert the speeds (twist.linear) messages from base_link frame to odom frame as
    //the Aerostack requires speeds in odom frame (i.e world frame)
    try{
        listener.transformVector("/odom_ekf",ros::Time(0) ,vin, "/base_link_ekf",vout);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }


    EstimatedSpeeds.dx   =  vout.vector.x;
    // EstimatedSpeeds.dy   =  vout.vector.y;
    EstimatedSpeeds.dy   = vout.vector.y;
    EstimatedSpeeds.dz   = vout.vector.z;
    // EstimatedSpeeds.dz   = 1.11f;
    EstimatedSpeeds.dyaw = msg.twist.twist.angular.z;


    // PublishEstimatedData();
    estimated_twist.twist.linear.x  = EstimatedSpeeds.dx;
    estimated_twist.twist.linear.y  = EstimatedSpeeds.dy;
    estimated_twist.twist.linear.z  = EstimatedSpeeds.dz;
    estimated_twist.twist.angular.z = EstimatedSpeeds.dyaw;

    if(moduleStarted == true){
        self_localization_pose_pub.publish(estimated_pose);
        self_localization_twist_pub.publish(estimated_twist);
    }
    

    return;
}



void BehaviorLocalizationWithSimpleEkf::PublishImuData()
{
    if(moduleStarted == true)
        droneImuPub.publish(Imu_data);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishRotationAnglesData()
{
    if(moduleStarted == true)
        droneRotationAnglesPub.publish(rotationAngleData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishOdomData()
{
    //Publishing the Controller command data to the ekf
    controllerOdometryData.header.stamp = ros::Time::now();
    controllerOdometryData.header.frame_id = "speeds_odom";
    controllerOdometryData.child_frame_id  = "fcu";

    controllerOdometryData.twist.twist.linear.x  = -PitchRollCmd.pitchCmd;
    controllerOdometryData.twist.twist.linear.y  = -PitchRollCmd.rollCmd;
    controllerOdometryData.twist.twist.linear.z  = DaltitudeCmd.dAltitudeCmd;
    controllerOdometryData.twist.twist.angular.z = +DyawCmd.dYawCmd;
    controllerOdometryData.twist.covariance[0]  = pitch_cmd_co;
    controllerOdometryData.twist.covariance[7]  = roll_cmd_co;
    controllerOdometryData.twist.covariance[14] = daltitude_cmd_co;
    controllerOdometryData.twist.covariance[29] = dyaw_cmd_co;

    if(moduleStarted == true)
        droneOdomPub.publish(controllerOdometryData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishAltitudeData()
{
    if(moduleStarted == true)
        droneAltitudePub.publish(altitudeData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishAltitudeGTData()
{
    if(moduleStarted == true)
        droneAltitudeGTPub.publish(altitudeGTData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishSpeedsData()
{
    if(moduleStarted == true)
        droneGroundSpeedsPub.publish(groundSpeedsData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishPnPData()
{
    if(moduleStarted == true)
        dronePnPPosePub.publish(PnPPoseData);
    return;

}

void BehaviorLocalizationWithSimpleEkf::PublishGPSData()
{
    if(moduleStarted == true)
        droneGPSdataPub.publish(gps_data);
    return;

}

void BehaviorLocalizationWithSimpleEkf::PublishHectorSlamData()
{
    if(moduleStarted == true)
        droneHectorSlamDataPub.publish(HectorSlamPoseData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishSlamdunkData()
{
    if(moduleStarted == true)
        droneSlamDunkPosePub.publish(SlamDunkPoseData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishVOSemanticSLAMPoseData()
{
    if(moduleStarted == true)
        droneVOSematicSLAMPub.publish(VOSemanticSLAMPoseData);
    return;

}

void BehaviorLocalizationWithSimpleEkf::PublishLaserScanPoseData()
{
    if(moduleStarted == true)
        droneLaserScanPosePub.publish(LaserScanPoseData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishLaserScanWindowPoseData()
{
    if(moduleStarted == true)
        droneLaserScanWindowPosePub.publish(LaserScanWindowPoseData);
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishEstimatedData()
{
    if(moduleStarted == true)
    {
        droneEstimatedPosePub.publish(EstimatedPose);
        droneEstimatedSpeedsPub.publish(EstimatedSpeeds);
    }
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishBebopIMUData()
{
    if(moduleStarted == true)
    {
        droneBebopIMUPub.publish(bebop_imu_data);
        //dronebebopVelandAltiPub.publish(bebop_vel_alti_data);
    }
    return;
}

void BehaviorLocalizationWithSimpleEkf::PublishBebopOdomData()
{
    if(moduleStarted == true)
    {
        droneBebopOdomDataPub.publish(bebop_odom_data);
    }

}

bool BehaviorLocalizationWithSimpleEkf::iniciar(std::string configFile)
{
    readConfigs(configFile);

    filtered_derivative_wcb_x.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
    filtered_derivative_wcb_y.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
    filtered_derivative_wcb_z.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);

    filtered_derivative_wcb_x.reset();
    filtered_derivative_wcb_y.reset();
    filtered_derivative_wcb_z.reset();

    filtered_derivative_wcb_lx.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
    filtered_derivative_wcb_ly.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);

    filtered_derivative_wcb_lx.reset();
    filtered_derivative_wcb_ly.reset();

    filtered_derivative_wcb_lwx.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);
    filtered_derivative_wcb_lwy.setTimeParameters( 0.005,0.005,0.200,1.0,100.000);

    filtered_derivative_wcb_lwx.reset();
    filtered_derivative_wcb_lwy.reset();


    filtered_derivative_vos_x.setTimeParameters( 0.05,0.05,0.200,1.0,10.000);
    filtered_derivative_vos_y.setTimeParameters( 0.05,0.05,0.200,1.0,10.000);
    filtered_derivative_vos_z.setTimeParameters( 0.05,0.05,0.200,1.0,10.000);

    filtered_derivative_vos_x.reset();
    filtered_derivative_vos_y.reset();
    filtered_derivative_vos_z.reset();

    bebop_first_yaw_measurement_ = false;

    return true;
}

bool BehaviorLocalizationWithSimpleEkf::readConfigs(std::string configFile)
{

    try
    {
        XMLFileReader my_xml_reader(configFile);

        init_position_x   = my_xml_reader.readDoubleValue("take_off_site:position:x");
        init_position_y   = my_xml_reader.readDoubleValue("take_off_site:position:y");
        init_position_z   = my_xml_reader.readDoubleValue("take_off_site:position:z");

        init_yaw          = my_xml_reader.readDoubleValue("take_off_site:attitude:yaw");
        init_roll         = my_xml_reader.readDoubleValue("take_off_site:attitude:roll");
        init_pitch        = my_xml_reader.readDoubleValue("take_off_site:attitude:pitch");

        //Converting degrees to radians
        init_yaw    = init_yaw*(M_PI/180.0);
        init_roll   = init_roll*(M_PI/180.0);
        init_pitch  = init_pitch*(M_PI/180.0);

        altitude_co_z     = my_xml_reader.readDoubleValue("covariances:altitude:z");
        altitude_co_dz    = my_xml_reader.readDoubleValue("covariances:altitude:dz");


        optical_flow_co_x = my_xml_reader.readDoubleValue("covariances:optical_flow:dx");
        optical_flow_co_y = my_xml_reader.readDoubleValue("covariances:optical_flow:dy");

        pitch_cmd_co      = my_xml_reader.readDoubleValue("covariances:controller_odom:pitch_command");
        roll_cmd_co       = my_xml_reader.readDoubleValue("covariances:controller_odom:roll_command");
        daltitude_cmd_co  = my_xml_reader.readDoubleValue("covariances:controller_odom:daltitude_command");
        dyaw_cmd_co       = my_xml_reader.readDoubleValue("covariances:controller_odom:dyaw_command");
    }

    catch ( cvg_XMLFileReader_exception &e)
    {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }

    return true;
}

}
PLUGINLIB_EXPORT_CLASS(localization_with_simple_ekf::BehaviorLocalizationWithSimpleEkf, nodelet::Nodelet)
