#pragma once

namespace TaskNames
{
    const std::string mars("mars");;
    const std::string robotFrames("robot_frames");
    const std::string jointDispatcher("joint_dispatcher");
    const std::string motionControl("motion_control");
    const std::string camera("camera");
    const std::string hbridgeReader("hbridge_reader");
    const std::string hbridgeWriter("hbridge_writer");
    const std::string canbus("canbus");
    const std::string wheelDrives("wheel_drives");
    const std::string xsens("xsens");
    const std::string velodyne("velodyne");
    const std::string odometry("odometry");
    const std::string simple_mapper("simple_mapper");
    const std::string tofCamera("tof_camera");
    const std::string local_mapper("local_mapper");
    const std::string localization("localization");
    const std::string areaExploration("area_exploration");
    const std::string safetyControl("safety_control");
    
    const std::string depthMapConverter("depth_map_converter");
    const std::string depthMapOutlierFilter("depth_map_outlier_filter");
    const std::string slam3d("slam3d");
    const std::string fastPoseIntegrator("pose_provider");
    
    const std::string pathPlanner("path_planner");
    const std::string trajectoryFollower("trajectory_follower");
    
    const std::string forceTorqueSensors("ft_sensors");

    const std::string footForceEstimator("foot_force_estimator");
    
    const std::string frontLeftLeg("front_left_leg");
    const std::string frontRightLeg("front_right_leg");
    const std::string middleLeftLeg("middle_left_leg");
    const std::string middleRightLeg("middle_right_leg");
    const std::string rearLeftLeg("rear_left_leg");
    const std::string rearRightLeg("rear_right_leg");
    const std::string relayboard("relayboard");    
    const std::string crexLegInterpolator("crexLegInterpolator");
    const std::string joypad("joypad");
    const std::string genericRawToMotion2D("generic_raw_to_motion_2D");
    
    const std::string odometryInterface("odometry_interface");
    const std::string contactPointOdometry("contact_point_odometry");
}
