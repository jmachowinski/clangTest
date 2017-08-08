#pragma once

#include "CommonRealRobot.hpp"

namespace orocos_cpp
{
    class Deployment;
}

namespace init 
{
    class VelodyneDriver;
    class RobotFrames;
    class Canbus;
    class HBridge;
    class JointDriver;
}

class AsguardRobot : public CommonRealRobot
{
    std::shared_ptr<init::MotionControl2D> motionController;
    std::shared_ptr<init::PositionProvider> odometry;
    std::shared_ptr<init::RobotFrames> robotFrames;

    
protected:

    virtual std::shared_ptr< init::MotionControl2D > getMotionController();
    virtual std::shared_ptr< init::PositionProvider > getOdometry();
    
    std::shared_ptr<orocos_cpp::Deployment> asguardBaseDeployment;

    std::shared_ptr<init::Canbus> canbus;
    std::shared_ptr<init::HBridge> hbridges;
    
    std::shared_ptr<init::JointDriver> wheelDrives;
public:
    AsguardRobot();
    virtual ~AsguardRobot();    
};


