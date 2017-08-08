#pragma once
#include "CommonSimRobot.hpp"

namespace init {
    class Simulator;
    class JointDriver;
    class SimVelodyneDriver;
};

namespace orocos_cpp
{
    class Deployment;
}

class AsguardSimRobot : public CommonSimRobot
{
private:
    std::shared_ptr<init::MotionControl2D> motionController;
    std::shared_ptr<init::PositionProvider> odometry;
    std::shared_ptr<init::JointDriver> wheelDrives;
    
public:
    AsguardSimRobot();
    virtual ~AsguardSimRobot();

    virtual std::shared_ptr< init::MotionControl2D > getMotionController();
    virtual std::shared_ptr< init::PositionProvider > getOdometry();
};


