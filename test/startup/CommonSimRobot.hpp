#pragma once

#include "AbstractRobot.hpp"

namespace init {
    class Simulator;
    class JointDriver;
    class DepthMapProvider;
    class Joypad;
    class GenericRawToMotion2D;
};

namespace orocos_cpp
{
    class Deployment;
}

class CommonSimRobot : public AbstractRobot
{
protected:
    std::shared_ptr<init::DepthMapProvider> depthMapProvider;
    std::shared_ptr<init::PointCloudProvider> pointCloudProvider;
    std::shared_ptr<init::CameraDriver> cameraDriver;
    std::shared_ptr<init::IMUDriver> imuDriver;
    std::shared_ptr<orocos_cpp::Deployment> simDeployment;    
    std::shared_ptr<init::Simulator> sim;
    std::shared_ptr< init::GenericRawToMotion2D > genericRawToMotion2D;
    std::shared_ptr< init::Joypad > joypad;
    
    void initSimulator(std::shared_ptr<orocos_cpp::Deployment> deployment);

    std::shared_ptr<init::JointDriver> getSimJoint(init::Simulator &sim, const std::string &taskName, const::std::string &configName);
public:
    CommonSimRobot();
    virtual std::shared_ptr< init::CameraDriver > getCameraDriver();
    virtual std::shared_ptr< init::IMUDriver > getImuDriver();
    virtual std::shared_ptr< init::PointCloudProvider > getPointCloudProvider();
    virtual std::shared_ptr< init::DepthMapProvider > getDepthMapProvider();
    virtual std::shared_ptr< init::Joypad > getJoypad();
};


