#pragma once

#include "AbstractRobot.hpp"

namespace orocos_cpp
{
    class Deployment;
}

namespace init 
{
    class VelodyneDriver;
    class Joypad;
    class GenericRawToMotion2D;
}

class CommonRealRobot : public AbstractRobot
{
protected:
    std::shared_ptr<orocos_cpp::Deployment> laserScannerDeployment;
    std::shared_ptr<init::CameraDriver> cameraDriver;
    std::shared_ptr<init::IMUDriver> imuDriver;
    std::shared_ptr<init::PointCloudProvider> pointCloudProvider;
    std::shared_ptr<init::VelodyneDriver> velodyneDriver;
    std::shared_ptr< init::Joypad > joypad;
    std::shared_ptr< init::GenericRawToMotion2D > genericRawToMotion2D;
    
public:
    CommonRealRobot();
    virtual ~CommonRealRobot();
    virtual std::shared_ptr< init::CameraDriver > getCameraDriver();
    virtual std::shared_ptr< init::IMUDriver > getImuDriver();
    virtual std::shared_ptr< init::PointCloudProvider > getPointCloudProvider();
    virtual std::shared_ptr< init::DepthMapProvider > getDepthMapProvider();
    virtual std::shared_ptr< init::Joypad > getJoypad();
};


