#pragma once

#include <memory>

namespace init {
    class CameraDriver;
    class IMUDriver;
    class MotionControl2D;
    class PositionProvider;
    class PointCloudProvider;
    class DepthMapProvider;
    class Slam3d;
    class RobotFrames;
    class Joypad;
};

class AbstractRobot
{
public:
    virtual ~AbstractRobot();
    virtual std::shared_ptr<init::CameraDriver> getCameraDriver() = 0;
    virtual std::shared_ptr<init::IMUDriver> getImuDriver() = 0;
    virtual std::shared_ptr<init::MotionControl2D> getMotionController() = 0;
    virtual std::shared_ptr<init::PositionProvider> getOdometry() = 0;
    virtual std::shared_ptr<init::PointCloudProvider> getPointCloudProvider() = 0;
    virtual std::shared_ptr<init::DepthMapProvider> getDepthMapProvider() = 0;
    virtual std::shared_ptr<init::Joypad> getJoypad() = 0;
};

