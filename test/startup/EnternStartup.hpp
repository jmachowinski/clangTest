#pragma once

#include "AbstractRobot.hpp"

//provide full declaration here, because 
//the using code needs to know that 
//init::CameraDriver is a subclass of init::Base
#include <lib_init/CameraDriver.hpp>
#include <lib_init/IMUDriver.hpp>
#include "init/Slam3dMLS.hpp"
#include <lib_init/PositionProvider.hpp>
#include <lib_init/MotionControl2D.hpp>
#include <lib_init/UGVNav4d.hpp>
#include <lib_init/TrajectoryFollower.hpp>
#include <lib_init/PoseProvider.hpp>
#include <lib_init/OutlierFilter.hpp>
#include "init/LocalTSDFMapper.hpp"
#include <lib_init/Localization.hpp>
#include <lib_init/UGVAreaExploration.hpp>

class EnternStartupBase
{
protected:
    virtual std::shared_ptr<AbstractRobot> getRealRobot() = 0;
    virtual std::shared_ptr<AbstractRobot> getSimRobot() = 0;

    void init(bool simulationActive);
public :
    EnternStartupBase();
    virtual ~EnternStartupBase();

    std::shared_ptr<AbstractRobot> robot;
    
    std::shared_ptr<init::CameraDriver> cameraDriver;
    std::shared_ptr<init::IMUDriver> imuDriver;
    std::shared_ptr<init::MotionControl2D> motionController;
    std::shared_ptr<init::PositionProvider> odometry;
    std::shared_ptr<init::Localization> localizer;
    std::shared_ptr<init::PointCloudProvider> pointCloudProvider;
    std::shared_ptr<init::OutlierFilter> outlierFilter;
    std::shared_ptr<init::UGVNav4d> pathPlanner;
    std::shared_ptr<init::TrajectoryFollower> trajectoryFollower;
    std::shared_ptr<init::PoseProvider> fastPoseIntegrator;
    std::shared_ptr<init::Joypad> joypad;
    
    //higher level 
    std::shared_ptr<init::Slam3dMLS> slam3d;
    std::shared_ptr<init::LocalTSDFMapper> local_mapper;
    
    std::shared_ptr<init::UGVAreaExploration> areaExploration;
};

template <class RealRobot, class SimRobot>
class EnternStartup : public EnternStartupBase
{
protected:
    std::shared_ptr<AbstractRobot> getRealRobot()
    {
        return std::make_shared<RealRobot>();
    };
    
    std::shared_ptr<AbstractRobot> getSimRobot()
    {
        return std::make_shared<SimRobot>();
    };
public :
    EnternStartup(bool simulationActive) : EnternStartupBase()
    {
        init(simulationActive);
    };
};

