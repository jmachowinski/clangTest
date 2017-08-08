#include <iostream>

#include <lib_init/Base.hpp>
#include <lib_init/CommonStart.hpp>
#include <lib_config/Bundle.hpp>
#include <state_machine/Config.hpp>

#include "states/DummyState.hpp"
#include "states/Sleep.hpp"
#include "states/GenerateMap.hpp"
#include "states/RandomTargetSelector.hpp"

#include "startup/EnternStartup.hpp"
#include "startup/AsguardRobot.hpp"
#include "startup/AsguardSimRobot.hpp"

#include "AsguardSmurf.hpp"
#include "ExcludeLogs.hpp"

#include <lib_init/MLSMapKalmanExporter.hpp>
#include <trajectory_follower/proxies/Task.hpp>
#include <simple_pose_integrator/proxies/Task.hpp>

typedef EnternStartup<AsguardRobot, AsguardSimRobot> AsguardStartup;

void loadTargets(const std::vector<base::Vector3d> positions, const std::vector<base::Orientation> orientations, std::vector<base::samples::RigidBodyState>& targets)
{
    base::samples::RigidBodyState goal_pose;	        
    for (unsigned int i = 0; i < positions.size(); i++)
    {			
        goal_pose.initSane();        
        goal_pose.position = positions[i];
        goal_pose.orientation = orientations[i];
        targets.push_back(goal_pose);
    }
}

int main(int argc, char** argv)
{
    StartCommon startCommon(argc, argv);
    setLogExcludes(startCommon);

    bool sim = false;
    for(int i = 0; i < argc; i++)
        if(std::string(argv[i]) == std::string("sim"))
            sim = true;

    DummyState dummy;
    AsguardSmurf robot;
    GenerateMap *genMap = nullptr;
    RandomTargetSelector *rts = nullptr;
    std::string smurfPath(libConfig::Bundle::getInstance().getConfigurationDirectory() + state_machine::Config::getConfig().getValue("robotSmurf"));
    robot.loadFromSmurf(smurfPath);

    std::shared_ptr<init::TrajectoryFollower> trFollower;
    std::shared_ptr<init::MLSMapKalmanExporter> mapLoader;
    std::shared_ptr<init::UGVNav4d> planner;
    std::shared_ptr<init::PoseProvider> fastPoseIntegrator;

    startCommon.run<AsguardStartup>(&robot, [&] (AsguardStartup &start, std::vector<init::Base *> &toInit) {

        std::cout << "Running TdoT asguard executable" << std::endl;
        fastPoseIntegrator.reset(new init::PoseProvider(*start.localizer, TaskNames::fastPoseIntegrator));
        trFollower.reset(new init::TrajectoryFollower(*fastPoseIntegrator, *start.motionController.get(), TaskNames::trajectoryFollower));
        mapLoader.reset(new init::MLSMapKalmanExporter("MLS_Loader"));
        planner.reset(new init::UGVNav4d(*mapLoader.get(), TaskNames::pathPlanner));

        toInit = std::vector<init::Base *>({
                planner.get(), 
                trFollower.get(),
                mapLoader.get(),
                fastPoseIntegrator.get(),
                start.odometry.get()
                });


        rts = new RandomTargetSelector(planner, start.motionController, trFollower, start.localizer);

        std::vector<base::samples::RigidBodyState> targets;

        std::vector<base::Vector3d> positions;
        std::vector<base::Orientation> orientations;

        /*positions.push_back(base::Vector3d(-0.46, 0.26, -0.16));
          orientations.push_back(base::Orientation(Eigen::AngleAxisd(3.07, Eigen::Vector3d::UnitZ())));
          positions.push_back(base::Vector3d(-3.39, -0.31, -0.15));
          orientations.push_back(base::Orientation(Eigen::AngleAxisd(3.08, Eigen::Vector3d::UnitZ())));*/
        // First test behind RH5, (didn't work) - 14.06.2017-
        /*
           positions.push_back(base::Vector3d(-3.80704, 0.08376, -0.170018));
           orientations.push_back(base::Orientation(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())));
           positions.push_back(base::Vector3d(-11.016, 1.8297, -0.289551));
           orientations.push_back(base::Orientation(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())));
           positions.push_back(base::Vector3d(-24.8079, 0.0886953, -0.232089));
           orientations.push_back(base::Orientation(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())));
           positions.push_back(base::Vector3d(-32.1744, 0.140914, -0.165105));
           orientations.push_back(base::Orientation(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ())));
         */

        //positions.push_back(base::Vector3d(2.92463, -0.107396, -0.302534));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(5.72167, -0.380535, -0.381703));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d( 10.8061, -1.95619, -0.479297));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(6.77455, 0.0998613, 0.218728 ));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(15.8145, -4.85188, 0.859545 ));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(4.26264, -6.4549, 0.237561));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(-0.142904, -10.2614, 0.653329 ));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(9.35825, -12.7502, 0.396045));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));

        // DEMO Behind RH1
        positions.push_back(base::Vector3d(2.48495, 0.376565, -0.0964368 ));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d( 4.48284, -0.0691919, -0.0318608));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(6.46147, -0.825914, 0.0531781));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(9.43791, -1.09659, 0.210128 ));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(9.10118, -3.9572, 0.197976 ));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(7.63462, -7.71548, 0.121888));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(-1.51847, -1.87632, -0.222879));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(-2.55288, -5.22218, -0.381494));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(-2.75232, 1.37566, -0.271205));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(10.0458, -6.62885, 0.311045));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(11.9903, 0.98652, 0.348847));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(11.3409, 2.59624, 0.247355));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        positions.push_back(base::Vector3d(-2.35638, 2.93522, -0.200286));
        orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));

        // Corridor RH5
        //positions.push_back(base::Vector3d(0.28408, -0.047556, -0.159109));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(3.33149, -0.227094, -0.0878983));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));
        //positions.push_back(base::Vector3d(2.02096, -0.226287, -0.120229));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())));

        // DEMO Space Hall
        //positions.push_back(base::Vector3d(1.5, 5.52732, 0.345865));
        //orientations.push_back(base::Orientation(Eigen::AngleAxisd(2.25, Eigen::Vector3d::UnitZ())));



        loadTargets(positions, orientations, targets);

        //FIXME, set correct targets
        rts->setTargets(targets);

        genMap = new GenerateMap(rts, mapLoader);

        return genMap;

    }
    );

    return 0;
}
