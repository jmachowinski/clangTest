#pragma once
#include <state_machine/State.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "DriveTo.hpp"
#include <random>

class RandomTargetSelector: public state_machine::State
{
public:
    RandomTargetSelector(std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
            std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv);
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();

    void setTargets(const std::vector<base::samples::RigidBodyState> &targets);
private:
    std::shared_ptr<init::PositionProvider> posProv;
    std::vector<base::samples::RigidBodyState> targets;
    RTT::InputPort<base::samples::RigidBodyState> *poseReader;
    DriveTo driveTo;
    std::mt19937 mt;
};

