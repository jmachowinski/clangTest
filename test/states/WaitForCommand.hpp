#pragma once

#include <state_machine/State.hpp>
#include <commands/commandsTypes.hpp>
#include <rtt/InputPort.hpp>

#include "DriveTo.hpp"
#include "ExploreArea.hpp"
#include "GenerateMap.hpp"

class WaitForCommand : public state_machine::State
{
    RTT::InputPort<commands::Command> *cmdIn;
    
    DriveTo driveTo;
    ExploreArea exploreArea;
    GenerateMap genMap;
public:
    WaitForCommand(std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
            std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv,
            std::shared_ptr<init::MLSProvider> mapper, std::shared_ptr<init::UGVAreaExploration> exploration,
            State* success, State* failue);
    
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
};

