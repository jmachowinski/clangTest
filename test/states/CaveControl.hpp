#pragma once

#include <state_machine/State.hpp>
#include "DriveTo.hpp"

class CaveControl : public state_machine::State
{
public:
    CaveControl(std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
                std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv,
                State *successState, State *failureState);
    virtual ~CaveControl() = default;
    
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void init();
    virtual void exit();
    
private:
    DriveTo driveTo;
    RTT::InputPort< base::samples::RigidBodyState > *caveInputReader;
    base::samples::RigidBodyState goal;
    
};