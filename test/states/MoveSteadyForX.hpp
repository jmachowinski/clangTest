#pragma once
#include <state_machine/State.hpp>
#include <lib_init/MotionControl2D.hpp>
#include <lib_init/PositionProvider.hpp>

class MoveSteadyForX : public state_machine::State
{
    std::shared_ptr<init::MotionControl2D> control;
    std::shared_ptr<init::PositionProvider> pose;
    
    RTT::InputPort<base::samples::RigidBodyState> *poseReader;
    RTT::OutputPort<base::commands::Motion2D> *cmdWriter;
    
    base::samples::RigidBodyState startPose;
    base::commands::Motion2D cmd;
    
    double maxDistance;
    
public:
    MoveSteadyForX(const std::shared_ptr<init::MotionControl2D> &motionCtrl, const std::shared_ptr<init::PositionProvider> &pose, State* success);
    
    void setDistAndCmd(base::commands::Motion2D cmd, double distance);
    
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
};


