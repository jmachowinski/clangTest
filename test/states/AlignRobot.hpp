#pragma once

#include <state_machine/State.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/commands/Motion2D.hpp>
#include <rtt/Port.hpp>

extern template class RTT::InputPort<base::samples::RigidBodyState>;
extern template class RTT::OutputPort<base::commands::Motion2D>;


namespace init {
    class MotionControl2D;
    class PositionProvider;
}

class AlignRobot: public state_machine::State
{
public:
    AlignRobot(std::shared_ptr<init::MotionControl2D> control, std::shared_ptr<init::PositionProvider> posProv, State* success, State* failue);
    virtual ~AlignRobot();
    virtual void init();
    virtual void enter(const State* lastState);
    virtual void executeFunction();
    virtual void exit();
    
    void setGoalPose(const base::samples::RigidBodyState &goal);
    void setYawDiff(double yawDiff);
    
    base::samples::RigidBodyState getCurrentSlamPosition();
    
    bool robotMoved();
    
private:
    std::shared_ptr<init::MotionControl2D> control;
    std::shared_ptr<init::PositionProvider> positionProvider;
    
    bool moved;
    RTT::InputPort<base::samples::RigidBodyState> *poseReader;
    RTT::OutputPort<base::commands::Motion2D> *cmdWriter;
    
    
    base::samples::RigidBodyState targetPose;
};

