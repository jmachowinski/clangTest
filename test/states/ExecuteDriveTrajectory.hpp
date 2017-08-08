#pragma once

#include <state_machine/State.hpp>
#include <base/Trajectory.hpp>
#include <rtt/Port.hpp>
#include <trajectory_follower/SubTrajectory.hpp>

extern template class RTT::InputPort<int32_t>;
extern template class RTT::OutputPort<std::vector<base::Trajectory> >;
// extern template class RTT::OutputPort<std::vector<trajectory_follower::SubTrajectory> >;

namespace init 
{
    class TrajectoryFollower;
    class MotionControl2D;
};

class ExecuteDriveTrajectory : public state_machine::State
{
public:
    ExecuteDriveTrajectory(std::shared_ptr<init::TrajectoryFollower> follower, 
                           std::shared_ptr<init::MotionControl2D> motionController, State *successState, State *failState);
    virtual ~ExecuteDriveTrajectory();
    
    virtual void init();
    virtual void enter(const State *lastState1);
    virtual void executeFunction();
    virtual void exit();
    
    void setTrajectory(const std::vector< base::Trajectory >& traj);
    void setTrajectory(const std::vector< trajectory_follower::SubTrajectory >& traj);
    
protected:
    std::shared_ptr<init::TrajectoryFollower> follower;
    std::shared_ptr<init::MotionControl2D> motionController;
    
    RTT::InputPort<int32_t> *trStateReader;
    RTT::OutputPort<std::vector<trajectory_follower::SubTrajectory> > *trWriter;
    RTT::OutputPort<std::vector<base::Trajectory> > *oldTrWriter;
    std::vector<base::Trajectory> curTrajectory;
    
    bool gotTrajectory;
    
    bool trStarted;
    
};


