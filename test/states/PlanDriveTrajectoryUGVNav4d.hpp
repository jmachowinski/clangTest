#pragma once

#include <state_machine/State.hpp>
#include <base/Trajectory.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <rtt/Port.hpp>

extern template class RTT::InputPort<int>;
extern template class RTT::InputPort<std::vector<base::Trajectory> >;
extern template class RTT::OutputPort<base::samples::RigidBodyState>;

namespace init {
    class UGVNav4d;
}

class PlanDriveTrajectoryUGVNav4d : public state_machine::State
{
    std::shared_ptr<init::UGVNav4d> planner;
public:
    PlanDriveTrajectoryUGVNav4d(std::shared_ptr<init::UGVNav4d> planner, State* successState, State* failureState);
    virtual ~PlanDriveTrajectoryUGVNav4d();
    
    virtual void init();
    virtual void enter(const State *lastState);
    virtual void executeFunction();
    virtual void exit();
    
    void setGoal(const base::samples::RigidBodyState &g);
    void setStart(const base::samples::RigidBodyState &start);
    
    bool gotPlan() const;
    
    bool getTrajectory(std::vector<base::Trajectory> &tr);
    
protected:
    
    void startPlanning();
    
    RTT::InputPort<int> *stateReader;
    RTT::InputPort<std::vector<base::Trajectory> > *trReader;

    base::samples::RigidBodyState start;
    base::samples::RigidBodyState goal;
    
    bool hasStarted;
    bool gotGoal;
    
    bool planerFinished;
    
    bool gotTrajectory;
    
    std::vector<base::Trajectory> trajectory;
    
    base::Time startTime;
    
};


