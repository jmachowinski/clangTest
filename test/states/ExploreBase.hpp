#pragma once
#include <state_machine/State.hpp>
#include "DriveTo.hpp"
#include "GenerateMap.hpp"
#include <deque>

extern template class RTT::InputPort<std::vector<base::samples::RigidBodyState> >;

class ExploreBase : public state_machine::State
{
public:
    ExploreBase(const std::string &stateName, std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
            std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv, 
            std::shared_ptr<init::MLSProvider> mapper, State* success, State* failue);
    
    virtual ~ExploreBase();

    virtual void init();
    virtual void enter(const State *lastState);
    virtual void executeFunction();
    virtual void exit() {};

protected:
    RTT::InputPort<std::vector<base::samples::RigidBodyState> > *targetVectorReader;
    virtual RTT::InputPort<std::vector<base::samples::RigidBodyState> > *getTargetVectorReader() = 0;
    /**
     * Triggers a new set of goals.
     * Returns false, if trigger failed
     * */
    virtual bool triggerNewGoals() = 0;

    /**
     * Checks the status of the goal 
     * generation. Returns true if 
     * goal are generated and can
     * be executed
     * */
    virtual bool checkGenerationStatus() = 0;
    
    
    bool triggered;
    size_t cnt;
    DriveTo drive;
    GenerateMap genMap;
};


