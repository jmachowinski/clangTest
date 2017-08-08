#pragma once

#include "ExploreBase.hpp"
#include <lib_init/UGVAreaExploration.hpp>

class ExploreArea : public ExploreBase
{
public:
    ExploreArea(std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
            std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv, 
            std::shared_ptr<init::MLSProvider> mapper, std::shared_ptr<init::UGVAreaExploration> exploration, 
            State* success, State* failue);

    virtual void init();
    virtual void enter(const State* lastState);
    virtual void exit();
    void setArea(const ugv_nav4d::OrientedBoxConfig &area);
    
protected:
    bool gotArea;
    ugv_nav4d::OrientedBoxConfig area;

    RTT::InputPort<int> *stateReader;
    virtual RTT::InputPort< std::vector< base::samples::RigidBodyState > >* getTargetVectorReader();
    virtual bool triggerNewGoals();
    virtual bool checkGenerationStatus();
    
    std::shared_ptr<init::UGVAreaExploration> exploration;

};


