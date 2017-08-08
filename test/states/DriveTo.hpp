#pragma once

#include <state_machine/State.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "PlanDriveTrajectoryUGVNav4d.hpp"
#include "ExecuteDriveTrajectory.hpp"
#include "AlignRobot.hpp"
#include "Forward.hpp"

class DriveTo : public state_machine::State
{
public:
    DriveTo(std::shared_ptr<init::UGVNav4d> planner, std::shared_ptr<init::MotionControl2D> control, 
            std::shared_ptr<init::TrajectoryFollower> follower, std::shared_ptr<init::PositionProvider> posProv, 
            State *successState, State *failureState);
    virtual ~DriveTo();
    
    void setGoal(const base::samples::RigidBodyState& goal);
    
    virtual void init();
    virtual void enter(const State *lastState);
    virtual void executeFunction();
    virtual void exit();
    
    /**
     * Returns if the state was able to drive
     * to the given goal position
     * */
    bool reachedGoal();
    
    /**
     * Adds an Area, that should be avoided by
     * the drive planner.
     * 
     * @return id of the Area
     * */
    int addAvoidArea(const Eigen::Vector3d &position, double radius, double drivability);
    
    /**
     * Removed an area from the drive planner
     * */
    void removeAvoidArea(int id);
    
    /**
     * returns if the robot actually moved during the execution of this state
     * */
    bool robotMoved();
    
protected:
    
    bool moved;
    
    PlanDriveTrajectoryUGVNav4d plan;
    ExecuteDriveTrajectory execute;
    AlignRobot align;
    
    base::samples::RigidBodyState goal;
    
    bool gotGoal;
};


