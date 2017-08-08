/* Generated from orogen/lib/orogen/templates/tasks/TaskStates.hpp */

#ifndef trajectory_follower_TASKS_STATES
#define trajectory_follower_TASKS_STATES

namespace trajectory_follower
{
    
    enum Task_STATES
    {
        
            Task_INIT,
        
            Task_PRE_OPERATIONAL,
        
            Task_FATAL_ERROR,
        
            Task_EXCEPTION,
        
            Task_STOPPED,
        
            Task_RUNNING,
        
            Task_RUNTIME_ERROR,
        
            Task_FINISHED_TRAJECTORIES,
        
            Task_FOLLOWING_TRAJECTORY,
        
            Task_LATERAL,
        
            Task_SLAM_POSE_INVALID,
        
            Task_STABILITY_FAILED,
        
            Task_TURN_ON_SPOT
        
    };
    
    enum TurnVelocityToSteerAngleTask_STATES
    {
        
            TurnVelocityToSteerAngleTask_INIT,
        
            TurnVelocityToSteerAngleTask_PRE_OPERATIONAL,
        
            TurnVelocityToSteerAngleTask_FATAL_ERROR,
        
            TurnVelocityToSteerAngleTask_EXCEPTION,
        
            TurnVelocityToSteerAngleTask_STOPPED,
        
            TurnVelocityToSteerAngleTask_RUNNING,
        
            TurnVelocityToSteerAngleTask_RUNTIME_ERROR
        
    };
    
}

#endif

