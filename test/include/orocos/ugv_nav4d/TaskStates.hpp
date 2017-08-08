/* Generated from orogen/lib/orogen/templates/tasks/TaskStates.hpp */

#ifndef ugv_nav4d_TASKS_STATES
#define ugv_nav4d_TASKS_STATES

namespace ugv_nav4d
{
    
    enum PathPlanner_STATES
    {
        
            PathPlanner_INIT,
        
            PathPlanner_PRE_OPERATIONAL,
        
            PathPlanner_FATAL_ERROR,
        
            PathPlanner_EXCEPTION,
        
            PathPlanner_STOPPED,
        
            PathPlanner_RUNNING,
        
            PathPlanner_RUNTIME_ERROR,
        
            PathPlanner_FOUND_SOLUTION,
        
            PathPlanner_NO_GOAL,
        
            PathPlanner_NO_MAP,
        
            PathPlanner_NO_SOLUTION,
        
            PathPlanner_NO_START,
        
            PathPlanner_PLANNING
        
    };
    
    enum AreaExploration_STATES
    {
        
            AreaExploration_INIT,
        
            AreaExploration_PRE_OPERATIONAL,
        
            AreaExploration_FATAL_ERROR,
        
            AreaExploration_EXCEPTION,
        
            AreaExploration_STOPPED,
        
            AreaExploration_RUNNING,
        
            AreaExploration_RUNTIME_ERROR,
        
            AreaExploration_AREA_EXPLORED,
        
            AreaExploration_GOALS_GENERATED,
        
            AreaExploration_NO_MAP,
        
            AreaExploration_NO_POSE,
        
            AreaExploration_PLANNING
        
    };
    
}

#endif

