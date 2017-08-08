/* Generated from orogen/lib/orogen/templates/tasks/TaskStates.hpp */

#ifndef localization_TASKS_STATES
#define localization_TASKS_STATES

namespace localization
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
        
            Task_ICP_ALIGNMENT_FAILED,
        
            Task_MISSING_TRANSFORMATION
        
    };
    
    enum PointcloudInMLS_STATES
    {
        
            PointcloudInMLS_INIT,
        
            PointcloudInMLS_PRE_OPERATIONAL,
        
            PointcloudInMLS_FATAL_ERROR,
        
            PointcloudInMLS_EXCEPTION,
        
            PointcloudInMLS_STOPPED,
        
            PointcloudInMLS_RUNNING,
        
            PointcloudInMLS_RUNTIME_ERROR,
        
            PointcloudInMLS_ICP_ALIGNMENT_FAILED,
        
            PointcloudInMLS_MISSING_TRANSFORMATION
        
    };
    
    enum VelodyneInMLS_STATES
    {
        
            VelodyneInMLS_INIT,
        
            VelodyneInMLS_PRE_OPERATIONAL,
        
            VelodyneInMLS_FATAL_ERROR,
        
            VelodyneInMLS_EXCEPTION,
        
            VelodyneInMLS_STOPPED,
        
            VelodyneInMLS_RUNNING,
        
            VelodyneInMLS_RUNTIME_ERROR,
        
            VelodyneInMLS_ICP_ALIGNMENT_FAILED,
        
            VelodyneInMLS_MISSING_TRANSFORMATION
        
    };
    
}

#endif

