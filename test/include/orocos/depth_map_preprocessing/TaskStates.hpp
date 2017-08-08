/* Generated from orogen/lib/orogen/templates/tasks/TaskStates.hpp */

#ifndef depth_map_preprocessing_TASKS_STATES
#define depth_map_preprocessing_TASKS_STATES

namespace depth_map_preprocessing
{
    
    enum OutlierFilter_STATES
    {
        
            OutlierFilter_INIT,
        
            OutlierFilter_PRE_OPERATIONAL,
        
            OutlierFilter_FATAL_ERROR,
        
            OutlierFilter_EXCEPTION,
        
            OutlierFilter_STOPPED,
        
            OutlierFilter_RUNNING,
        
            OutlierFilter_RUNTIME_ERROR
        
    };
    
    enum ConverterBase_STATES
    {
        
            ConverterBase_INIT,
        
            ConverterBase_PRE_OPERATIONAL,
        
            ConverterBase_FATAL_ERROR,
        
            ConverterBase_EXCEPTION,
        
            ConverterBase_STOPPED,
        
            ConverterBase_RUNNING,
        
            ConverterBase_RUNTIME_ERROR
        
    };
    
    enum PointcloudConverter_STATES
    {
        
            PointcloudConverter_INIT,
        
            PointcloudConverter_PRE_OPERATIONAL,
        
            PointcloudConverter_FATAL_ERROR,
        
            PointcloudConverter_EXCEPTION,
        
            PointcloudConverter_STOPPED,
        
            PointcloudConverter_RUNNING,
        
            PointcloudConverter_RUNTIME_ERROR
        
    };
    
}

#endif

