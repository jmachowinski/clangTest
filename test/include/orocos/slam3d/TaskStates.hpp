/* Generated from orogen/lib/orogen/templates/tasks/TaskStates.hpp */

#ifndef slam3d_TASKS_STATES
#define slam3d_TASKS_STATES

namespace slam3d
{
    
    enum PointcloudMapper_STATES
    {
        
            PointcloudMapper_INIT,
        
            PointcloudMapper_PRE_OPERATIONAL,
        
            PointcloudMapper_FATAL_ERROR,
        
            PointcloudMapper_EXCEPTION,
        
            PointcloudMapper_STOPPED,
        
            PointcloudMapper_RUNNING,
        
            PointcloudMapper_RUNTIME_ERROR
        
    };
    
    enum OcTreeMapper_STATES
    {
        
            OcTreeMapper_INIT,
        
            OcTreeMapper_PRE_OPERATIONAL,
        
            OcTreeMapper_FATAL_ERROR,
        
            OcTreeMapper_EXCEPTION,
        
            OcTreeMapper_STOPPED,
        
            OcTreeMapper_RUNNING,
        
            OcTreeMapper_RUNTIME_ERROR
        
    };
    
    enum PointcloudMapper2D_STATES
    {
        
            PointcloudMapper2D_INIT,
        
            PointcloudMapper2D_PRE_OPERATIONAL,
        
            PointcloudMapper2D_FATAL_ERROR,
        
            PointcloudMapper2D_EXCEPTION,
        
            PointcloudMapper2D_STOPPED,
        
            PointcloudMapper2D_RUNNING,
        
            PointcloudMapper2D_RUNTIME_ERROR
        
    };
    
    enum PointcloudToBinary_STATES
    {
        
            PointcloudToBinary_INIT,
        
            PointcloudToBinary_PRE_OPERATIONAL,
        
            PointcloudToBinary_FATAL_ERROR,
        
            PointcloudToBinary_EXCEPTION,
        
            PointcloudToBinary_STOPPED,
        
            PointcloudToBinary_RUNNING,
        
            PointcloudToBinary_RUNTIME_ERROR
        
    };
    
    enum PointcloudFilter_STATES
    {
        
            PointcloudFilter_INIT,
        
            PointcloudFilter_PRE_OPERATIONAL,
        
            PointcloudFilter_FATAL_ERROR,
        
            PointcloudFilter_EXCEPTION,
        
            PointcloudFilter_STOPPED,
        
            PointcloudFilter_RUNNING,
        
            PointcloudFilter_RUNTIME_ERROR
        
    };
    
    enum OcTreeFilter_STATES
    {
        
            OcTreeFilter_INIT,
        
            OcTreeFilter_PRE_OPERATIONAL,
        
            OcTreeFilter_FATAL_ERROR,
        
            OcTreeFilter_EXCEPTION,
        
            OcTreeFilter_STOPPED,
        
            OcTreeFilter_RUNNING,
        
            OcTreeFilter_RUNTIME_ERROR
        
    };
    
    enum ScanConverter_STATES
    {
        
            ScanConverter_INIT,
        
            ScanConverter_PRE_OPERATIONAL,
        
            ScanConverter_FATAL_ERROR,
        
            ScanConverter_EXCEPTION,
        
            ScanConverter_STOPPED,
        
            ScanConverter_RUNNING,
        
            ScanConverter_RUNTIME_ERROR,
        
            ScanConverter_MULTIPLE_INPUT_CONNECTIONS
        
    };
    
    enum LineScanConverter_STATES
    {
        
            LineScanConverter_INIT,
        
            LineScanConverter_PRE_OPERATIONAL,
        
            LineScanConverter_FATAL_ERROR,
        
            LineScanConverter_EXCEPTION,
        
            LineScanConverter_STOPPED,
        
            LineScanConverter_RUNNING,
        
            LineScanConverter_RUNTIME_ERROR
        
    };
    
    enum MLSMapProjector_STATES
    {
        
            MLSMapProjector_INIT,
        
            MLSMapProjector_PRE_OPERATIONAL,
        
            MLSMapProjector_FATAL_ERROR,
        
            MLSMapProjector_EXCEPTION,
        
            MLSMapProjector_STOPPED,
        
            MLSMapProjector_RUNNING,
        
            MLSMapProjector_RUNTIME_ERROR
        
    };
    
}

#endif

