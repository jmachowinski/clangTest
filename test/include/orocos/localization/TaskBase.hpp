/* Generated from orogen/lib/orogen/templates/tasks/TaskBase.hpp */

#ifndef LOCALIZATION_TASK_TASK_BASE_HPP
#define LOCALIZATION_TASK_TASK_BASE_HPP

#include <string>
#include <boost/cstdint.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <localization/TaskStates.hpp>

#include <localization/LocalizationConfig.hpp>
extern template class RTT::Property< ::localization::GICPConfiguration >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <base/geometry/Spline.hpp>
extern template class RTT::Property< ::base::Vector6d >;
#include <base/Pose.hpp>
extern template class RTT::Property< ::base::Pose >;
#include <localization/LocalizationConfig.hpp>
extern template class RTT::Property< ::localization::SubSampling >;
#include <base/geometry/Spline.hpp>
extern template class RTT::Property< ::base::Vector3d >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <base/samples/Pointcloud.hpp>
extern template class RTT::OutputPort< ::base::samples::Pointcloud >;
extern template class RTT::base::ChannelElement< ::base::samples::Pointcloud >;
#include <localization/LocalizationConfig.hpp>
extern template class RTT::OutputPort< ::localization::ICPDebugInformation >;
extern template class RTT::base::ChannelElement< ::localization::ICPDebugInformation >;
#include <base/samples/Pointcloud.hpp>
extern template class RTT::InputPort< ::base::samples::Pointcloud >;
extern template class RTT::base::ChannelElement< ::base::samples::Pointcloud >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::OutputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>





namespace localization{
    class Task;
    /** The oroGen-generated part of localization::Task
     *
     * It is used by oroGen and its plugins to define the task interfaces and set
     * up the component. Never edit yourself, as its content would be
     * overwritten at the next code generation.
     */
    class TaskBase : public ::RTT::TaskContext
        
    {
    protected:
        // Common implementation of interface setup for both task constructors
        void setupComponentInterface();

        /* base implementation of the __orogen_getTID operation
         */
        virtual boost::int32_t __orogen_getTID();
        /* base implementation of the getModelName operation
         */
        virtual ::std::string getModelName();
        RTT::Attribute< ::metadata::TaskContext > _metadata;
        RTT::InputPort< ::base::samples::Pointcloud > _model_pointcloud;
        RTT::InputPort< ::base::samples::RigidBodyState > _pose_update;
        RTT::Operation< boost::int32_t() > ___orogen_getTID;
        RTT::Operation< ::std::string() > _getModelName;
        RTT::OutputPort< ::base::samples::Pointcloud > _debug_map_pointcloud;
        RTT::OutputPort< ::localization::ICPDebugInformation > _icp_debug_information;
        RTT::OutputPort< ::base::samples::RigidBodyState > _pose_samples;
        RTT::OutputPort< boost::int32_t > _state;
        RTT::Property< ::localization::GICPConfiguration > _gicp_configuration;
        RTT::Property< ::std::string > _output_frame_name;
        RTT::Property< ::std::string > _ply_path;
        RTT::Property< ::base::Vector6d > _process_noise_diagonal;
        RTT::Property< ::base::Pose > _start_pose;
        RTT::Property< ::localization::SubSampling > _subsampling;
        RTT::Property< ::base::Vector3d > _subsampling_resolution;
        RTT::Property< bool > _write_debug_pointcloud;

    public:
        
        enum States
        {
            
                INIT = Task_INIT,
            
                PRE_OPERATIONAL = Task_PRE_OPERATIONAL,
            
                FATAL_ERROR = Task_FATAL_ERROR,
            
                EXCEPTION = Task_EXCEPTION,
            
                STOPPED = Task_STOPPED,
            
                RUNNING = Task_RUNNING,
            
                RUNTIME_ERROR = Task_RUNTIME_ERROR,
            
                ICP_ALIGNMENT_FAILED = Task_ICP_ALIGNMENT_FAILED,
            
                MISSING_TRANSFORMATION = Task_MISSING_TRANSFORMATION
            
        };
        

	TaskBase(std::string const& name);
	TaskBase(std::string const& name, RTT::ExecutionEngine* engine);
        ~TaskBase();

        bool start();

        
        // Reimplement TaskCore base methods to export the states to the outside
        // world
        bool configure();
        bool recover();
        bool stop();
        bool cleanup();
        void error();
        void fatal();
        void exception();
        

        
        void report(States state);
        void state(States state);
        void error(States state);
        void fatal(States state);
        void exception(States state);
        States state() const;
        

        bool startHook();
    };

}


#endif

