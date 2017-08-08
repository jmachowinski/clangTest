/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef SIMPLE_POSE_INTEGRATOR_TASK_PROXY_TASK_HPP
#define SIMPLE_POSE_INTEGRATOR_TASK_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <simple_pose_integrator/TaskStates.hpp>

#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;
#include <base/samples/RigidBodyState.hpp>
#include <vector>
extern template class RTT::Property< ::std::vector< ::base::samples::RigidBodyState > >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::OutputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;
#include <transformer/TransformationStatus.hpp>
extern template class RTT::OutputPort< ::transformer::TransformerStatus >;
extern template class RTT::base::ChannelElement< ::transformer::TransformerStatus >;
#include <aggregator/StreamAlignerStatus.hpp>
extern template class RTT::OutputPort< ::aggregator::StreamAlignerStatus >;
extern template class RTT::base::ChannelElement< ::aggregator::StreamAlignerStatus >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>
#include <transformer/BroadcastTypes.hpp>
#include <vector>

namespace simple_pose_integrator {

namespace proxies {
    
class TaskInitializer : public RTT::corba::TaskContextProxy
{
    public:
        TaskInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class Task : public TaskInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::base::samples::RigidBodyState > pose_samples_in;
        InputProxyPort< ::base::samples::RigidBodyState > dynamic_transformations;
        OutputProxyPort< ::base::samples::RigidBodyState > pose_samples;
        OutputProxyPort< ::aggregator::StreamAlignerStatus > transformer_stream_aligner_status;
        OutputProxyPort< ::transformer::TransformerStatus > transformer_status;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< double > &body_in_odometry_period;
        RTT::Property< double > &transformer_max_latency;
        RTT::Property< double > &transformer_status_period;
        RTT::Property< ::std::vector< ::base::samples::RigidBodyState > > &static_transformations;
        RTT::Property< double > &pose_samples_in_period;
        RTT::Property< ::std::string > &body_frame;
        RTT::Property< ::std::string > &odometry_frame;


        ::std::string getModelName();
        boost::int32_t __orogen_getTID();
        ::std::vector< ::transformer::TransformationDescription > getNeededTransformations();


};
}
}
#endif // TASK_H
