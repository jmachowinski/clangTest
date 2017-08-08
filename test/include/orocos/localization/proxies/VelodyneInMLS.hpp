/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef LOCALIZATION_VELODYNEINMLS_PROXY_TASK_HPP
#define LOCALIZATION_VELODYNEINMLS_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <localization/Task.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <localization/TaskStates.hpp>

#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <base/samples/RigidBodyState.hpp>
#include <vector>
extern template class RTT::Property< ::std::vector< ::base::samples::RigidBodyState > >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/DepthMap.hpp>
extern template class RTT::InputPort< ::base::samples::DepthMap >;
extern template class RTT::base::ChannelElement< ::base::samples::DepthMap >;
#include <transformer/TransformationStatus.hpp>
extern template class RTT::OutputPort< ::transformer::TransformerStatus >;
extern template class RTT::base::ChannelElement< ::transformer::TransformerStatus >;
#include <aggregator/StreamAlignerStatus.hpp>
extern template class RTT::OutputPort< ::aggregator::StreamAlignerStatus >;
extern template class RTT::base::ChannelElement< ::aggregator::StreamAlignerStatus >;



#include <transformer/BroadcastTypes.hpp>
#include <vector>

namespace localization {

namespace proxies {
    
class VelodyneInMLSInitializer : public RTT::corba::TaskContextProxy
{
    public:
        VelodyneInMLSInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class VelodyneInMLS : public VelodyneInMLSInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for VelodyneInMLS
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        VelodyneInMLS(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::base::samples::Pointcloud > model_pointcloud;
        InputProxyPort< ::base::samples::RigidBodyState > pose_update;
        InputProxyPort< ::base::samples::DepthMap > lidar_samples;
        InputProxyPort< ::base::samples::RigidBodyState > dynamic_transformations;
        OutputProxyPort< ::base::samples::RigidBodyState > pose_samples;
        OutputProxyPort< ::base::samples::Pointcloud > debug_map_pointcloud;
        OutputProxyPort< ::localization::ICPDebugInformation > icp_debug_information;
        OutputProxyPort< boost::int32_t > state;
        OutputProxyPort< ::aggregator::StreamAlignerStatus > transformer_stream_aligner_status;
        OutputProxyPort< ::transformer::TransformerStatus > transformer_status;


        RTT::Property< ::std::string > &ply_path;
        RTT::Property< ::base::Pose > &start_pose;
        RTT::Property< ::localization::GICPConfiguration > &gicp_configuration;
        RTT::Property< ::base::Vector6d > &process_noise_diagonal;
        RTT::Property< ::localization::SubSampling > &subsampling;
        RTT::Property< ::base::Vector3d > &subsampling_resolution;
        RTT::Property< ::std::string > &output_frame_name;
        RTT::Property< bool > &write_debug_pointcloud;
        RTT::Property< double > &maximum_angle_to_neighbor;
        RTT::Property< boost::int32_t > &minimum_valid_neighbors;
        RTT::Property< double > &transformer_max_latency;
        RTT::Property< double > &transformer_status_period;
        RTT::Property< ::std::vector< ::base::samples::RigidBodyState > > &static_transformations;
        RTT::Property< double > &lidar_samples_period;
        RTT::Property< ::std::string > &velodyne_frame;
        RTT::Property< ::std::string > &body_frame;
        RTT::Property< ::std::string > &odometry_frame;


        ::std::string getModelName();
        boost::int32_t __orogen_getTID();
        ::std::vector< ::transformer::TransformationDescription > getNeededTransformations();


};
}
}
#endif // TASK_H
