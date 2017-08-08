/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef LOCAL_TSDF_MAPPER_TASK_PROXY_TASK_HPP
#define LOCAL_TSDF_MAPPER_TASK_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <local_tsdf_mapper/TaskStates.hpp>

#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <base/Time.hpp>
extern template class RTT::Property< ::base::Time >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;
#include <base/samples/RigidBodyState.hpp>
#include <vector>
extern template class RTT::Property< ::std::vector< ::base::samples::RigidBodyState > >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <odometry/ContactState.hpp>
extern template class RTT::InputPort< ::odometry::BodyContactState >;
extern template class RTT::base::ChannelElement< ::odometry::BodyContactState >;
#include <base/samples/DepthMap.hpp>
extern template class RTT::InputPort< ::base::samples::DepthMap >;
extern template class RTT::base::ChannelElement< ::base::samples::DepthMap >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::InputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::OutputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::PRECALCULATED > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::PRECALCULATED > > >;
#include <base/samples/Pointcloud.hpp>
extern template class RTT::InputPort< ::base::samples::Pointcloud >;
extern template class RTT::base::ChannelElement< ::base::samples::Pointcloud >;
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
#include <boost/cstdint.hpp>
#include <string>
#include <transformer/BroadcastTypes.hpp>
#include <vector>

namespace local_tsdf_mapper {

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
        InputProxyPort< ::base::samples::DepthMap > depth_map;
        InputProxyPort< ::base::samples::Pointcloud > pointcloud;
        InputProxyPort< ::odometry::BodyContactState > body_contact_state;
        InputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > > global_map;
        InputProxyPort< ::base::samples::RigidBodyState > dynamic_transformations;
        OutputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::PRECALCULATED > > > local_map;
        OutputProxyPort< ::aggregator::StreamAlignerStatus > transformer_stream_aligner_status;
        OutputProxyPort< ::transformer::TransformerStatus > transformer_status;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< double > &map_size;
        RTT::Property< double > &map_resolution;
        RTT::Property< ::base::Time > &map_update_rate;
        RTT::Property< double > &inner_boundary_size;
        RTT::Property< double > &variance_lower_bound;
        RTT::Property< double > &initial_footprint_size;
        RTT::Property< bool > &init_from_footpoints;
        RTT::Property< bool > &provide_map_in_odometry_frame;
        RTT::Property< bool > &continuous_gloabl_map_integration;
        RTT::Property< bool > &add_initial_footprint_on_full_body_contact;
        RTT::Property< double > &transformer_max_latency;
        RTT::Property< double > &transformer_status_period;
        RTT::Property< ::std::vector< ::base::samples::RigidBodyState > > &static_transformations;
        RTT::Property< double > &depth_map_period;
        RTT::Property< double > &pointcloud_period;
        RTT::Property< double > &body_contact_state_period;
        RTT::Property< ::std::string > &depth_map_provider_frame;
        RTT::Property< ::std::string > &body_frame;
        RTT::Property< ::std::string > &pointcloud_provider_frame;
        RTT::Property< ::std::string > &odometry_frame;
        RTT::Property< ::std::string > &map_frame;


        bool generateMap();
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();
        ::std::vector< ::transformer::TransformationDescription > getNeededTransformations();


};
}
}
#endif // TASK_H
