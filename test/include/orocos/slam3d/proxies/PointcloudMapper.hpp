/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef SLAM3D_POINTCLOUDMAPPER_PROXY_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <slam3d/TaskStates.hpp>

#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <slam3d/GICPConfiguration.hpp>
extern template class RTT::Property< ::slam3d::GICPConfiguration >;
#include <slam3d/GICPConfiguration.hpp>
extern template class RTT::Property< ::slam3d::GICPConfiguration >;
#include <slam3d/GridConfiguration.hpp>
extern template class RTT::Property< ::slam3d::GridConfiguration >;
#include <maps/grid/MLSConfig.hpp>
extern template class RTT::Property< ::maps::grid::MLSConfig >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

extern template class RTT::Property< double >;
#include <base/Pose.hpp>
extern template class RTT::Property< ::base::Pose >;
#include <base/samples/RigidBodyState.hpp>
#include <vector>
extern template class RTT::Property< ::std::vector< ::base::samples::RigidBodyState > >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< bool >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <pcl/PCLPointCloud2WithTimestamp.hpp>
extern template class RTT::OutputPort< ::pcl::PCLPointCloud2 >;
extern template class RTT::base::ChannelElement< ::pcl::PCLPointCloud2 >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::OutputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::OutputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::OutputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
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
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>

#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>

#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>
#include <transformer/BroadcastTypes.hpp>
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>

namespace slam3d {

namespace proxies {
    
class PointcloudMapperInitializer : public RTT::corba::TaskContextProxy
{
    public:
        PointcloudMapperInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class PointcloudMapper : public PointcloudMapperInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for PointcloudMapper
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PointcloudMapper(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::base::samples::Pointcloud > scan;
        InputProxyPort< ::base::samples::RigidBodyState > dynamic_transformations;
        OutputProxyPort< ::pcl::PCLPointCloud2 > cloud;
        OutputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > > mls_map;
        OutputProxyPort< ::base::samples::RigidBodyState > robot2map;
        OutputProxyPort< ::base::samples::RigidBodyState > odometry2map;
        OutputProxyPort< ::aggregator::StreamAlignerStatus > transformer_stream_aligner_status;
        OutputProxyPort< ::transformer::TransformerStatus > transformer_status;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< double > &transformer_max_latency;
        RTT::Property< double > &transformer_status_period;
        RTT::Property< ::std::vector< ::base::samples::RigidBodyState > > &static_transformations;
        RTT::Property< ::std::string > &laser_frame;
        RTT::Property< ::std::string > &robot_frame;
        RTT::Property< ::std::string > &odometry_frame;
        RTT::Property< double > &scan_resolution;
        RTT::Property< double > &map_resolution;
        RTT::Property< boost::int32_t > &optimization_rate;
        RTT::Property< boost::int32_t > &map_publish_rate;
        RTT::Property< double > &neighbor_radius;
        RTT::Property< boost::int32_t > &max_neighbor_links;
        RTT::Property< boost::int32_t > &patch_building_range;
        RTT::Property< double > &map_outlier_radius;
        RTT::Property< boost::int32_t > &map_outlier_neighbors;
        RTT::Property< double > &min_translation;
        RTT::Property< double > &min_rotation;
        RTT::Property< bool > &use_odometry;
        RTT::Property< bool > &add_odometry_edges;
        RTT::Property< ::base::Pose > &start_pose;
        RTT::Property< bool > &use_odometry_heading;
        RTT::Property< ::slam3d::GICPConfiguration > &gicp_config;
        RTT::Property< ::slam3d::GICPConfiguration > &gicp_coarse_config;
        RTT::Property< boost::int32_t > &log_type;
        RTT::Property< boost::int32_t > &log_level;
        RTT::Property< ::std::string > &robot_name;
        RTT::Property< ::std::string > &map_frame;
        RTT::Property< bool > &use_colors_as_viewpoints;
        RTT::Property< ::slam3d::GridConfiguration > &grid_config;
        RTT::Property< ::maps::grid::MLSConfig > &grid_mls_config;
        RTT::Property< ::std::string > &envire_path;
        RTT::Property< ::std::string > &apriori_ply_map;


        bool generate_cloud();
        bool generate_map();
        bool optimize();
        bool force_add();
        bool write_graph();
        bool write_envire();
        bool write_ply(::std::string const & folder_path);
        bool __orogen_setMap_resolution(double value);
        bool __orogen_setOptimization_rate(boost::int32_t value);
        bool __orogen_setMap_publish_rate(boost::int32_t value);
        bool __orogen_setMap_outlier_radius(double value);
        bool __orogen_setMap_outlier_neighbors(boost::int32_t value);
        bool __orogen_setLog_level(boost::int32_t value);
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();
        ::std::vector< ::transformer::TransformationDescription > getNeededTransformations();


};
}
}
#endif // TASK_H
