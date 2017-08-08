/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef UGV_NAV4D_AREAEXPLORATION_PROXY_TASK_HPP
#define UGV_NAV4D_AREAEXPLORATION_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <ugv_nav4d/TaskStates.hpp>

#include <ugv_nav4d/Config.hpp>
extern template class RTT::Property< ::ugv_nav4d::CostFunctionParameters >;

extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <ugv_nav4d/TraversabilityConfig.hpp>
extern template class RTT::Property< ::ugv_nav4d::TraversabilityConfig >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include </usr/include/boost/shared_ptr.hpp>
#include <vizkit3d_debug_drawings/commands/CommandBuffer.h>
extern template class RTT::OutputPort< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > >;
extern template class RTT::base::ChannelElement< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::OutputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <base/samples/RigidBodyState.hpp>
#include <vector>
extern template class RTT::OutputPort< ::std::vector< ::base::samples::RigidBodyState > >;
extern template class RTT::base::ChannelElement< ::std::vector< ::base::samples::RigidBodyState > >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::InputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
extern template class RTT::OutputPort< ::envire::core::SpatioTemporal< ::maps::grid::TraversabilityMap3d< ::maps::grid::TraversabilityNodeBase * > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::TraversabilityMap3d< ::maps::grid::TraversabilityNodeBase * > > >;



#include <boost/cstdint.hpp>
#include <ugv_nav4d/OrientedBox.hpp>
#include <boost/cstdint.hpp>
#include <string>

namespace ugv_nav4d {

namespace proxies {
    
class AreaExplorationInitializer : public RTT::corba::TaskContextProxy
{
    public:
        AreaExplorationInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class AreaExploration : public AreaExplorationInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for AreaExploration
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        AreaExploration(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > > map;
        InputProxyPort< ::base::samples::RigidBodyState > pose_samples;
        OutputProxyPort< ::std::vector< ::base::samples::RigidBodyState > > goals_out;
        OutputProxyPort< ::base::samples::RigidBodyState > goal_out_best;
        OutputProxyPort< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > > debugDrawings;
        OutputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::TraversabilityMap3d< ::maps::grid::TraversabilityNodeBase * > > > tr_map;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< ::ugv_nav4d::TraversabilityConfig > &travConfig;
        RTT::Property< ::ugv_nav4d::CostFunctionParameters > &costConfig;
        RTT::Property< double > &distToGround;
        RTT::Property< double > &initialPatchRadius;


        void clearPlannerMap();
        void calculateGoals(::ugv_nav4d::OrientedBoxConfig const & area);
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();


};
}
}
#endif // TASK_H
