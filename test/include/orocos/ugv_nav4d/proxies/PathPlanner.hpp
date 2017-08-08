/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef UGV_NAV4D_PATHPLANNER_PROXY_TASK_HPP
#define UGV_NAV4D_PATHPLANNER_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <ugv_nav4d/TaskStates.hpp>


extern template class RTT::Property< double >;

extern template class RTT::Property< double >;
#include <base/Time.hpp>
extern template class RTT::Property< ::base::Time >;
#include <motion_planning_libraries/Config.hpp>
extern template class RTT::Property< ::motion_planning_libraries::Mobility >;
#include <motion_planning_libraries/sbpl/SplinePrimitivesConfig.hpp>
extern template class RTT::Property< ::motion_planning_libraries::SplinePrimitivesConfig >;
#include <ugv_nav4d/TraversabilityConfig.hpp>
extern template class RTT::Property< ::ugv_nav4d::TraversabilityConfig >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include </usr/include/boost/shared_ptr.hpp>
#include <vizkit3d_debug_drawings/commands/CommandBuffer.h>
extern template class RTT::OutputPort< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > >;
extern template class RTT::base::ChannelElement< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > >;
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::InputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
#include <ugv_nav4d/PreComputedMotions.hpp>
#include <vector>
extern template class RTT::OutputPort< ::std::vector< ::ugv_nav4d::Motion > >;
extern template class RTT::base::ChannelElement< ::std::vector< ::ugv_nav4d::Motion > >;
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
#include <base/Trajectory.hpp>
#include <vector>
extern template class RTT::OutputPort< ::std::vector< ::base::Trajectory > >;
extern template class RTT::base::ChannelElement< ::std::vector< ::base::Trajectory > >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>
#include <boost/cstdint.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace ugv_nav4d {

namespace proxies {
    
class PathPlannerInitializer : public RTT::corba::TaskContextProxy
{
    public:
        PathPlannerInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class PathPlanner : public PathPlannerInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for PathPlanner
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PathPlanner(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > > map;
        OutputProxyPort< ::std::vector< ::base::Trajectory > > trajectory;
        OutputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::TraversabilityMap3d< ::maps::grid::TraversabilityNodeBase * > > > tr_map;
        OutputProxyPort< ::boost::shared_ptr< ::vizkit3dDebugDrawings::CommandBuffer > > debugDrawings;
        OutputProxyPort< ::std::vector< ::ugv_nav4d::Motion > > motionPrims;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< ::base::Time > &maxTime;
        RTT::Property< ::motion_planning_libraries::SplinePrimitivesConfig > &primConfig;
        RTT::Property< ::ugv_nav4d::TraversabilityConfig > &travConfig;
        RTT::Property< ::motion_planning_libraries::Mobility > &mobilityConfig;
        RTT::Property< double > &distToGround;
        RTT::Property< double > &initialPatchRadius;


        boost::int32_t triggerPathPlanning(::base::samples::RigidBodyState const & start_position, ::base::samples::RigidBodyState const & goal_position);
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();


};
}
}
#endif // TASK_H
