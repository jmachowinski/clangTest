/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef TRAJECTORY_FOLLOWER_TASK_PROXY_TASK_HPP
#define TRAJECTORY_FOLLOWER_TASK_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <trajectory_follower/TaskStates.hpp>

#include <trajectory_follower/TrajectoryFollowerTypes.hpp>
extern template class RTT::Property< ::trajectory_follower::FollowerConfig >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <trajectory_follower/TrajectoryFollowerTypes.hpp>
extern template class RTT::OutputPort< ::trajectory_follower::FollowerData >;
extern template class RTT::base::ChannelElement< ::trajectory_follower::FollowerData >;
#include <trajectory_follower/SubTrajectory.hpp>
extern template class RTT::InputPort< ::trajectory_follower::SubTrajectory >;
extern template class RTT::base::ChannelElement< ::trajectory_follower::SubTrajectory >;
#include <base/commands/Motion2D.hpp>
extern template class RTT::OutputPort< ::base::commands::Motion2D >;
extern template class RTT::base::ChannelElement< ::base::commands::Motion2D >;
#include <base/samples/RigidBodyState.hpp>
extern template class RTT::InputPort< ::base::samples::RigidBodyState >;
extern template class RTT::base::ChannelElement< ::base::samples::RigidBodyState >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;
#include <base/Trajectory.hpp>
#include <vector>
extern template class RTT::InputPort< ::std::vector< ::base::Trajectory > >;
extern template class RTT::base::ChannelElement< ::std::vector< ::base::Trajectory > >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>

namespace trajectory_follower {

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
        InputProxyPort< ::std::vector< ::base::Trajectory > > trajectory;
        InputProxyPort< ::base::samples::RigidBodyState > robot_pose;
        InputProxyPort< ::trajectory_follower::SubTrajectory > holonomic_trajectory;
        OutputProxyPort< ::base::commands::Motion2D > motion_command;
        OutputProxyPort< ::trajectory_follower::FollowerData > follower_data;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< ::trajectory_follower::FollowerConfig > &follower_config;


        ::std::string getModelName();
        boost::int32_t __orogen_getTID();


};
}
}
#endif // TASK_H
