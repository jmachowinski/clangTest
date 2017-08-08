/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef DEPTH_MAP_PREPROCESSING_OUTLIERFILTER_PROXY_TASK_HPP
#define DEPTH_MAP_PREPROCESSING_OUTLIERFILTER_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <depth_map_preprocessing/TaskStates.hpp>


extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
extern template class RTT::Property< boost::int32_t >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <base/samples/DepthMap.hpp>
extern template class RTT::InputPort< ::base::samples::DepthMap >;
extern template class RTT::base::ChannelElement< ::base::samples::DepthMap >;
#include <base/samples/DepthMap.hpp>
extern template class RTT::OutputPort< ::base::samples::DepthMap >;
extern template class RTT::base::ChannelElement< ::base::samples::DepthMap >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>

#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>

namespace depth_map_preprocessing {

namespace proxies {
    
class OutlierFilterInitializer : public RTT::corba::TaskContextProxy
{
    public:
        OutlierFilterInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class OutlierFilter : public OutlierFilterInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for OutlierFilter
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        OutlierFilter(std::string location, bool is_ior = false);

        void synchronize();
        InputProxyPort< ::base::samples::DepthMap > depth_map;
        OutputProxyPort< ::base::samples::DepthMap > filtered_depth_map;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< double > &maximum_angle_to_neighbor;
        RTT::Property< boost::int32_t > &valid_neighbors;


        bool __orogen_setMaximum_angle_to_neighbor(double value);
        bool __orogen_setValid_neighbors(boost::int32_t value);
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();


};
}
}
#endif // TASK_H
