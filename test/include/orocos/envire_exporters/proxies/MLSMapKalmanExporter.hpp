/* Generated from orogen/lib/orogen/templates/proxies/Task.hpp */

#ifndef ENVIRE_EXPORTERS_MLSMAPKALMANEXPORTER_PROXY_TASK_HPP
#define ENVIRE_EXPORTERS_MLSMAPKALMANEXPORTER_PROXY_TASK_HPP


#include <rtt/transports/corba/TaskContextProxy.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Operation.hpp>
#include <rtt/Port.hpp>
#include <orocos_cpp_base/ProxyPort.hpp>
#include <envire_exporters/TaskStates.hpp>


extern template class RTT::Property< double >;
#include <boost/cstdint.hpp>
#include <string>
extern template class RTT::Property< ::std::string >;

#include <orogen_metadata/Metadata.hpp>
extern template class RTT::Attribute< ::metadata::TaskContext >;

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <envire_orogen/typekit/BinaryBufferHelper.hpp>
#include <maps/grid/MLSMap.hpp>
extern template class RTT::OutputPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
extern template class RTT::base::ChannelElement< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > >;
#include <boost/cstdint.hpp>
extern template class RTT::OutputPort< boost::int32_t >;
extern template class RTT::base::ChannelElement< boost::int32_t >;



#include <boost/cstdint.hpp>
#include <boost/cstdint.hpp>
#include <string>

namespace envire_exporters {

namespace proxies {
    
class MLSMapKalmanExporterInitializer : public RTT::corba::TaskContextProxy
{
    public:
        MLSMapKalmanExporterInitializer(std::string location, bool is_ior = false);

        static void initTypes();
};

class MLSMapKalmanExporter : public MLSMapKalmanExporterInitializer
{
    protected:

    public:
        
        static const std::string ModelName;
        
        /** TaskContext constructor for MLSMapKalmanExporter
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        MLSMapKalmanExporter(std::string location, bool is_ior = false);

        void synchronize();
        OutputProxyPort< ::envire::core::SpatioTemporal< ::maps::grid::MLSMap< ::maps::grid::MLSConfig::KALMAN > > > map;
        OutputProxyPort< boost::int32_t > state;


        RTT::Property< ::std::string > &path;
        RTT::Property< double > &mls_resolution;


        void writeMap();
        ::std::string getModelName();
        boost::int32_t __orogen_getTID();


};
}
}
#endif // TASK_H
