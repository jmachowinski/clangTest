#pragma once

#include <lib_init/PositionProvider.hpp>
#include <lib_init/PointCloudProvider.hpp>
#include <lib_init/MLSProvider.hpp>
#include <slam3d/proxies/PointcloudMapper.hpp>

namespace init
{

class Slam3dMLS : public PositionProvider , public MLSProvider
{
    PointCloudProvider &pclProv;
    
public:
    Slam3dMLS(PositionProvider &odometry, PointCloudProvider &pclProv, const std::string &mapperTaskName);
    Slam3dMLS(PointCloudProvider &pclProv, const std::string &mapperTaskName);
    virtual ~Slam3dMLS();
    
    virtual bool connect();
    
    virtual OutputProxyPort< base::samples::RigidBodyState >& getPositionSamples();

    virtual OutputProxyPort<envire::core::SpatioTemporal< maps::grid::MLSMapKalman> >& getMapPort();
    
    virtual bool generateMap();

    
    DependentTask<slam3d::proxies::PointcloudMapper> mapper;
};


}