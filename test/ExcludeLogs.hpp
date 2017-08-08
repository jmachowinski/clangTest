#pragma once

#include "TaskNames.hpp"

void addLegExcludes(std::vector<std::string> &excludes, std::string legName)
{
    excludes.push_back("serial_" + legName);
    excludes.push_back(legName + ".joints_info");
    excludes.push_back(legName + ".comm_stats");
    excludes.push_back(legName + ".ndlcom_message_out");
    excludes.push_back(legName + ".processing_time");
    excludes.push_back(legName + ".sampling_time");
}

void setLogExcludes(StartCommon &startup)
{
    std::vector<std::string> excludes;
    excludes.push_back("ft_serial");
    excludes.push_back(TaskNames::canbus);
    excludes.push_back(TaskNames::crexLegInterpolator);
    addLegExcludes(excludes, TaskNames::frontLeftLeg);
    addLegExcludes(excludes, TaskNames::frontRightLeg);
    addLegExcludes(excludes, TaskNames::middleLeftLeg);
    addLegExcludes(excludes, TaskNames::middleRightLeg);
    addLegExcludes(excludes, TaskNames::rearLeftLeg);
    addLegExcludes(excludes, TaskNames::rearRightLeg);

    // Disable logging of debug-map which would write a full MLS at every odometry change:
    excludes.push_back(TaskNames::motionControl + ".mls_map_out");
    // Debug drawings produce a lot of messages:
    excludes.push_back(TaskNames::motionControl + ".debugDrawings");

    startup.setLoggingExcludes(excludes);
};
