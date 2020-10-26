#pragma once

#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>

namespace streaming_controllers
{

class ValidityChecker
{
public:
    ValidityChecker() = default;
    virtual ~ValidityChecker() = default;
    virtual void init(ros::NodeHandle &n) = 0;
    virtual bool is_valid(const std::vector<double>& old_position, const std::vector<double>& new_posiiton) = 0;
};

} // namespace streaming_controllers