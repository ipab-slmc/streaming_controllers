//
// Copyright (c) 2018-2020, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <streaming_controllers/validity_checker.h>

namespace streaming_controllers
{
class JointGroupPositionController: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
    enum COMMAND_TYPE
    {
        CMD_POSITION = 0,
        CMD_VELOCITY,
        CMD_ACCELERATION,
    };

    JointGroupPositionController() : validity_checkers_("streaming_controllers", "streaming_controllers::ValidityChecker") {};
    ~JointGroupPositionController();

    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);

    void starting(const ros::Time& time);
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    bool load_joint_limits(ros::NodeHandle &n);
    void load_safety_factor();
    void set_safety_factor(double p_value, double v_value, double a_value);

    void limit_position_command(const std::vector<double>& desired_position, std::vector<double>& position_command, std::vector<double>& velocity_command);
    void limit_velocity_command(const std::vector<double>& desired_velocity, std::vector<double>& position_command, std::vector<double>& velocity_command);
    bool is_valid(const std::vector<double>& old_position, const std::vector<double>& new_posiiton);

    std::vector< std::string > joint_names_;
    std::vector< hardware_interface::JointHandle > joints_;
    realtime_tools::RealtimeBuffer<std::vector<double> > position_commands_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double> > velocity_commands_buffer_;
    realtime_tools::RealtimeBuffer<std::vector<double> > acceleration_commands_buffer_;
    unsigned int n_joints_;
    std::vector<double> command_tmp_;
    std::vector<double> command_position_;
    std::vector<double> command_position_last_;
    std::vector<double> command_velocity_;
    std::vector<double> command_velocity_last_;
    std::vector<double> limit_position_min_;
    std::vector<double> limit_position_max_;
    std::vector<double> limit_velocity_;
    std::vector<double> limit_acceleration_;
    std::vector<double> limit_model_position_min_;
    std::vector<double> limit_model_position_max_;
    std::vector<double> limit_model_velocity_;
    std::vector<double> limit_model_acceleration_;
    std::vector<double> error_;
    std::vector<double> error_last_;
    double limit_position_safety_factor_;
    double limit_velocity_safety_factor_;
    double limit_acceleration_safety_factor_;
    COMMAND_TYPE command_type_;
    double dt_;
    double k_p_;
    double k_d_;

private:
    ros::Subscriber sub_command_position_;
    ros::Subscriber sub_command_velocity_;
    ros::Subscriber sub_command_acceleration_;
    void position_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    void velocity_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    void acceleration_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    std::mutex command_mutex_;
    ros::NodeHandle nh_;
    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>  ddr_;
    pluginlib::ClassLoader<streaming_controllers::ValidityChecker> validity_checkers_;
    std::shared_ptr<streaming_controllers::ValidityChecker> validity_checker_;
};

}

namespace
{
template <class SharedPointer>
struct Holder
{
    SharedPointer p;

    Holder(const SharedPointer& p) : p(p) {}
    Holder(const Holder& other) : p(other.p) {}
    Holder(Holder&& other) : p(std::move(other.p)) {}
    void operator()(...) { p.reset(); }
};
}

template <class T>
std::shared_ptr<T> ToStdPtr(const boost::shared_ptr<T>& p)
{
    return std::shared_ptr<T>(p.get(), Holder<boost::shared_ptr<T>>(p));
}

template <class T>
std::shared_ptr<T> ToStdPtr(const std::shared_ptr<T>& p)
{
    return p;
}