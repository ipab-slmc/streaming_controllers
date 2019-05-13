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
#include <pluginlib/class_list_macros.hpp>

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

    JointGroupPositionController() = default;
    ~JointGroupPositionController();

    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);

    void starting(const ros::Time& time);
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    bool load_joint_limits(ros::NodeHandle &n);
    void load_safety_factor();
    void set_safety_factor(double p_value, double v_value, double a_value);

    void limit_position_command(const std::vector<double>& desired_position, std::vector<double>& position_command, std::vector<double>& velocity_command);
    void limit_velocity_command(const std::vector<double>& desired_velocity, std::vector<double>& position_command, std::vector<double>& velocity_command);

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
    double velocity_alpha_;

private:
    ros::Subscriber sub_command_position_;
    ros::Subscriber sub_command_velocity_;
    ros::Subscriber sub_command_acceleration_;
    void position_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    void velocity_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    void acceleration_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg);
    std::mutex command_mutex_;
    ros::NodeHandle nh_;
};

}