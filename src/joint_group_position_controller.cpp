#include <streaming_controllers/joint_group_position_controller.h>

#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>

namespace streaming_controllers
{

constexpr double eps = std::numeric_limits<double>::epsilon();

JointGroupPositionController::~JointGroupPositionController()
{
    sub_command_position_.shutdown();
    sub_command_velocity_.shutdown();
    sub_command_acceleration_.shutdown();
}

bool JointGroupPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
    nh_ = n;

    ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(n));

    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
        return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0)
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }
    for(int i=0; i<n_joints_; ++i)
    {
        try
        {
            joints_.push_back(hw->getHandle(joint_names_[i]));  
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }
    }

    dt_ = 1.0;

    position_commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    velocity_commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    acceleration_commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    command_type_ = CMD_POSITION;

    load_joint_limits(n);
    load_safety_factor();

    n.param<double>("Kp", k_p_, 0.2);
    n.param<double>("Kd", k_d_, 0.8);
    ddr_->registerVariable("Kp", &k_p_, "", 0.0, 1000.0);
    ddr_->registerVariable("Kd", &k_d_, "", 0.0, 1000.0);
    ddr_->registerVariable("PositionSafetyFactor", &limit_position_safety_factor_, "", 0.0, 1.0);
    ddr_->registerVariable("VelocitySafetyFactor", &limit_velocity_safety_factor_, "", 0.0, 1.0);
    ddr_->registerVariable("AccelerationSafetyFactor", &limit_acceleration_safety_factor_, "", 0.0, 1.0);
    ddr_->publishServicesTopics();
    
    ROS_WARN_STREAM("Kp: " << k_p_ << ", Kd: " << k_d_);

    // Load validity checker if defined
    std::vector<std::string> checkers = validity_checkers_.getDeclaredClasses();
    std::vector<std::string> params;
    n.getParamNames(params);
    std::string checker_name = "";
    for (auto name : checkers)
    {
        if (n.hasParam(name.substr(22)))
        {
            checker_name = name.substr(22);
            break;
        }
    }
    if (checker_name != "")
    {
        auto checker = ToStdPtr(validity_checkers_.createInstance("streaming_controllers/" + checker_name));
        ROS_WARN_STREAM("Loading " << checker_name);
        ros::NodeHandle nh(n, checker_name);
        checker->init(nh);
        validity_checker_ = checker;
    }
    else
    {
        ROS_WARN_STREAM("No validity checker defined");
    }
    
    
    sub_command_position_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointGroupPositionController::position_command_callback, this);
    sub_command_velocity_ = n.subscribe<std_msgs::Float64MultiArray>("command_velocity", 1, &JointGroupPositionController::velocity_command_callback, this);
    sub_command_acceleration_ = n.subscribe<std_msgs::Float64MultiArray>("command_acceleration", 1, &JointGroupPositionController::acceleration_command_callback, this);
    return true;
}

template <typename DerivedA,typename DerivedB>
Eigen::VectorXd vector_min(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>&  b)
{
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedA, DerivedB);
    Eigen::VectorXd ret(a.rows());
    for(int i = 0; i < a.rows(); ++i)
    {
        if (a(i) < b(i))
            ret(i) = a(i);
        else
            ret(i) = b(i);
    }
    return ret;
}

template <typename DerivedA,typename DerivedB>
Eigen::VectorXd vector_max(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>&  b)
{
    EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedA, DerivedB);
    Eigen::VectorXd ret(a.rows());
    for(int i = 0; i < a.rows(); ++i)
    {
        if (a(i) > b(i))
            ret(i) = a(i);
        else
            ret(i) = b(i);
    }
    return ret;
}

template <typename DerivedA,typename DerivedB, typename DerivedC>
Eigen::VectorXd vector_clamp(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>&  l, const Eigen::MatrixBase<DerivedC>&  u)
{
    return vector_min(vector_max(a, l), u);
}

template <typename Derived>
Eigen::VectorXd sqrt(const Eigen::MatrixBase<Derived>& a, const double& val = 0.0)
{
    Eigen::VectorXd ret(a.rows());
    for(int i = 0; i < a.rows(); ++i)
    {
        if(a(i) > eps)
            ret(i) = std::sqrt(a(i));
        else
            ret(i) = val;        
    }
    return ret;
}

template <typename DerivedA, typename DerivedB>
Eigen::VectorXd divide(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, const double& val = 0.0)
{
    Eigen::VectorXd ret(a.rows());
    for(int i = 0; i < a.rows(); ++i)
    {
        if(std::fabs(b(i)) > eps)
            ret(i) = a(i) / b(i);
        else
            ret(i) = val;        
    }
    return ret;
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename DerivedA, typename DerivedB>
Eigen::VectorXd limit_velocity(const Eigen::MatrixBase<DerivedA>& vel, const Eigen::MatrixBase<DerivedB>& v_max)
{
    Eigen::VectorXd::Index max_id;
    double max_val = ((vel.array().abs()-v_max.array())/v_max.array()).maxCoeff(&max_id);
    if (max_val>0.0)
    {
        if (std::fabs(vel(max_id))>eps)
            return vel*v_max(max_id)/std::fabs(vel(max_id));
        else
            return vel*0.0;
    }
    else
        return vel;
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
Eigen::VectorXd limit_position(const Eigen::MatrixBase<DerivedA>& pos, const Eigen::MatrixBase<DerivedB>& p_min, const Eigen::MatrixBase<DerivedC>& p_max)
{
    Eigen::VectorXd center = 0.5 * (p_min + p_max);
    return limit_velocity(pos - center, 0.5 * (p_max - p_min)) + center;
}

template <typename DerivedA, typename DerivedB>
Eigen::VectorXd limit_acceleration(const Eigen::MatrixBase<DerivedA>& acc, const Eigen::MatrixBase<DerivedB>& a_max)
{
    return limit_velocity(acc, a_max);
}

void JointGroupPositionController::limit_position_command(const std::vector<double>& _desired_position, std::vector<double>& _position_command, std::vector<double>& _velocity_command)
{
    Eigen::Map<const Eigen::VectorXd> desired_position(_desired_position.data(), _desired_position.size());
    Eigen::VectorXd position_last = Eigen::Map<Eigen::VectorXd>(command_position_.data(), command_position_.size());
    Eigen::VectorXd velocity_last = Eigen::Map<Eigen::VectorXd>(command_velocity_.data(), command_velocity_.size());
    std::vector<double> _desired_velocity(desired_position.rows());
    Eigen::Map<Eigen::VectorXd> desired_velocity(_desired_velocity.data(), _desired_velocity.size());

    desired_velocity = (desired_position - position_last)*k_p_ - velocity_last*k_d_;
    double d = desired_velocity.norm();
    if (d>eps)
        desired_velocity=desired_velocity/d*std::pow(d, 5.0);
   
    limit_velocity_command(_desired_velocity, _position_command, _velocity_command);
}


void JointGroupPositionController::limit_velocity_command(const std::vector<double>& _desired_velocity, std::vector<double>& _position_command, std::vector<double>& _velocity_command)
{
    Eigen::Map<const Eigen::VectorXd> desired_velocity(_desired_velocity.data(), _desired_velocity.size());
    Eigen::Map<Eigen::VectorXd> position_command(_position_command.data(), _position_command.size());
    Eigen::Map<Eigen::VectorXd> velocity_command(_velocity_command.data(), _velocity_command.size());
    std::vector<double> old_position = command_position_;
    std::vector<double> old_velocity_ = command_velocity_;
    Eigen::Map<const Eigen::VectorXd> position_last(old_position.data(), old_position.size());
    Eigen::Map<const Eigen::VectorXd> velocity_last(old_velocity_.data(), old_velocity_.size());
    Eigen::Map<const Eigen::VectorXd> limit_position_min(limit_position_min_.data(), limit_position_min_.size());
    Eigen::Map<const Eigen::VectorXd> limit_position_max(limit_position_max_.data(), limit_position_max_.size());
    Eigen::Map<const Eigen::VectorXd> limit_velocity_max(limit_velocity_.data(), limit_velocity_.size());
    Eigen::Map<const Eigen::VectorXd> limit_acceleration_max(limit_acceleration_.data(), limit_acceleration_.size());

    Eigen::VectorXd acceleration(position_command.rows());
    velocity_command = limit_velocity(desired_velocity, limit_velocity_max);
    //ROS_WARN_STREAM(desired_velocity.transpose()<<", "<<velocity_command.transpose());
    double scale = 1.0;

    Eigen::VectorXd bound = -0.5*velocity_last.array().pow(2.0)/limit_acceleration_max.array();
    for (int i=0; i<velocity_command.rows(); ++i)
    {
        if(position_last(i)<limit_position_min(i)+bound(i) && (velocity_last(i)<0.0 || velocity_command(i)-velocity_last(i)<0.0 ) )
            scale = 0.0;
        if(position_last(i)>limit_position_max(i)+bound(i) && (velocity_last(i)>0.0 || velocity_command(i)-velocity_last(i)>0.0 ) )
            scale = 0.0;
    }

    velocity_command = scale*velocity_command - velocity_last;

    acceleration = limit_acceleration(velocity_command/dt_, limit_acceleration_max);
    double w = std::min(1.0, std::max(0.0 ,divide(velocity_command/dt_, acceleration, 1e300).array().abs().minCoeff()));
    acceleration = acceleration*w;

    position_command = position_last + velocity_last*dt_ + 0.5*acceleration*dt_*dt_;
    velocity_command = velocity_last + acceleration*dt_;
    if (!is_valid(old_position, _position_command))
    {
        velocity_command.setZero();
        scale = 1.0;
        bound = -0.5*velocity_last.array().pow(2.0)/limit_acceleration_max.array();
        for (int i=0; i<velocity_command.rows(); ++i)
        {
            if(position_last(i)<limit_position_min(i)+bound(i) && (velocity_last(i)<0.0 || velocity_command(i)-velocity_last(i)<0.0 ) )
                scale = 0.0;
            if(position_last(i)>limit_position_max(i)+bound(i) && (velocity_last(i)>0.0 || velocity_command(i)-velocity_last(i)>0.0 ) )
                scale = 0.0;
        }

        velocity_command = scale*velocity_command - velocity_last;

        acceleration = limit_acceleration(velocity_command/dt_, limit_acceleration_max);
        w = std::min(1.0, std::max(0.0 ,divide(velocity_command/dt_, acceleration, 1e300).array().abs().minCoeff()));
        acceleration = acceleration*w;

        position_command = position_last + velocity_last*dt_ + 0.5*acceleration*dt_*dt_;
        velocity_command = velocity_last + acceleration*dt_;
    }
}

bool JointGroupPositionController::is_valid(const std::vector<double>& old_position, const std::vector<double>& new_posiiton)
{
    if (!validity_checker_)
    {
        return true;
    }
    else
    {
        return validity_checker_->is_valid(old_position, new_posiiton);
    }
    
}

void JointGroupPositionController::update(const ros::Time& /*time*/, const ros::Duration& dt)
{
    COMMAND_TYPE command_type;
    dt_ = dt.toSec();
    set_safety_factor(limit_position_safety_factor_, limit_velocity_safety_factor_, limit_acceleration_safety_factor_);
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_type = command_type_;
        switch (command_type_)
        {
            case CMD_POSITION:
                command_tmp_ = *position_commands_buffer_.readFromRT();
                break;
            case CMD_VELOCITY:
                command_tmp_ = *velocity_commands_buffer_.readFromRT();
                break;
            case CMD_ACCELERATION:
                command_tmp_ = *acceleration_commands_buffer_.readFromRT();
                for(int i=0; i<n_joints_; ++i) 
                {
                    command_tmp_[i] = command_velocity_[i] + command_tmp_[i]*dt_;
                }
                break;
        }
    }

    switch (command_type)
    {
        case CMD_POSITION:
            limit_position_command(command_tmp_, command_position_, command_velocity_);
            break;
        case CMD_VELOCITY:
        case CMD_ACCELERATION:
            limit_velocity_command(command_tmp_, command_position_, command_velocity_);
            break;
    }
    
    //limit_command(command_tmp_, command_position_, command_velocity_);

    for(int i=0; i<n_joints_; ++i)
    {  
        joints_[i].setCommand(command_position_[i]);  
    }
    
}

void JointGroupPositionController::position_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg) 
{
    if(msg->data.size()!=n_joints_)
    { 
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    position_commands_buffer_.writeFromNonRT(msg->data);
    command_type_ = CMD_POSITION;
}

void JointGroupPositionController::velocity_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg) 
{
    if(msg->data.size()!=n_joints_)
    { 
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    velocity_commands_buffer_.writeFromNonRT(msg->data);
    command_type_ = CMD_VELOCITY;    
}

void JointGroupPositionController::acceleration_command_callback(const std_msgs::Float64MultiArrayConstPtr& msg) 
{
    if(msg->data.size()!=n_joints_)
    { 
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    acceleration_commands_buffer_.writeFromNonRT(msg->data);
    command_type_ = CMD_ACCELERATION;
}

void JointGroupPositionController::starting(const ros::Time& time)
{
    // Start controller with current joint positions
    std::vector<double> & position_commands = *position_commands_buffer_.readFromRT();
    for(int i=0; i<n_joints_; ++i)
    {
        position_commands[i]=joints_[i].getPosition();
    }
    command_position_ = position_commands;
    command_position_last_ = command_position_;
    command_velocity_.assign(n_joints_, 0.0);
    command_velocity_last_.assign(n_joints_, 0.0);
    error_.assign(n_joints_, 0.0);
    error_last_.assign(n_joints_, 0.0);
    command_type_ = CMD_POSITION;

    // Start controller with zero velocity
    std::vector<double> & velocity_commands = *velocity_commands_buffer_.readFromRT();
    for(int i=0; i<n_joints_; ++i)
    {
        velocity_commands[i]=0.0;
    }

    // Start controller with zero acceleration
    std::vector<double> & acceleration_commands = *acceleration_commands_buffer_.readFromRT();
    for(int i=0; i<n_joints_; ++i)
    {
        acceleration_commands[i]=0.0;
    }
}

void JointGroupPositionController::load_safety_factor()
{
    nh_.param<double>("PositionSafetyFactor", limit_position_safety_factor_, 1.0);
    nh_.param<double>("VelocitySafetyFactor", limit_velocity_safety_factor_, 0.1);
    nh_.param<double>("AccelerationSafetyFactor", limit_acceleration_safety_factor_, 0.1);
    ROS_WARN_STREAM("Safetey factors: " << limit_position_safety_factor_ << ", " << limit_velocity_safety_factor_ << ", " << limit_acceleration_safety_factor_);
    set_safety_factor(limit_position_safety_factor_, limit_velocity_safety_factor_, limit_acceleration_safety_factor_);
}

void JointGroupPositionController::set_safety_factor(double p_value, double v_value, double a_value)
{
    if(p_value <= 0.0 || p_value > 1.0)
    {
        ROS_ERROR_STREAM("Position safety factor outside of range (0,1>. Got  " << p_value << ", keeping current value: " << limit_position_safety_factor_);
    }
    if(p_value <= 0.0 || p_value > 1.0)
    {
        ROS_ERROR_STREAM("Velocity safety factor outside of range (0,1>. Got  " << v_value << ", keeping current value: " << limit_velocity_safety_factor_);
    }
    if(p_value <= 0.0 || p_value > 1.0)
    {
        ROS_ERROR_STREAM("Acceleration safety factor outside of range (0,1>. Got  " << a_value << ", keeping current value: " << limit_acceleration_safety_factor_);
    }
    limit_position_safety_factor_ = p_value;
    limit_velocity_safety_factor_ = v_value;
    limit_acceleration_safety_factor_ = a_value;
    for(int i=0; i<n_joints_; ++i)
    {
        limit_position_min_[i] = (limit_model_position_min_[i] + limit_model_position_max_[i]) * 0.5 - (limit_model_position_max_[i] - limit_model_position_min_[i]) * 0.5 * limit_position_safety_factor_;
        limit_position_max_[i] = (limit_model_position_min_[i] + limit_model_position_max_[i]) * 0.5 + (limit_model_position_max_[i] - limit_model_position_min_[i]) * 0.5 * limit_position_safety_factor_;
        limit_velocity_[i] =  limit_model_velocity_[i] * limit_velocity_safety_factor_;
        limit_acceleration_[i] = limit_model_acceleration_[i] * limit_acceleration_safety_factor_;
    }
}

bool JointGroupPositionController::load_joint_limits(ros::NodeHandle &n)
{
    std::string robot_description_param;
    std::string robot_model_string; 
    n.param<std::string>("RobotDescriptionParameter", robot_description_param, "robot_description");
    n.param<std::string>(robot_description_param, robot_model_string, "");

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_string));

    limit_model_position_min_.resize(n_joints_);
    limit_model_position_max_.resize(n_joints_);
    limit_model_velocity_.resize(n_joints_);
    limit_model_acceleration_.resize(n_joints_);

    for(int i=0; i<n_joints_; ++i)
    {
        urdf::JointConstSharedPtr urdf_joint(model->getJoint(joint_names_[i]));
        if(!urdf_joint)
        {
            ROS_ERROR_STREAM(joint_names_[i] << " couldn't be retrieved from model description");
            return false;
        }
        limit_model_position_min_[i] = urdf_joint->limits->lower;
        limit_model_position_max_[i] = urdf_joint->limits->upper;
        limit_model_velocity_[i] = urdf_joint->limits->velocity;
        limit_model_acceleration_[i] = urdf_joint->limits->effort;
    }
    limit_position_min_ = limit_model_position_min_;
    limit_position_max_ = limit_model_position_max_;
    limit_velocity_ = limit_model_velocity_;
    limit_acceleration_ = limit_model_acceleration_;
    return true;
}

}

PLUGINLIB_EXPORT_CLASS(streaming_controllers::JointGroupPositionController, controller_interface::ControllerBase)