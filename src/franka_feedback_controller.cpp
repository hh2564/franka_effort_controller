#include <franka_effort_controller/franka_feedback_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka/robot_state.h>
#include "std_msgs/Float64MultiArray.h"

namespace franka_effort_controller {
    bool FeedbackController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    //checking to see if the default parameters can be access through node handle
    //and also getting information and interfaces  
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
            "controller init!");
        return false;
    }

    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
                        << publish_rate);
    }
   
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "JointImpedanceController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }
    realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison> torques_publisher_;
    torques_publisher_.init(node_handle, "torque_comparison", 1);

    std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

    ros::NodeHandle nh;

    pospub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose", 1000);
    torquepub = nh.advertise<std_msgs::Float64MultiArray>("/tau_command", 1000);


    return true;

    }

    void FeedbackController::starting(const ros::Time& /* time */) {
    //getting the intial time to generate command in update 
    elapsed_time_ = ros::Duration(0.0);


    

    
    }


    void FeedbackController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
    std::array<double, 42> jacobian_array =
    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // position
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Matrix<double, 6, 6> kp{};  
    kp << 0.75,0,0,0,0,0,
         0,1.5,0,0,0,0, 
         0,0,0.75,0,0,0,
         0,0,0,0.75,0,0,
         0,0,0,0,1.5,0, 
         0,0,0,0,0,0.75;

    Eigen::Matrix<double, 6, 6> kd{};  
    kd << 0.5,0,0,0,0,0,
         0,0.5,0,0,0,0, 
         0,0,0.5,0,0,0,
         0,0,0,0.5,0,0,
         0,0,0,0,0.5,0, 
         0,0,0,0,0,0.5;
    
    Eigen::Matrix<double, 6, 1> desired_pos{};
    desired_pos << 0.501,0.503,0.478,1.5707,0,0.707;
    Eigen::Affine3d curr_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position_curr_(curr_transform.translation());
    Eigen::Quaterniond orientation_curr_(Eigen::Quaterniond(curr_transform.linear()));
    auto euler = orientation_curr_.toRotationMatrix().eulerAngles(0, 1, 2);


    Eigen::Matrix<double, 6, 1> pos;
    pos << position_curr_[0],position_curr_[1],position_curr_[2],euler[0],euler[1],euler[2];
    Eigen::Matrix<double, 6, 1> pos_diff(desired_pos-pos);
    Eigen::Matrix<double, 6, 1> delta_dq(jacobian*dq);
    Eigen::Matrix<double, 6, 1> F = kp*pos_diff-kd*delta_dq;
    Eigen::Matrix<double, 7, 1> TF = jacobian.transpose()*F;

    // orientation
  
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::VectorXd tau_d(7);
    if (pos_diff.norm()< tol) {
        tau_d << coriolis-20*dq;
    }
    else {
        tau_d << TF+coriolis-dq/2;

        
    }

    // tau_d << coriolis-20*dq;
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position_d_(transform.translation());
    Eigen::Quaterniond orientation_d_(Eigen::Quaterniond(transform.linear()));

    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.x = orientation_d_.x();
    pose.pose.orientation.y = orientation_d_.y();
    pose.pose.orientation.z = orientation_d_.z();
    pose.pose.orientation.w = orientation_d_.w();
    pose.pose.position.x = position_d_[0];
    pose.pose.position.y = position_d_[1];
    pose.pose.position.z = position_d_[2];
    pospub.publish(pose);

    std_msgs::Float64MultiArray tau; 
    for (size_t i = 0; i < 7; ++i) {
         tau.data.push_back(tau_d[i]);
     }
    torquepub.publish(tau);

     for (size_t i = 0; i < 7; ++i) {
         joint_handles_[i].setCommand(tau_d[i]);
     }

        


    //sending a fix command of message 
    // double tau_d_saturated[] {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    // for (size_t i = 0; i < 7; ++i) {
    //     joint_handles_[i].setCommand(tau_d_saturated[i]);
    // }
    // updating time 
    // elapsed_time_ += period;

    //calculating torque to send command 
    // double torque = (std::cos( elapsed_time_.toSec())) * 10;
    //sending the command to each joint by joint_handles_ 
    // double tau_d_saturated[] {torque, torque, torque, torque, torque, torque, torque};
    // for (size_t i = 0; i < 7; ++i) {
    //     joint_handles_[i].setCommand(tau_d_saturated[i]);
    // }

    

    }
    Eigen::Matrix<double, 7, 1> FeedbackController::saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}

}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FeedbackController,
                       controller_interface::ControllerBase)