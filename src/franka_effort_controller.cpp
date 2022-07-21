#include <franka_effort_controller/franka_effort_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_effort_controller {
    bool JointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    //checking to see if the default parameters can be access through node handle 
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

    return true;

    }


    void JointImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
    //sending a fix command of message 
    // double tau_d_saturated[] {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    // for (size_t i = 0; i < 7; ++i) {
    //     joint_handles_[i].setCommand(tau_d_saturated[i]);
    // }
    double tau_d_saturated[] {10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d_saturated[i]);
    }

    }

}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::JointImpedanceController,
                       controller_interface::ControllerBase)