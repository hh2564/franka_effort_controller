#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_effort_controller {
class JointImpedanceController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private: 
    // std::array<double, 7> saturateTorqueRate(
    //     const std::array<double, 7>& tau_d_calculated,
    //     const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // static constexpr double kDeltaTauMax{1.0};
    // double radius_{0.1};
    // double acceleration_time_{2.0};
    // double vel_max_{0.05};
    // double angle_{0.0};
    // double vel_current_{0.0};

    // std::vector<double> k_gains_;
    // std::vector<double> d_gains_;
    // double coriolis_factor_{1.0};
    // std::array<double, 7> dq_filtered_;
    ros::Duration elapsed_time_;
    std::array<double, 7> initial_pose_{};


};

}
