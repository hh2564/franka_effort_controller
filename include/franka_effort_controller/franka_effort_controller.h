#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <Eigen/Dense>

namespace franka_effort_controller {
class JointImpedanceController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface,
                                        franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private: 
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    static constexpr double kDeltaTauMax{1.0};
    double radius_{0.1};
    double acceleration_time_{2.0};
    double vel_max_{0.05};
    double angle_{0.0};
    double vel_current_{0.0};

    std::vector<double> k_gains_;
    std::vector<double> d_gains_;
    double coriolis_factor_{1.0};
    std::array<double, 7> dq_filtered_;
    ros::Duration elapsed_time_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    franka::RobotState cur_state;
    std::array<double, 16> robot_pose_;
    const double delta_tau_max_{1.0};
    const double tol{5e-3};
    ros::Publisher pospub;
    

};

}
