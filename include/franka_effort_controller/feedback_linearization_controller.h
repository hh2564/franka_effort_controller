#pragma once

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <franka_effort_controller/compliance_paramConfig.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <Eigen/Geometry> 

namespace franka_effort_controller {
class FeedbackLinearizationController : public controller_interface::MultiInterfaceController<
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


    ros::Duration elapsed_time_;
    double coriolis_factor_{1.0};
    std::array<double, 7> dq_filtered_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    franka::RobotState cur_state;
    std::array<double, 16> robot_pose_;
    const double delta_tau_max_{1.0};
    const double tol{5e-3};
    ros::Publisher pospub;
    ros::Publisher torquepub;
    ros::Publisher goalpub;
    double T; 
    ros::Time beginTime; 
    ros::Duration MessageTime;
    ros::Time endTime;
    std::string controlled_frame;
    Eigen::Matrix<double, 6, 6> A{};
    Eigen::Matrix<double, 6, 6> Ainv{};
    Eigen::Matrix<double, 6, 1> Bx{};
    Eigen::Matrix<double, 6, 1> xx{};
    Eigen::Matrix<double, 6, 1> By{};
    Eigen::Matrix<double, 6, 1> xy{};
    Eigen::Matrix<double, 6, 1> Bz{};
    Eigen::Matrix<double, 6, 1> xz{};
    Eigen::Matrix<double, 6, 1> Br{};
    Eigen::Matrix<double, 6, 1> xr{};
    Eigen::Matrix<double, 6, 1> Bp{};
    Eigen::Matrix<double, 6, 1> xp{};
    Eigen::Matrix<double, 6, 1> Bya{};
    Eigen::Matrix<double, 6, 1> xya{};
    double xd{0.5};
    double yd{0.5};
    double zd{0.5};
    double rd{1.5707};
    double pd{0}; 
    double yad{0.707}; 
    pinocchio::Model model;
    pinocchio::Data data;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    std::mutex position_and_orientation_d_target_mutex_;

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    std::unique_ptr<dynamic_reconfigure::Server<panda_controller::compliance_paramConfig>>
        dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(panda_controller::compliance_paramConfig& config,
                                uint32_t level);

    ros::Subscriber sub_equilibrium_pose_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    

    // pinocchio::Model model;
    

};

}
