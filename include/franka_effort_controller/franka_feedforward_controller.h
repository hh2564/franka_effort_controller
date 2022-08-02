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
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <Eigen/Dense>

// // Pinocchio
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/aba.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/crba.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/multibody/model.hpp"
// #include "pinocchio/algorithm/model.hpp"


namespace franka_effort_controller {
class FeedforwardController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface,
                                        franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private: 

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::vector<std::string> joint_names;

    static constexpr double kDeltaTauMax{1.0};
    double radius_{0.1};
    double acceleration_time_{2.0};
    double vel_max_{0.05};
    double angle_{0.0};
    double vel_current_{0.0};
    double xd{0.5};
    double yd{0.5};
    double zd{0.5};
    double rd{1.5707};
    double pd{0}; 
    double yad{0.707}; 

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
    ros::Time beginTime; 
    ros::Duration MessageTime;
    ros::Time endTime;
    ros::Publisher pospub;
    ros::Publisher torquepub;
    double T; 
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
    pinocchio::Model model;
    pinocchio::Data data;


    
    

};

}
