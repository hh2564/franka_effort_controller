#include <franka_effort_controller/feedback_linearization_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka/robot_state.h>

namespace franka_effort_controller {
    bool FeedbackLinearizationController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    //checking to see if the default parameters can be access through node handle
    //and also getting information and interfaces  
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FeedbackLinearizationController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "FeedbackLinearizationController: Invalid or no joint_names parameters provided, aborting "
            "controller init!");
        return false;
    }

    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("FeedbackLinearizationController: publish_rate not found. Defaulting to "
                        << publish_rate);
    }
   
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedbackLinearizationController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }

    ros::NodeHandle nh;

    pospub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose", 1000);
    torquepub = nh.advertise<std_msgs::Float64MultiArray>("/tau_command", 1000);
    goalpub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1000);

    dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

    dynamic_server_compliance_param_ = std::make_unique<
        dynamic_reconfigure::Server<panda_controller::compliance_paramConfig>>(
        dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&FeedbackLinearizationController::complianceParamCallback, this, _1, _2));

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    sub_equilibrium_pose_ = node_handle.subscribe(
      "/goal_pose", 20, &FeedbackLinearizationController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

    //need to change urdf_filename to the path of urdf file in franaka_effort_controller/urdf while using 
    std::string urdf_filename = "/home/crrl/pin_ws/src/franka_effort_controller/urdf/model.urdf";
    
    //Importing model and data for pinocchio 
    pinocchio::urdf::buildModel(urdf_filename,model);
    data = pinocchio::Data(model);
    controlled_frame = model.frames[model.nframes-1].name;
    
    
    return true;

    }

    void FeedbackLinearizationController::starting(const ros::Time& /* time */) {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
     //getting the initial position and orientation of the ee 
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init_(initial_transform.translation());
    Eigen::Quaterniond orientation_init_(Eigen::Quaterniond(initial_transform.linear()));
    //converting from quaternion to euler angle 
    auto euler = orientation_init_.toRotationMatrix().eulerAngles(0, 1, 2);

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
   
    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;

    //getting the intial time to generate command in update 
    beginTime = ros::Time::now(); 
    MessageTime = ros::Duration(5.0);
    endTime = beginTime + MessageTime;
    //solves for the constants x_0-6 for the six degree polynomial 
    //T is the end time for the feedforward control 
    T = MessageTime.toSec(); 
    // A is a 6*6 matrix consists of f(0);f'(0);f''(0);f(T);f'(T);f''(T)
    A << 1,0,0,0,0,0,
         0,1,0,0,0,0, 
         0,0,2,0,0,0,
         1,T,pow(T,2),pow(T,3), pow(T,4),pow(T,5),
         0,1,2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
         0,0,2,6*T, 12*pow(T,2),20*pow(T,3); 
    // calculating A^-1 
    Ainv << A.inverse(); 
    // B_<var> is the vector representation of the initial and end boundary conditions of varable var 
    // <var>d is the desired end position of var in each direction/orientation 
    Bx <<  position_init_[0],0,0,xd,0,0;
    // x<var> is a vector consists of constants we need to solve for on direction <var> 
    // A*x<var> = B<var> ====> x<var> = A^-1*B<var>
    xx << Ainv*Bx; 
    By <<  position_init_[1],0,0,yd,0,0;
    xy << Ainv*By;
    Bz <<  position_init_[2],0,0,zd,0,0;
    xz << Ainv*Bz; 
    Br <<  euler[0],0,0,euler[0],0,0;
    xr << Ainv*Br; 
    Bp <<  euler[1],0,0,euler[1],0,0;
    xp << Ainv*Bp; 
    Bya <<  euler[2],0,0,euler[2],0,0;
    xya << Ainv*Bya; 





    

    
    }


    void FeedbackLinearizationController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
    //calculating for the time variable t to use in polynomial 
    ros::Time curTime = ros::Time::now(); 
    ros::Duration passedTime = curTime - beginTime;
    double t = passedTime.toSec(); 
     //calculating time vector t_pos, t_vel, t_acc, that could use to calculate the position, velocity, and acceleration of the ee at current time t
    // by multiplying time vector and the constant vector x<var> that we found previously 
    Eigen::Matrix<double, 1, 6> tpos {};
    tpos << 1,t,pow(t,2),pow(t,3), pow(t,4),pow(t,5); 
    Eigen::Matrix<double, 1, 6> tvel {};
    tvel << 0,1,2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4); 
    Eigen::Matrix<double, 1, 6> tacc {};
    tacc << 0,0,2,6*t, 12*pow(t,2),20*pow(t,3);


    //calculating the position, velocity, acceleration in direction <var> for the ee
    double x = tpos*xx;
    double dx = tvel*xx; 
    double ddx = tacc*xx; 
    double y = tpos*xy;
    double dy = tvel*xy; 
    double ddy = tacc*xy; 
    double z = tpos*xz;
    double dz = tvel*xz; 
    double ddz = tacc*xz;
    double r = tpos*xr;
    double dr = tvel*xr; 
    double ddr = tacc*xr;
    double p = tpos*xp;
    double dp = tvel*xp;
    double ddp = tacc*xp;
    double ya = tpos*xya;
    double dya = tvel*xya; 
    double ddya = tacc*xya;

    //concentrating postion, velocity, acceleration varables into vectors respectively 
    Eigen::Matrix <double,6,1> X{}; 
    X << x,y,z,r,p,ya;
    Eigen::Matrix <double,6,1> dX{};
    dX <<dx,dy,dz,dr,dp,dya;
    Eigen::Matrix <double,6,1> ddX{};   
    ddX <<ddx,ddy,ddz,ddr,ddp,ddya;

    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Vector3d position_curr_{};
    position_curr_ << x,y,z; 
    double cy = cos(ya * 0.5);
    double sy = sin(ya * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);
    Eigen::Quaterniond Quaternion;

    Quaternion.w() = cr * cp * cy + sr * sp * sy;
    Quaternion.x() = sr * cp * cy - cr * sp * sy;
    Quaternion.y() = cr * sp * cy + sr * cp * sy;
    Quaternion.z() = cr * cp * sy - sr * sp * cy;

    //for the ee_pose publisher
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(Eigen::Quaterniond(transform.linear()));
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];
    pose.header.stamp = ros::Time::now(); 
    pospub.publish(pose);

    //for the goal_pose publisher
    geometry_msgs::PoseStamped goalpose;
    if (passedTime.toSec()< MessageTime.toSec()) {
        goalpose.pose.orientation.x = Quaternion.x();
        goalpose.pose.orientation.y = Quaternion.y();
        goalpose.pose.orientation.z = Quaternion.z();
        goalpose.pose.orientation.w = Quaternion.w();
        goalpose.pose.position.x = x;
        goalpose.pose.position.y = y;
        goalpose.pose.position.z = z;
        goalpose.header.stamp = ros::Time::now(); 
    }
    else {
        goalpose.pose.orientation.x = Quaternion.x();
        goalpose.pose.orientation.y = Quaternion.y();
        goalpose.pose.orientation.z = Quaternion.z();
        goalpose.pose.orientation.w = Quaternion.w();
        goalpose.pose.position.x = 0.5;
        goalpose.pose.position.y = 0.5;
        goalpose.pose.position.z = 0.5;
        goalpose.header.stamp = ros::Time::now();      
    }

    goalpub.publish(goalpose);

    std::array<double, 42> jacobian_array =
    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
     std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;
    
    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);


//     //getting jacobian and pseudo jacobian at this time instance
    Eigen::MatrixXd jacobian_pinv;
    franka_example_controllers::pseudoInverse(jacobian, jacobian_pinv);



//     // updating model data for pinocchio and calculating jacobian dot 
    pinocchio::forwardKinematics(model,data,q,dq,0.0*dq); 
    pinocchio::updateFramePlacements(model,data);
    Eigen::Matrix<double, 6, 7> dJ(pinocchio::computeJointJacobiansTimeVariation(model,data,q,dq));



    std::array<double, 49> mass_array = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data()); 

    

    Eigen::VectorXd deltaq(7), deltadq(7), xdot(6), ddotqdes(7);
    xdot << jacobian*dq; 
    deltaq << jacobian_pinv*error; 
    deltadq << jacobian_pinv*(xdot - dX); 
    ddotqdes << jacobian_pinv*(ddX-dJ*dq); 


    // compute control
    // allocate variables
    Eigen::VectorXd task(7), nullspace(7),tau_d(7);

    Eigen::MatrixXd jacobian_transpose_pinv;
    franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    task << jacobian_pinv *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (xdot - dX));
  // nullspace PD control with damping ratio = 1
    nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                        jacobian.transpose() * jacobian_transpose_pinv) *
                        (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                            (2.0 * sqrt(nullspace_stiffness_)) * dq);

    if (passedTime.toSec()< MessageTime.toSec()) {
        tau_d << mass*(ddotqdes+nullspace+task)+coriolis; 
    }
    else {
        tau_d << coriolis-10*dq;
  
    }
    
 
    //for the tau_command publisher 
    std_msgs::Float64MultiArray tau; 
    for (size_t i = 0; i < 7; ++i) {
         tau.data.push_back(tau_d[i]);
     }
    torquepub.publish(tau);

    for (size_t i = 0; i < 7; ++i) {
         joint_handles_[i].setCommand(tau_d[i]);
     }

    cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
    






    

    }
    Eigen::Matrix<double, 7, 1> FeedbackLinearizationController::saturateTorqueRate(
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

    void FeedbackLinearizationController::complianceParamCallback(
        panda_controller::compliance_paramConfig& config,
        uint32_t /*level*/) {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << config.translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << config.rotational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
    nullspace_stiffness_target_ = config.nullspace_stiffness;
    }

    void FeedbackLinearizationController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
    }

}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FeedbackLinearizationController,
                       controller_interface::ControllerBase)