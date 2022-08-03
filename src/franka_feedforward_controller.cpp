#include <franka_effort_controller/franka_feedforward_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64MultiArray.h"


namespace franka_effort_controller {
    bool FeedforwardController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FeedforwardController: Could not read parameter arm_id");
        return false;
    }
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "FeedforwardController: Invalid or no joint_names parameters provided, aborting "
            "controller init!");
        return false;
    }

    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
        ROS_INFO_STREAM("FeedforwardController: publish_rate not found. Defaulting to "
                        << publish_rate);
    }
   
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FeedforwardController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }
    ros::NodeHandle nh;

    pospub = nh.advertise<geometry_msgs::PoseStamped>("/ee_pose", 1000);
    torquepub = nh.advertise<std_msgs::Float64MultiArray>("/tau_command", 1000);

    //need to change urdf_filename to the path of urdf file in franaka_effort_controller/urdf while using 
    std::string urdf_filename = "/home/crrl/pin_ws/src/franka_effort_controller/urdf/model.urdf";

    //Importing model and data for pinocchio 
    pinocchio::urdf::buildModel(urdf_filename,model);
    data = pinocchio::Data(model);


    return true;

    }

    void FeedforwardController::starting(const ros::Time& /* time */) {
    //getting the intial time to generate command in update 
    beginTime = ros::Time::now(); 
    MessageTime = ros::Duration(10.0);
    endTime = beginTime + MessageTime;
    franka::RobotState initial_state = state_handle_->getRobotState();

    //getting the initial position and orientation of the ee 
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init_(initial_transform.translation());
    Eigen::Quaterniond orientation_init_(Eigen::Quaterniond(initial_transform.linear()));
    //converting from quaternion to euler angle 
    auto euler = orientation_init_.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

    
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

    void FeedforwardController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
    franka::RobotState robot_state = state_handle_->getRobotState();
    //getting jacobian and pseudo jacobian at this time instance                                             
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_pinv;
    franka_example_controllers::pseudoInverse(jacobian, jacobian_pinv);

    //calculating for the time variable t to use in polynomial 
    ros::Time curTime = ros::Time::now(); 
    ros::Duration passedTime = curTime - beginTime;
    double t = passedTime.toSec(); 

    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); 

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

    //getting current joint position and velocity 
    //dX = Jdq =====> J^{+}dX  = dq 
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Matrix<double, 7, 1> dq(robot_state.dq.data()); 

    // updating model data for pinocchio and calculating jacobian dot 
    pinocchio::forwardKinematics(model,data,q,dq,0.0*dq); 
    pinocchio::updateFramePlacements(model,data);
    Eigen::Matrix<double, 6, 7> dJ(pinocchio::computeJointJacobiansTimeVariation(model,data,q,dq));

    Eigen::VectorXd tau_d(7);

    if (passedTime.toSec()< MessageTime.toSec()) {

        //gettign the mass matrix 
        Eigen::Matrix<double, 7, 1> dq_cal(jacobian_pinv*dX); 
        std::array<double, 49> mass_array = model_handle_->getMass();
        Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data()); 
        //ddX = Jdq+Jddq ====> ddX-Jdq = Jddq =====> J^{+} (ddX-Jdq) = ddq ========>MJ^{+}(ddX-Jdq) = Mddq = TF 
        Eigen::Matrix<double, 7, 1>  TF(mass*jacobian_pinv*(ddX-dJ*dq)); 

        tau_d << coriolis+TF;
    }
    else {
        tau_d << coriolis-10*dq;        
    }

    //for the ee_pose publisher
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
    pose.header.stamp = ros::Time::now(); 
    pospub.publish(pose);


    //for the tau_command publisher 
    std_msgs::Float64MultiArray tau; 
    for (size_t i = 0; i < 7; ++i) {
         tau.data.push_back(tau_d[i]);
     }
    torquepub.publish(tau);

    for (size_t i = 0; i < 7; ++i) {
         joint_handles_[i].setCommand(tau_d[i]);
     }
    
    }




}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FeedforwardController,
                       controller_interface::ControllerBase)