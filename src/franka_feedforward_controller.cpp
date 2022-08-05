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
    goalpub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1000);

    //need to change urdf_filename to the path of urdf file in franaka_effort_controller/urdf while using 
    std::string urdf_filename = "/home/ubuntu/catkin_ws/src/franka_effort_controller/urdf/model.urdf";

    //Importing model and data for pinocchio 
    pinocchio::urdf::buildModel(urdf_filename,model);
    data = pinocchio::Data(model);
    controlled_frame = model.frames[model.nframes-1].name;


    return true;

    }

    void FeedforwardController::starting(const ros::Time& /* time */) {
    //getting the intial time to generate command in update 
    beginTime = ros::Time::now(); 
    MessageTime = ros::Duration(5.0);
    endTime = beginTime + MessageTime;
    franka::RobotState initial_state = state_handle_->getRobotState();

    //getting the initial position and orientation of the ee 
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init_(initial_transform.translation());
    Eigen::Quaterniond orientation_init_(Eigen::Quaterniond(initial_transform.linear()));
    //converting from quaternion to euler angle 
    auto euler = orientation_init_.toRotationMatrix().eulerAngles(0, 1, 2);

    
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
    Eigen::Affine3d curr_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));




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

    
    const int JOINT_ID = 7;
    const pinocchio::SE3 oMdes(Quaternion, position_curr_);
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    const double eps  = 1e-4;
    const int IT_MAX  = 1000;
    const double DT   = 1e-1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J(6,model.nv);
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);

    for (int i=0;;i++){
     pinocchio::forwardKinematics(model,data,q);
     const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
     err = pinocchio::log6(dMi).toVector();
     if(err.norm() < eps)
     {
       success = true;
       break;
     }
     if (i >= IT_MAX)
     {
       success = false;
       break;
     }
    }

    //getting jacobian and pseudo jacobian at this time instance
    pinocchio::getFrameJacobian(model, data, model.getFrameId(controlled_frame), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model,q,v*DT);
    Eigen::MatrixXd jacobian_pinv;
    franka_example_controllers::pseudoInverse(J, jacobian_pinv);
    //getting current joint position and velocity 
    //dX = Jdq =====> J^{+}dX  = dq 
    Eigen::Matrix<double, 7, 1> dq(jacobian_pinv*dX); 

    // updating model data for pinocchio and calculating jacobian dot 
    pinocchio::forwardKinematics(model,data,q,dq,0.0*dq); 
    pinocchio::updateFramePlacements(model,data);
    pinocchio::computeMinverse(model,data,q);
    data.Minv.triangularView<Eigen::StrictlyLower>() = data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Eigen::Matrix<double, 6, 7> dJ(pinocchio::computeJointJacobiansTimeVariation(model,data,q,dq));
    Eigen::Matrix<double, 7, 7> coriolism(pinocchio::computeCoriolisMatrix(model,data,q,dq));
    Eigen::Matrix<double, 7, 1> coriolis(coriolism*dq);
    std::array<double, 49> mass_array = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data()); 

    Eigen::VectorXd tau_d(7);

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

    //for the goal_pose publisher
    geometry_msgs::PoseStamped goalpose;
    goalpose.pose.orientation.x = Quaternion.x();
    goalpose.pose.orientation.y = Quaternion.y();
    goalpose.pose.orientation.z = Quaternion.z();
    goalpose.pose.orientation.w = Quaternion.w();
    goalpose.pose.position.x = x;
    goalpose.pose.position.y = y;
    goalpose.pose.position.z = z;
    goalpose.header.stamp = ros::Time::now(); 
    goalpub.publish(goalpose);

    if (passedTime.toSec()< MessageTime.toSec()) {
        //ddX = Jdq+Jddq ====> ddX-Jdq = Jddq =====> J^{+} (ddX-Jdq) = ddq ========>MJ^{+}(ddX-Jdq) = Mddq = TF 
        Eigen::Matrix<double, 7, 1>  TF(mass*jacobian_pinv*(ddX-dJ*dq)); 
        tau_d << coriolis+TF;
    }
    else {
        tau_d << coriolis;        
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
    
    }

    




}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FeedforwardController,
                       controller_interface::ControllerBase)