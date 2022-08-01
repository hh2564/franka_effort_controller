#include <franka_effort_controller/franka_feedforward_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <geometry_msgs/PoseStamped.h>


namespace franka_effort_controller {
    bool FeedforwardController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FeedforwardController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
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

    return true;

    }

    void FeedforwardController::starting(const ros::Time& /* time */) {
    //getting the intial time to generate command in update 
    beginTime = ros::Time::now(); 
    MessageTime = ros::Duration(10.0);
    endTime = beginTime + MessageTime;
    franka::RobotState initial_state = state_handle_->getRobotState();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init_(initial_transform.translation());
    Eigen::Quaterniond orientation_init_(Eigen::Quaterniond(initial_transform.linear()));
    auto euler = orientation_init_.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;

    

    T = MessageTime.toSec(); 
    A << 1,0,0,0,0,0,
         0,1,0,0,0,0, 
         0,0,2,0,0,0,
         1,T,pow(T,2),pow(T,3), pow(T,4),pow(T,5),
         0,1,2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
         0,0,2,6*T, 12*pow(T,2),20*pow(T,3); 
    Ainv << A.inverse(); 
    Bx <<  position_init_[0],0,0,xd,0,0;
    xx << Ainv*Bx; 
    By <<  position_init_[1],0,0,yd,0,0;
    xy << Ainv*By;
    Bz <<  position_init_[2],0,0,zd,0,0;
    xz << Ainv*Bz; 
    Br <<  euler[0],0,0,rd,0,0;
    xr << Ainv*Br; 
    Bp <<  euler[1],0,0,pd,0,0;
    xp << Ainv*Bp; 
    Bya <<  euler[2],0,0,yad,0,0;
    xya << Ainv*Bya; 
    

    // initial_q << 0.0,-0.785398,0.0,-2.356194,0.0,1.570796,0.785398;
    // goal_q << 0.249814, -0.403797, 0.811459, -2.01424, 0.302385, 2.02715, 0.762035;  

    }

    void FeedforwardController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
    std::array<double, 42> jacobian_array =
    model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Matrix<double, 7, 6> pseudo_jacobian(jacobian.transpose()*(jacobian*jacobian.transpose()).inverse()); 
    
    std::array<double, 7> qarray  = {0.0,-0.785398,0.0,-2.356194,0.0,1.570796,0.785398};
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_q(qarray.data());
    std::array<double, 7> garray  = {0.327733, 0.340527, 0.553659, -1.37565, -0.141557, 1.98037, 0.671678};
    Eigen::Map<Eigen::Matrix<double, 7, 1>> goal_q(garray.data());
    ros::Time curTime = ros::Time::now(); 
    ros::Duration passedTime = curTime - beginTime;
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data()); 
    Eigen::VectorXd tau_d(7);
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Matrix<double, 7, 1> diff(goal_q-initial_q);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position_d_(transform.translation());
    Eigen::Quaterniond orientation_d_(Eigen::Quaterniond(transform.linear()));
    double t = passedTime.toSec(); 
    Eigen::Matrix<double, 1, 6> tpos {};
    tpos << 1,t,pow(t,2),pow(t,3), pow(t,4),pow(t,5); 
    Eigen::Matrix<double, 1, 6> tvel {};
    tvel << 0,1,2*t, 3*pow(t,2), 4*pow(t,3), 5*pow(t,4); 
    Eigen::Matrix<double, 1, 6> tacc {};
    tacc << 0,0,2,6*t, 12*pow(t,2),20*pow(t,3);
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



    if (passedTime.toSec()< MessageTime.toSec()) {
        Eigen::Matrix <double,6,1> X{}; 
        X << x,y,z,r,p,ya;
        Eigen::Matrix <double,6,1> dX{};
        dX <<dx,dy,dz,dr,dp,dya;
        Eigen::Matrix <double,6,1> ddX{};   
        ddX <<ddx,ddy,ddz,ddr,ddp,ddya;


        // Eigen::Matrix<double, 7, 1> q (initial_q + (3*pow(passedTime.toSec(),2)/pow(MessageTime.toSec(),2)-2*pow(passedTime.toSec(),3)/pow(MessageTime.toSec(),3))*diff);
        // Eigen::Matrix<double, 7, 1> dq ((6*passedTime.toSec()/pow(MessageTime.toSec(),2)-6*pow(passedTime.toSec(),2)/pow(MessageTime.toSec(),3))*diff);
        // Eigen::Matrix<double, 7, 1> ddq ((6/pow(MessageTime.toSec(),2)-12*passedTime.toSec()/pow(MessageTime.toSec(),3))*diff);
        std::array<double, 49> mass_array = model_handle_->getMass();
        Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data()); 
        // Eigen::Matrix<double, 7, 1>  TF(mass*ddq); 

        // tau_d << coriolis+TF;
    }
    else {
        Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        tau_d << coriolis-10*dq;        
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.x = orientation_d_.x();
    pose.pose.orientation.y = orientation_d_.y();
    pose.pose.orientation.z = orientation_d_.z();
    pose.pose.orientation.w = orientation_d_.w();
    pose.pose.position.x = position_d_[0];
    pose.pose.position.y = position_d_[1];
    pose.pose.position.z = position_d_[2];
    pospub.publish(pose);

    for (size_t i = 0; i < 7; ++i) {
         joint_handles_[i].setCommand(tau_d[i]);
     }
    
    }




}

PLUGINLIB_EXPORT_CLASS(franka_effort_controller::FeedforwardController,
                       controller_interface::ControllerBase)

