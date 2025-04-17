#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include </usr/include/eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "sim2real_msg/ControlState.h"
#include "sim2real_msg/MtrState.h"
#include "sim2real_msg/RbtState.h"
#include "sim2real_msg/MtrTarget.h"
#include "sim2real_msg/RbtTarget.h"
#include "sim2real_msg/Orient.h"
#include "sim2real_msg/PD2RL.h"
#include "sim2real_msg/RL2PD.h"
#include "hightorque_hardware_sdk/usr2ctrl.h"
#include "hightorque_hardware_sdk/ctrl2usr.h"
#include <deque>
struct Command{
    double vx = 0.0;
    double vy = 0.0;
    double dyaw = 0.0;
};

enum Init_State{
    I_STATE_SAMPLE = 2002,
    I_STATE_RUNNING,
};

enum Ctrl_State{
    C_STATE_WAITING = 206,
    C_STATE_INIT,
    C_STATE_RUNNING,
    C_STATE_STANDBY,
    C_STATE_SHUT_DOWN,
    C_STATE_FORCE_SHUT_DOWN,
    C_DEVELOP_CONTROL_JOINT,
    C_DEVELOP_CONTROL_MOTOR,
    C_STATE_TRANSITION
};

enum Shut_Down_State{
    D_STATE_STAND = 608,
    D_STATE_CALC,
    D_STATE_SQUAT,
    D_STATE_OFF,
};

struct Motor_State{
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd tau;
};

struct Robot_State{
    Eigen::VectorXd eu_ang;
    Eigen::VectorXd base_ang_vel;
    Eigen::Quaterniond quat;
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd tau;
};

struct Motor_Output{
    Eigen::VectorXd target_q;
    Eigen::VectorXd target_dq;
    Eigen::VectorXd target_tau;
};

struct Robot_Output{
    Eigen::VectorXd target_q;
    Eigen::VectorXd target_dq;
    Eigen::VectorXd target_tau;
    Eigen::VectorXd action;
};

struct Transform{
    Eigen::Matrix<double, 2, 2> knee_J;
    Eigen::Matrix<double, 2, 2> ankle_J_left;
    Eigen::Matrix<double, 2, 2> ankle_J_right;
    Eigen::Matrix<double, 2, 2> ankle_J_left_inv;
    Eigen::Matrix<double, 2, 2> ankle_J_right_inv;
};

struct Observations{
    Eigen::VectorXd observations;
    Eigen::MatrixXd input;
    std::deque<Eigen::VectorXd> hist_obs;

};

struct DynamicOffset{
    double hip;
    double knee;
    double ankle;
};

struct RL_Subscriber{
    ros::Subscriber pd2rl_sub_;
    ros::Subscriber joy_model_sub_;
    ros::Subscriber joy_control_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber usr_ctrl_sub_;
};

struct RL_Publisher{
    ros::Publisher rl2pd_pub_;
    ros::Publisher ori_pub_;
    ros::Publisher ctrl_usr_pub_;
};

struct RL_Msgs{
    sensor_msgs::Imu yesenseIMU;
    sim2real_msg::Orient ori_msg;
    sim2real_msg::PD2RL pd2rl_msg;
    sim2real_msg::RL2PD rl2pd_msg;
    hightorque_hardware_sdk::ctrl2usr ctrl_usr_msg;
};

struct PD_Subscriber{
    ros::Subscriber rl2pd_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ori_sub_;
};

struct PD_Publisher{
    ros::Publisher pd2rl_pub_;
    ros::Publisher rviz_pos_pub_;
    ros::Publisher mtr_state_pub_;
    ros::Publisher rbt_state_pub_;
    ros::Publisher mtr_target_pub_;
    ros::Publisher rbt_target_pub_;
};

struct PD_Msgs{
    sensor_msgs::Imu yesenseIMU;
    sim2real_msg::Orient ori_msg;
    sensor_msgs::JointState rviz_pos_msg;
    sim2real_msg::PD2RL pd2rl_msg;
    sim2real_msg::RL2PD rl2pd_msg;
    sim2real_msg::MtrState mtr_state_msg;
    sim2real_msg::RbtState rbt_state_msg;
    sim2real_msg::MtrTarget mtr_target_msg;
    sim2real_msg::RbtTarget rbt_target_msg;
};

#endif