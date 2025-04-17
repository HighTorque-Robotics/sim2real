#include "pd_controller.h"
#include <ros/ros.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <signal.h>
//#include "../include/hardware/robot.h"
#include "hardware/robot.h"
#include "pinocchio/algorithm/kinematics.hpp"
#include "robot_data.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/time.h"
#include <ros/package.h>

#include <fstream>
#include <filesystem>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "livelybot_logger/logger_interface.h"

// protect kill
namespace hightorque{

PD::PD() : nh()
{
    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("resources_dir", resources_dir_, "");
    init_data();
}

PD::PD(std::shared_ptr<livelybot_serial::robot> rb_ptr,const std::string&  config_path): nh(),
    current_joint_index_(0)

{
    rb_ptr_=rb_ptr;
    info=PD_YamlInfo(config_path);
    init_data();
    // rb_ptr_->set_motor_runzero();
    // ros::Duration(5).sleep();
}


PD::PD(std::shared_ptr<livelybot_serial::robot> rb_ptr): nh(),
    current_joint_index_(0)
{
    rb_ptr_=rb_ptr;
    init_data();
    // rb_ptr_->set_motor_runzero();
    // ros::Duration(5).sleep();
}

PD::~PD()
{
    ROS_INFO("PD Shut Down && Motor Torque Set to Protection");
    // for (int j = 0; j < 10; j++)
    //     send_motor_zero();
    ctrl_state = C_STATE_FORCE_SHUT_DOWN;
    msg.pd2rl_msg.ctrl_state = C_STATE_FORCE_SHUT_DOWN;
    pub.pd2rl_pub_.publish(msg.pd2rl_msg);
    quit.store(true);

    if (thread_exec_ptr_->joinable())
    {
        thread_exec_ptr_->join();
    }
    ROS_ERROR("PD destruction");
}

void PD::init_data()
{

    quit.store(true);
    init_state = I_STATE_SAMPLE;
    ctrl_state = C_STATE_WAITING;
    msg.pd2rl_msg.ctrl_state = C_STATE_WAITING;
    msg.pd2rl_msg.is_falldown = false;
    msg.pd2rl_msg.is_transition = false;
    shut_down_state = D_STATE_STAND;
    action_type = hightorque::ActionType::ACTION_NULL;
    action_state = hightorque::ActionState::COMPLETED;
    cnt_ = 0;

    mtr_state.q = Eigen::VectorXd::Zero(info.params.num_actions);
    mtr_state.dq = Eigen::VectorXd::Zero(info.params.num_actions);
    mtr_state.tau = Eigen::VectorXd::Zero(info.params.num_actions);

    rbt_state.eu_ang = Eigen::VectorXd::Zero(3);
    rbt_state.base_ang_vel = Eigen::VectorXd::Zero(3);
    rbt_state.q = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_state.dq = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_state.tau = Eigen::VectorXd::Zero(info.params.num_actions);

    mtr_output.target_q = Eigen::VectorXd::Zero(info.params.num_actions);
    mtr_output.target_dq = Eigen::VectorXd::Zero(info.params.num_actions);
    mtr_output.target_tau = Eigen::VectorXd::Zero(info.params.num_actions);

    rbt_output.target_q = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output.target_dq = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output.target_tau = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output.action = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output_last.target_q = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output_last.target_dq = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output_last.target_tau = Eigen::VectorXd::Zero(info.params.num_actions);
    rbt_output_last.action = Eigen::VectorXd::Zero(info.params.num_actions);
    dynamic_offset = {info.params.dynamic_offset_hip, info.params.dynamic_offset_knee, info.params.dynamic_offset_ankle};

    sub.rl2pd_sub_ = nh.subscribe<sim2real_msg::RL2PD>("rl2pd", 100, &PD::rl2pd_callback, this);
    pub.pd2rl_pub_ = nh.advertise<sim2real_msg::PD2RL>("pd2rl", 100);

    sub.joy_sub_ = nh.subscribe<sim2real_msg::ControlState>("/joy_msg", 10, &PD::joy_callback, this);
    sub.odom_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &PD::odom_callback, this);
    sub.ori_sub_ = nh.subscribe<sim2real_msg::Orient>("ori_state", 100, &PD::ori_callback, this);

    pub.mtr_state_pub_ = nh.advertise<sim2real_msg::MtrState>("mtr_state", 1000);
    pub.rbt_state_pub_ = nh.advertise<sim2real_msg::RbtState>("rbt_state", 1000);
    pub.mtr_target_pub_ = nh.advertise<sim2real_msg::MtrTarget>("mtr_target", 1000);
    pub.rbt_target_pub_ = nh.advertise<sim2real_msg::RbtTarget>("rbt_target", 1000);
    pub.rviz_pos_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
    msg.rviz_pos_msg.name = info.params.joint_controller_names;
    msg.rviz_pos_msg.position.resize(info.params.num_actions);

    //urdf_path = CMAKE_CURRENT_SOURCE_DIR + std::string("/../clpai_12dof_0905/urdf/clpai_12dof_0905.urdf");
    //mesh_path = CMAKE_CURRENT_SOURCE_DIR + std::string("/../clpai_12dof_0905/meshes");
    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("urdf_path", urdf_path, "");
    nh_param.param<std::string>("mesh_path", mesh_path, "");
    
    pinocchio::urdf::buildModel(urdf_path, rbt_model);
    rbt_data = pinocchio::Data(rbt_model);

    running_scale_=info.params.action_scale;
    int control_type;
    if (nh.getParam("/robot/control_type", control_type))
    {
        set_motor_command_type(control_type);
        ROS_INFO("control type:%d", control_type);
    }

    std::string text_printf = "";
    for (const auto& name : info.params.waypoint_files) {
        text_printf += name + " \r\n";
    }
    ROS_INFO("waypoint_files:\r\n%s", text_printf.c_str());
    
    thread_exec_ptr_.reset(new std::thread(&PD::exec_loop, this));
    quit.store(false);
}


void PD::set_motor_command_type(int type)
{
    switch (type)
    {
    case 9:
        motor_command_type_ = MotorCommandType::pd_param;
        break;
    case 3:
        motor_command_type_ = MotorCommandType::only_torque;
        break;
    default:
        motor_command_type_ = MotorCommandType::pd_param;
        break;
    }
}

void PD::set_ctrl_state(int idx)//206-211
{
    ctrl_state= static_cast<Ctrl_State>(idx);
}

void PD::set_joint_command( sensor_msgs::JointState  joint_command)
{
    for(int i=0;i<info.params.num_actions;++i)
    {
        rbt_output_last.target_q[i]=joint_command.position[i];
        rbt_output_last.target_dq[i]=joint_command.velocity[i];
        rbt_output_last.target_tau[i]=joint_command.effort[i];
    }
}

void PD::set_motor_command(sensor_msgs::JointState  motor_command)
{
    for(int i=0;i<info.params.num_actions;++i)
    {
        mtr_output.target_q[i]=motor_command.position[i];
        mtr_output.target_dq[i]=motor_command.velocity[i];
        mtr_output.target_tau[i]=motor_command.effort[i];
    }
}

void PD::rl2pd_callback(const sim2real_msg::RL2PD::ConstPtr &rl2pd_msg)
{
    for (int i = 0; i < info.params.num_actions; i++)
    {
        rbt_output.action[i] = rl2pd_msg->action[i];
    }
    if (rl2pd_msg->ctrl_state != ctrl_state && ctrl_state != C_STATE_TRANSITION)
    {
        if ((ctrl_state == C_STATE_INIT || ctrl_state == C_STATE_STANDBY) && rl2pd_msg->ctrl_state == C_STATE_SHUT_DOWN)
        {
            shut_down_state = D_STATE_CALC;
        }
        ctrl_state = static_cast<Ctrl_State>(rl2pd_msg->ctrl_state);
        msg.pd2rl_msg.ctrl_state = rl2pd_msg->ctrl_state;
        ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        if (ctrl_state == C_STATE_SHUT_DOWN)
            shut_down_time = ros::Time::now();
    }
}

bool PD::exec_loop_is_terminate()
{
    return quit.load();
}

void PD::joy_callback(const sim2real_msg::ControlState::ConstPtr &joy_msg)
{
    if (joy_msg->save_all_angle_left == 1 && joy_msg->save_all_angle_right == 1)
    {
        info.params.dynamic_offset_hip = dynamic_offset.hip;
        info.params.dynamic_offset_knee = dynamic_offset.knee;
        info.params.dynamic_offset_ankle = dynamic_offset.ankle;
        info.write_yaml();
    }

    if (joy_msg->candidate_right <= -0.5)
    {
        if(joy_msg->confirm == 1)
        {
            action_type = hightorque::ActionType::Action1;  // 执行动作 1
            action_state = ActionState::LOADFILE;
            ROS_INFO("Button A pressed, executing action 1");
        }
        else if (joy_msg->quit == 1)
        {
            action_type = hightorque::ActionType::Action2;  // 执行动作 2
            action_state = ActionState::LOADFILE;
            ROS_INFO("Button B pressed, executing action 2");
        }
        else if (joy_msg->hip_angle_increase == 1)
        {
            action_type = hightorque::ActionType::Action3;  // 执行动作 3
            action_state = ActionState::LOADFILE;
            ROS_INFO("Button X pressed, executing action 3");
        }
        else if (joy_msg->knee_angle_decrease == 1)
        {
            action_type = hightorque::ActionType::Action4;  // 执行动作 4
            action_state = ActionState::LOADFILE;
            ROS_INFO("Button Y pressed, executing action 4");
        }
    }
    else
    {
        if (joy_msg->hip_angle_increase == 1)
        {
            dynamic_offset.hip += 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
        if (joy_msg->hip_angle_decrease == 1)
        {
            dynamic_offset.hip -= 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
        if (joy_msg->knee_angle_increase == 1)
        {
            dynamic_offset.knee += 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
        if (joy_msg->knee_angle_decrease == 1)
        {
            dynamic_offset.knee -= 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
        if (joy_msg->ankle_angle_increase == 1)
        {
            dynamic_offset.ankle += 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
        if (joy_msg->ankle_angle_decrease == 1)
        {
            dynamic_offset.ankle -= 0.005;
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
        }
    }
}

void PD::odom_callback(const sensor_msgs::Imu::ConstPtr &odom)
{
    msg.yesenseIMU = *odom;
}

void PD::ori_callback(const sim2real_msg::Orient::ConstPtr& ori)
{
    msg.ori_msg = *ori;
    // ROS_INFO("ori:::roll:%f, pitch:%f, yaw:%f", msg.ori_msg.eu_ang[0], msg.ori_msg.eu_ang[1], msg.ori_msg.eu_ang[2]);
}

void PD::read_joint()
{
    if (ctrl_state == C_STATE_FORCE_SHUT_DOWN)
        return;

    float pos, vel, tau;
    for (int i = 0; i < info.params.num_actions; ++i)
    {
        //rb_ptr_->get_motor_state_dynamic_config(pos, vel, tau, info.params.map_index[0][i]);
        int map_index = info.params.map_index[0][i];
        
        motor_back_t motor_back_data = *rb_ptr_->Motors[map_index]->get_current_motor_state();
        pos = motor_back_data.position;
        vel = motor_back_data.velocity;
        tau = motor_back_data.torque;

        mtr_state.q[i] = pos;
        mtr_state.dq[i] = vel;
        mtr_state.tau[i] = tau;

        if (mtr_state.q[i] < info.params.motor_lower_limit[0][i] || mtr_state.q[i] > info.params.motor_upper_limit[0][i])
        {
            ROS_ERROR("motor:::%d, q=%f, limit!! shut down!!", i, mtr_state.q[i]);
            ROS_INFO("offset:::hip:%f, knee:%f, ankle:%f", dynamic_offset.hip, dynamic_offset.knee, dynamic_offset.ankle);
            ctrl_state = C_STATE_FORCE_SHUT_DOWN;
            msg.pd2rl_msg.ctrl_state = C_STATE_FORCE_SHUT_DOWN;
            pub.pd2rl_pub_.publish(msg.pd2rl_msg);
            quit.store(true);
            return;
        }
    }
    // kinematics solution
    mtr2rbt_map();

    msg.rviz_pos_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < info.params.num_actions; ++i)
    {
        msg.pd2rl_msg.rbt_state_q[i] = rbt_state.q[i];
        msg.pd2rl_msg.rbt_state_dq[i] = rbt_state.dq[i];

        msg.mtr_state_msg.q[i] = mtr_state.q[i];
        msg.mtr_state_msg.dq[i] = mtr_state.dq[i];
        msg.mtr_state_msg.tau[i] = mtr_state.tau[i];
        msg.rbt_state_msg.q[i] = rbt_state.q[i];
        msg.rbt_state_msg.dq[i] = rbt_state.dq[i];
        msg.rbt_state_msg.tau[i] = rbt_state.tau[i];

        msg.rviz_pos_msg.position[i] = rbt_state.q[i];

        if (ctrl_state == C_STATE_INIT || ctrl_state == C_STATE_TRANSITION)
        {
            if (init_state == I_STATE_SAMPLE)
            {
                rbt_q_init[i] = rbt_state.q[i];
            }
        }
    }
    pub.rviz_pos_pub_.publish(msg.rviz_pos_msg);
    pub.mtr_state_pub_.publish(msg.mtr_state_msg);
    pub.rbt_state_pub_.publish(msg.rbt_state_msg);
}

double ease_out_transition(double currentValue, double targetValue, double startTime, double currentTime, double duration)
{
    double t = (currentTime - startTime) / duration;
    t = (t < 0.0f) ? 0.0f : (t > 1.0f) ? 1.0f
                                       : t;
    return currentValue + (targetValue - currentValue) * (1 - (1 - t) * (1 - t));
}

void PD::write_joint()
{

    if (ctrl_state == C_STATE_FORCE_SHUT_DOWN)
        return;

    if (ctrl_state == C_STATE_SHUT_DOWN && shut_down_state == D_STATE_SQUAT)
    {
        pinocchio::SE3 target_transform_left(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.075, 0.08, shut_down_height));
        pinocchio::SE3 target_transform_right(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.075, -0.08, shut_down_height));

        Eigen::VectorXd rbt_tq_init = neutral(rbt_model);
        Eigen::VectorXd rbt_tq_ik_left = rbt_tq_init.eval();
        Eigen::VectorXd rbt_tq_ik_right = rbt_tq_init.eval();

        bool success_left = whole_body_inverse_kinematics(rbt_model, rbt_data, "l_ankle_roll_link", target_transform_left, rbt_tq_ik_left);
        bool success_right = whole_body_inverse_kinematics(rbt_model, rbt_data, "r_ankle_roll_link", target_transform_right, rbt_tq_ik_right);

        if (success_left == false || success_right == false)
        {
            ROS_ERROR("Inverse kinematics failed!");
            ctrl_state = C_STATE_FORCE_SHUT_DOWN;
            msg.pd2rl_msg.ctrl_state = C_STATE_FORCE_SHUT_DOWN;
            pub.pd2rl_pub_.publish(msg.pd2rl_msg);
            return;
        }

        Eigen::VectorXd rbt_tq_combined = rbt_tq_ik_left.eval();
        rbt_tq_combined.segment<6>(0) = rbt_tq_ik_left.segment<6>(0);
        rbt_tq_combined.segment<6>(6) = rbt_tq_ik_right.segment<6>(6);

        for (int i = 0; i < info.params.num_actions; i++)
        {
            rbt_output.target_q[i] = rbt_tq_combined[i];
        }
    }
    else
    {
        for (int i = 0; i < info.params.num_actions; i++)
        {
            rbt_output.target_q[i] = rbt_output.action[i] * info.params.action_scale;
        }
    }

    if (ctrl_state == C_STATE_INIT || ctrl_state == C_STATE_TRANSITION)
    {
        if (init_state == I_STATE_SAMPLE)
        {
            for (int i = 0; i < info.params.num_actions; i++)
            {
                rbt_tq_init[i] = rbt_output.target_q[i];
            }
            init_state = I_STATE_RUNNING;
            init_start_time = ros::Time::now();
        }
        for (int i = 0; i < info.params.num_actions; i++)
        {
            rbt_output.target_q[i] = ease_out_transition(rbt_q_init[i], rbt_tq_init[i], static_cast<double>(init_start_time.toSec()), static_cast<double>(ros::Time::now().toSec()), 5.0);
        }
        double elapsed_time = (ros::Time::now() - init_start_time).toSec();
        
        if (ctrl_state == C_STATE_TRANSITION)
        {
        if (elapsed_time >= 1.0) // 5.0 为过渡持续时间
        {
            ctrl_state = C_STATE_STANDBY;
        }        
        }
        else{
        if (elapsed_time >= 5.0) // 5.0 为过渡持续时间
        {
            ctrl_state = C_STATE_STANDBY;
        }        
        }

    }

    rbt_output.target_q[0] += dynamic_offset.hip;
    rbt_output.target_q[6] += dynamic_offset.hip;
    rbt_output.target_q[3] += dynamic_offset.knee;
    rbt_output.target_q[9] += dynamic_offset.knee;
    rbt_output.target_q[4] += dynamic_offset.ankle;
    rbt_output.target_q[10] += dynamic_offset.ankle;
    
    rbt2mtr_map();
    for (int i = 0; i < info.params.num_actions; i++)
    {
        msg.rbt_target_msg.q[i] = rbt_output.target_q[i];
    }
    pub.rbt_target_pub_.publish(msg.rbt_target_msg);
    

    for (int i = 0; i < info.params.num_actions; ++i)
    {
            msg.mtr_target_msg.q[i] = mtr_output.target_q[i];
    }
    pub.mtr_target_pub_.publish(msg.mtr_target_msg);
}

void PD::write_joint_dev()
{
    if(ctrl_state == C_DEVELOP_CONTROL_MOTOR)
    {
        for (int i = 0; i < info.params.num_actions; ++i)
        {

            msg.mtr_target_msg.q[i] = mtr_output.target_q[i];
            msg.mtr_target_msg.dq[i] = mtr_output.target_dq[i];
            msg.mtr_target_msg.tau[i] = mtr_output.target_tau[i];
        }
        pub.mtr_target_pub_.publish(msg.mtr_target_msg);
        return;
    }


    for (int i = 0; i < info.params.num_actions; i++)
    {
        rbt_output.target_q[i] = rbt_output_last.target_q[i] ;
        rbt_output.target_dq[i] = rbt_output_last.target_dq[i];
        rbt_output.target_tau[i] = rbt_output_last.target_tau[i] ;
    }
    
    rbt_output.target_q[0] += dynamic_offset.hip;
    rbt_output.target_q[6] += dynamic_offset.hip;
    rbt_output.target_q[3] += dynamic_offset.knee;
    rbt_output.target_q[9] += dynamic_offset.knee;
    rbt_output.target_q[4] += dynamic_offset.ankle;
    rbt_output.target_q[10] += dynamic_offset.ankle;

    rbt2mtr_map();
    for (int i = 0; i < info.params.num_actions; i++)
    {
        msg.rbt_target_msg.q[i] = rbt_output.target_q[i];
    }
    pub.rbt_target_pub_.publish(msg.rbt_target_msg);
    

    for (int i = 0; i < info.params.num_actions; ++i)
    {
            msg.mtr_target_msg.q[i] = mtr_output.target_q[i];
    }
    pub.mtr_target_pub_.publish(msg.mtr_target_msg);
}

void PD::motor_pd_dev()
{

    if (motor_command_type_ == MotorCommandType::only_torque)
    {
        // ROS_INFO("only_torque");

        for (int i = 0; i < info.params.num_actions; i++)
        {
            mtr_output.target_tau[i] += info.params.running_kp[0][i] * (mtr_output.target_q[i] - mtr_state.q[i]) + info.params.running_kd[0][i] * (mtr_output.target_dq[i] - mtr_state.dq[i]);
            // rb_ptr_->fresh_cmd_dynamic_config(0, 0, mtr_output.target_tau[i], 0, 0, info.params.map_index[0][i].item<int>());
            //rb_ptr_->fresh_cmd_dynamic_config(0, 0, 0, 0, 0, info.params.map_index[0][i]);
            int map_index = info.params.map_index[0][i];
            rb_ptr_->Motors[map_index]->fresh_cmd_int16(0, 0, 0, 
                                                        0, 0, 0,
                                                        0, 0, 0);
        }
    }
    if (motor_command_type_ == MotorCommandType::pd_param)
    {

        for (int i = 0; i < info.params.num_actions; i++)
        {
            // rb_ptr_->fresh_cmd_dynamic_config(mtr_output.target_q[i], mtr_output.target_dq[i], mtr_output.target_tau[i], info.params.running_kp[0][i].item<double>(), info.params.running_kd[0][i].item<double>(), info.params.map_index[0][i].item<int>());
            //rb_ptr_->fresh_cmd_dynamic_config(0, 0, 0, 0, 0, info.params.map_index[0][i]);
            int map_index = info.params.map_index[0][i];
            rb_ptr_->Motors[map_index]->fresh_cmd_int16(0, 0, 0, 
                                                        0, 0, 0,
                                                        0, 0, 0);
        }
    }
    rb_ptr_->motor_send_2();
}



void PD::motor_pd(bool is_teach_mod)
{
    if (ctrl_state == C_STATE_FORCE_SHUT_DOWN)
        return;

    if (motor_command_type_ == MotorCommandType::only_torque)
    {
        // ROS_INFO("only_torque");

        for (int i = 0; i < info.params.num_actions; i++)
        {
            mtr_output.target_tau[i] = info.params.running_kp[0][i] * (mtr_output.target_q[i] - mtr_state.q[i]) - info.params.running_kd[0][i] * mtr_state.dq[i];
            //rb_ptr_->fresh_cmd_dynamic_config(0, 0, mtr_output.target_tau[i], 0, 0, info.params.map_index[0][i]);
            int map_index = info.params.map_index[0][i];
            rb_ptr_->Motors[map_index]->fresh_cmd_int16(0, 0, mtr_output.target_tau[i], 
                                                        0, 0, 0,
                                                        0, 0, 0);
        }
    }
    if (motor_command_type_ == MotorCommandType::pd_param)
    {
        // ROS_INFO("pd_param");
        if(is_teach_mod == true){
            //ROS_INFO(" motor_pd is_teach_mod KP KD !!!! ");
            //int running_kp_[12] = {60, 40, 20, 60, 30, 10, 60, 40, 20, 60, 30, 10 };
            //float running_kd_[12] = {2.4, 0.8, 0.4, 2.8, 1.6, 0.3, 2.4, 0.8, 0.4, 2.8, 1.6, 0.3 };
            for (int i = 0; i < info.params.num_actions; i++) {
                //rb_ptr_->fresh_cmd_dynamic_config(mtr_output.target_q[i], 0, 0, info.params.teach_kp[0][i], info.params.teach_kd[0][i],
                //                                  info.params.map_index[0][i]);
                int map_index = info.params.map_index[0][i];
                rb_ptr_->Motors[map_index]->fresh_cmd_int16(mtr_output.target_q[i], 0, 0, 
                                                        info.params.teach_kp[0][i], 0, info.params.teach_kd[0][i],
                                                        0, 0, 0);
            }
        } else {
            for (int i = 0; i < info.params.num_actions; i++) {
                //rb_ptr_->fresh_cmd_dynamic_config(mtr_output.target_q[i], 0, 0, info.params.running_kp[0][i], info.params.running_kd[0][i],
                //                                  info.params.map_index[0][i]);
                int map_index = info.params.map_index[0][i];
                rb_ptr_->Motors[map_index]->fresh_cmd_int16(mtr_output.target_q[i], 0, 0, 
                                                        info.params.running_kp[0][i], 0, info.params.running_kd[0][i],
                                                        0, 0, 0);
            }
        }
    }
    rb_ptr_->motor_send_2();
}

void PD::shut_down_motor_pd()
{
    if (ctrl_state == C_STATE_FORCE_SHUT_DOWN)
        return;
    if (motor_command_type_ == MotorCommandType::only_torque)
    {
        // ROS_INFO("only_torque");

        for (int i = 0; i < info.params.num_actions; i++)
        {
            mtr_output.target_tau[i] = info.params.shut_down_kp[0][i] * (mtr_output.target_q[i] - mtr_state.q[i]) - info.params.shut_down_kd[0][i] * mtr_state.dq[i];
            //rb_ptr_->fresh_cmd_dynamic_config(0, 0, mtr_output.target_tau[i], 0, 0, info.params.map_index[0][i]);
            int map_index = info.params.map_index[0][i];
            rb_ptr_->Motors[map_index]->fresh_cmd_int16(0, 0, mtr_output.target_tau[i], 
                                                        0, 0, 0,
                                                        0, 0, 0);
        }
    }
    if (motor_command_type_ == MotorCommandType::pd_param)
    {
        // ROS_INFO("pd_param");

        for (int i = 0; i < info.params.num_actions; i++)
        {
            //rb_ptr_->fresh_cmd_dynamic_config(mtr_output.target_q[i], 0, 0, info.params.shut_down_kp[0][i], info.params.shut_down_kd[0][i], info.params.map_index[0][i]);
            int map_index = info.params.map_index[0][i];
            rb_ptr_->Motors[map_index]->fresh_cmd_int16(mtr_output.target_q[i], 0, 0,
                                                        info.params.shut_down_kp[0][i], 0, info.params.shut_down_kd[0][i],
                                                        0, 0, 0);
        }
    }
    rb_ptr_->motor_send_2();
}

void PD::send_motor_zero()
{
    for (int i = 0; i < info.params.num_actions; i++)
    {
        //rb_ptr_->fresh_cmd_dynamic_config(0, 0, 0, 0, 0, info.params.map_index[0][i]);
        int map_index = info.params.map_index[0][i];
        rb_ptr_->Motors[map_index]->fresh_cmd_int16(0, 0, 0,
                                                    0, 0, 0,
                                                    0, 0, 0);
    }
    rb_ptr_->motor_send_2();
}

std::vector<std::vector<double>> PD::loadVectorWithBoost(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        ROS_ERROR("Failed to open waypoint file: %s", filename.c_str());
        return {};
    }
    boost::archive::text_iarchive ia(inFile);
    std::vector<std::vector<double>> data;
    ia >> data;
    return data;
}

std::vector<std::vector<double>> PD::vector_transpose(const std::vector<std::vector<double>>& matrix) {
    if (matrix.empty()) {
        return {};
    }

    size_t rows = matrix.size();
    size_t cols = matrix[0].size();

    std::vector<std::vector<double>> transposed(cols, std::vector<double>(rows));

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            transposed[j][i] = matrix[i][j];
        }
    }

    return transposed;
}

bool PD::load_waypoint(ActionType action_type)
{
    std::string package_path = ros::package::getPath("sim2real"); // 请替换为您的包名

    std::string waypoint_file;
    switch (action_type)
    {
        case ActionType::Action1:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[0]);
            break;
        case ActionType::Action2:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[1]);
            break;
        case ActionType::Action3:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[2]);
            break;
        case ActionType::Action4:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[3]);
            break;
        case ActionType::Action_Falldown_Front:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[4]);
            break;
        case ActionType::Action_Falldown_Back:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[5]);
            break;
        case ActionType::ActionFalldown_Left:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[6]);
            break;
        case ActionType::ActionFalldown_Right:
            way_point_ = loadVectorWithBoost(package_path + "/" + info.params.waypoint_files[7]);
            break;
        default:
            ROS_ERROR("Invalid action_type: %d", static_cast<int>(action_type));
            return false;
    }

    // 转置 waypoint，使每一行对应一个关节的轨迹
    transposed_waypoint_ = vector_transpose(way_point_);

    // 获取关节数量和时间点数量
    num_joints_ = transposed_waypoint_.size();
    if (num_joints_ == 0) {
        return 0;
    }
    num_points_ = transposed_waypoint_[0].size();
    t_list_.resize(num_points_);
    std::iota(t_list_.begin(), t_list_.end(), 0);
    
    pos_tr_.resize(num_joints_);
    vel_tr_.resize(num_joints_);

    current_joint_index_ = 0;

    return true;
}

void PD::generateInterpolatedTrajectory()
{
    // 检查是否所有关节已处理完
    if (current_joint_index_ >= num_joints_) {
        action_state = ActionState::INTERPOLATION_FINISHED;
        return;
    }

    // 对当前关节进行插值
    int i = current_joint_index_;
    const auto& joint_points = transposed_waypoint_[i];

    // 创建插值器实例
    CubicSplineInterpolator interpolator(joint_points, 0.0, 0.0, t_list_);

    // 计算插值参数
    interpolator.calculatePosParameters();
    interpolator.calculateVelParameters();

    // 生成插值后的位置和速度
    double interpolation_interval = 0.05; // 根据需要设置插值间隔
    interpolator.generateInterpolatedPoints(interpolation_interval);
    interpolator.generateInterpolatedVels(interpolation_interval);

    // 存储插值结果
    pos_tr_[i] = interpolator.get_poly_pos_trace();
    vel_tr_[i] = interpolator.get_poly_vel_trace();

    // 移动到下一个关节
    current_joint_index_++;
}


void PD::execute_action()
{
    switch(action_state)
    {
        case ActionState::LOADFILE:
            load_waypoint(action_type);
            action_state = ActionState::INTERPOLATION;
            break;
        case ActionState::INTERPOLATION:
            generateInterpolatedTrajectory();
            break;

        case ActionState::INTERPOLATION_FINISHED:
        {
            // 所有关节的插值已完成
            // 转置回原来的格式，每个元素是所有关节在某个时间点的状态
            pos_way_point_continue_ = vector_transpose(pos_tr_);
            vel_way_point_continue_ = vector_transpose(vel_tr_);

            joint_pos_ = pos_way_point_continue_.begin();
            joint_vel_ = vel_way_point_continue_.begin();
            cnt_ = 0;
            current_joint_index_ = 0;

            ROS_INFO("Waypoint %d loaded and interpolated", static_cast<int>(action_type));
            //ROS_INFO("rb_ptr_->set_reset()!!!!!!");
            //rb_ptr_->set_reset();
            ROS_INFO(" set_KP KD!!!!!!");

            //int running_kp_[12] = {60, 40, 20, 60, 30, 10, 60, 40, 20, 60, 30, 10 };
            //float running_kd_[12] = {2.4, 0.8, 0.4, 2.8, 1.6, 0.3, 2.4, 0.8, 0.4, 2.8, 1.6, 0.3 };
            auto pos = (*joint_pos_).begin();
            size_t map_index_loc[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
            for (int i = 0; i < info.params.num_actions; i++) {
                //rb_ptr_->fresh_cmd_dynamic_config(*pos, 0, 0, info.params.teach_kp[0][i], info.params.teach_kd[0][i], info.params.map_index[0][i]);
                //rb_ptr_->fresh_cmd_dynamic_config(*pos, 0, 0, info.params.teach_kp[0][i], info.params.teach_kd[0][i], map_index_loc[i]);
                int map_index = map_index_loc[i];
                rb_ptr_->Motors[map_index]->fresh_cmd_int16(*pos, 0, 0,
                                                    info.params.teach_kp[0][i], 0, info.params.teach_kd[0][i],
                                                    0, 0, 0);
                
                pos ++;
                //rb_ptr_->fresh_cmd_dynamic_config(0, 0, 0, running_kp_[i], running_kd_[i], info.params.map_index[0][i]);
            }
            rb_ptr_->motor_send_2();
            action_state = ActionState::EXECUTION;
        }
            break;

        case ActionState::EXECUTION:
            {
                if (cnt_ % 50 == 0)
                {
                    auto pos = (*joint_pos_).begin();
                    auto vel = (*joint_vel_).begin();
                    for(auto m:rb_ptr_->Motors)
                    {                
                        m->pos_vel_MAXtqe(*pos, *vel, 10);
                        pos ++;
                        vel ++;
                    }
                    rb_ptr_->motor_send_2();
                    joint_pos_ ++;
                    joint_vel_ ++;
                    if(joint_pos_ == pos_way_point_continue_.end())
                    {
                        action_state = ActionState::COMPLETED;
                        ROS_INFO("-------->Execute end.<-------");
                    }
                }
                cnt_++;
            }
            break;
        
        case ActionState::COMPLETED:
            {
                if(msg.pd2rl_msg.is_falldown = true)
                    msg.pd2rl_msg.is_falldown = false;
                transition_changemode_flag = false;
                ctrl_state = C_STATE_TRANSITION;
                ROS_INFO("-------->to C_STATE_TRANSITION<-------");
                action_type = hightorque::ActionType::ACTION_NULL;
            }
            break;
        default:
            break;
    }
}

void PD::exec_loop()
{
    ROS_INFO("pd_ctrl_f: %dHz", info.params.pd_ctrl_f);
    ros::Rate rate(info.params.pd_ctrl_f);

    while (!quit.load()&&ros::ok())
    {
        switch (ctrl_state)
        {
        case C_DEVELOP_CONTROL_JOINT:
        case C_DEVELOP_CONTROL_MOTOR:
            info.params.action_scale = 1;
            read_joint();
            write_joint_dev();
            motor_pd_dev();            
            break;
        
        case C_STATE_INIT:
        case C_STATE_RUNNING:
            info.params.action_scale =running_scale_;
            read_joint();
            write_joint();
            motor_pd();
            if(msg.yesenseIMU.linear_acceleration.x < -9.0f &&  msg.ori_msg.eu_ang[1] > 1.3f && !msg.pd2rl_msg.is_falldown)
            {
                // 检测前倾倒
                livelybot_logger::LoggerInterface::logOperation("PDCtrl_Fall_Down", "Front");
                msg.pd2rl_msg.is_falldown = true;
                pub.pd2rl_pub_.publish(msg.pd2rl_msg);
                action_type = ActionType::Action_Falldown_Front;
                action_state = ActionState::LOADFILE;
            }
            else if(msg.yesenseIMU.linear_acceleration.x > 9.0f &&  msg.ori_msg.eu_ang[1] < -1.3f && !msg.pd2rl_msg.is_falldown)
            {
                // 检测后倾倒
                livelybot_logger::LoggerInterface::logOperation("PDCtrl_Fall_Down", "Back");
                msg.pd2rl_msg.is_falldown = true;
                pub.pd2rl_pub_.publish(msg.pd2rl_msg);
                action_type = ActionType::Action_Falldown_Back;
                action_state = ActionState::LOADFILE;
            }
            else if(msg.yesenseIMU.linear_acceleration.y < -9.0f && msg.ori_msg.eu_ang[0] < -1.3f && !msg.pd2rl_msg.is_falldown) {
                // 检测向左倾倒
                livelybot_logger::LoggerInterface::logOperation("PDCtrl_Fall_Down", "Left");
                msg.pd2rl_msg.is_falldown = true;
                pub.pd2rl_pub_.publish(msg.pd2rl_msg);
                action_type = ActionType::ActionFalldown_Left;
                action_state = ActionState::LOADFILE;
            }
            else if(msg.yesenseIMU.linear_acceleration.y > 9.0f && msg.ori_msg.eu_ang[0] > 1.3f && !msg.pd2rl_msg.is_falldown) {
            // 检测向右倾倒
                livelybot_logger::LoggerInterface::logOperation("PDCtrl_Fall_Down", "Right");
                msg.pd2rl_msg.is_falldown = true;
                pub.pd2rl_pub_.publish(msg.pd2rl_msg);
                action_type = ActionType::ActionFalldown_Right;
                action_state = ActionState::LOADFILE;
            }
            break;

        case C_STATE_STANDBY:
            if (action_type > hightorque::ActionType::ACTION_NULL)
            {   
                execute_action();
            } 
            else
            {
                info.params.action_scale = 0.05;
                read_joint();
                write_joint();
                motor_pd();
            }    
            break;

        case C_STATE_SHUT_DOWN:
            if (shut_down_state == D_STATE_STAND)
            {
                info.params.action_scale = 0.01;
                read_joint();
                write_joint();
                motor_pd();
                if (static_cast<double>((ros::Time::now() - shut_down_time).toSec()) > 1.5)
                {
                    shut_down_state = D_STATE_CALC;
                }
            }
            else if (shut_down_state == D_STATE_CALC)
            {
                Eigen::Vector3d left_ankle_position = whole_body_kinematics(rbt_model, rbt_data, "l_ankle_roll_link", rbt_state.q);
                Eigen::Vector3d right_ankle_position = whole_body_kinematics(rbt_model, rbt_data, "r_ankle_roll_link", rbt_state.q);
                shut_down_total_height = (left_ankle_position.z() + right_ankle_position.z()) / 2.0;
                shut_down_state = D_STATE_SQUAT;
                shut_down_time = ros::Time::now();
            }
            else if (shut_down_state == D_STATE_SQUAT)
            {
                shut_down_height = shut_down_total_height + static_cast<double>((ros::Time::now() - shut_down_time).toSec()) * info.params.shut_down_speed;
                read_joint();
                write_joint();
                shut_down_motor_pd();
                if (shut_down_height > -0.21)
                {
                    shut_down_state = D_STATE_OFF;
                }
            }
            else
            {
                ctrl_state = C_STATE_FORCE_SHUT_DOWN;
                msg.pd2rl_msg.ctrl_state = C_STATE_FORCE_SHUT_DOWN;
                ROS_INFO("Robot shut down!!");
            }
            break;

        case C_STATE_FORCE_SHUT_DOWN:
            quit.store(true);
            send_motor_zero();
            break;
        case C_STATE_TRANSITION:
            {
                info.params.action_scale =running_scale_;

                if(transition_changemode_flag == false)
                {
                    transition_changemode_flag = true;
                    init_state = I_STATE_SAMPLE;
                }
                //ROS_INFO("C_STATE_TRANSITION"); 长时间持续
                read_joint();
                write_joint();
                motor_pd(true);
            }
            break;
        case C_STATE_WAITING:
        default:
            send_motor_zero();
            break;
        }
        pub.pd2rl_pub_.publish(msg.pd2rl_msg);
        rate.sleep();
    }
    ROS_INFO("pd_controller exec loop terminate");
}

}//namespace hightorque