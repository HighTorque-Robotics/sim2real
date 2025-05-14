#ifndef RL_SDK_H
#define RL_SDK_H

#include <ros/ros.h>
#include "read_rl_yaml.h"
#include "robot_data.h"
#include </usr/include/eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <memory>

#ifdef PLATFORM_X86_64
#include <openvino/openvino.hpp>
#elif defined(PLATFORM_ARM)
#include <rknn_api.h>
#endif
namespace hightorque{

enum Mode
{
    DEFAULT_MODE,
    CUSTOM_MODE
};

struct GaitParam
{
    double frequency = 1.5;
    double offsets = 0.5;
    double durations = 0.5;
    double swing_height = 0.05;

    double gait_indices = 0;
    double swing_height_indices = 0;

    Eigen::Matrix<double, 2, 1> gait_update(double dt){
        auto remainder = [](const double& x) -> double {
            return std::fmod(std::fmod(x, 1.0) + 1.0, 1.0);
        };

        // gait index update
        gait_indices = remainder(
            gait_indices + dt * frequency
        );
        swing_height_indices = remainder(
            swing_height_indices + dt * frequency * 2.0
        );

        double contact_phase_1 = remainder(gait_indices + offsets + 1);
        double contact_phase_2 = remainder(gait_indices);

        Eigen::Matrix<double, 2, 1> desired_contact_states = Eigen::Matrix<double, 2, 1>(
            contact_phase_1 - durations, 
            contact_phase_2 - durations);

        // lambda：swing height function
        auto swing_func = [&](double phase, double contact_state) -> double {
            return swing_height * (std::sin(2.0 * M_PI * phase - 0.5 * M_PI) / 2.0 + 0.5) * (contact_state > 0.0 ? 1.0 : 0.0);
        };

        Eigen::Matrix<double, 2, 1> swing_height_target = Eigen::Matrix<double, 2, 1>(
            swing_func(swing_height_indices, desired_contact_states[0]), 
            swing_func(swing_height_indices, desired_contact_states[1])
        );
        return swing_height_target;
    }
};

class RL{
public:
    RL();
    RL(const std::string&  config_path);
    ~RL();

    void init_data();

    void update_observation_ts();
    void update_action_ts();

    void update_observation_dreamwaq();
    void update_action_dreamwaq();

    //落足点
    void update_observation_footstep();
    void update_action_footstep();

    void update_observation_humanoidgym();
    void update_action_humanoidgym();
    void read_orient();

    void quat2euler();

    void pd2rl_callback(const sim2real_msg::PD2RL::ConstPtr& pd2rl_msg);
    void joy_model_callback(const sim2real_msg::ControlState::ConstPtr& joy_msg);
    void joy_control_callback(const geometry_msgs::Twist::ConstPtr& joy_msg);
    void usr_control_callback(const hightorque_hardware_sdk::usr2ctrl::ConstPtr& usr_msg);
    void odom_callback(const sensor_msgs::Imu::ConstPtr& odom);

    void exec_loop();
    void collect_ctrl2usr_data();
    bool exec_loop_is_terminate();
    Eigen::Vector3d qr_ty_i(const Eigen::Vector4d& q,const Eigen::Vector3d& v);

public:
    RL_YamlInfo info;
    Command command;
    Ctrl_State ctrl_state;
    Robot_State rbt_state;
    Robot_Output rbt_output;
    Observations obs;
    RL_Subscriber sub;
    RL_Publisher pub;
    RL_Msgs msg;
    ros::NodeHandle nh;
    ros::Time start_time;
    int standby_left = 0;
    int standby_right = 0;
    uint8_t joy_usr_flag = 0;
private:
    // torch::jit::script::Module policy;
    std::shared_ptr<std::thread> thread_exec_ptr_;
    std::atomic<bool> quit;
    Mode mode_; // 模式成员变量
    bool init_timer_started;
    double phase;
    double step_period;
    double full_step_period;

    ros::Publisher action_pub_;

    GaitParam gait_params_; 

    Eigen::Matrix<double, 12, 1> mujoco2Lab(const Eigen::Matrix<double, 12, 1>& x) {
        const std::array<int, 12> joint_index_mujoco = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        const std::array<int, 12> joint_index_lab    = {0, 2, 4, 6, 8, 10, 1, 3, 5, 7, 9, 11};

        Eigen::Matrix<double, 12, 1> x_lab = Eigen::Matrix<double, 12, 1>::Zero();

        for (int i = 0; i < 12; ++i) {
            x_lab[joint_index_lab[i]] = x[joint_index_mujoco[i]];
        }
    
        return x_lab;
    };

    Eigen::Matrix<double, 12, 1> lab2Mujoco(const Eigen::Matrix<double, 12, 1>& actions) {
        const std::array<int, 12> joint_index_mujoco = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        const std::array<int, 12> joint_index_lab    = {0, 2, 4, 6, 8, 10, 1, 3, 5, 7, 9, 11};
    
        Eigen::Matrix<double, 12, 1> actions_mujoco = Eigen::Matrix<double, 12, 1>::Zero();
    
        for (int i = 0; i < 12; ++i) {
            actions_mujoco[joint_index_mujoco[i]] = actions[joint_index_lab[i]];
        }
    
        return actions_mujoco;
    };

#ifdef PLATFORM_X86_64
    // OpenVINO 相关成员变量
    ov::Core core;  // OpenVINO 核心对象
    std::shared_ptr<ov::Model> model;  // 模型对象
    ov::CompiledModel compiled_model;  // 编译后的模型
    ov::InferRequest infer_request;  // 推理请求
    #elif defined(PLATFORM_ARM)
    // RKNN 相关成员变量
    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_input rknn_inputs[1];
    rknn_output rknn_outputs[1];
    #endif

};
}
#endif