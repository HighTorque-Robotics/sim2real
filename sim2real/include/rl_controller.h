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

class RL{
public:
    RL();
    RL(const std::string&  config_path);
    ~RL();

    void init_data();

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