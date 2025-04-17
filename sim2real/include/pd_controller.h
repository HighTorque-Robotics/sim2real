#ifndef PD_SDK_H
#define PD_SDK_H

#include <pinocchio/fwd.hpp>

#include <ros/ros.h>
#include "read_pd_yaml.h"
#include "robot_data.h"
#include </usr/include/eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <string>
//#include "../include/hardware/robot.h"
#include "hardware/robot.h"
#include "ros/time.h"
#include "kinematics.h"
#include <atomic>
#include "sim2real/CubicSplineInterpolator.h"

namespace hightorque
{
    enum class MotorCommandType
    {
        pd_param,
        only_torque
    };

    enum class ActionType
    {
        ACTION_NULL = 0,
        Action1 = 1,
        Action2 = 2,
        Action3 = 3,
        Action4 = 4,
        Action_Falldown_Front = 5,
        Action_Falldown_Back = 6,
        ActionFalldown_Left = 7,
        ActionFalldown_Right = 8
    };

    enum class ActionState
    {
        LOADFILE,
        INTERPOLATION,
        INTERPOLATION_FINISHED,
        EXECUTION,
        COMPLETED
    };


    class PD
    {
    public:
        PD();
        PD(std::shared_ptr<livelybot_serial::robot> rb_ptr,const std::string&  config_path);
        PD(std::shared_ptr<livelybot_serial::robot> rb_ptr);
        ~PD();

        void init_data();

        void mtr2rbt_map();
        void read_joint();

        void ankle_pos_inv_map();
        void rbt2mtr_map();
        void write_joint();
        void write_joint_dev();

        void motor_pd(bool is_teach_mod=false);
        void motor_pd_dev();
        void send_motor_zero();
        void shut_down_motor_pd();

        void rl2pd_callback(const sim2real_msg::RL2PD::ConstPtr &rl2pd_msg);
        void joy_callback(const sim2real_msg::ControlState::ConstPtr &joy_msg);
        void odom_callback(const sensor_msgs::Imu::ConstPtr& odom);
        void ori_callback(const sim2real_msg::Orient::ConstPtr& ori);
        

        void exec_loop();

        bool exec_loop_is_terminate();

        void set_motor_command_type(int type);


        void set_ctrl_state(int state_idx);

        void set_joint_command(sensor_msgs::JointState  joint_command);
        
        void set_motor_command(sensor_msgs::JointState  motor_command);

        // 加载 waypoint 数据
        std::vector<std::vector<double>> loadVectorWithBoost(const std::string& filename);

        // 矩阵转置函数
        std::vector<std::vector<double>> vector_transpose(const std::vector<std::vector<double>>& matrix);

        bool load_waypoint(ActionType action_type);

        void generateInterpolatedTrajectory();

        void execute_action();
    public:
        PD_YamlInfo info;
        Ctrl_State ctrl_state;
        Motor_State mtr_state;
        Robot_State rbt_state;
        Motor_Output mtr_output;
        Robot_Output rbt_output;
        Robot_Output rbt_output_last;
        Transform trans;
        DynamicOffset dynamic_offset;
        PD_Subscriber sub;
        PD_Publisher pub;
        PD_Msgs msg;
        ros::NodeHandle nh;
        // state_machine
        double rbt_q_init[12];
        double rbt_tq_init[12];
        Init_State init_state;
        Shut_Down_State shut_down_state;
        ros::Time init_start_time;
        ros::Time shut_down_time;
        double shut_down_height;
        double shut_down_total_height;
        // pinocchio
        std::string urdf_path;
        std::string mesh_path;
        pinocchio::Model rbt_model;
        pinocchio::Data rbt_data;
        MotorCommandType motor_command_type_;
        double running_scale_;
        
        ActionType action_type;
        ActionState action_state;
        // Waypoint 数据
        std::vector<std::vector<double>> way_point_;       // 加载的原始 waypoint 数据
        std::vector<std::vector<double>> pos_way_point_continue_; // 插值后的关节位置序列
        std::vector<std::vector<double>> vel_way_point_continue_; // 插值后的关节速度序列
        // 轨迹执行指针
        std::vector<std::vector<double>>::iterator joint_pos_; // 当前执行的关节位置
        std::vector<std::vector<double>>::iterator joint_vel_; // 当前执行的关节速度

        int cnt_;                    // 执行计数器
        int current_joint_index_;  // 当前关节索引

        // 用于存储中间数据
        std::vector<std::vector<double>> transposed_waypoint_;
        size_t num_joints_;
        size_t num_points_;
        std::vector<double> t_list_;
        std::vector<std::vector<double>> pos_tr_;
        std::vector<std::vector<double>> vel_tr_;
        bool transition_changemode_flag;

    private:
        std::shared_ptr<livelybot_serial::robot> rb_ptr_;
        std::atomic<bool> quit;
        std::shared_ptr<std::thread> thread_exec_ptr_;
        std::string resources_dir_;
    };

} // namespace hightorque

#endif