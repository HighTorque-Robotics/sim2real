#include "rl_controller.h"
#include "robot_data.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
// #include <torch/torch.h>
#include <algorithm> // for std::clam
#include <bits/algorithmfwd.h>
#include "livelybot_logger/logger_interface.h"

#include "std_msgs/Float64MultiArray.h"

static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz)
{
    unsigned char* data;
    int ret;

    data = NULL;

    if (NULL == fp) {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0) {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char*)malloc(sz);
    if (data == NULL) {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}
static unsigned char* read_file_data(const char* filename, int* model_size)
{
    FILE* fp;
    unsigned char* data;

    fp = fopen(filename, "rb");
    if (NULL == fp) {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

namespace hightorque
{

    RL::RL() : nh(), mode_(DEFAULT_MODE), init_timer_started(false) 
    {
        init_data();
    }
    RL::RL(const std::string &config_path) : nh(), mode_(CUSTOM_MODE)
    {
        info = RL_YamlInfo(config_path);
        init_data();
    }

    RL::~RL()
    {
        quit.store(true);
        if (thread_exec_ptr_->joinable())
        {
            thread_exec_ptr_->join();
        }
        ROS_ERROR("RL destruction");
    }

    void RL::init_data()
    {
        phase = 0;
        step_period = 30;
        full_step_period = step_period * 2;

        ctrl_state = C_STATE_WAITING;
        msg.rl2pd_msg.ctrl_state = C_STATE_WAITING;

        rbt_state.eu_ang = Eigen::VectorXd::Zero(3);
        rbt_state.base_ang_vel = Eigen::VectorXd::Zero(3);
        rbt_state.q = Eigen::VectorXd::Zero(info.params.num_actions);
        rbt_state.dq = Eigen::VectorXd::Zero(info.params.num_actions);
        rbt_state.tau = Eigen::VectorXd::Zero(info.params.num_actions);

        rbt_output.target_q = Eigen::VectorXd::Zero(info.params.num_actions);
        rbt_output.target_dq = Eigen::VectorXd::Zero(info.params.num_actions);
        rbt_output.target_tau = Eigen::VectorXd::Zero(info.params.num_actions);
        rbt_output.action = Eigen::VectorXd::Zero(info.params.num_actions);

        obs.observations = Eigen::VectorXd::Zero(info.params.num_single_obs);
        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.hist_obs.push_back(Eigen::VectorXd::Zero(info.params.num_single_obs));
        }
        obs.input = Eigen::MatrixXd::Zero(1, info.params.num_single_obs * info.params.frame_stack);

        sub.pd2rl_sub_ = nh.subscribe<sim2real_msg::PD2RL>("pd2rl", 100, &RL::pd2rl_callback, this);
        pub.rl2pd_pub_ = nh.advertise<sim2real_msg::RL2PD>("rl2pd", 100);

        sub.joy_model_sub_ = nh.subscribe<sim2real_msg::ControlState>("joy_msg", 10, &RL::joy_model_callback, this);
        sub.joy_control_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &RL::joy_control_callback, this);

        sub.usr_ctrl_sub_ = nh.subscribe<hightorque_hardware_sdk::usr2ctrl>("usr2ctrl_data", 10, &RL::usr_control_callback, this);
        pub.ctrl_usr_pub_ = nh.advertise<hightorque_hardware_sdk::ctrl2usr>("ctrl2usr_data", 10);

        sub.odom_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, &RL::odom_callback, this);

        pub.ori_pub_ = nh.advertise<sim2real_msg::Orient>("ori_state", 100);
        action_pub_ = nh.advertise<std_msgs::Float64MultiArray>("policy_output", 1000);
#ifdef PLATFORM_X86_64
        model = core.read_model(info.policy_path);             // 加载模型
        compiled_model = core.compile_model(model, "CPU");     // 编译模型（选择设备为 "CPU" 或 "GPU"）
        infer_request = compiled_model.create_infer_request(); // 创建推理请求
        #elif defined(PLATFORM_ARM)
        int model_data_size = 0;
        unsigned char* model_data = read_file_data(info.policy_path.c_str(), &model_data_size);
        int ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
        if (ret < 0) {
            printf("rknn_init error ret=%d\n", ret);
        }
        std::cout << "RKNN model loaded successfully" << std::endl;

        rknn_sdk_version version;
        ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
        if (ret < 0) {
            printf("rknn_query RKNN_QUERY_SDK_VERSION error ret=%d\n", ret);
        }


        ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
        if (ret < 0) {
            printf("rknn_query RKNN_QUERY_IN_OUT_NUM error ret=%d\n", ret);
        }
        printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

        memset(rknn_inputs, 0, sizeof(rknn_inputs));
        rknn_inputs[0].index = 0;
        rknn_inputs[0].size = obs.input.size() * sizeof(float);
        rknn_inputs[0].pass_through = false;
        rknn_inputs[0].type = RKNN_TENSOR_FLOAT32;
        rknn_inputs[0].fmt = RKNN_TENSOR_NHWC;

        memset(rknn_outputs, 0, sizeof(rknn_outputs));
        rknn_outputs[0].want_float = true;
        #endif

        thread_exec_ptr_.reset(new std::thread(&RL::exec_loop, this));
        quit.store(false);
    }

    void RL::pd2rl_callback(const sim2real_msg::PD2RL::ConstPtr &pd2rl_msg)
    {
        for (int i = 0; i < info.params.num_actions; i++)
        {
            rbt_state.q[i] = pd2rl_msg->rbt_state_q[i];
            rbt_state.dq[i] = pd2rl_msg->rbt_state_dq[i];
        }
        if (pd2rl_msg->ctrl_state == static_cast<Ctrl_State>(C_STATE_FORCE_SHUT_DOWN))
        {
            ctrl_state = C_STATE_FORCE_SHUT_DOWN;
            msg.rl2pd_msg.ctrl_state = C_STATE_FORCE_SHUT_DOWN;
        }
        if(pd2rl_msg->is_falldown == true)
        {
            ctrl_state = C_STATE_STANDBY;
            msg.rl2pd_msg.ctrl_state = ctrl_state;
        }
    }

    void RL::odom_callback(const sensor_msgs::Imu::ConstPtr &odom)
    {
        msg.yesenseIMU = *odom;
    }

    void RL::joy_model_callback(const sim2real_msg::ControlState::ConstPtr &joy_msg)
    {
        if(joy_usr_flag == 1)
        {
            return;
        }
        static int running_standby_switch_last = 0;
        switch (ctrl_state)
        {
        case C_STATE_WAITING:
            ROS_INFO_ONCE("state waiting");
            if (joy_msg->standby == 1)
            {
                ctrl_state = C_STATE_INIT;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_INIT");
                ROS_INFO("state init jiangbo");
            }
            if (joy_msg->sitdown == 1)
            {
                ctrl_state = C_STATE_SHUT_DOWN;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_SHUT_DOWN");
                ROS_INFO("state shut down");
            }
            break;

        case C_STATE_INIT:
            #if 0 //禁止从init 模式到running模式，机器人会跳起
            if (joy_msg->running_standby_switch == 1 && running_standby_switch_last == 0)
            {
                ctrl_state = C_STATE_RUNNING;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_RUNNING");
                ROS_INFO("state RL running, From C_STATE_INIT");
                command.vx = 0.0;
                command.vy = 0.0;
                command.dyaw = 0.0;
                start_time = ros::Time::now();
            }
            #endif
            if (joy_msg->sitdown == 1)
            {
                ctrl_state = C_STATE_SHUT_DOWN;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_SHUT_DOWN");
                ROS_INFO("state shutting down");
                standby_left = 0;
                standby_right = 0;
            }
            break;

        case C_STATE_STANDBY:
            if (joy_msg->running_standby_switch == 1 && running_standby_switch_last == 0)
            {
                msg.ctrl_usr_msg.robot_status = 4;
                ctrl_state = C_STATE_RUNNING;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_RUNNING");
                ROS_INFO("state RL running, From C_STATE_STANDBY");
                command.vx = 0.0;
                command.vy = 0.0;
                command.dyaw = 0.0;
                standby_left = 0;
                standby_right = 0;
                start_time = ros::Time::now();
            }
            if (joy_msg->sitdown == 1)
            {
                ctrl_state = C_STATE_SHUT_DOWN;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_SHUT_DOWN");
                ROS_INFO("state shutting down");
                standby_left = 0;
                standby_right = 0;
            }
            break;

        case C_STATE_RUNNING:
            if (joy_msg->running_standby_switch == 1 && running_standby_switch_last == 0 && command.vx == 0.0 && command.vy == 0.0 && command.dyaw == 0.0)
            {
                msg.ctrl_usr_msg.robot_status = 3;
                ctrl_state = C_STATE_STANDBY;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_STANDBY");
                standby_left = 0;
                standby_right = 0;
                ROS_INFO("state standby");
            }
            if (joy_msg->sitdown == 1)
            {
                ctrl_state = C_STATE_SHUT_DOWN;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                livelybot_logger::LoggerInterface::logOperation("RLCtrl_STATE", "C_STATE_SHUT_DOWN");
                ROS_INFO("state shutting down");
                standby_left = 0;
                standby_right = 0;
            }
            break;
        case C_STATE_SHUT_DOWN:
        case C_STATE_FORCE_SHUT_DOWN:
            msg.ctrl_usr_msg.robot_status = 0;
        default:
            break;
        }
        running_standby_switch_last = joy_msg->running_standby_switch;
    }

    void RL::joy_control_callback(const geometry_msgs::Twist::ConstPtr &joy_msg)
    {if(joy_usr_flag == 1)
        {
            return;
        }
        switch (ctrl_state)
        {
        case C_STATE_RUNNING:
        {
            static double start_time = -1.0;
            if (start_time < 0.0)
            {
                start_time = ros::Time::now().toSec();
            }
            bool is_save_command = false;
            if (ros::Time::now().toSec() - start_time > 1.0)
            {
                is_save_command = true;
                start_time = ros::Time::now().toSec();
            } 

            bool is_have_command = false;

            if ((double)joy_msg->linear.x >= 0.0)
            {
                if ((double)joy_msg->linear.z < -0.5)
                    command.vx = ((double)joy_msg->linear.x) * 1.5;
                else
                    command.vx = ((double)joy_msg->linear.x);
                is_have_command = true;
            }
            else if ((double)joy_msg->linear.x < -0.0)
            {
                if ((double)joy_msg->linear.z < -0.5)
                    command.vx = ((double)joy_msg->linear.x) * 1.2;
                else
                    command.vx = ((double)joy_msg->linear.x);
                is_have_command = true;
            }

            if ((double)joy_msg->linear.y >= 0.0){
                command.vy = ((double)joy_msg->linear.y);
                is_have_command = true;
            }
            else if ((double)joy_msg->linear.y < -0.0){
                command.vy = ((double)joy_msg->linear.y);
                is_have_command = true;
            }

            if ((double)joy_msg->angular.z >= 0.0){
                command.dyaw = ((double)joy_msg->angular.z);
                is_have_command = true;
            }
            else if ((double)joy_msg->angular.z < -0.0){
                command.dyaw = ((double)joy_msg->angular.z);
                is_have_command = true;
            }

            if (is_save_command && is_have_command)
            {
                std::string cmd_str = "vx: " + std::to_string(command.vx) + ", vy: " + std::to_string(command.vy) + ", dyaw: " + std::to_string(command.dyaw);
                livelybot_logger::LoggerInterface::logOperation("JoyCmd", cmd_str);
            }
        }
            break;
        default:
            break;
        }
    }
    void RL::usr_control_callback(const hightorque_hardware_sdk::usr2ctrl::ConstPtr &usr_msg)
    {
        joy_usr_flag = 1;

        static uint32_t robot_waiting_state = 0;

        if(usr_msg->robot_status == 1 || (robot_waiting_state>0 && robot_waiting_state<15))
        {
            robot_waiting_state++;
            ctrl_state = C_STATE_INIT;
            msg.ctrl_usr_msg.robot_status = 2;
            msg.rl2pd_msg.ctrl_state = ctrl_state;
            ROS_INFO("state init");
            ROS_INFO("robot_waiting_state:%d",robot_waiting_state);
            return;
        }

        static int running_standby_switch_last = 0;
        command.vx = usr_msg->twist_data.linear.x;
        command.vy = usr_msg->twist_data.linear.y;
        command.dyaw = usr_msg->twist_data.angular.z;
        if(usr_msg->robot_status == 3)
        {
                msg.ctrl_usr_msg.robot_status = 3;
                ctrl_state = C_STATE_STANDBY;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
            
                ROS_INFO("cmd_status: %d", msg.ctrl_usr_msg.robot_status);
        }
        if(usr_msg->robot_status == 4)
        {
                msg.ctrl_usr_msg.robot_status = 4;
                ctrl_state = C_STATE_RUNNING;
                msg.rl2pd_msg.ctrl_state = ctrl_state;
                ROS_INFO("cmd_status: %d", msg.ctrl_usr_msg.robot_status);
            
                start_time = ros::Time::now();
        }
        if(running_standby_switch_last != usr_msg->robot_status)
        {
                standby_left = 0;
                standby_right = 0;
        }
        running_standby_switch_last = usr_msg->robot_status;

    }
    void RL::quat2euler()
    {
        double x = rbt_state.quat.x();
        double y = rbt_state.quat.y();
        double z = rbt_state.quat.z();
        double w = rbt_state.quat.w();

        double t0 = +2.0 * (w * x + y * z);
        double t1 = +1.0 - 2.0 * (x * x + y * y);
        double roll_x = std::atan2(t0, t1);

        double t2 = +2.0 * (w * y - z * x);
        if (t2 > 1.0)
        {
            t2 = 1.0;
        }
        else if (t2 < -1.0)
        {
            t2 = -1.0;
        }

        double pitch_y = std::asin(t2);
        double t3 = +2.0 * (w * z + x * y);
        double t4 = +1.0 - 2.0 * (y * y + z * z);
        double yaw_z = std::atan2(t3, t4);

        rbt_state.eu_ang << roll_x, pitch_y, yaw_z;
        rbt_state.eu_ang = rbt_state.eu_ang.unaryExpr([](double angle)
                                                      { return angle > M_PI ? angle - 2 * M_PI : angle; });
    }
    Eigen::Vector3d RL::qr_ty_i(const Eigen::Vector4d &q, const Eigen::Vector3d &v)
    {
        double q_w = q[3];
        Eigen::Vector3d q_vec = q.head<3>();
        Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);
        Eigen::Vector3d b = q_vec.cross(v) * q_w * 2.0;
        Eigen::Vector3d c = q_vec * (q_vec.dot(v) * 2.0);
        return a - b + c;
    }
    void RL::update_observation_dreamwaq()
    {
        #ifdef PLATFORM_X86_64
        if (ctrl_state == C_STATE_RUNNING)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[10] >= 0.95)
                    standby_left = 1;
            }
            else
                obs.observations[10] = 1;
            if (standby_right == 0)
            {
                obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[11] <= -0.95)
                    standby_right = 1;
            }
            else
                obs.observations[11] = -1;
        }
        // 0-2 base_ang_vel
        obs.observations.segment(0, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        Eigen::Vector3d v(0, 0, -1);
        Eigen::Vector4d qt(rbt_state.quat.x(), rbt_state.quat.y(), rbt_state.quat.z(), rbt_state.quat.w());
        // 3-5 projected_gravity
        obs.observations.segment(3, 3) = qr_ty_i(qt, v);
        // 6,7,8commands
        obs.observations[6] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[7] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[8] = command.dyaw * info.params.cmd_ang_vel_scale;
        // 9 standing_command_mask
        if (std::abs(obs.observations[6]) > 1e-9 || std::abs(obs.observations[7]) > 1e-9 || std::abs(obs.observations[8]) > 1e-9)
        {
            obs.observations[9] = 0.0;
        }
        else
        {
            obs.observations[9] = 0.0;
            // obs.observations[9] = 1.0;
        }

        obs.observations.segment(12, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(24, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(36, 12) = rbt_output.action;
        #elif defined(PLATFORM_ARM)
        if (ctrl_state == C_STATE_RUNNING)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[10] >= 0.95)
                    standby_left = 1;
            }
            else
                obs.observations[10] = 1;
            if (standby_right == 0)
            {
                obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[11] <= -0.95)
                    standby_right = 1;
            }
            else
                obs.observations[11] = -1;
        }

        // 0-2 base_ang_vel
        obs.observations.segment(0, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        Eigen::Vector3d v(0, 0, -1);
        Eigen::Vector4d qt(rbt_state.quat.x(), rbt_state.quat.y(), rbt_state.quat.z(), rbt_state.quat.w());
        // 3-5 projected_gravity
        obs.observations.segment(3, 3) = qr_ty_i(qt, v);
        // 6,7,8commands
        obs.observations[6] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[7] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[8] = command.dyaw * info.params.cmd_ang_vel_scale;
        // 9 standing_command_mask
        if (std::abs(obs.observations[6]) > 1e-9 || std::abs(obs.observations[7]) > 1e-9 || std::abs(obs.observations[8]) > 1e-9)
        {
            obs.observations[9] = 0.0;
        }
        else
        {
            obs.observations[9] = 0.0;
            // obs.observations[9] = 1.0;
        }
        obs.observations.segment(12, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(24, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(36, 12) = rbt_output.action;
        #endif

        for (int i = 0; i < info.params.num_single_obs; ++i)
        {
            if (obs.observations[i] > info.params.clip_obs)
            {
                obs.observations[i] = info.params.clip_obs;
            }
            else if (obs.observations[i] < -info.params.clip_obs)
            {
                obs.observations[i] = -info.params.clip_obs;
            }
        }

        obs.hist_obs.push_back(obs.observations);
        obs.hist_obs.pop_front();
    }

    void RL::update_action_dreamwaq()
    {
        #ifdef PLATFORM_X86_64

        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        auto input_port = compiled_model.input();
        ov::Tensor input_tensor = infer_request.get_tensor(input_port);
        float *input_data = input_tensor.data<float>();
        for (size_t i = 0; i < input_tensor.get_size(); ++i)
        {
            input_data[i] = obs.input(i); // Example data
        }
        // double *data_ptr = obs.input.data();// 填充输入张量
        // std::memcpy(input_tensor.data<float>(), data_ptr, input_tensor.get_byte_size());
        infer_request.infer();// 执行推理
        auto output_port = compiled_model.output();// 获取输出张量
        ov::Tensor output_tensor = infer_request.get_tensor(output_port);

        // 处理输出数据
        const float *output_data = output_tensor.data<const float>();

        // std::cout << info.params.clip_actions_lower << std::endl;
        // std::cout << info.params.clip_actions_upper<< std::endl;
        for (int i = 0; i < info.params.num_actions; ++i)
        {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower_ov[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper_ov[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
        }
        #elif defined(PLATFORM_ARM)
        // 准备输入数据
        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        std::vector<float> input_data(obs.input.size());
        for (size_t i = 0; i < obs.input.size(); ++i) {
            input_data[i] = obs.input(i); // 转换 obs.input 到向量
        }

        rknn_inputs[0].buf = input_data.data();

        int ret = rknn_inputs_set(ctx, io_num.n_input, rknn_inputs);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to set RKNN input!"  << ret << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_run(ctx, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to run RKNN inference!" << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_outputs_get(ctx, io_num.n_output, rknn_outputs, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to get RKNN output!" << std::endl;
            rknn_destroy(ctx);
        }
        
        float *output_data = static_cast<float *>(rknn_outputs[0].buf);
        for (int i = 0; i < info.params.num_actions; ++i) {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
        }
        #endif
    }

    void RL::update_observation_footstep(){
#ifdef PLATFORM_X86_64
        if (ctrl_state == C_STATE_RUNNING)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[10] >= 0.95)
                    standby_left = 1;
            }
            else
                obs.observations[10] = 1;
            if (standby_right == 0)
            {
                obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[11] <= -0.95)
                    standby_right = 1;
            }
            else
                obs.observations[11] = -1;
        }
        // 0-2 base_ang_vel
        obs.observations.segment(0, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        Eigen::Vector3d v(0, 0, -1);
        Eigen::Vector4d qt(rbt_state.quat.x(), rbt_state.quat.y(), rbt_state.quat.z(), rbt_state.quat.w());
        // 3-5 projected_gravity
        obs.observations.segment(3, 3) = qr_ty_i(qt, v);
        // 6,7,8commands
        obs.observations[6] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[7] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[8] = command.dyaw * info.params.cmd_ang_vel_scale;
        // 9 standing_command_mask
        if (std::abs(obs.observations[6]) > 1e-9 || std::abs(obs.observations[7]) > 1e-9 || std::abs(obs.observations[8]) > 1e-9)
        {
            obs.observations[9] = 0.0;
        }
        else
        {
            obs.observations[9] = 0.0;
            // obs.observations[9] = 1.0;
        }

        obs.observations.segment(12, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(24, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(36, 12) = rbt_output.action;
#elif defined(PLATFORM_ARM)
        phase += 1 / full_step_period;

        if (ctrl_state == C_STATE_RUNNING)
        {
            //double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            //obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            //obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[0] = std::sin(2 * M_PI * phase);
            obs.observations[1] = std::cos(2 * M_PI * phase);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            //double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                //obs.observations[10] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                //if (obs.observations[10] >= 0.95)
                //    standby_left = 1;
                obs.observations[0] = std::sin(2 * M_PI * phase);
                if (obs.observations[0] >= 0.95)
                    standby_left = 1;
            }
            else{
                //obs.observations[10] = 1;
                obs.observations[0] = 1;
            }
            if (standby_right == 0)
            {
                //obs.observations[11] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                //if (obs.observations[11] <= -0.95)
                //    standby_right = 1;
                obs.observations[1] = std::cos(2 * M_PI * phase);
                if (obs.observations[1] <= -0.95)
                    standby_right = 1;
            }
            else{
                //obs.observations[11] = -1;
                obs.observations[1] = -1;
            }
        }
        // 0-2 base_ang_vel
        obs.observations[2] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[3] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[4] = command.dyaw * info.params.cmd_ang_vel_scale;
        // 6,7,8commands
        obs.observations.segment(5, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(17, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(29, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        obs.observations.segment(32, 3) = rbt_state.eu_ang;
#endif

        for (int i = 0; i < info.params.num_single_obs; ++i)
        {
            if (obs.observations[i] > info.params.clip_obs)
            {
                obs.observations[i] = info.params.clip_obs;
            }
            else if (obs.observations[i] < -info.params.clip_obs)
            {
                obs.observations[i] = -info.params.clip_obs;
            }
        }

        obs.hist_obs.push_back(obs.observations);
        obs.hist_obs.pop_front();
    }
    void RL::update_action_footstep(){
#ifdef PLATFORM_X86_64

        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        auto input_port = compiled_model.input();
        ov::Tensor input_tensor = infer_request.get_tensor(input_port);
        float *input_data = input_tensor.data<float>();
        for (size_t i = 0; i < input_tensor.get_size(); ++i)
        {
            input_data[i] = obs.input(i); // Example data
        }
        // double *data_ptr = obs.input.data();// 填充输入张量
        // std::memcpy(input_tensor.data<float>(), data_ptr, input_tensor.get_byte_size());
        infer_request.infer();// 执行推理
        auto output_port = compiled_model.output();// 获取输出张量
        ov::Tensor output_tensor = infer_request.get_tensor(output_port);

        // 处理输出数据
        const float *output_data = output_tensor.data<const float>();

        // std::cout << info.params.clip_actions_lower << std::endl;
        // std::cout << info.params.clip_actions_upper<< std::endl;
        for (int i = 0; i < info.params.num_actions; ++i)
        {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower_ov[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper_ov[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
        }
#elif defined(PLATFORM_ARM)
        // 准备输入数据
        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        std::vector<float> input_data(obs.input.size());
        for (size_t i = 0; i < obs.input.size(); ++i) {
            input_data[i] = obs.input(i); // 转换 obs.input 到向量
        }

        rknn_inputs[0].buf = input_data.data();

        int ret = rknn_inputs_set(ctx, io_num.n_input, rknn_inputs);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to set RKNN input!"  << ret << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_run(ctx, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to run RKNN inference!" << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_outputs_get(ctx, io_num.n_output, rknn_outputs, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to get RKNN output!" << std::endl;
            rknn_destroy(ctx);
        }
        
        float *output_data = static_cast<float *>(rknn_outputs[0].buf);
        std_msgs::Float64MultiArray policy_msg;
        for (int i = 0; i < info.params.num_actions; ++i) {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
            policy_msg.data.push_back(clamped_value);
        }
        action_pub_.publish(policy_msg);
#endif
    }

    void RL::update_observation_humanoidgym()
    {
        #ifdef PLATFORM_X86_64
        if (ctrl_state == C_STATE_RUNNING)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            obs.observations[0] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[1] = std::cos(2 * M_PI * time_diff / info.params.frequency);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                obs.observations[0] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[0] >= 0.95)
                    standby_left = 1;
            }
            else
                obs.observations[0] = 1;
            if (standby_right == 0)
            {
                obs.observations[1] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[1] <= -0.95)
                    standby_right = 1;
            }
            else
                obs.observations[1] = -1;
        }
        obs.observations[2] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[3] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[4] = command.dyaw * info.params.cmd_ang_vel_scale;
        obs.observations.segment(5, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(17, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(29, 12) = rbt_output.action;
        obs.observations.segment(41, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        obs.observations.segment(44, 3) = rbt_state.eu_ang;

        #elif defined(PLATFORM_ARM)
        if (ctrl_state == C_STATE_RUNNING)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            obs.observations[0] = std::sin(2 * M_PI * time_diff / info.params.frequency);
            obs.observations[1] = std::cos(2 * M_PI * time_diff / info.params.frequency);
        }
        else if (ctrl_state == C_STATE_STANDBY)
        {
            double time_diff = static_cast<double>((ros::Time::now() - start_time).toSec());
            if (standby_left == 0)
            {
                obs.observations[0] = std::sin(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[0] >= 0.95)
                    standby_left = 1;
            }
            else
                obs.observations[0] = 1;
            if (standby_right == 0)
            {
                obs.observations[1] = std::cos(2 * M_PI * time_diff / info.params.frequency);
                if (obs.observations[1] <= -0.95)
                    standby_right = 1;
            }
            else
                obs.observations[1] = -1;
        }
        obs.observations[2] = command.vx * info.params.cmd_lin_vel_scale;
        obs.observations[3] = command.vy * info.params.cmd_lin_vel_scale;
        obs.observations[4] = command.dyaw * info.params.cmd_ang_vel_scale;
        obs.observations.segment(5, 12) = rbt_state.q * info.params.rbt_lin_pos_scale;
        obs.observations.segment(17, 12) = rbt_state.dq * info.params.rbt_lin_vel_scale;
        obs.observations.segment(29, 12) = rbt_output.action;
        obs.observations.segment(41, 3) = rbt_state.base_ang_vel * info.params.rbt_ang_vel_scale;
        obs.observations.segment(44, 3) = rbt_state.eu_ang;
        #endif

        for (int i = 0; i < info.params.num_single_obs; ++i)
        {
            if (obs.observations[i] > info.params.clip_obs)
            {
                obs.observations[i] = info.params.clip_obs;
            }
            else if (obs.observations[i] < -info.params.clip_obs)
            {
                obs.observations[i] = -info.params.clip_obs;
            }
        }

        obs.hist_obs.push_back(obs.observations);
        obs.hist_obs.pop_front();
    }

    void RL::update_action_humanoidgym()
    {
        #ifdef PLATFORM_X86_64
        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        auto input_port = compiled_model.input();
        ov::Tensor input_tensor = infer_request.get_tensor(input_port);
        float *input_data = input_tensor.data<float>();

        for (size_t i = 0; i < input_tensor.get_size(); ++i)
        {
            input_data[i] = obs.input(i); // Example data
        }
        infer_request.infer();// 执行推理

        auto output_port = compiled_model.output();
        ov::Tensor output_tensor = infer_request.get_tensor(output_port);
        
        // 处理输出数据
        const float *output_data = output_tensor.data<const float>();

        for (int i = 0; i < info.params.num_actions; ++i)
        {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower_ov[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper_ov[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
        }
        
        #elif defined(PLATFORM_ARM)
        printf("FUCK IN TO ARM \r\n");
        // 准备输入数据
        for (int i = 0; i < info.params.frame_stack; ++i)
        {
            obs.input.block(0, i * info.params.num_single_obs, 1, info.params.num_single_obs) = obs.hist_obs[i].transpose();
        }

        // 获取输入张量
        std::vector<float> input_data(obs.input.size());
        for (size_t i = 0; i < obs.input.size(); ++i) {
            input_data[i] = obs.input(i); // 转换 obs.input 到向量
        }

        rknn_inputs[0].buf = input_data.data();

        int ret = rknn_inputs_set(ctx, io_num.n_input, rknn_inputs);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to set RKNN input!"  << ret << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_run(ctx, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to run RKNN inference!" << std::endl;
            rknn_destroy(ctx);
        }

        ret = rknn_outputs_get(ctx, io_num.n_output, rknn_outputs, nullptr);
        if (ret != RKNN_SUCC) {
            std::cerr << "Failed to get RKNN output!" << std::endl;
            rknn_destroy(ctx);
        }
        
        float *output_data = static_cast<float *>(rknn_outputs[0].buf);
        for (int i = 0; i < info.params.num_actions; ++i) {
            float lower_bound = static_cast<float>(info.params.clip_actions_lower[i]);
            float upper_bound = static_cast<float>(info.params.clip_actions_upper[i]);
            float clamped_value = std::clamp(output_data[i], lower_bound, upper_bound);
            rbt_output.action[i] = clamped_value;
            msg.rl2pd_msg.action[i] = clamped_value;
        }
        #endif
        printf("FUCK end \r\n");
    }

    void RL::read_orient()
    {
        rbt_state.quat.x() = msg.yesenseIMU.orientation.x;
        rbt_state.quat.y() = msg.yesenseIMU.orientation.y;
        rbt_state.quat.z() = msg.yesenseIMU.orientation.z;
        rbt_state.quat.w() = msg.yesenseIMU.orientation.w;
        quat2euler();
        rbt_state.base_ang_vel[0] = msg.yesenseIMU.angular_velocity.x;
        rbt_state.base_ang_vel[1] = msg.yesenseIMU.angular_velocity.y;
        rbt_state.base_ang_vel[2] = msg.yesenseIMU.angular_velocity.z;

        for (int i = 0; i < 3; ++i)
        {
            msg.ori_msg.eu_ang[i] = rbt_state.eu_ang[i];
            msg.ori_msg.base_ang_vel[i] = rbt_state.base_ang_vel[i];
        }
        pub.ori_pub_.publish(msg.ori_msg);
    }

    bool RL::exec_loop_is_terminate()
    {
        return quit.load();
    }
    void RL::collect_ctrl2usr_data()
    {
        msg.ctrl_usr_msg.robot_speed[0] = rbt_state.base_ang_vel[0];
        msg.ctrl_usr_msg.robot_speed[1] = command.vy;
        msg.ctrl_usr_msg.robot_speed[5] = command.dyaw;
        ;
        pub.ctrl_usr_pub_.publish(msg.ctrl_usr_msg);
    }
    void RL::exec_loop()
    {
        const int rl_ctrl_f = static_cast<int>(1 / (info.params.decimation * info.params.dt));
        ROS_INFO("rl_ctrl_f: %dHz", rl_ctrl_f);
        ros::Rate rate(rl_ctrl_f);
        while (!quit.load() && ros::ok())
        {
            switch (ctrl_state)
            {
            case C_STATE_SHUT_DOWN:
            case C_STATE_STANDBY:
            case C_STATE_RUNNING:
                read_orient();
                if( mode_ == DEFAULT_MODE )
                {
                    //update_observation_dreamwaq();
                    //update_action_dreamwaq();
                    update_observation_footstep();
                    update_action_footstep();
                }
                else if(mode_ == CUSTOM_MODE)
                {
                    update_observation_humanoidgym();
                    update_action_humanoidgym();
                }
                break;
            case C_STATE_FORCE_SHUT_DOWN:
                quit.store(true);
                break;

            case C_STATE_INIT:
                {
                    // 使用静态局部变量
                    //static bool init_timer_started = false;
                    static ros::Time init_start_time;

                    if (!init_timer_started)
                    {
                        init_start_time = ros::Time::now();
                        init_timer_started = true;
                        ROS_INFO("Initialization started.");
                    }
                    else
                    {
                        double elapsed_time = (ros::Time::now() - init_start_time).toSec();
                        if (elapsed_time >= 5.0)
                        {
                            ctrl_state = C_STATE_STANDBY;
                            msg.rl2pd_msg.ctrl_state = ctrl_state;

                            // 重置静态变量
                            init_timer_started = false;

                            ROS_INFO("Initialization complete. Switching to STANDBY mode.");
                        }
                    }
                    break;

                }

            case C_STATE_WAITING:

            default:
                break;
            }
            collect_ctrl2usr_data();
            pub.rl2pd_pub_.publish(msg.rl2pd_msg);
            rate.sleep();
        }
        init_timer_started = false;
        ROS_INFO("rl_controller exec loop terminate.");
    }

} // namespace hightorque
