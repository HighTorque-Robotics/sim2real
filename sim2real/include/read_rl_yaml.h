#ifndef READ_RL_YAML_H
#define READ_RL_YAML_H

#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

struct RL_ModelParams{
    int num_actions;

    std::string policy_name;

    double dt;
    int decimation;
    int rl_spinner_thread_num;
    int torch_thread_num;

    int num_single_obs;

    int frame_stack;

    double frequency;
    double offsets;
    double durations;
    double swing_height;

    double clip_ankle;

    double cmd_lin_vel_scale;
    double cmd_ang_vel_scale;
    double rbt_lin_pos_scale;
    double rbt_lin_vel_scale;
    double rbt_ang_vel_scale;

    double clip_obs;
    std::vector<double> clip_actions_upper_ov;
    std::vector<double> clip_actions_lower_ov;
    std::vector<float> clip_actions_upper;
    std::vector<float> clip_actions_lower;
};

class RL_YamlInfo{
public:
    RL_YamlInfo() {
        ros::NodeHandle nh_param("~");
        nh_param.param<std::string>("config_file", CONFIG_PATH, "_config.yaml");
        nh_param.param<std::string>("policy_dir", CURRENT_POLICY, "policy_dir");
        
        read_yaml();
    }
    RL_YamlInfo(const std::string&  path) {
        read_yaml(path);
    }

    std::string policy_path;
    std::string load_path;
    RL_ModelParams params;
private:
    void read_yaml();
    void read_yaml(const std::string& path);
    std::string CONFIG_PATH;
    std::string CURRENT_POLICY;

};

#endif