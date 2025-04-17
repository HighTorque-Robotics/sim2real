#include <fstream>
#include "read_rl_yaml.h"

//#define CONFIG_PATH CMAKE_CURRENT_SOURCE_DIR "/config.yaml"
//#define CURRENT_POLICY CMAKE_CURRENT_SOURCE_DIR "/policy"

template <typename T>
static std::vector<T> readVectorFromYaml(const YAML::Node& node){
    std::vector<T> values;
    for (const auto& val : node) {
        values.push_back(val.as<T>());
    }
    return values;
}


void RL_YamlInfo::read_yaml(const std::string& load_path){
    YAML::Node config;
    try {
        config = YAML::LoadFile(load_path+"/config.yaml");
    } catch (YAML::BadFile &e) {
        ROS_INFO("The file %s%s does not exist", load_path.c_str(),"/config.yaml");
        return;
    }

    this->params.num_actions = config["num_actions"].as<int>();

    this->params.policy_name = config["policy_name"].as<std::string>();

    this->params.dt = config["dt"].as<double>();
    this->params.decimation = config["decimation"].as<int>();
    this->params.rl_spinner_thread_num = config["rl_spinner_thread_num"].as<int>();
    this->params.torch_thread_num = config["torch_thread_num"].as<int>();

    this->params.num_single_obs = config["num_single_obs"].as<int>();

    this->params.frame_stack = config["frame_stack"].as<int>();

    this->params.frequency = config["frequency"].as<double>();

    this->params.cmd_lin_vel_scale = config["cmd_lin_vel_scale"].as<double>();
    this->params.cmd_ang_vel_scale = config["cmd_ang_vel_scale"].as<double>();
    this->params.rbt_lin_pos_scale = config["rbt_lin_pos_scale"].as<double>();
    this->params.rbt_lin_vel_scale = config["rbt_lin_vel_scale"].as<double>();
    this->params.rbt_ang_vel_scale = config["rbt_ang_vel_scale"].as<double>();

    this->params.clip_obs = config["clip_obs"].as<double>();
    #ifdef PLATFORM_X86_64
    this->params.clip_actions_upper_ov = readVectorFromYaml<double>(config["clip_actions_upper"]);
    this->params.clip_actions_lower_ov = readVectorFromYaml<double>(config["clip_actions_lower"]);
    #elif defined(PLATFORM_ARM)
    this->params.clip_actions_upper = readVectorFromYaml<float>(config["clip_actions_upper"]);
    this->params.clip_actions_lower = readVectorFromYaml<float>(config["clip_actions_lower"]);
    #endif
    policy_path = load_path + std::string("/policy/") + this->params.policy_name;
}

void RL_YamlInfo::read_yaml(){
    YAML::Node config;
    try {
        config = YAML::LoadFile(CONFIG_PATH);
    } catch (YAML::BadFile &e) {
        ROS_INFO("The file %s does not exist", CONFIG_PATH.c_str());
        return;
    }

    this->params.num_actions = config["num_actions"].as<int>();

    this->params.policy_name = config["policy_name"].as<std::string>();

    this->params.dt = config["dt"].as<double>();
    this->params.decimation = config["decimation"].as<int>();
    this->params.rl_spinner_thread_num = config["rl_spinner_thread_num"].as<int>();
    this->params.torch_thread_num = config["torch_thread_num"].as<int>();

    this->params.num_single_obs = config["num_single_obs"].as<int>();

    this->params.frame_stack = config["frame_stack"].as<int>();

    this->params.frequency = config["frequency"].as<double>();

    this->params.cmd_lin_vel_scale = config["cmd_lin_vel_scale"].as<double>();
    this->params.cmd_ang_vel_scale = config["cmd_ang_vel_scale"].as<double>();
    this->params.rbt_lin_pos_scale = config["rbt_lin_pos_scale"].as<double>();
    this->params.rbt_lin_vel_scale = config["rbt_lin_vel_scale"].as<double>();
    this->params.rbt_ang_vel_scale = config["rbt_ang_vel_scale"].as<double>();

    this->params.clip_obs = config["clip_obs"].as<double>();
    #ifdef PLATFORM_X86_64
    this->params.clip_actions_upper_ov = readVectorFromYaml<double>(config["clip_actions_upper"]);
    this->params.clip_actions_lower_ov = readVectorFromYaml<double>(config["clip_actions_lower"]);
    #elif defined(PLATFORM_ARM)
    this->params.clip_actions_upper = readVectorFromYaml<float>(config["clip_actions_upper"]);
    this->params.clip_actions_lower = readVectorFromYaml<float>(config["clip_actions_lower"]);
    #endif

    policy_path = CURRENT_POLICY + std::string("/") + this->params.policy_name;
}
