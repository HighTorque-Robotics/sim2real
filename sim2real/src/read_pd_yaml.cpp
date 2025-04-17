#include <fstream>
#include "read_pd_yaml.h"
#include <ros/ros.h>

//#define CONFIG_PATH CMAKE_CURRENT_SOURCE_DIR "/config.yaml"
//#define DYNAMIC_CONFIG_PATH CMAKE_CURRENT_SOURCE_DIR "/dynamic_config.yaml"

template <typename T>
static std::vector<T> readVectorFromYaml(const YAML::Node& node){
    std::vector<T> values;
    for (const auto& val : node) {
        values.push_back(val.as<T>());
    }
    return values;
}

void PD_YamlInfo::read_yaml(){
    YAML::Node config;
    ros::NodeHandle nh_param("~");
    std::string CONFIG_PATH = "";
    nh_param.param<std::string>("config_file", CONFIG_PATH, "_config.yaml");
    nh_param.param<std::string>("dynamic_config_file", DYNAMIC_CONFIG_PATH, "_dynamic_config.yaml");
    try {
        config = YAML::LoadFile(CONFIG_PATH);
    } catch (YAML::BadFile &e) {
        ROS_ERROR("The file %s does not exist", CONFIG_PATH.c_str());
        return;
    }
    YAML::Node dynamic_config;
    try {
        dynamic_config = YAML::LoadFile(DYNAMIC_CONFIG_PATH);
    } catch (YAML::BadFile &e) {
        ROS_ERROR("The file %s does not exist", DYNAMIC_CONFIG_PATH.c_str());
        return;
    }

    this->params.num_actions = config["num_actions"].as<int>();

    this->params.pd_ctrl_f = config["pd_ctrl_f"].as<int>();
    this->params.pd_spinner_thread_num = config["pd_spinner_thread_num"].as<int>();

    this->params.action_scale = config["action_scale"].as<double>();

    this->params.shut_down_speed = config["shut_down_speed"].as<double>();

    this->params.map_index = {readVectorFromYaml<int>(config["map_index"])};

    this->params.motor_direction = {readVectorFromYaml<int>(config["motor_direction"])};
    this->params.motor_lower_limit = {readVectorFromYaml<double>(config["motor_lower_limit"])};
    this->params.motor_upper_limit = {readVectorFromYaml<double>(config["motor_upper_limit"])};

    this->params.running_kp = {readVectorFromYaml<double>(config["running_kp"])};
    this->params.running_kd = {readVectorFromYaml<double>(config["running_kd"])};

    this->params.shut_down_kp = {readVectorFromYaml<double>(config["running_kp"])};
    this->params.shut_down_kd = {readVectorFromYaml<double>(config["running_kd"])};
    
    this->params.teach_kp = {readVectorFromYaml<double>(config["teach_kp"])};
    this->params.teach_kd = {readVectorFromYaml<double>(config["teach_kd"])};

    this->params.torque_limits = {readVectorFromYaml<double>(config["torque_limits"])};

    this->params.urdf_dof_pos_offset = {readVectorFromYaml<double>(config["urdf_dof_pos_offset"])};

    this->params.joint_controller_names = {readVectorFromYaml<std::string>(config["joint_controller_names"])};


    this->params.dynamic_offset_hip = dynamic_config["dynamic_offset_hip"].as<double>();
    this->params.dynamic_offset_knee = dynamic_config["dynamic_offset_knee"].as<double>();
    this->params.dynamic_offset_ankle = dynamic_config["dynamic_offset_ankle"].as<double>();

    if (config["waypoint_files"]) {
        this->params.waypoint_files = readVectorFromYaml<std::string>(config["waypoint_files"]);
    } else {
        ROS_WARN("No waypoint_files found in %s/config.yaml", CONFIG_PATH.c_str());
    }
}


void PD_YamlInfo::read_yaml(const std::string& path){
    YAML::Node config;
    try {
        config = YAML::LoadFile(path+"/config.yaml");
    } catch (YAML::BadFile &e) {
        ROS_ERROR("The file %s%s does not exist", path.c_str(),"/config.yaml");
        return;
    }
    YAML::Node dynamic_config;
    try {
        dynamic_config = YAML::LoadFile(path+"/dynamic_config.yaml");
    } catch (YAML::BadFile &e) {
        ROS_ERROR("The file %s%s does not exist", path.c_str(),"/dynamic_config.yaml");
        return;
    }

    this->params.num_actions = config["num_actions"].as<int>();

    this->params.pd_ctrl_f = config["pd_ctrl_f"].as<int>();
    this->params.pd_spinner_thread_num = config["pd_spinner_thread_num"].as<int>();

    this->params.action_scale = config["action_scale"].as<double>();

    this->params.shut_down_speed = config["shut_down_speed"].as<double>();

    this->params.map_index = {readVectorFromYaml<int>(config["map_index"])};

    this->params.motor_direction = {readVectorFromYaml<int>(config["motor_direction"])};
    this->params.motor_lower_limit = {readVectorFromYaml<double>(config["motor_lower_limit"])};
    this->params.motor_upper_limit = {readVectorFromYaml<double>(config["motor_upper_limit"])};

    this->params.running_kp = {readVectorFromYaml<double>(config["running_kp"])};
    this->params.running_kd = {readVectorFromYaml<double>(config["running_kd"])};

    this->params.shut_down_kp = {readVectorFromYaml<double>(config["running_kp"])};
    this->params.shut_down_kd = {readVectorFromYaml<double>(config["running_kd"])};

    this->params.torque_limits = {readVectorFromYaml<double>(config["torque_limits"])};

    this->params.urdf_dof_pos_offset = {readVectorFromYaml<double>(config["urdf_dof_pos_offset"])};

    this->params.joint_controller_names = {readVectorFromYaml<std::string>(config["joint_controller_names"])};

    this->params.dynamic_offset_hip = dynamic_config["dynamic_offset_hip"].as<double>();
    this->params.dynamic_offset_knee = dynamic_config["dynamic_offset_knee"].as<double>();
    this->params.dynamic_offset_ankle = dynamic_config["dynamic_offset_ankle"].as<double>();

    if (config["waypoint_files"]) {
        this->params.waypoint_files = readVectorFromYaml<std::string>(config["waypoint_files"]);
    } else {
        ROS_WARN("No waypoint_files found in %s/config.yaml", path.c_str());
    }
}






void PD_YamlInfo::write_yaml()
{
    YAML::Node dynamic_config;

    if (this->use_custom_path_)
    {
        try
        {
            dynamic_config = YAML::LoadFile(custom_path_+"/config.yaml");
        }
        catch (YAML::BadFile &e)
        {
            ROS_ERROR("The file %s%s does not exist", custom_path_.c_str(),"/config.yaml");
            return;
        }
        dynamic_config["dynamic_offset_hip"] = this->params.dynamic_offset_hip;
        dynamic_config["dynamic_offset_knee"] = this->params.dynamic_offset_knee;
        dynamic_config["dynamic_offset_ankle"] = this->params.dynamic_offset_ankle;

        std::ofstream out(custom_path_+"/dynamic_config.yaml");
        out << dynamic_config;
    }
    else
    {
        try
        {
            dynamic_config = YAML::LoadFile(DYNAMIC_CONFIG_PATH);
        }
        catch (YAML::BadFile &e)
        {
            ROS_ERROR("The file %s does not exist", DYNAMIC_CONFIG_PATH.c_str());
            return;
        }
        dynamic_config["dynamic_offset_hip"] = this->params.dynamic_offset_hip;
        dynamic_config["dynamic_offset_knee"] = this->params.dynamic_offset_knee;
        dynamic_config["dynamic_offset_ankle"] = this->params.dynamic_offset_ankle;

        std::ofstream out(DYNAMIC_CONFIG_PATH);
        out << dynamic_config;
    }
}
