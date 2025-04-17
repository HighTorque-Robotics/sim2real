#ifndef READ_PD_YAML_H
#define READ_PD_YAML_H

#include <string>
#include <yaml-cpp/yaml.h>

struct PD_ModelParams{
    int num_actions;

    int pd_ctrl_f;
    int pd_spinner_thread_num;

    double action_scale;

    double shut_down_speed;

    std::vector<std::vector<int>> map_index;

    std::vector<std::vector<int>> motor_direction;
    std::vector<std::vector<double>> motor_lower_limit;
    std::vector<std::vector<double>> motor_upper_limit;

    std::vector<std::vector<double>> running_kp;
    std::vector<std::vector<double>> running_kd;

    std::vector<std::vector<double>> shut_down_kp;
    std::vector<std::vector<double>> shut_down_kd;
    
    std::vector<std::vector<double>> teach_kp;
    std::vector<std::vector<double>> teach_kd;

    std::vector<std::vector<double>> torque_limits;

    std::vector<std::vector<double>> urdf_dof_pos_offset;
    std::vector<std::string> joint_controller_names;

    double dynamic_offset_hip;
    double dynamic_offset_knee;
    double dynamic_offset_ankle;
    
    std::vector<std::string> waypoint_files;
};

class PD_YamlInfo{
public:
    PD_YamlInfo() {
        read_yaml();
        use_custom_path_=false;
    }

    PD_YamlInfo(const std::string&  path) {
        read_yaml(path);
        use_custom_path_=true;
        custom_path_=path;
    }

    
    void write_yaml();


    PD_ModelParams params;
private:
    bool use_custom_path_;
    std::string custom_path_;
    void read_yaml();
    void read_yaml(const std::string&  path);
    std::string DYNAMIC_CONFIG_PATH;
};

#endif