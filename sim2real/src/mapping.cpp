#include "pd_controller.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "robot_data.h"
namespace hightorque{



void PD::mtr2rbt_map(){

    for (int i = 0; i < info.params.num_actions; i++) {
        rbt_state. q[ i] = mtr_state. q[ i] * info.params.motor_direction[0][i];
        rbt_state.dq[ i] = mtr_state.dq[ i] * info.params.motor_direction[0][i];
        rbt_state.q[i] -= info.params.urdf_dof_pos_offset[0][i];
    }
}





void PD::rbt2mtr_map(){
    //urdf_offset
    for (int i = 0; i < info.params.num_actions; i++) {
        rbt_output.target_q[i] += info.params.urdf_dof_pos_offset[0][i];
        mtr_output.target_q[i]=rbt_output.target_q[i] * info.params.motor_direction[0][i];
    }
}




}//namespace hightorque