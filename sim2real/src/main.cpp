
#include "pd_controller.h"
#include "rl_controller.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "rl_pd_controller");
    std::shared_ptr<livelybot_serial::robot> rb_ptr=std::make_shared<livelybot_serial::robot>();
    hightorque::PD pd(rb_ptr);
    hightorque::RL rl;
    ros::spin();
    ROS_INFO("rl_pd_controller shutdown");
    return 0;
}