#include "rl_controller.h"  // 假设 RL 类在这个头文件中
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rl_controller.h"  // Assuming RL class is defined in this header
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sim2real_msg/PD2RL.h>
#include <sim2real_msg/RL2PD.h>
#include <geometry_msgs/Twist.h>
#include <hightorque_hardware_sdk/usr2ctrl.h>
#include <hightorque_hardware_sdk/ctrl2usr.h>

class RLControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Create an instance of RL
        rl_controller_ = std::make_shared<hightorque::RL>();

        // Set up publishers and subscribers for testing
        pd2rl_pub_ = nh.advertise<sim2real_msg::PD2RL>("pd2rl", 10);
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
        joy_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        usr2ctrl_pub_ = nh.advertise<hightorque_hardware_sdk::usr2ctrl>("usr2ctrl_data", 10);

        rl2pd_sub_ = nh.subscribe("rl2pd", 10, &RLControllerTest::rl2pdCallback, this);
        ctrl2usr_sub_ = nh.subscribe("ctrl2usr_data", 10, &RLControllerTest::ctrl2usrCallback, this);
    }

    void TearDown() override {
        rl_controller_.reset();
    }

    void rl2pdCallback(const sim2real_msg::RL2PD::ConstPtr& msg) {
        rl2pd_msg_ = *msg;
    }

    void ctrl2usrCallback(const hightorque_hardware_sdk::ctrl2usr::ConstPtr& msg) {
        ctrl2usr_msg_ = *msg;
    }

    std::shared_ptr<hightorque::RL> rl_controller_;
    ros::Publisher pd2rl_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher joy_pub_;
    ros::Publisher usr2ctrl_pub_;
    ros::Subscriber rl2pd_sub_;
    ros::Subscriber ctrl2usr_sub_;

    sim2real_msg::RL2PD rl2pd_msg_;
    hightorque_hardware_sdk::ctrl2usr ctrl2usr_msg_;
};

TEST_F(RLControllerTest, TestInitialization) {
    ASSERT_EQ(rl_controller_->ctrl_state, hightorque::C_STATE_WAITING);
    ASSERT_EQ(rl_controller_->msg.rl2pd_msg.ctrl_state, hightorque::C_STATE_WAITING);
}

TEST_F(RLControllerTest, TestPD2RLCallback) {
    sim2real_msg::PD2RL pd2rl_msg;
    pd2rl_msg.ctrl_state = hightorque::C_STATE_FORCE_SHUT_DOWN;
    pd2rl_msg.is_falldown = true;
    pd2rl_msg.rbt_state_q = std::vector<double>(rl_controller_->info.params.num_actions, 1.0);
    pd2rl_msg.rbt_state_dq = std::vector<double>(rl_controller_->info.params.num_actions, 0.5);

    pd2rl_pub_.publish(pd2rl_msg);
    ros::spinOnce();

    ASSERT_EQ(rl_controller_->ctrl_state, hightorque::C_STATE_STANDBY);
    ASSERT_EQ(rl_controller_->msg.rl2pd_msg.ctrl_state, hightorque::C_STATE_STANDBY);
    for (int i = 0; i < rl_controller_->info.params.num_actions; ++i) {
        ASSERT_EQ(rl_controller_->rbt_state.q[i], 1.0);
        ASSERT_EQ(rl_controller_->rbt_state.dq[i], 0.5);
    }
}

TEST_F(RLControllerTest, TestOdomCallback) {
    sensor_msgs::Imu imu_msg;
    imu_msg.orientation.x = 0.1;
    imu_msg.orientation.y = 0.2;
    imu_msg.orientation.z = 0.3;
    imu_msg.orientation.w = 0.4;

    imu_pub_.publish(imu_msg);
    ros::spinOnce();

    ASSERT_EQ(rl_controller_->msg.yesenseIMU.orientation.x, 0.1);
    ASSERT_EQ(rl_controller_->msg.yesenseIMU.orientation.y, 0.2);
    ASSERT_EQ(rl_controller_->msg.yesenseIMU.orientation.z, 0.3);
    ASSERT_EQ(rl_controller_->msg.yesenseIMU.orientation.w, 0.4);
}

TEST_F(RLControllerTest, TestJoyControlCallback) {
    geometry_msgs::Twist joy_msg;
    joy_msg.linear.x = 1.0;
    joy_msg.linear.y = 0.5;
    joy_msg.angular.z = 0.2;

    rl_controller_->ctrl_state = hightorque::C_STATE_RUNNING;
    joy_pub_.publish(joy_msg);
    ros::spinOnce();

    ASSERT_EQ(rl_controller_->command.vx, 1.0);
    ASSERT_EQ(rl_controller_->command.vy, 0.5);
    ASSERT_EQ(rl_controller_->command.dyaw, 0.2);
}

TEST_F(RLControllerTest, TestUsrControlCallback) {
    hightorque_hardware_sdk::usr2ctrl usr_msg;
    usr_msg.robot_status = 4;
    usr_msg.twist_data.linear.x = 1.0;
    usr_msg.twist_data.linear.y = 0.5;
    usr_msg.twist_data.angular.z = 0.2;

    usr2ctrl_pub_.publish(usr_msg);
    ros::spinOnce();

    ASSERT_EQ(rl_controller_->ctrl_state, hightorque::C_STATE_RUNNING);
    ASSERT_EQ(rl_controller_->command.vx, 1.0);
    ASSERT_EQ(rl_controller_->command.vy, 0.5);
    ASSERT_EQ(rl_controller_->command.dyaw, 0.2);
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_rl_controller");
    return RUN_ALL_TESTS();
}
