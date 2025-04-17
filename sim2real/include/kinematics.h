#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <ros/ros.h>

bool whole_body_inverse_kinematics(pinocchio::Model &model, pinocchio::Data &data, const std::string &end_effector_name, const pinocchio::SE3 &target_transform, Eigen::VectorXd &q);
Eigen::Vector3d whole_body_kinematics(pinocchio::Model &model, pinocchio::Data &data, const std::string &end_effector_name, Eigen::VectorXd &q);
void knee_kinematics(double motor_hip, double motor_knee, double &theta_hip, double &theta_knee);
void knee_inverse_kinematics(double theta_hip, double theta_knee, double &motor_hip, double &motor_knee);
void ankle_kinematics(double theta_upper, double theta_lower, double &theta_p, double &theta_r);
void ankle_inverse_kinematics(double theta_p, double theta_r, double &theta_upper, double &theta_lower);

#endif