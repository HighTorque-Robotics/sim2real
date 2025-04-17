#include "kinematics.h"

Eigen::Vector3d whole_body_kinematics(pinocchio::Model &model, pinocchio::Data &data, const std::string &end_effector_name, Eigen::VectorXd &q)
{
    pinocchio::forwardKinematics(model, data, q);
    updateFramePlacements(model, data);
    const int end_effector_id = model.getFrameId(end_effector_name);
    return data.oMf[end_effector_id].translation();
}

bool whole_body_inverse_kinematics(pinocchio::Model &model, pinocchio::Data &data, const std::string &end_effector_name, const pinocchio::SE3 &target_transform, Eigen::VectorXd &q)
{
    const int max_iter = 10000;
    const double eps = 1e-4;
    const int end_effector_id = model.getFrameId(end_effector_name);
    for (int i = 0; i < max_iter; ++i)
    {
        forwardKinematics(model, data, q);
        updateFramePlacement(model, data, end_effector_id);
        pinocchio::SE3 current_transform = data.oMf[end_effector_id];
        pinocchio::SE3 error_se3 = current_transform.actInv(target_transform);
        pinocchio::Motion error = log6(error_se3);

        if (error.toVector().norm() < eps)
            return true;

        pinocchio::Data::Matrix6x J(6, model.nv);
        computeFrameJacobian(model, data, q, end_effector_id, pinocchio::LOCAL, J);
        Eigen::JacobiSVD<pinocchio::Data::Matrix6x> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd dq = svd.solve(error.toVector());
        q = integrate(model, q, dq);
    }
    return false;
}

void knee_kinematics(double motor_hip, double motor_knee, double &theta_hip, double &theta_knee){
    theta_hip  = motor_hip;
    theta_knee = motor_knee ;
}

void knee_inverse_kinematics(double theta_hip, double theta_knee, double &motor_hip, double &motor_knee){
    motor_hip  = theta_hip;
    motor_knee = theta_knee ;
}

void ankle_kinematics(double theta_upper, double theta_lower, double &theta_p, double &theta_r){
   theta_p=theta_upper;
   theta_r=theta_lower;
}

void ankle_inverse_kinematics(double theta_p, double theta_r, double &theta_upper, double &theta_lower){
    theta_upper=theta_p;
    theta_lower=theta_r;
}

