#ifndef JACOBIAN_H
#define JACOBIAN_H

#include </usr/include/eigen3/Eigen/Dense>

Eigen::Matrix<double, 2, 2> ankle_J_calc(double theta_p, double theta_r);
Eigen::Matrix<double, 2, 2> ensureFullRank(const Eigen::Matrix<double, 2, 2>& matrix);

#endif