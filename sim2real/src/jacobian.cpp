#include "jacobian.h"
#include <ros/ros.h>

Eigen::Matrix<double, 2, 2> ankle_J_calc(double theta_p, double theta_r){
    double d1 = 0.036;
    double d2 = 0.023;
    double L1 = 0.025;
    double h1 = 0.112;
    double a1 = 0.115;
    double h2 = 0.065;
    double a2 = 0.07;

    a2 = h2;
    a1 = h1;
    d2 = L1;

    double J[4];

    double J_tmp;
    double J_tmp_tmp;
    double a_tmp;
    double a_tmp_tmp;
    double a_tmp_tmp_tmp;
    double b_J_tmp;
    double b_J_tmp_tmp;
    double b_a_tmp;
    double b_a_tmp_tmp;
    double b_a_tmp_tmp_tmp;
    double c_J_tmp;
    double c_J_tmp_tmp;
    double c_a_tmp;
    double c_a_tmp_tmp;
    double c_a_tmp_tmp_tmp;
    double d_J_tmp_tmp;
    double d_a_tmp;
    double d_a_tmp_tmp;
    double d_a_tmp_tmp_tmp;
    double e_a_tmp;
    double e_a_tmp_tmp;
    double e_a_tmp_tmp_tmp;
    double f_a_tmp;
    double f_a_tmp_tmp;
    double f_a_tmp_tmp_tmp;
    double g_a_tmp;
    double g_a_tmp_tmp;
    double g_a_tmp_tmp_tmp;
    double h_a_tmp;
    double h_a_tmp_tmp_tmp;
    double i_a_tmp;
    double i_a_tmp_tmp_tmp;
    J_tmp_tmp = std::cos(theta_p);
    b_J_tmp_tmp = std::sin(theta_r);
    c_J_tmp_tmp = std::sin(theta_p);
    d_J_tmp_tmp = std::cos(theta_r);
    a_tmp_tmp = 2.0 * L1 * d2;
    b_a_tmp_tmp = 2.0 * L1 * h1;
    c_a_tmp_tmp = a_tmp_tmp * c_J_tmp_tmp;
    a_tmp_tmp_tmp = 2.0 * L1 * d1;
    d_a_tmp_tmp = a_tmp_tmp_tmp * b_J_tmp_tmp;
    a_tmp = (c_a_tmp_tmp - b_a_tmp_tmp) + d_a_tmp_tmp;
    b_a_tmp = 2.0 * (d1 * d1);
    b_a_tmp_tmp_tmp = L1 * L1;
    c_a_tmp_tmp_tmp = d2 * d2;
    d_a_tmp_tmp_tmp = b_a_tmp * d_J_tmp_tmp;
    e_a_tmp_tmp_tmp = 2.0 * d2 * h1;
    f_a_tmp_tmp_tmp = 2.0 * d1 * h1;
    e_a_tmp_tmp = ((((((b_a_tmp_tmp_tmp - a1 * a1) + b_a_tmp) + c_a_tmp_tmp_tmp) +
                    h1 * h1) -
                    d_a_tmp_tmp_tmp) -
                    e_a_tmp_tmp_tmp * c_J_tmp_tmp) -
                    f_a_tmp_tmp_tmp * b_J_tmp_tmp;
    g_a_tmp_tmp_tmp = 2.0 * d1 * d2;
    f_a_tmp_tmp = g_a_tmp_tmp_tmp * c_J_tmp_tmp * b_J_tmp_tmp;
    c_a_tmp = e_a_tmp_tmp + f_a_tmp_tmp;
    a_tmp_tmp *= J_tmp_tmp;
    d_a_tmp = (e_a_tmp_tmp + a_tmp_tmp) + f_a_tmp_tmp;
    e_a_tmp_tmp =
        4.0 * b_a_tmp_tmp_tmp * c_a_tmp_tmp_tmp * (J_tmp_tmp * J_tmp_tmp);
    g_a_tmp_tmp = std::sqrt((a_tmp * a_tmp - c_a_tmp * c_a_tmp) + e_a_tmp_tmp);
    e_a_tmp = ((g_a_tmp_tmp - b_a_tmp_tmp) + c_a_tmp_tmp) + d_a_tmp_tmp;
    b_a_tmp_tmp = 2.0 * L1 * h2;
    f_a_tmp = (b_a_tmp_tmp - c_a_tmp_tmp) + d_a_tmp_tmp;
    h_a_tmp_tmp_tmp = 2.0 * d2 * h2;
    i_a_tmp_tmp_tmp = 2.0 * d1 * h2;
    d_a_tmp_tmp_tmp =
        ((((((b_a_tmp_tmp_tmp - a2 * a2) + b_a_tmp) + c_a_tmp_tmp_tmp) +
            h2 * h2) -
            d_a_tmp_tmp_tmp) -
        h_a_tmp_tmp_tmp * c_J_tmp_tmp) +
        i_a_tmp_tmp_tmp * b_J_tmp_tmp;
    g_a_tmp = d_a_tmp_tmp_tmp - f_a_tmp_tmp;
    h_a_tmp = (d_a_tmp_tmp_tmp + a_tmp_tmp) - f_a_tmp_tmp;
    e_a_tmp_tmp =
        std::sqrt((f_a_tmp * f_a_tmp - g_a_tmp * g_a_tmp) + e_a_tmp_tmp);
    i_a_tmp = ((b_a_tmp_tmp - e_a_tmp_tmp) - c_a_tmp_tmp) + d_a_tmp_tmp;
    J_tmp = e_a_tmp_tmp_tmp * J_tmp_tmp;
    b_J_tmp = g_a_tmp_tmp_tmp * J_tmp_tmp * b_J_tmp_tmp;
    c_J_tmp = d_a_tmp * d_a_tmp;
    d_a_tmp_tmp = 2.0 * g_a_tmp_tmp;
    e_a_tmp_tmp_tmp = e_a_tmp * e_a_tmp / c_J_tmp + 1.0;
    g_a_tmp_tmp = 4.0 * L1 * d2 * J_tmp_tmp;
    b_a_tmp_tmp =
        8.0 * b_a_tmp_tmp_tmp * c_a_tmp_tmp_tmp * J_tmp_tmp * c_J_tmp_tmp;
    J[0] = 2.0 *
            ((((2.0 * (J_tmp - b_J_tmp) * c_a_tmp + g_a_tmp_tmp * a_tmp) -
                b_a_tmp_tmp) /
                d_a_tmp_tmp +
            a_tmp_tmp) /
                d_a_tmp +
            ((J_tmp + c_a_tmp_tmp) - b_J_tmp) * e_a_tmp / c_J_tmp) /
            e_a_tmp_tmp_tmp;
    b_J_tmp_tmp *= b_a_tmp;
    c_J_tmp_tmp *= g_a_tmp_tmp_tmp * d_J_tmp_tmp;
    J_tmp = (b_J_tmp_tmp - f_a_tmp_tmp_tmp * d_J_tmp_tmp) + c_J_tmp_tmp;
    f_a_tmp_tmp = 4.0 * L1 * d1 * d_J_tmp_tmp;
    d_a_tmp_tmp_tmp = a_tmp_tmp_tmp * d_J_tmp_tmp;
    J[2] = -(2.0 * (((2.0 * J_tmp * c_a_tmp - f_a_tmp_tmp * a_tmp) / d_a_tmp_tmp -
                    d_a_tmp_tmp_tmp) /
                        d_a_tmp +
                    J_tmp * e_a_tmp / c_J_tmp)) /
            e_a_tmp_tmp_tmp;
    J_tmp = h_a_tmp_tmp_tmp * J_tmp_tmp;
    c_J_tmp = h_a_tmp * h_a_tmp;
    d_a_tmp_tmp = 2.0 * e_a_tmp_tmp;
    e_a_tmp_tmp_tmp = i_a_tmp * i_a_tmp / c_J_tmp + 1.0;
    J[1] = 2.0 *
            ((((g_a_tmp_tmp * f_a_tmp - 2.0 * (J_tmp + b_J_tmp) * g_a_tmp) +
                b_a_tmp_tmp) /
                d_a_tmp_tmp -
            a_tmp_tmp) /
                h_a_tmp +
            ((J_tmp + c_a_tmp_tmp) + b_J_tmp) * i_a_tmp / c_J_tmp) /
            e_a_tmp_tmp_tmp;
    J_tmp = (b_J_tmp_tmp + i_a_tmp_tmp_tmp * d_J_tmp_tmp) - c_J_tmp_tmp;
    J[3] = 2.0 *
            (((2.0 * J_tmp * g_a_tmp - f_a_tmp_tmp * f_a_tmp) / d_a_tmp_tmp +
            d_a_tmp_tmp_tmp) /
                h_a_tmp -
            J_tmp * i_a_tmp / c_J_tmp) /
            e_a_tmp_tmp_tmp;
    Eigen::Matrix<double, 2, 2> J_return;
    J_return << J[0], J[2], J[1], J[3];
    return J_return;
}

Eigen::Matrix<double, 2, 2> ensureFullRank(const Eigen::Matrix<double, 2, 2>& matrix){
    Eigen::Matrix<double, 2, 2> modified_matrix = matrix;
    Eigen::Matrix<double, 2, 2> identity_matrix = Eigen::Matrix<double, 2, 2>::Identity();

    while (Eigen::FullPivLU<Eigen::Matrix<double, 2, 2>>(modified_matrix).rank() < 2) {
        modified_matrix += 0.01 * identity_matrix;
    }

    return modified_matrix;
}