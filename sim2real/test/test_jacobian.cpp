#include <gtest/gtest.h>
#include "jacobian.h"

// Test for ankle_J_calc function
TEST(JacobianTest, AnkleJCalc) {
    // Test case 1: Check if the function returns a valid matrix
    double theta_p = 0.5; // Example input
    double theta_r = 0.3; // Example input
    Eigen::Matrix<double, 2, 2> result = ankle_J_calc(theta_p, theta_r);

    // Check the dimensions of the result
    EXPECT_EQ(result.rows(), 2);
    EXPECT_EQ(result.cols(), 2);

    // Check if the values are finite
    for (int i = 0; i < result.rows(); ++i) {
        for (int j = 0; j < result.cols(); ++j) {
            EXPECT_TRUE(std::isfinite(result(i, j)));
        }
    }
}

// Test for ensureFullRank function
TEST(JacobianTest, EnsureFullRank) {
    // Test case 1: Matrix with full rank
    Eigen::Matrix<double, 2, 2> full_rank_matrix;
    full_rank_matrix << 1, 2, 3, 4;
    Eigen::Matrix<double, 2, 2> result = ensureFullRank(full_rank_matrix);

    auto rank = Eigen::FullPivLU<Eigen::Matrix<double, 2, 2 > >(result).rank();
    // Check if the rank is 2
    EXPECT_EQ(rank, 2);

    // Test case 2: Matrix with rank < 2
    Eigen::Matrix<double, 2, 2> low_rank_matrix;
    low_rank_matrix << 1, 2, 2, 4; // Rank-deficient matrix
    result = ensureFullRank(low_rank_matrix);

    auto rank_low = Eigen::FullPivLU<Eigen::Matrix<double, 2, 2 > >(result).rank();
    // Check if the rank is 2 after modification
    EXPECT_EQ(rank_low, 2);

    // Check if the result is close to the original matrix
    EXPECT_TRUE((result - low_rank_matrix).norm() > 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}