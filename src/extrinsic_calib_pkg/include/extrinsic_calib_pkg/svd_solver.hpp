#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

namespace extrinsic_calib
{

inline void solveSVD(const std::vector<Eigen::Vector3d>& src,
                     const std::vector<Eigen::Vector3d>& dst,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& t)
{
    if (src.size() != dst.size() || src.size() < 3) {
        throw std::runtime_error(
            "solveSVD requires src/dst with same size and at least 3 points");
    }

    Eigen::Vector3d src_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d dst_mean = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < src.size(); ++i) {
        src_mean += src[i];
        dst_mean += dst[i];
    }

    src_mean /= static_cast<double>(src.size());
    dst_mean /= static_cast<double>(dst.size());

    Eigen::MatrixXd src_centered(3, src.size());
    Eigen::MatrixXd dst_centered(3, dst.size());

    for (size_t i = 0; i < src.size(); ++i) {
        src_centered.col(i) = src[i] - src_mean;
        dst_centered.col(i) = dst[i] - dst_mean;
    }

    Eigen::Matrix3d H = src_centered * dst_centered.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R = V * U.transpose();

    if (R.determinant() < 0.0) {
        V.col(2) *= -1.0;
        R = V * U.transpose();
    }

    t = dst_mean - R * src_mean;
}

}  // namespace extrinsic_calib