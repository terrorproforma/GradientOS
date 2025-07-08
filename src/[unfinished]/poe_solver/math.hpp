#pragma once
#include <Eigen/Dense>

namespace ik::math {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W <<     0, -w.z(),  w.y(),
         w.z(),      0, -w.x(),
        -w.y(),  w.x(),      0;
    return W;
}

// se(3) exponential   S=[w;v] , θ in rad.
inline Eigen::Matrix4d expTwist(const Eigen::Matrix<double,6,1>& S, double th) {
    const Eigen::Vector3d w = S.head<3>();
    const Eigen::Vector3d v = S.tail<3>();
    const double wn = w.norm();

    Eigen::Matrix4d g = Eigen::Matrix4d::Identity();

    if (wn < 1e-12) { // prismatic
        g.block<3,1>(0,3) = v * th;
        return g;
    }

    const Eigen::Vector3d ŵ = w / wn;
    const double θ = th * wn;
    const Eigen::Matrix3d ωx = hat(ŵ);
    const Eigen::Matrix3d R  = Eigen::Matrix3d::Identity() +
                                sin(θ)*ωx + (1-cos(θ))*ωx*ωx;

    const Eigen::Matrix3d V  = (Eigen::Matrix3d::Identity() - R)*ωx + ŵ*ŵ.transpose()*θ;
    g.block<3,3>(0,0) = R;
    g.block<3,1>(0,3) = V * v / wn;
    return g;
}

inline Eigen::Matrix<double,6,6> adjoint(const Eigen::Matrix4d& g) {
    Eigen::Matrix<double,6,6> Ad = Eigen::Matrix<double,6,6>::Zero();
    const Eigen::Matrix3d R = g.block<3,3>(0,0);
    const Eigen::Vector3d p = g.block<3,1>(0,3);
    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,3) = R;
    Ad.block<3,3>(3,0) = hat(p) * R;
    return Ad;
}

} // namespace ik::math
