#include "solver.hpp"
#include <cmath>

using namespace ik;
using ik::math::expTwist;
using ik::math::adjoint;

// ---- static data ----
const Eigen::Matrix<double,N,N> _dummy = Eigen::Matrix<double,N,N>::Zero(); // avoids ODR issues

const Eigen::Matrix<double,6,N> S = []{
    Eigen::Matrix<double,6,N> out;
    for (int i=0;i<N;++i) {
        Eigen::Vector3d q{POE[i][0], POE[i][1], POE[i][2]};
        Eigen::Vector3d w{POE[i][3], POE[i][4], POE[i][5]};
        Eigen::Vector3d v = -w.cross(q);
        out.col(i).head<3>() = w;
        out.col(i).tail<3>() = v;
    }
    return out;
}();

const Eigen::Matrix4d M = []{
    Eigen::Matrix4d m;
    int k=0;
    for (int r=0;r<4;++r)
        for (int c=0;c<4;++c)
            m(r,c) = M_HOME[k++];
    return m;
}();

// ---- FK ----
Eigen::Matrix4d Spherical6DofSolver::fk(const std::array<double,N>& θ) {
    Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
    for (int i=0;i<N;++i)
        g *= expTwist(S.col(i), θ[i]);
    return g * M;
}

// ---- Jacobian ----
Eigen::Matrix<double,6,N> Spherical6DofSolver::jacobian(const std::array<double,N>& θ) {
    Eigen::Matrix<double,6,N> J;
    Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
    for (int i=0;i<N;++i) {
        J.col(i) = adjoint(g) * S.col(i);
        g *= expTwist(S.col(i), θ[i]);
    }
    return J;
}

// ---- Analytic IK ----
std::vector<Solution> Spherical6DofSolver::ik(const Eigen::Matrix4d& g06, double tol) {
    std::vector<Solution> sols;

    const Eigen::Vector3d p06 = g06.block<3,1>(0,3);
    const Eigen::Matrix3d R06 = g06.block<3,3>(0,0);
    const Eigen::Vector3d z_ee = R06.col(0);            // eₓ of EE
    const Eigen::Vector3d p_wc = p06 - d6 * z_ee;       // wrist center

    // --- θ1 ---
    double θ1 = std::atan2(p_wc.y(), p_wc.x());

    // rotate into joint‑2 plane
    const Eigen::AngleAxisd Rz_negθ1(-θ1, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d pw = Rz_negθ1 * p_wc;

    const double s  =  pw.x();
    const double dz =  pw.z() - POE[1][2];

    const double d  = std::hypot(s, dz);

    // Reachability check
    const double cos3 = (l1*l1 + l2*l2 - d*d)/(2*l1*l2);
    if (cos3 < -1.0 - tol || cos3 > 1.0 + tol)
        return sols;                                // unreachable

    // Numerical clamp
    const double c3 = std::clamp(cos3, -1.0, 1.0);

    for (int sign3 : {+1,-1})  {
        const double θ3 = sign3 * std::acos(c3);
        const double s3 = std::sin(θ3);

        const double β  = std::atan2(l2*s3, l1 + l2*std::cos(θ3));
        const double α  = std::atan2(dz, s);
        const double θ2 = α - β;

        // T03
        Eigen::Matrix4d g03 = Eigen::Matrix4d::Identity();
        g03 *= expTwist(S.col(0), θ1);
        g03 *= expTwist(S.col(1), θ2);
        g03 *= expTwist(S.col(2), θ3);
        const Eigen::Matrix3d R03 = g03.block<3,3>(0,0);

        const Eigen::Matrix3d R36 = R03.transpose() * R06;

        // Extract θ4‑θ6 from Rx(θ4)Ry(θ5)Rx(θ6)
        const double θ5_ = std::atan2(std::hypot(R36(0,1), R36(0,2)), R36(0,0));

        for (int sign5 : {+1,-1}) {
            double θ5 = (sign5==1)? θ5_ : -θ5_;
            if (std::abs(std::sin(θ5)) < 1e-8) continue; // singular wrist

            const double θ4 = std::atan2(R36(1,0), -R36(2,0));
            const double θ6 = std::atan2(R36(0,1),  R36(0,2));

            Solution sol{{θ1, θ2, θ3, θ4, θ5, θ6}};
            sols.push_back(sol);
        }
    }
    return sols;
}
