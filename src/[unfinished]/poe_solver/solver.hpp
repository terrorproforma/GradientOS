#pragma once
#include <array>
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "poe.hpp"          // your auto‑generated file
#include "math.hpp"

namespace ik {

struct Solution { std::array<double,N> th; };

class Spherical6DofSolver {
public:
    // Forward Kinematics
    static Eigen::Matrix4d fk(const std::array<double,N>& θ);

    // Spatial Jacobian (6×6)
    static Eigen::Matrix<double,6,N> jacobian(const std::array<double,N>& θ);

    // Analytic IK – returns 0‑to‑8 solutions, empty if unreachable.
    static std::vector<Solution> ik(const Eigen::Matrix4d& g06, double tol=1e-6);

private:
    // Pre‑computed screw matrix 6×6 and home pose
    static const Eigen::Matrix<double,6,N> S;
    static const Eigen::Matrix4d           M;

    // Link lengths used in IK
    static constexpr double l1 = POE[2][2] - POE[1][2];
    static constexpr double l2 = std::hypot(POE[3][0]-POE[2][0],
                                            POE[3][2]-POE[2][2]);
};

} // namespace ik
