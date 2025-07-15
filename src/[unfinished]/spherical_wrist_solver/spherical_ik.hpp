// spherical_ik.hpp – analytic POE solver (Pieper: z2 ‖ z3, z1 ⟂ z2)
// ------------------------------------------------------------------
//  - Uses constants generated in ik_poe.hpp (POE, M_HOME, d6, N=6)
//  - Works for both X‑Y‑X (w4 ∥ w6) and X‑Y‑Z (w4 ⟂ w6) wrists.
//  - No numeric refinement step; 100 % closed‑form.
// ------------------------------------------------------------------
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include "ik_poe.hpp"
#include <iostream>

#ifndef M_PI
#   define M_PI 3.14159265358979323846
#endif

namespace ik {
using Vec6 = Eigen::Matrix<double,6,1>;
using Mat6 = Eigen::Matrix<double,6,6>;
using Mat4 = Eigen::Matrix4d;

/* -------- SE(3) exponential ------------------------------------- */
inline Mat4 exp6(const Eigen::Vector3d& w,
                 const Eigen::Vector3d& v,
                 double                 t)
{
    const double ct = std::cos(t), st = std::sin(t);

    Eigen::Matrix3d wx;
    wx << 0, -w.z(), w.y(),
          w.z(), 0, -w.x(),
         -w.y(), w.x(), 0;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = I + st*wx + (1-ct)*wx*wx;
    Eigen::Matrix3d V = I*t + (1-ct)*wx + (t-st)*wx*wx;

    Mat4 g = Mat4::Identity();
    g.topLeftCorner<3,3>() = R;
    g.topRightCorner<3,1>() = V * v;
    return g;
}

/* ---------------------------------------------------------------- */
struct Solution { Vec6 q; };

class SphericalIK
{
public:
    SphericalIK() {
        std::cout << "POE constants:" << std::endl;
        for(int i=0; i<N; ++i) {
            std::cout << "POE[" << i << "]: ";
            for(double val : POE[i]) std::cout << val << " ";
            std::cout << std::endl;
        }
        std::cout << "M_HOME:" << std::endl;
        for(double val : M_HOME) std::cout << val << " ";
        std::cout << std::endl;
    }

    /* FK ---------------------------------------------------------------- */
    Mat4 fk(const Vec6& q) const
    {
        Mat4 T = Mat4::Identity();
        for (int i=0;i<N;++i)
        {
            Eigen::Vector3d p{POE[i][0],POE[i][1],POE[i][2]};
            Eigen::Vector3d w{POE[i][3],POE[i][4],POE[i][5]};
            Eigen::Vector3d v = -w.cross(p);
            T *= exp6(w,v,q(i));
        }
        Mat4 M;
        std::copy(M_HOME.begin(),M_HOME.end(),M.data());
        return T*M;
    }

    /* Jacobian ---------------------------------------------------------- */
    Mat6 jacobian(const Vec6& q) const
    {
        Mat6 J;  Mat4 T = Mat4::Identity();
        for (int i=0;i<N;++i)
        {
            Eigen::Vector3d p{POE[i][0],POE[i][1],POE[i][2]};
            Eigen::Vector3d w{POE[i][3],POE[i][4],POE[i][5]};
            Eigen::Vector3d v = -w.cross(p);
            Eigen::Matrix<double,6,1> S;  S<<w,v;

            Eigen::Matrix3d R = T.topLeftCorner<3,3>();
            Eigen::Vector3d t = T.topRightCorner<3,1>();
            Eigen::Matrix3d tx;
            tx<<0,-t.z(),t.y(), t.z(),0,-t.x(), -t.y(),t.x(),0;

            Mat6 Ad = Mat6::Zero();
            Ad.topLeftCorner<3,3>()     = R;
            Ad.bottomRightCorner<3,3>() = R;
            Ad.topRightCorner<3,3>()    = tx*R;

            J.col(i)=Ad*S;
            T*=exp6(w,v,q(i));
        }
        return J;
    }

    /* IK ---------------------------------------------------------------- */
    std::vector<Solution>
    solve(const Eigen::Matrix3d& R06,
          const Eigen::Vector3d& p06) const
    {
        std::vector<Solution> sols;

        /* ---- constants from POE ------------------------------------- */
        Eigen::Vector3d p2{POE[1][0],POE[1][1],POE[1][2]};
        Eigen::Vector3d p3{POE[2][0],POE[2][1],POE[2][2]};
        Eigen::Vector3d p4{POE[3][0],POE[3][1],POE[3][2]};

        Eigen::Vector3d z1{POE[0][3],POE[0][4],POE[0][5]};
        Eigen::Vector3d z4{POE[3][3],POE[3][4],POE[3][5]};
        Eigen::Vector3d z5{POE[4][3],POE[4][4],POE[4][5]};
        Eigen::Vector3d z6{POE[5][3],POE[5][4],POE[5][5]};

        /* ---- wrist centre (generalized for arbitrary tool offset) --- */
        Mat4 M; std::copy(M_HOME.begin(), M_HOME.end(), M.data());
        Eigen::Matrix3d R_home = M.topLeftCorner<3,3>();
        Eigen::Vector3d tool_pos_home = M.topRightCorner<3,1>();
        Eigen::Vector3d p_wc_home{POE[5][0], POE[5][1], POE[5][2]};
        Eigen::Vector3d vec_tool = R_home.transpose() * (p_wc_home - tool_pos_home);
        Eigen::Vector3d p_wc = p06 + R06 * vec_tool;

        std::cout << "Target p06: " << p06.transpose() << std::endl;
        std::cout << "Computed p_wc: " << p_wc.transpose() << std::endl;

        /* ---- θ1 ------------------------------------------------------ */
        double theta1 = std::atan2(p_wc.y(),p_wc.x());
        std::cout << "theta1: " << theta1 << std::endl;

        /* ---- rotate into frame {1} ---------------------------------- */
        Eigen::Matrix3d R01 = Eigen::AngleAxisd(theta1,z1).toRotationMatrix();
        Eigen::Vector3d r1  = R01.transpose()*(p_wc - p2);
        double px=r1.x(), pz=r1.z();
        std::cout << "px, pz: " << px << " " << pz << std::endl;

        /* ---- elbow geometry ----------------------------------------- */
        double L2 = (p3-p2).norm();
        Eigen::Vector3d d34_1 = p4 - p3;  // FIXED: no rotation applied
        double a=d34_1.x(), b=d34_1.z();
        double L3 = std::hypot(a,b);
        double gamma = std::atan2(b,a);             // *** CHANGED ***

        double cosB = (px*px+pz*pz-L2*L2-L3*L3)/(2*L2*L3);
        std::cout << "L2, L3: " << L2 << " " << L3 << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        std::cout << "cosB: " << cosB << std::endl;

        if(std::fabs(cosB)>1.0) return sols;
        double sinB = std::sqrt(1.0-cosB*cosB);

        for(double sgn:{+1.0,-1.0})
        {
            double B      = sgn*std::atan2(sinB,cosB);
            double theta3 = B - gamma;              // *** CHANGED ***

            double theta2 = std::atan2(px,pz)       // *** CHANGED ***
                           - std::atan2(L3*std::sin(B),
                                        L2+L3*std::cos(B));
            std::cout << "sgn: " << sgn << " B: " << B << " theta3: " << theta3 << " theta2: " << theta2 << std::endl;

            /* ---- wrist ------------------------------------------------ */
            Vec6 q123; q123<<theta1,theta2,theta3,0,0,0;
            Eigen::Matrix3d R03 = fk(q123).topLeftCorner<3,3>();
            Eigen::Matrix3d R36 = R03.transpose()*R06;

            Eigen::Vector3d w4=z4.normalized(), w5=z5.normalized(), w6=z6.normalized();
            Eigen::Matrix3d E; E.col(0)=w4; E.col(1)=w5;
            Eigen::Vector3d wc=w4.cross(w5);

            auto emit=[&](double t4,double t5,double t6){
                Solution s; s.q<<theta1,theta2,theta3,t4,t5,t6; sols.push_back(s);};

            if(w4.cross(w6).norm()>1e-6)             /* XYZ wrist */
            {
                if(wc.squaredNorm()<1e-12) continue;
                wc.normalize(); if(wc.dot(w6)<0) wc=-wc; E.col(2)=wc;
                Eigen::Matrix3d Rstd=E.transpose()*R36*E;
                double pitch=std::asin(-Rstd(2,0)), eps=1e-8;

                if(std::fabs(std::cos(pitch))>eps){
                    emit(std::atan2(Rstd(2,1),Rstd(2,2)), pitch,
                         std::atan2(Rstd(1,0),Rstd(0,0)));
                    emit(std::atan2(-Rstd(2,1),-Rstd(2,2)), M_PI-pitch,
                         std::atan2(-Rstd(1,0),-Rstd(0,0)));
                }else{
                    emit(std::atan2(-Rstd(1,2),Rstd(1,1)), pitch, 0.0);
                }
            }
            else                                       /* X‑Y‑X wrist */
            {
                if(wc.squaredNorm()<1e-12) continue;
                wc.normalize(); E.col(2)=wc;
                Eigen::Matrix3d Rb=E.transpose()*R36*E;
                double t5=std::acos(std::clamp(Rb(0,0),-1.0,1.0)), eps=1e-8;

                if(std::fabs(std::sin(t5))>eps){
                    emit(std::atan2( Rb(1,0),-Rb(2,0)), t5,
                         std::atan2( Rb(0,1), Rb(0,2)));
                    emit(std::atan2(-Rb(1,0), Rb(2,0)), -t5,
                         std::atan2(-Rb(0,1),-Rb(0,2)));
                }else{
                    emit(0.0,0.0,std::atan2(-Rb(1,2),Rb(1,1)));
                }
            }
        }
        return sols;
    }

private:
    static constexpr int N = ik::N;
};
}   // namespace ik