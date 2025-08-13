#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include "swing_trajectory.hpp"  

using swing::SwingLeg3DSpline;

static void dump_csv(const std::string &path,
                     const std::vector<double> &t,
                     const std::vector<Eigen::Vector3d> &p,
                     const std::vector<Eigen::Vector3d> &v,
                     const std::vector<Eigen::Vector3d> &a)
{
    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: could not open '" << path << "' for writing\n";
        return;
    }
    out << std::fixed << std::setprecision(6);
    out << "t,x,y,z,vx,vy,vz,ax,ay,az\n";
    for (size_t i = 0; i < t.size(); ++i) {
        out << t[i] << ","
            << p[i].x() << "," << p[i].y() << "," << p[i].z() << ","
            << v[i].x() << "," << v[i].y() << "," << v[i].z() << ","
            << a[i].x() << "," << a[i].y() << "," << a[i].z() << "\n";
    }
}

int main() {
    // start/end waypoints and timing
    const double t0 = 0.0;
    const double t2 = 0.60;               // 600 ms swing
    const Eigen::Vector3d P0(0.00, 0.00, 0.00);   // lift-off pose
    const Eigen::Vector3d P2(0.25, 0.00, 0.00);   // touch-down pose (25 cm forward)

    
    SwingLeg3DSpline traj;

    // clamped endpoints
    {
        SwingLeg3DSpline::BuildArgs args;
        args.policy       = SwingLeg3DSpline::BuildPolicy::AutoMidClamped;
        args.t0           = t0;
        args.t2           = t2;
        args.P0           = P0;
        args.P2           = P2;
        args.swing_height = 0.07;     // 7 cm clearance
        args.mid_tau      = 0.45;     // decides the how fast to reach the peak
        args.v0.setZero();            // clamp to zero vel at lift-off
        args.v2.setZero();            // clamp to zero vel at touch-down
        traj.build(args);
    }

    const double dt = 0.01;  // 10 ms
    std::vector<double> T; T.reserve((size_t)((t2-t0)/dt)+2);
    std::vector<Eigen::Vector3d> P, V, A; P.reserve(T.capacity()); V.reserve(T.capacity()); A.reserve(T.capacity());

    for (double t = t0; t <= t2 + 1e-12; t += dt) {
        Eigen::Vector3d p, v, a;
        traj.evaluate3(t, p, v, a);
        T.push_back(t);
        P.push_back(p);
        V.push_back(v);
        A.push_back(a);
    }

    // CSV
    dump_csv("trajectory.csv", T, P, V, A);
    std::cout << "Saved trajectory.csv with " << T.size() << " samples\n";

    // key values
    Eigen::Vector3d ps, vs, as, pe, ve, ae;
    traj.evaluate3(t0, ps, vs, as);
    traj.evaluate3(t2, pe, ve, ae);
    std::cout << "Start vel = " << vs.transpose() << "\n"
              << "End   vel = " << ve.transpose() << "\n";

    return 0;
}
