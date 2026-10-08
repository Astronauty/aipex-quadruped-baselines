#pragma once

#include <Eigen/Dense>
#include <array>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

/**
 * @class LegKinematics
 * @brief Foot kinematics and leg dynamics for the Go2, with no optimiser attached.
 *
 * Exists so the fast swing-control loop can have its OWN Pinocchio model and
 * Data. The MPC's solve takes ~12 ms; if the swing loop shared a model with it,
 * every access would have to be serialised behind that solve by a mutex, and a
 * std::mutex makes the waiting thread stop dead rather than skip. That is
 * precisely what defeated an earlier multi-threaded attempt: the swing timer
 * blocked on the lock for the whole solve, so the starvation moved from the
 * thread to the mutex and STAND degraded to max |roll| 3.14 rad.
 *
 * Giving each loop its own instance removes the shared state entirely, so no
 * lock is needed between them and the two can genuinely run concurrently.
 *
 * Leg order is FL, FR, RL, RR throughout, matching the rest of the controller.
 */
class LegKinematics
{
public:
    /**
     * @param urdf_path Fixed-base 12-DoF Go2 URDF.
     * @param joint_armature Reflected rotor inertia per leg joint, kg*m^2. The
     *        MuJoCo model sets armature="0.01" on every leg joint and the URDF
     *        carries no such term, so a mass matrix built from the URDF alone
     *        understates the inertia the plant actually has -- measured 70% low
     *        in the operational-space inertia without it.
     */
    LegKinematics(const std::string& urdf_path, double joint_armature,
                  const rclcpp::Logger& logger)
        : joint_armature_(joint_armature), logger_(logger)
    {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        if (model_.nq != 12 || model_.nv != 12) {
            throw std::runtime_error(
                "LegKinematics expects a fixed-base 12-DoF model, got nq=" +
                std::to_string(model_.nq) + " nv=" + std::to_string(model_.nv));
        }

        static const std::array<std::array<const char*, 3>, 4> joint_names = {{
            {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
            {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
            {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}},
            {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}}}};
        static const std::array<const char*, 4> foot_names = {{
            "FL_foot", "FR_foot", "RL_foot", "RR_foot"}};

        for (int foot = 0; foot < 4; foot++) {
            for (int j = 0; j < 3; j++) {
                const std::string name = joint_names[foot][j];
                if (!model_.existJointName(name)) {
                    throw std::runtime_error("LegKinematics: URDF missing joint " + name);
                }
                const auto jid = model_.getJointId(name);
                qidx_[foot][j] = model_.joints[jid].idx_q();
                vidx_[foot][j] = model_.joints[jid].idx_v();
            }
            if (!model_.existFrame(foot_names[foot])) {
                throw std::runtime_error(
                    std::string("LegKinematics: URDF missing frame ") + foot_names[foot]);
            }
            foot_fid_[foot] = model_.getFrameId(foot_names[foot]);
        }

        q_ = Eigen::VectorXd::Zero(model_.nq);
        v_ = Eigen::VectorXd::Zero(model_.nv);

        // Body CoM in base frame, from the base link's inertial origin. Fixed by
        // the massless-leg assumption the controller is built on.
        com_B_ = model_.inertias[1].lever();
    }

    /** Joint angles, foot order FL/FR/RL/RR. Marks the FK cache stale. */
    void set_joint_angles(const Eigen::Matrix<double, 12, 1>& theta)
    {
        for (int foot = 0; foot < 4; foot++)
            for (int j = 0; j < 3; j++)
                q_[qidx_[foot][j]] = theta[foot * 3 + j];
        dirty_ = true;
    }

    /** Joint velocities, same order. Needed only by the dynamics terms. */
    void set_joint_velocities(const Eigen::Matrix<double, 12, 1>& theta_dot)
    {
        for (int foot = 0; foot < 4; foot++)
            for (int j = 0; j < 3; j++)
                v_[vidx_[foot][j]] = theta_dot[foot * 3 + j];
    }

    /** Foot positions relative to the body CoM, in base-frame axes. */
    Eigen::Matrix<double, 3, 4> foot_positions_body()
    {
        refresh();
        Eigen::Matrix<double, 3, 4> p;
        for (int foot = 0; foot < 4; foot++) {
            p.col(foot) = data_.oMf[foot_fid_[foot]].translation() - com_B_;
        }
        return p;
    }

    /**
     * Per-foot 3x3 linear Jacobian, LOCAL_WORLD_ALIGNED. On a fixed-base model
     * that is base-origin with base-frame axes, which is the frame
     * foot_positions_body() reports in -- so a Cartesian error formed from these
     * two is consistent, and J^T maps a base-frame force.
     */
    std::vector<Eigen::Matrix3d> foot_jacobians()
    {
        refresh();
        std::vector<Eigen::Matrix3d> J(4, Eigen::Matrix3d::Zero());
        Eigen::MatrixXd J6(6, model_.nv);
        for (int foot = 0; foot < 4; foot++) {
            J6.setZero();
            pinocchio::computeFrameJacobian(model_, data_, q_, foot_fid_[foot],
                                            pinocchio::LOCAL_WORLD_ALIGNED, J6);
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    J[foot](r, c) = J6(r, vidx_[foot][c]);
        }
        return J;
    }

    /** Base-origin to body-CoM offset, base-frame axes. Configuration independent. */
    const Eigen::Vector3d& com_offset_body() const { return com_B_; }

    /**
     * Operational-space inertia, Lambda_i = (J M^-1 J^T)^-1: the apparent mass
     * seen at the foot. Exact here because a fixed-base model's legs are
     * kinematically independent, so M is block diagonal with one 3x3 per leg.
     */
    Eigen::Matrix3d operational_space_inertia(int leg, const Eigen::Matrix3d& J_i)
    {
        refresh();
        pinocchio::crba(model_, data_, q_);
        data_.M.triangularView<Eigen::StrictlyLower>() =
            data_.M.transpose().triangularView<Eigen::StrictlyLower>();

        Eigen::Matrix3d M_i;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                M_i(r, c) = data_.M(vidx_[leg][r], vidx_[leg][c]);

        M_i.diagonal().array() += joint_armature_;

        // Guard the inverse: at full extension J loses rank and J M^-1 J^T becomes
        // ill-conditioned, where a raw inverse returns huge gains. Fall back to a
        // scalar inertia, which is the right order of magnitude and cannot blow up.
        const Eigen::Matrix3d JMinvJt = J_i * M_i.inverse() * J_i.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(JMinvJt);
        const double cond = svd.singularValues()(0) /
                            std::max(svd.singularValues()(2), 1e-12);
        if (!JMinvJt.allFinite() || cond > 1e6) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                "Leg %d near-singular (cond %.1e); scalar inertia fallback", leg, cond);
            return Eigen::Matrix3d::Identity() * M_i.diagonal().mean();
        }
        return JMinvJt.inverse();
    }

    /** C(q,qdot)qdot + G(q) for one leg, the bias torque of the paper's eq. (2). */
    Eigen::Vector3d gravity_coriolis(int leg)
    {
        refresh();
        const Eigen::VectorXd tau_bias = pinocchio::rnea(
            model_, data_, q_, v_, Eigen::VectorXd::Zero(model_.nv));
        Eigen::Vector3d out;
        for (int j = 0; j < 3; j++) out[j] = tau_bias[vidx_[leg][j]];
        return out;
    }

    /** Jdot*qdot for one foot, LOCAL_WORLD_ALIGNED, the eq. (2) correction. */
    Eigen::Vector3d jacobian_time_variation_term(int leg)
    {
        refresh();
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q_, v_);
        Eigen::Matrix<double, 6, Eigen::Dynamic> dJ(6, model_.nv);
        dJ.setZero();
        pinocchio::getFrameJacobianTimeVariation(
            model_, data_, foot_fid_[leg], pinocchio::LOCAL_WORLD_ALIGNED, dJ);

        Eigen::Vector3d out = Eigen::Vector3d::Zero();
        for (int j = 0; j < 3; j++) {
            const int v_idx = vidx_[leg][j];
            out += dJ.block<3, 1>(0, v_idx) * v_[v_idx];
        }
        return out;
    }

private:
    void refresh()
    {
        if (!dirty_) return;
        pinocchio::forwardKinematics(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);
        dirty_ = false;
    }

    pinocchio::Model model_;
    pinocchio::Data data_;

    std::array<std::array<int, 3>, 4> qidx_{};
    std::array<std::array<int, 3>, 4> vidx_{};
    std::array<pinocchio::FrameIndex, 4> foot_fid_{};

    Eigen::VectorXd q_, v_;
    Eigen::Vector3d com_B_ = Eigen::Vector3d::Zero();
    bool dirty_ = true;

    double joint_armature_;
    rclcpp::Logger logger_;
    std::shared_ptr<rclcpp::Clock> clock_ =
        std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
};


/**
 * @brief Swing-leg torques, the Cheetah 3 paper's eq. (1) with the eq. (2)
 *        feedforward, over a LegKinematics instance.
 *
 *   tau = J^T [ Kp (p_ref - p) + Kd (v_ref - v) ]            (feedback)
 *       + J^T Lambda (a_ref - Jdot qdot) + C qdot + G        (feedforward)
 *
 * Free function rather than a ConvexMPC method so the swing controller does not
 * need the optimiser at all; see LegKinematics for why that matters.
 *
 * Everything is in the BASE frame, which is what the Jacobian maps on this
 * fixed-base model. J^T takes a POSITIVE force: by virtual work tau = J^T f with
 * f the force to apply AT the foot. (The stance path negates because its input
 * is the force the ground exerts on the foot, so the leg must produce the
 * reaction -- the two cases are not analogous.)
 *
 * @param use_feedforward Add the eq. (2) term. Note Jdot*qdot uses the MEASURED
 *        joint velocity, so it is a feedback path despite sitting in the
 *        feedforward: it was measured at 20-45x the Lambda*a_ref term during a
 *        divergence, and is only safe with heavy damping and uniform sampling.
 */
inline Eigen::Vector3d swing_leg_torques(
    LegKinematics& kin,
    int leg,
    const Eigen::Matrix3d& J_i,
    const Eigen::Matrix3d& Kp, const Eigen::Matrix3d& Kd,
    const Eigen::Vector3d& p, const Eigen::Vector3d& v,
    const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref,
    const Eigen::Vector3d& a_ref,
    bool use_feedforward)
{
    const Eigen::Vector3d f_fb = Kp * (p_ref - p) + Kd * (v_ref - v);
    if (!use_feedforward) {
        return J_i.transpose() * f_fb;
    }

    const Eigen::Matrix3d Lambda = kin.operational_space_inertia(leg, J_i);
    const Eigen::Vector3d Jdot_qdot = kin.jacobian_time_variation_term(leg);
    return J_i.transpose() * (f_fb + Lambda * (a_ref - Jdot_qdot))
         + kin.gravity_coriolis(leg);
}


/**
 * @brief Swing torques with gains scheduled from Lambda, the paper's eq. (3).
 *
 *   Kp_ii = omega_i^2 Lambda_ii,  Kd_ii = 2 zeta omega_i Lambda_ii
 *
 * Lambda cancels in the closed loop, so omega and zeta become what you specify
 * rather than consequences of the gains and the pose. The paper notes a fixed Kp
 * lets the closed-loop natural frequency drift as the leg's apparent mass
 * changes with configuration; this holds it constant. It also derives the
 * per-axis gain ratio from physics instead of hand tuning.
 */
inline Eigen::Vector3d swing_leg_torques_scheduled(
    LegKinematics& kin,
    int leg,
    const Eigen::Matrix3d& J_i,
    const Eigen::Vector3d& omega, double zeta,
    const Eigen::Vector3d& p, const Eigen::Vector3d& v,
    const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref,
    const Eigen::Vector3d& a_ref,
    bool use_feedforward)
{
    const Eigen::Matrix3d Lambda = kin.operational_space_inertia(leg, J_i);

    Eigen::Vector3d kp, kd;
    for (int i = 0; i < 3; i++) {
        kp[i] = omega[i] * omega[i] * Lambda(i, i);
        kd[i] = 2.0 * zeta * omega[i] * Lambda(i, i);
    }

    const Eigen::Vector3d f_fb =
        kp.asDiagonal() * (p_ref - p) + kd.asDiagonal() * (v_ref - v);

    if (!use_feedforward) {
        return J_i.transpose() * f_fb;
    }
    const Eigen::Vector3d Jdot_qdot = kin.jacobian_time_variation_term(leg);
    return J_i.transpose() * (f_fb + Lambda * (a_ref - Jdot_qdot))
         + kin.gravity_coriolis(leg);
}
