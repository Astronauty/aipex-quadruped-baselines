#pragma once
// leg_ik.hpp — minimal, production-friendly IK helper using Pinocchio.
// Tracks a swing-foot trajectory by velocity IK and integrates to get joint angles.
// - WORLD-frame linear control of the foot (position + feedforward velocity).
// - Damped least-squares pseudoinverse; select only the leg's joint columns.
// - Free-flyer OK: just exclude the first 6 base DoFs from 'actuatedCols'.
// Dependencies: Pinocchio, Eigen.

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/integrate.hpp>

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

namespace ik {

struct IKParams {
  double dt        { 0.002 }; // controller period [s]
  double Kp        { 50.0  }; // m/s per m (pos gain)
  double lambda    { 1e-3  }; // damping for pseudoinverse
  int    max_iters { 1     }; // usually 1 per control tick
  double tol_pos   { 1e-5  }; // early exit if |pos error| small
};

inline Eigen::MatrixXd selectColumns(const Eigen::Ref<const Eigen::MatrixXd>& J,
                                     const std::vector<int>& idx) {
  Eigen::MatrixXd Jr(J.rows(), (int)idx.size());
  for (int i = 0; i < (int)idx.size(); ++i) Jr.col(i) = J.col(idx[i]);
  return Jr;
}

// J# = Jᵀ (J Jᵀ + λ² I)⁻¹
inline Eigen::MatrixXd dampedPinv(const Eigen::Ref<const Eigen::MatrixXd>& J,
                                  double lambda) {
  const int m = (int)J.rows();
  Eigen::MatrixXd JJt = J * J.transpose();
  JJt.diagonal().array() += lambda * lambda;
  return J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(m, m));
}

class LegIK {
public:
  // model:     Pinocchio model (fixed or free-flyer)
  // foot_fid:  frame index of the foot (model.getFrameId("FR_foot"))
  // actuatedCols: indices of the *joint velocity* columns you allow the IK to use.
  //   - Fixed base: usually the 3 columns of that leg (or more).
  //   - Free-flyer: DO NOT include the first 6 base columns.
  LegIK(const pinocchio::Model& model,
        pinocchio::FrameIndex foot_fid,
        std::vector<int> actuatedCols)
  : model_(model),
    data_(model),
    fid_(foot_fid),
    cols_(std::move(actuatedCols))
  {
    if (fid_ >= model_.frames.size()) throw std::runtime_error("LegIK: bad foot frame id");
    if (cols_.empty())                 throw std::runtime_error("LegIK: actuatedCols empty");
    for (int c : cols_) if (c < 0 || c >= model_.nv) throw std::runtime_error("LegIK: bad column index");
  }

  // One control step: track desired pos/vel (WORLD) -> update q and qdot (full-size).
  // Only the actuatedCols are written in qdot; others remain 0.
  void trackStep(const Eigen::Vector3d& p_des_W,
                 const Eigen::Vector3d& v_des_W,
                 Eigen::VectorXd& q,           // size model.nq
                 Eigen::VectorXd& qdot,        // size model.nv
                 const IKParams& prm = IKParams())
  {
    if (q.size()    != model_.nq) throw std::runtime_error("LegIK::trackStep: q size != model.nq");
    if (qdot.size() != model_.nv) throw std::runtime_error("LegIK::trackStep: qdot size != model.nv");

    for (int it = 0; it < prm.max_iters; ++it) {
      // Kinematics + frame placements
      pinocchio::forwardKinematics(model_, data_, q, qdot);
      pinocchio::updateFramePlacements(model_, data_);
      const Eigen::Vector3d p_cur_W = data_.oMf[fid_].translation();

      // WORLD frame Jacobian (6 x nv) -> take linear part (bottom 3 rows)
      Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model_.nv);
      pinocchio::computeFrameJacobian(model_, data_, q, fid_, pinocchio::WORLD, J6);
      const Eigen::MatrixXd Jv = J6.bottomRows<3>();             // 3 x nv
      const Eigen::MatrixXd Jleg = selectColumns(Jv, cols_);     // 3 x na

      // Reference velocity with P on position error
      const Eigen::Vector3d v_ref = v_des_W + prm.Kp * (p_des_W - p_cur_W);

      // Solve least-squares for the leg's joint velocities
      const Eigen::VectorXd dq_leg = dampedPinv(Jleg, prm.lambda) * v_ref; // (na)

      // Scatter into full qdot (zero elsewhere)
      qdot.setZero();
      for (int i = 0; i < (int)cols_.size(); ++i) qdot[ cols_[i] ] = dq_leg[i];

      // Integrate one step (base stays fixed if you didn’t include its cols)
      q = pinocchio::integrate(model_, q, qdot * prm.dt);

      if ((p_des_W - p_cur_W).norm() < prm.tol_pos) break;
    }
  }

  // Convenience: position IK (no feedforward velocity). Small incremental integration.
  Eigen::VectorXd solvePosition(const Eigen::VectorXd& q_init,
                                const Eigen::Vector3d& p_des_W,
                                int max_iters = 50,
                                double lambda = 1e-3,
                                double tol = 1e-5)
  {
    Eigen::VectorXd q = q_init;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    for (int it = 0; it < max_iters; ++it) {
      pinocchio::forwardKinematics(model_, data_, q, v);
      pinocchio::updateFramePlacements(model_, data_);
      const Eigen::Vector3d p_cur_W = data_.oMf[fid_].translation();
      const Eigen::Vector3d e = p_des_W - p_cur_W;
      if (e.norm() < tol) break;

      Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model_.nv);
      pinocchio::computeFrameJacobian(model_, data_, q, fid_, pinocchio::WORLD, J6);
      const Eigen::MatrixXd Jv = J6.bottomRows<3>();
      const Eigen::MatrixXd Jleg = selectColumns(Jv, cols_);
      const Eigen::VectorXd dq_leg = dampedPinv(Jleg, lambda) * e;

      Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
      for (int i = 0; i < (int)cols_.size(); ++i) dq[ cols_[i] ] = dq_leg[i];
      q = pinocchio::integrate(model_, q, dq);
    }
    return q;
  }

  // Helper to build a contiguous set of columns
  static std::vector<int> colsRange(int start, int count) {
    std::vector<int> v; v.reserve(count);
    for (int i = 0; i < count; ++i) v.push_back(start + i);
    return v;
  }

private:
  const pinocchio::Model& model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex fid_;
  std::vector<int> cols_;
};

} // namespace ik
