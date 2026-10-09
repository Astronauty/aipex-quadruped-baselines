#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include <algorithm>

#include "convex_mpc/convex_mpc.hpp"

// using namespace std;
// using namespace Eigen;


/*
QP Form:
min 0.5 x^T P x + q^T x
*/

ConvexMPC::ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params, const rclcpp::Logger& logger)
    : mpc_params(mpc_params), 
    quad_params(quad_params),
    logger_(logger)
{
    // std::cout << "ConvexMPC constructor started" << std::endl;
    RCLCPP_INFO(logger, "ConvexMPC constructor started");

    RCLCPP_INFO(logger, "MPCParams: N_MPC: %d, N_CONTROLS: %d, N_STATES: %d, dt: %f",
        mpc_params.N_MPC, mpc_params.N_CONTROLS, mpc_params.N_STATES, mpc_params.dt);

    RCLCPP_INFO(logger, "QuadrupedParams: mass: %f, gravity: %f",
        quad_params.mass, quad_params.g);

    try
    {
        // Create an GRB environment & model
        env = make_unique<GRBEnv>(true);
        env->set("LogFile", "convex_mpc.log");
        env->set("OutputFlag", "0");
        env->start();
        model = make_unique<GRBModel>(*env);

        model->set(GRB_DoubleParam_FeasibilityTol, 1E-8);
        model->set(GRB_DoubleParam_OptimalityTol, 1E-8);

        // Initialize robot state
        x0 = VectorXd::Zero(mpc_params.N_STATES); // Initial state of the robot
        ground_reaction_forces = Matrix<double, 3, 4>::Zero(); // Initialize GRFs for the 4 feet of the quadruped robot
        X_ref = VectorXd::Ones(mpc_params.N_STATES*mpc_params.N_MPC); // Reference trajectory
        theta = VectorXd::Zero(12); // Initialize joint angles of the quadruped robot

        // Initialize pinocchio model & data (for foot jacobian computaiton)
        string urdf_path = PINOCCHIO_MODEL_DIR "/go2_description.urdf";
        pinocchio::urdf::buildModel(urdf_path, pinocchio_model);
        pinocchio_data = pinocchio::Data(pinocchio_model);

        resolve_model_indices();
        
        // Initialize state space prediction matrices
        // StateSpace quad_dss = get_default_dss_model(); // TODO: proper initialization, perhaps based on the initial state of robot?
        StateSpace quad_dss = quadruped_state_space_discrete(0, ground_reaction_forces, quad_params.inertiaTensor, quad_params.mass, mpc_params.dt);
        const auto t_dss1 = std::chrono::steady_clock::now();
    tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);
    const auto t_pred = std::chrono::steady_clock::now();

        // Initialize swing leg tracker gains
        Kp = Matrix3d::Identity() * 10.0; // Proportional gain for swing leg tracking
        Kd = Matrix3d::Identity() * 1; // Derivative gain for swing leg tracking

        // Formulate the QP 
        U = model->addVars(mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1), GRB_CONTINUOUS);
        cout << "Number of Decision Variables:" << mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1) << endl;

        Q_bar = compute_Q_bar(); // Diagonal block matrix of quadratic state cost for N_MPC steps
        R_bar = compute_R_bar(); // Diagonal block matrix of quadratic control cost for N_MPC steps

        // add_friction_cone_constraints(*model, U, 0.5);

        // Contact constraints are (re)applied by the node each control cycle via
        // set_contact_constraints(contact_states) before every solve, so there is
        // no contact state to apply here at construction time.

        this->solve_joint_torques();
    } 
    catch(GRBException e) 
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }

}

void ConvexMPC::resolve_model_indices()
{
    static const std::array<std::array<const char*, 3>, 4> joint_names_by_foot = {{
        {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
        {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
        {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}},
        {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}}
    }};

    static const std::array<const char*, 4> foot_names = {{
        "FL_foot", "FR_foot", "RL_foot", "RR_foot"
    }};

    
    for (int foot = 0; foot < 4; foot++)
    {
        for (int j = 0; j < 3; j++)
        {
            const std::string name = joint_names_by_foot[foot][j];

            if (!pinocchio_model.existJointName(name))
            {

                // Check if joint indices exist based on joint names and stash
                RCLCPP_FATAL(logger_, "URDF missing joint '%s' (foot %d, joint %d)", name.c_str(), foot, j);
                throw std::runtime_error("resolve_model_indices: missing joint " + name);
            }

            const pinocchio::JointIndex jid = pinocchio_model.getJointId(name);
            joint_qidx_[foot][j] = pinocchio_model.joints[jid].idx_q();
            joint_vidx_[foot][j] = pinocchio_model.joints[jid].idx_v();
        }

        // Check if foot frames exist based on foot names
        const std::string foot_name = foot_names[foot];
        if (!pinocchio_model.existFrame(foot_name))
        {
            RCLCPP_FATAL(logger_, "URDF missing frame '%s' (foot %d)", foot_name.c_str(), foot);
            throw std::runtime_error("resolve_model_indices: missing frame " + foot_name);
        }

        foot_fid_[foot] = pinocchio_model.getFrameId(foot_name);
    }


    if (pinocchio_model.nq != 12 || pinocchio_model.nv != 12)
    {
        RCLCPP_FATAL(logger_, "Expected fixed-base 12-DoF model for pinocchio, got nq=%d, nv=%d", pinocchio_model.nq, pinocchio_model.nv);
        throw std::runtime_error("resolve_model_indices: unexpected model dimensions");
    }
    

    this->q_pin_ = Eigen::VectorXd::Zero(pinocchio_model.nq);

    // Body CoM offset, taken once from the base link's inertial origin. Under the
    // massless-leg assumption this is the CoM the dynamics take moments about, and
    // it does not move with joint configuration. Joint 1 is the first (and here
    // only) body in a fixed-base model; its lever gives the inertial origin.
    com_B_ = pinocchio_model.inertias[1].lever();

    // Startup self-check: drive the model to the nominal stance (hip 0, thigh 0.9,
    // calf -1.8) and report where FK puts the feet. Expected, relative to the CoM,
    // is x ~ +0.19/+0.19/-0.19/-0.19 and y ~ +0.14/-0.14/+0.14/-0.14. If this line
    // disagrees, the model or the index resolution is wrong; if it agrees but the
    // live PB_ trace does not, the joint angles being fed in are wrong.
    {
        Eigen::VectorXd q_check = Eigen::VectorXd::Zero(pinocchio_model.nq);
        const double nominal[3] = {0.0, 0.9, -1.8};
        for (int foot = 0; foot < 4; foot++)
            for (int j = 0; j < 3; j++)
                q_check[joint_qidx_[foot][j]] = nominal[j];

        pinocchio::Data check_data(pinocchio_model);
        pinocchio::forwardKinematics(pinocchio_model, check_data, q_check);
        pinocchio::updateFramePlacements(pinocchio_model, check_data);

        const Vector3d com = pinocchio_model.inertias[1].lever();
        Vector3d f[4];
        for (int foot = 0; foot < 4; foot++)
            f[foot] = check_data.oMf[foot_fid_[foot]].translation() - com;

        RCLCPP_INFO(logger_,
            "FKCHECK nominal stance -> X %.4f %.4f %.4f %.4f | Y %.4f %.4f %.4f %.4f | Z %.4f %.4f %.4f %.4f",
            f[0].x(), f[1].x(), f[2].x(), f[3].x(),
            f[0].y(), f[1].y(), f[2].y(), f[3].y(),
            f[0].z(), f[1].z(), f[2].z(), f[3].z());

        // Jacobian consistency: compare the LOCAL_WORLD_ALIGNED frame Jacobian
        // against a finite difference of the same FK, at the same configuration.
        // This checks the frame decision end to end -- if the Jacobian is
        // expressed in a different frame than get_foot_positions_body() reports
        // in, the swing PD maps its Cartesian error onto the wrong joint torques
        // and the foot is driven along an axis the trajectory never commanded.
        {
            const double eps = 1e-6;
            double worst = 0.0;
            int worst_foot = -1, worst_joint = -1;

            for (int foot = 0; foot < 4; foot++)
            {
                Eigen::MatrixXd J6(6, pinocchio_model.nv);
                pinocchio::computeFrameJacobian(pinocchio_model, check_data, q_check,
                                                foot_fid_[foot],
                                                pinocchio::LOCAL_WORLD_ALIGNED, J6);

                for (int j = 0; j < 3; j++)
                {
                    Eigen::VectorXd q_pert = q_check;
                    q_pert[joint_qidx_[foot][j]] += eps;

                    pinocchio::Data fd_data(pinocchio_model);
                    pinocchio::forwardKinematics(pinocchio_model, fd_data, q_pert);
                    pinocchio::updateFramePlacements(pinocchio_model, fd_data);

                    const Vector3d fd =
                        (fd_data.oMf[foot_fid_[foot]].translation() -
                         check_data.oMf[foot_fid_[foot]].translation()) / eps;

                    const Vector3d jac = J6.block<3, 1>(0, joint_vidx_[foot][j]);
                    const double err = (fd - jac).norm();
                    if (err > worst) { worst = err; worst_foot = foot; worst_joint = j; }
                }
            }

            RCLCPP_INFO(logger_,
                "JACCHECK max |J_col - dFK/dq| = %.3e (foot %d joint %d); "
                "LOCAL_WORLD_ALIGNED %s consistent with get_foot_positions_body()",
                worst, worst_foot, worst_joint, worst < 1e-4 ? "IS" : "IS NOT");
        }
    }

    // Lambda / gravity sanity check at the nominal stance. Lambda is an apparent
    // mass seen at the foot, so its diagonal should be a few kg -- the same order
    // as one leg (measured 2.07 kg) -- symmetric and positive definite. A
    // hardcoded identity (the reason the previous
    // get_foot_operation_space_inertia_matrix was deleted) would read exactly
    // 1.0, and a singular-configuration blow-up would read something huge.
    {
        const double nominal[3] = {0.0, 0.9, -1.8};
        Vector<double, 12> q_nom;
        const Vector<double, 12> qd_zero = Vector<double, 12>::Zero();
        for (int leg = 0; leg < 4; leg++)
            for (int j = 0; j < 3; j++) q_nom[leg * 3 + j] = nominal[j];

        const Vector<double, 12> theta_saved = this->theta;
        update_joint_angles(q_nom);
        update_joint_velocities(qd_zero);
        const auto J_all = get_foot_jacobians();

        const Matrix3d L = get_foot_operational_space_inertia(0, J_all[0]);
        const Vector3d g0 = get_leg_gravity_coriolis(0);
        Eigen::SelfAdjointEigenSolver<Matrix3d> es(L);

        // Does calling crba / rnea / dJ corrupt the FK results that
        // get_foot_positions_body() and get_foot_jacobians() read from the SAME
        // pinocchio_data? All of these share one Data object, and several of them
        // write intermediate placement and velocity fields. If any of them
        // invalidates data.oMf or data.J, then the feedforward path would silently
        // poison the feedback path that runs after it in the same cycle -- which
        // would explain a divergence that only appears when the feedforward is on.
        {
            const double nominal2[3] = {0.0, 0.9, -1.8};
            Vector<double, 12> q_n, qd_n;
            for (int leg = 0; leg < 4; leg++)
                for (int j = 0; j < 3; j++) {
                    q_n[leg * 3 + j] = nominal2[j];
                    qd_n[leg * 3 + j] = 0.3;   // nonzero so rnea/dJ do real work
                }
            update_joint_angles(q_n);
            update_joint_velocities(qd_n);

            const Matrix<double, 3, 4> p_before = get_foot_positions_body();
            const Matrix3d J_before = get_foot_jacobians()[0];

            // Now run every dynamics call the feedforward path uses.
            (void)get_foot_operational_space_inertia(0, J_before);
            (void)get_leg_gravity_coriolis(0);
            (void)get_foot_jacobian_time_variation_term(0);

            // Re-read WITHOUT marking dirty, exactly as publish_cmd would.
            const Matrix<double, 3, 4> p_after = get_foot_positions_body();
            const Matrix3d J_after = get_foot_jacobians()[0];

            RCLCPP_INFO(logger_,
                "DATACHECK after crba/rnea/dJ: foot-pos drift %.3e m | jacobian drift %.3e"
                " | %s",
                (p_after - p_before).cwiseAbs().maxCoeff(),
                (J_after - J_before).cwiseAbs().maxCoeff(),
                ((p_after - p_before).cwiseAbs().maxCoeff() < 1e-12 &&
                 (J_after - J_before).cwiseAbs().maxCoeff() < 1e-12)
                    ? "CLEAN (shared Data is safe)" : "CORRUPTED (shared Data is NOT safe)");
        }

        // Jdot*qdot check against a finite difference of J. Pinocchio's dJ is a
        // different algorithm from our J, so this verifies the two are consistent
        // rather than merely both present.
        {
            Vector<double, 12> qd_test = Vector<double, 12>::Zero();
            qd_test[0] = -0.127; qd_test[1] = 0.007; qd_test[2] = -0.839;  // FL, fast late swing
            update_joint_velocities(qd_test);
            const Vector3d jdq = get_foot_jacobian_time_variation_term(0);

            // Finite difference: Jdot ~ (J(q + h*qdot) - J(q - h*qdot)) / (2h)
            const double h = 1e-6;
            const Vector<double, 12> q_base = this->theta;
            Vector<double, 12> qp = q_base, qm = q_base;
            for (int j = 0; j < 3; j++) { qp[j] += h * qd_test[j]; qm[j] -= h * qd_test[j]; }

            update_joint_angles(qp);
            const Matrix3d Jp = get_foot_jacobians()[0];
            update_joint_angles(qm);
            const Matrix3d Jm = get_foot_jacobians()[0];
            update_joint_angles(q_base);

            const Vector3d fd = ((Jp - Jm) / (2.0 * h)) * qd_test.head<3>();
            RCLCPP_INFO(logger_,
                "JDOTCHECK pinocchio %.5f %.5f %.5f | finite-diff %.5f %.5f %.5f | err %.2e",
                jdq[0], jdq[1], jdq[2], fd[0], fd[1], fd[2], (jdq - fd).norm());
        }

        RCLCPP_INFO(logger_,
            "LAMBDACHECK FL diag %.3f %.3f %.3f kg | eig %.3f %.3f %.3f"
            " | asym %.2e | gravity tau %.3f %.3f %.3f Nm",
            L(0,0), L(1,1), L(2,2),
            es.eigenvalues()(0), es.eigenvalues()(1), es.eigenvalues()(2),
            (L - L.transpose()).cwiseAbs().maxCoeff(),
            g0[0], g0[1], g0[2]);

        update_joint_angles(theta_saved);
    }

    RCLCPP_INFO(logger_, "Pinocchio indices resolved: nq=%d, nv= %d, nframes=%d, com_B=(%.5f, %.5f, %.5f)",
                pinocchio_model.nq, pinocchio_model.nv, static_cast<int>(pinocchio_model.nframes),
                com_B_(0), com_B_(1), com_B_(2));
}

void print_eigen_matrix(const Eigen::MatrixXd& mat, string name, const rclcpp::Logger& logger) 
{
    RCLCPP_INFO(logger, "%s size: %ld x %ld", name.c_str(), mat.rows(), mat.cols());
    std::stringstream ss;
    ss << "\n";
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            ss << std::fixed << std::setprecision(3) << mat(i, j);
            if (j < mat.cols() - 1) ss << " ";
        }
        ss << "\n";
    }
    RCLCPP_INFO(logger, "%s: %s", name.c_str(), ss.str().c_str());
}

// ConvexMPC::~ConvexMPC() {
//     if (U != nullptr) {
//         delete[] U; // Free the memory allocated for U
//         U = nullptr; // Avoid dangling pointer
//     }
// }


/**
 * @brief Creates state prediction matrices for Convex MPC.
 * 
 * This function generates the state prediction matrices A_qp and B_qp based on the given discrete state space 
 * representation of the quadruped and the specified MPC horizon length. These can be used to generate a vector of 
 * the predicted states X for the next N_MPC timesteps, where U = [u_1... u_N-1] and x0 is the current state:
 * X = A_qp * x0 + B_qp * U
 *
 * @param quad_dss The discrete state space representation of the quadruped.
 * @param N_MPC Number of steps for the horizon length in MPC.
 * @return A tuple containing two Eigen::MatrixXd objects representing the state evolution matrices.
 */
tuple<MatrixXd, MatrixXd> ConvexMPC::create_state_space_prediction_matrices(const StateSpace& quad_dss)
{
    const int& N_STATES = this->mpc_params.N_STATES;
    const int& N_CONTROLS = this->mpc_params.N_CONTROLS;
    const int& N_MPC = this->mpc_params.N_MPC;

    MatrixXd A_qp = MatrixXd::Zero(N_STATES * N_MPC, N_STATES); 
    MatrixXd B_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * (N_MPC-1));

    // Precompute A^k once for k = 0..N_MPC. The previous version called A.pow() inside
    // the double loop, recomputing the same matrix powers O(N^2) times per solve.
    std::vector<MatrixXd> A_pow;
    A_pow.reserve(N_MPC + 1);
    A_pow.push_back(MatrixXd::Identity(N_STATES, N_STATES)); // A^0
    for (int k = 1; k <= N_MPC; ++k) {
        A_pow.push_back(A_pow[k - 1] * quad_dss.A);
    }

    // Precompute A^k * B as well, since each is reused across the diagonal of B_qp.
    std::vector<MatrixXd> A_pow_B;
    A_pow_B.reserve(N_MPC);
    for (int k = 0; k < N_MPC; ++k) {
        A_pow_B.push_back(A_pow[k] * quad_dss.B);
    }

    for (int i = 0; i < mpc_params.N_MPC; i++)
    {
        for (int j = 0; j < i; j++)
        {
            B_qp.block(i * N_STATES, j * N_CONTROLS, N_STATES, N_CONTROLS) = A_pow_B[i - j];
        }
        A_qp.block(i*N_STATES, 0, N_STATES, N_STATES) = A_pow[i + 1];
    }

    // cout << "state space predict success " << endl;
    // cout << "\n" << endl;

    return std::make_tuple(A_qp, B_qp);
}

StateSpace ConvexMPC::get_default_dss_model()
{
    Matrix<double, 3, 4> foot_positions;
    foot_positions << 0.2, 0.2, -0.2, -0.2,
                     0.15, -0.15, 0.15, -0.15,
                     0.0, 0.0, 0.0, 0.0; // Default foot positions in the body frame

    double yaw = 0.0f;
    StateSpace quad_dss = quadruped_state_space_discrete(yaw, foot_positions, quad_params.inertiaTensor, quad_params.mass, mpc_params.dt);

    return quad_dss;
    
}

MatrixXd blkdiag(const std::vector<MatrixXd>& matrices) {
    // Calculate the total size of the block diagonal matrix
    int totalRows = 0, totalCols = 0;
    for (const auto& mat : matrices) {
        // cout << "Matrix dimensions: " << mat.rows() << " x " << mat.cols() << endl;
        totalRows += mat.rows();
        totalCols += mat.cols();
    }
    // cout << "Total Rows " << totalRows << endl;
    // cout << "Total Cols " << totalCols << endl;
    // Create the block diagonal matrix
    MatrixXd blockDiagonal = MatrixXd::Zero(totalRows, totalCols);

    // Place each matrix along the diagonal
    int currentRow = 0, currentCol = 0;
    for (const auto& mat : matrices) {
        blockDiagonal.block(currentRow, currentCol, mat.rows(), mat.cols()) = mat;
        currentRow += mat.rows();
        currentCol += mat.cols();
    }
    // cout << "Block diagonal matrix dimensions: " << blockDiagonal.rows() << " x " << blockDiagonal.cols() << endl;
    return blockDiagonal;
    // return MatrixXd::Identity(totalRows, totalCols);
}

MatrixXd ConvexMPC::compute_R_bar()
{
    MatrixXd R = mpc_params.R;
    // int N_CONTROLS = mpc_params.N_CONTROLS;
    int N_MPC = mpc_params.N_MPC;

    std::vector<MatrixXd> R_vec(N_MPC-1, R);
    MatrixXd R_bar = blkdiag(R_vec);

    return R_bar;
}

MatrixXd ConvexMPC::compute_Q_bar()
{
    MatrixXd Q = mpc_params.Q;
    int N_MPC = mpc_params.N_MPC;

    std::vector<MatrixXd> Q_vec(N_MPC, Q);
    Q_vec[N_MPC - 1] = mpc_params.Q_n_scale * Q;

    // std::cout << "Q vec:\n" << Q_vec[0] << std::endl;
    MatrixXd Q_bar = blkdiag(Q_vec);

    // std::cout << "Q_bar:\n" << Q_bar << std::endl;
    // std::stringstream ss;
    // ss << "Q_bar:\n" << Q_bar;
    // RCLCPP_INFO(logger_, "%s", ss.str().c_str());


    return Q_bar;
}


/*
Computes the quadratic cost fo the unconstrained linear mpc problem
P = 2*R_bar + 2*B_qp'*Q_bar*B_qp
*/
MatrixXd ConvexMPC::compute_P(const MatrixXd& R_bar, const MatrixXd& Q_bar, const MatrixXd& B_qp, double regularization)
{
    // RCLCPP_INFO(logger_, "Computing P matrix for Convex MPC.");
    assert(R_bar.rows() == R_bar.cols() && "R_bar must be square.");
    assert(Q_bar.rows() == Q_bar.cols() && "Q_bar must be square.");
    assert(B_qp.cols() == R_bar.rows() && "B_qp and R_bar dimensions must match.");

    // Q_bar is block diagonal (one N_STATES x N_STATES block per horizon step), so
    // B_qp' * Q_bar * B_qp is accumulated blockwise rather than as two dense
    // products against the full N_STATES*N_MPC square matrix. Mathematically
    // identical, but avoids the dominant cost in the solve path.
    const int NS = mpc_params.N_STATES;
    const int N_MPC = mpc_params.N_MPC;

    // Each Q block is itself diagonal (Q is built from Q_diag), so Qk*Bk is a row
    // scaling rather than a dense NS x NS multiply. Written as a plain
    // Bk.transpose() * (Qk * Bk) over dense blocks this measured ~17ms per solve,
    // which alone exceeded two thirds of the 25ms control period and starved the
    // swing-PD timer sharing the executor thread.
    //
    // B_qp is block lower triangular -- row block k has nonzeros only in the
    // first k*N_CONTROLS columns -- so the structurally zero trailing columns are
    // skipped instead of being multiplied as dense zeros.
    // The row-scaling shortcut is only valid while Q is diagonal, which it is:
    // the node builds it as Q_diag.asDiagonal(). Checked rather than assumed, so
    // a future off-diagonal Q fails loudly instead of being silently ignored.
    assert((Q_bar - MatrixXd(Q_bar.diagonal().asDiagonal())).cwiseAbs().maxCoeff() < 1e-12
           && "compute_P assumes a diagonal Q_bar");

    const Eigen::VectorXd q_diag = Q_bar.diagonal();
    MatrixXd BtQB = MatrixXd::Zero(B_qp.cols(), B_qp.cols());
    for (int k = 0; k < N_MPC; ++k)
    {
        const int n_active = std::min<int>(B_qp.cols(), k * mpc_params.N_CONTROLS);
        if (n_active == 0) continue; // Row block 0 of B_qp is all zeros

        const auto Bk = B_qp.block(k * NS, 0, NS, n_active);        // NS x n_active
        const auto wk = q_diag.segment(k * NS, NS);                 // Diagonal of Qk

        // Qk * Bk as a row scaling, then one dense n_active x n_active product.
        const MatrixXd WB = wk.asDiagonal() * Bk;
        BtQB.topLeftCorner(n_active, n_active).noalias() += Bk.transpose() * WB;
    }

    MatrixXd P = 2*R_bar + 2*BtQB;
    P.diagonal().array() += regularization; // Regularization term to avoid singularity

    return P;
}

VectorXd ConvexMPC::compute_q(const MatrixXd& Q_bar, const MatrixXd& A_qp, const MatrixXd& B_qp, const VectorXd& x0, const VectorXd& X_ref)
{
    // VectorXd q = 2*x0.transpose() * B_qp * Q_bar.transpose() * A_qp; // Zero reference linear cost
    // VectorXd q = 2*(x0.transpose() * A_qp.transpose() * Q_bar * B_qp - X_ref.transpose() * Q_bar * B_qp); // Old
    // Parenthesised and accumulated blockwise. Written as
    // 2*B_qp.transpose() * Q_bar * (A_qp*x0 - X_ref), Eigen forms the dense
    // product B_qp^T * Q_bar first -- a 108x130 by 130x130 multiply, ~1.8M flops,
    // measured at ~17ms per solve and by itself a third of the control period.
    // Q_bar is block diagonal, so weighting the error blockwise and then applying
    // B_qp^T as a single matrix-vector product is mathematically identical and
    // costs ~14k flops. Same reasoning as compute_P above.
    const int NS = mpc_params.N_STATES;
    const int N_MPC = mpc_params.N_MPC;

    const VectorXd err = A_qp * x0 - X_ref;        // (NS*N_MPC) x 1
    VectorXd Qerr(err.size());
    for (int k = 0; k < N_MPC; ++k)
    {
        Qerr.segment(k * NS, NS).noalias() =
            Q_bar.block(k * NS, k * NS, NS, NS) * err.segment(k * NS, NS);
    }

    VectorXd q = 2.0 * (B_qp.transpose() * Qerr);

    // Centring the force regularizer on the gravity-compensating distribution
    // (q -= 2*R_bar*U_nom, which leaves P unchanged) was tried here to remove the
    // standing height bias, on the theory that charging R*|mg|^2 forever for
    // simply standing let the solver trade height error for force effort. It made
    // no difference: the z trajectory matched the plain ||U||^2 form to within
    // 0.3 mm at every point over 400 s. The apparent dependence of the settled
    // height on R was a convergence-rate effect, not a change of equilibrium.
    return q;
}

void ConvexMPC::update_x0(Vector<double, 13> x0)
{
    // Update the initial state vector x0
    this->x0 = x0;
    this->q = compute_q(Q_bar, A_qp, B_qp, this->x0, X_ref);

}

void ConvexMPC::update_joint_angles(Vector<double, 12> theta)
{
    this->theta = theta;
    this->foot_kinematics_dirty_ = true;

}

void ConvexMPC::update_foot_positions(const Matrix<double, 3, 4>& foot_positions_world)
{
    // Update the foot positions in the body frame
    if (mpc_params.verbose_diagnostics)
    {
        RCLCPP_INFO(logger_, "Updating foot positions in ConvexMPC.");
        RCLCPP_INFO(logger_, "Foot positions: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
            foot_positions_world(0, 0), foot_positions_world(0, 1), foot_positions_world(0, 2), foot_positions_world(0, 3),
            foot_positions_world(1, 0), foot_positions_world(1, 1), foot_positions_world(1, 2), foot_positions_world(1, 3),
            foot_positions_world(2, 0), foot_positions_world(2, 1), foot_positions_world(2, 2), foot_positions_world(2, 3));
    }

    this->foot_positions_world = foot_positions_world;
}

void ConvexMPC::update_reference_trajectory(const VectorXd& X_ref)
{
    if (X_ref.size() != mpc_params.N_STATES * mpc_params.N_MPC) {
        RCLCPP_ERROR(logger_, "Reference trajectory size mismatch: expected %d, got %ld",
                     mpc_params.N_STATES * mpc_params.N_MPC, X_ref.size());
        throw std::invalid_argument("Reference trajectory size mismatch");
    }
    // RCLCPP_INFO(logger_, "Updating reference trajectory in ConvexMPC.");
    // RCLCPP_INFO(logger_, "Reference trajectory size: %ld", X_ref.size());
    // RCLCPP_INFO(logger_, "Reference trajectory: \n[%f %f %f %f %f %f %f %f %f %f %f %f]",
    //     X_ref(0), X_ref(1), X_ref(2), X_ref(3), X_ref(4), X_ref(5), 
    //     X_ref(6), X_ref(7), X_ref(8), X_ref(9), X_ref(10), X_ref(11));
    
    this->X_ref = X_ref;
}

std::vector<Matrix<double, 3, 4>> ConvexMPC::grf_from_mpc_solution()
{
    // Each timestep has 12 controls (3 per foot, 4 feet)
    int num_timesteps = mpc_params.N_MPC - 1;
    std::vector<Matrix<double, 3, 4>> grf_vec(num_timesteps, Matrix<double, 3, 4>::Zero());

    for (int t = 0; t < num_timesteps; ++t)
    {
        Matrix<double, 3, 4>& grf = grf_vec[t];
        for (int foot = 0; foot < 4; ++foot)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                int idx = t * 12 + foot * 3 + axis;
                grf(axis, foot) = U[idx].get(GRB_DoubleAttr_X);
            }
        }
    }
    return grf_vec;
}

Vector<double, 12> ConvexMPC::solve_joint_torques()
{
    std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------"
              << std::endl;

    // Check if state is uninitialized - gravity state (index 12) should always be 9.81
    // If it's zero or very small, the state hasn't been updated yet
    if (x0.size() < 13 || std::abs(x0[12]) < 1e-3) {
        RCLCPP_WARN(logger_, "State x0 is uninitialized (gravity state = %.6f) - skipping MPC solve. Returning zero torques.", 
                    x0.size() >= 13 ? x0[12] : 0.0);
        return Vector<double, 12>::Zero();
    }

    if (mpc_params.verbose_diagnostics) {
        RCLCPP_INFO(logger_, "Solving joint torques using Convex MPC...");
    }
    // print_eigen_matrix(Q_bar, "Q_bar", logger_);
    // print_eigen_matrix(R_bar, "R_bar", logger_);


    // Convert foot positions to COM-relative positions in world frame
    // foot_positions are in world frame (absolute), but dynamics need COM-relative
    // x0 = [theta, p, omega, p_dot, g] where p is COM position in world frame
    double roll = x0[0];
    double pitch = x0[1];
    double yaw = x0[2];

    Matrix3d R_WB = (AngleAxisd(yaw, Vector3d::UnitZ()) *
           AngleAxisd(pitch, Vector3d::UnitY()) *
           AngleAxisd(roll, Vector3d::UnitX())).toRotationMatrix(); // Body frame orientation in world frame

    const auto t_fn0 = std::chrono::steady_clock::now();
    Matrix<double, 3, 4> p_B = get_foot_positions_body();
    Matrix<double, 3, 4> r_W;
    for (int i = 0; i < 4; i++)
    {
        r_W.col(i) = R_WB * p_B.col(i);
    }

    // Compact, throttled foot-geometry trace, in the same spirit as the STATE line
    // in the node. Columns are FL, FR, RL, RR, measured from the CoM in base-frame
    // axes. At a nominal stance x should be roughly +0.19/+0.19/-0.19/-0.19 shifted
    // by the CoM x offset, y ~ +0.14/-0.14/+0.14/-0.14, and z ~ -0.31 for all four.
    // Front/rear x signs swapped would mean the joint-angle remap feeding theta is
    // wrong. The fixed CoM offset is logged once at startup, not here.
    static rclcpp::Clock pb_log_clock(RCL_STEADY_TIME);
    RCLCPP_INFO_THROTTLE(logger_, pb_log_clock, 500,
        "PB_X %.4f %.4f %.4f %.4f | PB_Y %.4f %.4f %.4f %.4f | PB_Z %.4f %.4f %.4f %.4f",
        p_B(0, 0), p_B(0, 1), p_B(0, 2), p_B(0, 3),
        p_B(1, 0), p_B(1, 1), p_B(1, 2), p_B(1, 3),
        p_B(2, 0), p_B(2, 1), p_B(2, 2), p_B(2, 3));

    // Update the dynamics model to account for changing foot position and yaw
    const auto t_dss0 = std::chrono::steady_clock::now();
    StateSpace quad_dss = quadruped_state_space_discrete(x0[2], r_W, quad_params.inertiaTensor, quad_params.mass, mpc_params.dt); // TODO: proper initialization, perhaps based on the initial state of robot?
    // print_eigen_matrix(quad_dss.A, "A", logger_);
    // print_eigen_matrix(quad_dss.B, "B", logger_);
    const auto t_dss1 = std::chrono::steady_clock::now();
    tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);
    const auto t_pred = std::chrono::steady_clock::now();
    // print_eigen_matrix(B_qp, "B_qp", logger_);

    VectorXd U_temp = VectorXd::Zero(mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1)); // Initialize U_temp to zero

    // Swing gains are NOT reset here. They are set once at construction and then
    // overridden from ROS params via set_swing_gains(); re-assigning them every
    // solve would silently discard whatever the config asked for.

    // Q/R weights are constant after construction, so only log them when debugging.
    if (mpc_params.verbose_diagnostics)
    {
        RCLCPP_INFO(logger_, "Q matrix diagonal (state weights):");
        for (int i = 0; i < mpc_params.N_STATES; ++i) {
            RCLCPP_INFO(logger_, "  Q[%d] = %.2f", i, mpc_params.Q(i, i));
        }
        RCLCPP_INFO(logger_, "Q[5] (vertical position) = %.2f, Q[11] (vertical velocity) = %.2f",
                    mpc_params.Q(5, 5), mpc_params.Q(11, 11));
        RCLCPP_INFO(logger_, "R (control effort) weight = %.4f (diagonal, all forces)", mpc_params.R(0, 0));
        RCLCPP_INFO(logger_, "Q[5]/R ratio = %.2f (should be >> 1 to prioritize tracking over control effort)",
                    mpc_params.Q(5, 5) / mpc_params.R(0, 0));
    }

    // Update QP objective function
    double regularization = 1E-6; // Regularization term for numerical stability
    const auto t_p0 = std::chrono::steady_clock::now();
    P = compute_P(R_bar, Q_bar, B_qp, regularization);
    const auto t_p1 = std::chrono::steady_clock::now(); // Quadratic cost of linear mpc

    // A full eigendecomposition of the 108x108 P costs ~30ms and is only used to
    // print a condition number, so it is debug-only.
    if (mpc_params.verbose_diagnostics)
    {
        Eigen::SelfAdjointEigenSolver<MatrixXd> es(P);
        std::cout << "Min Eigenvalue: " << es.eigenvalues().minCoeff() << std::endl;
        std::cout << "Max Eigenvalue: " << es.eigenvalues().maxCoeff() << std::endl;
        std::cout << "Condition Number: " << es.eigenvalues().maxCoeff() / es.eigenvalues().minCoeff() << std::endl;
    }


    // print_eigen_matrix(x0, "x0", logger_);
    // print_eigen_matrix(X_ref.segment(0, mpc_params.N_STATES), "X_ref", logger_);

    if (mpc_params.verbose_diagnostics) {
        print_eigen_matrix(x0 - X_ref.segment(0, mpc_params.N_STATES), "x0 - X_ref", logger_);
    }
    // RCLCPP_INFO(logger_, "(x0 - X_ref)^T Q (x0 - X_ref): %f", ((x0 - X_ref.segment(0, mpc_params.N_STATES)).transpose() * mpc_params.Q * (x0 - X_ref.segment(0, mpc_params.N_STATES)))(0,0));

    // VectorXd x0_repeated = VectorXd::Zero(mpc_params.N_STATES * mpc_params.N_MPC);
    // for (int i = 0; i < mpc_params.N_MPC; ++i) {
    //     x0_repeated.segment(i * mpc_params.N_STATES, mpc_params.N_STATES) = x0;
    // }

    // RCLCPP_INFO(logger_, "(x0 - X_ref)^T Q_bar (x0 - X_ref): %f",
    //     ((x0_repeated - X_ref).transpose() * Q_bar * (x0_repeated - X_ref))(0,0));

    const auto t_q0 = std::chrono::steady_clock::now();
    q = compute_q(Q_bar, A_qp, B_qp, x0, X_ref);
    const auto t_q1 = std::chrono::steady_clock::now(); // Linear cost
    // print_eigen_matrix(q, "q", logger_);

    // Set QP objective
    // q has size N_CONTROLS * (N_MPC - 1), matching the decision variables U
    const auto t_pre = std::chrono::steady_clock::now();
    const auto t_obj0 = std::chrono::steady_clock::now();
    lin_expr = create_lin_obj(U, q, mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1));
    quad_expr = create_quad_obj(U, P , mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1));

    model->setObjective(quad_expr + lin_expr, GRB_MINIMIZE);
    const auto t_obj1 = std::chrono::steady_clock::now();
    model->optimize();
    const auto t_obj2 = std::chrono::steady_clock::now();

    // Objective assembly vs. the optimization itself. create_quad_obj walks the
    // dense 108x108 P and builds ~11.6k quadratic terms every cycle; if that
    // dominates, the fix is to stop rebuilding the objective rather than to make
    // the solver faster.
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "QPTIME pre %.2f (dss %.2f pred %.2f P %.2f q %.2f) | obj %.2f | optimize %.2f | gurobi %.2f ms",
        std::chrono::duration<double, std::milli>(t_pre - t_fn0).count(),
        std::chrono::duration<double, std::milli>(t_dss1 - t_dss0).count(),
        std::chrono::duration<double, std::milli>(t_pred - t_dss1).count(),
        std::chrono::duration<double, std::milli>(t_p1 - t_p0).count(),
        std::chrono::duration<double, std::milli>(t_q1 - t_q0).count(),
        std::chrono::duration<double, std::milli>(t_obj1 - t_obj0).count(),
        std::chrono::duration<double, std::milli>(t_obj2 - t_obj1).count(),
        model->get(GRB_DoubleAttr_Runtime) * 1000.0);
    
    // Check solver status
    int status = model->get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
        RCLCPP_ERROR(logger_, "Gurobi optimization failed with status: %d", status);
        RCLCPP_ERROR(logger_, "Status codes: 2=OPTIMAL, 3=INFEASIBLE, 4=UNBOUNDED, 5=INF_OR_UNBD");
        // Return zero torques on failure (safer than using invalid solution)
        return Vector<double, 12>::Zero();
    }
    
    if (mpc_params.verbose_diagnostics)
    {
        RCLCPP_INFO(logger_, "Gurobi solve time: %.6f seconds", model->get(GRB_DoubleAttr_Runtime));
        RCLCPP_INFO(logger_, "Gurobi objective value: %.6f", model->get(GRB_DoubleAttr_ObjVal));

        // Compute tracking error cost to verify Q weights are working
        VectorXd state_error = x0 - X_ref.segment(0, mpc_params.N_STATES);
        double tracking_cost = state_error.transpose() * mpc_params.Q * state_error;
        RCLCPP_INFO(logger_, "Current state tracking cost: %.6f (should be high if Q weights are large)", tracking_cost);
        RCLCPP_INFO(logger_, "Vertical position error: %.4f m, Vertical velocity error: %.4f m/s",
                    state_error(5), state_error(11));
    }

    // Set terminal constraint based on reference trajectory
    // VectorXd xN_ref = X_ref.segment(mpc_params.N_STATES * (mpc_params.N_MPC - 1), mpc_params.N_STATES);
    // RCLCPP_INFO(logger_, "Terminal reference state xN_ref: \n%f", xN_ref);

    // Extract ground reaction forces from Gurobi solution
    vector<Matrix<double, 3, 4>> grf_vec(mpc_params.N_MPC - 1, Matrix<double, 3, 4>::Zero());
    grf_vec = grf_from_mpc_solution();

    // Commanded vertical force against the weight it has to support, to locate the
    // standing height bias. At rest the first horizon step should sum to mg; a
    // persistent excess means the solver is commanding too much lift, while a
    // match means the bias lies in the measurement or the reference rather than in
    // the force solution. Also reports the moment the solution produces about the
    // CoM, which should be zero in a balanced stance.
    {
        const Matrix<double, 3, 4>& grf0 = grf_vec[0];
        const double fz_sum = grf0.row(2).sum();
        const double mg = quad_params.mass * quad_params.g;
        Vector3d moment = Vector3d::Zero();
        for (int i = 0; i < 4; ++i) {
            moment += r_W.col(i).cross(grf0.col(i));
        }
        static rclcpp::Clock grf_log_clock(RCL_STEADY_TIME);
        RCLCPP_INFO_THROTTLE(logger_, grf_log_clock, 500,
            "GRF fz %.2f %.2f %.2f %.2f | sum %.2f mg %.2f excess %+.2f | moment %+.3f %+.3f %+.3f",
            grf0(2, 0), grf0(2, 1), grf0(2, 2), grf0(2, 3),
            fz_sum, mg, fz_sum - mg,
            moment(0), moment(1), moment(2));
    }

    // GRF print
    // for (int i = 0; i < mpc_params.N_MPC - 1; ++i)
    // {
    //     Matrix<double, 3, 4> grf = grf_vec[i];
    //     print_eigen_matrix(grf, "GRF at timestep " + std::to_string(i), logger_);
    // }

    Matrix<double, 3, 4> grf = grf_vec[0];

    if (mpc_params.verbose_diagnostics)
    {
        print_eigen_matrix(grf, "GRF at k = 0", logger_);

        // Check total vertical GRF vs expected weight
        double total_vertical_grf = 0.0;
        for (int i = 0; i < 4; ++i) {
            total_vertical_grf += grf(2, i); // z-component (vertical)
        }
        double expected_weight = quad_params.mass * quad_params.g;
        double grf_deficit = expected_weight - total_vertical_grf;
        RCLCPP_INFO(logger_, "Total vertical GRF: %.2f N, Expected (mg): %.2f N, Deficit: %.2f N",
                    total_vertical_grf, expected_weight, grf_deficit);
        RCLCPP_INFO(logger_, "Per-leg vertical GRF: [%.2f, %.2f, %.2f, %.2f] N",
                    grf(2, 0), grf(2, 1), grf(2, 2), grf(2, 3));
    }

    // Predict states based on GRB solution
    for (int i = 0; i < U_temp.size(); i++) {
        U_temp(i) = U[i].get(GRB_DoubleAttr_X);
    }

    VectorXd X_pred = A_qp * x0 + B_qp * U_temp;


    // Print predicted states
    // std::ostringstream oss;
    // oss << "X_pred (rows: timesteps, cols: states):\n";
    // int num_timesteps = mpc_params.N_MPC;
    // int num_states = mpc_params.N_STATES;
    // for (int t = 0; t < num_timesteps; ++t) {
    //     oss << "Timestep " << std::setw(2) << t << ": [";
    //     for (int s = 0; s < num_states; ++s) {
    //         oss << std::fixed << std::setprecision(4) << std::setw(8) << X_pred(t * num_states + s);
    //         if (s < num_states - 1) oss << ", ";
    //     }
    //     oss << "]\n";
    // }
    // RCLCPP_INFO(logger_, "%s", oss.str().c_str());

    // Get the foot jacobian
    vector<Matrix3d> foot_jacobians = get_foot_jacobians();

    if (mpc_params.verbose_diagnostics)
    {
        print_eigen_matrix(foot_jacobians[0], "Foot Jacobian 0", logger_);
        print_eigen_matrix(foot_jacobians[1], "Foot Jacobian 1", logger_);
        print_eigen_matrix(foot_jacobians[2], "Foot Jacobian 2", logger_);
        print_eigen_matrix(foot_jacobians[3], "Foot Jacobian 3", logger_);
    }

    // Compute joint torques by utilizing the foot jacobians
    Vector<double, 12> joint_torques;

    if (mpc_params.verbose_diagnostics)
    {
        RCLCPP_INFO(logger_, "Roll: %.3f, Pitch: %.3f, Yaw: %.3f", roll, pitch, yaw);
        print_eigen_matrix(R_WB, "R_WB", logger_);
        print_eigen_matrix(R_WB.transpose(), "R_BW", logger_);

        MatrixXd grf_body_frame = R_WB.transpose() * grf; // Transform GRF to body frame
        print_eigen_matrix(grf_body_frame, "GRF in Body Frame", logger_);
    }


    for (int foot = 0; foot < 4; foot++)
    {
        // FRAME NOTE, left deliberately unresolved. tau = J^T f needs J and f in
        // the same frame, and by measurement they are not:
        //   - JACCHECK validates the Jacobian against a finite difference of
        //     get_foot_positions_body(), which is BASE frame, to 1.4e-07. So J's
        //     axes are the base's.
        //   - The model is fixed-base (asserted nq=12, nv=12), so q carries no body
        //     orientation and J structurally cannot encode R_WB.
        //   - The QP solves in WORLD frame (simplified_quad_dynamics.cpp has
        //     B[9:12] = I/mass with no rotation; convex_mpc.cpp rotates the moment
        //     arms out with r_W = R_WB * p_B).
        // So f is world, J is base, and the paper's tau_i = J_i^T R_i f_i (eq. 4)
        // has an R for exactly this reason.
        //
        // Inserting R_WB^T here was TRIED AND REVERTED on 2026-10-09. It is very
        // likely correct, but it measurably degraded STAND -- roll 0.0062 -> 0.0190,
        // pitch 0.0270 -> 0.0573, with the GRF shifting rearward (front 33.2 -> 31.6
        // N, rear 36.7 -> 38.0 N), a coherent pitch-axis redistribution rather than
        // noise. A correct fix should not make standing attitude 2-3x worse, so
        // something else is inconsistent with it and that was not identified.
        //
        // Omitting it is a small-angle approximation. Error on a 37.3 N vertical
        // GRF: 2.8% at the standing attitude, 15.2% at roll 0.152, 70.5% at roll
        // 0.720. Harmless where the robot actually operates, which is why STAND has
        // always been solid.
        //
        // Do not re-apply this until the trot is robust. The gait is currently
        // marginally stable (two runs from initial conditions matching to 4 decimal
        // places diverged at t~1.07 s, one surviving 25.6 s and one inverting), so a
        // single run cannot evaluate a 3% correction. The established root cause is
        // elsewhere: the Raibert heuristic reads COMMANDED velocity, never measured,
        // so the gait is open-loop in horizontal velocity.
        Vector3d foot_grf = grf_vec[0].col(foot); // Extract the grf for the current foot (world frame)

        Vector3d foot_joint_torques = foot_jacobians[foot].transpose()* (-foot_grf); // Compute joint torques for the current foot (correpsonding to its 3 actuators)
        joint_torques.segment<3>(foot * 3) = foot_joint_torques; // Store the joint torques in the joint_torques vector

       // joint_torques[foot * 3] = -joint_torques[foot * 3];  // Hip
        // joint_torques[foot * 3 + 1] = -joint_torques[foot * 3 + 1]; // Thigh 
        // joint_torques[foot * 3 + 2] = -joint_torques[foot * 3 + 2]; // Invert sign of calf joint?

        // print_eigen_matrix(R_WB.transpose() * foot_grf, "Foot " + std::to_string(foot) + " GRF in Body Frame", logger_);
        // // Test
        // Vector3d test_foot_grf;
        // test_foot_grf << 1, 0 , 1;
        // print_eigen_matrix(foot_jacobians[foot].transpose() * R_WB.transpose() * test_foot_grf, "Foot " + std::to_string(foot) + " Joint Torques", logger_);
    }

    


    // What the QP actually asked each foot to push with, before the Jacobian and
    // the clamp. Compare against the simulator's FMEAS: if GRFZ sums to the robot
    // weight but FMEAS does not, the force is being lost between the solution and
    // the joints (frame, sign or clamp); if GRFZ itself is short, it is the QP.
    {
        const Matrix<double, 3, 4> &g0 = grf_vec[0];
        // Print the stance flags the constraints were built from alongside the
        // solved force, so a force-on-a-swing-leg mismatch is unambiguous rather
        // than inferred by eye from two separately throttled log lines.
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
            "GRFZ stance %d%d%d%d | solved %.1f %.1f %.1f %.1f | sum %.1f N (weight %.1f N)",
            stance_[0] ? 1 : 0, stance_[1] ? 1 : 0, stance_[2] ? 1 : 0, stance_[3] ? 1 : 0,
            g0(2,0), g0(2,1), g0(2,2), g0(2,3),
            g0.row(2).sum(), quad_params.mass * quad_params.g);
    }

    const VectorXd joint_torques_preclamp = joint_torques;
    joint_torques = clamp_joint_torques(joint_torques); // Clamp the joint torques to the limits

    {
        const double dmax = (joint_torques - joint_torques_preclamp).cwiseAbs().maxCoeff();
        if (dmax > 1e-9) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
                "STANCECLAMP stance torques clipped, max change %.2f Nm", dmax);
        }
    }

    return joint_torques;
}

/**
 * @brief Updates the pinocchio kinematics model based on the private class theta variable.
 * 
 * Needs to be called prior to referencing any jacobians (e.g. foot jacobians) or referencing frame locations
 * (i.e. foot positions) within the convex mpc pipeline.
 * @return None
 */
void ConvexMPC::update_kinematics()
{
    if (!foot_kinematics_dirty_) return;


    for (int foot=0; foot<4; foot++)
    {
        for(int joint = 0; joint <3; joint++)
        {
            q_pin_[joint_qidx_[foot][joint]] = this->theta[foot*3 + joint];
        }
    }

    // FK to update model state
    pinocchio::forwardKinematics(pinocchio_model, pinocchio_data, q_pin_);
    pinocchio::updateFramePlacements(pinocchio_model, pinocchio_data);

    foot_kinematics_dirty_ = false;
}

Matrix<double, 3, 4> ConvexMPC::get_foot_positions_body()
{
    update_kinematics();

    // Feet relative to the body CoM, not the base link origin: the dynamics take
    // moments about the CoM (tau = r x f), so r must start there. com_B_ is the
    // fixed base-link inertial origin, consistent with the massless-leg assumption
    // this formulation is built on -- the same assumption that lets the inertia
    // tensor be a constant. A configuration-dependent CoM would contradict it.
    Matrix<double, 3, 4> p_B;
    for (int foot = 0; foot<4; foot++)
    {
        p_B.col(foot) = pinocchio_data.oMf[foot_fid_[foot]].translation() - com_B_;
    }
    return p_B;
}

/**
 * @brief Updates the foot jacobians 
 * @param
 * @return None
 */
vector<Matrix3d> ConvexMPC::get_foot_jacobians()
{
    vector<Matrix<double, 3, 3>> J(4, Matrix3d::Zero()); // Initialize a vector of 4 matrices for the foot jacobians


    // Update the pinocchio model config q with the joint angles (doing this to make sure joint indices match)
    this->update_kinematics();

    // Compute Jacobian for each foot
    for (int foot=0; foot < 4; foot++)
    {
        // pinocchio::FrameIndex foot_frame_id = pinocchio_model.getFrameId(foot_names[foot_index]);
        pinocchio::FrameIndex foot_frame_id = foot_fid_[foot];

        Eigen::MatrixXd J_temp(6, pinocchio_model.nv);
        pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q_pin_, foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        // Construct the 3x3 Jacobian for each foot
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // int joint_id = pinocchio_model.getJointId(joint_names_by_foot[foot_index][j]);
                // int v_idx = pinocchio_model.joints[joint_id].idx_v();

                int v_idx = joint_vidx_[foot][j];

                
                // std::cout << "  Joint: " << joint_names_by_foot[foot_index][j] << " (velocity index " << v_idx << ")\n";
                // std::cout << "  Linear part:\n" << J_temp.block<3,1>(0, v_idx) << "\n";
                // std::cout << "  Angular part:\n" << J_temp.block<3,1>(3, v_idx) << "\n";

                // J[foot_index](i, j) = J_temp(i, pinocchio_model.joints[joint_id].idx_v());
                J[foot](i, j) = J_temp(i, v_idx);
            }

        }
        // J[foot_index](2, 1) = -J[foot_index](2, 1); // Thigh to z GRF mapping
    }

    return J;
}

/**
 * @brief Joint torques that track a swing-foot reference, by Jacobian transpose.
 *
 * tau = J^T * ( Kp*(p_ref - p) + Kd*(v_ref - v) )
 *
 * A Cartesian-space PD law mapped into joint torques. This mirrors the stance
 * path, which maps GRFs through the same transpose (see solve_joint_torques),
 * so both branches of the gait FSM share one frame convention.
 *
 * All quantities are in the foot Jacobian's frame (LOCAL_WORLD_ALIGNED: base
 * origin, world-aligned axes), so the caller is responsible for rotating a
 * world-frame reference into it.
 *
 * An operational-space feedforward (Lambda * a_ref) is deliberately omitted:
 * it needs the joint-space mass matrix and Jdot, and the previous placeholder
 * identity mass matrix would have produced a confidently wrong feedforward.
 * With the legs light relative to the body, Cartesian PD is what the MIT
 * Cheetah convex-MPC formulation this controller follows uses for swing.
 */
void ConvexMPC::update_joint_velocities(const Vector<double, 12>& theta_dot)
{
    if (v_pin_.size() != pinocchio_model.nv) {
        v_pin_ = Eigen::VectorXd::Zero(pinocchio_model.nv);
    }
    for (int foot = 0; foot < 4; foot++) {
        for (int j = 0; j < 3; j++) {
            v_pin_[joint_vidx_[foot][j]] = theta_dot[foot * 3 + j];
        }
    }
}


Matrix3d ConvexMPC::get_foot_operational_space_inertia(int leg_index, const Matrix3d& J_i)
{
    update_kinematics();

    // Full joint-space mass matrix. crba fills only the upper triangle.
    pinocchio::crba(pinocchio_model, pinocchio_data, q_pin_);
    pinocchio_data.M.triangularView<Eigen::StrictlyLower>() =
        pinocchio_data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // This leg's 3x3 block. Exact on a fixed-base model: the legs are
    // kinematically independent so M is block diagonal.
    Matrix3d M_i;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            M_i(r, c) = pinocchio_data.M(joint_vidx_[leg_index][r],
                                         joint_vidx_[leg_index][c]);
        }
    }

    // Reflected rotor inertia. go2.xml gives every leg joint armature="0.01",
    // which the URDF does not carry, so a Lambda built from the URDF alone
    // understates the apparent mass the simulator actually presents. Checked
    // against MuJoCo's own mj_fullM at the nominal stance: without this term our
    // Lambda diagonal reads 0.257/0.264/0.214 kg against MuJoCo's
    // 0.426/0.487/0.710 (-70%); with it the two agree to ~15%, the remainder
    // being URDF/MJCF link inertia differences. On hardware this is the real
    // motor-plus-gearbox inertia seen at the joint, so it belongs here too.
    M_i.diagonal().array() += quad_params.joint_armature;

    // Lambda = (J M^-1 J^T)^-1. Guard the inverse: at a singular configuration
    // (fully extended leg) J loses rank and J M^-1 J^T becomes ill-conditioned,
    // where a raw inverse would return huge gains. Fall back to the scalar leg
    // mass, which is the right order of magnitude and cannot blow up.
    const Matrix3d JMinvJt = J_i * M_i.inverse() * J_i.transpose();
    Eigen::JacobiSVD<Matrix3d> svd(JMinvJt);
    const double cond = svd.singularValues()(0) /
                        std::max(svd.singularValues()(2), 1e-12);
    if (!JMinvJt.allFinite() || cond > 1e6) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
            "Leg %d near-singular (cond %.1e); using scalar inertia for Lambda",
            leg_index, cond);
        return Matrix3d::Identity() * M_i.diagonal().mean();
    }
    return JMinvJt.inverse();
}


Vector3d ConvexMPC::get_foot_jacobian_time_variation_term(int leg_index)
{
    update_kinematics();

    if (v_pin_.size() != pinocchio_model.nv) {
        v_pin_ = Eigen::VectorXd::Zero(pinocchio_model.nv);
    }

    // Jdot, in the same LOCAL_WORLD_ALIGNED frame get_foot_jacobians() uses, so
    // the product below is consistent with the J it will be combined with.
    pinocchio::computeJointJacobiansTimeVariation(
        pinocchio_model, pinocchio_data, q_pin_, v_pin_);

    Eigen::Matrix<double, 6, Eigen::Dynamic> dJ(6, pinocchio_model.nv);
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(
        pinocchio_model, pinocchio_data, foot_fid_[leg_index],
        pinocchio::LOCAL_WORLD_ALIGNED, dJ);

    // Jdot * qdot, linear rows only, restricted to this leg's three joints. The
    // legs are kinematically independent on a fixed-base model, so no other
    // joint contributes to this foot's acceleration.
    Vector3d out = Vector3d::Zero();
    for (int j = 0; j < 3; j++) {
        const int v_idx = joint_vidx_[leg_index][j];
        out += dJ.block<3, 1>(0, v_idx) * v_pin_[v_idx];
    }
    return out;
}


Vector3d ConvexMPC::get_leg_gravity_coriolis(int leg_index)
{
    update_kinematics();

    if (v_pin_.size() != pinocchio_model.nv) {
        v_pin_ = Eigen::VectorXd::Zero(pinocchio_model.nv);
    }

    // rnea with zero acceleration returns C(q,qdot)*qdot + G(q), the paper's
    // C_i qdot_i + G_i once restricted to this leg's rows.
    //
    // The Coriolis part scales as qdot^2 and depends on the MEASURED joint
    // velocity, so unlike Lambda*a_ref it sits inside the feedback loop. With a
    // swing leg transiently at 57 rad/s (measured) it produced 792 Nm commands
    // and diverged. Gravity G(q) depends only on configuration and is safe, so
    // the velocity is clamped for this term rather than dropping it: below the
    // clamp the full Coriolis compensation is used, above it the term is
    // bounded. A leg legitimately moving faster than this is already failing.
    constexpr double kMaxCoriolisJointVel = 8.0;  // rad/s
    Eigen::VectorXd v_clamped = v_pin_;
    for (int i = 0; i < v_clamped.size(); i++) {
        v_clamped[i] = std::clamp(v_clamped[i],
                                  -kMaxCoriolisJointVel, kMaxCoriolisJointVel);
    }

    const Eigen::VectorXd tau_bias = pinocchio::rnea(
        pinocchio_model, pinocchio_data, q_pin_, v_clamped,
        Eigen::VectorXd::Zero(pinocchio_model.nv));

    Vector3d out;
    for (int j = 0; j < 3; j++) {
        out[j] = tau_bias[joint_vidx_[leg_index][j]];
    }
    return out;
}


Matrix3d ConvexMPC::swing_Kp_from_natural_frequency(const Vector3d& omega,
                                                    int leg_index,
                                                    const Matrix3d& J_i)
{
    // Paper eq. (3): Kp_ii = omega_i^2 * Lambda_ii. Only the diagonal is used,
    // matching the paper's statement that Kp is a diagonal gain matrix.
    const Matrix3d Lambda = get_foot_operational_space_inertia(leg_index, J_i);
    Vector3d kp;
    for (int i = 0; i < 3; i++) {
        kp[i] = omega[i] * omega[i] * Lambda(i, i);
    }
    return kp.asDiagonal();
}


Vector<double, 3> ConvexMPC::compute_swing_leg_torques_natural_freq(
    int leg_index,
    const Matrix3d& J_i,
    const Vector3d& p_i, const Vector3d& v_i,
    const Vector3d& p_i_ref, const Vector3d& v_i_ref,
    const Vector3d& a_i_ref,
    const Vector3d& omega,
    double zeta,
    bool use_feedforward)
{
    // Paper eq. (3): Kp_ii = omega_i^2 * Lambda_ii, recomputed from the CURRENT
    // configuration so the closed-loop natural frequency stays constant as the
    // leg's apparent mass changes with pose. A fixed Kp lets that frequency
    // drift, which the paper calls out explicitly, and is the likely reason a
    // hand-tuned Kp needed 3x more gain on y than on x and z -- the apparent
    // mass along y is simply different.
    const Matrix3d Lambda = get_foot_operational_space_inertia(leg_index, J_i);

    // Kd follows from the same Lambda at the requested damping ratio:
    // Kd_ii = 2 * zeta * omega_i * Lambda_ii. Tying both gains to Lambda keeps
    // the damping ratio constant too, rather than only the frequency.
    Vector3d kp, kd;
    for (int i = 0; i < 3; i++) {
        kp[i] = omega[i] * omega[i] * Lambda(i, i);
        kd[i] = 2.0 * zeta * omega[i] * Lambda(i, i);
    }

    const Vector3d f_fb = kp.asDiagonal() * (p_i_ref - p_i)
                        + kd.asDiagonal() * (v_i_ref - v_i);

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000,
        "EQ3GAINS leg %d | Lambda diag %.3f %.3f %.3f | Kp %.1f %.1f %.1f | Kd %.1f %.1f %.1f",
        leg_index, Lambda(0,0), Lambda(1,1), Lambda(2,2),
        kp[0], kp[1], kp[2], kd[0], kd[1], kd[2]);

    if (!use_feedforward) {
        return J_i.transpose() * f_fb;
    }
    const Vector3d Jdot_qdot = get_foot_jacobian_time_variation_term(leg_index);
    return J_i.transpose() * (f_fb + Lambda * (a_i_ref - Jdot_qdot))
         + get_leg_gravity_coriolis(leg_index);
}


Vector<double, 3> ConvexMPC::compute_swing_leg_torques_with_feedforward(
    int leg_index,
    const Matrix3d& J_i,
    const Vector3d& p_i, const Vector3d& v_i,
    const Vector3d& p_i_ref, const Vector3d& v_i_ref,
    const Vector3d& a_i_ref)
{
    // Feedback bracket, identical to compute_swing_leg_tracking_torques().
    const Vector3d f_fb = Kp * (p_i_ref - p_i) + Kd * (v_i_ref - v_i);

    // Feedforward, paper eq. (2). Lambda*a_ref is the force needed to accelerate
    // the foot along its reference arc; without it the feedback term has to
    // produce that force out of tracking error, which is why the error scales
    // with arc speed.
    // Paper eq. (2): Lambda * (a_ref - Jdot qdot), not Lambda * a_ref. The
    // Jdot*qdot term is the centripetal/Coriolis part of the foot acceleration --
    // the acceleration the foot has purely because the leg is moving, even at
    // constant joint rates. Measured at 0.1-2% of the control signal on a 1 s
    // swing, but it scales as qdot^2, so at the production 0.25 s swing it is
    // ~16x larger and comparable to the PD force.
    const Matrix3d Lambda = get_foot_operational_space_inertia(leg_index, J_i);
    const Vector3d Jdot_qdot = get_foot_jacobian_time_variation_term(leg_index);
    const Vector3d f_ff = Lambda * (a_i_ref - Jdot_qdot);

    // Leg gravity and Coriolis act in JOINT space, so they are added after the
    // Jacobian transpose rather than being folded into the endpoint force.
    const Vector3d tau_bias = get_leg_gravity_coriolis(leg_index);

    // Component trace. The divergence is a positive feedback loop and exactly one
    // of these terms drives it; three hypotheses (Coriolis magnitude, the missing
    // Jdot*qdot, a stale or corrupted Pinocchio Data) have each been tested and
    // disproved, so log the terms rather than guess a fourth time. The first
    // sample where one component grows while the others stay bounded names it.
    RCLCPP_INFO(logger_,
        "FFTERMS leg %d | f_fb %+9.2f %+9.2f %+9.2f | Lam_a %+9.2f %+9.2f %+9.2f"
        " | Lam_Jdq %+9.2f %+9.2f %+9.2f | bias %+9.2f %+9.2f %+9.2f",
        leg_index,
        f_fb[0], f_fb[1], f_fb[2],
        (Lambda * a_i_ref)[0], (Lambda * a_i_ref)[1], (Lambda * a_i_ref)[2],
        (Lambda * Jdot_qdot)[0], (Lambda * Jdot_qdot)[1], (Lambda * Jdot_qdot)[2],
        tau_bias[0], tau_bias[1], tau_bias[2]);

    return J_i.transpose() * (f_fb + f_ff) + tau_bias;
}


Vector<double, 3> ConvexMPC::compute_swing_leg_tracking_torques(
    const Matrix3d& J_i,
    const Vector3d& p_i, const Vector3d& v_i,
    const Vector3d& p_i_ref, const Vector3d& v_i_ref) const
{
    // Force the foot should feel in order to move toward its reference.
    const Vector3d f_i = Kp * (p_i_ref - p_i) + Kd * (v_i_ref - v_i);

    // tau = J^T f, by virtual work: f_i is the force we want applied AT THE FOOT,
    // and J maps joint rates to foot velocity, so its transpose maps an endpoint
    // force to the equivalent joint torques. No negation.
    //
    // This previously returned J^T * (-f_i), copied from the stance path in
    // solve_joint_torques(). The two cases are not analogous: there, `foot_grf` is
    // the force the GROUND exerts on the foot, so the torque the leg must produce
    // is the reaction and the negation is correct. Here f_i is already the force
    // to apply to the foot, so negating it drives the foot away from its target.
    //
    // Verified in MuJoCo on this model, foot off the floor and gravity disabled,
    // commanding +30 N in z: J^T*(+f) moves the foot +0.212 m (toward the force),
    // J^T*(-f) moves it +0.006 m, i.e. the command does essentially nothing except
    // fight the leg's own dynamics. In the live controller the wrong sign made a
    // swing foot climb 7 m/s upward while being commanded downward.
    return J_i.transpose() * f_i;
}

void ConvexMPC::set_swing_gains(const Matrix3d& Kp_swing, const Matrix3d& Kd_swing)
{
    Kp = Kp_swing;
    Kd = Kd_swing;
}

// void ConvexMPC::add_friction_cone_constraints(GRBModel& model, GRBVar* U, const double& mu)
// {
//     for (int k = 0; k < mpc_params.N_MPC - 1; k++) // For each timestep
//     {
//         // TODO: Friction cone constraints
//         for (int i = 0; i < 4; i++) // Legs
//         {
//             // Create the friction cone constraints
//             model.addConstr(U[k*12 +3*i] <= mu * U[k*12 +3*i + 2]); // Ensure the horizontal force is within the friction cone
//             model.addConstr(U[k*12 +3*i + 1] <= mu * U[k*12 +3*i + 2]); // Ensure the horizontal force is within the friction cone
//             model.addConstr(U[k*12 +3*i + 2] >= 10); // Ensure the vertical force is greater than 10N

//         }
//     }

// }

Vector<double, 12> ConvexMPC::clamp_joint_torques(Vector<double, 12>& joint_torques)
{
    Vector<double, 12> clamped_torques = joint_torques;

    // Clamp per joint, not against one scalar: the Go2's hip and thigh actuators
    // stop at 23.7 Nm while only the calf reaches 45.43. A single 45 Nm limit let
    // hip and thigh commands through at nearly twice what the hardware can apply,
    // so the torque the controller believed it had applied and the torque the
    // plant actually saw diverged without any warning.
    for (int i = 0; i < 12; i++) {
        const double limit = quad_params.joint_torque_limit(i % 3); // 0=hip, 1=thigh, 2=calf
        clamped_torques[i] = std::clamp(clamped_torques[i], -limit, limit);
    }

    return clamped_torques;
}

void ConvexMPC::set_contact_constraints(unordered_map<std::string, int>& contact_states)
{
    // Single-state overload: hold this contact state across the whole horizon.
    // Correct only for a static gait (all four feet in stance for every step);
    // any gait that lifts a foot must use the schedule overload below.
    std::vector<std::array<bool, 4>> schedule(
        mpc_params.N_MPC - 1,
        std::array<bool, 4>{
            contact_states[quad_params.LEG_INDEX_TO_NAME.at(0)] == 1,
            contact_states[quad_params.LEG_INDEX_TO_NAME.at(1)] == 1,
            contact_states[quad_params.LEG_INDEX_TO_NAME.at(2)] == 1,
            contact_states[quad_params.LEG_INDEX_TO_NAME.at(3)] == 1});

    set_contact_constraints(schedule);
}

void ConvexMPC::set_contact_constraints(const std::vector<std::array<bool, 4>>& contact_schedule)
{
    if (static_cast<int>(contact_schedule.size()) < mpc_params.N_MPC - 1) {
        RCLCPP_ERROR(logger_, "Contact schedule too short: need %d steps, got %zu",
                     mpc_params.N_MPC - 1, contact_schedule.size());
        throw std::invalid_argument("Contact schedule shorter than MPC horizon");
    }

    // Log the applied step's contact state. Throttled: at 40 Hz an unthrottled
    // line here is one of the noisier things in the log during a gait.
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 200, "Contact states: FL=%d, FR=%d, RL=%d, RR=%d",
                contact_schedule[0][0] ? 1 : 0, contact_schedule[0][1] ? 1 : 0,
                contact_schedule[0][2] ? 1 : 0, contact_schedule[0][3] ? 1 : 0);

    // Remember which feet are in stance so the force regularizer can be centred on
    // the gravity-compensating distribution rather than on zero. Only the k=0 step
    // matters: compute_U_nom() shapes the regularizer for the force that is
    // actually applied this cycle.
    for (int i = 0; i < 4; i++) {
        stance_[i] = contact_schedule[0][i];
    }

    // Remove previous contact constraints
    for (const auto& constr : contact_constraints_) {
        model->remove(constr);
    }

    contact_constraints_.clear();

    for (int k = 0; k < mpc_params.N_MPC - 1; k++) // For each timestep
    {
        for (int i = 0; i < 4; i++)
        {
            // Contact is read per timestep, not frozen at k=0: over a 0.25 s
            // horizon a trot lifts and plants feet partway through, and pinning
            // the k=0 state would have the solver plan forces on feet that have
            // already left the ground.
            const bool in_stance = contact_schedule[k][i];

            if (in_stance) // If the leg is in stance, add friction cone constraints
            {
                // Friction cone constraints
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i] <= quad_params.mu * U[k*12 +3*i + 2]) // Ensure the horizontal force is within the friction cone
                );
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i] >= -quad_params.mu * U[k*12 +3*i + 2]) // Ensure the horizontal force is within the friction cone
                );

                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i + 1] <= quad_params.mu * U[k*12 +3*i + 2]) // Ensure the horizontal force is within the friction cone
                );
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i + 1] >= -quad_params.mu * U[k*12 +3*i + 2]) // Ensure the horizontal force is within the friction cone
                );

                // Minimum vertical GRF constraint - critical for standing
                // MATLAB enforces f_min = 10N to ensure all stance legs apply force
                // This prevents the MPC from trying to use zero forces when standing
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i + 2] >= mpc_params.min_vertical_grf) // Ensure the vertical force is greater than the minimum
                );
                // Maximum vertical GRF constraint
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 +3*i + 2] <= mpc_params.max_vertical_grf) // Ensure the vertical force is less than the maximum
                );
            }
            else
            {
                // If the leg is in swing, set the ground reaction forces to zero
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 + 3*i] == 0)
                );
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 + 3*i + 1] == 0)
                );
                contact_constraints_.push_back(
                    model->addConstr(U[k*12 + 3*i + 2] == 0)
                );
            }

        }
    }
}
