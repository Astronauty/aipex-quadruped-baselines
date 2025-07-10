#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
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
        // GRBEnv env = GRBEnv();
        env = make_unique<GRBEnv>(true);
        // env.set("LicenseFile", "/home/daniel/gurobi.lic");
        env->set("LogFile", "convex_mpc.log");
        env->set("OutputFlag", "1");
        env->start();
        model = make_unique<GRBModel>(*env);

        // Initialize robot state
        x0 = VectorXd::Zero(mpc_params.N_STATES); // Initial state of the robot
        // u = Vector<double, 12>::Zero();
        ground_reaction_forces = Matrix<double, 3, 4>::Zero(); // Initialize GRFs for the 4 feet of the quadruped robot
        X_ref = VectorXd::Ones(mpc_params.N_STATES*mpc_params.N_MPC); // Reference trajectory
        theta = VectorXd::Zero(12); // Initialize joint angles of the quadruped robot

        // Initialize pinocchio model & data (for foot jacobian computaiton)
        string urdf_path = PINOCCHIO_MODEL_DIR "/go2_description.urdf";
        pinocchio::urdf::buildModel(urdf_path, pinocchio_model);
        pinocchio_data = pinocchio::Data(pinocchio_model);
        
        // Initialize state space prediction matrices
        // StateSpace quad_dss = get_default_dss_model(); // TODO: proper initialization, perhaps based on the initial state of robot?
        StateSpace quad_dss = quadruped_state_space_discrete(0, ground_reaction_forces, quad_params.inertiaTensor, mpc_params.dt);
        tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);

        // Initialize swing leg tracker gains
        Kp = Matrix3d::Identity() * 10.0; // Proportional gain for swing leg tracking
        Kd = Matrix3d::Identity() * 1; // Derivative gain for swing leg tracking

        // Formulate the QP 
        U = model->addVars(mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1), GRB_CONTINUOUS);
        cout << "Number of Decision Variables:" << mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1) << endl;


        Q_bar = compute_Q_bar(); // Diagonal block matrix of quadratic state cost for N_MPC steps
        // cout << "Size of Q_bar: " << Q_bar.rows() << " x " << Q_bar.cols() << endl;

        R_bar = compute_R_bar(); // Diagonal block matrix of quadratic control cost for N_MPC steps
        // cout << "Size of R_bar: " << R_bar.rows() << " x " << R_bar.cols() << endl;

        P = compute_P(R_bar, Q_bar, A_qp); // Quadratic cost of linear mpc
        // cout << "Size of P: " << P.rows() << " x " << P.cols() << endl;

        q = compute_q(Q_bar, A_qp, B_qp, x0, X_ref);
        // cout << "Size of q: " << q.rows() << " x " << q.cols() << endl;


        quad_expr = create_quad_obj(U, P , mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1));
        lin_expr = create_lin_obj(U, q, mpc_params.N_STATES);

        // GRBLinExpr test_expr = U[1];
        // model.setObjective(create_quad_obj(U, P , mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1)) 
            // + create_lin_obj(U, q, mpc_params.N_STATES));
        model->setObjective(quad_expr + lin_expr, GRB_MINIMIZE);
        // model.setObjective(test_expr, GRB_MINIMIZE);

        // cout << "Objective Function: " << model.getObjective() << endl;

    } 
    catch(GRBException e) 
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }

}

void print_eigen_matrix(const Eigen::MatrixXd& mat, string name, const rclcpp::Logger& logger) 
{
    std::stringstream ss;
    ss << "\n" << mat;
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

    MatrixXd A_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * (N_MPC-1));
    MatrixXd B_qp = MatrixXd::Zero(N_STATES * N_MPC, N_STATES); 

    for (int i = 0; i < mpc_params.N_MPC; i++) 
    {
        for (int j = 0; j < i; j++) 
        {
            // cout << i << " x " << j << endl;
            A_qp.block(i * N_STATES, j * N_CONTROLS, N_STATES, N_CONTROLS) = quad_dss.A.pow(i - j) * quad_dss.B;
            // cout << "A_qp accessing: " << i * N_STATES << "x" << j * N_CONTROLS << endl;
            // cout << "A_qp block size: " << N_STATES << "x" << N_CONTROLS << endl;
            // cout << "A_qp End: " << (i+1)*N_STATES << "x" << (j+1)*N_CONTROLS << endl;
            // cout << "\n" << endl;
        }
        // cout << "pog2 " << i << endl;
        // cout << "\n" << quad_dss.A.pow(0) << endl;
        B_qp.block(i*N_STATES, 0, N_STATES, N_STATES) = quad_dss.A.pow(i+1); 
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
    StateSpace quad_dss = quadruped_state_space_discrete(yaw, foot_positions, quad_params.inertiaTensor,mpc_params.dt);


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
P = 2*R_bar + 2*A_qp'*Q_bar*A_qp
*/
MatrixXd ConvexMPC::compute_P(MatrixXd R_bar, MatrixXd Q_bar, MatrixXd A_qp)
{
    assert(R_bar.rows() == R_bar.cols() && "R_bar must be square.");
    assert(Q_bar.rows() == Q_bar.cols() && "Q_bar must be square.");
    assert(A_qp.cols() == R_bar.rows() && "A_qp and R_bar dimensions must match.");
    
    MatrixXd P = 2*R_bar + 2*A_qp.transpose() * Q_bar * A_qp;
    return P;   
}

VectorXd ConvexMPC::compute_q(MatrixXd Q_bar, MatrixXd A_qp, MatrixXd B_qp, VectorXd x0, VectorXd X_ref)
{
    // VectorXd q = 2*x0.transpose() * B_qp * Q_bar.transpose() * A_qp; // Zero reference linear cost
    VectorXd q = 2*(x0.transpose() * B_qp.transpose() * Q_bar * A_qp - X_ref.transpose() * Q_bar * A_qp);
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

}

void ConvexMPC::update_foot_positions(const Matrix<double, 3, 4>& foot_positions)
{
    // Update the foot positions in the body frame
    RCLCPP_INFO(logger_, "Updating foot positions in ConvexMPC.");
    RCLCPP_INFO(logger_, "Foot positions: \n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
        foot_positions(0, 0), foot_positions(0, 1), foot_positions(0, 2), foot_positions(0, 3),
        foot_positions(1, 0), foot_positions(1, 1), foot_positions(1, 2), foot_positions(1, 3),
        foot_positions(2, 0), foot_positions(2, 1), foot_positions(2, 2), foot_positions(2, 3));

    this->foot_positions = foot_positions;
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

Vector<double, 12> ConvexMPC::solve_joint_torques()
{
    std::cout << "-------------------------------------------------------------------------"
              << std::endl;

    // Update the dynamics model to account for changing foot position and yaw
    StateSpace quad_dss = quadruped_state_space_discrete(x0[2], foot_positions, quad_params.inertiaTensor, mpc_params.dt); // TODO: proper initialization, perhaps based on the initial state of robot?
    tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);

    VectorXd U_temp = VectorXd::Zero(mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1)); // Initialize U_temp to zero 

    // for (int i = 0; i < mpc_params.N_MPC-1; i++)
    // {
    //     for (int j = 0; j < 4; j++)
    //     {
    //         U_temp(i*12 + j*3 + 2) = 50.0;

    //         U_temp(i*12 + j*3 + 0) = 5.0;
    //     }

    // }

    // RCLCPP_INFO(logger_, "U_temp:");
    // for (int i = 0; i < U_temp.size(); ++i) {
    //     RCLCPP_INFO(logger_, "U_temp[%d] = %f", i, U_temp(i));
    // }

    // RCLCPP_INFO(logger_, "A_qp size: %ld x %ld", A_qp.rows(), A_qp.cols());
    // RCLCPP_INFO(logger_, "B_qp size: %ld x %ld", B_qp.rows(), B_qp.cols());




    // Initialize swing leg tracker gains
    Kp = Matrix3d::Identity() * 10.0; // Proportional gain for swing leg tracking
    Kd = Matrix3d::Identity() * 1; // Derivative gain for swing leg tracking

    // Update QP objective function
    P = compute_P(R_bar, Q_bar, A_qp); // Quadratic cost of linear mpc
    q = compute_q(Q_bar, A_qp, B_qp, x0, X_ref);

    lin_expr = create_lin_obj(U, q, mpc_params.N_STATES); // Only the linear part of the objective is influenced by x0
    quad_expr = create_quad_obj(U, P , mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1));
    model->setObjective(quad_expr + lin_expr, GRB_MINIMIZE);
    model->optimize(); 

    // Extract ground reaction forces from Gurobi solution
    Matrix<double, 3, 4> grf = Matrix<double, 3, 4>::Zero();
    // for (int axis = 0; axis < 3; axis++) 
    // {
    //     for (int foot = 0; foot < 4; foot++) 
    //     {
    //         int idx = foot * 3 + axis;
    //         grf(axis, foot) = U[idx].get(GRB_DoubleAttr_X);
    //     }
    
    // }

    for (int i = 0; i < mpc_params.N_MPC - 1; ++i) 
    {
        std::ostringstream oss;
        oss << "GRF_z step " << i << ": ";
        for (int foot = 0; foot < 4; ++foot) {
            int idx = i * 12 + foot * 3 + 2; // z component for each foot at step i
            double grf_z = U[idx].get(GRB_DoubleAttr_X);
            oss << "foot " << foot << ": " << grf_z;
            if (foot < 3) oss << ", ";
        }
        RCLCPP_INFO(logger_, "%s", oss.str().c_str());
    }
    // RCLCPP_INFO(logger_, "GRFs:");
    // print_eigen_matrix(grf, "GRFs", logger_);

    // Predict states based on GRB solution
    for (int i = 0; i < U_temp.size(); i++) {
        U_temp(i) = U[i].get(GRB_DoubleAttr_X);
    }

    VectorXd X_pred = A_qp * U_temp + B_qp * x0;
    
    RCLCPP_INFO(logger_, "X_pred size: %ld, %ld", X_pred.rows(), X_pred.cols());

    for (int i = 0; i < mpc_params.N_MPC; i++) 
    {
        for (int j = 0; j < 13; ++j) {
            if (j == 0) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(6);  // Set precision for consistent formatting
                oss << "X_pred[" << std::setw(2) << i << "] = [";
                for (int k = 0; k < 13; ++k) {
                    oss << std::setw(10) << X_pred(i * 13 + k);  // Fixed width of 10 characters
                    if (k < 12) oss << ", ";
                }
                oss << "]";
                RCLCPP_INFO(logger_, "%s", oss.str().c_str());
            }
        }
    }


    // Get the foot jacobian
    vector<Matrix3d> foot_jacobians = get_foot_jacobians(this->theta);

    print_eigen_matrix(foot_jacobians[0], "Foot Jacobian 0", logger_);
    print_eigen_matrix(foot_jacobians[1], "Foot Jacobian 1", logger_);
    print_eigen_matrix(foot_jacobians[2], "Foot Jacobian 2", logger_);
    print_eigen_matrix(foot_jacobians[3], "Foot Jacobian 3", logger_);

    // Compute joint torques by utilizing the foot jacobians
    Vector<double, 12> joint_torques;

    double roll = x0[0];
    double pitch = x0[1];
    double yaw = x0[2];

    Matrix3d R_WB = (AngleAxisd(roll, Vector3d::UnitZ()) * 
           AngleAxisd(pitch, Vector3d::UnitY()) * 
           AngleAxisd(yaw, Vector3d::UnitX())).toRotationMatrix(); // Body frame orientation in world frame

    print_eigen_matrix(R_WB, "R_WB", logger_);
    for (int foot = 0; foot < 4; foot++)
    {
        Vector3d foot_grf = grf.col(foot); // Extract the grf for the current foot (world frame)
        
        Vector3d foot_joint_torques = foot_jacobians[foot].transpose() * R_WB.transpose() * foot_grf; // Compute joint torques for the current foot (correpsonding to its 3 actuators)
        joint_torques.segment<3>(foot * 3) = foot_joint_torques; // Store the joint torques in the joint_torques vector
    }

    return -joint_torques;
}


vector<Matrix3d> ConvexMPC::get_foot_jacobians(const Vector<double, 12>& theta)
{

    // TODO: check joint order
    vector<vector<string>> joint_names_by_foot = {
        {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}, // Foot 0
        {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}, // Foot 1
        {"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}, // Foot 2
        {"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}  // Foot 3
    };

    vector<string> foot_names = {
        "FL_foot", // Foot 0
        "FR_foot", // Foot 1
        "RL_foot", // Foot 2
        "RR_foot"  // Foot 3
    };

    vector<Matrix<double, 3, 3>> J(4); // Initialize a vector of 4 matrices for the foot jacobians


    // Update the pinocchio model config q with the joint angles (doing this to make sure joint indices match)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model.nq);
    for (int foot_index = 0; foot_index < 4; foot_index++) {
        for (int joint_index = 0; joint_index < 3; joint_index++) {
            int joint_id = pinocchio_model.getJointId(joint_names_by_foot[foot_index][joint_index]);
            q[pinocchio_model.joints[joint_id].idx_q()] = theta[foot_index * 3 + joint_index];
        }
    }

    // FK to update model state
    pinocchio::forwardKinematics(pinocchio_model, pinocchio_data, q);
    pinocchio::updateFramePlacements(pinocchio_model, pinocchio_data);

    // Compute Jacobian for each foot
    for (int foot_index=0; foot_index < 4; foot_index++)
    {
        pinocchio::FrameIndex foot_frame_id = pinocchio_model.getFrameId(foot_names[foot_index]);

        Eigen::MatrixXd J_temp(6, pinocchio_model.nv);
        pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        // Construct the 3x3 Jacobian for each foot
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int joint_id = pinocchio_model.getJointId(joint_names_by_foot[foot_index][j]);
                J[foot_index](i, j) = J_temp(i, pinocchio_model.joints[joint_id].idx_v());
            }

        }
    }

    return J;
}

/**
 * @brief Solves for joint torques per foot to follow a positional reference in the world coordinate system.
 * @param q Joint configuration vector of the quadruped robot.
 * @param J_i_B Jacobian for foot i in the body frame.
 * @param p_i_ref_W Reference to track of foot i position in the world frame.
 * @param Kp Proportional gain
 * @param Kd Derivative gain
 * @param p_i_B Position of foot i in body frame.
 * @param v_i_B Velocity of foot i in body frame.
 * @return Vector of joint torques for foot i.
 */
Vector<double, 3> ConvexMPC::compute_swing_leg_tracking_torques(
    const Vector<double, 12>& q, const Vector<double, 12> q_i_dot,
    Matrix3d J_i_B, 
    Vector3d p_i_B, Vector3d v_i_B,
    Vector3d p_i_ref_B, Vector3d v_i_ref_B,
    Matrix3d Kp, Matrix3d Kd,
    Vector3d a_i_ref_B, 
    int foot_index)
{

    Matrix3d I_i_opspace = get_foot_operation_space_inertia_matrix(q, foot_index);
    Matrix3d J_i_B_dot = Matrix3d::Zero(); // TODO: Compute the time derivative of the Jacobian (if needed, otherwise can be set to zero)
    
    Matrix3d C_i = Matrix3d::Zero(); // TODO: compute via pinocchio
    Matrix3d G_i = Matrix3d::Zero(); // TODO: compute via pinocchio

    // Vector3d tau_i_ff = J_i_B.transpose() * I_i_opspace * (a_i_ref_B - J_i_B_dot * q_i_dot) + C_i + G_i;
    // Vector3d tau_i = J_i_B.transpose() * (Kp * (p_i_ref_B - p_i_B) + Kd * (v_i_ref_B - v_i_B)) + tau_i_ff;

    Vector3d tau_i = Vector3d::Zero();
    return tau_i;
}

/**
* @brief Computes the operation space inertia matrix for a foot in the world frame.
* @param q Joint configuration vector of the quadruped robot.
* @param foot_index Index of the foot for which the operation space inertia matrix is computed.
* @return The operation space inertia matrix for the specified foot.
*/
Matrix3d ConvexMPC::get_foot_operation_space_inertia_matrix(const Vector<double, 12>& q, int foot_index)
{
    vector<Matrix3d> J = get_foot_jacobians(this->theta);
    Matrix3d J_i = J[foot_index];
    cout << "Foot jacobian size " << J_i.rows() << " x " << J_i.cols() << endl;

    // Matrix<double, 12, 12> M = pinocchio::computeMassMatrix(pinocchio_model, pinocchio_data, q);
    Matrix<double, 3, 3> M = Matrix<double, 3, 3>::Identity(); // TODO: Compute the mass matrix using pinocchio

    Matrix3d I_opspace = (J_i * M.inverse() * J_i.transpose()).inverse();
    return I_opspace;
}


