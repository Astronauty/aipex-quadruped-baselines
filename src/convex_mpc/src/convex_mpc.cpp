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
        env->start();
        model = make_unique<GRBModel>(*env);

        // Initialize robot state
        x0 = Vector<double, 13>::Zero();
        // u = Vector<double, 12>::Zero();
        ground_reaction_forces = Matrix<double, 3, 4>::Zero(); // Initialize GRFs for the 4 feet of the quadruped robot
        x_ref = VectorXd::Ones(mpc_params.N_STATES*mpc_params.N_MPC);
        theta = Vector<double, 12>::Zero(); // Initialize joint angles of the quadruped robot

        // Initialize pinocchio model & data (for foot jacobian computaiton)
        string urdf_path = PINOCCHIO_MODEL_DIR "urdf/go2_description.urdf";
        pinocchio::urdf::buildModel(urdf_path, pinocchio_model);
        pinocchio_data = pinocchio::Data(pinocchio_model);
        
        // Initialize state space prediction matrices
        StateSpace quad_dss = get_default_dss_model(); // TODO: proper initialization, perhaps based on the initial state of robot?
        tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);

        // Formulate the QP 
        U = model->addVars(mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1), GRB_CONTINUOUS);
        cout << "Number of Decision Variables:" << mpc_params.N_CONTROLS * (mpc_params.N_MPC - 1) << endl;


        Q_bar = compute_Q_bar(); // Diagonal block matrix of quadratic state cost for N_MPC steps
        // cout << "Size of Q_bar: " << Q_bar.rows() << " x " << Q_bar.cols() << endl;

        R_bar = compute_R_bar(); // Diagonal block matrix of quadratic control cost for N_MPC steps
        // cout << "Size of R_bar: " << R_bar.rows() << " x " << R_bar.cols() << endl;

        P = compute_P(R_bar, Q_bar, A_qp); // Quadratic cost of linear mpc
        // cout << "Size of P: " << P.rows() << " x " << P.cols() << endl;

        q = compute_q(Q_bar, A_qp, B_qp, x0, x_ref);
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
 * X = A_qp * U + B_qp * x0
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
    Matrix<double, 4, 3> foot_positions;
    foot_positions << 0.1, 0.1, 0,
                      0.1, -0.1, 0,
                      -0.1, 0.1, 0,
                      -0.1, -0.1, 0;


    double yaw = 0.0f;
    StateSpace quad_dss = quadruped_state_space_discrete(yaw, foot_positions, this->mpc_params.dt);
    RCLCPP_INFO(logger_, "Default discrete state space model created.");
    RCLCPP_INFO(logger_, "A Size: %ldx%ld", quad_dss.A.rows(), quad_dss.A.cols());
    RCLCPP_INFO(logger_, "B Size: %ldx%ld", quad_dss.B.rows(), quad_dss.B.cols());


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
    // return blockDiagonal;
    return MatrixXd::Identity(totalRows, totalCols);
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
    // int N_CONTROLS = mpc_params.N_CONTROLS;
    int N_MPC = mpc_params.N_MPC;


    // cout << "Size of Q: " << Q.rows() << " x " << Q.cols() << endl;
    // cout << "mpc_params.Q:\n" << Q << endl;
    std::vector<MatrixXd> Q_vec(N_MPC, Q);
    // cout << "here 4" << endl;
    MatrixXd Q_bar = blkdiag(Q_vec);
    // cout << "here 5" << endl;
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

VectorXd ConvexMPC::compute_q(MatrixXd Q_bar, MatrixXd A_qp, MatrixXd B_qp, VectorXd x0, VectorXd x_ref)
{
    // VectorXd q = 2*x0.transpose() * B_qp * Q_bar.transpose() * A_qp; // Zero reference linear cost
    VectorXd q = 2*(x0.transpose() * B_qp.transpose() * Q_bar * A_qp - x_ref.transpose() * Q_bar * A_qp);
    return q;
}

void ConvexMPC::update_x0(Vector<double, 13> x0)
{
    // Update the initial state vector x0
    this->x0 = x0;
    this->q = compute_q(Q_bar, A_qp, B_qp, this->x0, x_ref);

    lin_expr = create_lin_obj(U, q, mpc_params.N_STATES); // Only the linear part of the objective is influenced by x0
    this->model->setObjective(quad_expr + lin_expr, GRB_MINIMIZE); // Update the MPC cost with new x0
}

void ConvexMPC::update_joint_angles(Vector<double, 12> theta)
{
    this->theta = theta;

}

Vector<double, 12> ConvexMPC::solve_joint_torques()
{
    model->optimize(); // Resolve to capture any changes made to model

    // Extract ground reaction forces from Gurobi solution
    Matrix<double, 3, 4> grf = Matrix<double, 3, 4>::Zero();
    for (int foot = 0; foot < 4; foot++) {
        for (int axis = 0; axis < 3; axis++) {
            int idx = foot * 3 + axis;
            grf(axis, foot) = U[idx].get(GRB_DoubleAttr_X);
            RCLCPP_INFO(logger_, "GRF[foot %d, axis %d] = %f", foot, axis, grf(axis, foot));
        }
    }

    // Get the foot jacobian
    vector<Matrix3d> foot_jacobians = get_foot_jacobians(this->theta);

    // Compute joint torques by utilizing the foot jacobians
    Vector<double, 12> joint_torques;

    double roll = x0[0];
    double pitch = x0[1];
    double yaw = x0[2];

    Matrix3d R_WB = (AngleAxisd(roll, Vector3d::UnitX()) * 
           AngleAxisd(pitch, Vector3d::UnitY()) * 
           AngleAxisd(yaw, Vector3d::UnitZ())).toRotationMatrix(); // Body frame orientation in world frame

    for (int foot = 0; foot < 4; foot++)
    {
        Vector3d foot_grf = grf.col(foot); // Extract the grf for the current foot (world frame)
        
        Vector3d foot_joint_torques = foot_jacobians[foot].transpose() * R_WB.transpose() * foot_grf; // Compute joint torques for the current foot (correpsonding to its 3 actuators)
        joint_torques.segment<3>(foot * 3) = foot_joint_torques; // Store the joint torques in the joint_torques vector
    }

    return joint_torques;
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



// int main(int, char**)
// {
//     // Define the parameters for the MPC and the quadruped
//     int N_MPC = 5;
//     double dt = 0.01;
//     int N_STATES = 13;
//     int N_CONTROLS = 12;

//     // Define MPC Params
//     MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES);
//     MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS);
//     VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -1.0);
//     VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 1.0);
//     MPCParams mpc_params = MPCParams(N_MPC, N_CONTROLS, N_STATES,
//          dt, Q, R, u_lower, u_upper);

//     // Define Quadruped Params
//     Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
//     double mass = 1.0;
//     double gravity = 9.81;
//     QuadrupedParams quadruped_params = QuadrupedParams(inertiaTensor, mass, gravity);

//     // Initialize the ConvexMPC object
//     ConvexMPC convex_mpc = ConvexMPC(mpc_params, quadruped_params);

//     // Example of creating state space prediction matrices
//     // StateSpace quad_dss = convex_mpc.get_default_dss_model(); // TODO: Implement actual updates to the dss based on yaw and foot position
//     // auto [A_qp, B_qp] = convex_mpc.create_state_space_prediction_matrices(quad_dss);

//     // // Print the matrices to verify

//     // cout << "A_qp: \n" << A_qp << endl;
//     // cout << "B_qp: \n" << B_qp << endl;
//     // cout << "Size of A_qp: " << A_qp.rows() << " x " << A_qp.cols() << endl;
//     // cout << "Size of B_qp: " << B_qp.rows() << " x " << B_qp.cols() << endl;

//     return 0;
// }
