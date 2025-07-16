import numpy as np
import osqp
import scipy.sparse as sp

# Define the P matrix (24x24)
P = np.array([
    [1.81339,  -2.45829,  0.476189,  -1.76541,  -2.45829, -0.681686,   1.81339,   2.41594,  0.681686,  -1.76541,   2.41594, -0.476189,  0.905572,  -1.22762,    0.2378, -0.881615,  -1.22762, -0.340421,  0.905572,   1.20647,  0.340421, -0.881615,   1.20647,   -0.2378],
    [-2.45829,   3.44411,   -2.0438,   2.45864,   3.44411,    2.0309,  -2.45829,  -3.25261,   -2.0309,   2.45864,  -3.25261,    2.0438,  -1.22762,   1.71992,  -1.02064,    1.2278,   1.71992,   1.01419,  -1.22762,  -1.62429,  -1.01419,    1.2278,  -1.62429,   1.02064],
    [0.476189,   -2.0438,    38.165,  -0.69116,   -2.0438,  -30.1183,  0.476189, -0.453904,   30.1607,  -0.69116, -0.453904,  -38.1226,    0.2378,  -1.02064,   19.0589, -0.345153,  -1.02064,  -15.0405,    0.2378, -0.226671,   15.0617, -0.345153, -0.226671,  -19.0377],
    [-1.76541,   2.45864,  -0.69116,   1.81369,   2.45864,  0.474472,  -1.76541,    -2.416, -0.474472,   1.81369,    -2.416,   0.69116, -0.881615,    1.2278, -0.345153,  0.905723,    1.2278,  0.236942, -0.881615,  -1.20651, -0.236942,  0.905723,  -1.20651,  0.345153],
    [-2.45829,   3.44411,   -2.0438,   2.45864,   3.44411,    2.0309,  -2.45829,  -3.25261,   -2.0309,   2.45864,  -3.25261,    2.0438,  -1.22762,   1.71992,  -1.02064,    1.2278,   1.71992,   1.01419,  -1.22762,  -1.62429,  -1.01419,    1.2278,  -1.62429,   1.02064],
    [-0.681686,    2.0309,  -30.1183,  0.474472,    2.0309,   37.8762, -0.681686,  0.456243,  -37.8337,  0.474472,  0.456243,   30.1607, -0.340421,   1.01419,  -15.0405,  0.236942,   1.01419,   18.9147, -0.340421,  0.227839,  -18.8935,  0.236942,  0.227839,   15.0617],
    [1.81339,  -2.45829,  0.476189,  -1.76541,  -2.45829, -0.681686,   1.81339,   2.41594,  0.681686,  -1.76541,   2.41594, -0.476189,  0.905572,  -1.22762,    0.2378, -0.881615,  -1.22762, -0.340421,  0.905572,   1.20647,  0.340421, -0.881615,   1.20647,   -0.2378],
    [2.41594,  -3.25261, -0.453904,    -2.416,  -3.25261,  0.456243,   2.41594,   3.32835, -0.456243,    -2.416,   3.32835,  0.453904,   1.20647,  -1.62429, -0.226671,  -1.20651,  -1.62429,  0.227839,   1.20647,   1.66212, -0.227839,  -1.20651,   1.66212,  0.226671],
    [0.681686,   -2.0309,   30.1607, -0.474472,   -2.0309,  -37.8337,  0.681686, -0.456243,   37.8762, -0.474472, -0.456243,  -30.1183,  0.340421,  -1.01419,   15.0617, -0.236942,  -1.01419,  -18.8935,  0.340421, -0.227839,   18.9147, -0.236942, -0.227839,  -15.0405],
    [-1.76541,   2.45864,  -0.69116,   1.81369,   2.45864,  0.474472,  -1.76541,    -2.416, -0.474472,   1.81369,    -2.416,   0.69116, -0.881615,    1.2278, -0.345153,  0.905723,    1.2278,  0.236942, -0.881615,  -1.20651, -0.236942,  0.905723,  -1.20651,  0.345153],
    [2.41594,  -3.25261, -0.453904,    -2.416,  -3.25261,  0.456243,   2.41594,   3.32835, -0.456243,    -2.416,   3.32835,  0.453904,   1.20647,  -1.62429, -0.226671,  -1.20651,  -1.62429,  0.227839,   1.20647,   1.66212, -0.227839,  -1.20651,   1.66212,  0.226671],
    [-0.476189,    2.0438,  -38.1226,   0.69116,    2.0438,   30.1607, -0.476189,  0.453904,  -30.1183,   0.69116,  0.453904,    38.165,   -0.2378,   1.02064,  -19.0377,  0.345153,   1.02064,   15.0617,   -0.2378,  0.226671,  -15.0405,  0.345153,  0.226671,   19.0589],
    [0.905572,  -1.22762,    0.2378, -0.881615,  -1.22762, -0.340421,  0.905572,   1.20647,  0.340421, -0.881615,   1.20647,   -0.2378,  0.902208,  -1.22306,  0.236916,  -0.87834,  -1.22306, -0.339156,  0.902208,   1.20199,  0.339156,  -0.87834,   1.20199, -0.236916],
    [-1.22762,   1.71992,  -1.02064,    1.2278,   1.71992,   1.01419,  -1.22762,  -1.62429,  -1.01419,    1.2278,  -1.62429,   1.02064,  -1.22306,   1.71353,  -1.01684,   1.22324,   1.71353,   1.01042,  -1.22306,  -1.61826,  -1.01042,   1.22324,  -1.61826,   1.01684],
    [0.2378,  -1.02064,   19.0589, -0.345153,  -1.02064,  -15.0405,    0.2378, -0.226671,   15.0617, -0.345153, -0.226671,  -19.0377,  0.236916,  -1.01684,   18.9881,  -0.34387,  -1.01684,  -14.9846,  0.236916, -0.225829,   15.0058,  -0.34387, -0.225829,   -18.967],
    [-0.881615,    1.2278, -0.345153,  0.905723,    1.2278,  0.236942, -0.881615,  -1.20651, -0.236942,  0.905723,  -1.20651,  0.345153,  -0.87834,   1.22324,  -0.34387,  0.902358,   1.22324,  0.236062,  -0.87834,  -1.20202, -0.236062,  0.902358,  -1.20202,   0.34387],
    [-1.22762,   1.71992,  -1.02064,    1.2278,   1.71992,   1.01419,  -1.22762,  -1.62429,  -1.01419,    1.2278,  -1.62429,   1.02064,  -1.22306,   1.71353,  -1.01684,   1.22324,   1.71353,   1.01042,  -1.22306,  -1.61826,  -1.01042,   1.22324,  -1.61826,   1.01684],
    [-0.340421,   1.01419,  -15.0405,  0.236942,   1.01419,   18.9147, -0.340421,  0.227839,  -18.8935,  0.236942,  0.227839,   15.0617, -0.339156,   1.01042,  -14.9846,  0.236062,   1.01042,   18.8444, -0.339156,  0.226993,  -18.8233,  0.236062,  0.226993,   15.0058],
    [0.905572,  -1.22762,    0.2378, -0.881615,  -1.22762, -0.340421,  0.905572,   1.20647,  0.340421, -0.881615,   1.20647,   -0.2378,  0.902208,  -1.22306,  0.236916,  -0.87834,  -1.22306, -0.339156,  0.902208,   1.20199,  0.339156,  -0.87834,   1.20199, -0.236916],
    [1.20647,  -1.62429, -0.226671,  -1.20651,  -1.62429,  0.227839,   1.20647,   1.66212, -0.227839,  -1.20651,   1.66212,  0.226671,   1.20199,  -1.61826, -0.225829,  -1.20202,  -1.61826,  0.226993,   1.20199,   1.65594, -0.226993,  -1.20202,   1.65594,  0.225829],
    [0.340421,  -1.01419,   15.0617, -0.236942,  -1.01419,  -18.8935,  0.340421, -0.227839,   18.9147, -0.236942, -0.227839,  -15.0405,  0.339156,  -1.01042,   15.0058, -0.236062,  -1.01042,  -18.8233,  0.339156, -0.226993,   18.8444, -0.236062, -0.226993,  -14.9846],
    [-0.881615,    1.2278, -0.345153,  0.905723,    1.2278,  0.236942, -0.881615,  -1.20651, -0.236942,  0.905723,  -1.20651,  0.345153,  -0.87834,   1.22324,  -0.34387,  0.902358,   1.22324,  0.236062,  -0.87834,  -1.20202, -0.236062,  0.902358,  -1.20202,   0.34387],
    [1.20647,  -1.62429, -0.226671,  -1.20651,  -1.62429,  0.227839,   1.20647,   1.66212, -0.227839,  -1.20651,   1.66212,  0.226671,   1.20199,  -1.61826, -0.225829,  -1.20202,  -1.61826,  0.226993,   1.20199,   1.65594, -0.226993,  -1.20202,   1.65594,  0.225829],
    [-0.2378,   1.02064,  -19.0377,  0.345153,   1.02064,   15.0617,   -0.2378,  0.226671,  -15.0405,  0.345153,  0.226671,   19.0589, -0.236916,   1.01684,   -18.967,   0.34387,   1.01684,   15.0058, -0.236916,  0.225829,  -14.9846,   0.34387,  0.225829,   18.9881]
])

# Ensure P is symmetric (due to numerical issues, sometimes symmetry is slightly off)
# P = 0.5 * (P + P.T)

# Convert P to sparse matrix
P_sparse = sp.csc_matrix(P)

# Define q vector
q = np.array([
    1.18691e-10, 0, -3.57967, 1.18691e-10, 0, -3.57967,
    1.18691e-10, 0, -3.57967, 1.18691e-10, 0, -3.57967,
    4.45091e-11, 0, -2.1446,  4.45091e-11, 0, -2.1446,
    4.45091e-11, 0, -2.1446,  4.45091e-11, 0, -2.1446
])

print("=== QP Matrix Analysis ===")
print(f"P matrix shape: {P.shape}")
print(f"q vector shape: {q.shape}")

# Check if P is positive semidefinite
eigenvalues = np.linalg.eigvals(P)
print(f"P matrix eigenvalues:")
print(f"  Min eigenvalue: {np.min(eigenvalues):.6f}")
print(f"  Max eigenvalue: {np.max(eigenvalues):.6f}")
print(f"  Number of negative eigenvalues: {np.sum(eigenvalues < -1e-12)}")
print(f"  Number of zero eigenvalues: {np.sum(np.abs(eigenvalues) < 1e-12)}")
print(f"  Number of positive eigenvalues: {np.sum(eigenvalues > 1e-12)}")

# Check condition number
if np.min(eigenvalues) > 1e-12:
    cond_num = np.linalg.cond(P)
    print(f"P matrix condition number: {cond_num:.2e}")
else:
    print("P matrix is singular or indefinite - cannot compute condition number")

# Check if matrix is symmetric
symmetry_error = np.max(np.abs(P - P.T))
print(f"Matrix symmetry error: {symmetry_error:.2e}")

# Problem diagnosis
if np.min(eigenvalues) < -1e-12:
    print("❌ PROBLEM: P matrix is indefinite (has negative eigenvalues)")
    print("   This makes the QP non-convex!")
    
    # Try to fix by adding regularization
    print("\n=== Attempting to Fix P Matrix ===")
    regularization = abs(np.min(eigenvalues)) + 1e-6
    P_fixed = P + regularization * np.eye(P.shape[0])
    
    # Check fixed matrix
    eigenvalues_fixed = np.linalg.eigvals(P_fixed)
    print(f"After regularization (+{regularization:.6f}*I):")
    print(f"  Min eigenvalue: {np.min(eigenvalues_fixed):.6f}")
    print(f"  Max eigenvalue: {np.max(eigenvalues_fixed):.6f}")
    
    if np.min(eigenvalues_fixed) > 1e-12:
        print("✅ Fixed! P matrix is now positive definite")
        P = P_fixed
    else:
        print("❌ Still not positive definite")
        
elif np.min(eigenvalues) < 1e-12:
    print("⚠️  WARNING: P matrix is positive semidefinite (has zero eigenvalues)")
    print("   This might cause numerical issues")
    
    # Add small regularization
    regularization = 1e-8
    P = P + regularization * np.eye(P.shape[0])
    print(f"Added small regularization: {regularization}")
    
else:
    print("✅ P matrix is positive definite - good!")

print("\n=== QP Solution ===")

# Convert P to sparse matrix
P_sparse = sp.csc_matrix(P)

# Setup constraints (no constraints)
n = len(q)
A = sp.csc_matrix((0, n))
l = np.array([])
u = np.array([])

try:
    # Setup OSQP solver
    solver = osqp.OSQP()
    solver.setup(P=P_sparse, q=q, A=A, l=l, u=u, verbose=True)
    
    # Solve the problem
    result = solver.solve()
    
    print("✅ Solution found!")
    print("Optimal solution x:")
    
    # Method 1: Print as 2 timesteps × 12 forces
    x_reshaped = result.x.reshape(2, 12)
    for i in range(x_reshaped.shape[0]):
        print(f"  Timestep {i}: {x_reshaped[i]}")
    
    print("\n--- Organized by Foot ---")
    # Method 2: Reshape as 2 timesteps × 4 feet × 3 forces
    x_by_foot = result.x.reshape(2, 4, 3)
    for timestep in range(2):
        print(f"Timestep {timestep}:")
        for foot in range(4):
            fx, fy, fz = x_by_foot[timestep, foot, :]
            print(f"  Foot {foot}: fx={fx:.6f}, fy={fy:.6f}, fz={fz:.6f}")
    
    print(f"\nOptimal objective value: {result.info.obj_val}")
    
    # Check if solution makes sense
    nonzero_forces = np.sum(np.abs(result.x) > 1e-6)
    print(f"Number of non-zero forces: {nonzero_forces}/{len(result.x)}")
    
    # Analyze the pattern - check if only first timestep has forces
    timestep_0_forces = np.sum(np.abs(result.x[:12]) > 1e-6)
    timestep_1_forces = np.sum(np.abs(result.x[12:]) > 1e-6)
    print(f"Timestep 0 non-zero forces: {timestep_0_forces}/12")
    print(f"Timestep 1 non-zero forces: {timestep_1_forces}/12")
    
    if timestep_0_forces > 0 and timestep_1_forces == 0:
        print("⚠️  WARNING: Only first timestep has forces - same pattern as your MPC!")
        print("This suggests the P matrix or constraints are causing the issue.")
    
    # Additional analysis
    print("\n--- Force Analysis ---")
    print(f"Total vertical force (timestep 0): {np.sum(x_by_foot[0, :, 2]):.6f}")
    print(f"Total vertical force (timestep 1): {np.sum(x_by_foot[1, :, 2]):.6f}")
    
    print(f"Max force magnitude: {np.max(np.abs(result.x)):.6f}")
    print(f"Min force magnitude: {np.min(np.abs(result.x)):.6f}")
    
    # Check q vector influence
    print("\n--- Q Vector Analysis ---")
    print("q vector by timestep and foot:")
    q_by_foot = q.reshape(2, 4, 3)
    for timestep in range(2):
        print(f"Timestep {timestep}:")
        for foot in range(4):
            qx, qy, qz = q_by_foot[timestep, foot, :]
            print(f"  Foot {foot}: qx={qx:.2e}, qy={qy:.2e}, qz={qz:.6f}")
    
except Exception as e:
    print(f"❌ Solver failed: {e}")
    print("This indicates a fundamental problem with the QP formulation")