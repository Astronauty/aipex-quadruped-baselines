import pinocchio 
# import pinocchio.casadi as cpin
import os
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation as R
import time
import matplotlib.pyplot as plt

class URDFManipulatorDynamics:
    """Parse manipulator dynamics from a provided URDF and generate Casadi representations for use in OCPs.
    """
    def __init__(self, urdf_path, verbose=False):
        # urdf_path = "/src/convex_mpc/urdf/go2_description.urdf"
        # self.model = pinocchio.buildModelFromUrdf(urdf_path)
        self.verbose=verbose
        
        self.base_model = pinocchio.buildModelFromUrdf(urdf_path) # Adds a 6DoF Joint for floating base
        print("----------------------")
        print("Base Model Dimensions: ")
        
        print(f"nq:{self.base_model.nq}")
        print(f"nv:{self.base_model.nv}")
        # print(f"nu:{self.base_model.nu}")
        print()
        
        self.model = pinocchio.buildModelFromUrdf(urdf_path, pinocchio.JointModelFreeFlyer()) # Adds a 6DoF Joint for floating base
        print("----------------------")
        print("Floating Base Model: ")
        print(f"nq:{self.model.nq}")
        print(f"nv:{self.model.nv}")
        # print(f"nu:{self._model.nu}")
        print()
        self.data = self.model.createData()
        
        # for j in range(len(self.model.joints)):
        #     Ji = self.model.joints[j]
        #     print(j, Ji.shortname(), self.model.names[j], self.model.idx_vs[j], self.model.nvs[j])

        
        return None
    
    @property
    def nq(self):
        return self.model.nq
    
    @property
    def nv(self):
        return self.model.nv
    
    @property
    def nu(self):
        # return 18 # 6 free joints for free-flyer
        return self.model.nv - 6
    
    @property
    def nc(self):
        return 12
    
    def get_manipulator_dynamics(self, x):
        """_summary_

        Args:
            x (ca.SX): _description_

        Returns:
            _type_: _description_
        """
        q = x[0:self.nq]
        dq = x[self.nq:]
        
        assert len(q) == self.nq
        assert len(dq) == self.nv

        # assert len(q) == self.model.nq, f"q dimension: {len(q)} does not match urdf q dimension: {self.model.nq}"
        # assert len(dq) == self.model.nv, f"dq dimension: {len(dq)} does not match urdf dq dimension: {self.model.nv}"
        
        M = pinocchio.crba(self.model, self.data, q)               # Mass matrix
        C = pinocchio.computeCoriolisMatrix(self.model, self.data, q, dq)  # Coriolis matrix
        g = pinocchio.computeGeneralizedGravity(self.model, self.data, q)  # Gravity vector
        
        if self.verbose:
            print("Go2 Manipulator Dynamics")
            print("------------------------")
            print(f"nq: {self.nq}")
            print(f"nv: {self.nv}")
            print()
            print(f"M {M.shape}:\n", M)
            print()
            print(f"C {C.shape}:\n", C)
            print()
            # print("g:\n ", g)
            print(f"g {g.shape}:\n", g)
        
        
        return M, C, g
    
    def print_model_structure(self):
        print("Joints:")
        for j, joint in enumerate(self.model.joints):
            name = self.model.names[j]
            idx_v = self.model.idx_vs[j]      # starting velocity index
            nv_j = self.model.nvs[j]          # number of velocity dofs for this joint
            print(f"  joint_id={j:2d} name={name:20s} idx_v={idx_v:2d} nv={nv_j}")

        print("\nFrames (bodies only):")
        for i, f in enumerate(self.model.frames):
            if f.type == pinocchio.FrameType.BODY:
                # print(f"  frame_id={i:2d} body_name={f.name:20s} parent_joint={f.parent}")
                print(f"  frame_id={i:2d} body_name={f.name:20s} parent_joint={f.parentJoint}")
                
    
    def state_space_residual(self, x, u, lam):
        """Returns a dynamics residual in the form of state space dynamics. 
        xdot - f(x, u) = 0

        Args:
            x (_type_): state
            u (_type_): control
            
        Returns:
            dx = f(x, u, lam) : State space derivative
        """
        q = x[0:self.nq]
        dq = x[self.nq:]
        
        assert len(q) == self.nq

        
        M, C, g = self.get_manipulator_dynamics(x)
        
        tau = np.zeros(self.nv)
        tau[6:] = u
        
        # # Contact Jacobian
        # Jc = ca.DM.zeros(nc, nv)  # fill with FK jacobian of contact frames
        # tau_full = S @ u + Jc.T @ lam
        
        # Unconstrained manipulator dynamics
        ddq = ca.mtimes(ca.inv(M), -C @ dq + g + tau)
        
        dx = ca.vertcat(dq, ddq) # State space derivative

        # f = ca.Function('f', [x, u, lam], [dx])
        return dx
    @staticmethod
    def quat_dot_xyzw_from_omega(quat_xyzw, omega):
        qx, qy, qz, qw = quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]
        wx, wy, wz = omega[0], omega[1], omega[2]
        qx_dot = 0.5 * ( wx*qw + wy*qz - wz*qy )
        qy_dot = 0.5 * (-wx*qz + wy*qw + wz*qx )
        qz_dot = 0.5 * ( wx*qy - wy*qx + wz*qw )
        qw_dot = -0.5 * ( wx*qx + wy*qy + wz*qz )
        return ca.vertcat(qx_dot, qy_dot, qz_dot, qw_dot)
    
    
    
    # def casadi_dynamics_function(self, x_sym, u_sym):
    def casadi_dynamics_function(self):
        x_sym = ca.SX.sym('x', self.nq + self.nv)      # 37
        u_sym = ca.SX.sym('u', self.nu)                # 12
        # lam_sym = ca.SX.sym('lam', self.nc)            # 12 (placeholder)

        q = x_sym[0:self.nq] # 19
        dq = x_sym[self.nq:self.nq + self.nv] # 18

        # Pinocchio expects numeric; wrap with ca.Callback or build via external for full symbolic.
        # Here we treat M,C,g as parameters by evaluating at q,v=0 (example placeholder).
        # For true symbolic you need manual expressions; this placeholder keeps shapes.
        M_num, C_num, g_num = self.get_manipulator_dynamics(np.zeros(self.nq + self.nv))
        M = ca.DM(M_num)
        C = ca.DM(C_num)
        g = ca.DM(g_num)

        u_full_sym = ca.vertcat(ca.SX.zeros(6), u_sym) # add zero 6doF forces for the free flyer joint
        
        # print("u_full_sym length: ", u_full_sym.shape)
        ddq = ca.solve(M, u_full_sym - (C @ dq) - g)
        # qdot = ca.vertcat(dq[3:6], ca.DM.zeros(4), v[6:])
        
        # Solve for the quaternion derivative
        omega = dq[0:3] 
        v_lin = dq[3:6]
        quat = q[3:7]
        quat_dot = self.quat_dot_xyzw_from_omega(quat, omega)
        joint_pos_dot = dq[6:]
        qdot = ca.vertcat(v_lin, quat_dot, joint_pos_dot)  # (19,)

        
        # dx = ca.vertcat(dq, ddq)
        dx = ca.vertcat(qdot, ddq)
        # f = ca.Function('f', [x_sym, u_sym, lam_sym], [dx])
        f = ca.Function('f', [x_sym, u_sym], [dx])
        
        return f

    def discrete_step_integrate(self, dt):
        """
        Single step integration for a pinocchio config, handling quaternion derivatives. 
        q_{k+1} = integrate(q_k, v_k * dt)
        v_{k+1} = v_k + ddq_k * dt
        with forward dynamics: M ddq = tau - C v - g
        """
        # q = ca.SX('q', self.nq) # 19
        # dq = ca.SX('dq', self.nv) # 18
        x = ca.SX.sym('x', self.nq + self.nv)
        u = ca.SX.sym('u', self.nu) # 12
        
        q = x[0:self.nq]
        dq = x[self.nq: self.nq + self.nv]
        
        # Compute dynamics terms at (q,v)
        M = pinocchio.crba(self.model, self.data, q)
        C = pinocchio.computeCoriolisMatrix(self.model, self.data, q, dq)
        g = pinocchio.computeGeneralizedGravity(self.model, self.data, q)

        u_aug = ca.vertcat(ca.SX.zeros(6), u) # Pad with zeros for the free flyer joints

        # Forward dynamics
        ddq = np.linalg.solve(M, u_aug - C @ dq - g)

        # Integrate configuration using group operation (handles quaternion)
        q_next = pinocchio.integrate(self.model, q, dq * dt)

        # Explicit Euler for velocities
        dq_next = dq + ddq * dt
        x_next = ca.vertcat(q_next, dq_next)
        # return q_next, dq_next
        return ca.Function('f', [x, u], [x_next], ['x_k', 'u_k'], ['x_kp1'])
    
    def get_frame_pose(self, q, frame_name):
        assert len(q) == self.nq
        
        fid = self.model.getFrameId(frame_name)
        pinocchio.forwardKinematics(self.model, self.data, q)
        pinocchio.updateFramePlacements(self.model, self.data)
        
        T = self.data.oMf[fid]
        pos = T.translation
        R = T.rotation
        return pos, R
        

    @property
    def S(active_legs):
        """_summary_
        Returns a matrix S which selects leg torques which may be active or not.
        Args:
            active_leg_index (ndarray): Index ranging from 0-3 w
            
        Returns:
        
        """
        I = np.diag([1, 1, 1]) 
        
        S = np.array()
        
        return S
    


 
class Go2DirectMultipleShooting:
    def __init__(self, N_MPC : int, dt : float, urdf_path):
        # Create the dynamics model for the go2
        self.go2= URDFManipulatorDynamics(urdf_path)
        self.go2.print_model_structure()
        
        # Parse parameters
        self.N_MPC = N_MPC
        self.dt = dt
        self.nx = self.go2.nq + self.go2.nv
        self.nu = self.go2.nu
        self.nx_rigidbody = 7 # Number of dofs in rigid body pose
        
        self.T = N_MPC * dt # Time horizon

        # Declare model variables
        self.X = ca.SX.sym('X', self.nx * N_MPC)
        self.U = ca.SX.sym('U', self.nu * (N_MPC - 1))
        self.LAM = ca.SX.sym('LAM', self.go2.nc * N_MPC)

        self.q0 = np.zeros(self.go2.nq) # Current q0

        # Objective term
        self.X_ref_rb = ca.SX.sym('X_ref_rb', self.nx_rigidbody * N_MPC) # Reference trajectory for the rigid body pose (note that it is only defined with the rigid body states)

        # TODO: Weights for Q and R
        Q = 1.0 * ca.DM.eye(self.nx_rigidbody)
        Qf = 1.0 * ca.DM.eye(self.nx_rigidbody)
        R = 1.0 * ca.DM.eye(self.nu)
        
        J = 0
        for k in range(N_MPC - 1):
            x_rb_k = self.get_kth_rigid_body_state(self.X, k) # kth rigid body state
            x_ref_rb_k = self.get_state_k(self.X_ref_rb, self.nx_rigidbody, k) # kth rigid body state from ref traj
            e_rb_k = x_ref_rb_k - x_rb_k # Rigid body pose error for this timestep
            
            u_k = self.get_state_k(self.U, self.nu, k)
            
            J += ca.mtimes([e_rb_k.T, Q, e_rb_k]) + ca.mtimes([u_k.T, R, u_k])
        
        self.J_fun = ca.Function("J", [self.X_ref_rb, self.X, self.U], [J], ['X_ref_rb', 'X', 'U'], ['J'])
        
        
        ### Test the objective function w/ dummy values
        X0 = ca.DM.zeros(self.nx * self.N_MPC)
        U0 = ca.DM.zeros(self.nu * (self.N_MPC - 1))
        Xref_rb0 = ca.DM(np.tile([0,0,0,0,0,0, 1.0], self.N_MPC))
        J_val = self.J_fun(Xref_rb0, X0, U0)
        print(f"J: {float(J_val)}")
        
        
        ### Dynamics function
        p_WB, R_WB = self.go2.get_frame_pose(self.q0, "base")
        print(R_WB)
        
        # xk = ca.SX.sym('xk', self.nx)
        # uk = ca.SX.sym('uk', self.nu)
        xk = self.get_state_k(self.X, self.nx, 0)
        uk = self.get_state_k(self.U, self.nu, 0)

        f_continuous_dynamics = self.go2.casadi_dynamics_function() # Continuous dynamics funciton dx = f(x,u)
        
        
        ### Equality constraints
        # Multiple shooting constraints
        g_list = []
        for k in range(self.N_MPC-1):
            x_k = self.get_state_k(self.X, self.nx, k)
            u_k = self.get_state_k(self.U, self.nu, k)
            
            x_kp1 = self.get_state_k(self.X, self.nx, k+1)
            x_kp1_pred = self.rk4_step(f_continuous_dynamics, x_k, u_k, self.dt) # Dynamics rollout 
            g_list.append(x_kp1 - x_kp1_pred)
            
        G = ca.vertcat(*g_list)
        
        # Dynamics equlaity constraints
        self.lbg = ca.DM.zeros(self.nx * (self.N_MPC - 1))
        self.ubg = ca.DM.zeros(self.nx * (self.N_MPC - 1))
        
        
        # Primal bounds
        x_lb = -1e2 * ca.DM.ones(self.nx * self.N_MPC)
        x_ub =  1e2 * ca.DM.ones(self.nx * self.N_MPC)
        u_lb = -1e2 * ca.DM.ones(self.nu * (self.N_MPC - 1))
        u_ub =  1e2 * ca.DM.ones(self.nu * (self.N_MPC - 1))
        self.lbx = ca.vertcat(x_lb, u_lb)
        self.ubx = ca.vertcat(x_ub, u_ub)
        
        
        nlp = {
            'x': ca.vertcat(self.X, self.U),
            'f': J,
            'g': G,
            'p': self.X_ref_rb
        }
        
        
    
        self.g_lb = ca.DM.zeros(self.nx * (self.N_MPC - 1))
        self.g_ub = ca.DM.zeros(self.nx * (self.N_MPC - 1))
        
        
        ### Solver settings
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, {
            'ipopt.print_level': 0,
            'print_time': False,
            'ipopt.max_iter': 200
        })
        
        # Decision variable initialization 
        w0 = ca.vertcat(ca.DM.zeros(self.nx * self.N_MPC),
                ca.DM.zeros(self.nu * (self.N_MPC - 1)))
        
        # Xref_rb0 = ca.DM(np.tile([0,0,0, 0,0,0, 1.0], self.N_MPC)).reshape((-1,1))
        # Xref_rb0 = ca.DM(np.tile([0.0, 0.0, 0.31232, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], self.N_MPC)).rehsape((-1, 1))
        Xref_rb0 = ca.DM(np.tile([0.0, 0.0, 0.31232, 0.0, 0.0, 0.0, 1.0], self.N_MPC)).reshape((-1, 1))
        
        
        iters = 500
        
        solve_times =[]
  
        for i in range(iters):
            t0 = time.time()
            sol = self.solver(x0=w0, lbg=self.lbg, ubg=self.ubg, lbx=self.lbx, ubx=self.ubx, p=Xref_rb0)
            t1 = time.time()
            solve_times.append(t1 - t0)
            # stats = self.solver.stats()
            # print(stats)
            # print(stats["t_wall_total"])
            
        print(f"Avg solve time: {np.mean(solve_times):.6f} s, std: {np.std(solve_times):.6f} s")
            
        # Histogram plot
        plt.figure(figsize=(6,4))
        
        mu = np.mean(solve_times)
        sigma = np.std(solve_times)
        
        legend_text = f"mean = {mu:.6f}s\nstd  = {sigma:.6f}s"
        # Dummy handle for text-only legend




        # plt.figure(figsize=(6,4))
        plt.hist(solve_times, bins=30, edgecolor='k')
        plt.xscale('log')
        
        plt.plot([], [], ' ', label=legend_text)
        plt.legend(loc='upper right', frameon=True)
        
        
        plt.xlabel('Solve time (s)')
        plt.ylabel('Count')
        plt.title('Nonlinear MPC (IPOPT) Solve Time (s)')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('solve_times_hist.png', dpi=500)
        plt.show()
        
        
        


    
    def get_state_k(self, X, nx, k):
        """Returns a partition of the vector X, from X[k*nx : (k+1)*nx]

        Args:
            X (_type_): _description_
            nx (_type_): _description_
            k (_type_): _description_

        Returns:
            _type_: _description_
        """
        x_k = X[k*nx : (k+1)*nx]
        return x_k

    def get_kth_rigid_body_state(self, X, k):
        """Returns the rigid-body pose (x, y, z, a, b, c, d) for the kth time index in the mpc horizon
        Args:
            k (int) : timestep index within mpc horizon
        """
        assert k <= self.N_MPC
        
        x_k = X[k*self.nx : (k+1)*self.nx]
        x_k_rb = x_k[:self.nx_rigidbody]
           
        return x_k_rb
    

    # def get_contro_k(self, k):
    #     assert k < self.N_MPC
        
    #     u_k = self.U[k * self.nu : (k+1) * self.nu]
    #     return u_k
    
    def rk4_step(self, f, xk, uk, dt):
        """Given 

        Args:
            f (_type_): function derivative
            x (_type_): _description_
            u (_type_): _description_
            dt (_type_): _description_
            
        Returns
        """
        k1 = f(xk, uk)
        k2 = f(xk + dt/2 * k1, uk)
        k3 = f(xk + dt/2 * k2, uk)
        k4 = f(xk + dt * k3, uk)
        xkp1=xk+dt/6*(k1 +2*k2 +2*k3 +k4)
        
        return xkp1

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(current_dir, "../src/convex_mpc/urdf/go2_description.urdf")
    
    
    # ### URDF PARSE TEST
    # go2= URDFManipulatorDynamics(urdf_path)
    
    # # Create dummy joint config
    # # q = np.zeros(go2.nq)
    # # dq = np.zeros(go2.nv)
    # x = np.zeros(go2.nq + go2.nv)
    
    # u = np.zeros(12)
    # lam = np.zeros(12)
    
    # # go2.get_manipulator_dynamics()
    # # print(go2.nu)
    # print(go2.state_space_residual(x, u, lam))

    prob = Go2DirectMultipleShooting(N_MPC=10, dt=0.025, urdf_path=urdf_path)
    
