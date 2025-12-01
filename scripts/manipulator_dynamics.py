import pinocchio 
import os
import numpy as np
import casadi as ca

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
    
    def casadi_dynamics_function(self):
        x_sym = ca.SX.sym('x', self.nq + self.nv)      # 37
        u_sym = ca.SX.sym('u', self.nu)                # 12
        lam_sym = ca.SX.sym('lam', self.nc)            # 12 (placeholder)

        q = x_sym[0:self.nq]
        dq = x_sym[self.nq:self.nq + self.nv]

        # Pinocchio expects numeric; wrap with ca.Callback or build via external for full symbolic.
        # Here we treat M,C,g as parameters by evaluating at q,v=0 (example placeholder).
        # For true symbolic you need manual expressions; this placeholder keeps shapes.
        M_num, C_num, g_num = self.get_manipulator_dynamics(np.zeros(self.nq + self.nv))
        M = ca.DM(M_num)
        C = ca.DM(C_num)
        g = ca.DM(g_num)

        u_full_sym = ca.vertcat(ca.DM.zeros(6), u_sym) # add zero 6doF forces for the free flyer joint
        print("u_full_sym length: ", u_full_sym.shape)
        ddq = ca.solve(M, u_full_sym - (C @ dq) - g)
        # qdot = ca.vertcat(dq[3:6], ca.DM.zeros(4), v[6:])
        
        dx = ca.vertcat(dq, ddq)
        # f = ca.Function('f', [x_sym, u_sym, lam_sym], [dx])
        f = ca.Function('f', [x_sym, u_sym], [dx])
        
        return f

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
        
        # Parse parameters
        self.N_MPC = N_MPC
        self.dt = dt
        self.nx = self.go2.nq + self.go2.nv
        self.nu = self.go2.nu
        
        self.nx_rigidbody = 7 # Number of dofs in rigid body pose
        
        
        self.T = N_MPC * dt # Time horizon

        # Declare model variables
        self.X = ca.SX.sym('X', self.go2.nq * N_MPC)
        self.U = ca.SX.sym('U', self.go2.nu * (N_MPC - 1))
        self.LAM = ca.SX.sym('LAM', self.go2.nc * N_MPC)

        self.q0 = np.zeros(self.go2.nq) # Current q0

        # Model equations

        # Objective term
        X_ref_rb = ca.SX.sym('X_ref_rb', self.nx_rigidbody * N_MPC) # Reference trajectory for the rigid body pose
        
        # TODO: Weights for Q and R
        Q_diag = 1.0 * ca.DM.eye(self.nx_rigidbody)
        Qf = 1.0 * ca.DM.eye(self.nx_rigidbody)
        R = 1.0 * ca.DM.eye(self.nu)
        
        f = self.go2.casadi_dynamics_function() # xdot = f(x,u,lam)
        
        # L = x1**2 + x2**2 + u**2
        self.go2.print_model_structure()
        
        p_WB, R_WB = self.go2.get_frame_pose(self.q0, "base")
        
        print(f"Base Link Frame Position: {p_WB}")
        print(f"Base Link Orientation: {R_WB}")

        # # Formulate discrete time dynamics
        # if False:
        #     # CVODES from the SUNDIALS suite
        #     dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
        #     F = integrator('F', 'cvodes', dae, 0, T/N)
        # else:
        
        # Fixed step Runge-Kutta 4 integrator
        M = 4 # RK4 steps per interval
        
        # f = ca.Function('f', [x, u], [xdot, L])
        x0 = ca.MX.sym('x0', self.nx)
        u0 = ca.MX.sym('u0', self.nu)
        
        xdot = f(x0, u0)
        print(xdot)
        # X = X0
        # Q = 0
        
        # f(X, U)
        
        for k in range(N_MPC - 1):
            
        pass

    def x_rb_k(self, k):
        """Returns the rigid-body pose () for the kth time index in the mpc horizon
        Args:
            k (int) : timestep index within mpc horizon
        """
        assert k <= self.N_MPC
        
        x_k = self.X[k*self.nx : (k+1)*self.nx]
        x_k_rb = x_k[:self.nx_rigidbody]
           
        return x_k_rb
    
    def u_k(self, k):
        assert k < self.N_MPC
        
        u_k = self.U[k * self.nu : (k+1) * self.nu]
        return u_k
    
    def rk4_step(f, xk, uk, dt):
        """Given 

        Args:
            f (_type_): _description_
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

    prob = Go2DirectMultipleShooting(N_MPC=10, dt=0.005, urdf_path=urdf_path)
    
