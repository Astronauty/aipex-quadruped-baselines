import pinocchio 
import os
import numpy as np
import casadi as ca

class URDFManipulatorDynamics:
    def __init__(self, urdf_path):
        # urdf_path = "/src/convex_mpc/urdf/go2_description.urdf"
        # self.model = pinocchio.buildModelFromUrdf(urdf_path)
        
        self.base_model = pinocchio.buildModelFromUrdf(urdf_path) # Adds a 6DoF Joint for floating base
        print("Base Model: ")
        print(f"nq:{self.base_model.nq}")
        print(f"nv:{self.base_model.nv}")
        # print(f"nu:{self.base_model.nu}")
        print()
        
        self.model = pinocchio.buildModelFromUrdf(urdf_path, pinocchio.JointModelFreeFlyer()) # Adds a 6DoF Joint for floating base
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
        return 18 # 6 free joints for free-flyer
    
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
        
        
        # Jacobian
        

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
        v = x_sym[self.nq:self.nq + self.nv]

        # Pinocchio expects numeric; wrap with ca.Callback or build via external for full symbolic.
        # Here we treat M,C,g as parameters by evaluating at q,v=0 (example placeholder).
        # For true symbolic you need manual expressions; this placeholder keeps shapes.
        M_num, C_num, g_num = self.get_manipulator_dynamics(np.zeros(self.nq + self.nv))
        M = ca.DM(M_num)
        C = ca.DM(C_num)
        g = ca.DM(g_num)

        tau = ca.vertcat(ca.DM.zeros(6), u_sym)
        a = ca.solve(M, tau - C @ v - g)
        qdot = ca.vertcat(v[3:6], ca.DM.zeros(4), v[6:])
        dx = ca.vertcat(qdot, a)
        f = ca.Function('f', [x_sym, u_sym, lam_sym], [dx])
        return f



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
        # Parse parameters
        self.N_MPC = N_MPC
        self.dt = dt
        
        T = N_MPC * dt # Time horizon
        
        # Create the dynamics model for the go2
        go2= URDFManipulatorDynamics(urdf_path)

        # Declare model variables
        X = ca.SX.sym('X', go2.nq)
        U = ca.SX.sym('U', go2.nu)
        LAM = ca.SX.sym('LAM', go2.nc)

        # Model equations
        xdot = go2.casadi_dynamics_function()

#         # Objective term
#         L = x1**2 + x2**2 + u**2

#         # Formulate discrete time dynamics
#         if False:
#             # CVODES from the SUNDIALS suite
#             dae = {'x':x, 'p':u, 'ode':xdot, 'quad':L}
#             F = integrator('F', 'cvodes', dae, 0, T/N)
#         else:
#         # Fixed step Runge-Kutta 4 integrator
#         M = 4 # RK4 steps per interval
#         DT = T/N/M
#         f = Function('f', [x, u], [xdot, L])
#         X0 = MX.sym('X0', 2)
#         U = MX.sym('U')
#         X = X0
#         Q = 0
#         for j in range(M):
#             k1, k1_q = f(X, U)
#             k2, k2_q = f(X + DT/2 * k1, U)
#             k3, k3_q = f(X + DT/2 * k2, U)
#             k4, k4_q = f(X + DT * k3, U)
#             X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
#             Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
#         F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

#         # Evaluate at a test point
#         Fk = F(x0=[0.2,0.3],p=0.4)
#         print(Fk['xf'])
#         print(Fk['qf'])

#         # Start with an empty NLP
#         w=[]
#         w0 = []
#         lbw = []
#         ubw = []
#         J = 0
#         g=[]
#         lbg = []
#         ubg = []

#         # "Lift" initial conditions
#         Xk = MX.sym('X0', 2)
#         w += [Xk]
#         lbw += [0, 1]
#         ubw += [0, 1]
#         w0 += [0, 1]

#         # Formulate the NLP
#         for k in range(N):
#             # New NLP variable for the control
#             Uk = MX.sym('U_' + str(k))
#             w   += [Uk]
#             lbw += [-1]
#             ubw += [1]
#             w0  += [0]

#             # Integrate till the end of the interval
#             Fk = F(x0=Xk, p=Uk)
#             Xk_end = Fk['xf']
#             J=J+Fk['qf']

#             # New NLP variable for state at end of interval
#             Xk = MX.sym('X_' + str(k+1), 2)
#             w   += [Xk]
#             lbw += [-0.25, -inf]
#             ubw += [  inf,  inf]
#             w0  += [0, 0]

#             # Add equality constraint
#             g   += [Xk_end-Xk]
#             lbg += [0, 0]
#             ubg += [0, 0]

#         # Create an NLP solver
#         prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
#         solver = nlpsol('solver', 'ipopt', prob);

#         # Solve the NLP
#         sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
#         w_opt = sol['x'].full().flatten()

#         # Plot the solution
#         x1_opt = w_opt[0::3]
#         x2_opt = w_opt[1::3]
#         u_opt = w_opt[2::3]

#         tgrid = [T/N*k for k in range(N+1)]
#         import matplotlib.pyplot as plt
#         plt.figure(1)
#         plt.clf()
#         plt.plot(tgrid, x1_opt, '--')
#         plt.plot(tgrid, x2_opt, '-')
#         plt.step(tgrid, vertcat(DM.nan(1), u_opt), '-.')
#         plt.xlabel('t')
#         plt.legend(['x1','x2','u'])
#         plt.grid()
#         plt.show()
        
        pass




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
    
