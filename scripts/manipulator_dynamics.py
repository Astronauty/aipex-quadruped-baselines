import pinocchio
import os
import numpy as np


class URDFManipulatorDynamics:
    def __init__(self, urdf_path):
        # urdf_path = "/src/convex_mpc/urdf/go2_description.urdf"
        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        
        self.data = self.model.createData()
        
        # self.nq = np.zeros(self.model.nq)
        # self.nv = np.zeros(self.model.nv)
        
        None
    
    @property
    def nq(self):
        return self.model.nq
    
    @property
    def nv(self):
        return self.model.nv
    
    
    def get_manipulator_dynamics(self, q, dq):
        """_summary_

        Args:
            q (_type_): _description_
            qd (_type_): _description_

        Returns:
            M (ndarray): _description_
            
        """
        assert len(q) == self.model.nq, f"q dimension: {len(q)} does not match urdf q dimension: {self.model.nq}"
        assert len(dq) == self.model.nv, f"dq dimension: {len(dq)} does not match urdf dq dimension: {self.model.nv}"
        
        M = pinocchio.crba(self.model, self.data, q)               # Mass matrix
        C = pinocchio.computeCoriolisMatrix(self.model, self.data, q, dq)  # Coriolis matrix
        g = pinocchio.computeGeneralizedGravity(self.model, self.data, q)  # Gravity vector

        print("Go2 Manipulator Dynamics")
        print("------------------------")
        print(f"nq: {self.nq}")
        print(f"nv: {self.nv}")
        print()
        print(f"M {M.shape}:\n", M)
        print()
        print(f"C {C.shape}:\n", C)
        print()
        print("g:\n ", g)
        
        return M, C, g

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


# data = model

if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(current_dir, "../src/convex_mpc/urdf/go2_description.urdf")
    go2= URDFManipulatorDynamics(urdf_path)
    
    # Create dummy joint config
    q = np.zeros(go2.nq)
    dq = np.zeros(go2.nv)
    u = np.zeros(go2.nq)
    
    pinocchio.JointModelFreeFlyer()
    
    
    M, C, g = go2.get_manipulator_dynamics(q, dq)

    
