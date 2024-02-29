from robot3D_basic import getLocalFrameMatrix, RotationMatrix
import numbers as np

def forward_kinematics(Phi, L1, L2, L3, L4):
   # Diameter of the spheres
   dia = 0.8
   phi1, phi2, phi3, phi4 = Phi
   
   R_01 = RotationMatrix(phi1, axis_name='y')
   t_01 = np.array([[3], [2], [0]]) 
   T_01 = getLocalFrameMatrix(R_01, t_01)
   
   
   
   R_12 = RotationMatrix(phi2, axis_name='z')
   t_12 = np.array([[L1 + dia], [0.0], [0.0]]) 
   T_12 = getLocalFrameMatrix(R_12, t_12)
   
   T_02 = T_01 @ T_12
   
   
   
   R_23 = RotationMatrix(phi3, axis_name='y')
   t_23 = np.array([[L2 + dia], [0.0], [0.0]]) 
   T_23 = getLocalFrameMatrix(R_23, t_23)
   
   T_03 = T_02 @ T_23
   
   
   
   R_34 = RotationMatrix(phi3, axis_name='z')
   t_34 = np.array([[L3 + dia], [0.0], [0.0]]) 
   T_34 = getLocalFrameMatrix(R_34, t_34)
   
   T_04 = T_03 @ T_34
   
   e = T_04[:, 4][:3]
   
   return T_01, T_02, T_03, T_04, e
   
   