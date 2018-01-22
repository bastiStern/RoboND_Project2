from sympy import *
from time import time
from mpmath import radians
import tf

class InverseKinematicSolver(object):
    """
    class Object to solve the inverse kinematics of the kuka kr210 6dof robot arm
    """
    def __init__(self):
        # set up symbols
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8') #joints
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8') #link offsets
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7') #link length
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7') #twist angles 
        self.r, self.p, self.y = symbols('r p y')

        # dH Parameters dictionary
        self.dh_param = {self.alpha0:      0,      self.a0:     0,      self.d1:     0.75,       self.q1:     self.q1,      
                         self.alpha1:     -pi/2.,  self.a1:     0.35,   self.d2:     0,          self.q2:     self.q2-pi/2,            
                         self.alpha2:      0,      self.a2:     1.25,   self.d3:     0,          self.q3:     self.q3,      
                         self.alpha3:      -pi/2., self.a3:     -0.054, self.d4:     1.50,       self.q4:     self.q4,      
                         self.alpha4:      pi/2,   self.a4:     0,      self.d5:     0,          self.q5:     self.q5,      
                         self.alpha5:      -pi/2., self.a5:     0,      self.d6:     0,          self.q6:     self.q6,      
                         self.alpha6:      0,      self.a6:     0,      self.d7:     0.303,      self.q7:     0}

                         
        # create individual transformation matrices and fill in the DH-param dictionary
        self.T0_1 = self.transformation_matrix(self.alpha0, self.a0, self.d1, self.q1).subs(self.dh_param)
        self.T1_2 = self.transformation_matrix(self.alpha1, self.a1, self.d2, self.q2).subs(self.dh_param)
        self.T2_3 = self.transformation_matrix(self.alpha2, self.a2, self.d3, self.q3).subs(self.dh_param)
        self.T3_4 = self.transformation_matrix(self.alpha3, self.a3, self.d4, self.q4).subs(self.dh_param)
        self.T4_5 = self.transformation_matrix(self.alpha4, self.a4, self.d5, self.q5).subs(self.dh_param)
        self.T5_6 = self.transformation_matrix(self.alpha5, self.a5, self.d6, self.q6).subs(self.dh_param)
        self.T6_TCP = self.transformation_matrix(self.alpha6, self.a6, self.d7, self.q7).subs(self.dh_param)
        self.TBASE_TCP = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_TCP                 

        self.ROT_TCP = self.rot_z(self.y) * self.rot_y(self.p) * self.rot_x(self.r)
        self.ROT_Error = self.rot_z(self.y).subs(self.y, radians(180)) * self.rot_y(self.p).subs(self.p, radians(-90))
        self.ROT_TCP = self.ROT_TCP * self.ROT_Error
        self.WC = None
        self.TCP = None

    def calculate_angles(self, px, py, pz, roll, pitch, yaw):
        """
        calculate a set of 6 angles with this function
        """

        # fill in the matrix to get the TCP orientation 
        self.ROT_TCP = self.ROT_TCP.subs({'r': roll, 'p': pitch, 'y': yaw})

        self.TCP = Matrix([[px],
                           [py],
                           [pz]])

        # calculate the wrist center
        self.WC = self.TCP - (self.dh_param.get(self.d7) * self.ROT_TCP[:,2])

        # calculate the joint angles
        # use a geometrical approch with a traingle abc
        x = sqrt(self.WC[0]*self.WC[0] + self.WC[1]*self.WC[1]) - self.dh_param.get(self.a1)
        y = self.WC[2] - self.dh_param.get(self.d1)

        a = round(sqrt(self.dh_param.get(self.d4)**2 + self.dh_param.get(self.a3)**2), 4)
        b = sqrt(x**2 + y**2)
        c = self.dh_param.get(self.a2)
       

        # get the inner angels of triangle abc with the law of cosine
        error = abs(self.dh_param.get(self.a3)) / self.dh_param.get(self.d4)
        alpha = acos((b**2 + c**2 - a**2) / (2*b*c))
        beta = acos((a**2 + c**2 - b**2) / (2*a*c))
        gamma = acos((a**2 + b**2 - c**2) / (2*a*b))
        
        # get the thetas with help of alpha, beta, gamma
        theta1 = atan2(self.WC[1], self.WC[0])
        theta2 = pi/2 - alpha - atan2(y, x)
        theta3 = pi/2 - beta - error

        # get partial rotation matrix
        R0_3 = self.T0_1[0:3,0:3] * self.T1_2[0:3,0:3] * self.T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={self.q1: theta1, self.q2: theta2, self.q3: theta3})
        R3_6 = R0_3.transpose() * self.ROT_TCP

        # get the euler angles for the orientation of the wrist from the R3_6 rotation matrix
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
        if sin(theta5) < 0:
        	theta4 = atan2(-R3_6[2,2], R3_6[0,2])
	        theta6 = atan2(R3_6[1,1], -R3_6[1,0])
        else:	
	        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	        theta6 = atan2(-R3_6[1,1], R3_6[1,0])


        return theta1, theta2, theta3, theta4, theta5, theta6



    @staticmethod
    def transformation_matrix(alpha, a, d, q):
        return Matrix([[cos(q),                       -sin(q),             0,              a],
                       [sin(q)*cos(alpha),  cos(q)*cos(alpha),   -sin(alpha),  -sin(alpha)*d],
                       [sin(q)*sin(alpha),  cos(q)*sin(alpha),    cos(alpha),   cos(alpha)*d],
                       [                0,                  0,             0,              1]])
    
    @staticmethod    
    def rot_x(q):
        return Matrix([ [1,        0,       0],
                        [0,   cos(q), -sin(q)],
                        [0,   sin(q),  cos(q)]])

    @staticmethod
    def rot_y(q):
        return Matrix([ [cos(q),   0,   sin(q)],
                        [0,        1,        0],
                        [-sin(q),  0,   cos(q)]])  
    @staticmethod    
    def rot_z(q):
        return Matrix([ [cos(q),  -sin(q), 0],
                        [sin(q),   cos(q), 0],
                        [0,             0, 1]])