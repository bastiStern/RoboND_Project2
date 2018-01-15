[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Pick and Place Project
---
The main idea was to make the calculation of the joint angles theta1 to 6 more ressource effective. In order to achieve that i generated a class object named `InverseKinematicSolver`
in the `IK_solver.py` file. This class is instantiated right before the for-loop in the`handle_calculate_IK()` function. So each time the for loop iterates over it's body it uses the method 
`Solver.calculate_angles(px, py, pz, roll, pitch, yaw)` to get the angles. By doing this we don't need to calculate the transformation matrices every iteration.
```python
if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
    	# generate ik solver object
    	Solver = InverseKinematicSolver()

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # do stuff
            # do stuff
            # do stuff
```
### Kinematic Analysis

I derived the following Denavit-Hartenberg Parameters

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1  | 0      | 0      | 0.75  | q1
1->2  | - pi/2 | 0.35   | 0     | q2-pi/2
2->3  | 0      | 1.25   | 0     | q3
3->4  | - pi/2 | -0.054 | 1.50  | q4
4->5  | pi/2   | 0      | 0     | q5
5->6  | - pi/2 | 0      | 0     | q6
6->EE | 0      | 0      | 0.303 | q7

In the constructor of the  `InverseKinematicSolver` class the DH-Parameters are stored as dictionary like this
```python
# set up symbols
self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8') #joints
self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8') #link offsets
self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7') #link length
self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7') #twist angles 
self.r, self.p, self.y = symbols('r p y')

# DH Parameters dictionary
self.dh_param = {self.alpha0:      0,      self.a0:     0,      self.d1:     0.75,       self.q1:     self.q1,      
                 self.alpha1:     -pi/2.,  self.a1:     0.35,   self.d2:     0,          self.q2:     self.q2-pi/2,            
                 self.alpha2:      0,      self.a2:     1.25,   self.d3:     0,          self.q3:     self.q3,      
                 self.alpha3:      -pi/2., self.a3:     -0.054, self.d4:     1.50,       self.q4:     self.q4,      
                 self.alpha4:      pi/2,   self.a4:     0,      self.d5:     0,          self.q5:     self.q5,      
                 self.alpha5:      -pi/2., self.a5:     0,      self.d6:     0,          self.q6:     self.q6,      
                 self.alpha6:      0,      self.a6:     0,      self.d7:     0.303,      self.q7:     0}
```



### Inverse Position Kinematics 
To get the wristcenter of the robot arm we need to calculate it like this.
```python 
self.WC = self.TCP - (self.dh_param.get(self.d7) * self.ROT_TCP[:,2])
```
Once we have the wristcenter x,y,z we can continue to calculate theta1, theta2, theta3. Here a geometrical approac was used.
```python 
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
```
### Inverse Orientation Kinematics
Now that we have the first three angles we can continue to solve for theta4, theta5, theta6. To do so we need a partial rotation matrix called `self.R3_6`
```python 
R0_3 = self.T0_1[0:3,0:3] * self.T1_2[0:3,0:3] * self.T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={self.q1: theta1, self.q2: theta2, self.q3: theta3})
R3_6 = R0_3.inv('LU') * self.ROT_TCP
```
Now the angles can be extracted from the rotation matrix of the wrist
```python 
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation
Unfortunately i was not able to run the `IK_server.py` because either Gazeebo or Rviz chrashed on startup but i implemented it in the file like this. 
```python 
# calcuclate angles theta 1-6 with solver method and append
joint_trajectory_point.positions = [Solver.calculate_angles(px, py, pz, roll, pitch, yaw)]
joint_trajectory_list.append(joint_trajectory_point)
```
The implementation in the `IK_debug.py` workes as expected 
```python 
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

# roll, pitch yaw define the orentation of the EE/TCP
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w])

# generate class object
Solver = InverseKinematicSolver()

# solve for current position 
theta1, theta2, theta3, theta4, theta5 , theta6 = Solver.calculate_angles(px, py, pz, roll, pitch, yaw)
```



