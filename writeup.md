## Project: Kinematics Pick & Place

[//]: # (Image References)

-[ ] Screenshot with showing joint angles
-[ ] Screenshot of completed pick and place
-[x] DH Parameter diagram


[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[dh_params]: ./misc_images/dh_params.png
[dh_diagram]: ./misc_images/dh_diagram.jpg

### Forward Kinematics
#### Theory
The process of calculating a robot's end effector pose from its joint angles is known as *Forward Kinematics*. Each joint
produces a transformation, and this chain of transformation matrices gives the final end effector pose. 
#### DH Table
In order to calculate forward kinematics, we need a mathematical representation of the robot's joints. Each joint could 
be represented as a set of 6 values, three for position and three for rotation. 

However, in this project, each joint was
represented more concisely using 4 *Denavit-Hartenburg* parameters. These four special values can represent all of the 
possible transformations in a single revolute joint. They are as follows:

- alpha : The twist angle between one joint axis and the next
- a : Joint length
- d : Joint offset
- theta : Joint angle, this is the only part that changes for revolute joints

![alt text][dh_params]

In order to calculate DH parameters, each joint must be assigned a coordinate system. The Z axis must be the joint's axis
of rotation, but otherwise each axis is chosen in order to make as many parameters equal to zero as possible.

Each link length and joint offest must be calculated manually by looking through the URDF file and converting those 
measurements into the frame of the previous joint. I've attached a diagram that I used to calculate my DH table, the
axes in the top left are the frame used by the URDF file.

![DH Parameter Diagram][dh_diagram]

Note that Joints 4, 5, and 6 share a common origin in my diagram even though that is not the case in the URDF. The joint
origins can be moved arbitrarily along their Z axes, and this was done to create a spherical wrist. The parameter d_g is
a composite of Joint 6's offset from it assigned origin in my diagram and the distance from the gripper to joint 6 in 
the URDF file.

Here's the complete DH table filled in with values form the URDF file:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### Matrices
Like I mentioned before, each Joint's reference transformation is described as a matrix. All DH parameterized joints 
have a transformation matrix written as:

col 1| col 2| col 3 | col 4
--- | --- | --- | ---
cos(theta) | -sin(theta) | 0 | a
sin(theta)*cos(alpha) | cos(theta)*cos(alpha) | -sin(alpha) | -d*sin(alpha)
sin(theta)*sin(alpha) | cos(theta)*sin(alpha) | cos(alpha) | d *cos(alpha) 
0 | 0 | 0 | 1

The transformation matrices for each joint are as follows:

##### T0_1
col 1 | col 2 | col 3 | col 4
---|---|---|---
cos(q1) | -sin(q1) | 0 | 0
sin(q1) | cos(q1) | 0 | 0
0 | 0 | 1 | 0.75
0 | 0 | 0 | 1

##### T1_2
col 1 | col 2 | col 3 | col 4
---|---|---|---
sin(q2) | cos(q2) | 0 | 0.35
0 | 0 | 1 | 0
cos(q2) | -sin(q2) | 0 | 0
0 | 0 | 0 | 1

##### T2_3
col 1 | col 2 | col 3 | col 4
---|---|---|---
cos(q3) | -sin(q3) | 0 | 1.25
sin(q3) | cos(q3) | 0 | 0
0 | 0 | 1 | 0
0 | 0 | 0 | 1

##### T3_4
col 1 | col 2 | col 3 | col 4
---|---|---|---
cos(q4) | -sin(q4) | 0 | -0.054
0 | 0 | 1 | 1.5
-sin(q4) | -cos(q4) | 0 | 0
0 | 0 | 0 | 1

##### T4_5
col 1 | col 2 | col 3 | col 4
---|---|---|---
cos(q5) | -sin(q5) | 0 | 0
0 | 0 | -1 | 0
sin(q5) | cos(q5) | 0 | 0
0 | 0 | 0 | 1

##### T5_6
col 1 | col 2 | col 3 | col 4
---|---|---|---
cos(q6) | -sin(q6) | 0 | 0
0 | 0 | 1 | 0
-sin(q6) | -cos(q6) | 0 | 0
0 | 0 | 0 | 1

##### T6_G
col 1 | col 2 | col 3 | col 4
---|---|---|---
1 | 0 | 0 | 0
0 | 1 | 0 | 0
0 | 0 | 1 | 0.303
0 | 0 | 0 | 1

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G * T_Corr

#### Corrections
A correction matrix must be applied to T0_G in order to convert to the URDF frame for the gripper. In this case, the 
correction matrix is a rotation of pi radians about Z and -pi/2 radians about Y.

##### T_Corr
col 1 | col 2 | col 3 | col 4
---|---|---|---
0 | 0 | 1 | 0
0 | -1 | 0 | 0
1 | 0 | 0 | 0
0 | 0 | 0 | 1

#### Total Transformation
The total transformation matrix from base to gripper can be calculated from the gripper's position and orientation.
In the 4x4 transformation matrix, the top left 3X3 matrix is the rotation (including correction matrix), the top right 
3X1 is the translation, and the bottom row is [0, 0, 0, 1].

The total transformation matrix is given below, where (Px, Py, Pz) is the gripper position and (r, p, y) are its Euler
angles.

col 1 | col 2 | col 3 | col 4
---|---|---|---
-sin(p) | sin(r)*cos(p) | cos(p)*cos(r) | Px
-sin(y)*cos(p) | -sin(p)*sin(r)*sin(y) - cos(r)*cos(y) | -sin(p)*sin(y)*cos(r) + sin(r)*cos(y) | Py
cos(p)*cos(y) | sin(p)*sin(r)*cos(y) - sin(y)*cos(r) | sin(p)*cos(r)*cos(y) + sin(r)*sin(y) | Pz
0 | 0 | 0 | 1

### Inverse Kinematics
#### Wrist Center
#### Joints 1-3
#### Joint 4-6

#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


