## Project: Kinematics Pick & Place

[//]: # (Image References)

[joints]: ./misc_images/joint_screenshot.png
[dh_params]: ./misc_images/dh_params.png
[dh_diagram]: ./misc_images/dh_diagram.jpg
[joints_2_and_3]: misc_images/joints2and3.png
[pick_and_place]: misc_images/pick_and_place.png

### Forward Kinematics
#### Theory
The process of calculating a robot's end effector pose from its joint angles is known as *Forward Kinematics*. Each joint
produces a transformation, and this chain of transformation matrices gives the final end effector pose. 

![Joint Screenshot][joints]

#### DH Table
In order to calculate forward kinematics, we need a mathematical representation of the robot's joints. Each joint could 
be represented as a set of 6 values, three for position and three for rotation. 

However, in this project, each joint was
represented more concisely using 4 *Denavit-Hartenburg* parameters. These four special values can represent all of the 
possible transformations in a single revolute joint. They are as follows:

- alpha : The twist angle between one joint axis and the next
- a : Joint length
- d : Joint offset
- theta : Joint angle, this is the only variable for revolute joints

![dh example][dh_params]

In order to calculate DH parameters, each joint must be assigned a coordinate system. The Z axis must be the joint's axis
of rotation, but otherwise each axis is chosen in order to make as many parameters equal to zero as possible.

Each link length and joint offest must be calculated manually by looking through the URDF file and converting those 
measurements into the frame of the previous joint. I've attached a diagram that I used to calculate my DH table, the
axes in the top left are the frame used by the URDF file.

![DH Parameter Diagram][dh_diagram]

Note that Joints 4, 5, and 6 share a common origin in my diagram even though that is not the case in the URDF. The joint
origins can be moved arbitrarily along their Z axes, and this was done to create a spherical wrist, which I'll explain 
in detail later. The parameter d_g is a composite of Joint 6's offset from it assigned origin in my diagram and the 
distance from the gripper to joint 6 in the URDF file.

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

The transformation matrix for each joint is created by substituting values from the DH table into the above matrix. Each
transformation matrix is given below.

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
The total transformation matrix T0_G can be calculated directly from the gripper's position and orientation.
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
As its name implies, Inverse Kinematics is the opposite of Forward Kinematics. Instead of calculating the pose of an end
effector, we take a desired pose and calculate the require joint angles to produce it. 

#### Wrist Center
The key to inverse kinematics is using extra degrees of freedom to decouple the end effector's position from its 
orientation. By taking the end effector's orientation and working backwards, we can easily calculate the position of 
joints 4, 5, and 6. These joints share a common origin, known as the Wrist Center. 

Once the wrist center is known, joints 4, 5, and 6 can determine the orientation of the gripper, and joints 1, 2, and 3 
determine the position of the wrist.

The wrist center is calculated in the following steps:
##### 1. Calculate total transformation matrix
Use the desired end effector position and angles to calculate the total transformation matrix, T0_G.

##### 2. Extract rotation component from total transformation
The top left 3x3 of the total transformation T0_G is the corresponding rotation matrix. Extract it and call it R0_6.

##### 3. Move backwards from end effector to get wrist center
R0_6 defines the angle between the wrist center and the end effector. Since the distance between them is also known, we
can now calculate the wrist center directly.

EE = WC + R0_6 * [0, 0, d]

WC = EE - R0_6 * [0, 0, d] where d is the Z distance from joint 6 to the gripper according the DH diagram.

#### Joints 1-3
Now that the wrist center is known, we can decouple the joints into two sets. The first set (1, 2, and 3) determines the
position of the wrist, and the other (4, 5, and 6) determines the orientation of the gripper.

Joints 1, 2, and 3 can be calculated using trigonometry alone. First, project the robot arm onto the XY plane. The angle
formed by the wrist center and origin is theta1.

theta1 = atan2(WC_y, WC_x)

Next, draw the robot in a vertical plane as shown below. Note that the origin is Joint 2 (a1, d1).

![alt text][joints_2_and_3]

Link2, link3, and the distance from 2 to the WC (D2_wc) form a triangle. Each length can be calculated now using values 
from the DH table.

- link2=a2
- link3=sqrt(d4^2 + a3^2)
- D2_wc=sqrt((WC_x - a1)^2 + (WC_y - d1)^2)

The interior angles of the triangle can now be calculated using the law of cosines and then used to get theta2 and theta3.

theta2 = pi/2 - interior_angle_2 - atan2(WC_y - d1, WC_x - a1)

Theta3 requires a small correction because link three contains a perpendicular offset of a3.

theta3 = pi/2 - interior_angle_3 + atan2(a3, d4)
 
#### Joints 4-6
Joints 4, 5, and 6 determine the orientation of the gripper relative to the wrist center. This orientation change is 
also known as R3_6, and can be calculated as follows.

- Calculate R0_3 using theta1, theta2, and theta3
- R3_6 = transpose(R0_3) * R0_6

This matrix R3_6 is the sum of all rotations of joints 4, 5, and 6. Each angle can be found by taking a generating a 
symbolic version of the matrix and solving for each angle.

R3_6 = top left 3x3 of T3_4 * T4_5 * T5_6 

col 1 | col 2 | col 3 | col 4
---|---|---|---
-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6) | -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5) | -sin(q5)*cos(q4) 
sin(q5)*cos(q6) | -sin(q5)*sin(q6) | cos(q5) 
-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4) | sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6) | sin(q4)*sin(q5) 

Using trig identities, we solve for each angle as follows.

- theta4 = atan2(r33, -r13)
- theta5 = atan2(sqrt(r22 ** 2 + r21 ** 2), r23)
- theta6 = atan2(-r22, r21)

The real theta values are then calculated from the previously calculated R3_6, and then inverse kinematics is complete!

### Results
I chose to write all of my kinematics code in a separate file a run it outside of ROS first, and I believe that my extra
unit tests allowed me to finish this much more quickly than if I had run the simulation each time.

The robot successfully moves to each position, and the only issue I see is some extra rotation in joints 4 and 6. 
Sometimes joints 4 and 6 have large angles that cancel each other out while theta5 is small. This doesn't affect the 
end position or orientation, but it adds extra motion that takes time.

I noticed that the simulation often moves too quickly to grip the cylinder, but that's not a fault of the kinematics in 
this case. I fixed it by adding this to line 327 in the /src/trajectory_sampler.cpp file:

`ros::Duration(2.0).sleep();`

Then everything worked great!

![Pick and Place][pick_and_place]
