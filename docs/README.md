# Kinematics Implementation

## Overview
The forward and inverse kinematics in this project are implemented using Screw Theory. Mozzi-Chasles' theorem states that any rigid-body motion can be described by a rotation and translation along some screw axis. This write-up will go over the key concepts required to describe the kinematics of the robot arm as I understand them, and will serve as a reference for myself.

## Identify Joint Axes and End-Effector Coordinate Frame

![UR5e Axes](/assets/UR5e_Axes.png)

![UR5e End Effector Frame](/assets/UR5e_End-Effector_Frame.png)

Based on the diagrams above, the transformation matrix to get from the origin (at the base of the robot) to the end-effector in the "zero" position is:
$$ M = 
\begin{bmatrix}
0 & -1 & 0 & L_{1} + L_{2} \\
0 & 0 & 1 & L_{4} + L_5 \\
-1 & 0 & 0 & L_{0} - L_{3} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The 3x3 section at the top-left of the $M$ matrix is the rotation matrix where each column is the x, y, and z axis of the end-effector relative to the space frame.

## Define Screw Axes
First we need to define the screw axis for each joint on the robot arm. This is very straightforward, all you need are the lengths of each link and the global coordinate axes (space frame).

1. Identify the axis of rotation of each joint as a unit vector relative to the space frame. All we care about is the direction of positive rotation; we don't care about the direction of the $x$ and $y$ axes.

    $\omega_{0} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$,
    $\omega_{1} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$, 
    $\omega_{2} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$, 
    $\omega_{3} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$, 
    $\omega_{4} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$, 
    $\omega_{5} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$

2. Identify a point on each axis. The easiest way is to just use the origin of each joint relative to the global origin. These points are denoted as a vector from the origin and they represent the lever arm.

    $q_{0} = \begin{bmatrix} 0 \\ 0 \\ L_{0} \end{bmatrix}$,
    $q_{1} = \begin{bmatrix} 0 \\ 0 \\ L_{0} \end{bmatrix}$, 
    $q_{2} = \begin{bmatrix} L_{1} \\ 0 \\ L_{0} \end{bmatrix}$, 
    $q_{3} = \begin{bmatrix} L_{1} + L_{2} \\ 0 \\ L_{0} \end{bmatrix}$, 
    $q_{4} = \begin{bmatrix} L_{1} + L_{2} + L_{3} \\ 0 \\ L_{0} \end{bmatrix}$, 
    $q_{5} = \begin{bmatrix} L_{1} + L_{2} + L_{3} + L_{4} \\ 0 \\ L_{0} - L_{3} \end{bmatrix}$

3. Compute the linear velocity of each joint. You can think of this as the speed of the origin that each joint "sees".

    $v_{i} = -\omega_{i} \times q_{i}$

4. Finally, we build the screw axes using the rotation axes and linear velocities.

    $S_{i} = \begin{bmatrix} \omega_{i} \\ v_{i} \end{bmatrix}$


## Body Screws
The screw axes above are in the space frame. When we do end-effector manipulations, it is helpful to work in the body frame because it is easier to think "I want to move the end-effector 10mm left" rather than determine your current position then determine the location in space that you want to be. Thus, we will compute the body screws using the inverse of the Adjoint transform.

The Adjoint map matrix is:

$ [Ad_{M}] = 
\begin{bmatrix}
R & 0 \\
[p]R & R
\end{bmatrix}
$

The body screws are computed using the Adjoint transform:

$B_{i} = \begin{bmatrix} Ad_{M^{-1}} \end{bmatrix}S_{i}$

## Forward Kinematics (Body Frame)
The goal is to obtain a transformation matrix that describes the transformation from the robot's base frame to the end-effector frame. In this case, the transformation will be expressed in the body frame.

1. Convert each 6D screw axis $B_{i}$ into an se(3) matrix $[B_{i}]$.
2. For each screw matrix, compute the matrix exponential $e^{[S_{i}]\theta_{i}}$.
3. Start from the base and accummulate the transformations (post-multiply in sequence). We post-multiply because we want to rotate about the latest joint in the sequence at each step (body transformation rather than space transformation).
    
    $T_{bs} = Me^{[B_{0}]\theta_{0}}e^{[B_{1}]\theta_{1}}...e^{[B_{n}]\theta_{n}}$

The transformation matrix $T_{sb}$ describes the pose of the end-effector in the body frame.

## Jacobian Matrix (Body Frame)
The Jacobian is a matrix that relates joint velocities to the end-effector velocity. The relationship is as follows:

$V = J(\theta)\dot{\theta}$

Where:

* $V$ = end-effector velocity as a 6D twist vector
* $J(\theta)$ = Jacobian matrix
* $\theta$ = joint angles as an $n \times 1$ matrix (where $n$ is the number of joints)
* $\dot{\theta}$ = joint velocities as an $n \times 1$ matrix (where $n$ is the number of joints)

This formula shows that we can use the inverse of the Jacobian to solve for a set of joint velocities $\dot{\theta}$ that will result in a desired end-effector velocity. However, this is a problem for later. First we need to build the Jacobian matrix.

1. We start by computing the forward kinematics in the body frame to get the current end-effector's transformation $T_{bs}$.

    $T_{bs} = Me^{[B_{0}]\theta_{0}}e^{[B_{1}]\theta_{1}}...e^{[B_{n}]\theta_{n}}$

2. We will need the body screw vectors that were determined in a previous step.

    $B_{i} = \begin{bmatrix} Ad_{M^{-1}} \end{bmatrix}S_{i}$

3. Now this part is tricky, so pay attention to the indices. The goal here is to compute each column in the matrix separately. We need to loop backwards from joint $n$ down to 1 and build the Jacobian matrix one column at a time, where each column is the screw axis of one joint expressed in the end-effector's frame. 

    What this means physically is that for each joint (starting from the end-effector), we are taking that joint's screw vector and applying a transform (the cumulative adjoint maps) to take it back to the end-effector's frame (this hurts my brain too).

    The algorithm is

    $J_{b_{i}} = Ad_{T^{-1}_{i+1}}B_{i}$

    Refer to the [Linear Algebra](/docs/Linear_Algebra.md) document for the adjoint matrix computation.

    Note that it really starts from the ${n-1}^{th}$ joint because at the end-effector there is no transformation necessary, so $J_{b_{n}} = B_{n}$.

4. Now we just insert each column $J_{b_{i}}$ into the Jacobian matrix $J_{b}(\theta)$.

    $J_{b}(\theta) = [J_{b_{0}}, J_{b_{1}}, ..., J_{b_{n}}]$

## Inverse Kinematics
In general when we talk about inverse kinematics (IK), we talk about what we need to set each joint angle to in order to move the end-effector to a desired position. Since we're using screws instead of independent rotations and translations, we can't really solve this analytically to jump straight to the target joint angles. We'll have to solve this numerically, and this will initially be done using the Newton-Raphson method.

Step 0 is feeding in the transformation matrix of the desired end-effector pose, a 4x4 homogeneous matrix in the space frame $T_{sd}$.

1. Get the transformation matrix of the current end-effector pose in the end-effector frame.

    $T_{bs} = Me^{[B_{0}]\theta_{0}}e^{[B_{1}]\theta_{1}}...e^{[B_{n}]\theta_{n}}$

2. Build the body Jacobian matrix as described in the previous section.

    $J_{b}(\theta) = [J_{b_{0}}, J_{b_{1}}, ..., J_{b_{n}}]$

3. Now we need to compute the error as a twist between the target end-effector frame $T_{sd}$, and the current end-effector frame $T_{bs}$. This is where it gets interesting because normally when we think of the linear distance between two points, we just subtract their coordinates. Since we are combining linear and rotational displacement as a twist, the "distance" (or error) is computed using the matrix logarithm, given $T_{sd}$ and an initial guess $\theta^{0}$ (which we can reasonably use the current joint angles).

    $[V_{b}] = log(T_{bs}(\theta^{i})T_{sd})$

    where $[V_{b}]$ is the 6D twist error vector:
    
    $$[V_{b}] = 
    \begin{bmatrix}
    \omega_{bx} \\
    \omega_{by} \\
    \omega_{bz} \\
    v_{bx} \\
    v_{by} \\
    v_{bz}
    \end{bmatrix}
    $$

    The algorithm for computing the matrix logarithm can be found in the [Linear Algebra](/docs/Linear_Algebra.md) document.

    The norm of first 3 components of $[V_{b}]$ corresponds to the rotational error ($||\omega_{b}||$), and the norm of last 3 components ($||v_{b}||$) corresponds to the translational error.

4. Calculate the joint angle increments by solving

    $\Delta\theta = J^{+}[V_{b}]$

    where $J^{+}$ is the Moore-Penrose pseudo-inverse of the body Jacobian. We can't just take the inverse because the Jacobian is not always square.

5. Calculate the new joint angles by adding the angle increments to the current joint angles.

    $\theta_{new} = \theta_{current} + \Delta\theta$

6. Iterate steps 1-5 until the twist error is below your threshold.