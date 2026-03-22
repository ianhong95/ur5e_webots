# Supplementary Linear Algebra Notes

## Overview
This document expands on some specific linear algebra concepts that are used to compute robot kinematics.

## Skew-Symmetric Matrices
A unit vector $\hat{\omega} = (\omega_{x}, \omega_{y}, \omega_{z})$ can be written as a 3x3 matrix

$$[\hat{\omega}] =
\begin{bmatrix}
0 & -\omega_{z} & \omega_{y} \\
\omega_{z} & 0 & -\omega_{x} \\
-\omega_{y} & \omega_{x} & 0
\end{bmatrix}
$$

Notice that $[\hat{\omega}]^{T} = -[\hat{\omega}]$. Skew-symmetric matrices have a cyclical property when you multiply them together.

$[\hat{\omega}]^{3} = -[\hat{\omega}]$

$[\hat{\omega}]^{4} = -[\hat{\omega}]^{2}$

$[\hat{\omega}]^{5} = -[\hat{\omega}]^{3} = [\hat{\omega}]$

Isn't that neat?

## Matrix Exponential
We've seen matrices as exponential coordinates such as $e^{[\hat{\omega}]\theta}$, but what does this really mean? How do you take the exponent of a matrix?

The matrix exponential "integrates" the matrix representation of a velocity to give a motion that results in a pose. For example, let's take the matrix exponential of a homogeneous transformation expressed as a displacement along a screw axis $S$, $e^{[S]\theta}$.

$$e^{[S]\theta}=
\begin{bmatrix}
e^{[\omega]\theta} & G(\theta)v \\
0 & 1
\end{bmatrix}
$$

where $e^{[\omega]\theta}$ is the 3x3 rotation component and $G(\theta)v$ is the 3x1 translation component. $G(\theta)$ is given by

$$
G(\theta) = I\theta + (1 - cos(\theta))[\hat{\omega}] + (\theta - sin(\theta))[\hat{\omega}]^{2}
$$

For pure translation ($\omega = 0$), $||v|| = 1$. Then the matrix exponential is simply

$$e^{[S]\theta}=
\begin{bmatrix}
I & v\theta \\
0 & 1
\end{bmatrix}
$$

### Rodrigues' Formula For Rotations
Instead of computing a massive 3x3 rotation matrix with a an overwhelming amount of $sin$ and $cos$ terms, we can use Rodrigues' formula, which is a matrix exponential.

$$
Rot(\hat{\omega}, \theta) = e^{[\hat{\omega}]\theta} = I + sin(\theta)[\hat{\omega}] + (1 - cos(\theta))[\hat{\omega}]^{2}
$$

This formula is basically the Taylor series expansion of $e^{[\hat{\omega}]\theta}$, collapsed down using the cyclical property of skew-symmetric matrices!

## Matrix Logarithm
The matrix logarithm is the inverse operation of the matrix exponential. That is, it "differentiates" a two poses into a motion about an axis $\hat{\omega}$ and a scalar $\theta$ that transforms between those two poses.

### Matrix Logarithm of Rotations
For a rotation $R = Rot(\hat{\omega},\theta)$, the skew-symmetrix matrix $[\hat{\omega}\theta] = [\hat{\omega}]\theta$ is the matrix logarithm of R. We get R and p from the transformation matrix $T_{bd}$.

The algorithm for computing the matrix logarithm is split up to handle special cases of $\theta$. This is where extra care must be taken to ensure that the math is mathematically stable by ensuring that small values of $\theta$ are handled properly.

#### Case 1: General Case
In the general case, we obtain the axis of rotation $[\hat{\omega}]$ using

$$[\hat{\omega}] =
\begin{bmatrix}
0 & -\omega_{z} & \omega_{y} \\
\omega_{z} & 0 & -\omega_{x} \\
-\omega_{y} & \omega_{x} & 0
\end{bmatrix} = 
\frac{1}{2 sin\theta}(R - R^{T})
$$

where $\theta$ is given by

$$
\theta = acos(\frac{trR - 1}{2})
$$

#### Case 2: Pure Translation
If $R = I$ then $\theta = 0$ and $\hat{\omega}$ is undefined.

#### Case 3: $\theta = \pi$
If $trR = -1$ then $\theta = \pi$. Use one of the below equations:

$$
\hat{\omega} =
\frac{1}{\sqrt{2(1+r_{33})}}
\begin{bmatrix}
r_{13} \\
r_{23} \\
1 + r_{33}
\end{bmatrix}
$$

$$
\hat{\omega} =
\frac{1}{\sqrt{2(1+r_{22})}}
\begin{bmatrix}
r_{12} \\
1 + r_{22} \\
r_{32}
\end{bmatrix}
$$

$$
\hat{\omega} =
\frac{1}{\sqrt{2(1+r_{11})}}
\begin{bmatrix}
1 + r_{11} \\
r_{21} \\
r_{31}
\end{bmatrix}
$$

### Matrix Logarithm of Twists

For twists, we want to find the matrix

$$[S]\theta =
\begin{bmatrix}
[\omega]\theta & v\theta \\
0 & 0
\end{bmatrix}
$$

which is the matrix logarithm of the transformation matrix $T = (R,p)$.

Again, we need to handle special cases.

#### Case 1: $R = I$ (Pure Translation)
If $R = I$, then:

* $\omega = 0$
* $v = p / ||p||$
* $\theta = ||p||$

#### Case 2: General Case
We use the matrix logarithm algorithm for rotations to determine $\hat{\omega}$.

Then we calculate $v$ using

$v = G^{-1}(\theta)p$

where

$G^{-1}(\theta) = \frac{1}{\theta}I - \frac{1}{2}[\omega] + (\frac{1}{\theta} - \frac{1}{2}cot\frac{\theta}{2})[\omega]^{2}$.

Notice how this formula explodes when theta is very small (division by 0). In that case, we should assume pure rotation and use case 1.

## Adjoint Representation
In normal rigid-body motion, we use a transformation matrix $T$ to transfrom between frames. In screw theory, we use the adjoint representation of $T$, a 6x6 matrix, to transform twists.

The adjoint representation of $T$ is
$$
[Ad_{T}]=
\begin{bmatrix}
R & 0 \\
[p]R & R
\end{bmatrix}
$$

where $[p]$ is the skew-symmetric representation of the translation component of $T$.

The adjoint map or adjoint transformation is

$$
V' = [Ad_{T}](V)
$$

where $V$ is a 6D twist vector given by

$$
V = 
\begin{bmatrix}
\omega \\
v
\end{bmatrix}
$$