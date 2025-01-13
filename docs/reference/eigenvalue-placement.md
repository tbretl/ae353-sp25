---
title: Eigenvalue placement
parent: Reference
nav_order: 5
---

# Eigenvalue placement
{: .no_toc }

- TOC
{:toc }

## Overview

Apply the input

$$u=-Kx$$

to the open-loop system

$$\dot{x} = Ax+Bu $$

and you get the closed-loop system

$$\dot{x} = (A-BK)x.$$

Suppose we want to choose $K$ to put the eigenvalues of the closed-loop system, i.e., the eigenvalues of the matrix $A-BK$, at given locations. We will derive a formula called **Ackermann's method** that allows us to do this when possible, and will show how to decide when doing so is impossible.

{: .warning}
> This entire discussion will be based on the assumption that there is exactly one input (i.e., that $u$ is a vector of length $1$, or that $n_u = 1$). Ackermann's method, in particular, *cannot be used* for eigenvalue placement when there is more than one input.
> 
> However, other similar methods — for example, the method of Tits and Yang [("Globally convergent algorithms for robust pole assignment by state feedback," *IEEE Transactions on Automatic Control*, 41:1432-1452, 1996)](https://doi.org/10.1109/9.539425), as implemented by [scipy.signal.place_poles](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html#scipy-signal-place-poles) in python — <em>can</em> be used when there are multiple inputs. Our result about when eigenvalue placement is possible — i.e., about when a system is "controllable" — also generalizes to systems with multiple inputs, although it becomes harder to prove.

## Eigenvalues are invariant to coordinate transformation

Consider the system

$$\dot{x} = Ax+Bu.$$

Suppose we define a new state variable $z$ so that

$$x = Vz$$

for some invertible matrix $V$, and so

$$\dot{x} = V\dot{z}$$

by differentiation. We have called this process "coordinate transformation" — it is exactly the same process we used for diagonalization when establishing our result about asymptotic stability.
Plug these two things into our original state-space model and we get


$$V\dot{z} = AVz+Bu.$$

Solve for $\dot{z}$ and we get the equivalent state-space model

$$\dot{z} = V^{-1}AV z + V^{-1}B u.$$

Finding a solution $z(t)$ to this transformed system allows us to recover a solution

$$x(t) = Vz(t)$$

to the original system. We would like to know if these two solutions "behave" the same way. In particular, we would like to know if the eigenvalues of $A$ are the same as the eigenvalues of $V^{-1}AV$.

First, let's look at the eigenvalues of $A$. We know that they are the roots of

$$\det\left(sI-A\right).$$

Second, let's look at the eigenvalues of $V^{-1}AV$. We know that they are the roots of

$$\det\left( sI-V^{-1}AV \right)$$

We can play a trick.
Notice that

$$V^{-1}(sI)V = s V^{-1}V = sI$$

and so

$$
\begin{align*}
\det\left( sI-V^{-1}AV \right)
&= \det\left( V^{-1}(sI)V-V^{-1}AV \right) && \text{because of our trick} \\
&= \det\left( V^{-1}(sI - A) V \right).
\end{align*}
$$

It is a fact that

$$\det(MN)=\det(M)\det(N)$$

for any square matrices $M$ and $N$.
Applying this fact, we find

$$\det\left( V^{-1}(sI - A) V \right) = \det(V^{-1})\det(sI-A)\det(V).$$

It is another fact that

$$\det(M^{-1}) = (\det(M))^{-1} = \dfrac{1}{\det(M)}$$

Applying this other fact, we find

$$
\det(V^{-1})\det(sI-A)\det(V)
= \dfrac{\det(sI-A)\det(V)}{\det(V)}
= \det(sI-A)
$$

In summary, we have established that

$$\det(sI-A) = \det\left( sI-V^{-1}AV \right)$$

and so the eigenvalues of $A$ and $V^{-1}AV$ are the same. The consequence is, if you design state feedback for the transformed system, you'll recover the behavior you want for the original system. In particular, suppose you apply the input

$$u = -Lz$$

to the transformed system and choose $L$ to place the eigenvalues of $V^{-1}AV$ in given locations.
Applying the input

$$u = -LV^{-1}x$$

to the original system, i.e., choosing

$$K = LV^{-1},$$

will result in placing the eigenvalues of $A$ at these same locations. The reason this is important is that it is often easier to choose $L$ than to choose $K$. The process of [diagonalization](stability#what-is-the-solution-to-a-linear-system-that-is-diagonalizable) was important for a similar reason.

## Controllable canonical form

In the previous section, we showed that eigenvalues are invariant to coordinate transformation. The next question is what coordinates are useful for control design. The answer to that question turns out to be something called **controllable canonical form.**

A system with $n_x$ states and $1$ input is in controllable canonical form if it looks like

$$\dot{z} = A_\text{ccf}z + B_\text{ccf} u$$

where

$$
A_\text{ccf} =
\begin{bmatrix}
-a_{1} & -a_{2} & \dotsm & -a_{n_x-1} & -a_{n_x} \\
1 & 0 & \dotsm & 0 & 0 \\
0 & 1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & 1 & 0
\end{bmatrix}
\qquad\qquad
B_\text{ccf} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ \vdots \\ 0 \end{bmatrix}.
$$

Notice that

* $A_\text{ccf}$ is a matrix of size $n_x \times n_x$,
* $B_\text{ccf}$ is a matrix of size $n_x \times 1$,
* $z$ is a vector of length $n_x$, and
* $u$ is a vector of length $1$.

It is a fact that the characteristic equation of this system is given by

$$\det(sI-A_\text{ccf}) = s^{n_x} + a_{1}s^{n_x-1} + \dotsm + a_{n_x-1}s + a_{n_x}.$$

It is easy to see that this formula is true for $n_x=2$ and $n_x=3$. In particular:


* If $n_x=2$, then:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det\begin{bmatrix} s+a_{1} & a_{2} \\ -1 & s \end{bmatrix} \\
&= (s+a_{1})s+a_{2} = s^{2}+a_{1}s+a_{2}.
\end{align*}
$$

* If $n_x=3$, then:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det\begin{bmatrix} s+a_{1} & a_{2} & a_{3}\\ -1 & s & 0 \\ 0 & -1 & s \end{bmatrix} \\
&= (s+a_{1})s^{2}+a_{3}-(-a_{2}s)= s^{3}+a_{1}s^{2}+a_{2}s+a_{3}.
\end{align*}
$$

There are a variety of ways to prove that this same formula is true in general. Applying the general formula to compute the matrix determinant, for example, we would find:

$$
\begin{align*}
\det(sI-A_\text{ccf})
&= \det
\begin{bmatrix}
s+a_{1} & a_{2} & \dotsm & a_{n_x-1} & a_{n_x} \\
-1 & s & \dotsm & 0 & 0 \\
0 & -1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & -1 & s
\end{bmatrix} \\
&= (s+a_{1}) \det(T_{1})-a_{2}\det(T_{2})+a_{3}\det(T_{3})-\dotsm
\end{align*}
$$

where each matrix $T_{i}$ is upper-triangular with $-1$ in $i-1$ diagonal entries and $s$ in $n_x-i$ diagonal entries. Since the determinant of an upper-triangular matrix is the product of its diagonal entries, we have

$$
\det(T_{i}) = \begin{cases} s^{n_x-i} & \text{when $i$ is odd} \\ -s^{n_x-i} & \text{when $i$ is even} \end{cases}
$$

Plug this in, and our result follows.
Now, the reason that controllable canonical form is useful is that if we choose the input

$$
u = -Lz
$$

for some choice of gains

$$
L = \begin{bmatrix} \ell_{1} & \dotsm & \ell_{n_x} \end{bmatrix}
$$

then the "$A$ matrix" of the closed-loop system is

$$
A_\text{ccf}-B_\text{ccf}L =
\begin{bmatrix}
-a_{1}-\ell_{1} & -a_{2}-\ell_{2} & \dotsm & -a_{n_x-1}-\ell_{n_x-1} & -a_{n_x}-\ell_{n_x} \\
1 & 0 & \dotsm & 0 & 0 \\
0 & 1 & \dotsm & 0 & 0 \\
\vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & \dotsm & 1 & 0
\end{bmatrix}
$$

The characteristic equation of this closed-loop system, computed in the same way as for $A_\text{ccf}$, is

$$
s^{n_x} + (a_{1}+\ell_{1}) s^{n_x-1} + \dotsm + (a_{n_x-1}+\ell_{n_x-1}) s + (a_{n_x}+\ell_{n_x}).
$$

If you want this characteristic equation to look like

$$
s^{n_x} + r_{1} s^{n_x-1} + \dotsm + r_{n_x-1} s + r_{n_x}
$$

then it's obvious what gains you should choose

$$
\ell_{1} = r_{1}-a_{1} \qquad\qquad \ell_{2} = r_{2}-a_{2} \qquad\qquad \dotsm \qquad\qquad \ell_{n_x}=r_{n_x}-a_{n_x}.
$$

So, if you have a system in controllable canonical form, then it is easy to choose gains that make the characteristic equation of the closed-loop system look like anything you want (i.e., to put the closed-loop eigenvalues anywhere you want). In other words, it is easy to do control design.


## Putting a system in controllable canonical form

We have seen that controllable canonical form is useful. Now we'll see how to put a system in this form.
Suppose we have a system

$$
\dot{x} = Ax+Bu
$$

and we want to choose an invertible matrix $V$ so that if we define a new state variable $z$ by

$$
x = Vz
$$

then we can rewrite the system as

$$
\dot{z} = A_\text{ccf}z+B_\text{ccf}u
$$

where

$$
A_\text{ccf} = V^{-1}AV
\qquad\qquad\text{and}\qquad\qquad
B_\text{ccf} = V^{-1}B
$$

are in controllable canonical form.
The trick is to look at the so-called **controllability matrix** that is associated with the transformed system:

$$
W_\text{ccf} = \begin{bmatrix} B_\text{ccf} & A_\text{ccf}B_\text{ccf} & \dotsm & A_\text{ccf}^{n_x-1}B_\text{ccf} \end{bmatrix}.
$$

We will talk more later about the controllability matrix — for now, notice that

$$
\begin{align*}
B_\text{ccf} &= V^{-1}B \\
A_\text{ccf}B_\text{ccf} &= V^{-1}AVV^{-1}B = V^{-1}AB \\
A_\text{ccf}^{2}B_\text{ccf} &= A_\text{ccf} (A_\text{ccf}B_\text{ccf}) = V^{-1}AVV^{-1}AB = V^{-1}A^{2}B \\
\vdots\qquad &= \qquad\vdots
\end{align*}
$$

You see the pattern here, I'm sure. The result is:

$$
\begin{align*}
W_\text{ccf} &= \begin{bmatrix} V^{-1}B & V^{-1}AB & \dotsm & V^{-1}A^{n_x-1}B \end{bmatrix} \\
&= V^{-1}\begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix} \\
&= V^{-1}W
\end{align*}
$$

where

$$
W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}
$$

is the controllability matrix associated with the original system.

There are three things to note:

* $A$ and $B$ are things that you know — you have a description of the original system, as always — so you can compute $W$.

* $A_\text{ccf}$ and $B_\text{ccf}$ are also things that you know — the values $a_1, \dotsc, a_{n_x}$ in the top row of $A_\text{ccf}$ are the coefficients of the characteristic polynomial of the matrix $A$ — so you can compute $W_\text{ccf}$.

* $W$ is a square $n_x \times n_x$ matrix — it has $n_x$ columns $B$, $AB$, and so forth, all of which have size $n_x \times 1$. So, if $W$ has non-zero determinant, then you can find its inverse.

As a consequence, you can solve for the matrix $V^{-1}$:

$$
V^{-1} = W_\text{ccf}W^{-1}.
$$

Now, suppose you design a control policy for the transformed system:

$$
u = -K_\text{ccf}z.
$$

Remember, you can do this easily, because the transformed system is in controllable canonical form.
We can compute the equivalent control policy, that would be applied to the original system:

$$
u = -K_\text{ccf}z = -K_\text{ccf}V^{-1}x = -K_\text{ccf}W_\text{ccf}W^{-1}x.
$$

In particular, if we choose

$$
K = K_\text{ccf}W_\text{ccf}W^{-1}
$$

then we get the behavior that we want.
Again, we emphasize that this only works if $W$ is invertible, and that $W$ is only invertible if $\det(W)\neq 0$.

## A systematic process for control design

Apply the input

$$
u=-Kx
$$

to the open-loop system

$$
\dot{x} = Ax+Bu
$$

and you get the closed-loop system

$$
\dot{x} = (A-BK)x.
$$

Suppose we want to choose $K$ to put the eigenvalues of the closed-loop system at

$$p_{1},\dotsc,p_{n_x}.$$

Using the results of the previous sections, we know we can do this as follows:

*   Compute the characteristic equation that we want:

    $$(s-p_{1})\dotsm(s-p_{n_x}) = s^{n_x}+r_{1}s^{n_x-1}+ \dotsm + r_{n_x-1}s + r_{n_x}.$$

*   Compute the characteristic equation that we have:
    
    $$\det(sI-A) = s^{n_x}+a_{1}s^{n_x-1}+ \dotsm + a_{n_x-1}s + a_{n_x}$$
    
    
*   Compute the controllability matrix of the original system (and check that $\det(W)\neq 0$):

    $$W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}$$
    
*   Compute the controllability matrix of the transformed system:
    
    $$
    W_\text{ccf} = \begin{bmatrix} B_\text{ccf} & A_\text{ccf}B_\text{ccf} & \dotsm & A_\text{ccf}^{n_x-1}B_\text{ccf} \end{bmatrix}
    $$

    where

    $$
    A_\text{ccf} =
    \begin{bmatrix}
    -a_{1} & -a_{2} & \dotsm & -a_{n-1} & -a_{n} \\
    1 & 0 & \dotsm & 0 & 0 \\
    0 & 1 & \dotsm & 0 & 0 \\
    \vdots & \vdots & \ddots & \vdots & \vdots \\
    0 & 0 & \dotsm & 1 & 0
    \end{bmatrix}
    \qquad\qquad
    B_\text{ccf} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ \vdots \\ 0 \end{bmatrix}
    $$

*   Compute the gains for the transformed system:

    $$
    K_\text{ccf} = \begin{bmatrix} r_{1}-a_{1} & \dotsm & r_{n_x}-a_{n_x} \end{bmatrix}
    $$

*   Compute the gains for the original system:

    $$
    K = K_\text{ccf}W_\text{ccf}W^{-1}
    $$

And we're done!
This process is easy to implement, without any symbolic computation.
Remember, although this method only works for systems with exactly one input (i.e., when $n_u = 1$), similar methods work for systems with multiple inputs — in python, use [scipy.signal.place_poles](https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html#scipy-signal-place-poles).

## How to decide when eigenvalue placement is possible

We say that a system is **controllable** if eigenvalue placement is possible. We have seen eigenvalue placement with Ackermann's method (for the special case when $n_u = 1$) is only possible when the controllability matrix $W$ is invertible. Here is a generalization of that same result to any system:

{: .result}
> The system
> 
> $$\dot{x} = Ax+Bu$$
> 
> is **controllable** if and only if
> the **controllability matrix**
> 
> $$W = \begin{bmatrix} B & AB & \dotsm & A^{n_x-1}B \end{bmatrix}$$
> 
> is full rank, where $n_x$ is the number of states.

Let's break this statement down.

First, suppose there is only one input, so $n_u = 1$.  In that case, $A$ has size $n_x \times n_x$ and $B$ has size $n_x \times 1$. Therefore, $W$ is a square matrix of size $n_x \times n_x$, so $W$ being full rank simply means that it is invertible (i.e., that $\det(W) \neq 0$).

Now, suppose there is more than one input, so $n_u > 1$. Then, $B$ has size $n_x \times n_u$.  The matrix $W$ then has size $n_x \times n_xn_u$, and $W$ is no longer square. Although we can't invert a non-square matrix, we can still find its [rank](https://en.wikipedia.org/wiki/Rank_(linear_algebra)). In particular, it is easy to find the rank of a non-square matrix in python using [numpy.linalg.matrix_rank](https://numpy.org/doc/stable/reference/generated/numpy.linalg.matrix_rank.html#numpy-linalg-matrix-rank). We say that $W$ is **full rank** if and only if its rank is $n_x$.

If $W$ is full rank, the system is controllable, and eigenvalue placement will work. If $W$ is not full rank, the system is not controllable, and eigenvalue placement will not work.

We can actually say a little more than this. It turns out that if the controllability matrix $W$ is not full rank, then although we cannot place *all* of the system's eigenvalues, we can place *some* of these eigenvalues, while the others will remain in the same location no matter what gain matrix $K$ we choose. The eigenvalues we *can* place are often called "controllable eigenvalues" or "controllable modes," while the eigenvalues we can *not* place are often called "uncontrollable eigenvalues" or "uncontrollable modes." If the rank of $W$ is $n_x - 1$, then we can place $n_x - 1$ eigenvalues, and $1$ eigenvalue is uncontrollable. If the rank of $W$ is $n_x - 2$, then we can place $n_x - 2$ eigenvalues, and $2$ eigenvalues are uncontrollable. And so forth.


