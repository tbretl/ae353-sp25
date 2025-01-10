---
title: State space models
parent: Reference
nav_order: 2
---

# State space models
{: .no_toc }

- TOC
{:toc }

## What is a state space model?

A **state-space model** is a set of ordinary differential equations that can be written in this form:

$$
\dot{x} = Ax + Bu
$$

There are two variables:

* $x$ is the **state**
* $u$ is the **input**

Both variables are functions of time, so we could have written $x(t)$ and $u(t)$ --- but in general, we won't use that notation unless we need it. We use dots to indicate time derivatives, so $\dot{x}$ means $dx(t)/dt$.

The state and input may have more than one element --- that is to say, they may be vectors (represented by a matrix of numbers) and not scalars (represented by a single number). Suppose, in particular, that:

* the state $x$ has $n_x$ elements
* the input $u$ has $n_u$ elements

Then, we would represent them as column matrices:

$$
x =
\begin{bmatrix}
x_1 \\ \vdots \\ x_{n_x}
\end{bmatrix}
\qquad
u =
\begin{bmatrix}
u_1 \\ \vdots \\ u_{n_u}
\end{bmatrix}
$$

Taking the time derivative of a matrix is equivalent to taking the time derivative of each element of that matrix, so we would write

$$
\dot{x} =
\begin{bmatrix}
\dot{x}_1 \\ \vdots \\ \dot{x}_{n_x}
\end{bmatrix}
$$

The state-space model also has two constants: $A$ and $B$. If $x$ and $u$ are column matrices with (possibly) more than one element, then these constants have to be matrices as well:

* $A$ is a constant matrix of size $n_x \times n_x$
* $B$ is a constant matrix of size $n_x \times n_u$

The state space model has two key properties:

* it is **linear** because both $\dot{x}$ is a linear function of $x$ and $u$
* it is **time-invariant** because $A$ and $B$ are constant

<div class="alert alert-warning">
Other people use other symbols for both the variables and constants in a state space model. Indeed, we will sometimes use other symbols for these things as well. For example:

$$
\dot{z} = Ez + Fv
$$

This is also a "state space model," in which the state is $z$ and the input is $v$.
</div>


## How do I put a system in state space form?

Suppose we are given a description of a dynamic system. We would like to describe this same system with a state space model.

Remember that every state space model is linear. The equations of motion that describe a dynamic system are often nonlinear. So apparently, we will have to accept that state space models can only *approximate* some of the systems we want to describe.

We will use **linearization** to arrive at this approximation, in four steps.

**Step 1.** Rewrite the system as a set of first-order ordinary differential equations:

$$
\dot{m} = f(m,n)
$$

In this expression, the variables $m$ and $n$ are functions of time and can have more than one element --- in general, you should represent them as column matrices. As we said before, the function $f(\cdot)$ will often be nonlinear.

**Step 2.** Find an equilibrium point $m_{e}, n_{e}$ of the system by solving this equation:

$$
0 = f(m_{e},n_{e})
$$

A solution to this equation is called an equilibrium point because if

$$m = m_e \qquad n = n_e$$

then

$$\dot{m} = f(m_e, n_e) = 0$$

and so $m$ remains constant. That is to say, if the system reaches an equilibrium point, then it stays there. This is an important property! The goal of almost every controller we design in this course will be to make a system reach an equilibrium point quickly and reliably.

This equation may have no solutions, in which case no equilibrium point exists for the system. This is bad. We will ignore this situation for now.

This equation also may have many solutions, in which case you have to make a choice. A good choice is whatever equilibrium point you would like your system to achieve.

**Step 3.** Define the state and input as follows:

$$
x = m-m_{e}
\qquad
u = n-n_{e}
$$

Note that $x$ measures *error* --- the *difference* between $m$ and its equilibrium value. When error is zero, the system has reached equilibrium and is doing what we want. Apparently, with this way of defining the state and the input, "control design" means choosing $u$ so that $x$ goes to zero.

**Step 4.** Compute $A$ and $B$ as follows:

$$
A = \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
\qquad
B = \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})}
$$

<div class="alert alert-warning">
Recall that

$$
\frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})}
$$

is the <a href="https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant">Jacobian (i.e., matrix of partial derivatives)</a> of $f$ with respect to $m$, evaluated at the equilibrium point.
</div>

Why does this make any sense? Look again at the ODEs that describe the original system:

$$\dot{m} = f(m, n)$$

First, because

$$x = m - m_e$$

then

$$\dot{x} = \dot{m} - 0 = \dot{m}$$

So the *left*-hand side of the ODEs can simply be replaced with $\dot{x}$.

Now, suppose we want a linear approximation to the *right*-hand side of the ODEs --- the function $f(m, n)$. One way to find such an approximation is to take a [Taylor's series expansion](https://en.wikipedia.org/wiki/Taylor_series) about $m_e, n_e$ up to first order:

$$
\begin{aligned}
f(m, n)
&\approx f(m_e, n_e) + \frac{\partial f}{\partial m}\biggr\rvert_{(m_{e},n_{e})} \left( m - m_e \right) + \frac{\partial f}{\partial n}\biggr\rvert_{(m_{e},n_{e})} \left( n - n_e \right) \\
&= 0 + A x + B u \\
&= A x + B u
\end{aligned}
$$

There you have it: "$\dot{x} = Ax + Bu$" is a first-order (i.e., linear) approximation to "$\dot{m} = f(m, n)$".


### Example (first-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\dot{\omega} + 2 \omega = \tau$$

Let's apply our method to put this system in state space form.

We begin by rewriting it as a set of first-order ODEs. Lucky us, the system is already described by just one first-order ODE, so all we need to do is solve for $\dot{w}$:

$$
\dot{\omega} = f(\omega, \tau) = -2\omega + \tau
$$

Next, we find an equilibrium point by solving

$$0 = -2 \omega_e + \tau_e$$

In this case, there are many solutions. Suppose we pick this one:

$$\omega_e = 10 \qquad \tau_e = 20$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \omega - \omega_e \qquad u = \tau - \tau_e$$

Finally, we compute $A$ and $B$ by taking Jacobians (easy in this case because all the variables are scalars):

$$
\begin{aligned}
A &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \omega}\biggr\rvert_{(\omega_{e},\tau_{e})} = -2 \\[1em]
B &= \frac{\partial \left(-2\omega + \tau\right)}{\partial \tau}\biggr\rvert_{(\omega_{e},\tau_{e})} = 1
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= -2x + u
\end{aligned}
$$

Note that the original system was linear, so there was no approximation here. We could probably have skipped most of these steps and written the system in state-space form by inspection. On the other hand, it is nice to know that the process of "linearization" still works even in this simple case.


### Example (second-order)

Consider the system with dynamics that are described by the following equation of motion:

$$\ddot{q} + 3 \sin q = \tau$$

Let's apply our method to put this system in state-space form.

We begin by rewriting it as a set of first-order ODEs:

* We find the time-derivative of $q$ with highest order --- in this case, $\ddot{q}$, of order 2.
* We define *new variables* for each time-derivative of $q$ with lower order --- in this case, $\dot{q}$, the only time-derivative with order between 0 and 2. We might choose the following name for this time-derivative:

$$ v = \dot{q} $$

* We rewrite the original ODEs (in this case, just one) in terms of these new variables:

$$\dot{v} + 3\sin q = \tau$$

* We add one extra ODE for each new variable (in this case, just one extra) --- this is trivial, coming from the way we defined these new variables:

$$\dot{q} = v$$

* We collect the original ODEs and the extra ODEs together, if necessary solving for all of the time-derivatives (that's not necessary here):

$$
\begin{aligned}
\dot{q} &= v \\
\dot{v} &= -3\sin q + \tau
\end{aligned}
$$

* Finally, we rewrite our result in the form $\dot{m} = f(m, n)$ by collecting things in column matrices as follows:

$$\begin{bmatrix} \dot{q} \\ \dot{v} \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

Note that, as desired, these rewritten ODEs have time derivatives that are at most of first order. Also note that all of these time-derivatives are on the left-hand side of the equations --- none appear on the right-hand side.

Next, we find an equilibrium point by solving

$$\begin{bmatrix} 0 \\ 0 \end{bmatrix} = \begin{bmatrix} v \\ -3\sin q + \tau \end{bmatrix}$$

There are many solutions. Suppose we pick this one:

In this case, there are many solutions. Suppose we pick this one:

$$q_e = \pi / 2 \qquad v_e = 0 \qquad \tau_e = 3$$

Then, we define the state and input based on this choice of equilibrium point:

$$x = \begin{bmatrix} q - q_e \\ v - v_e \end{bmatrix} \qquad u = \begin{bmatrix}\tau - \tau_e\end{bmatrix}$$

Finally, we compute $A$ and $B$ by taking Jacobians.

$$
\begin{aligned}
A
&= \frac{\partial f}{\partial \begin{bmatrix} q \\ v \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial q} & \dfrac{\partial(v)}{\partial v} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial q} & \dfrac{\partial(-3\sin q + \tau)}{\partial v} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 & 1 \\ -3\cos q & 0 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \\[1em]
B
&= \frac{\partial f}{\partial \begin{bmatrix} \tau \end{bmatrix}}\biggr\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} \dfrac{\partial(v)}{\partial \tau} \\ \dfrac{\partial(-3\sin q + \tau)}{\partial \tau} \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \left.\begin{bmatrix} 0 \\ 1 \end{bmatrix}\right\rvert_{\left(\begin{bmatrix}q_e\\v_e\end{bmatrix},\begin{bmatrix}\tau_{e}\end{bmatrix}\right)} \\
&= \begin{bmatrix} 0 \\ 1 \end{bmatrix}
\end{aligned}
$$

The resulting state space model is

$$
\begin{aligned}
\dot{x} &= Ax + Bu \\
&= \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}x + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u
\end{aligned}
$$

The original system was nonlinear and the state space model is linear, so there *must* be some approximation here! As we will see, this approximation is good near the equilibrium point but can be very bad elsewhere.