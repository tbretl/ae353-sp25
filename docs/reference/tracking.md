---
title: Reference tracking
parent: Reference
nav_order: 9
---

# Reference tracking
{: .no_toc }

- TOC
{:toc }

## Overview

The controllers we have designed so far make the state converge to zero — that is, they make

$$x(t) \rightarrow 0 \quad\text{as}\quad t \rightarrow \infty.$$

Suppose we want the state to converge to something else — that is, to make

$$x(t) \rightarrow x_\text{des} \quad\text{as}\quad t \rightarrow \infty.$$

We will see that, under certain conditions, this is easy to do.

## Reference tracking with full state feedback

Consider the dynamic model

$$\dot{m} = f(m, n)$$

where $m \in \mathbb{R}^\ell$. Suppose we linearize this model about some equilibrium point $(m_e, n_e)$ to produce the state-space model

$$\dot{x} = Ax + Bu$$

where

$$x = m - m_e \qquad\qquad u = n - n_e.$$

Suppose we design linear state feedback

$$u = -Kx$$

that would make the closed-loop system

$$\dot{x} = (A - BK) x$$

asymptotically stable — that is, that would make

$$x(t) \rightarrow 0 \qquad\text{as}\qquad t \rightarrow 0.$$

Denote the standard basis for $\mathbb{R}^\ell$ by

$$
e_1 = \begin{bmatrix} 1 \\ 0 \\ \vdots \\ 0 \end{bmatrix}
\qquad
e_2 = \begin{bmatrix} 0 \\ 1 \\ \vdots \\ 0 \end{bmatrix}
\qquad
\dotsm
\qquad
e_\ell = \begin{bmatrix} 0 \\ 0 \\ \vdots \\ 1 \end{bmatrix}.
$$

Suppose there is some index $i \in \\{ 1, \dotsc, \ell\\}$ that satisfies

$$ \tag{9} f(m + e_i r, n) = f(m, n) \qquad\text{for all}\qquad r \in \mathbb{R}. $$

That is, suppose the function $f$ is constant in — or, does not vary with — the $i$'th element of $m$. Then, the following three things are true:

---

**Invariance of equilibrium point.**
Since 

$$
\begin{align*}
f(m_e + e_i r, n_e)
&= f(m_e, n_e) && \qquad\text{because of (9)} \\
&= 0 && \qquad\text{because ($m_e, n_e$) is an equilibrium point},
\end{align*} 
$$

then $(m_e + e_i r, n_e)$ is also an equilibrium point for any $r \in \mathbb{R}$.

---

**Invariance of error in approximation of the dynamic model.** The linear model

$$\dot{x} = Ax + Bu$$

is an approximation to the nonlinear model

$$\dot{m} = f(m, n).$$

The amount of error in this approximation is

$$
\begin{align*}
e(m, n)
&= f(m, n) - (Ax + Bu) \\
&= f(m, n) - (A (m - m_e) + B (n - n_e)).
\end{align*}
$$

We will show that this approximation error is constant in — or, does not vary with — the $i$'th element of $m$.
Before we do so, we will prove that

$$\tag{10} A e_i r = 0 \text{ for any } r \in \mathbb{R}.$$

First, we show that the $i$'th column of $A$ is zero:

$$
\begin{align*}
\frac{\partial f}{\partial m_i} \Biggr\rvert_{(m_e, n_e)}
&= \lim_{h \rightarrow 0} \frac{f(m_e + e_i h, n_e) - f(m_e, n_e)}{h} \\
&= \lim_{h \rightarrow 0} \frac{f(m_e, n_e) - f(m_e, n_e)}{h} \\
&= \lim_{h \rightarrow 0} \frac{0}{h} \\
&= 0.
\end{align*}
$$

Next, denote the columns of $A$ by

$$A = \begin{bmatrix} A_1 & A_2 & \dotsm & A_\ell \end{bmatrix}.$$

Then, we compute

$$A e_i r = \left( \sum_{j = 1}^{\ell} A_j e_{ij} \right) r = A_i r = 0.$$

Now, for the approximation error:

$$
\begin{align*}
e(m + e_i r, n)
&= f(m + e_i r, n) - (A (m + e_i r - m_e) + B (n - n_e)) \\
&= f(m, n) - (A (m + e_i r - m_e) + B (n - n_e)) &&\qquad\text{because of (9)}\\
&= f(m, n) - (A (m + m_e) + B (n - n_e)) - A e_i r \\
&= f(m, n) - (A (m + m_e) + B (n - n_e))  &&\qquad\text{because of (10)} \\
&= e(m, n).
\end{align*}
$$

What this means is that our state-space model is just as accurate near $(m_e + e_i r, n_e)$ as it is near the equilibrium point $(m_e, n_e)$.

---

**Invariance of control.** Suppose we implement linear state feedback **with reference tracking**:

$$u = -K(x - x_\text{des})$$

where

$$x_\text{des} = e_i r$$

for any $r\in\mathbb{R}$. Let's assume (for now) that $r$ is constant, and so $x_\text{des}$ is also constant. What will $x(t)$ converge to in this case? Let's find out. First, we define the error

$$ z = x - x_\text{des} $$

and note that

$$ u = - K (x - x_\text{des}) = - K z.$$

Second, we derive an expression for the closed-loop system in terms of this error:

$$
\begin{align*}
\dot{z}
&= \frac{d}{dt} \left(x - x_\text{des}\right) \\
&= \dot{x} - 0 \\
&= Ax + Bu \\
&= A(z + x_\text{des}) + B(-Kz) \\
&= (A - BK) z + A x_\text{des} \\
&= (A - BK) z + A e_i r \\
&= (A - BK) z && \qquad\text{because of (10)}.
\end{align*}
$$

This means that

$$z(t) \rightarrow 0 \qquad\text{as}\qquad t \rightarrow \infty$$

or equivalently that

$$x(t) \rightarrow x_\text{des} \qquad\text{as}\qquad t \rightarrow \infty$$

so long as all eigenvalues of $A - BK$ have negative real part — exactly the same conditions under which the closed-loop system *without* reference tracking would have been asymptotically stable.



{: .result-title}
> Result: **Reference tracking with full state feedback**
> 
> Consider a system
> 
> $$\dot{m} = f(m, n)$$
> 
> that satisfies
> 
> $$f(m + e_i r, n) = f(m, n) \quad\text{for any}\quad r \in \mathbb{R}.$$ 
> 
> Linearize this system about an equilibrium point $(m_e, n_e)$ to produce the state-space model
> 
> $$\dot{x} = Ax + Bu$$
> 
> where
> 
> $$x = m - m_e \qquad\text{and}\qquad u = n - n_e.$$
> 
> Apply linear state feedback with reference tracking as
> 
> $$u = - K (x - x_\text{des})$$
> 
> where
> 
> $$x_\text{des} = e_i r$$
> 
> for any $r \in \mathbb{R}$. Then,
> 
> $$x(t) \rightarrow x_\text{des} \qquad\text{as}\qquad t \rightarrow \infty$$
> 
> if and only if all eigenvalues of $A - BK$ have negative real part.






## Reference tracking with partial state feedback

Consider the system

$$
\begin{align*}
\dot{m} &= f(m, n) && \qquad\qquad\text{dynamic model} \\
o &= g(m, n) && \qquad\qquad\text{sensor model} \\
\end{align*}
$$

where $m \in \mathbb{R}^\ell$. Suppose we linearize this model about some equilibrium point $(m_e, n_e)$ to produce the state-space model

$$
\begin{align*}
\dot{x} &= Ax+Bu \\
y &= Cx + Du \\
\end{align*}
$$

where

$$x = m - m_e \qquad\qquad u = n - n_e \qquad\qquad y = o - g(m_e, n_e).$$

Suppose we design a controller

$$u = -K\widehat{x}$$

and observer

$$\widehat{x} = A\widehat{x} + Bu - L(C\widehat{x} - y)$$

that would make the closed-loop system

$$
\begin{bmatrix} \dot{x} \\ \dot{x}_\text{err} \end{bmatrix}
=
\begin{bmatrix} A - BK & -BK \\ 0 & A - LC \end{bmatrix} \begin{bmatrix} x \\ x_\text{err} \end{bmatrix}
$$

asymptotically stable — that is, that would make

$$x(t) \rightarrow 0 \qquad\text{and}\qquad x_\text{err}(t) \rightarrow 0 \qquad\text{as}\qquad t \rightarrow 0$$

where

$$x_\text{err} = \widehat{x} - x.$$


Suppose, as for [reference tracking with full state feedback](#reference-tracking-with-full-state-feedback), that there is some index $i \in \\{ 1, \dotsc, \ell\\}$ for which

$$ f(m + e_i r, n) = f(m, n) \qquad\text{for all}\qquad r \in \mathbb{R}. $$

This implies **invariance of equilibrium point** and **invariance of error in approximation of the dynamic model**, just like before. Suppose it is also true that, for some constant vector $g_0$, the sensor model satisfies

$$ \tag{11} g(m + e_i r, n) = g_0 r + g(m, n) \qquad\text{for all}\qquad r \in \mathbb{R}. $$

Then, the following two more things are true:

---

**Invariance of error in approximation of the sensor model.** The linear model

$$y = Cx + Du$$

is an approximation to the nonlinear model

$$o = g(m, n).$$

The amount of error in this approximation is

$$
\begin{align*}
e(m, n)
&= g(m, n) - (g(m_e, n_e) + Cx + Du) \\
&= g(m, n) - (g(m_e, n_e) + C (m - m_e) + D (n - n_e)).
\end{align*}
$$

We will show that this approximation error is constant in — or, does not vary with — the $i$'th element of $m$.
First, we show that the $i$'th column of $C$ is $g_0$:

$$
\begin{align*}
\frac{\partial g}{\partial m_i} \Biggr\rvert_{(m_e, n_e)}
&= \lim_{h \rightarrow 0} \frac{g(m_e + e_i h, n_e) - g(m_e, n_e)}{h} \\
&= \lim_{h \rightarrow 0} \frac{g_0 h + g(m_e, n_e) - g(m_e, n_e)}{h} && \qquad\text{because of (11)}\\
&= \lim_{h \rightarrow 0} \frac{g_0 h}{h} \\
&= g_0.
\end{align*}
$$

Next, denote the columns of $C$ by

$$C = \begin{bmatrix} C_1 & C_2 & \dotsm & C_\ell \end{bmatrix}.$$

Then, we compute

$$\tag{12} C e_i r = \left( \sum_{j = 1}^{\ell} C_j e_{ij} \right) r = C_i r = g_0 r.$$

Now, for the approximation error:


$$
\begin{align*}
e(m + e_i r, n)
&= g(m + e_i r, n) - (g(m_e, n_e) + C (m + e_i r - m_e) + D (n - n_e)) \\
&= g_0 r + g(m, n) - (g(m_e, n_e) + C (m + e_i r - m_e) + D (n - n_e)) &&\text{from (11)}\\
&= g_0 r + g(m, n) - (g(m_e, n_e) + C (m - m_e) + D (n - n_e)) - C e_i r \\
&= g_0 r + g(m, n) - (g(m_e, n_e) + C (m - m_e) + D (n - n_e)) - g_0 r &&\text{from (12)}\\
&= g(m, n) - (g(m_e, n_e) + C (m - m_e) + D (n - n_e)) \\
&= e(m, n).
\end{align*}
$$

What this means is that our state-space model is just as accurate near $(m_e + e_i r, n_e)$ as it is near the equilibrium point $(m_e, n_e)$.

---

**Invariance of control.** Suppose we implement linear state feedback **with reference tracking**:

$$u = -K(\widehat{x} - x_\text{des})$$

where

$$x_\text{des} = e_i r$$

for any $r\in\mathbb{R}$. Let's assume (for now) that $r$ is constant, and so $x_\text{des}$ is also constant. What will $x(t)$ converge to in this case? Let's find out. First, we define the state error

$$ z = x - x_\text{des} $$

and the state estimate error

$$ x_\text{err} = \widehat{x} - x.$$

Second, we derive an expression for the closed-loop system in terms of these errors. Let's start with the state error:

$$
\begin{align*}
\dot{z}
&= \frac{d}{dt} \left(x - x_\text{des}\right) \\
&= \dot{x} - 0 \\
&= Ax + Bu \\
&= A(z + x_\text{des}) + B(-K(\widehat{x} - x)) \\
&= A(z + x_\text{des}) - BK(x_\text{err} + z) \\
&= (A - BK) z - BK x_\text{err} + A x_\text{des} \\
&= (A - BK) z - BK x_\text{err} + A e_i r \\
&= (A - BK) z - BK x_\text{err} && \qquad\text{because of (10)}.
\end{align*}
$$

Now, for the state estimate error:

$$
\begin{align*}
\dot{x}_\text{err}
&= \dot{\widehat{x}} - \dot{x} \\
&= \left( A \widehat{x} + Bu - L(C\widehat{x} - y) \right) - \left(Ax + Bu\right) \\
&= A(\widehat{x} - x) - LC (\widehat{x} - x) \\
&= (A - LC) x_\text{err}.
\end{align*}
$$

Putting these together, we have

$$
\begin{bmatrix} \dot{z} \\ \dot{x}_\text{err} \end{bmatrix} = \begin{bmatrix} A - BK & -BK \\ 0 & A - LC \end{bmatrix} \begin{bmatrix} z \\ x_\text{err} \end{bmatrix}.
$$

This means that

$$z(t) \rightarrow 0 \qquad\text{and}\qquad x_\text{err}(t) \rightarrow 0 \qquad\text{as}\qquad t \rightarrow \infty$$

or equivalently that

$$x(t) \rightarrow x_\text{des} \qquad \widehat{x}(t) \rightarrow x(t) \qquad\text{as}\qquad t \rightarrow \infty$$

so long as all eigenvalues of $A - BK$ and all eigenvalues of $A - LC$ have negative real part — exactly the same conditions under which the closed-loop system *without* reference tracking would have been asymptotically stable.



{: .result-title}
> Result: **Reference tracking with partial state feedback**
> 
> Consider a system
> 
> $$
> \begin{align*}
> \dot{m} &= f(m, n) && \qquad\qquad\text{dynamic model} \\
> o &= g(m, n) && \qquad\qquad\text{sensor model} \\
> \end{align*}
> $$
> 
> that satisfies
> 
> $$f(m + e_i r, n) = f(m, n) \quad\text{for any}\quad r \in \mathbb{R}$$
> 
> and
> 
> $$ g(m + e_i r, n) = g_0 r + g(m, n) \qquad\text{for all}\qquad r \in \mathbb{R} $$
> 
> for some constant vector $g_0$. Linearize this system about some equilibrium point $(m_e, n_e)$ to produce the state-space model
> 
> $$
> \begin{align*}
> \dot{x} &= Ax+Bu \\
> y &= Cx + Du \\
> \end{align*}
> $$
> 
> where
> 
> $$x = m - m_e \qquad\qquad u = n - n_e \qquad\qquad y = o - g(m_e, n_e).$$
> 
> Apply the observer
> 
> $$\widehat{x} = A\widehat{x} + Bu - L(C\widehat{x} - y)$$
> 
> and the controller (with reference tracking)
> 
> $$u = -K(\widehat{x} - x_\text{des})$$
> 
> where
> 
> $$x_\text{des} = e_i r$$
> 
> for any $r \in \mathbb{R}$. Then,
> 
> $$x(t) \rightarrow x_\text{des} \qquad \widehat{x}(t) \rightarrow x(t) \qquad\text{as}\qquad t \rightarrow \infty$$
> 
> if and only if all eigenvalues of $A - BK$ and all eigenvalues of $A - LC$ have negative real part.



## Tracking more than one element of the state

Our discussion of reference tracking with [full state feedback](#reference-tracking-with-full-state-feedback) and with [partial state feedback](#reference-tracking-with-partial-state-feedback) has assumed that we want to track desired values of exactly one element $m_i$ of the nonlinear state $m$. All of this generalizes immediately to the case where we want to track desired values of more than one element of $m$.

In particular, suppose there are *two* indices $i, j \in \\{1, \dotsc, \ell\\}$ that satisfy

$$
f(m + e_i r_i + e_j r_j, n) = f(m, n) \quad\text{for any}\quad r_i, r_j \in \mathbb{R}
$$

and, in the case of partial state feedback, that also satisfy

$$
g(m + e_i r_i + e_j r_j, n) = g_{0i} r_i + g_{0j} r_j + g(m, n) \quad\text{for any}\quad r_i, r_j \in \mathbb{R}
$$

for constant vectors $g_{0i}$ and $g_{0j}$. Then, choosing

$$
x_\text{des} = e_ir_i + e_jr_j
$$

would produce the same results that were derived previously.


## Choosing the desired state to keep errors small

Our proof that tracking "works" relies largely on having shown that our state-space model is just as accurate near $(m_e + e_i r, n_e)$ as it is near the equilibrium point $(m_e, n_e)$. Equivalently, it relies on having shown that this model is just as accurate near $x = x_\text{des}$ as it is near $x = 0$.

Despite this fact, it is still important to keep the state error

$$x - x_\text{des}$$

small. The reason is that the input is proportional to the error — for full state feedback as

$$u = - K (x - x_\text{des})$$

and for partial state feedback as

$$u = - K (\widehat{x} - x_\text{des}).$$

So, if error is large, the input may exceed bounds (e.g., limits on actuator torque). Since our state-space model does not include these bounds, it may be inaccurate when inputs are large.

As a consequence, it is important in practice to choose $x_\text{des}$ so that the state error

$$x - x_\text{des}$$

remains small. Here is one common way to do this, both in the case of full state feedback and partial state feedback.


{: .result-title}
> Result: Choosing the desired state in the case of full state feedback
> 
> Suppose $x_\text{goal}$ is the state you actually want to achieve. Suppose $e_\text{max} > 0$ is an upper bound on the state error that you are willing to tolerate. Then, choose
> 
> $$
> x_\text{des} =
> \begin{cases}
> x + e_\text{max} \left(\dfrac{x_\text{goal} - x}{\|x_\text{goal} - x\|} \right) & \text{if } \| x_\text{goal} - x \| > e_\text{max}, \\
> x_\text{goal} & \text{otherwise.}
> \end{cases}
> $$

{: .result-title}
> Result: Choosing the desired state in the case of partial state feedback
> 
> Suppose $x_\text{goal}$ is the state you actually want to achieve. Suppose $e_\text{max} > 0$ is an upper bound on the state error that you are willing to tolerate. Then, choose
> 
> $$
> x_\text{des} =
> \begin{cases}
> \widehat{x} + e_\text{max} \left(\dfrac{x_\text{goal} - \widehat{x}}{\|x_\text{goal} - \widehat{x}\|} \right) & \text{if } \| x_\text{goal} - \widehat{x} \| > e_\text{max}, \\
> x_\text{goal} & \text{otherwise.}
> \end{cases}
> $$
