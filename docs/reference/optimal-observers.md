---
title: Optimal observers
parent: Reference
nav_order: 8
---

# Optimal observer design
{: .no_toc }

These notes were originally written by T. Bretl and were transcribed by S. Bout.

- TOC
{:toc }

## Statement of the problem {#secStatement-kf}

Here is the deterministic, finite-horizon, continuous-time **Kalman
Filter (KF)** problem --- i.e., the optimal control problem that one would
solve to produce an optimal observer:

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),n_{[t_{0},t_{1}]},d_{[t_{0},t_{1}]}}
&\qquad
n(t_{0})^{T}M_{o}n(t_{0})+
\int_{t_{0}}^{t_{1}} \left( n(t)^{T}Q_{o}n(t)+d(t)^{T}R_{o}d(t) \right) dt \\
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t) \\
&\qquad
y(t) = Cx(t)+n(t)\end{aligned}$$

The interpretation of this problem is
as follows. The current time is $t_{1}$. You have taken measurements
$y(t)$ over the time interval $[t_{0}, t_{1}]$. You are looking for
noise $n(t)$ and disturbance $d(t)$ over this same time interval and for
an estimate $x(t_{1})$ of the current state that would best explain
these measurements.

The matrices $Q_{o}$, $R_{o}$, and $M_{o}$ are parameters that can be
used to trade off noise (the difference between the measurements and
what you expect them to be) with disturbance (the difference between the
time derivative of the state and what you expect it to be). These
matrices have to be symmetric, have to be the right size, and also have
to satisfy the following conditions in order for the KF problem to have
a solution:

$$Q_{o} \geq 0 \qquad\qquad R_{o}>0 \qquad\qquad M_{o}\geq 0.$$

Just as
with the LQR problem, this notation means that $Q_{o}$ and $M_{o}$
are **positive semidefinite** and that $R_{o}$ is **positive definite**
([see
wikipedia](https://en.wikipedia.org/wiki/Positive-definite_matrix)).

By plugging in the expression for $n(t)$ that appears in the constraint,
this optimal control problem can be rewritten as

$$\begin{aligned}
\mathop{\mathrm{minimize}}_{x(t_{1}),d_{[t_{0},t_{1}]}}
&\qquad
(Cx(t_{0}) - y(t_{0}))^{T}M_{o}(Cx(t_{0}) - y(t_{0}))\\
&\qquad\qquad
+\int_{t_{0}}^{t_{1}} \left( (Cx(t) - y(t))^{T}Q_{o}(Cx(t) - y(t))+d(t)^{T}R_{o}d(t) \right) dt \\[1em]
\text{subject to}
&\qquad
\dot{x}(t) = Ax(t)+Bu(t)+d(t)\end{aligned}$$

It is an optimal control
problem, just like LQR --- if you define

$$\begin{aligned}
f(t,x,d) &= Ax+Bu(t)+d \\
g(t,x,d) &= (Cx-y(t))^{T}Q_{o}(Cx-y(t))+d^{T}R_{o}d(t) \\
h(t,x) &= (Cx-y(t))^{T}M_{o}(Cx-y(t))\end{aligned}$$

then you see that
this problem has the general form

$$\begin{aligned}
\underset{x(t_1),d_{[t_0,t_1]}}{\text{minimize}}
&\qquad
h(t_0,x(t_{0}))+
\int_{t_{0}}^{t_{1}} g(t,x(t),d(t)) dt \\
\text{subject to}
&\qquad
\frac{dx(t)}{dt}=f(t,x(t),d(t)).\end{aligned}$$

There are four
differences between this form and the one we saw when solving the LQR
problem:

-   The "input" in this problem is $u$, not $d$.

-   The "current time" is $t_{1}$ and not $t_{0}$.

-   The final state --- i.e., the state at the current time --- is *not*
    given. Indeed, the point here is to *choose* a final state
    $x(t_{1})$ that best explains $u(t)$ and $y(t)$.

-   The functions $f$, $g$, and $h$ vary with time (because they have
    parameters in them --- $u(t)$ and $y(t)$ --- that are functions of
    time).

Because of these four differences, the HJB equation for a problem of
this form is

$$0 = -\frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{d} \left\{ -\frac{\partial v(t,x)}{\partial x} f(t,x,d)+g(t,x,d) \right\}, \qquad v(t_{0},x) = h(t_{0}, x(t_{0})).$$

Note the change in sign of both the first term outside the minimum and
the first term inside the minimum --- this is because we are effectively
solving an optimal control problem in which time flows backward (from
the current time $t_{1}$ to the initial time $t_{0}$, instead of from
the current time $t_{0}$ to the final time $t_{1}$). It is possible to
derive this form of the HJB equation in exactly the same way as it was
done in the [notes on LQR](optimal-controllers#lqr).

## Solution to the problem

As usual, our first step is to find a function $v(t, x)$ that satisfies
the HJB equation. Here is that equation, with the functions $f$, $g$,
and $h$ filled in:

$$\begin{aligned}
0 &= -\frac{\partial v(t,x)}{\partial t} + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\frac{\partial v(t,x)}{\partial x} \left(Ax+Bu(t)+d\right) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y(t))^{T}Q_{o}(Cx-y(t))+d(t)^{T}R_{o}d(t) \biggr\} \\
v(t_{0},x) &= (Cx(t_{0})-y(t_{0}))^{T}M_{o}(Cx(t_{0})-y(t_{0})).
\end{aligned}$$

Expand the boundary condition:

$$\begin{aligned}
v(t_{0},x)
&= (Cx(t_{0})-y(t_{0}))^{T}M_{o}(Cx(t_{0})-y(t_{0})) \\
&= x(t_{0})^{T} C^{T}M_{o}C x(t_{0}) - 2 y(t_{0})^{T}M_{o}C^{T}x(t_{0}) + y(t_{0})^{T}M_{o}y(t_{0})
\end{aligned}$$

This function has the form

$$v(t, x) = x^{T}P(t)x +2 o(t)^{T} x + w(t)$$

for some symmetric matrix $P(t)$ and some other matrices $o(t)$ and
$w(t)$ that satisfy the following boundary conditions:

$$P(t_{0}) = C^{T}M_{o}C
\qquad
o(t_{0}) = -CM_{o}y(t_{0})
\qquad
w(t_{0}) = y(t_{0})^{T}M_{o}y(t_{0}).$$

Let's "guess" that this form of
$v$ is the solution we are looking for, and see if it satisfies the HJB
equation. Before proceeding, we need to compute the partial derivatives
of $v$:

$$\frac{\partial v}{\partial t} = x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w} \qquad\qquad \frac{\partial v}{\partial x} = 2x^{T}P + 2o^{T}$$

Here again --- just as for LQR --- we are applying [matrix
calculus](https://en.wikipedia.org/wiki/Matrix_calculus). Plug these
partial derivatives into HJB and we have

$$\begin{align*}
0 &=  -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\left(2x^{T}P+ 2o^{T}\right) (Ax+Bu+d) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y)^{T}Q_{o}(Cx-y)+d^{T}R_{o}d \biggr\} \\
\tag{1}
&= -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ d^{T}R_{o}d - 2(Px + o)^{T}d \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad -2 (x^{T} P+o^{T}) (Ax+Bu) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad + (Cx-y)^{T}Q_{o}(Cx-y) \biggr\}
\end{align*}$$

To evaluate the minimum, we apply the first-derivative
test (more matrix calculus!):

$$\begin{aligned}
0
&= \frac{\partial}{\partial d} \left( d^{T}R_{o}d - 2(Px + o)^{T}d -2 (x^{T} P+o^{T}) (Ax+Bu) + (Cx-y)^{T}Q_{o}(Cx-y) \right) \\
&= 2d^{T}R_{o}-2(Px+o)^{T}.
\end{aligned}$$

This equation is easily
solved:

$$\begin{align*}
\tag{2}
d = R^{-1}(Px+o).
\end{align*}$$

Plugging this back into (1), we have

$$\begin{aligned}
0
&=  -\left(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}\right) + \mathop{\mathrm{minimum}}_{d} \biggl\{ -\left(2x^{T}P+ 2o^{T}\right) (Ax+Bu+d) \\
&\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad\qquad +(Cx-y)^{T}Q_{o}(Cx-y)+d^{T}R_{o}d \biggr\} \\
&= -(x^{T} \dot{P} x + 2 \dot{o}^{T} x + \dot{w}) - (Px+o)^{T}R_{o}^{-1}(Px+o) \\
&\qquad\qquad -2 (x^{T} P+o^{T}) (Ax+Bu) + (Cx-y)^{T}Q_{o}(Cx-y) \\
&= x^{T}\left( -\dot{P} - PR_{o}^{-1}P -2PA + C^{T}Q_{o}C \right)x \\
&\qquad\qquad + 2x^{T} \left( -\dot{o} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \right) \\
&\qquad\qquad\qquad +\left( -\dot{w} - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y \right) \\
&= x^{T}\left( -\dot{P} - PR_{o}^{-1}P -PA - A^{T}P + C^{T}Q_{o}C \right)x \\
&\qquad\qquad + 2x^{T} \left( -\dot{o} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \right) \\
&\qquad\qquad\qquad +\left( -\dot{w} - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y \right)
\end{aligned}$$

where the last step is because

$$x^{T}(N+N^{T})x=2x^{T}Nx \text{ for any } N \text{ and } x.$$

In order
for this equation to be true for any $x$, it must be the case that

$$\begin{aligned}
\dot{P} &= -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C \\
\dot{o} &= - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \\
\dot{w} &= - o^{T}R_{o}^{-1}o -2o^{T}Bu + y^{T}Q_{o}y.
\end{aligned}$$

In summary, we have found that

$$v(t, x) = x^{T}P(t)x +2 o(t)^{T} x + w(t)$$

solves the HJB equation,
where $P$, $o$, and $w$ are found by integrating the above ODEs forward
in time, starting from

$$P(t_{0}) = C^{T}M_{o}C
\qquad
o(t_{0}) = -CM_{o}y(t_{0})
\qquad
w(t_{0}) = y(t_{0})^{T}M_{o}y(t_{0}).$$

The optimal choice of state
estimate at time $t$ is the choice of $x$ that minimizes $v(t, x)$, that
is, the solution to

$$\mathop{\mathrm{minimize}}_{x} \qquad x^{T}P(t)x +2 o(t)^{T} x + w(t).$$

We can find the solution to this problem by application of the first
derivative test, with some matrix calculus:

$$\begin{aligned}
0
&= \frac{\partial}{\partial x} \left( x^{T}Px +2 o^{T} x + w \right) \\
&= 2x^{T}P + 2o^{T},
\end{aligned}$$

 implying that

 $$x = -P^{-1}o.$$

Let's call this solution $\widehat{x}$. Note that we can, equivalently,
write

$$0 = P\widehat{x} + o.$$

Suppose we take the time derivative of
this expression, plugging in what we found earlier for $\dot{P}$ and
$\dot{o},$ as well as plugging in yet another version of this same
expression, $o = -P\widehat{x}$:

$$\begin{aligned}
0
&= \dot{P} \widehat{x} + P\dot{\widehat{x}} + \dot{o} \\
&= \left( -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C \right)\widehat{x} + P\dot{\widehat{x}} - PR_{o}^{-1}o - PBu -C^{T}Q_{o}y - A^{T}o \\
&= -PR_{o}^{-1}P\widehat{x}-P A\widehat{x}-A^{T}P\widehat{x} +C^{T}Q_{o}C \widehat{x} + P\dot{\widehat{x}} + PR_{o}^{-1}P\widehat{x} - PBu -C^{T}Q_{o}y + A^{T}P\widehat{x} \\
&= P\dot{\widehat{x}} -PA\widehat{x}-PBu + C^{T}Q_{o}(C\widehat{x} - y) \\
&= P \left( \dot{\widehat{x}} - A\widehat{x} - Bu + P^{-1}C^{T}Q_{o}(C\widehat{x} - y) \right).
\end{aligned}$$

For this equation to hold for any $P$, we must have

$$\dot{\widehat{x}} = A\widehat{x} + Bu - P^{-1}C^{T}Q_{o}(C\widehat{x} - y).$$

**Behold!** This is our expression for an optimal observer, if we define

$$L = P^{-1}C^{T}Q_{o}.$$

Finally, suppose we take the limit as
$t_{0}\rightarrow-\infty$, so assume an infinite horizon. It is a fact
that $P$ tends to a steady-state value, and so $L$ does as well. When
this happens, $\dot{P} = 0$, and so the steady-state value of $P$ is the
solution to the *algebraic* equation

$$0 = -PR_{o}^{-1}P-P A-A^{T}P +C^{T}Q_{o}C.$$

It is customary to write
these last two equations in a slightly different way. In particular,
suppose we pre- and post-multiply both sides of this last equation by
$P^{-1}$, and define

$$P_{o} = P^{-1}$$

Then, we have

$$L = P_{o}C^{T}Q_{o}$$

and

$$0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.$$

## Summary

An optimal observer --- a deterministic, infinite-horizon, continuous-time
Kalman Filter --- is given by

$$\dot{\widehat{x}} = A\widehat{x} + Bu - L(C\widehat{x} - y).$$

where

$$L = P_{o}C^{T}Q_{o}$$

and $P_{o}$ satisfies

$$0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.$$

## Comparison between LQR and KF (i.e., why you can use "LQR" in Python to compute an optimal observer)

An optimal controller is given by

$$u = -Kx$$

where

$$\begin{align*}
\tag{3}
K = R_{c}^{-1}B^{T}P_{c}
\end{align*}$$

and $P_{c}$ satisfies

$$\begin{align*}
\tag{4}
0 = P_{c}BR_{c}^{-1}B^{T}P_{c} - P_{c}A - A^{T}P_{c}-Q_{c}.
\end{align*}$$

An optimal observer is given by

$$\dot{\widehat{x}} = A\widehat{x} + Bu - L(C\widehat{x} - y).$$

where

$$\tag{5}
L = P_{o}C^{T}Q_{o}
$$

and $P_{o}$ satisfies

$$\tag{6}
0 = P_{o}C^{T}Q_{o}CP_{o}-AP_{o}-P_{o}A^{T} -R_{o}^{-1}.
$$

Take the transpose of (5) and --- remembering that $P_{o}$ and $Q_{o}$ are symmetric --- we get

$$\tag{7}
L^{T} = Q_{o}CP_{o}.
$$

Take the transpose of (6)
and --- remembering that $R_{o}$ is also symmetric --- we get

$$\tag{8}
0 = P_{o}C^{T}Q_{o}CP_{o}-P_{o}A^{T}-AP_{o} -R_{o}^{-1}.
$$

Compare (3) and (4) with (7) and (8). They are **exactly the same** if we make the
following replacements:

-   replace $K$ with $L^{T}$

-   replace $A$ with $A^{T}$

-   replace $B$ with $C^{T}$

-   replace $Q_{c}$ with $R_{o}^{-1}$

-   replace $R_{c}$ with $Q_{o}^{-1}$

**This** is the reason why

```python
L = lqr(A.T, C.T, linalg.inv(Ro), linalg.inv(Qo)).T
```

produces an optimal observer, just like

```python
K = lqr(A, B, Qc, Rc)
```

produces an optimal controller. **WWWWWOOOOOWWWWW!!!!!!!** ([See the code used to find the solution to an LQR problem with Python](optimal-controllers#lqr-code).)




