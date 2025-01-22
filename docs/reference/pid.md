---
title: PID
parent: Reference
nav_order: 1
---

# PID
{: .no_toc }

- TOC
{:toc }

## Overview

The **proportional-integral-derivative** or **PID** controller is a simple, widely-used, and often very effective way to make a dynamic system do what you want.

Suppose this dynamic system has exactly one real-valued **input** $u(t)$ and one real-valued **output** $y(t)$. Suppose our goal is to choose the input $u(t)$ so that the output $y(t)$ converges to a desired **reference** value $r$. Define the **error** as

$$
e(t) = y(t) - r.
$$

The PID controller is

$$
u(t) = - k_p e(t) - k_i \int_0^t e(\tau) d\tau - k_d \frac{d e(t)}{dt}
$$

for some choice of constant **gains** $k_p$, $k_i$, and $k_d$. The first term in this controller is "proportional feedback," the second term is "integral feedback," and the third term is "derivative feedback."

## How to compute the proportional feedback term

We assume the controller is given $r$ and has access to a measurement of $y(t)$, so computing the term

$$
- k_p e(t)
$$

is just arithmetic.

## How to compute the integral feedback term

To compute the term

$$
- k_i \int_0^t e(\tau) d\tau
$$

we need to find the integral

$$
\int_0^t e(\tau) d\tau
$$

at the current time $t$. In most cases, we cannot find this integral exactly even if we wanted to, because we do not know $y(t)$ --- and, therefore, $e(t)$ --- at *all* times $t \in \mathbb{R}$, but rather only at *sampled* times

$$
t \in \{ 0, \Delta t, 2\Delta t, 3\Delta t, \dotsc \}
$$

for some time step $\Delta t > 0$. For this reason, it is common to use the right-hand rectangle method of numerical integration:

$$
\int_0^t e(\tau) d\tau \approx \int_0^{t - \Delta t} e(\tau) d\tau + \Delta t \cdot e(t).
$$

Here is another way of thinking about this. Suppose we define a new variable

$$
z(t) = \int_0^t e(\tau) d\tau
$$

to contain the integral of the error. It is equivalent to say that $z(t)$ is described by the first-order ordinary differential equation

$$
\frac{d z(t)}{dt} = e(t)
$$

with the initial condition

$$
z(0) = 0.
$$

Then, it is common to use the **backward Euler method**

$$
z(t) \approx z(t - \Delta t) + \Delta t \cdot e(t)
$$

to compute an approximation to $z(t)$. Applying the backward Euler method to find $z(t)$ is exactly the same as applying the right-hand rectangle method to find the integral that $z(t)$ represents.

To implement the backward Euler method with code, you need to define a variable to store the prior value $z(t - \Delta t)$ of the error integral, and you need to update the value of this variable at each time step. You will have to initialize the error integral to something when you start your controller --- it is common to start with a value of zero.

## How to compute the derivative feedback term

To compute the term

$$
- k_d \frac{d e(t)}{dt}
$$

we need to find the derivative

$$
\frac{d e(t)}{dt}
$$

at the current time $t$. Just like for the integral term, we usually cannot find this derivative exactly. So, just like it is common to use the backward Euler method to find the integral of the error, it is common to use the **backward finite difference method** to find the derivative of the error:

$$
\frac{d e(t)}{dt} \approx \frac{e(t) - e(t - \Delta t)}{\Delta t}.
$$

To implement the backward finite difference method with code, you need to define a variable to store the prior value $e(t - \Delta t)$ of the error, and you need to update the value of this variable at each time step. Again, you will have to initialize this prior error to something when starting your controller --- it is common to start with a value of zero.


