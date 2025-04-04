"""
Run as:

bokeh serve optimal-observer.py
"""

import numpy as np
from scipy import integrate
from scipy import linalg
from scipy import signal
from bokeh.io import curdoc
from bokeh.layouts import column, row, Spacer
from bokeh.models import Slider, Checkbox, Div, Title, Button
from bokeh.plotting import figure

colors = {
    'illini_orange': '#FF5F05',
    'illini_blue': '#13294B',
    'altgeld': '#C84113',
    'alma_mater_dark': '#1E3877',
    'alma_mater_light': '#AFC7DB',
    'cloud': '#F8FAFC',
    'arches_blue_dark': '#009FD4',
    'arches_blue_light': '#D2EBF5',
    'heritage_orange_dark': '#B74D04',
    'heritage_orange_light': '#F5821E',
}

def integrand(t, a, b, q, r, k, x0, t0):
    x = np.exp((a - b * k) * (t - t0)) * x0
    u = - k * x
    return q * x**2 + r * u**2

def get_cost(a, b, q, r, k, x0, t0, t1):
    cost, err = integrate.quad(integrand, t0, t1, args=(a, b, q, r, k, x0, t0))
    return cost

def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R) @  B.T @ P
    return K

"""
Define a simulator class that can do two things:

* Run a simulator (`run_sim`) to generate the state x, input u, and output y, as well as disturbance d and sensor noise n.
* Run an observer (`run_obs`) to compute the state estimate xhat given the input u and output y.

Recall that state estimates are computed by integrating the following ODE:

xhatdot = A xhat + B u - L (C xhat - y)

If `simulator.use_euler=True`, then the simulator class will apply Euler integration to compute state estimates as we often do when implementing continuous-time observers in practice.

If `simulator.use_euler=False`, then the simulator class will integrate the observer equation "exactly" (e.g., with a Runge-Kutta solver).

Note that, in either case, the simulator class shifts the state estimate forward by one time step so that, when we plot the results, we emphasize the association between the output (i.e., the sensor measurements) and the state estimates.
"""

class Simulator:
    def __init__(self, t0=0., t1=5., dt=0.04, seed=None):
        self.t0 = t0
        self.t1 = t1
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        
        self.A = np.array([[0.]])
        self.B = np.array([[1.]])
        self.C = np.array([[1.]])
        self.D = np.array([[0.]])
        
        self.x0 = np.array([1.])
        self.u_avg = 0.
        self.d_std = 0.1
        self.n_std = 0.1
        
        self.xhat0 = np.array([1.])
        self.L = np.array([[1.]])
        self.use_euler = False # <-- careful when making this True, results won't look as you expect
        
        self.shift_time = True # <-- really shouldn't be here, since it has to do only with display
        
        self.t = np.linspace(t0, t1, 1 + np.ceil((t1 - t0) / dt).astype(int))
        self.nt = len(self.t)
    
    def run_sim(self):
        self.u = self.u_avg * np.ones((self.nt, 1))
        self.d = self.d_std * self.rng.standard_normal((self.nt, 1))
        self.n = self.n_std * self.rng.standard_normal((self.nt, 1))
        _, y, x = signal.lsim(
            (self.A, self.B, self.C, self.D),
            (self.u + self.d).flatten(),
            self.t,
            X0=self.x0,
            interp=False,
        )
        self.y = np.reshape(y, (-1, 1)) + self.n
        self.x = np.reshape(x, (-1, 1))
    
    def run_obs(self):
        if self.use_euler:
            xhat = np.empty((self.nt + 1, 1))
            xhat[0] = self.xhat0
            for i in range(self.nt):
                xhat[i + 1] = xhat[i] + self.dt * (self.A @ xhat[i] + self.B @ self.u[i] \
                                                   - self.L @ (self.C @ xhat[i] - self.y[i]))
            self.xhat = xhat[:-1, :]
            
        else:
            _, _, xhat = signal.lsim(
                (self.A - self.L @ self.C, np.hstack([self.B, self.L]), self.C, np.zeros((1, 2))),
                np.vstack([self.u.flatten(), self.y.flatten()]).T,
                self.t,
                X0=self.xhat0,
                interp=False,
            )
            self.xhat = np.reshape(xhat, (-1, 1))

# Create an instance of the simulator
simulator = Simulator()

# Widgets
styles = {"font-size": "150%"}
solve_lqr = Checkbox(label='Solve LQR', styles=styles, active=True)
use_euler = Checkbox(label='Use Euler', styles=styles, active=simulator.use_euler)
shift_time = Checkbox(label='Plot xhat(t + dt)', styles=styles, active=True)
button = Button(label='Simulate', styles=styles, button_type='success')

x0s = Slider(title=r"$$x(0)$$", value=simulator.x0.item(), start=-2.0, end=2.0, step=0.1, styles=styles)
uavgs = Slider(title=r"$$u(t)$$", value=simulator.u_avg, start=-1.0, end=1.0, step=0.1, styles=styles)
dstds = Slider(title=r"$$\text{std}(d)$$", value=simulator.d_std, start=0.0, end=1.0, step=0.01, styles=styles)
nstds = Slider(title=r"$$\text{std}(n)$$", value=simulator.n_std, start=0.0, end=1.0, step=0.01, styles=styles)
xhat0s = Slider(title=r"$$\widehat{x}(0)$$", value=simulator.xhat0.item(), start=-2.0, end=2.0, step=0.1, styles=styles)
ls = Slider(title=r"$$\ell$$", value=simulator.L.item(), start=-5.0, end=50.0, step=0.1, styles=styles)
qs = Slider(title=r"$$\log(q)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
rs = Slider(title=r"$$\log(r)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
ps = Slider(title=r"$$p$$", value=-7.0, start=-15.0, end=5.0, step=0.1, styles=styles)


# Plots
text_font_size = '14pt'
major_label_text_font_size = '14pt'
axis_label_text_font_size = '14pt'
x_fig = figure(
                #title=Title(text='STATE', text_font_size=text_font_size),
                height=480, width=640,
                x_range=(simulator.t0, simulator.t1), y_range=(0, 2), x_axis_label='time (seconds)')
x_plt_x = x_fig.line(
    simulator.t,
    np.zeros_like(simulator.t),
    line_width=5,
    line_color='navy',
    legend_label='state (x)',
)
x_plt_xhat = x_fig.line(
    simulator.t[0:-1],
    np.zeros_like(simulator.t[0:-1]),
    line_width=4,
    line_color='coral',
    line_dash=[10, 1],
    legend_label='state estimate (xhat)',
)
x_plt_y = x_fig.scatter(
    simulator.t,
    np.zeros_like(simulator.t),
    color='green',
    size=6,
    legend_label='output (y)',
)
for f in [x_fig]:
    f.xaxis.major_label_text_font_size = major_label_text_font_size
    f.yaxis.major_label_text_font_size = major_label_text_font_size
    f.xaxis.axis_label_text_font_size = axis_label_text_font_size
    f.yaxis.axis_label_text_font_size = axis_label_text_font_size
    f.legend.label_text_font_size = '14pt'



# Labels

def mat_to_str(name, M):
    return fr'{name}=\begin{{bmatrix}}{int(M.item())}\end{{bmatrix}}'

def sym_mat_to_str(name, M):
    return fr'{name}=\begin{{bmatrix}}{M}\end{{bmatrix}}'

div_style = {'font-size': '150%'}
q_lab = Div(text='', styles=div_style, align='center')
r_lab = Div(text='', styles=div_style, align='center')
model_lab = Div(
    text=r'$$\begin{align*}\dot{x} &= Ax+Bu +d \\ y &= Cx+n \end{align*}$$',
    styles=div_style,
    align='center',
)
ABC_lab = Div(
    text=fr'$${mat_to_str("A", simulator.A)} \qquad {mat_to_str("B", simulator.B)} \qquad {mat_to_str("C", simulator.C)}$$',
    styles=div_style,
    align='center',
)
obs_lab = Div(
    text=r'$$\dot{\widehat{x}} = A\widehat{x}+Bu-L(C\widehat{x}-y)$$',
    styles=div_style,
    align='center',
)
QRL_lab = Div(
    text=fr'$${sym_mat_to_str(r"Q_o", r"q")} \qquad {sym_mat_to_str(r"R_o", r"r")} \qquad {sym_mat_to_str(r"L", r"\ell")}$$',
    styles=div_style,
    align='center',
)

def run_obs():
    simulator.run_obs()
    if simulator.shift_time:
        x_plt_xhat.data_source.data['y'] = simulator.xhat.flatten()[1:]
    else:
        x_plt_xhat.data_source.data['y'] = simulator.xhat.flatten()[:-1]
    
def run_sim():
    simulator.run_sim()
    x_plt_x.data_source.data['y'] = simulator.x.flatten()
    x_plt_y.data_source.data['y'] = simulator.y.flatten()
    run_obs()

def update_sim_params(attrname, old, new):
    simulator.x0 = np.array([x0s.value])
    simulator.u_avg = uavgs.value
    simulator.d_std = dstds.value
    simulator.n_std = nstds.value

def update_obs_params(attrname, old, new):
    l = ls.value
    xhat0 = xhat0s.value
    q = 10.**qs.value
    r = 10.**rs.value
    q_lab.text = fr'q = {q:8.3f}'
    r_lab.text = fr'r = {r:8.3f}'
    if solve_lqr.active:
        ls.disabled = True
        Qo = np.array([[q]])
        Ro = np.array([[r]])
        L = lqr(simulator.A.T, simulator.C.T, linalg.inv(Ro), linalg.inv(Qo)).T
        l = L[0, 0]
        ls.value = l
    else:
        L = np.array([[l]])
        ls.disabled = False
    simulator.L = L
    simulator.xhat0 = np.array([[xhat0]])
    simulator.use_euler = use_euler.active
    simulator.shift_time = shift_time.active
    run_obs()

# Set a callback for each widget
button.on_click(run_sim)
for w in [x0s, uavgs, dstds, nstds]:
    w.on_change('value', update_sim_params)
for w in [ls, xhat0s, qs, rs]:
    w.on_change('value', update_obs_params)
for w in [solve_lqr, use_euler, shift_time]:
    w.on_change('active', update_obs_params)

# Run simulator and observer for the first time
run_sim()
update_obs_params(None, None, None)
update_sim_params(None, None, None)

# Specify layout
curdoc().add_root(
    column(
        row(Spacer(), height=20),
        row(
            column(Spacer(width=50)),
            column(
                button,
                x0s,
                uavgs,
                dstds,
                nstds,
                model_lab,
                ABC_lab,
                spacing=10,
            ),
            column(Spacer(width=50)),
            column(
                column(
                    solve_lqr,
                    use_euler,
                    shift_time,
                    spacing=0,
                ),
                ls,
                xhat0s,
                qs,
                rs,
                q_lab,
                r_lab,
                obs_lab,
                QRL_lab,
                spacing=10,
            ),
            column(Spacer(width=50)),
            column(x_fig, sizing_mode='stretch_width'),
            sizing_mode='stretch_both',
        ),
        sizing_mode='stretch_both',
        width=2000,
    )
)
curdoc().title = "Optimal Observer Demo"