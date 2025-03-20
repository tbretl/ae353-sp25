"""
Run as:

bokeh serve error-vs-effort.py
"""

import numpy as np
from scipy import integrate
from scipy import linalg
from bokeh.io import curdoc
from bokeh.layouts import column, row, Spacer
from bokeh.models import Slider, Checkbox, Div, Title
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
    return K, P

# Widgets
styles = {"font-size": "150%"}
solve_lqr = Checkbox(label='Solve LQR', styles=styles)
show_costs = Checkbox(label='Show costs', styles=styles)
ks = Slider(title=r"$$k$$", value=12.0, start=0.0, end=20.0, step=0.1, styles=styles)
x0s = Slider(title=r"$$x_0$$", value=1.0, start=-2.0, end=2.0, step=0.1, styles=styles)
qs = Slider(title=r"$$\log(q)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
rs = Slider(title=r"$$\log(r)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
ps = Slider(title=r"$$p$$", value=-7.0, start=-15.0, end=5.0, step=0.1, styles=styles)

# Parameters that define the state-space model
(a, b) = (5., 1.)

# Parameters that define the simulation
(t0, t1, dt) = (0., 3., 0.05)
nt = int(1 + np.ceil((t1 - t0) / dt))
t = np.linspace(t0, t1, nt)
k_list = np.linspace(5.001, 20, 50)

# Plots
s_fig = figure(
    title=Title(text='CLOSED-LOOP EIGENVALUES', text_font_size='14pt'),
    height=340,
    width=340,
    x_range=(-15, 5),
    y_range=(-10, 10),
)
s_plt = s_fig.scatter([0], [0], size=10, color='navy', alpha=0.5)
x_fig = figure(title=Title(text='STATE', text_font_size='14pt'), height=220, width=400,
                x_range=(t0, t1), y_range=(-2, 2))
x_plt = x_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
u_fig = figure(title=Title(text='INPUT', text_font_size='14pt'), height=220, width=400,
                x_range=(t0, t1), y_range=(-15, 15))
u_plt = u_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
cost_fig = figure(title=Title(text='FINITE-HORIZON COST', text_font_size='14pt'), height=220, width=400,
                x_range=(t0, t1), y_range=(0, 40))
cost_plt = cost_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
cost_at_infinity_fig = figure(title=Title(text='INFINITE-HORIZON COST', text_font_size='14pt'), height=340, width=340,
                x_range=(0, 20), y_range=(0, 30))
costs_at_infinity_plt = cost_at_infinity_fig.line(
    k_list,
    np.zeros_like(k_list),
    line_width=4,
    line_color='cornflowerblue',
)
cost_at_infinity_plt = cost_at_infinity_fig.scatter([0], [0], size=15, color='coral', alpha=1.0)

# Labels
div_style = {'font-size': '150%'}
prob_lab = Div(
    text=r'$$\begin{gather}\dot{x} = ax + bu \\ a=5 \qquad b=1\end{gather}$$',
    styles=div_style,
    align='center',
)
x_lab = Div(text=r'$$x(t) = e^{(a - bk)t}x_0$$', styles=div_style, align='center')
u_lab = Div(text=r'$$u(t) = -kx(t)$$', styles=div_style, align='center')
cost_lab = Div(text=r'$$\int_{0}^{t} (qx(t)^2 + ru(t)^2) dt$$', styles=div_style, align='center')
q_lab = Div(text='', styles=div_style, align='center')
r_lab = Div(text='', styles=div_style, align='center')

def update_data_p(attrname, old, new):
    ks.value = 5. - ps.value

def update_data_k(attrname, old, new):
    # Get the current slider values
    k = ks.value
    ps.value = 5. - ks.value
    x0 = x0s.value
    q = 10.**qs.value
    r = 10.**rs.value

    q_lab.text = fr'q = {q:8.2f}'
    r_lab.text = fr'r = {r:8.2f}'

    # Find optimal k (if desired)
    if solve_lqr.active:
        ks.disabled = True
        K, P = lqr(np.array([[a]]), np.array([[b]]), np.array([[q]]), np.array([[r]]))
        k = K[0, 0]
        ks.value = k
    else:
        ks.disabled = False

    # Get the closed-loop eigenvalue
    s = complex(a - b * k)
    
    # Get the state, input, and cost as functions of time
    x = np.exp((a - b * k) * (t - t0)) * x0
    u = - k * x
    cost = np.array([get_cost(a, b, q, r, k, x0, t0, t1) for t1 in t])

    # Get the cost at infinity
    if (a - b * k) < 0:
        cost_at_infinity = get_cost(a, b, q, r, k, x0, t0, np.inf)
    else:
        cost_at_infinity = np.inf
    
    # Get the cost at infinity as a function of k (if desired)
    c_list = -np.ones_like(k_list)
    if show_costs.active:
        for i, k_cur in enumerate(k_list):
            c_list[i] = get_cost(a, b, q, r, k_cur, x0, t0, np.inf)
    
    s_plt.data_source.data = dict(x=[s.real], y=[s.imag])
    x_plt.data_source.data['y'] = x
    u_plt.data_source.data['y'] = u
    cost_plt.data_source.data['y'] = cost
    cost_at_infinity_plt.data_source.data['x'] = [k]
    cost_at_infinity_plt.data_source.data['y'] = [cost_at_infinity]
    costs_at_infinity_plt.data_source.data['x'] = k_list
    costs_at_infinity_plt.data_source.data['y'] = c_list

# Initialize everything
update_data_k(None, None, None)

# Set a callback for each widget
for w in [ks, x0s, qs, rs]:
    w.on_change('value', update_data_k)
for w in [solve_lqr, show_costs]:
    w.on_change('active', update_data_k)
for w in [ps]:
    w.on_change('value', update_data_p)

# Specify layout
curdoc().add_root(
    column(
        row(Spacer(), height=20),
        row(
            column(Spacer(), sizing_mode='stretch_width'),
            column(
                prob_lab,
                Spacer(height=10),
                solve_lqr,
                show_costs,
                x0s,
                ks,
                ps,
                qs,
                rs,
                q_lab,
                r_lab,
                spacing=10,
            ),
            column(Spacer(), sizing_mode='stretch_width'),
            column(s_fig, Spacer(height=20), cost_at_infinity_fig),
            column(Spacer(), sizing_mode='stretch_width'),
            column(row(x_fig, x_lab), Spacer(height=20), row(u_fig, u_lab), Spacer(height=20), row(cost_fig, cost_lab)),
            column(Spacer(), sizing_mode='stretch_width'),
            sizing_mode='stretch_both',
        ),
        sizing_mode='stretch_both',
        width=2000,
    )
)
curdoc().title = "LQR Demo"