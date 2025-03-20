"""
Run as:

bokeh serve lqr-wheel.py
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

def integrand(t, A, B, Q, R, K, x0, t0):
    x = linalg.expm((A - B @ K) * (t - t0)) @ x0
    u = - K @ x
    return (x.T @ Q @ x + u.T @ R @ u).item()

def get_cost(A, B, Q, R, K, x0, t0, t1):
    cost, err = integrate.quad(integrand, t0, t1, args=(A, B, Q, R, K, x0, t0))
    return cost

def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R) @  B.T @ P
    return K, P

# Parameters that define the state-space model (wheel in gravity)
A = np.array([[0., 1.], [2., 0.]])
B = np.array([[0.], [1.]])

# Parameters that define the simulation
(t0, t1, dt) = (0., 3., 0.025)
nt = int(1 + np.ceil((t1 - t0) / dt))
t = np.linspace(t0, t1, nt)

# Widgets
styles = {"font-size": "150%"}
solve_lqr = Checkbox(label='Solve LQR', styles=styles)
k1_slider = Slider(title=r"$$k_1$$", value=10.0, start=-5.0, end=20.0, step=0.1, styles=styles)
k2_slider = Slider(title=r"$$k_2$$", value=10.0, start=-5.0, end=20.0, step=0.1, styles=styles)
q1_slider = Slider(title=r"$$\log(q_1)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
q2_slider = Slider(title=r"$$\log(q_2)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
r1_slider = Slider(title=r"$$\log(r_1)$$", value=0.0, start=-3.0, end=3.0, step=0.1, styles=styles)
x1i_slider = Slider(title=r"$$x_1(0)$$", value=1.0, start=-2.0, end=2.0, step=0.1, styles=styles)
x2i_slider = Slider(title=r"$$x_2(0)$$", value=0.0, start=-2.0, end=2.0, step=0.1, styles=styles)

# Plots
text_font_size = '14pt'
major_label_text_font_size = '14pt'
s_fig = figure(
    title=Title(text='CLOSED-LOOP EIGENVALUES', text_font_size=text_font_size),
    height=340,
    width=340,
    x_range=(-15, 5),
    y_range=(-10, 10),
)
s_plt = s_fig.scatter([0, 0], [0, 0], size=10, color='navy', alpha=0.5)
x1_fig = figure(title=Title(text=r'$$x_1(t)$$', text_font_size=text_font_size), height=220, width=400,
                x_range=(t0, t1), y_range=(-2, 2))
x2_fig = figure(title=Title(text='$$x_2(t)$$', text_font_size=text_font_size), height=220, width=400,
                x_range=(t0, t1), y_range=(-2, 2))
x1_plt = x1_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
x2_plt = x2_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
u1_fig = figure(title=Title(text='$$u_1(t)$$', text_font_size=text_font_size), height=220, width=400,
                x_range=(t0, t1), y_range=(-15, 15))
u1_plt = u1_fig.line(t, np.zeros_like(t), line_width=4, line_color='navy')
for f in [s_fig, x1_fig, x2_fig, u1_fig]:
    f.xaxis.major_label_text_font_size = major_label_text_font_size
    f.yaxis.major_label_text_font_size = major_label_text_font_size

# Labels
div_style = {'font-size': '150%'}
system_label = Div(
    text=r'''
        $$\begin{gather}
        \dot{x} = Ax + Bu  \qquad u = -Kx \\[1em]
        A = \begin{bmatrix} 0 & 1 \\ 2 & 0 \end{bmatrix}
        \qquad
        B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}
        \qquad
        K = \begin{bmatrix} k_1 & k_2 \end{bmatrix}
        \end{gather}$$
    ''',
    styles=div_style,
    align='center',
)
QRK_label = Div(text='', styles=div_style, align='center')
cost_label = Div(text='', styles=div_style, align='center')

def update_data_k(attrname, old, new):
    # Get the current slider values
    K = np.array([[k1_slider.value, k2_slider.value]])
    x0 = np.array([x1i_slider.value, x2i_slider.value])
    Q = np.diag([10.**q1_slider.value, 10.**q2_slider.value])
    R = np.diag([10.**r1_slider.value])

    QRK_label.text = fr'''$$\large \begin{{align*}}
                            Q &= \begin{{bmatrix}}
                                    {Q[0, 0]:.3f} & \mathmakebox[4em][c]{{0}} \\
                                    \mathmakebox[4em][c]{{0}} & {Q[1, 1]:.3f}
                                 \end{{bmatrix}} \\[0.5em]
                            R &= \begin{{bmatrix}} \mathmakebox[4em][c]{{ {R[0, 0]:.3f} }} \end{{bmatrix}} \\[0.5em]
                            K &= \begin{{bmatrix}} \mathmakebox[4em][c]{{ {K[0, 0]:.3f} }} &
                                    \mathmakebox[4em][c]{{ {K[0, 1]:.3f} }} \end{{bmatrix}}
                            \end{{align*}}$$'''
    
    # Find optimal K (if desired)
    if solve_lqr.active:
        k1_slider.disabled = True
        k2_slider.disabled = True
        K, P = lqr(A, B, Q, R)
        k1_slider.value = K[0, 0]
        k2_slider.value = K[0, 1]
    else:
        k1_slider.disabled = False
        k2_slider.disabled = False

    # Get the closed-loop eigenvalue
    s = np.linalg.eigvals(A - B @ K)
    
    # Get the state, input, and cost as functions of time
    x = []
    u = []
    for t1 in t:
        x1 = linalg.expm((A - B @ K) * (t1 - t0)) @ x0
        x.append(x1)
        u.append(- K @ x1)
    x = np.array(x)
    u = np.array(u)
    
    # Get the cost at infinity
    if solve_lqr.active:
        formula = r'& \mathmakebox[5em][l]{{\qquad = x(0)^T P x(0)}} \\[.5em]'
    else:
        formula = r'& \mathmakebox[5em][l]{{\phantom{{\qquad = x(0)^T P x(0)}}}} \\[.5em]'
    
    if (s.real < 0).all():
        cost_at_infinity = get_cost(A, B, Q, R, K, x0, t0, np.inf)
        cost = fr'& \qquad = {cost_at_infinity:.2f} \qquad \text{{(total cost)}}'
        if solve_lqr.active:
            assert(np.isclose(cost_at_infinity, (x0.T @ P @ x0).item()))
    else:
        cost_at_infinity = np.inf
        cost = r'& \qquad = \infty'
    cost_label.text = fr'''
        $$
        \large
        \begin{{align*}}
        & \int_0^\infty \left(x(t)^TQx(t) + u(t)^TRu(t)\right)dt \\
        {formula}
        {cost}
        \end{{align*}}
        $$
    '''
    
    s_plt.data_source.data = dict(x=s.real, y=s.imag)
    x1_plt.data_source.data = dict(x=t, y=x[:, 0])
    x2_plt.data_source.data = dict(x=t, y=x[:, 1])
    u1_plt.data_source.data = dict(x=t, y=u[:, 0])

# Initialize everything
update_data_k(None, None, None)

# Set a callback for each widget
for w in [k1_slider, k2_slider, x1i_slider, x2i_slider, q1_slider, q2_slider, r1_slider]:
    w.on_change('value', update_data_k)
for w in [solve_lqr]:
    w.on_change('active', update_data_k)

# Specify layout
curdoc().add_root(
    column(
        row(Spacer(), height=20),
        row(
            column(Spacer(), sizing_mode='stretch_width'),
            column(
                system_label,
                Spacer(height=10),
                solve_lqr,
                x1i_slider,
                x2i_slider,
                k1_slider,
                k2_slider,
                q1_slider,
                q2_slider,
                r1_slider,
                spacing=10,
            ),
            column(Spacer(), sizing_mode='stretch_width'),
            column(s_fig, Spacer(height=30), QRK_label, Spacer(height=30), cost_label),
            column(Spacer(), sizing_mode='stretch_width'),
            column(x1_fig, Spacer(height=20), x2_fig, Spacer(height=20), u1_fig),
            column(Spacer(), sizing_mode='stretch_width'),
            sizing_mode='stretch_both',
        ),
        sizing_mode='stretch_both',
        width=2000,
    )
)
curdoc().title = "LQR Demo"
