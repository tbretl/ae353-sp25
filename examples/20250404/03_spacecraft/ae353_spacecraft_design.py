import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from pathlib import Path
import meshcat
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import json

# Minimum distance between the center of wheel pairs
min_dist_rw_to_rw = np.sqrt(0.5**2 + 0.1**2) + np.sqrt(0.5**2 + 0.1**2) + 0.001

# Minimum distance between center of each wheel and center of scope
min_dist_rw_to_scope = np.sqrt(0.5**2 + 0.1**2) + np.sqrt(0.8**2 + 0.1**2) + 0.001

# Minimum distance between center of each wheel and center of hatch
min_dist_rw_to_hatch = np.sqrt(0.5**2 + 0.1**2) + np.sqrt(0.3**2 + 0.1**2) + 0.001

# Distance between center of spacecraft and center of each wheel
DISTANCE_TO_WHEELS = 2.2

# Minimum distance between the center of any two stars in the scope
min_dist_star_to_star = 0.2

# Scope radius
scope_radius = 0.8 / 2.1


def transform_inertia_matrix(m, J_inB_aboutB, R_inC_ofB, p_inC_ofB):
    """
    INPUT
    
    m is the mass of the rigid body
    J_inB_aboutB is the inertia matrix of the rigid body about the origin of B in frame B
    R_inC_ofB is the orientation of frame B in frame C
    p_inC_ofB is the position of frame B in frame C

    OUTPUT

    J_inC_aboutC is the inertia matrix of the rigid body about the origin of C in frame C
    """
    
    p_inB_ofC = -R_inC_ofB.T @ p_inC_ofB
    R_inB_ofC = R_inC_ofB.T

    J_inB_aboutC = J_inB_aboutB + m * ((np.inner(p_inB_ofC, p_inB_ofC) * np.eye(3) - np.outer(p_inB_ofC, p_inB_ofC)))
    J_inC_aboutC = R_inC_ofB @ J_inB_aboutC @ R_inB_ofC

    return J_inC_aboutC

def get_spacecraft_inertia(m_s=5., r_s=2.1):
    J = (2. / 5.) * m_s * r_s**2
    return m_s, np.diag([J, J, J])

def get_wheel_inertias(wheels, m_w=1., r_w=0.5, h_w=0.2):
    # In wheel frame about wheel center (each wheel alone)
    Jx = (1. / 12.) * m_w * (3 * r_w**2 + h_w**2)
    Jz = (1. / 2.) * m_w * r_w**2
    for w in wheels:
        w['m'] = m_w
        w['J'] = np.diag([Jx, Jx, Jz])
    
    # In body frame about body center (all wheels together)
    m_wheels = 0.
    c_wheels = np.zeros(3)
    J_wheels = np.zeros((3, 3))
    for w in wheels:
        m_wheels += w['m']
        c_wheels += w['xyz'] * w['m']
        J_wheels += transform_inertia_matrix(w['m'], w['J'], w['R_inC_ofB'], w['p_inC_ofB'])
    c_wheels /= m_wheels

    return m_wheels, J_wheels, c_wheels

def get_masses(m_wheels, J_wheels, c_wheels):
    masses = []
    J = J_wheels.copy()
    m = m_wheels
    
    # Move center of mass to body center
    mass = {
        'm': m_wheels,
        'p': -c_wheels,
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)

    # Eliminate off-diagonal elements of inertia matrix
    # - xy
    m_mass = 1.
    r_mass = np.sqrt(np.abs(J[0, 1]) / (2 * m_mass))
    mass = {
        'm': m_mass,
        'p': np.array([np.sign(J[0, 1]) * r_mass, r_mass, 0.])
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    mass = {
        'm': m_mass,
        'p': - np.array([np.sign(J[0, 1]) * r_mass, r_mass, 0.])
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    # - xz
    m_mass = 1.
    r_mass = np.sqrt(np.abs(J[0, 2]) / (2 * m_mass))
    mass = {
        'm': m_mass,
        'p': np.array([np.sign(J[0, 2]) * r_mass, 0., r_mass]) #<-- FIXME
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    mass = {
        'm': m_mass,
        'p': - np.array([np.sign(J[0, 2]) * r_mass, 0., r_mass]) #<-- FIXME
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    # - yz
    m_mass = 1.
    r_mass = np.sqrt(np.abs(J[1, 2]) / (2 * m_mass))
    mass = {
        'm': m_mass,
        'p': np.array([0., np.sign(J[1, 2]) * r_mass, r_mass])
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    mass = {
        'm': m_mass,
        'p': - np.array([0., np.sign(J[1, 2]) * r_mass, r_mass])
    }
    m += mass['m']
    J += transform_inertia_matrix(mass['m'], np.zeros((3, 3)), np.eye(3), mass['p'])
    masses.append(mass)
    
    return m, J, masses

def get_mass_properties(wheels):
    # Mass and inertia of spacecraft alone
    m_spacecraft, J_spacecraft = get_spacecraft_inertia()

    # Mass and inertia of wheels alone
    m_wheels, J_wheels, c_wheels = get_wheel_inertias(wheels)
    
    # Place masses to (1) move COM back to body center and (2) eliminate
    # off-diagonal elements of the inertia matrix, i.e., align principal
    # axes with body axes
    m_with_wheels, J_with_wheels, masses = get_masses(m_wheels, J_wheels, c_wheels)

    # Get total mass and inertia
    m_with_wheels += m_spacecraft
    J_with_wheels += J_spacecraft
    
    return m_spacecraft, J_spacecraft, m_with_wheels, J_with_wheels, masses

def get_wheel_frames(wheels, r=DISTANCE_TO_WHEELS):
    for w in wheels:
        alpha = w['alpha']
        delta = w['delta']
        w['xyz'] = r * np.array([np.cos(alpha) * np.cos(delta), np.sin(alpha) * np.cos(delta), np.sin(delta)])
        w['rpy'] = [(np.pi / 2) - delta, 0., (np.pi / 2) + alpha]
        roll, pitch, yaw = w['rpy']
        w['R_inC_ofB'] = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=False).as_matrix()
        w['p_inC_ofB'] = w['xyz']

def add_material(robot, name, rgba):
    mat = ET.SubElement(robot, 'material', attrib={'name': name})
    col = ET.SubElement(mat, 'color', attrib={'rgba': ' '.join(str(v) for v in rgba)})

def add_link(robot, name, stl, scale, mass, inertia, material, concave=True):
    link = ET.SubElement(robot, 'link', attrib={'name': name, 'concave': f'{"yes" if concave else "no"}'})
    vis = ET.SubElement(link, 'visual')
    geo = ET.SubElement(vis, 'geometry')
    ET.SubElement(
        geo,
        'mesh',
        attrib={
            'filename': stl,
            'scale': ' '.join(str(v) for v in scale),
        },
    )
    ET.SubElement(vis, 'material', attrib={'name': material})
    col = ET.SubElement(link, 'collision', attrib={'concave': f'{"yes" if concave else "no"}'})
    geo = ET.SubElement(col, 'geometry')
    ET.SubElement(
        geo,
        'mesh',
        attrib={
            'filename': stl,
            'scale': ' '.join(str(v) for v in scale),
        },
    )
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', attrib={
        'rpy': ' '.join(str(v) for v in [0., 0., 0.]),
        'xyz': ' '.join(str(v) for v in [0., 0., 0.]),
    })
    ET.SubElement(inertial, 'mass', attrib={'value': f'{mass}'})
    ET.SubElement(inertial, 'inertia', attrib={
        'ixx': f'{inertia[0]:.8f}',
        'ixy': f'{inertia[1]:.8f}',
        'ixz': f'{inertia[2]:.8f}',
        'iyy': f'{inertia[3]:.8f}',
        'iyz': f'{inertia[4]:.8f}',
        'izz': f'{inertia[5]:.8f}',
    })

def add_point_mass_link(robot, name, mass):
    link = ET.SubElement(robot, 'link', attrib={'name': name})
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', attrib={
        'rpy': ' '.join(str(v) for v in [0., 0., 0.]),
        'xyz': ' '.join(str(v) for v in [0., 0., 0.]),
    })
    ET.SubElement(inertial, 'mass', attrib={'value': f'{mass}'})
    ET.SubElement(inertial, 'inertia', attrib={
        'ixx': '0',
        'ixy': '0',
        'ixz': '0',
        'iyy': '0',
        'iyz': '0',
        'izz': '0',
    })

def add_joint(robot, props):
    joint = ET.SubElement(robot, 'joint', attrib={
        'name': f'{props["parent"]}_to_{props["child"]}',
        'type': props['type'],
    })
    ET.SubElement(joint, 'parent', attrib={'link': props['parent']})
    ET.SubElement(joint, 'child', attrib={'link': props['child']})
    ET.SubElement(joint, 'origin', attrib={
        'xyz': ' '.join(f'{v:.8f}' for v in props['origin']['xyz']),
        'rpy': ' '.join(f'{v:.8f}' for v in props['origin']['rpy']),
    })
    if props['axis'] is not None:
        ET.SubElement(joint, 'axis', attrib={'xyz': ' '.join(f'{v:.8f}' for v in props['axis']['xyz'])})
    if props['limit'] is not None:
        ET.SubElement(joint, 'limit', attrib={
            'effort': f'{props["limit"]["effort"]}',
            'velocity': f'{props["limit"]["velocity"]}',
        })

def wheels_are_valid(wheels):
    valid = True

    if len(wheels) != 4:
        raise Exception(f'There must be exactly four wheels (you defined {len(wheels)}).')

    # Find body frame of each wheel
    get_wheel_frames(wheels)
    
    # Find xyz of scope
    scope_xyz = DISTANCE_TO_WHEELS * np.array([1., 0., 0.])

    # Find xyz of hatch
    hatch_xyz = DISTANCE_TO_WHEELS * np.array([0., 0., 1.])
    
    # Check if wheels are too close to scope
    for i, w in enumerate(wheels):
        if np.linalg.norm(w['xyz'] - scope_xyz) <= min_dist_rw_to_scope:
            print(f'WARNING: RW{i + 1} is too close to scope')
            valid = False

    # Check if wheels are too close to hatch
    for i, w in enumerate(wheels):
        if np.linalg.norm(w['xyz'] - hatch_xyz) <= min_dist_rw_to_hatch:
            print(f'WARNING: RW{i + 1} is too close to hatch')
            valid = False
    
    # Check if wheels are too close to each other
    for i, w_i in enumerate(wheels):
        for j, w_j in enumerate(wheels):
            if j <= i:
                continue
            if np.linalg.norm(w_i['xyz'] - w_j['xyz']) <= min_dist_rw_to_rw:
                print(f'WARNING: RW{i + 1} is too close to RW{j + 1}')
                valid = False
    
    return valid

def add_wheel_joints(robot, wheels):
    # Add wheels to URDF
    for i, w in enumerate(wheels):
        add_joint(robot, {
            'type': 'continuous',
            'parent': 'bus',
            'child': f'wheel_{i + 1}',
            'origin': {'xyz': w['xyz'], 'rpy': w['rpy']},
            'axis': {'xyz': [0., 0., 1.]},
            'limit': {'effort': 1000., 'velocity': 1000.},
        })

def add_mass_joints(robot, masses):
    # Add masses to URDF
    for i, mass in enumerate(masses):
        add_joint(robot, {
            'type': 'fixed',
            'parent': 'bus',
            'child': f'mass_{i + 1}',
            'origin': {'xyz': mass['p'], 'rpy': [0., 0., 0.]},
            'axis': None,
            'limit': None,
        })

def get_inertial_parameters(J):
    assert(np.allclose(J, J.T))
    ixx = J[0, 0]
    iyy = J[1, 1]
    izz = J[2, 2]
    ixy = J[0, 1]
    iyz = J[1, 2]
    ixz = J[0, 2]
    return [ixx, ixy, ixz, iyy, iyz, izz]

def create_spacecraft(wheels, urdf='spacecraft.urdf'):
    # Geometry
    if not wheels_are_valid(wheels):
        raise Exception('Invalid placement of reaction wheels')
    
    # Inertia
    m_spacecraft, J_spacecraft, m_with_wheels, J_with_wheels, masses = get_mass_properties(wheels)
    assert(np.isclose(J_with_wheels[0, 1], 0.))
    assert(np.isclose(J_with_wheels[0, 2], 0.))
    assert(np.isclose(J_with_wheels[1, 2], 0.))
    assert(np.isclose(J_spacecraft[0, 1], 0.))
    assert(np.isclose(J_spacecraft[0, 2], 0.))
    assert(np.isclose(J_spacecraft[1, 2], 0.))
    
    robot = ET.Element('robot', attrib={'name': 'spacecraft'})
    add_material(robot, 'industrial-blue', [0.11372549019607843, 0.34509803921568627, 0.6549019607843137, 1.])
    add_material(robot, 'arches-blue', [0., 0.6235294117647059, 0.8313725490196079, 1.])
    add_material(robot, 'heritage-orange', [0.96078431, 0.50980392, 0.11764706, 1.])

    add_link(
        robot,
        'bus',
        'spacecraft.stl',
        [1., 1., 1.],
        m_spacecraft,
        get_inertial_parameters(J_spacecraft),
        'industrial-blue',
    )

    for (i, w) in enumerate(wheels):
        add_link(
            robot,
            f'wheel_{i + 1}',
            f'rw{i + 1}.stl',
            [0.5, 0.5, 0.2],
            w['m'],
            get_inertial_parameters(w['J']),
            'heritage-orange',
        )
    
    for (i, mass) in enumerate(masses):
        add_point_mass_link(
            robot,
            f'mass_{i + 1}',
            mass['m'],
        )

    add_wheel_joints(robot, wheels)
    add_mass_joints(robot, masses)

    xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    with open(Path(f'./urdf/{urdf}'), 'w') as f:
        f.write(xmlstr)
    
    return m_with_wheels, J_with_wheels.round(decimals=8)

def convert_color(rgba):
    color = int(rgba[0] * 255) * 256**2 + int(rgba[1] * 255) * 256 + int(rgba[2] * 255)
    opacity = rgba[3]
    transparent = opacity != 1.0
    return {
        'color': color,
        'opacity': opacity,
        'transparent': transparent,
    }

def create_visualizer():
    # Create visualizer
    vis = meshcat.Visualizer()

    # Create spacecraft
    color = convert_color([0.11372549019607843, 0.34509803921568627, 0.6549019607843137, 1.])
    vis['spacecraft'].set_object(
        meshcat.geometry.StlMeshGeometry.from_file(Path('./urdf/spacecraft.stl')),
        meshcat.geometry.MeshPhongMaterial(
            color=color['color'],
            transparent=color['transparent'],
            opacity=color['opacity'],
            reflectivity=0.8,
        )
    )

    # Create wheels
    color = convert_color([0.96078431, 0.50980392, 0.11764706, 1.])
    for i in range(4):
        vis[f'rw{i + 1}'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(Path(f'./urdf/rw{i + 1}.stl')),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )
    
    # Set camera view
    vis['/Cameras/default'].set_transform(
        meshcat.transformations.compose_matrix(
            angles=[
                0.,
                np.deg2rad(-30.),
                np.deg2rad(60. - 180.),
            ],
            translate=[0., 0., 0.],
        )
    )
    vis['/Cameras/default/rotated/<object>'].set_property(
        'position', [5., 0., 0.],
    )
    vis['/Cameras/default/rotated/<object>'].set_property(
        'fov', 90,
    )
    
    # Return visualizer
    return vis

def show_wheels(vis, wheels):
    if not wheels_are_valid(wheels):
        print('WARNING: Invalid placement of reaction wheels')
    
    for i, w in enumerate(wheels):
        S = np.diag(np.concatenate(([0.5, 0.5, 0.2], [1.0])))
        T = meshcat.transformations.euler_matrix(*w['rpy'])
        roll, pitch, yaw = w['rpy']
        assert(np.allclose(T[0:3, 0:3], Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=False).as_matrix()))
        T[:3, 3] = np.array(w['xyz'])[:3]
        vis[f'rw{i + 1}'].set_transform(T @ S)

def project_star(alpha, delta, scope_radius):
    x = np.cos(alpha) * np.cos(delta)
    y = np.sin(alpha) * np.cos(delta)
    z = np.sin(delta)
    return (1 / scope_radius) * (y / x), (1 / scope_radius) * (z / x)

def plot_delta(ax, delta, scope_radius, y_lim=1.1):
    alpha_lim = np.atan(y_lim * scope_radius)
    y_tick, z_tick = project_star(alpha_lim, delta, scope_radius)
    y = []
    z = []
    for alpha in np.linspace(-0.4, 0.4, 100):
        y_star, z_star = project_star(alpha, delta, scope_radius)
        y.append(y_star)
        z.append(z_star)
    ax.plot(y, z, color='w', linewidth=0.5)
    return z_tick

def plot_alpha(ax, alpha, scope_radius, z_lim=-1.1):
    delta_lim = np.acos(np.tan(alpha) / (scope_radius * z_lim))
    y_tick, z_tick = project_star(alpha, delta_lim, scope_radius)
    y = []
    z = []
    for delta in np.linspace(-0.4, 0.4, 100):
        y_star, z_star = project_star(alpha, delta, scope_radius)
        y.append(y_star)
        z.append(z_star)
    ax.plot(y, z, color='w', linewidth=0.5)
    return y_tick

def stars_are_valid(stars):
    valid = True
    
    if len(stars) > 10:
        raise Exception(f'There must be at most ten stars (you defined {len(stars)}).')

    # Check if stars are too close to each other
    for i, s_i in enumerate(stars):
        for j, s_j in enumerate(stars):
            if j <= i:
                continue
            y_i, z_i = project_star(s_i['alpha'], s_i['delta'], scope_radius)
            y_j, z_j = project_star(s_j['alpha'], s_j['delta'], scope_radius)
            if np.linalg.norm(np.array([y_i - y_j, z_i - z_j])) <= min_dist_star_to_star:
                print(f'WARNING: STAR {i + 1} is too close to STAR {j + 1}')
                valid = False
    
    # Check if stars are outside scope
    for i, s_i in enumerate(stars):
        y_i, z_i = project_star(s_i['alpha'], s_i['delta'], scope_radius)
        if y_i**2 + z_i**2 > 1.:
            print(f'WARNING: STAR {i + 1} is out of view')
            valid = False

    return valid

def show_stars_on_axis(stars, ax, show_alpha_delta_scale):
    dark_yellow = [1.0, 0.8196078431, 0.1450980392]
    industrial_blue = [0.11372549019607843, 0.34509803921568627, 0.6549019607843137]
    
    for i, star in enumerate(stars):
        y_star, z_star = project_star(star['alpha'], star['delta'], scope_radius)
        ax.add_patch(
            Circle(
                [y_star, z_star],
                0.09,
                edgecolor='none',
                facecolor=dark_yellow,
                zorder=3,
            )
        )
        ax.text(y_star, z_star, f'{i + 1}', horizontalalignment='center', verticalalignment='center', weight='bold')
        

    ax.add_patch(Circle([0., 0.], 1., edgecolor='none', facecolor='k'))

    if show_alpha_delta_scale:
        z_ticks = []
        deltas = np.linspace(-0.3, 0.3, 7)
        for delta in deltas:
            z_ticks.append(plot_delta(ax, delta, scope_radius))
        ax.set_yticks(z_ticks)
        ax.set_yticklabels([fr'${delta:.1f}$' for delta in deltas])
        ax.set_ylabel(r'$\delta$', fontsize=12, rotation=0)

        y_ticks = []
        alphas = np.linspace(-0.3, 0.3, 7)
        for alpha in alphas:
            y_ticks.append(plot_alpha(ax, alpha, scope_radius))
        ax.set_xticks(y_ticks)
        ax.set_xticklabels([fr'${alpha:.1f}$' for alpha in alphas])
        ax.set_xlabel(r'$\alpha$', fontsize=12, rotation=0)
    else:
        ax.set_xticks(np.linspace(-1, 1, 9))
        ax.set_yticks(np.linspace(-1, 1, 9))
        ax.set_xlabel(r'$y_\text{star}$', fontsize=12, rotation=0)
        ax.set_ylabel(r'$z_\text{star}$', fontsize=12, rotation=90)
        ax.grid()

    ax.set_xlim([-1.1, 1.1])
    ax.set_ylim([-1.1, 1.1])
    ax.set_aspect('equal')
    ax.invert_xaxis()
    ax.set_facecolor(industrial_blue)

def show_stars(stars):
    if not stars_are_valid(stars):
        print('WARNING: Invalid placement of stars')

    fig, (ax_ad, ax_yz) = plt.subplots(1, 2, figsize=(9, 4))
    show_stars_on_axis(stars, ax_ad, True)
    show_stars_on_axis(stars, ax_yz, False)
    fig.tight_layout()

    

def create_stars(stars, filename='stars.json'):
    if not stars_are_valid(stars):
        raise Exception('Invalid placement of stars')
    
    with open(Path(f'./urdf/{filename}'), 'w') as f:
        json.dump(stars, f, indent=4)