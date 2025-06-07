# utils.py

import numpy as np
import raylibpy as rl

def distance_vec3(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return (dx*dx + dy*dy + dz*dz) ** 0.5
def vec3_add(v1, v2):
    return rl.Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

def vec3_scale(v, scalar):
    return rl.Vector3(v.x * scalar, v.y * scalar, v.z * scalar)

def rotation_x(angle):
    return [
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ]

def rotation_y(angle):
    return [
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ]

def rotation_z(angle):
    return [
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ]

def apply_rotation(vec, mat):
    x = vec.x * mat[0][0] + vec.y * mat[0][1] + vec.z * mat[0][2]
    y = vec.x * mat[1][0] + vec.y * mat[1][1] + vec.z * mat[1][2]
    z = vec.x * mat[2][0] + vec.y * mat[2][1] + vec.z * mat[2][2]
    return rl.Vector3(x, y, z)

def dh_transform(a, alpha, d, theta):
    """
    Macierz transformacji DH (4x4)
    a: długość łącza wzdłuż x_{i}
    alpha: kąt między z_{i-1} a z_i wokół x_i
    d: przesunięcie wzdłuż z_{i-1}
    theta: kąt obrotu wokół z_{i-1}
    """
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    st = np.sin(theta)
    ct = np.cos(theta)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def rot_y(theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s,0, c, 0],
            [0, 0, 0, 1]
        ])

def dh_link(a, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, -s, 0, a*c],
            [s,  c, 0, a*s],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])

def to_vec3(arr):
        return rl.Vector3(arr[0], arr[1], arr[2])

def compute_inverse_kinematics_dh(x, y, z, L1, L2, L3):
    solutions = []

    # dwie opcje dla theta1
    theta1_options = [np.arctan2(-z, x)]

    for theta1 in theta1_options:
        r = np.sqrt(x**2 + z**2)
        s = y - L1

        D = (r**2 + s**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(D) > 1:
            continue  # brak rozwiązania w tym układzie

        # dwie opcje dla theta3
        for sign in [+1, -1]:
            theta3 = np.arctan2(sign * np.sqrt(1 - D**2), D)

            phi = np.arctan2(s, r)
            psi = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
            theta2 = phi - psi

            # Normalizacja kątów
            def normalize(angle):
                return np.arctan2(np.sin(angle), np.cos(angle))

            t1 = normalize(theta1)
            t2 = normalize(theta2)
            t3 = normalize(theta3)

            solutions.append((t2, t1, t3))

    return solutions

def filter_solutions_by_limits(solutions, current_angles):
    # Definicja zakresów kątów (w radianach)
    # (tutaj przykładowo, dopasuj do swojego robota)
    limits = [
        (np.deg2rad(40), np.deg2rad(120)),   # pitch
        (np.deg2rad(-175), np.deg2rad(175)), # yaw
        (np.deg2rad(-150), 0)                 # elbow
    ]

    def within_limits(angles):
        for angle, (low, high) in zip(angles, limits):
            # Normalizacja kąta do [-pi, pi]
            a = np.arctan2(np.sin(angle), np.cos(angle))
            if a < low or a > high:
                return False
        return True

    # Filtrujemy rozwiązania mieszczące się w zakresie
    valid_solutions = [sol for sol in solutions if within_limits(sol)]

    if not valid_solutions:
        return None

    # Wybierz rozwiązanie najbliższe obecnym kątom (minimalizując sumę różnic kątów)
    def distance(sol):
        return sum(abs(a - b) for a, b in zip(sol, current_angles))

    best_solution = min(valid_solutions, key=distance)
    return best_solution

    # Wybierz rozwiązanie najbliższe obecnym kątom
    def angle_distance(a1, a2):
        return sum(abs(x - y) for x, y in zip(a1, a2))

    best = min(valid_solutions, key=lambda s: angle_distance(s, current_angles))
    return best
