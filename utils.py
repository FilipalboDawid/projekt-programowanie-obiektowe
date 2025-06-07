# utils.py

import numpy as np
import raylibpy as rl

def distance_vec3(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return (dx*dx + dy*dy + dz*dz) ** 0.5

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
