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