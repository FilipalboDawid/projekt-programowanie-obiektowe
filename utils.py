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