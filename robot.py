import raylibpy as rl
import math

# Pomocnicze funkcje wektorowe
def vec3_add(v1, v2):
    return rl.Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

def vec3_scale(v, scalar):
    return rl.Vector3(v.x * scalar, v.y * scalar, v.z * scalar)

def rotation_x(angle):
    return [
        [1, 0, 0],
        [0, math.cos(angle), -math.sin(angle)],
        [0, math.sin(angle), math.cos(angle)]
    ]

def rotation_y(angle):
    return [
        [math.cos(angle), 0, math.sin(angle)],
        [0, 1, 0],
        [-math.sin(angle), 0, math.cos(angle)]
    ]

def rotation_z(angle):
    return [
        [math.cos(angle), -math.sin(angle), 0],
        [math.sin(angle), math.cos(angle), 0],
        [0, 0, 1]
    ]

def apply_rotation(vec, mat):
    x = vec.x * mat[0][0] + vec.y * mat[0][1] + vec.z * mat[0][2]
    y = vec.x * mat[1][0] + vec.y * mat[1][1] + vec.z * mat[1][2]
    z = vec.x * mat[2][0] + vec.y * mat[2][1] + vec.z * mat[2][2]
    return rl.Vector3(x, y, z)

class RobotArm:
    def __init__(self):
        self.joint_angles = [0.0, 0.0, 0.0]  # shoulder_pitch, shoulder_yaw, elbow
        self.segment_length = 2.0
        self.grabbing = False
        self.grabbed_object = None

    def handle_input(self):
        if rl.is_key_down(rl.KEY_W): self.joint_angles[0] += 0.01  # Shoulder Pitch (X)
        if rl.is_key_down(rl.KEY_S): self.joint_angles[0] -= 0.01
        if rl.is_key_down(rl.KEY_A): self.joint_angles[1] += 0.01  # Shoulder Yaw (Y)
        if rl.is_key_down(rl.KEY_D): self.joint_angles[1] -= 0.01
        if rl.is_key_down(rl.KEY_UP): self.joint_angles[2] += 0.01  # Elbow (X)
        if rl.is_key_down(rl.KEY_DOWN): self.joint_angles[2] -= 0.01

    def draw(self):
        base = rl.Vector3(0, 0.5, 0)
        up = rl.Vector3(0, 1, 0)

        # Ramię 1: Shoulder Pitch + Yaw
        rot_pitch = rotation_x(self.joint_angles[0])
        rot_yaw = rotation_y(self.joint_angles[1])
        dir1 = apply_rotation(up, rot_pitch)
        dir1 = apply_rotation(dir1, rot_yaw)
        joint1 = vec3_add(base, vec3_scale(dir1, self.segment_length))

        # Ramię 2: Elbow
        rot_elbow = rotation_x(self.joint_angles[2])
        dir2 = apply_rotation(up, rot_elbow)
        dir2 = apply_rotation(dir2, rot_pitch)
        dir2 = apply_rotation(dir2, rot_yaw)
        joint2 = vec3_add(joint1, vec3_scale(dir2, self.segment_length))

        rl.draw_cylinder_ex(base, joint1, 0.1, 0.1, 10, rl.GRAY)  # Rysowanie ramienia 1
        rl.draw_cylinder_ex(joint1, joint2, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_sphere(joint1, 0.1, rl.DARKGRAY)  # Rysowanie stawów
        rl.draw_sphere(base, 0.1, rl.DARKGRAY)
        rl.draw_cube(joint2, 0.2, 0.2, 0.2, rl.BLACK)  # Rysowanie końcówki ramienia
        rl.draw_cylinder_ex([0,0,0], base, 0.5, 0.5, 20, rl.DARKGRAY)  # Rysowanie podstawy

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = joint2

        #print(self.get_end_effector_pos())

    def get_end_effector_pos(self):
        base = rl.Vector3(0, 0, 0)
        up = rl.Vector3(0, 1, 0)

        rot_pitch = rotation_x(self.joint_angles[0])
        rot_yaw = rotation_y(self.joint_angles[1])
        dir1 = apply_rotation(up, rot_pitch)
        dir1 = apply_rotation(dir1, rot_yaw)
        joint1 = vec3_add(base, vec3_scale(dir1, self.segment_length))

        rot_elbow = rotation_x(self.joint_angles[2])
        dir2 = apply_rotation(up, rot_elbow)
        dir2 = apply_rotation(dir2, rot_pitch)
        dir2 = apply_rotation(dir2, rot_yaw)
        joint2 = vec3_add(joint1, vec3_scale(dir2, self.segment_length))

        return joint2

    def grasp(self, primitive):
        self.grabbing = True
        self.grabbed_object = primitive

    def release(self):
        self.grabbing = False
        self.grabbed_object = None
