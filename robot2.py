import raylibpy as rl
import numpy as np

# Pomocnicze funkcje wektorowe
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

def color_with_alpha(color, alpha):
    # alpha w zakresie 0.0 - 1.0
    return rl.Color(color.r, color.g, color.b, int(alpha * 255))

class RobotArm:
    def __init__(self):
        self.joint_angles = [0.0, 0.0, 0.0] # shoulder_pitch, shoulder_yaw, elbow
        self.segment_length = 2.0
        self.grabbing = False
        self.grabbed_object = None

    def calculate_shadow_pos(self, position, light_height=10.0):
        # Pozycja światła (zakładamy, że źródło jest nad sceną)
        light_pos = rl.Vector3(0, light_height, 0)
        # Wektor od światła do obiektu
        light_dir = rl.vector3_subtract(position, light_pos)
        # Punkt przecięcia z podłogą (y=0)
        t = -light_pos.y / light_dir.y
        shadow_x = light_pos.x + light_dir.x * t
        shadow_z = light_pos.z + light_dir.z * t
        return rl.Vector3(shadow_x, 0.01, shadow_z) # 0.01 aby uniknąć "z-fighting"

    def handle_input(self):
        if rl.is_key_down(rl.KEY_W): self.joint_angles[0] += 0.01 # Shoulder Pitch (X)
        if rl.is_key_down(rl.KEY_S): self.joint_angles[0] -= 0.01
        if rl.is_key_down(rl.KEY_A): self.joint_angles[1] += 0.01 # Shoulder Yaw (Y)
        if rl.is_key_down(rl.KEY_D): self.joint_angles[1] -= 0.01
        if rl.is_key_down(rl.KEY_UP): self.joint_angles[2] += 0.01 # Elbow (X)
        if rl.is_key_down(rl.KEY_DOWN): self.joint_angles[2] -= 0.01

    def draw(self):
        base = rl.Vector3(0, 0.5, 0)
        up = rl.Vector3(0, 1, 0)

        # Oblicz pozycje stawów
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

        # Rysuj cień podstawy
        shadow_base = self.calculate_shadow_pos(base)
        rl.draw_cylinder(shadow_base, 0.15, 0.15, 0.01, 16, color_with_alpha(rl.BLACK, 0.5))

        # Rysuj ramię 1 + jego cień
        shadow_joint1 = self.calculate_shadow_pos(joint1)
        rl.draw_cylinder_ex(base, joint1, 0.1, 0.1, 4, rl.GRAY)
        rl.draw_cylinder_ex(shadow_base, shadow_joint1, 0.1, 0.1, 4, color_with_alpha(rl.BLACK, 0.3))

        # Rysuj ramię 2 + jego cień
        shadow_joint2 = self.calculate_shadow_pos(joint2)
        rl.draw_cylinder_ex(joint1, joint2, 0.1, 0.1, 4, rl.GRAY)
        rl.draw_cylinder_ex(shadow_joint1, shadow_joint2, 0.1, 0.1, 4, color_with_alpha(rl.BLACK, 0.3))

        # Rysuj końcówkę + cień
        end_effector = joint2
        shadow_end = self.calculate_shadow_pos(end_effector)
        rl.draw_cube(end_effector, 0.2, 0.2, 0.2, rl.BLACK)
        rl.draw_cube(shadow_end, 0.2, 0.01, 0.2, color_with_alpha(rl.BLACK, 0.5))

        # Rysuj stawy
        rl.draw_sphere(joint1, 0.1, rl.RED)
        rl.draw_sphere(joint2, 0.1, rl.RED)

        # Chwytak (szczęki)
        jaw_offset = 0.15 if not self.grabbing else 0.05
        jaw1 = rl.Vector3(end_effector.x + jaw_offset, end_effector.y, end_effector.z)
        jaw2 = rl.Vector3(end_effector.x - jaw_offset, end_effector.y, end_effector.z)
        rl.draw_cube(jaw1, 0.05, 0.05, 0.15, rl.YELLOW)
        rl.draw_cube(jaw2, 0.05, 0.05, 0.15, rl.YELLOW)

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = end_effector

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
