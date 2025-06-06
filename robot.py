#robot.py

# Importy
import raylibpy as rl
import numpy as np
from utils import rotation_x, rotation_y, rotation_z, apply_rotation, vec3_add, vec3_scale

# Definicje
init_angles = [np.deg2rad(25), np.deg2rad(0), np.deg2rad(100)]

# Obliczanie cienia
def calculate_shading(light_direction, normal):
    light_dir_normalized = rl.vector3_normalize(light_direction)
    normal_normalized = rl.vector3_normalize(normal)
    dot_product = rl.vector3_dot_product(light_dir_normalized, normal_normalized)

    # Im większy dot_product, tym jaśniejszy kolor
    intensity = max(0, dot_product)  # Upewnij się, że wartość jest w zakresie [0, 1]
    base_color = rl.GRAY
    shaded_color = rl.color_from_normalized(intensity * rl.color_to_normalized(base_color))
    return shaded_color

# Klasa RobotArm
class RobotArm:
    # Initializacja
    def __init__(self):
        self.joint_angles = init_angles  # shoulder_pitch, shoulder_yaw, elbow
        self.segment_length = 2.0
        self.grabbing = False
        self.grabbed_object = None
        self.gripper_model = rl.gen_mesh_cube(1.0, 1.0, 1.0)
        self.gripper_model = rl.load_model_from_mesh(self.gripper_model)
        self.reaching = False
        self.target_pos = rl.Vector3(0, 0, 0)
        self.interp_alpha = 0.0  # lub 1.0, jeśli wolisz domyślnie "brak interpolacji"

    # Obsługa wejścia z klawiatury
    def handle_input(self):
        if rl.is_key_down(rl.KEY_W) and self.joint_angles[0] > -0.8*(np.pi / 4): self.joint_angles[0] -= 0.02
        if rl.is_key_down(rl.KEY_S) and self.joint_angles[0] < 0.95*(np.pi / 4) and self.get_end_effector_pos().y > 0.1: self.joint_angles[0] += 0.02  # Shoulder Pitch (X)
        if rl.is_key_down(rl.KEY_A) and self.joint_angles[1] < 0.9* np.pi: self.joint_angles[1] += 0.02  # Shoulder Yaw (Y)
        if rl.is_key_down(rl.KEY_D) and self.joint_angles[1] > - 0.9 * np.pi: self.joint_angles[1] -= 0.02
        if rl.is_key_down(rl.KEY_UP)and self.joint_angles[2] > 0.8 * (np.pi/6): self.joint_angles[2] -= 0.02  # Elbow (X)
        if rl.is_key_down(rl.KEY_DOWN) and self.joint_angles[2] < 0.8 * (np.pi) and self.get_end_effector_pos().y > 0.1: self.joint_angles[2] += 0.02
        if rl.is_key_pressed(rl.KEY_L):
            self.joint_angles = init_angles  # shoulder_pitch, shoulder_yaw, elbow

    # Rysowanie ramienia robota
    def compute_forward_kinematics(self):
        base = rl.Vector3(0, 0.5, 0)
        up = rl.Vector3(0, 1, 0)

        # Obliczenia rotacji
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

        return {
            "base": base,
            "joint1": joint1,
            "joint2": joint2,
            "dir1": dir1,
            "dir2": dir2
        }
    
    def draw(self):
        kin = self.compute_forward_kinematics()
        base = kin["base"]
        joint1 = kin["joint1"]
        joint2 = kin["joint2"]
        dir2 = kin["dir2"]

        # Segmenty ramienia
        rl.draw_cylinder_ex(base, joint1, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_cylinder_ex(joint1, joint2, 0.1, 0.1, 10, rl.GRAY)

        # Stawy i podstawa
        rl.draw_sphere(base, 0.2, rl.DARKGRAY)
        rl.draw_sphere(joint1, 0.2, rl.DARKGRAY)
        rl.draw_cylinder_ex([0, 0, 0], base, 0.5, 0.5, 20, rl.DARKGRAY)

        # Chwytak
        normalized_dir = rl.vector3_normalize(dir2)
        default_up = rl.Vector3(0, 1, 0)
        axis = rl.vector3_cross_product(default_up, normalized_dir)
        dot = rl.vector3_dot_product(default_up, normalized_dir)
        angle_rad = np.arccos(dot)
        angle_deg = np.degrees(angle_rad)

        if rl.vector3_length(axis) < 0.0001:
            axis = rl.Vector3(1, 0, 0)

        spread = 0.05 if self.grabbing else 0.15
        default_right = rl.Vector3(0, 1, 0)
        side_vector = rl.vector3_cross_product(normalized_dir, default_right)
        if rl.vector3_length(side_vector) < 0.001:
            side_vector = rl.Vector3(0, 0, 1)

        side_vector = rl.vector3_normalize(side_vector)
        finger1_pos = rl.vector3_add(joint2, rl.vector3_scale(side_vector, spread))
        finger2_pos = rl.vector3_subtract(joint2, rl.vector3_scale(side_vector, spread))
        finger_size = rl.Vector3(0.05, 0.2, 0.05)
        color = rl.DARKGRAY if self.grabbing else rl.DARKGRAY

        rl.draw_cube(finger1_pos, finger_size.x, finger_size.y, finger_size.z, color)
        rl.draw_cube(finger2_pos, finger_size.x, finger_size.y, finger_size.z, color)

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = joint2

    # Pobranie pozycji końcówki ramienia robota
    def get_end_effector_pos(self):
        kin = self.compute_forward_kinematics()
        return kin["joint2"]
    
    # Ruch do określonej pozycji (teleportacja)
    def move_to_position(self, target_position):
        x, y, z = target_position.x, target_position.y, target_position.z

        dx = x
        dy = y - 0.5
        dz = z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        if distance > 2 * self.segment_length:
            print("Cel poza zasięgiem!")
            return False

        shoulder_yaw = np.arctan2(dz, dx)
        planar_distance = np.sqrt(dx**2 + dz**2)
        total_distance = np.sqrt(planar_distance**2 + dy**2)

        cos_elbow = (2 * self.segment_length**2 - total_distance**2) / (2 * self.segment_length**2)
        if not -1 <= cos_elbow <= 1:
            print("Błąd geometrii (cos_elbow poza zakresem)")
            return False
        elbow_angle = np.pi + np.arccos(cos_elbow)
        shoulder_pitch = elbow_angle / 2 - np.arctan2(dy, planar_distance)

        self.target_joint_angles = [shoulder_pitch, shoulder_yaw, elbow_angle]
        self.reaching = True
        return True
    
    def update_motion(self, speed=0.02):
        if not self.reaching:
            return

        done = True
        for i in range(3):
            diff = self.target_joint_angles[i] - self.joint_angles[i]
            if abs(diff) > 0.01:
                self.joint_angles[i] += np.clip(diff, -speed, speed)
                done = False

        if done:
            self.reaching = False
            print("Osiągnięto cel.")

    # Chwytanie obiektu
    def grasp(self, primitive):
        self.grabbing = True
        self.grabbed_object = primitive

    def release(self):
        self.grabbing = False
        self.grabbed_object = None
