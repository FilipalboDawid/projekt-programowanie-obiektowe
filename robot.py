#robot.py

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
def calculate_shading(light_direction, normal):
    """
    Oblicza kolor na podstawie kąta między kierunkiem światła a normalną powierzchni.
    :param light_direction: Kierunek światła (Vector3).
    :param normal: Normalna powierzchni (Vector3).
    :return: Kolor (Color).
    """
    light_dir_normalized = rl.vector3_normalize(light_direction)
    normal_normalized = rl.vector3_normalize(normal)
    dot_product = rl.vector3_dot_product(light_dir_normalized, normal_normalized)

    # Im większy dot_product, tym jaśniejszy kolor
    intensity = max(0, dot_product)  # Upewnij się, że wartość jest w zakresie [0, 1]
    base_color = rl.GRAY
    shaded_color = rl.color_from_normalized(intensity * rl.color_to_normalized(base_color))
    return shaded_color

def apply_rotation(vec, mat):
    x = vec.x * mat[0][0] + vec.y * mat[0][1] + vec.z * mat[0][2]
    y = vec.x * mat[1][0] + vec.y * mat[1][1] + vec.z * mat[1][2]
    z = vec.x * mat[2][0] + vec.y * mat[2][1] + vec.z * mat[2][2]
    return rl.Vector3(x, y, z)

class RobotArm:
    def __init__(self):
        self.joint_angles = [np.deg2rad(-25), np.deg2rad(0), np.deg2rad(-100)]  # shoulder_pitch, shoulder_yaw, elbow
        self.segment_length = 2.0
        self.grabbing = False
        self.grabbed_object = None
        self.gripper_model = rl.gen_mesh_cube(1.0, 1.0, 1.0)
        self.gripper_model = rl.load_model_from_mesh(self.gripper_model)
    
    
    def move_to_position(self, target_position):
        """
        Przemieszcza ramię robota do określonej pozycji.
        :param target_position: Wektor docelowej pozycji (Vector3).
        """
        x, y, z = target_position.x, target_position.y, target_position.z

        # Oblicz odległość od podstawy do celu w płaszczyźnie XZ
        distance = np.sqrt(x**2 + z**2)

        # Sprawdź, czy cel jest osiągalny
        if distance > 2 * self.segment_length or y < 0:
            print("Cel poza zasięgiem robota!")
            return False

        # Oblicz kąty stawów
        shoulder_yaw = np.atan2(z, x)  # Kąt obrotu w płaszczyźnie XZ
        shoulder_pitch = np.atan2(y, distance)  # Kąt podniesienia ramienia
        elbow_angle = -np.acos((distance**2 + y**2) / (2 * self.segment_length**2))  # Kąt łokcia

        # Zaktualizuj kąty stawów
        self.joint_angles[0] = shoulder_pitch
        self.joint_angles[1] = shoulder_yaw
        self.joint_angles[2] = elbow_angle

        print(f"Robot przemieścił się do pozycji: {target_position}")
        return True
    
    def move_to_position_smooth(self, target_position, step=0.01):
        """
        Przemieszcza ramię robota stopniowo do określonej pozycji.
        :param target_position: Wektor docelowej pozycji (Vector3).
        :param step: Wielkość kroku przesunięcia.
        """
        x, y, z = target_position.x, target_position.y, target_position.z

        # Oblicz odległość od podstawy do celu w płaszczyźnie XZ
        distance = np.sqrt(x**2 + z**2)

        # Sprawdź, czy cel jest osiągalny
        if distance > 2 * self.segment_length or y < 0:
            print("Cel poza zasięgiem robota!")
            return False

        # Oblicz docelowe kąty stawów
        shoulder_yaw = np.atan2(z, x)  # Kąt obrotu w płaszczyźnie XZ
        shoulder_pitch = np.atan2(y, distance)  # Kąt podniesienia ramienia
        elbow_angle = -np.acos((distance**2 + y**2) / (2 * self.segment_length**2))  # Kąt łokcia

        # Stopniowe przesuwanie kątów
        self.joint_angles[0] += step * (shoulder_pitch - self.joint_angles[0])
        self.joint_angles[1] += step * (shoulder_yaw - self.joint_angles[1])
        self.joint_angles[2] += step * (elbow_angle - self.joint_angles[2])

        return True
    
    def draw_shadow(self, light_direction):
        """
        Rysuje cień ramienia robota na płaszczyźnie na podstawie kierunku światła.
        :param light_direction: Kierunek światła (Vector3).
        """
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

        # Oblicz pozycje cieni na płaszczyźnie Y=0
        shadow_base = rl.Vector3(base.x - light_direction.x * base.y, 0.0, base.z - light_direction.z * base.y)
        shadow_joint1 = rl.Vector3(joint1.x - light_direction.x * joint1.y, 0.0, joint1.z - light_direction.z * joint1.y)
        shadow_joint2 = rl.Vector3(joint2.x - light_direction.x * joint2.y, 0.0, joint2.z - light_direction.z * joint2.y)

        # Rysowanie cieni jako cylindry
        rl.draw_cylinder_ex(shadow_base, shadow_joint1, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_cylinder_ex(shadow_joint1, shadow_joint2, 0.1, 0.1, 10, rl.GRAY)

        # Rysowanie cieni dla stawów
        rl.draw_sphere(shadow_joint1, 0.2, rl.GRAY)
        
        # Rysowanie cienia chwytaka
        if self.grabbing:
            shadow_gripper = rl.Vector3(joint2.x - light_direction.x * joint2.y, 0.0, joint2.z - light_direction.z * joint2.y)
            rl.draw_cube(shadow_gripper, 0.3, 0.1, 0.3, rl.GRAY)
        else:
            shadow_gripper = rl.Vector3(joint2.x - light_direction.x * joint2.y, 0.0, joint2.z - light_direction.z * joint2.y)
            rl.draw_cube(shadow_gripper, 0.2, 0.1, 0.2, rl.GRAY)
    
    def handle_input(self):
        if rl.is_key_down(rl.KEY_W) and self.joint_angles[0] < 0.8*(np.pi / 4): self.joint_angles[0] += 0.02  # Shoulder Pitch (X)
        if rl.is_key_down(rl.KEY_S) and self.joint_angles[0] > -0.95*(np.pi / 4) and self.get_end_effector_pos().y > 0.1: self.joint_angles[0] -= 0.02
        if rl.is_key_down(rl.KEY_A) and self.joint_angles[1] < 0.9* np.pi: self.joint_angles[1] += 0.02  # Shoulder Yaw (Y)
        if rl.is_key_down(rl.KEY_D) and self.joint_angles[1] > - 0.9 * np.pi: self.joint_angles[1] -= 0.02
        if rl.is_key_down(rl.KEY_UP)and self.joint_angles[2] < 0.8 * (np.pi/8): self.joint_angles[2] += 0.02  # Elbow (X)
        if rl.is_key_down(rl.KEY_DOWN) and self.joint_angles[2] > -0.8 * np.pi and self.get_end_effector_pos().y > 0.1: self.joint_angles[2] -= 0.02
        if rl.is_key_pressed(rl.KEY_ZERO):
            self.joint_angles = [np.deg2rad(-25), np.deg2rad(0), np.deg2rad(-100)]  # shoulder_pitch, shoulder_yaw, elbow


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
        rl.draw_sphere(joint1, 0.2, rl.DARKGRAY)  # Rysowanie stawów
        rl.draw_sphere(base, 0.2, rl.DARKGRAY)
        rl.draw_cylinder_ex([0,0,0], base, 0.5, 0.5, 20, rl.DARKGRAY)  # Rysowanie podstawy
        # Gripper orientation
        gripper_direction = dir2
        normalized_dir = rl.vector3_normalize(gripper_direction)

        default_up = rl.Vector3(0, 1, 0)
        axis = rl.vector3_cross_product(default_up, normalized_dir)
        dot = rl.vector3_dot_product(default_up, normalized_dir)
        angle_rad = np.arccos(dot)
        angle_deg = np.degrees(angle_rad)

        if rl.vector3_length(axis) < 0.0001:
            axis = rl.Vector3(1, 0, 0)  # dowolna oś, jeśli są równoległe

        scale = rl.Vector3(0.3, 0.1, 0.3) if self.grabbing else rl.Vector3(0.2, 0.1, 0.2)
        gripper_color = rl.DARKGRAY if self.grabbing else rl.BLACK

        # Zmienna gripper spread - odległość palców
        spread = 0.05 if self.grabbing else 0.15

        # Kierunek chwytaka
        gripper_direction = dir2
        normalized_dir = rl.vector3_normalize(gripper_direction)

        # Znajdź wektor prostopadły do kierunku chwytaka – np. oś lokalna X chwytaka
        default_right = rl.Vector3(0, 1, 0)
        side_vector = rl.vector3_cross_product(normalized_dir, default_right)
        if rl.vector3_length(side_vector) < 0.001:
            side_vector = rl.Vector3(0, 0, 1)  # fallback, jeśli wektory są równoległe

        side_vector = rl.vector3_normalize(side_vector)

        # Pozycje dwóch palców
        finger1_pos = rl.vector3_add(joint2, rl.vector3_scale(side_vector, spread))
        finger2_pos = rl.vector3_subtract(joint2, rl.vector3_scale(side_vector, spread))

        # Rozmiar palców
        finger_size = rl.Vector3(0.05, 0.2, 0.05)

        # Kolor
        color = rl.DARKGRAY if self.grabbing else rl.DARKGRAY

        # Narysuj oba palce
        rl.draw_cube(finger1_pos, finger_size.x, finger_size.y, finger_size.z, color)
        rl.draw_cube(finger2_pos, finger_size.x, finger_size.y, finger_size.z, color)

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = joint2

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
