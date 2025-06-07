#robot.py

# Importy
import raylibpy as rl
import numpy as np
from utils import rotation_x, rotation_y, rotation_z, apply_rotation, vec3_add, vec3_scale, dh_transform, rot_y, dh_link, to_vec3

# Definicje
init_angles = [np.deg2rad(50), np.deg2rad(0), np.deg2rad(0)]

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
        # shoulder_pitch (X)
        if rl.is_key_down(rl.KEY_W) and self.joint_angles[0] < np.deg2rad(120):
            self.joint_angles[0] += 0.02
        if rl.is_key_down(rl.KEY_S) and self.joint_angles[0] > np.deg2rad(40):
            self.joint_angles[0] -= 0.02

        # shoulder_yaw (Y)
        if rl.is_key_down(rl.KEY_A) and self.joint_angles[1] < np.deg2rad(175):
            self.joint_angles[1] += 0.02
        if rl.is_key_down(rl.KEY_D) and self.joint_angles[1] > -np.deg2rad(175):
            self.joint_angles[1] -= 0.02

        # elbow (X)
        if rl.is_key_down(rl.KEY_UP) and self.joint_angles[2] < 0:
            self.joint_angles[2] += 0.02
        if rl.is_key_down(rl.KEY_DOWN) and self.joint_angles[2] > -np.deg2rad(150):
            self.joint_angles[2] -= 0.02

        # Reset pozycji
        if rl.is_key_pressed(rl.KEY_L):
            self.joint_angles = init_angles

    # Rysowanie ramienia robota
    def compute_forward_kinematics(self):
        a = self.segment_length
        theta_pitch = self.joint_angles[0]  # shoulder_pitch
        theta_elbow = self.joint_angles[2]  # elbow
        shoulder_yaw = self.joint_angles[1] # shoulder_yaw

        # Transformacja podstawowa - przesunięcie bazy do (0, 0.5, 0)
        base_translation = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0.5],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Składamy macierze:
        T_base = base_translation
        T_yaw = rot_y(shoulder_yaw)
        T_pitch = dh_link(a, theta_pitch)
        T_elbow = dh_link(a, theta_elbow)

        T0_1 = T_base @ T_yaw
        T0_2 = T0_1 @ T_pitch
        T0_3 = T0_2 @ T_elbow

        # Pozycje punktów w przestrzeni bazowej
        
        base = rl.Vector3(0, 0.5, 0)
        joint1 = to_vec3(T0_1 @ np.array([0,0,0,1]))
        joint2 = to_vec3(T0_2 @ np.array([0,0,0,1]))
        joint3 = to_vec3(T0_3 @ np.array([0,0,0,1]))

        return {
            "base": base,
            "joint1": joint1,
            "joint2": joint2,
            "joint3": joint3,
            "T0_1": T0_1,
            "T0_2": T0_2,
            "T0_3": T0_3
        }
    
    def draw(self):
        kin = self.compute_forward_kinematics()
        base = kin["base"]
        joint1 = kin["joint1"]
        joint2 = kin["joint2"]
        joint3 = kin["joint3"]

        # Rysowanie segmentów ramienia (3 segmenty):
        rl.draw_cylinder_ex(base, joint1, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_cylinder_ex(joint1, joint2, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_cylinder_ex(joint2, joint3, 0.1, 0.1, 10, rl.GRAY)

        # Rysowanie stawów
        rl.draw_sphere(base, 0.2, rl.DARKGRAY)
        rl.draw_sphere(joint1, 0.2, rl.DARKGRAY)
        rl.draw_sphere(joint2, 0.2, rl.DARKGRAY)

        # Podstawa (np. cylinder od zera do bazy)
        rl.draw_cylinder_ex(rl.Vector3(0, 0, 0), base, 0.5, 0.5, 20, rl.DARKGRAY)

        # Kierunek końcówki (chwytaka) - wektor od joint2 do joint3
        forward = rl.Vector3(
            joint3.x - joint2.x,
            joint3.y - joint2.y,
            joint3.z - joint2.z
        )
        forward = rl.vector3_normalize(forward)

        # Globalny "up"
        global_up = rl.Vector3(0, 1, 0)
        dot = rl.vector3_dot_product(forward, global_up)
        if abs(dot) > 0.99:
            global_up = rl.Vector3(0, 0, 1)

        # Wektor boczny prostopadły do forward i global_up
        side_vector = rl.vector3_cross_product(forward, global_up)
        side_vector = rl.vector3_normalize(side_vector)

        # Rozstaw szczypiec
        spread = 0.05 if self.grabbing else 0.15

        # Pozycje palców szczypiec
        finger1_pos = rl.vector3_add(joint3, rl.vector3_scale(side_vector, spread))
        finger2_pos = rl.vector3_subtract(joint3, rl.vector3_scale(side_vector, spread))

        # Rozmiar palców szczypiec
        finger_size = rl.Vector3(0.05, 0.2, 0.05)
        color = rl.DARKGRAY if self.grabbing else rl.DARKGRAY

        # Rysuj szczypce
        rl.draw_cube(finger1_pos, finger_size.x, finger_size.y, finger_size.z, color)
        rl.draw_cube(finger2_pos, finger_size.x, finger_size.y, finger_size.z, color)

        # Jeśli chwytamy obiekt, aktualizuj jego pozycję na końcówkę
        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = joint3


    # Pobranie pozycji końcówki ramienia robota
    def get_end_effector_pos(self):
        kin = self.compute_forward_kinematics()
        return kin["joint3"]
    
    # Ruch do określonej pozycji (teleportacja)
    def compute_inverse_kinematics(self, target_pos: rl.Vector3):
        """
        Oblicz odwrotną kinematykę metodą DH dla 3DOF ramienia (shoulder_pitch, shoulder_yaw, elbow).
        target_pos - zadana pozycja końcówki ramienia (Vector3).
        Zwraca: tuple (success: bool, angles: list of 3 angles w radianach)
        """

        x = target_pos.x
        y = target_pos.y - 0.5  # baza jest na wysokości 0.5
        z = target_pos.z

        shoulder_yaw = np.arctan2(z, x)
        planar_dist = np.sqrt(x**2 + z**2)
        total_dist = np.sqrt(planar_dist**2 + y**2)

        L = self.segment_length

        if total_dist > 2 * L:
            return False, [0, 0, 0]

        cos_elbow = (2 * L**2 - total_dist**2) / (2 * L**2)

        if cos_elbow < -1 or cos_elbow > 1:
            return False, [0, 0, 0]

        elbow_angle = np.pi + np.arccos(cos_elbow)

        k1 = L + L * np.cos(elbow_angle)
        k2 = L * np.sin(elbow_angle)
        shoulder_pitch = np.arctan2(y, planar_dist) - np.arctan2(k2, k1)

        shoulder_pitch = np.clip(shoulder_pitch, np.deg2rad(40), np.deg2rad(120))
        shoulder_yaw = np.clip(shoulder_yaw, -np.deg2rad(175), np.deg2rad(175))
        elbow_angle = np.clip(elbow_angle - np.pi, -np.deg2rad(150), 0)

        print(f"IK Debug: target=({x:.2f},{y+0.5:.2f},{z:.2f}), angles=({np.rad2deg(shoulder_pitch):.1f},{np.rad2deg(shoulder_yaw):.1f},{np.rad2deg(elbow_angle):.1f})")

        return True, [shoulder_pitch, shoulder_yaw, elbow_angle]

    def move_to_position(self, target_position):
        success, angles = self.compute_inverse_kinematics(target_position)
        if not success:
            print("Cel poza zasięgiem lub nieosiągalny.")
            return False

        self.target_joint_angles = angles
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
