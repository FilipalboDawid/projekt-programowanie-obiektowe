#robot.py

# Importy
import raylibpy as rl
import numpy as np
from utils import rotation_x, rotation_y, rotation_z, apply_rotation, vec3_add, vec3_scale, dh_transform, rot_y, dh_link, to_vec3

# Definicje
init_angles = [np.deg2rad(65), np.deg2rad(0), np.deg2rad(-100)]

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

def compute_inverse_kinematics_dh(x, y, z, L1, L2, L3):
    solutions = []

    # dwie opcje dla theta1
    theta1_options = [np.arctan2(z, x), np.arctan2(-z, -x)]

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
        theta1 = self.joint_angles[1]  # yaw
        theta2 = self.joint_angles[0]  # pitch
        theta3 = self.joint_angles[2]  # elbow

        L1 = 0.5
        L2 = self.segment_length
        L3 = self.segment_length

        # DH: Z0→Z1 (obrót yaw wokół Y, baza w górę Y)
        T0_1 = np.array([
            [np.cos(theta1), 0, np.sin(theta1), 0],
            [0, 1, 0, L1],
            [-np.sin(theta1), 0, np.cos(theta1), 0],
            [0, 0, 0, 1]
        ])

        # DH: Z1→Z2 (pitch)
        T1_2 = np.array([
            [np.cos(theta2), -np.sin(theta2), 0, L2 * np.cos(theta2)],
            [np.sin(theta2), np.cos(theta2),  0, L2 * np.sin(theta2)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # DH: Z2→Z3 (elbow)
        T2_3 = np.array([
            [np.cos(theta3), -np.sin(theta3), 0, L3 * np.cos(theta3)],
            [np.sin(theta3), np.cos(theta3),  0, L3 * np.sin(theta3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        T0_2 = T0_1 @ T1_2
        T0_3 = T0_2 @ T2_3

        # Punkty
        base = rl.Vector3(0, 0, 0)
        joint1 = to_vec3(T0_1 @ np.array([0, 0, 0, 1]))
        joint2 = to_vec3(T0_2 @ np.array([0, 0, 0, 1]))
        joint3 = to_vec3(T0_3 @ np.array([0, 0, 0, 1]))

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
        rl.draw_cylinder_ex(base, joint1, 0.5, 0.5, 10, rl.GRAY)
        rl.draw_cylinder_ex(joint1, joint2, 0.1, 0.1, 10, rl.GRAY)
        rl.draw_cylinder_ex(joint2, joint3, 0.1, 0.1, 10, rl.GRAY)

        # Rysowanie stawów
        rl.draw_sphere(joint1, 0.2, rl.DARKGRAY)
        rl.draw_sphere(joint2, 0.2, rl.DARKGRAY)
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

    def move_to_position(self, target_position):
        solutions = compute_inverse_kinematics_dh(target_position.x, target_position.y, target_position.z, 0.5, self.segment_length, self.segment_length)
        if not solutions:
            print("Brak rozwiązań odwrotnej kinematyki")
            return False

        def end_effector_error(joint_angles):
            backup = self.joint_angles.copy()
            self.joint_angles = joint_angles
            end_pos = self.get_end_effector_pos()
            self.joint_angles = backup
            dx = end_pos.x - target_position.x
            dy = end_pos.y - target_position.y
            dz = end_pos.z - target_position.z
            return dx**2 + dy**2 + dz**2

        best_solution = min(solutions, key=end_effector_error)
        self.target_joint_angles = best_solution
        self.reaching = True

        for i, sol in enumerate(solutions):
            deg = np.rad2deg(sol)
            print(f"Rozwiązanie {i+1}: pitch={deg[0]:.1f}, yaw={deg[1]:.1f}, elbow={deg[2]:.1f}")
        print("Wybrane kąty:", np.rad2deg(self.target_joint_angles))
        print("Docelowa pozycja:", target_position)

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
