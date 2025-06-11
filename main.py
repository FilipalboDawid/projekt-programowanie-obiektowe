#main.py

# Importy
import raylibpy as rl
from robot import RobotArm
from primitives import Primitive
from sequence import SequenceManager
from camera import Camera
from utils import distance_vec3
from gui import PositionGUI
import numpy as np
import ctypes
import math
import math


def draw_ellipse_shadow(center, radius_x, radius_z, color, segments=36):
    points = []
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = center[0] + math.cos(angle) * radius_x
        z = center[2] + math.sin(angle) * radius_z
        points.append(rl.Vector3(x, center[1], z))
    # Rysuj trójkąty "z góry" (w prawo)
    for i in range(segments):
        rl.draw_triangle3d(
            rl.Vector3(center[0], center[1], center[2]),
            points[i],
            points[(i+1)%segments],
            color
        )
    # Rysuj trójkąty "od dołu" (w lewo)
    for i in range(segments):
        rl.draw_triangle3d(
            rl.Vector3(center[0], center[1], center[2]),
            points[(i+1)%segments],
            points[i],
            color
        )
# Inicjalizacja
WIDTH, HEIGHT = 1600, 900
rl.init_window(WIDTH, HEIGHT, b"3D Robot Arm Simulation")
rl.set_target_fps(60)
mode = 'free'
error_message = ""

light_dir = (1.0, -0.5, 0.0)  # Światło z prawej strony
light_dir_ctypes = (ctypes.c_float * 3)(*light_dir)

camera = Camera()
robot = RobotArm()
primitive = Primitive()
seq_manager = SequenceManager(robot, primitive)
gui = PositionGUI(robot)

# Shader
shader = rl.load_shader("shadow.vs", "shadow.fs")

light_dir_loc = rl.get_shader_location(shader, b"lightDirection")
rl.set_shader_value(shader, light_dir_loc, light_dir_ctypes, rl.SHADER_UNIFORM_VEC3)

# Pętla główna
while not rl.window_should_close():
    if rl.is_key_down(rl.KEY_T):
        mode = 'teach'
    if rl.is_key_down(rl.KEY_P):
        mode = 'play'
    if rl.is_key_down(rl.KEY_F):
        mode = 'free'

    camera.camera_usage()
    # Obsługa robota
    # Tryb nauki
    if mode == 'teach':
        robot.handle_input()
        seq_manager.record_frame()
        if rl.is_key_pressed(rl.KEY_G):
            end_pos = robot.get_end_effector_pos()
            obj_pos = primitive.position

            if distance_vec3(end_pos, obj_pos) < primitive.radius:
                if not robot.grabbing:
                    robot.grasp(primitive)
                    error_message = ""  # Błąd rozwiązany — resetujemy komunikat
                else:
                    error_message = "Can't grab - grabber closed."
            else:
                robot.grasp(None)
                error_message = "Can't grab - too far."
        if rl.is_key_pressed(rl.KEY_R):
            robot.release()
        if rl.is_key_pressed(rl.KEY_M):  # Klawisz do podania współrzędnych
            gui.toggle()
    # Tryb odtwarzania ruchów
    elif mode == 'play':
        if seq_manager.frames == []:
            rl.draw_text(f"No moves were taught. Press (T) to go into Teach mode", 10, 150, 20, rl.DARKGRAY)
        else:
            seq_manager.playback()
    # Tryb swobodny
    elif mode == 'free':
        robot.handle_input()
        if rl.is_key_pressed(rl.KEY_G):
            end_pos = robot.get_end_effector_pos()
            obj_pos = primitive.position
            if distance_vec3(end_pos, obj_pos) < primitive.radius:
                if not robot.grabbing:
                    robot.grasp(primitive)
                    error_message = ""  # Błąd rozwiązany — resetujemy komunikat
                else:
                    error_message = "Can't grab - grabber closed."
            else:
                robot.grasp(None)
                error_message = "Can't grab - too far."
        if rl.is_key_pressed(rl.KEY_R):
            robot.release()
        if rl.is_key_pressed(rl.KEY_M):  # Klawisz do podania współrzędnych
            gui.toggle()

    # Aktualizacja ruchu robota
    robot.update_motion()
    
    # Rysowanie
    rl.begin_drawing()

    rl.clear_background(rl.RAYWHITE)

    gui.update()
    gui.draw()

    rl.begin_mode3d(camera.camera)
    # Osie do wykasowania, na razie są *sparkles*developer tool*sparkles*
    # # Osi X, Y, Z – długość 0.3, kolory: czerwony (X), zielony (Y), niebieski (Z)
    # axis_length = 2
    # origin = (0, 0, 0)

    # # Oś X (czerwona)
    # rl.draw_line3d(origin, (axis_length, 0, 0), rl.RED)

    # # Oś Y (zielona)
    # rl.draw_line3d(origin, (0, axis_length, 0), rl.GREEN)

    # # Oś Z (niebieska)
    # rl.draw_line3d(origin, (0, 0, axis_length), rl.BLUE)
    
    # Kierunek światła (znormalizowany)
    light_dir = np.array([0.5, 0.5, 0.5])
    light_dir = light_dir / np.linalg.norm(light_dir)

    # Cień piłki (mniejsza elipsa)
    shadow_pos = (primitive.position.x, 0.01, primitive.position.z)
    shadow_radius_x = primitive.radius * 0.5 * (1 + abs(light_dir[0]))
    shadow_radius_z = primitive.radius * 0.5 * (1 + abs(light_dir[2]))
    draw_ellipse_shadow(shadow_pos, shadow_radius_x, shadow_radius_z, rl.fade(rl.BLACK, 0.45))

    # Cienie stawów
    kin = robot.compute_forward_kinematics()
    joints = [kin["base"], kin["joint1"], kin["joint2"], kin["joint3"]]
    for i, joint in enumerate(joints):
        joint_pos = np.array([joint.x, joint.y, joint.z])
        t = joint_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        shadow_pos = joint_pos + light_dir * t
        shadow_pos[1] = 0.01
        
        if i == len(joints)-1:  # Dla ostatniego stawu (chwytaka)
            # Cień chwytaka w kształcie dwóch "palców"
            finger_width = 0.08
            finger_length = 0.25
            gap = 0.1  # Odstęp między palcami
            
            # Lewy palec
            p1 = rl.Vector3(shadow_pos[0] - finger_width - gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
            p2 = rl.Vector3(shadow_pos[0] - gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
            p3 = rl.Vector3(shadow_pos[0] - gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
            p4 = rl.Vector3(shadow_pos[0] - finger_width - gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
            
            # Prawy palec
            p5 = rl.Vector3(shadow_pos[0] + gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
            p6 = rl.Vector3(shadow_pos[0] + finger_width + gap/2, shadow_pos[1], shadow_pos[2] - finger_length/2)
            p7 = rl.Vector3(shadow_pos[0] + finger_width + gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
            p8 = rl.Vector3(shadow_pos[0] + gap/2, shadow_pos[1], shadow_pos[2] + finger_length/2)
            
            # Rysuj cień lewego palca
            rl.draw_triangle3d(p1, p2, p3, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p1, p3, p4, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p3, p2, p1, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p4, p3, p1, rl.fade(rl.BLACK, 0.5))
            
            # Rysuj cień prawego palca
            rl.draw_triangle3d(p5, p6, p7, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p5, p7, p8, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p7, p6, p5, rl.fade(rl.BLACK, 0.5))
            rl.draw_triangle3d(p8, p7, p5, rl.fade(rl.BLACK, 0.5))
        else:  # Dla pozostałych stawów
            shadow_radius_x = 0.2 * (1 + abs(light_dir[0]))
            shadow_radius_z = 0.28 * (1 + abs(light_dir[2]))
            draw_ellipse_shadow(tuple(shadow_pos), shadow_radius_x, shadow_radius_z, rl.fade(rl.BLACK, 0.35))

    # Cienie ramion między stawami (prostokąt)
    def draw_shadow_rect(start, end, width, color):
        dx = end[0] - start[0]
        dz = end[2] - start[2]
        length = math.sqrt(dx*dx + dz*dz)
        if length == 0:
            return
        nx = -dz / length
        nz = dx / length
        half_w = width / 2
        p1 = rl.Vector3(start[0] + nx * half_w, start[1], start[2] + nz * half_w)
        p2 = rl.Vector3(start[0] - nx * half_w, start[1], start[2] - nz * half_w)
        p3 = rl.Vector3(end[0] - nx * half_w, end[1], end[2] - nz * half_w)
        p4 = rl.Vector3(end[0] + nx * half_w, end[1], end[2] + nz * half_w)
        # Z góry
        rl.draw_triangle3d(p1, p2, p3, color)
        rl.draw_triangle3d(p1, p3, p4, color)
        # Od dołu (odwrócona kolejność)
        rl.draw_triangle3d(p3, p2, p1, color)
        rl.draw_triangle3d(p4, p3, p1, color)

    for i in range(len(joints)-1):
        # Rzutuj oba końce segmentu na podłogę zgodnie z kierunkiem światła
        start_pos = np.array([joints[i].x, joints[i].y, joints[i].z])
        end_pos = np.array([joints[i+1].x, joints[i+1].y, joints[i+1].z])
        t_start = start_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        t_end = end_pos[1] / -light_dir[1] if light_dir[1] != 0 else 0
        shadow_start = start_pos + light_dir * t_start
        shadow_end = end_pos + light_dir * t_end
        shadow_start[1] = 0.01
        shadow_end[1] = 0.01
        shadow_width = 0.15 * (1 + 0.5 * (abs(light_dir[0]) + abs(light_dir[2])))
        draw_shadow_rect(tuple(shadow_start), tuple(shadow_end), shadow_width, rl.fade(rl.BLACK, 0.35))
    # Rysowanie robota
    rl.begin_shader_mode(shader)
    robot.draw()
    primitive.draw()
    rl.end_shader_mode()

    rl.draw_grid(20, 0.5)
    rl.end_mode3d()
    
    degrees = np.degrees(robot.joint_angles)

    # Wyświetlanie informacji na ekranie
    gui.set_info(
        mode=mode,
        joint_angles=degrees,
        end_effector_pos=robot.get_end_effector_pos(),
        target_pos=primitive.position,  # <-- tu przekazujesz bieżącą pozycję piłki!
        error_message=error_message
    )
    gui.draw_info()

    rl.end_drawing()

rl.close_window()
