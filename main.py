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

def draw_ellipse_shadow(center, radius_x, radius_z, color, segments=36):
    points = []
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = center[0] + math.cos(angle) * radius_x
        z = center[2] + math.sin(angle) * radius_z
        points.append(rl.Vector3(x, center[1], z))
    for i in range(segments):
        rl.draw_triangle3d(
            rl.Vector3(center[0], center[1], center[2]),
            points[i],
            points[(i+1)%segments],
            color
        )
# Inicjalizacja
WIDTH, HEIGHT = 1600, 900
rl.init_window(WIDTH, HEIGHT, b"3D Robot Arm Simulation")
rl.set_target_fps(60)
mode = 'free'
error_message = ""

light_dir = (-0.5, -1.0, -0.5)
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
    
    # Rysowanie cieni
    # Cień piłki
    shadow_pos = (primitive.position.x, 0.01, primitive.position.z)
    rl.draw_cylinder(shadow_pos, primitive.radius * 1, primitive.radius * 1, 0.01, 32, rl.fade(rl.BLACK, 0.3))
    
    # Cień robota (pod każdym segmentem)
    kin = robot.compute_forward_kinematics()
    joints = [kin["base"], kin["joint1"], kin["joint2"], kin["joint3"]]
    
    # Cienie stawów
    for joint in joints:
        shadow_pos = (joint.x, 0.01, joint.z)
        rl.draw_cylinder(shadow_pos, 0.2, 0.2, 0.01, 32, rl.fade(rl.BLACK, 0.3))
    
    # Cienie ramion między stawami (mniejszy promień)
    for i in range(len(joints)-1):
        start = (joints[i].x, 0.01, joints[i].z)
        end = (joints[i+1].x, 0.01, joints[i+1].z)
        rl.draw_cylinder_ex(start, end, 0.08, 0.08, 32, rl.fade(rl.BLACK, 0.3))
    
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
