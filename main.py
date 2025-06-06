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

# Inicjalizacja
WIDTH, HEIGHT = 1600, 900
rl.init_window(WIDTH, HEIGHT, b"3D Robot Arm Simulation")
rl.set_target_fps(60)
mode = 'free'

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
                if robot.grabbing == False:
                    robot.grasp(primitive)
                else:
                    print("Chwytak jest zamknięty, nie można chwycić.")
            else:
                robot.grasp(None)
                print("Za daleko od obiektu, nie można chwycić.")
        if rl.is_key_pressed(rl.KEY_R):
            robot.release()
        if rl.is_key_pressed(rl.KEY_M):  # Klawisz do podania współrzędnych
            gui.toggle()
    # Tryb odtwarzania ruchów
    elif mode == 'play':
        if seq_manager.frames == []:
            rl.draw_text(f"Nie nagrano zadnych ruchow. Przelacz sie na tryb nauki (T), aby nagrac sekwencje.", 10, 150, 20, rl.DARKGRAY)
        else:
            seq_manager.playback()
    # Tryb swobodny
    elif mode == 'free':
        robot.handle_input()
        if rl.is_key_pressed(rl.KEY_G):
            end_pos = robot.get_end_effector_pos()
            obj_pos = primitive.position
            if distance_vec3(end_pos, obj_pos) < primitive.radius:
                if robot.grabbing == False:
                    robot.grasp(primitive)
                else:
                    print("Chwytak jest zamknięty, nie można chwycić.")
            else:
                robot.grasp(None)
                print("Za daleko od obiektu, nie można chwycić.")
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
    
    # Osi X, Y, Z – długość 0.3, kolory: czerwony (X), zielony (Y), niebieski (Z)
    axis_length = 2
    origin = (0, 0, 0)

    # Oś X (czerwona)
    rl.draw_line3d(origin, (axis_length, 0, 0), rl.RED)

    # Oś Y (zielona)
    rl.draw_line3d(origin, (0, axis_length, 0), rl.GREEN)

    # Oś Z (niebieska)
    rl.draw_line3d(origin, (0, 0, axis_length), rl.BLUE)

    # Rysowanie robota
    rl.begin_shader_mode(shader)
    robot.draw()  # lub ręczne rysowanie segmentów
    primitive.draw()  # lub ręczne rysowanie segmentów
    rl.end_shader_mode()


    # Rysowanie cienia robota
    light_direction = rl.Vector3(-0.5, -1.0, -0.5)  # Kierunek światła

    # # Rysowanie obiektu
    # rl.begin_shader_mode(shadow_shader)
    # robot.draw()  # lub ręczne rysowanie segmentów
    # primitive.draw()  # lub ręczne rysowanie segmentów
    # rl.end_shader_mode()

    rl.draw_grid(20, 0.5)
    rl.end_mode3d()
    
    degrees = np.degrees(robot.joint_angles)

    # Wyświetlanie informacji na ekranie
    rl.draw_text(f"Mode: {mode.upper()} (T/P/F) | G: Grab | R: Release | M: Give coordinates", 10, 10, 20, rl.DARKGRAY)
    rl.draw_text(f"Joint Angles: {degrees}", 10, 40, 20, rl.DARKGRAY)
    rl.draw_text(f"End Effector Position: {robot.get_end_effector_pos()}", 10, 70, 20, rl.DARKGRAY)
    rl.draw_text(f"Grabbing: {robot.grabbing}", 10, 100, 20, rl.DARKGRAY)
    rl.draw_text(f"A,D - Shoulder yaw; W,S - Shoulder pitch, UP, DOWN - Elbow pitch", 10, 130, 20, rl.DARKGRAY)
    rl.draw_text(f"Target location: {gui.target}", 10, 160, 20, rl.DARKGRAY)
    rl.end_drawing()

rl.close_window()
