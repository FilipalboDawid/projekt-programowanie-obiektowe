#main.py

# Importy
import raylibpy as rl
from robot import RobotArm
from primitives import Primitive
from sequence import SequenceManager
import numpy as np
import ctypes


# Stałe
WIDTH, HEIGHT = 1600, 900

light_dir = (-0.5, -1.0, -0.5)
light_dir_ctypes = (ctypes.c_float * 3)(*light_dir)

# Definicje
def distance_vec3(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return (dx*dx + dy*dy + dz*dz) ** 0.5


# Inicjalizacja
rl.init_window(WIDTH, HEIGHT, b"3D Robot Arm Simulation")

# Shader
shader = rl.load_shader("shadow.vs", "shadow.fs")

light_dir_loc = rl.get_shader_location(shader, b"lightDirection")
rl.set_shader_value(shader, light_dir_loc, light_dir_ctypes, rl.SHADER_UNIFORM_VEC3)

rl.set_target_fps(60)

camera = rl.Camera3D()
camera.position = rl.Vector3(-5.0, 6.0, -5.0)
camera.target = rl.Vector3(0.0, 0.0, 0.0)
camera.up = rl.Vector3(0.0, 1.0, 0.0)
camera.fovy = 45.0
camera.projection = rl.CAMERA_PERSPECTIVE

radius = distance_vec3(camera.position, camera.target)

# Obliczanie początkowych yaw i pitch na podstawie pozycji kamery
offset = rl.vector3_subtract(camera.position, camera.target)
radius = distance_vec3(camera.position, camera.target)

yaw = np.arctan2(offset.x, offset.z)
pitch = np.arcsin(offset.y / radius)

is_rotating = False
start_yaw = yaw
start_pitch = pitch
mouse_start = rl.Vector2(0, 0)


robot = RobotArm()
primitive = Primitive()
seq_manager = SequenceManager(robot, primitive)

show_position_gui = False
position_gui_message = ""
input_field = 'x'  # aktywne pole: 'x', 'y', 'z'
input_str = {'x': '', 'y': '', 'z': ''}
target_x_gui = 0.0
target_y_gui = 0.0
target_z_gui = 0.0

mode = 'free'
# Pętla główna
while not rl.window_should_close():
    if rl.is_key_down(rl.KEY_T):
        mode = 'teach'
    if rl.is_key_down(rl.KEY_P):
        mode = 'play'
    if rl.is_key_down(rl.KEY_F):
        mode = 'free'

    # Obsługa kamery
    mouse_delta = rl.get_mouse_delta()
    sensitivity = 0.003
    pan_speed = 0.01

    # Wektor forward (kierunek patrzenia kamery)
    forward = rl.vector3_subtract(camera.target, camera.position)
    forward = rl.vector3_normalize(forward)

    # Wektor right (prostopadły do forward i up)
    right = rl.vector3_cross_product(forward, camera.up)
    right = rl.vector3_normalize(right)

    # Obrót kamery LPM
    # Rozpoczęcie rotacji kamery
    if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT) and rl.is_cursor_on_screen():
        is_rotating = True
        radius = distance_vec3(camera.position, camera.target)  # Zapamiętaj aktualny zoom

    # Zakończenie rotacji kamery
    if rl.is_mouse_button_released(rl.MOUSE_BUTTON_LEFT):
        is_rotating = False

    # Aktualizacja rotacji kamery
    if is_rotating and rl.is_cursor_on_screen():
        yaw += mouse_delta.x * sensitivity
        pitch -= mouse_delta.y * sensitivity
        pitch = max(-np.pi / 2 + 0.1, min(np.pi / 2 - 0.1, pitch))

        radius = distance_vec3(camera.position, camera.target)  # dynamiczne

        # Nowa pozycja kamery na podstawie aktualnego zoomu
        camera.position.x = camera.target.x + radius * np.cos(pitch) * np.sin(yaw)
        camera.position.y = camera.target.y + radius * np.sin(pitch)
        camera.position.z = camera.target.z + radius * np.cos(pitch) * np.cos(yaw)

    # Przesuwanie kamery PPM
    if rl.is_mouse_button_down(rl.MOUSE_BUTTON_RIGHT) and rl.is_cursor_on_screen():
        dx = -mouse_delta.x * pan_speed
        dy = mouse_delta.y * pan_speed

        # Ruch lewo/prawo
        move_right = rl.vector3_scale(right, dx)
        # Ruch góra/dół
        move_up = rl.vector3_scale(camera.up, dy)

        move_total = rl.vector3_add(move_right, move_up)

        camera.position = rl.vector3_add(camera.position, move_total)
        camera.target = rl.vector3_add(camera.target, move_total)

    # Zoom (opcjonalnie: kółko myszy)
    wheel = rl.get_mouse_wheel_move()
    if wheel != 0:
        zoom_amount = wheel * 0.5
        forward = rl.vector3_subtract(camera.target, camera.position)
        forward = rl.vector3_normalize(forward)
        zoom_vector = rl.vector3_scale(forward, zoom_amount)
        camera.position = rl.vector3_add(camera.position, zoom_vector)

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
            show_position_gui = not show_position_gui
            input_str['x'] = ''
            input_str['y'] = ''
            input_str['z'] = ''
            input_field = 'x'
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
            show_position_gui = not show_position_gui
            input_str['x'] = ''
            input_str['y'] = ''
            input_str['z'] = ''
            input_field = 'x'

    # Aktualizacja ruchu robota
    robot.update_motion()
    # Rysowanie
    rl.begin_drawing()

    rl.clear_background(rl.RAYWHITE)

    if show_position_gui:
        gui_x, gui_y = 20, 200

        rl.draw_rectangle(gui_x - 10, gui_y - 10, 400, 250, rl.fade(rl.LIGHTGRAY, 0.9))
        rl.draw_text("Set grabber position", gui_x, gui_y, 20, rl.DARKGRAY)
        rl.draw_text("TAB - next coordinate", gui_x , gui_y+20, 20, rl.DARKGRAY)
        # rl.draw_text("BACKSPACE - clear last", gui_x, gui_y + 40, 20, rl.DARKGRAY)
        rl.draw_text(position_gui_message, gui_x, gui_y + 20, 20, rl.RED)
        # Pola tekstowe
        for i, axis in enumerate(['x', 'y', 'z']):
            y_offset = gui_y + 40 + i * 50
            color = rl.RED if input_field == axis else rl.BLACK
            rl.draw_text(f"{axis.upper()}: {input_str[axis]}", gui_x, y_offset, 30, color)
            rl.draw_rectangle(gui_x, y_offset + 30, 280, 2, rl.GRAY)

        # Przycisk PRZENIEŚ
        button_bounds = rl.Rectangle(gui_x, gui_y + 200, 200, 30)
        rl.draw_rectangle_rec(button_bounds, rl.DARKGREEN)
        rl.draw_text("ENTER - MOVE", int(gui_x + 10), int(gui_y + 205), 20, rl.WHITE)

        if rl.is_mouse_button_pressed(rl.MOUSE_LEFT_BUTTON) and rl.check_collision_point_rec(rl.get_mouse_position(), button_bounds):
            try:
                target_x_gui = float(input_str['x'])
                target_y_gui = float(input_str['y'])
                target_z_gui = float(input_str['z'])
                target = rl.Vector3(target_x_gui, target_y_gui, target_z_gui)
                success = robot.move_to_position(target)
                if success:
                    show_position_gui = False
                    position_gui_message = ""
                else:
                    position_gui_message = "Nie można przenieść ramienia – poza zasięgiem!"
            except ValueError:
                print("Nieprawidłowe wartości XYZ!")

        # Obsługa klawiatury
        key = rl.get_key_pressed()
        if rl.is_key_pressed(rl.KEY_TAB):
            input_field = {'x': 'y', 'y': 'z', 'z': 'x'}[input_field]
        elif rl.is_key_pressed(rl.KEY_ENTER):
            try:
                target_x_gui = float(input_str['x'])
                target_y_gui = float(input_str['y'])
                target_z_gui = float(input_str['z'])
                target = rl.Vector3(target_x_gui, target_y_gui, target_z_gui)
                success = robot.move_to_position(target)
                if success:
                    show_position_gui = False
                    position_gui_message = ""
                else:
                    position_gui_message = "Nie można przenieść ramienia – poza zasięgiem!"
            except ValueError:
                print("Nieprawidłowe wartości XYZ!")
        elif rl.is_key_pressed(rl.KEY_ESCAPE):
            show_position_gui = False
        elif key >= 32 and key <= 125:
            input_str[input_field] += chr(key)
        elif rl.is_key_pressed(rl.KEY_BACKSPACE):
            if len(input_str[input_field]) > 0:
                input_str[input_field] = input_str[input_field][:-1]

            

    rl.begin_mode3d(camera)

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
    rl.end_drawing()

rl.close_window()
