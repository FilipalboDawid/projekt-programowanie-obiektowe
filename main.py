import raylibpy as rl
from robot import RobotArm
from primitives import Primitive
from sequence import SequenceManager
import math

WIDTH, HEIGHT = 1600, 900

yaw = 0.0
pitch = 0.0
radius = 5.0

rl.init_window(WIDTH, HEIGHT, b"3D Robot Arm Simulation")
rl.set_target_fps(60)

camera = rl.Camera3D()
camera.position = rl.Vector3(5.0, 5.0, 5.0)
camera.target = rl.Vector3(0.0, 0.0, 0.0)
camera.up = rl.Vector3(0.0, 1.0, 0.0)
camera.fovy = 45.0
camera.projection = rl.CAMERA_PERSPECTIVE

def distance_vec3(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return (dx*dx + dy*dy + dz*dz) ** 0.5

robot = RobotArm()
primitive = Primitive()
seq_manager = SequenceManager(robot)

mode = 'teach'

while not rl.window_should_close():
    if rl.is_key_down(rl.KEY_T):
        mode = 'teach'
    if rl.is_key_down(rl.KEY_P):
        mode = 'play'

    mouse_delta = rl.get_mouse_delta()
    sensitivity = 0.003
    pan_speed = 0.01

    # Wektor forward (kierunek patrzenia kamery)
    forward = rl.vector3_subtract(camera.target, camera.position)
    forward = rl.vector3_normalize(forward)

    # Wektor right (prostopadły do forward i up)
    right = rl.vector3_cross_product(forward, camera.up)
    right = rl.vector3_normalize(right)

    # Obrót kamery PPM
    if rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT) and rl.is_cursor_on_screen():
        yaw += mouse_delta.x * sensitivity
        pitch -= mouse_delta.y * sensitivity
        pitch = max(-math.pi/2 + 0.1, min(math.pi/2 - 0.1, pitch))

        # Oblicz pozycję kamery względem celu
        camera.position.x = camera.target.x + radius * math.cos(pitch) * math.sin(yaw)
        camera.position.y = camera.target.y + radius * math.sin(pitch)
        camera.position.z = camera.target.z + radius * math.cos(pitch) * math.cos(yaw)

    # Przesuwanie kamery LPM
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

    if mode == 'teach':
        robot.handle_input()
        seq_manager.record_frame()
        if rl.is_key_pressed(rl.KEY_G):
            end_pos = robot.get_end_effector_pos()
            obj_pos = primitive.position

            if distance_vec3(end_pos, obj_pos) < primitive.radius:
                robot.grasp(primitive)
            else:
                print("Za daleko od obiektu, nie można chwycić.")
        if rl.is_key_pressed(rl.KEY_R):
            robot.release()
        if rl.is_key_down(rl.KEY_W):
            robot.joint_angles[0] += 0.01
        if rl.is_key_down(rl.KEY_S):
            robot.joint_angles[0] -= 0.01
        if rl.is_key_down(rl.KEY_A):
            robot.joint_angles[1] += 0.01
        if rl.is_key_down(rl.KEY_D):
            robot.joint_angles[1] -= 0.01
        if rl.is_key_down(rl.KEY_UP):
            robot.joint_angles[2] += 0.01
        if rl.is_key_down(rl.KEY_DOWN):
            robot.joint_angles[2] -= 0.01
    elif mode == 'play':
        seq_manager.playback()

    # Rysowanie
    rl.begin_drawing()
    rl.clear_background(rl.RAYWHITE)

    rl.begin_mode3d(camera)
    robot.draw()
    primitive.draw()
    rl.draw_grid(10, 1.0)
    rl.end_mode3d()
    
    rl.draw_text(f"Mode: {mode.upper()} (T/P) | G: Grab | R: Release", 10, 10, 20, rl.DARKGRAY)
    rl.end_drawing()

rl.close_window()