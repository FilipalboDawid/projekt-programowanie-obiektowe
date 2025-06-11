import raylibpy as rl
import numpy as np
from utils import distance_vec3

class Camera():
    def __init__(self):
        self.camera = rl.Camera3D()
        self.position = rl.Vector3(5.0, 6.0, 5.0)
        self.target = rl.Vector3(0.0, 0.0, 0.0)
        self.up = rl.Vector3(0.0, 1.0, 0.0)
        self.fovy = 45.0
        self.projection = rl.CAMERA_PERSPECTIVE
        self.yaw, self.pitch = self.starting_pitch_and_yaw()
        self.is_rotating = False
        self.start_yaw = self.yaw
        self.start_pitch = self.pitch
        self.mouse_start = rl.Vector2(0, 0)
        self.radius = distance_vec3(self.position, self.target)
    
    def starting_pitch_and_yaw(self):
        # Obliczanie początkowych yaw i pitch na podstawie pozycji kamery
        offset = rl.vector3_subtract(self.position, self.target)
        radius = distance_vec3(self.position, self.target)

        yaw = np.arctan2(offset.x, offset.z)
        pitch = np.arcsin(offset.y / radius)
        return yaw, pitch
    
    def camera_usage(self):
        # Obsługa kamery
        mouse_delta = rl.get_mouse_delta()
        sensitivity = 0.003
        pan_speed = 0.01

        # Wektor forward (kierunek patrzenia kamery)
        forward = rl.vector3_subtract(self.target, self.position)
        forward = rl.vector3_normalize(forward)

        # Wektor right (prostopadły do forward i up)
        right = rl.vector3_cross_product(forward, self.up)
        right = rl.vector3_normalize(right)

        # Obrót kamery LPM
        # Rozpoczęcie rotacji kamery
        if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT) and rl.is_cursor_on_screen():
            self.is_rotating = True
            radius = distance_vec3(self.position, self.target)  # Zapamiętaj aktualny zoom

        # Zakończenie rotacji kamery
        if rl.is_mouse_button_released(rl.MOUSE_BUTTON_LEFT):
            self.is_rotating = False

        # Aktualizacja rotacji kamery
        if self.is_rotating and rl.is_cursor_on_screen():
            self.yaw += mouse_delta.x * sensitivity
            self.pitch -= mouse_delta.y * sensitivity
            self.pitch = max(-np.pi / 2 + 0.1, min(np.pi / 2 - 0.1, self.pitch))

            radius = distance_vec3(self.position, self.target)  # dynamiczne

            # Nowa pozycja kamery na podstawie aktualnego zoomu
            self.position.x = self.target.x + radius * np.cos(self.pitch) * np.sin(self.yaw)
            self.position.y = self.target.y + radius * np.sin(self.pitch)
            self.position.z = self.target.z + radius * np.cos(self.pitch) * np.cos(self.yaw)

        # Przesuwanie kamery PPM
        if rl.is_mouse_button_down(rl.MOUSE_BUTTON_RIGHT) and rl.is_cursor_on_screen():
            dx = -mouse_delta.x * pan_speed
            dy = mouse_delta.y * pan_speed

            # Ruch lewo/prawo
            move_right = rl.vector3_scale(right, dx)
            # Ruch góra/dół
            move_up = rl.vector3_scale(self.up, dy)

            move_total = rl.vector3_add(move_right, move_up)

            self.position = rl.vector3_add(self.position, move_total)
            self.target = rl.vector3_add(self.target, move_total)

        # Zoom (opcjonalnie: kółko myszy)
        wheel = rl.get_mouse_wheel_move()
        if wheel != 0:
            zoom_amount = wheel * 0.5
            forward = rl.vector3_subtract(self.target, self.position)
            forward = rl.vector3_normalize(forward)
            zoom_vector = rl.vector3_scale(forward, zoom_amount)
            self.position = rl.vector3_add(self.position, zoom_vector)

        self.camera.position = self.position
        self.camera.target = self.target
        self.camera.up = self.up
        self.camera.fovy = self.fovy
        self.camera.projection = self.projection
