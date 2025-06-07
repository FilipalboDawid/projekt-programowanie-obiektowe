#primitives.py

# Importy
import raylibpy as rl

# Klasa Primitive
class Primitive:
    # Inicjalizacja
    def __init__(self):
        self.position = rl.Vector3(1.5, 0.5, 0.0)
        self.radius = 0.5

    # Rysowanie kuli
    def draw(self):
        rl.draw_sphere(self.position, self.radius, rl.DARKBLUE)