import raylibpy as rl

class Primitive:
    def __init__(self):
        self.position = rl.Vector3(0.0, 0.5, -1.0)
        self.radius = 0.5

    def draw_shadow(self, light_direction):
        """
        Rysuje cień kuli na płaszczyźnie na podstawie kierunku światła.
        :param light_direction: Kierunek światła (Vector3).
        """
        shadow_position = rl.Vector3(
            self.position.x - light_direction.x * self.position.y,
            0.0,  # Cień na płaszczyźnie
            self.position.z - light_direction.z * self.position.y
        )
        rl.draw_circle(int(shadow_position.x * 100), int(shadow_position.z * 100), self.radius * 100, rl.GRAY)

    def draw(self):
        rl.draw_sphere(self.position, self.radius, rl.DARKBLUE)