import raylibpy as rl

class Primitive:
    def __init__(self):
        self.position = rl.Vector3(1.0, 2.0, 0.0)
        self.radius = 0.5

    def draw(self):
        rl.draw_sphere(self.position, self.radius, rl.DARKBLUE)