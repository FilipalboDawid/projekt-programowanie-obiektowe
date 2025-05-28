import raylib as rl

class Joint:
    def __init__(self, position):
        self.position = position  # Vector3
        #self.rotation = rotation  # Vector3 (Euler angles in radians)

    def draw(self):
        rl.draw_sphere(self.position, 0.1, rl.RED)  # Draw joint as a small sphere

class Link:
    def __init__(self, start_joint, end_joint):
        self.start_joint = start_joint  # Joint
        self.end_joint = end_joint  # Joint

    def draw(self):
        rl.draw_line3d(self.start_joint.position, self.end_joint.position, rl.GREEN)  # Draw link as a line

class Claw:
    def __init__(self, position, rotation):
        self.position = position  # Vector3
        self.rotation = rotation  # Vector3 (Euler angles in radians)

    def draw(self):
        rl.draw_cube(self.position, 0.5, 0.1, 0.1, rl.BLUE)  # Draw claw as a cube