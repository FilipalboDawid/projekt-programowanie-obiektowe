import raylibpy as rl
import math

def vec3_add(v1, v2):
    return rl.Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

def vec3_scale(v, scalar):
    return rl.Vector3(v.x * scalar, v.y * scalar, v.z * scalar)

def vec3_scale(v, scalar):
    return rl.Vector3(v.x * scalar, v.y * scalar, v.z * scalar)

class RobotArm:
    def __init__(self):
        self.joint_angles = [0.0, 0.0, 0.0]  # 3 DOF
        self.segment_length = 1.0
        self.grabbing = False
        self.grabbed_object = None

    def handle_input(self):
        if rl.is_key_down(rl.KEY_ONE): self.joint_angles[0] += 0.02
        if rl.is_key_down(rl.KEY_TWO): self.joint_angles[0] -= 0.02
        if rl.is_key_down(rl.KEY_THREE): self.joint_angles[1] += 0.02
        if rl.is_key_down(rl.KEY_FOUR): self.joint_angles[1] -= 0.02
        if rl.is_key_down(rl.KEY_FIVE): self.joint_angles[2] += 0.02
        if rl.is_key_down(rl.KEY_SIX): self.joint_angles[2] -= 0.02

    def draw(self):
        base = rl.Vector3(0, 0, 0)
        rot1 = rl.Vector3(math.cos(self.joint_angles[0]), 0, math.sin(self.joint_angles[0]))
        joint1 = vec3_add(base, vec3_scale(rot1, self.segment_length))

        rot2 = rl.Vector3(math.cos(self.joint_angles[1]), math.sin(self.joint_angles[1]), 0)
        joint2 = vec3_add(joint1, vec3_scale(rot2, self.segment_length))

        rot3 = rl.Vector3(0, math.cos(self.joint_angles[2]), math.sin(self.joint_angles[2]))
        end = vec3_add(joint2, vec3_scale(rot3, self.segment_length))

        rl.draw_line3d(base, joint1, rl.RED)
        rl.draw_line3d(joint1, joint2, rl.GREEN)
        rl.draw_line3d(joint2, end, rl.BLUE)

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = end

    def get_end_effector_pos(self):
        base = rl.Vector3(0, 0, 0)

        rot1 = rl.Vector3(math.cos(self.joint_angles[0]), 0, math.sin(self.joint_angles[0]))
        joint1 = vec3_add(base, vec3_scale(rot1, self.segment_length))

        rot2 = rl.Vector3(math.cos(self.joint_angles[1]), math.sin(self.joint_angles[1]), 0)
        joint2 = vec3_add(joint1, vec3_scale(rot2, self.segment_length))

        rot3 = rl.Vector3(0, math.cos(self.joint_angles[2]), math.sin(self.joint_angles[2]))
        end = vec3_add(joint2, vec3_scale(rot3, self.segment_length))

        return end

    def grasp(self, primitive):
        self.grabbing = True
        self.grabbed_object = primitive

    def release(self):
        self.grabbing = False
        self.grabbed_object = None