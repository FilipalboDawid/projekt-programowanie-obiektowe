import raylibpy as rl
import math

# Pomocnicze funkcje wektorowe
def vec3_add(v1, v2):
    return rl.Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

def vec3_sub(v1, v2):
    return rl.Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)

def vec3_scale(v, scalar):
    return rl.Vector3(v.x * scalar, v.y * scalar, v.z * scalar)

def vec3_length(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

def vec3_normalize(v):
    length = vec3_length(v)
    if length == 0:
        return rl.Vector3(0, 0, 0)
    return rl.Vector3(v.x / length, v.y / length, v.z / length)

def rotation_x(angle):
    return [
        [1, 0, 0],
        [0, math.cos(angle), -math.sin(angle)],
        [0, math.sin(angle), math.cos(angle)]
    ]

def rotation_y(angle):
    return [
        [math.cos(angle), 0, math.sin(angle)],
        [0, 1, 0],
        [-math.sin(angle), 0, math.cos(angle)]
    ]

def rotation_z(angle):
    return [
        [math.cos(angle), -math.sin(angle), 0],
        [math.sin(angle), math.cos(angle), 0],
        [0, 0, 1]
    ]

def apply_rotation(vec, mat):
    x = vec.x * mat[0][0] + vec.y * mat[0][1] + vec.z * mat[0][2]
    y = vec.x * mat[1][0] + vec.y * mat[1][1] + vec.z * mat[1][2]
    z = vec.x * mat[2][0] + vec.y * mat[2][1] + vec.z * mat[2][2]
    return rl.Vector3(x, y, z)

class RobotArm:
    def __init__(self):
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]  # base, shoulder, elbow, wrist
        self.segment_length = 1.0
        self.grabbing = False
        self.grabbed_object = None
        self.target = None

    def handle_input(self):
        if rl.is_key_down(rl.KEY_ONE): self.joint_angles[0] += 0.02  # Base Z
        if rl.is_key_down(rl.KEY_TWO): self.joint_angles[0] -= 0.02
        if rl.is_key_down(rl.KEY_THREE): self.joint_angles[1] += 0.02  # Shoulder X
        if rl.is_key_down(rl.KEY_FOUR): self.joint_angles[1] -= 0.02
        if rl.is_key_down(rl.KEY_FIVE): self.joint_angles[2] += 0.02  # Elbow X
        if rl.is_key_down(rl.KEY_SIX): self.joint_angles[2] -= 0.02
        if rl.is_key_down(rl.KEY_SEVEN): self.joint_angles[3] += 0.02  # Wrist Y
        if rl.is_key_down(rl.KEY_EIGHT): self.joint_angles[3] -= 0.02

        if rl.is_key_pressed(rl.KEY_M):
            # Przykładowy punkt docelowy do przemieszczenia
            self.target = rl.Vector3(1.0, 1.5, 1.0)
            self.move_to(self.target)

    def draw(self):
        base = rl.Vector3(0, 0, 0)
        up = rl.Vector3(0, 1, 0)

        rot_base = rotation_z(self.joint_angles[0])
        base_dir = apply_rotation(up, rot_base)
        joint1 = vec3_add(base, vec3_scale(base_dir, self.segment_length))

        rot_shoulder = rotation_x(self.joint_angles[1])
        shoulder_dir = apply_rotation(up, rot_shoulder)
        shoulder_dir = apply_rotation(shoulder_dir, rot_base)
        joint2 = vec3_add(joint1, vec3_scale(shoulder_dir, self.segment_length))

        rot_elbow = rotation_x(self.joint_angles[2])
        elbow_dir = apply_rotation(up, rot_elbow)
        elbow_dir = apply_rotation(elbow_dir, rot_base)
        joint3 = vec3_add(joint2, vec3_scale(elbow_dir, self.segment_length))

        rot_wrist = rotation_y(self.joint_angles[3])
        wrist_dir = apply_rotation(up, rot_wrist)
        wrist_dir = apply_rotation(wrist_dir, rot_base)
        end = vec3_add(joint3, vec3_scale(wrist_dir, self.segment_length))

        rl.draw_line3d(base, joint1, rl.RED)
        rl.draw_line3d(joint1, joint2, rl.GREEN)
        rl.draw_line3d(joint2, joint3, rl.BLUE)
        rl.draw_line3d(joint3, end, rl.PURPLE)

        if self.grabbing and self.grabbed_object:
            self.grabbed_object.position = end

    def get_end_effector_pos(self):
        base = rl.Vector3(0, 0, 0)
        up = rl.Vector3(0, 1, 0)

        rot_base = rotation_z(self.joint_angles[0])
        base_dir = apply_rotation(up, rot_base)
        joint1 = vec3_add(base, vec3_scale(base_dir, self.segment_length))

        rot_shoulder = rotation_x(self.joint_angles[1])
        shoulder_dir = apply_rotation(up, rot_shoulder)
        shoulder_dir = apply_rotation(shoulder_dir, rot_base)
        joint2 = vec3_add(joint1, vec3_scale(shoulder_dir, self.segment_length))

        rot_elbow = rotation_x(self.joint_angles[2])
        elbow_dir = apply_rotation(up, rot_elbow)
        elbow_dir = apply_rotation(elbow_dir, rot_base)
        joint3 = vec3_add(joint2, vec3_scale(elbow_dir, self.segment_length))

        rot_wrist = rotation_y(self.joint_angles[3])
        wrist_dir = apply_rotation(up, rot_wrist)
        wrist_dir = apply_rotation(wrist_dir, rot_base)
        end = vec3_add(joint3, vec3_scale(wrist_dir, self.segment_length))

        return end

    def move_to(self, target):
        dx = target.x
        dz = target.z
        dy = target.y

        base_angle = math.atan2(dx, dz)
        self.joint_angles[0] = base_angle

        r = math.sqrt(dx**2 + dz**2)
        d = math.sqrt(r**2 + dy**2)
        d = min(d, 2 * self.segment_length)

        try:
            angle2 = math.acos((2 * self.segment_length**2 - d**2) / (2 * self.segment_length**2))
        except ValueError:
            angle2 = 0

        angle1 = math.atan2(dy, r) - angle2 / 2

        self.joint_angles[1] = angle1
        self.joint_angles[2] = angle2

    def grasp(self, primitive):
        self.grabbing = True
        self.grabbed_object = primitive

    def release(self):
        self.grabbing = False
        self.grabbed_object = None
