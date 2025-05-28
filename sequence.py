import copy

class SequenceManager:
    def __init__(self, robot):
        self.robot = robot
        self.frames = []
        self.play_index = 0

    def record_frame(self):
        self.frames.append(copy.deepcopy(self.robot.joint_angles))

    def playback(self):
        if not self.frames:
            return
        if self.play_index >= len(self.frames):
            self.play_index = 0
        self.robot.joint_angles = copy.deepcopy(self.frames[self.play_index])
        self.play_index += 1
