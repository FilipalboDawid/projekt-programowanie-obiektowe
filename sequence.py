import copy

class SequenceManager:
    def __init__(self, robot):
        self.robot = robot
        self.frames = []
        self.play_index = 0

    def record_frame(self):
        self.frames.append({
        'angles': copy.deepcopy(self.robot.joint_angles),
        'grabbing': self.robot.grabbing,
        'grabbed_object': self.robot.grabbed_object  # tu tylko referencja, nie deep copy!
    })
    def playback(self):
        if not self.frames:
            return
        if self.play_index >= len(self.frames):
            self.play_index = 0

        frame = self.frames[self.play_index]
        self.robot.joint_angles = copy.deepcopy(frame['angles'])
        self.robot.grabbing = frame['grabbing']
        self.robot.grabbed_object = frame['grabbed_object']
        self.play_index += 1

        if self.robot.grabbing and self.robot.grabbed_object:
            self.robot.grabbed_object.position = self.robot.get_end_effector_pos()