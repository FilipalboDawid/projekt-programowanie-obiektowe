#sequence.py

# Importy
import copy

# Klasa SequenceManager
class SequenceManager:
    # Inicjalizcja
    def __init__(self, robot, primitive):
        self.robot = robot
        self.primitive = primitive
        self.frames = []
        self.play_index = 0
    # Nagrywanie ruchów
    def record_frame(self):
        self.frames.append({
        'angles': copy.deepcopy(self.robot.joint_angles),
        'grabbing': self.robot.grabbing,
        'grabbed_object': self.robot.grabbed_object,
        'primitive_position': copy.deepcopy(self.primitive.position)
        })
    # Odtwarzanie nagranych ruchów
    def playback(self):
        if not self.frames:
            return
        if self.play_index >= len(self.frames):
            self.play_index = 0

        frame = self.frames[self.play_index]
        self.robot.joint_angles = copy.deepcopy(frame['angles'])
        self.robot.grabbing = frame['grabbing']

        # Ustaw chwytany obiekt tylko jeśli "grabbing" jest True
        if self.robot.grabbing:
            self.robot.grasp(self.primitive)
            self.primitive.position = self.robot.get_end_effector_pos()
        else:
            self.robot.release()
            self.primitive.position = copy.deepcopy(frame['primitive_position'])


        frame = self.frames[self.play_index]
        self.robot.joint_angles = copy.deepcopy(frame['angles'])
        self.robot.grabbing = frame['grabbing']
        self.robot.grabbed_object = frame['grabbed_object']
        self.primitive.position = copy.deepcopy(frame['primitive_position'])
        self.play_index += 1
