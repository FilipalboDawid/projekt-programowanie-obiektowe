# gui.py

import raylibpy as rl
import numpy as np
from primitives import Primitive

primitive = Primitive()

class PositionGUI:
    def __init__(self, robot):
        self.robot = robot
        self.show = False
        self.message = ""
        self.input_field = 'x'
        self.input_str = {'x': '', 'y': '', 'z': ''}
        self.target = rl.Vector3(0.0, 0.0, 0.0)
        self.just_opened = False

        self.info_mode = ""
        self.info_joint_angles = None
        self.info_end_effector_pos = None
        self.info_target_pos = None
        self.info_error_message = ""

    def set_info(self, mode, joint_angles, end_effector_pos, target_pos, error_message):
        self.info_mode = mode
        self.info_joint_angles = joint_angles
        self.info_end_effector_pos = end_effector_pos
        self.info_target_pos = target_pos
        self.info_error_message = error_message

    def draw_info(self):
        base_x = 10
        base_y = 10
        line_spacing = 30
        font_size = 20
        color = rl.DARKGRAY

        lines = [
            f"Mode: {self.info_mode.upper()} (T/P/F) | G: Grab | R: Release | M: Move to (only in Free/Teach)",
            "A,D - Shoulder yaw; W,S - Shoulder pitch, UP, DOWN - Elbow pitch",
            f"Joint Angles: {np.array2string(self.info_joint_angles, precision=1, separator=', ')}",
            f"End Effector Position: ({self.info_end_effector_pos.x:.2f}, {self.info_end_effector_pos.y:.2f}, {self.info_end_effector_pos.z:.2f})",
            f"Target location: ({self.info_target_pos.x:.2f}, {self.info_target_pos.y:.2f}, {self.info_target_pos.z:.2f})",
        ]

        for i, line in enumerate(lines):
            rl.draw_text(line, base_x, base_y + i * line_spacing, font_size, color)

        if self.info_error_message:
            rl.draw_text(self.info_error_message, base_x, base_y + len(lines)*line_spacing, font_size, rl.RED)

        def toggle(self):
            self.show = not self.show
            # Ustaw aktualne współrzędne piłki jako domyślne wartości
            self.input_str = {
                'x': str(primitive.position.x),
                'y': str(primitive.position.y),
                'z': str(primitive.position.z)
            }
            self.input_field = 'x'
            self.just_opened = True # Zignoruj pierwszy klawisz po otwarciu

    def update(self):
        if not self.show:
            return

        if self.just_opened:
            # Zignoruj pierwszy klawisz po otwarciu GUI
            rl.get_key_pressed()
            self.just_opened = False
            return

        key = rl.get_key_pressed()
        if rl.is_key_pressed(rl.KEY_TAB):
            self.input_field = {'x': 'y', 'y': 'z', 'z': 'x'}[self.input_field]
        elif rl.is_key_pressed(rl.KEY_ENTER):
            self.try_move()
        elif rl.is_key_pressed(rl.KEY_LEFT_ALT):
            self.show = False
        elif key >= 32 and key <= 125:
            self.input_str[self.input_field] += chr(key)
        elif rl.is_key_pressed(rl.KEY_BACKSPACE):
            if len(self.input_str[self.input_field]) > 0:
                self.input_str[self.input_field] = self.input_str[self.input_field][:-1]
        elif rl.is_key_pressed(rl.KEY_RIGHT_ALT):
            # Przypisz do input_str aktualne pozycje kuli (primitive.position)
            primitive.position.x = self.info_target_pos.x
            primitive.position.y = self.info_target_pos.y
            primitive.position.z = self.info_target_pos.z
            self.input_str = {
                'x': str(primitive.position.x),
                'y': str(primitive.position.y),
                'z': str(primitive.position.z)
            }
            self.target = rl.Vector3(primitive.position.x, primitive.position.y, primitive.position.z)
            self.message = "Position set to object position."

    def try_move(self):
        try:
            x = float(self.input_str['x'])
            y = float(self.input_str['y'])
            z = float(self.input_str['z'])
            self.target = rl.Vector3(x, y, z)
            success = self.robot.move_to_position(self.target)
            if success:
                self.show = False
                self.message = ""
            else:
                self.message = "Cannot move to this position."
        except ValueError:
            self.message = "Invalid input. Please enter valid numbers."

    def toggle(self):
        self.show = not self.show
        self.input_str = {'x': '', 'y': '', 'z': ''}
        self.input_field = 'x'
        self.just_opened = True  # Zignoruj pierwszy klawisz po otwarciu

    def draw(self):
        if not self.show:
            return

        gui_x, gui_y = 20, 200
        rl.draw_rectangle(gui_x - 10, gui_y - 10, 500, 300, rl.fade(rl.LIGHTGRAY, 0.9))
        rl.draw_text("Set grabber position", gui_x, gui_y, 20, rl.DARKGRAY)
        rl.draw_text("TAB - next coordinate", gui_x , gui_y+20, 20, rl.DARKGRAY)
        rl.draw_text("L_ALT - close GUI", gui_x, gui_y + 40, 20, rl.DARKGRAY)
        rl.draw_text("RIGHT_ALT - set to object position", gui_x, gui_y + 60, 20, rl.DARKGRAY)
        rl.draw_text(self.message, gui_x, gui_y + 80, 20, rl.RED)

        for i, axis in enumerate(['x', 'y', 'z']):
            y_offset = gui_y + 100 + i * 40
            color = rl.RED if self.input_field == axis else rl.BLACK
            rl.draw_text(f"{axis.upper()}: {self.input_str[axis]}", gui_x, y_offset, 30, color)
            rl.draw_rectangle(gui_x, y_offset + 30, 150, 2, rl.GRAY)

        button_bounds = rl.Rectangle(gui_x, gui_y + 250, 200, 30)
        rl.draw_rectangle_rec(button_bounds, rl.DARKGREEN)
        rl.draw_text("ENTER - MOVE", int(gui_x + 10), int(gui_y + 255), 20, rl.WHITE)

        if rl.is_mouse_button_pressed(rl.MOUSE_LEFT_BUTTON) and rl.check_collision_point_rec(rl.get_mouse_position(), button_bounds):
            self.try_move()
