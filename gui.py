# gui.py

import raylibpy as rl

class PositionGUI:
    def __init__(self, robot):
        self.robot = robot
        self.show = False
        self.message = ""
        self.input_field = 'x'
        self.input_str = {'x': '', 'y': '', 'z': ''}
        self.target = rl.Vector3(0.0, 0.0, 0.0)
        self.just_opened = False

    def toggle(self):
        self.show = not self.show
        self.input_str = {'x': '', 'y': '', 'z': ''}
        self.input_field = 'x'
        self.just_opened = True  # Zignoruj pierwszy klawisz po otwarciu

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
                self.message = "Nie można przenieść ramienia – poza zasięgiem!"
        except ValueError:
            self.message = "Nieprawidłowe wartości XYZ!"

    def draw(self):
        if not self.show:
            return

        gui_x, gui_y = 20, 200
        rl.draw_rectangle(gui_x - 10, gui_y - 10, 400, 250, rl.fade(rl.LIGHTGRAY, 0.9))
        rl.draw_text("Set grabber position", gui_x, gui_y, 20, rl.DARKGRAY)
        rl.draw_text("TAB - next coordinate", gui_x , gui_y+20, 20, rl.DARKGRAY)
        rl.draw_text("L_ALT - close GUI", gui_x, gui_y + 40, 20, rl.DARKGRAY)
        rl.draw_text(self.message, gui_x, gui_y + 60, 20, rl.RED)

        for i, axis in enumerate(['x', 'y', 'z']):
            y_offset = gui_y + 90 + i * 40
            color = rl.RED if self.input_field == axis else rl.BLACK
            rl.draw_text(f"{axis.upper()}: {self.input_str[axis]}", gui_x, y_offset, 30, color)
            rl.draw_rectangle(gui_x, y_offset + 30, 280, 2, rl.GRAY)

        button_bounds = rl.Rectangle(gui_x, gui_y + 200, 200, 30)
        rl.draw_rectangle_rec(button_bounds, rl.DARKGREEN)
        rl.draw_text("ENTER - MOVE", int(gui_x + 10), int(gui_y + 205), 20, rl.WHITE)

        if rl.is_mouse_button_pressed(rl.MOUSE_LEFT_BUTTON) and rl.check_collision_point_rec(rl.get_mouse_position(), button_bounds):
            self.try_move()
