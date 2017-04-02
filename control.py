import threading

import robot_time


class RPMController:
    def __init__(self, target_speed, max_speed, kP, source, output):
        self.enabled = False
        self.target_speed = target_speed
        self.source = source
        self.output = output
        self.max_speed = max_speed
        self.kP = kP

    def run(self):
        while True:
            if self.enabled:
                err = self.error()
                power = self.target_speed / self.max_speed
                power += self.kP * err
                self.output(power)
            robot_time.sleep(millis=10)

    def error(self):
        return self.target_speed - self.source()

    def start(self):
        control_thread = threading.Thread(target=self.run)
        control_thread.start()
        return control_thread
