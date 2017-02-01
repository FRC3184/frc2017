from wpilib.powerdistributionpanel import PowerDistributionPanel


_pdp = PowerDistributionPanel()


class PWMMotor:
    """
    Abstraction for PWM motor controllers

    Future: integrated feedback control
    """
    def __init__(self, clazz, pwm_port, pdp_port):
        self.controller = clazz(pwm_port)
        self.pdp_port = pdp_port

    def get_current(self):
        return _pdp.getCurrent(self.pdp_port)

    def set(self, power):
        self.controller.set(power)
