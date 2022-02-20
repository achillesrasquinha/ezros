import rosutils.util.time as rostime
from rosutils.controllers.base import BaseController

# This controller is heavily inspired by
# https://github.com/m-lundberg/simple-pid

class PIDController(BaseController):
    """
        Create a PID Controller

        :param kp: Proportional Gain
        :param ki: Integral Gain
        :param kd: Derivative Gain
    """
    def __init__(self,
        kp = 1.0,
        ki = 0.0,
        kd = 0.0,
        initial = 0,
        *args, **kwargs):
        self._super = super(PIDController, self)
        self._super.__init__(*args, **kwargs)

        self.kp = kp
        self.ki = ki
        self.kd = kd

        # The initial value for the controller...
        self._initial       = initial

        self._previous_time = None

    def __call__(self, system):
        now     = rostime.now()

        dt      = self._previous_time - now
        di      = system - self._previous_input

        error   = self._set_point - system

        self._proportional   = self.kp * error
        self._integral      += self.ki * error * dt

        self._derivative     = self.kd * (di / dt)

        self._previous_time  = now
        self._previous_input = system

        output = self._proportional + self._integral + self._derivative

        return output

    @property
    def components(self):
        """
            Provide the computed values of the P, I and D terms.
        """
        return (self._proportional, self._integral, self._derivative)

    def __call__(self):
        pass