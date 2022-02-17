import rosutils.util.time as rostime
from rosutils.controllers.base import BaseController

class PIDController(BaseController):
    """
        Create a PID Controller

        :param kp: Proportional Gain
        :param ki: Integral Gain
        :param kd: 
    """
    def __init__(self,
        kp = 1.0,
        ki = 0.0,
        kd = 0.0,
        *args, **kwargs):
        self._super = super(PIDController, self)
        self._super.__init__(*args, **kwargs)

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._set_point     = 0

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