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

        :param initial: The initial point.
        :param output_range: The output range within which the PID must stabilize.
        If provided as a tuple (low, high), the output range must never go lesser than
        the lower value and higher than the upper value.

    """
    def __init__(self,
        kp = 1.0,
        ki = 0.0,
        kd = 0.0,
        initial = 0,
        output_range = None,
        *args, **kwargs):
        self._super = super(PIDController, self)
        self._super.__init__(*args, **kwargs)

        self._kp = kp
        self._ki = ki
        self._kd = kd

        # The initial value for the controller...
        self._initial       = initial
        self._output_range  = output_range

        self._previous_time = None

        self.reset()

    def __call__(self, value):
        """
            Perform an update to the PID controller given the provided value

            Example

                >>> from rosutils.controller import PIDController
                >>> pid = PIDController(1, 0, 0)
                >>> next_value = pid(2)
        """
        # get the current timestamp
        timestamp = rostime.now()

        # difference in time
        d_time   = self._previous_time - timestamp

        # difference in input
        d_input = value - (self._previous_input if self._previous_input is not None else value)
        error   = self._initial - value

        self._proportional   = self.kp * error
        self._integral      += self.ki * error * d_input

        self._derivative    -= self.kd * (d_input / d_time)

        # calculate the PID function
        output = self._proportional + self._integral + self._derivative

        # update the current to previous time for next iteration
        self._previous_time  = timestamp
        # update the current to previous time for next iteration
        self._previous_input = value

        return output

    @property
    def components(self):
        """
            Provide the computed values of the P, I and D terms.
        """
        return (self._proportional, self._integral, self._derivative)

    # return the string representation of this object.
    def __repr__(self):
        return "<PID kp=%s ki=%s kd=%s>" % (self._kp, self._ki, self._kd)

    def reset(self):
        """
            Reset the PID to an initial state.
        """
        self._proportional  = 0
        self._derivative    = 0
        self._integral      = 0