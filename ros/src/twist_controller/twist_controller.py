
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.yaw_controller = YawController(kwargs['wheel_base'], 
                                            kwargs['steer_ratio'], 
                                            1,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

        # these need to be tuned
        self.pid_steering = PID(1, 0., 0.)
        self.pid_throttle = PID(1, 0., 0., 0, 1)

        self.dbw_enabled = False
        

    def control(self, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular, dbw_enabled):
        # handle safety driver
        if not self.dbw_enabled and dbw_enabled:
            self.pid_steering.reset()

        self.dbw_enabled = dbw_enabled

        if not self.dbw_enabled:
            return 0., 0., 0.

        # control steering
        steer = self.control_steering(req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular)

        # control velocity
        throttle, brake = self.control_velocity(req_vel_linear, cur_vel_linear) 
        
        return throttle, brake, steer

    def control_steering(self, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular):
        cur_steer = self.yaw_controller.get_steering(cur_vel_linear, cur_vel_angular, cur_vel_linear)
        new_steer = self.yaw_controller.get_steering(req_vel_linear, req_vel_angular, cur_vel_linear)

        return self.pid_steering.step(new_steer - cur_steer, 1)

    def control_velocity(self, req_vel_linear, cur_vel_linear):
        throttle = self.pid_throttle.step(req_vel_linear - cur_vel_linear, 1)
        brake = 0
        return throttle, brake

