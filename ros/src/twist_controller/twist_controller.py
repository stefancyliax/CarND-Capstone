from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy # for testing purposes

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0, self.max_lat_accel, max_steer_angle)
        self.pid_controller = PID(1, 0.1, 0.1)
        # wheel_base, steer_ratio, 0, max_lat_accel, max_steer_angle)
        pass

    def control(self, planned_linear_velocity, planned_angular_velocity, current_linear_velocity, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        steering_cmd = self.yaw_controller.get_steering(planned_linear_velocity, planned_angular_velocity, current_linear_velocity)

        velocity_error = planned_linear_velocity - current_linear_velocity
        acceleration = self.pid_controller.step(velocity_error, dt)
        rospy.logwarn('calulated acceleration: %3.2f, current velocity: %3.2f', acceleration, current_linear_velocity)

        if acceleration > 0:
            # accelerate! Set throttle to positive value and brake to zero
            throttle = 1.0  ## from 0 to 1
            brake = 0.0
            pass

        else:
            # brake! 
            throttle = 0.0
            brake = 1.0 ## from 0 to 20000 Nm
            pass

        return throttle, brake, steering_cmd
    
    def reset_pid(self):
        self.pid_controller.reset()

