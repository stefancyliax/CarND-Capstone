from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, vehicle_mass, fuel_capacity, wheel_radius, brake_deadband, decel_limit, accel_limit):
        self.wheel_base = wheel_base 
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel 
        self.max_steer_angle = max_steer_angle 
        self.vehicle_mass = vehicle_mass + GAS_DENSITY*fuel_capacity 
        self.wheel_radius = wheel_radius 
        self.brake_deadband = brake_deadband 
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit 

        # init yaw controller
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)

        # Init PID controller
        kP = 0.5
        kI = 0.0
        kD = 0.0
        self.pid_controller = PID(kP, kI, kD)
        
        # init low pass filter 
        tau = 0.5
        ts = 0.02
        self.velocity_lowpass = LowPassFilter(tau, ts)
        
    def control(self, planned_linear_velocity, planned_angular_velocity, current_linear_velocity, dt):
        # Return throttle, brake, steer
        
        # use yaw controller
        steering_cmd = self.yaw_controller.get_steering(planned_linear_velocity, planned_angular_velocity, current_linear_velocity)

        # use PID controller
        current_linear_velocity = self.velocity_lowpass.filt(current_linear_velocity)
        velocity_error = planned_linear_velocity - current_linear_velocity
        acceleration_cmd = self.pid_controller.step(velocity_error, dt)

        throttle = brake = 0.0

        if planned_linear_velocity == 0. and current_linear_velocity < 0.1:
            # hold vehicle statinary
            throttle = 0.
            brake = 400 # Nm
        elif acceleration_cmd > 0:
            # accelerate! Set throttle to positive value and brake to zero
            throttle = min(acceleration_cmd, self.accel_limit) ## set throttle to 1 above a velocity difference of 5 m/s
            brake = 0.0

        else:
            # brake! 
            throttle = 0.0
            brake = abs(acceleration_cmd*500) ## from 0 to 20000 Nm
            brake_force_limit = self.wheel_radius * self.vehicle_mass * abs(self.decel_limit)
            brake_force_deadband = brake_force_limit * self.brake_deadband
            rospy.logerr('brake before limit: %s', brake)

            brake = min(brake_force_limit, brake)
            # not useful! What is brake_deadbands purpose?
            # if brake < brake_force_deadband: # if brake force is below deadband, then just coast
            #     brake = 0

        rospy.logwarn('current_velocity: %s, planned_velocity: %s', current_linear_velocity, planned_linear_velocity)
        rospy.logwarn('current acceleration_cmd: %s, throttle: %s, brake: %s', acceleration_cmd, throttle, brake)
            
        return throttle, brake, steering_cmd
    
    def reset_pid(self):
        self.pid_controller.reset()

