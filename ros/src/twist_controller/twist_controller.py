#!/usr/bin/env python

from pid import PID
from yaw_controller import YawController
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

	#  Calculating the steer angle and acceleration by the YawController and PID controller
    def __init__(self, vehicle_mass, fuel_capacity, acceleration_limit, deceleration_limit,
                 wheel_base, wheel_radius, steer_ratio, max_lat_accel, max_steer_angle, min_speed):

        self.deceleration_limit = deceleration_limit
        self.velocity_controller = PID(1.5, 0.01, 0., deceleration_limit, acceleration_limit)
        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    #  Getting the velocity (current and required) and time elapsed from last call
	#  Calculating the velocity correction (acceleration/deceleration) by PID
	#  Return the acceleration (throttle), brake (deceleration) and steer angle 
    def control(self, twist_cmd, current_velocity, time_elapsed):
        vel_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        throttle = self.velocity_controller.step(vel_error, time_elapsed)
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z, current_velocity.twist.linear.x)

        if current_velocity.twist.linear.x < 0.1 and np.isclose(twist_cmd.twist.linear.x, 0.):
            torque = self.total_mass * self.wheel_radius * self.deceleration_limit
            return 0., torque, steer
        elif throttle > 0:
            return throttle, 0., steer
        else:
            torque = self.total_mass * self.wheel_radius * -throttle
            return 0., torque, steer

    def reset(self):
        self.velocity_controller.reset()