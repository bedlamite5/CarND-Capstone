from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
from math import atan2, sin, cos

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
PVEL = 0.9
IVEL = 0.0005
DVEL = 0.07
PACC = 0.4
IACC = 0.05
DACC = 0.0
TAU = 0.2


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
		rospy.loginfo('TwistController: Start init')
       
	    self.sampling_rate = kwargs["sampling_rate"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]

        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]

		self.brake_deadband = kwargs["brake_deadband"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]
		
		self.delta_t = 1/self.sampling_rate
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity \
            * GAS_DENSITY) * self.wheel_radius

        self.past_vel_linear = 0.0
        self.current_accel = 0.0
        self.low_pass_filter_accel = LowPassFilter(TAU, self.delta_t)
		
		self.pid_vel_linear = PID(PVEL, IVEL, DVEL, self.decel_limit, self.accel_limit)

        self.accel_pid = PID(PACC, IACC, DACC, 0.0, 0.75)

        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=5.0,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)

        rospy.loginfo('TwistController: Complete init')
        rospy.loginfo('TwistController: Steer ratio = ' + str(self.steer_ratio))

    def control(self, linear_vel, angular_vel, current_linear_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
		throttle = 0.
		brake = 0.
		steer = 0.
		
		cte = linear_vel - current_linear_vel
		
		accel = self.sampling_rate * (self.past_vel_linear - current_linear_vel)
		
		self.past_vel_linear = current_linear_vel
		self.lowpass_accel.filt(accel)
		self.current_accel = self.lowpass_accel.get()
		
		accel_des = sef.pid_linvel.step(cte, self.delta_t)

		
		if desired_accel > 0.0:
            if desired_accel < self.accel_limit:
                throttle = self.accel_pid.step(desired_accel - self.current_accel, self.delta_t)
            else:
                throttle = self.accel_pid.step(self.accel_limit - self.current_accel, self.delta_t)
            brake = 0.0
        else:
            throttle = 0.0
            self.accel_pid.reset()
		
            if abs(desired_accel) > self.brake_deadband:

				if abs(desired_accel) > abs(self.decel_limit):
                    brake = abs(self.decel_limit) * self.brake_torque_const
                else:
                    brake = abs(desired_accel) * self.brake_torque_const
					
		steer = self.yaw_controller.get_steer(linear_vel, angular_vel, linear_vel)
		
		if abs(steer) <> 0.0:
            rospy.loginfo('Veer: steer = ' + str(steer) + ', required = ' + str(required_vel_angular))
		
		
        return throttle, brake, steer
		
	def set_controllers(self):
		self.pid_throttle = PID(0.25, 0.0, 0.0, 0.0, 1.0)
		self.pid_brake = PID(0.25, 0.0, 0.0, 0.0, 1.0)
		self.lowpass_steer = PID(0.25, 0.0, 0.0, 0.0, 1.0)
		
    def reset(self):
        self.pid_linvel.reset()
