"""Vehicle geometry and uncalibrated dynamics; world units are cm.

Positive Handle means LEFT, negative Handle means RIGHT.
Body and axle dimensions are user-specified; dynamics remain uncalibrated.
"""
from dataclasses import dataclass

@dataclass(frozen=True)
class VehicleConfig:
    length_cm: float = 28.0
    width_cm: float = 20.0
    wheelbase_cm: float = 17.0
    front_axle_from_front_cm: float = 7.0
    tire_diameter_cm: float = 5.5
    max_steer_deg: float = 32.0
    forward_cm_s: float = 80.0
    reverse_cm_s: float = 40.0
    motor_response_s_inv: float = 5.0
    acceleration_cm_s2: float = 65.0
    braking_cm_s2: float = 100.0
    servo_deg_s: float = 80.0
    sensor_range_cm: float = 250.0
    sensor_noise_std_cm: float = 0.0

    def __post_init__(self):
        import math
        for name, value in self.__dict__.items():
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{name}: a finite number is required')
            if value < 0 or (value == 0 and name != 'sensor_noise_std_cm'):
                raise ValueError(f'{name}: invalid value {value}')
        if self.wheelbase_cm > self.length_cm or self.max_steer_deg >= 80:
            raise ValueError('Invalid wheelbase or steering angle')
        if self.front_axle_from_front_cm + self.wheelbase_cm > self.length_cm:
            raise ValueError('Axles must lie within the body')

    @property
    def front_axle_cm(self):
        """Front axle's forward coordinate relative to the body center."""
        return self.length_cm / 2 - self.front_axle_from_front_cm

    @property
    def rear_axle_cm(self):
        return self.front_axle_cm - self.wheelbase_cm

    def sensor_specs(self):
        # Local coordinates: forward, LEFT. Angles: right positive.
        # HC-SR04: provisional mounts in the front half of the body.
        # RrLh/RrRh retain legacy API names but now point directly sideways.
        return {
            'Fr': (self.length_cm / 2 - 2.0, 0.0, 0.0),
            'FrLh': (self.length_cm / 2 - 1, self.width_cm / 2, -45.0),
            'RrLh': (self.front_axle_cm, self.width_cm / 2, -90.0),
            'FrRh': (self.length_cm / 2 - 1, -self.width_cm / 2, 45.0),
            'RrRh': (self.front_axle_cm, -self.width_cm / 2, 90.0),
        }

STEERING_RIGHT_PWM = 330
STEERING_CENTER_PWM = 435
STEERING_LEFT_PWM = 540
THROTTLE_FORWARD_PWM = 500
THROTTLE_STOPPED_PWM = 375
THROTTLE_REVERSE_PWM = 220
PHYSICS_HZ = 120
CONTROL_HZ = 20
