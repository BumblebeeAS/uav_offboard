from dataclasses import dataclass
from typing import Dict


@dataclass
class OffboardMode:
    """
    Encapsulates the current offboard control mode.
    """

    is_position: bool = True
    is_velocity: bool = False
    is_acceleration: bool = False
    is_attitude: bool = False
    is_body_rate: bool = False
    is_thrust_and_torque: bool = False
    is_direct_actuator: bool = False

    def set_mode(
        self,
        position: bool,
        velocity: bool,
        acceleration: bool,
        attitude: bool,
        body_rate: bool,
        thrust_and_torque: bool,
        direct_actuator: bool,
    ) -> None:
        """
        Set the offboard control mode.

        Raises:
            ValueError: If not exactly one control mode is set to True.
        """
        num_set = (
            position
            + velocity
            + acceleration
            + attitude
            + body_rate
            + thrust_and_torque
            + direct_actuator
        )

        if num_set != 1:
            raise ValueError("Exactly one control mode must be set to True.")

        self.is_position = position
        self.is_velocity = velocity
        self.is_acceleration = acceleration
        self.is_attitude = attitude
        self.is_body_rate = body_rate
        self.is_thrust_and_torque = thrust_and_torque
        self.is_direct_actuator = direct_actuator

    def get_mode(self) -> Dict[str, bool]:
        """
        Get the current offboard control mode.
        """
        return {
            "position": self.is_position,
            "velocity": self.is_velocity,
            "acceleration": self.is_acceleration,
            "attitude": self.is_attitude,
            "body_rate": self.is_body_rate,
            "thrust_and_torque": self.is_thrust_and_torque,
            "direct_actuator": self.is_direct_actuator,
        }
