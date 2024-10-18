

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0  # Previous error to compute derivative

    def compute_control(self, mission):
        """Compute PD control for each reference and measured height in the mission."""
        control_signals = []  # List to store control signals for each entry
        previous_error = 0  # Initialize the previous error as 0

        # Loop through all the reference and height values from the mission object
        for ref, height in zip(mission.reference, mission.cave_height):
            error = ref - height  # Proportional term (P)
            error_rate = error - previous_error  # Derivative term (D), change in error

            # PD control formula: u = kp * error + kd * error_rate
            control_signal = self.kp * error + self.kd * error_rate
            control_signals.append(control_signal)  # Append the control signal to the list

            previous_error = error  # Update previous error for the next iteration

        return control_signals