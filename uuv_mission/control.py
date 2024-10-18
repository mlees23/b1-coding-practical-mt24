

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0  # Previous error to compute derivative

    def compute_control(self, current_path, reference):
        """Compute PD control for each reference and measured height in the mission."""
                
        error = reference - current_path  # Proportional term (P)
        derivative_error = error - self.previous_error  # Derivative term (D), change in error

        # PD control formula: u = kp * error + kd * error_rate
        control_signal = self.kp * error + self.kd * derivative_error
        
        self.previous_error = error  # Update previous error for the next iteration

        return control_signal