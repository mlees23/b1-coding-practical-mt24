class PDController:
    def __init__(self, kp: float, kd: float):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0.0  # Previous error to compute derivative

    def compute_control(self, reference: float, current_value: float) -> float:
        """Compute the control signal based on reference and current value."""
        # Calculate the current error
        current_error = reference - current_value
        
        # Calculate the derivative of the error
        derivative_error = current_error - self.previous_error
        
        # PD control output
        control_signal = self.kp * current_error + self.kd * derivative_error
        
        # Update previous error
        self.previous_error = current_error
        
        return control_signal
