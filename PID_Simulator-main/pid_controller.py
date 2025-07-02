
import numpy as np

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.01):
        self.Kp = Kp
        self.Ki = Ki  
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.error_history = []
        self.output_history = []
        
    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.error_history.append(error)
        
        
        Parpotional = self.Kp * error
        
    
        if len(self.error_history) == 1:
            prev_error = 0
        else:
            prev_error = self.error_history[-2]
        Deffrential = self.Kd * ((error - prev_error) / self.dt)
        
        
        self.integral += error * self.dt  
        Intgral = self.Ki * self.integral
        
        total_output = Parpotional + Intgral + Deffrential
        self.output_history.append(total_output)
        return total_output
    
    def reset(self):
        self.integral = 0
        self.error_history = []
        self.output_history = []
        
    def set_gains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
    def get_pid_terms(self):
        if len(self.error_history) == 0:
            return {'proportional': 0, 'integral': 0, 'derivative': 0, 'total_output': 0}
        
       
        error = self.error_history[-1]
        
      
        Parpotional = self.Kp * error
        
        
        if len(self.error_history) == 1:
            prev_error = 0
        else:
            prev_error = self.error_history[-2]
        Deffrential = self.Kd * ((error - prev_error) / self.dt)
        
        
        Intgral = self.Ki * self.integral
        
        return {'proportional': Parpotional, 'integral': Intgral, 'derivative': Deffrential, 'total_output': Parpotional + Intgral + Deffrential}
     
    def get_gains(self):
        return {'Kp': self.Kp, 'Ki': self.Ki, 'Kd': self.Kd}
