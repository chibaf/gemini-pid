class pid_controller():
  def __init__(self,target,kp0,ki0,kd0,dt0):
    self.kp=kp0
    self.ki=ki0
    self.kd=kd0
    self.dt=dt0
    self.target_temp=target
  def pid(self,temp,previous_error,integral):
    error = self.target_temp - temp
    integral += error * self.dt
    derivative = (error - previous_error) / self.dt
    control = self.kp * error + self.ki * integral + self.kd * derivative
    return control, error, integral
    
def pf(uf,time):
    import numpy as np
    x=np.mod(time,1800.)
    ur=np.heaviside(-uf,0)*np.heaviside(x,0)*np.heaviside(1500.-x,0)
    return int(ur)

    dt = 1 # Time step
    #
    pv1 = 0  # Initial process variable
    kp1 = 1.0  # Proportional gain
    ki1 = 0.0  # Integral gain
    kd1 = 0.0  # Derivative gain
    previous_error1 = 10
    integral1 = 0
    pid1=pid_controller(TARGET_TEMP1,kp1,ki1,kd1,dt)
    
TARGET_TEMP1 = 44.0 # motor temperature in Celsius
TARGET_TEMP2 = -18.0 # freezer temperature in Celsius

        pv1=temp[0]
        control1, error1, integral1 = pid1.pid(pv1,previous_error1,integral1)
#        print(control, error, integral)
        pv1 += control1 * dt  # Update process variable based on control output (simplified)
        previous_error1=error1 #error
        print(str(error1)+": "+str(i))
        uf1=pf(-control1,now-time0)
