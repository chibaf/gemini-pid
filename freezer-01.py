import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import date
import matplotlib.pyplot as plt
import numpy as np

TARGET_TEMP1 = 44.0 # motor temperature in Celsius
TARGET_TEMP2 = -18.0 # freezer temperature in Celsius
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
PID_CONTROL_DURATION = 1500 # PID control phase duration in seconds (25 minutes)
OFF_DURATION = 300  # Mandatory off phase duration in seconds (5 minutes)

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
def read_temperature():
    """Reads and parses temperature data from the serial port."""
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            parts = line.split(',')
            if parts[0] == '03' and len(parts) > 10:
                try:
                    # data2 is at index 2 of the split list
                    temperature = [float(parts[2]),float(parts[3])]
                    print(temperature)
                    return temperature
                except ValueError:
                    print(f"Bad data format or conversion error in data2: {parts[2]}")
    except Exception as e:
        print(f"Serial reading error: {e}")
        return [0,0]

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
    
def main():
    fn = "PID-M_" + str(date.today()) + time.strftime("_H%H_M%M_S%S", time.localtime()) + ".csv"
    f=open(fn, 'w', encoding="utf-8")
    start=time.time()
    #
    dt = 1 # Time step
    #
    pv1 = 0  # Initial process variable
    kp1 = 1.0  # Proportional gain
    ki1 = 0.0  # Integral gain
    kd1 = 0.0  # Derivative gain
    previous_error1 = 10
    integral1 = 0
    pid1=pid_controller(TARGET_TEMP1,kp1,ki1,kd1,dt)
#    pf1=pf()
    #
    pv2 = 0  # Initial process variable
    kp2 = 1.0  # Proportional gain
    ki2 = 0.0  # Integral gain
    kd2 = 0.0  # Derivative gain
    previous_error2 = 10
    integral2 = 0
    pid2=pid_controller(TARGET_TEMP2,kp2,ki2,kd2,dt)
#    pf2=pf()
    #
    x=range(0,100)
    y1=[0]*100
    y2=[0]*100
#
    i=0
# GPIO Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(18, GPIO.OUT)
    GPIO.output(18,0) # Ensure freezer starts OFF
    #
    time0=time.time()
    while 1:
      try:
        now=time.time()
        temp=read_temperature()
        try:
          if temp[0]==0 and temp[1]==0:
            continue
        except:
          continue
        i=i+1
        time.sleep(dt)
        #
        pv1=temp[0]
        control1, error1, integral1 = pid1.pid(pv1,previous_error1,integral1)
#        print(control, error, integral)
        pv1 += control1 * dt  # Update process variable based on control output (simplified)
        previous_error1=error1 #error
        print(str(error1)+": "+str(i))
        uf1=pf(-control1,now-time0)
        #
        pv2=temp[1]
        control2, error2, integral2 = pid2.pid(pv2,previous_error2,integral2)
#        print(control, error, integral)
        pv2 += control2 * dt  # Update process variable based on control output (simplified)
        previous_error2=error2 #error
        print(str(error2)+": "+str(i))
        uf2=pf(control2,now-time0)
        #
        ssr18=uf1*uf2  # SSR flag
        GPIO.output(18,ssr18)
        st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
        ss = str(time.time() - int(time.time()))
        sss=str(round(time.time()-start,2))
        row=st + ss[1:5] + "," + sss + ","
        row=row+str(temp[0])+","+str(temp[1])+","+str(ssr18)+"\n"
        f.write(row)
        ttl="total time="+str(round(time.time()-start,1))+" time="+str(round(np.mod(time.time()-start,1800.),1))+",ssr18="+str(ssr18)+",temp1="+str(round(temp[0],2))+",temp2="+str(round(temp[1],2))
        plt.clf()
        y1.pop(-1)
        y1.insert(0,temp[0])
        y2.pop(-1)
        y2.insert(0,temp[1])
        plt.ylim(-30,100)
        plt.title(ttl)
        line1,=plt.plot(x,y1,label="1")
        line2,=plt.plot(x,y2,label="2")
        plt.legend(handles=[line1,line2])
        plt.pause(0.1)
      except KeyboardInterrupt:
        print("Program stopped by user. Cleaning up GPIO.")
        ser.close()
        f.close()
        GPIO.output(18,0)
        GPIO.cleanup()

if __name__ == "__main__":
    main()