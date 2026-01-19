import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import date
import matplotlib.pyplot as plt
import numpy as np

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

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
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
    
def main():
    TARGET_TEMP1 = 45.0 # motor temperature in Celsius
    dt = 0.5 # Time step
    pv1 = 0  # Initial process variable
    kp1 = 2.0  # Proportional gain
#    kp1 = 1.0  # Proportional gain
    ki1 = 0.05  # Integral gain
#    ki1 = 0.01  # Integral gain
    kd1 = 0.0  # Derivative gain
    previous_error1 = 10
    integral1 = 0
    pid1=pid_controller(TARGET_TEMP1,kp1,ki1,kd1,dt)
    fn = "MTtemp_" + str(date.today()) + time.strftime("_H%H_M%M_S%S", time.localtime()) + ".csv"
    f=open(fn, 'w', encoding="utf-8")
    start=time.time()
    #
    x=range(0,100)
    y1=[0]*100
    y2=[0]*100
#
    i=0
    ssr18=0
# GPIO Setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(18, GPIO.OUT)
    GPIO.output(18,0) # Ensure freezer starts OFF
    #
    flag=0
    stm=0
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
        pv1=temp[0]
        control1, error1, integral1 = pid1.pid(pv1,previous_error1,integral1)
        pv1 += control1 * dt  # Update process variable based on control output (simplified)
        previous_error1=error1 #error
        print(str(error1)+": "+str(i))
        
        if control1<0 and flag==0:
          stm=time.time()
          GPIO.output(18,0)
          ssr18=0
          flag=1
        elif flag==1 and time.time()-stm<300:
            GPIO.output(18,0)
            ssr18=0
        else:
          GPIO.output(18,1)
          flag=0
          ssr18=1
          
        st = time.strftime("%Y %b %d %H:%M:%S", time.localtime())
        ss = str(time.time() - int(time.time()))
        sss=str(round(time.time()-start,2))
        row=st + ss[1:5] + "," + sss + ","
        row=row+str(temp[0])+","+str(temp[1])+","+str(ssr18)+"\n"
        f.write(row)
        ttl="total time="+str(round(time.time()-start,1))+",ssr18="+str(ssr18)+",temp1="+str(round(temp[0],2))+",temp2="+str(round(temp[1],2))
        plt.clf()
        y1.pop(-1)
        y1.insert(0,temp[0])
        y2.pop(-1)
        y2.insert(0,temp[1])
        plt.ylim(-40,80)
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