import Adafruit_MCP4725
import RPi.GPIO as GPIO
import time
from math import pi

#dac = Adafruit_MCP4725.MCP4725(address=0x60)
dac1 = Adafruit_MCP4725.MCP4725()
dac1 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)

dac2 = Adafruit_MCP4725.MCP4725()
dac2 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=4)

dac1.set_voltage(4096)
dac1.set_voltage(0)
max_pwm_val_dac1=100
min_pwm_val_dac1=0

while True:

    for x in range(0,4097,150):

        

        print(x)

        dac1.set_voltage(x)
        dac2.set_voltage(x)
        #dac1.set_voltage(4096)
        #dac1.set_voltage(0)

        voltage = x/4096.0*5.0
        print (voltage)

        time.sleep(2)
