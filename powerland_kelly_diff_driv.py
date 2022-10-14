#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import RPi.GPIO as GPIO
import time
from math import pi

import Adafruit_MCP4725

dac1 = Adafruit_MCP4725.MCP4725()
dac1 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)

dac2 = Adafruit_MCP4725.MCP4725()
dac2 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=4)

     #   Red
rev_sw_r = 27
fwd_sw_r = 22
brk_sw_r = 26
rev_sw_l = 5
fwd_sw_l = 6
brk_sw_l = 25

right_fwd=False
left_fwd=False
right_rev=False
left_rev=False

#motor_rpm = 60              #   max rpm of motor on full voltage 
r_wheel = 5.08      
wheel_dist = 17

default_vel=1
max_vel=4096
min_vel=0
#max_pwm_val = 100.00           
#min_pwm_val = 30.00           

#wheel_radius = wheel_diameter/2
#circumference_of_wheel = 2 * pi * wheel_radius
#max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(rev_sw_r, GPIO.OUT)
GPIO.setup(fwd_sw_r, GPIO.OUT)
GPIO.setup(brk_sw_r, GPIO.OUT)
GPIO.setup(rev_sw_l, GPIO.OUT)
GPIO.setup(fwd_sw_l, GPIO.OUT)
GPIO.setup(brk_sw_l, GPIO.OUT)


dac1.set_voltage(4096)
dac1.set_voltage(0)

dac2.set_voltage(4096)
dac2.set_voltage(0)

GPIO.output(fwd_sw_r, right_fwd)
GPIO.output(fwd_sw_l, left_fwd)
GPIO.output(brk_sw_r, GPIO.HIGH)
GPIO.output(rev_sw_r, right_rev)
GPIO.output(rev_sw_l, left_rev)
GPIO.output(brk_sw_l, GPIO.HIGH)

def setvoltage(a,b):
    print (a)
    print(b)
    dac1.set_voltage(a)
    dac2.set_voltage(b)


def Movestop():
    GPIO.output(fwd_sw_r, GPIO.LOW)
    GPIO.output(fwd_sw_l, GPIO.LOW)
    GPIO.output(brk_sw_r, GPIO.HIGH)
    GPIO.output(rev_sw_r, GPIO.LOW)
    GPIO.output(rev_sw_l, GPIO.LOW)
    GPIO.output(brk_sw_l, GPIO.HIGH)
    setvoltage(0,0)


def callback(data):
    global r_wheel
    global wheel_dist
    
    x = data.linear.x                  # Linear Velocity of Robot
    z_rotation = data.angular.z
    
    right_cmd = ((2*x)+(z_rotation*wheel_dist))/2*r_wheel
    left_cmd = ((2*x)-(z_rotation*wheel_dist))/2*r_wheel
    
    right_dir_fwd= right_fwd if (right_cmd>0) else (not right_fwd);
    left_dir_fwd= left_fwd if (left_cmd>0) else (not left_fwd);
    right_dir_rev = not right_dir_fwd
    left_dir_rev = not left_dir_fwd
    
    GPIO.output(fwd_sw_r, right_dir_fwd)
    GPIO.output(fwd_sw_l, left_dir_fwd)
    GPIO.output(rev_sw_r, right_dir_rev)
    GPIO.output(rev_sw_l, left_dir_rev)
    
    right_write=int(default_vel * right_cmd);
    left_write=int(default_vel * left_cmd);
    
    if (x == 0 and z_rotation ==0):
        Movestop();
    
    else:
        GPIO.output(brk_sw_r, GPIO.LOW)
        GPIO.output(brk_sw_l, GPIO.LOW)
        
    
    abs_left_write= abs(left_write)
    abs_right_write= abs(right_write)
    
    abs_left_write= max(min(abs_left_write,max_vel),min_vel)
    abs_right_write= max(min(abs_right_write,max_vel),min_vel)
    
    setvoltage(abs_left_write,abs_right_write)



def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('started')
    listener()