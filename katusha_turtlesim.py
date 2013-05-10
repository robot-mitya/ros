#!/usr/bin/env python
#coding=utf8

# Description: Control robot using ROS turtle_teleop_key from tutorial
#              just for fun and practice using ROS
# Howto use: http://robotlife.ru/upravlenie-ros-turtlesim/
# Author: Stepan

from turtlesim.msg import Velocity
from time import sleep
import serial
import rospy

def dec2hex(n):
    """return the hexademical string in XXXX format"""
    return ("%0.4X" % (n>=0 and n or n+256**2))[-4:]
# the mystery here n>=0 actually goes to n+256*2 and returns 10000 in hex. but it's cuted to 0000 by [-4:0]

def move(speed=255):
    """move foreward, or backward if speed<0"""
    ser.write('G'+dec2hex(speed))

def rotate(speed=255):
    """rotate clockwise if speed<0, or anti-clockwise if speed>0"""
    ser.write('R'+dec2hex(speed)+'L'+dec2hex(-speed))

def turtlesim_cmd(msg):
    """callback for 'turtle1/command_velocity' messages from turtlesim"""
    # Logs
    #rospy.loginfo(rospy.get_name() + ": I heard %s and %s" % (msg.linear, msg.angular))
    # Determine the direction to move
    if msg.linear+msg.angular>0:
        speed=255
    else:
        speed=-255
    # Go straight or rotate
    if msg.linear!=0:
        move(speed)
        sleep(abs(msg.linear/100))
    else:
        rotate(speed)
        sleep(abs(msg.angular/100))
    # Stop
    move(0)

def listener():
    rospy.init_node('katusha', anonymous=True)
    # Listen for messages from turtlesim
    rospy.Subscriber('turtle1/command_velocity', Velocity, turtlesim_cmd)

    while not rospy.is_shutdown():
        # read and print data from serial port (bluetooth)
        data = ser.read(9999)
        if len(data)>0:
            print data
    #rospy.spin()

if __name__ == '__main__':
    # connect to bluetooth. Make sure you have  connected robot with 
    # rfcomm connect 0 00:13:02:01:70:17 1 
    ser = serial.Serial('/dev/rfcomm0', 9600, timeout=0)
    listener()
    # stop before exit
    move(0)
