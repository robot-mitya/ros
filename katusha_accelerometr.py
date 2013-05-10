#!/usr/bin/env python
#coding=utf8

# Description: Control robot using another phone accelerometr
# Howto use: http://robotlife.ru/upravlenie-akselerometrom/
# Author: Stepan

from turtlesim.msg import Velocity
from sensor_msgs.msg import Imu
from time import sleep
import serial
import rospy

# Data from accelerometers
acc_x = acc_y = acc_z = 0

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

def moveR(speed=255):
    """Move right weels only. Forward if speed>0, backwards if speed<0"""
    ser.write('R'+dec2hex(speed))

def moveL(speed=255):
    """Move left weels only. Forward if speed>0, backwards if speed<0"""
    ser.write('L'+dec2hex(speed))

def imucallback(msg):
    """callback for 'android/imu' messages from turtlesim. Updateing current positoin of accelerometr"""
    global acc_x, acc_y, acc_z
#    rospy.loginfo(rospy.get_name() + ": I heard %s and %s and %s" % (msg.linear_acceleration.x, msg.linear_acceleration.y,  msg.linear_acceleration.z))
    acc_x = msg.linear_acceleration.x
    acc_y = -msg.linear_acceleration.y
    acc_z = msg.linear_acceleration.z

def listener():
    """Main loop"""
    rospy.init_node('katusha', anonymous=True)
    rospy.Subscriber('android/imu', Imu, imucallback)

    while not rospy.is_shutdown():
        # This way we prevent from sending values to motors which will not move the robot, but only make noise from motors. Also prevent acidental moving when phone in horiosntal position.
        if (abs(acc_y+acc_x)>2) or (abs(acc_y-acc_x)>2):
            moveR((acc_y+acc_x)*255/10)
            moveL((acc_y-acc_x)*255/10)
        else:
            move(0)
        print 'Y:%s' % acc_y
        print 'X:%s' % acc_x
        # read and print data from serial port (bluetooth)
        data = ser.read(9999)
        if len(data)>0:
            print data
        # Move with
        # rospy.sleep(0.1)

if __name__ == '__main__':
    # connect to bluetooth. Make sure you have  connected robot with 
    # rfcomm connect 0 00:13:02:01:70:17 1 
    ser = serial.Serial('/dev/rfcomm0', 9600, timeout=0)
    listener()
    # stop motors before exit
    move(0)
