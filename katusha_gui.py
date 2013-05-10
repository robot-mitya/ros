#!/usr/bin/env python
#coding=utf8

# Description: Main gui interface to control Robot with mouse
# Howto use: work in progress
# Author: Stepan

from Tkinter import *
import Image, ImageTk
from StringIO import StringIO
from urllib import urlopen
import time
import threading
import serial
import rospy
import re

exit = False
thread_running = False
i=0
j=0

THREAD_DEBUG = False

# Format of robot message in Regular Expression
COMMAND_FORMAT = r'[^0-9a-fA-F][0-9a-fA-F]{4}'

# Buffer for store total reseived data from robot
MessageBuffer = ""

def dec2hex(n):
    """return the hexademical string in XXXX format"""
    return ("%0.4X" % (n>=0 and n or n+256**2))[-4:]
# the mystery here n>=0 actually goes to n+256*2 and returns 10000 in hex. but it's cuted to 0000 by [-4:0]


def sensorsOff():
    ser.write('='+dec2hex(0x0000)) # Timer 1
    ser.write('='+dec2hex(0x1000)) # Timer 1

def sensorsOn():
    #print "value: ", value
    ser.write('='+dec2hex(0x0064)) # Timer 0 (R1)
    ser.write('='+dec2hex(0x1064)) # Timer 1 (R2)

def headH(pos=255):
    """rotate horizontal head servo"""
    ser.write('H'+dec2hex(pos))

def headV(pos=255):
    """rotate vertical head servo"""
    if pos<35: pos=35
    ser.write('V'+dec2hex(pos))

def move(speed=255):
    """move foreward, or backward if speed<0"""
    ser.write('G'+dec2hex(speed))

def rotate(speed=255):
    """rotate clockwise if speed<0, or anti-clockwise if speed>0"""
    ser.write('R'+dec2hex(speed)+'L'+dec2hex(-speed))

def loadImage():
    url = "http://192.168.0.90:8080/shot.jpg"
    try:
        im = Image.open(StringIO(urlopen(url).read()))
        #im = Image.open("/home/a/Pictures/st573.jpg")
    except IOError:
        print "Жопа!"
        im = Image.open("/home/a/Pictures/test.jpg")
    return ImageTk.PhotoImage(im)
    #f = open('test.jpg','wb')
    #f.write(tkimage)
    #f.close()

def updateImage():
    global exit, thread_running
    i=0
    thread_running = True
#    exit = True
    while not exit:
        if THREAD_DEBUG: print "looping"

        url = "http://192.168.0.90:8080/shot.jpg"
        try:
            im = Image.open(StringIO(urlopen(url).read()))
        except IOError:
            print "Жопа!"
            im = Image.open("/home/a/Pictures/test.jpg")
#        tkimage = loadImage()
        if not exit:
            if THREAD_DEBUG: print "PhotoImage started"
            tkimage = ImageTk.PhotoImage(im)
            if THREAD_DEBUG: print "PhotoImage stoped"
            try:
                image_label.config(image=tkimage)
                image_label.image = tkimage
                i+=1
                if THREAD_DEBUG: print "New frame:", i
            except:
                print "Can't assign picture. May be exited?"
        else:
            print "exited!!!!!!"

    print "exited from updateImage"
    thread_running = False

def execCommand(cmd, value=0):
    if cmd=='~':
        timer = value>>12
        data = (value & 0x0FFF)*10
        if timer==0:
            label_r1.config(text='R1='+str(data/1000.0))
        elif timer==1:
            label_r2.config(text='R2='+str(data/1000.0))
        else:
            print "unknown timer: ", timer
        print cmd, value


def processMessageBuffer():
    """ Read from buffer and Search for commands """
    global MessageBuffer
    data = ser.read(9999)
    #print "New data:", data
    if len(data)!=0:
        MessageBuffer+=data
        commands = re.findall(COMMAND_FORMAT, MessageBuffer)
        for cmd in commands:
            print cmd
            execCommand(cmd[0], int(cmd[1:5], 16))
        MessageBuffer = MessageBuffer[-4:]
    root.after(100, processMessageBuffer)


def mouseMove(event):
    posH = 255-event.x*255/640
    posV = event.y*255/480
    #print "MouseX: ",event.x," posH:",posH, " MouseY: ",event.y," posV:",posV
    headH(posH)
    headV(posV)

def printEventInfo(event):
    items = event.__dict__.items()
    items.sort()
    for k, v in items:
        print "%s: %s\n" % (k, v)

def mouseButtonPress(event):
    #print "MouseButPress"
    speed = 255
    if event.num == 1:
        move(speed)
    elif event.num == 3:
        move(-speed)
    elif event.num == 4:
        rotate(-speed)
    elif event.num == 5:
        rotate(speed)
    #printEventInfo(event)

def mouseButtonRelease(event):
    #print "MouseButRelease"
    move(0)
    #printEventInfo(event)

def safeExit():
    global exit
    print "safeExit"
    exit = True
    move(0)
    sensorsOff()
    while thread_running:
        pass

def keyPress(event):
    if event.keycode==9:
        safeExit()
        root.destroy()
#    print "keyPressed"
#    printEventInfo(event)

def destroy(event):
    safeExit()
    print "destroy"
#    printEventInfo(event)


ser = serial.Serial('/dev/rfcomm0', 9600, timeout=0)

data = ser.read(9999)
print data
print "started"
sensorsOn()

root = Tk()
root.title("Панель управления роботом")
root.geometry("%dx%d+%d+%d" % (800, 600, 100, 100))  # width*height x y
frame = Frame(root, bg='black')
frame.pack(fill='both', expand='yes')

tkimage = loadImage()
image_label = Label(frame, image=tkimage)
image_label.image = tkimage
image_label.place(x=0, y=0)

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
label_r1 = Label(frame, text="R1=NO DATA")
label_r1.place(x=CAMERA_WIDTH+10, y=10)

label_r2 = Label(frame, text="R2=NO DATA")
label_r2.place(x=CAMERA_WIDTH+10, y=30)

#updateImage()
img_update = threading.Thread(target=updateImage)
img_update.start()
#image_label.after(100, updateImage)
root.after(100, processMessageBuffer)
image_label.bind("<Motion>", mouseMove)
image_label.bind("<ButtonPress>", mouseButtonPress)
image_label.bind("<ButtonRelease>", mouseButtonRelease)
#image_label.bind("<KeyPress>", keyPress)
root.bind("<KeyPress>", keyPress)
root.bind("<Destroy>", destroy)
root.mainloop()


#def Hello(event):
#    print "Yet another hello world"
#
#btn = Button(root,                  #родительское окно
#    text="Click me",       #надпись на кнопке
#    width=30,height=5,     #ширина и высота
#    bg="black",fg="white") #цвет фона и надписи
#btn.bind("<Button-1>", Hello)       #при нажатии ЛКМ на кнопку вызывается функция Hello
#btn.pack()                          #расположить кнопку на главном окне
#root.mainloop()
#tk = Tk(); f = Frame(); f.pack()
#time_var = StringVar()
#time_label = Label(f, textvariable=time_var, font="Courier 60",
#                   bg="Black", fg="#00B000")
#time_label.pack()
#
#def tick():
#  t = time.localtime(time.time())
#  if t[5] % 2:
#    fmt = "%H:%M :)"
#  else:
#    fmt = "%H %M"
#  time_var.set(time.strftime(fmt, t))
#  time_label.after(500, tick)
#
#time_label.after(500, tick)
#tk.mainloop()
#
