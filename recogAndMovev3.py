# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
from gpiozero import Servo
import RPi.GPIO as GPIO
import math
import pygame
from pygame.mixer import Sound
import os
from time import sleep

#function definitions
def playCible():
    soundObj=pygame.mixer.Sound("cible.wav")
    soundObj.play()
    sleep(1)
    soundObj.stop()

def playPew():
    soundObj=pygame.mixer.Sound("piou.wav")
    soundObj.play()
    sleep(1)
    soundObj.stop()

def relay_act():
    relay_on(channel)
    time.sleep(0.5)
    relay_off(channel)
    time.sleep(0.1)
    playPew()

def relay_on(pin):
    GPIO.output(pin, GPIO.HIGH)  # Turn relay on
    
def relay_off(pin):
    GPIO.output(pin, GPIO.LOW)  # Turn relay off

def moveServoX(s):
    if  (servoX.value - s >= -1) and (servoX.value + s < 1):
        servoX.value += s
def moveServoY(s):
    if  (servoY.value - s >= -0.-38) and (servoY.value + s < -0.15):
        servoY.value += s

#initialize PyGame
pygame.init()
pygame.mixer.init()
print(os.getcwd())

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
width = 640
height = 480
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

#initialise servo motors
servoX = Servo(pin = 22,
              min_pulse_width=0.5/1000,
              max_pulse_width=2.5/1000)

servoY = Servo(pin = 27,
              min_pulse_width=0.5/1000,
              max_pulse_width=2.5/1000)
servoX.value = 0
servoY.value = -0.28

#initialize relay (for water pump)
channel = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.OUT)

#initialize face recognition cascade
filename = 'haarcascade_frontalface_alt.xml'
face_cascade = cv.CascadeClassifier()
face_cascade.load(filename)

#Initialize variables
faceFound = False
i = 0
targetX = 0
targetY = 0
faceW = 0
faceH = 0
countPew = 0
countNoFace = 0
threshold = 30

i = 0

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    print("Frame: " + str(i))
    i += 1
    #prep frame for cascade analysis
    image = frame.array
    frame_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)

    #detect faces
    faces = face_cascade.detectMultiScale(frame_gray,
                                          scaleFactor = 1.05,
                                          minNeighbors = 5,
                                          minSize = (70,70),
                                          maxSize = (300,300))
    
    faceW = 0

    #grab references of detected face
    for (x, y, w, h) in faces:
        targetX = x + w/2 - width/2
        targetY = y + h/2 - height/2
        faceW = w
        faceH = h
        break

    if faceW != 0:
        #if a face has been detected
        countPew += 1
        if faceFound == False:
            playCible()
            faceFound = True
        #move motor in X direction
        if targetX < -threshold/2:
            moveServoX(0.05)
        elif targetX > threshold/2:
            moveServoX(-0.05)
        #move motor in Y direction
        if targetY < -threshold/2:
            moveServoY(-0.01)
        elif targetY > threshold/2:
            moveServoY(0.01)
        if countPew == 5:
            playPew()
            relay_act()
            countPew = 0
    else:
        countNoFace += 1
        countPew = 0
        
        if countNoFace == 3:
            faceFound = False
            countNoFace = 0
        
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    #print(play_sound)

camera.close()
