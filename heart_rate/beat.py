import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)
lock = 0

beats=0

previousTime = 15
bpm = 0
startTime = time.time()

def elapsedTime():
  return time.time()-startTime

while True:
  if(elapsedTime()>=previousTime):
    startTime = time.time()
    bpm = beats*4
    beats=0
    print("BPM: "+str(bpm))

  if(GPIO.input(17)==0 and lock==0):
    beats+=1
    lock=1
  elif(GPIO.input(17)==1 and lock==1):
    lock=0
