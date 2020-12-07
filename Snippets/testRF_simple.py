import time
import RPi.GPIO as GPIO #lib for Pin / Interrupt

GPIO.setmode(GPIO.BCM) #set Pin Modes
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP) #init IRQ pin


def rxInt(channel): #interrupt handler
    print("Interrupted") #debug output

GPIO.add_event_detect(18, GPIO.FALLING, callback = rxInt) #add interrupt to pin

import signal #sleep
signal.pause()