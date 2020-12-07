import time
from RF24 import * #lib for NRF Module
import RPi.GPIO as GPIO #lib for Pin / Interrupt

GPIO.setmode(GPIO.BCM) #set Pin Modes
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP) #init IRQ pin

radio = RF24(22, 0); #init NRF module
payload_size = 32 #max size
radio.begin() #construct NRF module
radio.maskIRQ(1,0,0) #enable Rx interrupt only
radio.startListening() #listen

rx = "0KEVN" #server adress
tx = "1KEVN" #static sensor adress (needs to be made dynamic)

rxb = bytearray(rx, 'utf-8') #transfer to byte arr
txb = bytearray(tx, 'utf-8') #trandfer to byte arr

radio.openWritingPipe(txb) #open tx pipe (needs to be opened and closed when densor adress changes
radio.openReadingPipe(1,rxb) #open rx pipe (can be static)



def rxInt(channel): #interrupt handler
    print("Interrupted") #debug output
    tout = 0
    while (not radio.available()) and tout < 10: #wait for data ready
        time.sleep(0.01)
        tout += 1
    while radio.available(): #read all data and decide
        recieve_payload = radio.read(32)
        print(recieve_payload)


GPIO.add_event_detect(18, GPIO.FALLING, callback = rxInt) #add interrupt to pin

import signal #sleep
signal.pause()