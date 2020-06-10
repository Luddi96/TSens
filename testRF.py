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

rx = "5ABRE" #server adress
tx = "0ABRE" #static sensor adress (needs to be made dynamic)

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
        recieve_payload = radio.read(32).decode('utf-8').rstrip('\x00')
        if recieve_payload[0] == "u":
            sendText("No Update")
        if recieve_payload[0] == "i":
            sendText("ID:23,Addr:12,Ts:60")
        if recieve_payload[0] == "g":
            sendText("ID:"+recieve_payload[1]+"Addr:12,Ts:60")
        if recieve_payload[0] == "d":
            sendText("Recieved: "+recieve_payload)


GPIO.add_event_detect(18, GPIO.FALLING, callback = rxInt) #add interrupt to pin

def sendText(txt): #send data to sensor
    radio.stopListening()
    time.sleep(0.2)
    arr = bytearray(txt, 'utf-8')
    radio.write(arr[:payload_size])
    radio.startListening()

import signal #sleep
signal.pause()
