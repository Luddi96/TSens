from tkinter import *
import tkinter as tk
import tkinter.scrolledtext as st
import socket
from datetime import datetime

############################
#Insert Rx and Tx info here#
############################

UDP_TX_IP = "192.168.178.93"
UDP_TX_PORT = 8888

UDP_RX_IP = "192.168.178.19"
UDP_RX_PORT = 8888

LperMM = 7500.0/1050.0

#generate Rx socket
rxSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rxSock.bind((UDP_RX_IP, UDP_RX_PORT))

txSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

root = tk.Tk()
root.title("Water Sens Control V1.1")
text_area = st.ScrolledText(root, 
                            width = 40,  
                            height = 8,  
                            font = ("Arial", 
                                    15))
  
#text_area.grid(column = 0, pady = 10, padx = 10) 

# Inserting Text which is read only 
text_area.insert(tk.INSERT, """Using Rx IP """)
text_area.insert(tk.INSERT, UDP_RX_IP)
text_area.insert(tk.INSERT, """, Port """)
text_area.insert(tk.INSERT, UDP_RX_PORT)
text_area.insert(tk.INSERT, """\nUsing Tx IP """)
text_area.insert(tk.INSERT, UDP_TX_IP)
text_area.insert(tk.INSERT, """, Port """)
text_area.insert(tk.INSERT, UDP_TX_PORT)
text_area.insert(tk.INSERT, """\nSystem Ready!\n\n""")

pumpstate = tk.StringVar()
pumpstate.set("Pump unknown")
pstat = tk.Label(root, textvariable=pumpstate, font = ("Arial", 15))
def changeTextOn():
    pumpstate.set("Pump on")
    pstat.config(bg="green")
def changeTextOff():
    pumpstate.set("Pump off")
    pstat.config(bg="red")
def sendGet():
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    text_area.insert(tk.INSERT, current_time)
    text_area.insert(tk.INSERT, """: Send GetLast\n""")
    hex_dig = "4F54545202001234"
    hex_arr = bytearray.fromhex(hex_dig)
    txSock.sendto(hex_arr, (UDP_TX_IP, UDP_TX_PORT))
def sendMeas():
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    text_area.insert(tk.INSERT, current_time)
    text_area.insert(tk.INSERT, """: Send TrigMeas\n""")
    hex_dig = "4F54545203001234"
    hex_arr = bytearray.fromhex(hex_dig)
    txSock.sendto(hex_arr, (UDP_TX_IP, UDP_TX_PORT))
def sendMGet():
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    text_area.insert(tk.INSERT, current_time)
    text_area.insert(tk.INSERT, """: Send Trig&Get\n""")
    hex_dig = "4F54545204001234"
    hex_arr = bytearray.fromhex(hex_dig)
    txSock.sendto(hex_arr, (UDP_TX_IP, UDP_TX_PORT))


pstat.pack()
text_area.pack()
button1 = tk.Button(root, text='GetLast', width=12, command=sendGet)
button1.pack(padx=9, pady=10, side=LEFT)
button2 = tk.Button(root, text='TrigMeas', width=12, command=sendMeas)
button2.pack(padx=9, pady=10, side=LEFT)
button3 = tk.Button(root, text='Trig&Get', width=12, command=sendMGet)
button3.pack(padx=9, pady=10, side=LEFT)
button4 = tk.Button(root, text='Quit', width=12, command=root.destroy)
button4.pack(padx=9, pady=10, side=LEFT)
#text_area.config(state=DISABLED)

#root.mainloop()

rxSock.settimeout(0.1)

while True:
    root.update_idletasks()
    root.update()
    text_area.see("end")
    try:
        data, addr = rxSock.recvfrom(1024) # buffer size is 1024 bytes
        if data[0] == 0x4F and data[1] == 0x54 and data[2] == 0x54 and data[3] == 0x52 and data[5] == 0x01:
            if data[4] == 0x02:
                water = data[11] | (data[10] << 8) | (data[9] << 16)
                water /= 100.0
                now = datetime.now()
                current_time = now.strftime("%H:%M:%S")
                text_area.insert(tk.INSERT, current_time)
                text_area.insert(tk.INSERT, """: Recieve Get: """)
                text_area.insert(tk.INSERT, water)
                text_area.insert(tk.INSERT, """ mm, """)
                text_area.insert(tk.INSERT, water*LperMM)
                text_area.insert(tk.INSERT, """ L\n""")
            if data[4] == 0x03:
                now = datetime.now()
                current_time = now.strftime("%H:%M:%S")
                text_area.insert(tk.INSERT, current_time)
                text_area.insert(tk.INSERT, """: Recieve Meas done.\n""")
            if data[4] == 0x04:
                water = data[11] | (data[10] << 8) | (data[9] << 16)
                water /= 100.0
                now = datetime.now()
                current_time = now.strftime("%H:%M:%S")
                text_area.insert(tk.INSERT, current_time)
                text_area.insert(tk.INSERT, """: Recieve Trig&Get: """)
                text_area.insert(tk.INSERT, water)
                text_area.insert(tk.INSERT, """ mm, """)
                text_area.insert(tk.INSERT, water*LperMM)
                text_area.insert(tk.INSERT, """ L\n""")
        if data[0] == 0x4F and data[1] == 0x54 and data[2] == 0x54 and data[3] == 0x52 and data[5] == 0x00:    
            if data[4] == 0x05:
                if data[8] == 0x01:
                    now = datetime.now()
                    current_time = now.strftime("%H:%M:%S")
                    text_area.insert(tk.INSERT, current_time)
                    text_area.insert(tk.INSERT, """: Recieve Pump On.\n""")
                    changeTextOn()
                if data[8] == 0x02:
                    now = datetime.now()
                    current_time = now.strftime("%H:%M:%S")
                    text_area.insert(tk.INSERT, current_time)
                    text_area.insert(tk.INSERT, """: Recieve Pump Off.\n""")
                    changeTextOff()
    except:
        pass