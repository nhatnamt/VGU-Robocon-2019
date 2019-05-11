#!/usr/bin/env python3
import json
from math import *
import time
import serial

CW = 1
CWW = 2
def serconnect():
    #Chọn Baudrate và Port kết nối
    global ser
    port = "/dev/rfcomm0"
    b_rate = 115200
    #Kết nối Robot thông qua thư viện PySerial
    while True:
            try:
                print("Connecting...")
                ser = serial.Serial(port, baudrate = b_rate, timeout=1, write_timeout = 1)
                break
            #Trong trường hợp không kết nối được
            except serial.SerialException:
                print("No Connection, no serial port found")
                time.sleep(1)
    #Thời gian chờ khởi động kết nối
    time.sleep(3)

def sendSignal(sendStr):
    global ser
    port = "/dev/rfcomm0"
    b_rate = 115200
    while(True):
        try:
            if(ser == None):
                ser = serial.Serial(port, baudrate = b_rate, timeout=1, write_timeout = 1)
                print("Reconnecting")
            #Gửi dữ liệu tới Robot
            ser.write(sendStr.encode('utf-8')) 
            break
        #Giải quyết việc mất kết nối
        except serial.SerialTimeoutException:
            if(not(ser == None)):
                ser.close()
                ser = None
                print("Write time out")
            print("No Connection, can't write")
            time.sleep(1)
        except serial.SerialException:
            if(not(ser == None)):
                ser.close()
                ser = None
                print("Serial disconnected")
            print("No Connection, no serial port found")
            time.sleep(1)

serconnect()
while True:
    pwmL = 255 
    dirL = 1
    pwmR = 255  
    dirR = 1
    sig = "<" + str(dirL) + "," + str(pwmL) + "," + str(dirR) + "," + str(pwmR) + ">"
    sendSignal(sig)
    print(sig)
    time.sleep(1)
    pwmL = 0 
    dirL = 1
    pwmR = 0  
    dirR = 1
    sig = "<" + str(dirL) + "," + str(pwmL) + "," + str(dirR) + "," + str(pwmR) + ">"
    sendSignal(sig)
    time.sleep(1)

