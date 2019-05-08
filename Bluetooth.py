#!/usr/bin/env python3

import json
from math import *
import time
import serial

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

def sendSignal(input):
    global ser
    port = "/dev/rfcomm0"
    b_rate = 115200
    while(True):
        """ Xử lý mất kết nối giữa trận """
        try:
            if(ser == None):
                ser = serial.Serial(port, baudrate = b_rate, timeout=1, write_timeout = 1)
                print("Reconnecting")
            #Gửi dữ liệu tới Robot
            ser.write(str(input).encode())
            time.sleep(0.2)
            #Đọc dữ liệu (nếu có)
            data = ser.readline().decode()
            print("Writing Data...")
            print("Data: "+ data)
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
    sendSignal(1)
    time.sleep(1)
    sendSignal(0)
    time.sleep(1)

