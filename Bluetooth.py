#!/usr/bin/env python3

import json
from math import *
import time
import serial

def serconnect():
    #Chọn Baudrate và Port kết nối
    global ser
    port = "COM22"
    b_rate = 38400
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
    port = "COM22"
    b_rate = 38400
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


#Sử dụng công thức Pythagoras
def calculateDistance(autoIndex_c, element_c):
	x_auto = data[autoIndex_c]["position"][0]
	y_auto = data[autoIndex_c]["position"][1]
	x_element = element_c["position"][0]
	y_element = element_c["position"][1]
	return sqrt( (x_auto - x_element)**2 + (y_auto - y_element)**2 )

#Sắp xếp các phần tử dữ liệu với các đặc tính tương ứng
def bubbleSort(array, indexArray_b, objectSocreRanking_b):
	for count in range(len(array)):
		for count_1 in range(len(array) - count - 1):
			if (array[count + count_1 + 1] < array[count]):
				#Sắp xếp theo khoảng cách
				temporary = array[count]
				array[count] = array[count + count_1 + 1]
				array[count + count_1 + 1] = temporary
				
				#Cập nhập điểm
				temporary = objectSocreRanking_b[count]
				objectSocreRanking_b[count] = objectSocreRanking_b[count + count_1 + 1]
				objectSocreRanking_b[count + count_1 + 1] = temporary
				
				#Cập nhập index
				temporary = indexArray_b[count]
				indexArray_b[count] = indexArray_b[count + count_1 + 1]
				indexArray_b[count + count_1 + 1] = temporary

#S#!/usr/bin/env python3

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

