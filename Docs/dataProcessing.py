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

#Sử dụng toán vector
def returnControl(point_1, point_2, vector):
    #Vector từ điểm hai đến điểm 1
    x_axis = point_1[0] - point_2[0]
    y_axis = point_1[1] - point_2[1]

    #Tính tích vô hướng của hai vector
    lengthVector1 = sqrt(x_axis**2 + y_axis**2)
    lengthVector2 = sqrt(vector[0]**2 + vector[1]**2)
    dotProduct = x_axis*vector[0] + y_axis*vector[1]

    #Trong trường hợp không có Object
    if (lengthVector1*lengthVector2 == 0):
        cosOfAngle = 1
        print("vector multiplication = 0")
    #Trong trường hợp có ít nhất 1 object
    else:
        cosOfAngle = dotProduct / (lengthVector1*lengthVector2)

    print("Cos of Angle:" + str(cosOfAngle))

    if (cosOfAngle > -0.9):
        print ("Rotate")
        return 1
    else:
        print ("Go straight")
        return 2

#Xử lý và gửi tín hiệu
def main(dataRaw, previousDistance):
    global data
    global ser
    """ ************************ Nhận dữ liệu ************************** """
    data = dataRaw["data"]
    """
    Trong trường hợp này, chỉ có một con Robot tự động, auto_1 và 1 Object, object_1 trên sân.
    20 là số index của auto_1
    0 là số index của object_1
    """
    
    data = [data[0], data[20]]
    print(data)

    #Khởi tạo
    objectScore = [5, 0] #Tương ứng object_1 và auto_1
    objectScoreRanking = [1, 2]
    autoIndex = 1
    storageZone = [150, 20]
  
    distanceArray = []
    indexArray = []
    scoreArray = []

    #Tính toán khoảng cách và đưa vào array
    for count in range(len(data)):
            distanceArray.append(calculateDistance(autoIndex, data[count]))
            indexArray.append(count)

    #Sắp xếp array của khoảng cách
    bubbleSort(distanceArray, indexArray, objectScoreRanking)

    #Tính toán điểm các object trên sân
    for count in range(len(data)):
	    #kiếm index của auto_1
            if (distanceArray[count] != 0):
                    scoreArray.append(objectScoreRanking[count] + count)
            else:
                    scoreArray.append(100000000)

    #xem xét lựa chọn tốt nhất (trong trường hợp này là ít điểm xếp hạng nhất)
    selectionScore = min(scoreArray)
    selectionIndex = 0

    #Lấy index của lựa chọn tốt nhất trong array
    for count in range(len(scoreArray)):
            if selectionScore == scoreArray[count]:
                    selectionIndex = indexArray[count]

    currentDistance = calculateDistance(autoIndex, data[selectionIndex])
    print ("Distance to the selected object: " + str(currentDistance))

    if (abs(currentDistance - previousDistance) > 5):
            #Hướng về object
            signal = returnControl(data[autoIndex]["position"], data[selectionIndex]["position"] ,data[autoIndex]["dimension"])
            print (str(signal) + " To selected object")
    elif(currentDistance < 95 and abs(currentDistance - previousDistance) <= 5): 
            #Hướng về storage zone
            signal = returnControl(data[autoIndex]["position"], storageZone ,data[autoIndex]["dimension"])
            print (str(signal) + " To storage zone")


    previousDistance = currentDistance
    #Gửi tín hiệu cho Robot
    sendSignal(signal)
    return previousDistance

