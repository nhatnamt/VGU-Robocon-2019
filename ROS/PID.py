#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

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


lasterror=0
baseSpeed=100
kP=1
kD=0
dirL=0
dirR=0
pwmL=0
pwmR=0

def PIDcalc(data):
    global lasterror, pwmL, pwmR, dirL, dirR
    error = data.data
    deltaSpeed = kP*error+kD*(error-lasterror)
    lasterror=error
    pwmL=baseSpeed+deltaSpeed
    pwmR=baseSpeed-deltaSpeed
    
    if (pwmL<0): 
        dirL=2
        pwmL=-pwmL
    else:
        pwmL=min(255,pwmL)
        dirL=1

    if (pwmR<0):
        dirR=2
        pwmR=-pwmR
    else:
        pwmR=min(255,pwmR)
        dirR=1




serconnect()
rospy.init_node('PID', anonymous=True)
rospy.Subscriber('fine_path', Float32, PIDcalc)
#CW = 1, CCW = 2
sig = "<" + str(dirL) + "," + str(pwmL) + "," + str(dirR) + "," + str(pwmR) + ">"
sendSignal(sig)

rospy.spin()