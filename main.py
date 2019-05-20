#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import websocket
import json
import ssl
import time
import urllib.request
from pathlib import Path
import math
import matplotlib.animation as animation
from math import *
import serial

CW = 1
CCW = 2

#Vo thi nho sua vi tri
points = [5]*7 + [10]*4 + [15]*3 + [20]*2 + [25]*2 + [0]*4 + [30]*2 + [-20] + [50]*3
colors = ['#bdf300']*7 + ['#5fe73a']*4 + ['#00d06a']*3 + ['#008c8b']*2 + ['#29678f']*2 + ['red']*4 + ['#41408e']*2 + ['k'] + ['#4f1186']*3
shapes = ['s']*7 + ['s']*4 + ['s']*3 + ['s']*2 + ['s']*2 + ['v']*4 + ['s']*2 + ['X'] + ['s']*3

kp = 6.5 #5.5
kd = 4 #3.5
basespeed = 200
dirR = 1
dirL = 1
## Tinh goc giua 2 vecto
class Vis:
    def init(lim,res):
        ax.clear()
        ax.xaxis.tick_top()                     # and move the X-Axis      
        ax.set_ylim(ax.get_ylim()[::-1])   
        ax.set_xlim(ax.get_xlim()[::1])   
        plt.xlabel('X Coordinates')
        plt.ylabel('Y Coordinates')
        plt.xticks(np.arange(0, lim, res))
        plt.yticks(np.arange(0, lim, res))
    def draw(name,i,x,y):
        if name[:1] == "a" :
            ax.scatter(x, y, s=50, marker=shapes[i], c=colors[i])
            ax.text(x-5,y+15, "au" + name[5:], fontsize=7)
        elif name[:1] == "m" :
            ax.scatter(x, y, s=50, marker=shapes[i], c=colors[i])
            ax.text(x-5,y+15, "man" + name[7:], fontsize=7)
        else :
            ax.scatter(x, y, s=40, marker=shapes[i], c=colors[i])
            ax.text(x-5,y+15, name[7:], fontsize=6)

def vAngle(v1,v2):
    # Ang = math.atan2(v2[1],v2[0]) - math.atan2(v1[1],v1[0])
    # Ang = math.degrees(Ang)
    # if Ang > 180 :   
    #     return((Ang-180))
    # elif Ang < -180:
    #     return(Ang+180)
    # else :
    # return(Ang)
    cross=v1[0]*v2[1]-v2[0]*v1[1]
    dot=v1[0]*v2[0]+v1[1]*v2[1]
    angle=math.atan2(cross,dot)
    angle=math.degrees(angle)/2
    return(angle)

def makeSSLContext(ca, crt, key):
    sslCTX = ssl.create_default_context(
        purpose=ssl.Purpose.SERVER_AUTH,
        cafile=ca
    )

    sslCTX.load_cert_chain(crt, key)

    return sslCTX


## Trả về chuỗi JSON chứa thông tin đăng nhâp
def makeJSONCredentials(username, password):
    creds = {
        "username": username,
        "password": password
    }

    return json.dumps(creds).encode("utf-8")


## Cài đặt và trả về thông tin của yêu cầu HTTPS
def makeRequestHeader(url, contentType, content):
    req = urllib.request.Request(url)

    req.add_header('Content-Type', contentType)
    req.add_header('Content-Length', len(content))

    return req


## Gửi yêu cầu đăng nhập và trả về mã xác thực
def getToken(url, username, password,
             ca, crt, key):
    reqSSLContext = makeSSLContext(ca, crt, key)
    reqContent = makeJSONCredentials(username, password)
    req = makeRequestHeader(
        url,
        'application/json; charset=utf-8',
        reqContent
    )

    # Gửi yêu cầu và nhận kết quả trả về
    resp = urllib.request.urlopen(
        req, data=reqContent, context=reqSSLContext)

    # Đọc và trả về mã xác thực
    respBody = resp.read()
    respBodyJSON = json.loads(respBody.decode('utf-8'))

    return respBodyJSON["token"]


# Đăng nhập và nhận dữ liệu


## Cài đặt thông tin của giao thức mã hoá cho Websocket

# Đường dẫn đến các tập tin nhận từ BTC
CA_CRT = str(Path("cacert.pem"))
CRT = str(Path("clientcert.pem"))
KEY = str(Path("clientkey.pem"))

sslopt = {
    'cert_reqs': ssl.PROTOCOL_SSLv23,
    'keyfile': KEY,
    'certfile': CRT,
    'ca_certs': CA_CRT,
}


## Nhận mã xác thực và thêm mã xác thực vào thông tin yêu cầu Websocket

# Tên địa chỉ server (thay đổi vào ngày thi đấu)
HOST = "192.168.1.100"
PORT = 4433

url = 'https://%s:%s/subscribe' % (HOST, PORT)
token = getToken(url,
                 # Thông tin tài khoản của mỗi đội
                 '400iq', 'ogOi8l6G',
                 CA_CRT, CRT, KEY)

header = {
    'Authorization': 'Bearer %s' % (token)
}

sslopt = {
    'cert_reqs': ssl.PROTOCOL_SSLv23,
    'keyfile': KEY,
    'certfile': CRT,
    'ca_certs': CA_CRT,
}

## Thiết lập kết nối Websocket và bắt đầu nhận dữ liệu
url = 'wss://%s:%s/data' % (HOST, PORT)
ws = websocket.create_connection(url,
                                 header=header,
                                 sslopt=sslopt)

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
    print("Connected")
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
vec  = [0,0]
lasterror = 0
while True:
    msg = ws.recv()
    packet = json.loads(msg.decode('utf-8'))
    data = (packet['data'])
    manid = 18
    auid = 21
    vec[0] = data[manid]['position'][0]-data[auid]['position'][0]
    vec[1] = data[manid]['position'][1]-data[auid]['position'][1]
    error = vAngle(data[auid]['dimension'],vec) 
    #print(error)
    pid = error * kp + (error - lasterror)*kd
    print(pid)
    lasterror = error
    pwmL = basespeed - pid
    pwmR = basespeed + pid
    pwmL = round(pwmL,1)
    pwmR = round(pwmR,1)
    if pwmL < 0:
        dirL = 2
        pwmL = min(255,abs(pwmL))
    else:
        dirL = 1
        pwmL = min(255,pwmL)
    if pwmR < 0:
        dirR = 2
        pwmR = min(255,abs(pwmR))
    else:
        dirR = 1
        pwmR = min(255,pwmR)
    sig = "<" + str(dirL) + "," + str(pwmL) + "," + str(dirR) + "," + str(pwmR) + ">"
    print(sig)
    sendSignal(sig)
    time.sleep(0.08)    
    ws.send(json.dumps({'finished': True}).encode('utf-8'))