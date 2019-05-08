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

#Vo thi nho sua vi tri
points = [0]*4 + [5]*7 + [10]*4 + [15]*3 + [20]*2 + [25]*2 + [30]*2 + [-20] + [50]*3
colors = ['red']*4 +['#bdf300']*7 + ['#5fe73a']*4 + ['#00d06a']*3 + ['#008c8b']*2 + ['#29678f']*2 + ['#41408e']*2 + ['k'] + ['#4f1186']*3
shapes = ['v']*4 + ['s']*7 + ['s']*4 + ['s']*3 + ['s']*2 + ['s']*2 + ['s']*2 + ['X'] + ['s']*3

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
            ax.text(x-20,y+70, "au" + name[5:], fontsize=7)
        elif name[:1] == "m" :
            ax.scatter(x, y, s=50, marker=shapes[i], c=colors[i])
            ax.text(x-20,y+70, "man" + name[7:], fontsize=7)
        else:
            ax.scatter(x, y, s=40, marker=shapes[i], c=colors[i])
            ax.text(x-15,y+70, name[7:], fontsize=6)

def vAngle(v1,v2):
    Ang = math.atan2(v2[1],v2[0]) - math.atan2(v1[1],v1[0])
    Ang = math.degrees(Ang)
    if Ang > 180 :   
        return(360-Ang)
    else :
        return(Ang)

## Cài đặt và trả về thông tin cho giao thức mã hoá HTTPS
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
# Server name & port 
HOST = "test.tunglevo.com"
PORT = 4433

url = 'https://%s:%s/subscribe' % (HOST, PORT)
token = getToken(url,
                 # Thông tin tài khoản của mỗi đội
                 'user', 'password',
                 CA_CRT, CRT, KEY)

header = {
    'Authorization': 'Bearer %s' % (token)
}

## Thiết lập kết nối Websocket và bắt đầu nhận dữ liệu
url = 'wss://%s:%s/data' % (HOST, PORT)
ws = websocket.create_connection(url,
                                 header=header,
                                 sslopt=sslopt)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

def animate(t):
    msg = ws.recv()
    packet = json.loads(msg.decode('utf-8'))
    data = (packet['data'])
    Vis.init(2000,200)
    for i in range(len(data)):
        name = str(data[i]['name'])
        x = data[i]['position'][0]
        y = data[i]['position'][1]
        Vis.draw(name,i,x,y)
    time.sleep(0.01)
    ws.send(json.dumps({'finished': True}).encode('utf-8'))
ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
    
