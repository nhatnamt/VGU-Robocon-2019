#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import websocket
import json
import ssl
import time
import urllib.request
from pathlib import Path
import numpy as np
import numpy.linalg as lin
import math

## Tinh goc giua 2 vecto
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

while True:
    msg = ws.recv()
    packet = json.loads(msg.decode('utf-8'))
    data = (packet['data'])
    with open("out.txt", 'a') as outt:
        #outt.writelines(str(thisTime)+'\n') #thoi gian   
        for i in range(len(data)):
            outt.write(str(data[i]['name']) + ' ')
            outt.write(str(data[i]['position']) + ' ')
            outt.write(str(data[i]['dimension']) + '\n')
            outt.write(str(vAngle(data[i]['dimension'],[1,0]))+'\n') # Tinh goc so voi Ox
            outt.write('\n')
    time.sleep(1)
    ws.send(json.dumps({'finished': True}).encode('utf-8'))
