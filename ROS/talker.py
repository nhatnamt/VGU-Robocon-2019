#!/usr/bin/env python3
import json
import ssl
import time
import urllib.request
import rospy
import websocket
from pathlib import Path
from collections import namedtuple
from std_msgs.msg import String

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
    
if __name__ == '__main__':
    try:
        ssl._create_default_https_context = ssl._create_unverified_context
        CA_CRT = str("/home/khanh/catkin_ws/src/beginner_tutorials/scripts/cacert.pem")
        CRT = str("/home/khanh/catkin_ws/src/beginner_tutorials/scripts/clientcert.pem")
        KEY = str("/home/khanh/catkin_ws/src/beginner_tutorials/scripts/clientkey.pem")
        sslopt = {
            'cert_reqs': ssl.PROTOCOL_SSLv23,
            'keyfile': KEY,
            'certfile': CRT,
            'ca_certs': CA_CRT,
        }


        ## Nhận mã xác thực và thêm mã xác thực vào thông tin yêu cầu Websocket

        # Tên địa chỉ server (thay đổi vào ngày thi đấu)
        HOST = "192.168.1.100"
        # Tên cổng kết nối (thay đổi vào ngày thi đấu)
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


        #ROS chatter communication via nodes
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10)

        #Receiving data from server
        while not rospy.is_shutdown():
            msg = ws.recv()
            packet = json.loads(msg.decode('utf-8'))
            packet = json.dumps(packet)
            rospy.loginfo('Receiving data from server...')
            pub.publish(packet)
            rate.sleep()
            ws.send(json.dumps({'finished': True}).encode('utf-8'))

    except rospy.ROSInterruptException:
        pass
