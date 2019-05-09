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

def makeSSLContext(ca, crt, key):
    sslCTX = ssl.create_default_context(purpose=ssl.Purpose.SERVER_AUTH, cafile=ca)
    sslCTX.load_cert_chain(crt, key)
    return sslCTX

def makeJSONCredentials(username, password):
    creds = {
        "username": username,
        "password": password
    }
    return json.dumps(creds).encode("utf-8")

def makeRequestHeader(url, contentType, content):
    req = urllib.request.Request(url)
    req.add_header('Content-Type', contentType)
    req.add_header('Content-Length', len(content))
    return req

def getToken(url, username, password, ca, crt, key):
    reqContent = makeJSONCredentials(username, password)
    req = makeRequestHeader(url, 'application/json; charset=utf-8', reqContent)
    resp = urllib.request.urlopen(req, data=reqContent)
    respBody = resp.read()
    respBodyJSON = json.loads(respBody.decode('utf-8'))
    return respBodyJSON["token"]

def _json_object_hook(d): 
    return namedtuple('X', d.keys())(*d.values())

def json2obj(data): 
    return json.loads(data, object_hook=_json_object_hook)

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
        HOST = "test.tunglevo.com"
        PORT = 4433
        url = 'https://%s:%s/subscribe' % (HOST, PORT)
        token = getToken(url, 'user', 'password', CA_CRT, CRT, KEY)
        header = {
            'Authorization': 'Bearer %s' % (token)
        }

        url = 'wss://%s:%s/data' % (HOST, PORT)
        ws = websocket.create_connection(url, header=header, sslopt=sslopt)

        #ROS chatter communication via nodes
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1) # 1 message per second

        #Receiving data from server
        while not rospy.is_shutdown():
            msg = ws.recv()
            packet = json.loads(msg.decode('utf-8'))
            packet = json.dumps(packet)
            rospy.loginfo(packet)
            pub.publish(packet)
            rate.sleep()
            ws.send(json.dumps({'finished': True}).encode('utf-8'))

    except rospy.ROSInterruptException:
        pass
