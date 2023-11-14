import cv2 as cv
import numpy as np
import time
import json
import queue
import serial
import random
import threading

import AiPhile

from pyzbar.pyzbar import decode
from math import sqrt, atan2, pi, ceil
from paho.mqtt import client as mqtt_client
from enum import Enum

# global variable
class State(Enum):
     pNONE          = 0
     pSTEP1         = 1
     pSTEP2         = 2
     pSTEP3         = 3
     pSTEP4         = 4
     pSTEPDONE      = 5
     pRUN           = 6
     pROTATELEFT    = 7
     pROTATERIGHT   = 8
     pBACK          = 9
     pDONE          = 10
     pSTOP          = 11
     BACKFLAG       = 12
     HEADFLAG       = 13
     p0             = 14
     p90            = 15
     n90            = 16
     n180           = 17
     pHEAD          = 18
     
HEAD            = [1, 1]
BACK            = [0, 0]
LEFT            = [0, 1]
RIGHT           = [1, 0]

tPROCESS        = State.pNONE
velocity        = 'normal'
L               = None
i               = 0

qDATA           = queue.Queue()     # Queue for transmit and receive data
qGUI2DATA       = queue.Queue()     # Queue for subcribe data from GUI
qDATA2GUI       = queue.Queue()     # Queue for publish data to GUI
qSTEP3          = queue.Queue()     # Queue for check terminate in STEP3
qBACK           = queue.Queue()     # Queue for BackWard
qVEL            = queue.Queue()     # Queue for Velocity

# Function - UART
Ser = serial.Serial(
    port        = '/dev/ttyS0',
    baudrate    = 115200,
    parity      = serial.PARITY_NONE,
    stopbits    = serial.STOPBITS_ONE,
    bytesize    = serial.EIGHTBITS,
    timeout     = 1
)

# Load file config
with open("config.json", "r") as f:
    config      = json.load(f)
    
# Utils
def Pre_Processing(s):
    split_string        = s.split("|")
    list_process_string = split_string[0].split(",")
    step_flag_string    = split_string[1].split(",")
    
    # Convert string to list
    list_process        = [int(i) for i in list_process_string]
    step_flag           = [int(i) for i in step_flag_string]

    # Remove 4 (None value) from list_process 
    while 4 in list_process: 
        list_process.remove(4)
    while 0 in list_process: 
        list_process.remove(0) 
    return list_process, step_flag

def Format_Data(Pos, Vel, Acc, Dir1, Dir2):
    PosStr = Fix_Length(Pos)
    VelStr = Fix_Length(Vel)
    AccStr = Fix_Length(Acc)
    Dir1Str = str(Dir1)
    Dir2Str = str(Dir2)
    result = "{},{},{},{},{}".format(PosStr, VelStr, AccStr, Dir1Str, Dir2Str)
    return result

def Compute_Distance(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    d = sqrt((T_x - G_x) ** 2 + (T_y - G_y) ** 2)
    return d 

def Compute_Angle(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    
    y = T_x - G_x 
    x = T_y - G_y 
    angle = atan2(y, x) # radian 
    # convert radian to degree 
    angle = angle * 180 / pi
    return angle

def Send_Data(Pos, Vel, Acc, Dir1, Dir2):
    data2Send = Format_Data(Pos, Vel, Acc, Dir1, Dir2)
    print(data2Send)
    print("into the SendData function")
    Ser.write(bytes(data2Send, 'utf-8'))

# The accessing information of the broker
broker          = 'broker.emqx.io'
port            = 1883
topic_sub       = "AGV/AGV_01/control"
topic_pub       = "AGV/AGV_01/feedback"
client_id       = f'python-mqtt--{random.randint(0, 100)}'
username        = 'pi'
password        = '123456789'

# MQTT Connect
def Connect_MQTT_Sub() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker - Subscribe Mission!")
        else:
            print("Failed to connect, return code %d \n", rc)
    
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def Connect_MQTT_Pub():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker - Publish Mission!")
        else:
            print("Failed to connect, return code %d \n", rc)
    
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def Subscribe(client: mqtt_client):
    global qGUI2DATA
    global velocity
    global tPROCESS
    
    def On_Message(client, userdata, msg):
        message = msg.payload.decode()
        message = message[:-1]
        if 'STOP' in message:
            tPROCESS = State.pSTOP
        else:
            data        = json.loads(message)
            payload     = data['payload']
            velocity    = payload['velocity']
            
            if qVEL.qsize() == 0:
                qVEL.put(velocity)
            path = payload['path']
            path = Pre_Processing(path)
            
            if qGUI2DATA.qsize() == 0:
                qGUI2DATA.put(path)
               
            if qGUI2DATA.qsize() == 1:
                qGUI2DATA.put("run")
                
    client.SUBSCRIBE(topic_sub)
    client.On_Message = On_Message
    
def Publish(client):
    if qDATA2GUI.qsize() == 1:
        data    = qDATA2GUI.get()
        time.sleep(1)
        message = data
        result  = client.Publish(topic_pub, message)
        status  = result[0]                 
        
        if status == 0:                   # Check the sending was successful ?
            print(message)
        else:
            print(f"Failed to send message to topic {topic_pub}")

def Sub_Message():
    global qGUI2DATA
    client = Connect_MQTT_Sub()
    Subscribe(client)
    client.loop_forever()

def Pub_Message():
    global qDATA2GUI
    client = Connect_MQTT_Pub()
    client.loop_start()
    Publish(client)
    
'''
STEP 1: Adjust the angle between the camera center and the QR code to be on the North line of the vehicle 
'''
def Do_Angle(qDATA):
    global tPROCESS
    if qDATA.qsize() == 3:
        angle_point = qDATA.get()
    angle_point = angle_point * config["coefficient"]
    
    if angle_point > 0:
    # positive angle -> rotate left
        if angle_point <= 90 * config["coefficient"]:           
            angle_point = round(angle_point)
            tPROCESS    = State.pSTEP2
            Send_Data(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], LEFT[0], LEFT[1])
        else:
            angle_point = 180 * config["coefficient"] - round(angle_point)
            tPROCESS    = State.pSTEP2
            Send_Data(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], RIGHT[0], RIGHT[1])
    # negative angle -> rotate right
    else:
        angle_point = round(abs(angle_point))
        if angle_point <= 90 * config["coefficient"]:
            tPROCESS    = State.pSTEP2
            Send_Data(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], RIGHT[0], RIGHT[1])
        else:
            tPROCESS    = State.pSTEP2
            angle_point = 180 * config["coefficient"] - angle_point
            Send_Data(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], LEFT[0], LEFT[1])

'''
STEP 2: Adjust the distance between the camera center and the QR code
'''
def Do_Distance(qDATA):
    global tPROCESS
    if qDATA.qsize() == 2:
        flag        = qDATA.get()
        distance    = qDATA.get()
        
    if flag     == State.BACKFLAG:
        distance    = round(distance * config["back_doDistance"])
        tPROCESS    = State.pSTEP3
        Send_Data(distance, config["vel_doDistance"], config["acc_doDistance"], BACK[0], BACK[1])
    elif flag   == State.HEADFLAG:
        distance    = round(distance * config["head_doDistance"])
        tPROCESS    = State.pSTEP3
        Send_Data(distance, config["vel_doDistance"], config["acc_doDistance"], HEAD[0], HEAD[1])
        
'''
STEP 3: Adjust the angle to coincide with the desired direction of movement
'''
def Do_Correct(qDATA):
    print(f"Into Do_Correct function and size of qDATA = {qDATA.qsize()}")
    global tPROCESS
    if qDATA.qsize() == 2:
        angle       = qDATA.get()
    if qDATA.qsize() == 1:
        step_flag   = qDATA.get()
    print(f"into Do_Correct function, angle = {angle}, step_flag = {step_flag}")
    
    if step_flag == 0:
        angle = round(angle * config["coefficient"])
        if angle >= -1 and angle <= 1:
            tPROCESS = State.pSTEP4
            if qSTEP3.qsize() == 0:
                qSTEP3.put("Terminate STEP3")
        elif angle > 1:
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle - 20), config["vel_doCorrect"], config["acc_doCorrect"], LEFT[0], LEFT[1])
        elif angle < -1:
            angle       = abs(angle)
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle - 20), config["vel_doCorrect"], config["acc_doCorrect"], RIGHT[0], RIGHT[1])
            
    if step_flag == 1:
        if angle >= 89 and angle <= 91:
            tPROCESS = State.pSTEP4
            if qSTEP3.qsize() == 0:
                qSTEP3.put("Terminate STEP3")
        elif angle > 91:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle - 335), config["vel_doCorrect"], config["acc_doCorrect"], LEFT[0], LEFT[1])
        elif angle < 89:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle - 335), config["vel_doCorrect"], config["acc_doCorrect"], RIGHT[0], RIGHT[1])
            
    if step_flag == 2:
        if angle == -90:
            tPROCESS = State.pSTEP4
            if qSTEP3.qsize() == 0:
                qSTEP3.put("Terminate STEP3")
        elif angle > -90:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle + 335), config["vel_doCorrect"], config["acc_doCorrect"], LEFT[0], LEFT[1])
        elif angle < -90:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(angle + 335), config["vel_doCorrect"], config["acc_doCorrect"], RIGHT[0], RIGHT[1])
            
    if step_flag == 3:
        if abs(angle) >= 179 and abs(angle) <= 180:
            tPROCESS = State.pSTEP4
            if qSTEP3.qsize() == 0:
                qSTEP3.put("Terminate STEP3")
        elif angle < 0 and angle > -179:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(round(angle + 180 * config["coefficient"])), config["vel_doCorrect"], config["acc_doCorrect"], LEFT[0], LEFT[1])
        elif angle > 0 and angle < 179:
            angle = round(angle * config["coefficient"])
            tPROCESS    = State.pSTEP4
            Send_Data(abs(round(angle - 180 * config["coefficient"])), config["vel_doCorrect"], config["acc_doCorrect"], RIGHT[0], RIGHT[1])
        
'''
STEP 4: Adjust the distance to the center
'''
def Do_Correct_Distance(qDATA):
    global tPROCESS
    print(f"Into Do_Correct_Distance function and the size of qDATA is {qDATA.qsize()}")
    if qDATA.qsize() == 2:
        flag = qDATA.get()
    print(f"Flag is {flag}")
    if qDATA.qsize() == 1:
        distance = qDATA.get()
        
    if flag == State.BACKFLAG:
        distance = round(distance * config["back_doCorrectDistance"])
        tPROCESS = State.pSTEPDONE
        Send_Data(distance, 15, 10, 0, 0)
    else:
        distance = round(distance * config["head_doCorrectDistance"])
        tPROCESS = State.pSTEPDONE
        Send_Data(distance, 15, 10, 1, 1)
        
# Action of vehicle
def Do_Head(qDATA):
    global tPROCESS
    global velocity
    global L
    if qVEL.qsize() == 1:
        velocity = qVEL.get()
    print(f"Into Do_Head velocity = {velocity}")
    
    if velocity == "hight":
        if L    == None:
            Send_Data(495, 100, 80, 1, 1)
        elif L  == "reduce_distance":
            L = None
            Send_Data(450, 100, 80, 1, 1)
    elif velocity == "low":
        if L    == None:
            Send_Data(495, 100, 80, 1, 1)
        elif L  == "reduce_distance":
            L = None
            Send_Data(450, 100, 80, 1, 1)
    elif velocity == "normal":
        if L    == None:
            Send_Data(495, 100, 80, 1, 1)
        elif L  == "reduce_distance":
            L = None
            Send_Data(450, 100, 80, 1, 1)
            
    tPROCESS = State.pDONE
    if qDATA.qsize() == 0:
        qDATA.put(State.pHEAD)
        
def Do_Back(qDATA):
    global tPROCESS
    Send_Data(670, 30, 20, 0, 1)
    if qBACK.qsize() == 0:
        qBACK.put("Go STEP 1")
        
def Do_Rotate_Left(qDATA):
    global tPROCESS
    tPROCESS = State.pSTEPDONE
    Send_Data(335, 40, 30, 0, 1)
    
def Do_Rotate_Right(qDATA):
    global tPROCESS
    tPROCESS = State.pSTEPDONE
    Send_Data(335, 35, 25, 1, 0)
    
def Receive_Data(qDATA):
    print("Into Receive_Data thread")
    global tPROCESS
    count_step4 = 0
    while True:
        s       = Ser.readline()
        data    = s.decode('utf-8')
        data    = data.rstrip()
        print(data)
        
        if (tPROCESS == State.pSTEP4) and "OK" not in data:
            count_step4 += 1
            print(f"count_step4 = {count_step4}")
        
        if count_step4 == config["count_step4"]:
            count_step4 = 0
            if qDATA.qsize() == 0:
                qDATA.put(State.BACKFLAG)
            if qDATA.qsize() == 1:
                qDATA.put(65)               # 65 is distance
            if qDATA.qsize() == 2:
                Do_Correct_Distance(qDATA)
                
        if "OK" in data:
            print("Into OK condition")
            print(f"tPROCESS = {tPROCESS}")
            if qBACK.qsize() == 1:
                print(qBACK.get())
                tPROCESS = State.pNONE
                
            if tPROCESS     == State.pNONE:
                pass
            elif tPROCESS   == State.pSTOP:
                pass
            elif tPROCESS   == State.pDONE:
                if qDATA.get() == State.pHEAD:
                    tPROCESS = State.pSTEP1
                    
            elif tPROCESS   == State.pSTEP1:
                print(f'into tProcess == pSTEP1 condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 3:
                    Do_Angle(qDATA)
                else:
                    continue
            
            elif tPROCESS   == State.pSTEP2:
                print(f'into tProcess == pSTEP2 condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 2:
                    Do_Distance(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pSTEP3:
                print(f'into tProcess == pSTEP3 condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 2:
                    Do_Correct(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pSTEP4:
                print(f'into tProcess == pSTEP4 condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 2:
                    Do_Correct_Distance(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pRUN:
                print(f'into tProcess == pRUN condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 0:
                    Do_Head(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pROTATELEFT:
                print(f'into tProcess == pROTATELEFT condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 0:
                    Do_Rotate_Left(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pROTATERIGHT:
                print(f'into tProcess == pROTATERIGHT condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 0:
                    Do_Rotate_Right(qDATA)
                else:
                    continue
                
            elif tPROCESS   == State.pBACK:
                print(f'into tProcess == pBACK condition and size of q = {qDATA.qsize()}')
                if qDATA.qsize() == 0:
                    Do_Back(qDATA)
                else:
                    continue
                
def Do_OpenCV(qDATA, qGUI2DATA):
    global tPROCESS
    global L
    global i
    print("Into Do_OpenCV thread")
    
    T_x             = 315
    T_y             = 245
    angle           = 0
    distance        = 0
    angle_point     = 0
    count           = 0             # Forward or Backward
    count_img       = 0
    counter         = 0
    count_send      = 0
    step_mode       = 0             # Between 2 mode in step_process
    pre_angle       = 500
    run             = None
    pre_message     = None
    
    cap = cv.VideoCapture(-1)
    frame_width     = int(cap.get(3))
    frame_height    = int(cap.get(4))
    frame_size      = (frame_width, frame_height)
    fourcc = cv.VideoWriter_fourcc(*'XVID')  
    out = cv.VideoWriter('output_video.avi', fourcc, 8, (480, 640))
    
    _, frame = cap.read()
    old_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    lk_params = dict(winSize=(20, 20),      # Optical Flow (Lucas-Kanade)
                    maxLevel=4,
                    criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.01))
    
    cv.circle(frame, (265, 177), radius = 0, color = (255, 0, 0), thickness = 4)
    old_points      = np.array([[]])
    qr_detected     = False
    frame_counter   = 0
    starting_time   = time.time()
    
    while True:
        if qGUI2DATA.qsize() == 2:
            list_process, step_process = qGUI2DATA.get()
            if 7 in list_process:
                list_process_backward = [8 if i == 7 else i for i in list_process]
                list_process_backward.reverse()         # reverse the order of elements
            elif 8 in list_process: 
                list_process_backward = [7 if i == 8 else i for i in list_process]
                list_process_backward.reverse()
            list_process.append(12)
            list_process_iter   = iter(list_process)    # iter allows iterating through each element
            
            list_process_backward_iter  = iter(list_process_backward)
            step_process_backward       = [step_process[1], step_process[0]]
            
            if step_process_backward[0] == 0:
                step_process_backward[0] = 3
            elif step_process_backward[0] == 3:
                step_process_backward[0] = 0
            elif step_process_backward[0] == 1:
                step_process_backward[0] = 2
            elif step_process_backward[0] == 2:
                step_process_backward[0] = 1
        
            if step_process_backward[1] == 0:
                step_process_backward[1] = 3
            elif step_process_backward[1] == 3:
                step_process_backward[1] = 0
            elif step_process_backward[1] == 1:
                step_process_backward[1] = 2
            elif step_process_backward[1] == 2:
                step_process_backward[1] = 1
                
            print(f"list_process = {list_process}")
            print(f"step_process = {step_process}")
        
        if qGUI2DATA.qsize() == 1:
            run = qGUI2DATA.get()
            
        if list_process:
            frame_counter += 1
            _, frame = cap.read()
            img = frame.copy()
            barcodes = decode(frame) 
            for barcode in barcodes: 
                barcodeData = barcode.data.decode("utf-8")
                try: 
                    barcodeData = json.loads(barcodeData)
                    data = str(barcodeData["Row"]) + "," + str(barcodeData["Column"])
                except:
                    pass
                # Forward
                if count == 0:
                    message = "DATA|0," + data
                    if pre_message != message:
                        if qDATA2GUI.qsize() == 0:
                            qDATA2GUI.put(message)
                        if qDATA2GUI.qsize() == 1:
                            client = Connect_MQTT_Pub()
                            Publish(client)
                        pre_message = message
                # Backward
                if count == 1:
                    message = "DATA|1," + data
                    if pre_message != message:
                        if qDATA2GUI.qsize() == 0:
                            qDATA2GUI.put(message)
                        if qDATA2GUI.qsize() == 1:
                            client = Connect_MQTT_Pub()
                            Publish(client)
                        pre_message = message
                        
                if len(barcode.polygon) != 4: 
                    pass 
                else:
                    p1_x = barcode.polygon[0].x
                    p1_y = barcode.polygon[0].y
                    
                    p2_x = barcode.polygon[1].x 
                    p2_y = barcode.polygon[1].y

                    p3_x = barcode.polygon[2].x 
                    p3_y = barcode.polygon[2].y

                    p4_x = barcode.polygon[3].x 
                    p4_y = barcode.polygon[3].y
                    
                (rv, points, straight_qrcode) = cv.QRCodeDetector().detectAndDecode(frame)  
                if rv:
                    points = points[0]
                    pt1 = points[0] 
                    pt2 = points[1]
                    pt3 = points[2]
                    pt4 = points[3]
                    a = int(pt2[1])
                    b = int(pt1[1])
                    c = int(pt2[0])
                    d = int(pt1[0])
                    
                    angle = atan2(b - a, c - d)     
                    angle = (angle * 180 / pi)
                    
                    if pre_angle > angle - 2 or pre_angle < angle + 2:                        
                        count_send += 1 
                    pre_angle = angle
                    
                    if count_send == 5:
                        count_send = 0
                        
                        if tPROCESS == State.pNONE:
                            print("Do_OpenCV into tPROCESS == pNONE")
                            tPROCESS = State.pSTEP1
                        elif tPROCESS == State.pSTEP1:
                            angle_point = round(Compute_Angle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            distance    = round(Compute_Distance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            G_x = (p2_x + p4_x) / 2.0
                            
                            print(f"**************  angle_point = {angle_point}")
                            print(f"**************  distance    = {distance}")
                            print(f"**************  angle       = {angle}")
                            print(f"**************  G_x - 315   = {G_x - 315}")
                            
                            if abs(G_x - 315) > config["saturation"]:
                                if qDATA.qsize() == 0:
                                    qDATA.put(int(angle_point))
                                if qDATA.qsize() == 1:
                                    if abs(angle_point) > 90:
                                        qDATA.put(State.BACKFLAG)
                                    elif abs(angle_point) <= 90:
                                        qDATA.put(State.HEADFLAG)
                                if qDATA.qsize() == 2:
                                    qDATA.put(int(distance))
                                Send_Data(1111,1111,1111,1,1)
                            else:
                                tPROCESS = State.pSTEP3
                                if qSTEP3.qsize() == 0:
                                    qSTEP3.put("Terminata")
                            count_step3 = 0
                            
                        elif tPROCESS == State.pSTEP3:
                            count_step3 += 1
                            print(f"count_step3 = {count_step3} in tPROCESS = pSTEP3")
                            G_y = (p2_y + p4_y) / 2.0
                            print(f"G_y = {G_y} in tPROCESS = pSTEP3")
                            
                            if qDATA.qsize() == 0:
                                qDATA.put(int(angle))
                                
                            if qDATA.qsize() == 1:
                                if count == 0:
                                    if step_mode == 0:
                                        qDATA.put(step_process[0])
                                    if step_mode == 1:
                                        qDATA.put(step_process[1])
                                elif count == 1:
                                    if step_mode == 0:
                                        qDATA.put(step_process_backward[0])
                                    if step_mode == 1:
                                        qDATA.put(step_process_backward[1])
                                        
                            if qDATA.qsize() == 2 and qSTEP3.qsize() == 1:          # In case angle_point < 20
                                count_step3 = 0
                                print(qSTEP3.get())
                                print(f"Size of queue after getting data {qSTEP3.qsize()}")
                                Send_Data(1111,1111,1111,1,1)
                                
                            if qDATA.qsize() == 2 and count_step3 > 5:
                                count_step3 = 0
                                Send_Data(1111,1111,1111,1,1)
                            count_step4 = 0
                            
                        elif tPROCESS == State.pSTEP4:
                            print(f"count_step4 = {count_step4} in tPROCESS = pSTEP4")
                            distance = round(distance = round(Compute_Distance(p2_x, p2_y, p4_x, p4_y, T_x, T_y)))
                            if (abs(angle) > 170) or (abs(angle) > 80 and abs(angle) < 100) or (abs(angle) < 10):
                                G_y = (p2_y + p4_y) / 2.0
                                print(f"G_y = {G_y}")
                                count_step4 += 1
                                print(f"count_step4 = {count_step4}")
                                
                                if qDATA.qsize() == 0:
                                    if G_y > 230:
                                        qDATA.put(State.BACKFLAG)
                                    else:
                                        qDATA.put(State.HEADFLAG)
                                        
                                if qDATA.qsize() == 1:
                                    qDATA.put(abs(G_y - 245))
                                    
                                if qDATA.qsize() == 2 and qSTEP3.qsize() == 1:
                                    count_step4 = 0
                                    print(qSTEP3.get())
                                    Send_Data(1111,1111,1111,1,1)
                                elif qDATA.qsize() == 2 and count_step4 > 5:
                                    count_step4 = 0
                                    Send_Data(1111,1111,1111,1,1)
                                    
                        elif tPROCESS == State.pSTEPDONE:
                            print(f"Into tPROCESS = pSTEPDONE")
                            if count == 1:
                                tPROCESS == next(list_process_backward_iter)
                                if tPROCESS == State.pROTATERIGHT:
                                    L = 'reduce_distance'
                                if tPROCESS == State.pROTATELEFT or tPROCESS == State.pROTATERIGHT:
                                    step_mode = 1
                                    
                            if count == 0:
                                tPROCESS = next(list_process_iter)
                                if tPROCESS == State.pROTATERIGHT:
                                    L = 'reduce_distance'
                                if tPROCESS == State.pROTATELEFT or tPROCESS == State.pROTATERIGHT:
                                    step_mode = 1
                                if tPROCESS == State.pBACK:
                                    count = 1
                                    step_mode = 0
                                    pre_message = None
                                
            img = cv.resize(img, None, fx=2, fy=2,interpolation=cv.INTER_CUBIC)
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            stop_code = False
            
            if qr_detected and stop_code == False:
                new_points, status, error = cv.calcOpticalFlowPyrLK(old_gray, gray_frame, old_points, None, **lk_params)
                old_points = new_points 
                new_points = new_points.astype(int)
                n = (len(new_points))
                frame = AiPhile.fillPolyTrans(frame, new_points, AiPhile.GREEN, 0.4)
                AiPhile.textBGoutline(frame, f'Detection: Optical Flow', (30,80), scaling=0.5,text_color=AiPhile.GREEN)
                cv.circle(frame, (new_points[0]), 3,AiPhile.GREEN, 2)
                
            old_gray    = gray_frame.copy()
            key         = cv.waitKey(1)
            if key == ord("s"):
                cv.imwrite(f'reference_img/Ref_img{frame_counter}.png', img)
            if key == ord("q"):
                break
            fps = frame_counter/(time.time()-starting_time)
            AiPhile.textBGoutline(frame, f'FPS: {round(fps,1)}', (30,40), scaling=0.6)
            
try:
    t   = time.time()
    t1  = threading.Thread(target=Receive_Data, args=(qDATA, ))
    t2  = threading.Thread(target=Do_OpenCV, args=(qDATA, qGUI2DATA))
    t3  = threading.Thread(target=Sub_Message)
    
    t1.start()
    t2.start()
    t3.start()
    
    t1.join()
    t2.join()
    t3.join()
    
    print("All thread join!")
except KeyboardInterrupt:
    Ser.close()