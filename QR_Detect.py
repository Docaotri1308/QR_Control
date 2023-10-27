import cv2 as cv
import numpy as np
import time
import json
import queue
import serial
import threading
import random
import AiPhile

from pyzbar.pyzbar import decode
from math import sqrt, atan2, pi, ceil
from paho.mqtt import client as mqtt_client

# global variable
BACK            = [0, 0]
HEAD            = [1, 1]
LEFT            = [0, 1]
RIGHT           = [1, 0]

pNONE           = 0
pSTEP1          = 1
pSTEP2          = 2
pSTEP3          = 3
pSTEP4          = 4
pSTEPDONE       = 5

pRUN            = 6
pROTATELEFT     = 7
pROTATERIGHT    = 8

pBACK           = 9
pDONE           = 10
pSTOP           = 11

BackFlag        = 12
HeadFlag        = 13

p0              = 14 
p90             = 15
n90             = 16
n180            = 17

Head_doing      = 18

tProcess        = pNONE
velocity        = 'normal'

q1              = queue.Queue()     # Init empty queue
q2              = queue.Queue()     # Data subscribe from GUI
q3              = queue.Queue()     
q4              = queue.Queue()     # Data publish to GUI
q5              = queue.Queue()     # Queue for Backward
q6              = queue.Queue()     # Queue for Velocity

# Function - Uart
ser = serial.Serial(
    port        = '/dev/ttyS0',
    baudrate    = 115200,
    parity      = serial.PARITY_NONE,
    stopbits    = serial.STOPBITS_ONE,
    bytesize    = serial.EIGHTBITS,
    timeout     = 1
)

# The accessing information of the broker
broker      = 'localhost'
port        = 1883
topic_sub   = "AGV/AGV_01/control"
topic_pub   = "AGV/AGV_01/feedback"
client_id   = f'python-mqtt-{random.randint(0, 100)}'
username    = 'pi'
password    = '123456789'

# Load file config
with open("config.json", "r") as f:
    config  = json.load(f)
    
def PreProcessing(s):
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

def FixLength(value):
    if len(str(value)) == 1:
        return "000" + str(value)
    elif len(str(value)) == 2:
        return "00" + str(value)
    elif len(str(value)) == 3:
        return "0" + str(value)
    else: 
        return str(value)

def FormatData(Pos, Vel, Acc, Dir1, Dir2):
    PosStr  = FixLength(Pos)
    VelStr  = FixLength(Vel)
    AccStr  = FixLength(Acc)
    Dir1Str = str(Dir1)
    Dir2Str = str(Dir2)
    result  = "{},{},{},{},{}".format(PosStr, VelStr, AccStr, Dir1Str, Dir2Str)
    return result

def SendData(Pos, Vel, Acc, Dir1, Dir2):
    Data2Send = FormatData(Pos, Vel, Acc, Dir1, Dir2)
    print(Data2Send)
    print("into the SendData function")
    ser.write(bytes(Data2Send, 'utf-8'))
    
def ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    d = sqrt((T_x - G_x) ** 2 + (T_y - G_y) ** 2)
    return d

def ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y):
    G_x = (p2_x + p4_x) / 2.0
    G_y = (p2_y + p4_y) / 2.0
    
    y = T_x - G_x 
    x = T_y - G_y 
    angle = atan2(y, x) # radian 
    # convert radian to degree 
    angle = angle * 180 / pi
    return angle

# Mqtt connect
def connect_mqtt_sub() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:                                                 # rc: mã trạng thái của lỗi
            print("Connected to MQTT Broker - Subcribe Mission!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def connect_mqtt_pub():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker - Publish Mission!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def subscribe(client: mqtt_client):
    global q2 
    global velocity 
    global tProcess 
    def on_message(client, userdata, msg):      
        message = msg.payload.decode()          
        message = message[:-1]
        if 'STOP' in message: 
            tProcess = pSTOP                
        else:
            data        = json.loads(message)          
            payload     = data['payload']
            velocity    = payload['velocity']
            if q6.qsize() == 0:
                q6.put(velocity)
            path = payload['path']          # class string
            path = PreProcessing(path)
            if q2.qsize() == 0:
                q2.put(path)
            if q2.qsize() == 1: 
                q2.put("run")
    client.subscribe(topic_sub)
    client.on_message = on_message
    
def publish(client):
    if q4.qsize() == 1: 
        data    = q4.get()
        time.sleep(1)
        message = data
        result  = client.publish(topic_pub, message)
        status  = result[0]
        if status == 0:                     # kiểm tra gửi thành công chưa
            print(message)
        else:
            print(f"Failed to send message to topic {topic_pub}")
            
def SubMessage():
    global q2 
    client = connect_mqtt_sub()
    subscribe(client)
    client.loop_forever()

def PubMessage():
    global q4 
    client = connect_mqtt_pub()
    client.loop_start()
    publish(client)

'''
STEP 1: Điều chỉnh góc giữa tâm camera tâm mã QR cùng nằm trên đường Bắc của xe 
'''
def doAngle(q):
    global tProcess
    if q.qsize() == 3: 
        angle_point = q.get()
    angle_point = angle_point * config["coefficient"]
    if angle_point > 0:
        # goc (duong) be hon 90 -> xoay trai 
        if angle_point <= 90 * config["coefficient"]:
            angle_point = round(angle_point)
            tProcess    = pSTEP2
            SendData(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], LEFT[0], LEFT[1])
        # goc (duong) lon hon 90 -> xoay trai
        else: 
            angle_point = 180 * config["coefficient"] - round(angle_point)
            tProcess    = pSTEP2 
            SendData(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], RIGHT[0], RIGHT[1])
    # angle_point < 0 
    else: 
        angle_point = round(abs(angle_point))
        if angle_point <= 90 * config["coefficient"]: 
            tProcess    = pSTEP2
            SendData(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], RIGHT[0], RIGHT[1])
        else: 
            tProcess    = pSTEP2 
            angle_point = 180 * config["coefficient"] - angle_point 
            SendData(round(angle_point), config["vel_doAngle"], config["acc_doAngle"], LEFT[0], LEFT[1])
            
'''
STEP 2: Điều chỉnh khoảng cách giữa tâm camera và mã QR
'''
def doDistance(q):
    global tProcess
    if q.qsize() == 2:
        Flag     = q.get()
        distance = q.get()
    # 1000 -> go back 
    if Flag == BackFlag:
        distance = round(distance * config["back_doDistance"])
        tProcess = pSTEP3
        SendData(distance, config["vel_doDistance"], config["acc_doDistance"], 0, 0)
    # 1001 -> go head 
    elif Flag == HeadFlag:
        distance = round(distance * config["head_doDistance"])
        tProcess = pSTEP3
        SendData(distance, config["vel_doDistance"], config["acc_doDistance"], 1, 1)
        
'''
STEP 3: Điều chỉnh góc trùng với hướng di chuyển mong muốn
'''
def doCorrect(q):
    print(f'into doCorrect function and size of q = {q.qsize()}')
    global tProcess
    if q.qsize() == 2:
        angle = q.get()
    if q.qsize() == 1:
        step_flag = q.get()
    print(f"into doCorrect function, angle = {angle}, step_flag = {step_flag}")
    if step_flag == 0:
        angle = round(angle * config["coefficient"])
        if angle >= -1 and angle <= 1: 
            tProcess = pSTEP4 
            if q3.qsize() == 0:
                q3.put("terminate step 3")
        elif angle > 1:
            tProcess = pSTEP4
            SendData(abs(angle-20), config["vel_doCorrect"], config["acc_doCorrect"], 0, 1)
        elif angle < -1: 
            angle    = abs(angle)
            tProcess = pSTEP4
            SendData(abs(angle-20), config["vel_doCorrect"], config["acc_doCorrect"], 1, 0)
            
    if step_flag == 1:
        if angle >= 89 and angle <= 91:
            tProcess = pSTEP4 
            if q3.qsize() == 0:
                q3.put("terminate step 3")
        if angle > 91: 
            angle    = round(angle * config["coefficient"])
            tProcess = pSTEP4
            SendData(abs(angle - 335), config["vel_doCorrect"], config["acc_doCorrect"], 0, 1)
        elif angle < 89: 
            angle    = round(angle * config["coefficient"])
            tProcess = pSTEP4
            SendData(abs(angle - 335), config["vel_doCorrect"], config["acc_doCorrect"], 1, 0)

    if step_flag == 2:
        if angle == -90: 
            tProcess = pSTEP4 
            if q3.qsize() == 0:
                q3.put("terminate step 3")
        elif angle > -90:
            angle    = round(angle * config["coefficient"]) 
            tProcess = pSTEP4 
            SendData(abs(angle + 335), config["vel_doCorrect"], config["acc_doCorrect"], 0, 1)
        elif angle < -90: 
            angle    = round(angle * config["coefficient"]) 
            tProcess = pSTEP4 
            SendData(abs(angle + 335), config["vel_doCorrect"], config["acc_doCorrect"], 1, 0)
 
    if step_flag == 3:
        if abs(angle) >= 179 and abs(angle) <= 180: 
            tProcess = pSTEP4
            if q3.qsize() == 0:
                q3.put("terminate step 3") 
        elif angle < 0 and angle > -179:
            angle    = round(angle * config["coefficient"])
            tProcess = pSTEP4 
            SendData(abs(round(angle + 180 * config["coefficient"])), config["vel_doCorrect"], config["acc_doCorrect"], 0, 1)
        elif angle > 0 and angle < 179: 
            angle    = round(angle * config["coefficient"])
            tProcess = pSTEP4
            SendData(abs(round(angle - 180 * config["coefficient"])), config["vel_doCorrect"], config["acc_doCorrect"], 1, 0)
            
'''
STEP 4: Điều chỉnh khoảng cách về tâm
'''
def doCorrectDistance(q):
    global tProcess 
    print(f'into doCorrectDistance function and the size of q is {q.qsize()}') 
    if q.qsize() == 2: 
        Flag = q.get()
    print(f"Flag is {Flag}")
    if q.qsize() == 1: 
        distance = q.get()
    if Flag == 50:
        distance = round(distance * config["back_doCorrectDistance"])
        tProcess = pSTEPDONE 
        SendData(distance, 15, 10, 0, 0)
    else:
        distance = round(distance * config["head_doCorrectDistance"])
        tProcess = pSTEPDONE 
        SendData(distance, 15, 10, 1, 1)
        
def doHead(q):
    global tProcess 
    global velocity 
    global L 
    if q6.qsize() == 1: 
        velocity = q6.get()
    a = None 
    v = None 
    # normal: 60, 40
    # max: 80, 60
    print(f"into doHead velocity = {velocity}")
    if velocity == "high":
        if L == None:
            SendData(495, 100, 80, 1, 1)
        elif L == 'reduce_distance': 
            L = None 
            SendData(450, 100, 80, 1, 1)
    elif velocity == "low": 
        if L == None:
            SendData(495, 100, 80, 1, 1)
        elif L == 'reduce_distance': 
            L = None 
            SendData(450, 100, 80, 1, 1)
    elif velocity == "normal":
        if L == None:
            SendData(495, 100, 80, 1, 1)
        elif L == 'reduce_distance': 
            L = None 
            SendData(450, 100, 80, 1, 1)
    # Nếu gán tProcess = pNONE ngay ở đây thì ngay lập tức luồng doOpenCV sẽ thực hiện CALIB 
    tProcess = pDONE 
    if q.qsize() == 0: 
        q.put(Head_doing)
        
def doBack(q):
    global tProcess 
    SendData(670, 30, 20, 0, 1)
    if q5.qsize() == 0: 
        q5.put("go cablib 1")
def doRotateLeft(q):
    global tProcess 
    tProcess = pSTEPDONE
    SendData(335, 40, 30, 0, 1)

def doRotateRight(q): 
    global tProcess
    tProcess = pSTEPDONE 
    SendData(335, 35, 25, 1, 0)
    
def ReceiveData(q):
    print('into RecieveData thread')
    global tProcess 
    count_step4 = 0 
    while True:
        s = ser.readline()          # s is an bytes object 
        data = s.decode('utf-8')    # bytes object b'\xe2\x82\xac100' -> after using decode() method -> data: €100
        data = data.rstrip()        # remove trailing whitespace ('\n')
        print(data) 
        '''
        Nếu trong STEP 4 mà camera không thấy được mã QR 
        '''
        if (tProcess == pSTEP4) and "OK" not in data: 
            count_step4 += 1 
            print(f"count_step4 = {count_step4}")
        if count_step4 == config["count_step4"]: 
            count_step4 = 0 
            if q.qsize() == 0:
                q.put(12)
            if q.qsize() == 1: 
                q.put(65)       
            if q.qsize() == 2:
                doCorrectDistance(q)

        if "OK" in data:
            print('into OK condition')
            print(f"tProcess = {tProcess}")
            if q5.qsize() == 1: 
                print(q5.get())
                tProcess = pNONE 

            if tProcess == pNONE:
                pass
            elif tProcess == pSTOP: 
                pass 
            elif tProcess == pDONE: 
                if q.get() == Head_doing: 
                    tProcess = pSTEP1 
                           
            elif tProcess == pSTEP1: # pSTEP1 = 2 
                # print(f'into tProcess == pSTEP1 condition and size of q = {q.qsize()}')
                if q.qsize() == 3: 
                    doAngle(q)
                else: 
                    continue 
            elif tProcess == pSTEP2: # pSTEP2 = 3 
                # print(f'into tProcess == pSTEP2 condition and size of q = {q.qsize()}')
                if q.qsize() == 2:
                    doDistance(q)
                else: 
                    continue 
            elif tProcess == pSTEP3: # pSTEP3 = 4 
                # print(f'into tProcess == pSTEP3 condition and size of q = {q.qsize()}') 
                if q.qsize() == 2:
                    doCorrect(q)
                else:  
                    # print(f"RecieveData into tProcess == pSTEP3 but can't do function doCorrect(q) because q.qsize()={q.qsize()}")
                    continue  
            elif tProcess == pSTEP4: # pSTEP4 = 9 
                print(f'ReceiveData tProcess == pSTEP4 condition and size of q = {q.qsize()}')
                if q.qsize() == 2:
                    doCorrectDistance(q)
                else: 
                    continue
            elif tProcess == pRUN:
                # print(f'into tProcess == pRUN condition and size of q = {q.qsize()}')
                if q.qsize() == 0:
                    doHead(q)
                else: 
                    continue 
            elif tProcess == pROTATELEFT: 
                # print(f"into tProcess == pROTATELEFT condition and size of q = {q.qsize()}")
                if q.qsize() == 0:
                    doRotateLeft(q)
                else: 
                    continue 
            elif tProcess == pROTATERIGHT:
                if q.qsize() == 0:
                    doRotateRight(q)
                else:
                    continue 
            elif tProcess == pBACK: 
                if q.qsize() == 0:
                    doBack(q)
                else:
                    continue
                
def doOpenCV(q, q2):
    
    global tProcess
    global i       
    global L            
    print('into doOpenCV thread')
    
    count_image         = 0 
    angle_point         = 0 
    distance            = 0     
    angle               = 0
    T_x                 = 315 
    T_y                 = 245
    pre_angle           = 500
    count_send          = 0
    count               = 0
    step_mode           = 0 
    run                 = None 
    pre_message         = None 
    list_process        = None 
    list_process_backward = None  
    # list_process        = [6, 6, 7, 6]
    # step_process       = [0, 2]
    # list_process_iter = iter(list_process)
    cap = cv.VideoCapture(-1)
    
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    frame_size = (frame_width, frame_height)

    # Define the codec and create VideoWriter object
    fourc = cv.VideoWriter_fourcc(*'XVID')  # Codec của video (ở đây sử dụng XVID)
    out = cv.VideoWriter('output_video.avi', fourc, 8, (480, 640))
    
    _, frame = cap.read()
    old_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    lk_params = dict(winSize=(20, 20),
                    maxLevel=4,
                    criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.01))
    cv.circle(frame, (265, 177), radius = 0, color = (255, 0, 0), thickness = 4)
    old_points = np.array([[]])
    qr_detected= False
    frame_counter = 0
    starting_time = time.time()
    
    while True:
        if q2.qsize() == 2:
            list_process, step_process = q2.get() 
            if 7 in list_process: 
                list_process_backward = [8 if i == 7 else i for i in list_process]
                list_process_backward.reverse()
            elif 8 in list_process: 
                list_process_backward = [7 if i == 8 else i for i in list_process]
                list_process_backward.reverse()
            list_process.append(12)
            list_process_iter = iter(list_process)
            
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
        if q2.qsize() == 1:
            run = q2.get()

        if list_process: 
            count_image += 1 
            frame_counter += 1
            _, frame = cap.read()
            # print(type(frame))
            cv.imwrite(f"/home/pi/adu_final/final/images/number{count_image}.png", frame)
            # out.write(frame)
            # cv.line(frame, (65, 70), (565, 70), (255, 255, 0), 2)
            # cv.line(frame, (565, 70), (565, 420), (255, 255, 0), 2)
            # cv.line(frame, (565, 420), (65, 420), (255, 255, 0), 2)
            # cv.line(frame, (65, 420), (65, 70), (255, 255, 0), 2)
            img = frame.copy()
            barcodes = decode(frame) 
            for barcode in barcodes: 
                barcodeData = barcode.data.decode("utf-8")
                try: 
                    barcodeData = json.loads(barcodeData)
                    data = str(barcodeData["Row"]) + "," + str(barcodeData["Column"])
                except:
                    pass 
                # forward 
                if count == 0:  
                    message = "DATA|0," + data
                    if pre_message != message:
                        if q4.qsize() == 0:
                            q4.put(message)
                        if q4.qsize() == 1: 
                            client = connect_mqtt_pub()
                            publish(client)     
                        pre_message = message 
                # backward 
                if count == 1:
                    message = "DATA|1," + data 
                    if pre_message != message: 
                        if q4.qsize() == 0:
                            q4.put(message)
                        if q4.qsize() == 1: 
                            client = connect_mqtt_pub()
                            publish(client)     
                        pre_message = message 
                # print(data)
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
                    # Toa do dung cua ma QR
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
                    # print(f"**************G_x - 315   = {G_x - 315}")
                    # print(f"count_send = {count_send}")
                    # print(f"G_y = {(p2_y + p4_y) / 2.0}")
                    # print(f"angle_point = {round(utils.ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))}")
                    if count_send == 5:
                        # print(f"angle is {angle}")
                        # print(f"count is {count}")
                        count_send = 0
                        if tProcess == pNONE:
                            print('doOpenCV into tprocess == pNONE')
                            tProcess = pSTEP1
                            # tProcess = pSTOP 
                            # tProcess = pSTEP3
                        elif tProcess == pSTEP1:
                            angle_point = round(ComputeAngle(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            distance    = round(ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            G_x = (p2_x + p4_x) / 2.0
                            # print(f'doOpenCV into tProcess == pSTEP1 and distance = {distance}')
                            '''
                            Goc 20 y nghia la gi ?
                            + Neu angle_point nho hon 20 thi bo qua hai buoc pSTEP1 (angle_point) va pSTEP2 (distance)
                            + Con lon hon 20 thi cho di 
                            '''
                            print(f"**************angle_point = {angle_point}")
                            print(f"**************distance    = {distance}")
                            print(f"**************angle       = {angle}")
                            print(f"**************G_x - 315   = {G_x - 315}")
                            if abs(G_x - 315) > config["saturation"]:
                                if q.qsize() == 0:
                                    q.put(int(angle_point))  
                                if q.qsize() == 1:
                                    if abs(angle_point) > 90:
                                        q.put(BackFlag) # di lui, goc lon hon 90
                                    elif abs(angle_point) <= 90: 
                                        q.put(HeadFlag) # di toi, goc nho hon 90 
                                if q.qsize() == 2: 
                                    q.put(int(distance)) 
                                SendData(1111,1111,1111,1,1)       
                            else: 
                                tProcess = pSTEP3
                                if q3.qsize() == 0: 
                                    q3.put("terminate")
                            count_step3 = 0
                        # Calib flag nên quăng vào trong này để xử lý 
                        # Ý tưởng để xử lý:   
                        elif tProcess == pSTEP3:
                            count_step3 += 1 
                            print(f"count_step3 = {count_step3} in tProcess = pSTEP3")
                            G_y = (p2_y + p4_y) / 2.0
                            print(f"G_y={G_y} in tProcess == pSTEP3")
                            # print("doOpenCV into tProcess == pSTEP3")
                            # print(f"doOpenCV tProcess = {tProcess} and angle = {angle} and size of q = {q.qsize()}")
                            if q.qsize() == 0: 
                                q.put(int(angle))
                            if q.qsize() == 1: 
                                if count == 0: 
                                    if step_mode == 0: 
                                        q.put(step_process[0])
                                    if step_mode == 1: 
                                        q.put(step_process[1])
                                elif count == 1: 
                                    if step_mode == 0: 
                                        q.put(step_process_backward[0])
                                    if step_mode == 1: 
                                        q.put(step_process_backward[1])
                            if q.qsize() == 2 and q3.qsize() == 1:
                                count_step3 = 0 
                                print(q3.get())
                                print(f"size of q after geting data {q3.qsize()}")
                                SendData(1111,1111,1111,1,1)                           
                            if q.qsize() == 2 and count_step3 > 5: 
                               count_step3 = 0 
                               SendData(1111,1111,1111,1,1)                        
                            count_step4 = 0 
                        elif tProcess == pSTEP4:
                            # print(f"into tProcess == pSTEP4 and q.qsize() = {q.qsize()}, q3.qsize() = {q3.qsize()}")                       
                            print(f"count_step4 = {count_step4} in tProcess == pSTEP4")
                            distance = round(ComputeDistance(p2_x, p2_y, p4_x, p4_y, T_x, T_y))
                            # print(f"G_y = {G_y}")
                            # print(f"abs(angle) = {abs(angle)}")
                            if (abs(angle) > 170) or (abs(angle) > 80 and abs(angle) < 100) or (abs(angle) < 10):
                                G_y = (p2_y + p4_y) / 2.0
                                print(f"G_y = {G_y}")
                                count_step4 += 1     
                                print(f"count_step4 = {count_step4}")
                                if q.qsize() == 0: 
                                    if G_y > 230:
                                        q.put(12) # is BackFlag
                                    else:
                                        q.put(13) # is HeadFlag 
                                if q.qsize() == 1: 
                                    q.put(abs(G_y - 245))
                                if q.qsize() == 2 and q3.qsize() == 1:
                                    count_step4 = 0 
                                    print(q3.get()) 
                                    SendData(1111,1111,1111,1,1)
                                elif q.qsize() == 2 and count_step4 > 5:
                                    count_step4 = 0 
                                    SendData(1111,1111,1111,1,1)                               
                        # Giữa hai hành động trong list_process là quá trình calib
                        elif tProcess == pSTEPDONE:
                            # Lý do count == 1 nằm phía trên để chứ không phải dưới count == 0 -> ngẫm một tí là hiểu 
                            if count == 1: 
                                tProcess = next(list_process_backward_iter)
                                if tProcess == 8:
                                    L = 'reduce_distance'
                                if tProcess == 7 or tProcess == 8:
                                    step_mode = 1 

                            if count == 0: 
                                tProcess = next(list_process_iter)   
                                if tProcess == 8: 
                                    L = 'reduce_distance'                 
                                if tProcess == 7 or tProcess == 8: 
                                    step_mode = 1 
                                if tProcess == 9: 
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
            old_gray = gray_frame.copy()
            # press 'r' to reset the window
            key = cv.waitKey(1)
            if key == ord("s"):
                cv.imwrite(f'reference_img/Ref_img{frame_counter}.png', img)
            if key == ord("q"):
                break
            fps = frame_counter/(time.time()-starting_time)
            AiPhile.textBGoutline(frame, f'FPS: {round(fps,1)}', (30,40), scaling=0.6)
            # cv.imshow("Streaming", frame)
        # cv.destroyAllWindows()
        # cap.release()
try:
    t = time.time()
    t1 = threading.Thread(target=ReceiveData, args=(q, ))
    t2 = threading.Thread(target=doOpenCV, args=(q, q2))
    t3 = threading.Thread(target=SubMessage)
    # t4 = threading.Thread(target=PubMessage)
    t1.start()
    t2.start()
    t3.start()
    # t4.start()
    t1.join()
    t2.join()
    t3.join()
    # t4.join()
    print("All thread join!")
except KeyboardInterrupt:
    ser.close()           