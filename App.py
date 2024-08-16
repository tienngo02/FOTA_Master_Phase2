import subprocess
import time
import json
import threading

import ftplib
import ssl
import os
import io
import sys
import time
import paho.mqtt.client as mqtt
from Security import Security

import serial.tools.list_ports
import serial

import datetime
import shlex
import cv2
import numpy as np
import json

import asyncio
import websockets
import ftplib
import ssl

print()
print("================================")
print("App is running...")

JSONFILE = 'Version_information_file.json'

PYTHON = 'python3.12'
APP = 'App.py'
BOOT = 'Boot.py'
CLIENT = 'FOTA_Client.py'

# global ser
# newClient = True
# stop_thread = False
# thread = None
'''
=========================================================
Server communication and security
=========================================================
'''

File_path = os.path.abspath(__file__)
Folder_Dir = os.path.dirname(File_path)
sys.path.append(Folder_Dir)

CONTROL_TOPIC='/jetson/control'


# CAMERA
def run_camera_process():
    stdout_file_name  = "/dev/stdout"
    gst_pipeline = f'gst-launch-1.0 --quiet nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=3264,height=2464,framerate=21/1,format=NV12 ! nvvidconv flip-method=0 ! video/x-raw,width=816,height=616,format=BGRx ! filesink location={stdout_file_name}'

    process = subprocess.Popen(shlex.split(gst_pipeline), stdout=subprocess.PIPE)
    return process


def capture_image(process):
    raw_image = process.stdout.read(816 * 616 * 4)     
    image = np.frombuffer(raw_image, np.uint8).reshape((616, 816, 4))
    _,buffer = cv2.imencode('.jpg', image)

    return buffer


class MyFTP_TLS(ftplib.FTP_TLS):
    """Explicit FTPS, with shared TLS session"""
    def ntransfercmd(self, cmd, rest=None):
        conn, size = ftplib.FTP.ntransfercmd(self, cmd, rest)
        if self._prot_p:
            conn = self.context.wrap_socket(conn,
                                            server_hostname=self.host,
                                            session=self.sock.session)  # this is the fix
        return conn, size
    

class Cloud_COM:
    def __init__(self) -> None:
        self.host = 'begvn.home'
        self.FTPport = 21
        self.MQTTPort = 8883
        self.MQTTProtocol = "tcp"
        self.user = 'user1'
        self.passwd = '123456'
        self.acct = 'Normal'
        self.SensorTopic = 'jetson/jetsonano/imu'
        self.BatteryTopic = 'jetson/jetsonano/batt'
        self.StreamTopic = 'jetson/jetsonano/stream'
        self.ca_cert_path = Folder_Dir + '/certs/ca.crt'
        self.isStreaming = False
        # print(self.ca_cert_path)
        self.ssl_context = ssl.create_default_context(cafile=self.ca_cert_path)
        self.ftps = MyFTP_TLS(context=self.ssl_context)
        # self.ftps.context
        self.ftps.set_debuglevel(0)
        self.MQTTclient = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                    protocol=mqtt.MQTTv5,
                    transport=self.MQTTProtocol)
        self.MQTTclient.tls_set(ca_certs=self.ca_cert_path)
        self.isFTPConnected = False
        self.isMQTTConnected = False
        self.NewMasterApp = False
        self.NewMasterBoot = False
        self.NewClient = False
        # self.isReceiveResponse = False
        

    def __del__(self):
        self.FTP_Disconnect()

    # WEBSOCKET
    async def manage_pings(self,websocket):
        """Send pings and handle pongs asynchronously."""
        # await asyncio.sleep(0.5)
        try:
            await websocket.send(bytearray([1]))
            while True:
                ping = await websocket.recv()
                print('Rcv ping')
                if ping == '1':
                    await websocket.send(bytearray([1]))
                    print('ping: ',(ping))
        except asyncio.CancelledError:
            print("Ping task cancelled")
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed, cannot send pings.")
        except Exception as e:
            print('Error: ',e)


    async def listen_for_messages(self, websocket, cap):
        """Listen for messages and pongs asynchronously."""
        try:
            while self.isStreaming:
                # print('Sending')
                # ret, frame = cap.read()
                # if not ret:
                #     break

                # _, buffer = cv2.imencode('.jpg', frame)
                # await websocket.send(buffer.tobytes())

                await websocket.send(capture_image(cap).tobytes())
                await asyncio.sleep(0.005)
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed by server.")
        except Exception as e:
            print('Error: ',e)


    async def websocket_client(self):
        try:
            uri = "wss://begvn.home:9090/stream"
            
            self.camera_process = run_camera_process()
            context = ssl.SSLContext(protocol=ssl.PROTOCOL_TLS_CLIENT)
            context.load_verify_locations(cafile='./ca.crt')
            # cap = cv2.VideoCapture(0)
            
            async with websockets.connect(uri,ssl=context,open_timeout=120) as websocket:
                print('Server created')
                # Create tasks for managing pings and receiving messages
                ping_task = asyncio.create_task(self.manage_pings(websocket))
                # listen_task = asyncio.create_task(listen_for_messages(websocket,cap))
                listen_task = asyncio.create_task(self.listen_for_messages(websocket,self.camera_process))

                # Wait for either task to complete
                done, pending = await asyncio.wait(
                    [listen_task,ping_task],
                    return_when=asyncio.FIRST_COMPLETED
                )

                # If here, one of the tasks has completed, cancel the others
                for task in pending:
                    task.cancel()
                if listen_task in done:
                #     # Handle specific completion, if needed
                    print("Message listening task completed")

        except Exception as e:
            print("websocket_client() error")
            print(e)


    def FTP_Connect(self):
        try:
            self.ftps.connect(self.host, self.FTPport)

            # print(self.ftps.getwelcome())
            # print(self.ftps.sock)

            self.ftps.auth()

            self.ftps.login(self.user, self.passwd, self.acct)

            self.ftps.set_pasv(True)
            self.ftps.prot_p()
            self.ftps.cwd("SW")
            self.isFTPConnected = True
        except:
            print("FTP Connect failed")
            return

    def FTP_Disconnect(self):
        if self.isFTPConnected == True:
            self.ftps.quit()
            self.isFTPConnected = False

    def MQTT_Connect(self):
        self.MQTTclient.username_pw_set(self.user, self.passwd)
        self.MQTTclient.connect(self.host,self.MQTTPort)
        self.MQTTclient.loop_start()
        self.MQTTclient.on_message = self.MQTT_On_message
        self.isMQTTConnected = True

    def MQTT_Disconnect(self):
        # self.MQTTclient.loop_stop()
        self.MQTTclient.disconnect()
        self.isMQTTConnected = False

    def send_image(self):
        asyncio.run(self.websocket_client())
        print("End asyncio")
        if self.camera_process :
            self.camera_process.terminate()
            self.camera_process.wait()
        self.isStreaming = False
        print("End websockets")


    def MQTT_On_message(self,client, userdata, message):
        print(message.payload.decode())
        payload = message.payload.decode()
        topic = message.topic
        if topic == "SW/Jetson/FOTA_Master_App" or topic == "SW/Jetson/FOTA_Master_Boot" or topic == "SW/Jetson/FOTA_Client":
            self.NotifiSW_CB(self,payload)
        elif topic == 'jetson/jetsonano/stream':
            print(self.isStreaming)
            if payload == 'start' and self.isStreaming == False:
                #start streaming
                print('Connecting...')
                
                self.isStreaming = True
                # send_image_thread = threading.Thread(target=self.send_image, args=(self,))
                send_image_thread = threading.Thread(target=self.send_image)
                send_image_thread.daemon = True
                send_image_thread.start()

            elif payload == 'stop':
                #stop streaming
                if self.isStreaming :
                    self.isStreaming = False
                    if self.camera_process :
                        self.camera_process.terminate()
                        self.camera_process.wait()

        elif topic == CONTROL_TOPIC:
            control_FOTA_Client(payload)



    def startWaitNewSW(self,NewSWCB):
        try:
            if self.isFTPConnected == False:
                self.FTP_Connect()
            if self.isMQTTConnected == False:
                self.MQTT_Connect()
            self.NotifiSW_CB = NewSWCB
            self.MQTTclient.subscribe("SW/Jetson/#",qos=2)
            return True
        except Exception as e:
            print("Connect error: ",e)
            return False
    

    def startWaitStreamReq(self):
        try:
            if self.isMQTTConnected == False:
                self.MQTT_Connect()
            self.MQTTclient.subscribe(self.StreamTopic,qos=2)
            return True
        except Exception as e:
            print("Connect error: ",e)
            return False

    def startControl(self):
        if self.isMQTTConnected == False:
            self.MQTT_Connect()
        self.MQTTclient.subscribe(CONTROL_TOPIC,qos=2)

    def GetNewSW(self,SWname: str):
        try:
            Unverified_SW_io = io.BytesIO()
            self.ftps.retrbinary('RETR ' + SWname,Unverified_SW_io.write)
            Unverified_SW_io.seek(0)
            Unverified_SW = Unverified_SW_io.read()
            print(len(Unverified_SW_io.read()))
            Verified_SW = Security.Verify_Decrypt_SW(Unverified_SW)
            return Verified_SW
        except Exception as e:
            print("Failed to get new SW, e: ",e)
            return 
        

    def Publish_Sensor(self,DataJSON: str):
        try:
            temp = self.MQTTclient.publish(self.SensorTopic,DataJSON,qos=2)
            # print(temp)
        except:
            print('Can not publish')
            pass

    def Publish_Batt(self,DataJSON: str):
        try:
            self.MQTTclient.publish(self.BatteryTopic,DataJSON,qos=1)
        except:
            print('Can not publish')
            pass


def NewSW_CB(Cloud, Swname):
    # global isProcessing
    # if isCBProcessing == True:
    #     return
    try:
        print("CB: ", Swname)
        parts = Swname.split('_')
        file_name = '_'.join(parts[1:-1])  # Join parts excluding the first and last
        version = float(parts[-1].lstrip('v'))  # Remove the leading 'v' from the last part

        version_control_obj = Version_File_Control()
        running, non_running = version_control_obj.read_2latest_version(file_name)

        if version > running and version > non_running:
            print('Download: ' + Swname)
            New_SW = Cloud.GetNewSW(Swname)
            if New_SW:
                new_file_name = file_name + '_new.py'
                with open(new_file_name, "wb") as file:
                    file.write(New_SW)
                version_control_obj.update_version(file_name, version)
        #         activate_newSW(file_name)
                
        # elif version == non_running and version_control_obj.activate(file_name):
        #     activate_newSW(file_name)

    except Exception as e:
        print("NewSW_CB() error: ", e)


def activate_newSW(file_name):
    print(datetime.datetime.now())
    print(file_name)
    # global stop_thread
    # global thread
    if file_name == 'FOTA_Master_App':
        # stop_thread = True
        # thread.join()
        subprocess.Popen([PYTHON, BOOT, 'activate_App'])
        exit()

    elif file_name == 'FOTA_Master_Boot':
        # stop_thread = True
        # thread.join()
        subprocess.Popen([PYTHON, BOOT, 'activate_Boot'])
        exit()

    elif file_name == 'FOTA_Client':
        # global newClient
        # global ser
        # ser = connect_serial_port()
        # if ser:
        #     time.sleep(1)
        #     byteRead = ser.inWaiting()
        #     if byteRead > 0:
        #         data = ser.read(byteRead)
        #         data_value = [b for b in data]
        #     newClient = True
        while True: 
            if send_msg(NOTIFY_NEW_SW):
                break
            time.sleep(0.001)

    else:
        print('Invalid file name')


def connectToServer():
    connectCount = 0
    isConnect = Cloud.startWaitNewSW(NewSW_CB) 
    isConnect = isConnect and Cloud.startWaitStreamReq()
    while connectCount < 5 :
        if isConnect:
            break
        else:
            print("Connect server error, retrying")
            time.sleep(5)
            isConnect = Cloud.startWaitNewSW(NewSW_CB)
            isConnect = isConnect and Cloud.startWaitStreamReq()
            connectCount += 1

    if not isConnect:
        print("Can not connect to server")

'''
=========================================================
Version file control
=========================================================
'''

class Version_File_Control:
    data = None

    def __init__(self):
        with open(JSONFILE, 'r') as file:
            self.data = json.load(file)
        
    def read_2latest_version(self, file_name):
        return self.data[file_name]['running'], self.data[file_name]['non-running']
        
    def update_version(self, file_name, version):
        self.data[file_name]['non-running'] = version
        self.data[file_name]['activate'] = True
        with open(JSONFILE, 'w') as file:
            json.dump(self.data, file, indent=4)

    def activate(self, file_name):
        return self.data[file_name]['activate']

    def deactive(self, filename):
        self.data[filename]['activate'] = False
        with open(JSONFILE, 'w') as file:
            json.dump(self.data, file, indent=4)


'''
=========================================================
UART Communication
=========================================================
'''

NOTIFY_NEW_SW = bytes([1, 120, 0, 0, 0, 0])
RESPONSE_CONFIMATION = bytes([1, 121, 0, 0, 0, 0])
REQUEST_FLASH_SW = bytes([1, 122, 0, 0, 0, 111])
FLASH_SUCCESS_YET = bytes([1, 123, 0, 0, 0, 0])


def getPort():
    ports = serial.tools.list_ports.comports()
    N = len(ports)
    commPort = "None"

    for i in range(0, N):
        port = ports[i]
        strPort = str(port)
        print(strPort)
        # if "S" in strPort:
        splitPort = strPort.split(" ")
        commPort = (splitPort[0])
    return commPort


MAX_RETRIES = 5
RETRY_DELAY = 5


def connect_serial_port():
    attempt = 0
    while attempt < MAX_RETRIES:
        try:
            seri = serial.Serial(port=getPort(), baudrate=115200, parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS, timeout=1)
            print("Open successfully")
            return seri

        except serial.SerialException as e:
            attempt += 1
            print(f"Attempt {attempt} failed")
            if attempt < MAX_RETRIES:
                print(f"Retrying in {RETRY_DELAY} seconds...")
                time.sleep(RETRY_DELAY)
            else:
                return None


# ser = connect_serial_port()
TIMEOUT = 5
startTime = 0
isFlashSuccess = False

def flash_SW():

    # global newClient
    global isFlashSuccess
    global client_pause_event
    global activate_pause_event

    client_pause_event.clear()
    activate_pause_event.clear()

    bytesRead = ser.inWaiting()
    ser.read(bytesRead)
    ser.close()
    # newClient = False
    # time.sleep(1)
    print("Flash SW for FOTA Client")
    # exit()

    subprocess.run([PYTHON, BOOT, 'activate_Client'])
    
    ser = connect_serial_port()
    time.sleep(1)
    if ser:
        byteRead = ser.inWaiting()
        if byteRead > 0:
            data = ser.read(byteRead)
            # data_value = [b for b in data]
            # print(data)
        client_pause_event.set()
        
        while True:
            if send_msg(FLASH_SUCCESS_YET):
                print("Sent: New client flash success yet?")
                break
            time.sleep(0.001)

        startTime = time.time()
        while True:
            current = time.time()
            if isFlashSuccess:
                print("New client flash success")
                # ser.close()
                isFlashSuccess = False
                activate_pause_event.clear()
                break

            if current - startTime > TIMEOUT:
                print("New client error")
                bytesRead = ser.inWaiting()
                ser.read(bytesRead)
                if ser :
                    ser.close()
                # global stop_thread
                # global thread
                # stop_thread = True
                # thread.join()
                subprocess.Popen([PYTHON, BOOT, 'rollback_Client'])
                
                exit()

            # receive_message()
            # time.sleep(0.001)


crc_table = []
for byte in range(256):
    crc = 0
    for _ in range(8):
        if (byte ^ crc) & 1:
            crc = (crc >> 1) ^ 0xA001
        else:
            crc >>= 1
        byte >>= 1
    crc_table.append(crc)
 
def calc_crc_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ crc_table[(crc ^ byte) & 0xFF]
    return crc.to_bytes(2, 'little')


def notify_New_SW():
    message = bytes([35]) + NOTIFY_NEW_SW + calc_crc_modbus(NOTIFY_NEW_SW)
    print('--------------------------')
    ser.write(message)
    print(message)
    print('--------------------------')
    time.sleep(0.01)


RUN = bytes([1, 110, 0, 0, 0, 30])
TURN_LEFT = bytes([1, 111, 0, 0, 20, 0])
TURN_RIGHT= bytes([1, 111, 0, 10, 20, 0])
STOP = bytes([1, 112, 0, 10, 0, 0,])
GO_BACKWARD = bytes([1, 113, 0, 0, 0, 30])


def send_msg(data):
    # create msg
    if ser.cts:
        message = bytes([35]) + data + calc_crc_modbus(data)
        ser.write(message)
        return True
    return False


YAW_PITCH_ROLL = bytes([1, 200, 0, 0, 0, 0, 0, 0])
ACCELEROMETER = bytes([210, 211, 212, 0, 0, 0, 0, 0])
GRYOSCOPE = bytes([220, 221, 222, 0, 0, 0, 0, 0])
GESTURE = bytes([1, 230, 0, 0, 0, 0, 0, 0])
BATTERY = bytes([1, 240, 0, 0, 0, 0, 0, 0])


FOTA_Client_data = {
    "deviceId": 1,
    "Orientation": {
        "yaw": 0,
        "pitch": 0,
        "roll": 0
    },
    "Accelerator": {
        "x": 0,
        "y": 0,
        "z": 0
    },
    "Gyroscope": {
        "x": 0,
        "y": 0,
        "z": 0
    },
    "Gesture": 0,
    "recordTime": 0
}

FOTA_Client_Battery = {
    "deviceId": 1,
    "Battery": {
        "Temperature": 0,
        "Capacity": 0
    },
    "recordTime": 0
}


def classify_msg(msg, Cloud):

    if msg[1] == RESPONSE_CONFIMATION[1]:
        print("Send function has been confirmed")

    elif msg[1] == REQUEST_FLASH_SW[1]:
        flash_SW()
    
    # FOTA Client response flash success
    elif msg[1] == 124:
        global isFlashSuccess
        isFlashSuccess = True

    elif msg[1] == YAW_PITCH_ROLL[1]:
        # print(msg)
        
        sign_byte = msg[2]
        yaw = msg[3]
        pitch = msg[4]
        roll = msg[5]

        if sign_byte % 10 == 1:
            roll = -roll
        sign_byte = int(sign_byte / 10)
        if sign_byte % 10 == 1:
            pitch = -pitch
        sign_byte = int(sign_byte / 10)
        if sign_byte % 10 == 1:
            yaw = -yaw

        FOTA_Client_data["Orientation"]["yaw"] = yaw
        FOTA_Client_data["Orientation"]["pitch"] = pitch
        FOTA_Client_data["Orientation"]["roll"] = roll

    # ACCELEROMETER Function code 210 -> 212
    elif int(msg[1] / 10) == 21 :
        accelerometer = FOTA_Client_data["Accelerator"]
        accelerometer_data = int.from_bytes(msg[2:6], byteorder='big', signed=True)

        if msg[1] == ACCELEROMETER[0]:
            accelerometer["x"] = accelerometer_data
        elif msg[1] == ACCELEROMETER[1]:
            accelerometer["y"] = accelerometer_data
        elif msg[1] == ACCELEROMETER[2]:
            accelerometer["z"] = accelerometer_data

    # GRYOSCOPE Function code 220 -> 222
    elif int(msg[1] / 10) == 22 :
        gryoscope = FOTA_Client_data["Gyroscope"]
        gryoscope_data = int.from_bytes(msg[2:6], byteorder='big', signed=True)

        if msg[1] == GRYOSCOPE[0]:
            gryoscope["x"] = gryoscope_data
        elif msg[1] == GRYOSCOPE[1]:
            gryoscope["y"] = gryoscope_data
        elif msg[1] == GRYOSCOPE[2]:
            gryoscope["z"] = gryoscope_data

    elif msg[1] == GESTURE[1]:
        FOTA_Client_data["Gesture"] = msg[5]
        FOTA_Client_data["recordTime"] = datetime.datetime.now().isoformat()
        
        # Send data to server
        json_data = json.dumps(FOTA_Client_data, indent=4)
        # print(json_data)
        Cloud.Publish_Sensor(json_data)

    elif msg[1] == BATTERY[1]:
        FOTA_Client_Battery["Battery"]["Temperature"] = msg[4]
        FOTA_Client_Battery["Battery"]["Capacity"] = msg[5]
        FOTA_Client_Battery["recordTime"] = datetime.datetime.now().isoformat()

        # Send data to server
        json_data = json.dumps(FOTA_Client_Battery, indent=4)
        
        Cloud.Publish_Batt(json_data)

    else :
        print("Invalid message")


def receive_message(Cloud):
    bytesToRead = ser.inWaiting()
    if bytesToRead > 0 :
        start_byte = ser.read(1)
        if start_byte == b'#' and bytesToRead >= 9:
            data_bytes = ser.read(8)
            message = [b for b in data_bytes]
            crc = calc_crc_modbus(message[0:6])
            if message[6] == crc[0] and message[7] == crc[1]:
                classify_msg(message, Cloud)
                return True
            else:
                print("Error message:", message)
                return False
        else:
            return False


# run_type = 0
recv_mess_start_time = time.time()
def send_client_data_loop(Cloud, client_pause_event):
    global recv_mess_start_time
    while True:
        if receive_message(Cloud):
            recv_mess_start_time = time.time()

        current_time =  time.time()
        if (current_time - recv_mess_start_time) > 5:
            print("TimeoutError: cannot receive message from FOTA Client")
            recv_mess_start_time =  time.time()
        
        client_pause_event.wait()


def control_FOTA_Client(command):
    if command == 'w':
        send_msg(RUN)
        print("RUN")
    elif command == 'a':
        send_msg(TURN_LEFT)
        print("TURN_LEFT")
    elif command == 'd':
        send_msg(TURN_RIGHT)
        print("TURN_RIGHT")
    elif command == 's':
        send_msg(GO_BACKWARD)
        print("GO_BACKWARD")


def start_client_thread(Cloud):
    global client_pause_event
    client_pause_event = threading.Event()
    client_pause_event.set()  # Initially allow the thread to run
    client_thread = threading.Thread(target=send_client_data_loop, args=(Cloud,client_pause_event,))
    client_thread.daemon = True
    client_thread.start()
    

'''
=========================================================
Main
=========================================================
'''


def handle_activate_newSW(activate_pause_event):
    while True:
        activate_pause_event.wait()

        version_control_obj = Version_File_Control()
        if version_control_obj.activate('FOTA_Master_Boot'):
            activate_newSW('FOTA_Master_Boot')
            
        elif version_control_obj.activate('FOTA_Master_App'):
            activate_newSW('FOTA_Master_App')
            
        elif version_control_obj.activate('FOTA_Client'):
            activate_newSW('FOTA_Client')
            # version_control_obj.deactive('FOTA_Client')

        time.sleep(3)


if __name__ == '__main__':
    try:
        print("New APP ne")
        print("Path: ", sys.path)
        Cloud = Cloud_COM()
        connectToServer()
        Cloud.startControl()

        activate_pause_event = threading.Event()
        activate_pause_event.set()
        activate_thread = threading.Thread(target=handle_activate_newSW, args=(activate_pause_event,))
        activate_thread.daemon = True

        ser = connect_serial_port()
        # time.sleep(1)
        if ser:
            byteRead = ser.inWaiting()
            if byteRead > 0:
                data = ser.read(byteRead)
                data_value = [b for b in data]

            start_client_thread(Cloud)
        else:
            print("Can not open the port.")

        # activate_thread.start()
        
        while True:
            # if newClient:
            #     receive_message()
            time.sleep(0.001)
    except Exception as e:
        # print('App is error, rollback app')
        print(e)
        # subprocess.Popen(['python3.12', 'Boot.py', 'rollback_App'])
        exit()