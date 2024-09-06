import sys
import hub
from hub import port         # Port module to set / get Port from LEGO hub
from hub import display      # Display module to control LEGO hub Display Screen
from hub import Image        # Image module to use built-in image
from hub import button       # To use Button on LEGO hub
from hub import led          # To control LED on LEGO hub
from hub import motion       # For current motion status on LEGO hub
from hub import sound
from hub import USB_VCP
from utime import sleep_ms, ticks_ms  # Import delay function and ticks for timing
import micropython

import math
# Setup Ports, Motors, Sensors
MotorA = port.A.motor        # MotorA defines the port A of Hub
MotorB = port.B.motor        # MotorB defines the port B of Hub
MotorB.default(max_power = 100, stop = 2)

# Setup necessary Variables
command = bytes([])
message = bytes([])
new_SW = False          #Check new SW available
safe_State = True       #Safe_State for flashing new SW
motor_running = False   #Check if any motor is busy
motor_end_time = 0      #The limit time for running car to stop
angle = 0               #Setup Angle for reset and go forward
direct = 0
safe_period = 0         #Set time to maintain Safe_state
vcp = USB_VCP(0)
vcp.setinterrupt(-1)        #Set USB Virtual ComPort
vcp.init(flow = USB_VCP.RTS | USB_VCP.CTS)
last_sensor_send = 0
last_hub_send = 0
# Timer interval (milliseconds)
timer_interval = 0
sensor_end_time = 10
hub_end_time = 1000

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
    

def set_timer_interval(interval):
    global timer_interval
    timer_interval = interval

def request_Flash_SW():     #When car in Safe State, it will request the Master to flash new SW
    global new_SW
    global vcp

    data = bytes([1, 122, 0, 0, 0, 111])
    message = bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))

    # hub.led(8)
    vcp.write(message)
    new_SW = False
    vcp.close()
    hub.power_off(fast=True, restart=True)

def response_Confirmation():    #When car receive Notify new SW successfully, it will response the Confirmation to Master
    global new_SW
    global vcp

    new_SW = True
    data = bytes([1, 121, 0, 0, 0, 0])
    message = bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    # hub.led(5)
    vcp.write(message)
    
    if safe_State == True:
        request_Flash_SW()

def response_Flash_Status():    #The Master will ask whether new SW is flashed successfully or not
    # hub.led(3)
    data = bytes([1, 124, 0, 0, 0, 0])
    message = bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

def restart_command():
    hub.power_off(fast=True, restart=True)


def send_Hub_Info():
    data = bytes([1, 240, 0, 0, int(hub.battery.temperature()), hub.battery.capacity_left()])
    message = bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

def send_IMU_Sensor():
    # global message
    rotations = motion.yaw_pitch_roll()
    sign_byte = 0
    if rotations[0] < 0 and rotations [1] < 0 and rotations[2] < 0 :
        sign_byte = 111
    elif rotations[0] < 0 and rotations [1] < 0 and rotations[2] >= 0 :
        sign_byte = 110
    elif rotations[0] < 0 and rotations [1] >= 0 and rotations[2] < 0 :
        sign_byte = 101
    elif rotations[0] < 0 and rotations [1] >= 0 and rotations[2] >= 0 :
        sign_byte = 100
    elif rotations[0] >= 0 and rotations [1] < 0 and rotations[2] < 0 :
        sign_byte = 11
    elif rotations[0] >= 0 and rotations [1] < 0 and rotations[2] >= 0 :
        sign_byte = 10
    elif rotations[0] >= 0 and rotations [1] > 0 and rotations[2] < 0 :
        sign_byte = 1
    elif rotations[0] >= 0 and rotations [1] >= 0 and rotations[2] >= 0 :
        sign_byte = 0
    
    data = bytes([1, 200, sign_byte, int(math.fabs(rotations[0])), int(math.fabs(rotations[1])), int(math.fabs(rotations[2]))]) 
    # print (rotations[0])
    # print (rotations[1])
    # print (rotations[2])   
    message = bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)
    
    accelerations = motion.accelerometer() #range {-16384; 16384}

    offset_value = accelerations[0] + 16384
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 210, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    offset_value = accelerations[1] + 16384
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 211, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    offset_value = accelerations[2] + 16384
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 212, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    

    rates = motion.gyroscope() #range {-500; 500}
    # print("Gyro:")
    # print(rates[0])
    # print(rates[1])
    # print(rates[2])

    offset_value = rates[0] + 500
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 220, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    offset_value = rates[1] + 500
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 221, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    offset_value = rates[2] + 500
    mapped_byte_array = offset_value.to_bytes(2, 'big')
    data = bytes([1, 222, 0, 0]) + mapped_byte_array
    message= bytes([35]) + data + bytes(calc_crc_modbus(data[0:6]))
    vcp.write(message)

    
    gesture = motion.gesture()
    if gesture == None:
        gesture = 0
    data = bytes([1, 230, 0, 0, 0, gesture])
    message = bytes([35]) + data + calc_crc_modbus(data[0:6])
    vcp.write(message)

def run_Forward():
    global motor_running
    global safe_State
    MotorB.run_at_speed(max(command[5] - 100, -100))
    motor_running = True
    if (command[5] - 100 == 0):
        motor_running = False
    
    #sleep_ms(1000)
    #MotorB.brake()

def turn_LeftRight():
    global angle, direct, motor_running
    if command[3] == 0:
        MotorA.run_for_degrees(command[4], speed = 50)
        motor_running = True
        #sleep_ms(10)
        direct = -50
    else:
        MotorA.run_for_degrees(command[4], speed = -50)
        motor_running = True
        #sleep_ms(10)
        direct = 50
    angle = command[4]
 
def stop_Motor():
    global motor_running
    MotorB.run_at_speed(0)
    motor_running = False

# def run_Backward():
#     MotorB.run_at_speed(command[5])

def hub_Beep():
    hub.sound.beep(freq = 1000, time = 500)

def classify_Command(command):  #This function will define which kind of message that the car receive is
    # hub.led(1)
    if command[1] == 120:
        response_Confirmation()
    elif command[1] == 123: 
        response_Flash_Status()
    elif command[1] == 110:
        # print("Current Speed:", command[5] - 100)

        run_Forward()

    elif command[1] == 111:
        turn_LeftRight()
    elif command[1] == 112:
        stop_Motor()
    # elif command[1] == 113:
    #     run_Backward()
    elif command[1] == 114:
        hub_Beep()
    elif command[1] == 125:
        restart_command()

def handle_VCP():
    # print("Handle Start")
    global vcp
    global command
    global safe_State
    
    if vcp.isconnected():
        if vcp.any(): #aAAe

            #hub.led(8)
            start_char = vcp.read(1)
            if start_char == b'#':

                command = vcp.read(8)
                # print(command)
                if command and len(command) >= 8:
                     
                    if command[6:8] == calc_crc_modbus(command[0:6]):
                        # hub.led(3)
                        
                        classify_Command(command)
                else:
                    # hub.led(8)
                    pass
        if safe_State == True and new_SW == True:
            request_Flash_SW()

    # print("Handle End")


def timer_function(callback):
    global timer_interval
    start_time = ticks_ms()
    
    while True:
        current_time = ticks_ms()
        if current_time - start_time >= timer_interval:
            start_time = current_time
            callback()
            break

# current = MotorA.get()
# if (current[1] < 0):
#     MotorA.run_to_position(50)
# else:
#     MotorA.run_to_position(-50)


motion.yaw_pitch_roll(yaw_preset = 0)

while (True):
    hub.led(3)
    handle_VCP()
    if not motor_running:
            safe_State = True
    else: 
        safe_State = False 
    
    
    # Get the current time
    current_time = ticks_ms()
    
    # Check if 10 milliseconds have passed
    if current_time - last_sensor_send >= sensor_end_time:
        # Update the start time
        last_sensor_send = current_time

        # Print the current time
        # Print sensor data
        send_IMU_Sensor()
        
    # Check if 1 seconds have passed
    if current_time - last_hub_send >= hub_end_time:
        # Update the start time
        last_hub_send = current_time

        # Print sensor data
        send_Hub_Info()