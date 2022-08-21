#!/usr/bin/python3
# -*- coding: utf-8 -*-

from logging import debug
import os
import sys
import signal
import time
import serial
import struct
import _thread
import traceback
from queue import Queue
import json
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import modbus
import settings
import logger
import serviceReport

sendQueue = Queue(maxsize=0)
current_sec_time = lambda: int(round(time.time()))
usleep = lambda x: time.sleep(x / 1000000.0)

exit = False
serialPort = None


def signal_handler(_signal, frame):
    global exit

    print('You pressed Ctrl+C!')
    exit = True


def printHexString(str):
    for char in str:
        print("%02X " % (ord(char)), end='')
    print()


def printHexByteString(recvMsg):
    for x in recvMsg:
        print("%02X " % x, end='')
    print()
    # print(" msg length: %d" % len(recvMsg))


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        client.subscribe([(settings.MQTT_TOPIC_REPORT, 1), (settings.MQTT_TOPIC_CHECK, 1)])
    else:
        print(("ERROR: MQTT Client connected with result code %s " % str(rc)))


# The callback for when a PUBLISH message is received from the server
def on_message(client, userdata, msg):
    print(('ERROR: Received ' + msg.topic + ' in on_message function' + str(msg.payload)))


def on_message_homelogic(client, userdata, msg):
    #print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2] #huis/RFXtrx/KaKu-12/out
    cmnd = deviceName.split("-") #KaKu-12

    # KaKu-12
    if cmnd[0] == "KaKu":
        #print("Activate KaKu WCD: %s" % cmnd[1])
        # setKaKu(int(cmnd[1]), msg.payload)
        pass


def openSerialPort():
    global exit
    try:
        ser = serial.Serial(port=settings.serialPortDevice,  # port='/dev/ttyACM0',
                            baudrate=settings.serialPortBaudrate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("rflink_mqtt: Successfully connected to serial port %s" % (settings.serialPortDevice)))

        return ser

    # Handle other exceptions and print the error
    except Exception as arg:
        print("%s" % str(arg))
        # traceback.print_exc()

        #Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_NOTHING, 'Serial port open failure on port %s, wrong port or USB cable missing' % (settings.serialPortDevice))

        # Suppress restart loops
        time.sleep(900) # 15 min
        exit = True


def closeSerialPort(ser):
    ser.close()


def serialPortThread(serialPortDeviceName, serialPort):
    global exit
    global checkMsg

    # Wait a while, the OS is probably testing what kind of device is there
    # with sending 'ATEE' commands and others
    time.sleep(2)
    serialPort.reset_input_buffer()

    # Ask for board Id
    print("serialPortThread started")
    serialPort.setRTS(0) # Disable RS485 send
    powerCntAdd = 1000 # Send power directly
    powerAvgAdd = 0
    powerAvgSend = 0
    lastFeedInMode = None

    while not exit:
        try:
            if serialPort.isOpen():
                recvMsg = serialPort.read(110)
                # "".join([chr(i) for i in recvMsgWithoutCRC])
            else:
                recvMsg = ""
                time.sleep(5)
                print("Serial Port not open")

            # Check if something is received
            if recvMsg != b"":
                msgLen = len(recvMsg)
                # print("msgLen: %d msg: " % msgLen , end='')

                if msgLen == 51:
                    # Correct response from ADL3000 meter
                    # print("51 - ", end='')
                    pass
                elif msgLen == 59:
                    # Received command and response in 1 msg: Remove the command, leave the response
                    # print("59 - ", end='')
                    recvMsg = recvMsg[8:]
                    # Correct the msgLen with - 8
                    msgLen = 51 # 59-8
                elif msgLen == 8:
                    # Received the command to the ADL3000 meter: ignore this
                    # print("8 - ", end='')
                    continue
                else:
                    # Unknown msgLen
                    # print("Unknown msgLen: %d msg: " % msgLen)
                    # print("U")
                    continue

                # Check the receive msg CRC
                if not modbus.checkRecvMsgCRC(recvMsg):
                    # print(" - Wrong CRC")
                    # printHexByteString(recvMsg)
                    continue

                # Check msgLen (+5 bytes=modBusAddr,Cmnd,dataLength,CRChigh,CRClow)
                if msgLen != (recvMsg[2] + 5):
                    # print("Wrong msgLen!", end='')
                    # printHexByteString(recvMsg)
                    continue

                # Reset the Rx timeout timer
                serviceReport.systemWatchTimer = current_sec_time()

                sensorData = {}

                # printHexByteString(recvMsg)
                # Message format:
                # 01 03 2E 09 58 09 54 09 5B+00 59-00 9B-00 77+00 9A-01 13-00 E7-02 95+00 0E-FF 9C-00 3F-FF EA+00 D6-01 74-01 1F-03 69+00 48-00 49-00 50-00 63+01 F4+3A 15  - Len 51

                # Ad-Cm-??+--Ua----Ub---Uc--+--Ia----Ib----Ic-+----Actief vermogen----+---Reactief vermogen---+----Blind vermogen-----+---Vermogensfactor-----+--Hz-+-CRC-+

                # # 0+1: Addr+Function
                # # 2  : ????????????????????
                # 0x61:3+4: 2 bytes: Uan
                # i = 3
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Ua: %.1f Volt" % (float(val)/10))
                # # 0x62:5+6: 2 bytes: Ubn
                # i = 5
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Ub: %.1f Volt" % (float(val)/10))
                # # 0x63:7+8: 2 bytes: Ucn
                # i = 7
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Uc: %.1f Volt" % (float(val)/10))

                # # 0x64:9+10: 2 bytes: Ia
                # i = 9
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Ia: %.3f A" % (float(val)/1000))
                # # 0x65:11+12: 2 bytes: Ib
                # i = 11
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Ib: %.3f A" % (float(val)/1000))
                # # 0x66:13+14: 2 bytes: Ic
                # i = 13
                # val = struct.unpack(">h", recvMsg[i:i+2])[0]
                # print("Ic: %.3f A" % (float(val)/1000))

                # # 0x67:15+16: 2 bytes: Pa
                # i = 15
                # val = struct.unpack(">h", recvMsg[i:i + 2])[0]
                # print("Pa: %d W " % val)
                # sensorData['Pa'] = val

                # # 0x68:17+18: 2 bytes: Pb
                # i = 17
                # val = struct.unpack(">h", recvMsg[i:i + 2])[0]
                # print("Pb: %d W  " % val)
                # sensorData['Pb'] = val

                # # 0x69:19+20: 2 bytes: Pc
                # i = 19
                # val = struct.unpack(">h", recvMsg[i:i + 2])[0]
                # print("Pc: %d W  " % val)
                # sensorData['Pc'] = val

                # 0x6A:21+22: 2 bytes: total active power (0.00kW)
                i = 21
                powerTotal = int(struct.unpack(">h", recvMsg[i:i + 2])[0]) #
                # print("Ptot: %d W  " % powerTotal)

                sensorData['Ptot'] = powerTotal
                feedInMode = (powerTotal < 0)

                powerAvgAdd += powerTotal
                powerCntAdd += 1
                powerAvg = powerAvgAdd / powerCntAdd
                powerAvgDiff = abs(powerAvg / 2) # 25% of powerTotal
                # print("powerAvgDiff: %d" % powerAvgDiff)
                if (feedInMode != lastFeedInMode) or (powerCntAdd >= 7) or (powerTotal > (powerAvg + powerAvgDiff)) or (powerTotal < (powerAvg - powerAvgDiff)):
                    powerAvgSend = powerAvg
                    powerCntAdd = 1
                    powerAvgAdd = powerTotal
                    powerAvg = powerTotal
                    sendPower = True
                else:
                    sendPower = False

                # print("Pavg: %3dW" % powerAvg)
                sensorData['Pavg'] = int(powerAvgSend)

                lastFeedInMode = (powerTotal < 0)
                # print("feedInMode: %d" % feedInMode)

                # if powerCntAdd >= 30:
                #     print(" -> reset powerAvg after 30 counts")
                # elif powerTotal > (powerAvg + powerAvgDiff):
                #     print(" -> reset powerAvg (> 20%)")
                # elif powerTotal < (powerAvg - powerAvgDiff):
                #     print(" -> reset powerAvg (< 20%)")
                # else:
                #     print()

                # msgLen = len(recvMsg)
                # while i < (msgLen - 2):
                #     # Process all data in msg - 4: addr,cmnd and 2xbyte CRC
                #     valStr = recvMsg[i:i+2]
                #     # print(valStr)
                #     val = struct.unpack("<h", valStr)[0]
                #     # print("%s: %d" % (valStr, val))
                #     i += 2

                if sendPower:
                    mqttTopic = "huis/ADL3000/ADL3000-power-meter/power"
                    mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)


            # Check if there is any message to send
            if not sendQueue.empty():
                serialPort.setRTS(1) # Enable RS485 send
                sendMsg = sendQueue.get_nowait()
                msgLen = len(sendMsg)
                # print(("SendMsg: %s" % sendMsg))
                # printHexByteString(sendMsg)
                serialPort.write(sendMsg)
                # 9600 baud->1bit=104,1667uS
                # 1 byte=10bits->10*104,1667=1041,667uS
                usleep(msgLen * 1041.6667)
                # msleep(8)

                serialPort.setRTS(0) # Disable RS485 send
                # print("Tx ready")

        # In case the message contains unusual data
        except ValueError as arg:
            print(arg)
            traceback.print_exc()
            time.sleep(1)

        # Quit the program by Ctrl-C
        except KeyboardInterrupt:
            print("Program aborted by Ctrl-C")
            exit()

        # Handle other exceptions and print the error
        except Exception as arg:
            print("Exception in serialPortThread serialPortDeviceName:%s" % serialPortDeviceName)
            print("%s" % str(arg))
            traceback.print_exc()
            time.sleep(120)


def sendModbusMsg(sendMsg, modBusAddr):
    sendMsgList = list(sendMsg)
    sendMsgList[0] = chr(modBusAddr)
    # print(sendMsgList)
    # print("modBusAddr=%d" % modBusAddr, end='')
    # print(" -> send request to Storion T10: ", end='')
    sendMsg = ''
    for element in sendMsgList:
        # print("%02X " % ord(element), end='')
        sendMsg += element
    # printHexString(sendMsg)
    request = sendMsg + modbus.calculateCRC(sendMsg)
    # printHexString(request)
    sendQueue.put(request.encode('latin'))


def print_time(delay):
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print("%s" % (time.ctime(time.time())))


###
# Initalisation ####
###
logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings.serialPortDevice)

# Give Home Assistant and Mosquitto the time to startup
time.sleep(2)

serialPort = openSerialPort()

if serialPort is None:
    print("Program terminated.")
    sys.exit(1)
else:
    # Create the serialPortThread
    try:
        # thread.start_new_thread( print_time, (60, ) )
        _thread.start_new_thread(serialPortThread, (settings.serialPortDevice, serialPort))
    except Exception:
        print("Error: unable to start the serialPortThread")
        sys.exit(1)

# Start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings.MQTT_TOPIC_REPORT,   on_message_homelogic)
client.message_callback_add(settings.MQTT_TOPIC_CHECK,     serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect(settings.MQTT_ServerIP, settings.MQTT_ServerPort, 60)
client.loop_start()

# The thread is waiting 2 sec, so also wait here before sending msgs
time.sleep(2)

try:
    while not exit:
        time.sleep(5) #[5s]

finally:
    if serialPort is not None:
        serialPort.setRTS(0) # Disable RS485 send
        closeSerialPort(serialPort)
        print('Closed serial port')

print("Clean exit!")
