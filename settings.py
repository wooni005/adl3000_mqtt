import logging

MQTT_ServerIP     = "192.168.5.248"
MQTT_ServerPort   = 1883

serialPortDevice  = '/dev/ttyUSB1'
serialPortBaudrate = 9600
tcpipServerAddress = ('192.168.5.225', 4001)
LOG_FILENAME      = "/home/pi/log/adl3000_mqtt.log"
LOG_LEVEL = logging.INFO  # Could be e.g. "INFO", "DEBUG" or "WARNING"

MQTT_TOPIC_CHECK     = "huis/ADL3000/RPiInfra/check"
MQTT_TOPIC_REPORT    = "huis/ADL3000/RPiInfra/report"
