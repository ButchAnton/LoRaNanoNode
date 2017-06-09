#
# LoRaNanoNode.py
#

import os
import socket
import time
import struct
from network import LoRa
from uos import urandom
from machine import I2C
from LoPy_BME280 import bme280

#
# Set up the BME280
#

i2c = I2C(0, I2C.MASTER, baudrate=100000)
bme = bme280.BME280(i2c=i2c)

# A basic package header
# B: 1 byte for the deviceId
# B: 1 byte for the pkg size
# B: 1 byte for the messageId
# %ds: Formated string for string
_LORA_PKG_FORMAT = "!BBB%ds"

# A basic ack package
# B: 1 byte for the deviceId
# B: 1 byte for the pkg size
# B: 1 byte for the messageId
# B: 1 byte for the Ok (200) or error messages
_LORA_PKG_ACK_FORMAT = "BBBB"

# This device ID, use different device id for each device
_DEVICE_ID = 0x01
_MAX_ACK_TIME = 5000
_RETRY_COUNT = 3

# Let the world know we're starting up.

print("Starting LoRaNanoNode on device %d" % _DEVICE_ID)

# Open a Lora Socket, use tx_iq to avoid listening to our own messages
lora = LoRa(mode=LoRa.LORA, tx_iq=True)
lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
lora_sock.setsockopt(socket.SOL_LORA, socket.SO_DR, 4)
lora_sock.setblocking(False)

# Method to increase message id and keep in between 1 and 255
msg_id = 0
def increase_msg_id():
    global msg_id
    msg_id = (msg_id + 1) & 0xFF

# Method for acknoledge waiting time keep
def check_ack_time(from_time):
    current_time = time.ticks_ms()
    return (current_time - from_time > _MAX_ACK_TIME)

# Method to send messages
def send_msg(msg):
    global msg_id
    retry = _RETRY_COUNT
    while (retry > 0 and not retry == -1):
        retry -= 1
        pkg = struct.pack(_LORA_PKG_FORMAT % len(msg), _DEVICE_ID, len(msg), msg_id, msg)
        bytes_sent = lora_sock.send(pkg)
        print(">>>>>>>>>>>>>> Sending package %s, length = %d, bytes sent = %d" % (pkg, len(pkg), bytes_sent))

        # Wait for the response from the server.
        start_time = time.ticks_ms()

        while(not check_ack_time(start_time)):
            recv_ack = lora_sock.recv(256)
            # If a message of the size of the acknoledge message is received
            if (len(recv_ack) == 4):
                device_id, pkg_len, recv_msg_id, status = struct.unpack(_LORA_PKG_ACK_FORMAT, recv_ack)
                if (device_id == _DEVICE_ID and recv_msg_id == msg_id):
                    if (status == 200):
                        # Do some code if your message arrived at the central
                        return True
                    else:
                        return False
        time.sleep_ms(urandom(1)[0] << 2)
    return False

# Main Loop
while(True):

# Read the BME280 and get temperature, pressure, and humidity (all strings with units)

    temperatureWithUnit = bme.temperature
    temperature = bme.read_temperature() / 100.0    # temperature in degrees C
    pressureWithUnit = bme.pressure
    pressure = bme.read_pressure() / 25600.0        # barometric pressure in hPa
    humidityWithUnit = bme.humidity
    humidity = bme.read_humidity() / 1024.0         # relative humidity in %
    altitudeWithUnit = bme.altitude
    altitude = bme.read_altitude() / 100.0          # approximate altitude in meters

    # msg = '*' * 63
    # msg = '{"id": %s, "temperature": %s,"unit": "C","time":"14-Apr-2017@21:48:32"}' % (_DEVICE_ID, temperature)
    msg = '{"id": %s, "temperature": %s, "pressure:" %s, "humidity:" %s, "altitude:", %s, time": "14-Apr-2017@21:48:32"}' % (_DEVICE_ID, temperature, pressure, humidity, altitude)
    # msg = "DEVICE %d HERE" % _DEVICE_ID

    print("Sending a message: %s, length = %d" % (msg, len(msg)))

    # success = send_msg("DEVICE %d HERE" % _DEVICE_ID)
    success = send_msg(msg)
    if (success):
        print("ACK RECEIVED: %d" % msg_id)
        increase_msg_id()
    else:
        print("MESSAGE FAILED")
        # Manage the error message

    time.sleep(10)
#
# >>>>>>>>>>>>>> Sending package b'\x01I\x00{"id": 1, "temperature": -14.3,"unit": "C","time":"14-Apr-2017@21:48:32"}', length = 76
# >>>>>>>>>>>>>> Sending package b'\x01\r\x10DEVICE 1 HERE', length = 16
#
