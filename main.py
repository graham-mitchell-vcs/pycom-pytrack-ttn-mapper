#
# ttnmapper.py
#
# Implementation of a ttnmapper-node for LoPy.
# See http://ttnmapper.org and https://www.pycom.io/product/lopy for details.
#
# Copyright (C) 2017, Peter Affolter and Pascal Mainini
# Licensed under MIT license, see included file LICENSE or
# http://opensource.org/licenses/MIT
#

import array
import pycom
import socket
import time
import utime
import math

from binascii import hexlify, unhexlify
from struct import unpack
from machine import Pin, UART, Timer, idle
from network import LoRa
from config import *

from L76GNSS_mapper import L76GNSS
from pytrack import Pytrack

###############################################################################
# Configuration and Constants
###############################################################################

# Colors used for status LED
RGB_OFF         = 0x000000
RGB_POS_UPDATE  = 0x101000
RGB_POS_FOUND   = 0x001000
RGB_POS_NFOUND  = 0x100000
RGB_LORA        = 0x000010
LED_TIMEOUT     = 0.2

MAX_TX_COUNT    = 300
TX_COUNT        = 0

LAST_TX_LAT     = 0.0
LAST_TX_LON     = 0.0

###############################################################################
# Function Definitions
###############################################################################

def log(msg):
    """Helper method for logging messages"""
    print('ttnmapper: {}'.format(msg))

def join_otaa():
    """Joins the LoRaWAN network using Over The Air Activation (OTAA)"""
    lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.AS923)
    log('Initializing LoRaWAN (OTAA), DEV EUI: {} ...'.format(
        hexlify(lora.mac()).decode('ascii').upper()))

    if not LORA_OTAA_KEY:
        log('ERROR: LoRaWAN APP KEY not set!')
        log('Send your DEV EUI to thethingsnetwork@bfh.ch to obtain one.')
        return None

    pycom.rgbled(RGB_LORA)

    authentication = (unhexlify(LORA_OTAA_EUI),
                      unhexlify(LORA_OTAA_KEY))

    lora.join(activation=LoRa.OTAA, auth=authentication, timeout=0)

    while not lora.has_joined():
        log('Joining...')
        pycom.rgbled(RGB_OFF)
        time.sleep(LED_TIMEOUT)
        pycom.rgbled(RGB_LORA)
        time.sleep(2.5)
        # if no connection in a few seconds, then reboot
        if utime.time() > 15:
            print("Possible timeout")
            machine.reset()

    pycom.rgbled(RGB_OFF)
    return lora

def init_lora():
    """Initialize LoRaWAN connection"""

    lora = join_otaa()

    # Setup socket
    sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    sock.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)      # Set data rate
    sock.setblocking(False)

    log('Done!')
    return (lora, sock)

def gnss_position2():
    #Obtain current GNSS position.
    #If a position has been obtained, returns an instance of NmeaParser
    #containing the data. Otherwise, returns None.

    py = Pytrack()
    l76 = L76GNSS(py, timeout=30)

    return l76.coordinates()

def distance_check(lat1, lon1, lat2, lon2):
    R = 6378.137 # Radius of earth in KM
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    d = d * 1000 # meters

    print("Meters between this and last transmitted coordinates: " + str(d))
    return d

def transmit2(pos):
    """Encode current position, altitude and hdop and send it using LoRaWAN"""
    pycom.rgbled(RGB_LORA)

    global TX_COUNT
    global MAX_TX_COUNT
    global LAST_TX_LAT
    global LAST_TX_LON

    if TX_COUNT >= MAX_TX_COUNT:
        print("Max TX count reached,TX skipped, reset needed")
        return

    if distance_check(LAST_TX_LAT,LAST_TX_LON,pos[0],pos[1]) > MIN_MOVE_DISTANCE:
        LAST_TX_LAT = pos[0]
        LAST_TX_LON = pos[1]
        TX_COUNT = TX_COUNT + 1
    else:
        print("No movement detected, TX skipped, saving air time")
        return

    data = array.array('B', [0, 0, 0, 0, 0, 0, 0, 0, 0])

    lat = int(((pos[0] + 90) / 180) * 16777215)
    data[0] = (lat >> 16) & 0xff
    data[1] = (lat >> 8) & 0xff
    data[2] = lat & 0xff

    lon = int(((pos[1] + 180) / 360) * 16777215)
    data[3] = (lon >> 16) & 0xff
    data[4] = (lon >> 8) & 0xff
    data[5] = lon & 0xff

    alt = int(pos[2])
    data[6] = (alt >> 8) & 0xff
    data[7] = alt & 0xff

    hdop = int(pos[3] * 10)
    data[8] = hdop & 0xff

    message = bytes(data)
    count = sock.send(message)

    log('Message sent: {} ({} bytes)'.format(hexlify(message).upper(), count))

def update_task(alarmtrigger):
    """Periodically run task which tries to get current position and update
       ttnmapper"""

    pycom.rgbled(RGB_POS_UPDATE)
    time.sleep(LED_TIMEOUT)

    pos = gnss_position2()
    print("Position data (lat,lon,alt,hdop): " + str(pos))

    if all(v is not None for v in pos):
        print("Position data, prepare for TX")
        pycom.rgbled(RGB_POS_FOUND)
        time.sleep(LED_TIMEOUT)
        if lora:
            transmit2(pos)
        else:
            log('LoRa disabled, not transmitting!')
    else:
        print("No position data, skipping")
        pycom.rgbled(RGB_POS_NFOUND)

    time.sleep(LED_TIMEOUT)
    pycom.rgbled(RGB_OFF)

    idle()


###############################################################################
# Main Program
###############################################################################

log('Starting up...')

pycom.heartbeat(False)      # Turn off hearbeat LED

(lora, sock) = init_lora()

mapper = Timer.Alarm(update_task, s=LORA_SEND_RATE, periodic=True)

log('Startup completed')
