# boot.py -- run on boot-up
import machine
from machine import Pin
from network import WLAN, Bluetooth
from config import *

# Turn off Bluetooth
bt = Bluetooth()
bt.deinit()

# Turn off WLAN
wlan = WLAN()
wlan.deinit()

# Disable debug logging by default, may be enabled later on in config.py
DEBUG = False
