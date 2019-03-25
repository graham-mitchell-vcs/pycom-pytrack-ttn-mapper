###############################################################################
# LoRaWAN Configuration
###############################################################################

# May be either 'otaa', 'abp', or 'off'
LORA_MODE         = 'otaa'

LORA_OTAA_EUI     = '#################'
LORA_OTAA_KEY     = '############################'      # See README.md for instrufrctions!

# Interval between measures transmitted to TTN.
# Measured airtime of transmission is 56.6 ms, fair use policy limits us to
# 30 seconds per day (= roughly 500 messages). We default to a 180 second
# interval (=480 messages / day).
LORA_SEND_RATE    = 10

MIN_MOVE_DISTANCE = 3
