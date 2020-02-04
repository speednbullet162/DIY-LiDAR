##############################################################|| Notes ||#################################################################
# Obtaining Measurements from the I2C Interface
# 1 Write 0x04 to register 0x00.
# 2 Read register 0x01.
# 3 Repeat step 2 until bit 0 (LSB) goes low.
# 4 Read two bytes from 0x10 (low byte 0x10 then high byte 0x11) to obtain the 16-bit measured distance in centimeters.
#
# triggering and Reading Distance Measurements
# 1 Toggle the TRIGGER pin.
# 2 Wait for the MONITOR pin to go low.
# 3 Read two bytes from 0x10 (low byte 0x10, then high byte 0x11) to obtain the 16-bit measured distance in centimeters.
#
# Pin Name    Function            V Max     other      for pi
# VIN         5 V Power           5 V
# GND         Ground              - -
# I2C SDA     I2C Data            3.3 V                input
# I2C SCL     I2C Clock           3.3 V                input
# GPIOA       General Purpose I/O 3.3 V     trigger    output
# GPIOB       General Purpose I/O 3.3 V     monitor    input
# VRETURN     nRF52840 DBG        3.3 V
# nRESET      nRF52840 DBG        3.3 V
# SWCLK       nRF52840 DBG        3.3 V
# SWDIO       nRF52840 DBG        3.3 V
#
# GPIO A TRIGGER measurement trigger input
# Toggle to start a distance measurement. The LIDARLite v4 LED starts a distance measurement on either the rising or falling edge. If a
# distance measurement is triggered while the device is busy, the requested measurement is ignored.
#
# GPIO B MONITOR BUSY status output
# Indicates when the LIDARLite v4 LED is busy. If low, the device is idle and is ready to start a distancemeasurement. If high, the
# device is busy taking a distance measurement. Wait for the signal to drop before you toggle GPIO A to trigger a distance measurement.
###########################################################################################################################################

# try:
#     import RPi.GPIO as GPIO
# except RuntimeError:
#     print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")


# GPIO.setmode(GPIO.BOARD)
# GPIO.setwarnings(False)
# outList = []
# inputList = []
# GPIO.setup(outList, GPIO.OUT)
# GPIO.setup(inputList, GPIO.IN)
