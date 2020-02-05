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
#                                                                       Initial
# Address   R/W Name Description                                        Value
# 0x00      W   ACQ_COMMANDS Device command                             - -
# 0x01      R   STATUS System status                                    - -
# 0x05      R/W ACQUISITION_COUNT Maximum acquisition count             0xFF
# 0x10      R   FULL_DELAY_LOW Distance measurement low byte            - -
# 0x11      R   FULL_DELAY_HIGH Distance measurement high byte          - -
# 0x16      R   UNIT_ID_0 Unit ID, byte 0                               - -
# 0x16      W   UNIT_ID_0_UNLOCK Write unit ID 0 for I2C address unlock - -
# 0x17      R   UNIT_ID_1 Unit ID, byte 1                               - -
# 0x17      W   UNIT_ID_1_UNLOCK Write unit ID 1 for I2C address unlock - -
# 0x18      R   UNIT_ID_2 Unit ID, byte 2                               - -
# 0x18      W   UNIT_ID_2_UNLOCK Write unit ID 2 for I2C address unlock - -
# 0x19      R   UNIT_ID_3 Unit ID, byte 3                               - -
# 0x19      W   UNIT_ID_3_UNLOCK Write unit ID 3 for I2C address unlock - -
# 0x1A      R/W I2C_SEC_ADDR Write new I2C address after unlock         - -
# 0x1B      W   I2C_CONFIG Default address response control             0x00
# 0x1C      R/W DETECTION_SENSITIVITY Peak detection threshold bypass   0x00
# 0x30      R   LIB_VERSION Read Garmin software library version string - -
# 0x52      R/W CORR_DATA Correlation record data control               - -
# 0x72      R   CP_VER_LO Coprocessor firmware version low byte         - -
# 0x73      R   CP_VER_HI Coprocessor firmware version high byte        - -
# 0xE0      R   BOARD_TEMPERATURE Board temperature                     - -
# 0xE1      R   HARDWARE_VERSION Board hardware version                 - -
# 0xE2      R/W POWER_MODE Power state control                          0xFF
# 0xE3      R/W MEASUREMENT_INTERVAL Automatic measurement rate         0xFF
# 0xE4      W   FACTORY_RESET Reset default settings                    - -
# 0xE5      R/W QUICK_TERMINATION Quick acquisition termination         0x08
# 0xE6      W   START_BOOTLOADER Start secure Bluetooth LE bootloader   - -
# 0xEA      R/W ENABLE_FLASH_STORAGE Store register settings            0x00
# 0xEB      R/W HIGH_ACCURACY_MODE Improved accuracy setting            0x14
# 0xEC      R   SOC_TEMPERATURE SoC temperature                         - -
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

class LIDARLiteV4:
    i2c = None

    _acqCommand = 0x00
    _status = 0x01
    _acqCount = 0x05
    _fullDelayLow = 0x10
    _fullDelayHigh = 0x11
    _unitID0 = 0x16
    _unitID0Unlock = 0x16
    _unitID1 = 0x17
    _unitID1Unlock = 0x17
    _unitID2 = 0x18
    _unitID2Unlock = 0x18
    _unitID3 = 0x19
    _unitID3Unlock = 0x19
    _i2cSetAddress = 0x1A
    _i2cConfig = 0x1B
    _detectSensitivity = 0x1C
    _libVersion = 0x30
    _corrData = 0x52
    _CPVerLo = 0x72
    _CPVerHi = 0x73
    _BoardTemp = 0xE0
    _HardwareVersion = 0xE1
    _powerMode = 0xE2
    _measureInterval = 0xE3
    _factoryReset = 0xE4
    _quickTermination = 0xE5
    _startBootloader = 0xE6
    _enableFlashStore = 0xEA
    _highAccMode = 0xEB
    _socTemp = 0xEC

    def __init__(self):
        self.i2c = I2C()
