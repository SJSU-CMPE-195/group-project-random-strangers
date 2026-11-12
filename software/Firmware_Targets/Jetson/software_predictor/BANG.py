import math
from queue import Queue

from I2C_Driver.I2C_Handler import (
    I2CTransaction,
    Jetson_I2C_Handler,
    I2CResult
)

from I2C_Driver.I2C_Devices.Jetson_Flywheel_Controller_Driver import (
    JetsonFlywheelControllerDriver
)

from I2C_Driver.Jetson_Gimbal_Controller_Driver import (
    get_coords
)

FLYWHEEL_I2C_ADDRESS = 0x62

transactionQueue = Queue()

i2cHandler = Jetson_I2C_Handler(
    1,
    transactionQueue
)

i2cHandler.register_device(
    "flywheel",
    FLYWHEEL_I2C_ADDRESS,
    False
)

i2cHandler.startDriverThread()

flywheelDriver = JetsonFlywheelControllerDriver(
    i2cHandler,
    FLYWHEEL_I2C_ADDRESS
)

angleTolerance = math.radians(1.0)


def gimbalTARGET ( 
    targetYaw, 
    targetPitch, 
    currentYaw, 
    currentPitch
): 

    return (
        abs(targetYaw - currentYaw) <= angleTolerance
        and 
        abs(targetPitch - currentPitch) <= angleTolerance
    )

fired = False

def fire(flywheelDriver):
    global fired

    if fired:
        return False

    flywheelDriver.fireSolenoid().result()

    fired = True
    return True

