import math


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

def fire(flywheelDriver):
    flywheelDriver.fireSolenoid().result()

    