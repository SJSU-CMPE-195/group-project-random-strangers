#file to get coords and turn them into angles, and send them to I2C controller/Gimbal
import math #uhh
import struct
from queue import Queue
from I2C_Handler import I2CTransaction, Jetson_I2C_Handler
#IF CAMERA IS OFFSET FROM TURRET, NEW MATH NEEDS TO BE DONE!!!!!!!


GIMBAL_I2C_ADDRESS = 0x54 #Assume Gimbal Address, feel free to alter this
TARGET_ANGLE_REGISTER = 0x01 #this aswell

transactionQueue = Queue()

i2cHandler = Jetson_I2C_Handler( #handler instance
    1,
    transactionQueue
)

i2cHandler.register_device( #register ESP32 as an object for uselater
    "gimbal",
    GIMBAL_I2C_ADDRESS,
    False
)

i2cHandler.startDriverThread() #starts  handler thread



def get_coords(x, y, z) : #probably don't touch this 

    #recieves inctercpt coords from projectileMotion

    print("recieved: ")
    print(f"x={x:.3f}, y={y:.3f}, z={z:.3f}")

    yawRadians = math.atan2(
        x,
        -z
    )

    horizontalDistance = math.sqrt( 
        x**2 + z**2
    )

    pitchRadians = math.atan2( #find radians w/ y coord
        y,
        horizontalDistance
    )

    yawDegrees = math.degrees( #convert to degrees 
        yawRadians
    )

    pitchDegrees = math.degrees(
        pitchRadians
    )


    yaw, pitch = encode_angles(
    yawDegrees,
    pitchDegrees
)

    send_angles(
    yaw,
    pitch
)
    return yawDegrees, pitchDegrees


def encode_angles(yawDegrees, pitchDegrees):  #multiples values by 100, encodes for byte placement
    yaw = int(yawDegrees * 100)
    pitch = int (pitchDegrees * 100)

    return yaw, pitch



#hopefully sends them correctly in an orderly queue to the cpp file. ask AI for more, didn't write this function entirely natural
def send_angles(yaw, pitch): 
    payload = struct.pack(
        ">hh",
        yaw,
        pitch
    )

    transaction = I2CTransaction(
        deviceAddress=GIMBAL_I2C_ADDRESS, #look above
        registerAddress=TARGET_ANGLE_REGISTER,
        writeData=payload
    )

    i2cHandler.submitTransaction(transaction) #check out I2C_Handler function here ->

###
# I2C PROTOCOL ASSUMPTIONS:

# Payload is four bytes:
# [yaw high][yaw low][pitch high][pitch low]
#
# yaw/pitch are signed int16 values


# TODO: ESP32 receiver must decode this same format.
### 