#file to get coords and turn them into angles. 
import math #uhh

#IF CAMERA IS OFFSET FROM TURRET, NEW MATH NEEDS TO BE DONE!!!!



def get_coords(x, y, z) :

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

    return yawDegrees, pitchDegrees

