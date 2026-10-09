import numpy as np
#import pyzed as zed
import pyzed.sl as sl
GRAVITY = -9.81
predictorTime = 0.05 

def main(): 
    while True: 
#something lmao


    zed.retrieve_measure(
        point_cloud,
        sl.MEASURE.XYZ
    )

    return None 
def calculateFunction(oldPoint, newPoint): 
#t0, x0, y0, z0 --> GRAB FROM ZED
#t1, x1, y1, z1 

    oldPoint = None   #for now, will need to grab a couple more points

    #some zed shit


    #grab time, x1, x2, x3 (and y z)
    #find the angle with the tangent line?

    oldPoint = t0, x0, y0, z0
    newPoint = t1, x1, y1, z1

    deltaT = t1 - t0

    vx = (x1 - x0) / deltaT
    vy = (y1 - y0) / deltaT
    vz = (z1 - z0) / deltaT

    return vx, vy, vz




def predictorFunction(x0, x1, y0, y1, z0, z1, vx, vy, vz): 
# vcurrent = x y z 
#velocity = vx vy vz
#future time = delta t  
    t = predictorTime
    future_x = x0 + vx * t
    future_y = y0 + vy * t - 0.5 * GRAVITY * t**2
    future_z = z0 + vz * t

    return future_x, future_y, future_z