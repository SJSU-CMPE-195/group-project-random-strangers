
GRAVITY = -9.81
predictorTime = 0.05 
def calculateFunction(oldPoint, newPoint): 
#t0, x0, y0, z0 --> GRAB FROM ZED
#t1, x1, y1, z1 

   # oldPoint = None   #for now, will need to grab a couple more points

    #some zed shit


    #grab time, x1, x2, x3 (and y z)
    #find the angle with the tangent line?

    t0, x0, y0, z0 = oldPoint 
    t1, x1, y1, z1 = newPoint

    deltaT = t1 - t0

    vx = (x1 - x0) / deltaT
    vy = (y1 - y0) / deltaT
    vz = (z1 - z0) / deltaT

    return vx, vy, vz




def predictorFunction(currentPoint, vx, vy, vz): 
# vcurrent = x y z 
#velocity = vx vy vz
#future time = delta t  
    x, y, z = currentPoint 
    t = predictorTime
    future_x = x + vx * t
    future_y = y + vy * t - 0.5 * GRAVITY * t**2
    future_z = z + vz * t

    return future_x, future_y, future_z