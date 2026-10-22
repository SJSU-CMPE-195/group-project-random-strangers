import numpy

GRAVITY = 9.81
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


class BallKalman: 
    def __init__(self):
        #state [x,y z, vx, vy, vz]
        self.state = numpy.zeros (6, dtype=numpy.float64)


        self.P = numpy.diag([  #uncertainty estimates. Position is certain, velocity is not since we don't know it immediately.
                                #correspondnds to [x,y,z,vx,vy,vz]
            0.05**2,
            0.05**2,
            0.10**2,
            5.0**2,
            5.0**2,
            5.0**2
        ]) 
        self.P = self.P.copy()

        self.H = numpy.array([ #x y z measurements
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], dtype = numpy.float64)

        self.R = numpy.diag([
            0.03**2, 
            0.03**2, 
            0.08**2
        ])

        self.accelerationstd = 2.0
        self.last_time = None
        self.initialized = False

        self.maxTrackingGap = 0.5

    def initialize(self, t, measurement):
        x , y , z = measurement
        self.state[:] = [
            x, y , z, 0.0, 0.0 ,0.0
        ]

        self.last_time = t
        self.initialized = True 

    def predict (self, dt): #will make a prediction on where the ball will be next b4 next predcition

        F = numpy.eye(6) #returns a 2D array (matrix) with ones (1) on a specified diagonal and zeros (0) elsewhere.

        F [0,3] = dt
        F[1,4] = dt
        F[2,5] = dt

        gravityEffect = numpy.array([
            0.0,
            -0.5 * GRAVITY * dt**2,
            0.0,
            0.0,
            -GRAVITY * dt,
            0.0
        ])

        self.state = F @ self.state + gravityEffect


        #bunch of stuff that i pulled from the internet that processes noise. main point is that Q is how much deviation from the 
        #model that is allowed. Air resisstance, spin, bad measurements, notably
        q = self.accelerationstd**2

        Q = numpy.zeros((6, 6))

        for positionIndex in range(3):

            velocityIndex = positionIndex + 3

            Q[positionIndex, positionIndex] = (
                0.25 * dt**4 * q
            )

            Q[positionIndex, velocityIndex] = (
                0.5 * dt**3 * q
            )

            Q[velocityIndex, positionIndex] = (
                0.5 * dt**3 * q
            )

            Q[velocityIndex, velocityIndex] = (
                dt**2 * q
            )


            self.P = F @ self.P @ F.T + Q #rmr, @ is matrix mult operator, aka matrix H by vector state.

    def correct (self, measurement): #function to corrct noted deviations in the last one
        z = numpy.asarray(
            measurement, 
            dtype = numpy.float64
        )

        fix = z - self.H @ self.state




        S = ( self.H @ self.P @ self.H.T + self.R)

        mahalanobisSquared = (
                fix.T
                @ numpy.linalg.inv(S)
                @ fix
        )
        
        if mahalanobisSquared > 11.34:
            return False

            #kalman gain, how much trust between camera and predicton

        K = (self.P @ self.H.T @ numpy.linalg.inv(S))


        self.state = (
            self.state + K @ fix
        )

        I = numpy.eye(6) 

        self.P = ((I - K @ self.H) @ self.P)

    def update (self, t, measurement): #push the new camera measurements into the filter) 

        if not self.initialized: 
            self.initialize( t, measurement)
            return None 

        dt = t - self.last_time

        if dt <= 0:
            return None

        print(f"dt={dt:.3f}s")

        # If the ball has been gone too long, this is no longer
        # assumed to be the same continuous projectile trajectory.
        if dt > self.maxTrackingGap:
            print(
                f"New trajectory detected "
                f"(tracking gap = {dt:.2f}s)"
            )

            self.initialize(
                t,
                measurement
            )

            return None

        self.predict(dt)
        self.correct(measurement)

        self.last_time = t

        return self.state.copy() 

    def predicttheFuture(self, seconds = predictorTime):

            #ballistic trajectory some time int he future with the filter'd position and velo

        x , y , z, vx , vy , vz = self.state

        futureX = ( x + vx * seconds)
        futureY = (y + vy * seconds -( 0.5 * GRAVITY * seconds **2))
        futureZ = (z + vz * seconds)

        return futureX, futureY, futureZ