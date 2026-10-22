import numpy

GRAVITY = 9.81

TENNIS_BALL_SPEED = 11.176       # 25 mph in m/s
PROCESSING_LATENCY = 0.039       # current measured pipeline delay


LAUNCHER_POSITION = numpy.array([
    0.0,
    0.254,
    0.0
], dtype=numpy.float64)


class BallKalman:

    def __init__(self):
        self.state = numpy.zeros(6, dtype=numpy.float64)


        self.measurementCount = 0
        self.finalIntercept = None
        self.minMeasurementsForIntercept = 4


        self.initialP = numpy.diag([
            0.05**2,
            0.05**2,
            0.10**2,
            5.0**2,
            5.0**2,
            5.0**2
        ])

        self.P = self.initialP.copy()

        self.H = numpy.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], dtype=numpy.float64)

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
        x, y, z = measurement

        self.measurementCount = 1
        self.finalIntercept = None

        self.state[:] = [
            x,
            y,
            z,
            0.0,
            0.0,
            0.0
        ]

        self.P = self.initialP.copy()

        self.last_time = t
        self.initialized = True


    def predict(self, dt):
        F = numpy.eye(6)

        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        gravityEffect = numpy.array([
            0.0,
            -0.5 * GRAVITY * dt**2,
            0.0,
            0.0,
            -GRAVITY * dt,
            0.0
        ], dtype=numpy.float64)

        self.state = F @ self.state + gravityEffect

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

        self.P = F @ self.P @ F.T + Q


    def correct(self, measurement):
        z = numpy.asarray(
            measurement,
            dtype=numpy.float64
        )

        fix = z - self.H @ self.state

        S = (
            self.H
            @ self.P
            @ self.H.T
            + self.R
        )

        mahalanobisSquared = (
            fix.T
            @ numpy.linalg.inv(S)
            @ fix
        )

        if mahalanobisSquared > 11.34:
            return False

        K = (
            self.P
            @ self.H.T
            @ numpy.linalg.inv(S)
        )

        self.state = (
            self.state
            + K @ fix
        )

        I = numpy.eye(6)

        self.P = (
            (I - K @ self.H)
            @ self.P
        )

        return True


    def update(self, t, measurement):
        if not self.initialized:
            self.initialize(
                t,
                measurement
            )
            return None

        dt = t - self.last_time

        if dt <= 0:
            return None

        print(f"dt={dt:.3f}s")

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

        accepted = self.correct(measurement)

        if accepted:
            self.measurementCount += 1

        self.last_time = t

        return self.state.copy()


    def predictTargetPosition(self, seconds):
        x, y, z, vx, vy, vz = self.state

        targetX = x + vx * seconds

        targetY = (
            y
            + vy * seconds
            - 0.5 * GRAVITY * seconds**2
        )

        targetZ = z + vz * seconds

        return numpy.array([
            targetX,
            targetY,
            targetZ
        ], dtype=numpy.float64)


    def findIntercept(
        self,
        launcherPosition=LAUNCHER_POSITION,
        tennisBallSpeed=TENNIS_BALL_SPEED,
        processingLatency=PROCESSING_LATENCY,
        maxTime=2.0,
        searchStep=0.002
    ):
        if not self.initialized:
            return None

        gravityVector = numpy.array([
            0.0,
            -GRAVITY,
            0.0
        ], dtype=numpy.float64)

        def interceptError(interceptTime):
            flightTime = (
                interceptTime
                - processingLatency
            )

            if flightTime <= 0:
                return None, None, None

            targetPoint = self.predictTargetPosition(
                interceptTime
            )

            requiredVelocity = (
                targetPoint
                - launcherPosition
                - 0.5
                * gravityVector
                * flightTime**2
            ) / flightTime

            requiredSpeed = numpy.linalg.norm(
                requiredVelocity
            )

            error = (
                requiredSpeed
                - tennisBallSpeed
            )

            return (
                error,
                targetPoint,
                requiredVelocity
            )

        startTime = processingLatency + 0.005

        times = numpy.arange(
            startTime,
            maxTime + searchStep,
            searchStep
        )

        previousTime = None
        previousError = None

        for currentTime in times:
            (
                currentError,
                _,
                _
            ) = interceptError(
                currentTime
            )

            if currentError is None:
                continue

            if previousError is not None:
                if previousError * currentError <= 0:
                    low = previousTime
                    high = currentTime

                    for _ in range(30):
                        mid = (
                            low + high
                        ) / 2.0

                        (
                            midError,
                            _,
                            _
                        ) = interceptError(
                            mid
                        )

                        (
                            lowError,
                            _,
                            _
                        ) = interceptError(
                            low
                        )

                        if lowError * midError <= 0:
                            high = mid
                        else:
                            low = mid

                    interceptTime = (
                        low + high
                    ) / 2.0

                    (
                        _,
                        targetPoint,
                        launchVelocity
                    ) = interceptError(
                        interceptTime
                    )

                    flightTime = (
                        interceptTime
                        - processingLatency
                    )

                    launchSpeed = numpy.linalg.norm(
                        launchVelocity
                    )

                    direction = (
                        launchVelocity
                        / launchSpeed
                    )

                    vx = launchVelocity[0]
                    vy = launchVelocity[1]
                    vz = launchVelocity[2]

                    yawDegrees = numpy.degrees(
                        numpy.arctan2(
                            vx,
                            -vz
                        )
                    )

                    horizontalSpeed = numpy.sqrt(
                        vx**2
                        + vz**2
                    )

                    pitchDegrees = numpy.degrees(
                        numpy.arctan2(
                            vy,
                            horizontalSpeed
                        )
                    )

                    return {
                        "point": targetPoint,
                        "interceptTime": interceptTime,
                        "flightTime": flightTime,
                        "launchVelocity": launchVelocity,
                        "direction": direction,
                        "yawDegrees": yawDegrees,
                        "pitchDegrees": pitchDegrees
                    }

            previousTime = currentTime
            previousError = currentError

        return None
    def getFinalIntercept(self):

    # Once locked, always return the same prediction
    # for this trajectory.
        if self.finalIntercept is not None:
            return self.finalIntercept

    # Give Kalman a few measurements to learn velocity.
        if self.measurementCount < self.minMeasurementsForIntercept:
            return None

        intercept = self.findIntercept()

        if intercept is None:
            return None

        self.finalIntercept = intercept

        return self.finalIntercept
