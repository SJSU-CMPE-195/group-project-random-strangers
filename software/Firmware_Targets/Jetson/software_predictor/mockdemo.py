import numpy
#BUNCH OF AI GENERATED TESTS FOR YOU, TA!

# - EH
#Also, cannot really run these without the actual hardware, so i will have to create the video on tuesday 10/27
from projectileMotion import BallKalman, GRAVITY


def main():
    kalman = BallKalman()

    # Simulated incoming ball trajectory.
    #
    # Initial position:
    #   x = 0.0 m
    #   y = 1.5 m
    #   z = -4.0 m
    #
    # Initial velocity:
    #   vx = 0.0 m/s
    #   vy = 2.0 m/s
    #   vz = 4.0 m/s
    #
    # Positive Y is upward, so gravity reduces Y over time.

    initial_position = numpy.array(
        [0.0, 1.5, -4.0],
        dtype=numpy.float64
    )

    initial_velocity = numpy.array(
        [0.0, 2.0, 4.0],
        dtype=numpy.float64
    )

    dt = 0.05

    print("=== Ball Tracking Mock Demo ===")
    print("No ZED camera or physical hardware required.")
    print()

    for i in range(8):
        t = i * dt

        measurement = numpy.array([
            initial_position[0]
            + initial_velocity[0] * t,

            initial_position[1]
            + initial_velocity[1] * t
            - 0.5 * GRAVITY * t**2,

            initial_position[2]
            + initial_velocity[2] * t
        ])

        state = kalman.update(
            t,
            measurement
        )

        print(
            f"t={t:.2f}s "
            f"measurement="
            f"({measurement[0]:.3f}, "
            f"{measurement[1]:.3f}, "
            f"{measurement[2]:.3f})"
        )

        if state is not None:
            x, y, z, vx, vy, vz = state

            print(
                "  filtered position: "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )

            print(
                "  estimated velocity: "
                f"({vx:.3f}, {vy:.3f}, {vz:.3f}) m/s"
            )

    print()
    print("=== Intercept Calculation ===")

    intercept = kalman.getFinalIntercept()

    if intercept is None:
        print("No reachable intercept found.")
        return

    point = intercept["point"]
    velocity = intercept["launchVelocity"]

    print(
        "Intercept point: "
        f"({point[0]:.3f}, "
        f"{point[1]:.3f}, "
        f"{point[2]:.3f}) m"
    )

    print(
        f"Intercept time: "
        f"{intercept['interceptTime']:.3f} s"
    )

    print(
        f"Projectile flight time: "
        f"{intercept['flightTime']:.3f} s"
    )

    print(
        "Required launch velocity: "
        f"({velocity[0]:.3f}, "
        f"{velocity[1]:.3f}, "
        f"{velocity[2]:.3f}) m/s"
    )

    print(
        f"Launcher yaw: "
        f"{intercept['yawDegrees']:.2f} degrees"
    )

    print(
        f"Launcher pitch: "
        f"{intercept['pitchDegrees']:.2f} degrees"
    )

    print()
    print("PASS: Kalman tracking and intercept prediction completed.")


if __name__ == "__main__":
    main()