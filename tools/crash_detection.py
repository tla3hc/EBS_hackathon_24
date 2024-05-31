import time

DELTA_TIME = 0.01  # Time step in seconds
CRASH_THRESHOLD = 50.0  # Threshold for acceleration difference to consider as a crash

def check_if_crashed(vehicle):
    previous_acceleration = vehicle.get_vehicle_acceleration().linear

    while True:
        time.sleep(DELTA_TIME)
        current_acceleration = vehicle.get_vehicle_acceleration().linear
        if current_acceleration.x - previous_acceleration.x > CRASH_THRESHOLD:
            print("Crash detected!")
            # break
        elif current_acceleration.y - previous_acceleration.y > CRASH_THRESHOLD:
            print("Crash detected!")
            # break
        elif current_acceleration.z - previous_acceleration.z > CRASH_THRESHOLD:
            print("Crash detected!")
            # break
        # print(current_acceleration.x - previous_acceleration.x )
        previous_acceleration = current_acceleration