import time

CALIBRATION_FACTOR = 11658482

def sleep(duration):
    if duration < 0.1:
        count = 0
        while count < duration * CALIBRATION_FACTOR:
            count += 1
    else:
        time.sleep(duration)


if __name__ == "__main__":

    # Calibrate
    time_to_wait = 4 # secs
    resolution = 1000

    start_time = time.time()
    for i in range(time_to_wait * resolution):
        sleep(1.0 / resolution)
    end_time = time.time()

    diff = end_time - start_time

    # Time it actually took
    print "Actual time: " + str(diff)

    # Ratio
    print "Ratio: " + str(time_to_wait / diff)

    # New calibration number to use
    print "New calibration number: " + str((time_to_wait / diff) * CALIBRATION_FACTOR)
