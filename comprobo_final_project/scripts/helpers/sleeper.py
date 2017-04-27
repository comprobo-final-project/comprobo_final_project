import time

def sleep(duration):
    if duration < 0.1:
        inverse = 1 / duration
        count = 0
        while count < inverse:
            count += 1
    else:
        time.sleep(duration)
