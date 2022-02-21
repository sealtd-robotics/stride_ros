import time

def find_rate_limited_speed(speed_rate, initial_time, speed_goal, initial_speed):
    dt = time.time() - initial_time
    sign = (1 if (speed_goal - initial_speed) >= 0 else -1)
    dy = min(abs(speed_goal-initial_speed), abs(speed_rate)*dt)
    return initial_speed + sign*dy