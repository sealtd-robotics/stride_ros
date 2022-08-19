rc = RobotCommander()

target_lat = 40.1101486 # degress
target_long = -82.9919143  # degrees
target_heading = 228 # degrees
target_velocity = 3 # m/s
sleep_time = 5 # seconds

# # Trigger using target position and heading
rc.wait_for_target_position(target_lat, target_long, target_heading)

# # Trigger using target vehicle
# rc.wait_for_target_velocity(target_velocity)

# # Trigger time-base
# rc.sleep(sleep_time)

rc.move_until_index(3, 0.1*9.81, 50)
rc.brake_to_stop(0.2*9.81)


