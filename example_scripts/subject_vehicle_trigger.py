# ===========================================================================================================
# This sample shows how to write a script for Stride to follow a path based on a subject vehicle's movement.
# There are two functions that can be used: one based on position and one based on velocity. 

# The recorded path for these scenarios is 100 indices long, with he robot traveling at 2m/s with an 
# acceleration of 0.1g. For this case, the robot will move to index 50, stop and wait for the vehicle trigger, 
# continue at the same speed and acceleration, and then stop at index 100.

# # To comment/uncomment code, add or remove the "#" symbol from each line. You can do this with multiple 
# lines at once by pressing 'ctrl' + '/'.

# More information about the different functions available and when to use them can be found in the user 
# manual under 'Path Scripts -> 'Robot Commander Commands' 
# ===========================================================================================================

### Have this line at the beginning of the script for every scenario. Each function call will begin with 'rc.'
rc = RobotCommander()

### Vehicle Variables: Taken from GUI's 'Robot Status' page
target_lat = 40.1101486 # Vehicle latitude in degrees. Used in wait_for_vehicle_position().
target_long = -82.9919143  # Vehicle longitude in degrees. Used in wait_for_vehicle_position().
target_heading = 228 # Vehicle heading in degrees. Used in wait_for_vehicle_position().
target_velocity = 3 # Vehicle speed in m/s. Used in wait_for_vehicle_velocity().

### Trigger using subject vehicle position and heading
# rc.move_until_index(2, 0.1, 45) #Move at 2 m/s with 0.1g acceleration to index 45
# rc.decel_to_stop_at_index(50) #Decelerate and stop at index 50
# rc.wait_for_vehicle_position(target_lat, target_long, target_heading) #Wait for vehicle to trigger desired position/heading
# rc.move_until_index(2, 0.1, 95) #Move at 2 m/s with 0.1g acceleration to index 95
# rc.decel_to_stop_at_index(100) #Decelerate and stop at index 100
## End test
# rc.custom_compensation(1,1,1,1,1,1,1)
rc.move_until_end_of_path(2, 0.1)

### Trigger using subject vehicle velocity
# rc.move_until_index(2, 0.1, 45) #Move at 2 m/s with 0.1g acceleration to index 45
# rc.decel_to_stop_at_index(50) #Decelerate and stop at index 50
# rc.wait_for_vehicle_velocity(target_velocity) #Wait for vehicle to reach desired velocity
# rc.move_until_index(2, 0.1, 95) #Move at 2 m/s with 0.1g acceleration to index 95
# rc.decel_to_stop_at_index(100) #Decelerate and stop at index 100
## End test
