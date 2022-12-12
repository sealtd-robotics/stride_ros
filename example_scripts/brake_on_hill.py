# ===================================================================================================================
# This sample shows how to write a script for Stride to brake and hold on a hill during a path following test.

# The recorded path for these scenarios is a straight path up a hill with 30% grade(~16.7 degrees). 
# It is 100 indices long. The function calls for this case all use 2 m/s for the speed and 0.1g for the acceleration. 
# For this case, the robot will move along the path and stop at index 50. It will then engage the mechanical brakes, 
# wait for a postion trigger from an external vehicle, disengage the mechanical brakes, and finally move until the 
# end of the path.

# To comment/uncomment code, add or remove the "#" symbol from each line. You can do this with multiple 
# lines at once by pressing 'ctrl' + '/'.

# More information about the different functions available and when to use them can be found in the user 
# manual under 'Path Scripts -> 'Robot Commander Commands'
# ===================================================================================================================

### Have this line at the beginning of the script for every scenario. Each function call will begin with 'rc.'
rc = RobotCommander()

### Vehicle Variables: Taken from GUI's 'Robot Status' page
target_lat = 40.1099814 # Vehicle latitude in degrees. Used in wait_for_vehicle_position().
target_long = -82.9919913  # Vehicle longitude in degrees. Used in wait_for_vehicle_position().
target_heading = 271.10 # Vehicle heading in degrees. Used in wait_for_vehicle_position().
target_velocity = 3 # Vehicle speed in m/s. Used in wait_for_vehicle_velocity().

### Function Calls
rc.move_until_index(2, 0.1 * 9.81, 45)#Move at 2 m/s with 0.1g acceleration to index 45
rc.decel_to_stop_at_index(50) #Decelerate and stop at index 50
rc.engage_brake_hill() #Engage the mechanical brakes
rc.wait_for_vehicle_position(target_lat, target_long, target_heading) #Wait for vehicle to trigger desired position/heading.
rc.disengage_brake_hill() #Disenagge the mechanical brakes
rc.move_until_end_of_path(2, 0.1 * 9.81) #Move until end of path at 2m/s and 0.1g acceleration
## End test