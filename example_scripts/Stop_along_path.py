# ===========================================================================================================
# This sample shows how to write a script for Stride to follow a straight or curved path, stop for a desired 
# length of time, and then continue. The same logic can be used for straight and curved paths.

# The recorded path for these scenarios is a straight path that is 100 indices long. The function calls all 
# use 2 m/s for the speed and 0.1g for the acceleration. For this case, the robot will stop at index 55, wait 
# for 2 seconds, and then continue until the path's end.

# To comment/uncomment code, add or remove the "#" symbol from each line. You can do this with multiple 
# lines at once by pressing 'ctrl' + '/'.

# More information about the different functions available and when to use them can be found in the user 
# manual under 'Path Scripts -> 'Robot Commander Commands'
# ===========================================================================================================

### Have this line at the beginning of the script for every scenario. Each function call will begin with 'rc.'
rc = RobotCommander()

### Function Calls
rc.move_until_index(2, 0.1 * 9.81, 50) #Move at 2 m/s with 0.1g acceleration until index 50.
## If a specific stopping point is desired, use decel_to_stop_at_index(). Otherwise, brake_to_stop() will work here too.
rc.decel_to_stop_at_index(55) #Decelerate and stop at index 55
rc.sleep(2) #Stop for 2 seconds
## The below function can be replaced with other path following function calls for the desired operation. Ex) move_until_index() followed by decel_to_stop_at_index()
rc.move_until_end_of_path(2, 0.1 * 9.81) # Move until end of path at 2 m/s with 0.1g acceleration.
## End of Test