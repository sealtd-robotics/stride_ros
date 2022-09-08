# ===========================================================================================================
# This sample shows how to write a script for Stride to be able to follow a curved path. The same methods for 
# following straight paths can be followed for following curved paths, but with a few recommendations.

# It is recommended to use lower speeds (2 or 3 m/s) when following a curve path with a tighter turn. To get a 
# more accurate path to follow, drive Stride slowly while path recording. 

# The recorded path for these scenarios is a half circle with 100 indices. The function calls for 
# this case all use 2 m/s for the speed and 0.1g for the acceleration.

# To comment/uncomment code, add or remove the "#" symbol from each line. You can do this with multiple 
# lines at once by pressing 'ctrl' + '/'.

# More information about the different functions available and when to use them can be found in the user 
# manual under 'Path Scripts -> 'Robot Commander Commands'
# ===========================================================================================================

### Have this line at the beginning of the script for every scenario. Each function call will begin with 'rc.'
rc = RobotCommander()

### Method 1 using indices
rc.move_until_index(2, 0.1 * 9.81, 93) #Move until index 93 at desired speed and acceleration.
rc.decel_to_stop_at_index(98) #Recommended to put stop index 1-2 points ahead of path end.

### Method 2: Use brake_to_stop() instead of decel_to_stop()
# rc.move_until_index(2, 0.1 * 9.81, 93) 
# rc.brake_to_stop(0.1 * 9.81)

### Method 3 without using indices
# rc.move_until_end_of_path(2, 0.1 * 9.81)