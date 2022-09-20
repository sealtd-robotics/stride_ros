# ===============================================================================================================
# This sample shows how to write a script for Stride to follow a path that requires rotation. The rotation 
# will be to an input heading.

# The recorded path for these scenarios is a straight path with a turn. It is 100 indices long. The function 
# calls for this case all use 2 m/s for the speed and 0.1g for the acceleration. For this case, the robot will 
# move along the path and stop at index 50. It will then rotate to a heading of 45 degrees (clockwise from North), 
# sleep for 2 seconds, and proceed to the end of the path.

# While path recording: When reaching the point in the path Stride should turn at, use the spin buttons on the on 
# the 'Manual Control' screen of the GUI to get the robot pointed in the desired direction to continue path 
# following. On the same screen, there will be a value showing the heading. Enter this value for the heading 
# parameter of rotate_until_heading(). 

# To comment/uncomment code, add or remove the "#" symbol from each line. You can do this with multiple 
# lines at once by pressing 'ctrl' + '/'.

# More information about the different functions available and when to use them can be found in the user 
# manual under 'Path Scripts -> 'Robot Commander Commands'
# ==============================================================================================================

### Have this line at the beginning of the script for every scenario. Each function call will begin with 'rc.'
rc = RobotCommander()

### Function Calls
rc.move_until_index(2, 0.1 * 9.81, 45)#Move at 2 m/s with 0.1g acceleration to index 45
rc.decel_to_stop_at_index(50) #Decelerate and stop at index 50
rc.rotate_until_heading(0.87, 45) #rotate at 0.87 rad/s to a heading of 45 degrees
rc.sleep(2) #Stop for 2 seconds
rc.move_until_index(2, 0.1 * 9.81, 93) #Move at 2 m/s with 0.1g acceleration to index 93
rc.decel_to_stop_at_index(98) #Decelerate and stop at index 98
## End of test
