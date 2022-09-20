rc = RobotCommander()

print("INFO: Return to start")
rc.move_until_beginning_of_path(-1.5, 0.1*9.81)