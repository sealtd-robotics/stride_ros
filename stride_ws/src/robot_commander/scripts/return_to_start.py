rc = RobotCommander()

print("INFO: Return to start")
rc.move_until_beginning_of_path(-2, 0.5)