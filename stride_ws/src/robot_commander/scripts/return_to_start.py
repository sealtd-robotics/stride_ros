rc = RobotCommander()

print("INFO: Return to start")
rc.move_until_beginning_of_path(rc.reverse_speed_goal, rc.reverse_speed_rate)

