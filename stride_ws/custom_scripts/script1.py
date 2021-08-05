speed = 1
t = 1000

rc = RobotCommander()
print("1")
rc.go_straight_for_milliseconds(speed, t)
print("2")
time.sleep(1)
print("3")
rc.go_straight_for_milliseconds(speed, t)
print("4")

# while True:
#     print("hi")
#     time.sleep(1)


