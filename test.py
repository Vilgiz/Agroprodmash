from Robot import Robot


robot_tsp = Robot(print_debug = True)
robot_tsp.start()
robot_tsp.send_start()
while (True):
    robot_tsp.send_step(2, 3)

#d.send_step(hole[0], hole[1])
#d.send_step(hole[0], hole[1])