# ros

# try_ws:
# All service functions are defined in DobotServer.cpp. DobotPublisher.cpp and DobotColor.cpp call the service function to get the sensor readings, and publish to DobotClient_step.cpp (main file).
# 1. DobotServer.cpp: all the service function are defined here.
# 2. DobotClient_step.cpp: main file.
# 3. DobotPublisher.cpp: publisher for infrared sensor.
# 4. DobotColor.cpp: publisher for color sensor.
# 5. DobotClient_other.cpp: Dobot arm initialization precedures (go to the home location, etc.).

# dobot_ws:
# An attempt to implement multiple agents.
# There are connectDobot() in DobotServer.cpp and InfraredAgent.cpp.
# Running both files will cause conflict.
