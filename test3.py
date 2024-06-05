from stwrt_move_nm1 import *
import csv
import time

# Create a Robot instance with specified parameters
robot = Robot(port='COM4', LINK_1=80, LINK_2=230) 


## ------------------------------------------------------------------------------------------------

### Continous movement in X-direction
# for x in range(0,101,1):
#     robot.goto(x=x, y=0, z=290+0, roll=0, pitch=0, yaw=0)
#     time.sleep(1)


### Continous movement in Y-direction
# for y in range(0,101,1):
#     robot.goto(x=0, y=y, z=290+0, roll=0, pitch=0, yaw=0)
#     time.sleep(1)


### Continous movement in Z-direction
# for z in range(0,101,1):
#     robot.goto(x=0, y=0, z=290+z, roll=0, pitch=0, yaw=0)
#     time.sleep(1)

### Continous movement in ROLL
# for roll in range(0,21,1):
#     robot.goto(x=0, y=0, z=290+0, roll=roll, pitch=0, yaw=0)
#     time.sleep(1)


### Continous movement in PITCH
# for pitch in range(0,15,1):
#     robot.goto(x=0, y=0, z=290+0, roll=0, pitch=pitch, yaw=0)
#     time.sleep(1)


### Continous movement in YAW
for yaw in range(0,20,1):
    robot.goto(x=0, y=0, z=290+0, roll=0, pitch=0, yaw=yaw)
    time.sleep(1)


## ------------------------------------------------------------------------------------------------

# for x in range(0,50,10):
#     for y in range(0,50,10):
#         for z in range(0,50,10):
#             # for roll in range(1,20):
#             #     for pitch in range(1,20):
#             #         for yaw in range(1,5):
#                         print('\n \n -----------')
#                         print(f"x={x}, y={y}, z={z}")
#                         print('-----------')
#                         robot.move(x=x, y=y, z=z, roll=0, pitch=0, yaw=0)

# Close the port
#robot.close()