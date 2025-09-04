import sys
import json
from controller import Robot    #Motor, DistanceSensor


# ================================= #
# === EXAMPLE WEBOTS CONTROLLER === #
# ================================= #


print(sys.path)
print("Controller started")

MAX_SPEED = 3   # max 6.28

def run_robot(robot):
    TIMESTEP = int(robot.getBasicTimeStep())
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    #ir0 = robot.getDevice('ps0')
    #ir0.enable(TIMESTEP)
    
    list_ps = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        list_ps.append(robot.getDevice(sensor_name))
        list_ps[-1].enable(TIMESTEP)
    
    # set inf to indicate that velocity (not position) is used for control
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # main loop
    while robot.step(TIMESTEP) != -1:
        #val = ds.getValue()
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        
        for ps in list_ps:
            ps_val = ps.getValue()
            
            if ps_val > 80:
                left_speed = -MAX_SPEED
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
