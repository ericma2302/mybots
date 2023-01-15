from world import WORLD
from robot import ROBOT
import pybullet as p
import time
import pybullet_data
import constants as c




class SIMULATION:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)    
        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(0, c.iterations): 
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i)

            time.sleep(c.timeStep)

    def __del__(self):
        p.disconnect()
