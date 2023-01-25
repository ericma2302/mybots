from world import WORLD
from robot import ROBOT
import pybullet as p
import time
import pybullet_data
import constants as c




class SIMULATION:
    def __init__(self, directOrGui):
        if directOrGui == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)    
        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(0, c.iterations): 
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act()

            time.sleep(c.timeStep)

    def Get_Fitness(self):
        self.robot.Get_Fitness()


    def __del__(self):
        p.disconnect()
