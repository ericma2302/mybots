from os import link
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import pybullet as p
from sensor import SENSOR
from motor import MOTOR
import os
import constants as c

class ROBOT:
    def __init__(self, solutionID):

        self.robotId = p.loadURDF("body.urdf")
        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        #os.system("rm brain" + str(solutionID) + ".nndf")
        self.solutionID = solutionID
        

    def Prepare_To_Sense(self):
        self.sensors = {}
        print('helllo')
        for linkName in pyrosim.linkNamesToIndices:
            print(pyrosim.linkNamesToIndices)
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, time):
        for sensor in self.sensors:
            self.sensors[sensor].Get_Value(time)

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self):

        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                print(self.nn.Get_Value_Of(neuronName))
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                jointName = self.nn.Get_Motor_Sensor_Joint(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)

                #print(neuronName, jointName)


    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0] * .7 + basePosition[2] * .3#- abs(basePosition[1])


        stateOfLinkZero = p.getLinkState(self.robotId,0)
        if(basePosition[2] < stateOfLinkZero[0][2]):
            xPosition = -10
        stateOfLinkOne = p.getLinkState(self.robotId,1)
        if(basePosition[2] < stateOfLinkOne[0][2]):
            xPosition = -10
        stateOfLinkTwo = p.getLinkState(self.robotId,2)
        if(basePosition[2] < stateOfLinkTwo[0][2]):
            xPosition = -10
        stateOfLinkThree = p.getLinkState(self.robotId,3)
        if(basePosition[2] < stateOfLinkThree[0][2]):
            xPosition = -10
        stateOfLinkFour = p.getLinkState(self.robotId,4)
        if(basePosition[2] < stateOfLinkFour[0][2]):
            xPosition = -10
            
        f = open("tmp" + str(self.solutionID) + ".txt", "w")
        f.write(str(xPosition))
        f.close()

        os.system("mv tmp" + str(self.solutionID) + ".txt fitness" + str(self.solutionID) + ".txt")
        