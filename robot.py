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

        self.robotId = p.loadURDF("body" + str(solutionID) + ".urdf")
        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        self.solutionID = solutionID

    def deleteFiles(self):
        os.system("rm brain" + str(self.solutionID) + ".nndf")
        os.system("rm body" + str(self.solutionID) + ".urdf")
        

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            #print(pyrosim.linkNamesToIndices)
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
                #print(self.nn.Get_Value_Of(neuronName))
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                jointName = self.nn.Get_Motor_Sensor_Joint(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)

                #print(neuronName, jointName)


    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        basePositionAndOrientation = p.getLinkState(self.robotId,0)
        basePosition = basePositionAndOrientation[0]
        xPosition = basePosition[0]

            
        f = open("tmp" + str(self.solutionID) + ".txt", "w")
        f.write(str(xPosition))
        f.close()

        os.system("mv tmp" + str(self.solutionID) + ".txt fitness" + str(self.solutionID) + ".txt")
        