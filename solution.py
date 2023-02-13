import numpy
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = numpy.random.rand(9, 8) * 2 - 1
        self.myID = nextAvailableID
        self.sensors = []

    def Start_Simulation(self, directOrGui):
        pyrosim.Start_SDF("world.sdf")
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) + " 2&>1")


    def Wait_For_Simulation_To_End(self):
        fitnessString = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessString):
            time.sleep(0.001)

        f = open(fitnessString, "r")
        self.fitness = float(f.read())
       # print(self.fitness)
        f.close()

        os.system("rm " + fitnessString)

    def Evaluate(self, directOrGui):
        pass
        



    def Create_World(self):

        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")

        #make random things
        #first randomize how many links there will be 
        numLinks = random.randint(5, 12)
        #randomly assign sensors to some links
        sensors = []
        for i in range(numLinks):
            sensors.append(random.randint(0, 1))
            #0 represent no sensor, 1 represent sensor
        #color links accordingly
        #randomly shape links
        for i in range(numLinks):
            linkWidth = random.random() * 1.1 + .1
            linkLength = random.random() * 1.1 + .1
            linkHeight = random.random() * 1.1 + .1
            if sensors[i] == 1:
                colorName = 'Green'
                rbg = (0, 1.0, 0)
            else:
                colorName = 'Cyan'
                rbg = (0, 1.0, 1.0)

            if(i != 0):
                jointHeight = min(previousLinkHeight/2, linkHeight/2) if i == 1 else min(previousLinkHeight/2, linkHeight/2) - absolutePrevJointHeight
                pyrosim.Send_Joint(name = str(i-1) + '_' + str(i) , parent= str(i-1) , child = str(i) , type = "revolute", position = [0, previousWidth, jointHeight], jointAxis="0 0 1")

            previousLinkHeight = linkHeight
            previousWidth = linkWidth
            absolutePrevJointHeight = 0 if i == 0 else absolutePrevJointHeight + jointHeight

            zpos = linkHeight/2 if i == 0 else -absolutePrevJointHeight + linkHeight/2
            pyrosim.Send_Cube(name=str(i), pos=[0, linkWidth / 2, zpos] , size=[linkLength,linkWidth,linkHeight], colorName=colorName, rgb=rbg)
           
            print(" zpos: ", zpos, " link height: ", linkHeight, ' absol joint height: ', absolutePrevJointHeight)

        #join with joints (not sure which axis to rotate on)
        
        self.sensors = sensors



        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        #make sensors for the assigned links
        for i in range(len(self.sensors)):
            if(self.sensors[i] == 1):
                pyrosim.Send_Sensor_Neuron(name = i , linkName = str(i))
                print('sending sensor for ', i)
            if(i != len(self.sensors) - 1):
                pyrosim.Send_Motor_Neuron(name = i + 100, jointName = str(i) + "_" + str(i+1))
        #attach a motor to every joint
    
        #create a syanpse between every sensor neuron and every motor neuron
        for i in range(len(self.sensors)):
            if(self.sensors[i] == 1):
                for j in range(len(self.sensors) - 1):
                    pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + 100, weight=1)

        pyrosim.End()
    def Mutate(self):
        randomRow = random.randint(0, 2)
     
        randomColumn = random.randint(0, 1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, id):
        self.myID = id