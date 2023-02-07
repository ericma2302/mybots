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

    def Start_Simulation(self, directOrGui):
        pyrosim.Start_SDF("world.sdf")
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) + " 2&>1 &")


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

        for i in range(10):
            pyrosim.Send_Cube(name="Box0" + str(i), pos=[1.75 + .5 * i ,0,.10/2* (i+1)] , size=[.5,30,.10* (i+1)], mass = 1000000, inertia=.83)
           # pyrosim.Send_Cube(name="Box1" + str(i), pos=[-1.75 - .5 * i ,0,.15* (i+1)] , size=[.5,30,.15* (i+1)], mass = 1000000, inertia=.83)
           # pyrosim.Send_Cube(name="Box2" + str(i), pos=[0  ,1.75 + .5 * i,.15* (i+1)] , size=[.5,30,.15* (i+1)], mass = 1000000, inertia=.83)


        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")

        zpos = 1

        torsox = 1
        torsoy = .7
        torsoz = .40
        legx = .3
        legy=.2
        legz = .35

        lowerlegx = .25
        lowerlegy = .1
        lowerlegz = zpos + torsoz/2 - torsoz - legz
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, zpos] , size=[torsox,torsoy,torsoz])


        #+x, +y
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [torsox / 2, torsoy /2, zpos - torsoz /2], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0, -legz/2] , size=[legx,legy,legz])

        pyrosim.Send_Joint(name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [legx / 2,0,-legz], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0, 0, -lowerlegz / 2] , size=[lowerlegx,lowerlegy,lowerlegz])

        #-x, -y
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-torsox / 2, -torsoy / 2, zpos - torsoz/2], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0, 0, -legz/2] , size=[legx,-legy,legz])

        pyrosim.Send_Joint(name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [-legx / 2,0,-legz], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0, 0, -lowerlegz / 2] , size=[lowerlegx,lowerlegy,lowerlegz])

        #-x, +y
        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-torsox / 2, torsoy / 2, zpos - torsoz/2], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[0, 0, -legz / 2] , size=[legx,legy,legz])

        pyrosim.Send_Joint(name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-legx/2,0,-legz], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0, 0, -legz/2] , size=[lowerlegx,lowerlegy,lowerlegz])

        #+x, -y
        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [torsox / 2, -torsoy/ 2, zpos - torsoz / 2], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0, 0, -legz/ 2] , size=[legx,-legy,legz])

        pyrosim.Send_Joint(name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [legx/2, 0,-legz], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0, 0, -legz/2] , size=[lowerlegx,lowerlegy,lowerlegz])

       # pyrosim.Send_Joint(name = "Torso_Head" , parent= "Torso" , child = "Head" , type = "revolute", position = [torsox/2, 0,1 + torsoz/2], jointAxis="0 0 1")
       # pyrosim.Send_Cube(name="Head", pos=[0, 0, .1] , size=[torsox/3,torsoy-.15,.3])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 5, linkName="FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName="BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7, linkName="LeftLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 8, linkName="RightLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 9, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = 'Torso_FrontLeg')
        pyrosim.Send_Motor_Neuron(name = 11, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 12, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 13, jointName="FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 14, jointName="BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 15, jointName="LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 16, jointName="RightLeg_RightLowerLeg")


        
        
        #pyrosim.Send_Synapse( sourceNeuronName = 0, targetNeuronName= 4, weight= 1.0)
        #pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=1.0)

        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn + c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])

        pyrosim.End()
    def Mutate(self):
        randomRow = random.randint(0, 2)
     
        randomColumn = random.randint(0, 1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, id):
        self.myID = id