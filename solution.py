import numpy
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c



def getColorAndRbg(sensors, index):
    if sensors[index] == 1:
            colorName = 'Green'
            rbg = (0, 1.0, 0)
    else:
            colorName = 'Cyan'
            rbg = (0, 1.0, 1.0)
    return (colorName, rbg)

class SOLUTION:
    def __init__(self, nextAvailableID):
        self.weights = numpy.random.rand(9, 8) * 2 - 1
        self.myID = nextAvailableID
        self.sensors = []

    def Start_Simulation(self, directOrGui):
        pyrosim.Start_SDF("world.sdf")
        self.Create_World()
        choice = random.randint(0, 1)
        if choice == 0:
            self.Generate_3d_body()
        else:
            self.Generate_2d_body()
        self.Generate_3d_Brain()
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


    def Generate_3d_body(self):
        pyrosim.Start_URDF("body.urdf")
        #make random horse
        numLinks = 1
        numLegLinks = random.randint(1, 4)
        numLinks = numLinks + 4 * numLegLinks
        print(numLegLinks, numLinks)

        #randomize sensors
        sensors = []
        for i in range(numLinks):
            sensors.append(random.randint(0, 1))

        legHeights = []
        sum = 0
        for i in range(numLegLinks):
            height = random.random() * .5 + .1
            legHeights.append(height)
            sum += height
    

        torsoWidth = random.random() * .5 + .7
        torsoLength = random.random() * .5 + .7
        torsoHeight = random.random() * .5 + .1
        startHeight = sum + torsoHeight/2

        pyrosim.Send_Cube(name = '0', pos = [0, 0, startHeight], size=[torsoWidth, torsoLength, torsoHeight], colorName= getColorAndRbg(sensors, 0)[0], rgb= getColorAndRbg(sensors, 0)[1])


        linkWidth = random.random() * .5 + .1
        linkLength = random.random() * .5 + .1
        linkHeight = legHeights[0]


        pyrosim.Send_Joint(name = '0_10', parent='0', child='10', type="revolute", position=[torsoWidth/2, torsoLength/2, startHeight - torsoHeight/2], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '10', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 1)[0], rgb=getColorAndRbg(sensors, 1)[1])

        pyrosim.Send_Joint(name = '0_11', parent='0', child='11', type="revolute", position=[-torsoWidth/2, torsoLength/2, startHeight - torsoHeight/2], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '11', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 2)[0], rgb=getColorAndRbg(sensors, 2)[1])

        pyrosim.Send_Joint(name = '0_12', parent='0', child='12', type="revolute", position=[torsoWidth/2, -torsoLength/2, startHeight - torsoHeight/2], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '12', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 3)[0], rgb=getColorAndRbg(sensors, 3)[1])

        pyrosim.Send_Joint(name = '0_13', parent='0', child='13', type="revolute", position=[-torsoWidth/2, -torsoLength/2, startHeight - torsoHeight/2], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '13', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 4)[0], rgb=getColorAndRbg(sensors, 4)[1])


        prevLinkHeight = linkHeight
        for i in range(1, numLegLinks):
            linkWidth = random.random() * .7 + .1
            linkLength = random.random() * .5 + .1
            linkHeight = legHeights[i]

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '0', parent=str(i)+'0', child=str(i+1) + '0', type="revolute", position=[0, 0, -prevLinkHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '0', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 1)[0], rgb=getColorAndRbg(sensors, i * 4 + 1)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '1', parent=str(i)+'1', child=str(i+1) + '1', type="revolute", position=[0, 0, -prevLinkHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '1', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 2)[0], rgb=getColorAndRbg(sensors, i * 4 + 2)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '2', parent=str(i)+'2', child=str(i+1) + '2', type="revolute", position=[0, 0, -prevLinkHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '2', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 3)[0], rgb=getColorAndRbg(sensors, i * 4 + 3)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '3', parent=str(i)+'3', child=str(i+1) + '3', type="revolute", position=[0, 0, -prevLinkHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '3', pos=[0, 0, -linkHeight/2], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 4)[0], rgb=getColorAndRbg(sensors, i * 4 + 4)[1])
        
            prevLinkHeight = linkHeight


        self.sensors = sensors
        self.numArmLinks = numLegLinks
        pyrosim.End()


    def Generate_3d_Brain(self):
            pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
            if(self.sensors[0] == 1):
                pyrosim.Send_Sensor_Neuron(name = 0, linkName='0')
            #make sensors for the assigned links
            for i in range(1, len(self.sensors)):
                if(self.sensors[i] == 1):
                    pyrosim.Send_Sensor_Neuron(name = i , linkName = str(int((i-1)/4) + 1) + str((i-1) % 4))
            #attach a motor to every joint

            jointNos = 100
            for i in range(self.numArmLinks):
                pyrosim.Send_Motor_Neuron(name = jointNos, jointName=str(i) + '_' + str(i+1) + '0')
                pyrosim.Send_Motor_Neuron(name = jointNos + 1, jointName=str(i) + '_' + str(i+1) + '1')
                pyrosim.Send_Motor_Neuron(name = jointNos + 2, jointName=str(i) + '_' + str(i+1) + '2')
                pyrosim.Send_Motor_Neuron(name = jointNos + 3, jointName=str(i) + '_' + str(i+1) + '3')
                jointNos = jointNos + 4

        
            #create a syanpse between every sensor neuron and every motor neuron
            for i in range(1, len(self.sensors)):
                if(self.sensors[i] == 1):
                    jointNos = 100 + ((i - 1) % 4)
                    for j in range(self.numArmLinks):
                        pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=jointNos, weight=1)
                        jointNos += 4

            jointNos = 100
            if(self.sensors[0] == 1):
                for j in range(self.numArmLinks * 4):
                    pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=jointNos, weight=1)
                    jointNos + 1

            pyrosim.End()



    def Generate_2d_body(self):
        pyrosim.Start_URDF("body.urdf")

        #make a random lizard
        #first reserve 1 link for torso
        numLinks = 1
        #then randomize h w many links for arms
        numArmLinks = random.randint(2, 5)
        #four arms, so multiply by 4 and add to numLinks
        numLinks = numLinks + 4 * numArmLinks
        
        print(numArmLinks, numLinks)

        #randomize sensors
        sensors = []
        for i in range(numLinks):
            sensors.append(random.randint(0, 1))

        #build torso
        torsoWidth = random.random() * .5 + .7
        torsoLength = random.random() * 1.1 + .1
        torsoHeight = random.random() * 1.1 + .1

        pyrosim.Send_Cube(name = '0', pos = [0, 0, torsoHeight/2], size=[torsoWidth, torsoLength, torsoHeight], colorName= getColorAndRbg(sensors, 0)[0], rgb= getColorAndRbg(sensors, 0)[1])

        #build first four arms
        linkWidth = random.random() * .5 + .1
        linkLength = random.random() * .5 + .1
        linkHeight = random.random() * .5 + .1

        jointHeight = min(torsoHeight,linkHeight) / 2
        absolutePrevJointHeight = jointHeight
        zpos = linkHeight/2 - absolutePrevJointHeight
        prevLinkHeight = linkHeight
        prevLinkLength = linkLength

        pyrosim.Send_Joint(name = '0_10', parent='0', child='10', type="revolute", position=[torsoWidth/2, torsoLength/2, jointHeight], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '10', pos=[0, linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 1)[0], rgb=getColorAndRbg(sensors, 1)[1])

        pyrosim.Send_Joint(name = '0_11', parent='0', child='11', type="revolute", position=[-torsoWidth/2, torsoLength/2, jointHeight], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '11', pos=[0, linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 2)[0], rgb=getColorAndRbg(sensors, 2)[1])

        pyrosim.Send_Joint(name = '0_12', parent='0', child='12', type="revolute", position=[torsoWidth/2, -torsoLength/2, jointHeight], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '12', pos=[0, -linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 3)[0], rgb=getColorAndRbg(sensors, 3)[1])

        pyrosim.Send_Joint(name = '0_13', parent='0', child='13', type="revolute", position=[-torsoWidth/2, -torsoLength/2, jointHeight], jointAxis="1 0 0")
        pyrosim.Send_Cube(name = '13', pos=[0, -linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, 4)[0], rgb=getColorAndRbg(sensors, 4)[1])

        #now build out arms
        for i in range(1, numArmLinks):
            linkWidth = random.random() * .5 + .1
            linkLength = random.random() * .5 + .1
            linkHeight = random.random() * .5 + .1

            jointHeight = min(prevLinkHeight,linkHeight)/2 - absolutePrevJointHeight
            absolutePrevJointHeight = jointHeight +absolutePrevJointHeight
            zpos = linkHeight/2 - absolutePrevJointHeight



            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '0', parent=str(i)+'0', child=str(i+1) + '0', type="revolute", position=[0, prevLinkLength, jointHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '0', pos=[0, linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 1)[0], rgb=getColorAndRbg(sensors, i * 4 + 1)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '1', parent=str(i)+'1', child=str(i+1) + '1', type="revolute", position=[0, prevLinkLength, jointHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '1', pos=[0, linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 2)[0], rgb=getColorAndRbg(sensors, i * 4 + 2)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '2', parent=str(i)+'2', child=str(i+1) + '2', type="revolute", position=[0, -prevLinkLength, jointHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '2', pos=[0, -linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 3)[0], rgb=getColorAndRbg(sensors, i * 4 + 3)[1])

            pyrosim.Send_Joint(name = str(i) + '_' + str(i+1) + '3', parent=str(i)+'3', child=str(i+1) + '3', type="revolute", position=[0, -prevLinkLength, jointHeight], jointAxis="1 0 0")
            pyrosim.Send_Cube(name = str(i+1) + '3', pos=[0, -linkLength/2, zpos], size=[linkWidth, linkLength, linkHeight], colorName=getColorAndRbg(sensors, i * 4 + 4)[0], rgb=getColorAndRbg(sensors, i * 4 + 4)[1])
        
            prevLinkLength = linkLength
            prevLinkHeight = linkHeight
        self.sensors = sensors
        self.numArmLinks = numArmLinks
        pyrosim.End()


   

    def Generate_1d_Body(self):
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
           

        #join with joints (not sure which axis to rotate on)
        
        self.sensors = sensors

        pyrosim.End()

    def Generate_1d_Brain(self):
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