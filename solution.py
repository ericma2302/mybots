import numpy
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c



def getColorAndRbg(val):
    if val == 1:
            colorName = 'Green'
            rbg = (0, 1.0, 0)
    else:
            colorName = 'Cyan'
            rbg = (0, 1.0, 1.0)
    return (colorName, rbg)

class SOLUTION:
    def __init__(self, nextAvailableID):

        self.links = {}
        self.instructions = []
        self.sensors = []
        self.joints = []
        self.directions = {}
        self.weights = []
        self.nextAvailableName = 0
        self.myID = nextAvailableID
        self.maxDepth = 3

    def clearAll(self):
        self.links.clear()
        self.instructions.clear()
        self.sensors.clear()
        self.joints.clear()
        self.directions.clear()
        self.weights.clear()
        self.nextAvailableName = 0

    def Start_Simulation(self, directOrGui):
        pyrosim.Start_SDF("world.sdf")
        self.Create_World()
        if len(self.instructions) == 0:
            self.Generate_3d_body()
            while len(self.instructions) < 3:
                self.clearAll()
                self.Generate_3d_body()
        self.doInstructions()
        self.Generate_3d_Brain()

        os.system("python3 simulate.py " + directOrGui + " " + str(self.myID) + " 2&>1")


    def Wait_For_Simulation_To_End(self):
        fitnessString = "fitness" + str(self.myID) + ".txt"
        
        while not os.path.exists(fitnessString):
            time.sleep(0.001)

        f = open(fitnessString, "r")
        self.fitness = float(f.read())

        f.close()

        os.system("rm " + fitnessString)
        os.system("rm brain" + str(self.myID) + ".nndf")
        os.system("rm body" + str(self.myID) + ".urdf")


    def Create_World(self):
        pyrosim.End()


    def checkContact(self, linkCoords, linkSize):
        minx = linkCoords[0] - linkSize[0]/2
        maxx = linkCoords[0] + linkSize[0]/2

        miny = linkCoords[1] - linkSize[1]/2
        maxy = linkCoords[1] + linkSize[1]/2

        minz = linkCoords[2] - linkSize[2]/2
        maxz = linkCoords[2] + linkSize[2]/2

        xoverlap = False
        yoverlap = False
        zoverlap = False

        for link in self.links.keys():
            if (self.links[link][0][1] > minx) and (self.links[link][0][0] < maxx):
                xoverlap = True
            if (self.links[link][1][1] > miny) and (self.links[link][1][0] < maxy):
                yoverlap = True
            if (self.links[link][2][1] > minz) and (self.links[link][2][0] < maxz):
                zoverlap = True
            if minz < 0 or maxz <=0:
                return True

            if(xoverlap and yoverlap and zoverlap):
                return True
            else:
                xoverlap = False
                yoverlap = False
                zoverlap = False

        return False




    def makeNextLink(self, depth, direction, prevLinkAbsCoords, prevLinkSize, abs, prevJoint, parentName):
        
        linkWidth = random.random() * .5 + .1
        linkLength = random.random() * .5 + .1
        linkHeight = random.random() * .5 + .1
        sensor = random.randint(0, 1)
        colorAndRbg = getColorAndRbg(sensor)

        if depth == self.maxDepth: #calculate this
            return 0

        jointpos = None
        if direction == -1:


            #name, pos, size, colors, parentName, jointpos, jointaxis
            self.instructions.append(('0', [0, 0, linkHeight/2], [linkWidth, linkLength, linkHeight], (colorAndRbg[0], colorAndRbg[1]),
                    None, None, None))

            linkAbsPos = [0, 0, linkHeight/2]

            self.links[self.nextAvailableName] = [[linkAbsPos[0] - linkWidth/2, linkAbsPos[0] + linkWidth/2], 
                [linkAbsPos[1] - linkLength/2, linkAbsPos[1] + linkLength/2],
                [linkAbsPos[2] - linkHeight/2, linkAbsPos[2] + linkHeight/2]]

            self.nextAvailableName += 1
            if sensor == 1:
                self.sensors.append(0)

            for i in range(1, 7):
                rand = random.randint(1,1)
                if rand == 1:
                    self.makeNextLink(depth + 1, i, linkAbsPos, [linkWidth, linkLength, linkHeight], 1, direction, "0")
            return

        elif direction == 1:
            if abs == 1:
                jointpos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1], prevLinkAbsCoords[2] + prevLinkSize[2] / 2]
            else:
                if prevJoint == 1:
                    jointpos = [0, 0, prevLinkSize[2]]
                elif prevJoint == 3:
                    jointpos = [-prevLinkSize[0]/2, 0, prevLinkSize[2]/2]
                elif prevJoint == 4:
                    jointpos = [prevLinkSize[0]/2, 0, prevLinkSize[2]/2]
                elif prevJoint == 5:
                    jointpos = [0, -prevLinkSize[1]/2, prevLinkSize[2]/2]
                elif prevJoint ==6 :
                    jointpos = [0, prevLinkSize[1]/2, prevLinkSize[2]/2]

            linkPos = [0, 0, linkHeight/2]
            linkAbsPos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1], prevLinkAbsCoords[2] + prevLinkSize[2]/2 + linkHeight/2]
            jointAxis = "0 1 1"
        elif direction == 2:
            if prevJoint == 2:
                jointpos = [0, 0, -prevLinkSize[2]]
            elif prevJoint == 3:
                jointpos = [-prevLinkSize[0]/2, 0, -prevLinkSize[2]/2]
            elif prevJoint == 4:
                jointpos = [prevLinkSize[0]/2, 0, -prevLinkSize[2]/2]
            elif prevJoint == 5:
                jointpos = [0, -prevLinkSize[1]/2, -prevLinkSize[2]/2]
            elif prevJoint == 6:
                jointpos = [0, prevLinkSize[1]/2, -prevLinkSize[2]/2]

            linkPos = [0, 0, -linkHeight/2]
            linkAbsPos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1], prevLinkAbsCoords[2] - prevLinkSize[2]/2 - linkHeight/2]
            jointAxis = "0 1 1"
        elif direction == 3:
            if abs == 1:
                jointpos = [prevLinkAbsCoords[0] - prevLinkSize[0]/2, prevLinkAbsCoords[1], prevLinkAbsCoords[2]]
            else:
                if prevJoint == 1:
                    jointpos = [-prevLinkSize[0]/2, 0, prevLinkSize[2]/2]
                elif prevJoint == 3:
                    jointpos = [-prevLinkSize[0], 0, 0]
                elif prevJoint == 2:
                    jointpos = [-prevLinkSize[0]/2, 0, -prevLinkSize[2]/2]
                elif prevJoint == 5:
                    jointpos = [-prevLinkSize[0]/2, -prevLinkSize[1]/2, 0]
                elif prevJoint ==6 :
                    jointpos = [-prevLinkSize[0]/2, prevLinkSize[1]/2, 0]

            linkPos = [-linkWidth/2, 0, 0]
            linkAbsPos = [prevLinkAbsCoords[0] - prevLinkSize[0]/2 - linkWidth/2, prevLinkAbsCoords[1], prevLinkAbsCoords[2]]
            jointAxis = "1 1 0" 
        elif direction == 4:
            if abs == 1:
                jointpos = [prevLinkAbsCoords[0] + prevLinkSize[0]/2, prevLinkAbsCoords[1], prevLinkAbsCoords[2]]
            else:
                if prevJoint == 1:
                    jointpos = [prevLinkSize[0]/2, 0, prevLinkSize[2]/2]
                elif prevJoint == 4:
                    jointpos = [prevLinkSize[0], 0, 0]
                elif prevJoint == 2:
                    jointpos = [prevLinkSize[0]/2, 0, -prevLinkSize[2]/2]
                elif prevJoint == 5:
                    jointpos = [prevLinkSize[0]/2, -prevLinkSize[1]/2, 0]
                elif prevJoint ==6 :
                    jointpos = [prevLinkSize[0]/2, prevLinkSize[1]/2, 0]

            linkPos = [linkWidth/2, 0, 0]
            linkAbsPos = [prevLinkAbsCoords[0] + prevLinkSize[0]/2 + linkWidth/2, prevLinkAbsCoords[1], prevLinkAbsCoords[2]]
            jointAxis = "1 1 0"
        elif direction == 5:
            if abs == 1:
                jointpos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1] - prevLinkSize[1]/2, prevLinkAbsCoords[2]]
            else:
                if prevJoint == 1:
                    jointpos = [0, -prevLinkSize[1]/2, prevLinkSize[2]/2]
                elif prevJoint == 4:
                    jointpos = [prevLinkSize[0]/2, -prevLinkSize[1]/2, 0]
                elif prevJoint == 2:
                    jointpos = [0, -prevLinkSize[1]/2, -prevLinkSize[2]/2]
                elif prevJoint == 5:
                    jointpos = [0, -prevLinkSize[1], 0]
                elif prevJoint ==3 :
                    jointpos = [-prevLinkSize[0]/2, -prevLinkSize[1]/2, 0]

            linkPos = [0, -linkLength/2, 0]
            linkAbsPos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1] - prevLinkSize[1]/2 - linkLength/2, prevLinkAbsCoords[2]]
            jointAxis = "1 1 0"
        else:
            if abs == 1:
                jointpos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1] + prevLinkSize[1]/2, prevLinkAbsCoords[2]]
            else:
                if prevJoint == 1:
                    jointpos = [0, prevLinkSize[1]/2, prevLinkSize[2]/2]
                elif prevJoint == 4:
                    jointpos = [prevLinkSize[0]/2, prevLinkSize[1]/2, 0]
                elif prevJoint == 2:
                    jointpos = [0, prevLinkSize[1]/2, -prevLinkSize[2]/2]
                elif prevJoint == 6:
                    jointpos = [0, prevLinkSize[1], 0]
                elif prevJoint ==3 :
                    jointpos = [-prevLinkSize[0]/2, prevLinkSize[1]/2, 0]

            linkPos = [0, linkLength/2, 0]
            linkAbsPos = [prevLinkAbsCoords[0], prevLinkAbsCoords[1] + prevLinkSize[1]/2 + linkLength/2, prevLinkAbsCoords[2]]

            jointAxis = "1 1 0"
        

        
        if jointpos == None or self.checkContact(linkAbsPos, [linkWidth, linkLength, linkHeight]):
            return -1


        self.links[self.nextAvailableName] = [[linkAbsPos[0] - linkWidth/2, linkAbsPos[0] + linkWidth/2], 
            [linkAbsPos[1] - linkLength/2, linkAbsPos[1] + linkLength/2],
            [linkAbsPos[2] - linkHeight/2, linkAbsPos[2] + linkHeight/2]]

        self.joints.append(parentName + "_" + str(self.nextAvailableName))


        #name, pos, size, colors, parentName, jointpos, jointaxis
        self.instructions.append((str(self.nextAvailableName), linkPos, [linkWidth, linkLength, linkHeight], (colorAndRbg[0], colorAndRbg[1]), 
                parentName, jointpos, jointAxis))

        name = self.nextAvailableName
        if sensor == 1:
            self.sensors.append(name) 
        self.directions[name] = [parentName, direction]
        
        self.nextAvailableName += 1

        for i in range(1, 7):
            rand = random.randint(1,2)
            if rand == 1:
                self.makeNextLink(depth + 1, i, linkAbsPos, [linkWidth, linkLength, linkHeight], 0, direction, str(name))
        
        return 1

    def doInstructions(self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")
        for instruction in self.instructions:
            name = instruction[0]
            pos = instruction[1]
            size = instruction[2]
            colors = instruction[3]
            parentName = instruction[4]
            jointpos = instruction[5]
            jointaxis = instruction[6]

            if parentName == None:
                 pyrosim.Send_Cube(name = name, pos = [0, 0, size[2] / 2], size=[size[0], size[1], size[2]], 
                    colorName= colors[0], rgb= colors[1])
            else:
                 pyrosim.Send_Joint(name = parentName + "_" + name, parent = parentName, child=name, type = "revolute", position=jointpos, jointAxis=jointaxis)
                 pyrosim.Send_Cube(name = name, pos = pos, size=[size[0], size[1], size[2]], colorName = colors[0], rgb= colors[1])

        pyrosim.End()

    def Generate_3d_body(self):

        self.makeNextLink(0, -1, None, None, None, None, None)
        print(self.instructions)
        #make random cube, randomize how many directions to branch into, randomize sensors
        #if the space is alreaady taken, just return

        #how to know if space is already taken? dictionary of min max x, y, z coordinates

    def Generate_3d_Brain(self):
            pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
            #make sensors for the assigned links
            for i in range(0, len(self.sensors)):
                pyrosim.Send_Sensor_Neuron(name = i , linkName = str(self.sensors[i]))
            #attach a motor to every joint

            for i in range(len(self.joints)):
                pyrosim.Send_Motor_Neuron(name = 100 + i, jointName=self.joints[i])

        
            #create a syanpse between every sensor neuron and every motor neuron

            if len(self.weights) == 0:
                for i in range(len(self.sensors)):
                    for j in range(len(self.joints)):
                        self.weights.append(random.random() * 2 - .5)
            

            self.weights = numpy.reshape(self.weights, (len(self.sensors), len(self.joints)))


            for i in range(0, len(self.sensors)):
                for j in range(len(self.joints)):
                    pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=100 + j, weight= self.weights[i][j])

            pyrosim.End()


    def addInstruction(self):
        #add = random.randint(0, 1) only add for now
        
        #pick random link, pick random direction, add a new link
        added = -1
 

        while added == -1:
            randLink = random.choice(list(self.links))
            #print('key is ', randLink)
            # randLink = self.links[randKey]
            linkCoords = self.links[randLink]
            randDirection = random.randint(0, 6)
            prevLinkAbsCoords = [(linkCoords[0][0] + linkCoords[0][1])/2, (linkCoords[1][0] + linkCoords[1][1])/2, (linkCoords[2][0] + linkCoords[2][1])/2]
            prevLinkSize = [abs(linkCoords[0][0] - linkCoords[0][1]), abs(linkCoords[1][0] - linkCoords[1][1]), abs(linkCoords[2][0] -
                linkCoords[2][1])]
            absol = 1 if randLink == 0 else 0
            
            prevJoint = self.directions[randLink][1] if randLink != 0 else 0
            # parentName = self.directions[randLink][0]

            added = self.makeNextLink(depth = self.maxDepth - 1, direction = randDirection, prevLinkAbsCoords=prevLinkAbsCoords, prevLinkSize=
                prevLinkSize, abs = absol, prevJoint=prevJoint, parentName=str(randLink))

        #update weights (possible one more row, definitely one more column)
        newWeights = []
        for i in range(len(self.sensors)):
            if(i < len(self.weights)):
                for j in range(len(self.joints)):
                    if j < len(self.joints) - 1:
                        newWeights.append(self.weights[i][j])
                    else:
                        newWeights.append(random.random() * 2 - .5)
            else:
                for j in range(len(self.joints)):
                    newWeights.append(random.random() * 2 - .5)
        self.weights = numpy.reshape(newWeights, (len(self.sensors), len(self.joints)))


    def removeInstruction(self):
        allLeafs = []
        for link in self.links.keys():
            allLeafs.append(int(link))

        for direction in self.directions.keys():
            
            parent = int(self.directions[direction][0])
            if parent in allLeafs:
                allLeafs.remove(parent)
        
        
        #randomize which leaf to remove
        randomLeaf = random.randint(0, len(allLeafs) - 1)
        randomLeaf = allLeafs[randomLeaf]
        #print('removing link ', randomLeaf)

        #name, pos, size, colors, parentName, jointpos, jointaxis
        self.instructions = [instruction for instruction in self.instructions if instruction[0] != str(randomLeaf)]
 

        #update self.links
        self.links.pop(randomLeaf)

        #update self.sensors

        rowIndex = None
        if randomLeaf in self.sensors:
            rowIndex  = self.sensors.index(randomLeaf)
            self.sensors.remove(randomLeaf)
 

        #update self.joints
        for i in range(len(self.joints)):
            joint = self.joints[i]
            if joint.split("_")[1] == str(randomLeaf):
                columnIndex = i #find column for weights

                

        self.joints = [joint for joint in self.joints if joint.split("_")[1] != str(randomLeaf)]


        #update self.directions, pop off leaf being removed
        self.directions.pop(randomLeaf)

        #update weights (one less row possibly, definitely one less columns
        self.weights = numpy.delete(self.weights, columnIndex, 1)
        if rowIndex != None:
            self.weights = numpy.delete(self.weights, rowIndex, 0)
        


    def Mutate(self):
        #print('mutating')
        if len(self.links) == 1: #this should never happen, but just in case, reset
            self.clearAll()
            self.Generate_3d_body() 
            return


        choice = random.randint(0, 1)
        if choice == 0 or len(self.weights) == 0:
            self.MutateShape()
        else:
            self.MutateSynapse()
        



    def MutateShape(self):
        #print('mutating shape')
        if(len(self.links) <= 3):
            choice = 0
        elif(len(self.links) >= 12):
            choice = 1
        else:
            choice = random.randint(0, 1)

        if choice == 0:
            self.addInstruction()
        else:
            self.removeInstruction()
    def MutateSynapse(self):
        #print('mutating synapse')
        randomRow = random.randint(0, len(self.weights) - 1)
        randomColumn = random.randint(0, len(self.weights[0]) - 1)
        self.weights[randomRow, randomColumn] = random.random() * 2 - .5

    def Set_ID(self, id):
        self.myID = id