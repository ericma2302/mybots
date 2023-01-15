import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitude_front
        self.frequency = c.frequency_front
        if(self.jointName == 'Torso_BackLeg'):
            self.frequency = self.frequency /2
        self.offset = c.phaseOffset_front
        self.motorValues = numpy.linspace(0, 2*numpy.pi, c.iterations)
        for i in range(0, c.iterations):
          self.motorValues[i] = self.amplitude * numpy.sin(self.frequency * self.motorValues[i] + self.offset) 

    def Set_Value(self, robotId, time):
        pyrosim.Set_Motor_For_Joint(
                bodyIndex = robotId,
                jointName = self.jointName,
                controlMode = p.POSITION_CONTROL,
                targetPosition = self.motorValues[time],
                maxForce = c.maxForce)

    def Save_Values(self):
        numpy.save("data/" + self.jointName, self.motorValues)
