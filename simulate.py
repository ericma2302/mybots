from cmath import pi
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import random


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)

amplitude_back = pi/3
frequency_back = 10
phaseOffset_back = 0

amplitude_front = pi/4
frequency_front = 11
phaseOffset_front = 0

preSinValues = numpy.linspace(0, 2*pi, 1000)
sinValuesBack = numpy.linspace(0, 2*pi, 1000)
sinValuesFront = numpy.linspace(0, 2*pi, 1000)
for i in range(0, 1000):
    sinValuesBack[i] = amplitude_back * numpy.sin(frequency_back * preSinValues[i] + phaseOffset_back) 
    sinValuesFront[i] = amplitude_front * numpy.sin(frequency_front * preSinValues[i] + phaseOffset_front) 
#sinValuesFront = sinValuesBack



for i in range(0, 1000): 
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_BackLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = sinValuesBack[i],
        maxForce = 500)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = "Torso_FrontLeg",
        controlMode = p.POSITION_CONTROL,
        targetPosition = sinValuesFront[i],
        maxForce = 500)
    time.sleep(1/120)

numpy.save("data/sinValuesBack", sinValuesBack)
numpy.save("data/sinValuesFront", sinValuesFront)
numpy.save("data/backLegSensor", backLegSensorValues)
numpy.save("data/frontLegSensor", frontLegSensorValues)
p.disconnect()