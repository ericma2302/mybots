import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(500)
frontLegSensorValues = numpy.zeros(500)

for i in range(0, 500): 
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    time.sleep(1/60)
#print(backLegSensorValues)
numpy.save("data/backLegSensor", backLegSensorValues)
numpy.save("data/frontLegSensor", frontLegSensorValues)
p.disconnect()