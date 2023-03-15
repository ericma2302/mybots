import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import constants as c
from simulation import SIMULATION


#evolve phc, comment these lines until 19 to prevent new robots being generated

os.system("rm data/hc*")
os.system("rm bestFitnessRobots/*")
os.system("rm firstFitnessRobots/*")


for i in range(c.numSeeds):
    phc = PARALLEL_HILL_CLIMBER(i)
    phc.Evolve()


#run best robots in GUI mode
for i in range(c.numSeeds):
    os.system("cp bestFitnessRobots/body_Seed" + str(i) + ".urdf body_Seed" + str(i) + ".urdf")
    os.system("cp bestFitnessRobots/brain_Seed" + str(i) + ".nndf brain_Seed" + str(i) + ".nndf")
   
    os.system("python3 simulate.py GUI _Seed" + str(i) + " 2&>1")
   

    os.system("rm body_Seed" + str(i) + ".urdf")
    os.system("rm brain_Seed" + str(i) + ".nndf")
    os.system("rm fitness_Seed" + str(i) + ".txt")
