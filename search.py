import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER

os.system("rm data/hc*")
for i in range(5):
    phc = PARALLEL_HILL_CLIMBER(i)
    phc.Show_First()
    phc.Evolve()
    phc.Show_Best()