from solution import SOLUTION
import constants as c
import copy
import os
import matplotlib.pyplot
import numpy

class PARALLEL_HILL_CLIMBER:
    def __init__(self, id):
        os.system("rm brain*.nndf")
        os.system("rm body*.urdf")
        os.system("rm fitness*.txt")

        self.id = str(id)
        self.parents = {}
        self.nextAvailbleID = 0
        self.fitnessCurve = numpy.zeros(c.numberOfGenerations)
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailbleID)
            self.nextAvailbleID = self.nextAvailbleID + 1

    def Evolve(self):
        self.Evaluate(self.parents)
       
        
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration)


        numpy.save("data/hc" + self.id, self.fitnessCurve)

    def Evolve_For_One_Generation(self, currentGen):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
        self.recordBestFitness(currentGen)

    def Spawn(self):
        self.children = {}
        for parent in self.parents:
            self.children[parent] = copy.deepcopy(self.parents[parent])
            self.children[parent].Set_ID(self.nextAvailbleID)
            self.nextAvailbleID = self.nextAvailbleID + 1


    def Mutate(self):
        for child in self.children:
            self.children[child].Mutate()
        #print(self.parent.weights)
        #print(self.child.weights)

    def Select(self):
        for parent in self.parents:
            if(self.parents[parent].fitness < self.children[parent].fitness):
                self.parents[parent] = self.children[parent]

    

    def recordBestFitness(self, currentGen):
        best_fitness_parent = 0
        for parent in self.parents:
            if self.parents[parent].fitness > self.parents[best_fitness_parent].fitness:
                best_fitness_parent = parent
        self.fitnessCurve[currentGen] = self.parents[best_fitness_parent].fitness

        if currentGen == c.numberOfGenerations - 1:
            self.parents[best_fitness_parent].Start_Simulation("DIRECT")
            os.system("mv brain* bestFitnessRobots/brain_Seed" + str(self.id) + ".nndf")
            os.system("rm fitness*")
            os.system("mv body* bestFitnessRobots/body_Seed" + str(self.id) + ".urdf")

        
    def Print(self):
        print('\n')
        for parent in self.parents:
            print('parent fitness: ', self.parents[parent].fitness,
                ' child fitness: ', self.children[parent].fitness)
        print('\n')
    
    def Show_First(self):
        self.parents[0].Start_Simulation("GUI")
        self.parents[0].Wait_For_Simulation_To_End()
        print(len(self.parents[0].links))

    def Show_Best(self):
        lowest_fitness_parent = 0
        for parent in self.parents:
            if self.parents[parent].fitness > self.parents[lowest_fitness_parent].fitness:
                lowest_fitness_parent = parent
        self.parents[lowest_fitness_parent].Start_Simulation("GUI")
        print(len(self.parents[lowest_fitness_parent].links))

        os.system("mv brain* bestFitnessRobots/brain_Seed" + str(self.id) + ".nndf")
        os.system("rm fitness*")
        os.system("mv body* bestFitnessRobots/body_Seed" + str(self.id) + ".urdf")


    def Evaluate(self, solutions, save = False):
        for solution in solutions:
            solutions[solution].Start_Simulation("DIRECT")
        for solution in solutions:
            # if save:
            #     solutions[solution].Wait_For_Simulation_To_End(True, self.id)
            # else:
                solutions[solution].Wait_For_Simulation_To_End()
