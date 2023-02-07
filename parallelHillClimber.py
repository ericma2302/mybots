from solution import SOLUTION
import constants as c
import copy
import os

class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")

        self.parents = {}
        self.nextAvailbleID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailbleID)
            self.nextAvailbleID = self.nextAvailbleID + 1

    def Evolve(self):
        self.Evaluate(self.parents)
       
        
        #self.parent.Evaluate("GUI")
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

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
        
    def Print(self):
        print('\n')
        for parent in self.parents:
            print('parent fitness: ', self.parents[parent].fitness,
                ' child fitness: ', self.children[parent].fitness)
        print('\n')
    
    def Show_First(self):
        self.parents[0].Start_Simulation("GUI")
        self.parents[0].Wait_For_Simulation_To_End()

    def Show_Best(self):
        lowest_fitness_parent = 0
        for parent in self.parents:
            if self.parents[parent].fitness > self.parents[lowest_fitness_parent].fitness:
                lowest_fitness_parent = parent
        self.parents[lowest_fitness_parent].Start_Simulation("GUI")

    def Evaluate(self, solutions):
        for solution in solutions:
            solutions[solution].Start_Simulation("DIRECT")
        for solution in solutions:
            solutions[solution].Wait_For_Simulation_To_End()
