from turtle import back
import numpy
import matplotlib.pyplot

fitnessCurve = numpy.load('data/hc9.npy')

#print(back_data)
#print(front_data)
xticks = [i*50 for i in range(10)]

matplotlib.pyplot.plot(fitnessCurve, label='Fitness Values', linewidth =3)
matplotlib.pyplot.xticks(xticks)
matplotlib.pyplot.ylabel("Distance in x-axis")
matplotlib.pyplot.xlabel("Generation #")
matplotlib.pyplot.legend()
matplotlib.pyplot.show()


