from turtle import back
import numpy
import matplotlib.pyplot

back_data = numpy.load('data/backLegSensor.npy')
front_data = numpy.load('data/frontLegSensor.npy')
print(back_data)
print(front_data)

matplotlib.pyplot.plot(back_data, label='Back Leg', linewidth=3)
matplotlib.pyplot.plot(front_data, label='Front Leg')

matplotlib.pyplot.legend(loc=1)
matplotlib.pyplot.show()