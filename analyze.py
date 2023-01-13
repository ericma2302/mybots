from turtle import back
import numpy
import matplotlib.pyplot

back_data = numpy.load('data/backLegSensor.npy')
front_data = numpy.load('data/frontLegSensor.npy')
sin_valuesBack = numpy.load('data/sinValuesBack.npy')
sin_valuesFront = numpy.load('data/sinValuesFront.npy')
#print(back_data)
#print(front_data)

matplotlib.pyplot.plot(sin_valuesBack, label='Back Leg', linewidth =3)
matplotlib.pyplot.plot(sin_valuesFront, label='Front Leg')

#matplotlib.pyplot.plot(back_data, label='Back Leg', linewidth=3)
#matplotlib.pyplot.plot(front_data, label='Front Leg')

matplotlib.pyplot.legend(loc=1)
matplotlib.pyplot.show()