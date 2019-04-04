#!/usr/bin/python3

from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
	from PyQt5.QtCore import QLineF, QPointF
elif PYQT_VER == 'PYQT4':
	from PyQt4.QtCore import QLineF, QPointF
else:
	raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

class My_City:
    cost = float('inf')

    def init(self, path, matrix, loc, city, cost):
        self.path = path
        self.matrix = matrix
        self.loc = loc
        self.city = city
        self.cost = cost


    def __lt__(self, city):
        # if len(self.path) > len(city.path):
        #     return self.cost
        return self.cost < city.cost

    def __le__(self, city):
        return self.cost <= city.cost

    def __gt__(self, city):
        return self.cost > city.cost

    def __ge__(self, city):
        return self.cost >= city.cost

