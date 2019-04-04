#!/usr/bin/python3

from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
	from PyQt5.QtCore import QLineF, QPointF
elif PYQT_VER == 'PYQT4':
	from PyQt4.QtCore import QLineF, QPointF
else:
	raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))




import time
import numpy as np
from TSPClasses import *
import heapq
import itertools
import copy
from City import My_City



class TSPSolver:
	def __init__( self, gui_view ):
		self._scenario = None

	def setupWithScenario( self, scenario ):
		self._scenario = scenario


	''' <summary>
		This is the entry point for the default solver
		which just finds a valid random tour.  Note this could be used to find your
		initial BSSF.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of solution, 
		time spent to find solution, number of permutations tried during search, the 
		solution found, and three null values for fields not used for this 
		algorithm</returns> 
	'''
	
	def defaultRandomTour( self, time_allowance=60.0 ):
		results = {}
		cities = self._scenario.getCities()
		ncities = len(cities)
		foundTour = False
		count = 0
		bssf = None
		start_time = time.time()
		while not foundTour and time.time()-start_time < time_allowance:
			# create a random permutation
			perm = np.random.permutation( ncities )
			route = []
			# Now build the route using the random permutation
			for i in range( ncities ):
				route.append( cities[ perm[i] ] )
			bssf = TSPSolution(route)
			count += 1
			if bssf.cost < np.inf:
				# Found a valid route
				foundTour = True
		end_time = time.time()
		results['cost'] = bssf.cost if foundTour else math.inf
		results['time'] = end_time - start_time
		results['count'] = count
		results['soln'] = bssf
		results['max'] = None
		results['total'] = None
		results['pruned'] = None
		return results


	''' <summary>
		This is the entry point for the greedy solver, which you must implement for 
		the group project (but it is probably a good idea to just do it for the branch-and
		bound project as a way to get your feet wet).  Note this could be used to find your
		initial BSSF.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number of solutions found, the best
		solution found, and three null values for fields not used for this 
		algorithm</returns> 
	'''

	def greedy( self,time_allowance=60.0 ):
		pass
	
	
	
	''' <summary>
		This is the entry point for the branch-and-bound algorithm that you will implement
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number solutions found during search (does
		not include the initial BSSF), the best solution found, and three more ints: 
		max queue size, total number of states created, and number of pruned states.</returns> 
	'''
		
	def branchAndBound(self, time_allowance=60.0 ):
		self.cities = self._scenario.getCities()
		# 5 seed 1 = 6993
		# 5 seed 20 = 5167
		# 7 seed 20 = 7060
		# 8 seed 20 = 7255

		matrix = []
		for i in range(len(self.cities)):
			matrix.append([])
			for j in range(len(self.cities)):
				matrix[i].append(self.cities[i].costTo(self.cities[j]))
		temp_matrix, reduce_cost = self.reduce(matrix)
		matrix = copy.deepcopy(temp_matrix)

		start_time = time.time()
		for i in range(len(matrix)):
			matrix[0][i] == float('inf')
		bssf = self.init_bsf(self.cities, matrix)
		start_city = My_City()
		start_city.init([0], matrix, 0, self.cities[0], reduce_cost)
		Q = [start_city]
		self.start_loc = 0
		self.bssf_update_counter = 0
		self.created_nodes = 0
		self.pruned_nodes = 0

		while not Q == []:
			curr_city = heapq.heappop(Q)
			if bssf < curr_city:
				self.pruned_nodes = self.pruned_nodes + 1
			else:
				for i in range(len(curr_city.matrix[curr_city.loc])):
					city = self.expand(curr_city, i)
					self.created_nodes = self.created_nodes + 1
					if city <= bssf and city.cost != float('inf'):
						heapq.heappush(Q, city)
						if len(city.path) == len(self.cities):
							start_path = True
							if city.matrix[city.loc][self.start_loc] == float('inf'):
								start_path = False
							if start_path:
								bssf = city
								self.bssf_update_counter = self.bssf_update_counter + 1
					else:
						self.pruned_nodes = self.pruned_nodes + 1

		solution_cities = []
		for i in range(len(bssf.path)):
			solution_cities.append(self.cities[bssf.path[i]])

		end_time = time.time()
		results = {}
		results['cost'] = bssf.cost # + self.cities[bssf.path[-1]].costTo(self.cities[0]) if bssf.cost < float('inf') else math.inf
		results['time'] = end_time - start_time
		results['count'] = self.bssf_update_counter
		results['soln'] = TSPSolution(solution_cities)
		results['max'] = None
		results['total'] = self.created_nodes
		results['pruned'] = self.pruned_nodes
		return results

	def init_bsf(self, cities, matrix):
		city = My_City()
		results = self.defaultRandomTour()
		city.init([0], matrix, 0, cities[0], results['cost'])
		return city

	def reduce(self, matrix):
		reduce_cost = 0
		for i in range(len(matrix)):
			mini = float('inf')
			has_zero = False
			for j in range(len(matrix[0])):
				if matrix[i][j] == 0:
					has_zero == True
				if matrix[i][j] < mini:
					mini = copy.deepcopy(matrix[i][j])

			if not has_zero:
				for j in range(len(matrix[0])):
					if not matrix[i][j] == float('inf'):
						matrix[i][j] = (matrix[i][j] - mini)
				if not mini == float('inf'):
					reduce_cost = reduce_cost + mini

		for i in range(len(matrix)):
			has_zero = False
			mini = float('inf')
			for j in range(len(matrix)):
				if matrix[j][i] == 0:
					has_zero = True
				if matrix[j][i] < mini:
					mini = copy.deepcopy(matrix[j][i])

			if not has_zero:
				for j in range(len(matrix)):
					if not matrix[j][i] == float('inf'):
						matrix[j][i] = copy.deepcopy(matrix[j][i]) - mini
				if not mini == float('inf'):
					reduce_cost = reduce_cost + mini
		return matrix, reduce_cost



	def expand(self, city_from, next_city_loc):
		city = My_City()
		prev_path = copy.deepcopy(city_from.path)
		prev_path.append(next_city_loc)
		path = prev_path
		next_matrix = copy.deepcopy(city_from.matrix)
		tempCost = city_from.matrix[city_from.loc][next_city_loc] + city_from.cost
		next_matrix = self.add_inf(next_matrix, city_from.loc, next_city_loc, path)
		next_matrix, reduce_cost = self.reduce(next_matrix)
		cost = tempCost + reduce_cost

		city.init(path, next_matrix, next_city_loc, self.cities[next_city_loc], cost)
		return city


	def add_inf(self, matrix, city_from, city_to, city_path):
		for col in range(len(matrix[0])):
			matrix[city_from][col] = float('inf')
		for row in range(len(matrix)):
			matrix[row][city_to] = float('inf')
		matrix[city_to][city_from] = float('inf')
		if len(city_path) < len(self.cities):
			matrix[city_to][self.start_loc] = float('inf')
		return matrix



	''' <summary>
		This is the entry point for the algorithm you'll write for your group project.
		</summary>
		<returns>results dictionary for GUI that contains three ints: cost of best solution, 
		time spent to find best solution, total number of solutions found during search, the 
		best solution found.  You may use the other three field however you like.
		algorithm</returns> 
	'''
		
	def fancy( self,time_allowance=60.0 ):
		pass
		



