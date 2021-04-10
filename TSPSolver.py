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
from random import randint
from convex_hull import *
from math import inf
from TSPClasses import *


class TSPSolver:
    def __init__(self, gui_view):
        self._scenario = None

    def setupWithScenario(self, scenario):
        self._scenario = scenario

    ''' <summary>
        This is the entry point for the default solver
        which just finds a valid random tour. Note this could be used to find your
        initial BSSF.
        </summary>
        <returns>results dictionary for GUI that contains three ints: cost of solution, 
        time spent to find solution, number of permutations tried during search, the 
        solution found, and three null values for fields not used for this 
        algorithm</returns> 
    '''

    def defaultRandomTour(self, time_allowance=60.0):
        results = {}
        cities = self._scenario.getCities()
        ncities = len(cities)
        foundTour = False
        count = 0
        bssf = None
        start_time = time.time()
        while not foundTour and time.time() - start_time < time_allowance:
            # create a random permutation
            perm = np.random.permutation(ncities)
            route = []
            # Now build the route using the random permutation
            for i in range(ncities):
                route.append(cities[perm[i]])
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

    # This determines my initial lower bound
    # At worst the TIME COMPLEXITY is O(n^2) - I check each city's n edges (at most) n times
    # Space complexity is O(n) n cities in an n-sized list
    def greedy(self, time_allowance=60.0):
        results = {}
        cities = self._scenario.getCities()
        ncities = len(cities)
        foundTour = False
        count = 0
        bssf = None

        rand = randint(0, ncities - 1)
        currentCity = cities[rand]
        route = [currentCity]
        start_time = time.time()
        while time.time() - start_time < time_allowance:
            tempCity = None
            leastDist = math.inf
            for i in range(ncities):
                if cities[i] not in route:  # Check all cities not in the path
                    if currentCity.costTo(cities[i]) < leastDist:  # if there is a path, take shortest
                        leastDist = currentCity.costTo(cities[i])
                        tempCity = cities[i]
            if tempCity is not None:
                currentCity = tempCity  # Move to next city and append it to path
                route.append(tempCity)
            else:
                route.clear()  # This means no path - start with another city
                currentCity = cities[randint(0, ncities - 1)]
                route.append(currentCity)

            if len(route) == ncities:  # O(n) Space for route
                bssf = TSPSolution(route)
                if bssf.cost < np.inf:  # Path found - initial lower bound
                    foundTour = True
                    break
                else:
                    route.clear()  # No path found - no
                    currentCity = cities[randint(0, ncities - 1)]
                    route.append(currentCity)

        end_time = time.time()
        results['cost'] = bssf.cost if foundTour else math.inf
        results['time'] = end_time - start_time
        results['count'] = count
        results['soln'] = bssf
        results['max'] = None
        results['total'] = None
        results['pruned'] = None

        return results

    # We're going to combine Convex Hull and insertion sort
    def fancy(self, time_allowance=60.0):
        results = {}
        cities = self._scenario.getCities()
        # ncities = len(cities)
        convexHull = ConvexHullSolver()
        perimeter = convexHull.compute_hull(cities)  # This will return the cities that make up the perimeter
        ogIndex = 0
        # ogIndex = randint(0, len(perimeter) - 1)
        path = [perimeter[0]]
        bestConnectingPerimeter = []
        currentCityIndex = 0
        foundTour = False
        lengthBestPerimeter = len(bestConnectingPerimeter)
        nextCityIndex = 1
        start_time = time.time()

        numTries = 15
        numPointsTested = len(perimeter)

        # This will find the best tour given the constraints.
        # numPointsTested makes sure that it tests starting from each point
        while not foundTour and numPointsTested > 0:
            # try:
            if nextCityIndex < len(perimeter) - 1 and perimeter[nextCityIndex] != path[0]:  # Out of bounds check
                if perimeter[currentCityIndex].costTo(perimeter[nextCityIndex]) < inf:  # if path, add
                    # it then move to next
                    path.append(perimeter[nextCityIndex])
                    currentCityIndex = nextCityIndex
                nextCityIndex += 1
            elif perimeter[nextCityIndex] == path[0]:  # Check for full path
                outerTSPSol = TSPSolution(path)
                if outerTSPSol.cost < inf and len(path) > lengthBestPerimeter:  # Check for best full path
                    lengthBestPerimeter = len(path)
                    bestConnectingPerimeter = path.copy()
                    foundTour = True
                if outerTSPSol.cost == inf:
                    # This makes sure that at the end, we have at least some connected path - it's not
                    # always great, but I'd say >85% of the time, it's pretty dang good
                    while len(path) > 2 and not foundTour:
                        path.pop()
                        outerTSPSol = TSPSolution(path)
                        if outerTSPSol.cost < inf:
                            lengthBestPerimeter = len(path)
                            bestConnectingPerimeter = path.copy()
                            foundTour = True
                numPointsTested -= 1
                path.clear()
                ogIndex += 1
                currentCityIndex = ogIndex % len(perimeter)
                path.append(perimeter[currentCityIndex])
                nextCityIndex = (ogIndex + 1) % len(perimeter)

            elif nextCityIndex == len(perimeter) - 1:
                if perimeter[currentCityIndex].costTo(perimeter[0]) < inf:
                    path.append(perimeter[nextCityIndex])
                    currentCityIndex = len(perimeter) - 1
                nextCityIndex = 0
        # except IndexError:
        #     print("Next City Index: ", nextCityIndex)
        #     print("Length perimeter: ", len(perimeter))
        #     print("Path[0]: ", path[0])

        count = 0
        bssf = TSPSolution(bestConnectingPerimeter)

        end_time = time.time()
        results['cost'] = bssf.cost if foundTour else math.inf
        results['time'] = end_time - start_time
        results['count'] = count
        results['soln'] = bssf
        results['max'] = None
        results['total'] = None
        results['pruned'] = None
        return results
