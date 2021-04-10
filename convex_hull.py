from which_pyqt import PYQT_VER

if PYQT_VER == 'PYQT5':
    from PyQt5.QtCore import QLineF, QPointF, QObject
elif PYQT_VER == 'PYQT4':
    from PyQt4.QtCore import QLineF, QPointF, QObject
else:
    raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))

import time

# Some global color constants that might be useful
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Global variable that controls the speed of the recursion automation, in seconds
#
PAUSE = 0.25


def slope(point1, point2):
    return (point2._y - point1._y) / (point2._x- point1._x)


#
# This is the class you have to complete.
#
class ConvexHullSolver(QObject):

    # Class constructor
    def __init__(self):
        super().__init__()
        self.pause = False

    def showTangent(self, line, color):
        self.view.addLines(line, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseTangent(self, line):
        self.view.clearLines(line)

    def blinkTangent(self, line, color):
        self.showTangent(line, color)
        self.eraseTangent(line)

    def showHull(self, polygon, color):
        self.view.addLines(polygon, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseHull(self, polygon):
        self.view.clearLines(polygon)

    def showText(self, text):
        self.view.displayStatusText(text)

    # Time complexity here is O(n log n) - space complexity is O(n). It would be 4n,
    # since I'm using the points and putting them into two other data structures. But by max rule, it's only O(n)
    def sortCounterClock(self, points):
        # Space O(n) - create dict
        dictSlopes = dict()

        # Time O(n) - iterate over dict
        for i in reversed(range(0, len(points) - 1)):
            dictSlopes.update({slope(points[len(points) - 1], points[i]): points[i]})

        # Space O(n) - create list
        listPoints = [points[len(points) - 1]]

        # Time - O(n log n) - sorting dictionary using Python's Timsort + O(n) for iterating over the dict
        # Space - O(n) - sorted(dictSlopes) creates new list from keys
        for i in sorted(dictSlopes):
            listPoints.append(dictSlopes[i])
        return listPoints

    # Time complexity here is O(n log n) - space complexity is O(n). It would be 4n,
    # since I'm using the points, and three other data structures are created. But by max rule, it's only O(n)
    def sortClock(self, points):
        # Space O(n) - create dict
        dictSlopes = dict()

        # Time O(n) - iterate over dict
        for i in range(1, len(points)):
            dictSlopes.update({slope(points[0], points[i]): points[i]})

        # Space O(n) - create list
        listPoints = [points[0]]

        # Time O(n log n) - sorting dictionary using Python's Timsort + O(n) for iterating over the dict
        # Space - O(n) - sorted(dictSlopes) creates new list from keys
        for i in sorted(dictSlopes):
            listPoints.append(dictSlopes[i])
        return listPoints

    # Time - O(n) at worst and space - O(1)
    # Only making space for a few variables and my while loop will at worst go through all of the points in the hulls
    def findTopTangent(self, leftHull, rightHull):
        yes = 1
        indexLeftTop = 0
        indexRightTop = 0
        while yes:
            curSlope = slope(leftHull[indexLeftTop], rightHull[indexRightTop])
            if slope(leftHull[indexLeftTop], rightHull[(indexRightTop - 1) % len(rightHull)]) > curSlope:
                indexRightTop = (indexRightTop - 1) % len(rightHull)
                continue
            elif slope(leftHull[(indexLeftTop + 1) % len(leftHull)], rightHull[indexRightTop]) < curSlope:
                indexLeftTop = (indexLeftTop + 1) % len(leftHull)
                continue
            else:
                break
        return indexLeftTop, indexRightTop

    # Time - O(n) at worst and space - O(1)
    # Only making space for a few variables and my while loop will at worst go through all of the points in the hull
    def findBottomTangent(self, leftHull, rightHull):
        yes = 1
        indexLeftBottom = 0
        indexRightBottom = 0
        while yes:
            curSlope = slope(leftHull[indexLeftBottom], rightHull[indexRightBottom])
            if slope(leftHull[indexLeftBottom], rightHull[(indexRightBottom + 1) % len(rightHull)]) < curSlope:
                indexRightBottom = (indexRightBottom + 1) % len(rightHull)
                continue
            elif slope(leftHull[(indexLeftBottom - 1) % len(leftHull)], rightHull[indexRightBottom]) > curSlope:
                indexLeftBottom = (indexLeftBottom - 1) % len(leftHull)
                continue
            else:
                break
        return indexLeftBottom, indexRightBottom

    # Overall time - O(n log n)
    # Overall space - O(n log n)
    #
    # from functools import cache
    # @cache
    def hull(self, points):
        if len(points) < 4:
            return points
        # Space complexity here is O(log n) due to the recursion depth
        leftHull = self.hull(points[:len(points) // 2])
        rightHull = self.hull(points[len(points) // 2:])

        # Time complexity for these two here is O(n log n) - space complexity is O(n).
        rightHull = self.sortClock(rightHull)
        leftHull = self.sortCounterClock(leftHull)

        # Find top tangent - Time O(n) space O(1)
        indexLeftTop, indexRightTop = self.findTopTangent(leftHull, rightHull)

        # Find bottom tangent - Time O(n) space O(1)
        indexLeftBottom, indexRightBottom = self.findBottomTangent(leftHull, rightHull)

        # Now connect the hulls
        # Time - O(n) for both loops
        tempPoint = indexLeftTop
        newHull = []
        while tempPoint != indexLeftBottom:
            newHull.append(leftHull[tempPoint])
            tempPoint = (tempPoint + 1) % len(leftHull)
        newHull.append(leftHull[tempPoint])

        tempPoint = indexRightBottom
        while tempPoint != indexRightTop:
            newHull.append(rightHull[tempPoint])
            tempPoint = (tempPoint + 1) % len(rightHull)
        newHull.append(rightHull[tempPoint])
        # Time - O(n log n) - Timsort, space O(n) - new list was created and returned
        return sorted(newHull, key=lambda point: point._x)

    # This is the method that gets called by the GUI and actually executes
    # the finding of the hull
    def compute_hull(self, points):
        # self.pause = pause
        # self.view = view
        assert (type(points) == list and type(points[0]._qPoint) == QPointF)

        #  Time complexity of Timsort O(n log n) in worst case, O(1) best case
        #  Space complexity O(n) worst case, O(1) best case
        points.sort(key=lambda point: point._x)

        t3 = time.time()
        points = self.hull(points)  # Change to self.hull(points)?
        points = self.sortClock(points)
        return points
        # t4 = time.time()
        # polygon = [QLineF(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]
        # return polygon
        #
        # # when passing lines to the display, pass a list of QLineF objects.  Each QLineF
        # # object can be created with two QPointF objects corresponding to the endpoints
        # self.showHull(polygon, BLUE)
        # self.showText('Time Elapsed (Convex Hull): {:3.5f} sec'.format(t4 - t3))
