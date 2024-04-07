import dataclasses
import threading
import time
from typing import List

#from sklearn.cluster import KMeans  # pip install scikit-learn

from piosdk.piosdk import Pioneer

import conflict_based_search

from  Drone import  Drone
from  Point import Point

import threading



# Классы для хранения настроек подключения
@dataclasses.dataclass
class IpPort:
    ip: str
    port: int


class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)
    drone2: IpPort = IpPort(ip="127.0.0.1", port=8002)
    drone3: IpPort = IpPort(ip="127.0.0.1", port=8003)


class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8004)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8005)
    robot2: IpPort = IpPort(ip="127.0.0.1", port=8006)
    robot3: IpPort = IpPort(ip="127.0.0.1", port=8007)




drones: List[Drone] = []

drones.append(Drone(ip = DroneConnectingData.drone0.ip, port = DroneConnectingData.drone0.port))
drones.append(Drone(ip = DroneConnectingData.drone1.ip, port = DroneConnectingData.drone1.port))
drones.append(Drone(ip = DroneConnectingData.drone2.ip, port = DroneConnectingData.drone2.port))
drones.append(Drone(ip = DroneConnectingData.drone3.ip, port = DroneConnectingData.drone3.port))

eshelones = [
    1,
    1.5,
    2,
    2.5
]

#Сортировочные пункты
SP = [
    Point(-1,  1, 2),
    Point( 1,  1, 2),
    Point( 1, -1, 2),
    Point(-1, -1, 2)
]
#Населенные пункты
NP = [
    Point(-3, 3, 2),
    Point( 3, 3, 2),
    Point( 3,-3, 2),
    Point(-3,-3, 2)
]
#Логистические центры
LP = [
    Point(-1, -4, 0),
    Point(1, -4, 0)
]
#Магазины
Shops = [
    Point(0, 4, 0),
    Point(4, 1, 0)
]
# HomePoints = [
#     Point(-4, -4.5, 2),
#     Point(-3, -4.5, 2),
#     Point(-2, -4.5, 2),
#     Point(-1, -4.5, 2)
# ]
HomePoints = [
    Point(-4, -4, 2),
    Point(-3, -4, 2),
    Point(-2, -4, 2),
    Point(-1, -4, 2)
]
RTC_HomePoints =[
    Point(2, -4, 0),
    Point(3, -4, 0)
]


def makeDimension(xSize: int, ySize: int):
    return [xSize, ySize]
def appendTask(taskList, start:Point, stop:Point, name:str):
 taskList.append(dict(start=[start.x, start.y], goal=[stop.x, stop.y], name=name))
def appendObstacles(obstacles, x:int, y:int):
    obstacles.append(tuple(x,y))

def convertPointTo(point:Point):
    return Point(point.x + 5, point.y + 5, point.z)

def convertPointFrom(point:Point):
    return Point(point.x - 5, point.y - 5, point.z)

def flyStep(s: int):
    tasks = []
    treads = []
    for i in range(4):
        appendTask(taskList=tasks,
                   start=convertPointTo(drones[i].m_route[s]),
                   stop=convertPointTo(drones[i].m_route[s + 1]),
                   name=drones[i].m_name)
    env = conflict_based_search.Environment(dimension, tasks, obstacles)
    cbs = conflict_based_search.CBS(env)
    solution = cbs.search()
    steps = max(len(solution[drones[0].m_name]),
                len(solution[drones[1].m_name]),
                len(solution[drones[2].m_name]),
                len(solution[drones[3].m_name]))
    for step in range(steps):
        treads = []
        for i in range(4):
            if len(solution[drones[i].m_name]) > step:
                test = len(solution[drones[i].m_name])
                treads.append(threading.Thread(target=drones[i].goToPointStraight, args=( convertPointFrom(solution[drones[i].m_name][step]),)) )
        for thread in treads:
            thread.start()
        for thread in treads:
            thread.join()
    treads = []
    for i in range(4):
        treads.append(threading.Thread(target=drones[i].loadCargo, args=(drones[i].m_route[s + 1], ) ) )
    for i in range(4):
        treads[i].start()
    for i in range(4):
        treads[i].join()


# def main():
dimension = [11,11]
obstacles = [tuple([0,0])]

[drones[i].setName('agent' + str(i)) for i in range(4)]
[drones[i].setHome(HomePoints[i]) for i in range(4)]
[drones[i].setSSPoint(SP[i]) for i in range(4)]
[drones[i].setEshelon(eshelones[i]) for i in range(4)]
for i in range(4):
    for j in range(4):
        drones[i].addNPPoint(NP[(j + i) % 4])
    treads = []
for i in range(4):
    drones[i].createRoute()
    treads.append(threading.Thread(target=drones[i].m_drone.arm))
for i in range(4):
    treads[i].start()
for i in range(4):
    treads[i].join()

treads = []
for i in range(4):
    drones[i].createRoute()
    treads.append(threading.Thread(target=drones[i].m_drone.takeoff))
for i in range(4):
    treads[i].start()
for i in range(4):
    treads[i].join()

for i in range(len(drones[0].m_route)):
    flyStep(i)

pass

