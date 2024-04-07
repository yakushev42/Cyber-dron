from typing import List

import piosdk.piosdk
from  Point import  Point
from piosdk.piosdk import Pioneer
import time


def intermediate_points(point_a: Point, point_b: Point, step_size:float = 1) -> List[Point]:
    """
    Функция для расчёта промежуточных точек, чтобы траектория была более ровной
    Из точки А в точку Б, step-size - расстояние между шагами
    Возвращает список точек, включая А и Б
    """
    # Разность координат между точками
    dx = point_b.x - point_a.x
    dy = point_b.y - point_a.y

    # Определяем длину отрезка между точками
    length = ((dx ** 2) + (dy ** 2)) ** 0.5

    # Вычисляем количество шагов
    steps = int(length / step_size)

    if steps == 0:
        steps = 1

    # Шаг изменения координаты
    step_x = dx / steps
    step_y = dy / steps

    # Генерация промежуточных точек
    intermediate = []
    # intermediate.append(point_a)

    for i in range(steps):
        x = point_a.x + i * step_x
        y = point_a.y + i * step_y
        intermediate.append((Point(round(x, 2), round(y, 2), point_b.z)))
    intermediate.append(point_b)

    return intermediate

class Drone:
    def __init__(self, ip: str, port: int):
        self.m_drone = Pioneer(ip=ip, mavlink_port=port)
        self.m_np = []
        self.m_sp = 0
        self.m_homePoint = Point(0,0,0)
        self.m_eshelon = 2
        self.m_name = 'drone'
        self.m_route = []
        self.m_currentPointId = 0

    def setEshelon(self, height: float):
        self.m_eshelon = height

    def setHome(self, home:Point):
        self.m_homePoint = home

    def setName(self, name:str):
        self.m_name = name
    def createRoute(self):
        self.m_route.append(self.m_homePoint)
        for i in range(len(self.m_np)):
            self.m_route.append(self.m_sp)
            self.m_route.append(self.m_np[i])
    def checkFire(self) -> bool:
        """
        Проверка на наличие огня
        True - False
        """
        temp = self.m_drone.get_piro_sensor_data()
        sec = 5
        while sec:
            if (temp is not None) and (temp > 40):
                return True
            time.sleep(1)
            sec = sec - 1
        return False

    def getCurrentPos(self) -> Point:
        """
        Получение текущей позиции дрона
        """
        current = None
        while current is None:
            current = self.m_drone.get_local_position_lps()
        return Point(current[0], current[1], current[1])
    def goToPoint(self, point: Point):
        """
        Лететь в точку
        """
        print("----" + self.m_name + "  " + point.x.__str__() + "," + point.y.__str__())
        self.m_drone.go_to_local_point(point.x, point.y, self.m_eshelon)
        while not self.m_drone.point_reached():
            time.sleep(0.5)
    def goToPointStraight(self, point: Point):
        """
        лететь в точку с промежуточными точками
        """
        # currentPos = self.getCurrentPos()
        # route = intermediate_points(currentPos, point)
        # for step in route:
        #     self.goToPoint(step)
        self.goToPoint(point)

    def addNPPoint(self, point: Point):
        """
        Добавить точку населенного пункта
        """
        self.m_np.append(point)

    def setSSPoint(self, point: Point):
        self.m_sp = point

    def landStraight(self, point: Point):
        self.m_drone.go_to_local_point(point.x, point.y, 0.3)
        while not self.m_drone.point_reached():
            time.sleep(0.5)
        self.m_drone.land()

    def loadCargo(self, point: Point):
        self.landStraight(point)
        test = self.m_drone.disarm()
        # while self.m_drone.get_autopilot_state() is not piosdk.piosdk.Pioneer.AUTOPILOT_STATE[1]:
        #     test2 = self.m_drone.get_autopilot_state()
        #     self.m_drone.disarm()
        print("disarm " + self.m_name + test.__str__())
        time.sleep(5)
        self.m_drone.arm()
        self.m_drone.takeoff()
    def startTask(self):
        self.m_drone.arm()
        self.m_drone.takeoff()
        for npPoint in self.m_np:
            self.goToPointStraight(self.m_sp)
            self.loadCargo()
            self.goToPointStraight(self.m_np[0])
            self.loadCargo()



    m_drone: Pioneer
    m_sp: Point
    m_np = []
    m_route = []
    m_currentPointId:Point

    m_homePoint: Point
    m_eshelon: float
    m_name:str