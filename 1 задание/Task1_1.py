import time
import threading
import multiprocessing
from piosdk.piosdk import Pioneer
#from sklearn.cluster import KMeans  # pip install scikit-learn
import dataclasses
from typing import List

# Классы для хранения настроек подключения
@dataclasses.dataclass
class IpPort:
    ip: str
    port: int

# Параметры для подключения дронов. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует этим портам
class DroneConnectingData:
    drone0: IpPort = IpPort(ip="127.0.0.1", port=8000)
    drone1: IpPort = IpPort(ip="127.0.0.1", port=8001)
    drone2: IpPort = IpPort(ip="127.0.0.1", port=8002)
    drone3: IpPort = IpPort(ip="127.0.0.1", port=8003)

# Параметры для подключения РТС. В симуляторе смотрим номер порта и здесь смотрим какой объект класса соотвествует этим портам
class RobotConnectingData:
    robot0: IpPort = IpPort(ip="127.0.0.1", port=8004)
    robot1: IpPort = IpPort(ip="127.0.0.1", port=8005)
    robot2: IpPort = IpPort(ip="127.0.0.1", port=8006)
    robot3: IpPort = IpPort(ip="127.0.0.1", port=8007)


class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z
    x: float = 0
    y: float = 0
    z: float = 0
    fire: bool = False


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
        self.m_parolPoints = []
        self.m_homePoint = Point(0,0,0)
        self.m_eshelon = 2

    def setEshelon(self, height: float):
        self.m_eshelon = height
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
        self.m_drone.go_to_local_point(point.x, point.y, self.m_eshelon)
        while not self.m_drone.point_reached():
            time.sleep(0.5)
    def goToPointStraight(self, point: Point):
        """
        лететь в точку с промежуточными точками
        """
        currentPos = self.getCurrentPos()
        route = intermediate_points(currentPos, point)
        for step in route:
            self.goToPoint(step)

    def addPatrolPoint(self, point: Point):
        """
        Добавить точку патрулирования
        """
        self.m_parolPoints.append(point)
    def startPatrol(self):
        """
        Бесконечное патрулирование точек
        """
        self.m_drone.arm()
        self.m_drone.takeoff()
        while True:
            for point in self.m_parolPoints:

                self.goToPointStraight(point)
                fire = self.checkFire()
                point.fire = fire
            time.sleep(1)

    m_drone: Pioneer
    m_parolPoints: List[Point] = []
    m_homePoint: Point
    m_eshelon: float

drones: List[Drone] = []

drones.append(Drone(ip = DroneConnectingData.drone0.ip, port = DroneConnectingData.drone0.port))
drones.append(Drone(ip = DroneConnectingData.drone1.ip, port = DroneConnectingData.drone1.port))
drones.append(Drone(ip = DroneConnectingData.drone2.ip, port = DroneConnectingData.drone2.port))
drones.append(Drone(ip = DroneConnectingData.drone3.ip, port = DroneConnectingData.drone3.port))

start_position_dron2 = [-1.09, -4.76, 2]
start_position_dron1 = [-0.25, -4.76, 2]
#массив координат полета, где есть пожар
mission_point = [
        Point(-4, -4, 2),  # fire
        Point(-4, -0, 2),
        Point(0, -4, 2),
        Point(-4, 0, 2),
        Point(0, 0, 2),  # fire
        Point(0, 2, 2),
        Point(-4, 4, 2),
        Point(-4, 2, 2),
        Point(-2, 2, 2),
        Point(-3, 3, 2)
    ]


if __name__ == '__main__':
    drones[0].addPatrolPoint(mission_point[0])
    drones[0].addPatrolPoint(mission_point[1])
    drones[0].addPatrolPoint(mission_point[2])
    drones[1].addPatrolPoint(mission_point[3])
    drones[1].addPatrolPoint(mission_point[4])
    drones[2].addPatrolPoint(mission_point[5])
    drones[2].addPatrolPoint(mission_point[6])
    drones[3].addPatrolPoint(mission_point[7])
    drones[3].addPatrolPoint(mission_point[8])
    drones[3].addPatrolPoint(mission_point[9])

    drones[0].setEshelon(0.75)
    drones[1].setEshelon(1.25)
    drones[2].setEshelon(1.75)
    drones[3].setEshelon(2.25)


    # Создаем поток для поиска пожаров и запускаем
    dron0_thread = threading.Thread(target=drones[0].startPatrol)
    dron1_thread = threading.Thread(target=drones[1].startPatrol)
    dron2_thread = threading.Thread(target=drones[2].startPatrol)
    dron3_thread = threading.Thread(target=drones[3].startPatrol)

    dron0_thread.start()
    dron1_thread.start()
    dron2_thread.start()
    dron3_thread.start()

    # dron0_thread.join()
    # dron1_thread.join()
    # dron2_thread.join()
    # dron3_thread.join()


