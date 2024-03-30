#Импорт библиотек
import dataclasses
import threading
import time

from sklearn.cluster import KMeans  # pip install scikit-learn

from piosdk.piosdk import Pioneer


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

#Иницииализация первого дрона
drone = Pioneer(ip=DroneConnectingData.drone2.ip, mavlink_port=DroneConnectingData.drone2.port)
#Иницииализация Второго дрона
drone1 = Pioneer(ip=DroneConnectingData.drone3.ip, mavlink_port=DroneConnectingData.drone3.port)

#Запуск двигателей дронов
drone.arm()
drone1.arm()

drone.takeoff()
drone.land()
drone.disarm()
