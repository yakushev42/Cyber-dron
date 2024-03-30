import time
import threading
from piosdk.piosdk import Pioneer
from sklearn.cluster import KMeans  # pip install scikit-learn
import dataclasses


def get_center_clusters(data, num_clasters):
    """ Поиск кластеров в наборе данных """
    kmeans = KMeans(n_clusters=num_clasters, max_iter=100)
    kmeans.fit(data)
    return kmeans.cluster_centers_

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


drones = []
#Иницииализация дронов
drones.append(Pioneer(ip=DroneConnectingData.drone0.ip, mavlink_port=DroneConnectingData.drone0.port))
drones.append(Pioneer(ip=DroneConnectingData.drone1.ip, mavlink_port=DroneConnectingData.drone1.port))
drones.append(Pioneer(ip=DroneConnectingData.drone2.ip, mavlink_port=DroneConnectingData.drone2.port))
drones.append(Pioneer(ip=DroneConnectingData.drone3.ip, mavlink_port=DroneConnectingData.drone3.port))


centre_fire = []  # массив куда запишутся центры пожаров
start_position_dron2 = [-1.09, -4.76, 2]
start_position_dron1 = [-0.25, -4.76, 2]
#массив координат полета, где есть пожар
mission_point = [
        [-3.85, -1.06, 2],  # fire
        [-0.86, -1.92, 2],
        [2.02, -1.29, 2],
        [1.23, 0.20, 2],
        [0.68, 1.50, 2],  # fire
        [-0.64, 2.53, 2],
        [-2.39, 2.95, 2],
        [3.40, 2.04, 2],
        [2.28, 3.60, 2],
        [-4.81, 4.36, 2],
    ]
def dron1_task():
    drones[2].arm()
    drones[2].takeoff()
    drones[2].go_to_local_point(x=mission_point[0][0],
                                y=mission_point[0][1],
                                z=mission_point[0][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=mission_point[1][0],
                                y=mission_point[1][1],
                                z=mission_point[1][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=mission_point[2][0],
                                y=mission_point[2][1],
                                z=mission_point[2][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=mission_point[3][0],
                                y=mission_point[3][1],
                                z=mission_point[3][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=mission_point[4][0],
                                y=mission_point[4][1],
                                z=mission_point[4][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=mission_point[5][0],
                                y=mission_point[5][1],
                                z=mission_point[5][2])
    while not drones[2].point_reached():
        pass
    time.sleep(5)
    drones[2].go_to_local_point(x=start_position_dron1[0],
                                y=start_position_dron1[1],
                                z=start_position_dron1[2])
    while not drones[2].point_reached():
        pass
    drones[2].land()
    drones[2].disarm()


def dron2_task():
    drones[3].arm()
    drones[3].takeoff()
    drones[3].go_to_local_point(x=mission_point[6][0],
                                y=mission_point[6][1],
                                z=mission_point[6][2])
    while not drones[3].point_reached():
        pass
    time.sleep(5)
    drones[3].go_to_local_point(x=mission_point[7][0],
                                y=mission_point[7][1],
                                z=mission_point[7][2])
    while not drones[3].point_reached():
        pass
    time.sleep(5)
    drones[3].go_to_local_point(x=mission_point[8][0],
                                y=mission_point[8][1],
                                z=mission_point[8][2])
    while not drones[3].point_reached():
        pass
    time.sleep(5)
    drones[3].go_to_local_point(x=mission_point[9][0],
                                y=mission_point[9][1],
                                z=mission_point[9][2])
    while not drones[3].point_reached():
        pass
    time.sleep(5)
    drones[3].go_to_local_point(x=start_position_dron2[0],
                                y=start_position_dron2[1],
                                z=start_position_dron2[2])
    while not drones[3].point_reached():
        pass
    drones[3].land()
    drones[3].disarm()

if __name__ == '__main__':
    # Создаем поток для поиска пожаров и запускаем
    dron1_thread = threading.Thread(target=dron1_task)
    dron2_thread = threading.Thread(target=dron2_task)
    dron2_thread.start()
    time.sleep(2)
    dron1_thread.start()


