import numpy as np
import serial
from pyproj import Proj, Transformer

lat0 = 25.0102477
lng0 = 121.5399238

path = path = [
    [
        173.94,
        222.799
    ],
    [
        170.508,
        226.454
    ],
    [
        166.975,
        230.11
    ],
    [
        163.543,
        233.765
    ],
    [
        160.01,
        237.421
    ],
    [
        156.578,
        241.076
    ],
    [
        153.146,
        244.732
    ],
    [
        149.613,
        248.387
    ],
    [
        146.181,
        252.153
    ],
    [
        142.648,
        255.809
    ],
    [
        139.216,
        259.464
    ],
    [
        135.683,
        263.12
    ],
    [
        132.251,
        266.775
    ],
    [
        128.718,
        270.431
    ],
    [
        125.286,
        274.086
    ],
    [
        121.753,
        277.742
    ],
    [
        118.321,
        281.508
    ],
    [
        114.889,
        285.164
    ],
    [
        111.357,
        288.819
    ],
    [
        107.925,
        292.475
    ],
    [
        104.392,
        296.13
    ],
    [
        108.025,
        299.564
    ],
    [
        111.659,
        302.998
    ],
    [
        115.394,
        306.543
    ],
    [
        119.028,
        309.977
    ],
    [
        122.662,
        313.411
    ],
    [
        126.295,
        316.845
    ],
    [
        129.929,
        320.279
    ],
    [
        133.563,
        323.713
    ],
    [
        137.197,
        327.258
    ],
    [
        140.83,
        330.692
    ],
    [
        144.565,
        334.126
    ],
    [
        148.199,
        337.56
    ],
    [
        151.833,
        340.994
    ],
    [
        148.199,
        344.76
    ],
    [
        144.666,
        348.526
    ],
    [
        141.133,
        352.292
    ],
    [
        137.499,
        356.059
    ],
    [
        133.966,
        359.825
    ],
    [
        130.433,
        363.591
    ],
    [
        126.8,
        367.468
    ]
]
path = np.array(path)

ser_readA = serial.Serial('COM6', 9600, timeout=1)

def serial_arduino():
    global ser_readA
    try:
        if ser_readA.in_waiting > 0:
            serial_data = ser_readA.readline().decode('ascii', errors='ignore').strip().split(",")
            serial_data = [float(i) for i in serial_data]
    except Exception as e:
        print("Error reading serial port A data: ", e)

    return serial_data

def latlngToXY(lat, lng, lat0, lng0):
    # print("Converting lat/lng to x/y:", lat, lng)
    
    proj_wgs84 = Proj("epsg:4326")

    proj_local = Proj(
        proj='aeqd', 
        ellps='WGS84', 
        datum='WGS84', 
        lat_0=lat0, 
        lon_0=lng0
    )
    
    transformer = Transformer.from_proj(proj_wgs84, proj_local)
    x_coords, y_coords = transformer.transform(lat, lng)

    # print("x_coords:", x_coords, "y_coords:", y_coords)

    return round(x_coords, 10), round(y_coords, 10)

def purePursuit(position, yaw, target_path, lookahead_dist, wheelbase):
    """
    Pure Pursuit 控制器
    :param state: 當前車輛狀態 (VehicleState 物件)
    :param target_path: 目標軌跡點陣列
    :param lookahead_dist: 前視距離
    :return: 所需的前輪轉角 (rad)
    """

    # 1. 尋找最近點 (Find Nearest Point)
    # 計算車輛當前位置與所有軌跡點的距離
    dx = target_path[:, 0] - position[0]
    dy = target_path[:, 1] - position[1]
    distances = np.sqrt(dx**2 + dy**2)
    
    # 找到最近點的索引
    nearest_idx = np.argmin(distances)
    
    # 2. 尋找目標點 (Find Goal Point)
    target_idx = -1
    for i in range(nearest_idx, len(target_path)):
        if distances[i] > lookahead_dist:
            target_idx = i
            break
            
    # 如果找不到點 (已到達軌跡末端)，則選最後一個點作為目標
    if target_idx == -1:
        target_idx = len(target_path) - 1
        
    # 目標點坐標
    target_point = target_path[target_idx]
    
    # 3. 計算幾何關係
    
    # 將目標點從世界坐標轉換到車輛坐標系下
    # (dx_car, dy_car) 是目標點相對於車輛的坐標
    dx_car = target_point[0] - position[0]
    dy_car = target_point[1] - position[1]
    
    # 計算目標點在車輛坐標系下的角度 (alpha)
    # alpha 是目標點與車輛縱軸 (車頭方向) 之間的夾角
    # 公式：atan2(Y_target_car, X_target_car)
    # 這裡需要將世界坐標下的 (dx, dy) 旋轉 -yaw 
    
    # 旋轉矩陣: [ cos(-yaw)  -sin(-yaw) ]   [ cos(yaw)  sin(yaw) ]
    #           [ sin(-yaw)   cos(-yaw) ] = [ -sin(yaw) cos(yaw) ]
    
    rotated_x = dx_car * np.cos(-yaw) - dy_car * np.sin(-yaw)
    rotated_y = dx_car * np.sin(-yaw) + dy_car * np.cos(-yaw)

    # 最終 alpha (目標點在車輛坐標系下的角度)
    alpha = np.arctan2(rotated_y, rotated_x)

    # 4. Pure Pursuit 核心公式 (輸出前輪轉角 delta)
    # L_d 是幾何上的圓半徑 R
    # sin(alpha) = (L_d / 2) / R  -> R = L_d / (2 * sin(alpha))
    # 轉角公式: tan(delta) = L / R 
    # 結合後: delta = atan2(2 * L * sin(alpha), L_d)
    
    # atan2 可以處理 alpha=0 的情況
    delta = np.arctan2(2.0 * wheelbase * np.sin(alpha), lookahead_dist)
    
    return delta, target_point, nearest_idx

if __name__ == "__main__":
    while True:
        serial_data = serial_arduino()
        if serial_data:
            print(f"serial_arduino: {serial_data}")
            gps_position = (serial_data[0], serial_data[1])
            gps_position_xy = latlngToXY(
                lat=gps_position[0], 
                lng=gps_position[1], 
                lat0=lat0, 
                lng0=lng0
            )
            yaw = serial_data[2]

            delta, target_point, nearest_idx = purePursuit(
                position=gps_position_xy,
                yaw=np.radians(yaw),
                target_path=path,
                lookahead_dist=3.0,
                wheelbase=1.0
            )
            delta_deg = np.degrees(delta)

            print("--------------------------------------------------")
            print("GPS Position:", gps_position, "GPS Position (x, y):", gps_position, "Yaw (deg):", yaw)
            print(f"Pure Pursuit => nearest_idx: {nearest_idx}")
            print(f"                target_point: {target_point}")
            print(f"                delta (deg): {delta_deg}")
            print("--------------------------------------------------")