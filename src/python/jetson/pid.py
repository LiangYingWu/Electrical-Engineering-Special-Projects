import time
import numpy as np

# PID 參數
Kp, Ki, Kd = 0.3, 0.0, 0.0
integral = 0.0
last_error = 0.0
last_time = time.time()

def pidControl(target_speed, current_speed):
    global Kp, Ki, Kd, integral, last_error, last_time

    error = target_speed - current_speed
    
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    integral += error * dt
    derivative = (error - last_error) / dt if dt > 0 else 0
    pid_output = Kp * error + Ki * integral + Kd * derivative
    last_error = error

    return pid_output

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
    nearest_point = target_path[nearest_idx]
    
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
    
    return delta, alpha, nearest_idx, nearest_point, target_idx, target_point

# def findLookaheadPoint(path, position, lookahead_dist, last_index=0):
#     """
#     path: Nx2 array of waypoints [[x,y],...]
#     position: (x,y)
#     lookahead_dist: desired lookahead distance Ld
#     last_index: start searching from this index (to ensure forward progress)
#     returns: (index, point) where point is (x,y). If not found, returns last point.
#     """
#     px, py = position

#     N = len(path)
#     # search for the first path point whose distance along path from current pos >= lookahead_dist
#     for i in range(last_index, N):
#         dx = path[i,0] - px
#         dy = path[i,1] - py
#         if math.hypot(dx,dy) >= lookahead_dist:
#             return i, (path[i, 0], path[i, 1])
        
#     # fallback: return final point
#     return N-1, (path[-1, 0], path[-1, 1])

# def purePursuit(position, yaw, lookahead_point, lookahead_dist, wheelbase):
#     """
#     position: (x,y)
#     yaw: heading angle (rad)
#     lookahead_point: (x_ld, y_ld)
#     returns: delta(rad) (steering_angle), alpha (angle to lookahead relative to heading)
#     """
#     px, py = position
#     lx, ly = lookahead_point

#     # transform lookahead point to vehicle coordinates
#     dx = lx - px
#     dy = ly - py

#     # angle from heading to lookahead point
#     local_x =  math.cos(-yaw) * dx - math.sin(-yaw) * dy  # rotate coordinates by -yaw
#     local_y =  math.sin(-yaw) * dx + math.cos(-yaw) * dy

#     # alpha is angle between heading and vector to lookahead
#     alpha = math.atan2(local_y, local_x)

#     # curvature kappa = 2*sin(alpha)/Ld
#     if lookahead_dist == 0:
#         return 0.0, alpha
    
#     curvature = 2.0 * math.sin(alpha) / lookahead_dist

#     # steering angle (bicycle): delta = atan(L * kappa)
#     delta = math.atan(wheelbase * curvature)

#     return delta, alpha