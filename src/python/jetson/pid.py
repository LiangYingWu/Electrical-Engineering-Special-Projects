import time
import numpy as np
import math

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
    dx = target_path[:, 0] - position[0]
    dy = target_path[:, 1] - position[1]
    distances = np.sqrt(dx**2 + dy**2)
    
    nearest_idx = np.argmin(distances)
    nearest_point = target_path[nearest_idx]
    
    target_idx = -1
    for i in range(nearest_idx, len(target_path)):
        if distances[i] > lookahead_dist:
            target_idx = i
            break
    
    if target_idx == -1:
        target_idx = len(target_path) - 1
    
    target_point = target_path[target_idx]
    
    dx_car = target_point[0] - position[0]
    dy_car = target_point[1] - position[1]
    
    rotated_x = dx_car * np.cos(-yaw) - dy_car * np.sin(-yaw)
    rotated_y = dx_car * np.sin(-yaw) + dy_car * np.cos(-yaw)

    alpha = np.arctan2(rotated_y, rotated_x)

    delta = np.arctan2(2.0 * wheelbase * np.sin(alpha), lookahead_dist)
    
    return delta, alpha, nearest_idx, nearest_point, target_idx, target_point

def carPositionIntegration(prev_xy, yaw, rps, wheel_circumference, dt):
    x_prev, y_prev = prev_xy
    
    v = rps * wheel_circumference

    heading_rad = math.radians(yaw)

    dx = v * math.cos(heading_rad) * dt
    dy = v * math.sin(heading_rad) * dt

    return (x_prev + dx, y_prev + dy)