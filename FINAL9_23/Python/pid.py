import serial
import threading
import time
import keyboard
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

lat0 = 25.011029
lng0 = 121.539077

path = []
lookahead_dist = 2.0
wheelbase = 1.0

# PID 參數
Kp = 0.3
Ki = 0.0
Kd = 0.0
target_speed = 30.0
integral = 0.0
last_error = 0.0

# "gps,1,2,campass,4,hall,6,7,8,9,SR-04,11,12,13,14,15"
sensor_data = ""
action_down = True

w_pressed = False
s_pressed = False

def getSensorData():
    global sensor_data

    if ser_readA is None:
        print("初始化序列埠 A 錯誤")
        return

    while True:
        try:
            if ser_readA.in_waiting > 0:
                sensor_data = ser_readA.readline().decode('utf-8').strip()

        except Exception as e:
            print("讀取序列埠 A 數據錯誤: ", e)
            time.sleep(1)
        time.sleep(0.01)

def getDownSignal():
    if ser_readB is None:
        print("初始化序列埠 B 錯誤")
        return False
    
    if ser_readB.in_waiting > 0:
        line = ser_readB.readline().decode('utf-8').strip()
        if line == "motor action down":
            return True
        else:
            return False

def send_command(cmd):
    if ser_readB is None:
        print("序列埠 B 未初始化")
        return
    try:
        ser_readB.write((cmd + "\n").encode('utf-8'))
    except Exception as e:
        print("序列埠 B 傳送錯誤: ", e)

def latlngToXY(lat, lng, lat0, lng0):
    deg_to_rad = math.pi / 180.0
    dlat = (lat - lat0) * deg_to_rad
    dlng = (lng - lng0) * deg_to_rad

    r = 6378137.0
    x = dlng * r * math.cos(lat0 * deg_to_rad)
    y = dlat * r

    return x, y

def processData(data):
    d = data.split(",")
    if len(d) < 16:
        return None
    
    data_processed = {}

    x, y = latlngToXY(int(d[1]), int(d[2]), lat0, lng0)
    data_processed[d[0]] = {
        "lat": float(d[1]), 
        "lng": float(d[2]), 
        "x": x, 
        "y": y
    }
    data_processed[d[3]] = {
        "angle": float(d[4]), 
    }
    data_processed[d[5]] = {
        "rps_ave": float(d[6]),
        "rpsa": float(d[7]), 
        "rpsb": float(d[8]), 
        "rpsc": float(d[9]) 
    }
    data_processed[d[10]] = {
        "1": float(d[11]), 
        "2": float(d[12]), 
        "3": float(d[13]), 
        "4": float(d[14]), 
        "5": float(d[15])
    }
    
    return data_processed

def pidControl(target_speed, data_processed):
    global integral, last_error

    error = target_speed - data_processed["hall"]["rps_ave"]
    integral += error * 0.5
    derivative = (error - last_error) / 0.5
    pid_output = Kp * error + Ki * integral + Kd * derivative
    last_error = error

    return pid_output

def find_lookahead_point(path, position, lookahead_dist, last_index=0):
    """
    path: Nx2 array of waypoints [[x,y],...]
    position: (x,y)
    lookahead_dist: desired lookahead distance Ld
    last_index: start searching from this index (to ensure forward progress)
    returns: (index, point) where point is (x,y). If not found, returns last point.
    """
    px, py = position

    N = len(path)
    # search for the first path point whose distance along path from current pos >= lookahead_dist
    for i in range(last_index, N):
        dx = path[i,0] - px
        dy = path[i,1] - py
        if math.hypot(dx,dy) >= lookahead_dist:
            return i, (path[i, 0], path[i, 1])
        
    # fallback: return final point
    return N-1, (path[-1, 0], path[-1, 1])

def purePursuit(position, yaw, lookahead_point, lookahead_dist, wheelbase):
    """
    position: (x,y)
    yaw: heading angle (rad)
    lookahead_point: (x_ld, y_ld)
    returns: steering_angle (rad), alpha (angle to lookahead relative to heading)
    """
    px, py = position
    lx, ly = lookahead_point

    # transform lookahead point to vehicle coordinates
    dx = lx - px
    dy = ly - py

    # angle from heading to lookahead point
    local_x =  math.cos(-yaw) * dx - math.sin(-yaw) * dy  # rotate coordinates by -yaw
    local_y =  math.sin(-yaw) * dx + math.cos(-yaw) * dy

    # alpha is angle between heading and vector to lookahead
    alpha = math.atan2(local_y, local_x)

    # curvature kappa = 2*sin(alpha)/Ld
    if lookahead_dist == 0:
        return 0.0, alpha
    
    curvature = 2.0 * math.sin(alpha) / lookahead_dist

    # steering angle (bicycle): delta = atan(L * kappa)
    delta = math.atan(wheelbase * curvature)

    return delta, alpha


# --- 繪圖資料 ---
time_window = 50   # 最多保留多少筆數據
times = deque(maxlen=time_window)
rps_values = deque(maxlen=time_window)

counter = 0  # 🔥 用來當 x 軸計數器

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')
ax.set_ylim(0, 75)   # 根據你的轉速範圍調整
ax.set_xlim(0, time_window)
ax.set_xlabel("Samples")
ax.set_ylabel("RPS (轉速)")
ax.set_title("即時 RPS 圖表")

def update_plot(rps):
    global counter
    counter += 1   # 🔥 每次呼叫就+1
    times.append(counter)
    rps_values.append(rps)

    line.set_xdata(times)
    line.set_ydata(rps_values)

    # 🔥 自動滾動視窗
    ax.set_xlim(counter - time_window, counter)
    ax.figure.canvas.draw()
    ax.figure.canvas.flush_events()

if __name__ == "__main__":
    try:
        ser_readA = serial.Serial('COM3', 9600, timeout=1)  # sensor
        ser_readB = serial.Serial('COM4', 9600, timeout=1)  # motor
    except Exception as e:
        print("初始化序列埠錯誤: ", e)
        ser_readA, ser_readB, ser_readB = None, None, None

    threading.Thread(target=getSensorData, daemon=True).start()

    last_index = 0

    while True:
        if keyboard.is_pressed('w'):
            if w_pressed == False:
                w_pressed = True
                target_speed += 5
                print("========= target speed: ", target_speed)

        elif keyboard.is_pressed('e'):
            if s_pressed == False:
                s_pressed = True
                target_speed -= 5
                print("========= target speed: ", target_speed)

        elif keyboard.is_pressed('q'):
            send_command("pid," + str(int(-200)) + ",ec," + str(int(0)))
            break

        else:
            w_pressed = False
            s_pressed = False

        if action_down:
            data_processed = processData(sensor_data)

            if data_processed == None:
                print("no data")
                continue

            if data_processed:
                if data_processed["hall"]["rps_ave"] > 100:
                    print("忽略異常 RPS:", data_processed["hall"]["rps_ave"])
                    continue
                
                pid_output = pidControl(target_speed, data_processed)

                # last_index, (tx, ty) = find_lookahead_point(
                #     path=path, 
                #     position=(data_processed["gps"]["x"], data_processed["gps"]["x"]), 
                #     lookahead_dist=lookahead_dist, 
                #     last_index=last_index
                # )

                # ec_output = purePursuit(
                #     position=(data_processed["gps"]["x"], data_processed["gps"]["x"]), 
                #     lookahead_point=(tx, ty), 
                #     lookahead_dist=lookahead_dist, 
                #     wheelbase=wheelbase
                # )

                # delta, alpha = ec_output

                delta = 0
                
                print(sensor_data)
                print(f"PID 輸出: {pid_output:.2f}")
                send_command("pid," + str(int(pid_output)) + ",ec," + str(int(delta)))
                update_plot(data_processed["hall"]["rps_ave"])
                action_down = False
        else:
            if getDownSignal() == True:
                print("Get Down => next step")
                action_down = True
        
        time.sleep(0.05)