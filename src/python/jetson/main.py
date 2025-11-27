import time
import keyboard
import math
import numpy as np

from wifi import start_stream, update_json, get_arduino_data
from pid import pidControl, purePursuit
from utils import processData
import serial_utils
from plot_utils import updatePlot

lat0 = 25.0102477
lng0 = 121.5399238
base_station_gps = (25.011933, 121.541187)

target_speed = 30.0
stop_moving = False
stopping_distance = 50.0
no_sensor_data = False

path = [
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

lookahead_dist, wheelbase = 3.0, 1.0

def main():
    global target_speed, stop_moving, stopping_distance, no_sensor_data, path, lookahead_dist, wheelbase

    ffmpeg_proc, sock_json = start_stream()
    serial_utils.init_serial(portA='COM6', portB='COM4', baudrate=9600)

    serial_utils.start_sensor_thread()

    last_index = 0

    b_pressed = [False for _ in range(10)]
    
    mode = input("Semi / Fully Automatic? (0/1): ")
    print("========== Press 0 to leave ==========")
    
    running = True

    while running:
        if serial_utils.action_down:
            for i in range(10):
                if keyboard.is_pressed(str(i)):
                    if not b_pressed[i]:
                        b_pressed[i] = True
                        if i == 0:
                            serial_utils.send_command("pid," + str(-200) + ",ec," + str(0))

                            try:
                                ffmpeg_proc.terminate()
                            except Exception as e:
                                print("FFmpeg terminate error:", e)
                            try:
                                sock_json.close()
                            except Exception as e:
                                print("Socket close error:", e)
                            
                            serial_utils.stop_sensor_thread()

                            running = False
                            
                            break

                        elif i == 1:
                            if mode == "0":
                                serial_utils.send_command("pid," + str(20) + ",ec," + str(0))
                        elif i == 2:
                            if mode == "0":
                                serial_utils.send_command("pid," + str(-20) + ",ec," + str(0))
                        elif i == 3:
                            if mode == "0":
                                serial_utils.send_command("pid," + str(0) + ",ec," + str(10))
                        elif i == 4:
                            if mode == "0":
                                serial_utils.send_command("pid," + str(0) + ",ec," + str(-10))
                        elif i == 8:
                            target_speed += 5
                            print("========== target_speed:", target_speed, "==========")
                        elif i == 9:
                            target_speed -= 5
                            print("========== target_speed:", target_speed, "==========")
                else:
                    b_pressed[i] = False
            
            if not running:
                break

            # Get Arduino Data (GPS Error)
            arduino_data = get_arduino_data()
            if arduino_data is None:
                arduino_data = [base_station_gps[0], base_station_gps[1]]
                print("========== Can not get GPS error message ==========")
            gps_error = [round(float(arduino_data[0]) - base_station_gps[0], 10), 
                         round(float(arduino_data[1]) - base_station_gps[1], 10)]
            # print(gps_error)

            # Get Sensor Data
            data_processed = processData(serial_utils.sensor_data, (lat0, lng0), gps_error)
            
            if data_processed:
                no_sensor_data = False
                update_json(data_processed)

                # Ingnore Abnormal RPS
                if data_processed["hall"]["rps_avg"] > 100:
                    print("========== Ingnore Abnormal RPS: ", data_processed["hall"]["rps_avg"], "==========")
                    continue
                
                updatePlot(data_processed["hall"]["rps_avg"])
                for outer_key, inner_dict in data_processed.items():
                    print("    ", outer_key, end=": \n")
                    for key, value in inner_dict.items():
                        print(f"        {key}: {value}")
                print()
                
                if not stop_moving:
                    # Check Stopping Distance
                    if any(v < stopping_distance for v in data_processed["hr-04"].values()):
                        print("========== Stop Moving ==========")
                        serial_utils.send_command("pid," + str(-200) + ",ec," + str(0))
                        stop_moving = True

                    if stop_moving:
                        continue

                else:
                    if all(value > stopping_distance for value in data_processed["hr-04"].values()):
                        print("========== All Clear, Resume Moving ==========")
                        stop_moving = False
                    else:
                        continue

                if not stop_moving:
                    if mode == "1":
                        # PID Control
                        pid_output = pidControl(target_speed=target_speed, current_speed=data_processed["hall"]["rps_avg"])

                        # Pure Pursuit
                        # last_index, (tx, ty) = findLookaheadPoint(
                        #     path=path, 
                        #     position=(data_processed["gps"]["x"], data_processed["gps"]["y"]), 
                        #     lookahead_dist=lookahead_dist, 
                        #     last_index=last_index
                        # )
                        purePursuit_output = purePursuit(
                            position=(data_processed["gps"]["x"], data_processed["gps"]["y"]), 
                            yaw=math.radians(data_processed["campass"]["degree"]),
                            # lookahead_point=(tx, ty), 
                            lookahead_dist=lookahead_dist, 
                            wheelbase=wheelbase
                        )
                        delta, alpha, nearest_idx, nearest_point, target_idx, target_point = purePursuit_output
                        delta_deg = math.degrees(delta)
                        alpha_deg = math.degrees(alpha)

                        print(f"Nearest Point: {nearest_idx}, {nearest_point}")
                        print(f"Target Point: {target_idx}, {target_point}")
                        print(f"Delta (deg): {delta_deg:.2f}, Alpha (deg): {alpha_deg:.2f}")
                        print(f"Get Command => pid: {int(pid_output)}, ec: {int(delta_deg)}")
                        
                        # pid_output = 0
                        # delta_deg = 0
                        
                        # Send Command to Arduino
                        serial_utils.send_command("pid," + str(int(pid_output)) + ",ec," + str(int(delta_deg)))
            
            else:
                print("========== No Sensor Data ==========")
                if not no_sensor_data:
                    serial_utils.send_command("pid," + str(-200) + ",ec," + str(0))
                    print("========== Stop Moving ==========")
                    no_sensor_data = True
                continue
        else:
            while True:
                if serial_utils.getDownSignal() == True:
                    print("========== Get Down Signal => Next Step ==========")
                    serial_utils.action_down = True
                    break
                time.sleep(0.05)
        time.sleep(0.05)
    print("========== ALL STOP ==========")

if __name__ == "__main__":
    main()