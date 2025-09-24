import serial
import time
import threading
import tkinter as tk
from tkinter import ttk
obstacle_detected = threading.Event()
# 建立 Serial 物件，參數視你的 Arduino COM 口而定
# Windows 可能是 COM3, COM4, Mac/Linux 是 /dev/ttyUSB0
# 設定兩塊 Arduino 的 COM port
COM_PORTS = {
    'Motor': 'COM3',  # 請改成你實際接 Arduino A 的 Port
    'SR-04': 'COM5',  # Arduino B 的 Port
    'Compass': 'COM6'
}
# 儲存 Serial 物件
arduinos = {}

# 讀取資料的背景執行緒
def read_from_arduino(name, ser):
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            print(f"From{name}: {line}")
        time.sleep(0.1)

# 初始化所有序列連接
for name, port in COM_PORTS.items():
    try:
        ser = serial.Serial(port=port, baudrate=9600, timeout=1)
        time.sleep(2)
        arduinos[name] = ser
        threading.Thread(target=read_from_arduino, args=(name, ser), daemon=True).start()
        print(f"Connect to Arduino {name} in {port} Success ")
    except Exception as e:
        print(f"Connect to Arduino {name} Fail：{e}")

def send_command(ser, cmd):
    ser.write(cmd.encode())
    print(f"Sand: {cmd}")
    timeout = 0
    while timeout < 5:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Arduino FeedBack: {response}")
            if response == "DONE":
                return True
        time.sleep(1)
        timeout += 1
    print("Error: Arduino NO response")
    return False

# 自動模式邏輯
def auto_mode():
    print("Auto Mode")
    

# MANU FUNCTION

try:
    while True:
        print("""
Main menu ：
1. Step mode
2. Auto mode
"Q" to Leave
""")

        cmd = input("select: ")
        if cmd == 'Q':
            break
        elif cmd == '1':
            while True:
                 print("""
Control menu ：
1. Turn Left
2. Turn right
3. Speed Up
4. Speed down
5. Breaking
6. Breakloseing
"Q" to Leave
""")
                 cmd = input("select: ")
                 if cmd in ['1','2','3','4','5','6']:
                     send_command(arduinos['Motor'], cmd)
                 elif cmd == 'Q':
                    break
                 else:
                    print("Please re-enter")
        elif cmd == '2':
            auto_mode()
        else:
            print("Please re-enter")
except KeyboardInterrupt:
    print("Forced exit")
finally:
    print("TURN OFF")
    for ser in arduinos.values():
        ser.close()

          