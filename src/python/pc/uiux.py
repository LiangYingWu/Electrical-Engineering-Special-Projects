import cv2
from ultralytics import YOLO
import requests
import numpy as np
import sys, os, json, copy
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QVBoxLayout, \
    QHBoxLayout, QTextEdit, QLabel, QStackedLayout, QListWidget, QSplitter
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QUrl, pyqtSlot, QObject, QTimer, Qt, QThread, pyqtSignal
from PyQt5.QtWebChannel import QWebChannel

from pyproj import Proj, Transformer

import osmnx as ox
import networkx as nx
import math
import time
import socket

# ---------------------- 與 JS 溝通的橋樑 ----------------------
class Bridge(QObject):
    def __init__(self, routes_file_path="routes.json"):
        super().__init__()
        data = self.load_routes(routes_file_path)
        self.routes = data.get("routes", {}).get("default_route", {})
        self.current_route = data.get("routes", {}).get("current_route", [])
        self.current_route_relay_point = data.get("routes", {}).get("current_route_relay_point", [])
        self.current_route_xy = data.get("routes", {}).get("current_route_xy", [])
        self.current_route_relay_point_xy = data.get("routes", {}).get("current_route_relay_point_xy", [])

    def load_routes(self, filename):
        if os.path.exists(filename):
            with open(filename, "r", encoding="utf-8") as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError:
                    print("JSON 格式錯誤，載入失敗")
                    return {}
        else:
            print(f"找不到 {filename}，使用空路線")
            return {}

    @pyqtSlot(float, float)
    def receiveCoords(self, lat, lng):
        print(f"新增節點: [{lat:.6f}, {lng:.6f}]")
        self.current_route.append([lat, lng])

    @pyqtSlot(int)
    def deleteNode(self, index):
        if 0 <= index < len(self.current_route):
            lat, lng = self.current_route[index]
            print(f"刪除節點: [{lat:.6f}, {lng:.6f}]")
            self.current_route.pop(index)
        # 同步到地圖
        js_code = f"drawInitialRoute({json.dumps(self.current_route)});"
        self.parent_window.webview.page().runJavaScript(js_code)

    @pyqtSlot(int, float, float)
    def nodeMoved(self, index, lat, lng):
        if 0 <= index < len(self.current_route):
            self.current_route[index] = [round(lat, 6), round(lng, 6)]
            print(f"節點 {index + 1} 移動到座標: [{lat:.6f}, {lng:.6f}]")

    @pyqtSlot("QVariantList")
    def updateRoute(self, new_route):
        formatted_route = [[round(lat, 6), round(lng, 6)] for lat, lng in new_route]
        self.current_route = formatted_route
        print(f"更新後路線: ")
        for point in self.current_route:
            lat, lng = point
            print(f"[{lat:.6f}, {lng:.6f}]")

# ---------------------- 自訂 Logger ----------------------
class Logger:
    def __init__(self, widget):
        self.widget = widget

    def write(self, message):
        message = message.strip()
        if message:
            self.widget.append(message)
            self.widget.verticalScrollBar().setValue(self.widget.verticalScrollBar().maximum())

    def flush(self):
        pass

# ====================== JSON 接收執行緒 ======================
class JSONWorker(QThread):
    jsonReceived = pyqtSignal(dict)
    error = pyqtSignal(str)

    def __init__(self, url, bufsize=65535):
        super().__init__()
        self.url = url
        self.bufsize = bufsize
        self._running = True

        if url.startswith("udp://"):
            parts = url[6:].split(":")
            self.ip = parts[0]
            self.port = int(parts[1])
        else:
            raise ValueError("目前只支援 udp:// 格式")

    def stop(self):
        self._running = False
        self.quit()
        self.wait()

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.ip, self.port))
            sock.settimeout(1.0)
            # print(f"JSONWorker 正在監聽 {self.url} ...")

            while self._running:
                try:
                    data, _ = sock.recvfrom(self.bufsize)
                    msg = data.decode("utf-8").strip()
                    try:
                        obj = json.loads(msg)
                        self.jsonReceived.emit(obj)
                    except json.JSONDecodeError:
                        print("收到的資料不是 JSON: ", msg)
                except socket.timeout:
                    continue
                except Exception as e:
                    self.error.emit(str(e))
                    break
        except Exception as e:
            self.error.emit(str(e))

# ====================== 影像顯示元件 + YOLO ======================
class VideoWidget(QWidget):
    detectedClass = pyqtSignal(int, str)  # (class_id, label)

    def __init__(self, url: str, model_path="D:/ML/project/code/runs/detect/trainxx6/weights/best.pt"):
        super().__init__()
        self.url = url
        self.label = QLabel("連線中…")
        self.label.setAlignment(Qt.AlignCenter)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0,0,0,0)
        lay.addWidget(self.label)

        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

        # 加入 YOLO 模型
        self.model = YOLO(model_path)
        self.frame_count = 0          # 幀計數
        self.yolo_interval = 5        # 每 5 幀跑一次 YOLO
        self.last_annotated = None    # 上次推論結果

        self.running = False
        self.color = [(0, 255, 255), (0, 255, 0), (0, 0, 255), (255, 255, 255)]

        self.last_frame = None
        self.last_boxes = []

    def use_camera(self):
        self.url = 0  # 使用預設攝影機
        self.start()

    def start(self):
        self.running = True
        self.label.setText("連線中…")
        QApplication.processEvents()

        if self.cap is None:
            self.cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            # self.cap = cv2.VideoCapture("C:/Users/LiangYingWu/Downloads/car.mp4")
            if not self.cap.isOpened():
                self.label.setText("無法連線到影像串流")
                self.use_camera()
                # return
        self.timer.start(30)

    def stop(self):
        self.running = False
        self.timer.stop()
        if self.cap:
            self.cap.release()
            self.cap = None
        self.last_annotated = None
        self.frame_count = 0
        self.label.setText("已停止影像串流")

    def update_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.frame_count += 1

                if self.frame_count % self.yolo_interval == 0:
                    results = self.model(frame, verbose=False)
                    annotated_frame = frame.copy()

                    self.last_boxes.clear()

                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy()
                    confidences = results[0].boxes.conf.cpu().numpy() 

                    for i, box in enumerate(boxes):
                        x1, y1, x2, y2 = map(int, box)
                        cls = int(classes[i])
                        conf = confidences[i]
                        label = results[0].names[cls]

                        if conf < 0.5:
                            continue

                        # 存框框資訊
                        self.last_boxes.append({
                            "class_id": int(cls),
                            "label": str(label),
                            "confidence": round(float(conf), 6),
                            "bbox": [float(x1), float(y1), float(x2), float(y2)]
                        })

                        label = f"{label} {conf:.2f}"

                        self.detectedClass.emit(cls, results[0].names[cls])

                        if cls == 0:
                            color = (0, 255, 255)
                        elif cls == 1:
                            color = (0, 255, 0)
                        elif cls == 2:
                            color = (0, 0, 255)
                        else:
                            color = (255, 255, 255)

                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(annotated_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(annotated_frame, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)

                        cv2.putText(annotated_frame, label, (x1, y1 - 3),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    
                    self.last_annotated = annotated_frame
                    self.last_frame = frame.copy()

                display_frame = self.last_annotated if self.last_annotated is not None else frame

                rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
                pix = QPixmap.fromImage(qimg).scaled(self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.label.setPixmap(pix)

    def resizeEvent(self, e):
        if self.label.pixmap():
            self.label.setPixmap(self.label.pixmap().scaled(self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        super().resizeEvent(e)

# ---------------------- 主視窗 ----------------------
class MainWindow(QMainWindow):
    def __init__(self, html_file, video_url, json_url, model_path="yolov8n.pt", \
                 routes_file_path="routes.json", historical_record_path="historical_record"):
        super().__init__()
        self.setWindowTitle("智慧校園自駕巡檢系統")
        self.resize(1280, 720)

        self.route_planning_active = False

        main_widget = QWidget()
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # ---------------- 左側區域：改為疊層（地圖 / 影像 / 歷史紀錄） ----------------
        left_container = QWidget()
        self.left_stack = QStackedLayout(left_container)
        main_layout.addWidget(left_container, 3)

        # 地圖頁
        self.webview = QWebEngineView()
        file_path = os.path.abspath(html_file)

        self.channel = QWebChannel()
        self.bridge = Bridge(routes_file_path)
        self.bridge.parent_window = self
        self.channel.registerObject("bridge", self.bridge)
        self.webview.page().setWebChannel(self.channel)
        self.webview.load(QUrl.fromLocalFile(file_path))

        # 路線存檔路徑
        self.routes_file_path = routes_file_path

        # 影像頁（RTSP）
        self.video_url = video_url
        self.video_widget = VideoWidget(self.video_url, model_path=model_path)
        
        # 歷史紀錄頁
        self.history_widget = QWidget()
        history_layout = QHBoxLayout()
        self.history_widget.setLayout(history_layout)

        self.file_list = QListWidget()
        self.file_list.itemClicked.connect(self.load_history_item)
        history_layout.addWidget(self.file_list, 1)

        # 右邊：上下分 → 圖片 / JSON+地圖
        history_splitter = QSplitter(Qt.Vertical)
        self.history_image = QLabel("尚未選擇檔案")
        self.history_image.setAlignment(Qt.AlignCenter)
        self.history_image.setScaledContents(True)
        history_splitter.addWidget(self.history_image)

        # 下半部：左右分 → JSON / 地圖
        json_map_splitter = QSplitter(Qt.Horizontal)
        self.history_json = QTextEdit()
        self.history_json.setReadOnly(True)
        json_map_splitter.addWidget(self.history_json)

        self.map_view = QWebEngineView()
        json_map_splitter.addWidget(self.map_view)
        json_map_splitter.setSizes([500, 500]) 

        # 將包含 JSON 和地圖的 splitter 加到上層 splitter
        history_splitter.addWidget(json_map_splitter)
        history_layout.addWidget(history_splitter, 3) # 將完整的 splitter 加入主佈局

        # 放入堆疊：index 0 = 地圖、index 1 = 影像、index 2 = 歷史紀錄
        self.left_stack.addWidget(self.webview)
        self.left_stack.addWidget(self.video_widget)
        self.left_stack.addWidget(self.history_widget)
        self.left_stack.setCurrentIndex(0)

        # ---------------- 右側面板 ----------------
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)
        main_layout.addWidget(right_panel, 1)

        # 按鈕區
        self.button_layout = QVBoxLayout()
        right_layout.addLayout(self.button_layout)

        self.btn_route = QPushButton("路線規劃")
        self.btn_route.clicked.connect(self.toggle_route_planning)
        self.button_layout.addWidget(self.btn_route)

        self.btn_calculate = QPushButton("計算路線")
        self.btn_calculate.clicked.connect(self.calculate_route_points)
        self.btn_calculate.setVisible(False)
        self.button_layout.addWidget(self.btn_calculate)

        self.route_buttons = []

        self.btn_info = QPushButton("自駕車資訊")
        self.btn_info.clicked.connect(self.self_driving_information)
        self.button_layout.addWidget(self.btn_info)

        self.btn_history = QPushButton("歷史紀錄")
        self.btn_history.clicked.connect(self.historical_record)
        self.button_layout.addWidget(self.btn_history)

        self.button_layout.addStretch()

        # Log 視窗固定在右側下方
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setFixedHeight(200)
        right_layout.addWidget(self.log_widget)  # 放在按鈕區下方
        
        # 監聽網頁載入完成事件
        self.webview.loadFinished.connect(self.on_page_load_finished)

        # 將 print 重定向到 log_widget
        sys.stdout = Logger(self.log_widget)
        sys.stderr = Logger(self.log_widget)

        # 初始化地理座標轉換參數
        self.lat0 = 25.0102477
        self.lng0 = 121.5399238
        
        # 初始化 OSM 路網 (地圖 API)
        center_point = (25.01335, 121.54057)
        dist = 800
        self.G = ox.graph_from_point(center_point, dist=dist, network_type="walk")

        # 啟動 JSONWorker
        self.json_worker = JSONWorker(json_url)
        self.json_worker.jsonReceived.connect(self.handle_json)
        self.json_worker.error.connect(lambda e: print("JSONWorker 錯誤:", e))
        self.json_worker.start()
        self.historical_record_path = historical_record_path
        os.makedirs(os.path.join(self.historical_record_path, "json"), exist_ok=True)
        os.makedirs(os.path.join(self.historical_record_path, "image"), exist_ok=True)

        self.target_detected = False
        self.target_cls = [0, 2]
        # self.target_cls = [1]
        self.video_widget.detectedClass.connect(self.on_yolo_detected)

    # def latlngToXY(self, lat, lng):
    #     deg_to_rad = math.pi / 180.0
    #     dlat = (lat - self.lat0) * deg_to_rad
    #     dlng = (lng - self.lng0) * deg_to_rad

    #     r = 6378137.0  # WGS84
    #     x = dlng * r * math.cos(self.lat0 * deg_to_rad)
    #     y = dlat * r

    #     return x, y
    
    def latlngToXY(self, lat, lng):
        print("Converting lat/lng to x/y:", lat, lng)
        
        proj_wgs84 = Proj("epsg:4326")

        proj_local = Proj(
            proj='aeqd', 
            ellps='WGS84', 
            datum='WGS84', 
            lat_0=self.lat0, 
            lon_0=self.lng0
        )
        
        transformer = Transformer.from_proj(proj_wgs84, proj_local)
        x_coords, y_coords = transformer.transform(lat, lng)

        print("x_coords:", x_coords, "y_coords:", y_coords)

        return round(x_coords, 10), round(y_coords, 10)

    def on_page_load_finished(self):
        if self.bridge.current_route:
            js_code = f"drawInitialRoute({json.dumps(self.bridge.current_route)});"
            self.webview.page().runJavaScript(js_code)
        if self.bridge.current_route_relay_point:
            js_code = f"drawRelayPoints({json.dumps(self.bridge.current_route_relay_point)});"
            self.webview.page().runJavaScript(js_code)
        print("---已載入當前路線到地圖---")

    def on_yolo_detected(self, cls, label):
        if cls in self.target_cls:
            # print("get target:", cls, label)
            self.target_detected = True

    def handle_json(self, obj):
        # print("get json:", obj)

        if self.target_detected:
            timestamp = str(obj.get("timestamp", int(time.time())))

            if self.video_widget.last_boxes:
                # print(self.video_widget.last_boxes)
                obj["yolo_detections"] = self.video_widget.last_boxes

            json_filename = os.path.join(self.historical_record_path, "json", f"{timestamp}.json")
            with open(json_filename, "w", encoding="utf-8") as f:
                json.dump(obj, f, ensure_ascii=False, indent=2)
            print(f"json saved: {json_filename}")

            if self.video_widget.last_frame is not None:
                img_filename = os.path.join(self.historical_record_path, "image", f"{timestamp}.jpg")
                cv2.imwrite(img_filename, self.video_widget.last_frame)
                print(f"image saved: {img_filename}")

            self.target_detected = False

    def toggle_route_planning(self):
        # if self.left_stack.currentIndex() != 0:
        if not self.route_planning_active:
            print("---啟動路線規劃模式---")
            self.webview.page().runJavaScript("enableRoutePlanning();")
            self.left_stack.setCurrentIndex(0)
            self.video_widget.stop()
            self.btn_route.setText("退出路線規劃")
            self.route_planning_active = True
            self.show_route_buttons()

            # 禁用其他按鈕
            self.btn_info.setEnabled(False)
            self.btn_history.setEnabled(False)
        else:
            print("---退出路線規劃模式---")
            self.webview.page().runJavaScript("disableRoutePlanning();")
            self.btn_route.setText("路線規劃")
            self.route_planning_active = False
            self.clear_route_buttons()

            # 啟用其他按鈕
            self.btn_info.setEnabled(True)
            self.btn_history.setEnabled(True)

            self.calculate_route_points()

            # 將路線存成json
            self.save_routes_to_json()

    def show_route_buttons(self):
        for i, (name, route) in enumerate(self.bridge.routes.items()):
            btn = QPushButton(name)
            btn.setFixedWidth(int(self.btn_route.width() * 0.9))
            btn.clicked.connect(lambda checked, r=route: self.load_route(r))

            # 使用水平 layout 讓按鈕靠右
            h_layout = QHBoxLayout()
            h_layout.addStretch()
            h_layout.addWidget(btn)
            self.button_layout.insertLayout(i + 1, h_layout)

            self.route_buttons.append(btn)
            self.btn_calculate.setVisible(True)

    def clear_route_buttons(self):
        for btn in self.route_buttons:
            self.button_layout.removeWidget(btn)
            btn.deleteLater()
        self.route_buttons = []
        self.btn_calculate.setVisible(False)

    def load_route(self, route):
        print(f"載入路線: ")
        for point in route:
            lat, lng = point
            print(f"[{lat:.6f}, {lng:.6f}]")
        self.bridge.current_route = copy.deepcopy(route)
        self.webview.page().runJavaScript(f"drawInitialRoute({json.dumps(route)});")

    def save_routes_to_json(self):
        data = {
            "routes": {
                "default_route": self.bridge.routes,
                "current_route": self.bridge.current_route,
                "current_route_relay_point": self.bridge.current_route_relay_point,
                "current_route_xy": self.bridge.current_route_xy,
                "current_route_relay_point_xy": self.bridge.current_route_relay_point_xy
            }
        }
        with open(self.routes_file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
        print(f"---路線已儲存於 {self.routes_file_path}---")

    def calculate_real_route(self, points, step=2):
        """
        points: [[lat, lon], ...] 主要點
        step: 中繼點間隔 (公尺)
        return: {
            "new_current_route": [[lat, lon], ...],  # 全部路徑 (包含路口與非路口)
            "delay_points": [[lat, lon], ...]        # 中繼點
        }
        """
        if len(points) < 2:
            print("路線點數不足，無法計算路線")
            return {
                "new_current_route": points,
                "delay_points": [],
                "new_current_route_xy": [],
                "delay_points_xy": []
            }
        
        # 找最近 OSM 節點
        nodes = [ox.distance.nearest_nodes(self.G, lon, lat) for lat, lon in points]

        full_path = []
        for i in range(len(nodes) - 1):
            try:
                segment = nx.shortest_path(self.G, nodes[i], nodes[i+1], weight="length")
                full_path.extend(segment if i == 0 else segment[1:])
            except nx.NetworkXNoPath:
                print(f"找不到路徑: {nodes[i]} → {nodes[i+1]}")
                continue

        coords = [(self.G.nodes[n]["y"], self.G.nodes[n]["x"]) for n in full_path]

        # 全部節點（不管路口或不是路口）都要保留
        new_current_route = []
        for n in full_path:
            lat, lon = self.G.nodes[n]["y"], self.G.nodes[n]["x"]
            p = [round(lat, 6), round(lon, 6)]
            if not new_current_route or p != new_current_route[-1]:
                new_current_route.append(p)

        # 計算中繼點
        waypoints = []
        for i in range(len(coords) - 1):
            lat1, lon1 = coords[i]
            lat2, lon2 = coords[i + 1]
            length = ox.distance.great_circle(lat1, lon1, lat2, lon2)
            num_points = int(length // step)
            if num_points > 0:
                lats = np.linspace(lat1, lat2, num_points, endpoint=False)
                lons = np.linspace(lon1, lon2, num_points, endpoint=False)
                waypoints.extend(zip(lats, lons))
        waypoints.append(coords[-1])

        delay_points = [[round(float(lat), 6), round(float(lon), 6)] for lat, lon in waypoints]
        
        print(f"原始路線點數: {len(points)}，計算後路線點數: {len(new_current_route)}，中繼點數: {len(delay_points)}")
        new_current_route_xy = [list(map(lambda v: round(v, 3), self.latlngToXY(lat, lon)))
                                for lat, lon in new_current_route]
        delay_points_xy = [list(map(lambda v: round(v, 3), self.latlngToXY(lat, lon)))
                        for lat, lon in delay_points]

        return {
            "new_current_route": new_current_route,
            "delay_points": delay_points,
            "new_current_route_xy": new_current_route_xy,
            "delay_points_xy": delay_points_xy
        }

    def calculate_route_points(self):
        """計算中繼點並在地圖顯示小點"""
        if not self.bridge.current_route:
            print("尚未設定路線")

        # 計算中繼點
        result = self.calculate_real_route(self.bridge.current_route, step=2)
        self.bridge.current_route = result["new_current_route"]
        self.bridge.current_route_relay_point = result["delay_points"]
        self.bridge.current_route_xy  = result["new_current_route_xy"]
        self.bridge.current_route_relay_point_xy = result["delay_points_xy"]
        print("計算中繼點完成，總點數:", len(self.bridge.current_route_relay_point))

        # 傳給 JS 繪圖
        js_code = f"drawInitialRoute({json.dumps(self.bridge.current_route)});"
        self.webview.page().runJavaScript(js_code)
        js_code = f"drawRelayPoints({json.dumps(self.bridge.current_route_relay_point)});"
        self.webview.page().runJavaScript(js_code)

    def self_driving_information(self):
        if self.left_stack.currentIndex() != 1:
            print("---切換到自駕車影像---")
            self.left_stack.setCurrentIndex(1)
            # 這行確保每次切過來先顯示文字
            self.video_widget.label.setText("連線中…")
            QApplication.processEvents()
            self.video_widget.start()

    def historical_record(self):
        print("---歷史紀錄---")
        self.left_stack.setCurrentIndex(2)

        # 清空舊清單
        self.file_list.clear()

        # 列出 json 資料夾檔案
        json_dir = os.path.join(self.historical_record_path, "json")
        if not os.path.exists(json_dir):
            return

        files = sorted(os.listdir(json_dir))
        for f in files:
            if f.endswith(".json"):
                name = os.path.splitext(f)[0]
                self.file_list.addItem(name)
    
    def load_history_item(self, item):
        from PyQt5.QtGui import QPainter, QPen, QFont

        name = item.text()

        # 檔案路徑
        img_path = os.path.join(self.historical_record_path, "image", f"{name}.jpg")
        json_path = os.path.join(self.historical_record_path, "json", f"{name}.json")

        # 讀取 JSON
        gps_lat, gps_lng = None, None
        data = "找不到 JSON 檔"
        if os.path.exists(json_path):
            with open(json_path, "r", encoding="utf-8") as f:
                data = f.read()
            try:
                json_obj = json.loads(data)
                detections = json_obj.get("yolo_detections", [])
                gps_lat = json_obj.get("gps", {}).get("lat", None)
                gps_lng = json_obj.get("gps", {}).get("lng", None)
            except Exception as e:
                print("JSON 解析錯誤:", e)
                data = "JSON 解析錯誤"
        
        # 更新 JSON 顯示
        self.history_json.setPlainText(data)
        
        # 顯示圖片並繪製 YOLO 框
        if os.path.exists(img_path):
            pixmap = QPixmap(img_path)
            if detections:
                painter = QPainter(pixmap)

                colors = {
                    0: Qt.yellow,
                    1: Qt.green,
                    2: Qt.red
                }

                font = QFont()
                font.setPointSize(12)
                font.setBold(True)
                painter.setFont(font)
                
                for det in detections:
                    try:
                        x1, y1, x2, y2 = map(int, det["bbox"])
                        cls = det["class_id"]
                        label = det["label"]
                        conf = det["confidence"]
                        
                        pen_color = colors.get(cls, Qt.white)
                        pen = QPen(pen_color, 3)
                        painter.setPen(pen)
                        
                        painter.drawRect(x1, y1, x2 - x1, y2 - y1)

                        text = f"{label} {conf:.2f}"
                        rect = painter.boundingRect(x1, y1 - 20, 200, 20, Qt.AlignLeft, text)
                        painter.fillRect(rect, pen_color)
                        painter.setPen(Qt.black)
                        painter.drawText(rect, Qt.AlignLeft, text)
                        painter.setPen(pen)
                    except Exception as e:
                        print("繪圖錯誤:", e)
                painter.end()
            self.history_image.setPixmap(pixmap)
        else:
            self.history_image.setText("找不到圖片")
        
        # 更新地圖
        if gps_lat is not None and gps_lng is not None:
            map_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8" />
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
                <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
                <style>
                    html, body {{
                        height: 100%;
                        margin: 0;
                    }}
                    #map {{
                        width: 100%;
                        height: 100%;
                    }}
                </style>
            </head>
            <body>
                <div id="map"></div>
                <script>
                    var map = L.map('map').setView([{gps_lat}, {gps_lng}], 17);
                    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                        attribution: '&copy; OpenStreetMap contributors',
                        maxZoom: 19
                    }}).addTo(map);
                    L.marker([{gps_lat}, {gps_lng}]).addTo(map)
                        .bindPopup("位置: {gps_lat}, {gps_lng}").openPopup();
                </script>
            </body>
            </html>
            """
            self.map_view.setHtml(map_html)
        else:
            self.map_view.setHtml("<h3>沒有 GPS 資料</h3>")
        
        # 強制處理事件以確保渲染
        QApplication.processEvents()
        # print("GPS:", gps_lat, gps_lng)

    # 確保關閉視窗時停止串流執行緒
    def closeEvent(self, event):
        try:
            self.video_widget.stop()
        except Exception:
            pass
        super().closeEvent(event)

# ----------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    html_file = "code/map.html"
    video_url = "udp://0.0.0.0:5000"
    json_url = "udp://0.0.0.0:5001"
    model_path = "D:/ML/project/code/runs/detect/trainxx6/weights/best.pt"
    routes_file_path = "code/routes.json"
    historical_record_path = "code/historical_record"
    window = MainWindow(html_file, video_url, json_url, model_path, routes_file_path, historical_record_path)
    window.show()
    sys.exit(app.exec_())
