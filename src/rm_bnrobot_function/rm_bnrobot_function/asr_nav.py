#!/usr/bin/env python3
import sys
import os
import json
import threading
import subprocess

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import vosk
import pyaudio
from ament_index_python.packages import get_package_share_directory

# 引入 PyQt5 模块
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTextEdit, QLabel, QGroupBox, QPushButton)
from PyQt5.QtCore import QThread, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QFont, QTextCursor

# ==========================================
# 1. Qt 
# ==========================================
class Communicate(QObject):
    update_text = pyqtSignal(str)
    update_coord = pyqtSignal(str, float, float, float)
    update_mic = pyqtSignal(str)  

# ==========================================
# 2. ROS 2 节点定义
# ==========================================
class VoiceCommander(Node):
    def __init__(self, comms):
        super().__init__('voice_commander_gui')
        self.comms = comms 
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # --- 初始化模型 ---
        model_path = os.path.join(get_package_share_directory('rm_bnrobot_function'), 'model')
        if not os.path.exists(model_path):
            self.comms.update_text.emit(f"错误: 模型路径不存在 {model_path}")
            return
            
        self.comms.update_text.emit("正在加载 Openai Whisper 模型...")
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, 16000)
        
        # --- 初始化音频 ---

        self.p = pyaudio.PyAudio()
        target_index = None
        device_name = "未找到麦克风"
        
        self.comms.update_text.emit("正在扫描麦克风设备...")
        for i in range(self.p.get_device_count()):
            dev_info = self.p.get_device_info_by_index(i)
            name = dev_info['name']
            # 优先匹配蓝牙或耳机
            if "bluez" in name.lower() or "headset" in name.lower() or "handsfree" in name.lower():
                if dev_info['maxInputChannels'] > 0:
                    target_index = i
                    device_name = f"{name}"
                    break
        
        # 如果没找到蓝牙，使用默认输入设备
        if target_index is None:
            try:
                default_dev = self.p.get_default_input_device_info()
                target_index = default_dev['index']
                device_name = f" 默认设备: {default_dev['name']}"
            except:
                device_name = " 无可用录音设备"

        # 【核心逻辑】将选中的麦克风名称打印到 QT 界面
        self.comms.update_mic.emit(device_name)
        
        try:
            self.stream = self.p.open(
                format=pyaudio.paInt16, channels=1, rate=16000, 
                input=True, input_device_index=target_index, frames_per_buffer=8000
            )
            self.comms.update_text.emit("语音监听已就绪")
        except Exception as e:
            self.comms.update_text.emit(f"录音设备打开失败: {e}")
            self.comms.update_mic.emit("录音设备打开失败")
            return

        # --- 启动录音监听线程 ---
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def listen_loop(self):
        while rclpy.ok():
            try:
                data = self.stream.read(4000, exception_on_overflow=False)
                if self.rec.AcceptWaveform(data):
                    result = json.loads(self.rec.FinalResult())
                    text = result.get('text', '').replace(' ', '')
                    if text:
                        self.comms.update_text.emit(f"识别到: {text}")
                        self.process_command(text)
            except Exception as e:
                pass

    def process_command(self, text):
        text_lower = text.lower()
        locations = {
            '大门': (-1.79, 3.52, 0.0),
            '原点': (0.0, 0.0, 0.0),
            '教室': (2.38, 10.41, 0.0),
            '厨房': (4.14, 2.3, 0.0),
            '卧室': (-3.43, 2.06, 0.0)
        }
        for keyword, coords in locations.items():
            if keyword in text_lower:
                self.comms.update_text.emit(f"前往: {keyword}")
                self.comms.update_coord.emit(keyword, coords[0], coords[1], coords[2])
                self.send_goal(*coords)
                return

    def send_goal(self, x, y, z):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = float(z)
        goal_msg.pose.orientation.w = 1.0 
        self.publisher_.publish(goal_msg)

# ==========================================
# 3. ROS 线程
# ==========================================
class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
    def run(self):
        rclpy.spin(self.node)

# ==========================================
# 4. PyQt5 主窗口
# ==========================================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 语音导航控制台")
        self.resize(900, 650)
        self.rviz_process = None

        # --- 信号连接 ---
        self.comms = Communicate()
        self.comms.update_text.connect(self.append_log)
        self.comms.update_coord.connect(self.update_target_display)
        self.comms.update_mic.connect(self.set_mic_display) # 【新增】

        # --- 初始化 UI ---
        self.init_ui()

        # --- 初始化 ROS ---
        rclpy.init()
        self.node = VoiceCommander(self.comms)
        self.ros_thread = RosThread(self.node)
        self.ros_thread.start()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # 【新增】顶端设备状态条
        self.mic_status_label = QLabel(" 正在检测麦克风...")
        self.mic_status_label.setStyleSheet("""
            background-color: #34495E; 
            color: #F1C40F; 
            padding: 8px; 
            border-radius: 5px;
            font-weight: bold;
        """)
        self.mic_status_label.setFont(QFont("Arial", 11))
        main_layout.addWidget(self.mic_status_label)

        # 上半部分
        top_layout = QHBoxLayout()

        # [左上] 日志
        log_group = QGroupBox(" 语音识别记录")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 11))
        self.log_text.setStyleSheet("background-color: #FDFEFE;")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)

        # [右上] 坐标
        coord_group = QGroupBox(" 导航状态预览")
        coord_layout = QVBoxLayout()
        self.coord_label = QLabel("等待语音指令...")
        self.coord_label.setAlignment(Qt.AlignCenter)
        self.coord_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.coord_label.setStyleSheet("color: #7F8C8D; border: 1px dashed #BDC3C7;")
        coord_layout.addWidget(self.coord_label)
        coord_group.setLayout(coord_layout)

        top_layout.addWidget(log_group, 3)
        top_layout.addWidget(coord_group, 2)

        # [下半] 控制
        rviz_group = QGroupBox(" 系统控制")
        rviz_layout = QHBoxLayout()
        
        self.btn_launch_rviz = QPushButton("启动 RViz2 视图")
        self.btn_launch_rviz.setMinimumHeight(60)
        self.btn_launch_rviz.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        self.btn_launch_rviz.setStyleSheet("""
            QPushButton { background-color: #2980B9; color: white; border-radius: 8px; }
            QPushButton:hover { background-color: #3498DB; }
        """)
        self.btn_launch_rviz.clicked.connect(self.launch_rviz)

        rviz_layout.addWidget(self.btn_launch_rviz)
        rviz_group.setLayout(rviz_layout)

        main_layout.addLayout(top_layout, 2) 
        main_layout.addWidget(rviz_group, 1) 
        self.setLayout(main_layout)

    # --- 槽函数 ---
    def set_mic_display(self, name):
        """【新增】在界面状态条显示麦克风名称"""
        self.mic_status_label.setText(f"当前输入源: {name}")

    def append_log(self, text):
        self.log_text.append(text)
        self.log_text.moveCursor(QTextCursor.End)

    def update_target_display(self, name, x, y, z):
        display_text = f"目标点: {name}\n\nX: {x:.2f}\nY: {y:.2f}\nZ: {z:.2f}"
        self.coord_label.setText(display_text)
        self.coord_label.setStyleSheet("color: #27AE60; border: 2px solid #27AE60; background-color: #EBF5FB;")

    def launch_rviz(self):
        if self.rviz_process is None or self.rviz_process.poll() is not None:
            self.append_log(">>> 正在启动 RViz2...")
            self.rviz_process = subprocess.Popen(['rviz2'])
            self.btn_launch_rviz.setText(" RViz2 运行中")
            self.btn_launch_rviz.setEnabled(False)

    def closeEvent(self, event):
        if self.rviz_process:
            self.rviz_process.terminate()
        if hasattr(self, 'node'):
            self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


