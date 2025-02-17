import sys
import base64
import json
import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QPushButton, QHBoxLayout, QTextEdit
import asyncio
import websockets


class VideoStreamThread(QThread):
    frame_received = pyqtSignal(np.ndarray)
    state_received = pyqtSignal(str)

    def __init__(self, websocket_url):
        super().__init__()
        self.websocket_url = websocket_url
        self.websocket = None

    async def connect(self):
        self.websocket = await websockets.connect(self.websocket_url)
        # Register GUI role
        await self.websocket.send(json.dumps({"type": "register", "role": "gui"}))
        print("Connected and registered as GUI.")

    async def receive_data(self):
        try:
            await self.connect()
            while True:
                message = await self.websocket.recv()
                data = json.loads(message)

                if data["type"] == "video_stream":
                    # Handle video stream
                    image_data = base64.b64decode(data["data"])
                    np_arr = np.frombuffer(image_data, dtype=np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if cv_image is not None:
                        self.frame_received.emit(cv_image)

                elif data["type"] == "state_update":
                    # Handle state updates
                    self.state_received.emit(data["state"])
        except Exception as e:
            print(f"Error receiving data: {e}")

    async def safe_send_command(self, command):
        try:
            if self.websocket:
                message = {"type": "command", "command": command}
                await self.websocket.send(json.dumps(message))
                print(f"Sent command: {command}")
        except Exception as e:
            print(f"Failed to send command: {e}")

    def run(self):
        asyncio.run(self.receive_data())


class VideoStreamWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Real-Time Video Stream with State Machine Control")
        self.setGeometry(100, 100, 1200, 800)

        # Video display
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)

        # Current state display
        self.state_text = QTextEdit(self)
        self.state_text.setReadOnly(True)

        # Control buttons
        self.shutdown_button = QPushButton("Shutdown", self)
        self.shutdown_button.clicked.connect(lambda: self.send_command("shutdown"))

        self.teleop_button = QPushButton("Teleoperation", self)
        self.teleop_button.clicked.connect(lambda: self.send_command("teleop"))

        self.autonomous_button = QPushButton("Autonomous", self)
        self.autonomous_button.clicked.connect(lambda: self.send_command("autonomous"))

        self.spiral_search_button = QPushButton("Spiral Search", self)
        self.spiral_search_button.clicked.connect(lambda: self.send_command("spiral_search"))

        # Layouts
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.shutdown_button)
        button_layout.addWidget(self.teleop_button)
        button_layout.addWidget(self.autonomous_button)
        button_layout.addWidget(self.spiral_search_button)

        left_layout = QVBoxLayout()
        left_layout.addWidget(QLabel("Robot State:", self))
        left_layout.addWidget(self.state_text)
        left_layout.addLayout(button_layout)

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.video_label)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 2)

        central_widget = QWidget(self)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Video thread
        self.video_thread = VideoStreamThread('ws://localhost:8080')
        self.video_thread.frame_received.connect(self.update_video_frame)
        self.video_thread.state_received.connect(self.update_state_text)

    def start_streaming(self):
        self.video_thread.start()

    def update_video_frame(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)

    def update_state_text(self, state):
        self.state_text.setText(state)

    def send_command(self, command):
        asyncio.create_task(self.video_thread.safe_send_command(command))

    def closeEvent(self, event):
        self.video_thread.quit()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoStreamWindow()
    window.show()
    window.start_streaming()
    sys.exit(app.exec_())
