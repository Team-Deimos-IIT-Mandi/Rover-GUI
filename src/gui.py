import sys
import base64
import cv2
import numpy as np
import asyncio
import websockets
import rospy
from std_msgs.msg import String
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget,
                             QPushButton, QHBoxLayout)


class VideoStreamThread(QThread):
    frame_received = pyqtSignal(np.ndarray)

    def __init__(self, websocket_url):
        super().__init__()
        self.websocket_url = websocket_url
        self.running = True  # Flag to control thread execution

    async def receive_video(self):
        while self.running:
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    print("Connected to signaling server.")
                    while self.running:
                        image_base64 = await websocket.recv()
                        image_data = base64.b64decode(image_base64)
                        np_arr = np.frombuffer(image_data, dtype=np.uint8)
                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if cv_image is not None:
                            self.frame_received.emit(cv_image)
            except Exception as e:
                print(f"WebSocket error: {e}, reconnecting in 3 seconds...")
                await asyncio.sleep(3)  # Reconnect delay

    def run(self):
        loop = asyncio.new_event_loop()  # Create a new event loop for the thread
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.receive_video())

    def stop(self):
        self.running = False  # Stop receiving video
        self.quit()  # Stop the thread


class VideoStreamWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        rospy.init_node('smach_gui', anonymous=True)  # Initialize ROS node
        self.state_publisher = rospy.Publisher('/smach_state', String, queue_size=10)

        self.setWindowTitle("Real-Time Video Stream")
        self.setGeometry(100, 100, 800, 600)

        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)

        self.state_label = QLabel("Current State: Idle")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 18px; font-weight: bold;")

        # Buttons for different states
        self.btn_idle = QPushButton("Idle")
        self.btn_teleop = QPushButton("Teleoperation")
        self.btn_auto = QPushButton("Autonomous Navigation")
        self.btn_search = QPushButton("Spiral Search")

        self.btn_idle.clicked.connect(lambda: self.update_state("idle"))
        self.btn_teleop.clicked.connect(lambda: self.update_state("teleop"))
        self.btn_auto.clicked.connect(lambda: self.update_state("autonomous"))
        self.btn_search.clicked.connect(lambda: self.update_state("spiral_search"))

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.btn_idle)
        button_layout.addWidget(self.btn_teleop)
        button_layout.addWidget(self.btn_auto)
        button_layout.addWidget(self.btn_search)

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.state_label)
        layout.addLayout(button_layout)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.video_thread = VideoStreamThread('ws://localhost:8080')
        self.video_thread.frame_received.connect(self.update_video_frame)

    def start_streaming(self):
        self.video_thread.start()

    def update_video_frame(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)

    def update_state(self, state):
        """Update state label and publish the state to ROS topic."""
        self.state_label.setText(f"Current State: {state.capitalize()}")
        rospy.loginfo(f"Publishing state: {state}")
        self.state_publisher.publish(String(state))  # Publish to ROS topic

    def closeEvent(self, event):
        self.video_thread.stop()  # Gracefully stop the thread
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoStreamWindow()
    window.show()
    window.start_streaming()
    sys.exit(app.exec_())
