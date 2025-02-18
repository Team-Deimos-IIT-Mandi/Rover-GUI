#!/usr/bin/env python3
import rospy
import base64
import cv2
import numpy as np
import asyncio
import websockets
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from threading import Thread
import subprocess
import os

bridge = CvBridge()
websocket_url = 'ws://localhost:8080'  # Update with server IP if needed

class ImageSender:
    def __init__(self, websocket_url):
        self.websocket_url = websocket_url
        self.loop = asyncio.new_event_loop()
        self.ws = None
        self.running = True

        # Start the WebSocket loop in a background thread
        self.thread = Thread(target=self.run_loop, daemon=True)
        self.thread.start()

    def run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.connect_websocket())

    async def connect_websocket(self):
        while self.running:
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    rospy.loginfo("Connected to WebSocket server")
                    self.ws = websocket
                    await asyncio.Future()  # Keep the connection open
            except Exception as e:
                rospy.logerr(f"WebSocket connection failed: {e}, retrying in 3 seconds...")
                await asyncio.sleep(3)  # Retry connection

    def send_image(self, data):
        try:
            if self.ws:
                # Convert ROS Image to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                _, buffer = cv2.imencode('.jpg', cv_image)
                image_base64 = base64.b64encode(buffer).decode('utf-8')

                # Send the image asynchronously
                asyncio.run_coroutine_threadsafe(self.ws.send(image_base64), self.loop)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def launch_smach(self) :
        try:
           package_name = "smach_rover"
           launch_file = "smach.launch"
           rospy.loginfo("Launching SMACH state machine...")
           subprocess.Popen(["roslaunch", package_name, launch_file], preexec_fn=os.setpgrp)
        except Exception as e:
           rospy.logerr(f"Failed to launch SMACH state machine: {e}")        

    def stop(self):
        self.running = False
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()

if __name__ == '__main__':
    rospy.init_node('image_sender_node', anonymous=True)
    
    sender = ImageSender(websocket_url)
    sender.launch_smach()
    rospy.Subscriber('/camera/color/image_raw', Image, sender.send_image)
    rospy.loginfo("Subscribed to /camera/color/image_raw")

    try:
        rospy.spin()  # Keep ROS running
    except KeyboardInterrupt:
        sender.stop()
        rospy.loginfo("Shutting down image sender.")
