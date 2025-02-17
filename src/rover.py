#!/usr/bin/env python3
import rospy
import base64
import json
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import asyncio
import websockets
import subprocess
import os

bridge = CvBridge()
current_state = "idle"

def robot_state_callback(msg):
    global current_state
    current_state = msg.data
    rospy.loginfo(f"State updated to: {current_state}")

async def handle_commands(websocket):
    command_publisher = rospy.Publisher('/change_state', String, queue_size=10)
    async for message in websocket:
        try:
            data = json.loads(message)
            if data["type"] == "command":
                command = data["command"]
                rospy.loginfo(f"Received command: {command}")
                command_publisher.publish(command)
        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")

async def send_state_update(websocket):
    global current_state
    while True:
        try:
            state_message = json.dumps({"type": "state_update", "state": current_state})
            await websocket.send(state_message)
            await asyncio.sleep(0.1)  # 10 Hz
        except Exception as e:
            rospy.logerr(f"Error sending state update: {e}")

async def send_image_to_server(websocket_url):
    async with websockets.connect(websocket_url) as websocket:
        await websocket.send(json.dumps({"type": "register", "role": "robot"}))
        rospy.loginfo("Registered as robot.")

        asyncio.create_task(handle_commands(websocket))
        asyncio.create_task(send_state_update(websocket))

        def callback(data):
            try:
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                _, buffer = cv2.imencode('.jpg', cv_image)
                image_base64 = base64.b64encode(buffer).decode('utf-8')
                image_message = json.dumps({"type": "video_stream", "data": image_base64})
                asyncio.run(websocket.send(image_message))
            except Exception as e:
                rospy.logerr(f"Error processing image: {e}")

        rospy.Subscriber('/camera/color/image_raw', Image, callback)
        rospy.spin()

def launch_smach_state_machine():
    """Launch the SMACH state machine."""
    try:
        launch_file_path = os.path.join(
            os.environ["HOME"],
            "marz/src/Mars-Rover/smach_rover/launch/smach_rover.launch"
        )
        rospy.loginfo(f"Launching SMACH state machine from: {launch_file_path}")
        subprocess.Popen(["roslaunch", launch_file_path])
    except Exception as e:
        rospy.logerr(f"Failed to launch SMACH state machine: {e}")

if __name__ == '__main__':
    rospy.init_node('image_sender_node', anonymous=True)

    # Launch SMACH state machine
    launch_smach_state_machine()

    rospy.Subscriber('/robot_state', String, robot_state_callback)
    websocket_url = 'ws://localhost:8080'
    asyncio.get_event_loop().run_until_complete(send_image_to_server(websocket_url))
