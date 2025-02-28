import streamlit as st
import cv2
import numpy as np
import threading
import time
import math
import warnings
from streamlit_autorefresh import st_autorefresh

# Suppress ScriptRunContext warnings from background threads
warnings.filterwarnings("ignore", message=".*missing ScriptRunContext.*")

# ROS imports
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge, CvBridgeError

# Auto-refresh every 100ms for real-time updates
st_autorefresh(interval=100, key="auto_refresh")

# Initialize ROS node once
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_ros_bridge', anonymous=True, disable_signals=True)

# --- Persistent ROS Data using st.experimental_singleton ---
@st.cache_resource
def get_ros_data():
    return {
        "camera_feed": np.zeros((480, 640, 3), dtype=np.uint8),
        "odometry": {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        "waypoints": []
    }

@st.cache_resource
def get_ros_data_lock():
    return threading.Lock()

ros_data = get_ros_data()
ros_data_lock = get_ros_data_lock()

# Initialize ROS-related globals
bridge = CvBridge()
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# --- ROS Callback Functions ---
def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding.lower() == "rgb8":
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        with ros_data_lock:
            ros_data["camera_feed"] = cv_image
    except CvBridgeError as e:
        print("CvBridge Error:", e)

def odom_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    theta = math.atan2(siny_cosp, cosy_cosp)
    with ros_data_lock:
        ros_data["odometry"] = {'x': x, 'y': y, 'theta': theta}

def waypoints_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    with ros_data_lock:
        ros_data["waypoints"].append((x, y))

# --- ROS Listener Thread ---
if "ros_thread_started" not in st.session_state:
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/odometry/filtered/global", Odometry, odom_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, waypoints_callback)
    st.session_state["ros_thread_started"] = True

# --- Teleoperation State Management ---
if 'movement' not in st.session_state:
    st.session_state.movement = {'linear': 0.0, 'angular': 0.0}

def set_movement(linear, angular):
    st.session_state.movement['linear'] = linear
    st.session_state.movement['angular'] = angular

def send_continuous_cmd_vel():
    twist = Twist()
    twist.linear.x = st.session_state.movement['linear']
    twist.angular.z = st.session_state.movement['angular']
    try:
        cmd_vel_pub.publish(twist)
    except Exception as e:
        st.error(f"Command error: {str(e)}")

# --- Main Update Loop ---
with ros_data_lock:
    frame = ros_data["camera_feed"].copy()
    odom = ros_data["odometry"].copy()
    waypoints = list(ros_data["waypoints"])

# Update session state for odometry and waypoints (if needed elsewhere)
st.session_state["odometry"] = odom
st.session_state["waypoints"] = waypoints

# Send continuous commands
send_continuous_cmd_vel()

# --- GUI Layout ---
st.title("Mars Rover Control Interface")

# Create two columns: one for Camera Feed and one for Odometry/Status
col1, col2 = st.columns([3, 2])

with col1:
    st.header("Camera Feed")
    # Use a placeholder to update the image directly
    camera_placeholder = st.empty()
    if frame.size > 0:
        camera_placeholder.image(frame, channels="BGR", use_container_width=True)
    else:
        camera_placeholder.image(np.zeros((480, 640, 3), dtype=np.uint8), 
                                 channels="BGR", 
                                 caption="Waiting for camera feed...")

with col2:
    st.header("Odometry")
    st.metric("X Position", f"{odom['x']:.2f} m")
    st.metric("Y Position", f"{odom['y']:.2f} m")
    st.metric("Heading", f"{odom['theta']:.2f} rad")
    
    st.header("System Status")
    st.metric("Temperature", "42°C")
    st.metric("Battery", "78%")
    st.metric("Speed", f"{abs(st.session_state.movement['linear']):.1f} m/s")

# --- Sidebar Layout ---
st.sidebar.header("Controls")
speed = st.sidebar.slider("Speed", 0.0, 1.0, 0.5, key="speed")

if st.sidebar.button("↑ Forward", key="forward"):
    set_movement(0.5 * speed, 0.0)
if st.sidebar.button("↓ Back", key="back"):
    set_movement(-0.5 * speed, 0.0)
if st.sidebar.button("← Left", key="left"):
    set_movement(0.0, 0.5 * speed)
if st.sidebar.button("→ Right", key="right"):
    set_movement(0.0, -0.5 * speed)
if st.sidebar.button("🛑 Stop", key="stop"):
    set_movement(0.0, 0.0)

# Checkbox for Aruco Detection
st.sidebar.checkbox("Enable Aruco Detection", key="aruco_detection")

st.sidebar.header("Navigation")
st.sidebar.write("Received Waypoints:")
if waypoints:
    for wp in waypoints:
        st.sidebar.write(f"{wp[0]:.2f}, {wp[1]:.2f}")
else:
    st.sidebar.write("No waypoints received.")

# Optionally display Aruco detection status
if st.session_state.get("aruco_detection", False):
    st.sidebar.info("Aruco Detection is enabled!")
