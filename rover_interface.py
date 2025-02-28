import streamlit as st
import cv2
import numpy as np
import threading
import time
import math
import warnings

# Suppress ScriptRunContext warnings from background threads
warnings.filterwarnings("ignore", message=".*missing ScriptRunContext.*")

# ROS imports
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge, CvBridgeError

# For auto-refreshing the Streamlit app (install via: pip install streamlit-autorefresh)
from streamlit_autorefresh import st_autorefresh

# Auto-refresh every 100ms (adjust as needed, remove or increase limit if necessary)
st_autorefresh(interval=100, limit=10000, key="ros_autorefresh")

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
cmd_vel_pub = None  # Will be assigned later

# --- ROS Callback Functions ---
def image_callback(msg):
    """Convert incoming ROS image messages to an OpenCV image and update persistent data."""
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Check encoding: if it is rgb8 then convert; if already bgr8, skip conversion.
        if msg.encoding.lower() == "rgb8":
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        with ros_data_lock:
            ros_data["camera_feed"] = cv_image
    except CvBridgeError as e:
        print("CvBridge Error:", e)

def odom_callback(msg):
    """Extract odometry data from ROS messages and update persistent data."""
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    theta = math.atan2(siny_cosp, cosy_cosp)
    with ros_data_lock:
        ros_data["odometry"] = {'x': x, 'y': y, 'theta': theta}

def waypoints_callback(msg):
    """Receive waypoints (assumed as PoseStamped messages) and update persistent data."""
    x = msg.pose.position.x
    y = msg.pose.position.y
    with ros_data_lock:
        ros_data["waypoints"].append((x, y))

# --- ROS Listener in a Background Thread ---
def ros_listener():
    global cmd_vel_pub
    # Create a publisher for teleoperation commands
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # Subscribe to the necessary topics
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/odometry/filtered/global", Odometry, odom_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, waypoints_callback)
    rospy.spin()

# --- Initialize ROS Node in Main Thread ---
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_ros_bridge', anonymous=True, disable_signals=True)

# Start the ROS listener thread only once (using session state to check)
if "ros_thread_started" not in st.session_state:
    ros_thread = threading.Thread(target=ros_listener, daemon=True)
    ros_thread.start()
    st.session_state["ros_thread_started"] = True

# --- Teleoperation Helper ---
def send_cmd_vel(linear, angular):
    """Publish a Twist command to the /cmd_vel topic."""
    global cmd_vel_pub
    if cmd_vel_pub is not None:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_vel_pub.publish(twist)
    else:
        st.warning("Command publisher not ready yet.")

# --- Optional: Simulated ArUco Detection ---
def detect_aruco_markers(image):
    """
    Stub function for ArUco detection.
    Replace this with your actual detection code if available.
    """
    return [{'id': 42, 'position': (300, 200)}]

# --- Main Thread: Update st.session_state from persistent ROS data ---
with ros_data_lock:
    st.session_state["camera_feed"] = ros_data["camera_feed"].copy()
    st.session_state["odometry"] = ros_data["odometry"].copy()
    st.session_state["waypoints"] = list(ros_data["waypoints"])

# --- GUI Layout ---
st.title("Mars Rover Control Interface")

# Camera Feed and Vital Information
col1, col2 = st.columns([3, 2])
with col1:
    st.header("Camera Feed")
    frame = st.session_state["camera_feed"].copy()
    if st.sidebar.checkbox("Enable ArUco Detection", value=False):
        markers = detect_aruco_markers(frame)
        for marker in markers:
            cv2.drawMarker(frame, marker['position'], (0, 255, 0),
                           cv2.MARKER_CROSS, 20, 2)
    st.image(frame, channels="BGR")

with col2:
    st.header("Vital Information")
    st.subheader("Odometry")
    odom = st.session_state["odometry"]
    st.metric("X Position", f"{odom['x']:.2f} m")
    st.metric("Y Position", f"{odom['y']:.2f} m")
    st.metric("Heading", f"{odom['theta']:.2f} rad")
    
    st.subheader("System Status")
    st.metric("Temperature", "42°C")
    st.metric("Battery", "78%")
    st.metric("Speed", "0.5 m/s")

# Sidebar for Controls
st.sidebar.header("Control Panel")
st.sidebar.subheader("Manual Control")
speed = st.sidebar.slider("Speed", 0.0, 1.0, 0.5)

col_left, col_center, col_right = st.sidebar.columns(3)
with col_center:
    if st.sidebar.button("↑ Forward"):
        send_cmd_vel(0.5 * speed, 0.0)
with col_left:
    if st.sidebar.button("← Left"):
        send_cmd_vel(0.0, 0.5)
with col_right:
    if st.sidebar.button("→ Right"):
        send_cmd_vel(0.0, -0.5)
with col_center:
    if st.sidebar.button("↓ Back"):
        send_cmd_vel(-0.5 * speed, 0.0)

# Autonomous Waypoints Display
st.sidebar.subheader("Autonomous Navigation")
st.sidebar.write("Received Waypoints:")
if st.session_state["waypoints"]:
    for wp in st.session_state["waypoints"]:
        st.sidebar.write(f"{wp[0]:.2f}, {wp[1]:.2f}")
else:
    st.sidebar.write("No waypoints received.")

# Autonomous Patterns
st.sidebar.subheader("Autonomous Patterns")
if st.sidebar.button("Start Spiral Search"):
    st.sidebar.write("Initiating spiral search pattern...")

# Note:
# Run this application with: streamlit run <this_script.py>
