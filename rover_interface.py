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

# Initialize ROS node once
if not rospy.core.is_initialized():
    rospy.init_node('streamlit_ros_bridge', anonymous=True, disable_signals=True)

# --- Persistent ROS Data using st.experimental_singleton ---
@st.cache_resource
def get_ros_data():
    return {
        # Initialize with a blank image
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
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Initialize publisher here

# --- ROS Callback Functions ---
def image_callback(msg):
    """Convert incoming ROS image messages to an OpenCV image and update persistent data."""
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
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
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/odometry/filtered/global", Odometry, odom_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, waypoints_callback)
    rospy.spin()

# Start the ROS listener thread only once
if "ros_thread_started" not in st.session_state:
    ros_thread = threading.Thread(target=ros_listener, daemon=True)
    ros_thread.start()
    st.session_state["ros_thread_started"] = True

# --- Teleoperation Helper ---
def send_cmd_vel(linear, angular):
    """Publish a Twist command to the /cmd_vel topic."""
    try:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_vel_pub.publish(twist)
    except Exception as e:
        st.warning(f"Error publishing command: {str(e)}")

# --- Main Thread: Update st.session_state from persistent ROS data ---
with ros_data_lock:
    # Always copy the camera feed, even if it's the initial blank image
    frame = ros_data["camera_feed"].copy()
    
    # Convert OpenCV image to bytes only if it's not empty
    if frame is not None and frame.size > 0:
        _, buffer = cv2.imencode('.jpg', frame)
        st.session_state["camera_feed"] = buffer.tobytes()
    else:
        st.session_state["camera_feed"] = None
    
    st.session_state["odometry"] = ros_data["odometry"].copy()
    st.session_state["waypoints"] = list(ros_data["waypoints"])

# --- GUI Layout ---
st.title("Mars Rover Control Interface")

# Camera Feed and Vital Information
col1, col2 = st.columns([3, 2])
with col1:
    st.header("Camera Feed")
    frame_bytes = st.session_state["camera_feed"]
    
    if frame_bytes:
        # Add a placeholder with spinner while loading
        with st.spinner("Loading camera feed..."):
            st.image(frame_bytes, channels="BGR", use_container_width=True)
    else:
        # Show a black placeholder if no feed available
        st.image(np.zeros((480, 640, 3)), channels="BGR", 
                caption="Waiting for camera feed...",
                use_container_width=True)



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
    if st.button("↑ Forward"):
        send_cmd_vel(0.5 * speed, 0.0)
with col_left:
    if st.button("← Left"):
        send_cmd_vel(0.0, 0.5 * speed)
with col_right:
    if st.button("→ Right"):
        send_cmd_vel(0.0, -0.5 * speed)
with col_center:
    if st.button("↓ Back"):
        send_cmd_vel(-0.5 * speed, 0.0)

# Autonomous Waypoints Display
st.sidebar.subheader("Autonomous Navigation")
st.sidebar.write("Received Waypoints:")
if st.session_state["waypoints"]:
    for wp in st.session_state["waypoints"]:
        st.sidebar.write(f"{wp[0]:.2f}, {wp[1]:.2f}")
else:
    st.sidebar.write("No waypoints received.")