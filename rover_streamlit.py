import streamlit as st
import cv2
import numpy as np
from PIL import Image
import time

# Initialize session state
if 'waypoints' not in st.session_state:
    st.session_state.waypoints = [(1,2,0)]
if 'odometry' not in st.session_state:
    st.session_state.odometry = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
if 'aruco_data' not in st.session_state:
    st.session_state.aruco_data = []

# Helper functions
def generate_synthetic_camera_image():
    img = np.zeros((400, 600, 3), dtype=np.uint8)
    cv2.putText(img, "Rover Camera Feed", (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.circle(img, (300, 200), 10, (0, 0, 255), -1)
    return img

def detect_aruco_markers(image):
    # Simulated ArUco detection
    return [{'id': 42, 'position': (300, 200)}]

def update_odometry(dx, dy, dtheta):
    st.session_state.odometry['x'] += dx
    st.session_state.odometry['y'] += dy
    st.session_state.odometry['theta'] += dtheta

# GUI Layout
st.title("Mars Rover Control Interface")

# Camera Feed
col1, col2 = st.columns([3, 2])
with col1:
    st.header("Camera Feed")
    camera_placeholder = st.empty()
    
    # Simulated camera feed
    while True:
        frame = generate_synthetic_camera_image()
        if st.session_state.aruco_data:
            markers = detect_aruco_markers(frame)
            for marker in markers:
                cv2.drawMarker(frame, marker['position'], 
                              (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        camera_placeholder.image(frame, channels="BGR")
        time.sleep(0.1)

with col2:
    st.header("Vital Information")
    
    # Odometry
    st.subheader("Odometry")
    st.metric("X Position", f"{st.session_state.odometry['x']:.2f} m")
    st.metric("Y Position", f"{st.session_state.odometry['y']:.2f} m")
    st.metric("Heading", f"{st.session_state.odometry['theta']:.2f}°")
    
    # System Status
    st.subheader("System Status")
    st.metric("Temperature", "42°C")
    st.metric("Battery", "78%")
    st.metric("Speed", "0.5 m/s")

# Controls
st.sidebar.header("Control Panel")

# Teleoperation Controls
st.sidebar.subheader("Manual Control")
speed = st.sidebar.slider("Speed", 0.0, 1.0, 0.5)

col1, col2, col3 = st.sidebar.columns(3)
with col2:
    if st.button("↑ Forward"):
        update_odometry(0, 0.1 * speed, 0)
with col1:
    if st.button("← Left"):
        update_odometry(-0.1 * speed, 0, 5)
with col3:
    if st.button("→ Right"):
        update_odometry(0.1 * speed, 0, -5)
with col2:
    if st.button("↓ Back"):
        update_odometry(0, -0.1 * speed, 0)

# Autonomous Waypoints
st.sidebar.subheader("Autonomous Navigation")
lat = st.sidebar.number_input("Latitude")
lon = st.sidebar.number_input("Longitude")

if st.sidebar.button("Add Waypoint"):
    st.session_state.waypoints.append((lat, lon))
    
if st.sidebar.button("Clear Waypoints"):
    st.session_state.waypoints = []

st.sidebar.write("Current Waypoints:")
for wp in st.session_state.waypoints:
    st.sidebar.write(f"{wp[0]}, {wp[1]}")

# Spiral Search
st.sidebar.subheader("Autonomous Patterns")
if st.sidebar.button("Start Spiral Search"):
    st.sidebar.write("Initiating spiral search pattern...")

# ArUco Detection
st.sidebar.subheader("Computer Vision")
st.session_state.aruco_data = st.sidebar.checkbox("Enable ArUco Detection")

if st.session_state.aruco_data:
    st.sidebar.write("Detected Markers:")
    for marker in st.session_state.aruco_data:
        st.sidebar.write(f"Marker ID: {marker['id']} at {marker['position']}")

# Run with: streamlit run rover_interface.py