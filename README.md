# Mars Rover GUI

## Setup and Running Instructions

To run the GUI, follow these steps:

### 1. Start the Signaling Server
Ensure you have Node.js installed, then navigate to the directory containing `signaling_server.js` and run:
```sh
node signaling_server.js
```

### 2. Run rosmaster
```sh
roscore
```


### 2. Run the Rover Backend
Navigate to the `src` folder inside the `Rover-GUI` directory and execute:
```sh
python3 rover.py
```

### 3. Start the GUI
Move to the `src` directory where `rover_gui.py` is located and run:
```sh
python3 rover_gui.py
```