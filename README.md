# Autonomous Robotics 

### The goal of this project is to Transport 9 colored blocks from a cluttered area to a construction zone of a 10x10 square feet arena. 

Order of Pickup: Red, Green, Blue repeated three times (R-G-B -> R-G-B -> R-G-B)

### The testing area consists of the following zones:
- Landing Zone
- Construction Zone
- Cluttered Block Area

### Hardware 
- Raspberry Pi for compute and control
- PiCamera2 for block detection 
- DFRobot Baron platform with a parallel gripper
- IMU for heading angle estimation
- Arduino Nano acts as a bridge between IMU and Raspberry Pi
- Sonar for collision detection

### Software 
- Closed-loop proportional controller for motor control 
- Real-time block detection: Uses HSV masking and block geometry based on bounding box size and shape
- Block distance and angle offset are estimated using bounding box aspect ratio
- A* algorithm applied on a static 2D grid map 

### Dependencies
- RPi.GPIO 
- Picamera2
- Python serial 
- Python socket

### To install, clone the Git repository in your working directory
```bash
git clone git@github.com:pranavdm99/Autonomous_Mobile_Robot.git
```

### To run the code, navigate into the 'Autonomous_Mobile_Robot' folder and type the following command
```bash
python3 main.py
```
