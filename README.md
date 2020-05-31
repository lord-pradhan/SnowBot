# SnowBot

SnowBot is an autonomous snow-clearing mobile robot that won Best Project Overall at MechE Design Expo 2019 at Carnegie Mellon University. The final [demo](https://www.youtube.com/watch?v=jlkrziDI0P4 "demo") shows SnowBot successfully localizing itself and covering a user-defined area while executing dump manouvers to get rid of the snow at designated 'dump zones'. The only user input provided is the target area, and dump zone definition. 

The only sensors used were inexpensive UWB beacons (4 anchor, 1 tag), an IMU, and a magnetometer. Based on these inputs, SnowBot was able to localize and cover the pre-defined target map.

This repository houses the autonomy stack and Arduino firmware that goes into making SnowBot autonomous. The autonomy stack was developed in Python3 and deployed on a Raspi 4, while the low-level firmware was deployed on three different Arduinos. 

Autonomy stack - High-level decision making module, coverage planner, path-following controller, sensor fusion using an extended Kalman-filter.

Low-level Arduino code - Motor control, processing sensor data and communicating with Raspi, interfacing with linear actuator for plow, etc.

![alt text](https://i.ibb.co/vVtWMqC/IMG-2951.jpg)
![alt text](https://i.ibb.co/WVKszJW/IMG-2991.jpg)
