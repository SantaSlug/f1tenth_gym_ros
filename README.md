# 🚗 F1TENTH – Path Planning and Control Algorithms

This repository contains my individual implementations of key planning and control algorithms for autonomous driving using the 1/10th scale F1TENTH car in a ROS2 simulation environment. Each algorithm was developed, deployed, and tested independently to analyze its behavior in a race-like setting.

---

## 🔧 Tech Stack

- **Languages**: Python
- **Tools**: ROS2 Foxy, Docker, LiDAR
- **Visualization**: Gazebo, RViz

---

## 🧠 Algorithms Implemented

| Algorithm          | Description                                                                 |
|--------------------|-----------------------------------------------------------------------------|
| **Pure Pursuit**   | Curvature-based path tracking controller using forward lookahead points.    |
| **Wall Following** | Follows walls using side LiDAR distance buffers and direction logic.        |
| **Gap Following**  | Selects the widest navigable gap from LiDAR data and drives toward its center. |
| **Emergency Braking** | Safety node to stop the car if an obstacle is within a critical distance.     |

Each algorithm runs as a standalone ROS2 node and was tested independently in simulation.

---

## 🎥 Demo Videos

https://www.youtube.com/playlist?list=PLRxSesE54WqcCtlRP9rbtux69_9wy42aC

## 🚀 Running the Code

To install and run the simulation follow the instructions here-->https://github.com/f1tenth/f1tenth_gym_ros 

## 📊 Results
Pure Pursuit: Reduced lap time by ~3 seconds on benchmark track with parameter tuning.

Wall Following: Maintained consistent 0.8m lateral offset along curved walls.

Gap Following: Successfully navigated cluttered environments by dynamically selecting safe driving directions.

Emergency Braking: Triggered reliably if a collision was going to occure, ensuring collision-free stops.

Each module was tested in isolation for control smoothness, safety, and path quality.

## 📚 References
O’Kelly, M., Zheng, H., Karthik, D., & Mangharam, R. (2020). F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning. In NeurIPS 2019 Competition and Demonstration Track (pp. 77–89). PMLR.
