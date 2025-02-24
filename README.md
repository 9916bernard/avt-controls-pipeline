🚗 Autonomous Vehicle Team (AVT) - Controls Pipeline
This repository contains the Controls Pipeline for Penn State's Autonomous Vehicle Team (AVT) in 2024, focusing on sensor data integration, decision-making, and state machine execution.

🔹 Key Features:
Sensor Decider 📡: Processes LiDAR, Camera, and HDMaps data in ROS2 (Python) to generate real-time vehicle trajectories.
Mock Dataset Publisher 🛠️: Simulates sensor inputs to handle uncertainty in perception data and enable continuous testing.
State Machine Integration 🚦: Publishes control signals for lane changes, stops, and emergency stops based on real-time sensor inputs.
Real-Time Debugging & Adaptation 🔄: Improves decision-making by dynamically adjusting to sensor updates without regenerating full waypoints.

🔧 Technologies Used:
✅ ROS2 (Humble) | ✅ Python | ✅ State Machine
