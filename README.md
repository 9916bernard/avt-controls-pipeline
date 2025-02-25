AVT Controls Pipeline

🚗 Autonomous Vehicle Team (AVT) - Controls & State Machine for Decision Making

This repository contains the Control & Decision-Making Pipeline for Penn State’s Autonomous Vehicle Team (AVT) in 2024 Fall, integrating sensor data, state machine logic, and vehicle control mechanisms. The pipeline ensures real-time trajectory planning, lane changes, and stop behavior based on sensor fusion data.

🛠️ Core Components

1️⃣ State Machine & Controls

controls_state_machine.py → Finite State Machine managing vehicle actions (stop, lane change, acceleration).

devel_csm_custom.py → Custom version aligning with the sensor_decider, modified for real-time testing.

2️⃣ Sensor & Decision Layer

sensor_decider.py → Determines vehicle behavior (stop/lane change) based on LiDAR, HDMaps, and simulated perception data.

test_detector.py → Generates mock object detection data in [Object ID, Lat, Long] format.

test_location.py → Provides hard-coded GPS locations for local testing without real GPS input.

3️⃣ Path & Motion Planning

updatepath_custom.py → Updates the path dynamically based on real-time sensor inputs.

4️⃣ HDMaps & Localization

get_data_custom.py → Retrieves HDMaps data from CSV resources and aligns objects with vehicle position.

🔍 Key Functionalities & Challenges

Dynamic Decision Pipeline: Integrated sensor-decider with state machine, ensuring seamless transitions based on perception and HDMaps data.

Mock Testing & Debugging: Created mock dataset publishers (test_detector.py, test_location.py) to simulate real sensor input before perception data was fully integrated.

Collaborative Pipeline Development: Addressed inconsistent upstream data formats by establishing a standardized input format and iteratively adjusting mock data for testing.

Real-Time Data Handling: Removed unnecessary plotting delays to improve real-time performance.

