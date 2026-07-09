🤖 Autonomous Line Following Robot using Webots (C Programming)

<p align="center">
  <img src="media/demo.gif" width="600"/>
</p><p align="center">
  🚀 Robotics • C Programming • PID Control • Webots Simulation
</p>---

📌 Overview

This project demonstrates the design and simulation of an Autonomous Line Following Robot using the Webots robotics simulator, implemented entirely in C programming.

The robot intelligently follows a predefined path using distance sensors and applies a control-based approach to maintain stability and accuracy. By continuously processing sensor feedback and dynamically adjusting motor velocities, the system achieves smooth and reliable navigation.

Unlike basic line-following bots, this project integrates control system principles, making it more robust, efficient, and closer to real-world autonomous systems.

---

✨ Key Features

- 🚗 Real-time line following using distance sensors
- 🎯 Proportional (P) control for smooth navigation
- ⚡ Low-level motor control using C
- 🔁 Continuous feedback-based correction system
- 🌐 Fully simulated in Webots environment

---

🧠 System Architecture

- Sensors → Detect line position
- Controller (C Code) → Compute error & correction
- Motors → Adjust velocity dynamically
- Environment (Webots) → Simulated track

---

⚙️ How It Works

1. The robot reads values from left and right sensors
2. Calculates deviation (error) from the line
3. Applies control logic:
   error = left_sensor - right_sensor;
correction = Kp * error;
4. Adjusts motor speeds accordingly
5. Continuously repeats for stable path tracking

---

🛠️ Tech Stack

- Language: C
- Simulator: Webots
- Concepts: Control Systems, Robotics, Embedded Logic

---

📁 Project Structure

autonomous-line-follower-webots-c/
│
├── controllers/
│   └── line_follower/
│       ├── line_follower.c
│       ├── pid_controller.c
│       ├── pid_controller.h
│
├── worlds/
│   └── line_world.wbt
│
├── media/
│   ├── demo.gif
│   └── screenshots/
│
├── Makefile
└── README.md

---

▶️ How to Run

1. Open Webots
2. Load the world file:
   worlds/line_world.wbt
3. Run the simulation ▶️

---

📷 Demo

<p align="center">
  <img src="media/demo.gif" width="600"/>
</p>

---

🎯 Key Highlights

- Built using pure C (low-level control)
- Demonstrates real-time robotics logic
- Implements control system fundamentals
- Clean modular project structure

---

🚀 Future Improvements

- 🔄 Full PID Control (Kp + Ki + Kd)
- 👁️ Camera-based obstacle detection (OpenCV)
- 🧭 Autonomous path planning
- 🤖 SLAM & AI integration

---

💼 Resume Value

This project showcases:

- ✅ Embedded Systems Programming
- ✅ Robotics Simulation
- ✅ Control Systems (PID)
- ✅ Real-time Decision Making

---

🏆 Why This Project Stands Out

Unlike basic projects, this implementation focuses on control accuracy and system design, making it highly relevant for roles in:

- Robotics
- Embedded Systems
- Autonomous Vehicles
- Control Engineering

---

👨‍💻 Author

Shruthi Addu
