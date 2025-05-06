# 🤖 Dynamic Modeling and Control of a Multi-Prismatic Robotic System

This project explores the dynamic modeling and control strategies of a robotic arm with multiple prismatic joints. It includes simulation, analytical derivation of dynamics, indirect force control, and a Graphical User Interface (GUI) to visualize and interact with the system in real-time.

---

## 📌 Overview

- Built a dynamic model of an **n-link prismatic robotic system** using MATLAB
- Derived **equations of motion** using Denavit–Hartenberg parameters and dynamic modeling techniques
- Implemented **compliance control (PD + gravity compensation)** and **impedance control**
- Developed an **interactive GUI** in MATLAB App Designer to allow users to input parameters, visualize joint behavior, and control the robot

---

## 🧠 Key Features

### 🧮 Analytical Derivations
- Used DH parameters to compute transformation matrices
- Derived Jacobians for motors and links
- Formulated equations including **inertial**, **centrifugal**, **Coriolis**, and **gravitational** effects
- Generated torque equations for dynamic simulations

### 🧪 Simulation
- Simulated joint displacement, velocity, and acceleration over time
- Visualized desired vs. actual end-effector positions and forces under:
  - **Compliance Control**
  - **Impedance Control**

### 🖥️ GUI Interface
- Designed using MATLAB App Designer
- Allows real-time input of:
  - DH parameters
  - Mass and inertia of links/motors
  - Control gains
  - Environmental stiffness
- Outputs:
  - Motion plots
  - Force response
  - Governing equations

---

## 📊 Results

- Generated torque and motion profiles for joint behaviors
- Achieved realistic simulation of compliant and stiff interaction via force control
- Demonstrated control system response through real-time GUI plots:
  - Position vs time
  - Velocity vs time
  - Acceleration vs time
  - Desired vs Actual Position (both controls)
  - Contact forces (both controls)

---

## 📚 Learning Outcomes

- Derived full dynamic equations for prismatic robotic systems
- Gained experience in MATLAB-based modeling, simulation, and GUI development
- Understood advanced robotic control strategies (compliance & impedance)
- Built a robust framework for modeling and testing n-DOF robotic manipulators

---
