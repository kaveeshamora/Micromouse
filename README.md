# 🐭 Micromouse Competition Code

This repository contains the code used in our **Micromouse Competition**, where we successfully implemented **maze exploration and solving algorithms**. Our team competed against 12 others and won the challenge using a combination of **Depth-First Search (DFS) for exploration** and **Floodfill for solving**.

## 🚀 Features
- **Maze Exploration**: DFS algorithm for discovering the maze layout.
- **Maze Solving**: Floodfill algorithm for optimal pathfinding.
- **PWM Motor Control**: Smooth and precise movement of the robot.
- **Ultrasonic Sensors**: Distance measurement for wall-following.
- **PID Controller**: Ensuring accurate straight-line movement.

## 🛠️ Hardware Used
- **Microcontroller**: Arduino Uno
- **Motors**: DC motors without encoders
- **Sensors**: Three ultrasonic sensors for left, right and front wall detection, One IR sensor for goal detection
- **Power Supply**: LiPo battery

## 📜 Code Overview
- `main.ino` - The arduino code for the micromouse
- `depth_first_search.py` - Python code for the simulator which used to check the algorythm
