# 0001 – Development environment

Date: 2026-10-15
Status: accepted

## Context
The project needs a C++ toolchain, version control and later ROS 2. The laptop runs Windows
(i5-8350U, 16 GB RAM); drive C: has little free space, drive D: has plenty.

## Options considered
1. Windows native (VS Code + MSVC): simplest USB access to the robot, but little Linux experience.
2. Windows + WSL2 Ubuntu 24.04: real Linux environment as used in industry robotics, ROS 2 compatible;
   robot USB needs to be passed through with usbipd-win.
3. Dual boot Ubuntu: pure Linux, but most setup effort and risk.

## Decision
Option 2. Ubuntu 24.04 in WSL2, moved to D:\WSL\Ubuntu because of limited space on C:.
Toolchain: GCC 13, CMake 3.28, Git, GitHub CLI, VS Code with the WSL extension.

## Consequences
- The code must keep building on Linux (checked by GitHub Actions on every push).
- USB access to the robot goes through usbipd-win until the ESP32 WiFi bridge replaces the cable.
