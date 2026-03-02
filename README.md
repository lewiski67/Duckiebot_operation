# VPA_ROBOT_OPERATION

This package allows operating the robots in MiniCCAM, VPA for a complete process.

It calls for the lower layer packages, including robot interface for traction and sensors, robot perception for preprocessing the data from the sensors.

# 🎮 DuckieRace – Joystick Operation Guide

This guide explains how to operate a robot during the DuckieRace demo.

The robot drives autonomously using lane following.  
The joystick is used to adjust speed and manage game mechanics.

---

## ✅ Before Starting

Make sure:

- The robot is powered on
- The line-follow node is running
- The robot is placed correctly on the yellow lane
- (Optional) Overhead camera manager is running for full game mode

Once started, the robot should begin driving forward automatically.

---

# 🕹 Joystick Controls

## Speed Control

### L1 – Increase Speed
- Increases forward velocity
- Also increases energy consumption

### R1 – Decrease Speed
- Decreases forward velocity
- Reduces energy consumption

The robot always moves forward unless braking or losing the line.

---

## 🔋 Charging (Fuel Mechanic)

Charging works only when:

- The robot is inside the **Fuel Zone**
- The overhead camera manager is running
- Fuel-zone signal is active

### Y – Start Charging
- Press and hold to recharge energy
- Robot must be inside the fuel zone

If charging does not work:
- Check that the overhead camera system is running
- Confirm the robot is physically inside the fuel zone

---

## 🛑 Brake Override

### X – Release Local Brake

If the robot is stopped due to merge control:

- Press **X** once
- This sends a brake release command

If the robot stops again immediately:
- It is still inside a merge conflict zone

---

## 🔴 Red Line Mode (Optional)

### B – Toggle Red Line Mode

When enabled:
- Robot prefers red line instead of yellow

If the red line disappears for ~1 second:
- Robot automatically switches back to yellow tracking

---

# ⚠️ Common Demo Situations

## Robot Suddenly Stops

Possible reasons:

- Lost the lane → Reposition robot on yellow line
- Merge zone brake → Wait or press X
- Energy too low → Move to fuel zone and charge

---

## Robot Not Charging

- Confirm robot is inside fuel zone
- Confirm overhead camera manager is running
- Hold Y button

---

## Steering Looks Unstable

- Check lighting conditions
- Ensure yellow line is clearly visible
- Slightly reposition the robot

---

# 🧠 Important Notes

- The joystick does NOT steer the robot.
- Steering is fully automatic via onboard camera.
- The joystick only controls speed and game mechanics.
- If the overhead camera manager is NOT running:
  - Merge control is disabled
  - Charging is disabled
  - Robots still drive normally