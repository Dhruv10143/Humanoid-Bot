# Humanoid_2023  

## Overview  
**Humanoid_2023** is a project focused on building a humanoid robot capable of simulating human-like motion using accessible hardware. The robot is actuated with servo motors mounted on a metallic frame and controlled via an Arduino UNO.  

The motion of the humanoid is based on **Inverse Kinematics (IK)**, which allows calculation of joint angles needed for the robot’s limbs to follow a desired trajectory. This project combines **mechanical design, embedded systems, and robotics theory** into a functional system.  

---

## Core Theory – Inverse Kinematics  

Inverse Kinematics (IK) is used to compute the required joint angles of a robot’s limb in order to reach a specified end position.  

- **Forward Kinematics (FK):** Joint angles → End-effector position  
- **Inverse Kinematics (IK):** End-effector position → Joint angles  

For humanoids, IK is essential because we define where we want the **foot or hand** to move, and the system calculates how the **joints (hip, knee, etc.)** must rotate.  

**Reference Playlists for IK:**  
- [Introduction to Inverse Kinematics – Angela Sodemann](https://www.youtube.com/watch?v=BkMQ5Rek_vM&list=PLT_0lwItn0sAfi3o4xwx-fNfcnbfMrXa7&ab_channel=AngelaSodemann)  
- [Advanced IK Applications – Angela Sodemann](https://www.youtube.com/watch?v=pLXoDRctwRg&list=PLT_0lwItn0sDBE98BsbaZezflB96ws12b&ab_channel=AngelaSodemann)  

---

## Components Used  

### MG958 Servo Motor  
- High torque (~25kg/cm).  
- Used in **leg joints** where heavy loads are applied.  
- Provides precise and stable control.  

### MG995 Servo Motor  
- Medium torque (~10kg/cm).  
- Suitable for **arm joints and lighter loads**.  
- Affordable and widely available in hobby robotics.  

### Metallic Frame  
- Provides rigidity and structural stability.  
- Withstands repeated motion and vibrations.  
- Serves as the chassis for mounting all servo motors.  

*These components were chosen to balance strength, affordability, and motion accuracy.*  

---

## Core Logic – IK Based Motion  

The humanoid’s gait is modeled using a **parabolic trajectory** for each leg. This ensures smooth, continuous walking-like motion. The inverse kinematics equations compute the required joint angles, which are then applied to the servo motors via the Arduino UNO.  

### Example IK Implementation  

```cpp
#include <Servo.h>

Servo hipServo, kneeServo;

// Link lengths (cm)
float L1 = 10.0;  // thigh
float L2 = 10.0;  // shin

// Inverse Kinematics calculation
void computeIK(float x, float y, float &theta1, float &theta2) {
    float D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
    theta2 = atan2(-sqrt(1 - D*D), D);  // knee
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));  // hip
}

// Move leg to (x,y) position
void moveLeg(float x, float y) {
    float theta1, theta2;
    computeIK(x, y, theta1, theta2);

    int hipAngle = (int)(theta1 * 180.0 / M_PI);
    int kneeAngle = (int)(theta2 * 180.0 / M_PI);

    hipServo.write(90 + hipAngle);   // calibration offset
    kneeServo.write(90 + kneeAngle);
}

void setup() {
    hipServo.attach(9);
    kneeServo.attach(10);
}

void loop() {
    // Parabolic trajectory
    for (float t = 0; t < 1.0; t += 0.05) {
        float x = 10;
        float y = 5 + 5 * t * (1 - t); // parabola
        moveLeg(x, y);
        delay(50);
    }
}
```

---

## Key Highlights  
- Implemented **Inverse Kinematics** for realistic humanoid motion.  
- Designed motion using **parabolic trajectory planning**.  
- Used **MG958 and MG995 servos** for torque-optimized performance.  
- Controlled via **Arduino UNO** with PWM-based servo actuation.  

---

## Repository Structure  

```
Humanoid_2023/
│── Humanoid_v2_1.ino   # Arduino code implementing IK and servo control
│── README.md           # Project documentation
│── /media              # Images or diagrams (optional)
```
