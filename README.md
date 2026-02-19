# David — 6DOF Robotic Manipulator
A fully custom 6DOF robotic manipulator designed, simulated and physically built as part of my engineering thesis.  
The project integrates mechanical design, embedded systems, and ROS2-based motion planning into a complete robotic platform.

<img width="600" height="800" alt="image" src="https://github.com/user-attachments/assets/70c9cca8-ebec-4b3f-b11e-c3b0d6a3039b" />


The project covers the complete development pipeline:

- Mechanical design (Autodesk Inventor)
- Manufacturing of custom STL parts (3D printing)
- URDF generation (fusion2urdf)
- SRDF configuration (MoveIt Setup Assistant)
- Motion planning (MoveIt2)
- ROS2-based control system
- Hardware interface via ESP32 (micro-ROS)
- Voice-controlled TCP positioning

---

## 🎥 Demo

See David in action here!

https://youtu.be/BOY79-WEXMo?si=CrP1gkWR68Nr-e8x

---
## 📄 Thesis
Thesis PDF:

docs/David_6DOF_Robot_Thesis.pdf

## Project architecture: 

<img width="500" height="600" alt="Diagram bez tytułu drawio" src="https://github.com/user-attachments/assets/e57ab350-f511-4cb2-b557-80b26c5b4de7" />

## 🧱 Mechanical Design

- Fully custom CAD model  

<img width="669" height="639" alt="image" src="https://github.com/user-attachments/assets/26eef864-e761-4b04-aa57-82adac00d398" />

- 3D-printed structural components  
- Modular assembly for easy maintenance and reconfiguration  
- Designed with focus on low weight and structural rigidity  
- Press-fitted radial and needle bearings used to reduce friction and improve load handling  
- Rigid connections implemented using heat-set brass inserts embedded in the printed parts  

### Base Joint Design

The base joint uses:
- Two radial bearings for structural rigidity  
- One needle bearing to ensure smooth rotation under axial load  

The first joint is driven by a stepper motor, allowing unlimited rotation.  
A Hall sensor is used for homing and establishing a repeatable reference position.

To prevent cable twisting during continuous rotation, a slip ring was integrated into the base design.

Here is the base schematic:

<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/bff7783b-c00c-4406-b12e-2ec868c77aa4" />

### Internal Cable Routing

The links were designed with internal channels for cable routing.  
Cables pass through the joint structures and are guided through the bearing assemblies, minimizing external wiring and improving mechanical protection.

Cross-section showing cable routing:

<img width="320" height="500" alt="image" src="https://github.com/user-attachments/assets/61d74a06-cc14-46b7-a764-c1ee3addf0d2" />

---

## 🔌 Electronics

The system is powered from a single 12V DC supply.  
Voltage regulation for logic-level electronics is handled using DC-DC buck converters.

The ESP32 (micro-ROS node) is powered via USB connection to the host PC, which also provides the high-level control interface.

<img width="500" height="500" alt="power_system drawio" src="https://github.com/user-attachments/assets/01134cf4-ac9f-483f-a39b-010bfaf3f74f" />

### Rotating Base Interface

Electrical connection between the fixed base and the rotating platform is implemented using a 4-wire slip ring:

- 2 wires for power transmission
- 2 wires for communication between the ESP32 and the PCA9685 servo driver

The PCA9685 PWM driver controls the five servomotors in the manipulator arm and allows for future expansion (e.g., additional actuators in an end-effector or tool).

This solution enables continuous base rotation without cable twisting while maintaining reliable power and control communication.

## 🤖 Software Stack

- ROS2 humble
- fusion2urdf
- RViz
<img width="1920" height="945" alt="image" src="https://github.com/user-attachments/assets/5290cf80-5209-49c0-b7b4-d7d695ae8c76" />

- MoveIt2
<img width="1724" height="691" alt="image" src="https://github.com/user-attachments/assets/036515ff-96b4-47f6-8a59-40c56857063d" />

- Micro-ROS

Software architecture: 

<img width="300" height="600" alt="power drawio" src="https://github.com/user-attachments/assets/b6d2b3c4-7159-49b5-b1a3-ae3412250860" />


---

### Voice Control Interface

To showcase the modularity and versatility of the system, an additional ROS2 node was implemented for voice-based control of the TCP position.

The node uses the VOSK speech recognition library and is written in Python. It interprets spoken commands and publishes target TCP positions. These commands are then processed by a separate node utilizing the MoveIt2 C++ API for motion planning and execution.

Even though the nodes are written in different languages (Python and C++), ROS2 DDS communication allows seamless integration between them.

Thanks to the distributed ROS2 architecture, alternative control interfaces can be easily implemented by replacing the voice control node — for example with a GUI, joystick, or vision-based controller — without modifying the motion planning layer.



