# 🦾 Robotic Hand Gesture Control

Control a **5-DOF simulated robotic hand** in real-time using your webcam!  
This project combines **computer vision (MediaPipe)** and **physics simulation (MuJoCo)** to let a robot hand imitate your **fist open/close** gestures instantly.  

---

## ✨ Features
- 🎯 Real-time hand gesture detection (open/close)
- 🦿 Shadow Hand physics simulation using **MuJoCo**
- ⚙️ Smooth transition animation between gestures
- 🧠 Fully integrated — vision + control in one script
- 🔌 Easily extendable for real robotic hand hardware (e.g., CISMR arm)

---

## 🗂️ Project Structure

Robotic-Hand-Gesture-Control/
│
├── mujoco_menagerie/
│ └── shadow_hand/
│ ├── right_hand.xml
│ ├── assets/
│ └── (other MuJoCo model files)
│
├── gesture.py # Main script (vision + simulation)
├── requirements.txt # Dependencies
└── README.md

yaml
Copy code

---

## ⚙️ Installation & Setup

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/priyanshivmehta/Robotic-Hand-Gesture-Control.git
cd Robotic-Hand-Gesture-Control
```
### 2️⃣ Create and Activate a Virtual Environment
Use Python 3.10 or 3.11 (avoid 3.12 for MediaPipe compatibility).

```bash
Copy code
py -3.10 -m venv cv
cv\Scripts\activate          # On Windows
# or
source cv/bin/activate       # On macOS/Linux
```
### 3️⃣ Install Required Libraries
```bash
Copy code
pip install --upgrade pip
pip install -r requirements.txt
If you don’t have a requirements.txt, create one with:

txt
Copy code
opencv-python
mediapipe
mujoco
numpy
```
### 4️⃣ Download the MuJoCo Menagerie (if not included)
Copy code
```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```
Then copy the folder:

```bash
Copy code
mujoco_menagerie/shadow_hand/
into your project directory.
```
Make sure this file exists:

```bash
Copy code
mujoco_menagerie/shadow_hand/right_hand.xml
```
🚀 Running the Simulation
Start the gesture control simulation with:

```bash
Copy code
python gesture.py
```
Two windows will open:

🎥 Webcam Feed – detects your real-time hand gesture

🦿 MuJoCo Viewer – robotic hand follows your gesture

Gesture	Simulation
🖐️ Open palm	Hand opens
✊ Closed fist	Hand closes

Press ESC to exit.

## 🛠️ Hardware Requirements

To build the physical robotic arm as detailed in the project presentation:

### **Components List**
| Component | Description |
| :--- | :--- |
| **Arduino Nano** | Acts as the main microcontroller to read commands and control servos. |
| **Micro Servo Motors** | 5 motors (one for each finger). |
| **3D Printed Hand** | Custom or InMoov standard parts. |
| **Fishing Lines** | Acts as tendons to connect the fingers to the motors. |
| **Breadboard** | Used for power distribution. |
| **Power Supply** | External 5V supply recommended for servos. |

### **3D Printing & Assembly**
* **Material:** PLA+ or PETG is recommended for structural integrity.
* **Tendons:** Thread fishing lines through the 3D-printed fingers and tie them to the servo horns. Ensure the line is taut when the servo is at 0 degrees (Open position).

---

## ⚡ Circuit Connections

### **Wiring**
1.  **Servos to Arduino:** Connect the signal pins of the 5 servos to the Arduino PWM-enabled pins (e.g., D3, D5, D6, D9, D10).
2.  **Power Distribution (Breadboard):**
    * **Upper Horizontal Rail:** Connect to the **+5V** external power source (Powers all servo motors).
    * **Lower Horizontal Rail:** Connect to **Ground**.
    * **Common Ground:** **Important:** Connect the Arduino GND to the external power source GND to complete the circuit.

---

## 🔌 Hardware Setup (Arduino)

1.  Connect your Arduino Nano to your PC via USB.
2.  Upload the control sketch (e.g., `hardware/arduino_code/hand_control.ino`) to the board.
3.  **Running with Hardware:**
    * Update your Python controller script with the correct `COM_PORT` (e.g., `COM3` or `/dev/ttyUSB0`).
    * Run the script to start sending serial data to the Arduino.
