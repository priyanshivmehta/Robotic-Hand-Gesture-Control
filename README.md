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
python -m venv cv
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
