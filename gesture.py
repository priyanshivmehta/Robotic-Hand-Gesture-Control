# import cv2
# import mediapipe as mp


# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils
# hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

# cap = cv2.VideoCapture(0)

# def is_fist_open(landmarks):
#     # Tip landmarks for fingers (index, middle, ring, pinky)
#     finger_tips = [8, 12, 16, 20]  
#     open_fingers = 0
    
#     for tip in finger_tips:
#         if landmarks[tip].y < landmarks[tip - 2].y:  # Tip above PIP joint = finger extended
#             open_fingers += 1
    
#     # If 3 or more fingers open = open palm, else fist
#     return open_fingers >= 3

# while True:
#     success, frame = cap.read()
#     frame = cv2.flip(frame, 1)  # Mirror view
#     rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     result = hands.process(rgb)

#     gesture = "NO HAND"

#     if result.multi_hand_landmarks:
#         for hand_landmarks in result.multi_hand_landmarks:
#             mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
#             if is_fist_open(hand_landmarks.landmark):
#                 gesture = "OPEN HAND"
#                 # Simulated robot action
#                 print("ROBOT ACTION → OPEN HAND")
#             else:
#                 gesture = "CLOSED FIST"
#                 print("ROBOT ACTION → CLOSE HAND")

#     cv2.putText(frame, f"Gesture: {gesture}", (10,40),
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

#     cv2.imshow("Hand Gesture Recognition", frame)

#     if cv2.waitKey(1) & 0xFF == 27: # ESC to exit
#         break

# cap.release()
# cv2.destroyAllWindows()

# gesture.py  (Python 3.10 venv)
# simulate_hand.py
import os, time, threading
import numpy as np
import mujoco
import mujoco.viewer as viewer
import cv2
import mediapipe as mp

# ===== MUJOCO SETUP =====
MENAGERIE = os.path.join(os.getcwd(), "mujoco_menagerie")
XML = os.path.join(MENAGERIE, "shadow_hand", "right_hand.xml")  # RIGHT hand file exists

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)

# Actual Shadow Hand MCP joints (first flex joint of each finger)
FINGER_JOINTS = [
    "rh_FFJ1",   # index finger base
    "rh_MFJ1",   # middle finger base
    "rh_RFJ1",   # ring finger base
    "rh_LFJ1",   # little finger base
    "rh_THJ1",   # thumb base
]

joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in FINGER_JOINTS]

OPEN = np.zeros(len(joint_ids))
CLOSE = np.array([0.8, 0.8, 0.8, 0.8, 0.7])

current_target = OPEN.copy()

def set_fingers(q):
    for qpos, jid in zip(q, joint_ids):
        addr = model.jnt_qposadr[jid]
        data.qpos[addr] = qpos
    mujoco.mj_forward(model, data)

# ===== MEDIAPIPE SETUP =====
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    model_complexity=0,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6,
)

TIP=[8, 12, 16, 20]; PIP=[6, 10, 14, 18]
def is_open_palm(lm):
    return sum(lm[tip].y < lm[pip].y for tip,pip in zip(TIP,PIP)) >= 3

def webcam_thread():
    global current_target
    cap = cv2.VideoCapture(0)
    last = "UNKNOWN"; stable = 0; state = "UNKNOWN"

    while True:
        ok, frame = cap.read()
        if not ok: break
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        label = "NO"
        if res.multi_hand_landmarks:
            lm = res.multi_hand_landmarks[0]
            label = "OPEN" if is_open_palm(lm.landmark) else "CLOSE"

            stable = stable + 1 if label == last else 0
            if stable > 3: 
                state = label
                current_target = OPEN if state=="OPEN" else CLOSE
            last = label

        cv2.putText(frame, f"Gesture:{state}", (10,40), 
                    cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        cv2.imshow("Hand Gesture", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break  

    cap.release()
    cv2.destroyAllWindows()

t = threading.Thread(target=webcam_thread, daemon=True)
t.start()

# ===== RUN SIM =====
with viewer.launch_passive(model, data) as v:
    set_fingers(OPEN)
    while v.is_running():
        set_fingers(current_target)
        v.sync()
