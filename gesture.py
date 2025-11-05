import cv2
import mediapipe as mp


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

cap = cv2.VideoCapture(0)

def is_fist_open(landmarks):
    # Tip landmarks for fingers (index, middle, ring, pinky)
    finger_tips = [8, 12, 16, 20]  
    open_fingers = 0
    
    for tip in finger_tips:
        if landmarks[tip].y < landmarks[tip - 2].y:  # Tip above PIP joint = finger extended
            open_fingers += 1
    
    # If 3 or more fingers open = open palm, else fist
    return open_fingers >= 3

while True:
    success, frame = cap.read()
    frame = cv2.flip(frame, 1)  # Mirror view
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    gesture = "NO HAND"

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            if is_fist_open(hand_landmarks.landmark):
                gesture = "OPEN HAND"
                # Simulated robot action
                print("ROBOT ACTION → OPEN HAND")
            else:
                gesture = "CLOSED FIST"
                print("ROBOT ACTION → CLOSE HAND")

    cv2.putText(frame, f"Gesture: {gesture}", (10,40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("Hand Gesture Recognition", frame)

    if cv2.waitKey(1) & 0xFF == 27: # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
