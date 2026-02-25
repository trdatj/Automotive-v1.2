import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image

# Cấu hình nâng cao
CONFIDENCE_THRESHOLD = 0.96
MIN_SIGN_AREA = 1500
ASPECT_RATIO = (0.7, 1.5)
RED_PIXEL_RATIO = 0.25
MIN_CIRCULARITY = 0.6

# Load model
model = load_model("traffic_sign_mobilenetv2_final.h5")
class_names = ['cong_trinh', 'speed_limit_40', 'speed_limit_50', 'speed_limit_60', 'speed_limit_80', 'stop']

def calculate_circularity(contour):
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0
    area = cv2.contourArea(contour)
    return (4 * np.pi * area) / (perimeter ** 2)

def advanced_color_filter(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([165, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    
    return red_mask

def is_valid_sign(contour, frame, predicted_class=None):
    area = cv2.contourArea(contour)
    if area < MIN_SIGN_AREA:
        return False
    
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = w / float(h)
    if not ASPECT_RATIO[0] <= aspect_ratio <= ASPECT_RATIO[1]:
        return False
    
    roi = frame[y:y+h, x:x+w]
    red_mask = advanced_color_filter(roi)
    red_ratio = np.sum(red_mask > 0) / float(red_mask.size)
    if red_ratio < RED_PIXEL_RATIO:
        return False
    
    circularity = calculate_circularity(contour)
    if predicted_class is not None and class_names[predicted_class] == 'stop' and circularity < MIN_CIRCULARITY:
        return False
    
    return True

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    red_mask = advanced_color_filter(frame)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    sign_detected = False
    for cnt in contours:
        if not is_valid_sign(cnt, frame):
            continue
            
        x, y, w, h = cv2.boundingRect(cnt)
        roi = frame[y:y+h, x:x+w]
        
        img = cv2.resize(roi, (192, 192))
        img_array = image.img_to_array(img)
        img_array = np.expand_dims(img_array, axis=0) / 255.0
        
        predictions = model.predict(img_array)
        confidence = np.max(predictions[0])
        predicted_class = np.argmax(predictions[0])
        
        # Kiểm tra lại với predicted_class
        if confidence > CONFIDENCE_THRESHOLD and is_valid_sign(cnt, frame, predicted_class):
            label = f"{class_names[predicted_class]} ({confidence:.2f})"
            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            sign_detected = True

    if not sign_detected:
        cv2.putText(frame, "Khong phat hien bien bao", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.imshow('Nhan dang bien bao', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()