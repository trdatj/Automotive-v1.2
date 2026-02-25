import time
import sys
import os
import cv2
import numpy as np
import serial
from picamera2 import Picamera2

try:
    import tensorflow.lite as tflite
except ImportError:
    try:
        import tflite_runtime.interpreter as tflite
    except ImportError:
        print("Lỗi: Chưa cài thư viện TensorFlow!")
        sys.exit()

MODEL_NAME = "traffic_sign_mobilenetv2_quant.tflite" 

FRAME_WIDTH = 320 
FRAME_HEIGHT = 180
UART_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

CONFIDENCE_THRESHOLD = 0.85
MIN_SIGN_AREA = 1500
ASPECT_RATIO = (0.7, 1.5)
RED_PIXEL_RATIO = 0.3
MIN_CIRCULARITY = 0.6
SIGN_HOLD_TIME = 2
DISPLAY_GUI = True

CLASS_NAMES = {
    0: "cong_trinh",
    1: "speed_limit_40",
    2: "speed_limit_50",
    3: "speed_limit_60",
    4: "speed_limit_80",
    5: "stop"
}

CMD_DICT = {
    0: "CMD_CONSTRUCT",
    1: "CMD_LIMIT_40",
    2: "CMD_LIMIT_50",
    3: "CMD_LIMIT_60",
    4: "CMD_LIMIT_80",
    5: "CMD_STOP"
}


try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    print(f"UART Initialized on {UART_PORT}")
except Exception as e:
    print(f"UART Error: {e} (Chạy chế độ không gửi lệnh)")
    ser = None

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(CURRENT_DIR, MODEL_NAME)

if not os.path.exists(MODEL_PATH):
    backup_name = "traffic_sign_mobilenetv2_quant.tflite"
    MODEL_PATH = os.path.join(CURRENT_DIR, backup_name)
    if not os.path.exists(MODEL_PATH):
        print(f"LỖI: Không tìm thấy file model '{MODEL_NAME}' hoặc '{backup_name}'")
        sys.exit()

print(f"Loading Model: {os.path.basename(MODEL_PATH)}")
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

model_h, model_w = input_details[0]['shape'][1], input_details[0]['shape'][2]

def send_uart(cmd_key, confidence):
    if ser is None: return
    try:
        cmd_str = CMD_DICT.get(cmd_key, "CMD_UNKNOWN")
        packet = f"{cmd_str},{int(confidence * 100)}\n"
        ser.write(packet.encode('utf-8'))
        print(f"📡 Sent: {packet.strip()}")
    except Exception as e:
        print(f"Send Error: {e}")

def advanced_color_filter(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Dải đỏ 1
    lower1 = np.array([0, 100, 100])
    upper1 = np.array([10, 255, 255])
    # Dải đỏ 2
    lower2 = np.array([160, 100, 100])
    upper2 = np.array([180, 255, 255])
    
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
    
    # Khử nhiễu
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def calculate_circularity(contour):
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return 0
    area = cv2.contourArea(contour)
    return (4 * np.pi * area) / (perimeter ** 2)

def is_valid_sign(contour, frame, predicted_class=None):
    if cv2.contourArea(contour) < MIN_SIGN_AREA: return False
    x, y, w, h = cv2.boundingRect(contour)
    
    ratio = w / float(h)
    if not ASPECT_RATIO[0] <= ratio <= ASPECT_RATIO[1]: return False
    
    roi = frame[y:y+h, x:x+w]
    mask = advanced_color_filter(roi)
    if (np.sum(mask > 0) / mask.size) < RED_PIXEL_RATIO: return False
    
    if predicted_class == 5:
        if calculate_circularity(contour) < MIN_CIRCULARITY: return False
        
    return True

last_sign_id = None
last_sign_time = 0

def detect_signs(frame, current_time):
    global last_sign_id, last_sign_time

    mask = advanced_color_filter(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if not is_valid_sign(cnt, frame):
            continue
            
        x, y, w, h = cv2.boundingRect(cnt)
        roi = frame[y:y+h, x:x+w]
        
        img = cv2.resize(roi, (model_w, model_h))
        
        img_array = np.array(img, dtype=np.float32)
        
        img_array = img_array / 255.0
        
        img_array = np.expand_dims(img_array, axis=0)

        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        predictions = interpreter.get_tensor(output_details[0]['index'])
        
        class_id = np.argmax(predictions)
        confidence = np.max(predictions)

        # Logic xử lý kết quả
        if confidence > CONFIDENCE_THRESHOLD and is_valid_sign(cnt, frame, class_id):
            if class_id != last_sign_id or (current_time - last_sign_time) > SIGN_HOLD_TIME:
                send_uart(class_id, confidence)
                last_sign_id = class_id
                last_sign_time = current_time

            if DISPLAY_GUI:
                label = f"{CLASS_NAMES.get(class_id)} {confidence:.2f}"
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            return True
    return False

def main():
    print("Khởi tạo Picamera2...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (FRAME_WIDTH, FRAME_HEIGHT)})
    picam2.configure(config)
    picam2.start()
    
    time.sleep(2) # Đợi cam ổn định
    print("System Ready (Traffic Sign Mode)...")

    try:
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            current_time = time.time()

            detect_signs(frame_bgr, current_time)

            if DISPLAY_GUI:
                cv2.imshow("Traffic Sign Detection", frame_bgr)
                if cv2.waitKey(1) == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        if ser: ser.close()

if __name__ == "__main__":
    main()