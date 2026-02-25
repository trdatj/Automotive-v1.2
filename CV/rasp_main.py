from picamera2 import Picamera2
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
import serial
import time

FRAME_WIDTH = 320
FRAME_HEIGHT = 180
UART_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

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

CONFIDENCE_THRESHOLD = 0.85
MIN_SIGN_AREA = 1500
ASPECT_RATIO = (0.7, 1.5)
RED_PIXEL_RATIO = 0.3
MIN_CIRCULARITY = 0.6

SIGN_HOLD_TIME = 2  # giây
DISPLAY_GUI = True

try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    print(f"UART Initialized on {UART_PORT}")
except Exception as e:
    print(f"UART Error: {e}")
    ser = None

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (FRAME_WIDTH, FRAME_HEIGHT)}, controls={"FrameDurationLimits": (50000, 50000)}))
picam2.start()
time.sleep(1)

print("Loading Model...")
interpreter = tflite.Interpreter(model_path="traffic_sign_mobilenetv2_quant.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# ========== 7. Hàm hỗ trợ ==========
def send_uart(cmd_key, confidence):
    if ser is None: return
    try:
        cmd_str = CMD_DICT.get(cmd_key, "CMD_UNKNOWN")
        packet = f"{cmd_str},{int(confidence * 100)}\n"
        ser.write(packet.encode('utf-8'))
        print(f"Sent: {packet.strip()}")
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
    
    # Check tỷ lệ pixel đỏ
    roi = frame[y:y+h, x:x+w]
    mask = advanced_color_filter(roi)
    if (np.sum(mask > 0) / mask.size) < RED_PIXEL_RATIO: return False
    
    # Check độ tròn (chỉ áp dụng cho biển STOP - Class 5)
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
        # cắt ảnh và tiền xử lý
        x, y, w, h = cv2.boundingRect(cnt)
        roi = frame[y:y+h, x:x+w]
        img = cv2.resize(roi, (192, 192))
        img_array = np.array(img, dtype=np.float32) / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        interpreter.set_tensor(input_details[0]['index'], img_array)
        interpreter.invoke()
        predictions = interpreter.get_tensor(output_details[0]['index'])
        class_id = np.argmax(predictions)
        confidence = np.max(predictions)

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

try:
    print("System Ready (Traffic Sign Only mode)...")
    while True:
        frame = picam2.capture_array()
        current_time = time.time()

        detect_signs(frame, current_time)

        if DISPLAY_GUI:
            cv2.imshow("Traffic Sign Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break
finally:
    if DISPLAY_GUI:
        cv2.destroyAllWindows()
    if ser: ser.close()
    picam2.stop()
    cv2.destroyAllWindows()