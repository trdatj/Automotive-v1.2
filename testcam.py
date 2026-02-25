import time
import sys
import cv2
import numpy as np
import serial
from picamera2 import Picamera2

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        print("❌ Chưa cài thư viện AI! Chạy: pip3 install tensorflow --break-system-packages")
        sys.exit()

# --- 2. CẤU HÌNH ---
MODEL_PATH = "traffic_sign_mobilenetv2_quant.tflite" # <--- Tên file model .tflite của bro
LABEL_PATH = "labels.txt" # (Nếu có file nhãn riêng, không thì dùng Dict bên dưới)

FRAME_WIDTH = 640  # Để 640x480 cho nét
FRAME_HEIGHT = 480
CONFIDENCE_THRESHOLD = 0.5 # Độ tin cậy tối thiểu (50%)

# Định nghĩa các lệnh UART
CMD_DICT = {
    0: "CMD_CONSTRUCT",
    1: "CMD_LIMIT_40",
    2: "CMD_LIMIT_50",
    3: "CMD_LIMIT_60",
    4: "CMD_LIMIT_80",
    5: "CMD_STOP"
}

CLASS_NAMES = {
    0: "Cong trinh",
    1: "Limit 40",
    2: "Limit 50",
    3: "Limit 60",
    4: "Limit 80",
    5: "Stop"
}

# --- 3. KẾT NỐI UART ---
try:
    # Cổng thường là /dev/ttyUSB0 hoặc /dev/ttyAMA0
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("✅ UART Connected!")
except:
    print("⚠️ Warning: Không tìm thấy cổng UART (Chạy chế độ không gửi lệnh)")
    ser = None

def send_uart(cmd_idx, conf):
    if ser:
        msg = f"{CMD_DICT.get(cmd_idx, 'UNKNOWN')},{int(conf*100)}\n"
        ser.write(msg.encode('utf-8'))
        print(f"📡 Gửi: {msg.strip()}")

# --- 4. LOAD MODEL ---
print(f"🧠 Loading Model: {MODEL_PATH}...")
try:
    interpreter = tflite.Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    # Lấy kích thước đầu vào mà Model yêu cầu (thường là 320x320 hoặc 224x224)
    input_shape = input_details[0]['shape']
    input_h, input_w = input_shape[1], input_shape[2]
    print(f"ℹ️ Model input size: {input_w}x{input_h}")
    
except Exception as e:
    print(f"❌ Lỗi Load Model: {e}")
    sys.exit()

# --- 5. HÀM XỬ LÝ ẢNH & NHẬN DIỆN ---
def detect_objects(frame):
    # 1. Resize ảnh về kích thước Model yêu cầu
    img_resized = cv2.resize(frame, (input_w, input_h))
    
    # 2. Chuẩn bị ảnh đầu vào (Thường Model train bằng ảnh RGB)
    # Vì Picamera2 trả về RGB rồi nên không cần convert màu ở đây nữa
    input_data = np.expand_dims(img_resized, axis=0)

    # Nếu model train dạng float (0..1) thì chia 255, nếu uint8 (0..255) thì giữ nguyên
    if input_details[0]['dtype'] == np.float32:
        input_data = (np.float32(input_data) - 127.5) / 127.5 # Hoặc / 255.0 tùy model bro train
    else:
        input_data = np.uint8(input_data)

    # 3. Chạy suy luận (Inference)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # 4. Lấy kết quả
    # Tùy loại model (Detection hay Classification) mà output sẽ khác nhau
    # Code dưới đây giả định model Classification (Ra xác suất của từng lớp)
    output_data = interpreter.get_tensor(output_details[0]['index'])
    
    # Lấy class có điểm số cao nhất
    predictions = np.squeeze(output_data)
    class_id = np.argmax(predictions)
    confidence = predictions[class_id]
    
    # Nếu output model chưa phải dạng 0-1 mà là raw score, có thể cần softmax (tùy model)
    if confidence > 1.0: confidence = confidence / 255.0 

    return class_id, confidence

# --- 6. MAIN LOOP ---
def main():
    print("📷 Khởi tạo Camera...")
    picam2 = Picamera2()
    # Cấu hình camera giống hệt lúc test OK
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (FRAME_WIDTH, FRAME_HEIGHT)})
    picam2.configure(config)
    picam2.start()
    
    # Đợi cam ổn định
    time.sleep(2)
    print("🚀 Bắt đầu nhận diện! (Nhấn Ctrl+C để dừng)")

    try:
        while True:
            # Lấy ảnh từ camera
            frame = picam2.capture_array()
            
            # --- NHẬN DIỆN ---
            class_id, confidence = detect_objects(frame)

            # --- XỬ LÝ KẾT QUẢ ---
            label = "Unknown"
            color = (0, 0, 255) # Đỏ mặc định
            
            if confidence >= CONFIDENCE_THRESHOLD:
                label = f"{CLASS_NAMES.get(class_id, str(class_id))} ({confidence*100:.1f}%)"
                color = (0, 255, 0) # Xanh lá nếu nhận diện tốt
                
                # Gửi UART (chỉ gửi khi chắc chắn)
                send_uart(class_id, confidence)
            
            # --- HIỂN THỊ ---
            # Vẽ lên hình (Dùng copy để không ảnh hưởng luồng ảnh gốc)
            display_frame = frame.copy()
            
            # Vì OpenCV hiển thị hệ màu BGR, mà ảnh là RGB
            # Nếu bro thấy hiển thị trên màn hình bị sai màu thì uncomment dòng dưới:
            display_frame = cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR) 
            
            cv2.putText(display_frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("ADAS System", display_frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình...")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        if ser: ser.close()

if __name__ == "__main__":
    main()
