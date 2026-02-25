import tensorflow as pd
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image
import matplotlib.pyplot as plt
import os

# tải mô hình đã huấn luyện
num_classes = 6

model = load_model('traffic_sign_mobilenetv2_final.h5')

class_names = ["Cong trinh", "Toc do toi da 40km/h", "Toc do toi da 50km/h", 
               "Toc do toi da 60km/h", "Toc do toi da 80km/h", "Stop"]

def predict_and_show(image_path):
    # 2. Tiền xử lý ảnh (Phải giống hệt lúc train)
    img = image.load_img(image_path, target_size=(192, 192))
    img_array = image.img_to_array(img)
    img_array = np.expand_dims(img_array, axis=0)
    img_array = img_array / 255.0  # MobileNet thường chuẩn hóa về 0-1 hoặc -1 đến 1

    # 3. Dự đoán
    predictions = model.predict(img_array)
    predicted_class = np.argmax(predictions[0])
    
    # Kiểm tra xem có bị tràn index không
    if predicted_class < len(class_names):
        label = class_names[predicted_class]
    else:
        label = f"Unknown Class {predicted_class}"

    # 4. Hiển thị
    plt.imshow(img)
    plt.title(f'Dự đoán: {label}')
    plt.axis('off')
    plt.show()

# Chạy loop
folder_path = 'test_model'
for filename in os.listdir(folder_path):
    if filename.endswith(('png', 'jpg', 'jpeg')):
        predict_and_show(os.path.join(folder_path, filename))