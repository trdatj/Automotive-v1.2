import os
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.preprocessing.image import load_img
import numpy as np

# Đường dẫn đến thư mục chứa ảnh
base_dir = r"datasets"

sign_types = ["tao anh"]

# Số lượng ảnh mới muốn tạo ra từ mỗi ảnh gốc
augmentation_factor = 25

# Khởi tạo ImageDataGenerator với các tham số tương tự như trước
aug = ImageDataGenerator(
	rescale=1./255,
    rotation_range=15,      # Xoay ảnh ngẫu nhiên ±15 độ
    width_shift_range=0.1,  # Dịch chuyển ngang 10%
    height_shift_range=0.1, # Dịch chuyển dọc 10%
    shear_range=0.1,        # Biến dạng trượt
    zoom_range=0.1,         # Phóng to/thu nhỏ 10%
    horizontal_flip=False,  # Không lật ngang (biển báo thường đối xứng)
    fill_mode='nearest',
    brightness_range=[0.9,1.1]  # Điều chỉnh độ sáng
    )

# Lặp qua từng biển báo
for animal_type in sign_types:
    print(f"[INFO] Generating images for {animal_type}...")
    
    # Tạo đường dẫn đầy đủ đến thư mục chứa ảnh
    animal_dir = os.path.join(base_dir, animal_type)
    
    # Đảm bảo rằng thư mục tồn tại
    if not os.path.exists(animal_dir):
        print(f"[ERROR] Directory {animal_dir} not found.")
        continue
    
    # Lặp qua mỗi file ảnh trong thư mục của loại biển báo hiện tại
    for root, dirs, files in os.walk(animal_dir):
        for file in files:
            # Load ảnh và chuyển đổi thành mảng NumPy
            image_path = os.path.join(root, file)
            image = load_img(image_path)
            image = img_to_array(image)
            image = np.expand_dims(image, axis=0)
            
            # Tạo thư mục để lưu ảnh được tạo ra từ ảnh gốc
            save_dir = os.path.join(root, "generated")
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # Tạo generator dữ liệu từ ảnh hiện tại
            imageGen = aug.flow(image, batch_size=1, save_to_dir=save_dir, save_prefix=file.split('.')[0], save_format="jpg")
            
            # Tạo thêm ảnh đến khi đạt đến augmentation_factor
            count = 0
            for _ in imageGen:
                count += 1
                if count == augmentation_factor:
                    break