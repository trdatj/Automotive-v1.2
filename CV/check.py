import os

# Đường dẫn đến thư mục chứa ảnh
image_dir = r"datasets\train_data\speed_limit_80"

# Định dạng tệp tin ảnh được chấp nhận
accepted_extensions = [".jpg", ".jpeg", ".png", ".bmp"]

# Đếm số lượng tệp tin ảnh
image_count = sum(
    1 for file_name in os.listdir(image_dir)
    if os.path.isfile(os.path.join(image_dir, file_name))
    and any(file_name.lower().endswith(ext) for ext in accepted_extensions)
)

print(f"Số lượng tệp tin ảnh: {image_count}")