import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D, Dropout #giảm thiểu overfit
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
import matplotlib.pyplot as plt
import numpy as np
from sklearn.utils.class_weight import compute_class_weight

# cấu hình cho rasp
tf.config.threading.set_inter_op_parallelism_threads(2)  # Giới hạn thread
tf.config.threading.set_intra_op_parallelism_threads(2)

TRAIN_DIR = "datasets/train_data"
VAL_DIR = "datasets/val_data"
IMG_SIZE = (192, 192)  # Giảm kích thước để tăng tốc độ
BATCH_SIZE = 16  # Giảm batch size cho Pi

train_datagen = ImageDataGenerator(
    rescale=1./255,
    rotation_range=15,
    width_shift_range=0.1,
    height_shift_range=0.1,
    zoom_range=0.1,
    brightness_range=[0.9, 1.1],
    fill_mode='nearest'
)

val_datagen = ImageDataGenerator(rescale=1./255)

# Load dữ liệu
train_generator = train_datagen.flow_from_directory(
    TRAIN_DIR,
    target_size=IMG_SIZE,
    batch_size=BATCH_SIZE,
    class_mode="categorical",
    shuffle=True
)
val_generator = val_datagen.flow_from_directory(
    VAL_DIR,
    target_size=IMG_SIZE,
    batch_size=BATCH_SIZE,
    class_mode="categorical",
    shuffle=False
)

NUM_CLASSES = len(train_generator.class_indices)
print(f"Số lớp biển báo: {NUM_CLASSES}")

# tính toán class weights để xử lý mất cân bằng dữ liệu
class_weights = compute_class_weight('balanced',
                                   classes=np.unique(train_generator.classes),
                                   y=train_generator.classes)
class_weights = dict(enumerate(class_weights))

# xây dựng mô hình
def create_model():
    base_model = MobileNetV2(
        input_shape=(IMG_SIZE[0], IMG_SIZE[1], 3),
        include_top=False,
        weights="imagenet",
        alpha=0.5  # Nhẹ hơn bản gốc
    )
    
    # Freeze các layer đầu
    for layer in base_model.layers[:50]:
        layer.trainable = False
    
    # giảm thiểu overfit
    x = base_model.output
    x = GlobalAveragePooling2D()(x)
    x = Dense(128, activation='relu')(x)
    x = Dropout(0.3)(x)
    predictions = Dense(NUM_CLASSES, activation='softmax')(x)
    
    model = Model(inputs=base_model.input, outputs=predictions)
    
    return model

model = create_model()

# Callbacks để tối ưu quá trình training
callbacks = [
    EarlyStopping(monitor='val_loss', patience=8, restore_best_weights=True),
    ModelCheckpoint('best_model.h5', monitor='val_accuracy', save_best_only=True),
    ReduceLROnPlateau(monitor='val_loss', factor=0.2, patience=5, min_lr=1e-6)
]

model.compile(
    optimizer=Adam(learning_rate=0.0005),  # Learning rate nhỏ hơn
    loss="categorical_crossentropy",
    metrics=["accuracy"]
)

EPOCHS = 30

history = model.fit(
    train_generator,
    steps_per_epoch=max(1, train_generator.samples // BATCH_SIZE),
    validation_data=val_generator,
    validation_steps=max(1, val_generator.samples // BATCH_SIZE),
    epochs=EPOCHS,
    callbacks=callbacks,
    class_weight=class_weights
)

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # quantization
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]  # Tương thích Pi
tflite_model = converter.convert()

with open('traffic_sign_mobilenetv2_quant.tflite', 'wb') as f:
    f.write(tflite_model)

plt.figure(figsize=(12, 4))

plt.subplot(1, 2, 1)
plt.plot(history.history['accuracy'], label='Train Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(history.history['loss'], label='Train Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend()

plt.tight_layout()
plt.savefig('training_results.png')
plt.show()

# 9. Benchmark trên tập validation
val_loss, val_acc = model.evaluate(val_generator)
print(f"\nValidation Accuracy: {val_acc:.4f}")
print(f"Validation Loss: {val_loss:.4f}")

# 10. Lưu model cuối cùng (không quantized)
model.save("traffic_sign_mobilenetv2_final.h5")