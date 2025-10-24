import os
import shutil
import random

def split_dataset(image_folder, label_folder, output_folder, train_ratio=0.8):
    # 파일 목록 불러오기
    images = [f for f in os.listdir(image_folder) if f.endswith((".jpg", ".png"))]
    labels = [f for f in os.listdir(label_folder) if f.endswith(".txt")]

    # 이미지와 라벨 파일 짝 맞추기
    images.sort()
    labels.sort()
    
    # 데이터셋을 섞고 분할하기
    data = list(zip(images, labels))
    random.shuffle(data)
    train_size = int(len(data) * train_ratio)
    train_data = data[:train_size]
    valid_data = data[train_size:]

    # train, valid 폴더 생성
    train_image_folder = os.path.join(output_folder, "train", "images")
    train_label_folder = os.path.join(output_folder, "train", "labels")
    valid_image_folder = os.path.join(output_folder, "valid", "images")
    valid_label_folder = os.path.join(output_folder, "valid", "labels")

    os.makedirs(train_image_folder, exist_ok=True)
    os.makedirs(train_label_folder, exist_ok=True)
    os.makedirs(valid_image_folder, exist_ok=True)
    os.makedirs(valid_label_folder, exist_ok=True)

    # 파일 복사
    for image_file, label_file in train_data:
        shutil.copy(os.path.join(image_folder, image_file), train_image_folder)
        shutil.copy(os.path.join(label_folder, label_file), train_label_folder)
    
    for image_file, label_file in valid_data:
        shutil.copy(os.path.join(image_folder, image_file), valid_image_folder)
        shutil.copy(os.path.join(label_folder, label_file), valid_label_folder)

    print("Dataset split complete. Files saved to:", output_folder)

# 사용 예시
split_dataset("/home/heven/simul_ws/src/yolov5_ros/src/yolov5/data/image", "/home/heven/simul_ws/src/yolov5_ros/src/yolov5/data/label", "/home/heven/simul_ws/src/yolov5_ros/src/yolov5/data")
