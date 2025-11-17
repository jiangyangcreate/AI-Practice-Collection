import os
import cv2
from sklearn.neighbors import KNeighborsClassifier
import numpy as np



class DataProcessor:
    @staticmethod
    def read_video(video_path, save_path):
        cap = cv2.VideoCapture(video_path)
        index = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
        
            index += 1
            cv2.imwrite(f"{save_path}/{index}.jpg", frame)
        cap.release()
    
    @staticmethod
    def change_img_to_array(img_path):
        img = cv2.imread(img_path)
        # 如果图像的尺寸大于256*256，则需要进行缩小，否则不缩放
        w,h = img.shape[:2]

        if w*h > 256*256:
            img = cv2.resize(img, (256, 256))
        # 将图像转换为灰度图
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img.reshape(1, -1)
        return img

source_path = "mp4"
data_path = "data"
for i in os.listdir(source_path):
    if i.endswith(".mp4"):
        tag = i.split(".")[0]
        full_path = os.path.join(source_path, i)
        tag_path = os.path.join(data_path, tag)
        os.makedirs(tag_path, exist_ok=True)
        DataProcessor.read_video(full_path, tag_path)

tag_dict = {'paper':0, 'water':1,"table":2}

# 创建一些示例数据
X = []  # 特征
y = []  # 目标标签

for tag in os.listdir(data_path):

    level_2_filedir = os.path.join(data_path, tag)

    for file in os.listdir(level_2_filedir):

        img_path = os.path.join(level_2_filedir, file)

        img = DataProcessor.change_img_to_array(img_path)
        X.append(img)
        y.append(tag_dict[tag])


X = np.array(X)
y = np.array(y)
X = X.reshape(X.shape[0], -1)
print(X.shape)
print(y.shape)


# 创建K-最近邻分类器
k = 3  # 选择K的值
model = KNeighborsClassifier(n_neighbors=k).fit(X, y)

# 预测新数据点
path = '1.jpg'
new_data_point = DataProcessor.change_img_to_array(path)

# .predicts()方法返回一个数组，数组中包含了预测的类别
predicted_class = model.predict(new_data_point)

if predicted_class == 0:
    print("预测类别: paper")
elif predicted_class == 1:
    print("预测类别: water")
elif predicted_class == 2:
    print("预测类别: table")
# 等价于
# classes = "paper" if predicted_class == 0 else "water"
# print("预测类别: ", classes)