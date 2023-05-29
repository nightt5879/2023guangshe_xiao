from main_program.util.mine_classify import MinesClassifier
import cv2

mine_dict = {
    0: "蓝色三角",
    1: "蓝色圆形",
    2: "红色圆形",
    3: "红色三角",
}
# 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
mine_classifier = MinesClassifier(paddle_model="./main_program/model/MobileNet_small.nb") #
cap = cv2.VideoCapture(1)
while True:
    success, img = cap.read()
    if success:
        cv2.imshow("img",img)
        result, pro = mine_classifier.recognize_img(img)
        print(mine_dict[result], pro)
        cv2.waitKey(1)