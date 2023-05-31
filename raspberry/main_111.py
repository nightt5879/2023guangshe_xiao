from main_program.util.mine_classify import MinesClassifier
import cv2



def classify_treasure(team = "red"):
    mine_dict = {
        0: "蓝色三角",
        1: "蓝色圆形",
        2: "红色圆形",
        3: "红色三角",
    }
    mine_classifier = MinesClassifier(paddle_model="./main_program/model/MobileNet_big.nb")  #
    for i in range (5):
        try:  # 多次重复调用有几率出错
            cap = cv2.VideoCapture(1)
            cap.set(4, 240)
            cap.set(3, 320)
            cam = "open"
            break
        except:
            cam = "cannot open"
            pass
    if cam == "cannot open":  # 五次都打不开直接return假宝藏
        print("Cannot open camera")
        class_of = "fake"
        return class_of
    list_of_treasure = [0,0,0,0,0] # 0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角 4:无类别
    while True:
        success, img = cap.read()
        if success:
            cv2.imshow("img", img)
            result, pro = mine_classifier.recognize_img(img)
            # print(pro)
            if mine_dict[result] == "蓝色三角" and pro[0] > 0.42:
                print("蓝色三角")
                list_of_treasure[0] += 1
            elif mine_dict[result] == "蓝色圆形" and pro[1] > 0.42:
                print("蓝色圆形")
                list_of_treasure[1] += 1
            elif mine_dict[result] == "红色圆形" and pro[2] > 0.42:
                print("红色圆形")
                list_of_treasure[2] += 1
            elif mine_dict[result] == "红色三角" and pro[3] > 0.42:
                print("红色三角")
                list_of_treasure[3] += 1
            else:
                list_of_treasure[4] += 1
            # sum of the list_of_treasure
            if list_of_treasure[0] > 10 or list_of_treasure[1] > 10 or \
               list_of_treasure[2] > 10 or list_of_treasure[3] > 10 or list_of_treasure[4] >10:
                break
            cv2.waitKey(1)
    print(list_of_treasure)
    # 分辨宝藏是真的还是假的,0:蓝色三角 1:蓝色圆形 2:红色圆形 3:红色三角
    if team == "red" and list_of_treasure[2] > 10:
        class_of = "fake"
    elif team == "red" and list_of_treasure[3] > 10:
        class_of = "true"
    elif team == "blue" and list_of_treasure [0] > 10:
        class_of = "fake"
    elif team == "blue" and list_of_treasure [1] > 10:
        class_of = "true"
    else:
        print("no treasure")
        class_of = "fake"
    return class_of

a = classify_treasure("blue")
print(a)
# mine_dict = {
#     0: "蓝色三角",
#     1: "蓝色圆形",
#     2: "红色圆形",
#     3: "红色三角",
# }
# # 初始化宝藏分类器，用法是：result = mine_classifier.recognize_img(img)。其中result的结果是0、1、2、3分别对应宝藏的四个类别。
# mine_classifier = MinesClassifier(paddle_model="./main_program/model/MobileNet_big.nb") #
# cap = cv2.VideoCapture(1)
# while True:
#     success, img = cap.read()
#     if success:
#         cv2.imshow("img",img)
#         result, pro = mine_classifier.recognize_img(img)
#         print(mine_dict[result], pro)
#         cv2.waitKey(1)