import numpy as np
import paddlelite.lite
import os
import cv2


def process_image(image_data, shape=224):
    """
    对图片进行预处理
    """
    img_mean = [0.485, 0.456, 0.406]
    img_std = [0.229, 0.224, 0.225]
    image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
    image_data = cv2.resize(image_data, (shape, shape))
    image_data = image_data.transpose((2, 0, 1)) / 255.0

    image_data = (image_data - np.array(img_mean).reshape(
        (3, 1, 1))) / np.array(img_std).reshape((3, 1, 1))
    image_data = image_data.reshape([1, 3, shape, shape]).astype('float32')
    return image_data
# 模型位置
paddle_model = "./model/MobileNet_big.nb"
config = paddlelite.lite.MobileConfig()
config.set_model_from_file(paddle_model)
predictor = paddlelite.lite.create_paddle_predictor(config)
input_tensor0 = predictor.get_input(0)
result_list = []
true_result = 0
Dict = {
    "0":"blue_false",
    "1":"blue_true",
    "2":"red_false",
    "3":"red_true"
    }
for class_num in range(4):
    img = cv2.imread(f"./测试图片/{Dict[str(class_num)]}/0.png")

    image_data = process_image(img)
    input_tensor0.from_numpy(image_data)
    predictor.run()
    output_tensor = predictor.get_output(0)
    output_tensor = output_tensor.numpy()
    e_x = np.exp(output_tensor.squeeze() - np.max(output_tensor.squeeze()))
    pro = e_x / e_x.sum()
    # result_list.append(np.argmax(pro))
    if np.argmax(pro) == class_num:
        print(f"{class_num}识别正确")
    else:
        print(f"{class_num}识别错误")
        
    #print("e_x=",e_x)
    #print("result:",np.argmax(pro))
    #print("confidence:",np.max(pro))






