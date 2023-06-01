import cv2

import cv2

def resize_image(image_path, output_path, width=320, height=240):
    # 加载图像
    img = cv2.imread(image_path)
    if img is None:
        print("Could not open or find the image")
        return

    # 改变图像的尺寸
    resized_img = cv2.resize(img, (width, height))

    # 保存图像
    cv2.imwrite(output_path, resized_img)


    return resized_img

# img = cv2.imread("/home/pi/Desktop/guangshe2023/main_program/util/success.jpg")
img = resize_image("/home/pi/Desktop/guangshe2023/main_program/util/success.jpg","/home/pi/Desktop/guangshe2023/main_program/success.jpg")
cv2.imshow("img", img)
cv2.waitKey(0)
