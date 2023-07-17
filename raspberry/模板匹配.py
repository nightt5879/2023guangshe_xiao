import cv2
import numpy as np
import matplotlib.pyplot as plt
def calculate_similarity(image_path1, image_path2):
    # 加载图片
    image1 = cv2.imread(image_path1)
    image2 = cv2.imread(image_path2)

    # 确保图片大小相同
    if image1.shape != image2.shape:
        print("两张图片的大小不同，无法比较。")
        return
    else:
        # 转换为灰度图
        image1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        # 计算图片的直方图
        hist1 = cv2.calcHist([image1], [0], None, [256], [0,256])
        hist2 = cv2.calcHist([image2], [0], None, [256], [0,256])

        # 通过compareHist函数来计算相似度
        similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_BHATTACHARYYA)

        return similarity

def template_match(image_path, template_path):
    image = cv2.imread(image_path, 0)
    template = cv2.imread(template_path, 0)
    w, h = template.shape[::-1]

    res = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(image,top_left, bottom_right, 255, 2)

    plt.subplot(121),plt.imshow(res,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(image,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    plt.suptitle('cv2.TM_CCOEFF_NORMED')
    plt.show()


def orb_sim(img1, img2):
    # 初始化ORB检测器
    orb = cv2.ORB_create()

    # 检测关键点和计算描述符
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    # 定义匹配器并进行匹配
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # 按距离排序
    matches = sorted(matches, key=lambda x: x.distance)

    # 返回匹配特征点的数量
    return len(matches)



# 测试
image1_path = 'C:/Users/14692/Desktop/target.png'
image2_path = 'C:/Users/14692/Desktop/test_0.jpg'
img1 = cv2.imread(image1_path, 0)  # 原图像
img2 = cv2.imread(image2_path, 0)  # 旋转后的图像
similarity = calculate_similarity(image1_path, image2_path)
print('图片的0相似度是：', similarity)
print(orb_sim(img1, img2))
image2_path = 'C:/Users/14692/Desktop/test_1.jpg'
img2 = cv2.imread(image2_path, 0)  # 旋转后的图像
similarity = calculate_similarity(image1_path, image2_path)
print('图片的1相似度是：', similarity)
print(orb_sim(img1, img2))
# 使用方法：


# template_match(image2_path, image1_path)
