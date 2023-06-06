
import cv2

def resize_and_save(image_path, output_path):
    # 读取图像
    img = cv2.imread(image_path)

    # 检查是否正确读取图像
    if img is None:
        print(f"Error: Unable to open image file: {image_path}")
        return

    # 调整图像大小
    resized_img = cv2.resize(img, (320, 240))

    # 保存调整后的图像
    cv2.imwrite(output_path, resized_img)

# 使用方式
resize_and_save('path_to_input_image', 'path_to_output_image')
"""
if（第一位等于 0x05 && 第二位等于）
{
    左转的GPIO
    while(1)
    {
        6050
        if （大于一定值）
        {
            break;
        }
    }
}
else if （第一位等于 0x05 && 第二位等于）
    {
        右转的GPIO
    while(1)
    {
        6050
        if （大于一定值）
        {
            break;
        }
    }
    }

"""
