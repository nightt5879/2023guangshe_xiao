# 定义一个包含坐标的列表
points = [(1, 2), (3, 4), (5, 1), (-1, 3)]

# 使用Lambda函数和enumerate()函数计算x+y值最小的坐标在原列表中的索引
min_index = min(enumerate(points), key=lambda index_point: index_point[1][0] + index_point[1][1])[0]

# 输出结果
print(min_index)  # 输出结果为 3
