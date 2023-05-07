def get_rect_centers(quadrilaterals):
    centers = []
    for quad in quadrilaterals:
        x1, y1 = quad[0]
        x2, y2 = quad[2]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        centers.append((center_x, center_y))
    return centers



quadrilaterals = [
    [(0, 0), (2, 0), (2, 2), (0, 2)],
    [(4, 1), (7, 1), (4, 3), (7, 3)],
    [(2, 2), (5, 2), (5, 5), (2, 5)],
    [(3, 3), (5, 3), (5, 5), (3, 5)],
    [(4, 4), (8, 4), (8, 8), (4, 8)]]
centers = get_rect_centers(quadrilaterals)
print(centers)  # 输出四个矩形的中心点
centers = [(1, 1), (4, 2), (5, 6), (9, 4), (7, 1)]
left_top_index = min(enumerate(centers), key=lambda index_point: index_point[1][0] + index_point[1][1])[0]
left_bottom_index = min(enumerate(centers), key=lambda index_point: index_point[1][0] - index_point[1][1])[0]
right_top_index = min(enumerate(centers), key=lambda index_point: -index_point[1][0] + index_point[1][1])[0]
right_bottom_index = min(enumerate(centers), key=lambda index_point: -index_point[1][0] - index_point[1][1])[0]
left_top_point = min(quadrilaterals[left_top_index], key=lambda point: - point[0] - point[1])

print(left_top_index,left_bottom_index,right_top_index,right_bottom_index,left_top_point)
