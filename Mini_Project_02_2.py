## Mini Project 2
import cv2 as cv
import numpy as np
img = cv.imread('E:\Machine Vision\XLA_01FIE\lena_color.png')

# Lấy kích thước ảnh
h, w, channel = img.shape

# Khai báo 3 biến để chứa hình 3 kênh R-G-B
red = np.zeros((h, w, 3), np.uint8)
green = np.zeros((h, w, 3), np.uint8)
blue = np.zeros((h, w, 3), np.uint8)

# Ban đầu set zero cho tất cả các điểm ảnh có trong 3 kênh trong mỗi hình
red[:] = [0, 0, 0]
green[:] = [0, 0, 0]
blue[:] = [0, 0, 0]


# Mỗi hình là 1 ma trận 2 chiều nên sẽ dùng 2 vòng for
# để đọc hết các điểm ảnh (pixel) có trong hình
for x in range(w):
    for y in range(h):
        # Lấy giá trị điểm ảnh tại vị trí (x, y)
        R = img[y, x, 2]
        G = img[y, x, 1]
        B = img[y, x, 0]

        # Thiết lập màu cho các kênh
        red[y, x, 2] = R
        green[y, x, 1] = G
        blue[y, x, 0] = B

# Hiển thị hình dùng thư viện OpenCV
cv.imshow('Original', img)
cv.imshow('Blue', blue)
cv.imshow('Green', green)
cv.imshow('Red', red)
cv.waitKey(0)
