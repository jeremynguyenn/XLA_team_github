## Mini Project 2
import cv2 as cv
import numpy as np

# Đọc ảnh màu cô gái Lena và lưu vào biến img
img = cv.imread('E:\Machine Vision\XLA_01FIE\lena_color.png', 1)
R = img[:,:,2]
G = img[:,:,1]
B = img[:,:,0]

# Khai báo 3 biến R-G-B chứa các trị
cv.imshow('Original', img)
cv.imshow('Blue', B)
cv.imshow('Green', G)
cv.imshow('Red', R)
cv.waitKey(0)