## Mini Project 2
import cv2 as cv
import numpy as np
img = cv.imread('E:\Machine Vision\XLA_01FIE\lena_color.png')
blank = np.zeros(img.shape[:2], dtype='uint8')

b,g,r = cv.split(img)
blue = cv.merge([b,blank, blank])
green = cv.merge([blank, g, blank])
red = cv.merge([blank,blank,r])

cv.imshow('Original', img)
cv.imshow('Blue', blue)
cv.imshow('Green', green)
cv.imshow('Red', red)
cv.waitKey(0)