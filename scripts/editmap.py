from cgitb import grey
import rospy
import cv2
import numpy as np
import sys


def editmap():
    #im = cv2.imread("/home/teun/Downloads/map.pgm",-1)

    print(sys.argv[0])

    im = cv2.imread(sys.argv[1],-1)

    grey = im[0,0]
    

    mask = cv2.inRange(im,200,210)

    #cv2.imshow("input", im)

    cv2.imshow("mask", mask)

    edges = cv2.Canny(image=mask, threshold1=100, threshold2=200)

    mask1 = np.zeros(im.shape[:2], dtype="uint8")


    #masked = cv2.bitwise_or(im, mask1, mask=edges)

    #cv2.imshow("edges", edges)

    #cv2.imshow("mask1",mask1)

    #cv2.imshow("masked",masked)

    for y in range(0,im.shape[0]):
        for x in range(0,im.shape[1]):
            if (edges[y,x] == 1):
                mask1[y,x] = 0
            else:
                mask1[y,x] = im[y,x]

    cv2.imshow("newmap",mask1)
    cv2.waitKey(0)

    
    




editmap()