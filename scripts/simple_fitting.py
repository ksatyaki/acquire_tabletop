#!/usr/bin/env python
import numpy
import numpy.linalg
import cv2
from numpy import linalg

dest_size_x = 640
dest_size_y = 480

def doTheMath(file1, file2):  
    matrixAleft = numpy.loadtxt(file1, dtype=numpy.float32, usecols=[1, 2])
    matrixAxtion = numpy.loadtxt(file2, dtype=numpy.float32)

    matrixA_x = numpy.array([[matrixAleft[0][0], (dest_size_x-matrixAleft[0][0])]])
    matrixb_x = numpy.array([ matrixAxtion[0][0] * dest_size_x ])
    for i in range(1, 54):
        matrixAi = numpy.array([[ matrixAleft[i][0], (dest_size_x-matrixAleft[i][0]) ]])
        matrixA_x = numpy.append(matrixA_x, matrixAi, axis=0)
        matrixb_x = numpy.append(matrixb_x, [ matrixAxtion[i][0] * dest_size_x ])

    ATb_x = numpy.dot (numpy.transpose(matrixA_x), matrixb_x)
    ATAinverse_x = linalg.inv( numpy.dot( numpy.transpose(matrixA_x), matrixA_x ) )
    x = numpy.dot (ATAinverse_x, ATb_x)

    print '[horizon_x, start_x] is', x

    matrixA_y = numpy.array([[matrixAleft[0][1], (dest_size_y-matrixAleft[0][1])]])
    matrixb_y = numpy.array([ matrixAxtion[0][1] * dest_size_y ])
    for i in range(1, 54):
        matrixAi = numpy.array([[ matrixAleft[i][1], (dest_size_y-matrixAleft[i][1]) ]])
        matrixA_y = numpy.append(matrixA_y, matrixAi, axis=0)
        matrixb_y = numpy.append(matrixb_y, [ matrixAxtion[i][1] * dest_size_y ])

    ATb_y = numpy.dot (numpy.transpose(matrixA_y), matrixb_y)
    ATAinverse_y = linalg.inv( numpy.dot( numpy.transpose(matrixA_y), matrixA_y ) )
    y = numpy.dot (ATAinverse_y, ATb_y)

    print '[horizon_y, start_y] is', y
    
if __name__ == '__main__':
    doTheMath("/home/ace/Desktop/right_camera.txt", "/home/ace/Desktop/xtion_camera.txt")