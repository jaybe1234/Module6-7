import cv2 as cv
import numpy as np
import time
import serial
import struct
from Control.coordinate import coordinate
from scipy.spatial import distance as dist
from math import atan2,pi,sqrt

def midpoint(A,B):
    return(int((A[0]+B[0])*0.5),int((A[1]+B[1])*0.5))

ser = serial.Serial()
ser.port = 'COM9'
ser.baudrate=9600
ser.timeout = 1
ser.rts = 0
ser.dtr = 0
ser.open()

# cap = cv.VideoCapture(1)


print('check1')
A = coordinate(11,ser)
print('check2')
B = coordinate(22,ser)
#
#A.setZero()
#B.setZero()
#A.down(1)
# B.rotate(90)
# A.move(14.47)
# time.sleep(1)
# B.move(-11.65)
# A.grab()
# A.downdro()
B.rotate(270)
# A.move(32.16)
# B.move(1.A
#A.put(2)

