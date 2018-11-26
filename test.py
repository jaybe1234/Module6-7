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
A = coordinate(111,ser)
print('check2')
B = coordinate(222,ser)
# # A.grab()
# A.setZero()
# B.setZero()
# A.setZ()
# B.rotate(0)
# B.ro  tate(90)
# A.put(3)
# A.grab()
# A.move(14.47)
# time.sleep(1)
# B.move(-11.65)
# A.down(2)
#A.put(2)
# A.grab()
# A.setZ()
# A.grab()
# A.downdrop(4.5)
# B.rotate(4.5)
#A.move(12.9)
#B.move(-)
# A.downdrop(9)
# B.rotate(180)
# A.move(36.67)
# B.move(-1.76)
# B.rotate(90)
# A.grab()
# A.setZ()
# B.rotate(0)# A.setZ()
# B.rotate(18)
# A.setZ()#
# A.grab()
A.put(3)
#   # A.move(20)
#A.setZ()
# A.setZ()
# B.setZero()
#A.move(23.58)
#B.move(-3.26)
#A.grab()
#A.setZ()
# A.put(3)
#B.rotate(60)
#A.setZ()
# A.down(3)
#B.rotate(180)