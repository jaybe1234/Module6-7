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
ser.port = 'COM19'
ser.baudrate=9600
ser.timeout = 1
ser.rts = 0
ser.dtr = 0
ser.open()
# x = ser.read()
cap = cv.VideoCapture(0)

print('check1')
A = coordinate(11,ser)
print('check2')
B = coordinate(22,ser)
print('check3')


A.setZero()
A.setZero()
B.setZero()
B.setZero()

#verify 1 setzero xy
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('setzero finish')
            break
B.rotate(0)
ser.reset_output_buffer()
B.put(3)

A.setZ()
#verify 2 setzero z
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('setzero_z finish')
            break

# time.sleep(5)
t = time.clock()

while(time.clock()-t<=2):
    ret, frame = cap.read()
    #cv.imshow('frame', frame)
frame[0:480, 417:640] = 0
frame[0:480, 0:313] = 0
# frame = frame[121:357,227:372]
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
blur = cv.GaussianBlur(gray, (15, 15), 0)
sharpened = cv.addWeighted(blur, -0.3, gray, 1, 0)
ret, thres1 = cv.threshold(blur, 92, 255,0)
(_, cnts, _) = cv.findContours(thres1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
for contour in cnts:
    if cv.contourArea(contour) < 3163 or cv.contourArea(contour) > 5581:
        continue
    rect = cv.minAreaRect(contour)
    (center, _, _) = rect
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(frame, [box], 0, (255, 0, 0), 2)
    (tl, tr, br, bl) = box
    for i in range(len(box)):
        cv.circle(frame, (int(box[i][0]), int(box[i][1])), 1, (0, 0, 255), 2)
    mid1 = midpoint(tl, tr)
    mid2 = midpoint(tr, br)
    mid3 = midpoint(br, bl)
    mid4 = midpoint(bl, tl)
    # print(type(midpoint(tl,tr)[0]))
    cv.circle(frame, mid1, 1, (0, 255, 0), 2)
    cv.circle(frame, mid2, 1, (0, 0, 255), 2)
    cv.circle(frame, mid3, 1, (0, 0, 255), 2)
    cv.circle(frame, mid4, 1, (0, 0, 255), 2)
    # print(midpoint(tl,tr))
    cv.circle(frame, (int(center[0]), int(center[1])), 1, (0, 0, 255), 2)
    if dist.euclidean(mid3, center) > dist.euclidean(mid2, center):
        if mid3[0] > mid1[0]:
            angle = atan2((mid3[1] - center[1]), (mid3[0] - center[0])) * 180 / pi
        else:
            angle = atan2((mid1[1] - center[1]), (mid1[0] - center[0])) * 180 / pi
    else:
        if mid2[0] > mid4[0]:
            angle = atan2((mid2[1] - center[1]), (mid2[0] - center[0])) * 180 / pi
        else:
            angle = atan2((mid4[1] - center[1]), (mid4[0] - center[0])) * 180 / pi
    angle += 90
    y = 480 - 121 - center[1]
    print(y)
    # cv.imshow('frame',frame)
    bag_pos = (float((center[0]-227) * 22 / 145), float(y * 22 / 145))
    bag_pos_AB = ((bag_pos[0] + bag_pos[1]) / sqrt(2), ((bag_pos[0] - bag_pos[1]) / sqrt(2)))
    cv.imwrite('frame.png',frame)
    #print(bag_pos)
print(bag_pos_AB)
print('checkpoint4')
A.move(bag_pos_AB[0])
print('checkpoint5')
B.move(bag_pos_AB[1])
print('checkpoint6')
#verify 2 setzero z
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('go posAB finish')
            break



B.rotate(angle)
print('check7')
# time.sleep(1)
ser.reset_output_buffer()
B.put(3)
ser.reset_output_buffer()
# time.sleep(1)
A.down(6)
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('done move Z')
            break
# time.sleep(2)
ser.reset_output_buffer()
B.grab()
print('check8')
ser.reset_output_buffer()
# time.sleep(1)
A.setZ()
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('setZ finish')
            break
A.move(19.06)
B.move(-19.00)
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('go posAB finish')
            break
B.rotate(0)
A.down(10)
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('go posAB finish')
            break
# time.sleep(2)
ser.reset_output_buffer()
B.put(3)


A.setZ()
while (1):
    if ser.inWaiting() > 0:
        data = ser.read(1)
        print("data =", ord(data))
        if ord(data) == 107:
            print('go posAB finish')
            break
A.setZero()
B.setZero()



