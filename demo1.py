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


pattern1A = [31.36,25.13,31.36,25.13,31.36,25.13]
pattern1B =[2.40,-3.83,2.40,-3.83,2.40,-3.83]
pattern1angle=[180,0,180,0,180,0]
dropdown1 = [10,10,10,10,10,10]

pattern2A = [30.39,23.50,31.79,36.68]
pattern2B =[3.01,-3.30,-9.94,-2.75]
pattern2angle = [180,270,0,90]
dropdown2 = [10,10,10,10]

pattern3A = [30.3,24.07,32.41,26.19,35.59,29.37]
pattern3B =[3.46,-2.77,1.33,-4.89,-1.85,-8.07]
pattern3angle = [180,0,180,0,180,0]
dropdown3 = [10,10,10,10,10,10]

ser = serial.Serial()
ser.port = 'COM9'
ser.baudrate=9600
ser.timeout = 1
ser.rts = 0
ser.dtr = 0
ser.open()
# x = ser.read()
cap = cv.VideoCapture(1)

print('check1')
A = coordinate(111,ser)
print('check2')
B = coordinate(222,ser)
print('check3')
redzone = [17.50,10.20]

inputtype = 'small'
x = 0
pressw = cv.imread('press w please.jpg')
pressq = cv.imread('press q please.jpg')
cv.imshow('press Q please',pressq)

while True:
    k = cv.waitKey(0)
    if k == ord('q'):
        cv.destroyAllWindows()
        break
while x < len(pattern3A):

    A.put(3)
    B.setZero()
    B.setZero()
    A.setZero()
    A.setZero()


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
    #B.put(3)

    A.setZ()
    #verify 2 setzero z
    t = time.clock()
    while (1):
        if time.clock()>=3:
            A.setZ()
            t = time.clock()
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 107:
                print('setzero_z finish')
                break

    # time.sleep(5)

    bag_pos_AB =(0,0)

    while (bag_pos_AB[0] == 0 and bag_pos_AB[1]==0):
        t = time.clock()
        while(time.clock()-t<=1):
            ret, frame = cap.read()
            #cv.imshow('frame', frame)
        frame[0:480,275:640] = 0
        frame[384:480,0:640] = 0
        frame[0:480, 0:179] = 0
        #frame = frame[121:357,227:372]
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (15, 15), 0)
        sharpened = cv.addWeighted(blur, -0.3, gray, 1, 0)
        ret, thres1 = cv.threshold(blur, 70, 255,0)
        (_, cnts, _) = cv.findContours(thres1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for contour in cnts:
            if cv.contourArea(contour) < 2635 or cv.contourArea(contour) > 6000:
                continue
            rect = cv.minAreaRect(contour)
            (center, _, _) = rect
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(frame, [box], 0, (255, 0, 0), 2)
            (tl, tr, br, bl) = box
            # for i in range(len(box)):
                # cv.circle(frame, (int(box[i][0]), int(box[i][1])), 1, (0, 0, 255), 2)
            mid1 = midpoint(tl, tr)
            mid2 = midpoint(tr, br)
            mid3 = midpoint(br, bl)
            mid4 = midpoint(bl, tl)
            # print(type(midpoint(tl,tr)[0]))
            # cv.circle(frame, mid1, 1, (0, 255, 0), 2)
            # cv.circle(frame, mid2, 1, (0, 0, 255), 2)
            # cv.circle(frame, mid3, 1, (0, 0, 255), 2)
            # cv.circle(frame, mid4, 1, (0, 0, 255), 2)
            # # print(midpoint(tl,tr))
            # cv.circle(frame, (int(center[0]), int(center[1])), 1, (0, 0, 255), 2)
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
            if angle <= 90:
                angle += 180
            y = 480 - 121 - center[1]
            print(y)
            # cv.imshow('frame',frame)
            bag_pos = [float((center[0] - 227) * 30 / 201 + 2.25), float(y * 30 / 201) + 4.3]
            if bag_pos[1]<12:
                bag_pos[0]+=1
                bag_pos[1]+=1
            bag_pos_AB = ((bag_pos[0] + bag_pos[1]) / sqrt(2), ((bag_pos[0] - bag_pos[1]) / sqrt(2)))



            # cv.imwrite('frame.png',frame)
        print('loop1')
    print(bag_pos)
    crop = frame[int(center[1]) - 50:int(center[1]) + 50, int(center[0]) - 50:int(center[0]) + 50]
    hsv = cv.cvtColor(crop, cv.COLOR_BGR2HSV)
    hsv_big = cv.inRange(hsv, (128, 68, 0), (255, 255, 255))
    (_, cnts_big, _) = cv.findContours(hsv_big, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    n = 0
    for contour in cnts_big:
        if cv.contourArea(contour)<7 or cv.contourArea(contour)>109 :
            continue
        n += 1
    type =''

    if n>=3:
        type = 'big'
    else:
        n=0
        # hsv_medium = cv.inRange(hsv,(h_low,s_low,v_low),(h_high,s_high,v_high))
        hsv_medium = cv.inRange(hsv, (0, 17, 32), (255, 34, 116)) #color intensity 100, brightness 33, contrast 69
        canny = cv.Canny(hsv_medium, 0, 255)
        # cv.imshow('hsv_big',canny)
        (_, cnts_medium, _) = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for contour in cnts_medium:
            if cv.contourArea(contour) < 0 or cv.contourArea(contour) > 2:
                continue
            n+=1
        if n>145:
            type = 'small'
        elif n>40:
            type = 'medium'
    print(type)
    if type != inputtype:
        target = redzone
        targetangle = angle
        targetdrop = 0
    else:
        target = [pattern1A[x], pattern1B[x]]
        targetangle = pattern1angle[x]
        targetdrop =dropdown1[x]
        x+=1
    print(bag_pos_AB)
    print('checkpoint4')
    A.move(bag_pos_AB[0])
    # time.sleep(2)
    print('checkpoint5')
    B.move(bag_pos_AB[1])
    print('checkpoint6')

    #verify 2 setzero z
    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 80:
                print('go posAB finish')
                break

    B.rotate(angle)
    print('check7')
    # time.sleep(1)
    #ser.reset_output_buffer()
    #B.put(3)
    ser.reset_output_buffer()
    #time.sleep(1)
    A.down(6)
    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 107:
                print('done move Z')
                break
    time.sleep(2)
    ser.reset_output_buffer()
    #B.grab()
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
    B.move(target[1])
    time.sleep(1)
    A.move(target[0])
    print(target[0])
    print(target[1])
    # time.sleep(1)

    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 80:
                print('go posAB finish')
                break

    time.sleep(1)
    B.rotate(targetangle)
    #A.down(8)
    print(targetdrop)
    A.downdrop(targetdrop)
    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 107:
                print('go posAB finish')
                break
    # time.sleep(2)
    ser.reset_output_buffer()
    #B.put(3)


    A.setZ()
    t = time.clock()
    while (1):
        if time.clock()-t>=3:
            A.setZ()
            t = time.clock()
        if ser.inWaiting() > 0:
            data = ser.read(1)
            print("data =", ord(data))
            if ord(data) == 107:
                print('go posAB finish')
                break
    A.put(3)
    cv.imshow('Press W please',pressw)

    while True:
        k = cv.waitKey(0)
        if k == ord('w'):
            cv.destroyAllWindows()
            break







