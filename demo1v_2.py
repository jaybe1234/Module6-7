import cv2 as cv
import numpy as np
import time
import serial
import struct
from Control.coordinate import coordinate
from scipy.spatial import distance as dist
from math import atan2,pi,sqrt

from keras.models import model_from_json
json_file = open('model.json','r')
loaded_model_json = json_file.read()
model = model_from_json(loaded_model_json)
model.load_weights('model.h5')

def midpoint(A,B):
    return(int((A[0]+B[0])*0.5),int((A[1]+B[1])*0.5))


pattern1A = [29.65,22.25,29.65,22.25,29.65,22.25]
pattern1B =[5.27,-2.79,5.27,-2.79,5.27,-2.79]
pattern1angle=[180,0,180,0,180,0]
dropdown1 = [4.9,4.9,4.6,4.6,3.5,3.5]

pattern2A = [29.65,22.59,28.97,36.67]
pattern2B =[5.27,-1.39,-9.65,-1.76]
pattern2angle = [180,270,0,90]
dropdown2 = [5,5,5,5]

pattern3A = [29.65,22.25,32.41,25.84,35.59,29.02]
pattern3B =[5.27,-2.79,1.33,-5.24,-1.85,-8.72]
# pattern3A = [30.3,24.07,32.41,26.19,35.59,29.37]
# pattern3B =[5.27,-2.79,1.33,-4.89,-1.85,-8.07]
# pattern3A = [29.65,22.25,31.77,24.37,33.89,26.49]
# pattern3B =[5.27,-2.79,3.15,-4.90,1.03,-7.03]
pattern3angle = [180,0,180,0,180,0]
dropdown3 = [4.9,4.9,4.8,4.8,4.8,4.8]

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
while x < len(pattern2A):

    A.put(3)
    B.setZero()
    B.setZero()
    A.setZero()
    A.setZero()


    #verify 1 setzero xy
    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            # print("data =", ord(data))
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
            # print("data =", ord(data))
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
        frame[318:480,0:640] = 0
        frame[0:480, 0:179] = 0
        frame[0:50, 0:640] = 0
        #frame = frame[121:357,227:372]
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (15, 15), 0)
        sharpened = cv.addWeighted(blur, -0.3, gray, 1, 0)
        ret, thres1 = cv.threshold(blur, 53, 255,0)
        (_, cnts, _) = cv.findContours(thres1, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for contour in cnts:
            if cv.contourArea(contour) < 2635 or cv.contourArea(contour) > 6000:
                continue
            rect = cv.minAreaRect(contour)
            (center, _, _) = rect
            box = cv.boxPoints(rect)
            box = np.int0(box)
            # cv.drawContours(frame, [box], 0, (255, 0, 0), 2)
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
            # print(y)
            # cv.imshow('frame',frame)
            print(angle)
            bag_pos = [float((center[0] - 227) * 30 / 204 + 3.98), float(y * 30 / 204)-4.5]
            if angle >60 and angle<120:
                bag_pos[0] -= 0.74

            if angle>75 and angle<105:
                bag_pos[1] += 0.8
            if angle >160 and angle<210:
                bag_pos[0]+=0.6
                bag_pos[1]-=1
            if angle > 250:
                 bag_pos[0]-=1
                 bag_pos[1] -= 3.3
            elif angle > 235:
                bag_pos[0] -= 0.5
                bag_pos[1] -= 2.6
                print('woyyy')
            elif angle>=210:
                print('210woy')
                # bag_pos[0]-=0.
                bag_pos[1]-=2.2



            if bag_pos[1] >= 20:
                bag_pos[1] += 0
            elif bag_pos[1]>=12:
                bag_pos[1]+=1.3
            if bag_pos[1]<= 12:
                bag_pos[1]+=2
            # elif bag_pos[1] <= 15:
            #     bag_pos[1] -= 4.1

            bag_pos_AB = ((bag_pos[0] + bag_pos[1]) / sqrt(2), ((bag_pos[0] - bag_pos[1]) / sqrt(2)))



            # cv.imwrite('frame.png',frame)
        print('loop1')
    print(bag_pos)
    type = ''
    img = frame[int(center[1]) - 75:int(center[1]) + 75, int(center[0]) - 75:int(center[0]) + 75]
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    # cv.imwrite('img.png',img)
    img = np.array([img.tolist()])
    prediction = model.predict(img)
    prediction = prediction.tolist()[0]
    print(prediction)
    if prediction[0]>prediction[1] and prediction[0]>prediction[2]:
        type = 'big'
    elif prediction[1]>prediction[0] and prediction[1]>prediction[2]:
        type = 'medium'
    elif prediction[2]>prediction[0] and prediction[2]>prediction[1]:
        type = 'small'
    print(type)
    if type != inputtype:
        target = redzone
        targetangle = angle
        targetdrop = 0
    else:
        target = [pattern2A[x], pattern2B[x]]
        targetangle = pattern2angle[x]
        targetdrop =dropdown2[x]
        x+=1
    print(bag_pos)
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
            # print("data =", ord(data))
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
    A.down(3)
    while (1):
        if ser.inWaiting() > 0:
            data = ser.read(1)
            # print("data =", ord(data))
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
            # print("data =", ord(data))
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
            # print("data =", ord(data))
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
            # print("data =", ord(data))
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
            # print("data =", ord(data))
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


