# Robotic arm control and movement using hand co-ordinates

# This is the python code required to detect hand co-ordinates from the video provided by webcam and send then serially using in built
# bluetooth to the ATmega328P (Arduino UNO) with the help of HC-06. A record and play of robotic arm option is also provided.

#     AUTHOR  : Nevil Pooniwala
#     MOBILE  : 9870000336
#     EMAIL   : nevilpooniwala1997@gmail.com

import cv2
import numpy as np
import copy
import math
import serial
import time
port="COM5"
bluetooth=serial.Serial(port, 9600)

# parameters
cap_region_x_begin=0.4  # start point/total width make this 0
cap_region_y_end=0.8  	# start point/total width make this 1
threshold = 20  	#  BINARY threshold
blurValue = 41  	# GaussianBlur parameter
bgSubThreshold = 50

# variables
isBgCaptured = 0   	# bool, whether the background captured

ang_x = 90
pre_x = 0

ang_y = 130
pre_y = 0

ang_z = 70
pre_z = 0

nvl = 0

nevilp = 0
nev_x = []
nev_y = []
nev_z = []
nev_grip = []

def x_deg(con_x):
    global ang_x, pre_x
    if(pre_x > con_x):
        qaz = pre_x - con_x
        if(qaz < 50):
            ang_x = ang_x + int(round( qaz/1.5))
            if(ang_x > 180):
                             ang_x = 180
            if(ang_x < 0):
                             ang_x = 0
    if (con_x >= pre_x):
        qaz = con_x - pre_x
        if (qaz < 50):
            ang_x = ang_x - int(round( qaz/1.5))
            if (ang_x > 180):
                             ang_x = 180
            if (ang_x < 0):
                             ang_x = 0
    pre_x = con_x
    return ang_x

def y_deg(con_y):
    global ang_y, pre_y
    if(pre_y > con_y):
        wsx = pre_y - con_y
        if(wsx < 50):
            ang_y = ang_y + int(round( wsx/2.8))
            if(ang_y > 180):
                             ang_y = 180
            if(ang_y < 70):
                             ang_y = 70
    if (con_y >= pre_y):
        wsx = con_y - pre_y
        if (wsx < 50):
            ang_y = ang_y - int(round( wsx/2.8))
            if (ang_y > 180):
                             ang_y = 180
            if (ang_y < 70):
                             ang_y = 70
    pre_y = con_y
    return ang_y

def z_deg(con_z):
    global ang_z, pre_z
    if(pre_z > con_z):
        edc = pre_z - con_z
        if(edc < 50):
            ang_z = ang_z + int(round( edc/0.9))
            if(ang_z > 115):
                             ang_z = 115
            if(ang_z < 50):
                             ang_z = 50
    if (con_z >= pre_z):
        edc = con_z - pre_z
        if (edc < 50):
            ang_z = ang_z - int(round( edc/0.9))
            if (ang_z > 115):
                             ang_z = 115
            if (ang_z < 50):
                             ang_z = 50
    pre_z = con_z
    return ang_z

def neville(con_x, con_y, con_z):
    global nvl , nevilp
    if ( con_x < 50 or con_x > 340):
        final_ang_x = ang_x
    else :
        final_ang_x = x_deg(con_x)
    if ( con_y < 40 or con_y > 350):
        final_ang_y = ang_y
    else :
        final_ang_y = y_deg(con_y)
    if ( con_z < 50 or con_z > 160):
        final_ang_z = ang_z
    else :
        final_ang_z = z_deg(con_z)
    if ( nvl == 0):
        grip_ang = 70
    if ( nvl == 1):
        grip_ang = 100 
    nevil_avg(final_ang_x, final_ang_y, final_ang_z, grip_ang)

x_avg = []
y_avg = []
z_avg = []
avg_count_no = 2
avg_count = avg_count_no

def nevil_avg(final__x, final__y, final__z, grip__ang):
    global avg_count, avg_count_no, x_avg, y_avg, z_avg
    if(avg_count != 0 ):
        x_avg.append(final__x)
        y_avg.append(final__y)
        z_avg.append(final__z)
        avg_count = avg_count-1
    else:
        avg_count = avg_count_no
        final_ang_avg_x = sum(x_avg)/len(x_avg)
        final_ang_avg_y = sum(y_avg)/len(y_avg)
        final_ang_avg_z = sum(z_avg)/len(z_avg)
        x_avg = []
        y_avg = []
        z_avg = []
        if nevilp == 1:
            nev_x.append(final_ang_avg_x)
            nev_y.append(final_ang_avg_y)
            nev_z.append(final_ang_avg_z)
            nev_grip.append(grip__ang)
        print "x:" , final_ang_avg_x , "y:" , final_ang_avg_y , "z:" , final_ang_avg_z , "grip" , grip__ang
        bluetoth("x: " + str(final_ang_avg_x) + " y: " + str(final_ang_avg_y) + " z: " + str(final_ang_avg_z) + " grip: " + str(grip__ang) + ";")


def printThreshold(thr):
    print("! Changed threshold to "+str(thr))

def bluetoth(nevil):
    bluetooth.write(str.encode(nevil))

def removeBG(frame):
    fgmask = bgModel.apply(frame)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    res = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

    kernel = np.ones((3, 3), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=fgmask)
    return res


def calculateFingers(res,drawing):  # -> finished bool, cnt: finger count
    #  convexity defect
    hull = cv2.convexHull(res, returnPoints=False)
    if len(hull) > 3:
        defects = cv2.convexityDefects(res, hull)
        if type(defects) != type(None):  # To avoid crashing.
            my_start = []
            my_far = []
            my_farx =[]
            cnt = 0
            for i in range(defects.shape[0]):  # calculate the angle
                s, e, f, d = defects[i][0]
                start = tuple(res[s][0])
                end = tuple(res[e][0])
                far = tuple(res[f][0])
                a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))  # cosine theorem
                if angle <= math.pi / 2:  # angle less than 90 degree, treat as fingers
                    cnt += 1
                    cv2.circle(drawing, far, 8, [211, 84, 0], -1)  #far has blue points values, find nearesrt far by diff
                    my_start.append(start[1])
                    my_farx.append(far[0])
                    my_far.append(far[1])
            if(my_start and my_far):
                z_cor = min(my_far) - min(my_start)   #x,y,z_cor
                y_cor = min(my_far)
                x_cor = min(my_farx)
                # if isFinishCal is True and cnt <= 1: #it was <=2, shows no. of fingers
                #  print cnt   #close arm grip
                if cnt > 2:
                    neville(x_cor, y_cor, z_cor)
            return True, cnt
    return False, 0


# Camera
camera = cv2.VideoCapture(0)
camera.set(10,200)
cv2.namedWindow('trackbar')
cv2.createTrackbar('trh1', 'trackbar', threshold, 100, printThreshold)


while camera.isOpened():
    ret, frame = camera.read()
    threshold = cv2.getTrackbarPos('trh1', 'trackbar')
    frame = cv2.bilateralFilter(frame, 5, 50, 100)  # smoothing filter
    frame = cv2.flip(frame, 1)  # flip the frame horizontally
    cv2.rectangle(frame, (int(cap_region_x_begin * frame.shape[1]), 0),
                 (frame.shape[1], int(cap_region_y_end * frame.shape[0])), (255, 0, 0), 2)
    cv2.imshow('original', frame)

    #  Main operation
    if isBgCaptured == 1:  # this part wont run until background captured
        img = removeBG(frame)
        img = img[0:int(cap_region_y_end * frame.shape[0]),
                    int(cap_region_x_begin * frame.shape[1]):frame.shape[1]]  # clip the ROI
        cv2.imshow('mask', img)

        # convert the image into binary image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (blurValue, blurValue), 0)
        cv2.imshow('blur', blur)
        ret, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow('ori', thresh)


        # get the coutours
        thresh1 = copy.deepcopy(thresh)
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        length = len(contours)
        maxArea = -1
        if length > 0:
            for i in range(length):  # find the biggest contour (according to area)
                temp = contours[i]
                area = cv2.contourArea(temp)
                if area > maxArea:
                    maxArea = area
                    ci = i

            res = contours[ci]
            hull = cv2.convexHull(res)
            drawing = np.zeros(img.shape, np.uint8)
            cv2.drawContours(drawing, [res], 0, (0, 255, 0), 2) #draws fingers
            cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3) #hull has outer circle
            #print res, hull
            if (nevilp == 2):
                for npp in range(len(nev_grip)):
                     print "x:" , nev_x[npp] , "y:" , nev_y[npp] , "z:" , nev_z[npp] , "grip" , nev_grip[npp]
                     time.sleep(0.1)
                nevilp = 0
            else :
                isFinishCal,cnt = calculateFingers(res,drawing)
                if isFinishCal is True and cnt <= 2: #it was <=2, shows no. of fingers
                    nvl = not nvl
                  #  print cnt   #close arm grip

        cv2.imshow('output', drawing)

    # Keyboard OP
    k = cv2.waitKey(10)
    if k == 27:  # press ESC to exit
        break
    elif k == ord('b'):  # press 'b' to capture the background
        bgModel = cv2.BackgroundSubtractorMOG2(0, bgSubThreshold)
        isBgCaptured = 1
        print '!!!Background Captured!!!'
    elif k == ord('r'):  # press 'r' to reset the background
        bgModel = None
        isBgCaptured = 0
        print '!!!Reset Background!!'
    elif k == ord('i'):
        if (nevilp != 2):
            nvl = not nvl #for gripper
    elif k == ord('s'):
        nevilp = 1
        nev_x = []
        nev_y = []
        nev_z = []
        nev_grip = []
        print 'Started Recording!!'
    elif k == ord('p'):
        nevilp = 2
        print 'Started Playing!!'
