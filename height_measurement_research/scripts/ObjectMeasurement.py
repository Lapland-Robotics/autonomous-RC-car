#!/usr/bin/env python

import cv2
import numpy as np
#import rospy


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

webcam = True
path = '1.jpg'
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
# cap.set(10,160)
# cap.set(3,1920)
# cap.set(4,1080)
scale = 3
wP = 210 * scale
hP = 297 * scale

lower = [1, 0, 20]
upper = [60, 40, 220]

lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")

def getContours(img, cThr=[50, 400], showCanny=False, minArea=1000, filter=0, draw=False):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)

    imgCanny = cv2.Canny(imgBlur, cThr[0], cThr[1])
    kernel = np.ones((5, 5))
    imgDial = cv2.dilate(imgCanny, kernel, iterations=3)
    imgThre = cv2.erode(imgDial, kernel, iterations=2)
    #if showCanny:
    cv2.imshow('Canny', imgCanny)
    cv2.imshow('threshold', imgThre)
    contours, hiearchy = cv2.findContours(imgThre, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    img_contours = np.zeros(img.shape)
    cv2.drawContours(img_contours, contours, -1, (0, 255, 0), 3)
    mask = cv2.inRange(img, lower, upper)
    output = cv2.bitwise_and(img, img, mask=mask)

    

    # print(contours)
    finalCountours = []
    for i in contours:
        area = cv2.contourArea(i)
        #print("area", area)
        if area > minArea:
            peri = cv2.arcLength(i, True)
            approx = cv2.approxPolyDP(i, 0.02*peri, True)
            bbox = cv2.boundingRect(approx)

            cv2.drawContours(output, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(approx)

            # draw the biggest contour (c) in green
            cv2.rectangle(output,(x,y),(x+w,y+h),(0,255,0),2)

            cv2.imshow("contours", output)

            # Filter for 4 rectangle points
            # if filter > 0:
            #     if len(approx) == filter:
            #         print(filter)
            #         finalCountours.append([len(approx), area, approx, bbox, i])
            # else:
            finalCountours.append([len(approx), area, approx, bbox, i])
            

    #lambda lenght and area
    finalCountours = sorted(finalCountours, key=lambda x: x[1], reverse=True)

    #print(finalCountours)

    for con in finalCountours:
        cv2.drawContours(img, con[4], -1, (0, 0, 255), 4)
        cv2.imshow("finalContours", img)

    if draw:
        for con in finalCountours:
            cv2.drawContours(img, con[4], -1, (0, 0, 255), 3)
    return img, finalCountours


def reorder(myPoints):
    #print("my points", myPoints)
    print("shape", myPoints.shape[1])
    myPointsNew = np.zeros_like(myPoints)
    myPoints = myPoints.reshape(myPoints.shape[1], 2)
    add = myPoints.sum(1)
    myPointsNew[0] = myPoints[np.argmin(add)]
    myPointsNew[3] = myPoints[np.argmax(add)]
    diff = np.diff(myPoints, axis=1)
    myPointsNew[1] = myPoints[np.argmin(diff)]
    myPointsNew[2] = myPoints[np.argmax(diff)]
    return myPointsNew


def warpImg(img, points, w, h, pad=20):
    print("points", points)
    points = reorder(points)
    print("reordered points", points)
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    imgWarp = imgWarp[pad:imgWarp.shape[0]-pad, pad:imgWarp.shape[1]-pad]
    return imgWarp


def findDis(pts1, pts2):
    return ((pts2[0]-pts1[0])**2 + (pts2[1]-pts1[1])**2)**0.5


def listener():
    while cap.isOpened():
        if webcam:
            success, img = cap.read()
        else:
            img = cv2.imread(path)

        imgContours, conts = getContours(img, minArea=50000, filter=4)

        #print(conts)
        if len(conts) != 0:
            biggest = conts[0][2]
            #print(biggest)
            imgWarp = warpImg(img, biggest, wP, hP)
            imgContours2, conts2 = getContours(imgWarp,
                                               minArea=2000, filter=4,
                                               cThr=[100, 100], draw=False)
            if len(conts) != 0:
                for obj in conts2:
                    cv2.polylines(imgContours2, [obj[2]], True, (0, 255, 0), 2)
                    nPoints = reorder(obj[2])
                    nW = round(
                        (findDis(nPoints[0][0]//scale, nPoints[1][0]//scale)/10), 1)
                    nH = round(
                        (findDis(nPoints[0][0]//scale, nPoints[2][0]//scale)/10), 1)
                    cv2.arrowedLine(imgContours2, (nPoints[0][0][0], nPoints[0][0][1]), (nPoints[1][0][0], nPoints[1][0][1]),
                                    (255, 0, 255), 3, 8, 0, 0.05)
                    cv2.arrowedLine(imgContours2, (nPoints[0][0][0], nPoints[0][0][1]), (nPoints[2][0][0], nPoints[2][0][1]),
                                    (255, 0, 255), 3, 8, 0, 0.05)
                    x, y, w, h = obj[3]
                    cv2.putText(imgContours2, '{}cm'.format(nW), (x + 30, y - 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                                (255, 0, 255), 2)
                    cv2.putText(imgContours2, '{}cm'.format(nH), (x - 70, y + h // 2), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                                (255, 0, 255), 2)
            cv2.imshow('A4', imgContours2)

        # img = cv2.resize(img,(0,0),None,0.5,0.5)
        cv2.imshow('Original', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    listener()
