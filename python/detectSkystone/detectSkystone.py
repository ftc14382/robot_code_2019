#!/usr/local/bin/python3
import numpy as np
import cv2
from matplotlib import pyplot as plt

#cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
outImg = np.zeros((h, w * 3, 3), np.uint8)
print("width = %d" % w)
print("height = %d" % h)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    blur = cv2.GaussianBlur(frame,(7,7), 0)

    rgbFrame = cv2.cvtColor(blur, cv2.COLOR_BGR2RGB)
    hsvFrame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([20,50, 50])
    upper_yellow = np.array([40,255,255])

#    lower_white  = np.array([0, 0, 248])
#    upper_white  = np.array([255, 7, 255])

    mask = cv2.inRange(hsvFrame, lower_yellow, upper_yellow)
    mask2 = cv2.bitwise_not(mask);
#    contours, hierarchy = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#    maskWhite = cv2.inRange(hsvFrame, lower_white, upper_white)
#    notMaskWhite = cv2.bitwise_not(maskWhite)
#    rgbMaskWhite = cv2.cvtColor(maskWhite, cv2.COLOR_BGR2RGB)


#    mask2 = cv2.bitwise_or(mask, maskWhite)

    rgbMask = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
#    cv2.drawContours(rgbMask, contours, -1, (0,255,0), 3)
#    cv2.fillPoly(rgbMask, pts =contours, color=(255,0,0))
    for c in contours:
        print("area: %d" % cv2.contourArea(c))
        if (cv2.contourArea(c) < 600 ):
            cv2.fillPoly(rgbMask, [c], color = (0, 0, 255))
            cv2.fillPoly(mask, [c], color = (255))

#    rgbMask2 = cv2.cvtColor(mask2, cv2.COLOR_BGR2RGB)

    img = cv2.bitwise_and(frame,frame, mask=mask)
    rgbImg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    lineStartx = 0
    lineStarty = 30
    lineEndx   = w
    lineEndy   = lineStarty
    cv2.line(rgbMask, (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)
    cv2.line(frame,   (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)
    cv2.line(img,     (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)

    lineStarty = lineStarty + 200
    lineEndy   = lineStarty
    cv2.line(rgbMask, (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)
    cv2.line(frame,   (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)
    cv2.line(img,     (lineStartx, lineStarty), (lineEndx, lineEndy), (250, 0, 0), 3)

#    img2 = cv2.bitwise_and(frame, frame, mask=maskWhite)

#    img3 = cv2.bitwise_or(img, img2)


    outImg[:h, :w, :3]          = frame
    outImg[:h, w:(2*w), :3]     = rgbMask
    outImg[:h, (2*w):(3*w), :3] = img

#    ret,thresh1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
#    ret,thresh2 = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
#    ret,thresh3 = cv2.threshold(img,127,255,cv2.THRESH_TRUNC)
#    ret,thresh4 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO)
#    ret,thresh5 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO_INV)
    
#    titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
#    images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]

#    for i in range(6):
#        plt.subplot(2,3,i+1),plt.imshow('foo', images[i],'gray')
#        plt.title(titles[i])
#        plt.xticks([]),plt.yticks([])
#
#    plt.subplot(1,3,1),plt.imshow(rgbFrame)
#    plt.subplot(1,3,2),plt.imshow(rgbMask)
#    plt.subplot(1,3,3),plt.imshow(rgbImg)
#    plt.imshow(img, 'gray')


#    plt.show()
    cv2.imshow('Find Yellow', outImg)
#    plt.hist(frame.ravel(), 256, [0,256])
#    plt.show()
#    cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
#    foo = cv2.waitKey(200)
#    print("foo: %d" % foo)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
cv2.waitKey(1)
