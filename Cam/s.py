EXPOSURE_TIME_SCALE = 0.7

thresholds = [(74, 95, -40, 2, 35, 76),
(12, 35, 11, 26, -76, -26)]


#thresholds = [(42, 67, -12, 15, 7, 37),
#(29, 38, -32, 8, -26, -16)]

cX = 163
cY = 129

#cX = 174
#cY = 137

import sensor, image, math, pyb, time, random
from pyb import UART
uart = UART(3, 115200, timeout_char=1)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1)
led = pyb.LED(3)


def dot(x, y, color=(255, 0, 0), sz = 5):
    x -= sz / 2
    y -= sz / 2
    for i in range(sz * sz):
        img.set_pixel(int(x) + (i % sz) - 1, int(y) + int(i / sz) - 1, color)

yX = 0
yY = 0
bX = 0
bY = 0
yAngle = 0
yDist = 0
bAngle = 0
bDist = 0
xCoord = 0
yCoord = 0
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
current_exposure_time_in_microseconds =  sensor.get_exposure_us()
sensor.set_auto_exposure(True, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
clock = time.clock()
sensor.skip_frames(time = 1500)

#sensor.reset() #reset camera
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
current_exposure_time_in_microseconds =  sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds* EXPOSURE_TIME_SCALE))
#sensor.set_gainceiling(2)
clock = time.clock()
sensor.skip_frames(time = 2000) #delay

goalCoords = [[0, 0],
              [10, 19],
              [15, 25],
              [20, 33],
              [30, 46],
              [40, 60],
              [50, 74],
              [60, 83],
              [70, 91],
              [80, 97],
              [90, 101],
              [100, 106],
              [110, 109],
              [120, 111],
              [130, 113],
              [140, 115],
              [150, 116],
              [160, 118]]

def realDistance(x):
    xOne = 0
    xTwo = 0
    yOne = 0
    yTwo = 0
    fSign = 1
    if x<0:
        xAbs = x*-1
        fSign = -1
    else:
        xAbs = x
    for i in range(0, len(goalCoords) - 1):
        if goalCoords[i][1] <= xAbs and xAbs < goalCoords[i + 1][1]:
            xOne = goalCoords[i][1]
            xTwo = goalCoords[i + 1][1]
            yOne = goalCoords[i][0]
            yTwo = goalCoords[i + 1][0]
            return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign
    if goalCoords[len(goalCoords) - 1][1] <= xAbs:
        xOne = goalCoords[len(goalCoords) - 2][1]
        xTwo = goalCoords[len(goalCoords) - 1][1]
        yOne = goalCoords[len(goalCoords) - 2][0]
        yTwo = goalCoords[len(goalCoords) - 1][0]
    return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign

def crc8(data, len):
    crc = 0xFF
    for i in range(len):
        crc ^= data[i]
        for j in range(8):
            if crc & 0x80:
                crc = (char)((crc << 1) ^ 0x31)
            else:
                crc <<= 1
    return crc


def getCenterBetween(blob1, blob2):
    if (blob1[2] * blob1[3] == 0 or blob2[2] * blob2[3] == 0):
            yGkX = 0
            yGkY = 0
    else:
        if (blob1[0] > blob2[0]):
            yGkX = -(blob1[0] + blob2[0] + blob2[2]) / 2 + cX
            yGkY = -(blob1[1] + blob1[3] / 2 + blob2[1] + blob2[3] / 2) / 2 + cY
        else:
            yGkX = -(blob1[0] + blob1[2] + blob2[0]) / 2 + cX
            yGkY = -(blob1[1] + blob1[3] / 2 + blob2[1] + blob2[3] / 2) / 2 + cY
    return (yGkX, yGkY)



def partition(array, begin, end):
    pivot = begin
    for i in range(begin+1, end+1):
        if array[i][0] <= array[begin][0]:
            pivot += 1
            array[i], array[pivot] = array[pivot], array[i]
    array[pivot], array[begin] = array[begin], array[pivot]
    return pivot

def quicksort(array, begin=0, end=None):
    if end is None:
        end = len(array) - 1
    def _quicksort(array, begin, end):
        if begin >= end:
            return
        pivot = partition(array, begin, end)
        _quicksort(array, begin, pivot-1)
        _quicksort(array, pivot+1, end)
    return _quicksort(array, begin, end)

while(True):
    img = sensor.snapshot().mask_circle(cX, cY, 125)

    yS = [0] * 2
    bS = [0] * 2
    yX = [0] * 2
    yY = [0] * 2
    bX = [0] * 2
    bY = [0] * 2
    #led.on()
    yAngle=0
    yDist=0
    bAngle=0
    bDist=0
    yFlag = 0
    bFlag = 0

    truebX = 0
    truebY = 0
    trueyY = 0
    trueyX = 0

    yBlob = [[0] * 4] * 2

    bBlob = [[0] * 4] * 2

    firstYArea = 0
    secondYArea = 0
    firstBArea = 0
    secondBArea = 0

    blobsY = []
    blobsB = []

    for blob in img.find_blobs(thresholds, pixels_threshold=50, area_threshold=50, merge=True):
        if blob.code() == 1:
            if (blob[2] * blob[3] > firstYArea):
                secondYArea = firstYArea
                yBlob[1] = yBlob[0]
                firstYArea = blob[2] * blob[3]
                yBlob[0] = blob.rect()
            else:
                if (blob[2] * blob[3] > secondYArea):
                    secondYArea = blob[2] * blob[3]
                    yBlob[1] = blob.rect()
            yFlag = 1
            blobsY.append(blob.rect())

        if blob.code() == 2:
            if (blob[2] * blob[3] > firstBArea):
                secondBArea = firstBArea
                bBlob[1] = bBlob[0]
                firstBArea = blob[2] * blob[3]
                bBlob[0] = blob.rect()
            else:
                if (blob[2] * blob[3] > secondBArea):
                    secondBArea = blob[2] * blob[3]
                    bBlob[1] = blob.rect()
            bFlag = 1
            blobsB.append(blob.rect())

    if (yFlag == 1):
        quicksort(blobsY, 0, len(blobsY) - 1)
        for i in range(len(blobsY) - 1):
           gkDot = getCenterBetween(blobsY[i], blobsY[i + 1])
           img.draw_rectangle(blobsY[i], color=(255,255, 0), fill=0)
           dot(cX - gkDot[0], cY - gkDot[1], color = (255,255,255))
        img.draw_rectangle(blobsY[len(blobsY) - 1], color=(255,255, 0), fill=0)

    if (yFlag == 1):
        biggestBlob = blobsY[0]
        for i in range(len(blobsY) - 1):
            if (blobsY[i + 1][2] * blobsY[i + 1][3] > biggestBlob[2] * biggestBlob[3]):
                biggestBlob = blobsY[i + 1]
        dot(biggestBlob[0] + biggestBlob[2] / 2, biggestBlob[1] + biggestBlob[3] / 2, color = (54,123,11))


    if (yFlag == 1):
        yBlob[0] = blobsY[0]
        yBlob[1] = blobsY[0]
        for i in range(len(blobsY)):
            if (blobsY[i][0] < yBlob[0][0]):
                yBlob[0] = blobsY[i]
            if (blobsY[i][0] + blobsY[i][2] > yBlob[1][0] + yBlob[1][2]):
                yBlob[1] = blobsY[i]

        if (yBlob[1][2] * yBlob[1][3] == 0):
            trueyX = -(yBlob[0][0] + yBlob[0][2] / 2) + cX
            trueyY = -(yBlob[0][1] + yBlob[0][3] / 2) + cY
        else:
            if (yBlob[0][0] < yBlob[1][0]):
                trueyX = -(yBlob[0][0] + yBlob[1][0] + yBlob[1][2]) / 2 + cX
                trueyY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY
            else:
                trueyX = -(yBlob[0][0] + yBlob[0][2] + yBlob[1][0]) / 2 + cX
                trueyY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY

        #yGkX = 0
        #yGkY = 0

        #if (yBlob[1][2] * yBlob[1][3] == 0):
            #yGkX = 0
            #yGkY = 0
        #else:
            #if (yBlob[0][0] > yBlob[1][0]):
                #yGkX = -(yBlob[0][0] + yBlob[1][0] + yBlob[1][2]) / 2 + cX
                #yGkY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY
            #else:
                #yGkX = -(yBlob[0][0] + yBlob[0][2] + yBlob[1][0]) / 2 + cX
                #yGkY = -(yBlob[0][1] + yBlob[0][3] / 2 + yBlob[1][1] + yBlob[1][3] / 2) / 2 + cY

    if (bFlag == 1):
        bBlob[0] = blobsB[0]
        bBlob[1] = blobsB[0]
        for i in range(len(blobsB)):
            if (blobsB[i][0] < bBlob[0][0]):
                bBlob[0] = blobsB[i]
            if (blobsB[i][0] + blobsB[i][2] > bBlob[1][0] + bBlob[1][2]):
                bBlob[1] = blobsB[i]

        img.draw_rectangle(bBlob[0], color=(0,0,255), fill=0)
        img.draw_rectangle(bBlob[1], color=(0,0,255), fill=0)

        if (bBlob[1][2] * bBlob[1][3] == 0):
            truebX = -(bBlob[0][0] + bBlob[0][2] / 2) + cX
            truebY = -(bBlob[0][1] + bBlob[0][3] / 2) + cY
        else:
            if (bBlob[0][0] < bBlob[1][0]):
                truebX = -(bBlob[0][0] + bBlob[1][0] + bBlob[1][2]) / 2 + cX
                truebY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY
            else:
                truebX = -(bBlob[0][0] + bBlob[0][2] + bBlob[1][0]) / 2 + cX
                truebY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY

        bGkX = 0
        bGkY = 0

        if (bBlob[1][2] * bBlob[1][3] == 0):
            bGkX = 0
            bGkY = 0
        else:
            if (bBlob[0][0] > bBlob[1][0]):
                bGkX = -(bBlob[0][0] + bBlob[1][0] + bBlob[1][2]) / 2 + cX
                bGkY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY
            else:
                bGkX = -(bBlob[0][0] + bBlob[0][2] + bBlob[1][0]) / 2 + cX
                bGkY = -(bBlob[0][1] + bBlob[0][3] / 2 + bBlob[1][1] + bBlob[1][3] / 2) / 2 + cY


    dot(cX, cY, color = (0,255,0), sz = 10)

    dot(cX - trueyX, cY - trueyY)
    dot(cX - truebX, cY - truebY)

    yAngle = int(math.atan2(trueyX, trueyY) / 3.14 * 180)
    yDist = int(math.sqrt(trueyX*trueyX + trueyY*trueyY))
    bAngle = int(math.atan2(truebX, truebY) / 3.14 * 180)
    bDist = int(math.sqrt(truebX*truebX + truebY*truebY))
    #print(bDist * math.cos(bAngle / 57.3))
    yDist = realDistance(yDist)
    bDist = realDistance(bDist)
    #print(yAngle)
    if yFlag == 0:
        yDist = 0
    if bFlag == 0:
        bDist = 0
    #xCoord = (yS * yDist * sin(yAngle / 57.3) + bS * bDist * sin(bAngle / 57.3)) / (yS + bS)
    #yCoord = (yS * yDist * cos(yAngle / 57.3) + bS * bDist * cos(bAngle / 57.3)) / (yS + bS)
    x = yDist * math.sin(yAngle / 57.3)
    y = 85 - yDist * math.cos(yAngle / 57.3)



    if yAngle < 0:
      yAngle = int(yAngle + 360)
    if bAngle < 0:
      bAngle = int(bAngle + 360)

    if yAngle < 0:
       yAngle = 0
    if bAngle < 0:
       bAngle = 0
    if yDist < 0:
       yDist = 0
    if bDist < 0:
       bDist = 0
    bDist = int(bDist)
    yDist = int(yDist)

    try:
        #uart.write("a")
        buf = [yAngle, yDist, bAngle, bDist]
        if(yAngle < 10):
            uart.write(str(0))
            uart.write(str(0))
            uart.write(str(yAngle))
        elif(yAngle < 100):
            uart.write(str(0))
            uart.write(str(yAngle))
        else:
            uart.write(str(yAngle))
        if(yDist < 10):
            uart.write(str(0))
            uart.write(str(0))
            uart.write(str(yDist))
        elif(yDist < 100):
            uart.write(str(0))
            uart.write(str(yDist))
        else:
            uart.write(str(yDist))
        if(bAngle < 10):
            uart.write(str(0))
            uart.write(str(0))
            uart.write(str(bAngle))
        elif(bAngle < 100):
            uart.write(str(0))
            uart.write(str(bAngle))
            #print("chop")
        else:
            uart.write(str(bAngle))
            #print("keka")
        if(bDist < 10):
            uart.write(str(0))
            uart.write(str(0))
            uart.write(str(bDist))
        elif(bDist < 100):
            uart.write(str(0))
            uart.write(str(bDist))
        else:
            uart.write(str(bDist))

        #print(yAngle, end = " ")
        #print(yDist, end = " ")
        #print(bAngle, end = " ")
        #print(bDist)
    except: print("Uart is broken")
