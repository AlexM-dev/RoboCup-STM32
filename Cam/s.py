thresholds = [(50, 86, 18, 35, 26, 66),
(28, 38, -2, 19, -43, -22)]


#cX = 162
#cY = 141

cX = 134
cY = 132

import sensor, image, math, pyb
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
#sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(False)
#sensor.set_auto_exposure(False)
sensor.skip_frames(100)



goalCoords = [[0, 0],
              [15, 40],
              [30, 52],
              [40, 70],
              [50, 73],
              [60, 81],
              [70, 89],
              [80, 95],
              [90, 104],
              [100, 109],
              [110, 111],
              [120, 112],
              [130, 116],
              [140, 118],
              [150, 119],
              [160, 121],
              [170, 123],
              [180, 125]]

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

while(True):
    #img = sensor.snapshot()
    img = sensor.snapshot().mask_circle(134,132,125)
    yS=0
    bS=0
    #led.on()
    yAngle=0
    yDist=0
    bAngle=0
    bDist=0
    for blob in img.find_blobs(thresholds, pixels_threshold=20, area_threshold=20, merge=True):
        if blob.code() == 1:
            img.draw_rectangle(blob.rect(), color=(255,255,0), fill=1)
            yS=blob.w() * blob.h()
            yY = cY - blob.y() - blob.h() / 2
            yX = cX - blob.x() - blob.w() / 2
        if blob.code() == 2:
            img.draw_rectangle(blob.rect(), color=(50,50,255), fill=1)
            bS=blob.w() * blob.h()
            bY = cY - blob.y() - blob.h() / 2
            bX = cX - blob.x() - blob.w() / 2

    dot(cX, cY, color = (0,255,0), sz = 10)
    dot(cX - yX, cY - yY)
    dot(cX - bX, cY - bY)


    yAngle = int(math.atan2(yX, yY) / 3.14 * 180)
    yDist = int(math.sqrt(yX*yX + yY*yY))
    bAngle = int(math.atan2(bX, bY) / 3.14 * 180)
    bDist = int(math.sqrt(bX*bX + bY*bY))
    yDist = realDistance(yDist)
    bDist = realDistance(bDist)

    #xCoord = (yS * yDist * sin(yAngle / 57.3) + bS * bDist * sin(bAngle / 57.3)) / (yS + bS)
    #yCoord = (yS * yDist * cos(yAngle / 57.3) + bS * bDist * cos(bAngle / 57.3)) / (yS + bS)
    #print(yDist * math.cos(yAngle / 57.3))

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
        uart.write("*")
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
    except: pass
