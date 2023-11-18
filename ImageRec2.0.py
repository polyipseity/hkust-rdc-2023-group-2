import cv2
import numpy as np
from serial import Serial as _Ser




def OffSetColors(frame, blue, green, red):
    frame[:,:,0] = frame[:,:,0] + blue
    frame[:,:,1] = frame[:,:,1] + green
    frame[:,:,2] = frame[:,:,2] + red


start = 0
webcam = cv2.VideoCapture(0) 
serial = _Ser("COM8")
serial.baudrate = 115200
serial.timeout = 100

while 1:
    regions = []
    colors = []
    colorList = []
    i = 0
   
    for i in range(4):
        _, frame = webcam.read() 
        #frame = cv2.imread('Test2.png')
        OffSetColors(frame,0,-20,-50)
        height, width, _ = frame.shape
        region_width = width//4
        regions.append(frame[height//2:,start:(start + region_width)])
        average_color_row = np.average(regions[i], axis=0)
        average_color = np.average(average_color_row, axis=0)
        colors.append(average_color)
        #d_img = np.ones((312, 312, 3), dtype=np.uint8)
        #d_img[:, :] = average_color
        start = start + region_width

    for color in colors:
        MaxIndex = color.squeeze().tolist().index(max(color))
        match MaxIndex:
            case 0:
                colorList.append('b')
            case 1:
                colorList.append('g')
            case 2:
                colorList.append('r')
            case _:
                pass      

    
    print(colorList)

    #send data over bluetooth
    j = 0
    data = ""
    for char in colorList:
        if(char == 'b' or char == 'r'):
            data = data+str(j)
        j = j+1   
    b = bytes(data, 'utf-8')
    print(data)
    serial.write(b)    

    


    cv2.imshow('frame', frame)
    cv2.waitKey()
    

    # close the window is 'd' is pressed
    if cv2.waitKey(20) & 0xFF==ord('d'):
        break



         
capture.release()
cv2.destroyAllWindows()

cv2.waitKey(0)
