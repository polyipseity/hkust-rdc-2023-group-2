import cv2
import numpy as np

# Video capture
cap = cv2.VideoCapture(0) 

x1 = 50 
y1 = 50
x2 = 100
y2 = 100

while(True):

    
    
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Crop the region of interest
    roi = frame[y1:y2, x1:x2]
    
    # Convert ROI to HSV colorspace
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Compute histogram of Hue channel
    hist = cv2.calcHist([hsv],[0],None,[180],[0,180])
    
    # Find peak histogram value (dominant color)
    dominant_color = np.argmax(hist)
    
    print("Dominant Hue:", dominant_color) 

    # Skip HSV conversion

    # Compute histograms for R, G, B channels  
    bhist = cv2.calcHist([roi],[0],None,[256],[0,256])
    ghist = cv2.calcHist([roi],[1],None,[256],[0,256])
    rhist = cv2.calcHist([roi],[2],None,[256],[0,256])

    # Find peak histogram bin for each channel
    b_dominant = np.argmax(bhist)
    g_dominant = np.argmax(ghist) 
    r_dominant = np.argmax(rhist)

    # Print dominant colors
    print("Dominant RGB:", r_dominant, g_dominant, b_dominant)

    # image_bordered = cv2.copyMakeBorder(src=frame, top=100, bottom=10, left=100, right=100, borderType=cv2.BORDER_CONSTANT) 
    # borderType=cv2.BORDER_CONSTANT


    
    # Display ROI
    cv2.imshow('roi',frame)
    
    # Break on escape key
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break
        
cap.release()
cv2.destroyAllWindows()