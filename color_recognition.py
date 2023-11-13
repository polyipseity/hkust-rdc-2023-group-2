import numpy as np
import cv2


## change depending on the size in the future
# working area defines min size of box
min_working_area = 30000
max_working_area = 50000
# subject to change based on actual tests


# red team = 0, blue team = 1
Team = 1


# Capturing video through webcam
webcam = cv2.VideoCapture(0)

# Start a while loop
while True:
    Red_List = []
    Blue_List = []
    Green_List = []

    # Reading the video from the
    # webcam in image frames
    _, imageFrame = webcam.read()

    # Convert the imageFrame in
    # BGR(RGB color space) to
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color and
    # define mask
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([200, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and
    # define mask
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Set range for blue color and
    # define mask
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernel = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernel)
    median_red = cv2.medianBlur(red_mask, 15)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=median_red)

    # For green color
    green_mask = cv2.dilate(green_mask, kernel)
    median_green = cv2.medianBlur(green_mask, 15)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=median_green)

    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernel)
    median_blue = cv2.medianBlur(blue_mask, 15)

    # kernel = np.ones((15,15), np.float32)/255
    # smoothed = cv2.filter2D(blue_mask, -1, kernel)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=median_blue)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(
        red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > min_working_area and area < max_working_area:
            x, y, w, h = cv2.boundingRect(contour)
            # store the position and color of the box in a common array
            x_pos = (x + w) / 2
            y_pos = (y + h) / 2
            List = []
            List.append(x_pos)
            List.append(y_pos)
            Red_List.append(List)
            imageFrame = cv2.rectangle(
                imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2
            )

            cv2.putText(
                imageFrame,
                "Red Colour",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
            )

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(
        green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > min_working_area and area < max_working_area:
            x, y, w, h = cv2.boundingRect(contour)
            # store the position and color of the box in a common array
            x_pos = (x + w) / 2
            y_pos = (y + h) / 2
            List = []
            List.append(x_pos)
            List.append(y_pos)
            Green_List.append(List)
            print(x, y)
            imageFrame = cv2.rectangle(
                imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2
            )

            cv2.putText(
                imageFrame,
                "Green Colour",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
            )

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(
        blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > min_working_area and area < max_working_area:
            x, y, w, h = cv2.boundingRect(contour)
            # store the position and color of the box in a common array
            x_pos = (x + w) / 2
            y_pos = (y + h) / 2
            List = []
            List.append(x_pos)
            List.append(y_pos)
            Blue_List.append(List)
            imageFrame = cv2.rectangle(
                imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2
            )

            cv2.putText(
                imageFrame,
                "Blue Colour",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 0, 0),
            )

    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)

    # make an unsorted array of the x positions of the boxes detected, 1 represents blue/red, 0 represents green
    FinalPos = []
    if Team and len(Green_List) >= 2 and len(Blue_List >= 2):
        FinalPos = [
            ["0", Green_List[0][0]],
            ["0", Green_List[1][0]],
            ["0", Blue_List[0][0]],
            ["0", Blue_List[1][0]],
        ]
    elif not (Team) and len(Green_List) >= 2 and len(Red_List >= 2):
        FinalPos = [
            ["0", Green_List[0][0]],
            ["0", Green_List[1][0]],
            ["0", Red_List[0][0]],
            ["0", Red_List[1][0]],
        ]
    finalString = ""
    if len(FinalPos):
        # sort array and convert into a string
        FinalPos_sorted = sorted(FinalPos, key=lambda x: x[1])
        data = np.array(FinalPos_sorted)
        str1 = ""
        finalString = str1.join(data[:, 0:1].squeeze().tolist())
    print(finalString)

    # Program Termination
    if cv2.waitKey(10) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
