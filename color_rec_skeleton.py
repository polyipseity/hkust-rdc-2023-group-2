#!/usr/bin/env python3
import cv2
import numpy as np
from tkinter import Tk

Team = 1
min_area = 30000

# capture the video
webcam = cv2.VideoCapture(0) 


# You should have no reason to modify this class
class ColorDisplayWindow:
    def __init__(
        self,
        group_name="One More Martin",
        window_name="displayWindow1",
        window_height=Tk().winfo_screenheight(),
        window_width=Tk().winfo_screenwidth(),
    ):
        self.group_name = group_name
        self.window_name = window_name

        self.title_height = 100
        self.window_height = window_height - self.title_height
        self.window_width = window_width

        self.window = np.zeros((window_height, window_width, 3), dtype=np.uint8)

        self.color_detection_count = 4
        self.color_box_width = int(self.window_width / self.color_detection_count)
        self.color_box_height = self.window_height

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 2
        self.font_thickness = 5

    # helper function to draw the color boxes
    def display(self, __color_detection_list):
        # add a white title bar
        self.window[self.window_height :, :] = (255, 255, 255)

        # add the title "RDC 2023 Color Detection"
        title = "RDC 2023 Color Detection : " + self.group_name
        title_size = cv2.getTextSize(
            title, self.font, self.font_scale, self.font_thickness
        )[0]
        title_x = (self.window_width - title_size[0]) // 2
        title_y = self.window_height + ((self.title_height + title_size[1]) // 2)
        cv2.putText(
            self.window,
            title,
            (title_x, title_y),
            self.font,
            self.font_scale,
            (0, 0, 0),
            self.font_thickness,
            cv2.LINE_AA,
        )

        for i in range(self.color_detection_count):
            # draw the color box
            color = __color_detection_list[i]
            color_box = np.zeros(
                (self.color_box_height, self.color_box_width, 3), dtype=np.uint8
            )
            color_box[:] = color
            self.window[
                : self.color_box_height,
                i * self.color_box_width : (i + 1) * self.color_box_width,
            ] = color_box

            # prepare the number at the center of each color box
            number = str(i + 1)
            text_size = cv2.getTextSize(
                number, self.font, self.font_scale, self.font_thickness
            )[0]
            text_x = (i * self.color_box_width) + (
                (self.color_box_width - text_size[0]) // 2
            )
            text_y = (self.color_box_height + text_size[1]) // 2

            # Draw a black square beneath the number
            square_size = text_size[1] + 40  # Add some padding
            square_top_left = (
                text_x - int(0.5 * (square_size // 2)),
                text_y - 3 * (text_size[1] // 2),
            )
            square_bottom_right = (
                text_x + int(1.5 * (square_size // 2)),
                text_y + (text_size[1] // 2),
            )
            cv2.rectangle(
                self.window,
                square_top_left,
                square_bottom_right,
                (0, 0, 0),
                -1,
                cv2.LINE_AA,
            )

            # Draw the number
            cv2.putText(
                self.window,
                number,
                (text_x, text_y),
                self.font,
                self.font_scale,
                (255, 255, 255),
                self.font_thickness,
                cv2.LINE_AA,
            )

        cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(
            self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )
        cv2.imshow(self.window_name, self.window)


if __name__ == "__main__":
    # TODO: Change your team's name
    color_display_1 = ColorDisplayWindow(
        group_name="RDC Team 5",
    )

    color_detection_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

    while True:
        # TODO: You should constantly update the values of color_detection_list based on the BGR values you get
        
        Red = []
        Blue = []
        Green = [] 
        
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
        red_upper = np.array([180, 255, 255], np.uint8) 
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
        res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask) 
        
        # For green color 
        green_mask = cv2.dilate(green_mask, kernel) 
        res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                    mask = green_mask) 
        
        # For blue color 
        blue_mask = cv2.dilate(blue_mask, kernel) 
        res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = blue_mask) 

        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > min_area): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2) 
                
                cv2.putText(imageFrame, "Red Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (0, 0, 255))
                
                Red.append([(x + w) / 2, (y + h) / 2])

        # Creating contour to track green color 
        contours, hierarchy = cv2.findContours(green_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > min_area): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 255, 0), 2) 
                
                cv2.putText(imageFrame, "Green Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, (0, 255, 0))
                Green.append([(x + w) / 2, (y + h) / 2]) 

        # Creating contour to track blue color 
        contours, hierarchy = cv2.findContours(blue_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > min_area): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (255, 0, 0), 2) 
                
                cv2.putText(imageFrame, "Blue Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, (255, 0, 0)) 
                Blue.append([(x + w) / 2, (y + h) / 2])
                
        print("Red: ", Red) 
        print("Blue: ", Blue) 
        print("Green: ", Green)
        
        color_ranges = []
        
        # Red Team
        if Team and len(Red) and len(Green):
            print("found red and green")
            for x, y in Red:
                color_ranges.append([x, 'r'])
            for x, y in Green:
                color_ranges.append([x, 'g'])
        # Blue Team		
        if not Team and len(Blue) and len(Green):
            print("found blue and greed")
            for x, y in Blue:
                color_ranges.append([x, 'b'])
            for x, y in Green:
                color_ranges.append([x, 'g'])
        



        color_ranges = sorted(color_ranges, key=lambda x: x[0])
        print(color_ranges)
        width = 640
        final = []
        message = ""
        j = 0 # color_ranges indexer
        if len(color_ranges):
            for i in range(width):
                final.append(color_ranges[j][1])
                if i > color_ranges[j][0] and j != len(color_ranges)-1:
                    j += 1
        print(final)
        if len(final):
            message = final[160] + final[320] + final[480] + final[635]
            print(message)


        color_detection_list = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        index = 0
        for c in message:
            if c == 'r':
                color_detection_list[index] = [0, 0, 235]
            if c == 'g':
                color_detection_list[index] = [0, 235, 0]
            if c == 'b':
                color_detection_list[index] = [235, 0, 0]
            index += 1

        


        # Updates display
        color_display_1.display(color_detection_list)

        if cv2.waitKey(10) & 0xFF == ord("q"):  # waits for 'q' key to be pressed
            break

    cv2.destroyAllWindows()
