import cv2
import numpy as np


# Global variable that holds the BGR values of your detection for the code to output
color_detection_list = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 0)]


import cv2
import numpy as np
from serial import Serial as _Ser




def OffSetColors(frame, blue, green, red):
    frame[:,:,0] = frame[:,:,0] + blue
    frame[:,:,1] = frame[:,:,1] + green
    frame[:,:,2] = frame[:,:,2] + red



#webcam = cv2.VideoCapture(1) 
#serial = _Ser("COM8")
#serial.baudrate = 115200
#serial.timeout = 1
    


# You should have no reason to modify this class
class ColorDisplayWindow:
    def __init__(
        self,
        window_height=300,
        window_width=2200,
        window_name="displayWindow",
        group_name="Group name here",
    ):
        global color_detection_list
        self.group_name = group_name
        self.window_height = window_height
        self.window_width = window_width
        self.window_name = window_name

        self.title_height = 100
        self.window = np.zeros(
            (window_height + self.title_height, window_width, 3), dtype=np.uint8
        )

        self.color_detection_count = len(color_detection_list)
        self.color_box_width = int(self.window_width / self.color_detection_count)
        self.color_box_height = self.window_height

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 2
        self.font_thickness = 5

    # helper function to draw the color boxes
    def display(self):
        global color_detection_list

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
            color = color_detection_list[i]
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

        cv2.imshow(self.window_name, self.window)


if __name__ == "__main__":
    # TODO: Change your team's name and modify the size of the window to your liking
    color_display_1 = ColorDisplayWindow(
        window_height=500,
        window_width=1500,
        window_name="displayWindow1",
        group_name="RDC Team 2",
    )

    # for fun only


    while True:

        # TODO: You should constantly update the values of color_detection_list based on the BGR values you get
        color_detection_list = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        regions = []
        colors = []
        colorList = []
        #_, frame = webcam.read() 
        frame = cv2.imread('Test2.png')
        i = 0
        OffSetColors(frame,0,-20,-50)
        height, width, _ = frame.shape
        region_width = width//4
        start = 0
        cropped_frame = frame[height//2:,:]
    
        for i in range(4):
            
            cv2.imshow('cropped_frame', cropped_frame)
            #frame = cv2.imread('Test2.png')
            regions.append(cropped_frame[:,start:(start + region_width)])
            average_color_row = np.average(regions[i], axis=0)
            average_color = np.average(average_color_row, axis=0)
            colors.append(average_color)
            #d_img = np.ones((312, 312, 3), dtype=np.uint8)
            #d_img[:, :] = average_color
            start = start + region_width
            string = str(i) + "frame"
            cv2.imshow(string, regions[i])

        
        
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

        for j in range(4):
            match colorList[j]:

                case 'b':
                    color_detection_list[j] = (255,0,0)

                case 'g':
                    color_detection_list[j] = (0,255,0)

                case 'r':
                    color_detection_list[j] = (0,0,255)

        
        print(colorList)

        #send data over bluetooth
        j = 1
        data = ""
        for char in colorList:
            if(char == 'b' or char == 'r'):
                data = data+str(j)
            j = j+2  
        data = "t"+data    
        b = bytes(data, 'utf-8')
        print(data)
        #serial.write(b)    

        


        cv2.imshow('frame', frame)
        




        # Updates display
        color_display_1.display()

        if cv2.waitKey(10) & 0xFF == ord("q"):  # waits for 'q' key to be pressed
            break

    cv2.destroyAllWindows()
