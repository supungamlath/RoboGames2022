import cv2
import numpy as np

class ColorDetector:
    def __init__(self, camera):
        self.camera = camera
        self.hex_to_names = {
            "#00ffff": "aqua",
            "#000000": "black",
            "#0000ff": "blue",
            "#ff00ff": "fuchsia",
            "#008000": "green",
            "#808080": "grey",
            "#00ff00": "lime",
            "#800000": "maroon",
            "#000080": "navy",
            "#808000": "olive",
            "#800080": "purple",
            "#ff0000": "red",
            "#c0c0c0": "silver",
            "#008080": "teal",
            "#ffffff": "white",
            "#ffff00": "yellow",
        }

    @staticmethod
    def hex_to_rgb(hex_value):
        return tuple(
            map(
                lambda s: int(s, 16),
                (hex_value[1:3], hex_value[3:5], hex_value[5:7]),
            )
        )

    def closestColour(self, requested_colour):
        min_colours = {}
        for key, name in self.hex_to_names.items():
            r_c, g_c, b_c = self.hex_to_rgb(key)
            rd = (r_c - requested_colour[0]) ** 2
            gd = (g_c - requested_colour[1]) ** 2
            bd = (b_c - requested_colour[2]) ** 2
            min_colours[(rd + gd + bd)] = name
        return min_colours[min(min_colours.keys())]

    def testColorInCameraRow(self, test_colors, row):
        # image = self.camera.getImage()
        # width = self.camera.getWidth()
        # height = self.camera.getHeight()
        # area = width * height
        # blue, green, red = 0, 0, 0
        # for h in range(height):
        #     for w in range(width):
        #         blue += self.camera.imageGetBlue(image, width, w, h)
        #         green += self.camera.imageGetGreen(image, width, w, h)
        #         red += self.camera.imageGetRed(image, width, w, h)
        # color_name = self.closestColour((red // area, green // area, blue // area))
        # print(color_name)
        # return color_name in test_colors
        
        self.findRectangle()

    def get_image_from_camera(self):
        """
        Take an image from the camera device and prepare it for OpenCV processing:
        - convert data type,
        - convert to RGB format (from BGRA), and
        - rotate & flip to match the actual image.
        """
        img = self.camera.getImageArray()
        img = np.asarray(img, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        return cv2.flip(img, 1)

    def findRectangle(self):
        image = self.get_image_from_camera()
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of colors in the HSV color space to select (red-orange in this case)
        lower_color = np.array([0, 70, 50])
        upper_color = np.array([10, 255, 255])

        # Create a mask for the image to only select red-orange pixels
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Apply the mask to the image
        filtered_image = cv2.bitwise_and(image, image, mask=mask)

        # Convert the filtered image to grayscale
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

        # Perform Canny edge detection
        edges = cv2.Canny(gray, 50, 150)

        # Find contours in the image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through the contours and draw the rectangle around each one
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 1)

        # Show the image with the rectangles
        cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
        
        # Using resizeWindow()
        cv2.resizeWindow("Resized_Window", 520, 390)
        
        # Displaying the image
        cv2.imshow("Resized_Window", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

