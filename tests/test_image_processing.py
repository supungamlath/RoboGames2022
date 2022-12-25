import sys
sys.path.append('../dave_controller')

# import cv2
# import numpy as np
# import glob

# def findRectangle(image_path):
#     image = cv2.imread(image_path)
    
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#     # Define the range of colors in the HSV color space to select (red-orange in this case)
#     lower_color = np.array([0, 70, 50])
#     upper_color = np.array([10, 255, 255])

#     # Create a mask for the image to only select red-orange pixels
#     mask = cv2.inRange(hsv, lower_color, upper_color)

#     # Apply the mask to the image
#     filtered_image = cv2.bitwise_and(image, image, mask=mask)

#     # Convert the filtered image to grayscale
#     gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)

#     # gray = cv2.GaussianBlur(gray, (3,3), 0)

#     # Perform Canny edge detection
#     edges = cv2.Canny(gray, 10, 150,apertureSize=3, L2gradient=True)

#     # Find contours in the image
#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     # Iterate through the contours and draw the rectangle around each one
#     for contour in contours:
#         x,y,w,h = cv2.boundingRect(contour)
#         cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 1)

#     # Show the image with the rectangles
#     cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL)
    
#     # Using resizeWindow()
#     cv2.resizeWindow("Resized_Window", 520, 390)
    
#     # Displaying the image
#     cv2.imshow("Resized_Window", image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()



# # Define the directory containing the images
# directory = 'images/'

# # Get a list of all the images in the directory
# image_list = glob.glob(directory + '*.png')

# # Iterate through the images in the list
# for image_path in image_list:
#     # Load the image and convert it to HSV color space
#     findRectangle(image_path)
