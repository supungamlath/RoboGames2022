import sys
sys.path.append('../dave_controller')

from backups.color_detector import ColorDetector
import cv2, glob

def test_houghlines():
    cd = ColorDetector(None)

    # Define the directory containing the images
    directory = 'images/'

    # Get a list of all the images in the directory
    image_list = glob.glob(directory + '*.png')

    # Iterate through the images in the list
    for image_path in image_list:
        # Load the image and convert it to HSV color space
        image = cv2.imread(image_path)
        image, line_distances = cd.findBlackLinesInGreenBg(image)
        print(line_distances)
        # cd.showImage(image)

if __name__ == "__main__":
    test_houghlines()