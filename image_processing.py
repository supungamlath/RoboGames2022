import cv2, numpy as np

def fillSingleWhitePixels(image):
    # Find all the single white pixels in the image
    rows, cols, _ = np.where(image == [255, 255, 255])
    for row, col in zip(rows, cols):
        # Check if the single white pixel is surrounded by a single color on all sides
        top_color = image[row-1, col]
        bottom_color = image[row+1, col]
        left_color = image[row, col-1]
        right_color = image[row, col+1]
        if (top_color == bottom_color).all() and (left_color == right_color).all() and (top_color == left_color).all():
            # Fill the single white pixel with the surrounding color
            image[row, col] = top_color
    return image

def drawPath(image):
    # Define a range of green colors in the HSV color space
    green = np.array([0, 255, 0])

    # Create a mask for the green colors
    mask = cv2.inRange(image, green, green)

    # Find the contours of the green regions in the image
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # cont_img = cv2.drawContours(image, contours, -1, (0, 0, 255), 1)
    # Iterate over the contours and draw lines 5 pixels thick using the green pixels
    # for contour in contours:
    #     if cv2.contourArea(contour) > 5:
    #         cv2.polylines(image, [contour], False, (0, 0, 255), 1)

    for cnt in contours:
        epsilon = 0.0001*cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # Delat threshold
        t = 1

        # n - Number of vertices
        n = approx.shape[0]

        if n < 800:
            continue

        print(approx.shape)
        for i in range(n):
            #      p1              p2
            #       *--------------*
            #       |
            #       |
            #       |
            #       *
            #      p0

            p0 = approx[(i+n-1) % n][0]    # Previous vertex
            p1 = approx[i][0]              # Current vertex
            p2 = approx[(i + 1) % n][0]    # Next vertex
            dx = p2[0] - p1[0]             # Delta pixels in horizontal direction
            dy = p2[1] - p1[1]             # Delta pixels in vertical direction

            # Fix x index of vertices p1 and p2 to be with same x coordinate ([<p1>, <p2>] form horizontal line).
            if abs(dx) < t:
                if ((dx < 0) and (p0[0] > p1[0])) or ((dx > 0) and (p0[0] < p1[0])):
                    p2[0] = p1[0]
                else:
                    p1[0] = p2[0]

            # Fix y index of vertices p1 and p2 to be with same y coordinate ([<p1>, <p2>] form vertical line).
            if abs(dy) < t:
                if ((dy < 0) and (p0[1] > p1[1])) or ((dy > 0) and (p0[1] < p1[1])):
                    p2[1] = p1[1]
                else:
                    p1[1] = p2[1]

            approx[i][0] = p1
            approx[(i + 1) % n][0] = p2
        cv2.drawContours(image, [approx], 0, (0, 0 , 255), 1)

    return image

def drawMaze(image):
    # image =  fillSingleWhitePixels(image)
    image = drawPath(image)
    return image
