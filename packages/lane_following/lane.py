import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import numpy as np 


def region_of_interest(img, vertices):
    # Define a blank matrix that matches the image height/width.
    mask = np.zeros(img.shape[:2], dtype="uint8")
      
    # Fill inside the polygon
    cv2.fillPoly(mask, vertices, 255)
    
    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, img, mask=mask)

    return masked_image


def HSV_filter(img, hsv_values):

    # Set minimum and max HSV values to display
    lower = np.array([hsv_values['hMin'], hsv_values['sMin'], hsv_values['vMin']])
    upper = np.array([hsv_values['hMax'], hsv_values['sMax'], hsv_values['vMax']])

    # Create HSV Image and threshold into a range.
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    output = cv2.bitwise_and(img, img, mask= mask)
    
    return cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)


def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    # If there are no lines to draw, exit.
    if lines is None:
        return
    
    # Make a copy of the original image.
    img = np.copy(img)

    # Loop over all lines and draw them on the blank image.
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)


    # Return the modified image.
    return img


def find_max_line_lenght(lines):
    max_line = lines[0,0]
    max_len = 0

    if len(lines) == 0:
        return 
    
    for line in lines:
        lenght = np.sqrt((line[0,0]-line[0,2])**2 + (line[0,1]-line[0,3])**2)
        print(lenght)
        if lenght > max_len:
            max_len = lenght
            max_line = line

    return [max_line]


# reading in an image
img = cv2.pyrDown(cv2.imread('test1.png'))
img = cv2.pyrDown(cv2.pyrDown(img))


(height, width, _) = img.shape


region_of_interest_vertices = [
    (0, height),
    (0, height-50),
    (width // 2 -25, 50),
    (width // 2 +25, 50),
    (width, height-50),
    (width, height),
]


cropped_image = region_of_interest(
    img,
    np.array([region_of_interest_vertices], np.int32),
)


yellow_HSV_filter =  {'hMin':21, 'sMin':68, 'vMin':0, 'hMax':111, 'sMax':201, 'vMax':255}
white_HSV_filter = {'hMin':92, 'sMin':21, 'vMin':124, 'hMax':121, 'sMax':67, 'vMax':255}


yellow_img = HSV_filter(img, yellow_HSV_filter)
white_img = HSV_filter(img, white_HSV_filter)


# Call Canny Edge Detection here.
yellow_edges = cv2.Canny(yellow_img, 100, 300)
white_edges = cv2.Canny(white_img, 100, 300)


yellow_line_img = region_of_interest(
    yellow_edges,
    np.array(
        [region_of_interest_vertices],
        np.int32
    ),
)

white_line_img = region_of_interest(
    white_edges,
    np.array(
        [region_of_interest_vertices],
        np.int32
    ),
)

white_lines = cv2.HoughLinesP(
    white_line_img,
    rho=6,
    theta=np.pi / 180,
    threshold=100,
    lines=np.array([]),
    minLineLength=40,
    maxLineGap=25
)

yellow_lines = cv2.HoughLinesP(
    yellow_line_img,
    rho=6,
    theta=np.pi / 180,
    threshold=100,
    lines=np.array([]),
    minLineLength=40,
    maxLineGap=25
)


white_line = find_max_line_lenght(white_lines)
yellow_line = find_max_line_lenght(yellow_lines)

lines = np.concatenate((white_line, yellow_line), axis=0)

slope_right = (lines[0,0,3] - lines[0,0,1]) / (lines[0,0,2] - lines[0,0,0])
slope_left = (lines[1,0,3] - lines[1,0,1]) / (lines[1,0,2] - lines[1,0,0])


max_y = height
min_y = int(height // 2)

left_x_start = int((max_y - lines[1,0,1]) / slope_left + lines[1,0,0])
left_x_end = int((min_y - lines[1,0,1]) / slope_left + lines[1,0,0])

right_x_start = int((max_y - lines[0,0,1]) / slope_right + lines[0,0,0])
right_x_end = int((min_y - lines[0,0,1]) / slope_right + lines[0,0,0])


line_image = draw_lines(
    img,
    [[
        [left_x_start, max_y, left_x_end, min_y],
        [right_x_start, max_y, right_x_end, min_y],
    ]],
    thickness=5,
)

plt.figure()
plt.imshow(line_image)

plt.figure()
plt.imshow(cropped_image)

# plt.figure()
# plt.imshow(yellow_line_img)

# plt.figure()
# plt.imshow(white_line_img)

plt.show()