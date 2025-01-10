#!/usr/bin/env python3
import cv2
import numpy as np
import argparse

# function to order points correctly for perspective transformation
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # Top-left
    rect[2] = pts[np.argmax(s)]  # Bottom-right
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # Top-right
    rect[3] = pts[np.argmax(diff)]  # Bottom-left
    return rect

# function to apply the four-point perspective transform
def four_point_transform(image, pts):
    (h, w) = image.shape[:2]
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (w, h))
    return warped

def detect_obstacle(gray):
    blank_image = np.zeros(gray.shape, dtype=np.uint8)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)[1]
    #cv2.imshow("Threshold", thresh)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    total_obstacles = 0
    total_area = 0
    area_arr = []

    # filter out small contours by area (adjust the minimum area as needed)
    min_area_threshold = 500  # based on the size of your cuboids
    valid_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area_threshold:
            # to check aspect ratio (width/height) to filter out noise
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            
            # set thresholds for aspect ratio to decide criteria for a valid obstacle
            if aspect_ratio < 3 and aspect_ratio > 0.3:
                valid_contours.append(contour)
                total_obstacles += 1
                total_area += area
                area_arr.append(area)

                # draw the contour
                approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                cv2.drawContours(blank_image, [approx], -1, (0, 255, 0), 5)

    print("Number of obstacles:", total_obstacles)
    print("Total area:", total_area)
    return total_area, total_obstacles  

def text_file(flattened_ids, total_obstacles, total_area):
    # Convert ids to a list if not empty
    if flattened_ids is not None and len(flattened_ids) > 0:
        flattened_ids = list(map(int, flattened_ids))
    else:
        flattened_ids = []
    with open("obstacles.txt", "w") as f:
        f.write(f"Aruco ID: {flattened_ids}\n")  # Ensure ArUco IDs are printed
        f.write(f"Obstacles: {int(total_obstacles)}\n")
        f.write(f"Area: {float(total_area)}\n")
    print("Text file created with ArUco IDs")
    print(type(flattened_ids))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True, help="Path to input image containing ArUCo tag")
    args = vars(ap.parse_args())

    # Step 1: Load and preprocess the image
    image = cv2.imread(args["image"])
    if image is None:
        print(f"Error: Unable to load image '{args['image']}'")
        return

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Step 2: Load ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    # Step 3: Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None or len(ids) < 4:
        print("Error: less than 4 ArUco markers detected.")
        return

    # Step 4: Draw bounding boxes around detected markers
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    #cv2.imshow("ArUco Markers", image)

    # Step 5: Define and apply perspective transformation
    ids = ids.flatten()
    sorted_indices = np.argsort(ids)
    sorted_corners = [corners[i] for i in sorted_indices]

    # extract the corner points in order
    top_left = sorted_corners[0][0][0][0]+1,sorted_corners[0][0][0][1]+1  # Top-left
    top_right = sorted_corners[1][0][1][0]-1,sorted_corners[1][0][1][1]+1  # Top-right
    bottom_right = sorted_corners[2][0][2][0]-1,sorted_corners[2][0][2][1]-1  # Bottom-right
    bottom_left = sorted_corners[3][0][3][0]-1,sorted_corners[3][0][3][1]+1  # Bottom-left

    pts = np.array([top_left, top_right, bottom_right, bottom_left], dtype="float32")
    warped = four_point_transform(image, pts)

    # Step 6: Obstacle detection
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    total_area, total_obstacles = detect_obstacle(warped_gray)

    # Step 7: Save results to a text file
    text_file(ids, total_obstacles, total_area)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()