#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# from cv2 import aruco

class ArenaProcessor(Node):
    def __init__(self):
        super().__init__('arena_processor')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', 
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.stop_processing = False

    def image_callback(self, msg):
        self.get_logger().info('Receiving the video frame')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if cv_image is None:
            print("Image conversion failed")
            return
        #display the image being processed
        # cv2.imshow("Processing Image", cv_image)
        key = cv2.waitKey(1)
        if key == 27:  # 'ESC' key
            self.stop_processing = True
            return
        #ArUco markers
        self.process_image(cv_image)

    # function to order points correctly for perspective transformation
    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left
        return rect

    # function to apply the four-point perspective transform
    def four_point_transform(self, image, pts):
        (h, w) = image.shape[:2]
        rect = self.order_points(pts)
        (tl, tr, br, bl) = rect
        dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype="float32")
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (w, h))
        return warped
        

    def detect_obstacle(self, gray, warped):
        # Convert the image to binary (thresholding)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
        # cv2.imshow("Threshold", thresh)
        # contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # blank image to draw the detected obstacles (racks)
        obstacle_map = np.zeros_like(gray)
        # set the minimum area threshold to filter out small noise
        min_area_threshold = 500
        total_obstacles = 0

        # blank color image for labeled obstacles
        labeled_obstacles_img = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)

        # Open the file to save obstacle coordinates
        with open("obstacle_coordinates.txt", "w") as file:
            # Loop through the contours to filter and identify racks
            for contour in contours:
                area = cv2.contourArea(contour)
                # contours that are above a minimum area threshold
                if area > min_area_threshold:
                    cv2.drawContours(obstacle_map, [contour], -1, 255, -1)  # Fill the contour
                    total_obstacles += 1

                    # Get the centroid of the contour for labeling
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = contour[0][0]

                    # Label the obstacle on the labeled_obstacles_img
                    cv2.putText(labeled_obstacles_img, f"{total_obstacles}", (cx, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Step 1: initial Contour Approximation
                    epsilon = 0.005 * cv2.arcLength(contour, True)  # Fine-tune 0.005 as needed
                    approx_corners = cv2.approxPolyDP(contour, epsilon, True)

                    # Step 2: filter based on angle to remove slight distortions
                    filtered_corners = []
                    for i in range(len(approx_corners)):
                        # Get three consecutive points
                        prev_point = approx_corners[i - 1][0]
                        curr_point = approx_corners[i][0]
                        next_point = approx_corners[(i + 1) % len(approx_corners)][0]

                        # Calculate vectors
                        v1 = np.array(curr_point) - np.array(prev_point)
                        v2 = np.array(next_point) - np.array(curr_point)

                        # the angle between the vectors
                        angle = np.degrees(np.arccos(
                            np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-5)
                        ))

                        # we will keep the points with significant angle changes
                        if angle < 160:  # threshold for sensitivity
                            filtered_corners.append(curr_point)

                    # Save the filtered corner points to the file
                    file.write(f"Obstacle {total_obstacles}:\n")
                    for i, point in enumerate(filtered_corners):
                        file.write(f"Corner {i + 1}: {tuple(point)}\n")
                    file.write("\n")

        # # Loop through the contours to filter and identify racks
        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     # Consider only contours that are above a minimum area threshold
        #     if area > min_area_threshold:
        #         cv2.drawContours(obstacle_map, [contour], -1, 255, -1)  # Fill the contour
        #         total_obstacles += 1
        # Print the total number of detected obstacles
        self.get_logger().info(f"Total number of obstacles detected: {total_obstacles}")

        # Invert the colors: Convert white obstacles to black, and the rest of the area to white
        inverted_obstacle_map = cv2.bitwise_not(obstacle_map)

        # Save the inverted bitmap as a 2D image
        # cv2.imwrite("2D_bit_map.png", inverted_obstacle_map)
        # cv2.imshow("Detected Obstacles (Inverted)", inverted_obstacle_map)
        
        # Inflate the obstacles using dilation
        kernel = np.ones((10, 10), np.uint8)
        inflated_obstacle_map = cv2.dilate(obstacle_map, kernel, iterations=1)

        # Create only the inflation layer by subtracting the original obstacles
        inflation_layer = cv2.subtract(inflated_obstacle_map, obstacle_map)

        # Convert the inflation layer to BGR and color it red
        inflation_layer_bgr = cv2.cvtColor(inflation_layer, cv2.COLOR_GRAY2BGR)
        inflation_layer_bgr[inflation_layer > 0] = (0, 0, 255)  # Set inflation areas to red

        # Overlay the inflation layer on the warped image
        inflated_overlay = cv2.addWeighted(warped, 1, inflation_layer_bgr, 0.5, 0)
        
        # Display and save the result
        cv2.imshow("Inflated Obstacle Layer", inflated_overlay)
        cv2.imshow("Labeled Obstacles", labeled_obstacles_img) 
        # self.get_logger().info(f"Inflated overlay dimensions (Height, Width, Channels): {inflated_overlay.shape}")
        # cv2.waitKey(1)
        # return inflation_layer
        return inverted_obstacle_map

    def process_image(self, image):
        # print("check")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite("gray.jpg",gray)
        #to load ArUco dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # print(ids,corners)
        # if ids is None:
        #     self.get_logger().info("No ArUco markers detected")
        #     return
        # elif len(ids) < 4:
        #     self.get_logger().info("Less than 4 markers detected.")
        #     return
        
        # if ids is None or len(corners) == 0:
        #     self.get_logger().info("No ArUco markers detected")
        #     return

        # draw bounding boxes around detected markers
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        # cv2.imshow("ArUco Markers", image)
        # cv2.waitKey(0)

        ids = ids.flatten()
        sorted_indices = np.argsort(ids)
        sorted_corners = [corners[i] for i in sorted_indices]

        # extract the corner points in order
        top_left = sorted_corners[0][0][0][0]+1,sorted_corners[0][0][0][1]+1  # Top-left
        top_right = sorted_corners[1][0][1][0]-1,sorted_corners[1][0][1][1]+1  # Top-right
        bottom_right = sorted_corners[2][0][2][0]-1,sorted_corners[2][0][2][1]-1  # Bottom-right
        bottom_left = sorted_corners[3][0][3][0]-1,sorted_corners[3][0][3][1]+1  # Bottom-left

        pts = np.array([top_left, top_right, bottom_right, bottom_left], dtype="float32")
        warped = self.four_point_transform(image, pts)

        warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        # total_area, total_obstacles = self.detect_obstacle(warped_gray)
        # cv2.imshow("Warped Image", warped)

        # Obstacle detection
        bit_image = self.detect_obstacle(warped_gray, warped)
        cv2.imshow("Bit Image of Arena", bit_image)
        # self.get_logger().info(f"Bit image dimensions (Height, Width): {bit_image.shape[:2]}")
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    arena_processor = ArenaProcessor()
    try:
        rclpy.spin(arena_processor)
    except KeyboardInterrupt:
        arena_processor.get_logger().info("Node interrupted. Shutting down...")
    finally:
        # Clean up and shutdown
        arena_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()