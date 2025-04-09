import pyrealsense2 as rs
import cv2
import numpy as np

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

def is_rectangle(approx):
    return len(approx) == 4 and cv2.isContourConvex(approx)

def is_circle(contour, approx):
    if len(approx) >= 7:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return False
        circularity = 4 * np.pi * area / (perimeter ** 2)
        return 0.7 < circularity < 1.2  # Roughly circular
    return False

def get_center_of_contour(contour):
    M = cv2.moments(contour)
    if M["m00"] != 0:
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        return (center_x, center_y)
    return None

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.4)
        edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            shape_type = None

            if is_rectangle(approx):
                shape_type = "Rectangle"
            elif is_circle(contour, approx):
                shape_type = "Circle"

            if shape_type:
                center = get_center_of_contour(contour)
                if center:
                    center_x, center_y = center
                    # Draw shape and center
                    cv2.drawContours(color_image, [approx], 0, (0, 255, 0), 3)
                    cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Get depth and convert to meters
                    depth_value = depth_image[center_y, center_x]
                    distance = depth_value / 1000.0

                    # Display shape type and distance
                    cv2.putText(color_image, f"{shape_type} - {distance:.2f}m",
                                (center_x + 10, center_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow('Shapes with Centers and Distance', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
