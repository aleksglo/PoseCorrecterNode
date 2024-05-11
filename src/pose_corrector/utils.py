import numpy as np
import cv2
import math



def mask_images_based_on_distance(color_image, depth_image, min_distance = 250, max_distance=450, mask_end_effector=True):
   
    mask_distance_thresholding = (depth_image > min_distance) & (depth_image < max_distance)

    if mask_end_effector:
        end_effector_mask = get_end_effector_mask()
        mask = np.bitwise_and(mask_distance_thresholding, end_effector_mask)
    else:
        mask = mask_distance_thresholding

    masked_color_image = apply_mask(color_image, mask)
    masked_depth_image = apply_mask(depth_image, mask)

    return mask, masked_color_image, masked_depth_image

def get_end_effector_mask():
    image_width = 640
    image_height = 480

    mask = np.full((image_height, image_width), True)

    # end effector region
    y_start, y_end = 465, 480
    x_start, x_end = 324, 442

    mask[y_start:y_end, x_start:x_end] = False

    return mask

def apply_mask(image, mask):
    masked_image = np.zeros_like(image)
    masked_image[mask] = image[mask]
    return masked_image

def detect_hands_by_skin_color(image_rgb, detected_pixel_threshold = 200, image_bgr=None):

    image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

    min_HSV = np.array([0, 58, 30], dtype = "uint8")
    max_HSV = np.array([33, 255, 255], dtype = "uint8")

    skin_mask = cv2.inRange(image_hsv, min_HSV, max_HSV)
    skin_mask = cv2.bitwise_not(skin_mask)

    unique, counts = np.unique(skin_mask, return_counts=True)
    detected_pixels = counts[0]

    if detected_pixels > detected_pixel_threshold and detected_pixels < 307000 :
        if image_bgr is not None:
            cv2.putText(image_bgr, f'hand pixel detected: {detected_pixels}',(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        return True
    else:
        return False
    
def get_inboard_rotation_around_vertical_axis(mask):  
    mask_pixel_coords = get_mask_pixel_coordinates(mask)
    mean, eigenvectors, eigenvalues = compute_pca(mask_pixel_coords)

    rotation_vector = eigenvectors[0]

    rotation_vector = make_rot_vec_negative_in_y_direction(rotation_vector)
    angle = math.atan2(rotation_vector[1], rotation_vector[0]) * -1 # orientation in radians, -1 because y axis is inverted on image
    
    return angle, mean, rotation_vector

def get_mask_pixel_coordinates(mask):
    height, width = mask.shape
    x_coords, y_coords = np.meshgrid(range(width), range(height))
    mask_pixel_coords = np.column_stack((x_coords[mask != 0], y_coords[mask != 0]))
    return mask_pixel_coords

def compute_pca(mask_pixel_coords):
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(mask_pixel_coords.astype(float), None)
    return mean.squeeze(), eigenvectors, eigenvalues

def make_rot_vec_negative_in_y_direction(rotation_vector):
    # the eigenvector of the pca can point in positive y-direction, this makes sure it always points in negative direction
    if rotation_vector[1] > 0:
        rotation_vector *= -1
    return rotation_vector

def inboard_angle_valid(angle, threshold_good_bad_orientation=30):
    tresh = threshold_good_bad_orientation
    if (angle < math.radians(180 - tresh) and angle > math.radians(tresh)):
        return True
    else:
        return False

def draw_eigenvector_on_image(image, mean, eigenvector, angle):
    center_coordinates = mean.squeeze().astype(int)
    center_coordinates = (center_coordinates[0], center_coordinates[1])

    end_point = (mean.squeeze() + (eigenvector * 100)).astype(int)
    end_point = (end_point[0], end_point[1])
    end_point_off = (end_point[0] + 10, end_point[1] + 10)
 
    cv2.putText(image, f'angle: {math.degrees(angle):.1f} deg', end_point_off , cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
    cv2.arrowedLine(image, center_coordinates, end_point, color=(0,255,0), thickness=3)  
    cv2.circle(image, center_coordinates, radius=5, color=(255,0,0) , thickness=-1) 
    return image

def draw_hand_detected_marker_on_image(color_image_bgr):
    cv2.circle(color_image_bgr, (10, 10), radius=5, color=(0,0,255) , thickness=-1) 
    return color_image_bgr



