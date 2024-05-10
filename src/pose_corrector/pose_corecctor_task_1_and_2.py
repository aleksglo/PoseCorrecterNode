import cv2
from pathlib import Path

from utils import *

THRESHOLD_ROTATION = 30

DIRECTORY_COLOR_IMAGES = Path('/home/aleks/pose_corrector_data/dataset/inboard_rgbd/rgb')
DIRECTORY_DEPTH_IMAGES = Path('/home/aleks/pose_corrector_data/dataset/inboard_rgbd/depth')
OUTPUT_DIRECTORY = Path('/home/aleks/pose_corrector_data/output')

output_path_masked_images = OUTPUT_DIRECTORY / 'masked_images'
output_path_detected_hands = OUTPUT_DIRECTORY / 'detected_hands'
output_path_inboard_pose_good = OUTPUT_DIRECTORY / 'inboard_pose_good'
output_path_inboard_pose_bad = OUTPUT_DIRECTORY / 'inboard_pose_bad'

color_image_paths = sorted(DIRECTORY_COLOR_IMAGES.glob("*.png"))
depth_image_paths = sorted(DIRECTORY_DEPTH_IMAGES.glob("*.png"))

directories = [output_path_masked_images, output_path_detected_hands, output_path_inboard_pose_good, output_path_inboard_pose_bad]
[directory.mkdir(parents=True, exist_ok=True) for directory in directories]

for color_image_path, depth_image_path in zip(color_image_paths, depth_image_paths):

    color_image_bgr = cv2.imread(str(color_image_path))
    color_image_rgb = cv2.cvtColor(color_image_bgr, cv2.COLOR_BGR2RGB)
    depth_image = cv2.imread(str(depth_image_path), cv2.IMREAD_ANYDEPTH)

    mask, masked_color_image, masked_depth_image = mask_images_based_on_distance(color_image_bgr, depth_image, mask_end_effector=True)
    masked_color_image_rgb = cv2.cvtColor(masked_color_image, cv2.COLOR_BGR2RGB)
    cv2.imwrite(str(output_path_masked_images / f'masked_{color_image_path.name}'), masked_color_image) 

    # discard all images with hands
    if detect_hands_by_skin_color(masked_color_image_rgb, image_bgr=color_image_bgr):
        color_image_rgb = draw_hand_detected_marker_on_image(color_image_bgr)
        cv2.imwrite(str(output_path_detected_hands / color_image_path.name), color_image_bgr) 
        continue

    angle, mean, rotation_vector = get_inboard_rotation_around_vertical_axis(mask)
    color_image_bgr = draw_eigenvector_on_image(color_image_bgr, mean, rotation_vector, angle)

    if not inboard_angle_valid(angle, threshold_good_bad_orientation=THRESHOLD_ROTATION):
        cv2.imwrite(str(output_path_inboard_pose_bad / color_image_path.name), color_image_bgr) 
    else:
        cv2.imwrite(str(output_path_inboard_pose_good / color_image_path.name), color_image_bgr) 



