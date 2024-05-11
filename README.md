# Pose Corrector Node

A ros node that estimates the 6D pose of an metallic object (named `inboard`) object using RGB-D images.

## Usage

Launch the pose estimator:

```bash
roslaunch pose_corrector run.launch
```

## Task 1 and 2

The processed images can be downloaded from [Dropbox](https://www.example.com).  
The dataset images were processed using `pose_corrector_task_1_and_2.py`.

### Depth Segmentation and PCA

The images are segmented using the depth information.
To estimate the onboard angle PCA is performed on the image mask.

![segmentation_pca_example.png](docs/imgs/segmentation_pca_example.png)

#### Images with hands

Images containing hands are discarded and not utilized for inboard angle estimation. The hands are detected based on skin color pixels within the masked images.
In the future hands can be segmented and removed from the image masks to enable estimation during hand-object interactions.

## Task 3

![pose_estimation](docs/imgs/pose_estimation.gif)

To demo the pose estimator online, a python ros node `pose_corrector_node.py` was implemented. The node subscribes to the topics from the provided rosbag.
To run the pose estimation node, see [Usage](##Usage).

### ICP Registration

To estimate the pose of the observed inboard, the point cloud model of the inboard is registered  with the point cloud captured by the camera. The icp registration pipeline is only applied to images  where the inboard is upright enough (good), as determined by angle detector in task 2. The detected angle is used to provide an initial transformation for the ICP algorithm.

![ICP](docs/imgs/icp_example.png)

*The camera pointcloud (blue) aligned with the inboard model pointcloud (yellow)*

### C++ Node

A simple RGB-D image subscriber in C++ can be found in `pose_corrector_node.cpp`. The whole pose estimation pipeline was not implemented in C++.  
To run the C++ image subscriber:

```bash
roslaunch pose_corrector cpp.launch
```