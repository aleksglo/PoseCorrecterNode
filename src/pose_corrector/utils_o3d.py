import open3d as o3d
import numpy as np
import copy
import yaml


def load_inboard_model_pcl(path_mesh, path_mesh_texture):
    mesh = o3d.io.read_triangle_mesh(path_mesh, True)
    texture = o3d.io.read_image(path_mesh_texture)
    mesh.textures = [texture]
 
    pcl = mesh.sample_points_uniformly(number_of_points=100000)

    # scale model to mm
    pcl.scale(0.001, center=np.array([0,0,0]))
    pcl.estimate_normals()
    return pcl

def read_camera_intrinsic(camera_intrinsic_yaml_path):

    with open(camera_intrinsic_yaml_path) as stream:
        yaml_camera_parameters = yaml.safe_load(stream)
    
    width = yaml_camera_parameters['width']
    height = yaml_camera_parameters['height']

    intrinsic_camera_matrix = np.reshape(yaml_camera_parameters['K'], (3, 3))

    fx = intrinsic_camera_matrix[0,0]
    fy = intrinsic_camera_matrix[1,1]
    cx = intrinsic_camera_matrix[0,2]
    cy = intrinsic_camera_matrix[1,2]

    intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height,  fx, fy, cx, cy)
    return intrinsics

def create_camera_pcl(color_image, depth_image, intrinsics):
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image)
    camera_pcl = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)
    #camera_pcl.estimate_normals()
    return camera_pcl

def pcl_registration_icp(source, target, init_transformation, threshold=0.01):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     source, target, threshold, init_transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())

    return reg_p2p.transformation

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([source_temp, target_temp, mesh_coord_frame])

def get_transformation_matrix(rot_matrix, translation):
    T = np.eye(4)
    T[:3, :3] = rot_matrix
    T[:3, 3] = translation
    return T

def get_initial_inboard_transformation(angle):
    # translation found by visual inspection of point cloud data, in the future translation can be extracted from end effector frame + offset 
    translation = np.array([0.05, 0.13, 0.42])
    rot_matrix = o3d.geometry.get_rotation_matrix_from_xyz((np.pi / 2, (np.pi / 2) - angle , 0))
    return get_transformation_matrix(rot_matrix, translation)

def downsample_pcls(source_pcl, target_pcl, voxel_size=0.005):
    source_pcl = source_pcl.voxel_down_sample(voxel_size)
    camera_pcl = target_pcl.voxel_down_sample(voxel_size)
    return source_pcl, target_pcl
