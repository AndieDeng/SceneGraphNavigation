import pickle
import git
import numpy as np
import rerun as rr
import open3d as o3d
import quaternion
import magnum as mn
from concurrent.futures import ProcessPoolExecutor

# Load the pickle file containing the observations, rotations, translations, and camera info
def load_data(file_path):
    with open(file_path, "rb") as file:
        return pickle.load(file)

# Convert depth and color to point cloud with transformations
def depth_to_point_cloud_with_pose(depth, color, fx, fy, cx, cy, translation, rotation, voxel_size=0.05):
    height, width = depth.shape
    i, j = np.meshgrid(np.arange(width), np.arange(height), indexing='xy')
    x = (i - cx) * depth / fx
    y = (j - cy) * depth / fy
    z = depth
    valid = depth > 0
    points = np.stack((x[valid], y[valid], z[valid]), axis=-1)
    colors = color[valid] / 255.0  # Normalize colors

    # Transform points using rotation and translation
    R_image2camera = np.array([[1, 0., 0], [0, -1, 0], [0, 0, -1]])
    points = points @ R_image2camera.T  # Rotate to camera frame
    points = points @ rotation.transposed() + translation  # Rotate to world frame and apply translation

    # Convert points and colors to open3d PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Downsample the point cloud using voxel grid
    pcd_downsampled = pcd.voxel_down_sample(voxel_size)

    return np.array(pcd_downsampled.points), np.array(pcd_downsampled.colors)

# Function to process and downsample each frame
def process_single_frame(observation, translation, rotation, fx, fy, cx, cy, voxel_size):
    depth = observation['depth_sensor']
    color = observation['color_sensor_1st_person'][:, :, :3]
    rotation_matrix = rotation.to_matrix()
    translation_vector = np.array([translation.x, translation.y, translation.z])

    # Convert depth and color to point cloud and apply downsampling
    points, colors = depth_to_point_cloud_with_pose(depth, color, fx, fy, cx, cy, translation_vector, rotation_matrix, voxel_size)
    return points, colors

# Process point clouds in parallel
def process_point_clouds(observations, translations, rotations, fx, fy, cx, cy, voxel_size=0.05):
    all_pcd = o3d.geometry.PointCloud()

    with ProcessPoolExecutor() as executor:
        # Process each frame in parallel and get downsampled point clouds
        results = executor.map(process_single_frame, observations, translations, rotations, [fx]*len(observations), 
                               [fy]*len(observations), [cx]*len(observations), [cy]*len(observations), [voxel_size]*len(observations))

        # Incrementally add the downsampled point cloud to all_pcd
        for points, colors in results:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            all_pcd += pcd
            all_pcd = all_pcd.voxel_down_sample(voxel_size)

    return all_pcd

# Log merged point cloud and camera frames to Rerun
def log_to_rerun(merged_pcd, rotations, translations):
    rr.init("PointCloud Visualization", spawn=True)

    # Extract points and colors from the merged PointCloud
    merged_points = np.asarray(merged_pcd.points)
    merged_colors = np.asarray(merged_pcd.colors)

    # Log merged point cloud
    rr.log("merged_pointcloud", rr.Points3D(merged_points, colors=(merged_colors * 255).astype(np.uint8)))

    log_data = []
    R_image2camera = mn.Matrix3x3(np.array([[1, 0., 0], [0, -1, 0], [0, 0, -1]]))
    for count, (rotation, translation) in enumerate(zip(rotations, translations)):
        translation_vector = np.array([translation.x, translation.y, translation.z])
        transformed_rotation = quaternion.from_rotation_matrix(np.array(rotation.to_matrix() @ R_image2camera))
        rotation_quat = rr.Quaternion(xyzw=[transformed_rotation.x, transformed_rotation.y, transformed_rotation.z, transformed_rotation.w])
        rr.log(f"camera_frame_{count}", rr.Transform3D(rotation=rotation_quat, translation=translation))


# Main execution
def main():
    repo = git.Repo(".", search_parent_directories=True)
    dir_path = repo.working_tree_dir
    file_path = dir_path + "/output/data.pkl"

    # Load data
    loaded_data = load_data(file_path)
    observations, rotations, translations = loaded_data['observations'], loaded_data['rotations'], loaded_data['translations']
    camera_info = loaded_data['camera_info']
    fx, fy, cx, cy = camera_info['fx'], camera_info['fy'], camera_info['cx'], camera_info['cy']

    # Process and downsample point clouds
    merged_pcb = process_point_clouds(observations, translations, rotations, fx, fy, cx, cy)

    # Log the results to Rerun
    log_to_rerun(merged_pcb, rotations, translations)

if __name__ == "__main__":
    main()
