# ROS2 Point Cloud Clustering and Segmentation for Autonomous Behaviour

ROS2 + PCL (Point Cloud Library) point cloud processing pipeline, built while working through a ROS2 point cloud processing course and extended with a TurtleBot3/Gazebo simulation environment and KITTI raw LiDAR data.

The core package, `point_cloud_processing`, progresses from basic PCL exercises to a full perception pipeline: filtering → plane/road segmentation (RANSAC) → Euclidean clustering (KD-tree) → bounding boxes, applied both to static `.pcd` scenes and to a live KITTI point cloud stream over ROS2 topics.

## Repository layout

```
point_cloud_processing/   ROS2 C++ package — the actual point cloud pipeline (see below)
turtlebot3_gazebo/        Vendored TurtleBot3 Gazebo simulation package (ROBOTIS, third-party — see Attribution)
data/raw_data_downloader.sh   Helper script to download KITTI raw sequences
```

## `point_cloud_processing` package

Each numbered source file under `point_cloud_processing/src/` is a self-contained exercise, built as its own executable:

| Executable                | Source file                              | What it does                                                                                 |
|----------------------------|-------------------------------------------|------------------------------------------------------------------------------------------------|
| `pcd_import`               | `0_table_scene.cpp`                       | Loads a `.pcd` scene as a first PCL sanity check.                                             |
| `plane_cloud`               | `1_planner_cloud.cpp`                     | RANSAC-based planar segmentation on a sample cloud.                                           |
| `circular_cloud`            | `2_circular_cloud.cpp`                    | Generates a synthetic circular point cloud.                                                   |
| `filtering_segmentation`    | `3_filtering_segmentation.cpp`            | Voxel-grid downsampling + pass-through filtering + planar RANSAC segmentation.                |
| `clustering_segmentation`   | `4_clustering_filtering_segmentation.cpp` | Filtering + normal estimation + iterative cylinder segmentation (RANSAC), saving each cluster.|
| `kitti_voxel`                | `5_kitti_data_voxel.cpp`                  | ROS2 node: subscribes to `/kitti/point_cloud`, voxel-downsamples it, republishes it.          |
| `kd_tree`                   | `6_kd_tree.cpp`                           | Standalone KD-tree (FLANN) nearest-neighbor search demo.                                      |
| `traffic_segmentation`      | `7_traffic_segmentation.cpp`              | ROS2 node: removes the road plane (RANSAC + normals) from KITTI data, then Euclidean-clusters the remaining points and computes bounding boxes — aimed at picking out traffic participants (cars, pedestrians, etc.). |

Supporting resources:
- `point_cloud_processing/point_clouds/` — sample `.pcd` files used by the offline exercises.
- `point_cloud_processing/launch/` — launch files for RViz visualization (`bring_rviz.launch.py`) and the KITTI pipeline (`process_kitti.launch.py`).
- `point_cloud_processing/rviz/kitti_viz.rviz` — RViz config for visualizing the KITTI point cloud stream.

## Requirements

- Ubuntu with ROS2 (developed against Humble)
- PCL >= 1.3 and `pcl_conversions`
- `colcon` and `rosdep`
- Gazebo, for the `turtlebot3_gazebo` simulation

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running the exercises

Offline exercises (0–4) read a `.pcd` file from a path hardcoded inside the source — see [Known limitations](#known-limitations) below before running them:

```bash
ros2 run point_cloud_processing filtering_segmentation
```

KITTI-based nodes (`kitti_voxel`, `traffic_segmentation`) expect point clouds published on `/kitti/point_cloud`:

```bash
ros2 launch point_cloud_processing bring_rviz.launch.py   # RViz visualization
ros2 run point_cloud_processing traffic_segmentation
```

TurtleBot3 Gazebo simulation (e.g.):

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Known limitations

Being upfront about the current rough edges rather than hiding them:

- Source files `0`–`4` read/write `.pcd` files from a hardcoded absolute path (`/home/david/ros2_ws/...`). Edit that path to match your own workspace before running them.
- `process_kitti.launch.py` launches a node from a `ros2_kitti_publishers` package that fed KITTI data onto `/kitti/point_cloud`. That package was tracked in this repo as a broken git submodule reference (no `.gitmodules`, so it never actually cloned) and has been removed rather than left as dead weight — to run the KITTI-fed nodes end-to-end you'll need to supply your own publisher for `/kitti/point_cloud` (see `data/raw_data_downloader.sh` for getting the raw KITTI sequences).

## Attribution

`turtlebot3_gazebo/` is vendored from ROBOTIS' [`turtlebot3_simulations`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), used here as the Gazebo simulation environment. It keeps its original Apache-2.0 license (see the license headers in its source files and its own `package.xml`) — it is not original work from this project.

## License

MIT, for the code in `point_cloud_processing/` and `data/` — see [LICENSE](LICENSE). `turtlebot3_gazebo/` remains Apache-2.0 as noted above.
