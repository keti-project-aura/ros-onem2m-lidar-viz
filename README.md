# ros_onem2m_lidar_viz

ROS workspace for live LiDAR (Velodyne) visualization plus on-demand save/upload to Mobius (oneM2M). A TCP command triggers local PCD saves or Mobius uploads, while RViz shows the point cloud stream.

## Modules
- `control_module`: TCP server (default port `5000`) that forwards commands to ROS topic `/sensor_command`.
- `data_save_module`
  - `local_save.py`: Saves `/velodyne_points` to PCD when requested. Default path: `~/uneck-ws/save`.
  - `mobius_save.py`: Converts `/velodyne_points` to PCD binary and uploads to Mobius when requested. Config is loaded from JSON.
- `ros_onem2m_lidar_viz`: Launch file and RViz config to start Velodyne, rosbridge, and RViz together.

## Requirements
- ROS (Noetic tested) with `velodyne_pointcloud`, `rosbridge_server`, `rviz`
- Python: `numpy`, `open3d`, `requests`
- LiDAR topic: `/velodyne_points`

## Mobius config
1) Copy the template:
```bash
cp src/data_save_module/config/mobius_config.json.example src/data_save_module/config/mobius_config.json
```
2) Fill real values:
- `url`: Mobius endpoint (e.g., `http://<host>:7579/Mobius/AIoT_Test`)
- `origin`, `request_id`, `content_type`, `accept`: set to your environment
3) Use a custom path via env:
```bash
export MOBIUS_CONFIG_PATH=/path/to/mobius_config.json
```

## Build & run
```bash
cd /path/to/your/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ros_onem2m_lidar_viz ros_onem2m_lidar_viz.launch
```

## Flow
1) A TCP client sends a command containing `sensor` to `0.0.0.0:5000`.
2) `control_module` publishes `/sensor_command`.
3) `local_save.py` / `mobius_save.py` set save/upload flags and handle the next `/velodyne_points` message.
4) RViz shows `/velodyne_points` live (config: `src/ros_onem2m_lidar_viz/rviz/ros_onem2m_lidar_viz.rviz`).

## Notes
- Real Mobius config file (`src/data_save_module/config/mobius_config.json`) is gitignored; keep your secrets out of the repo.
- If you use different topics/sensors, update the topic names in code/launch as needed.
