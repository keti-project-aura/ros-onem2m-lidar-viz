#!/usr/bin/env python3

import json
import os
import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from datetime import datetime

class SensorRecorder:
    def __init__(self):
        rospy.init_node("local_save", anonymous=True)
        self.config = self.load_config()
        
        # 저장 디렉토리 설정
        self.save_dir = os.path.expanduser(self.config["save_dir"])
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # ROS 토픽 구독 (Velodyne)
        rospy.Subscriber(self.config["velodyne_topic"], PointCloud2, self.save_velodyne_pcd)

        # 데이터 저장 요청 토픽 구독 (control_module에서 전달)
        rospy.Subscriber(self.config["command_topic"], String, self.sensor_command_callback)
        
        # 저장 요청 상태
        self.velodyne_pcd_requested = False

    def load_config(self):
        """Load local save configuration from JSON."""
        default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "local_save_config.json")
        config_path = os.environ.get("LOCAL_SAVE_CONFIG_PATH", default_path)

        if not os.path.exists(config_path):
            raise RuntimeError(
                f"Local save config file not found: {config_path}. "
                "Copy local_save_config.json.example and set LOCAL_SAVE_CONFIG_PATH if needed."
            )

        with open(config_path, "r") as f:
            data = json.load(f)

        required = ["save_dir", "velodyne_topic", "command_topic"]
        missing = [k for k in required if k not in data or data[k] in (None, "")]
        if missing:
            raise RuntimeError(f"Local save config missing keys: {missing}")

        return data

    def sensor_command_callback(self, msg):
        command = msg.data
        if "save" in command:
            rospy.loginfo("Saving data...")
            self.velodyne_pcd_requested = True
    
    def get_timestamped_filename(self, name):
        """현재 날짜와 시간을 기반으로 파일명 생성"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # 형식: YYYYMMDD_HHMMSS
        return os.path.join(self.save_dir, f"{timestamp}_{name}")
    
    def save_velodyne_pcd(self, msg):
        if self.velodyne_pcd_requested:
            try:
                rospy.loginfo("Saving Point Cloud (Velodyne)...")
                cloud_points = []
                for point in point_cloud2.read_points(msg, skip_nans=True):
                    cloud_points.append([point[0], point[1], point[2]])
                
                # Open3D로 PCD 파일 저장
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
                pcd_filename = self.get_timestamped_filename("velodyne_point_cloud.pcd")
                o3d.io.write_point_cloud(pcd_filename, pcd)
                rospy.loginfo(f"Point Cloud (Velodyne) saved: {pcd_filename}")
                self.velodyne_pcd_requested = False
            except Exception as e:
                rospy.logerr(f"Failed to save Point Cloud (Velodyne): {e}")

if __name__ == '__main__':
    try:
        recorder = SensorRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
