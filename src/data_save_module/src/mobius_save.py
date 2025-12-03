import base64
import json
import os
import numpy as np
import requests
import rospy
from datetime import datetime
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class DataUploader:
    def __init__(self):
        rospy.init_node("mobius_save", anonymous=True)

        self.latest_velodyne_pcd_bytes = None
        self.config = self.load_config()

        self.mobius_url = self.config["url"]

        self.headers = {
            "X-M2M-Origin": self.config["origin"],
            "X-M2M-RI": self.config["request_id"],
            "Content-Type": self.config["content_type"],
            "Accept": self.config["accept"],
        }

        rospy.Subscriber("/velodyne_points", PointCloud2, self.handle_velodyne_pcd)
        rospy.Subscriber("/sensor_command", String, self.sensor_command_callback)

    def sensor_command_callback(self, msg):
        command = msg.data
        if "save" in command:
            rospy.loginfo("Success Connection...")
            self.upload_all()

    def handle_velodyne_pcd(self, msg):
        self.latest_velodyne_pcd_bytes = self.convert_pcd_to_bytes(msg)

    def load_config(self):
        """Load Mobius configuration from JSON file specified by MOBIUS_CONFIG_PATH or default path."""
        default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "mobius_config.json")
        config_path = os.environ.get("MOBIUS_CONFIG_PATH", default_path)

        if not os.path.exists(config_path):
            raise RuntimeError(
                f"Mobius config file not found: {config_path}. "
                "Create it (copy from mobius_config.json.example) and set MOBIUS_CONFIG_PATH if needed."
            )

        with open(config_path, "r") as f:
            data = json.load(f)

        required_keys = ["url", "origin", "request_id", "content_type", "accept"]
        missing = [k for k in required_keys if k not in data or not data[k]]
        if missing:
            raise RuntimeError(f"Mobius config missing keys: {missing}")

        return data

    def convert_pcd_to_bytes(self, msg):
        """PointCloud2 메시지를 PCD 바이너리 형식으로 변환하여 bytes로 반환"""
        try:
            # PointCloud2에서 NumPy 배열로 변환
            points = np.array(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)

            if points.size == 0:
                rospy.logwarn("Received empty point cloud message")
                return None

            # PCD 헤더 생성
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                f"WIDTH {len(points)}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(points)}\n"
                "DATA binary\n"
            )

            header_bytes = header.encode("ascii")  # 헤더를 ASCII로 변환
            data_bytes = points.tobytes()  # NumPy 배열을 바이너리로 변환

            # PCD 바이너리 형식으로 반환
            return header_bytes + data_bytes  # 헤더 + 데이터 합치기
        except Exception as e:
            rospy.logerr(f"Failed to convert PCD: {e}")
            return None

    def create_container(self):
        """ 서버에 타임스탬프 기반 컨테이너 생성 """
        # 타임스탬프 기반으로 컨테이너 이름 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        container_name = f"{timestamp}"

        url = f"{self.mobius_url}"
        payload = {"m2m:cnt": {"rn": container_name, "mbs": 4000000000}}
        response = requests.post(url, json=payload, headers=self.headers)

        if response.status_code in [200, 201]:
            rospy.loginfo(f"Container {container_name} created successfully")
            return f"{url}/{container_name}"
        else:
            rospy.logerr(f"Failed to create container: {response.text}")
            return None


    def upload_all(self):
        if self.latest_velodyne_pcd_bytes is None:
            rospy.logwarn("No Velodyne data available for upload")
            return

        container_url = self.create_container()
        if not container_url:
            return

        if self.latest_velodyne_pcd_bytes:
            data_velodyne_pcd = {
                "m2m:cin": {
                    "con": base64.b64encode(self.latest_velodyne_pcd_bytes).decode("ascii"),
                    "cnf": "application/octet-stream",
                    "rn": "velodyne_pcd",
                }
            }
            response = requests.post(container_url, json=data_velodyne_pcd, headers=self.headers)
            if response.status_code == 201:
                rospy.loginfo("Velodyne PCD uploaded successfully")
            else:
                rospy.logerr(f"Failed to upload Velodyne PCD: {response.text}")


if __name__ == "__main__":
    uploader = DataUploader()
    rospy.spin()
