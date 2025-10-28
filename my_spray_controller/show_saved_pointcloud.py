#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import os
from std_msgs.msg import Header


class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('saved_pointcloud_publisher')

        # === Parameter anpassen ===
        self.declare_parameter('directory', '/home/matthes/Projects/ros2_ws/src/my_spray_controller/lidar_scans')
        self.declare_parameter('filename', '')  # Falls leer, wird neueste Datei gewählt
        self.declare_parameter('frame_id', 'world')  # RViz Frame

        directory = self.get_parameter('directory').value
        filename = self.get_parameter('filename').value
        self.frame_id = self.get_parameter('frame_id').value

        # === Punktwolke laden ===
        if filename == '':
            files = sorted([f for f in os.listdir(directory) if f.endswith('.npy')])
            if not files:
                self.get_logger().error(f"Keine .npy-Dateien in {directory} gefunden!")
                rclpy.shutdown()
                return
            filename = files[-1]
        
        file_path = os.path.join(directory, filename)
        self.get_logger().info(f"Lade Punktwolke: {file_path}")

        try:
            points = np.load(file_path)
        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden: {e}")
            rclpy.shutdown()
            return
        
        # Punkte zentrieren, damit sie bei (0,0,0) liegen
        mean_x = np.mean(points[:, 0])
        mean_y = np.mean(points[:, 1])
        mean_z = np.mean(points[:, 2])
        points[:, 0] -= mean_x
        points[:, 1] -= mean_y
        points[:, 2] -= mean_z


        if points.ndim != 2 or points.shape[1] < 3:
            self.get_logger().error(f"Ungültige Datenform: {points.shape}")
            rclpy.shutdown()
            return

        # Optional: Filterung oder Skalierung
        # points[:, 0] -= np.min(points[:, 0])   # → Setze Startpunkt auf 0
        # points -= np.mean(points, axis=0)      # → Zentriere Wolke im Ursprung
        # points *= np.array([1, 1, 1])          # → Skalierung falls nötig

        self.points = points.astype(np.float32)
        self.get_logger().info(f"{len(self.points)} Punkte geladen")

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, '/pointcloud_loaded', 10)

        # Timer → wiederholtes Publizieren
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = self.create_pointcloud2(self.points)
        self.publisher.publish(msg)

    def create_pointcloud2(self, points: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(struct.pack('fff', *p) for p in points)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
