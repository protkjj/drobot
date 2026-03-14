import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import struct

from vision_msgs.msg import Detection3DArray
from drobot_msgs.msg import TrackedObstacle, TrackedObstacleArray
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from drobot_tracker.track import Track
from drobot_tracker.data_association import associate_detections_to_tracks


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker_node')

        # Parameters
        self.declare_parameter('gate_threshold', 2.0)
        self.declare_parameter('max_misses', 5)
        self.declare_parameter('min_hits', 3)
        self.declare_parameter('process_noise_std', 0.5)
        self.declare_parameter('measurement_noise_std', 0.3)

        self.gate_threshold = self.get_parameter('gate_threshold').value
        self.max_misses = self.get_parameter('max_misses').value
        self.min_hits = self.get_parameter('min_hits').value
        self.process_noise_std = self.get_parameter('process_noise_std').value
        self.measurement_noise_std = self.get_parameter('measurement_noise_std').value

        # State
        self.tracks = []
        self.prev_stamp = None

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.create_subscription(
            Detection3DArray,
            '/detections/detections3d',
            self.detection_callback,
            qos
        )

        # Publishers
        self.pub_tracked = self.create_publisher(
            TrackedObstacleArray, '/tracked_obstacles', 10
        )
        self.pub_pointcloud = self.create_publisher(
            PointCloud2, '/tracked_obstacles/pointcloud', 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, '/tracked_obstacles/markers', 10
        )

        self.get_logger().info(
            f'Object Tracker started (gate={self.gate_threshold}m, '
            f'min_hits={self.min_hits}, max_misses={self.max_misses})'
        )

    def detection_callback(self, msg):
        # 1. Compute dt
        current_stamp = msg.header.stamp
        if self.prev_stamp is None:
            dt = 0.1  # default for first frame
        else:
            dt = (current_stamp.sec - self.prev_stamp.sec) + \
                 (current_stamp.nanosec - self.prev_stamp.nanosec) * 1e-9
            if dt <= 0.0:
                dt = 0.1
        self.prev_stamp = current_stamp

        # 2. Predict all existing tracks
        for track in self.tracks:
            track.predict(dt)

        # 3. Extract 3D positions and metadata from detections
        det_positions = []
        det_labels = []
        det_confs = []
        for det in msg.detections:
            pos = det.results[0].pose.pose.position if det.results else det.bbox.center.position
            det_positions.append([pos.x, pos.y, pos.z])
            if det.results:
                det_labels.append(det.results[0].hypothesis.class_id)
                det_confs.append(det.results[0].hypothesis.score)
            else:
                det_labels.append('')
                det_confs.append(0.0)

        # 4. Associate detections to tracks (Hungarian)
        matches, unmatched_dets, unmatched_trks = associate_detections_to_tracks(
            det_positions, self.tracks, self.gate_threshold
        )

        # 5. Update matched tracks
        for d_idx, t_idx in matches:
            self.tracks[t_idx].update(
                det_positions[d_idx],
                label=det_labels[d_idx],
                confidence=det_confs[d_idx]
            )

        # 6. Mark missed tracks
        for t_idx in unmatched_trks:
            self.tracks[t_idx].mark_missed()

        # 7. Create new tracks for unmatched detections
        for d_idx in unmatched_dets:
            new_track = Track(
                position=det_positions[d_idx],
                label=det_labels[d_idx],
                confidence=det_confs[d_idx],
                process_noise_std=self.process_noise_std,
                measurement_noise_std=self.measurement_noise_std,
            )
            self.tracks.append(new_track)

        # 8. Remove dead tracks
        self.tracks = [t for t in self.tracks if not t.is_dead(self.max_misses)]

        # 9. Publish confirmed tracks only
        confirmed = [t for t in self.tracks if t.is_confirmed(self.min_hits)]
        self._publish_tracked_obstacles(confirmed, msg.header)
        self._publish_pointcloud(confirmed, msg.header)
        self._publish_markers(confirmed, msg.header)

    def _publish_tracked_obstacles(self, tracks, header):
        arr = TrackedObstacleArray()
        arr.header = header
        for t in tracks:
            obs = TrackedObstacle()
            obs.track_id = t.track_id
            obs.label = t.label
            obs.confidence = t.confidence
            pos = t.position
            obs.position.x = float(pos[0])
            obs.position.y = float(pos[1])
            obs.position.z = float(pos[2])
            vel = t.velocity
            obs.velocity.x = float(vel[0])
            obs.velocity.y = float(vel[1])
            obs.velocity.z = float(vel[2])
            obs.age = float(t.age)
            obs.is_active = t.consecutive_misses == 0
            arr.obstacles.append(obs)
        self.pub_tracked.publish(arr)

    def _publish_pointcloud(self, tracks, header):
        """Publish tracked obstacle positions as PointCloud2 for Nav2 costmap."""
        pc_header = Header()
        pc_header.stamp = header.stamp
        pc_header.frame_id = header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12
        data = bytearray()
        for t in tracks:
            pos = t.position
            data += struct.pack('fff', float(pos[0]), float(pos[1]), float(pos[2]))

        pc = PointCloud2()
        pc.header = pc_header
        pc.height = 1
        pc.width = len(tracks)
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = point_step
        pc.row_step = point_step * len(tracks)
        pc.data = bytes(data)
        pc.is_dense = True

        self.pub_pointcloud.publish(pc)

    def _publish_markers(self, tracks, header):
        """Publish RViz markers: sphere for position, arrow for velocity."""
        marker_array = MarkerArray()

        # First, a DELETE_ALL marker to clean up old markers
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, t in enumerate(tracks):
            pos = t.position
            vel = t.velocity

            # Position sphere
            sphere = Marker()
            sphere.header = header
            sphere.ns = 'tracked_obstacles'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale = Vector3(x=0.3, y=0.3, z=0.3)
            sphere.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            sphere.lifetime.sec = 0
            sphere.lifetime.nanosec = 500000000  # 0.5s
            marker_array.markers.append(sphere)

            # Velocity arrow
            speed = np.linalg.norm(vel)
            if speed > 0.05:  # only show if moving
                arrow = Marker()
                arrow.header = header
                arrow.ns = 'tracked_velocities'
                arrow.id = i * 2 + 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
                end = Point(
                    x=float(pos[0] + vel[0]),
                    y=float(pos[1] + vel[1]),
                    z=float(pos[2] + vel[2])
                )
                arrow.points = [start, end]
                arrow.scale = Vector3(x=0.05, y=0.1, z=0.1)
                arrow.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
                arrow.lifetime.sec = 0
                arrow.lifetime.nanosec = 500000000
                marker_array.markers.append(arrow)

            # Text label
            text = Marker()
            text.header = header
            text.ns = 'tracked_labels'
            text.id = i * 2 + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(pos[0])
            text.pose.position.y = float(pos[1])
            text.pose.position.z = float(pos[2]) + 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'ID:{t.track_id} {t.label} v={speed:.1f}'
            text.lifetime.sec = 0
            text.lifetime.nanosec = 500000000
            marker_array.markers.append(text)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
