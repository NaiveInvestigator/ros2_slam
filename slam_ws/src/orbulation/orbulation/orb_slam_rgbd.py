#!/usr/bin/env python3
"""
Python port of orb_slam_rgbd.cpp

Uses orb_slam3.python_wrapper.orb_slam_pybind (ORB_SLAM3 wrapper class)
which mirrors the C++ ORB_SLAM3::System interface directly:

    ORB_SLAM3(vocab, config, sensor, vis)
    slam.TrackRGBD(rgb, depth, timestamp)   -> Tcw 4x4 numpy (Sophus::SE3f)
    slam.isLost()                           -> bool
    slam.Shutdown()

Publishes:
    /slam/odometry   (nav_msgs/Odometry)
    /slam/pointcloud (sensor_msgs/PointCloud2)  -- stub; wrapper has no map point API

Static TF: map → odom
"""

import threading
import queue
import argparse
import numpy as np
import cv2
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import cv_bridge

from orb_slam3 import ORB_SLAM3   # the high-level wrapper

# ---------------------------------------------------------------------------
# Static TF broadcaster  (mirrors publish_static_transform in C++)
# ---------------------------------------------------------------------------
 
def publish_static_transform(node: Node) -> None:
    """Broadcast identity transform: map → odom."""
    broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    tf_msg = TransformStamped()
    tf_msg.header.stamp    = node.get_clock().now().to_msg()
    tf_msg.header.frame_id = "map"
    tf_msg.child_frame_id  = "odom"
    tf_msg.transform.rotation.w = 1.0   # identity quaternion
    broadcaster.sendTransform(tf_msg)
 

def _rotation_matrix_to_quaternion(R: np.ndarray):
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class ImageGrabber:
    """
    Mirrors ImageGrabber from image_grabber_rgbd.cpp.

    Uses orb_slam3.python_wrapper (ORB_SLAM3 wrapper class) which exposes
    TrackRGBD() returning Tcw as a 4x4 numpy array — identical to the C++
    mpSLAM->TrackRGBD() return value.

    Coordinate convention (identical to the C++ version):
        X_ROS =  Z_OCV
        Y_ROS = -X_OCV
        Z_ROS = -Y_OCV
    """

    _POS_VAR = 0.01
    _ORI_VAR = 0.02

    def __init__(
        self,
        slam,                        # orb_slam3.python_wrapper.ORB_SLAM3 instance
        odom_pub,                    # rclpy Publisher[Odometry]
        cloud_pub,                   # rclpy Publisher[PointCloud2]
        ros_node,
        camera_frame_name: str = "map",
        use_clahe: bool = False,
    ):
        self._slam      = slam
        self._odom_pub  = odom_pub
        self._cloud_pub = cloud_pub
        self._node      = ros_node
        self._tf_frame  = camera_frame_name
        self._use_clahe = use_clahe
        self._bridge    = cv_bridge.CvBridge()
        self._clahe     = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Pre-build odometry message (mirrors C++ constructor)
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = self._tf_frame
        self._odom_msg.child_frame_id  = "odom"

        cov = [0.0] * 36
        cov[0]  = self._POS_VAR   # x
        cov[7]  = self._POS_VAR   # y
        cov[14] = self._POS_VAR   # z
        cov[21] = self._ORI_VAR   # roll
        cov[28] = self._ORI_VAR   # pitch
        cov[35] = self._ORI_VAR   # yaw
        self._odom_msg.pose.covariance = cov

    # ------------------------------------------------------------------
    # Image conversion  (mirrors getImage / getDepthImage)
    # ------------------------------------------------------------------

    def get_image(self, img_msg) -> np.ndarray:
        """Convert RGB ROS Image → BGR numpy array (optionally CLAHE)."""
        try:
            image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            if self._use_clahe:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                image = self._clahe.apply(image)
            return image
        except cv_bridge.CvBridgeError as e:
            self._node.get_logger().error(f"cv_bridge exception (RGB): {e}")
            return None

    def get_depth_image(self, depth_msg) -> np.ndarray:
        """Convert depth ROS Image → 16-bit numpy array."""
        try:
            return self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
        except cv_bridge.CvBridgeError as e:
            self._node.get_logger().error(f"cv_bridge exception (Depth): {e}")
            return None

    # ------------------------------------------------------------------
    # Core processing  (mirrors processImages)
    # ------------------------------------------------------------------

    def process_images(self, rgb_msg, depth_msg):
        """
        Convert messages → numpy, call TrackRGBD, publish odometry.

        TrackRGBD() mirrors the C++ mpSLAM->TrackRGBD() — it returns Tcw
        as a 4x4 numpy array (Sophus::SE3f exposed via pybind).
        isLost() replaces the C++ tracking-state check.
        """
        rgb_image   = self.get_image(rgb_msg)
        depth_image = self.get_depth_image(depth_msg)

        if rgb_image is None or depth_image is None:
            return

        timestamp = (
            rgb_msg.header.stamp.sec
            + 1e-9 * rgb_msg.header.stamp.nanosec
        )

        # ---- Core ORB-SLAM3 call (mirrors C++ TrackRGBD) ----
        Tcw = self._slam.TrackRGBD(rgb_image, depth_image, timestamp)
        # ------------------------------------------------------

        if self._slam.isLost():
            return

        if Tcw is None:
            return

        Tcw = self._normalise_pose(Tcw)
        if Tcw is None:
            return

        self._publish_pose_to_odom(Tcw)
        # Note: this wrapper does not expose GetTrackedMapPoints().
        # Point cloud publishing requires the orbslam3_enhanced binding.

    # ------------------------------------------------------------------
    # Pose normaliser  (handles any shape Sophus might expose)
    # ------------------------------------------------------------------

    def _normalise_pose(self, pose) -> np.ndarray:
        """
        Normalise whatever TrackRGBD() returns into a (4,4) float64 array.
        Handles: (4,4), (16,) flat, (3,4) [R|t], Sophus object with .matrix().
        Returns None if the pose is degenerate.
        """
        try:
            arr = np.array(pose, dtype=np.float64)
        except Exception:
            if hasattr(pose, 'matrix'):
                arr = np.array(pose.matrix(), dtype=np.float64)
            else:
                self._node.get_logger().warning(
                    f"Unknown pose type from TrackRGBD: {type(pose)}")
                return None

        if arr.ndim == 0 or arr.size == 0:
            return None
        if arr.shape == (4, 4):
            return arr
        if arr.shape == (16,):
            return arr.reshape(4, 4)
        if arr.shape == (3, 4):
            return np.vstack([arr, [0.0, 0.0, 0.0, 1.0]])

        self._node.get_logger().warning(
            f"Unexpected pose shape {arr.shape} from TrackRGBD, skipping.")
        return None

    # ------------------------------------------------------------------
    # Pose → Odometry  (mirrors publishSE3fToOdom)
    # ------------------------------------------------------------------

    def _publish_pose_to_odom(self, Tcw: np.ndarray):
        """
        Tcw is the camera-to-world 4x4 transform returned by TrackRGBD.
        Invert to Twc, then apply OpenCV→ROS remapping (same as C++):
            X_ROS =  Z_OCV
            Y_ROS = -X_OCV
            Z_ROS = -Y_OCV
        """
        Twc = np.linalg.inv(Tcw)

        t = Twc[:3, 3]
        R = Twc[:3, :3]
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(R)

        msg = self._odom_msg
        msg.pose.pose.position.x =  t[2]    # Z_OCV → X_ROS
        msg.pose.pose.position.y = -t[0]    # -X_OCV → Y_ROS
        msg.pose.pose.position.z = -t[1]    # -Y_OCV → Z_ROS

        msg.pose.pose.orientation.x =  qz   # q.z
        msg.pose.pose.orientation.y = -qx   # -q.x
        msg.pose.pose.orientation.z = -qy   # -q.y
        msg.pose.pose.orientation.w =  qw

        msg.header.stamp = self._node.get_clock().now().to_msg()
        self._odom_pub.publish(msg)

    # ------------------------------------------------------------------
    # Optional pose file logging  (mirrors savePoseToFile)
    # ------------------------------------------------------------------

    def save_pose_to_file(self, Tcw: np.ndarray, stamp_sec: int,
                          stamp_nanosec: int, path: str = "pose.txt"):
        try:
            with open(path, "a") as f:
                row = [f"{stamp_sec}.{stamp_nanosec}"]
                row += [str(Tcw[r, c]) for r in range(4) for c in range(4)]
                f.write(" ".join(row) + "\n")
        except OSError as e:
            self._node.get_logger().error(f"Failed to open {path}: {e}")


 
# ---------------------------------------------------------------------------
# ROS2 node  (mirrors main() in orb_slam_rgbd.cpp)
# ---------------------------------------------------------------------------
 
class OrbSlamRGBDNode(Node):
    """
    Thin ROS2 node wiring ORB-SLAM3 to ROS2 topics.
 
    Synchronisation between the RGB and depth queues is done in a daemon
    thread, exactly as the C++ sync_callback thread.
    """
 
    _SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
 
    def __init__(self, vocab_path: str, config_path: str, show_viewer: bool = False):
        super().__init__("orbslam3_rgbd_node")
 
        # ---- ROS2 parameters (overridable by launch file) ------------------
        self.declare_parameter("vocab_path",  vocab_path)
        self.declare_parameter("config_path", config_path)
        self.declare_parameter("show_viewer", show_viewer)
 
        vocab_path  = self.get_parameter("vocab_path").get_parameter_value().string_value
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        show_viewer = self.get_parameter("show_viewer").get_parameter_value().bool_value
 
        self.get_logger().info(f"vocab_path  : {vocab_path}")
        self.get_logger().info(f"config_path : {config_path}")
        self.get_logger().info(f"show_viewer : {show_viewer}")
 
        # ---- Publishers ----------------------------------------------------
        self._odom_pub  = self.create_publisher(Odometry,    "/slam/odometry",   10)
        self._cloud_pub = self.create_publisher(PointCloud2, "/slam/pointcloud",  10)
 
        # ---- Static TF: map → odom ----------------------------------------
        publish_static_transform(self)
 
        # ---- ORB-SLAM3 system (orb_slam3.python_wrapper) -------------------
        # ORB_SLAM3(vocabulary, config, sensor, vis)
        # sensor string "RGBD" is mapped to _System.eSensor.RGBD internally.
        self._slam = ORB_SLAM3(
            vocabulary=vocab_path,
            config=config_path,
            sensor="RGBD",
            vis=show_viewer,
        )
 
        # ---- ImageGrabber --------------------------------------------------
        self._igb = ImageGrabber(
            slam=self._slam,
            odom_pub=self._odom_pub,
            cloud_pub=self._cloud_pub,
            ros_node=self,
            camera_frame_name="map",
            use_clahe=False,
        )
 
        # ---- Frame queues: maxsize=1 so we always get the LATEST frame -----
        # Without a size limit, TrackRGBD (~50-100ms) can't keep up with
        # 30fps incoming frames (33ms/frame), causing unbounded lag.
        # maxsize=1 + drain-before-put means the sync thread always wakes up
        # to find the most recent frame pair, not a stale one.
        self._rgb_queue   = queue.Queue(maxsize=1)
        self._depth_queue = queue.Queue(maxsize=1)
        self._cond        = threading.Condition()
 
        # ---- Subscriptions (mirror rgb_sub / depth_sub) --------------------
        self.create_subscription(
            Image,
            "/a200_0000/sensors/camera_0/color/image",
            self._rgb_callback,
            self._SENSOR_QOS,
        )
        self.create_subscription(
            Image,
            "/a200_0000/sensors/camera_0/depth/image",
            self._depth_callback,
            self._SENSOR_QOS,
        )
 
        # ---- Sync thread (mirrors std::thread sync_callback in C++) --------
        self._sync_thread = threading.Thread(
            target=self._sync_loop, daemon=True, name="slam_sync"
        )
        self._sync_thread.start()
 
        self.get_logger().info("OrbSlamRGBDNode initialised and running.")
 
    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
 
    def _rgb_callback(self, msg: Image) -> None:
        with self._cond:
            # Drain the queue first so we only ever hold the latest frame.
            # If the sync thread hasn't consumed the previous one yet, discard it.
            try:
                self._rgb_queue.get_nowait()
            except queue.Empty:
                pass
            self._rgb_queue.put_nowait(msg)
            self._cond.notify()
 
    def _depth_callback(self, msg: Image) -> None:
        with self._cond:
            try:
                self._depth_queue.get_nowait()
            except queue.Empty:
                pass
            self._depth_queue.put_nowait(msg)
            self._cond.notify()
 
    # ------------------------------------------------------------------
    # Sync loop  (mirrors sync_callback() in C++)
    # ------------------------------------------------------------------
 
    def _sync_loop(self) -> None:
        while rclpy.ok():
            with self._cond:
                self._cond.wait_for(
                    lambda: not self._rgb_queue.empty()
                            and not self._depth_queue.empty()
                )
                rgb_msg   = self._rgb_queue.get()
                depth_msg = self._depth_queue.get()
 
            self._igb.process_images(rgb_msg, depth_msg)
 
    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------
 
    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down ORB-SLAM3 …")
        self._slam.Shutdown()      # mirrors slam.shutdown() / SLAM.Shutdown()
        super().destroy_node()
 
 
# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
 
def main():
    parser = argparse.ArgumentParser(description="ORB-SLAM3 RGBD ROS2 node")
    parser.add_argument("--vocab",      default="", help="Path to ORB vocabulary")
    parser.add_argument("--config",     default="", help="Path to SLAM config YAML")
    parser.add_argument("--show-viewer", action="store_true",
                        help="Enable Pangolin viewer (requires GTK display)")
    args, ros_args = parser.parse_known_args()
 
    rclpy.init(args=ros_args)
 
    node = OrbSlamRGBDNode(
        vocab_path=args.vocab,
        config_path=args.config,
        show_viewer=args.show_viewer,
    )
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
