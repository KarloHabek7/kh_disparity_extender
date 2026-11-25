"""
ROS 2 Čvor za Disparity Extender algoritam (kh_disparity_extender).

Pretplaćuje se na:
    /scan (sensor_msgs/LaserScan)

Objavljuje na:
    /drive (ackermann_msgs/AckermannDriveStamped)
    /scan_proc (sensor_msgs/LaserScan) - preprocesirani scan za RViz
    /scan_extended (sensor_msgs/LaserScan) - scan s proširenim preprekama za RViz
    /target_marker (visualization_msgs/Marker) - zelena strelica cilja za RViz
"""
import math

from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from kh_disparity_extender import algorithm
from kh_disparity_extender import viz


class DisparityExtenderNode(Node):

    def __init__(self):
        super().__init__('disparity_extender_node')

        # ==============================================================
        # 1. Deklaracija parametara (učitavaju se iz config/params.yaml)
        # ==============================================================
        self.declare_parameter('car_width', 0.31)
        self.declare_parameter('safety_margin', 0.15)
        self.declare_parameter('max_steering_angle', 0.34)
        self.declare_parameter('disparity_threshold', 0.35)
        self.declare_parameter('fov_degrees', 150.0)
        self.declare_parameter('max_lidar_range', 12.0)
        self.declare_parameter('filter_window_size', 3)
        self.declare_parameter('far_ratio', 0.90)
        self.declare_parameter('max_speed', 4.5)
        self.declare_parameter('min_speed', 1.5)
        self.declare_parameter('speed_steer_threshold', 0.12)
        self.declare_parameter('slow_down_distance', 1.5)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('drive_topic', 'drive')

        # Dohvat vrijednosti parametara
        self.car_width = self.get_parameter('car_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.fov_degrees = self.get_parameter('fov_degrees').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.far_ratio = self.get_parameter('far_ratio').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.speed_steer_threshold = self.get_parameter('speed_steer_threshold').value
        self.slow_down_distance = self.get_parameter('slow_down_distance').value
        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        self.get_logger().info('=' * 50)
        self.get_logger().info('kh_disparity_extender čvor uspješno pokrenut!')
        self.get_logger().info(f'Širina vozila: {self.car_width}m, Margina: {self.safety_margin}m')
        self.get_logger().info(f'Prag disparityja: {self.disparity_threshold}m, FOV: {self.fov_degrees}°')
        self.get_logger().info(f'Raspon brzine: {self.min_speed} - {self.max_speed} m/s')
        self.get_logger().info('=' * 50)

        # ==============================================================
        # 2. ROS 2 Pretplatnici i izdavači (Subscribers & Publishers)
        # ==============================================================
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.lidar_callback, 10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )

        # Izdavači za RViz vizualizaciju
        self.proc_pub = self.create_publisher(LaserScan, 'scan_proc', 10)
        self.extended_pub = self.create_publisher(LaserScan, 'scan_extended', 10)
        self.target_pub = self.create_publisher(Marker, 'target_marker', 10)

    def lidar_callback(self, scan_msg_in: LaserScan):
        """
        Glavna petlja algoritma: izvršava se pri svakom pristiglom LiDAR skeniranju.
        """
        # 1. Izračun kuteva za sve zrake u LaserScan poruci
        raw_ranges = np.array(scan_msg_in.ranges)
        raw_angles = scan_msg_in.angle_min + np.arange(len(raw_ranges)) * scan_msg_in.angle_increment

        # 2. Preprocesiranje (ograničenje FOV, nan/inf, medijan filtar)
        proc_ranges, proc_angles = algorithm.preprocess_scan(
            raw_ranges,
            raw_angles,
            fov_deg=self.fov_degrees,
            max_range=self.max_lidar_range,
            filter_window_size=self.filter_window_size
        )

        # 3. Detekcija disparityja i proširenje sjene prepreke za širinu vozila
        extended_ranges = algorithm.detect_and_extend_disparities(
            proc_ranges,
            angle_increment=scan_msg_in.angle_increment,
            car_width=self.car_width,
            safety_margin=self.safety_margin,
            disparity_threshold=self.disparity_threshold
        )

        # 4. Odabir najboljeg smjera (središte najšireg dubokog prolaza)
        best_idx, target_angle, target_distance = algorithm.find_furthest_direction(
            extended_ranges,
            proc_angles,
            far_ratio=self.far_ratio
        )

        # 5. Ograničenje kuta upravljanja na mehaničke granice vozila
        steer_angle = float(np.clip(
            target_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        ))

        # 6. Udaljenost točno ispred vozila (kut najbliži 0 rad)
        forward_idx = int(np.argmin(np.abs(proc_angles)))
        forward_distance = float(proc_ranges[forward_idx])

        # 7. Određivanje brzine s obzirom na kut i slobodan prostor ispred
        speed = algorithm.calculate_speed(
            steering_angle=steer_angle,
            forward_distance=forward_distance,
            min_speed=self.min_speed,
            max_speed=self.max_speed,
            max_steer=self.max_steering_angle,
            steer_threshold=self.speed_steer_threshold,
            slow_down_distance=self.slow_down_distance
        )

        # 8. Objavljivanje komande vožnje
        self.publish_drive(steer_angle, speed)

        # 9. Objavljivanje vizualizacijskih podataka za RViz
        self.publish_visualizations(
            scan_msg_in, proc_ranges, extended_ranges, proc_angles,
            target_distance, target_angle
        )

        # 10. Periodički ispis u terminal (svake sekunde)
        self.get_logger().info(
            f'Kut: {math.degrees(steer_angle):+5.1f}° | '
            f'Brzina: {speed:4.2f} m/s | '
            f'Naprijed: {forward_distance:4.2f} m | '
            f'Cilj dist: {target_distance:4.2f} m',
            throttle_duration_sec=1.0
        )

    def publish_drive(self, steering_angle: float, speed: float):
        """Objavljuje AckermannDriveStamped poruku na temu /drive."""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def publish_visualizations(self, original_scan, proc_ranges, extended_ranges,
                               angles, target_dist, target_angle):
        """Slanje poruka za grafički prikaz u RViz-u."""
        # Preprocesirane zrake
        self.proc_pub.publish(viz.scan_msg(original_scan, proc_ranges, angles))

        # Proširene zrake (vidi se "sjena" prepreke)
        self.extended_pub.publish(viz.scan_msg(original_scan, extended_ranges, angles))

        # Zelena strelica koja pokazuje u odabranu točku
        self.target_pub.publish(viz.target_marker(original_scan, target_dist, target_angle))


def main(args=None):
    # Koristimo SignalHandlerOptions.NO kako bismo pri Ctrl+C poslali brzinu 0 prije gašenja
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DisparityExtenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Prilikom prekida (Ctrl+C), zaustavi vozilo
        node.get_logger().info('Prekid rada (Ctrl+C). Šaljem komandu zaustavljanja (brzina = 0.0 m/s)...')
        node.publish_drive(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
