import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from math import cos, radians, sqrt

class FusionWeightNode(Node):
    def __init__(self):
        super().__init__('fusion_weight_node')

        # Subscriber for IMU odometry
        self.create_subscription(Odometry, '/mavros/local_position/odom', self.imu_callback, 10)

        # Visual GPS accuracy (std dev in meters)
        self.vgps_accuracy_m = 1.5

        # Example latitude (should update if you get GPS latitude dynamically)
        self.latitude_deg = 26.5

        # Publisher for fusion weights
        self.weight_pub = self.create_publisher(Float32MultiArray, '/fusion/weights', 10)

    def convert_accuracy_meters_to_degrees(self, accuracy_m):
        meters_per_deg_lat = 111139
        meters_per_deg_lon = 111139 * cos(radians(self.latitude_deg))

        sigma_lat = accuracy_m / meters_per_deg_lat
        sigma_lon = accuracy_m / meters_per_deg_lon

        return sigma_lat, sigma_lon

    def imu_callback(self, msg):
        cov = msg.pose.covariance

        if cov[0] == 0.0 or cov[7] == 0.0:
            self.get_logger().warn('IMU covariance is zero or missing.')
            return

        # IMU variances in meters squared
        imu_pos_var_x = cov[0]  # Longitude variance
        imu_pos_var_y = cov[7]  # Latitude variance

        # Convert variance to std dev
        sigma_imu_lon = sqrt(imu_pos_var_x)
        sigma_imu_lat = sqrt(imu_pos_var_y)

        # Convert IMU std dev from meters to degrees separately for lat and lon
        imu_lat_sigma_deg, _ = self.convert_accuracy_meters_to_degrees(sigma_imu_lat)
        _, imu_lon_sigma_deg = self.convert_accuracy_meters_to_degrees(sigma_imu_lon)

        imu_lat_var = imu_lat_sigma_deg ** 2
        imu_lon_var = imu_lon_sigma_deg ** 2

        # Visual GPS std dev converted to degrees
        sigma_vgps_lat, sigma_vgps_lon = self.convert_accuracy_meters_to_degrees(self.vgps_accuracy_m)
        vgps_lat_var = sigma_vgps_lat ** 2
        vgps_lon_var = sigma_vgps_lon ** 2

        # Compute fusion weights (inverse variance weighting)
        w_imu_lat = 1 / imu_lat_var / (1 / imu_lat_var + 1 / vgps_lat_var)
        w_vgps_lat = 1 - w_imu_lat

        w_imu_lon = 1 / imu_lon_var / (1 / imu_lon_var + 1 / vgps_lon_var)
        w_vgps_lon = 1 - w_imu_lon

        # Publish fusion weights
        weight_msg = Float32MultiArray()
        weight_msg.data = [w_imu_lat, w_vgps_lat, w_imu_lon, w_vgps_lon]
        self.weight_pub.publish(weight_msg)

        self.get_logger().info(
            f'Weights => Lat: IMU={w_imu_lat:.3f}, VGPS={w_vgps_lat:.3f} | Lon: IMU={w_imu_lon:.3f}, VGPS={w_vgps_lon:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = FusionWeightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
