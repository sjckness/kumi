import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class FrontDistanceNode(Node):
    def __init__(self):
        super().__init__('front_distance_node')

        # param: ampiezza settore frontale in radianti (es. ±10°)
        self.declare_parameter('front_angle_width_deg', 10.0)
        width_deg = self.get_parameter('front_angle_width_deg').value
        self.front_angle_width = math.radians(width_deg)

        self.subscription = self.create_subscription(
            LaserScan,
            '/camera/scan',  
            self.scan_callback,
            100
        )

        self.pub_distance = self.create_publisher(
            Float32,
            '/front_obstacle_distance',
            100
        )

        self.get_logger().info(
            f'FrontDistanceNode started, front sector ±{width_deg} deg'
        )

    def scan_callback(self, msg: LaserScan):
        # prendiamo un settore centrato a 0 rad
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # indice dell'angolo 0 (davanti)
        idx_center = int(round((0.0 - angle_min) / angle_inc))

        half_width_rad = self.front_angle_width
        half_idx = int(round(half_width_rad / abs(angle_inc)))

        idx_start = max(0, idx_center - half_idx)
        idx_end = min(len(msg.ranges) - 1, idx_center + half_idx)

        valid_ranges = []
        for i in range(idx_start, idx_end + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                valid_ranges.append(r)

        if not valid_ranges:
            # niente davanti (tutto inf / nan)
            return

        min_dist = min(valid_ranges)

        # log opzionale
        self.get_logger().info(f'Front obstacle distance: {min_dist:.2f} m')

        # pubblica
        msg_out = Float32()
        msg_out.data = float(min_dist)
        self.pub_distance.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = FrontDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
