# ~/ros2_ws/src/lidar_test/lidar_test/test_lidar.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Subscribe to the scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.scan_callback,
            10)  # Queue size
        
        self.get_logger().info('LidarProcessor node started, listening to /scan')
        
        # Statistics
        self.scan_count = 0
        self.min_distance = float('inf')
        self.max_distance = 0.0
        
    def scan_callback(self, msg):
        # Increment scan count
        self.scan_count += 1
        
        # convert ranges to numpy array for processing
        ranges = np.array(msg.ranges)
        
        # filter out obnoxious values
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            # calculate statistics
            current_min = np.min(valid_ranges)
            current_max = np.max(valid_ranges)
            current_avg = np.mean(valid_ranges)
            
            # update overall stats
            self.min_distance = min(self.min_distance, current_min)
            self.max_distance = max(self.max_distance, current_max)
            
            # log every 10th scan
            if self.scan_count % 10 == 0:
                self.get_logger().info(
                    f'Scan #{self.scan_count}: '
                    f'{len(valid_ranges)}/{len(ranges)} valid measurements\n'
                    f'  Current: min={current_min:.2f}m, max={current_max:.2f}m, avg={current_avg:.2f}m\n'
                    f'  Overall: min={self.min_distance:.2f}m, max={self.max_distance:.2f}m'
                )
                
                # detect close obstacles
                close_obstacles = valid_ranges[valid_ranges < 0.5]  # Within 0.5m
                if len(close_obstacles) > 0:
                    self.get_logger().warning(
                        f'{len(close_obstacles)} close obstacle(s) detected (< 0.5m)'
                    )
                    
        else:
            self.get_logger().warn('No valid distance measurements in this scan')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting off republisher node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()