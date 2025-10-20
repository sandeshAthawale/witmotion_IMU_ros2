import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R
from .src.bwt901cl import BWT901CL


class Imu901cl(Node):
    def __init__(self, time_interval=1.0):
        super().__init__('imu_bwt901cl')

        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        baudrate = self.declare_parameter('baudrate', 115200).value
        log_dir_param = self.declare_parameter('log_dir', str(Path.cwd() / 'log')).value
        
        # Create a TransformBroadcaster to broadcast IMU's pose
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a publisher for IMU raw data
        self.imu_publisher = self.create_publisher(Imu, 'imu_data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu_magnetic_field', 10)
        self.temp_publisher = self.create_publisher(Temperature, 'imu_temperature', 10)
        self.angle_publisher = self.create_publisher(Vector3Stamped, 'imu_angle_deg', 10)
        
        # Timer for publishing data
        self.tmr = self.create_timer(time_interval, self.timer_callback)
        
        # Initialize IMU sensor
        self.get_logger().info(f'Connecting IMU on {port} @ {baudrate} baud')
        self.imu_sensor = BWT901CL(port, baudrate=baudrate)

        # Fixed covariance assumptions (diagonal matrices with zero cross-correlation)
        self.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]
        self.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]
        self.linear_acceleration_covariance = [
            0.10, 0.0, 0.0,
            0.0, 0.10, 0.0,
            0.0, 0.0, 0.10
        ]
        self.temperature_variance = 0.25  
        '''
        default variance for the temperature measurement - so the sensor_msgs/Temperature message carries an uncertainty estimate. 
        ROS defines temperature in Â°C and variance in (Â°C)Â². Since the BWT901C spec doesnâ€t publish its exact temperature noise, 
        I chose 0.25â€¯Â°CÂ² (ââ€¯0.5â€¯Â°C standard deviation) as a conservative placeholderâ€enough to signal â€moderate confidenceâ€ without pretending the sensor is perfect. 
        Downstream nodes can use that to weight temperature data appropriately, and you can tighten or loosen it once you have a better estimate. 
        '''

        # Prepare CSV logging
        log_dir = Path(log_dir_param).expanduser()
        if not log_dir.is_absolute():
            log_dir = Path.cwd() / log_dir
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        self.log_file_path = log_dir / f'{timestamp}.csv'
        self.log_file = self.log_file_path.open('w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp_iso',
            'angle_x_deg',
            'angle_y_deg',
            'angle_z_deg',
            'angular_velocity_x_deg_s',
            'angular_velocity_y_deg_s',
            'angular_velocity_z_deg_s',
            'accel_x_m_s2',
            'accel_y_m_s2',
            'accel_z_m_s2',
            'temperature_c',
            'magnetic_x',
            'magnetic_y',
            'magnetic_z',
            'quaternion_x',
            'quaternion_y',
            'quaternion_z',
            'quaternion_w'
        ])
        self.get_logger().info(f'Logging remaining IMU fields to {self.log_file_path}')



    def timer_callback(self):
        # Get data from IMU
        angle, angular_velocity, accel, temp, magnetic, quaternion, time = self.imu_sensor.getData()
        
        # Assume angle is a list of Euler angles in degrees: [roll, pitch, yaw]
        # Convert Euler angles to quaternion
        r = R.from_euler('xyz', angle, degrees=True)  # Convert from degrees to quaternion
        quaternion = r.as_quat()  # Returns [x, y, z, w]

        # Create a TransformStamped message for broadcasting IMU pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # The fixed parent frame
        t.child_frame_id = 'imu_link'    # The IMU frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

        # Publish the raw IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Fill orientation with quaternion (if available)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        
        # Fill angular velocity (assuming angular_velocity is a list [x, y, z])
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]

        # Fill linear acceleration (assuming accel is a list [x, y, z])
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.orientation_covariance = self.orientation_covariance
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        # Publish the IMU raw data message
        self.imu_publisher.publish(imu_msg)

        # Publish magnetic field
        magnetic_msg = MagneticField()
        magnetic_msg.header = imu_msg.header
        magnetic_msg.magnetic_field.x = float(magnetic[0])
        magnetic_msg.magnetic_field.y = float(magnetic[1])
        magnetic_msg.magnetic_field.z = float(magnetic[2])
        self.mag_publisher.publish(magnetic_msg)

        # Publish temperature
        temp_msg = Temperature()
        temp_msg.header = imu_msg.header
        temp_msg.temperature = float(temp)
        temp_msg.variance = self.temperature_variance
        self.temp_publisher.publish(temp_msg)

        # Publish Euler angles in degrees
        angle_msg = Vector3Stamped()
        angle_msg.header = imu_msg.header
        angle_msg.vector.x = float(angle[0])
        angle_msg.vector.y = float(angle[1])
        angle_msg.vector.z = float(angle[2])
        self.angle_publisher.publish(angle_msg)

        timestamp_iso = datetime.now().isoformat(timespec='milliseconds')
        self.csv_writer.writerow([
            timestamp_iso,
            f'{angle[0]:.3f}',
            f'{angle[1]:.3f}',
            f'{angle[2]:.3f}',
            f'{angular_velocity[0]:.3f}',
            f'{angular_velocity[1]:.3f}',
            f'{angular_velocity[2]:.3f}',
            f'{accel[0]:.3f}',
            f'{accel[1]:.3f}',
            f'{accel[2]:.3f}',
            f'{temp:.3f}',
            f'{magnetic[0]:.3f}',
            f'{magnetic[1]:.3f}',
            f'{magnetic[2]:.3f}',
            f'{quaternion[0]:.3f}',
            f'{quaternion[1]:.3f}',
            f'{quaternion[2]:.3f}',
            f'{quaternion[3]:.3f}'
        ])
        self.log_file.flush()

        self.get_logger().info(
            f'Angles (deg) -> roll: {angle[0]:.2f}, pitch: {angle[1]:.2f}, yaw: {angle[2]:.2f}'
        )

    def destroy_node(self):
        if hasattr(self, 'log_file') and self.log_file and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node_imu_bwt901cl = Imu901cl(time_interval=0.1)
    rclpy.spin(node_imu_bwt901cl)

    node_imu_bwt901cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
