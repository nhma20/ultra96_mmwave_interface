import rclpy
from rclpy.node import Node
import serial
import std_msgs.msg
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import Imu
import time
import math
import numpy as np
    	
    
class mmwImuPubNode(Node):
	def __init__(self):
		super().__init__("mmwImuPubNode")
		
		self.declare_parameter('port', "/dev/ttyUSB1")
		self.declare_parameter('baud', 115200)
		self.declare_parameter('n_points', 8)
		
		self.port = self.get_parameter('port').value
		self.n_points = self.get_parameter('n_points').value
		self.baud = self.get_parameter('baud').value
		self.num_imu_vals = 6
		self.get_logger().info("mmWave_imu_publisher_node started")
		self.publisher_ = self.create_publisher(PointCloud2, '/iwr6843_pcl', 1)
		self.imu_publisher_ = self.create_publisher(Imu, '/imu', 1)
		self.serial_port, self.points, self.n_points = self.open_serial_port()
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.read_BRAM_points)

		
	def open_serial_port(self):
		print(self.port, self.baud, self.n_points)
		serial_port = serial.Serial(self.port, self.baud) # Open serial
		points = np.zeros((self.n_points,3)) # n_points points xyz
		return serial_port, points, self.n_points
	
	
	def read_BRAM_points(self):
		line = self.serial_port.readline()[:-1]
		vals = [v.decode('ISO-8859-1') for v in line.split()]
		
		if (len(vals) != (0+self.n_points*3+self.num_imu_vals)):
			print(len(vals))
			print("Mismatch between expected and received data points")
			return -1		
			
		for i in range(0,0+self.n_points*3):
			self.points[math.floor((i-0)/3),((i-0)%3)] = float(vals[i]) # 0 .. 11 = mag; 12=x, 13=y, 14=z, 15=x, ... 46=z
			
			
		cloud_arr = np.asarray(self.points).astype(np.float32) # on form [[x,y,z],[x,y,z],[x,y,z]..]
		pcl_msg = PointCloud2()
		pcl_msg.header = std_msgs.msg.Header()
		pcl_msg.header.stamp = self.get_clock().now().to_msg()
		pcl_msg.header.frame_id = 'mmwave_frame'
		pcl_msg.height = 1 # because unordered cloud
		pcl_msg.width = cloud_arr.shape[0] # number of points in cloud
		# define interpretation of pointcloud message (offset is in bytes, float32 is 4 bytes)
		pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
						    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
						    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
		#cloud_msg.is_bigendian = False # assumption        
		pcl_msg.point_step = cloud_arr.dtype.itemsize*cloud_arr.shape[1] #size of 1 point (float32 * dimensions (3 when xyz))
		pcl_msg.row_step = pcl_msg.point_step*cloud_arr.shape[0] # only 1 row because unordered
		pcl_msg.is_dense = True
		pcl_msg.data = cloud_arr.tostring()
		self.publisher_.publish(pcl_msg)
		xyz_mutex = False
		self.get_logger().info('Publishing %s points' % cloud_arr.shape[0] )
		print(self.points)
		
		#https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Imu.msg
		imu_msg = Imu()
		imu_msg.header = std_msgs.msg.Header()
		imu_msg.header.stamp = self.get_clock().now().to_msg()
		imu_msg.header.frame_id = 'imu_frame'
		imu_msg.orientation_covariance[0] = -1 # to indicate not known
		imu_msg.angular_velocity.x = float(vals[24])
		imu_msg.angular_velocity.y = float(vals[25])
		imu_msg.angular_velocity.z = float(vals[26])
		imu_msg.linear_acceleration.x = float(vals[27])
		imu_msg.linear_acceleration.y = float(vals[28])
		imu_msg.linear_acceleration.z = float(vals[29])
		self.imu_publisher_.publish(imu_msg)
		self.get_logger().info('Publishing IMU data')
		print("ang_vel_x: ", float(vals[24]))
		print("ang_vel_y: ", float(vals[25]))
		print("ang_vel_z: ", float(vals[26]))
		print("lin_acc_x: ", float(vals[27]))
		print("lin_acc_y: ", float(vals[28]))
		print("lin_acc_z: ", float(vals[29]))
		
		
		return 1
			
        
def main(args=None):
	rclpy.init(args=args)
	print(args)
	node = mmwImuPubNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
    
if __name__ == "__main__":
	main()
