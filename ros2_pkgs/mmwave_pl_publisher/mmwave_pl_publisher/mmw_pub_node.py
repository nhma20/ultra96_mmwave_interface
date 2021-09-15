import rclpy
from rclpy.node import Node
import serial
import std_msgs.msg
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import time
import math
import numpy as np
    	
    
class mmwPubNode(Node):
	def __init__(self):
		super().__init__("mmwPubNode")
		
		self.declare_parameter('port', "/dev/ttyUSB1")
		self.declare_parameter('baud', 115200)
		self.declare_parameter('n_points', 8)
		
		self.port = self.get_parameter('port').value
		self.n_points = self.get_parameter('n_points').value
		self.baud = self.get_parameter('baud').value
		
		self.get_logger().info("mmWave_publisher_node started")
		self.publisher_ = self.create_publisher(PointCloud2, 'mmwave_pub/pcl', 1)
		self.serial_port, self.points, self.n_points = self.open_serial_port()
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.read_BRAM_points)

		
	def open_serial_port(self):
		print(self.port, self.baud, self.n_points)
		serial_port = serial.Serial(self.port, self.baud) # Open serial
		points = np.zeros((self.n_points,3)) # n_points points xyz
		return serial_port, points, self.n_points
	
	
	def read_BRAM_points(self):
		print("start")
		line = self.serial_port.readline()[:-1]
		print("next")
		#vals = [float(v) for v in line.split()] # maybe wrong cast?
		vals = [0, 0,0,0,0,0,0,0,0,0,0,0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]
		if (len(vals) != (12+self.n_points*3)):
		#	print(len(vals))
			return -1
		for i in range(12,12+self.n_points*3):
			self.points[math.floor((i-12)/3),((i-12)%3)] = vals[i] # 0 .. 11 = mag; 12=x, 13=y, 14=z, 15=x, ... 46=z
		print("middle")
		cloud_arr = np.asarray(self.points).astype(np.float32) # on form [[x,y,z],[x,y,z],[x,y,z]..]
		pcl_msg = PointCloud2()
		pcl_msg.header = std_msgs.msg.Header()
		pcl_msg.header.stamp = self.get_clock().now().to_msg()
		pcl_msg.header.frame_id = 'mmwave_pl_frame'
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
		print("end")
		return 1
			
        
def main(args=None):
	rclpy.init(args=args)
	print(args)
	node = mmwPubNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
    
if __name__ == "__main__":
	main()
