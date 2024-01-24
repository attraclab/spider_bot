import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import  Float32MultiArray
import csv
import os
from datetime import datetime

class TrainDataCollector(Node):

	def __init__(self):

		super().__init__('train_data_collector_node')
		print("start train_data_collector")


		self.train_data_sub = self.create_subscription(Float32MultiArray, "/spider/train_data", self.train_data_callback, 10)

		now = datetime.fromtimestamp(time.time())
		filename = now.strftime("%Y-%m-%d-%H-%M-%S")
		save_path = "/home/rasheed/dev_ws/src/spider_bot/spider_bot"
		self.csv_filename = filename + ".csv"
		self.csv_file_path = os.path.join(save_path, self.csv_filename)
		self.data_file = open(self.csv_file_path , "w+")
		self.dataCSVWriter = csv.writer(self.data_file , delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
		self.dataCSVWriter.writerow(['diff_x', 'diff_y', 'diff_z', 'cur_2', 'cur_3'])
		self.line_counter = 1



	def train_data_callback(self, msg):


		diff_x = msg.data[0]
		diff_y = msg.data[1]
		diff_z = msg.data[2]
		cur_2 = msg.data[3]
		cur_3 = msg.data[4]
		print("line: {:d} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(self.line_counter, diff_x, diff_y, diff_z, cur_2, cur_3))
		self.dataCSVWriter.writerow([diff_x, diff_y, diff_z, cur_2, cur_3])
		self.line_counter += 1


def main(args=None):
	rclpy.init(args=None)
	node = TrainDataCollector()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()