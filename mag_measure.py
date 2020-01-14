import rclpy
from rclpy.node import Node 
import sensor_msgs.msg
from rclpy.qos import  qos_profile_sensor_data
import logging
import numpy as np

class MagLoggerNode(Node):
    def __init__(self):
        super().__init__("mag_logger_node")
        self.mag_sub = self.create_subscription(sensor_msgs.msg.MagneticField,"imu/mag_data", self.mag_data,qos_profile_sensor_data)
        self.stats = {}
        self.stats['x'] = [] 
        self.stats['y'] = []
        self.stats['z'] = []


    def mag_data(self, mag_data):
        self.stats['x'].append(mag_data.magnetic_field.x)
        self.stats['y'].append(mag_data.magnetic_field.y)
        self.stats['z'].append(mag_data.magnetic_field.z)
        #self.spit_stats()

    def spit_stats(self):
        for key,val in self.stats.items():
            if len(val) == 0:
                continue
            if len(val) > 100:
                val = val[1:]
            logging.info(f"stat {key} has std_dev: {np.std(val)}")



def main(args=None): 
    logging.basicConfig(
            format='%(asctime)s %(levelname)-8s %(message)s',
            level=logging.INFO,
            datefmt='%Y-%m-%d %H:%M:%S')

    rclpy.init(args=args)

    log_node = MagLoggerNode()
    try:
        rclpy.spin(log_node)
    except:
        logging.info(f"hit exception")
        log_node.spit_stats()
if __name__ == '__main__':
    main()
