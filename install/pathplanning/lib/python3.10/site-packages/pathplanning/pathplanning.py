import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from open3d import *
import numpy as np
import time


class Map(Node):
    def __init__(self):
        super().__init__('map')

        self.map_file = '/home/prethivi/ros2_ws/pathplanning/room_scan2.pcd'
        self.pcd = open3d.io.read_point_cloud(self.map_file)
        self.points_ = np.asarray(self.pcd.points)
        self.map_publisher = self.create_publisher(OccupancyGrid,'/map',10)
        self.timer = self.create_timer(0.5,self.map_callback)

    def map_callback(self):
        self.map_ = OccupancyGrid()
        resolution = 0.05
        self.min_x,self.min_y = np.min(self.points_[:,:2],axis=0)
        self.max_x,self.max_y = np.max(self.points_[:,:2],axis=0)

        self.width = int((self.max_x - self.min_x)/resolution)

        self.height = int((self.max_y - self.min_y)/resolution)


        self.occupancy_grid = -np.ones((self.height,self.width),dtype=np.int8)

        for x,y,_ in self.points_:
            grid_x = int((x - self.min_x) / resolution)
            grid_x = np.clip(grid_x,0, self.width-1)
            grid_y = int((y - self.min_y) / resolution)
            grid_y = np.clip(grid_y,0, self.height-1)
            self.occupancy_grid[grid_y, grid_x] = 100  # Mark occupied cells

        self.map_.header = Header()
        self.map_.header.frame_id = 'map'
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.info.resolution = resolution
        self.map_.info.width = self.width
        self.map_.info.height = self.height
        
        self.map_.info.origin = Pose()
        self.map_.info.origin.position.x = 0.0
        self.map_.info.origin.position.y = 0.0

        self.map_.data = self.occupancy_grid.flatten().tolist()

        self.map_publisher.publish(self.map_)
        self.get_logger().info('Map Published')

def main():
    rclpy.init()
    map = Map()
    rclpy.spin(map)
    map.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()



        
