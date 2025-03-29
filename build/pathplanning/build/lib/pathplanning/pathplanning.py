import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import Pose,Point    
from visualization_msgs.msg import Marker
from open3d import *
import numpy as np
import time
import math

class graph_node():
    def __init__(self,idx,x,y):
        
        #index of the node
        self.idx = idx

        #position of the node
        self.x = x
        self.y = y

        #neighbour nodes
        self.neighbours_ = []
        self.neighbours__cost = []

    def distance_to(self,other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2) 


class Graph(Node):
    def __init__(self,map):
        super().__init__('graph')

        self.map = map

        self.nodes = []

        print('>>>>>')

        self.grid_stepsize = 2

        self.marker_publish = self.create_publisher(Marker,'/marker',10)
        self.marker_timer = self.create_timer(0.5,self.create_grid)

        self.marker_node = Marker()
        self.marker_node.header = Header()
        self.marker_node.header.frame_id = 'map'
        self.marker_node.ns = 'nodes'
        self.marker_node.type = Marker.POINTS
        self.marker_node.action = Marker.ADD
        self.marker_node.pose.position.x = 0.0
        self.marker_node.pose.position.y = 0.0
        self.marker_node.pose.position.z = 0.0
        self.marker_node.scale.x = 0.3
        self.marker_node.scale.y = 0.3
        self.marker_node.scale.z = 0.3
        self.marker_node.color.a = 1.0
        self.marker_node.color.r = 1.0
        self.marker_node.color.g = 1.0
        self.marker_node.color.b = 1.0

        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.scale.x = 0.3
        self.marker_edges_.scale.y = 0.3
        self.marker_edges_.scale.z = 0.3
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4

    def create_grid(self):

        print('<<<<<>>>>>')

        idx=0
        for x in range(0,10,self.grid_stepsize): # Chnage it to map width
            for y in range(0,10,self.grid_stepsize): # Change it to map height
                self.nodes.append(graph_node(idx,x,y))
                idx += 1

        
        count = 0
        distance_threshold = self.grid_stepsize*1.01

        for nodei in self.nodes:
            count += 1

            for nodej in self.nodes:
                if nodei != nodej:
                    distance = nodei.distance_to(nodej)

                    if distance<distance_threshold:
                        nodei.neighbours_.append(nodej)
        
                
        self.visualise_grid()
    
    def visualise_grid(self):

        print('<><><><><')

        self.marker_node.points = []
        
        for node in self.nodes:
            node.z = 0
            point = Point()
            point.x = float(node.x)
            point.y = float(node.y)
            point.z = float(node.z)
            self.marker_node.points.append(point)

        self.marker_edges_.points = []

        for nodei in self.nodes:
            for nodej_idx in range(len(nodei.neighbours_)):

                nodej = nodei.neighbours_[nodej_idx]
                nodei.z = 0
                nodej.z = 0
                point = Point()
                point.x = float(nodei.x)
                point.y = float(nodei.y)
                point.z = float(nodei.z)
                self.marker_edges_.points.append(point)
                point = Point()
                point.x = float(nodej.x)
                point.y = float(nodej.y)
                point.z = float(nodej.z)
                self.marker_edges_.points.append(point)

        self.marker_publish.publish(self.marker_node)

class Map(Node):
    def __init__(self):
        super().__init__('map')

        self.map_file = '/home/prethivi/ros2_ws/pathplanning/processed.pcd'
        self.pcd = open3d.io.read_point_cloud(self.map_file)
        self.points_ = np.asarray(self.pcd.points)
        self.map_publisher = self.create_publisher(OccupancyGrid,'/map',1)
        self.timer = self.create_timer(0.5,self.map_callback)
        
        self.resolution = 0.05
        self.min_x,self.min_y = np.min(self.points_[:,:2],axis=0)
        self.max_x,self.max_y = np.max(self.points_[:,:2],axis=0)
        self.obstacle_ = []

        self.width = int((self.max_x - self.min_x)/self.resolution)

        self.height = int((self.max_y - self.min_y)/self.resolution)

        # self.map_callback()
        # rclpy.spin(self)

    def map_callback(self):
        
        
        self.occupancy_grid = -np.ones((self.height,self.width),dtype=np.int8)

        for x,y,_ in self.points_:
            grid_x = int((x - self.min_x) / self.resolution)
            grid_x = np.clip(grid_x,0, self.width-1)
            grid_y = int((y - self.min_y) / self.resolution)
            grid_y = np.clip(grid_y,0, self.height-1)
            self.occupancy_grid[grid_y, grid_x] = 100  # Mark occupied cells
            # self.obstacle_.append[(grid_y,grid_x)]

        
        self.map_ = OccupancyGrid()
        self.map_.header = Header()
        self.map_.header.frame_id = 'map'
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.info.resolution = self.resolution
        self.map_.info.width = self.width
        self.map_.info.height = self.height
        
        self.map_.info.origin = Pose()

        self.map_.data = self.occupancy_grid.flatten().tolist()

        self.map_publisher.publish(self.map_)
        self.get_logger().info('Map Published')


class Djikstra(Node):
    def __init__(self,graph):
        super().__init__('djikstra')
        pass


def main():
    rclpy.init()
    map_data = Map() 
    graph = Graph(map_data)
    rclpy.spin(graph)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
