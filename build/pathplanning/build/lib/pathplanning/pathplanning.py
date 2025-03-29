import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import Pose,Point    
from visualization_msgs.msg import Marker
import open3d as o3d
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

        self.parent_node = None #Parent Node of the neighbour

    def distance_to(self,other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2) 


class Graph(Node):
    def __init__(self):
        super().__init__('graph')

        self.map_height = 0
        self.map_width = 0
        self.map_data = None
        self.resolution = None

        self.nodes = []
        self.obstacles_ = None
        self.obstacles_position = None
        self.map_origin_x = None
        self.map_origin_y = None

        print('>>>>>')

        self.grid_stepsize = 1

        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
 
        self.marker_publish = self.create_publisher(Marker,'/marker',10)

        self.marker_node = Marker()
        self.marker_node.header = Header()
        self.marker_node.header.frame_id = 'map'
        self.marker_node.ns = 'nodes'
        self.marker_node.type = Marker.POINTS
        self.marker_node.action = Marker.ADD
        self.marker_node.pose.position.x = 0.0
        self.marker_node.pose.position.y = 0.0
        self.marker_node.pose.position.z = 0.0
        self.marker_node.scale.x = 0.05
        self.marker_node.scale.y = 0.05
        self.marker_node.scale.z = 0.05
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
        self.marker_edges_.scale.x = 0.01
        self.marker_edges_.scale.y = 0.01
        self.marker_edges_.scale.z = 0.01
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 0.0
        self.marker_edges_.color.g = 0.0
        self.marker_edges_.color.b = 0.0

        self.marker_path = Marker()
        self.marker_path.header.frame_id = "map"
        self.marker_path.ns = "path"
        self.marker_path.id = 0
        self.marker_path.type = Marker.LINE_LIST
        self.marker_path.action = Marker.ADD
        self.marker_path.pose.position.x = 0.0
        self.marker_path.pose.position.y = 0.0
        self.marker_path.pose.position.z = 0.0
        self.marker_path.scale.x = 0.01
        self.marker_path.scale.y = 0.01
        self.marker_path.scale.z = 0.01
        self.marker_path.color.a = 1.0
        self.marker_path.color.r = 0.0
        self.marker_path.color.g = 1.0
        self.marker_path.color.b = 0.0


    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.obstacles_ = np.where(self.map_data >= 50)
        self.obstacles_position = list(zip(self.obstacles_[0],self.obstacles_[1]))
        self.get_logger().info("Graph received map data")
        self.create_grid()

    def create_grid(self):

        print('<<<<<>>>>>')
        if self.map_data is None:
            self.get_logger("No Map Data Recieved")
            return

        idx=0
        for i in range(0, self.map_width, self.grid_stepsize): 
            for j in range(0, self.map_height, self.grid_stepsize): 
                x = i * self.resolution + self.map_origin_x
                y = j * self.resolution + self.map_origin_y
                if self.obstacle_node(x,y):
                    continue
                else:
                    self.nodes.append(graph_node(idx,x,y)) #create Node
                idx += 1

        
        distance_threshold = self.grid_stepsize*0.5

        for nodei in self.nodes:
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
            point = Point()
            point.x = float(node.x)
            point.y = float(node.y)
            point.z = 0.0
            self.marker_node.points.append(point)

        self.marker_edges_.points = []

        for nodei in self.nodes:
            for nodej_idx in range(len(nodei.neighbours_)):

                nodej = nodei.neighbours_[nodej_idx]
                point = Point()
                point.x = float(nodei.x)
                point.y = float(nodei.y)
                point.z = 0.0
                self.marker_edges_.points.append(point)
                point = Point()
                point.x = float(nodej.x)
                point.y = float(nodej.y)
                point.z = 0.0
                self.marker_edges_.points.append(point)

        self.marker_publish.publish(self.marker_node)
        self.marker_publish.publish(self.marker_edges_)

    def get_closest_node(self,xy):
        best_distance = 9999999
        best_idx = None
        
        for node in self.nodes:
            distance = math.sqrt((node.x-xy[0])**2 + (node.y-xy[1])**2)
            if distance < best_distance:
                best_distance = distance
                best_idx = node.idx

        return best_idx
    
    def obstacle_node(self,x,y):
        min_distance = 9999999
        obstacle_threshold = self.resolution/2
        
        for i in range(len(self.obstacles_position)):
            obs_x = self.obstacles_position[i][1] * self.resolution + self.map_origin_x
            obs_y = self.obstacles_position[i][0] * self.resolution + self.map_origin_y
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if distance < min_distance:
                min_distance = distance
        
        if min_distance <= obstacle_threshold:
            return True
        return False
    
    def is_path_obstructed(self, x1, y1, x2, y2):
        num_points = int(math.dist([x1, y1], [x2, y2]) / self.resolution * 2) 
        for i in range(num_points + 1):
            px = x1 + i * (x2 - x1) / num_points
            py = y1 + i * (y2 - y1) / num_points
            if self.obstacle_node(px, py):  
                return True
        return False



class Map(Node):
    def __init__(self):
        super().__init__('map')

        self.map_file = '/home/prethivi/ros2_ws/pathplanning/processed.pcd'
        self.pcd = o3d.io.read_point_cloud(self.map_file)
        self.points_ = np.asarray(self.pcd.points)
        self.map_publisher = self.create_publisher(OccupancyGrid,'/map',10)
        self.timer = self.create_timer(0.5,self.map_callback)
        
        self.resolution = 0.25
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
        self.map_.info.origin.position.x = self.min_x
        self.map_.info.origin.position.y = self.min_y
        self.map_.info.origin.position.z = 0.0

        self.map_.data = self.occupancy_grid.flatten().tolist()

        self.map_publisher.publish(self.map_)
        self.get_logger().info('Map Published')


class Djikstra(Node):
    def __init__(self,graph,startxy=None,goalxy=None):
        super().__init__('djikstra')

        self.graph_ = graph

        self.start_idx = self.graph_.get_closest_node(startxy)
        self.goal_idx = self.graph_.get_closest_node(goalxy)

        if self.start_idx == None or self.goal_idx == None:
            pass

        else:

            if self.start_idx == self.goal_idx:
                self.get_logger().info('Goal reached !!')
            else:
                self.djikstra_search()

    
    def djikstra_search(self):

        self.unvisited_set_ = []
        self.visited_set_ = []
        best_distance = 9999999
        best_neighbour = None
        

        self.unvisited_set_.append(self.graph_.nodes[self.start_idx]) ## Adding start node to unvisited set.

        current_node = self.graph_.nodes[self.start_idx] # moving the start node to current node. 

        for node in current_node.neighbours_:
            self.unvisited_set_.append(node)
            distance = node.distance_to(current_node)
            if distance<best_distance:
                best_neighbour = node
                best_neighbour.parent_node = current_node
                current_node = best_neighbour
        
        self.visited_set_.append(best_neighbour)
        
        

        print(self.visited_set_)


def main():
    rclpy.init()
    map_data = Map() 
    graph = Graph()

    startx=0
    starty=0
    goalx=15
    goaly=15

    djikstra_algorithm = Djikstra(graph,[startx,starty],[goalx,goaly])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(map_data)
    executor.add_node(graph)
    executor.add_node(djikstra_algorithm)

    try:
        executor.spin() 
    finally:
        map_data.destroy_node()
        graph.destroy_node()
        djikstra_algorithm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



        
