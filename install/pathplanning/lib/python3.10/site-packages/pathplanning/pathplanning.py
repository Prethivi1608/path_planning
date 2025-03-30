import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import Pose,Point,PoseStamped    
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
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

        self.parent_idx = None #Parent Node of the neighbour
        self.cost_to_node = 9999999

        self.cost_to_node_heuristic = None

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

        self.grid_created = False

        print('inside graph')

        self.grid_stepsize = 1

        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
 
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

        print('create grid')
        while self.map_data is None:
            self.get_logger().info("No Map Data Recieved!!")

        idx=0
        for i in range(0, self.map_width, self.grid_stepsize): 
            for j in range(0, self.map_height, self.grid_stepsize): 
                x = i * self.resolution + self.map_origin_x
                y = j * self.resolution + self.map_origin_y
                if self.obstacle_node(x,y):
                    continue
                else:
                    self.nodes.append(graph_node(idx,x,y)) #create Node
                    self.get_logger().info(f'{idx} Nodes are created')
                idx += 1

        
        distance_threshold = self.grid_stepsize*0.5

        for nodei in self.nodes:
            for nodej in self.nodes:
                if nodei != nodej:
                    distance = nodei.distance_to(nodej)

                    if distance<distance_threshold:
                        nodei.neighbours_.append(nodej)
                        nodei.neighbours__cost.append(distance)
        
                
        self.grid_created = True
        self.visualise_grid()
        return
        

    def is_grid_created(self):
        return self.grid_created
    
    def visualise_grid(self):

        print('viz grid')

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

        Astar(self,[0,0],[40,40])
        
        time.sleep(100)
        



    def get_closest_node(self,xy):

        print('Inside closest node')

        best_distance = float('inf')
        best_idx = None

        if self.nodes is None:
            print('No Nodes')
            return
        
        for node in self.nodes:
            print(node.x)
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
        time.sleep(1000)
        return


class Astar(Node):
    def __init__(self,graph,startxy=None,goalxy=None):
        super().__init__('astar')
        self.graph_ = graph

        self.path = []


        self.path_publish = self.create_publisher(Path,'/path',10)

        self.start_idx = self.graph_.get_closest_node(startxy)
        self.goal_idx = self.graph_.get_closest_node(goalxy)
        print(f'goal:{self.goal_idx}')

        if self.start_idx == None or self.goal_idx == None:
            print('No Start and Goal Recieved')

        else:
            print("Goal Recieved")

            if self.start_idx == self.goal_idx:
                print('Goal reached !!')
            else:
                self.astar_search()

    def distance_to_goal(self,idx):
        return self.graph_.nodes[idx].distance_to(self.graph_.nodes[self.goal_idx])
    
    def cost_to_node_cal(self,idx):

        parent_idx = self.graph_.nodes[idx].parent_idx
        self.graph_.nodes[idx].cost_to_node = self.graph_.nodes[parent_idx].cost_to_node + self.graph_.nodes[idx].neighbours__cost[0]
        return self.graph_.nodes[idx].cost_to_node
    
    def cost_to_node_heuristic(self,idx,heuristic_weight):
        self.graph_.nodes[idx].cost_to_node_heuristic = self.cost_to_node_cal(idx) + (self.distance_to_goal(idx)*heuristic_weight)

    
    def astar_search(self):

        self.unvisited_set_ = []
        self.visited_set_ = []
        minimum_distance = 99999
        minimum_idx = None

        self.graph_.nodes[self.start_idx].cost_to_node = 0
        self.graph_.nodes[self.start_idx].cost_to_node_heuristic = 0

        current_idx = self.start_idx
        
        heuristic_weight = 1 #Zero for Djikstra 1 for Astar
        
        self.unvisited_set_.append(current_idx)

        while self.unvisited_set_:

            self.visited_set_.append(current_idx)
            self.unvisited_set_.remove(current_idx)


            for neighbour in self.graph_.nodes[current_idx].neighbours_:
                idx = neighbour.idx
                if idx in self.visited_set_:
                    continue
                if idx not in self.unvisited_set_:
                    self.unvisited_set_.append(idx)
                    self.graph_.nodes[idx].parent_idx = current_idx
                
                self.cost_to_node_heuristic(idx,heuristic_weight)
                        
            for unv_idx in self.unvisited_set_:
                minimum_distance = 99999
                minimum_idx = None
                print(unv_idx)
                dist = self.graph_.nodes[unv_idx].cost_to_node_heuristic
                if dist<minimum_distance:
                    minimum_distance = dist
                    minimum_idx = unv_idx
                    
            if minimum_idx == None:
                print("Path cannot be found")
    
            current_idx = minimum_idx

            if current_idx == self.goal_idx:
                print('Goal reached !!')
                self.get_path()
    

    def get_path(self):
        current_idx = self.goal_idx
        while current_idx != self.start_idx:
            self.path.append(current_idx)
            current_idx = self.graph_.nodes[current_idx].parent_idx
        self.path.append(self.start_idx)
        self.path.reverse()
        self.visualise_path(self.path)
    
    def visualise_path(self,path):
        print("Inside visualise path")
        msg = Path()
        msg.header.frame_id = 'map'
        for idx in path:
            pose = PoseStamped()
            pose.pose.position.x = self.graph_.nodes[idx].x
            pose.pose.position.y = self.graph_.nodes[idx].y
            pose.pose.position.z = 0.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        self.path_publish.publish(msg)
        self.get_logger().info('Path Published')


def main():
    rclpy.init()
    map_data = Map()
    graph = Graph()

    startx=5
    starty=15
    goalx=14
    goaly=50

    # djikstra_algorithm = Astar(graph,[startx,starty],[goalx,goaly])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(map_data)
    executor.add_node(graph)
    # executor.add_node(djikstra_algorithm)

    executor.spin()
    
    map_data.destroy_node()
    graph.destroy_node()
    # djikstra_algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
