#!/usr/bin/env python3

import csv
from collections import defaultdict
import heapq
from car_ros_msgs.msg import Trajectory, Waypoint
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
import argparse
import numpy as np

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Initialize the default dictionary for segment ID to new points mapping
        self.segmentIdToNewPoints = defaultdict(lambda: defaultdict(list))

        # Initialize global list for blockage points
        self.global_blockage = []
#modified directory
        # Path to the uploaded CSV files
        self.csv_file_path = '~/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/allpoints.csv'

        self.blockage_lat = None
        self.blockage_lon = None

        # Initialize the publisher regardless of final location check
        self.pub = self.create_publisher(Trajectory, 'global_planner', 10)
        self.sub = self.create_subscription(Float64MultiArray, 'LLA_lane_change_topic', self.lla_change_trajectory_callback, 10)

        # Now load the final location
        final_location_file = '/root/ros2_ws/src/CarROS/car_ros2/car_ros_controls/config/final_location.txt'
        final_location = self.get_final_location(final_location_file, self.csv_file_path)

        if final_location is None:
            self.get_logger().error("Failed to retrieve final location. Aborting initialization.")
            return

        # Set parameters and waypoints based on the final location
        self.declare_parameter('lane_id_list_float', [3007, int(final_location)])
        self.rosparam_waypoints = self.get_parameter('lane_id_list_float').get_parameter_value().integer_array_value
        self.input_waypoints = list(map(int, self.rosparam_waypoints))

        self.get_logger().info(f"Received parameters: {self.rosparam_waypoints}")


        self.pub = self.create_publisher(Trajectory, 'global_planner', 10)
        self.sub = self.create_subscription(Float64MultiArray, 'LLA_lane_change_topic', self.lla_change_trajectory_callback, 10)

    def lla_change_trajectory_callback(self, msg):
        self.get_logger().info(f"Received new trajectory: {msg.data}")

        self.blockage_lat = msg.data[0]
        self.blockage_lon = msg.data[1]

        self.get_logger().info(f"The Decider set the block point at: {self.blockage_lat}, {self.blockage_lon}")

        closest_point_id = self.find_closest_point(self.blockage_lat, self.blockage_lon, self.csv_file_path)
        if closest_point_id not in self.global_blockage:
            self.global_blockage.append(closest_point_id)

        self.main_execution(self.blockage_lat, self.blockage_lon)

    def read_and_organize_data(self, csv_file_path):
        segment_to_lanes = defaultdict(lambda: defaultdict(list))
        with open(csv_file_path, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                segment_id = row['segment']
                lane_id = row['lane_id']
                point_id = row['ID']
                segment_to_lanes[segment_id][lane_id].append(point_id)
                self.segmentIdToNewPoints[segment_id][lane_id].append(point_id)
        return segment_to_lanes

    
    def get_final_location(self, file_path, csv_file_path):
        try:
            with open(file_path, 'r') as file:
                line = file.readline().strip()
                lat, lon = map(float, line.split(','))
                self.get_logger().info(f"Final location read from file: Latitude={lat}, Longitude={lon}")

                closest_point_id = self.find_closest_point(lat, lon, csv_file_path)
                return closest_point_id
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {file_path}")
            return None
        except ValueError:
            self.get_logger().error(f"Invalid integer value in file: {file_path}")
            return None

    def connect_points(self, segment_to_lanes):
        connections = []
        for segment_id, lanes in segment_to_lanes.items():
            sorted_lanes = sorted(lanes.items(), key=lambda x: int(x[0]))
            for lane_id, points in sorted_lanes:
                for i in range(len(points) - 1):
                    connections.append((points[i], points[i+1]))
            for i in range(len(sorted_lanes) - 1):
                current_lane_points = sorted_lanes[i][1]
                next_lane_points = sorted_lanes[i+1][1]
                for j in range(min(len(current_lane_points), len(next_lane_points))):
                    connections.append((current_lane_points[j], next_lane_points[j]))
                    connections.append((next_lane_points[j], current_lane_points[j]))
        return connections

    def connect_segments(self):
        connection_list = []
        lane_connections_path = '~/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/laneconnection.csv'
        with open(lane_connections_path, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader, None)
            for row in csv_reader:
                from_lane_id, from_segment_id, to_lane_id, to_segment_id = row
                start = max(self.segmentIdToNewPoints[from_segment_id][from_lane_id]) if self.segmentIdToNewPoints[from_segment_id][from_lane_id] else -69
                end = min(self.segmentIdToNewPoints[to_segment_id][to_lane_id]) if self.segmentIdToNewPoints[to_segment_id][to_lane_id] else -69
                if start == -69 or end == -69:
                    continue
                connection_list.append((start, end))
        return connection_list

    def apply_blockage(self, graph, blocked_points=None):
        if blocked_points is None:
            return
        
        for blocked_point in blocked_points:
            if blocked_point in graph:
                del graph[blocked_point]
            for node in list(graph.keys()):
                if blocked_point in graph[node]:
                    graph[node].remove(blocked_point)

    def create_graph(self, connections):
        graph = defaultdict(list)
        for start, end in connections:
            graph[start].append(end)
        return graph
    

# Handling Missing Nodes Gracefully in Dijkstra\u2019s Algorithm
# Modify dijkstra to handle missing nodes gracefully by checking if each neighbor is present in distances before accessing it. 


    def dijkstra(self, graph, start_point, end_point):
        distances = {vertex: float('infinity') for vertex in graph}
        previous = {vertex: None for vertex in graph}
        distances[start_point] = 0
        priority_queue = [(0, start_point)]

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)
            if current_vertex == end_point:
                break
            for neighbor in graph.get(current_vertex, []):  # Safely get neighbors
                if neighbor not in distances:
                    self.get_logger().warning(f"Skipping unknown neighbor {neighbor}")
                    continue
                distance = current_distance + 1
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_vertex
                    heapq.heappush(priority_queue, (distance, neighbor))

        
        path = []
        current = end_point
        while current is not None:
            path.append(current)
            current = previous[current]
        path.reverse()

        return path if path[0] == start_point else []

    def calculate_shortest_path(self, graph, start_point, end_point):
        return self.dijkstra(graph, start_point, end_point)

    def calculate_shortest_waypoints(self, graph, waypoints):
        self.get_logger().info(f"Processing waypoints: {waypoints}")
        complete_path = []
        path_segment = []
        start_point = str(waypoints[0])

        for end_point in waypoints[1:]:
            end_point = str(end_point)
            self.get_logger().info(f'This is the end point: {end_point}')
            if start_point not in graph or end_point not in graph:
                self.get_logger().error(f"One of the points {start_point} or {end_point} is not in the graph")
                return []
            path_segment = self.calculate_shortest_path(graph, start_point, end_point)
            if not path_segment:
                self.get_logger().error(f"No path found from {start_point} to {end_point}")
                return []
            complete_path.extend(path_segment[:-1])
            start_point = end_point
        complete_path.append(waypoints[-1])
        return complete_path

    def find_closest_point(self, lat, lon, csv_file_path):
        closest_point_id = None
        min_distance = float('inf')

        with open(csv_file_path, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                point_id = row['ID']
                point_lat = float(row['Latitude'])
                point_lon = float(row['Longitude'])
                distance = (point_lat - lat)**2 + (point_lon - lon)**2
                if distance < min_distance:
                    min_distance = distance
                    closest_point_id = point_id

        return closest_point_id

    def main_execution(self, blockage_lat, blockage_lon):
        segment_to_lanes = self.read_and_organize_data(self.csv_file_path)
        intra_segment_connections = self.connect_points(segment_to_lanes)
        inter_segment_connections = self.connect_segments()
        all_connections = intra_segment_connections + inter_segment_connections

        graph = self.create_graph(all_connections)

        self.apply_blockage(graph, self.global_blockage)

        shortest_path = self.calculate_shortest_waypoints(graph, self.input_waypoints)

        all_points_data = {}
        with open(self.csv_file_path, 'r') as all_points_file:
            csv_reader = csv.DictReader(all_points_file)
            for row in csv_reader:
                all_points_data[row['ID']] = (row['Latitude'], row['Longitude'])

        waypoints = []
        for point_id in shortest_path:
            if point_id in all_points_data:
                lat, lon = all_points_data[point_id]
                velocity = 3.0
                waypoints.append((lat, lon, velocity))

        traj = Trajectory()
        for waypoint_data in waypoints:
            waypoint_msg = Waypoint()
            waypoint_msg.latitude = float(waypoint_data[0])
            waypoint_msg.longitude = float(waypoint_data[1])
            waypoint_msg.velocity = waypoint_data[2]
            traj.trajectory.append(waypoint_msg)

        self.pub.publish(traj)
        self.get_logger().info(f"Published trajectory: {traj.trajectory}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    # Wait for the required number of subscribers to connect
    required_connections = 1
    node.get_logger().info(f"Waiting for {required_connections} subscribers to connect...")
    while node.pub.get_subscription_count() < required_connections and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)

    node.get_logger().info("Subscriber connected. Publishing trajectory.")
    node.main_execution(node.blockage_lat, node.blockage_lon)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

