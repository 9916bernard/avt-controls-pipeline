# server_node.py
import rclpy
from rclpy.node import Node
import pandas as pd
import math
from car_ros_msgs.msg import Hdmappoint, Enu
from car_ros_msgs.srv import DatabaseData
from std_msgs.msg import Float64MultiArray

class DatabaseServiceServer(Node):
    def __init__(self):
        super().__init__('get_data')

        # Load CSV data and log column information
        self.sign_df = pd.read_csv('~/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/signs_LTI.csv')
        self.get_logger().info(f"Sign Data Columns: {self.sign_df.columns}")

        # # Load other CSV data
        # self.lane_df = pd.read_csv('/home/sungheon/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/lane.csv')
        # self.edge_df = pd.read_csv('/home/sungheon/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/edges.csv')
        # self.point_df = pd.read_csv('/home/sungheon/ros2_ws/src/CarROS/car_ros2/car_ros_controls/resource/point.csv')

        # Initialize publisher for sign data and subscriber for vehicle position
        self.sign_publisher = self.create_publisher(Hdmappoint, 'sign_pos', 10)
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            'pos_enu',
            self.position_callback,
            10
        )

        # Instance variables for vehicle position in ENU coordinates
        self.vehicle_lat = None
        self.vehicle_lon = None

        # Store unique published signs to avoid duplicates
        self.published_signs = set()

        # Setup service for database queries
        self.srv = self.create_service(DatabaseData, 'query_database', self.query_database_callback)

    def position_callback(self, msg):
        # Update vehicle position and publish nearby signs
        self.vehicle_lat, self.vehicle_lon = msg.data
        self.get_logger().info(f"Updated vehicle position: Latitude={self.vehicle_lat}, Longitude={self.vehicle_lon}")

        # Publish signs nearby based on the updated position
        close_signs = self.find_nearby_signs()
        for sign in close_signs:
            sign_tuple = (sign.type, sign.latitude, sign.longitude)
            # Publish only if the sign is not in the set of published signs
            if sign_tuple not in self.published_signs:
                self.sign_publisher.publish(sign)
                self.get_logger().info(f"Published nearby sign: Type={sign.type}, Lat={sign.latitude}, Lon={sign.longitude}")

                # Add the sign to the set of published signs
                self.published_signs.add(sign_tuple)

    def query_database_callback(self, request, response):
        # Initialize response lists with empty Hdmappoint messages
        temp = Hdmappoint()
        temp_lst = [temp]
        response.edge_list = response.lane_list = response.sign_list = response.point_list = temp_lst

        # Determine request type and populate response accordingly
        request_type = request.type

        # if request_type == 'all':
            # Populate all data types from CSVs
        response.sign_list = self.find_nearby_signs()
        #     response.edge_list = self.query_edge_data()
        #     response.lane_list = self.query_lane_data()
        #     response.point_list = self.query_point_data()
        # elif request_type == 'edge':
        #     response.edge_list = self.query_edge_data()
        # elif request_type == 'lane':
        #     response.lane_list = self.query_lane_data()
        # elif request_type == 'sign':
        #     response.sign_list = self.find_nearby_signs()
        # elif request_type == 'point':
        #     response.point_list = self.query_point_data()

        return response

    def query_sign_data(self):
        # Convert sign data from DataFrame into Hdmappoint messages
        sign_msgs = []
        for _, row in self.sign_df.iterrows():
            sign_msg = Hdmappoint()
            sign_msg.id = 4 #consider everthing as stop sign 
            sign_msg.segment_id = row['segment_id']
            sign_msg.lane_id = row['lane_id']
            # sign_msg.type = row['type']
            sign_msg.latitude = row['ycoord']
            sign_msg.longitude = row['xcoord']
            sign_msgs.append(sign_msg)

        return sign_msgs

    def find_nearby_signs(self):
        # Check if any signs are within a specified radius of the vehicle position
        radius = 10  # Radius in meters within which to check for nearby signs
        sign_msgs = self.query_sign_data()

        nearby_signs = []
        for sign in sign_msgs:
            dist = self.haversine(self.vehicle_lat, self.vehicle_lon, sign.latitude, sign.longitude)
            if dist < radius:
                nearby_signs.append(sign)

        return nearby_signs

    def query_edge_data(self):
        # Populate edge data from CSV into Hdmappoint messages
        edge_msgs = []
        for _, row in self.edge_df.iterrows():
            edge_msg = Hdmappoint()
            edge_msg.id = row['id_']
            edge_msg.segment_id = row['segment_id']
            edge_msg.xs_id = row['xs_id']
            edge_msg.width = row['width']
            edge_msg.type = row['type']
            edge_msg.latitude = row['ycoord']
            edge_msg.longitude = row['xcoord']
            edge_msgs.append(edge_msg)

        return edge_msgs

    def query_lane_data(self):
        # Populate lane data from CSV into Hdmappoint messages
        lane_msgs = []
        for _, row in self.lane_df.iterrows():
            lane_msg = Hdmappoint()
            lane_msg.id = row['id_']
            lane_msg.segment_id = row['segment_id']
            lane_msg.xs_id = row['xs_id']
            lane_msg.lane_id = row['lane_id']
            lane_msg.point_id = row['point_id']
            lane_msg.speed_limit = row['speed_limit']
            lane_msg.stacking = row['stacking']
            lane_msg.surface_type = row['surface_type']
            lane_msg.width = row['width']
            lane_msg.latitude = row['ycoord']
            lane_msg.longitude = row['xcoord']
            lane_msgs.append(lane_msg)

        return lane_msgs

    def query_point_data(self):
        # Populate point data from CSV into Hdmappoint messages
        point_msgs = []
        for _, row in self.point_df.iterrows():
            point_msg = Hdmappoint()
            point_msg.id = row['id_']
            point_msg.segment_id = row['segment_id']
            point_msg.point_id = row['point_id']
            point_msg.latitude = row['ycoord']
            point_msg.longitude = row['xcoord']
            point_msgs.append(point_msg)

        return point_msgs

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        # Haversine formula for distance calculation
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return 6371000 * c  # Distance in meters

def main(args=None):
    rclpy.init(args=args)
    service_server = DatabaseServiceServer()
    rclpy.spin(service_server)
    service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
