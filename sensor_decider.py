import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64
from car_ros_msgs.msg import Hdmappoint, ODLidar
import math


class SensorDecider(Node):
    def __init__(self):
        super().__init__('sensor_decider')

        self.test = True
        # Subscriptions
        if self.test:
            self.subscription = self.create_subscription(
                String,
                'detected_object',
                self.listener_callback,
                10
            )
        else:
            self.subscription = self.create_subscription(
                ODLidar,
                'pos_detection',
                self.listener_callback,
                10
            )

        self.sign_subscription = self.create_subscription(
            Hdmappoint,
            'sign_pos',
            self.sign_callback,
            10
        )
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            'pos_enu',
            self.position_callback,
            10
        )

        # Publishers
        self.stop_point_pub = self.create_publisher(Float64MultiArray, 'WaypointDecide_decision_topic', 10)
        self.velocity_pub = self.create_publisher(Float64, 'Speed_topic', 10)
        self.lane_change_pub = self.create_publisher(Float64MultiArray, "Lane_change_topic", 10)

        # Vehicle ENU position
        self.vehicle_east = None
        self.vehicle_north = None

        # To handle delayed velocity for stop cases
        self.timer = None
        self.velocity_published = False  # Flag to ensure velocity is published only once

    def listener_callback(self, msg):
        if self.test:
            self.get_logger().info(f"Received from test_detector: {msg.data}")
            object_id, object_east, object_north = self.parse_message(msg.data)

            if object_id is None:
                self.get_logger().error("Failed to parse object ID from test_detector.")
                return

            # Handle actions based on object ID
            if object_id == 1:  # Traffic light: Green
                self.handle_green_light()
            elif object_id in [2, 3, 4, 5, 6, 8]:  # Traffic light (Yellow/Red), Stop sign, Pedestrian, Barrel
                self.handle_stop(object_east, object_north, object_id)
            elif object_id == 7:  # Static object
                self.handle_static_obstacle(object_east, object_north)
        else:
            self.get_logger().info(f"Received object detection information: {msg.data}")
            parsed_messages = self.parse_od_lidar(msg)
            for i, parsed_message in enumerate(parsed_messages):
                object_id, object_east, object_north = parsed_message[0], parsed_message[1], parsed_message[2]

                if object_id is None:
                    self.get_logger().error("Failed to parse object ID from detection.")
                    return

                # Handle actions based on object ID
                if object_id == 1:  # Traffic light: Green
                    self.handle_green_light()
                elif object_id in [2, 3, 4, 5, 6, 8]:  # Traffic light (Yellow/Red), Stop sign, Pedestrian, Barrel, Railroad Gate
                    self.handle_stop(object_east, object_north, object_id)
                elif object_id == 7:  # Static object
                    self.handle_static_obstacle(object_east, object_north)
        
    def sign_callback(self, msg):
        # Process sign data from test_get_data
        self.get_logger().info(f"Received sign data: ID={msg.id}, Lat={msg.latitude}, Lon={msg.longitude}")

        # Handle stop based on sign location, converting lat/lon to ENU if necessary
        if self.vehicle_east is not None and self.vehicle_north is not None:
            east, north = self.convert_to_enu(msg.latitude, msg.longitude)
            self.handle_stop(east, north, msg.id)

    def handle_stop(self, east, north, object_id):
        # Publish stop position
        if east is None or north is None:
            self.get_logger().error(f"Invalid ENU coordinates for object ID {object_id}. Skipping stop.")
            return
        
        id_to_name = {
            2: "Yellow light",
            3: "Red light",
            4: "Stop Sign",
            5: "Pedestrian",
            6: "Barrel",
            8: "Railroad Gate"
        }

        stop_msg = Float64MultiArray()
        stop_msg.data = [east, north]
        self.stop_point_pub.publish(stop_msg)
        self.get_logger().info(f"Published stop point at ENU ({east:.2f}, {north:.2f}) for object {id_to_name[object_id]}.")

        # Schedule velocity reset for stop-related objects after 5 seconds
        if object_id in [4, 5, 6, 8]:  # Stop sign, Pedestrian, Barrel
            self.get_logger().info(f"Object {id_to_name[object_id]} detected. Setting velocity to 2 m/s after 5 seconds.")
            self.velocity_published = False  # Reset the flag for the current stop case
            self.timer = self.create_timer(5.0, self.delayed_velocity_callback)

    def delayed_velocity_callback(self):
        if not self.velocity_published:  # Check the flag
            self.publish_velocity(2.0)
            self.velocity_published = True  # Set the flag to prevent repeated publishing

    def handle_green_light(self):
        # Publish speed for green light
        self.publish_velocity(2.0)
        self.get_logger().info("Green light detected. Published speed of 2.0 m/s.")

    def handle_static_obstacle(self, east, north):
        # Publish lane change for static obstacle
        if east is None or north is None:
            self.get_logger().error("Invalid ENU coordinates for lane change. Skipping.")
            return

        lane_change_msg = Float64MultiArray()
        lane_change_msg.data = [east, north]
        self.lane_change_pub.publish(lane_change_msg)
        self.get_logger().info(f"Published lane change request due to static obstacle at ENU ({east:.2f}, {north:.2f}).")

    def publish_velocity(self, velocity):
        # General method to publish velocity
        velocity_msg = Float64()
        velocity_msg.data = velocity
        self.velocity_pub.publish(velocity_msg)
        self.get_logger().info(f"Published velocity: {velocity} m/s.")

    def parse_message(self, data):
        try:
            object_data = data.split(", ")
            object_id = int(object_data[0].split(": ")[1])
            location_data = data.split("Location: ")[1]
            object_east, object_north = map(float, location_data.split(", "))
            return object_id, object_east, object_north
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse message: {e}")
            return None, None, None
    
    def parse_od_lidar(self, msg):
        parsed_messages = []
        for i in range(len(msg.camera.object_id)):
            object_id = msg.camera.object_id[i]
            long_pos = msg.long_pos[i]
            lat_pos = msg.lat_pos[i]
            parsed_messages.append((object_id, long_pos, lat_pos))
        return parsed_messages


    def calculate_distance(self, east1, north1, east2, north2):
        distance = math.sqrt((east2 - east1) ** 2 + (north2 - north1) ** 2)
        return distance

    def position_callback(self, msg):
        try:
            self.vehicle_east, self.vehicle_north = msg.data[:2]
            self.get_logger().info(f"Updated vehicle position: (East: {self.vehicle_east}, North: {self.vehicle_north})")
        except IndexError:
            self.get_logger().error("Invalid vehicle position data.")

    def convert_to_enu(self, lat, lon):
        east = self.vehicle_east + (lon - self.vehicle_north) * 100  # Simplified conversion
        north = self.vehicle_north + (lat - self.vehicle_east) * 100
        return east, north


def main(args=None):
    rclpy.init(args=args)
    node = SensorDecider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
