#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class WaypointManager(Node):
    
    def __init__(self, waypoints):

        self.publisher_dic = {}
        self.waypoint_dic = {}

        super().__init__('waypoint_manager')
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        for index, waypoint in enumerate(waypoints):

            name = 'marker_{0}'.format(index)
            
            self.publisher_dic[name] = self.create_publisher(Marker, name, 10)
            self.waypoint_dic[name] = self.create_marker_msg(index, waypoint[0], waypoint[1])
            self.get_logger().info("Marker {0} created.".format(name))


    def timer_callback(self):
        for name in self.waypoint_dic:
            self.publisher_dic[name].publish(self.waypoint_dic[name])

    def create_marker_msg(self, id, x, y):
        
        name = 'marker_{0}'.format(id)

        marker = Marker()
        marker.header.frame_id = '/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.CUBE
        marker.id = id
        marker.action = marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.3

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        
        return marker

def main(args=None):
    rclpy.init(args=args)

    waypoints = []

    w1 = [1.0, 0.0]
    w2 = [1.0, 1.0]
    w3 = [0.0, 1.0]
    w4 = [0.0, 0.0]

    waypoints.append(w1)
    waypoints.append(w2)
    waypoints.append(w3)
    waypoints.append(w4)

    manager = WaypointManager(waypoints=waypoints)
    
    rclpy.spin(manager)

    manager.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()