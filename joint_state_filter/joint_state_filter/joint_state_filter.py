#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')
        # Subscriber to 'isaac_joint_states'
        self.subscription = self.create_subscription(
            JointState,
            'isaac_joint_states',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Publisher to 'filtered_joint_states'
        self.publisher = self.create_publisher(JointState, 'filtered_joint_states', 10)

        # Define joints to ignore
        self.joints_to_ignore = ['RevoluteJoint', 'RevoluteJoint2']

        self.get_logger().info('JointStateFilter node has been started.')

    def listener_callback(self, msg):
        # Create a new JointState message
        filtered_msg = JointState()
        filtered_msg.header = msg.header

        # Filter out the mimic joints
        filtered_msg.name = []
        filtered_msg.position = []
        filtered_msg.velocity = []
        filtered_msg.effort = []

        for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort):
            if name not in self.joints_to_ignore:
                filtered_msg.name.append(name)
                filtered_msg.position.append(position)
                filtered_msg.velocity.append(velocity)
                filtered_msg.effort.append(effort)

        # Publish the filtered JointState message
        self.publisher.publish(filtered_msg)
        self.get_logger().debug(f'Published filtered_joint_states: {filtered_msg.name}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
