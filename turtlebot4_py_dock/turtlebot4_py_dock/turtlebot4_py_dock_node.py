import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator


class TurtleDockingNode(Node):
    

    def __init__(self):
        super().__init__("Turtle_Dock")
        self.turtle_dock = TurtleBot4Navigator()
        self.get_logger().info("Dock/Undock started")
        self.turtle_docking()

    def turtle_docking(self):
        if not self.turtle_dock.getDockedStatus():
            self.get_logger().info("Docking the Turtle!")
            self.turtle_dock.dock()
        else:
            self.get_logger().info("Undocking the Turtle!")
            self.turtle_dock.undock()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDockingNode()
    rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


