from gui_interfaces.srv import Gui                                                         # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('TurtleGUI')
        self.get_logger().info('SERVICE started')
        self.srv = self.create_service(Gui, 'GuiService', self.gui_callback)       # CHANGE

    def gui_callback(self, request, response):
        response.deliveryid = 1
        self.get_logger().info('Got some instructions!')
        self.get_logger().info(f'type: {request.type}')
        self.get_logger().info(f'pickupid: {request.pickupid}')
        self.get_logger().info(f'dropoffid: {request.dropoffid}')
        c = 0

        for i in request.machineidarray:
            self.get_logger().info(f'machineidarray[{c}]: {i}')
            c += 1


        response.isaccepted = True
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()