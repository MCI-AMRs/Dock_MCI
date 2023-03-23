import rclpy
from rclpy.node import Node
from gui_interfaces.srv import Gui
import time
import sys

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Gui, 'GuiService')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Gui.Request()

    def send_request(self, a, b, c, d,e):
        self.req.type = a
        self.req.pickupid = b
        self.req.dropoffid = c
        machineid_array = []
        machineid_array.append(d)
        machineid_array.append(e)
        self.req.machineidarray = machineid_array

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()

    while rclpy.ok():
        response = minimal_client.send_request(1,1,1,2,1)
        print("waiting")
        time.sleep(2)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()