import rclpy

from rclpy.node import Node

from gui_interfaces.srv import Gui


class GUI_Node(Node):
    def __init__(self):
        super().__init__("Turtle_GUI")
        #self.publisher_ = self.create_publisher(Int16MultiArray,'/button_gui',1)
        self.cli = self.create_client(Gui, 'GuiService')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ready!')    
        self.req = Gui.Request()

    def send_request(self,a,b,c,d):
        self.req.type = a
        self.req.pickupid = b
        self.req.dropoffid = c
        self.req.machineidarray = d
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = GUI_Node()
    a = 1
    b = 1
    c = 2
    d = [1,2,3]
    response = node.send_request(int(a),int(b),int(c),d)
    node.get_logger().info('Response: %d\t %r\t' % (response.deliveryid,response.isaccepted))

    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
