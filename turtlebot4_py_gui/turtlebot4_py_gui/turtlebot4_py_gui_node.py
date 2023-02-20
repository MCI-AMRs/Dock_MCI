import rclpy
import threading
from rclpy.node import Node
from gui_interfaces.srv import Gui

from kivymd.app import MDApp
from kivymd.uix.screen import MDScreen
from kivymd.uix.chip.chip import MDChip
from kivymd.toast.kivytoast.kivytoast import toast
from kivy.lang import Builder
from kivy.animation import Animation
from kivy.clock import Clock

class GUI(MDApp):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.machine_array1 = None
        self.machine_array2 = None
        self.machine_array3 = None
        self.active_value = [None,None,None]

        self.node = GUI_Node()
        self.screen=Builder.load_file('/home/chris/ROS2_WS/src/turtlebot4_py_gui/resource/ros_gui.kv')

    def build(self):
        return self.screen
        
    def my_function(self,*args):
        self.machine_array = []

        if self.machine_array1 is not None:
            self.machine_array.append(self.machine_array1)

        if self.machine_array2 is not None:
            self.machine_array.append(self.machine_array2)

        if self.machine_array3 is not None:
            self.machine_array.append(self.machine_array3)            

        response = self.node.send_request(self.machine_array)

        self.node.get_logger().info('Data send to the Turtles!')

        self.node.get_logger().info('Response deliveryid: %d' % response.deliveryid)
        self.node.get_logger().info('Request isaccepted: %r' % response.isaccepted)

        self.machine_array = None

        
    def machine1(self,machine):
        self.machine_array1 = machine
    
    def machine2(self,machine):
        self.machine_array2 = machine

    def machine3(self,machine):
        self.machine_array3 = machine


class MyChip(MDChip):
    icon_check_color = (0, 0, 0, 1)
    text_color = (0, 0, 0, 0.5)
    _no_ripple_effect = True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.bind(active=self.set_chip_bg_color)
        self.bind(active=self.set_chip_text_color)
        

    def set_chip_bg_color(self, instance_chip ,active_value: int):
        ''' Will be called every time the chip is activated/deactivated.
        Sets the background color of the chip. '''

        self.md_bg_color = (
            (0, 0, 0, 0.4)
            if active_value
            else (
                self.theme_cls.bg_darkest
                if self.theme_cls.theme_style == "Light"
                else (
                    self.theme_cls.bg_light
                    if not self.disabled
                    else self.theme_cls.disabled_hint_text_color
                )
            )
        )

    def set_chip_text_color(self, instance_chip, active_value: int):
        Animation(
            color=(0, 0, 0, 1) if active_value else (0, 0, 0, 0.5), d=0.2
        ).start(self.ids.label)


class MyScreen(MDScreen):

    def button_methode(self,*args):
        self.ids.my_button.disabled = True
        self.ids.my_button.md_bg_color= 0,0,0,0.2
        self.ids.my_button.text = 'Sending data to robot...'


        Clock.schedule_once(self.rm_ticks,10)
        toast('Instructions are sent to the robot!',duration=3)
    def rm_ticks(self,*args):
        for i in self.ids.chip_box.children:
            i.active = False
            
        for i in self.ids.chip_box2.children:
            i.active = False

        for i in self.ids.chip_box3.children:
            i.active = False


        self.ids.my_button.disabled = False
        self.ids.my_button.md_bg_color = 1,0,0,1
        self.ids.my_button.text = 'Confirm'


    def remove_all(self,instances):
        for i in self.ids.chip_box.children:
            i.active = False
        
        for i in self.ids.chip_box2.children:
            i.active = False

        for i in self.ids.chip_box3.children:
            i.active = False
  
    def removes_marks_all_chips(self, selected_instance_chip):
        for instance_chip in self.ids.chip_box.children:
            if instance_chip != selected_instance_chip:
               instance_chip.active = False
   
    def removes_marks_all_chips2(self, selected_instance_chip):
        for instance_chip in self.ids.chip_box2.children:
            if instance_chip != selected_instance_chip:
                instance_chip.active = False

    def removes_marks_all_chips3(self, selected_instance_chip): 
        for instance_chip in self.ids.chip_box3.children:
            if instance_chip != selected_instance_chip:
                instance_chip.active = False

class GUI_Node(Node):
    def __init__(self):
        super().__init__("Turtle_GUI")
        self.cli = self.create_client(Gui, 'GuiService')
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self,instructions):
        req = Gui.Request()
        req.type = 1
        req.pickupid = 1
        req.dropoffid = 1
        req.machineidarray = instructions

        future = self.cli.call_async(req)
        # spin until result is ready
        rclpy.spin_until_future_complete(self,future)
        # if there is a result return the result
        if future.result() is not None:
            self.get_logger().info('Running!!')
            return future.result()
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = GUI_Node()

    # Start node in Thread
    t = threading.Thread(target=spin_node, args=(node,))
    t.start

    GUI().run()

    t.join()
    node.destroy_node
    rclpy.shutdown()

def spin_node(node):
    while rclpy.ok():
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()
