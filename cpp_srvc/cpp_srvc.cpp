#include "rclcpp/rclcpp.hpp"
#include "gui_interfaces/srv/gui.hpp"
#include <vector>
#include <iostream>


using std::cout;
using std::endl;

void print(const std::shared_ptr<gui_interfaces::srv::Gui::Request> request,
          std::shared_ptr<gui_interfaces::srv::Gui::Response>      response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\ntype: %i" " pickupid: %i" " dropoffid: %i",
              request->type,request->pickupid, request->dropoffid);

  for(auto i:request->machineidarray){
    i--;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"machineidarray[%i]: %i",i,request->machineidarray[i]);
  }

  response->deliveryid = 1;
  response->isaccepted = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("TurtleGUI");

  rclcpp::Service<gui_interfaces::srv::Gui>::SharedPtr service =
    node->create_service<gui_interfaces::srv::Gui>("GuiService", &print);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to start GUI");

  rclcpp::spin(node);
  rclcpp::shutdown();
}