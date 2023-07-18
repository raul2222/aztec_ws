#include <rclcpp/rclcpp.hpp>
// #include "rclcpp/rclcpp/rclcpp.hpp"
#include <iostream>
#include <cstdlib>
#include <math.h>
#include "synexens_package/synexens_ros_device.h"

void Callback()
{
  // Do something
}
int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  // ======================= test ============================== //
  // std::shared_ptr<rclcpp::Node> nh(new rclcpp::Node("synexens_node"));
  // bool ir_enabled;
  // nh->declare_parameter("ir_enabled", true);
  // nh->get_parameter<bool>("ir_enabled", ir_enabled);
  // printf("ir_enabled:%d", ir_enabled);
  // rclcpp::spin(nh);
  // rclcpp::shutdown();

  // ==================== start ================== //
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr synexesn_node = rclcpp::Node::make_shared("synexens_ros_node", options);
  std::shared_ptr<SynexensROSDevice> synexens_device = std::make_shared<SynexensROSDevice>(synexesn_node);

  // synexesn_node->startCameras();
  synexens_device->startCameras();

  printf("============= ros run end ============= \n");
  rclcpp::spin(synexesn_node);
  
  rclcpp::shutdown();
  // ==================== end ================== //

  // ======================= test 01 start ============================== //
  // test public node
  // rclcpp::NodeOptions options;
  // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);

  // bool ir_enabled;
  // node->declare_parameter("ir_enabled", false);
  // node->get_parameter<bool>("ir_enabled", ir_enabled);

  // printf("ir_enabled:%d \n", ir_enabled);

  // rclcpp::spin(node);
  // rclcpp::shutdown();
  // ======================= test 01 end ============================== //
  return 0;
}
