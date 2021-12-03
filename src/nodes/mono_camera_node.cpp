// #include <ros/ros.h>
// #include <avt_vimba_camera/mono_camera.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "mono_camera_node");

//   ros::NodeHandle nh;
//   ros::NodeHandle nhp("~");

//   avt_vimba_camera::MonoCamera mc(nh, nhp);

//   ros::spin();
//   return 0;
// }


#include "avt_vimba_camera/mono_camera.h"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<avt_vimba_camera::MonoCameraNode>();
  rclcpp::spin(node);

  return 0;
}
