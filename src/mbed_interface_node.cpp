#include "mbed_interface/MbedInterface.h"

#define DEBUG

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mbed_interface_node");
  MbedInterface left_mbed("LEFT","/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F27A7A-if01", 57600);
  MbedInterface right_mbed("RIGHT","/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F24A7E-if01", 57600);
  ros::Rate loop_rate_Hz(10);

  while(ros::ok())
  {
    left_mbed.writeTxData();
    right_mbed.writeTxData();
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }
  return 0;
}
