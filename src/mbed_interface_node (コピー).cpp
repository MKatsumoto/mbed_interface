#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "gecko_control/mbed_tx.h"
#include <iostream>

using namespace std;

const int TX_DATA_SIZE = 6;  // size of sending data
const int CONTROL_F = 10; // rate of sending control input [Hz]

uint8_t tx_dataL[TX_DATA_SIZE], tx_dataR[TX_DATA_SIZE]; // Left & Right

void mbedTxCallback(const gecko_control::mbed_tx::ConstPtr& msg);



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mbed_interface_node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("mbed_tx", 1000, mbedTxCallback);

  //========== Serial Communication ==========
  // ! Include "serial/serial.h" at first.
  // To use serial communication,
  // set port, baudrate, and timeout(wait).
  //==========================================
  // string port("/dev/ttyACM0");  // port
  string port1("/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F1B634-if01");
  string port2("/dev/serial/by-id/usb-mbed_Microcontroller_101000000000000000000002F7F24A7E-if01");
  unsigned long baud = 57600; // baudrate
  serial::Serial my_serial1(port1, baud, serial::Timeout::simpleTimeout(5));
  serial::Serial my_serial2(port2, baud, serial::Timeout::simpleTimeout(5));
  std::cout << "Is the serial port1 open?";
  if(my_serial1.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  std::cout << "Is the serial port2 open?";
  if(my_serial2.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;
  // Initialize Buffers for Transmit
  memset(tx_dataL, 0, TX_DATA_SIZE);
  tx_dataL[0] = (uint8_t)255;//(char)0xff; // header
  tx_dataL[1] = (uint8_t)255;//(char)0xff; // header

  memset(tx_dataR, 0, TX_DATA_SIZE);
  tx_dataR[0] = (uint8_t)255;//(char)0xff; // header
  tx_dataR[1] = (uint8_t)255;//(char)0xff; // header


  ros::Rate loop_rate(CONTROL_F);

  cout << "Start" << endl;
  // Main Loop
  while(ros::ok())
  {
    // Send message to mbed
    my_serial1.write(tx_dataL, TX_DATA_SIZE);
    my_serial2.write(tx_dataR, TX_DATA_SIZE);

 //   uint8_t data[1] = {0};
 //  int result = my_serial.read(data, 1);  // return value: the number of received data
 //    cout << result << "," << (int)data[0] << endl;

    ros::spinOnce();
    
    loop_rate.sleep();
  }
  return 0;
}

void mbedTxCallback(const gecko_control::mbed_tx::ConstPtr& msg)
{
  tx_dataL[2] = msg->command;
  tx_dataL[3] = msg->data[0];
  tx_dataL[4] = msg->data[1];
  tx_dataL[5] = 0;
  for(int i = 0; i < 5; i++)
  {
    tx_dataL[5] += tx_dataL[i]; // CheckSum
  }

  tx_dataR[2] = msg->command;
  tx_dataR[3] = msg->data[2];
  tx_dataR[4] = msg->data[3];
  tx_dataR[5] = 0;
  for(int i = 0; i < 5; i++)
  {
    tx_dataR[5] += tx_dataR[i]; // CheckSum
  }
  
  // Debug
  for(int i = 0; i < 5; i++)
  {
    cout << (int)tx_dataL[i] << " ";
  }
  cout << endl;
}





