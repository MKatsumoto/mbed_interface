#ifndef GECKO_MBED_INTERFACE_H_20160212_
#define GECKO_MBED_INTERFACE_H_20160212_

#include <ros/ros.h>
#include "serial/serial.h"
#include "gecko_msgs/MbedTx.h"

#include <string>
#include <vector>
#include "stdint.h"

class MbedInterface : public serial::Serial
{
public:
  MbedInterface(const std::string& id,
                const std::string& port,
                const uint32_t baudrate = 115200,
                const uint16_t timeout = 1000);
  size_t writeTxData();

private:
  const std::string ID_;
  ros::NodeHandle nh_;
  ros::Subscriber mbed_sub_;
  static const int TX_DATA_SIZE_ = 6; // mbed tx data size
  std::vector<uint8_t> tx_data_;

  void mbedTxCallback(const gecko_msgs::MbedTx::ConstPtr& msg);
};

#endif // GECKO_MBED_INTERFACE_H_20160212_
