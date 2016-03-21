//#define DEBUG
#include "mbed_interface/MbedInterface.h"

/*!
 * \brief Open serial port for mbed and set header data.
 * \param id        which mbed (LEFT or RIGHT)
 * \param port      /dev/serial/by-id/...
 * \param baudrate  use same value in mbed (default 115200)
 * \param timeout   [msec]
 * @TODO: Error handling
 */
MbedInterface::MbedInterface(const std::string &id,
                             const std::string &port,
                             const uint32_t baudrate,
                             const uint16_t timeout)
  : ID_(id),
    serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout)),
    tx_data_(TX_DATA_SIZE_, 0)
{
  if(!serial::Serial::isOpen())
  {
    // TODO: Error Handling
  }
  mbed_sub_ = nh_.subscribe("mbed_tx", 100, &MbedInterface::mbedTxCallback, this);
}

/*!
 * \brief Callback function called when receive commands from PC to mbed.
 * \param msg (MbedTx.msg)
 *          uint8 command
 *          uint8[4] data
 *
 *                << LEFT DATA >>               << RIGHT DATA >>
 * command  | data[0]        data[1]         data[2]       data[3]
 * 0x01     | (left MSB)     (left LSB)      (right MSB)   (right LSB)
 * 0x02     | (front left)   (rear left)     (front right) (rear right)
 *
 * @TODO: check
 */
void MbedInterface::mbedTxCallback(const gecko_msgs::MbedTx::ConstPtr &msg)
{
  tx_data_.at(0) = 0xff; // header
  tx_data_.at(1) = 0xff; // header
  tx_data_.at(2) = msg->command;
  if(ID_ == "LEFT") // get data for left
  {
    tx_data_.at(3) = msg->data[0];
    tx_data_.at(4) = msg->data[1];
  }
  else if (ID_ == "RIGHT")  // get data for right
  {
    tx_data_.at(3) = msg->data[2];
    tx_data_.at(4) = msg->data[3];
  }

  tx_data_.at(5) = 0;
  for (int i = 2; i < TX_DATA_SIZE_-1; i++)
  {
    tx_data_.at(5) += tx_data_.at(i); // check sum
  }

#ifdef DEBUG
  for (int i = 0; i < TX_DATA_SIZE_; i++)
  {
    ROS_INFO("tx_data[%d]: %3d \n", i, tx_data_.at(i));
  }
#endif
}

/*!
 * \brief wrapper of serial::Serial::write(tx_data_)
 * \return A size_t representing the number of bytes actually written
 * to the serial port.
 */
void MbedInterface::writeTxData()
{
  serial::Serial::write(tx_data_);
}
