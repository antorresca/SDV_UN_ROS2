#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <libserial/SerialPort.h>      // cabeceras de libserial

using namespace std::chrono_literals;
using LibSerial::SerialPort;
using LibSerial::BaudRate;
using LibSerial::CharacterSize;
using LibSerial::Parity;
using LibSerial::StopBits;
using LibSerial::FlowControl;

class SdvSerialNode : public rclcpp::Node
{
public:
  SdvSerialNode()
  : Node("sdv_serial_node"), serial_port_()
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baudrate", 921600);

    port_ = this->get_parameter("port").as_string();
    int baud = this->get_parameter("baudrate").as_int();

    RCLCPP_INFO(this->get_logger(), "Opening serial port %s @ %d", port_.c_str(), baud);

    try {
      serial_port_.Open(port_);
      if (baud == 115200) {
        serial_port_.SetBaudRate(BaudRate::BAUD_115200);
      } else if (baud == 230400) {
        serial_port_.SetBaudRate(BaudRate::BAUD_230400);
      } else if (baud == 921600){
        serial_port_.SetBaudRate(BaudRate::BAUD_921600);
      }else {
        RCLCPP_WARN(this->get_logger(), "Baud %d no está mapeado, usando BAUD_115200", baud);
        serial_port_.SetBaudRate(BaudRate::BAUD_115200);
      }
      serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(Parity::PARITY_NONE);
      serial_port_.SetStopBits(StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
      RCLCPP_INFO(this-> get_logger(), "Puerto serial abierto correctamente");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo/configurando serial: %s", e.what());
    }
    try{
      serial_port_.Write("m 1 +30 +30 \r");
      RCLCPP_INFO(this-> get_logger(), "Comando enviado");
    }catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    }
  }

  ~SdvSerialNode()
  {
    if (serial_port_.IsOpen()) {
      try { serial_port_.Close(); } catch(...) {}
    }
  }

private:
  SerialPort serial_port_;
  std::string port_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdvSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
