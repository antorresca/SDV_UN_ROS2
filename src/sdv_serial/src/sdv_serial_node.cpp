#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <libserial/SerialPort.h>  // cabeceras de libserial

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
      } else {
        RCLCPP_WARN(this->get_logger(), "Baud %d no está mapeado, usando BAUD_115200", baud);
        serial_port_.SetBaudRate(BaudRate::BAUD_115200);
      }
      serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(Parity::PARITY_NONE);
      serial_port_.SetStopBits(StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
      RCLCPP_INFO(this->get_logger(), "Puerto serial abierto correctamente");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error abriendo/configurando serial: %s", e.what());
    }

    // Suscripción
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vel2cmd", 10, std::bind(&SdvSerialNode::topic_callback, this, std::placeholders::_1)
    );
  }

  ~SdvSerialNode()
  {
    detener_robot();  // Enviar "m 0 0 0" al destruir el nodo

    if (serial_port_.IsOpen()) {
      try { serial_port_.Close(); } catch(...) {}
    }
  }

  void detener_robot()
  {
    if (serial_port_.IsOpen()) {
      try {
        std::string stop_cmd = "m 0 0 0\r";
        serial_port_.Write(stop_cmd);
        RCLCPP_WARN(this->get_logger(), "Ctrl+C detectado. Enviando comando de parada: '%s'", stop_cmd.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error enviando comando de parada: %s", e.what());
      }
    }
  }

private:
  SerialPort serial_port_;
  std::string port_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string mensaje = msg->data + "\r";
    try {
      serial_port_.Write(mensaje);
      RCLCPP_INFO(this->get_logger(), "Mensaje enviado a Tiva: '%s'", mensaje.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error enviando a Tiva: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdvSerialNode>();

  // Registrar callback que se ejecuta antes de cerrar ROS 2
  rclcpp::on_shutdown([node]() {
    node->detener_robot();
  });

  RCLCPP_INFO(node->get_logger(), "Nodo iniciado. Presiona Ctrl+C para detener.");

  try {
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Excepción en spin: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
