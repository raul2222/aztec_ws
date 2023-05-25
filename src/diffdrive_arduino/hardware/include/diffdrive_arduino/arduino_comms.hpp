#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::istringstream iss(response);
    std::string token_1, token_2;

    iss >> token_1 >> token_2 ;

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
    //val_3 = std::stod(token_3.c_str());
    //val_4 = std::stod(token_4.c_str());

  }
  void set_motor_values(double val_1, double val_2)
  {
    std::stringstream ss;
    ss << "m" << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(double k_p, double k_i, double k_d, double n)
  {
    std::stringstream ss;
    ss << "p" << k_p << "\r";
    send_msg(ss.str());
    ss << "i" << k_i << "\r";
    send_msg(ss.str());
    ss << "d" << k_d << "\r";
    send_msg(ss.str());
    ss << "n" << n << "\r";
    send_msg(ss.str());

  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP