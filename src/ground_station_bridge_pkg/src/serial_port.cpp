#include "ground_station_bridge_pkg/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace ground_station_bridge_pkg
{

SerialPort::~SerialPort()
{
  close_port();
}

int SerialPort::baudrate_to_flag(int baudrate)
{
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B115200;
  }
}

bool SerialPort::open_port(const std::string& device, int baudrate)
{
  close_port();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close_port();
    return false;
  }

  cfsetispeed(&tty, baudrate_to_flag(baudrate));
  cfsetospeed(&tty, baudrate_to_flag(baudrate));

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close_port();
    return false;
  }

  return true;
}

void SerialPort::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::write_data(const std::string& data)
{
  if (fd_ < 0) {
    return false;
  }

  const auto n = ::write(fd_, data.data(), data.size());
  return n == static_cast<ssize_t>(data.size());
}

int SerialPort::read_data(char* buffer, int max_len)
{
  if (fd_ < 0 || buffer == nullptr || max_len <= 0) {
    return -1;
  }

  return static_cast<int>(::read(fd_, buffer, static_cast<size_t>(max_len)));
}

}  // namespace ground_station_bridge_pkg
