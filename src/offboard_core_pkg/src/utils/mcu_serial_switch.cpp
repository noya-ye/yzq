#include "offboard_core_pkg/utils/mcu_serial_switch.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace offboard_core_pkg
{

McuSerialSwitch::McuSerialSwitch(
    rclcpp::Logger logger,
    std::string port,
    int baudrate,
    bool enabled)
: logger_(logger),
  port_(std::move(port)),
  baudrate_(baudrate),
  enabled_(enabled)
{}

McuSerialSwitch::~McuSerialSwitch()
{
  // 析构时尽量关闭开关，防止程序退出后单片机还保持开启
  off();
  close();
}

bool McuSerialSwitch::open()
{
  if (!enabled_) {
    RCLCPP_WARN(logger_, "[MCU_SWITCH] disabled");
    return false;
  }

  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ < 0) {
    RCLCPP_ERROR(
      logger_,
      "[MCU_SWITCH] failed to open %s: %s",
      port_.c_str(),
      std::strerror(errno));
    return false;
  }

  if (!configure_port_()) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  RCLCPP_WARN(
    logger_,
    "[MCU_SWITCH] opened %s baud=%d",
    port_.c_str(),
    baudrate_);

  return true;
}

void McuSerialSwitch::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
    RCLCPP_WARN(logger_, "[MCU_SWITCH] closed");
  }
}

bool McuSerialSwitch::is_open() const
{
  return fd_ >= 0;
}

bool McuSerialSwitch::is_on() const
{
  return switch_on_;
}

bool McuSerialSwitch::set(bool on)
{
  if (!enabled_) {
    return false;
  }

  if (fd_ < 0) {
    RCLCPP_WARN(logger_, "[MCU_SWITCH] serial not open, try open");
    if (!open()) {
      return false;
    }
  }

  const char cmd = on ? 'a' : 'b';

  const ssize_t n = ::write(fd_, &cmd, 1);

  if (n == 1) {
    switch_on_ = on;

    RCLCPP_WARN(
      logger_,
      "[MCU_SWITCH] %s, sent '%c'",
      on ? "ON" : "OFF",
      cmd);

    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "[MCU_SWITCH] write '%c' failed: %s",
    cmd,
    std::strerror(errno));

  return false;
}

bool McuSerialSwitch::on()
{
  return set(true);
}

bool McuSerialSwitch::off()
{
  return set(false);
}

bool McuSerialSwitch::configure_port_()
{
  termios tty{};

  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(
      logger_,
      "[MCU_SWITCH] tcgetattr failed: %s",
      std::strerror(errno));
    return false;
  }

  speed_t speed = B115200;

  switch (baudrate_) {
    case 9600:
      speed = B9600;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    default:
      RCLCPP_WARN(
        logger_,
        "[MCU_SWITCH] unsupported baudrate=%d, fallback to 115200",
        baudrate_);
      speed = B115200;
      break;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // 不使用硬件流控
  tty.c_cflag &= ~CRTSCTS;

  // 本地连接，启用接收
  tty.c_cflag |= CREAD | CLOCAL;

  // 原始模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      logger_,
      "[MCU_SWITCH] tcsetattr failed: %s",
      std::strerror(errno));
    return false;
  }

  return true;
}

}  // namespace offboard_core_pkg